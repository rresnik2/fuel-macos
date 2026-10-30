#include "vehicle_physics.hpp"

#include "core/vehicle_params.hpp"

// clang-format off
#include <Jolt/Jolt.h>
#include <Jolt/RegisterTypes.h>
#include <Jolt/Core/Factory.h>
#include <Jolt/Core/JobSystemThreadPool.h>
#include <Jolt/Core/TempAllocator.h>
#include <Jolt/Physics/Body/BodyCreationSettings.h>
#include <Jolt/Physics/Collision/BroadPhase/BroadPhaseLayerInterfaceTable.h>
#include <Jolt/Physics/Collision/BroadPhase/ObjectVsBroadPhaseLayerFilterTable.h>
#include <Jolt/Physics/Collision/CastResult.h>
#include <Jolt/Physics/Collision/ContactListener.h>
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/ObjectLayerPairFilterTable.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/HeightFieldShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Collision/Shape/ScaledShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Vehicle/VehicleCollisionTester.h>
#include <Jolt/Physics/Vehicle/VehicleConstraint.h>
#include <Jolt/Physics/Vehicle/MotorcycleController.h>
#include <Jolt/Physics/Vehicle/WheeledVehicleController.h>
// clang-format on

#include <algorithm>
#include <cstdlib>
#include <cmath>
#include <future>
#include <mutex>
#include <stdexcept>
#include <map>
#include <thread>
#include <unordered_map>

namespace fuel {

using namespace JPH;

namespace {

constexpr ObjectLayer kStatic = 0, kMoving = 1, kProp = 2, kWheel = 3;  // kWheel: tyre casts, which see only static ground
constexpr float kStep = 1.0f / 60.0f;  // FUEL's own physics rate
constexpr float kGravity = 9.81f;
constexpr uint32_t kPatchSamples = 512;  // terrain collision patch, samples per side
constexpr float kPatchSpacing = 4.0f;    // metres (the source grid is 30 m; B-spline resampled)
constexpr double kPatchReach = 600.0;    // a vehicle further than this from every patch centre gets a new patch
constexpr double kPatchKeep = 900.0;     // a patch stays while some vehicle is within this of its centre

void initJoltOnce() {
    static std::once_flag once;
    std::call_once(once, [] {
        RegisterDefaultAllocator();
        Factory::sInstance = new Factory();
        RegisterTypes();
    });
}

Vec3 toVec3(const std::array<float, 3>& a) { return Vec3(a[0], a[1], a[2]); }

SurfaceQuery& defaultSurfaceQuery() {
    static SurfaceQuery q;
    return q;
}

// Notes how each vehicle's chassis body touches the world (terrain, objects and other
// vehicles): the fastest sliding speed along a contact and the fastest normal speed of a
// new contact. Wheels do not count (they are ray casts, not contacts).
class ChassisContacts : public ContactListener {
public:
    void track(BodyID car) {
        std::lock_guard<std::mutex> lock(mutex_);
        values_[car.GetIndexAndSequenceNumber()] = {0, 0};
    }
    void untrack(BodyID car) {
        std::lock_guard<std::mutex> lock(mutex_);
        values_.erase(car.GetIndexAndSequenceNumber());
    }
    void reset() {
        std::lock_guard<std::mutex> lock(mutex_);
        for (auto& [id, v] : values_) v = {0, 0};
    }
    void read(BodyID car, float& scrape, float& impact) const {
        std::lock_guard<std::mutex> lock(mutex_);
        const auto it = values_.find(car.GetIndexAndSequenceNumber());
        scrape = it == values_.end() ? 0.0f : it->second.first;
        impact = it == values_.end() ? 0.0f : it->second.second;
    }
    void OnContactAdded(const Body& a, const Body& b, const ContactManifold& m, ContactSettings&) override { note(a, b, m, true); }
    void OnContactPersisted(const Body& a, const Body& b, const ContactManifold& m, ContactSettings&) override { note(a, b, m, false); }

private:
    void note(const Body& a, const Body& b, const ContactManifold& m, bool added) {
        std::lock_guard<std::mutex> lock(mutex_);
        auto ia = values_.find(a.GetID().GetIndexAndSequenceNumber());
        auto ib = values_.find(b.GetID().GetIndexAndSequenceNumber());
        if (ia == values_.end() && ib == values_.end()) return;
        const RVec3 p = m.GetWorldSpaceContactPointOn1(0);
        const Vec3 v = a.GetPointVelocity(p) - b.GetPointVelocity(p);
        const float vn = v.Dot(m.mWorldSpaceNormal);
        const float vt = (v - m.mWorldSpaceNormal * vn).Length();
        for (auto it : {ia, ib}) {
            if (it == values_.end()) continue;
            it->second.first = std::max(it->second.first, vt);
            if (added) it->second.second = std::max(it->second.second, std::fabs(vn));
        }
    }
    mutable std::mutex mutex_;
    std::unordered_map<uint32_t, std::pair<float, float>> values_;  // body -> (scrape, impact)
};

}  // namespace

// ---- setup from game data ------------------------------------------------------


VehicleSetup makeVehicleSetup(const DpcFile& dpc, const std::string& variant, const std::vector<float>& p,
                              const std::vector<std::array<float, 3>>& hubs) {
    auto param = [&](int i) { return size_t(i) < p.size() ? p[size_t(i)] : 0.0f; };
    VehicleSetup s;
    s.name = variant;
    s.mass = param(vp::Mass);
    if (!(s.mass > 10 && s.mass < 50000)) throw std::runtime_error(variant + ": implausible mass");

    const VehicleProxy proxy = findVehicleProxy(dpc, variant);
    // Wheel spheres by name: front-left, front-right, rear-left, rear-right.
    const uint32_t order[4] = {0xD8B7A397, 0xA828D8ED, 0xFC33776C, 0x8CAC0C16};
    for (int k = 0; k < 4; ++k) {
        for (const ProxySphere& sp : proxy.spheres) {
            if (sp.name != order[k]) continue;
            VehicleSetup::Wheel w;
            w.hub = sp.centre;
            float best = 1e9f;
            for (const auto& h : hubs) {
                const float d = std::hypot(h[0] - sp.centre[0], h[1] - sp.centre[1], h[2] - sp.centre[2]);
                if (d < best && d < sp.radius * 1.5f) best = d, w.hub = h;
            }
            w.radius = sp.radius;
            w.width = std::clamp(sp.radius * 0.8f, 0.15f, 0.9f);
            w.front = k < 2;
            s.wheels.push_back(w);
        }
    }
    if (s.wheels.size() == 2 && s.wheels[0].front && !s.wheels[1].front) {
        s.bike = true;  // front-left and rear-left spheres only: a motorbike
        for (auto& w : s.wheels) w.width = std::min(w.width, w.radius * 0.45f);
    } else if (s.wheels.size() != 4) {
        throw std::runtime_error(variant + ": needs two or four wheel spheres");
    }

    // The proxy boxes include the wheels (their bottoms reach the ground);
    // the collision chassis keeps the part above the axles and between the wheels.
    float hubY = 0, innerX = 1e9f;
    for (const VehicleSetup::Wheel& w : s.wheels) {
        hubY += w.hub[1] / float(s.wheels.size());
        innerX = std::min(innerX, std::fabs(w.hub[0]) - w.width * 0.5f);
    }
    if (s.bike) innerX = 0.25f;  // a bike's body is narrow, not boxed in by wheels
    for (const ProxyBox& b : proxy.boxes) {
        const float top = b.centre[1] + b.halfExtents[1];
        const float bottom = std::max(b.centre[1] - b.halfExtents[1], hubY);
        if (top - bottom < 0.1f) continue;
        VehicleSetup::Box box;
        box.centre = {b.centre[0], (top + bottom) / 2, b.centre[2]};
        box.halfExtents = {std::min(b.halfExtents[0], std::max(0.15f, innerX)), (top - bottom) / 2, b.halfExtents[2]};
        s.chassis.push_back(box);
    }
    if (s.chassis.empty()) throw std::runtime_error(variant + ": physics proxy has no chassis box");

    s.frontDriven = param(vp::FrontDriven) != 0;
    s.rearDriven = param(vp::RearDriven) != 0;
    if (!s.frontDriven && !s.rearDriven) s.rearDriven = true;
    s.maxRpm = param(vp::MaxRpm);
    s.idleRpm = param(vp::IdleRpm);
    if (!(s.maxRpm > s.idleRpm && s.idleRpm > 0)) throw std::runtime_error(variant + ": implausible engine rpm");
    for (int i = 0; i < 11; ++i) s.torqueCurve[size_t(i)] = param(vp::TorqueCurve + i);
    const int gears = int(param(vp::GearCount));
    for (int g = 0; g < gears && g < 7; ++g) s.gears.push_back(param(vp::FirstGear + g));
    if (s.gears.empty() || s.gears[0] <= 0) throw std::runtime_error(variant + ": no gears");
    s.reverseGear = -std::fabs(param(vp::ReverseGear));
    s.finalDrive = param(vp::FinalDrive);
    s.topSpeedKmh = param(vp::TopSpeedKmh);
    // FUEL's torque scale (P327) is in unknown units; size the engine for a
    // lively 0.9 g launch in first gear instead.
    const float r = s.wheels.back().radius;
    s.peakTorque = 0.9f * kGravity * s.mass * r / (s.finalDrive * s.gears[0]);
    return s;
}

// ---- simulation ----------------------------------------------------------------

struct DriveWorld::Impl {
    const Heightfield& terrain;
    TempAllocatorImpl temp{16 * 1024 * 1024};
    JobSystemThreadPool jobs{cMaxPhysicsJobs, cMaxPhysicsBarriers, int(std::max(1u, std::thread::hardware_concurrency() - 1))};
    ObjectLayerPairFilterTable layerPairs{4};
    BroadPhaseLayerInterfaceTable broadPhaseLayers{4, 2};
    std::unique_ptr<ObjectVsBroadPhaseLayerFilterTable> objectVsBroadPhase;
    PhysicsSystem physics;

    // Terrain patches: heightfield bodies ~2 km across. Vehicles near each other share
    // one; a vehicle that strays kPatchReach from every patch centre gets its own, and
    // patches no vehicle is near are dropped. They overlap where they meet (same heights).
    struct Patch {
        BodyID body;
        double x = 0, z = 0;  // centre
    };
    std::vector<Patch> patches;
    // Patches resampled on a worker thread ahead of need (a rebuild took ~12 ms).
    struct PendingPatch {
        std::future<RefConst<Shape>> shape;
        double x0 = 0, z0 = 0;
    };
    std::vector<PendingPatch> pendingPatches;

    std::unordered_map<uint32_t, RefConst<Shape>> staticShapes;
    std::map<std::pair<uint32_t, std::array<double, 16>>, BodyID> staticBodies;

    // Dynamic props: shapes by key (with their mass), bodies by prop id.
    struct PropShape {
        RefConst<Shape> shape;
        float mass = 5;
    };
    std::unordered_map<uint32_t, PropShape> propShapes;
    struct Prop {
        BodyID body;
        Vec3 scale = Vec3::sReplicate(1.0f);
        RVec3 lastPos;
        Quat lastRot;
        RVec3 startPos;
    };
    std::unordered_map<uint32_t, Prop> props;

    // One simulated vehicle (index 0 is the player). Heap-allocated: the tyre callback keeps a pointer.
    struct Vehicle {
        VehicleSetup setup;
        BodyID car;
        Ref<VehicleConstraint> constraint;
        Ref<VehicleCollisionTester> tester;
        DriveInput input;
        float upsideDownTime = 0;
        int lastWheelsOnGround = 4;
        // Surfaces: type under each wheel, its smoothed grip scale (read by the tyre callback)
        // and the suspension length last step (for the compression speed).
        std::vector<int> wheelSurface;
        std::vector<float> wheelGrip, wheelCompression, prevSuspension;
    };
    std::vector<std::unique_ptr<Vehicle>> vehicles;
    float accumulator = 0;
    SurfaceQuery surfaceQuery = defaultSurfaceQuery();
    ChassisContacts contacts;

    explicit Impl(const Heightfield& hf) : terrain(hf) {
        layerPairs.EnableCollision(kMoving, kStatic);
        layerPairs.EnableCollision(kMoving, kMoving);
        layerPairs.EnableCollision(kProp, kStatic);
        layerPairs.EnableCollision(kProp, kMoving);
        layerPairs.EnableCollision(kProp, kProp);
        layerPairs.EnableCollision(kWheel, kStatic);
        broadPhaseLayers.MapObjectToBroadPhaseLayer(kStatic, BroadPhaseLayer(0));
        broadPhaseLayers.MapObjectToBroadPhaseLayer(kMoving, BroadPhaseLayer(1));
        broadPhaseLayers.MapObjectToBroadPhaseLayer(kProp, BroadPhaseLayer(1));
        broadPhaseLayers.MapObjectToBroadPhaseLayer(kWheel, BroadPhaseLayer(1));
        objectVsBroadPhase = std::make_unique<ObjectVsBroadPhaseLayerFilterTable>(broadPhaseLayers, 2, layerPairs, 4);
        physics.Init(8192, 0, 8192, 8192, broadPhaseLayers, *objectVsBroadPhase, layerPairs);
        physics.SetGravity(Vec3(0, -kGravity, 0));
        physics.SetContactListener(&contacts);
    }

    BodyInterface& bodies() { return physics.GetBodyInterface(); }

    // A heightfield body covering ~2 km around (x, z), resampled from the
    // world map with the same B-spline the renderer uses.
    static constexpr double kPatchHalf = 0.5 * kPatchSpacing * (kPatchSamples - 1);
    static double patchOrigin(double v) { return std::round((v - kPatchHalf) / kPatchSpacing) * kPatchSpacing; }

    // The heightfield shape of the patch with corner (x0, z0); thread-safe.
    static RefConst<Shape> makePatchShape(const Heightfield& terrain, double x0, double z0) {
        std::vector<float> samples(size_t(kPatchSamples) * kPatchSamples);
        for (uint32_t r = 0; r < kPatchSamples; ++r)
            for (uint32_t c = 0; c < kPatchSamples; ++c)
                samples[size_t(r) * kPatchSamples + c] = terrain.smoothHeightAt(x0 + c * kPatchSpacing, z0 + r * kPatchSpacing);
        HeightFieldShapeSettings hs(samples.data(), Vec3::sZero(), Vec3(kPatchSpacing, 1.0f, kPatchSpacing), kPatchSamples);
        auto result = hs.Create();
        if (result.HasError()) throw std::runtime_error(std::string("terrain patch: ") + result.GetError().c_str());
        return result.Get();
    }

    void addPatchBody(RefConst<Shape> shape, double x0, double z0) {
        BodyCreationSettings bs(shape, RVec3(x0, 0, z0), Quat::sIdentity(), EMotionType::Static, kStatic);
        bs.mFriction = 0.9f;
        patches.push_back({bodies().CreateAndAddBody(bs, EActivation::DontActivate), x0 + kPatchHalf, z0 + kPatchHalf});
    }

    void buildTerrainPatch(double x, double z) {
        const double x0 = patchOrigin(x), z0 = patchOrigin(z);
        addPatchBody(makePatchShape(terrain, x0, z0), x0, z0);
    }

    bool onPatch(double x, double z, double reach) const {
        for (const Patch& p : patches)
            if (std::hypot(x - p.x, z - p.z) <= reach) return true;
        return false;
    }

    bool patchPending(double x, double z, double reach) const {
        for (const PendingPatch& p : pendingPatches)
            if (std::hypot(x - (p.x0 + kPatchHalf), z - (p.z0 + kPatchHalf)) <= reach) return true;
        return false;
    }

    // Every vehicle within kPatchReach of a patch centre; patches far from all vehicles go.
    // A vehicle heading out of its patch gets the next one prepared in the background.
    void updatePatches() {
        for (auto it = pendingPatches.begin(); it != pendingPatches.end();) {
            if (it->shape.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
                ++it;
                continue;
            }
            addPatchBody(it->shape.get(), it->x0, it->z0);
            it = pendingPatches.erase(it);
        }
        constexpr double kPrefetch = kPatchReach * 0.6;
        for (const auto& v : vehicles) {
            if (v->car.IsInvalid()) continue;
            const RVec3 pos = bodies().GetPosition(v->car);
            const double x = pos.GetX(), z = pos.GetZ();
            if (!onPatch(x, z, kPatchReach)) {
                // Out of reach and nothing usable ready: wait for a pending one nearby, else build now.
                bool waited = false;
                for (auto it = pendingPatches.begin(); it != pendingPatches.end(); ++it)
                    if (std::hypot(x - (it->x0 + kPatchHalf), z - (it->z0 + kPatchHalf)) <= kPatchReach) {
                        addPatchBody(it->shape.get(), it->x0, it->z0);
                        pendingPatches.erase(it);
                        waited = true;
                        break;
                    }
                if (!waited) buildTerrainPatch(x, z);
            } else if (!onPatch(x, z, kPrefetch) && !patchPending(x, z, kPrefetch)) {
                const double x0 = patchOrigin(x), z0 = patchOrigin(z);
                pendingPatches.push_back({std::async(std::launch::async, [&t = terrain, x0, z0] { return makePatchShape(t, x0, z0); }), x0, z0});
            }
        }
        for (auto it = patches.begin(); it != patches.end();) {
            bool used = false;
            for (const auto& v : vehicles) {
                if (v->car.IsInvalid()) continue;
                const RVec3 pos = bodies().GetPosition(v->car);
                used = used || std::hypot(double(pos.GetX()) - it->x, double(pos.GetZ()) - it->z) <= kPatchKeep;
            }
            if (used) {
                ++it;
                continue;
            }
            bodies().RemoveBody(it->body);
            bodies().DestroyBody(it->body);
            it = patches.erase(it);
        }
    }

    void createVehicle(Vehicle& v, RVec3 position, Quat rotation) {
        const VehicleSetup& s = v.setup;
        StaticCompoundShapeSettings compound;
        float boxTop = 0;
        for (const VehicleSetup::Box& b : s.chassis) {
            const Vec3 half = Vec3::sMax(toVec3(b.halfExtents), Vec3::sReplicate(0.06f));
            compound.AddShape(toVec3(b.centre), Quat::sIdentity(), new BoxShape(half, 0.05f));
            boxTop = std::max(boxTop, b.centre[1]);
        }
        // Arcade stability: put the centre of mass down near the axles.
        const float hubY = s.wheels[0].hub[1];
        auto compoundShape = compound.Create().Get();
        const float comY = compoundShape->GetCenterOfMass().GetY();
        auto shape = OffsetCenterOfMassShapeSettings(Vec3(0, (hubY + 0.1f) - comY, 0), compoundShape).Create().Get();

        BodyCreationSettings bs(shape, position, rotation, EMotionType::Dynamic, kMoving);
        bs.mOverrideMassProperties = EOverrideMassProperties::CalculateInertia;
        bs.mMassPropertiesOverride.mMass = s.mass;
        bs.mAllowSleeping = false;
        bs.mAngularDamping = 0.3f;
        bs.mMotionQuality = EMotionQuality::LinearCast;  // no tunnelling into objects at speed
        v.car = bodies().CreateAndAddBody(bs, EActivation::Activate);

        VehicleConstraintSettings vs;
        vs.mUp = Vec3::sAxisY();
        vs.mForward = Vec3::sAxisZ();
        vs.mMaxPitchRollAngle = JPH_PI;  // FUEL lets vehicles roll over
        const bool heavy = s.mass > 2500;
        for (const VehicleSetup::Wheel& w : s.wheels) {
            auto* ws = new WheelSettingsWV;
            const float travel = std::clamp(w.radius * 0.9f, 0.18f, 0.9f);
            // The suspension hangs from above the hub so the wheel rests near
            // its authored position.
            // Bikes: a raked front fork (25 degrees of caster) steering about its own axis.
            const Vec3 dir = s.bike && w.front ? Vec3(0, -1, std::tan(0.44f)).Normalized() : Vec3(0, -1, 0);
            ws->mPosition = toVec3(w.hub) - dir * (0.6f * travel);
            ws->mSuspensionDirection = dir;
            ws->mSteeringAxis = -dir;
            ws->mSuspensionMinLength = 0.02f;
            ws->mSuspensionMaxLength = travel;
            ws->mSuspensionSpring.mFrequency = heavy ? 1.1f : 1.6f;
            ws->mSuspensionSpring.mDamping = 0.55f;
            ws->mRadius = w.radius;
            ws->mWidth = w.width;
            // Jolt's default inertia suits a 20 kg car wheel; size it for this one (solid disc).
            const float wheelMass = std::clamp(s.mass * 0.02f, 8.0f, 200.0f);
            ws->mInertia = 0.5f * wheelMass * w.radius * w.radius;
            ws->mMaxSteerAngle = w.front ? (s.bike ? 0.45f : 0.6f) : 0.0f;
            const float brake = s.mass * kGravity * w.radius * 0.6f;
            ws->mMaxBrakeTorque = brake;
            ws->mMaxHandBrakeTorque = w.front ? 0.0f : brake * 2.0f;
            // Off-road tyres: forgiving peak, gentle fall-off when sliding.
            ws->mLongitudinalFriction.Clear();
            ws->mLongitudinalFriction.AddPoint(0.0f, 0.0f);
            ws->mLongitudinalFriction.AddPoint(0.08f, 1.25f);
            ws->mLongitudinalFriction.AddPoint(0.3f, 1.05f);
            ws->mLongitudinalFriction.AddPoint(1.0f, 0.9f);
            ws->mLateralFriction.Clear();
            ws->mLateralFriction.AddPoint(0.0f, 0.0f);
            ws->mLateralFriction.AddPoint(4.0f, 1.2f);
            ws->mLateralFriction.AddPoint(20.0f, 1.0f);
            ws->mLateralFriction.AddPoint(90.0f, 0.85f);
            vs.mWheels.push_back(ws);
        }
        if (!s.bike) vs.mAntiRollBars.resize(2);
        if (!s.bike) {
            vs.mAntiRollBars[0].mLeftWheel = 0;
            vs.mAntiRollBars[0].mRightWheel = 1;
            vs.mAntiRollBars[1].mLeftWheel = 2;
            vs.mAntiRollBars[1].mRightWheel = 3;
        }
        for (auto& bar : vs.mAntiRollBars) bar.mStiffness = s.mass * 0.7f;  // Jolt's reference: ~1000 N/m for a 1500 kg car

        // Bikes use Jolt's motorcycle controller, which leans the bike into turns.
        auto* cs = s.bike ? new MotorcycleControllerSettings : new WheeledVehicleControllerSettings;
        cs->mEngine.mMaxTorque = s.peakTorque;
        cs->mEngine.mMinRPM = s.idleRpm;
        cs->mEngine.mMaxRPM = s.maxRpm;
        // Jolt's defaults suit a car engine; scale friction and inertia to this one.
        cs->mEngine.mAngularDamping = s.peakTorque * 0.0004f;
        cs->mEngine.mInertia = std::max(0.02f, s.peakTorque * 0.002f);
        cs->mEngine.mNormalizedTorque.Clear();
        for (int i = 0; i < 11; ++i)  // FUEL cuts torque to 0 at the limiter; keep a little so Jolt can hold redline
            cs->mEngine.mNormalizedTorque.AddPoint(i * 0.1f, std::max(s.torqueCurve[size_t(i)], i == 10 ? 0.3f : 0.0f));
        cs->mTransmission.mMode = ETransmissionMode::Auto;
        cs->mTransmission.mClutchStrength = std::max(10.0f, s.peakTorque * 0.2f);  // grip in proportion to the engine
        cs->mTransmission.mGearRatios.clear();
        for (float g : s.gears) cs->mTransmission.mGearRatios.push_back(g);
        cs->mTransmission.mReverseGearRatios = {s.reverseGear};
        cs->mTransmission.mShiftUpRPM = s.maxRpm * 0.85f;
        cs->mTransmission.mShiftDownRPM = s.idleRpm + (s.maxRpm - s.idleRpm) * 0.3f;
        auto addDiff = [&](uint32_t left, uint32_t right, float share) {
            VehicleDifferentialSettings d;
            d.mLeftWheel = int(left);
            d.mRightWheel = int(right);
            d.mDifferentialRatio = s.finalDrive;
            d.mEngineTorqueRatio = share;
            cs->mDifferentials.push_back(d);
        };
        if (s.bike) {
            addDiff(uint32_t(-1), 1, 1.0f);  // rear wheel only (no left wheel)
        } else {
            const float share = s.frontDriven && s.rearDriven ? 0.5f : 1.0f;
            if (s.frontDriven) addDiff(0, 1, share);
            if (s.rearDriven) addDiff(2, 3, share);
        }
        vs.mController = cs;

        v.constraint = new VehicleConstraint(*physics.GetBodyLockInterface().TryGetBody(v.car), vs);
        v.tester = new VehicleCollisionTesterCastCylinder(kWheel, s.bike ? 1.0f : 0.1f);  // bikes: rounded tyres
        v.constraint->SetVehicleCollisionTester(v.tester);
        // Per-wheel tyre grip: the surface under each wheel scales both friction impulses.
        v.wheelSurface.assign(s.wheels.size(), -1);
        v.wheelGrip.assign(s.wheels.size(), 1.0f);
        v.wheelCompression.assign(s.wheels.size(), 0.0f);
        v.prevSuspension.assign(s.wheels.size(), -1.0f);
        contacts.track(v.car);
        Vehicle* vp = &v;
        static_cast<WheeledVehicleController*>(v.constraint->GetController())
            ->SetTireMaxImpulseCallback([vp](uint wheel, float& longitudinal, float& lateral, float suspensionImpulse, float longitudinalFriction,
                                             float lateralFriction, float, float, float) {
                const float grip = wheel < vp->wheelGrip.size() ? vp->wheelGrip[wheel] : 1.0f;
                longitudinal = longitudinalFriction * suspensionImpulse * grip;
                lateral = lateralFriction * suspensionImpulse * grip;
            });
        physics.AddConstraint(v.constraint);
        physics.AddStepListener(v.constraint);
    }

    void destroyVehicle(Vehicle& v) {
        if (v.car.IsInvalid()) return;
        physics.RemoveStepListener(v.constraint);
        physics.RemoveConstraint(v.constraint);
        v.constraint = nullptr;
        contacts.untrack(v.car);
        bodies().RemoveBody(v.car);
        bodies().DestroyBody(v.car);
        v.car = BodyID();
    }

    // Places a vehicle on its wheels at (x, z) with the given heading.
    void place(Vehicle& v, double x, double z, float yaw) {
        if (!onPatch(x, z, kPatchReach)) buildTerrainPatch(x, z);
        const double ground = terrain.smoothHeightAt(x, z);
        destroyVehicle(v);
        createVehicle(v, RVec3(x, ground + 0.4, z), Quat::sRotation(Vec3::sAxisY(), yaw));
        v.upsideDownTime = 0;
        v.lastWheelsOnGround = int(v.setup.wheels.size());
    }

    static int wheelsOnGround(const Vehicle& v) {
        int n = 0;
        for (const Wheel* w : v.constraint->GetWheels()) n += w->HasContact() ? 1 : 0;
        return n;
    }

    // Looks up the surface under each wheel's contact point; the tyre grip follows it
    // (smoothed over a few steps) and soft ground adds rolling resistance.
    void updateSurfaces(Vehicle& v, const Vec3& vel) {
        const auto& wheels = v.constraint->GetWheels();
        float crr = 0;
        for (size_t i = 0; i < wheels.size() && i < v.wheelSurface.size(); ++i) {
            const Wheel* w = wheels[i];
            const float len = w->GetSuspensionLength();
            v.wheelCompression[i] = v.prevSuspension[i] >= 0 ? (v.prevSuspension[i] - len) / kStep : 0.0f;
            v.prevSuspension[i] = len;
            int type = -1;
            if (w->HasContact()) {
                const RVec3 c = w->GetContactPosition();
                type = surfaceQuery ? surfaceQuery(double(c.GetX()), double(c.GetZ())) : -1;
                crr += rollingResistance(gripCategory(type));
                v.wheelGrip[i] += (surfaceGrip(type) - v.wheelGrip[i]) * 0.25f;
            }
            v.wheelSurface[i] = w->HasContact() ? std::max(type, -1) : -1;
        }
        const float horizontal = std::sqrt(vel.GetX() * vel.GetX() + vel.GetZ() * vel.GetZ());
        if (horizontal > 0.3f && !wheels.empty()) {
            // crr is summed over wheels on the ground: mean over all wheels = share of the weight carried by them
            const float drag = v.setup.mass * kGravity * crr / float(wheels.size()) * std::min(1.0f, horizontal / 2.0f);
            bodies().AddForce(v.car, Vec3(-vel.GetX(), 0, -vel.GetZ()) / horizontal * drag);
        }
    }

    // Driver input and the arcade helpers for one vehicle, before the physics step.
    void drive(Vehicle& v) {
        BodyInterface& bi = bodies();
        const DriveInput& in = v.input;
        const Quat rot = bi.GetRotation(v.car);
        const Vec3 vel = bi.GetLinearVelocity(v.car);
        const Vec3 forward = rot * Vec3::sAxisZ(), up = rot * Vec3::sAxisY(), right = rot * -Vec3::sAxisX();
        const float speed = vel.Dot(forward);
        const int grounded = wheelsOnGround(v);

        // Driver input: brake reverses once nearly stopped.
        float forwardInput = in.throttle, brakeInput = in.brake;
        if (in.brake > 0.1f && in.throttle < 0.1f && speed < 1.0f) {
            forwardInput = -in.brake;
            brakeInput = 0;
        }
        // Speed-sensitive steering keeps fast driving stable.
        const float steer = in.steer * std::clamp(1.0f / (1.0f + std::fabs(speed) / 22.0f), 0.3f, 1.0f);
        auto* controller = static_cast<WheeledVehicleController*>(v.constraint->GetController());
        controller->SetDriverInput(forwardInput, steer, brakeInput, in.handbrake ? 1.0f : 0.0f);

        Vec3 angular = bi.GetAngularVelocity(v.car);
        if (grounded == 0) {
            // Air control: pitch with throttle/brake, yaw and roll with steering.
            angular += (right * (in.throttle - in.brake) * 3.0f + up * in.steer * 2.0f - forward * in.steer * 1.5f) * kStep;
        } else if (v.lastWheelsOnGround == 0) {
            // Landing: absorb the tumble so jumps end on the wheels.
            angular *= 0.5f;
        }
        bi.SetAngularVelocity(v.car, angular);
        // A little downforce helps the tyres at speed.
        bi.AddForce(v.car, -up * v.setup.mass * std::min(std::fabs(speed), 40.0f) * 0.04f);
        v.lastWheelsOnGround = grounded;

        // Self-righting: back on the wheels after two seconds upside down and slow.
        v.upsideDownTime = (up.GetY() < 0.3f && vel.Length() < 3.0f) ? v.upsideDownTime + kStep : 0.0f;
        if (v.upsideDownTime > 2.0f) {
            resetUpright(v);
            return;
        }
        updateSurfaces(v, vel);
    }

    void fixedStep() {
        for (auto& v : vehicles)
            if (!v->car.IsInvalid()) drive(*v);
        updatePatches();  // keep terrain under every vehicle
        physics.Update(kStep, 1, &temp, &jobs);
    }

    void resetUpright(Vehicle& v) {
        const RVec3 pos = bodies().GetPosition(v.car);
        const Vec3 fwd = bodies().GetRotation(v.car) * Vec3::sAxisZ();
        place(v, pos.GetX(), pos.GetZ(), std::atan2(fwd.GetX(), fwd.GetZ()));
    }

    Vehicle* at(int i) const { return i >= 0 && size_t(i) < vehicles.size() ? vehicles[size_t(i)].get() : nullptr; }
};

DriveWorld::DriveWorld(const Heightfield& terrain) {
    initJoltOnce();
    impl_ = std::make_unique<Impl>(terrain);
}

DriveWorld::~DriveWorld() {
    if (impl_)
        for (auto& v : impl_->vehicles) impl_->destroyVehicle(*v);
}

int DriveWorld::addVehicle(const VehicleSetup& setup, double x, double z, float yaw) {
    auto v = std::make_unique<Impl::Vehicle>();
    v->setup = setup;
    impl_->place(*v, x, z, yaw);
    impl_->vehicles.push_back(std::move(v));
    return int(impl_->vehicles.size()) - 1;
}

int DriveWorld::vehicleCount() const { return int(impl_->vehicles.size()); }

void DriveWorld::replaceVehicle(int i, const VehicleSetup& setup, double x, double z, float yaw) {
    if (Impl::Vehicle* v = impl_->at(i)) {
        v->setup = setup;
        impl_->place(*v, x, z, yaw);
    }
}

void DriveWorld::placeVehicle(int i, double x, double z, float yaw) {
    if (Impl::Vehicle* v = impl_->at(i)) impl_->place(*v, x, z, yaw);
}

void DriveWorld::reset(int i) {
    if (Impl::Vehicle* v = impl_->at(i); v && !v->car.IsInvalid()) impl_->resetUpright(*v);
}

void DriveWorld::setInput(int i, const DriveInput& input) {
    if (Impl::Vehicle* v = impl_->at(i)) v->input = input;
}

void DriveWorld::update(float dt) {
    if (impl_->vehicles.empty()) return;
    impl_->accumulator = std::min(impl_->accumulator + dt, 0.25f);
    if (impl_->accumulator >= kStep) impl_->contacts.reset();
    while (impl_->accumulator >= kStep) {
        impl_->fixedStep();
        impl_->accumulator -= kStep;
    }
}

void DriveWorld::spawn(const VehicleSetup& setup, double x, double z, float yaw) {
    if (impl_->vehicles.empty())
        addVehicle(setup, x, z, yaw);
    else
        replaceVehicle(0, setup, x, z, yaw);
}

void DriveWorld::update(float dt, const DriveInput& input) {
    setInput(0, input);
    update(dt);
}


bool DriveWorld::hasStaticShape(uint32_t key) const { return impl_->staticShapes.count(key) != 0; }

void DriveWorld::addStaticShape(uint32_t key, const std::vector<float>& positions, const std::vector<uint32_t>& indices) {
    TriangleList tris;
    tris.reserve(indices.size() / 3);
    for (size_t i = 0; i + 2 < indices.size(); i += 3) {
        auto v = [&](uint32_t k) { return Float3(positions[k * 3], positions[k * 3 + 1], positions[k * 3 + 2]); };
        tris.push_back(Triangle(v(indices[i]), v(indices[i + 1]), v(indices[i + 2])));
    }
    RefConst<Shape> shape;
    if (!tris.empty()) {
        // Props get a solid convex hull (a hollow triangle shell can trap a fast
        // vehicle inside); big structures (buildings, cliffs) keep their exact triangles.
        float radius = 0;
        for (size_t k = 0; k + 2 < positions.size(); k += 3)
            radius = std::max(radius, Vec3(positions[k], positions[k + 1], positions[k + 2]).Length());
        if (radius < 15.0f) {
            Array<Vec3> points;
            for (size_t k = 0; k + 2 < positions.size(); k += 3) points.push_back(Vec3(positions[k], positions[k + 1], positions[k + 2]));
            ConvexHullShapeSettings hs(points, 0.05f);
            auto r = hs.Create();
            if (!r.HasError()) shape = r.Get();
        }
        if (shape == nullptr) {
            MeshShapeSettings ms(tris);
            auto r = ms.Create();
            if (!r.HasError()) shape = r.Get();
        }
    }
    impl_->staticShapes[key] = shape;  // null: nothing to collide with
}

void DriveWorld::setStaticObjects(const std::vector<std::pair<uint32_t, std::array<double, 16>>>& objects) {
    BodyInterface& bi = impl_->bodies();
    std::map<std::pair<uint32_t, std::array<double, 16>>, BodyID> keep;
    for (const auto& o : objects) {
        if (auto it = impl_->staticBodies.find(o); it != impl_->staticBodies.end()) {
            keep.insert(*it);
            impl_->staticBodies.erase(it);
            continue;
        }
        auto s = impl_->staticShapes.find(o.first);
        if (s == impl_->staticShapes.end() || s->second == nullptr) continue;
        const auto& m = o.second;
        // Rows are the basis vectors (row-vector convention); split off their lengths as scale.
        Vec3 axes[3];
        Vec3 scale;
        for (int r = 0; r < 3; ++r) {
            axes[r] = Vec3(float(m[size_t(r * 4)]), float(m[size_t(r * 4 + 1)]), float(m[size_t(r * 4 + 2)]));
            scale.SetComponent(uint(r), std::max(1e-4f, axes[r].Length()));
            axes[r] /= scale[r];
        }
        const Mat44 rot(Vec4(axes[0], 0), Vec4(axes[1], 0), Vec4(axes[2], 0), Vec4(0, 0, 0, 1));
        RefConst<Shape> shape = s->second;
        if (!scale.IsClose(Vec3::sReplicate(1.0f), 1e-4f)) shape = new ScaledShape(shape, scale);
        BodyCreationSettings bs(shape, RVec3(m[12], m[13], m[14]), rot.GetQuaternion().Normalized(), EMotionType::Static, kStatic);
        bs.mFriction = 0.7f;
        keep[o] = bi.CreateAndAddBody(bs, EActivation::DontActivate);
    }
    for (const auto& [key, id] : impl_->staticBodies) {  // objects no longer near
        bi.RemoveBody(id);
        bi.DestroyBody(id);
    }
    impl_->staticBodies = std::move(keep);
}

namespace {

// Static bodies other than the terrain patches.
class NotTerrain : public BodyFilter {
public:
    bool ShouldCollideLocked(const Body& body) const override { return body.GetShape()->GetSubType() != EShapeSubType::HeightField; }
};

}  // namespace

float DriveWorld::clearance(double x, double z, float yaw, float length, float maxSlope, float height) const {
    if (maxSlope > 0) {
        constexpr float kStepLength = 4.0f;
        float prev = impl_->terrain.smoothHeightAt(x, z);
        for (float d = kStepLength; d <= length; d += kStepLength) {
            const float h = impl_->terrain.smoothHeightAt(x + std::sin(yaw) * d, z + std::cos(yaw) * d);
            if (h - prev > maxSlope * kStepLength || prev - h > 1.6f * maxSlope * kStepLength) {  // steeper down is fine
                length = d - kStepLength;
                break;
            }
            prev = h;
        }
    }
    const double x1 = x + std::sin(yaw) * length, z1 = z + std::cos(yaw) * length;
    const RVec3 from(x, impl_->terrain.smoothHeightAt(x, z) + height, z);
    const RVec3 to(x1, impl_->terrain.smoothHeightAt(x1, z1) + height, z1);
    RayCastResult hit;
    const bool blocked = impl_->physics.GetNarrowPhaseQuery().CastRay(RRayCast(from, Vec3(to - from)), hit, SpecifiedBroadPhaseLayerFilter(BroadPhaseLayer(0)),
                                                                      SpecifiedObjectLayerFilter(kStatic), NotTerrain());
    return blocked ? hit.mFraction * length : length;
}

float DriveWorld::rayFraction(const std::array<double, 3>& from, const std::array<double, 3>& to) const {
    const RVec3 a(from[0], from[1], from[2]), b(to[0], to[1], to[2]);
    RayCastResult hit;
    const bool blocked = impl_->physics.GetNarrowPhaseQuery().CastRay(RRayCast(a, Vec3(b - a)), hit, SpecifiedBroadPhaseLayerFilter(BroadPhaseLayer(0)),
                                                                      SpecifiedObjectLayerFilter(kStatic), NotTerrain());
    return blocked ? hit.mFraction : 1.0f;
}

void DriveWorld::setSurfaceQuery(SurfaceQuery query) { impl_->surfaceQuery = std::move(query); }
void DriveWorld::setDefaultSurfaceQuery(SurfaceQuery query) { defaultSurfaceQuery() = std::move(query); }

bool DriveWorld::hasPropShape(uint32_t key) const { return impl_->propShapes.count(key) != 0; }

void DriveWorld::addPropShape(uint32_t key, const std::vector<float>& positions) {
    Impl::PropShape ps;
    Array<Vec3> points;
    Vec3 lo = Vec3::sReplicate(1e9f), hi = Vec3::sReplicate(-1e9f);
    for (size_t k = 0; k + 2 < positions.size(); k += 3) {
        const Vec3 v(positions[k], positions[k + 1], positions[k + 2]);
        points.push_back(v);
        lo = Vec3::sMin(lo, v);
        hi = Vec3::sMax(hi, v);
    }
    if (points.size() >= 4) {
        ConvexHullShapeSettings hs(points, 0.02f);
        auto r = hs.Create();
        if (!r.HasError()) ps.shape = r.Get();
    }
    if (ps.shape == nullptr && points.size() >= 3) {  // flat or degenerate: a thin box around the bounds
        const Vec3 half = Vec3::sMax((hi - lo) * 0.5f, Vec3::sReplicate(0.03f));
        ps.shape = RotatedTranslatedShapeSettings((lo + hi) * 0.5f, Quat::sIdentity(), new BoxShape(half, 0.0f)).Create().Get();
    }
    if (ps.shape != nullptr) ps.mass = std::clamp(ps.shape->GetMassProperties().mMass * (150.0f / 1000.0f), 5.0f, 400.0f);
    impl_->propShapes[key] = ps;
}

bool DriveWorld::addProp(uint32_t id, uint32_t shapeKey, const std::array<double, 16>& m) {
    auto s = impl_->propShapes.find(shapeKey);
    if (s == impl_->propShapes.end() || s->second.shape == nullptr || impl_->props.count(id)) return false;
    Vec3 axes[3], scale;
    for (int r = 0; r < 3; ++r) {
        axes[r] = Vec3(float(m[size_t(r * 4)]), float(m[size_t(r * 4 + 1)]), float(m[size_t(r * 4 + 2)]));
        scale.SetComponent(uint(r), std::max(1e-4f, axes[r].Length()));
        axes[r] /= scale[r];
    }
    const Mat44 rot(Vec4(axes[0], 0), Vec4(axes[1], 0), Vec4(axes[2], 0), Vec4(0, 0, 0, 1));
    RefConst<Shape> shape = s->second.shape;
    float mass = s->second.mass;
    if (!scale.IsClose(Vec3::sReplicate(1.0f), 1e-4f)) {
        shape = new ScaledShape(shape, scale);
        mass = std::clamp(mass * scale.GetX() * scale.GetY() * scale.GetZ(), 5.0f, 400.0f);
    }
    // Stay light next to the vehicle so it pushes through (hulls overstate the volume of hollow props).
    if (!impl_->vehicles.empty()) mass = std::max(5.0f, std::min(mass, impl_->vehicles[0]->setup.mass * 0.2f));
    const Quat q = rot.GetQuaternion().Normalized();
    BodyCreationSettings bs(shape, RVec3(m[12], m[13], m[14]), q, EMotionType::Dynamic, kProp);
    bs.mOverrideMassProperties = EOverrideMassProperties::CalculateInertia;
    bs.mMassPropertiesOverride.mMass = mass;
    bs.mFriction = 0.6f;
    bs.mRestitution = 0.15f;
    bs.mLinearDamping = 0.05f;
    bs.mAngularDamping = 0.2f;
    bs.mAllowSleeping = true;
    bs.mMotionQuality = EMotionQuality::LinearCast;  // a shove into the ground must not tunnel
    Impl::Prop p;
    p.body = impl_->bodies().CreateAndAddBody(bs, EActivation::DontActivate);
    if (p.body.IsInvalid()) return false;
    p.scale = scale;
    p.lastPos = p.startPos = RVec3(m[12], m[13], m[14]);
    p.lastRot = q;
    impl_->props[id] = p;
    return true;
}

bool DriveWorld::hasProp(uint32_t id) const { return impl_->props.count(id) != 0; }

void DriveWorld::removeProp(uint32_t id) {
    auto it = impl_->props.find(id);
    if (it == impl_->props.end()) return;
    impl_->bodies().RemoveBody(it->second.body);
    impl_->bodies().DestroyBody(it->second.body);
    impl_->props.erase(it);
}

std::vector<PropPose> DriveWorld::changedProps() {
    std::vector<PropPose> out;
    BodyInterface& bi = impl_->bodies();
    for (auto& [id, p] : impl_->props) {
        RVec3 pos;
        Quat rot;
        bi.GetPositionAndRotation(p.body, pos, rot);
        if (bi.IsActive(p.body)) {
            // Rescue a prop that still sank through the ground (terrain patch edge, deep embedding).
            const float ground = impl_->terrain.smoothHeightAt(float(pos.GetX()), float(pos.GetZ()));
            if (impl_->onPatch(pos.GetX(), pos.GetZ(), 1000.0) && pos.GetY() < ground - 1.0f) {
                pos = RVec3(pos.GetX(), ground + 0.3f, pos.GetZ());
                bi.SetPositionAndRotation(p.body, pos, rot, EActivation::Activate);
                bi.SetLinearAndAngularVelocity(p.body, Vec3::sZero(), Vec3::sZero());
            }
        }
        if ((pos - p.lastPos).LengthSq() < 1e-8f && rot.IsClose(p.lastRot, 1e-8f)) continue;
        p.lastPos = pos;
        p.lastRot = rot;
        PropPose pp;
        pp.id = id;
        pp.pose.position = {double(pos.GetX()), double(pos.GetY()), double(pos.GetZ())};
        pp.pose.rotation = {rot.GetX(), rot.GetY(), rot.GetZ(), rot.GetW()};
        pp.awake = bi.IsActive(p.body);
        pp.displacement = float((pos - p.startPos).Length());
        out.push_back(pp);
    }
    return out;
}

VehicleState DriveWorld::state(int i) const {
    VehicleState st;
    const Impl::Vehicle* v = impl_->at(i);
    if (v == nullptr || v->car.IsInvalid()) return st;
    BodyInterface& bi = impl_->bodies();
    RVec3 pos;
    Quat rot;
    bi.GetPositionAndRotation(v->car, pos, rot);
    st.chassis.position = {double(pos.GetX()), double(pos.GetY()), double(pos.GetZ())};
    st.chassis.rotation = {rot.GetX(), rot.GetY(), rot.GetZ(), rot.GetW()};
    for (uint32_t i = 0; i < v->constraint->GetWheels().size(); ++i) {
        // Wheel models have their axle along x and the vehicle's right is -x.
        const RMat44 m = v->constraint->GetWheelWorldTransform(i, -Vec3::sAxisX(), Vec3::sAxisY());
        const RVec3 t = m.GetTranslation();
        const Quat q = m.GetQuaternion();
        st.wheels.push_back({{double(t.GetX()), double(t.GetY()), double(t.GetZ())}, {q.GetX(), q.GetY(), q.GetZ(), q.GetW()}});
    }
    st.speedKmh = bi.GetLinearVelocity(v->car).Dot(rot * Vec3::sAxisZ()) * 3.6f;
    auto* controller = static_cast<WheeledVehicleController*>(v->constraint->GetController());
    st.rpm = controller->GetEngine().GetCurrentRPM();
    st.gear = controller->GetTransmission().GetCurrentGear();
    st.wheelsOnGround = Impl::wheelsOnGround(*v);
    // Slip proxy: sideways slip angle (rad) or wheelspin/lock-up ratio beyond what grips normally.
    float skid = 0;
    int touching = 0;
    for (const Wheel* w : v->constraint->GetWheels()) {
        float wheelSkid = 0;
        if (w->HasContact()) {
            const auto* wv = static_cast<const WheelWV*>(w);
            ++touching;
            wheelSkid = std::clamp(std::max((std::fabs(wv->mLateralSlip) - 0.12f) / 0.3f, (std::fabs(wv->mLongitudinalSlip) - 0.35f) / 0.5f), 0.0f, 1.0f);
            skid += wheelSkid;
        }
        st.wheelSkid.push_back(wheelSkid);
    }
    st.skid = touching ? skid / float(touching) : 0.0f;
    st.wheelSurface = v->wheelSurface;
    st.wheelCompression = v->wheelCompression;
    impl_->contacts.read(v->car, st.scrapeSpeed, st.impactSpeed);
    return st;
}

}  // namespace fuel
