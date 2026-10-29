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
#include <Jolt/Physics/Collision/ObjectLayerPairFilterTable.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/HeightFieldShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Collision/Shape/ScaledShape.h>
#include <Jolt/Physics/Collision/Shape/OffsetCenterOfMassShape.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Vehicle/VehicleCollisionTester.h>
#include <Jolt/Physics/Vehicle/VehicleConstraint.h>
#include <Jolt/Physics/Vehicle/WheeledVehicleController.h>
// clang-format on

#include <algorithm>
#include <cstdlib>
#include <cmath>
#include <mutex>
#include <stdexcept>
#include <map>
#include <thread>
#include <unordered_map>

namespace fuel {

using namespace JPH;

namespace {

constexpr ObjectLayer kStatic = 0, kMoving = 1;
constexpr float kStep = 1.0f / 60.0f;  // FUEL's own physics rate
constexpr float kGravity = 9.81f;
constexpr uint32_t kPatchSamples = 512;  // terrain collision patch, samples per side
constexpr float kPatchSpacing = 4.0f;    // metres (the source grid is 30 m; B-spline resampled)

void initJoltOnce() {
    static std::once_flag once;
    std::call_once(once, [] {
        RegisterDefaultAllocator();
        Factory::sInstance = new Factory();
        RegisterTypes();
    });
}

Vec3 toVec3(const std::array<float, 3>& a) { return Vec3(a[0], a[1], a[2]); }

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
    if (s.wheels.size() != 4) throw std::runtime_error(variant + ": needs four wheel spheres (bikes are not supported yet)");

    // The proxy boxes include the wheels (their bottoms reach the ground);
    // the collision chassis keeps the part above the axles and between the wheels.
    float hubY = 0, innerX = 1e9f;
    for (const VehicleSetup::Wheel& w : s.wheels) {
        hubY += w.hub[1] / 4;
        innerX = std::min(innerX, std::fabs(w.hub[0]) - w.width * 0.5f);
    }
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
    const float r = s.wheels[2].radius;
    s.peakTorque = 0.9f * kGravity * s.mass * r / (s.finalDrive * s.gears[0]);
    return s;
}

// ---- simulation ----------------------------------------------------------------

struct DriveWorld::Impl {
    const Heightfield& terrain;
    TempAllocatorImpl temp{16 * 1024 * 1024};
    JobSystemThreadPool jobs{cMaxPhysicsJobs, cMaxPhysicsBarriers, int(std::max(1u, std::thread::hardware_concurrency() - 1))};
    ObjectLayerPairFilterTable layerPairs{2};
    BroadPhaseLayerInterfaceTable broadPhaseLayers{2, 2};
    std::unique_ptr<ObjectVsBroadPhaseLayerFilterTable> objectVsBroadPhase;
    PhysicsSystem physics;

    BodyID terrainBody;
    double patchX = 0, patchZ = 0;  // centre of the terrain patch

    std::unordered_map<uint32_t, RefConst<Shape>> staticShapes;
    std::map<std::pair<uint32_t, std::array<double, 16>>, BodyID> staticBodies;

    VehicleSetup setup;
    BodyID car;
    Ref<VehicleConstraint> vehicle;
    Ref<VehicleCollisionTester> tester;
    float accumulator = 0;
    float upsideDownTime = 0;
    int lastWheelsOnGround = 4;

    explicit Impl(const Heightfield& hf) : terrain(hf) {
        layerPairs.EnableCollision(kMoving, kStatic);
        layerPairs.EnableCollision(kMoving, kMoving);
        broadPhaseLayers.MapObjectToBroadPhaseLayer(kStatic, BroadPhaseLayer(0));
        broadPhaseLayers.MapObjectToBroadPhaseLayer(kMoving, BroadPhaseLayer(1));
        objectVsBroadPhase = std::make_unique<ObjectVsBroadPhaseLayerFilterTable>(broadPhaseLayers, 2, layerPairs, 2);
        physics.Init(4096, 0, 4096, 4096, broadPhaseLayers, *objectVsBroadPhase, layerPairs);
        physics.SetGravity(Vec3(0, -kGravity, 0));
    }

    BodyInterface& bodies() { return physics.GetBodyInterface(); }

    // A heightfield body covering ~2 km around (x, z), resampled from the
    // world map with the same B-spline the renderer uses.
    void buildTerrainPatch(double x, double z) {
        if (!terrainBody.IsInvalid()) {
            bodies().RemoveBody(terrainBody);
            bodies().DestroyBody(terrainBody);
        }
        const double half = 0.5 * kPatchSpacing * (kPatchSamples - 1);
        const double x0 = std::round((x - half) / kPatchSpacing) * kPatchSpacing;
        const double z0 = std::round((z - half) / kPatchSpacing) * kPatchSpacing;
        std::vector<float> samples(size_t(kPatchSamples) * kPatchSamples);
        for (uint32_t r = 0; r < kPatchSamples; ++r)
            for (uint32_t c = 0; c < kPatchSamples; ++c)
                samples[size_t(r) * kPatchSamples + c] = terrain.smoothHeightAt(x0 + c * kPatchSpacing, z0 + r * kPatchSpacing);
        HeightFieldShapeSettings hs(samples.data(), Vec3::sZero(), Vec3(kPatchSpacing, 1.0f, kPatchSpacing), kPatchSamples);
        auto result = hs.Create();
        if (result.HasError()) throw std::runtime_error(std::string("terrain patch: ") + result.GetError().c_str());
        BodyCreationSettings bs(result.Get(), RVec3(x0, 0, z0), Quat::sIdentity(), EMotionType::Static, kStatic);
        bs.mFriction = 0.9f;
        terrainBody = bodies().CreateAndAddBody(bs, EActivation::DontActivate);
        patchX = x0 + half;
        patchZ = z0 + half;
    }

    void createVehicle(RVec3 position, Quat rotation) {
        const VehicleSetup& s = setup;
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
        car = bodies().CreateAndAddBody(bs, EActivation::Activate);

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
            ws->mPosition = toVec3(w.hub) + Vec3(0, 0.6f * travel, 0);
            ws->mSuspensionDirection = Vec3(0, -1, 0);
            ws->mSuspensionMinLength = 0.02f;
            ws->mSuspensionMaxLength = travel;
            ws->mSuspensionSpring.mFrequency = heavy ? 1.1f : 1.6f;
            ws->mSuspensionSpring.mDamping = 0.55f;
            ws->mRadius = w.radius;
            ws->mWidth = w.width;
            // Jolt's default inertia suits a 20 kg car wheel; size it for this one (solid disc).
            const float wheelMass = std::clamp(s.mass * 0.02f, 8.0f, 200.0f);
            ws->mInertia = 0.5f * wheelMass * w.radius * w.radius;
            ws->mMaxSteerAngle = w.front ? 0.6f : 0.0f;
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
        vs.mAntiRollBars.resize(2);
        vs.mAntiRollBars[0].mLeftWheel = 0;
        vs.mAntiRollBars[0].mRightWheel = 1;
        vs.mAntiRollBars[1].mLeftWheel = 2;
        vs.mAntiRollBars[1].mRightWheel = 3;
        for (auto& bar : vs.mAntiRollBars) bar.mStiffness = s.mass * 0.7f;  // Jolt's reference: ~1000 N/m for a 1500 kg car

        auto* cs = new WheeledVehicleControllerSettings;
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
        const float share = s.frontDriven && s.rearDriven ? 0.5f : 1.0f;
        if (s.frontDriven) addDiff(0, 1, share);
        if (s.rearDriven) addDiff(2, 3, share);
        vs.mController = cs;

        vehicle = new VehicleConstraint(*physics.GetBodyLockInterface().TryGetBody(car), vs);
        tester = new VehicleCollisionTesterCastCylinder(kMoving);
        vehicle->SetVehicleCollisionTester(tester);
        physics.AddConstraint(vehicle);
        physics.AddStepListener(vehicle);
    }

    void destroyVehicle() {
        if (car.IsInvalid()) return;
        physics.RemoveStepListener(vehicle);
        physics.RemoveConstraint(vehicle);
        vehicle = nullptr;
        bodies().RemoveBody(car);
        bodies().DestroyBody(car);
        car = BodyID();
    }

    // Places the vehicle on its wheels at (x, z) with the given heading.
    void place(double x, double z, float yaw) {
        if (std::hypot(x - patchX, z - patchZ) > 300.0 || terrainBody.IsInvalid()) buildTerrainPatch(x, z);
        const double ground = terrain.smoothHeightAt(x, z);
        destroyVehicle();
        createVehicle(RVec3(x, ground + 0.4, z), Quat::sRotation(Vec3::sAxisY(), yaw));
    }

    int wheelsOnGround() const {
        int n = 0;
        for (const Wheel* w : vehicle->GetWheels()) n += w->HasContact() ? 1 : 0;
        return n;
    }

    void fixedStep(const DriveInput& in) {
        BodyInterface& bi = bodies();
        const RVec3 pos = bi.GetPosition(car);
        const Quat rot = bi.GetRotation(car);
        const Vec3 vel = bi.GetLinearVelocity(car);
        const Vec3 forward = rot * Vec3::sAxisZ(), up = rot * Vec3::sAxisY(), right = rot * -Vec3::sAxisX();
        const float speed = vel.Dot(forward);
        const int grounded = wheelsOnGround();

        // Driver input: brake reverses once nearly stopped.
        float forwardInput = in.throttle, brakeInput = in.brake;
        if (in.brake > 0.1f && in.throttle < 0.1f && speed < 1.0f) {
            forwardInput = -in.brake;
            brakeInput = 0;
        }
        // Speed-sensitive steering keeps fast driving stable.
        const float steer = in.steer * std::clamp(1.0f / (1.0f + std::fabs(speed) / 22.0f), 0.3f, 1.0f);
        auto* controller = static_cast<WheeledVehicleController*>(vehicle->GetController());
        controller->SetDriverInput(forwardInput, steer, brakeInput, in.handbrake ? 1.0f : 0.0f);

        Vec3 angular = bi.GetAngularVelocity(car);
        if (grounded == 0) {
            // Air control: pitch with throttle/brake, yaw and roll with steering.
            angular += (right * (in.throttle - in.brake) * 3.0f + up * in.steer * 2.0f - forward * in.steer * 1.5f) * kStep;
        } else if (lastWheelsOnGround == 0) {
            // Landing: absorb the tumble so jumps end on the wheels.
            angular *= 0.5f;
        }
        bi.SetAngularVelocity(car, angular);
        // A little downforce helps the tyres at speed.
        bi.AddForce(car, -up * setup.mass * std::min(std::fabs(speed), 40.0f) * 0.04f);
        lastWheelsOnGround = grounded;

        // Self-righting: back on the wheels after two seconds upside down and slow.
        upsideDownTime = (up.GetY() < 0.3f && vel.Length() < 3.0f) ? upsideDownTime + kStep : 0.0f;
        if (upsideDownTime > 2.0f) {
            upsideDownTime = 0;
            resetUpright();
        }

        // Keep the terrain patch under the vehicle.
        if (std::hypot(double(pos.GetX()) - patchX, double(pos.GetZ()) - patchZ) > 600.0) buildTerrainPatch(pos.GetX(), pos.GetZ());

        physics.Update(kStep, 1, &temp, &jobs);
    }

    void resetUpright() {
        const RVec3 pos = bodies().GetPosition(car);
        const Vec3 fwd = bodies().GetRotation(car) * Vec3::sAxisZ();
        place(pos.GetX(), pos.GetZ(), std::atan2(fwd.GetX(), fwd.GetZ()));
    }
};

DriveWorld::DriveWorld(const Heightfield& terrain) {
    initJoltOnce();
    impl_ = std::make_unique<Impl>(terrain);
}

DriveWorld::~DriveWorld() {
    if (impl_) impl_->destroyVehicle();
}

void DriveWorld::spawn(const VehicleSetup& setup, double x, double z, float yaw) {
    impl_->setup = setup;
    impl_->place(x, z, yaw);
}

void DriveWorld::reset() {
    if (!impl_->car.IsInvalid()) impl_->resetUpright();
}

void DriveWorld::update(float dt, const DriveInput& input) {
    if (impl_->car.IsInvalid()) return;
    impl_->accumulator = std::min(impl_->accumulator + dt, 0.25f);
    while (impl_->accumulator >= kStep) {
        impl_->fixedStep(input);
        impl_->accumulator -= kStep;
    }
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
        MeshShapeSettings ms(tris);
        auto r = ms.Create();
        if (!r.HasError()) shape = r.Get();
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

VehicleState DriveWorld::state() const {
    VehicleState st;
    if (impl_->car.IsInvalid()) return st;
    BodyInterface& bi = impl_->bodies();
    RVec3 pos;
    Quat rot;
    bi.GetPositionAndRotation(impl_->car, pos, rot);
    st.chassis.position = {double(pos.GetX()), double(pos.GetY()), double(pos.GetZ())};
    st.chassis.rotation = {rot.GetX(), rot.GetY(), rot.GetZ(), rot.GetW()};
    for (uint32_t i = 0; i < impl_->vehicle->GetWheels().size(); ++i) {
        // Wheel models have their axle along x and the vehicle's right is -x.
        const RMat44 m = impl_->vehicle->GetWheelWorldTransform(i, -Vec3::sAxisX(), Vec3::sAxisY());
        const RVec3 t = m.GetTranslation();
        const Quat q = m.GetQuaternion();
        st.wheels.push_back({{double(t.GetX()), double(t.GetY()), double(t.GetZ())}, {q.GetX(), q.GetY(), q.GetZ(), q.GetW()}});
    }
    st.speedKmh = bi.GetLinearVelocity(impl_->car).Dot(rot * Vec3::sAxisZ()) * 3.6f;
    auto* controller = static_cast<WheeledVehicleController*>(impl_->vehicle->GetController());
    st.rpm = controller->GetEngine().GetCurrentRPM();
    st.gear = controller->GetTransmission().GetCurrentGear();
    st.wheelsOnGround = impl_->wheelsOnGround();
    return st;
}

}  // namespace fuel
