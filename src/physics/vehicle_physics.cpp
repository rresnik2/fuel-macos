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
#include <Jolt/Physics/Collision/RayCast.h>
#include <Jolt/Physics/Collision/ShapeCast.h>
#include <Jolt/Physics/Collision/CollisionCollectorImpl.h>
#include <Jolt/Physics/Collision/Shape/SphereShape.h>
#include <Jolt/Physics/Collision/ObjectLayerPairFilterTable.h>
#include <Jolt/Physics/Collision/Shape/BoxShape.h>
#include <Jolt/Physics/Collision/Shape/HeightFieldShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Collision/Shape/ScaledShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Collision/CollideShape.h>
// clang-format on

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstdlib>
#include <ctime>
#include <deque>
#include <filesystem>
#include <fstream>
#include <sstream>
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
// Terrain collision patches sample FUEL's ground (AGW_Altitude, terrain_height.hpp) every
// metre, the vertex spacing of the engine's nearest ground chunks (24 m / 24), so the
// wheels roll on the same detail the player sees.
constexpr uint32_t kPatchSamples = 384;  // terrain collision patch, samples per side (383 m)
constexpr float kPatchSpacing = 1.0f;    // metres
constexpr double kPatchReach = 120.0;    // a vehicle further than this from every patch centre gets a new patch
constexpr double kPatchKeep = 260.0;     // a patch stays while some vehicle is within this of its centre

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

DriveWorld::WaterDepthQuery& defaultWaterQuery() {
    static DriveWorld::WaterDepthQuery q;
    return q;
}

// Ground tables (pp001.tsc PP 4000-4398) by file, read once.
std::shared_ptr<const fv::GroundTables> groundTablesNear(const std::string& dpcPath) {
    static std::mutex mu;
    static std::map<std::string, std::shared_ptr<const fv::GroundTables>> cache;
    namespace fs = std::filesystem;
    fs::path dir = fs::path(dpcPath).parent_path();
    for (int up = 0; up < 6 && !dir.empty(); ++up, dir = dir.parent_path()) {
        const fs::path f = dir / "CommonComponent" / "Param" / "pp001.tsc";
        if (!fs::exists(f)) continue;
        std::lock_guard<std::mutex> lock(mu);
        auto& t = cache[f.string()];
        if (!t) {
            std::ifstream in(f);
            std::stringstream text;
            text << in.rdbuf();
            t = std::make_shared<fv::GroundTables>(fv::parseGroundTables(text.str()));
        }
        return t;
    }
    return nullptr;
}

// FUEL's 18 ground classes from the port's surface types (approximation until the ground generator's class
// fractions, 0x6b76a0, are ported: notes p24 "Ground classes"). Class 0 = tarmac road, 1 = dirt road,
// 3 = bare earth, 5 = rock, 7 = grass, 8 = mud, 9 = sand, 17 = snow.
int physicsGroundClass(int surfaceType) {
    if (surfaceType == 13) return 1;  // DIRT_FLAT: the port's dirt-road surface
    switch (gripCategory(surfaceType)) {
    case GripCategory::Tarmac: return 0;
    case GripCategory::Dirt: return 3;
    case GripCategory::Grass: return 7;
    case GripCategory::Sand: return 9;
    case GripCategory::Snow: return 17;
    case GripCategory::Mud: return 8;
    case GripCategory::Rock: return 5;
    default: return 3;
    }
}

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
    s.peakTorque = param(vp::TorqueScale);  // FUEL's torque scale (engine accel units, see notes p23 8.1)
    s.params = p;
    s.proxy = proxy;
    s.ground = groundTablesNear(dpc.path());
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

    // Terrain patches for the props (Jolt heightfield bodies ~383 m across that follow the vehicles; the
    // vehicles themselves touch FUEL's terrain triangles, fv::TerrainTriangles).
    struct Patch {
        BodyID body;
        double x = 0, z = 0;  // centre
    };
    std::vector<Patch> patches;
    struct PendingPatch {
        std::future<RefConst<Shape>> shape;
        double x0 = 0, z0 = 0;
    };
    std::vector<PendingPatch> pendingPatches;

    std::unordered_map<uint32_t, RefConst<Shape>> staticShapes;
    std::map<std::pair<uint32_t, std::array<double, 16>>, BodyID> staticBodies;

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

    // One vehicle (index 0 is the player): FUEL's model plus a kinematic Jolt body of its proxy that
    // pushes the props.
    struct Vehicle {
        VehicleSetup setup;
        std::unique_ptr<fv::FuelVehicle> model;
        DriveInput input;
        BodyID kin;
        std::vector<int> wheelMap;  // setup wheel -> FUEL wheel
        std::vector<int> wheelSurface;
        std::vector<float> wheelCompression, prevLoad;
        struct Crumb {
            double x, z;
            float yaw;
        };
        std::deque<Crumb> crumbs;  // respawn breadcrumbs (0x528830, simplified)
        int respawns = 0;          // resets by the vehicle's own logic (wreck, overturn, fell out)
        float crumbTimer = 0;
    };
    std::vector<std::unique_ptr<Vehicle>> vehicles;
    float accumulator = 0;
    SurfaceQuery surfaceQuery = defaultSurfaceQuery();
    DriveWorld::WaterDepthQuery waterQuery = defaultWaterQuery();

    // FUEL's model runs in local coordinates around an origin (multiple of 1024 m, so the 4 m terrain cells
    // stay aligned) that follows the player: floats keep millimetre precision anywhere in the 140 km world.
    double ox = 0, oz = 0;
    bool originSet = false;
    fv::TerrainTriangles tris;
    fv::Environment env;
    float stepMs = 0, vehicleMs = 0, vehicleCpuMs = 0, staticsMs = 0, groundMs = 0;

    explicit Impl(const Heightfield& hf) : terrain(hf), tris([&hf](double x, double z) { return hf.smoothHeightAt(x, z); }) {
        layerPairs.EnableCollision(kMoving, kProp);
        layerPairs.EnableCollision(kProp, kStatic);
        layerPairs.EnableCollision(kProp, kProp);
        broadPhaseLayers.MapObjectToBroadPhaseLayer(kStatic, BroadPhaseLayer(0));
        broadPhaseLayers.MapObjectToBroadPhaseLayer(kMoving, BroadPhaseLayer(1));
        broadPhaseLayers.MapObjectToBroadPhaseLayer(kProp, BroadPhaseLayer(1));
        broadPhaseLayers.MapObjectToBroadPhaseLayer(kWheel, BroadPhaseLayer(1));
        objectVsBroadPhase = std::make_unique<ObjectVsBroadPhaseLayerFilterTable>(broadPhaseLayers, 2, layerPairs, 4);
        physics.Init(8192, 0, 8192, 8192, broadPhaseLayers, *objectVsBroadPhase, layerPairs);
        physics.SetGravity(Vec3(0, -kGravity, 0));
        env.terrain = &tris;
        env.sphereStatics = [this](const fv::Vec3& c, float r, std::vector<fv::StaticHit>& out) {
            const auto t0 = std::chrono::steady_clock::now();
            collideStatics(new SphereShape(r), c, fv::Quat{}, out);
            staticsMs += std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - t0).count();
        };
        env.boxStatics = [this](const fv::Vec3& c, const fv::Quat& q, const fv::Vec3& half, std::vector<fv::StaticHit>& out) {
            const auto t0 = std::chrono::steady_clock::now();
            collideStatics(new BoxShape(Vec3(std::max(half.x, 0.06f), std::max(half.y, 0.06f), std::max(half.z, 0.06f)), 0.05f), c, q, out);
            staticsMs += std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - t0).count();
        };
        env.waterDepthAt = [this](double x, double z) { return waterQuery ? waterQuery(x, z) : 0.0f; };
        env.groundClasses = [this](double x, double z, float frac[fv::kGroundClasses], float& sink) {
            const auto t0 = std::chrono::steady_clock::now();
            const int t = surfaceQuery ? surfaceQuery(x, z) : -1;
            groundMs += std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - t0).count();
            frac[physicsGroundClass(t)] = 1;
            sink = 0;
        };
    }

    BodyInterface& bodies() { return physics.GetBodyInterface(); }

    // ---- static objects for FUEL's contacts (0x723400 / 0x723b80 / 0x724120 query the scene's boxes,
    // spheres and mesh triangles; the port asks Jolt for the placed objects' shapes) ----
    void collideStatics(Shape* shapeRaw, const fv::Vec3& c, const fv::Quat& q, std::vector<fv::StaticHit>& out) {
        RefConst<Shape> shape = shapeRaw;
        const RMat44 at = RMat44::sRotationTranslation(Quat(q.x, q.y, q.z, q.w), RVec3(double(c.x) + ox, double(c.y), double(c.z) + oz));
        CollideShapeSettings settings;
        settings.mActiveEdgeMode = EActiveEdgeMode::CollideOnlyWithActive;
        // At most 8 hits per query (FUEL caps a body at 256 contacts): a shape deep inside a dense mesh would
        // otherwise collect hundreds of triangles (spikes of 50+ ms).
        class Limited : public CollideShapeCollector {
        public:
            std::vector<CollideShapeResult> hits;
            void AddHit(const CollideShapeResult& r) override {
                hits.push_back(r);
                if (hits.size() >= 8) ForceEarlyOut();
            }
        } hits;
        physics.GetNarrowPhaseQuery().CollideShape(shape, Vec3::sReplicate(1.0f), at, settings, RVec3::sZero(), hits,
                                                   SpecifiedBroadPhaseLayerFilter(BroadPhaseLayer(0)), SpecifiedObjectLayerFilter(kStatic),
                                                   NotTerrainFilter());
        for (const CollideShapeResult& h : hits.hits) {
            if (!(h.mPenetrationDepth > 0)) continue;
            const Vec3 axis = h.mPenetrationAxis.NormalizedOr(Vec3::sAxisY());
            fv::StaticHit s;
            s.n = {-axis.GetX(), -axis.GetY(), -axis.GetZ()};  // pushes the query shape out
            s.depth = h.mPenetrationDepth;
            s.p = {float(double(h.mContactPointOn2.GetX()) - ox), float(h.mContactPointOn2.GetY()), float(double(h.mContactPointOn2.GetZ()) - oz)};
            out.push_back(s);
        }
    }
    class NotTerrainFilter : public BodyFilter {
    public:
        bool ShouldCollideLocked(const Body& body) const override { return body.GetShape()->GetSubType() != EShapeSubType::HeightField; }
    };

    // ---- terrain patches (props only) ----
    static constexpr double kPatchHalf = 0.5 * kPatchSpacing * (kPatchSamples - 1);
    static double patchOrigin(double v) { return std::round((v - kPatchHalf) / kPatchSpacing) * kPatchSpacing; }
    static RefConst<Shape> makePatchShape(const Heightfield& terrain, double x0, double z0) {
        std::vector<float> samples(size_t(kPatchSamples) * kPatchSamples);
        constexpr uint32_t kThreads = 4;
        std::vector<std::future<void>> jobs;
        for (uint32_t t = 0; t < kThreads; ++t)
            jobs.push_back(std::async(std::launch::async, [&, t] {
                for (uint32_t r = t; r < kPatchSamples; r += kThreads)
                    for (uint32_t c = 0; c < kPatchSamples; ++c)
                        samples[size_t(r) * kPatchSamples + c] = terrain.smoothHeightAt(x0 + c * kPatchSpacing, z0 + r * kPatchSpacing);
            }));
        for (auto& j : jobs) j.get();
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
    // Patches are needed only where props are awake near a vehicle: built in the background ahead of need.
    void updatePatches() {
        for (auto it = pendingPatches.begin(); it != pendingPatches.end();) {
            if (it->shape.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
                ++it;
                continue;
            }
            addPatchBody(it->shape.get(), it->x0, it->z0);
            it = pendingPatches.erase(it);
        }
        if (props.empty()) return;
        constexpr double kPrefetch = kPatchReach * 0.6;
        for (const auto& v : vehicles) {
            const fv::Vec3 o = v->model->modelOrigin();
            const double x = double(o.x) + ox, z = double(o.z) + oz;
            // Never block the frame: a missing patch is built in the background (asleep props wait; one that
            // falls meanwhile is lifted back by changedProps).
            if (!onPatch(x, z, kPrefetch) && !patchPending(x, z, kPatchReach * 0.5)) {
                const double x0 = patchOrigin(x), z0 = patchOrigin(z);
                pendingPatches.push_back({std::async(std::launch::async, [&t = terrain, x0, z0] { return makePatchShape(t, x0, z0); }), x0, z0});
            }
        }
        for (auto it = patches.begin(); it != patches.end();) {
            bool used = false;
            for (const auto& v : vehicles) {
                const fv::Vec3 o = v->model->modelOrigin();
                used = used || std::hypot(double(o.x) + ox - it->x, double(o.z) + oz - it->z) <= kPatchKeep;
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

    // ---- local origin ----
    void setOrigin(double x, double z) {
        const double nx = std::round(x / 1024.0) * 1024.0, nz = std::round(z / 1024.0) * 1024.0;
        if (originSet && nx == ox && nz == oz) return;
        const fv::Vec3 shift{float(ox - nx), 0, float(oz - nz)};
        for (auto& v : vehicles) {
            fv::RigidBody& b = v->model->body();
            for (fv::BodyState* s : {&b.S, &b.P, &b.N}) s->x += shift;
        }
        ox = nx, oz = nz;
        originSet = true;
        tris.setOrigin(ox, oz);
        env.originX = ox, env.originZ = oz;
    }
    fv::Vec3 local(double x, double y, double z) const { return {float(x - ox), float(y), float(z - oz)}; }

    // ---- vehicles ----
    static std::vector<int> wheelMap(const VehicleSetup& s) {
        std::vector<int> m;
        for (const auto& w : s.wheels) {
            if (s.bike)
                m.push_back(w.front ? 0 : 1);
            else
                m.push_back(w.front ? (w.hub[0] > 0 ? 0 : 2) : (w.hub[0] > 0 ? 1 : 3));
        }
        return m;
    }

    void createModel(Vehicle& v) {
        fv::ProxyShapes proxy;
        for (const ProxySphere& sp : v.setup.proxy.spheres) proxy.spheres.push_back({{sp.centre[0], sp.centre[1], sp.centre[2]}, sp.radius, sp.name});
        for (const ProxyBox& b : v.setup.proxy.boxes)
            proxy.boxes.push_back({{b.centre[0], b.centre[1], b.centre[2]}, {b.halfExtents[0], b.halfExtents[1], b.halfExtents[2]}});
        const bool player = &v == vehicles.front().get() || vehicles.empty();
        v.model = std::make_unique<fv::FuelVehicle>(v.setup.params, proxy, v.setup.ground.get(), player);
        v.wheelMap = wheelMap(v.setup);
        v.wheelSurface.assign(v.setup.wheels.size(), -1);
        v.wheelCompression.assign(v.setup.wheels.size(), 0.0f);
        v.prevLoad.assign(v.setup.wheels.size(), 0.0f);
        // Kinematic Jolt body: the proxy boxes and wheel spheres, in model space (body origin = model origin).
        destroyKinematic(v);
        StaticCompoundShapeSettings compound;
        for (const VehicleSetup::Box& b : v.setup.chassis)
            compound.AddShape(toVec3(b.centre), Quat::sIdentity(), new BoxShape(Vec3::sMax(toVec3(b.halfExtents), Vec3::sReplicate(0.06f)), 0.05f));
        for (const auto& w : v.setup.wheels) compound.AddShape(toVec3(w.hub), Quat::sIdentity(), new SphereShape(w.radius));
        auto shape = compound.Create();
        if (!shape.HasError()) {
            BodyCreationSettings bs(shape.Get(), RVec3::sZero(), Quat::sIdentity(), EMotionType::Kinematic, kMoving);
            bs.mAllowSleeping = false;
            v.kin = bodies().CreateAndAddBody(bs, EActivation::Activate);
        }
    }
    void destroyKinematic(Vehicle& v) {
        if (v.kin.IsInvalid()) return;
        bodies().RemoveBody(v.kin);
        bodies().DestroyBody(v.kin);
        v.kin = BodyID();
    }

    // World pose of the model origin (FUEL's body is at the centre of mass).
    void modelPose(const Vehicle& v, RVec3& pos, Quat& rot) const {
        const fv::Vec3 o = v.model->modelOrigin();
        const fv::Quat q = v.model->body().S.q;
        pos = RVec3(double(o.x) + ox, double(o.y), double(o.z) + oz);
        rot = Quat(q.x, q.y, q.z, q.w).Normalized();
    }

    void place(Vehicle& v, double x, double z, float yaw) {
        if (&v == vehicles.front().get() || !originSet) setOrigin(x, z);
        v.model->resetTo(local(x, 0, z), yaw, env);
        v.crumbs.clear();
        v.crumbTimer = 0;
        if (!v.kin.IsInvalid()) {
            RVec3 p;
            Quat q;
            modelPose(v, p, q);
            bodies().SetPositionAndRotation(v.kin, p, q, EActivation::Activate);
        }
    }

    float yawOf(const Vehicle& v) const {
        const fv::Vec3 f = fv::rotate(v.model->body().S.q, {0, 0, 1});
        return std::atan2(f.x, f.z);
    }

    // Respawn (0x529410 -> 0x5288d0(10 m) + 0x5273b0): the newest breadcrumb farther than 10 m, else in place.
    void respawn(Vehicle& v) {
        ++v.respawns;
        const fv::Vec3 o = v.model->modelOrigin();
        const double x = double(o.x) + ox, z = double(o.z) + oz;
        for (auto it = v.crumbs.rbegin(); it != v.crumbs.rend(); ++it)
            if (std::hypot(it->x - x, it->z - z) > 10.0) {
                const Vehicle::Crumb c = *it;
                place(v, c.x, c.z, c.yaw);
                return;
            }
        place(v, x, z, yawOf(v));
    }

    void breadcrumbs(Vehicle& v, float dt) {  // 0x528830 / 0x5281c0 (simplified safety test)
        v.crumbTimer += dt;
        if (v.crumbTimer < 1.0f) return;
        v.crumbTimer = 0;
        const fv::FuelVehicle& m = *v.model;
        for (int w = 0; w < m.wheelCount(); ++w)
            if (!(m.wheelLoad(w) > 0)) return;
        const fv::Vec3 up = fv::rotate(m.body().S.q, {0, 1, 0}), fwd = fv::rotate(m.body().S.q, {0, 0, 1});
        if (up.y < 0.819f || m.body().S.v.y < -30 || fwd.y < -0.707f || fwd.y > 0.5f || m.wrecked()) return;
        const fv::Vec3 o = m.modelOrigin();
        const double x = double(o.x) + ox, z = double(o.z) + oz;
        if (!v.crumbs.empty() && std::hypot(v.crumbs.back().x - x, v.crumbs.back().z - z) < 10.0) return;
        v.crumbs.push_back({x, z, yawOf(v)});
        if (v.crumbs.size() > 32) v.crumbs.pop_front();
    }

    void fixedStep() {
        const auto t0 = std::chrono::steady_clock::now();
        if (!vehicles.empty()) {
            const fv::Vec3 p = vehicles.front()->model->modelOrigin();
            if (std::fabs(p.x) > 3000 || std::fabs(p.z) > 3000) setOrigin(double(p.x) + ox, double(p.z) + oz);
        }
        std::vector<fv::FuelVehicle*> models;
        for (auto& v : vehicles) {
            fv::Inputs in;
            in.accel = std::clamp(v->input.throttle, 0.0f, 1.0f);
            in.brake = std::clamp(v->input.brake, 0.0f, 1.0f);
            in.steer = std::clamp(v->input.steer, -1.0f, 1.0f);
            in.handbrake = v->input.handbrake;
            v->model->setInputs(in);
            models.push_back(v->model.get());
        }
        const auto t1 = std::chrono::steady_clock::now();
        const uint64_t cells0 = tris.cellsBuilt, heights0 = tris.heightsEvaluated;
        staticsMs = groundMs = 0;
        timespec c0, c1;
        clock_gettime(CLOCK_THREAD_CPUTIME_ID, &c0);
        fv::stepVehicles(models, env, kStep);
        clock_gettime(CLOCK_THREAD_CPUTIME_ID, &c1);
        const auto t2 = std::chrono::steady_clock::now();
        vehicleCpuMs += float(c1.tv_sec - c0.tv_sec) * 1000.0f + float(c1.tv_nsec - c0.tv_nsec) * 1e-6f;
        static const bool trace = std::getenv("FUEL_PHYS_TRACE") != nullptr;
        if (trace && std::chrono::duration<float, std::milli>(t2 - t1).count() > 3.0f)
            std::fprintf(stderr, "slow vehicle step %.1f ms (static-object queries %.1f ms, surface queries %.1f ms): %llu cells built, %llu heights evaluated, %zu cached cells\n",
                         std::chrono::duration<float, std::milli>(t2 - t1).count(), staticsMs, groundMs, (unsigned long long)(tris.cellsBuilt - cells0),
                         (unsigned long long)(tris.heightsEvaluated - heights0), tris.cachedCells());
        tris.trim(6000);
        for (auto& vp : vehicles) {
            Vehicle& v = *vp;
            fv::FuelVehicle& m = *v.model;
            if (m.wantsRespawn()) {
                m.clearRespawn();
                respawn(v);
            }
            breadcrumbs(v, kStep);
            for (size_t k = 0; k < v.wheelMap.size(); ++k) {
                const int w = v.wheelMap[k];
                const float load = m.wheelLoad(w);
                v.wheelCompression[k] = (load - v.prevLoad[k]) / kStep;
                v.prevLoad[k] = load;
                const fv::Vec3 hub = m.wheel(w).hubW;
                v.wheelSurface[k] = load > 0 && surfaceQuery ? surfaceQuery(double(hub.x) + ox, double(hub.z) + oz) : -1;
            }
            if (!v.kin.IsInvalid()) {
                RVec3 p;
                Quat q;
                modelPose(v, p, q);
                bodies().MoveKinematic(v.kin, p, q, kStep);
            }
        }
        updatePatches();
        physics.Update(kStep, 1, &temp, &jobs);
        // The vehicles push the props as kinematic bodies (infinite mass): keep a struck prop from leaving
        // faster than the fastest vehicle (+5 m/s), as a dynamic push would.
        if (!props.empty()) {
            float vmax = 0;
            for (const auto& v : vehicles) vmax = std::max(vmax, fv::len(v->model->body().S.v));
            const float cap = vmax + 5.0f;
            for (auto& [id, p] : props) {
                if (!bodies().IsActive(p.body)) continue;
                const Vec3 lv = bodies().GetLinearVelocity(p.body);
                if (lv.Length() > cap) bodies().SetLinearVelocity(p.body, lv * (cap / lv.Length()));
            }
        }
        const auto t3 = std::chrono::steady_clock::now();
        vehicleMs += std::chrono::duration<float, std::milli>(t2 - t1).count();
        stepMs += std::chrono::duration<float, std::milli>(t3 - t0).count();
    }

    Vehicle* at(int i) const { return i >= 0 && size_t(i) < vehicles.size() ? vehicles[size_t(i)].get() : nullptr; }
};

DriveWorld::DriveWorld(const Heightfield& terrain) {
    initJoltOnce();
    impl_ = std::make_unique<Impl>(terrain);
}

DriveWorld::~DriveWorld() {
    if (impl_)
        for (auto& v : impl_->vehicles) impl_->destroyKinematic(*v);
}

int DriveWorld::addVehicle(const VehicleSetup& setup, double x, double z, float yaw) {
    auto v = std::make_unique<Impl::Vehicle>();
    v->setup = setup;
    Impl::Vehicle* raw = v.get();
    impl_->vehicles.push_back(std::move(v));
    impl_->createModel(*raw);
    impl_->place(*raw, x, z, yaw);
    return int(impl_->vehicles.size()) - 1;
}

int DriveWorld::vehicleCount() const { return int(impl_->vehicles.size()); }

void DriveWorld::replaceVehicle(int i, const VehicleSetup& setup, double x, double z, float yaw) {
    if (Impl::Vehicle* v = impl_->at(i)) {
        v->setup = setup;
        impl_->createModel(*v);
        impl_->place(*v, x, z, yaw);
    }
}

void DriveWorld::placeVehicle(int i, double x, double z, float yaw) {
    if (Impl::Vehicle* v = impl_->at(i)) impl_->place(*v, x, z, yaw);
}

void DriveWorld::reset(int i) {
    if (Impl::Vehicle* v = impl_->at(i)) {
        const fv::Vec3 o = v->model->modelOrigin();
        impl_->place(*v, double(o.x) + impl_->ox, double(o.z) + impl_->oz, impl_->yawOf(*v));
    }
}

void DriveWorld::setInput(int i, const DriveInput& input) {
    if (Impl::Vehicle* v = impl_->at(i)) v->input = input;
}

void DriveWorld::update(float dt) {
    impl_->stepMs = impl_->vehicleMs = impl_->vehicleCpuMs = 0;
    if (impl_->vehicles.empty()) return;
    impl_->accumulator = std::min(impl_->accumulator + dt, 0.25f);
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

void DriveWorld::setGuide(int i, bool on, const std::array<double, 3>& ground, const std::array<float, 3>& velocity,
                          const std::array<float, 4>& rotation) {
    if (Impl::Vehicle* v = impl_->at(i)) {
        fv::FuelVehicle::Guide g;
        if (on) {
            fv::FuelVehicle& m = *v->model;
            // The CoM target: the ghost's ground point raised by the CoM's current height over the model origin.
            const fv::Vec3 lift = m.body().S.x - m.modelOrigin();
            g.pos = g.rot = true;
            g.p = impl_->local(ground[0], ground[1], ground[2]) + lift;
            g.v = {velocity[0], velocity[1], velocity[2]};
            g.q = {rotation[0], rotation[1], rotation[2], rotation[3]};
        }
        v->model->setGuide(g);
    }
}

void DriveWorld::holdPose(int i, const std::array<double, 3>& origin, const std::array<float, 4>& rotation, const std::array<float, 3>& velocity) {
    if (Impl::Vehicle* v = impl_->at(i)) {
        fv::FuelVehicle& m = *v->model;
        m.setGuide({});
        const fv::Quat q = fv::qnormalize({rotation[0], rotation[1], rotation[2], rotation[3]});
        const fv::Vec3 x = impl_->local(origin[0], origin[1], origin[2]) + fv::rotate(q, m.comLocal());
        fv::RigidBody& b = m.body();
        for (fv::BodyState* st : {&b.S, &b.P, &b.N}) {
            st->x = x;
            st->q = q;
            st->v = {velocity[0], velocity[1], velocity[2]};
            st->w = {0, 0, 0};
        }
        v->crumbs.clear();
    }
}

int DriveWorld::respawns(int i) const {
    const Impl::Vehicle* v = impl_->at(i);
    return v ? v->respawns : 0;
}

void DriveWorld::nudge(int i, double dy, const std::array<float, 3>& velocity) {
    if (Impl::Vehicle* v = impl_->at(i)) {
        fv::RigidBody& b = v->model->body();
        for (fv::BodyState* s : {&b.S, &b.P, &b.N}) {
            s->x.y += float(dy);
            s->v = {velocity[0], velocity[1], velocity[2]};
        }
    }
}

float DriveWorld::lastStepMs() const { return impl_->stepMs; }
float DriveWorld::lastVehicleMs() const { return impl_->vehicleMs; }
float DriveWorld::lastVehicleCpuMs() const { return impl_->vehicleCpuMs; }
const fv::FuelVehicle* DriveWorld::model(int i) const {
    const Impl::Vehicle* v = impl_->at(i);
    return v ? v->model.get() : nullptr;
}
void DriveWorld::setWaterDepthQuery(WaterDepthQuery query) { impl_->waterQuery = std::move(query); }
void DriveWorld::setDefaultWaterDepthQuery(WaterDepthQuery query) { defaultWaterQuery() = std::move(query); }
float DriveWorld::waterDepthAt(double x, double z) const { return impl_->waterQuery ? impl_->waterQuery(x, z) : 0.0f; }

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

float DriveWorld::rayFraction(const std::array<double, 3>& from, const std::array<double, 3>& to, float radius) const {
    const RVec3 a(from[0], from[1], from[2]), b(to[0], to[1], to[2]);
    if (radius > 0) {
        const SphereShape sphere(radius);
        const RShapeCast cast(&sphere, Vec3::sReplicate(1.0f), RMat44::sTranslation(a), Vec3(b - a));
        AllHitCollisionCollector<CastShapeCollector> hits;
        impl_->physics.GetNarrowPhaseQuery().CastShape(cast, ShapeCastSettings(), RVec3::sZero(), hits,
                                                       SpecifiedBroadPhaseLayerFilter(BroadPhaseLayer(0)), SpecifiedObjectLayerFilter(kStatic),
                                                       NotTerrain());
        float t = 1.0f;
        for (const ShapeCastResult& h : hits.mHits)
            if (h.mFraction > 1e-4f) t = std::min(t, h.mFraction);
        return t;
    }
    RayCastResult hit;
    const bool blocked = impl_->physics.GetNarrowPhaseQuery().CastRay(RRayCast(a, Vec3(b - a)), hit, SpecifiedBroadPhaseLayerFilter(BroadPhaseLayer(0)),
                                                                      SpecifiedObjectLayerFilter(kStatic), NotTerrain());
    return blocked ? hit.mFraction : 1.0f;
}

void DriveWorld::staticBounds(double x0, double z0, double x1, double z1, std::vector<std::array<double, 6>>& out) const {
    out.clear();
    AllHitCollisionCollector<CollideShapeBodyCollector> hits;
    const AABox box(Vec3(float(std::min(x0, x1)), -1.0e5f, float(std::min(z0, z1))), Vec3(float(std::max(x0, x1)), 1.0e5f, float(std::max(z0, z1))));
    impl_->physics.GetBroadPhaseQuery().CollideAABox(box, hits, SpecifiedBroadPhaseLayerFilter(BroadPhaseLayer(0)), SpecifiedObjectLayerFilter(kStatic));
    const BodyLockInterfaceLocking& locks = impl_->physics.GetBodyLockInterface();
    for (const BodyID& id : hits.mHits) {
        BodyLockRead lock(locks, id);
        if (!lock.Succeeded() || lock.GetBody().GetShape()->GetSubType() == EShapeSubType::HeightField) continue;
        const AABox b = lock.GetBody().GetWorldSpaceBounds();
        out.push_back({double(b.mMin.GetX()), double(b.mMin.GetY()), double(b.mMin.GetZ()), double(b.mMax.GetX()), double(b.mMax.GetY()), double(b.mMax.GetZ())});
        if (out.size() >= 512) break;
    }
}

void DriveWorld::overlapStatics(const std::array<double, 3>& centre, float radius, std::vector<SphereContact>& out) const {
    RefConst<Shape> shape = new SphereShape(radius);
    CollideShapeSettings settings;
    settings.mActiveEdgeMode = EActiveEdgeMode::CollideOnlyWithActive;
    class Limited : public CollideShapeCollector {
    public:
        std::vector<CollideShapeResult> hits;
        void AddHit(const CollideShapeResult& r) override {
            hits.push_back(r);
            if (hits.size() >= 16) ForceEarlyOut();
        }
    } hits;
    impl_->physics.GetNarrowPhaseQuery().CollideShape(shape, Vec3::sReplicate(1.0f), RMat44::sTranslation(RVec3(centre[0], centre[1], centre[2])),
                                                      settings, RVec3::sZero(), hits, SpecifiedBroadPhaseLayerFilter(BroadPhaseLayer(0)),
                                                      SpecifiedObjectLayerFilter(kStatic), NotTerrain());
    const BodyLockInterfaceLocking& locks = impl_->physics.GetBodyLockInterface();
    for (const CollideShapeResult& h : hits.hits) {
        SphereContact c;
        const Vec3 axis = h.mPenetrationAxis.NormalizedOr(Vec3::sAxisY());
        c.n = {-axis.GetX(), -axis.GetY(), -axis.GetZ()};
        c.p = {double(h.mContactPointOn2.GetX()), double(h.mContactPointOn2.GetY()), double(h.mContactPointOn2.GetZ())};
        BodyLockRead lock(locks, h.mBodyID2);
        if (lock.Succeeded()) {
            const AABox b = lock.GetBody().GetWorldSpaceBounds();
            const Vec3 e = b.GetExtent() * 2.0f;
            c.volume = e.GetX() * e.GetY() * e.GetZ();
        }
        out.push_back(c);
    }
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
            if (pos.GetY() < ground - 1.0f) {
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
    if (v == nullptr || !v->model) return st;
    const fv::FuelVehicle& m = *v->model;
    RVec3 pos;
    Quat rot;
    impl_->modelPose(*v, pos, rot);
    // Bikes: FUEL keeps the physics body upright and leans the drawn body (0x529ce0, P656-P661).
    if (v->setup.bike && m.visualLean() != 0) rot = (rot * Quat::sRotation(Vec3::sAxisZ(), m.visualLean())).Normalized();
    st.chassis.position = {double(pos.GetX()), double(pos.GetY()), double(pos.GetZ())};
    st.chassis.rotation = {rot.GetX(), rot.GetY(), rot.GetZ(), rot.GetW()};
    float skid = 0;
    int touching = 0;
    for (size_t k = 0; k < v->setup.wheels.size(); ++k) {
        const int w = v->wheelMap[k];
        const auto& sw = v->setup.wheels[k];
        // Drawn wheel: at its hub, dropped by the part of the sphere margin (P55/P56) the contact does not
        // take up (port choice for the visuals; the engine's visual update 0x5cb8c0 is not decoded).
        const float margin = m.param(m.front(w) ? 55 : 56);
        const float droop = std::max(0.0f, margin - m.wheelLoad(w));
        const Vec3 local(sw.hub[0], sw.hub[1] - droop, sw.hub[2]);
        const RVec3 wp = pos + rot * local;
        const Quat wq = rot * Quat::sRotation(Vec3::sAxisY(), m.wheelSteer(w)) * Quat::sRotation(Vec3::sAxisX(), m.wheelSpin(w));
        st.wheels.push_back({{double(wp.GetX()), double(wp.GetY()), double(wp.GetZ())}, {wq.GetX(), wq.GetY(), wq.GetZ(), wq.GetW()}});
        float wheelSkid = 0;
        if (m.wheelLoad(w) > 0) {
            ++touching;
            ++st.wheelsOnGround;
            wheelSkid = std::clamp((m.wheelUtil(w) - 0.8f) / 0.5f, 0.0f, 1.0f);  // grip utilisation past the peak
            skid += wheelSkid;
        }
        st.wheelSkid.push_back(wheelSkid);
    }
    st.speedKmh = m.forwardSpeed() * 3.6f;
    st.rpm = m.displayRpm();  // the smoothed rpm FUEL exports for the sound (+0x1280)
    st.gear = m.gear();
    st.skid = touching ? skid / float(touching) : 0.0f;
    st.wheelSurface = v->wheelSurface;
    st.wheelCompression = v->wheelCompression;
    st.scrapeSpeed = m.body().scrapeSpeed;
    st.impactSpeed = m.body().impactSpeed;
    return st;
}

}  // namespace fuel
