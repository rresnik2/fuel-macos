#include "vehicle_physics.hpp"
#include "physics/breadcrumbs.hpp"
#include "core/road_segments.hpp"

#include "core/custom_vehicles.hpp"  // CUSTOM VEHICLES
#include "core/genworld_noise.hpp"
#include "core/vehicle_params.hpp"
#include "core/dyn_object.hpp"

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
#include <Jolt/Physics/Collision/Shape/CylinderShape.h>
#include <Jolt/Physics/Collision/Shape/HeightFieldShape.h>
#include <Jolt/Physics/Collision/Shape/ConvexHullShape.h>
#include <Jolt/Physics/Collision/Shape/MeshShape.h>
#include <Jolt/Physics/Collision/Shape/ScaledShape.h>
#include <Jolt/Physics/Collision/Shape/RotatedTranslatedShape.h>
#include <Jolt/Physics/Collision/Shape/StaticCompoundShape.h>
#include <Jolt/Physics/PhysicsSystem.h>
#include <Jolt/Physics/Collision/ContactListener.h>
#include <Jolt/Physics/Collision/CollideShape.h>
// clang-format on

#include <algorithm>
#include <chrono>
#include <cstdio>
#include <cstring>
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
constexpr ObjectLayer kNoContact = 4;  // P87: IT_DebrisMove with +0xeb2 (the crash debris of 0x52ee60): collides with nothing
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

DriveWorld::ClassQuery& defaultClassQuery() {
    static DriveWorld::ClassQuery q;
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
    {  // P87: VehicleInfo+0x38 by the variant's prefix (VehiclesDef.tsc: BIKE_* BIKE, QUAD_* QUAD, ...)
        static const char* const kClasses[] = {"BUGGY", "BIKE", "MUSCLE", "QUAD", "SUV", "TRUCK"};
        for (int k = 0; k < 6; ++k)
            if (variant.rfind(std::string(kClasses[k]) + "_", 0) == 0) s.vehicleClass = k;
    }
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
    s.ground = groundTablesNear(custom::anchor(dpc.path()));  // CUSTOM VEHICLES: a pack's archive is outside the data
    s.damageDirs = readEngineFloats(  // (the player's FUEL.exe: <data>/XFinalComponent, three folders above DATAS/x.DPC)
        (std::filesystem::path(custom::anchor(dpc.path())).parent_path().parent_path().parent_path() / "XFinalComponent/FUEL.exe").string(), 0xa057a8, 30);  // 0x51fd50's direction table, read from the player's FUEL.exe
    return s;
}

// ---- simulation ----------------------------------------------------------------

namespace {
DriveWorld::CrumbTarget& defaultCrumbGps() {
    static DriveWorld::CrumbTarget t;
    return t;
}
}  // namespace

struct DriveWorld::Impl {
    const Heightfield& terrain;
    TempAllocatorImpl temp{16 * 1024 * 1024};
    JobSystemThreadPool jobs{cMaxPhysicsJobs, cMaxPhysicsBarriers, int(std::max(1u, std::thread::hardware_concurrency() - 1))};
    ObjectLayerPairFilterTable layerPairs{5};
    BroadPhaseLayerInterfaceTable broadPhaseLayers{5, 2};
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

    // FUEL's static scene (P46): collision per object key, the placed instances and a 64 m grid over them.
    std::unordered_map<uint32_t, std::shared_ptr<const StaticCollision>> fuelShapes;
    struct FuelStatic {
        std::shared_ptr<const StaticCollision> shape;
        std::array<double, 16> m{};
        double cx = 0, cy = 0, cz = 0, r = 0;  // world bounding sphere
        uint64_t stamp = 0;
        bool built = false;  // local geometry for the current origin
        std::vector<fv::Obb> boxes;
        std::vector<std::pair<fv::Vec3, float>> spheres;
        std::vector<fv::Tri> tris;
    };
    std::vector<FuelStatic> fuelStatics;
    std::unordered_map<int64_t, std::vector<uint32_t>> fuelGrid;
    uint64_t fuelStamp = 0;
    static constexpr double kFuelCell = 64.0;
    static int64_t fuelCellKey(int64_t x, int64_t z) { return (x << 32) ^ (z & 0xffffffff); }
    void buildLocal(FuelStatic& f) {  // node matrix -> local records (0x743ce0 boxes, 0x817cf0 vertices)
        f.boxes.clear(), f.spheres.clear(), f.tris.clear();
        float m[16];
        for (int k = 0; k < 16; ++k) m[k] = float(f.m[size_t(k)]);
        m[12] = float(f.m[12] - ox), m[14] = float(f.m[14] - oz);
        const float scale = std::sqrt(m[0] * m[0] + m[1] * m[1] + m[2] * m[2]);
        auto xf = [&](float x, float y, float z) {
            return fv::Vec3{x * m[0] + y * m[4] + z * m[8] + m[12], x * m[1] + y * m[5] + z * m[9] + m[13], x * m[2] + y * m[6] + z * m[10] + m[14]};
        };
        for (const auto& b : f.shape->boxes) {
            fv::Obb ob = fv::placeObb(b.frame.data(), b.half.data(), b.r, m);
            ob.type = b.type;
            f.boxes.push_back(ob);
        }
        for (const auto& sp : f.shape->spheres) f.spheres.push_back({xf(sp.c[0], sp.c[1], sp.c[2]), sp.r * scale});
        f.tris.reserve(f.shape->tris.size());
        for (size_t i = 0; i < f.shape->tris.size(); ++i) {
            const auto& t = f.shape->tris[i];
            f.tris.push_back(fv::makeTri(xf(t[0], t[1], t[2]), xf(t[3], t[4], t[5]), xf(t[6], t[7], t[8]), int(f.shape->triFlags[i])));
        }
        f.built = true;
    }
    void gatherStatics(const fv::Vec3& c, float r, fv::StaticScene& out) {
        const double wx = double(c.x) + ox, wz = double(c.z) + oz;
        ++fuelStamp;
        const int64_t x0 = int64_t(std::floor((wx - r) / kFuelCell)), x1 = int64_t(std::floor((wx + r) / kFuelCell));
        const int64_t z0 = int64_t(std::floor((wz - r) / kFuelCell)), z1 = int64_t(std::floor((wz + r) / kFuelCell));
        for (int64_t gx = x0; gx <= x1; ++gx)
            for (int64_t gz = z0; gz <= z1; ++gz) {
                auto it = fuelGrid.find(fuelCellKey(gx, gz));
                if (it == fuelGrid.end()) continue;
                for (uint32_t i : it->second) {
                    FuelStatic& f = fuelStatics[i];
                    if (f.stamp == fuelStamp) continue;
                    f.stamp = fuelStamp;
                    const double dx = f.cx - wx, dy = f.cy - double(c.y), dz = f.cz - wz;
                    if (dx * dx + dy * dy + dz * dz > (f.r + r) * (f.r + r)) continue;
                    if (!f.built) buildLocal(f);
                    // 0x73f460 / 0x884f50 / 0x7260a0(1): the shapes touching the island's sphere.
                    for (const fv::Obb& b : f.boxes)
                        if (fv::obbSphereCull(b, c, r)) out.boxes.push_back(b);
                    for (const auto& sp : f.spheres)
                        if (fv::len2(sp.first - c) < (sp.second + r) * (sp.second + r)) out.spheres.push_back(sp);
                    for (const fv::Tri& t : f.tris) {
                        float d;
                        fv::Vec3 n, p;
                        if (fv::sphereTriangle(t, c, r, d, n, p)) out.tris.push_back(&t);
                    }
                }
            }
    }

    struct PropShape {
        RefConst<Shape> shape;
        float mass = 5;
        bool fuelShapes = false;  // P55: built from FUEL's body spheres and boxes (not a hull)
    };
    std::unordered_map<uint32_t, PropShape> propShapes;
    struct Prop {
        BodyID body;
        Vec3 scale = Vec3::sReplicate(1.0f);
        RVec3 lastPos;
        Quat lastRot;
        RVec3 startPos;
        bool fuelDrag = false;  // P55: IT_DYNOBJMove_G drag (0x5321e0) instead of Jolt's damping
        bool debris = false;    // P61: IT_DebrisMove drag (0x533790) and force-volume cap 100 m
        float radius = 0;       // body +0x408 (bounding radius; +0x40c = its square)
        float mass = 0;
        float impactPeak = 0;   // P61: +0xef0 / +0xefc, the strongest approach speed since the last takePropImpacts
    };
    std::unordered_map<uint32_t, Prop> props;
    DriveWorld::PropForceField propForceField;
    // P55: which props touch a vehicle's kinematic proxy this step (Jolt calls from its job threads).
    struct PushListener : public ContactListener {
        std::mutex m;
        std::vector<std::pair<BodyID, BodyID>> pairs;  // (vehicle proxy, prop)
        void note(const Body& a, const Body& b) {
            const bool av = a.GetObjectLayer() == kMoving && b.GetObjectLayer() == kProp;
            const bool bv = b.GetObjectLayer() == kMoving && a.GetObjectLayer() == kProp;
            if (!av && !bv) return;
            std::lock_guard<std::mutex> l(m);
            pairs.push_back(av ? std::make_pair(a.GetID(), b.GetID()) : std::make_pair(b.GetID(), a.GetID()));
        }
        void OnContactAdded(const Body& a, const Body& b, const ContactManifold&, ContactSettings&) override { note(a, b); }
        void OnContactPersisted(const Body& a, const Body& b, const ContactManifold&, ContactSettings&) override { note(a, b); }
    } pushListener;

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
        std::vector<float> visCompression;  // veh+0x1618 (0x5caf40): the exported compression smoothed at rate 2
        fv::Breadcrumbs crumbs;  // respawn breadcrumbs (0x528830 .. 0x5288d0, physics/breadcrumbs.*)
        int respawns = 0;        // resets by the vehicle's own logic (wreck, overturn, fell out)
        // ---- [nitro] (physics/nitro.hpp): the tank, the air / slide detectors, the near-miss pairs (by vehicle index),
        // the half width / length over the wheels and chassis boxes, the top speed (m/s, P619), placements (teleports,
        // respawns, the start hold: a flight ended by one earns nothing).
        nitro::Tank tank;
        nitro::Detector det;
        std::vector<nitro::NearMissPair> pairs;
        float halfW = 1, halfL = 2, top = 0;
        int placed = 0, placedSeen = 0;
        // [upgrades U1] the vehicle's own nitro parameters when its nitrous stages are not stock (refreshed from
        // nitro::params() each step: FUEL_NITRO_PARAMS and the benches change the process's); else the process's.
        bool ownNitro = false;
        nitro::Params np;
        const nitro::Params& nitroParams() const { return ownNitro ? np : nitro::params(); }
        void refreshNitro() {
            if (!ownNitro) return;
            np = nitro::params();
            np.power *= setup.nitroPower;
            np.tankSeconds *= setup.nitroTank;
            np.rechargeRate *= setup.nitroRecharge;
        }
    };
    std::vector<std::unique_ptr<Vehicle>> vehicles;
    int raceState = 0;  // P87: g_Game+0x334c +0x48 (setRaceState)
    float accumulator = 0;
    SurfaceQuery surfaceQuery = defaultSurfaceQuery();
    DriveWorld::ClassQuery classQuery = defaultClassQuery();
    DriveWorld::WaterDepthQuery waterQuery = defaultWaterQuery();
    DriveWorld::TornadoQuery tornadoQuery;

    // FUEL's model runs in local coordinates around an origin (multiple of 1024 m, so the 4 m terrain cells
    // stay aligned) that follows the player: floats keep millimetre precision anywhere in the 140 km world.
    double ox = 0, oz = 0;
    bool originSet = false;
    fv::TerrainTriangles tris;
    fv::Environment env;
    float stepMs = 0, vehicleMs = 0, vehicleCpuMs = 0, staticsMs = 0, groundMs = 0;

    DriveWorld* owner = nullptr;  // for the static-object queries of the breadcrumbs and the respawn probe
    DriveWorld::CrumbTarget crumbRaceTarget, crumbGpsTarget;  // 0x528690 / 0x421dc0 (P70b)
    DriveWorld::StartHold startHold;  // PK01: the pre-start hold (0x5279c0 -> 0x62ce40)
    DriveWorld::WaterDepthQuery lakeDepthQuery;               // 0x40fd10 for the breadcrumbs (P70b)
    explicit Impl(const Heightfield& hf) : terrain(hf), tris([&hf](double x, double z) { return hf.smoothHeightAt(x, z); }) {
        layerPairs.EnableCollision(kMoving, kProp);
        layerPairs.EnableCollision(kProp, kStatic);
        layerPairs.EnableCollision(kProp, kProp);
        broadPhaseLayers.MapObjectToBroadPhaseLayer(kStatic, BroadPhaseLayer(0));
        broadPhaseLayers.MapObjectToBroadPhaseLayer(kMoving, BroadPhaseLayer(1));
        broadPhaseLayers.MapObjectToBroadPhaseLayer(kProp, BroadPhaseLayer(1));
        broadPhaseLayers.MapObjectToBroadPhaseLayer(kWheel, BroadPhaseLayer(1));
        broadPhaseLayers.MapObjectToBroadPhaseLayer(kNoContact, BroadPhaseLayer(1));
        objectVsBroadPhase = std::make_unique<ObjectVsBroadPhaseLayerFilterTable>(broadPhaseLayers, 2, layerPairs, 5);
        physics.Init(8192, 0, 8192, 8192, broadPhaseLayers, *objectVsBroadPhase, layerPairs);
        physics.SetGravity(Vec3(0, -kGravity, 0));
        physics.SetContactListener(&pushListener);
        env.terrain = &tris;
        env.gatherStatics = [this](const fv::Vec3& c, float r, fv::StaticScene& out) {
            const auto t0 = std::chrono::steady_clock::now();
            gatherStatics(c, r, out);
            staticsMs += std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - t0).count();
        };
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
        env.tornadoDistance = [this](double x, double z, float& r) { return tornadoQuery ? tornadoQuery(x, z, r) : (r = 0, 1e15f); };
        env.groundClasses = [this](double x, double y, double z, float frac[fv::kGroundClasses], float& sink) {
            const auto t0 = std::chrono::steady_clock::now();
            if (classQuery) {
                classQuery(x, y, z, frac, sink);
                groundMs += std::chrono::duration<float, std::milli>(std::chrono::steady_clock::now() - t0).count();
                return;
            }
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
                                                   JoltContactFilter());
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
    // Static bodies the vehicles touch through Jolt (user data 1: trunks; objects with FUEL collision are 0).
    class JoltContactFilter : public BodyFilter {
    public:
        bool ShouldCollideLocked(const Body& body) const override { return body.GetUserData() == 1; }
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
        for (FuelStatic& f : fuelStatics) f.built = false;
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
            proxy.boxes.push_back({{b.centre[0], b.centre[1], b.centre[2]}, {b.halfExtents[0], b.halfExtents[1], b.halfExtents[2]}, b.radius});
        const bool player = &v == vehicles.front().get() || vehicles.empty();
        v.model = std::make_unique<fv::FuelVehicle>(v.setup.params, proxy, v.setup.ground.get(), player);
        if (v.setup.damageDirs.size() == 30) v.model->setDamageDirections(v.setup.damageDirs.data());
        v.model->setVehicleClass(v.setup.vehicleClass);  // P87: 0x51e9e0's class skip
        v.model->setRaceState(raceState);
        if (v.setup.torqueScale != 1.0f) v.model->setTorqueScale(v.setup.torqueScale);
        if (v.setup.upgraded) v.model->setUpgraded(true);  // [upgrades U1]
        if (v.setup.lift != 0 || v.setup.widenFront != 0 || v.setup.widenRear != 0)
            v.model->setHubShift(v.setup.lift, v.setup.widenFront, v.setup.widenRear);
        // [upgrades U1] per-vehicle nitro parameters (the vehicle's nitrous stages); stock: the process's (nullptr).
        v.ownNitro = v.setup.nitroPower != 1.0f || v.setup.nitroTank != 1.0f || v.setup.nitroRecharge != 1.0f;
        v.refreshNitro();
        v.model->setNitroParams(v.ownNitro ? &v.np : nullptr);
        v.wheelMap = wheelMap(v.setup);
        v.wheelSurface.assign(v.setup.wheels.size(), -1);
        v.wheelCompression.assign(v.setup.wheels.size(), 0.0f);
        v.prevLoad.assign(v.setup.wheels.size(), 0.0f);
        v.visCompression.assign(v.setup.wheels.size(), 0.0f);  // 0x5c7fe0 zeroes +0x1618
        {  // [nitro]
            v.tank.reset(v.nitroParams());
            v.det = {};
            v.pairs.clear();
            float hw = 0.5f, hl = 1.0f;
            for (const auto& w : v.setup.wheels) hw = std::max(hw, std::fabs(w.hub[0]) + w.width * 0.5f), hl = std::max(hl, std::fabs(w.hub[2]) + w.radius);
            for (const VehicleSetup::Box& b : v.setup.chassis)
                hw = std::max(hw, std::fabs(b.centre[0]) + b.halfExtents[0]), hl = std::max(hl, std::fabs(b.centre[2]) + b.halfExtents[2]);
            v.halfW = hw, v.halfL = hl;
            v.top = v.setup.params.size() > 619 ? nitro::topSpeedMs(v.setup.params[619]) : 0.0f;
        }
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
        placeAt(v, x, std::nan(""), z, yaw);
        v.crumbs.clear();
        v.crumbs.setTimer(0);
    }

    // VehMove_ResetTo 0x5273b0 (snap) at (x, y, z); with a known y the ground height is VehMove_GroundHeightProbe
    // 0x529600: a scene ray from y + 0.25 [0x9dbfb0] (0xa06320 up) 100 m [0x9dbfc0] down, its hit against the
    // terrain height, the higher one (the port's ray: static objects, front faces, DriveWorld::staticFloorBelow).
    void placeAt(Vehicle& v, double x, double y, double z, float yaw) {
        ++v.placed;  // [nitro]
        if (&v == vehicles.front().get() || !originSet) setOrigin(x, z);
        float floorY = std::numeric_limits<float>::quiet_NaN();
        if (!std::isnan(y) && owner) floorY = owner->staticFloorBelow(x, y + 0.25, z, 100.0f);
        v.model->resetTo(local(x, 0, z), yaw, env, floorY);
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

    // The breadcrumb code's view of a vehicle (body position = the CoM state, as FUEL's +0x800).
    fv::CrumbVehicle crumbState(const Vehicle& v) const {
        const fv::FuelVehicle& m = *v.model;
        fv::CrumbVehicle c;
        const fv::Vec3 p = m.body().S.x, lx = m.axisX(), fz = m.axisZ();
        c.pos[0] = double(p.x) + ox, c.pos[1] = double(p.y), c.pos[2] = double(p.z) + oz;
        c.lat[0] = lx.x, c.lat[1] = lx.y, c.lat[2] = lx.z;
        c.fwd[0] = fz.x, c.fwd[1] = fz.y, c.fwd[2] = fz.z;
        c.yaw = yawOf(v);
        c.upY = m.axisY().y;
        c.velY = m.body().S.v.y;
        c.probeNormalY = m.probeNormalY();
        c.maxWaterDepth = m.maxWaterDepth();
        c.allWheelsLoaded = m.allLoaded();
        c.radius = m.extentRadius();
        return c;
    }
    fv::CrumbWorld crumbWorld() const {
        fv::CrumbWorld w;
        if (terrain.roads)
            w.roadCentreline = [r = terrain.roads.get()](double x, double z, double& px, double& pz, float dir[3]) {
                float dx = dir[0], dz = dir[2];
                const bool any = r->coveringCentreline(x, z, px, pz, dx, dz);
                if (any) dir[0] = dx, dir[1] = 0, dir[2] = dz;
                return any;
            };
        if (!vehicles.empty()) {
            const fv::Vec3 pl = vehicles.front()->model->body().S.x;
            const double px = double(pl.x) + ox, pz = double(pl.z) + oz;
            if (crumbRaceTarget) w.raceTarget = [this, px, pz](double& x, double& z) { return crumbRaceTarget(px, pz, x, z); };
            const DriveWorld::CrumbTarget& gps = crumbGpsTarget ? crumbGpsTarget : defaultCrumbGps();
            if (gps) w.gpsTarget = [&gps, px, pz](double& x, double& z) { return gps(px, pz, x, z); };
        }
        // 0x527bf0: the port's scene = the static objects (Jolt), not the props or vehicles.
        if (owner)
            w.blocked = [o = owner](const double p[3], float r) {
                std::vector<DriveWorld::SphereContact> hits;
                o->overlapStatics({p[0], p[1], p[2]}, r, hits);
                return !hits.empty();
            };
        w.waterDepth = [this](double x, double z) { return lakeDepthQuery ? lakeDepthQuery(x, z) : 0.0f; };
        w.baseHeight = [this](double x, double z) { return float(terrain.baseHeightAt(x, z)); };
        return w;
    }

    // Respawn (0x529410 / 0x528b60 -> VehMove_PickBreadcrumb 0x5288d0 + VehMove_ResetTo 0x5273b0 with snap):
    // 10 m for a wreck or an overturn [0x996494], 100 m for the RESPAWN button (0x528c10, [0x9dbfc0]).
    void respawn(Vehicle& v) {
        ++v.respawns;
        const float minDist = v.model->respawnReason() == 2 ? 100.0f : 10.0f;
        const fv::CrumbVehicle c = crumbState(v);
        fv::Breadcrumbs::Crumb at;
        if (!v.crumbs.pick(minDist, c, at)) at.x = c.pos[0], at.y = c.pos[1], at.z = c.pos[2], at.yaw = c.yaw;  // in place
        // FUEL's target is the body (CoM) position; the port places the model origin: the CoM's horizontal offset
        // under the new heading is taken off.
        const fv::Vec3 com = fv::rotate(fv::axisAngle({0, 1, 0}, at.yaw), v.model->comLocal());
        placeAt(v, at.x - double(com.x), at.y, at.z - double(com.z), at.yaw);
    }

    void breadcrumbs(Vehicle& v, float dt) { v.crumbs.tick(dt, crumbState(v), crumbWorld()); }  // 0x528830

    // [nitro] The fills the physics sees, once per step after it: a landed flight, a powerslide, a near miss with any
    // other vehicle (opponents, traffic). Race overtakes come from the race (addNitroBonus).
    void nitroBonuses() {
        const size_t n = vehicles.size();
        for (size_t i = 0; i < n; ++i) {
            Vehicle& v = *vehicles[i];
            const nitro::Params& np = v.nitroParams();  // [upgrades U1] the vehicle's tank
            const fv::FuelVehicle& m = *v.model;
            const bool placed = v.placed != v.placedSeen;
            v.placedSeen = v.placed;
            const fv::Vec3 u = m.axisY(), f = m.axisZ(), vel = m.body().S.v;
            if (const float got = v.det.air(m.airtime(), u.y > 0.5f, m.wrecked(), placed, np); got > 0) v.tank.add(got, nitro::KindAir, np);
            const float slip = nitro::slipAngleDeg({vel.x, vel.y, vel.z}, {f.x, f.y, f.z}, {u.x, u.y, u.z});
            if (const float got = v.det.powerslide(kStep, slip, fv::len(vel), m.anyLoaded(), m.wrecked(), np); got > 0) v.tank.add(got, nitro::KindSlide, np);
            if (v.pairs.size() != n) v.pairs.assign(n, {});
            const fv::Vec3 l = m.axisX();
            for (size_t j = 0; j < n; ++j) {
                if (j == i) continue;
                const Vehicle& o = *vehicles[j];
                nitro::NearMissPair& pr = v.pairs[j];
                const fv::Vec3 r = o.model->body().S.x - m.body().S.x;
                const float lat = fv::dot(r, l), lon = fv::dot(r, f);
                if (std::fabs(lon) > 30.0f || std::fabs(lat) > 30.0f || placed) {  // far (or i was moved): no pass under way
                    pr.close = false;
                    pr.cooldown = std::max(0.0f, pr.cooldown - kStep);
                    continue;
                }
                const float closing = std::fabs(fv::dot(o.model->body().S.v - vel, f));
                if (const float got = pr.step(kStep, lat, lon, v.halfW + o.halfW, v.halfL + o.halfL, closing, np); got > 0) v.tank.add(got, nitro::KindNearMiss, np);
            }
        }
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
            in.stickZ = std::clamp(v->input.stickZ, -1.0f, 1.0f);  // P47: phys +0x1228 (input block +0x198)
            in.handbrake = v->input.handbrake;
            v->model->setInputs(in);
            if (nitro::enabled()) {  // [nitro] the tank on the last step's ground contact; boosts only in free roam / racing
                const bool allowed = !v->model->wrecked() && !(raceState >= 2 && raceState <= 4) && raceState < 6;
                v->refreshNitro();  // [upgrades U1]
                v->tank.step(kStep, v->input.nitro, v->model->anyLoaded(), allowed, v->nitroParams());
                v->model->setNitro(v->tank.amount, v->top);
            } else if (v->model->nitro() != 0) {  // switched off live (options page): FUEL's forces again
                v->model->setNitro(0, v->top);
                v->tank.reset(v->nitroParams());
            }
            models.push_back(v->model.get());
        }
        const auto t1 = std::chrono::steady_clock::now();
        const uint64_t cells0 = tris.cellsBuilt, heights0 = tris.heightsEvaluated;
        staticsMs = groundMs = 0;
        timespec c0, c1;
        clock_gettime(CLOCK_THREAD_CPUTIME_ID, &c0);
        {  // FUEL_TEST_WRECK=T: the player takes a full damage event at T seconds (films of the wreck and ejection)
            static const char* tw = std::getenv("FUEL_TEST_WRECK");
            static double simT = 0;
            simT += kStep;
            if (tw && !vehicles.empty() && simT >= std::atof(tw) && simT - kStep < std::atof(tw)) vehicles.front()->model->addTestDamage();
            // P85 FUEL_TEST_DAMAGE=T:slot=v[,slot=v...]: every vehicle's impact record (pm+0x144) takes these slots at T s
            // (films of the visual damage below the wreck threshold).
            // P87 FUEL_TEST_SCRAPE=T: at T s the player is rolled onto its roof (180 deg about its forward axis, 1 m up,
            // velocity kept): the chassis then slides on the ground (the crash debris' scrape case).
            static const char* tsc = std::getenv("FUEL_TEST_SCRAPE");
            if (tsc && !vehicles.empty() && simT >= std::atof(tsc) && simT - kStep < std::atof(tsc)) {
                fv::RigidBody& b = vehicles.front()->model->body();
                const fv::Quat flip = fv::axisAngle(fv::normalize(fv::rotate(b.S.q, {0, 0, 1})), float(M_PI));
                for (fv::BodyState* st : {&b.S, &b.P, &b.N}) st->q = fv::qnormalize(fv::mul(flip, st->q)), st->x.y += 1.0f, st->w = {};
                b.updateWorldInertia();
            }
            static const char* td = std::getenv("FUEL_TEST_DAMAGE");
            if (td && simT >= std::atof(td) && simT - kStep < std::atof(td))
                for (const char* q = std::strchr(td, ':'); q != nullptr; q = std::strchr(q + 1, ',')) {
                    int slot = 0;
                    float v = 0;
                    if (std::sscanf(q + 1, "%d=%f", &slot, &v) == 2 && slot >= 0 && slot < 10)
                        for (auto& veh : vehicles) veh->model->addTestDamage(slot, v);
                }
        }
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
            // PK01: 0x529410's first requests: _CheckpointRespawn 0x5279c0 (the player only, +0x142b) -> 0x62ce40.
            // In race states 2-4 (before the start) the player's vehicle goes back on the start pose after every
            // step (0x529480 orientation + VehMove_ResetTo 0x5273b0 without the snap, y = the start's terrain
            // height + veh +0x230: the port's placement on the terrain, the same pose), its breadcrumbs cleared
            // (0x52f620); the wreck / button / fell-out requests of the step are not looked at (first true wins).
            // The cut-scene slot branch is the port's TELEPORT_HELICO hold (no physics steps while it plays).
            if (&vp == &vehicles.front() && startHold && raceState >= 2 && raceState <= 4) {
                double sx = 0, sz = 0;
                float heading = 0;
                if (startHold(sx, sz, heading)) {
                    m.clearRespawn();
                    place(v, sx, sz, heading);
                }
            }
            if (m.wantsRespawn()) {
                static const bool rtrace = std::getenv("FUEL_RESPAWN_TRACE") != nullptr;
                if (rtrace) {
                    const fv::Vec3 o = m.modelOrigin();
                    std::fprintf(stderr, "respawn vehicle %d reason %d at %.1f %.1f %.1f\n", int(&vp - &vehicles.front()), m.respawnReason(), double(o.x) + ox, double(o.y), double(o.z) + oz);
                }
                m.clearRespawn();
                respawn(v);
            }
            breadcrumbs(v, kStep);
            for (size_t k = 0; k < v.wheelMap.size(); ++k) {
                const int w = v.wheelMap[k];
                const float load = m.wheelLoad(w);
                v.wheelCompression[k] = (load - v.prevLoad[k]) / kStep;
                v.prevLoad[k] = load;
                // 0x5caf40: +0x1618 = Math_ExpSmooth(+0x1618 -> +0x15e8, 2 [0x99601c], dt) (per render frame in FUEL,
                // per physics step here).
                v.visCompression[k] = fv::expSmooth(v.visCompression[k], m.exportedCompression(w), 2.0f, kStep);
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
        if (nitro::enabled()) nitroBonuses();  // [nitro]
        updatePatches();
        // P55: IT_DYNOBJMove_G body forces per step (vt[129] 0x5315d0): gravity -9.81 is Jolt's own; the drag
        // 0x5321e0 decelerates the linear and the angular velocity by min(speed, 10) per second.
        for (auto& [id, p] : props) {
            if (!p.fuelDrag || !bodies().IsActive(p.body)) continue;
            Vec3 lv, av;
            bodies().GetLinearAndAngularVelocity(p.body, lv, av);
            if (p.debris) {  // 0x533790: F = -200 r^2 0.01 min(|v|, 10)^2 v/|v| (k = +0x564 = 200, 0x70bf50); no angular term
                const float sp = lv.Length();
                if (sp > 0.0001f) {
                    const float c = std::min(sp, 10.0f);
                    const float dv = std::min(200.0f * p.radius * p.radius * 0.01f * c * c / p.mass * kStep, sp);
                    bodies().SetLinearVelocity(p.body, lv * ((sp - dv) / sp));
                }
                continue;
            }
            const auto l = dynObjDrag({lv.GetX(), lv.GetY(), lv.GetZ()}, kDynDragMaxSpeed, kStep);
            const auto a = dynObjDrag({av.GetX(), av.GetY(), av.GetZ()}, kDynDragMaxSpin, kStep);
            bodies().SetLinearAndAngularVelocity(p.body, Vec3(l[0], l[1], l[2]), Vec3(a[0], a[1], a[2]));
        }
        // PhysObj_ForceVolumes(150 m) (0x710e30): the force fields' pull, capped at 150 m (wind / tornado / magnets;
        // P61: the tornado vortex of 0x681540 comes through setPropForceField, viewer/storm_view).
        if (propForceField)
            for (auto& [id, p] : props) {
                if (!p.fuelDrag) continue;
                const RVec3 x = bodies().GetCenterOfMassPosition(p.body);
                const float m = p.mass;
                const std::array<float, 3> f = propForceField({double(x.GetX()), double(x.GetY()), double(x.GetZ())}, m);
                Vec3 F(f[0], f[1], f[2]);
                const float cap = (p.debris ? 100.0f : kDynForceVolumeLimit) * m;  // 0x710f20(m x 100) for debris
                if (F.LengthSq() < 1e-12f) continue;
                if (F.Length() > cap) F *= cap / F.Length();
                bodies().AddForce(p.body, F, EActivation::Activate);
            }
        std::unordered_map<uint32_t, Vec3> propVelBefore;  // BodyID index -> velocity before the step (FUEL props)
        for (auto& [id, p] : props)
            if (p.fuelDrag && bodies().IsActive(p.body)) propVelBefore[p.body.GetIndexAndSequenceNumber()] = bodies().GetLinearVelocity(p.body);
        pushListener.pairs.clear();
        physics.Update(kStep, 1, &temp, &jobs);
        pushBack(propVelBefore);
        // P61: the contact filter vt[141] (0x531c00 / 0x5341b0) keeps the peak approach speed of the frame's contacts;
        // [port] estimated as the step's velocity change less gravity.
        for (auto& [id, p] : props) {
            if (!p.fuelDrag || !bodies().IsActive(p.body)) continue;
            const auto vb = propVelBefore.find(p.body.GetIndexAndSequenceNumber());
            const Vec3 v0 = vb == propVelBefore.end() ? Vec3::sZero() : vb->second;
            const float dv = (bodies().GetLinearVelocity(p.body) - v0 - Vec3(0, -kGravity, 0) * kStep).Length();
            p.impactPeak = std::max(p.impactPeak, dv);
        }
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

    // P55: the vehicles push FUEL's dynamic objects through a kinematic proxy (infinite mass); give the vehicle the
    // momentum the prop took (Newton's third law; FUEL resolves the body-body contact with both masses, 0x70ce10 /
    // 0x683ee0): dv' = (dv - g dt) m_v / (m_v + m_prop) for the prop, J = m_prop dv' at its centre for the vehicle. FUEL_P55_NO_PUSHBACK=1 off.
    void pushBack(const std::unordered_map<uint32_t, Vec3>& before) {
        static const bool off = std::getenv("FUEL_P55_NO_PUSHBACK") != nullptr;
        if (off || pushListener.pairs.empty()) return;
        std::sort(pushListener.pairs.begin(), pushListener.pairs.end(), [](const auto& x, const auto& y) {
            return x.second.GetIndexAndSequenceNumber() < y.second.GetIndexAndSequenceNumber() ||
                   (x.second == y.second && x.first.GetIndexAndSequenceNumber() < y.first.GetIndexAndSequenceNumber());
        });
        pushListener.pairs.erase(std::unique(pushListener.pairs.begin(), pushListener.pairs.end()), pushListener.pairs.end());
        std::unordered_map<uint32_t, const Prop*> byBody;
        for (const auto& [id, p] : props) byBody[p.body.GetIndexAndSequenceNumber()] = &p;
        for (const auto& [kin, propBody] : pushListener.pairs) {
            auto pb = byBody.find(propBody.GetIndexAndSequenceNumber());
            auto vb = before.find(propBody.GetIndexAndSequenceNumber());  // absent: it was asleep (at rest)
            if (pb == byBody.end() || !pb->second->fuelDrag) continue;
            const Vec3 v0 = vb == before.end() ? Vec3::sZero() : vb->second;
            Vehicle* veh = nullptr;
            for (auto& v : vehicles)
                if (v->kin == kin) veh = v.get();
            if (veh == nullptr) continue;
            fv::RigidBody& b = veh->model->body();
            const Vec3 g = Vec3(0, -kGravity, 0) * kStep;
            Vec3 dv = bodies().GetLinearVelocity(propBody) - v0 - g;
            // The proxy moved the prop as an infinite mass would; with the vehicle's mass the exchange is shared:
            // both the prop's velocity change and the vehicle's reaction scale by m_vehicle / (m_vehicle + m_prop).
            const float share = b.mass / std::max(1e-3f, b.mass + pb->second->mass);
            dv *= share;
            bodies().SetLinearVelocity(propBody, v0 + g + dv);
            const Vec3 J = dv * pb->second->mass;
            const RVec3 pc = bodies().GetCenterOfMassPosition(propBody);
            const fv::Vec3 r = local(pc.GetX(), pc.GetY(), pc.GetZ()) - b.S.x;
            const fv::Vec3 j{J.GetX(), J.GetY(), J.GetZ()};
            static const bool trace = std::getenv("FUEL_P55_PUSHTRACE") != nullptr;
            if (trace) std::fprintf(stderr, "pushback: prop %.1f kg dv %.2f m/s J %.0f Ns (r.J %.1f), vehicle %.0f kg\n", pb->second->mass, dv.Length(), J.Length(), fv::dot(j, r), b.mass);
            b.S.v = b.S.v - j * b.invMass;
            b.S.w = b.S.w - b.I2invW * fv::cross(r, j);
        }
    }

    Vehicle* at(int i) const { return i >= 0 && size_t(i) < vehicles.size() ? vehicles[size_t(i)].get() : nullptr; }
};

DriveWorld::DriveWorld(const Heightfield& terrain) {
    initJoltOnce();
    impl_ = std::make_unique<Impl>(terrain);
    impl_->owner = this;
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

void DriveWorld::setRaceState(int s) {  // P87
    impl_->raceState = s;
    for (auto& v : impl_->vehicles)
        if (v->model) v->model->setRaceState(s);
}

void DriveWorld::replaceVehicle(int i, const VehicleSetup& setup, double x, double z, float yaw) {
    if (Impl::Vehicle* v = impl_->at(i)) {
        v->setup = setup;
        impl_->createModel(*v);
        impl_->place(*v, x, z, yaw);
    }
}

void DriveWorld::endingRestore(int i) {  // PK14
    if (Impl::Vehicle* v = impl_->at(i); v && v->model) v->model->endingRestore();
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
    for (auto& v : impl_->vehicles) {  // the sound reads the frame's largest impact (0x5d9f70 clears it per frame)
        v->model->body().soundImpact = {0, 0, 0};
        v->model->body().soundImpactKey = 0;
    }
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

void DriveWorld::setCrumbTargets(CrumbTarget race, CrumbTarget gps) {
    impl_->crumbRaceTarget = std::move(race);
    impl_->crumbGpsTarget = std::move(gps);
}

void DriveWorld::setStartHold(StartHold hold) { impl_->startHold = std::move(hold); }  // PK01

void DriveWorld::placeOnNode(int i, const std::array<double, 3>& node, const std::array<float, 4>& rotation) {  // PK01 0x42bd30
    // Player_G vf137: the vehicle (MVehicle +0x4a8 / +0x4c0, body vf117 0x713e80: placed at rest) at the agent
    // node's world position + (0, 1, 0) [0xa06320] x vf60 (0.3 m, 0x428b40), with the node's rotation.
    holdPose(i, {node[0], node[1] + 0.3, node[2]}, rotation, {0, 0, 0});
}

void DriveWorld::setDefaultCrumbGpsTarget(CrumbTarget gps) { defaultCrumbGps() = std::move(gps); }

void DriveWorld::setLakeDepthQuery(std::function<float(double x, double z)> query) { impl_->lakeDepthQuery = std::move(query); }

int DriveWorld::respawns(int i) const {
    const Impl::Vehicle* v = impl_->at(i);
    return v ? v->respawns : 0;
}

// ---- [nitro]
void DriveWorld::addNitroBonus(int i, float seconds, int kind) {
    if (Impl::Vehicle* v = impl_->at(i); v && nitro::enabled()) v->tank.add(seconds, kind, v->nitroParams());
}

void DriveWorld::resetNitro() {
    for (auto& v : impl_->vehicles) {
        v->refreshNitro();  // [upgrades U1]
        v->tank.reset(v->nitroParams());
        v->det = {};
        v->pairs.clear();
        if (v->model) v->model->setNitro(0, v->top);
    }
}

const nitro::Tank* DriveWorld::nitroTank(int i) const {
    const Impl::Vehicle* v = impl_->at(i);
    return v ? &v->tank : nullptr;
}

const nitro::Params& DriveWorld::nitroParams(int i) const {  // [upgrades U1]
    const Impl::Vehicle* v = impl_->at(i);
    return v ? v->nitroParams() : nitro::params();
}
// ---- [nitro] end

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

void DriveWorld::addStaticCylinder(uint32_t key) {
    // Jolt cylinders take a scale with x == z; the 0.02 convex radius rounds the rims only.
    RefConst<Shape> cyl = new CylinderShape(0.5f, 1.0f, 0.02f);
    impl_->staticShapes[key] = new RotatedTranslatedShape(Vec3(0, 0.5f, 0), Quat::sIdentity(), cyl);
}

bool DriveWorld::hasStaticCollision(uint32_t key) const { return impl_->fuelShapes.count(key) != 0; }

void DriveWorld::addStaticCollision(uint32_t key, std::shared_ptr<const StaticCollision> collision) {
    // The Jolt mesh for rays: the collision triangles and the boxes' faces.
    TriangleList tris;
    for (const auto& t : collision->tris) tris.push_back(Triangle(Float3(t[0], t[1], t[2]), Float3(t[3], t[4], t[5]), Float3(t[6], t[7], t[8])));
    for (const auto& b : collision->boxes) {
        Vec3 c[8];
        for (int k = 0; k < 8; ++k) {
            Vec3 p(b.frame[3], b.frame[7], b.frame[11]);
            for (int i = 0; i < 3; ++i) {
                const float sgn = (k >> i) & 1 ? 1.0f : -1.0f;
                p += Vec3(b.frame[size_t(4 * i)], b.frame[size_t(4 * i + 1)], b.frame[size_t(4 * i + 2)]) * (sgn * b.half[size_t(i)]);
            }
            c[k] = p;
        }
        static const int kFaces[6][4] = {{0, 2, 6, 4}, {1, 5, 7, 3}, {0, 4, 5, 1}, {2, 3, 7, 6}, {0, 1, 3, 2}, {4, 6, 7, 5}};
        for (const auto& f : kFaces) {
            auto v = [&](int i) { return Float3(c[f[i]].GetX(), c[f[i]].GetY(), c[f[i]].GetZ()); };
            tris.push_back(Triangle(v(0), v(1), v(2)));
            tris.push_back(Triangle(v(0), v(2), v(3)));
        }
    }
    RefConst<Shape> shape;
    if (!tris.empty()) {
        MeshShapeSettings ms(tris);
        auto r = ms.Create();
        if (!r.HasError()) shape = r.Get();
    }
    impl_->staticShapes[key] = shape;
    impl_->fuelShapes[key] = std::move(collision);
}

void DriveWorld::setStaticObjects(const std::vector<std::pair<uint32_t, std::array<double, 16>>>& objects) {
    // FUEL's scene: the instances with FUEL collision.
    {
        Impl& I = *impl_;
        I.fuelStatics.clear();
        I.fuelGrid.clear();
        for (const auto& o : objects) {
            auto f = I.fuelShapes.find(o.first);
            if (f == I.fuelShapes.end() || f->second->empty()) continue;
            Impl::FuelStatic s;
            s.shape = f->second;
            s.m = o.second;
            const double scale = std::sqrt(o.second[0] * o.second[0] + o.second[1] * o.second[1] + o.second[2] * o.second[2]);
            s.cx = o.second[12], s.cy = o.second[13], s.cz = o.second[14], s.r = double(s.shape->radius) * scale + 0.1;
            const uint32_t idx = uint32_t(I.fuelStatics.size());
            const int64_t x0 = int64_t(std::floor((s.cx - s.r) / Impl::kFuelCell)), x1 = int64_t(std::floor((s.cx + s.r) / Impl::kFuelCell));
            const int64_t z0 = int64_t(std::floor((s.cz - s.r) / Impl::kFuelCell)), z1 = int64_t(std::floor((s.cz + s.r) / Impl::kFuelCell));
            for (int64_t gx = x0; gx <= x1; ++gx)
                for (int64_t gz = z0; gz <= z1; ++gz) I.fuelGrid[Impl::fuelCellKey(gx, gz)].push_back(idx);
            I.fuelStatics.push_back(std::move(s));
        }
    }
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
        bs.mUserData = impl_->fuelShapes.count(o.first) ? 0 : 1;  // FUEL's own tests for FUEL collision
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

float DriveWorld::staticFloorBelow(double x, double y, double z, float depth) const {
    RayCastSettings settings;
    settings.mTreatConvexAsSolid = false;  // starting inside an object: no hit at 0
    settings.SetBackFaceMode(EBackFaceMode::IgnoreBackFaces);
    ClosestHitCollisionCollector<CastRayCollector> hit;
    const RRayCast ray(RVec3(x, y, z), Vec3(0, -depth, 0));
    impl_->physics.GetNarrowPhaseQuery().CastRay(ray, settings, hit, SpecifiedBroadPhaseLayerFilter(BroadPhaseLayer(0)),
                                                 SpecifiedObjectLayerFilter(kStatic), NotTerrain());
    return hit.HadHit() ? float(y - double(depth) * double(hit.mHit.mFraction)) : std::nanf("");
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

void DriveWorld::staticBoxes(double x0, double z0, double x1, double z1, std::vector<WorldBox>& out) const {
    out.clear();
    const Impl& I = *impl_;
    const int64_t gx0 = int64_t(std::floor(std::min(x0, x1) / Impl::kFuelCell)), gx1 = int64_t(std::floor(std::max(x0, x1) / Impl::kFuelCell));
    const int64_t gz0 = int64_t(std::floor(std::min(z0, z1) / Impl::kFuelCell)), gz1 = int64_t(std::floor(std::max(z0, z1) / Impl::kFuelCell));
    std::vector<uint32_t> seen;
    for (int64_t gx = gx0; gx <= gx1; ++gx)
        for (int64_t gz = gz0; gz <= gz1; ++gz) {
            const auto it = I.fuelGrid.find(Impl::fuelCellKey(gx, gz));
            if (it == I.fuelGrid.end()) continue;
            for (uint32_t idx : it->second) {
                if (std::find(seen.begin(), seen.end(), idx) != seen.end()) continue;
                seen.push_back(idx);
                const Impl::FuelStatic& st = I.fuelStatics[idx];
                const auto& m = st.m;  // row vectors: world = local x row0 + y row1 + z row2 + row3
                for (const auto& b : st.shape->aiBoxes) {
                    WorldBox w;
                    for (int a = 0; a < 3; ++a) {
                        const float lx = b.frame[size_t(4 * a)], ly = b.frame[size_t(4 * a + 1)], lz = b.frame[size_t(4 * a + 2)];
                        for (int c = 0; c < 3; ++c) w.frame[size_t(4 * a + c)] = float(lx * m[size_t(c)] + ly * m[size_t(4 + c)] + lz * m[size_t(8 + c)]);
                    }
                    const float cx = b.frame[3], cy = b.frame[7], cz = b.frame[11];
                    for (int c = 0; c < 3; ++c) w.frame[size_t(4 * c + 3)] = float(cx * m[size_t(c)] + cy * m[size_t(4 + c)] + cz * m[size_t(8 + c)] + m[size_t(12 + c)]);
                    w.half = b.half, w.r = b.r, w.type = b.type;
                    out.push_back(w);
                    if (out.size() >= 1024) return;
                }
            }
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
void DriveWorld::setClassQuery(ClassQuery query) { impl_->classQuery = std::move(query); }
void DriveWorld::setDefaultClassQuery(ClassQuery query) { defaultClassQuery() = std::move(query); }
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

void DriveWorld::addPropShape(uint32_t key, const StaticCollision& c) {
    // P55: a PhysicObj_Z body is its spheres and boxes (the volume and inertia passes of 0x712de0 walk vt 0x1c8 / 0x1c4;
    // the object's triangles are what vehicles meet in the static scene). Objects without either keep the hull.
    if (c.spheres.empty() && c.boxes.empty()) {
        addPropShape(key, collisionHullPoints(c));
        return;
    }
    StaticCompoundShapeSettings cs;
    for (const auto& sp : c.spheres) cs.AddShape(Vec3(sp.c[0], sp.c[1], sp.c[2]), Quat::sIdentity(), new SphereShape(std::max(sp.r, 0.02f)));
    for (const auto& b : c.boxes) {
        Vec3 ax[3];
        for (int a = 0; a < 3; ++a) ax[a] = Vec3(b.frame[size_t(4 * a)], b.frame[size_t(4 * a + 1)], b.frame[size_t(4 * a + 2)]);
        // rows are the box axes (static_collision.hpp); orthonormalise for the rotation
        ax[0] = ax[0].NormalizedOr(Vec3::sAxisX());
        ax[1] = (ax[1] - ax[0] * ax[0].Dot(ax[1])).NormalizedOr(Vec3::sAxisY());
        ax[2] = ax[0].Cross(ax[1]);
        const Quat q = Mat44(Vec4(ax[0], 0), Vec4(ax[1], 0), Vec4(ax[2], 0), Vec4(0, 0, 0, 1)).GetQuaternion().Normalized();
        const Vec3 half = Vec3::sMax(Vec3(b.half[0], b.half[1], b.half[2]), Vec3::sReplicate(0.03f));
        cs.AddShape(Vec3(b.frame[3], b.frame[7], b.frame[11]), q, new BoxShape(half, std::min(0.02f, half.ReduceMin() * 0.5f)));
    }
    Impl::PropShape ps;
    auto r = cs.Create();
    if (r.HasError()) {
        addPropShape(key, collisionHullPoints(c));
        return;
    }
    ps.shape = r.Get();
    ps.mass = float(std::max(0.2, dynObjVolume(c) * 5.0));  // replaced by the caller's FUEL mass (addProp)
    ps.fuelShapes = true;
    impl_->propShapes[key] = ps;
}

bool DriveWorld::addProp(uint32_t id, uint32_t shapeKey, const std::array<double, 16>& m) { return addProp(id, shapeKey, m, 0.0f); }

bool DriveWorld::addProp(uint32_t id, uint32_t shapeKey, const std::array<double, 16>& m, float fuelMass, bool debris) {
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
    if (fuelMass <= 0 && !impl_->vehicles.empty()) mass = std::max(5.0f, std::min(mass, impl_->vehicles[0]->setup.mass * 0.2f));
    // P55: IT_DYNOBJMove_G's own mass (0x531560, dyn_object.hpp); port floor 0.2 kg for the solver.
    if (fuelMass > 0) mass = std::max(fuelMass, 0.2f);
    const Quat q = rot.GetQuaternion().Normalized();
    BodyCreationSettings bs(shape, RVec3(m[12], m[13], m[14]), q, EMotionType::Dynamic, kProp);
    bs.mOverrideMassProperties = EOverrideMassProperties::CalculateInertia;
    bs.mMassPropertiesOverride.mMass = mass;
    bs.mFriction = 0.6f;
    bs.mRestitution = 0.15f;
    bs.mLinearDamping = fuelMass > 0 ? 0.0f : 0.05f;  // FUEL's drag is applied per step instead (fixedStep)
    bs.mAngularDamping = fuelMass > 0 && !debris ? 0.0f : 0.2f;  // [port] debris: 0x533790 has no angular drag
    bs.mAllowSleeping = true;
    bs.mMotionQuality = EMotionQuality::LinearCast;  // a shove into the ground must not tunnel
    Impl::Prop p;
    p.body = impl_->bodies().CreateAndAddBody(bs, EActivation::DontActivate);
    if (p.body.IsInvalid()) return false;
    p.scale = scale;
    p.lastPos = p.startPos = RVec3(m[12], m[13], m[14]);
    p.lastRot = q;
    p.fuelDrag = fuelMass > 0;
    p.debris = debris && fuelMass > 0;
    {
        const AABox b = shape->GetLocalBounds();
        p.radius = 0.5f * (b.mMax - b.mMin).Length();
    }
    p.mass = mass;
    impl_->props[id] = p;
    return true;
}

void DriveWorld::setPropForceField(PropForceField field) { impl_->propForceField = std::move(field); }
void DriveWorld::setTornadoQuery(TornadoQuery query) { impl_->tornadoQuery = std::move(query); }

std::vector<DriveWorld::PropImpact> DriveWorld::takePropImpacts() {
    std::vector<PropImpact> out;
    for (auto& [id, p] : impl_->props) {
        if (p.impactPeak <= 0) continue;
        const RVec3 x = impl_->bodies().GetCenterOfMassPosition(p.body);
        out.push_back({id, p.impactPeak, {double(x.GetX()), double(x.GetY()), double(x.GetZ())}, p.radius});
        p.impactPeak = 0;
    }
    return out;
}

bool DriveWorld::propVelocity(uint32_t id, std::array<float, 3>& v) const {
    auto it = impl_->props.find(id);
    if (it == impl_->props.end()) return false;
    const Vec3 lv = impl_->bodies().GetLinearVelocity(it->second.body);
    v = {lv.GetX(), lv.GetY(), lv.GetZ()};
    return true;
}

void DriveWorld::setPropNoContact(uint32_t id) {  // P87
    auto it = impl_->props.find(id);
    if (it == impl_->props.end()) return;
    impl_->bodies().SetObjectLayer(it->second.body, kNoContact);
}

void DriveWorld::wakeProp(uint32_t id, const std::array<float, 3>& velocity) {
    auto it = impl_->props.find(id);
    if (it == impl_->props.end()) return;
    impl_->bodies().SetLinearVelocity(it->second.body, Vec3(velocity[0], velocity[1], velocity[2]));
    impl_->bodies().ActivateBody(it->second.body);
}

void DriveWorld::wakeProp(uint32_t id, const std::array<float, 3>& velocity, const std::array<float, 3>& angular) {  // P85
    auto it = impl_->props.find(id);
    if (it == impl_->props.end()) return;
    impl_->bodies().SetLinearAndAngularVelocity(it->second.body, Vec3(velocity[0], velocity[1], velocity[2]), Vec3(angular[0], angular[1], angular[2]));
    impl_->bodies().ActivateBody(it->second.body);
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
        // Drawn wheel: at its hub, dropped by the part of the sphere margin (P55/P56) the contact does not take up.
        // Port choice for the height: FUEL's 0x5caf40 puts the node at rest + hub offset + (0, c + P63|P64 + P65, 0),
        // which sits P55 + P63 + P65 (-0.1 .. -0.27 m) under the contact plane of this physics (notes
        // p81-physics-big.md, open W1). The camber and its smoothed compression are FUEL's.
        const float margin = m.param(m.front(w) ? 55 : 56);
        const float droop = std::max(0.0f, margin - m.wheelLoad(w));
        const Vec3 local(sw.hub[0], sw.hub[1] - droop, sw.hub[2]);
        const RVec3 wp = pos + rot * local;
        const float camber = m.visualCamber(w, k < v->visCompression.size() ? v->visCompression[k] : 0.0f);
        const Quat wq = rot * Quat::sRotation(Vec3::sAxisZ(), camber) * Quat::sRotation(Vec3::sAxisY(), m.wheelSteer(w)) *
                        Quat::sRotation(Vec3::sAxisX(), m.wheelSpin(w));
        st.wheels.push_back({{double(wp.GetX()), double(wp.GetY()), double(wp.GetZ())}, {wq.GetX(), wq.GetY(), wq.GetZ(), wq.GetW()}});
        float wheelSkid = 0;
        if (m.wheelLoad(w) > 0) {
            ++touching;
            ++st.wheelsOnGround;
            wheelSkid = std::clamp((m.wheelUtil(w) - 0.8f) / 0.5f, 0.0f, 1.0f);  // grip utilisation past the peak
            skid += wheelSkid;
        }
        st.wheelSkid.push_back(wheelSkid);
        {
            VehicleState::WheelFx fx;
            const float r = m.wheelRadius(w);
            const Vec3 up = rot * Vec3::sAxisY(), fwdW = rot * Vec3::sAxisZ();
            const RVec3 c = wp - RVec3(up * r);
            fx.contact = {double(c.GetX()), double(c.GetY()), double(c.GetZ())};
            const RVec3 hubW = pos + rot * Vec3(sw.hub[0], sw.hub[1], sw.hub[2]);
            const fv::Vec3 pv = m.pointVelocity({float(hubW.GetX() - impl_->ox), float(hubW.GetY()), float(hubW.GetZ() - impl_->oz)});
            fx.velocity = {pv.x, pv.y, pv.z};
            fx.forward = {fwdW.GetX(), fwdW.GetY(), fwdW.GetZ()};
            fx.radius = r;
            fx.spin = m.rimSpeed(w) / std::max(r, 0.01f);
            fx.load = m.wheelLoad(w);
            fx.slip = fx.load > 0.01f ? std::max(0.0f, m.wheelSlipRatio(w)) : 0.0f;
            fx.depth = m.waterDepth(w);
            const auto& cf = m.classFractions(w);
            for (size_t k = 0; k < fx.classes.size() && k < cf.size(); ++k) fx.classes[k] = cf[k];
            st.wheelFx.push_back(fx);
        }
    }
    st.speedKmh = m.forwardSpeed() * 3.6f;
    st.velocity = {m.body().S.v.x, m.body().S.v.y, m.body().S.v.z};
    st.rpm = m.displayRpm();  // the smoothed rpm FUEL exports for the sound (+0x1280)
    st.gear = m.gear();
    st.skid = touching ? skid / float(touching) : 0.0f;
    st.wheelSurface = v->wheelSurface;
    st.wheelCompression = v->wheelCompression;
    st.scrapeSpeed = m.body().scrapeSpeed;
    st.damage = m.damage();
    st.wrecked = m.wrecked();
    st.riderEjected = m.riderEjected();
    st.impactSpeed = m.body().impactSpeed;
    st.soundRpm = m.soundRpm();
    st.engineThrottle = m.engineThrottle();
    st.revLimiter = m.revLimiter();
    st.velocity = {m.body().S.v.x, m.body().S.v.y, m.body().S.v.z};
    st.soundImpact = {m.body().soundImpact.x, m.body().soundImpact.y, m.body().soundImpact.z};
    st.impactMeter = m.impactMeter();
    st.slipPedal = m.slipPedal();
    if (nitro::enabled()) {  // [nitro]
        const nitro::Tank& t = v->tank;
        st.nitro.on = true;
        st.nitro.level = t.level, st.nitro.amount = t.amount;
        st.nitro.active = t.active, st.nitro.firing = t.firing, st.nitro.held = t.held;
        st.nitro.emptyFlash = t.emptyFlash, st.nitro.bonusGlow = t.bonusGlow;
        st.nitro.lastKind = t.lastKind, st.nitro.lastBonus = t.lastBonus, st.nitro.starts = t.starts;
        const nitro::Params& np = v->nitroParams();  // [upgrades U1] the vehicle's tank and power
        st.nitro.tankSeconds = np.tankSeconds, st.nitro.minStartLevel = np.fraction(np.minStart), st.nitro.power = np.power;
    }
    return st;
}

}  // namespace fuel
