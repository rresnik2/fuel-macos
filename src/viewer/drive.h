// Drive mode: assembled vehicles on the terrain, moved by the physics.
#pragma once

#include <simd/simd.h>

#include "core/assembly.hpp"
#include "core/automesh.hpp"
#include "core/roads.hpp"
#include "core/vegetation.hpp"
#include "core/world.hpp"
#include "physics/fuel_camera.hpp"
#include "physics/vehicle_physics.hpp"
#include "core/sfx.hpp"
#include "renderer.h"

#include <functional>
#include <future>
#include <memory>
#include <unordered_map>
#include <vector>

class RiderView;
// The riders' animation and skinning (rider_view.h); set by the viewer, null = static riders.
extern RiderView* gRiderView;

// The drawn parts of one vehicle and how they follow its physics state.
class VehicleRig {
public:
    // `riders`: extra meshes (the rider, in vehicle space) that follow the chassis; animated when gRiderView
    // knows them (RiderView::animated).
    VehicleRig(const fuel::AssembledVehicle& vehicle, const fuel::VehicleSetup& setup, const std::vector<uint32_t>& riders);
    // The parts in vehicle space, for MeshRenderer::setScene.
    const std::vector<fuel::MeshInstance>& parts() const { return parts_; }
    const fuel::VehicleSetup& setup() const { return setup_; }
    // The rider for this frame (FUEL's rider state machine, rider_anim.hpp).
    void animate(float dt, const fuel::DriveInput& input, const fuel::VehicleState& st, const fuel::fv::FuelVehicle* model, bool player,
                 int raceState, const fuel::Heightfield& terrain);
    // Skins the animated rider into the renderer's mesh.
    void uploadRider() const;
    // World matrices for the parts, in parts() order.
    std::vector<simd_float4x4> partWorlds(const fuel::VehicleState& st) const;
    simd_float3 focus(const fuel::VehicleState& st) const;  // a point on the body, for the camera

private:
    struct PartLink {
        int wheel = -1;              // physics wheel driving this part, -1 = chassis
        simd_float4x4 local;         // vehicle space (chassis parts) or wheel-model space (rotation only)
        bool rider = false;          // the rider (follows the chassis, or its own fall when ejected)
        uint32_t key = 0;            // rider mesh key
    };
    fuel::VehicleSetup setup_;
    std::vector<fuel::MeshInstance> parts_;
    std::vector<PartLink> links_;
    float focusHeight_ = 0.5f;  // where the camera looks, above the vehicle origin
};

// Drive mode: assembled vehicles on the terrain, moved by the physics. Vehicle 0
// is the player; the others (race opponents) are added with addVehicle and
// driven through setInput.
class DriveSession {
public:
    // Builds the physics vehicle for `vehicle`'s parts and places it at (x, z).
    // `riders`: extra meshes (the baked rider, in vehicle space) that follow the chassis.
    DriveSession(const fuel::AssembledVehicle& vehicle, const fuel::VehicleSetup& setup, const fuel::Heightfield& terrain, double x,
                 double z, float yaw, const std::vector<uint32_t>& riders = {});

    // Adds another vehicle at (x, z) facing `yaw`; returns its index.
    int addVehicle(const fuel::AssembledVehicle& vehicle, const fuel::VehicleSetup& setup, double x, double z, float yaw,
                   const std::vector<uint32_t>& riders = {});
    int vehicleCount() const { return int(rigs_.size()); }
    // Swaps the player's vehicle for another where it stands (the world and other vehicles stay).
    void replacePlayer(const fuel::AssembledVehicle& vehicle, const fuel::VehicleSetup& setup, const std::vector<uint32_t>& riders = {});
    // Input for vehicle `i` > 0 from the next update on (the player's comes with update).
    void setInput(int i, const fuel::DriveInput& input);
    const fuel::DriveInput& input(int i) const { return inputs_[size_t(i)]; }

    // The parts of vehicle `i` in vehicle space, for MeshRenderer::setScene.
    const std::vector<fuel::MeshInstance>& parts(int i = 0) const { return rigs_[size_t(i)].parts(); }
    const fuel::VehicleSetup& setup(int i = 0) const { return rigs_[size_t(i)].setup(); }
    // Advances every vehicle; `input` drives the player.
    void update(float dt, const fuel::DriveInput& input);
    void reset(int i = 0) { world_.reset(i); }
    // Puts vehicle `i` on its wheels at (x, z) facing `yaw`.
    void teleport(double x, double z, float yaw, int i = 0) { world_.placeVehicle(i, x, z, yaw); }
    // World matrices for the parts of vehicle `i`, in parts(i) order.
    std::vector<simd_float4x4> partWorlds(int i = 0) const { return rigs_[size_t(i)].partWorlds(world_.state(i)); }
    fuel::VehicleState state(int i = 0) const { return world_.state(i); }
    simd_float3 position(int i = 0) const { return rigs_[size_t(i)].focus(world_.state(i)); }  // a point on the body, for the camera
    fuel::DriveWorld& physics() { return world_; }
    const fuel::DriveWorld& physics() const { return world_; }
    const fuel::Heightfield& terrain() const { return terrain_; }
    float heading(int i = 0) const;  // radians; 0 = +z
    // Race state for the riders (FUEL's race-state +0x48: 4 start, 6 finished, 0 otherwise).
    void setRaceState(int s) { raceState_ = s; }
    // Skins the animated riders (call before drawing a frame).
    void uploadRiders() const {
        for (const VehicleRig& r : rigs_) r.uploadRider();
    }

private:
    const fuel::Heightfield& terrain_;
    int raceState_ = 0;
    fuel::DriveWorld world_;
    std::vector<VehicleRig> rigs_;
    std::vector<fuel::DriveInput> inputs_;
};

// FUEL's player camera (CameraMove_G, physics/fuel_camera.hpp) on the player's vehicle of a drive session:
// near chase (view 5, default), first person (1) and far chase (4), cycled with the CAMERA button, plus the
// race intro (8), finish (9) and wreck/drown (10/11) views. The camera collides with the terrain and the
// static objects (sphere push-out) and with the other vehicles.
class ChaseCamera {
public:
    fuel::OrbitCamera update(float dt, const DriveSession& drive, const fuel::cam::CameraControls& in = {},
                             fuel::cam::Override ov = fuel::cam::Override::None);
    simd_float3 focus() const { return focus_; }  // for MeshRenderer::setFocus (the vehicle)
    float speedEffect() const { return blur_; }   // 0..1 of FUEL's speed blur (P572..P573), for MeshRenderer::speedBlur
    int view() const { return cam_.view(); }
    void setView(int v) { cam_.setPlayerView(v); }
    const fuel::cam::CameraPose& pose() const { return pose_; }

private:
    fuel::cam::ChaseCamera cam_;
    fuel::cam::CameraPose pose_;
    simd_float3 focus_ = {0, 0, 0};
    float blur_ = 0;
    double prev_[3] = {0, 0, 0};
    bool havePrev_ = false;
};

// FUEL's wheel effects (core/sfx.hpp: SpecialEffectsWheelEmitter_G per wheel, fed like 0x5ceb60) for every
// vehicle near the camera: dust, stones, mud/water projections and tyre smoke, drawn by the renderer's
// billboard_soft pass. Tuning from pp001.tsc and FUEL.exe (setDataDir).
class DustSystem {
public:
    // The FUEL install (CommonComponent/ and XFinalComponent/ below it); without it nothing is emitted.
    static void setDataDir(const std::string& dir);
    // FUEL's point colour of the ground at (x, z), rgba 0..1 (0x6b76a0 colour; the viewer's ground map).
    static std::function<std::array<float, 4>(double x, double z)> groundColour;
    void update(float dt, const DriveSession& drive, simd_float3 eye);
    const fuel::SfxFrame& particles() const { return frame_; }

private:
    std::unique_ptr<fuel::sfx::System> sys_;
    fuel::SfxFrame frame_;
};

// Streams the static world around the player: every object the sub-worlds
// place, drawn with FUEL's per-object LOD and fade (renderer setLodObjects) and solid within kCollideRadius.
class WorldStreamer {
public:
    WorldStreamer(const std::vector<const fuel::DpcFile*>& subWorlds, const std::vector<const fuel::DpcFile*>& lookup);
    ~WorldStreamer() {
        if (job_.valid()) job_.wait();
        if (grassJob_.valid()) grassJob_.wait();
        if (tileJob_.valid()) tileJob_.wait();
    }
    size_t placementCount() const { return placed_.size(); }
    // USA1.DPC's own drawn placements (collectWorldRootMeshes, p36); always loaded like FUEL's
    // resident groups. Call once after construction.
    void addResident(const fuel::DpcFile& usa1);
    // FUEL's sub-world streaming (p35, decode P31 out-F): with the WorldRef table, only the two
    // nearest regions within 5000 m and the course of the running mission are drawn and solid;
    // without it every sub-world is. `mission` = the event id ("" = free roam: no course).
    void setSubWorlds(std::vector<fuel::SubWorldRef> refs);
    void setMission(const std::string& mission);
    // Procedural trees and bushes (drawn; tree trunks solid within kCollideRadius).
    void setVegetation(const fuel::Vegetation* v, const fuel::Heightfield* terrain);
    // Surface types for ground clutter (grass tufts, stones); must be thread-safe.
    void setSurfaces(fuel::SurfaceQuery q) { surfaces_ = std::move(q); }
    // The road network (plant filters near roads). Roads are drawn by the terrain (decals).
    void setRoads(const fuel::RoadNetwork* roads) { roads_ = roads; }
    // Lake water meshes around the player (needs setGround).
    void setLakes(const fuel::Lakes* lakes) { lakes_ = lakes; }
    // FUEL's AutoMesh / RoadMesh objects (core/automesh.hpp), generated with the plants on the
    // worker (needs setVegetation, setGround and setRoads first). FUEL_NO_AUTOMESH=1 disables.
    void setAutoMesh(std::unique_ptr<fuel::AutoMesh> a) { autoMesh_ = std::move(a); }
    // Call every frame; refreshes after the player has moved a little.
    void update(simd_float3 player, fuel::MeshRenderer& renderer, fuel::DriveWorld& physics, bool force = false);
    // Waits for every worker job, then rebuilds everything (objects, plants, roads, grass,
    // fine ground colour) for `player` on this thread: the world as it stands when streaming
    // has fully caught up (--capture-spot).
    void settle(simd_float3 player, fuel::MeshRenderer& renderer, fuel::DriveWorld& physics);
    // Placed objects within `radius` of (x, z) whose base is more than `minGap` metres off the
    // port's terrain (authored heights; the engine seats no static placement, p35).
    struct GroundGap {
        uint32_t meshId = 0;
        simd_float3 position = {0, 0, 0};  // the placement's origin
        float gap = 0;                     // origin y - terrain height, metres (+ floats, - buried)
    };
    std::vector<GroundGap> groundGaps(double x, double z, double radius, float minGap) const;
    // Knock-aside props: small objects near the player are dynamic bodies.
    struct PropStats {
        size_t tracked = 0;  // props with state (a body near the player, or knocked away)
        size_t bodies = 0;   // props currently simulated
        size_t moved = 0;    // props displaced from where they were placed
        float maxDisplacement = 0;  // metres, the furthest of them
        simd_float3 where = {0, 0, 0};  // current position of that one
    };
    PropStats propStats() const;
    // True when (x, z) is within `clearance` metres (at most 5) of a placed object's ground-plane
    // bounding circle: the spot is inside, or touching, a building, rock or prop.
    bool nearObject(double x, double z, float clearance) const { return nearObstacle(x, z, clearance); }
    // Position of the movable prop nearest to `from` (diagnostics); false when there is none.
    bool nearestProp(simd_float3 from, const std::vector<simd_float3>& skip, simd_float3& out) const;

private:
    static constexpr float kCell = 256.0f, kCollideRadius = 250.0f;
    // Decor draw distances follow FUEL's decor quadtree (see decorChunkSize in drive.mm):
    // FUEL's trees end with the 384 m chunks (camera within 2.0625 x 768 m of the 768 m
    // chunk centre, so at most ~2130 m away), bushes with the 96 m chunks, full meshes
    // with the 48 m chunks (~200 m). kPlantRadius only bounds the scatter.
    static constexpr float kPlantRadius = 600.0f;  // every plant within this; only tree cards beyond (the 96 m chunks end within ~540 m)
    static constexpr double kTreeChunkMax = 600.0, kBushChunkMax = 100.0, kFullMeshChunk = 60.0;
    static constexpr float kCardRadius = 2150.0f;  // tree cards beyond kPlantRadius, out to the last 384 m chunks
    static constexpr float kPropRadius = 120.0f, kPropForget = 300.0f;  // dynamic within, state reset beyond
    const fuel::Vegetation* vegetation_ = nullptr;
    fuel::SurfaceQuery surfaces_;
    static constexpr float kClutterRadius = 150.0f;
    const fuel::RoadNetwork* roads_ = nullptr;
    const fuel::Lakes* lakes_ = nullptr;
    // Plants are rebuilt on a worker thread (they took ~6 ms
    // per update on the main thread); the renderer gets them when ready.
    struct Background {
        std::vector<fuel::MeshRenderer::WaterVertex> water;
        std::vector<uint32_t> waterIndices;
        bool haveWater = false;
        std::vector<fuel::MeshInstance> plants;
        std::vector<std::array<double, 16>> trunks;  // solid cylinders for nearby trees (row-vector matrices)
        bool haveAuto = false;
        std::vector<fuel::PlacedMesh> autoObjects;   // AutoMesh / RoadMesh spawns (renderer set 1)
    };
    std::unique_ptr<fuel::AutoMesh> autoMesh_;
    std::vector<std::pair<uint32_t, std::array<double, 16>>> autoSolid_;  // big AutoMesh objects near the player
    Background buildBackground(simd_float3 player) const;
    void buildWater(simd_float3 player, Background& bg) const;
    void applyBackground(Background&& bg, fuel::MeshRenderer& renderer, fuel::DriveWorld& physics);
    void pollBackground(fuel::MeshRenderer& renderer, fuel::DriveWorld& physics);
    void applySolids(fuel::DriveWorld& physics);
    std::vector<std::pair<uint32_t, std::array<double, 16>>> cellSolid_;  // placed objects near the player
    std::vector<std::array<double, 16>> trunks_;
    std::future<Background> job_;
    bool rerun_ = false;      // the player moved on while the job ran
    std::vector<fuel::MeshInstance> cellDraw_, plantDraw_;  // the static world's draw list, in two halves
    const fuel::Heightfield* terrain_ = nullptr;
    struct Placed : fuel::PlacedMesh {  // with its level-of-detail data (p36)
        uint16_t sub = 0;  // index into subFiles_
    };
    void addPlacement(const fuel::PlacedMesh& pm, uint16_t sub);
    // The renderer's object list (setLodObjects): placement index -> list index, or -1.
    std::vector<int32_t> lodIndex_;
    bool lodDirty_ = true;  // the active set changed: hand the renderer a new list
    std::vector<std::string> subFiles_;  // the sub-world DPC file names, in constructor order
    std::vector<fuel::SubWorldRef> refs_;
    std::vector<int> subRef_;            // per sub-world file: index into refs_ (-1 = not streamed)
    std::vector<char> subActive_;        // per sub-world file
    std::string mission_;
    bool streamed_ = false;              // refs_ set: stream regions / courses like FUEL
    bool refreshActive(simd_float3 player);  // true when the loaded set changed
    bool active(const Placed& p) const { return subActive_.empty() || subActive_[p.sub]; }
    // Movable props, keyed by placement index. A prop keeps its state until it is far away.
    struct Prop {
        std::array<float, 16> world;  // current transform (row-vector convention)
        float scale[3];
        bool moved = false, body = false;
    };
    std::unordered_map<uint32_t, Prop> props_;
    std::unordered_map<uint32_t, float> meshRadius_;  // collision radius per mesh (-1: none)
    void updateProps(fuel::MeshRenderer& renderer, fuel::DriveWorld& physics, bool dirty);

    // ---- Grass and the ground generator (p14) ----
public:
    // The decoded ground generator: grass tufts near the player and the trees' ground tests.
    void setGround(const fuel::GroundGenerator* g) { ground_ = g; }

private:
    const fuel::GroundGenerator* ground_ = nullptr;
    // FUEL builds grass for decor chunks of at most 30 m (W+0x5af98), i.e. the 24 m leaves
    // of the decor quadtree (decorChunkSize): they exist while the camera is within
    // 2.0625 x 48 m of their parent's centre, so grass reaches 65..133 m depending on
    // where the chunks fall. Rebuilt on a worker after kGrassStep of travel.
    static constexpr float kGrassReach = 2.0625f * 48.0f + 24.0f * 1.4143f, kGrassStep = 4.0f;
    static constexpr int kGrassChunk = 24;  // metres, FUEL's smallest decor chunk; tufts are cached per chunk
    using GrassCache = std::unordered_map<uint64_t, std::vector<fuel::GrassTuft>>;
    struct GrassJob {
        std::vector<fuel::GrassTuft> tufts;
        GrassCache cache;
    };
    GrassJob buildGrass(simd_float3 player, GrassCache cache) const;
    void updateGrass(simd_float3 player, fuel::MeshRenderer& renderer, bool force);
    // Fine ground colour around the player (MeshRenderer::setGroundTile): FUEL colours its
    // terrain per vertex, 1 m apart near the camera, finer than the 60 m colour map.
    struct GroundTile {
        int level = 0;
        double originX = 0, originZ = 0;
        float extent = 0;
        std::vector<uint8_t> rgba;
        std::vector<uint16_t> types;  // 4 per texel: surface type | weight byte << 8 (bit 21, 0xff = none)
        std::vector<float> decal;     // 4 per texel: road decal u (across), v (along), weight, slot (-1 none)
    };
    // Texels sit on FUEL's vertex lattice of the 24, 48 and 96 m chunks (1, 2 and 4 m).
    static constexpr int kTileLevels = 3;
    static constexpr float kTileSpacing[kTileLevels] = {1.0f, 2.0f, 4.0f}, kTileStep[kTileLevels] = {48.0f, 96.0f, 192.0f};  // rebuilt after this much travel
    GroundTile buildGroundTile(int level, simd_float3 player) const;
    void updateGroundTiles(simd_float3 player, fuel::MeshRenderer& renderer, bool force);
    std::future<GroundTile> tileJob_;
    simd_float2 tileAt_[kTileLevels] = {{1e30f, 1e30f}, {1e30f, 1e30f}, {1e30f, 1e30f}};
    GrassCache grassCache_;  // handed to the worker and back, never shared
    std::future<GrassJob> grassJob_;
    simd_float3 grassAt_ = {1e30f, 0, 1e30f};
    std::vector<const fuel::DpcFile*> lookup_;
    std::vector<Placed> placed_;
    std::unordered_map<uint64_t, std::vector<uint32_t>> cells_;
    // Ground-plane bounding circles of the placements: FUEL plants no tree or bush
    // within 5 m of a placed object.
    struct Obstacle {
        float x, z, r;
    };
    static constexpr float kObstacleCell = 64.0f, kPlantClearance = 5.0f, kMaxObstacle = 60.0f;
    std::vector<Obstacle> obstacles_;
    std::unordered_map<uint64_t, std::vector<uint32_t>> obstacleCells_;
    std::unordered_map<uint32_t, std::array<float, 4>> sphereOf_;  // mesh bounding spheres (load time only)
    void addObstacle(const fuel::PlacedMesh& pm);
    bool nearObstacle(double x, double z, float clearance) const;
    simd_float3 last_ = {1e30f, 0, 0};
};

// The assembled wheel centres in vehicle space (for makeVehicleSetup).
std::vector<std::array<float, 3>> assembledHubs(const fuel::AssembledVehicle& vehicle);
