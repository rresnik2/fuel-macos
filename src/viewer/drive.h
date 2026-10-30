// Drive mode: assembled vehicles on the terrain, moved by the physics.
#pragma once

#include <simd/simd.h>

#include "core/assembly.hpp"
#include "core/roads.hpp"
#include "core/vegetation.hpp"
#include "core/world.hpp"
#include "physics/vehicle_physics.hpp"
#include "renderer.h"

#include <future>
#include <memory>
#include <unordered_map>
#include <vector>

// The drawn parts of one vehicle and how they follow its physics state.
class VehicleRig {
public:
    // `riders`: extra meshes (the baked rider, in vehicle space) that follow the chassis.
    VehicleRig(const fuel::AssembledVehicle& vehicle, const fuel::VehicleSetup& setup, const std::vector<uint32_t>& riders);
    // The parts in vehicle space, for MeshRenderer::setScene.
    const std::vector<fuel::MeshInstance>& parts() const { return parts_; }
    const fuel::VehicleSetup& setup() const { return setup_; }
    // Rider body language for this frame's input.
    void animate(float dt, const fuel::DriveInput& input, float speedKmh);
    // World matrices for the parts, in parts() order.
    std::vector<simd_float4x4> partWorlds(const fuel::VehicleState& st) const;
    simd_float3 focus(const fuel::VehicleState& st) const;  // a point on the body, for the camera

private:
    struct PartLink {
        int wheel = -1;              // physics wheel driving this part, -1 = chassis
        simd_float4x4 local;         // vehicle space (chassis parts) or wheel-model space (rotation only)
        bool rider = false;          // leans with the driving input
    };
    float lean_ = 0, pitch_ = 0;     // rider body language (radians), smoothed
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
    float heading(int i = 0) const;  // radians; 0 = +z

private:
    fuel::DriveWorld world_;
    std::vector<VehicleRig> rigs_;
    std::vector<fuel::DriveInput> inputs_;
};

// The racing chase camera: low and close behind the vehicle, swinging round after
// it with some lag. It drops back when the vehicle accelerates and closes in when
// it brakes, and pulls back and widens its field of view with speed.
class ChaseCamera {
public:
    // Call every frame (or physics step). `yawOffset` looks around the vehicle
    // (radians), `pitchOffset` raises the camera, `zoom` scales the distance.
    fuel::OrbitCamera update(float dt, const DriveSession& drive, float sceneRadius, float yawOffset = 0, float pitchOffset = 0,
                             float zoom = 1);
    simd_float3 focus() const { return focus_; }  // for MeshRenderer::setFocus (vertically smoothed)
    float speedEffect() const;                     // 0..1, for MeshRenderer::speedBlur

private:
    bool started_ = false;
    float yaw_ = 0, speedFast_ = 0, speedSlow_ = 0, surge_ = 0;
    simd_float3 focus_ = {0, 0, 0};
};

// Dust thrown up by the wheels: billowing particles, more on loose ground and
// when the tyres slide or spin, for every vehicle near the camera.
class DustSystem {
public:
    void update(float dt, const DriveSession& drive, simd_float3 eye);
    const std::vector<fuel::DustParticle>& particles() const { return drawn_; }

private:
    struct Particle {
        simd_float3 position, velocity, colour;
        float age, life, size, grow, rotation, spin, opacity, seed;
    };
    std::vector<Particle> particles_;
    std::vector<fuel::DustParticle> drawn_;
    std::vector<simd_float3> lastChassis_;  // per vehicle, for its velocity
    std::vector<float> carry_;              // per vehicle and wheel: fractional particles owed
    uint32_t rng_ = 0x9e3779b9u;
    float random();                         // 0..1
};

// Streams the static world around the player: every object the sub-worlds
// place, drawn within kDrawRadius and solid within kCollideRadius.
class WorldStreamer {
public:
    WorldStreamer(const std::vector<const fuel::DpcFile*>& subWorlds, const std::vector<const fuel::DpcFile*>& lookup);
    ~WorldStreamer() {
        if (job_.valid()) job_.wait();
        if (grassJob_.valid()) grassJob_.wait();
        if (tileJob_.valid()) tileJob_.wait();
    }
    size_t placementCount() const { return placed_.size(); }
    // Procedural trees and bushes (drawn; tree trunks solid within kCollideRadius).
    // Also seats the FITTOGROUND objects on `terrain`, as the engine does at load.
    void setVegetation(const fuel::Vegetation* v, const fuel::Heightfield* terrain);
    // Surface types for ground clutter (grass tufts, stones); must be thread-safe.
    void setSurfaces(fuel::SurfaceQuery q) { surfaces_ = std::move(q); }
    // Road ribbons within kRoadRadius (needs the terrain from setVegetation).
    void setRoads(const fuel::RoadNetwork* roads) { roads_ = roads; }
    // Call every frame; refreshes after the player has moved a little.
    void update(simd_float3 player, fuel::MeshRenderer& renderer, fuel::DriveWorld& physics, bool force = false);
    // Knock-aside props: small objects near the player are dynamic bodies.
    struct PropStats {
        size_t tracked = 0;  // props with state (a body near the player, or knocked away)
        size_t bodies = 0;   // props currently simulated
        size_t moved = 0;    // props displaced from where they were placed
        float maxDisplacement = 0;  // metres, the furthest of them
        simd_float3 where = {0, 0, 0};  // current position of that one
    };
    PropStats propStats() const;
    // Position of the movable prop nearest to `from` (diagnostics); false when there is none.
    bool nearestProp(simd_float3 from, const std::vector<simd_float3>& skip, simd_float3& out) const;

private:
    static constexpr float kCell = 256.0f, kDrawRadius = 1500.0f, kCollideRadius = 250.0f;
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
    static constexpr float kRoadRadius = 1200.0f;
    struct RoadMesh {
        std::vector<fuel::MeshRenderer::RoadVertex> vertices;
        std::vector<uint32_t> indices;
        std::vector<fuel::MeshRenderer::RoadBatch> batches;
    };
    RoadMesh buildRoads(simd_float3 player) const;  // thread-safe
    // Plants and road ribbons are rebuilt on a worker thread (they took ~6 ms
    // per update on the main thread); the renderer gets them when ready.
    struct Background {
        std::vector<fuel::MeshInstance> plants;
        std::vector<std::array<double, 16>> trunks;  // solid cylinders for nearby trees (row-vector matrices)
        RoadMesh roads;
        bool haveRoads = false;
    };
    Background buildBackground(simd_float3 player) const;
    void applyBackground(Background&& bg, fuel::MeshRenderer& renderer, fuel::DriveWorld& physics);
    void pollBackground(fuel::MeshRenderer& renderer, fuel::DriveWorld& physics);
    void applySolids(fuel::DriveWorld& physics);
    std::vector<std::pair<uint32_t, std::array<double, 16>>> cellSolid_;  // placed objects near the player
    std::vector<std::array<double, 16>> trunks_;
    std::future<Background> job_;
    bool rerun_ = false;      // the player moved on while the job ran
    std::vector<fuel::MeshInstance> cellDraw_, plantDraw_;  // the static world's draw list, in two halves
    const fuel::Heightfield* terrain_ = nullptr;
    struct Placed {
        uint32_t meshId;
        std::array<float, 16> world;  // row-vector convention
        bool fitToGround = false, groundNormal = false;
    };
    bool seated_ = false;
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
    };
    static constexpr float kTileSpacing[2] = {1.0f, 4.0f}, kTileStep[2] = {48.0f, 192.0f};  // rebuilt after this much travel
    GroundTile buildGroundTile(int level, simd_float3 player) const;
    void updateGroundTiles(simd_float3 player, fuel::MeshRenderer& renderer, bool force);
    std::future<GroundTile> tileJob_;
    simd_float2 tileAt_[2] = {{1e30f, 1e30f}, {1e30f, 1e30f}};
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
