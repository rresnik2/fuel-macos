// Drive mode: an assembled vehicle on the terrain, moved by the physics.
#pragma once

#include <simd/simd.h>

#include "core/assembly.hpp"
#include "core/roads.hpp"
#include "core/vegetation.hpp"
#include "physics/vehicle_physics.hpp"
#include "renderer.h"

#include <memory>
#include <unordered_map>
#include <vector>

class DriveSession {
public:
    // Builds the physics vehicle for `vehicle`'s parts and places it at (x, z).
    // `riders`: extra meshes (the baked rider, in vehicle space) that follow the chassis.
    DriveSession(const fuel::AssembledVehicle& vehicle, const fuel::VehicleSetup& setup, const fuel::Heightfield& terrain, double x,
                 double z, float yaw, const std::vector<uint32_t>& riders = {});

    // The parts in vehicle space, for MeshRenderer::setScene.
    const std::vector<fuel::MeshInstance>& parts() const { return parts_; }
    void update(float dt, const fuel::DriveInput& input) { world_.update(dt, input); }
    void reset() { world_.reset(); }
    // Puts the vehicle on its wheels at (x, z) facing `yaw`.
    void teleport(double x, double z, float yaw) { world_.spawn(setup_, x, z, yaw); }
    // World matrices for the parts, in parts() order.
    std::vector<simd_float4x4> partWorlds() const;
    fuel::VehicleState state() const { return world_.state(); }
    simd_float3 position() const;  // a point on the body, for the camera
    fuel::DriveWorld& physics() { return world_; }
    float heading() const;  // radians; 0 = +z

private:
    struct PartLink {
        int wheel = -1;              // physics wheel driving this part, -1 = chassis
        simd_float4x4 local;         // vehicle space (chassis parts) or wheel-model space (rotation only)
    };
    fuel::DriveWorld world_;
    fuel::VehicleSetup setup_;
    std::vector<fuel::MeshInstance> parts_;
    std::vector<PartLink> links_;
    float focusHeight_ = 0.5f;  // where the camera looks, above the vehicle origin
};

// Streams the static world around the player: every object the sub-worlds
// place, drawn within kDrawRadius and solid within kCollideRadius.
class WorldStreamer {
public:
    WorldStreamer(const std::vector<const fuel::DpcFile*>& subWorlds, const std::vector<const fuel::DpcFile*>& lookup);
    size_t placementCount() const { return placed_.size(); }
    // Procedural trees and bushes within kPlantRadius (drawn, not solid).
    void setVegetation(const fuel::Vegetation* v, const fuel::Heightfield* terrain) { vegetation_ = v, terrain_ = terrain; }
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
    static constexpr float kCell = 256.0f, kDrawRadius = 1500.0f, kCollideRadius = 250.0f, kPlantRadius = 700.0f;
    static constexpr float kPropRadius = 120.0f, kPropForget = 300.0f;  // dynamic within, state reset beyond
    const fuel::Vegetation* vegetation_ = nullptr;
    const fuel::RoadNetwork* roads_ = nullptr;
    static constexpr float kRoadRadius = 1200.0f;
    void buildRoads(simd_float3 player, fuel::MeshRenderer& renderer) const;
    const fuel::Heightfield* terrain_ = nullptr;
    struct Placed {
        uint32_t meshId;
        std::array<float, 16> world;  // row-vector convention
    };
    // Movable props, keyed by placement index. A prop keeps its state until it is far away.
    struct Prop {
        std::array<float, 16> world;  // current transform (row-vector convention)
        float scale[3];
        bool moved = false, body = false;
    };
    std::unordered_map<uint32_t, Prop> props_;
    std::unordered_map<uint32_t, float> meshRadius_;  // collision radius per mesh (-1: none)
    void updateProps(fuel::MeshRenderer& renderer, fuel::DriveWorld& physics, bool dirty);
    std::vector<const fuel::DpcFile*> lookup_;
    std::vector<Placed> placed_;
    std::unordered_map<uint64_t, std::vector<uint32_t>> cells_;
    simd_float3 last_ = {1e30f, 0, 0};
};

// The assembled wheel centres in vehicle space (for makeVehicleSetup).
std::vector<std::array<float, 3>> assembledHubs(const fuel::AssembledVehicle& vehicle);
