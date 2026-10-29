// Drive mode: an assembled vehicle on the terrain, moved by the physics.
#pragma once

#include <simd/simd.h>

#include "core/assembly.hpp"
#include "physics/vehicle_physics.hpp"
#include "renderer.h"

#include <memory>
#include <unordered_map>
#include <vector>

class DriveSession {
public:
    // Builds the physics vehicle for `vehicle`'s parts and places it at (x, z).
    DriveSession(const fuel::AssembledVehicle& vehicle, const fuel::VehicleSetup& setup, const fuel::Heightfield& terrain, double x,
                 double z, float yaw);

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
    // Call every frame; refreshes after the player has moved a little.
    void update(simd_float3 player, fuel::MeshRenderer& renderer, fuel::DriveWorld& physics, bool force = false);

private:
    static constexpr float kCell = 256.0f, kDrawRadius = 1500.0f, kCollideRadius = 250.0f;
    struct Placed {
        uint32_t meshId;
        std::array<float, 16> world;  // row-vector convention
    };
    std::vector<const fuel::DpcFile*> lookup_;
    std::vector<Placed> placed_;
    std::unordered_map<uint64_t, std::vector<uint32_t>> cells_;
    simd_float3 last_ = {1e30f, 0, 0};
};

// The assembled wheel centres in vehicle space (for makeVehicleSetup).
std::vector<std::array<float, 3>> assembledHubs(const fuel::AssembledVehicle& vehicle);
