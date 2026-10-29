// Drive mode: an assembled vehicle on the terrain, moved by the physics.
#pragma once

#include <simd/simd.h>

#include "core/assembly.hpp"
#include "physics/vehicle_physics.hpp"
#include "renderer.h"

#include <memory>
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
    // World matrices for the parts, in parts() order.
    std::vector<simd_float4x4> partWorlds() const;
    fuel::VehicleState state() const { return world_.state(); }
    simd_float3 position() const;  // a point on the body, for the camera
    float heading() const;  // radians; 0 = +z

private:
    struct PartLink {
        int wheel = -1;              // physics wheel driving this part, -1 = chassis
        simd_float4x4 local;         // vehicle space (chassis parts) or wheel-model space (rotation only)
    };
    fuel::DriveWorld world_;
    std::vector<fuel::MeshInstance> parts_;
    std::vector<PartLink> links_;
    float focusHeight_ = 0.5f;  // where the camera looks, above the vehicle origin
};

// The assembled wheel centres in vehicle space (for makeVehicleSetup).
std::vector<std::array<float, 3>> assembledHubs(const fuel::AssembledVehicle& vehicle);
