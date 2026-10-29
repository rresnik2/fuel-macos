#import "drive.h"

#include <algorithm>
#include <cmath>

namespace {

simd_float4x4 rowsToSimd(const fuel::Mat4& m) {
    return simd_matrix(simd_make_float4(float(m[0]), float(m[1]), float(m[2]), float(m[3])),
                       simd_make_float4(float(m[4]), float(m[5]), float(m[6]), float(m[7])),
                       simd_make_float4(float(m[8]), float(m[9]), float(m[10]), float(m[11])),
                       simd_make_float4(float(m[12]), float(m[13]), float(m[14]), float(m[15])));
}

simd_float4x4 poseMatrix(const fuel::Pose& p) {
    simd_float4x4 m = simd_matrix4x4(simd_quaternion(p.rotation[0], p.rotation[1], p.rotation[2], p.rotation[3]));
    m.columns[3] = simd_make_float4(float(p.position[0]), float(p.position[1]), float(p.position[2]), 1);
    return m;
}

bool isWheel(const fuel::PlacedPart& p) {
    return p.placement == fuel::Placement::Wheel || p.placement == fuel::Placement::WheelFlipped;
}

}  // namespace

std::vector<std::array<float, 3>> assembledHubs(const fuel::AssembledVehicle& vehicle) {
    std::vector<std::array<float, 3>> hubs;
    for (const fuel::PlacedPart& p : vehicle.parts)
        if (p.drawn() && isWheel(p)) hubs.push_back({float(p.world[12]), float(p.world[13]), float(p.world[14])});
    return hubs;
}

DriveSession::DriveSession(const fuel::AssembledVehicle& vehicle, const fuel::VehicleSetup& setup, const fuel::Heightfield& terrain,
                           double x, double z, float yaw)
    : world_(terrain) {
    for (const fuel::PlacedPart& p : vehicle.parts) {
        if (!p.drawn()) continue;
        const simd_float4x4 m = rowsToSimd(p.world);
        PartLink link{-1, m};
        if (isWheel(p)) {
            // Follow the physics wheel whose hub is nearest; keep the part's own
            // orientation (wheels shared left/right are flipped 180 degrees).
            float best = 1e9f;
            for (size_t i = 0; i < setup.wheels.size(); ++i) {
                const auto& h = setup.wheels[i].hub;
                const float d = simd_distance(simd_make_float3(h[0], h[1], h[2]), m.columns[3].xyz);
                if (d < best) best = d, link.wheel = int(i);
            }
            link.local.columns[3] = simd_make_float4(0, 0, 0, 1);
        }
        parts_.push_back({p.meshId, m});
        links_.push_back(link);
    }
    for (const auto& w : setup.wheels) focusHeight_ = std::max(focusHeight_, w.hub[1] + w.radius * 0.6f);
    world_.spawn(setup, x, z, yaw);
}

std::vector<simd_float4x4> DriveSession::partWorlds() const {
    const fuel::VehicleState st = world_.state();
    const simd_float4x4 chassis = poseMatrix(st.chassis);
    std::vector<simd_float4x4> out;
    out.reserve(links_.size());
    for (const PartLink& l : links_) {
        if (l.wheel >= 0 && size_t(l.wheel) < st.wheels.size())
            out.push_back(simd_mul(poseMatrix(st.wheels[size_t(l.wheel)]), l.local));
        else
            out.push_back(simd_mul(chassis, l.local));
    }
    return out;
}

simd_float3 DriveSession::position() const {
    const fuel::VehicleState st = world_.state();
    return simd_mul(poseMatrix(st.chassis), simd_make_float4(0, focusHeight_, 0, 1)).xyz;
}

float DriveSession::heading() const {
    const auto q = world_.state().chassis.rotation;
    const simd_float3 f = simd_act(simd_quaternion(q[0], q[1], q[2], q[3]), simd_make_float3(0, 0, 1));
    return std::atan2(f.x, f.z);
}
