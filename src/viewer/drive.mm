#import "drive.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>

#include "core/world.hpp"

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
                           double x, double z, float yaw, const std::vector<uint32_t>& riders)
    : world_(terrain), setup_(setup) {
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
    for (uint32_t id : riders) {
        parts_.push_back({id, matrix_identity_float4x4});
        links_.push_back({-1, matrix_identity_float4x4});
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

namespace {

uint64_t cellKey(int64_t cx, int64_t cz) { return (uint64_t(uint32_t(int32_t(cx))) << 32) | uint32_t(int32_t(cz)); }

}  // namespace

WorldStreamer::WorldStreamer(const std::vector<const fuel::DpcFile*>& subWorlds, const std::vector<const fuel::DpcFile*>& lookup)
    : lookup_(lookup) {
    for (const fuel::DpcFile* sub : subWorlds)
        for (const fuel::PlacedMesh& pm : fuel::collectPlacedMeshes(*sub, lookup)) {
            const float x = pm.world[12], z = pm.world[14];
            if (std::fabs(x) < 50 && std::fabs(z) < 50) continue;  // stray objects at the origin
            const auto index = uint32_t(placed_.size());
            placed_.push_back({pm.meshId, pm.world});
            cells_[cellKey(int64_t(std::floor(x / kCell)), int64_t(std::floor(z / kCell)))].push_back(index);
        }
}

void WorldStreamer::update(simd_float3 player, fuel::MeshRenderer& renderer, fuel::DriveWorld& physics, bool force) {
    if (!force && simd_distance(player, last_) < 40.0f) return;
    last_ = player;
    std::vector<fuel::MeshInstance> draw;
    std::vector<std::pair<uint32_t, std::array<double, 16>>> solid;
    const int64_t r = int64_t(std::ceil(kDrawRadius / kCell));
    const int64_t cx = int64_t(std::floor(player.x / kCell)), cz = int64_t(std::floor(player.z / kCell));
    for (int64_t dz = -r; dz <= r; ++dz)
        for (int64_t dx = -r; dx <= r; ++dx) {
            auto it = cells_.find(cellKey(cx + dx, cz + dz));
            if (it == cells_.end()) continue;
            for (uint32_t i : it->second) {
                const Placed& p = placed_[i];
                const float d = std::hypot(p.world[12] - player.x, p.world[14] - player.z);
                if (d > kDrawRadius) continue;
                simd_float4 cols[4];
                for (size_t c = 0; c < 4; ++c) cols[c] = simd_make_float4(p.world[c * 4], p.world[c * 4 + 1], p.world[c * 4 + 2], p.world[c * 4 + 3]);
                draw.push_back({p.meshId, simd_matrix(cols[0], cols[1], cols[2], cols[3])});
                if (d > kCollideRadius) continue;
                if (!physics.hasStaticShape(p.meshId)) {
                    fuel::CollisionMesh cm;
                    try {
                        cm = fuel::loadCollisionMesh(lookup_, p.meshId);
                    } catch (const std::exception&) {
                    }
                    // Only big things are solid (buildings, large rocks, structures). FUEL's
                    // course props (barriers, banners, signs, fences) get knocked aside, so
                    // as static walls they would block the courses; they stay passable
                    // until they become movable bodies.
                    if (cm.radius < 3.0f) cm.indices.clear();
                    physics.addStaticShape(p.meshId, cm.positions, cm.indices);
                }
                std::array<double, 16> m;
                for (size_t k = 0; k < 16; ++k) m[k] = p.world[k];
                solid.push_back({p.meshId, m});
            }
        }
    if (vegetation_ && terrain_) {
        std::vector<fuel::Plant> plants;
        fuel::scatterPlants(*vegetation_, *terrain_, player.x, player.z, kPlantRadius, plants);
        fuel::scatterPlants(*vegetation_, *terrain_, player.x, player.z, kImpostorRadius, plants, true, kPlantRadius);
        for (const fuel::Plant& p : plants) {
            const float c = std::cos(p.yaw) * p.scale, s = std::sin(p.yaw) * p.scale;
            draw.push_back({p.meshId, simd_matrix(simd_make_float4(c, 0, -s, 0), simd_make_float4(0, p.scale, 0, 0), simd_make_float4(s, 0, c, 0),
                                                  simd_make_float4(p.position[0], p.position[1], p.position[2], 1))});
        }
    }
    renderer.setWorldInstances(draw);
    if (roads_ && terrain_) buildRoads(player, renderer);
    physics.setStaticObjects(solid);
}

void WorldStreamer::buildRoads(simd_float3 player, fuel::MeshRenderer& renderer) const {
    // Nearby lines from the road grid.
    std::set<uint32_t> lines;
    const int32_t r = int32_t(std::ceil(kRoadRadius / fuel::RoadNetwork::kCell));
    const int32_t cx = int32_t(std::floor(player.x / fuel::RoadNetwork::kCell)), cz = int32_t(std::floor(player.z / fuel::RoadNetwork::kCell));
    for (int32_t dz = -r; dz <= r; ++dz)
        for (int32_t dx = -r; dx <= r; ++dx)
            if (auto it = roads_->cells.find(fuel::RoadNetwork::cellKey(cx + dx, cz + dz)); it != roads_->cells.end())
                lines.insert(it->second.begin(), it->second.end());
    // One vertex/index list per road type, then batches by type.
    std::map<uint8_t, std::pair<std::vector<fuel::MeshRenderer::RoadVertex>, std::vector<uint32_t>>> byType;
    for (uint32_t li : lines) {
        const fuel::RoadNetwork::Line& line = roads_->lines[li];
        const fuel::RoadStyle& st = roads_->styles[line.type];
        auto& [verts, idx] = byType[line.type];
        // Resample to at most 6 m between points so the ribbon follows the ground.
        std::vector<simd_float2> pts;
        for (size_t k = 0; k + 1 < line.points.size(); ++k) {
            const simd_float2 a = simd_make_float2(line.points[k][0], line.points[k][1]);
            const simd_float2 b = simd_make_float2(line.points[k + 1][0], line.points[k + 1][1]);
            const int steps = std::max(1, int(std::ceil(simd_distance(a, b) / 6.0f)));
            for (int s = 0; s < steps; ++s) pts.push_back(a + (b - a) * (float(s) / float(steps)));
        }
        pts.push_back(simd_make_float2(line.points.back()[0], line.points.back()[1]));
        // Lines are stored in pieces; extend both ends by half the width so joints overlap.
        if (pts.size() >= 2) {
            const float ext = st.width * 0.5f;
            const simd_float2 d0 = pts[0] - pts[1], d1 = pts.back() - pts[pts.size() - 2];
            if (simd_length(d0) > 1e-3f) pts.front() += simd_normalize(d0) * ext;
            if (simd_length(d1) > 1e-3f) pts.back() += simd_normalize(d1) * ext;
        }
        float along = 0;
        for (size_t k = 0; k < pts.size(); ++k) {
            const simd_float2 prev = pts[k > 0 ? k - 1 : k], next = pts[k + 1 < pts.size() ? k + 1 : k];
            simd_float2 dir = next - prev;
            if (simd_length(dir) < 1e-4f) dir = simd_make_float2(1, 0);
            dir = simd_normalize(dir);
            // Miter at corners: widen the offset by 1/cos(half the turn) so the road keeps its width.
            float miter = 1.0f;
            if (k > 0 && k + 1 < pts.size() && simd_length(pts[k] - prev) > 1e-3f) {
                const simd_float2 d1 = simd_normalize(pts[k] - prev);
                miter = 1.0f / std::max(0.35f, simd_dot(simd_make_float2(-dir.y, dir.x), simd_make_float2(-d1.y, d1.x)));
            }
            const simd_float2 side = simd_make_float2(-dir.y, dir.x) * (st.width * 0.5f * miter);
            if (k > 0) along += simd_distance(pts[k], pts[k - 1]);
            const auto base = uint32_t(verts.size());
            for (int e = 0; e < 2; ++e) {
                const simd_float2 p = pts[k] + (e == 0 ? -side : side);
                const float y = terrain_->smoothHeightAt(p.x, p.y) + 0.35f;
                verts.push_back({simd_make_float4(p.x, y, p.y, st.opacity), simd_make_float4(float(e), along / std::max(st.width, 4.0f), 0, 0)});
            }
            if (k > 0) idx.insert(idx.end(), {base - 2, base - 1, base, base - 1, base + 1, base});
        }
        // Round caps at both ends hide the notches where pieces meet at an angle.
        for (const auto& end : {line.points.front(), line.points.back()}) {
            const float y = terrain_->smoothHeightAt(end[0], end[1]) + 0.2f;
            const auto centre = uint32_t(verts.size());
            verts.push_back({simd_make_float4(end[0], y, end[1], st.opacity), simd_make_float4(0.5f, 0, 0, 0)});
            constexpr int kSegments = 10;
            for (int s = 0; s <= kSegments; ++s) {
                const float a = float(s) / kSegments * 6.2831853f;
                const float px = end[0] + std::cos(a) * st.width * 0.5f, pz = end[1] + std::sin(a) * st.width * 0.5f;
                verts.push_back({simd_make_float4(px, terrain_->smoothHeightAt(px, pz) + 0.2f, pz, st.opacity),
                                 simd_make_float4(0.08f, std::sin(a) * 0.5f, 0, 0)});
                if (s > 0) idx.insert(idx.end(), {centre, centre + uint32_t(s), centre + uint32_t(s) + 1});
            }
        }
    }
    std::vector<fuel::MeshRenderer::RoadVertex> vertices;
    std::vector<uint32_t> indices;
    std::vector<fuel::MeshRenderer::RoadBatch> batches;
    for (auto& [type, vi] : byType) {  // narrow, faint types first so tarmac draws on top
        const auto base = uint32_t(vertices.size());
        const auto first = uint32_t(indices.size());
        vertices.insert(vertices.end(), vi.first.begin(), vi.first.end());
        for (uint32_t i : vi.second) indices.push_back(base + i);
        const bool tarmac = (type >= 10 && type <= 12) || type == 15 || type == 18 || type == 19;
        batches.push_back({roads_->styles[type].texture, first, uint32_t(vi.second.size()), !tarmac});
    }
    renderer.setRoads(vertices, indices, batches);
}
