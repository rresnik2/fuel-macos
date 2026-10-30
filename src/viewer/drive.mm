#import "drive.h"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>

#include "core/resources.hpp"
#include "core/surface.hpp"
#include "core/world.hpp"

#include <unordered_set>

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

VehicleRig::VehicleRig(const fuel::AssembledVehicle& vehicle, const fuel::VehicleSetup& setup, const std::vector<uint32_t>& riders)
    : setup_(setup) {
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
        links_.push_back({-1, matrix_identity_float4x4, true});
    }
    for (const auto& w : setup.wheels) focusHeight_ = std::max(focusHeight_, w.hub[1] + w.radius * 0.6f);
}

void VehicleRig::animate(float dt, const fuel::DriveInput& input, float speedKmh) {
    // The rider leans into turns (more with speed) and forward/back with throttle and brakes.
    const float speed = std::fabs(speedKmh) / 3.6f;
    const float leanTarget = setup_.bike ? 0.0f : input.steer * std::min(speed / 15.0f, 1.0f) * 0.32f;
    const float pitchTarget = (input.throttle - input.brake) * 0.12f;
    const float k = std::min(1.0f, dt * 6.0f);
    lean_ += (leanTarget - lean_) * k;
    pitch_ += (pitchTarget - pitch_) * k;
}

std::vector<simd_float4x4> VehicleRig::partWorlds(const fuel::VehicleState& st) const {
    const simd_float4x4 chassis = poseMatrix(st.chassis);
    // Rider pose offset: rotate about the seat (forward axis for lean, sideways for pitch).
    const float seat = focusHeight_ * 0.55f;
    simd_float4x4 toSeat = matrix_identity_float4x4, fromSeat = matrix_identity_float4x4;
    toSeat.columns[3] = simd_make_float4(0, -seat, 0, 1);
    fromSeat.columns[3] = simd_make_float4(0, seat, 0, 1);
    const simd_float4x4 rot = simd_matrix4x4(simd_mul(simd_quaternion(lean_, simd_make_float3(0, 0, 1)),
                                                      simd_quaternion(pitch_, simd_make_float3(1, 0, 0))));
    const simd_float4x4 riderPose = simd_mul(chassis, simd_mul(fromSeat, simd_mul(rot, toSeat)));
    std::vector<simd_float4x4> out;
    out.reserve(links_.size());
    for (const PartLink& l : links_) {
        if (l.wheel >= 0 && size_t(l.wheel) < st.wheels.size())
            out.push_back(simd_mul(poseMatrix(st.wheels[size_t(l.wheel)]), l.local));
        else
            out.push_back(simd_mul(l.rider ? riderPose : chassis, l.local));
    }
    return out;
}

simd_float3 VehicleRig::focus(const fuel::VehicleState& st) const {
    return simd_mul(poseMatrix(st.chassis), simd_make_float4(0, focusHeight_, 0, 1)).xyz;
}

DriveSession::DriveSession(const fuel::AssembledVehicle& vehicle, const fuel::VehicleSetup& setup, const fuel::Heightfield& terrain,
                           double x, double z, float yaw, const std::vector<uint32_t>& riders)
    : world_(terrain) {
    addVehicle(vehicle, setup, x, z, yaw, riders);
}

int DriveSession::addVehicle(const fuel::AssembledVehicle& vehicle, const fuel::VehicleSetup& setup, double x, double z, float yaw,
                             const std::vector<uint32_t>& riders) {
    rigs_.emplace_back(vehicle, setup, riders);
    inputs_.emplace_back();
    return world_.addVehicle(setup, x, z, yaw);
}

void DriveSession::replacePlayer(const fuel::AssembledVehicle& vehicle, const fuel::VehicleSetup& setup, const std::vector<uint32_t>& riders) {
    const fuel::Pose p = world_.state(0).chassis;
    const float yaw = heading(0);
    rigs_[0] = VehicleRig(vehicle, setup, riders);
    world_.replaceVehicle(0, setup, p.position[0], p.position[2], yaw);
}

void DriveSession::setInput(int i, const fuel::DriveInput& input) {
    if (i > 0 && i < vehicleCount()) inputs_[size_t(i)] = input;
}

void DriveSession::update(float dt, const fuel::DriveInput& input) {
    inputs_[0] = input;
    for (int i = 0; i < vehicleCount(); ++i) world_.setInput(i, inputs_[size_t(i)]);
    world_.update(dt);
    for (int i = 0; i < vehicleCount(); ++i) rigs_[size_t(i)].animate(dt, inputs_[size_t(i)], world_.state(i).speedKmh);
}

float DriveSession::heading(int i) const {
    const auto q = world_.state(i).chassis.rotation;
    const simd_float3 f = simd_act(simd_quaternion(q[0], q[1], q[2], q[3]), simd_make_float3(0, 0, 1));
    return std::atan2(f.x, f.z);
}

namespace {

float smoothstep(float a, float b, float x) {
    const float t = std::clamp((x - a) / (b - a), 0.0f, 1.0f);
    return t * t * (3 - 2 * t);
}

}  // namespace

fuel::OrbitCamera ChaseCamera::update(float dt, const DriveSession& drive, float sceneRadius, float yawOffset, float pitchOffset,
                                      float zoom) {
    const float speed = std::fabs(drive.state().speedKmh);
    const simd_float3 at = drive.position();
    const float behind = drive.heading() + float(M_PI) + yawOffset;
    if (!started_ || simd_distance(at, focus_) > 30.0f) {  // first frame, or a reset/teleport
        started_ = true;
        yaw_ = behind;
        speedFast_ = speedSlow_ = speed;
        surge_ = 0;
        focus_ = at;
    }
    speedFast_ += (speed - speedFast_) * std::min(1.0f, dt * 6.0f);
    speedSlow_ += (speed - speedSlow_) * std::min(1.0f, dt * 1.2f);
    // Accelerating leaves the camera behind for a moment; braking lets it catch up.
    surge_ += (std::clamp((speedFast_ - speedSlow_) * 0.025f, -0.5f, 0.6f) - surge_) * std::min(1.0f, dt * 4.0f);
    const float fast = smoothstep(20.0f, 160.0f, speedSlow_);
    yaw_ += std::remainder(behind - yaw_, 2.0f * float(M_PI)) * std::min(1.0f, dt * (2.5f + 2.5f * fast));
    // Follow the body exactly across the ground, but smooth its bumps.
    focus_.x = at.x, focus_.z = at.z;
    focus_.y += (at.y - focus_.y) * std::min(1.0f, dt * 9.0f);

    const float base = std::clamp(sceneRadius * 2.4f, 3.4f, 10.0f);  // metres behind at rest
    const float back = std::max(1.5f, (base * (1.0f + 0.08f * fast) + surge_) * zoom);
    const float up = std::max(0.3f, base * 0.36f * zoom + pitchOffset * back);
    fuel::OrbitCamera cam;
    cam.yaw = yaw_;
    cam.pitch = std::atan2(up, back);
    cam.distance = std::hypot(back, up) / std::max(sceneRadius, 0.05f);
    cam.fovY = (56.0f + 12.0f * fast) * float(M_PI) / 180.0f;
    cam.aim = base * 0.2f;
    return cam;
}

float ChaseCamera::speedEffect() const { return smoothstep(40.0f, 170.0f, speedSlow_); }

float DustSystem::random() {
    rng_ ^= rng_ << 13, rng_ ^= rng_ >> 17, rng_ ^= rng_ << 5;
    return float(rng_ >> 8) / float(1u << 24);
}

void DustSystem::update(float dt, const DriveSession& drive, simd_float3 eye) {
    using G = fuel::GripCategory;
    // Per grip category: particles per second per wheel at speed, and the dust colour (linear).
    struct Kind {
        float rate;
        simd_float3 colour;
    };
    static const Kind kKinds[int(G::Count)] = {
        {0.6f, {0.30f, 0.28f, 0.26f}},   // tarmac: a little grit
        {34.0f, {0.36f, 0.27f, 0.18f}},  // dirt
        {3.0f, {0.30f, 0.27f, 0.18f}},   // grass
        {40.0f, {0.50f, 0.40f, 0.28f}},  // sand
        {18.0f, {0.80f, 0.83f, 0.88f}},  // snow
        {7.0f, {0.22f, 0.17f, 0.11f}},   // mud
        {10.0f, {0.38f, 0.34f, 0.29f}},  // rock and gravel
    };
    constexpr size_t kMaxParticles = 3000;
    constexpr int kWheelSlots = 8;
    const int vehicles = drive.vehicleCount();
    if (dt > 0) {
        lastChassis_.resize(size_t(vehicles), simd_make_float3(NAN, NAN, NAN));
        carry_.resize(size_t(vehicles) * kWheelSlots, 0.0f);
        for (int v = 0; v < vehicles; ++v) {
            const fuel::VehicleState st = drive.state(v);
            const simd_float3 body = simd_make_float3(float(st.chassis.position[0]), float(st.chassis.position[1]), float(st.chassis.position[2]));
            simd_float3 velocity = (body - lastChassis_[size_t(v)]) / dt;
            lastChassis_[size_t(v)] = body;
            if (!simd_all(velocity == velocity) || simd_length(velocity) > 120.0f) velocity = 0;  // first frame or a teleport
            if (simd_distance(body, eye) > 250.0f) continue;
            const float speed = std::fabs(st.speedKmh);
            const fuel::VehicleSetup& setup = drive.setup(v);
            for (size_t w = 0; w < st.wheels.size() && w < size_t(kWheelSlots); ++w) {
                float& owed = carry_[size_t(v) * kWheelSlots + w];
                const int surface = w < st.wheelSurface.size() ? st.wheelSurface[w] : -1;
                if (surface < 0) {
                    owed = 0;
                    continue;
                }
                const Kind& kind = kKinds[int(fuel::gripCategory(surface))];
                const float skid = w < st.wheelSkid.size() ? st.wheelSkid[w] : st.skid;
                // Wheelspin on the spot (a start line full of revving opponents) throws little.
                const float push = smoothstep(4.0f, 80.0f, speed) + skid * 1.5f * smoothstep(3.0f, 25.0f, speed);
                owed += kind.rate * push * dt;
                const float radius = w < setup.wheels.size() ? setup.wheels[w].radius : 0.3f;
                const auto& wp = st.wheels[w].position;
                const simd_float3 contact = simd_make_float3(float(wp[0]), float(wp[1]) - radius * 0.8f, float(wp[2]));
                for (; owed >= 1.0f; owed -= 1.0f) {
                    if (particles_.size() >= kMaxParticles) particles_.erase(particles_.begin());
                    Particle p;
                    p.position = contact + simd_make_float3(random() - 0.5f, random() * 0.2f, random() - 0.5f) * 0.4f;
                    // Thrown up and out, keeping some of the vehicle's speed; drag soon stops it.
                    p.velocity = velocity * (0.1f + 0.15f * random()) +
                                 simd_make_float3((random() - 0.5f) * 2.4f, 0.6f + random() * 1.6f + skid * 1.5f, (random() - 0.5f) * 2.4f);
                    p.colour = kind.colour * (0.85f + 0.3f * random());
                    p.age = 0;
                    p.life = 1.2f + random() * 1.2f;
                    p.size = 0.35f + random() * 0.3f;
                    p.grow = 1.0f + random() * 1.2f + 0.01f * speed;
                    p.rotation = random() * 6.2832f;
                    p.spin = (random() - 0.5f) * 1.2f;
                    p.opacity = (0.3f + 0.2f * random()) * std::min(1.0f, 0.4f + 0.6f * push);
                    p.seed = random();
                    particles_.push_back(p);
                }
            }
        }
        const float drag = std::exp(-1.8f * dt);
        for (Particle& p : particles_) {
            p.age += dt;
            p.velocity = p.velocity * drag + simd_make_float3(0, 0.3f * dt, 0);  // warm dust drifts up
            p.position += p.velocity * dt;
            p.size += p.grow * dt * std::max(0.2f, 1.0f - p.age / p.life);
            p.rotation += p.spin * dt;
        }
        particles_.erase(std::remove_if(particles_.begin(), particles_.end(), [](const Particle& p) { return p.age >= p.life; }),
                         particles_.end());
    }
    drawn_.clear();
    drawn_.reserve(particles_.size());
    for (const Particle& p : particles_) {
        const float t = p.age / p.life;
        const float a = p.opacity * smoothstep(0.0f, 0.12f, t) * std::pow(1.0f - t, 1.5f);
        drawn_.push_back({simd_make_float4(p.position, p.size), simd_make_float4(p.colour, a), simd_make_float4(p.rotation, p.seed, 0, 0)});
    }
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
            placed_.push_back({pm.meshId, pm.world, pm.fitToGround, pm.groundNormal});
            cells_[cellKey(int64_t(std::floor(x / kCell)), int64_t(std::floor(z / kCell)))].push_back(index);
            addObstacle(pm);
        }
}

void WorldStreamer::setVegetation(const fuel::Vegetation* v, const fuel::Heightfield* terrain) {
    vegetation_ = v, terrain_ = terrain;
    if (terrain == nullptr || seated_) return;
    seated_ = true;
    for (Placed& p : placed_) {
        if (!p.fitToGround) continue;
        const float x = p.world[12], z = p.world[14];
        p.world[13] = terrain->smoothHeightAt(x, z);
        if (!p.groundNormal) continue;
        // Tilt the object's axes by the rotation that takes +y to the terrain normal.
        constexpr float e = 2.0f;
        const float dx = (terrain->smoothHeightAt(x + e, z) - terrain->smoothHeightAt(x - e, z)) / (2 * e);
        const float dz = (terrain->smoothHeightAt(x, z + e) - terrain->smoothHeightAt(x, z - e)) / (2 * e);
        const simd_float3 n = simd_normalize(simd_make_float3(-dx, 1, -dz));
        const simd_float3 axis = simd_cross(simd_make_float3(0, 1, 0), n);
        if (simd_length(axis) < 1e-5f) continue;
        const simd_quatf q = simd_quaternion(std::asin(std::min(1.0f, simd_length(axis))), simd_normalize(axis));
        for (size_t r = 0; r < 3; ++r) {
            const simd_float3 row = simd_act(q, simd_make_float3(p.world[r * 4], p.world[r * 4 + 1], p.world[r * 4 + 2]));
            p.world[r * 4] = row.x, p.world[r * 4 + 1] = row.y, p.world[r * 4 + 2] = row.z;
        }
    }
}

// Bounding circle (ground plane) of a placement, from its mesh's bounding sphere.
void WorldStreamer::addObstacle(const fuel::PlacedMesh& pm) {
    auto sphere = sphereOf_.find(pm.meshId);
    if (sphere == sphereOf_.end()) {
        std::array<float, 4> s{0, 0, 0, -1};
        for (const fuel::DpcFile* f : lookup_)
            if (const fuel::DpcFile::Entry* e = f->find(pm.meshId); e != nullptr && e->header != nullptr && e->classCrc == fuel::cls::Mesh) {
                try {
                    s = fuel::parseMeshLink(f->link(*e)).sphere;
                } catch (const std::exception&) {
                }
                break;
            }
        sphere = sphereOf_.emplace(pm.meshId, s).first;
    }
    const std::array<float, 4>& s = sphere->second;
    if (s[3] <= 0) return;
    const auto& w = pm.world;
    float scale = 0;
    for (int r = 0; r < 3; ++r) scale = std::max(scale, std::hypot(w[size_t(r * 4)], w[size_t(r * 4 + 1)], w[size_t(r * 4 + 2)]));
    const float radius = s[3] * scale;
    if (radius > kMaxObstacle) return;  // terrain-sized pieces would clear whole hillsides
    Obstacle o;
    o.x = s[0] * w[0] + s[1] * w[4] + s[2] * w[8] + w[12];
    o.z = s[0] * w[2] + s[1] * w[6] + s[2] * w[10] + w[14];
    o.r = radius;
    const auto index = uint32_t(obstacles_.size());
    obstacles_.push_back(o);
    const float reach = radius + kPlantClearance;
    for (int64_t cz = int64_t(std::floor((o.z - reach) / kObstacleCell)); cz <= int64_t(std::floor((o.z + reach) / kObstacleCell)); ++cz)
        for (int64_t cx = int64_t(std::floor((o.x - reach) / kObstacleCell)); cx <= int64_t(std::floor((o.x + reach) / kObstacleCell)); ++cx)
            obstacleCells_[cellKey(cx, cz)].push_back(index);
}

bool WorldStreamer::nearObstacle(double x, double z, float clearance) const {
    const auto it = obstacleCells_.find(cellKey(int64_t(std::floor(x / kObstacleCell)), int64_t(std::floor(z / kObstacleCell))));
    if (it == obstacleCells_.end()) return false;
    for (uint32_t i : it->second) {
        const Obstacle& o = obstacles_[i];
        if (std::hypot(x - o.x, z - o.z) < o.r + clearance) return true;
    }
    return false;
}

namespace {

simd_float4x4 rowsToMatrix(const std::array<float, 16>& w) {
    return simd_matrix(simd_make_float4(w[0], w[1], w[2], w[3]), simd_make_float4(w[4], w[5], w[6], w[7]),
                       simd_make_float4(w[8], w[9], w[10], w[11]), simd_make_float4(w[12], w[13], w[14], w[15]));
}

}  // namespace

WorldStreamer::PropStats WorldStreamer::propStats() const {
    PropStats s;
    s.tracked = props_.size();
    for (const auto& [i, p] : props_) {
        s.bodies += p.body, s.moved += p.moved;
        if (!p.moved) continue;
        const float d = std::hypot(p.world[12] - placed_[i].world[12], p.world[13] - placed_[i].world[13], p.world[14] - placed_[i].world[14]);
        if (d > s.maxDisplacement) s.maxDisplacement = d, s.where = simd_make_float3(p.world[12], p.world[13], p.world[14]);
    }
    return s;
}

bool WorldStreamer::nearestProp(simd_float3 from, const std::vector<simd_float3>& skip, simd_float3& out) const {
    float best = 1e30f;
    for (const auto& [i, p] : props_) {
        if (p.moved) continue;
        const simd_float3 c = simd_make_float3(p.world[12], p.world[13], p.world[14]);
        if (std::any_of(skip.begin(), skip.end(), [&](simd_float3 t) { return simd_distance(t, c) < 3.0f; })) continue;
        if (terrain_ && std::fabs(c.y - terrain_->smoothHeightAt(c.x, c.z)) > 1.0f) continue;  // on a deck, not on the ground
        if (simd_distance(c, from) < best) best = simd_distance(c, from), out = c;
    }
    return best < 1e30f;
}

// Cheap per-frame path: pull the poses of props that moved and redraw the (small) prop list.
void WorldStreamer::updateProps(fuel::MeshRenderer& renderer, fuel::DriveWorld& physics, bool dirty) {
    for (const fuel::PropPose& pp : physics.changedProps()) {
        auto it = props_.find(pp.id);
        if (it == props_.end()) continue;
        Prop& p = it->second;
        if (pp.displacement > 0.05f || pp.awake) p.moved = true;
        const simd_quatf q = simd_quaternion(pp.pose.rotation[0], pp.pose.rotation[1], pp.pose.rotation[2], pp.pose.rotation[3]);
        for (int r = 0; r < 3; ++r) {
            const simd_float3 axis = simd_act(q, simd_make_float3(r == 0, r == 1, r == 2)) * p.scale[r];
            p.world[size_t(r * 4)] = axis.x, p.world[size_t(r * 4 + 1)] = axis.y, p.world[size_t(r * 4 + 2)] = axis.z;
        }
        p.world[12] = float(pp.pose.position[0]), p.world[13] = float(pp.pose.position[1]), p.world[14] = float(pp.pose.position[2]);
        dirty = true;
    }
    if (!dirty) return;
    std::vector<fuel::MeshInstance> list;
    list.reserve(props_.size());
    for (const auto& [i, p] : props_) list.push_back({placed_[i].meshId, rowsToMatrix(p.world)});
    renderer.setPropInstances(list);
}

namespace {

// FUEL's decor quadtree (FUN_007df0e0, FUN_007dfdc0, FUN_007de770): 12288 m root
// chunks centred on multiples of 12288 m; a chunk splits into four (centres at
// +-1/4 of its size) while the camera is within 2.0625 x its size of its centre,
// measured on the ground plane; chunks of 24 m never split. Returns the size of
// the leaf chunk holding (px, pz), which picks how the engine draws decor there.
double decorChunkSize(double px, double pz, double camX, double camZ) {
    constexpr double kRoot = 12288.0, kSplit = 2.0625, kMinChunk = 24.01;
    double size = kRoot;
    double ox = std::round(px / kRoot) * kRoot, oz = std::round(pz / kRoot) * kRoot;
    while (size > kMinChunk) {
        const double r = size * kSplit;
        if ((ox - camX) * (ox - camX) + (oz - camZ) * (oz - camZ) >= r * r) break;
        ox += (px >= ox ? 0.25 : -0.25) * size;
        oz += (pz >= oz ? 0.25 : -0.25) * size;
        size *= 0.5;
    }
    return size;
}

}  // namespace

WorldStreamer::Background WorldStreamer::buildBackground(simd_float3 player) const {
    Background bg;
    if (vegetation_ && terrain_) {
        std::vector<fuel::Plant> scattered, plants;
        const fuel::BlockedQuery blocked = [this](double x, double z) { return nearObstacle(x, z, kPlantClearance); };
        fuel::scatterPlants(*vegetation_, *terrain_, roads_, blocked, player.x, player.z, kPlantRadius, scattered);
        // Trees are solid near the player: a trunk cylinder sized with the tree.
        std::unordered_map<uint32_t, uint32_t> card;  // full mesh -> distant card (0 = none)
        std::unordered_set<uint32_t> trees;
        for (const fuel::Species& sp : vegetation_->trees) trees.insert(sp.full), card[sp.full] = sp.impostor;
        for (const fuel::Species& sp : vegetation_->bushes) card.emplace(sp.full, sp.impostor);
        for (const fuel::Plant& p : scattered) {
            if (!trees.count(p.meshId) || std::hypot(p.position[0] - player.x, p.position[2] - player.z) > kCollideRadius) continue;
            const float size = std::hypot(p.axes[1][0], p.axes[1][1], p.axes[1][2]);
            const double r = std::clamp(0.07f * size, 0.2f, 0.5f);
            bg.trunks.push_back({r, 0, 0, 0, 0, 5.0, 0, 0, 0, 0, r, 0, p.position[0], p.position[1] - 0.5, p.position[2], 1});
        }
        // Mesh or card by the leaf chunk under each plant (decoded from FUEL.exe,
        // default "GenWorld" settings at 0x74d5b0): trees only in chunks < 600 m,
        // full meshes in chunks < 60 m (W+0xa68), cards beyond; bushes only in
        // chunks < 100 m, full meshes below 60 m (W+0xa64). In 30..60 m chunks the
        // engine also queues the card next to the full mesh; the port draws the mesh.
        // Port limitation: the renderer does not draw the tree cards correctly yet
        // (they come out invisible or as dark rectangles; bush cards are fine), so
        // trees keep their full mesh out to the 192 m chunks (~800 m) and the 384 m
        // chunks, where FUEL draws only cards, stay empty.
        for (fuel::Plant& p : scattered) {
            const bool tree = trees.count(p.meshId) != 0;
            const double chunk = decorChunkSize(p.position[0], p.position[2], player.x, player.z);
            if (chunk >= (tree ? kTreeCardStandIn : kBushChunkMax)) continue;
            if (!tree && chunk >= kFullMeshChunk) {
                const auto it = card.find(p.meshId);
                if (it == card.end() || it->second == 0) continue;
                p.meshId = it->second;
            }
            plants.push_back(p);
        }
        if (std::getenv("FUEL_STREAM_TRACE"))
            std::printf("plants: %zu scattered, %zu drawn (%zu trunks)\n", scattered.size(), plants.size(), bg.trunks.size());
        fuel::scatterClutter(*vegetation_, *terrain_, surfaces_, player.x, player.z, kClutterRadius, plants);
        bg.plants.reserve(plants.size());
        for (const fuel::Plant& p : plants) {
            auto col = [&](size_t i) { return simd_make_float4(p.axes[i][0], p.axes[i][1], p.axes[i][2], 0); };
            bg.plants.push_back({p.meshId, simd_matrix(col(0), col(1), col(2), simd_make_float4(p.position[0], p.position[1], p.position[2], 1))});
        }
    }
    if (roads_ && terrain_) {
        bg.roads = buildRoads(player);
        bg.haveRoads = true;
    }
    return bg;
}

// A unit cylinder (radius 1, y from 0 to 1) for tree trunks.
constexpr uint32_t kTrunkShape = 0x7E0E0001u;

void WorldStreamer::applySolids(fuel::DriveWorld& physics) {
    if (!physics.hasStaticShape(kTrunkShape)) {
        std::vector<float> pos;
        std::vector<uint32_t> idx;
        constexpr int kSides = 8;
        for (int k = 0; k < kSides; ++k) {
            const float a = float(k) / kSides * 6.2831853f;
            pos.insert(pos.end(), {std::cos(a), 0.0f, std::sin(a), std::cos(a), 1.0f, std::sin(a)});
            const uint32_t i = uint32_t(k * 2), j = uint32_t(((k + 1) % kSides) * 2);
            idx.insert(idx.end(), {i, i + 1, j, j, i + 1, j + 1});
        }
        physics.addStaticShape(kTrunkShape, pos, idx);
    }
    std::vector<std::pair<uint32_t, std::array<double, 16>>> solid = cellSolid_;
    for (const auto& m : trunks_) solid.push_back({kTrunkShape, m});
    if (std::getenv("FUEL_STREAM_TRACE")) std::printf("solids: %zu placed, %zu trunks\n", cellSolid_.size(), trunks_.size());
    physics.setStaticObjects(solid);
}

void WorldStreamer::applyBackground(Background&& bg, fuel::MeshRenderer& renderer, fuel::DriveWorld& physics) {
    trunks_ = std::move(bg.trunks);
    applySolids(physics);
    plantDraw_ = std::move(bg.plants);
    std::vector<fuel::MeshInstance> draw = cellDraw_;
    draw.insert(draw.end(), plantDraw_.begin(), plantDraw_.end());
    renderer.setWorldInstances(draw);
    if (bg.haveRoads) renderer.setRoads(bg.roads.vertices, bg.roads.indices, bg.roads.batches);
}

void WorldStreamer::pollBackground(fuel::MeshRenderer& renderer, fuel::DriveWorld& physics) {
    if (!job_.valid() || job_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) return;
    applyBackground(job_.get(), renderer, physics);
    if (rerun_) {  // catch up with where the player is now
        rerun_ = false;
        job_ = std::async(std::launch::async, [this, at = last_] { return buildBackground(at); });
    }
}

void WorldStreamer::update(simd_float3 player, fuel::MeshRenderer& renderer, fuel::DriveWorld& physics, bool force) {
    pollBackground(renderer, physics);
    if (!force && simd_distance(player, last_) < 40.0f) {
        updateProps(renderer, physics, false);
        return;
    }
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
                if (props_.count(i)) continue;  // drawn from the prop list
                if (d <= kPropRadius) {
                    // Small objects become movable props (cached radius per mesh).
                    auto mr = meshRadius_.find(p.meshId);
                    if (mr == meshRadius_.end()) {
                        fuel::CollisionMesh cm;
                        try {
                            cm = fuel::loadCollisionMesh(lookup_, p.meshId);
                        } catch (const std::exception&) {
                        }
                        float radius = cm.indices.empty() ? -1.0f : cm.radius;
                        if (radius >= 0.4f && radius < 3.0f) {
                            physics.addPropShape(p.meshId, cm.positions);
                            if (!physics.hasPropShape(p.meshId)) radius = -1.0f;
                        }
                        mr = meshRadius_.emplace(p.meshId, radius).first;
                    }
                    if (mr->second >= 0.4f && mr->second < 3.0f) {
                        Prop prop;
                        prop.world = p.world;
                        for (int r = 0; r < 3; ++r) prop.scale[r] = std::max(1e-4f, std::hypot(p.world[size_t(r * 4)], p.world[size_t(r * 4 + 1)], p.world[size_t(r * 4 + 2)]));
                        props_[i] = prop;
                        continue;
                    }
                }
                simd_float4 cols[4];
                for (size_t c = 0; c < 4; ++c) cols[c] = simd_make_float4(p.world[c * 4], p.world[c * 4 + 1], p.world[c * 4 + 2], p.world[c * 4 + 3]);
                draw.push_back({p.meshId, simd_matrix(cols[0], cols[1], cols[2], cols[3])});
                if (d > kCollideRadius) continue;
                if (auto mr = meshRadius_.find(p.meshId); mr != meshRadius_.end() && mr->second >= 0.4f && mr->second < 3.0f) continue;  // a prop
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
    cellDraw_ = std::move(draw);
    if (force) {  // teleports and the first frame: build everything now
        if (job_.valid()) (void)job_.get();
        rerun_ = false;
        applyBackground(buildBackground(player), renderer, physics);
    } else if (job_.valid()) {
        rerun_ = true;
    } else {
        job_ = std::async(std::launch::async, [this, at = player] { return buildBackground(at); });
    }
    // Props: simulate those near the player (and not knocked far off), forget distant ones.
    for (auto it = props_.begin(); it != props_.end();) {
        Prop& p = it->second;
        const float d = std::hypot(p.world[12] - player.x, p.world[14] - player.z);
        const bool keep = d <= kPropForget && (p.moved || d <= kPropRadius * 1.25f);
        const bool want = keep && d <= kPropRadius * 1.25f;
        if (!keep) {
            if (p.body) physics.removeProp(it->first);
            it = props_.erase(it);
            continue;
        }
        if (want && !p.body) {
            std::array<double, 16> m;
            for (size_t k = 0; k < 16; ++k) m[k] = p.world[k];
            p.body = physics.addProp(it->first, placed_[it->first].meshId, m);
        } else if (!want && p.body) {
            physics.removeProp(it->first);
            p.body = false;
        }
        ++it;
    }
    updateProps(renderer, physics, true);
    cellSolid_ = std::move(solid);
    applySolids(physics);
}

WorldStreamer::RoadMesh WorldStreamer::buildRoads(simd_float3 player) const {
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
    RoadMesh out;
    for (auto& [type, vi] : byType) {  // narrow, faint types first so tarmac draws on top
        const auto base = uint32_t(out.vertices.size());
        const auto first = uint32_t(out.indices.size());
        out.vertices.insert(out.vertices.end(), vi.first.begin(), vi.first.end());
        for (uint32_t i : vi.second) out.indices.push_back(base + i);
        const bool tarmac = (type >= 10 && type <= 12) || type == 15 || type == 18 || type == 19;
        out.batches.push_back({roads_->styles[type].texture, first, uint32_t(vi.second.size()), !tarmac});
    }
    return out;
}
