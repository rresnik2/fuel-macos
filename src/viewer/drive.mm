#import "drive.h"
#include "rider_view.h"

#include <filesystem>

#include "core/weather.hpp"

#include <algorithm>
#include <cmath>
#include <map>
#include <set>
#include <strings.h>
#include <thread>

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
        links_.push_back({-1, matrix_identity_float4x4, true, id});
    }
    for (const auto& w : setup.wheels) focusHeight_ = std::max(focusHeight_, w.hub[1] + w.radius * 0.6f);
}

RiderView* gRiderView = nullptr;

void VehicleRig::animate(float dt, const fuel::DriveInput& input, const fuel::VehicleState& st, const fuel::fv::FuelVehicle* model, bool player,
                         int raceState, const fuel::Heightfield& terrain) {
    if (gRiderView == nullptr || model == nullptr || dt <= 0) return;
    for (const PartLink& l : links_) {
        if (!l.rider) continue;
        fuel::RiderAnimator* a = gRiderView->animator(l.key);
        if (a == nullptr) continue;
        fuel::RiderDrive d;
        d.dt = dt;
        d.q = st.chassis.rotation;
        d.pos = st.chassis.position;
        const fuel::fv::Vec3 v = model->body().avgV;
        d.vel = {v.x, v.y, v.z};
        d.steer = input.steer;
        d.stickZ = 0;  // the port's input has no fore/aft lean axis
        d.airtime = model->airtime();
        d.heightAbove = model->heightAboveGround();
        d.gear = model->gear();
        d.sliding = model->slidingMajority();
        d.bump = model->body().impactSpeed;
        d.wheelBumps = model->meanWheelBumpsRot();
        d.accel = input.throttle > 0;
        d.brake = input.brake > 0;
        d.player = player;
        d.ejected = model->riderEjected();
        d.raceState = raceState;
        d.ground = [&terrain](double x, double z) { return terrain.smoothHeightAt(x, z); };
        a->update(d);
    }
}

void VehicleRig::uploadRider() const {
    if (gRiderView == nullptr) return;
    for (const PartLink& l : links_)
        if (l.rider) gRiderView->upload(l.key);
}

std::vector<simd_float4x4> VehicleRig::partWorlds(const fuel::VehicleState& st) const {
    const simd_float4x4 chassis = poseMatrix(st.chassis);
    std::vector<simd_float4x4> out;
    out.reserve(links_.size());
    for (const PartLink& l : links_) {
        if (l.wheel >= 0 && size_t(l.wheel) < st.wheels.size()) {
            out.push_back(simd_mul(poseMatrix(st.wheels[size_t(l.wheel)]), l.local));
        } else if (l.rider && gRiderView != nullptr && gRiderView->animator(l.key) && gRiderView->animator(l.key)->detached()) {
            // the ejected rider: its bones are world-space offsets from where it left the vehicle
            const auto o = gRiderView->animator(l.key)->worldOrigin();
            simd_float4x4 m = matrix_identity_float4x4;
            m.columns[3] = simd_make_float4(float(o[0]), float(o[1]), float(o[2]), 1);
            out.push_back(m);
        } else {
            out.push_back(simd_mul(chassis, l.local));
        }
    }
    return out;
}

simd_float3 VehicleRig::focus(const fuel::VehicleState& st) const {
    return simd_mul(poseMatrix(st.chassis), simd_make_float4(0, focusHeight_, 0, 1)).xyz;
}

DriveSession::DriveSession(const fuel::AssembledVehicle& vehicle, const fuel::VehicleSetup& setup, const fuel::Heightfield& terrain,
                           double x, double z, float yaw, const std::vector<uint32_t>& riders)
    : terrain_(terrain), world_(terrain) {
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
    for (int i = 0; i < vehicleCount(); ++i)
        rigs_[size_t(i)].animate(dt, inputs_[size_t(i)], world_.state(i), world_.model(i), i == 0, i == 0 ? raceState_ : 0, terrain_);
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

fuel::OrbitCamera ChaseCamera::update(float dt, const DriveSession& drive, const fuel::cam::CameraControls& in, fuel::cam::Override ov) {
    namespace cam = fuel::cam;
    const fuel::fv::FuelVehicle* m = drive.physics().model(0);
    const fuel::VehicleState st = drive.state(0);
    cam::VehicleInput vi;
    const std::vector<float>& P = drive.setup(0).params;
    if (m != nullptr && !P.empty() && dt > 0) {
        vi.valid = true;
        vi.q = {st.chassis.rotation[0], st.chassis.rotation[1], st.chassis.rotation[2], st.chassis.rotation[3]};
        vi.ox = st.chassis.position[0], vi.oy = st.chassis.position[1], vi.oz = st.chassis.position[2];
        vi.vel = m->body().avgV;
        vi.angVel = m->body().avgW;
        if (havePrev_)
            vi.fdVel = {float((vi.ox - prev_[0]) / dt), float((vi.oy - prev_[1]) / dt), float((vi.oz - prev_[2]) / dt)};
        else
            vi.fdVel = vi.vel;
        prev_[0] = vi.ox, prev_[1] = vi.oy, prev_[2] = vi.oz, havePrev_ = true;
        vi.gear = m->gear();
        vi.sliding = m->slidingMajority();
        vi.airtime = m->airtime();
        vi.heightAbove = m->heightAboveGround();
        vi.wreck = m->wreckState();
        vi.anyWheelLoaded = m->anyLoaded();
        vi.maxBumps = m->maxWheelBumps();
        const float top = P.size() > 619 ? P[619] : 0.0f;
        vi.speedFrac = top > 0 ? std::clamp(fuel::fv::len(m->body().S.v) / (top * 0.869565f * 0.277778f), 0.0f, 1.0f) : 0.0f;
        vi.P = &P;
    }
    cam::WorldQueries wq;
    const fuel::Heightfield& terrain = drive.terrain();
    const fuel::DriveWorld& world = drive.physics();
    wq.ground = [&terrain, &world](double x, double y, double z, bool detail) {  // Cam_GroundHeight 0x402620
        float h = std::max(detail ? terrain.smoothHeightAt(x, z) : terrain.baseHeightAt(x, z), -10000.0f);
        h += std::max(0.0f, world.waterDepthAt(x, z));  // the water layer (U-B5): the lake surface
        const float t = world.rayFraction({x, y, z}, {x, y - 100.0, z});
        if (t < 1.0f) h = std::max(h, float(y - 100.0 * t));
        return h;
    };
    wq.overlap = [&world, &drive](double x, double y, double z, float r, std::vector<cam::WorldQueries::Contact>& out) {  // 0x63a0b0
        std::vector<fuel::DriveWorld::SphereContact> hits;
        world.overlapStatics({x, y, z}, r, hits);
        for (const auto& h : hits) {
            if (h.volume <= 1.0f) continue;  // 0x4063a0: objects of more than 1 m^3
            cam::WorldQueries::Contact c;
            c.p = {float(h.p[0] - x), float(h.p[1] - y), float(h.p[2] - z)};
            c.n = {h.n[0], h.n[1], h.n[2]};
            c.d = fuel::fv::len(c.p);
            out.push_back(c);
        }
        // the other vehicles (I: the query's mask includes vehicles; 0x402450 excludes the player's own), 1 m spheres
        for (int i = 1; i < drive.vehicleCount(); ++i) {
            const simd_float3 p = drive.position(i);
            const fuel::fv::Vec3 d{float(p.x - x), float(p.y - y), float(p.z - z)};
            const float l = fuel::fv::len(d), rr = 1.0f;
            if (l >= r + rr || l < 1e-4f) continue;
            cam::WorldQueries::Contact c;
            c.n = d * (-1.0f / l);
            c.p = d + c.n * rr;
            c.d = fuel::fv::len(c.p);
            out.push_back(c);
        }
    };
    pose_ = cam_.update(dt, vi, wq, in, ov);
    focus_ = drive.position(0);
    const float lo = P.size() > 574 ? P[572] : 0, hi = P.size() > 574 ? P[573] : 1, mx = P.size() > 574 ? P[574] : 0;
    blur_ = std::clamp(mx > 0 ? pose_.blur / mx : (hi > lo ? (fuel::fv::len(vi.vel) - lo) / (hi - lo) : 0.0f), 0.0f, 1.0f);
    fuel::OrbitCamera c;
    c.explicitPose = vi.valid;
    c.eye = simd_make_float3(float(pose_.ex), float(pose_.ey), float(pose_.ez));
    c.target = simd_make_float3(float(pose_.tx), float(pose_.ty), float(pose_.tz));
    c.roll = pose_.roll;
    c.zNear = pose_.zNear;
    c.fovH43 = pose_.fovDeg * 0.017453292f;
    c.fovY = 1.0f;
    return c;
}


namespace {
std::string gEffectsDataDir;
std::unique_ptr<fuel::sfx::Tables> gEffectsTables;
}  // namespace

std::function<std::array<float, 4>(double x, double z)> DustSystem::groundColour;

void DustSystem::setDataDir(const std::string& dir) { gEffectsDataDir = dir; }

void DustSystem::update(float dt, const DriveSession& drive, simd_float3 eye) {
    namespace fs = std::filesystem;
    if (!gEffectsTables && !gEffectsDataDir.empty()) {
        const fs::path d(gEffectsDataDir);
        gEffectsTables = std::make_unique<fuel::sfx::Tables>(fuel::sfx::loadTables(
            (d / "XFinalComponent" / "FUEL.exe").string(), (d / "CommonComponent" / "Param" / "pp001.tsc").string()));
        if (!gEffectsTables->ok) std::fprintf(stderr, "fuelview: no wheel effects: %s\n", gEffectsTables->error.c_str());
    }
    if (!gEffectsTables || !gEffectsTables->ok) return;
    if (!sys_) sys_ = std::make_unique<fuel::sfx::System>(*gEffectsTables);
    if (dt <= 0) return;
    float rain = 0;
    if (const auto wf = fuel::weatherFieldsCached((fs::path(gEffectsDataDir) / "CommonComponent" / "WORLD" / "USA1.DPC").string()))
        rain = wf->sample(eye.x, eye.y, eye.z).rain;
    // The feed of 0x5ceb60, per vehicle and wheel: a wheel emits while loaded (> 0.01) and within 200 m.
    std::vector<fuel::sfx::System::WheelFeed> feeds;
    for (int v = 0; v < drive.vehicleCount(); ++v) {
        const fuel::VehicleState st = drive.state(v);
        const std::vector<float>& vp = drive.setup(v).params;
        auto VP = [&](int n) { return size_t(n) < vp.size() ? vp[size_t(n)] : 0.0f; };
        const int noFx = int(std::floor(VP(670)));  // veh+0x1048 = VehiclePP 670: wheels without emitters (bit w)
        for (size_t w = 0; w < st.wheelFx.size() && w < 16; ++w) {
            const fuel::VehicleState::WheelFx& fx = st.wheelFx[w];
            fuel::sfx::System::WheelFeed f;
            f.key = v * 16 + int(w);
            f.player = v == 0;
            fuel::sfx::WheelInput& in = f.in;
            in.contact = {float(fx.contact[0]), float(fx.contact[1]), float(fx.contact[2])};
            in.velocity = fx.velocity;
            in.forward = fx.forward;
            in.spin = fx.spin, in.radius = fx.radius, in.load = fx.load, in.slip = fx.slip, in.depth = fx.depth;
            in.classes = fx.classes;
            if (groundColour) in.groundColour = groundColour(fx.contact[0], fx.contact[2]);
            const simd_float3 c = simd_make_float3(in.contact[0], in.contact[1], in.contact[2]);
            in.cameraDistance = simd_distance(c, eye);
            in.rain = rain;  // WeatherManager +0xb8 at the camera (core/weather.hpp)
            for (int k = 0; k < 6; ++k) in.intensity[size_t(k)] = VP(671 + k);
            if (noFx & (1 << int(w))) continue;
            f.active = fx.load > 0.01f && in.cameraDistance <= 200.0f;
            feeds.push_back(f);
        }
    }
    sys_->update(dt, feeds, {eye.x, eye.y, eye.z});
    // Sprites for the renderer.
    std::vector<const fuel::sfx::Particle*> ps;
    sys_->collect(ps);
    frame_.sprites.clear();
    frame_.sprites.reserve(ps.size());
    for (const fuel::sfx::Particle* p : ps) {
        fuel::SfxSprite s;
        const bool gpu = p->gpu;
        const auto& at = gpu ? p->start : p->pos;
        s.posAngle = simd_make_float4(at[0], at[1], at[2], p->angle);
        s.uv = simd_make_float4(p->uv[0], p->uv[1], p->uv[2], p->uv[3]);
        s.colour = simd_make_float4(p->colour[0], p->colour[1], p->colour[2], gpu ? p->colour[3] : p->drawAlpha);
        s.vel = simd_make_float4(p->vel[0], p->vel[1], p->vel[2], gpu ? p->size : p->drawSize);
        s.acc = simd_make_float4(p->accel[0], p->accel[1], p->accel[2], p->rotSpeed);
        s.life = simd_make_float4(p->spawnTime, p->life, gpu ? 1.0f : 0.0f, 0);
        const auto pa = fuel::sfx::codeParams(fuel::sfx::fadeCode(p->alphaFn));
        const auto pz = fuel::sfx::codeParams(fuel::sfx::fadeCode(p->sizeFn));
        s.alphaP = simd_make_float4(pa[0], pa[1], pa[2], pa[3]);
        s.scaleP = simd_make_float4(pz[0], pz[1], pz[2], pz[3]);
        frame_.sprites.push_back(s);
    }
    frame_.clock = sys_->clock();
    if (std::getenv("FUEL_SFX_DEBUG") && !frame_.sprites.empty() && int(sys_->clock() * 60) % 60 == 0) {
        const fuel::SfxSprite& s = frame_.sprites.back();
        if (std::atoi(std::getenv("FUEL_SFX_DEBUG")) == 3)
            for (const fuel::SfxSprite& q : frame_.sprites)
                std::fprintf(stderr, "  sprite rel (%.2f %.2f %.2f) a %.2f size %.2f\n", q.posAngle.x - eye.x, q.posAngle.y - eye.y, q.posAngle.z - eye.z,
                             q.colour.w, q.vel.w);
        std::fprintf(stderr, "sfx: %zu sprites (%zu gpu), last: pos %.1f %.1f %.1f size %.2f col %.2f %.2f %.2f a %.2f life %.2f gpu %.0f uv %.2f %.2f %.2f %.2f\n",
                     frame_.sprites.size(), sys_->gpuCount(), s.posAngle.x, s.posAngle.y, s.posAngle.z, s.vel.w, s.colour.x, s.colour.y,
                     s.colour.z, s.colour.w, s.life.y, s.life.z, s.uv.x, s.uv.y, s.uv.z, s.uv.w);
    }
    frame_.fadeTo = gEffectsTables->P(5530), frame_.fadeOut = gEffectsTables->P(5531);
}

namespace {

uint64_t cellKey(int64_t cx, int64_t cz) { return (uint64_t(uint32_t(int32_t(cx))) << 32) | uint32_t(int32_t(cz)); }

}  // namespace

WorldStreamer::WorldStreamer(const std::vector<const fuel::DpcFile*>& subWorlds, const std::vector<const fuel::DpcFile*>& lookup)
    : lookup_(lookup) {
    for (const fuel::DpcFile* sub : subWorlds) {
        const auto subIndex = uint16_t(subFiles_.size());
        subFiles_.push_back(std::filesystem::path(sub->path()).filename().string());
        for (const fuel::PlacedMesh& pm : fuel::collectPlacedMeshes(*sub, lookup)) {
            const float x = pm.world[12], z = pm.world[14];
            if (std::fabs(x) < 50 && std::fabs(z) < 50) continue;  // stray objects at the origin
            const auto index = uint32_t(placed_.size());
            placed_.push_back({pm.meshId, pm.world, pm.fitToGround, pm.groundNormal, subIndex});
            cells_[cellKey(int64_t(std::floor(x / kCell)), int64_t(std::floor(z / kCell)))].push_back(index);
            addObstacle(pm);
        }
    }
}

void WorldStreamer::setSubWorlds(std::vector<fuel::SubWorldRef> refs) {
    refs_ = std::move(refs);
    subRef_.assign(subFiles_.size(), -1);
    for (size_t f = 0; f < subFiles_.size(); ++f)
        for (size_t r = 0; r < refs_.size(); ++r)
            if (strcasecmp(refs_[r].file.c_str(), subFiles_[f].c_str()) == 0) subRef_[f] = int(r);
    subActive_.assign(subFiles_.size(), 1);
    streamed_ = true;
    last_ = {1e30f, 0, 0};  // rebuild on the next update
}

void WorldStreamer::setMission(const std::string& mission) {
    if (strcasecmp(mission.c_str(), mission_.c_str()) == 0) return;
    mission_ = mission;  // M_TELEPORT of the new mission / M_END of the old one (0x513630)
    last_ = {1e30f, 0, 0};
}

bool WorldStreamer::refreshActive(simd_float3 player) {
    if (!streamed_) return false;
    const std::vector<size_t> regions = fuel::nearestRegions(refs_, player.x, player.z);
    bool changed = false;
    for (size_t f = 0; f < subFiles_.size(); ++f) {
        const int r = subRef_[f];
        bool on = true;  // files without a WorldRef: not streamed by FUEL, keep the old behaviour
        if (r >= 0 && refs_[size_t(r)].kind == fuel::SubWorldRef::Region)
            on = std::find(regions.begin(), regions.end(), size_t(r)) != regions.end();
        else if (r >= 0 && refs_[size_t(r)].kind == fuel::SubWorldRef::Course)
            on = !mission_.empty() && strcasecmp(refs_[size_t(r)].mission.c_str(), mission_.c_str()) == 0;
        changed = changed || subActive_[f] != char(on);
        subActive_[f] = char(on);
    }
    return changed;
}

void WorldStreamer::setVegetation(const fuel::Vegetation* v, const fuel::Heightfield* terrain) {
    // Static placements keep their authored height, FITTOGROUND or not: the engine reads that
    // keyword (and GROUNDNORMAL) only in 0x778650, the AutoMesh spawn of ground elements of kind
    // 0x17 (0x779440), never for sub-world nodes (p35 F2; the strings' only references, and the
    // keyword search 0x7b1bc0's only caller).
    vegetation_ = v, terrain_ = terrain;
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
        fuel::scatterPlants(*vegetation_, *terrain_, roads_, blocked, player.x, player.z, kPlantRadius, scattered, false, 0, ground_);
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
        // Beyond kPlantRadius only trees remain, all as cards (scattered with them).
        fuel::scatterPlants(*vegetation_, *terrain_, roads_, blocked, player.x, player.z, kCardRadius, scattered, true, kPlantRadius, ground_);
        for (fuel::Plant& p : scattered) {
            const double chunk = decorChunkSize(p.position[0], p.position[2], player.x, player.z);
            const auto it = card.find(p.meshId);
            const bool tree = trees.count(p.meshId) != 0, isCard = it == card.end();
            if (chunk >= (tree || isCard ? kTreeChunkMax : kBushChunkMax)) continue;
            if (!isCard && chunk >= kFullMeshChunk) {
                if (it->second == 0) continue;
                p.meshId = it->second;
            }
            plants.push_back(p);
        }
        if (std::getenv("FUEL_STREAM_TRACE"))
            std::printf("plants: %zu scattered, %zu drawn (%zu trunks)\n", scattered.size(), plants.size(), bg.trunks.size());
        bg.plants.reserve(plants.size());
        for (const fuel::Plant& p : plants) {
            auto col = [&](size_t i) { return simd_make_float4(p.axes[i][0], p.axes[i][1], p.axes[i][2], 0); };
            bg.plants.push_back({p.meshId, simd_matrix(col(0), col(1), col(2), simd_make_float4(p.position[0], p.position[1], p.position[2], 1))});
        }
    }
    if (lakes_ && ground_ && terrain_) buildWater(player, bg);
    return bg;
}

// Lake and river water around the player, by Paysage_BuildWaterMesh's rules (0x7fbc10, water.md
// 3.1): lakes on one 8 m grid over 2 km, rivers (category 0xf, p34) on one 2 m grid over 384 m
// (the engine builds both per terrain chunk at the chunk's vertex spacing). The vertex `pad`
// carries the category for the shader: 0 lake (vh/ph_water), 1 river (vh_water_noanim /
// ph_water_river).
void WorldStreamer::buildWater(simd_float3 player, Background& bg) const {
    bg.haveWater = true;
    struct P {
        float y, W, S, rf;
        float c60[3], c70[3];
    };
    auto pack = [](const float* v, float scale) {  // BGRA bytes; 0x7fd280 packs floor(v * 128)
        uint32_t out = 0;
        for (int i = 0; i < 3; ++i) out |= uint32_t(std::clamp(int(std::floor(v[2 - i] * scale)), 0, 255)) << (8 * i);
        return out;
    };
    // One grid: n x n points at spacing sp from (ox, oz).
    auto grid = [&](int n, double sp, double ox, double oz, bool river, const std::function<void(double, double, P&)>& eval) {
        std::vector<P> pts(size_t(n) * n);
        for (int r = 0; r < n; ++r)
            for (int c = 0; c < n; ++c) eval(ox + c * sp, oz + r * sp, pts[size_t(r) * n + c]);
        const uint32_t base = uint32_t(bg.water.size());
        std::vector<uint8_t> alpha(pts.size(), 0);
        bg.water.resize(base + pts.size());
        for (int r = 0; r < n; ++r)
            for (int c = 0; c < n; ++c) {
                const P& p = pts[size_t(r) * n + c];
                float W = p.W, S = p.S;
                const float* c60 = p.c60;
                if (!river && S <= p.y + 0.01f) {  // dry: a wet diagonal neighbour's level (last one wins); not for 0xf
                    for (int rr = std::max(r - 1, 0); rr <= std::min(r + 1, n - 1); ++rr)
                        for (int cc = std::max(c - 1, 0); cc <= std::min(c + 1, n - 1); ++cc) {
                            if (rr == r || cc == c) continue;
                            const P& q = pts[size_t(rr) * n + cc];
                            if (q.y + 0.01f < q.S) W = q.W, S = q.S, c60 = q.c60;
                        }
                }
                uint32_t col;
                uint8_t a;
                if (S - p.y < 0.01f) col = pack(c60, 128.0f), a = 0;
                else if (W - p.y <= 0.01f) col = pack(p.c70, 255.0f), a = 10;
                else {
                    const float m[3] = {(p.c70[0] + c60[0]) * 0.5f, (p.c70[1] + c60[1]) * 0.5f, (p.c70[2] + c60[2]) * 0.5f};
                    col = pack(m, 128.0f), a = 40;
                }
                alpha[size_t(r) * n + c] = a;
                fuel::MeshRenderer::WaterVertex& v = bg.water[base + size_t(r) * n + size_t(c)];
                const float x = float(ox + c * sp), z = float(oz + r * sp);
                v.position[0] = x, v.position[1] = S, v.position[2] = z;
                v.colour = col | uint32_t(a) << 24;
                v.uv[0] = x, v.uv[1] = z;
                v.depth = S - p.y;
                v.pad = river ? 1.0f : 0.0f;
            }
        auto rf = [&](uint32_t i) { return pts[i].rf > 0.99f; };
        for (int r = 1; r < n; ++r)
            for (int c = 1; c < n; ++c) {
                const uint32_t v = uint32_t(r * n + c), a = v - n - 1, b = v - n, l = v - 1;
                // Triangles with a wet vertex; river ones also need a river point (rf > 0.99).
                if ((alpha[a] || alpha[b] || alpha[v]) && (!river || rf(a) || rf(v) || rf(b)))
                    bg.waterIndices.insert(bg.waterIndices.end(), {base + a, base + b, base + v});
                if ((alpha[a] || alpha[v] || alpha[l]) && (!river || rf(a) || rf(l) || rf(v)))
                    bg.waterIndices.insert(bg.waterIndices.end(), {base + a, base + v, base + l});
            }
    };
    auto colours = [&](double x, double z, const fuel::GroundPoint& gp, P& p) {
        const fuel::GroundColours gc = ground_->colours(x, z, gp);
        for (int i = 0; i < 3; ++i) p.c60[i] = gc.water[i], p.c70[i] = gc.base[i];
    };
    // Lakes (category 0xe): the lake query alone (river points belong to 0xf).
    {
        constexpr int n = 257;
        constexpr double sp = 8.0, half = sp * (n - 1) / 2;
        const double ox = std::round(player.x / 64.0) * 64.0 - half, oz = std::round(player.z / 64.0) * 64.0 - half;
        bool any = false;
        for (const fuel::Lake& L : lakes_->lakes())
            if (L.maxX >= ox && L.minX <= ox + 2 * half && L.maxZ >= oz && L.minZ <= oz + 2 * half) any = true;
        if (any)
            grid(n, sp, ox, oz, false, [&](double x, double z, P& p) {
                p.y = float(terrain_->smoothHeightAt(x, z));
                const fuel::WaterPoint w = fuel::waterAt(*lakes_, x, z, p.y);
                const bool flag = w.depth - w.base > 0.001f;
                p.W = flag ? p.y + (w.depth - w.base) : p.y;
                p.S = std::max(p.y, p.W);
                p.rf = 0;
                if (w.depth > 0) colours(x, z, ground_->at(x, z), p);
                else for (int i = 0; i < 3; ++i) p.c60[i] = p.c70[i] = 0;
            });
    }
    // Rivers (category 0xf): points whose water is all river (ctx+0x30c > 0.99) take part.
    if (terrain_->detail) {
        constexpr int n = 193;
        constexpr double sp = 2.0, half = sp * (n - 1) / 2;
        const double ox = std::round(player.x / 16.0) * 16.0 - half, oz = std::round(player.z / 16.0) * 16.0 - half;
        const fuel::TerrainDetail& det = *terrain_->detail;
        // D does not depend on y: find the river points first; they and their neighbours are evaluated.
        std::vector<uint8_t> wet(size_t(n) * n, 0), need(size_t(n) * n, 0);
        bool any = false;
        for (int r = 0; r < n; ++r)
            for (int c = 0; c < n; ++c)
                if (det.river(ox + c * sp, oz + r * sp, 0).flag) {
                    wet[size_t(r) * n + c] = 1, any = true;
                    for (int rr = std::max(r - 1, 0); rr <= std::min(r + 1, n - 1); ++rr)
                        for (int cc = std::max(c - 1, 0); cc <= std::min(c + 1, n - 1); ++cc) need[size_t(rr) * n + cc] = 1;
                }
        if (any)
        grid(n, sp, ox, oz, true, [&](double x, double z, P& p) {
            p.rf = 0;
            for (int i = 0; i < 3; ++i) p.c60[i] = p.c70[i] = 0;
            const size_t k = size_t(std::lround((z - oz) / sp)) * n + size_t(std::lround((x - ox) / sp));
            if (!need[k]) {
                p.y = p.W = p.S = 0;
                return;
            }
            if (!wet[k]) {
                p.y = p.W = p.S = float(terrain_->smoothHeightAt(x, z));
                return;
            }
            const fuel::GroundPoint gp = ground_->at(x, z);
            p.y = float(terrain_->smoothHeightAt(x, z));
            const bool flag = gp.waterDepth - gp.waterBase > 0.001f;
            p.W = flag ? p.y + (gp.waterDepth - gp.waterBase) : p.y;
            p.S = std::max(p.y, p.W);
            p.rf = gp.riverShare;
            colours(x, z, gp, p);
        });
    }
    if (bg.waterIndices.empty()) bg.water.clear();
    if (std::getenv("FUEL_STREAM_TRACE")) std::printf("water: %zu triangles\n", bg.waterIndices.size() / 3);
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
    if (bg.haveWater) renderer.setWater(bg.water, bg.waterIndices);
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
    updateGrass(player, renderer, force);
    updateGroundTiles(player, renderer, force);
    pollBackground(renderer, physics);
    if (refreshActive(player)) {
        force = true;
        for (auto it = props_.begin(); it != props_.end();) {  // detached sub-worlds take their props along
            if (active(placed_[it->first])) {
                ++it;
                continue;
            }
            if (it->second.body) physics.removeProp(it->first);
            it = props_.erase(it);
        }
    }
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
                if (d > kDrawRadius || !active(p)) continue;
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

void WorldStreamer::settle(simd_float3 player, fuel::MeshRenderer& renderer, fuel::DriveWorld& physics) {
    if (job_.valid()) (void)job_.get();  // stale: built for an earlier position
    if (grassJob_.valid()) grassCache_ = std::move(grassJob_.get().cache);
    if (tileJob_.valid()) (void)tileJob_.get();
    rerun_ = false;
    grassAt_ = {1e30f, 0, 1e30f};  // rebuild grass and both fine colour tiles here
    tileAt_[0] = tileAt_[1] = simd_make_float2(1e30f, 1e30f);
    update(player, renderer, physics, true);
}

std::vector<WorldStreamer::GroundGap> WorldStreamer::groundGaps(double x, double z, double radius, float minGap) const {
    std::vector<GroundGap> out;
    if (terrain_ == nullptr) return out;
    const int64_t r = int64_t(std::ceil(radius / kCell));
    const int64_t cx = int64_t(std::floor(x / kCell)), cz = int64_t(std::floor(z / kCell));
    for (int64_t dz = -r; dz <= r; ++dz)
        for (int64_t dx = -r; dx <= r; ++dx) {
            auto it = cells_.find(cellKey(cx + dx, cz + dz));
            if (it == cells_.end()) continue;
            for (uint32_t i : it->second) {
                const Placed& p = placed_[i];
                if (std::hypot(p.world[12] - x, p.world[14] - z) > radius || !active(p)) continue;
                const float gap = p.world[13] - terrain_->smoothHeightAt(p.world[12], p.world[14]);
                if (std::fabs(gap) >= minGap) out.push_back({p.meshId, simd_make_float3(p.world[12], p.world[13], p.world[14]), gap});
            }
        }
    std::sort(out.begin(), out.end(), [](const GroundGap& a, const GroundGap& b) { return a.gap > b.gap; });
    return out;
}


// ---- Grass (p14) ----

WorldStreamer::GrassJob WorldStreamer::buildGrass(simd_float3 player, GrassCache cache) const {
    GrassJob job;
    const auto t0 = std::chrono::steady_clock::now();
    size_t built = 0;
    const int32_t c0x = int32_t(std::floor((player.x - kGrassReach) / kGrassChunk)), c1x = int32_t(std::floor((player.x + kGrassReach) / kGrassChunk));
    const int32_t c0z = int32_t(std::floor((player.z - kGrassReach) / kGrassChunk)), c1z = int32_t(std::floor((player.z + kGrassReach) / kGrassChunk));
    // Forget chunks well outside the radius.
    for (auto it = cache.begin(); it != cache.end();) {
        const int32_t cx = int32_t(uint32_t(it->first >> 32)), cz = int32_t(uint32_t(it->first));
        if (cx < c0x - 2 || cx > c1x + 2 || cz < c0z - 2 || cz > c1z + 2) it = cache.erase(it);
        else ++it;
    }
    for (int32_t cz = c0z; cz <= c1z; ++cz)
        for (int32_t cx = c0x; cx <= c1x; ++cx) {
            // Grass only where this 24 m chunk is a leaf of the decor quadtree.
            if (decorChunkSize((cx + 0.5) * kGrassChunk, (cz + 0.5) * kGrassChunk, player.x, player.z) > 30.0) continue;
            const uint64_t key = (uint64_t(uint32_t(cx)) << 32) | uint32_t(cz);
            auto it = cache.find(key);
            if (it == cache.end()) {
                std::vector<fuel::GrassTuft> tufts;
                ground_->grass(cx * kGrassChunk, cz * kGrassChunk, (cx + 1) * kGrassChunk, (cz + 1) * kGrassChunk, tufts);
                ++built;
                it = cache.emplace(key, std::move(tufts)).first;
            }
            job.tufts.insert(job.tufts.end(), it->second.begin(), it->second.end());
        }
    job.cache = std::move(cache);
    if (std::getenv("FUEL_STREAM_TRACE"))
        std::printf("grass: %zu tufts, %zu new chunks in %.1f ms\n", job.tufts.size(), built, std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());
    return job;
}

void WorldStreamer::updateGrass(simd_float3 player, fuel::MeshRenderer& renderer, bool force) {
    if (ground_ == nullptr) return;
    auto apply = [&](GrassJob&& job) {
        grassCache_ = std::move(job.cache);
        renderer.grassRadius = kGrassReach;
        renderer.setGrass(job.tufts);
    };
    if (grassJob_.valid() && (force || grassJob_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)) apply(grassJob_.get());
    const float moved = std::hypot(player.x - grassAt_.x, player.z - grassAt_.z);
    if (moved < kGrassStep || grassJob_.valid()) return;
    grassAt_ = player;
    if (force) {
        apply(buildGrass(player, std::move(grassCache_)));
        return;
    }
    grassJob_ = std::async(std::launch::async, [this, player, cache = std::move(grassCache_)]() mutable { return buildGrass(player, std::move(cache)); });
}

// ---- Fine ground colour ----

WorldStreamer::GroundTile WorldStreamer::buildGroundTile(int level, simd_float3 player) const {
    constexpr uint32_t n = fuel::MeshRenderer::kGroundTile;
    const double sp = kTileSpacing[level], extent = sp * n;
    // Snap to 16 texels so a rebuilt tile lines up with the last one; texel centres fall on
    // multiples of the spacing, i.e. on FUEL's ground vertices (Paysage_FillPoints 0x7f92c0).
    const double snap = 16 * sp;
    GroundTile t{level, std::round(player.x / snap) * snap - extent / 2 - sp / 2, std::round(player.z / snap) * snap - extent / 2 - sp / 2,
                 float(extent), std::vector<uint8_t>(size_t(n) * n * 4), std::vector<uint16_t>(size_t(n) * n * 4),
                 std::vector<float>(size_t(n) * n * 4)};
    const auto t0 = std::chrono::steady_clock::now();
    // Two threads: more starve physics and streaming on the main thread (4 performance cores).
    constexpr unsigned threads = 2;
    std::vector<std::thread> pool;
    for (unsigned k = 0; k < threads; ++k)
        pool.emplace_back([&, k] {
            for (uint32_t j = k; j < n; j += threads)
                for (uint32_t i = 0; i < n; ++i) {
                    const double x = t.originX + (i + 0.5) * sp, z = t.originZ + (j + 0.5) * sp;
                    const fuel::GroundPoint p = ground_->at(x, z);
                    // The 6 m palette-half noise is taken at the point (1 m) or averaged over the texel.
                    const fuel::GroundColours c = ground_->colours(x, z, p, level == 0 ? -1.0f : ground_->meanT0(x, z, sp, 2));
                    const size_t o = (size_t(j) * n + i) * 4;
                    uint8_t* px = &t.rgba[o];
                    for (int q = 0; q < 3; ++q) px[q] = uint8_t(std::clamp(c.base[q], 0.0f, 1.0f) * 255.0f + 0.5f);
                    // Alpha: the ground's sky occlusion for the lighting (ground.vhl; P28). Inferred to be stored
                    // like the tufts' byte clamp(0.8 a + 0.2), a = ctx+0x230. (The far band's forest weight comes
                    // from the 60 m map.)
                    px[3] = uint8_t(std::clamp(0.8f * c.occlusion + 0.2f, 0.0f, 1.0f) * 255.0f + 0.5f);
                    // Record bytes R+0x3a..0x41 (0x6ac120): weight byte = floor(255 w + 0.5).
                    for (int q = 0; q < 4; ++q)
                        t.types[o + size_t(q)] =
                            uint16_t(p.surfaceType[q] | uint16_t(std::floor(std::clamp(p.surfaceWeight[q], 0.0f, 1.0f) * 255.0f + 0.5f)) << 8);
                    // Decal: R+0x43 = byte(64 w) / 64 (0x6a5500), uv floats, material slot.
                    const float w = std::floor(std::clamp(p.decal, 0.0f, 3.98f) * 64.0f + 0.5f) / 64.0f;
                    t.decal[o] = p.decalU;
                    t.decal[o + 1] = p.decalV;
                    t.decal[o + 2] = p.decalMaterial == 0xfff ? 0.0f : w;
                    t.decal[o + 3] = float(fuel::MeshRenderer::decalSlot(p.decalMaterial));
                }
        });
    for (auto& th : pool) th.join();
    // Seam rule (0x7f9b50): neighbouring vertices whose decal uv differ by more than 0.5 both
    // lose their decal (junctions, where two roads' uv meet).
    {
        std::vector<uint8_t> kill(size_t(n) * n, 0);
        for (uint32_t j = 0; j < n; ++j)
            for (uint32_t i = 0; i < n; ++i) {
                const float* a = &t.decal[(size_t(j) * n + i) * 4];
                if (!(a[2] > 0.01f)) continue;
                for (int dj = 0; dj <= 1; ++dj)
                    for (int di = -1; di <= 1; ++di) {
                        if (dj == 0 && di <= 0) continue;
                        const int64_t ii = int64_t(i) + di, jj = int64_t(j) + dj;
                        if (ii < 0 || jj < 0 || ii >= int64_t(n) || jj >= int64_t(n)) continue;
                        const float* b = &t.decal[(size_t(jj) * n + size_t(ii)) * 4];
                        if (!(b[2] > 0.01f)) continue;
                        if (std::fabs(a[0] - b[0]) > 0.5f || std::fabs(a[1] - b[1]) > 0.5f)
                            kill[size_t(j) * n + i] = kill[size_t(jj) * n + size_t(ii)] = 1;
                    }
            }
        for (size_t q = 0; q < kill.size(); ++q)
            if (kill[q]) t.decal[q * 4 + 2] = 0;
    }
    if (std::getenv("FUEL_STREAM_TRACE"))
        std::printf("ground tile %d: %.0f m at (%.0f, %.0f) in %.1f ms\n", level, extent, t.originX, t.originZ,
                    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());
    return t;
}

void WorldStreamer::updateGroundTiles(simd_float3 player, fuel::MeshRenderer& renderer, bool force) {
    if (ground_ == nullptr || !ground_->hasPalette()) return;
    auto apply = [&](GroundTile&& t) { renderer.setGroundTile(t.level, t.originX, t.originZ, t.extent, t.rgba, t.types, t.decal); };
    if (tileJob_.valid() && (force || tileJob_.wait_for(std::chrono::seconds(0)) == std::future_status::ready)) apply(tileJob_.get());
    if (tileJob_.valid()) return;
    for (int level = 0; level < kTileLevels; ++level) {
        if (std::hypot(player.x - tileAt_[level].x, player.z - tileAt_[level].y) < kTileStep[level]) continue;
        tileAt_[level] = simd_make_float2(player.x, player.z);
        if (force) {
            apply(buildGroundTile(level, player));
            continue;
        }
        tileJob_ = std::async(std::launch::async, [this, level, player] { return buildGroundTile(level, player); });
        return;  // one tile at a time
    }
}
