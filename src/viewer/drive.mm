#import "drive.h"
#include "rider_view.h"
#include "storm_view.h"

#include <filesystem>

#include "core/weather.hpp"

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstring>
#include <map>
#include <set>
#include <strings.h>
#include <thread>

#include "core/resources.hpp"
#include "core/surface.hpp"
#include "core/static_collision.hpp"
#include "core/dyn_object.hpp"
#include "core/assembly.hpp"
#include "core/world.hpp"
#include "core/terrain_mesh.hpp"  // PK32: groundCurvatureGrid

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

// ---- P104: FUEL's vehicle LOD (drive.h VehicleLods; notes p104-vehicle-lod.md)
bool vehicleLodEnabled() {
    static const bool on = [] {
        const char* e = std::getenv("FUEL_VEHICLE_LOD");
        return !(e && std::atoi(e) == 0);
    }();
    return on;
}

void VehicleRig::addPart(const fuel::PlacedPart& p, int level, const fuel::upgvis::PartEdit* e, int susp, int suspNode) {
    if (e && e->hide) return;  // [upgrades U5] optional bodywork taken off (weight)
    simd_float4x4 m = rowsToSimd(p.world);
    const simd_float4x4 rest = m;  // [suspanim]
    if (e && e->moved && !isWheel(p)) m = simd_mul(rowsToSimd(e->adjust), m);  // [upgrades U5] re-aimed / grown (vehicle space)
    const uint32_t mesh = e && e->mesh ? e->mesh : p.meshId;               // [upgrades U5] a derived mesh (carbon, batches off)
    PartLink link{-1, m};
    link.level = level;
    link.dyn = p.dyn, link.dynPivot = p.dynPivot, link.dynBoneQ = p.dynBoneQ;  // PK22
    link.susp = susp, link.suspNode = isWheel(p) ? -1 : suspNode, link.rest = rest;  // [suspanim]
    if (isWheel(p)) {
        // Follow the physics wheel whose hub is nearest; keep the part's own
        // orientation (wheels shared left/right are flipped 180 degrees).
        float best = 1e9f;
        for (size_t i = 0; i < setup_.wheels.size(); ++i) {
            const auto& h = setup_.wheels[i].hub;
            const float d = simd_distance(simd_make_float3(h[0], h[1], h[2]), m.columns[3].xyz);
            if (d < best) best = d, link.wheel = int(i);
        }
        link.local.columns[3] = simd_make_float4(0, 0, 0, 1);
        if (e && (e->axle != 1.0f || e->radial != 1.0f)) {  // [upgrades U5] the tyre stage: wider (mesh x), taller
            const simd_float4x4 sc = simd_diagonal_matrix(simd_make_float4(e->axle, e->radial, e->radial, 1));
            link.local = simd_mul(link.local, sc);
            m = simd_mul(m, sc);
        }
    }
    parts_.push_back({mesh, m});
    links_.push_back(link);
}

// [upgrades U5] the plan's procedural parts: on the chassis (vehicle space) or on a physics wheel (its pose's space).
void VehicleRig::addExtras(const fuel::upgvis::Plan& plan, int level, int susp) {
    for (const fuel::upgvis::Extra& x : plan.extras) {
        PartLink link{x.wheel, matrix_identity_float4x4};
        link.level = level;
        link.susp = susp;  // [suspanim] a wheel extra moves with its wheel's fork shift
        simd_float4x4 rest = matrix_identity_float4x4;
        if (x.wheel >= 0 && size_t(x.wheel) < setup_.wheels.size()) {
            const auto& h = setup_.wheels[size_t(x.wheel)].hub;
            rest.columns[3] = simd_make_float4(h[0], h[1], h[2], 1);
        } else {
            link.wheel = -1;
        }
        parts_.push_back({x.key, rest});
        links_.push_back(link);
    }
    // [suspanim] the coils / coil-overs: chassis-space meshes at the static stance, moved by their rig node every frame
    const fuel::susp::Rig* rig = susp >= 0 ? &susp_[size_t(susp)] : nullptr;
    for (size_t i = 0; i < plan.liveExtras.size(); ++i) {
        PartLink link{-1, matrix_identity_float4x4};
        link.level = level;
        link.susp = susp;
        link.suspNode = rig && i < rig->extraNode.size() ? rig->extraNode[i] : -1;
        parts_.push_back({plan.liveExtras[i].key, matrix_identity_float4x4});
        links_.push_back(link);
    }
}

// [suspanim] one LOD record: its rig (FUEL's links + the plan's live parts), its parts and extras
int VehicleRig::addRecord(const fuel::AssembledVehicle& av, const fuel::upgvis::Plan& plan, int level) {
    std::vector<fuel::susp::V3> hubs;
    for (const auto& w : setup_.wheels) hubs.push_back({w.hub[0], w.hub[1], w.hub[2]});
    susp_.push_back(fuel::susp::build(av, hubs, plan.active ? &plan : nullptr));
    const int ri = int(susp_.size()) - 1;
    const fuel::susp::Rig& rig = susp_.back();
    for (size_t k = 0; k < av.parts.size(); ++k)
        if (av.parts[k].drawn()) addPart(av.parts[k], level, plan.active ? &plan.parts[k] : nullptr, ri, rig.partNode[k]);
    if (plan.active) addExtras(plan, level, ri);
    return ri;
}

void VehicleRig::updateLod(const fuel::VehicleState& st, simd_float3 eye) {
    if (!lodOn_) return;
    const fuel::LodView view;  // K = K' = 10, view+0xec = 0.3972 (p36 L2 / L3)
    const simd_float4x4 node = poseMatrix(st.chassis);
    anchor_ = node.columns[3].xyz;  // the vehicle agent's node (node+0xd4 row 3)
    float d = simd_distance(anchor_, eye);
    level_ = fuel::lodSwitchLevel(d, lodD_, lodScale_, std::min(minLevel_, levels_ - 1), levels_ - 1, view);
    alpha_ = lodView_ != 0 ? fuel::lodFade(lodView_, 0, 0, lodScale_ * view.viewScale, d, view) : 1.0f;
    if (!(alpha_ >= 0.01f)) alpha_ = 0;  // [0x99614c]
    alpha_ = std::min(alpha_, 1.0f);
    // The rider's own node: the seat (Rider_PlaceRootNode 0x5ca410), or where it fell when ejected.
    riderAnchor_ = simd_mul(node, simd_make_float4(float(seat_[0]), float(seat_[1]), float(seat_[2]), 1)).xyz;
    for (const PartLink& l : links_)
        if (l.rider && gRiderView != nullptr && gRiderView->animator(l.key) && gRiderView->animator(l.key)->detached()) {
            const auto o = gRiderView->animator(l.key)->worldOrigin();
            riderAnchor_ = simd_make_float3(float(o[0]), float(o[1]), float(o[2]));
            break;
        }
    d = simd_distance(riderAnchor_, eye);
    riderLevel_ = fuel::lodSwitchLevel(d, riderD_, 1.0f, 0, riderLevels_ - 1, view);  // no flag on an AI rider
    static const char* forced = std::getenv("FUEL_P104_RIDER_LEVEL");  // debug: force the rider level
    if (forced) riderLevel_ = std::clamp(std::atoi(forced), 0, riderLevels_ - 1);
    riderAlpha_ = riderView_ != 0 ? fuel::lodFade(riderView_, 0, 0, view.viewScale, d, view) : 1.0f;
    if (!(riderAlpha_ >= 0.01f)) riderAlpha_ = 0;
    riderAlpha_ = std::min(riderAlpha_, 1.0f);
}

std::vector<float> VehicleRig::partAlphas(bool lod1) const {
    std::vector<float> out(links_.size(), 1.0f);
    if (!lodOn_) return out;
    for (size_t k = 0; k < links_.size(); ++k) {
        const PartLink& l = links_[k];
        if (l.level < 0) continue;
        if (lod1) {
            out[k] = l.level == 0 ? 1.0f : 0.0f;
            continue;
        }
        out[k] = l.rider ? (l.level == riderLevel_ ? riderAlpha_ : 0.0f) : (l.level == level_ ? alpha_ : 0.0f);
    }
    return out;
}
// ---- P104 end

VehicleRig::VehicleRig(const fuel::AssembledVehicle& vehicle, const fuel::VehicleSetup& setup, const std::vector<uint32_t>& riders,
                       const VehicleLods* lods)
    : setup_(setup), occlusionExtents_(vehicle.occlusionExtents), lights_(vehicle.lights), hasEngine_(vehicle.hasEngine), engine_(vehicle.engine),
      dyn_(vehicle.dyn), headlightRecords_(vehicle.headlightRecords) {  // PK22: dyn_, headlightRecords_
    // ---- PK23: 0x5cf170 searches the LOD record of the visual's flag bit 22 (0x400000 = never LOD1: AI racers): LOD2
    exhausts_ = vehicle.exhausts;
    if (lods && lods->minLevel == 1 && !lods->lower.empty()) exhausts_ = lods->lower.front().exhausts;
    // ---- PK23 end
    {  // P77: engine wheel slots (front 0, 2, ..., rear 1, 3, ...) and the body blob's sizes from the hubs at rest (0x5ca7a0)
        std::vector<int> fronts, rears;
        for (size_t i = 0; i < setup.wheels.size(); ++i) (setup.wheels[i].front ? fronts : rears).push_back(int(i));
        for (size_t k = 0; k < std::max(fronts.size(), rears.size()); ++k) {
            if (k < fronts.size()) engineWheels_.push_back(fronts[k]);
            if (k < rears.size()) engineWheels_.push_back(rears[k]);
        }
        std::vector<fuel::blob::V3> local;
        for (int w : engineWheels_) local.push_back(setup.wheels[size_t(w)].hub);
        blobBodySize_ = fuel::blob::bodySizes(vehicle.boxHalf[0], vehicle.boxHalf[2], local);
    }
    lodOn_ = lods != nullptr && vehicleLodEnabled() && vehicle.lodLevels > 1;  // P104
    // [upgrades U5] the plan per drawn record (the player: LOD1 with everything; an opponent: the subset that shows)
    fuel::upgvis::Options upgOpt;
    upgOpt.opponent = lods != nullptr;
    const fuel::upgvis::Plan plan0 = setup.upgraded ? fuel::upgvis::makePlan(vehicle, setup, upgOpt) : fuel::upgvis::Plan{};
    addRecord(vehicle, plan0, lodOn_ ? 0 : -1);  // [suspanim] (parts, extras and the record's per-frame links)
    upgPlan_ = plan0;
    if (plan0.active && std::getenv("FUEL_U5_TRACE")) {
        std::printf("u5: %s LOD%d: %s\n", setup.name.c_str(), vehicle.lod, fuel::upgvis::describe(plan0).c_str());
        for (const std::string& n : plan0.notes) std::printf("u5:   %s\n", n.c_str());
    }
    for (uint32_t id : riders) {
        parts_.push_back({id, matrix_identity_float4x4});
        links_.push_back({-1, matrix_identity_float4x4, true, id, lodOn_ ? 0 : -1});
    }
    if (lodOn_) {  // P104: the lower levels' parts and rider meshes after LOD1's (the P85 part indices stay)
        levels_ = std::min(vehicle.lodLevels, 1 + int(lods->lower.size()));
        minLevel_ = lods->minLevel;
        level_ = std::min(minLevel_, levels_ - 1);  // until the first updateLod
        lodD_ = vehicle.lodBaseDistance, lodView_ = vehicle.lodViewDist, lodScale_ = vehicle.lodNodeScale;
        for (int k = 1; k < levels_; ++k) {
            const fuel::AssembledVehicle& lv = lods->lower[size_t(k - 1)];
            const fuel::upgvis::Plan plan = setup.upgraded ? fuel::upgvis::makePlan(lv, setup, upgOpt) : fuel::upgvis::Plan{};  // [upgrades U5]
            addRecord(lv, plan, k);  // [suspanim]
            if (k == 1 && minLevel_ == 1 && plan.active) upgPlan_ = plan;  // (the record the exhaust FX use, PK23)
        }
        const bool riderHere = !riders.empty();
        riderLevels_ = riderHere ? std::min(lods->riderLevels, 1 + int(lods->riderLower.size())) : 1;
        riderD_ = lods->riderD, riderView_ = lods->riderView, seat_ = lods->seat;
        for (int k = 1; k < riderLevels_; ++k) {
            const uint32_t id = lods->riderLower[size_t(k - 1)];
            parts_.push_back({id, matrix_identity_float4x4});
            links_.push_back({-1, matrix_identity_float4x4, true, id, k});
        }
    }
    for (const auto& w : setup.wheels) focusHeight_ = std::max(focusHeight_, w.hub[1] + w.radius * 0.6f);
}

RiderView* gRiderView = nullptr;

void VehicleRig::animate(float dt, const fuel::DriveInput& input, const fuel::VehicleState& st, const fuel::fv::FuelVehicle* model, bool player,
                         int raceState, const fuel::Heightfield& terrain) {
    if (gRiderView == nullptr || model == nullptr || dt <= 0) return;
    for (const PartLink& l : links_) {
        if (!l.rider || l.level > 0) continue;  // P104: the rider's lower levels share LOD1's animator
        fuel::RiderAnimator* a = gRiderView->animator(l.key);
        if (a == nullptr) continue;
        fuel::RiderDrive d;
        d.dt = dt;
        d.q = st.chassis.rotation;
        d.pos = st.chassis.position;
        const fuel::fv::Vec3 v = model->body().avgV;
        d.vel = {v.x, v.y, v.z};
        d.steer = input.steer;
        d.stickZ = input.stickZ;  // veh+0x19f4: the left stick's up/down (P47 input map)
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
        if (player && a->detached()) gRiderView->resetJiggle();  // PHYSIQUE: thrown off: the springs rest
        if (player && !a->detached()) {  // PHYSIQUE: the player's glute springs follow the pelvis in the world (animation + chassis)
            const simd_float4x4 c = poseMatrix(st.chassis);
            fuel::Mat4 vw{};
            for (int r = 0; r < 4; ++r)
                for (int q = 0; q < 4; ++q) vw[size_t(r * 4 + q)] = c.columns[r][q];
            gRiderView->stepJiggle(l.key, vw, dt, !a->detached());
        }
    }
}

// ---- PK31: overtake gestures (0x5c2370) on the rider's animator (shared by its LOD keys)
void VehicleRig::riderGesture(int base) {
    if (gRiderView == nullptr) return;
    for (const PartLink& l : links_)
        if (l.rider && l.level <= 0)
            if (fuel::RiderAnimator* a = gRiderView->animator(l.key)) a->proximityReaction(base);
}
// ---- end PK31

// ---- PK14
void VehicleRig::riderKillMission() {
    if (gRiderView == nullptr) return;
    for (const PartLink& l : links_)
        if (l.rider && l.level <= 0)
            if (fuel::RiderAnimator* a = gRiderView->animator(l.key)) a->killMission();
}
// ---- end PK14

void VehicleRig::uploadRider() const {
    if (gRiderView == nullptr) return;
    for (const PartLink& l : links_)
        if (l.rider && (!lodOn_ || l.level < 0 || (l.level == riderLevel_ && riderAlpha_ > 0)))  // P104: the drawn level only
            gRiderView->upload(l.key);
}

std::vector<simd_float4x4> VehicleRig::partWorlds(const fuel::VehicleState& st) const {
    const simd_float4x4 chassis = poseMatrix(st.chassis);
    // ---- [suspanim] the wheels in vehicle space (hub, steer vector) and every record's links for this frame
    bool susp = false;
    for (const fuel::susp::Rig& r : susp_) susp = susp || r.any();
    static const bool timeIt = std::getenv("FUEL_SUSP_TIME") != nullptr;  // the links' CPU cost per call (notes/suspanim.md)
    const auto t0 = timeIt ? std::chrono::steady_clock::now() : std::chrono::steady_clock::time_point{};
    if (susp) {
        const simd_quatf qc = simd_quaternion(st.chassis.rotation[0], st.chassis.rotation[1], st.chassis.rotation[2], st.chassis.rotation[3]);
        const simd_quatf qi = simd_inverse(qc);
        const simd_float3 pc = simd_make_float3(float(st.chassis.position[0]), float(st.chassis.position[1]), float(st.chassis.position[2]));
        suspWheels_.resize(st.wheels.size());
        for (size_t i = 0; i < st.wheels.size(); ++i) {
            const fuel::Pose& w = st.wheels[i];
            const simd_float3 h = simd_act(qi, simd_make_float3(float(w.position[0]), float(w.position[1]), float(w.position[2])) - pc);
            const simd_quatf qw = simd_quaternion(w.rotation[0], w.rotation[1], w.rotation[2], w.rotation[3]);
            const simd_float3 axle = simd_act(simd_mul(qi, qw), simd_make_float3(1, 0, 0));  // spin-free
            simd_float3 f = simd_make_float3(-axle.z, 0, axle.x);
            f = simd_length(f) > 1e-6f ? simd_normalize(f) : simd_make_float3(0, 0, 1);
            suspWheels_[i].hub = {h.x, h.y, h.z};
            suspWheels_[i].forward = {f.x, f.y, f.z};
        }
        suspOut_.resize(susp_.size());
        for (size_t r = 0; r < susp_.size(); ++r)
            if (susp_[r].any()) fuel::susp::evaluate(susp_[r], suspWheels_, suspOut_[r]);
    }
    if (timeIt && susp) {
        static double sum = 0;
        static long calls = 0, nodes = 0;
        sum += std::chrono::duration<double, std::micro>(std::chrono::steady_clock::now() - t0).count();
        for (const fuel::susp::Rig& r : susp_) nodes += long(r.nodes.size());
        if (++calls % 2000 == 0) std::printf("susp time: %ld calls, mean %.2f us / vehicle, %.1f nodes / vehicle\n", calls, sum / double(calls), double(nodes) / double(calls));
    }
    auto affine = [](const fuel::susp::Affine& a) {
        return simd_matrix(simd_make_float4(a.m[0], a.m[3], a.m[6], 0), simd_make_float4(a.m[1], a.m[4], a.m[7], 0),
                           simd_make_float4(a.m[2], a.m[5], a.m[8], 0), simd_make_float4(a.t[0], a.t[1], a.t[2], 1));
    };
    // ---- [suspanim] end
    std::vector<simd_float4x4> out;
    out.reserve(links_.size());
    for (const PartLink& l : links_) {
        if (susp && l.suspNode >= 0 && l.susp >= 0 && size_t(l.suspNode) < suspOut_[size_t(l.susp)].node.size()) {  // [suspanim]
            out.push_back(simd_mul(chassis, simd_mul(affine(suspOut_[size_t(l.susp)].node[size_t(l.suspNode)]), l.rest)));
        } else if (l.wheel >= 0 && size_t(l.wheel) < st.wheels.size()) {
            simd_float4x4 m = simd_mul(poseMatrix(st.wheels[size_t(l.wheel)]), l.local);
            if (susp && l.susp >= 0 && susp_[size_t(l.susp)].slideWheel == l.wheel && size_t(l.wheel) < suspOut_[size_t(l.susp)].wheelShift.size()) {
                const auto& d = suspOut_[size_t(l.susp)].wheelShift[size_t(l.wheel)];  // [suspanim] FUEL's fork wheel on the fork axis
                m.columns[3] += simd_mul(chassis, simd_make_float4(d[0], d[1], d[2], 0));
            }
            out.push_back(m);
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

simd_float4x4 VehicleRig::nodeWorld(const fuel::VehicleState& st) const { return poseMatrix(st.chassis); }

simd_float3 VehicleRig::focus(const fuel::VehicleState& st) const {
    return simd_mul(poseMatrix(st.chassis), simd_make_float4(0, focusHeight_, 0, 1)).xyz;
}

DriveSession::DriveSession(const fuel::AssembledVehicle& vehicle, const fuel::VehicleSetup& setup, const fuel::Heightfield& terrain,
                           double x, double z, float yaw, const std::vector<uint32_t>& riders)
    : terrain_(terrain), world_(terrain) {
    addVehicle(vehicle, setup, x, z, yaw, riders);
}

int DriveSession::addVehicle(const fuel::AssembledVehicle& vehicle, const fuel::VehicleSetup& setup, double x, double z, float yaw,
                             const std::vector<uint32_t>& riders, const VehicleLods* lods) {
    rigs_.emplace_back(vehicle, setup, riders, lods);
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
        if (st.nitro.amount > 0) {  // [nitro] the FOV kick and the extra speed blur (the blur grows in from 5 to 25 m/s)
            vi.nitroFov = st.nitro.amount * fuel::nitro::params().fovKickDeg;
            vi.nitroBlur = st.nitro.amount * fuel::nitro::params().blur * std::clamp((fuel::fv::len(m->body().S.v) - 5.0f) / 20.0f, 0.0f, 1.0f);
        }
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
    wq.tornado = [](double x, double z, double out[3]) { return StormView::nearestTornado(x, z, out); };  // P61: view 11
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
std::function<float()> DustSystem::playerFlash;  // PK23

void DustSystem::setDataDir(const std::string& dir) { gEffectsDataDir = dir; }
const fuel::sfx::Tables* DustSystem::tables() { return gEffectsTables && gEffectsTables->ok ? gEffectsTables.get() : nullptr; }

void DustSystem::update(float dt, const DriveSession& drive, simd_float3 eye, simd_float3 look) {
    namespace fs = std::filesystem;
    // P77: the blob shadows of every vehicle but the player's (0x5cb8c0 -> 0x5d86b0 per wheel, 0x5d8420 body).
    if (dt > 0) {
        frame_.blobs.clear();
        const simd_float3 camFwd = simd_length(look - eye) > 1e-6f ? simd_normalize(look - eye) : simd_make_float3(0, 0, 0);
        for (int v = 0; v < drive.vehicleCount(); ++v) {
            const fuel::VehicleState st = drive.state(v);
            const VehicleRig& rig = drive.rig(v);
            const std::vector<float>& vp = drive.setup(v).params;
            auto VP = [&](int n) { return size_t(n) < vp.size() ? vp[size_t(n)] : 0.0f; };
            fuel::blob::VehicleInput in;
            in.player = v == 0;
            const auto& q = st.chassis.rotation;
            in.quat = {q[0], q[1], q[2], q[3]};
            // +0x10a4 (0x5cb8c0): |camera - vehicle|, x 3 when the vehicle is behind the camera.
            const simd_float3 pos = simd_make_float3(float(st.chassis.position[0]), float(st.chassis.position[1]), float(st.chassis.position[2]));
            const simd_float3 d = eye - pos;
            in.cameraDistance = simd_length(d) * (simd_dot(camFwd, d) > 0 ? 3.0f : 1.0f);
            const simd_float3 up = simd_act(simd_quaternion(q[0], q[1], q[2], q[3]), simd_make_float3(0, 1, 0));
            for (int w : rig.engineWheels()) {
                if (size_t(w) >= st.wheelFx.size() || size_t(w) >= st.wheels.size()) break;
                const auto& fx = st.wheelFx[size_t(w)];
                in.contacts.push_back({float(fx.contact[0]), float(fx.contact[1]), float(fx.contact[2])});
                in.loads.push_back(fx.load);
                // the drawn wheel's heading: its axle (wheel model x, steering included, spin about x) x the body up
                const auto& wq = st.wheels[size_t(w)].rotation;
                const simd_float3 axle = simd_act(simd_quaternion(wq[0], wq[1], wq[2], wq[3]), simd_make_float3(1, 0, 0));
                const simd_float3 h = simd_cross(axle, up);
                in.headings.push_back({h.x, h.y, h.z});
            }
            in.bodySize = rig.blobBodySize();
            in.pp = {VP(49), VP(50), VP(51), VP(52)};
            blobs_.update(v, dt, drive.raceState(), in, frame_.blobs);
        }
    }
    if (!gEffectsTables && !gEffectsDataDir.empty()) {
        const fs::path d(gEffectsDataDir);
        gEffectsTables = std::make_unique<fuel::sfx::Tables>(fuel::sfx::loadTables(
            (d / "XFinalComponent" / "FUEL.exe").string(), (d / "CommonComponent" / "Param" / "pp001.tsc").string()));
        if (!gEffectsTables->ok) std::fprintf(stderr, "fuelview: no wheel effects: %s\n", gEffectsTables->error.c_str());
    }
    if (!gEffectsTables || !gEffectsTables->ok) return;
    if (!sys_) sys_ = std::make_unique<fuel::sfx::System>(*gEffectsTables);
    if (dt <= 0) return;
    updateVehicleFx(dt, drive, eye, look);  // PK23: before the wheel feeds (0x5d07f0 sets +0x1b5c, read by 0x5ce530)
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
        // The vehicle's up axis: rotate (0, 1, 0) by the body quaternion (0x410510).
        const auto& q = st.chassis.rotation;
        const fuel::sfx::V3 up{2.0f * (q[0] * q[1] - q[3] * q[2]), 1.0f - 2.0f * (q[0] * q[0] + q[2] * q[2]),
                               2.0f * (q[1] * q[2] + q[3] * q[0])};
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
            in.width = VP((w & 1) ? 52 : 51);  // +0x68 = veh+0x69c / 0x6a0 by w & 1
            in.up = up;                         // +0xfc (0x5ceb10)
            in.trackPos = in.contact;           // +0xac (port: the body x offset veh+0x1630[w] taken as 0)
            in.cameraDistSq = in.cameraDistance * in.cameraDistance;
            if (noFx & (1 << int(w))) continue;
            f.active = fx.load > 0.01f && in.cameraDistance <= 200.0f;
            feeds.push_back(f);
        }
        // 0x5ce530: one emitter per vehicle (+0x3c = 1: mist and spray) fed with the mean of wheels 1 and 3,
        // stopped beyond 200 m (veh+0x10a4) or while veh+0x1b5c is set.
        if (st.wheelFx.size() >= 2) {
            const fuel::VehicleState::WheelFx& a = st.wheelFx[1];
            const fuel::VehicleState::WheelFx& b = st.wheelFx[st.wheelFx.size() >= 4 ? 3 : 1];
            fuel::sfx::System::WheelFeed f;
            f.key = v * 16 + 15, f.player = false, f.water = true;
            fuel::sfx::WheelInput& in = f.in;
            for (int k = 0; k < 3; ++k) {
                in.contact[size_t(k)] = float((a.contact[size_t(k)] + b.contact[size_t(k)]) * 0.5);
                in.velocity[size_t(k)] = (a.velocity[size_t(k)] + b.velocity[size_t(k)]) * 0.5f;
                in.forward[size_t(k)] = (a.forward[size_t(k)] + b.forward[size_t(k)]) * 0.5f;
            }
            in.spin = (a.spin + b.spin) * 0.5f, in.radius = VP(50), in.load = (a.load + b.load) * 0.5f;
            in.slip = (a.slip + b.slip) * 0.5f, in.depth = (a.depth + b.depth) * 0.5f;
            in.classes = a.classes;  // veh+0x10f0: wheel 1's fractions
            if (groundColour) in.groundColour = groundColour(in.contact[0], in.contact[2]);
            const simd_float3 c = simd_make_float3(in.contact[0], in.contact[1], in.contact[2]);
            in.cameraDistance = simd_distance(c, eye);
            in.cameraDistSq = in.cameraDistance * in.cameraDistance;
            in.rain = rain, in.up = up, in.width = VP(52);  // +0x68 = veh+0x6a0
            f.active = in.cameraDistance <= 200.0f;
            if (vfx_ && vfx_->fxFlag(size_t(v))) f.active = false;  // PK23: veh+0x1b5c (set by the scrape sparks 0x5d07f0)
            feeds.push_back(f);
        }
    }
    sys_->update(dt, feeds, {eye.x, eye.y, eye.z});
    // ---- PK24: the player's feed for the lens projections (0x5fd270 / 0x5fd390 / 0x5ff8b0)
    lens_ = fuel::wfx::LensVehicle{};
    if (drive.vehicleCount() > 0) {
        const fuel::VehicleState st = drive.state(0);
        lens_.present = true;
        lens_.velocity = {st.velocity[0], st.velocity[1], st.velocity[2]};
        lens_.wheels = int(std::min<size_t>(st.wheelFx.size(), 16));
        for (int w = 0; w < lens_.wheels; ++w) {
            lens_.waterDepth[size_t(w)] = st.wheelFx[size_t(w)].depth;  // veh+0x1990 + 4 w
            if (const fuel::sfx::WheelEmitter* we = sys_->wheel(w); we != nullptr && !we->dust.stopped) lens_.dust = true;
        }
        if (const fuel::fv::FuelVehicle* m = drive.physics().model(0)) {
            lens_.dirtValid = true;
            lens_.dirt = m->damageRecord(2)[1];  // record 2 slot 1 (pm+0x174, p46)
        }
    }
    // ---- PK24 end
    if (std::getenv("FUEL_SPRAY_TRACE"))  // P83: the per-vehicle (0x5ce530) emitters' inputs and which half runs
        for (const fuel::sfx::System::WheelFeed& f : feeds) {
            if (!f.water || !f.active) continue;
            const fuel::sfx::WheelEmitter* we = sys_->wheel(f.key);
            if (!we || (we->spray.stopped && we->mist.stopped)) continue;
            const auto& c = f.in.classes;
            std::printf("spray trace t %.3f veh %d at (%.1f, %.1f, %.1f) depth %.3f rain %.2f mode %d tarmac %.2f wet %.2f spin %.1f%s%s\n",
                        sys_->clock(), f.key / 16, f.in.contact[0], f.in.contact[1], f.in.contact[2], f.in.depth, f.in.rain, we->mode, c[0],
                        c[11] + c[12], f.in.spin, we->spray.stopped ? "" : " SPRAY", we->mist.stopped ? "" : " MIST");
        }
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
    sys_->collectTracks(frame_.decals, frame_.decalCounts);
    if (vfx_) vfx_->collect(frame_.exhaustSmoke, frame_.exhaustFlames, frame_.streaks, {eye.x, eye.y, eye.z});  // PK23
    if (const char* tr = std::getenv("FUEL_PK23_TRACE"); tr && std::atoi(tr) == 3 && !frame_.streaks.empty()) {  // PK23: one streak
        const fuel::vfx::Streak& q = frame_.streaks.front();
        std::fprintf(stderr, "pk23 streak: %zu, first rope rel eye (%.2f %.2f %.2f) (%.2f %.2f %.2f) half %.3f rgba %.1f %.1f %.1f %.2f\n", frame_.streaks.size(),
                     q.rope[0][0] - eye.x, q.rope[0][1] - eye.y, q.rope[0][2] - eye.z, q.rope[3][0] - eye.x, q.rope[3][1] - eye.y, q.rope[3][2] - eye.z,
                     q.halfWidth, q.rgba[0], q.rgba[1], q.rgba[2], q.rgba[3]);
    }
    if (std::getenv("FUEL_SFX_DEBUG") && !frame_.sprites.empty() && int(sys_->clock() * 60) % 60 == 0) {
        const fuel::SfxSprite& s = frame_.sprites.back();
        if (std::atoi(std::getenv("FUEL_SFX_DEBUG")) == 3)
            for (const fuel::SfxSprite& q : frame_.sprites)
                std::fprintf(stderr, "  sprite rel (%.2f %.2f %.2f) a %.2f size %.2f\n", q.posAngle.x - eye.x, q.posAngle.y - eye.y, q.posAngle.z - eye.z,
                             q.colour.w, q.vel.w);
        std::fprintf(stderr, "sfx tracks: %zu decal vertices in %zu ribbons\n", frame_.decals.size(), frame_.decalCounts.size());
        std::fprintf(stderr, "sfx: %zu sprites (%zu gpu), last: pos %.1f %.1f %.1f size %.2f col %.2f %.2f %.2f a %.2f life %.2f gpu %.0f uv %.2f %.2f %.2f %.2f\n",
                     frame_.sprites.size(), sys_->gpuCount(), s.posAngle.x, s.posAngle.y, s.posAngle.z, s.vel.w, s.colour.x, s.colour.y,
                     s.colour.z, s.colour.w, s.life.y, s.life.z, s.uv.x, s.uv.y, s.uv.z, s.uv.w);
    }
    frame_.fadeTo = gEffectsTables->P(5530), frame_.fadeOut = gEffectsTables->P(5531);
}

// ---- PK23: the vehicle effects' feed (core/vehicle_fx; notes pk23.md): per vehicle what 0x5cf2d0 and 0x5d07f0 read.
void DustSystem::updateVehicleFx(float dt, const DriveSession& drive, simd_float3 eye, simd_float3 look) {
    if (std::getenv("FUEL_NO_VEHICLE_FX")) {
        vfx_.reset();
        return;
    }
    if (!vfx_) vfx_ = std::make_unique<fuel::vfx::VehicleFx>(*sys_);
    const simd_float3 camFwd = simd_length(look - eye) > 1e-6f ? simd_normalize(look - eye) : simd_make_float3(0, 0, 0);
    std::vector<fuel::vfx::VehicleIn> in(size_t(std::max(drive.vehicleCount(), 0)));
    fuel::vfx::PlayerIn player;
    for (int v = 0; v < drive.vehicleCount(); ++v) {
        const fuel::VehicleState st = drive.state(v);
        const VehicleRig& rig = drive.rig(v);
        const std::vector<float>& vp = drive.setup(v).params;
        auto VP = [&](int n) { return size_t(n) < vp.size() ? vp[size_t(n)] : 0.0f; };
        const fuel::fv::FuelVehicle* m = drive.physics().model(v);
        fuel::vfx::VehicleIn& x = in[size_t(v)];
        x.throttle = m ? m->engineThrottle() : 0.0f;  // +0x1088
        x.velocity = {st.velocity[0], st.velocity[1], st.velocity[2]};
        x.topSpeed = VP(619);
        x.wheel0Load = st.wheelFx.empty() ? 0.0f : st.wheelFx[0].load;  // +0x15e8
        x.p52 = VP(52);                                                   // +0x6a0
        x.nitro = st.nitro.amount * fuel::nitro::params().flame;          // [nitro] the blue flame
        const fuel::upgvis::Plan& up = rig.upgrades();                     // [upgrades U5] the nitrous power stage's flame
        if (up.active) x.nitroLength = up.flameLength, x.nitroSize = up.flameSize;
        const simd_float3 pos = simd_make_float3(float(st.chassis.position[0]), float(st.chassis.position[1]), float(st.chassis.position[2]));
        const simd_float3 d = eye - pos;
        x.cameraDistance = simd_length(d) * (simd_dot(camFwd, d) > 0 ? 3.0f : 1.0f);  // +0x10a4 (0x5cb8c0)
        {  // +0x42d8 (0x5cbf10's view test 0x771fa0) [port]: a 3 m sphere at the vehicle inside a 52 deg cone around the view
            const float dist = simd_length(d);
            const float c = dist > 1e-3f ? simd_dot(camFwd, -d / dist) : 1.0f;
            const float half = std::min(0.9f + std::asin(std::min(1.0f, 3.0f / std::max(dist, 1e-3f))), 3.1415927f);
            x.visible = dist < 3.0f || c > std::cos(half);
        }
        const simd_float4x4 node = rig.nodeWorld(st);
        for (size_t ei = 0; ei < rig.exhausts().size(); ++ei) {  // 0x6a23b0 -> position, q x (0, 0, -1)
            fuel::AssembledVehicle::Exhaust e = rig.exhausts()[ei];
            // [upgrades U5] with exhaust tips on the SFX_EXHAUST slots, the FX leave the tips' ends
            if (up.active && up.outletsFromSfx && up.stages[0] > 0 && ei < up.outlets.size()) e.pos = up.outlets[ei].pos;
            const simd_float4 p = simd_mul(node, simd_make_float4(e.pos[0], e.pos[1], e.pos[2], 1));
            const simd_float4 q = simd_mul(node, simd_make_float4(e.dir[0], e.dir[1], e.dir[2], 0));
            x.slotPos.push_back({p.x, p.y, p.z});
            x.slotDir.push_back({q.x, q.y, q.z});
        }
        // [nitro] 67 of FUEL's 83 variants have no SFX_EXHAUST boxes (QUAD_ROAD_2, the bikes, ...): with nitrous on they get
        // the port's nitro outlets (VehicleIn::nitroSlotsFrom: the boost's flame only), pointing back and a little up.
        // Bikes and quads (track under 1.2 m): over the rear wheel at 50 / 60 % of the body's height, where their pipes end
        // (one on a bike's right, two 0.3 m either side on a quad; +x = left); the others: behind the chassis boxes, low at
        // bumper height, two at 35 % of the track.
        if (rig.exhausts().empty() && fuel::nitro::enabled() && up.active && !up.outlets.empty() && (up.stages[0] > 0 || up.stages[4] > 0)) {
            // [upgrades U5] the outlets of the plan (the model's exhaust parts / batches, at the tips' ends with an engine stage)
            x.nitroSlotsFrom = x.slotPos.size();
            for (const fuel::upgvis::Outlet& o : up.outlets) {
                const simd_float4 p = simd_mul(node, simd_make_float4(o.pos[0], o.pos[1], o.pos[2], 1));
                const simd_float4 q = simd_mul(node, simd_make_float4(o.dir[0], o.dir[1], o.dir[2], 0));
                x.slotPos.push_back({p.x, p.y, p.z});
                x.slotDir.push_back({q.x, q.y, q.z});
            }
        } else if (rig.exhausts().empty() && fuel::nitro::enabled()) {
            const fuel::VehicleSetup& su = drive.setup(v);
            float rearZ = 1e9f, hubY = 0, r = 0.3f, track = 0, back = 1e9f, yBot = 1e9f, yTop = -1e9f;
            for (const auto& w : su.wheels) {
                track = std::max(track, std::fabs(w.hub[0]));
                if (w.hub[2] < rearZ) rearZ = w.hub[2], hubY = w.hub[1], r = w.radius;
            }
            for (const auto& b : su.chassis)
                back = std::min(back, b.centre[2] - b.halfExtents[2]), yBot = std::min(yBot, b.centre[1] - b.halfExtents[1]),
                yTop = std::max(yTop, b.centre[1] + b.halfExtents[1]);
            if (rearZ < 1e8f) {
                const bool small = su.bike || track < 0.6f;
                const float z = small ? rearZ - 0.5f * r - 0.05f : std::min(rearZ - 0.9f * r, back) - 0.05f;
                const float y = small && yTop > yBot ? yBot + (su.bike ? 0.5f : 0.6f) * (yTop - yBot) : hubY + 0.15f * r;
                const std::vector<float> xs = su.bike ? std::vector<float>{-0.12f}
                                              : small ? std::vector<float>{-0.3f, 0.3f}
                                                      : std::vector<float>{-0.35f * track, 0.35f * track};
                x.nitroSlotsFrom = x.slotPos.size();
                for (float sx : xs) {
                    const simd_float4 p = simd_mul(node, simd_make_float4(sx, y, z, 1));
                    const simd_float4 q = simd_mul(node, simd_make_float4(0, 0.15f, -1, 0));
                    x.slotPos.push_back({p.x, p.y, p.z});
                    x.slotDir.push_back({q.x, q.y, q.z});
                }
            }
        }
        // the identity: the rig and its slot count (= the SFX_EXHAUST count without the nitro outlets, as before)
        x.id = uint64_t(reinterpret_cast<uintptr_t>(&rig)) ^ (uint64_t(x.slotPos.size()) << 56);
        if (m != nullptr) {  // 0x5d07f0's calls of this frame (the physics' contact hook)
            auto& calls = const_cast<fuel::fv::RigidBody&>(m->body()).sparkCalls;
            // The physics works about a moving origin: its model origin vs the drawn chassis gives the shift (as damage_view).
            const fuel::fv::Vec3 mo = m->modelOrigin();
            const double sx = st.chassis.position[0] - mo.x, sy = st.chassis.position[1] - mo.y, sz = st.chassis.position[2] - mo.z;
            for (const auto& c : calls) {
                fuel::vfx::SparkHit h;
                h.a = c.a, h.b = c.b;
                h.point = {float(c.point.x + sx), float(c.point.y + sy), float(c.point.z + sz)};
                h.tangent = {c.tangent.x, c.tangent.y, c.tangent.z};
                h.q = {c.q.x, c.q.y, c.q.z, c.q.w};
                x.hits.push_back(h);
            }
            calls.clear();
        }
        if (v == 0 && m != nullptr) {
            // 0x422510 / 0x423380: the player's gear and speed fraction (every vehicle's 0x5cf2d0 reads them).
            player.valid = true;
            player.gear = m->gear();
            const float top = VP(619);
            player.speedFrac = top > 0 ? std::clamp(fuel::fv::len(m->body().S.v) / (top * 0.86956525f * 0.2777778f), 0.0f, 1.0f) : 0.0f;
            // +0x2ebc: the sound's flash (0x5d9d60), else the same rule here (the pop slot test taken as free).
            const float f = playerFlash ? playerFlash() : -1.0f;
            Backfire& b = backfire_;
            const float rpm = m->soundRpm(), th = m->engineThrottle(), dts = std::max(dt, 1e-4f);
            if (!b.started) b.prevRpm = rpm, b.started = true;
            if (b.flash > 0) b.flash = std::max(0.0f, b.flash - 2.0f * dts);
            b.timer -= dts;
            const float rate = (rpm - b.prevRpm) / dts;
            b.prevRpm = rpm;
            const float span = VP(324) - VP(325);
            const float norm = span != 0 ? (rpm - VP(325)) / span : 0.0f;
            if (!(VP(653) < th || norm < VP(654) || VP(646) <= rate || b.timer >= 0)) {
                b.pm = uint32_t((uint64_t(b.pm) * 16807u) % 2147483647u);
                b.timer = (float(b.pm) / 2147483647.0f * 0.5f + 1.0f) * VP(645);
                b.flash = 1.0f;
            }
            x.flash = f >= 0 ? f : b.flash;
        }
    }
    vfx_->update(dt, in, player, {eye.x, eye.y, eye.z});
    if (std::getenv("FUEL_PK23_TRACE") && int(sys_->clock() * 60) % 30 == 0) {
        const fuel::vfx::Stats& s = vfx_->stats();
        std::fprintf(stderr, "pk23: slots %d smoke %d flames %d spark records %d streaks %d embers %d (emitted %d, spark calls %d); player gear %d frac %.2f flash %.2f\n",
                     s.slots, s.smokeLive, s.flamesOn, s.sparkRecords, s.sparkLive, s.emberLive, s.embersEmitted, s.sparkCalls, player.gear,
                     player.speedFrac, in.empty() ? 0.0f : in[0].flash);
    }
}
// ---- PK23 end

namespace {

uint64_t cellKey(int64_t cx, int64_t cz) { return (uint64_t(uint32_t(int32_t(cx))) << 32) | uint32_t(int32_t(cz)); }

}  // namespace

WorldStreamer::WorldStreamer(const std::vector<const fuel::DpcFile*>& subWorlds, const std::vector<const fuel::DpcFile*>& lookup)
    : lookup_(lookup) {
    for (const fuel::DpcFile* sub : subWorlds) {
        const auto subIndex = uint16_t(subFiles_.size());
        subFiles_.push_back(std::filesystem::path(sub->path()).filename().string());
        for (const fuel::PlacedMesh& pm : fuel::collectPlacedMeshes(*sub, lookup)) addPlacement(pm, subIndex);
        for (fuel::AnimStartVolume& v : fuel::loadAnimStartVolumes(*sub, lookup)) {  // P55: STARTANIMATED volumes
            AnimVolume av;
            static_cast<fuel::AnimStartVolume&>(av) = std::move(v);
            av.sub = subIndex;
            animVolumes_.push_back(std::move(av));
        }
    }
    if (std::getenv("FUEL_STREAM_TRACE")) std::printf("STARTANIMATED volumes: %zu\n", animVolumes_.size());
}

// P55: the player entering a shown STARTANIMATED volume of an active sub-world starts its children once (0x5708b0 ->
// 0x570ae0 -> 0x518170); a RESET_ rule (setMission) re-arms it.
void WorldStreamer::checkAnimVolumes(simd_float3 player) {
    for (AnimVolume& v : animVolumes_) {
        if (v.fired || (!subActive_.empty() && v.sub < subActive_.size() && !subActive_[v.sub])) continue;
        if (v.condition && fuel::evaluateObjectCondition(*v.condition, objects_).value_or(v.condition->defaultHidden)) continue;  // hidden
        if (!v.contains(player.x, player.y, player.z)) continue;
        // ---- PK12: EvenementVolAgent_G vf43 0x5708b0 (vf46 0x570870 checked the player and the armed flag +0x7c).
        const fuel::EventVolumeOptions& o = v.options;
        if (!o.onlyWhenMission.empty() && strcasecmp(o.onlyWhenMission.c_str(), objects_.runningMission.c_str()) != 0) continue;  // 0x57f540 < 0
        // 0x57fe50: a running mission's vt[83] can veto; Mission_Global / OfflineGoto / Template return 0 (0x580130).
        if (o.textInfo > 0 && std::getenv("FUEL_STREAM_TRACE"))  // [not wired] no shipped volume has a text id
            std::printf("EvenementVolAgent %08x: AddInGameTextInfos %d 1.0%s\n", v.node, o.textInfo, o.facing ? " (facing test)" : "");
        if (o.actionInVolume) continue;  // the 0xc9 routine (vf47) instead of the event: not ported (no shipped user)
        // ACTIVE_IF_PREVIOUS_DONE (+0x70) needs 0x410470's previous event node: not ported (no shipped user).
        v.fired = !o.alwaysActive;  // 0x570f60: +0x7c = 1 unless ALWAYS_ACTIVE; SWITCH_MODE 0x570f10 not ported (no user)
        if (o.startAnimated) {
            animStarts_.insert(animStarts_.end(), v.objects.begin(), v.objects.end());
            noteAnimStarts(v.objects);  // PK14: 0x518170 marks the node (+0xd8 = 1)
        }
        for (const fuel::StartFxChild& c : v.startFx) {  // 0x570ae0 STARTFX: 0x5714f0 (+0x84 list), 0x5bc340
            v.started.push_back(c.fire.node);
            startFxFires_.push_back(c.fire);
        }
        // ---- PK12 end
        if (std::getenv("FUEL_STREAM_TRACE"))
            std::printf("STARTANIMATED: volume %08x, %zu objects, %zu STARTFX\n", v.node, o.startAnimated ? v.objects.size() : 0, v.startFx.size());
    }
}

// ---- PK12: hands the STARTFX actions to the renderer, the hidden state of every STARTFX node (the volume's CONDITION_OBJECT
// SONS rules reach G; a detached sub-world's nodes count as gone) and, for volumes whose sub-world is no longer active,
// EvenementVolAgent_G vf25 0x570780: each started node released (0x5bc290).
void WorldStreamer::pushStartFx(fuel::MeshRenderer& renderer, bool dropHidden) {
    // FUEL_SAND_FORCE=1 (diagnostics, not FUEL): every STARTFX child fires once at the first update and stays shown, so
    // the walls can be looked at from anywhere (--at) without driving the race.
    static const bool force = std::getenv("FUEL_SAND_FORCE") != nullptr;
    static bool forced = false;
    if (force && !forced) {
        forced = true;
        for (AnimVolume& v : animVolumes_)
            for (const fuel::StartFxChild& c : v.startFx) startFxFires_.push_back(c.fire), v.started.push_back(c.fire.node);
    }
    for (const fuel::sand::Fire& f : startFxFires_) renderer.startFxFire(f);
    startFxFires_.clear();
    std::unordered_map<uint32_t, bool> hidden;
    for (AnimVolume& v : animVolumes_) {
        if (v.startFx.empty()) continue;
        const bool attached = subActive_.empty() || v.sub >= subActive_.size() || subActive_[v.sub];
        if (!attached && !force) {
            for (uint32_t n : v.started) renderer.startFxRelease(n);
            v.started.clear();
        }
        const bool h = !force && (!attached || (v.condition && fuel::evaluateObjectCondition(*v.condition, objects_).value_or(v.condition->defaultHidden)));
        for (const fuel::StartFxChild& c : v.startFx) hidden[c.fire.node] = h;
        if (dropHidden && h) v.started.clear();  // the retry detaches the course sub-world (vf25 clears +0x84)
    }
    if (dropHidden) {
        std::vector<uint32_t> nodes;
        for (const auto& [n, h] : hidden)
            if (h) nodes.push_back(n);
        if (std::getenv("FUEL_STREAM_TRACE")) std::printf("PK12 KillMission: %zu STARTFX nodes hidden, their records dropped\n", nodes.size());
        renderer.startFxDropHidden(std::move(nodes));
    }
    if (!hidden.empty()) renderer.setStartFxHidden(std::move(hidden));
    if (!sandColourSet_ && ground_ != nullptr) {  // 0x640760: the point context's colour band +0x70 (the decoded ground colour)
        const fuel::GroundGenerator* g = ground_;
        renderer.setSandGroundColour([g](float x, float, float z) {
            const fuel::GroundPoint p = g->at(x, z);
            const fuel::GroundColours c = g->colours(x, z, p);
            return std::array<float, 4>{c.base[0], c.base[1], c.base[2], 1.0f};
        });
        sandColourSet_ = true;
    }
}
// ---- PK12 end

std::vector<WorldStreamer::ObjectSound> WorldStreamer::updateImpacts(float dt, fuel::DriveWorld& physics) {
    std::vector<ObjectSound> out;
    for (const auto& im : physics.takePropImpacts()) {
        auto it = props_.find(im.id);
        if (it == props_.end()) continue;
        it->second.impactPeak = std::max(it->second.impactPeak, im.peak);
        it->second.radius = im.radius;
    }
    for (auto& [id, p] : props_) {
        if (!p.body) continue;
        int symbol = 0;
        float minD = 50, maxD = 500, pitch = 1, volume = 1;
        if (p.debris) {  // 0x5341f0
            if (p.impactTimer > 0) p.impactTimer -= dt;
            if (p.impactPeak > 0.5f) {
                symbol = p.impactPeak > 5.0f ? 0x84 : p.impactPeak > 1.0f ? 0x83 : 0x86;
                minD = 50, maxD = 500;  // node +0x88 = 50, +0x8c = 500
                p.impactPeak = 0, p.impactTimer = 0.5f;
            }
        } else {  // 0x531710
            if (p.impactTimer > 0) {
                p.impactTimer -= dt;
            } else if (p.impactPeak > 1.0f) {
                symbol = p.impactPeak > 5.0f ? 0x83 : 0x84;
                pitch = p.impactPeak * 0.02f + 0.8f;  // 0x637040 -> 0x732bc0: frequency = rate x (0.02 peak + 0.8)
                volume = 0.55f;                       // 0x6370c0 -> 0x732c60(0.55) [I: that field is the volume]
                p.impactPeak = 0, p.impactTimer = 0.5f;
            }
        }
        if (!symbol) continue;
        uint32_t& name = objectNames_[p.objectId];
        if (name == 0)
            for (const fuel::DpcFile* f : lookup_)
                if (const fuel::DpcFile::Entry* e = f->find(p.objectId); e != nullptr && e->header != nullptr) {
                    const auto l = f->link(*e);
                    if (l.size() >= 4) std::memcpy(&name, l.data(), 4);
                    break;
                }
        if (!p.debris) {  // +0xe74 = clamp(r x 50, 50, 500), +0xe78 = half (the sound distances [I])
            maxD = std::clamp(p.radius * 50.0f, 50.0f, 500.0f), minD = maxD * 0.5f;  // r = body +0x408
        }
        {  // P96: 0x733080 scales the owner's distances by its node's world scale (first column length)
            const float sc = std::sqrt(float(p.world[0] * p.world[0] + p.world[4] * p.world[4] + p.world[8] * p.world[8]));
            if (sc > 1e-4f) minD *= sc, maxD *= sc;
        }
        ObjectSound o;
        o.lodName = name, o.symbol = symbol, o.pitch = pitch, o.volume = volume, o.minDist = minD, o.maxDist = maxD;
        o.pos = {p.world[12], p.world[13], p.world[14]};
        out.push_back(o);
    }
    return out;
}

void WorldStreamer::startAnimatedNear(double x, double y, double z, float r) {
    if (animatedScanned_ != placed_.size()) {  // the IT_ANIMATED placements (clone.tsc classes), found once
        for (size_t i = animatedScanned_; i < placed_.size(); ++i)
            if (placed_[i].collisionFirst && gameClass(placed_[i].nodeName) == fuel::GameClass::Animated) animated_.push_back(uint32_t(i));
        animatedScanned_ = placed_.size();
    }
    for (uint32_t i : animated_) {
        const Placed& p = placed_[i];
        if (!active(p)) continue;
        const double dx = p.world[12] - x, dy = p.world[13] - y, dz = p.world[14] - z;
        if (dx * dx + dy * dy + dz * dz <= double(r) * r) {
            animStarts_.push_back(p.objectId);  // 0x518170 (once until reset)
            noteAnimStarts({p.objectId});       // PK14
        }
    }
}

void WorldStreamer::placementsOf(uint32_t lod, std::vector<std::array<float, 16>>& out) const {  // PK29
    out.clear();
    for (const Placed& p : placed_)
        if (p.objectId == lod && p.collisionFirst && active(p)) out.push_back(p.world);
}

size_t WorldStreamer::breakNear(double x, double y, double z, float r, fuel::MeshRenderer& renderer, fuel::DriveWorld& physics) {
    if (decorScanned_ != placed_.size()) {  // the IT_DECORDYNMESH_G placements (registry 0xa666f0), found once
        for (size_t i = decorScanned_; i < placed_.size(); ++i)
            if (gameClass(placed_[i].nodeName) == fuel::GameClass::DecorDynMesh) decorBreak_.push_back(uint32_t(i));
        decorScanned_ = placed_.size();
    }
    const double r2 = double(r) * r;
    size_t n = 0;
    bool changed = false, autoChanged = false;
    for (uint32_t i : decorBreak_) {
        Placed& p = placed_[i];
        if (!active(p)) continue;  // intact (+0x6c == 0) and the node not hidden (0x40000)
        const double dx = p.world[12] - x, dy = p.world[13] - y, dz = p.world[14] - z;
        if (!(dx * dx + dy * dy + dz * dz < r2)) continue;
        std::array<double, 16> m;
        for (size_t k = 0; k < 16; ++k) m[k] = p.world[k];
        p.broken = true;
        for (auto s = cellSolid_.begin(); s != cellSolid_.end(); ++s)
            if (s->first == p.objectId && s->second == m) {
                cellSolid_.erase(s);
                break;
            }
        breakables_.erase(std::remove_if(breakables_.begin(), breakables_.end(), [&](const Breakable& b) { return b.placed == int64_t(i); }), breakables_.end());
        spawnPieces(p.objectId, m, physics);  // 0x515cc0
        changed = true, ++brokenCount_, ++n;
    }
    for (auto it = autoBreakables_.begin(); it != autoBreakables_.end();) {
        const double dx = it->m[12] - x, dy = it->m[13] - y, dz = it->m[14] - z;
        if (!(dx * dx + dy * dy + dz * dz < r2)) {
            ++it;
            continue;
        }
        {
            std::lock_guard<std::mutex> l(takenMutex_);
            brokenAuto_.push_back({float(it->m[12]), float(it->m[14])});
        }
        for (auto s = autoSolid_.begin(); s != autoSolid_.end(); ++s)
            if (s->first == it->objectId && s->second == it->m) {
                autoSolid_.erase(s);
                break;
            }
        const float bx = float(it->m[12]), bz = float(it->m[14]);  // as breakThrough: the template's parts at the root go too
        autoObjects_.erase(std::remove_if(autoObjects_.begin(), autoObjects_.end(),
                                          [&](const fuel::PlacedMesh& q) { return std::fabs(q.world[12] - bx) < 0.05f && std::fabs(q.world[14] - bz) < 0.05f; }),
                           autoObjects_.end());
        spawnPieces(it->objectId, it->m, physics);
        it = autoBreakables_.erase(it), autoChanged = true, ++brokenCount_, ++n;
    }
    if (changed) refreshHidden(renderer);
    if (autoChanged) pushAutoDraw(renderer);
    if (changed || autoChanged) {
        applySolids(physics);
        updateProps(renderer, physics, true);
        if (std::getenv("FUEL_STREAM_TRACE")) std::printf("tornado break: %zu broken (%zu total), %zu pieces\n", n, brokenCount_, piecesSpawned_);
    }
    return n;
}

void WorldStreamer::addPlacement(const fuel::PlacedMesh& pm, uint16_t sub) {
    const float x = pm.world[12], z = pm.world[14];
    if (std::fabs(x) < 50 && std::fabs(z) < 50) return;  // stray objects at the origin
    const auto index = uint32_t(placed_.size());
    Placed p;
    static_cast<fuel::PlacedMesh&>(p) = pm;
    p.sub = sub;
    if (p.condition) {
        p.hidden = p.condition->defaultHidden;
        p.hidden = fuel::evaluateObjectCondition(*p.condition, objects_).value_or(p.hidden);
    }
    placed_.push_back(std::move(p));
    cells_[cellKey(int64_t(std::floor(x / kCell)), int64_t(std::floor(z / kCell)))].push_back(index);
    addObstacle(pm);
    lodDirty_ = true;
}

void WorldStreamer::addResident(const fuel::DpcFile& usa1) {
    const auto subIndex = uint16_t(subFiles_.size());
    subFiles_.push_back(std::filesystem::path(usa1.path()).filename().string());
    if (!subActive_.empty()) subActive_.push_back(1), subRef_.push_back(-1);
    size_t n = 0;
    for (const fuel::PlacedMesh& pm : fuel::collectWorldRootMeshes(usa1, lookup_)) addPlacement(pm, subIndex), ++n;
    if (std::getenv("FUEL_STREAM_TRACE")) std::printf("USA1 world-root placements: %zu\n", n);
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
    objects_.runningMission = mission;  // M_RUNNING (p38)
    evaluateConditions();
    // P55: RESET_M_START <mission> (action 4, 0x513ca0 -> the agent's reset 0x51b930) on the mission's start event.
    if (!mission.empty()) {
        fuel::ObjectEvent ev;
        ev.kind = fuel::ObjectCondition::Start, ev.mission = mission;
        for (size_t i = 0; i < placed_.size(); ++i) {
            if (!placed_[i].condition) continue;
            std::vector<const fuel::ObjectCondition::Rule*> fired;
            (void)fuel::evaluateObjectCondition(*placed_[i].condition, objects_, ev, &fired);
            for (const auto* r : fired)
                if (r->action == fuel::ObjectCondition::Reset) {
                    resetPending_.push_back(uint32_t(i));
                    break;
                }
        }
        for (AnimVolume& v : animVolumes_) {  // RESET_M_START on a STARTANIMATED volume: SONS -> its children reset
            if (!v.condition) continue;
            std::vector<const fuel::ObjectCondition::Rule*> fired;
            (void)fuel::evaluateObjectCondition(*v.condition, objects_, ev, &fired);
            for (const auto* r : fired)
                if (r->action == fuel::ObjectCondition::Reset) {
                    v.fired = false;
                    animResets_.insert(animResets_.end(), v.objects.begin(), v.objects.end());
                    break;
                }
        }
    }
    last_ = {1e30f, 0, 0};
}

void WorldStreamer::setMissionState(std::function<bool(const std::string&)> open, std::function<bool(const std::string&)> won) {
    objects_.missionOpen = std::move(open);
    objects_.missionWon = std::move(won);
    evaluateConditions();
    last_ = {1e30f, 0, 0};
}

void WorldStreamer::setRtcRunning(bool on) {
    if (objects_.rtcRunning == on) return;
    objects_.rtcRunning = on;
    evaluateConditions();
    last_ = {1e30f, 0, 0};
}

// 0x513630 for every placement under a CONDITION_OBJECT (p38): e.g. "DEFAULTHIDE ;
// UNHIDE_M_RUNNING CR2-Yell-CP" shows course dressing only while that race runs.
void WorldStreamer::evaluateConditions() {
    size_t shown = 0, hidden = 0, byRtc = 0;
    for (Placed& p : placed_) {
        if (!p.condition) continue;
        byRtc += p.condition->hideByRtc;
        // DONTHIDEUNHIDE: the rules leave the visibility as it was (shown, unless DEFAULTHIDE hid it at load).
        p.hidden = fuel::evaluateObjectCondition(*p.condition, objects_).value_or(p.hidden);  // nothing: left as it is
        (p.hidden ? hidden : shown)++;
    }
    lodDirty_ = true;
    if (std::getenv("FUEL_STREAM_TRACE"))
        std::printf("object conditions: %zu placements shown, %zu hidden (%zu HIDEBYRTC, RTC %s)\n", shown, hidden, byRtc, objects_.rtcRunning ? "on" : "off");
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
        if (on && !subActive_[f]) freshSub(f);  // PK14: WorldRef_Z::Attach 0x740690 binds the sub-world anew
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

namespace {

// Row-vector transform of an object-space point.
std::array<double, 3> xform(const std::array<double, 16>& m, double x, double y, double z) {
    return {x * m[0] + y * m[4] + z * m[8] + m[12], x * m[1] + y * m[5] + z * m[9] + m[13], x * m[2] + y * m[6] + z * m[10] + m[14]};
}
std::array<double, 3> xdir(const std::array<double, 16>& m, double x, double y, double z) {
    return {x * m[0] + y * m[4] + z * m[8], x * m[1] + y * m[5] + z * m[9], x * m[2] + y * m[6] + z * m[10]};
}
double dot3(const std::array<double, 3>& a, const std::array<double, 3>& b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
std::array<double, 3> sub3(const std::array<double, 3>& a, const std::array<double, 3>& b) { return {a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }

// Distance from p to triangle abc (Ericson's closest point).
double pointTriangle(const std::array<double, 3>& p, const std::array<double, 3>& a, const std::array<double, 3>& b, const std::array<double, 3>& c) {
    const auto ab = sub3(b, a), ac = sub3(c, a), ap = sub3(p, a);
    auto at = [&](double v, double w) {
        const std::array<double, 3> q{a[0] + ab[0] * v + ac[0] * w, a[1] + ab[1] * v + ac[1] * w, a[2] + ab[2] * v + ac[2] * w};
        const auto d = sub3(p, q);
        return std::sqrt(dot3(d, d));
    };
    const double d1 = dot3(ab, ap), d2 = dot3(ac, ap);
    if (d1 <= 0 && d2 <= 0) return at(0, 0);
    const auto bp = sub3(p, b);
    const double d3 = dot3(ab, bp), d4 = dot3(ac, bp);
    if (d3 >= 0 && d4 <= d3) return at(1, 0);
    const double vc = d1 * d4 - d3 * d2;
    if (vc <= 0 && d1 >= 0 && d3 <= 0) return at(d1 / (d1 - d3), 0);
    const auto cp = sub3(p, c);
    const double d5 = dot3(ab, cp), d6 = dot3(ac, cp);
    if (d6 >= 0 && d5 <= d6) return at(0, 1);
    const double vb = d5 * d2 - d1 * d6;
    if (vb <= 0 && d2 >= 0 && d6 <= 0) return at(0, d2 / (d2 - d6));
    const double va = d3 * d6 - d5 * d4;
    if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) {
        const double w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return at(1 - w, w);
    }
    const double den = 1 / (va + vb + vc);
    return at(vb * den, vc * den);
}

// Distance from world point p to the object's FUEL collision placed at m (0 inside a box).
double collisionDistance(const fuel::StaticCollision& c, const std::array<double, 16>& m, const std::array<double, 3>& p) {
    double best = 1e30;
    const double s = (std::sqrt(dot3({m[0], m[1], m[2]}, {m[0], m[1], m[2]})) + std::sqrt(dot3({m[4], m[5], m[6]}, {m[4], m[5], m[6]})) +
                      std::sqrt(dot3({m[8], m[9], m[10]}, {m[8], m[9], m[10]}))) / 3;
    for (const auto& sp : c.spheres) {
        const auto d = sub3(p, xform(m, sp.c[0], sp.c[1], sp.c[2]));
        best = std::min(best, std::max(0.0, std::sqrt(dot3(d, d)) - sp.r * s));
    }
    for (const auto& b : c.boxes) {
        const auto ctr = xform(m, b.frame[3], b.frame[7], b.frame[11]);
        const auto d = sub3(p, ctr);
        double out2 = 0;
        for (int a = 0; a < 3; ++a) {
            auto ax = xdir(m, b.frame[size_t(4 * a)], b.frame[size_t(4 * a + 1)], b.frame[size_t(4 * a + 2)]);
            const double len = std::sqrt(dot3(ax, ax));
            if (len < 1e-9) continue;
            const double e = std::fabs(dot3(d, ax) / len) - b.half[size_t(a)] * len;
            if (e > 0) out2 += e * e;
        }
        best = std::min(best, std::sqrt(out2));
    }
    for (const auto& t : c.tris)
        best = std::min(best, pointTriangle(p, xform(m, t[0], t[1], t[2]), xform(m, t[3], t[4], t[5]), xform(m, t[6], t[7], t[8])));
    return best;
}

std::array<double, 16> toDouble(const std::array<float, 16>& w) {
    std::array<double, 16> m;
    for (size_t k = 0; k < 16; ++k) m[k] = w[k];
    return m;
}

}  // namespace

std::vector<std::string> WorldStreamer::solidsAt(double x, double y, double z, double radius) const {
    std::vector<std::string> hits;
    const std::array<double, 3> p{x, y, z};
    char buf[160];
    auto test = [&](uint32_t key, const std::array<double, 16>& m, const char* what) {
        if (std::hypot(m[12] - x, m[14] - z) > 300) return;
        const fuel::StaticCollision& c = collision(key);
        const double d = collisionDistance(c, m, p);
        if (d >= radius) return;
        std::snprintf(buf, sizeof buf, "%s object %08x at (%.1f, %.1f, %.1f), %.2f m", what, key, m[12], m[13], m[14], d);
        hits.push_back(buf);
    };
    for (const auto& [key, m] : cellSolid_) test(key, m, "placed static");
    for (const auto& [key, m] : autoSolid_) test(key, m, "automesh static");
    for (const auto& [i, prop] : props_) test(prop.objectId, toDouble(prop.world), prop.piece ? "piece" : "movable prop");
    for (const auto& m : trunks_) {
        const double r = m[0], h = m[5], d = std::hypot(m[12] - x, m[14] - z) - r;
        if (d < radius && y > m[13] - radius && y < m[13] + h + radius) {
            std::snprintf(buf, sizeof buf, "trunk r %.2f at (%.1f, %.1f, %.1f), %.2f m", r, m[12], m[13] + 2.0, m[14], std::max(0.0, d));
            hits.push_back(buf);
        }
    }
    return hits;
}

WorldStreamer::CollisionStats WorldStreamer::collisionStats() const {
    CollisionStats st;
    static std::unordered_map<uint32_t, float> renderRadius;  // main's rule: the render mesh's radius
    static std::unordered_map<uint32_t, std::array<float, 4>> renderSphere;
    auto radiusOf = [&](uint32_t mesh) {
        auto it = renderRadius.find(mesh);
        if (it != renderRadius.end()) return it->second;
        float r = -1;
        try {
            const fuel::CollisionMesh cm = fuel::loadCollisionMesh(lookup_, mesh);
            if (!cm.indices.empty()) r = cm.radius;
        } catch (const std::exception&) {
        }
        return renderRadius.emplace(mesh, r).first->second;
    };
    auto sphereOf = [&](uint32_t mesh) {
        auto it = renderSphere.find(mesh);
        if (it != renderSphere.end()) return it->second;
        std::array<float, 4> s{0, 0, 0, -1};
        for (const fuel::DpcFile* f : lookup_)
            if (const fuel::DpcFile::Entry* e = f->find(mesh); e != nullptr && e->header != nullptr && e->classCrc == fuel::cls::Mesh) {
                try {
                    s = fuel::parseMeshLink(f->link(*e)).sphere;
                } catch (const std::exception&) {
                }
                break;
            }
        return renderSphere.emplace(mesh, s).first->second;
    };
    // Does a solid placement show? Collision outside the drawn bounds, or nothing drawn at 10 m.
    auto visibility = [&](const fuel::PlacedMesh& p, const char* where) {
        const fuel::StaticCollision& c = collision(p.objectId);
        const std::array<float, 4> sph = sphereOf(p.meshId);
        bool outside = false;
        const std::vector<float> pts = fuel::collisionHullPoints(c);
        if (sph[3] > 0)
            for (size_t k = 0; k + 2 < pts.size(); k += 3) {
                const double dx = pts[k] - sph[0], dy = pts[k + 1] - sph[1], dz = pts[k + 2] - sph[2];
                if (std::sqrt(dx * dx + dy * dy + dz * dz) > sph[3] + 1.0) outside = true;
            }
        const bool invisible = p.meshId == 0 || sph[3] <= 0 || radiusOf(p.meshId) < 0 || fuel::placementAlpha(p, 0, 10.0f) < 0.01f;  // no render triangles, or faded
        if (outside) ++st.outsideMesh;
        if (invisible) ++st.invisibleNear;
        if ((outside || invisible) && st.examples.size() < 12) {
            char b[200];
            std::snprintf(b, sizeof b, "%s %s object %08x mesh %08x at (%.0f, %.0f, %.0f): %s%s", where, fuel::gameClassName(gameClass(p.nodeName)), p.objectId,
                          p.meshId, p.world[12], p.world[13], p.world[14], outside ? "collision outside the mesh " : "", invisible ? "not drawn at 10 m" : "");
            st.examples.push_back(b);
        }
    };
    const simd_float3 at = last_;
    const int64_t r = int64_t(std::ceil(kCollideRadius / kCell));
    const int64_t cx = int64_t(std::floor(at.x / kCell)), cz = int64_t(std::floor(at.z / kCell));
    for (int64_t dz = -r; dz <= r; ++dz)
        for (int64_t dx = -r; dx <= r; ++dx) {
            auto it = cells_.find(cellKey(cx + dx, cz + dz));
            if (it == cells_.end()) continue;
            for (uint32_t i : it->second) {
                const Placed& p = placed_[i];
                const float d = std::hypot(p.world[12] - at.x, p.world[14] - at.z);
                if (d > kCollideRadius || !active(p)) continue;
                const std::string cls = fuel::gameClassName(gameClass(p.nodeName));
                // main 1efc828
                const float rr = radiusOf(p.meshId);
                if (d <= kPropRadius && rr >= 0.4f && rr < 3.0f) {
                    ++st.classes["placed movable: " + cls].main;
                    if (collision(p.objectId).empty() || !p.collisionFirst) ++st.classes["  of which FUEL has no collision for: " + cls].main;
                }
                else if (p.collisionFirst && fuel::inPhysicsScene(p.nodeFlags) && !collision(p.objectId).empty()) ++st.classes["placed static: " + cls].main;
                // this build
                const Solid k = solidKind(p);
                if (k == Solid::None || collision(p.objectId).empty()) continue;
                const bool prop = k == Solid::Movable && d <= kPropRadius;
                ++st.classes[(prop ? "placed movable: " : "placed static: ") + cls].now;
                visibility(p, prop ? "movable" : "static");
            }
        }
    for (size_t i = 0; i < autoObjects_.size(); ++i) {
        const fuel::PlacedMesh& p = autoObjects_[i];
        if (std::hypot(p.world[12] - at.x, p.world[14] - at.z) > kCollideRadius) continue;
        const bool road = i >= autoRoadFrom_;
        const std::string cls = std::string(road ? "automesh road: " : "automesh cell: ") + fuel::gameClassName(gameClass(p.nodeName));
        const fuel::GameClass gc = gameClass(p.nodeName);
        if (!road && p.collisionFirst && fuel::inPhysicsScene(p.nodeFlags) && gc != fuel::GameClass::Collect && !collision(p.objectId).empty())
            ++st.classes[cls].main;
        if (gc != fuel::GameClass::Collect && solidKind(p) == Solid::Static && !collision(p.objectId).empty()) {
            ++st.classes[cls].now;
            visibility(p, "automesh");
        } else if (p.collisionFirst && autoProps_.count(autoPropKey(p))) {  // P55: AutoMesh IT_DYNOBJ_G props
            ++st.classes[std::string(road ? "automesh road movable: " : "automesh cell movable: ") + fuel::gameClassName(gc)].now;
        }
    }
    st.classes["trees"].now = treeTrunks_, st.classes["trees"].main = treeTrunks_;
    st.classes["bushes"].now = bushTrunks_;
    return st;
}

void WorldStreamer::refreshHidden(fuel::MeshRenderer& renderer) {
    std::vector<uint32_t> hidden;
    for (const auto& [i, prop] : props_)
        if (i < kAutoPropId && i < lodIndex_.size() && lodIndex_[i] >= 0) hidden.push_back(uint32_t(lodIndex_[i]));
    for (size_t i = 0; i < placed_.size() && i < lodIndex_.size(); ++i)
        if (placed_[i].broken && lodIndex_[i] >= 0) hidden.push_back(uint32_t(lodIndex_[i]));
    renderer.setLodHidden(hidden);
}

void WorldStreamer::breakThrough(const DriveSession& drive, float dt, fuel::MeshRenderer& renderer, fuel::DriveWorld& physics) {
    if ((breakables_.empty() && autoBreakables_.empty()) || dt <= 0) return;
    static const bool dbg = std::getenv("FUEL_P50_BREAKDBG") != nullptr;
    // A moving body: proxy shapes (world centre, bounding radius), velocity, and the approach speed that breaks.
    struct Mover {
        std::vector<std::pair<std::array<double, 3>, double>> shapes;
        std::array<double, 3> x{}, v{};
        double speed = 0, threshold = 0;
        int vehicle = -1;  // -1: a dynamic object
    };
    std::vector<Mover> movers;
    // P55: every vehicle (FUEL's contact hook 0x51f6c0 -> 0x51edb0 runs for each IT_Dyn_VehicleMove_G).
    for (int i = 0; i < drive.vehicleCount(); ++i) {
        const std::vector<float>& P = drive.setup(i).params;
        if (P.size() <= 575) continue;
        const fuel::VehicleState st = drive.state(i);
        Mover mv;
        mv.vehicle = i;
        mv.x = {st.chassis.position[0], st.chassis.position[1], st.chassis.position[2]};
        mv.v = {st.velocity[0], st.velocity[1], st.velocity[2]};
        mv.speed = std::sqrt(dot3(mv.v, mv.v));
        mv.threshold = P[575];
        if (dbg && i == 0) {
            static int n = 0;
            if (++n % 60 == 0) std::printf("  breakThrough: %zu + %zu breakables, speed %.1f m/s, P575 %.1f\n", breakables_.size(), autoBreakables_.size(), mv.speed, mv.threshold);
        }
        if (mv.speed < mv.threshold) continue;
        // Touching: a proxy shape of the vehicle (VehicleProxy boxes and spheres, bounding radii, placed with the
        // chassis pose; port: about the chassis position, not the centre of mass) reaches the object within this
        // step; the approach speed is the rate the distance closes (0x51edb0 reads the contact's normal approach R[0x33]).
        const auto& q = st.chassis.rotation;  // x, y, z, w
        const std::array<double, 3> x = mv.x;
        auto rot = [&](const std::array<float, 3>& c) {
            const double qx = q[0], qy = q[1], qz = q[2], qw = q[3];
            const double tx = 2 * (qy * c[2] - qz * c[1]), ty = 2 * (qz * c[0] - qx * c[2]), tz = 2 * (qx * c[1] - qy * c[0]);
            return std::array<double, 3>{x[0] + c[0] + qw * tx + (qy * tz - qz * ty), x[1] + c[1] + qw * ty + (qz * tx - qx * tz),
                                         x[2] + c[2] + qw * tz + (qx * ty - qy * tx)};
        };
        for (const auto& bx : drive.setup(i).proxy.boxes) mv.shapes.push_back({rot(bx.centre), bx.radius});
        for (const auto& sp : drive.setup(i).proxy.spheres) mv.shapes.push_back({rot(sp.centre), sp.radius});
        if (mv.shapes.empty()) mv.shapes.push_back({x, 1.0});
        movers.push_back(std::move(mv));
    }
    // P55: dynamic objects in motion (IT_DYNOBJMove_G vt[141] 0x531c00: contact approach above 5 m/s breaks).
    for (const auto& [id, prop] : props_) {
        if (!prop.body || !prop.moved) continue;
        std::array<float, 3> v;
        if (!physics.propVelocity(id, v)) continue;
        Mover mv;
        mv.v = {v[0], v[1], v[2]};
        mv.speed = std::sqrt(dot3(mv.v, mv.v));
        mv.threshold = fuel::kDynBreakSpeed;
        if (mv.speed < mv.threshold) continue;
        mv.x = {prop.world[12], prop.world[13], prop.world[14]};
        const float r = collision(prop.objectId).radius * std::max({prop.scale[0], prop.scale[1], prop.scale[2]});
        mv.shapes.push_back({mv.x, std::max(0.1f, r)});
        movers.push_back(std::move(mv));
    }
    if (movers.empty()) return;
    bool changed = false, autoChanged = false;
    auto hit = [&](const Breakable& b) {
        for (const Mover& mv : movers) {
            if (std::hypot(b.m[12] - mv.x[0], b.m[14] - mv.x[2]) > 40 + mv.speed * dt) continue;
            const fuel::StaticCollision& c = collision(b.objectId);
            double gap = 1e30, approach = 0;
            for (const auto& [ctr, r] : mv.shapes) {
                const std::array<double, 3> c1{ctr[0] + mv.v[0] * dt, ctr[1] + mv.v[1] * dt, ctr[2] + mv.v[2] * dt};
                const double d0 = collisionDistance(c, b.m, ctr), d1 = collisionDistance(c, b.m, c1);
                if (d1 - r < gap) gap = d1 - r, approach = (d0 - d1) / dt;
            }
            if (dbg && gap < 2) std::printf("  breakable %08x gap %.2f approach %.1f m/s (break at %.1f, vehicle %d)\n", b.objectId, gap, approach, mv.threshold, mv.vehicle);
            if (gap < 0.05 && approach >= mv.threshold) {
                if (std::getenv("FUEL_STREAM_TRACE"))
                    std::printf("break-through of %08x by %s %d at %.0f km/h (threshold %.1f m/s)\n", b.objectId, mv.vehicle >= 0 ? "vehicle" : "object", mv.vehicle,
                                mv.speed * 3.6, mv.threshold);
                return true;
            }
        }
        return false;
    };
    for (auto it = breakables_.begin(); it != breakables_.end();) {
        if (!hit(*it)) {
            ++it;
            continue;
        }
        placed_[size_t(it->placed)].broken = true;
        for (auto s = cellSolid_.begin(); s != cellSolid_.end(); ++s)
            if (s->first == it->objectId && s->second == it->m) {
                cellSolid_.erase(s);
                break;
            }
        spawnPieces(it->objectId, it->m, physics);
        it = breakables_.erase(it), changed = true, ++brokenCount_;
    }
    for (auto it = autoBreakables_.begin(); it != autoBreakables_.end();) {
        if (!hit(*it)) {
            ++it;
            continue;
        }
        {
            std::lock_guard<std::mutex> l(takenMutex_);
            brokenAuto_.push_back({float(it->m[12]), float(it->m[14])});
        }
        for (auto s = autoSolid_.begin(); s != autoSolid_.end(); ++s)
            if (s->first == it->objectId && s->second == it->m) {
                autoSolid_.erase(s);
                break;
            }
        // Hide every AutoMesh part of the broken spawn (the template's nodes share the root's position only for
        // the root; parts within 0.05 m of it go too).
        const float bx = float(it->m[12]), bz = float(it->m[14]);
        autoObjects_.erase(std::remove_if(autoObjects_.begin(), autoObjects_.end(),
                                          [&](const fuel::PlacedMesh& p) { return std::fabs(p.world[12] - bx) < 0.05f && std::fabs(p.world[14] - bz) < 0.05f; }),
                           autoObjects_.end());
        spawnPieces(it->objectId, it->m, physics);
        it = autoBreakables_.erase(it), autoChanged = true, ++brokenCount_;
    }
    if (changed) refreshHidden(renderer);
    if (autoChanged) pushAutoDraw(renderer);
    if (changed || autoChanged) {
        applySolids(physics);
        updateProps(renderer, physics, true);
        if (std::getenv("FUEL_STREAM_TRACE")) std::printf("break-through: %zu broken, %zu pieces\n", brokenCount_, piecesSpawned_);
    }
}

// ---- P55: FUEL's dynamic objects (core/dyn_object.hpp) ----

const fuel::DynObjUserDefine& WorldStreamer::dynUserDefine(uint32_t objectId) const {
    auto it = dynUserDefine_.find(objectId);
    if (it != dynUserDefine_.end()) return it->second;
    fuel::DynObjUserDefine ud;
    try {  // the Lod's UserDefine (Lod +0x88; 0x51c170 reads node +0x98 -> +0x88)
        for (const fuel::DpcFile* f : lookup_)
            if (const fuel::DpcFile::Entry* e = f->find(objectId); e && e->header) {
                if (e->classCrc != fuel::cls::Lod) break;
                const fuel::Lod lod = fuel::parseLodBody(*f->body(*e));
                if (lod.userDefine == 0) break;
                for (const fuel::DpcFile* g : lookup_)
                    if (const fuel::DpcFile::Entry* u = g->find(lod.userDefine); u && u->header) {
                        if (const auto body = g->body(*u)) ud = fuel::parseDynObjUserDefine(fuel::parseUserDefineBody(*body));
                        break;
                    }
                break;
            }
    } catch (const std::exception&) {
    }
    return dynUserDefine_.emplace(objectId, std::move(ud)).first->second;
}

float WorldStreamer::propMass(uint32_t objectId, float scale) const {
    return float(fuel::dynObjMass(fuel::dynObjVolume(collision(objectId), scale), dynUserDefine(objectId), dynParams_));
}

uint64_t WorldStreamer::autoPropKey(const fuel::PlacedMesh& p) {
    const auto qx = uint64_t(int64_t(std::llround(double(p.world[12]) * 20.0))) & 0xffffffu;
    const auto qz = uint64_t(int64_t(std::llround(double(p.world[14]) * 20.0))) & 0xffffffu;
    return (uint64_t(p.nodeName & 0xffffu) << 48) | (qx << 24) | qz;
}

// A movable body for placement `p` (its pose, scale, FUEL mass, meshes); false when FUEL has no body shape for it.
bool WorldStreamer::makeProp(Prop& prop, const fuel::PlacedMesh& p, fuel::DriveWorld& physics) {
    if (!dynParamsLoaded_) {
        dynParamsLoaded_ = true;
        for (const fuel::DpcFile* f : lookup_) {  // <data>/CommonComponent/WORLD/USA1.DPC -> <data>/CommonComponent/Param/pp001.tsc
            const std::filesystem::path path(f->path());
            if (strcasecmp(path.filename().c_str(), "USA1.DPC") == 0) dynParams_ = fuel::loadDynObjParams((path.parent_path().parent_path() / "Param" / "pp001.tsc").string());
        }
        if (!dynParams_.loaded) std::fprintf(stderr, "fuelview: pp001.tsc PP150/PP151 not found: dynamic objects use 50 / 5 kg/m^3\n");
    }
    const fuel::StaticCollision& col = collision(p.objectId);
    if (col.empty()) return false;
    if (!physics.hasPropShape(p.objectId)) physics.addPropShape(p.objectId, col);
    if (!physics.hasPropShape(p.objectId)) return false;
    prop.world = p.world;
    for (int r = 0; r < 3; ++r) prop.scale[r] = std::max(1e-4f, std::hypot(p.world[size_t(r * 4)], p.world[size_t(r * 4 + 1)], p.world[size_t(r * 4 + 2)]));
    prop.objectId = p.objectId;
    prop.meshes = {p.meshId};
    prop.parts = {p};  // P94
    prop.origin = {p.world[12], p.world[13], p.world[14]};
    // IT_DYNOBJMove_G's mass and drag (D2). P61: IT_DebrisMove (vt 0x9aef2c): mass = volume (0x712de0) x density +0xe24 =
    // 100 (0x533cb0; PhysObj_SetMassFromParam 0x712d90, explicit +0xe3c never set [I]), its own drag 0x533790.
    const float sc = std::cbrt(prop.scale[0] * prop.scale[1] * prop.scale[2]);
    const fuel::GameClass gc = gameClass(p.nodeName);
    prop.debris = gc == fuel::GameClass::Debris;
    prop.mass = gc == fuel::GameClass::DynObj ? propMass(p.objectId, sc) : prop.debris ? float(100.0 * fuel::dynObjVolume(col, sc)) : 0.0f;
    static const bool trace = std::getenv("FUEL_P55_PROPTRACE") != nullptr;
    if (trace)
        std::printf("prop %08x %s at (%.0f, %.0f): %zu spheres, %zu boxes, volume %.3f m^3, mass %.1f kg\n", p.objectId, fuel::gameClassName(gameClass(p.nodeName)),
                    p.world[12], p.world[14], col.spheres.size(), col.boxes.size(), fuel::dynObjVolume(col, std::cbrt(prop.scale[0] * prop.scale[1] * prop.scale[2])), prop.mass);
    return true;
}

void WorldStreamer::spawnPieces(uint32_t decorObject, const std::array<double, 16>& m, fuel::DriveWorld& physics) {
    static const bool off = std::getenv("FUEL_NO_PIECES") != nullptr;
    if (off) return;
    if (lodByName_.empty()) {  // the pieces are Lod_Z objects named after their agents (USA1, e.g. BarriereBois01_DYN00)
        for (const fuel::DpcFile* f : lookup_)
            for (uint32_t id : f->ids(fuel::cls::Lod)) try {
                    if (const fuel::DpcFile::Entry* e = f->find(id)) lodByName_.emplace(fuel::parseObjectLink(f->link(*e)).name, id);
                } catch (const std::exception&) {
                }
    }
    // 0x713f20: the piece's body takes the decor node's world rotation and position (node scale 1, 0x51b930).
    std::array<float, 16> pose{};
    for (int r = 0; r < 3; ++r) {
        const double len = std::max(1e-6, std::hypot(m[size_t(r * 4)], m[size_t(r * 4 + 1)], m[size_t(r * 4 + 2)]));
        for (int c = 0; c < 3; ++c) pose[size_t(r * 4 + c)] = float(m[size_t(r * 4 + c)] / len);
    }
    pose[12] = float(m[12]), pose[13] = float(m[13]), pose[14] = float(m[14]), pose[15] = 1;
    for (const std::string& name : dynUserDefine(decorObject).names) {
        const uint32_t crc = fuel::asoboCrc(name);
        if (gameClass(crc) != fuel::GameClass::DynObj) continue;  // 0x4f16f0 finds IT_DYNOBJ_G agents by name (clone.tsc)
        const auto lod = lodByName_.find(crc);
        if (lod == lodByName_.end()) continue;
        std::vector<fuel::PlacedMesh> meshes = fuel::collectObjectMeshes(lookup_, lod->second);
        if (meshes.empty()) continue;
        fuel::PlacedMesh pm = meshes.front();
        pm.world = pose;
        pm.nodeName = crc;
        Prop prop;
        if (!makeProp(prop, pm, physics)) continue;
        prop.meshes.clear();
        for (const auto& mm : meshes) prop.meshes.push_back(mm.meshId);
        prop.parts = meshes;  // P94
        prop.piece = prop.moved = true;  // +0xef5 = 1; a piece is never a placement to go back to
        const uint32_t id = kAutoPropId | nextPropId_++;
        std::array<double, 16> md;
        for (size_t k = 0; k < 16; ++k) md[k] = pose[k];
        prop.body = physics.addProp(id, prop.objectId, md, prop.mass, prop.debris);
        if (prop.body) physics.wakeProp(id, {0, 0, 0});
        props_[id] = std::move(prop);
        ++piecesSpawned_;
    }
}

void WorldStreamer::prewarmDebris(const std::vector<const char*>& items, fuel::MeshRenderer& renderer) {
    if (lodByName_.empty())
        for (const fuel::DpcFile* f : lookup_)
            for (uint32_t id : f->ids(fuel::cls::Lod)) try {
                    if (const fuel::DpcFile::Entry* e = f->find(id)) lodByName_.emplace(fuel::parseObjectLink(f->link(*e)).name, id);
                } catch (const std::exception&) {
                }
    for (const char* item : items) {
        const auto lod = lodByName_.find(fuel::asoboCrc(item));
        if (lod == lodByName_.end()) continue;
        for (const fuel::PlacedMesh& pm : fuel::collectObjectMeshes(lookup_, lod->second)) {
            std::string err;
            renderer.preloadMesh(pm.meshId, &err);  // the GPU mesh and its textures, cached for the first throw
        }
    }
}

bool WorldStreamer::throwDebris(const char* item, const std::array<double, 3>& pos, const std::array<float, 3>& vel, float yaw,
                                fuel::DriveWorld& physics, const std::array<float, 3>* angular, uint32_t* idOut, bool noContact) {
    if (lodByName_.empty())
        for (const fuel::DpcFile* f : lookup_)
            for (uint32_t id : f->ids(fuel::cls::Lod)) try {
                    if (const fuel::DpcFile::Entry* e = f->find(id)) lodByName_.emplace(fuel::parseObjectLink(f->link(*e)).name, id);
                } catch (const std::exception&) {
                }
    const uint32_t crc = fuel::asoboCrc(item);
    const auto lod = lodByName_.find(crc);
    if (lod == lodByName_.end()) return false;
    std::vector<fuel::PlacedMesh> meshes = fuel::collectObjectMeshes(lookup_, lod->second);
    if (meshes.empty()) return false;
    fuel::PlacedMesh pm = meshes.front();
    // 0x533d70: Node_Z::SetWorldPosRot(pos, a rotation about y), scale 1.
    const float c = std::cos(yaw), s = std::sin(yaw);
    pm.world = {c, 0, -s, 0, 0, 1, 0, 0, s, 0, c, 0, float(pos[0]), float(pos[1]), float(pos[2]), 1};
    pm.nodeName = crc;
    Prop prop;
    if (!makeProp(prop, pm, physics)) return false;
    prop.meshes.clear();
    for (const auto& mm : meshes) prop.meshes.push_back(mm.meshId);
    prop.parts = meshes;  // P94
    prop.piece = prop.moved = true;  // never a placement to go back to
    const uint32_t id = kAutoPropId | nextPropId_++;
    std::array<double, 16> md;
    for (size_t k = 0; k < 16; ++k) md[k] = pm.world[k];
    prop.body = physics.addProp(id, prop.objectId, md, prop.mass, prop.debris);
    if (!prop.body) return false;
    if (noContact) physics.setPropNoContact(id);  // P87
    if (angular != nullptr) physics.wakeProp(id, vel, *angular);  // P85: 0x533d70's w
    else physics.wakeProp(id, vel);  // the agent's velocity at the launch (+0x20c.. -> +0x820)
    props_[id] = std::move(prop);
    if (idOut != nullptr) *idOut = id;
    thrown_.push_back(id);
    ++debrisThrown_;
    while (thrown_.size() > kMaxThrown) {
        const uint32_t old = thrown_.front();
        thrown_.pop_front();
        if (auto it = props_.find(old); it != props_.end()) {
            if (it->second.body) physics.removeProp(old);
            props_.erase(it);
        }
    }
    return true;
}

// ---- P85: thrown debris for the crash debris' lifetime (damage_view.mm, IT_DebrisMove 0x533ec0).
bool WorldStreamer::thrownPosition(uint32_t id, std::array<double, 3>& out) const {
    auto it = props_.find(id);
    if (it == props_.end()) return false;
    out = {it->second.world[12], it->second.world[13], it->second.world[14]};
    return true;
}

void WorldStreamer::dropThrown(uint32_t id, fuel::DriveWorld& physics) {
    auto it = props_.find(id);
    if (it == props_.end()) return;
    if (it->second.body) physics.removeProp(id);
    props_.erase(it);
    thrown_.erase(std::remove(thrown_.begin(), thrown_.end(), id), thrown_.end());
}
// ---- P85 end

// RESET_ rules (0x513630 action 4 -> 0x513ca0 -> the agent's reset; IT_DYNBASE_G 0x51b930 puts the body back on its
// placement and the decor it replaced comes back with the world reset): knocked props go back, broken decor is whole.
void WorldStreamer::applyResets(fuel::MeshRenderer& renderer, fuel::DriveWorld& physics) {
    if (resetPending_.empty()) return;
    bool hiddenChanged = false;
    for (uint32_t i : resetPending_) {
        if (i >= placed_.size()) continue;
        if (auto it = props_.find(i); it != props_.end()) {
            if (it->second.body) physics.removeProp(i);
            props_.erase(it);
        }
        if (placed_[i].broken) placed_[i].broken = false, hiddenChanged = true, lodDirty_ = true;
    }
    if (std::getenv("FUEL_STREAM_TRACE")) std::printf("object resets: %zu placements\n", resetPending_.size());
    resetPending_.clear();
    if (hiddenChanged) refreshHidden(renderer);
    last_ = {1e30f, 0, 0};  // rebuild props and solids on the next update
}

// ---- PK14: KillMission 0x62baa0 on the world objects (drive.h).

void WorldStreamer::noteAnimStarts(const std::vector<uint32_t>& objects) {
    for (uint32_t o : objects)
        if (std::find(startedAnims_.begin(), startedAnims_.end(), o) == startedAnims_.end()) startedAnims_.push_back(o);
}

size_t WorldStreamer::freshSub(size_t f) {
    // [I] Attach 0x740690 binds the WorldRef's nodes again (0x72fde0: new agents; vf34 0x55ed20: ObjCond_RegisterTree adds
    // the CONDITION_OBJECT records anew, then the streaming re-evaluation 0x512920): broken decor (the old nodes' 0x40000)
    // is whole, every condition starts from its default (DEFAULTHIDE, else shown), the STARTANIMATED children re-armed.
    size_t n = 0;
    for (Placed& p : placed_) {
        if (p.sub != f) continue;
        if (p.broken) p.broken = false, ++n, lodDirty_ = true;
        if (p.condition) {
            const bool was = p.hidden;
            p.hidden = p.condition->defaultHidden;
            p.hidden = fuel::evaluateObjectCondition(*p.condition, objects_).value_or(p.hidden);
            if (was != p.hidden) lodDirty_ = true;
        }
    }
    for (AnimVolume& v : animVolumes_)
        if (v.sub == f && v.fired) {
            v.fired = false;
            animResets_.insert(animResets_.end(), v.objects.begin(), v.objects.end());
        }
    return n;
}

size_t WorldStreamer::fireResets(int kind, const std::string& mission) {
    if (mission.empty()) return 0;
    fuel::ObjectEvent ev;
    ev.kind = kind, ev.mission = mission;
    size_t n = 0;
    for (size_t i = 0; i < placed_.size(); ++i) {
        if (!placed_[i].condition) continue;
        std::vector<const fuel::ObjectCondition::Rule*> fired;
        (void)fuel::evaluateObjectCondition(*placed_[i].condition, objects_, ev, &fired);
        for (const auto* r : fired)
            if (r->action == fuel::ObjectCondition::Reset) {
                resetPending_.push_back(uint32_t(i)), ++n;
                break;
            }
    }
    return n;
}

WorldStreamer::KillReport WorldStreamer::killMission(fuel::MeshRenderer& renderer, fuel::DriveWorld& physics) {
    KillReport rep;
    // 0x515be0: IT_DECORDYNMESH_G vf20 0x515c80 -> 0x5161d0: the pieces (IT_DYNOBJ_G agents of the factory) go back to it
    // (0x4f1b60: hidden, removed). The decor node keeps its 0x40000: placed_[].broken and brokenAuto_ stay.
    for (auto it = props_.begin(); it != props_.end();) {
        if (!it->second.piece) {
            ++it;
            continue;
        }
        if (it->second.body) physics.removeProp(it->first);
        it = props_.erase(it), ++rep.pieces;
    }
    // 0x51df10: 0x51dea0 on every IT_DYNOBJ_G (registry 0xa66728): the agent's reset 0x51b930 and the mover's (vt+0x38
    // defaults, 0x70e160 body re-synced from the node = the placement, +0x5aa / +0x5ab = 0, +0xedc = 1). The port's prop
    // state is dropped: the next update makes the props within kPropRadius again at their placement / spawn. IT_Debris
    // (the tornado's and the crashes' thrown items, IT_DebrisMove) are not IT_DYNOBJ_G agents [I]: they stay.
    bool autoForgot = false;
    for (auto it = props_.begin(); it != props_.end();) {
        Prop& p = it->second;
        if (p.debris || std::find(thrown_.begin(), thrown_.end(), it->first) != thrown_.end()) {
            ++it;
            continue;
        }
        if (p.body) physics.removeProp(it->first);
        if (p.autoKey) autoProps_.erase(p.autoKey), autoForgot = true, ++rep.autoProps;
        else ++rep.props;
        it = props_.erase(it);
    }
    // Mission::SetState(0) (0x632c10 case 0): the M_END event of the mission (0x514300 / 0x512980; its RESET_ rules), the
    // course sub-world's UNSTREAM_M_END: Detach 0x740420 (0x55ed70 drops its condition records, 0x72ffb0 unbinds; [O]
    // only the root node is walked there, the props go with their agents) - the retry attaches it again (freshSub).
    rep.endResets = fireResets(fuel::ObjectCondition::End, mission_);
    if (!mission_.empty())
        for (size_t f = 0; f < subFiles_.size(); ++f) {
            const int r = f < subRef_.size() ? subRef_[f] : -1;
            if (r >= 0 && refs_[size_t(r)].kind == fuel::SubWorldRef::Course && strcasecmp(refs_[size_t(r)].mission.c_str(), mission_.c_str()) == 0)
                rep.courseFresh += freshSub(f);
        }
    // 0x62c850: the started IT_ANIMATED (node +0xd8 bit 0) reset (0x517f80: re-armed); their volumes fire again.
    rep.anims = startedAnims_.size();
    animResets_.insert(animResets_.end(), startedAnims_.begin(), startedAnims_.end());
    startedAnims_.clear();
    for (AnimVolume& v : animVolumes_) v.fired = false;
    // M_RUNNING off and 0x512920(0) (setMission re-evaluates every condition and sets the rebuild mark).
    setMission("");
    pushStartFx(renderer, true);  // PK12: the STARTFX nodes are hidden now (and the course detached): their records go
    last_ = {1e30f, 0, 0};  // the rebuild mark (P107: the next update launches the job at the player) even without a mission
    refreshHidden(renderer);
    if (autoForgot) pushAutoDraw(renderer);
    updateProps(renderer, physics, true);
    if (std::getenv("FUEL_STREAM_TRACE"))
        std::printf("KillMission: %zu pieces released, %zu props and %zu AutoMesh props reset, %zu course decor whole, %zu animations reset, "
                    "%zu M_END resets\n", rep.pieces, rep.props, rep.autoProps, rep.courseFresh, rep.anims, rep.endResets);
    return rep;
}

std::vector<WorldStreamer::PropPlace> WorldStreamer::propsNear(double x, double z, float r) const {
    std::vector<PropPlace> out;
    for (const auto& [id, p] : props_) {
        if (p.piece || p.debris) continue;
        if (std::hypot(p.world[12] - x, p.world[14] - z) > r && std::hypot(p.origin[0] - x, p.origin[2] - z) > r) continue;
        PropPlace pp;
        pp.pos = simd_make_float3(p.world[12], p.world[13], p.world[14]);
        pp.origin = simd_make_float3(p.origin[0], p.origin[1], p.origin[2]);
        pp.displacement = simd_distance(pp.pos, pp.origin);
        out.push_back(pp);
    }
    return out;
}

size_t WorldStreamer::piecesNow() const {
    return size_t(std::count_if(props_.begin(), props_.end(), [](const auto& kv) { return kv.second.piece; }));
}
// ---- end PK14

// The AutoMesh draw list (renderer set 1) without the spawns that are props now.
void WorldStreamer::pushAutoDraw(fuel::MeshRenderer& renderer) {
    if (autoProps_.empty()) {
        renderer.setLodObjects(autoObjects_, 1);
        return;
    }
    std::vector<fuel::PlacedMesh> draw;
    draw.reserve(autoObjects_.size());
    for (const fuel::PlacedMesh& p : autoObjects_)
        if (!autoProps_.count(autoPropKey(p))) draw.push_back(p);
    renderer.setLodObjects(draw, 1);
}

WorldStreamer::PropStats WorldStreamer::propStats() const {
    PropStats s;
    s.tracked = props_.size();
    for (const auto& [i, p] : props_) {
        s.bodies += p.body, s.moved += p.moved;
        if (!p.moved) continue;
        const float d = std::hypot(p.world[12] - p.origin[0], p.world[13] - p.origin[1], p.world[14] - p.origin[2]);
        if (d > s.maxDisplacement) s.maxDisplacement = d, s.where = simd_make_float3(p.world[12], p.world[13], p.world[14]);
    }
    return s;
}

bool WorldStreamer::nearestProp(simd_float3 from, const std::vector<simd_float3>& skip, simd_float3& out) const {
    float best = 1e30f;
    for (const auto& [i, p] : props_) {
        if (p.moved || p.piece) continue;
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
        static const bool trace = std::getenv("FUEL_P86_PROPTRACE") != nullptr;  // P86: props flying (> 3 m up or > 5 m/s)
        if (trace) {
            std::array<float, 3> v{};
            physics.propVelocity(pp.id, v);
            const float speed = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
            const double rise = pp.pose.position[1] - p.origin[1];
            if (speed > 5.0f || rise > 3.0)
                std::printf("p86 prop %08x%s at (%.1f, %.1f, %.1f), %.1f m above its origin, %.1f m/s\n", p.objectId, p.piece ? " (piece)" : "",
                            pp.pose.position[0], pp.pose.position[1], pp.pose.position[2], rise, speed);
        }
    }
    if (!dirty) return;
    // P94: props are drawn through their Lod (renderer LOD set 3) at their pose, like FUEL's dynamic objects; the plain
    // prop list keeps only entries without Lod data.
    std::vector<fuel::MeshInstance> list;
    std::vector<fuel::PlacedMesh> lod;
    list.reserve(props_.size());
    static const bool legacy = std::getenv("FUEL_POP_LEGACY") != nullptr;  // pre-P94: level 0 at full alpha (A/B)
    for (const auto& [i, p] : props_) {
        if (p.parts.empty() || legacy) {
            for (uint32_t mesh : p.meshes) list.push_back({mesh, rowsToMatrix(p.world)});
            continue;
        }
        for (const fuel::PlacedMesh& part : p.parts) {
            lod.push_back(part);
            lod.back().world = p.world;
            lod.back().autoCell.on = false;  // a prop stays until the streamer forgets it
        }
    }
    renderer.setPropInstances(list);
    renderer.setLodObjects(lod, 3);
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
        // Trees and bushes are solid near the player (P50: 0x74ee80 mode 2 -> 0x785200 trees W+0x48418,
        // 0x788ab0 bushes W+0x48578): the nearest trunk of a species without flag 2 (and not marked broken, record
        // +0x24, set by 0x77d1c0 when a Lod object fails to load), a vertical segment with no lower end up to the
        // plant point + 5 m (trees) / + 10 m (bushes). Radius (P52, fuel::trunkRadius): min(trunk x drawn scale, 2)
        // with the draw's own noise 0x784370 / 0x787c70. Port: a cylinder from 2 m under the ground point.
        // FUEL tests the chassis spheres and boxes, not the wheels. Bushes collide where FUEL has them: chunks
        // below kBushChunkMax (as drawn).
        for (const fuel::Plant& p : scattered) {
            if (std::hypot(p.position[0] - player.x, p.position[2] - player.z) > kCollideRadius) continue;
            // The plant's own species record (P52: by mesh, a bush whose mesh is also a tree took the tree's settings).
            const std::vector<fuel::Species>& list = p.tree ? vegetation_->trees : vegetation_->bushes;
            if (p.species == 0 || p.species > list.size()) continue;
            const fuel::Species& sp = list[p.species - 1];
            if (sp.flags & 2) continue;  // trunk 0 still collides (a line)
            const bool tree = p.tree;
            if (!tree && decorChunkSize(p.position[0], p.position[2], player.x, player.z) >= kBushChunkMax) continue;
            const double r = std::max(fuel::trunkRadius(*vegetation_, sp, p.position[0], p.position[2]), 0.01f);  // > 0 for Jolt
            const double ground = double(p.position[1]) + fuel::kPlantSink;  // FUEL's plant point (unsunk)
            const double h = (tree ? fuel::kTreeTrunkTop : fuel::kBushTrunkTop) + 2.0;
            bg.trunks.push_back({r, 0, 0, 0, 0, h, 0, 0, 0, 0, r, 0, p.position[0], ground - 2.0, p.position[2], 1});
            (tree ? bg.treeTrunks : bg.bushTrunks)++;
        }
        // ---- P94: FUEL decides mesh or card per frame from the camera (core/vegetation.hpp decorAtt: the decor leaf
        // under the plant picks the ground element kinds, each fading with the horizontal eye distance in
        // ground_rotshape.vhl). The worker hands the renderer every plant that may show before the next rebuild: the
        // mesh within kMeshRadius, the card (both trees and bushes) within kCardRadius / kBushCardRadius, each tagged
        // with its kind (instance column 1 w); vsMain applies the chunk rule and the fade for the frame's eye.
        // FUEL_POP_LEGACY=1: the pre-P94 choice, made once at the build position (A/B comparisons).
        static const bool legacy = std::getenv("FUEL_POP_LEGACY") != nullptr;
        fuel::scatterPlants(*vegetation_, *terrain_, roads_, blocked, player.x, player.z, legacy ? kLegacyCardRadius : kCardRadius, scattered, true,
                            kPlantRadius, ground_);
        std::vector<fuel::DecorFade> kinds;
        for (fuel::Plant& p : scattered) {
            const std::vector<fuel::Species>& list = p.tree ? vegetation_->trees : vegetation_->bushes;
            if (p.species == 0 || p.species > list.size()) continue;
            const fuel::Species& sp = list[p.species - 1];
            const bool tree = p.tree, isCard = p.meshId != sp.full;
            if (legacy) {  // full meshes in chunks < 60 m, cards beyond; trees in chunks < 600, bushes < 100, at the build position
                const double chunk = decorChunkSize(p.position[0], p.position[2], player.x, player.z);
                if (chunk >= (tree || isCard ? kTreeChunkMax : kBushChunkMax)) continue;
                if (!isCard && chunk >= kFullMeshChunk) {
                    if (sp.impostor == 0) continue;
                    p.meshId = sp.impostor;
                }
                plants.push_back(p), kinds.push_back(fuel::DecorFade::None);
                continue;
            }
            if (isCard) {  // beyond kPlantRadius: trees only, as cards
                plants.push_back(p), kinds.push_back(fuel::DecorFade::TreeCard);
                continue;
            }
            const float d = std::hypot(p.position[0] - player.x, p.position[2] - player.z);
            if (d <= kMeshRadius) plants.push_back(p), kinds.push_back(fuel::DecorFade::Mesh);
            if (sp.impostor != 0 && d <= (tree ? kCardRadius : kBushCardRadius)) {
                fuel::Plant c = p;
                c.meshId = sp.impostor;
                plants.push_back(c), kinds.push_back(tree ? fuel::DecorFade::TreeCard : fuel::DecorFade::BushCard);
            }
        }
        if (std::getenv("FUEL_STREAM_TRACE"))
            std::printf("plants: %zu scattered, %zu drawn (%zu trunks)\n", scattered.size(), plants.size(), bg.trunks.size());
        bg.plants.reserve(plants.size());
        for (size_t k = 0; k < plants.size(); ++k) {
            const fuel::Plant& p = plants[k];
            auto col = [&](size_t i) { return simd_make_float4(p.axes[i][0], p.axes[i][1], p.axes[i][2], 0); };
            simd_float4 c1 = col(1);
            c1.w = float(kinds[k]);  // the decor kind (vsMain decorAtt; 0 = always drawn)
            bg.plants.push_back({p.meshId, simd_matrix(col(0), c1, col(2), simd_make_float4(p.position[0], p.position[1], p.position[2], 1))});
        }
        // ---- P94 end
    }
    if (lakes_ && ground_ && terrain_) buildWater(player, bg);
    static const bool noAuto = std::getenv("FUEL_NO_AUTOMESH") != nullptr;
    if (autoMesh_ && !noAuto && ground_ && terrain_ && terrain_->detail) {
        fuel::AutoMesh::Context ctx;
        ctx.terrain = terrain_;
        ctx.ground = ground_;
        ctx.roads = terrain_->roads.get();
        ctx.lines = roads_;
        ctx.types = ctx.roads ? &ctx.roads->types() : nullptr;
        ctx.free = [this](double x, double z, float) { return !nearObstacle(x, z, 0.0f); };  // W[0] (port: placed objects' circles)
        const auto t0 = std::chrono::steady_clock::now();
        std::vector<fuel::AutoMesh::Collectible> fuelSpawns;
        // P94: a superset for any camera within kAutoSlack of the player (the list is rebuilt every 40 m, a worker job
        // late); the renderer keeps FUEL's set for the frame's camera (PlacedMesh::autoCell), so nothing arrives late.
        static const bool legacy = std::getenv("FUEL_POP_LEGACY") != nullptr;  // pre-P94: FUEL's set at the build position
        autoMesh_->generate(ctx, player.x, player.z, bg.autoObjects, &bg.autoRoadFrom, &fuelSpawns, &bg.decor, legacy ? 0.0 : kAutoSlack);
        bg.haveAuto = true;
        // P44: barrels taken by the player lose their meshes (FUEL hides the collected agent); none is solid.
        std::vector<std::array<float, 2>> taken;
        {
            std::lock_guard<std::mutex> l(takenMutex_);
            taken = taken_;
        }
        bg.autoCollect.assign(bg.autoObjects.size(), 0);
        std::vector<uint8_t> drop(bg.autoObjects.size(), 0);
        {  // P50: breakable decor the player broke stays broken while its cell is cached
            std::lock_guard<std::mutex> l(takenMutex_);
            for (size_t k = 0; k < bg.autoObjects.size(); ++k)
                for (const auto& b : brokenAuto_)
                    if (std::fabs(b[0] - bg.autoObjects[k].world[12]) < 0.05f && std::fabs(b[1] - bg.autoObjects[k].world[14]) < 0.05f) drop[k] = 1;
        }
        for (const auto& c : fuelSpawns) {
            bool gone = false;
            for (const auto& t : taken) gone |= std::fabs(t[0] - float(c.x)) < 0.05f && std::fabs(t[1] - float(c.z)) < 0.05f;
            if (!gone) bg.barrels.push_back({float(c.x), float(c.y), float(c.z), c.amount, c.radius});
            for (size_t k = c.first; k < c.first + c.count && k < drop.size(); ++k) (gone ? drop[k] : bg.autoCollect[k]) = 1;
        }
        if (std::find(drop.begin(), drop.end(), 1) != drop.end()) {
            size_t w = 0, road = bg.autoRoadFrom;
            for (size_t k = 0; k < bg.autoObjects.size(); ++k) {
                if (drop[k]) {
                    if (k < bg.autoRoadFrom) --road;
                    continue;
                }
                // P107: never `x = std::move(x)`: libc++'s vector self-move-assignment leaves it empty, so every spawn before
                // the first dropped one lost its levels (drawn by nothing: CR21-Val1-SR ab46f2c7 vanished at 286 m once the
                // tornado had broken AutoMesh decor; a barrel taken by the player did the same).
                if (w != k) bg.autoCollect[w] = bg.autoCollect[k], bg.autoObjects[w] = std::move(bg.autoObjects[k]);
                ++w;
            }
            bg.autoObjects.resize(w), bg.autoCollect.resize(w), bg.autoRoadFrom = road;
        }
        if (std::getenv("FUEL_STREAM_TRACE")) {
            const fuel::AutoMesh::Stats st = autoMesh_->lastStats();
            std::printf("automesh: %zu new cells, %zu cell objects, %zu road objects, %zu parts in %.1f ms\n", st.cells, st.cellObjects,
                        st.roadObjects, st.parts, std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());
        }
    }
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

// A unit cylinder (radius 1, y from 0 to 1) for tree and bush trunks.
constexpr uint32_t kTrunkShape = 0x7E0E0001u;

// FUEL's static collision of a node object (Lod_Z / Mesh_Z), loaded once and kept here (P46/P50,
// core/static_collision.hpp).
const fuel::StaticCollision& WorldStreamer::collision(uint32_t objectId) const {
    std::lock_guard<std::mutex> l(collisionMutex_);
    auto it = collision_.find(objectId);
    if (it == collision_.end()) {
        auto c = std::make_shared<fuel::StaticCollision>();
        try {
            *c = fuel::loadStaticCollision(lookup_, objectId);
        } catch (const std::exception&) {
        }
        it = collision_.emplace(objectId, std::move(c)).first;
    }
    return *it->second;
}

// Registered once with the physics (P46).
void WorldStreamer::ensureCollision(fuel::DriveWorld& physics, uint32_t key) {
    if (physics.hasStaticCollision(key)) return;
    (void)collision(key);
    std::shared_ptr<const fuel::StaticCollision> c;
    {
        std::lock_guard<std::mutex> l(collisionMutex_);
        c = collision_.at(key);
    }
    physics.addStaticCollision(key, std::move(c));
}

// What FUEL makes of a placement in its collision world (P50, notes p50-collision-world.md):
//  - only the node's first entry carries its collision (a Skin level gives one entry per mesh);
//  - IT_DYNCOLLECT barrels: none (picked up on contact, p44);
//  - IT_DYNOBJ_G, IT_Debris (clone.tsc class of the node's name): movable. IT_DYNOBJ_G puts its node in the physics
//    scene itself (0x51b930 sets 0x8000000), so only the hash test applies;
//  - IT_DECORDYNMESH_G (breakable decor): static, broken through at >= P575 (breakThrough);
//  - everything else: static when the node is in the physics scene (0x8000000, not 0x40000) and in the collision
//    hash (not node 0x10000, not object flag 8; 0x65ef40).
// Hidden placements (conditions) and detached sub-worlds are filtered by active() before this.
WorldStreamer::Solid WorldStreamer::solidKind(const fuel::PlacedMesh& p) const {
    if (!p.collisionFirst) return Solid::None;
    const fuel::GameClass c = gameClass(p.nodeName);
    if (c == fuel::GameClass::Collect) return Solid::None;
    if (c == fuel::GameClass::DynObj) return fuel::inCollisionHash(p.nodeFlags, p.objectFlags) && !(p.nodeFlags & 0x40000u) ? Solid::Movable : Solid::None;
    if (!fuel::staticSolid(p.nodeFlags, p.objectFlags)) return Solid::None;
    return c == fuel::GameClass::Debris ? Solid::Movable : Solid::Static;
}

fuel::GameClass WorldStreamer::gameClass(uint32_t nodeName) const {
    const auto it = gameClasses_.find(nodeName);
    return it == gameClasses_.end() ? fuel::GameClass::None : it->second;
}

void WorldStreamer::applySolids(fuel::DriveWorld& physics) {
    if (!physics.hasStaticShape(kTrunkShape)) physics.addStaticCylinder(kTrunkShape);  // P52: round (was an 8-sided hull)
    std::vector<std::pair<uint32_t, std::array<double, 16>>> solid = cellSolid_;
    for (const auto& m : trunks_) solid.push_back({kTrunkShape, m});
    for (const auto& [key, m] : autoSolid_) {  // AutoMesh buildings and props (their template nodes' collision)
        ensureCollision(physics, key);
        solid.push_back({key, m});
    }
    if (std::getenv("FUEL_STREAM_TRACE")) std::printf("solids: %zu placed, %zu trunks\n", cellSolid_.size(), trunks_.size());
    physics.setStaticObjects(solid);
}

void WorldStreamer::applyBackground(Background&& bg, fuel::MeshRenderer& renderer, fuel::DriveWorld& physics) {
    trunks_ = std::move(bg.trunks);
    treeTrunks_ = bg.treeTrunks, bushTrunks_ = bg.bushTrunks;
    if (std::getenv("FUEL_P52_TRUNKS")) {  // draw the trunk cylinders (radius, from the ground point up 6 / 11 m)
        std::vector<simd_float4> dbg;
        for (const auto& m : trunks_)
            dbg.push_back(simd_make_float4(float(m[12]), float(m[13] + 2.0), float(m[14]), float((m[5] > 9.0 ? 20.0 : 10.0) + m[0])));
        renderer.setDebugTrunks(dbg);
    }
    if (bg.haveAuto) {
        autoSolid_.clear();
        autoBreakables_.clear();
        // AutoMesh spawns go through 0x778650, which inserts every spawned node like any scene node (0x65ef40):
        // cell objects and road furniture follow the placement rule (solidKind). Port: AutoMesh objects have no
        // movable bodies yet, so movable classes (guard rails Glissiere*, *_DYN vehicles) stay passable.
        for (size_t i = 0; i < bg.autoObjects.size(); ++i)
            if (const fuel::PlacedMesh& p = bg.autoObjects[i];
                !(i < bg.autoCollect.size() && bg.autoCollect[i]) && solidKind(p) == Solid::Static &&
                std::hypot(p.world[12] - last_.x, p.world[14] - last_.z) <= kCollideRadius) {
                std::array<double, 16> m;
                for (size_t k = 0; k < 16; ++k) m[k] = p.world[k];
                autoSolid_.push_back({p.objectId, m});
                if (gameClass(p.nodeName) == fuel::GameClass::DecorDynMesh) autoBreakables_.push_back({-1, p.objectId, m});
            }
        autoObjects_ = bg.autoObjects;
        autoRoadFrom_ = bg.autoRoadFrom;
        // P55: AutoMesh IT_DYNOBJ_G spawns (road signs, barriers, wrecks; 0x778650 inserts them like any node) are
        // movable props near the player, as placed ones are; those already props keep their state.
        bool newProps = false;  // P107
        for (size_t i = 0; i < autoObjects_.size(); ++i) {
            const fuel::PlacedMesh& p = autoObjects_[i];
            if ((i < bg.autoCollect.size() && bg.autoCollect[i]) || gameClass(p.nodeName) != fuel::GameClass::DynObj) continue;
            if (std::hypot(p.world[12] - last_.x, p.world[14] - last_.z) > kPropRadius) continue;
            const uint64_t key = autoPropKey(p);
            if (autoProps_.count(key)) continue;
            Prop prop;
            if (!p.collisionFirst || !makeProp(prop, p, physics)) continue;
            prop.autoKey = key;
            prop.meshes.clear();  // every mesh entry of the node (a Skin level gives several)
            prop.parts.clear();
            for (const fuel::PlacedMesh& q : autoObjects_)
                if (autoPropKey(q) == key) prop.meshes.push_back(q.meshId), prop.parts.push_back(q);
            const uint32_t id = kAutoPropId | nextPropId_++;
            autoProps_[key] = id;
            props_[id] = std::move(prop);
            newProps = true;
        }
        pushAutoDraw(renderer);
        // P107: pushAutoDraw drops the spawns that just became props from set 1, so their prop draw (set 3) must go
        // to the renderer in the same frame. FUEL draws the IT_DYNOBJ_G agent through its one Lod_Z all along (no
        // hand-over); without this the spawn vanished until the next updateProps(dirty) (0.4 s on CR2-Yell-CP).
        if (newProps) updateProps(renderer, physics, true);
        barrels_ = std::move(bg.barrels);
        decor_ = std::move(bg.decor);
        ++barrelGen_;
    }
    applySolids(physics);
    plantDraw_ = std::move(bg.plants);
    std::vector<fuel::MeshInstance> draw = cellDraw_;
    draw.insert(draw.end(), plantDraw_.begin(), plantDraw_.end());
    renderer.setWorldInstances(draw);
    if (bg.haveWater) renderer.setWater(bg.water, bg.waterIndices);
}

void WorldStreamer::pollBackground(fuel::MeshRenderer& renderer, fuel::DriveWorld& physics) {
    if (!job_.valid()) return;
    // P107: a headless check lands every job exactly maxJobFrames_ frames after its launch (setMaxJobFrames), waiting
    // for it then if need be: the same streaming latency on every run, whatever the machine's load.
    if (maxJobFrames_ > 0) {
        if (++jobFrames_ < maxJobFrames_) return;
    } else if (job_.wait_for(std::chrono::seconds(0)) != std::future_status::ready) {
        return;
    }
    jobFrames_ = 0;
    applyBackground(job_.get(), renderer, physics);
    // P107: not while last_ is the "rebuild on the next update" mark (1e30, set by setMission / setRtcRunning /
    // applyResets): that job would be built 1e30 m away (no plants, no AutoMesh). update() then launches it at the player.
    if (std::exchange(rerun_, false) && last_.x < 1e29f) {  // catch up with where the player is now
        jobFrames_ = 0;
        job_ = std::async(std::launch::async, [this, at = last_] { return buildBackground(at); });
    }
}

void WorldStreamer::update(simd_float3 player, fuel::MeshRenderer& renderer, fuel::DriveWorld& physics, bool force) {
    updateGrass(player, renderer, force);
    updateGroundTiles(player, renderer, force);
    applyResets(renderer, physics);
    checkAnimVolumes(player);
    pushStartFx(renderer, false);  // PK12
    pollBackground(renderer, physics);
    if (refreshActive(player)) {
        force = true;
        lodDirty_ = true;
        for (auto it = props_.begin(); it != props_.end();) {  // detached sub-worlds take their props along
            if (it->first >= kAutoPropId || active(placed_[it->first])) {
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
    breakables_.clear();
    // Drawing: every active placement goes to the renderer, which picks its level and fade each
    // frame like Lod_Z::Draw (p36); this loop only makes props and collision near the player.
    const int64_t r = int64_t(std::ceil(kCollideRadius / kCell));
    const int64_t cx = int64_t(std::floor(player.x / kCell)), cz = int64_t(std::floor(player.z / kCell));
    for (int64_t dz = -r; dz <= r; ++dz)
        for (int64_t dx = -r; dx <= r; ++dx) {
            auto it = cells_.find(cellKey(cx + dx, cz + dz));
            if (it == cells_.end()) continue;
            for (uint32_t i : it->second) {
                const Placed& p = placed_[i];
                const float d = std::hypot(p.world[12] - player.x, p.world[14] - player.z);
                if (d > kCollideRadius || !active(p)) continue;
                if (props_.count(i)) continue;  // drawn from the prop list
                // P50: which placements FUEL collides with (solidKind): static FUEL collision, a movable body
                // (IT_DYNOBJ_G / IT_DECORDYNMESH_G / IT_Debris near the player), or nothing.
                const Solid kind = solidKind(p);
                if (kind == Solid::None) continue;
                const fuel::StaticCollision& col = collision(p.objectId);
                if (col.empty()) continue;  // FUEL has no shape for it (CollisionVol-only, Skin levels, decals)
                if (kind == Solid::Movable && d <= kPropRadius) {
                    // P55: FUEL's body (spheres and boxes) and mass (dyn_object.hpp).
                    Prop prop;
                    if (makeProp(prop, p, physics)) {
                        props_[i] = std::move(prop);
                        continue;
                    }
                }
                // Static: FUEL's static scene (and a movable object beyond kPropRadius: FUEL keeps the
                // IT_DYNOBJ_G / breakable decor node in the static scene until a hit activates it, 0x51b930 [I]).
                ensureCollision(physics, p.objectId);
                std::array<double, 16> m;
                for (size_t k = 0; k < 16; ++k) m[k] = p.world[k];
                solid.push_back({p.objectId, m});
                if (gameClass(p.nodeName) == fuel::GameClass::DecorDynMesh) breakables_.push_back({int64_t(i), p.objectId, m});
            }
        }
    cellDraw_ = std::move(draw);
    if (lodDirty_) {
        lodDirty_ = false;
        std::vector<fuel::PlacedMesh> list;
        lodIndex_.assign(placed_.size(), -1);
        for (size_t i = 0; i < placed_.size(); ++i)
            if (active(placed_[i])) lodIndex_[i] = int32_t(list.size()), list.push_back(placed_[i]);
        const size_t n = renderer.setLodObjects(list);
        if (std::getenv("FUEL_STREAM_TRACE")) std::printf("lod objects: %zu active placements, %zu drawable\n", list.size(), n);
    }
    refreshHidden(renderer);
    if (force) {  // teleports and the first frame: build everything now
        if (job_.valid()) (void)job_.get();
        rerun_ = false;
        applyBackground(buildBackground(player), renderer, physics);
    } else if (job_.valid()) {
        rerun_ = true;
    } else {
        jobFrames_ = 0;
        job_ = std::async(std::launch::async, [this, at = player] { return buildBackground(at); });
    }
    // Props: simulate those near the player (and not knocked far off), forget distant ones.
    bool autoForgot = false;  // an AutoMesh spawn is drawn by the AutoMesh list again
    for (auto it = props_.begin(); it != props_.end();) {
        Prop& p = it->second;
        const float d = std::hypot(p.world[12] - player.x, p.world[14] - player.z);
        // Pieces leave the physics world 500 m away (IT_DYNOBJMove_G vt[140] 0x531660 -> +0xeb3); props reset at 300 m.
        const bool keep = d <= (p.piece ? fuel::kDynFarDistance : kPropForget) && (p.moved || d <= kPropRadius * 1.25f);
        const bool want = keep && d <= kPropRadius * 1.25f;
        if (!keep) {
            if (p.body) physics.removeProp(it->first);
            if (p.autoKey) autoProps_.erase(p.autoKey), autoForgot = true;
            it = props_.erase(it);
            continue;
        }
        if (want && !p.body) {
            std::array<double, 16> m;
            for (size_t k = 0; k < 16; ++k) m[k] = p.world[k];
            p.body = physics.addProp(it->first, p.objectId, m, p.mass, p.debris);
        } else if (!want && p.body) {
            physics.removeProp(it->first);
            p.body = false;
        }
        ++it;
    }
    if (autoForgot) pushAutoDraw(renderer);
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
    // PK32: the curvature term 0x7f90d0 on the occlusion byte (Paysage_FillPoints, chunks under 120 m: every tile level).
    const std::vector<float> curv = terrain_ && terrain_->detail && !std::getenv("FUEL_NO_GROUND_CURVATURE")
                                        ? fuel::groundCurvatureGrid(*terrain_->detail, t.originX + 0.5 * sp, t.originZ + 0.5 * sp, sp, int(n))
                                        : std::vector<float>();
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
                    const float occ = curv.empty() ? c.occlusion : fuel::curvedOcclusion(c.occlusion, curv[size_t(j) * n + i]);  // PK32
                    px[3] = uint8_t(std::clamp(0.8f * occ + 0.2f, 0.0f, 1.0f) * 255.0f + 0.5f);
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

void setCarOcclusion(const DriveSession& drive, fuel::MeshRenderer& renderer) {
    // P74: Vehicle_CarOcclusion 0x5cc410 runs for the vehicle whose +0x1a48 (is_player) is set: 0x42b600 sets it
    // on the player's vehicle only (0x5c6ca0 clears it). The others take the blob shadows instead (0x5d8420 /
    // 0x5d86b0 run only while +0x1a48 is 0). 0x5cbf10 hides the omni with the vehicle.
    std::vector<fuel::MeshRenderer::CarOcclusion> boxes;
    if (drive.vehicleCount() > 0) {
        const auto [world, extents] = drive.carOcclusion(0);
        boxes.push_back({world, extents});
    }
    renderer.setCarOcclusion(std::move(boxes));
    // P77: every vehicle's headlights (0x5cbde0 runs for the player and the opponents alike), placed at their bone
    // (vehicle space) under the vehicle node. gDOmniDir = minus the bone's z axis: omni.phl's v points from the lit
    // point to the light, and the 15 / 45 degree cone (+0x11c / +0x120) must open along the bone's z, which points
    // forward and down on every variant (I: the record builder that hands vf43 its direction is not decoded).
    std::vector<fuel::MeshRenderer::VehicleOmni> lights;
    for (int v = 0; v < drive.vehicleCount(); ++v) {
        const simd_float4x4 node = drive.rig(v).nodeWorld(drive.state(v));
        for (const fuel::VehicleLight& l : drive.rig(v).lights()) {
            const simd_float4x4 m = simd_mul(node, rowsToSimd(l.world));
            fuel::MeshRenderer::VehicleOmni o;
            o.pos = m.columns[3].xyz;
            o.dir = -simd_normalize(m.columns[2].xyz);
            const auto val = fuel::omni::value(l.params);
            o.value = simd_make_float4(val[0], val[1], val[2], val[3]);
            o.colour = simd_make_float3(l.params.colour[0], l.params.colour[1], l.params.colour[2]);
            o.radius = l.info.bound;  // PK28: bound sphere x node scale (1) for the gather's 0x821830 test
            o.flags = l.info.flags;   // PK28: 0x20000000 spot (0x7fdd80)
            lights.push_back(o);
        }
    }
    renderer.setVehicleLights(std::move(lights));
}
