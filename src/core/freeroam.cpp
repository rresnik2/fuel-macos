// FUEL's free-roam rules (P44); see freeroam.hpp and ~/re-work/fuel/notes/p44-freeroam.md (F1-F9).
#include "freeroam.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>

namespace fuel {

// ---------------------------------------------------------------- MessagesHUD

void MessageQueue::update(float dt) {
    // 0x4afb20: the shown message counts down and clears at <= 0; with none shown the next one starts
    // (0x4afa70: +0x4f0 = +0x4ec = duration, +0x4f4 kind, +0x4f8 param) [O].
    if (left_ > 0) {
        left_ -= dt;
        if (left_ <= 0) left_ = 0, cur_ = {};
        return;
    }
    if (queue_.empty()) return;
    cur_ = queue_.front();
    queue_.pop_front();
    left_ = cur_.duration;
}

bool MessageQueue::skip() {
    // 0x4af920: t = +0x4ec, T = +0x4f0; the x87 test is T 0.9 > t (the unordered pair of compares) and T 0.1 < t.
    if (left_ <= 0) return false;
    if (!(left_ < cur_.duration * 0.9f && left_ > cur_.duration * 0.1f)) return false;
    left_ = cur_.duration * 0.1f;
    return true;
}

float MessageQueue::alpha() const {
    // 0x4afe00: f = left / duration; the first and last tenth fade (x 10) [O].
    if (left_ <= 0 || cur_.duration <= 0) return 0;
    const float f = left_ / cur_.duration;
    if (f > 0.9f) return (1.0f - f) * 10.0f;
    if (f > 0.1f) return 1.0f;
    return f * 10.0f;
}

// ---------------------------------------------------------------- FreeRoam

FreeRoam::FreeRoam(Career& career, StartOf startOf) : career_(career), startOf_(std::move(startOf)) {
    // The POI list as the scripts fill it (F1): hubinfos.tsc's liveries of kind POI (0x5756e0, type 2), then poi.tsc's
    // vista points (type 3) and teleports (type 4) in file order (0x54f760). PK09: then the grid rebuild 0x4ecf00
    // (DiscoveryPointVisibleObject reset 0x4ecec0): the unstable type sort 0x735ac0, the type-0 rule, the grid. The
    // resulting index order is the iteration order of the pickup / reveal queries (newest first per cell).
    const CareerRules& r = career_.rules();
    for (const CareerRules::PoiRef& e : r.poiScript) {
        if (e.hub < 0 || size_t(e.hub) >= r.hubs.size()) continue;
        const auto& hub = r.hubs[size_t(e.hub)];
        const size_t k = size_t(e.index);
        const int order = int(pois_.size());
        if (e.type == 2 && k < hub.liveries.size())
            pois_.push_back({2, e.hub, e.index, order, hub.liveries[k].x, hub.liveries[k].z, false, hub.liveries[k].mesh});
        else if (e.type == 3 && k < hub.vistaPoints.size())
            pois_.push_back({3, e.hub, e.index, order, hub.vistaPoints[k].x, hub.vistaPoints[k].z, false, hub.vistaPoints[k].name});
        else if (e.type == 4 && k < hub.teleports.size())
            pois_.push_back({4, e.hub, e.index, order, hub.teleports[k].x, hub.teleports[k].z, false, hub.teleports[k].name});
    }
    poiExchangeSort(pois_);                                   // 0x735ac0
    poiAssignUntyped(pois_, [] { return std::rand(); });      // never in retail (no type-0 entry)
    std::vector<std::array<float, 2>> xz;
    for (const Poi& p : pois_) xz.push_back({p.x, p.z});
    grid_.build(xz);                                          // 0x4edb00 / 0x4edc10
}

namespace {

// PK09: does a barrel (one sphere) touch a vehicle body (its spheres and boxes)? [port: FUEL's contact pass on the
// barrel's own shapes, 0x531310 being its per-contact callback]
bool touches(const FuelBarrel& b, const ContactBody& v) {
    const float r = b.radius;
    for (const auto& s : v.spheres) {
        const float dx = s[0] - b.x, dy = s[1] - b.y, dz = s[2] - b.z, rr = s[3] + r;
        if (dx * dx + dy * dy + dz * dz < rr * rr) return true;
    }
    for (const auto& o : v.boxes) {
        const float d[3] = {b.x - o.c[0], b.y - o.c[1], b.z - o.c[2]};
        float out2 = 0;
        for (int a = 0; a < 3; ++a) {
            const float t = d[0] * o.ax[size_t(a)][0] + d[1] * o.ax[size_t(a)][1] + d[2] * o.ax[size_t(a)][2];
            const float e = std::fabs(t) - o.h[size_t(a)];
            if (e > 0) out2 += e * e;
        }
        if (out2 < r * r) return true;
    }
    return false;
}

}  // namespace

void FreeRoam::setBarrels(std::vector<FuelBarrel> now) {
    // A barrel still spawned keeps its agent (and taken flag); a new spawn is a new agent (0x51d540) [O].
    for (FuelBarrel& b : now)
        for (const FuelBarrel& o : barrels_)
            if (std::fabs(o.x - b.x) < 0.05f && std::fabs(o.z - b.z) < 0.05f && o.amount == b.amount) {
                b.taken = o.taken;
                break;
            }
    barrels_ = std::move(now);
}

void FreeRoam::update(const FreeRoamFrame& f) {
    Career& c = career_;
    const CareerRules& R = c.rules();
    CareerState& S = c.state();
    const float dt = std::max(0.0f, f.dt);
    messages_.update(dt);
    bool changed = false;

    // Race flags (race +0x64 / +0x65): a new race resets them (0x62d74c), each frame updates them (0x632b50).
    if (f.mission && !mission_) c.raceStart();
    mission_ = f.mission;
    if (f.mission) c.raceFrame(f.wrecked, f.gpsShown);

    // Distance (0x42d3e0 -> 0x569460): |velocity| x dt; top speed (0x42d730).
    const float speed = std::sqrt(f.vel[0] * f.vel[0] + f.vel[1] * f.vel[1] + f.vel[2] * f.vel[2]);
    if (dt > 0) c.addDistance(speed * dt, f.mission, f.offRoad);
    if (speed > S.topSpeed()) S.topSpeed() = speed;

    // Jumps (0x4c8450): track while no wheel touches; keep the records in the stats block.
    const bool airborne = !f.onGround;
    if (!jump_.tracking) {
        jump_ = Jump{};
        jump_.tracking = airborne;
    } else if (!airborne) {
        jump_ = Jump{};
    } else {
        jump_.length += std::sqrt(f.vel[0] * f.vel[0] + f.vel[2] * f.vel[2]) * dt;  // +0x40
        if (f.vel[1] < 0) jump_.fall += -f.vel[1] * dt;                             // +0x44
        jump_.airTime += dt;                                                         // +0x3c
        jump_.height = std::max(jump_.height, f.heightAbove);                        // +0x48 (0x4c8790)
        if (jump_.airTime > S.longestAirTime()) S.longestAirTime() = jump_.airTime;  // game+0x3458
        if (jump_.length > S.longestJump()) S.longestJump() = jump_.length;          // game+0x3428
        if (jump_.fall > S.longestFall()) S.longestFall() = jump_.fall;              // game+0x342c
    }

    // Tricks (rider R4 5a, 0x5d4a80): the big-jump one-shot starts once per flight when airtime > 0.1 s,
    // height > 2 m and climbing faster than 8 m/s; the stunt counter counts it for BIKE (1) and QUAD (3).
    if (f.airtime > 0.1f) {
        if (trickArmed_ && f.heightAbove > 2.0f && f.vel[1] > 8.0f) {
            trickArmed_ = false;
            if (f.vehicleClass == 1 || f.vehicleClass == 3) ++S.tricks();
        }
    } else {
        trickArmed_ = true;  // landed (+0x254 cleared)
    }

    const float px = f.pos[0], pz = f.pos[2];
    auto d2 = [&](float x, float z) { return (x - px) * (x - px) + (z - pz) * (z - pz); };
    // minimap-fix [O asm 0x4c02da..0x4c0323]: 0x4c0260's discovery and reveal measure from the camera (0x421980) and use
    // the hub whose zone holds it (0x4a7f40 -> 0x577500; none: no list, no reveal); the pickups below stay at the vehicle.
    const float ex = f.haveEye ? f.eye[0] : px, ez = f.haveEye ? f.eye[2] : pz;
    auto e2 = [&](float x, float z) { return (x - ex) * (x - ex) + (z - ez) * (z - ez); };
    const int hub = f.zoneHub == -2 ? c.currentHub() : f.zoneHub;
    const bool hubOk = hub >= 0 && size_t(hub) < R.hubs.size();
    const bool opened = hubOk && S.hubs[size_t(hub)].opened != 0;  // H+0x248
    const bool noPage = !f.pageShown;                               // IGP+0x6b84 +0x28 <= 1e-6

    // Challenge discovery (0x4c0260 / 0x4c0150): the zone hub's challenges within 5000 m of the camera, once opened.
    if (hubOk && opened && noPage && startOf_) {
        const auto& list = R.hubs[size_t(hub)].challenges;
        for (size_t k = 0; k < list.size(); ++k) {
            float x, z;
            if (!startOf_(R.missions[size_t(list[k])].name, x, z) || e2(x, z) >= kRevealRadius * kRevealRadius) continue;
            if (c.discoverChallenge(hub, int(k))) messages_.push(1, -1), changed = true;
        }
    }

    // POIs: screen reveal (0x4c0260 over 0x4ed950's list), pickup by distance (0x4ed0a0), livery contact (0x4fcb60).
    // PK09: both go through the POI grid in FUEL's order. 0x4ed950: the cells within 5000 m (0x459c4000), at most 256
    // entries (later ones overwrite the last slot).
    std::vector<int> near;
    grid_.query(ex, ez, kRevealRadius, [&](int i) {
        if (near.size() < 256) near.push_back(i);
        else near.back() = i;
        return true;
    });
    for (int i : near) {
        Poi& p = pois_[size_t(i)];
        if (e2(p.x, p.z) >= kRevealRadius * kRevealRadius) continue;
        if (!p.taken && opened && (p.type == 2 || p.type == 3) && f.onScreen && f.onScreen(p.x, f.pos[1], p.z)) {
            if (p.type == 2 && noPage && c.revealLivery(p.hub, p.index)) messages_.push(2, -1), changed = true;  // 0x4c00c0
            if (p.type == 3 && noPage && c.revealVista(p.hub, p.index)) messages_.push(3, -1), changed = true;   // 0x4c01d0
        }
    }
    // 0x4ed0a0: the cells within 30 m (0x41f00000); entries within 4 m that are not liveries. The message kind / param
    // live across the walk; the first entry not taken yet is taken, pushes the kind set so far (if any) with
    // "SaveGameData", and ends the walk for this frame.
    {
        int kind = 0, param = -1;
        grid_.query(px, pz, 30.0f, [&](int i) {
            Poi& p = pois_[size_t(i)];
            if (d2(p.x, p.z) >= kPickupRadius * kPickupRadius || p.type == 2) return true;
            if (p.type == 3 && p.index >= 0 && c.visitVista(p.hub, p.index)) {
                kind = f.mission ? 6 : 7;  // 7 - (0x633170(race) != 0)
                param = int(R.hubs[size_t(p.hub)].vistaPoints[size_t(p.index)].text);
                changed = true;
            }
            if (p.taken) return true;
            p.taken = true;
            if (kind) messages_.push(kind, param), save_ = true;
            return false;
        });
    }
    // 0x4fcb60 on contact with the spawned IT_LIVERY [port: within the 4 m pickup radius, in POI index order].
    for (Poi& p : pois_) {
        if (p.type != 2 || p.taken || d2(p.x, p.z) >= kPickupRadius * kPickupRadius) continue;
        p.taken = true;
        if (c.pickLivery(p.hub, p.index)) {
            const auto& l = R.hubs[size_t(p.hub)].liveries[size_t(p.index)];
            const int v = R.vehicleIndex(l.mesh);
            messages_.push(f.mission ? 4 : 5, v >= 0 ? int(R.vehicles[size_t(v)].nameText) : -1);
            save_ = true;  // [port: FUEL's 0x4fcb60 does not save; its other save points are not traced]
            changed = true;
        }
    }

    // Fuel barrels: IT_DYNCOLLECTMove_G vf141 0x531310 on a contact with a node whose +0xd8 has 0x20c -> 0x51d600(barrel,
    // byPlayer = +0xd8 bit 0x4), then the IT_DYNOBJ contact filter. PK09: contact = the barrel's sphere touching a body's
    // sphere or box (bodies in the frame's order: the player's first). 0x51d600: taken already -> nothing; taken = 1;
    // by the player: fuel += trunc(+0xa0), barrels + 1 / + 4, "SaveGameData", MessagesHUD kind 12; a 400 barrel (by
    // anyone) takes every other 400 collectible within 5 m (d^2 < 25, x/z) too (a pack of 4) [O].
    for (size_t i = 0; i < barrels_.size(); ++i) {
        FuelBarrel& b = barrels_[i];
        for (const ContactBody& body : f.bodies) {
            if (b.taken) break;
            if ((body.flags & 0x20c) == 0 || !touches(b, body)) continue;
            b.taken = true;
            picked_.push_back(i);
            if (body.flags & 0x4) {
                const uint32_t got = c.collectFuel(b.amount);
                save_ = true;
                messages_.push(12, int(got));
                changed = true;
            }
            if (int(b.amount) == 400)
                for (size_t j = 0; j < barrels_.size(); ++j) {
                    FuelBarrel& o = barrels_[j];
                    if (j == i || o.taken || o.amount != 400.0f) continue;
                    if ((o.x - b.x) * (o.x - b.x) + (o.z - b.z) * (o.z - b.z) < 25.0f) o.taken = true, picked_.push_back(j);
                }
        }
    }

    // Achievements [port: after a change and once a second for the counters FUEL keeps updating].
    trophyClock_ += dt;
    if (changed || trophyClock_ >= 1.0f) {
        trophyClock_ = 0;
        for (int id : c.checkTrophies()) newTrophies_.push_back(id), save_ = true;
    }
}

bool FreeRoam::catchTraffic(int hub, int k) {
    const int msg = career_.catchTraffic(hub, k);
    if (msg == 0) return false;
    save_ = true;                 // Console "SaveGameData"
    // PK08: kind 8 (0x4afc70) shows the vehicle's picture and name: the param is the traffic entry's vehicle (rules index).
    int param = -1;
    const auto& hubTraffic = career_.rules().hubs[size_t(hub)].traffic;
    if (msg == 8 && k >= 0 && size_t(k) < hubTraffic.size()) param = career_.rules().vehicleIndex(hubTraffic[size_t(k)].mesh);
    messages_.push(msg, param, 5.0f);  // 0x4afa70(5.0)
    for (int id : career_.checkTrophies()) newTrophies_.push_back(id);  // [port: the achievements see it at once]
    return true;
}

}  // namespace fuel
