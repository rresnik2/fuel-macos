#include "race_rules.hpp"

#include "physics/fuel_math.hpp"
#include "road_graph.hpp"
#include "road_types.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace fuel {

namespace {

float dist2d(const RaceRules::Vec3& a, const RaceRules::Vec3& b) { return std::hypot(a[0] - b[0], a[2] - b[2]); }

bool set(const RaceRules::Vec3& p) { return p[0] != 0 || p[2] != 0; }

// 0x644980 (quaternion from a rotation vector: angle |v|, axis v / |v|), then 0x410510 applied to
// (1, 0, 0) (0xa06338) and 0x410050(1.0) (normalised): the gate's side direction.
std::array<float, 3> sideOf(const std::array<float, 3>& rv) {
    const double a = std::sqrt(double(rv[0]) * rv[0] + double(rv[1]) * rv[1] + double(rv[2]) * rv[2]);
    if (a < 1e-6) return {1, 0, 0};
    const double s = std::sin(a * 0.5) / a, qx = rv[0] * s, qy = rv[1] * s, qz = rv[2] * s, qw = std::cos(a * 0.5);
    // q (1,0,0) q*: the first column of the rotation matrix.
    double x = 1 - 2 * (qy * qy + qz * qz), y = 2 * (qx * qy + qw * qz), z = 2 * (qx * qz - qw * qy);
    const double l = std::sqrt(x * x + y * y + z * z);
    if (l > 1e-6) x /= l, y /= l, z /= l;
    return {float(x), float(y), float(z)};
}

}  // namespace

bool segmentsCross(float ax, float az, float bx, float bz, float cx, float cz, float dx, float dz) {
    // 0x75a440 with p1 = a, p2 = b (the gate), EDI = c (previous position), EDX = d (extended move).
    auto cls = [](float v) { return v > 0 ? 2 : v < 0 ? 1 : 0; };
    const float ex = bx - ax, ez = bz - az;
    const int sc = cls((cz - az) * ex - (bz - az) * (cx - ax));
    const int sd = cls((dz - az) * ex - (bz - az) * (dx - ax));
    if (sc == sd) {
        if (sc != 0) return false;
        // Collinear: overlapping along the gate line.
        if (ex * (cx - ax) + ez * (cz - az) >= 0) {
            if ((cz - bz) * ez + (cx - bx) * ex > 0 && (dx - bx) * ex + (dz - bz) * ez > 0) return false;
            return true;
        }
        return ex * (dx - ax) + ez * (dz - az) >= 0;
    }
    // 0x75a3f0 (EAX = c, EDX = d): sides of a and b against the move.
    auto side = [&](float px, float pz) { return cls((dx - cx) * (pz - cz) - (px - cx) * (dz - cz)); };
    return side(ax, az) != side(bx, bz);
}

float missionGateWidth(const RoadGraph& graph, const RoadTypeParams* types, size_t typeCount, float x, float z) {
    int rec = -1, k = 0;
    float qx = 0, qz = 0;
    float w;
    if (graph.nearestRoad(x, z, 1000.0f, rec, k, qx, qz) < 0) return 20.0f;  // none within 1000 m (then none within 20 m)
    const float d2 = graph.nearestRoad(x, z, 20.0f, rec, k, qx, qz);
    if (d2 < 0 || d2 > 4.0f || rec < 0) return 20.0f;  // 10 + 10
    const int type = graph.rec(size_t(rec)).type;
    if (type == 0x12) {
        w = 16.0f;
    } else {
        if (size_t(type) >= typeCount) return 20.0f;
        switch (type) {
        case 0: case 1: case 2: case 3: case 4: case 5: case 7: case 8: case 10: case 11: case 13: case 14:
            w = types[type].cc8;  // table 0xa678a8 = record +0xc8
            break;
        default:
            w = types[type].core;  // 0xa678a4 = +0xc4
        }
        if (w < 1e-6f) return 20.0f;
    }
    // 0x4fe0d7: v = w + 2; 2 max(8, v) (the "< 4" branch cannot be reached once v >= 8).
    return 2.0f * std::max(8.0f, w + 2.0f);
}

bool RaceRules::passes(const Gate& g, const Vec3& from, const Vec3& to) const {
    if (dist2d(to, g.centre) > gateRadius) return false;  // 0x5c1ad0: sqrt(d^2) <= +0x14c
    // The move is stretched by 10% (0x99648c = 0.1) past the current position.
    const float ex = to[0] + (to[0] - from[0]) * 0.1f, ez = to[2] + (to[2] - from[2]) * 0.1f;
    return segmentsCross(g.ax, g.az, g.bx, g.bz, from[0], from[2], ex, ez);
}

void RaceRules::begin(const Race& r, size_t racerCount, float targetTime) {
    race = r;
    mode = r.gameMode();
    route.clear();
    aiPath.clear();
    aiTargets.clear();
    knockedAt.clear();
    gates.clear();
    // Every racer's gates are all the checkpoints (0x4f6600 copies the whole list, flag-4 AI path
    // points included; the flags only choose the flag models, 0x4f73b0) and then the finish; with
    // laps the finish closes every lap (0x5c1ca0: index % (n + 1) == n is the finish).
    const std::vector<Vec3>& path = r.checkpoints;
    std::vector<Gate> lap;
    const float radius = r.scenario == 7 || r.scenario == 8 ? kGateRadiusBaja : kGateRadius;
    // World-course missions (type 0x1d, 0x4f4db0): 0x5c1360 hands them to 0x5c16c0, whose gates use
    // the mission's own half-widths (+0x188 per checkpoint, +0x180 the finish) and keep R = the
    // radius of 0x62c540 (no widest-gate rule).
    auto gateAt = [&](const Vec3& c, const std::array<float, 3>& rot, float ownHalf) {
        Gate g;
        g.centre = c;
        g.half = r.worldCourse ? ownHalf : gateWidth ? gateWidth(c[0], c[2]) * 0.5f : radius;
        const auto sd = sideOf(rot);
        g.ax = c[0] + sd[0] * g.half, g.az = c[2] + sd[2] * g.half;
        g.bx = c[0] - sd[0] * g.half, g.bz = c[2] - sd[2] * g.half;
        return g;
    };
    for (size_t i = 0; i < r.checkpoints.size(); ++i)
        lap.push_back(gateAt(r.checkpoints[i], i < r.checkpointRot.size() ? r.checkpointRot[i] : std::array<float, 3>{0, r.finishYaw, 0},
                             i < r.checkpointHalf.size() ? r.checkpointHalf[i] : 0.0f));
    if (set(r.finish)) lap.push_back(gateAt(r.finish, r.finishRot, r.finishHalf));
    gateRadius = r.worldCourse ? radius : 0;
    if (!r.worldCourse)
        for (const Gate& g : lap) gateRadius = std::max(gateRadius, g.half);  // 0x5c1360: R = the widest half-width
    // port: the catch-AI player has no gates (FUEL gives it the same list; the catches decide).
    if (mode != Race::GmCatchAI)
        for (int l = 0; l < (r.loops() ? r.laps : 1); ++l) gates.insert(gates.end(), lap.begin(), lap.end());
    for (const Gate& g : gates) route.push_back(g.centre);
    if (mode == Race::GmHelicopter || mode == Race::GmCatchAI) {
        Vec3 from = r.start;
        if (mode == Race::GmHelicopter && !r.aiGrid.empty()) from = {r.aiGrid[0][0], r.start[1], r.aiGrid[0][1]};
        aiPath.push_back(from);
        aiPath.insert(aiPath.end(), path.begin(), path.end());
        if (set(r.finish)) aiPath.push_back(r.finish);
    }
    if (ground) {
        auto snap = [&](Vec3& p) {
            const float h = ground(p[0], p[2]);
            if (std::isfinite(h) && std::fabs(p[1] - h) > kSnap) p[1] = h;
        };
        for (Vec3& p : route) snap(p);
        for (size_t i = 0; i < gates.size(); ++i) gates[i].centre = route[i];
        for (Vec3& p : aiPath) snap(p);
    }
    aiTargets.assign(aiPath.size() > 1 ? aiPath.begin() + 1 : aiPath.end(), aiPath.end());
    racers.assign(std::max<size_t>(1, racerCount), Racer{});
    finishOrder.clear();
    state = Countdown;
    outcome = Pending;
    reason.clear();
    clock = -3.0;
    timed = r.timeLimit > 0;
    timeLeft = timed ? r.timeLimit : 0;
    helicopter = mode == Race::GmHelicopter;
    heliNext = 0;
    heliHavePrev = false;
    heliLanded = false;
    hel = RaceHelicopter{};
    if (helicopter) {
        // 0x620080 (via 0x6211a0): the display start is the mission start + (0, 0, 100) turned by the mission
        // rotation (or the forced start x / z), 3 m above the ground; the required speed is the route length /
        // time A (FUEL: the road route 0x5c2150; port: the polyline).
        const Pose sp = startPose();
        Vec3 from = sp.pos;
        if (!r.aiGrid.empty()) {
            from = {r.aiGrid[0][0], r.start[1], r.aiGrid[0][1]};
        } else {
            const fv::Vec3 ahead = fv::rotate({sp.quat[0], sp.quat[1], sp.quat[2], sp.quat[3]}, {0, 0, RaceHelicopter::kStartAhead});
            from = {r.start[0] + ahead.x, r.start[1] + ahead.y, r.start[2] + ahead.z};
        }
        float length = 0;
        Vec3 prev = r.start;
        for (size_t i = 1; i < aiPath.size(); ++i) {
            length += dist2d(prev, aiPath[i]);
            prev = aiPath[i];
        }
        // port: without a target time (AddMissionTime 1st value) the helicopter flies at 100 km/h.
        const float timeA = targetTime > 0 ? targetTime : length / 27.8f;
        hel.ground = ground;
        const Vec3 end = set(r.finish) ? r.finish : aiPath.back();
        hel.init(from, end, {sp.quat[0], sp.quat[1], sp.quat[2], sp.quat[3]}, length, timeA, r.start);
        heli = hel.disp;
        heliPrev = hel.disp;
    }
}

void RaceRules::heliRecord() {
    // 0x620460: the record's current position = the display position; the 40 m step (within 40 m (xz) of the
    // target, not yet on the last gate: next waypoint, and the record's "pass" byte +0 when its target is the
    // helicopter's); its gate passing (port_RaceRules_update 0x5c1d40: RaceRec_TestGate 0x5c1ad0 (helicopter kind:
    // the radius test is skipped, the move must still cross the gate line 0x75a440) or the pass byte, which it
    // clears); the next target (the record's) at the cruise height.
    // heli-fix: PK07 kept the byte but nothing read it, so a helicopter circling a checkpoint wider than its line
    // (Cam_SmoothV3 turn radius ~ speed / 0.3 s^-1) never passed it (event-check Valley 4 / Fields / Lake Powell).
    const Vec3 cur = hel.disp;
    const Vec3 recTarget = heliNext < gates.size() ? gates[heliNext].centre : hel.target;
    hel.waypointStep(int(gates.size()), recTarget);
    if (heliNext < gates.size()) {
        const Gate& g = gates[heliNext];
        const Vec3 from = heliHavePrev ? heliPrev : cur;
        const float ex = cur[0] + (cur[0] - from[0]) * 0.1f, ez = cur[2] + (cur[2] - from[2]) * 0.1f;
        if (segmentsCross(g.ax, g.az, g.bx, g.bz, from[0], from[2], ex, ez) || hel.forcePass) {
            hel.forcePass = false;
            ++heliNext;
        }
    }
    heliPrev = cur, heliHavePrev = true;
    if (heliNext >= gates.size()) heliLanded = true;
    if (heliNext < gates.size()) {
        hel.target[0] = gates[heliNext].centre[0];
        hel.target[2] = gates[heliNext].centre[2];
        hel.target[1] = hel.cruise;
    }
}

void RaceRules::heliStep(float dt, const Vec3& player) {
    hel.timeLeft -= dt;
    hel.cruiseHeight();
    hel.wobble(dt);
    heliRecord();
    hel.velocity(dt, player);
    // Port safety clamp (heli-fix; FUEL has none): once time A has run out 0x620670 returns before normalising the
    // steering direction (+0xc = target - display, raw), and the next frame's wobble 0x620370 adds that vector
    // turned by cos(t) R1 + cos(0.8 t) R2 to the position. Fed back through display -> +0xc that map has a spectral
    // radius up to sqrt 2, so FUEL's display position diverges (EmuRun HD1: emulated frames of 0x620370 / 0x620670
    // on FUEL.exe) whenever its helicopter has not landed by time A; the port keeps the wobble's 1 m by giving it
    // the unit direction (the velocity is untouched: it stays FUEL's last one, as in FUEL).
    {
        const float l2 = hel.dir[0] * hel.dir[0] + hel.dir[1] * hel.dir[1] + hel.dir[2] * hel.dir[2];
        if (l2 > 1.0f + 1e-4f) {
            const float k = 1.0f / std::sqrt(l2);
            for (float& c : hel.dir) c *= k;
        }
    }
    hel.integrate(dt);
    hel.attitude(dt);
    heli = hel.disp;
}

bool RaceRules::usesRoute(size_t i) const { return !(mode == Race::GmCatchAI && i > 0); }

void RaceRules::aiCheckpointBlock(size_t i, const Vec3& centre) {
    // 0x6182b0 asm 0x618497..0x6184d0: (target.z - rec.z)^2 + (target.x - rec.x)^2 < [0x996184] (25) -> record bytes
    // +0 / +1 = 1. The target (record +0x108 = racer +0x12d4 / +0x12dc) is the gate the record heads for (0x5c1ca0).
    if (i >= racers.size() || !usesRoute(i)) return;
    Racer& rc = racers[i];
    if (rc.finished >= 0 || rc.out || rc.next >= gates.size()) return;  // (0x5c1d40 returns before reading the byte)
    const Vec3& t = gates[rc.next].centre;
    const float dz = t[2] - centre[2], dx = t[0] - centre[0];
    if (dz * dz + dx * dx < 25.0f) rc.forcePass = rc.forcePass1 = true;
}

void RaceRules::update(double dt, const std::vector<Vec3>& pos) {
    clock += dt;
    if (state == Countdown) {
        if (clock >= 0) state = Running;
        return;
    }
    if (state == Finished || pos.empty()) return;
    auto end = [&](Outcome o, const std::string& why) {
        state = Finished;
        outcome = o;
        reason = why;
    };
    // Gates (or path points) for every racer.
    const size_t perLap = route.empty() ? 0 : route.size() / size_t(race.loops() ? std::max(1, race.laps) : 1);  // checkpoints + finish
    for (size_t i = 0; i < racers.size() && i < pos.size(); ++i) {
        Racer& rc = racers[i];
        const std::vector<Vec3>& line = lineFor(i);
        if (rc.finished >= 0 || rc.out || rc.next >= line.size()) continue;
        const Vec3 from = rc.havePrev ? rc.prev : pos[i];
        rc.prev = pos[i], rc.havePrev = true;
        if (usesRoute(i)) {
            // FUEL_GATE_TRACE=R (rain-ai, debug): every 6th frame each racer within R m of the gate it heads for (distance,
            // position along the gate line from its centre, side of it), and every pass made by a checkpoint block.
            static const char* gt = std::getenv("FUEL_GATE_TRACE");
            if (gt) {
                const Gate& g = gates[rc.next];
                const float d = dist2d(pos[i], g.centre);
                static int frame = 0;
                if (i == 0) ++frame;
                if (d < std::atof(gt) && frame % 6 == 0) {
                    const float ux = g.bx - g.ax, uz = g.bz - g.az, ul = std::hypot(ux, uz);
                    const float along = ((pos[i][0] - g.ax) * ux + (pos[i][2] - g.az) * uz) / std::max(ul, 1e-6f) - ul * 0.5f;
                    const float side = ((pos[i][0] - g.ax) * uz - (pos[i][2] - g.az) * ux) / std::max(ul, 1e-6f);
                    std::printf("gate trace t=%.2f racer %zu gate %zu: d %.1f (R %.1f) along %.1f (half %.1f) side %.2f at %.1f %.1f %.1f\n", clock, i,
                                rc.next, d, gateRadius, along, g.half, side, pos[i][0], pos[i][1], pos[i][2]);
                }
            }
            // 0x5c1d40: the line test or the record's byte +0 (rain-ai: an AI racer's checkpoint block, aiCheckpointBlock).
            const bool crossed = passes(gates[rc.next], from, pos[i]);
            if (!crossed && !rc.forcePass) continue;
            if (gt && !crossed) std::printf("gate trace t=%.2f racer %zu gate %zu: passed by its checkpoint block\n", clock, i, rc.next);
            rc.forcePass = false;
        } else if (dist2d(pos[i], line[rc.next]) >= kGateRadius) {
            continue;  // hunted racers (port: they have no gates of their own here)
        }
        const size_t passed = rc.next++;
        if (i == 0 && timed && passed + 1 < line.size()) {
            const size_t k = perLap ? passed % perLap : passed;
            if (k < race.checkpoints.size()) timeLeft += k < race.checkpointBonus.size() ? race.checkpointBonus[k] : race.checkBonus;
        }
        if (rc.next >= line.size()) rc.finished = clock, finishOrder.push_back(i);
    }
    if (timed) timeLeft = std::max(0.0f, timeLeft - float(dt));

    // Knock-out (0x6217b0, after the AI update, scenario 3): with the racers still in, in race
    // order, T = gates of the race - AI racers at the start + eliminations so far; when at most one
    // racer has passed no more than T gates, the last one is out (an AI stops, the player loses).
    // So the last n gates (n = AI racers) each knock out the racer last through them.
    if (mode == Race::GmKnockOut && racers.size() > 1) {
        const std::vector<int> order = positions(pos);
        size_t last = 0;
        int worst = 0;
        for (size_t i = 0; i < racers.size(); ++i)
            if (!racers[i].out && order[i] > worst) worst = order[i], last = i;
        const size_t threshold = route.size() + knockedAt.size() - (racers.size() - 1);
        size_t behind = 0;
        for (const Racer& rc : racers) behind += !rc.out && rc.next <= threshold;
        if (behind <= 1 && route.size() + knockedAt.size() >= racers.size() - 1) {
            racers[last].out = true;
            racers[last].outAt = clock;
            knockedAt.push_back(racers[last].next);
            if (last == 0) return end(Lost, "knocked out at checkpoint " + std::to_string(racers[0].next + 1));
        }
        bool alone = true;
        for (size_t i = 1; i < racers.size(); ++i) alone = alone && racers[i].out;
        if (alone) return end(Won, "every opponent knocked out");
    }
    // Hunt: touch every hunted racer before the time runs out.
    if (mode == Race::GmCatchAI) {
        size_t caught = 0;
        for (size_t i = 1; i < racers.size() && i < pos.size(); ++i) {
            if (!racers[i].out && dist2d(pos[0], pos[i]) < kCatchDistance && std::fabs(pos[0][1] - pos[i][1]) < 5) {
                racers[i].out = true;
                racers[i].outAt = clock;
            }
            caught += racers[i].out;
        }
        if (racers.size() > 1 && caught + 1 == racers.size()) return end(Won, "every contender caught");
    }
    // Helicopter (PK07): FUEL's flight model while the mission runs.
    if (helicopter && !heliLanded) heliStep(float(dt), pos[0]);
    // The player's finish.
    if (racers[0].finished >= 0) {
        if (timed && racers.size() == 1) return end(Won, "finished in time");
        if (helicopter) return end(Won, "landed before the helicopter");
        const int p = playerPosition(pos);
        return end(p == 1 ? Won : Lost, p == 1 ? "finished first" : "finished " + std::to_string(p) + (p == 2 ? "nd" : p == 3 ? "rd" : "th"));
    }
    if (helicopter && heliLanded) return end(Lost, "the helicopter landed first");
    if (timed && timeLeft <= 0) return end(Lost, "out of time");
}

double RaceRules::progress(size_t i, const Vec3& at) const {
    const Racer& r = racers[i];
    if (r.finished >= 0) return 1e12 - r.finished;
    if (r.out) return -1e12 + r.outAt;
    const std::vector<Vec3>& line = lineFor(i);
    if (r.next >= line.size()) return double(r.next) * 1e6;
    return double(r.next) * 1e6 - dist2d(at, line[r.next]);
}

std::vector<int> RaceRules::positions(const std::vector<Vec3>& at) const {
    std::vector<double> score;
    for (size_t i = 0; i < racers.size(); ++i) score.push_back(i < at.size() ? progress(i, at[i]) : -2e12);
    std::vector<int> pos(racers.size(), 1);
    for (size_t i = 0; i < racers.size(); ++i)
        for (size_t j = 0; j < racers.size(); ++j) pos[i] += j != i && (score[j] > score[i] || (score[j] == score[i] && j < i));
    return pos;
}

// Stars as FUEL counts them (0x578b50 over the career lists; P39): rank 0/1/2 gives 3/2/1. medals: a career race is won
// by finishing 1st (end-condition list 0x62cff0) and its rank is the race's level (race +0x3e8, 0x630dd0), not the place.
// Solo, helicopter and hunt events exist only as challenges, which give fuel but no stars (0x630ca0). Career progress
// itself (a better rank only) is kept by fuel::Career (career.hpp).
int RaceRules::stars(const std::vector<Vec3>&) const {
    if (outcome != Won || level < 0) return 0;
    if (racers.size() == 1 || mode == Race::GmCatchAI || helicopter) return 0;
    return 3 - std::clamp(level, 0, 2);
}

std::string RaceRules::result(const std::vector<Vec3>& at) const {
    char buf[256];
    const double t = std::max(0.0, clock);
    const char* o = outcome == Won ? "WON" : outcome == Lost ? "LOST" : "RUNNING";
    std::snprintf(buf, sizeof buf, "%s %d:%04.1f gate %zu/%zu P%d/%zu stars %d%s%s", o, int(t / 60), t - int(t / 60) * 60,
                  std::min(racers[0].next, route.size()), route.size(), playerPosition(at), racers.size(), stars(at), reason.empty() ? "" : ": ",
                  reason.c_str());
    return buf;
}

// ---- PK01: the pre-start hold (0x5279c0 -> 0x62ce40) ----

RaceRules::Pose RaceRules::startPose() const {
    Pose p;
    p.pos = race.start;
    if (ground) {  // 0x62d480: start y = the detailed terrain height (0x6407e0)
        const float h = ground(p.pos[0], p.pos[2]);
        if (std::isfinite(h)) p.pos[1] = h;
    }
    // 0x620b50 -> 0x644980: angle |v|, axis v / |v|.
    const std::array<float, 3>& v = race.startRot;
    const double a = std::sqrt(double(v[0]) * v[0] + double(v[1]) * v[1] + double(v[2]) * v[2]);
    if (a >= 1e-6) {
        const double s = std::sin(a * 0.5) / a;
        p.quat = {float(v[0] * s), float(v[1] * s), float(v[2] * s), float(std::cos(a * 0.5))};
    }
    return p;
}

float RaceRules::headingOf(const std::array<float, 4>& q) {
    // (0, 0, 1) turned by q: the third column of its matrix; then y = 0 (0x529480 at 0x52959a).
    const double x = q[0], y = q[1], z = q[2], w = q[3];
    const double fx = 2 * (x * z + w * y), fz = 1 - 2 * (x * x + y * y);
    if (std::sqrt(fx * fx + fz * fz) < 1e-6) return 0.0f;  // Quat_FromArc: identity for a zero vector
    return float(std::atan2(fx, fz));
}

bool RaceRules::checkpointRespawnPose(int raceState, float elapsed, const Pose& start, const Pose* rtcSlot, Pose& out,
                                      const bool* online) {
    if (raceState == 0 || raceState == 1 || raceState == 5 || raceState == 7 || raceState == 6) return false;
    if (0.0f < elapsed) {  // fldz; fcomp +0x244: C0 when 0 < +0x244
        out = start;
        return true;
    }
    if (online) return *online;  // +0x3b8 -> 0x62d250
    out = rtcSlot ? *rtcSlot : start;  // 0x539b30 -> 0x514b00, else the start pose
    return true;
}

}  // namespace fuel
