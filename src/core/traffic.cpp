// FUEL's free-roam road traffic (see traffic.hpp). Decoded from FUEL.exe by P58: update 0x5c3010, cell scan
// 0x5c36b0, candidates 0x5c3380 / 0x5c33f0, clock state machine 0x5c3530, slots 0x5c3d40 / 0x5c3d70, spawn
// 0x5c4640, placement 0x5c4980, free 0x5c4890, reset 0x5c3210, clock 0x6de150 / 0x6ddfa0 / 0x6de0a0.
#include "traffic.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace fuel {

namespace {
// R(x) of the clock helpers: round half up (positive), towards the larger value for negatives (0x9960c0, 0x9dc020).
int64_t roundMs(double x) { return x >= 0 ? int64_t(x + 0.5) : int64_t(x + 0.5 - 0.9999999); }
// 0x6de150: seconds into the period P of the millisecond clock t.
float phase(uint64_t t, float P) {
    const int64_t p = roundMs(double(P) * 1000.0);
    return p > 0 ? float(double(t % uint64_t(p)) * 0.001) : 0.0f;
}
uint64_t advance(uint64_t t, float dt) { return uint64_t(int64_t(t) + roundMs(double(dt) * 1000.0)); }  // 0x6ddfa0
using Q = std::array<float, 4>;
Q qmul(const Q& a, const Q& b) {
    return {a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1], a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
            a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3], a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2]};
}
// 0x6443a0: the shortest rotation taking unit u to unit v.
Q arc(const float u[3], const float v[3], const float fallbackAxis[3]) {
    const float d = u[0] * v[0] + u[1] * v[1] + u[2] * v[2];
    if (d < -0.999999f) return {fallbackAxis[0], fallbackAxis[1], fallbackAxis[2], 0};
    Q q{u[1] * v[2] - u[2] * v[1], u[2] * v[0] - u[0] * v[2], u[0] * v[1] - u[1] * v[0], 1 + d};
    const float l = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    for (float& c : q) c /= l;
    return q;
}
std::array<float, 3> rotate(const Q& q, const std::array<float, 3>& v) {
    const Q p{v[0], v[1], v[2], 0}, c{-q[0], -q[1], -q[2], q[3]};
    const Q r = qmul(qmul(q, p), c);
    return {r[0], r[1], r[2]};
}
// The byte table at world +0x5bc84 that picks the model and colour is filled at run time by code not found
// (notes [U]); the port uses a fixed hash of the same index.
float spawnRandom(int idx) {
    uint32_t h = uint32_t(idx & 0xff) * 2654435761u;
    h ^= h >> 15, h *= 2246822519u, h ^= h >> 13;
    return float(h & 0xff) / 255.0f;
}
}  // namespace

int Traffic::loopsFound() const {
    int n = 0;
    for (const Cell& c : cells_) n += c.n > 0;
    return n;
}

void Traffic::freeVehicle(int ci, int k) {
    Cell& c = cellAt(ci);
    const int s = c.slot[k];
    if (s < 0) return;
    slots_[s] = Slot{};
    c.slot[k] = -1;
}

void Traffic::reset() {
    for (int ci = 0; ci < 16; ++ci) {
        for (int k = 0; k < 16; ++k) freeVehicle(ci, k);
        cells_[ci].key = 0x7fff7fff;
        cells_[ci].n = 0;
    }
    for (size_t i = 0; i < specials_.size(); ++i) freeVehicle(int(16 + i), 0);
}

void Traffic::setSpecials(std::vector<SpecialDef> defs) {
    for (size_t i = 0; i < specials_.size(); ++i) freeVehicle(int(16 + i), 0);
    specialDefs_ = std::move(defs);
    specials_.assign(specialDefs_.size(), Cell{});
    specialSearched_.assign(specialDefs_.size(), 0);
}

void Traffic::removeSpecial(int i) {
    if (i < 0 || size_t(i) >= specials_.size()) return;
    freeVehicle(16 + i, 0);
    specials_[size_t(i)].n = 0;
    specialSearched_[size_t(i)] = 1;  // n = 0: never offered again
}

void Traffic::updateSpecials(float dt, float camX, float camZ, uint64_t G) {
    for (size_t i = 0; i < specials_.size(); ++i) {
        const SpecialDef& e = specialDefs_[i];
        Cell& c = specials_[i];
        const int ci = int(16 + i);
        const float cx = std::clamp(camX, e.x0, e.x1), cz = std::clamp(camZ, e.z0, e.z1);
        if ((cx - camX) * (cx - camX) + (cz - camZ) * (cz - camZ) > kSpecialRange * kSpecialRange) {
            freeVehicle(ci, 0);  // 0x5c4070: entries not in the camera's set lose their vehicle (and loop: kept here)
            continue;
        }
        if (!specialSearched_[i]) {  // 0x5c4370 first use: RoadLoop_FindInBox, speed 17, P = L / 17
            specialSearched_[i] = 1;
            if (graph_.findLoop(e.x0, e.z0, e.x1, e.z1, c.loop)) {
                c.speed = 17.0f;
                c.P = c.loop.length * 0.05882353f;
                c.state = 0;
                c.n = 1;
            }
            if (std::getenv("FUEL_TRAFFIC_TRACE"))
                std::printf("special traffic %d/%d: box (%.0f..%.0f, %.0f..%.0f) loop %.0f m%s\n", e.hub, e.index, e.x0, e.x1, e.z0, e.z1,
                            c.loop.length, c.n ? "" : " (none)");
        }
        if (c.n <= 0) continue;
        if (c.slot[0] < 0 && c.state != 0) c.state = 3;  // 0x5c4370: no vehicle -> catch up 3x
        uint64_t tc = G;
        stateMachine(c, tc, dt);
        tc = advance(tc, e.frac * c.P);  // 0x6ddfa0(+0x30 x P)
        float s = phase(tc, c.P) * c.speed;
        if (s > c.loop.length) s -= c.loop.length;
        if (c.slot[0] >= 0) slots_[c.slot[0]].s = s;
        const auto p = graph_.loopPoint(c.loop, s);
        const float d2 = (p[0] - camX) * (p[0] - camX) + (p[1] - camZ) * (p[1] - camZ);
        if (std::getenv("FUEL_TRAFFIC_TRACE") && (G / 1000) % 5 == 0 && (G % 1000) < 17)
            std::printf("special traffic %d/%d at (%.0f, %.0f), %.0f m from the camera\n", e.hub, e.index, p[0], p[1], std::sqrt(d2));
        if (!(d2 <= 1e6f) || offer(d2, ci, 0, s) < 0) freeVehicle(ci, 0);  // within 1000 m: a candidate (0x5c33f0)
    }
}

int Traffic::offer(float d2, int ci, int k, float s) {
    int worst = 0;
    for (int i = 1; i < 5; ++i)
        if (cand_[i].d2 >= cand_[worst].d2) worst = i;
    if (!(d2 <= cand_[worst].d2)) return -1;
    if (cand_[worst].cell >= 0) freeVehicle(cand_[worst].cell, cand_[worst].k);
    cand_[worst] = {d2, ci, k, s};
    return worst;
}

void Traffic::stateMachine(Cell& c, uint64_t& tc, float dt) {
    const float e = float(double(int64_t(tc) - int64_t(c.tref)) * 0.001) * c.speed;
    switch (c.state) {
    case 0:
        c.f = 1, c.tref = tc;
        break;
    case 1:
        c.f = 1, tc = c.tref;
        break;
    case 2:
        if (e < 0.1f) {
            c.state = 0, c.tref = tc;
        } else {
            if (e > 10.0f && c.f < 1.5882353f) c.f += 0.1f * dt;
            else if (c.f > 1.05f) c.f -= 0.1f * dt;
            c.tref = advance(c.tref, dt * c.f);
            tc = c.tref;
        }
        break;
    case 3:
        if (e < 0.1f) {
            c.state = 0, c.tref = tc;
        } else {
            c.tref = advance(c.tref, 3.0f * dt);
            tc = c.tref;
        }
        break;
    }
}

void Traffic::update(float dt, float camX, float camZ, float fwdX, float fwdZ, const std::function<std::array<double, 2>(int)>& where) {
    if (!enabled) {
        reset();
        for (Vehicle& v : out_) v = Vehicle{};
        return;
    }
    // The clock: at least 10 ms a frame; a jump back of more than 1 s resets (0x5c3010).
    gameMs_ = advance(gameMs_, dt);
    uint64_t G = gameMs_;
    if (last_ != 0) {
        if (G + 1000 < last_) reset();
        else if (G < last_ + 10) G = last_ + 10;
    }
    last_ = gameMs_ = G;
    for (Candidate& c : cand_) c = Candidate{};  // 0x5c3380
    // Cell scan 0x5c36b0.
    const float ax = camX + 225.0f * fwdX, az = camZ + 225.0f * fwdZ;
    const int cx = int(std::floor(camX * 0.0005f)), cz = int(std::floor(camZ / 2000.0f));
    for (int z = cz - 2; z < cz + 2; ++z)
        for (int x = cx - 2; x < cx + 2; ++x) {
            const int ci = (z & 3) * 4 + (x & 3);
            Cell& c = cells_[ci];
            const uint32_t key = (uint32_t(x) & 0xffffu) ^ (uint32_t(z) << 16);
            if (key != c.key) {
                for (int k = 0; k < 16; ++k) freeVehicle(ci, k);
                c.n = 0;
                c.key = key, c.x = x, c.z = z;
                if (graph_.findLoop(float(x) * 2000.0f, float(z) * 2000.0f, float(x + 2) * 2000.0f, float(z + 2) * 2000.0f, c.loop)) {
                    c.speed = 17.0f;
                    c.P = c.loop.length * 0.05882353f;
                    c.state = 0;
                    c.n = std::clamp(int(c.P * 0.028333334f), 1, 16);
                }
                if (std::getenv("FUEL_TRAFFIC_TRACE")) {
                    const auto p0 = c.n > 0 ? graph_.loopPoint(c.loop, 0) : std::array<float, 2>{0, 0};
                    std::printf("traffic cell (%d, %d): loop of %zu records, %.0f m, %d vehicles, starts (%.0f, %.0f), camera %.0f m away\n", x, z,
                                c.loop.recs.size(), c.loop.length, c.n, p0[0], p0[1], std::hypot(p0[0] - camX, p0[1] - camZ));
                }
                continue;
            }
            if (c.n <= 0) continue;
            uint64_t tc = G;
            stateMachine(c, tc, dt);
            const float L = c.loop.length, step = c.P / float(c.n);
            bool any = false;
            for (int k = 0; k < c.n; ++k) {
                float s = (phase(tc, c.P) + float(ci) * step * 0.03125f) * c.speed;
                if (s > L) s -= L;
                if (c.slot[k] >= 0) slots_[c.slot[k]].s = s;
                const auto p = graph_.loopPoint(c.loop, s);
                const float d1 = (p[0] - camX) * (p[0] - camX) + (p[1] - camZ) * (p[1] - camZ);
                const float d2 = (p[0] - ax) * (p[0] - ax) + (p[1] - az) * (p[1] - az);
                if (d1 > 202500.0f || d2 >= 122500.0f) {
                    freeVehicle(ci, k);
                } else if (offer(d1, ci, k, s) < 0) {
                    freeVehicle(ci, k);
                } else {
                    any = true;
                }
                tc = advance(tc, step);
            }
            if (!any && c.state != 0) c.state = 3;
        }
    updateSpecials(dt, camX, camZ, G);
    // Spawn and drive (0x5c3d70), in candidate order.
    for (Vehicle& v : out_) v.spawned = false;
    for (const Candidate& e : cand_) {
        if (e.cell < 0) continue;
        Cell& c = cellAt(e.cell);
        int si = c.slot[e.k];
        if (si < 0) {
            for (int i = 0; i < kSlots && si < 0; ++i)
                if (slots_[i].free) si = i;  // 0x5c3d40
            if (si < 0) continue;
            const float sx = float(uint16_t(c.x + e.k)), sz = float(uint16_t(c.z));
            const float r = spawnRandom(int(std::lrint(sz * 10.193578f + sx * 12.317513f)));
            Slot& sl = slots_[si];
            sl = Slot{};
            sl.free = false, sl.cell = e.cell, sl.k = e.k, sl.s = e.s;
            sl.colour = std::min(int(r * 10.0f), 9);
            sl.model = e.cell >= 16 ? 2 : r > 0.5f ? 1 : 0;
            c.slot[e.k] = int8_t(si);
            out_[size_t(si)].spawned = true;
        }
        drive(si, dt, where);
    }
    for (int i = 0; i < kSlots; ++i) {
        Vehicle& v = out_[size_t(i)];
        const Slot& sl = slots_[i];
        v.active = !sl.free && sl.placed;
        if (!v.active) {
            v.spawned = false;
            continue;
        }
        v.model = sl.model, v.colour = sl.colour;
        v.special = sl.cell >= 16 ? sl.cell - 16 : -1;
        v.pos = {sl.target[0], sl.target[1], sl.target[2]};
        v.rot = sl.q;
        const auto f = rotate(sl.q, {0, 0, 1});
        v.vel = {f[0] * 17.0f, f[1] * 17.0f, f[2] * 17.0f};  // always 17 m/s, even frozen (0x5c96e0)
    }
}

void Traffic::drive(int si, float dt, const std::function<std::array<double, 2>(int)>& where) {
    (void)dt;
    Slot& sl = slots_[si];
    Cell& c = cellAt(sl.cell);
    const RoadGraph::Loop& loop = c.loop;
    int rec = -1;
    float ux = 0, uz = 1;
    const auto P = graph_.loopPoint(loop, sl.s, &ux, &uz, &rec);
    // Freeze check: the vehicle pushed more than 5 m off its place stops its loop (state 1), then catches up (2).
    if (sl.placed && where) {
        const auto w = where(si);
        const double dx = w[0] - sl.target[0], dz = w[1] - sl.target[2];
        if (dx * dx + dz * dz > 25.0) c.state = 1;
        else if (c.state == 1) c.state = 2;
    } else if (c.state == 1) {
        c.state = 2;
    }
    const float half = rec >= 0 && world_.flatHalf ? world_.flatHalf(graph_.rec(size_t(rec)).type) : 0.0f;
    const float w = std::max(half * 0.5f, 1.5f);
    float sa = sl.s + 10.0f;
    if (sa > loop.length) sa -= loop.length;
    const auto A = graph_.loopPoint(loop, sa);
    const float fx = A[0] - P[0], fz = A[1] - P[1], fl2 = fx * fx + fz * fz;
    if (fl2 > 1e-6f) {
        const float l = std::sqrt(fl2);
        sl.fwd = {fx / l, fz / l};
    }
    // The right-hand lane: fwd x up = (-fz, fx).
    const float ox = -sl.fwd[1] * w, oz = sl.fwd[0] * w;
    const float px = P[0] + ox, pz = P[1] + oz, qx = A[0] + ox, qz = A[1] + oz;
    float tx, tz;
    const float ex = sl.target[0] - px, ez = sl.target[2] - pz;
    if (ex * ex + ez * ez >= 400.0f) tx = (px + qx) * 0.5f, tz = (pz + qz) * 0.5f;
    else tx = 0.5f * px + 0.25f * qx + 0.25f * sl.target[0], tz = 0.5f * pz + 0.25f * qz + 0.25f * sl.target[2];
    float hx = qx - tx, hz = qz - tz;
    const float hl = std::sqrt(hx * hx + hz * hz);
    if (hl > 1e-6f) hx /= hl, hz /= hl;
    else hx = sl.fwd[0], hz = sl.fwd[1];
    float n[3] = {0, 1, 0};
    const float ty = world_.ground ? world_.ground(tx, tz, n) : 0.0f;
    sl.target = {tx, ty, tz};
    // q = the yaw (Z -> heading) then the tilt onto the ground (Y -> normal) (0x6443a0, 0x410260; order [I]).
    const float Z[3] = {0, 0, 1}, Y[3] = {0, 1, 0}, fwd[3] = {hx, 0, hz}, axisY[3] = {0, 1, 0}, axisX[3] = {1, 0, 0};
    sl.q = qmul(arc(Y, n, axisX), arc(Z, fwd, axisY));
    sl.placed = true;
}

}  // namespace fuel
