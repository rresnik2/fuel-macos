// PK23: FUEL's per-vehicle effects; see vehicle_fx.hpp and ~/re-work/fuel/notes/pk23.md.
#include "vehicle_fx.hpp"

#include <algorithm>
#include <cmath>
#include <string>

#include "assembly.hpp"
#include "dpc.hpp"
#include "resources.hpp"

namespace fuel::vfx {

namespace {
V3 add(const V3& a, const V3& b) { return {a[0] + b[0], a[1] + b[1], a[2] + b[2]}; }
V3 sub(const V3& a, const V3& b) { return {a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }
V3 mul(const V3& a, float s) { return {a[0] * s, a[1] * s, a[2] * s}; }
float dot(const V3& a, const V3& b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
float len(const V3& a) { return std::sqrt(dot(a, a)); }
V3 cross(const V3& a, const V3& b) { return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]}; }
V3 normalized(const V3& a, float to = 1.0f) {  // Vec3_Normalize 0x410050(len): a x len / |a| (unchanged when |a| = 0)
    const float l = len(a);
    return l > 0 ? mul(a, to / l) : a;
}
// 0x410510 Quat_RotateVec: q v q^-1 (x y z w).
V3 rotate(const Quat& q, const V3& v) {
    const V3 u{q[0], q[1], q[2]};
    const float s = q[3];
    const V3 t = mul(cross(u, v), 2.0f);
    return add(add(v, mul(t, s)), cross(u, t));
}
float clamp01(float x) { return std::min(std::max(x, 0.0f), 1.0f); }
}  // namespace

// ---- 0x5cf170 ----------------------------------------------------------------------------------------------------------
std::vector<ExhaustPoint> findExhausts(const DpcFile& dpc, const AssembledVehicle& av) {
    std::vector<ExhaustPoint> out;
    struct PartBoxes {
        const PlacedPart* part;
        std::vector<NamedBox> boxes;
    };
    std::vector<PartBoxes> parts;
    for (const PlacedPart& p : av.parts) {  // the LOD record's parts in order; a part's object data finds the name (vt+0x60)
        if (!p.drawn() || p.shape != PartShape::Mesh) continue;
        const DpcFile::Entry* e = dpc.find(p.meshId);
        if (e == nullptr || e->classCrc != cls::Mesh) continue;
        try {
            parts.push_back({&p, parseMeshLinkBoxes(dpc.link(*e))});
        } catch (const std::exception&) {
        }
    }
    for (int k = 1; k <= 16; ++k) {  // sprintf "SFX_EXHAUST%i" (k = 1..16), AsoboCrc, 0x6a23b0; found -> list (0x5d1550)
        const uint32_t want = asoboCrc("SFX_EXHAUST" + std::to_string(k));
        bool found = false;
        for (const PartBoxes& pb : parts) {
            for (const NamedBox& b : pb.boxes) {
                if (b.name != want) continue;
                // Obb_PlaceByNode: the box frame (rows r0 r1 r2, translation in column 3) times the part's node (row-vector).
                const Mat4& W = pb.part->world;
                const V3 t{b.rows[3], b.rows[7], b.rows[11]};
                const V3 z{b.rows[8], b.rows[9], b.rows[10]};
                ExhaustPoint e;
                for (int c = 0; c < 3; ++c) {
                    e.pos[size_t(c)] = t[0] * W[size_t(c)] + t[1] * W[size_t(4 + c)] + t[2] * W[size_t(8 + c)] + W[size_t(12 + c)];
                    e.dir[size_t(c)] = -(z[0] * W[size_t(c)] + z[1] * W[size_t(4 + c)] + z[2] * W[size_t(8 + c)]);
                }
                e.dir = normalized(e.dir);
                e.index = k;
                e.dyn = pb.part->dyn;
                out.push_back(e);
                found = true;
                break;
            }
            if (found) break;
        }
    }
    return out;
}

ExhaustParams ExhaustParams::fromTables(const sfx::Tables& t) {
    ExhaustParams p;
    p.lifeLo = t.P(5320), p.lifeHi = t.P(5321);
    p.sizeLo = t.P(5323), p.sizeHi = t.P(5324);
    p.rgbA = t.P(5325), p.rgbB = t.P(5326);
    p.intervalLo = t.P(5327), p.intervalHi = t.P(5328);
    p.alpha = t.P(5329);
    p.rot = t.P(5331);
    p.push = t.P(5332);
    p.drag = t.P(5333);
    return p;
}

float flameLevel(float flash, float playerSpeedFrac, float wheel0Load) {
    // asm 0x5cf4d4..0x5cf560: x = (SpeedFrac - 0.9 [0x996180]) x 20 [0x996154] clamped to [0, 1]; y = +0x15e8 x 50
    // [0x9dc048], 0 unless > 0, at most 1; level = x y, or the flash when that is larger.
    const float x = clamp01((playerSpeedFrac - 0.9f) * 20.0f);
    float y = wheel0Load * 50.0f;
    y = y > 0 ? std::min(y, 1.0f) : 0.0f;
    const float l = x * y;
    return l < flash ? flash : l;
}

bool nearTopSpeed(float speed, float topSpeedKmh) {
    // asm 0x5cf41a..0x5cf475: r = |v| / (P619 x 0.86956525 x 0.2777778); 0 < r and (r >= 1 or r > 0.95).
    const float r = speed / (topSpeedKmh * 0.86956525f * 0.2777778f);
    return 0 < r && (r >= 1.0f || r > 0.95f);
}

float smokeAlpha(float speed, float pp5329) {
    // asm 0x5cf9f0..0x5cfa18: 15 [0x9dc058] >= |v|: (1 - |v| x 0.06666667 [0x9dc308]) x PP5329, else 0.
    return speed <= 15.0f ? (1.0f - speed * 0.06666667f) * pp5329 : 0.0f;
}

float sparkAlpha(float age, float life) {  // 0x4c5e60 (pow 0x40f970 with 0.25 [0x3e800000])
    const float t = life > 1e-6f ? age / life : 1.0f;
    return 1.0f - t > 0 ? std::pow(1.0f - t, 0.25f) : 0.0f;
}

void sparkVelocity(const Quat& q, const V3& tangent, V3& lo, V3& hi) {
    lo = rotate(q, {0, 0, -10});  // 10 [0x996494] x DAT_00a0635c (0, 0, -1)
    hi = rotate(q, {0, 10, 0});   // 10 x DAT_00a06320 (0, 1, 0)
    if (dot(lo, tangent) > 0) lo = mul(lo, -1.0f);
}

void ropeStep(std::array<V3, 4>& rope, const V3& head, float ropeLen, float thr2) {
    rope[0] = head;
    for (size_t k = 1; k < rope.size(); ++k) {
        const V3 d = sub(rope[k - 1], rope[k]);
        const float q = dot(d, d);
        if (thr2 < q) rope[k] = sub(rope[k - 1], mul(d, ropeLen / std::sqrt(q)));
    }
}

// ---- SpecialEffectsFlameEmitter_G vf06 0x5edb20 --------------------------------------------------------------------------
void FlameEmitter::update(float dt, const std::array<float, 4>& rect) {
    t += dt;
    if (stopped) return;
    if (!haveQuads) {  // count == 0: +0x18c = 1, vf08 emits the 3 quads (perEmission +0x168 = 3)
        fresh = true;
        haveQuads = true;
    }
    if (fresh) {  // the atlas entry's rect (0xa66c24 + 0x1c x +0x70: u0 +8, v0 +0xc, du +0x10, dv +0x14)
        fresh = false;
        const float u0 = rect[0], v0 = rect[1], half = rect[2] * 0.5f, v1 = rect[1] + rect[3];
        const float mid = u0 + half, u1 = mid + half;
        // +0x1a8..+0x1c4: quad 0 (the right half), +0x1c8..+0x1e4: quads 1 and 2 (the left half, the tip at v0).
        uv = {mid, v0, u1, v0, mid, v1, u1, v1, u0, v1, mid, v1, u0, v0, mid, v0};
        for (int k = 0; k < 4; ++k) quads[0].uv[size_t(k)] = {uv[size_t(2 * k)], uv[size_t(2 * k + 1)]};
        for (int q = 1; q < 3; ++q)
            for (int k = 0; k < 4; ++k) quads[size_t(q)].uv[size_t(k)] = {uv[size_t(8 + 2 * k)], uv[size_t(9 + 2 * k)]};
    }
    // L = (0.1 sin(8 t) + 1) x +0x190 (0x40f3d0 = sin); the frame: a = d x X, b = |d x a|, c = |b x d|.
    const float L = (std::sin(t * 8.0f) * 0.1f + 1.0f) * length;
    const V3& d = dir;
    const V3 a = cross(d, {1, 0, 0});  // DAT_00a06338 = (1, 0, 0)
    const V3 b = normalized(cross(d, a));
    const V3 c = normalized(cross(b, d));
    const V3 hi = add(pos, mul(d, 0.05f)), lo = sub(pos, mul(d, 0.05f));
    const V3 e1 = normalized(sub(b, c), size), e2 = normalized(add(b, c), size);  // Vec3_Normalize(+0xfc)
    const V3 Ld = mul(d, L);
    quads[0].v = {add(hi, e1), sub(hi, e2), add(hi, e2), sub(hi, e1)};  // the end cap at pos + 0.05 d
    quads[1].v = {add(lo, e1), sub(lo, e1), add(add(lo, e1), Ld), add(sub(lo, e1), Ld)};
    quads[2].v = {add(lo, e2), sub(lo, e2), add(add(lo, e2), Ld), add(sub(lo, e2), Ld)};
    colour = blue ? C4{1.2f, 1.2f, 3.0f, 1.0f} : C4{2.0f, 1.6f, 0.4f, 1.0f};  // +0x1f8 / +0x1e8
}

// ---- VehicleFx -------------------------------------------------------------------------------------------------------
VehicleFx::VehicleFx(sfx::System& sys) : sys_(sys), pp_(ExhaustParams::fromTables(sys.tables())) {
    // UniqueEmitters +0x14 (0x5fc500): a Ribbon emitter, spawn mode 0 (emits only through vf08), 1 per emission,
    // size 0.003 .. 0.005 (after 0.01 .. 0.015), accel (0, -9.81, 0), life 0.7 .. 1, velocity +-(1, 1, 1), colour
    // (10, 10, 4, 1) both, motion 3, kill flags 1, alpha 0x4c5e60, size Fade_One, atlas 0, SubCode 0x16, started.
    sfx::Well512& r = sys_.rng();
    RibbonEmitter& e = embers_;
    e.spawnMode = 0;
    e.perEmission = 1;
    e.size.lo = 0.003f, e.size.hi = 0.005f, e.size.cur = r.range(0.003f, 0.005f);
    e.accel.lo = e.accel.hi = e.accel.cur = {0, -9.81f, 0};
    e.life.lo = 0.7f, e.life.hi = 1.0f, e.life.cur = r.range(0.7f, 1.0f);
    e.vel.lo = {-1, -1, -1}, e.vel.hi = {1, 1, 1};
    for (int k = 0; k < 3; ++k) e.vel.cur[size_t(k)] = r.range(-1, 1);
    e.colourA = e.colourB = e.colour = {10, 10, 4, 1};  // [esp+0x28] of 0x5fc500: 10 [0x996494], 10, 4 [0x996378], 1
    e.motion = 3, e.killFlags = 1, e.atlas = 0, e.kind = 0x16;
    e.start();
}

float VehicleFx::parkMiller() {
    pm_ = uint32_t((uint64_t(pm_) * 16807u) % 2147483647u);
    return float(pm_) / 2147483647.0f;
}

void VehicleFx::setupVehicle(Vehicle& v, size_t slots) {  // 0x5d00c0 (slots of 0x5cf170)
    v.init = true;
    v.fxFlag = false;  // +0x1b5c = 0
    v.slots.clear();
    v.slots.resize(slots);
    sfx::Well512& r = sys_.rng();
    for (Slot& s : v.slots) {
        sfx::Emitter& e = s.smoke;  // SfxEmitter_Ctor 0x4c4730, atlas 6
        e = sfx::makeEmitter(r);
        e.atlas = 6;
        sfx::setRange3One(r, e.vel, {0, 0, 0}, {0, 0, 0});  // DAT_00a66208 (0) both, 0x4c4cf0
        sfx::setColours(r, e, {0, 0, 0, 0.5f}, {0.1f, 0.1f, 0.1f, 0.8f});  // +0x124.. / +0x134.., 0x4c4f70
        e.spawnMode = 1;
        sfx::setRange(r, e.interval, 0.03f, 0.08f);
        sfx::setRange(r, e.spacing, 0.001f, 0.002f);
        e.uvFlags = 8, e.motion = 4, e.killFlags = 1;
        e.alphaFn = sfx::Fade::OneMinusT, e.sizeFn = sfx::Fade::Quarter;  // 0x4beab0, 0x5d2fd0
        e.offset.lo = e.offset.hi = {0, 0, 0};
        sfx::redraw3(r, e.offset);  // 0x4c5240
        e.perEmission = 1, e.kind = 0x15;
        sfx::setRange(r, e.angle, -3.14159274f, 3.14159274f);
        e.gpuStatic = false;
        sfx::setRange(r, e.rotSpeed, -pp_.rot, pp_.rot);
        // SpecialEffectsFlameEmitter_G (base 0x5d1690): dir (0, 0, -1) [0xa0635c], +0x190 / +0x1a0 / +0x1a4 = 0,
        // fresh +0x18c = 1; atlas 0x30, spawn mode 0, Fade_One, SubCode 0x15, 3 quads per emission, started.
        s.flame = FlameEmitter{};
        s.flame.fresh = true;
        s.flame.start();
    }
}

void VehicleFx::emitRibbon(RibbonEmitter& e) {  // vf08 0x4be560 (as 0x5d2830; the budget 0x4bb9e0 is not ported)
    sfx::Well512& r = sys_.rng();
    for (int n = 0; n < e.perEmission; ++n) {
        RibbonParticle p;  // init vf11 0x4bba50: the emitter's current draws
        p.pos = add(e.pos, e.offset.cur);
        p.start = (e.motion == 3 || e.motion == 4) ? p.pos : e.offset.cur;
        p.accel = e.accel.cur, p.vel = e.vel.cur;
        p.life = e.life.cur, p.size = e.size.cur;
        p.colour = {e.colour[0], e.colour[1], e.colour[2], e.colour[3]};
        p.alpha = e.colour[3];
        p.ropeLen = e.rope.cur;
        p.motion = e.motion, p.killFlags = e.killFlags;
        p.rope.fill(p.pos);  // 0x5def80
        e.live.push_back(p);
        e.lastEmit = e.pos;
        e.burstDone = true;
        // the redraws: interval, accel (per axis), size, angle, rot speed, colour (one random), velocity (per axis),
        // offset (per axis), +0x15c, life.
        e.interval.cur = r.range(e.interval.lo, e.interval.hi);
        for (int k = 0; k < 3; ++k) e.accel.cur[size_t(k)] = e.accel.lo[size_t(k)] + r.uniform() * (e.accel.hi[size_t(k)] - e.accel.lo[size_t(k)]);
        e.size.cur = r.range(e.size.lo, e.size.hi);
        r.uniform(), r.uniform();  // angle, rot speed (ranges 0)
        const float u = r.uniform();
        for (int k = 0; k < 4; ++k) e.colour[size_t(k)] = (e.colourB[size_t(k)] - e.colourA[size_t(k)]) * u + e.colourA[size_t(k)];
        for (int k = 0; k < 3; ++k) e.vel.cur[size_t(k)] = e.vel.lo[size_t(k)] + r.uniform() * (e.vel.hi[size_t(k)] - e.vel.lo[size_t(k)]);
        for (int k = 0; k < 3; ++k) e.offset.cur[size_t(k)] = e.offset.lo[size_t(k)] + r.uniform() * (e.offset.hi[size_t(k)] - e.offset.lo[size_t(k)]);
        e.rope.cur = r.range(e.rope.lo, e.rope.hi);
        ++e.emissions;
        e.life.cur = r.range(e.life.lo, e.life.hi);
    }
}

void VehicleFx::runRibbon(RibbonEmitter& e, float dt, const V3& eye) {  // vf06 0x4be230
    e.delta = sub(e.pos, e.prevPos);
    if (dot(e.delta, e.delta) > 400.0f) e.delta = {0, 0, 0};
    // vf09 0x4be7f0: vf12 0x4be930 (motion 3: start + v a + acc a^2 / 2 with the age before the step), then the particle's
    // vf01 0x5defc0 (age += dt, alpha / size, the rope: head = pos, each next point pulled to +0x80 of the one before once
    // it is more than sqrt(+0x7c) away); kill flag 1 past the life, flag 2 beyond 200 m of the camera.
    for (size_t i = 0; i < e.live.size();) {
        RibbonParticle& p = e.live[i];
        const float a = p.age;
        if (p.motion == 3) p.pos = add(add(p.start, mul(p.vel, a)), mul(p.accel, a * a * 0.5f));
        else if (p.motion == 4) p.start = add(p.start, e.delta), p.pos = add(add(p.start, mul(p.vel, a)), mul(p.accel, a * a * 0.5f));
        else if (p.motion == 0) p.pos = add(p.pos, mul(p.vel, dt));
        p.age += dt;
        p.drawAlpha = sparkAlpha(p.age, p.life) * p.alpha;
        p.drawSize = p.size;  // Fade_One
        ropeStep(p.rope, p.pos, p.ropeLen, p.ropeThr2);
        bool dead = (p.killFlags & 1) && p.age > p.life;
        if (p.killFlags & 2) dead = dead || dot(sub(p.pos, eye), sub(p.pos, eye)) > 40000.0f;
        if (dead) e.live.erase(e.live.begin() + long(i));
        else ++i;
    }
    if (e.stopped) return;
    if ((e.spawnMode & 0xf) == 1) {
        e.timer += dt;
        if (e.timer > e.interval.cur) {
            e.timer = 0;
            emitRibbon(e);
        }
    }
    e.age += dt;  // +0x54 die mode 0: never stops by age or count
}

void VehicleFx::sparkCall(Vehicle& v, const VehicleIn& in, const SparkHit& h) {  // 0x5d07f0
    if (!(in.cameraDistance <= 100.0f)) return;  // veh+0x10a4 <= 100 [0x996184]
    v.fxFlag = true;                             // +0x1b5c = 1
    ++stats_.sparkCalls;
    SparkRecord* rec = nullptr;
    for (SparkRecord& s : v.sparks)  // the same body pair, either order
        if ((s.a == h.a && s.b == h.b) || (s.a == h.b && s.b == h.a)) rec = &s;
    if (rec == nullptr) {  // a new record (pool 0x5d1ad0) and its ribbon emitter (0x4bd380)
        v.sparks.push_back({});
        rec = &v.sparks.back();
        rec->a = h.a, rec->b = h.b;
        rec->e = std::make_unique<RibbonEmitter>();
        RibbonEmitter& e = *rec->e;
        sfx::Well512& r = sys_.rng();
        e.spawnMode = 1;
        e.perEmission = 20;  // 0x14
        e.size.lo = e.size.hi = 0.025f, e.size.cur = r.range(0.025f, 0.025f);
        e.interval.lo = e.interval.hi = 0.01f, e.interval.cur = r.range(0.01f, 0.01f);
        e.accel.lo = e.accel.hi = {0, -5, 0};
        for (int k = 0; k < 3; ++k) e.accel.cur[size_t(k)] = r.range(e.accel.lo[size_t(k)], e.accel.hi[size_t(k)]);  // 0x4bdd70
        e.life.lo = 0.1f, e.life.hi = 0.3f, e.life.cur = r.range(0.1f, 0.3f);
        e.offset.lo = e.offset.hi = {0, 0, 0};
        for (int k = 0; k < 3; ++k) e.offset.cur[size_t(k)] = r.range(0, 0);  // 0x4bde20
        e.vel.lo = {-1, -1, -1}, e.vel.hi = {1, 1, 1};  // -DAT_00a06308 .. DAT_00a06308
        for (int k = 0; k < 3; ++k) e.vel.cur[size_t(k)] = r.range(-1, 1);  // 0x4bdbf0
        e.colourA = e.colourB = {15, 15, 8, 1};
        const float u = r.uniform();  // 0x4bdf50
        for (int k = 0; k < 4; ++k) e.colour[size_t(k)] = (e.colourB[size_t(k)] - e.colourA[size_t(k)]) * u + e.colourA[size_t(k)];
        e.motion = 3, e.killFlags = 1, e.atlas = 0;  // +0x6c = 0, alpha 0x4c5e60, size Fade_One
        e.setPosition(h.point);
        e.kind = 0x15;
        e.offset.lo = {0, -0.3f, 0}, e.offset.hi = {0, 0.3f, 0};
        for (int k = 0; k < 3; ++k) e.offset.cur[size_t(k)] = r.range(e.offset.lo[size_t(k)], e.offset.hi[size_t(k)]);
        e.rope.lo = 0.05f, e.rope.hi = 0.15f, e.rope.cur = r.range(0.05f, 0.15f);
        e.start();
    }
    // Every call: velocity range A = q (0, 0, -10) (negated when it points along the sliding velocity R[0x14..0x16]),
    // B = q (0, 10, 0) (asm 0x5d0d8d..0x5d0ee7: 10 [0x996494] x DAT_00a0635c / DAT_00a06320); redrawn (0x4bdbf0).
    RibbonEmitter& e = *rec->e;
    rec->last = e.pos;
    V3 A, B;
    sparkVelocity(h.q, h.tangent, A, B);
    rec->hit = true;
    e.vel.lo = A, e.vel.hi = B;
    sfx::Well512& r = sys_.rng();
    for (int k = 0; k < 3; ++k) e.vel.cur[size_t(k)] = e.vel.lo[size_t(k)] + r.uniform() * (e.vel.hi[size_t(k)] - e.vel.lo[size_t(k)]);
    e.setPosition(h.point);
    // The streaks already out move with the contact (start +0x20 and the rope points +0xb8).
    const V3 d = sub(h.point, rec->last);
    for (RibbonParticle& p : e.live) {
        p.start = add(p.start, d);
        for (V3& q : p.rope) q = add(q, d);
    }
}

void VehicleFx::exhaust(Vehicle& v, VehicleIn& in, const PlayerIn& player, float dt) {  // 0x5cf2d0
    in.throttle = clamp01(in.throttle);  // +0x1088 clamped to [0, 1] and written back
    const int gear = player.valid ? player.gear : v.gear;  // 0x422510 (the player's +0x1270), else the last one
    // The spark records not hit since the last frame are killed (vf02) and unlinked (0x5d1620); the others re-armed.
    for (size_t i = 0; i < v.sparks.size();) {
        if (!v.sparks[i].hit) {
            v.sparks[i].e->kill();
            dying_.push_back(std::move(v.sparks[i].e));
            v.sparks.erase(v.sparks.begin() + long(i));
        } else {
            v.sparks[i].hit = false;
            ++i;
        }
    }
    const float speed = len(in.velocity);
    const bool nearTop = nearTopSpeed(speed, in.topSpeed);
    bool burst = false;
    v.level = std::max(v.level - 2.0f * dt, 0.0f);
    if ((gear != v.gear && 0.4f < in.throttle) || nearTop) {  // asm 0x5cf499..0x5cf4cc
        v.gear = gear;
        v.level = 1.0f;
        burst = true;
    }
    // 0x5cf4d4: the level is overwritten (the decay and the 1.0 above are dead stores).
    v.level = flameLevel(in.flash, player.speedFrac, in.wheel0Load);
    // [nitro] (physics/nitro.hpp): a boosting vehicle's flame holds at least its nitro level and burns blue (FUEL's own
    // near-top-speed colour), with the near-top embers; 0 without a boost.
    const bool nitro = in.nitro > 0.05f;
    if (nitro) v.level = std::max(v.level, in.nitro);
    const size_t n = std::min(v.slots.size(), in.slotPos.size());
    const float sizeK = in.p52 * in.throttle * 1.5f + 1.0f;  // +0x6a0 x +0x1088 x 1.5 [0x995ee4] + 1
    const float strength = smokeAlpha(speed, pp_.alpha);
    for (size_t i = 0; i < n; ++i) {
        Slot& s = v.slots[i];
        // [nitro] the port's nitro outlets: only the boost's flame (no smoke, FUEL's level ignored)
        const bool nitroOnly = i >= in.nitroSlotsFrom;
        const float level = nitroOnly ? (nitro ? in.nitro : 0.0f) : v.level;
        // +0x1ad8[i]: start while [0x9fd061] (1) and visible and |v| < 15, else stop.
        if (in.visible && speed < 15.0f && !nitroOnly) s.smoke.start();
        else s.smoke.stop();
        // +0x1b18[i]: start while [0x9fd062] (1) and visible and level^2 >= 0.01 [0x99614c], else stop.
        if (in.visible && !(level * level < 0.01f)) s.flame.start();
        else s.flame.stop();
        const V3 pos = in.slotPos[i];
        const V3 d = normalized(in.slotDir[i]);
        if (!s.flame.stopped) {
            s.flame.length = level * level * 0.5f;  // +0x190 = level^2 x 0.5 [0x9960c0]
            float sz = level * level * 0.4f;  // 0x5d1930(0.4 [0x9dc034] level^2, same)
            if (nitro && (in.nitroLength != 1.0f || in.nitroSize != 1.0f))  // [upgrades U5] the nitrous power stage
                s.flame.length *= in.nitroLength, sz *= in.nitroSize;
            s.flame.size = sys_.rng().range(sz, sz);
            s.flame.dir = normalized(d);  // 0x5ce3b0
            s.flame.setPosition(pos);
            s.flame.blue = nearTop || nitro;  // +0x1a4 ([nitro]: blue while boosting)
        }
        if (!s.smoke.stopped) {
            sfx::Emitter& e = s.smoke;
            e.setPosition(pos);
            e.life.lo = pp_.lifeLo, e.life.hi = pp_.lifeHi;           // +0x10c / +0x110 (no redraw)
            e.size.lo = pp_.sizeLo * sizeK, e.size.hi = sizeK * pp_.sizeHi;  // +0x100 / +0x104
            e.colourA = {pp_.rgbA, pp_.rgbA, pp_.rgbA, strength};     // 0x5d1410: A = PP5325 grey, B = PP5326 grey
            e.colourB = {pp_.rgbB, pp_.rgbB, pp_.rgbB, strength};
            e.interval.lo = pp_.intervalLo, e.interval.hi = pp_.intervalHi;  // +0x78 / +0x7c
            e.rotSpeed.lo = -pp_.rot, e.rotSpeed.hi = pp_.rot;        // +0x154 / +0x158
            const V3 vel = add(mul(in.velocity, -0.12f), mul(d, pp_.push));  // -0.12 [0x9dc23c] x +0x4d0 + dir PP5332
            e.vel.lo = e.vel.hi = vel;
            e.accel.lo = e.accel.hi = mul(d, -pp_.drag);              // -dir PP5333
        }
        // The exhaust embers (UniqueEmitters +0x14): near top speed at random (16 r < 1 / slot count), else on the burst.
        bool ember = false;
        if (nearTop || nitro) ember = parkMiller() * 16.0f < 1.0f / float(n);  // [nitro] embers while boosting too
        else ember = burst && !nitroOnly;
        if (ember) {
            RibbonEmitter& e = embers_;
            e.setPosition(pos);
            // 0x4bd7b0: velocity range [1.5 d - 5 (1, 1, 1), 1.5 d + 5 (1, 1, 1)] (5 [0x9dbf8c] x DAT_00a06308), redrawn.
            const V3 c = mul(d, 1.5f);
            e.vel.lo = sub(c, {5, 5, 5}), e.vel.hi = add(c, {5, 5, 5});
            sfx::Well512& r = sys_.rng();
            for (int k = 0; k < 3; ++k) e.vel.cur[size_t(k)] = e.vel.lo[size_t(k)] + r.uniform() * (e.vel.hi[size_t(k)] - e.vel.lo[size_t(k)]);
            emitRibbon(e);  // vf08
            ++stats_.embersEmitted;
        }
    }
    // 0x5cfda0 (the damage smoke) is P85's (damage_view.mm).
}

void VehicleFx::update(float dt, std::vector<VehicleIn>& vehicles, const PlayerIn& player, const V3& eye) {
    // A vehicle that leaves (0x5d1060 teardown) takes its emitters with it; a new one in a slot is set up (0x5d00c0).
    if (veh_.size() != vehicles.size()) veh_.resize(vehicles.size());
    const int embers = stats_.embersEmitted, calls = stats_.sparkCalls;  // running totals
    stats_ = Stats{};
    stats_.embersEmitted = embers, stats_.sparkCalls = calls;
    for (size_t i = 0; i < vehicles.size(); ++i) {
        Vehicle& v = veh_[i];
        if (!v.init || v.id != vehicles[i].id) {
            v = Vehicle{};
            setupVehicle(v, vehicles[i].slotPos.size());
            v.id = vehicles[i].id;
        }
        for (const SparkHit& h : vehicles[i].hits) sparkCall(v, vehicles[i], h);
    }
    for (size_t i = 0; i < vehicles.size(); ++i) exhaust(veh_[i], vehicles[i], player, dt);
    // The FX manager's step (0x5f62c0): every emitter's vf06.
    const std::array<float, 4> flameRect = [&] {
        const auto& at = sys_.tables().atlas;
        if (at.size() > 0x30) return std::array<float, 4>{at[0x30].u0, at[0x30].v0, at[0x30].du, at[0x30].dv};
        return std::array<float, 4>{0, 0, 1, 1};
    }();
    for (Vehicle& v : veh_) {
        for (Slot& s : v.slots) {
            sys_.runEmitter(s.smoke, dt, eye);  // 0x4c53b0
            s.flame.update(dt, flameRect);      // 0x5edb20
            stats_.smokeLive += int(s.smoke.live.size());
            stats_.flamesOn += s.flame.stopped ? 0 : 1;
            ++stats_.slots;
        }
        for (SparkRecord& s : v.sparks) {
            runRibbon(*s.e, dt, eye);
            stats_.sparkLive += int(s.e->live.size());
            ++stats_.sparkRecords;
        }
    }
    for (size_t i = 0; i < dying_.size();) {
        runRibbon(*dying_[i], dt, eye);
        stats_.sparkLive += int(dying_[i]->live.size());
        if (dying_[i]->live.empty()) dying_.erase(dying_.begin() + long(i));  // dead (+6) -> deleted by the manager
        else ++i;
    }
    runRibbon(embers_, dt, eye);
    stats_.emberLive = int(embers_.live.size());
}

void VehicleFx::collect(std::vector<SmokeSprite>& smoke, std::vector<FlameQuad>& flames, std::vector<Streak>& streaks, const V3& eye) const {
    smoke.clear(), flames.clear(), streaks.clear();
    for (const Vehicle& v : veh_)
        for (const Slot& s : v.slots) {
            for (const sfx::Particle& p : s.smoke.live) {
                if (p.age <= 0) continue;
                SmokeSprite q;
                q.pos = p.pos, q.angle = p.angle, q.size = p.drawSize;
                q.uv = {p.uv[0], p.uv[1], p.uv[2], p.uv[3]};
                q.rgba = {p.colour[0], p.colour[1], p.colour[2], p.drawAlpha};
                smoke.push_back(q);
            }
            if (!s.flame.stopped && s.flame.haveQuads)
                for (const FlameEmitter::Quad& fq : s.flame.quads) flames.push_back({fq.v, fq.uv, s.flame.colour});
        }
    // +0x6c & 8: the smoke emitters sort their particles far to near (0x4c49f0); the port sorts them all.
    std::sort(smoke.begin(), smoke.end(), [&](const SmokeSprite& a, const SmokeSprite& b) {
        return dot(sub(a.pos, eye), sub(a.pos, eye)) > dot(sub(b.pos, eye), sub(b.pos, eye));
    });
    auto ribbons = [&](const RibbonEmitter& e) {
        for (const RibbonParticle& p : e.live) {
            if (p.age <= 0 || p.drawAlpha <= 0) continue;
            Streak s;
            s.rope = p.rope;
            s.halfWidth = p.drawSize;
            s.rgba = {p.colour[0], p.colour[1], p.colour[2], p.drawAlpha};
            s.atlas = e.atlas, s.kind = e.kind;
            streaks.push_back(s);
        }
    };
    for (const Vehicle& v : veh_)
        for (const SparkRecord& s : v.sparks) ribbons(*s.e);
    for (const auto& e : dying_) ribbons(*e);
    ribbons(embers_);
}

}  // namespace fuel::vfx
