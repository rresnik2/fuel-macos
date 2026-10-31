#include "fuel_camera.hpp"

#include "core/camera_params.hpp"

#include <algorithm>
#include <cmath>

namespace fuel::cam {

using fv::clampf;
using fv::cross;
using fv::dot;
using fv::len;
using fv::len2;
using fv::normalize;
using fv::rotate;

namespace {

constexpr float kDeg = 0.017453292f;
const Vec3 kUp{0, 1, 0};

Quat slerpIdentity(const Quat& q, float t) {  // 0x644e00 (I): the rotation q scaled to t of its angle
    Quat a = q;
    if (a.w < 0) a = {-a.x, -a.y, -a.z, -a.w};
    const float s = std::sqrt(a.x * a.x + a.y * a.y + a.z * a.z);
    if (s < 1e-7f) return {};
    const float ang = 2.0f * std::atan2(s, a.w) * t;
    const float k = std::sin(ang * 0.5f) / s;
    return {a.x * k, a.y * k, a.z * k, std::cos(ang * 0.5f)};
}

float sgn(float x) { return x < 0 ? -1.0f : 1.0f; }

}  // namespace

float smooth1(float cur, float target, float rate, float dt) {
    const float d = target - cur;
    if (std::fabs(d) < 1e-6f) return target;
    return float(double(cur) + double(d) * (1.0 - 1.0 / std::pow(double(rate), double(dt) * 30.30303)));
}

Vec3 smoothV(const Vec3& cur, const Vec3& target, float rate, float dt) {
    const Vec3 d = target - cur;
    if (len2(d) < 1e-6f) return target;
    const float k = float(1.0 - 1.0 / std::pow(double(rate), double(dt) * 30.30303));
    return cur + d * k;
}

Quat quatFromTo(const Vec3& a, const Vec3& b) {
    const float c = dot(a, b);
    if (c < -0.999999f) {  // opposite: any perpendicular axis
        Vec3 ax = cross({1, 0, 0}, a);
        if (len2(ax) < 1e-6f) ax = cross({0, 1, 0}, a);
        ax = normalize(ax);
        return {ax.x, ax.y, ax.z, 0};
    }
    const Vec3 x = cross(a, b);
    return fv::qnormalize({x.x, x.y, x.z, 1.0f + c});
}

Vec3 removeAgainst(const Vec3& step, const Vec3& N) {
    if (!(len2(N) > 1e-6f)) return step;
    const Vec3 n = normalize(N);
    const float d = dot(step, n);
    return d < 0 ? step - n * d : step;
}

Vec3 flatNormalize(const Vec3& v, const Vec3& fallback) {
    const float l2 = v.x * v.x + v.z * v.z;
    if (l2 <= 1e-6f) return fallback;
    const float l = std::sqrt(l2);
    return {v.x / l, 0, v.z / l};
}

// The per-frame context of 0x402930 (A4); offsets in the comments.
struct ChaseCamera::Ctx {
    CameraParams k;
    float scale = 1.5f, scale2 = 0.75f;              // +0x18, +0x1c
    float orbitRate = 2.0f, liftRate = 1.05f;        // +0x20, +0x24
    float extraRate = 0;                             // +0x28
    float D0 = 0;                                    // +0x30
    float pushR = 0.8f, pivotF = 0.5f;               // +0x34, +0x38
    float lift = 0, liftFwd = 0;                     // +0x3c, +0x40
    float orbitSpeed = 15.0f, yawDiv = 0.33f;        // +0x44, +0x48
    float stiff = 0, pitch = 0, fovTgt = 60;         // +0x4c, +0x54, +0x58
    float roll = 0;                                  // +0x8c
    Vec3 Dd, F, side, vel, angVel, fdVel, C, up;     // +0x68, +0xa8, +0xb4, +0xc0, +0xcc, +0xd8, +0xe4, +0xf0
    Vec3 Fxz, Sxz, Fu, Su, Fuxz, Suxz;               // +0x114, +0x120, +0x12c, +0x138, ...
    Quat yawDelta;                                   // +0x74
    double ox = 0, oy = 0, oz = 0;                   // +0xfc origin (world)
    float speed = 0;
    bool snap = false, flipFrame = false;
    int view = 5;
    const VehicleInput* veh = nullptr;
    const WorldQueries* world = nullptr;
    // ground relative to the origin (C3), detail flag
    float groundRel(const Vec3& p, bool detail) const {
        if (!world || !world->ground) return -1e9f;
        return float(double(world->ground(ox + p.x, oy + p.y, oz + p.z, detail)) - oy);
    }
};

ChaseCamera::ChaseCamera() = default;

float ChaseCamera::rand01() {  // 0x6460f0: Park-Miller 16807 (Schrage) with a 32-entry Bays-Durham shuffle
    auto next = [](uint32_t& s) {
        const int32_t k = int32_t(s) / 127773;
        int32_t v = 16807 * (int32_t(s) - k * 127773) - 2836 * k;
        if (v < 0) v += 2147483647;
        s = uint32_t(v);
    };
    uint32_t& seed = rng_[32];
    if (!rngInit_) {
        rngInit_ = true;
        seed = 1;
        for (int j = 32 + 7; j >= 0; --j) {
            next(seed);
            if (j < 32) rng_[j] = seed;
        }
        rng_[31] = rng_[0];
    }
    const uint32_t y = rng_[31];
    next(seed);
    const int j = int(y / (1 + (2147483647 - 1) / 32)) & 31;
    rng_[31] = rng_[j];
    rng_[j] = seed;
    return std::min(float(rng_[31]) * 4.656613e-10f, 0.9999999f);
}

// ---- 0x404040 lift (emulator-matched) ----
Vec3 ChaseCamera::liftOffset(const Vec3& up, const Vec3& flatSide, const Vec3& F, float ctx3c, float ctx40, float scale) {
    const Vec3 U = normalize(up - flatSide * dot(up, flatSide));
    return U * (ctx3c * scale) + F * (scale * ctx40);
}

// ---- 0x409790 snap offset ----
Vec3 ChaseCamera::snapOffset(const Vec3& F, float pitch, float D) {
    const Quat q = fv::axisAngle(normalize(cross(F, kUp)), pitch);
    return rotate(q, normalize(-F) * D);
}

// ---- 0x404c90 manual orbit ----
Vec3 ChaseCamera::orbit(const Vec3& Oin, float stickX, float stickY, float speed, float ctx30, float dt, bool snap) {
    Vec3 O = Oin;
    if (snap) {
        f1982_ = false;
        yaw_ = pitch_ = 0;
        N_ = {};
        for (float& t : timer1930_) t = 0;
        prevO_ = O;
        return O;
    }
    float yawT = -stickX, pitchT = -stickY;
    if (yawT * yawT + pitchT * pitchT > 0.05f) s1940_ = 0;
    // contact timers: ceiling blocks pitch < 0, ground blocks pitch > 0 (+0x1938/+0x193c likewise for yaw)
    if (timer1930_[0] > 0 && pitchT <= 0) pitchT = 0;
    if (timer1930_[1] > 0 && pitchT >= 0) pitchT = 0;
    if (timer1930_[2] > 0 && yawT <= 0) yawT = 0;
    if (timer1930_[3] > 0 && yawT >= 0) yawT = 0;
    for (float& t : timer1930_) t -= dt;
    yaw_ = smooth1(yaw_, yawT, 2.0f, dt);
    pitch_ = smooth1(pitch_, pitchT, 2.0f, dt);
    if (f1982_) yaw_ = pitch_ = 0;
    const float L = len(O);
    if (L > 1e-6f) {
        const float K = 15.0f * dt * L / ctx30;
        const Vec3 n = O / L;
        // both steps from the incoming O, summed, then filtered once by the contact normal (asm 0x404ff5..0x4051d6)
        Vec3 step = normalize(cross(O, kUp)) * (std::cos(std::fabs(n.y)) * yaw_ * K);
        if ((pitch_ > 0 && n.y > -0.9f) || (pitch_ < 0 && n.y < 0.9f)) step = step + normalize(kUp * len2(O) - O * dot(O, kUp)) * (pitch_ * K);
        O = O - removeAgainst(step, N_);
    }
    if (std::fabs(yaw_) > 0.01f || std::fabs(pitch_) > 0.01f) {
        af18fc_ = 0;
        pause18dc_ = 2.9e12f;
    }
    if (pause18dc_ > 0 && speed > 1) pause18dc_ = std::min(pause18dc_, 1.0f);
    if (pause18dc_ <= 0)
        af18e8_ = af18f0_ = af18fc_ = 1;
    else
        af18e8_ = af18f0_ = 0;
    pause18dc_ -= dt;
    return O;
}

// ---- 0x405440 follow spring ----
Vec3 ChaseCamera::followSpring(const Vec3& O, const Vec3& Dd, float stiff, float dt) {
    const float L = len(O);
    if (L < 1e-6f) return O;
    const Vec3 A = Dd * L, E = A - O, n = O / L;
    Vec3 Tg = (E - n * dot(E, n)) * 2.0f;
    const float lim = 1.5f * 5.0f;
    if (len(Tg) > lim) Tg = Tg * (lim / len(Tg));
    if (dot(A, O) < 0) {
        const Vec3 c = normalize(cross(A, kUp));
        const float s = dot(c, O) > 0 ? 1.0f : -1.0f;
        Tg = {Tg.x + 1.5f * s * c.x, 1.5f * s * c.y, Tg.z + 1.5f * s * c.z};
        Tg = Tg - n * dot(Tg, n);
        const float l = len(Tg);
        if (l > 1e-6f && l < lim) Tg = Tg * (lim / l);
    }
    const float tg = af18f0_ * stiff;
    g_ = tg <= g_ ? tg : smooth1(g_, tg, 1.1f, dt);
    if (len(Dd) * g_ > 0.01f) af18fc_ = 0;
    const Vec3 V = normalize(cross(cross(O, kUp), O));
    const float sv = dot(Tg, V);
    return O + (Tg - V * sv) * (5.0f * dt * g_) + V * (1.0f * sv * g_ * dt);
}

// ---- 0x4091e0 lag ----
Vec3 ChaseCamera::lag(const Vec3& O, const Vec3& vel, float pitch, const Vec3& side, float dt, bool snap, float ctx48) {
    g2_ = smooth1(g2_, ctx48 * af18e8_, 1.05f, dt);
    const Quat q = fv::axisAngle(normalize(cross(kUp, O)), pitch);
    const float ctx10 = 0;
    if (snap) {
        f197d_ = true;
        s1940_ = 0;
    } else if (pitch == 0) {
        f197d_ = true;
    } else if (af18e8_ < 0.5f && ctx10 < 0.01f) {
        f197d_ = false;
    }
    const float lv = len(vel);
    const Vec3 vv = lv > 1e-6f ? vel * (std::max(lv - 0.1f, 0.0f) / lv) : Vec3{};
    Vec3 lg = Vec3{vv.x, 0, vv.z} * (dt * g2_);
    const float d = dot(Vec3{vv.x, 0, vv.z}, O);
    const Vec3 n = normalize(O);
    if (d < 0 && pitch != 0 && f197d_ && ctx10 < 0.01f && std::hypot(lg.x, lg.z) < 0.1f)
        lg = lg + normalize(Vec3{-O.x, 0, -O.z}) * (2.0f * dt * g2_);
    if (!(d > 0)) {
        Vec3 r = rotate(q, lg);
        r = r - n * dot(r, n);
        return O - r;
    }
    const float hl = std::hypot(lg.x, lg.z);
    const Vec3 p = lg - n * dot(lg, n);
    const float hp = std::hypot(p.x, p.z);
    return O - (hp > 1e-4f ? Vec3{p.x / hp * hl, 0, p.z / hp * hl} : side * hl);
}

// ---- 0x405800 distance ----
Vec3 ChaseCamera::distance(const Vec3& Oin, float D, float speed, bool snap) {
    Vec3 O = Oin;
    if (snap) lock_ = false;
    const float L = len(O);
    if (L > 0) {
        const float hxz = std::hypot(O.x, O.z);
        const float Deff = (O.y < 0 && hxz > 0) ? D * L / hxz : D;
        if (lock_ || L > Deff) O = O + removeAgainst(O * (Deff / L - 1.0f), N_);
    }
    lock_ = speed > 1.5f || std::fabs(yaw_) + std::fabs(pitch_) > 0.01f;
    return O;
}

// ---- 0x408d30 first-person head spring ----
void ChaseCamera::headSpring(const Vec3& U, const Vec3& F, Vec3& eye, Vec3& target, float dt, bool snap, float& roll, float k10) {
    const Vec3 rel = U * 0.33f;
    const Vec3 pivot = eye - rel;
    if (snap) head_ = {rel, {}, {}};
    head_.v = head_.v + (rel - head_.prev) * 10.0f;
    head_.prev = rel;
    head_.v = head_.v - head_.p * (50.0f * dt);
    head_.v = head_.v - head_.v * (5.0f * dt);
    head_.p = head_.p + head_.v * dt;
    const Vec3 nRel = normalize(rel), nNew = normalize(rel + head_.p * k10);
    const Quat q = quatFromTo(nRel, nNew);
    eye = pivot + rotate(q, eye - pivot);
    target = pivot + rotate(q, target - pivot);
    roll += std::asin(clampf(dot(cross(nNew, nRel), F), -1, 1));
}

// ---- 0x402930 context (A3, A4) ----
void ChaseCamera::buildContext(Ctx& c, float dt, const VehicleInput& veh, const WorldQueries& world, const CameraControls& in, Override ov) {
    c.veh = &veh;
    c.world = &world;
    // view selection (A3)
    if (in.viewChange) {
        static const int kCycle[3] = {1, 4, 5};
        int idx = 0;
        for (int i = 0; i < 3; ++i)
            if (kCycle[i] == choice_) idx = i;
        int next = kCycle[(idx + 1) % 3];
        if (next > 6) next = 1;
        choice_ = next;
    }
    int ovr = int(ov);
    if (veh.wreck != 0) ovr = veh.wreck == 2 ? 11 : 10;  // 0x421670 precedence
    if (ovr != override_) tView_ = 0;
    override_ = ovr;
    cur_ = override_ ? override_ : choice_;
    if (cur_ != prevView_) {
        tView_ = 0;
        snap_ = true;
        prevView_ = cur_;
    }
    tView_ += dt;
    c.view = cur_;
    c.snap = snap_;
    c.k = cameraParams(*veh.P, cur_);
    const CameraParams& k = c.k;

    c.ox = veh.ox, c.oy = veh.oy, c.oz = veh.oz;
    c.D0 = k[0] + k[1] - 0.33f;
    c.lift = k[3] + 1.0f;
    c.vel = veh.vel;
    if (len(c.vel) > 300.0f) c.vel = {}, teleport_ = true;
    c.speed = len(c.vel);
    c.angVel = veh.angVel;
    c.fdVel = veh.fdVel;
    c.stiff = (veh.sliding ? k[12] : k[11]) * clampf((c.speed - 2.0f) * 0.1f, 0, 1);
    c.F = rotate(veh.q, {0, 0, 1});
    c.side = rotate(veh.q, {1, 0, 0});
    c.up = rotate(veh.q, {0, 1, 0});
    c.C = c.F * k[1];
    c.Fu = c.F, c.Su = c.side;
    c.Fuxz = flatNormalize(c.F, {0, 0, 1}), c.Suxz = flatNormalize(c.side, {1, 0, 0});
    // reverse flip (+0x18b4)
    if (c.snap) forward_ = 1;
    c.flipFrame = false;
    if (cur_ >= 4) {
        if (forward_ == 1 && veh.gear < 0 && dot(c.F, c.vel) < 0) forward_ = 0, c.flipFrame = true;
        else if (forward_ == 0 && veh.gear > 0 && dot(c.F, c.vel) > 0) forward_ = 1, c.flipFrame = true;
        if (forward_ == 0) c.F = -c.F, c.side = -c.side;
    } else {
        forward_ = 0;
    }
    c.Fxz = flatNormalize(c.F, c.Fuxz);
    c.Sxz = flatNormalize(c.side, c.Suxz);
    c.yawDelta = c.flipFrame || c.snap ? Quat{} : quatFromTo(prevFlatF_, c.Fxz);
    prevFlatF_ = c.Fxz;
    // slope look-ahead (views >= 4, not on the flip frame): ground 15/30/60 m ahead, no detail
    if (cur_ >= 4 && !c.flipFrame && world.ground) {
        const Vec3 P0 = c.C + Vec3{0, 1, 0};
        const float h0 = c.groundRel(P0, false);
        float s = 0;
        for (float d : {15.0f, 30.0f, 60.0f}) s += (c.groundRel(P0 + c.F * d, false) - h0) / d;
        const float slope = std::max(0.333f * s, -2.0f);
        c.F = normalize(c.F + normalize(Vec3{c.Fxz.x, slope, c.Fxz.z}));
    }
    c.pitch = clampf(std::asin(clampf(c.F.y, -1, 1)) + k[5] * kDeg, -1, 1);
    const float s = clampf(c.speed * 0.03f, 0, 1);
    c.fovTgt = k[6] * (1 - s) + k[7] * s;
    if (veh.boost) c.fovTgt = fv::lerpf(c.fovTgt, k[8], clampf(c.speed * 0.1f, 0, 1));
    c.Dd = normalize(-c.F);
}

// ---- 0x404b10 look target ----
Vec3 ChaseCamera::lookTarget(const Ctx& c, float dt) {
    const Vec3 a = rotate(c.veh->q, c.veh->targetNode);
    const Vec3 L = liftOffset(c.up, c.Sxz, c.F, c.lift, c.liftFwd, c.scale);
    liftS_ = c.snap ? L : smoothV(liftS_, L, c.liftRate, dt);
    return a + liftS_ + c.Fu * c.k[1] + Vec3{c.up.x, 0, c.up.z} * (-c.k[4] * clampf(c.up.y, 0, 1));
}

// ---- 0x406490 sphere push-out (B C1) ----
Vec3 ChaseCamera::pushOut(Ctx& c, const Vec3& B, const Vec3& O, float dt, const WorldQueries& world) {
    if (!world.overlap) return O;
    const float r = c.pushR * c.scale;  // 1.2 m
    const Vec3 P = B + O;               // relative to the origin
    if (c.snap) {
        Q_ = P;
        probeT_ = 100;
        f1981_ = false;
    }
    const Vec3 delta = P - Q_;
    int n = 1;
    if (!f1981_ && len(delta) < 10 * r) n = std::min(int(len(delta) / (0.5f * r)) + 1, 5);
    std::vector<WorldQueries::Contact> hits;
    bool pushed = false;
    Vec3 acc;
    for (int s = 0; s < n; ++s) {
        Q_ = Q_ + delta / float(n);
        for (int pass = 0; pass < 10; ++pass) {
            hits.clear();
            world.overlap(c.ox + Q_.x, c.oy + Q_.y, c.oz + Q_.z, r, hits);
            Vec3 sum;
            for (const auto& h : hits) {
                if (!(dot(h.p, h.n) < 0)) continue;
                const Vec3 u = normalize(h.p);
                const float pen = r - h.d;
                if (pen <= 0) continue;
                const Vec3 push = u * (dot(u, h.n) * pen);
                Q_ = Q_ + push;  // push points out of the object (dot(u, m) < 0)
                sum = sum + push;
            }
            if (len2(sum) > 0) pushed = true, acc = acc + sum;
            if (len(sum) <= 0.02f) break;
            if (pass == 9) teleport_ = true;
        }
    }
    // contact normal for the orbit/distance filters, re-probed every 0.05 s with 1.1 r
    probeT_ += dt;
    if (probeT_ >= 0.05f) {
        probeT_ = 0;
        hits.clear();
        world.overlap(c.ox + Q_.x, c.oy + Q_.y, c.oz + Q_.z, 1.1f * r, hits);
        acc = {};
        for (const auto& h : hits)
            if (dot(h.p, h.n) < 0 && 1.1f * r - h.d > 0) acc = acc + h.n * (1.1f * r - h.d);
        accN_ = len2(acc) > 0 ? normalize(acc * 10000.0f) : Vec3{};
    }
    N_ = len(accN_) <= 0.1f ? accN_ : smoothV(N_, accN_, 1.5f, dt);
    if (dot(accN_, Vec3{0, -1, 0}) > 0.707f) timer1930_[0] = 0.33f;
    if (dot(accN_, kUp) > 0.707f) timer1930_[1] = 0.33f;
    if (len2(accN_) > 0 && (dot(normalize(O), accN_) > 0.33f || dot(accN_, kUp) > 0.707f)) f197f_ = true;
    pushed_ = pushed;
    return pushed ? Q_ - B : O;
}

// ---- 0x409870 offset ----
Vec3 ChaseCamera::offset(Ctx& c, const Vec3& T, float dt, const WorldQueries& world) {
    const Vec3 B = T + Vec3{0, c.scale * c.pivotF, 0};
    const float D = c.D0 * c.scale;
    Vec3 O = O_;
    if (c.snap) {
        O = snapOffset(c.F, c.pitch, D);
        g2_ = af18e8_ = g_ = af18f0_ = af18f4_ = af18f8_ = 1.0f;
    }
    O = orbit(O, orbitStickX_, orbitStickY_, c.speed, c.D0, dt, c.snap);
    O = followSpring(O, c.Dd, c.stiff, dt);
    O = lag(O, c.vel, c.pitch, c.Su, dt, c.snap, c.yawDiv);
    {  // 0x409170 yaw carry
        const float t = clampf(1.0f - g2_ / c.yawDiv, 0, 1);
        O = rotate(slerpIdentity(c.yawDelta, t), O);
    }
    O = distance(O, D, c.speed, c.snap);
    if (std::hypot(O.x, O.z) > 3 * D || len(O) < 0.25f * D) teleport_ = true;
    {  // 0x409630 elevation clamp
        minElev_ = -1.0f;
        const float hxz = std::hypot(O.x, O.z);
        const float el = std::atan2(O.y, hxz), yw = std::atan2(-O.z, O.x);
        if (el > 0.95f || el < minElev_) {
            const float e = clampf(el, minElev_, 0.95f), L = len(O);
            O = {L * std::cos(e) * std::cos(yw), L * std::sin(e), -L * std::cos(e) * std::sin(yw)};
        }
    }
    O = pushOut(c, B, O, dt, world);
    {  // 0x405a20 max length
        const float L = len(O);
        if (L > 2 * D) O = O * (D / L);
        if (len(O) > 1.25f * D) timer1930_[1] = 0.5f;
        f1982_ = false;
    }
    // ground: O.y >= ground(T + O) - T.y + 0.5 (origin-relative heights)
    if (world.ground) O.y = std::max(O.y, c.groundRel(T + O, true) - T.y + 0.5f);
    O_ = O;
    // 0x410d10 zone constraint: the level's CameraEngineZone polygons are not loaded (approximation): none apply
    return O + Vec3{0, 0.75f, 0};
}

// ---- 0x404290 sway (B A) ----
void ChaseCamera::sway(Ctx& c, const Vec3& O, Vec3& T, float dt) {
    if (c.snap) s195c_ = s1960_ = s1964_ = s1968_ = s196c_ = s1970_ = 0;
    const CameraParams& k = c.k;  // k13 = id 419 ...
    auto rate = [](float state, float input, float rise, float fall) { return 1.0f + (std::fabs(state) < std::fabs(input) ? rise : fall); };
    const Vec3 V = c.fdVel, Fw = c.Fu, Sd = c.Su, W = c.angVel;
    Vec3 S = normalize(Vec3{-O.z, 0, O.x});
    if (len2(S) < 1e-12f) S = {1, 0, 0};
    Vec3 D;
    const float Lr = -dot(V, Sd);
    s1960_ = smooth1(s1960_, Lr, rate(s1960_, Lr, k[13], k[14]), dt);
    c.roll += k[15] * s1960_;
    const float Fv = dot(Fw, V);
    s1970_ = smooth1(s1970_, Fv, rate(s1970_, Fv, k[16], k[17]), dt);
    D = D + Fw * (s1970_ * (s1970_ > 0 ? k[18] : k[19]));
    const float Vy = V.y;
    s196c_ = smooth1(s196c_, Vy, rate(s196c_, Vy, k[20], k[21]), dt);
    D = D + kUp * (s196c_ * (s196c_ > 0 ? k[22] : k[23]));
    const float w = clampf((c.speed - 5.0f) * 0.05f, 0, 1);
    s195c_ = smooth1(s195c_, W.y, 1.1f, dt);
    const float x = s195c_ * w * 500.0f;
    const float Y2 = sgn(x) * std::sqrt(std::fabs(x));
    s1968_ = smooth1(s1968_, Y2, 1.0f + 0.5f * (std::fabs(s1968_) < std::fabs(Y2) ? k[24] : k[25]), dt);
    D = D + S * (s1968_ * k[26]);
    const float Pp = -dot(V, Fw);
    s1964_ = smooth1(s1964_, Pp, rate(s1964_, Pp, k[27], k[28]), dt);
    const float a = s1964_ * (s1964_ > 0 ? k[29] : k[30]);
    if (c.veh->speedFrac >= 0.95f) signToggle_ = -signToggle_;
    if (len(D) > 1e-4f) T = T + D;
    T = rotate(fv::axisAngle(S, a), T);
}

// ---- 0x408940 rough-surface shake (B B) ----
void ChaseCamera::roughShake(const Ctx& c, Vec3& pos, Vec3& target, float dt) {
    if (c.snap) {
        shakeT_ = 0.2f;
        shakeV_ = shakeOff_ = {};
    }
    const float f = c.veh->maxBumps;
    if (!c.veh->anyWheelLoaded || !(f > 0)) return;
    const float A = 1.5f * std::sqrt(std::max(c.speed - 5.0f, 0.0f));
    shakeT_ -= dt;
    if (shakeT_ < 0) {
        shakeT_ += (rand01() + 1.0f) * 0.1f;
        const float r1 = rand01(), r2 = rand01(), r3 = rand01();
        Vec3 kk = Vec3{r1 - 0.5f, r2 - 0.5f, r3 - 0.5f} * (A * f);
        const Vec3 d = normalize(target - pos);
        kk = kk - d * dot(kk, d);
        if (kk.x * shakeV_.x > 0) kk.x = -kk.x;
        if (kk.y * shakeV_.y > 0) kk.y = -kk.y;
        if (kk.z * shakeV_.z > 0) kk.z = -kk.z;
        shakeV_ = shakeV_ + kk;
    }
    shakeV_ = shakeV_ - shakeOff_ * (50.0f * dt);
    shakeV_ = shakeV_ - shakeV_ * (5.0f * dt);
    shakeOff_ = shakeOff_ + shakeV_ * dt;
    pos = pos - shakeOff_ * 0.5f;
    target = target + shakeOff_ * 0.5f;
}

// ---- 0x412e80 / 0x40a160 ----
CameraPose ChaseCamera::update(float dt, const VehicleInput& veh, const WorldQueries& world, const CameraControls& in, Override ov) {
    CameraPose out;
    if (!veh.valid || !veh.P || dt <= 0) return out;
    // teleport/reset: the breadcrumb push sets +0x138 when the vehicle moved > 15 m (A5 step 2)
    if (haveLast_ && std::hypot(std::hypot(veh.ox - lastOx_, veh.oy - lastOy_), veh.oz - lastOz_) > 15.0) snap_ = true;
    lastOx_ = veh.ox, lastOy_ = veh.oy, lastOz_ = veh.oz, haveLast_ = true;
    if (teleport_) snap_ = true, teleport_ = false;

    Ctx c;
    buildContext(c, dt, veh, world, in, ov);
    const int v = c.view;
    // look stick for the orbit
    orbitStickX_ = in.stickX, orbitStickY_ = in.stickY;

    Vec3 T = lookTarget(c, dt);
    Vec3 Orel = offset(c, T, dt, world);
    sway(c, Orel, T, dt);
    float blur = 0;
    {
        const float lo = c.veh->P->size() > 574 ? (*c.veh->P)[572] : 0, hi = c.veh->P->size() > 574 ? (*c.veh->P)[573] : 1;
        const float mx = c.veh->P->size() > 574 ? (*c.veh->P)[574] : 0;
        blur = hi > lo ? mx * clampf((c.speed - lo) / (hi - lo), 0, 1) : 0;
    }
    if (Orel.y < 0) {
        const Vec3 f = flatNormalize(Orel, {0, 0, 0});
        const float L = len(Orel);
        Orel = {f.x * L, Orel.y, f.z * L};
    }
    const Vec3 P = T + Orel;
    // 0x409d10 final smoothing (snap only for views >= 4)
    const bool snap2 = v >= 4 ? c.snap : false;
    if (snap2 || c.snap) {
        rel_ = P - T;
        f197f_ = false;
    } else {
        rel_ = f197f_ ? P - T : smoothV(rel_, P - T, 2.0f, dt);
        f197f_ = false;
    }
    tgt_ = T;
    pos_ = T + rel_;
    // A9 ground term (as read: x saturates at -0.1 for terrain above 0 m) and landing dip
    float hRel = -1e9f;
    if (world.ground) {
        const float h = world.ground(c.ox + pos_.x, c.oy + pos_.y, c.oz + pos_.z, true);
        hRel = float(double(h) - c.oy);
        groundG_ = smooth1(groundG_, float(c.oy + pos_.y) - h, 1.025f, dt);
        const float x = clampf(groundG_ - h, -0.1f, 0.1f);
        pos_.y += c.k[10] * x;
        tgt_.y += c.k[10] * x;
    }
    {
        const std::vector<float>& Pp = *veh.P;
        auto p = [&](int n) { return size_t(n) < Pp.size() ? Pp[size_t(n)] : 0.0f; };
        if (veh.airtime > 0.1f) {
            maxH_ = std::max(maxH_, veh.heightAbove);
            air_ = true;
        } else if (air_) {
            air_ = false;
            dip_ = true;
            dipD_ = 0;
            dipA_ = clampf(maxH_ * p(campp::kLandHeight), 0, 1) * p(campp::kLandAmp);
        }
        if (dip_) {
            if (dipD_ >= 2.25f * dipA_) {
                dip_ = false, dipD_ = 0, maxH_ = 0;
            } else {
                const float step = p(campp::kLandRate) * dt * dipA_;
                const float base = tgt_.y;
                if (dipD_ < dipA_) {
                    dipD_ += step;
                    tgt_.y = base + dipD_;
                } else {
                    dipD_ += 0.5f * step;
                    tgt_.y = base + 2 * dipA_ - dipD_;
                }
            }
        }
    }
    float nearCand = 0.25f;
    const Vec3 Uw{0, 1, 0};
    if (v >= 1 && v <= 3) {  // first person (B D)
        const CameraParams& k = c.k;
        const Vec3 F = c.Fu, Sd = c.Su, U = c.up;
        Vec3 eye = F * k[1] + F * k[0] + Sd * k[2] + U * k[3];
        Vec3 target = eye + F * 100.0f;
        const Vec3 R = flatNormalize(Sd, {1, 0, 0});
        const Vec3 cc = normalize(cross(F, Sd));
        const float a = dot(R, cc), b = cc.y;
        c.roll += std::atan2(sgn(b) * a, b);
        headSpring(U, F, eye, target, dt, c.snap, c.roll, k[10]);
        pos_ = eye, tgt_ = target;
    } else if (v == 8) {  // race intro: three shots
        const Vec3 C = c.C, F = c.Fu, S = c.Su;
        const float t = tView_;
        auto sm = [](float x) { return x * (2 - std::fabs(x)) / 2 + 0.5f; };
        const Vec3 P1 = Uw + C + F * 50.0f - S * (5 - 3 * t), P2 = Uw * 3.5f + C - F * 10.0f - S * (5 - 3 * (t - 1)),
                   P3 = Uw * 0.5f + C + F * 5.0f + S * (5 - 3 * (t - 2));
        tgt_ = Uw * 0.5f + C + F * 2.0f;
        float fov = 30;
        blur = 0.05f;
        if (t <= 0.8f) pos_ = P1, fov = 30;
        else if (t < 1.0f) {
            const float s = sm(clampf(10 * (t - 0.8f) - 1, -1, 1));
            pos_ = P1 + (P2 - P1) * s, fov = 30 * (1 - s) + 40 * s, blur = 0.05f + 0.1f * std::sin(3.14f * s);
        } else if (t <= 1.8f) pos_ = P2, fov = 40;
        else if (t < 2.0f) {
            const float s = sm(clampf(10 * (t - 1.8f) - 1, -1, 1));
            pos_ = P2 + (P3 - P2) * s, fov = 40 * (1 - s) + 50 * s, blur = 0.05f + 0.1f * std::sin(3.14f * s);
        } else pos_ = P3, fov = 50;
        c.roll = -0.3f;
        c.snap = true;  // FOV written straight (the end-of-frame smoothing is skipped while snapping)
        c.fovTgt = fov;
    } else if (v == 10 || v == 11) {  // wreck / drown: slow orbit from a fixed world point
        const int w = v - 10;
        wreckT_[w] += dt;
        const float t = wreckT_[w];
        if (c.snap) {
            const Vec3 p = c.C + c.up * 1.5f + (c.Fuxz * -std::fabs(std::cos(0.1f * t)) + c.Suxz * std::sin(0.1f * t)) * 10.0f;
            wreckPos_[w][0] = c.ox + p.x, wreckPos_[w][1] = c.oy + p.y, wreckPos_[w][2] = c.oz + p.z;
            if (v == 11 && world.ground)  // (I) above the water/ground
                wreckPos_[w][1] = std::max(wreckPos_[w][1], double(world.ground(wreckPos_[w][0], wreckPos_[w][1], wreckPos_[w][2], true)) + 1.0);
        }
        pos_ = {float(wreckPos_[w][0] - c.ox), float(wreckPos_[w][1] - c.oy), float(wreckPos_[w][2] - c.oz)};
        tgt_ = c.C + c.up * 0.5f + c.Fu * 0.5f;
        c.roll = 0.25f * std::cos(0.133f * t);
        c.fovTgt = std::min(200.0f / std::max(len(tgt_ - pos_), 1e-3f), 90.0f);
    } else if (v == 13) {  // page camera (absolute)
        pos_ = {float(page_[0] - c.ox), float(page_[1] - c.oy), float(page_[2] - c.oz)};
        tgt_ = {float(page_[3] - c.ox), float(page_[4] - c.oy), float(page_[5] - c.oz)};
        c.fovTgt = 85;
    }
    if (v >= 4 && v != 13 && world.ground) {
        pos_.y = std::max(pos_.y, hRel + 0.5f);
        nearCand = std::max(pos_.y - hRel - 0.1f, 0.25f);
    }
    if (v < 4) {
        if (world.ground) nearCand = std::max(pos_.y - c.groundRel(pos_, true) - 0.1f, 0.25f);
    }
    if (v <= 5) roughShake(c, pos_, tgt_, dt);
    out.blur = blur;
    out.zNear = std::max(std::min(c.k[9], nearCand), 0.06f);
    out.zFar = 50000;
    fov_ = c.snap ? c.fovTgt : smooth1(fov_, c.fovTgt, 1.05f, dt);
    if (c.veh->speedFrac >= 0.95f) rand01(), rand01();
    out.ex = c.ox + pos_.x, out.ey = c.oy + pos_.y, out.ez = c.oz + pos_.z;
    out.tx = c.ox + tgt_.x, out.ty = c.oy + tgt_.y, out.tz = c.oz + tgt_.z;
    out.fovDeg = fov_;
    out.roll = c.roll;
    out.view = v;
    snap_ = v == 8 && tView_ >= 3.0f;  // the intro snaps every frame after its shots
    return out;
}

}  // namespace fuel::cam
