#include "rider_anim.hpp"

#include <algorithm>
#include <cmath>
#include <limits>

namespace fuel {

namespace {

using Q = std::array<double, 4>;
using V3 = std::array<double, 3>;
using M3 = std::array<double, 9>;  // row-vector rotation (v' = v M), rows = images of the basis

constexpr int kNone = 47;
constexpr int kHydro = 6;
constexpr int kMbikeSport = 3;

float clamp01(float x) { return x < 0 ? 0 : (x > 1 ? 1 : x); }
float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }

// Smoothing primitive 0x40fb70 / 0x40ef00: x += (target - x) (1 - rate^(-30.30303 dt)).
double smoothK(float rate, float dt) { return 1.0 - 1.0 / std::pow(double(rate), double(dt) * 30.30303); }

Q slerp(Q a, Q b, double f) {
    double d = a[0] * b[0] + a[1] * b[1] + a[2] * b[2] + a[3] * b[3];
    if (d < 0) {
        for (double& c : b) c = -c;
        d = -d;
    }
    Q o;
    if (d > 0.9995) {
        double n = 0;
        for (size_t i = 0; i < 4; ++i) o[i] = a[i] + (b[i] - a[i]) * f, n += o[i] * o[i];
        n = std::sqrt(n);
        for (double& c : o) c /= n;
        return o;
    }
    const double th = std::acos(std::min(1.0, d)), s = std::sin(th);
    const double wa = std::sin((1 - f) * th) / s, wb = std::sin(f * th) / s;
    for (size_t i = 0; i < 4; ++i) o[i] = wa * a[i] + wb * b[i];
    return o;
}

// Row-vector rotation matrix of q, as D3DXMatrixRotationQuaternion (rider.cpp).
M3 rot(const Q& qin) {
    Q q = qin;
    const double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (n > 0)
        for (double& c : q) c /= n;
    const double x = q[0], y = q[1], z = q[2], w = q[3];
    return {1 - 2 * (y * y + z * z), 2 * (x * y + z * w), 2 * (x * z - y * w), 2 * (x * y - z * w), 1 - 2 * (x * x + z * z),
            2 * (y * z + x * w), 2 * (x * z + y * w), 2 * (y * z - x * w), 1 - 2 * (x * x + y * y)};
}
Q quatOf(const M3& m) {  // inverse of rot()
    const double tr = m[0] + m[4] + m[8];
    Q q;
    if (tr > 0) {
        const double s = std::sqrt(tr + 1.0) * 2;
        q = {(m[5] - m[7]) / s, (m[6] - m[2]) / s, (m[1] - m[3]) / s, 0.25 * s};
    } else if (m[0] > m[4] && m[0] > m[8]) {
        const double s = std::sqrt(1.0 + m[0] - m[4] - m[8]) * 2;
        q = {0.25 * s, (m[1] + m[3]) / s, (m[6] + m[2]) / s, (m[5] - m[7]) / s};
    } else if (m[4] > m[8]) {
        const double s = std::sqrt(1.0 + m[4] - m[0] - m[8]) * 2;
        q = {(m[1] + m[3]) / s, 0.25 * s, (m[5] + m[7]) / s, (m[6] - m[2]) / s};
    } else {
        const double s = std::sqrt(1.0 + m[8] - m[0] - m[4]) * 2;
        q = {(m[6] + m[2]) / s, (m[5] + m[7]) / s, 0.25 * s, (m[1] - m[3]) / s};
    }
    return q;
}
M3 mul3(const M3& a, const M3& b) {  // apply a then b (row vectors)
    M3 r{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) r[size_t(i * 3 + j)] = a[size_t(i * 3)] * b[size_t(j)] + a[size_t(i * 3 + 1)] * b[size_t(3 + j)] + a[size_t(i * 3 + 2)] * b[size_t(6 + j)];
    return r;
}
M3 transpose(const M3& a) { return {a[0], a[3], a[6], a[1], a[4], a[7], a[2], a[5], a[8]}; }
M3 rot3Of(const Mat4& m) {  // rotation part with the row scale removed
    M3 r;
    for (int i = 0; i < 3; ++i) {
        const double l = std::sqrt(m[size_t(i * 4)] * m[size_t(i * 4)] + m[size_t(i * 4 + 1)] * m[size_t(i * 4 + 1)] + m[size_t(i * 4 + 2)] * m[size_t(i * 4 + 2)]);
        for (int j = 0; j < 3; ++j) r[size_t(i * 3 + j)] = l > 0 ? m[size_t(i * 4 + j)] / l : (i == j);
    }
    return r;
}
V3 posOf(const Mat4& m) { return {m[12], m[13], m[14]}; }
V3 sub(const V3& a, const V3& b) { return {a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }
V3 add(const V3& a, const V3& b) { return {a[0] + b[0], a[1] + b[1], a[2] + b[2]}; }
V3 scl(const V3& a, double s) { return {a[0] * s, a[1] * s, a[2] * s}; }
double dot(const V3& a, const V3& b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
V3 cross(const V3& a, const V3& b) { return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]}; }
double length(const V3& a) { return std::sqrt(dot(a, a)); }
V3 unit(const V3& a) {
    const double l = length(a);
    return l > 1e-12 ? scl(a, 1.0 / l) : a;
}
// Row-vector matrix rotating unit a onto unit b (shortest arc).
M3 fromTo(const V3& a, const V3& b) {
    const V3 ax = cross(a, b);
    const double c = std::clamp(dot(a, b), -1.0, 1.0), s = length(ax);
    if (s < 1e-9) return {1, 0, 0, 0, 1, 0, 0, 0, 1};
    const V3 k = scl(ax, 1.0 / s);
    // column-vector Rodrigues R, then transpose for row vectors
    const double t = 1 - c;
    const M3 R = {c + k[0] * k[0] * t,        k[0] * k[1] * t - k[2] * s, k[0] * k[2] * t + k[1] * s,
                  k[1] * k[0] * t + k[2] * s, c + k[1] * k[1] * t,        k[1] * k[2] * t - k[0] * s,
                  k[2] * k[0] * t - k[1] * s, k[2] * k[1] * t + k[0] * s, c + k[2] * k[2] * t};
    return transpose(R);
}
// Vehicle quaternion (Hamilton, rotate = q v q*) as a row-vector matrix.
M3 vehRot(const std::array<float, 4>& q) {
    // column matrix of q, transposed
    const double x = q[0], y = q[1], z = q[2], w = q[3];
    const M3 R = {1 - 2 * (y * y + z * z), 2 * (x * y - z * w),     2 * (x * z + y * w),
                  2 * (x * y + z * w),     1 - 2 * (x * x + z * z), 2 * (y * z - x * w),
                  2 * (x * z - y * w),     2 * (y * z + x * w),     1 - 2 * (x * x + y * y)};
    return transpose(R);
}
std::array<float, 3> qrotate(const std::array<float, 4>& q, const std::array<float, 3>& v) {
    const float x = q[0], y = q[1], z = q[2], w = q[3];
    const float tx = 2 * (y * v[2] - z * v[1]), ty = 2 * (z * v[0] - x * v[2]), tz = 2 * (x * v[1] - y * v[0]);
    return {v[0] + w * tx + (y * tz - z * ty), v[1] + w * ty + (z * tx - x * tz), v[2] + w * tz + (x * ty - y * tx)};
}
std::array<float, 4> conjq(const std::array<float, 4>& q) { return {-q[0], -q[1], -q[2], q[3]}; }

}  // namespace

std::array<float, 3> RiderAnimator::swayTarget(const std::array<float, 4>& q, const std::array<float, 3>& vel) {
    const float s = std::sqrt(vel[0] * vel[0] + vel[1] * vel[1] + vel[2] * vel[2]);
    return qrotate(q, {0, std::min(0.005f * s, 0.05f), 0});
}

RiderAnimator::RiderAnimator(const Rider& rider, const RiderTables& tables, const std::string& archetype, int vclass,
                             const std::vector<float>& P, const std::array<std::array<float, 3>, 4>& ik)
    : rider_(rider), tables_(tables), arch_(riderArchetypeIndex(archetype)), vclass_(vclass), P_(P), ikTarget_(ik) {
    head_ = rider.bone("Head");
    hips_ = rider.bone("Hips");
    const char* names[4][3] = {{"LeftHand", "LeftForeArm", "LeftArm"},
                               {"RightHand", "RightForeArm", "RightArm"},
                               {"LeftFoot", "LeftLeg", "LeftUpLeg"},
                               {"RightFoot", "RightLeg", "RightUpLeg"}};
    for (int l = 0; l < 4; ++l)
        for (int k = 0; k < 3; ++k) chain_[size_t(l)][size_t(k)] = rider.bone(names[l][k]);
    if (arch_ >= 0 && !rider.hasSlot(tables_.slots[size_t(arch_)][0])) arch_ = -1;
    reset();
    if (arch_ >= 0) pose(RiderDrive{});
}

float RiderAnimator::rand01() {  // a Park-Miller draw (the engine's 0x6460f0 shares its sequence with the whole game)
    rng_ = uint32_t((uint64_t(rng_) * 16807u) % 2147483647u);
    return std::min(float(rng_) * 4.656613e-10f, 0.9999999f);
}

void RiderAnimator::reset() {  // 0x5d43d0
    for (int l = 0; l < 3; ++l) layers_[l] = {};
    oneShot_ = lastOneShot_ = kNone;
    oneShotLeft_ = -1;
    stanceT_ = 0;
    bigJump_ = landing_ = false;
    stance_ = 4;
}

void RiderAnimator::push(Request* req, int act, float w, float rate) {  // 0x5d5bb0: first free slot; a third is dropped
    for (int i = 0; i < 2; ++i)
        if (req[i].act == -1) {
            req[i] = {act, w, rate};
            return;
        }
}

void RiderAnimator::startOneShot(int base, int count, bool allowRepeat) {  // 0x5d5c10
    int idx = base + int(std::floor(rand01() * (float(count) - 1 + 0.9999f)));
    if (!allowRepeat && idx == lastOneShot_) {
        ++idx;
        if (idx >= base + count) idx = base;
    }
    oneShot_ = lastOneShot_ = idx;
}

void RiderAnimator::startLayer(int l, int act, float w0, bool oneShot) {
    Layer& L = layers_[l];
    L.action = act;
    L.slot = tables_.slots[size_t(arch_)][size_t(act)];
    if (!rider_.hasSlot(L.slot)) L.slot = tables_.slots[size_t(arch_)][0];
    L.time = 0;
    L.weight = w0;
    L.oneShot = oneShot;
}

float RiderAnimator::mixer(Request a, Request b, float dt, bool oneShotRunning) {  // 0x5d3b40
    if (layers_[0].action != 0) startLayer(0, 0, 1, false);
    layers_[0].weight = 1;
    bool doneA = a.act < 0, doneB = b.act < 0;
    auto approach = [&](Layer& L, const Request& r) {
        if (r.rate < 0) L.weight = r.w;
        else if (L.weight < r.w) L.weight = std::min(r.w, L.weight + r.rate * dt);
        else L.weight = std::max(r.w, L.weight - r.rate * dt);
    };
    for (int l = 1; l < 3; ++l) {
        Layer& L = layers_[l];
        if (L.action < 0) continue;
        if (!doneA && L.action == a.act) approach(L, a), doneA = true;
        else if (!doneB && L.action == b.act) approach(L, b), doneB = true;
        else {
            L.weight -= 1.5f * dt;
            if (L.weight <= 0) L = {};
        }
    }
    float started = -1;
    for (const Request* r : {&a, &b}) {
        if ((r == &a ? doneA : doneB) || r->act < 0) continue;
        int l = layers_[1].action < 0 ? 1 : layers_[2].action < 0 ? 2 : -1;
        const RiderActionProps& np = tables_.props[size_t(r->act)];
        if (l > 0) {  // 0x5d4430 gate against the other layer
            const Layer& other = layers_[3 - l];
            if (other.action >= 0) {
                const RiderActionProps& op = tables_.props[size_t(other.action)];
                if (op.priority > np.priority && op.mode == 0) l = -1;
            }
        } else {  // displace a lower-priority layer whose mode allows it
            for (int k = 1; k < 3; ++k) {
                const RiderActionProps& op = tables_.props[size_t(layers_[k].action)];
                if (op.mode != 0 && op.priority < np.priority) l = k;
            }
        }
        if (l < 0) continue;
        const bool oneShot = r->act == oneShot_;
        startLayer(l, r->act, r->rate < 0 ? r->w : 0, oneShot);
        if (oneShot) started = rider_.duration(layers_[l].slot);
    }
    // 0x5d42a0: the higher priority blends last
    if (layers_[1].action >= 0 && layers_[2].action >= 0 &&
        tables_.props[size_t(layers_[1].action)].priority > tables_.props[size_t(layers_[2].action)].priority)
        std::swap(layers_[1], layers_[2]);
    if (started > 0) return started;
    for (int l = 1; l < 3; ++l)
        if (layers_[l].action == oneShot_ && layers_[l].oneShot) return float(rider_.duration(layers_[l].slot) - layers_[l].time);
    return oneShotRunning ? 0.0f : -1.0f;
}

void RiderAnimator::sway(const RiderDrive& d) {  // 0x5d7e00
    const float dt = d.dt;
    const std::array<float, 3> T = swayTarget(d.q, d.vel);
    std::array<float, 3> a{};
    if (haveVel_ && dt > 0)
        for (int i = 0; i < 3; ++i) a[size_t(i)] = (d.vel[size_t(i)] - prevVel_[size_t(i)]) / dt;
    prevVel_ = d.vel, haveVel_ = true;
    const auto U = qrotate(d.q, {0, 1, 0}), F = qrotate(d.q, {0, 0, 1}), X = qrotate(d.q, {1, 0, 0});
    auto dotf = [](const std::array<float, 3>& p, const std::array<float, 3>& q) { return p[0] * q[0] + p[1] * q[1] + p[2] * q[2]; };
    const float aU = dotf(a, U), aF = dotf(a, F), aX = dotf(a, X);
    for (int i = 0; i < 3; ++i) {
        float& w = swayW_[size_t(i)];
        w += (T[size_t(i)] - swayOff_[size_t(i)]) * 50 * dt;
        w -= 5 * dt * aU * U[size_t(i)];
        w -= dt * aF * F[size_t(i)];
        w += dt * aX * X[size_t(i)];
    }
    for (int i = 0; i < 3; ++i) {
        swayW_[size_t(i)] -= 5 * dt * swayW_[size_t(i)];
        swayOff_[size_t(i)] += swayW_[size_t(i)] * dt;
    }
    // clamp in the vehicle frame
    const float sp = std::sqrt(dotf(d.vel, d.vel));
    auto loc = qrotate(conjq(d.q), swayOff_);
    loc[0] = clampf(loc[0], -0.05f, 0.05f);
    loc[2] = clampf(loc[2], -0.05f, 0.05f);
    loc[1] = clampf(loc[1], -std::min(0.1f * sp, 0.1f), 0.1f);
    swayOff_ = qrotate(d.q, loc);
    const float k = float(smoothK(1.05f, dt));
    for (int i = 0; i < 3; ++i) swayOut_[size_t(i)] += (swayOff_[size_t(i)] - swayOut_[size_t(i)]) * k;
}

void RiderAnimator::update(const RiderDrive& d) {
    if (arch_ < 0) return;
    const float dt = d.dt;
    // R8: ejection (P655 > 0.5 on a wreck) and restore
    if (d.ejected) {
        if (!ejected_) {
            reset();
            ejected_ = true;
            fallT_ = 0;
            fallStart_ = world_;
            fallOrigin_ = d.pos;
            const auto hp = hips_ >= 0 ? posOf(world_[size_t(hips_)]) : V3{0, 1, 0};
            const auto hw = qrotate(d.q, {float(hp[0]), float(hp[1]), float(hp[2])});
            fallPos_ = {d.pos[0] + hw[0], d.pos[1] + hw[1], d.pos[2] + hw[2]};
            fallVel_ = {d.vel[0], d.vel[1] + 2.5f, d.vel[2]};
            fallQ_ = d.q;
            const auto side = qrotate(d.q, {1, 0, 0});
            fallSpin_ = {side[0] * -4.0f, side[1] * -4.0f, side[2] * -4.0f};
        }
        fall(d);
        return;
    }
    if (ejected_) {  // 0x5c8c30 restore: the machine restarts from its reset
        ejected_ = false;
        reset();
    }
    sway(d);

    const float speed = std::sqrt(d.vel[0] * d.vel[0] + d.vel[1] * d.vel[1] + d.vel[2] * d.vel[2]);
    const float S = -d.steer, Z = d.stickZ;
    const bool bikeOrQuad = vclass_ == 1 || vclass_ == 3;
    Request req[2];
    float oneShotRate = 3.0f;
    levelHead_ = vclass_ != 1;
    // vehicle-local lateral velocity (for the powerslides)
    const auto lv = qrotate(conjq(d.q), d.vel);
    const float lat = lv[0];
    const float top = P_.size() > 619 ? P_[619] : 100.0f;
    const float speedRatio = clamp01(speed / (top * 0.86956525f * 0.2777778f));
    const bool fast = speedRatio >= 0.85f;
    auto layerPlays = [&](int a) { return layers_[1].action == a || layers_[2].action == a; };

    if (d.raceState == 6 || d.raceState == 7) {
        bigJump_ = landing_ = false;
        stance_ = 4;
    } else if (d.raceState == 3 || d.raceState == 4) {
        if (oneShot_ == kNone) startOneShot(8, 1, true);
        stanceT_ = 0;
        stance_ = 4;
    } else {
        if (stance_ != 4 && speed > 1) {
            oneShot_ = kNone;
            stanceT_ = 0;
            stance_ = 4;
        }
        if (d.airtime > 0.1f) {  // R4 5a
            if (d.heightAbove > 0.8f && !bigJump_) landing_ = true;
            if (!bigJump_ && d.heightAbove > 2 && d.vel[1] > 8 && arch_ != kMbikeSport && vclass_ != kHydro) {
                startOneShot(14, 3, false);
                bigJump_ = true;
                oneShotRate = 5.0f;
            } else {
                const float zw = clamp01(2 * (std::fabs(Z) - 0.5f)), sw = clamp01(2 * (std::fabs(S) - 0.5f));
                bool any = false;
                if (Z < -0.5f) any = true, push(req, 20, zw, oneShotRate);
                else if (Z > 0.5f) any = true, push(req, 19, zw, oneShotRate);
                if (S < -0.5f) any = true, push(req, 17, sw, oneShotRate);
                else if (S > 0.5f) any = true, push(req, 18, sw, oneShotRate);
                if (!any) {
                    oneShot_ = kNone;
                    const float hw = clamp01(2 * d.heightAbove);
                    if (hw > 0) push(req, 13, hw, oneShotRate);
                }
            }
        } else {  // R4 5b
            if (landing_) {
                startOneShot(45, 1, true);
                oneShotRate = -1;
                bigJump_ = landing_ = false;
            } else if (d.bump > 4) {
                startOneShot(46, 1, true);
                oneShotRate = -1;
                bigJump_ = false;
            } else if (speed > 1) {
                stanceT_ = 0;
                if (d.gear == -1) {
                    levelHead_ = true;
                    oneShot_ = kNone;
                    push(req, 7, 1, oneShotRate);
                }
                if (d.airtime < 0.1f) {
                    const float w = std::min(d.wheelBumps * 50.0f, 1.0f) * std::min(speed * 0.04f, 1.0f);
                    if (w > 0) push(req, 43, w, oneShotRate);
                }
            } else {  // stance machine (+0x258)
                const bool input = d.accel || d.brake;
                switch (stance_) {
                    case 4:
                        stance_ = 0;
                        oneShotRate = 1;
                        startOneShot(8, 1, true);
                        break;
                    case 0:
                        if (stanceT_ < 5 || !bikeOrQuad) {
                            oneShotRate = 1;
                            stanceT_ += dt;
                            if (oneShot_ == kNone) startOneShot(8, 1, true);
                        } else if (oneShot_ == kNone) {
                            oneShotRate = -1;
                            stance_ = 1;
                            startOneShot(9, 1, true);
                        }
                        break;
                    case 1:
                        oneShotRate = -1;
                        if (oneShot_ == kNone) stance_ = 2, startOneShot(10, 1, true);
                        break;
                    case 2:
                        if (!d.player || !input) {
                            oneShotRate = -1;
                            if (oneShot_ == kNone) startOneShot(10, 3, false);
                        } else {
                            oneShotRate = 1;
                            stance_ = 3;
                            startOneShot(8, 1, true);
                            stanceT_ = 0;
                        }
                        break;
                    case 3:
                        if (!d.player || input) {
                            if (stanceT_ <= 0.5f) {
                                oneShotRate = 1;
                                if (oneShot_ == kNone) startOneShot(8, 1, true);
                                stanceT_ += dt;
                                break;
                            }
                            oneShot_ = kNone;
                            oneShotLeft_ = -1;
                        }
                        stance_ = 0;
                        stanceT_ = 0;
                        break;
                }
            }
            const float rate = (layerPlays(3) || layerPlays(4)) ? 1.0f : 3.0f;
            // powerslides 5/6
            if (d.sliding && lat > 0.2f && S < -0.5f && !layerPlays(6)) levelHead_ = true, push(req, 5, 1, rate);
            else if (d.sliding && lat < -0.2f && S > 0.5f && !layerPlays(5)) levelHead_ = true, push(req, 6, 1, rate);
            // lean fore/aft and tuck (1, 2, 44)
            const float zw = clamp01(2 * (std::fabs(Z) - 0.5f));
            if (fast && !layerPlays(43)) push(req, 44, 1, rate);
            else if (Z > 0.5f && stance_ == 4 && zw > 0) push(req, 1, zw, rate);
            if (Z < -0.5f && oneShot_ != 45 && stance_ == 4 && zw > 0) push(req, 2, zw, rate);
            // full-lock steering (3/4, high speed 41/42), archetype rates for the bikes
            float steerRate = rate;
            switch (arch_) {
                case 2: steerRate = 2.0f; break;
                case 3: steerRate = 1.75f; break;
                case 4: steerRate = 1.5f; break;
                case 5: steerRate = 1.75f; break;
                default: break;
            }
            const float sw = clamp01(2 * (std::fabs(S) - 0.5f));
            if (std::fabs(S) > 0.95f && !d.sliding && stance_ == 4 && sw > 0) {
                if (fast) levelHead_ = false;
                push(req, fast ? (S < 0 ? 41 : 42) : (S < 0 ? 3 : 4), sw, steerRate);
            }
        }
    }
    // R4 6: apply
    if (oneShot_ == kNone) {
        mixer(req[0], req[1], dt, false);
    } else {
        Request a = req[0], b = req[1];
        const Request os{oneShot_, 1, oneShotRate};
        if (a.act == -1) a = os;
        else if (b.act == -1) b = os;
        else if (b.w <= a.w) b = os;
        else a = os;
        float rem = mixer(a, b, dt, oneShotLeft_ > 0);
        oneShotLeft_ = rem;
        if (rem > 0) {
            rem -= dt;
            oneShotLeft_ = rem;
            if (rem <= dt) oneShot_ = kNone, oneShotLeft_ = -1;
        }
    }
    pose(d);
}

void RiderAnimator::pose(const RiderDrive& d) {
    const float dt = d.dt;
    if (layers_[0].action != 0) startLayer(0, 0, 1, false), layers_[0].weight = 1;
    // advance and sample the layers (0x7c02a0), then the sequential override blend (0x7bff70)
    std::vector<Rider::BoneTRS> base, other;
    std::vector<uint8_t> mask;
    for (int l = 0; l < 3; ++l) {
        Layer& L = layers_[l];
        if (L.action < 0) continue;
        const double dur = rider_.duration(L.slot);
        L.time += dt;
        if (rider_.loops(L.slot) && !L.oneShot && dur > 0) L.time = std::fmod(L.time, dur);
        else L.time = std::min(L.time, dur);
    }
    rider_.sampleLocal(layers_[0].slot, layers_[0].time, base, nullptr);
    for (int l = 1; l < 3; ++l) {
        const Layer& L = layers_[l];
        if (L.action < 0 || L.weight <= 0) continue;
        rider_.sampleLocal(L.slot, L.time, other, &mask);
        for (size_t b = 0; b < base.size(); ++b) {
            const double f = double(L.weight) * mask[b];
            if (f <= 0) continue;
            for (int k = 0; k < 3; ++k) {
                base[b].t[size_t(k)] += (other[b].t[size_t(k)] - base[b].t[size_t(k)]) * f;
                base[b].s[size_t(k)] += (other[b].s[size_t(k)] - base[b].s[size_t(k)]) * f;
            }
            base[b].q = slerp(base[b].q, other[b].q, f);
        }
    }
    // R7: rider root offset = sway output * P632 (vehicle frame), on the skeleton root
    const float p632 = P_.size() > 632 ? P_[632] : 0;
    if (p632 != 0) {
        const auto loc = qrotate(conjq(d.q), swayOut_);
        for (size_t b = 0; b < base.size(); ++b)
            if (rider_.parent(int(b)) < 0)
                for (int k = 0; k < 3; ++k) base[b].t[size_t(k)] += double(loc[size_t(k)] * p632);
    }
    // R6: head levelling for the player's bike/quad rider
    if (d.player && (vclass_ == 1 || vclass_ == 3) && head_ >= 0) headLevel(d, base, levelHead_, levelHead_ ? 1.1f : (stance_ == 4 ? 1.1f : 1.8f));
    world_ = rider_.worlds(base);
    if (d.player) ik(base, dt);
}

void RiderAnimator::headLevel(const RiderDrive& d, std::vector<Rider::BoneTRS>& local, bool level, float rate) {
    const Q anim = local[size_t(head_)].q;
    Q target = anim;
    if (level) {
        // the head kept level with the horizon: undo the vehicle's tilt (shortest arc from its up to world up)
        const std::vector<Mat4> w = rider_.worlds(local);
        const int p = rider_.parent(head_);
        const M3 V = vehRot(d.q);
        const auto vu = qrotate(d.q, {0, 1, 0});
        const M3 C = fromTo({vu[0], vu[1], vu[2]}, {0, 1, 0});
        const M3 Mh = rot3Of(w[size_t(head_)]);
        const M3 desiredModel = mul3(mul3(mul3(Mh, V), C), transpose(V));
        const M3 parentM = p >= 0 ? rot3Of(w[size_t(p)]) : M3{1, 0, 0, 0, 1, 0, 0, 0, 1};
        const Q lvl = quatOf(mul3(desiredModel, transpose(parentM)));
        target = slerp(anim, lvl, vclass_ == 1 ? 0.5 : 1.0);
    }
    if (!headInit_) headQ_ = target, headInit_ = true;
    headQ_ = slerp(headQ_, target, smoothK(rate, d.dt));
    local[size_t(head_)].q = headQ_;
}

void RiderAnimator::ik(std::vector<Rider::BoneTRS>& local, float dt) {
    // R9, approximated: an analytic two-bone solve per limb toward the vehicle's IK point (the engine iterates
    // a Jacobian, up to 15 steps, 5 mm; joint limits elbow 0.1..3.0 rad), eased in over 0.5 s (rate 1.4).
    ikT_ += dt;
    bool any = false;
    for (int l = 0; l < 4; ++l) {
        const auto& tg = ikTarget_[size_t(l)];
        const auto& ch = chain_[size_t(l)];
        if (!std::isnan(tg[0]) && ch[0] >= 0 && ch[1] >= 0 && ch[2] >= 0) any = true;
    }
    if (!any) return;
    std::vector<Mat4>& world = world_;
    const double ease = 1.0 - std::pow(1.4, -30.30303 * double(ikT_));
    auto rotateBone = [&](int b, const M3& Rmodel) {  // rotate bone b's model-space orientation by Rmodel (row)
        const int p = rider_.parent(b);
        const M3 W = rot3Of(world[size_t(b)]);
        const M3 Pm = p >= 0 ? rot3Of(world[size_t(p)]) : M3{1, 0, 0, 0, 1, 0, 0, 0, 1};
        local[size_t(b)].q = quatOf(mul3(mul3(W, Rmodel), transpose(Pm)));
        world = rider_.worlds(local);
    };
    for (int l = 0; l < 4; ++l) {
        const auto& tg = ikTarget_[size_t(l)];
        const auto& ch = chain_[size_t(l)];
        if (std::isnan(tg[0]) || ch[0] < 0 || ch[1] < 0 || ch[2] < 0) continue;
        const V3 a = posOf(world[size_t(ch[2])]), b = posOf(world[size_t(ch[1])]), c = posOf(world[size_t(ch[0])]);
        V3 t{tg[0], tg[1], tg[2]};
        t = add(c, scl(sub(t, c), ease));
        const double la = length(sub(b, a)), lb = length(sub(c, b));
        if (la < 1e-4 || lb < 1e-4) continue;
        V3 dir = sub(t, a);
        double dd = length(dir);
        if (dd < 1e-4) continue;
        dir = scl(dir, 1.0 / dd);
        // joint limit on the bend (pi - interior angle) 0.1 .. 3.0 rad
        const double dMax = std::sqrt(la * la + lb * lb - 2 * la * lb * std::cos(M_PI - 0.1));
        const double dMin = std::sqrt(la * la + lb * lb - 2 * la * lb * std::cos(M_PI - 3.0));
        dd = std::clamp(dd, dMin, dMax);
        const double cosA = std::clamp((la * la + dd * dd - lb * lb) / (2 * la * dd), -1.0, 1.0);
        V3 pole = sub(sub(b, a), scl(dir, dot(sub(b, a), dir)));
        if (length(pole) < 1e-6) pole = cross(dir, {1, 0, 0});
        pole = unit(pole);
        const V3 bNew = add(a, add(scl(dir, la * cosA), scl(pole, la * std::sqrt(std::max(0.0, 1 - cosA * cosA)))));
        rotateBone(ch[2], fromTo(unit(sub(b, a)), unit(sub(bNew, a))));
        const V3 b2 = posOf(world[size_t(ch[1])]), c2 = posOf(world[size_t(ch[0])]);
        const V3 tReach = add(a, scl(dir, dd));
        rotateBone(ch[1], fromTo(unit(sub(c2, b2)), unit(sub(tReach, b2))));
    }
}

void RiderAnimator::fall(const RiderDrive& d) {
    // R8, approximated: FUEL builds a 10-body ragdoll (nodes 25,31,28,59,56,51,32,29,30,27, masses 4000/500,
    // ~13 joints; U-R2) on its rigid bodies. The port throws the rider as one tumbling body with gravity
    // -9.81 that slides to rest on the terrain, keeping the last riding pose eased toward a limp one.
    const float dt = d.dt;
    fallT_ += dt;
    fallVel_[1] -= 9.81f * dt;
    for (int k = 0; k < 3; ++k) fallPos_[size_t(k)] += double(fallVel_[size_t(k)] * dt);
    const float g = d.ground ? d.ground(fallPos_[0], fallPos_[2]) : -1e9f;
    bool onGround = false;
    if (fallPos_[1] < double(g) + 0.25) {
        fallPos_[1] = double(g) + 0.25;
        if (fallVel_[1] < 0) fallVel_[1] = -0.25f * fallVel_[1];
        const float f = std::max(0.0f, 1.0f - 4.0f * dt);
        fallVel_[0] *= f, fallVel_[2] *= f;
        for (float& s : fallSpin_) s *= std::max(0.0f, 1.0f - 6.0f * dt);
        onGround = true;
    }
    // tumble: integrate the spin until the body lies flat
    const float sl = std::sqrt(fallSpin_[0] * fallSpin_[0] + fallSpin_[1] * fallSpin_[1] + fallSpin_[2] * fallSpin_[2]);
    if (sl > 1e-4f && !(onGround && sl < 0.3f)) {
        const float h = sl * dt * 0.5f, s = std::sin(h) / sl;
        const std::array<float, 4> dq{fallSpin_[0] * s, fallSpin_[1] * s, fallSpin_[2] * s, std::cos(h)};
        const auto& q = fallQ_;
        fallQ_ = {dq[3] * q[0] + dq[0] * q[3] + dq[1] * q[2] - dq[2] * q[1], dq[3] * q[1] - dq[0] * q[2] + dq[1] * q[3] + dq[2] * q[0],
                  dq[3] * q[2] + dq[0] * q[1] - dq[1] * q[0] + dq[2] * q[3], dq[3] * q[3] - dq[0] * q[0] - dq[1] * q[1] - dq[2] * q[2]};
    }
    // limbs ease (0x607bd0: 0.33 s) from the riding pose toward the bind pose (arms and legs spread)
    const double e = std::min(1.0, double(fallT_) * 3.0303) * 0.5;
    std::vector<Rider::BoneTRS> bind = rider_.bindLocal();
    std::vector<Rider::BoneTRS> ride = bind;
    // reconstruct the riding locals from fallStart_ rotations
    for (size_t b = 0; b < bind.size() && b < fallStart_.size(); ++b) {
        const int p = rider_.parent(int(b));
        const M3 R = rot3Of(fallStart_[b]);
        const M3 Pm = p >= 0 ? rot3Of(fallStart_[size_t(p)]) : M3{1, 0, 0, 0, 1, 0, 0, 0, 1};
        ride[b].q = slerp(quatOf(mul3(R, transpose(Pm))), bind[b].q, p >= 0 ? e : 0.0);
    }
    std::vector<Mat4> w = rider_.worlds(ride);
    // place: hips at fallPos_, orientation fallQ_, all relative to fallOrigin_
    const V3 hp = hips_ >= 0 ? posOf(w[size_t(hips_)]) : V3{0, 0, 0};
    const M3 R = vehRot(fallQ_);
    world_.resize(w.size());
    for (size_t b = 0; b < w.size(); ++b) {
        Mat4 m = w[b];
        const V3 p = sub(posOf(m), hp);
        // rotation rows times R
        for (int r = 0; r < 3; ++r) {
            const double x = m[size_t(r * 4)], y = m[size_t(r * 4 + 1)], z = m[size_t(r * 4 + 2)];
            m[size_t(r * 4)] = x * R[0] + y * R[3] + z * R[6];
            m[size_t(r * 4 + 1)] = x * R[1] + y * R[4] + z * R[7];
            m[size_t(r * 4 + 2)] = x * R[2] + y * R[5] + z * R[8];
        }
        const V3 pr = {p[0] * R[0] + p[1] * R[3] + p[2] * R[6], p[0] * R[1] + p[1] * R[4] + p[2] * R[7], p[0] * R[2] + p[1] * R[5] + p[2] * R[8]};
        m[12] = pr[0] + (fallPos_[0] - fallOrigin_[0]);
        m[13] = pr[1] + (fallPos_[1] - fallOrigin_[1]);
        m[14] = pr[2] + (fallPos_[2] - fallOrigin_[2]);
        world_[b] = m;
    }
}

}  // namespace fuel
