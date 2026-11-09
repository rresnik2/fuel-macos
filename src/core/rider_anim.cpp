#include "rider_anim.hpp"

#include "resources.hpp"

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
Q conjQ(const Q& q) { return {-q[0], -q[1], -q[2], q[3]}; }

}  // namespace

std::array<float, 3> RiderAnimator::swayTarget(const std::array<float, 4>& q, const std::array<float, 3>& vel) {
    const float s = std::sqrt(vel[0] * vel[0] + vel[1] * vel[1] + vel[2] * vel[2]);
    return qrotate(q, {0, std::min(0.005f * s, 0.05f), 0});
}

RiderAnimator::RiderAnimator(const Rider& rider, const RiderTables& tables, const std::string& archetype, int vclass,
                             const std::vector<float>& P, const std::array<std::array<float, 3>, 4>& ik, const std::array<double, 3>& seat)
    : rider_(rider), tables_(tables), arch_(riderArchetypeIndex(archetype)), vclass_(vclass), P_(P), seat_(seat), ikTarget_(ik) {
    head_ = rider.bone("Head");
    hips_ = rider.bone("Hips");
    const char* names[4][3] = {{"LeftHand", "LeftForeArm", "LeftArm"},
                               {"RightHand", "RightForeArm", "RightArm"},
                               {"LeftFoot", "LeftLeg", "LeftUpLeg"},
                               {"RightFoot", "RightLeg", "RightUpLeg"}};
    for (int l = 0; l < 4; ++l)
        for (int k = 0; k < 3; ++k) chain_[size_t(l)][size_t(k)] = rider.bone(names[l][k]);
    for (int l = 0; l < 4; ++l) limbBone_[size_t(l)] = asoboCrc(names[l][2]);  // the event tracks: LeftArm ... RightUpLeg
    for (auto& f : handFrame_) f[0] = std::numeric_limits<double>::quiet_NaN();  // PK31: no grip orientation until set
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
    applyEvents(L.slot, -1e30, 0, true);  // P76, 0x64f3f0 (PlayBlendAnim vf22): state events with time <= the start time
}

// P76: 0x7c0480 (after each layer advance, 0x7c02a0) and 0x64f3f0: kinds 0x10 / 0x11 on LeftArm, RightArm, LeftUpLeg,
// RightUpLeg set / clear node flag 0x400 on that limb (0x675f60 / 0x675f90, subtree); the hand / foot IK (0x6032f0,
// 0x603b80, 0x602330, 0x602ad0) leaves a flagged limb to the animation. Applied whatever the layer's weight.
void RiderAnimator::applyEvents(int slot, double from, double to, bool closed) {
    if (!rider_.hasSlot(slot)) return;
    std::vector<std::pair<uint32_t, uint32_t>> ev;
    rider_.events(slot, from, to, closed, ev);
    for (const auto& [bone, kind] : ev) {
        if (kind != 0x10 && kind != 0x11) continue;
        for (size_t l = 0; l < 4; ++l)
            if (limbBone_[l] == bone && limbOff_[l] != (kind == 0x10)) limbOff_[l] = kind == 0x10, limbT_[l] = 0;
    }
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
            fallQ_ = d.q;
            ragStart(d);
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
        const double prev = L.time;
        L.time += dt;
        if (rider_.loops(L.slot) && !L.oneShot && dur > 0) {
            if (L.time > dur) applyEvents(L.slot, prev, dur, true), applyEvents(L.slot, 0, std::fmod(L.time, dur), false);  // wrap
            else applyEvents(L.slot, prev, L.time, false);
            L.time = std::fmod(L.time, dur);
        } else {
            L.time = std::min(L.time, dur);
            applyEvents(L.slot, prev, L.time, L.time >= dur);
        }
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
    // 0x5ca410: the pilot object sits at the vehicle's node 61 (its skeleton's Head bone, seat_) plus the sway
    // output * P632 (R7, vehicle frame); the rider's skeleton root carries both here.
    const float p632 = P_.size() > 632 ? P_[632] : 0;
    std::array<double, 3> rootOff = seat_;
    if (p632 != 0) {
        const auto loc = qrotate(conjq(d.q), swayOut_);
        for (int k = 0; k < 3; ++k) rootOff[size_t(k)] += double(loc[size_t(k)] * p632);
    }
    for (size_t b = 0; b < base.size(); ++b)
        if (rider_.parent(int(b)) < 0)
            for (int k = 0; k < 3; ++k) base[b].t[size_t(k)] += rootOff[size_t(k)];
    // R6 (0x5d5800): levelling for the player's bike/quad rider. It works on node 25 of the rider's node array =
    // the Hips in the bone-name registry (P59), not the head: the whole rider is turned toward the horizon.
    if (d.player && (vclass_ == 1 || vclass_ == 3) && hips_ >= 0) headLevel(d, base, levelHead_, levelHead_ ? 1.1f : (stance_ == 4 ? 1.1f : 1.8f));
    world_ = rider_.worlds(base);
    if (d.player) ik(base, dt);
}

void RiderAnimator::headLevel(const RiderDrive& d, std::vector<Rider::BoneTRS>& local, bool level, float rate) {
    const Q anim = local[size_t(hips_)].q;
    Q target = anim;
    if (level) {
        // the rider kept level with the horizon: undo the vehicle's tilt (shortest arc from its up to world up)
        const std::vector<Mat4> w = rider_.worlds(local);
        const int p = rider_.parent(hips_);
        const M3 V = vehRot(d.q);
        const auto vu = qrotate(d.q, {0, 1, 0});
        const M3 C = fromTo({vu[0], vu[1], vu[2]}, {0, 1, 0});
        const M3 Mh = rot3Of(w[size_t(hips_)]);
        const M3 desiredModel = mul3(mul3(mul3(Mh, V), C), transpose(V));
        const M3 parentM = p >= 0 ? rot3Of(w[size_t(p)]) : M3{1, 0, 0, 0, 1, 0, 0, 0, 1};
        const Q lvl = quatOf(mul3(desiredModel, transpose(parentM)));
        target = slerp(anim, lvl, vclass_ == 1 ? 0.5 : 1.0);
    }
    if (!headInit_) headQ_ = target, headInit_ = true;
    headQ_ = slerp(headQ_, target, smoothK(rate, d.dt));
    local[size_t(hips_)].q = headQ_;
}

namespace {
Q qmulH(const Q& a, const Q& b) {  // Hamilton a x b
    return {a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1], a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
            a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3], a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2]};
}
V3 qrotD(const Q& q, const V3& v) {  // q v q* (0x410510)
    const Q r = qmulH(qmulH(q, {v[0], v[1], v[2], 0}), {-q[0], -q[1], -q[2], q[3]});
    return {r[0], r[1], r[2]};
}
Q qnorm(Q q) {  // Quat_Normalise 0x41d4b0
    const double n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (n > 0)
        for (double& c : q) c /= n;
    return q;
}
}  // namespace

// ---- PK31: IK_SolveJacobian 0x601220
bool ikSolveJacobian(const IkJoint* joints, int count, const IkAccess& io, const V3& target, int* passes, bool* lockedOut) {
#pragma clang fp contract(off)  // x87 has no fused multiply-add (a singular J^T J must stay det == 0)
    count = std::min(count, 3);
    bool locked[3] = {false, false, false};
    V3 eff = io.effector();
    V3 e = sub(target, eff);
    if (passes) *passes = 0;
    if (dot(e, e) < 2.5e-5) return false;
    struct LockOut {
        bool* out;
        bool* l;
        ~LockOut() {
            if (out)
                for (int j = 0; j < 3; ++j) out[j] = l[j];
        }
    } lockOut{lockedOut, locked};
    for (int k = -2; k != 13; ++k) {
        if (passes) ++*passes;
        // Jacobian columns (rows of M = J^T)
        double M[3][3] = {};
        for (int j = 0; j < count; ++j) {
            if (locked[j]) continue;
            const V3 w = qrotD(io.worldQ(j), joints[j].axis);
            const V3 c = cross(w, sub(eff, io.jointPos(j)));
            M[j][0] = c[0], M[j][1] = c[1], M[j][2] = c[2];
        }
        // A = J^T J; d = A^-1 J^T e, or J^T e when A is singular (det == 0)
        double A[3][3];
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) A[i][j] = M[i][0] * M[j][0] + M[i][1] * M[j][1] + M[i][2] * M[j][2];
        double g[3];
        for (int j = 0; j < 3; ++j) g[j] = M[j][0] * e[0] + M[j][1] * e[1] + M[j][2] * e[2];
        const double c00 = A[1][1] * A[2][2] - A[1][2] * A[2][1], c01 = A[1][2] * A[2][0] - A[1][0] * A[2][2], c02 = A[1][0] * A[2][1] - A[1][1] * A[2][0];
        const double det = A[0][0] * c00 + A[0][1] * c01 + A[0][2] * c02;
        double th[3];
        if (det != 0) {
            const double inv[3][3] = {{c00 / det, (A[0][2] * A[2][1] - A[0][1] * A[2][2]) / det, (A[0][1] * A[1][2] - A[0][2] * A[1][1]) / det},
                                      {c01 / det, (A[0][0] * A[2][2] - A[0][2] * A[2][0]) / det, (A[0][2] * A[1][0] - A[0][0] * A[1][2]) / det},
                                      {c02 / det, (A[0][1] * A[2][0] - A[0][0] * A[2][1]) / det, (A[0][0] * A[1][1] - A[0][1] * A[1][0]) / det}};
            for (int j = 0; j < 3; ++j) th[j] = float(inv[j][0] * g[0] + inv[j][1] * g[1] + inv[j][2] * g[2]);
        } else {
            for (int j = 0; j < 3; ++j) th[j] = float(g[j]);
        }
        for (int j = 0; j < count; ++j) {
            if (locked[j]) th[j] = 0;
            th[j] = float(std::atan(th[j]));
        }
        // 0x601a50: max |theta| <= 0.25 / max(k, 1) (over all three slots)
        const double lim = float(0.25 / float(k > 0 ? k : 1));
        const double mx = std::max({std::fabs(th[0]), std::fabs(th[1]), std::fabs(th[2])});
        if (lim < mx) {
            const double f = float(lim / mx);
            for (double& t : th) t = float(t * f);
        }
        for (int j = 0; j < count; ++j) {
            const IkJoint& J = joints[j];
            Q& ql = io.local(j);
            const Q qold = ql;
            const double sh = std::sin(th[j] * 0.5), ch = std::cos(th[j] * 0.5);
            Q q = qmulH(qold, {J.axis[0] * sh, J.axis[1] * sh, J.axis[2] * sh, ch});
            if (J.limited && !locked[j]) {
                const double sdot = (q[2] * J.axis[2] + q[0] * J.axis[0]) + q[1] * J.axis[1];
                double lim2 = sdot;
                bool out = false;
                if (sdot < J.lo) lim2 = J.lo, out = true;
                else if (sdot > J.hi) lim2 = J.hi, out = true;
                if (out) {
                    locked[j] = true;
                    const double c = lim2 - sdot;
                    if (c != 0) {
                        const double s2 = std::sin(c * 0.5), c2 = std::cos(c * 0.5);
                        q = qmulH(qold, {J.axis[0] * s2, J.axis[1] * s2, J.axis[2] * s2, c2});
                    }
                }
            }
            ql = qnorm(q);
        }
        io.fk();
        eff = io.effector();
        e = sub(target, eff);
        if (dot(e, e) < 2.5e-5) return true;
    }
    return true;  // AL = 1 once a pass ran (uVar15), reached or not
}

// R9 (PK31: 0x604630 -> hands 0x6032f0 / 0x603b80, feet 0x602330 / 0x602ad0), the player's rider only. Per limb:
//  - limb let go by the animation (flag 0x400 on LeftArm / RightArm / LeftUpLeg / RightUpLeg, P76): "was off" = 1;
//    during 0.5 s after the switch (blend-out flag), when the animated effector is over 0.025 m from the cached
//    position the cached quats SmoothQuat (rate 1.4) toward the animation and are written back; else the cache
//    takes the animation pose and the blend ends.
//  - IK on: blend timer 0, blend-out flag on; coming back from "off" the cache takes the animated pose and effector
//    position; the chain (IK_AddJoint 0x6011b0): joint 0 = the mid node (elbow / knee) about its own rotation
//    vector (Quat_ToRotationVector 0x41d280 of its local quat, normalised; (0, 1, 0) under 0.001), hands limited to
//    [0.1, 3.0]; joint 1 = the root node (upper arm / leg) about that same axis carried into its frame (right hand
//    limited to [-10, 0], the others free); joint 2 = the root node about its local z, free; solved toward the
//    vehicle's IK point (0x601220). Solved effector within 0.05 m of the cached position: the cache takes the
//    solution; else the cache SmoothQuats (rate 1.6) toward it, is written back and the cached position updated.
//  - hands: 0x604410 sets the hand's local quaternion so its world orientation is the IK box's.
void RiderAnimator::ik(std::vector<Rider::BoneTRS>& local, float dt) {
    for (float& t : limbT_) t += dt;
    std::vector<Mat4>& world = world_;
    auto worldQ = [&](int b) { return quatOf(rot3Of(world[size_t(b)])); };
    auto fk = [&] { world = rider_.worlds(local); };
    auto smoothQ = [&](Q& c, const Q& tq, float rate) { c = qnorm(slerp(c, tq, smoothK(rate, dt))); };
    for (int l = 0; l < 4; ++l) {
        const auto& tg = ikTarget_[size_t(l)];
        const auto& ch = chain_[size_t(l)];  // {end, mid, root}
        if (std::isnan(tg[0]) || ch[0] < 0 || ch[1] < 0 || ch[2] < 0) continue;
        LimbIk& L = limbIk_[size_t(l)];
        const bool hand = l < 2;
        ikResult_[size_t(l)] = -1;
        auto readAnim = [&] {
            for (int k = 0; k < 3; ++k) L.q[size_t(k)] = local[size_t(ch[size_t(k)])].q;
        };
        auto writeCache = [&] {
            for (int k = 0; k < 3; ++k) local[size_t(ch[size_t(k)])].q = L.q[size_t(k)] = qnorm(L.q[size_t(k)]);
            fk();
        };
        const V3 effNow = posOf(world[size_t(ch[0])]);
        if (limbOff_[size_t(l)]) {
            L.wasOff = true;
            if (L.blendOut && L.blendT < 0.5f) {
                L.blendT += dt;
                if (length(sub(effNow, L.pos)) > 0.025) {
                    for (int k = 0; k < 3; ++k) smoothQ(L.q[size_t(k)], local[size_t(ch[size_t(k)])].q, 1.4f);
                    writeCache();
                    L.pos = posOf(world[size_t(ch[0])]);
                    continue;
                }
                L.blendOut = false;
                readAnim();
                continue;
            }
            L.blendOut = false;
            continue;
        }
        L.blendT = 0;
        L.blendOut = true;
        if (L.wasOff) {
            L.wasOff = false;
            readAnim();
            L.pos = effNow;
        }
        // the chain
        const int mid = ch[1], root = ch[2];
        IkJoint J[3];
        {
            const Q qm = local[size_t(mid)].q;
            const double w = std::clamp(qm[3], -1.0, 1.0), sn = std::sqrt(std::max(0.0, 1.0 - w * w));
            V3 rv{0, 0, 0};
            if (sn > 1e-6) rv = scl(V3{qm[0], qm[1], qm[2]}, 2.0 * std::acos(w) / sn);
            J[0].axis = length(rv) < 0.001 ? V3{0, 1, 0} : unit(rv);
        }
        J[1].axis = qrotD(qmulH(conjQ(worldQ(root)), worldQ(mid)), J[0].axis);
        J[2].axis = {0, 0, 1};
        if (hand) J[0].limited = true, J[0].lo = 0.1, J[0].hi = 3.0;
        J[1].lo = -10, J[1].hi = l == 1 ? 0.0 : 10.0, J[1].limited = l == 1;
        if (l == 0) J[1].lo = 0, J[1].hi = 10;
        const int nodes[3] = {mid, root, root};
        IkAccess io;
        io.local = [&](int j) -> Q& { return local[size_t(nodes[j])].q; };
        io.worldQ = [&](int j) { return worldQ(nodes[j]); };
        io.jointPos = [&](int j) { return posOf(world[size_t(nodes[j])]); };
        io.effector = [&] { return posOf(world[size_t(ch[0])]); };
        io.fk = fk;
        int passes = 0;
        ikSolveJacobian(J, 3, io, {tg[0], tg[1], tg[2]}, &passes);
        ikResult_[size_t(l)] = passes;
        const V3 solved = posOf(world[size_t(ch[0])]);
        if (length(sub(solved, L.pos)) <= 0.05) {
            readAnim();
        } else {
            for (int k = 0; k < 3; ++k) smoothQ(L.q[size_t(k)], local[size_t(ch[size_t(k)])].q, 1.6f);
            writeCache();
            L.pos = posOf(world[size_t(ch[0])]);
        }
        // 0x604410: the hand turned to the grip box
        if (hand && !std::isnan(handFrame_[size_t(l)][0])) {
            const int b = ch[0], p = rider_.parent(b);
            const M3 Pm = p >= 0 ? rot3Of(world[size_t(p)]) : M3{1, 0, 0, 0, 1, 0, 0, 0, 1};
            local[size_t(b)].q = quatOf(mul3(handFrame_[size_t(l)], transpose(Pm)));
            fk();
        }
    }
}

// R8 / P46: FUEL's ragdoll (0x605650): 10 IT_DYN_PILOT bodies at the rider's nodes [25, 31, 28, 59, 56, 51, 32, 29,
// 30, 27] with density 4000 (bodies 0 and 5) or 500, joined by 13 joints: type C (0x604f30) 1-0 and 2-0 (anchors
// 0.2125 down, 0.05 to the side, 0.15 forward of body 0: the hips), a ball (0x6047f0) 5-0 (0.2 up: the spine), type B
// (0x604cc0) 3-5 and 4-5 (0.175 to the side, 0.35 up: the shoulders), and two type A (0x604a60) per limb 6-1, 7-2,
// 8-3, 9-4 (knees, elbows; A places body B at the anchor distance from body A). Body 1/3 sit on -x (left), 2/4 on +x.
// Every body starts with the vehicle's velocity (0x6078d0), gravity -9.81 and damping 0.1 (0x601e00); the skeleton
// root follows body 0 and the other bodies drive their nodes (0x606f40 / 0x606e00); the other nodes ease to their
// rest pose over 0.33 s (0x607bd0).
// Port: the bodies are points solved as position constraints (the joints' rest lengths from the pose at the ejection,
// plus braces that keep the two torso bodies rigid) against the terrain; the joint types' limits and the engine's
// joint impulses (0x71ea90) are not ported, nor the bodies' shapes and their contacts with objects.
void RiderAnimator::ragStart(const RiderDrive& d) {
    static const char* const kBones[10] = {"Hips", "LeftUpLeg", "RightUpLeg", "LeftArm", "RightArm", "Spine2", "LeftLeg", "RightLeg",
                                           "LeftForeArm", "RightForeArm"};
    ragOk_ = true;
    ragSettled_ = false;  // PK31: 0x71b2f0 on the first step
    for (int i = 0; i < 10; ++i) {
        int b = rider_.bone(kBones[i]);
        if (b < 0 && i == 5) b = rider_.bone("Spine1");
        if (b < 0 && i == 5) b = rider_.bone("Spine");
        if (b < 0 || size_t(b) >= world_.size()) {
            ragOk_ = false;
            return;
        }
        RagBody& r = rag_[size_t(i)];
        r.bone = b;
        const V3 m = posOf(world_[size_t(b)]);
        const auto w = qrotate(d.q, {float(m[0]), float(m[1]), float(m[2])});
        r.x = r.prev = {double(w[0]), double(w[1]), double(w[2])};
        r.v = {double(d.vel[0]), double(d.vel[1]), double(d.vel[2])};
        r.invMass = 1.0 / (i == 0 || i == 5 ? 4000.0 : 500.0);
    }
    static const int kLinks[][2] = {{1, 0}, {2, 0}, {5, 0}, {3, 5}, {4, 5}, {6, 1}, {7, 2}, {8, 3}, {9, 4},
                                    // port braces standing in for the torso bodies' extent and the joint anchors
                                    {1, 2}, {3, 4}, {1, 5}, {2, 5}, {3, 0}, {4, 0}};
    ragLinks_.clear();
    for (const auto& l : kLinks)
        ragLinks_.push_back({l[0], l[1], length(sub(rag_[size_t(l[0])].x, rag_[size_t(l[1])].x))});
}

void RiderAnimator::ragStep(const RiderDrive& d) {
    constexpr int kSub = 4;
    constexpr double kR = 0.12;  // contact radius of a body
    const double h = double(d.dt) / kSub;
    if (!(h > 0)) return;
    // PK31 (from PK30, notes/pk30.md 3): the placement settle 0x71b2f0 (PhysIsland_Step 0x685170) runs once on an island
    // with a body whose +0x5ac == 0, and IT_DynPilotMove_G (0x601e50 / 0x601ea0) leaves it 0 on all 10 ragdoll bodies:
    // 5 rounds of y -= 0.01 on every body, then 0x71b5c0 lifts each by min(max over its contacts 0..n-2 of n.y x
    // depth, 0.05) (the last contact is skipped). The port's bodies have one contact each (the ground), the skipped
    // one, so they keep the 5 cm drop; the ground constraint below pushes them out as the solver would.
    if (!ragSettled_) {
        ragSettled_ = true;
        for (RagBody& r : rag_)
            for (int k = 0; k < 5; ++k) r.x[1] -= 0.01, r.prev[1] -= 0.01;
    }
    for (int s = 0; s < kSub; ++s) {
        for (RagBody& r : rag_) {
            r.v[1] -= 9.81 * h;
            r.v = scl(r.v, std::max(0.0, 1.0 - 0.1 * h));
            r.prev = r.x;
            r.x = add(r.x, scl(r.v, h));
        }
        for (int it = 0; it < 6; ++it) {
            for (const RagLink& l : ragLinks_) {
                RagBody &a = rag_[size_t(l.a)], &b = rag_[size_t(l.b)];
                const V3 dlt = sub(b.x, a.x);
                const double len = length(dlt), w = a.invMass + b.invMass;
                if (len < 1e-9 || w <= 0) continue;
                const V3 corr = scl(dlt, (len - l.rest) / (len * w));
                a.x = add(a.x, scl(corr, a.invMass));
                b.x = sub(b.x, scl(corr, b.invMass));
            }
            for (RagBody& r : rag_) {
                const double g = d.ground ? double(d.ground(r.x[0] + fallOrigin_[0], r.x[2] + fallOrigin_[2])) - fallOrigin_[1] : -1e9;
                if (r.x[1] < g + kR) r.x[1] = g + kR;
            }
        }
        for (RagBody& r : rag_) {
            r.v = scl(sub(r.x, r.prev), 1.0 / h);
            const double g = d.ground ? double(d.ground(r.x[0] + fallOrigin_[0], r.x[2] + fallOrigin_[2])) - fallOrigin_[1] : -1e9;
            if (r.x[1] <= g + kR + 1e-6) {  // ground friction on the sliding body
                const double k = std::max(0.0, 1.0 - 4.0 * h);
                r.v[0] *= k, r.v[2] *= k;
            }
        }
    }
}

void RiderAnimator::fall(const RiderDrive& d) {
    fallT_ += d.dt;
    // The non-ragdoll nodes ease (0x607bd0, 0.33 s) from the riding pose toward the rest pose.
    const double e = std::min(1.0, double(fallT_) * 3.0303) * 0.5;
    std::vector<Rider::BoneTRS> bind = rider_.bindLocal();
    std::vector<Rider::BoneTRS> local = bind;
    for (size_t b = 0; b < bind.size() && b < fallStart_.size(); ++b) {
        const int p = rider_.parent(int(b));
        const M3 R = rot3Of(fallStart_[b]);
        const M3 Pm = p >= 0 ? rot3Of(fallStart_[size_t(p)]) : M3{1, 0, 0, 0, 1, 0, 0, 0, 1};
        local[b].q = slerp(quatOf(mul3(R, transpose(Pm))), bind[b].q, p >= 0 ? e : 0.0);
    }
    if (!ragOk_) return;
    ragStep(d);
    // World = model x the vehicle rotation at the ejection; then the root (body 0) and the driven limbs.
    const M3 V = vehRot(fallQ_);
    auto worldOf = [&](const std::vector<Rider::BoneTRS>& loc) {
        std::vector<Mat4> w = rider_.worlds(loc);
        for (Mat4& m : w) {
            for (int r = 0; r < 4; ++r) {
                const double x = m[size_t(r * 4)], y = m[size_t(r * 4 + 1)], z = m[size_t(r * 4 + 2)];
                m[size_t(r * 4)] = x * V[0] + y * V[3] + z * V[6];
                m[size_t(r * 4 + 1)] = x * V[1] + y * V[4] + z * V[7];
                m[size_t(r * 4 + 2)] = x * V[2] + y * V[5] + z * V[8];
            }
        }
        return w;
    };
    // Rotate bone b (world rotation, about its own position) by R (row-vector), through its local rotation.
    auto rotateBone = [&](std::vector<Mat4>& w, int b, const M3& R) {
        const int p = rider_.parent(b);
        const M3 W = rot3Of(w[size_t(b)]);
        const M3 Pm = p >= 0 ? rot3Of(w[size_t(p)]) : V;
        local[size_t(b)].q = quatOf(mul3(mul3(W, R), transpose(Pm)));
    };
    std::vector<Mat4> w = worldOf(local);
    // Root: the pose's hip line and spine onto the bodies' (1 -> 2 and 0 -> 5).
    {
        const auto& B = rag_;
        const V3 pr = unit(sub(posOf(w[size_t(B[2].bone)]), posOf(w[size_t(B[1].bone)])));
        const V3 pu = unit(sub(posOf(w[size_t(B[5].bone)]), posOf(w[size_t(B[0].bone)])));
        const V3 rr = unit(sub(B[2].x, B[1].x)), ru = unit(sub(B[5].x, B[0].x));
        const M3 R1 = fromTo(pu, ru);
        const V3 pr1 = {pr[0] * R1[0] + pr[1] * R1[3] + pr[2] * R1[6], pr[0] * R1[1] + pr[1] * R1[4] + pr[2] * R1[7],
                        pr[0] * R1[2] + pr[1] * R1[5] + pr[2] * R1[8]};
        auto flat = [&](const V3& v) { return unit(sub(v, scl(ru, dot(v, ru)))); };
        const M3 R2 = fromTo(flat(pr1), flat(rr));
        const int root = B[0].bone;
        int top = root;
        while (rider_.parent(top) >= 0) top = rider_.parent(top);
        rotateBone(w, top, mul3(R1, R2));
        w = worldOf(local);
    }
    // Limbs: upper bone toward its lower body (1 -> 6, 2 -> 7, 3 -> 8, 4 -> 9).
    for (const auto& pr : {std::array<int, 2>{1, 6}, {2, 7}, {3, 8}, {4, 9}}) {
        const RagBody &u = rag_[size_t(pr[0])], &l = rag_[size_t(pr[1])];
        const V3 cur = unit(sub(posOf(w[size_t(l.bone)]), posOf(w[size_t(u.bone)])));
        rotateBone(w, u.bone, fromTo(cur, unit(sub(l.x, u.x))));
        w = worldOf(local);
    }
    // Place: body 0 at its position.
    const V3 off = sub(rag_[0].x, posOf(w[size_t(rag_[0].bone)]));
    world_.resize(w.size());
    for (size_t b = 0; b < w.size(); ++b) {
        Mat4 m = w[b];
        m[12] += off[0], m[13] += off[1], m[14] += off[2];
        world_[b] = m;
    }
}

// ---- PK31: AI racer lean (0x61c0c0 / 0x61c1b0 / 0x61c150); the x87 temporaries are doubles, the stores floats.
void AiLean::spring(float& v, float& x, float dt, float a, float b, float rate, float limit) {
#pragma clang fp contract(off)  // x87 has no fused multiply-add (a singular J^T J must stay det == 0)
    v = float(double(v) - (double(a) * double(b) + double(x) * 10.0) * double(dt));
    // Math_ExpSmooth 0x4100b0 (target 0): within 1e-6 snap, else v += (0 - v)(1 - 1 / rate^(30.30303 dt)).
    const float d = 0.0f - v;
    if (std::fabs(d) < 1e-6f) v = 0.0f;
    else v = float((1.0 - 1.0 / std::pow(double(rate), double(dt) * 30.30303)) * double(d) + double(v));
    float nx = float(double(v) * double(dt) + double(x));
    if (limit <= nx) nx = limit;  // fcom C0|C3: limit <= x
    if (!(-limit < nx)) nx = -limit;
    x = nx;
}

std::array<float, 4> AiLean::quatMul(const std::array<float, 4>& a, const std::array<float, 4>& b) {
#pragma clang fp contract(off)  // x87 has no fused multiply-add (a singular J^T J must stay det == 0)
    // 0x410260 operation order: ax bw and ay bw pass through float stores.
    const double t0 = float(double(a[0]) * b[3]), t1 = float(double(a[1]) * b[3]);
    const double w = double(b[3]) * a[3] - ((double(a[2]) * b[2] + double(b[1]) * a[1]) + double(a[0]) * b[0]);
    const double x = (double(a[2]) * b[1] - double(a[1]) * b[2]) + (double(b[0]) * a[3] + t0);
    const double y = (double(a[0]) * b[2] - double(a[2]) * b[0]) + (double(b[1]) * a[3] + t1);
    const double z = (double(a[1]) * b[0] - double(b[1]) * a[0]) + (double(b[2]) * a[3] + double(b[3]) * a[2]);
    return {float(x), float(y), float(z), float(w)};
}

void AiLean::step(float dt, const std::array<float, 3>& acc, const std::array<float, 4>& orient, bool bike) {
#pragma clang fp contract(off)  // x87 has no fused multiply-add (a singular J^T J must stay det == 0)
    const auto F = qrotate(orient, {0, 0, 1}), R = qrotate(orient, {1, 0, 0}), U = qrotate(orient, {0, 1, 0});  // 0x410510
    auto dotA = [&acc](const std::array<float, 3>& e) { return float((double(acc[2]) * e[2] + double(acc[1]) * e[1]) + double(acc[0]) * e[0]); };
    auto axisAngle = [](const std::array<float, 3>& ax, float deg) {  // 0x40f8b0 with deg * 0.017453292 (0x9dc18c)
        const float ang = float(double(deg) * double(0.017453292f));
        const double h = double(ang) * 0.5;
        const float s = float(std::sin(h)), c = float(std::cos(h));
        return std::array<float, 4>{ax[0] * s, ax[1] * s, s * ax[2], c};
    };
    // pitch about the right axis from the forward acceleration
    spring(v[0], x[0], dt, dotA(F), 20.0f, 1.04f, bike ? 3.5f : 2.0f);
    q = axisAngle(R, x[0]);
    // roll about the forward axis from the lateral acceleration (bikes lean into the turn, the others away, 6 deg)
    const float lat = dotA(R);
    if (bike) spring(v[1], x[1], dt, lat, 20.0f, 1.05f, 45.0f);
    else spring(v[1], x[1], dt, -lat, 18.0f, 1.05f, 6.0f);
    q = quatMul(axisAngle(F, x[1]), q);
    // yaw about the up axis
    spring(v[2], x[2], dt, -lat, 10.0f, 1.06f, 45.0f);
    q = quatMul(q, axisAngle(U, x[2]));
}

std::vector<RankReactions::Gesture> RankReactions::update(float dt, const std::vector<int>& order, const std::vector<Rec>& recs) {
    std::vector<Gesture> out;
    const size_t n = recs.size();
    if (rank_.size() != n) rank_.assign(n, -1), cool_.assign(n, 0.0f), list_.clear();
    for (size_t i = 0; i < n; ++i) cool_[i] -= dt;  // 0x5c1e..: +0x17c -= dt
    const std::vector<int> list = list_.empty() ? order : list_;
    const int count = int(list.size());
    for (int i = 0; i < count; ++i) {
        const int id = list[size_t(i)];
        if (id < 0 || size_t(id) >= n || !recs[size_t(id)].racing) continue;
        const int was = rank_[size_t(id)];
        rank_[size_t(id)] = i;
        if (was == -1 || i == was) continue;
        const int delta = i - was;
        const int nb = delta < 1 ? i + 1 : i - 1;
        if (nb < 0 || nb >= count) continue;
        // 0x5c2370 (ECX = the neighbour's record, arg 1 = this record, arg 2 = delta)
        const int other = list[size_t(nb)];
        if (other < 0 || size_t(other) >= n || cool_[size_t(id)] > 0.0f) continue;
        const Rec &me = recs[size_t(id)], &o = recs[size_t(other)];
        const float ex = o.x - me.x, ez = o.z - me.z;
        if (!(std::sqrt(ez * ez + ex * ex) < 25.0f)) continue;
        const float side = me.dx * ez + -me.dz * ex;
        const int base = side > 0 ? (delta > 0 ? 29 : 21) : (delta > 0 ? 33 : 25);
        out.push_back({id, base});
        cool_[size_t(id)] = 1.0f;
    }
    list_ = order;
    return out;
}
// ---- end PK31

}  // namespace fuel
