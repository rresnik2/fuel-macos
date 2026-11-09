#include "race_helicopter.hpp"

#include "anim_clip.hpp"
#include "physics/fuel_math.hpp"

#include <algorithm>
#include <cmath>

namespace fuel {

namespace {
using fv::Quat;
using fv::Vec3;

Vec3 v3(const RaceHelicopter::V3& a) { return {a[0], a[1], a[2]}; }
RaceHelicopter::V3 a3(const Vec3& v) { return {v.x, v.y, v.z}; }

// 0x410050: scale to length 1 when the squared length is above 1e-6, else untouched.
void normalize1(Vec3& v) {
    const float l2 = v.x * v.x + v.y * v.y + v.z * v.z;
    if (1e-6f < l2) v = v * float(1.0 / std::sqrt(double(l2)));
}
float hyp2(float x, float z) { return float(std::sqrt(double(x * x + z * z))); }
}  // namespace

RaceHelicopter::Q4 RaceHelicopter::fromArc(const V3& from, const V3& to) {  // 0x6443a0: EBX = from, EAX = to
    const double la = std::sqrt(double(from[0]) * from[0] + double(from[1]) * from[1] + double(from[2]) * from[2]);
    const double lb = std::sqrt(double(to[0]) * to[0] + double(to[1]) * to[1] + double(to[2]) * to[2]);
    Q4 out{0, 0, 0, 1};
    if (!(la >= 1e-6 && lb >= 1e-6)) return out;
    const double a[3] = {from[0] / la, from[1] / la, from[2] / la}, b[3] = {to[0] / lb, to[1] / lb, to[2] / lb};
    const double c = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    // The half-angle rotation about a x b (FUEL's two branches, a + b and a - b, agree); opposite vectors: a half
    // turn about a perpendicular axis (as fv::quatFromArc, PK01).
    double v[3] = {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
    double s = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    double w;
    if (s < 1e-9) {
        if (c > 0) return out;
        v[0] = std::fabs(a[0]) < 0.9 ? 0 : -a[1], v[1] = std::fabs(a[0]) < 0.9 ? -a[2] : a[0], v[2] = std::fabs(a[0]) < 0.9 ? a[1] : 0;
        s = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
        w = 0;
        for (double& x : v) x /= s;
    } else {
        const double ang = std::atan2(s, c) * 0.5;
        for (double& x : v) x = x / s * std::sin(ang);
        w = std::cos(ang);
    }
    return {float(v[0]), float(v[1]), float(v[2]), float(w)};
}

void RaceHelicopter::init(const V3& start, const V3& end, const Q4& startQuat, float pathLength, float timeA, const V3& missionStartPos) {
    quat = startQuat;
    missionStart = missionStartPos;
    missionEnd = end;
    disp = start;
    disp[1] = (ground ? ground(start[0], start[2]) : start[1]) + kStartUp;
    target = end;
    target[1] = (ground ? ground(end[0], end[2]) : end[1]) + kStartUp;
    requiredSpeed = prevSpeed = speed = timeA > 0 ? pathLength / timeA : 0;
    timeLeft = timeA;
    vel = {0, 0, 0};  // DAT_00a66208..10 (a runtime zero vector)
    dir = {0, 0, 0};
    waypoint = 0;
    pitchRaw = 0;
    rollRaw = 0.5f;
    phase = 0;
    pos = disp;
    forcePass = forcePass1 = false;
}

void RaceHelicopter::cruiseHeight() {  // 0x620530
    Vec3 d = v3(target) - v3(pos);
    normalize1(d);
    const float k = requiredSpeed + requiredSpeed;
    const float px = d.x * k + pos[0], pz = d.z * k + pos[2];
    const float g1 = ground ? ground(px, pz) : 0.0f, g2 = ground ? ground(pos[0], pos[2]) : 0.0f;
    if (!std::isfinite(g1) || !std::isfinite(g2)) return;
    cruise = (g1 > g2 ? g1 : g2) + kCruiseUp;
}

void RaceHelicopter::wobble(float dt) {  // 0x620370
    static const V3 kNegX{-1, 0, 0}, kY{0, 1, 0}, kZ{0, 0, 1};  // 0xa06344, 0xa06320, 0xa06350
    const Q4 q1 = fromArc(kZ, kNegX);
    const Vec3 side = fv::rotate({q1[0], q1[1], q1[2], q1[3]}, v3(dir));
    const Q4 q2 = fromArc(kZ, kY);
    const Vec3 fwd = fv::rotate({q2[0], q2[1], q2[2], q2[3]}, v3(dir));
    // fcos (0x40f3f0), not a sine: the code multiplies by cos(0.8 t) and cos(t).
    const float c08 = float(std::cos(double(phase * 0.8f))), c1 = float(std::cos(double(phase)));
    disp[0] = float(double(pos[0]) + double(side.x) * c1 + double(fwd.x * c08));
    disp[1] = float(double(side.y) * c1 + double(pos[1]) + double(fwd.y * c08));
    disp[2] = float(double(pos[2]) + double(side.z) * c1 + double(fwd.z * c08));
    phase = dt * 1.5f + phase;
}

void RaceHelicopter::waypointStep(int gateCount, const V3& recTarget) {  // 0x620460 (the 40 m part)
    const float dx = disp[0] - target[0], dz = disp[2] - target[2];
    if (hyp2(dz, dx) < kWaypoint) {
        if (waypoint < gateCount - 1) {
            ++waypoint;
            if ((recTarget[2] - target[2]) * (recTarget[2] - target[2]) + (recTarget[0] - target[0]) * (recTarget[0] - target[0]) < 25.0f)
                forcePass = forcePass1 = true;
        }
    }
}

void RaceHelicopter::velocity(float dt, const V3& player) {  // 0x620670
    Vec3 d{target[0] - disp[0], target[1] - disp[1], target[2] - disp[2]};
    dir = a3(d);
    if (dir[2] == dir[0]) return;  // the zero-vector guard (compares z with x)
    if (!(dt < timeLeft)) return;
    normalize1(d);
    dir = a3(d);
    const float dEnd = hyp2(pos[0] - missionEnd[0], pos[2] - missionEnd[2]);
    const float v = 0.1f < timeLeft ? float(double(dEnd) / timeLeft) : 40.0f;
    prevSpeed = speed;
    speed = v;
    if (dEnd <= kFar) {
        float c = v;
        if (c <= 10.0f) c = 10.0f;
        else if (40.0f < c) c = 40.0f;
        vel = {d.x * c, d.y * c, d.z * c};
    } else {
        const float ex = missionEnd[0] - player[0], ez = missionEnd[2] - player[2];
        const float hx = missionEnd[0] - pos[0], hz = missionEnd[2] - pos[2];
        const float dp = hyp2(ex, ez), dh = hyp2(hx, hz);
        const float lead = float(double(dp) - double(dh));
        float s = speed;
        if (lead <= kFar) s = lead <= kClose ? s + s : s * 0.5f;
        else s = s * 0.2f;
        speed = s;
        float best = v;
        if (best < speed) best = speed;
        speed = best;
        const Vec3 tgt = d * speed;
        const float dxv = tgt.x - vel[0], dyv = tgt.y - vel[1], dzv = tgt.z - vel[2];  // Cam_SmoothV3 0x40fb70
        if (dyv * dyv + dzv * dzv + dxv * dxv < 1e-6f) {
            vel = a3(tgt);
        } else {
            const double k = 1.0 - 1.0 / std::pow(double(kSmoothRate), double(dt * 30.30303f));
            vel = {float(double(dxv) * k + vel[0]), float(double(dyv) * k + vel[1]), float(double(dzv) * k + vel[2])};
        }
    }
    float vert = cruise - (vel[1] * dt + pos[1]);
    const float dStart = hyp2(pos[0] - missionStart[0], pos[2] - missionStart[2]);
    const float hi = kFar <= dStart ? 10.0f : 2.0f;
    if (vert < -10.0f) vert = -10.0f;
    else if (hi < vert) vert = hi;
    vel[1] = vert;
    Vec3 n = v3(vel);
    normalize1(n);
    vel = {n.x * speed, n.y * speed, n.z * speed};
    vel[1] = vert;
}

void RaceHelicopter::integrate(float dt) {
    pos[0] = pos[0] + vel[0] * dt;
    pos[1] = vel[1] * dt + pos[1];
    pos[2] = pos[2] + vel[2] * dt;
}

void RaceHelicopter::attitude(float dt) {  // 0x620980
    Vec3 D = v3(dir);
    normalize1(D);
    const float len = float(std::sqrt(double(D.x * D.x + D.y * D.y + D.z * D.z)));
    if (!(len > 0.0001f)) return;
    Vec3 U = v3(vel);
    normalize1(U);
    float roll = (D.x * U.x + D.y * U.y + D.z * U.z) * -0.7f;  // stored only
    if (D.x * U.z - D.z * U.x < 0.0f) roll = -roll;
    const float pitch = U.y * -0.2f;
    U.y = 0;
    normalize1(U);
    const float sm = fv::expSmooth(pitchRaw, pitch, 1.1f, dt);  // Math_ExpSmooth 0x4100b0: cur = last raw pitch
    pitchRaw = pitch;
    rollRaw = roll;
    const Q4 qA = fromArc({0, 0, 1}, a3(U));
    const float ang = float(std::cos(double(phase))) * 0.2f + sm;
    const Quat qB = fv::axisAngle({1, 0, 0}, ang);
    const Quat m = fv::mul(qB, {qA[0], qA[1], qA[2], qA[3]});  // Quat_Mul(ECX = qA, EDX = qB) = qB * qA
    // port_anim_slerp 0x431010 (t = 0.5)
    const anim::Quat o = anim::slerp({quat[0], quat[1], quat[2], quat[3]}, {m.x, m.y, m.z, m.w}, 0.5);
    quat = {float(o[0]), float(o[1]), float(o[2]), float(o[3])};
}

}  // namespace fuel
