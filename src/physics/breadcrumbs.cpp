// FUEL's respawn breadcrumbs, ported 1:1 from FUEL.exe by P70b (see breadcrumbs.hpp and
// ~/re-work/fuel/notes/p70-port-decoded.md "Breadcrumbs"). Checked by fuelphysvec "BC*" (EmuRun of
// 0x5281c0, 0x5283b0, 0x528830 and 0x5288d0 on synthetic states).
#include "physics/breadcrumbs.hpp"

#include <cmath>

namespace fuel::fv {

namespace {

// The engine's horizontal distance: the squared sum stored as a float, then 0x40ed20 (sqrt).
float hdist(double ax, double az, double bx, double bz) {
    const float dx = float(ax - bx), dz = float(az - bz);
    return std::sqrt(dz * dz + dx * dx);
}

// Vec_FlattenNormalize 0x40fc80: y = 0 and unit length when x^2 + z^2 > 1e-6; otherwise unchanged.
void flattenNormalize(float v[3]) {
    const float l2 = v[0] * v[0] + v[2] * v[2];
    if (l2 > 1e-6f) {
        const float inv = 1.0f / std::sqrt(l2);
        v[0] *= inv, v[1] = 0, v[2] *= inv;
    }
}

}  // namespace

float Breadcrumbs::yawOf(float x, float z) {
    // Quat_FromArc 0x6443a0 with from = (0, 0, 1) [0xa06350]: identity when |to| < 1e-6.
    if (std::sqrt(x * x + z * z) < 1e-6f) return 0.0f;
    return std::atan2(x, z);
}

bool Breadcrumbs::worldOutside(double x, double z, bool tutorial) {  // 0x5280a0
    if (!tutorial) return std::fabs(float(x)) > 65500.0f || std::fabs(float(z)) > 65500.0f;
    // Tutorial (game +0x32bc == 2): the box centred (-50875, -59875), half size 2625.
    return std::fabs(float(x) + 50875.0f) > 2625.0f || std::fabs(float(z) + 59875.0f) > 2625.0f;
}

bool Breadcrumbs::onRoad(const double p[3], const CrumbWorld& w, float dir[3]) {  // 0x527f50
    double px = 0, pz = 0;  // no covering segment: 0x6fd3a0 returns the zero vector [0xa66208], dir untouched
    if (w.roadCentreline) w.roadCentreline(p[0], p[2], px, pz, dir);
    return hdist(p[0], p[2], px, pz) < 20.0f;  // [0x996154]
}

bool Breadcrumbs::safe(const double p[3], const CrumbVehicle& v, const CrumbWorld& w) {  // 0x5281c0
    if (worldOutside(v.pos[0], v.pos[2], w.tutorialBounds)) return false;
    if (v.upY < 0.0f) return false;
    if (!v.allWheelsLoaded) return false;
    if (v.velY < -30.0f) return false;           // [0x9dd264]
    if (v.probeNormalY < 0.819f) return false;   // [0x9dd260]
    if (v.fwd[1] > 0.5f) return false;           // [0x9960c0]
    if (v.fwd[1] < -0.707f) return false;        // [0x9dcff0]
    if (v.maxWaterDepth > 0.0f) return false;
    if (w.blocked && w.blocked(p, v.radius + 2.0f)) return false;  // 0x527bf0: +0x404 + 2 [0x99601c]
    if (w.waterDepth && w.waterDepth(p[0], p[2]) > 0.1f) return false;  // 0x40fd10 [0x99648c]
    if (hdist(p[0], p[2], v.pos[0], v.pos[2]) > 1.0f) {  // [0x9963b8]
        const float h = w.baseHeight ? w.baseHeight(p[0], p[2]) : 0.0f;  // 0x445500 (else veh +0x13b4)
        if (float(p[1]) - h > 50.0f) return false;  // [0x9dc048]
    }
    return true;
}

void Breadcrumbs::recordAt(const double p[3], const CrumbVehicle& v, const CrumbWorld& w) {  // 0x5283b0
    if (!list_.empty()) {
        const Crumb& t = list_.back();
        const float d = hdist(p[0], p[2], t.x, t.z);
        if (d < 10.0f) return;  // [0x996494]
        if (!safe(p, v, w)) return;
        if (d > 1000.0f) list_.clear();  // [0x996374]: Mem::FreeMembers 0x52f620
    }
    Crumb c;  // 0x52f6a0: appended as the new tail
    c.x = p[0], c.y = p[1], c.z = p[2];
    float dir[3] = {0, 0, 0};
    c.onRoad = onRoad(p, w, dir);
    flattenNormalize(dir);
    double tx = 0, tz = 0;
    if (w.raceTarget && w.raceTarget(tx, tz)) {  // 0x528690: target - p (y = 0), straight to Quat_FromArc
        c.yaw = yawOf(float(tx - p[0]), float(tz - p[2]));
    } else {
        if (!(w.gpsTarget && w.gpsTarget(tx, tz))) {  // 0x421dc0, else 100 m [0x9dbfc0] ahead of the crumb
            tx = c.x + double(v.fwd[0] * 100.0f);
            tz = c.z + double(v.fwd[2] * 100.0f);
        }
        float to[3] = {float(tx - v.pos[0]), 0, float(tz - v.pos[2])};  // from the body position
        flattenNormalize(to);
        if (c.onRoad) {
            if (to[1] * dir[1] + to[2] * dir[2] + to[0] * dir[0] < 0.0f) dir[0] = -dir[0], dir[1] = -dir[1], dir[2] = -dir[2];
            c.yaw = yawOf(dir[0], dir[2]);
        } else {
            c.yaw = yawOf(to[0], to[2]);
        }
    }
    list_.push_back(c);
}

void Breadcrumbs::record3(const CrumbVehicle& v, const CrumbWorld& w) {  // 0x528780
    const double p0[3] = {v.pos[0], v.pos[1], v.pos[2]};
    recordAt(p0, v, w);
    const float lx = v.lat[0] * 5.0f, ly = v.lat[1] * 5.0f, lz = v.lat[2] * 5.0f;  // [0x9dbf8c]
    const double p1[3] = {p0[0] + lx, p0[1] + ly, p0[2] + lz};
    recordAt(p1, v, w);
    const double p2[3] = {p0[0] - lx, p0[1] - ly, p0[2] - lz};
    recordAt(p2, v, w);
}

void Breadcrumbs::tick(float dt, const CrumbVehicle& v, const CrumbWorld& w) {  // 0x528830
    if (!list_.empty()) {
        timer_ -= dt;
        if (timer_ > 0.0f) return;
        timer_ = 1.0f;
    }
    record3(v, w);
}

bool Breadcrumbs::pick(float minDist, const CrumbVehicle& v, Crumb& out) {  // 0x5288d0
    if (list_.empty()) return false;
    const size_t n = list_.size();
    auto choose = [&](size_t i) {  // 0x527af0: pop the tail until crumb i is the tail
        list_.resize(i + 1);
        out = list_[i];
        return true;
    };
    if (hdist(v.pos[0], v.pos[2], list_[n - 1].x, list_[n - 1].z) > minDist) return choose(n - 1);
    const bool tailOnRoad = list_[n - 1].onRoad;
    float farD = 0;
    size_t far = n;
    for (size_t i = n - 1; i-- > 0;) {  // older crumbs, newest first (prev links +0x20)
        const float d = hdist(v.pos[0], v.pos[2], list_[i].x, list_[i].z);
        if (d > minDist) return choose(i);
        if (farD < d) farD = d, far = i;
        if (!tailOnRoad && list_[i].onRoad) return choose(i);
    }
    if (far < n) return choose(far);
    return false;
}

// ---- PK01 ----

void quatFromArc(const float from[3], const float to[3], float out[4]) {  // 0x6443a0
    const double la = std::sqrt(double(from[0]) * from[0] + double(from[1]) * from[1] + double(from[2]) * from[2]);
    const double lb = std::sqrt(double(to[0]) * to[0] + double(to[1]) * to[1] + double(to[2]) * to[2]);
    out[0] = out[1] = out[2] = 0, out[3] = 1;
    if (!(la >= 1e-6 && lb >= 1e-6)) return;
    const double a[3] = {from[0] / la, from[1] / la, from[2] / la}, b[3] = {to[0] / lb, to[1] / lb, to[2] / lb};
    const double c = a[0] * b[0] + a[1] * b[1] + a[2] * b[2];
    // Both of FUEL's branches (half vector a + b when c >= 0, a - b otherwise) give the half-angle rotation
    // about a x b; opposite vectors: a half turn about a perpendicular axis (0x40f9a0).
    double v[3] = {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]};
    double s = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    double w;
    if (s < 1e-9) {
        if (c > 0) return;
        v[0] = std::fabs(a[0]) < 0.9 ? 0 : -a[1], v[1] = std::fabs(a[0]) < 0.9 ? -a[2] : a[0], v[2] = std::fabs(a[0]) < 0.9 ? a[1] : 0;
        s = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
        w = 0;
        for (double& x : v) x /= s;
    } else {
        const double ang = std::atan2(s, c) * 0.5;
        for (double& x : v) x = x / s * std::sin(ang);
        w = std::cos(ang);
    }
    out[0] = float(v[0]), out[1] = float(v[1]), out[2] = float(v[2]), out[3] = float(w);
}

void respawnOrient(float q[4], float posY, float terrainY, const float n[3], bool worldFlag) {  // 0x529480
    if (worldFlag) return;
    if (!(std::fabs(posY - terrainY) < 1.0f)) return;  // 0x529561..0x529577: 1 > |d|
    // 0x410510(q, (0, 0, 1) [0xa06350]), y = 0.
    const double x = q[0], y = q[1], z = q[2], w = q[3];
    const float f[3] = {float(2 * (x * z + w * y)), 0.0f, float(1 - 2 * (x * x + y * y))};
    static const float kZ[3] = {0, 0, 1}, kY[3] = {0, 1, 0};  // 0xa06350, 0xa06320
    float yaw[4], tilt[4];
    quatFromArc(kZ, f, yaw);
    quatFromArc(kY, n, tilt);
    // 0x410260 (ECX = yaw, EDX = tilt): the Hamilton product tilt * yaw.
    const float r[4] = {tilt[3] * yaw[0] + tilt[0] * yaw[3] + tilt[1] * yaw[2] - tilt[2] * yaw[1],
                        tilt[3] * yaw[1] - tilt[0] * yaw[2] + tilt[1] * yaw[3] + tilt[2] * yaw[0],
                        tilt[3] * yaw[2] + tilt[0] * yaw[1] - tilt[1] * yaw[0] + tilt[2] * yaw[3],
                        tilt[3] * yaw[3] - tilt[0] * yaw[0] - tilt[1] * yaw[1] - tilt[2] * yaw[2]};
    for (int k = 0; k < 4; ++k) q[k] = r[k];
}

}  // namespace fuel::fv
