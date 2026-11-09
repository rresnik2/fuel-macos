// FUEL's vehicle headlights; see vehicle_lights.hpp and notes p77-shadows-lights.md 2.
#include "core/vehicle_lights.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace fuel::omni {

bool parseParams(const std::vector<uint8_t>& body, Params& out) {
    if (body.size() < 0xc0) return false;
    auto f = [&](size_t objOffset) {
        float v;
        std::memcpy(&v, body.data() + (objOffset - 0x6c), 4);
        return v;
    };
    out.colour = {f(0xfc), f(0x100), f(0x104)};
    out.cutZ = f(0x11c), out.cutW = f(0x120);
    out.inner = f(0x124), out.outer = f(0x128);
    return true;
}

// ---- PK28: 0x7984c0 / 0x821830 / 0x6a2ed0 / 0x7aabe0 (see the header)
bool parseObjectInfo(const std::vector<uint8_t>& link, const std::vector<uint8_t>& body, ObjectInfo& out) {
    if (link.size() < 96 || body.size() < 0xc0) return false;
    std::memcpy(&out.flags, link.data() + 92, 4);
    std::memcpy(&out.bound, link.data() + 20, 4);
    std::memcpy(&out.inner, body.data() + (0x114 - 0x6c), 4);
    std::memcpy(&out.outer, body.data() + (0x118 - 0x6c), 4);
    return true;
}

Record makeRecord(const ObjectInfo& o, const Params& p, const V3& translation, const V3& axis, float scale) {
    Record r;
    r.pos = translation;
    r.axis = axis;
    const float rs = p.inner * scale;  // fld [Omni +0x124]; fmul [node +0x44]; fmul st, st
    r.range2 = rs * rs;
    r.radius = o.bound * scale;
    r.spot = (o.flags & kOmniSpot) != 0;
    if (r.spot) {
        const float sn = std::sin(o.outer), cs = std::cos(o.outer);  // 0x40f3d0 / 0x40f3f0 of +0x118
        r.invSin = 1.0f / sn;
        r.sin2 = sn * sn;
        r.cos2 = cs * cs;
        r.dir = axis;
    } else {
        r.dir = {0, 0, 0};  // 0xa66208..0xa66210: .bss, never written (I: zero)
    }
    return r;
}

bool sphereVisible(const V3& c, float r, const std::array<V3, 4>& planes, float zNear, float zFar) {
    if (!(zNear - c[2] <= r)) return false;  // fVar3 = near - z: continue when fVar3 <= r
    if (!(-r <= zFar - c[2])) return false;  // fVar4 = far - z: continue when -r <= fVar4
    for (const V3& n : planes)
        if (!(-r <= n[0] * c[0] + n[1] * c[1] + n[2] * c[2])) return false;
    return true;
}

uint32_t rtcSwitch(uint32_t flags, bool on) {
    if ((flags & kOmniRtcKeep) != 0 || (flags & kOmniRtcSwitch) == 0) return flags;
    return on ? (flags | kOmniGathered) : (flags & ~kOmniGathered);
}
// ---- PK28 end

std::array<float, 4> value(const Params& p, float s) {
    // 0x798050: r = +0x124 s, R = +0x128 s; (r^2, 1 / (R^2 - r^2), +0x11c, +0x120).
    const float r2 = (p.inner * s) * (p.inner * s), R = p.outer * s;
    return {r2, 1.0f / (R * R - r2), p.cutZ, p.cutW};
}

float dayFactor(double t) {
    const double d = std::fmod(t, 86400.0);  // 0x5b98d0 (fmod by 0x9dc3e0)
    if (d <= 19800.0) return 0.0f;
    if (d < 23400.0) return float((d - 19800.0) / 3600.0);
    if (d <= 63000.0) return 1.0f;
    if (d <= 66600.0) return float(1.0 - (d - 63000.0) / 3600.0);
    return 0.0f;
}

V3 term(const V3& P, const V3& N, const V3& p, const V3& dir, const std::array<float, 4>& val, const V3& colour, const V3& viewUp) {
    auto dot = [](const V3& a, const V3& b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; };
    auto sat = [](float x) { return std::clamp(x, 0.0f, 1.0f); };
    const V3 o{p[0] - P[0], p[1] - P[1], p[2] - P[2]};  // omnipos = gDOmniPos - fragment
    const float d = dot(o, o);
    const float inv = 1.0f / std::sqrt(d);
    const V3 v{o[0] * inv, o[1] * inv, o[2] * inv};
    const float f = dot(dir, v);
    float n = sat(dot(dir, N));
    if (f < 0) n = sat(dot(v, N));
    float r = sat(1.0f - (d - val[0]) * val[1]);
    r *= r, r *= r, r *= r;
    const float up = sat(0.33f - dot(v, viewUp) * 0.67f - d * 0.5f);
    const float c = 2.0f * up + (1.0f - up * 2.0f) * sat(1.0f - (f * val[2] + val[3]));  // BACKOMNIPOWER 2
    const float k = r * n * c;  // x gDOmniPos.w = 1
    return {k * colour[0], k * colour[1], k * colour[2]};
}

}  // namespace fuel::omni
