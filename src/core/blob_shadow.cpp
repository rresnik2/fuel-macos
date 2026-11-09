// FUEL's blob shadows (FakeShadows_G); see blob_shadow.hpp and notes p77-shadows-lights.md 1.
#include "core/blob_shadow.hpp"

#include <algorithm>
#include <cmath>

namespace fuel::blob {
namespace {

V3 cross(const V3& a, const V3& b) { return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]}; }
V3 scale(const V3& a, float s) { return {a[0] * s, a[1] * s, a[2] * s}; }
V3 normalized(const V3& a) {
    const float l = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
    return l > 0 ? scale(a, 1.0f / l) : a;
}
V3 rotate(const std::array<float, 4>& q, const V3& v) {
    // v + 2 w (u x v) + 2 u x (u x v), u = q.xyz
    const V3 u{q[0], q[1], q[2]};
    const V3 t = scale(cross(u, v), 2.0f);
    const V3 c = cross(u, t);
    return {v[0] + q[3] * t[0] + c[0], v[1] + q[3] * t[1] + c[1], v[2] + q[3] * t[2] + c[2]};
}
float clamp01(float x) { return x <= 0 ? 0.0f : (x < 1 ? x : 1.0f); }

}  // namespace

Record makeRecord(const V3& p, const V3& n, const V3& f, float side, float fwd, float alpha) {
    Record r;
    // 0x509c00: centre = p + n x 0.01 [0x99614c]; sizes and alpha as given.
    r.centre = {p[0] + n[0] * 0.01f, p[1] + n[1] * 0.01f, p[2] + n[2] * 0.01f};
    r.side = side, r.fwd = fwd, r.alpha = alpha;
    // 0x509cb0: v = f flattened (y 0, xz normalised) unless f is (nearly) vertical; b = normalize(-(n x v)) x side;
    // a = normalize(n x b) x fwd.
    V3 v = f;
    const float h = f[0] * f[0] + f[2] * f[2];
    if (h > 1e-6f) {
        const float l = std::sqrt(h);
        v = {f[0] / l, 0.0f, f[2] / l};
    }
    const V3 nv = cross(n, v);
    r.b = scale(normalized({-nv[0], -nv[1], -nv[2]}), side);
    r.a = scale(normalized(cross(n, r.b)), fwd);
    return r;
}

float alphaFactor(float invIntensity, float fog) {
    float l = 1.0f - invIntensity;  // 0x5cba4f: 0.3 [0x9dbfa4] .. 1
    l = l > 0.3f ? (l < 1.0f ? l : 1.0f) : 0.3f;
    float w = 1.0f - fog;  // 0x5cba9e: 0.2 [0x9dbfa8] .. 1
    w = w > 0.2f ? (w < 1.0f ? w : 1.0f) : 0.2f;
    return l * w;
}

V3 nudge(const V3& P, const V3& E, const V3& up) {
    const V3 d = normalized({E[0] - P[0], E[1] - P[1], E[2] - P[2]});
    const float dy = std::fabs(d[0] * up[0] + d[1] * up[1] + d[2] * up[2]);
    const float k = (1.0f - dy) * (1.0f - dy) * 0.15f + 0.05f;
    return {P[0] + d[0] * k, P[1] + d[1] * k, P[2] + d[2] * k};
}

std::array<float, 2> bodySizes(float hx, float hz, const std::vector<V3>& w) {
    // +0x2e98 = 2 min(hx, hz), +0x2e9c = 2 max(hx, hz) [0x99601c]; x 0.75 [0x9960b8]; then the wheel geometry.
    std::array<float, 2> s{2.0f * std::min(hx, hz) * 0.75f, 2.0f * std::max(hx, hz) * 0.75f};
    if (w.size() >= 2) s[1] = std::fabs(w[0][2] - w[1][2]);  // +0x1260 - +0x1270 (0x5ca7a0 reads w1 even for 1 wheel)
    if (w.size() > 2) s[0] = std::fabs(w[0][0] - w[2][0]);   // +0x1258 - +0x1278
    return s;
}

void Producer::update(int vehicle, float dt, int raceState, const VehicleInput& in, std::vector<Record>& out) {
    if (vehicle < 0) return;
    if (size_t(vehicle) >= fades_.size()) fades_.resize(size_t(vehicle) + 1);
    Fades& F = fades_[size_t(vehicle)];
    if (in.player) return;  // 0x5d86b0 is skipped and 0x5d8420 returns for +0x1a48 (fades untouched)
    const bool intro = raceState >= 2 && raceState <= 4;  // g_Game+0x334c +0x48 in 2..4
    const V3 up = rotate(in.quat, {0, 1, 0});       // S+0x2c
    const V3 forward = rotate(in.quat, {0, 0, 1});  // S+0x14
    const int n = int(std::min<size_t>(in.contacts.size(), 16));
    auto load = [&](int i) { return size_t(i) < in.loads.size() ? in.loads[size_t(i)] : 0.0f; };

    // 0x5d86b0 per wheel (+0x19d9 / +0x19d8 / the node's world: set in the port).
    for (int i = 0; i < n; ++i) {
        if ((!intro && in.cameraDistance > 20.0f) || (!(load(i) > 1e-6f) && !intro)) {
            F.wheel[size_t(i)] = 0;
            continue;
        }
        const V3& c = in.contacts[size_t(i)];
        const V3 p{c[0], c[1] + 0.02f, c[2]};  // + up axis x 0.02 [0x9dc018] + right x +0x1630[i] (0 in the port)
        F.wheel[size_t(i)] = clamp01(dt * 3.0f + F.wheel[size_t(i)]);
        const V3 f = size_t(i) < in.headings.size() ? in.headings[size_t(i)] : forward;  // q x +0x12b8[i]
        const int odd = (i & 1) != 0 ? 1 : 0;
        const float side = in.pp[size_t(2 + odd)];             // VehiclePP 51 / 52
        const float fwd = in.pp[size_t(odd)] * 2.0f * 0.8f;    // VehiclePP 49 / 50 x 2 x 0.8 [0x9dc154]
        out.push_back(makeRecord(p, up, f, side, fwd, F.wheel[size_t(i)] * 0.5f));
    }

    // 0x5d8420.
    if (!intro && in.cameraDistance > 200.0f) {
        F.body = 0;
        return;
    }
    if (n < 2) return;
    if (!intro && load(1) < 1e-6f && load(0) < 1e-6f) {
        F.body = 0;
        return;
    }
    const int m = std::min(n, 4);
    V3 p{0, 0, 0};
    for (int i = 0; i < m; ++i)
        for (int k = 0; k < 3; ++k) p[size_t(k)] += in.contacts[size_t(i)][size_t(k)];
    for (float& v : p) v /= float(m);
    p[1] += 0.1f;  // + (0, 1, 0) x 0.1 [0x99648c]
    const float fwdFactor = n >= 4 ? 0.8f : 0.5f, alphaK = n >= 4 ? 1.0f : 0.6f;
    F.body = clamp01(dt * 3.0f + F.body);
    out.push_back(makeRecord(p, up, forward, in.bodySize[0] * 0.8f, in.bodySize[1] * fwdFactor, F.body * alphaK));
}

}  // namespace fuel::blob
