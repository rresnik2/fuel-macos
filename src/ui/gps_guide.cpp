// FUEL's GPS samples, floating trail and ribbon arrows; see gps_guide.hpp and notes/p40-work/gps.md.
#include "ui/gps_guide.hpp"

#include <algorithm>
#include <cmath>

namespace fuel::hud {
namespace {

using V3 = GpsGuide::V3;
float len2(float x, float z) { return std::sqrt(x * x + z * z); }

// Foot of (x, z) on a polyline: distance along it and the distance from it.
float project(const std::vector<std::array<float, 2>>& p, const std::vector<float>& along, float x, float z, float& off) {
    float best = 1e30f, s = 0;
    for (size_t i = 0; i + 1 < p.size(); ++i) {
        const float ax = p[i][0], az = p[i][1], dx = p[i + 1][0] - ax, dz = p[i + 1][1] - az;
        const float l = dx * dx + dz * dz;
        const float t = l > 0 ? std::clamp(((x - ax) * dx + (z - az) * dz) / l, 0.0f, 1.0f) : 0.0f;
        const float qx = ax + dx * t - x, qz = az + dz * t - z, d = qx * qx + qz * qz;
        if (d < best) best = d, s = along[i] + t * (along[i + 1] - along[i]);
    }
    off = std::sqrt(best);
    return s;
}

std::array<float, 2> pointAt(const std::vector<std::array<float, 2>>& p, const std::vector<float>& along, float s) {
    if (s <= 0) return p.front();
    const auto it = std::upper_bound(along.begin(), along.end(), s);
    if (it == along.end()) return p.back();
    const size_t i = size_t(it - along.begin()) - 1;
    const float seg = along[i + 1] - along[i], t = seg > 0 ? (s - along[i]) / seg : 0;
    return {p[i][0] + (p[i + 1][0] - p[i][0]) * t, p[i][1] + (p[i + 1][1] - p[i][1]) * t};
}

}  // namespace

float GpsGuide::fade(float x, float L) {
    if (x < 10) return 0;
    if (x < 40) return (x - 10) * 0.0333f;  // 1/30 as stored
    if (x <= L - 50) return 1;
    if (x <= L) return (L - x) * 0.02f;
    return 0;
}

float GpsGuide::advance(float d, float dt, bool* wrapped) {
    d += 40.0f * dt;  // 40 m/s
    if (wrapped) *wrapped = d > 300.0f;
    while (d > 300.0f) d -= 300.0f;
    return d;
}

void GpsGuide::reset() {
    samples_.clear(), trail_.clear(), prevA_.clear(), trailAlong_.clear(), d_.clear(), tails_.clear(), ribbons_.clear();
}

V3 GpsGuide::at(float s) const {
    if (trail_.empty()) return {0, 0, 0};
    if (s <= 0) return trail_.front();
    const auto it = std::upper_bound(trailAlong_.begin(), trailAlong_.end(), s);
    if (it == trailAlong_.end()) return trail_.back();
    const size_t i = size_t(it - trailAlong_.begin()) - 1;
    const float seg = trailAlong_[i + 1] - trailAlong_[i], t = seg > 0 ? (s - trailAlong_[i]) / seg : 0;
    V3 r;
    for (int k = 0; k < 3; ++k) r[size_t(k)] = trail_[i][size_t(k)] + (trail_[i + 1][size_t(k)] - trail_[i][size_t(k)]) * t;
    return r;
}

void GpsGuide::update(float dt, const V3& player, const V3& eye, const V3& camForward, const std::vector<std::array<float, 2>>& path,
                      const std::vector<float>& along, const std::function<float(float, float)>& ground) {
    if (path.size() < 2 || along.size() != path.size()) {
        reset();
        return;
    }
    // ---- 0x4a2250: 30 samples 10 m apart (straight back to the path first when > 10 m off it).
    float off = 0;
    const float s0 = project(path, along, player[0], player[2], off);
    std::vector<V3> S;
    if (off > 10.0f) {
        const auto r = pointAt(path, along, s0);
        const float dx = r[0] - player[0], dz = r[1] - player[2], l = len2(dx, dz);
        for (int k = 0; float(k) * 10.0f < l - 5.0f && S.size() < 30; ++k)
            S.push_back({player[0] + dx / l * 10.0f * float(k), 0, player[2] + dz / l * 10.0f * float(k)});
    }
    for (int k = 0; S.size() < 30 && s0 + 10.0f * float(k) <= along.back(); ++k) {
        const auto q = pointAt(path, along, s0 + 10.0f * float(k));
        S.push_back({q[0], 0, q[1]});
    }
    if (S.empty()) {
        reset();
        return;
    }
    for (V3& q : S) q[1] = ground(q[0], q[2]);
    for (size_t i = 1; i + 1 < S.size(); ++i) {  // single-sample dips filled (0x4a3596)
        const float m = (S[i - 1][1] + S[i + 1][1]) * 0.5f;
        if (S[i][1] < S[i - 1][1] && m > S[i][1]) S[i][1] = m;
    }
    // ---- 0x4bc0a0: smoothing toward the previous trail list A, in place, by dt.
    if (prevA_.size() > 3) {
        std::vector<std::array<float, 2>> a2;
        std::vector<float> aAlong{0};
        for (size_t i = 0; i < prevA_.size(); ++i) {
            a2.push_back({prevA_[i][0], prevA_[i][2]});
            if (i) aAlong.push_back(aAlong.back() + len2(prevA_[i][0] - prevA_[i - 1][0], prevA_[i][2] - prevA_[i - 1][2]));
        }
        float d0 = 0;
        const float a0 = project(a2, aAlong, S[0][0], S[0][2], d0);
        if (d0 < 20.0f && a0 < 10.0f)
            for (V3& q : S) {
                float d = 0;
                const float a = project(a2, aAlong, q[0], q[2], d);
                if (d > 20.0f) continue;
                const auto p = pointAt(a2, aAlong, a);
                q[0] = p[0] + (q[0] - p[0]) * std::min(dt, 1.0f);
                q[2] = p[1] + (q[2] - p[1]) * std::min(dt, 1.0f);
            }
    }
    samples_ = S;
    prevA_ = S;
    std::vector<float> aAlong{0};
    for (size_t i = 1; i < S.size(); ++i) aAlong.push_back(aAlong.back() + len2(S[i][0] - S[i - 1][0], S[i][2] - S[i - 1][2]));
    const float L = aAlong.back();
    auto A = [&](float s) {
        std::vector<std::array<float, 2>> p2;
        for (const V3& q : S) p2.push_back({q[0], q[2]});
        const auto xz = pointAt(p2, aAlong, s);
        // height: linear between samples
        const auto it = std::upper_bound(aAlong.begin(), aAlong.end(), s);
        float y = S.back()[1];
        if (s <= 0) y = S[0][1];
        else if (it != aAlong.end()) {
            const size_t i = size_t(it - aAlong.begin()) - 1;
            const float seg = aAlong[i + 1] - aAlong[i];
            y = S[i][1] + (S[i + 1][1] - S[i][1]) * (seg > 0 ? (s - aAlong[i]) / seg : 0);
        }
        return V3{xz[0], y, xz[1]};
    };
    constexpr float kLift = 7.5f;  // 0x9dcefc
    std::vector<V3> B{{S[0][0], S[0][1] + kLift, S[0][2]}};
    V3 P = A(35.0f);  // 0x9dc788
    P[1] += kLift;
    const float Dx = P[0] - B[0][0], Dz = P[2] - B[0][2], h = len2(Dx, Dz);
    if (h > 200.0f && !trail_.empty()) {
        // keep the old trail
    } else {
        float cx = camForward[0], cz = camForward[2];
        const float cl = len2(cx, cz);
        if (cl > 1e-6f) cx /= cl, cz /= cl;
        else cx = Dx / std::max(h, 1e-6f), cz = Dz / std::max(h, 1e-6f);
        const float dnx = h > 1e-6f ? Dx / h : cx, dnz = h > 1e-6f ? Dz / h : cz;
        const float angle = std::acos(std::clamp(cx * dnx + cz * dnz, -1.0f, 1.0f));
        V3 last;
        if (angle <= 0.1f) {
            last = {S[0][0] + 25.0f * cx, P[1], S[0][2] + 25.0f * cz};  // 0x996184
            B.push_back(last);
        } else {
            const float a0 = std::atan2(cz, cx);
            float da = std::atan2(dnz, dnx) - a0;
            while (da > float(M_PI)) da -= 2 * float(M_PI);
            while (da < -float(M_PI)) da += 2 * float(M_PI);
            for (int k = 0; k < 4; ++k) {
                const float t = 0.25f * float(k);
                const float r = 25.0f + (h - 25.0f) * t;
                const float a = a0 + da * t;  // slerp in the plane (0x644e00)
                V3 q{S[0][0] + std::cos(a) * r, 0, S[0][2] + std::sin(a) * r};
                q[1] = std::max({P[1] - 1.5923567f * angle * t, S[0][1], ground(q[0], q[2]) + 5.0f});
                B.push_back(q);
                last = q;
            }
        }
        int j = 0;
        for (float a = 45.0f; a < L; a += 10.0f, ++j) {
            const float w = std::max(0.8f - 0.2f * float(j), 0.0f);
            V3 q = A(a);
            q[1] += kLift;
            for (int k = 0; k < 3; ++k) q[size_t(k)] += w * (last[size_t(k)] - P[size_t(k)]);
            B.push_back(q);
        }
        trail_ = B;
        trailAlong_.assign(1, 0.0f);
        for (size_t i = 1; i < trail_.size(); ++i)
            trailAlong_.push_back(trailAlong_.back() + len2(trail_[i][0] - trail_[i - 1][0], trail_[i][2] - trail_[i - 1][2]));
    }
    // ---- 0x4bca50: 16 ribbons, starting at -(i/16) x 300 m (0x4bc000); each a SpecialEffectsRibbon_G of 4 points
    // (ctor 0x5deef0) that 0x5def80 puts all on the head at emission.
    if (d_.size() != 16) {
        d_.resize(16);
        tails_.assign(16, {});
        for (size_t i = 0; i < 16; ++i) {
            d_[i] = -float(i) / 16.0f * 300.0f;
            tails_[i].fill(at(d_[i]));
        }
    }
    const float Lp = std::min(300.0f, trailAlong_.empty() ? 0.0f : trailAlong_.back());  // 0x4101c0 min(300, L)
    ribbons_.clear();
    auto sub = [](const V3& a, const V3& b) { return V3{a[0] - b[0], a[1] - b[1], a[2] - b[2]}; };
    auto flatNorm = [](V3 v) {  // Vec_FlattenNormalize 0x40fc80
        v[1] = 0;
        const float l = std::sqrt(v[0] * v[0] + v[2] * v[2]);
        return l > 1e-6f ? V3{v[0] / l, 0, v[2] / l} : v;
    };
    for (size_t r = 0; r < d_.size(); ++r) {
        float& d = d_[r];
        // The fades read the head before it moves (pfVar9[-3..-1] is the previous frame's position).
        const V3 old = tails_[r][0];
        const V3 po = sub(old, player);
        const float dp = std::sqrt(po[0] * po[0] + po[1] * po[1] + po[2] * po[2]);  // 0x4106f0
        bool wrapped = false;
        d = advance(d, dt, &wrapped);
        float alpha = std::min(fade(dp, Lp), fade(d, Lp));  // particle +0x70
        const V3 pos = at(d), ahead = at(d + 10.0f);
        auto& pts = tails_[r];
        if (wrapped) {  // the tail laid back along the travel direction, +0x80 = 2 m apart
            V3 n = sub(ahead, pos);
            const float l = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
            if (l > 1e-6f) n = {n[0] / l, n[1] / l, n[2] / l};
            for (size_t k = 0; k < pts.size(); ++k)
                for (int c = 0; c < 3; ++c) pts[k][size_t(c)] = pos[size_t(c)] - n[size_t(c)] * 2.0f * float(k);
        }
        // Bank (+0xbc): world up turned about the flat travel direction by the angle between it and the player->arrow
        // direction (0x638530 acos), signed by the side the arrow passes the camera on, clamped to +-pi/2.
        const V3 t = flatNorm(sub(ahead, pos));
        const V3 pp = flatNorm(sub(pos, player));
        float angle = 0;
        const float lt = t[0] * t[0] + t[2] * t[2], lp = pp[0] * pp[0] + pp[2] * pp[2];
        if (lt > 1e-12f && lp > 1e-12f) angle = std::acos(std::clamp(t[0] * pp[0] + t[2] * pp[2], -1.0f, 1.0f));
        const V3 N{-pp[2], 0, pp[0]};
        const V3 ac = sub(ahead, eye), pc = sub(pos, eye);
        if (ac[0] * N[0] + ac[2] * N[2] < pc[0] * N[0] + pc[2] * N[2]) angle = -angle;
        angle = std::clamp(angle, -float(M_PI) * 0.5f, float(M_PI) * 0.5f);
        const float ca = std::cos(angle), sa = std::sin(angle);
        const V3 up{-t[2] * sa, ca, t[0] * sa};  // (0, 1, 0) cos + (t x (0, 1, 0)) sin
        // Seen flat on, near the player: alpha x max(1 - 4 clamp(|up . view| - 0.1) (1 - clamp(0.02 dist)), 0.5).
        V3 v = sub(pos, eye);
        const float lv = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
        if (lv > 1e-6f) v = {v[0] / lv, v[1] / lv, v[2] / lv};
        const float face = std::clamp(std::fabs(up[0] * v[0] + up[1] * v[1] + up[2] * v[2]) - 0.1f, 0.0f, 1.0f);
        const float near = std::clamp(dp * 0.02f, 0.0f, 1.0f);
        alpha *= std::max(1.0f - face * (1.0f - near) * 4.0f, 0.5f);
        // SpecialEffectsRibbon_G vf01 (0x5defc0): the head takes the position, each next point is pulled to within
        // +0x80 = 2 m of the one before it once it is more than sqrt(+0x7c) = 2 m away.
        pts[0] = pos;
        for (size_t k = 1; k < pts.size(); ++k) {
            const V3 e = sub(pts[k - 1], pts[k]);
            const float q = e[0] * e[0] + e[1] * e[1] + e[2] * e[2];
            if (q > 4.0f) {
                const float s = 2.0f / std::sqrt(q);
                for (int c = 0; c < 3; ++c) pts[k][size_t(c)] = pts[k - 1][size_t(c)] - e[size_t(c)] * s;
            }
        }
        if (alpha <= 0) continue;
        Ribbon rb;
        rb.alpha = alpha;
        rb.up = up;
        rb.points.assign(pts.begin(), pts.end());
        ribbons_.push_back(std::move(rb));
    }
}

}  // namespace fuel::hud
