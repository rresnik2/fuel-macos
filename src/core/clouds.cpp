// FUEL's billboard clouds (SpecialEffectsCloudTank_G); see clouds.hpp and notes p37-sky-clouds.md.
#include "clouds.hpp"

#include <algorithm>
#include <cmath>

namespace fuel::clouds {

namespace {
float clamp01(float v) { return std::min(1.0f, std::max(0.0f, v)); }
float pp(const std::vector<float>& v, int n, float fallback) { return n >= 0 && size_t(n) < v.size() && v[size_t(n)] != 0 ? v[size_t(n)] : fallback; }
}  // namespace

Params Params::fromPP(const std::vector<float>& v) {
    Params p;
    p.colour = pp(v, 5201, p.colour);
    p.heightCloudy = pp(v, 5202, p.heightCloudy);
    p.heightClear = pp(v, 5203, p.heightClear);
    p.sizeMin = pp(v, 5209, p.sizeMin);
    p.sizeMax = pp(v, 5208, p.sizeMax);
    p.spacing = pp(v, 5207, p.spacing);
    p.sight = pp(v, 5206, p.sight);
    return p;
}

V3 Tank::randomPosition(bool inside) {
    // 0x5fdbf0 (asm read): a = U x 6.2831855 - 3.1415927 (0x9dc258, 0x9a1550); radius U x sight (inside) or
    // sight + U (1.25 sight - sight) (the ring the camera has not reached); direction normalize(cos a - sin a, 0,
    // cos a + sin a) (0x40f3f0 cos, 0x40f3d0 sin, 0x410050 normalize); y = 0.
    const float a = well_() * 6.2831855f - 3.1415927f;
    const float sight = p_.sight, outer = sight * 1.25f;
    const float r = inside ? well_() * sight : sight + well_() * (outer - sight);
    const float c = std::cos(a), s = std::sin(a);
    float d[3] = {c - s, 0.0f, c + s};
    const float l = std::sqrt(d[0] * d[0] + d[2] * d[2]);
    d[0] /= l, d[2] /= l;
    return {pos_[0] + d[0] * r, 0.0f, d[2] * r + pos_[2]};
}

bool Tank::accept(const V3& q) const {
    // 0x5fde90: refused when an active cloud is closer than PP5207 (+0x84).
    for (const Cloud& c : clouds_) {
        if (!c.active) continue;
        const float dx = c.pos[0] - q[0], dy = c.pos[1] - q[1], dz = c.pos[2] - q[2];
        if (std::sqrt(dy * dy + dz * dz + dx * dx) < p_.spacing) return false;
    }
    return true;
}

void Tank::build(Cloud& c, const V3& q) {
    // 0x5fdf60 (asm read).
    c.pos = q;
    const float dx = pos_[0] - q[0], dy = pos_[1] - q[1], dz = pos_[2] - q[2];
    c.dist = std::sqrt(dz * dz + dy * dy + dx * dx);
    c.variant = std::clamp(int(std::floor(game_() * 4.0f)), 0, 3);  // ftol(rand x 4.0 0x996378), round down
    c.height = game_() * (p_.sizeMax - p_.sizeMin) + p_.sizeMin;    // +0x18 first
    c.width = game_() * (p_.sizeMax - p_.sizeMin) + p_.sizeMin;     // +0x14
    if (c.variant == 0) c.width *= 1.15f;                            // 0x9dcaf8
    c.active = true;
    c.mirror = game_() > 0.5f;                                       // fcomp 0x9960c0 (0.5): <= 0.5 -> 0
}

bool Tank::backToFront(const Cloud& a, const Cloud& b) {
    // 0x5fdf10: active clouds first, then by distance, farthest first.
    if (a.active != b.active) return a.active;
    return a.dist > b.dist;
}

void Tank::populate(float cloudness) {
    // 0x5fdd10.
    clouds_.resize(size_t(p_.count));
    const int target = int(std::floor(clamp01(cloudness) * float(p_.count)));
    active_ = target;
    int tries = 0;
    for (int i = 0; i < p_.count; ++i) {
        const bool inside = i < target;
        V3 q = randomPosition(inside);
        while (!accept(q) && tries < 0x400) {
            q = randomPosition(inside);
            ++tries;
        }
        build(clouds_[size_t(i)], q);
        clouds_[size_t(i)].active = inside;
    }
    dirty_ = true;
}

void Tank::setPosition(const V3& pos, float cloudness) {
    // 0x5fd890: distance from the previous position; > 1000 m resets the timer and repopulates.
    const float dx = pos[0] - pos_[0], dy = pos[1] - pos_[1], dz = pos[2] - pos_[2];
    const float d = std::sqrt(dx * dx + dy * dy + dz * dz);
    pos_ = pos;
    if (1000.0f < d || !havePos_) {
        havePos_ = true;
        timer_ = 0;
        populate(cloudness);
    }
}

void Tank::update(float dt, float cloud, float cloudness) {
    // 0x5fe040. (+0x2c += wind dt is kept by the engine but no cloud path reads it: the clouds do not drift.)
    layer_ = (p_.heightCloudy - p_.heightClear) * cloud + p_.heightClear;
    const int target = int(std::floor(clamp01(cloudness) * float(p_.count)));
    if (clouds_.empty()) populate(cloudness);
    if (0.0f < timer_) {
        timer_ -= dt;
        return;
    }
    timer_ = p_.refresh;
    dirty_ = true;
    active_ = 0;
    for (Cloud& c : clouds_) {
        const float dx = pos_[0] - c.pos[0], dz = pos_[2] - c.pos[2], dy = -c.pos[1];
        c.dist = std::sqrt(dz * dz + dy * dy + dx * dx);  // the tank y is not used: horizontal distance
        if (p_.sight * 1.25f < c.dist) c.active = false;
        if (c.active) ++active_;
    }
    int tries = 0;
    for (Cloud& c : clouds_) {
        if (0x3f < tries) break;
        if (c.active || target <= active_) continue;
        V3 q = randomPosition(false);
        bool ok = accept(q);
        while (!ok) {
            if (0x3f < tries) break;
            q = randomPosition(false);
            ++tries;
            ok = accept(q);
        }
        if (ok && tries < 0x40) {
            build(c, q);
            ++active_;
        }
    }
    std::stable_sort(clouds_.begin(), clouds_.end(), backToFront);
}

std::array<float, 28> drawConstants(float cloudness, float cloudsAlpha, float sight, float layer, float tankY) {
    std::array<float, 28> c{};
    c[0] = cloudness, c[1] = cloudsAlpha;
    c[20] = -100.0f;                       // 0xc2c80000
    c[24] = 2000.0f, c[25] = sight;        // 0x44fa0000, +0x8c
    c[26] = 0.8f * sight;                  // +0x88 (ctor / 0x5fe040: 0.8 x +0x8c)
    c[27] = layer + tankY;                 // +0xf8 + +0x18
    return c;
}

Corner vertexCorner(const Cloud& c, int sx, int sy, const V3& eye, float layerY, float sight, float cloudness) {
    // sfx.vhl vh_cloud with C_USER5.x = -100 (downHeight) and C_USER6 = (2000, sight, 0.8 sight, layer + tank y)
    // (constants 0x5fe370): AppearanceZone = sight, SightLine = 0.8 sight.
    const float zone = sight, sightLine = 0.8f * sight, down = -100.0f;
    float p[3] = {c.pos[0], c.pos[1] + layerY, c.pos[2]};
    float v[3] = {p[0] - eye[0], p[1] - eye[1], p[2] - eye[2]};
    const float len = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    float scale = 1.0f + (0.75f - 1.0f) * (len / zone);
    if (len > zone) scale = 0;
    p[1] += down * (len / zone);
    float vd[3] = {p[0] - eye[0], p[1] - eye[1], p[2] - eye[2]};
    const float vl = std::sqrt(vd[0] * vd[0] + vd[1] * vd[1] + vd[2] * vd[2]);
    for (float& x : vd) x /= vl;
    // right = normalize(cross(viewDir, up)), up' = cross(right, viewDir)
    float r[3] = {vd[1] * 0 - vd[2] * 1, vd[2] * 0 - vd[0] * 0, vd[0] * 1 - vd[1] * 0};
    const float rl = std::sqrt(r[0] * r[0] + r[1] * r[1] + r[2] * r[2]);
    for (float& x : r) x /= rl;
    const float u[3] = {r[1] * vd[2] - r[2] * vd[1], r[2] * vd[0] - r[0] * vd[2], r[0] * vd[1] - r[1] * vd[0]};
    const float w = float(sx) * c.width, h = float(sy) * c.height;
    Corner o;
    for (int i = 0; i < 3; ++i) o.pos[size_t(i)] = p[i] + w * r[i] * scale + h * u[i] * scale;
    o.alpha = clamp01(1.0f - (len - (sightLine - zone)) / zone);
    const float s = 1.0f - (1.0f + (h > 0 ? 1.0f : h < 0 ? -1.0f : 0.0f)) * 0.5f;
    o.shade = 1.0f - cloudness * s;
    return o;
}

}  // namespace fuel::clouds
