// P65: FUEL's tornado look (see tornado_fx.hpp; notes ~/re-work/fuel/notes/p65-storm-visuals.md).
#include "tornado_fx.hpp"

#include <algorithm>
#include <cmath>

namespace fuel::storm {

namespace {
constexpr float kTwoPi = 6.2831855f;  // .rdata as 0x5e7ca0 / 0x5e7a00 use it
using F3 = std::array<float, 3>;
F3 sub(const F3& a, const F3& b) { return {a[0] - b[0], a[1] - b[1], a[2] - b[2]}; }
F3 cross(const F3& a, const F3& b) { return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]}; }
float dot(const F3& a, const F3& b) { return a[0] * b[0] + a[1] * b[1] + a[2] * b[2]; }
// The inline normalise of 0x5e82d0: only when the squared length is above 1e-6.
F3 normIf(F3 v) {
    const float l2 = dot(v, v);
    if (l2 > 1e-6f) {
        const float k = 1.0f / std::sqrt(l2);
        v = {v[0] * k, v[1] * k, v[2] * k};
    }
    return v;
}
}  // namespace

TornadoLook TornadoLook::load(const std::function<float(int)>& P, bool mother) {
    TornadoLook l;  // m_5e6860(param): mother = the mesh 0xa6570c
    if (mother) {
        l.topFlare = P(5210), l.baseRadius = P(5211), l.wobble = 200, l.vScale = P(5214), l.scroll = P(5254);
        l.smokeSize = P(5221), l.smokeRise = P(5223), l.smokeSpread = P(5225), l.smokeAlpha = P(5227);
        l.smokeSpinMin = P(5255), l.smokeSpinMax = P(5256), l.cloudSpin = P(5253);
    } else {
        l.topFlare = P(5212), l.baseRadius = P(5213), l.wobble = 250, l.vScale = P(5215), l.scroll = P(5264);
        l.smokeSize = P(5220), l.smokeRise = P(5222), l.smokeSpread = P(5224), l.smokeAlpha = P(5226);
        l.smokeSpinMin = P(5265), l.smokeSpinMax = P(5266), l.cloudSpin = P(5263);
    }
    l.fadeIn = std::max(0.01f, P(5216)), l.fadeOut = std::max(0.01f, P(5217));  // 0x5e6480: 1 / max(x, 0.01)
    l.moveU = P(5218), l.moveV = P(5219);
    return l;
}

TornadoEffect::TornadoEffect(const TornadoLook& look, uint32_t seed) : look_(look), rng_(seed ? seed : 1u) {
    // vf02 0x5e7ca0: two retargets (old = new, new = random), the (sin, cos) table of j 2 pi / (16 - 1), the smoke.
    retarget();
    retarget();
    for (int j = 0; j < kSegments; ++j) {
        const float a = float(j) * kTwoPi / float(kSegments - 1);
        segSin_[j] = {std::sin(a), std::cos(a)};  // 0x40f3d0 = fsin, 0x40f3f0 = fcos
    }
    // 0x5e7a00 (Well512 draws in FUEL): per particle alpha scale, height fraction, fall rate, angle, spin per update,
    // rotation, rotation speed.
    parts_.resize(kSmoke);
    for (Particle& p : parts_) {
        p.angle = rand01() * kTwoPi - 3.1415927f;
        p.spin = -look_.smokeSpinMin + rand01() * (-look_.smokeSpinMax + look_.smokeSpinMin);
        p.rot = rand01() * kTwoPi - 3.1415927f;
        p.rotSpeed = (rand01() + 1.0f) * 0.04f;
        p.h = rand01();
        p.fall = (rand01() + 1.0f) * 0.03f;
        p.alpha = look_.smokeAlpha * 0.7f + rand01() * (look_.smokeAlpha * 1.3f - look_.smokeAlpha * 0.7f);
    }
    verts_.resize(size_t(kRings * kSegments));
    smokeOut_.resize(kSmoke);
}

float TornadoEffect::rand01() {  // [port] Park-Miller minimal standard (FUEL: Rand01_ParkMillerShuffle / Well512)
    rng_ = uint32_t((uint64_t(rng_) * 48271u) % 2147483647u);
    return float(rng_ - 1) / 2147483646.0f;
}

void TornadoEffect::retarget() {  // 0x5e81d0
    lerp_ = 0;
    const float f = 2.0f * look_.wobble;  // +0x50 (2, the ctor) x +0x4c
    sOld_ = sNew_, eOld_ = eNew_;
    sNew_ = {0, f * 0.5f, 0};
    const float rz = rand01(), rx = rand01();  // the first draw is +0x90 (z), the second +0x88 (x)
    eNew_ = {rx * (f * 0.5f + f * 0.5f) - f * 0.5f, f, rz * (f * 0.5f + f * 0.5f) - f * 0.5f};
}

std::array<float, 3> TornadoEffect::spine(float t) const {
    // 0x5e7fe0: ((A t + B) t + C) t + D from the base (0, 0, 0) to the top +0x114 with tangents 3 s and 3 e (s = ESI
    // = the +0xfc lerp, e = EDI = the +0x108 lerp): a cubic Hermite.
    F3 o;
    for (int k = 0; k < 3; ++k) {
        const float p0 = 0, p1 = top_[size_t(k)], s = s_[size_t(k)], e = e_[size_t(k)];
        const float A = p0 * 2.0f + s * 3.0f + e * 3.0f - p1 * 2.0f;
        const float B = (p0 * -3.0f - s * 6.0f) - e * 3.0f + p1 * 3.0f;
        o[size_t(k)] = ((A * t + B) * t + s * 3.0f) * t + p0;
    }
    return o;
}

void TornadoEffect::build(float dt) {
    // 0x5e82d0 (the 0x5e91d0 ground heights feed only the smoke's clip constants, not ported).
    const float k = 0.05f * dt;  // +0x48 x dt
    wob_ += k;                   // +0x98
    const float c = std::cos(wob_), s = std::sin(wob_);
    top_ = {(c - s) * look_.wobble, height_, (c + s) * look_.wobble};  // +0x114 / +0x118 = +0x12c / +0x11c
    for (int i = 0; i < 3; ++i) {
        s_[size_t(i)] = (sNew_[size_t(i)] - sOld_[size_t(i)]) * lerp_ + sOld_[size_t(i)];
        e_[size_t(i)] = (eNew_[size_t(i)] - eOld_[size_t(i)]) * lerp_ + eOld_[size_t(i)];
    }
    lerp_ += k * 4.0f;
    if (lerp_ >= 1.0f) retarget();
    const float growth = 1.0f;  // +0xa0 = SpecialEffectsTornado_G +0x40 = 1 (ctor; nothing else writes it)
    const float g = std::min(1.0f, growth * 1.3333334f);
    const float base = std::max(0.0f, (growth - 0.75f) * 4.0f) * look_.baseRadius;
    const float t0 = 1.0f - g;
    F3 prev = spine(t0);
    const F3 C = {0, 0, 1};  // 0xa06350
    for (int i = 0; i < kRings; ++i) {
        const float f = float(i) / float(kRings);
        const float r = std::pow(f, 4.0f) * look_.topFlare + base;
        const float squash = std::pow(float(i) / float(kRings - 1), 12.0f);
        const F3 next = spine((1.0f / float(kRings) + f) * g + t0);
        const F3 T = normIf(sub(next, prev));
        const F3 U = normIf(cross(C, T));
        const F3 V = normIf(cross(T, U));
        float q = 1.0f, qa = 1.0f;
        if (i >= kRings - 5) {
            q = 1.0f - float(i - kRings + 5) * 0.25f;
            qa = std::fabs(q);
        }
        const float a = (2.0f - qa) * q * alpha_;  // +0xdc
        const float v = look_.vScale * 1.0f * (1.0f - f) * ((1.0f - f) + 1.0f) * g;  // +0x138 x +0x130
        for (int j = 0; j < kSegments; ++j) {
            // 0x5e7850: rotation about T by (sin, cos); n = M^T V (the rows read as columns).
            const float sn = segSin_[j][0], cs = segSin_[j][1], omc = 1.0f - cs;
            float M[11];
            M[0] = T[0] * T[0] * omc + cs, M[5] = T[1] * T[1] * omc + cs, M[10] = T[2] * T[2] * omc + cs;
            M[4] = T[2] * sn + T[1] * T[0] * omc, M[1] = T[1] * T[0] * omc - T[2] * sn;
            M[8] = T[2] * T[0] * omc - T[1] * sn, M[2] = T[1] * sn + T[2] * T[0] * omc;
            M[9] = sn * T[0] + T[2] * T[1] * omc, M[6] = T[2] * T[1] * omc - sn * T[0];
            F3 n = {M[0] * V[0] + M[4] * V[1] + M[8] * V[2], M[1] * V[0] + M[5] * V[1] + M[9] * V[2],
                    M[2] * V[0] + M[6] * V[1] + M[10] * V[2]};
            n[1] *= 1.0f - squash;
            TornadoVertex& o = verts_[size_t(i * kSegments + j)];
            o.pos[0] = n[0] * r + prev[0], o.pos[1] = n[1] * r + prev[1], o.pos[2] = n[2] * r + prev[2];
            o.normal[0] = n[0], o.normal[1] = n[1], o.normal[2] = n[2];
            o.uv[0] = float(j) / float(kSegments - 1) * 1.0f;  // +0x134
            o.uv[1] = v;
            o.alpha = a, o.add = 0;
            o.tangent[0] = 1, o.tangent[1] = 0, o.tangent[2] = 0;
        }
        // Tangent = this vertex - the next one round the ring, binormal = tangent x normal; the last copies the first.
        for (int j = 0; j < kSegments; ++j) {
            TornadoVertex& o = verts_[size_t(i * kSegments + j)];
            if (j == kSegments - 1) {
                const TornadoVertex& f0 = verts_[size_t(i * kSegments)];
                std::copy(f0.tangent, f0.tangent + 3, o.tangent);
                std::copy(f0.binormal, f0.binormal + 3, o.binormal);
                continue;
            }
            const TornadoVertex& nx = verts_[size_t(i * kSegments + (j + 1) % kSegments)];
            const F3 t = {o.pos[0] - nx.pos[0], o.pos[1] - nx.pos[1], o.pos[2] - nx.pos[2]};
            const F3 nn = {o.normal[0], o.normal[1], o.normal[2]};
            const F3 b = cross(t, nn);
            std::copy(t.begin(), t.end(), o.tangent);
            std::copy(b.begin(), b.end(), o.binormal);
        }
        prev = next;
    }
    // 0x5e8ca0: the base smoke.
    for (size_t n = 0; n < parts_.size(); ++n) {
        Particle& p = parts_[n];
        p.h -= p.fall * dt;
        if (!(0.0f < p.h)) p.h = 1.0f;
        p.angle += p.spin;  // per update, not scaled by dt (as FUEL)
        p.rot += p.rotSpeed * dt;
        const float w = std::pow(1.0f - p.h, 8.0f) * look_.smokeRise;  // 0x8e07c0 = pow, 0x9dcca0 = 8.0
        TornadoSmoke& o = smokeOut_[n];
        o.y = look_.smokeSize * 0.05f + w;
        o.size = (p.h * 0.75f + 0.25f) * look_.smokeSize;
        o.diam = (look_.smokeSpread * p.h + 0.5f) * look_.baseRadius;
        o.alpha = (1.0f - w / look_.smokeRise) * (1.0f - p.h) * alpha_ * p.alpha;
        o.angle = p.angle;
        o.rot = p.rot;
    }
    built_ = true;
}

void TornadoEffect::update(float dt, const std::array<double, 3>& base, const std::array<double, 3>& camera, bool fading) {
    // 0x5e6e20
    if (fading && state_ != 1) state_ = 1;
    if (state_ == 1) {
        alpha_ = std::max(0.0f, alpha_ - dt / look_.fadeOut);
    } else if (state_ == 2) {
        alpha_ = std::min(1.0f, alpha_ + dt / look_.fadeIn);
        if (alpha_ >= 1.0f) state_ = 0;
    } else {
        alpha_ = 1.0f;
    }
    base_ = base;
    if (!started_) prev_ = base;
    const double dx = prev_[0] - base[0], dy = prev_[1] - base[1], dz = prev_[2] - base[2];
    prev_ = base;
    uOff_ += float(std::sqrt(dx * dx + dz * dz)) * look_.moveU;  // +0xb4 += |d xz| PP5218
    vOff_ += float(std::fabs(dy)) * look_.moveV;                 // +0xb8 += |d y| PP5219
    // The cloud height: max(base + 600, camera + 200) (0x9dc004 / 0x996370), followed at 0.1 / s.
    const float target = std::max(float(base[1]) + 600.0f, float(camera[1]) + 200.0f);
    if (!started_) cloudY_ = target;
    cloudY_ += (target - cloudY_) * dt * 0.1f;
    started_ = true;
    height_ = (cloudY_ - float(base[1])) + 250.0f;  // mesh +0x12c (0x9dc3a4 = 250)
    // The mesh's vf03 (0x5e9510): time, then the build on every other call (+0x3c toggles); FUEL builds only while
    // the camera is within 10000 m (0xa2a4e0) of a registered tornado.
    time_ += dt;
    const double cx = camera[0] - base[0], cy = camera[1] - base[1], cz = camera[2] - base[2];
    if (toggle_ && std::sqrt(cx * cx + cy * cy + cz * cz) < 10000.0) build(dt);
    toggle_ = !toggle_;
    cloudTime_ += dt;  // CloudTop vf03 0x5e5da0: +0x60 += dt
}

std::array<float, 2> TornadoEffect::uvScroll() const {
    const float t = time_ * look_.scroll;
    return {uOff_ - (t + t), t + vOff_};
}

const std::vector<uint16_t>& TornadoEffect::funnelIndices() {
    static const std::vector<uint16_t> idx = [] {  // 0x5e7ca0
        std::vector<uint16_t> v;
        for (int r = 0; r + 1 < kRings; ++r)
            for (int j = 0; j < kSegments; ++j) {
                const int j1 = (j + 1) % kSegments;
                const int a = r * kSegments + j, b = (r + 1) * kSegments + j1, c = (r + 1) * kSegments + j, d = r * kSegments + j1;
                for (int x : {a, b, c, a, d, b}) v.push_back(uint16_t(x));
            }
        return v;
    }();
    return idx;
}

std::array<TornadoCloudTop, 2> TornadoEffect::cloudTops() const {
    // 0x5e6e20 registers (base + top xz, cloud height) with the first two static CloudTops (ctor 0x5e6480: +0x48 half
    // size 6000 / 2000, +0x58 lift 500 / 200); vf04 0x5e5e40 turns them by -(+0x60 x +0x5c) about y.
    std::array<TornadoCloudTop, 2> out;
    const std::array<double, 3> c = {base_[0] + top_[0], double(cloudY_), base_[2] + top_[2]};
    out[0] = {c, 6000.0f, 500.0f, -(cloudTime_ * look_.cloudSpin)};
    out[1] = {c, 2000.0f, 200.0f, -(cloudTime_ * look_.cloudSpin * 1.5f)};
    return out;
}

}  // namespace fuel::storm
