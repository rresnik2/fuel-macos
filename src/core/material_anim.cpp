#include "material_anim.hpp"

#include "reader.hpp"

#include <cmath>
#include <stdexcept>

namespace fuel {

namespace {

// Curve_Read12/16/20 (0x804b90 / 0x804a40 / 0x804ce0): u16 mode, u32 n, n x {f32 t, N x f32}.
template <size_t N>
MaterialAnim::Track<N> readCurve(ByteReader& r) {
    MaterialAnim::Track<N> tr;
    tr.mode = r.u16();
    const uint32_t n = r.count(1 << 16);
    tr.keys.resize(n);
    for (auto& k : tr.keys) {
        k.t = r.f32();
        for (float& v : k.v) v = r.f32();
    }
    return tr;
}

// 0x804960: u16 mode, u32 n, n x {f32 t, i16 v, i16 pad}; 0x72fa90 reads v / 4096.
MaterialAnim::Track<1> readShortCurve(ByteReader& r) {
    MaterialAnim::Track<1> tr;
    tr.mode = r.u16();
    const uint32_t n = r.count(1 << 16);
    tr.keys.resize(n);
    for (auto& k : tr.keys) {
        k.t = r.f32();
        const auto v = r.get<int16_t>();
        r.u16();
        k.v[0] = float(v) * 0.00024414062f;
    }
    return tr;
}

// 0x804b30: u32 n, n x {f32 t, u32 v}.
std::vector<MaterialAnim::StepKey> readSteps(ByteReader& r) {
    std::vector<MaterialAnim::StepKey> keys(r.count(1 << 16));
    for (auto& k : keys) {
        k.t = r.f32();
        k.v = r.u32();
    }
    return keys;
}

}  // namespace

MaterialAnim parseMaterialAnimBody(const std::vector<uint8_t>& body) {
    ByteReader r(body);
    MaterialAnim a;
    // 0x804ab0: u32 n, n x {u32 bitmap, f32 t} (stored swapped: the time first).
    a.texture.resize(r.count(1 << 16));
    for (auto& k : a.texture) {
        k.v = r.u32();
        k.t = r.f32();
    }
    a.uvOffset = readCurve<2>(r);
    a.uvScale = readCurve<2>(r);
    a.uvAngle = readShortCurve(r);
    a.diffuse = readCurve<3>(r);
    a.emissive = readCurve<3>(r);
    a.alpha = readShortCurve(r);
    a.specular = readCurve<4>(r);
    a.params = readCurve<4>(r);
    a.field2c = readSteps(r);
    a.flags = readSteps(r);
    a.material = r.u32();
    a.period = r.f32();
    a.state = r.get<uint8_t>();
    return a;
}

bool MaterialAnimClock::advance(float dt, float period) {
    if ((state & 0x40) == 0) return false;
    time += dt;
    if (time <= period || (state & 2) == 0) {
        // FUEL loops `while (period < t) t -= period` (endless for period 0; the port stops there).
        if (period > 0)
            while (period < time) time -= period;
    } else {
        time = period;
        state = uint8_t((state & 0x3f) | 0x80);
    }
    return true;
}

// Curve_Eval12 0x800000 / Curve_Eval16 0x7d5870 / Curve_Eval20 0x7d20d0 from key 1: the segment
// [t(i-1), t(i)) holding t is interpolated (held when mode bit 3 is set); a time outside every
// segment (before the first key too) takes the last key.
template <size_t N>
std::array<float, N> evalTrack(const MaterialAnim::Track<N>& tr, float t) {
    const auto& k = tr.keys;
    const size_t n = k.size();
    if (n == 0) return {};
    if (n == 1) return k[0].v;
    for (size_t i = 1; i < n; ++i) {
        if (t < k[i - 1].t || k[i].t <= t) continue;
        if (tr.mode & 8) return k[i - 1].v;
        const float s = (t - k[i - 1].t) / (k[i].t - k[i - 1].t);
        std::array<float, N> out;
        for (size_t j = 0; j < N; ++j) out[j] = (k[i].v[j] - k[i - 1].v[j]) * s + k[i - 1].v[j];
        return out;
    }
    return k[n - 1].v;
}
template std::array<float, 1> evalTrack(const MaterialAnim::Track<1>&, float);
template std::array<float, 2> evalTrack(const MaterialAnim::Track<2>&, float);
template std::array<float, 3> evalTrack(const MaterialAnim::Track<3>&, float);
template std::array<float, 4> evalTrack(const MaterialAnim::Track<4>&, float);

// 0x72fa90: as above on the i16 / 4096 values (a + (b - a) s).
float evalFloatTrack(const MaterialAnim::Track<1>& tr, float t) {
    const auto& k = tr.keys;
    const size_t n = k.size();
    if (n == 0) return 0;
    if (n == 1) return k[0].v[0];
    for (size_t i = 1; i < n; ++i) {
        if (k[i - 1].t <= t && t < k[i].t) {
            if (tr.mode & 8) return k[i - 1].v[0];
            return k[i - 1].v[0] + (k[i].v[0] - k[i - 1].v[0]) * ((t - k[i - 1].t) / (k[i].t - k[i - 1].t));
        }
    }
    return k[n - 1].v[0];
}

// 0x800470 / 0x7ffae0: the value of the key whose segment [t(i-1), t(i)) holds t, else the last.
uint32_t evalStepTrack(const std::vector<MaterialAnim::StepKey>& k, float t) {
    const size_t n = k.size();
    if (n == 0) return 0;
    for (size_t i = 1; i < n; ++i)
        if (k[i - 1].t <= t && t < k[i].t) return k[i - 1].v;
    return k[n - 1].v;
}

MaterialDrawState materialDrawState(const Material& m, uint8_t alphaKind) {
    MaterialDrawState d;
    const uint32_t code = m.flags & 0x1f;
    const bool unlit = (m.flags & 0x3000) != 0;
    bool blend = false, test = false;
    if (m.alpha >= 1.0f && !unlit) {
        if (m.flags & 0x180000)
            test = true;
        else if (alphaKind == 2)
            blend = true;
        else if (alphaKind == 1)
            test = true;
    } else {
        blend = true;
    }
    if (code == 17 || code == 19)
        d.mode = MaterialDrawState::Opaque;
    else if (blend)
        d.mode = MaterialDrawState::Blend;
    else if (test)
        d.mode = MaterialDrawState::Test;
    d.alphaRef = (m.flags & 0x100000) ? 128.0f / 255.0f : 0.0f;
    d.twoSided = (m.flags & 0x8000) != 0 || d.mode == MaterialDrawState::Blend;
    d.cubemap = m.fresnel[0] + m.fresnel[1] != 0.0f || m.textures[8] != 0;
    return d;
}

uint32_t blendPhase(uint32_t flags) {
    const uint32_t code = flags & 0x1f;
    if (code == 0x17) return 0x13;
    const uint32_t low = (flags & 0x10000) ? 0u : 1u;
    if (code == 0x12 || code == 0x14) return 0x12 | low;
    return ((flags & 0x40000) ? 0x0cu : 0x12u) | low;
}

// ---- PK28: 0x856160
BlendKind blendKind(uint32_t flags) {
    if (flags & 0x1000) return BlendKind::Add;
    if (flags & 0x4000) return BlendKind::AddDestAlpha;
    if (flags & 0x2000) return BlendKind::RevSubtract;
    return BlendKind::Alpha;
}

BlendState blendState(uint32_t flags) {
    switch (blendKind(flags)) {
        case BlendKind::Add: return {1, 5, 2};           // ADD, SRCALPHA, ONE
        case BlendKind::AddDestAlpha: return {1, 7, 2};  // ADD, DESTALPHA, ONE
        case BlendKind::RevSubtract: return {3, 5, 2};   // REVSUBTRACT, SRCALPHA, ONE
        default: return {1, 5, 6};                       // ADD, SRCALPHA, INVSRCALPHA
    }
}
// ---- PK28 end

bool ignoresFade(uint32_t code) {
    return code == 1 || code == 0xd || code == 0x11 || code == 0x12 || code == 0x13 || code == 0x14 || code == 0x17;
}

bool fadePrePass(uint32_t flags) { return (flags & 0xb000) == 0 || (flags & 0x200000) != 0; }

bool blendZWrite(uint32_t flags) {
    const uint32_t code = flags & 0x1f;
    return (flags & 0x40000) != 0 || code == 0x12 || code == 0x14;
}

void updateUvMatrix(Material& m) {
    // 0x695ee0 (FUN_0040f3f0 = cos, FUN_0040f3d0 = sin).
    const float c = std::cos(m.uvAngle), s = std::sin(m.uvAngle);
    const float su = m.uvScale[0], sv = m.uvScale[1];
    const float a = su * c, b = su * s, d = sv * c, e = sv * s;
    m.uvMatrix[0] = a;   // +0x2c
    m.uvMatrix[2] = b;   // +0x3c
    m.uvMatrix[1] = -e;  // +0x30
    m.uvMatrix[3] = d;   // +0x40
    m.uvMatrix[4] = (m.uvOffset[0] - (b + a) * 0.5f) + 0.5f;
    m.uvMatrix[5] = (m.uvOffset[1] - (d + -e) * 0.5f) + 0.5f;
}

void evaluateMaterialAnim(const MaterialAnim& a, float t, Material& m) {
    if (!a.uvOffset.keys.empty()) {
        const auto v = evalTrack(a.uvOffset, t);
        m.uvOffset = {v[0], v[1]};
    }
    if (!a.uvScale.keys.empty()) {
        const auto v = evalTrack(a.uvScale, t);
        m.uvScale = {v[0], v[1]};
    }
    if (!a.uvAngle.keys.empty()) m.uvAngle = evalFloatTrack(a.uvAngle, t);
    if (!a.diffuse.keys.empty()) {
        const auto v = evalTrack(a.diffuse, t);
        m.diffuse = {v[0], v[1], v[2]};
    }
    if (!a.emissive.keys.empty()) {
        const auto v = evalTrack(a.emissive, t);
        m.emissive = {v[0], v[1], v[2]};
    }
    if (!a.specular.keys.empty()) {
        const auto v = evalTrack(a.specular, t);
        m.specular = {v[0], v[1], v[2], v[3]};
    }
    if (!a.params.keys.empty()) {
        const auto v = evalTrack(a.params, t);
        m.params = {v[0], v[1], v[2] * 0.01f, v[3]};  // obj+0x74 x 0.01 (0x99614c)
    }
    if (!a.field2c.empty()) m.field2c = int32_t(evalStepTrack(a.field2c, t));
    if (!a.texture.empty()) m.textures[0] = evalStepTrack(a.texture, t);
    if (!a.alpha.keys.empty()) m.alpha = evalFloatTrack(a.alpha, t);
    if (!a.flags.empty()) m.flags = evalStepTrack(a.flags, t);
    updateUvMatrix(m);
}

}  // namespace fuel
