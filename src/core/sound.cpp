#include "sound.hpp"

#include "reader.hpp"
#include "resources.hpp"
#include "vehicle_params.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>

namespace fuel {

namespace {
constexpr uint32_t kLodClass = 0x73DC6A13;
constexpr uint32_t kMaxSoundBytes = 64u << 20;
}  // namespace

Sound decodeSound(const DpcFile& dpc, uint32_t id) {
    const DpcFile::Entry* e = dpc.find(id);
    if (e == nullptr || e->classCrc != kSoundClass) throw std::runtime_error("no Sound_Z with that ID in " + dpc.path());
    // Link: u32 name CRC, u32 sample rate, u32 byte size, u16 flags (bit 1 loops, bit 2 stereo).
    const std::vector<uint8_t> link = dpc.link(*e);
    ByteReader r(link);
    r.u32();
    Sound s;
    s.rate = r.u32();
    const uint32_t bytes = r.u32();
    const uint16_t flags = r.u16();
    s.loops = (flags & 2) != 0;
    s.channels = (flags & 4) ? 2 : 1;
    if (s.rate < 4000 || s.rate > 192000) throw std::runtime_error("implausible sample rate " + std::to_string(s.rate));
    if (bytes > kMaxSoundBytes || bytes % (2u * uint32_t(s.channels)) != 0) throw std::runtime_error("bad sound size " + std::to_string(bytes));
    const auto body = dpc.body(*e);
    if (!body || body->size() < bytes) throw std::runtime_error("sound body shorter than its header says");
    s.samples.resize(bytes / 2);
    if (bytes) std::memcpy(s.samples.data(), body->data(), bytes);
    return s;
}

SoundBank readSoundBank(const DpcFile& dpc, const std::string& variant) {
    const uint32_t want = asoboCrc(variant + "_SKEL");
    for (uint32_t id : dpc.ids(kLodClass)) {
        const DpcFile::Entry* e = dpc.find(id);
        const std::vector<uint8_t> link = dpc.link(*e);
        uint32_t name = 0;
        if (link.size() >= 4) std::memcpy(&name, link.data(), 4);
        if (name != want) continue;
        const auto body = dpc.body(*e);
        if (!body) break;
        // Lod_Z body: points (m x 24-byte spheres, k x 72-byte boxes), 3 u32, a list of u32 objects, u32 h,
        // u32 flag1 [+ pairs], u32 flag2 [+ pairs], u32. A pair list is u32 n, then n x {u32 a, u32 b}.
        ByteReader r(*body);
        r.skipArray(24, 100000);
        r.skipArray(72, 100000);
        r.raw(12);
        r.skipArray(4);
        r.u32();
        SoundBank bank;
        if (r.u32() != 0) r.skipArray(8);
        if (r.u32() != 0) {
            const uint32_t n = r.count(4096);
            for (uint32_t i = 0; i < n; ++i) {
                const uint32_t symbol = r.u32();
                bank[symbol] = r.u32();
            }
        }
        return bank;
    }
    throw std::runtime_error("no sound bank " + variant + "_SKEL in " + dpc.path());
}

EngineModel EngineModel::fromParams(const std::vector<float>& p) {
    if (p.size() <= 649) throw std::runtime_error("vehicle parameters too short for the engine model");
    EngineModel m;
    m.maxRpm = p[vp::MaxRpm];
    m.idleRpm = p[vp::IdleRpm];
    m.gain = p[649];
    for (size_t i = 0; i < 7; ++i) m.nativeRpm[i] = p[636 + i];
    if (!(m.maxRpm > m.idleRpm) || !(m.gain > 0)) throw std::runtime_error("implausible engine parameters");
    for (float v : m.nativeRpm)
        if (!(v > 0)) throw std::runtime_error("implausible engine sample rpm");
    if (p.size() > 654) {
        m.popInterval = p[645];
        m.popRateThreshold = p[646];
        m.absorberThreshold = p[651];
        m.absorberScale = p[652];
        m.popThrottleMax = p[653];
        m.popRpmMin = p[654];
    }
    m.build(p[644]);
    return m;
}

// The game builds six 100-entry tables indexed by normalised rpm: the idle
// sample fades out over the first 15 %, each other layer is a Hann window
// (half overlap) whose pitch runs piecewise-linear through its neighbours'
// recorded rpm, and the weights at each index are normalised to `gain`.
void EngineModel::build(float spread) {
    const auto& p = nativeRpm;
    for (auto& row : pitch_) row.fill(1.0f);
    for (size_t k = 0; k < 100; ++k) {
        const float u = std::min(float(k), 15.0f) / 15.0f;
        if (k < 15) vol_[0][k] = 1.0f - u;
        pitch_[0][k] = 1.0f + u * (p[1] / p[0] - 1.0f);
    }
    const float s = (spread - 0.1f) * 0.2f, h = 0.5f * s;
    for (size_t L = 1; L < size_t(kLayers); ++L) {
        const float c = 0.1f + float(L - 1) * s;
        const float lo = p[L - 1] / p[L], hi = p[L + 1] / p[L];
        const int a = int((c - h) * 100.0f), b = int((c + s + h) * 100.0f);
        if (b <= a) continue;
        for (int i = std::max(a, 0); i < std::min(b, 100); ++i) {
            const float t = float(i - a) / float(b - a);
            pitch_[L][size_t(i)] = t < 0.5f ? lo + 2 * t * (1 - lo) : (hi - 1) * (t - 0.5f) * 2 + 1;
            vol_[L][size_t(i)] = (L == 1 && t < 0.5f) ? std::min(4 * t, 1.0f) : (std::cos((2 * t + 1) * float(M_PI)) + 1) * 0.5f;
        }
    }
    for (size_t i = 0; i < 100; ++i) {
        float total = 0;
        for (size_t L = 0; L < size_t(kLayers); ++L) total += vol_[L][i];
        if (total > 0)
            for (size_t L = 0; L < size_t(kLayers); ++L) vol_[L][i] *= gain / total;
    }
}

EngineModel::Mix EngineModel::mix(float rpm) const {
    const float x = std::clamp((rpm - idleRpm) / (maxRpm - idleRpm), 0.001f, 0.99f);
    const float pos = x * 100.0f;
    const size_t i = size_t(pos), j = std::min<size_t>(i + 1, 99);
    const float f = pos - float(i);
    Mix m;
    for (size_t L = 0; L < size_t(kLayers); ++L) {
        m.pitch[L] = pitch_[L][i] + (pitch_[L][j] - pitch_[L][i]) * f;
        m.weight[L] = vol_[L][i] + (vol_[L][j] - vol_[L][i]) * f;
    }
    return m;
}

}  // namespace fuel
