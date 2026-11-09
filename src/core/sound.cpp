#include "sound.hpp"

#include "reader.hpp"
#include "resources.hpp"
#include "vehicle_params.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <fstream>
#include <unordered_map>
#include <sstream>
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

SoundBank readSoundBank(const DpcFile& dpc, const std::string& variant) { return readSoundBankNamed(dpc, variant + "_SKEL"); }

namespace {
// Lod_Z body: points (m x 24-byte spheres, k x 72-byte boxes), 3 u32, a list of u32 objects, u32 h,
// u32 flag1 [+ pairs], u32 flag2 [+ pairs], u32. A pair list is u32 n, then n x {u32 a, u32 b}.
SoundBank parseLodBank(const std::vector<uint8_t>& body) {
    ByteReader r(body);
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
}  // namespace

SoundBank readSoundBankNamed(const DpcFile& dpc, const std::string& lodName) {
    const uint32_t want = asoboCrc(lodName);
    for (uint32_t id : dpc.ids(kLodClass)) {
        const DpcFile::Entry* e = dpc.find(id);
        const std::vector<uint8_t> link = dpc.link(*e);
        uint32_t name = 0;
        if (link.size() >= 4) std::memcpy(&name, link.data(), 4);
        if (name != want) continue;
        const auto body = dpc.body(*e);
        if (!body) break;
        return parseLodBank(*body);
    }
    throw std::runtime_error("no sound bank " + lodName + " in " + dpc.path());
}

std::vector<LodSound> lodSoundsWithSymbol(const DpcFile& dpc, uint32_t symbol) {
    std::vector<LodSound> out;
    for (uint32_t id : dpc.ids(kLodClass)) {
        const DpcFile::Entry* e = dpc.find(id);
        const std::vector<uint8_t> link = dpc.link(*e);
        uint32_t name = 0;
        if (link.size() >= 4) std::memcpy(&name, link.data(), 4);
        const auto body = dpc.body(*e);
        if (!body) continue;
        try {
            const SoundBank b = parseLodBank(*body);
            const auto it = b.find(symbol);
            if (it != b.end()) out.push_back({id, name, it->second});
        } catch (const std::exception&) {
        }
    }
    return out;
}

std::unordered_set<uint32_t> cloneClassNames(const std::string& cloneTsc, const std::string& baseClass) {
    // clone.tsc: "cloneclass <Name> <Base>" (whitespace separated, "//" comments); a Base may be an earlier clone.
    std::ifstream in(cloneTsc);
    std::unordered_map<std::string, std::string> base;
    std::vector<std::string> names;
    std::string line;
    auto lower = [](std::string v) {
        for (char& c : v) c = char(std::tolower(static_cast<unsigned char>(c)));
        return v;
    };
    while (std::getline(in, line)) {
        std::istringstream ls(line);
        std::string kw, name, b;
        if (!(ls >> kw >> name >> b) || kw != "cloneclass") continue;
        if (!base.count(lower(name))) names.push_back(name);
        base[lower(name)] = b;
    }
    std::unordered_set<uint32_t> out;
    for (const std::string& n : names) {
        std::string cur = base[lower(n)];
        for (int depth = 0; depth < 16; ++depth) {
            const auto it = base.find(lower(cur));
            if (it == base.end()) break;
            cur = it->second;
        }
        if (cur == baseClass) out.insert(asoboCrc(n));
    }
    return out;
}

std::unordered_set<uint32_t> decorSoundNames(const DpcFile& world, const std::string& cloneTsc) {
    const auto decor = cloneClassNames(cloneTsc, "IT_DECORMESH_G");
    std::unordered_set<uint32_t> out;
    for (const LodSound& l : lodSoundsWithSymbol(world, 0))
        if (decor.count(l.nameCrc)) out.insert(l.nameCrc);
    return out;
}

EngineModel EngineModel::fromParams(const std::vector<float>& p, float layer1Gain) {
    if (p.size() <= 654) throw std::runtime_error("vehicle parameters too short for the engine model");
    EngineModel m;
    m.maxRpm = p[vp::MaxRpm];
    m.idleRpm = p[vp::IdleRpm];
    m.gain = p[649];
    for (size_t i = 0; i < 7; ++i) m.nativeRpm[i] = p[636 + i];
    if (!(m.maxRpm > m.idleRpm) || !(m.gain > 0)) throw std::runtime_error("implausible engine parameters");
    for (float v : m.nativeRpm)
        if (!(v > 0)) throw std::runtime_error("implausible engine sample rpm");
    m.skidPitchRef = p[635];
    m.skidThreshold = p[634];
    m.popInterval = p[645];
    m.popRateThreshold = p[646];
    m.curveA = p[647];
    m.curveB = p[648];
    m.whineGain = p[650];
    m.absorberThreshold = p[651];
    m.absorberScale = p[652];
    if (p.size() > 669) m.absorberRumbleThreshold = p[668], m.absorberRumbleScale = p[669];
    m.popThrottleMax = p[653];
    m.popRpmMin = p[654];
    m.build(p[644], layer1Gain);
    return m;
}

// 0x5dbed0: six 100-entry tables indexed by normalised rpm. The idle sample fades out over the first
// 15 entries; layer L spans [trunc((c - s/2) 100), trunc((c + s + s/2) 100)) with c = 0.1 + (L-1) s,
// s = (P644 - 0.1) 0.2; its pitch runs piecewise-linear from p[L-1]/p[L] through 1 (mid-span) to
// p[L+1]/p[L]; its weight is a Hann window (1 + cos((2t + 1) pi))/2, except layer 1 below mid-span:
// G 4t up to t = 0.25, then G (G = PP 2009). Each index's weights are then scaled to sum to P649.
void EngineModel::build(float spread, float G) {
    const auto& p = nativeRpm;
    for (auto& row : pitch_) row.fill(0.0f);
    for (auto& row : vol_) row.fill(0.0f);
    for (int k = 0; k < 15; ++k) {
        const float u = float(k) * 0.06666667f;
        vol_[0][size_t(k)] = 1.0f - u;
        pitch_[0][size_t(k)] = u * (p[1] / p[0] - 1.0f) + 1.0f;
    }
    // x87 at 53-bit precision: s and h are stored as floats, the bounds are computed in double and truncated.
    const float s = float((double(spread) - double(0.1f)) * double(0.2f)), h = float(double(s) * 0.5);
    for (int L = 1; L < kLayers; ++L) {
        const float lo = p[size_t(L - 1)] * (1.0f / p[size_t(L)]), hi = (1.0f / p[size_t(L)]) * p[size_t(L + 1)];
        const double c = double(L - 1) * double(s) + double(0.1f);
        const int a = int((c - double(h)) * 100.0), b = int((c + double(h) + double(s)) * 100.0);
        if (b <= a) continue;
        for (int i = std::max(a, 0); i < std::min(b, 100); ++i) {
            const float t = float(i - a) / float(b - a);
            pitch_[size_t(L)][size_t(i)] = t >= 0.5f ? (hi - 1.0f) * (t - 0.5f) * 2.0f + 1.0f : (1.0f - lo) * t * 2.0f + lo;
            float w;
            if (L != 1 || t >= 0.5f) w = (std::cos((t * 2.0f + 1.0f) * 3.1415927f) + 1.0f) * 0.5f;
            else w = t >= 0.25f ? G : G * t * 4.0f;
            vol_[size_t(L)][size_t(i)] = w;
        }
    }
    for (size_t i = 0; i < 100; ++i) {
        float total = 0;
        for (size_t L = 0; L < size_t(kLayers); ++L) total += vol_[L][i];
        if (total > 0) {
            const float k = (1.0f / total) * gain;
            for (size_t L = 0; L < size_t(kLayers); ++L) vol_[L][i] *= k;
        }
    }
}

float EngineModel::curve(float x) const {
    if (curveA > 0) return ((curveA + 1.0f) * x) / (x + curveA);
    if (curveB > 0) return x / (-curveB * (x - 1.0f) + x);
    return x;
}

EngineModel::Mix EngineModel::mixAt(float x) const {
    Mix m;
    // Volume index: trunc(x * 100) clamped to 0..99 (0x5d98c0).
    const int vi = std::clamp(int(x * 100.0f), 0, 99);
    // Pitch 0x5d9740: i = trunc(x * 100); i <= 0 -> 0; i >= 99 -> table[99]; else lerp(i, i + 1).
    int pi = int(x * 100.0f);
    if (pi < 1) pi = 0;
    for (size_t L = 0; L < size_t(kLayers); ++L) {
        m.weight[L] = vol_[L][size_t(vi)];
        if (pi >= 99) {
            m.pitch[L] = pitch_[L][99];
        } else {
            const float f = (x - float(pi) * 0.01f) * 100.0f;
            m.pitch[L] = (1.0f - f) * pitch_[L][size_t(pi)] + pitch_[L][size_t(pi) + 1] * f;
        }
    }
    return m;
}

EngineModel::Mix EngineModel::mix(float rpm) const { return mixAt(std::clamp(curve(normalised(rpm)), 0.001f, 0.99f)); }

}  // namespace fuel
