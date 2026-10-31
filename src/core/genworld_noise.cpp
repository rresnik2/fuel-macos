#include "genworld_noise.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <iterator>
#include <mutex>
#include <optional>
#include <stdexcept>

namespace fuel {

namespace {

constexpr uint32_t kGenWorldClass = 0x31FA94AE;

// The engine's fixed shuffle of 0..255 that the seed permutes further. It lives
// in FUEL.exe's .rdata (VA 0xa01220), not in the world data, and the repo holds
// nothing taken from the game's files: it is read from the player's own copy,
// found next to the world DPC (<data>/XFinalComponent/FUEL.exe).
constexpr uint32_t kShuffleVa = 0xA01220;

// `n` bytes at virtual address `va` of a PE executable, or nothing.
std::optional<std::vector<uint8_t>> readExe(const std::filesystem::path& exe, uint32_t va, size_t n) {
    std::ifstream f(exe, std::ios::binary);
    if (!f) return std::nullopt;
    const std::vector<uint8_t> b((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    auto u16 = [&](size_t o) { return o + 2 <= b.size() ? uint32_t(b[o] | b[o + 1] << 8) : 0u; };
    auto u32 = [&](size_t o) { return o + 4 <= b.size() ? uint32_t(b[o] | b[o + 1] << 8 | b[o + 2] << 16 | uint32_t(b[o + 3]) << 24) : 0u; };
    const size_t pe = u32(0x3c);
    if (u32(pe) != 0x00004550) return std::nullopt;  // "PE\0\0"
    const uint32_t sections = u16(pe + 6), optSize = u16(pe + 20), imageBase = u32(pe + 24 + 28);
    const uint32_t rva = va - imageBase;
    for (uint32_t k = 0; k < sections; ++k) {
        const size_t sh = pe + 24 + optSize + size_t(k) * 40;
        const uint32_t sva = u32(sh + 12), vsize = std::max(u32(sh + 8), u32(sh + 16)), raw = u32(sh + 20);
        if (rva < sva || rva + n > sva + vsize) continue;
        const size_t off = size_t(raw) + (rva - sva);
        if (off + n > b.size()) return std::nullopt;
        return std::vector<uint8_t>(b.begin() + std::ptrdiff_t(off), b.begin() + std::ptrdiff_t(off + n));
    }
    return std::nullopt;
}

std::filesystem::path exePath(const DpcFile& world) {
    return std::filesystem::path(world.path()).parent_path().parent_path().parent_path() / "XFinalComponent" / "FUEL.exe";
}

std::optional<std::array<uint8_t, 256>> readShuffle(const std::filesystem::path& exe) {
    const auto b = readExe(exe, kShuffleVa, 256);
    if (!b) return std::nullopt;
    std::array<uint8_t, 256> t{};
    bool seen[256] = {};
    for (size_t i = 0; i < 256; ++i) {
        t[i] = (*b)[i];
        if (seen[t[i]]) return std::nullopt;  // not a permutation: another build of the exe
        seen[t[i]] = true;
    }
    return t;
}

const std::array<uint8_t, 256>& baseShuffle(const DpcFile& world) {
    static std::once_flag once;
    static std::array<uint8_t, 256> table;
    std::call_once(once, [&] {
        if (auto t = readShuffle(exePath(world))) {
            table = *t;
            return;
        }
        std::fprintf(stderr, "fuel: FUEL.exe not found next to the game data; procedural scenery will differ from the game\n");
        for (size_t i = 0; i < 256; ++i) table[i] = uint8_t((i * 167 + 13) & 255);  // some fixed shuffle
    });
    return table;
}


uint32_t rd32(const std::vector<uint8_t>& b, size_t o) {
    if (o + 4 > b.size()) throw std::runtime_error("GenWorld_Z body too short");
    uint32_t v;
    std::memcpy(&v, &b[o], 4);
    return v;
}

constexpr float kEdgeSpread = 1.0f / 510.0f, kCentreSpread = 1.0f / 255.0f;

}  // namespace

Permutation buildPermutation(const DpcFile& world, uint32_t seed) {
    const std::array<uint8_t, 256>& kBaseShuffle = baseShuffle(world);
    // Each output draws from the base shuffle through a 4-step chain keyed by the
    // seed's bytes, taking the next unused entry when the slot is taken.
    int avail[256];
    for (int i = 0; i < 256; ++i) avail[i] = kBaseShuffle[i];
    const uint32_t s0 = seed & 255, s1 = (seed >> 8) & 255, s2 = (seed >> 16) & 255, s3 = seed >> 24;
    Permutation out{};
    for (uint32_t d = 0; d < 256; ++d) {
        uint32_t e = kBaseShuffle[(d + s0) & 255];
        e = kBaseShuffle[(e + d + s1) & 255];
        e = kBaseShuffle[(e + d + s2) & 255];
        uint32_t k = (e + d + s3) & 255;
        int v = -1;
        for (int tries = 0; tries < 256; ++tries, k = (k + 1) & 255)
            if (avail[k] >= 0) {
                v = avail[k];
                break;
            }
        out[d] = uint8_t(v);
        avail[k] = -1;
    }
    return out;
}

std::vector<float> readEngineFloats(const DpcFile& world, uint32_t va, size_t count) {
    const auto b = readExe(exePath(world), va, count * 4);
    if (!b) return {};
    std::vector<float> out(count);
    std::memcpy(out.data(), b->data(), count * 4);
    return out;
}

uint32_t genWorldSeed(const DpcFile& world) {
    const std::vector<uint32_t> gens = world.ids(kGenWorldClass);
    if (gens.empty()) throw std::runtime_error("no GenWorld_Z");
    const auto body = world.body(*world.find(gens.front()));
    if (!body) throw std::runtime_error("GenWorld_Z has no body");
    size_t p = 12;  // three ids, then the map, ground texture and material lists
    for (int list = 0; list < 3; ++list) p += 4 + size_t(rd32(*body, p)) * 4;
    return rd32(*body, p);
}

uint8_t positionHash(const Permutation& p, double x, double z) {
    const double v = x * double(12.317513f) + z * double(10.193578f);
    return p[size_t(int64_t(std::floor(v)) & 255)];  // fistp under round-down (p21 H0)
}

Fractal::Fractal(const Permutation& p, int levels, float size, float seed, float lowAmp, float highAmp, float gain)
    : table_(p), levels_(levels) {
    finest_ = size / float(1 << levels);
    seedOffset_ = int32_t(std::floor(double(seed) * 262897.0));  // round-down fistp
    mulX_ = uint32_t(1u << (13 - levels)) * 0xC5Bu;
    mulZ_ = uint32_t(1u << (13 - levels)) * 0xA31u;
    const float rough = highAmp - lowAmp;
    for (int k = 0; k <= levels; ++k) {
        const float r = 1.0f / float(1 << k);
        const float a = r * (r * 0.0005f + 1.0f) * gain;
        Level l;
        l.edgeScale = a * rough / 255.0f;
        l.edgeBias = a * lowAmp / 255.0f;
        l.centreScale = a * 1.4142135f * rough / 255.0f;
        l.centreBias = a * 1.4142135f * lowAmp * 4.0f / 255.0f;
        level_.push_back(l);
    }
}

float Fractal::rnd(int32_t x, int32_t z) const {
    const uint32_t h = mulZ_ * uint32_t(z) + mulX_ * uint32_t(x);
    return float(table_[(h >> 8) & 255]) - 127.5f;
}

float Fractal::at(double x, double z) const {
    if (levels_ <= 0) return 0;
    const double fx = x / finest_, fz = z / finest_;
    const auto ix = int32_t(std::floor(fx)), iz = int32_t(std::floor(fz));
    // Top cell corners (x0,z0), (x1,z0), (x1,z1), (x0,z1).
    const uint32_t hx = uint32_t(seedOffset_ + (ix >> levels_)) * 0x41543u;
    const uint32_t hz = uint32_t(iz >> levels_) * 0x47411u;
    auto corner = [&](uint32_t add) { return float(table_[((hx + hz + add) >> 16) & 255]) / 255.0f; };
    float c[4] = {corner(0), corner(0x41543u), corner(0x88954u), corner(0x47411u)};
    const int32_t mask = ~((1 << levels_) - 1);
    int32_t x0 = ix & mask, z0 = iz & mask;
    for (int depth = levels_; depth >= 1; --depth) {
        const Level& l = level_[size_t(levels_ - depth)];
        const int32_t h = 1 << (depth - 1);
        auto edge = [&](float a, float b, int32_t ex, int32_t ez) {
            const float m = (a + b) * 0.5f;
            return (l.edgeScale * m + kEdgeSpread * std::fabs(a - b) + l.edgeBias) * rnd(ex, ez) + m;
        };
        const float sum = c[0] + c[1] + c[2] + c[3];
        const float centre =
            (sum + (l.centreScale * sum + kCentreSpread * (std::fabs(c[2] - c[0]) + std::fabs(c[3] - c[1])) + l.centreBias) * rnd(x0 + h, z0 + h)) *
            0.25f;
        const bool qx = (ix & h) != 0, qz = (iz & h) != 0;
        float n[4];
        if (!qz) {
            const float e01 = edge(c[0], c[1], x0 + h, z0);
            if (!qx) {
                n[0] = c[0], n[1] = e01, n[2] = centre, n[3] = edge(c[0], c[3], x0, z0 + h);
            } else {
                n[0] = e01, n[1] = c[1], n[2] = edge(c[1], c[2], x0 + 2 * h, z0 + h), n[3] = centre;
            }
        } else {
            const float e23 = edge(c[2], c[3], x0 + h, z0 + 2 * h);
            if (!qx) {
                n[0] = edge(c[0], c[3], x0, z0 + h), n[1] = centre, n[2] = e23, n[3] = c[3];
            } else {
                n[0] = centre, n[1] = edge(c[1], c[2], x0 + 2 * h, z0 + h), n[2] = c[2], n[3] = e23;
            }
        }
        for (int i = 0; i < 4; ++i) c[i] = n[i];
        x0 += ix & h;
        z0 += iz & h;
    }
    const auto tx = float(fx - ix), tz = float(fz - iz);
    return c[0] + (c[1] - c[0]) * tx + (c[3] - c[0]) * tz + (c[2] - c[3] - (c[1] - c[0])) * tx * tz;
}

}  // namespace fuel
