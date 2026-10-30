#include "genworld_noise.hpp"

#include <cmath>
#include <cstring>
#include <stdexcept>

namespace fuel {

namespace {

constexpr uint32_t kGenWorldClass = 0x31FA94AE;

// The engine's fixed shuffle of 0..255 that the seed permutes further.
constexpr uint8_t kBaseShuffle[256] = {
    0xe8, 0xba, 0xe3, 0x14, 0x4f, 0x83, 0xa0, 0xa9, 0xc8, 0x9f, 0x1d, 0x4e, 0xb3, 0x38, 0x45, 0xbc, 0xeb, 0xb9, 0xc5, 0x53, 0x13, 0xd3,
    0xfc, 0x65, 0xf6, 0x85, 0xb4, 0xbf, 0x34, 0x35, 0x4a, 0x99, 0xad, 0x74, 0x56, 0x04, 0x4d, 0xf1, 0x25, 0x5c, 0x95, 0x02, 0x09, 0x26,
    0x18, 0xd5, 0x06, 0xf9, 0x1f, 0x63, 0x7f, 0xcc, 0x48, 0x30, 0x9a, 0x0c, 0x32, 0xc2, 0x49, 0x9e, 0xc0, 0xfa, 0xde, 0x8a, 0xf5, 0x73,
    0x21, 0x42, 0x07, 0xb7, 0x19, 0x5f, 0xe6, 0x6d, 0x9b, 0xbd, 0x0d, 0x54, 0x89, 0x40, 0xce, 0xec, 0xac, 0x23, 0xa4, 0x29, 0x1b, 0x36,
    0x87, 0xe2, 0x81, 0xd0, 0xdc, 0x01, 0x0f, 0x2a, 0xe7, 0x6f, 0x6e, 0xa2, 0xc6, 0x70, 0x77, 0x94, 0xf0, 0x0a, 0x2f, 0x92, 0x93, 0x96,
    0x7b, 0xa7, 0xb2, 0x27, 0xe4, 0xf3, 0x61, 0x9c, 0x60, 0xcb, 0x28, 0x84, 0x8e, 0x44, 0x24, 0x88, 0x47, 0x7d, 0x90, 0x91, 0x1c, 0x69,
    0x39, 0x3c, 0xa5, 0xd7, 0x17, 0x79, 0x41, 0x5e, 0x50, 0x15, 0x7e, 0xf7, 0x3b, 0x55, 0x08, 0xda, 0xf4, 0xc1, 0xdd, 0xa1, 0xab, 0x6b,
    0x4b, 0x6c, 0xe5, 0x67, 0x12, 0x5d, 0xea, 0x3d, 0x51, 0xa3, 0xd4, 0xed, 0xb5, 0xc4, 0xff, 0x57, 0x75, 0xfd, 0x5b, 0x4c, 0x80, 0xaa,
    0x52, 0xe1, 0x0e, 0x8f, 0x37, 0xcd, 0x1e, 0xaf, 0xc7, 0x0b, 0x1a, 0xb0, 0xd9, 0x82, 0xf2, 0x46, 0x7c, 0x16, 0x2c, 0x62, 0x64, 0xa8,
    0xae, 0x2d, 0xef, 0xa6, 0x05, 0x3a, 0x76, 0xd1, 0x20, 0xbe, 0xb6, 0x3e, 0xb1, 0xe0, 0x72, 0x9d, 0xb8, 0xd8, 0x6a, 0xc3, 0x66, 0x8c,
    0x3f, 0x98, 0x11, 0x43, 0x59, 0x78, 0xd2, 0xd6, 0xcf, 0xdb, 0xf8, 0xfb, 0xdf, 0x22, 0x7a, 0x5a, 0xfe, 0xca, 0x2e, 0x86, 0xe9, 0x33,
    0x71, 0x8d, 0x58, 0x68, 0x8b, 0xbb, 0xee, 0x10, 0x31, 0xc9, 0x03, 0x2b, 0x97, 0x00,
};

uint32_t rd32(const std::vector<uint8_t>& b, size_t o) {
    if (o + 4 > b.size()) throw std::runtime_error("GenWorld_Z body too short");
    uint32_t v;
    std::memcpy(&v, &b[o], 4);
    return v;
}

constexpr float kEdgeSpread = 1.0f / 510.0f, kCentreSpread = 1.0f / 255.0f;

}  // namespace

Permutation buildPermutation(uint32_t seed) {
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
    return p[size_t(int64_t(std::nearbyint(v)) & 255)];
}

Fractal::Fractal(const Permutation& p, int levels, float size, float seed, float lowAmp, float highAmp, float gain)
    : table_(p), levels_(levels) {
    finest_ = size / float(1 << levels);
    seedOffset_ = int32_t(std::nearbyint(double(seed) * 262897.0));
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
