#include "terrain.hpp"


#include <algorithm>
#include <cstdlib>
#include <cmath>
#include <cstring>
#include <stdexcept>

namespace fuel {

namespace {

constexpr uint32_t kGenWorldClass = 0x31FA94AE;
constexpr uint32_t kBinaryClass = 0x86B29C80;
constexpr float kWorldSpan = 140000.0f;    // metres covered by the map
constexpr float kHeightRange = 10000.0f;   // metres for the map's full value range

uint32_t rd32(const std::vector<uint8_t>& b, size_t o) {
    if (o + 4 > b.size()) throw std::runtime_error("heightfield: short data");
    uint32_t v;
    std::memcpy(&v, &b[o], 4);
    return v;
}

}  // namespace

float Heightfield::heightAt(float x, float z) const {
    const float fx = std::clamp((x - originX) / spacing, 0.0f, float(width - 1));
    const float fz = std::clamp((z - originZ) / spacing, 0.0f, float(height - 1));
    const uint32_t c0 = std::min(uint32_t(fx), width - 2), r0 = std::min(uint32_t(fz), height - 2);
    const float tx = fx - float(c0), tz = fz - float(r0);
    const float a = sampleHeight(c0, r0) * (1 - tx) + sampleHeight(c0 + 1, r0) * tx;
    const float b = sampleHeight(c0, r0 + 1) * (1 - tx) + sampleHeight(c0 + 1, r0 + 1) * tx;
    return a * (1 - tz) + b * tz;
}

float Heightfield::smoothHeightAt(double x, double z) const {
    const double tx = (x - originX) / spacing, tz = (z - originZ) / spacing;
    const double ix = std::floor(tx), iz = std::floor(tz);
    const double ax = tx - ix, az = tz - iz;
    auto weights = [](double a, double w[4]) {
        w[0] = (1 - a) * (1 - a) * (1 - a) / 6;
        w[1] = (4 - 6 * a * a + 3 * a * a * a) / 6;
        w[3] = a * a * a / 6;
        w[2] = 1 - w[0] - w[1] - w[3];
    };
    double wx[4], wz[4];
    weights(ax, wx);
    weights(az, wz);
    double h = 0;
    for (int j = 0; j < 4; ++j) {
        const auto r = uint32_t(std::clamp<double>(iz - 1 + j, 0, height - 1));
        for (int i = 0; i < 4; ++i) {
            const auto c = uint32_t(std::clamp<double>(ix - 1 + i, 0, width - 1));
            h += wx[i] * wz[j] * raw[size_t(r) * width + c];
        }
    }
    return float(h * metresPerUnit);
}

std::vector<uint32_t> genWorldMaps(const DpcFile& world) {
    const std::vector<uint32_t> gens = world.ids(kGenWorldClass);
    if (gens.empty()) throw std::runtime_error(world.path() + ": no GenWorld_Z");
    const auto gen = world.body(*world.find(gens.front()));
    if (!gen) throw std::runtime_error("GenWorld_Z has no body");
    // GenWorld_Z body: u32, u32, u32 road graph, then u32 n and n Binary_Z map IDs.
    const uint32_t n = rd32(*gen, 12);
    std::vector<uint32_t> maps;
    for (uint32_t i = 0; i < n; ++i) maps.push_back(rd32(*gen, 16 + size_t(i) * 4));
    return maps;
}

Raster decodeRaster(const std::vector<uint8_t>& b) {
    // Binary_Z body: u32 size; raster header (u32 w, u32 h, f32, i32, f32 maxValue); then
    // u32 nA, u32 K, the block table, and one u32 per 4x4 block: low 12 bits = base value,
    // high 20 bits = block index. Blocks below nA hold 16 four-bit offsets (8 bytes at 8 + 8*i);
    // the others hold 16 byte offsets (16 bytes at 16*i + 8 - 8*nA).
    Raster r;
    r.width = rd32(b, 4);
    r.height = rd32(b, 8);
    std::memcpy(&r.maxValue, &b[20], 4);
    if (r.width % 4 || r.height % 4 || r.width < 4 || r.height < 4 || !(r.maxValue > 0))
        throw std::runtime_error("map header is implausible");
    const size_t d = 24;  // start of the raster data
    const uint32_t nA = rd32(b, d), K = rd32(b, d + 4);
    const size_t gridOff = d + 4 * (size_t(1) + K);
    const uint32_t gw = r.width / 4, gh = r.height / 4;
    if (gridOff + size_t(gw) * gh * 4 != b.size()) throw std::runtime_error("map size mismatch");
    r.raw.resize(size_t(r.width) * r.height);
    for (uint32_t by = 0; by < gh; ++by) {
        for (uint32_t bx = 0; bx < gw; ++bx) {
            const uint32_t e = rd32(b, gridOff + (size_t(by) * gw + bx) * 4);
            const uint16_t base = uint16_t(e & 0xFFF);
            const uint32_t idx = e >> 12;
            uint16_t vals[16];
            if (idx < nA) {
                const size_t o = d + 8 + size_t(8) * idx;
                if (o + 8 > gridOff) throw std::runtime_error("map block out of range");
                for (int i = 0; i < 8; ++i) {
                    vals[2 * i] = uint16_t(base + (b[o + size_t(i)] >> 4));
                    vals[2 * i + 1] = uint16_t(base + (b[o + size_t(i)] & 15));
                }
            } else {
                const size_t o = d + size_t(16) * idx + 8 - size_t(8) * nA;
                if (o + 16 > gridOff) throw std::runtime_error("map block out of range");
                for (int i = 0; i < 16; ++i) vals[i] = uint16_t(base + b[o + size_t(i)]);
            }
            for (uint32_t j = 0; j < 4; ++j) std::memcpy(&r.raw[size_t(by * 4 + j) * r.width + bx * 4], &vals[4 * j], 8);
        }
    }
    return r;
}

size_t applyFlatSurfaces(Heightfield& hf, const DpcFile& world) {
    const std::vector<uint32_t> gens = world.ids(kGenWorldClass);
    if (gens.empty()) return 0;
    const auto body = world.body(*world.find(gens.front()));
    if (!body) return 0;
    const std::vector<uint8_t>& b = *body;
    // Skip to the flat surfaces: u32 x3; maps; ground textures; materials; u32;
    // groups {u32 len, name, u32, u32 n, ids[n]}; then u32 n and n 0x83-byte records.
    size_t p = 12;
    for (int list = 0; list < 3; ++list) p += 4 + size_t(rd32(b, p)) * 4;
    p += 4;
    const uint32_t groups = rd32(b, p);
    p += 4;
    for (uint32_t g = 0; g < groups; ++g) {
        p += 4 + rd32(b, p) + 4;
        p += 4 + size_t(rd32(b, p)) * 4;
    }
    const uint32_t n = rd32(b, p);
    p += 4;
    auto f32 = [&](size_t o) {
        if (o + 4 > b.size()) throw std::runtime_error("flat surface overruns");
        float v;
        std::memcpy(&v, &b[o], 4);
        return v;
    };
    size_t applied = 0;
    for (uint32_t i = 0; i < n; ++i, p += 0x83) {
        // Record: u32 id; axis x (3 floats + pad), axis y, axis z, centre; then parameters
        // (+0x4c = blend margin in metres).
        const float ax[3] = {f32(p + 4), f32(p + 8), f32(p + 12)};
        const float az[3] = {f32(p + 36), f32(p + 40), f32(p + 44)};
        const float c[3] = {f32(p + 52), f32(p + 56), f32(p + 60)};
        const float margin = std::clamp(f32(p + 0x4c), 0.0f, 100.0f);
        // Solve point = centre + u*axisX + v*axisZ in the ground plane.
        const float det = ax[0] * az[2] - ax[2] * az[0];
        if (std::fabs(det) < 1e-3f) continue;
        const float reach = std::fabs(ax[0]) + std::fabs(az[0]) + margin, reachZ = std::fabs(ax[2]) + std::fabs(az[2]) + margin;
        const int c0 = std::max(0, int((c[0] - reach - hf.originX) / hf.spacing)), c1 = std::min(int(hf.width) - 1, int((c[0] + reach - hf.originX) / hf.spacing) + 1);
        const int r0 = std::max(0, int((c[2] - reachZ - hf.originZ) / hf.spacing)), r1 = std::min(int(hf.height) - 1, int((c[2] + reachZ - hf.originZ) / hf.spacing) + 1);
        const float lenX = std::hypot(ax[0], ax[2]), lenZ = std::hypot(az[0], az[2]);
        for (int r = r0; r <= r1; ++r) {
            for (int col = c0; col <= c1; ++col) {
                const float dx = hf.originX + col * hf.spacing - c[0], dz = hf.originZ + r * hf.spacing - c[2];
                const float u = (dx * az[2] - dz * az[0]) / det, v = (ax[0] * dz - ax[2] * dx) / det;
                // Distance outside the rectangle, in metres.
                const float out = std::hypot(std::max(0.0f, std::fabs(u) - 1) * lenX, std::max(0.0f, std::fabs(v) - 1) * lenZ);
                if (out > margin) continue;
                const float w = margin > 0 ? 1 - std::clamp(out / margin, 0.0f, 1.0f) : 1.0f;
                const float target = c[1] + std::clamp(u, -1.0f, 1.0f) * ax[1] + std::clamp(v, -1.0f, 1.0f) * az[1];
                uint16_t& raw = hf.raw[size_t(r) * hf.width + size_t(col)];
                const float h = raw * hf.metresPerUnit;
                const float blended = h + (target - h) * (w * w * (3 - 2 * w));
                raw = uint16_t(std::clamp(std::lround(blended / hf.metresPerUnit), 0L, 65535L));
            }
        }
        ++applied;
    }
    return applied;
}

Heightfield loadWorldHeightfield(const DpcFile& world) {
    const std::vector<uint32_t> maps = genWorldMaps(world);
    if (maps.empty()) throw std::runtime_error("GenWorld_Z lists no maps");
    const DpcFile::Entry* mapEntry = world.find(maps[0]);
    if (mapEntry == nullptr || mapEntry->classCrc != kBinaryClass) throw std::runtime_error("height map not found");
    const auto blob = world.body(*mapEntry);
    if (!blob) throw std::runtime_error("height map has no body");
    Raster r = decodeRaster(*blob);
    if (r.width < 8 || r.height < 8) throw std::runtime_error("height map too small");
    Heightfield hf;
    hf.width = r.width;
    hf.height = r.height;
    hf.raw = std::move(r.raw);
    hf.spacing = kWorldSpan / float(hf.width);
    hf.originX = -float(hf.width / 2) * hf.spacing;
    hf.originZ = -float(hf.height / 2) * hf.spacing;
    hf.metresPerUnit = kHeightRange / r.maxValue;
    return hf;
}

Heightfield loadLevelledHeightfield(const DpcFile& world) {
    Heightfield hf = loadWorldHeightfield(world);
    applyFlatSurfaces(hf, world);
    return hf;
}

}  // namespace fuel
