#include "terrain.hpp"


#include <algorithm>
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

Heightfield loadWorldHeightfield(const DpcFile& world) {
    const std::vector<uint32_t> gens = world.ids(kGenWorldClass);
    if (gens.empty()) throw std::runtime_error(world.path() + ": no GenWorld_Z");
    const auto gen = world.body(*world.find(gens.front()));
    if (!gen) throw std::runtime_error("GenWorld_Z has no body");
    // GenWorld_Z body: u32, u32, u32 road graph, then u32 n and n Binary_Z map IDs; map 0 = heights.
    if (rd32(*gen, 12) == 0) throw std::runtime_error("GenWorld_Z lists no maps");
    const DpcFile::Entry* mapEntry = world.find(rd32(*gen, 16));
    if (mapEntry == nullptr || mapEntry->classCrc != kBinaryClass) throw std::runtime_error("height map not found");
    const auto blob = world.body(*mapEntry);
    if (!blob) throw std::runtime_error("height map has no body");

    // Binary_Z body: u32 size; raster header (u32 w, u32 h, f32, i32, f32 maxValue); then
    // u32 nA, u32 K, the block table, and one u32 per 4x4 block: low 12 bits = base value,
    // high 20 bits = block index. Blocks below nA hold 16 four-bit offsets (8 bytes at 8 + 8*i);
    // the others hold 16 byte offsets (16 bytes at 16*i + 8 - 8*nA).
    const std::vector<uint8_t>& b = *blob;
    Heightfield hf;
    hf.width = rd32(b, 4);
    hf.height = rd32(b, 8);
    float maxValue;
    std::memcpy(&maxValue, &b[20], 4);
    if (hf.width % 4 || hf.height % 4 || hf.width < 8 || hf.height < 8 || !(maxValue > 0))
        throw std::runtime_error("height map header is implausible");
    const size_t d = 24;  // start of the raster data
    const uint32_t nA = rd32(b, d), K = rd32(b, d + 4);
    const size_t gridOff = d + 4 * (size_t(1) + K);
    const uint32_t gw = hf.width / 4, gh = hf.height / 4;
    if (gridOff + size_t(gw) * gh * 4 != b.size()) throw std::runtime_error("height map size mismatch");

    hf.raw.resize(size_t(hf.width) * hf.height);
    const size_t dataEnd = gridOff;
    for (uint32_t by = 0; by < gh; ++by) {
        for (uint32_t bx = 0; bx < gw; ++bx) {
            const uint32_t e = rd32(b, gridOff + (size_t(by) * gw + bx) * 4);
            const uint16_t base = uint16_t(e & 0xFFF);
            const uint32_t idx = e >> 12;
            uint16_t vals[16];
            if (idx < nA) {
                const size_t o = d + 8 + size_t(8) * idx;
                if (o + 8 > dataEnd) throw std::runtime_error("height block out of range");
                for (int i = 0; i < 8; ++i) {
                    vals[2 * i] = uint16_t(base + (b[o + size_t(i)] >> 4));
                    vals[2 * i + 1] = uint16_t(base + (b[o + size_t(i)] & 15));
                }
            } else {
                const size_t o = d + size_t(16) * idx + 8 - size_t(8) * nA;
                if (o + 16 > dataEnd) throw std::runtime_error("height block out of range");
                for (int i = 0; i < 16; ++i) vals[i] = uint16_t(base + b[o + size_t(i)]);
            }
            for (uint32_t j = 0; j < 4; ++j)
                std::memcpy(&hf.raw[size_t(by * 4 + j) * hf.width + bx * 4], &vals[4 * j], 8);
        }
    }
    hf.spacing = kWorldSpan / float(hf.width);
    hf.originX = -float(hf.width / 2) * hf.spacing;
    hf.originZ = -float(hf.height / 2) * hf.spacing;
    hf.metresPerUnit = kHeightRange / maxValue;
    return hf;
}

}  // namespace fuel
