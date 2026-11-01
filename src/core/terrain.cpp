#include "terrain.hpp"
#include "genworld_noise.hpp"
#include "terrain_height.hpp"
#include "road_segments.hpp"


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
    return detail ? float(detail->ground(x, z)) : baseHeightAt(x, z);
}

float Heightfield::baseHeightAt(double x, double z) const {
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
    hf.detail = makeTerrainDetail(world, hf.width, hf.height, kHeightRange / hf.metresPerUnit, hf.raw);
    // Roads carve the ground through FUEL's segments (0x6fcaa0 as decoded by p22): against every
    // placement it fits better than terrain_height's own road decode (within 0.5 m 7,266 vs 5,314,
    // > 4 m off 1,730 vs 2,146). Profiles use the ground without roads (flags 3, 0x6f8a00).
    if (hf.detail) {
        try {
            RoadTypeTable types = loadRoadTypeTable(world);
            if (types.valid) {
                const TerrainDetail* d = hf.detail.get();
                auto rs = std::make_shared<RoadSampler>(world, std::move(types), [d](double x, double z) { return d->detailHeight(x, z); });
                const_cast<TerrainDetail*>(d)->setRoadCarver(
                    [rs](double x, double z, double& y, double c, double w) { return rs->carve(x, z, y, c, w); });
                // Road queries of the procedural flat surfaces (0x799910): 0x6fb320 / 0x6fb240.
                auto hit = [](const RoadSampler& r, const RoadSampler::Nearest& n) {
                    TerrainDetail::RoadHit h;
                    h.edge = n.edge;
                    if (n.seg) {
                        h.found = true;
                        h.px = n.px, h.pz = n.pz, h.dx = n.seg->dx, h.dz = n.seg->dz;
                        h.c8 = r.types().types[n.seg->type].at(0xc8);
                    }
                    return h;
                };
                const RoadSampler* rp = rs.get();
                const_cast<TerrainDetail*>(d)->setRoadQueries([rp, hit](double x, double z) { return hit(*rp, rp->nearestBigRoad(x, z)); },
                                                              [rp, hit](double x, double z, double r) { return hit(*rp, rp->nearestRoad(x, z, r)); });
                hf.roads = std::move(rs);
            }
        } catch (const std::exception&) {
            // no road graph: the ground keeps terrain_height's own road decode
        }
    }
    return hf;
}

}  // namespace fuel
