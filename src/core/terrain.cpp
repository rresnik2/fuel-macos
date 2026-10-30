#include "terrain.hpp"
#include "genworld_noise.hpp"


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
    // Skip to the flat surfaces: u32 x3; maps; ground textures; materials; u32 seed;
    // groups {u32 len, name, u32, u32 n, ids[n]}; then u32 n and n 0x83-byte records.
    size_t p = 12;
    for (int list = 0; list < 3; ++list) p += 4 + size_t(rd32(b, p)) * 4;
    const Permutation perm = buildPermutation(world, rd32(b, p));
    p += 4;
    const uint32_t groups = rd32(b, p);
    p += 4;
    for (uint32_t g = 0; g < groups; ++g) {
        p += 4 + rd32(b, p) + 4;
        p += 4 + size_t(rd32(b, p)) * 4;
    }
    const uint32_t n = rd32(b, p);
    p += 4;
    if (p + size_t(n) * 0x83 > b.size()) throw std::runtime_error("flat surfaces overrun GenWorld_Z");
    auto f32 = [&](size_t o) {
        float v;
        std::memcpy(&v, &b[o], 4);
        return v;
    };
    struct Surface {
        double cx, cy, cz;
        double ax, az, lenX;  // axis X in the ground plane, scaled to its full length
        double bx, bz, lenZ;  // axis Z likewise
        double nx, ny, nz;    // plane normal
        float invMargin, strength;
        uint8_t priority;
        bool ellipse;
    };
    std::vector<Surface> surfaces;
    for (uint32_t i = 0; i < n; ++i) {
        const size_t r = p + size_t(i) * 0x83;
        Surface s{};
        const double X[3] = {f32(r + 4), f32(r + 8), f32(r + 12)}, N[3] = {f32(r + 20), f32(r + 24), f32(r + 28)};
        const double Z[3] = {f32(r + 36), f32(r + 40), f32(r + 44)};
        s.cx = f32(r + 52), s.cy = f32(r + 56), s.cz = f32(r + 60);
        auto ground = [](const double v[3], double& gx, double& gz, double& len) {
            len = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
            const double h = std::hypot(v[0], v[2]);
            gx = h > 0 ? v[0] * len / h : 0, gz = h > 0 ? v[2] * len / h : 0;
        };
        ground(X, s.ax, s.az, s.lenX);
        ground(Z, s.bx, s.bz, s.lenZ);
        if (s.lenX < 1e-3 || s.lenZ < 1e-3 || std::fabs(N[1]) < 1e-3) continue;
        s.nx = N[0], s.ny = N[1], s.nz = N[2];
        s.invMargin = 1.0f / std::max(f32(r + 0x4c), 0.1f);
        s.strength = f32(r + 0x50);
        s.priority = b[r + 0x7d];
        s.ellipse = b[r + 0x82] != 0;
        if (s.strength > 0) surfaces.push_back(s);
    }
    // Edge jitter: two small fractals (10 m and 30 m cells) averaged, times 5 m.
    const Fractal edgeA(perm, 2, 10.0f, 33.129738f, 0.6f, 0.6f, 0.5f), edgeB(perm, 2, 30.0f, 43.129738f, 0.6f, 0.6f, 0.5f);
    // Signed distance (metres) from the shape's edge, positive inside.
    auto inside = [](const Surface& s, double x, double z) {
        const double dx = x - s.cx, dz = z - s.cz;
        const double u = (dx * s.ax + dz * s.az) / (s.lenX * s.lenX), v = (dx * s.bx + dz * s.bz) / (s.lenZ * s.lenZ);
        if (s.ellipse) {
            const double rr = std::hypot(u, v);
            if (rr == 0) return std::min(s.lenX, s.lenZ);
            const double ex = s.cx + s.ax * (u / rr) + s.bx * (v / rr) - x, ez = s.cz + s.az * (u / rr) + s.bz * (v / rr) - z;
            const double d = std::hypot(ex, ez);
            return rr > 1 ? -d : d;
        }
        const double eX = std::max(0.0, 1 - std::fabs(u)) * s.lenX, eZ = std::max(0.0, 1 - std::fabs(v)) * s.lenZ;
        double d = std::min(eX, eZ);
        if (v < 0) d -= std::max(0.0, 15.0 - std::hypot(eX, eZ));  // the -Z corners are rounded off
        return d;
    };
    // Every grid vertex inside a surface's bounds gets the list of surfaces to combine.
    std::vector<std::pair<uint32_t, uint32_t>> hits;  // (vertex, surface)
    for (uint32_t i = 0; i < surfaces.size(); ++i) {
        const Surface& s = surfaces[i];
        const double rx = std::fabs(s.ax) + std::fabs(s.bx), rz = std::fabs(s.az) + std::fabs(s.bz);
        const int c0 = std::max(0, int(std::floor((s.cx - rx - hf.originX) / hf.spacing)));
        const int c1 = std::min(int(hf.width) - 1, int(std::ceil((s.cx + rx - hf.originX) / hf.spacing)));
        const int r0 = std::max(0, int(std::floor((s.cz - rz - hf.originZ) / hf.spacing)));
        const int r1 = std::min(int(hf.height) - 1, int(std::ceil((s.cz + rz - hf.originZ) / hf.spacing)));
        for (int r = r0; r <= r1; ++r)
            for (int c = c0; c <= c1; ++c) hits.push_back({uint32_t(r) * hf.width + uint32_t(c), i});
    }
    std::sort(hits.begin(), hits.end(), [&](const auto& a, const auto& c) {
        if (a.first != c.first) return a.first < c.first;
        const Surface &sa = surfaces[a.second], &sc = surfaces[c.second];
        return sa.priority != sc.priority ? sa.priority < sc.priority : sa.cx < sc.cx;
    });
    std::vector<uint8_t> used(surfaces.size(), 0);
    for (size_t k = 0; k < hits.size();) {
        const uint32_t vtx = hits[k].first;
        const double x = hf.originX + double(vtx % hf.width) * hf.spacing, z = hf.originZ + double(vtx / hf.width) * hf.spacing;
        const double jitter = 5.0 * 0.5 * (double(edgeA.at(x, z)) + double(edgeB.at(x, z)));
        double h = double(hf.raw[vtx]) * hf.metresPerUnit;
        double acc = 0, wsum = 0;
        int group = -1;
        for (; k < hits.size() && hits[k].first == vtx; ++k) {
            const Surface& s = surfaces[hits[k].second];
            if (int(s.priority) != group) {  // a higher priority starts over from the result so far
                if (wsum > 0.01) h = acc / wsum;
                acc = wsum = 0;
                group = s.priority;
            }
            const double d = inside(s, x, z) - jitter;
            if (d <= 0) continue;
            const double t = std::clamp(d * s.invMargin, 0.0, 1.0), e = 2 * t - 1;
            const double curve = e <= 0 ? std::pow(e + 1, 1.25) - 1 : 1 - std::pow(1 - e, 1.25);
            const double w = (0.5 + 0.5 * curve) * s.strength;
            const double plane = s.cy - (s.nx * (x - s.cx) + s.nz * (z - s.cz)) / s.ny;
            acc += w * ((1 - w) * h + w * plane);
            wsum += w;
            used[hits[k].second] = 1;
        }
        if (wsum > 0.01) h = acc / wsum;
        hf.raw[vtx] = uint16_t(std::clamp(std::lround(h / hf.metresPerUnit), 0L, 65535L));
    }
    return size_t(std::count(used.begin(), used.end(), uint8_t(1)));
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
