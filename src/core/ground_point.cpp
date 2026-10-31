#include "ground_point.hpp"
#include "ground.hpp"
#include "resources.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>

namespace fuel {

namespace {

constexpr uint32_t kGenWorldClass = 0x31FA94AE;
constexpr float kDeg = 0.017453292f;

uint32_t rd32(const std::vector<uint8_t>& b, size_t o) {
    if (o + 4 > b.size()) throw std::runtime_error("GenWorld_Z body too short");
    uint32_t v;
    std::memcpy(&v, &b[o], 4);
    return v;
}

float clamp01(float v) { return std::clamp(v, 0.0f, 1.0f); }

// The engine's contrast curves on [-1, 1]: sign(t) * (1 - (1 - |t|)^n), and the
// quadratic one 0.5 + 0.5 t (2 - |t|) (already mapped to 0..1).
float contrast4(float t) {
    const float a = 1 - std::fabs(t), a2 = a * a;
    return std::copysign(1 - a2 * a2, t);
}
float contrast8(float t) {
    const float a = 1 - std::fabs(t), a2 = a * a, a4 = a2 * a2;
    return std::copysign(1 - a4 * a4, t);
}
float soft2(float t) { return 0.5f + 0.5f * t * (2 - std::fabs(t)); }
// S6(u) = 0.5 + 0.5 u (4 - 6|u| + (4 - |u|) u^2)
float soft6(float u) {
    const float a = std::fabs(u);
    return 0.5f + 0.5f * u * (4 - 6 * a + (4 - a) * u * u);
}
// S4 contrast of a clamped value, back to 0..1 (0x6a7ae0, 0x6a6650 tails).
float s4c(float v) { return contrast4(2 * clamp01(v) - 1) * 0.5f + 0.5f; }

// 0x6a57f0: 0 below lo - soft, a linear ramp up to lo, 1 on [lo, hi). Above hi the
// engine computes (hi - x) / soft, clamped, so the upper edge is a hard cut (Observed asm).
float band(float x, float lo, float hi, float soft) {
    if (x < lo - soft || x >= hi) return 0;
    if (x < lo) return (x - (lo - soft)) / soft;
    return 1;
}

// The engine rounds these position sums in single precision before taking the byte.
// fistp in FUEL's game threads rounds down (_RC_DOWN, p21 H0).
uint8_t hashByte(const Permutation& p, double v) { return p[size_t(int64_t(std::floor(float(v))) & 255)]; }

uint64_t cellKey(int64_t cx, int64_t cz) { return (uint64_t(uint32_t(int32_t(cx))) << 32) | uint32_t(int32_t(cz)); }
constexpr double kSurfaceCell = 64.0;

}  // namespace

GroundGenerator::GroundGenerator(const DpcFile& world, const Heightfield& terrain, const RoadNetwork* roads)
    : terrain_(&terrain), roads_(roads) {
    const std::vector<uint32_t> gens = world.ids(kGenWorldClass);
    if (gens.empty()) throw std::runtime_error("no GenWorld_Z");
    const auto body = world.body(*world.find(gens.front()));
    if (!body) throw std::runtime_error("GenWorld_Z has no body");
    const std::vector<uint8_t>& b = *body;
    size_t p = 12;
    const uint32_t nMaps = rd32(b, p);
    std::vector<uint32_t> mapIds;
    for (uint32_t i = 0; i < nMaps; ++i) mapIds.push_back(rd32(b, p + 4 + size_t(i) * 4));
    p += 4 + size_t(nMaps) * 4;
    p += 4 + size_t(rd32(b, p)) * 4;  // ground textures
    {
        // Materials by texture enum; 0x28 is "Grass" (p9 R8), whose diffuse is the tuft atlas.
        const uint32_t nMat = rd32(b, p);
        if (nMat > 0x28)
            if (const DpcFile::Entry* me = world.find(rd32(b, p + 4 + 0x28 * 4)))
                if (const auto mb = world.body(*me)) {
                    try {
                        grassAtlas_ = parseMaterialBody(*mb).textures[0];
                    } catch (const std::exception&) {
                    }
                }
        p += 4 + size_t(nMat) * 4;
    }
    perm_ = buildPermutation(world, rd32(b, p));
    p += 4;
    const uint32_t groups = rd32(b, p);
    p += 4;
    for (uint32_t g = 0; g < groups; ++g) {
        p += 4 + rd32(b, p) + 4;
        p += 4 + size_t(rd32(b, p)) * 4;
    }
    // Maps used here: 2 humidity, 8 population, 10 rock (Terrain_Map10); colours (p16):
    // 3 clay, 4 colour layer, 5 volcano, 6 chalk, 7 forest, 9 occlusion, 13 incinerated, 18 (ctx+0x184).
    for (int slot : {2, 3, 4, 5, 6, 7, 8, 9, 10, 13, 18}) {
        if (size_t(slot) >= mapIds.size() || mapIds[size_t(slot)] == 0) continue;
        if (const DpcFile::Entry* e = world.find(mapIds[size_t(slot)]))
            if (const auto mb = world.body(*e)) {
                try {
                    maps_[size_t(slot)] = decodeRaster(*mb);
                } catch (const std::exception&) {
                }
            }
    }
    // Flat surfaces (0x83-byte records, p13 V6 / p14 G4).
    const uint32_t n = rd32(b, p);
    p += 4;
    if (p + size_t(n) * 0x83 > b.size()) throw std::runtime_error("flat surfaces overrun GenWorld_Z");
    auto f32 = [&](size_t o) {
        float v;
        std::memcpy(&v, &b[o], 4);
        return v;
    };
    for (uint32_t i = 0; i < n; ++i) {
        const size_t r = p + size_t(i) * 0x83;
        Surface s{};
        const double X[3] = {f32(r + 4), f32(r + 8), f32(r + 12)}, Z[3] = {f32(r + 36), f32(r + 40), f32(r + 44)};
        s.cx = f32(r + 52), s.cz = f32(r + 60);
        auto ground = [](const double v[3], double& gx, double& gz, double& len) {
            len = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
            const double h = std::hypot(v[0], v[2]);
            gx = h > 0 ? v[0] * len / h : 0, gz = h > 0 ? v[2] * len / h : 0;
        };
        ground(X, s.ax, s.az, s.lenX);
        ground(Z, s.bx, s.bz, s.lenZ);
        if (s.lenX < 1e-3 || s.lenZ < 1e-3) continue;
        s.invMargin = 1.0f / std::max(f32(r + 0x4c), 0.1f);
        s.grassCut = f32(r + 0x54), s.grassNoise = f32(r + 0x58), s.grassAdd = f32(r + 0x5c), s.w300 = f32(r + 0x60);
        s.w9c = 0;  // object +0x9c: file field not identified (notes G4); 0 = no size change
        s.ellipse = b[r + 0x82] != 0;
        s.allowPlants = b[r + 0x81] != 0;
        s.radius = std::fabs(s.ax) + std::fabs(s.bx) + std::fabs(s.az) + std::fabs(s.bz) + 8.0;  // + max edge jitter
        const auto index = uint32_t(surfaces_.size());
        surfaces_.push_back(s);
        for (int64_t cz = int64_t(std::floor((s.cz - s.radius) / kSurfaceCell)); cz <= int64_t(std::floor((s.cz + s.radius) / kSurfaceCell)); ++cz)
            for (int64_t cx = int64_t(std::floor((s.cx - s.radius) / kSurfaceCell)); cx <= int64_t(std::floor((s.cx + s.radius) / kSurfaceCell)); ++cx)
                surfaceCells_[cellKey(cx, cz)].push_back(index);
    }
    // Noise bank genWorld+0x4498 (NoiseBank_Init 0x6ad83b..) and terrain-detail object T (0x7a32f0..).
    f1fac_ = Fractal(perm_, 1, 10.0f, 35.173195f, 0.3f, 0.3f, 5.0f);
    ffd8_ = Fractal(perm_, 2, 200.0f, 27.173193f, 0.3f, 0.3f, 5.0f);
    f2f80_ = Fractal(perm_, 2, 4.0f, 35.173195f, 0.3f, 0.3f, 5.0f);
    ffd44_ = Fractal(perm_, 3, 100.0f, 17.975342f, 0.025f, 0.15f, 0.5f);
    f7ea4_ = Fractal(perm_, 4, 3000.0f, 9.647993f, 0.025f, 0.15f, 0.5f);
    f6ed0_ = Fractal(perm_, 3, 100.0f, 40.929737f, 0.6f, 0.6f, 0.5f);
    edgeA_ = Fractal(perm_, 2, 10.0f, 33.129738f, 0.6f, 0.6f, 0.5f);
    edgeB_ = Fractal(perm_, 2, 30.0f, 43.129738f, 0.6f, 0.6f, 0.5f);
    t0c_ = Fractal(perm_, 6, 20000.0f, 95.992332f, 0.3f, 0.3f, 5.0f);
    t18_ = Fractal(perm_, 4, 2000.0f, 39.442329f, 0.3f, 0.3f, 0.5f);
    t24_ = Fractal(perm_, 2, 15.0f, 74.172333f, 0.3f, 0.3f, 0.5f);
    t28_ = Fractal(perm_, 5, 500.0f, 79.442329f, 0.3f, 0.3f, 0.5f);
    // Colour noises: bank+0x0004 (T0, palette half), bank+0x9e4c (colour layer), bank+0xcdc8 (strata).
    f0004_ = Fractal(perm_, 2, 6.13931f, 39.37579f, 0.3f, 0.5f, 0.5f);
    f3f54_ = Fractal(perm_, 1, 10.0f, 25.148701f, 0.3f, 0.3f, 5.0f);
    f9e4c_ = Fractal(perm_, 1, 100.0f, 65.145248f, 0.3f, 0.3f, 5.0f);
    fcdc8_ = Fractal(perm_, 3, 500.0f, 43.154682f, 0.025f, 0.15f, 0.5f);
    f8e78_ = Fractal(perm_, 1, 100.0f, 67.349632f, 0.3f, 0.3f, 5.0f);  // clay noise (0x45d010)
    fae20_ = Fractal(perm_, 1, 100.0f, 42.407471f, 0.3f, 0.3f, 5.0f);  // chalk noise (0x45c270)
    fbdf4_ = Fractal(perm_, 1, 100.0f, 52.488899f, 0.3f, 0.3f, 5.0f);  // volcano noise (0x459530)
    palette_ = loadGroundPalette(world);
    // Grass_EmitTuft tables: tile weights (0x9ff02c) and tile sizes (0x9ff04c); the
    // tile index reaches 8, and the engine reads past the weights into the sizes.
    const std::vector<float> t = readEngineFloats(world, 0x9FF02C, 17);
    if (t.size() == 17) {
        tileWeight_.assign(t.begin(), t.begin() + 9);
        tileSize_.assign(t.begin() + 8, t.end());
    }
}

float GroundGenerator::mapAt(int slot, double x, double z) const {
    const Raster& r = maps_[size_t(slot)];
    if (r.raw.empty()) return 0;
    const double spacing = 140000.0 / r.width;
    const double tx = x / spacing + r.width / 2, tz = z / spacing + r.height / 2;
    const double ix = std::floor(tx), iz = std::floor(tz);
    auto weights = [](double a, double w[4]) {
        w[0] = (1 - a) * (1 - a) * (1 - a) / 6;
        w[1] = (4 - 6 * a * a + 3 * a * a * a) / 6;
        w[3] = a * a * a / 6;
        w[2] = 1 - w[0] - w[1] - w[3];
    };
    double wx[4], wz[4];
    weights(tx - ix, wx);
    weights(tz - iz, wz);
    double v = 0;
    for (int j = 0; j < 4; ++j) {
        const auto row = uint32_t(std::clamp<double>(iz - 1 + j, 0, r.height - 1));
        for (int i = 0; i < 4; ++i) v += wx[i] * wz[j] * r.at(uint32_t(std::clamp<double>(ix - 1 + i, 0, r.width - 1)), row);
    }
    return float(v);
}

float GroundGenerator::edgeJitter(double x, double z) const { return 0.5f * (edgeA_.at(x, z) + edgeB_.at(x, z)); }

// FlatSurface_InsideDistance (0x799020) without the jitter term.
double GroundGenerator::insideDistance(const Surface& s, double x, double z) const {
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
    if (v < 0) d -= std::max(0.0, 15.0 - std::hypot(eX, eZ));
    return d;
}

// Point_FlatSurfaceWeights (0x79b480, bit 27).
GroundGenerator::SurfaceWeights GroundGenerator::surfaceWeights(double x, double z) const {
    SurfaceWeights w;
    const auto it = surfaceCells_.find(cellKey(int64_t(std::floor(x / kSurfaceCell)), int64_t(std::floor(z / kSurfaceCell))));
    if (it == surfaceCells_.end()) return w;
    float sum = 0, cover = 0, cut = 0, noise = 0, add = 0, keep = 0;
    float jitter = -1;
    for (uint32_t i : it->second) {
        const Surface& s = surfaces_[i];
        if (std::fabs(x - s.cx) > s.radius || std::fabs(z - s.cz) > s.radius) continue;
        if (jitter < 0) jitter = 5.0f * edgeJitter(x, z);
        const double d = insideDistance(s, x, z) - jitter;
        if (d <= 0) continue;
        const float k = clamp01(float(d) * s.invMargin);
        sum += k, cover += k, cut += k * s.grassCut, noise += k * s.grassNoise, add += k * s.grassAdd, keep += k * (1 - s.w9c);
    }
    if (sum > 1e-6f) {
        const float inv = 1.0f / sum;
        w.cover = cover * inv, w.cut = cut * inv, w.noise = noise * inv, w.add = add * inv, w.keep = 1 - keep * inv;
    }
    return w;
}

bool GroundGenerator::excludesPlants(double x, double z) const {
    // 0x7831e0 test 3: BigLake_MaxDepth 0x65a390 > 0.001 (inside a lake below its level).
    if (lakes_ != nullptr && terrain_ != nullptr && lakes_->depthAbove(x, z, terrain_->smoothHeightAt(x, z)) > 0.001) return true;
    const auto it = surfaceCells_.find(cellKey(int64_t(std::floor(x / kSurfaceCell)), int64_t(std::floor(z / kSurfaceCell))));
    if (it == surfaceCells_.end()) return false;
    for (uint32_t i : it->second) {
        const Surface& s = surfaces_[i];
        if (s.allowPlants || std::fabs(x - s.cx) > s.radius || std::fabs(z - s.cz) > s.radius) continue;
        if (insideDistance(s, x, z) > 0) return true;  // 0x79a850 calls the distance with no jitter
    }
    return false;
}

// Road weight (Point_RoadSampler 0x6fbab0 return value): 1 on the road, fading out over
// 5.5 m around its edge. APPROXIMATED: the engine's per-type profile table (0xa677e0, filled
// at run time) is replaced by the port's road widths.
float GroundGenerator::roadWeight(double x, double z, bool* nearRoad, float* tarmac) const {
    *nearRoad = false;
    if (tarmac) *tarmac = 0;
    if (roads_ == nullptr) return 0;
    const auto it = roads_->segCells.find(
        RoadNetwork::cellKey(int32_t(std::floor(x / RoadNetwork::kSegCell)), int32_t(std::floor(z / RoadNetwork::kSegCell))));
    if (it == roads_->segCells.end()) return 0;
    float best = 0;
    for (uint64_t ref : it->second) {
        const RoadNetwork::Line& line = roads_->lines[size_t(ref >> 20)];
        const float width = line.type < roads_->styles.size() ? roads_->styles[line.type].width : 0;
        if (width <= 0) continue;
        const size_t k = size_t(ref & 0xFFFFF);
        const double ax = line.points[k][0], az = line.points[k][1];
        const double bx = line.points[k + 1][0] - ax, bz = line.points[k + 1][1] - az;
        const double len2 = bx * bx + bz * bz;
        const double t = len2 > 0 ? std::clamp(((x - ax) * bx + (z - az) * bz) / len2, 0.0, 1.0) : 0.0;
        const double d = std::hypot(x - (ax + bx * t), z - (az + bz * t));
        if (d < 0.5 * width + 8.0) *nearRoad = true;
        const float w = clamp01(float((0.5 * width - d + 2.75) * 0.18181819));
        // ctx+0x1f4 is a per-profile "tarmac" value from the run-time profile table (0x6fc32b):
        // APPROXIMATED as 1 on tarmac road types, 0 on dirt.
        if (tarmac && isTarmacRoad(int(line.type)) && w > 0) *tarmac = 1;
        best = std::max(best, w);
    }
    return best;
}

// Point_RockCover (0x6b8d80) over Terrain_RockEval (0x7a4c20).
void GroundGenerator::rockCover(double x, double z, GroundPoint& p, float coarseNy) const {
    p.rock = p.rockSlope = 0;
    if (p.road > 0.95f) return;
    const float m = mapAt(10, x, z);
    float l = 0;
    const float a = t0c_.at(x, z) + m;
    if (a > 0.5f) {
        const float w1 = clamp01(2 * (std::min(a, 1.0f) - 0.5f));
        const float bb = t18_.at(x, z) * w1 + m;
        if (bb > 0.5f) {
            const float w2 = clamp01(2 * (bb - 0.5f));
            const float c = clamp01((t28_.at(x, z) - 0.67f) * 3);
            l = (contrast4(c)) * 20 * w2;
        }
    }
    const float s = clamp01((0.97f - coarseNy) * 20);
    const float k = m * 10 + 3 * s;
    if (k > 0.01f) {
        const float q = 0.75f * m + 0.25f;
        const float v = clamp01((t24_.at(x, z) - (1 - q)) / q);
        l += soft2(2 * v - 1) * k;
    }
    float rock = clamp01(clamp01(l) * s + m);  // + lake (map 11, empty in USA1)
    // ctx+0x1e4 adds slopes over 45 degrees (ramp from 40).
    const float slope = std::acos(clamp01(p.normal[1]));
    float rockSlope = clamp01((slope >= 45 * kDeg ? 1.0f : std::max(0.0f, (slope - 40 * kDeg) / (5 * kDeg))) + rock);
    if (p.height > 3000 - 500 * p.normal[2]) rock = rockSlope = 1;
    rock = clamp01(rock - p.cover - p.road - p.junction);
    rockSlope = clamp01(rockSlope - p.cover - p.road - p.junction);
    p.rock = clamp01(rock - p.highGround);  // - water (ctx+0x228, not ported)
    p.rockSlope = clamp01(rockSlope - p.highGround);
}

// Point_Fields (0x6b9100).
int GroundGenerator::fieldType(double x, double z, const GroundPoint& p, float coarseNy, bool nearRoad, float cover,
                               float* random) const {
    if (!(p.normal[1] > 0.8707f) || !(p.height < 1500)) return 0;
    const float pop = mapAt(8, x, z);
    if (!(pop > 0.25f && pop < 0.75f)) return 0;
    const float shift = clamp01(f7ea4_.at(x, z)) * 150.0f;
    const double a = (x + shift) * 0.005, c = (z + shift) * 0.005;
    const double i = std::floor(a), j = std::floor(c);  // fistp rounds down in FUEL's threads (p21 H0)
    const float u = float(a - i), v = float(c - j);
    const double cx = i * 200 + 100, cz = j * 200 + 100;
    const float r = float(hashByte(perm_, cx * 12.317513 + cz * 10.193578)) / 255.0f;
    *random = r;
    int type = 0;
    if (r > 0.3f && p.humidity > 0.2f && cover < 0.5f && !nearRoad && coarseNy > 0.98f) type = r < 0.33f ? 1 : 2;
    if (u <= 0.0125f || v <= 0.0125f || u >= 0.9875f || v >= 0.9875f) return 0;
    if (nearRoad) return 0;  // +0x1bc <= 3N + 1 (it is 0 near roads, 1000 elsewhere)
    const float nf = clamp01((soft2(2 * f1fac_.at(x, z) - 1) + f6ed0_.at(x, z)) * 0.5f);
    const float e = (nf + 0.5f) * 0.025f;
    if (u > e && v > e && u < 1 - e && v < 1 - e) return type;
    return 0;
}

GroundPoint GroundGenerator::at(double x, double z, bool full) const {
    GroundPoint p;
    if (terrain_ == nullptr) return p;
    const Heightfield& t = *terrain_;
    p.height = t.smoothHeightAt(x, z);
    auto normalAt = [&](double e, float n[3]) {
        const double dx = (t.smoothHeightAt(x + e, z) - t.smoothHeightAt(x - e, z)) / (2 * e);
        const double dz = (t.smoothHeightAt(x, z + e) - t.smoothHeightAt(x, z - e)) / (2 * e);
        const double len = std::sqrt(dx * dx + 1 + dz * dz);
        n[0] = float(-dx / len), n[1] = float(1 / len), n[2] = float(-dz / len);
    };
    normalAt(1.0, p.normal);
    // Coarse normal (ctx+0xe0, the W+0xcc4 sampler): APPROXIMATED by a 15 m difference.
    float coarse[3];
    normalAt(15.0, coarse);
    // Humidity (bit 2): APPROXIMATED as the plain map (FUN_004dde20 adds terms not decoded).
    p.humidity = clamp01(mapAt(2, x, z));
    const float v = p.humidity, nxz = std::hypot(p.normal[0], p.normal[2]);
    p.surfaceHumidity = clamp01(v - nxz * clamp01(1 - v * v - (1 - v) * (1 - v)));
    bool nearRoad = false;
    if (lakes_ != nullptr) {
        const WaterPoint w = waterAt(*lakes_, x, z, p.height);
        p.waterDepth = w.depth, p.waterBase = w.base, p.waterFade = w.fade;
    }
    const SurfaceWeights fs = surfaceWeights(x, z);
    p.cover = fs.cover;
    // Bit 12 (0x6b9bb0): road weight plus the flat surfaces that clear grass.
    if (sampler_ != nullptr) {
        const RoadFields rf = sampler_->sample(x, z);
        float r = clamp01(rf.weight + clamp01(1 - (fs.add + fs.noise)) * fs.cut);
        float u = rf.inside2;
        u = u >= 1 ? clamp01(1 - (u - 1) * 0.25f) : clamp01(u);
        float rough = clamp01(u * 0.5f + rf.roughness);
        float tarmac = rf.tarmac, decal = rf.decal;
        float puddle = clamp01((p.surfaceHumidity - 0.66f) * 10) * rf.puddle;
        puddle = clamp01((clamp01(p.normal[1] * 1.02f) - 0.99f) * 100 * (1 - fs.cover) * puddle);
        float junction = 0;
        if (rf.junction > 0.01f && p.normal[1] < 0.95f) {
            const float t2 = clamp01((0.95f - p.normal[1]) * 20);
            junction = rf.junction * t2;
            const float k = 1 - t2;
            r *= k, tarmac *= k, decal *= k, puddle *= k, rough *= k;
        }
        p.road = r;
        p.tarmac = tarmac;
        p.roughness = rough;
        p.decal = decal;
        p.decalU = rf.across;
        p.decalV = rf.along;
        p.decalMaterial = decal < 0.01f ? uint16_t(0xfff) : rf.material;
        p.puddle = puddle;
        p.junction = junction;
        p.roadEdge = rf.berm;
        nearRoad = rf.inside > 0;  // ctx+0x1bc = 0 when ctx+0x214 > 0, else 1000
    } else {
        p.road = clamp01(roadWeight(x, z, &nearRoad, &p.tarmac) + clamp01(1 - (fs.add + fs.noise)) * fs.cut);
    }
    // Bit 4: high ground (p13 V2 0x4de850) minus road.
    {
        float a = clamp01(p.height - (2500.0f + 250.0f * p.normal[2]));
        a = clamp01(a - 2 * (1 - p.normal[1]));
        const float hg = contrast8(2 * a - 1) * 0.5f + 0.5f;
        p.highGround = clamp01(hg - (2 - p.road) * p.road);
    }
    rockCover(x, z, p, coarse[1]);
    p.field = fieldType(x, z, p, coarse[1], nearRoad, fs.cover, &p.fieldRandom);
    p.nearRoad = nearRoad;
    if (!full) return p;

    // Bit 8 (Point_Grass 0x6ba610).
    const float slope = std::acos(clamp01(p.normal[1]));
    float g = 0, ground = 0;
    const float n10 = s4c(f1fac_.at(x, z));
    if (!(p.waterDepth > 0) && !(p.rock > 0.5f) && p.field != 1 && !(p.height > 3000 - 500 * p.normal[2])) {
        g = 1 - fs.cut + fs.noise * n10;
        g -= (2 - p.road) * p.road;
        g -= p.junction * (2 - p.junction);
        if (g > 0.1f) {
            const float h = p.surfaceHumidity;
            const float patch = clamp01(std::fabs((ffd8_.at(x, z) + f1fac_.at(x, z)) * 0.5f));
            if (h >= 0.05f) {
                const float m = clamp01(20 * patch);
                g *= (2 - m) * m;
            } else if (h >= 0.01f) {
                const float m = clamp01(2 * ((h - 0.01f) * 25 - 0.5f) + patch);
                const float a = 1 - m, a2 = a * a;
                g *= 1 - a2 * a2;
            } else {
                g = 0;
            }
        }
        g -= p.highGround;  // and water (ctx+0x228, not ported)
        // W+0xaaa == 0 (a runtime setting, value Unknown; the port takes 0): 4 m bare patches.
        g -= clamp01((f2f80_.at(x, z) - 1.3f) * 10);
        ground = g * band(slope, 0, 35 * kDeg, 5 * kDeg);
        g *= band(slope, 0, 30 * kDeg, 5 * kDeg);
        const float c = clamp01(n10 + 0.15f);
        const float a = 1 - c, a2 = a * a, a4 = a2 * a2;
        ground += a4 * a4 * p.road;  // (1 - (1 - (1-c)^8)) * road: grass strips on dirt tracks
        g = clamp01(g), ground = clamp01(ground);
    }
    const float addAll = (1 - p.road) * fs.add;
    p.grass = clamp01(g + addAll);
    p.groundGrass = clamp01(ground + addAll);
    // Bit 23: grass size.
    p.grassSize = clamp01(clamp01((1 - p.normal[1] - 0.1f) * 10) + s4c(ffd44_.at(x, z)) - (1 - fs.keep));
    // Bit 24 (0x6b8980): Stones; under water away from roads, Caillou (pebbles) too (water.md 1.8).
    if (p.road <= 0.95f) {
        const float F = ffd44_.at(x, z);
        float st = 0, pc = 0;
        if (p.waterDepth > 0 && !p.nearRoad) {
            const float t = clamp01(10 * p.normal[1] - 5);
            st = (2 - t) * t;
            if (st > 0) {
                const float a = clamp01(1 - F), b = 2 * (2 - a) * a - 1;
                st *= soft6(b);
                pc = soft6(2 * edgeA_.at(x, z) - 1);
            }
        } else {
            const float nst = clamp01(0.5f + 0.5f * contrast8(2 * F * (2 - std::fabs(F)) - 1));
            st = nst * band(slope, 30 * kDeg, 45 * kDeg, 5 * kDeg) - p.grass;
        }
        const float v = clamp01(st - p.road + p.junction);
        const float w = clamp01(soft6(2 * v - 1) - p.highGround);  // - Dune ctx+0x228 (0 without the Dune layer)
        p.caillou = w * pc;
        p.stones = w * (1 - pc);
    }
    surfaceTypes(x, z, p);
    return p;
}

float GroundGenerator::meanT0(double x, double z, double size, int n) const {
    float sum = 0;
    for (int j = 0; j < n; ++j)
        for (int i = 0; i < n; ++i)
            sum += clamp01(f0004_.at(x + ((i + 0.5) / n - 0.5) * size, z + ((j + 0.5) / n - 0.5) * size));
    return sum / float(n * n);
}

// The point's colours: bits 23 (weights), 9 and 10 (palette mixers), 7 (grass colour) and 14 (the
// composer), decoded in ~/re-work/fuel/notes/p16-colours.md (C2-C11). Colours are float4 in the
// engine; the alphas end up 0 or unused here, so only rgb is carried.
GroundColours GroundGenerator::colours(double x, double z, const GroundPoint& p, float t0) const {
    GroundColours out;
    if (!hasPalette()) return out;
    using C = std::array<float, 3>;
    auto P = [&](int k) { return C{palette_[size_t(k)][0], palette_[size_t(k)][1], palette_[size_t(k)][2]}; };
    auto lerp = [](const C& a, const C& b, float t) { return C{a[0] + (b[0] - a[0]) * t, a[1] + (b[1] - a[1]) * t, a[2] + (b[2] - a[2]) * t}; };
    auto madd = [](C& acc, const C& c, float w) { for (int i = 0; i < 3; ++i) acc[size_t(i)] += c[size_t(i)] * w; };

    // Bit 23 (0x6bb1f0): palette-half blend T0 and the material weights.
    const float T = t0 >= 0 ? t0 : clamp01(f0004_.at(x, z));
    auto L = [&](int k1, int k2) { return lerp(P(k1), P(k2), T); };
    // Clay 0x45d010 / chalk 0x45c270 / volcano 0x459530 = clamp(1.2 m - 0.1 + 0.05 F) (p32 pointcolours 1.1;
    // [0x9dc834] 1.2, [0x99648c] 0.1, [0x996158] 0.05), not the raw maps.
    const float clay = clamp01(1.2f * mapAt(3, x, z) - 0.1f + 0.05f * f8e78_.at(x, z));
    const float chalk = clamp01(1.2f * mapAt(6, x, z) - 0.1f + 0.05f * fae20_.at(x, z));
    const float volcano = clamp01(1.2f * mapAt(5, x, z) - 0.1f + 0.05f * fbdf4_.at(x, z));
    const float a0 = clamp01(1.3f * mapAt(4, x, z) - 0.15f + 0.1f * f9e4c_.at(x, z));  // 0x6a9d90
    float a = a0, u = 0;
    if (a0 < 1) {
        a = a0 + 0.5f * T * clay;
        u = clamp01(clay - a);
    }
    a = clamp01(a);
    const float sum = volcano + chalk + u + a, rest = clamp01(1 - sum), norm = 1.0f / (sum + rest);
    // The weight of a layer counts only above 0.025 (the mixers skip it).
    const float wa = a > 0.025f ? a : 0, wu = u > 0.025f ? u : 0, wc = chalk > 0.025f ? chalk : 0, wv = volcano > 0.025f ? volcano : 0;
    // One palette mix: slot of each material (rest, a', u, chalk, volcano) in both halves.
    auto mix = [&](const int (&first)[5], const int (&second)[5]) {
        C c{0, 0, 0};
        madd(c, L(first[0], second[0]), rest);
        madd(c, L(first[1], second[1]), wa);
        madd(c, L(first[2], second[2]), wu);
        madd(c, L(first[3], second[3]), wc);
        madd(c, L(first[4], second[4]), wv);
        for (float& v : c) v *= norm;
        return c;
    };

    // Humidity terms and the strata value ctx+0x1a8 (bit 20).
    const float H = p.surfaceHumidity;
    float strata;
    {
        const float v = fcdc8_.at(x, z);
        const float y = 0.25f * std::sin(0.2f * (100 * v + p.height)) + 0.75f * std::sin(0.01f * (500 * v + p.height));
        strata = clamp01(y * (2 - std::fabs(y)) * clamp01(1 - 5 * (p.normal[1] - 0.8f)) * 0.2f);
    }
    const float pm = clamp01(1.8f * (std::sqrt(H) - 0.333f) + 0.2f * strata);  // bit 9
    const float pr = clamp01(std::sqrt(H) + 0.2f * strata);                    // bit 10

    // Burnt ground (bit 7): Incinerated map 13.
    const float burnI = mapAt(13, x, z);
    const float burn = std::min(contrast8(2 * burnI - 1) * 0.5f + 0.5f, 0.9f);

    // Mixer 1 (0x6b0750): M1 earth, M2 road dirt.
    C c4[4];
    for (int j = 0; j < 4; ++j) c4[j] = mix({16 + j, 20 + j, 24 + j, 28 + j, 32 + j}, {70 + j, 74 + j, 78 + j, 82 + j, 86 + j});
    C M1 = lerp(c4[1], c4[0], pm), M2 = lerp(c4[3], c4[2], pm);
    M1 = lerp(M1, P(58), chalk * mapAt(18, x, z));
    // ctx+0x220 (wet ground) needs the water level map: 0 without water (port).
    if (burn > 0.01f) {
        const C b{0.4f, 0.35f, 0.35f};
        M1 = lerp(M1, b, burn), M2 = lerp(M2, b, burn);
    }
    // Mixers 2 and 3 (0x6b22e0, 0x6b3130): rock colours M3, M4 (0x6b22e0's chalk Y reads slot 54 twice).
    const C M3 = lerp(mix({51, 52, 53, 54, 55}, {101, 103, 105, 54, 109}), mix({46, 47, 48, 49, 50}, {100, 102, 104, 106, 108}), pr);
    const C M4 = lerp(mix({41, 42, 43, 44, 45}, {91, 93, 95, 97, 99}), mix({36, 37, 38, 39, 40}, {90, 92, 94, 96, 98}), pr);

    // Bit 7 (0x6b9f40): grass colour X.
    const float occ = clamp01(mapAt(9, x, z));  // ctx+0x230 (tree/bush shade near the camera not ported)
    const float s = clamp01(1.25f * H);
    C X;
    if (p.field == 2) {
        const bool first = p.fieldRandom < 0.66f;
        X = lerp(P(first ? 4 : 63), P(first ? 3 : 62), clamp01((H - 0.25f) * 1.8f));
    } else {
        X = lerp(L(8, 67), L(7, 66), s);
    }
    if (p.underWater()) {  // bit 7 under water: palette slot 7 as is
        X = P(7);
    }
    const float q = (1 - 0.5f * (1 - p.cover) * p.roadEdge) * occ;
    if (q < 0.99f && !p.underWater()) X = lerp(lerp(L(121, 122), L(119, 120), s), X, q);
    if (burn > 0.01f && !p.underWater()) {
        const float lum = X[0] + X[1] + X[2];  // weights g 0xa066f0 = (1, 1, 1)
        C k;
        for (int i = 0; i < 3; ++i) k[size_t(i)] = (lum + 0.75f * (0.2f - lum)) * (0.75f + 0.25f * occ);
        X = lerp(X, k, burn * (2 - occ));
    }

    // Bit 8 tail: grass colour weight.
    const float dry = clamp01(1 - 2 * clamp01((H - 0.33f) * 1.5625f + burn));
    const float g = clamp01(p.grass - 0.8f * dry);

    // Stones ctx+0x1dc (bit 24), computed by at().
    const float stones = p.stones + p.caillou;

    // Bit 14 (0x6bc230): the composer.
    C A = lerp(M1, X, g);
    const float cr = clamp01(stones + p.rockSlope - p.grass);
    if (cr > 0.025f) A = lerp(A, lerp(M3, M4, p.rockSlope), cr);
    C G = A;
    if (p.road > 0.025f) {
        G = lerp(M2, P(14), p.tarmac);
        A = lerp(A, G, p.road);
    }
    if (p.highGround > 0.025f) A = lerp(A, P(56), p.highGround);
    float f = clamp01(2 * (mapAt(7, x, z) * p.humidity - 0.5f));
    f *= f;
    if (p.height > 3000 - 500 * p.normal[2] - 200) f = 0;
    const float k = std::max(f - p.rock, 0.0f);
    const C far = lerp(lerp(A, P(57), k), P(56), 0.1f * p.highGround);
    for (int i = 0; i < 3; ++i) {
        out.base[i] = A[size_t(i)] * p.waterFade;  // ctx+0x70 = A * ctx+0x1b8
        out.far[i] = far[size_t(i)];
        out.road[i] = G[size_t(i)];
    }
    out.occlusion = occ;
    // Water colour ctx+0x60 (0x6b9560): palette slot 0, under water mixed with half the earth colour.
    for (int i = 0; i < 3; ++i) out.water[i] = P(0)[size_t(i)];
    if (p.underWater()) {
        const float wd = p.waterDepth - p.waterBase, kk = clamp01(p.sand + p.rockSlope);
        const float f = clamp01(wd * 0.1f + (kk + 1) * 0.5f);
        for (int i = 0; i < 3; ++i) out.water[i] = P(0)[size_t(i)] * f + M1[size_t(i)] * (1 - f) * 0.5f;
    }
    out.forest = k;
    return out;
}

// Point_SurfaceTypes (bit 21, 0x6bb670; notes p22 S1): weights of the 40 ground textures, the
// four strongest kept.
void GroundGenerator::surfaceTypes(double x, double z, GroundPoint& p) const {
    float w[kSurfaceTypes] = {};
    const float r = p.road, tk = p.tarmac;
    const float asph = tk * r, dirt = (1 - tk) * r, s3 = p.roughness * 3;
    w[11] = asph;
    w[13] = clamp01(1 - s3) * dirt;
    w[25] = clamp01(1 - std::fabs(1 - s3)) * dirt;
    w[2] = clamp01(1 - std::fabs(2 - s3)) * dirt;
    w[14] = clamp01(1 - std::fabs(3 - s3)) * dirt;
    const float cr = clamp01(mapAt(18, x, z) - r);  // ctx+0x184 = map 18
    w[24] = cr;
    const float m = 1 - cr;
    w[22] = p.grass * p.surfaceHumidity * m;
    w[15] = clamp01(p.rockSlope - p.caillou - p.stones - p.grass) * m;
    w[5] = m * p.caillou;
    w[3] = m * p.stones;
    w[8] = p.highGround * m;
    w[6] = m * p.sand;
    const float M = std::max({dirt, w[22], cr, w[5], w[8], w[3], asph, w[15], w[6]});
    const float steep = clamp01(1 - clamp01((p.normal[1] - 0.866f) * 7.4626865f) - p.sand), flat = 1 - steep;
    const float sz = p.grassSize;
    w[28] = steep * (1 - M);
    float bare = (1 - M) * flat;
    w[29] = (w[3] + w[5]) * steep;
    w[5] *= flat;
    w[32] = bare * sz;
    bare *= 1 - sz;
    w[30] = p.surfaceHumidity * bare;
    w[38] = w[6] * sz;
    w[6] *= 1 - sz;
    w[3] = w[3] * flat * (1 - sz);
    w[26] = w[3] * sz;
    w[34] = w[15] * sz;
    w[15] *= 1 - sz;
    if (p.field == 1) {
        w[10] = std::min((1 - p.surfaceHumidity) * bare + w[6], 1.0f);
        w[6] = 0;
    }
    const float v = clamp01(contrast8(2 * f3f54_.at(x, z) - 1) * 0.5f + 0.5f);  // 0x6a81f0
    for (const auto& [a, b] : {std::pair{38, 39}, {34, 35}, {32, 33}, {30, 31}, {11, 12}, {15, 16}, {22, 23}, {3, 4}, {8, 9}, {26, 27}})
        if (w[a] > 0.025f) w[b] = w[a] * (1 - v), w[a] *= v;
    if (w[6] > 0.025f) w[7] = w[6] * (1 - v), w[6] *= v;  // dry land (under water: WETSAND 36/37)
    // Keep weights > 0.025, strongest first, normalised by their total.
    int idx[kSurfaceTypes];
    int n = 0;
    float total = 0;
    for (int k = 0; k < kSurfaceTypes; ++k)
        if (w[k] > 0.025f) idx[n++] = k, total += w[k];
    for (int i = 0; i < n; ++i)  // the engine's exchange sort (not stable for equal weights)
        for (int j = i + 1; j < n; ++j)
            if (w[idx[i]] < w[idx[j]]) std::swap(idx[i], idx[j]);
    for (int k = 0; k < 4; ++k) {
        p.surfaceType[k] = k < n ? uint8_t(idx[k]) : uint8_t(0xff);
        p.surfaceWeight[k] = k < n ? w[idx[k]] / total : 0.0f;
    }
}

// Ground_ClassFractionsCached 0x6b76a0 (Observed decompile; p32 notes): the point is evaluated at the
// centre of its cell (floor(x + 0.5): fistp rounds down; cell scale G+0x43ec8 / G+0x43ee8 taken as 1 m),
// then 18 class weights from the ground layers, each S2-smoothed, normalised by their sum.
void GroundGenerator::classFractions(double x, double y, double z, float frac[kGroundClasses], float& depth) const {
    const int32_t ix = int32_t(std::floor(x + 0.5)), iz = int32_t(std::floor(z + 0.5));
    const uint64_t key = (uint64_t(uint32_t(ix)) << 32) | uint32_t(iz);
    ClassCell e{};
    bool have = false;
    {
        std::lock_guard<std::mutex> lk(classCache_->m);
        auto it = classCache_->cells.find(key);
        if (it != classCache_->cells.end()) e = it->second, have = true;
    }
    if (!have) {
        const GroundPoint p = at(double(ix), double(iz));
        for (float& f : e.frac) f = 0;
        e.height = p.height;
        e.soft = p.puddle;  // ctx+0x208
        const float r = p.road, tk = p.tarmac, ro = p.roughness, sz = p.grassSize;
        const float cr = clamp01(mapAt(18, double(ix), double(iz)) - r);  // ctx+0x184 (map 18) - road
        if (p.waterDepth <= p.waterBase + 0.01f) {  // ctx+0x1ac <= ctx+0x1b0 + 0.01: dry
            e.water = -10000.0f;
            // Grass weight ctx+0x1d0 = clamp(Grass - 0.8 dry) (bit 8; burn ctx+0x22c is 0 under water).
            const float burn = std::min(contrast8(2 * mapAt(13, double(ix), double(iz)) - 1) * 0.5f + 0.5f, 0.9f);
            const float dry = clamp01(1 - 2 * clamp01((p.surfaceHumidity - 0.33f) * 1.5625f + burn));
            const float g = clamp01(p.grass - 0.8f * dry);
            const float A = (1 - cr) * (1 - r);
            float* c = e.frac;
            c[0] = tk * (1 - cr) * r;                      // tarmac
            c[1] = clamp01((1 - ro) * (1 - tk) * r + cr);  // smooth dirt road (+ map-18 cover)
            c[2] = (1 - tk) * (1 - cr) * ro * r;           // rough dirt road
            const float gr = g * (1 - c[0]) * A;
            c[11] = gr * (1 - sz), c[12] = gr * sz;               // grass (ctx+0x1d0)
            c[5] = p.rock * A * (1 - sz), c[6] = p.rock * A * sz;  // rock ctx+0x1e0
            c[7] = p.caillou * A, c[8] = p.stones * A;             // ctx+0x1d8 / 0x1dc
            c[9] = p.sand * A * (1 - sz), c[10] = p.sand * A * sz;  // ctx+0x220
            c[17] = p.highGround * A;                              // ctx+0x19c
            const float rest = (1 - p.grass - p.rock - p.stones - p.caillou - p.sand - p.highGround) * A;
            c[3] = rest * (1 - sz), c[4] = rest * sz;
        } else {
            e.water = p.waterDepth + p.height - p.waterBase;  // ctx+0x1ac + y - ctx+0x1b0
            e.frac[0] = 1;
        }
        float sum = 0;
        for (float& f : e.frac) {
            const float v = 2 * clamp01(f) - 1;
            f = (2 - std::fabs(v)) * v * 0.5f + 0.5f;
            sum += f;
        }
        if (sum > 0)
            for (float& f : e.frac) f *= 1 / sum;
        std::lock_guard<std::mutex> lk(classCache_->m);
        if (classCache_->cells.size() > 65536) classCache_->cells.clear();
        classCache_->cells.emplace(key, e);
    }
    if (std::isnan(y)) y = double(e.height);  // caller without a contact height: at the ground
    if (y <= double(e.height) + 1.0) {
        for (int k = 0; k < kGroundClasses; ++k) frac[k] = e.frac[k];
    } else {  // more than 1 m above the ground: class 0 only
        for (int k = 0; k < kGroundClasses; ++k) frac[k] = k == 0 ? 1.0f : 0.0f;
    }
    // depth = max(0, water - y) + max(0, soft - 0.05) - 0.01, >= 0
    const float d = std::max(0.0f, e.water - float(y)) + std::max(0.0f, e.soft - 0.05f) - 0.01f;
    depth = d > 0 ? d : 0;
}

bool GroundGenerator::rejectsPlant(double x, double z) const {
    if (excludesPlants(x, z)) return true;
    const GroundPoint p = at(x, z, false);
    return p.field != 0 || p.rock > 0.1f;
}

void GroundGenerator::grass(int32_t x0, int32_t z0, int32_t x1, int32_t z1, std::vector<GrassTuft>& out) const {
    if (terrain_ == nullptr || tileWeight_.size() < 9) return;
    auto byte64 = [](float v) { return float(int64_t(std::floor(v * 64.0f + 0.5f)) & 255) * 0.015625f; };  // floor: _RC_DOWN
    for (int32_t iz = z0; iz < z1; ++iz)
        for (int32_t ix = x0; ix < x1; ++ix) {
            // Grass_Decide (0x7dddf0): jitter by the permutation nibbles.
            const uint8_t jb = hashByte(perm_, double(iz) * 10.193578 + double(ix) * 12.317513 + 29.823713);
            const double x = ix + float(jb & 15) * 0.06666667f, z = iz + float(jb >> 4) * 0.06666667f;
            const GroundPoint p = at(x, z);
            const float g = byte64(p.groundGrass);
            if (g <= 0.3f) continue;
            float h = g * g + 0.15f;
            if (p.field > 1) h += 0.3f;
            const float scale = (byte64(p.grassSize) + 0.5f) * h;
            const float kind = byte64(p.surfaceHumidity);
            // Grass_EmitTuft (0x7dce50).
            const float r0 = float(hashByte(perm_, z * 10.193578 + x * 12.317513)) / 255.0f, r0s = r0 * r0;
            const float size = std::max(1.0f, (r0s * 0.5f + 0.75f) * scale);
            auto base = uint32_t(std::floor(kind * 5.9f));  // fistp rounds down
            base = std::min(base, 6u);
            const float w0 = tileWeight_[base], w1 = tileWeight_[base + 1], w2 = tileWeight_[base + 2];
            const float inv = 1.0f / (w0 + w1 + w2);
            const float r1 = float(hashByte(perm_, z * 10.193578 + (double(r0s) * 931.9283 + x) * 12.317513)) / 255.0f;
            uint32_t tile = base;
            if (w0 * inv <= r1) tile = (w0 * inv + w1 * inv <= r1) ? base + 2 : base + 1;
            const float tileSize = tileSize_[tile];
            if (tileSize <= 0) continue;  // tile 8 has size 0 in the engine's table
            const float u0 = float(tile & 3) * 0.25f;
            const uint8_t r2 = hashByte(perm_, z * 10.193578 + (double(u0) * 931.9283 + x) * 12.317513);
            GrassTuft tuft{};
            tuft.x = float(x), tuft.z = float(z);
            tuft.y = p.height - (1 - p.normal[1]) * 0.5f;
            tuft.size = tileSize * size;
            tuft.tile = uint8_t(tile);
            tuft.windAmp = uint8_t(std::lround((float(r2) / 255.0f + 1) * 0.5f * 2.25f * 100.0f));
            const float road = byte64(p.road);
            tuft.tint = uint8_t(std::lround(road * 255.0f));
            // Grass_Decide (0x7ddf96..): colour lerp(R+0x0c, R+0x1c, road) = lerp(ctx+0x80, ctx+0x50, road);
            // alpha clamp(0.8 a + 0.2) (0x7dcdb0), which the grass shader uses as sky occlusion.
            const GroundColours col = colours(x, z, p);
            for (int i = 0; i < 3; ++i) {
                const float v = col.base[i] + (col.road[i] - col.base[i]) * road;
                tuft.rgb[i] = uint8_t(std::lround(clamp01(v) * 255.0f));
            }
            tuft.occlusion = uint8_t(std::lround(clamp01(0.8f * byte64(col.occlusion) + 0.2f) * 255.0f));
            out.push_back(tuft);
        }
}

}  // namespace fuel
