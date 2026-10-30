#include "ground_point.hpp"
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
uint8_t hashByte(const Permutation& p, double v) { return p[size_t(int64_t(std::nearbyint(float(v))) & 255)]; }

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
    // Maps used here: 2 humidity, 8 population, 10 rock (Terrain_Map10).
    for (int slot : {2, 8, 10}) {
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
float GroundGenerator::roadWeight(double x, double z, bool* nearRoad) const {
    *nearRoad = false;
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
        best = std::max(best, clamp01(float((0.5 * width - d + 2.75) * 0.18181819)));
    }
    return best;
}

// Point_RockCover (0x6b8d80) over Terrain_RockEval (0x7a4c20).
float GroundGenerator::rockCover(double x, double z, const GroundPoint& p, float coarseNy) const {
    if (p.road > 0.95f) return 0;
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
    if (p.height > 3000 - 500 * p.normal[2]) rock = 1;
    rock = clamp01(rock - p.cover - p.road);  // - junction term (ctx+0x238, not ported)
    return clamp01(rock - p.highGround);       // - water (ctx+0x228, not ported)
}

// Point_Fields (0x6b9100).
int GroundGenerator::fieldType(double x, double z, const GroundPoint& p, float coarseNy, bool nearRoad, float cover) const {
    if (!(p.normal[1] > 0.8707f) || !(p.height < 1500)) return 0;
    const float pop = mapAt(8, x, z);
    if (!(pop > 0.25f && pop < 0.75f)) return 0;
    const float shift = clamp01(f7ea4_.at(x, z)) * 150.0f;
    const double a = (x + shift) * 0.005, c = (z + shift) * 0.005;
    const double i = std::nearbyint(a), j = std::nearbyint(c);
    const float u = float(a - i), v = float(c - j);
    const double cx = i * 200 + 100, cz = j * 200 + 100;
    const float r = float(hashByte(perm_, cx * 12.317513 + cz * 10.193578)) / 255.0f;
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
    const SurfaceWeights fs = surfaceWeights(x, z);
    p.cover = fs.cover;
    // Bit 12: road weight plus the flat surfaces that clear grass.
    p.road = clamp01(roadWeight(x, z, &nearRoad) + clamp01(1 - (fs.add + fs.noise)) * fs.cut);
    // Bit 4: high ground (p13 V2 0x4de850) minus road.
    {
        float a = clamp01(p.height - (2500.0f + 250.0f * p.normal[2]));
        a = clamp01(a - 2 * (1 - p.normal[1]));
        const float hg = contrast8(2 * a - 1) * 0.5f + 0.5f;
        p.highGround = clamp01(hg - (2 - p.road) * p.road);
    }
    p.rock = rockCover(x, z, p, coarse[1]);
    p.field = fieldType(x, z, p, coarse[1], nearRoad, fs.cover);
    if (!full) return p;

    // Bit 8 (Point_Grass 0x6ba610).
    const float slope = std::acos(clamp01(p.normal[1]));
    float g = 0, ground = 0;
    const float n10 = s4c(f1fac_.at(x, z));
    if (!(p.rock > 0.5f) && p.field != 1 && !(p.height > 3000 - 500 * p.normal[2])) {
        g = 1 - fs.cut + fs.noise * n10;
        g -= (2 - p.road) * p.road;
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
    return p;
}

bool GroundGenerator::rejectsPlant(double x, double z) const {
    if (excludesPlants(x, z)) return true;
    const GroundPoint p = at(x, z, false);
    return p.field != 0 || p.rock > 0.1f;
}

void GroundGenerator::grass(int32_t x0, int32_t z0, int32_t x1, int32_t z1, std::vector<GrassTuft>& out) const {
    if (terrain_ == nullptr || tileWeight_.size() < 9) return;
    auto byte64 = [](float v) { return float(std::lround(v * 64.0f) & 255) * 0.015625f; };
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
            auto base = uint32_t(std::lround(kind * 5.9f));
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
            tuft.tint = uint8_t(std::lround(byte64(p.road) * 255.0f));
            out.push_back(tuft);
        }
}

}  // namespace fuel
