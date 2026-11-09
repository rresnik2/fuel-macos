#include "vegetation.hpp"
#include "world.hpp"
#include "road_segments.hpp"

#include "assembly.hpp"
#include "resources.hpp"

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <stdexcept>
#include <string>

namespace fuel {

namespace {

constexpr uint32_t kGenWorldClass = 0x31FA94AE;
constexpr uint32_t kUserDefineClass = 0x52F79F96;
constexpr int kCell = 20;          // one tree and one bush candidate per 20 m cell
constexpr float kJitter = 1.2f;    // permutation nibble x 1.2 m
constexpr float kTreeSalt = 33.13f, kBushSalt = 37.82f;
constexpr float kTreeline = 3000.0f;
constexpr float kMaxSlopeNy = 0.866f;   // 30 degrees
constexpr float kFadeNy = 0.707f;       // density fades out within ~0.6 degrees past 45
constexpr float kRoadQuery = 10.0f;     // the engine looks for roads within 10 m
constexpr float kSizeScale = 6.0f;      // species size range x 6

// A float32 store under FUEL's round-down x87 mode (notes p21 H0).
double f32down(double v) {
    float f = float(v);
    if (double(f) > v) f = std::nextafter(f, -std::numeric_limits<float>::infinity());
    return double(f);
}

uint32_t rd32(const std::vector<uint8_t>& b, size_t o) {
    if (o + 4 > b.size()) throw std::runtime_error("GenWorld_Z body too short");
    uint32_t v;
    std::memcpy(&v, &b[o], 4);
    return v;
}

// FUEL's contrast curve on [-1, 1].
float sharpen(float a) { return a <= 0 ? std::pow(a + 1, 8.0f) - 1 : 1 - std::pow(1 - a, 8.0f); }

// Cubic B-spline sample of a GenWorld map, on the same lattice as the heightfield
// (sample i at -width/2 * spacing + i * spacing over the 140 km square).
float sampleMap(const Raster& r, double x, double z) {
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

float clamp01(float v) { return std::clamp(v, 0.0f, 1.0f); }

// "GenWorld Arbre": { "<date key>" a [b] } items, values in percent.
void parseSpeciesScript(const std::vector<uint8_t>& body, Species& sp) {
    if (body.size() < 4) return;
    const uint32_t len = std::min<uint32_t>(rd32(body, 0), uint32_t(body.size() - 4));
    const std::string text(reinterpret_cast<const char*>(&body[4]), len);
    if (text.find("GenWorld Arbre") == std::string::npos) return;
    for (size_t q = text.find('"'); q != std::string::npos; q = text.find('"', q + 1)) {
        const size_t end = text.find('"', q + 1);
        if (end == std::string::npos) break;
        const std::string key = text.substr(q + 1, end - q - 1);
        const size_t close = text.find('}', end);
        if (close == std::string::npos) break;
        const char* p = text.c_str() + end + 1;
        const char* stop = text.c_str() + close;
        float v[2] = {0, 0};
        int n = 0;
        while (p < stop && n < 2) {
            char* next = nullptr;
            const float f = std::strtof(p, &next);
            if (next == p) {
                ++p;
                continue;
            }
            v[n++] = f;
            p = next;
        }
        q = end;
        if (n == 0) continue;
        // 0x7832c0: the number reader 0x6862e0 returns the digits as an x87 value; fmul 0.01f (and fsub / fadd
        // 0.001f for ranges) in 53 bits, fstp to the float field under round-down (USA1's values are all integers).
        const double pct = double(0.01f);
        auto range = [&](Species::Range& r) {
            if (n < 2) return;
            r.lo = float(f32down(double(v[0]) * pct - double(0.001f)));
            r.hi = float(f32down(double(v[1]) * pct + double(0.001f)));
        };
        auto percent = [&] { return float(f32down(double(v[0]) * pct)); };
        if (key == "12/11/2007 19:25") range(sp.size);  // record +0x44 / +0x48: the plant size (draw and trunks)
        else if (key == "12/11/2007 19:29") sp.groundAlign = percent();
        else if (key == "12/11/2007 19:30") sp.tilt = float(f32down(double(v[0]) * double(0.017453292f)));
        else if (key == "12/11/2007 20:01") sp.weight = percent();
        else if (key == "25/01/2009 13:52") sp.trunk = percent();  // 0x7832c0 at 0x7839ed, store 0x783a13 -> record +0x78
        else if (key == "12/11/2007 20:00") sp.flags = uint32_t(std::trunc(v[0]));  // fistp with RC = chop (0xc00) -> +0xf4
        else if (key == "12/11/2007 19:32") range(sp.humidity);
        else if (key == "12/11/2007 19:34") range(sp.density);
        else if (key == "17/04/2008 19:32") range(sp.stormed);
        else if (key == "17/04/2008 19:33") range(sp.incinerated);
        else if (key == "17/04/2008 19:34") range(sp.sanded);
        else if (key == "17/04/2008 19:35") range(sp.flooded);
        else if (key == "17/04/2008 19:36") range(sp.highGround);
    }
}

// Tree exclusion distance around each road type (road table +0xd0).
float roadClearance(const RoadNetwork& net, int type) {
    return type >= 0 && type < int(net.clearance.size()) ? net.clearance[size_t(type)] : 0;
}

// True when a road passes closer than its clearance (the engine only finds roads within 10 m).
bool nearRoad(const RoadNetwork& net, double x, double z) {
    const auto it = net.segCells.find(RoadNetwork::cellKey(int32_t(std::floor(x / RoadNetwork::kSegCell)), int32_t(std::floor(z / RoadNetwork::kSegCell))));
    if (it == net.segCells.end()) return false;
    double best = kRoadQuery;
    int type = -1;
    for (uint64_t ref : it->second) {
        const RoadNetwork::Line& line = net.lines[size_t(ref >> 20)];
        const size_t k = size_t(ref & 0xFFFFF);
        const double ax = line.points[k][0], az = line.points[k][1];
        const double bx = line.points[k + 1][0] - ax, bz = line.points[k + 1][1] - az;
        const double len2 = bx * bx + bz * bz;
        const double t = len2 > 0 ? std::clamp(((x - ax) * bx + (z - az) * bz) / len2, 0.0, 1.0) : 0.0;
        const double d = std::hypot(x - (ax + bx * t), z - (az + bz * t));
        if (d < best) best = d, type = line.type;
    }
    return type >= 0 && best < roadClearance(net, type);
}

struct Ground {
    float y = 0;
    float n[3] = {0, 1, 0};
};

Ground groundAt(const Heightfield& t, double x, double z) {
    Ground g;
    g.y = t.smoothHeightAt(x, z);
    constexpr double e = 2.0;
    const double dx = (t.smoothHeightAt(x + e, z) - t.smoothHeightAt(x - e, z)) / (2 * e);
    const double dz = (t.smoothHeightAt(x, z + e) - t.smoothHeightAt(x, z - e)) / (2 * e);
    const double len = std::sqrt(dx * dx + 1 + dz * dz);
    g.n[0] = float(-dx / len), g.n[1] = float(1 / len), g.n[2] = float(-dz / len);
    return g;
}

// Forest density 0..1: the sharpened forest map minus the sharpened clearing noise,
// faded out on slopes past 45 degrees.
float densityAt(const Vegetation& v, double x, double z, float ny) {
    const float f = sampleMap(v.forest, x, z);
    const float g = clamp01(v.clearings.at(x, z));
    float d = clamp01((sharpen(2 * f - 1) * 0.5f + 0.5f) - 0.95f * (sharpen(2 * g * g - 1) * 0.5f + 0.5f));
    if (ny < kFadeNy) d *= std::max(0.0f, 1 - (kFadeNy - ny) * 100);
    return clamp01(d);
}

// 0 on low ground, 1 above about 2500 m (less on slopes).
float highGround(float y, const float n[3]) {
    float a = clamp01(y - (2500.0f + 250.0f * n[2]));
    a = clamp01(a - 2 * (1 - n[1]));
    return sharpen(2 * a - 1) * 0.5f + 0.5f;
}

// Weighted pick among the species whose climate ranges hold; r in 0..1. Index or -1.
int pickSpecies(const std::vector<Species>& list, const Vegetation& v, double x, double z, const Ground& g, float density, float r,
                bool needImpostor) {
    const float hum = clamp01(sampleMap(v.humidity, x, z));
    const float storm = clamp01(sampleMap(v.stormed, x, z)), incin = clamp01(sampleMap(v.incinerated, x, z));
    const float sand = clamp01(sampleMap(v.sanded, x, z)), flood = clamp01(sampleMap(v.flooded, x, z));
    const float high = highGround(g.y, g.n);
    int ok[128];
    int n = 0;
    float total = 0;
    for (size_t i = 0; i < list.size() && n < 128; ++i) {
        const Species& s = list[i];
        if (!s.humidity.has(hum) || !s.density.has(density) || !s.stormed.has(storm) || !s.incinerated.has(incin) || !s.sanded.has(sand) ||
            !s.flooded.has(flood) || !s.highGround.has(high))
            continue;
        ok[n++] = int(i);
        total += s.weight;
    }
    if (n == 0) return -1;
    if (total > 1) r *= total;
    float acc = 0;
    for (int k = 0; k < n; ++k) {
        acc += list[size_t(ok[k])].weight;
        if (r <= acc) {
            const Species& s = list[size_t(ok[k])];
            if (needImpostor && s.impostor == 0) return -1;
            return ok[k];
        }
    }
    return -1;
}

using Mat3 = std::array<std::array<float, 3>, 3>;  // m[row][col]

Mat3 mul(const Mat3& a, const Mat3& b) {
    Mat3 r{};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) r[i][j] = a[i][0] * b[0][j] + a[i][1] * b[1][j] + a[i][2] * b[2][j];
    return r;
}

Mat3 axisAngle(const float axis[3], float angle) {
    const float c = std::cos(angle), s = std::sin(angle), t = 1 - c;
    const float x = axis[0], y = axis[1], z = axis[2];
    return {{{t * x * x + c, t * x * y - s * z, t * x * z + s * y}, {t * x * y + s * z, t * y * y + c, t * y * z - s * x}, {t * x * z - s * y, t * y * z + s * x, t * z * z + c}}};
}

// Rotation taking +y to `up` (unit).
Mat3 alignUp(const float up[3]) {
    const float axis[3] = {up[2], 0, -up[0]};  // (0,1,0) x up
    const float s = std::sqrt(axis[0] * axis[0] + axis[2] * axis[2]);
    if (s < 1e-6f) return {{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
    const float a[3] = {axis[0] / s, 0, axis[2] / s};
    return axisAngle(a, std::atan2(s, up[1]));
}

// The engine's instance randomness: four chained permutation draws.
void instance(const Vegetation& v, const Species& s, double x, double z, const Ground& g, Plant& p) {
    const float r0 = float(plantNoise(v, float(x), float(z)));  // 0x784370 / 0x787c70
    const float r1 = float(positionHash(v.perm, double(r0) * double(931.9283f) + x, z)) / 255.0f;
    const float r2 = float(positionHash(v.perm, double(r1) * double(931.9283f) + x, z)) / 255.0f;
    const float r3 = float(positionHash(v.perm, double(r2) * double(931.9283f) + x, z)) / 255.0f;
    const float size = plantScale(v, s, float(x), float(z));
    float up[3] = {g.n[0] * s.groundAlign, 1 - s.groundAlign + g.n[1] * s.groundAlign, g.n[2] * s.groundAlign};
    const float len = std::sqrt(up[0] * up[0] + up[1] * up[1] + up[2] * up[2]);
    for (float& c : up) c /= len;
    static constexpr float kX[3] = {1, 0, 0}, kY[3] = {0, 1, 0}, kZ[3] = {0, 0, 1};
    Mat3 m = mul(alignUp(up), mul(axisAngle(kX, (2 * r1 - 1) * s.tilt), mul(axisAngle(kZ, (2 * r2 - 1) * s.tilt), axisAngle(kY, r3 * 6.28f))));
    // Columns of m are the images of the mesh axes.
    for (int axis = 0; axis < 3; ++axis)
        for (int c = 0; c < 3; ++c) p.axes[size_t(axis)][size_t(c)] = m[size_t(c)][size_t(axis)] * size;
    p.position = {float(x), g.y - kPlantSink, float(z)};
}

}  // namespace

double plantNoise(const Vegetation& v, float x, float z) {
#pragma clang fp contract(off)
    // fld x; fadd 425.23886f; fmul 12.317513f; fld z; fmul 10.193578f; faddp; fstp float; fistp (floor).
    const double h = f32down((double(x) + double(425.23886f)) * double(12.317513f) + double(z) * double(10.193578f));
    return double(v.perm[size_t(int64_t(std::floor(h)) & 255)]) * double(0.003921569f);
}

float plantScale(const Vegetation& v, const Species& s, float x, float z) {
#pragma clang fp contract(off)
    const double n = f32down(plantNoise(v, x, z));  // 0x784400 keeps n as a float (fst [esp+0x2c])
    return float(f32down(((1 - n) * double(s.size.hi) + double(s.size.lo) * n) * double(kSizeScale)));
}

float trunkRadius(const Vegetation& v, const Species& s, float x, float z) {
#pragma clang fp contract(off)
    // 0x78531d: fld [rec+0x78]; fstp [ebx]; call 0x784370; fld1; fsub st, st(1); fmul [rec+0x48]; fld [rec+0x44];
    // fmulp st(2); faddp; fmul [ebx]; fmul 6.0; fst [ebx]; 2.0 <= r ? 2.0 : r (0x788bd3 for bushes: the same).
    const double n = plantNoise(v, x, z);
    const double r = ((1 - n) * double(s.size.hi) + double(s.size.lo) * n) * double(s.trunk) * 6.0;
    return r >= 2.0 ? 2.0f : float(f32down(r));
}

Vegetation loadVegetation(const DpcFile& world) {
    const std::vector<uint32_t> gens = world.ids(kGenWorldClass);
    if (gens.empty()) throw std::runtime_error("no GenWorld_Z");
    const auto body = world.body(*world.find(gens.front()));
    if (!body) throw std::runtime_error("GenWorld_Z has no body");
    const std::vector<uint8_t>& b = *body;
    // u32, u32, u32 roads; u32 n, maps[n]; u32 n, ground textures[n]; u32 n, materials[n]; u32 seed;
    // u32 groups, each {u32 len, name[len], u32, u32 n, Node_Z[n]}.
    size_t p = 12;
    std::vector<uint32_t> maps;
    const uint32_t nMaps = rd32(b, p);
    for (uint32_t i = 0; i < nMaps; ++i) maps.push_back(rd32(b, p + 4 + size_t(i) * 4));
    p += 4 + size_t(nMaps) * 4;
    p += 4 + size_t(rd32(b, p)) * 4;
    p += 4 + size_t(rd32(b, p)) * 4;
    Vegetation v;
    v.perm = buildPermutation(world, rd32(b, p));
    p += 4;
    // Clearing noise: 3 levels under a 500 m cell (finest 62.5 m).
    v.clearings = Fractal(v.perm, 3, 500.0f, 43.154682f, 0.025f, 0.15f, 0.5f);
    const uint32_t groups = rd32(b, p);
    p += 4;
    for (uint32_t g = 0; g < groups; ++g) {
        const uint32_t len = rd32(b, p);
        if (p + 4 + len > b.size()) throw std::runtime_error("GenWorld_Z group name overruns");
        const std::string name(reinterpret_cast<const char*>(&b[p + 4]), strnlen(reinterpret_cast<const char*>(&b[p + 4]), len));
        p += 4 + len + 4;
        const uint32_t n = rd32(b, p);
        p += 4;
        std::vector<Species>* list = name == "TREES" ? &v.trees : name == "BUSHES" ? &v.bushes : nullptr;
        for (uint32_t i = 0; i < n; ++i) {
            const uint32_t nodeId = rd32(b, p + size_t(i) * 4);
            if (list == nullptr) continue;
            try {  // Node_Z -> Lod_Z -> [full mesh, impostor], script
                const DpcFile::Entry* ne = world.find(nodeId);
                const auto nb = ne ? world.body(*ne) : std::nullopt;
                if (!nb) continue;
                const DpcFile::Entry* le = world.find(parseNodeBody(*nb).object);
                const auto lb = le ? world.body(*le) : std::nullopt;
                if (!lb) continue;
                const Lod lod = parseLodBody(*lb);
                if (lod.objects.empty() || world.find(lod.objects[0]) == nullptr) continue;
                Species sp;
                sp.full = lod.objects[0];
                if (lod.objects.size() > 1 && world.find(lod.objects[1]) != nullptr) sp.impostor = lod.objects[1];
                sp.highGround = {-0.001f, 0.501f};  // engine defaults for ranges a script leaves out
                for (Species::Range* r : {&sp.humidity, &sp.density, &sp.stormed, &sp.incinerated, &sp.sanded, &sp.flooded}) *r = {0.499f, 1.001f};
                sp.weight = 0.1f;
                if (const DpcFile::Entry* ue = world.find(lod.userDefine); ue != nullptr && ue->classCrc == kUserDefineClass)
                    if (const auto ub = world.body(*ue)) parseSpeciesScript(*ub, sp);
                list->push_back(sp);
            } catch (const std::exception&) {
            }
        }
        p += size_t(n) * 4;
    }
    // The 0x40-byte GenWorld records follow the groups: u32 n, n x 0x83-byte flat surfaces, u32 n, n x 0x40 (0x77c690).
    try {
        size_t q = p;
        const uint32_t nFlat = rd32(b, q);
        q += 4 + size_t(nFlat) * 0x83;
        const uint32_t nBox = rd32(b, q);
        q += 4;
        if (nBox < 100000 && q + size_t(nBox) * 0x40 <= b.size()) {
            for (uint32_t i = 0; i < nBox; ++i, q += 0x40) {
                PlantClearance::Box box;
                float f[16];
                std::memcpy(f, &b[q], sizeof f);
                for (int k = 0; k < 12; ++k) box.m[size_t(k)] = f[k];
                for (int k = 0; k < 3; ++k) box.ext[size_t(k)] = f[12 + k];
                box.radius = f[15];
                v.clearance.boxes.push_back(box);
            }
            v.clearance.index();
        }
    } catch (const std::exception&) {
    }
    auto loadMap = [&](size_t slot) {
        Raster r;
        if (slot < maps.size() && maps[slot] != 0)
            if (const DpcFile::Entry* e = world.find(maps[slot]))
                if (const auto mb = world.body(*e)) r = decodeRaster(*mb);
        return r;
    };
    v.forest = loadMap(7);
    v.humidity = loadMap(2);
    v.incinerated = loadMap(13);
    v.stormed = loadMap(14);
    v.flooded = loadMap(15);
    v.sanded = loadMap(16);
    return v;
}

void scatterPlants(const Vegetation& v, const Heightfield& terrain, const RoadNetwork* roads, const BlockedQuery& blocked, float x, float z,
                   float radius, std::vector<Plant>& out, bool impostors, float inner, const GroundGenerator* ground) {
    if (v.trees.empty() && v.bushes.empty()) return;
    // Candidates sit at multiples of 20 m and only move +x/+z, so widen by a cell.
    const int32_t c0x = int32_t(std::floor((x - radius) / kCell)) - 1, c1x = int32_t(std::floor((x + radius) / kCell));
    const int32_t c0z = int32_t(std::floor((z - radius) / kCell)) - 1, c1z = int32_t(std::floor((z + radius) / kCell));
    for (int32_t cz = c0z; cz <= c1z; ++cz) {
        for (int32_t cx = c0x; cx <= c1x; ++cx) {
            for (int kind = 0; kind < 2; ++kind) {
                const bool tree = kind == 0;
                const std::vector<Species>& species = tree ? v.trees : v.bushes;
                if (species.empty() || (impostors && !tree)) continue;
                const float gx = float(cx * kCell), gz = float(cz * kCell);
                // Jitter (0x7840c0 trees, 0x787a40 bushes): x87 sums (53 bits) stored as float32 under round-down,
                // then fistp (floor): hash = x * 12.317513 + z * 10.193578 + salt; the plant point x + nibble x 1.2
                // (exact in 53 bits) is stored as a float too, and every later hash reads that float (P52).
                const double hsum = f32down(double(gx) * double(12.317513f) + double(gz) * double(10.193578f) + double(tree ? kTreeSalt : kBushSalt));
                const uint8_t jb = v.perm[size_t(int64_t(std::floor(hsum)) & 255)];
                const double px = f32down(double(gx) + double(jb & 15) * double(kJitter)), pz = f32down(double(gz) + double(jb >> 4) * double(kJitter));
                const double dist = std::hypot(px - x, pz - z);
                if (dist > radius || dist < inner) continue;
                const Ground g = groundAt(terrain, px, pz);
                const float density = densityAt(v, px, pz, g.n[1]);
                const float r = float(positionHash(v.perm, px, pz)) / 255.0f;
                if (tree && !(r < density)) continue;
                // Trees_Filter 0x783d60 / Bushes_Filter 0x7876e0 (P70b): RoadQuery 0x6fb2a0 (r = 10) on the TerraRoad
                // segments = the segment with the smallest edge distance and its closest centreline point; reject when
                // that centreline is nearer than T[+0xd0] (0xa678b0); nearer than T[+0x00]: "near road", which later
                // needs n.y >= 0.707 [0x9dc0c8] (0x64e130 normal). Without FUEL's segments: the port's polyline test.
                bool nearRoadSlope = false;
                if (terrain.roads) {
                    const RoadSampler::Nearest nr = terrain.roads->nearestRoad(px, pz, kRoadQuery);
                    if (nr.seg != nullptr) {
                        const float dx = float(px) - nr.px, dz = float(pz) - nr.pz;
                        const float d = std::sqrt(dz * dz + dx * dx);
                        const RoadType& T = terrain.roads->types().types[std::min<size_t>(nr.seg->type, 20)];
                        if (d < T.at(0xd0)) continue;
                        nearRoadSlope = d < T.at(0x00);
                    }
                } else if (roads != nullptr && nearRoad(*roads, px, pz)) {
                    continue;
                }
                if (g.y > kTreeline || g.n[1] < kMaxSlopeNy) continue;
                if (nearRoadSlope && g.n[1] < 0.707f) continue;
                // Trees_Filter / Bushes_Filter: Trees_Excluded (0x7831e0), farm fields (point bit 19)
                // and rock cover > 0.1 (bit 11), decoded in ground_point.cpp.
                if (ground != nullptr && ground->rejectsPlant(px, pz)) continue;
                if (blocked && blocked(px, pz)) continue;
                // Decor_FreeOfObjects 0x557650: GenWorld record boxes, mission gates, POIs (PK05).
                if (!v.clearance.free(float(px), g.y, float(pz), kPlantSphereRadius)) continue;
                const int pick = pickSpecies(species, v, px, pz, g, density, r, impostors);
                if (pick < 0) continue;
                const Species& sp = species[size_t(pick)];
                Plant plant;
                plant.meshId = impostors ? sp.impostor : sp.full;
                plant.tree = tree;
                plant.species = uint16_t(pick + 1);
                instance(v, sp, px, pz, g, plant);
                out.push_back(plant);
            }
        }
    }
}

// ---- P94: FUEL's decor kinds and fades (vegetation.hpp).
double decorLeafSize(double px, double pz, double camX, double camZ) {
    constexpr double kRoot = 12288.0, kSplit = 2.0625, kMinChunk = 24.01;
    double size = kRoot;
    double ox = std::round(px / kRoot) * kRoot, oz = std::round(pz / kRoot) * kRoot;
    while (size > kMinChunk) {
        const double r = size * kSplit;
        if ((ox - camX) * (ox - camX) + (oz - camZ) * (oz - camZ) >= r * r) break;
        ox += (px >= ox ? 0.25 : -0.25) * size;
        oz += (pz >= oz ? 0.25 : -0.25) * size;
        size *= 0.5;
    }
    return size;
}

float decorAtt(DecorFade kind, double px, double pz, double eyeX, double eyeZ, float dxz) {
    if (kind == DecorFade::None) return 1.0f;
    auto att = [dxz](float min, float k) { return 1.0f - std::clamp((dxz - min) * k, 0.0f, 1.0f); };
    const double C = decorLeafSize(px, pz, eyeX, eyeZ);
    if (kind == DecorFade::Mesh) return C < 60.0 ? att(132.5f, 1.0f / 30.0f) : 0.0f;  // kinds 0 / 1
    if (C < 30.0) return 0.0f;
    if (C < 60.0) return att(147.5f, -1.0f / 30.0f);  // kind 0x16
    if (kind == DecorFade::TreeCard) return C < 600.0 ? att(1200.0f, 1.0f / 100.0f) : 0.0f;  // kind 0x14
    return C < 100.0 ? att(250.0f, 1.0f / 50.0f) : 0.0f;  // kind 0x15
}
// ---- P94 end

}  // namespace fuel

namespace fuel {

// ---- PK05: Decor_FreeOfObjects 0x557650 and its three sub-tests.
namespace {
constexpr double kClearCell = 128.0;
int64_t clearKey(int64_t cx, int64_t cz) { return (cx << 32) ^ (cz & 0xffffffff); }
template <typename F>
void forCells(float x0, float z0, float x1, float z1, F&& f) {
    for (int64_t cz = int64_t(std::floor(z0 / kClearCell)); cz <= int64_t(std::floor(z1 / kClearCell)); ++cz)
        for (int64_t cx = int64_t(std::floor(x0 / kClearCell)); cx <= int64_t(std::floor(x1 / kClearCell)); ++cx) f(clearKey(cx, cz));
}
}  // namespace

void PlantClearance::index() {
    boxGrid_.cells.clear();
    gateGrid_.cells.clear();
    for (size_t i = 0; i < boxes.size(); ++i) {
        const Box& b = boxes[i];
        forCells(b.m[3] - b.radius, b.m[11] - b.radius, b.m[3] + b.radius, b.m[11] + b.radius,
                 [&](int64_t k) { boxGrid_.cells[k].push_back(uint32_t(i)); });
    }
    for (size_t i = 0; i < gates.size(); ++i) {
        const Gate& g = gates[i];
        forCells(g.x - g.box, g.z - g.box, g.x + g.box, g.z + g.box, [&](int64_t k) { gateGrid_.cells[k].push_back(uint32_t(i)); });
    }
}

// 0x6404a0: the grid query over (x - r, z - r)..(x + r, z + r) (record rectangles = position +- float 15), then 0x73f460 per
// record: the sphere centre in the record's frame, squared distance to the box of half extents +0x30..+0x38 (> r^2 stops).
bool PlantClearance::hitsBox(float x, float y, float z, float r) const {
    if (boxes.empty()) return false;
    bool hit = false;
    forCells(x - r, z - r, x + r, z + r, [&](int64_t k) {
        if (hit) return;
        const auto it = boxGrid_.cells.find(k);
        if (it == boxGrid_.cells.end()) return;
        for (uint32_t i : it->second) {
            const Box& b = boxes[i];
            if (x + r < b.m[3] - b.radius || x - r > b.m[3] + b.radius || z + r < b.m[11] - b.radius || z - r > b.m[11] + b.radius) continue;
            const float dx = x - b.m[3], dy = y - b.m[7], dz = z - b.m[11];
            const float local[3] = {b.m[0] * dx + b.m[1] * dy + b.m[2] * dz, b.m[4] * dx + b.m[5] * dy + b.m[6] * dz,
                                    dz * b.m[10] + b.m[9] * dy + b.m[8] * dx};
            const float r2 = r * r;
            float sum = 0;
            bool inside = true;
            for (int a = 0; a < 3; ++a) {
                float d = 0;
                if (-b.ext[size_t(a)] > local[a]) d = b.ext[size_t(a)] + local[a];
                else if (b.ext[size_t(a)] < local[a]) d = local[a] - b.ext[size_t(a)];
                else { continue; }
                sum += d * d;
                if (sum > r2) { inside = false; break; }
            }
            if (inside) { hit = true; return; }
        }
    });
    return hit;
}

// 0x4f8a80 + 0x4f8e00. The grid hands back the gates whose box (start / finish ((n + 1) / 2 + 1) x 10, checkpoints +-25)
// overlaps the sphere's box; each blocks when the xz distance is below road width + half + r (the half is the box size).
bool PlantClearance::missionsFree(float x, float z, float r) const {
    if (gates.empty()) return true;
    bool free = true;
    forCells(x - r, z - r, x + r, z + r, [&](int64_t k) {
        if (!free) return;
        const auto it = gateGrid_.cells.find(k);
        if (it == gateGrid_.cells.end()) return;
        for (uint32_t i : it->second) {
            const Gate& g = gates[i];
            if (x + r < g.x - g.box || x - r > g.x + g.box || z + r < g.z - g.box || z - r > g.z + g.box) continue;
            const float dx = x - g.x, dz = z - g.z;
            const float half = g.box + r;
            const float d = std::sqrt(dx * dx + dz * dz);
            if (!(g.road + half <= d)) { free = false; return; }
        }
    });
    return free;
}

// 0x4ed9e0: POIs of the grid cells around the sphere; any within r + 25 (xz, strictly) blocks.
bool PlantClearance::poisFree(float x, float z, float r) const {
    const float lim = (r + 25.0f) * (r + 25.0f);
    for (const auto& p : pois) {
        const float dx = x - p[0], dz = z - p[1];
        if (dz * dz + dx * dx < lim) return false;
    }
    return true;
}

}  // namespace fuel
