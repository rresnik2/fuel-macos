// AutoMesh / RoadMesh: see automesh.hpp. Addresses are FUEL.exe; decode notes in
// ~/re-work/fuel/notes/p36-objects.md (A1-A9) on top of P31 out-B section 2.
#include "automesh.hpp"

#include "assembly.hpp"
#include "ground_point.hpp"
#include "resources.hpp"
#include "road_segments.hpp"
#include "roads.hpp"
#include "terrain.hpp"
#include "terrain_height.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>

namespace fuel {

namespace {

constexpr uint32_t kNodeClass = 0x85D02528;
constexpr double kHashX = 12.317513, kHashZ = 10.193578;  // [0x9dc204], [0x9dc200]
constexpr double kInv255 = 0.003921569;                    // [0x9dbfcc]

using M4 = std::array<double, 16>;  // row vectors: p' = p M, translation 12..14

M4 identity() { return {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1}; }
M4 mul(const M4& a, const M4& b) {
    M4 r{};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) {
            double v = 0;
            for (int k = 0; k < 4; ++k) v += a[size_t(i * 4 + k)] * b[size_t(k * 4 + j)];
            r[size_t(i * 4 + j)] = v;
        }
    return r;
}
M4 inverseAffine(const M4& m) {  // general 3x3 inverse + translation
    const double a = m[0], b = m[1], c = m[2], d = m[4], e = m[5], f = m[6], g = m[8], h = m[9], k = m[10];
    const double A = e * k - f * h, B = -(d * k - f * g), C = d * h - e * g;
    const double det = a * A + b * B + c * C;
    M4 r = identity();
    if (std::fabs(det) < 1e-12) return r;
    const double id = 1.0 / det;
    r[0] = A * id, r[1] = -(b * k - c * h) * id, r[2] = (b * f - c * e) * id;
    r[4] = B * id, r[5] = (a * k - c * g) * id, r[6] = -(a * f - c * d) * id;
    r[8] = C * id, r[9] = -(a * h - b * g) * id, r[10] = (a * e - b * d) * id;
    for (int j = 0; j < 3; ++j) r[size_t(12 + j)] = -(m[12] * r[size_t(j)] + m[13] * r[size_t(4 + j)] + m[14] * r[size_t(8 + j)]);
    return r;
}
M4 rotY(double yaw) {  // 0x40f8b0: rotation about +y
    M4 r = identity();
    const double c = std::cos(yaw), s = std::sin(yaw);
    r[0] = c, r[2] = -s, r[8] = s, r[10] = c;
    return r;
}
// Rotation taking +y to n (0x6443a0 quaternion from-to), as a row-vector matrix.
M4 fromUp(const double n[3]) {
    M4 r = identity();
    const double ax = n[2], az = -n[0];  // up x n
    const double s = std::sqrt(ax * ax + az * az), c = n[1];
    if (s < 1e-9) return r;
    const double ux = ax / s, uz = az / s, t = 1 - c;
    // Rodrigues about (ux, 0, uz) by acos(c); column-vector R, stored transposed for row vectors.
    const double R[3][3] = {{t * ux * ux + c, -s * uz, t * ux * uz},
                            {s * uz, c, -s * ux},
                            {t * ux * uz, s * ux, t * uz * uz + c}};
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) r[size_t(i * 4 + j)] = R[j][i];
    return r;
}

uint8_t rt(const Permutation& p, double v) { return p[size_t(int64_t(std::floor(v)) & 255)]; }  // fistp, RC_DOWN

// The decor quadtree leaf holding (px, pz) for a camera at (cx, cz) (p15 D1, P31 D: 12288 m
// roots, split while the camera is within 2.0625 x size of the chunk centre, 24 m leaves).
// P94: `slack` > 0 gives the smallest leaf for a camera anywhere within `slack` of (cx, cz).
double leafSize(double px, double pz, double camX, double camZ, double slack = 0) {
    constexpr double kRoot = 12288.0, kSplit = 2.0625, kMinChunk = 24.01;
    double size = kRoot;
    double ox = std::round(px / kRoot) * kRoot, oz = std::round(pz / kRoot) * kRoot;
    while (size > kMinChunk) {
        const double r = size * kSplit + slack;
        if ((ox - camX) * (ox - camX) + (oz - camZ) * (oz - camZ) >= r * r) break;
        ox += (px >= ox ? 0.25 : -0.25) * size;
        oz += (pz >= oz ? 0.25 : -0.25) * size;
        size *= 0.5;
    }
    return size;
}

// 0x78cbf0 / 0x78e2c0 emit cull for a chunk of size C (W+0xa78 = 30, +0xa7c = +0xa80 = 0.033,
// split 2.0625).
bool culled(bool noOcclusion, float radius, float viewDist, double C) {
    if (noOcclusion) return false;
    if (viewDist > 0 && (C * 0.707106 + viewDist) / 2.0625 < C) return true;
    if (C > 30.0 && radius < 0.033 * C) return true;
    return false;
}
bool culled(const AutoMeshRule& r, double C) { return culled(r.noOcclusion, r.radius, r.viewDist, C); }

// Ground normal over +-r (0x64e130): (h(x-r) - h(x+r), 2r, h(z-r) - h(z+r)), normalised.
void groundNormal(const TerrainDetail& d, double x, double z, double r, double n[3]) {
    n[0] = d.ground(x - r, z) - d.ground(x + r, z);
    n[1] = 2 * r;
    n[2] = d.ground(x, z - r) - d.ground(x, z + r);
    const double l = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
    for (int i = 0; i < 3; ++i) n[i] /= l;
}

// 0x74f9b0(W, r, maxRange, maxResidual, 2r): an n x n lattice over the square of half side r
// (n = floor(r) + 1, at most 5); fails when the height range exceeds maxRange or the
// least-squares plane leaves a point more than maxResidual away. Port: max |residual|.
bool flatEnough(const TerrainDetail& d, double x, double z, double r, double maxRange, double maxResidual) {
    int n = std::min(int(std::floor(r)) + 1, 5);
    if (n < 2) n = 2;
    const double step = 2 * r / (n - 1);
    double h[5][5], lo = 1e30, hi = -1e30;
    double sx = 0, sz = 0, sh = 0, sxx = 0, szz = 0, sxz = 0, sxh = 0, szh = 0;
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j) {
            const double px = -r + i * step, pz = -r + j * step;
            h[i][j] = d.ground(x + px, z + pz);
            lo = std::min(lo, h[i][j]), hi = std::max(hi, h[i][j]);
            if (hi - lo > maxRange) return false;
            sx += px, sz += pz, sh += h[i][j], sxx += px * px, szz += pz * pz, sxz += px * pz, sxh += px * h[i][j], szh += pz * h[i][j];
        }
    const double N = n * n, mx = sx / N, mz = sz / N, mh = sh / N;
    const double cxx = sxx / N - mx * mx, czz = szz / N - mz * mz, cxz = sxz / N - mx * mz;
    const double cxh = sxh / N - mx * mh, czh = szh / N - mz * mh, det = cxx * czz - cxz * cxz;
    const double a = std::fabs(det) > 1e-12 ? (cxh * czz - czh * cxz) / det : 0;
    const double b = std::fabs(det) > 1e-12 ? (czh * cxx - cxh * cxz) / det : 0;
    for (int i = 0; i < n; ++i)
        for (int j = 0; j < n; ++j) {
            const double px = -r + i * step, pz = -r + j * step;
            if (std::fabs(h[i][j] - (mh + a * (px - mx) + b * (pz - mz))) > maxResidual) return false;
        }
    return true;
}

// Climate env of 0x78b470 (P31 out-B 2.2).
struct Env {
    double y = 0;
    float pop = 0, stormed = 0, incinerated = 0, snowed = 0, flooded = 0, sanded = 0, forest = 0, humidity = 0, roughness = 0;
};

bool climateOK(const AutoMeshRule& r, const Env& e) {
    if (r.onForest && e.forest <= 0.5f) return false;
    if (r.noForest && e.forest > 0.5f) return false;
    if (r.country && !(0.1f < e.pop && e.pop <= 0.5f)) return false;
    if (r.town && !(0.5f < e.pop && e.pop <= 0.75f)) return false;
    if (r.city && e.pop <= 0.75f) return false;
    auto onNo = [](bool on, bool no, float v) { return !(on && v <= 0.5f) && !(no && v > 0.5f); };
    if (!onNo(r.onStormed, r.noStormed, e.stormed) || !onNo(r.onIncinerated, r.noIncinerated, e.incinerated) ||
        !onNo(r.onSnowed, r.noSnowed, e.snowed) || !onNo(r.onFlooded, r.noFlooded, e.flooded) || !onNo(r.onSanded, r.noSanded, e.sanded))
        return false;
    if (r.onMountain && e.y <= 1000) return false;
    if (r.noMountain && e.y > 1000) return false;
    if (r.onDesert && e.humidity > 0.33f) return false;
    if (r.noDesert && e.humidity <= 0.33f) return false;
    if (e.humidity < r.humidityMin || e.humidity > r.humidityMax) return false;
    if (e.roughness < r.roughnessMin || e.roughness > r.roughnessMax) return false;
    if (r.onFrozen && e.y <= 2000) return false;
    if (r.noFrozen && e.y > 2000) return false;
    return true;
}

Env envAt(const GroundGenerator& g, double x, double y, double z, const GroundPoint& p) {
    Env e;
    e.y = y;
    e.pop = std::clamp(g.map(8, x, z), 0.0f, 1.0f);  // 0x458e10 -> 0x458e60
    e.stormed = g.map(14, x, z);                      // 0x458780
    e.incinerated = g.map(13, x, z);                  // 0x458100
    e.flooded = g.map(15, x, z);                      // 0x457a80
    e.sanded = g.map(16, x, z);                       // 0x4573e0
    e.forest = g.forest(x, z, p.coarseNy);            // 0x45dd90 -> 0x6bd200
    e.humidity = p.humidity;                          // 0x4dde20 (map 2)
    // 0x4de850 (snow line) as the point's high-ground term before the road share; 0x6a6650
    // roughness is not ported (no rule tests Roughness or Snowed).
    e.snowed = p.highGround;
    return e;
}

// A flat surface as 0x79bfd0 / 0x79bea0 return it.
struct Surf {
    bool found = false;
    int mode = 0;
    double cx = 0, cy = 0, cz = 0, size = 0, ax = 0, az = 1;
};

int fileMode(const std::vector<uint8_t>& raw) { return raw.size() > 0x7c ? std::min<int>(raw[0x7c], 6) : 0; }

// 0x79bfd0: surfaces whose inside distance exceeds their margin; the nearest centre wins.
Surf surfaceAt(const TerrainDetail& d, double x, double z) {
    Surf s;
    double best = 1e8;
    for (const TerrainDetail::FileFlatInfo& f : d.fileFlatsAt(x, z)) {
        if (!(f.d > f.margin)) continue;
        const double dist = std::hypot(f.cx - x, f.cz - z);
        if (dist < best) {
            best = dist;
            s.found = true, s.mode = fileMode(f.raw), s.cx = f.cx, s.cy = f.cy, s.cz = f.cz;
            s.size = std::min(f.lenX, f.lenZ) - f.margin;
        }
    }
    for (const TerrainDetail::ProcFlatInfo& f : d.proceduralFlats(x, z, 120.0)) {
        // Inside test of a procedural flat (rectangle along the road), as 0x799020 without jitter.
        const double dx = x - f.cx, dz = z - f.cz;
        const double along = std::fabs(dx * f.ax + dz * f.az), across = std::fabs(-dx * f.az + dz * f.ax);
        const double inside = std::min(f.lenX - along, f.lenZ - across);
        if (!(inside > f.margin)) continue;
        const double dist = std::hypot(dx, dz);
        if (dist < best) {
            best = dist;
            s.found = true, s.mode = f.mode, s.cx = f.cx, s.cy = f.cy, s.cz = f.cz;
            s.size = std::min(f.lenX, f.lenZ) - f.margin;
        }
    }
    return s;
}

// 0x79bea0: the surface overlapping the cell rectangle whose centre is nearest the cell centre.
Surf surfaceOverlapping(const TerrainDetail& d, double x0, double z0, double cell) {
    Surf s;
    double best = 1e11;
    const double mx = x0 + cell * 0.5, mz = z0 + cell * 0.5;
    std::vector<uint32_t> seen;
    for (int k = 0; k < 5; ++k) {
        const double px = k == 4 ? mx : x0 + (k & 1) * cell, pz = k == 4 ? mz : z0 + (k >> 1) * cell;
        for (const TerrainDetail::FileFlatInfo& f : d.fileFlatsAt(px, pz)) {
            if (std::find(seen.begin(), seen.end(), f.index) != seen.end()) continue;
            seen.push_back(f.index);
            const double dist = std::hypot(mx - f.cx, mz - f.cz);
            if (dist < best) {
                best = dist;
                s.found = true, s.mode = fileMode(f.raw), s.cx = f.cx, s.cy = f.cy, s.cz = f.cz;
                s.size = std::min(f.lenX, f.lenZ) - f.margin;
                // +0x30 = the file X axis (raw +4 / +12) flattened: the yaw atan2(+0x30.x, +0x38).
                float X[3];
                std::memcpy(X, f.raw.data() + 4, 12);
                const double h = std::hypot(X[0], X[2]);
                s.ax = h > 0 ? X[0] / h : 0, s.az = h > 0 ? X[2] / h : 1;
            }
        }
    }
    return s;
}

struct NearRoad {
    bool found = false;
    double d = 1e8;           // 0x6fb3f0 centreline distance
    double px = 0, pz = 0;    // nearest centreline point
    double dx = 0, dz = 1;    // road direction
    int type = 0;
    float edge = 10000.0f;    // 0x6fac40
};

NearRoad nearestRoad(const RoadSampler* rs, double x, double z) {
    NearRoad n;
    if (rs == nullptr) return n;
    const RoadSampler::Nearest r = rs->nearestRoad(x, z, 200.0);
    n.edge = r.edge;
    if (r.seg == nullptr) return n;
    n.found = true;
    n.px = r.px, n.pz = r.pz;
    n.d = std::hypot(x - r.px, z - r.pz);
    n.dx = r.seg->dx, n.dz = r.seg->dz;
    n.type = r.seg->type;
    return n;
}

float halfWidth(const RoadTypeTable* t, int type) {  // 0xa678ac + type * 0x200 = g_RoadTypes +0xcc
    return t != nullptr && t->valid && type >= 0 && type < int(t->types.size()) ? t->types[size_t(type)].at(0xcc) : 3.0f;
}

// ---- rule parsing (0x78a450, defaults 0x78a2c0) ----
bool numberAfter(const std::string& tok, const char* prefix, int& v) {  // 0x78a1e0: sscanf("PREFIX%d")
    const size_t n = std::strlen(prefix);
    if (tok.size() <= n || strncasecmp(tok.c_str(), prefix, n) != 0) return false;
    const char* s = tok.c_str() + n;
    char* end = nullptr;
    const long x = std::strtol(s, &end, 10);
    if (end == s) return false;
    v = int(x);
    return true;
}

std::vector<std::vector<std::string>> readRules(const std::string& path) {
    std::vector<std::vector<std::string>> out;
    std::ifstream f(path);
    std::string line;
    while (std::getline(f, line)) {
        std::istringstream ss(line);
        std::vector<std::string> tok;
        std::string t;
        while (ss >> t) tok.push_back(t);
        if (tok.size() >= 2 && strcasecmp(tok[0].c_str(), "ADDMESH:") == 0) out.push_back(std::vector<std::string>(tok.begin() + 1, tok.end()));
    }
    return out;
}

void parseKeywords(AutoMeshRule& r, const std::vector<std::string>& toks, int& cls) {
    struct Flag {
        const char* k;
        bool AutoMeshRule::*f;
    };
    static const Flag flags[] = {
        {"OnHub", &AutoMeshRule::onHub}, {"NoHub", &AutoMeshRule::noHub}, {"InDeepWater", &AutoMeshRule::inDeepWater},
        {"InShallowWater", &AutoMeshRule::inShallowWater}, {"OnLakeBeach", &AutoMeshRule::onLakeBeach},
        {"FarFromRoad", &AutoMeshRule::farFromRoad}, {"OnTerra", &AutoMeshRule::onTerra}, {"CloseToRoad", &AutoMeshRule::closeToRoad},
        {"OnRockyArea", &AutoMeshRule::onRockyArea}, {"GulchSide", &AutoMeshRule::gulchSide}, {"RoadMiddle", &AutoMeshRule::roadMiddle},
        {"GroundNormal", &AutoMeshRule::groundNormal}, {"ParalelRoad", &AutoMeshRule::paralelRoad}, {"DownHill", &AutoMeshRule::downHill},
        {"CultivatedField", &AutoMeshRule::cultivatedField}, {"CountryPopulation", &AutoMeshRule::country},
        {"TownPopulation", &AutoMeshRule::town}, {"CityPopulation", &AutoMeshRule::city}, {"TakeStoneColor", &AutoMeshRule::takeStoneColor},
        {"OnStormed", &AutoMeshRule::onStormed}, {"OnIncinerated", &AutoMeshRule::onIncinerated}, {"OnFlooded", &AutoMeshRule::onFlooded},
        {"OnSanded", &AutoMeshRule::onSanded}, {"OnSnowed", &AutoMeshRule::onSnowed}, {"NoSnowed", &AutoMeshRule::noSnowed},
        {"NoStormed", &AutoMeshRule::noStormed}, {"NoIncinerated", &AutoMeshRule::noIncinerated}, {"NoFlooded", &AutoMeshRule::noFlooded},
        {"NoSanded", &AutoMeshRule::noSanded}, {"OnFrozen", &AutoMeshRule::onFrozen}, {"OnDesert", &AutoMeshRule::onDesert},
        {"OnMountain", &AutoMeshRule::onMountain}, {"NoFrozen", &AutoMeshRule::noFrozen}, {"NoDesert", &AutoMeshRule::noDesert},
        {"NoMountain", &AutoMeshRule::noMountain}, {"OnForest", &AutoMeshRule::onForest}, {"NoForest", &AutoMeshRule::noForest},
        {"RandomRot", &AutoMeshRule::randomRot}, {"NoOcclusion", &AutoMeshRule::noOcclusion}, {"AtTheWind", &AutoMeshRule::atTheWind},
        {"LinePosition", &AutoMeshRule::linePosition}, {"ShortCut", &AutoMeshRule::shortCut},
    };
    struct Road {
        const char* k;
        uint32_t mask;
    };
    static const Road roads[] = {{"RunWay", 1u << 19}, {"HighWay", 1u << 18}, {"BigTarmacRoad", 1u << 12}, {"NormalTarmacRoad", 1u << 11},
                                 {"SmallTarmacRoad", 1u << 10}, {"BigDirtRoad", 1u << 9}, {"NormalDirtRoad", 1u << 8},
                                 {"SmallDirtRoad", 1u << 7}, {"FieldRoad", 1u << 6}, {"GoatPath", (1u << 4) | (1u << 5)}};
    for (const std::string& t : toks) {
        if (strcasecmp(t.c_str(), "OnATVPark") == 0) r.onATVPark = r.onTerra = true;
        for (const Flag& f : flags)
            if (strcasecmp(t.c_str(), f.k) == 0) r.*(f.f) = true;
        for (const Road& rd : roads)
            if (strcasecmp(t.c_str(), rd.k) == 0) r.roadTypes |= rd.mask;
        if (strcasecmp(t.c_str(), "VeryFlatGround") == 0) r.pente = 0.05f;
        if (strcasecmp(t.c_str(), "FlatGround") == 0) r.pente = 0.3f;
        if (strcasecmp(t.c_str(), "EdgyGround") == 0) r.pente = 1.5f;
        if (strcasecmp(t.c_str(), "VeryRare") == 0) r.probability = 0.01f;
        if (strcasecmp(t.c_str(), "FairlyRare") == 0) r.probability = 0.05f;
        if (strcasecmp(t.c_str(), "FairlyFrequent") == 0) r.probability = 0.2f;
        if (strcasecmp(t.c_str(), "VeryFrequent") == 0) r.probability = 1.0f;
        int v = 0;
        if (numberAfter(t, "Probability", v)) r.probability = v * 0.01f;
        if (numberAfter(t, "GroundMaxPente", v)) r.pente = v * 0.01f;
        if (numberAfter(t, "GroundMaxTrou", v)) r.trou = v * 0.01f;
        if (numberAfter(t, "MaxDistToRoad", v)) r.maxDistToRoad = float(v);
        if (numberAfter(t, "MinDistToRoad", v)) r.minDistToRoad = float(v);
        if (numberAfter(t, "Frequency", v)) r.frequency = float(std::max(v, 5));
        if (numberAfter(t, "Phase", v)) r.phase = float(v);
        if (numberAfter(t, "CrossingDist", v)) r.crossingDist = float(v);
        if (numberAfter(t, "SideDist", v)) r.sideDist = float(v);
        if (numberAfter(t, "MiddleDist", v)) r.middleDist = float(v);
        if (numberAfter(t, "ExteriorCurveMinCurve", v)) r.extMin = float(v);
        if (numberAfter(t, "ExteriorCurveMaxCurve", v)) r.extMax = float(v);
        if (numberAfter(t, "InteriorCurveMinCurve", v)) r.intMin = float(v);
        if (numberAfter(t, "InteriorCurveMaxCurve", v)) r.intMax = float(v);
        if (numberAfter(t, "HumidityMin", v)) r.humidityMin = v * 0.01f;
        if (numberAfter(t, "HumidityMax", v)) r.humidityMax = v * 0.01f;
        if (numberAfter(t, "RoughnessMin", v)) r.roughnessMin = v * 0.01f;
        if (numberAfter(t, "RoughnessMax", v)) r.roughnessMax = v * 0.01f;
        if (numberAfter(t, "OnTerraMiddle", v) || numberAfter(t, "OnTerraMiddleMax", v)) {
            r.terraMiddleMax = float(v) <= 0.01f ? r.radius : float(v);
            r.onTerra = true, cls = 2;
        }
        if (numberAfter(t, "OnTerraMiddleMin", v)) {
            r.terraMiddleMin = float(v) <= 0.01f ? r.radius : float(v);
            r.onTerra = true, cls = 2;
        }
    }
}

}  // namespace

AutoMesh::AutoMesh(const std::string& commonComponent, const DpcFile& usa1, const std::vector<const DpcFile*>& lookup) {
    // Node tree of USA1 (in-memory +0x98 object, +0xa4 child, +0xac next = body links 4, 1, 3).
    struct N {
        uint32_t child = 0, next = 0, object = 0, objName = 0;
        float lodScale = 1, sphere[4] = {0, 0, 0, 0};
        std::array<float, 16> world{};
        bool fit = false, normal = false;
    };
    std::unordered_map<uint32_t, N> nodes;
    for (uint32_t id : usa1.ids(kNodeClass)) {
        try {
            const auto b = usa1.body(*usa1.find(id));
            if (!b || b->size() != 234) continue;
            const Node node = parseNodeBody(*b);
            N n;
            uint32_t links[4];
            std::memcpy(links, b->data(), 16);
            n.child = links[1], n.next = links[3], n.object = node.object, n.lodScale = node.lodScale, n.world = node.world;
            std::memcpy(n.sphere, b->data() + 40 + 0x50, 16);  // node +0x60..+0x6c bounding sphere
            if (n.object != 0)
                for (const DpcFile* f : lookup)
                    if (const DpcFile::Entry* e = f->find(n.object); e != nullptr && e->header != nullptr) {
                        try {
                            n.objName = parseObjectLink(f->link(*e)).name;
                        } catch (const std::exception&) {
                        }
                        break;
                    }
            if (node.userDefine != 0)
                for (const DpcFile* f : lookup)
                    if (const DpcFile::Entry* e = f->find(node.userDefine); e != nullptr && e->header != nullptr) {
                        if (const auto ub = f->body(*e))
                            for (const std::string& line : parseUserDefineBody(*ub)) {
                                if (line == "FITTOGROUND") n.fit = true;
                                if (line == "GROUNDNORMAL") n.normal = true;
                            }
                        break;
                    }
            nodes.emplace(id, n);
        } catch (const std::exception&) {
        }
    }
    // The roots the lookup walks: each resident (command-less) WorldRef's root node (body u32 0),
    // in USA1's WorldRef order (0x679ec0 walks the manager list at +0x30).
    std::vector<uint32_t> roots;
    for (uint32_t id : usa1.ids(asoboCrc("WorldRef_Z"))) {
        try {
            const auto b = usa1.body(*usa1.find(id));
            if (b && b->size() >= 4) {
                uint32_t root;
                std::memcpy(&root, b->data(), 4);
                if (nodes.count(root)) roots.push_back(root);
            }
        } catch (const std::exception&) {
        }
    }
    // 0x65eca0: a node matches when its object's name is the CRC; group nodes (no object) are
    // descended into, siblings follow +0xac.
    std::function<uint32_t(uint32_t, uint32_t, bool, int)> search = [&](uint32_t node, uint32_t crc, bool siblings, int depth) -> uint32_t {
        while (node != 0 && depth < 256) {
            const auto it = nodes.find(node);
            if (it == nodes.end()) return 0;
            const N& n = it->second;
            if (n.object != 0 && n.objName == crc) return node;
            if (n.object == 0 && n.child != 0)
                if (const uint32_t f = search(n.child, crc, true, depth + 1)) return f;
            if (!siblings) return 0;
            node = n.next;
            ++depth;
        }
        return 0;
    };
    auto buildTemplate = [&](uint32_t root) {
        Template t;
        const N& r = nodes.at(root);
        M4 rw;
        for (size_t k = 0; k < 16; ++k) rw[k] = r.world[k];
        t.rootInverse = inverseAffine(rw);
        // 0x65efa0: the node and its descendants.
        std::vector<uint32_t> stack = {root};
        int guard = 0;
        while (!stack.empty() && ++guard < 4096) {
            const uint32_t id = stack.back();
            stack.pop_back();
            const auto it = nodes.find(id);
            if (it == nodes.end()) continue;
            for (PlacedMesh& pm : collectNodeMeshes(usa1, lookup, id)) {
                Part p;
                p.mesh = std::move(pm);
                p.fit = it->second.fit, p.normal = it->second.normal;
                p.radius = std::max(it->second.sphere[3], 1.0f);
                t.parts.push_back(std::move(p));
            }
            for (uint32_t c = it->second.child; c != 0 && nodes.count(c) && guard < 4096; c = nodes.at(c).next, ++guard) stack.push_back(c);
        }
        return t;
    };
    // Template radius (0x67a060: the node spheres merged) and view distance (0x67a1e0).
    auto measure = [&](uint32_t root, float& radius, float& view) {
        const N& r = nodes.at(root);
        radius = r.sphere[3];
        view = -1.0f;  // [0xa05224]
        const LodView lv;
        std::vector<uint32_t> stack = {root};
        int guard = 0;
        while (!stack.empty() && ++guard < 4096) {
            const N& n = nodes.at(stack.back());
            stack.pop_back();
            const double d = std::sqrt(double(n.sphere[0] - r.sphere[0]) * (n.sphere[0] - r.sphere[0]) +
                                       double(n.sphere[1] - r.sphere[1]) * (n.sphere[1] - r.sphere[1]) +
                                       double(n.sphere[2] - r.sphere[2]) * (n.sphere[2] - r.sphere[2]));
            radius = std::max(radius, float(d + n.sphere[3]));
            if (n.object != 0) {
                float v = -1.0f, objView = 0;
                for (const DpcFile* f : lookup)
                    if (const DpcFile::Entry* e = f->find(n.object); e != nullptr && e->header != nullptr) {
                        const std::vector<uint8_t> link = f->link(*e);
                        if (link.size() >= 92) std::memcpy(&objView, &link[88], 4);
                        break;
                    }
                if (objView != 0) v = lv.Kfade * n.lodScale * objView * lv.viewScale + 20.0f;
                view = std::max(view, v);
            }
            for (uint32_t c = n.child; c != 0 && nodes.count(c) && guard < 4096; c = nodes.at(c).next, ++guard) stack.push_back(c);
        }
    };
    // Unlinked top-level nodes listed by the GameObj_Z groups (the World_Z GameObj a5dd0147
    // "GAMEOBJECTS" first, then the others in USA1 order): 1,833 of USA1's 1,953 parentless nodes,
    // e.g. the only TasPlanches01 / DebrisRoute_Planches nodes outside other templates. The engine
    // reaches them through the WorldRefs its GameObj pass lists (0x77bde0 -> 0x67aff0); how they are
    // linked at run time is not decoded (Inferred: searched after the library trees).
    std::vector<uint32_t> loose;
    {
        uint32_t worldGameObj = 0;
        for (uint32_t id : usa1.ids(0x39B67ECB))  // World_Z: WorldResources u32 2 = GameObj
            if (const auto b = usa1.body(*usa1.find(id)); b && b->size() >= 12) std::memcpy(&worldGameObj, b->data() + 8, 4);
        std::vector<uint32_t> gos = usa1.ids(cls::GameObj);
        std::stable_partition(gos.begin(), gos.end(), [&](uint32_t g) { return g == worldGameObj; });
        for (uint32_t g : gos) {
            try {
                const auto b = usa1.body(*usa1.find(g));
                if (!b) continue;
                for (const NodeGroup& grp : parseGameObjBody(*b))
                    for (uint32_t n : grp.nodes)
                        if (nodes.count(n)) loose.push_back(n);
            } catch (const std::exception&) {
            }
        }
    }
    auto resolve = [&](AutoMeshRule& r) {
        r.crc = asoboCrc(r.name);
        uint32_t node = 0;
        for (uint32_t root : roots)
            if ((node = search(root, r.crc, false, 0)) != 0) break;
        for (size_t k = 0; node == 0 && k < loose.size(); ++k)
            node = search(loose[k], r.crc, false, 0);
        if (node == 0) {
            templateOf_.emplace(r.crc, -1);
            return;
        }
        measure(node, r.radius, r.viewDist);
        if (!templateOf_.count(r.crc)) {
            templateOf_[r.crc] = int(templates_.size());
            templates_.push_back(buildTemplate(node));
        }
    };
    // Lists in load order (0x78ce40): class 2 first, then 1, then 0, then RoadMesh.
    const std::string dir = commonComponent + "/AutoMesh/";
    const auto autoRules = readRules(dir + "AutoMesh.txt"), roadRules = readRules(dir + "RoadMesh.txt");
    struct Load {
        int list;
        float lo, hi;
        int cls;
        const std::vector<std::vector<std::string>>* rules;
    };
    const Load loads[] = {{2, -1000, -1000, 2, &autoRules}, {1, 10, 1000, 1, &autoRules}, {0, 0, 10, 0, &autoRules}, {3, 0, 1000, 5, &roadRules}};
    for (const Load& L : loads)
        for (const auto& toks : *L.rules) {
            AutoMeshRule r;
            r.name = toks[0];
            resolve(r);
            int cls = L.cls == 5 ? 5 : (r.radius < L.lo || L.hi <= r.radius ? -1 : L.cls);
            parseKeywords(r, std::vector<std::string>(toks.begin() + 1, toks.end()), cls);
            if (cls != L.cls) continue;
            r.cls = cls;
            r.index = int(lists_[size_t(L.list)].size());
            lists_[size_t(L.list)].push_back(std::move(r));
        }
}

// AutoMesh_PlaceInCell 0x78b6b0 for the cell with corner (x0, z0).
void AutoMesh::placeInCell(const Context& ctx, int list, double x0, double z0, double cell, Spawn& out) const {
    out.rule = -1;
    const TerrainDetail& det = *ctx.terrain->detail;
    const GroundGenerator& g = *ctx.ground;
    const Permutation& RT = g.permutation();
    double rnd = RT[size_t(int64_t(std::floor(z0 * kHashZ + x0 * kHashX)) & 255)] * kInv255;
    Surf surf;
    double s = 0;
    if (list == 2) {
        surf = surfaceOverlapping(det, x0, z0, cell);
        if (!surf.found) return;
        s = surf.size;
    } else {
        surf = surfaceAt(det, x0, z0);
        if (surf.found) s = surf.size;
    }
    const int mode = surf.found ? surf.mode : 0;
    if (mode == 6 || mode == 5) return;
    if (list == 2 ? mode != 1 : mode == 1) return;
    if (g.map(18, x0, z0) > 0.5f) return;  // 0x646730
    // The candidate: the cell corner jittered by a hash byte (0..15 m each way).
    const uint8_t b = rt(RT, x0 * kHashX + z0 * kHashZ + 37.8999);  // [0x9dd530]
    const double px = x0 + (b & 15), pz = z0 + (b >> 4);
    const double py = det.ground(px, pz);  // 0x63ef50
    const GroundPoint gp = g.at(px, pz);    // 0x6ac460
    const bool onHub = std::hypot(pz + 31400.0, px + 50900.0) < 500.0;  // 0x78a000
    bool haveEnv = false, haveRoad = false;
    Env env;
    NearRoad road;
    auto needRoad = [&]() {
        if (!haveRoad) road = nearestRoad(ctx.roads, px, pz), haveRoad = true;
    };
    auto needEnv = [&](double x, double y, double z) {
        if (!haveEnv) env = envAt(g, x, y, z, gp), haveEnv = true;
    };
    struct Cand {
        int rule;
        M4 m;
        double x, y, z;
        bool moved;
    };
    std::vector<Cand> cands;
    bool normalCached = false;
    double cachedN[3] = {0, 1, 0};
    const std::vector<AutoMeshRule>& rules = lists_[size_t(list)];
    for (const AutoMeshRule& r : rules) {
        rnd = rt(RT, (r.index * 0.02597731 * 931.9283 + px + rnd) * kHashX + pz * kHashZ) * kInv255;
        if (rnd > r.probability) continue;
        if (haveEnv && !climateOK(r, env)) continue;
        if ((r.noHub && onHub) || (r.onHub && !onHub)) continue;
        const double yawRnd = rt(RT, (rnd * 931.9283 + px) * kHashX + pz * kHashZ) * kInv255;
        double yaw = r.atTheWind ? 0.0 : yawRnd * 6.28;
        double x = px, y = py, z = pz;
        bool moved = false, ok = false;
        if (mode == 1) {
            if (!r.onATVPark && r.terraMiddleMax > 0 && s <= r.terraMiddleMax && r.terraMiddleMin <= s) {
                // 0x789d80 (fits the surface) not decoded: taken as fitting. Position = the surface
                // centre, yaw along its +0x30 axis.
                x = surf.cx, z = surf.cz, y = det.ground(x, z);
                yaw = std::atan2(surf.ax, surf.az);
                needEnv(px, py, pz);
                moved = true;
                ok = climateOK(r, env);
            }
        } else if (mode == 2 || mode == 3) {
            if (!r.onATVPark && r.onTerra && r.terraMiddleMax <= 0) {
                needRoad();
                const double dx = px - road.px, dz = pz - road.pz, l = std::hypot(dx, dz);
                if (road.found && l >= 1.0) {
                    const double o = halfWidth(ctx.types, road.type) + r.radius;
                    x = road.px + dx / l * o, z = road.pz + dz / l * o, y = det.ground(x, z);
                    // 0x6fa9a0 road record length >= radius: not ported (records are long).
                    needEnv(px, py, pz);
                    moved = true;
                    ok = climateOK(r, env);
                }
            }
        } else if (mode == 4 ? r.onATVPark : !r.onTerra) {
            // LAB_0078bd93: free ground.
            if (r.roadMiddle) continue;  // no AutoMesh rule uses it (road centre placement not ported)
            needRoad();
            const double d = road.d;
            if (d > r.maxDistToRoad || d < r.minDistToRoad) continue;
            if (r.inDeepWater || r.inShallowWater || r.onLakeBeach) continue;
            needEnv(px, py, pz);
            if (!climateOK(r, env)) continue;
            // 0x789f90 dune test (<= 1): the port's dune share is at most 1.
            const double edge = double(road.edge) - r.radius;  // 0x6fac40 - radius
            if (!(r.roadMiddle || r.onTerra || edge >= 1.0)) continue;
            if (r.onRockyArea && g.decorDensity(px, pz) < yawRnd * 0.9 + 0.1) continue;  // 0x6b7f10 (p36: >= 0.9 r + 0.1)
            if (r.closeToRoad && edge > 50.0) continue;
            if (r.farFromRoad && edge < 50.0) continue;
            if (r.cultivatedField ? gp.field == 0 : gp.field != 0) continue;  // point bit 0x80000
            if (gp.underWater()) continue;                                 // 0x65a390 lake depth > 0.001
            // P80 (notes p80-terrain-big.md B3): Gulch_Query 0x7a0620 = gulch profile 0x7a00e0 > 0.001 at the point
            // (asm 0x78c271..). A rule without GulchSide (+0x66) is never placed in a gulch; a GulchSide rule needs the hit
            // and moves 10 x toward it (not ported: 3 rules, rejected). Rule byte +0x61 (no flatness test) is never set.
            if (r.gulchSide) continue;
            if (det.inGulch(x, z)) continue;
            if (!flatEnough(det, x, z, r.radius, r.pente * r.radius, r.trou * r.radius)) continue;
            ok = true;
        }
        if (!ok) continue;
        // Orientation (LAB_0078c47c): ParalelRoad, GroundNormal, then the yaw.
        if (r.paralelRoad) {
            needRoad();
            if (road.d < 50.0) yaw = std::atan2(road.dx, road.dz);
        }
        M4 m = identity();
        if (r.groundNormal) {
            double n[3];
            if (std::hypot(z - pz, x - px) <= 0.5) {
                if (!normalCached) groundNormal(det, x, z, 4.0, cachedN), normalCached = true;
                std::copy(cachedN, cachedN + 3, n);
            } else {
                groundNormal(det, x, z, 2.0, n);
            }
            m = fromUp(n);
        }
        m = mul(rotY(yaw), m);
        m[12] = x, m[13] = y, m[14] = z;
        cands.push_back({r.index, m, x, y, z, moved});
    }
    if (cands.empty()) return;
    // The pick: RT byte at the first candidate x 1/255 x 0.999 x count, truncated (0x8e01d0).
    const double f = rt(RT, cands[0].z * kHashZ + cands[0].x * kHashX) * kInv255 * 0.999 * double(cands.size());
    const Cand& c = cands[std::min(size_t(f), cands.size() - 1)];
    if (c.moved) {
        const Surf s2 = surfaceAt(det, c.x, c.z);
        if ((s2.found ? s2.mode : 0) != mode) return;
    }
    if (ctx.free && !ctx.free(c.x, c.z, rules[size_t(c.rule)].radius)) return;  // W[0]
    out.list = int16_t(list), out.rule = int16_t(c.rule), out.m = c.m;
}

std::vector<std::array<double, 2>> offsetPolyline(const std::vector<std::array<double, 2>>& p, double d) {
    const size_t n = std::min<size_t>(p.size(), 512);
    std::vector<std::array<double, 2>> out;
    for (size_t i = 0; i < n; ++i) {
        const auto& a = p[i == 0 ? 0 : i - 1];
        const auto& b = p[i + 1 < n ? i + 1 : n - 1];
        double dx = b[0] - a[0], dz = b[1] - a[1];
        const double l = std::hypot(dx, dz);
        if (l < 1e-6) dx = 0, dz = 1;
        else dx /= l, dz /= l;
        out.push_back({p[i][0] - dz * d, p[i][1] + dx * d});
    }
    return out;
}

bool roadShortCut(const std::vector<std::array<double, 2>>& pts, int start, int count, int step, double px, double pz, double& qx, double& qz) {
    // 0x78e030 (ECX = start, EDI = the offset curve's xz floats, EAX = the sample; args out, count, step). x87 at PC_53:
    // the squared lengths go to fsqrt through a float argument (0x40ed20); the straight distance is kept as a float, the
    // along-road length is the float running sum plus the new segment (asm 0x78e174..0x78e17d).
    const int end = step > 0 ? count - 1 : 0;
    auto len = [](double dx, double dz) { return std::sqrt(double(float(dz * dz + dx * dx))); };
    int k = start;
    if (k < 0 || k >= int(pts.size())) return false;
    float best = 1e6f, d = float(len(pts[size_t(k)][0] - px, pts[size_t(k)][1] - pz));
    double along = len(pts[size_t(k)][0] - px, pts[size_t(k)][1] - pz);
    bool found = false;
    while (d <= 300.0f) {
        if (50.0f < d && d < 300.0f && along / d > 1.5 && d < best) {
            best = d, found = true;
            qx = pts[size_t(k)][0], qz = pts[size_t(k)][1];
        }
        if (k == end) break;
        const auto prev = pts[size_t(k)];
        k += step;
        if (k < 0 || k >= int(pts.size())) break;  // port guard (FUEL stops at `end`)
        d = float(len(pts[size_t(k)][0] - px, pts[size_t(k)][1] - pz));
        along = len(pts[size_t(k)][0] - prev[0], pts[size_t(k)][1] - prev[1]) + double(float(along));
    }
    return found;
}

// RoadMesh along one GwRoad line on one side (+1, -1, 0 = middle): 0x78e2c0 + 0x78d7c0.
void AutoMesh::placeAlongRoad(const Context& ctx, uint32_t lineIndex, int side, std::vector<Spawn>& out) const {
    const RoadNetwork::Line& line = ctx.lines->lines[lineIndex];
    const int type = line.type;
    const TerrainDetail& det = *ctx.terrain->detail;
    const Permutation& RT = ctx.ground->permutation();
    if (line.points.size() < 2) return;
    for (const AutoMeshRule& r : lists_[3]) {
        if ((side != 0 ? r.sideDist : r.middleDist) > 1000.0f) continue;
        // Road class filter (switch at 0x78e4d8): types without a keyword take every rule.
        const bool classed = type == 4 || type == 5 || (type >= 6 && type <= 12) || type == 18 || type == 19;
        if (classed && !(r.roadTypes & (1u << type))) continue;
        const double off = side != 0 ? (halfWidth(ctx.types, type) + r.sideDist) * side : r.middleDist;
        // Offset polyline (0x6f8f80 -> 0x6f7360, offsetPolyline; the smoothing pass 0x6f8ea0 is not ported).
        std::vector<std::array<double, 2>> src;
        for (const auto& q : line.points) src.push_back({double(q[0]), double(q[1])});
        // The port's side +1 is right of travel; FUEL's d = side x (...) moves left (0x78e562), so the port's `side`
        // is FUEL's negated. Kept as before (OPEN: whether 0x78d7c0's interior/exterior and facing tests agree).
        const std::vector<std::array<double, 2>> pts = offsetPolyline(src, -off);
        // Resample every Frequency metres from Phase (<= 512 samples).
        std::vector<std::array<double, 2>> smp;
        std::vector<int> smpSeg;  // the offset-curve segment of each sample (auStack_800, for ShortCut)
        double along = r.phase, acc = 0;
        for (size_t i = 0; i + 1 < pts.size() && smp.size() < 512; ++i) {
            const double dx = pts[i + 1][0] - pts[i][0], dz = pts[i + 1][1] - pts[i][1], l = std::hypot(dx, dz);
            while (along < acc + l && smp.size() < 512) {
                const double t = (along - acc) / l;
                smp.push_back({pts[i][0] + dx * t, pts[i][1] + dz * t});
                smpSeg.push_back(int(i));
                along += r.frequency;
            }
            acc += l;
        }
        const int ns = int(smp.size());
        if (ns < 3) continue;
        const int W = int(std::floor(100.0 / r.frequency + 0.999999));
        const double endLimit = (r.phase + ns * r.frequency - r.frequency) - (r.frequency + r.crossingDist);
        for (int i = 1; i <= ns - 2; ++i) {
            const double a = r.phase + i * r.frequency;
            if (a < r.crossingDist || a > endLimit) continue;
            const double x = smp[size_t(i)][0], z = smp[size_t(i)][1];
            // Curvature: the smallest circumradius over the window (0x78e1f0) and its turn sign.
            double R = 1e15, cross = 0;
            for (int k = W; k >= 1; --k) {
                const auto& p0 = smp[size_t(std::max(i - k, 0))];
                const auto& p2 = smp[size_t(std::min(i + k, ns - 1))];
                const double ax = x - p0[0], az = z - p0[1], bx = p2[0] - x, bz = p2[1] - z;
                const double A = std::hypot(ax, az), B = std::hypot(bx, bz), C = std::hypot(ax + bx, az + bz);
                const double q = (C + B + A) * ((C - A) + B) * ((B - C) + A) * ((A - B) + C);
                double rad = 1e11;
                if (q > 0) {
                    const double sq = std::sqrt(q);
                    if (sq >= 1e-6) rad = C / sq * B * A;
                }
                if (rad < R) R = rad, cross = az * bx - bz * ax;
            }
            auto inRange = [&](float lo, float hi) { return lo >= 0 && (hi <= 0 || (lo <= R && R <= hi)); };
            if (cross * side <= 0 ? !inRange(r.intMin, r.intMax) : !inRange(r.extMin, r.extMax)) continue;
            if (rt(RT, x * kHashX + z * kHashZ) * kInv255 > r.probability) continue;
            // Element ends: LinePosition spans this sample and the next; else the direction x radius.
            const auto& nx = smp[size_t(i + 1)];
            double ex = nx[0] - x, ez = nx[1] - z;
            const double el = std::hypot(ex, ez);
            if (el < 1e-6) continue;
            double cx = x, cz = z;
            if (r.linePosition) cx = (x + nx[0]) * 0.5, cz = (z + nx[1]) * 0.5;
            ex /= el, ez /= el;
            if (!r.linePosition && side < 0) ex = -ex, ez = -ez;
            if (r.shortCut && !r.linePosition) {
                // PK32 (asm 0x78ed57..0x78eedf): a ShortCut rule (only DebrisFeu_Souche03) is placed only where 0x78e030
                // finds the road looping back, backward from the sample's segment k then forward from k + 1, and that
                // point Q lies on the sample's side: n = normalize(-t.z, t.x) (t = the segment, the offset direction of
                // 0x6f7360), (n . D) x FUEL's side >= 0, D = normalize(Q - sample). FUEL's side is the port's negated (above).
                // The instance faces D (flag 1 of 0x78d7c0: the end point sample + D x radius).
                const int k = smpSeg[size_t(i)], cnt = int(pts.size());
                double qx = 0, qz = 0;
                if (!roadShortCut(pts, k, cnt, -1, x, z, qx, qz) && !roadShortCut(pts, k + 1, cnt, 1, x, z, qx, qz)) continue;
                double Dx = qx - x, Dz = qz - z;
                const double Dl = std::hypot(Dx, Dz);
                if (Dl < 1e-9) continue;
                Dx /= Dl, Dz /= Dl;
                const double tx = pts[size_t(k) + 1][0] - pts[size_t(k)][0], tz = pts[size_t(k) + 1][1] - pts[size_t(k)][1];
                if (!((-tz * Dx + tx * Dz) * double(-side) >= 0)) continue;  // the normalisation of n keeps the sign
                ex = Dx, ez = Dz;
            }
            // 0x78d7c0: crossing roads, flat surfaces, climate, clearance, slope.
            if (r.sideDist < 1000.0f) {
                const NearRoad nr = nearestRoad(ctx.roads, cx, cz);
                if (nr.found && nr.d < std::fabs(halfWidth(ctx.types, nr.type) + r.sideDist) - 1.0) continue;
            }
            if (surfaceAt(det, cx, cz).found) continue;  // 0x79bde0
            const double cy = det.ground(cx, cz);
            const GroundPoint gp = ctx.ground->at(cx, cz);
            if (!climateOK(r, envAt(*ctx.ground, cx, cy, cz, gp))) continue;
            if (ctx.free && !ctx.free(cx, cz, r.radius)) continue;
            if (!flatEnough(det, cx, cz, r.radius, r.pente * r.radius, r.trou * r.radius)) continue;
            double yaw = std::atan2(ex, ez) + 1.57;  // [0x9dc4e8]
            if (r.randomRot) yaw = rt(RT, cx * kHashX + cz * kHashZ) * kInv255 * 6.28;
            M4 m = identity();
            if (r.groundNormal) {
                double nn[3];
                groundNormal(det, cx, cz, std::max(1.0f, r.radius), nn);
                m = fromUp(nn);
            }
            m = mul(rotY(yaw), m);
            m[12] = cx, m[13] = cy, m[14] = cz;
            Spawn sp;
            sp.list = 3, sp.rule = int16_t(r.index), sp.m = m;
            out.push_back(sp);
        }
    }
}

std::vector<PlacedMesh> AutoMesh::templateParts(uint32_t crc) const {
    std::vector<PlacedMesh> out;
    const auto it = templateOf_.find(crc);
    if (it == templateOf_.end() || it->second < 0) return out;
    for (const Part& p : templates_[size_t(it->second)].parts) out.push_back(p.mesh);
    return out;
}

// The spawn (0x78d1b0 -> ground element kind 0x17 -> 0x778650): the template's node tree moved
// to the record matrix; FITTOGROUND nodes take the ground height, GROUNDNORMAL nodes its slope.
void AutoMesh::emit(const Context& ctx, const Spawn& s, std::vector<PlacedMesh>& out) const {
    const AutoMeshRule& r = lists_[size_t(s.list)][size_t(s.rule)];
    static const bool list = std::getenv("FUEL_AUTOMESH_LIST") != nullptr;
    if (list) std::printf("automesh %d %s %.1f %.1f %.1f\n", s.list, r.name.c_str(), s.m[12], s.m[13], s.m[14]);
    const auto it = templateOf_.find(r.crc);
    if (it == templateOf_.end() || it->second < 0) return;
    const Template& t = templates_[size_t(it->second)];
    const M4 toRecord = mul(t.rootInverse, s.m);
    const TerrainDetail& det = *ctx.terrain->detail;
    for (const Part& p : t.parts) {
        M4 w;
        for (size_t k = 0; k < 16; ++k) w[k] = p.mesh.world[k];
        w = mul(w, toRecord);
        if (p.normal) {
            // Tilt the node's up axis onto the ground normal over max(node radius, 1) (0x64e130).
            double n[3];
            groundNormal(det, w[12], w[14], p.radius, n);
            const double up[3] = {w[4], w[5], w[6]};
            const double ul = std::sqrt(up[0] * up[0] + up[1] * up[1] + up[2] * up[2]);
            if (ul > 1e-9) {
                // rotation from the current up direction to n, applied to the 3x3 part
                const double u[3] = {up[0] / ul, up[1] / ul, up[2] / ul};
                const M4 a = fromUp(u), b = fromUp(n);
                M4 ai = inverseAffine(a);
                M4 rot = mul(ai, b);
                const double tx = w[12], ty = w[13], tz = w[14];
                w[12] = w[13] = w[14] = 0;
                w = mul(w, rot);
                w[12] = tx, w[13] = ty, w[14] = tz;
            }
        }
        if (p.fit) w[13] = det.ground(w[12], w[14]);  // 0x64e810
        PlacedMesh pm = p.mesh;
        for (size_t k = 0; k < 16; ++k) pm.world[k] = float(w[k]);
        if (list) std::printf("   part %08x at %.1f %.1f %.1f (template %.1f %.1f %.1f) fit %d\n", pm.meshId, w[12], w[13], w[14], p.mesh.world[12],
                              p.mesh.world[13], p.mesh.world[14], int(p.fit));
        out.push_back(std::move(pm));
    }
}

bool autoCellHolds(const PlacedMesh::AutoCell& cell, double camX, double camZ) {
    const double C = leafSize(cell.x, cell.z, camX, camZ);
    return C < cell.maxC && !culled(cell.noOcclusion, cell.radius, cell.viewDist, C);
}

void AutoMesh::generate(const Context& ctx, double camX, double camZ, std::vector<PlacedMesh>& out, size_t* roadFrom,
                        std::vector<Collectible>* collect, std::vector<DecorSound>* decor, double slack) {
    stats_ = {};
    if (roadFrom) *roadFrom = out.size();
    if (ctx.terrain == nullptr || !ctx.terrain->detail || ctx.ground == nullptr) return;
    // Cells: class 0 (20 m) in leaves C < 100 (W+0xa74), classes 1/2 (50 m) in leaves C < 400
    // (W+0xa70 / +0xa6c); C = the leaf holding the cell corner.
    struct Pass {
        int list;
        double cell, maxC, reach;
    };
    const Pass passes[] = {{0, 20.0, 100.0, 2.0625 * 192 + 140}, {1, 50.0, 400.0, 2.0625 * 768 + 560}, {2, 50.0, 400.0, 2.0625 * 768 + 560}};
    for (const Pass& P : passes) {
        if (lists_[size_t(P.list)].empty()) continue;
        const double reach = P.reach + slack;
        const int64_t i0 = int64_t(std::floor((camX - reach) / P.cell)), i1 = int64_t(std::floor((camX + reach) / P.cell));
        const int64_t j0 = int64_t(std::floor((camZ - reach) / P.cell)), j1 = int64_t(std::floor((camZ + reach) / P.cell));
        for (int64_t j = j0; j <= j1; ++j)
            for (int64_t i = i0; i <= i1; ++i) {
                const double x0 = double(i) * P.cell, z0 = double(j) * P.cell;
                // P94: with the slack, the smallest leaf any camera near (camX, camZ) gives this cell; the emit cull
                // below is monotonic in C, so a smaller C passes whatever a larger one does.
                const double C = leafSize(x0 + 0.5, z0 + 0.5, camX, camZ, slack);
                if (C >= P.maxC) continue;
                const uint64_t key = (uint64_t(uint32_t(int32_t(i))) << 32) | uint32_t(int32_t(j));
                auto [cit, fresh] = cellCache_[P.list].try_emplace(key);
                if (fresh) placeInCell(ctx, P.list, x0, z0, P.cell, cit->second.s), ++stats_.cells;
                const Spawn& s = cit->second.s;
                if (s.rule < 0 || culled(lists_[size_t(P.list)][size_t(s.rule)], C)) continue;
                const size_t before = out.size();
                emit(ctx, s, out);
                if (slack > 0) {  // P94: the renderer's per-frame test
                    const AutoMeshRule& r = lists_[size_t(P.list)][size_t(s.rule)];
                    for (size_t k = before; k < out.size(); ++k)
                        out[k].autoCell = {x0 + 0.5, z0 + 0.5, float(P.maxC), r.radius, r.viewDist, r.noOcclusion, true};
                }
                stats_.cellObjects += out.size() > before;
                if (decor && out.size() > before && !decorSound_.empty()) {
                    const AutoMeshRule& r = lists_[size_t(P.list)][size_t(s.rule)];
                    if (decorSound_.count(r.crc)) {  // the node: the mean of the parts' origins (port; the record's y is not the ground)
                        double cx = 0, cy = 0, cz = 0;
                        for (size_t k = before; k < out.size(); ++k) cx += out[k].world[12], cy += out[k].world[13], cz += out[k].world[14];
                        const double n = double(out.size() - before);
                        decor->push_back({cx / n, cy / n, cz / n, r.crc});
                    }
                }
                if (collect && out.size() > before) {
                    const AutoMeshRule& r = lists_[size_t(P.list)][size_t(s.rule)];
                    const bool f1 = r.crc == asoboCrc("IT_FUEL01"), f2 = r.crc == asoboCrc("IT_FUEL02");
                    if (f1 || f2) {  // where the drums stand: the mean of the template parts' origins (port)
                        double cx = 0, cy = 0, cz = 0;
                        for (size_t k = before; k < out.size(); ++k) cx += out[k].world[12], cy += out[k].world[13], cz += out[k].world[14];
                        const double n = double(out.size() - before);
                        collect->push_back({cx / n, cy / n, cz / n, f1 ? 100.0f : 400.0f, r.radius, before, out.size() - before});
                    }
                }
            }
    }
    if (roadFrom) *roadFrom = out.size();
    // Roads (C < 200, W+0xa84): GwRoad lines of the gathered types (0x78f130: 4, 5, 6, 8, 9, 10,
    // 11, 12, 18, 19; not 7) near the camera, both sides and the middle.
    if (ctx.lines != nullptr && ctx.roads != nullptr && !lists_[3].empty()) {
        const double reach = 2.0625 * 384 + 300 + slack;
        const int64_t c0x = int64_t(std::floor((camX - reach) / RoadNetwork::kCell)), c1x = int64_t(std::floor((camX + reach) / RoadNetwork::kCell));
        const int64_t c0z = int64_t(std::floor((camZ - reach) / RoadNetwork::kCell)), c1z = int64_t(std::floor((camZ + reach) / RoadNetwork::kCell));
        std::vector<uint32_t> lines;
        for (int64_t cz = c0z; cz <= c1z; ++cz)
            for (int64_t cx = c0x; cx <= c1x; ++cx) {
                const auto f = ctx.lines->cells.find(RoadNetwork::cellKey(int32_t(cx), int32_t(cz)));
                if (f != ctx.lines->cells.end()) lines.insert(lines.end(), f->second.begin(), f->second.end());
            }
        std::sort(lines.begin(), lines.end());
        lines.erase(std::unique(lines.begin(), lines.end()), lines.end());
        for (uint32_t li : lines) {
            const int type = ctx.lines->lines[li].type;
            if (!(type == 4 || type == 5 || type == 6 || type == 8 || type == 9 || type == 10 || type == 11 || type == 12 || type == 18 || type == 19))
                continue;
            for (int side : {1, -1, 0}) {
                const uint64_t key = uint64_t(li) * 4 + uint64_t(side + 1);
                auto [rit, fresh] = roadCache_.try_emplace(key);
                if (fresh) placeAlongRoad(ctx, li, side, rit->second);
                for (const Spawn& s : rit->second) {
                    const double C = leafSize(s.m[12], s.m[14], camX, camZ, slack);
                    if (C >= 200.0 || culled(lists_[3][size_t(s.rule)], C)) continue;
                    const size_t before = out.size();
                    emit(ctx, s, out);
                    if (slack > 0) {  // P94
                        const AutoMeshRule& r = lists_[3][size_t(s.rule)];
                        for (size_t k = before; k < out.size(); ++k)
                            out[k].autoCell = {s.m[12], s.m[14], 200.0f, r.radius, r.viewDist, r.noOcclusion, true};
                    }
                    stats_.roadObjects += out.size() > before;
                }
            }
        }
    }
    stats_.parts = out.size();
}

}  // namespace fuel
