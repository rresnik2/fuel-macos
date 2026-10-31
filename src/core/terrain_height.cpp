#include "terrain_height.hpp"
#include "terrain.hpp"
#include "road_types.hpp"

#include <cstdio>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <limits>
#include <stdexcept>

namespace fuel {

namespace {

constexpr uint32_t kGenWorldClass = 0x31FA94AE;

double clamp01(double v) { return v < 0 ? 0 : (v > 1 ? 1 : v); }
// 0.5 + 0.5 t (2 - |t|): the engine's soft S-curve on -1..1.
double S2(double t) { return 0.5 + 0.5 * t * (2 - std::fabs(t)); }
// FUEL runs the x87 in round-down mode (notes p21 H0): values stored to float32 round
// toward -infinity. Used where the stored value feeds a sine of a large argument.
double f32(double v) {
    float f = float(v);
    if (double(f) > v) f = std::nextafter(f, -std::numeric_limits<float>::infinity());
    return double(f);
}
double fsin(double v) { return std::sin(f32(v)); }  // FUN_0040f3d0 takes a float

uint32_t rd32(const std::vector<uint8_t>& b, size_t o) {
    if (o + 4 > b.size()) throw std::runtime_error("terrain: short data");
    uint32_t v;
    std::memcpy(&v, &b[o], 4);
    return v;
}
float rdf(const std::vector<uint8_t>& b, size_t o) {
    const uint32_t u = rd32(b, o);
    float f;
    std::memcpy(&f, &u, 4);
    return f;
}

uint64_t cellKey(int32_t cx, int32_t cz) { return (uint64_t(uint32_t(cx)) << 32) | uint32_t(cz); }
constexpr double kCell = 256.0;  // spatial index cells (port choice)

std::atomic<uint32_t> gNextId{1};

}  // namespace

// ---------------------------------------------------------------------------
// GwMapSampler (0x69b620 init, 0x69c500/0x69c560 cell polynomial, B-spline basis).

double MapSampler::value(double x, double z, double* dpx, double* dpz) const {
    if (raw.empty()) {
        if (dpx) *dpx = 0;
        if (dpz) *dpz = 0;
        return 0;
    }
    const double px = x * double(pxPerMetre) + double(width / 2), pz = z * double(pxPerMetre) + double(height / 2);
    // Cell = floor (round-down fistp). The engine mirrors cells beyond [1, W-3]; the
    // port clamps (only the outer 30 m of the 140 km map, outside the playable area).
    double fx = std::floor(px), fz = std::floor(pz);
    const auto ix = int32_t(std::clamp(fx, 1.0, double(width) - 3)), iz = int32_t(std::clamp(fz, 1.0, double(height) - 3));
    const double tx = px - fx, tz = pz - fz;
    auto w = [](double t, double o[4]) {
        const double t2 = t * t, t3 = t2 * t, u = 1 - t;
        o[0] = u * u * u / 6;
        o[1] = (3 * t3 - 6 * t2 + 4) / 6;
        o[2] = (-3 * t3 + 3 * t2 + 3 * t + 1) / 6;
        o[3] = t3 / 6;
    };
    auto dw = [](double t, double o[4]) {
        o[0] = -(1 - t) * (1 - t) / 2;
        o[1] = 1.5 * t * t - 2 * t;
        o[2] = -1.5 * t * t + t + 0.5;
        o[3] = t * t / 2;
    };
    double wx[4], wz[4];
    w(tx, wx);
    w(tz, wz);
    double v = 0, gx = 0, gz = 0;
    double dwx[4], dwz[4];
    const bool grad = dpx || dpz;
    if (grad) {
        dw(tx, dwx);
        dw(tz, dwz);
    }
    for (int j = 0; j < 4; ++j) {
        const uint16_t* row = &raw[size_t(iz - 1 + j) * width + size_t(ix - 1)];
        double rv = 0, rg = 0;
        for (int i = 0; i < 4; ++i) {
            rv += wx[i] * row[i];
            if (grad) rg += dwx[i] * row[i];
        }
        v += wz[j] * rv;
        if (grad) {
            gx += wz[j] * rg;
            gz += dwz[j] * rv;
        }
    }
    if (dpx) *dpx = gx * scale;
    if (dpz) *dpz = gz * scale;
    return v * scale;
}

// ---------------------------------------------------------------------------

struct TerrainDetail::Flat {
    double cx, cy, cz;
    double ax, az, lenX;  // axis X in the ground plane, scaled to its full length
    double bx, bz, lenZ;  // axis Z likewise
    double nx, nz;        // normalised file axis Y (x, z): the plane slope (no division by n.y)
    double invMargin, strength;
    uint8_t priority;
    bool ellipse;
    bool listB;  // file +0x81 != 0 (applied after the roads)
};

struct TerrainDetail::GulchLine {
    std::vector<std::array<double, 2>> pts;
};

struct TerrainDetail::GulchSeg {
    double ax, az, bx, bz;
    double ux, uz, invLen;  // unit direction, 1/length
    double R;
    double minX, minZ, maxX, maxZ;
};

struct TerrainDetail::RoadLine {
    uint8_t type = 0;
    std::vector<std::array<double, 2>> pts;
};

struct TerrainDetail::RoadBuilt {
    struct Seg {
        double ax, az, bx, bz, ux, uz, len, invLen;
        double start;  // chain distance of A
        double minX, minZ, maxX, maxZ;
    };
    std::vector<Seg> segs;
    std::vector<double> profile;  // height every 2 m of chain distance (cm steps), index k <-> s = 2k
};

namespace {

// GwRoad types the engine builds into the ground (0x6fa8a0): not 0-3 (invisible trails),
// 14 (gulches, 0x7a00e0) or 17 (bridges).
bool roadBuiltType(uint8_t t) { return t >= 4 && t <= 20 && t != 14 && t != 17; }
constexpr double kRoadCell = 64.0;

}  // namespace

TerrainDetail::~TerrainDetail() = default;

TerrainDetail::TerrainDetail(const DpcFile& world, MapSampler base) : base_(std::move(base)), id_(gNextId++) {
    if (const char* e = std::getenv("FUEL_TERRAIN_SKIP")) skip_ = std::atoi(e);  // diagnostics: bit 0 roads, 1 flats, 2 gulches, 3 rock, 4 fractals
    const std::vector<uint32_t> gens = world.ids(kGenWorldClass);
    if (gens.empty()) throw std::runtime_error(world.path() + ": no GenWorld_Z");
    const auto body = world.body(*world.find(gens.front()));
    if (!body) throw std::runtime_error("GenWorld_Z has no body");
    const std::vector<uint8_t>& b = *body;

    // Map samplers (0x77bb80): every slot 1..30 with scale 1/maxValue.
    const std::vector<uint32_t> maps = genWorldMaps(world);
    auto loadMap = [&](size_t slot, MapSampler& m) {
        if (slot >= maps.size() || maps[slot] == 0) return;
        const DpcFile::Entry* e = world.find(maps[slot]);
        if (!e) return;
        const auto blob = world.body(*e);
        if (!blob) return;
        Raster r = decodeRaster(*blob);
        m.width = r.width;
        m.height = r.height;
        m.pxPerMetre = float(r.width) * 7.142857e-06f;
        m.scale = 1.0f / r.maxValue;
        m.raw = std::move(r.raw);
    };
    loadMap(10, map10_);
    loadMap(11, map11_);
    loadMap(12, map12_);
    loadMap(18, map18_);

    // Permutation from the GenWorld seed (u32 after the three id lists).
    size_t p = 12;
    for (int list = 0; list < 3; ++list) p += 4 + size_t(rd32(b, p)) * 4;
    perm_ = buildPermutation(world, rd32(b, p));
    p += 4;

    // Fractals (Fractal_Init(L, S, seed, p4, p5, p6) arguments read from the asm).
    f1_ = Fractal(perm_, 3, 15.0f, 11.324f, 0.025f, 0.15f, 0.5f);   // 0x6485c0, altitude +4
    f2_ = Fractal(perm_, 1, 4.7f, 10.324f, 0.025f, 0.15f, 0.5f);    // altitude +0xfd8
    t0c_ = Fractal(perm_, 6, 20000.0f, 95.992332f, 0.3f, 0.3f, 5.0f);  // 0x7a32f0 (p14 G8)
    t18_ = Fractal(perm_, 4, 2000.0f, 39.442329f, 0.3f, 0.3f, 0.5f);
    t24_ = Fractal(perm_, 2, 15.0f, 74.172333f, 0.3f, 0.3f, 0.5f);
    t28_ = Fractal(perm_, 5, 500.0f, 79.442329f, 0.3f, 0.3f, 0.5f);
    canyonC_ = Fractal(perm_, 5, 100.0f, 25.442329f, 0.3f, 0.3f, 2.0f);  // 0x7a0b20 +0xc
    canyonD_ = Fractal(perm_, 5, 50.0f, 28.442329f, 0.3f, 0.3f, 2.0f);   // +0x10
    b5efc_ = Fractal(perm_, 2, 30.0f, 43.129738f, 0.6f, 0.6f, 0.5f);     // noise bank (p14 G3)
    b4f28_ = Fractal(perm_, 4, 50.0f, 29.409878f, 0.3f, 0.3f, 5.0f);
    b1fac_ = Fractal(perm_, 1, 10.0f, 35.173195f, 0.3f, 0.3f, 5.0f);
    bfd44_ = Fractal(perm_, 3, 100.0f, 17.975342f, 0.025f, 0.15f, 0.5f);
    b7ea4_ = Fractal(perm_, 4, 3000.0f, 9.647993f, 0.025f, 0.15f, 0.5f);
    b11cec_ = Fractal(perm_, 2, 10.0f, 33.129738f, 0.6f, 0.6f, 0.5f);
    b3f54_ = Fractal(perm_, 1, 10.0f, 25.148701f, 0.3f, 0.3f, 5.0f);

    // Flat surfaces (AGW_TerraFlat, GenWorld_Z 0x83-byte records; p13 V6, p21 H5).
    const uint32_t groups = rd32(b, p);
    p += 4;
    for (uint32_t g = 0; g < groups; ++g) {
        p += 4 + rd32(b, p) + 4;
        p += 4 + size_t(rd32(b, p)) * 4;
    }
    const uint32_t n = rd32(b, p);
    p += 4;
    if (p + size_t(n) * 0x83 > b.size()) throw std::runtime_error("flat surfaces overrun GenWorld_Z");
    for (uint32_t i = 0; i < n; ++i) {
        const size_t r = p + size_t(i) * 0x83;
        Flat s{};
        const double X[3] = {rdf(b, r + 4), rdf(b, r + 8), rdf(b, r + 12)};
        const double N[3] = {rdf(b, r + 20), rdf(b, r + 24), rdf(b, r + 28)};
        const double Z[3] = {rdf(b, r + 36), rdf(b, r + 40), rdf(b, r + 44)};
        s.cx = rdf(b, r + 52), s.cy = rdf(b, r + 56), s.cz = rdf(b, r + 60);
        auto ground = [](const double v[3], double& gx, double& gz, double& len) {
            len = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
            const double h = std::hypot(v[0], v[2]);
            gx = h > 0 ? v[0] * len / h : 0, gz = h > 0 ? v[2] * len / h : 0;
        };
        ground(X, s.ax, s.az, s.lenX);
        ground(Z, s.bx, s.bz, s.lenZ);
        const double nl = std::sqrt(N[0] * N[0] + N[1] * N[1] + N[2] * N[2]);
        if (s.lenX < 1e-3 || s.lenZ < 1e-3 || nl < 1e-6) continue;
        s.nx = N[0] / nl, s.nz = N[2] / nl;
        s.invMargin = 1.0f / std::max(rdf(b, r + 0x4c), 0.1f);
        s.strength = rdf(b, r + 0x50);
        s.priority = b[r + 0x7d];
        s.ellipse = b[r + 0x82] != 0;
        s.listB = b[r + 0x81] != 0;
        // +0x78 height map index and +0x7f noise flag: none in USA1 (all 0xffff / 0).
        const auto idx = uint32_t(flats_.size());
        flats_.push_back(s);
        const double rx = std::fabs(s.ax) + std::fabs(s.bx), rz = std::fabs(s.az) + std::fabs(s.bz);
        for (int cz = int(std::floor((s.cz - rz) / kCell)); cz <= int(std::floor((s.cz + rz) / kCell)); ++cz)
            for (int cx = int(std::floor((s.cx - rx) / kCell)); cx <= int(std::floor((s.cx + rx) / kCell)); ++cx)
                flatCells_[cellKey(cx, cz)].push_back(idx);
    }

    // Gulch lines: GwRoad_Z polylines with type byte exactly 0x0e (0x79fa80).
    const uint32_t roadId = rd32(b, 8);
    const DpcFile::Entry* re = world.find(roadId);
    const auto road = re ? world.body(*re) : std::nullopt;
    // Per-type road values, replayed from the player's FUEL.exe (road_types.hpp).
    std::string roadErr;
    if (auto rt = loadRoadTypes(world, &roadErr)) {
        roadTypes_ = *rt;
        haveRoadTypes_ = true;
    } else {
        std::fprintf(stderr, "fuel: %s; roads will not shape the ground\n", roadErr.c_str());
    }
    if (road) {
        const std::vector<uint8_t>& g = *road;
        const uint32_t count = rd32(g, 0);
        size_t q = 20;
        for (uint32_t i = 0; i < count && q + 3 <= g.size(); ++i) {
            const uint8_t type = g[q];
            const uint16_t np = uint16_t(g[q + 1] | (g[q + 2] << 8));
            q += 3;
            if (q + size_t(np) * 5 > g.size()) break;
            const uint8_t rtype = type & 0x7f;  // bit 7: built by 0x6f9150 (no XZ cleanup; port: same path)
            if (haveRoadTypes_ && roadBuiltType(rtype) && np >= 2) {
                RoadLine rl;
                rl.type = rtype;
                for (uint16_t k = 0; k < np; ++k) {
                    const size_t o = q + size_t(k) * 5;
                    const uint64_t v = (uint64_t(g[o]) << 32) | (uint64_t(g[o + 1]) << 24) | (uint64_t(g[o + 2]) << 16) |
                                       (uint64_t(g[o + 3]) << 8) | g[o + 4];
                    int32_t a = int32_t(v >> 20), c = int32_t(v & 0xFFFFF);
                    if (a & 0x80000) a -= 1 << 20;
                    if (c & 0x80000) c -= 1 << 20;
                    const std::array<double, 2> pt{a * 0.25, c * 0.25};
                    if (rl.pts.empty() || rl.pts.back() != pt) rl.pts.push_back(pt);
                }
                if (rl.pts.size() >= 2) {
                    const auto idx = uint32_t(roadLines_.size());
                    const double reach = roadTypes_[rtype].reach;
                    std::vector<uint64_t> keys;
                    for (size_t k = 0; k + 1 < rl.pts.size(); ++k) {
                        const double x0 = std::min(rl.pts[k][0], rl.pts[k + 1][0]) - reach, x1 = std::max(rl.pts[k][0], rl.pts[k + 1][0]) + reach;
                        const double z0 = std::min(rl.pts[k][1], rl.pts[k + 1][1]) - reach, z1 = std::max(rl.pts[k][1], rl.pts[k + 1][1]) + reach;
                        for (int cz = int(std::floor(z0 / kRoadCell)); cz <= int(std::floor(z1 / kRoadCell)); ++cz)
                            for (int cx = int(std::floor(x0 / kRoadCell)); cx <= int(std::floor(x1 / kRoadCell)); ++cx) keys.push_back(cellKey(cx, cz));
                    }
                    std::sort(keys.begin(), keys.end());
                    keys.erase(std::unique(keys.begin(), keys.end()), keys.end());
                    for (uint64_t key : keys) roadCells_[key].push_back(idx);
                    roadLines_.push_back(std::move(rl));
                }
            }
            if (type == 0x0e) {
                GulchLine line;
                for (uint16_t k = 0; k < np; ++k) {
                    const size_t o = q + size_t(k) * 5;
                    const uint64_t v = (uint64_t(g[o]) << 32) | (uint64_t(g[o + 1]) << 24) | (uint64_t(g[o + 2]) << 16) |
                                       (uint64_t(g[o + 3]) << 8) | g[o + 4];
                    int32_t a = int32_t(v >> 20), c = int32_t(v & 0xFFFFF);
                    if (a & 0x80000) a -= 1 << 20;
                    if (c & 0x80000) c -= 1 << 20;
                    const std::array<double, 2> pt{a * 0.25, c * 0.25};
                    if (line.pts.empty() || line.pts.back() != pt) line.pts.push_back(pt);  // 0x6d0990 drops repeats
                }
                if (line.pts.size() >= 2) {
                    const auto idx = uint32_t(gulchLines_.size());
                    const double maxR = std::sqrt(16.0 + 10.0 * double(line.pts.size()));
                    double x0 = 1e30, z0 = 1e30, x1 = -1e30, z1 = -1e30;
                    for (const auto& pt : line.pts) {
                        x0 = std::min(x0, pt[0]), x1 = std::max(x1, pt[0]);
                        z0 = std::min(z0, pt[1]), z1 = std::max(z1, pt[1]);
                    }
                    for (int cz = int(std::floor((z0 - maxR) / kCell)); cz <= int(std::floor((z1 + maxR) / kCell)); ++cz)
                        for (int cx = int(std::floor((x0 - maxR) / kCell)); cx <= int(std::floor((x1 + maxR) / kCell)); ++cx)
                            gulchCells_[cellKey(cx, cz)].push_back(idx);
                    gulchLines_.push_back(std::move(line));
                }
            }
            q += size_t(np) * 5;
        }
    }
    gulchOnce_ = std::make_unique<std::once_flag[]>(gulchLines_.size());
    roadOnce_ = std::make_unique<std::once_flag[]>(roadLines_.size());
    roadBuilt_.resize(roadLines_.size());
    gulchSegs_.resize(gulchLines_.size());
}

// ---------------------------------------------------------------------------

double TerrainDetail::baseWithNormal(double x, double z, double n[3]) const {
    double dx = 0, dz = 0;
    const double h = base_.value(x, z, &dx, &dz);
    // (local_e4, mpp, local_134) = (-dh/dpx, metres per pixel, -dh/dpz), normalised.
    const double mpp = 1.0 / double(base_.pxPerMetre);
    const double len = std::sqrt(dx * dx + mpp * mpp + dz * dz);
    n[0] = -dx / len, n[1] = mpp / len, n[2] = -dz / len;
    return h;
}

// Terrain_RockEval 0x7a4c20: returns the rock height L (unclamped).
double TerrainDetail::rockEval(double x, double z, double ny) const {
    const double m = map10_.value(x, z);
    double L = 0;
    const double A = double(t0c_.at(x, z)) + m;
    if (A > 0.5) {
        const double w1 = clamp01((std::min(A, 1.0) - 0.5) * 2);
        const double B = double(t18_.at(x, z)) * w1 + m;
        if (B > 0.5) {
            const double w2 = clamp01((B - 0.5) * 2);
            const double c = clamp01((double(t28_.at(x, z)) - 0.67) * 3.0);
            L = ((4 - c) * c * c + (4 - c * 6)) * c * 20.0 * w2;  // (1 - (1-c)^4) * 20 * w2
        }
    }
    const double s = clamp01((0.97 - ny) * 20.0);
    const double k = m * 10.0 + s * 3.0;
    if (k > 0.01) {
        const double q = m * 0.75 + 0.25;
        const double v = clamp01((1.0 / q) * (double(t24_.at(x, z)) - (1.0 - q)));
        L += S2(v * 2 - 1) * k;
    }
    return L;
}

// 0x7a69b0 near-camera path: bilinear over a 2 m lattice of 0x7a4c20 values, each with
// the base normal at its lattice point (0x45de10). Per-thread 128x128 cache like 0x7a6900.
double TerrainDetail::rockLattice(double x, double z) const {
    struct Entry {
        int32_t ix = INT32_MIN, iz = INT32_MIN;
        uint32_t owner = 0;
        double L = 0;
    };
    thread_local std::vector<Entry> cache(128 * 128);
    auto at = [&](int32_t ix, int32_t iz) {
        Entry& e = cache[size_t((uint32_t(iz) & 127) * 128 + (uint32_t(ix) & 127))];
        if (e.ix != ix || e.iz != iz || e.owner != id_) {
            double n[3];
            const double px = double(ix) * 2.0, pz = double(iz) * 2.0;
            baseWithNormal(px, pz, n);
            e.ix = ix, e.iz = iz, e.owner = id_;
            e.L = rockEval(px, pz, n[1]);
        }
        return e.L;
    };
    const double lx = x * 0.5, lz = z * 0.5;
    const double fx0 = std::floor(lx), fz0 = std::floor(lz);
    const auto ix = int32_t(fx0), iz = int32_t(fz0);
    const double fx = lx - fx0, fz = lz - fz0;
    const double l00 = at(ix, iz), l10 = at(ix + 1, iz), l01 = at(ix, iz + 1), l11 = at(ix + 1, iz + 1);
    return (l01 * (1 - fx) + l11 * fx) * fz + (l00 * (1 - fx) + l10 * fx) * (1 - fz);
}

// 0x7a0c80: canyon terraces at an already scaled position.
double TerrainDetail::canyon(double x, double z, double a, double b) const {
    const double A = 3.0 * fsin(x * double(0.002f)), B = 3.0 * fsin(z * double(0.002f));
    const double u = f32(double(0.707f) * x + double(0.707f) * z), v = f32(double(0.707f) * x - double(0.707f) * z);
    bool haveQ = false;
    double Q = 0;
    auto q = [&] {
        if (!haveQ) Q = double(0.15f) * S2(2 * double(canyonC_.at(x, z)) - 1), haveQ = true;
        return Q;
    };
    auto ridge = [&](float fv, float fu) {
        const double r = f32((fsin(v * double(fv) + A) + 1) * 0.5);
        const double t = f32(u * double(fu) + 0.75 * r + B);
        const double s1 = std::sin(t), s2 = fsin(t + t);
        double R = f32(((1 - std::fabs(s1)) - double(0.33f) * s2) * r);
        if (s2 > 0) R -= s2 * q();
        return R;
    };
    const double R1 = ridge(0.03517f, 0.01713f), R2 = ridge(0.02273f, 0.00975f);
    double M = (25.0 * R2 < 15.0 * R1) ? 15.0 * R1 : 25.0 * R2;
    M += f32(2.0 * fsin(x * double(0.05f)) * fsin(z * double(0.05f)));
    const double c = clamp01((double(canyonD_.at(x, z)) - double(0.9f)) * 8.0);
    return (M - double(0.33f) * c + 13.0) * double(0.02631579f) * b + a;
}

// 0x7a2310: canyon floor level from map slot 12.
double TerrainDetail::floorLevel(double x, double z, double h0) const {
    const double m = map12_.value(x, z);
    if (m < double(0.1f)) return h0 - 100.0;
    double r;
    if (m < double(0.2f)) {
        const double s = (m - double(0.1f)) * 10.0 * 2.0 - 1.0;
        r = canyon(4 * x, 4 * z, S2(s) * 110.0 - 100.0, 0.0);
    } else if (m < 0.5) {
        const double s = (m - double(0.2f)) * double(3.3333333f) * 2.0 - 1.0;
        r = canyon(4 * x, 4 * z, 10.0, 10.0 * S2(s));
    } else if (m < 0.75) {
        const double s = (m - 0.5) * 4.0 * 2.0 - 1.0, qv = S2(s);
        r = canyon(4 * x, 4 * z, 0.0, 10.0 - 10.0 * qv) + canyon(0.5 * x, 0.5 * z, 10.0, 50.0 * qv);
    } else {
        const double s = (m - 0.75) * 4.0 * 2.0 - 1.0;
        r = canyon(0.5 * x, 0.5 * z, 10.0, (S2(s) + 1.0) * 0.5 * 100.0);
    }
    return h0 + r;
}

// 0x64a2a0 up to the finishing pass (flags 3).
double TerrainDetail::detailOnly(double x, double z, GroundSample* out) const {
    double n[3];
    const double h0 = baseWithNormal(x, z, n);
    const double a1 = (clamp01(2 * (0.9 - n[1])) + 0.1) * 8.0;
    const double a2 = (clamp01(5 * (0.9 - n[1])) + 0.1) * 3.0;
    auto curve = [](double f) {
        const double t = f * 2 - 1;
        return (2 - std::fabs(t)) * t * 0.5;  // S2(t) - 0.5
    };
    const double h2 = (skip_ & 16) ? h0 : h0 + curve(f1_.at(x, z)) * a1 + curve(f2_.at(x, z)) * a2;
    const double R = (skip_ & 8) ? 0.0 : rockLattice(x, z);
    const double H = h2 + R;
    const double v = floorLevel(x, z, h0);
    const double T = (H > v + double(0.01f)) ? 0.0 : (H > v ? 0.0 : v - H);
    const double w = clamp01(map18_.value(x, z));
    const double h = (1 - w) * (T + H) + w * h0;
    if (out) {
        out->h0 = h0, out->nx = n[0], out->ny = n[1], out->nz = n[2];
        out->h2 = h2, out->rock = R, out->floor = v, out->w18 = w, out->detail = h;
    }
    return h;
}

// Gulch segments of one GwRoad type-14 line, built on first use (0x79fa80 / 0x6f10c0).
const std::vector<TerrainDetail::GulchSeg>& TerrainDetail::gulchSegments(uint32_t line) const {
    std::call_once(gulchOnce_[line], [&] {
        const auto& pts = gulchLines_[line].pts;
        std::vector<double> h(pts.size());
        for (size_t i = 0; i < pts.size(); ++i) h[i] = detailOnly(pts[i][0], pts[i][1], nullptr);
        std::vector<GulchSeg> segs;
        double R = 4.0;
        for (size_t i = 0; i + 1 < pts.size(); ++i) {
            if (!(h[i] < 2000.0 && h[i + 1] < 2000.0)) continue;
            GulchSeg s{};
            s.ax = pts[i][0], s.az = pts[i][1], s.bx = pts[i + 1][0], s.bz = pts[i + 1][1];
            const double dx = s.bx - s.ax, dz = s.bz - s.az, len = std::hypot(dx, dz);
            if (len <= 0) continue;
            s.ux = dx / len, s.uz = dz / len, s.invLen = 1.0 / len;
            s.R = R;
            s.minX = std::min(s.ax, s.bx) - R, s.maxX = std::max(s.ax, s.bx) + R;
            s.minZ = std::min(s.az, s.bz) - R, s.maxZ = std::max(s.az, s.bz) + R;
            segs.push_back(s);
            R = std::sqrt(R * R + 10.0);
        }
        gulchSegs_[line] = std::move(segs);
    });
    return gulchSegs_[line];
}

// AGW_Gulches 0x7a00e0: depth of the river bed at (x, z), 0 outside.
double TerrainDetail::gulchDepth(double x, double z) const {
    const auto it = gulchCells_.find(cellKey(int32_t(std::floor(x / kCell)), int32_t(std::floor(z / kCell))));
    if (it == gulchCells_.end()) return 0;
    bool have = false;
    double N = 0, B = 0, C = 0, E = 0, M = 0;
    auto P8 = [](double t) { return t <= 0 ? std::pow(t + 1, 8) - 1 : 1 - std::pow(1 - t, 8); };
    auto Q4 = [](double t) { return 0.5 + 0.5 * t * (4 - 6 * std::fabs(t) + (4 - std::fabs(t)) * t * t); };
    for (uint32_t line : it->second) {
        for (const GulchSeg& s : gulchSegments(line)) {
            if (x < s.minX || x > s.maxX || z < s.minZ || z > s.maxZ) continue;
            const double t = ((x - s.ax) * s.ux + (z - s.az) * s.uz) * s.invLen;
            double D;
            if (t <= 0) D = std::hypot(x - s.ax, z - s.az);
            else if (t > 1) D = std::hypot(x - s.bx, z - s.bz);
            else D = std::fabs(s.uz * (x - s.ax) - s.ux * (z - s.az));
            if (!(D < s.R)) continue;
            if (!have) {
                N = b5efc_.at(x, z);
                B = 0.66 * double(b4f28_.at(x, z)) + 0.34 * double(b1fac_.at(x, z));
                C = bfd44_.at(x, z);
                const double v = b7ea4_.at(x, z);
                E = 0.5 + 0.5 * P8(P8(2 * v * std::fabs(v) - 1));
                have = true;
            }
            const double q = clamp01((7.0 - s.R) * double(0.2f));
            const double vv = clamp01(E - q + C - B);
            const double u = clamp01((B - double(0.66f)) * 5.0);
            const double G = Q4(2 * clamp01(vv - u) - 1);
            const double T = clamp01(double(0.66f) * (0.75 * B + 0.25) + double(0.34f) * (1 - G));
            const double r = T * s.R;
            if (D < r) {
                const double f = 1 - D * (1.0 / r);
                const double f4 = 1 - std::pow(1 - f, 4);
                const double d = (double(0.66f) * G * f4 + double(0.33f) * (1 - G) * S2(2 * f - 1)) * (double(1.25f) * (E - q) + 1) * r *
                                 (0.25 * N + 0.75);
                M = std::max(M, d);
            }
        }
    }
    return M;
}

// FlatSurface_CombineHeight 0x79b870 over list A or B; edits y, returns min(d, 1) of the
// last covering surface (0 when none).
double TerrainDetail::combineFlats(bool listB, double x, double z, double& y) const {
    const auto it = flatCells_.find(cellKey(int32_t(std::floor(x / kCell)), int32_t(std::floor(z / kCell))));
    if (it == flatCells_.end()) return 0;
    struct Hit {
        const Flat* s;
        double d;
    };
    Hit hits[64];
    size_t nh = 0;
    double jitter = 0;
    bool haveJitter = false;
    for (uint32_t i : it->second) {
        const Flat& s = flats_[i];
        if (s.listB != listB) continue;
        const double dx = x - s.cx, dz = z - s.cz;
        // The engine lists surfaces whose bounding box holds the point (2.442 m cells).
        if (std::fabs(dx) > std::fabs(s.ax) + std::fabs(s.bx) + 2.5 || std::fabs(dz) > std::fabs(s.az) + std::fabs(s.bz) + 2.5) continue;
        const double u = (dx * s.ax + dz * s.az) / (s.lenX * s.lenX), v = (dx * s.bx + dz * s.bz) / (s.lenZ * s.lenZ);
        if (!haveJitter) jitter = 5.0 * 0.5 * (double(b11cec_.at(x, z)) + double(b5efc_.at(x, z))), haveJitter = true;
        double d;
        if (s.ellipse) {
            const double rr = std::hypot(u, v);
            if (rr == 0) {
                d = std::min(s.lenX, s.lenZ);
            } else {
                const double ex = s.cx + s.ax * (u / rr) + s.bx * (v / rr) - x, ez = s.cz + s.az * (u / rr) + s.bz * (v / rr) - z;
                d = std::hypot(ex, ez);
                if (rr > 1) d = -d;
            }
        } else {
            const double eX = std::max(0.0, 1 - std::fabs(u)) * s.lenX, eZ = std::max(0.0, 1 - std::fabs(v)) * s.lenZ;
            d = std::min(eX, eZ);
            if (v < 0) d -= std::max(0.0, 15.0 - std::hypot(eX, eZ));  // the -Z corners are rounded off
        }
        d -= jitter;
        if (d > 0 && nh < 64) hits[nh++] = {&s, d};
    }
    if (nh == 0) return 0;
    std::sort(hits, hits + nh, [](const Hit& a, const Hit& c) {
        return a.s->priority != c.s->priority ? a.s->priority < c.s->priority : a.s->cx < c.s->cx;
    });
    double ret = 0, acc = 0, wsum = 0;
    int group = 0;
    for (size_t k = 0; k < nh; ++k) {
        const Flat& s = *hits[k].s;
        if (int(s.priority) != group) {
            group = s.priority;
            if (wsum > double(0.01f)) y = acc / wsum;
            acc = wsum = 0;
        }
        const double d = hits[k].d;
        ret = std::min(d, 1.0);
        const double P = s.cy - (s.nx * (x - s.cx) + s.nz * (z - s.cz));
        const double t = clamp01(d * s.invMargin), e = 2 * t - 1;
        const double cv = e <= 0 ? std::pow(e + 1, 1.25) - 1 : 1 - std::pow(1 - e, 1.25);
        const double wt = (0.5 + 0.5 * cv) * s.strength;
        acc += wt * ((1 - wt) * y + wt * P);
        wsum += wt;
    }
    if (wsum > double(0.01f)) y = acc / wsum;
    return ret;
}

GroundSample TerrainDetail::sample(double x, double z) const {
    GroundSample s;
    double y = detailOnly(x, z, &s);
    // 0x64d550 (flags 0).
    const double g = (skip_ & 4) ? 0.0 : gulchDepth(x, z);
    if (g > 0) y -= g;
    s.gulch = y;
    const double a = (g > 0 ? 1.0 : 0.0) + ((skip_ & 2) ? 0.0 : combineFlats(false, x, z, y));
    s.flatA = y;
    const double c = clamp01(a);
    double r = 0;
    if (!(skip_ & 1)) r = roads_ ? roads_(x, z, y, 1 - c, 1 - s.w18) : roadCarve(x, z, y, 1 - c, 1 - s.w18);
    s.road = y;
    if (!(skip_ & 2)) combineFlats(true, x, z, y);
    s.flatB = y;
    // Work bumps (0x79cfa0) exist only inside procedural mode-3 TerraFlat surfaces (not ported).
    // High ground (snow) lift: 0x4de850 at (x, h0, z) with the base normal.
    const double hg0 = clamp01(s.h0 - (2500.0 - (-s.nz) * 250.0));
    const double hb = clamp01(hg0 - (1 - s.ny) * 2.0);
    const double t = hb * 2 - 1;
    const double hg = t <= 0 ? 0.5 + (std::pow(t + 1, 8) - 1) * 0.5 : 0.5 + (1 - std::pow(1 - t, 8)) * 0.5;
    y += std::max(0.0, double(0.3f) * hg - r);
    s.ground = y;
    return s;
}

size_t TerrainDetail::flatSurfaceCount() const { return flats_.size(); }
size_t TerrainDetail::gulchLineCount() const { return gulchLines_.size(); }
size_t TerrainDetail::roadLineCount() const { return roadLines_.size(); }

// Road height profile of one polyline (0x6fa8a0 build: 0x6f8a00 samples, 0x6f81f0 end
// blend, 0x6f85c0 smoothing, 0x6f8900/0x6f8850 undulation, 0x6f16e0 cm storage). The port
// builds each GwRoad polyline on its own (no chaining 0x6f66f0, no XZ cleanup 0x6f8f30,
// junction heights = the ground at the node): see p21 "Approximations".
const TerrainDetail::RoadBuilt& TerrainDetail::roadBuilt(uint32_t line) const {
    std::call_once(roadOnce_[line], [&] {
        auto out = std::make_unique<RoadBuilt>();
        const RoadLine& rl = roadLines_[line];
        const RoadTypeParams& T = roadTypes_[rl.type];
        const auto& pts = rl.pts;
        std::vector<double> cum(pts.size(), 0.0);
        for (size_t i = 1; i < pts.size(); ++i) cum[i] = cum[i - 1] + std::hypot(pts[i][0] - pts[i - 1][0], pts[i][1] - pts[i - 1][1]);
        const double L = cum.back();
        if (L <= 0) {
            roadBuilt_[line] = std::move(out);
            return;
        }
        auto at = [&](double s, double& px, double& pz) {  // point at chain distance s (straight beyond the ends)
            size_t i = 0;
            if (s <= 0) i = 0;
            else if (s >= L) i = pts.size() - 2;
            else i = size_t(std::upper_bound(cum.begin(), cum.end(), s) - cum.begin()) - 1;
            i = std::min(i, pts.size() - 2);
            const double segLen = cum[i + 1] - cum[i];
            const double t = segLen > 0 ? (s - cum[i]) / segLen : 0;
            px = pts[i][0] + (pts[i + 1][0] - pts[i][0]) * t, pz = pts[i][1] + (pts[i + 1][1] - pts[i][1]) * t;
        };
        const double M = std::min(std::max(double(T.smooth), double(T.smoothMin)), L / 4);
        const int K = int(std::floor(M / 2));
        // Raw heights every 2 m (0x64a2a0 flags 3), from s = -2K to L + 2K + 2.
        const int n = int(std::floor(L / 2)) + 2;
        std::vector<double> raw(size_t(n + 4 * K + 2));
        for (size_t k = 0; k < raw.size(); ++k) {
            double px, pz;
            at(2.0 * (double(k) - 2 * K), px, pz);
            raw[k] = detailOnly(px, pz, nullptr);
        }
        // Ends blend to the ground at the node (0x6f81f0 with junction heights = the node's ground).
        double ex, ez;
        at(0, ex, ez);
        const double hs = detailOnly(ex, ez, nullptr);
        at(L, ex, ez);
        const double he = detailOnly(ex, ez, nullptr);
        const double ls = std::min(double(T.flat) + std::min(double(T.flat), 1.0) * M, 0.33 * L);
        const double Rl = std::max(std::min(std::max(M, 50.0), L - 2 * ls), 0.001);
        // Smoothing kernel (2 - u) u, u = (K + 1 - |K - j|)/(K + 1) (0x6f85c0).
        std::vector<double> wk(size_t(2 * K + 1));
        double wsum = 0;
        for (int j = 0; j <= 2 * K; ++j) {
            const double u = double(K + 1 - std::abs(K - j)) / double(K + 1);
            wk[size_t(j)] = (2 - u) * u;
            wsum += wk[size_t(j)];
        }
        out->profile.resize(size_t(n + 1));
        for (int k = 0; k <= n; ++k) {
            const double sk = 2.0 * k;
            double h = 0;
            for (int j = 0; j <= 2 * K; ++j) {
                const int src = k + j;  // raw index of s = 2(k + j - 2K) ... centred at 2k - 2K? keep centred:
                h += wk[size_t(j)] * raw[size_t(std::clamp(src + K, 0, int(raw.size()) - 1))];
            }
            h /= wsum;
            const double r1 = S2(2 * clamp01(1 - (sk - ls) / Rl) - 1), r2 = S2(2 * clamp01(1 - ((L - sk) - ls) / Rl) - 1);
            h = r1 * hs + (1 - r1) * h;
            h = (1 - r2) * h + r2 * he;
            if (T.waveAmp != 0 && T.waveLen != 0) {
                const double a = 6.28 * sk / double(T.waveLen);
                h += 0.33 * 0.5 * double(T.waveAmp) * (std::sin(a) + std::sin(0.390933 * a) + std::sin(1.839735 * a));
            }
            if (T.bumpAmp != 0 && T.bumpLen != 0) h += double(T.bumpAmp) * std::pow((std::sin(0.390933 * 6.28 * sk / double(T.bumpLen)) + 1) / 2, 4);
            out->profile[size_t(k)] = h;
        }
        const double base = std::floor(out->profile[0]);
        for (double& h : out->profile) h = base + 0.01 * std::floor((h - base) * 100 + 0.5);  // cm shorts
        for (size_t i = 0; i + 1 < pts.size(); ++i) {
            RoadBuilt::Seg sg{};
            sg.ax = pts[i][0], sg.az = pts[i][1], sg.bx = pts[i + 1][0], sg.bz = pts[i + 1][1];
            sg.len = cum[i + 1] - cum[i];
            if (sg.len <= 0) continue;
            sg.ux = (sg.bx - sg.ax) / sg.len, sg.uz = (sg.bz - sg.az) / sg.len, sg.invLen = 1 / sg.len;
            sg.start = cum[i];
            sg.minX = std::min(sg.ax, sg.bx) - T.reach, sg.maxX = std::max(sg.ax, sg.bx) + T.reach;
            sg.minZ = std::min(sg.az, sg.bz) - T.reach, sg.maxZ = std::max(sg.az, sg.bz) + T.reach;
            out->segs.push_back(sg);
        }
        roadBuilt_[line] = std::move(out);
    });
    return *roadBuilt_[line];
}

// 0x6fcaa0: roads level the ground to their smoothed profile (p21 H6). Returns the on-road
// coverage (feeds the snow term only).
double TerrainDetail::roadCarve(double x, double z, double& y, double omc, double omw) const {
    const auto it = roadCells_.find(cellKey(int32_t(std::floor(x / kRoadCell)), int32_t(std::floor(z / kRoadCell))));
    if (it == roadCells_.end()) return 0;
    double sumWH = 0, sumW = 0, blend = 0, cov = 0, bump = 0, minLn = 1000;
    const RoadTypeParams* best = nullptr;
    bool haveF = false, haveF2 = false;
    double F = 0, F2 = 0;
    int used = 0;
    for (uint32_t line : it->second) {
        const RoadTypeParams& T = roadTypes_[roadLines_[line].type];
        const RoadBuilt& rb = roadBuilt(line);
        for (const RoadBuilt::Seg& S : rb.segs) {
            if (x < S.minX || x > S.maxX || z < S.minZ || z > S.maxZ) continue;
            if (used >= 128) break;  // the engine's per-query cap
            const double d = S.uz * (x - S.ax) - S.ux * (z - S.az);
            const double t = ((x - S.ax) * S.ux + (z - S.az) * S.uz) * S.invLen;
            double dist;
            if (t <= 0) dist = std::hypot(x - S.ax, z - S.az);
            else if (t > 1) dist = std::hypot(x - S.bx, z - S.bz);
            else dist = std::fabs(d);
            if (dist > T.reach) continue;
            ++used;
            const double tc = clamp01(t);
            const double offs = 0;  // bend m0/m1 (0x6fa7a0): not built by the port
            const double sd = (t <= 0 || t > 1) ? (d < 0 ? -dist : dist) : d;
            const double lat = std::fabs(sd + offs);
            const double wid = 1.0;  // S.52/S.53 * 0.02 without junction widening or cross slope
            double ln = lat / wid;
            if (ln > T.stretch) {
                if (!haveF) F = b11cec_.at(x, z), haveF = true;
                ln = (ln - T.stretch) / ((1.25 - 0.75) * F + 0.75) + T.stretch;
            }
            const double nW = T.band / wid, e = nW - ln;
            if (e <= 0) continue;
            const double wgt = S2(2 * clamp01(e * (1.0 / T.band) * wid) - 1) * nW;
            if (!best || best->band < T.band) best = &T;
            minLn = std::min(minLn, ln);
            // h_seg (0x6f1800): profile shorts of this segment, N = len * 0.5.
            const double N = S.len * 0.5;
            const double ti = tc * N;
            const double fi = std::floor(ti);
            const size_t g0 = size_t(std::floor(S.start / 2));
            auto prof = [&](size_t k) { return rb.profile[std::min(k, rb.profile.size() - 1)]; };
            const double h0 = prof(g0 + size_t(fi)), h1 = prof(g0 + size_t(fi) + 1);
            const double ext = t < 0 ? t * N : (t > 1 ? (t - 1) * N : 0.0);
            const double hseg = h0 + (h1 - h0) * (ti - fi) + (h1 - h0) * ext;
            if (!haveF2) F2 = b3f54_.at(x, z), haveF2 = true;
            const double target = hseg - 1.5 * omw - F2 * T.noise;  // + (offs + d) * cross slope * T8c: no cross slope
            double b = 0;
            const double q1 = T.flat - 0.5 - lat;
            if (q1 > 0) b = (2 - q1 / T.flat) * (q1 / T.flat) * T.flat * T.crown;
            if (T.flat + 1 - lat > 0) b += clamp01((T.flat + 1 - lat) / 2) * best->verge;
            bump = std::max(bump, b);
            const double cv = ln < T.core - 1.5 ? 1.0 : (ln > nW - 0.5 * T.fade ? 0.0 : clamp01((nW - ln - T.fade) * 2.0 / 3.0));
            cov = std::max(cov, cv);
            sumWH += target * wgt;
            sumW += wgt;
            blend = std::max(blend, S2(2 * clamp01(e / T.fade) - 1));
        }
    }
    if (sumW <= 1e-7) return 0;
    double H = sumWH / sumW;
    const double g = best->core - minLn;
    if (g > 0) {
        double dt = 0;
        if (g < best->ditch) {
            const double r = g / best->ditch, v = r <= 0.5 ? 1 - 4 * r : 4 * r - 3;
            const double sv = S2(v);
            dt = -(1 - sv * sv) * best->ditchDepth;
        }
        H += std::min(0.0, clamp01(H - y - 0.5) + dt * omc);
    }
    H += bump;
    y = blend * H + (1 - blend) * y;
    return cov;
}

double TerrainDetail::baseNormal(double x, double z, float n[3]) const {
    double d[3];
    const double h = baseWithNormal(x, z, d);
    n[0] = float(d[0]), n[1] = float(d[1]), n[2] = float(d[2]);
    return h;
}

double TerrainDetail::farHeight(double x, double z, float n[3]) const {
    const double h0 = baseNormal(x, z, n);
    const double v = floorLevel(x, z, h0);
    return h0 + ((h0 > v + double(0.01f)) ? 0.0 : (h0 > v ? 0.0 : v - h0));
}

double TerrainDetail::height(double x, double z) const { return sample(x, z).ground; }

double TerrainDetail::ground(double x, double z) const {
    struct Entry {
        int32_t ix = INT32_MIN, iz = INT32_MIN;
        uint32_t owner = 0;
        float h = 0;
    };
    thread_local std::vector<Entry> cache(1 << 16);
    auto at = [&](int32_t ix, int32_t iz) {
        Entry& e = cache[size_t(((uint32_t(iz) & 255) << 8) | (uint32_t(ix) & 255))];
        if (e.ix != ix || e.iz != iz || e.owner != id_) e.ix = ix, e.iz = iz, e.owner = id_, e.h = float(height(ix, iz));
        return double(e.h);
    };
    const double fx0 = std::floor(x), fz0 = std::floor(z);
    const auto ix = int32_t(fx0), iz = int32_t(fz0);
    const double u = x - fx0, v = z - fz0;
    return (1 - v) * ((1 - u) * at(ix, iz) + u * at(ix + 1, iz)) + v * ((1 - u) * at(ix, iz + 1) + u * at(ix + 1, iz + 1));
}
double TerrainDetail::detailHeight(double x, double z) const { return detailOnly(x, z, nullptr); }

void TerrainDetail::normal(double x, double z, double e, float out[3]) const {
    const double dx = (height(x + e, z) - height(x - e, z)) / (2 * e), dz = (height(x, z + e) - height(x, z - e)) / (2 * e);
    const double l = std::sqrt(dx * dx + 1 + dz * dz);
    out[0] = float(-dx / l), out[1] = float(1 / l), out[2] = float(-dz / l);
}

std::shared_ptr<TerrainDetail> makeTerrainDetail(const DpcFile& world, uint32_t width, uint32_t height, float maxValue,
                                                 const std::vector<uint16_t>& baseRaw) {
    MapSampler base;
    base.width = width;
    base.height = height;
    base.pxPerMetre = float(width) * 7.142857e-06f;
    base.scale = 10000.0f / maxValue;  // [0xa2a4e0] / maxValue
    base.raw = baseRaw;
    return std::make_shared<TerrainDetail>(world, std::move(base));
}

}  // namespace fuel
