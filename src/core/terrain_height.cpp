#include "terrain_height.hpp"
#include "terrain.hpp"
#include "road_types.hpp"
#include "terra_cells.hpp"

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
    // Procedural surfaces (0x799910): +0xa6 dirt bumps on the plane, +0xa2 mode, +0xa9 = 1.
    bool bumps = false;
    bool procedural = false;
    uint8_t mode = 0;
    CellBox box;  // u16 cell box (SetShape 0x79c330: centre and the four corners)
};

struct TerrainDetail::FlatCell {
    std::once_flag once;
    std::vector<Flat> flats;
    // Port index: every procedural flat (of this and the 8 neighbouring cells) whose box
    // overlaps this cell, built on first use.
    std::once_flag coverOnce;
    std::vector<const Flat*> cover;
};

struct TerrainDetail::BumpSeg {
    double ax, az, bx, bz, dx, dz, tx, tz, c30, c34, R;
    CellBox box;
    uint8_t mode;  // +0x50: 3 or 1 (the ground's zone byte, not the height)
};

struct TerrainDetail::BumpCell {
    std::once_flag once;
    std::vector<BumpSeg> segs;
    std::once_flag coverOnce;
    std::vector<const BumpSeg*> cover;  // bumps of this and the 8 neighbouring cells reaching it
};

struct TerrainDetail::GulchLine {
    std::vector<std::array<double, 2>> pts;
};

struct TerrainDetail::GulchSeg {
    double ax, az, bx, bz;
    double ux, uz, invLen;  // unit direction, 1/length
    double R;
    double minX, minZ, maxX, maxZ;
    CellBox box;  // 0x6f0ef0
    double hA, hB;  // +0x44 / +0x48: 0x64a2a0 flags 3 at the ends
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

TerrainDetail::~TerrainDetail() {
    if (flatCells2_)
        for (int32_t i = 0; i < kFlatCells * kFlatCells; ++i) delete flatCells2_[size_t(i)].load();
    if (bumpCells_)
        for (int32_t i = 0; i < kBumpCells * kBumpCells; ++i) delete bumpCells_[size_t(i)].load();
}

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
    loadMap(8, map08_);
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
        s.box = terra::boxAt(s.cx, s.cz);
        for (int k = 0; k < 4; ++k) {
            const double sx = (k & 1) ? -1 : 1, sz = (k & 2) ? -1 : 1;
            terra::extend(s.box, s.cx + sx * s.ax + sz * s.bx, s.cz + sx * s.az + sz * s.bz);
        }
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
    flatCells2_.reset(new std::atomic<FlatCell*>[size_t(kFlatCells) * kFlatCells]());
    bumpCells_.reset(new std::atomic<BumpCell*>[size_t(kBumpCells) * kBumpCells]());
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
            s.box = terra::segmentBox(s.ax, s.az, s.bx, s.bz, s.ux, s.uz, R);
            s.hA = terra::f32down(h[i]), s.hB = terra::f32down(h[i + 1]);
            segs.push_back(s);
            R = std::sqrt(R * R + 10.0);
        }
        gulchSegs_[line] = std::move(segs);
    });
    return gulchSegs_[line];
}

// AGW_Gulches 0x7a00e0: depth of the river bed at (x, z), 0 outside.
double TerrainDetail::gulchDepth(double x, double z) const { return gulchProfile(x, z, nullptr, nullptr); }

// 0x7a00e0 with its water outputs (water.md 1.3): sh / sw = sum over segments with D < r of
// (lerp(hA, hB, t unclamped) - 1.5)(r - D) and (r - D); the river surface is sh / sw.
double TerrainDetail::gulchProfile(double x, double z, double* shOut, double* swOut) const {
    if (shOut) *shOut = 0;
    if (swOut) *swOut = 0;
    const auto it = gulchCells_.find(cellKey(int32_t(std::floor(x / kCell)), int32_t(std::floor(z / kCell))));
    if (it == gulchCells_.end()) return 0;
    double Sh = 0, Sw = 0;
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
                // 0x79dae0: float32 constants 0.66f / 0.34f (.rdata 0x9dc93c / 0x9dc940), result stored as float.
                B = double(0.66f * b4f28_.at(x, z) + 0.34f * b1fac_.at(x, z));
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
                Sh += ((1 - t) * s.hA + t * s.hB - 1.5) * (r - D);
                Sw += r - D;
            }
        }
    }
    if (shOut) *shOut = Sh;
    if (swOut) *swOut = Sw;
    return M;
}

// Gulches_WaterQuery 0x7a07d0: river carve depth D (0 = no river) and the bed level output
// A = max(0.5, min(D, y + D - sh / sw)) of 0x7a00e0 (y = the finished ground height).
TerrainDetail::River TerrainDetail::river(double x, double z, double y) const {
    River r;
    if (skip_ & 4) return r;
    double sh = 0, sw = 0;
    const double D = gulchProfile(x, z, &sh, &sw);
    if (!(D >= double(0.001f))) return r;
    r.depth = D;
    r.flag = true;
    if (sw > 0) {
        const double a = std::min(y + D - sh / sw, D);
        r.bed = a <= 0.5 ? 0.5 : a;
    }
    return r;
}

// 0x6ef6f0: J = 0.5 (F 0x11cec + F 0x5efc); the flat-surface jitter is 5 J.
double TerrainDetail::jitterJ(double x, double z) const { return 0.5 * (double(b11cec_.at(x, z)) + double(b5efc_.at(x, z))); }

// FlatSurface_InsideDistance 0x799020 (p31 out-C sec. 3): distance inside the rectangle or
// ellipse minus `jitter`; rectangles lose their -Z corners (v < 0) by up to 15 m.
double TerrainDetail::insideDistance(const Flat& s, double x, double z, double jitter) const {
    const double dx = x - s.cx, dz = z - s.cz;
    const double u = (dx * s.ax + dz * s.az) / (s.lenX * s.lenX), v = (dx * s.bx + dz * s.bz) / (s.lenZ * s.lenZ);
    double d;
    if (s.ellipse) {
        const double rr = std::hypot(u, v);
        if (rr == 0) return std::hypot(s.lenX, s.lenZ);  // +0x70, no jitter (0x799020 r == 0 branch)
        const double ex = s.cx + s.ax * (u / rr) + s.bx * (v / rr) - x, ez = s.cz + s.az * (u / rr) + s.bz * (v / rr) - z;
        d = std::hypot(ex, ez);
        if (rr > 1) d = -d;
    } else {
        const double eX = std::max(0.0, 1 - std::fabs(u)) * s.lenX, eZ = std::max(0.0, 1 - std::fabs(v)) * s.lenZ;
        d = std::min(eX, eZ);
        if (v < 0) d -= std::max(0.0, 15.0 - std::hypot(eX, eZ));
    }
    return d - jitter;
}

// FlatSurface_CombineHeight 0x79b870 over list A or B; edits y, returns min(d, 1) of the
// last covering surface (0 when none). List A (0x79aa40) also holds the procedural surfaces.
double TerrainDetail::combineFlats(bool listB, double x, double z, double& y) const {
    struct Hit {
        const Flat* s;
        double d;
    };
    Hit hits[64];
    size_t nh = 0;
    double jitter = 0;
    bool haveJitter = false;
    auto consider = [&](const Flat& s) {
        if (!haveJitter) jitter = 5.0 * jitterJ(x, z), haveJitter = true;
        const double d = insideDistance(s, x, z, jitter);
        if (d > 0 && nh < 64) hits[nh++] = {&s, d};
    };
    const auto it = flatCells_.find(cellKey(int32_t(std::floor(x / kCell)), int32_t(std::floor(z / kCell))));
    if (it != flatCells_.end())
        for (uint32_t i : it->second) {
            const Flat& s = flats_[i];
            if (s.listB != listB) continue;
            // The engine lists surfaces whose u16 cell box holds the point (2.442 m cells).
            const double dx = x - s.cx, dz = z - s.cz;
            if (std::fabs(dx) > std::fabs(s.ax) + std::fabs(s.bx) + 2.5 || std::fabs(dz) > std::fabs(s.az) + std::fabs(s.bz) + 2.5) continue;
            consider(s);
        }
    if (!listB && !(skip_ & 32) && bigRoad_)
        for (const Flat* f : flatsCovering(x, z))
            if (terra::holds(f->box, x, z)) consider(*f);
    if (nh == 0) return 0;
    std::sort(hits, hits + nh, [](const Hit& a, const Hit& c) {
        return a.s->priority != c.s->priority ? a.s->priority < c.s->priority : a.s->cx < c.s->cx;
    });
    double ret = 0, acc = 0, wsum = 0;
    int group = 0;
    double bumps = -1;
    for (size_t k = 0; k < nh; ++k) {
        const Flat& s = *hits[k].s;
        if (int(s.priority) != group) {
            group = s.priority;
            if (wsum > double(0.01f)) y = acc / wsum;
            acc = wsum = 0;
        }
        const double d = hits[k].d;
        ret = std::min(d, 1.0);
        double P = s.cy - (s.nx * (x - s.cx) + s.nz * (z - s.cz));
        const double t = clamp01(d * s.invMargin), e = 2 * t - 1;
        const double cv = e <= 0 ? std::pow(e + 1, 1.25) - 1 : 1 - std::pow(1 - e, 1.25);
        const double wt = (0.5 + 0.5 * cv) * s.strength;
        if (s.bumps) {  // +0xa6: 0.1 x DirtBumps 0x6f0430 on the plane
            if (bumps < 0) {
                const double v = clamp01((double(b3f54_.at(x, z)) - 1.1) * 10.0);
                bumps = (2 - v) * v;
            }
            P += 0.1 * bumps;
        }
        acc += wt * ((1 - wt) * y + wt * P);
        wsum += wt;
    }
    if (wsum > double(0.01f)) y = acc / wsum;
    return ret;
}

const std::vector<const TerrainDetail::Flat*>& TerrainDetail::flatsCovering(double x, double z) const {
    static const std::vector<const Flat*> none;
    const int32_t cx = int32_t(std::floor(x / 450.0)), cz = int32_t(std::floor(z / 450.0));
    FlatCell* c = const_cast<FlatCell*>(flatCell(cx, cz));
    if (!c) return none;
    std::call_once(c->coverOnce, [&] {
        const double x0 = cx * 450.0, z0 = cz * 450.0;
        const CellBox cb{terra::q(x0), terra::q(z0), terra::q(x0 + 450), terra::q(z0 + 450)};
        for (int32_t i = cx - 1; i <= cx + 1; ++i)
            for (int32_t j = cz - 1; j <= cz + 1; ++j)
                if (const FlatCell* n = flatCell(i, j))
                    for (const Flat& f : n->flats)
                        if (f.box.minX <= cb.maxX && cb.minX <= f.box.maxX && f.box.minZ <= cb.maxZ && cb.minZ <= f.box.maxZ) c->cover.push_back(&f);
    });
    return c->cover;
}

void TerrainDetail::setRoadQueries(BigRoadQuery big, NearRoadQuery near) {
    bigRoad_ = std::move(big);
    nearRoad_ = std::move(near);
}

size_t TerrainDetail::proceduralFlatCount() const { return procCount_.load(); }
std::vector<TerrainDetail::ProcFlatInfo> TerrainDetail::proceduralFlats(double x, double z, double r) const {
    std::vector<ProcFlatInfo> out;
    for (int32_t i = int32_t(std::floor((x - r - 450) / 450.0)); i <= int32_t(std::floor((x + r + 450) / 450.0)); ++i)
        for (int32_t j = int32_t(std::floor((z - r - 450) / 450.0)); j <= int32_t(std::floor((z + r + 450) / 450.0)); ++j)
            if (const FlatCell* c = flatCell(i, j))
                for (const Flat& f : c->flats)
                    if (std::hypot(f.cx - x, f.cz - z) <= r)
                        out.push_back({f.cx, f.cy, f.cz, f.lenX, f.lenZ, f.ax / f.lenX, f.az / f.lenX, f.nx, f.nz, 1.0 / f.invMargin, f.strength, f.mode});
    return out;
}

void TerrainDetail::proceduralFlatsAt(double x, double z, double jitter, const std::function<void(double, double)>& fn) const {
    if (!bigRoad_ || (skip_ & 32)) return;
    for (const Flat* f : flatsCovering(x, z))
        if (terra::holds(f->box, x, z)) fn(insideDistance(*f, x, z, jitter), f->invMargin);
}

double TerrainDetail::duneRelief(double x, double z, double h, double h0) const {
    const double L = floorLevel(x, z, h0);
    const double r = L - h0;
    return (h <= L + double(0.01f) && r > 0) ? r : 0.0;
}

uint8_t TerrainDetail::workBumpZone(double x, double z) const {
    if (!bigRoad_ || (skip_ & 32)) return 0;
    const BumpCell* c = bumpCell(int32_t(std::floor(x / 400.0)), int32_t(std::floor(z / 400.0)));
    if (!c) return 0;
    double workBumpsDummy = 0;
    workBumps(x, z, workBumpsDummy);  // builds the cell's cover list
    uint8_t zone = 0;
    double best = 1e8;
    for (const BumpSeg* sp : c->cover) {
        const BumpSeg& s = *sp;
        if (!terra::holds(s.box, x, z)) continue;
        const double t = (s.tx * x + s.tz * z) - s.c34;
        double D;
        if (t <= 0) D = std::hypot(x - s.ax, z - s.az);
        else if (t >= 1) D = std::hypot(x - s.bx, z - s.bz);
        else D = std::fabs((s.dz * x - s.dx * z) - s.c30);
        if (!(D < s.R)) continue;
        const double d = std::hypot(s.bx - x, s.bz - z);  // point +0x40 = B (the jittered lattice point)
        if (d < best) best = d, zone = s.mode;
    }
    return zone;
}

size_t TerrainDetail::workBumpCount() const { return bumpCount_.load(); }

// 0x64a2a0 with flags 5: bit 0 skips the gulches, flat surfaces and work bumps of 0x64d550;
// roads get cover 1 (a = 0) and the snow lift is added.
double TerrainDetail::roadHeight(double x, double z) const {
    GroundSample s;
    double y = detailOnly(x, z, &s);
    double r = 0;
    if (!(skip_ & 1)) r = roads_ ? roads_(x, z, y, 1.0, 1 - s.w18) : roadCarve(x, z, y, 1.0, 1 - s.w18);
    const double hg0 = clamp01(s.h0 - (2500.0 - (-s.nz) * 250.0));
    const double hb = clamp01(hg0 - (1 - s.ny) * 2.0);
    const double t = hb * 2 - 1;
    const double hg = t <= 0 ? 0.5 + (std::pow(t + 1, 8) - 1) * 0.5 : 0.5 + (1 - std::pow(1 - t, 8)) * 0.5;
    return y + std::max(0.0, double(0.3f) * hg - r);
}

// Gulches_AnySegmentNear 0x79fe80: a gulch segment whose u16 box overlaps p +- r (0x7a9e60 box
// query, then 0x6f0c40). Box tests only, no capsule distance.
bool TerrainDetail::gulchNear(double x, double z, double r) const {
    for (int32_t cz = int32_t(std::floor((z - r) / kCell)); cz <= int32_t(std::floor((z + r) / kCell)); ++cz)
        for (int32_t cx = int32_t(std::floor((x - r) / kCell)); cx <= int32_t(std::floor((x + r) / kCell)); ++cx) {
            const auto it = gulchCells_.find(cellKey(cx, cz));
            if (it == gulchCells_.end()) continue;
            for (uint32_t line : it->second)
                for (const GulchSeg& s : gulchSegments(line))
                    if (terra::overlapsQuery(s.box, x, z, r) && terra::overlaps(s.box, x, z, r)) return true;
        }
    return false;
}

// The engine builds terraform cells around the moving focus and each new procedural flat must
// not overlap one already built (0x79b130), so its result depends on the build order. The port
// builds cells on first use in a fixed order that the engine could also have used: phase =
// (cx mod 3) + 3 (cz mod 3); a cell is built after every cell of a lower phase within two cells
// (flats of cells three or more apart cannot touch: centre <= ~220 m outside its cell, box
// r <= 100 m, half extent <= 141 m). See notes p34 F3.
const TerrainDetail::FlatCell* TerrainDetail::flatCell(int32_t cx, int32_t cz) const {
    const int32_t i = cx + kFlatCells / 2, j = cz + kFlatCells / 2;
    if (i < 0 || j < 0 || i >= kFlatCells || j >= kFlatCells) return nullptr;
    std::atomic<FlatCell*>& slot = flatCells2_[size_t(i) * kFlatCells + size_t(j)];
    FlatCell* c = slot.load(std::memory_order_acquire);
    if (!c) {
        auto* fresh = new FlatCell;
        if (slot.compare_exchange_strong(c, fresh, std::memory_order_acq_rel)) c = fresh;
        else delete fresh;
    }
    std::call_once(c->once, [&] { buildFlatCell(cx, cz, *c); });
    return c;
}

namespace {
int32_t mod3(int32_t v) { return ((v % 3) + 3) % 3; }
int32_t flatPhase(int32_t cx, int32_t cz) { return mod3(cx) + 3 * mod3(cz); }
}  // namespace

// AGW_TerraFlat::GenerateCell 0x799910 (asm 0x799910-0x79a835, notes p34 F1/F2).
void TerrainDetail::buildFlatCell(int32_t cx, int32_t cz, FlatCell& cell) const {
    if (!bigRoad_ || (skip_ & 32)) return;
    const int32_t phase = flatPhase(cx, cz);
    std::vector<const FlatCell*> earlier;
    for (int32_t i = cx - 2; i <= cx + 2; ++i)
        for (int32_t j = cz - 2; j <= cz + 2; ++j)
            if (flatPhase(i, j) < phase)
                if (const FlatCell* c = flatCell(i, j)) earlier.push_back(c);
    // 0x79b130: a procedural surface (+0xa9) whose box overlaps the box centre +- r.
    auto blocked = [&](double x, double z, double r) {
        for (const Flat& f : cell.flats)
            if (terra::overlapsQuery(f.box, x, z, r)) return true;
        for (const FlatCell* c : earlier)
            for (const Flat& f : c->flats)
                if (terra::overlapsQuery(f.box, x, z, r)) return true;
        return false;
    };
    const Permutation& RT = perm_;
    const double kx = double(12.317513f), kz = double(10.193578f);
    const double X0 = double(float(cx) * 450.0f), X1 = double(float(cx + 1) * 450.0f);
    const double Z0 = double(float(cz) * 450.0f), Z1 = double(float(cz + 1) * 450.0f);
    for (double x = X0; x < X1; x += 100.0) {
        for (double z = Z0; z < Z1; z += 100.0) {
            const double pop = clamp01(map08_.value(x, z));  // 0x458e60 Population (slot 8)
            if (map18_.value(x, z) > 0.5) continue;           // 0x646730
            // (0x64a2a0 at the lattice point: its value is not used.)
            const double zk = terra::f32down(z * kz);
            const double r = terra::f32down(RT[size_t(int64_t(std::floor(terra::f32down(x * kx + z * kz))) & 255)] * double(0.003921569f));
            if (!(pop >= 0.5 || (r <= 0.75 && (pop >= 0.1 || r <= 0.5)))) continue;
            const RoadHit road = bigRoad_(x, z);  // 0x6fb320
            if (!(road.edge < 50.0) || !road.found) continue;
            const double rpx = road.px, rpz = road.pz, dx = road.dx, dz = road.dz;
            // Toward the road (xz) and perp = dir x up = (-dz, 0, dx), turned toward the road.
            double px = -dz, pz = dx;
            const double pl = px * px + pz * pz;
            if (pl > 1e-6) px /= std::sqrt(pl), pz /= std::sqrt(pl);
            if (px * (rpx - x) + pz * (rpz - z) < 0) px = -px, pz = -pz;
            const double hR = terra::f32down(roadHeight(rpx, rpz));
            const double r2 = terra::f32down(RT[size_t(int64_t(std::floor(terra::f32down((r * double(931.9283f) + x) * kx + zk))) & 255)] *
                                             double(0.003921569f));
            double thr = pop > 0.5 ? 0.2 : 0.1;
            if (pop > 0.75) thr = double(0.8f);
            const double E = thr <= r2 ? 30.0 : (thr * double(0.1f) <= r2 ? 50.0 : 100.0);
            // Probes: f = min(5 / (|hR - h| + 1e-4), 1) shrinks each length; S = max f |dh|.
            double S = 0;
            auto probe = [&](double qx, double qz, double& h) {
                h = terra::f32down(roadHeight(qx, qz));
                const double dh = std::fabs(hR - h);
                const double f = std::min(5.0 / (dh + double(1e-4f)), 1.0);
                S = std::max(S, f * dh);
                return f;
            };
            double h1, h2, h3, h4;
            const double q1x = rpx - dx * E, q1z = rpz - dz * E;
            const double L = terra::f32down(probe(q1x, q1z, h1) * E);
            if (!(L > 10)) continue;
            const double q2x = rpx + dx * L, q2z = rpz + dz * L;
            const double L2 = terra::f32down(probe(q2x, q2z, h2) * L);
            if (!(L2 > 10)) continue;
            const double D1 = terra::f32down(probe(rpx - dx * L2 - px * 2 * E, rpz - dz * L2 - pz * 2 * E, h3) * E);
            if (!(D1 > 10)) continue;
            double D = terra::f32down(probe(rpx + dx * L2 - px * 2 * D1, rpz + dz * L2 - pz * 2 * D1, h4) * D1);
            if (!(D > 10)) continue;
            if (pop < r2 && D > 19) D = 19;
            double m = L2 > D ? D : L2;
            if (2 * S < m) m = 2 * S;
            if (!(m > 5)) m = 5;
            const double off = terra::f32down(D + road.c8 - m - 2);
            const double ccx = terra::f32down(rpx - px * off), ccz = terra::f32down(rpz - pz * off);
            // 0x6fb240: the nearest road (any type) to the centre must be > 0.99 off away.
            const RoadHit nr = nearRoad_(ccx, ccz, off);
            if (nr.found && !(std::hypot(ccx - nr.px, ccz - nr.pz) > off * double(0.99f))) continue;
            const double rr = std::max(L2, D);
            if (gulchNear(ccx, ccz, rr) || blocked(ccx, ccz, rr)) continue;
            // New surface: SetShape 0x79c330(centre, X = dir / L2, Z = perp / D, margin m).
            Flat f{};
            f.cx = ccx, f.cy = hR, f.cz = ccz;
            f.ax = dx * L2, f.az = dz * L2, f.lenX = L2;
            f.bx = px * D, f.bz = pz * D, f.lenZ = D;
            f.invMargin = terra::f32down(1.0 / std::max(m, double(0.1f)));
            f.strength = double(0.8f);
            f.priority = 0x80;  // constructor 0x74cb50
            f.ellipse = false;
            f.listB = false;
            f.bumps = true;
            f.procedural = true;
            double nx = 0, ny = 1, nz = 0;
            auto plane = [&] {  // n = normalize((q2 - q1) x perp) (0x4adfe0, 0x530b40), n.y >= 0
                const double vx = q2x - q1x, vy = h2 - h1, vz = q2z - q1z;
                nx = vz * 0 - vy * pz, ny = vx * pz - vz * px, nz = vy * px - vx * 0;
                const double l = std::sqrt(nx * nx + ny * ny + nz * nz);
                if (l * l > 1e-6) nx /= l, ny /= l, nz /= l;
                if (ny < 0) nx = -nx, ny = -ny, nz = -nz;
            };
            if (D > 25) {
                if (pop > double(0.9f) || L2 > 20) {
                    f.strength = 1.0, f.mode = 1, f.bumps = false;
                } else {
                    plane();
                    f.mode = 3;
                }
            } else {
                plane();
                f.mode = 2;
            }
            f.nx = nx, f.nz = nz;
            f.box = terra::boxAt(f.cx, f.cz);
            for (int k = 0; k < 4; ++k) {
                const double sx = (k & 1) ? -1 : 1, sz = (k & 2) ? -1 : 1;
                terra::extend(f.box, f.cx + sx * f.ax + sz * f.bx, f.cz + sx * f.az + sz * f.bz);
            }
            if (std::getenv("FUEL_FLAT_DEBUG"))
                std::fprintf(stderr, "flat lattice (%.0f,%.0f) pop %.2f r %.3f r2 %.3f road (%.2f,%.2f) edge %.2f dir (%.3f,%.3f) hR %.2f E %.0f L %.2f L2 %.2f D1 %.2f D %.2f S %.2f m %.2f c8 %.2f off %.2f h1..4 %.2f %.2f %.2f %.2f\n",
                             x, z, pop, r, r2, rpx, rpz, road.edge, dx, dz, hR, E, L, L2, D1, D, S, m, road.c8, off, h1, h2, h3, h4);
            cell.flats.push_back(f);
            procCount_.fetch_add(1, std::memory_order_relaxed);
        }
    }
}

const TerrainDetail::BumpCell* TerrainDetail::bumpCell(int32_t cx, int32_t cz) const {
    const int32_t i = cx + kBumpCells / 2, j = cz + kBumpCells / 2;
    if (i < 0 || j < 0 || i >= kBumpCells || j >= kBumpCells) return nullptr;
    std::atomic<BumpCell*>& slot = bumpCells_[size_t(i) * kBumpCells + size_t(j)];
    BumpCell* c = slot.load(std::memory_order_acquire);
    if (!c) {
        auto* fresh = new BumpCell;
        if (slot.compare_exchange_strong(c, fresh, std::memory_order_acq_rel)) c = fresh;
        else delete fresh;
    }
    std::call_once(c->once, [&] { buildBumpCell(cx, cz, *c); });
    return c;
}

// AGW_WorkBumps::GenerateCell 0x79c980 (asm 0x79c980-0x79ce3b, notes p34 F4): a 40 m lattice in
// 400 m cells, each point jittered by up to 12 x 0.8 m; inside the nearest fully covering flat
// surface (0x79bfd0) when that is a procedural mode-3 one with more than 15 m inside it.
void TerrainDetail::buildBumpCell(int32_t cx, int32_t cz, BumpCell& cell) const {
    if (!bigRoad_ || (skip_ & 32)) return;
    const double X0 = double(float(cx) * 400.0f), X1 = double(float(cx + 1) * 400.0f) - 0.5;
    const double Z0 = double(float(cz) * 400.0f), Z1 = double(float(cz + 1) * 400.0f) - 0.5;
    // Procedural mode-3 flats near the cell (a cheap early out; only they can make bumps).
    std::vector<const FlatCell*> fc;
    bool any3 = false;
    for (int32_t i = int32_t(std::floor((X0 - 450) / 450.0)); i <= int32_t(std::floor((X1 + 450) / 450.0)); ++i)
        for (int32_t j = int32_t(std::floor((Z0 - 450) / 450.0)); j <= int32_t(std::floor((Z1 + 450) / 450.0)); ++j)
            if (const FlatCell* c = flatCell(i, j)) {
                fc.push_back(c);
                for (const Flat& f : c->flats) any3 |= f.mode == 3;
            }
    if (!any3) return;
    const Permutation& RT = perm_;
    const double kx = double(12.317513f), kz = double(10.193578f);
    for (double x = X0; x < X1; x += 40.0) {
        const double xk = terra::f32down(x * kx);
        for (double z = Z0; z < Z1; z += 40.0) {
            const uint8_t b = RT[size_t(int64_t(std::floor(terra::f32down(z * kz + xk + double(37.82f))))) & 255];
            const double px = terra::f32down(x + double(b & 15) * double(0.8000001f));
            const double pz = terra::f32down(z + double(b >> 4) * double(0.8000001f));
            // 0x79bfd0 over 0x79b7a0: every surface (both lists, file and procedural) whose box
            // holds the point and with d(5 J) > margin; the nearest centre (xz) wins.
            const double jit = 5.0 * jitterJ(px, pz);
            const Flat* best = nullptr;
            double bestD = 1e8;
            auto consider = [&](const Flat& f) {
                if (!(1.0 / f.invMargin < insideDistance(f, px, pz, jit))) return;
                const double d = std::hypot(f.cx - px, f.cz - pz);
                if (d < bestD) bestD = d, best = &f;
            };
            const auto it = flatCells_.find(cellKey(int32_t(std::floor(px / kCell)), int32_t(std::floor(pz / kCell))));
            if (it != flatCells_.end())
                for (uint32_t i : it->second)
                    if (terra::holds(flats_[i].box, px, pz)) consider(flats_[i]);
            for (const Flat* f : flatsCovering(px, pz))
                if (terra::holds(f->box, px, pz)) consider(*f);
            if (!best || best->mode != 3 || !(insideDistance(*best, px, pz, 0) > 15.0)) continue;
            const double zk2 = pz * kz;
            const double r2 = RT[size_t(int64_t(std::floor(terra::f32down(zk2 + px * kx))) & 255)] * double(0.003921569f);
            BumpSeg s{};
            s.mode = r2 <= 0.5 ? 3 : 1;
            const double r3 =
                terra::f32down(RT[size_t(int64_t(std::floor(terra::f32down((r2 * double(931.9283f) + px) * kx + terra::f32down(zk2)))) & 255)] *
                               double(0.003921569f));
            // Direction: (p - c) x up = (-(pz - cz), (px - cx)), normalised; length/half width R.
            double ax = -(pz - best->cz), az = px - best->cx;
            const double l2 = ax * ax + az * az;
            if (l2 > 1e-6) ax /= std::sqrt(l2), az /= std::sqrt(l2);
            const double half = (best->lenZ + best->lenX) * 0.5;
            const double R = terra::f32down(std::max(5.0, half * double(0.02f) + double(0.05f) * half * r3));
            // Segment 0x6f10c0 from A = p + R a to B = p.
            s.ax = terra::f32down(px + ax * R), s.az = terra::f32down(pz + az * R), s.bx = px, s.bz = pz, s.R = R;
            const double ddx = s.bx - s.ax, ddz = s.bz - s.az, len = std::sqrt(ddx * ddx + ddz * ddz);
            if (!(len > 0)) continue;
            s.dx = ddx / len, s.dz = ddz / len, s.tx = s.dx / len, s.tz = s.dz / len;
            s.c30 = s.dz * s.ax - s.dx * s.az, s.c34 = s.tx * s.ax + s.tz * s.az;
            s.box = terra::segmentBox(s.ax, s.az, s.bx, s.bz, s.dx, s.dz, R);
            cell.segs.push_back(s);
            bumpCount_.fetch_add(1, std::memory_order_relaxed);
        }
    }
}

// 0x79cfa0 (after list B in 0x648400): each bump whose box holds the point adds
// 0.5 R S2(1 - 2 De / R) with De = D - 2 S2(2 J - 1). Returns 1 if any bump applied.
double TerrainDetail::workBumps(double x, double z, double& y) const {
    if (!bigRoad_ || (skip_ & 32)) return 0;
    const int32_t cx = int32_t(std::floor(x / 400.0)), cz = int32_t(std::floor(z / 400.0));
    double ret = 0, J = -1;
    BumpCell* c = const_cast<BumpCell*>(bumpCell(cx, cz));
    if (!c) return 0;
    std::call_once(c->coverOnce, [&] {
        const double x0 = cx * 400.0, z0 = cz * 400.0;
        const CellBox cb{terra::q(x0), terra::q(z0), terra::q(x0 + 400), terra::q(z0 + 400)};
        for (int32_t i = cx - 1; i <= cx + 1; ++i)
            for (int32_t j = cz - 1; j <= cz + 1; ++j)
                if (const BumpCell* n = bumpCell(i, j))
                    for (const BumpSeg& s : n->segs)
                        if (s.box.minX <= cb.maxX && cb.minX <= s.box.maxX && s.box.minZ <= cb.maxZ && cb.minZ <= s.box.maxZ) c->cover.push_back(&s);
    });
    {
            for (const BumpSeg* sp : c->cover) {
                const BumpSeg& s = *sp;
                if (!terra::holds(s.box, x, z)) continue;
                const double t = (s.tx * x + s.tz * z) - s.c34;
                double D;
                if (t <= 0) D = std::hypot(x - s.ax, z - s.az);
                else if (t >= 1) D = std::hypot(x - s.bx, z - s.bz);
                else D = std::fabs((s.dz * x - s.dx * z) - s.c30);
                D = terra::f32down(D);
                if (J < 0) J = jitterJ(x, z);
                const double De = D - S2(2 * J - 1) * 2.0;
                if (De < s.R) {
                    y += S2((s.R - De) / s.R * 2 - 1) * s.R * 0.5;
                    ret = 1;
                }
            }
        }
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
    // Work bumps (0x79cfa0), only inside procedural mode-3 TerraFlat surfaces.
    workBumps(x, z, y);
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
        // Ends blend to the ground at the node (0x6f81f0 with junction heights = the node's
        // ground), applied to the raw samples before the smoothing, as the engine does.
        double ex, ez;
        at(0, ex, ez);
        const double hs = detailOnly(ex, ez, nullptr);
        at(L, ex, ez);
        const double he = detailOnly(ex, ez, nullptr);
        const double ls = std::min(double(T.flat) + std::min(double(T.flat), 1.0) * M, 0.33 * L);
        const double Rl = std::max(std::min(std::max(M, 50.0), L - 2 * ls), 0.001);
        for (size_t k = 0; k < raw.size(); ++k) {
            const double sk = 2.0 * (double(k) - 2 * K);
            const double r1 = S2(2 * clamp01(1 - (sk - ls) / Rl) - 1), r2 = S2(2 * clamp01(1 - ((L - sk) - ls) / Rl) - 1);
            double h = r1 * hs + (1 - r1) * raw[k];
            raw[k] = (1 - r2) * h + r2 * he;
        }
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
