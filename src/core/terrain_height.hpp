// FUEL's ground height: the procedural detail the engine adds to the 30 m map at
// run time (AGW_Altitude, FUEL.exe 0x64a2a0, and everything it calls). Decoded in
// notes p21-terrain.md (H0-H5); this is a 1:1 port with the exceptions listed there.
//
//   h0, n  = cubic B-spline of map slot 0 and its normal
//   h2     = h0 + two slope-weighted detail fractals (15 m and 4.7 m cells)
//   R      = rock outcrops (map slot 10, four terrain fractals; 2 m lattice)
//   floor  = canyon terraces where map slot 12 >= 0.1
//   h      = lerp(max(h2 + R, floor), h0, clamp(map slot 18))
//   ground = h - gulches (GwRoad type-14 river beds), flat surfaces list A (file +
//            procedural, 0x799910), [roads: hook], flat surfaces list B, work bumps
//            (0x79c980 / 0x79cfa0), + 0.3 * high-ground (snow) lift
// Procedural flats and work bumps: notes p34-terrain-leftovers.md F1-F4.
#pragma once

#include "dpc.hpp"
#include "genworld_noise.hpp"
#include "road_types.hpp"

#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

namespace fuel {

// One GenWorld map (Binary_Z) sampled the way GwMapSampler does (0x69b620 / 0x69c560):
// uniform cubic B-spline, pixel = x * W/140000 + W/2, value = raw / maxValue (* scale).
struct MapSampler {
    uint32_t width = 0, height = 0;
    float pxPerMetre = 0;  // float32 W * 7.142857e-06 as the engine computes it
    float scale = 1;       // value per raw unit
    std::vector<uint16_t> raw;
    bool valid() const { return !raw.empty(); }
    // Value, and optionally d(value)/d(pixel) along x and z.
    double value(double x, double z, double* dpx = nullptr, double* dpz = nullptr) const;
};

// Everything the height function computed for one point (for checks and the mesh).
struct GroundSample {
    double h0 = 0;             // B-spline of the map
    double nx = 0, ny = 1, nz = 0;  // its normal
    double h2 = 0;             // + detail fractals
    double rock = 0;           // R
    double floor = 0;          // canyon floor level v (h0 - 100 when none)
    double w18 = 0;            // map slot 18 weight back to h0
    double detail = 0;         // height before the finishing pass (0x64a2a0 flags 3)
    double gulch = 0, flatA = 0, road = 0, flatB = 0;  // height after each finishing step
    double ground = 0;         // final height
};

class TerrainDetail {
public:
    // `base` is map slot 0 already decoded (moved in).
    TerrainDetail(const DpcFile& world, MapSampler base);
    ~TerrainDetail();

    double height(double x, double z) const;            // flags 0: the ground at a point
    // The engine's ground query (0x64e810, used by FITTOGROUND and gameplay): the ground at
    // the four integer-metre points around (x, z), bilinearly interpolated (cached per thread
    // like 0x63ea40).
    double ground(double x, double z) const;
    double detailHeight(double x, double z) const;      // flags 3: no gulches/flats/roads/snow
    GroundSample sample(double x, double z) const;
    double baseHeight(double x, double z) const { return base_.value(x, z); }
    // Map height and normal (0x64a2a0 step 1 / 0x45de10).
    double baseNormal(double x, double z, float n[3]) const;
    // The height far ground chunks (>= 240 m) use, 0x64be70: the map raised to the canyon
    // floor (0x7a2610), no detail fractals, rock, gulches, flats or roads. `n` = map normal.
    double farHeight(double x, double z, float n[3]) const;
    // Ground normal by central differences of height() over +-e metres.
    void normal(double x, double z, double e, float out[3]) const;

    // Roads carve and lift the ground (0x6fcaa0, decoded by the roads work). The hook
    // receives (x, z, y in/out, 1 - flat cover, 1 - slot-18 weight) and returns the road
    // weight r used by the snow term; unset = the port's own decode (roadCarve, p21 H6).
    using RoadCarver = std::function<double(double x, double z, double& y, double cover, double detailWeight)>;
    void setRoadCarver(RoadCarver c) { roads_ = std::move(c); }

    // Road queries for the procedural flat surfaces (AGW_TerraFlat::GenerateCell 0x799910): the
    // nearest big road within 50 m (0x6fb320) and the nearest road within r (0x6fb240). Without
    // them no procedural flats (and no work bumps) are made.
    struct RoadHit {
        bool found = false;
        double edge = 10000;    // min max(0, distance - T[+0xbc])
        double px = 0, pz = 0;  // closest centreline point of the chosen segment
        double dx = 0, dz = 0;  // its unit direction (+0x10 / +0x14)
        double c8 = 0;          // T[+0xc8] of its road type
    };
    using BigRoadQuery = std::function<RoadHit(double x, double z)>;
    using NearRoadQuery = std::function<RoadHit(double x, double z, double r)>;
    void setRoadQueries(BigRoadQuery big, NearRoadQuery near);

    // Rivers (Gulches_WaterQuery 0x7a07d0 over 0x7a00e0): the carve depth D at (x, z) and the
    // output A; `y` is the finished ground height there (0x63ea40). D = 0 off rivers.
    struct River {
        bool flag = false;
        double depth = 0;  // D (-> GenWorld_WaterQuery h1)
        double bed = 0;    // A (-> ctx+0x1b0 = max(A, 2))
    };
    River river(double x, double z, double y) const;
    // Dune_Offset 0x6ad0e0 (point bit 25): the floor relief r = L - h0 of Dune_Level 0x7a2310
    // (= the canyon floor level, map slot 12) when the ground h is at most 1 cm above L and r > 0,
    // else 0. h = finished height (ctx+0xb4), h0 = map height (ctx+0x190).
    double duneRelief(double x, double z, double h, double h0) const;
    // The canyon terraces 0x7a0c80 at an already scaled position (for the emulator vectors).
    double canyonTerraces(double sx, double sz, double a, double b) const { return canyon(sx, sz, a, b); }
    // WorkBump_Nearest 0x79d190 -> the ground point's zone byte ctx+0x240 (0x6b9ad0): the mode
    // (+0x50: 1 or 3) of the work bump whose capsule holds (x, z) with the nearest point +0x40;
    // 0 when none.
    uint8_t workBumpZone(double x, double z) const;

    size_t flatSurfaceCount() const;
    size_t proceduralFlatCount() const;  // built so far
    // Procedural flat surfaces with centres within r of (x, z) (builds their cells): centre,
    // half extents along / across the road (L2, D), the axis along the road, mode, strength.
    struct ProcFlatInfo {
        double cx, cy, cz, lenX, lenZ, ax, az, nx, nz, margin, strength;
        int mode;
    };
    std::vector<ProcFlatInfo> proceduralFlats(double x, double z, double r) const;
    // For each procedural flat whose u16 cell box holds (x, z) (the ground point's candidate
    // lists 0x79b1f0 / 0x79a850): fn(inside distance minus `jitter` (0x799020), 1 / margin).
    // Their grass fields are the constructor's (0x74cb50): +0x90 = 1, +0x94 = +0x98 = +0x9c = 0.
    void proceduralFlatsAt(double x, double z, double jitter, const std::function<void(double d, double invMargin)>& fn) const;
    size_t workBumpCount() const;        // built so far
    // The file flat surfaces (GenWorld_Z records) whose cell box holds (x, z), for diagnostics:
    // record index, centre, half lengths, priority, list B, ellipse, inside distance (no jitter),
    // plane height P there, strength, margin, and the record's raw 0x83 bytes.
    struct FileFlatInfo {
        uint32_t index;
        double cx, cy, cz, lenX, lenZ, d, P, strength, margin;
        int priority;
        bool listB, ellipse;
        std::vector<uint8_t> raw;
    };
    std::vector<FileFlatInfo> fileFlatsAt(double x, double z) const;
    // The same for record `index` at any point (d may be negative outside).
    FileFlatInfo fileFlat(uint32_t index, double x, double z) const;
    // The height with flags 5 (0x64a2a0 | 1): detail, roads and the snow lift, no gulches, flat
    // surfaces or work bumps (what 0x799910 probes the ground with).
    double roadHeight(double x, double z) const;
    size_t gulchLineCount() const;
    size_t roadLineCount() const;

private:
    struct Flat;
    struct GulchLine;
    struct RoadLine;
    struct RoadBuilt;
    struct GulchSeg;
    struct FlatCell;
    struct BumpSeg;
    struct BumpCell;

    double baseWithNormal(double x, double z, double n[3]) const;
    double rockEval(double x, double z, double ny) const;   // 0x7a4c20 return value L
    double rockLattice(double x, double z) const;           // 0x7a69b0 near-camera path
    double canyon(double sx, double sz, double a, double b) const;  // 0x7a0c80
    double floorLevel(double x, double z, double h0) const; // 0x7a2310
    double detailOnly(double x, double z, GroundSample* s) const;
    double gulchDepth(double x, double z) const;            // 0x7a00e0
    double gulchProfile(double x, double z, double* sh, double* sw) const;
    double combineFlats(bool listB, double x, double z, double& y) const;  // 0x79b870
    double insideDistance(const Flat& s, double x, double z, double jitter) const;  // 0x799020
    double jitterJ(double x, double z) const;                                        // 0x6ef6f0
    const FlatCell* flatCell(int32_t cx, int32_t cz) const;   // AGW_TerraFlat 450 m cell, built on first use
    void buildFlatCell(int32_t cx, int32_t cz, FlatCell& c) const;  // 0x799910
    const std::vector<const Flat*>& flatsCovering(double x, double z) const;
    const BumpCell* bumpCell(int32_t cx, int32_t cz) const;   // AGW_WorkBumps 400 m cell
    void buildBumpCell(int32_t cx, int32_t cz, BumpCell& c) const;  // 0x79c980
    bool gulchNear(double x, double z, double r) const;       // 0x79fe80
    double workBumps(double x, double z, double& y) const;    // 0x79cfa0
    const std::vector<GulchSeg>& gulchSegments(uint32_t line) const;
    const RoadBuilt& roadBuilt(uint32_t line) const;
    double roadCarve(double x, double z, double& y, double cover, double detailWeight) const;  // 0x6fcaa0

    Permutation perm_{};
    MapSampler base_, map08_, map10_, map11_, map12_, map18_;
    Fractal f1_, f2_;                       // altitude object +4 / +0xfd8
    Fractal t0c_, t18_, t24_, t28_;         // terrain-detail object W+0x483a0
    Fractal canyonC_, canyonD_;             // W+0x48404 +0xc / +0x10
    Fractal b5efc_, b4f28_, b1fac_, bfd44_, b7ea4_, b11cec_;  // ground noise bank (p14 G3)

    std::vector<Flat> flats_;
    std::vector<std::vector<uint8_t>> flatRaw_;  // the 0x83-byte records, for diagnostics
    std::unordered_map<uint64_t, std::vector<uint32_t>> flatCells_;
    std::vector<GulchLine> gulchLines_;
    std::unordered_map<uint64_t, std::vector<uint32_t>> gulchCells_;
    mutable std::unique_ptr<std::once_flag[]> gulchOnce_;
    mutable std::vector<std::vector<GulchSeg>> gulchSegs_;
    std::array<RoadTypeParams, 21> roadTypes_{};
    bool haveRoadTypes_ = false;
    std::vector<RoadLine> roadLines_;
    std::unordered_map<uint64_t, std::vector<uint32_t>> roadCells_;
    mutable std::unique_ptr<std::once_flag[]> roadOnce_;
    mutable std::vector<std::unique_ptr<RoadBuilt>> roadBuilt_;
    Fractal b3f54_;  // bank +0x3f54 (road height noise)
    RoadCarver roads_;
    BigRoadQuery bigRoad_;
    NearRoadQuery nearRoad_;
    // Procedural cells: [cx + kCellOff][cz + kCellOff], allocated on first use.
    static constexpr int32_t kFlatCells = 320, kBumpCells = 360;
    mutable std::unique_ptr<std::atomic<FlatCell*>[]> flatCells2_;
    mutable std::unique_ptr<std::atomic<BumpCell*>[]> bumpCells_;
    mutable std::atomic<size_t> procCount_{0}, bumpCount_{0};
    uint32_t id_ = 0;  // for the per-thread lattice caches
    int skip_ = 0;     // FUEL_TERRAIN_SKIP diagnostics
};

// For emulator comparisons only (this thread): round float32 stores to nearest like Ghidra's
// pcode emulator instead of FUEL's round-down.
void setFloat32RoundNearest(bool on);

// Builds the detail for a world DPC (WORLD/USA1.DPC) from its GenWorld_Z maps, flat
// surfaces and GwRoad_Z gulch lines, with the base map already decoded in `baseRaw`.
std::shared_ptr<TerrainDetail> makeTerrainDetail(const DpcFile& world, uint32_t width, uint32_t height, float maxValue,
                                                 const std::vector<uint16_t>& baseRaw);

}  // namespace fuel
