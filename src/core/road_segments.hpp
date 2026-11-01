// FUEL's roads as the terrain sees them (AGW_TerraRoad, FUEL.exe 0x6fa8a0 and friends).
//
// GwRoad_Z polylines are linked into a graph at their shared end points (0x6d0fc0),
// chained through junctions, smoothed, given a levelled height profile and cut into
// cA_RoadSegment pieces (0x6f9680 / 0x6f9150). Two point functions read the segments:
//   sample() - the ground point's road fields (0x6fbab0): road weight, tarmac share,
//              roughness, the road decal's uv / weight / material, puddles, berm;
//   carve()  - the road's pull on the ground height (0x6fcaa0), called at the end of
//              the terrain height function (AGW_Altitude 0x64a2a0 -> 0x64d550).
// Road shapes come from the road-type table FUEL builds at start-up (road_types.hpp).
// Decoded in ~/re-work/fuel/notes/p22-ground-roads-water.md (R0-R4) and
// ~/re-work/fuel/out/p22/roadbuild.md (graph, builder, segment fields).
#pragma once

#include "dpc.hpp"
#include "genworld_noise.hpp"
#include "road_table.hpp"

#include <array>
#include <atomic>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <unordered_map>
#include <vector>

namespace fuel {

// The road fields of the ground point context (ctx offsets in FUEL's point evaluator).
struct RoadFields {
    float weight = 0;      // 0x6fbab0 return value (ctx+0x1e8 before the flat-surface term)
    float tarmac = 0;      // ctx+0x1f4
    float roughness = 0;   // ctx+0x1f0 (before bit 12's adjustment)
    float across = 0;      // ctx+0x1f8: decal u, 0..1 across the road
    float along = 0;       // ctx+0x1fc: decal v along the road
    float decal = 0;       // ctx+0x204: decal weight
    float puddle = 0;      // ctx+0x208
    float minHeight = 1e6f;  // ctx+0x20c: lowest road profile height among nearby segments
    float maxT60 = 0;      // ctx+0x210
    float inside = 0;      // ctx+0x214: max(T[bc] - distance)
    float inside2 = 0;     // ctx+0x218: max(T[cc] - normalised distance)
    float berm = 0;        // ctx+0x21c
    float junction = 0;    // ctx+0x238
    uint16_t material = 0xfff;  // ctx+0x2dc: decal texture enum (0x2a.. Road01, 0x34 Dirt01), 0xfff none
    float colour[4] = {0, 0, 0, 0};  // ctx+0x50 as the sampler leaves it
};

class RoadSampler {
public:
    // The terrain height without roads (AGW_Altitude with flags 3: no gulches, flat
    // surfaces, roads or snow lift), used for the road height profiles.
    using HeightFn = std::function<double(double x, double z)>;

    // Throws if the world has no GenWorld_Z / GwRoad_Z. `types` must be valid.
    RoadSampler(const DpcFile& world, RoadTypeTable types, HeightFn rawHeight);
    ~RoadSampler();

    RoadFields sample(double x, double z) const;
    // 0x6fcaa0 with P21's RoadCarver signature: pulls `y` (the height so far) toward the
    // road; returns the road weight r used by the snow lift (0 off roads).
    double carve(double x, double z, double& y, double cover, double detailWeight) const;

    const RoadTypeTable& types() const { return types_; }
    size_t recordCount() const { return recs_.size(); }
    size_t builtSegments() const;  // segments built so far (they are built on first use)

    struct Segment {
        float ax, az, bx, bz;   // +0x20..+0x2c
        float dx, dz;           // +0x10/+0x14 unit direction
        float tx, tz;           // +0x18/+0x1c direction / length
        float c30, c34, len, w; // +0x30, +0x34, +0x38, +0x3c
        int16_t base;           // +0x44
        std::vector<int16_t> h; // +0x40 samples (cm above base)
        uint32_t record;        // +0x48
        uint16_t start;         // +0x4c (dm along the chain)
        int8_t tan0, tan1;      // +0x4e/+0x4f
        int8_t curv0, curv1;    // +0x50/+0x51
        uint8_t wid0, wid1;     // +0x52/+0x53
        uint8_t type;           // +0x56
        uint8_t pri;            // +0x57
        float heightAt(float t, const RoadType& T) const;  // 0x6f1800
    };
    // Segments whose influence may reach (x, z) (built on demand), in a fixed order.
    void segmentsNear(double x, double z, std::vector<const Segment*>& out) const;

    // Nearest-road queries of the terrain code (decoded by P32, out/p32/chunkroads.md; used by
    // the procedural flat surfaces 0x799910). Candidates: built segments whose u16 cell box
    // (0x6f0ef0) overlaps (x, z) +- r (0x6f0c40); edge = capsule distance (0x6f0da0) - T[+0xbc].
    struct Nearest {
        const Segment* seg = nullptr;
        float edge = 10000.0f;  // min max(0, edge); 10000 [0xa2a4e0] when nothing
        float px = 0, pz = 0;   // closest centreline point of `seg` (0x6f0e30)
    };
    // 0x6fac40 + 0x6fb240: any road; `seg` = the one with the smallest signed edge.
    Nearest nearestRoad(double x, double z, double r) const;
    // 0x6fb0c0 + 0x6fb320: types with T[+0x1fc] (big roads) within 50 m; `seg` = the one with the
    // smallest centreline distance.
    Nearest nearestBigRoad(double x, double z) const;

private:
    struct Record {
        int16_t minX, minZ, maxX, maxZ;  // 2 m cells
        uint32_t first;                  // into pts_
        uint16_t count;
        uint8_t type;
        bool rigid;
        uint32_t link0, link1;           // ring neighbour at end 0 / end 1
        uint8_t side0, side1;            // which end of that neighbour
    };
    struct Chain {
        std::vector<std::pair<uint32_t, bool>> recs;  // record, forward
        size_t main = 0;
    };
    using P2 = std::array<float, 2>;

    Nearest nearest(double x, double z, double r, bool big) const;
    P2 point(uint32_t rec, uint32_t k) const;
    float recordLength(uint32_t rec) const;
    void ring(uint32_t rec, int end, std::vector<std::pair<uint32_t, int>>& out) const;
    bool inEnd1Ring(uint32_t a, uint32_t b) const;
    std::pair<int64_t, int64_t> pairAtNode(uint32_t rec, int end) const;
    bool junctionHeight(uint32_t rec, int end, double& h) const;
    double altAlong(uint32_t r, uint32_t ref, double x) const;
    void build(uint32_t rec, std::vector<Segment>& out) const;
    void buildRigid(uint32_t rec, std::vector<Segment>& out) const;
    const std::vector<Segment>& segmentsOf(uint32_t rec) const;
    void setEndpoints(Segment& s, P2 a, P2 b, float w) const;

    RoadTypeTable types_;
    HeightFn alt_;
    std::vector<Record> recs_;
    std::vector<P2> pts_;
    std::unordered_map<uint64_t, std::vector<uint32_t>> cells_;  // 64 m cells -> records whose influence reaches them
    mutable std::unique_ptr<std::atomic<uint8_t>[]> state_;      // 0 not built, 1 building, 2 built
    mutable std::vector<std::vector<Segment>> segs_;
    mutable std::array<std::mutex, 64> locks_;
    Fractal f3f54_, f4f28_, f11cec_, f5efc_;  // ground noise bank (p14 G3)

    float edgeJitter(double x, double z) const;   // 0x6ef6f0
    float dirtBumps(double x, double z) const;    // 0x6f0430
    float roughNoise(double x, double z) const;   // 0x6ef020
};

// Palette slot k of the GenWorld Colors (the road table copies slots 15, 21, 25).
RoadTypeTable loadRoadTypeTable(const DpcFile& world);

}  // namespace fuel
