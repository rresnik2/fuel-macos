// FUEL's road graph (StockRoads, FUEL.exe 0x6cfe80..0x6dbfff): the GwRoad records as a graph whose
// nodes are record end points and whose edges are whole records, with FUEL's nearest-road snap,
// road-at-point lookup and route search. The AI (IAPathBlockRoad, p29) and the GPS route use it.
// Decoded in ~/re-work/fuel/out/p32/roadgraph.md (summary in notes/p32-roads-ground-water.md):
//   nearestRoad  0x6d19f0 (+ SegDist2 0x6d1520 / 0x6d15d0 / 0x4ae1f0), grid 0x6df8b0 / 0x6dfa10
//   roadAt       0x6d1d70
//   search       0x6d7f90 (request 0x6d8e20 + update 0x6d8ee0 with full = 1), part costs 0x6d7e90
//   costs        0x6d7e70 (length), 0x4ef600 / 0x4ef590 (the AI's weighted cost)
//   highway bit  0x6d4130 (record flag bit 30)
//   polyline     the path 0x6da240 walks along a route (start piece, list records, end piece)
#pragma once

#include "dpc.hpp"

#include <array>
#include <cstdint>
#include <functional>
#include <vector>

namespace fuel {

class RoadGraph {
public:
    explicit RoadGraph(const DpcFile& world);  // throws if the world has no GenWorld_Z / GwRoad_Z

    struct Rec {
        int16_t minX, minZ, maxX, maxZ;  // bbox in 2 m units
        uint32_t first;                  // into pts_ (quarter-metre integers)
        uint16_t n;
        uint8_t type;
        uint32_t link0, link1;
        uint8_t side0, side1;
        uint32_t flags;                  // +0x10 high bits: 25 excluded, 26 no end0->end1, 27 no end1->end0,
                                         // 28 rigid, 29 blocked (AI cost), 30 highway network
    };
    static constexpr uint32_t kExcluded = 1u << 25, kNoFwd = 1u << 26, kNoRev = 1u << 27, kRigid = 1u << 28,
                              kBlocked = 1u << 29, kHighway = 1u << 30;

    size_t size() const { return recs_.size(); }
    const Rec& rec(size_t i) const { return recs_[i]; }
    std::array<float, 2> point(uint32_t r, uint32_t k) const {
        const auto& q = pts_[recs_[r].first + k];
        return {float(q[0]) * 0.25f, float(q[1]) * 0.25f};
    }

    // 0x6d19f0: the nearest record within r (no gulches, no excluded records; `types` filters when
    // non-empty): returns the squared distance, or -1 when none. k = edge index, q = closest point.
    float nearestRoad(float x, float z, float r, int& rec, int& k, float& qx, float& qz,
                      const std::vector<int>& types = {}) const;
    // 0x6d1d70: the record under (x, z) (within 30 m and d^2 <= flatHalf(type)^2), -1 off roads.
    int roadAt(float x, float z, const std::function<float(int type)>& flatHalf) const;

    // Cost of `count` points of record `r` from point `off`, `dir` 1 = in point order (0x6d7e70 / 0x4ef600).
    using CostFn = std::function<float(uint32_t r, uint32_t off, uint32_t count, int dir)>;
    float lengthCost(uint32_t r, uint32_t off, uint32_t count) const;  // 0x4ae340
    // 0x4ef600 with ctx = c[0..5] speeds (tarmac, forest/ATV, double/bump, trails 4/5, uphill, downhill)
    // and forbid3 (+0x1c): the AI's cost. `height` = the terrain height (0x445570).
    float aiCost(uint32_t r, uint32_t off, uint32_t count, int dir, const std::array<float, 6>& c, bool forbid3,
                 const std::function<float(float x, float z)>& height) const;

    struct Route {
        bool valid = false;
        std::vector<uint32_t> list;  // intermediate records in travel order (+0x08)
        int startRec = -1, startK = -1, endRec = -1, endK = -1;
        float spx = 0, spz = 0, epx = 0, epz = 0;  // snapped start / end points
        bool startDir = false, endDir = false;     // +0x30 / +0x44
        float cost = 0;                            // +0x48
    };
    // 0x6d8e20 + 0x6d8ee0(full = 1): snaps both ends within 1000 m and searches (0x6d7f90).
    bool request(Route& route, float sx, float sz, float ex, float ez, const CostFn& cost) const;
    // The route as a polyline (the path 0x6da240 walks): start point, records, end point. `recOf` gets the
    // record each point belongs to.
    std::vector<std::array<float, 2>> polyline(const Route& route, std::vector<uint32_t>* recOf = nullptr) const;

    bool inEnd1Ring(uint32_t a, uint32_t b) const;  // 0x6d77c0

private:
    std::vector<Rec> recs_;
    std::vector<std::array<int32_t, 2>> pts_;
    // StockData grid (shift 7): 128 x 128 cells over the records' bbox.
    float gx0_ = 0, gz0_ = 0, invX_ = 1, invZ_ = 1;
    std::vector<uint32_t> cellStart_, items_;
    std::vector<uint16_t> cellCount_;

    float segDist2(uint32_t r, float px, float pz, float rad, int& k, float& qx, float& qz) const;  // 0x6d1520
    bool search(Route& route, const CostFn& cost) const;  // 0x6d7f90
    bool sameRecordDir(uint32_t r, int ks, int ke, float sx, float sz, float ex, float ez) const;  // 0x6d7820
};

}  // namespace fuel
