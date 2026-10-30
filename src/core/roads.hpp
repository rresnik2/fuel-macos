// The road network (GwRoad_Z in WORLD/USA1.DPC): 2-D typed polylines. FUEL
// paints and carves roads into its procedural terrain; the port drapes
// textured ribbons over the heightfield instead.
#pragma once

#include "dpc.hpp"

#include <array>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace fuel {

struct RoadStyle {
    float width = 0;       // metres; 0 = not drawn (invisible trails, gulches, bridges)
    uint32_t texture = 0;  // Bitmap_Z, 0 = plain dirt colour
    float opacity = 1;
};

struct RoadNetwork {
    struct Line {
        uint8_t type;  // road type (flag bit 7 removed)
        std::vector<std::array<float, 2>> points;  // x, z in metres
    };
    std::vector<Line> lines;
    std::array<RoadStyle, 32> styles{};
    // 256 m grid cells -> line indices, for nearby queries.
    std::unordered_map<uint64_t, std::vector<uint32_t>> cells;
    static constexpr float kCell = 256.0f;
    static uint64_t cellKey(int32_t cx, int32_t cz) { return (uint64_t(uint32_t(cx)) << 32) | uint32_t(cz); }
    // Proximity grid for surface queries: 64 m cells -> (line << 20 | segment) of every
    // segment that comes within kReach of the cell.
    std::unordered_map<uint64_t, std::vector<uint64_t>> segCells;
    static constexpr float kSegCell = 64.0f, kReach = 16.0f;
};

// Road surface under (x, z): the surface type a road lays down (ASPHALT = 11 on tarmac
// roads, DIRT_FLAT = 13 on dirt roads and trails) when the point is within half the road's
// drawn width of its centre line, else -1. Tarmac wins where roads overlap.
int roadSurfaceAt(const RoadNetwork& net, double x, double z);
bool isTarmacRoad(int roadType);  // types 10, 11, 12, 15, 18, 19

RoadNetwork loadRoads(const DpcFile& world);  // throws if the world has no GenWorld_Z/GwRoad_Z

}  // namespace fuel
