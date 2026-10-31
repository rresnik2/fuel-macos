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
bool isTarmacRoad(int roadType);
// GwRoad type of the road under (x, z) (within half its drawn width of the centre line), -1 off any road.
// The AI's road lookup (0x6d1d70 in IA speed model 0x617000) is not decoded; this is the port's stand-in.
int roadTypeAt(const RoadNetwork& net, double x, double z);  // types 10, 11, 12, 15, 18, 19

RoadNetwork loadRoads(const DpcFile& world);

// Bitmap_Z of the GenWorld texture enum `e` (GenWorld_Z materials list, p9 R8: 0x2a..0x2e
// Road01..05, 0x2f..0x33 RoadStop01..05, 0x34 Dirt01, 0x37 Water); 0 when absent.
uint32_t genWorldEnumTexture(const DpcFile& world, uint32_t e, int slot = 0);  // throws if the world has no GenWorld_Z/GwRoad_Z

}  // namespace fuel
