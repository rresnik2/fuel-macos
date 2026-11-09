// P75: the hub zones of the Map page (GenWorld_Z body tail, read by GenWorld_Z::Load 0x77c690; rings by
// Zone_Normalize 0x6ee8a0 / Zone_Normalize_All 0x6eead0). notes/p75-work/B.md sec. 5.
#pragma once

#include <array>
#include <string>
#include <vector>

namespace fuel {

class DpcFile;

struct HubZone {
    std::string name;                                // char[0x20], the hub's name ("YellowStone", ...)
    std::vector<std::array<int, 2>> edges;           // the zone's edges as point-index pairs (world +0xa8 entries)
    std::vector<std::array<float, 2>> ring;          // Zone_Normalize's ring (x, z), closing duplicate dropped
    float minX = 0, minZ = 0, maxX = 0, maxZ = 0;    // +0x30..+0x3c, clamped to +-65536
};
struct HubZones {
    std::vector<std::array<float, 2>> points;        // world +0xa0 (x, z)
    std::vector<HubZone> zones;                      // world +0xb0
    // Zone_FindContaining 0x6ee810: the first zone whose bounds and ring contain (x, z); -1 none.
    int find(float x, float z) const;
};

// Empty when the world has no GenWorld_Z or the tail does not parse.
HubZones loadHubZones(const DpcFile& world);

// 0x60a7c0's ear clipping of a ring (screen points): triangles as index triples.
std::vector<std::array<int, 3>> earClip(const std::vector<std::array<float, 2>>& ring);

}  // namespace fuel
