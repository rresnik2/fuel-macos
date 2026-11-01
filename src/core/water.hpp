// FUEL's lakes (AGW_TerraBigLake, FUEL.exe 0x65a670 / 0x6ffdd0), decoded in
// ~/re-work/fuel/out/p22/water.md (sections 1.4, 1.5, 2.1) and notes p22.
//
// USA1's world file has no lake table, so the engine adds 19 lakes from code: 8 flooded
// basins (a 100 m cell mask flood-filled from a seed point over the cells holding ground
// below the lake level) and 11 small square ponds. The lake's water surface is level - 2 m.
// The lake positions are read from the player's FUEL.exe at run time (the floats the
// generator 0x6ffdd0 loads), never copied into the port.
// Rivers (GULCH lines) also hold water, at a level that depends on the gulch carve of the
// terrain height (TerrainDetail::river, 0x7a07d0); waterAt combines both (0x6acf80).
#pragma once

#include "dpc.hpp"

#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <vector>

namespace fuel {

struct Lake {
    float x = 0, level = 0, z = 0, radius = 0;
    bool flood = false;         // flood-filled basin (0x6ffc90) or square pond (0x6ffd20)
    float minX = 0, minZ = 0, maxX = 0, maxZ = 0, cell = 0, inv = 0;
    int32_t w = 0, h = 0;
    std::vector<uint32_t> mask;  // bit i + w*j (DataLake +0x3c)
    bool contains(double px, double pz) const;  // 0x659bc0
};

class Lakes {
public:
    // `baseHeight` = the slot-0 B-spline of the height map (0x445570), used by the flood fill.
    Lakes(const DpcFile& world, std::function<double(double, double)> baseHeight);
    bool valid() const { return !lakes_.empty(); }
    const std::vector<Lake>& lakes() const { return lakes_; }
    // max over lakes containing (x, z) of level - h (0 if none): BigLake_WaterQuery 0x65a480.
    double depthAbove(double x, double z, double h) const;
    // The water surface (level - 2 m) of the lake containing (x, z) with the deepest water, or
    // -1e9 when no lake holds the point.
    double surfaceAt(double x, double z, double h) const;

private:
    std::vector<Lake> lakes_;
};

// The per-point water terms the ground generator uses (water.md 1.5/1.6, lakes only):
// depth = ctx+0x1ac, base = ctx+0x1b0 (2 m), fade = ctx+0x1b8.
struct WaterPoint {
    float depth = 0;      // ctx+0x1ac
    float base = 2.0f;    // ctx+0x1b0
    float fade = 1.0f;    // ctx+0x1b8: darkens wet shores (0.67 under water)
    float riverShare = 0; // ctx+0x30c
    bool under() const { return depth - base > 0; }
};
WaterPoint waterAt(const Lakes& lakes, double x, double z, double h);
// GenWorld_WaterQuery 0x6acf80 with rivers (water.md 1.2-1.5): river first (0x7a07d0 over the
// gulch carve, TerrainDetail::river), then the lakes. Either pointer may be null.
class TerrainDetail;
WaterPoint waterAt(const Lakes* lakes, const TerrainDetail* detail, double x, double z, double h);
// 0x6b9560 tail: the wet-shore fade ctx+0x1b8 from ctx+0x1ac / ctx+0x1b0.
float waterFade(float depth, float base);

}  // namespace fuel
