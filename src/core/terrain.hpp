// The world's base heightfield, stored in USA1.DPC.
//
// GenWorld_Z lists Binary_Z maps; the first is a 4668x4668 grid of 12-bit
// heights covering a 140 km square centred on the origin (about 30 m
// spacing, 10000 m for the full 12-bit range). FUEL adds procedural detail on
// top at run time (AGW_Altitude, FUEL.exe 0x64a2a0): see terrain_height.hpp.
#pragma once

#include "dpc.hpp"

#include <cstdint>
#include <memory>
#include <vector>

namespace fuel {

class TerrainDetail;
class RoadSampler;

struct Heightfield {
    uint32_t width = 0, height = 0;  // samples; column = x, row = z
    float spacing = 0;               // metres between samples
    float originX = 0, originZ = 0;  // world position of sample (0, 0)
    float metresPerUnit = 0;         // height of one raw step
    std::vector<uint16_t> raw;       // row-major

    // FUEL's procedural ground on top of the map (null for the bare map).
    std::shared_ptr<const TerrainDetail> detail;
    // FUEL's road segments (p22), when the world has them: the ground carves roads with them
    // (0x6fcaa0) and the decals sample them.
    std::shared_ptr<RoadSampler> roads;

    float sampleHeight(uint32_t col, uint32_t row) const { return float(raw[size_t(row) * width + col]) * metresPerUnit; }
    // Bilinear height of the raw samples at a world position (clamped to the edges).
    float heightAt(float x, float z) const;
    // Cubic B-spline of the map alone (the engine's base height h0).
    float baseHeightAt(double x, double z) const;
    // The ground height FUEL uses everywhere (AGW_Altitude 0x64a2a0 with the detail
    // fractals, rock, canyon floors, gulches and flat surfaces) when `detail` is set,
    // else the B-spline of the map.
    float smoothHeightAt(double x, double z) const;
    float groundHeight(double x, double z) const { return smoothHeightAt(x, z); }
};

// One GenWorld map (Binary_Z): a grid of 12-bit values over the same 140 km
// square as the heightfield.
struct Raster {
    uint32_t width = 0, height = 0;
    float maxValue = 1;  // the value that means "full"
    std::vector<uint16_t> raw;
    float at(uint32_t col, uint32_t row) const { return float(raw[size_t(row) * width + col]) / maxValue; }
};
// Decodes a Binary_Z body (4x4 block codec). Throws on malformed data.
Raster decodeRaster(const std::vector<uint8_t>& binaryBody);
// The Binary_Z map IDs listed by the world's GenWorld_Z (index 0 = heights).
std::vector<uint32_t> genWorldMaps(const DpcFile& world);

// Decodes the heightfield of a world DPC (WORLD/USA1.DPC). Throws when the
// file has no GenWorld_Z or the map does not decode.
Heightfield loadWorldHeightfield(const DpcFile& world);
// The heightfield with FUEL's procedural ground attached (`detail`): what the game
// drives on, draws and places objects on.
Heightfield loadLevelledHeightfield(const DpcFile& world);

}  // namespace fuel
