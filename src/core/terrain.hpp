// The world's base heightfield, stored in USA1.DPC.
//
// GenWorld_Z lists Binary_Z maps; the first is a 4668x4668 grid of 12-bit
// heights covering a 140 km square centred on the origin (about 30 m
// spacing, 10000 m for the full 12-bit range). The original engine adds
// procedural detail on top at run time; the port renders it its own way.
#pragma once

#include "dpc.hpp"

#include <cstdint>
#include <vector>

namespace fuel {

struct Heightfield {
    uint32_t width = 0, height = 0;  // samples; column = x, row = z
    float spacing = 0;               // metres between samples
    float originX = 0, originZ = 0;  // world position of sample (0, 0)
    float metresPerUnit = 0;         // height of one raw step
    std::vector<uint16_t> raw;       // row-major

    float sampleHeight(uint32_t col, uint32_t row) const { return float(raw[size_t(row) * width + col]) * metresPerUnit; }
    // Bilinear height at a world position (clamped to the edges).
    float heightAt(float x, float z) const;
    // Cubic B-spline height, the smooth surface the renderer draws (and the
    // original engine samples).
    float smoothHeightAt(double x, double z) const;
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

}  // namespace fuel
