// Ground colour and surface type over the whole world.
//
// FUEL's 40 ground textures are not colour maps: each packs a normal map
// (R, G, A) and a grey detail (B). The ground's colour comes from a palette
// ("GenWorld Colors") mixed per point from authored geology/climate layers
// (humidity, clay, chalk, volcano, forest, ...). The engine's own 40-weight
// generator is not decoded; the classification here is a port heuristic built
// from those named layers, height and slope.
#pragma once

#include "dpc.hpp"
#include "terrain.hpp"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace fuel {

inline constexpr int kSurfaceTypes = 40;
const char* surfaceName(int index);  // EARTH, ..., GRASS, ..., ROUGHSAND2

struct GroundMaps {
    uint32_t size = 0;              // samples per side, covering the 140 km world square
    std::vector<uint8_t> colour;    // sRGB RGBA8, row-major (row = z)
    std::vector<uint8_t> surface;   // surface type index per sample
    std::array<uint32_t, kSurfaceTypes> textures{};  // Bitmap_Z per surface type
};

// Builds the maps at `size` x `size` (e.g. 2334 = 60 m per sample).
GroundMaps buildGroundMaps(const DpcFile& world, const Heightfield& heights, uint32_t size);

}  // namespace fuel
