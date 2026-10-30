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
    std::vector<uint8_t> colour;    // sRGB RGBA8, row-major (row = z); see `decoded`
    bool decoded = false;           // colour = FUEL's point colour ctx+0x70, alpha = far-band forest weight
    std::array<float, 3> farColour{};  // palette 57 (sRGB 0..1): the far band's forest colour
    std::vector<uint8_t> surface;   // surface type index per sample
    std::array<uint32_t, kSurfaceTypes> textures{};  // Bitmap_Z per surface type
    // Surface type at world (x, z) (nearest sample, clamped to the map).
    int surfaceAt(double x, double z) const {
        if (size == 0) return 0;
        const double step = 140000.0 / double(size);
        auto idx = [&](double v) { return uint32_t(std::min(double(size - 1), std::max(0.0, std::floor((v + 70000.0) / step)))); };
        return surface[size_t(idx(z)) * size + idx(x)];
    }
};

// The "GenWorld Colors" palette by engine slot (GenWorld+0x47b74 + 16 * slot, p9 G11): rgb 0..1,
// alpha 1 for slots the world defines, 0 for the others (USA1: 9-13).
std::vector<std::array<float, 4>> loadGroundPalette(const DpcFile& world);

// Builds the maps at `size` x `size` (e.g. 2334 = 60 m per sample).
GroundMaps buildGroundMaps(const DpcFile& world, const Heightfield& heights, uint32_t size);

}  // namespace fuel
