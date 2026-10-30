// Trees and bushes. FUEL scatters them procedurally: at most one plant per
// jittered 20 m grid cell, kept with a probability from the forest map
// (sharpened, minus a clearing noise), never on slopes over 30 degrees or
// above the treeline. The species prototypes are the GenWorld_Z TREES and
// BUSHES groups (Node_Z -> Lod_Z -> [full mesh, impostor]). The game's jitter
// table and species weights are not decoded; a hash stands in for them.
// Foliage textures are grey: the renderer tints them with the local ground colour.
#pragma once

#include "dpc.hpp"
#include "surface.hpp"
#include "terrain.hpp"

#include <array>
#include <cstdint>
#include <vector>

namespace fuel {

struct Species {
    uint32_t full = 0;      // 3-D mesh
    uint32_t impostor = 0;  // distant camera-facing card (0 = none)
};

struct Vegetation {
    std::vector<Species> trees, bushes;
    // Ground clutter prototypes: small plants for grassy ground, a rock cluster that,
    // shrunk, makes stones on dirt and rock.
    std::vector<uint32_t> tufts, stones;
    Raster forest, humidity;  // GenWorld maps 7 and 2
};

struct Plant {
    uint32_t meshId = 0;
    std::array<float, 3> position{};
    float yaw = 0, scale = 1;
};

Vegetation loadVegetation(const DpcFile& world);  // throws if the world has no GenWorld_Z

// Plants between `inner` and `radius` metres of (x, z), appended to `out`;
// with `impostors` they use the species' distant cards (bushes are skipped).
void scatterPlants(const Vegetation& v, const Heightfield& terrain, float x, float z, float radius, std::vector<Plant>& out,
                   bool impostors = false, float inner = 0);

// Dense, small ground detail (grass tufts, scrub, stones) within `radius` of (x, z),
// picked by the surface type under each spot; none on roads or tarmac.
void scatterClutter(const Vegetation& v, const Heightfield& terrain, const SurfaceQuery& surface, float x, float z, float radius,
                    std::vector<Plant>& out);

}  // namespace fuel
