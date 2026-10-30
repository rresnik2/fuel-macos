// Trees and bushes, scattered the way FUEL does it (decoded from the game; see
// docs/formats.md "GenWorld_Z procedural layers"):
// - one candidate per 20 m grid cell, jittered 0..18 m by the seeded
//   permutation; trees are kept with the probability given by the forest map
//   (sharpened) minus a fractal clearing noise, bushes are always candidates;
// - rejected near roads, above 3000 m and on slopes over 30 degrees;
// - the species comes from the GenWorld_Z TREES / BUSHES prototypes, each with
//   a "GenWorld Arbre" script: climate ranges (humidity, forest density,
//   stormed / incinerated / sanded / flooded maps, high ground) and a weight;
// - each plant is scaled 6 x (0.5..1) of its mesh, yawed at random, tilted
//   by up to the species' angle and optionally leaned with the ground.
// Foliage textures are grey: the renderer tints them with the local ground colour.
#pragma once

#include "dpc.hpp"
#include "genworld_noise.hpp"
#include "roads.hpp"
#include "surface.hpp"
#include "terrain.hpp"

#include <array>
#include <cstdint>
#include <functional>
#include <vector>

namespace fuel {

struct Species {
    uint32_t full = 0;      // 3-D mesh
    uint32_t impostor = 0;  // distant camera-facing card (0 = none)
    // From the prototype's "GenWorld Arbre" script (fractions, ranges inclusive).
    struct Range {
        float lo = 0, hi = 1;
        bool has(float v) const { return v >= lo && v <= hi; }
    };
    Range humidity, density, stormed, incinerated, sanded, flooded, highGround;
    Range size{0.499f, 1.001f};  // x 6 = metres per mesh unit
    float weight = 1;            // selection weight
    float groundAlign = 0;       // 0 = upright, 1 = follows the terrain normal
    float tilt = 0;              // radians, random lean on two axes
};

struct Vegetation {
    std::vector<Species> trees, bushes;
    // Ground clutter prototypes: small plants for grassy ground, a rock cluster that,
    // shrunk, makes stones on dirt and rock.
    std::vector<uint32_t> tufts, stones;
    // GenWorld maps 7 (forest), 2 (humidity), 14 (stormed), 13 (incinerated),
    // 16 (sanded), 15 (flooded).
    Raster forest, humidity, stormed, incinerated, sanded, flooded;
    Permutation perm{};
    Fractal clearings;  // the noise that opens glades in forests
};

struct Plant {
    uint32_t meshId = 0;
    std::array<float, 3> position{};
    // Images of the mesh's x, y and z axes (rotation times scale).
    std::array<std::array<float, 3>, 3> axes{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
};

Vegetation loadVegetation(const DpcFile& world);  // throws if the world has no GenWorld_Z

// Plants between `inner` and `radius` metres of (x, z), appended to `out`;
// with `impostors` they use the species' distant cards (bushes are skipped).
// `roads` (optional) keeps them off the road network; `blocked` (optional) says
// whether placed objects come within 5 m of a spot (the engine's last test).
using BlockedQuery = std::function<bool(double x, double z)>;
void scatterPlants(const Vegetation& v, const Heightfield& terrain, const RoadNetwork* roads, const BlockedQuery& blocked, float x, float z,
                   float radius, std::vector<Plant>& out, bool impostors = false, float inner = 0);

// Dense, small ground detail (grass tufts, scrub, stones) within `radius` of (x, z),
// picked by the surface type under each spot; none on roads or tarmac. (FUEL draws
// grass as 1 m-grid sprite tufts from its Grass atlas, driven by per-point ground
// data the port does not have; this is an approximation.)
void scatterClutter(const Vegetation& v, const Heightfield& terrain, const SurfaceQuery& surface, float x, float z, float radius,
                    std::vector<Plant>& out);

}  // namespace fuel
