// FUEL's per-point ground generator (GenWorld "point context", FUN_006bcfd0 and
// its lazy property functions), as far as the port needs it: grass amount and
// size for the grass tufts, rock cover and farm fields for the tree filter.
// Decoded from FUEL.exe; see ~/re-work/fuel/notes/p14-grass-ground.md (G1-G12)
// for addresses and for what is approximated (water, the road profile table,
// the coarse normal sampler, humidity noise).
#pragma once

#include "dpc.hpp"
#include "genworld_noise.hpp"
#include "road_segments.hpp"
#include "water.hpp"
#include "roads.hpp"
#include "terrain.hpp"

#include <array>
#include <cstdint>
#include <unordered_map>
#include <vector>

namespace fuel {

struct GroundPoint {
    float height = 0;
    float normal[3] = {0, 1, 0};
    float humidity = 0;         // Underground Humidity (map 2)
    float surfaceHumidity = 0;  // Surface Humidity: drier on slopes
    float road = 0;             // road weight 0..1 (1 on the road surface)
    float cover = 0;            // flat-surface coverage (ctx+0x2e0)
    float highGround = 0;       // snow line factor minus road (ctx+0x19c)
    float rock = 0;             // rock cover (ctx+0x1e0)
    float rockSlope = 0;        // rock cover with steep slopes (ctx+0x1e4)
    float tarmac = 0;           // tarmac share of the road (ctx+0x1f4; port approximation)
    int field = 0;              // farm field type: 0 none, 1 ploughed, 2 crops (ctx+0x2e8)
    float fieldRandom = 0;      // the field cell's random byte / 255 (picks the crop colours)
    float grass = 0;            // Grass (ctx+0x1c8)
    float groundGrass = 0;      // GroundGrass (ctx+0x1cc): what the grass tufts use
    float grassSize = 0;        // ctx+0x234
    // Road fields (bit 12 = 0x6b9bb0 over the road sampler 0x6fbab0; zero without a RoadSampler).
    float roughness = 0;        // ctx+0x1f0: dirt road roughness (picks DIRT_FLAT .. DIRT_ROUGH)
    float decal = 0;            // ctx+0x204: road decal weight
    float decalU = 0, decalV = 0;  // ctx+0x1f8 / 0x1fc: decal uv (across, along)
    uint16_t decalMaterial = 0xfff;  // ctx+0x2dc: GenWorld texture enum of the decal (0xfff none)
    float puddle = 0;           // ctx+0x208
    float junction = 0;         // ctx+0x238
    float roadEdge = 0;         // ctx+0x21c (berm beside the road)
    float stones = 0;           // ctx+0x1dc "Stones" (bit 24, dry land)
    float caillou = 0;          // ctx+0x1d8 "Caillou" (pebbles; only under water)
    float sand = 0;             // ctx+0x220 "SandTexture" (bit 22; from the Dune layer)
    // Surface types (bit 21 = 0x6bb670): the four strongest of the 40 ground textures and their
    // weights (normalised by the sum of every weight > 0.025); type 0xff = none.
    // Water (bit 6 = 0x6b9560 over the lake query; rivers not yet): ctx+0x1ac, 0x1b0, 0x1b8.
    float waterDepth = 0, waterBase = 2, waterFade = 1;
    bool underWater() const { return waterDepth - waterBase > 0; }
    bool nearRoad = false;      // ctx+0x1bc <= 1 (inside a road band)
    uint8_t surfaceType[4] = {0xff, 0xff, 0xff, 0xff};
    float surfaceWeight[4] = {0, 0, 0, 0};
};

// The point's ground colours (sRGB 0..1, as the palette stores them), decoded from the
// point evaluator's colour bits (see ~/re-work/fuel/notes/p16-colours.md C1-C12).
struct GroundColours {
    float base[3] = {0, 0, 0};  // ctx+0x80 = ctx+0x70 on dry land: terrain near/mid bands, tufts off the road
    float far[3] = {0, 0, 0};   // ctx+0x90: terrain beyond 600 m chunks (forest floor and snow tint)
    float road[3] = {0, 0, 0};  // ctx+0x50: road colour (tufts on the road); base where road <= 0.025
    float occlusion = 1;        // ctx+0x230 (map 9; the engine also shades it under near trees)
    float forest = 0;           // k: weight of the far band's forest colour (palette 57)
    float water[3] = {0, 0, 0}; // ctx+0x60: water colour (palette 0 mixed with the earth colour under water)
};

// One grass tuft as Grass_EmitTuft (0x7dce50) builds it.
struct GrassTuft {
    float x, y, z;      // base (sunk on slopes)
    float size;         // card height in metres (width 1.5 x size)
    uint8_t tile;       // cell of the 4x4 Grass atlas
    uint8_t windAmp;    // (r2/255 + 1) * 0.5 * 2.25, stored x 100
    uint8_t tint;       // road weight x 255 (the engine lerps two ground colours by it)
    uint8_t occlusion;  // clamp(0.8 a + 0.2) x 255, a = ctx+0x230 (sky light occlusion in the shader)
    uint8_t rgb[3];     // lerp(ctx+0x80, ctx+0x50, road), sRGB bytes
    uint8_t pad2 = 0;
};

class GroundGenerator {
public:
    GroundGenerator() = default;
    // `terrain` is the levelled heightfield; `roads` may be null.
    GroundGenerator(const DpcFile& world, const Heightfield& terrain, const RoadNetwork* roads);
    bool valid() const { return terrain_ != nullptr; }
    // FUEL's road segments (point bit 12 reads them); without it the port's road widths are used.
    void setRoadSampler(const RoadSampler* s) { sampler_ = s; }
    void setLakes(const Lakes* l) { lakes_ = l; }
    const Lakes* lakes() const { return lakes_; }
    const RoadSampler* roadSampler() const { return sampler_; }

    // Evaluates the point (x, z). With `full` false only what the tree filter needs.
    GroundPoint at(double x, double z, bool full = true) const;
    // The point's colours (needs `p` from at(x, z) with full = true). `t0` overrides the
    // palette-half noise T0 (>= 0: e.g. its mean over a map cell).
    GroundColours colours(double x, double z, const GroundPoint& p, float t0 = -1) const;
    // Mean of T0 over a square of side `size` centred on (x, z) (n x n samples).
    float meanT0(double x, double z, double size, int n) const;
    bool hasPalette() const { return palette_.size() >= 123; }
    std::array<float, 3> paletteColour(int slot) const {
        return {palette_[size_t(slot)][0], palette_[size_t(slot)][1], palette_[size_t(slot)][2]};
    }
    // Trees_Excluded (0x7831e0), the part the port has: inside a flat surface that
    // allows no plants (byte +0x81 == 0). Lakes, rivers and zones are not tested.
    bool excludesPlants(double x, double z) const;
    // Trees_Filter / Bushes_Filter tests that use the point: rock cover > 0.1, farm field.
    bool rejectsPlant(double x, double z) const;

    // Grass_Build + Grass_Decide + Grass_EmitTuft over the 1 m grid points with
    // integer coordinates in [x0, x1) x [z0, z1); appends to `out`.
    void grass(int32_t x0, int32_t z0, int32_t x1, int32_t z1, std::vector<GrassTuft>& out) const;

    const Permutation& permutation() const { return perm_; }
    // Bitmap_Z of the Grass material's 4x4 tuft atlas (0 if not found).
    uint32_t grassAtlas() const { return grassAtlas_; }

private:
    struct Surface {
        double cx, cz, ax, az, lenX, bx, bz, lenZ;
        float invMargin;
        float grassCut, grassNoise, grassAdd, w300, w9c;  // file +0x54, +0x58, +0x5c, +0x60, ? (see notes G4)
        bool ellipse, allowPlants;
        double radius;
    };
    struct SurfaceWeights {
        float cover = 0, cut = 0, noise = 0, add = 0, keep = 1;
    };
    SurfaceWeights surfaceWeights(double x, double z) const;
    double insideDistance(const Surface& s, double x, double z) const;
    float edgeJitter(double x, double z) const;
    float roadWeight(double x, double z, bool* nearRoad, float* tarmac = nullptr) const;
    void surfaceTypes(double x, double z, GroundPoint& p) const;
    void rockCover(double x, double z, GroundPoint& p, float coarseNy) const;
    int fieldType(double x, double z, const GroundPoint& p, float coarseNy, bool nearRoad, float cover, float* random) const;
    float mapAt(int slot, double x, double z) const;

    const Heightfield* terrain_ = nullptr;
    const RoadNetwork* roads_ = nullptr;
    const RoadSampler* sampler_ = nullptr;
    const Lakes* lakes_ = nullptr;
    Permutation perm_{};
    std::array<Raster, 31> maps_{};
    std::vector<Surface> surfaces_;
    std::unordered_map<uint64_t, std::vector<uint32_t>> surfaceCells_;  // 64 m cells
    // Noise bank (genWorld+0x4498) and terrain-detail object T, by their offsets.
    Fractal f1fac_, ffd8_, f2f80_, ffd44_, f7ea4_, f6ed0_, edgeA_, edgeB_;
    Fractal t0c_, t18_, t24_, t28_;
    Fractal f3f54_;                                 // surface variant noise (0x6a81f0)
    Fractal f0004_, f9e4c_, fcdc8_;                 // colour noises (p16 C8, C11)
    std::vector<std::array<float, 4>> palette_;    // GenWorld Colors by slot
    std::vector<float> tileWeight_, tileSize_;
    uint32_t grassAtlas_ = 0;  // FUEL.exe 0x9ff02c / 0x9ff04c
};

}  // namespace fuel
