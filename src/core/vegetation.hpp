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
#include "ground_point.hpp"
#include "roads.hpp"
#include "surface.hpp"
#include "terrain.hpp"

#include <array>
#include <cstdint>
#include <functional>
#include <unordered_map>
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
    // Collision (P50, P52; notes p52-trunks.md). The parser 0x7832c0 writes the record at +0x40 (EDI = record + 0x40,
    // 0x63e9b0); key "25/01/2009 13:52" -> record +0x78 = v x 0.01 (store 0x783a13), default 0.25 (property ctor
    // 0x63e870 slot 0xe, called with EAX = record + 0x40 by 0x6418c0); key "12/11/2007 20:00" -> record +0xf4 as an
    // integer (fistp, RC = chop). The trunk queries (0x785200 trees, 0x788ab0 bushes) skip species with flag 2.
    // Radius: see trunkRadius().
    float trunk = 0.25f;
    uint32_t flags = 0;
};

// Plants stand this far into the ground (the draw 0x784400 / 0x787d00 moves the origin by down x 0.25); FUEL's
// plant point (the scatter cache, 0x785530) is the unsunk ground point, which the trunk queries use.
constexpr float kPlantSink = 0.25f;
// Trunk tops above that ground point: 0x785200 (trees) y + 5, 0x788ab0 (bushes) y + 10.
constexpr float kTreeTrunkTop = 5.0f, kBushTrunkTop = 10.0f;

// PK05: Decor_FreeOfObjects 0x557650, the tree / bush placement callback W[0] (bound at SetGameLogicAgent 0x5576c0):
// free = !GenWorldBoxes 0x6404a0 && Missions_SphereClear 0x4f8a80 && Pois 0x4ed9e0, for a sphere (x, y, z, r).
// Notes: ~/re-work/fuel/notes/pk05.md.
struct PlantClearance {
    // GenWorld_Z's 0x40-byte records (0x77c690 -> DAT_00a673dc, grid DAT_00a673e0): a 3 x 4 matrix (rows 0..2, 4..6, 8..10,
    // position floats 3, 7, 11), half extents floats 12..14, grid radius float 15.
    struct Box {
        std::array<float, 12> m{};
        std::array<float, 3> ext{};
        float radius = 0;
    };
    // One grid item of the mission list (0x4f9080): a start / finish / checkpoint at (x, z). `box` = the grid box half
    // size (start / finish ((aiCount + 1) / 2 + 1) x 10, checkpoints 25), `half` = the oriented test's half size
    // before the sphere radius is added (the same numbers), `road` = the road half width under the gate (20 without).
    struct Gate {
        float x = 0, z = 0, box = 0, road = 20;
    };
    std::vector<Box> boxes;
    std::vector<Gate> gates;
    std::vector<std::array<float, 2>> pois;  // 0x4ed9e0's list: records +8 / +0x10 of the POI table

    // 0x6404a0: true when a GenWorld record box touches the sphere (0x73f460).
    bool hitsBox(float x, float y, float z, float r) const;
    // 0x4f8a80: true when no mission gate blocks the sphere (0x4f8e00: xz distance < road + half + r, the gate
    // chosen by the grid box overlapping (x +- r, z +- r)).
    bool missionsFree(float x, float z, float r) const;
    // 0x4ed9e0: no POI within r + 25 m (xz).
    bool poisFree(float x, float z, float r) const;
    // 0x557650.
    bool free(float x, float y, float z, float r) const { return !hitsBox(x, y, z, r) && missionsFree(x, z, r) && poisFree(x, z, r); }
    void index();  // builds the lookup grid after boxes / gates change

private:
    struct Grid {
        std::unordered_map<int64_t, std::vector<uint32_t>> cells;
    };
    Grid boxGrid_, gateGrid_;
};

struct Vegetation {
    std::vector<Species> trees, bushes;
    PlantClearance clearance;
    // GenWorld maps 7 (forest), 2 (humidity), 14 (stormed), 13 (incinerated),
    // 16 (sanded), 15 (flooded).
    Raster forest, humidity, stormed, incinerated, sanded, flooded;
    Permutation perm{};
    Fractal clearings;  // the noise that opens glades in forests
};

class RoadGraph;
struct RoadTypeParams;
// Fills v.clearance's gates and POIs: every mission FUEL's level holds (the script events of loadEvents, then the
// world's MISSION_COURSE nodes, 0x4f9080 per mission) and poi.tsc's points of interest. `graph` / `types` give the road
// width under each gate (0x6d1d70 -> road type -> [0xa678ac + 0x200 type] = Tcc; 20 m without a road); either may be
// null. Returns the number of gates added.
size_t loadPlantClearance(Vegetation& v, const DpcFile& usa1, const std::string& dataDir, const RoadGraph* graph, const RoadTypeParams* types,
                          size_t typeCount);

struct Plant {
    uint32_t meshId = 0;
    bool tree = true;      // TREES or BUSHES list (several meshes are in both, with different settings)
    uint16_t species = 0;  // 1-based index in that list = FUEL's species record index (0x785530 / 0x788e00)
    std::array<float, 3> position{};
    // Images of the mesh's x, y and z axes (rotation times scale).
    std::array<std::array<float, 3>, 3> axes{{{1, 0, 0}, {0, 1, 0}, {0, 0, 1}}};
};

Vegetation loadVegetation(const DpcFile& world);  // throws if the world has no GenWorld_Z

// Plants between `inner` and `radius` metres of (x, z), appended to `out`;
// with `impostors` they use the species' distant cards (bushes are skipped).
// `roads` (optional) keeps them off the road network; `ground` (optional) applies the
// engine's ground tests (no plants on rocky ground, in farm fields or in flat surfaces
// that forbid them); `blocked` (optional) says whether placed objects come within 5 m
// of a spot (the engine's last test).
// The engine's sphere radius for a plant (the port's old 5 m clearance; 0x557650 gets the sphere from the placement call).
constexpr float kPlantSphereRadius = 5.0f;
// Grass is not a Plant: see GroundGenerator::grass (ground_point.hpp).
using BlockedQuery = std::function<bool(double x, double z)>;

// FUEL's per-plant noise n (0x784370 trees, 0x787c70 bushes: the same code): RT[floor(f32(z x 10.193578 +
// (x + 425.23886) x 12.317513)) & 255] x 0.003921569, from the plant's float position. The draw (0x784400 /
// 0x787d00) takes its size from it and the trunk queries their radius: one noise for both.
double plantNoise(const Vegetation& v, float x, float z);
// Drawn scale of a plant (0x784400 / 0x787d00, the scale passed to 0x782be0): ((1 - n) x size.hi + size.lo x n) x 6.
float plantScale(const Vegetation& v, const Species& s, float x, float z);
// Trunk radius of a plant (0x785200 at 0x78531d / 0x788ab0 at 0x788bd3): r = ((1 - n) x size.hi + size.lo x n) x
// trunk x 6 (x87, 53 bits), stored as a float; 2.0 when r >= 2 = min(trunk x drawn scale, 2). The trunk is the
// vertical segment below (x, ground + kTreeTrunkTop / kBushTrunkTop, z) with this radius (no lower end).
float trunkRadius(const Vegetation& v, const Species& s, float x, float z);
void scatterPlants(const Vegetation& v, const Heightfield& terrain, const RoadNetwork* roads, const BlockedQuery& blocked, float x, float z,
                   float radius, std::vector<Plant>& out, bool impostors = false, float inner = 0, const GroundGenerator* ground = nullptr);

// ---- P94: how FUEL shows a plant as the camera moves (decoded; notes p94-lod-popin.md). Per decor chunk of size C
// (the quadtree leaf holding the plant, from the camera: 0x7df0e0 / 0x7dfdc0, 12288 m roots split while the camera is
// within 2.0625 C of the centre, 24 m leaves) the chunk's objects queue the plant (cA_TreesObject 0x7847b0,
// cA_BushesObject 0x7880b0; a68 = a64 = 60, a8c = 600, bushes in chunks < 100 only):
//   C < 30: the mesh, ground element kind 0;  30 <= C < 60: the mesh (kind 1) and the card (kind 0x16);
//   60 <= C: the card (trees kind 0x14 while C < 600, bushes kind 0x15 while C < 100).
// Each kind draws through ground_rotshape.vhl, whose alpha att = 1 - saturate((|eye - vertex|_xz - min) x k) multiplies
// the texture alpha before the alpha test at 0.5 (the shader's own `tDiffuse.w > 0.5` branch: sprites collapse when
// att <= 0.5). min / k per kind, GenWorld_DrawGroundElements' constant blocks (0x77ac87..0x77ad3a, draw scale
// W+0x5c018 = 1): kinds 0 / 1 132.5 / (1 / 30): the mesh fades out over 132.5..162.5 m; 0x16 147.5 / (-1 / 30): the card
// fades in over 117.5..147.5 m; 0x14 1200 / (1 / 100); 0x15 250 / (1 / 50).
enum class DecorFade : uint8_t { None = 0, Mesh = 1, TreeCard = 2, BushCard = 3 };
// The decor quadtree leaf size at (px, pz) for a camera at (camX, camZ) (also drive.mm decorChunkSize).
double decorLeafSize(double px, double pz, double camX, double camZ);
// att for a plant of `kind` whose pivot is (px, pz), at horizontal distance `dxz` (FUEL: per vertex) from the eye;
// 0 when its chunk does not queue that kind. The renderer's vsMain has the same rule (decorAtt there).
float decorAtt(DecorFade kind, double px, double pz, double eyeX, double eyeZ, float dxz);
// ---- P94 end

}  // namespace fuel
