// FUEL's procedural world objects: AutoMesh (houses, sheds, rocks, debris, signs, wrecks on the
// open ground and on flat surfaces) and RoadMesh (guard rails, road debris, highway truck ramps
// along the roads). Decoded from FUEL.exe by P31 (out-B section 2) and p36
// (~/re-work/fuel/notes/p36-objects.md):
//  - rules: CommonComponent/AutoMesh/AutoMesh.txt and RoadMesh.txt, ADDMESH lines (0x78ce40,
//    0x78a450): four lists, class 0 (template radius < 10 m), 1 (10..1000 m), 2 (OnTerraMiddle*:
//    buildings on flat surfaces), 5 (RoadMesh);
//  - templates: the rule name is the CRC of a node OBJECT's name, found depth-first through the
//    object-less group nodes of the library WorldRefs (0x679ec0 / 0x65eca0); the spawn clones the
//    template's node tree to the placement (0x778650) and seats FITTOGROUND / GROUNDNORMAL nodes;
//  - placement: per decor quadtree leaf of size C (cA_MeshObject::Generate 0x78d390), at most one
//    object per 20 m cell (class 0, C < 100) or 50 m cell (classes 1 and 2, C < 400) picked by
//    AutoMesh_PlaceInCell 0x78b6b0, and road objects along the GwRoad lines (C < 200, 0x78e2c0 /
//    0x78d7c0); objects dropped by the emit cull (0x78cbf0) for big chunks.
#pragma once

#include "dpc.hpp"
#include "road_table.hpp"
#include "world.hpp"

#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <unordered_map>
#include <unordered_set>
#include <vector>

namespace fuel {

class GroundGenerator;
class RoadSampler;
struct Heightfield;
struct RoadNetwork;

// One ADDMESH rule (the 0xe4-byte record of 0x78a450; field offsets in the comments).
struct AutoMeshRule {
    std::string name;            // +0x00
    uint32_t crc = 0;            // +0x40 (asoboCrc of the name)
    int cls = 0;                 // +0x44
    int index = 0;               // +0x48: index within its list; +0x4c = index * 0.02597731
    float radius = 0;            // +0x50 template bounding radius (0x67a060)
    float viewDist = -1;         // +0x54 (0x67a1e0): max over the template nodes of K' s v f + 20, -1 = none
    bool onHub = false, noHub = false, inDeepWater = false, inShallowWater = false, onLakeBeach = false;
    bool farFromRoad = false, onTerra = false, closeToRoad = false, onRockyArea = false, gulchSide = false;
    bool roadMiddle = false, groundNormal = false, paralelRoad = false, downHill = false, cultivatedField = false;
    bool country = false, town = false, city = false, onATVPark = false, takeStoneColor = false;
    bool onStormed = false, onIncinerated = false, onFlooded = false, onSanded = false, onSnowed = false;
    bool noSnowed = false, noStormed = false, noIncinerated = false, noFlooded = false, noSanded = false;
    bool onFrozen = false, onDesert = false, onMountain = false, noFrozen = false, noDesert = false, noMountain = false;
    bool onForest = false, noForest = false, randomRot = false, noOcclusion = false, atTheWind = false;
    float maxDistToRoad = 1e8f, minDistToRoad = 0;       // +0x88, +0x8c
    float pente = 2.0f, trou = 2.0f;                       // +0x90, +0x94 (GroundMaxPente / Trou x 0.01)
    float probability = 0.1f;                              // +0x98
    float frequency = 100, phase = 0, crossingDist = 20;   // +0x9c, +0xa0, +0xa4
    float sideDist = 100000, middleDist = 100000;          // +0xa8, +0xac
    float extMin = 0, extMax = -1e6f, intMin = 0, intMax = -1e6f;  // +0xb0..+0xbc
    float terraMiddleMax = 0, terraMiddleMin = 0;          // +0xc0, +0xc4
    float humidityMin = 0, humidityMax = 1, roughnessMin = 0, roughnessMax = 1;  // +0xc8..+0xd4
    bool linePosition = false, shortCut = false;           // +0xd8, +0xe3
    uint32_t roadTypes = 0;  // RoadMesh road-class keywords as a mask of GwRoad types
};

// Polyline_Offset 0x6f7360 (called by RoadChain_OffsetCurve 0x6f8f80 from 0x78e2c0): every point moved by d along
// the left normal (-dz, dx) of the central-difference direction (end points: their one segment); only the first 512
// points (0x6f7360's buffer). 0x78e2c0 passes d = side x (half width + SideDist), or MiddleDist.
std::vector<std::array<double, 2>> offsetPolyline(const std::vector<std::array<double, 2>>& p, double d);
// PK32: the RoadMesh ShortCut search 0x78e030 (rule keyword ShortCut, byte +0xe3): from polyline point `start`, stepping
// `step` (+1 / -1) up to the last point (count - 1) or point 0, the nearest point Q with 50 < |Q - (px, pz)| < 300 whose
// along-road length from the sample is more than 1.5 x that straight distance (the road loops back); stops once the
// straight distance passes 300. Returns false when there is none.
bool roadShortCut(const std::vector<std::array<double, 2>>& pts, int start, int count, int step, double px, double pz, double& qx, double& qz);

// P94: whether FUEL holds the AutoMesh / RoadMesh object of `cell` for a camera at (camX, camZ): the decor leaf
// holding the cell is a chunk below cell.maxC and passes the rule's emit cull (0x78cbf0 / 0x78e2c0).
bool autoCellHolds(const PlacedMesh::AutoCell& cell, double camX, double camZ);

class AutoMesh {
public:
    // Reads the two rule files under `commonComponent` and resolves the templates in USA1.
    AutoMesh(const std::string& commonComponent, const DpcFile& usa1, const std::vector<const DpcFile*>& lookup);
    bool valid() const { return !lists_[0].empty() || !lists_[3].empty(); }
    size_t ruleCount(int list) const { return lists_[size_t(list)].size(); }
    size_t templateCount() const { return templates_.size(); }
    const std::vector<AutoMeshRule>& rules(int list) const { return lists_[size_t(list)]; }
    // P84: the nodes of the template a rule (name CRC) spawns, at the template's own matrices; empty when the rule
    // has no template in USA1.
    std::vector<PlacedMesh> templateParts(uint32_t crc) const;

    struct Context {
        const Heightfield* terrain = nullptr;     // with detail (ground 0x64e810, flat surfaces)
        const GroundGenerator* ground = nullptr;  // maps, ground point, permutation (W+0x704)
        const RoadSampler* roads = nullptr;       // nearest road 0x6fb3f0 / edge 0x6fac40
        const RoadNetwork* lines = nullptr;       // GwRoad polylines for RoadMesh
        const RoadTypeTable* types = nullptr;     // g_RoadTypes (+0xc8, +0xcc half width)
        // World callback W[0] (object clearance): true when (x, z) with radius r is free.
        std::function<bool(double x, double z, float r)> free;
    };

    // The objects FUEL has spawned for a camera at (x, z): every cell of every decor leaf chunk
    // that generates them, after the emit cull. Cells are cached (placement is deterministic), so
    // later calls only compute new cells. Thread: one caller at a time.
    // `roadFrom` (optional) receives the index in `out` where the road objects start.
    // `collect` (optional, P44) receives the fuel barrels among the cell spawns: rules IT_Fuel01 / IT_Fuel02,
    // the IT_DYNCOLLECT1_G / IT_DYNCOLLECT2_G collectibles of clone.tsc (0x51d900: +0xa0 = 100, 0x51dad0: 400),
    // with the range of `out` holding their meshes.
    struct Collectible {
        double x = 0, y = 0, z = 0;
        float amount = 0, radius = 0;  // radius: the rule's template radius (+0x50)
        size_t first = 0, count = 0;
    };
    // P49: the spawns whose template is an IT_DECORMESH_G with a sound of its own (bank symbol 0: the burnt-forest
    // fires DebrisFeu_*; IT_DECORMESH_G activation 0x516cb0 registers the node with 5 / 50 m). `decor` (optional)
    // receives their origin and the rule's name CRC; the rules are given by setDecorSoundRules.
    struct DecorSound {
        double x = 0, y = 0, z = 0;
        uint32_t crc = 0;
    };
    void setDecorSoundRules(std::unordered_set<uint32_t> crcs) { decorSound_ = std::move(crcs); }
    // P94: `slack` (metres) makes the result a superset: every object FUEL holds for a camera anywhere within
    // `slack` of (camX, camZ) (the chunk test with the camera that much nearer). Each object carries its cell
    // (PlacedMesh::autoCell) so that the renderer keeps exactly FUEL's set for the camera of the frame.
    void generate(const Context& ctx, double camX, double camZ, std::vector<PlacedMesh>& out, size_t* roadFrom = nullptr,
                  std::vector<Collectible>* collect = nullptr, std::vector<DecorSound>* decor = nullptr, double slack = 0);
    struct Stats {
        size_t cells = 0, cellObjects = 0, roadObjects = 0, parts = 0;
    };
    Stats lastStats() const { return stats_; }

private:
    struct Part {
        PlacedMesh mesh;            // world = the template node's matrix (row vectors)
        bool fit = false, normal = false;
        float radius = 1;           // node+0x6c (GROUNDNORMAL span max(r, 1))
    };
    struct Template {
        std::array<double, 16> rootInverse{};
        std::vector<Part> parts;
    };
    std::unordered_set<uint32_t> decorSound_;
    std::vector<Template> templates_;
    std::unordered_map<uint32_t, int> templateOf_;  // rule crc -> templates_ index (-1 none)
    std::array<std::vector<AutoMeshRule>, 4> lists_;  // class 0, 1, 2, RoadMesh (5)

    struct Spawn {
        int16_t list = -1, rule = -1;
        std::array<double, 16> m{};  // record matrix (row vectors)
    };
    struct Cell {
        Spawn s;
    };
    std::unordered_map<uint64_t, Cell> cellCache_[3];
    std::unordered_map<uint64_t, std::vector<Spawn>> roadCache_;  // per GwRoad line and side
    Stats stats_;

    void placeInCell(const Context& ctx, int list, double cx, double cz, double cell, Spawn& out) const;
    void placeAlongRoad(const Context& ctx, uint32_t line, int side, std::vector<Spawn>& out) const;
    void emit(const Context& ctx, const Spawn& s, std::vector<PlacedMesh>& out) const;
};

}  // namespace fuel
