// Static world content: every mesh a world DPC places through its Node_Z.
#pragma once

#include "dpc.hpp"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace fuel {

struct PlacedMesh {
    uint32_t meshId = 0;
    uint32_t nodeId = 0;
    std::array<float, 16> world{};  // row-vector convention (translation in 12..14)
    // The node's UserDefine_Z says FITTOGROUND / GROUNDNORMAL. Informational: the engine
    // applies these keywords only to AutoMesh spawns (0x778650), never to static placements,
    // which keep their authored matrix (p35 F2).
    bool fitToGround = false, groundNormal = false;
    // Per-object level of detail and fade (Lod_Z::Draw 0x884170, ComputeFade 0x672950, Mesh_Z::Draw
    // 0x80a280; decode p6 W7/W8, P31 out-F 1.5, p36). levels[k] = the mesh of this slot at level k
    // (levels[0] == meshId); levelView[k] = that Mesh_Z's own view distance (link +88, 0 = none).
    // lod: the node object is a Lod_Z with base distance `baseDistance` (Lod+0x84), view distance
    // `viewDist` (Lod link +88), nearFade/hardCull (Lod+0x7c/+0x80); otherwise a bare Mesh_Z.
    // lodScale = node+0x44.
    std::vector<uint32_t> levels;
    std::vector<float> levelView;
    bool lod = false;
    float baseDistance = 1, viewDist = 0, nearFade = 0, hardCull = 0, lodScale = 1;
};

// The view's LOD factors in FUEL's main view (p36): Lod_Z::Draw uses x = d / (K D s) with
// K = display+0x1094 and ComputeFade e = viewDist K' s with K' = display+0x1140 (both 10.0, set in
// the display constructor 0x64faf0 at 0x64ffa4/0x64ffb0, copied into the view by 0x651810), and
// s = node+0x44 * view+0xec, view+0xec = viewport+0x30 = 0x663790(fov, cam+0x1324, cam+0x1328)
// = tan(40 deg / 2) / tan(85 deg / 2) with the game camera's +0x1324 = 40 deg (0xa0667c) and
// +0x1328 = 85 deg (0x9dd0ac, set at 0x415a66).
struct LodView {
    float K = 10.0f, Kfade = 10.0f;
    float viewScale = 0.39720f;  // tan(20 deg) / tan(42.5 deg)
};
// Lod_Z::Draw's level for a camera distance d (0 .. levels-1), and the fade alpha (0..1) of
// ComputeFade for a Lod (or Mesh) with these fields; alpha < 0.01 is not drawn.
int lodLevel(const PlacedMesh& p, float d, const LodView& v = {});
float lodFade(float viewDist, float nearFade, float hardCull, float s, float d, const LodView& v = {});
// The alpha FUEL draws the placement with at distance d: the Lod's fade gates the draw
// (>= 0.01), then the level's Mesh_Z::Draw recomputes it with the mesh's own view distance.
float placementAlpha(const PlacedMesh& p, int level, float d, const LodView& v = {});

// Meshes placed by the nodes of `dpc`, at the highest level of detail: a
// node's object is a Mesh_Z, or a Lod_Z whose first object is a Mesh_Z or a
// Skin_Z (drawn with all its meshes). Objects are resolved in `lookup` order,
// so a sub-world can reference meshes stored in USA1.DPC.
std::vector<PlacedMesh> collectPlacedMeshes(const DpcFile& dpc, const std::vector<const DpcFile*>& lookup);
// The mesh entries of one node of `dpc` (as collectPlacedMeshes makes them); empty if none.
std::vector<PlacedMesh> collectNodeMeshes(const DpcFile& dpc, const std::vector<const DpcFile*>& lookup, uint32_t nodeId);
// USA1.DPC's own placements that FUEL draws (p36): the nodes under the World_Z root node (World_Z
// body u32 0, WorldResources +0x18; ad136f97 in USA1), including the resident WorldRef groups whose
// roots USA1 already links under it. The other trees (the object library under 6e863158 that
// AutoMesh and clone scripts instance, single unlinked nodes) are not in the scene.
std::vector<PlacedMesh> collectWorldRootMeshes(const DpcFile& usa1, const std::vector<const DpcFile*>& lookup);

// A sub-world as USA1.DPC's WorldRef_Z describes it (p6 W2 body grammar; streaming decoded by
// P31 out-F, ported by p35). FUEL keeps three kinds:
//  - region (link flags bit 0x800000, the 19 hub camps and 4 HABILLAGE dressings): loaded by
//    distance, only the two nearest centres within 5000 m of the player (0x57ce60);
//  - course (bit 0x400000 without 0x800000, 58 USA1_CR* + RESETCOURSE): loaded when its mission
//    teleports in (UserDefine STREAM_M_TELEPORT <mission> on the WorldRef's node, 0x513630) and
//    unloaded at M_END;
//  - resident (no command): object groups inside USA1 itself.
struct SubWorldRef {
    enum Kind { Resident, Region, Course };
    Kind kind = Resident;
    uint32_t id = 0, flags = 0;
    std::string file;     // "USA1_CR31_343CC3A6.DPC" (from the OpenBF command), "" for resident groups
    float cx = 0, cz = 0; // body +0xa4.. sixth/fifth float: the centre the region test measures to
    std::string mission;  // STREAM_M_TELEPORT name (courses)
};
std::vector<SubWorldRef> loadSubWorldRefs(const DpcFile& usa1);

// FUEL's region pick (LevelManipulator_G 0x57ce60): indexes of the regions to keep loaded for a
// player at (x, z): sorted by ground-plane distance to their centres, the first two whose distance
// is at most 5000 m.
std::vector<size_t> nearestRegions(const std::vector<SubWorldRef>& refs, double x, double z);

// Triangles of a Mesh_Z in its own space, for collision: positions (xyz per
// vertex) and indices (three per triangle). Empty when the mesh has no
// drawable geometry. Throws on malformed data.
struct CollisionMesh {
    std::vector<float> positions;
    std::vector<uint32_t> indices;
    float radius = 0;  // of the bounding sphere around the origin
};
CollisionMesh loadCollisionMesh(const std::vector<const DpcFile*>& lookup, uint32_t meshId);

}  // namespace fuel
