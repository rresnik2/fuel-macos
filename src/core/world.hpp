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
};

// Meshes placed by the nodes of `dpc`, at the highest level of detail: a
// node's object is a Mesh_Z, or a Lod_Z whose first object is a Mesh_Z or a
// Skin_Z (drawn with all its meshes). Objects are resolved in `lookup` order,
// so a sub-world can reference meshes stored in USA1.DPC.
std::vector<PlacedMesh> collectPlacedMeshes(const DpcFile& dpc, const std::vector<const DpcFile*>& lookup);

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
