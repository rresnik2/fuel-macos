// Static world content: every mesh a world DPC places through its Node_Z.
#pragma once

#include "dpc.hpp"

#include <array>
#include <cstdint>
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
