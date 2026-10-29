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
};

// Meshes placed by the nodes of `dpc`, at the highest level of detail: a
// node's object is a Mesh_Z, or a Lod_Z whose first object is a Mesh_Z or a
// Skin_Z (drawn with all its meshes). Objects are resolved in `lookup` order,
// so a sub-world can reference meshes stored in USA1.DPC.
std::vector<PlacedMesh> collectPlacedMeshes(const DpcFile& dpc, const std::vector<const DpcFile*>& lookup);

}  // namespace fuel
