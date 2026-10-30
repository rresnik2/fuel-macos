// The rider: FUEL's one pilot model (PILOT.DPC), skinned into the riding pose
// of a vehicle archetype and baked into a static mesh in vehicle space.
// See docs/formats.md, "Skinning and the rider".
#pragma once

#include "assembly.hpp"
#include "dpc.hpp"
#include "vehicle.hpp"

#include <array>
#include <cstdint>
#include <map>
#include <string>
#include <vector>

namespace fuel {

namespace cls {
inline constexpr uint32_t Animation = 0x46107D89;
}

// A material of a baked mesh: a Material_Z, with its diffuse (slot 0) and
// specular (slot 4) bitmaps replaced when non-zero (livery).
struct BakedMaterial {
    uint32_t materialId = 0;
    uint32_t diffuseTex = 0, specularTex = 0;
};

struct BakedBatch {
    uint32_t material = 0;  // index into BakedMesh::materials
    uint32_t firstIndex = 0, indexCount = 0;
};

// Static geometry with interleaved 36-byte vertices: float3 position, D3DCOLOR
// tangent @12, D3DCOLOR normal @16, float2 uv @20, float2 uv1 @28.
struct BakedMesh {
    static constexpr uint32_t kStride = 36;
    std::vector<uint8_t> vertices;
    std::vector<uint32_t> indices;  // triangle lists, relative to the first vertex
    std::vector<BakedMaterial> materials;
    std::vector<BakedBatch> batches;
    // Vertices are stored batch after batch; each batch holds the vertices its
    // triangles use, in source order.

    uint32_t vertexCount() const { return uint32_t(vertices.size() / kStride); }
    std::array<float, 3> position(uint32_t i) const;
};

// The AddVehicleInfo archetype (last token) of a vehicle variant such as
// "QUAD_ROAD_2" in VehiclesDef.tsc; empty if there is none.
std::string vehicleArchetype(const std::string& tscText, const std::string& variant);

class Rider {
public:
    explicit Rider(const DpcFile& pilot);  // throws on malformed resources

    // Animation slot of the neutral seated pose of an archetype (BUGGY,
    // BUGGY_ROAD, MBIKE, MBIKE_SPORT, MBIKE_CHOPPER, MBIKE_TRAIL, MUSCLE,
    // QUAD, QUAD_TRAIL, SUV, TRUCK); -1 for an unknown archetype.
    int ridingSlot(const std::string& archetype) const;

    // The Head<head>, Chest<chest> and Legs<legs> parts skinned to animation
    // `slot` at `time` seconds, with liveries `livery` (1..) applied.
    BakedMesh bake(int slot, double time = 0, int head = 1, int chest = 1, int legs = 1, int livery = 1) const;

    // Bone world matrices of a pose (row-vector, model space), for inspection.
    std::vector<Mat4> pose(int slot, double time) const;
    size_t boneCount() const { return skel_.bones.size(); }

private:
    struct Track {
        std::vector<std::pair<float, std::array<float, 3>>> translation, scale;
        std::vector<std::pair<float, std::array<float, 4>>> rotation;
    };
    struct Animation {
        std::map<uint32_t, Track> tracks;  // by bone name CRC
        float duration = 0;
    };
    const Animation& animation(int slot) const;
    std::vector<Mat4> poseWorlds(const Animation* a, double time) const;
    uint32_t materialNamed(uint32_t nameCrc, const std::vector<uint32_t>& preferred) const;
    uint32_t liveryMaterial(const std::string& name) const;

    const DpcFile& dpc_;
    Skel skel_;
    Skin skin_;
    std::map<uint32_t, uint32_t> animSlots_;  // slot -> Animation_Z id
    std::map<uint32_t, size_t> boneByName_;
    std::map<uint32_t, uint32_t> materialByName_;  // Material_Z by name CRC (first)
    std::map<uint32_t, uint32_t> liveryByName_;    // MaterialAnim_Z name CRC -> Material_Z
    mutable std::map<uint32_t, Animation> anims_;
};

}  // namespace fuel
