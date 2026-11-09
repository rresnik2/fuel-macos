// A skinned object outside the rider (P47): a Lod_Z whose object is a Skin_Z (bones Skel_Z, meshes Mesh_Z with
// per-batch bone palettes, the Lod's f1 (slot, Animation_Z) table), posed by anim::Clip (FUEL's samplers) and skinned
// on the CPU into BakedMesh vertices, one per Mesh_Z. Used for the animated world objects (BaseSharingPlayAnimAgent_G
// flags, IT_ANIMATED; notes/p47-work/anim-objects.md) and the RTC helicopter (TELEPORT_HELICO's Lod HELICO_RTC).
// The skinning math is the rider's (rider.cpp: palette slot = vertex index / 6, local = S R T, world = local x parent).
#pragma once

#include "anim_clip.hpp"
#include "assembly.hpp"
#include "dpc.hpp"
#include "rider.hpp"

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace fuel {

class SkinnedModel {
public:
    // files: where to resolve resource IDs (first hit wins). id: a Lod_Z (object[0] a Skin_Z) or a Skin_Z.
    bool load(const std::vector<const DpcFile*>& files, uint32_t id, std::string* error);

    struct Part {
        uint32_t meshId = 0;  // the Mesh_Z this part replaces when drawn
        BakedMesh mesh;       // bind-pose vertices, batches, materials
        struct V {
            float pos[3], nrm[3], tan[3];
            int bone[4];
            float w[4];
        };
        std::vector<V> verts;
    };
    std::vector<Part> parts;
    size_t boneCount() const { return skel_.bones.size(); }
    const std::vector<std::pair<uint32_t, uint32_t>>& slots() const { return lod_.animations; }  // (slot, Animation_Z)

    const anim::Clip* clipForSlot(uint32_t slot) const;
    const anim::Clip* clipById(uint32_t animId) const;  // nullptr when missing or malformed
    // Bone world matrices (model space) for clip at time t (no clip: the bind pose). P96: `joints` = bones whose
    // model-space origin is set after the clip (RTC 0x103 'AMEO', 0x676480: the bone's local translation becomes the
    // point in its parent's frame).
    std::vector<anim::Mat4> pose(const anim::Clip* clip, double t, const std::vector<std::pair<size_t, anim::Vec3>>& joints = {}) const;
    const Skel& skeleton() const { return skel_; }
    // PK29: the bone named `crc` (index into pose()), -1 when the skeleton has none.
    int boneIndex(uint32_t crc) const {
        const auto it = boneByName_.find(crc);
        return it == boneByName_.end() ? -1 : int(it->second);
    }
    void skin(const Part& p, const std::vector<anim::Mat4>& boneWorld, std::vector<uint8_t>& vertices) const;

private:
    const DpcFile* fileOf(uint32_t id) const;
    std::optional<std::vector<uint8_t>> body(uint32_t id) const;
    uint32_t materialNamed(uint32_t nameCrc, const std::vector<uint32_t>& preferred) const;

    std::vector<const DpcFile*> files_;
    Lod lod_;
    Skel skel_;
    std::vector<anim::Mat4> invBind_;
    std::map<uint32_t, size_t> boneByName_;
    mutable std::map<uint32_t, std::unique_ptr<anim::Clip>> clips_;
    mutable std::map<uint32_t, uint32_t> materialByName_;
};

}  // namespace fuel
