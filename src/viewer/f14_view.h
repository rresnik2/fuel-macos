// PK27: the F14 fly-by in the scene: the world's "F14" Lod_Z (USA1, one Mesh_Z) drawn through the renderer's prop list
// at the node's position and rotation (core/f14_flyby: what 0x65feb0 last set), only while the node is shown.
#pragma once

#include "core/f14_flyby.hpp"
#include "renderer.h"

#include <vector>

class F14View {
public:
    // `world` holds the F14 Lod_Z (name crc of "F14"). False: no such object.
    bool load(const fuel::DpcFile& world);
    bool loaded() const { return !meshes_.empty(); }
    // Once per frame. Hidden (between passes, or `f` null): the renderer's list is cleared once and then left alone.
    void frame(const fuel::F14Flyby* f, fuel::MeshRenderer& r);
    // The node's world matrix (the port's column layout; checks).
    static simd_float4x4 world(const fuel::F14Flyby& f);
    // FUEL_F14_LOOK=D[,H,A] (films): the camera D m from the jet, H m above it, A degrees round its heading (0 behind);
    // FUEL_F14_LOOK=player: from 2 m above the player, looking at the jet.
    static bool debugLook(fuel::OrbitCamera& cam, const fuel::F14Flyby& f, simd_float3 player);
    const std::vector<uint32_t>& meshes() const { return meshes_; }

private:
    std::vector<uint32_t> meshes_;  // the Lod's level-0 object (Mesh_Z ids)
    bool shown_ = false;
};
