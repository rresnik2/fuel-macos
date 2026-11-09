// PK07: the race helicopter of the chopper races as an object in the scene: the world's IT_HELICO Lod_Z (an IT_ANIMATED
// skin: the rotor loops on its slot-0 animation) at the entry's display position and attitude (core/race_helicopter,
// 0x4f0a58: the visible object gets H+0x5c as position and H+0x68 as orientation).
#pragma once

#include "core/race_helicopter.hpp"
#include "core/skinned_model.hpp"
#include "renderer.h"

#include <vector>

class HeliView {
public:
    // `files` resolve resource IDs; `world` holds the IT_HELICO Lod_Z. False: no such object.
    bool load(const std::vector<const fuel::DpcFile*>& files, const fuel::DpcFile& world, fuel::MeshRenderer& r);
    bool loaded() const { return !keys_.empty(); }
    // Once per frame; `h` null (no helicopter race) hides it.
    void frame(float dt, const fuel::RaceHelicopter* h, fuel::MeshRenderer& r);
    // FUEL_HELI_LOOK=D[,H,A] (films): the camera D m from the helicopter, H m above it, A degrees round its heading
    // (0 behind, 90 its left side).
    static void debugLook(fuel::OrbitCamera& cam, const fuel::RaceHelicopter& h);

private:
    fuel::SkinnedModel model_;
    const fuel::anim::Clip* clip_ = nullptr;
    std::vector<uint32_t> keys_;
    double t_ = 0;
    bool shown_ = false;
};
