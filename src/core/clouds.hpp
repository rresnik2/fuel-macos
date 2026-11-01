// FUEL's billboard clouds: SpecialEffectsCloudTank_G (ctor 0x5fd900, vtable 0x9c46cc), owned by the weather
// manager at +0x150 (0x5bb8fc, 350 clouds, atlas entry 14 FX_BILLBOARD_CLOUDS, SFX SubCode 0x27), drawn with
// sfx.vhl vh_cloud + sfx.phl ph_sfxScattering. Notes ~/re-work/fuel/notes/p37-sky-clouds.md.
//
// Nothing of the game is compiled in: the tuning is the player's pp001.tsc (rows 5201-5209); the cloud count (350,
// pushed at 0x5bb963), the 10 s refresh (ctor +0x9c), the 1.25 / 0.8 sight factors and the 1.15 width factor are the
// engine's scalar constants, cited where used.
#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <vector>

namespace fuel::clouds {

using V3 = std::array<float, 3>;

// One cloud (0x20 bytes at tank +0x100).
struct Cloud {
    V3 pos{};            // +0x00: x, 0, z (the height comes from the draw constants)
    int variant = 0;     // +0x0c: quarter of the cloud texture, floor(rand x 4) (0x5fdf60)
    float dist = 0;      // +0x10: horizontal distance to the tank position when last refreshed
    float width = 0;     // +0x14: half width, lerp(PP5209, PP5208, r), x 1.15 for variant 0 (0x9dcaf8)
    float height = 0;    // +0x18: half height, lerp(PP5209, PP5208, r)
    bool active = false; // +0x1c
    bool mirror = false; // +0x1d: u flipped (rand > 0.5)
};

// The pp001 rows the tank copies every frame (0x5fe040) and its fixed counts.
struct Params {
    float colour = 3;        // PP5201: vertex rgb = 1.25 x this (0x5fe580)
    float heightCloudy = 400, heightClear = 800;  // PP5202 / PP5203: layer height above the tank position
    float sizeMin = 300, sizeMax = 400;           // PP5209 (+0x7c) / PP5208 (+0x80)
    float spacing = 250;     // PP5207 (+0x84): no two active clouds closer than this
    float sight = 3000;      // PP5206 (+0x8c); +0x90 = 1.25 sight, +0x88 = 0.8 sight
    int count = 350;         // 0x5bb963 push 0x15e -> vf+0xc 0x5fdb80
    float refresh = 10;      // ctor +0x9c (0x41200000)
    static Params fromPP(const std::vector<float>& pp);
};

// The tank. `rndWell` stands for the FX generator 0x5fd710 (angles, radii), `rndGame` for the game's 0x6460f0
// (sizes, variant, mirror); both return [0, 1).
class Tank {
public:
    using Rnd = std::function<float()>;
    Tank(const Params& p, Rnd rndWell, Rnd rndGame) : p_(p), well_(std::move(rndWell)), game_(std::move(rndGame)) {}

    // 0x5fd890 (vf+0x24): the reference position; a jump of more than 1000 m repopulates.
    void setPosition(const V3& pos, float cloudness);
    // 0x5fe040 (vf+0x14) with the weather cloud amount (+0x6c, from 0x5bba30) and Cloudness = clamp01(2 cloud)
    // at the tank position (0x5b7e40 -> +0x94).
    void update(float dt, float cloud, float cloudness);
    // 0x5fdd10 (vf+0x10): every slot rebuilt, the first ftol(clamp01(cloudness) x count) active inside the sight.
    void populate(float cloudness);

    const std::vector<Cloud>& clouds() const { return clouds_; }  // sorted back to front (0x5fdf10) at each refresh
    int activeCount() const { return active_; }                    // +0x98
    float layerHeight() const { return layer_; }                   // +0xf8
    const V3& position() const { return pos_; }
    const Params& params() const { return p_; }
    bool dirty() const { return dirty_; }  // +0x44: the vertex buffer needs a rebuild (0x5fe580)
    void clearDirty() { dirty_ = false; }

    // Pieces, exposed for the checks.
    void placeAt(const V3& pos) { pos_ = pos, havePos_ = true; }  // the position without the repopulation test
    std::vector<Cloud>& mutableClouds() { return clouds_; }
    V3 randomPosition(bool inside);                         // 0x5fdbf0
    bool accept(const V3& p) const;                         // 0x5fde90
    void build(Cloud& c, const V3& p);                      // 0x5fdf60
    static bool backToFront(const Cloud& a, const Cloud& b); // 0x5fdf10

private:
    Params p_;
    Rnd well_, game_;
    std::vector<Cloud> clouds_;
    V3 pos_{};
    bool havePos_ = false, dirty_ = true;
    float timer_ = 0;   // +0xa0
    int active_ = 0;    // +0x98
    float layer_ = 800; // +0xf8
};

// 0x5fe370 (vf+0x28): C_USER0..C_USER6 of the cloud draw. C_USER0 = (Cloudness +0x94, CloudsAlpha +0xf4, 0, 0),
// C_USER1..4 = lightning flashes (+0xa4.., w +0xd4..; 0 in the port), C_USER5 = (-100, 0, 0, 0),
// C_USER6 = (2000, sight +0x8c, 0.8 sight +0x88, layer +0xf8 + tank y +0x18).
std::array<float, 28> drawConstants(float cloudness, float cloudsAlpha, float sight, float layer, float tankY);

// What vh_cloud computes for one corner (CPU copy for the checks): corner sign sx, sy (+-1), eye, constants.
struct Corner {
    V3 pos{};      // world position
    float alpha;   // o.diffuse.a before the texture
    float shade;   // 1 - Cloudness s (bottom corners darkened)
};
Corner vertexCorner(const Cloud& c, int sx, int sy, const V3& eye, float layerY, float sight, float cloudness);

}  // namespace fuel::clouds
