// P65: FUEL's tornado look, decoded from FUEL.exe (notes ~/re-work/fuel/notes/p65-storm-visuals.md).
//
// Each tornado agent owns a SpecialEffectsTornado_G (ctor 0x5e6480, made by the manipulators' vf00 0x5bcfa0 /
// 0x5bf4a0) that the agent updates every frame (0x5e6e20 from 0x5bea80 / 0x5bf5f0). It draws three things:
//   - the funnel, SpecialEffectsTornadoMesh_G (ctor 0x5e78d0, vf02 0x5e7ca0 buffers, vf03 0x5e9510 update, vf04
//     0x5e9600 draw): a tube of 32 rings x 16 vertices built every other update by 0x5e82d0 around a Hermite spine
//     (0x5e7fe0) from the base to a wobbling top in the cloud; SubCode 0x23, tornado.vhl vh_tornado / ph_tornado,
//     material TORNADE_CYLINDRE (SHARED.DPC);
//   - the base smoke, 200 soft particles orbiting the axis and rising (0x5e7a00 seeds, 0x5e8ca0 per update); SubCode
//     0x25, vh_tornadobasesmoke / ph_tornadosoftsmoke, FX atlas entry 6 (0xa66cd4 = 0xa66c24 + 6 x 0x1c + 8);
//   - two cloud-top discs (SpecialEffectsCloudTop_G 0xa65714 / 0xa65718 of the four static ones, 0x5e59d0 grid,
//     vf04 0x5e5e40 draw): flat squares turning about y above the funnel top; SubCode 0x26, vh_tornadoclouds /
//     ph_tornadoclouds, material FX_TORNADEHAUT.
// FUEL shares one funnel mesh per kind (0xa6570c mother, 0xa65710 small); the port keeps one per agent.
#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <vector>

namespace fuel::storm {

// The tuning m_5e6860 copies into the mesh (pp001.tsc rows), and the constants of the ctors.
struct TornadoLook {
    float topFlare = 400;    // mesh +0x128: PP5210 mother / PP5212 small (ring radius += (i/32)^4 x this)
    float baseRadius = 60;   // +0x124: PP5211 / PP5213
    float wobble = 200;      // +0x4c: 200 / 250 (the top's circle, and the spine tangents 0x5e81d0)
    float vScale = 0.7f;     // +0x138: PP5214 / PP5215 (+0x130 = 1)
    float scroll = 0.03f;    // +0xb0: PP5254 / PP5264 (C_USER0.yz uv scroll per second)
    float smokeSize = 50;    // +0xc4: PP5221 / PP5220
    float smokeRise = 20;    // +0xcc: PP5223 / PP5222
    float smokeSpread = 1.5f;// +0xc8: PP5225 / PP5224
    float smokeAlpha = 0.4f; // +0xd0: PP5227 / PP5226
    float smokeSpinMin = 0.02f, smokeSpinMax = 0.04f;  // +0xd4 / +0xd8: PP5255 / 5256, PP5265 / 5266
    float cloudSpin = 0.2f;  // CloudTop 0xa65714 +0x5c: PP5253 / PP5263 (0xa65718: 1.5 x)
    float fadeIn = 2, fadeOut = 2;       // PP5216 / PP5217 (0x5e6e20: alpha +8 moves 1 / x per second)
    float moveU = -0.005f, moveV = 0;    // PP5218 / PP5219 (uv offset per metre the base moves)
    // `P(n)` = pp001.tsc row n.
    static TornadoLook load(const std::function<float(int)>& P, bool mother);
};

// TornadoVertex_Z (0x40 bytes, VS_Tornado_Input): position, normal, UVAlphaAdd, tangent, binormal. Positions are
// relative to the tornado base.
struct TornadoVertex {
    float pos[3];
    float normal[3];
    float uv[2];
    float alpha, add;
    float tangent[3];
    float binormal[3];
};
static_assert(sizeof(TornadoVertex) == 0x40, "TornadoVertex_Z");

// One base smoke particle as the four TornadoSmokeVertex_Z of 0x5e8ca0 give it to vh_tornadobasesmoke: the point
// (0, y, 0) pushed out along the rotated normal (-1, 0, 0) about the axis (0, 1, 0) by `diam`, a view-facing quad of
// half size `size` turned by `rot`, brightness 0.05 (ph_tornadosoftsmoke ignores it) and `alpha`.
struct TornadoSmoke {
    float y, diam, angle, rot, size, alpha;
};

// A cloud-top disc (0x5e59d0: one 5 x 5 grid layer, normal (0, -1, 0), tangent (1, 0, 0), binormal (0, 0, 1),
// uv (1 - u, v)) centred `lift` above `centre`, half size `half`, turned about y by `angle`.
struct TornadoCloudTop {
    std::array<double, 3> centre{};
    float half = 0, lift = 0, angle = 0;
};

class TornadoEffect {
public:
    static constexpr int kSegments = 16;  // mesh +0x38
    static constexpr int kRings = 32;     // mesh +0xf8
    static constexpr int kSmoke = 200;    // mesh +0xa8

    TornadoEffect(const TornadoLook& look, uint32_t seed);

    // 0x5e6e20 + the mesh's vf03 (0x5e9510) for one frame: `base` = the agent position, `camera` = 0x421980's
    // point. `fading` = state 1 (alpha towards 0) instead of the fade in.
    void update(float dt, const std::array<double, 3>& base, const std::array<double, 3>& camera, bool fading = false);

    const std::array<double, 3>& base() const { return base_; }
    float alpha() const { return alpha_; }  // +8 (copied to the mesh +0xdc)
    // PK06: state 1 (0x5be290): alpha falls at 1 / PP5217 per second; over once it reaches 0.
    bool fadingOut() const { return state_ == 1; }
    bool faded() const { return state_ == 1 && alpha_ <= 0.0f; }
    // PK06: 0x5e91d0, run by the mesh build 0x5e82d0: the world's ground height 3 base radii (mesh +0x124) from the
    // nearest tornado to the camera, towards +z, -x, -z, +x (mesh +0xe0 .. +0xec). The shipped shaders (vh_tornadobasesmoke,
    // ph_tornadosoftsmoke, ph_tornado) read no clip height, so these four numbers feed nothing that draws; kept 1:1.
    // `tornado` = the position of the first listed tornado within 10000 m of the camera (0 = none: the heights stay).
    void setClipHeights(const std::array<float, 4>& h) { clip_ = h; }
    const std::array<float, 4>& clipHeights() const { return clip_; }
    float baseRadius() const { return look_.baseRadius; }
    // C_USER0.yz of the funnel draw (0x5e9600): (+0xb4 - 2 t +0xb0, t +0xb0 + +0xb8), t = mesh +0xa4.
    std::array<float, 2> uvScroll() const;
    // The funnel (kRings x kSegments vertices, ring-major) and its index list (0x5e7ca0, shared).
    const std::vector<TornadoVertex>& funnel() const { return verts_; }
    static const std::vector<uint16_t>& funnelIndices();
    const std::vector<TornadoSmoke>& smoke() const { return smokeOut_; }
    std::array<TornadoCloudTop, 2> cloudTops() const;
    bool built() const { return built_; }

private:
    struct Particle {  // 0x1c record (0x5e7a00)
        float alpha, h, fall, angle, spin, rot, rotSpeed;
    };
    float rand01();
    void retarget();          // 0x5e81d0
    void build(float dt);     // 0x5e82d0 (+ 0x5e8ca0)
    std::array<float, 3> spine(float t) const;  // 0x5e7fe0

    TornadoLook look_;
    uint32_t rng_;
    // SpecialEffectsTornado_G
    std::array<double, 3> base_{}, prev_{};
    bool started_ = false;   // +0x14
    int state_ = 2;          // +4: 2 fade in, 1 fade out, 0 steady
    float alpha_ = 0;        // +8
    float cloudY_ = 0;       // +0x1c
    // SpecialEffectsTornadoMesh_G
    bool toggle_ = true;     // +0x3c: builds on every other vf03
    float time_ = 0;         // +0xa4
    float uOff_ = 0, vOff_ = 0;  // +0xb4 / +0xb8
    float height_ = 400;     // +0x12c (400 in the ctor, then cloud - base + 250)
    float wob_ = 0;          // +0x98
    float lerp_ = 0;         // +0x94
    std::array<float, 3> sOld_{}, eOld_{}, sNew_{}, eNew_{};  // +0x64 / +0x70 -> +0x7c / +0x88
    std::array<float, 3> s_{}, e_{}, top_{};                  // +0xfc / +0x108, +0x114
    std::array<float, 2> segSin_[kSegments];                  // +0x44 (sin, cos) of j 2 pi / 15
    std::vector<TornadoVertex> verts_;
    std::vector<Particle> parts_;
    std::vector<TornadoSmoke> smokeOut_;
    std::array<float, 4> clip_{};  // PK06: mesh +0xe0..+0xec (0x5e91d0)
    float cloudTime_ = 0;    // CloudTop +0x60
    bool built_ = false;
};

}  // namespace fuel::storm
