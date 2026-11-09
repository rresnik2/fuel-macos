// MaterialAnim_Z: FUEL's animated materials (p57). Decoded from FUEL.exe:
//   load      MaterialAnim_Z::vf03 0x800570 (readers 0x804ab0, Curve_Read12 0x804b90, 0x804960,
//             Curve_Read16 0x804a40, Curve_Read20 0x804ce0, 0x804b30)
//   advance   0x76a420 (called for every registered anim by MaterialManager 0x7acfc0 with the frame dt)
//   evaluate  0x76a580 (track -> Material_Z field, then vf+0x48 UpdateFlags / vf+0x44 UpdateUVMatrix)
//   curves    Curve_Eval12 0x800000, float 0x72fa90, Curve_Eval16 0x7d5870, Curve_Eval20 0x7d20d0,
//             step 0x800470, flip-book 0x7ffae0
// No game data here: bodies are parsed from the player's DPC files at run time.
#pragma once

#include "resources.hpp"

#include <array>
#include <cstdint>
#include <vector>

namespace fuel {

namespace cls {
inline constexpr uint32_t MaterialAnimZ = 0xE48C96A6;  // = vehicle.hpp MaterialAnim
}

struct MaterialAnim {
    template <size_t N>
    struct Key {
        float t = 0;
        std::array<float, N> v{};
    };
    template <size_t N>
    struct Track {
        uint16_t mode = 0;  // bit 3: hold (no interpolation)
        std::vector<Key<N>> keys;
    };
    struct StepKey {
        float t = 0;
        uint32_t v = 0;
    };
    // Tracks in file order; the Material_Z field each one drives (object offset = body + 0xc).
    std::vector<StepKey> texture;  // flip-book -> slot 0 bitmap (obj+0x9c); file key {u32 bitmap, f32 t}
    Track<2> uvOffset;             // -> obj+0x7c (body 0x70)
    Track<2> uvScale;              // -> obj+0x84 (body 0x78)
    Track<1> uvAngle;              // -> obj+0x58 (body 0x4c), key {f32 t, i16 v / 4096}
    Track<3> diffuse;              // -> obj+0x0c (body 0x00)
    Track<3> emissive;             // -> obj+0x1c (body 0x10)
    Track<1> alpha;                // -> obj+0x18 (body 0x0c), i16 / 4096
    Track<4> specular;             // -> obj+0x5c (body 0x50)
    Track<4> params;               // -> obj+0x6c (body 0x60), then obj+0x74 (body 0x68) x 0.01
    std::vector<StepKey> field2c;  // -> obj+0x38 (body 0x2c)
    std::vector<StepKey> flags;    // -> obj+0x48 (body 0x3c, the material flags)
    uint32_t material = 0;         // the Material_Z it animates
    float period = 0;              // obj+0x8c
    uint8_t state = 0;             // obj+0x90: 0x40 playing, 0x02 play once, 0x80 finished, 0x08 auto-start on reset

    bool animatesUv() const { return !uvOffset.keys.empty() || !uvScale.keys.empty() || !uvAngle.keys.empty(); }
    bool empty() const {
        return texture.empty() && uvOffset.keys.empty() && uvScale.keys.empty() && uvAngle.keys.empty() && diffuse.keys.empty() &&
               emissive.keys.empty() && alpha.keys.empty() && specular.keys.empty() && params.keys.empty() && field2c.empty() &&
               flags.empty();
    }
};

MaterialAnim parseMaterialAnimBody(const std::vector<uint8_t>& body);  // throws

// The running state of one anim (obj+0x88 time, +0x90 state).
struct MaterialAnimClock {
    float time = 0;
    uint8_t state = 0;
    // 0x76a420: only while playing (0x40); loops by subtracting the period, or (play once, 0x02)
    // stops at the period with state = (state & 0x3f) | 0x80. Returns true when it evaluated.
    bool advance(float dt, float period);
};

// 0x76a580 applied to a copy of the material: every track present overwrites its field, then the
// UV matrix is rebuilt (0x695ee0). The flip-book writes its bitmap ID into m.textures[0].
void evaluateMaterialAnim(const MaterialAnim& a, float t, Material& m);

// Material_Z::UpdateUVMatrix 0x695ee0 from m.uvAngle / uvOffset / uvScale.
void updateUvMatrix(Material& m);

// How a Material_Z is drawn (p57): Material_Z::UpdateFlags 0x695f70 derives flags 0x20 (blend) / 0x40 (test):
// opacity < 1 or unlit (flags & 0x3000) -> blend; else flags & 0x180000 -> test; else the slot-0 bitmap's alpha
// kind (Bitmap +0x2d): 2 -> blend, 1 -> test. MeshPc_Z::Draw 0x848880 / render states 0x858810: codes 17 and 19
// draw opaque without alpha test (pass bits 0x2f); every other draw keeps ALPHAFUNC GREATER (0x838160) with
// ALPHAREF 0, or 128 with flag 0x100000; CULLMODE CCW (bit 0x80) unless flag 0x8000 or the draw is blended
// (bits 0x1d / 0x1f carry no cull bit). bCubemap (mask bit 8) = FallOff.x + FallOff.y != 0 or a slot-8 texture.
struct MaterialDrawState {
    enum Mode { Opaque = 0, Test = 1, Blend = 2 } mode = Opaque;
    float alphaRef = 0;
    bool twoSided = false;
    bool cubemap = false;
};
MaterialDrawState materialDrawState(const Material& m, uint8_t alphaKind);

// P72: the blended frame phases of MeshPc_Z::Draw 0x848880 (queue preset +0xd20, run in ascending order by the frame
// flush 0x858e60, each phase sorted back to front by 65535 / (depth + 1)).
// Phase of a blended draw or of any draw of a fading object: 0x12 | (~flags >> 16 & 1); 0x0c | .. with flag 0x40000;
// code 0x17 -> 0x13; codes 0x12 / 0x14 ignore 0x40000.
uint32_t blendPhase(uint32_t flags);
// Codes that never take the fade path (view alpha < 1): 1, 0x11, 0x13 (pass bits 0x2f), 0x12, 0x14, 0xd, 0x17.
bool ignoresFade(uint32_t code);
// A fading draw lays its depth first (pass bits 0x83 / 0x03) when (flags & 0xb000) == 0 or flags & 0x200000.
bool fadePrePass(uint32_t flags);
// The blended colour pass writes depth (0x1f instead of 0x1d) with flag 0x40000 or codes 0x12 / 0x14.
bool blendZWrite(uint32_t flags);

// ---- PK28: the blend mode of a material draw, 0x856160 (EDI = Material_Z flags +0x48, cached in renderer +0x3cbd4;
// called by the record draw 0x854d50, the pass begin 0x855280 and the frame flush 0x858e60): D3DRS_BLENDOP (0xab),
// SRCBLEND (0x13), DESTBLEND (0x14), tested in this order: 0x1000 -> ADD, SRCALPHA, ONE; 0x4000 -> ADD, DESTALPHA,
// ONE; 0x2000 -> REVSUBTRACT, SRCALPHA, ONE (dest - src x a); else ADD, SRCALPHA, INVSRCALPHA. D3D9 without
// SEPARATEALPHABLENDENABLE: the alpha channel blends the same way.
enum class BlendKind : uint32_t { Alpha = 0, Add = 1, AddDestAlpha = 2, RevSubtract = 3 };
BlendKind blendKind(uint32_t flags);
struct BlendState { uint32_t op, src, dst; };  // D3DBLENDOP / D3DBLEND values as 0x856160 writes them
BlendState blendState(uint32_t flags);
// ---- PK28 end

// The curve helpers, exposed for --rendercheck vectors.
float evalFloatTrack(const MaterialAnim::Track<1>& tr, float t);  // 0x72fa90 (values already / 4096)
template <size_t N>
std::array<float, N> evalTrack(const MaterialAnim::Track<N>& tr, float t);  // Curve_Eval12/16/20
uint32_t evalStepTrack(const std::vector<MaterialAnim::StepKey>& keys, float t);  // 0x800470 / 0x7ffae0

}  // namespace fuel
