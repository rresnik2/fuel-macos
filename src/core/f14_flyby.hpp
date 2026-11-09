// PK27: the F14 fly-by (notes/pk27.md; decode notes/p98/b000.md section 6, b063, b074). USA1 (resident) holds one
// Node_Z "F14" (name crc 0x410d5e27) with its Lod (one Mesh_Z) and a sound bank whose symbol 0 is the jet's loop; the
// clone pass makes it an F14_G agent ("cloneclass F14 F14_G") whose mover F14Move_G sits in the manipulator list and
// runs every frame the world runs (vf17 0x51a050). The jet is hidden between passes.
//   F14Move_G vf00 0x51a000: timer +0x20 = PP80 - 45 (855 s with the shipped pp001: the first pass 45 s in).
//   F14Move_G vf14 0x51a040: empty (F14_G vf20 reset reaches it: nothing is reset by a mission kill or retry).
//   vf17 0x51a050 (dt): the world object resolves (DAT_a7c0b0 +0x1c -> +0xc -> +0x28); timer >= 0: timer += dt, below
//     PP80 return; else spawn 0x51a380 (fails: try again next frame), unhide the node, timer = -1 [0xa05224] and fly
//     in the same frame. Flying (timer < 0): position += velocity dt; y = Math_ExpSmooth 0x4100b0 (rate 1.05
//     [0x9dc3c4]) towards ground(position + velocity 0.5 [0x9960c0]) + PP82; while (end - position) . velocity >= 0
//     the node gets the position and the rotation Quat_FromArc((0,0,1) [0xa06350], normalised node displacement of
//     the frame); else timer = 0 (the next pass PP80 later), node hidden, the loop stopped (0x4ead30 / 0x4e9e80).
//   spawn 0x51a380: needs a player (DAT_a6625c count, *DAT_a66260); player position 0x420330 -> +0x48, forward
//     0x421ab0 flattened 0x40fc80; centre = position + forward 300 [0x9dc070]; t = rand01 0x6460f0 x 2 pi [0x9dc258];
//     d = (fsin t, 0, fcos t) (0x40f3d0 is fsin, 0x40f3f0 fcos); velocity +0x3c = d PP81; start +0x24 = centre - d 1200
//     [0x9dbff4], end +0x30 = centre + d 1200 (y = centre y); start y = ground(start) + PP82; rotation +0x54 =
//     Quat_FromArc((0,0,1), d); the node is placed; Game_SoundMgr emitter 0x4e9da0: descriptor 0x4ea4a0 defaults
//     (flags 0x4f, volume 1), sound = the agent's bank symbol 0 (0x4ea930), attached to the F14 node, min / max
//     300 / 1200 m, doppler byte +0x30 = 1.
//   Emitter per frame 0x4ea560 (via 0x4eabf0 for the live emitters): position from the attached node; with the
//     doppler byte: rel = camera 0 - emitter (0x4ea8c0), pitch = 1 + clamp(-0.01 [0x9dc238] (|rel| - |rel_prev|) / dt,
//     -0.1 [0x9dca30], 0.1 [0x99648c]) (0x732e30), rel_prev = rel (+0xc..+0x14; zero at the start, so the first frame
//     reads the whole distance as approach: 0.9).
// Ground = 0x445500 (W+0x4b8 loaded -> 0x445570, the cubic map height; 0 when the map is not loaded).
// Nothing of the game is compiled in: PP80..82 come from the player's pp001.tsc.
#pragma once

#include <array>
#include <cstdint>
#include <functional>

namespace fuel {

struct F14Flyby {
    using V3 = std::array<float, 3>;
    using Q4 = std::array<float, 4>;  // x y z w

    // FUEL.exe .rdata constants.
    static constexpr float kAhead = 300.0f;        // 0x9dc070
    static constexpr float kHalfPass = 1200.0f;    // 0x9dbff4
    static constexpr float kLookAhead = 0.5f;      // 0x9960c0
    static constexpr float kClimbRate = 1.05f;     // 0x9dc3c4 (Math_ExpSmooth)
    static constexpr float kFlying = -1.0f;        // 0xa05224
    static constexpr float kFirstPassEarly = 45.0f;  // 0x51a000: PP80 - 45
    static constexpr float kSoundMin = 300.0f, kSoundMax = 1200.0f;  // 0x51a380 descriptor +0x24 / +0x28
    // pp001 rows (g_pp001_params [0x50], [0x51], [0x52]); shipped 900 / 250 / 80.
    float pp80 = 900.0f, pp81 = 250.0f, pp82 = 80.0f;

    // F14Move_G fields.
    float timer = 0;             // +0x20: >= 0 counting to the next pass, < 0 flying
    V3 pos{};                    // +0x24 (start, then the flight position)
    V3 end{};                    // +0x30
    V3 vel{};                    // +0x3c
    V3 player{};                 // +0x48 (0x420330 at the spawn)
    Q4 quat{0, 0, 0, 1};         // +0x54
    // The F14 node (hidden flag 0x40000 and its world position / rotation, what 0x65feb0 last set).
    bool shown = false;
    V3 nodePos{};
    Q4 nodeQuat{0, 0, 0, 1};
    // The emitter (Game_SoundMgr slot of the pass): playing, doppler state and the pitch 0x732e30 last set.
    bool soundOn = false;
    V3 relPrev{};                // descriptor +0xc..+0x14
    float pitch = 1.0f;
    int passes = 0;              // [port] passes started (checks, traces)

    std::function<float(float x, float y, float z)> ground;  // 0x445500
    std::function<float()> rand01;                            // 0x6460f0

    // F14Move_G vf00 0x51a000 (the world loads).
    void init(float pp80v, float pp81v, float pp82v);
    // vf17 0x51a050. `worldReady` = the world object chain resolves; `havePlayer` = 0x51a380's player test;
    // `playerPos` = 0x420330, `playerFwd` = 0x421ab0 (not normalised).
    void step(float dt, bool worldReady, bool havePlayer, const V3& playerPos, const V3& playerFwd);
    // 0x51a380 alone (vectors).
    bool spawn(bool havePlayer, const V3& playerPos, const V3& playerFwd);
    // The emitter's doppler 0x4ea560 for this frame (camera 0 at `cam`); returns the pitch (also in `pitch`).
    float doppler(float dt, const V3& cam);
    static float dopplerPitch(double distNow, double distPrev, float dt);  // 1 + clamp(-0.01 (now - prev) / dt, +-0.1)

    // Shared helpers (also used by the vectors).
    static Q4 fromArc(const V3& from, const V3& to);  // 0x6443a0 (core/race_helicopter)
};

}  // namespace fuel
