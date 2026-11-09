// PK23: FUEL's per-vehicle effects (notes ~/re-work/fuel/notes/pk23.md): the exhaust smoke, backfire / top-speed
// flames and gear-change embers of 0x5cf2d0 (setup 0x5d00c0, slots 0x5cf170), the exhaust flame quads of
// SpecialEffectsFlameEmitter_G (vf06 0x5edb20), and the scrape spark streaks of 0x5d07f0 (Vehicle_ImpactFx
// 0x51e9e0) drawn as SpecialEffectsRibbon_G ropes (emitter 0x4bd380, update 0x4be230, emit 0x4be560, step 0x4be930,
// rope 0x5defc0, alpha 0x4c5e60).
//
// Nothing of the game is compiled in: the tuning comes from pp001.tsc (PP5320..5333) through sfx::Tables and the
// exhaust points from the player's vehicle DPCs (named boxes SFX_EXHAUST1..16 of the parts' Mesh_Z links).
#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <vector>

#include "sfx.hpp"

namespace fuel {
class DpcFile;
struct AssembledVehicle;
}  // namespace fuel

namespace fuel::vfx {

using V3 = std::array<float, 3>;
using C4 = std::array<float, 4>;
using Quat = std::array<float, 4>;  // x y z w

// ---- 0x5cf170: the exhaust slots of a vehicle -----------------------------------------------------------------------
// The named boxes "SFX_EXHAUST1".."SFX_EXHAUST16" (0x6a23b0: the first part of the LOD record, in part order, whose
// Mesh_Z link has the box), in vehicle space. 0x5cf2d0 takes the box's world position and the direction q x (0, 0, -1)
// of its rotation (0x644c10 / 0x410510): the box's third row, negated.
struct ExhaustPoint {
    V3 pos{};      // vehicle space
    V3 dir{0, 0, -1};
    int index = 0; // n of SFX_EXHAUSTn
    int dyn = -1;  // PlacedPart::dyn of the part that carries it (FUEL updates the V:DYN pose first: 0x6a2270)
};
std::vector<ExhaustPoint> findExhausts(const DpcFile& dpc, const AssembledVehicle& av);

// ---- Tuning (pp001.tsc rows 5320..5333, read by 0x5cf2d0 / 0x5d00c0) ------------------------------------------------
struct ExhaustParams {
    float lifeLo = 0.5f, lifeHi = 1;         // PP5320, 5321 -> smoke life range (+0x10c / +0x110)
    float sizeLo = 0.15f, sizeHi = 0.3f;     // PP5323, 5324 -> size range x (1 + 1.5 P52 throttle)
    float rgbA = 0, rgbB = 0.1f;             // PP5325, 5326 -> colour A / B grey
    float intervalLo = 0.05f, intervalHi = 0.1f;  // PP5327, 5328 -> +0x78 / +0x7c
    float alpha = 0.6f;                      // PP5329 -> alpha (1 - v / 15) PP5329 below 15 m/s
    float rot = 1;                           // PP5331 -> rotation speed range +-PP5331
    float push = 1;                          // PP5332 -> velocity += dir PP5332
    float drag = 1;                          // PP5333 -> acceleration = -dir PP5333
    static ExhaustParams fromTables(const sfx::Tables& t);
};

// 0x5cf2d0's flame level (veh+0x1090) = max(flash +0x2ebc, clamp((SpeedFrac - 0.9) x 20, 0, 1) x clamp(50 x +0x15e8, 0, 1)).
// SpeedFrac (0x423380) and the gear (0x422510) are the PLAYER's (Cam_PlayerPhys), for every vehicle.
float flameLevel(float flash, float playerSpeedFrac, float wheel0Load);
// 0x5cf2d0: |v| / (P619 / 1.15 / 3.6) > 0.95 (the flame turns blue, embers at random).
bool nearTopSpeed(float speed, float topSpeedKmh);
// 0x5cf2d0: the smoke alpha (1 - v / 15) PP5329 at or below 15 m/s, else 0; the size factor 1 + 1.5 P52 throttle.
float smokeAlpha(float speed, float pp5329);
// 0x4c5e60: (1 - t)^0.25 of t = age / life (t = 1 when life <= 1e-6), 0 past the end.
float sparkAlpha(float age, float life);
// 0x5d07f0 (asm 0x5d0d8d..0x5d0ee7): the spark velocity range lo = q (0, 0, -10), negated when it points along the sliding
// velocity R[0x14..0x16], hi = q (0, 10, 0).
void sparkVelocity(const Quat& q, const V3& tangent, V3& lo, V3& hi);
// SpecialEffectsRibbon_G vf01 0x5defc0's rope: the head takes `head`, each next point is pulled to `len` of the one before
// once it is more than sqrt(thr2) from it.
void ropeStep(std::array<V3, 4>& rope, const V3& head, float len, float thr2);

// ---- SpecialEffectsFlameEmitter_G (vtable 0x9c39fc, 0x208 B) --------------------------------------------------------
struct FlameEmitter {
    bool stopped = true;          // +4
    bool fresh = false;           // +0x18c
    bool haveQuads = false;       // the 3 quad particles (+0x2c / +0x30)
    V3 pos{}, prevPos{};          // +0x08, +0x40
    float size = 0;               // +0xfc (redrawn in [+0x100, +0x104] = 0.4 L^2 both)
    float length = 0;             // +0x190 = 0.5 L^2
    V3 dir{0, 0, -1};             // +0x194 (0x5ce3b0, normalised)
    float t = 0;                  // +0x1a0
    bool blue = false;            // +0x1a4 near top speed
    std::array<float, 16> uv{};   // +0x1a8..+0x1e4: 8 uv pairs (quad 0 = right half of the atlas rect, quads 1-2 left half)
    struct Quad {
        std::array<V3, 4> v{};
        std::array<std::array<float, 2>, 4> uv{};
    };
    std::array<Quad, 3> quads{};
    C4 colour{2, 1.6f, 0.4f, 1};  // +0x1e8 (orange) or +0x1f8 (1.2, 1.2, 3, 1) when blue
    void start() {  // vf03 0x5d18f0
        if (stopped) stopped = false;
    }
    void stop() {  // vf04 0x5ce430: the quads are destroyed (the next start rebuilds them, fresh uvs)
        stopped = true;
        haveQuads = false;
    }
    void setPosition(const V3& p) { prevPos = pos, pos = p; }  // vf05 0x5d18c0
    // vf06 0x5edb20: `rect` = FX atlas entry +0x70 (0x30) rect (u0, v0, du, dv).
    void update(float dt, const std::array<float, 4>& rect);
};

// ---- SpecialEffectsEmitter_G<SpecialEffectsRibbon_G> (ctor 0x4bd380, 0x18c B) and SpecialEffectsRibbon_G (200 B) ----
struct RibbonParticle {
    V3 pos{};                     // +0x14
    V3 start{};                   // +0x20 (motion 3 / 4)
    V3 vel{}, accel{};            // +0x2c, +0x38
    float age = 0, life = 1;      // +0x54, +0x50
    float size = 0;               // +0x58
    float alpha = 1;              // +0x70
    C4 colour{1, 1, 1, 1};        // +0x6a.. rgb (halves), alpha drawn = alphaFn x +0x70 (+0x64)
    float drawAlpha = 0, drawSize = 0;  // +0x64 / +0x66 after the step
    float ropeLen = 0.029f;       // +0x80 (emitter +0x15c at the init; ctor 0x5deef0 0.029)
    float ropeThr2 = 0.000841f;   // +0x7c (ctor 0x5deef0: 0.029^2)
    std::array<V3, 4> rope{};     // +0xb8: 4 points (ctor 0x5deef0), all on the head at the init (0x5def80)
    uint8_t motion = 3, killFlags = 1;
};
struct RibbonEmitter {
    struct R { float cur = 0, lo = 0, hi = 0; };
    struct R3 { V3 cur{}, lo{}, hi{}; };
    bool stopped = true, burstDone = false, dying = false;  // +4, +5, +7
    V3 pos{}, prevPos{}, lastEmit{}, delta{};
    int spawnMode = 1;        // +0x50 & 0xf
    uint8_t motion = 3, killFlags = 1;
    int atlas = 0;            // +0x70
    int kind = 0x15;          // +0x14
    R interval{1, 1, 1};      // +0x74
    float timer = 0;          // +0x80
    R3 vel, accel, offset;    // +0x90, +0xb4, +0xd8
    R size, life{1, 1, 1};    // +0xfc, +0x108
    C4 colour{1, 1, 1, 1}, colourA{1, 1, 1, 1}, colourB{1, 1, 1, 1};  // +0x114, +0x124, +0x134
    R rope{0.02f, 0.02f, 0.035f};  // +0x15c (0x4bd960 at the ctor: 0.02 .. 0.035)
    int perEmission = 1;      // +0x168
    float age = 0;            // +0x17c
    int emissions = 0;        // +0x188
    std::vector<RibbonParticle> live;
    void start() {  // vf03 0x4bd730
        if (stopped) lastEmit = pos, stopped = burstDone = false;
    }
    void stop() { stopped = true; }                            // vf04 0x4be500 (motion 3: no rider particles)
    void kill() { if (!stopped) stop(); dying = true; }        // vf02 0x4be530 (flags +0x58 & 4 clear: particles live on)
    void setPosition(const V3& p) { prevPos = pos, pos = p; }  // vf05 0x4bd700
};

// ---- The per-vehicle feed (what 0x5cf2d0 / 0x5d07f0 read) -----------------------------------------------------------
struct SparkHit {         // one 0x5d07f0 call (from the physics' contact hook, in call order)
    uint64_t a = 0, b = 0; // R[0x37] / R[0x38]: the contact's two bodies (b = 0: static)
    V3 point{};            // R[0..2]
    V3 tangent{};          // R[0x14..0x16]: rel - n (rel . n)
    Quat q{0, 0, 0, 1};    // body +0x7f0
};
struct VehicleIn {
    uint64_t id = 0;             // the vehicle's identity: a new one runs the FX setup 0x5d00c0 again
    float throttle = 0;          // +0x1088 (engine throttle, shift cut included)
    V3 velocity{};               // +0x4d0
    float topSpeed = 0;          // +0xf7c = P619 (km/h)
    float wheel0Load = 0;        // +0x15e8
    float p52 = 0;               // +0x6a0 = VehiclePP 52
    float flash = 0;             // +0x2ebc (the backfire flash, 0x5d9d60)
    float nitro = 0;             // [nitro] the port's boost: the flame level it holds (amount x Params::flame), blue
    bool visible = false;        // +0x42d8 (0x5cbf10 view test)
    float cameraDistance = 0;    // +0x10a4 (x 3 behind the camera, 0x5cb8c0)
    std::vector<V3> slotPos, slotDir;  // world, per exhaust slot (0x6a23b0 + 0x644c10 / 0x410510)
    // [nitro] slots from this index on are the port's own nitro outlets (a vehicle without SFX_EXHAUST boxes, nitrous on):
    // no smoke, no FUEL flame or embers, only the boost's blue flame. SIZE_MAX: none.
    size_t nitroSlotsFrom = SIZE_MAX;
    // [upgrades U5] the nitrous power stage's flame (core/upgrade_visuals Plan::flameLength / flameSize): the boost's blue
    // flame x length / x size while boosting. 1 = as before.
    float nitroLength = 1, nitroSize = 1;
    std::vector<SparkHit> hits;
};
struct PlayerIn {  // 0x422510 / 0x423380 (Cam_PlayerPhys)
    bool valid = false;
    int gear = 0;           // +0x1270
    float speedFrac = 0;    // |phys+0x820| / (P619 x 0.86956525 x 0.2777778), clamped 0..1
};

// What the renderer draws (SubCodes 0x15 / 0x16: smoke.vhl vh_smoke + smoke.phl ph_smoke).
struct SmokeSprite {        // a CPU particle of the exhaust smoke (atlas 6)
    V3 pos{};
    float angle = 0, size = 0;
    std::array<float, 4> uv{};  // u at -x, v top, u at +x, v bottom
    C4 rgba{};
};
struct FlameQuad {          // 0x5fb890: 4 world vertices, the colour of vertex 0 (atlas 0x30 rect uvs)
    std::array<V3, 4> v{};
    std::array<std::array<float, 2>, 4> uv{};
    C4 rgba{};
};
struct Streak {             // 0x5fba20 / 0x5fae60 flags 0: a camera-facing strip along the rope
    std::array<V3, 4> rope{};
    float halfWidth = 0;    // sizeFn x +0x58
    C4 rgba{};              // rgb, a = alphaFn x +0x70
    int atlas = 0;          // FX_SPARKLE
    int kind = 0x15;        // 0x15 scrape sparks, 0x16 exhaust embers
};

struct Stats {
    int slots = 0, smokeLive = 0, flamesOn = 0, sparkRecords = 0, sparkLive = 0, emberLive = 0, embersEmitted = 0, sparkCalls = 0;
};

class VehicleFx {
public:
    explicit VehicleFx(sfx::System& sys);
    // One frame, after the physics: 0x5d07f0 calls of the frame (each vehicle's hits, in order), then 0x5cf2d0 per
    // vehicle (index 0 = the player), then the FX manager's step (vf17 0x5f62c0) of every emitter here.
    void update(float dt, std::vector<VehicleIn>& vehicles, const PlayerIn& player, const V3& eye);
    // veh+0x1b5c: set by 0x5d07f0 (and the teardown 0x5d1060), cleared only by the FX setup 0x5d00c0; the per-vehicle
    // water emitter (0x5ce530) stays stopped while it is set.
    bool fxFlag(size_t vehicle) const { return vehicle < veh_.size() && veh_[vehicle].fxFlag; }
    void collect(std::vector<SmokeSprite>& smoke, std::vector<FlameQuad>& flames, std::vector<Streak>& streaks, const V3& eye) const;
    const Stats& stats() const { return stats_; }
    const ExhaustParams& params() const { return pp_; }

private:
    struct SparkRecord {     // the 0x24-B record of veh+0x42cc
        bool hit = false;    // +0
        uint64_t a = 0, b = 0;  // +8, +0xc
        V3 last{};           // +0x14 the emitter position before this call
        std::unique_ptr<RibbonEmitter> e;  // +0x10
    };
    struct Slot {
        sfx::Emitter smoke;  // +0x1ad8[i] (SfxEmitter 0x4c4730)
        FlameEmitter flame;  // +0x1b18[i]
    };
    struct Vehicle {
        bool init = false;
        uint64_t id = 0;
        int gear = 0;            // +0x108c
        float level = 0;         // +0x1090
        bool fxFlag = false;     // +0x1b5c
        std::vector<Slot> slots;
        std::vector<SparkRecord> sparks;
    };
    void setupVehicle(Vehicle& v, size_t slots);           // 0x5d00c0
    void sparkCall(Vehicle& v, const VehicleIn& in, const SparkHit& h);  // 0x5d07f0
    void exhaust(Vehicle& v, VehicleIn& in, const PlayerIn& player, float dt);  // 0x5cf2d0
    void runRibbon(RibbonEmitter& e, float dt, const V3& eye);  // vf06 0x4be230
    void emitRibbon(RibbonEmitter& e);                         // vf08 0x4be560 (+ init vf11 0x4bba50)
    float parkMiller();  // [port] the game's 0x6460f0 stand-in
    sfx::System& sys_;
    ExhaustParams pp_;
    std::vector<Vehicle> veh_;
    std::vector<std::unique_ptr<RibbonEmitter>> dying_;  // killed spark emitters, kept until their streaks die
    RibbonEmitter embers_;   // UniqueEmitters +0x14 (0x5fc500): one particle per vf08, kind 0x16
    uint32_t pm_ = 0x13579bdu;
    Stats stats_;
};

}  // namespace fuel::vfx
