// FUEL's game special effects (SpecialEffects*_G): the range-parameter particle emitter, the wheel
// emitters (dust, stones, mud/water projections, slip smoke) and the per-surface rules that switch
// them, decoded from FUEL.exe (notes p30-effects.md, p26-work/particles.md).
//
// Nothing of the game is compiled in: the tuning comes from the player's pp001.tsc (rows 5000-5699),
// the FX atlas table is replayed from FUEL.exe's static initialiser 0x8e6ae0 and the random generator's
// start state is read from FUEL.exe .data 0xa02260.
#pragma once

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace fuel::sfx {

// ---- Tables read from the player's files ------------------------------------------------------------
struct AtlasEntry {
    std::string material;  // MaterialAnim_Z name (link name CRC) in DATAS/SHARED.DPC, e.g. FX_ATLAS_0
    float u0 = 0, v0 = 0, du = 1, dv = 1;
};

struct Tables {
    bool ok = false;
    std::string error;
    std::vector<float> pp;               // pp001.tsc rows 0..5999 (missing rows 0)
    std::vector<AtlasEntry> atlas;       // FX atlas (0x8e6ae0 -> BSS 0xa66c24, stride 0x1c), 51 entries
    std::array<uint32_t, 16> wellSeed{}; // WELL512 state (.data 0xa02260)
    float P(int n) const { return n >= 0 && size_t(n) < pp.size() ? pp[size_t(n)] : 0.0f; }
};
// `exe` = FUEL.exe, `pp001` = CommonComponent/Param/pp001.tsc. Never throws; ok = false with `error` set.
Tables loadTables(const std::string& exe, const std::string& pp001);
// Only the atlas replay (for the checks).
std::vector<AtlasEntry> replayAtlas(const std::string& exe, std::string* error);

// ---- The game FX random generator (0x5fd710, WELL512 with FUEL's tempering mask) -------------------
class Well512 {
public:
    explicit Well512(const std::array<uint32_t, 16>& seed) : s_(seed) {}
    uint32_t next();
    // The engine's uniform: (bits & 0x7fffff | 1.0f) - 1, in [0, 1).
    float uniform();
    float range(float lo, float hi) { return uniform() * (hi - lo) + lo; }

private:
    std::array<uint32_t, 16> s_;
    uint32_t i_ = 0;
};

// ---- Fade curves ---------------------------------------------------------------------------------
// The particle's alpha and size functions (emitter +0x5c/+0x64) and the u16 code (+0x60/+0x68) that
// encodes the same curve for the GPU particles (0x5fab60 -> sfx.h GetParameterValue).
enum class Fade : uint8_t {
    One,          // 0x4beae0, code 0x000
    OneMinusT,    // 0x4beab0, code 0x101
    OneMinusTSq,  // 0x5f4810, code 0x301
    SqrtT,        // 0x5f4840, code 0x401
    Quarter,      // 0x5d2fd0 0.25 + 0.75 t, code 0x003
    SqrtQuarter,  // 0x5edad0 sqrt(0.25 + 0.75 t), code 0x403
    LogPuff,      // 0x5f47c0 1 - ln(2.35 t + 0.38)^2 (CPU only; code 5 has no GPU meaning)
};
uint16_t fadeCode(Fade f);
float fadeValue(Fade f, float age, float life);  // the CPU function (t = age / life, 1 when life <= 1e-6)
// 0x5fab60: the float4 the GPU vertex carries for a code.
std::array<float, 4> codeParams(uint16_t code);
// sfx.h GetParameterValue(P, t) with C_USER0.y = 100 (CPU copy of the shader maths, for the checks).
float parameterValue(const std::array<float, 4>& P, float t);

// RGB <-> HSV of the wheel colours (0x5f4be0 / 0x5f4a90): h in [0, 6).
std::array<float, 4> rgbToHsv(const std::array<float, 4>& rgba);
std::array<float, 4> hsvToRgb(const std::array<float, 4>& hsva);

// ---- Particles -------------------------------------------------------------------------------------
using V3 = std::array<float, 3>;
using C4 = std::array<float, 4>;

// One billboard as both draw paths need it.
struct Particle {
    V3 start{};         // +0x14 spawn position (+0x20 start for motion 3/4)
    V3 pos{};           // current position (CPU particles)
    V3 vel{}, accel{};  // +0x2c, +0x38
    float angle = 0, rotSpeed = 0;  // +0x48, +0x4c
    float life = 1, age = 0;        // +0x50, +0x54
    float size = 0;                 // +0x58 corner offset (the quad spans +-size)
    float spawnTime = 0;            // +0x44 (FX clock at emission)
    C4 colour{1, 1, 1, 1};          // +0x6a.. rgb, +0x70 alpha
    float drawAlpha = 0, drawSize = 0;  // CPU particles after the step (alphaFn * a, sizeFn * size)
    float uv[4] = {0, 0, 1, 1};     // u at the -x corners, v top, u at the +x corners, v bottom
    Fade alphaFn = Fade::One, sizeFn = Fade::One;
    uint8_t motion = 2, killFlags = 1;
    bool gpu = false;               // kind 0xe: animated in the vertex shader (vh_bboard_soft_static)
};

// The range-parameter emitter (SpecialEffectsEmitter_G<P>, ctor 0x4c4730). Offsets in the comments.
struct Emitter {
    struct R { float cur = 0, lo = 0, hi = 0; };
    struct R3 { V3 cur{}, lo{}, hi{}; };
    bool stopped = true;    // +4
    bool burstDone = false; // +5
    V3 pos{}, lastEmit{}, delta{}, prevPos{};  // +0x08, +0x20, +0x34, +0x40
    int kind = 0x1f;        // +0x14 SFX SubCodeId (0xb billboard_soft, 0xe static)
    bool gpuStatic = false; // +0x4c
    int spawnMode = 1;      // +0x50 & 0xf: 1 timer, 2 distance, 3 burst
    int dieMode = 0;        // +0x54
    uint8_t killFlags = 1;  // +0x58
    uint8_t motion = 2;     // +0x59
    Fade alphaFn = Fade::One, sizeFn = Fade::One;  // +0x5c/+0x60, +0x64/+0x68
    uint32_t uvFlags = 0;   // +0x6c (0x10: random mirror in u)
    int atlas = 0;          // +0x70
    R interval{1, 1, 1};    // +0x74
    float timer = 0;        // +0x80
    R spacing{1, 1, 1};     // +0x84
    R3 vel, accel, offset;  // +0x90, +0xb4, +0xd8
    R size, life{1, 1, 1};  // +0xfc, +0x108
    C4 colour{1, 1, 1, 1}, colourA{1, 1, 1, 1}, colourB{1, 1, 1, 1};  // +0x114, +0x124, +0x134
    R angle, rotSpeed;      // +0x144, +0x150
    int perEmission = 1;    // +0x168
    float age = 0, duration = 10;  // +0x17c, +0x180
    int maxEmissions = 10, emissions = 0;  // +0x184, +0x188

    std::vector<Particle> live;  // CPU particles (+0x2c/+0x30 dynamic array)

    void start() {  // vf+0xc 0x4c4a90
        if (stopped) lastEmit = pos, stopped = burstDone = false;
    }
    void stop() {  // vf+0x10 0x4c5680: anchored (motion 1) particles freeze
        stopped = true;
        if (motion == 1)
            for (Particle& p : live) p.motion = 2;
    }
    void setPosition(const V3& p) { prevPos = pos, pos = p; }  // vf+0x14 0x4c4a60
    bool dying = false;  // PK23: +7 (vf02 0x4c56b0)
    void kill();         // PK23: vf02 0x4c56b0
};

// ---- PK23: the emitter setters (shared with core/vehicle_fx) ----
void setRange(Well512& r, Emitter::R& x, float lo, float hi);                 // a range and its current draw
void setRange3One(Well512& r, Emitter::R3& x, const V3& lo, const V3& hi);    // 0x4c4b10 + 0x4c4cf0: one random
void redraw3(Well512& r, Emitter::R3& x);                                     // per axis (0x4c4da0 / 0x4c5eb0 / 0x4c5f80)
void setColours(Well512& r, Emitter& e, const C4& a, const C4& b);            // 0x4c4bf0 + 0x4c4f70
Emitter makeEmitter(Well512& r);                                              // SfxEmitter_Ctor 0x4c4730
// ---- PK23 end

class System;

// ---- Tyre tracks: SpecialEffectsEmitter_G<SpecialEffectsRibbonTrail_G> (ctor 0x5f3050, vtable 0x9c42ec) and its
// particle SpecialEffectsRibbonTrail_G (ctor 0x5df0d0, vtable 0x9c3f00, 200 B). Kind 0x1b: decal.vhl / decal.phl.
// One ribbon is a strip of up to 32 points laid every 0.5 m along the wheel's path (step 0x5eec30); the vertex
// strip is rebuilt by the particle step 0x5df2c0 (two vertices per point, +- the half width across the path).
struct RibbonPoint {        // the four arrays the step appends to (+0x94 colour, +0x9c, +0xa4 normal, +0xb4 position)
    V3 pos{};
    C4 colour{1, 1, 1, 1};  // emitter +0x114 (the wheel's ground colour +0xe8)
    float deep = 0, tarmac = 0;  // emitter +0xb4 (wheel compression +0x64), +0xb8 (ground class 0 fraction)
    V3 normal{0, 1, 0};     // emitter +0x90 (wheel +0xfc: the vehicle's up axis, 0x5ceb10)
    float u = 0;            // the u 0x5df2c0 gave the point's vertices
};
struct Ribbon {
    std::vector<RibbonPoint> pts;
    V3 at{};                // +0x14 position (emitter + offset while it follows)
    V3 start{};             // +0x20 the emitter offset at the emission (+0xd8, 0 for the tracks)
    float halfWidth = 0.2f; // +0x58 = emitter size (+0xfc)
    float uBase = 0;        // +0x60 u at point `base`
    int base = 0;           // +0x8c first point the next rebuild starts from (n - 3)
    int prevBase = 0;       // +0x8c as the last rebuild found it
    float spacing = 0.5f;   // +0x80 (0x5df0d0)
    float uPerMetre = 0.3f; // +0x5c (0x5df0d0)
    float age = 0, life = 1;  // +0x54, +0x50 (the emitter life P5110..P5111; only used once fading)
    float colourAlpha = 1;  // +0x70
    uint8_t flags = 0x1e;   // +0x10 = emitter +0x58
    uint8_t motion = 1;     // +0x11: 1 grows with the emitter, 2 frozen
    bool fade = false;      // alpha fn 1 - t after the kill 0x5eeba0 (else 0x4beae0 = 1)
    bool dead = false;      // +0x68
};
// The decal vertex (VS_Decal, 0x38 B per vertex in FUEL): what 0x5df2c0 writes.
struct DecalVertex {
    V3 pos{};
    float u = 0, v = 0;
    V3 tangent{};          // uvscale.zw + positionRotation.w: vertex(j) - vertex(j + 2) (backwards along the strip)
    C4 rgba{1, 1, 1, 1};   // rgb = the point colour; a = 0 at the end pairs, 1 (or the fade) inside
    V3 normal{0, 1, 0};
    float lift = 0.0625f;  // normal.w: the shader moves the vertex along the normal (0x3d800000)
    float deep = 0, tarmac = 0;
};
struct TrackEmitter {
    bool stopped = true, burstDone = false;  // +4, +5
    V3 pos{}, prevPos{}, lastEmit{};
    float halfWidth = 0.2f;  // size range +0x100/+0x104 (0.8 x wheel +0x68 while running)
    C4 colour{1, 1, 1, 1};   // +0x114
    float deep = 0, tarmac = 0;
    V3 normal{0, 1, 0};
    float lifeLo = 0, lifeHi = 0;  // +0x10c / +0x110 = P5110, P5111
    std::vector<Ribbon> ribbons;
    void start() {  // 0x5f3330
        if (stopped) lastEmit = pos, stopped = burstDone = false;
    }
    void stop() {  // 0x5f41c0: the ribbons stop following
        stopped = true;
        for (Ribbon& r : ribbons) r.motion = 2;
    }
    void setPosition(const V3& p) { prevPos = pos, pos = p; }
    uint8_t flags = 0x1e;  // PK23: +0x58 (0x5f3050)
    bool dying = false;    // PK23: +7
    void kill();           // PK23: vf02 0x5eeb20
};
// PK23: 0x5f0b70: the track's last ribbon's last point snaps to `contact` (+0xac) when the track runs, the ribbon is not
// frozen, has two points or more and that point is within 2 m of the wheel emitter's position `wheelPos` (+8).
void trackHeadSnap(TrackEmitter& t, const V3& wheelPos, const V3& contact);
// 0x5df2c0's strip (two vertices per point) for one ribbon, appended to `out`.
void ribbonVertices(const Ribbon& r, std::vector<DecalVertex>& out);
// 0x5df2c0's u bookkeeping: the u of the points from +0x8c on, +0x60 / +0x8c advanced (run by every step).
void ribbonRebuild(Ribbon& r);
// 0x5df240: the u the next ribbon starts from = +0x60 + 0.3 x the ribbon's length.
float ribbonEndU(const Ribbon& r);

// ---- The wheel emitter (SpecialEffectsWheelEmitter_G 0x5efad0 / PlayerWheel 0x5cf0f0) -------------
// What the vehicle feeds every frame (0x5ceb60), per wheel.
struct WheelInput {
    V3 contact{};          // emitter position (vf+0x14)
    V3 velocity{};         // +0xc8 point velocity of the wheel
    V3 forward{};          // the wheel's rolling direction (for +0x84 / the spawn offset)
    float spin = 0;        // +0xe0 rad/s
    float radius = 0.3f;   // +0x70 / 2
    float load = 0;        // +0x64 compression; the emitters run while > 0.01
    float slip = 0;        // +0x74 (0x5da710)
    float depth = 0;       // +0xe4 water / soft depth
    std::array<float, 18> classes{};  // +0xb8 ground-class fractions
    C4 groundColour{0.5f, 0.5f, 0.5f, 1};  // +0xe8 (0x6b76a0 point colour)
    float cameraDistance = 0;
    float rain = 0;        // WeatherManager +0xb8
    // +0x24..+0x38 = veh+0x104c..0x1060 = VehiclePP 671..676 (Pn at veh+0x5d4 + 4(n-1)): size factors of
    // dust (+0x24), chunks (+0x28), stones (+0x2c), projections (+0x30), +0x34, +0x38.
    std::array<float, 6> intensity{1, 1, 1, 1, 1, 1};
    float width = 0.25f;   // +0x68 = veh+0x69c/0x6a0 (VehiclePP 51 / 52 by w & 1): the track is 0.8 x this wide
    V3 up{0, 1, 0};        // +0xfc (0x5ceb10): the vehicle's up axis, the tracks' normal
    V3 trackPos{};         // +0xac: where the track emitter sits (contact + body x offset veh+0x1630[w])
    float cameraDistSq = 0;  // emitter +0x18 (0x5f7af0: squared distance to the camera, set every frame)
};

class WheelEmitter {
public:
    // `water` = +0x3c: the per-vehicle emitter of 0x5ce530 (mist +0x48 and spray +0x50, atlas 46) instead of the
    // per-wheel set (dust, stones, projections, chunks, track).
    WheelEmitter(System& sys, bool player, bool water = false);
    void update(float dt, const WheelInput& in);
    void stopAll();
    void kill();                               // PK23: vf02 0x5f0a20
    bool dead() const { return dead_; }        // PK23: +6
    Emitter dust, stones, projection, chunks;  // +0x44, +0x58, +0x4c, +0x5c
    Emitter mist, spray;                       // +0x48, +0x50 (water form)
    TrackEmitter track;                        // +0x40 (kind 0x1b)
    bool water() const { return water_; }
    int mode = 3;  // +0xf8

private:
    void setup();
    void baseUpdate(float dt, const WheelInput& in);   // 0x5f0ef0
    void playerUpdate(const WheelInput& in);           // 0x5eefc0
    void surfaceRules(const WheelInput& in, bool rainFlag);  // 0x5f29d0
    void waterSetup();  // 0x5efc00 with +0x3c set
    System& sys_;
    bool player_, water_ = false;
    V3 pos_{}, prev_{};
    float phase_ = 0;          // +0x60 (+= 4 dt, 0x5f0b50)
    bool havePos_ = false;
    bool stopped_ = true;      // +4 of the wheel emitter
    bool dead_ = false;        // PK23: +6 (killed)
};

// ---- The effects system: clock, emitters, particles --------------------------------------------------
class System {
public:
    explicit System(const Tables& t);
    const Tables& tables() const { return t_; }
    float P(int n) const { return t_.P(n); }
    Well512& rng() { return rng_; }
    float clock() const { return clock_; }

    // Advances the clock, the wheel emitters fed by `wheels` (key = vehicle * 16 + wheel; vehicle 0 is the
    // player) and every particle. Wheels missing from the list are stopped.
    struct WheelFeed {
        int key = 0;
        bool player = false;
        bool water = false;   // the per-vehicle emitter of 0x5ce530 (key vehicle * 16 + 15)
        bool active = false;  // loaded and within 200 m (0x5ceb60 conditions)
        WheelInput in;
    };
    void update(float dt, const std::vector<WheelFeed>& wheels, const V3& eye);

    // Emitter step (0x4c53b0): moves the emitter's CPU particles (0x4c5960), then spawns.
    void runEmitter(Emitter& e, float dt, const V3& eye);
    // Everything to draw this frame: the GPU particles still alive and every emitter's CPU particles.
    void collect(std::vector<const Particle*>& out) const;
    // Every tyre-track ribbon's decal strip (0x5df2c0), as triangle-strip pieces of `stride` vertices.
    void collectTracks(std::vector<DecalVertex>& out, std::vector<uint32_t>& counts) const;
    // Track emitter step (0x5f3ef0 with the ribbon emitter's particle loop 0x5f4480 / step 0x5eec30).
    void runTrack(TrackEmitter& e, float dt, const V3& eye);
    size_t gpuCount() const { return gpu_.size(); }
    size_t wheelCount() const { return wheels_.size(); }
    const WheelEmitter* wheel(int key) const {  // the emitter fed with `key` (null when none)
        for (const Wheel& w : wheels_)
            if (w.key == key && !w.w->dead()) return w.w.get();  // PK23: not a killed one
        return nullptr;
    }

private:
    int emit(Emitter& e);                        // 0x4c56e0
    void initParticle(Emitter& e, Particle& p);  // 0x4c5a90
    void stepParticle(Emitter& e, Particle& p, float dt);  // 0x4c5ce0 + 0x5deca0
    Ribbon* emitRibbon(TrackEmitter& e);                   // 0x5f41f0 + init 0x5f4570
    void stepRibbon(TrackEmitter& e, size_t i, float dt);   // 0x5eec30
    const Tables& t_;
    Well512 rng_;
    float clock_ = 0;
    struct Wheel {
        int key = 0;
        std::unique_ptr<WheelEmitter> w;
        int idle = 0;  // frames without a feed
    };
    std::vector<Wheel> wheels_;
    std::vector<Particle> gpu_;  // fire-and-forget (FXParticlesDraw_G buckets), dead at spawn + life
};

// ---- beam-colour: the navigator beams (FX_NAVIGATOR3D_LINE, 0x4bf820; notes beam-colour.md) -----------------
// 0x4bf820 sets the ribbon particle's +0x60 = 0.05 and +0x5c = 0.9 every frame; 0x5fba20 writes vertex k's u = +0x60 +
// +0x5c k / (n - 1) (k 0 = the icon's ground, n - 1 = the band point) and v 0 / 1 for the two sides; vh_billboard passes
// uvscale.xy straight to ph_billboard. No FX atlas rect is applied (entry 0x1d's is mirrored: u0 1, du -1).
inline float navBeamU(size_t k, size_t n) { return n > 1 ? 0.05f + 0.9f * float(k) / float(n - 1) : 0.05f; }
// The particle colour: the record's rgb x 8 (0x9dbfbc, half floats +0x6a..), alpha = the record's a (Fade_One).
inline std::array<float, 4> navBeamParticleColour(const std::array<float, 4>& record) {
    return {record[0] * 8.0f, record[1] * 8.0f, record[2] * 8.0f, record[3]};
}
// ph_billboard (sfx.phl): colour = diffuse x texel, rgb x 2, sfxClip (a < 0.01 is discarded), PackHDRSample = identity
// on PC (hlsl_math.h): an unlit HDR value, alpha-blended into the scene before the exposure and the tone map.
inline std::array<float, 4> navBeamPixel(const std::array<float, 4>& particle, const std::array<float, 4>& texel) {
    return {particle[0] * texel[0] * 2.0f, particle[1] * texel[1] * 2.0f, particle[2] * texel[2] * 2.0f, particle[3] * texel[3]};
}

}  // namespace fuel::sfx
