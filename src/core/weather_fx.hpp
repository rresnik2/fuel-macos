// FUEL's rain splash and thunder tanks (WeatherManager +0x13c SpecialEffectsSplashTank_G and +0x140
// SpecialEffectsThunder_G), decoded from FUEL.exe (notes p48-effects2.md). The tuning is read from the player's
// pp001.tsc rows (5030-5039 splashes, 5230-5246 thunder); nothing of the game is compiled in.
#pragma once

#include <array>
#include <functional>
#include <vector>

namespace fuel::wfx {

using V3 = std::array<float, 3>;
using Rnd = std::function<float()>;  // uniform [0, 1)
using RndInt = std::function<int32_t()>;
using Ground = std::function<float(float x, float z)>;  // world ground height (0x63ea40 / 0x445500)

// ---- Splash tank (ctor 0x5e0a70, vtable 0x9c3fb8; 0x94 B per splash, a ring of 3 N ground points) ----------
struct SplashParams {
    float radius = 20;     // ctor +0x38 = PP5037
    float life = 0.25f;    // PP5039: one animation cycle (16 frames)
    float lift = 0.2f;     // populate 0x5e0e90: splash +0x58 (0.2) above the ground
    std::array<float, 4> colour{4, 4, 4, 0.08f};  // constants 0x5e0f60: C_USER0 = PP5030..5033
    std::array<float, 3> scale{0.08f, 0.08f, 0.1f};  // C_USER2 = PP5034..5036 (ParticleScale)
    static SplashParams fromPP(const std::vector<float>& pp);
};

struct Splash {
    V3 pos{};          // +0x14
    float phase = 0;   // +0x50 (1 - i / N) life: positionRotation.w, the animation's time offset
    float age = 0;     // +0x54
    float flip = 0.0625f;  // +0x5c: +-1/16, the sign alternates per placement (u mirrored when positive)
};

class SplashTank {
public:
    explicit SplashTank(const SplashParams& p) : p_(p) {}
    // vf+0xc 0x5e0bc0: N splashes, a ring of 3 N points reset to the origin (0x5e0e20), then populate 0x5e0e90.
    void start(int n, const RndInt& well);
    bool started() const { return !splashes_.empty(); }
    // vf+0x20 0x5e0990: drawn = ftol(N x amount) (the weather rain +0xb8).
    void setAmount(float amount);
    // vf+0x14 0x5e10e0: ages the drawn splashes; up to 32 a frame whose cycle is over move to the next ring point,
    // re-placed at random within the radius (cube, then the ground height + lift) once the point is out of range.
    void update(float dt, const V3& tankPos, const Rnd& game, const Ground& ground);
    const std::vector<Splash>& splashes() const { return splashes_; }
    int drawn() const { return drawn_; }
    const SplashParams& params() const { return p_; }

private:
    SplashParams p_;
    std::vector<Splash> splashes_;
    std::vector<V3> ring_;  // +0x74 / +0x78
    int ringIdx_ = 0;       // +0x70
    int cursor_ = 0;        // +0x6c
    int drawn_ = 0;         // +8
};

// ---- Thunder tank (ctor 0x5e2c50, vtable 0x9c3fe8; bolts 0x44 B at +0x9c / +0xa0) ----------------------------
struct ThunderParams {
    float intervalMin = 0, intervalMax = 5;     // PP5230 / 5231: time to the next strike (0x6460f0)
    float driftMin = 10000, driftMax = 20000;   // PP5232 / 5233: the bolt velocity range (+0x34, y = 0)
    float distMin = 600, distMax = 3000;        // PP5234 / 5235: horizontal distance from the tank
    float rangeBolt = 1500, rangeCloud = 1000;  // PP5238 / 5239: bolt +0x18 (types 0, 1, 3 / type 2)
    float intensity = 3.5f;                     // PP5240: the bolt colour and the cloud flash strength
    float lifeMin = 0.5f, lifeMax = 1.2f;       // PP5241 / 5242
    float widthRatio = 0.9f;                    // PP5243: width = height x this
    float above = 200;                          // PP5245: top = tank y + this + the cloud layer height
    float second = 1;                           // PP5246: chance of a second bolt per strike
    static ThunderParams fromPP(const std::vector<float>& pp);
};

struct Bolt {
    float u0 = 0;        // +0x00 (rand % 3) / 4: the bolt picture (a quarter of ECLAIR01 in u)
    float height = 0;    // +0x04 top.y - ground.y
    float width = 0;     // +0x08 height x PP5243
    float life = 0;      // +0x0c remaining seconds
    float fadeTime = 0.3f;  // +0x10
    float frac = 1;      // +0x14 min(life, 0.3) / 0.3
    float range = 0;     // +0x18
    V3 ground{};         // +0x1c
    V3 top{};            // +0x28
    V3 vel{};            // +0x34
    int type = 0;        // +0x40: 2 = a flash that drifts inside the clouds (never drawn as a bolt)
};

// What the game plays or flashes; P49 (audio) takes the sounds, the renderer the screen flash.
struct ThunderEvent {
    enum Kind { Strike, StormStart, StormStop } kind = Strike;
    int sound = 0;       // Strike: 99 + ftol(rand x 3.9999) (0x5e36ac: sound ids 0x63..0x66); storm loop 0x67
    V3 pos{};            // Strike: the bolt top (+0x28); the sound is placed there (range 30000)
    float distance = 0;  // Strike: horizontal distance from the tank (camera)
    int type = 0;        // the bolt type; types other than 2 are visible bolts
    float flash = 0;     // Strike: the HUD flash alpha 0.45 x (1 - d / PP5235) when in front, else 0
};
using ThunderHook = std::function<void(const ThunderEvent&)>;

class ThunderTank {
public:
    explicit ThunderTank(const ThunderParams& p) : p_(p) {}
    void setAmount(float cloud) { amount_ = cloud; }  // vf+0x20 0x5e2b70: the weather cloud amount (+0xb0)
    // vf+0x14 0x5e32a0. `forward` = the camera's forward axis (0x4217d0, +0x838); `layer` = the cloud tank's
    // +0xf8. Events go to `hook` (may be empty).
    void update(float dt, const V3& tankPos, const V3& forward, float layer, const RndInt& well, const Rnd& game,
                const Ground& ground, const ThunderHook& hook);
    // 0x5e2d40: one new bolt (exposed for the checks). `at` (optional): the bolt's ground point, when the strike
    // comes from the audio port's copy of the timer (P49 Ambience::onLightning) instead of this tank's own.
    Bolt& spawn(const V3& tankPos, float layer, const RndInt& well, const Rnd& game, const Ground& ground, const V3* at = nullptr);
    // External timer: the strikes of P49's thunder (same 0x5e32a0 timer and 0x5e2d40 ring) arrive here and this
    // tank's own timer stays off, so sound and flash stay in step.
    bool external = false;
    float lastFlash = -1;  // the HUD flash alpha of the latest strike (-1: none since the caller reset it)
    void strike(const V3& groundPos) { pending_.push_back(groundPos); }
    const std::vector<Bolt>& bolts() const { return bolts_; }
    // The cloud tank's four flashes (cloud +0xa4 position, +0xd4 strength frac x PP5240, +0xe4 range 2000).
    struct Flash {
        V3 pos{};
        float strength = 0;
    };
    std::array<Flash, 4> flashes() const;
    int drawn() const { return drawn_; }       // +8: bolts drawn (types other than 2)
    float nearest() const { return nearest_; } // +0x48: distance of the nearest drawn bolt
    bool storm() const { return amount_ >= 0.95f; }
    float stormVolume() const;                 // clamp01((amount - 0.95) x 2): the loop's gain factor (0x5e2b90 x)
    float amount() const { return amount_; }
    const ThunderParams& params() const { return p_; }

private:
    ThunderParams p_;
    std::vector<Bolt> bolts_;
    float amount_ = 0;   // +0x80
    float timer_ = 0;    // +0x78
    int drawn_ = 0;
    float nearest_ = 99999.0f;
    bool stormOn_ = false;
    std::vector<V3> pending_;
    void strikeEvent(const V3& tankPos, const V3& forward, const Rnd& game, const ThunderHook& hook);
};

// 0x5e3110 + vh_lightning: one camera-facing quad per drawn bolt.
struct BoltQuad {
    V3 centre{};         // ground + (0, height / 2, 0)
    float halfW = 0, halfH = 0;
    float u0 = 0, u1 = 0;  // v runs 0 (top) .. 1
    float rgb = 3.5f;    // PP5240 in each channel (x 2 in ph_lightning)
    float alpha = 1;     // frac
};
std::vector<BoltQuad> boltQuads(const ThunderTank& t);

// ---- PK24: the screen projections (lens water / mud / dirt) --------------------------------------------------------
// SpecialEffectsManager_G +0x184 (UniqueEmitters 0x5fc500: MudProjection +0x28 0x5ffe40, WaterProjection +0x2c, DirtScreen
// +0x30). Entries are 0x94 B in FUEL; positions and sizes are clip-space units (y -1..1; x offsets are divided by the
// viewport aspect by the vertex shaders, p26-work/post.md P5). The tuning is pp001.tsc rows 5510-5523.
struct LensParams {
    float dropMin = 0, dropMax = 0.05f, dropRain = 15;  // PP5510 / 5511 / 5512: rain drop interval x (1 + (1 - rain) PP5512)
    float splashMin = 0.1f, splashMax = 0.5f, splashSpeed = 3;  // PP5513 / 5514 / 5515: wheel splash interval, minimum speed
    float dropSizeMin = 0.01f, dropSizeMax = 0.03f;      // PP5516 / 5517
    float splashSizeMin = 0.3f, splashSizeMax = 0.6f;    // PP5518 / 5519
    float mudMin = 0.7f, mudMax = 1;                     // PP5520 / 5521: mud splat interval
    float mudSizeMin = 0.3f, mudSizeMax = 0.5f;          // PP5522 / 5523
    static LensParams fromPP(const std::vector<float>& pp);
};

struct LensEntry {
    enum Kind { Mud, Water, Dirt } kind = Mud;
    std::array<float, 8> uv{};  // 4 (u, v): corners (-,-) (+,-) (+,+) (-,+) (clip space, y up)
    std::array<float, 2> size{1, 1};  // half extents (current +0x54)
    std::array<float, 2> pos{0, 0};   // current +0x5c
    std::array<float, 4> rgba{0.5f, 0.5f, 0.5f, 1};  // current +0x64
    std::array<float, 2> vel{0, 0};   // +0x7c
    float angle = 0;                  // +0x84
    float startAlpha = 1;             // +0x3c
    float spawn = 0, life = 1;        // +0x88 / +0x8c
    int atlas = 30;                   // +0x90
};

// What the player's vehicle gives the projections each frame (0x5fd270 / 0x5fd390 / 0x5ff8b0).
struct LensVehicle {
    bool present = false;                 // a player vehicle exists (DAT 0xa6625c / 0xa66260)
    V3 velocity{};                        // phys +0x524
    std::array<float, 16> waterDepth{};   // veh +0x1990 + 4 w: the wheels' water depth
    int wheels = 0;                       // veh +0x2ea8
    bool dust = false;                    // a wheel emitter (+0x1a80[w]) whose dust (+0x44) runs
    bool dirtValid = false;               // the dirt record (0x423440) exists
    float dirt = 0;                       // record +0x5c
};

class ScreenFx {
public:
    explicit ScreenFx(const LensParams& p) : p_(p) {}
    // Manager vf17 0x5f62c0 -> UniqueEmitters 0x5fd390 (water), 0x5fd270 (mud), then the projections' vf02 (cull + update,
    // mud / water / dirt in list order). `well` = Well512_Next, `atlasRect(i)` = the FX atlas entry's (u0, v0, du, dv).
    using AtlasRect = std::function<std::array<float, 4>(int)>;
    void update(float dt, float rain, const LensVehicle& veh, const RndInt& well, const AtlasRect& atlasRect);
    const std::vector<LensEntry>& entries() const { return entries_; }
    bool anyWater() const;
    float clock() const { return clock_; }
    void clear() { entries_.clear(), dirtOn_ = false; }

private:
    LensEntry& spawnBase(LensEntry::Kind k);                       // Projection vf01 0x600680
    void spawnMud(const RndInt& well);                             // 0x5fff30
    void spawnDrop(const RndInt& well, const AtlasRect& rect);    // 0x600a40
    void spawnSplash(bool a, bool b, const RndInt& well);          // 0x600d10 (a: wheels 0, 1; b: wheels 2, 3)
    void spawnDirt();                                              // 0x5ffa70: four strips
    LensParams p_;
    std::vector<LensEntry> entries_;
    float clock_ = 0;                  // manager +0xd8
    float dropTimer_ = 0, splashTimer_ = 0, mudTimer_ = 0;  // UniqueEmitters +8 / +0xc / +0x10
    bool dirtOn_ = false;
    V3 vel_{};                         // WaterProjection +0x10
};

// ---- PK24 end

}  // namespace fuel::wfx
