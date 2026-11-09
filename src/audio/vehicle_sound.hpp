// FUEL's vehicle sounds, ported 1:1 from the vehicle sound code 0x5d8a40..0x5dbed0 and the game sound
// mixer Game_SoundMgr (0x4d1db0, 0x4e9b70, 0x4eaf00..0x4eb580). Decode notes: notes/p41-audio.md.
//
// Each vehicle has 0x60 sound slots (veh+0x2ef0, 0x2c bytes): bank symbol, target and current volume,
// pitch, fade-in / fade-out rates (per second, linear; 0 = jump). Every frame the category updaters set
// targets and pitches; 0x5ddfc0 fades each slot, starts its voice when the volume leaves 0 (loops are
// restarted whenever their voice is gone; one-shots fire once) and stops it at 0.
//
// Nothing of the game is compiled in: the global tuning comes from the player's pp001.tsc (PC rows), the
// surface sound table from the player's FUEL.exe (0xa06160), the samples from the vehicle archives.
#pragma once

#include "audio/sound_mixer.hpp"
#include "core/dpc.hpp"
#include "core/sound.hpp"
#include "physics/vehicle_physics.hpp"

namespace fuel::vib { class Manager; }

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace fuel::snd {

// Global sound tuning: pp001.tsc rows (global PP n = float at *(0xa6749c) + 4n) and the surface table.
struct Tuning {
    std::vector<float> pp;  // index = PP number (0..5999); rows tagged for another platform are skipped
    // 0xa06160: per ground class (18): rolling slow / fast loop symbol, skid loop symbol (-1 = none),
    // gravel rate, the global PP number that caps the skid pitch.
    struct SurfaceRow {
        int slow = -1, fast = -1, skid = -1;
        float gravelRate = 0;
        int pitchCapPP = 0;
    };
    std::array<SurfaceRow, 18> surface{};
    uint64_t surfaceHash = 0;  // FNV-1a of the 360 table bytes (checks compare whole tables by hash)
    bool ok = false;
    std::string error;
    float P(int n) const { return n >= 0 && size_t(n) < pp.size() ? pp[size_t(n)] : 0.0f; }
    // `exe` = FUEL.exe, `pp001` = CommonComponent/Param/pp001.tsc.
    static Tuning load(const std::string& exe, const std::string& pp001);
};

// Game_SoundMgr's mix block (+0x35c, targets at +0x414): a master, six faders {cur, target, up/s,
// down/s, override} and 15 category gains that move toward their targets at 2/s.
class GameMix {
public:
    enum Cat { Global, Motor, Bang, Rolling, Skid, Impact, Limiter, Absorber, Gravel, Amb0, Amb1, Amb2, Amb3, Amb4, Amb5, kCats };
    struct Fader {
        float cur = 1, tgt = 1, up = 1, down = -1, ovr = -1;
        void update(float dt);  // 0x4eaf00
    };
    explicit GameMix(const Tuning& t);
    // 0x402930 (CamChase_BuildContext, asm 0x402bbb..0x402c77): the category targets follow the camera view:
    // view 1 -> PP 2050.., view 4 -> PP 2080.., every other view (5, and the special views 8..11: start, finish,
    // wreck, page cameras) -> PP 2020.. (P103: the port used 2050 for every view but 4 and 5, so the RTC / wreck
    // cameras played the vehicle at global 1.5 x motor 1.5 instead of 1.15 x 0.9).
    static int ppBaseForView(int view) { return view == 1 ? 2050 : view == 4 ? 2080 : 2020; }
    void setView(int view);
    void update(float dt);  // 0x4eb2a0 + 0x4eb270
    // 0x4eb200 x global x fader 4 x category (the gain the updaters receive).
    float gain(Cat c) const;
    // 0x4eb580: the motor gain; the player's also takes the smallest of faders 0..3.
    float motorGain(bool player) const;
    std::array<Fader, 6> rec;
    std::array<float, kCats> cat{}, catTarget{};
    float master = 1;
    int view = 5;

private:
    const Tuning& t_;
    void loadCats(std::array<float, kCats>& out, int base) const;  // 0x4eb0a0
};

// The samples of one vehicle: bank symbol -> sample (empty when the bank lacks it).
struct VehicleBank {
    std::string variant;
    EngineModel engine;
    std::array<std::shared_ptr<const Pcm>, 0x100> bySymbol{};
    int wheels = 4;  // int(P45 + 0.5) (veh+0x2ea8)
};
// Decodes the bank of `variant` ("<V>_SKEL" Lod_Z). Throws when there is no bank or no idle sample.
VehicleBank loadVehicleBank(const DpcFile& dpc, const std::string& variant, const std::vector<float>& params, const Tuning& t);

// What the updaters read each frame (from VehicleState plus the time step).
struct VehicleInput {
    float dt = 1.0f / 60;
    float soundRpm = 0;        // veh+0x1080
    float engineThrottle = 0;  // veh+0x1088
    bool revLimiter = false;   // phys+0x142a
    Vec3f velocity;            // world m/s (phys+0x820 / veh+0x4d0)
    Vec3f position;            // world (the sound owner)
    std::array<float, 4> rotation{0, 0, 0, 1};  // chassis quaternion x, y, z, w
    Vec3f impact;              // phys+0xf2c (world)
    float impactMeter = 0;     // phys+0x138c, the crash meter of this frame (gates the impact pad rumble)
    float slipPedal = 0;       // 0x5cd8f0 (P103)
    struct Wheel {
        float load = 0, depth = 0, slip = 0, slipSpeed = 0;  // veh+0x15e8, +0x1990, 0x5da710, |0x5c8730|
        std::array<float, 18> classes{};                    // veh+0x10a8 + 0x48 w
    };
    std::vector<Wheel> wheels;
    static VehicleInput from(const VehicleState& st, float dt);
};
// P103: 0x5da710, the slip value of one wheel for the skid and gravel sounds: 0 unless loaded (> 0.01); s = slip ratio +
// 2 x pedal x k (pedal 0x5cd8f0, taken as 0 below 2 m/s of body speed), k = P634; s - k when s > k, else 0.
float soundSlip(const VehicleInput::Wheel& wh, float speed, float pedal, float k);

class VehicleSound {
public:
    static constexpr int kSlots = 0x60;
    struct Slot {
        int symbol = 0;
        float target = 0, current = 0, pitch = 1, fadeIn = 0, fadeOut = 0;
        int voice = -1;
    };
    // `player`: veh+0x1a48 (the player's vehicle runs every category; others only the engine at 0.8).
    VehicleSound(std::shared_ptr<const VehicleBank> bank, const Tuning& t, bool player, uint32_t seed = 1);
    // One frame: the updaters (0x5d8ed0) and the slots (0x5ddfc0). `mix` gets the player's fader targets.
    void update(const VehicleInput& in, GameMix& mix, Mixer& out);
    void stop(Mixer& out);
    const Slot& slot(int i) const { return slots_[size_t(i)]; }
    int slotOf(int symbol) const;  // 0x5da6e0 (-1 when absent)
    const VehicleBank& bank() const { return *bank_; }
    // P89: the pad vibrations the player's sounds ask for (0x557b40 from 0x5d9d60, 0x5d9f70, 0x5db350).
    void setVibration(vib::Manager* m) { vib_ = m; }
    // [upgrades U5, DEVIATION] the engine stage's note on the engine layers (slots 0..11): gain and low shelf (dB) of the
    // voices they start (Mixer::PlayParams::toneDb / shelfDb). 0 / 0 = FUEL's.
    void setTone(float gainDb, float shelfDb) { toneDb_ = gainDb, shelfDb_ = shelfDb; }
    // State the checks look at.
    float x() const { return x_; }        // veh+0x2eac
    float load() const { return load_; }  // veh+0x2ec4
    float rpmRate() const { return rpmRate_; }
    float popTimer() const { return popTimer_; }
    int pops() const { return pops_; }    // backfires fired so far
    float flash() const { return flash_; }  // PK23: +0x2ebc, the backfire flash (the exhaust flame level, 0x5cf2d0)
    float lastPopVolume() const { return lastPopVol_; }

private:
    std::shared_ptr<const VehicleBank> bank_;
    const Tuning& t_;
    bool player_;
    std::array<Slot, kSlots> slots_{};
    // veh+0x2ea8.. state
    float phase_ = 0, x_ = 0.001f, load_ = 0;
    float prevRpm_ = 0, rpmRate_ = 0, popTimer_ = 0, flash_ = 0;
    float impactTimer_ = 1000, prevImpactLen_ = 0;  // +0x2ec8, +0x2ecc
    Vec3f prevImpact_;                             // +0x2ed0
    float gravelAccum_ = 0, gravelTimer_ = 0, gravelInterval_ = 1;
    bool splashLatch_ = false, started_ = false;
    std::array<float, 6> absorberTimer_{}, absorberRate_{}, prevLoad_{};
    int pops_ = 0;
    float lastPopVol_ = 0;
    uint32_t rng_;
    vib::Manager* vib_ = nullptr;
    float toneDb_ = 0, shelfDb_ = 0;  // [upgrades U5]
    void rumble(const char* name, int priority, float scale);
    float rand01();  // 0x6460f0 stands in: a port LCG (the game's is a global Park-Miller stream)
    void setTarget(int symbol, float v);  // 0x5da6e0 + 0x5ddf90 (clamped to [0, 1])
    Slot* bySymbol(int symbol);
    void motor(const VehicleInput& in, float gain);
    void bang(const VehicleInput& in, float gain);
    void rolling(const VehicleInput& in, float gain);
    void skid(const VehicleInput& in, float gain, GameMix& mix);
    void gravel(const VehicleInput& in, float gain);
    void impacts(const VehicleInput& in, float gain, GameMix& mix);
    void limiter(const VehicleInput& in, float gain, Mixer& out);
    void absorbers(const VehicleInput& in, float gain);
    void process(Slot& s, float dt, const VehicleInput& in, Mixer& out);  // 0x5ddfc0
};

}  // namespace fuel::snd
