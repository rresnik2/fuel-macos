// FUEL's sounds: Sound_Z decoding, per-vehicle sound banks and the engine
// layer model (see docs/formats.md).
#pragma once

#include "dpc.hpp"

#include <array>
#include <cstdint>
#include <map>
#include <string>
#include <unordered_set>
#include <vector>

namespace fuel {

inline constexpr uint32_t kSoundClass = 0x329ECCE8;

// Bank symbols (the game's own sound slot numbers) used by the port.
namespace snd {
inline constexpr uint32_t EngineIdle = 2;
inline constexpr uint32_t EngineOff = 2;   // + layer 1..5 -> 3..7: decelerating (off load)
inline constexpr uint32_t EngineOn = 8;    // + layer 1..5 -> 9..13: accelerating (on load)
inline constexpr uint32_t SkidFirst = 0x0e, SkidLast = 0x1e;
inline constexpr uint32_t RollFirst = 0x20, RollLast = 0x39;
inline constexpr uint32_t Wind = 0x40;
inline constexpr uint32_t ImpactFirst = 0x42, ImpactLast = 0x4b;
inline constexpr uint32_t Scrape = 0x47;  // loop: body sliding against something
inline constexpr uint32_t AbsorberFirst = 0x54, AbsorberLast = 0x59, AbsorberBig = 0x5a;  // suspension thumps
inline constexpr uint32_t BackfireFirst = 0x5e, BackfireLast = 0x61;
}  // namespace snd

struct Sound {
    uint32_t rate = 44100;
    int channels = 1;
    bool loops = false;
    std::vector<int16_t> samples;  // interleaved when stereo
    size_t frames() const { return samples.size() / size_t(channels); }
};

// Decodes the Sound_Z with this resource ID. Throws std::runtime_error when it
// is missing or malformed.
Sound decodeSound(const DpcFile& dpc, uint32_t id);

// A vehicle's sound bank: symbol -> Sound_Z resource ID.
using SoundBank = std::map<uint32_t, uint32_t>;

// Reads the bank from the "<variant>_SKEL" Lod_Z. Throws when it is missing.
SoundBank readSoundBank(const DpcFile& dpc, const std::string& variant);
// The bank of the Lod_Z whose name is `lodName` (e.g. "IT_SOUND", the Game_SoundMgr emitters' template).
SoundBank readSoundBankNamed(const DpcFile& dpc, const std::string& lodName);

// Every Lod_Z of `dpc` whose sound bank holds `symbol`: {Lod_Z id, name CRC, Sound_Z id}. FUEL's world objects
// that carry their own sounds (decor fires, 0x516cb0) have the sound at bank symbol 0.
struct LodSound {
    uint32_t lod = 0, nameCrc = 0, sound = 0;
};
std::vector<LodSound> lodSoundsWithSymbol(const DpcFile& dpc, uint32_t symbol);

// The name CRCs (asoboCrc, case-insensitive like the engine's) of the classes CommonComponent/clone.tsc declares
// with `cloneclass <Name> <baseClass>`, following clones of clones. FUEL builds a game object of that class for
// every node whose object carries the name (notes/p4-side-formats.md: IT_DECORMESH_G 609).
std::unordered_set<uint32_t> cloneClassNames(const std::string& cloneTsc, const std::string& baseClass);

// The IT_DECORMESH_G templates of clone.tsc whose Lod_Z has a sound at bank symbol 0 (their name CRCs): the objects
// IT_DECORMESH_G activation 0x516cb0 registers as sound sources.
std::unordered_set<uint32_t> decorSoundNames(const DpcFile& world, const std::string& cloneTsc);

// Engine sample mixing from VehiclesParam values (P324, P325, P636..P649), FUEL's 0x5dbed0 tables
// (built once per vehicle) read by 0x5d98c0 / 0x5d9740. Layer 0 is the idle sample, layers 1..5 the
// rpm-ranged ones; layer L's sample was recorded at nativeRpm[L].
struct EngineModel {
    static constexpr int kLayers = 6;
    float idleRpm = 1000, maxRpm = 6000, gain = 1;  // P325, P324, P649
    std::array<float, 7> nativeRpm{};               // P636..P642
    float curveA = 0, curveB = 0;                   // P647, P648 (0x5d9ca0)
    float whineGain = 0;                            // P650 (symbol 0x41)
    float skidPitchRef = 0;                         // P635
    float skidThreshold = 0;                        // P634 (veh+0xfb8): 0x5da710's dead zone and pedal scale (P103)
    // Backfire 0x5d9d60: P645 pop interval s, P646 rpm drop rate threshold in rpm/s (negative),
    // P653 engine-throttle ceiling, P654 normalised-rpm floor. Absorbers 0x5db350: P651 compression
    // speed threshold, P652 volume scale.
    float popInterval = 0.15f, popRateThreshold = -1500, popThrottleMax = 0.5f, popRpmMin = 0.5f;
    float absorberThreshold = 2.0f, absorberScale = 10.0f;
    float absorberRumbleThreshold = 2.0f, absorberRumbleScale = 10.0f;  // P668 (veh+0x1040), P669 (+0x1044): pad rumble

    struct Mix {
        std::array<float, kLayers> pitch{};   // playback rate per layer (0x5d9740: interpolated)
        std::array<float, kLayers> weight{};  // table value at index trunc(x * 100) (not interpolated)
    };
    // `layer1Gain` = global PP 2009 (*(0xa6749c)+0x1f64): the first rpm layer's rise. Throws if `params`
    // lacks the engine values.
    static EngineModel fromParams(const std::vector<float>& params, float layer1Gain);
    // Normalised rpm (rpm - P325)/(P324 - P325) before FUEL's wobble and clamp.
    float normalised(float rpm) const { return (rpm - idleRpm) / (maxRpm - idleRpm); }
    // 0x5d9ca0: the P647 / P648 curve (identity when both are 0, as in every shipped vehicle).
    float curve(float x) const;
    // Pitches and weights at clamped normalised rpm `x` (0.001..0.99).
    Mix mixAt(float x) const;
    // Convenience for checks: mixAt(clamp(normalised(rpm))).
    Mix mix(float rpm) const;
    float tableVol(int layer, int i) const { return vol_[size_t(layer)][size_t(i)]; }
    float tablePitch(int layer, int i) const { return pitch_[size_t(layer)][size_t(i)]; }

private:
    std::array<std::array<float, 100>, kLayers> pitch_{}, vol_{};
    void build(float spread, float layer1Gain);
};

}  // namespace fuel
