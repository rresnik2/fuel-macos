// FUEL's sounds: Sound_Z decoding, per-vehicle sound banks and the engine
// layer model (see docs/formats.md).
#pragma once

#include "dpc.hpp"

#include <array>
#include <cstdint>
#include <map>
#include <string>
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

// Engine sample mixing from VehiclesParam values (P324, P325, P636..P649).
// Layer 0 is the idle sample, layers 1..5 the rpm-ranged ones; layer L's
// sample was recorded at nativeRpm[L].
struct EngineModel {
    static constexpr int kLayers = 6;
    float idleRpm = 1000, maxRpm = 6000, gain = 1;  // P325, P324, P649
    std::array<float, 7> nativeRpm{};               // P636..P642
    // Backfire (P645 pop interval s, P646 rpm drop rate threshold in rpm/s (negative),
    // P653 throttle ceiling, P654 normalised-rpm floor) and absorber thumps
    // (P651 compression speed threshold m/s, P652 volume scale).
    float popInterval = 0.15f, popRateThreshold = -1500, popThrottleMax = 0.5f, popRpmMin = 0.5f;
    float absorberThreshold = 2.0f, absorberScale = 10.0f;

    struct Mix {
        std::array<float, kLayers> pitch{};   // playback rate per layer
        std::array<float, kLayers> weight{};  // sums to `gain`
    };
    // Throws if `params` lacks the engine values.
    static EngineModel fromParams(const std::vector<float>& params);
    Mix mix(float rpm) const;

private:
    std::array<std::array<float, 100>, kLayers> pitch_{}, vol_{};
    void build(float spread);
};

}  // namespace fuel
