// FUEL's weather amounts (WeatherManager_G update 0x5b89e0): procedural fields of the world generator sampled at
// the camera every frame, the rain / snow split by altitude, fog, and the arguments of the atmosphere hook
// 0x6409d0 (apply 0x5bba30). Notes p30-effects.md section 5. Nothing is copied from the game: the maps are the
// world's GenWorld_Z Binary_Z maps, the tuning rows come from the player's pp001.tsc.
#pragma once

#include "atmosphere.hpp"
#include "dpc.hpp"
#include "terrain_height.hpp"

#include <array>
#include <cmath>
#include <memory>
#include <optional>
#include <string>
#include <vector>

namespace fuel {

struct WeatherAmounts {
    float cloud = 0;   // +0xb0
    float fog = 0;     // +0xb4
    float rain = 0;    // +0xb8
    float snow = 0;    // +0xbc
    float sand = 0;    // +0xc0
    float windSpeed = 0;  // +0x124 (m/s: field x PP5460)
    float precip = 0;  // the precipitation field before the altitude split
    float humidity = 0;   // 0x4dde20: clamp01(generator map slot 2) at the reference position (the wet term)
    std::array<float, 3> wind{};  // +0xc4..+0xcc before the gusts: 0x5b8330 direction field x windSpeed
    // PK04: the pieces 0x5b89e0's overrides need again.
    float fogField = 0;   // 0x5b9e00: the fog field alone (fog = max(this, PP5444 snow, PP5445 rain))
    float cloudness = 0;  // 0x5b7e40 Cloudness: clamp01(2 x the cloud field); the override's +0xe4 when set
};

// The cloud tank's layer height (SpecialEffectsCloudTank_G +0xf8, updated by 0x5fe040): lerp(PP5203, PP5202, cloud),
// i.e. 800 m above the camera for a clear sky and 400 m when overcast. The atmosphere hook takes h0 = this - 400.
float cloudLayerHeight(float cloud, float pp5202, float pp5203);

// WeatherManager_G wind 0x5b8680: gusts on top of the 0x5b8330 field. Each frame a gust starts with probability
// 0.004 (WELL512 0x5fd710 > 0.996) while fewer than ftol(PP5466) run: amplitude U(PP5461, PP5462), length
// U(PP5463, PP5464) s; the speed is |wind| x prod(amplitude x sin(pi (t - start) / length)^PP5465), at most PP5460.
class WindGusts {
public:
    // `rnd` returns the engine's uniform [0, 1). `clock` = the game clock (s). Returns the gusted wind vector.
    template <class Rnd>
    std::array<float, 3> apply(const std::array<float, 3>& wind, double clock, const std::vector<float>& pp, Rnd&& rnd);

private:
    struct Gust { float amplitude, start, length; };
    std::vector<Gust> gusts_;
};

// The generator fields of 0x5bac10 / 0x5ba530 / 0x5b9e50 / 0x6b4bb0 / 0x4573e0: GwMapSampler slots of the second
// sampler array (GenWorld + 0x74680 + 0x1814 k; the weather object W + 0x5fa18 + 0x38e48 etc. lands on k = 24 cloud,
// 23 precipitation, 22 fog, 21 wind, 16 sand).
class WeatherFields {
public:
    WeatherFields(const DpcFile& world, const std::string& pp001Path);
    bool valid() const { return cloud_.valid() && precip_.valid() && fog_.valid(); }
    // 0x5b89e0 at the reference (camera) position, no overrides.
    WeatherAmounts sample(double x, double y, double z) const;
    float P(int n) const { return n >= 0 && size_t(n) < pp_.size() ? pp_[size_t(n)] : 0.0f; }
    // Raw field values (for the checks): slot 24 / 23 / 22 / 21 / 16.
    double rawCloud(double x, double z) const { return cloud_.value(x, z); }
    const std::vector<float>& pp() const { return pp_; }

private:
    MapSampler cloud_, precip_, fog_, wind_, sand_, humidity_;
    std::vector<float> pp_;
};

// ---- PK04: the race's weather and time of day (WeatherManager_G +0xe0..+0x11b, race +0x2d8..+0x33c).
//
// WeatherManager_G's override block (vf14 0x5b9760 clears the flags; 0x5b89e0 reads them every frame):
// +0x114 time (+0xe0, a day fraction: time of day = +0xe0 x 86400), +0x115 cloud (+0xe4), +0x116 rain (+0xec),
// +0x117 snow (+0xf0), +0x118 wind (+0x100 x PP5460; the port's wind is applied by storm_view, P65 / P83),
// +0x119 storm (cloud = 0.1 after the clamps; 0x5b99a0 on the +0x104 pool), +0x11a fog (+0xe8), +0x11b sand (+0xf4).
struct WeatherOverrides {
    bool time = false, cloud = false, rain = false, snow = false, wind = false, storm = false, fog = false, sand = false;
    float dayFraction = 0;  // +0xe0
    float cloudValue = 0;   // +0xe4
    float fogValue = 0;     // +0xe8
    float rainValue = 0;    // +0xec
    float snowValue = 0;    // +0xf0
    float sandValue = 0;    // +0xf4
    float windValue = 0;    // +0x100 (x PP5460 = m/s)
    void clear() { *this = WeatherOverrides{}; }  // vf14 0x5b9760 (flags only in FUEL; the values are rewritten before use)
    bool any() const { return time || cloud || rain || snow || wind || storm || fog || sand; }
};

// The mission's weather (mission +0x18c..+0x1e8), set by AddMissionParam 0x575b00 from hubinfos.tsc:
// AddMissionParam name ai hourStart hourEnd weatherStart fogStart weatherEnd fogEnd x y.
// Each weather token starts at -1 (.data 0xa05224); the strings go through 0x5759c0 unless NONE (SUN: cloud, rain,
// snow, wind 0; TOR; C# R# S# W# G#, percent x 0.01) and fogStart / fogEnd through atof (no scale). A flag is set
// only when the start AND the end value are >= 0 (asm 0x575e1e..0x575f99). G# lands in +0x1d4, which 0x631bd0
// copies into the SAND override (+0x11b); the fog override (+0x18d) is the two numbers. Hours (+0x1e0): x 1/24;
// an end of NONE = start + 0.05.
struct MissionWeather {
    bool tornado = false;                          // +0x18c (TOR in the start string only)
    bool fog = false;   float fog0 = 0, fog1 = 0;  // +0x18d, +0x190 / +0x194
    bool cloud = false; float cloud0 = 0, cloud1 = 0;  // +0x198, +0x19c / +0x1a0
    bool rain = false;  float rain0 = 0, rain1 = 0;    // +0x1a4, +0x1a8 / +0x1ac
    bool wind = false;  float wind0 = 0, wind1 = 0;    // +0x1b0, +0x1b4 / +0x1b8
    bool snow = false;  float snow0 = 0, snow1 = 0;    // +0x1bc, +0x1c0 / +0x1c4
    bool sand = false;  float sand0 = 0, sand1 = 0;    // +0x1d4, +0x1d8 / +0x1dc (G#)
    bool hours = false; float day0 = 0, day1 = 0;      // +0x1e0, +0x1e4 / +0x1e8 (day fractions)
};
// `args` = the command's words after the name (args[0] = mission name, [1] AI list, [2] hourStart, ...).
MissionWeather parseAddMissionParam(const std::vector<std::string>& args);
// The AddMissionParam line of mission `id` in <gameRoot>/LocalizedComponent/GameTsc/Story/hubinfos.tsc
// (case-insensitive name; the last line wins, as the console runs them in order). nullopt: no line / no file.
std::optional<MissionWeather> missionWeatherFor(const std::string& gameRoot, const std::string& id);

// The race's copy (race +0x2d8..+0x33c): setup 0x631bd0 then 0x631aa0 (race start 0x62e7ce / 0x62e7d6), lerp
// 0x632250 every race frame (0x632b50) by t = clamp(|p - start|^2 / |finish - start|^2, 0, 1) in x z.
class RaceWeather {
public:
    // 0x631bd0 + 0x631aa0. `flags` = mission +0x54 (the editor's weather scenario bits; 0 for the shipped races,
    // EMD_SetWeatherScenario being ignored, P17). `now` = the weather manager's current amounts (+0xb0, +0xb8: the
    // 0x800 scenario without values keeps them). Writes the override block; returns true when 0x5b7bb0 runs (the
    // cloud tank repopulates at once with the new cover).
    bool begin(const MissionWeather& m, uint32_t flags, const WeatherAmounts& now, WeatherOverrides& w);
    // 0x632250 (race +0x3b8 clear). `t` = raceWeatherT(start, finish, player).
    void step(float t, WeatherOverrides& w);
    bool timeOfDay() const { return time_; }
    float dayStart() const { return day0_; }   // +0x328
    float dayEnd() const { return day1_; }     // +0x32c
    float dayProgress() const { return dayT_; }  // +0x330: the furthest t reached (the clock never runs back)

private:
    bool wind_ = false, rain_ = false, snow_ = false, sand_ = false, cloud_ = false, time_ = false, fog_ = false;  // +0x2f4 +0x30c +0x318 +0x2dc +0x2e8 +0x324 +0x335
    float wind0_ = 0, wind1_ = 0, rain0_ = 0, rain1_ = 0, snow0_ = 0, snow1_ = 0, sand0_ = 0, sand1_ = 0;
    float cloud0_ = 0, cloud1_ = 0, day0_ = 0, day1_ = 0, dayT_ = 0, fog0_ = 0, fog1_ = 0;
};
// 0x632250's t: 0 when start = finish; > 1 -> 1 (no lower clamp needed: a ratio of squares).
float raceWeatherT(const std::array<float, 3>& start, const std::array<float, 3>& finish, const std::array<double, 3>& p);
// 0x631aa0's table for the scenario bits 2-3 / 4-5: 0.292, 0.458, 0.6667, 0.9 = 07:00:29, 10:59:31, 16:00:03, 21:36
// (stack floats 0x3e958106, 0x3eea7efa, 0x3f2aacda, 0x3f666666).
float scenarioDayFraction(int index);
// 0x5b89e0's overrides on the sampled amounts (after the field sample and the debug forces; before the clamps):
// cloud +0xe4, sand +0xf4, rain +0xec, snow +0xf0, fog +0xe8 else max(fog field, PP5444 snow, PP5445 rain) with the
// overridden rain / snow, the clamps, then storm: cloud = 0.1. Cloudness (0x5b7e40) = +0xe4 with the cloud override.
// The wind override is not applied here (storm_view's hook, P65 / P83).
void applyOverrides(WeatherAmounts& w, const WeatherOverrides& o, float pp5444, float pp5445);
// 0x5b89e0 +0xac: the race's fixed time of day (+0xe0 x 86400) when +0x114 is set, else < 0.
inline double overrideTimeOfDay(const WeatherOverrides& o) { return o.time ? double(float(o.dayFraction * 86400.0f)) : -1.0; }
// ---- end PK04

// One cached instance per world file.
std::shared_ptr<const WeatherFields> weatherFieldsFor(const DpcFile& world);
// The instance weatherFieldsFor made for the world file at `path`, if any (no loading).
std::shared_ptr<const WeatherFields> weatherFieldsCached(const std::string& path);

// 0x5b8330: the wind direction at (x, z): one random angle per 1 km lattice corner (MSVC srand(ix iz) etc., rand %
// 1000 -> (2 r / 1000 - 1) 3.14159), bilinear in the cell, direction normalize(cos a - sin a, 0, sin a + cos a).
std::array<float, 3> windDirection(double x, double z);

// The atmosphere hook's arguments (0x5bba30 -> 0x6409d0): cloud, fogPrime = max(fog, 0.5 max(rain, snow)),
// h0 = cloud tank height - 400, h0 + 2000, the camera height, wet = max(rain, snow, clamp01(2 humidity)).
Weather weatherHook(const WeatherAmounts& w, float cameraY, float pp5202, float pp5203);

template <class Rnd>
std::array<float, 3> WindGusts::apply(const std::array<float, 3>& wind, double clock, const std::vector<float>& pp, Rnd&& rnd) {
    auto P = [&](int n) { return n >= 0 && size_t(n) < pp.size() ? pp[size_t(n)] : 0.0f; };
    const int maxGusts = int(std::floor(P(5466)));  // ftol under _RC_DOWN
    if (rnd() > 0.996f && int(gusts_.size()) < maxGusts) {
        const float a = P(5461) + rnd() * (P(5462) - P(5461));  // the engine draws the amplitude first
        const float l = (P(5464) - P(5463)) * rnd() + P(5463);
        gusts_.push_back({a, float(clock), l});
    }
    float f = 1.0f;
    if (!gusts_.empty()) {
        for (size_t i = 0; i < gusts_.size();) {
            const Gust& g = gusts_[i];
            if (float(clock) < g.length + g.start) {
                f *= std::pow(std::sin((float(clock) - g.start) / g.length * 3.1415927f), P(5465)) * g.amplitude;
                ++i;
            } else {
                gusts_.erase(gusts_.begin() + long(i));
            }
        }
        if (f <= 0) f = 0;
    }
    std::array<float, 3> w = wind;
    const float len2 = w[0] * w[0] + w[1] * w[1] + w[2] * w[2], len = std::sqrt(len2);
    if (1e-6f < len2)
        for (float& c : w) c /= len;
    float speed = len * f;
    if (speed <= 0 || P(5460) < speed) speed = speed <= 0 ? 0.0f : P(5460);
    for (float& c : w) c *= speed;
    return w;
}

}  // namespace fuel
