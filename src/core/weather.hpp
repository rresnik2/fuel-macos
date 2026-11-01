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
