// FUEL's weather amounts (WeatherManager_G update 0x5b89e0): procedural fields of the world generator sampled at
// the camera every frame, the rain / snow split by altitude, fog, and the arguments of the atmosphere hook
// 0x6409d0 (apply 0x5bba30). Notes p30-effects.md section 5. Nothing is copied from the game: the maps are the
// world's GenWorld_Z Binary_Z maps, the tuning rows come from the player's pp001.tsc.
#pragma once

#include "atmosphere.hpp"
#include "dpc.hpp"
#include "terrain_height.hpp"

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

private:
    MapSampler cloud_, precip_, fog_, wind_, sand_;
    std::vector<float> pp_;
};

// One cached instance per world file.
std::shared_ptr<const WeatherFields> weatherFieldsFor(const DpcFile& world);
// The instance weatherFieldsFor made for the world file at `path`, if any (no loading).
std::shared_ptr<const WeatherFields> weatherFieldsCached(const std::string& path);

// The atmosphere hook's arguments (0x5bba30 -> 0x6409d0): cloud, fogPrime = max(fog, 0.5 max(rain, snow)),
// wet = max(rain, snow, ...).
Weather weatherHook(const WeatherAmounts& w);

}  // namespace fuel
