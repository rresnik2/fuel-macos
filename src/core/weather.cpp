// FUEL's weather amounts; see weather.hpp and notes p30-effects.md section 5.
#include "weather.hpp"

#include "terrain.hpp"

#include <algorithm>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <map>
#include <mutex>
#include <sstream>

namespace fuel {

namespace {
float clamp01(float v) { return std::min(1.0f, std::max(0.0f, v)); }
}  // namespace

WeatherFields::WeatherFields(const DpcFile& world, const std::string& pp001Path) {
    pp_.assign(6000, 0.0f);
    if (std::ifstream f{pp001Path, std::ios::binary}) {
        std::string line;
        while (std::getline(f, line)) {
            std::istringstream ls(line);
            std::string tag;
            int n = 0;
            float v = 0;
            if (ls >> tag >> n >> v && tag == "PP" && n >= 0 && n < 6000) pp_[size_t(n)] = v;
        }
    }
    const std::vector<uint32_t> maps = genWorldMaps(world);
    // 0x77bb80: every slot with scale 1 / maxValue (table factor 1.0) and pixels per metre W x 7.142857e-06.
    auto load = [&](size_t slot, MapSampler& m) {
        if (slot >= maps.size() || maps[slot] == 0) return;
        const DpcFile::Entry* e = world.find(maps[slot]);
        if (!e) return;
        const auto blob = world.body(*e);
        if (!blob) return;
        Raster r = decodeRaster(*blob);
        m.width = r.width, m.height = r.height;
        m.pxPerMetre = float(r.width) * 7.142857e-06f;
        m.scale = 1.0f / r.maxValue;
        m.raw = std::move(r.raw);
    };
    load(24, cloud_);
    load(23, precip_);
    load(22, fog_);
    load(21, wind_);
    load(16, sand_);
}

WeatherAmounts WeatherFields::sample(double x, double y, double z) const {
    WeatherAmounts w;
    // 0x5bac10 / 0x5ba530 / 0x5b9e50: ((cell polynomial) - 0.1) x 1.25, kept within [0, 1].
    auto field = [&](const MapSampler& m) { return m.valid() ? clamp01(float((m.value(x, z) - 0.1) * 1.25)) : 0.0f; };
    w.cloud = field(cloud_);
    w.sand = sand_.valid() ? clamp01(float(sand_.value(x, z))) : 0.0f;  // 0x4573e0: the raw value, clamped by the caller
    // Rain / snow split by the reference height (PP5440..5443).
    const float h = float(y), a = P(5440), b = P(5441), c = P(5442), d = P(5443);
    float rainW = 0, snowW = 0;
    if (h < b) rainW = h <= a ? 1.0f : 1.0f - (h - a) / (b - a);
    if (a < h) {
        if (h < b) snowW = (h - a) / (b - a);
        else if (h < c) snowW = 1.0f;
        else if (h < d) snowW = 1.0f - (h - c) / (d - c);
    }
    w.precip = field(precip_);
    w.snow = snowW * w.precip;
    w.rain = w.precip * rainW;
    w.windSpeed = wind_.valid() ? float(wind_.value(x, z)) * P(5460) : 0.0f;  // 0x6b4bb0 x PP5460 (no clamp)
    // fog = max(fog field, PP5444 snow, PP5445 rain)
    w.fog = std::max(std::max(field(fog_), P(5444) * w.snow), w.rain * P(5445));
    w.cloud = clamp01(w.cloud), w.fog = clamp01(w.fog), w.rain = clamp01(w.rain), w.snow = clamp01(w.snow);
    return w;
}

namespace {
std::mutex gWeatherMu;
std::map<std::string, std::shared_ptr<const WeatherFields>> gWeatherCache;
}  // namespace

std::shared_ptr<const WeatherFields> weatherFieldsCached(const std::string& path) {
    std::lock_guard<std::mutex> lk(gWeatherMu);
    const auto it = gWeatherCache.find(path);
    return it == gWeatherCache.end() ? nullptr : it->second;
}

std::shared_ptr<const WeatherFields> weatherFieldsFor(const DpcFile& world) {
    auto& cache = gWeatherCache;
    std::lock_guard<std::mutex> lk(gWeatherMu);
    auto it = cache.find(world.path());
    if (it != cache.end()) return it->second;
    namespace fs = std::filesystem;
    const fs::path pp = fs::path(world.path()).parent_path().parent_path() / "Param" / "pp001.tsc";
    std::shared_ptr<const WeatherFields> w;
    try {
        w = std::make_shared<WeatherFields>(world, pp.string());
    } catch (const std::exception&) {
        w = nullptr;
    }
    cache[world.path()] = w;
    return w;
}

Weather weatherHook(const WeatherAmounts& w) {
    Weather h;
    h.cloud = w.cloud;
    h.fog = std::max(w.fog, 0.5f * std::max(w.rain, w.snow));
    // wet = max(rain, snow, clamp01(2 x)) with x from 0x4dde20 (U: not decoded; 0)
    h.wet = std::max(w.rain, w.snow);
    return h;
}

}  // namespace fuel
