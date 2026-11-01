// FUEL's weather amounts; see weather.hpp and notes p30-effects.md section 5.
#include "weather.hpp"

#include "terrain.hpp"

#include <algorithm>
#include <cstdlib>
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
    load(2, humidity_);  // 0x4dde20 (weather object + 0x17c90 = second sampler array slot 2)
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
    w.humidity = humidity_.valid() ? clamp01(float(humidity_.value(x, z))) : 0.0f;  // 0x4dde20 ends in clamp01
    const std::array<float, 3> wd = windDirection(x, z);
    for (int i = 0; i < 3; ++i) w.wind[size_t(i)] = wd[size_t(i)] * w.windSpeed;
    // The debug menu's "Force Rain" / "Force Snow" (0x5f7f90: flags 0xa654d9 / 0xa654da; 0x5b89e0 sets rain = 1,
    // 0x5bba30 starts the tanks with amount 1): FUEL_FORCE_RAIN=1 / FUEL_FORCE_SNOW=1.
    static const bool forceRain = std::getenv("FUEL_FORCE_RAIN") != nullptr, forceSnow = std::getenv("FUEL_FORCE_SNOW") != nullptr;
    if (forceRain) w.rain = 1;
    if (forceSnow) w.snow = 1;
    // The "Weather" tweak menu's edit mode (0xa654e5) replaces the cloud amount by [0xa022b8] (0x5bba30, 0x5b7e40):
    // FUEL_FORCE_CLOUD=0..1.
    static const char* forceCloud = std::getenv("FUEL_FORCE_CLOUD");
    if (forceCloud) w.cloud = clamp01(float(std::atof(forceCloud)));
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

float cloudLayerHeight(float cloud, float pp5202, float pp5203) {
    // 0x5fe040: +0xf8 = (+0x74 - +0x78) x +0x6c + +0x78 with +0x74 = PP5202, +0x78 = PP5203, +0x6c = weather cloud.
    return (pp5202 - pp5203) * cloud + pp5203;
}

namespace {
// The CRT's rand (0x8b3440) on its per-thread seed (srand 0x8b3433).
struct MsvcRand {
    uint32_t s;
    int next() {
        s = s * 214013u + 2531011u;
        return int((s >> 16) & 0x7fff);
    }
};
float cornerAngle(int seed, bool extraDraw) {
    MsvcRand r{uint32_t(seed)};
    const float a = (float(r.next() % 1000) * 0.001f * 2.0f - 1.0f) * 3.14159f;  // 0x9dc024, 0x9963b8, 0x9dccc0
    if (extraDraw) r.next();  // the engine draws once more after the first corner (result unused)
    return a;
}
}  // namespace

std::array<float, 3> windDirection(double x, double z) {
    // 0x5b8330 (asm read): ix = ftol(x 0.001), iz = ftol(z 0.001) with the game's round-down mode.
    const float fx = float(x) * 0.001f, fz = float(z) * 0.001f;
    const int ix = int(std::floor(fx)), iz = int(std::floor(fz));
    const float a00 = cornerAngle(ix * iz, true), a10 = cornerAngle((ix + 1) * iz, false);
    const float a01 = cornerAngle((iz + 1) * ix, false), a11 = cornerAngle((iz + 1) * (ix + 1), false);
    const float tx = fx - float(ix), tz = fz - float(iz);
    const float top = a00 + (a10 - a00) * tx, bottom = (a11 - a01) * tx + a01;
    const float a = top + (fz - float(iz)) * (bottom - top);
    (void)tz;
    const float c = std::cos(a), s = std::sin(a);
    float v[3] = {c - s, 0.0f, s + c};
    const float len = std::sqrt(v[0] * v[0] + v[2] * v[2]);
    return {v[0] / len, 0.0f, v[2] / len};
}

Weather weatherHook(const WeatherAmounts& w, float cameraY, float pp5202, float pp5203) {
    // 0x5bba30 (asm read 0x5bbe7c..0x5bc040).
    Weather h;
    h.cloud = w.cloud;
    h.fog = std::max(w.fog, 0.5f * std::max(w.rain, w.snow));
    h.cloudBase = cloudLayerHeight(w.cloud, pp5202, pp5203) - 400.0f;  // cloud tank +0xf8 - 400 (0x9dbffc)
    h.cloudTopArg = cameraY;                                            // the reference position's y
    // wet = max(clamp01(2 x 0x4dde20), max(rain, snow)).
    h.wet = std::max(std::min(std::max(2.0f * w.humidity, 0.0f), 1.0f), std::max(w.rain, w.snow));
    return h;
}

}  // namespace fuel
