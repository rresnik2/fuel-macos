// FUEL's weather amounts; see weather.hpp and notes p30-effects.md section 5.
#include "weather.hpp"

#include "terrain.hpp"
#include "console.hpp"

#include <cctype>
#include <cstring>
#include <strings.h>

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
    w.fogField = field(fog_);  // PK04: kept for the fog rule under the race overrides
    w.fog = std::max(std::max(w.fogField, P(5444) * w.snow), w.rain * P(5445));
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
    // PK04: 0x5b7e40 Cloudness = clamp01(2 x the cloud field), or the edit mode's [0xa022b8] as it is.
    w.cloudness = forceCloud ? clamp01(float(std::atof(forceCloud))) : clamp01(2.0f * w.cloud);
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

// ---- PK04: race weather (see weather.hpp)
namespace {
std::string upperCopy(std::string s) {
    for (char& c : s) c = char(std::toupper(static_cast<unsigned char>(c)));
    return s;
}
bool isNone(const std::string& s) { return strcasecmp(s.c_str(), "NONE") == 0; }
// 0x5759c0 into a token record (+0 TOR, +4 C, +8 R, +0xc S, +0x10 W, +0x18 G), every value -1 to start with.
struct Tok {
    bool tor = false;
    float c = -1, r = -1, s = -1, w = -1, g = -1;
};
void parseTok(const std::string& in, Tok& t) {
    const std::string u = upperCopy(in);  // _strupr on a copy
    if (u.find("SUN") != std::string::npos) t.c = t.r = t.s = t.w = 0;
    if (u.find("TOR") != std::string::npos) t.tor = true;
    auto pct = [&](const char* key, float& v) {
        const size_t at = u.find(key);
        if (at != std::string::npos) v = float(std::atof(u.c_str() + at + 2) * 0.01);  // float10 x 0.01
    };
    pct("C#", t.c), pct("R#", t.r), pct("S#", t.s), pct("W#", t.w), pct("G#", t.g);
}
// The console's word splitter for the .tsc lines: blanks separate, double quotes group (and are dropped).
std::vector<std::string> words(const std::string& line) {
    std::vector<std::string> out;
    size_t i = 0;
    while (i < line.size()) {
        while (i < line.size() && std::isspace(static_cast<unsigned char>(line[i]))) ++i;
        if (i >= line.size()) break;
        std::string w;
        if (line[i] == '"') {
            const size_t e = line.find('"', i + 1);
            w = line.substr(i + 1, e == std::string::npos ? std::string::npos : e - i - 1);
            i = e == std::string::npos ? line.size() : e + 1;
        } else {
            while (i < line.size() && !std::isspace(static_cast<unsigned char>(line[i]))) w += line[i++];
        }
        out.push_back(w);
    }
    return out;
}
}  // namespace

MissionWeather parseAddMissionParam(const std::vector<std::string>& a) {
    auto arg = [&](size_t i) { return i < a.size() ? a[i] : std::string("NONE"); };
    MissionWeather m;
    // Hours (asm before 0x575ce2): +0x1e0 = 1, +0x1e4 = atof x 1/24 (float10 0.041666666666666664), end the same or
    // start + 0.05.
    if (!isNone(arg(2))) {
        m.hours = true;
        m.day0 = float(std::atof(arg(2).c_str()) * 0.041666666666666664);
        m.day1 = isNone(arg(3)) ? float(double(m.day0) + 0.05) : float(std::atof(arg(3).c_str()) * 0.041666666666666664);
    }
    Tok s, e;
    float fog0 = -1, fog1 = -1;
    if (!isNone(arg(4))) parseTok(arg(4), s);
    if (!isNone(arg(5))) fog0 = float(std::atof(arg(5).c_str()));
    if (!isNone(arg(6))) parseTok(arg(6), e);
    if (!isNone(arg(7))) fog1 = float(std::atof(arg(7).c_str()));
    // args 8 / 9 only reach the unused st(0) and the never-set +0x1c8 record.
    m.tornado = s.tor;  // 0x575e1e: the START record's TOR only
    auto both = [](float a0, float a1, bool& f, float& v0, float& v1) {
        if (a0 >= 0 && a1 >= 0) f = true, v0 = a0, v1 = a1;
    };
    both(s.g, e.g, m.sand, m.sand0, m.sand1);
    both(fog0, fog1, m.fog, m.fog0, m.fog1);
    both(s.c, e.c, m.cloud, m.cloud0, m.cloud1);
    both(s.r, e.r, m.rain, m.rain0, m.rain1);
    both(s.s, e.s, m.snow, m.snow0, m.snow1);
    both(s.w, e.w, m.wind, m.wind0, m.wind1);
    return m;
}

std::optional<MissionWeather> missionWeatherFor(const std::string& gameRoot, const std::string& id) {
    namespace fs = std::filesystem;
    std::ifstream f(fs::path(gameRoot) / "LocalizedComponent" / "GameTsc" / "Story" / "hubinfos.tsc");
    if (!f) return std::nullopt;
    std::optional<MissionWeather> out;
    std::string line;
    while (std::getline(f, line)) {
        std::vector<std::string> t = words(line);
        if (t.size() < 2 || !consoleCommandIs(t[0], "AddMissionParam") || strcasecmp(t[1].c_str(), id.c_str()) != 0) continue;
        out = parseAddMissionParam(std::vector<std::string>(t.begin() + 1, t.end()));
    }
    return out;
}

float scenarioDayFraction(int i) {
    static const uint32_t bits[4] = {0x3e958106u, 0x3eea7efau, 0x3f2aacdau, 0x3f666666u};  // 0x631aa0 locals
    float f;
    std::memcpy(&f, &bits[i & 3], 4);
    return f;
}

bool RaceWeather::begin(const MissionWeather& m, uint32_t flags, const WeatherAmounts& now, WeatherOverrides& w) {
    // 0x631bd0
    dayT_ = 0;  // +0x330
    w.clear();  // Weather_ClearOverrides (vf14 0x5b9760)
    const bool scen = (flags & 0x800) != 0;
    bool repopulate = false;
    wind_ = scen || m.wind;
    if (wind_) {
        w.wind = true;
        wind0_ = wind1_ = 0;
        if (m.wind) wind0_ = m.wind0, wind1_ = m.wind1, w.windValue = wind0_;
    }
    rain_ = scen || m.rain;
    if (rain_) {
        w.rain = true;
        if (m.rain) {
            rain0_ = m.rain0, rain1_ = m.rain1, w.rainValue = rain0_;
        } else if ((flags & 0x180) == 0) {
            rain0_ = rain1_ = now.rain, w.rainValue = 0;
        } else {
            rain0_ = (flags & 0x80) ? 1.0f : 0.0f;
            rain1_ = (flags & 0x100) ? 1.0f : 0.0f;
            w.rainValue = rain0_;
        }
    }
    snow_ = scen || m.snow;
    if (snow_) {
        w.snow = true;
        snow0_ = snow1_ = 0;
        if (m.snow) snow0_ = m.snow0, snow1_ = m.snow1, w.snowValue = snow0_;  // 0x632250 never lerps the snow
    }
    sand_ = true;  // +0x2dc = 1 on both paths; without G# the values stay the race object's (0) and the flag is off
    if (m.sand) w.sand = true, sand0_ = m.sand0, sand1_ = m.sand1, w.sandValue = sand0_;
    cloud_ = scen || m.cloud;
    if (cloud_) {
        w.cloud = true;
        if (m.cloud) {
            cloud0_ = m.cloud0, cloud1_ = m.cloud1, w.cloudValue = cloud0_, repopulate = true;
        } else if ((flags & 0x600) == 0) {
            cloud0_ = cloud1_ = now.cloud, w.cloudValue = 0;
        } else {
            cloud0_ = (flags & 0x200) ? 1.0f : 0.0f;
            cloud1_ = (flags & 0x400) ? 1.0f : 0.0f;
            w.cloudValue = cloud0_, repopulate = true;
        }
    }
    fog_ = (flags & 0x10000) != 0 || m.fog;
    if (fog_) {
        static const float table[4] = {0.0f, 0.33f, 0.66f, 1.0f};
        w.fog = true;
        if (m.fog) fog0_ = m.fog0, fog1_ = m.fog1;
        else fog0_ = table[(flags >> 12) & 3], fog1_ = table[(flags >> 14) & 3];
        w.fogValue = fog0_;
    }
    if ((flags & 1) || m.tornado) w.storm = true;  // + the tornado route 0x6313b0 (storms, P61)
    // 0x631aa0 (after the clear above, so +0x114 survives)
    w.time = false;
    time_ = (flags & 2) != 0 || m.hours;
    if (time_) {
        w.time = true;
        if (m.hours) day0_ = m.day0, day1_ = m.day1;
        else day0_ = scenarioDayFraction(int(flags >> 2)), day1_ = scenarioDayFraction(int(flags >> 4));
        w.dayFraction = day0_;
    }
    return repopulate && w.cloud;  // 0x5b7bb0: tank vf+0x10 when the tank exists and +0x115 is set
}

void RaceWeather::step(float t, WeatherOverrides& w) {
    // 0x632250 (operand order as the asm)
    if (wind_) w.windValue = wind1_ * t + (1.0f - t) * wind0_;
    if (rain_) w.rainValue = rain1_ * t + (1.0f - t) * rain0_;
    if (cloud_) w.cloudValue = cloud1_ * t + (1.0f - t) * cloud0_;
    if (sand_) w.sandValue = sand1_ * t + (1.0f - t) * sand0_;
    if (time_) {
        if (dayT_ <= t) dayT_ = t;
        w.dayFraction = (1.0f - dayT_) * day0_ + day1_ * dayT_;
    }
    if (fog_) w.fogValue = (1.0f - t) * fog0_ + fog1_ * t;
}

float raceWeatherT(const std::array<float, 3>& start, const std::array<float, 3>& finish, const std::array<double, 3>& p) {
    const float dx = start[0] - finish[0], dz = start[2] - finish[2];
    const float den = dz * dz + dx * dx;
    const float px = start[0] - float(p[0]), pz = start[2] - float(p[2]);
    if (den == 0.0f) return 0.0f;
    const float t = (pz * pz + px * px) / den;
    return t > 1.0f ? 1.0f : t;
}

void applyOverrides(WeatherAmounts& w, const WeatherOverrides& o, float pp5444, float pp5445) {
    static const char* forceCloud = std::getenv("FUEL_FORCE_CLOUD");
    if (o.cloud) w.cloud = o.cloudValue, w.cloudness = clamp01(o.cloudValue);  // 0x5b7e40 takes +0xe4 as it is
    if (forceCloud) w.cloud = w.cloudness = clamp01(float(std::atof(forceCloud)));  // the edit mode wins (0x5bba30)
    if (o.sand) w.sand = o.sandValue;
    if (o.rain) w.rain = o.rainValue;
    if (o.snow) w.snow = o.snowValue;
    w.fog = o.fog ? o.fogValue : std::max(std::max(w.fogField, pp5444 * w.snow), w.rain * pp5445);
    w.cloud = clamp01(w.cloud), w.sand = clamp01(w.sand), w.fog = clamp01(w.fog), w.rain = clamp01(w.rain), w.snow = clamp01(w.snow);
    if (o.storm) w.cloud = 0.1f;  // 0x3dcccccd, after the clamps; Cloudness unchanged
}
// ---- end PK04

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
