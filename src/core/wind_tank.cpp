// P65: FUEL's wind and tornado-debris tanks and the race weather (see wind_tank.hpp).
#include "wind_tank.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstring>

namespace fuel::storm {

WindTankLook WindTankLook::load(const std::function<float(int)>& P, bool debris) {
    WindTankLook l;
    if (debris) {  // 0x5e71a0 / 0x5e7280 / 0x5e7400; 0x5bb680 starts it with vf+0xc(0x200), atlas 0x1c
        l.count = 0x200, l.atlas = 0x1c;
        l.scale = P(5503), l.particle = P(5502), l.frequency = P(5501), l.amplitude = P(5500), l.distRatio = P(5506);
        l.speedMin = P(5504), l.speedMax = P(5505);
    } else {  // 0x5e4560 / 0x5e4650 / 0x5e47e0; vf+0xc(0x100), atlas 8
        l.count = 0x100, l.atlas = 8;
        l.scale = P(5273), l.particle = P(5272), l.frequency = P(5271), l.amplitude = P(5270), l.distRatio = 0.6f;  // vh_wind's own 0.6
        l.speedMin = P(5274), l.speedMax = P(5275);
    }
    return l;
}

std::array<float, 2> windTankAmounts(float windSpeed, float pp5460) {
    // 0x5bba30: w = clamp01(|wind| / PP5460); a (+0x148) and b (+0x154).
    float w = pp5460 > 0 ? std::fabs(windSpeed) / pp5460 : 0.0f;
    w = std::clamp(w, 0.0f, 1.0f);
    float a = 0, b = 0;
    if (w < 0.3f) {
        a = w * 3.3333333f;
    } else if (w < 0.6f) {
        b = (w - 0.3f) * 1.4285715f;
        a = 1.0f - b;
    } else {
        b = (w - 0.6f) * 2.5000002f;
    }
    return {a, b};
}

WindTank::WindTank(const WindTankLook& look, uint32_t seed) : look_(look) {
    // vf07 (0x5e7590 / 0x5e4a3b): Rand01 draws (FUEL: Rand01_ParkMillerShuffle; [port] Park-Miller here) w, the column,
    // z, y, x, each 2 r - 1.
    uint32_t s = seed ? seed : 1u;
    auto r = [&]() {
        s = uint32_t((uint64_t(s) * 48271u) % 2147483647u);
        return float(s - 1) / 2147483646.0f;
    };
    parts_.resize(size_t(std::max(0, look.count)));
    for (Particle& p : parts_) {
        p.w = r() * 2.0f - 1.0f;
        p.column = std::floor(std::min(r() * 4.0f, 3.0f));  // [I] _ftol(r x 4): the four pieces of the sheet
        p.z = r() * 2.0f - 1.0f;
        p.y = r() * 2.0f - 1.0f;
        p.x = r() * 2.0f - 1.0f;
    }
}

void WindTank::update(float dt, const std::array<float, 3>& wind) {
    // vf05: speed = clamp(|wind|, lo, hi) along the wind; the displacement moves 0.3 of it.
    const float len = std::sqrt(wind[0] * wind[0] + wind[1] * wind[1] + wind[2] * wind[2]);
    const float sp = std::clamp(len, look_.speedMin, look_.speedMax);
    clock_ += dt;  // P90 (smooth mode only)
    if (len <= 0) return;
    for (int k = 0; k < 3; ++k) disp_[size_t(k)] += wind[size_t(k)] / len * sp * dt * 0.3f;
    // P90 [DEVIATION]: the smooth mode drifts the same vector in metres (/ TankScale), 3-6 m/s instead of 210-420.
    if (look_.scale > 0)
        for (int k = 0; k < 3; ++k) smoothDisp_[size_t(k)] += wind[size_t(k)] / len * sp * dt * 0.3f / look_.scale;
}

WindTank::Placed WindTank::place(const Particle& q, const std::array<float, 3>& adj) const {
    Placed o{};
    const float sc = look_.scale;
    const float cur[3] = {q.x + disp_[0], q.y + disp_[1], q.z + disp_[2]};
    for (int k = 0; k < 3; ++k) {
        const float m = cur[k] - adj[size_t(k)] / sc;
        const float sg = m > 0 ? 1.0f : (m < 0 ? -1.0f : 0.0f);
        const float f = std::fmod(m + sg, 2.0f);  // HLSL fmod: the sign of the dividend
        o.pos[size_t(k)] = (f - sg) * sc + adj[size_t(k)];
        o.cell[size_t(k)] = int(std::floor((m + 1.0f) * 0.5f));
    }
    const float len = std::sqrt(cur[0] * cur[0] + cur[1] * cur[1] + cur[2] * cur[2]);
    o.angle = look_.frequency * len + look_.frequency * q.w;
    const float s = std::sin(o.angle), c = std::cos(o.angle);
    o.pos[0] += look_.amplitude * (c - s), o.pos[1] += look_.amplitude * (s - c), o.pos[2] += look_.amplitude * (s + c);
    return o;
}

WindTank::Placed WindTank::placeSmooth(const Particle& q, const std::array<float, 3>& adj, const SmoothDebris& sd) const {
    // P90 [DEVIATION from FUEL] (see DebrisMotion): FUEL's MapTankCoordinate box on the metre drift, the 40 m wave
    // frozen at its starting phase plus a small swirl, and a fade towards the box faces.
    Placed o{};
    const float sc = look_.scale;
    const float cur[3] = {q.x + smoothDisp_[0], q.y + smoothDisp_[1], q.z + smoothDisp_[2]};
    float edge = 0;
    for (int k = 0; k < 3; ++k) {
        const float m = cur[k] - adj[size_t(k)] / sc;
        const float sg = m > 0 ? 1.0f : (m < 0 ? -1.0f : 0.0f);
        const float local = std::fmod(m + sg, 2.0f) - sg;  // (-1, 1): the place in the box
        o.pos[size_t(k)] = local * sc + adj[size_t(k)];
        o.cell[size_t(k)] = int(std::floor((m + 1.0f) * 0.5f));
        edge = std::max(edge, std::fabs(local));
    }
    o.alpha = sd.fadeWidth > 0 ? std::clamp((1.0f - edge) * sc / sd.fadeWidth, 0.0f, 1.0f) : 1.0f;
    const float f = look_.frequency, a = look_.amplitude;
    const float a0 = f * std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z) + f * q.w;  // FUEL's angle at displacement 0
    const float rate = sd.swirlRate * (0.5f + std::fabs(q.w)) * (q.w < 0 ? -1.0f : 1.0f);
    o.angle = a0 + rate * clock_;
    const float s0 = std::sin(a0), c0 = std::cos(a0), s = std::sin(o.angle), c = std::cos(o.angle);
    const float r = sd.swirlRadius;
    o.pos[0] += a * (c0 - s0) + r * (c - s), o.pos[1] += a * (s0 - c0) + r * (s - c), o.pos[2] += a * (s0 + c0) + r * (s + c);
    return o;
}

DebrisMotion debrisMotionFrom(const std::string& v, DebrisMotion fallback) {
    std::string u;
    for (char c : v)
        if (!std::isspace(static_cast<unsigned char>(c))) u += char(std::tolower(static_cast<unsigned char>(c)));
    if (u == "fuel") return DebrisMotion::Fuel;
    if (u == "smooth") return DebrisMotion::Smooth;
    return fallback;
}

const char* debrisMotionName(DebrisMotion m) { return m == DebrisMotion::Fuel ? "fuel" : "smooth"; }

float debrisFuelWeight(DebrisMotion m, float dist, const SmoothDebris& s) {
    if (m == DebrisMotion::Fuel) return 1.0f;
    if (dist < 0) return 0.0f;  // no tornado
    if (s.fuelFar <= s.fuelNear) return dist <= s.fuelNear ? 1.0f : 0.0f;
    const float t = std::clamp((dist - s.fuelNear) / (s.fuelFar - s.fuelNear), 0.0f, 1.0f);
    return 1.0f - t * t * (3.0f - 2.0f * t);
}

int WindTank::drawn(float amount) const {
    return std::clamp(int(std::floor(std::clamp(amount, 0.0f, 1.0f) * float(parts_.size()))), 0, int(parts_.size()));
}

WeatherToken parseWeatherToken(const std::string& in) {
    std::string u = in;
    for (char& c : u) c = char(std::toupper(static_cast<unsigned char>(c)));
    WeatherToken t;
    if (u.find("SUN") != std::string::npos) t.sun = true, t.cloud = 0.0f, t.rain = 0.0f, t.snow = 0.0f, t.wind = 0.0f;
    if (u.find("TOR") != std::string::npos) t.tornado = true;
    auto num = [&](const char* key, std::optional<float>& out) {
        const size_t p = u.find(key);
        if (p != std::string::npos) out = float(std::atof(u.c_str() + p + 2) * 0.01);  // _atof x 0.01
    };
    num("C#", t.cloud), num("R#", t.rain), num("S#", t.snow), num("W#", t.wind), num("G#", t.fog);
    return t;
}

float raceWeatherProgress(const std::array<float, 3>& start, const std::array<float, 3>& finish, const std::array<double, 3>& p) {
    const double fx = double(start[0]) - finish[0], fz = double(start[2]) - finish[2];
    const double total = fx * fx + fz * fz;
    if (total == 0.0) return 0.0f;
    const double px = double(start[0]) - p[0], pz = double(start[2]) - p[2];
    const double t = (px * px + pz * pz) / total;
    return t > 1.0 ? 1.0f : float(t);
}

std::optional<float> raceWind(const std::string& start, const std::string& end, float t) {
    // AddMissionParam (asm 0x575f70..0x575f99): both tokens start at -1 (.data 0xa05224); the mission's wind flag
    // +0x1b0 is set only when the start AND the end value are >= 0 (each from W# or SUN; "NONE" is not parsed).
    // 0x631bd0 copies +0x1b4 / +0x1b8 to the race (+0x2f8 / +0x2fc) and sets the weather's +0x118 only with it.
    const WeatherToken a = parseWeatherToken(start), b = parseWeatherToken(end);
    const float w0 = a.wind.value_or(-1.0f), w1 = b.wind.value_or(-1.0f);
    if (!(w0 >= 0.0f) || !(w1 >= 0.0f)) return std::nullopt;
    return w1 * t + (1.0f - t) * w0;  // 0x632250
}

}  // namespace fuel::storm
