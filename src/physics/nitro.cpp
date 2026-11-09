// [DEVIATION, the user's request 2026-11-06] Nitrous: the tank, the boost's numbers and the bonus detectors
// (nitro.hpp; notes ~/re-work/fuel/notes/nitro.md).
#include "physics/nitro.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <string>

namespace fuel::nitro {

namespace {
Params gParams;
bool envOr(bool def);
bool gEnabled = envOr(true);  // settings.txt `nitrous` (applySettings) replaces it; FUEL_NITRO always wins

std::string lower(const char* s) {
    std::string r = s ? s : "";
    for (char& c : r) c = char(std::tolower(static_cast<unsigned char>(c)));
    while (!r.empty() && std::isspace(static_cast<unsigned char>(r.back()))) r.pop_back();
    size_t a = 0;
    while (a < r.size() && std::isspace(static_cast<unsigned char>(r[a]))) ++a;
    return r.substr(a);
}
float clamp01(float x) { return x < 0 ? 0.0f : (x > 1 ? 1.0f : x); }
bool parseOnOff(const char* s, bool d) {
    if (!s) return d;
    const std::string t = lower(s);
    if (t == "on" || t == "1" || t == "true" || t == "yes") return true;
    if (t == "off" || t == "0" || t == "false" || t == "no") return false;
    return d;
}
bool envOr(bool def) { return parseOnOff(std::getenv("FUEL_NITRO"), def); }
}  // namespace

Params& params() {
    static const bool once = [] {
        if (const char* e = std::getenv("FUEL_NITRO_PARAMS")) applyOverrides(gParams, e);
        return true;
    }();
    (void)once;
    return gParams;
}

int applyOverrides(Params& p, const char* spec) {
    struct F {
        const char* name;
        float Params::*field;
    };
    static const F kFields[] = {
        {"tankSeconds", &Params::tankSeconds}, {"rechargeRate", &Params::rechargeRate}, {"rechargeDelay", &Params::rechargeDelay},
        {"startLevel", &Params::startLevel}, {"minStart", &Params::minStart}, {"power", &Params::power},
        {"torqueGain", &Params::torqueGain}, {"thrust", &Params::thrust}, {"topSpeedLift", &Params::topSpeedLift},
        {"dragRelease", &Params::dragRelease}, {"tractionCut", &Params::tractionCut}, {"rampUp", &Params::rampUp},
        {"rampDown", &Params::rampDown}, {"airMin", &Params::airMin}, {"airBase", &Params::airBase}, {"airRate", &Params::airRate},
        {"airMax", &Params::airMax}, {"slideMinDeg", &Params::slideMinDeg}, {"slideMaxDeg", &Params::slideMaxDeg},
        {"slideMinSpeed", &Params::slideMinSpeed}, {"slideHold", &Params::slideHold}, {"slideRate", &Params::slideRate},
        {"overtake", &Params::overtake}, {"overtakeHold", &Params::overtakeHold}, {"nearMiss", &Params::nearMiss},
        {"nearMissGap", &Params::nearMissGap}, {"nearMissSpeed", &Params::nearMissSpeed}, {"nearMissCooldown", &Params::nearMissCooldown},
        {"aiSpeedGain", &Params::aiSpeedGain}, {"fovKickDeg", &Params::fovKickDeg}, {"blur", &Params::blur}, {"flame", &Params::flame},
    };
    int n = 0;
    std::string s = spec ? spec : "";
    size_t at = 0;
    while (at < s.size()) {
        size_t end = s.find(',', at);
        if (end == std::string::npos) end = s.size();
        const std::string item = s.substr(at, end - at);
        at = end + 1;
        const size_t eq = item.find('=');
        if (eq == std::string::npos) continue;
        const std::string key = item.substr(0, eq);
        bool found = false;
        for (const F& f : kFields)
            if (key == f.name) p.*(f.field) = std::strtof(item.c_str() + eq + 1, nullptr), found = true, ++n;
        if (!found) std::fprintf(stderr, "nitro: unknown parameter %s\n", key.c_str());
    }
    return n;
}
bool enabled() { return gEnabled; }
void setEnabled(bool on) { gEnabled = on; }

bool enabledFrom(const char* v, bool def) { return envOr(parseOnOff(v, def)); }

const char* actionDisplayName() { return "Nitro"; }

bool scriptedHold(const char* spec, double t) {
    for (const char* q = spec; q && *q;) {
        char* end = nullptr;
        const double a = std::strtod(q, &end);
        if (end == q) break;
        double b = a;
        if (*end == '-') b = std::strtod(end + 1, &end);
        if (t >= a && t < b) return true;
        q = std::strchr(end, ',');
        if (q) ++q;
    }
    return false;
}

const char* kindName(int k) {
    switch (k) {
        case KindAir: return "air";
        case KindSlide: return "slide";
        case KindOvertake: return "overtake";
        case KindNearMiss: return "near miss";
        default: return "none";
    }
}

// ---- Tank ------------------------------------------------------------------------------------------------------------

void Tank::reset(const Params& p) {
    *this = Tank{};
    level = clamp01(p.startLevel);
}

void Tank::step(float dt, bool want, bool grounded, bool allowed, const Params& p) {
    const bool pressed = want && !held;
    held = want;
    if (!want) needRelease = false;
    bool engage = false;
    if (want && allowed && !needRelease) {
        engage = active ? level > 0 : level >= p.fraction(p.minStart) - 1e-6f;  // starting needs a little fuel, going on any
        if (!engage && pressed) emptyFlash = 0.6f;          // pressed with too little: the gauge flashes
    }
    if (engage && !active) ++starts;
    active = engage;
    firing = active && grounded;
    if (firing) {
        const float before = level;
        level = std::max(0.0f, level - dt / std::max(p.tankSeconds, 0.05f));
        boosted += dt;
        if (level <= 0 && before > 0) {  // ran dry: stop, flash, wait for the button to be let go
            needRelease = true;
            emptyFlash = 0.6f;
        }
    }
    idle = active ? 0.0f : idle + dt;
    if (idle >= p.rechargeDelay) level = std::min(1.0f, level + p.fraction(p.rechargeRate * dt));
    const float target = firing ? 1.0f : 0.0f;
    if (amount < target) amount = std::min(target, amount + dt / std::max(p.rampUp, 1e-3f));
    else amount = std::max(target, amount - dt / std::max(p.rampDown, 1e-3f));
    emptyFlash = std::max(0.0f, emptyFlash - dt);
    bonusGlow = std::max(0.0f, bonusGlow - dt);
}

float Tank::add(float seconds, int kind, const Params& p) {
    if (!(seconds > 0)) return 0;
    const float before = level;
    level = std::min(1.0f, level + p.fraction(seconds));
    const float got = (level - before) * p.tankSeconds;
    if (kind > KindNone && kind < kKinds) earned[size_t(kind)] += seconds, ++events[size_t(kind)];
    lastKind = kind, lastBonus = seconds;
    bonusGlow = 0.8f;
    return got;
}

// ---- the boost's physics ---------------------------------------------------------------------------------------------

float torqueMultiplier(float amount, const Params& p) { return 1.0f + amount * p.torqueGain * p.power; }

float wheelDragBlend(float amount, const Params& p) { return clamp01(amount * p.dragRelease); }

float rpmStretch(float amount, const Params& p) { return 1.0f + amount * p.topSpeedLift * p.power; }

float thrustFade(float speed, float topSpeed, const Params& p) {
    if (!(topSpeed > 0)) return 1.0f;
    const float vmax = topSpeed * (1.0f + p.topSpeedLift * p.power);
    return clamp01((vmax - speed) / (0.15f * vmax));
}

float thrustAccel(float amount, float speed, float topSpeed, float loaded, const Params& p) {
    return amount * p.thrust * p.power * thrustFade(speed, topSpeed, p) * clamp01(loaded);
}

float topSpeedMs(float p619) { return p619 * 0.86956525f * 0.2777778f; }

// ---- detectors -------------------------------------------------------------------------------------------------------

float slipAngleDeg(const std::array<float, 3>& v, const std::array<float, 3>& f, const std::array<float, 3>& u) {
    auto flat = [&u](const std::array<float, 3>& a) {
        const float d = a[0] * u[0] + a[1] * u[1] + a[2] * u[2];
        return std::array<float, 3>{a[0] - u[0] * d, a[1] - u[1] * d, a[2] - u[2] * d};
    };
    const auto a = flat(v), b = flat(f);
    const float la = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]), lb = std::sqrt(b[0] * b[0] + b[1] * b[1] + b[2] * b[2]);
    if (la < 1e-4f || lb < 1e-4f) return 0;
    const float c = std::clamp((a[0] * b[0] + a[1] * b[1] + a[2] * b[2]) / (la * lb), -1.0f, 1.0f);
    return std::acos(c) * 57.29578f;
}

float airBonus(float flight, const Params& p) {
    if (flight < p.airMin) return 0;
    return std::min(p.airMax, p.airBase + (flight - p.airMin) * p.airRate);
}

float Detector::air(float airtime, bool upright, bool wrecked, bool respawned, const Params& p) {
    float got = 0;
    if (respawned || wrecked) {
        flight = 0;
        return 0;
    }
    if (airtime > 0) {
        flight = airtime;
    } else if (flight > 0) {  // landed this step
        if (upright) got = airBonus(flight, p);
        flight = 0;
    }
    return got;
}

float Detector::powerslide(float dt, float slipDeg, float speed, bool grounded, bool wrecked, const Params& p) {
    const bool on = grounded && !wrecked && speed >= p.slideMinSpeed && slipDeg >= p.slideMinDeg && slipDeg <= p.slideMaxDeg;
    if (!on) {
        slide = 0;
        return 0;
    }
    const float before = slide;
    slide += dt;
    if (slide <= p.slideHold) return 0;
    return p.slideRate * (slide - std::max(before, p.slideHold));
}

float NearMissPair::step(float dt, float lat, float lon, float halfW, float halfL, float closing, const Params& p) {
    cooldown = std::max(0.0f, cooldown - dt);
    const float clear = std::fabs(lat) - halfW;
    const bool near = std::fabs(lon) < halfL + 4.0f && clear < p.nearMissGap + 1.5f;
    if (!near) {
        close = false;
        return 0;
    }
    if (!close) {  // a pass begins
        close = true, touched = false, side = lon >= 0 ? 1.0f : -1.0f, minClear = clear;
    }
    minClear = std::min(minClear, clear);
    if (clear < 0.05f && std::fabs(lon) < halfL) touched = true;  // overlapping boxes: the bodies met
    float got = 0;
    if ((lon >= 0 ? 1.0f : -1.0f) != side) {  // side by side: j went past (or i went past j)
        if (!touched && minClear <= p.nearMissGap && closing >= p.nearMissSpeed && cooldown <= 0) {
            got = p.nearMiss;
            cooldown = p.nearMissCooldown;
        }
        side = -side, touched = false, minClear = clear;
    }
    return got;
}

void Overtakes::step(float dt, const std::vector<int>& pos, const std::vector<bool>& racing, std::vector<float>& bonus, const Params& p) {
    const size_t n = pos.size();
    bonus.assign(n, 0.0f);
    if (confirmed_.size() != n) {  // the first call (or a new field): the places as they are, nothing earned
        confirmed_ = pending_ = pos;
        since_.assign(n, 0.0f);
        return;
    }
    for (size_t i = 0; i < n; ++i) {
        if (pos[i] != pending_[i]) pending_[i] = pos[i], since_[i] = 0;
        else since_[i] += dt;
        if (since_[i] >= p.overtakeHold && pending_[i] != confirmed_[i]) {
            if (pending_[i] < confirmed_[i] && i < racing.size() && racing[i]) bonus[i] = float(confirmed_[i] - pending_[i]) * p.overtake;
            confirmed_[i] = pending_[i];
        }
    }
}

}  // namespace fuel::nitro
