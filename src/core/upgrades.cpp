// [DEVIATION, the user's request 2026-11-06] Vehicle upgrades, the core (upgrades.hpp; notes upgrades-plan.md, upg1.md).
#include "core/upgrades.hpp"

#include "core/custom_vehicles.hpp"
#include "physics/vehicle_physics.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <sstream>

namespace fuel::upg {

namespace {

bool gEnabled = true;
Stages gPlayer;
bool gPlayerFromEnv = false;

std::string lower(std::string s) {
    for (char& c : s) c = char(std::tolower(static_cast<unsigned char>(c)));
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back()))) s.pop_back();
    size_t a = 0;
    while (a < s.size() && std::isspace(static_cast<unsigned char>(s[a]))) ++a;
    return s.substr(a);
}
int clampStage(int s) { return std::clamp(s, 0, kMaxStage); }
float table(const float (&t)[4], int stage) { return t[clampStage(stage)]; }

// FUEL_UPGRADES (read once): stages for the player, or "off".
struct EnvInit {
    EnvInit() {
        const char* e = std::getenv("FUEL_UPGRADES");
        if (!e || !*e) return;
        const std::string v = lower(e);
        if (v == "off" || v == "0,off") {
            gEnabled = false;
            return;
        }
        Stages s;
        std::string err;
        if (parseStages(v, s, &err))
            gPlayer = s, gPlayerFromEnv = true;
        else
            std::fprintf(stderr, "upgrades: FUEL_UPGRADES '%s' ignored: %s\n", e, err.c_str());
    }
};
const EnvInit& envInit() {
    static const EnvInit init;
    return init;
}

}  // namespace

// categoryName / categoryKey / Stages::stock / parseStages / formatStages: core/upgrade_rules.cpp (fuelcore: the career
// uses them) [upgrades U2].

// ---- classes

int classOf(const std::string& variant) {
    static const char* const kPrefix[] = {"BUGGY", "BIKE", "MUSCLE", "QUAD", "SUV", "TRUCK"};
    for (int k = 0; k < 6; ++k)
        if (variant.rfind(std::string(kPrefix[k]) + "_", 0) == 0) return k;
    // FUEL's gifts (VehiclesDef.tsc AddVehicleInfo: BONUS_ROAD_3 DRAGSTER, BONUS_OFF_1 HYDRO, BONUS_ALL_2 TRUCK).
    if (variant == "BONUS_ROAD_3") return Dragster;
    if (variant == "BONUS_OFF_1") return Hydro;
    if (variant == "BONUS_ALL_2") return Truck;
    if (const custom::Vehicle* v = custom::find(variant)) {  // a custom pack: its AddVehicleInfo class
        static const char* const kFamily[] = {"BUGGY", "BIKE", "MUSCLE", "QUAD", "SUV", "TRUCK", "HYDRO", "DRAGSTER"};
        for (int k = 0; k < kClasses; ++k)
            if (lower(v->family) == lower(kFamily[k])) return k;
    }
    return -1;  // traffic (FLOTTE_*, TRAFFIC_TRUCK), VEH_TEST_*: never upgraded
}

const char* className(int cls) {
    static const char* const k[] = {"BUGGY", "BIKE", "MUSCLE", "QUAD", "SUV", "TRUCK", "HYDRO", "DRAGSTER"};
    return cls >= 0 && cls < kClasses ? k[cls] : "NONE";
}

const ClassFactors& factors(int cls) {
    // Plan 2.3 (v2): engine, weight inertia, tyres lateral, anti-roll, nitro power, lift S1/S2/S3.
    static const ClassFactors kTable[kClasses] = {
        {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, {0.03f, 0.05f, 0.08f}, false},      // BUGGY
        {0.75f, 0.5f, 1.0f, 0.0f, 0.75f, {0.01f, 0.02f, 0.03f}, true, 0.0f},  // BIKE: fork / shock; tyres lateral only
        {1.0f, 1.0f, 1.0f, 1.0f, 1.0f, {0.015f, 0.03f, 0.045f}, false},    // MUSCLE
        {1.0f, 0.5f, 1.0f, 1.0f, 1.0f, {0.02f, 0.04f, 0.06f}, false},      // QUAD
        {1.0f, 1.0f, 0.5f, 1.0f, 1.0f, {0.04f, 0.08f, 0.12f}, false},      // SUV
        {1.0f, 1.0f, 0.5f, 1.0f, 1.0f, {0.04f, 0.08f, 0.12f}, false},      // TRUCK
        {1.0f, 0.5f, 1.0f, 1.0f, 1.0f, {0.02f, 0.04f, 0.06f}, false},      // HYDRO: as QUAD
        {0.5f, 1.0f, 1.0f, 1.0f, 0.5f, {0.015f, 0.03f, 0.045f}, false},    // DRAGSTER: lift as MUSCLE
    };
    static const ClassFactors kNone{0, 0, 0, 0, 0, {0, 0, 0}, false, 0};
    return cls >= 0 && cls < kClasses ? kTable[cls] : kNone;
}

// ---- stage tables

float engineGain(int s) { static const float t[4] = {1.0f, 1.25f, 1.5f, 1.8f}; return table(t, s); }
float dampScale(int s) {  // [CHANGED: user, suspension = travel] FUEL's damping (FUEL_UPG_DAMP=a,b,c: bench override)
    static const std::array<float, 4> t = [] {
        std::array<float, 4> d{1.0f, 1.0f, 1.0f, 1.0f};
        if (const char* e = std::getenv("FUEL_UPG_DAMP")) std::sscanf(e, "%f,%f,%f", &d[1], &d[2], &d[3]);
        return d;
    }();
    return t[size_t(clampStage(s))];
}
float travelScale(int s) { static const float t[4] = {1.0f, 1.33f, 1.67f, 2.0f}; return table(t, s); }
float antiRollScale(int s) { static const float t[4] = {1.0f, 1.17f, 1.33f, 1.5f}; return table(t, s); }
float bumpScale(int s) { static const float t[4] = {1.0f, 1.10f, 1.20f, 1.30f}; return table(t, s); }
float tyreScale(int s) { static const float t[4] = {1.0f, 1.04f, 1.08f, 1.12f}; return table(t, s); }
float effectiveMass(int s) { static const float t[4] = {1.0f, 0.96f, 0.92f, 0.88f}; return table(t, s); }
float nitroPowerScale(int s) { static const float t[4] = {1.0f, 1.15f, 1.30f, 1.45f}; return table(t, s); }
float nitroTankScale(int s) { static const float t[4] = {1.0f, 1.2f, 1.4f, 1.6f}; return table(t, s); }
float nitroRechargeScale(int s) { static const float t[4] = {1.0f, 1.1f, 1.2f, 1.3f}; return table(t, s); }

float dampCompliance(float c, float rate) {
    if (c >= 1.0f || c <= 0.0f || rate == 1.0f) return c;
    return 1.0f / (rate * (1.0f / c - 1.0f) + 1.0f);
}

float torqueScale(const Stages& st, int cls, float power, bool* clamped) {
    if (clamped) *clamped = false;
    if (cls < 0) return 1.0f;
    const ClassFactors& k = factors(cls);
    const float e = 1.0f + k.engine * (engineGain(st[Engine]) - 1.0f);
    const float m = effectiveMass(st[Weight]);
    float s = e / m;
    if (s == 1.0f) return 1.0f;
    const float cap = std::min(kStageTorqueCap, power > 0 ? kTotalTorqueCap / power : kStageTorqueCap);
    if (s > cap) {
        s = std::max(1.0f, cap);
        if (clamped) *clamped = true;
    }
    return s;
}

float widening(float halfTrack, float comHeight, float lift, float cap) {
    if (!(lift > 0) || !(halfTrack > 0)) return 0.0f;
    return std::min(halfTrack * lift / std::max(comHeight, 0.05f), cap < 0 ? lift : cap);
}

void transformParams(std::vector<float>& P, const Stages& st, int cls) {
    if (cls < 0 || st.stock()) return;
    if (P.size() < 160) return;  // not a VehiclePP block
    const ClassFactors& k = factors(cls);
    auto gain = [](float g, float kk) { return 1.0f + kk * (g - 1.0f); };
    // Suspension: damping rates (compression P79/80, rebound P81/82, quadratic P83-86), anti-roll P87/88, bump capacity
    // P89/90, travel P71/72 + the lift.
    if (const int s = st[Suspension]; s > 0) {
        for (int n = 79; n <= 86; ++n) P[size_t(n)] = dampCompliance(P[size_t(n)], dampScale(s));
        if (!k.bike)
            for (int n : {87, 88}) P[size_t(n)] *= gain(antiRollScale(s), k.antiRoll);
        for (int n : {89, 90}) P[size_t(n)] *= bumpScale(s);
        for (int n : {71, 72}) P[size_t(n)] *= travelScale(s);  // [CHANGED: user] the travel (max depth P71/72), +33 / 67 / 100 %
    }
    // Tyres: peak (set 1 long. P108/109 + loose P132/133, set 2 lat. P110/111 + P134/135) and sliding grip (P116-119,
    // P140-143); SUV / TRUCK half the lateral gain.
    if (const int s = st[Tires]; s > 0) {
        const float g = tyreScale(s);
        for (int base : {108, 116, 132, 140}) {
            for (int n : {base, base + 1}) P[size_t(n)] *= gain(g, k.tyreLong);     // longitudinal (front, rear)
            for (int n : {base + 2, base + 3}) P[size_t(n)] *= gain(g, k.tyreLat);  // lateral
        }
    }
    // Weight: force inertia P4-6 x m (half on light classes); the drive's 1/m is in the torque scale. [U1, against plan
    // 2.2.4] the aero drag P93/94 stays FUEL's: x 1/m cost rev-limited vehicles their top speed (SUV_ROAD_3 -5 km/h,
    // BIKE_ROAD_3, TRUCK_ALL_3, TRUCK_OFF_1 -2..-3 at W3; notes upg1.md 3), and the stages must never be slower.
    if (const int s = st[Weight]; s > 0) {
        const float m = effectiveMass(s);
        const float im = 1.0f - (1.0f - m) * k.inertia;
        for (int n : {4, 5, 6}) P[size_t(n)] *= im;
    }
}

Result apply(VehicleSetup& setup, const Stages& stIn, float power) {
    Result r;
    r.cls = classOf(setup.name);
    Stages st;
    for (int c = 0; c < kCategories; ++c) st[c] = clampStage(stIn[c]);
    r.stages = st;
    if (!enabled() || st.stock() || r.cls < 0) return r;
    const ClassFactors& k = factors(r.cls);
    // Geometry before the rewrite: CoM height over the tyres' bottom (model space, suspension unloaded), half tracks.
    float bottom = 1e9f, htF = 0, htR = 0;
    for (const auto& w : setup.wheels) {
        bottom = std::min(bottom, w.hub[1] - w.radius);
        (w.front ? htF : htR) = std::max(w.front ? htF : htR, std::fabs(w.hub[0]));
    }
    const float comY = setup.params.size() > 11 ? setup.params[11] : 0.0f;
    r.comHeight = setup.wheels.empty() ? 0.0f : comY - bottom;
    transformParams(setup.params, st, r.cls);
    r.torqueScale = torqueScale(st, r.cls, power, &r.clamped);
    setup.torqueScale = r.torqueScale;
    if (const int s = st[Suspension]; s > 0) {
        r.lift = k.lift[size_t(s - 1)];
        if (!k.bike && !setup.bike) {
            // [CHANGED U1] tall classes (SUV, TRUCK: the Jeeps rolled over at S2 / S3 in step steer with the capped
            // widening) take the full rollover formula half-track x lift / CoM height (at most 0.3 m); the rest stay
            // capped at the lift (QUAD_ROAD_2's CoM 0.1 m would ask +27 cm a side).
            const float cap = (r.cls == Suv || r.cls == Truck) ? 0.3f : r.lift;
            r.widenFront = widening(htF, r.comHeight, r.lift, cap);
            r.widenRear = widening(htR, r.comHeight, r.lift, cap);
        }
        setup.lift = r.lift, setup.widenFront = r.widenFront, setup.widenRear = r.widenRear;
        for (auto& w : setup.wheels) {  // as FuelVehicle::hubRest moves the physics wheels
            const float wide = w.front ? r.widenFront : r.widenRear;
            w.hub[0] += w.hub[0] >= 0 ? wide : -wide;
            w.hub[1] -= r.lift;
        }
    }
    r.nitroPower = 1.0f + k.nitroPower * (nitroPowerScale(st[NitroPower]) - 1.0f);
    r.nitroTank = nitroTankScale(st[NitroTank]);
    r.nitroRecharge = nitroRechargeScale(st[NitroTank]);
    setup.nitroPower = r.nitroPower, setup.nitroTank = r.nitroTank, setup.nitroRecharge = r.nitroRecharge;
    setup.upgraded = true;
    setup.upgStages = st.s;  // [upgrades U5] for the visible upgrades
    r.applied = true;
    return r;
}

std::string describe(const Result& r) {
    char b[400];
    if (!r.applied) {
        std::snprintf(b, sizeof b, "stock (%s)", !enabled() ? "upgrades off" : r.cls < 0 ? "never upgraded" : "all stages 0");
        return b;
    }
    std::snprintf(b, sizeof b, "%s stages e%d s%d t%d w%d np%d nt%d: torque x%.3f%s, lift %.3f m, widen %.3f / %.3f m (CoM %.3f m), nitro power x%.3f "
                  "tank x%.2f recharge x%.2f",
                  className(r.cls), r.stages[Engine], r.stages[Suspension], r.stages[Tires], r.stages[Weight], r.stages[NitroPower], r.stages[NitroTank],
                  r.torqueScale, r.clamped ? " (capped)" : "", r.lift, r.widenFront, r.widenRear, r.comHeight, r.nitroPower, r.nitroTank, r.nitroRecharge);
    return b;
}

// ---- settings and sessions

bool enabled() {
    envInit();
    return gEnabled;
}
void setEnabled(bool on) {
    envInit();
    const char* e = std::getenv("FUEL_UPGRADES");
    if (e && lower(e) == "off") on = false;  // FUEL_UPGRADES=off always wins
    gEnabled = on;
}
bool enabledFrom(const char* v, bool def) {
    if (!v) return def;
    const std::string t = lower(v);
    if (t == "on" || t == "1" || t == "true" || t == "yes") return true;
    if (t == "off" || t == "0" || t == "false" || t == "no") return false;
    return def;
}

const Stages& playerStages() {
    envInit();
    return gPlayer;
}
void setPlayerStages(const Stages& s) {
    envInit();
    if (gPlayerFromEnv) {
        std::fprintf(stderr, "upgrades: FUEL_UPGRADES overrides --upgrades %s\n", formatStages(s).c_str());
        return;
    }
    gPlayer = s;
}

Result applyPlayer(VehicleSetup& setup, float power) {
    const Result r = apply(setup, playerStages(), power);
    if (r.applied || !playerStages().stock())
        std::fprintf(stderr, "upgrades: %s: %s\n", setup.name.c_str(), describe(r).c_str());
    return r;
}

// ---- opponents (U4)

namespace {
int gAiTier = kAiTierAuto;
}  // namespace

int aiBaseStage(int hubTier) {
    if (hubTier < 0) return 0;
    return hubTier <= 2 ? 0 : hubTier <= 4 ? 1 : hubTier <= 6 ? 2 : 3;
}

uint64_t fnv1a64(const std::string& s) {
    uint64_t h = 0xcbf29ce484222325ull;
    for (unsigned char c : s) h = (h ^ c) * 0x100000001b3ull;
    return h;
}

uint64_t splitmix64(uint64_t& state) {
    uint64_t z = (state += 0x9E3779B97F4A7C15ull);
    z = (z ^ (z >> 30)) * 0xBF58476D1CE4E5B9ull;
    z = (z ^ (z >> 27)) * 0x94D049BB133111EBull;
    return z ^ (z >> 31);
}

AiDraw aiDraw(const std::string& race, int slot, int hubTier, uint64_t seedOverride) {
    AiDraw d;
    d.hubTier = hubTier < 0 ? kAiTierNone : hubTier;
    d.base = aiBaseStage(hubTier);
    d.seed = fnv1a64(race) ^ (uint64_t(slot + 1) * 0x9E3779B97F4A7C15ull) ^ seedOverride;
    uint64_t st = d.seed;
    auto uniform = [&st] { return double(splitmix64(st) >> 11) * (1.0 / 9007199254740992.0); };  // [0, 1), 53 bits
    if (hubTier < 0) return d;  // no hub: stock (nothing drawn)
    // 1. the rival roll (drawn at every base; at base 0, a stock-tier hub, no rival of either kind)
    static const double kMax[4] = {0.0, 0.03, 0.06, 0.10};
    const double u = uniform();
    if (d.base >= 1 && u < 0.05) d.rival = -1;
    else if (d.base >= 1 && u >= 1.0 - kMax[d.base]) d.rival = +1;
    // 2. each category, base -1 / 0 / +1 at 25 / 50 / 25 %, clamped
    for (int c = 0; c < kCategories; ++c) {
        const double v = uniform();
        const int dlt = v < 0.25 ? -1 : v < 0.75 ? 0 : 1;
        d.stages[c] = clampStage(d.base + dlt);
    }
    if (d.rival < 0) d.stages = Stages{};
    if (d.rival > 0) d.stages.s.fill(kMaxStage);
    return d;
}

int aiHubTier() {
    static const int env = [] {
        const char* e = std::getenv("FUEL_AI_UPGRADE_TIER");
        int t = kAiTierAuto;
        if (e && *e && !parseAiTier(e, t)) std::fprintf(stderr, "upgrades: FUEL_AI_UPGRADE_TIER '%s' ignored (0..9, -1 / none)\n", e), t = kAiTierAuto;
        return t;
    }();
    return env != kAiTierAuto ? env : gAiTier;
}
void setAiHubTier(int tier) { gAiTier = tier; }
bool parseAiTier(const std::string& s, int& tier) {
    const std::string v = lower(s);
    if (v == "none" || v == "-1") {
        tier = kAiTierNone;
        return true;
    }
    if (v == "auto") {
        tier = kAiTierAuto;
        return true;
    }
    char* end = nullptr;
    const long n = std::strtol(v.c_str(), &end, 10);
    if (v.empty() || *end != 0 || n < 0 || n > 9) return false;
    tier = int(n);
    return true;
}
uint64_t aiSeedOverride() {
    static const uint64_t seed = [] {
        const char* e = std::getenv("FUEL_AI_UPGRADE_SEED");
        return e && *e ? uint64_t(std::strtoull(e, nullptr, 0)) : uint64_t(0);
    }();
    return seed;
}

namespace {
// U4 calibration (notes upg4.md 3): per class the drive exponent (0-100 time ~ torque scale ^ -alpha: U1's flat bench,
// the E3 rows of plan 2.4) and the E3 top-speed gain.
struct AiClassTune {
    float alpha = 0.4f;   // 0-100: QUAD_ROAD_2 E3 3.32 -> 2.45 s at x1.8 = 0.52; MUSCLE_ROAD_6 0.11; TRUCK 0.30-0.42
    float topE3 = 0.03f;  // top speed gain at E3 (TRUCK_ALL_4 +21.7 %, SUV_ALL_2 +7.4 %, QUAD +2.3 %, BIKE 0)
};
const AiClassTune& aiTune(int cls) {
    static const AiClassTune kTable[kClasses] = {
        {0.30f, 0.035f},  // BUGGY: OFF_3 0.34 / +3.8 %, ROAD_1 0.14 / +3.3 %
        {0.55f, 0.0f},    // BIKE: OFF_3 0.55 (k 0.75: x1.6), top speed rev-limited
        {0.11f, 0.035f},  // MUSCLE: ROAD_6 0.11 / +3.5 %
        {0.52f, 0.023f},  // QUAD: ROAD_2 0.52 / +2.3 %
        {0.52f, 0.074f},  // SUV: ALL_2 0.52 / +7.4 %
        {0.36f, 0.20f},   // TRUCK: ALL_4 0.30 / +21.7 %, OFF_1 0.42 / +19.9 %
        {0.52f, 0.023f},  // HYDRO: as QUAD
        {0.11f, 0.0f},    // DRAGSTER: as MUSCLE, k 0.5
    };
    static const AiClassTune kNone{0, 0};
    return cls >= 0 && cls < kClasses ? kTable[cls] : kNone;
}
// Lap-time share of each category's stage (the fraction of the target time it takes off), U4's calibration on the lap
// bot (--ai-player bot, notes upg4.md 3): all categories S1 / S2 / S3 took 0.3 / 2.1 / 4.2 % off QUAD_ROAD_2's CR2-Yell-CP
// (off-road: the traction governor holds most of the extra torque on dirt) and 2.0 % off MUSCLE_ROAD_6's CR64-Val4-CP at
// S3; the plan's start values (S3 15 %) left a stage-matched player 10 % behind the field. Engine and weight scale with the
// class's drive exponent (alpha / 0.45, 0.4..1.2: torque-limited classes gain more), tyres and suspension do not.
float aiLapGain(int category, int stage, float driveScale) {
    static const float kEngine[4] = {0.0f, 0.0075f, 0.015f, 0.020f};
    static const float kWeight[4] = {0.0f, 0.003f, 0.006f, 0.009f};
    static const float kTires[4] = {0.0f, 0.0025f, 0.005f, 0.0075f};
    static const float kSusp[4] = {0.0f, 0.0025f, 0.005f, 0.0075f};
    const int s = clampStage(stage);
    switch (category) {
    case Engine: return kEngine[s] * driveScale;
    case Weight: return kWeight[s] * driveScale;
    case Tires: return kTires[s];
    case Suspension: return kSusp[s];
    default: return 0.0f;  // nitro stages act through the racer's own tank and boost (ai_nitro)
    }
}
}  // namespace

AiFactors aiFactors(const Stages& stIn, int cls, float power) {
    AiFactors f;
    Stages st;
    for (int c = 0; c < kCategories; ++c) st[c] = clampStage(stIn[c]);
    if (!enabled() || st.stock() || cls < 0) return f;
    const ClassFactors& k = factors(cls);
    const AiClassTune& tune = aiTune(cls);
    const float driveScale = std::clamp(tune.alpha / 0.45f, 0.4f, 1.2f);
    float gain = 0;
    for (int c : {int(Engine), int(Weight), int(Tires), int(Suspension)}) gain += aiLapGain(c, st[c], driveScale);
    f.time = 1.0f - gain;
    f.corner = std::sqrt(1.0f + k.tyreLat * (tyreScale(st[Tires]) - 1.0f));
    f.accel = std::pow(torqueScale(st, cls, power), tune.alpha);
    f.top = 1.0f + tune.topE3 * k.engine * (engineGain(st[Engine]) - 1.0f) / (engineGain(kMaxStage) - 1.0f);
    return f;
}

std::string describeAi(const AiDraw& d, const Result& r, const AiFactors& f) {
    char b[520];
    const char* rival = d.rival < 0 ? " (rival: all stock)" : d.rival > 0 ? " (rival: all max)" : "";
    std::snprintf(b, sizeof b, "hub tier %d base %d stages %s%s seed %016llx: %s; ghost time x%.3f corners x%.3f 0-100 /%.3f top x%.3f",
                  d.hubTier, d.base, formatStages(d.stages).c_str(), rival, (unsigned long long)d.seed, describe(r).c_str(), f.time, f.corner, f.accel,
                  f.top);
    return b;
}

// ---- prices and gates: core/upgrade_rules.cpp [upgrades U2]

}  // namespace fuel::upg
