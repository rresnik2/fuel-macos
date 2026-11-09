// [DEVIATION, the user's request 2026-11-06] Vehicle upgrades (U1, the core). FUEL (2009) has no upgrades: everything here
// is the port's. Plan: ~/re-work/fuel/notes/upgrades-plan.md (v2); this package: notes/upg1.md.
//
// Six categories, stock + 3 stages each, per vehicle. A vehicle's stages rewrite a copy of its VehiclePP values and set
// three port inputs to FUEL's model that are identities at stock (VehicleSetup::torqueScale, lift / widenFront /
// widenRear), plus the vehicle's nitrous multipliers, before DriveWorld builds fv::FuelVehicle. All stages 0, or
// `upgrades = off`, leave the setup untouched: FUEL's vehicle bit for bit.
//
//   Engine      governed torque scale (FuelVehicle::setTorqueScale, the traction governor) x1.25 / x1.5 / x1.8
//   Suspension  damping rates (P79-86) x1.33 / x1.67 / x2.0, anti-roll (P87/88) x1.17 / x1.33 / x1.5, bump capacity
//               (P89/90) x1.1 / x1.2 / x1.3, a physical lift (every wheel centre lower: FuelVehicle::setHubShift) with
//               the travel (P71/72) + lift, and a track widening per side min(half-track x lift / CoM height, lift)
//   Tires       peak and sliding grip, long. and lateral, hard and loose ground (P108-111, 116-119, 132-135, 140-143)
//               x1.04 / x1.08 / x1.12
//   Weight      effective mass m 0.96 / 0.92 / 0.88: drive x 1/m (in the torque scale), force inertia P4-6 x m; aero drag
//               (P93/94, kept: x 1/m slowed rev-limited vehicles), mass, crash mass and impulse inertia (P2, P3, P7-9) stay FUEL's
//   Nitro power the boost's gains (nitro::Params::power) x1.15 / x1.30 / x1.45
//   Nitro tank  tank seconds x1.2 / 1.4 / 1.6 (3.6 / 4.2 / 4.8 s), idle recharge x1.1 / 1.2 / 1.3
// A class factor k scales each gain (value x (1 + k x gain)): plan section 2.3. The stage product engine x 1/m is capped
// at 2.0 and engine_power x stages at 2.6 (engine_power keeps its own 0.5..2 clamp).
#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace fuel {
struct VehicleSetup;
}

namespace fuel::upg {

// Category order of the session argument `--upgrades e,s,t,w,np,nt` and of FUEL_UPGRADES (fixed, append-only).
enum Category : int { Engine = 0, Suspension, Tires, Weight, NitroPower, NitroTank, kCategories };
constexpr int kMaxStage = 3;
const char* categoryName(int c);  // "engine", "suspension", "tires", "weight", "nitro power", "nitro tank"
const char* categoryKey(int c);   // "e", "s", "t", "w", "np", "nt"
int categoryFromKey(const std::string& name);  // a key or a name ("np", "nitro power", "tyres"); -1 unknown [U2]

struct Stages {
    std::array<int, kCategories> s{};  // 0 = stock .. 3
    int& operator[](int c) { return s[size_t(c)]; }
    int operator[](int c) const { return s[size_t(c)]; }
    bool stock() const;
    bool operator==(const Stages& o) const { return s == o.s; }
};
// "e,s,t,w,np,nt" (each 0..3; fewer values = the rest 0) or "max" (all 3) / "stock" (all 0). False on a bad value.
bool parseStages(const std::string& spec, Stages& out, std::string* err = nullptr);
std::string formatStages(const Stages& s);  // "3,3,3,3,3,3"

// FUEL's vehicle class (VehicleInfo +0x38, 0x56a580): BUGGY 0, BIKE 1, MUSCLE 2, QUAD 3, SUV 4, TRUCK 5, HYDRO 6,
// DRAGSTER 7; -1 = never upgraded (traffic, the test vehicles, UNKNOWN). From the variant's prefix (VehiclesDef.tsc: every
// BIKE_* is BIKE, ...), FUEL's three gifts (BONUS_ROAD_3 DRAGSTER, BONUS_OFF_1 HYDRO, BONUS_ALL_2 TRUCK) and a custom
// vehicle pack's AddVehicleInfo class (core/custom_vehicles).
enum Class : int { Buggy = 0, Bike, Muscle, Quad, Suv, Truck, Hydro, Dragster, kClasses };
int classOf(const std::string& variant);
const char* className(int cls);

// The class factors (plan 2.3): k on each gain (1 = the stage tables).
struct ClassFactors {
    float engine = 1;      // the engine stage's gain above 1
    float inertia = 1;     // the weight stage's inertia change
    float tyreLat = 1;     // the tyre stage's lateral rows
    float antiRoll = 1;    // the suspension stage's anti-roll (0: none, bikes)
    float nitroPower = 1;  // the nitrous power stage's gain
    std::array<float, 3> lift{};  // metres at S1 / S2 / S3
    bool bike = false;     // no track widening, no anti-roll
    // [CHANGED U1] the tyre stage's longitudinal rows (bikes 0: FUEL's bike launch is bistable on the rear wheel's grip:
    // +12 % peak grip alone took BIKE_ROAD_4's 5 s distance 103.3 -> 85.1 m; the lateral rows are the cornering grip)
    float tyreLong = 1;
};
const ClassFactors& factors(int cls);

// ---- the stage tables (index = stage 0..3; stage 0 = 1 / 0: the identity)
float engineGain(int stage);   // 1, 1.25, 1.5, 1.8
float dampScale(int stage);    // rates: FUEL's (1) at every stage [CHANGED: user, suspension = travel; was 1.33 / 1.67 / 2.0]
float travelScale(int stage);  // suspension travel P71/72: 1, 1.33, 1.67, 2.0 [CHANGED: user, suspension = travel]
float antiRollScale(int stage);  // 1, 1.17, 1.33, 1.5
float bumpScale(int stage);    // 1, 1.10, 1.20, 1.30
float tyreScale(int stage);    // 1, 1.04, 1.08, 1.12
float effectiveMass(int stage);  // 1, 0.96, 0.92, 0.88
float nitroPowerScale(int stage);  // 1, 1.15, 1.30, 1.45
float nitroTankScale(int stage);   // 1, 1.2, 1.4, 1.6
float nitroRechargeScale(int stage);  // 1, 1.1, 1.2, 1.3
constexpr float kStageTorqueCap = 2.0f;  // engine x 1/m
constexpr float kTotalTorqueCap = 2.6f;  // engine_power x engine x 1/m

// A damping compliance c (P79-86: rate = 1/c - 1) at `rate` x the rate: c' = 1 / (rate (1/c - 1) + 1); c >= 1 or <= 0
// stays (no damping / not a compliance).
float dampCompliance(float c, float rate);
// The governed torque scale of the engine and weight stages for class `cls` at engine_power `power`: (1 + k (E - 1)) / m,
// capped at kStageTorqueCap and at kTotalTorqueCap / power; `clamped` set when a cap cut it.
float torqueScale(const Stages& st, int cls, float power, bool* clamped = nullptr);
// Track widening per side for a lift: min(halfTrack x lift / comHeight, lift) (plan 2.2.1, capped at the lift).
float widening(float halfTrack, float comHeight, float lift, float cap = -1);  // cap < 0: the lift; SUV / TRUCK: 0.3 m [CHANGED U1]

// The VehiclePP rewrite alone (no geometry, no torque scale, no nitro): suspension rates / anti-roll / bump / travel,
// tyres, weight drag and inertia. `cls` < 0 or stock stages: unchanged.
void transformParams(std::vector<float>& P, const Stages& st, int cls);

// What apply() did (logs, benches, the garage's numbers later).
struct Result {
    bool applied = false;  // false: stock stages, upgrades off, or a class that is never upgraded
    int cls = -1;
    Stages stages;
    float torqueScale = 1, lift = 0, widenFront = 0, widenRear = 0, comHeight = 0;
    float nitroPower = 1, nitroTank = 1, nitroRecharge = 1;
    bool clamped = false;  // a torque cap applied
};
// Rewrites `setup` for `st` (the class from setup.name): params (transformParams), torqueScale, lift / widening (also the
// setup's wheel hubs, for the props' kinematic body and the widths read from them), the nitro multipliers. `power` =
// engine_power (FuelVehicle::powerScale) for the 2.6 total cap. Nothing changes when upgrades are off (enabled()), the
// stages are stock or the class is -1.
Result apply(VehicleSetup& setup, const Stages& st, float power);
std::string describe(const Result& r);  // one line for the logs

// ---- settings and sessions
// settings.txt `upgrades = on|off` (default on; game_settings.mm). Off = every vehicle stock (stages are kept).
bool enabled();
void setEnabled(bool on);
bool enabledFrom(const char* settingsValue, bool def = true);  // on/1/true/yes, off/0/false/no, else def
// The stages of the vehicle the player drives in this process: the session's `--upgrades e,s,t,w,np,nt` (U2: the menu
// passes the career's equipped stages), overridden by FUEL_UPGRADES (testing; "off" turns upgrades off). Default stock.
const Stages& playerStages();
void setPlayerStages(const Stages& s);
// The player's vehicle: apply(setup, playerStages(), power) with a log line; returns the result.
Result applyPlayer(VehicleSetup& setup, float power);

// ---- opponents (U4: plan 5.2 / 5.3, notes upg4.md) [DEVIATION]
// The base stage of a race's hub tier (CareerRules::Hub::tier, SetHubTargetInfos' 2nd argument): tiers 0-2 stock, 3-4
// stage 1, 5-6 stage 2, 7-9 stage 3; no hub (tier < 0): stock.
int aiBaseStage(int hubTier);
// FNV-1a 64 of a string; SplitMix64 step (state += golden gamma, mixed output).
uint64_t fnv1a64(const std::string& s);
uint64_t splitmix64(uint64_t& state);
// One opponent's draw: per racer (grid slot = 0-based index among the opponents), seeded with
// fnv1a64(race) ^ (slot + 1) x 0x9E3779B97F4A7C15 ^ seedOverride, in a fixed order: a rival roll u (base >= 1 only:
// u < 0.05 all stock, u >= 1 - p_max all 3 with p_max 0.03 / 0.06 / 0.10 for base 1 / 2 / 3), then each category in
// the enum order: base - 1 / base / base + 1 with 25 / 50 / 25 %, clamped to 0..3. Every value is drawn every time.
struct AiDraw {
    Stages stages;
    int hubTier = -1, base = 0;
    int rival = 0;  // -1 all stock, +1 all max, 0 none
    uint64_t seed = 0;
};
AiDraw aiDraw(const std::string& race, int slot, int hubTier, uint64_t seedOverride);
// Session: the race's hub tier for the opponents (`--ai-upgrade-tier N`, the menu passes the event's hub tier;
// FUEL_AI_UPGRADE_TIER=N wins: checks). kAiTierAuto = not given: the session uses the event's own hub (an edited race:
// the hub under its start); kAiTierNone (-1) = no hub: stock.
constexpr int kAiTierAuto = -100, kAiTierNone = -1;
int aiHubTier();
void setAiHubTier(int tier);
bool parseAiTier(const std::string& s, int& tier);  // "0".."9", "-1" / "none"; false otherwise
uint64_t aiSeedOverride();  // FUEL_AI_UPGRADE_SEED (decimal or 0x hex), default 0
// What an opponent's stages change in its ghost (FUEL's AI has no grip or torque input; plan 5.3, calibrated in U4):
//   time    the event's target time K x time (each stage's lap-time gain for the class, summed over engine / weight /
//           tires / suspension: t = 1 - sum)
//   corner  the corner table's caps (0x61dcf0) x sqrt(1 + lateral tyre gain)
//   accel   own 0-100 times (P623 / P624) / accel: the governed torque scale ^ the class's drive exponent
//   top     own top speeds (P619 / P620) x top: the engine stage's measured top-speed gain (only the ai-check bound reads it)
// The player's P619 / P620 / P623 / P624 (FUEL's rubber band 0x617000 reads them) are never touched. Identity when
// upgrades are off, the stages are stock or the class is -1.
struct AiFactors {
    float time = 1, corner = 1, accel = 1, top = 1;
    bool identity() const { return time == 1 && corner == 1 && accel == 1 && top == 1; }
};
AiFactors aiFactors(const Stages& st, int cls, float power);
std::string describeAi(const AiDraw& d, const Result& r, const AiFactors& f);  // one log line

// ---- prices and gates (plan 3.2 / 3.3; the career's Career::buyUpgrade): fuel for stage 1..3 of a category on a vehicle of
// `level` (1..6; AddVehicleInfo's level, gifts 6 = 5), rounded to 50; the stars a stage needs (stage 1: none).
// [U2] These, the category names and the stage list are defined in core/upgrade_rules.cpp (fuelcore).
int price(int level, int category, int stage);
int starsNeeded(int stage);

}  // namespace fuel::upg
