// [DEVIATION, the user's request 2026-11-06] Nitrous. FUEL (2009) has no nitrous: this is the port's own feature,
// notes ~/re-work/fuel/notes/nitro.md. `nitrous = off` in settings.txt (or FUEL_NITRO=off) turns every part of it off
// and the game runs exactly as before (no tank, no AI use, no HUD gauge, no effects).
//
// What FUEL left behind and the port reuses (decoded, never live in the shipped data):
//  - VehMove_BoostMultiplier 0x525190: the engine torque x phys+0x13c0 (FuelVehicle::boostMul_); FUEL fed it from
//    P330-P334 x the ground's BOOST BONUS, all 0 in VehiclesParam.tsc, gated by P633 = 0 (VehMove_BoostActive 0x524030);
//  - the brake/rolling block of 0x520fd0: the wheels' quadratic drag (1 - b) P91 + b P92 with b = the boost (P92 is 0 on
//    most vehicles: the boost was meant to lift the top speed); the port blends it by the nitro amount;
//  - the BOOST action (slot 16 of the engine's 45 actions, FUEL's pad A / key E from input.PC and DftMap.map): the
//    port's Nitro button (display name "Nitro");
//  - the exhaust flames' blue colour (0x5cf2d0 near top speed): the nitro flame.
// FUEL's own boost gauge (player +0x2f5c, 0x423270 / 0x423310: an overheat meter, P335-P337) is not used: the
// user asked for a tank.
//
// Everything tunable is in Params (one struct; the planned upgrade system scales `power`, `tankSeconds` and
// `rechargeRate`).
#pragma once

#include <array>
#include <cstddef>
#include <vector>

namespace fuel::nitro {

struct Params {
    // ---- the tank (Tank::level 0..1 = empty..full). The upgrade plan's two nitrous categories scale exactly these
    // fields: "nitrous tank" = tankSeconds and rechargeRate, "nitrous power" = power (notes upgrades-plan.md 2.2.5).
    float tankSeconds = 3.0f;       // a full tank = this many seconds of boost held
    float rechargeRate = 0.12f;     // idle recharge, seconds of boost per second (3 s / 0.12 = 25 s from empty to full)
    float rechargeDelay = 1.5f;     // s after a boost before the idle recharge starts
    float startLevel = 1.0f;        // the tank at a race / session start (fraction)
    float minStart = 0.18f;         // s of boost a new press needs; a tank run dry needs the button let go first
    // ---- the boost. `power` scales every gain above 1 (torque, push, top-speed lift, the AI ghost's gain): 1 = stock,
    // the "nitrous power" stages are 1.15 / 1.30 / 1.45.
    float power = 1.0f;
    // The boost is mostly a push on the chassis (the same on every class and ground: the bench in notes nitro.md) with a
    // little extra engine torque for the launch. More torque spun the driven wheels on loose ground and the boost LOST
    // speed there (BIKE_OFF_3 on dirt near its top, torque gain 0.35).
    float torqueGain = 0.10f;       // engine torque x (1 + torqueGain x power x amount): FUEL's +0x13c0 multiplier
    float thrust = 2.2f;            // m/s^2 forward push on the chassis at full amount, while a wheel is on the ground
    // The top speed x (1 + lift x power x amount): the rev range (torque curve and rpm clamp) stretches by it, the
    // wheels' quadratic drag (0x520fd0 P91) and the body's (0x530d20 P93 / P94) scale by 1 / it^2, the push fades out
    // over its last 15 % (of FUEL's top speed P619 x the stretch).
    float topSpeedLift = 0.15f;
    // FUEL's own boost drag path P91 -> P92 (0x520fd0), blended by this x amount. Off by default: P92 is 0 on most
    // vehicles, so it would take the wheels' drag away outright on some classes only (quads, buggies, muscle cars).
    float dragRelease = 0.0f;
    // The extra torque's traction limiter: while half the loaded driven wheels or more slide (0x524240), the torque gain
    // falls by this fraction (in 0.05 s, back over 0.3 s), so on loose ground the boost pushes instead of spinning.
    float tractionCut = 1.0f;
    float rampUp = 0.12f;           // s for the amount 0 -> 1 (no jolt on a bike)
    float rampDown = 0.25f;         // s for 1 -> 0
    // ---- bonus fills, in seconds of boost (the same seconds at every tank stage: skill pays the same)
    float airMin = 0.6f;            // a flight shorter than this earns nothing
    float airBase = 0.25f;          // landing a flight of airMin
    float airRate = 0.6f;           // per second of flight beyond airMin
    float airMax = 1.5f;            // at most per flight (half a stock tank)
    float slideMinDeg = 15.0f, slideMaxDeg = 70.0f;  // slip angle of a powerslide (velocity vs heading, ground plane)
    float slideMinSpeed = 11.0f;    // m/s (40 km/h)
    float slideHold = 0.4f;         // s of sliding before it counts
    float slideRate = 0.6f;         // per second of powerslide
    float overtake = 0.5f;          // per place gained in a race (held for overtakeHold)
    float overtakeHold = 1.0f;      // s a new place must hold
    float nearMiss = 0.4f;          // per near miss (passing an opponent or traffic vehicle side by side)
    float nearMissGap = 1.2f;       // m of side clearance at most (body edges; touching earns nothing)
    float nearMissSpeed = 6.0f;     // m/s of relative speed along the heading at least
    float nearMissCooldown = 2.0f;  // s before the same vehicle counts again
    // ---- the AI ghost (race_ai): its speed cap x (1 + aiSpeedGain x power x amount), its acceleration + thrust x power
    float aiSpeedGain = 0.15f;
    // ---- feel
    float fovKickDeg = 7.0f;        // added to the chase camera's FOV target at full amount (FUEL smooths the FOV, 1.05/s)
    float blur = 0.012f;            // added to FUEL's speed blur (viewport +0x1dc, P574 = 0.01..0.03), from 5 to 25 m/s
    float flame = 1.15f;            // the exhaust flame level at full amount (FUEL's flame: length 0.5 L^2, size 0.4 L^2)

    // Seconds of boost -> a fraction of this tank.
    float fraction(float seconds) const { return seconds / (tankSeconds > 0.05f ? tankSeconds : 0.05f); }
};

// The process's parameters (settings, upgrades) and the switch (settings.txt `nitrous`, default on).
Params& params();
// Development / tuning: "name=value[,name=value...]" (the field names above) into `p`; FUEL_NITRO_PARAMS is applied to
// params() at start-up. Returns the count of fields set (unknown names are skipped, reported on stderr).
int applyOverrides(Params& p, const char* spec);
bool enabled();
void setEnabled(bool on);
// settings value -> on/off: on/1/true/yes, off/0/false/no, anything else `def`. FUEL_NITRO overrides when set.
bool enabledFrom(const char* settingsValue, bool def = true);
// The action's display name (InputMap's BOOST slot; the remap page and button prompts use it).
const char* actionDisplayName();
// Headless runs (FUEL_NITRO_HOLD = "T0-T1[,T0-T1...]" seconds of the run): the button held at time t.
bool scriptedHold(const char* spec, double t);

enum Kind : int { KindNone = 0, KindAir, KindSlide, KindOvertake, KindNearMiss, kKinds };
const char* kindName(int k);

// One vehicle's tank.
struct Tank {
    float level = 1;          // 0..1
    float amount = 0;         // 0..1: the boost applied (ramped)
    bool active = false;      // engaged: held with fuel (it drains and pushes only with wheels on the ground)
    bool firing = false;      // active and on the ground this step
    bool held = false;
    bool needRelease = false; // ran dry while held: no boost until the button is let go
    float idle = 10;          // s since the last boost
    float emptyFlash = 0;     // s left of the HUD's empty flash
    float bonusGlow = 0;      // s left of the HUD's bonus shimmer
    int lastKind = KindNone;
    float lastBonus = 0;
    // statistics (traces, --ai-check)
    int starts = 0;
    float boosted = 0;
    std::array<float, kKinds> earned{};  // seconds of boost earned per kind (before the cap)
    std::array<int, kKinds> events{};    // bonus events per kind (a powerslide adds one per physics step: 60 a second)
    void reset(const Params& p);
    // One step: want = the button; grounded = a wheel on the ground; allowed = the vehicle may boost (not wrecked, the
    // race running or free roam).
    void step(float dt, bool want, bool grounded, bool allowed, const Params& p);
    // A bonus fill of `seconds` of boost (capped at a full tank); returns the seconds added.
    float add(float seconds, int kind, const Params& p);
};

// ---- the boost's physics (FuelVehicle::setNitro, fuel_vehicle.cpp)
// 0x13c0 = 1 + amount x torqueGain x power (1 when amount = 0).
float torqueMultiplier(float amount, const Params& p);
// The wheels' quadratic drag blend b of 0x520fd0 (P91 -> P92): amount x dragRelease, at most 1.
float wheelDragBlend(float amount, const Params& p);
// The push's fade: 1 below 0.85 x vmax, 0 at vmax = (1 + lift x power) x topSpeed (m/s; topSpeed <= 0: no fade).
float thrustFade(float speed, float topSpeed, const Params& p);
// The engine's rev range while boosting: FUEL's torque curve (0x524c70, 0 at P324) and the rpm clamp of 0x5212d0 (P324)
// stretched by 1 + lift x power x amount, so a rev-limited vehicle's top gear reaches the same lifted top speed.
float rpmStretch(float amount, const Params& p);
// The push (m/s^2) at `amount`, forward speed `speed`, with `loaded` = the fraction of wheels on the ground.
float thrustAccel(float amount, float speed, float topSpeed, float loaded, const Params& p);
// FUEL's top speed of a vehicle in m/s from VehiclePP 619 (km/h on FUEL's speedometer, which reads 15 % high:
// 0x5cf2d0's |v| / (P619 x 0.86956525 x 0.2777778)).
float topSpeedMs(float p619);

// ---- the bonus detectors (per vehicle, per physics step)
// The slip angle (degrees, 0..180) between the velocity and the heading in the plane across `up`.
float slipAngleDeg(const std::array<float, 3>& vel, const std::array<float, 3>& fwd, const std::array<float, 3>& up);
// A landed flight of `flight` seconds -> the fill (seconds of boost).
float airBonus(float flight, const Params& p);
struct Detector {
    float flight = 0;   // the airtime of the flight under way (FUEL's +0x1258)
    float slide = 0;    // s of powerslide so far
    // Landing: `airtime` = the model's airtime this step (0 on the ground). Returns the fill of a flight that just
    // ended upright and not wrecked; a respawn ends a flight with nothing.
    float air(float airtime, bool upright, bool wrecked, bool respawned, const Params& p);
    // Powerslide: per step; returns the fill earned this step.
    float powerslide(float dt, float slipDeg, float speed, bool grounded, bool wrecked, const Params& p);
};
// A near miss between vehicle i and another vehicle j, followed from i's frame: lat / lon = j's centre across / along
// i's heading (m), halfW / halfL = the two half widths / lengths summed, closing = |relative velocity . i's heading|.
struct NearMissPair {
    bool close = false;      // following a pass
    bool touched = false;    // the bodies met during it
    float side = 0;          // sign of lon when the pass began
    float minClear = 1e9f;   // smallest side clearance during it
    float cooldown = 0;
    // Returns the fill when j has just passed i (lon changed sign) with a clear side gap <= nearMissGap.
    float step(float dt, float lat, float lon, float halfW, float halfL, float closing, const Params& p);
};
// Race places: a place gained and held for overtakeHold earns `overtake` seconds per place.
class Overtakes {
public:
    void reset() { confirmed_.clear(), pending_.clear(), since_.clear(); }
    // pos[i] = the race place (1 = leading) of racer i; racing[i] = still in the race. Fills bonus[i] (resized).
    void step(float dt, const std::vector<int>& pos, const std::vector<bool>& racing, std::vector<float>& bonus, const Params& p);

private:
    std::vector<int> confirmed_, pending_;
    std::vector<float> since_;
};

}  // namespace fuel::nitro
