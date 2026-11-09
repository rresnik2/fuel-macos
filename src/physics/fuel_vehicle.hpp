// FUEL's vehicle model: IT_Dyn_VehicleMove_G (FUEL.exe 0x51e000-0x52f000) on top of the engine's rigid
// body and contacts (fuel_contacts.hpp). Decoded in ~/re-work/fuel/notes/p23-vehicle-physics.md; port notes
// and deviations in p24-physics-port.md. One vehicle = one rigid body with extra integrated degrees of
// freedom (engine rpm + one rim speed per wheel); wheels are penalty contacts of oversized spheres; the
// tyre forces come from 8-coefficient friction curves scaled by per-surface tables (pp001 PP 4000-4398).
//
// Parameters: P[n] = VehiclePP n of the player's VehiclesParam.tsc (P[0] unused); the global ground
// tables from the player's pp001.tsc. Nothing from the game is compiled in.
#pragma once

#include <algorithm>
#include <cmath>
#include <limits>

#include "fuel_collide.hpp"
#include "fuel_contacts.hpp"
#include "nitro.hpp"  // [upgrades U1] nitro::Params per vehicle

#include <array>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace fuel::fv {

constexpr int kGroundClasses = 18;

// Per-surface tables of pp001.tsc (Vehicle_GroundClassProps 0x5c6790): rows PP 4000.. by class k at row+1+k.
struct GroundTables {
    // dry / wet rows: STATIC, KINETIC, DYNAMIC, ROLLING, SOFTNESS, SIDE SLOW
    std::array<float, kGroundClasses> stat[2]{}, kin[2]{}, dyn[2]{}, rolling[2]{}, soft[2]{}, sideSlow[2]{};
    std::array<float, kGroundClasses> dirty[2]{};
    std::array<float, kGroundClasses> bumpsRot{}, bumpsWheel{}, depth{}, boost{}, tremPitch{}, tremRoll{}, rollDamage{};
    bool loaded = false;
};
// Reads PP 4000-4398 from pp001.tsc text; `loaded` false when the rows are missing.
GroundTables parseGroundTables(const std::string& pp001Text);

// The ground block C[w] (veh+0x16a8 + 0x7c w), built by Vehicle_BuildGroundBlock 0x5c6890.
struct GroundBlock {
    float stat = 1, kin = 0.8f, dyn = 1, rolling = 0.01f, soft = 0, hard = 1, sideSlow = 0, dirty = 0;
    float bumpsRot = 0, bumpsWheel = 0, depth = 0, boost = 1, tremPitch = 0, tremRoll = 0, rollDamage = 0;
};
GroundBlock buildGroundBlock(const GroundTables& t, const float frac[kGroundClasses], float wet);

// The physics proxy `<V>_DYN` in vehicle model space (y up, +z forward, ground at y = 0).
struct ProxyShapes {
    struct Sphere {
        Vec3 c;
        float r = 0;
        uint32_t name = 0;
    };
    struct Box {
        Vec3 c, half;
        float r = 0;  // bounding radius of the record (+0xa0), the 72-byte box's radius; 0 = |half|
    };
    std::vector<Sphere> spheres;
    std::vector<Box> boxes;
};

// The input block (phys+0x1090) fields the physics reads.
struct Inputs {
    float accel = 0, brake = 0;  // 0..1
    float steer = 0;             // stick X, right +
    float stickZ = 0;            // stick up/down: lean fore/aft, air pitch
    bool handbrake = false, boost = false, respawn = false;
};

class FuelVehicle;

// Damage record helpers (P46): 10 slots in [0, 1] and the total at [10].
using DamageRecord = std::array<float, 11>;
void damageAdd(DamageRecord& r, int slot, float v);                                  // 0x420180
void damageDirectional(DamageRecord& r, const float dirs[30], const Vec3& dirL, float v);  // 0x51fd50 (dirs null: total only)
void damageLower(DamageRecord& r, float amount, float floor);                        // 0x4200c0 (repair: 1, 0)
float damageMax(const DamageRecord& r);                                              // 0x41ff30
// The crash meter 0x51ee90 on one accepted contact (rel = R[0x10..0x12], the relative velocity at the point).
void crashMeter(const Contact& c, RigidBody& body, const Vec3& rel);
// P87: the crash-debris branch of Vehicle_ImpactFx 0x51e9e0 for vehicle `self` on contact `c` (rel = |R[0x10..0x12]|'s
// vector): sets self.debrisPieces / debrisAt / debrisSerial and the FX timer (see RigidBody::fxTimer).
void impactDebris(const Contact& c, RigidBody& self, const Vec3& rel, const RigidBody* bodyA = nullptr);  // PK23: bodyA = R[0x37]

// What the world gives the vehicles: terrain triangles, static objects, water, ground classes.
struct StaticHit {
    Vec3 p, n;       // contact point on the object, normal towards the query shape
    float depth = 0; // penetration
};
// FUEL's static scene around a body (P46): the world lists 0x71dd80 gathers for an island (OBB records +0x134,
// spheres +0x140, triangles +0x150) from the scene nodes' Lod_Z / Mesh_Z collision, in local coordinates.
struct StaticScene {
    std::vector<Obb> boxes;
    std::vector<std::pair<Vec3, float>> spheres;
    std::vector<const Tri*> tris;
    void clear() { boxes.clear(), spheres.clear(), tris.clear(); }
};
struct Environment {
    TerrainTriangles* terrain = nullptr;
    // The static scene touching a sphere (0x71dd80 / 0x639ba0), local coordinates; FUEL's tests run on it.
    std::function<void(const Vec3& c, float r, StaticScene& out)> gatherStatics;
    // Tree and bush trunks touching a sphere (centre, radius), local coordinates (port: Jolt cylinders standing in
    // for 0x74ee80 mode 2 / 0x74ead0).
    std::function<void(const Vec3& c, float r, std::vector<StaticHit>& out)> sphereStatics;
    // Trunks touching an oriented box (centre, rotation, half extents), local coordinates.
    std::function<void(const Vec3& c, const Quat& q, const Vec3& half, std::vector<StaticHit>& out)> boxStatics;
    // Water depth at a world point (0 = dry): P22's water will plug in here.
    std::function<float(double x, double z)> waterDepthAt;
    // Ground class fractions (18, summing to 1) at a world point; depth = soft/water sink depth.
    // (x, y, z): the query point of 0x5c9e80 (mean of the wheel contact points; y decides the water depth and the
    // "more than 1 m above the ground" rule of 0x6b76a0).
    std::function<void(double x, double y, double z, float frac[kGroundClasses], float& sinkDepth)> groundClasses;
    // P61: Tornado_NearestDistance 0x5bc950 at a world point: horizontal distance to the nearest tornado agent of the
    // list 0xa66c08 and its radius (vt+0x1a4). Unset: no tornadoes.
    std::function<float(double x, double z, float& radius)> tornadoDistance;
    double originX = 0, originZ = 0;  // local -> world
};

// VehMove_GroundHeightProbe 0x529600 (P70b): the scene ray from y + 0.25 [0x9dbfb0] 100 m [0x9dbfc0] down gives
// `hitY` (NaN = no hit; a hit below -10000 [0x9dcb54] is ignored), the terrain height 0x40fe70 `terrain`; the
// result is the hit when strictly higher, else the terrain. Checked by fuelphysvec BCG*.
inline float groundHeightProbe(float hitY, float terrain) {
    float v = -10000.0f;
    if (!std::isnan(hitY) && !(hitY < -10000.0f)) v = hitY;
    return v > terrain ? v : terrain;
}

class FuelVehicle {
public:
    // `P` indexed by parameter number. Throws when the data is implausible.
    FuelVehicle(std::vector<float> P, const ProxyShapes& proxy, const GroundTables* ground, bool player);

    // Placement (VehMove_ResetTo 0x5273b0 with snap): the model origin at `origin` (local), facing `yaw`.
    // `staticFloor` (local y, NaN = none): the static-object hit of VehMove_GroundHeightProbe 0x529600 under the
    // target; the ground height is max(that, the terrain) as in 0x529600 (P70b).
    void resetTo(const Vec3& modelOrigin, float yaw, Environment& env, float staticFloor = std::numeric_limits<float>::quiet_NaN());
    void setInputs(const Inputs& in) { in_ = in; }
    const Inputs& inputs() const { return in_; }
    // P103: 0x5cd8f0, the pedal the wheel sounds and effects read (0x5da710): 1 when wrecked (+0x1420 != 0), else the
    // accel pedal (+0x109c) in reverse gear (+0x1270 == -1) and the brake pedal (+0x10b0) otherwise, at least 0.
    float slipPedal() const { return wrecked_ != 0 ? 1.0f : std::max(0.0f, gear_ == -1 ? in_.accel : in_.brake); }
    void setAi(bool ai) { player_ = !ai; }
    // P87: Vehicle_ImpactFx 0x51e9e0 reads the vehicle class (VehicleInfo+0x38, 0x56a580: BUGGY 0, BIKE 1, MUSCLE 2,
    // QUAD 3, SUV 4, TRUCK 5) and the race state (g_Game+0x334c +0x48: 6 won / 7 lost, no crash debris).
    void setVehicleClass(int c) { body_.fxClass = c; }
    void setRaceState(int s) { body_.fxRaceState = s; }
    // Guided target forces (0x522830 tail, veh+0x1421 on; P29): the AI's ghost. `pos`: CoM target and
    // target velocity (Pilot_SetTarget 0x5c9c20 -> +0xefc, 0x5c65e0 -> +0xeec); `rot`: heading target
    // (0x5c6610 -> +0xf0c). Local coordinates.
    struct Guide {
        bool pos = false, rot = false;
        Vec3 p{}, v{};
        Quat q{0, 0, 0, 1};
    };
    void setGuide(const Guide& g) { guide_ = g; }
    const Guide& guide() const { return guide_; }

    // Once per frame, before the substeps (0x526bd0).
    void preStep(float dt, Environment& env);
    // Per substep: wheel update (0x521570), contacts (0x722ac0, the vehicle's part), forces (0x70f490 with
    // body forces 0x522830), F -> a (0x70dc20), driveline (0x5211e0), integration (mode 3).
    void wheelUpdate();
    void generateContacts(Environment& env, const std::vector<FuelVehicle*>& others);
    void forces();
    void driveline();
    void integrate(float h);
    // After the impulse pass: 0x5212d0 + clutch lock 0x520790.
    void postSubstep();
    // Once per frame after the substeps (0x529ce0).
    void postStep(float dt, Environment& env);

    std::vector<Contact>& contacts() { return contacts_; }
    RigidBody& body() { return body_; }
    const RigidBody& body() const { return body_; }

    // ---- read-outs ----
    int wheelCount() const { return nWheels_; }
    const WheelRec& wheel(int w) const { return wheels_[size_t(w)]; }
    float rpm() const { return body_.S.auxVel[0]; }
    float displayRpm() const { return dispRpm_; }
    float soundRpm() const { return soundRpm_; }         // +0x1284, exported to veh+0x1080 (the sound's rpm)
    float engineThrottle() const { return engThrottle_; } // +0x13bc, exported to veh+0x1088
    bool revLimiter() const { return limiter_; }         // +0x142a (rev limiter hit this step)
    int gear() const { return gear_; }
    float speed() const { return speed_; }
    float forwardSpeed() const { return vfwd_; }
    float meanWheelSpeed() const;           // PK16 0x5257b0: all wheels' rim speed (the AI throttle's reference speed)
    float wheelSpin(int w) const;           // rim angle (radians) for the visuals
    float wheelSteer(int w) const;          // steer angle of wheel w (radians, + = left)
    float visualLean() const { return leanRender_; }  // bikes (0x529ce0), radians about forward
    float airtime() const { return airtime_; }
    bool wrecked() const { return wrecked_ != 0; }
    bool riderEjected() const { return riderEjected_; }  // wrecked with P655 > 0.5: the rider goes into the ragdoll
    // PK14: 0x632570 (ENDING_WON / LOST, every frame): Rider_Restore 0x5c8c30 (ragdoll destroyed: the rider is back on)
    // and Damage_Lower(1.0, 0) on the impact record (+0x144, record 1) only; the wreck state is left as it is.
    void endingRestore() {
        riderEjected_ = false;
        damageLower(dmg_[1], 1.0f, 0.0f);
    }
    // Damage (P46): the HUD's needle = 0x4235b0 = the largest slot of record 1 (pm+0x144); records for checks.
    float damage() const { return maxDamage(1); }
    const std::array<float, 11>& damageRecord(int r) const { return dmg_[size_t(r)]; }
    // ---- PK22: 0x529ce0's dirt block on record 2 (pm+0x170), once per frame: washed in water (P597) and by rain
    // >= 0.25 (P598), raised on loaded wheels by the mean ground dirtiness x P595 x (speed - P594) and dried by P596
    // (core/vehicle_links.hpp stepDirt). `rain` = the weather's +0xb8 (game +0x3330).
    void stepDirt(float dt, float rain);
    // ---- PK22 end
    // The 10 directions of 0x51fd50 (30 floats from the player's FUEL.exe at 0xa057a8); none = no directional damage.
    void setDamageDirections(const float* d30);
    // Checks and films: a full slot 9 of record 1 (as the deep-water / 100 m fall events), wrecking next frame.
    void addTestDamage() { addDamage(1, 9, 1.0f); }
    void addTestDamage(int slot, float v) { addDamage(1, slot, v); }  // P85: FUEL_TEST_DAMAGE (visual damage films)
    float impactMeter() const { return body_.impactMeter; }
    float scrapeMeter() const { return body_.scrapeMeter; }
    // ---- read-outs for the camera and the rider (p25/p27) ----
    int wreckState() const { return wrecked_; }           // +0x1420: 0 ok, 1 wrecked, 2 tornado
    float heightAboveGround() const { return heightAbove_; }  // +0x13b8
    bool slidingMajority() const;                         // 0x52f100: rear wheel(s) 1 (and 3, 4, 5) sliding
    bool anyLoaded() const { return anyWheelLoaded(); }   // 0x52e620
    float maxWheelBumps() const;                          // 0x526a30: max over wheels of C[w]+0x64 (BUMPS WHEEL)
    float meanWheelBumpsRot() const;                      // mean of C[w]+0x60 (veh+0x1708), the rider's action 43
    Vec3 comLocal() const { return com_; }  // model space
    // Model origin pose (position of the model's y = 0 origin, orientation).
    Vec3 modelOrigin() const { return body_.S.x - rotate(body_.S.q, com_); }
    // PK09: the chassis collision shapes in model space (place them with the model origin pose): the proxy spheres
    // that are not wheels, lowered by P10 as chassisSpheres() does, and the box records (axes = the model's).
    void modelShapes(std::vector<std::pair<Vec3, float>>& spheres, std::vector<ProxyShapes::Box>& boxes) const;
    const GroundBlock& ground(int w) const { return C_[size_t(w)]; }
    float wheelLoad(int w) const { return wheels_[size_t(w)].load; }
    float wheelUtil(int w) const;
    // ---- read-outs for the wheel effects (p30, SpecialEffectsWheelEmitter_G feed 0x5ceb60) ----
    float rimSpeed(int w) const { return body_.S.auxVel[1 + w]; }  // m/s (aux state, +0x15b8 = -rim/r is the engine's export)
    float wheelRadius(int w) const { return wheelR(w); }            // P49 front / P50 rear
    float wheelSlipRatio(int w) const { return slipRatio(w); }      // 0x523c50 (1e5 when unloaded)
    const std::array<float, kGroundClasses>& classFractions(int w) const { return frac_[size_t(w)]; }  // veh+0x10a8+0x48w
    float waterDepth(int w) const { return waterDepth_[size_t(w)]; }                                    // veh+0x1990+4w
    Vec3 pointVelocity(const Vec3& p) const { return body_.pointVel(p); }                              // 0x5cc370 (world)
    float param(int n) const { return n > 0 && size_t(n) < P_.size() ? P_[size_t(n)] : 0.0f; }
    // Wheel hub in model space (sphere centre of the proxy).
    Vec3 hubModel(int w) const { return wheels_[size_t(w)].hubL + com_; }
    // 0x5d7a90 (mode = w): the wheel hub offset in model space, VehiclePP 57-59 (front) / 60-62 (rear), x negated on
    // the -x side. Added to the proxy sphere by 0x52def0 / 0x5c8620 (physics) and to the wheel node by 0x5caf40.
    Vec3 hubOffset(int w) const;
    // Model-space hub of wheel w before the CoM: the proxy sphere centre (FUEL adds hubOffset(w): see hubRest).
    Vec3 hubRest(int w) const;
    // The per-wheel compression the physics exports to the vehicle (0x529ce0 asm 0x52a96b..0x52a9e7 -> veh+0x15e8):
    // the wheel load rec+0x18c, plus the rumble amplitude +0x1310 when the load is not 0.
    float exportedCompression(int w) const {
        const float l = wheels_[size_t(w)].load;
        return l != 0 ? l + rumbleA_[w] : l;
    }
    // 0x5caf40 swing-arm camber of the drawn wheel: about the body's forward axis by atan2(c + P67, P66), negated
    // for wheels 2..5 (the -x side); c = the smoothed exported compression (+0x1618); 0 when P66 == 0.
    float visualCamber(int w, float smoothedCompression) const;
    bool front(int w) const { return w == 0 || w == 2; }
    // A reset was requested by the vehicle's own logic (wreck, overturn, fell out): the owner places it.
    bool wantsRespawn() const { return respawnWanted_; }
    void clearRespawn() { respawnWanted_ = false; }
    int respawnReason() const { return respawnWhy_; }
    // Read-outs for the breadcrumbs (P70b, 0x5281c0 / 0x528780 / 0x5283b0).
    Vec3 axisX() const { return lat(); }                  // 0x410510 of +0x7f0 with (1, 0, 0)
    Vec3 axisY() const { return up(); }                   // +0x4e0
    Vec3 axisZ() const { return fwd(); }                  // +0x4fc..+0x504
    float probeNormalY() const { return probeNormal_.y; } // +0x1034
    float maxWaterDepth() const { return maxWaterDepth_; }  // +0x1380
    bool allLoaded() const { return allWheelsLoaded(); }  // 0x52e6d0
    float extentRadius() const { return extentR_; }       // +0x404 (0x712b60)

    // ---- control-layer pieces exposed for the test vectors ----
    struct Steer {
        float chA = 0, chB = 0, chC = 0, chD = 0;  // +0x136c..+0x1378
        float feedback = 0;                        // +0x13ac
        float sinceSteer = 0;                      // +0x13e0
        float base = 0;                            // +0x135c
        float tanF = 0;                            // +0x1354
        float drift = 0;                           // +0x1388
        float accelMag = 0;                        // +0x13c4
        float accel = 0;                           // +0x13c8
        Vec3 dirB{0, 0, 1}, dirF{0, 0, 1};         // +0xfdc, +0xfe8
        Vec3 dir0{0, 0, 1}, dir2{0, 0, 1};         // +0x1000, +0x1018
    };
    Steer& steer() { return st_; }
    const Steer& steer() const { return st_; }
    // Exposed so the vector check can drive single functions (0x52e830, 0x524710, 0x521f80, 0x52bb60).
    void steerChannels(float dt);   // 0x52e830 (channels part)
    void steerAngle(float dt);      // 0x524710
    void steerAccel(float dt);      // 0x521f80
    void assists(float dt);         // 0x52bb60
    float throttleEff() const;      // 0x524160
    float brakeEff() const;         // 0x523e90
    float engineTorque(float rpm) const;  // 0x524c70 (x the port's governed power scale; [nitro] the curve stretched while boosting)
    // [DEVIATION nitro, the user's request 2026-11-06; physics/nitro.hpp]: the boost amount 0..1 of this frame (the
    // vehicle's tank, DriveWorld) and its top speed (m/s, the push's fade). Amount 0 (the default, and always with
    // `nitrous = off`) leaves every force as FUEL's: 0x13c0 = 1, the 0x520fd0 drag blend 0 and scale 1, the rev range
    // (0x524c70 curve, 0x5212d0 clamp) P324, no push.
    void setNitro(float amount, float topSpeed);
    float nitro() const { return nitro_; }
    // [upgrades U1] this vehicle's nitro parameters (its nitrous stages; DriveWorld owns them); nullptr = nitro::params().
    void setNitroParams(const nitro::Params* p) { nitroParams_ = p; }
    const nitro::Params& nitroParams() const { return nitroParams_ ? *nitroParams_ : nitro::params(); }
    // [DEVIATION, the user's request 2026-11-05]: every vehicle's engine torque curve x this (settings.txt
    // `engine_power`, default 1 = FUEL). 1.25 = 25 % more power. Not used by the physics vectors (they keep 1).
    static inline float powerScale = 1.0f;
    // [DEVIATION, truck-power: notes/truck-power.md] the vehicle's own torque multiplier on top of powerScale (the
    // upgrade plan's engine and weight stages; default 1), and the traction governor on the extra torque: the engine
    // gives curve x P327 x (1 + (powerScale x torqueScale - 1) x g). g (0..1) falls while a loaded driven wheel's grip
    // ratio 0x523c50 is over kGovRatio (FUEL's direct-drive push, 0x520560 -> 0x5213c0, and the clutch push 0x520b00
    // both scale with the torque, and past the tyre curve's peak more push means less drive: see the notes) and rises
    // again below it. At a scale of exactly 1 nothing of this runs and the torque is FUEL's, bit for bit.
    // [merge with nitro] The engine step governs the boost's torque gain too: s = powerScale x torqueScale x +0x13c0
    // (nitroTorque, after its own slide cut, which shrinks the boost amount only while the driven wheels slide). The
    // governor therefore also runs at engine_power 1 while boosting (FUEL's 10 % boost torque flip-flopped direct-drive
    // trucks: TRUCK_ALL_5 0-60 2.60 s with the boost vs 2.10 s without). The gearbox (0x524d70) sees engineTorque (no
    // boost), as on main. Without a boost and at s == 1 nothing of this runs. See notes truck-power.md, "Merge into main".
    void setTorqueScale(float s) { torqueScale_ = s; }
    float torqueScale() const { return torqueScale_; }
    float tractionGovernor() const { return gov_; }
    // [DEVIATION, upgrade plan] physical suspension lift and track widening (metres; 0 = FUEL): every wheel centre
    // `lift` lower and front / rear wheels `widen` further out (hubRest). Call before resetTo.
    void setHubShift(float lift, float widenFront, float widenRear);
    // [upgrades U1] an upgraded vehicle (any stage): the gear-1 hold lets go at the limiter as with extra torque (a
    // grippier or lighter bike fell into FUEL's 5 s crawl at engine_power 1: BIKE_ALL_1 0-60 2.15 -> 5.80 s at T1).
    void setUpgraded(bool on) { upgraded_ = on; }
    float governedTorque(float rpm, float s) const;  // the curve x P327 x (1 + (s - 1) x g); s == 1: FUEL's exactly
    // [upgrades U1] one frame of the governor: g -> 0 with the clutch slipping (or in neutral), else down kGovDown / s
    // while the largest loaded driven grip ratio is over kGovRatio, up kGovUp / s below it; clamped to 0..1.
    // [upgrades U1] the traction hold on FUEL's own torque (upgraded vehicles only): down kGovDown / s to kHoldFloor while
    // g is 0 and the largest loaded driven grip ratio is over kHoldRatio (sliding), else up kGovUp / s to 1.
    static float holdNext(float h, float g, float ratio, float dt) {
        if (kHoldFloor >= 1.0f) return 1.0f;
        return g <= 0.0f && ratio > kHoldRatio ? std::max(h - kGovDown * dt, kHoldFloor) : std::min(h + kGovUp * dt, 1.0f);
    }
    static inline float kHoldFloor = 0.85f, kHoldRatio = 1.0f;  // 0.7 bogged BUGGY_ROAD_3 at W1 (5 s 81.5 -> 72.0 m)  // bench-tunable (fueldrive --hold F,R)
    static float governorNext(float g, float ratio, bool clutchSlipping, float dt) {
        if (clutchSlipping) return 0.0f;
        return ratio > kGovRatio ? std::max(g - kGovDown * dt, 0.0f) : std::min(g + kGovUp * dt, 1.0f);
    }
    static inline bool nitroGoverned = true;  // the boost's torque in s (false = main's ungoverned boost: fueldrive --nitro-ungoverned)
    static inline bool governorOn = true, govClutch = true, holdRelease = true;  // bench switch (fueldrive --no-governor)
    static inline float kGovRatio = 0.8f, kGovDown = 8.0f, kGovUp = 2.0f;  // bench-tunable (fueldrive --gov R,DOWN,UP)
    float rpmTarget() const;              // 0x5259f0 (the display/sound rpm's target)
    // Bench read-outs (fueldrive --flat): the clutch fraction 0x525580, the engine's wheel push dOmega (+0x12a4) of
    // the last driveline step, the gear the sequencer is heading for (+0x1274), the shift timer (+0x1268).
    float clutchNow() const { return clutchFraction(); }
    float driveDOmega(int w) const { return dOmega_[w]; }
    int gearTarget() const { return gearTarget_; }
    float shiftTimer() const { return shiftT_; }
    float sinceSteer() const { return st_.sinceSteer; }
    float auxAcc(int i) const { return body_.S.auxAcc[i]; }
    // Bench hook (null = off): called per substep after the forces (0), after the driveline (1), after 0x5212d0 (2).
    static inline void (*substepHook)(const FuelVehicle&, int phase) = nullptr;
    void setSpeedForTest(float v) { speed_ = v; }
    void setDriftForTest(float d) { st_.drift = d; }
    void setAutoBrake(float a) { autoBrake_ = a; }
    void setSlopeGrip(float s) { slopeGrip_ = s; }
    void setAirtime(float a) { airtime_ = a; }
    void setRefUp(const Vec3& n) { refUp_ = n; }
    void setAirTimer(float t) { airAssist_ = t; }
    float airTimer() const { return airAssist_; }
    void setGear(int g) { gear_ = g; }
    void setWheelSliding(int w, bool sliding, float load = 1);
    void setGroundBlock(int w, const GroundBlock& c, const float frac[kGroundClasses]);

private:
    friend struct FuelVehicleTest;
    // Body frame axes (world).
    Vec3 up() const { return rotate(body_.S.q, {0, 1, 0}); }
    Vec3 lat() const { return rotate(body_.S.q, {1, 0, 0}); }
    Vec3 fwd() const { return rotate(body_.S.q, {0, 0, 1}); }
    bool driven(int w) const { return front(w) ? P(46) > 0.5f : P(47) > 0.5f; }
    float P(int n) const { return P_[size_t(n)]; }
    float wheelInertia(int w) const { return (w == 1 || w == 3) ? P(54) : P(53); }
    float wheelR(int w) const { return front(w) ? P(49) : P(50); }
    float ratio(int g) const;
    float gearMul(int g) const;
    float slipRatio(int w) const;             // 0x523c50
    bool anyWheelLoaded() const;              // 0x52e620
    bool allWheelsLoaded() const;             // 0x52e6d0
    bool wheelSlidingOrUnloaded(int w) const; // 0x524290
    bool wheelSliding(int w) const;           // 0x524240
    bool enoughWheelsSliding() const;         // 0x5243a0
    bool allFrontSliding() const;             // 0x524560
    bool allFrontSlidingOrAir() const;        // 0x5246c0
    bool drivenWheelsSliding() const;         // 0x524440
    bool drivenWheelsSlidingEnough() const;   // PK16 0x5244d0: the rpm rise mode (P390 engaged / P389 free)
    float minDrivenSlipRatio() const;         // 0x523be0
    float drivePedal() const;                 // 0x523e10
    float brakePedal() const;                 // 0x523dd0
    float notBraking() const;                 // 0x523d50
    float brakeCombined() const;              // 0x523fa0
    float wheelBrakeFactor(int w) const;      // 0x523f00
    bool handbrakeMode() const;
    float steerStick() const;                 // 0x524120
    float clutchFraction() const;             // 0x525580
    float meanDrivenRadius() const;           // 0x525680
    float meanDrivenWheelSpeed() const;       // 0x525710
    float engineSpeedAtWheel() const;         // 0x525980
    float wheelTorqueAtSpeed(float v, int g) const;  // 0x524d70
    float surfaceClassFraction(int k) const;  // 0x51ffe0
    int autoGearbox();                        // 0x525c50
    void shiftSequencer(float dt);            // 0x526030
    void engineThrottle(float dt);            // 0x525310
    void ackermann();                         // 0x526480
    void rumble(float dt);                    // 3.15
    void bodyForces();                        // 0x522830
    void aeroDrag();                          // 0x530d20
    void antiRollLift();                      // 0x52c900
    void tyreCoefficients();                  // 0x52cce0
    void comShift();                          // 0x52b630
    void updateGround(Environment& env);      // 0x5c9e80
    void wreckDetect(float dt, Environment& env);  // 0x528e40
    void computeMassProperties();             // 0x712de0
    void buildWheels();                       // 0x52def0
    void groundProbe(Environment& env);       // 0x5298d0 (terrain only)
    void nitroPush();                         // [nitro] the chassis push (forces(), after 0x522830)
    void nitroTorque(float dt);               // [nitro] +0x13c0 and its traction limiter (preStep, after its reset)

    std::vector<float> P_;
    ProxyShapes proxy_;
    const GroundTables* tables_ = nullptr;
    bool player_ = true;

    RigidBody body_;
    std::vector<WheelRec> wheels_;
    int nWheels_ = 0;
    std::vector<int> wheelSphere_;  // proxy sphere index per wheel
    Vec3 com_;                      // CoM in model space (+0x440 + model offset)
    Vec3 comBase_;                  // UP*P11 + Z*P12
    Vec3 comPrev_;                  // +0x1024
    std::vector<Contact> contacts_;
    std::vector<GroundBlock> C_;
    std::vector<std::array<float, kGroundClasses>> frac_;
    // 0x5c9e80 with veh+0x1a48 == 0 (the AI): one class query at the mean contact point, refreshed after 5 m.
    bool groundQueried_ = false;
    double groundAt_[2] = {0, 0};
    std::array<float, kGroundClasses> groundFrac_{};
    float groundSink_ = 0;
    float groundWater_ = 0;  // +0x1990 water part at the last query (0x4df5a0 level - query y)
    std::vector<float> waterDepth_;  // veh+0x1990
    Vec3 groundQueryAt_{1e9f, 0, 1e9f};

    Inputs in_;
    Guide guide_;
    Steer st_;
    // Per-wheel state.
    float dOmega_[6] = {};        // +0x12a4
    float groundTime_[6] = {};    // +0x12bc
    float rumbleT_[6] = {}, rumblePeriod_[6] = {}, rumbleA_[6] = {}, rumbleB_[6] = {};  // +0x12e0, +0x12f8, +0x1310, +0x1328
    float wheelAngle_[6] = {};    // visual
    Vec3 wheelDirPost_[6];        // +0xf4c (post-step wheel dirs 0x526760)
    // Driveline / gearbox.
    int gear_ = 1, gearTarget_ = 1, gearPrev_ = 1;  // +0x1270, +0x1274, +0x1278
    float shiftT_ = 0;            // +0x1268
    float shiftTimes_[3] = {0.1f, 0.1f, 0.1f};  // +0x124c..
    float sinceUp_ = 10, sinceDown_ = 10;       // +0x1364, +0x1368
    float hysteresis_ = 0.99f;    // +0x13a8
    float standstill_ = 0;        // +0x1088
    float peakPedal_ = 0;         // +0x139c
    float engThrottle_ = 0;       // +0x13bc
    float boostMul_ = 1;          // +0x13c0
    float nitro_ = 0, nitroTop_ = 0;  // [nitro] setNitro
    float rpmStretch_ = 1;            // [nitro] the rev range x this (1 without a boost: FUEL's P324 exactly)
    float nitroGrip_ = 1;             // [nitro] the boost torque's traction limiter (1 = all of it)
    const nitro::Params* nitroParams_ = nullptr;  // [upgrades U1] setNitroParams
    float launchRamp_ = 1;        // +0x12a0
    float torqueScale_ = 1;       // port deviation (see setTorqueScale)
    float gov_ = 1;               // port deviation: traction governor on the extra torque
    bool hubShifted_ = false;     // port deviation: setHubShift
    bool upgraded_ = false;       // [upgrades U1] setUpgraded
    float hold_ = 1;              // [upgrades U1] traction hold on FUEL's torque (holdNext)
    float lift_ = 0, widenF_ = 0, widenR_ = 0;
    float dispRpm_ = 2000;        // +0x1280
    float soundRpm_ = 2000;       // +0x1284
    bool limiter_ = false;        // +0x142a
    // Speeds, timers.
    float speed_ = 0, vfwd_ = 0, vDriven_ = 0;  // +0x13cc, +0x13d4, +0x13d0
    float slopeGrip_ = 1;         // +0x134c
    float autoBrake_ = 0;         // +0x13d8
    float airtime_ = 0;           // +0x1258
    float notAllLoaded_ = 0;      // +0x125c
    float overturn_ = 0;          // +0x126c
    float airAssist_ = 0;         // +0x1350
    float bumpTimer_ = 0, bumpTimer2_ = 0;  // +0x1344, +0x1348
    float lean_ = 0, leanRender_ = 0;       // +0x1248, rendered
    float wreckTime_ = 0;         // +0x1384
    StaticScene scene_;         // the island's static scene this substep (port scratch)
    std::vector<Obb> boxes_;    // the body's box records this substep (port scratch)
    // Damage (P46): three records of 10 slots in [0, 1] and the frame's total (+0x28): pm+0x118 (0, scrapes),
    // pm+0x144 (1, impacts by direction; slot 9 = speed over rough ground, deep water, falls of 100 m; the HUD's
    // needle and the wreck rule read it), pm+0x170 (2, the dirt layer: not ported).
    std::array<DamageRecord, 3> dmg_{};
    std::array<float, 30> dmgDirs_{};  // the 10 body-frame directions of 0x51fd50 (FUEL.exe 0xa057a8, read at load)
    bool haveDmgDirs_ = false;
    float lastGroundY_ = 0;       // +0x1264: height at the last ground contact (the 100 m fall event)
    float landedAfter_ = 0;       // +0x1260: the not-all-loaded time that ended this frame (0 otherwise)
    void chassisSpheres(std::vector<std::pair<Vec3, float>>& out) const;  // world (0x71c5f0 +0xdbc)
    void chassisBoxes(std::vector<Obb>& out) const;                         // world box records
    void addDamage(int rec, int slot, float v);                 // 0x420180
    void directionalDamage(int rec, const Vec3& dirL, float v); // 0x51fd50
    void repair();                                              // 0x528d90
    float maxDamage(int rec) const;                             // 0x41ff30
    uint8_t wrecked_ = 0;         // +0x1420 (1 wrecked, 2 in a tornado: 0x5bc950)
    float tornadoTime_ = 0;       // +0x1404: time inside a tornado's radius (0x528fab)
    bool airborneLong_ = false;   // +0x1424
    bool inWater_ = false;        // +0x1428
    float waterTimer_ = 0;        // +0x13a4 (0x528cf0)
    bool riderEjected_ = false;
    bool respawnWanted_ = false;
    int respawnWhy_ = 0;          // port: 1 wreck, 2 button, 3 fell out, 4 overturned (for the checks' traces)
    float respawnHold_ = 0;
    Vec3 refUp_{0, 1, 0}, refUp2_{0, 1, 0};  // +0x103c, +0x1048
    Vec3 probeNormal_{0, 1, 0};   // +0x1030
    float groundHeight_ = 0, heightAbove_ = 0;  // +0x13b4, +0x13b8
    float maxWaterDepth_ = 0;     // +0x1380
    float extentR_ = 0.01f;       // +0x404: 0x712b60, max distance of the shapes from the centre of mass
};

// The island step for a set of vehicles (PhysicWorld_Z::Update 0x680f90 -> 0x685170 -> 0x684a40):
// pre-step, substeps of at most 0.01 s (each: wheel updates, contacts, forces, driveline, integration,
// impulse pass, post-substep), post-step.
void stepVehicles(const std::vector<FuelVehicle*>& vehicles, Environment& env, float dt);

}  // namespace fuel::fv
