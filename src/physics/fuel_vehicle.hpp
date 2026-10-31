// FUEL's vehicle model: IT_Dyn_VehicleMove_G (FUEL.exe 0x51e000-0x52f000) on top of the engine's rigid
// body and contacts (fuel_contacts.hpp). Decoded in ~/re-work/fuel/notes/p23-vehicle-physics.md; port notes
// and deviations in p24-physics-port.md. One vehicle = one rigid body with extra integrated degrees of
// freedom (engine rpm + one rim speed per wheel); wheels are penalty contacts of oversized spheres; the
// tyre forces come from 8-coefficient friction curves scaled by per-surface tables (pp001 PP 4000-4398).
//
// Parameters: P[n] = VehiclePP n of the player's VehiclesParam.tsc (P[0] unused); the global ground
// tables from the player's pp001.tsc. Nothing from the game is compiled in.
#pragma once

#include "fuel_contacts.hpp"

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

// What the world gives the vehicles: terrain triangles, static objects, water, ground classes.
struct StaticHit {
    Vec3 p, n;       // contact point on the object, normal towards the query shape
    float depth = 0; // penetration
};
struct Environment {
    TerrainTriangles* terrain = nullptr;
    // Static objects touching a sphere (centre, radius), local coordinates.
    std::function<void(const Vec3& c, float r, std::vector<StaticHit>& out)> sphereStatics;
    // Static objects touching an oriented box (centre, rotation, half extents), local coordinates.
    std::function<void(const Vec3& c, const Quat& q, const Vec3& half, std::vector<StaticHit>& out)> boxStatics;
    // Water depth at a world point (0 = dry): P22's water will plug in here.
    std::function<float(double x, double z)> waterDepthAt;
    // Ground class fractions (18, summing to 1) at a world point; depth = soft/water sink depth.
    std::function<void(double x, double z, float frac[kGroundClasses], float& sinkDepth)> groundClasses;
    double originX = 0, originZ = 0;  // local -> world
};

class FuelVehicle {
public:
    // `P` indexed by parameter number. Throws when the data is implausible.
    FuelVehicle(std::vector<float> P, const ProxyShapes& proxy, const GroundTables* ground, bool player);

    // Placement (VehMove_ResetTo 0x5273b0 with snap): the model origin at `origin` (local), facing `yaw`.
    void resetTo(const Vec3& modelOrigin, float yaw, Environment& env);
    void setInputs(const Inputs& in) { in_ = in; }
    void setAi(bool ai) { player_ = !ai; }

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
    int gear() const { return gear_; }
    float speed() const { return speed_; }
    float forwardSpeed() const { return vfwd_; }
    float wheelSpin(int w) const;           // rim angle (radians) for the visuals
    float wheelSteer(int w) const;          // steer angle of wheel w (radians, + = left)
    float visualLean() const { return leanRender_; }  // bikes (0x529ce0), radians about forward
    float airtime() const { return airtime_; }
    bool wrecked() const { return wrecked_ != 0; }
    Vec3 comLocal() const { return com_; }  // model space
    // Model origin pose (position of the model's y = 0 origin, orientation).
    Vec3 modelOrigin() const { return body_.S.x - rotate(body_.S.q, com_); }
    const GroundBlock& ground(int w) const { return C_[size_t(w)]; }
    float wheelLoad(int w) const { return wheels_[size_t(w)].load; }
    float wheelUtil(int w) const;
    float param(int n) const { return n > 0 && size_t(n) < P_.size() ? P_[size_t(n)] : 0.0f; }
    // Wheel hub in model space (sphere centre of the proxy).
    Vec3 hubModel(int w) const { return wheels_[size_t(w)].hubL + com_; }
    bool front(int w) const { return w == 0 || w == 2; }
    // A reset was requested by the vehicle's own logic (wreck, overturn, fell out): the owner places it.
    bool wantsRespawn() const { return respawnWanted_; }
    void clearRespawn() { respawnWanted_ = false; }

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
    // Exposed so the vector check can drive single functions (0x52e830, 0x524710, 0x521f80, 0x52bb60).
    void steerChannels(float dt);   // 0x52e830 (channels part)
    void steerAngle(float dt);      // 0x524710
    void steerAccel(float dt);      // 0x521f80
    void assists(float dt);         // 0x52bb60
    float throttleEff() const;      // 0x524160
    float brakeEff() const;         // 0x523e90
    float engineTorque(float rpm) const;  // 0x524c70
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
    void wreckDetect(float dt);               // 0x528e40
    void computeMassProperties();             // 0x712de0
    void buildWheels();                       // 0x52def0
    void groundProbe(Environment& env);       // 0x5298d0 (terrain only)

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
    std::vector<float> waterDepth_;  // veh+0x1990
    Vec3 groundQueryAt_{1e9f, 0, 1e9f};

    Inputs in_;
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
    float launchRamp_ = 1;        // +0x12a0
    float dispRpm_ = 2000;        // +0x1280
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
    float underwater_ = 0;        // +0x1404
    float wreckTime_ = 0;         // +0x1384
    float damage_ = 0;
    uint8_t wrecked_ = 0;         // +0x1420 (1 wrecked, 2 drowned)
    bool airborneLong_ = false;   // +0x1424
    bool inWater_ = false;        // +0x1428
    bool respawnWanted_ = false;
    float respawnHold_ = 0;
    Vec3 refUp_{0, 1, 0}, refUp2_{0, 1, 0};  // +0x103c, +0x1048
    Vec3 probeNormal_{0, 1, 0};   // +0x1030
    float groundHeight_ = 0, heightAbove_ = 0;  // +0x13b4, +0x13b8
    float maxWaterDepth_ = 0;     // +0x1380
};

// The island step for a set of vehicles (PhysicWorld_Z::Update 0x680f90 -> 0x685170 -> 0x684a40):
// pre-step, substeps of at most 0.01 s (each: wheel updates, contacts, forces, driveline, integration,
// impulse pass, post-substep), post-step.
void stepVehicles(const std::vector<FuelVehicle*>& vehicles, Environment& env, float dt);

}  // namespace fuel::fv
