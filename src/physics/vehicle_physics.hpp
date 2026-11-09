// Vehicle physics on the world terrain: FUEL's own vehicle model (physics/fuel_vehicle.hpp, decoded from
// FUEL.exe; notes p23-vehicle-physics.md, p24-physics-port.md) driving every vehicle. The wheels touch the
// terrain's collision triangles as the engine builds them (GenWorld_Z 0x77f510); Jolt Physics is kept only
// for the placed static objects, the movable props and ray/sphere queries.
#pragma once

#include "core/dpc.hpp"
#include "core/surface.hpp"
#include "core/terrain.hpp"
#include "core/vehicle_params.hpp"
#include "physics/fuel_vehicle.hpp"
#include "physics/nitro.hpp"

#include "core/static_collision.hpp"

#include <array>
#include <functional>
#include <memory>
#include <string>
#include <vector>

namespace fuel {

// Everything the simulation needs to know about one vehicle, in vehicle space
// (y up, +z forward, ground at y = 0).
struct VehicleSetup {
    std::string name;
    float mass = 300;  // kg
    struct Box {
        std::array<float, 3> centre{}, halfExtents{};
    };
    std::vector<Box> chassis;
    struct Wheel {
        std::array<float, 3> hub{};  // wheel centre at rest
        float radius = 0.3f, width = 0.25f;
        bool front = false;
    };
    std::vector<Wheel> wheels;  // FL, FR, RL, RR (pairs: left = +x); bikes: front, rear
    bool bike = false;          // two wheels, leans into turns
    bool frontDriven = true, rearDriven = true;
    float idleRpm = 1000, maxRpm = 6000;
    float peakTorque = 300;  // Nm at the crank
    std::array<float, 11> torqueCurve{};  // normalised torque at rpm/max = 0, 0.1 .. 1.0
    std::vector<float> gears;  // forward ratios
    float reverseGear = -3, finalDrive = 3.5f;
    float topSpeedKmh = 150;
    // What FUEL's model runs on: every VehiclePP value (index = parameter number), the `<V>_DYN` proxy and
    // the global ground tables (pp001.tsc PP 4000-4398), all read from the player's files.
    std::vector<float> params;
    VehicleProxy proxy;
    std::shared_ptr<const fv::GroundTables> ground;
    // The 10 damage directions of 0x51fd50 (FUEL.exe 0xa057a8, 30 floats; empty when the exe cannot be read).
    std::vector<float> damageDirs;
    // P87: VehicleInfo+0x38 (0x56a580 on AddVehicleInfo's class: BUGGY 0, BIKE 1, MUSCLE 2, QUAD 3, SUV 4, TRUCK 5).
    // [port] from the variant's prefix: in VehiclesDef.tsc every BIKE_* / QUAD_* / BUGGY_* / MUSCLE_* is that class.
    int vehicleClass = 0;
    // [DEVIATION, upgrade plan] the port's per-vehicle torque multiplier (FuelVehicle::setTorqueScale) and the
    // physical suspension lift / track widening in metres (FuelVehicle::setHubShift). 1 / 0 = FUEL.
    float torqueScale = 1, lift = 0, widenFront = 0, widenRear = 0;
    // [DEVIATION, upgrades U1: core/upgrades.hpp] the vehicle's nitrous stages as multipliers on the process's
    // nitro::Params: `power` (the boost's gains), `tankSeconds`, `rechargeRate`. 1 = stock: the vehicle uses
    // nitro::params() itself.
    float nitroPower = 1, nitroTank = 1, nitroRecharge = 1;
    bool upgraded = false;  // [upgrades U1] any stage applied (FuelVehicle::setUpgraded)
    // [upgrades U5] the stages apply() used (engine, suspension, tires, weight, nitro power, nitro tank; all 0 = stock):
    // the visible upgrades (core/upgrade_visuals) read them. Never read by the physics.
    std::array<int, 6> upgStages{};
};

// Builds a setup from the variant's VehiclesParam.tsc values and its physics
// proxy in the vehicle DPC. `hubs`, when given, are the assembled wheel
// centres (they can sit inboard of the proxy spheres); each proxy wheel takes
// the nearest one (for drawing only; the physics uses the proxy). The ground tables come from
// CommonComponent/Param/pp001.tsc found above the DPC. Throws when the data is missing or implausible.
VehicleSetup makeVehicleSetup(const DpcFile& vehicleDpc, const std::string& variant, const std::vector<float>& params,
                              const std::vector<std::array<float, 3>>& hubs = {});

struct Pose {
    std::array<double, 3> position{};
    std::array<float, 4> rotation{0, 0, 0, 1};  // quaternion x, y, z, w
};

struct DriveInput {
    float throttle = 0;   // 0..1
    float brake = 0;      // 0..1 (reverses when stopped)
    float steer = 0;      // -1 left .. 1 right
    bool handbrake = false;
    float stickZ = 0;     // P47: left stick up/down (FUEL UP_ACTION - DOWN_ACTION): rider lean fore/aft, air pitch
    bool nitro = false;   // [nitro] the Nitro button held (FUEL's dead BOOST action, physics/nitro.hpp)
};

struct VehicleState {
    Pose chassis;
    std::vector<Pose> wheels;  // wheel model space: x = axle, y = up
    float speedKmh = 0;
    std::array<float, 3> velocity{};  // body velocity, world m/s (veh+0x820; the HUD reads its length, p40)
    float rpm = 0;
    int gear = 0;
    int wheelsOnGround = 0;
    float skid = 0;  // 0..1: how hard the tyres slide or spin (average over wheels on the ground)
    // Per wheel (same order as `wheels`):
    std::vector<int> wheelSurface;       // surface type under the contact point, -1 when airborne or unknown
    std::vector<float> wheelSkid;        // 0..1 slide/spin of this wheel (0 in the air)
    std::vector<float> wheelCompression; // suspension compression speed, m/s (positive = compressing)
    // The chassis body touching the world (terrain or an object): sliding speed along the
    // contact in m/s (0 = no contact) and the largest normal speed of a new impact this step.
    float scrapeSpeed = 0;
    float impactSpeed = 0;
    // FUEL's damage (P46): the largest slot of damage record 1 (0x4235b0, what the HUD's needle reads), and the
    // wreck state (+0x1420) with the rider's ejection (P655 > 0.5).
    float damage = 0;
    bool wrecked = false, riderEjected = false;
    // What FUEL's vehicle sound reads (0x5d8ed0): the sound rpm veh+0x1080 (= phys+0x1284), the engine
    // throttle veh+0x1088 (= phys+0x13bc, with the shift cut), the rev limiter flag phys+0x142a, the body
    // velocity (`velocity` above) and the frame's impact vector phys+0xf2c (world; n * mass factor * approach speed).
    float soundRpm = 0, engineThrottle = 0;
    bool revLimiter = false;
    std::array<float, 3> soundImpact{};
    float slipPedal = 0;    // P103: 0x5cd8f0 (1 wrecked, else the brake pedal; the accel pedal in reverse), read by 0x5da710
    float impactMeter = 0;  // phys+0x138c: the crash meter 0x51ee90 of the last step (P89: gates the impact pad rumble)
    // What FUEL's wheel effects read per wheel (p30; the vehicle feed 0x5ceb60 into
    // SpecialEffectsWheelEmitter_G), same order as `wheels`.
    struct WheelFx {
        std::array<double, 3> contact{};  // world: hub - body up x radius
        std::array<float, 3> velocity{};  // world velocity of the hub point (0x5cc370)
        std::array<float, 3> forward{};   // world: the wheel's rolling direction (body forward)
        float spin = 0;                   // rim / radius, rad/s (emitter +0xe0 = -(veh+0x15b8+4w))
        float radius = 0;
        float load = 0;                   // suspension compression (veh+0x15e8+4w)
        float slip = 0;                   // the slip ratio 0x523c50 (>= 0) while loaded (> 0.01); 0x5da710 adds slipPedal (P103)
        float depth = 0;                  // water / soft depth (veh+0x1990+4w)
        std::array<float, 18> classes{};  // ground-class fractions (veh+0x10a8+0x48w)
    };
    std::vector<WheelFx> wheelFx;
    // [nitro] the vehicle's tank (physics/nitro.hpp); `on` false with `nitrous = off`.
    struct Nitro {
        bool on = false;
        float level = 0, amount = 0;  // tank 0..1, the boost applied 0..1
        bool active = false, firing = false, held = false;
        float emptyFlash = 0, bonusGlow = 0;  // s left (HUD)
        int lastKind = 0;
        float lastBonus = 0;
        int starts = 0;
        // [upgrades U1] this vehicle's tank seconds, minimum start level (fraction) and boost power (its nitrous stages).
        float tankSeconds = 3, minStartLevel = 0.06f, power = 1;
    } nitro;
};

struct PropPose {
    uint32_t id = 0;
    Pose pose;
    bool awake = false;
    float displacement = 0;  // metres from where it was placed
};

class DriveWorld {
public:
    explicit DriveWorld(const Heightfield& terrain);
    ~DriveWorld();
    DriveWorld(const DriveWorld&) = delete;
    DriveWorld& operator=(const DriveWorld&) = delete;

    // Vehicles: any number, each with its own input; index 0 is the player. The
    // terrain collides as heightfield patches (~2 km) that follow the vehicles:
    // those near each other share one, a vehicle far from the rest gets its own.
    // Vehicles collide with each other, the props and the static objects.
    //
    // Adds a vehicle on the terrain at (x, z), facing `yaw` radians; returns its index.
    int addVehicle(const VehicleSetup& setup, double x, double z, float yaw);
    int vehicleCount() const;
    // Swaps vehicle `i` for another setup, placed at (x, z) facing `yaw`.
    void replaceVehicle(int i, const VehicleSetup& setup, double x, double z, float yaw);
    // Puts vehicle `i` on its wheels at (x, z) facing `yaw` (at rest).
    void placeVehicle(int i, double x, double z, float yaw);
    // Puts vehicle `i` back on its wheels where it is.
    void reset(int i);
    // The input vehicle `i` drives with from the next update on.
    void setInput(int i, const DriveInput& input);
    // P87: the race state FUEL's vehicles read (g_Game+0x334c +0x48: 3 streaming, 4 intro / countdown, 5 running,
    // 6 won, 7 lost); Vehicle_ImpactFx 0x51e9e0 throws no crash debris in 6 / 7.
    void setRaceState(int s);
    // PK14: 0x632570 on vehicle i (see FuelVehicle::endingRestore).
    void endingRestore(int i);
    // Advances every vehicle by `dt` seconds in fixed 1/60 s steps.
    void update(float dt);
    VehicleState state(int i) const;

    // Single-vehicle shorthands for vehicle 0. spawn adds it (or replaces it).
    void spawn(const VehicleSetup& setup, double x, double z, float yaw);
    void reset() { reset(0); }
    void update(float dt, const DriveInput& input);
    VehicleState state() const { return state(0); }

    // For checks: lifts vehicle `i` by `dy` metres and sets its velocity (world, m/s).
    void nudge(int i, double dy, const std::array<float, 3>& velocity);
    // The AI's ghost (P29, race_ai): guided target forces towards a world ground point, velocity and
    // heading (on), or none (off).
    void setGuide(int i, bool on, const std::array<double, 3>& ground = {}, const std::array<float, 3>& velocity = {},
                  const std::array<float, 4>& rotation = {0, 0, 0, 1});
    // Puts vehicle i with its model origin on `origin` (world), turned by `rotation`, moving at `velocity`
    // (FUEL places far AI vehicles on their ghost, 0x5c9760).
    void holdPose(int i, const std::array<double, 3>& origin, const std::array<float, 4>& rotation, const std::array<float, 3>& velocity);
    int respawns(int i) const;  // resets by the vehicle's own wreck/overturn logic
    // ---- [nitro] (physics/nitro.hpp). Every vehicle has a tank, stepped with the physics (1/60 s) from its input's
    // `nitro` button; air time, powerslides and near misses (opponents and traffic) fill it here, race overtakes through
    // addNitroBonus. Nothing runs with nitro::enabled() false.
    void addNitroBonus(int i, float seconds, int kind);  // seconds of boost
    void resetNitro();  // every tank to the start level (a race start)
    const nitro::Tank* nitroTank(int i) const;
    // [upgrades U1] vehicle i's nitro parameters (its nitrous stages on nitro::params(); the process's for a stock vehicle).
    const nitro::Params& nitroParams(int i) const;
    // Breadcrumb orientation targets (P70b, physics/breadcrumbs.hpp), given the player's position: `race` = 0x528690,
    // the player's race path 30 m ahead of the player's cursor (FUEL uses it for every vehicle's crumbs); `gps` =
    // 0x421dc0, the player's GPS target. Either may be empty; each returns false when there is no target.
    using CrumbTarget = std::function<bool(double playerX, double playerZ, double& x, double& z)>;
    void setCrumbTargets(CrumbTarget race, CrumbTarget gps);
    static void setDefaultCrumbGpsTarget(CrumbTarget gps);  // used while no instance GPS target is set
    // PK01: the pre-start hold (_CheckpointRespawn 0x5279c0 -> 0x62ce40, fuel::RaceRules::checkpointRespawnPose):
    // while the race state (setRaceState) is 2, 3 or 4 the player's vehicle is put back, after every physics step,
    // on the pose `hold` gives (x, z on the terrain, heading); `hold` returns false when there is none (no race).
    using StartHold = std::function<bool(double& x, double& z, float& heading)>;
    void setStartHold(StartHold hold);
    // PK01: Player_G vf137 0x42bd30 (the tail of the player agent's reset vf20 -> vf136): vehicle i with its model
    // origin 0.3 m (vf60) above the agent node's world position `node`, turned by the node's `rotation`, at rest.
    void placeOnNode(int i, const std::array<double, 3>& node, const std::array<float, 4>& rotation);
    // Lake water depth (surface - ground, 0 outside lakes) for the breadcrumb safety test 0x40fd10 -> 0x65a480
    // (the vehicle's own water query above is the depth over the 2 m floor of 0x4df5a0).
    void setLakeDepthQuery(std::function<float(double x, double z)> query);

    // Free distance (metres, up to `length`) from (x, z) along heading `yaw` (0 = +z):
    // what a driver sees ahead. A ray `height` metres above the terrain at both ends
    // against static objects (not vehicles or props); with `maxSlope` > 0 the terrain
    // also blocks where it climbs more steeply than that (rise over run, in 4 m steps) or
    // falls more than 1.6 times as steeply.
    float clearance(double x, double z, float yaw, float length, float maxSlope = 0, float height = 1.0f) const;
    // Fraction (0..1) of the segment from -> to that is clear of static objects
    // (not the terrain, vehicles or props); 1 when nothing is in the way. With `radius`
    // > 0 a sphere of that radius is swept instead of a ray (objects the sphere already
    // touches at `from` are ignored).
    float rayFraction(const std::array<double, 3>& from, const std::array<double, 3>& to, float radius = 0) const;
    // Height of the first static-object surface below (x, y, z) within `depth` metres, NaN when none: a ray that
    // starts inside an object passes through it (front faces only), as the AI ghost's ground ray (0x61b810).
    float staticFloorBelow(double x, double y, double z, float depth) const;
    // Static objects overlapping a sphere (the camera's push-out query 0x63a0b0): contact point on the object
    // (world), unit normal out of the object, and the object's size (world bounding-box volume, m^3).
    struct SphereContact {
        std::array<double, 3> p{};
        std::array<float, 3> n{};
        float volume = 0;
    };
    void overlapStatics(const std::array<double, 3>& centre, float radius, std::vector<SphereContact>& out) const;
    // World bounds (min x, y, z, max x, y, z) of the static objects (not the terrain) over an x/z rectangle.
    void staticBounds(double x0, double z0, double x1, double z1, std::vector<std::array<double, 6>>& out) const;
    // P58: the AI boxes (StaticCollision::aiBoxes) of the static objects over an x/z rectangle, in world space:
    // frame 3 x 4 rows (axis i in [4i..4i+2], centre in column 3), half extents, bounding radius, type flags.
    struct WorldBox {
        std::array<float, 12> frame{};
        std::array<float, 3> half{};
        float r = 0;
        uint32_t type = 0;
    };
    void staticBoxes(double x0, double z0, double x1, double z1, std::vector<WorldBox>& out) const;

    // Surface type (0..39) under a world point, for per-wheel grip. Without a query every
    // surface counts as dirt. The default applies to worlds created afterwards.
    void setSurfaceQuery(SurfaceQuery query);
    static void setDefaultSurfaceQuery(SurfaceQuery query);
    // FUEL's ground-class fractions (0x4572d0 -> 0x6b76a0: 18 floats) and sink depth at (x, z); when set,
    // the tyres and wheel effects read these instead of the one-hot class of the surface type.
    using ClassQuery = std::function<void(double x, double y, double z, float* frac18, float& depth)>;  // y: query height (0x5c9e80)
    static void setDefaultClassQuery(ClassQuery query);
    void setClassQuery(ClassQuery query);  // this world only (P91's --trigger-check scripts the ground)
    // Water depth (metres above the ground, 0 = dry) at a world point, for FUEL's water floor contacts,
    // water drag and wet grip. Defaults to 0 everywhere (the world's water plugs in here).
    using WaterDepthQuery = std::function<float(double x, double z)>;
    void setWaterDepthQuery(WaterDepthQuery query);
    static void setDefaultWaterDepthQuery(WaterDepthQuery query);
    float waterDepthAt(double x, double z) const;

    // Physics cost of the last update (milliseconds, wall clock) and the vehicle model's own share.
    float lastStepMs() const;
    float lastVehicleMs() const;
    float lastVehicleCpuMs() const;  // thread CPU time of the vehicle model (excludes preemption)
    // FUEL's model of vehicle `i` (null when absent), for checks and telemetry.
    const fv::FuelVehicle* model(int i) const;

    // Static world objects the vehicle collides with. A shape is registered
    // once per mesh; instances place it with a row-vector world matrix
    // (translation in 12..14; rotation and uniform or axis scale).
    bool hasStaticShape(uint32_t key) const;
    void addStaticShape(uint32_t key, const std::vector<float>& positions, const std::vector<uint32_t>& indices);
    // A round cylinder of radius 1 from y = 0 to 1 (tree and bush trunks, P52): instances scale it by (r, h, r).
    void addStaticCylinder(uint32_t key);
    // FUEL's static collision of an object (P46, core/static_collision.hpp): the vehicles collide with it
    // through FUEL's own tests; the rays and the camera see it through a Jolt mesh of the same triangles and boxes.
    // Keys registered here instead of with addStaticShape are not Jolt contacts for the vehicles.
    bool hasStaticCollision(uint32_t key) const;
    void addStaticCollision(uint32_t key, std::shared_ptr<const StaticCollision> collision);
    // Replaces the set of placed static objects: (shape key, world matrix).
    void setStaticObjects(const std::vector<std::pair<uint32_t, std::array<double, 16>>>& objects);

    // Movable props: small objects that sit asleep until hit, then tumble. A
    // shape is registered once per mesh (a convex hull of its points; mass
    // from its volume at ~150 kg/m^3, clamped to 5..400 kg). Props collide
    // with the vehicle, terrain, static objects and each other.
    bool hasPropShape(uint32_t key) const;
    void addPropShape(uint32_t key, const std::vector<float>& positions);
    // Adds prop `id` at a row-vector world matrix (rotation and per-axis scale); false when it has no shape.
    bool addProp(uint32_t id, uint32_t shapeKey, const std::array<double, 16>& world);
    // P55: FUEL's dynamic objects (IT_DYNOBJ_G, core/dyn_object.hpp): the body is the object's collision spheres and
    // boxes (a hull when it has neither), `fuelMass` kg (0x531560; 0 = the hull estimate above) and FUEL's drag
    // (0x5321e0) instead of damping.
    void addPropShape(uint32_t key, const StaticCollision& collision);
    // P61: debris = IT_DebrisMove (vt 0x9aef2c): its own quadratic drag 0x533790 (k +0x564 x r^2 (+0x40c) x 0.01 x
    // min(|v|, 10)^2 against v) and force volumes capped at 100 x mass (vf129 0x533b50 -> 0x710f20).
    bool addProp(uint32_t id, uint32_t shapeKey, const std::array<double, 16>& world, float fuelMass, bool debris = false);
    bool propVelocity(uint32_t id, std::array<float, 3>& velocity) const;  // world, m/s; false when absent
    void wakeProp(uint32_t id, const std::array<float, 3>& velocity);
    void wakeProp(uint32_t id, const std::array<float, 3>& velocity, const std::array<float, 3>& angular);  // P85
    // P87: the prop touches nothing (IT_DebrisMove +0xeb2 = 1, set by its reset 0x533bd0 / init 0x533c40 and cleared
    // only by the tornado spawner 0x5bd590 and 0x5cce43: such a body joins no vehicle's island (0x683400), forms no
    // body-body pair (0x71e1d0 needs +0xeb2 == 0 on both) and gathers no contacts (0x722ac0 skips it at 0x722b02)).
    void setPropNoContact(uint32_t id);
    // The hook for FUEL's force volumes on dynamic objects (PhysObj_ForceVolumes 0x710e30 in IT_DYNOBJMove_G's body
    // forces: wind, tornado and magnet volumes 0x681540, not decoded yet): world force (N) on a body of `mass` kg at a
    // world point; FUEL caps it at 150 x mass. Unset: none.
    using PropForceField = std::function<std::array<float, 3>(const std::array<double, 3>& position, float mass)>;
    void setPropForceField(PropForceField field);
    // P61: the tornado list's nearest-distance query (Tornado_NearestDistance 0x5bc950) the vehicles' state 2 uses.
    using TornadoQuery = std::function<float(double x, double z, float& radius)>;
    void setTornadoQuery(TornadoQuery query);
    // P61: each FUEL prop's peak approach speed since the last call (vt[141] +0xef0; m/s), its position and radius.
    struct PropImpact {
        uint32_t id = 0;
        float peak = 0;
        std::array<double, 3> pos{};
        float radius = 0;
    };
    std::vector<PropImpact> takePropImpacts();
    bool hasProp(uint32_t id) const;
    void removeProp(uint32_t id);
    // Props whose pose changed since the last call (mesh-origin pose; scale is not included).
    std::vector<PropPose> changedProps();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace fuel
