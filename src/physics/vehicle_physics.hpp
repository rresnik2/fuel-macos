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
};

struct VehicleState {
    Pose chassis;
    std::vector<Pose> wheels;  // wheel model space: x = axle, y = up
    float speedKmh = 0;
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

    // Surface type (0..39) under a world point, for per-wheel grip. Without a query every
    // surface counts as dirt. The default applies to worlds created afterwards.
    void setSurfaceQuery(SurfaceQuery query);
    static void setDefaultSurfaceQuery(SurfaceQuery query);
    // Water depth (metres above the ground, 0 = dry) at a world point, for FUEL's water floor contacts,
    // water drag and wet grip. Defaults to 0 everywhere (the world's water plugs in here).
    using WaterDepthQuery = std::function<float(double x, double z)>;
    void setWaterDepthQuery(WaterDepthQuery query);
    static void setDefaultWaterDepthQuery(WaterDepthQuery query);
    float waterDepthAt(double x, double z) const;

    // Physics cost of the last update (milliseconds, wall clock) and the vehicle model's own share.
    float lastStepMs() const;
    float lastVehicleMs() const;
    // FUEL's model of vehicle `i` (null when absent), for checks and telemetry.
    const fv::FuelVehicle* model(int i) const;

    // Static world objects the vehicle collides with. A shape is registered
    // once per mesh; instances place it with a row-vector world matrix
    // (translation in 12..14; rotation and uniform or axis scale).
    bool hasStaticShape(uint32_t key) const;
    void addStaticShape(uint32_t key, const std::vector<float>& positions, const std::vector<uint32_t>& indices);
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
    bool hasProp(uint32_t id) const;
    void removeProp(uint32_t id);
    // Props whose pose changed since the last call (mesh-origin pose; scale is not included).
    std::vector<PropPose> changedProps();

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace fuel
