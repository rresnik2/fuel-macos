// Vehicle physics on the world terrain, built on Jolt Physics.
//
// Jolt's wheeled-vehicle constraint does the suspension, tyres, engine,
// gearbox and differentials; it is tuned from FUEL's own vehicle data. On top
// of it the port adds what an arcade off-roader needs: air control, speed-
// sensitive steering, self-righting, landing stabilisation and off-road tyre
// curves. The terrain collides as a heightfield patch that follows the vehicle.
#pragma once

#include "core/dpc.hpp"
#include "core/surface.hpp"
#include "core/terrain.hpp"

#include <array>
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
};

// Builds a setup from the variant's VehiclesParam.tsc values and its physics
// proxy in the vehicle DPC. `hubs`, when given, are the assembled wheel
// centres (they can sit inboard of the proxy spheres); each proxy wheel takes
// the nearest one. Throws when the data is missing or implausible.
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

    // Places the vehicle on the terrain at (x, z), facing `yaw` radians.
    void spawn(const VehicleSetup& setup, double x, double z, float yaw);
    // Puts the vehicle back on its wheels where it is.
    void reset();
    // Advances by `dt` seconds in fixed 1/60 s steps.
    void update(float dt, const DriveInput& input);
    VehicleState state() const;

    // Surface type (0..39) under a world point, for per-wheel grip. Without a query every
    // surface counts as dirt. The default applies to worlds created afterwards.
    void setSurfaceQuery(SurfaceQuery query);
    static void setDefaultSurfaceQuery(SurfaceQuery query);

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
