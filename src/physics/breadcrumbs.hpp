// FUEL's respawn breadcrumbs (IT_Dyn_VehicleMove_G, ported 1:1 by P70b from FUEL.exe; notes
// p70-port-decoded.md "Breadcrumbs", p23-vehicle-physics.md section 7):
//   0x528830 VehMove_BreadcrumbTick   while the list is empty, record every frame; else every 1 s (+0x13a0)
//   0x528780 _Record3                 record at the body position and 5 m to either side (body x axis)
//   0x5283b0 _RecordAt                one crumb: >= 10 m from the newest, safe, list dropped past 1000 m;
//                                     orientation toward the race path / GPS target / along the road
//   0x5281c0 VehMove_IsSafeForBreadcrumb
//   0x5288d0 VehMove_PickBreadcrumb   (+ 0x527af0: crumbs newer than the chosen one are dropped)
// The list (veh+0x220 pool: tail +0x224, head +0x226, 0x24-byte records {pos, quat, flag +0x1c, prev +0x20,
// next +0x22}) is a vector here, oldest first. FUEL keeps a quaternion made by Quat_FromArc((0,0,1), dir)
// (0x6443a0) from a horizontal dir: a pure yaw, kept as yaw = atan2(dir.x, dir.z) (identity when |dir| < 1e-6).
#pragma once

#include <functional>
#include <vector>

namespace fuel::fv {

// The vehicle state the breadcrumb code reads (world coordinates).
struct CrumbVehicle {
    double pos[3] = {0, 0, 0};  // +0x800 body position
    float lat[3] = {1, 0, 0};   // body x axis (0x410510: quat +0x7f0 applied to (1, 0, 0) [0xa06338])
    float fwd[3] = {0, 0, 1};   // +0x504 body z axis
    float yaw = 0;              // current heading (Pick's fallback keeps the current quaternion)
    float upY = 1;              // +0x4e8 body y axis . y
    float velY = 0;             // +0x824
    float probeNormalY = 1;     // +0x1034 ground probe normal (0x5298d0)
    float maxWaterDepth = 0;    // +0x1380
    bool allWheelsLoaded = true;  // 0x52e6d0
    float radius = 1;           // +0x404 extent about the centre of mass (0x712b60)
};

// The world queries (each may be empty: then the FUEL branch that needs it is not taken).
struct CrumbWorld {
    // 0x527f50 -> 0x5309e0 -> 0x6fd3a0: nearest centreline point of the road segments covering p (FUEL's
    // TerraRoad, 0x6fb6f0) and that segment's direction B - A (unnormalised, y = 0); false when none covers p.
    std::function<bool(double x, double z, double& px, double& pz, float dir[3])> roadCentreline;
    // 0x528690: the race's path target (0x628bc0: IAPath_Sample(30, 30) from the player's path cursor, the
    // second sample, else the first). Note FUEL uses the player's race path for every vehicle's crumbs.
    std::function<bool(double& x, double& z)> raceTarget;
    // 0x421dc0: the player's GPS target (MapHUD +0x60/+0x64/+0x6c; race +0xec/+0xf0 when it has a path).
    std::function<bool(double& x, double& z)> gpsTarget;
    // 0x527bf0: an object (scene query masks 0x202 / 0x10fc1, not flagged +0x60 0x4000000) overlaps the sphere.
    std::function<bool(const double p[3], float radius)> blocked;
    std::function<float(double x, double z)> waterDepth;  // 0x40fd10 -> 0x65a480
    std::function<float(double x, double z)> baseHeight;  // 0x445500 -> 0x445570 (coarse height)
    bool tutorialBounds = false;                           // game +0x32bc == 2 (World_IsOutside 0x5280a0)
};

class Breadcrumbs {
public:
    struct Crumb {
        double x = 0, y = 0, z = 0;
        float yaw = 0;
        bool onRoad = false;  // +0x1c: 0x527f50 held (on a road, within 20 m of its centreline)
    };

    void clear() { list_.clear(); }
    const std::vector<Crumb>& list() const { return list_; }
    std::vector<Crumb>& list() { return list_; }
    float timer() const { return timer_; }
    void setTimer(float t) { timer_ = t; }

    void tick(float dt, const CrumbVehicle& v, const CrumbWorld& w);    // 0x528830
    void record3(const CrumbVehicle& v, const CrumbWorld& w);           // 0x528780
    void recordAt(const double p[3], const CrumbVehicle& v, const CrumbWorld& w);  // 0x5283b0
    // 0x5288d0(minDist): the crumb to respawn at (and the list cut after it); false = respawn in place
    // (FUEL's out flag 1: the current position and orientation).
    bool pick(float minDist, const CrumbVehicle& v, Crumb& out);

    static bool worldOutside(double x, double z, bool tutorial);       // 0x5280a0
    static bool safe(const double p[3], const CrumbVehicle& v, const CrumbWorld& w);  // 0x5281c0
    // 0x527f50: on a road (nearest covering centreline point < 20 m [0x996154]); dir = that segment's B - A.
    static bool onRoad(const double p[3], const CrumbWorld& w, float dir[3]);
    static float yawOf(float x, float z);  // Quat_FromArc((0,0,1), (x, 0, z)) as a yaw

private:
    std::vector<Crumb> list_;
    float timer_ = 0;  // +0x13a0
};

// PK01: the respawn orientation of the pre-start hold (_CheckpointRespawn 0x5279c0 -> 0x529480
// VehMove_RespawnOrientFromWater, then VehMove_ResetTo 0x5273b0 without the ground snap). Unless the world's
// flag (game +0x31fc -> +0x908 -> +0x5c) is set, a target `posY` less than 1 m from the terrain height
// (0x40fe70) is turned upright on the terrain normal `n` (0x640690(1.0)) keeping q's flattened heading:
// Quat_Mul 0x410260(Quat_FromArc((0,1,0), n), Quat_FromArc((0,0,1), flat(q (0,0,1)))) = tilt * yaw.
// Otherwise q is kept. Quaternions (x, y, z, w). Checked by fuelphysvec "RO*" (EmuRun of 0x529480).
void respawnOrient(float q[4], float posY, float terrainY, const float n[3], bool worldFlag = false);
// Quat_FromArc 0x6443a0: the shortest rotation taking `from` to `to` (identity when either is shorter than 1e-6).
void quatFromArc(const float from[3], const float to[3], float out[4]);

}  // namespace fuel::fv
