// The rider's animation: FUEL's per-frame rider state machine Rider_AnimUpdate 0x5d46b0 with its 3-layer
// mixer 0x5d3b40 and blend 0x7bff70, the sway spring 0x5d7e00 (P632), head levelling 0x5d5800, the
// player's IK toward the vehicle's IK_LEFTHAND/IK_RIGHTHAND/IK_LEFTFOOT/IK_RIGHTFOOT points (0x604630),
// and the ejection on a wreck (P655 > 0.5). Decoded in ~/re-work/fuel/notes/p25-camera-rider.md Part R;
// port notes and approximations in p27-camera-rider-port.md.
//
// Rider space = vehicle model space (p9 S13). Animations come from the player's PILOT.DPC (rider.hpp),
// the slot table and action properties are replayed from the player's FUEL.exe (rider_tables.hpp).
#pragma once

#include "rider.hpp"
#include "rider_tables.hpp"

#include <array>
#include <functional>
#include <vector>

namespace fuel {

// What the rider reads from its vehicle each frame (veh/phys fields of R3/R4).
struct RiderDrive {
    float dt = 0;
    std::array<float, 4> q{0, 0, 0, 1};       // vehicle body quaternion (veh+0x4c0), xyzw
    std::array<double, 3> pos{};              // model origin, world
    std::array<float, 3> vel{};               // veh+0x4d0, world m/s
    float steer = 0;                          // stick X, right + (veh+0x19f0 = -steer)
    float stickZ = 0;                         // fore/aft (veh+0x19f4)
    float airtime = 0;                        // veh+0x19ec
    float heightAbove = 0;                    // phys+0x13b8
    int gear = 1;                             // phys+0x1270
    bool sliding = false;                     // 0x52f100
    float bump = 0;                           // |phys+0xf3c .xz| (impact accumulator)
    float wheelBumps = 0;                     // mean ground-block value veh+0x1708 (per wheel +0x60)
    bool accel = false, brake = false;        // phys+0x109c / +0x10b0 > 0
    bool player = true;                       // veh+0x1a48
    bool ejected = false;                     // pm+0x37c (P655 > 0.5 on a wreck)
    int raceState = 0;                        // race-state +0x48: 3/4 start, 6/7 finished (0 otherwise)
    std::function<float(double x, double z)> ground;  // terrain height (the ejected rider's fall)
};

// One vehicle's rider. Construct per vehicle; update() every frame; boneWorlds() for skinning.
class RiderAnimator {
public:
    // `archetype` = AddVehicleInfo archetype name; `vclass` = VehicleInfo+0x38 class (BUGGY 0, BIKE 1, MUSCLE 2,
    // QUAD 3, SUV 4, TRUCK 5); `P` = VehiclePP block; `ik` = vehicle-space IK points (left hand, right hand, left
    // foot, right foot), NaN x when the vehicle has none.
    RiderAnimator(const Rider& rider, const RiderTables& tables, const std::string& archetype, int vclass, const std::vector<float>& P,
                  const std::array<std::array<float, 3>, 4>& ik);
    bool valid() const { return arch_ >= 0; }
    void update(const RiderDrive& d);
    // Bone matrices (row-vector) in vehicle model space; while ejected, in world space relative to worldOrigin().
    const std::vector<Mat4>& boneWorlds() const { return world_; }
    bool detached() const { return ejected_; }
    std::array<double, 3> worldOrigin() const { return fallOrigin_; }
    // Debug / checks.
    int layerAction(int l) const { return layers_[l].action; }
    float layerWeight(int l) const { return layers_[l].weight; }
    int oneShot() const { return oneShot_; }
    int stance() const { return stance_; }
    // 0x5d7d40: the sway target, R_body (0, min(0.005 |v|, 0.05), 0).
    static std::array<float, 3> swayTarget(const std::array<float, 4>& q, const std::array<float, 3>& vel);

private:
    struct Layer {
        int action = -1, slot = -1;
        double time = 0;
        float weight = 0;
        bool oneShot = false;
    };
    struct Request {
        int act = -1;
        float w = 0, rate = 3;
    };
    void reset();
    void push(Request* req, int act, float w, float rate);
    void startOneShot(int base, int count, bool allowRepeat);
    float mixer(Request a, Request b, float dt, bool oneShotRunning);
    void startLayer(int l, int act, float w0, bool oneShot);
    void sway(const RiderDrive& d);
    void pose(const RiderDrive& d);
    void headLevel(const RiderDrive& d, std::vector<Rider::BoneTRS>& local, bool level, float rate);
    void ik(std::vector<Rider::BoneTRS>& local, float dt);
    void fall(const RiderDrive& d);
    float rand01();

    const Rider& rider_;
    const RiderTables& tables_;
    int arch_ = -1, vclass_ = 0;
    std::vector<float> P_;
    Layer layers_[3];
    int oneShot_ = 47, lastOneShot_ = 47;   // +0x244, +0x240
    float oneShotLeft_ = -1;               // +0x24c
    float stanceT_ = 0;                    // +0x250
    int stance_ = 4;                       // +0x258
    bool bigJump_ = false, landing_ = false;  // +0x254, +0x255
    bool levelHead_ = false;
    std::array<double, 4> headQ_{0, 0, 0, 1};  // veh+0x260
    bool headInit_ = false;
    // sway spring (R7)
    std::array<float, 3> swayW_{}, swayOff_{}, swayOut_{}, prevVel_{};
    bool haveVel_ = false;
    // IK
    std::array<std::array<float, 3>, 4> ikTarget_;
    float ikT_ = 0;
    std::array<std::vector<std::array<double, 4>>, 4> ikCache_;
    int head_ = -1, hips_ = -1;
    std::array<std::array<int, 3>, 4> chain_{};  // {end, mid, root} per limb: LH, RH, LF, RF
    // output
    std::vector<Mat4> world_;
    // ejection (canned fall; see the port notes)
    bool ejected_ = false;
    float fallT_ = 0;
    std::array<double, 3> fallOrigin_{}, fallPos_{};
    std::array<float, 3> fallVel_{}, fallSpin_{};
    std::array<float, 4> fallQ_{0, 0, 0, 1};
    std::vector<Mat4> fallStart_;
    uint32_t rng_ = 12345;
};

}  // namespace fuel
