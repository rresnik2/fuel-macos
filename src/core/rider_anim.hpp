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

// PK31: the AI racer's lean (IA racer +0x1128..+0x113c, quat +0x119c). 0x61c1b0 runs from IA_GhostOrient 0x61c4b0
// for a racer that is NOT simulated (pm+0x1a4f off; a simulated one gets the reset 0x61c150): three damped springs
// 0x61c0c0 on the racer's acceleration +0x111c (0x619330: second difference of the vehicle position) give a pitch
// about the ghost's right axis, a roll about its forward axis and a yaw about its up axis (degrees). The lean is
// applied after the ghost orientation in the vehicle's placement quaternion (IACarVisibleObject vf04 0x4f07d0:
// Quat_Mul(orient, lean) = lean x orient, Hamilton), so a kinematic bike leans into its turns.
struct AiLean {
    std::array<float, 3> x{}, v{};       // per spring (deg, deg/s): pitch +0x1128/+0x112c, roll +0x1130/+0x1134, yaw +0x1138/+0x113c
    std::array<float, 4> q{0, 0, 0, 1};  // +0x119c (x, y, z, w)
    void reset() { *this = AiLean{}; }   // 0x61c150
    // 0x61c1b0: `orient` = the ghost orientation +0x118c (Hamilton, rotate = q v q*), `acc` world m/s^2, `bike` =
    // VehicleInfo+0x38 == 1.
    void step(float dt, const std::array<float, 3>& acc, const std::array<float, 4>& orient, bool bike);
    // 0x61c0c0 (EAX = &v, EBX = &x): v -= (10 x + a b) dt; v = ExpSmooth(v -> 0, rate, dt); x = clamp(x + v dt, +-limit).
    static void spring(float& v, float& x, float dt, float a, float b, float rate, float limit);
    // Quat_Mul 0x410260(ECX = a, EDX = b) = b x a (Hamilton).
    static std::array<float, 4> quatMul(const std::array<float, 4>& a, const std::array<float, 4>& b);
};

// PK31: the overtake gestures. The race-record ranking pass 0x5c2820 (race manager, list = the records in the order
// the previous frame's sort 0x5c2b00 left them): every record still racing (+0xdc < 4) takes its list index as rank
// +0x178; when that changed (and was set), the neighbour it passed (index + 1 after gaining places) or that passed it
// (index - 1) gets Rider_ProximityReaction 0x5c2370 on the record: cooldown +0x17c <= 0 (counted down by dt in the
// record update 0x5c1e.., 1 s after a gesture) and the two within 25 m (xz): side s = d.x dz - d.z dx with d = the
// record's frame displacement +0xfc/+0x104 and (dx, dz) = neighbour - record; base 21 (s > 0, gained), 29 (s > 0, lost),
// 25 (s <= 0, gained), 33 (s <= 0, lost); Rider_StartOneShotVariant(base, 4, no repeat) on that racer's rider (the
// player's too: record kind +0x150 0 = the player's vehicle).
class RankReactions {
public:
    struct Rec {
        bool racing = true;          // +0xdc < 4
        float x = 0, z = 0;          // +0xf0 / +0xf8
        float dx = 0, dz = 0;        // +0xfc / +0x104 (position - previous position)
    };
    struct Gesture {
        int racer, base;
    };
    // `order`: racer ids by rank this frame (the sort 0x5c2b00); recs by racer id. Returns the gestures started.
    std::vector<Gesture> update(float dt, const std::vector<int>& order, const std::vector<Rec>& recs);
    void reset() { list_.clear(), rank_.clear(), cool_.clear(); }

private:
    std::vector<int> list_;   // the list as the previous frame's sort left it
    std::vector<int> rank_;   // +0x178 per racer (-1 unset)
    std::vector<float> cool_; // +0x17c per racer
};

// PK31: FUEL's rider IK solver IK_SolveJacobian 0x601220 (p98/b009). Up to 3 joints (IK_AddJoint 0x6011b0); the joint
// rotates its node about `axis` given in the node's own frame (FUEL passes the same node twice: the axis is turned to
// world by that node's world quaternion +0x60). Per pass k = -2..12: e = target - effector; J column j = world axis x
// (effector - joint); d = (J^T J)^-1 J^T e (0x412580 / inverse 0x601b20; J^T e when singular); locked joints 0;
// theta = atan(d); 0x601a50 scales theta so max |theta_j| <= 0.25 / max(k, 1); q_local = q_local x (axis sin(theta/2),
// cos(theta/2)); a joint with limits whose dot(q_local.xyz, axis) leaves [lo, hi] is locked and instead set to
// q_old x (axis sin(c/2), cos(c/2)), c = limit - dot; normalise; FK; stop once |e|^2 < 2.5e-5 (5 mm). Returns AL:
// false when already within 5 mm on entry, else true (reached or not after 15 passes). Quaternions are Hamilton.
struct IkJoint {
    std::array<double, 3> axis{0, 0, 1};
    bool limited = false;
    double lo = -10, hi = 10;
};
struct IkAccess {
    std::function<std::array<double, 4>&(int j)> local;      // node j's local quaternion (written)
    std::function<std::array<double, 4>(int j)> worldQ;      // node j's world quaternion (+0x60)
    std::function<std::array<double, 3>(int j)> jointPos;    // node j's world position (+0x4c +0x30)
    std::function<std::array<double, 3>()> effector;         // the effector node's world position
    std::function<void()> fk;                                // 0x66bc40(0)
};
bool ikSolveJacobian(const IkJoint* joints, int count, const IkAccess& io, const std::array<double, 3>& target, int* passes = nullptr,
                     bool* locked = nullptr);

// One vehicle's rider. Construct per vehicle; update() every frame; boneWorlds() for skinning.
class RiderAnimator {
public:
    // `archetype` = AddVehicleInfo archetype name; `vclass` = VehicleInfo+0x38 class (BUGGY 0, BIKE 1, MUSCLE 2,
    // QUAD 3, SUV 4, TRUCK 5); `P` = VehiclePP block; `ik` = vehicle-space IK points (left hand, right hand, left
    // foot, right foot), NaN x when the vehicle has none; `seat` = where FUEL puts the pilot object in vehicle
    // space (riderSeatPoint: the vehicle skeleton's Head bone, node 61 of 0x5ca410), added to the skeleton root.
    RiderAnimator(const Rider& rider, const RiderTables& tables, const std::string& archetype, int vclass, const std::vector<float>& P,
                  const std::array<std::array<float, 3>, 4>& ik, const std::array<double, 3>& seat = {});
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
    // PK31: Rider_StartOneShotVariant(base, 4, no repeat) from Rider_ProximityReaction 0x5c2370 (an overtake).
    void proximityReaction(int base) {
        if (arch_ >= 0) startOneShot(base, 4, false);
    }
    // PK14: KillMission 0x62baa0 sets the player's +0x154 object +0x244 and +0x240 to 0x2f + floor(Rand01 x 0.9999) =
    // 47 (no one-shot; the timer +0x24c is left as it is).
    void killMission() { oneShot_ = lastOneShot_ = 47; }
    // P76: limb l (0 left hand, 1 right hand, 2 left foot, 3 right foot) let go by the animation (event 0x10 on
    // LeftArm / RightArm / LeftUpLeg / RightUpLeg sets node flag 0x400 on its subtree, 0x11 clears it).
    bool limbFree(int l) const { return limbOff_[size_t(l)]; }
    // PK31: the hands' IK box orientations (vehicle space, row-vector rotation; NaN [0] = none): 0x604410 turns the
    // hand node so its world orientation is the box's (left hand, right hand).
    void setHandFrames(const std::array<std::array<double, 9>, 2>& f) { handFrame_ = f; }
    // PK31 debug: the passes of the last IK solve per limb (0: within 5 mm already, -1: not run).
    int ikResult(int l) const { return ikResult_[size_t(l)]; }
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
    void applyEvents(int slot, double from, double to, bool closed);
    void fall(const RiderDrive& d);
    float rand01();

    const Rider& rider_;
    const RiderTables& tables_;
    int arch_ = -1, vclass_ = 0;
    std::vector<float> P_;
    std::array<double, 3> seat_{};          // 0x5ca410: the vehicle's node 61 (Head bone), vehicle space
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
    std::array<bool, 4> limbOff_{};       // P76: node flag 0x400 on the limb (0x7c0480 / 0x64f3f0)
    std::array<float, 4> limbT_{};        // P76: time since the limb was switched
    // PK31: per limb (0x6032f0 / 0x603b80 / 0x602330 / 0x602ad0): the cached end / mid / root local quaternions
    // (hand +0x3a4.., +0x3b4.., +0x3c4..), the cached effector position (+0x398), "was let go" (+0x394, 1 at the
    // ctor 0x604550), the blend-out flag (+0x3d4) and its timer (+0x3d8).
    struct LimbIk {
        std::array<std::array<double, 4>, 3> q{};
        std::array<double, 3> pos{};
        bool wasOff = true, blendOut = false;
        float blendT = 0;
    };
    std::array<LimbIk, 4> limbIk_{};
    std::array<std::array<double, 9>, 2> handFrame_{};
    std::array<int, 4> ikResult_{-1, -1, -1, -1};
    std::array<uint32_t, 4> limbBone_{};  // name CRCs of LeftArm, RightArm, LeftUpLeg, RightUpLeg
    int head_ = -1, hips_ = -1;
    std::array<std::array<int, 3>, 4> chain_{};  // {end, mid, root} per limb: LH, RH, LF, RF
    // output
    std::vector<Mat4> world_;
    // ejection: FUEL's 10-body ragdoll (0x605650; P46), bodies at the rider's nodes, positions relative to fallOrigin_
    bool ejected_ = false;
    float fallT_ = 0;
    std::array<double, 3> fallOrigin_{};
    std::array<float, 4> fallQ_{0, 0, 0, 1};  // the vehicle's rotation at the ejection (model -> world)
    std::vector<Mat4> fallStart_;
    struct RagBody {
        int bone = -1;
        std::array<double, 3> x{}, prev{}, v{};
        double invMass = 0;
    };
    struct RagLink {
        int a = 0, b = 0;
        double rest = 0;
    };
    std::array<RagBody, 10> rag_{};
    std::vector<RagLink> ragLinks_;
    bool ragOk_ = false;
    bool ragSettled_ = false;  // PK31: the placement settle 0x71b2f0 done (+0x5ac)
    void ragStart(const RiderDrive& d);
    void ragStep(const RiderDrive& d);
    uint32_t rng_ = 12345;
};

}  // namespace fuel
