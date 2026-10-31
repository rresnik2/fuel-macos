// FUEL's in-race player camera: CameraMove_G (vt 0x9ae4ec) per-frame update 0x412e80 with the vehicle
// camera 0x40a160, decoded in ~/re-work/fuel/notes/p25-camera-rider.md (Parts A, B, P); port notes and
// deviations in p27-camera-rider-port.md. Views: 5 near chase (default) -> 1 first person -> 4 far chase,
// overrides 8 race intro, 9 finish, 10/11 wreck/drown orbit, 13 page camera.
//
// All maths runs in world axes relative to the vehicle model origin, as the engine does; the origin is
// added at the end (A5 step 18). Parameters come from the player's VehiclesParam.tsc (camera_params.hpp).
#pragma once

#include "fuel_math.hpp"

#include <cstdint>
#include <functional>
#include <vector>

namespace fuel::cam {

using fv::Quat;
using fv::Vec3;

// ---- primitives (A11, emulator-matched) ----
// Smooth1 0x4100b0: |d| < 1e-6 -> target, else cur + d (1 - rate^(-30.30303 dt)).
float smooth1(float cur, float target, float rate, float dt);
// SmoothV 0x40fb70: per component, snap when |d|^2 < 1e-6.
Vec3 smoothV(const Vec3& cur, const Vec3& target, float rate, float dt);
// Quat_FromTo 0x6443a0: shortest rotation taking unit a to unit b.
Quat quatFromTo(const Vec3& a, const Vec3& b);
// Vec_RemoveAgainst 0x402080: if step . n(N) < 0 the component along n(N) is removed (N any length, 0 = none).
Vec3 removeAgainst(const Vec3& step, const Vec3& N);
// 0x40fc80: (x, 0, z) / |xz|; `fallback` when |xz|^2 <= 1e-6.
Vec3 flatNormalize(const Vec3& v, const Vec3& fallback);

// What the camera reads from the player vehicle each frame (phys = IT_Dyn_VehicleMove_G).
struct VehicleInput {
    bool valid = false;
    Quat q;                        // body orientation (ctx+0 GetRotQuat)
    double ox = 0, oy = 0, oz = 0; // model origin = body pos - rotated CoM (ctx+0xfc), world
    Vec3 vel;                      // phys+0x524 averaged velocity
    Vec3 angVel;                   // phys+0x534
    Vec3 fdVel;                    // phys+0x4a8 finite-difference velocity
    int gear = 1;                  // phys+0x1270
    bool sliding = false;          // 0x52f100 VehMove_IsSlidingMajority
    float airtime = 0;             // phys+0x1258
    float heightAbove = 0;         // phys+0x13b8
    int wreck = 0;                 // phys+0x1420: 0 ok, 2 drowned, other wrecked
    bool anyWheelLoaded = true;    // 0x52e620
    float maxBumps = 0;            // 0x526a30 (BUMPS WHEEL of the ground under the wheels)
    float speedFrac = 0;           // 0x423380
    bool boost = false;            // player byte +0x2f61 (the port has no boost latch: false)
    Vec3 targetNode;               // node #15 of the vehicle hierarchy, model space (U-C1; the port uses the origin)
    const std::vector<float>* P = nullptr;  // VehiclePP block (index = parameter number)
};

// World queries (C3, C1).
struct WorldQueries {
    // Cam_GroundHeight 0x402620 at a world point: terrain (`detail`: the full AGW_Altitude height, else the base
    // map), water surface, then a 100 m ray down against static objects. Returns the height (world y).
    std::function<float(double x, double y, double z, bool detail)> ground;
    // 0x63a0b0 sphere overlap at world point c: contacts {point on the object (world, relative to c), unit normal
    // out of the object, distance |p - c|} for objects bigger than 1 m^3 (0x4063a0).
    struct Contact {
        Vec3 p;  // relative to the query centre
        Vec3 n;
        float d = 0;
    };
    std::function<void(double x, double y, double z, float r, std::vector<Contact>& out)> overlap;
};

struct CameraPose {
    double ex = 0, ey = 0, ez = 0;  // eye (world)
    double tx = 0, ty = 0, tz = 0;  // target (world)
    float fovDeg = 60;  // horizontal FOV of a 4:3 frame (Hor+ for wider screens), degrees
    float roll = 0;     // radians (ctx+0x8c)
    float zNear = 0.25f, zFar = 50000;
    float blur = 0;     // speed blur strength (viewport +0x1dc)
    int view = 5;
};

// Player input for the camera: look stick (cam+0x334/+0x338) and the CAMERA button edge (+0x33c).
struct CameraControls {
    float stickX = 0, stickY = 0;
    bool viewChange = false;
};

// Overrides chosen by 0x402930 (A3, B E): race state / wreck / page camera.
enum class Override { None = 0, Intro = 8, Finish = 9, Wreck = 10, Drown = 11, Page = 13 };

class ChaseCamera {
public:
    ChaseCamera();
    // One frame (render dt). `ov`: the override the game state asks for (wreck/drown are also taken from the vehicle).
    CameraPose update(float dt, const VehicleInput& veh, const WorldQueries& world, const CameraControls& in, Override ov = Override::None);
    void snapNext() { snap_ = true; }   // reset/teleport (cam+0x138)
    int view() const { return cur_; }
    int playerView() const { return choice_; }
    void setPlayerView(int v) { choice_ = v, snap_ = true; }
    // Absolute camera for view 13 (hub/menu page camera 0x40e4e0): world pos/target.
    void setPageCamera(double px, double py, double pz, double tx, double ty, double tz) {
        page_[0] = px, page_[1] = py, page_[2] = pz, page_[3] = tx, page_[4] = ty, page_[5] = tz;
    }

    // ---- the pieces, exposed for --camcheck (state is this camera's) ----
    struct Ctx;  // the per-frame context (A4)
    static Vec3 liftOffset(const Vec3& up, const Vec3& flatSide, const Vec3& F, float ctx3c, float ctx40, float scale = 1.5f);  // 0x404040
    static Vec3 snapOffset(const Vec3& F, float pitch, float D);  // 0x409790
    // 0x404c90 manual orbit; returns the new O.
    Vec3 orbit(const Vec3& O, float stickX, float stickY, float speed, float ctx30, float dt, bool snap);
    // 0x405440 follow spring: Dd desired direction, stiff = ctx+0x4c.
    Vec3 followSpring(const Vec3& O, const Vec3& Dd, float stiff, float dt);
    // 0x4091e0 lag.
    Vec3 lag(const Vec3& O, const Vec3& vel, float pitch, const Vec3& side, float dt, bool snap, float ctx48 = 0.33f);
    // 0x405800 distance.
    Vec3 distance(const Vec3& O, float D, float speed, bool snap);
    // 0x408d30 first-person head spring: moves eye/target, adds to roll.
    void headSpring(const Vec3& U, const Vec3& F, Vec3& eye, Vec3& target, float dt, bool snap, float& roll, float k10);
    // state access for the checks
    float& yaw() { return yaw_; }
    float& pitch() { return pitch_; }
    float& gain() { return g_; }
    float& gain2() { return g2_; }
    float& autoFollow() { return af18e8_; }
    float& autoFollowTarget() { return af18f0_; }
    float& pauseTimer() { return pause18dc_; }
    bool& lock() { return lock_; }
    bool& flag197d() { return f197d_; }
    Vec3& prevO() { return prevO_; }
    struct HeadState {
        Vec3 prev, p, v;
    };
    const HeadState& head() const { return head_; }

private:
    void buildContext(Ctx& c, float dt, const VehicleInput& veh, const WorldQueries& world, const CameraControls& in, Override ov);
    Vec3 lookTarget(const Ctx& c, float dt);
    Vec3 offset(Ctx& c, const Vec3& T, float dt, const WorldQueries& world);
    void sway(Ctx& c, const Vec3& O, Vec3& T, float dt);
    Vec3 pushOut(Ctx& c, const Vec3& B, const Vec3& O, float dt, const WorldQueries& world);
    void roughShake(const Ctx& c, Vec3& pos, Vec3& target, float dt);
    float rand01();

    // view selection (A3)
    int choice_ = 5;     // cam+0x1990
    int cur_ = 5;        // cam+0x1988
    int prevView_ = -1;  // cam+0x198c
    int override_ = 0;   // cam+0x1994
    float tView_ = 0;    // cam+0x1984
    bool snap_ = true;   // cam+0x220
    bool teleport_ = false;  // cam+0x1983 (I: snap on the next frame)
    int forward_ = 1;    // cam+0x18b4 (1 normal, 0 reversed)
    Vec3 prevFlatF_{0, 0, 1};  // cam+0x18a8
    double lastOx_ = 0, lastOy_ = 0, lastOz_ = 0;
    bool haveLast_ = false;

    // look target
    Vec3 liftS_;  // cam+0x1834
    // offset
    Vec3 O_{0, 1, -3};       // cam+0x1828
    Vec3 prevO_;             // cam+0x1870
    Vec3 N_;                 // cam+0x1864 contact normal
    float yaw_ = 0, pitch_ = 0;  // cam+0x1904/+0x1908
    float g_ = 1, g2_ = 1, af18e8_ = 1, af18f0_ = 1, af18f4_ = 1, af18f8_ = 1, af18fc_ = 1;  // +0x18ec, +0x18e4, ...
    float pause18dc_ = 0, s1940_ = 0;
    float timer1930_[4] = {};  // +0x1930 ceiling, +0x1934 ground, +0x1938, +0x193c
    bool lock_ = false, f197d_ = false, f197f_ = false, f1982_ = false, pushed_ = false;
    float minElev_ = -1.0f;    // +0x194c
    float orbitStickX_ = 0, orbitStickY_ = 0;  // cam+0x334/+0x338
    // push-out (C1)
    Vec3 Q_;                   // cam+0x1894 (relative to the origin)
    float probeT_ = 100;       // cam+0x18c0
    bool f1981_ = false;
    Vec3 accN_;                // cam+0x187c
    // final pose
    Vec3 rel_, pos_, tgt_;     // +0x174, +0x168, +0x198
    float fov_ = 60;           // +0x1914 (deg)
    float groundG_ = 1.0f;     // global 0xa26b20
    // landing dip (A9)
    bool air_ = false, dip_ = false;
    float maxH_ = 0, dipA_ = 0, dipD_ = 0;
    // sway (B A): s195c, s1960, s1964, s1968, s196c, s1970
    float s195c_ = 0, s1960_ = 0, s1964_ = 0, s1968_ = 0, s196c_ = 0, s1970_ = 0;
    int signToggle_ = 1;       // cam+0x34c
    // rough shake (B B)
    float shakeT_ = 0.2f;
    Vec3 shakeV_, shakeOff_;
    // first-person head spring
    HeadState head_;
    // wreck orbit (B E): time and saved world points
    float wreckT_[2] = {0, 0};
    double wreckPos_[2][3] = {};
    double page_[6] = {};
    uint32_t rng_[33] = {};    // Rand01 0x6460f0 (Park-Miller + Bays-Durham shuffle)
    bool rngInit_ = false;
};

}  // namespace fuel::cam
