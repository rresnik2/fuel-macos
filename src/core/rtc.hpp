// FUEL's real-time cut-scenes (RTC, P47; decode notes/p47-work/rtc.md, summary notes/p47-input-anim.md §3):
// the Rtc_Z body (Load 0x7d6ad0: nodes, cameras, look-at targets, lights, binds, RTC-only objects, events) and
// RtcPlayer_Z's playback (Advance 0x729b30 / Evaluate 0x729de0: time, the 0xc/0xd loop markers, event windows
// 0x73b2b0, node and camera tracks with the samplers of anim_clip.hpp).
#pragma once

#include "anim_clip.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace fuel::rtc {

using anim::Mat4;

struct Msg { uint32_t type = 0, a = 0, b = 0, c = 0, d = 0; };
struct EventKey { float time = 0; std::vector<Msg> msgs; };

struct VKey { int16_t frame; uint16_t flags; float v[3], in[3], out[3]; };  // 40 B, t = frame / 60
struct SKey { float t; int16_t v, in, out; uint16_t pad; };                  // 12 B, /4096
struct FKey { float t, v, in, out; };                                        // 16 B
struct RKey { float t; int16_t q[4]; };                                      // 12 B, x 0.0005

struct Node {      // A (mem stride 0x30)
    uint32_t id = 0;
    uint16_t e = 0;  // index into binds
    std::vector<RKey> rot;
    uint16_t scaleFlags = 0, transFlags = 0;
    std::vector<VKey> scale, trans;  // scale: .x only (uniform)
    std::vector<EventKey> events;
};
struct Camera {    // B (0x38)
    uint32_t id = 0;
    uint16_t e = 0;
    uint16_t rollFlags = 0, fovFlags = 0, farFlags = 0, nearFlags = 0;
    std::vector<SKey> roll, fov, nearT;
    std::vector<FKey> farT;
};
struct Bind {      // E (25 B on disk)
    uint32_t parentId = 0, nodeId = 0, unk = 0;
    int32_t parentE = -1;  // -1: under the RTC's father
    uint8_t flag = 0;
    uint32_t nodeRes = 0, res2 = 0;
};
struct Object {    // F (24 B): an RTC-only object shown while the RTC plays
    uint32_t e = 0, nodeId = 0, unk = 0, res = 0, anim = 0, userDefine = 0;
};

struct Rtc {
    float duration = 0;
    std::vector<Node> nodes;
    std::vector<Camera> cameras;
    std::vector<uint32_t> targets;  // C[i]: the look-at node of camera i
    std::vector<Bind> binds;
    std::vector<Object> objects;
    std::vector<EventKey> events;   // RTC level
    size_t lights = 0;
};
// Parses a Rtc_Z body; false (with a reason) unless the grammar consumes it exactly.
bool parse(const std::vector<uint8_t>& body, Rtc& out, std::string* error);

// Samplers (0x7d5640 vec3, 0x72f750 scalar, 0x7d61e0 quaternion, 0x72f910 float [I]).
anim::Vec3 sampleV(const std::vector<VKey>& k, uint16_t trackFlags, double t);
double sampleS(const std::vector<SKey>& k, uint16_t trackFlags, double t);
double sampleF(const std::vector<FKey>& k, uint16_t trackFlags, double t);
anim::Quat sampleR(const std::vector<RKey>& k, double t);

struct Fired {
    float time;
    int node;  // -1 for RTC-level events
    Msg msg;
    bool skipping = false;  // fired by a seek (Evaluate(1) in 0x72aeb0): no sounds, instant fades
};

class Player {
public:
    explicit Player(const Rtc& r) : rtc_(r) {}
    void start();
    // One frame (0x729b30): t += dt; loop markers; events of [prev, t) appended to `fired`; clamps at the end.
    void advance(float dt, std::vector<Fired>& fired);
    // 0x72aeb0: a forward seek evaluates the skipped window [t, target) with skipping set (camera cuts, fades and
    // hides still apply; sounds do not), then clears the loop flags 2 | 8 and sets the time.
    void seek(float t, std::vector<Fired>* fired = nullptr);
    void releaseLoop() { flags_ |= 8; }  // stop honouring 0xd (FUEL: player flag 8)
    // P76, 0x5385a0 -> 0x72ad90 / 0x73b190: the first RTC-level key at or after the current time holding a message
    // {type 0xd, a 0} (a loop end); false when there is none.
    bool nextLoopEnd(float& at) const;
    float time() const { return time_; }
    bool finished() const { return finished_; }
    // World matrices of every bind (row-vector), roots under `father`.
    void evaluate(const Mat4& father, std::vector<Mat4>& world) const;
    struct CamPose {
        anim::Vec3 eye, target, up;
        double fov = 1, roll = 0, nearZ = 1;
    };
    // Camera `cam` (index into rtc.cameras) from the evaluated binds.
    CamPose camera(size_t cam, const std::vector<Mat4>& world) const;
    int cameraById(uint32_t id) const;

private:
    void collect(double from, double to, bool closed, std::vector<Fired>* fired, bool loopOnly);
    const Rtc& rtc_;
    double time_ = 0, loopStart_ = 0;
    int flags_ = 0, count_ = 0;
    bool finished_ = false;
};

// P76: event 0x10e (TR_DEATH, RTC level, with the respawn 0x10b) -> 0x53c740 on the scripted player: +0x6e8 = 1,
// timer +0x6e4 = 0, and the global "pause all animation" 0xa65878 = 1 (game+0xe0 |= 1). While it is set FUEL skips
// the mission update 0x633190 (race clock and state machine), PlayerMove 0x42dc40, traffic 0x5c3010 and special
// effects. 0x53c770 (RtcScripted_G::Update 0x53c150, after Advance): timer += dt, held at 0 while the streamer is busy;
// when timer > 0.5 (0x9960c0, strict): released, and the player seeks to the next loop end + 0.01 (0x5385a0).
struct PauseHold {
    bool holding = false;
    float timer = 0;
    void start() {
        if (!holding) holding = true, timer = 0;
    }
    // Returns true on the frame it lets go (after seeking `p`).
    bool update(float dt, Player& p, std::vector<Fired>* fired, bool streamingBusy = false);
};

// P76: event 0x10f (node level, 0x5389d0, asm 0x538a30-0x538aa2): when the weather's day factor 0x5b98d0 is >= 0.8
// (0x9dc154) the node is hidden (flags |= 0x40000, children too); otherwise nothing. TELEPORT_HELICO fires it at 0 s
// on A305 (the helicopter's light mesh): shown only at dusk, night and dawn.
inline bool hidesInDaylight(float dayFactor) { return dayFactor >= 0.8f; }

// P96: FUEL's global bone-name ids (AnimationManager_Z, ctor 0x747550): 56 fixed names (TETE .. TRONCHE1 = 0..33,
// DYN1..DYN16 = 34..49, SKIN SHOES CLOTHE HAIRS MOUTH FACE = 50..55), then 0x748260 appends a name the first time it
// is registered: Skel_Z::Load 0x7b4830 registers its per-mesh name lists (Skel::palettes) when its DPC loads; a skin
// instance (SkelPc_Z vf18 0x66c0e0) registers every bone in pre-order (0x675e60) when it is built (RTC Init 0x728da0 for the
// helicopter); the instance list (+0x20 / +0x24) is indexed by these ids.
class BoneRegistry {
public:
    BoneRegistry();
    int id(uint32_t nameCrc);         // registers on first use (0x748260)
    int find(uint32_t nameCrc) const;  // -1 when not registered
    size_t size() const { return names_.size(); }

private:
    std::vector<uint32_t> names_;
};

// P96: event 0x103 'AMEO' (node level 0x5389d0, asm 0x538aa5-0x538be8; notes/p96-heli-intro.md): a = 1..4, joint =
// instance list [a + 0x49] of the event node's skin, target = the RTC bind named "<MESH_ID>_<a>" of the player's
// vehicle (else "MESH_ID_MUSCLE_OFF_2_<a>"); every frame after Advance (0x53ba20 -> 0x676480) the joint's origin is
// moved onto the target. kAmeoBase = the id of a = 1 (0x4a).
inline constexpr int kAmeoBase = 0x4a;
// The joint (bone index of `bones`) for `a`, replaying the registration: `registry` holds the names registered before
// the RTC (fixed names and the loaded skeletons' lists); the RTC's skeleton registers its lists (load) and then its
// bones (instance). FUEL's ids count resources the port never loads, so the run of ids the instance registers is
// anchored at kAmeoBase: a selects the a-th bone the instance registers. -1 when there is none.
// P100 [DEVIATION]: by default (fuelOrder false) a pairs with the bone named LINKa (four straight cables, no crossing);
// fuelOrder = true is the decoded FUEL pairing (a = 1..4 -> LINK1, Link3, Link2, Link4).
int ameoJoint(BoneRegistry registry, const std::vector<std::vector<uint32_t>>& palettes, const std::vector<uint32_t>& bones, int a, bool fuelOrder = false);

// ---- PK13 (notes ~/re-work/fuel/notes/pk13.md) ------------------------------------------------------------------
// The RTC attach slot (player +0xec records, 0x53bfc0 -> 0x514b70; per-frame step 0x5148a0 from RtcScripted_G::Update
// 0x53c150 after Advance). Node event 0xf1 attaches the player's vehicle to the event node with blend time = message
// c (float, +0xc; TELEPORT_HELICO: 0 -> it snaps); 0xf2 detaches (blend -1.0 [0xa05224]): elapsed = duration, one final
// step(0) (snap to the node), then the slot is cleared. Step: while duration (+0x20) > elapsed (+0x24) (x87 fcomp, asm
// 0x51490b-0x51491d) the pose eases from the vehicle's pose latched at elapsed == 0 (+0x28 pos, +0x34 quat) to the node:
// f = 1 - elapsed / duration, pos = node + (start - node) f, quat = slerp(node, start, f) (0x431010, EAX node, EBX start);
// else pose = the node's (pos, world quat; no scale), done (+0x44) = 1. Then elapsed += dt. The pose is written with
// SetWorldPos/Rot (plain object) or the creature teleport 0x41b6b0 - position and rotation only.
struct AttachSlot {
    bool active = false;
    float duration = 0, elapsed = 0;  // +0x20, +0x24
    anim::Vec3 start{};               // +0x28
    anim::Quat startQ{0, 0, 0, 1};    // +0x34
    bool done = false;                // +0x44
    void attach(float blend) { active = true, duration = blend, elapsed = 0, done = false; }  // 0x514b70 valid handle
    // 0x5148a0: `cur` = the attached object's pose now (latched when elapsed == 0); `node` = the target node's.
    void step(float dt, const anim::Vec3& nodeP, const anim::Quat& nodeQ, const anim::Vec3& curP, const anim::Quat& curQ, anim::Vec3& outP,
              anim::Quat& outQ);
};
// Rotation (x y z w) of a row-vector world matrix whose rows may carry a uniform scale (Node_Z world quat +0x10).
anim::Quat quatOfRows(const Mat4& m);
// A rigid row-vector matrix from a pose.
inline Mat4 poseMatrix(const anim::Vec3& p, const anim::Quat& q) { return anim::localMatrix({1, 1, 1}, q, p); }

// RTC node FX (0x5389d0 node events 0x20 / 0x22 / 0x23 -> the per-RTC cache 0x539140, released by 0x539310 at End
// 0x536230 and unload vf25): a ParticlesAgent_Z made from resource msg.d (0x730370 with class "ParticlesAgent_Z"
// [0x9b2c98]) under the RTC world, cached by (event node, msg +0x24, msg +0x28) + data id; 0x20: active (flags 0x20000,
// clear hidden 0x40000), attached to the event node, following it (0x677960: SuperSpray +0x29 = 1), (re)started
// (0x6774e0: accumulators, ages and keys reset). 0x22: hidden + stopped (0x677430). 0x23: as 0x20 but vt+0x50 instead of
// the restart. 0x21 is not handled by vf44 (falls to vf43 0x536d40, which ignores it): no effect.
// TELEPORT_HELICO: 0x20 at 0 s on A306 (d 0x4ee0b306 = the link key of USA1's Particles_Z 0d09da6c: the rotor dust),
// 0x21 at 15 s (no effect) -> the dust follows A306 until the RTC ends.
enum class NodeFxOp { None, Start, Hide, StartKeep };
inline NodeFxOp nodeFxOp(uint32_t type) {
    return type == 0x20 ? NodeFxOp::Start : type == 0x22 ? NodeFxOp::Hide : type == 0x23 ? NodeFxOp::StartKeep : NodeFxOp::None;
}

// RTC Play / End record flags (AddRTC, Story/Levels.tsc; 0x53b6b0 / 0x53b8c0): 1 (default, cleared by PLAYER_MOVE): the
// .tsc line runner pauses (console +0xa8e4 |= 1); 2 (freeze): at Play the viewports' radial-blur flag 0x8000 is cleared
// (the game camera sets it again from its speed effect next frame), hidden player vehicle nodes are re-shown; at End the
// creatures are re-shown (0x419b30: no shipped effect); 4 CAMERA_CAME: unless set, End restores the LDR soften strength
// (renderer +0x1144 = 50.0, 0x658f60: used only by the HDR-off fallback 0x8778c0) and clears viewport flag 0x10 (SETDOF).
struct PlayFlags {
    static constexpr int PauseScript = 1, Freeze = 2, CameraCame = 4;
    static bool clearsRadialBlurAtPlay(int f) { return (f & Freeze) != 0; }
    static bool restoresLdrSoftenAtEnd(int f) { return (f & CameraCame) == 0; }
    static constexpr float kLdrSoftenEnd = 50.0f;  // 0x42480000
};

// The TR_DEATH overlay (placement 0xea, port_play 0x535260): when the RTC has no 0xe9 at 0 s, the father is a new node
// attached (0x65e410) to a camera-space 3-D item (+0x138, 0x4362c0: view 0x4373b0 FOV 90 deg, near 0.1, far 100) at
// local (0, 0, 1); End's vf41 0x536210 hides the item (0x436020(0)). The port draws the RTC objects in the world, so
// the father sits 1 m in front of the game camera with x and y scaled by tan(fov / 2) / tan(45 deg): every point of the
// overlay then projects where the FOV-90 item camera would put it. `fovH` = the game camera's horizontal FOV (4:3).
Mat4 overlayFather(const anim::Vec3& eye, const anim::Vec3& right, const anim::Vec3& up, const anim::Vec3& fwd, double fovH);
inline constexpr double kOverlayFov = 1.5707963267948966, kOverlayNear = 0.1, kOverlayFar = 100.0, kOverlayDepth = 1.0;

}  // namespace fuel::rtc
