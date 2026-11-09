// [suspanim] The vehicle's suspension and steering parts every frame.
//
// [O] FUEL: Skel_Z::UpdateVehicleLinks 0x6a0200 recomputes every link record of the drawn LOD each frame (spec S18,
// notes/suspanim.md): a part sits at its commanding end K and turns about one hinge axis (X, or Z with AxeZ) so that its
// LOCAL_LOOKAT points at the other end: angle = sign(d . a) (acos(d^.y) - acos(l^.y)) with a = (0,0,1) [0xa06350] for the X
// hinge, (1,0,0) [0xa06338] for Z. The other end is a bone (V:LINK A: the wheel bones move with the wheels, 0x5caf40) or a
// point of another part (V:LINKB A+P: through A's live placement). V:LINKM parts sit at A.P and, aimed at a wheel bone,
// take that wheel's steer vector (Quat_FromArc((0,0,1), skel+0x98[w])). V:LINKR A turns about its LOCAL_LOOKAT by the steer
// angle (atan2 of the steer vector projected off the axis: handlebars, forks, steering wheels); with LOCAL_ATTACH /
// LOCAL_LIMIT the part slides along the axis to the wheel and the wheel is put on the axis at the hub's height (n = (0,1,0)
// [0xa06320]), clamped to the limit. V:LINKR A B C aims at the mid point of A and C and rolls with A - C (solid axles).
// Skinned parts (Lod_Z with a 3-bone Skin_Z: shocks) have their end bone set to the target (0x676480 / 0x66bb40); the port
// stretches the part along its axis instead ([port] approximation, same ends).
//
// [port] the visible upgrades (core/upgrade_visuals Live): re-aimed parts / coils / coil-overs follow the live hub.
//
// Everything is a vehicle-space affine map per part (rest -> live), so the renderer's per-instance worlds (and the TAA
// velocity pass's previous worlds) carry it. At rest (hubs at the bones, no steering) every FUEL map is the identity.
#pragma once

#include "assembly.hpp"

#include <array>
#include <vector>

namespace fuel::upgvis {
struct Plan;
struct Live;
}  // namespace fuel::upgvis

namespace fuel::susp {

using V3 = std::array<float, 3>;

// p' = M p + t (column vectors), M row-major.
struct Affine {
    std::array<float, 9> m{1, 0, 0, 0, 1, 0, 0, 0, 1};
    V3 t{};
    V3 apply(const V3& p) const {
        return {m[0] * p[0] + m[1] * p[1] + m[2] * p[2] + t[0], m[3] * p[0] + m[4] * p[1] + m[5] * p[2] + t[1],
                m[6] * p[0] + m[7] * p[1] + m[8] * p[2] + t[2]};
    }
};

struct WheelLive {
    V3 hub{};               // vehicle space
    V3 forward{0, 0, 1};    // the steer vector (skel+0x98[w]), vehicle space
};

// The switch: FUEL_SUSP_ANIM=0 keeps the parts rigid (the port before suspanim; before / after shots).
bool enabled();

enum class Kind : uint8_t { Hinge, Upright, Steer, Axle, Segment, Translate };

struct Node {
    Kind kind = Kind::Hinge;
    V3 K{};                // rest commanding end (vehicle space)
    int kWheel = -1;       // K is a wheel bone
    int atNode = -1;       // V:LINKM: placed at a point of that node's part
    V3 look{};             // LOCAL_LOOKAT
    int axis = 0;          // 0 X, 2 Z
    bool stretch = false;  // skinned: the end follows the target
    int aimWheel = -1, aimNode = -1, aimWheel2 = -1;
    V3 aim{}, aim2{};      // rest targets
    int steerWheel = -1;
    bool slide = false;    // V:LINKR with LOCAL_ATTACH / LOCAL_LIMIT
    float attachT = 0, limitT = 0, limitR = 0;
    // [port] upgrade visuals
    int wheel = -1, wheel2 = -1;
    V3 C{}, B{}, D{}, pivot{};
    float sB = 1, thick = 1;
};

struct Rig {
    std::vector<Node> nodes;           // evaluation order (dependencies first)
    std::vector<int> partNode;         // per AssembledVehicle part: node index, -1 = rigid
    std::vector<int> extraNode;        // per Plan::liveExtras entry
    std::vector<V3> hubRest;           // per setup wheel: the setup hub (the plan's rest)
    int slideWheel = -1;               // the wheel FUEL puts on a fork axis
    bool any() const { return !nodes.empty(); }
};

// `hubs`: the setup's wheel hubs (after upg::apply); `plan`: the visible upgrades of this record (nullptr / inactive: none).
Rig build(const AssembledVehicle& av, const std::vector<V3>& hubs, const upgvis::Plan* plan);

struct Output {
    std::vector<Affine> node;   // per Rig node
    std::vector<V3> wheelShift; // per setup wheel (FUEL's fork wheel), vehicle space
};
void evaluate(const Rig& rig, const std::vector<WheelLive>& wheels, Output& out);

// ---- the math (exposed for the vectors)
// 0x6a0200's hinge angle: d = target - K, l = LOCAL_LOOKAT, axis 0 (X) / 2 (Z).
float hingeAngle(const V3& d, const V3& l, int axis);
// A right-handed rotation about a unit axis (row-major 3x3).
std::array<float, 9> rotation(const V3& axis, float angle);
// The rotation about unit `u` that turns the projection of `from` off u onto that of `to` (identity when degenerate).
std::array<float, 9> turnAbout(const V3& u, const V3& from, const V3& to);
// The upgrade visuals' segment map (upgvis::segmentMap) as an Affine.
Affine segment(const V3& C, const V3& B, const V3& B2, float thick);

}  // namespace fuel::susp
