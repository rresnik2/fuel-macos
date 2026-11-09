// PK22: FUEL's vehicle visual links, decoded from FUEL.exe (notes pk22.md):
//  - lamp groups: BuildVehicleLinks' 0x69da00 clones, per vehicle, the first part's Material_Z named LIGHT_BACK /
//    LIGHT_STOP / LIGHT_POSITION / LIGHT_HEAD (Material_Z::Clone, "%s_CL"), multiplies its colour +0x1c..+0x24 (the
//    emissive) by 5.0 and binds the clone into every part's material slot of that name (MeshData_Z::vf23 0x7b3e90 ->
//    0x7b3f80); the vehicle update 0x5cc780 sets the clone's alpha +0x18 to I or 0 every frame and 0x5cdfc0 hides it
//    (flag 0x80, the draw skip) at alpha <= 0. I = 0.9 with the headlights on (0x5cd860), else 1.0.
//  - V:DYN cosmetic dynamics: the records of 0x69e020 (defaults 0x6a2930, spring points 0x6a4100, reset 0x66f490),
//    their per-frame step in 0x5cc780 (engine shake RPM, spin TPM, spring points kept in their BOX by 0x75fbc0), the
//    part rotation of UpdateVehicleLinks 0x6a0200 and the damage accumulation of 0x6a2480 (break loose above 1.6).
//  - the dirt record: 0x529ce0's dirt block (core of FuelVehicle::stepDirt) and 0x5c9090's C_USER0 push (mean of the
//    10 slots, pushed when it moved by more than 0.025) to the vehicle (0x5dd8c0) and its rider (0x5dd300).
#pragma once

#include <array>
#include <cmath>
#include <cstdint>
#include <optional>
#include <string>
#include <vector>

namespace fuel::vlinks {

using V3 = std::array<float, 3>;
using Q4 = std::array<float, 4>;  // x, y, z, w

// ---- lamp groups (0x69da00, 0x5cc780, 0x5cdfc0) ------------------------------------------------------------------
enum Lamp : int { kBack = 0, kStop = 1, kPosition = 2, kHead = 3, kLampCount = 4 };
// The lamp group of a Material_Z by its link name CRC (asoboCrc("LIGHT_BACK"), ...), -1 = none.
int lampOfMaterial(uint32_t nameCrc);
// 0x5cc780's inputs: the physics body (0x4297d0 non-null), its gear (+0x1270), the brake value 0x5cd8f0 and the
// headlight state 0x5cd860 (0x69df30: the skeleton has headlight records and none is hidden).
struct LampInputs {
    bool physics = true;
    int gear = 1;
    float brakeValue = 0;
    bool headlights = false;
};
// Each group's clone alpha: I or 0 (0 = the batches are not drawn, flag 0x80).
std::array<float, kLampCount> lampAlphas(const LampInputs& in);
// 0x69da00: the clone's emissive (the first part's material colour x 5.0).
inline V3 lampEmissive(const V3& c) { return {c[0] * 5.0f, c[1] * 5.0f, c[2] * 5.0f}; }

// ---- V:DYN records (0x69e020, 0x6a2930, 0x6a4100, 0x66f490) --------------------------------------------------------
// A spring point (0x88 bytes): +0 velocity (world), +0xc position (body space), +0x18 gravity, +0x1c damping,
// +0x20 spring, +0x24 POINT name, +0x28 rest (the sphere centre), +0x34 its radius, +0x38 BOX name, +0x3c the box
// (16 floats: three rows {axis, centre component}, half extents, radius), +0x7c the DYNADOPT copy of the position.
struct SpringPoint {
    V3 vel{}, pos{};
    float gravity = 9.81f, damping = 10.0f, spring = 100.0f;  // 0x6a4100: 0x411cf5c3, 10, 100
    uint32_t point = 0;
    V3 rest{};
    float radius = 0;
    uint32_t boxName = 0;
    std::array<float, 16> box{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 1, 1, 1};
    V3 adopt{};
};
// A V:DYN record (0xa8 bytes).
struct DynRecord {
    bool loose = false;      // +0: broken loose (0x6a2480, > 1.6)
    bool detached = false;   // +1: the IT_DEBRIS was asked (0x5cc780)
    bool autostart = false;  // +2: AUTOSTART, RPM, RPM=, TPM=
    bool limb = false;       // +3: LIMB=<box> found
    bool adopt = false;      // +4: DYNADOPT
    float ldamage = 20.0f;   // +8 LDAMAGE= (0x6a2930: 20.0; read by nothing in 0x5cc780 / 0x6a0200 / 0x6a2480)
    float ldyn = 30.0f;      // +0xc LDYN= (the detach limit of +0x38)
    float rpm = 0;           // +0x10 RPM rate (radians), the engine shake amplitude
    V3 rpmAxis{0, 0, 1};     // +0x14
    float tpm = 0;           // +0x20 TPM, the spin rate factor
    float tpmAngle = 0;      // +0x24
    V3 tpmAxis{0, 0, 1};     // +0x28
    float damage = 0;        // +0x34 (0x6a2480: f (v + s); zeroed when it breaks loose)
    float damageTotal = 0;   // +0x38
    Q4 quat{0, 0, 0, 1};     // +0x3c (RPM / TPM parts)
    V3 axe{};                // +0x4c..+0x54 AXEX / AXEY / AXEZ
    uint32_t key = 0;        // +0x58 asoboCrc of the command after "V:" ("DYN:2"): the V:LINK tag
    uint32_t name = 0;       // +0x5c asoboCrc of the first argument
    std::array<float, 16> limbBox{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 1, 1, 1, 1};  // +0x60
    std::vector<SpringPoint> points;  // +0xa0 / +0xa4
};
// The skeleton's named spheres (0x1c bytes: centre, radius, type, name) and boxes (0x4c bytes: 16 floats, type,
// name), Skel_Z +0x40 / +0x48 (0x66bd60 / 0x66bdf0), in skeleton space.
struct NamedPoint {
    V3 c{};
    float r = 0;
    uint32_t name = 0;
};
struct NamedObb {
    std::array<float, 16> box{};
    uint32_t name = 0;
};
// 0x69e020's V:DYN branch: `tokens` = the command's tokens in order, quotes removed ("V:DYN:2", "Spoiler", ...).
// nullopt when FUEL keeps no record (no point, no RPM / TPM; a LIMB record without exactly 2 points).
std::optional<DynRecord> parseDyn(const std::vector<std::string>& tokens, const std::vector<NamedPoint>& spheres,
                                  const std::vector<NamedObb>& boxes);
// Splits a bone command line into its tokens (quoted or bare), quotes removed.
std::vector<std::string> commandTokens(const std::string& line);
// 0x66f490: the record's dynamic state back to rest.
void reset(DynRecord& r);

// ---- per frame (0x5cc780) -----------------------------------------------------------------------------------------
// The vehicle node (veh+0x4a8 position, +0x4c0 rotation, +0x4d0 velocity, +0x4f4 angular velocity) and engine
// (+0x1080 sound rpm, +0x1084 the shake phase, +0x1088 throttle; 0x5cd940 the gear).
struct Motion {
    V3 pos{};
    Q4 q{0, 0, 0, 1};
    V3 vel{}, angVel{};
};
struct Engine {
    float rpm = 0, phase = 0, throttle = 0;
    int gear = 1;
};
// 0x5cb8c0: the shake phase += rpm dt 0.01, wrapped above 2 pi (dt already clamped to 0.1).
float advancePhase(float phase, float rpm, float dt);
// 0x5cc780's V:DYN part for one record. Returns true when the record asks for its IT_DEBRIS (loose, not yet
// detached, +0xc < +0x38).
bool step(DynRecord& r, const Motion& m, const Engine& e, float dt);
// 0x75fbc0: the point clamped into the box (16 floats); true when it was outside.
bool obbClamp(const std::array<float, 16>& box, const V3& p, V3& out);
// 0x410510 / 0x4db890: q v q*.
V3 rotate(const Q4& q, const V3& v);
// 0x410260 Quat_Mul(a, b) = b a (Hamilton).
Q4 quatMul(const Q4& a, const Q4& b);
// 0x40f8b0 Quat_FromAxisAngle.
Q4 axisAngle(const V3& axis, float angle);
// 0x5cc370: the body's velocity at world point W (v + (w.z ry - w.y rz, w.x rz - w.z rx, w.y rx - w.x ry), r = W - pos).
V3 pointVelocity(const Motion& m, const V3& W);

// ---- the part pose (0x6a0200's V:DYN branch) -----------------------------------------------------------------------
// state: 0 = the static placement (the record is neither AUTOSTART nor loose), 1 = moved, 2 = no placement (detached).
struct DynPose {
    int state = 0;
    Q4 rot{0, 0, 0, 1};  // skeleton space: Qacc x boneQ (actor rotation applied by the caller)
    V3 pos{};            // skeleton space pivot (the bone, or the LIMB mid point)
};
// `bonePos` = the link bone's skeleton-space translation (+0x4c matrix row 3), `boneQ` = its +0x60 rotation.
DynPose pose(const DynRecord& r, const V3& bonePos, const Q4& boneQ);

// ---- damage (0x6a2480's V:DYN part) ---------------------------------------------------------------------------------
// f = 0x7cb550's result (1 a morph point hit, 0 none) or, for a part without morph target, r / distance of the sphere
// to the part's DYN_DAMAGE box (0x75fd20); v, s = the impact and scrape slot amounts.
void addDamage(DynRecord& r, float f, float v, float s);
// 0x75fd20: sphere (centre, r) against the box; false = no contact, else `dist` = the distance of the centre to the box
// (outside) or the smallest axis penetration + r (inside).
bool sphereBox(const std::array<float, 16>& box, const V3& c, float r, float& dist);

// ---- dirt (0x529ce0 dirt block, 0x5c9090) --------------------------------------------------------------------------
// The record 2 step for one frame: `loaded` = 0x52e620, `meanDirty` = mean of the wheels' ground block dirty (C+0x5c),
// `speed` = +0x13cc, `waterDepth` = +0x1380, `rain` = the weather's +0xb8; P594..P598.
struct DirtParams {
    float p594 = 0, p595 = 0, p596 = 0, p597 = 0, p598 = 0;
};
void stepDirt(float slots[10], float& total, float dt, bool loaded, float meanDirty, float speed, float waterDepth, float rain,
              const DirtParams& P);
// 0x5c9090: the mean of the 10 slots, and whether it moved by more than 0.025 from the pushed value.
float dirtMean(const float slots[10]);
inline bool dirtPush(float mean, float pushed) { return std::abs(mean - pushed) > 0.025f; }

}  // namespace fuel::vlinks
