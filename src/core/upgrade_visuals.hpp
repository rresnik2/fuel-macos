// [DEVIATION, the user's request 2026-11-06] Visible vehicle upgrades (package U5 of ~/re-work/fuel/notes/upgrades-plan.md;
// notes/upg5.md). FUEL (2009) has no upgrades: everything here is the port's. Nothing in this file changes the physics or
// the collision: the physical lift and track widening are U1's (core/upgrades); the visuals follow them.
//
// A vehicle's stages (VehicleSetup::upgStages, set by upg::apply) become a Plan: per assembled part a vehicle-space
// adjustment (suspension parts re-aimed and stretched to the lowered hubs, thicker shocks and forks, engine / exhaust
// parts grown), a mesh override (the part drawn through a derived copy of its mesh: optional bodywork batches left out,
// the paint turned to a carbon weave), hidden parts (optional bodywork), the wheels' axle / radial scale, and procedural
// extras built in code (spacers, fender flares, coloured coil springs, exhaust tips, a hood scoop or intake pods, nitrous
// bottles, tyre sidewall stripes and tread lugs). Plus the nitro flame's size and outlets and the engine note's filter.
//
//   Suspension  the physical lift shows by itself (the body rides higher); suspension parts are re-aimed at the lowered
//               hubs (static hinge at their chassis end, stretched along their axis), shocks / forks 1.1 / 1.2 / 1.3 x
//               thicker, coil springs yellow / orange / red; wheel spacers where the track grew, fender flares over
//               the wheels of enclosed bodies. Vehicles without shock parts get procedural coil-overs.
//   Tires       wheel x1.08 / 1.15 / 1.25 along the axle, taller up to the physics radius; S2 a sidewall stripe, S3 tread
//               lugs (off-road classes) or a wider stripe (road)
//   Engine      exhaust tips (chrome) 1.2 / 1.45 / 1.7 x the pipe; ENGINE / EXHAUST parts x1.05..1.15 / 1.1..1.3; S2+ a
//               hood scoop (enclosed bodies) or intake pods (an ENGINE part); engine note +1.5 / 3 / 4.5 dB with a
//               low shelf +2 / 4 / 6 dB under 200 Hz
//   Weight      carbon weave on the paint: S1 the bonnet, S2 + every top surface, S3 all; S2 hides optional parts
//               (bumpers, spoilers, mirrors, spare wheels, loose V:DYN panels), S3 also bumper / net / spare-wheel batches
//   Nitro power the nitro flame x1.25 / 1.5 / 1.8 long, x1.1 / 1.2 / 1.3 wide
//   Nitro tank  1 / 2 / 3 blue bottles (bed, deck, roof or rack; by the tail on bikes)
// Opponents (never LOD1) get the subset that shows on them: suspension parts, wheels, spacers, carbon, flames.
#pragma once

#include "assembly.hpp"
#include "rider.hpp"

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace fuel {
struct VehicleSetup;
}

namespace fuel::upgvis {

// The switch: FUEL_UPGRADE_VISUALS=0 turns the visible upgrades off (the physics stays upgraded; A / B shots and perf).
bool enabled();
void setEnabled(bool on);

// Materials of the procedural parts (core/custom_rider.hpp CustomMaterial, drawn by the renderer's custom path).
enum class Mat : uint8_t { Bottle, Chrome, Dark, Plastic, Spacer, SpringS1, SpringS2, SpringS3, Stripe, StripeS3, Paint, Filter, Carbon, kCount };
const CustomMaterial* material(Mat m);

// A procedural part: its mesh (registered under `key`, see proceduralMesh) and where it hangs.
// [suspanim] how a re-aimed part or a procedural coil follows its wheel every frame (core/susp_anim): Segment = C fixed,
// B -> B + sB x (D + live hub - rest hub) (stretched along C-B, `thick` across; a fork also turns with the steering about its
// axis); Translate = moved by D + the live hub delta (an upright also steers about its hub `pivot`; an axle takes the mean of
// `wheel` / `wheel2` and rolls with their difference).
struct Live {
    enum Kind : uint8_t { None, Segment, Translate } kind = None;
    int wheel = -1, wheel2 = -1;  // setup wheels
    std::array<float, 3> C{}, B{}, D{}, pivot{};
    float sB = 1, thick = 1;
    bool steer = false;  // turns with the wheel's steering (forks, uprights)
};

struct Extra {
    uint32_t key = 0;
    Live live;  // [suspanim] Plan::liveExtras: how it follows the wheel (vehicle space at rest: the plan's static stance)
    int wheel = -1;  // -1: on the chassis (vehicle space); else the setup wheel whose pose it follows (wheel space)
    std::string what;  // "spacer", "flare", "coil", "tip", "scoop", "bottles", "stripe", "lugs", "pods", "coilover"
    uint32_t triangles = 0;
};

// What the plan does to one assembled part (AssembledVehicle::parts index).
struct PartEdit {
    bool hide = false;
    bool moved = false;   // `adjust` is not the identity
    Mat4 adjust{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1};  // vehicle space, row vectors: world' = world x adjust
    uint32_t mesh = 0;    // a derived mesh key to draw instead of PlacedPart::meshId (0: the part's own)
    float axle = 1, radial = 1;  // wheels: mesh-space scale along the axle (x) and across (y, z)
    int wheel = -1;       // wheels: the setup wheel it follows (nearest hub, as VehicleRig)
    std::string why;      // "arm", "shock", "upright", "axle", "fork", "engine", "exhaust", "hidden: BUMPER", ...
    Live live;            // [suspanim] suspension parts: how `adjust` follows the live wheel (adjust = the rest of it)
};

struct Outlet {  // an exhaust outlet, vehicle space: tips and the nitro flame
    std::array<float, 3> pos{}, dir{0, 0, -1};
    float radius = 0.04f;
};

struct Plan {
    bool active = false;
    int cls = -1;
    std::array<int, 6> stages{};  // upg::Category order
    bool opponent = false;
    float lift = 0, widenFront = 0, widenRear = 0;
    std::vector<PartEdit> parts;  // one per AssembledVehicle part
    std::vector<Extra> extras;
    // [suspanim] procedural parts that move with a wheel every frame (coils round the shocks, coil-overs): one mesh each,
    // vehicle space at the static stance (restScene draws them there, as the chassis extras).
    std::vector<Extra> liveExtras;
    std::vector<Outlet> outlets;  // the nitro flame's outlets when the engine or nitro stages are set (else FUEL's / the nitro branch's)
    bool outletsFromSfx = false;  // the outlets are the SFX_EXHAUST slots (the FX keep using those)
    float flameLength = 1, flameSize = 1;  // nitro flame scale (nitro power stage)
    float engineGainDb = 0, engineShelfDb = 0;  // the engine note (player)
    int carbon = 0;
    uint32_t extraTriangles = 0;
    std::vector<std::string> notes;  // one line per decision (FUEL_U5_TRACE, the checks)
};

struct Options {
    bool opponent = false;  // the subset that shows on a non-LOD1 opponent
    bool rest = false;      // (unused by makePlan; restScene) the garage / static stance
};

// The plan for `av` (one LOD record of the vehicle) with `setup` after upg::apply (hubs shifted, lift / widening set,
// upgStages). Inactive when the visuals are off, the setup is not upgraded or its stages are all stock.
Plan makePlan(const AssembledVehicle& av, const VehicleSetup& setup, const Options& o = {});
std::string describe(const Plan& p);

// A static scene of the planned vehicle at rest (the garage preview, --assemble shots): every drawn part (with its mesh
// override; hidden ones left out) and extra, vehicle space, row vectors. The body rides `lift` higher than FUEL's rest,
// the wheels stand where they did, `widen` further out (the physics' rest stance on flat ground).
struct ScenePart {
    uint32_t mesh = 0;
    Mat4 world{};
    int part = -1;  // AssembledVehicle part index, -1 for an extra
};
std::vector<ScenePart> restScene(const AssembledVehicle& av, const VehicleSetup& setup, const Plan& p);

// ---- the registry (process-wide, thread-safe): procedural meshes and derived meshes by key
// A procedural mesh's geometry (nullptr: not one).
std::shared_ptr<const BakedMesh> proceduralMesh(uint32_t key);
// A derived mesh: `base` (a Mesh_Z) drawn without the batches of the listed material names, its paint batches (codes 17 /
// 19) with the carbon weave of `carbon` (1..3; 0 none): `carbonParams` = (level, front z, scale, 0) in the mesh's space.
struct Derived {
    uint32_t base = 0;
    std::vector<uint32_t> hiddenMaterials;  // Material_Z name CRCs
    int carbon = 0;
    std::array<float, 4> carbonParams{};
    std::vector<uint32_t> carbonMaterials;  // body-skin materials (code 0 / 18, by name CRC) that take the carbon tint too
};
const Derived* derivedMesh(uint32_t key);
// The Mesh_Z a key stands for (the base of a derived mesh, else the key): the damage morphs and the V:DYN boxes.
uint32_t baseMesh(uint32_t key);

// ---- geometry helpers (exposed for the checks)
// The segment map of a re-aimed part: C fixed, B -> B' (stretch along C->B, `thick` across), as a vehicle-space row-vector
// matrix.
Mat4 segmentMap(const std::array<float, 3>& C, const std::array<float, 3>& B, const std::array<float, 3>& B2, float thick);
std::array<float, 3> transformPoint(const Mat4& m, const std::array<float, 3>& p);  // row vector: [p 1] x m

}  // namespace fuel::upgvis
