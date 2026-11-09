// FUEL's dynamic game objects (P55, notes ~/re-work/fuel/notes/p55-objects2.md): IT_DYNBASE_G / IT_DYNOBJ_G
// (movable props and the pieces breakable decor leaves) and their move object IT_DYNOBJMove_G (a PhysicObj_Z body).
// Decoded from FUEL.exe:
//   IT_DYNOBJ_G vt[0]  0x51e080  init: the move object (+0x74) learns its agent (+0xe0), object flags +0x3c |= 0x8000000
//   IT_DYNBASE_G vt[20] 0x51b930 reset: move vt[14] (0x531490: back to the body defaults), the UserDefine (0x51c170 ->
//                       0x51bd10, below), hides the decor node it replaces (+0x88: node flag 0x40000, out of the hash),
//                       node scale 1, node flag 0x8000000 (in the physics scene)
//   IT_DYNOBJMove_G vt  0x9af3fc: vt[0] 0x5313b0 / vt[14] 0x531490 defaults (mass +0xe20 = 1000, density +0xe24 = PP150),
//                       vt[104] 0x531560 mass, vt[129] 0x5315d0 forces, vt[140] 0x531660 far check, vt[141] 0x531c00 contact
//   0x515cc0            break of an IT_DECORDYNMESH_G: every line of its Lod's UserDefine names an IT_DYNOBJ_G piece
//                       (agent by name, 0x4f16f0); dynamic pieces (agent +0x60 & 0x1000000) get the decor node's
//                       world rotation and position (0x713f20) and +0xef5 = 1; with decor +0x3c & 0x80000000 the
//                       piece agent gets 0x40000000 (the far check below); then decor +0x3c |= 0x40000000, the decor
//                       node is hidden (0x40000, 0x65eed0) and marked broken (+0x6c = 1)
// Game data stays in the player's files: the UserDefine text, the collision shapes and pp001.tsc rows are read at run time.
#pragma once

#include "static_collision.hpp"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace fuel {

// The IT_DYNBASE_G UserDefine keywords (0x51bd10, run from the reset 0x51b930 through 0x51c170 on the Lod's
// UserDefine, Lod +0x88; it upper-cases the word first, so the keywords match without case). PhysAgent_Z's copy
// 0x741280 -> 0x740e80 runs earlier in the same reset and matches nothing in retail (PK30: splitter 0x740df0 bug, see
// parseDynObjUserDefine). Each line is "KEYWORD [value]".
struct DynObjUserDefine {
    // MASSEVOLUMIQUE v: density +0xe24 = clamp(v, 10, 4000); v = 0 or no value: 500. < 0 = keyword absent (PP150).
    float massVolumique = -1;
    bool immobile = false;  // HIMMOBILE (+0xed6 = 1)
    bool noMagnet = false;  // NOMAGNET (+0xeb4 = 1: the body ignores force volumes of kind 4, 0x710e30)
    // SPEEDFACTOR v (+0xe18 = clamp(v x 0.01, 0.25, 2)), < 0 = absent. Read by code not decoded yet.
    float speedFactor = -1;
    // P79: the rest of PhysAgent_Z's table (0x740e80, the same keywords as 0x51bd10, upper-cased first): the explosion /
    // damage fields of the body. Kept for the explosion code (not ported).
    bool hrDepth = false;        // HRDEPTH (+0xed3)
    float explodeDelay = -1;     // DISAPPEAREXPLO / DELAY v: +0xe28 = v x 0.001 s, no value 0.5; < 0 = absent
    float explodeDamage = -1;    // EXPLODAMAGE / MINPOWER v: +0xe2c = +0xe30 = v (no value 0); < 0 = absent
    float damage = -1;           // DAMAGE v (+0xe08, no value 1.0); < 0 = absent
    bool explodeOnTouch = false; // EXPLOTOUCH (+0xeaa = +0xeab = +0xeb5 = 1)
    std::string explodeFx;       // EXPLOFX name (+0xdfc = its CRC; +0xe28 <= 0 -> 0.01)
    std::string replaceWith;     // REPLACE name (+0xe00 = its CRC; same default)
    // Lines that are not keywords: the first token of each (0x515cc0 looks every line up as an agent name; only names of
    // existing IT_DYNOBJ_G agents spawn pieces).
    std::vector<std::string> names;
};
DynObjUserDefine parseDynObjUserDefine(const std::vector<std::string>& lines);

// PK09: 0x7b1a80, the DISTACTIVE_ parser of AnimatedAgent_Z's init (vf22 0x667220, run for a Lod object (+0x64 == 5)
// with a UserDefine +0x88): every line starting with "DISTACTIVE_" (strnicmp, 11 chars) gives the '_'-separated
// decimals after it (sscanf %d, kept when >= 0) in order; returns them (FUEL: the count, values in out[]).
std::vector<int> parseDistActive(const std::vector<std::string>& lines);
// The agent's activation distance squared (+0x40): DISTACTIVE's first value squared (int product, then float) when the
// UserDefine has one, else `fallback` (IT_DYNBASE_G's reset 0x51b930 / 0x51d080: [0xa0521c] = 5625, 75 m). Its reader is
// not traced [U] (p47: no class is gated by it in the port; IT_DISCOVERY carries DISTACTIVE_1000, the 1000 m of 0x4ed2d0).
float distActive2(const std::vector<std::string>& lines, float fallback = 0);
constexpr float kDynActiveDist2 = 5625.0f;  // [0xa0521c]

// pp001.tsc rows the dynamic objects read (CommonComponent/Param/pp001.tsc, "PP n value", array at *0xa6749c).
struct DynObjParams {
    float densityBig = 50;   // PP150: default density +0xe24 (0x5313b0 / 0x531490), used for volumes above 1 m^3
    float densitySmall = 5;  // PP151: density of bodies of at most 1 m^3 (0x531560)
    bool loaded = false;
};
DynObjParams loadDynObjParams(const std::string& pp001Path);

// PhysicObj_Z's body volume (0x712de0, volume pass): 0.0001 + sum over the body's spheres 4.1762004 r^3 (the
// constant as coded, not 4 pi / 3) + over its boxes 8 hx hy hz. `scale` multiplies lengths [I: placements scale the
// shapes; the pieces are unscaled].
double dynObjVolume(const StaticCollision& c, float scale = 1.0f);
// IT_DYNOBJMove_G mass (vt[104] 0x531560, the explicit mass +0xe3c is never set for these): volume > 1 m^3 ->
// density x volume (density = MASSEVOLUMIQUE or PP150), else PP151 x volume.
double dynObjMass(double volume, const DynObjUserDefine& ud, const DynObjParams& pp);

// Per-step body forces of IT_DYNOBJMove_G (vt[129] 0x5315d0, per substep):
//  - gravity VehMove_Gravity(-9.81) (0x530cb0: F = m (0, -9.81, 0));
//  - drag 0x5321e0(m x 0.005, m x 0.005) with k = +0x564 = 200 (PhysicObj_Z reset 0x70bf50): linear
//    F = -k (0.005 m) min(|v|, 10) v/|v| (|v| > 0.0001), angular torque = -I k 0.005 min(|w|, 10) w/|w| (world inertia
//    +0x6f0 times the inverse mass): both a deceleration of min(speed, 10) per second, independent of the mass;
//  - vt 0x218(1, 1, 1, 0) (axis factors) and the force volumes PhysObj_ForceVolumes(150 m) (0x710e30: wind, tornado and
//    magnet volumes 0x681540, the force capped at 150 m; not ported, see p55 notes).
constexpr float kDynGravity = 9.81f;
constexpr float kDynDragRate = 200.0f * 0.005f;  // 1/s
constexpr float kDynDragMaxSpeed = 10.0f;        // m/s (0x996494)
constexpr float kDynDragMaxSpin = 10.0f;         // rad/s
constexpr float kDynForceVolumeLimit = 150.0f;   // x mass (0x9dca04)
// Applies one step of that drag to a velocity (linear or angular): v -= min(|v|, cap) v/|v| x rate x dt (never reversing).
std::array<float, 3> dynObjDrag(const std::array<float, 3>& v, float cap, float dt);

// vt[141] 0x531c00 (contact filter of a dynamic object): the peak approach speed is kept (+0xef0, impact effects in
// vt[139]); a contact with a breakable decor object (agent +0x60 & 0x800000, not broken +0x6c) at an approach faster than
// 5 m/s (contact +0xcc < -5, 0x9dc100) breaks it (0x515cc0) like a vehicle above its P575.
constexpr float kDynBreakSpeed = 5.0f;
// vt[140] 0x531660: a piece whose agent has 0x40000000 (spawned by a break of a decor with +0x3c & 0x80000000) and that
// is more than 500 m (0x9dbfc4) from the point 0x421980 gives is put out of the physics world (+0xeb3 -> 0x6837c0 ->
// 0x730ee0(agent, 1)).
constexpr float kDynFarDistance = 500.0f;

}  // namespace fuel
