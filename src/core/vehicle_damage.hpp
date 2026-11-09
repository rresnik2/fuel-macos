// FUEL's visual vehicle damage (P85, notes p85-damage.md), decoded from FUEL.exe:
//  - 0x5c7760 (per frame, from the vehicle visual update 0x5cb8c0): when a slot of the impact record (pm+0x144)
//    moved by more than 0.01 since the last pass (0x5c6560), the morph damage is cleared (0x6a27b0) and rebuilt from
//    the 10 slots: slot i > 0 is a sphere at the body's local AABB (0x712410) centre + dir_i x half extents (dir =
//    FUEL.exe 0xa057a8, the 10 directions of 0x51fd50), radius v_i (T_i . half) / (T_i . G) (T = 0xa060e8, G =
//    0xa06308), handed to 0x6a2480 with the scrape slot i (pm+0x118) as a second amount.
//  - 0x6a2480 per V:LINK part of the vehicle skeleton's first LOD (skel+0x144 record 0): a Mesh_Z part runs
//    0x7cb550 in the part's local space; the 6 wheel slots bend by up to 4 degrees (not ported: no shipped wheel
//    carries the DYN_DAMAGE box it tests); V:DYN parts accumulate and break loose above 1.6 (V:DYN not ported).
//  - 0x7cb550: every morph point (int16 / 1024) strictly inside the sphere copies its delta (morph target 0's int16
//    x4 / 4096) into a per-instance stride-12 VB (zeroed on creation, one per part until 0x6a27b0 frees it); the draw
//    0x848880 binds that VB as the morph stream with weight 1 (else target 0's own VB with object data +0x4c, which
//    nothing in FUEL.exe writes: no static morph is ever shown). Returns 1 (a point hit), 0, or -1 (no morph table).
// Nothing of the game is compiled in: the three tables are read from the player's FUEL.exe at run time.
#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace fuel {

struct Mesh;
class DpcFile;

namespace damage {

struct Tables {
    bool ok = false;
    std::array<float, 30> dirs{};    // 0xa057a8: the 10 body-frame directions (8 horizontal at 45 deg, up, zero)
    std::array<float, 30> radius{};  // 0xa060e8: the axes whose half extents give the sphere radius
    std::array<float, 3> norm{};     // 0xa06308: their normaliser
};
// From the FUEL.exe found next to `anyDpc` (genworld_noise readEngineFloats). ok = false when it cannot be read.
Tables loadTables(const DpcFile& anyDpc);
Tables loadTables(const std::string& exe);

// 0x5c6560: true when one of the 10 impact slots differs from the copy by more than 0.01.
bool slotsChanged(const float prev[10], const float cur[10]);

struct Sphere {
    std::array<float, 3> c{};
    float r = 0;
};
// 0x5c7760: the sphere of slot i (amount v > 0) in the body's frame, from its local AABB (min, max).
Sphere slotSphere(const Tables& t, int slot, const std::array<float, 3>& mn, const std::array<float, 3>& mx, float v);

// 0x7cb550 on one mesh: `sphere` in the mesh's local space; `vb` (3 floats per vertex of morph target 0) is created
// zeroed on the first hit and keeps earlier hits. Returns 1 / 0 / -1 as FUEL.
float morphDamage(const Mesh& m, const Sphere& sphere, std::vector<float>& vb);

}  // namespace damage
}  // namespace fuel
