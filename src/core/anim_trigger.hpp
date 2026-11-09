// P55: the trigger volumes that start FUEL's animated storm objects (IT_ANIMATED event slot 334), notes
// ~/re-work/fuel/notes/p55-objects2.md (D6) and p47-work/anim-objects.md:
//   - course sub-worlds hold CollisionVol_Z nodes whose UserDefine reads e.g. "CONDITION_OBJECT SONS ; DEFAULTHIDE ;
//     UNHIDE_M_RUNNING CR9-ISLA-SR ; RESET_M_START CR9-ISLA-SR ; STARTANIMATED ; STARTFX" (CR9 8, CR36 4, CR54 9);
//   - the volume's event agent (EvenementVolAgent_G vt 0x5708b0) runs the node's UserDefine actions 0x570ae0 when the
//     player enters; STARTANIMATED (0x570d3d) walks the volume node's children (+0xa4, then +0xac) and starts each
//     child's agent: IT_ANIMATED 0x518170 = if not started: active, started, play slot 334 once (0x667a00, ecx 0x14e);
//   - RESET_ (0x513ca0 -> IT_ANIMATED vt[20] 0x517f80) plays slot 0 again and re-arms;
//   - the volume box: CollisionVol_Z box 0 (0x7375d0: box matrix x node matrix, row vectors; 0x743210 OBB = normalised
//     rows, extents = row lengths, centre = translation).
#pragma once

#include "dpc.hpp"
#include "object_script.hpp"
#include "sand_tank.hpp"  // PK12

#include <array>
#include <cstdint>
#include <memory>
#include <string>
#include <vector>

namespace fuel {

// ---- PK12: EvenementVolAgent_G (notes pk12.md): vf24 0x570580 options, vf43 0x5708b0 gates, the STARTFX children.
// Shipped data (22 EvenementVolAgent_G volumes: 8 CR9, 4 CR36, 9 CR54, 1 USA1 PLAYRTC): no option keyword, text id 0,
// no facing test; STARTFX finds a Spline_Z child (with a CollisionVol_Z below it) only in CR54's four SAND volumes.
struct EventVolumeOptions {
    bool activeIfPreviousDone = false;  // +0x70 ACTIVE_IF_PREVIOUS_DONE (vf43: the previous event's node must be hidden)
    bool actionInVolume = false;        // +0x71 ACTION_DANS_VOLUME (vf43 starts the 0xc9 routine instead of firing)
    bool inactiveInMission = false;     // +0x72 INACTIVE_IN_MISSION (parsed; no reader in vf43)
    bool alwaysActive = false;          // +0x73 ALWAYS_ACTIVE (0x570f60 does not disarm +0x7c)
    bool switchMode = false;            // +0x74 SWITCH_MODE (0x570f60 -> 0x570f10)
    std::string onlyWhenMission;        // +0x80 ONLYWHENMISSION <name> (0x558100): fires only while it runs (0x57f540)
    int textInfo = 0;                   // +0x78 = ftol(CollisionVol +0xac + 0.5): AddInGameTextInfos id when > 0
    bool facing = false;                // +0x75 = CollisionVol +0xb0 > 1e-6: the text needs the player facing the volume
    bool startAnimated = false, startFx = false;  // the UserDefine actions 0x570ae0 runs (STARTANIMATED / STARTFX)
};
// One STARTFX child C (Spline_Z) with its CollisionVol_Z child G (0x570ae0 at 0x570e50).
struct StartFxChild {
    uint32_t spline = 0;  // C
    sand::Fire fire;      // G, TIME (C), SAND (G), G's OBB
};
// ---- PK12 end

struct AnimStartVolume {
    uint32_t node = 0;
    std::array<float, 3> centre{};
    std::array<std::array<float, 3>, 3> axis{};  // unit axes (world)
    std::array<float, 3> half{};                 // extents along them (PK12: 0.5 x the row lengths, 0x7375d0)
    std::vector<uint32_t> objects;               // the children's objects (IT_ANIMATED Lods)
    std::shared_ptr<const ObjectCondition> condition;  // the volume's own CONDITION_OBJECT (shown while ...), may be null
    bool contains(double x, double y, double z) const;
    EventVolumeOptions options;           // PK12
    std::vector<StartFxChild> startFx;    // PK12
};

// The STARTANIMATED / STARTFX volumes of one world DPC (objects resolved in `lookup`).
std::vector<AnimStartVolume> loadAnimStartVolumes(const DpcFile& dpc, const std::vector<const DpcFile*>& lookup);

}  // namespace fuel
