// The 19 hub regions for free roam: camp spawn, teleports and vista points
// (hubinfos.tsc SetHubTargetInfos, poi.tsc AddTypePointOfInterest, the
// USA1_REGION_HUB_* worlds). Shared contract between the hub loader (P18) and
// the menus (P19); see ~/re-work/fuel/notes/p18-hubs.md. Heights are not stored:
// callers take y from the terrain.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace fuel {

struct HubPoint {
    std::string name;   // e.g. "Yellowstone_Teleport_3"
    float x = 0, z = 0;
    float yaw = 0;      // radians, forward = (sin, cos) in x, z
};

struct Hub {
    std::string id;           // e.g. "YELLOWSTONE" (as hubinfos.tsc spells it, upper case)
    uint32_t nameText = 0;    // localized text id of the hub's name (0 = none; show `id`)
    int tier = 0;             // unlock order in the career (0 = first)
    HubPoint camp;            // where free roam in this hub starts
    std::vector<HubPoint> teleports;  // helicopter teleport targets
    std::vector<HubPoint> vistas;     // vista points
};

// The hubs in career order. `dataDir` is the extracted game folder.
std::vector<Hub> loadHubs(const std::string& dataDir);

}  // namespace fuel
