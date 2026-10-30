// Every event the player can start: the 72 career races and 190 hub challenges
// (hubinfos.tsc AddHubCareerRaces / AddHubCHallengeRaces, the EMD blocks in
// miss_official.tsc / missChallenge.tsc, and whatever the races without an EMD
// block are decoded from). Shared contract between the event loader (P17) and
// the menus (P19); see ~/re-work/fuel/notes/p17-events.md.
#pragma once

#include "races.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace fuel {

struct Event {
    enum Kind { Career = 0, Challenge = 1 };
    std::string id;            // mission name, e.g. "CR2-Yell-CP" or a challenge's "(n)name:..:2km"
    Kind kind = Career;
    std::string hub;           // hub id as in hubs.hpp, e.g. "YELLOWSTONE"
    int tier = 0;              // hub tier / order in the career (0 = first)
    uint32_t nameText = 0;     // localized text id of the event's name (0 = none; show `id`)
    std::string vehicleFamily; // AddMissionNeededVehicle class, e.g. "QUAD", "BIKE", "BUGGY" ("" = any)
    std::string surface;       // e.g. "OFF", "ROAD", "OFF_ALL" ("" = any)
    bool playable = false;     // false when the port cannot run it yet (say why in `note`)
    std::string note;
    Race race;                 // start, checkpoints, finish, laps, mode
};

// All events in career order (hub, then tier), challenges after the career races
// of their hub. `dataDir` is the extracted game folder (the one holding CommonComponent).
std::vector<Event> loadEvents(const std::string& dataDir);

}  // namespace fuel
