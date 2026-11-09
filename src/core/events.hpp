// Every event the player can start: the 72 career races and 190 hub challenges
// (hubinfos.tsc AddHubCareerRaces / AddHubCHallengeRaces, the EMD blocks in
// miss_official.tsc / missChallenge.tsc, and the MISSION_COURSE nodes of USA1.DPC
// for the 15 career races without an EMD block, notes/p42-missions.md). Shared contract between the event loader (P17) and
// the menus (P19); see ~/re-work/fuel/notes/p17-events.md.
#pragma once

#include "races.hpp"

#include <cstdint>
#include <string>
#include <vector>

namespace fuel {

struct Event {
    enum Kind { Career = 0, Challenge = 1, Edited = 2 /* PK21: an edited race of the career (race_editor.hpp) */ };
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

    // Added by P17 (all decoded from hubinfos.tsc; see notes/p17-events.md):
    uint32_t descText = 0;     // challenges: rule description text (6013..6022; the challenge kind)
    uint32_t surfaceText = 0, vehicleText = 0;  // AddMissionSurfaceType: e.g. 930 "Mostly Offroad", 2955 "Buggy"
    std::string vehicle;       // challenges: the forced variant, e.g. "BUGGY_OFF_3" ("" = the player's choice)
    float times[3] = {0, 0, 0};  // AddMissionTime 1st..3rd values (career: three target times; challenges: 1st only)
    std::string aiVehicles;    // AddMissionParam AI vehicle ids ("18_61_33_..."; "NONE" = none)
    // Those ids as vehicle labels (e.g. "BIKE_OFF_1"): 0x575b00 atol's each '_' field (at most 32) into the
    // mission's list +0xa8/+0xac; IA_RaceSetup 0x620e30 gives AI racer k the entry k mod n, an index into the
    // game's vehicle table (DAT_00a7c094+0x3254) = the AddVehicleInfo lines of VehiclesDef.tsc in order (P43).
    std::vector<std::string> aiVehicleLabels;
    int hubIndex = 0;          // position of `hub` in the hub list (SetHubTargetInfos order)
    // AddMissionParam's weather strings (args 5 and 7, e.g. "C#0_TOR_W#100"; parser 0x5759c0). P61: "TOR" in the
    // start string sets mission +0x18c, the tornado race (0x631bd0 places the tornado route).
    std::string weatherStart, weatherEnd;
    bool tornado = false;
};

// How many opponents an event puts on the grid: EMD_SetNbIA (no vehicle for the helicopter), or `requested`
// (--opponents) when given (>= 0), at most 11. Shared by --race and --career-audit (P99).
inline int eventOpponents(const Event& e, int requested = -1) {
    if (e.race.gameMode() == Race::GmHelicopter) return 0;
    const int n = requested >= 0 ? requested : e.race.aiCount;
    return n < 0 ? 0 : n > 11 ? 11 : n;
}

// Short English name of a game mode (Race::gameMode()), e.g. "knock out".
const char* gameModeName(int gameMode);

// All events in career order (hub, then tier), challenges after the career races
// of their hub. `dataDir` is the extracted game folder (the one holding CommonComponent).
std::vector<Event> loadEvents(const std::string& dataDir);

}  // namespace fuel
