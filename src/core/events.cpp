// The event catalogue: hubinfos.tsc (hub order, career and challenge lists, needed vehicles,
// times) joined with the missions: the EMD blocks of miss_official.tsc and missChallenge.tsc, then
// the MISSION_COURSE nodes of USA1.DPC (0x4f4db0, notes/p42-missions.md).
// FUEL.exe joins them the same way: AddHubCareerRaces (0x575370) and AddHubCHallengeRaces
// (0x5755e0) look the mission up by name with _stricmp and silently skip names that match no
// mission. See ~/re-work/fuel/notes/p17-events.md (E1-E9).
#include "custom_vehicles.hpp"  // CUSTOM VEHICLES
#include "events.hpp"
#include "storms.hpp"

#include "console.hpp"
#include "dpc.hpp"

#include <strings.h>

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <map>
#include <cstdlib>
#include <sstream>

namespace fuel {

namespace {

std::vector<std::string> tokens(const std::string& line) {
    std::vector<std::string> out;
    size_t i = 0;
    while (i < line.size()) {
        const char c = line[i];
        if (c == ' ' || c == '\t' || c == '\r') {
            ++i;
        } else if (line.compare(i, 2, "//") == 0) {
            break;
        } else if (c == '"') {
            const size_t end = line.find('"', i + 1);
            out.push_back(line.substr(i + 1, end == std::string::npos ? std::string::npos : end - i - 1));
            i = end == std::string::npos ? line.size() : end + 1;
        } else {
            size_t end = i;
            while (end < line.size() && line[end] != ' ' && line[end] != '\t' && line[end] != '\r') ++end;
            out.push_back(line.substr(i, end - i));
            i = end;
        }
    }
    return out;
}

std::string upper(std::string s) {
    for (char& c : s) c = char(std::toupper(static_cast<unsigned char>(c)));
    return s;
}

std::string readFile(const std::filesystem::path& p) {
    std::ifstream f(p, std::ios::binary);
    std::stringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

// The game rounds script numbers to int as (int)(value + 0.5) (0x575443).
uint32_t textId(const std::string& s) {
    if (s.empty() || s == "NONE") return 0;
    return uint32_t(std::max(0.0, std::floor(std::strtod(s.c_str(), nullptr) + 0.5)));
}

float number(const std::string& s) { return s == "NONE" ? 0.0f : std::strtof(s.c_str(), nullptr); }

}  // namespace

const char* gameModeName(int m) {
    switch (m) {
    case Race::GmTimeTrial: return "time trial";
    case Race::GmHelicopter: return "helicopter chase";
    case Race::GmCatchAI: return "hunt";
    case Race::GmRaid: return "raid";
    case Race::GmTimeBonus: return "time trial, checkpoint bonus";
    case Race::GmTimeLimitCheckpoints: return "checkpoints, time limit";
    case Race::GmKnockOut: return "knock out";
    case Race::GmCircuit: return "circuit";
    case Race::GmCheckpoint: return "checkpoint race";
    case Race::GmBaja: return "baja";
    case Race::GmAtoB: return "A to B";
    default: return "unknown";
    }
}

std::vector<Event> loadEvents(const std::string& dataDir) {
    const std::filesystem::path story = std::filesystem::path(dataDir) / "LocalizedComponent/GameTsc/Story";

    // Missions by upper-case name (FUEL matches with _stricmp). The level's mission list (0x4f40f0)
    // holds the EMD blocks first, then the world's MISSION_COURSE nodes (0x4f4db0); the name lookup
    // 0x4f8f90 returns the first match, so emplace keeps the earlier one.
    std::map<std::string, Race> missions;
    for (const char* file : {"miss_official.tsc", "missChallenge.tsc"})
        for (Race& r : parseMissions(readFile(story / file))) missions.emplace(upper(r.name), std::move(r));
    try {
        const DpcFile usa1((std::filesystem::path(dataDir) / "CommonComponent/WORLD/USA1.DPC").string());
        for (Race& r : loadWorldMissions(usa1)) missions.emplace(upper(r.name), std::move(r));
    } catch (const std::exception&) {
        // No world file: only the script missions exist.
    }

    struct Hub {
        std::string id;
        int tier = 0;
        std::vector<Event> career, challenges;
    };
    std::vector<Hub> hubs;
    std::map<std::string, Event*> byName;  // upper-case event id -> event (for the AddMission* lines)
    auto hubOf = [&](const std::string& name) -> Hub* {
        const std::string id = upper(name);
        for (Hub& h : hubs)
            if (h.id == id) return &h;
        return nullptr;
    };

    // The game's vehicle table (AddVehicleInfo, VehiclesDef.tsc order): the ids of AddMissionParam's AI list.
    std::vector<std::string> vehicleTable;
    try {
        std::istringstream vin(custom::readScript((story / "VehiclesDef.tsc").string()));  // CUSTOM VEHICLES: packs appended
        std::string vl;
        while (std::getline(vin, vl)) {
            const std::vector<std::string> t = tokens(vl);
            if (t.size() > 1 && consoleCommandIs(t[0], "AddVehicleInfo"))
                vehicleTable.push_back(t[1].compare(0, 8, "MESH_ID_") == 0 ? t[1].substr(8) : t[1]);
        }
    } catch (const std::exception&) {
    }

    std::istringstream in(readFile(story / "hubinfos.tsc"));
    std::string line;
    std::vector<std::vector<std::string>> lines;
    while (std::getline(in, line)) {
        std::vector<std::string> t = tokens(line);
        if (!t.empty()) lines.push_back(std::move(t));
    }
    // Pass 1: hubs and their event lists (pointers into the lists are taken after they are complete).
    for (const auto& t : lines) {
        if (consoleCommandIs(t[0], "SetHubTargetInfos") && t.size() > 3) {
            Hub h;
            h.id = upper(t[1]);
            h.tier = int(number(t[2]));
            hubs.push_back(h);
        } else if ((consoleCommandIs(t[0], "AddHubCareerRaces") || consoleCommandIs(t[0], "AddHubCHallengeRaces")) && t.size() > 3) {
            Hub* h = hubOf(t[1]);
            if (!h) continue;  // the game prints "Career Hub +%s+ non connue" and skips it
            Event e;
            e.id = t[2];
            e.kind = consoleCommandIs(t[0], "AddHubCareerRaces") ? Event::Career : Event::Challenge;
            e.hub = h->id;
            e.tier = h->tier;
            e.nameText = textId(t[3]);
            if (e.kind == Event::Challenge && t.size() > 4) e.descText = textId(t[4]);
            (e.kind == Event::Career ? h->career : h->challenges).push_back(e);
        }
    }
    for (Hub& h : hubs) {
        for (Event& e : h.career) byName[upper(e.id)] = &e;
        for (Event& e : h.challenges) byName[upper(e.id)] = &e;
    }
    // Pass 2: per-mission lines.
    for (const auto& t : lines) {
        if (t.size() < 2 || strncasecmp(t[0].c_str(), "AddMission", 10) != 0) continue;
        const auto it = byName.find(upper(t[1]));
        if (it == byName.end()) continue;
        Event& e = *it->second;
        auto arg = [&](size_t i) { return i < t.size() ? t[i] : std::string(); };
        if (consoleCommandIs(t[0], "AddMissionNeededVehicle")) {
            // "<forced MESH_ID_... | NONE> <surface | NONE> <family | NONE> NONE"
            const std::string forced = arg(2), surface = arg(3), family = arg(4);
            if (forced != "NONE" && forced.compare(0, 8, "MESH_ID_") == 0) {
                e.vehicle = forced.substr(8);  // e.g. BUGGY_OFF_3
                const size_t a = e.vehicle.find('_'), b = e.vehicle.rfind('_');
                e.vehicleFamily = e.vehicle.substr(0, a);
                if (a != std::string::npos && b > a) e.surface = e.vehicle.substr(a + 1, b - a - 1);
            }
            if (surface != "NONE" && !surface.empty()) e.surface = surface;
            if (family != "NONE" && !family.empty()) e.vehicleFamily = family;
        } else if (consoleCommandIs(t[0], "AddMissionSurfaceType")) {
            e.surfaceText = textId(arg(2));
            e.vehicleText = textId(arg(3));
        } else if (consoleCommandIs(t[0], "AddMissionTime")) {
            for (int i = 0; i < 3; ++i) e.times[i] = number(arg(size_t(2 + i)));
            // 0x5761e0: a 4th value other than NONE replaces the mission's timer (+0x48).
            if (arg(5) != "NONE" && !arg(5).empty()) e.race.timeLimit = number(arg(5));
        } else if (consoleCommandIs(t[0], "AddMissionParam")) {
            e.aiVehicles = arg(2);
            e.weatherStart = arg(5), e.weatherEnd = arg(7);
            e.tornado = strcasecmp(e.weatherStart.c_str(), "NONE") != 0 && storm::weatherHasTornado(e.weatherStart);
            e.aiVehicleLabels.clear();
            if (strcasecmp(e.aiVehicles.c_str(), "NONE") != 0) {
                std::istringstream ids(e.aiVehicles);
                std::string f;
                while (std::getline(ids, f, '_') && e.aiVehicleLabels.size() < 32) {
                    const long id = std::atol(f.c_str());
                    e.aiVehicleLabels.push_back(id >= 0 && size_t(id) < vehicleTable.size() ? vehicleTable[size_t(id)] : std::string());
                }
            }
        }
    }

    std::vector<Event> out;
    for (size_t hi = 0; hi < hubs.size(); ++hi) {
        Hub& h = hubs[hi];
        for (std::vector<Event>* list : {&h.career, &h.challenges})
            for (Event& e : *list) {
                e.hubIndex = int(hi);
                const float timerOverride = e.race.timeLimit;
                auto m = missions.find(upper(e.id));
                std::string note;
                if (m == missions.end()) {
                    // AddHubCareerRaces 0x575370 / AddHubCHallengeRaces 0x5755e0: the mission index by name
                    // (0x4f8f90) is < 0, so the handler returns without adding the race to the hub (p38). With
                    // the world's MISSION_COURSE missions (p42) every listed name in the shipped data matches.
                    continue;
                } else {
                    e.race = m->second;
                    // PK02: a vehicle name in EMD_SetPlayerForbiddenVehiculeFilter (0x5d6180: the table index of the name;
                    // no shipped script uses one). A name the table does not know ends the command in FUEL (returns 0).
                    for (const std::string& nm : e.race.forbiddenNames) {
                        const std::string bare = nm.compare(0, 8, "MESH_ID_") == 0 ? nm.substr(8) : nm;
                        for (size_t vi = 0; vi < vehicleTable.size(); ++vi)
                            if (strcasecmp(vehicleTable[vi].c_str(), bare.c_str()) == 0) {
                                e.race.forbidden.push_back(int(vi));
                                break;
                            }
                    }
                    e.race.forbiddenNames.clear();
                    // PK10: EMD_SetFilterIA (0x551530): integers as they are, a vehicle name by its table index; a name
                    // the table does not know ends the command there (the earlier entries stay).
                    for (const std::string& tk : e.race.aiFilterTokens) {
                        char* end = nullptr;
                        const long v = std::strtol(tk.c_str(), &end, 10);
                        if (!tk.empty() && *end == '\0') {
                            e.race.aiFilter.push_back(int(v));
                            continue;
                        }
                        const std::string bare = tk.compare(0, 8, "MESH_ID_") == 0 ? tk.substr(8) : tk;
                        bool found = false;
                        for (size_t vi = 0; vi < vehicleTable.size(); ++vi)
                            if (strcasecmp(vehicleTable[vi].c_str(), bare.c_str()) == 0) {
                                e.race.aiFilter.push_back(int(vi)), found = true;
                                break;
                            }
                        if (!found) break;
                    }
                    e.race.aiFilterTokens.clear();
                    // The race's AI list is the EMD_SetFilterIA one unless AddMissionParam gave a list (0x575b00 replaces it).
                    if (e.aiVehicleLabels.empty())
                        for (int id : e.race.aiFilter)
                            e.aiVehicleLabels.push_back(id >= 0 && size_t(id) < vehicleTable.size() ? vehicleTable[size_t(id)] : std::string());
                    if (timerOverride > 0) e.race.timeLimit = timerOverride;
                    // AddMissionPARAM 0x575b00: a vehicle list other than NONE ("41_10_20_...") becomes the
                    // mission's AI vehicles (+0xa8) and its AI count (+0x50 = their number), replacing
                    // EMD_SetNbIA. hubinfos.tsc runs after the mission build (level load 0x559290).
                    if (!e.aiVehicles.empty() && strcasecmp(e.aiVehicles.c_str(), "NONE") != 0)
                        e.race.aiCount = int(std::count(e.aiVehicles.begin(), e.aiVehicles.end(), '_')) + 1;
                    e.note = note;
                    const bool hasRoute = (e.race.start[0] != 0 || e.race.start[2] != 0) &&
                                          (!e.race.checkpoints.empty() || e.race.finish[0] != 0 || e.race.finish[2] != 0);
                    e.playable = hasRoute && e.race.gameMode() != Race::GmInvalid;
                    if (!hasRoute) e.note = "the EMD block has no start or no finish";
                    else if (!e.playable) e.note = "scenario " + std::to_string(e.race.scenario) + " has no game mode";
                }
                out.push_back(std::move(e));
            }
    }
    return out;
}

}  // namespace fuel
