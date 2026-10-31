// The event catalogue: hubinfos.tsc (hub order, career and challenge lists, needed vehicles,
// times) joined with the EMD blocks of miss_official.tsc and missChallenge.tsc.
// FUEL.exe joins them the same way: AddHubCareerRaces (0x575370) and AddHubCHallengeRaces
// (0x5755e0) look the mission up by name with _stricmp and silently skip names that match no
// mission. See ~/re-work/fuel/notes/p17-events.md (E1-E9).
#include "events.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <map>
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

    // Missions by upper-case name (FUEL matches with _stricmp).
    std::map<std::string, Race> missions;
    for (const char* file : {"miss_official.tsc", "missChallenge.tsc"})
        for (Race& r : parseMissions(readFile(story / file))) missions.emplace(upper(r.name), std::move(r));

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

    std::istringstream in(readFile(story / "hubinfos.tsc"));
    std::string line;
    std::vector<std::vector<std::string>> lines;
    while (std::getline(in, line)) {
        std::vector<std::string> t = tokens(line);
        if (!t.empty()) lines.push_back(std::move(t));
    }
    // Pass 1: hubs and their event lists (pointers into the lists are taken after they are complete).
    for (const auto& t : lines) {
        if (t[0] == "SetHubTargetInfos" && t.size() > 3) {
            Hub h;
            h.id = upper(t[1]);
            h.tier = int(number(t[2]));
            hubs.push_back(h);
        } else if ((t[0] == "AddHubCareerRaces" || t[0] == "AddHubCHallengeRaces") && t.size() > 3) {
            Hub* h = hubOf(t[1]);
            if (!h) continue;  // the game prints "Career Hub +%s+ non connue" and skips it
            Event e;
            e.id = t[2];
            e.kind = t[0] == "AddHubCareerRaces" ? Event::Career : Event::Challenge;
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
        if (t.size() < 2 || t[0].compare(0, 10, "AddMission") != 0) continue;
        const auto it = byName.find(upper(t[1]));
        if (it == byName.end()) continue;
        Event& e = *it->second;
        auto arg = [&](size_t i) { return i < t.size() ? t[i] : std::string(); };
        if (t[0] == "AddMissionNeededVehicle") {
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
        } else if (t[0] == "AddMissionSurfaceType") {
            e.surfaceText = textId(arg(2));
            e.vehicleText = textId(arg(3));
        } else if (t[0] == "AddMissionTime") {
            for (int i = 0; i < 3; ++i) e.times[i] = number(arg(size_t(2 + i)));
            // 0x5761e0: a 4th value other than NONE replaces the mission's timer (+0x48).
            if (arg(5) != "NONE" && !arg(5).empty()) e.race.timeLimit = number(arg(5));
        } else if (t[0] == "AddMissionParam") {
            e.aiVehicles = arg(2);
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
                if (m == missions.end() && e.kind == Event::Career) {
                    // CR42: the hub lists "CR42-Chec-CT" but the career script names its block "CR42-Chec-BJ".
                    // FUEL drops the event (no mission of that name); the port runs the block anyway.
                    const size_t dash = e.id.rfind('-');
                    for (const char* suffix : {"-BJ", "-CP", "-AB", "-RD", "-SR"}) {
                        const auto alt = missions.find(upper(e.id.substr(0, dash) + suffix));
                        if (dash != std::string::npos && alt != missions.end()) {
                            m = alt;
                            note = "port choice: FUEL's hub list names " + e.id + ", which matches no mission, so the game drops it; "
                                   "the port runs the " + alt->second.name + " block of miss_official.tsc";
                            break;
                        }
                    }
                }
                if (m == missions.end()) {
                    e.race.name = e.id;
                    e.playable = false;
                    e.note = "no EMD block in the mission scripts and no route anywhere in the game data; FUEL.exe drops it too "
                             "(AddHubCareerRaces 0x575370 skips names with no mission), so it is not in the shipped game";
                } else {
                    e.race = m->second;
                    if (timerOverride > 0) e.race.timeLimit = timerOverride;
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
