// FUEL's career, economy, unlocks, local achievements and save file. See career.hpp for
// the FUEL.exe addresses and ~/re-work/fuel/notes/p39-career.md for the decoding.
// Tags in comments: [O] observed in the code, [I] inferred, [U] unknown, [port] port choice.
#include "career.hpp"

#include "custom_vehicles.hpp"  // CUSTOM VEHICLES
#include "dpc.hpp"
#include "races.hpp"

#include "resources.hpp"  // asoboCrc (0x669160)
#include "save_import.hpp"  // PK25: saveCrc, result codes

#include <cerrno>
#include <fcntl.h>
#include <unistd.h>

#include <algorithm>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <strings.h>
#include <filesystem>
#include <fstream>
#include <map>
#include <set>
#include <sstream>

namespace fs = std::filesystem;

namespace fuel {

const char* const kTrophyVerbs[12] = {"NONE", "FIND", "OWN", "WON", "RIDE", "JUMP", "FALL", "TRICK", "CREATE", "OPEN", "GOTO", "USE"};
const char* const kTrophyObjects[19] = {"NONE", "VEHICLE", "STARS", "CHALLENGE", "VEHICLELIVERY", "PILOTLIVERY", "FREERIDE",
                                        "VISTAPOINT", "FUEL", "RACE", "RACEGOLD", "ALL_TYPE_VEHICLE", "HUB", "CAREER_NO_CRASH",
                                        "CAREER_NO_GPS", "RACE_ONLINE", "RACE_EDITED_BY_OTHER_ONLINE", "RACE_EDITED_BY_USER_ONLINE",
                                        "HELIPORT"};

namespace {

constexpr uint32_t kFuelCap = 99999999;  // 0x630dd0 / 0x630ca0 / 0x44e020 clamp [O]

std::string upper(std::string s) {
    for (char& c : s) c = char(std::toupper(static_cast<unsigned char>(c)));
    return s;
}

// Hub names compared the way findHub does (no case, no spaces or punctuation).
std::string squash(const std::string& s) {
    std::string o;
    for (char c : s)
        if (std::isalnum(static_cast<unsigned char>(c))) o += char(std::toupper(static_cast<unsigned char>(c)));
    return o;
}

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

std::vector<std::vector<std::string>> scriptLines(const fs::path& p) {
    // CUSTOM VEHICLES: VehiclesDef.tsc / VehiclesUnlock.tsc get the packs' lines appended (nothing for other scripts).
    std::istringstream f(custom::readScript(p.string()));
    std::vector<std::vector<std::string>> out;
    for (std::string line; std::getline(f, line);) {
        auto t = tokens(line);
        if (!t.empty()) out.push_back(std::move(t));
    }
    return out;
}

double num(const std::string& s) { return s.empty() || upper(s) == "NONE" ? 0.0 : std::strtod(s.c_str(), nullptr); }
// The console's int arguments: ftol of (value + 0.5) for text ids (0x575443) [O].
uint32_t textId(const std::string& s) { return s.empty() || upper(s) == "NONE" ? 0 : uint32_t(std::max(0.0, std::floor(num(s) + 0.5))); }

// AddVehicleInfo class (0x56a580) [O].
int vehicleClass(const std::string& f) {
    const std::string u = upper(f);
    if (u == "BUGGY") return 0;
    if (u == "BIKE") return 1;
    if (u == "MUSCLE") return 2;
    if (u == "QUAD") return 3;
    if (u == "SUV") return 4;
    if (u == "TRUCK") return 5;
    if (u == "HYDRO") return 6;
    if (u == "DRAGSTER") return 7;
    if (u == "UNKNOWN") return -1;
    return 2;  // the function's fall-through value [O]
}

// ---- FUEL's save buffer (0x565930..0x565ee0): sections "SECTION_<NAME>\0", padded with
// pos = (pos | 3) + 1, then a u32 body size and the body [O].
struct Writer {
    std::vector<uint8_t> b;
    size_t sizeAt = 0;
    void raw(const void* p, size_t n) { b.insert(b.end(), static_cast<const uint8_t*>(p), static_cast<const uint8_t*>(p) + n); }
    void u8(uint8_t v) { raw(&v, 1); }
    void u32(uint32_t v) { raw(&v, 4); }
    void f32(float v) { raw(&v, 4); }
    void begin(const char* name) {
        const std::string s = "SECTION_" + upper(name);
        raw(s.c_str(), s.size() + 1);
        b.resize((b.size() | 3) + 1, 0);
        sizeAt = b.size();
        u32(0);
    }
    void end() {
        const uint32_t n = uint32_t(b.size() - sizeAt - 4);
        std::memcpy(&b[sizeAt], &n, 4);
    }
};

struct Reader {
    const std::vector<uint8_t>& b;
    size_t pos = 0, end = 0;
    bool ok = true;
    // 0x565c40: find the section by name anywhere in the buffer.
    bool find(const char* name) {
        const std::string s = "SECTION_" + upper(name);
        for (size_t i = 0; i + s.size() + 1 <= b.size(); ++i)
            if (b[i] == 'S' && std::memcmp(&b[i], s.c_str(), s.size() + 1) == 0) {
                pos = ((i + s.size() + 1) | 3) + 1;
                uint32_t n = 0;
                if (pos + 4 > b.size()) return false;
                std::memcpy(&n, &b[pos], 4);
                pos += 4;
                end = std::min(b.size(), pos + n);
                ok = true;
                return true;
            }
        return false;
    }
    void raw(void* p, size_t n) {
        if (pos + n > end) { ok = false; std::memset(p, 0, n); return; }
        std::memcpy(p, &b[pos], n);
        pos += n;
    }
    uint8_t u8() { uint8_t v; raw(&v, 1); return v; }
    uint32_t u32() { uint32_t v; raw(&v, 4); return v; }
    float f32() { float v; raw(&v, 4); return v; }
};

constexpr char kMagic[8] = {'F', 'U', 'E', 'L', 'P', 'S', 'A', 'V'};
constexpr uint32_t kVersion = 1;

}  // namespace

// ---------------------------------------------------------------- rules

int CareerRules::missionIndex(const std::string& name) const {
    const std::string u = upper(name);
    for (size_t i = 0; i < missions.size(); ++i)
        if (upper(missions[i].name) == u) return int(i);
    return -1;
}

int CareerRules::hubIndex(const std::string& name) const {
    const std::string s = squash(name);
    for (size_t i = 0; i < hubs.size(); ++i)
        if (squash(hubs[i].name) == s) return int(i);
    return -1;
}

int CareerRules::vehicleIndex(const std::string& meshOrLabel) const {
    const std::string u = upper(meshOrLabel);
    for (size_t i = 0; i < vehicles.size(); ++i)
        if (upper(vehicles[i].mesh) == u || upper(vehicles[i].label) == u) return int(i);
    return -1;
}

CareerRules loadCareerRules(const std::string& dataDir) {
    CareerRules r;
    const fs::path story = fs::path(dataDir) / "LocalizedComponent/GameTsc/Story";

    // Missions: one per EMD block (LoadMissionsTsc.tsc runs miss_official then missChallenge) [O],
    // then the world's MISSION_COURSE nodes (0x4f40f0 -> 0x4f4db0, p42) [O].
    for (const char* file : {"miss_official.tsc", "missChallenge.tsc"})
        for (const auto& t : scriptLines(story / file))
            if (t.size() > 1 && upper(t[0]) == "EMD_SETNAME" && r.missionIndex(t[1]) < 0) r.missions.push_back({t[1]});
    try {
        const DpcFile usa1((fs::path(dataDir) / "CommonComponent/WORLD/USA1.DPC").string());
        for (const Race& m : loadWorldMissions(usa1))
            if (r.missionIndex(m.name) < 0) r.missions.push_back({m.name}), r.missions.back().world = true;
    } catch (const std::exception&) {
    }

    // hubinfos.tsc, in file order (SortHubListByUID keeps it: uids are given in parse order) [O].
    std::vector<std::vector<std::string>> liveryLines;
    for (const auto& t : scriptLines(story / "hubinfos.tsc")) {
        const std::string c = t[0];
        auto missionOf = [&](size_t k) { return k < t.size() ? r.missionIndex(t[k]) : -1; };
        if (c == "SetHubTargetInfos" && t.size() > 5) {
            CareerRules::Hub h;
            h.name = t[1];
            h.tier = int(num(t[2]));
            h.nameText = textId(t[3]);
            h.starsNeeded = int(num(t[5]));                   // -> hub +0x23c (0x578a20) [O]
            h.missionNeeded = t.size() > 6 ? t[6] : "NONE";   // -> hub +0x244 [O]
            r.hubs.push_back(h);
        } else if ((c == "AddHubCareerRaces" || c == "AddHubCHallengeRaces" || c == "AddHubTutorialRaces") && t.size() > 2) {
            const int hi = r.hubIndex(t[1]);
            const int mi = missionOf(2);
            // 0x575370 / 0x5755e0: unknown hub or no mission of that name -> not added [O]. With the
            // world's MISSION_COURSE missions (p42) all 72 listed career races exist (216 stars).
            if (hi < 0 || mi < 0) continue;
            auto& list = c == "AddHubCareerRaces" ? r.hubs[size_t(hi)].career : c == "AddHubCHallengeRaces" ? r.hubs[size_t(hi)].challenges
                                                                                                         : r.hubs[size_t(hi)].tutorial;
            list.push_back(mi);
        } else if (c == "AddMissionFuel" && t.size() > 4) {
            if (const int mi = missionOf(1); mi >= 0)
                for (int k = 0; k < 3; ++k) r.missions[size_t(mi)].fuel[k] = float(num(t[size_t(2 + k)]));  // 0x576420 [O]
        } else if (c == "AddMissionTime" && t.size() > 4) {
            if (const int mi = missionOf(1); mi >= 0)
                for (int k = 0; k < 3; ++k) r.missions[size_t(mi)].times[k] = float(num(t[size_t(2 + k)]));  // 0x5761e0 [O]
        } else if (c == "AddHubTrafficVehicle" && t.size() > 8) {
            if (const int hi = r.hubIndex(t[1]); hi >= 0) {
                CareerRules::Traffic tr{t[2], t[3], t[8]};
                tr.x0 = float(num(t[4])), tr.x1 = float(num(t[5])), tr.z0 = float(num(t[6])), tr.z1 = float(num(t[7]));
                r.hubs[size_t(hi)].traffic.push_back(tr);
            }
        } else if (c == "RemoveOldVehicleLiveries") {
            for (auto& h : r.hubs) h.liveries.clear();
        } else if (c == "AddVehicleLivery" && t.size() > 5) {
            liveryLines.push_back(t);
        } else if (c == "RemoveOldPilotLiveries") {
            r.pilotLiveries.clear();
        } else if (c == "AddPilotLivery" && t.size() > 4) {
            CareerRules::PilotLivery p;
            p.part = upper(t[1]);
            p.index = int(num(t[2]));
            p.text = textId(t[3]);
            p.unlock = t[4];
            if (t.size() > 5) p.code = t[5];
            r.pilotLiveries.push_back(p);
        }
    }
    for (const auto& t : liveryLines) {
        const int hi = r.hubIndex(t[3]);
        if (hi < 0) continue;
        CareerRules::Livery l;
        l.mesh = t[1];
        l.index = int(num(t[2]));
        l.text = textId(t[4]);
        l.kind = upper(t[5]);
        if (t.size() > 6) l.arg = t[6];
        if (t.size() > 8) l.x = float(num(t[7])), l.z = float(num(t[8]));
        if (l.kind == "POI") r.poiScript.push_back({2, hi, int(r.hubs[size_t(hi)].liveries.size())});  // PK09: 0x575855
        r.hubs[size_t(hi)].liveries.push_back(l);
    }
    // Vista points per hub (poi.tsc "VP", the hub's +0x234 list).
    // P44: with their positions (0x54f760: x = arg 3, z = arg 4, name arg 5, hub arg 6; VP +0x48 = floor(arg 2 + 0.5)).
    for (const auto& t : scriptLines(story / "poi.tsc"))
        if (t.size() > 6 && t[0] == "AddTypePointOfInterest") {
            const int hi = r.hubIndex(t[6]);
            if (hi < 0) continue;
            auto& hub = r.hubs[size_t(hi)];
            CareerRules::Hub::Point pt;
            pt.x = float(num(t[3])), pt.z = float(num(t[4])), pt.name = t[5];
            if (upper(t[1]) == "VP") {
                pt.text = uint32_t(std::floor(num(t[2]) + 0.5));
                r.poiScript.push_back({3, hi, int(hub.vistaPoints.size())});  // PK09
                hub.vistaPoints.push_back(pt);
                ++hub.vistas;
            } else if (upper(t[1]) == "TELEPORT") {
                r.poiScript.push_back({4, hi, int(hub.teleports.size())});  // PK09
                hub.teleports.push_back(pt);
            }
        }

    // Vehicles (VehiclesDef.tsc): AddVehicleInfo MESH FAMILY SURFACE n NAME text tier GPS; AddVehicleUnlock MESH price race code.
    for (const auto& t : scriptLines(story / "VehiclesDef.tsc")) {
        if (t[0] == "AddVehicleInfo" && t.size() > 6) {
            CareerRules::Vehicle v;
            v.mesh = t[1];
            v.label = t[1].rfind("MESH_ID_", 0) == 0 ? t[1].substr(8) : t[1];
            v.family = upper(t[2]);
            v.cls = vehicleClass(t[2]);
            v.terrain = strcasecmp(t[3].c_str(), "OFF") == 0 ? 0 : strcasecmp(t[3].c_str(), "ROAD") == 0 ? 1 : 2;
            v.nameText = textId(t[6]);
            v.level = t.size() > 7 ? int(std::strtol(t[7].c_str(), nullptr, 10)) : -1;  // [upgrades U2] the price tier
            v.custom = custom::isCustom(v.label);  // CUSTOM VEHICLES
            r.vehicles.push_back(v);
        } else if (t[0] == "AddVehicleUnlock" && t.size() > 2) {
            const int vi = r.vehicleIndex(t[1]);
            if (vi < 0) continue;
            CareerRules::Vehicle& v = r.vehicles[size_t(vi)];
            v.price = int(std::strtol(t[2].c_str(), nullptr, 10));  // atoi -> +0x34 [O]
            if (t.size() > 3 && upper(t[3]) != "NONE") v.raceUnlock = t[3];
            if (t.size() > 4 && upper(t[4]) != "NONE") v.code = t[4];
        }
    }
    for (const auto& t : scriptLines(story / "VehiclesUnlock.tsc"))
        if (t[0] == "UnlockNeed" && t.size() > 3) r.unlocks.push_back({upper(t[1]), t[2], upper(t[3]), t.size() > 4 ? t[4] : ""});
    for (const auto& t : scriptLines(story / "Events.tsc"))
        if (t[0] == "UnlockNeed" && t.size() > 3) r.unlocks.push_back({upper(t[1]), t[2], upper(t[3]), t.size() > 4 ? t[4] : ""});

    // Trophy.tsc: AddNewTROPHY id grade credits Show|Hide b b; AddConditionTROPHY id VERB OBJECT "a" "b".
    for (const auto& t : scriptLines(story / "Trophy.tsc")) {
        if (t[0] == "AddNewTROPHY" && t.size() > 4) {
            CareerRules::Trophy tr;
            tr.id = int(num(t[1]));
            const std::string g = upper(t[2]);
            tr.grade = g == "BRONZE" ? 1 : g == "SILVER" ? 2 : g == "GOLD" ? 3 : 0;  // 0x56c0f0 [O]
            tr.credits = int(num(t[3]));
            tr.hidden = upper(t[4]) != "SHOW";
            r.trophies.push_back(tr);
        } else if (t[0] == "AddConditionTROPHY" && t.size() > 3) {
            const int id = int(std::floor(num(t[1]) + 0.5));  // + 0.5 (0x9960c0) then ftol [O]
            for (auto& tr : r.trophies)
                if (tr.id == id) {
                    for (int k = 0; k < 12; ++k)
                        if (upper(t[2]) == kTrophyVerbs[k]) tr.verb = k;
                    for (int k = 0; k < 19; ++k)
                        if (upper(t[3]) == kTrophyObjects[k]) tr.object = k;
                    tr.arg1 = t.size() > 4 ? t[4] : "";
                    tr.arg2 = t.size() > 5 ? t[5] : "";
                    tr.value = num(tr.arg1);
                }
        }
    }
    return r;
}

// ---------------------------------------------------------------- state

Career::Career(CareerRules rules) : rules_(std::move(rules)) { newGame(); }

void Career::newGame() {
    state_ = CareerState{};
    CareerState& s = state_;
    {  // ---- PK27: 0x553c80(1), the options block's factory values (options[i] = options-struct +0xc0 + i)
        const float one = 1.0f;
        const int32_t eight = 8, two = 2;
        std::memcpy(s.options + 0x00, &one, 4);  // +0xc0 brightness (World_SetBrightness)
        for (size_t o : {0x08u, 0x0cu, 0x10u, 0x14u, 0x18u}) std::memcpy(s.options + o, &eight, 4);  // +0xc8 .. +0xd8
        s.options[0x24] = 1, s.options[0x25] = 0, s.options[0x26] = 1;                               // +0xe4, +0xe5, +0xe6
        std::memcpy(s.options + 0x28, &two, 4);                                                      // +0xe8
        s.options[0x2d] = s.options[0x2e] = s.options[0x2f] = s.options[0x30] = 1;                   // +0xed .. +0xf0
    }  // ---- PK27 end
    s.currentHub() = 0;
    s.currentVehicle() = -1;  // 0x5539f0 picks the default vehicle (0x56a7d0) when < 0 [O]
    for (const auto& v : rules_.vehicles) s.vehicles.push_back({asoboCrc(v.mesh)});
    s.upgrades.resize(s.vehicles.size());  // [upgrades U2] all stock
    s.dev.resize(s.vehicles.size());       // [dev mode] no record
    const size_t H = rules_.hubs.size();
    s.hubs.resize(H);
    s.vistas.resize(H), s.traffic.resize(H), s.liveries.resize(H);
    s.tutorial.resize(H), s.career.resize(H), s.challenges.resize(H);
    for (size_t h = 0; h < H; ++h) {
        const auto& rh = rules_.hubs[h];
        s.hubs[h].unlocked = rh.starsNeeded < 1;  // 0x578a20: needed < 1 -> +0x249 = 1 [O]
        s.vistas[h].resize(size_t(rh.vistas));
        s.traffic[h].resize(rh.traffic.size());
        s.liveries[h].resize(rh.liveries.size());
        for (const auto& t : rh.traffic) s.trafficCrc.push_back(asoboCrc(t.mesh));     // entry +0x18 [I: which argument]
        for (size_t k = 0; k < rh.liveries.size(); ++k) {
            // P66: entry +4 = crc("%s_%s" mesh, index) (AddVehicleLivery 0x5756e0, matched by 0x569db0) [O]. ALWAYS
            // liveries (kind 0) are owned with their vehicle (0x569b20(0) from 0x569e60 / 0x569eb0), below.
            s.liveryCrc.push_back(asoboCrc(rh.liveries[k].mesh + "_" + std::to_string(rh.liveries[k].index)));
        }
        s.tutorial[h].resize(rh.tutorial.size());
        s.career[h].resize(rh.career.size());
        s.challenges[h].resize(rh.challenges.size());
    }
    s.pilotOwned.resize(rules_.pilotLiveries.size());
    for (size_t k = 0; k < rules_.pilotLiveries.size(); ++k)
        s.pilotOwned[k] = upper(rules_.pilotLiveries[k].unlock) == "ALWAYS";  // 0x56b920: ALWAYS -> entry +0 = 1 [O]
    // P66: EndOfParsePilotLivery 0x56bc00: one pair per part type name (entry +0x10 = crc("HEAD1")) in script order,
    // appended by 0x56be40 as {crc, -1}; the second word is the type's chosen texture livery (0x4721f0 / 0x471e80).
    for (const auto& pl : rules_.pilotLiveries) {
        const uint32_t c = asoboCrc(pl.part);
        bool have = false;
        for (const auto& pr : s.pilotPairs) have = have || pr.first == c;
        if (!have) s.pilotPairs.push_back({c, 0xffffffffu});
    }
    unlockDone_.assign(rules_.unlocks.size(), 0);
    checkUnlocks();  // Events.tsc: CheckUnLock
    for (size_t v = 0; v < s.vehicles.size(); ++v)
        if (s.vehicles[v].owned) ownAlwaysLiveries(v);
}

// ---- PK27: 0x554130 (from the options apply 0x553c80(0) after a load): +0xd0 = n clamped to 0..16, vt+0x48(n / 16).
int Career::voiceLevel() const {
    int32_t n = 0;
    std::memcpy(&n, state_.options + 0x10, 4);
    return std::clamp(n, 0, 16);
}
// ---- PK27 end

// P66: 0x569b20(kind 0), EBX = the vehicle: every livery entry of every hub with entry +8 == the vehicle's mesh crc
// and kind +0x20 == 0 (ALWAYS) gets revealed (+0) and owned (+1). Called when a vehicle is unlocked as a gift
// (0x569e60), owned (0x569eb0 / 0x569ed0 / 0x569f60), bought (0x4ce590 / 0x479590) and on load (0x5681e0) [O].
void Career::ownAlwaysLiveries(size_t v) {
    if (v >= rules_.vehicles.size()) return;
    const std::string& mesh = rules_.vehicles[v].mesh;
    for (size_t h = 0; h < rules_.hubs.size() && h < state_.liveries.size(); ++h)
        for (size_t k = 0; k < rules_.hubs[h].liveries.size() && k < state_.liveries[h].size(); ++k)
            if (rules_.hubs[h].liveries[k].kind == "ALWAYS" && upper(rules_.hubs[h].liveries[k].mesh) == upper(mesh))
                state_.liveries[h][k] = {1, 1};
}

bool Career::locate(const std::string& event, int* hub, int* list, int* index) const {
    const int mi = rules_.missionIndex(event);
    if (mi < 0) return false;
    for (size_t h = 0; h < rules_.hubs.size(); ++h) {
        const auto& rh = rules_.hubs[h];
        for (int l = 0; l < 3; ++l) {
            const auto& v = l == 0 ? rh.tutorial : l == 1 ? rh.career : rh.challenges;
            for (size_t k = 0; k < v.size(); ++k)
                if (v[k] == mi) {
                    if (hub) *hub = int(h);
                    if (list) *list = l;
                    if (index) *index = int(k);
                    return true;
                }
        }
    }
    return false;
}

MissionStatus* Career::status(const std::string& event) {
    int h, l, k;
    if (!locate(event, &h, &l, &k)) return nullptr;
    auto& v = l == 0 ? state_.tutorial : l == 1 ? state_.career : state_.challenges;
    return &v[size_t(h)][size_t(k)];
}

const MissionStatus* Career::status(const std::string& event) const { return const_cast<Career*>(this)->status(event); }

bool Career::eventAvailable(const std::string& event) const {
    const MissionStatus* m = status(event);
    return m && m->available;
}

int Career::stars() const {
    int n = 0;
    for (const auto& hub : state_.career)
        for (const auto& m : hub) n += m.bestRank == 0 ? 3 : m.bestRank == 1 ? 2 : m.bestRank == 2 ? 1 : 0;
    return n;
}

int Career::challengesWon() const {
    int n = 0;
    for (const auto& hub : state_.challenges)
        for (const auto& m : hub) n += m.bestRank != 3;
    return n;
}

bool Career::hubUnlockTest(int hub) const {
    const auto& rh = rules_.hubs[size_t(hub)];
    if (rh.starsNeeded < 0) return false;
    if (rh.starsNeeded == 0) return true;
    if (stars() < rh.starsNeeded) return false;
    if (upper(rh.missionNeeded) == "NONE" || rh.missionNeeded.empty()) return true;
    const MissionStatus* m = status(rh.missionNeeded);
    return m == nullptr || m->bestRank != 3;  // index < 0 -> no mission needed [O]
}

std::vector<std::string> Career::newHubUnlocks() {
    std::vector<std::string> out;
    for (size_t h = 0; h < rules_.hubs.size(); ++h)
        if (!state_.hubs[h].unlocked && hubUnlockTest(int(h))) {
            state_.hubs[h].unlocked = 1;
            out.push_back(rules_.hubs[h].name);
        }
    return out;
}

bool Career::enterHub(int hub) {
    if (hub < 0 || size_t(hub) >= rules_.hubs.size() || !state_.hubs[size_t(hub)].unlocked) return false;
    state_.hubs[size_t(hub)].opened = 1;                                   // +0x248 [O]
    for (auto& m : state_.career[size_t(hub)]) m.available = 1;           // 0x57a100 [O]
    auto& ch = state_.challenges[size_t(hub)];
    for (size_t k = 0; k < std::min<size_t>(3, ch.size()); ++k) ch[k].available = 1;  // 0x57a240 (3) [O]
    return true;
}

// ---------------------------------------------------------------- free roam (P44)

void Career::setCurrentHub(int hub) {
    if (hub < 0 || size_t(hub) >= rules_.hubs.size()) return;
    state_.currentHub() = hub;  // 0x5693b0: game+0x33c0 = uid (= index: uids follow the parse order) [O]
    // 0x56ca20 on the trophies' history: 8 entries, the oldest dropped; the open trip is closed at `hub`
    // (or kept when it is the same hub) and a new one opens from it [O].
    auto& v = state_.trips;
    if (v.size() == 8) v.erase(v.begin());
    if (!v.empty()) {
        if (v.back().from == hub) {
            v.back().valid = true;
            return;
        }
        v.back().to = hub;
    }
    v.push_back({hub, -1, true});
}

void Career::useHeliport() {
    if (!state_.trips.empty()) state_.trips.back().valid = false;  // 0x56ca00 [O]
    ++state_.heliportUses();                                         // game+0x3460 (0x482b60) [O]
}

bool Career::travelled(int from, int to) const {
    if (from < 0 || to < 0) return false;
    for (const auto& t : state_.trips)  // 0x56cb20 [O]
        if (t.valid && t.from == from && t.to == to) return true;
    return false;
}

void Career::addDistance(float metres, bool mission, bool offRoad) {
    // 0x569460: metres into the f32, whole kilometres carried into the u32 when it passes 1000 [O].
    auto add = [&](uint32_t& km, float& m) {
        m += metres;
        if (m > 1000.0f) {
            const uint32_t k = uint32_t(m / 1000.0f);  // ftol, truncating [O: 0x8e01d0]
            km += k;
            m = m - float(k) * 1000.0f;
        }
    };
    if (mission) add(state_.raceKm(), state_.raceM());
    else add(state_.freeRideKm(), state_.freeRideM());
    if (offRoad) add(state_.offRoadKm(), state_.offRoadM());
    else add(state_.onRoadKm(), state_.onRoadM());
}

double Career::freeRideDistance() const { return double(state_.get<uint32_t>(0x50)) * 1000.0 + state_.get<float>(0x54); }
double Career::raceDistance() const { return double(state_.get<uint32_t>(0x58)) * 1000.0 + state_.get<float>(0x5c); }

void Career::raceStart() { state_.raceCrashed = 0, state_.raceNoGps = 1, state_.race3b8 = 0; }  // 0x62d74c [O]

void Career::raceFrame(bool wrecked, bool gpsShown) {
    if (wrecked) state_.raceCrashed = 1;   // player phys +0x1420 (0x632b50) [O]
    if (gpsShown) state_.raceNoGps = 0;    // [0xa6536e] == 0: the GPS is on [O]
}

uint32_t Career::collectFuel(float amount) {
    // 0x51d600 for the player: fuel += trunc(amount) (fistp under RC_CHOP) capped, barrels by amount [O].
    const uint32_t a = uint32_t(std::max(0.0f, std::trunc(amount)));
    state_.fuel = uint32_t(std::min<uint64_t>(uint64_t(state_.fuel) + a, kFuelCap));
    if (a == 100) state_.fuelFound += 1;
    else if (a == 400) state_.fuelFound += 4;
    return a;
}

bool Career::visitVista(int hub, int k) {
    if (hub < 0 || size_t(hub) >= state_.vistas.size() || k < 0 || size_t(k) >= state_.vistas[size_t(hub)].size()) return false;
    auto& v = state_.vistas[size_t(hub)][size_t(k)];
    if (v.b) return false;
    v.a = v.b = 1;  // 0x4ed0a0: visited (+1) and revealed (+0) [O]
    return true;
}

bool Career::revealVista(int hub, int k) {
    if (hub < 0 || size_t(hub) >= state_.vistas.size() || k < 0 || size_t(k) >= state_.vistas[size_t(hub)].size()) return false;
    auto& v = state_.vistas[size_t(hub)][size_t(k)];
    if (v.a) return false;
    v.a = 1;  // 0x4c01d0 [O]
    return true;
}

bool Career::revealLivery(int hub, int k) {
    if (hub < 0 || size_t(hub) >= state_.liveries.size() || k < 0 || size_t(k) >= state_.liveries[size_t(hub)].size()) return false;
    auto& v = state_.liveries[size_t(hub)][size_t(k)];
    if (v.a) return false;
    v.a = 1;  // 0x4c00c0 [O]
    return true;
}

bool Career::pickLivery(int hub, int k) {
    if (hub < 0 || size_t(hub) >= state_.liveries.size() || k < 0 || size_t(k) >= state_.liveries[size_t(hub)].size()) return false;
    auto& v = state_.liveries[size_t(hub)][size_t(k)];
    if (v.b) return false;
    v.a = v.b = 1;  // 0x4fcb60: owned (+1), revealed (+0) [O]
    return true;
}

bool Career::discoverChallenge(int hub, int k) {
    if (hub < 0 || size_t(hub) >= state_.challenges.size() || k < 0 || size_t(k) >= state_.challenges[size_t(hub)].size()) return false;
    auto& m = state_.challenges[size_t(hub)][size_t(k)];
    if (m.available) return false;
    m.available = 1;  // 0x4c0150 [O]
    return true;
}

int CareerRules::Traffic::kind() const {  // 0x574fb0 [O]: _stricmp Radar_VistaPoint 0, Radar_VehicleLivery 1, Radar_Challenge 2, Vehicle 3
    const std::string u = upper(radar);
    if (u == "RADAR_VISTAPOINT") return 0;
    if (u == "RADAR_VEHICLELIVERY") return 1;
    if (u == "RADAR_CHALLENGE") return 2;
    return 3;  // "Vehicle" (the data's only other word)
}

bool Career::trafficCaught(int hub, int k) const {
    return hub >= 0 && size_t(hub) < state_.traffic.size() && k >= 0 && size_t(k) < state_.traffic[size_t(hub)].size() &&
           state_.traffic[size_t(hub)][size_t(k)].b;
}

int Career::catchTraffic(int hub, int k) {  // 0x5c53b0 [O]
    if (hub < 0 || size_t(hub) >= state_.traffic.size() || k < 0 || size_t(k) >= state_.traffic[size_t(hub)].size()) return 0;
    auto& t = state_.traffic[size_t(hub)][size_t(k)];
    if (t.b) return 0;
    t.b = 1, t.a = 1;
    const auto& rh = rules_.hubs[size_t(hub)];
    switch (rh.traffic[size_t(k)].kind()) {
    case 0:  // 0x57a060: hub +0x238[i] = 1 for every vista
        for (auto& v : state_.vistas[size_t(hub)]) v.a = 1;
        return 10;
    case 1:  // 0x57a390: hub +0x228[i] (0x24 each) = 1 for every vehicle livery
        for (auto& v : state_.liveries[size_t(hub)]) v.a = 1;
        return 9;
    case 2:  // 0x57a1d0: every challenge of the hub +0x10d = 1
        for (auto& m : state_.challenges[size_t(hub)]) m.available = 1;
        return 11;
    default: {  // 0x477f40 (vehicle record by entry +0x18) owned = unlocked = 1
        const uint32_t crc = asoboCrc(rh.traffic[size_t(k)].mesh);
        for (size_t v = 0; v < state_.vehicles.size(); ++v)
            if (state_.vehicles[v].crc == crc) state_.vehicles[v].owned = 1, state_.vehicles[v].unlocked = 1, ownAlwaysLiveries(v);  // 0x569eb0
        return 8;
    }
    }
}

int Career::starsMax() const {
    int n = 0;
    for (const auto& h : rules_.hubs) n += 3 * int(h.career.size());  // 0x578070 [O]
    return n;
}

void Career::unlockVehicle(size_t v) {
    // 0x569e60(1): unlocked; a vehicle that costs nothing is owned at once [O].
    auto& s = state_.vehicles[v];
    if (s.unlocked) return;
    s.unlocked = 1;
    if (rules_.vehicles[v].price < 1) s.owned = 1, ownAlwaysLiveries(v);
}

bool Career::conditionMet(const CareerRules::Unlock& u) const {
    // 0x564560 kinds: 0 MISSION (+0x120 != 3), 1 VEHICLE (+0 unlocked), 2 HUB (0x579720), 8 ALWAYS [O].
    if (u.cond == "ALWAYS") return true;
    if (u.cond == "HUB") {
        const int h = rules_.hubIndex(u.arg);
        return h >= 0 && hubUnlockTest(h);
    }
    if (u.cond == "MISSION") {
        const MissionStatus* m = status(u.arg);
        return m && m->bestRank != 3;
    }
    if (u.cond == "VEHICLE") {
        const int v = rules_.vehicleIndex(u.arg);
        return v >= 0 && state_.vehicles[size_t(v)].unlocked;
    }
    return false;  // TT / RTC / LEVEL / others: not used by the shipped scripts
}

std::string Career::applyUnlock(const CareerRules::Unlock& u) {
    // 0x564a70: 0 MISSION -> +0x10d = 1, 1 VEHICLE -> 0x569e60(1) [O].
    if (u.kind == "VEHICLE") {
        const int v = rules_.vehicleIndex(u.target);
        if (v >= 0 && !state_.vehicles[size_t(v)].unlocked) {
            unlockVehicle(size_t(v));
            return rules_.vehicles[size_t(v)].label;
        }
    } else if (u.kind == "MISSION") {
        if (MissionStatus* m = status(u.target); m && !m->available) {
            m->available = 1;
            return u.target;
        }
    }
    return "";
}

std::vector<std::string> Career::checkUnlocks() {
    // 0x5641c0: repeat until nothing changes; a rule fires when its condition holds and no
    // other rule for the same target is unmet; ALWAYS rules (kind 8) are never marked done [O].
    std::vector<std::string> out;
    unlockDone_.resize(rules_.unlocks.size(), 0);
    for (bool changed = true; changed;) {
        changed = false;
        for (size_t i = 0; i < rules_.unlocks.size(); ++i) {
            const auto& u = rules_.unlocks[i];
            if (unlockDone_[i] || !conditionMet(u)) continue;
            bool blocked = false;
            for (size_t j = 0; j < rules_.unlocks.size(); ++j)
                if (j != i && rules_.unlocks[j].kind == u.kind && upper(rules_.unlocks[j].target) == upper(u.target) &&
                    !conditionMet(rules_.unlocks[j]))
                    blocked = true;
            if (blocked) continue;
            if (u.cond != "ALWAYS") unlockDone_[i] = 1, changed = true;
            const std::string t = applyUnlock(u);
            if (!t.empty()) out.push_back(t);
        }
    }
    return out;
}

CareerAward Career::finishEvent(const std::string& event, bool won, int rank, int place, double seconds) {
    CareerAward a;
    int h, l, k;
    if (!locate(event, &h, &l, &k)) return a;  // not in FUEL's hub lists: no progress
    MissionStatus& m = (l == 0 ? state_.tutorial : l == 1 ? state_.career : state_.challenges)[size_t(h)][size_t(k)];
    for (int i = 0; i < 3; ++i) a.medalHad[i] = m.medal[i];  // the +0x134 snapshot
    // P66: SetState 0x632c10, states 6 / 7 (won or lost), offline: game +0x3440 (races finished) + 1 and the mission's
    // +0x10c = 1 ("played": the profile's career races played, 0x577f10) [O].
    m.flag10c = 1;
    state_.at<int32_t>(0x88) += 1;
    if (!won) return a;  // MISSION_ENDING_LOST (state 7): nothing changes, no save [O]
    a.counted = true;
    const auto& mission = rules_.missions[size_t(rules_.missionIndex(event))];
    const uint32_t crc = asoboCrc(mission.name);
    uint32_t& fuel = state_.fuel;
    auto addFuel = [&](float f) {
        const uint32_t add = uint32_t(std::lround(f));  // ROUND (fistp) [O]
        a.fuelWon += add;
        fuel = uint32_t(std::min<uint64_t>(uint64_t(fuel) + add, kFuelCap));
    };

    // Rewards keyed by the race's name (0x56a320 vehicles, 0x56b830 pilot liveries,
    // 0x574cb0 traffic vehicles of kinds 0..3, first match of each kind) [O].
    for (size_t v = 0; v < rules_.vehicles.size(); ++v)
        if (!state_.vehicles[v].owned && !rules_.vehicles[v].raceUnlock.empty() && asoboCrc(rules_.vehicles[v].raceUnlock) == crc) {
            state_.vehicles[v].unlocked = 1, state_.vehicles[v].owned = 1, ownAlwaysLiveries(v);  // 0x569ed0
            a.vehiclesUnlocked.push_back(rules_.vehicles[v].label);
            a.giftVehicle = rules_.vehicles[v].label;
            break;
        }
    for (size_t p = 0; p < rules_.pilotLiveries.size(); ++p)
        if (!state_.pilotOwned[p] && upper(rules_.pilotLiveries[p].unlock) != "ALWAYS" && upper(rules_.pilotLiveries[p].unlock) != "NONE" &&
            asoboCrc(rules_.pilotLiveries[p].unlock) == crc) {
            state_.pilotOwned[p] = 1;
            a.pilotLiveries.push_back(rules_.pilotLiveries[p].part + " " + std::to_string(rules_.pilotLiveries[p].index));
        }
    if (l == 1 && !state_.race3b8) {  // 0x630dfc: race +0x3b8 == 0 -> the no-crash / no-GPS flags [O]
        if (!state_.raceCrashed) state_.noCrashWin() = 1;
        if (state_.raceNoGps) state_.noGpsWin() = 1;
    }
    if (l == 1) {
        // 0x574cb0(kind) for kinds 1, 0, 2, 3 [O]: in every hub, the first entry of that kind (+4: AddHubTrafficVehicle
        // 0x574fb0 Radar_VistaPoint 0, Radar_VehicleLivery 1, Radar_Challenge 2, Vehicle 3) not revealed with +0x2c ==
        // crc(race) is revealed. P99: kind 3 is the word "Vehicle" (the port compared "RADAR_VEHICLE": never revealed).
        for (int kind : {1, 0, 2, 3})
            for (size_t hh = 0; hh < rules_.hubs.size(); ++hh)
                for (size_t t = 0; t < rules_.hubs[hh].traffic.size(); ++t) {
                    const auto& tr = rules_.hubs[hh].traffic[t];
                    if (tr.kind() == kind && !state_.traffic[hh][t].a && asoboCrc(tr.mission) == crc) {
                        state_.traffic[hh][t].a = 1;
                        a.trafficRevealed.push_back(tr.radar + " in " + rules_.hubs[hh].name);
                        break;
                    }
                }
    }

    if (l == 1) {
        // Career race (0x630dd0) [O].
        m.lastRank = rank;
        if (rank < m.bestRank) {
            m.bestRank = rank;
            a.hubsUnlocked = newHubUnlocks();  // 0x578aa0
            if (!a.hubsUnlocked.empty()) ++state_.hubsUnlockedByRaces();
            for (int i = std::max(0, rank); i < 3; ++i)
                if (!m.medal[i]) {
                    m.medal[i] = 1;
                    addFuel(mission.fuel[i]);
                }
        }
    } else if (l == 2) {
        // Challenge (0x630ca0) [O].
        if (m.bestRank > 0) {
            m.bestRank = 0;
            if (!m.medal[0]) {
                m.medal[0] = 1;
                addFuel(mission.fuel[0]);
            }
        }
    }
    // Common bookkeeping in 0x6300c0 [O].
    const uint32_t score = 0x7fffffffu - uint32_t(std::lround(std::max(0.0, seconds) * 100.0));
    m.bestScore = std::max(m.bestScore, score);
    m.lastScore = score;
    if (place < int(m.bestPlace)) m.bestPlace = uint8_t(std::clamp(place, 0, 255));
    m.lastPlace = uint8_t(std::clamp(place, 0, 255));

    for (int i = 0; i < 3; ++i) {
        a.medalNew[i] = m.medal[i] && !a.medalHad[i];
        a.newMedals += a.medalNew[i];
    }
    for (const std::string& v : checkUnlocks()) a.vehiclesUnlocked.push_back(v);
    a.trophies = checkTrophies();
    return a;
}

bool Career::vehicleOwned(const std::string& v) const {
    const int i = rules_.vehicleIndex(v);
    return i >= 0 && (state_.vehicles[size_t(i)].owned || devOn(i));  // [dev mode] the overlay: owned
}

bool Career::vehicleUnlocked(const std::string& v) const {
    const int i = rules_.vehicleIndex(v);
    return i >= 0 && (state_.vehicles[size_t(i)].unlocked || devOn(i));  // [dev mode] the overlay: unlocked
}

// ---- [DEVIATION dev mode] (notes devmode.md; career.hpp) the overlay's tests and its store.
bool Career::devOn(int i) const {
    return devMode_ && i >= 0 && size_t(i) < rules_.vehicles.size() && rules_.vehicles[size_t(i)].cls >= 0;
}

bool Career::devLook(int i) const { return devOn(i) && size_t(i) < state_.vehicles.size() && !state_.vehicles[size_t(i)].owned; }

bool Career::devOverlay(const std::string& v) const { return devOn(rules_.vehicleIndex(v)); }

bool Career::devStages(const std::string& v) const {
    const CareerState::DevVehicle* d = devAt(rules_.vehicleIndex(v));
    return d && (d->flags & CareerState::kDevStages);
}

CareerState::DevVehicle* Career::devAt(int i) {
    if (i < 0 || size_t(i) >= state_.vehicles.size()) return nullptr;
    if (state_.dev.size() < state_.vehicles.size()) state_.dev.resize(state_.vehicles.size());
    return &state_.dev[size_t(i)];
}

const CareerState::DevVehicle* Career::devAt(int i) const {
    return i >= 0 && size_t(i) < state_.dev.size() && size_t(i) < state_.vehicles.size() ? &state_.dev[size_t(i)] : nullptr;
}
// ---- end dev mode

int Career::vehiclePrice(const std::string& v) const {
    const int i = rules_.vehicleIndex(v);
    return i >= 0 ? rules_.vehicles[size_t(i)].price : -1;
}

Career::BuyResult Career::buyVehicle(const std::string& v, std::vector<int>* trophies) {
    const int i = rules_.vehicleIndex(v);
    if (i < 0) return NotForSale;
    auto& s = state_.vehicles[size_t(i)];
    const int price = rules_.vehicles[size_t(i)].price;
    if (s.owned || devOn(i)) return AlreadyOwned;  // [dev mode] owned in the overlay: nothing bought, nothing spent
    if (!s.unlocked) return Locked;        // InfoBoxGarageVehicleIsLocked
    if (price < 0) return NotForSale;
    if (!spendFuel(uint32_t(price))) return TooExpensive;  // InfoBoxGarageVehicleIsTooExpensive; 0x4ce590 [O]
    s.owned = 1;
    ownAlwaysLiveries(size_t(i));          // 0x569eb0 -> 0x569b20(0) [O]
    state_.currentVehicle() = i;           // +0x33c8 = the bought vehicle (0x56a040) [O]
    const auto t = checkTrophies();        // 0x56cdd0 [O]
    if (trophies) *trophies = t;
    return Bought;
}

// 0x4ce590 (and the compare before it, 0x560a90 [I]): the purchase only when the price fits the fuel, then
// game +0x408 -= price [O]. [upgrades U2] shared by the vehicle and the upgrade purchases.
bool Career::spendFuel(uint32_t price) {
    if (price > state_.fuel) return false;
    state_.fuel -= price;
    return true;
}

// ---- garage (P66) ----

std::vector<int> Career::vehicleLiveryChoices(const std::string& v) const {
    // 0x47aa40: for i < 0x569c00 (entries whose +8 == the mesh crc), entry 0x569db0(i) = the one with +4 ==
    // crc(mesh "_" i+1); when owned (+1) three carousel entries 3i .. 3i+2 ("_LIV%d" of 3i+1 ..) [O].
    std::vector<int> out;
    const int vi = rules_.vehicleIndex(v);
    if (vi < 0) return out;
    const std::string mesh = upper(rules_.vehicles[size_t(vi)].mesh);
    int count = 0;
    for (const auto& h : rules_.hubs)
        for (const auto& l : h.liveries) count += upper(l.mesh) == mesh;
    auto entryOwned = [&](int n) {  // 0x569db0: the first entry (hub order) with that crc
        for (size_t h = 0; h < rules_.hubs.size(); ++h)
            for (size_t k = 0; k < rules_.hubs[h].liveries.size(); ++k) {
                const auto& l = rules_.hubs[h].liveries[k];
                if (upper(l.mesh) == mesh && l.index == n)  // [dev mode] the overlay owns the vehicle: its ALWAYS liveries come with it (0x569b20(0))
                    return (h < state_.liveries.size() && k < state_.liveries[h].size() && state_.liveries[h][k].b != 0) || (devOn(vi) && l.kind == "ALWAYS");
            }
        return false;
    };
    for (int i = 0; i < count; ++i)
        if (entryOwned(i + 1))
            for (int j = 0; j < 3; ++j) out.push_back(3 * i + j);
    return out;
}

// [dev mode] a vehicle not really owned keeps the livery / colour chosen for it in dev mode in the dev store (FUEL's record
// +0xc / +0xd stays as the real career has it); a really owned one uses its record as always.
int Career::vehicleLivery(const std::string& v) const {
    const int i = rules_.vehicleIndex(v);
    if (i < 0) return 0;
    if (const CareerState::DevVehicle* d = devAt(i); d && devLook(i) && (d->flags & CareerState::kDevLook)) return d->livery;
    return state_.vehicles[size_t(i)].livery;
}

int Career::vehicleColor(const std::string& v) const {
    const int i = rules_.vehicleIndex(v);
    if (i < 0) return 0;
    if (const CareerState::DevVehicle* d = devAt(i); d && devLook(i) && (d->flags & CareerState::kDevLook)) return d->color;
    return state_.vehicles[size_t(i)].color;
}

void Career::setVehicleLivery(const std::string& v, int k) {  // 0x4796e0: +0xc = entry +0x18 [O]
    const int i = rules_.vehicleIndex(v);
    if (i < 0) return;
    if (devLook(i)) {  // [dev mode]
        CareerState::DevVehicle* d = devAt(i);
        if (!(d->flags & CareerState::kDevLook)) d->color = state_.vehicles[size_t(i)].color;
        d->flags |= CareerState::kDevLook, d->livery = uint8_t(k);
        return;
    }
    state_.vehicles[size_t(i)].livery = uint8_t(k);
}

void Career::setVehicleColor(const std::string& v, int c) {  // 0x479a60: +0xd = the colour index [O]
    const int i = rules_.vehicleIndex(v);
    if (i < 0) return;
    if (devLook(i)) {  // [dev mode]
        CareerState::DevVehicle* d = devAt(i);
        if (!(d->flags & CareerState::kDevLook)) d->livery = state_.vehicles[size_t(i)].livery;
        d->flags |= CareerState::kDevLook, d->color = uint8_t(c);
        return;
    }
    state_.vehicles[size_t(i)].color = uint8_t(c);
}

void Career::addVehicleTime(const std::string& v, float seconds) {
    const int i = rules_.vehicleIndex(v);
    if (i < 0) return;
    if (devLook(i)) return;  // [dev mode] a vehicle the real career does not own: its record +0x10 stays as it is
    float& t = state_.vehicles[size_t(i)].f10;
    if (!(t >= 0)) t = 0;
    t += seconds;
}

bool Career::pilotTypeOwned(const std::string& type) const {  // 0x4740f0: entries +0x10 == crc(type) with +0 set [O]
    for (size_t k = 0; k < rules_.pilotLiveries.size(); ++k)
        if (upper(rules_.pilotLiveries[k].part) == upper(type) && k < state_.pilotOwned.size() && state_.pilotOwned[k]) return true;
    return false;
}

const CareerRules::PilotLivery* Career::pilotLivery(const std::string& type, int n, bool* owned) const {
    // 0x474320: the first entry with +0x10 == crc(type) and +0x14 (index) == n [O].
    for (size_t k = 0; k < rules_.pilotLiveries.size(); ++k)
        if (upper(rules_.pilotLiveries[k].part) == upper(type) && rules_.pilotLiveries[k].index == n) {
            if (owned) *owned = k < state_.pilotOwned.size() && state_.pilotOwned[k];
            return &rules_.pilotLiveries[k];
        }
    if (owned) *owned = false;
    return nullptr;
}

int Career::pilotTypeLivery(const std::string& type) const {
    // 0x471e80: the pair's value; < 0 -> 0x56b740 (1-based ordinal of the first owned entry of the type, counting
    // every entry of the type; -1 when none) x 3 - 2 [O].
    const uint32_t c = asoboCrc(upper(type));
    for (const auto& pr : state_.pilotPairs)
        if (pr.first == c && int32_t(pr.second) >= 0) return int(pr.second);
    int ord = 1;
    for (size_t k = 0; k < rules_.pilotLiveries.size(); ++k)
        if (upper(rules_.pilotLiveries[k].part) == upper(type)) {
            if (k < state_.pilotOwned.size() && state_.pilotOwned[k]) return ord * 3 - 2;
            ++ord;
        }
    return -5;  // 0x56b740 = -1
}

void Career::setPilotTypeLivery(const std::string& type, int k) {  // 0x4721f0 [O]
    const uint32_t c = asoboCrc(upper(type));
    for (auto& pr : state_.pilotPairs)
        if (pr.first == c) pr.second = uint32_t(k);
}

PilotLook Career::pilotLook(bool tutorialMode) const {
    // 0x42b420 [O]: each game field < 1 takes its default.
    PilotLook p;
    for (int k = 0; k < 8; ++k) p.at(k) = state_.get<int32_t>(0x18 + 4 * size_t(k));
    if (p.head < 1) {  // 0x56b610: the first n (1..255) whose HEAD<n> has an owned livery
        p.head = -1;
        for (int n = 1; n < 256 && p.head < 0; ++n)
            if (pilotTypeOwned("HEAD" + std::to_string(n))) p.head = n;
    }
    if (p.headLivery < 1) p.headLivery = pilotTypeLivery("HEAD" + std::to_string(p.head));  // 0x56b6f0 x 3 - 2
    if (p.chest < 1) p.chest = tutorialMode ? 4 : 1;
    if (p.chestLivery < 1) p.chestLivery = 1;
    if (p.legs < 1) p.legs = tutorialMode ? 3 : 1;
    if (p.legsLivery < 1) p.legsLivery = 1;
    if (p.skin < 1) p.skin = 1;
    if (p.bodyLivery < 1) p.bodyLivery = 1;
    return p;
}

void Career::setPilotLook(const PilotLook& p) {
    for (int k = 0; k < 8; ++k) state_.pilot(k) = p.at(k);
}

int Career::ownedVehicles() const {
    int n = 0;
    for (size_t v = 0; v < rules_.vehicles.size(); ++v)  // 0x56a180: owned, no code, a class [O]
        n += state_.vehicles[v].owned && rules_.vehicles[v].code.empty() && rules_.vehicles[v].cls != -1 && !rules_.vehicles[v].custom;
    return n;
}

int Career::ownedVehicleLiveries() const {
    int n = 0;
    for (const auto& hub : state_.liveries)
        for (const auto& l : hub) n += l.b != 0;
    return n;
}

int Career::ownedPilotLiveries() const {
    int n = 0;
    for (uint8_t o : state_.pilotOwned) n += o != 0;
    return n;
}

int Career::vistasFound() const {
    int n = 0;
    for (const auto& hub : state_.vistas)
        for (const auto& v : hub) n += v.b != 0;
    return n;
}

bool Career::trophyMet(const CareerRules::Trophy& t, std::string* progress) const {
    const int v = int(t.value);
    auto count = [&](int have, int need) {
        if (progress) *progress = std::to_string(have) + " / " + std::to_string(need);
        return have >= need;
    };
    switch (t.verb) {
        case 1:  // FIND
            if (t.object == 7) return count(vistasFound(), v);
            if (t.object == 8) return count(int(state_.fuelFound), v);
            return false;
        case 2:  // OWN
            if (t.object == 1) return count(ownedVehicles(), v);
            if (t.object == 4) return count(ownedVehicleLiveries(), v);
            if (t.object == 5) return count(ownedPilotLiveries(), v);
            if (t.object == 11) {  // one of each class 0..5; 6 and 7 count as owned [O]
                bool seen[8] = {false, false, false, false, false, false, true, true};
                for (size_t i = 0; i < rules_.vehicles.size(); ++i)
                    if (state_.vehicles[i].owned && !rules_.vehicles[i].custom && rules_.vehicles[i].cls >= 0 && rules_.vehicles[i].cls < 8)
                        seen[rules_.vehicles[i].cls] = true;
                return count(int(std::count(seen, seen + 8, true)), 8);
            }
            return false;
        case 3:  // WON
            if (t.object == 2) return count(stars(), v);
            if (t.object == 3) return count(challengesWon(), v);
            if (t.object == 9) { const MissionStatus* m = status(t.arg1); return m && m->bestRank != 3; }
            if (t.object == 10) { const MissionStatus* m = status(t.arg1); return m && m->medal[0]; }
            if (t.object == 13) return state_.stats[0x79] != 0;
            if (t.object == 14) return state_.stats[0x84] != 0;
            return false;  // online counts: out of scope
        case 4: {  // RIDE: 0x49ad40 (free ride) for FREERIDE, + 0x49ad20 (race) for NONE; f >= value [O]
            const double d = t.object == 6 ? freeRideDistance() : t.object == 0 ? freeRideDistance() + raceDistance() : -1.0;
            if (progress) *progress = std::to_string(int64_t(d)) + " / " + std::to_string(int64_t(t.value));
            return d >= 0 && float(d) >= float(t.value);
        }
        case 5: { float f; std::memcpy(&f, state_.stats + 0x70, 4); return f >= float(t.value); }
        case 6: { float f; std::memcpy(&f, state_.stats + 0x74, 4); return f >= float(t.value); }
        case 7: { int32_t n; std::memcpy(&n, state_.stats + 0x7c, 4); return count(n, v); }
        case 8: { int32_t n; std::memcpy(&n, state_.stats + 0x80, 4); return t.object == 9 && count(n, v); }
        case 9:  // OPEN HUB
            if (t.object == 12) { const int h = rules_.hubIndex(t.arg1); return h >= 0 && hubUnlockTest(h); }
            return false;
        case 10:  // GOTO HUB a b: the hub-to-hub travel history (0x56cb20) [O]
            return t.object == 12 && travelled(rules_.hubIndex(t.arg1), rules_.hubIndex(t.arg2));
        case 11: { int32_t n; std::memcpy(&n, state_.stats + 0xa8, 4); return t.object == 18 && count(n, v); }
        default: return false;
    }
}

std::vector<int> Career::checkTrophies() {
    std::vector<int> out;
    for (const auto& t : rules_.trophies) {
        if (std::find(state_.trophiesWon.begin(), state_.trophiesWon.end(), t.id) != state_.trophiesWon.end()) continue;
        if (trophyMet(t)) {
            state_.trophiesWon.push_back(t.id);
            out.push_back(t.id);
        }
    }
    return out;
}

// ---------------------------------------------------------------- save file

std::string Career::careerPath() {
    if (const char* p = std::getenv("FUEL_CAREER_SAVE"); p && *p) return p;  // tests and shots
    const char* home = std::getenv("HOME");
    return (fs::path(home ? home : ".") / "Library/Application Support/FUEL Port/career.sav").string();
}

std::vector<uint8_t> Career::serialize() const {
    Writer w;
    // Header [port]: FUEL PC wraps the packed buffer in an XLive-protected container
    // (SaveGamePc_Z 0x840000); the port keeps the section stream plain.
    w.raw(kMagic, 8);
    w.u32(kVersion);
    const std::vector<uint8_t> p = sections();
    w.u32(uint32_t(p.size()));
    w.raw(p.data(), p.size());
    // PK25: the body's integrity mark of 0x4ebea0 after the payload: '=' (body[0x30d3b]) and the save CRC of every byte
    // before it (0x30d3c; core/save_import.hpp saveCrc). Readers before PK25 ignore bytes after the payload.
    w.u8(0x3d);
    const uint32_t crc = pcsave::saveCrc(w.b.data(), w.b.size());
    w.u32(crc);
    return w.b;
}

std::vector<uint8_t> Career::sections() const {
    const CareerState& s = state_;
    Writer p;  // FUEL's buffer starts at 0 (section padding is relative to it)
    // Order of 0x566050 [O].
    p.begin("SAVESTRUCT_INFOS");  // 0x566110
    p.raw(s.profile[0], 33), p.raw(s.profile[1], 33);
    p.u32(s.fuel), p.u32(s.fuelFound);
    {  // CUSTOM VEHICLES: FUEL's +0x33c8 never holds a port vehicle's index (-1 = FUEL's default; PORT_VEHICLES has it)
        uint8_t stats[sizeof s.stats];
        std::memcpy(stats, s.stats, sizeof stats);
        const int32_t cur = s.get<int32_t>(0x10);
        if (cur >= 0 && size_t(cur) < rules_.vehicles.size() && rules_.vehicles[size_t(cur)].custom) {
            const int32_t none = -1;
            std::memcpy(stats + 0x10, &none, 4);
        }
        p.raw(stats, sizeof stats);
    }
    p.raw(s.options, sizeof s.options);
    p.u32(s.options2a0);
    p.end();
    p.begin("VEHICLES_INFOS");  // 0x566c40 (CUSTOM VEHICLES: FUEL's vehicles only)
    {
        auto fuelOwn = [&](size_t i) { return i >= rules_.vehicles.size() || !rules_.vehicles[i].custom; };
        uint32_t n = 0;
        for (size_t i = 0; i < s.vehicles.size(); ++i) n += fuelOwn(i);
        p.u32(n);
        for (size_t i = 0; i < s.vehicles.size(); ++i)
            if (fuelOwn(i)) {
                const auto& v = s.vehicles[i];
                p.u32(v.crc), p.u8(v.unlocked), p.u8(v.owned), p.u8(v.livery), p.u8(v.color), p.f32(v.f10);
            }
    }
    p.end();
    p.begin("INPUT"), p.end();     // 0x567940: key mapping [port: the port keeps its own keys]
    p.begin("TUTORIAL");  // 0x567a90 -> 0x56e140 [O]: u32 n, n x {u32 index (entry +4), u8 seen (+0), u8 off (+1)}
    p.u32(uint32_t(s.tutorials.size()));
    for (size_t k = 0; k < s.tutorials.size(); ++k) p.u32(uint32_t(k)), p.u8(s.tutorials[k].seen), p.u8(s.tutorials[k].off);
    p.end();
    p.begin("HUB_INFOS");  // 0x566200
    p.u32(uint32_t(s.hubs.size()));
    for (const auto& h : s.hubs) p.u8(h.opened), p.u8(h.unlocked);
    p.end();
    auto perHub = [&](const char* name, const std::vector<std::vector<CareerState::Flags2>>& v, const std::vector<uint32_t>* crcs) {
        p.begin(name);
        p.u32(uint32_t(v.size()));
        size_t flat = 0;
        for (const auto& hub : v) {
            p.u32(uint32_t(hub.size()));
            for (const auto& e : hub) {
                if (crcs) p.u32(flat < crcs->size() ? (*crcs)[flat] : 0);
                ++flat;
                p.u8(e.a), p.u8(e.b);
            }
        }
        p.end();
    };
    perHub("VISTAPOINT_INFOS", s.vistas, nullptr);  // 0x5663e0
    p.begin("EDITEDRACE_INFOS");  // 0x566680: 0x2dc8 bytes [O]
    p.raw(s.editedRaces.data(), s.editedRaces.size());
    p.end();
    p.begin("MISSION_INFOS");  // 0x5666f0
    p.u32(uint32_t(s.career.size()));
    for (size_t h = 0; h < s.career.size(); ++h) {
        p.u32(uint32_t(h));
        for (const auto* list : {&s.tutorial[h], &s.career[h], &s.challenges[h]}) {
            p.u32(uint32_t(list->size()));
            for (const MissionStatus& m : *list) p.raw(&m, sizeof m);
        }
    }
    p.end();
    perHub("TRAFFIC_INFOS", s.traffic, &s.trafficCrc);             // 0x567000
    perHub("VEHICLES_LIVERIES_INFOS", s.liveries, &s.liveryCrc);   // 0x567320
    p.begin("PILOT_LIVERIES_INFOS");  // 0x567640
    p.u32(uint32_t(s.pilotOwned.size()));
    for (uint8_t o : s.pilotOwned) p.u8(o);
    p.u32(uint32_t(s.pilotPairs.size()));
    for (const auto& pr : s.pilotPairs) p.u32(pr.first), p.u32(pr.second);
    p.end();
    // [port] achievements awarded locally (FUEL left them to Games for Windows - LIVE).
    p.begin("PORT_TROPHIES");
    p.u32(uint32_t(s.trophiesWon.size()));
    for (int id : s.trophiesWon) p.u32(uint32_t(id));
    p.end();
    // [port, P44] the trophies' hub travel history (FUEL keeps it in memory for the session; the port's sessions
    // change hub by restarting the process): u32 n, n x {i32 from, i32 to, u8 valid}.
    p.begin("PORT_TRAVEL");
    p.u32(uint32_t(s.trips.size()));
    for (const auto& t : s.trips) p.u32(uint32_t(t.from)), p.u32(uint32_t(t.to)), p.u8(t.valid);
    p.end();
    // [port, PHYSIQUE] the player's outfit: u32 version 1, u8 outfit, u8 helmet.
    p.begin("PORT_WARDROBE");
    p.u32(1), p.u8(s.riderOutfit), p.u8(s.riderHelmet);
    p.end();
    // CUSTOM VEHICLES [port]: the port packs' vehicles (core/custom_vehicles.hpp), matched by crc(mesh) on load like
    // VEHICLES_INFOS: u32 version 1, u32 n, n x {u32 crc, u8 unlocked, u8 owned, u8 livery, u8 color, f32 +0x10} (FUEL's record), then the
    // current vehicle's crc (0 = none: SAVESTRUCT's +0x33c8 holds FUEL's choice). Written only when there are packs.
    {
        std::vector<size_t> own;
        for (size_t i = 0; i < s.vehicles.size() && i < rules_.vehicles.size(); ++i)
            if (rules_.vehicles[i].custom) own.push_back(i);
        if (!own.empty()) {
            p.begin("PORT_VEHICLES");
            p.u32(1), p.u32(uint32_t(own.size()));
            for (size_t i : own) {
                const auto& v = s.vehicles[i];
                p.u32(v.crc), p.u8(v.unlocked), p.u8(v.owned), p.u8(v.livery), p.u8(v.color), p.f32(v.f10);
            }
            const int32_t cur = s.get<int32_t>(0x10);
            p.u32(cur >= 0 && size_t(cur) < rules_.vehicles.size() && rules_.vehicles[size_t(cur)].custom ? s.vehicles[size_t(cur)].crc : 0u);
            p.end();
        }
    }
    // [DEVIATION upgrades U2] vehicle upgrades (notes upgrades-plan.md 6, upg2.md), the last section: u32 format 1, u32 flags 0,
    // u32 n, n x {u32 crc(mesh) (VEHICLES_INFOS' key; the packs' too), u8 owned[6], u8 equipped[6], u32 fuel spent} (20 bytes,
    // categories in upg::Category order e, s, t, w, np, nt; append-only). Only vehicles with something bought or equipped.
    // FUEL's sections before it are byte for byte what they were; the PK25 '=' + CRC trailer covers it with the rest.
    p.begin("PORT_UPGRADES");
    {
        std::vector<size_t> rec;
        for (size_t i = 0; i < s.vehicles.size() && i < s.upgrades.size(); ++i)
            if (s.upgrades[i].any()) rec.push_back(i);
        p.u32(1), p.u32(0), p.u32(uint32_t(rec.size()));
        for (size_t i : rec) {
            const auto& u = s.upgrades[i];
            p.u32(s.vehicles[i].crc);
            p.raw(u.owned, sizeof u.owned), p.raw(u.equipped, sizeof u.equipped);
            p.u32(u.spent);
        }
    }
    p.end();
    // [DEVIATION dev mode] (notes devmode.md) the overlay's own store, after PORT_UPGRADES and only when it holds a record (a
    // career never put in dev mode saves exactly as before): u32 format 1, u32 flags 0, u32 n, n x {u32 crc(mesh), u8 flags
    // (1 stages, 2 livery / colour), u8 livery, u8 color, u8 0, u8 equipped[6] (upg::Category order), u8 0, u8 0} (16 bytes).
    // FUEL's sections and PORT_UPGRADES never hold what dev mode chose; the PK25 trailer's CRC covers this section too.
    {
        std::vector<size_t> rec;
        for (size_t i = 0; i < s.vehicles.size() && i < s.dev.size(); ++i)
            if (s.dev[i].flags & (CareerState::kDevStages | CareerState::kDevLook)) rec.push_back(i);
        if (!rec.empty()) {
            p.begin("PORT_DEVMODE");
            p.u32(1), p.u32(0), p.u32(uint32_t(rec.size()));
            for (size_t i : rec) {
                const auto& d = s.dev[i];
                p.u32(s.vehicles[i].crc), p.u8(d.flags & (CareerState::kDevStages | CareerState::kDevLook)), p.u8(d.livery), p.u8(d.color), p.u8(0);
                p.raw(d.equipped, sizeof d.equipped), p.u8(0), p.u8(0);
            }
            p.end();
        }
    }
    return p.b;
}

bool Career::deserializeSections(const std::vector<uint8_t>& stream, std::string* error) {
    Writer w;
    w.raw(kMagic, 8);
    w.u32(kVersion);
    w.u32(uint32_t(stream.size()));
    w.raw(stream.data(), stream.size());
    return deserialize(w.b, error);
}

bool Career::deserialize(const std::vector<uint8_t>& bytes, std::string* error) {
    auto fail = [&](const std::string& e) {
        if (error) *error = e;
        return false;
    };
    if (bytes.size() < 16 || std::memcmp(bytes.data(), kMagic, 8) != 0) return fail("not a FUEL Port career save");
    uint32_t ver = 0, size = 0;
    std::memcpy(&ver, &bytes[8], 4);
    std::memcpy(&size, &bytes[12], 4);
    if (ver != kVersion) return fail("save version " + std::to_string(ver));
    if (16 + size_t(size) > bytes.size()) return fail("truncated save");
    // PK25 0x4eb9c0: the '=' marker and the CRC after the payload (saves before PK25 end at the payload).
    if (bytes.size() > 16 + size_t(size)) {
        const size_t at = 16 + size_t(size);
        uint32_t crc = 0;
        if (bytes.size() != at + 5 || bytes[at] != 0x3d) return fail("save marker missing (corrupt)");
        std::memcpy(&crc, &bytes[at + 1], 4);
        if (crc != pcsave::saveCrc(bytes.data(), at + 1)) return fail("save CRC mismatch (corrupt)");
    }
    const std::vector<uint8_t> payload(bytes.begin() + 16, bytes.begin() + 16 + size);

    newGame();  // missing sections keep the script defaults
    CareerState& s = state_;
    std::vector<std::string> notes;
    Reader r{payload};
    if (r.find("SAVESTRUCT_INFOS")) {  // 0x567c00
        r.raw(s.profile[0], 33), r.raw(s.profile[1], 33);
        s.fuel = std::min(r.u32(), kFuelCap), s.fuelFound = r.u32();
        r.raw(s.stats, sizeof s.stats);
        r.raw(s.options, sizeof s.options);
        s.options2a0 = r.u32();
    }
    if (r.find("VEHICLES_INFOS")) {  // 0x5681e0: matched by mesh crc (+0x30) [O]
        const uint32_t n = r.u32();
        for (uint32_t i = 0; i < n && r.ok; ++i) {
            CareerState::Vehicle v;
            v.crc = r.u32(), v.unlocked = r.u8(), v.owned = r.u8(), v.livery = r.u8(), v.color = r.u8(), v.f10 = r.f32();
            for (auto& sv : s.vehicles)
                if (sv.crc == v.crc) sv = v;
        }
    }
    if (r.find("TUTORIAL")) {  // 0x56e0d0 [O]: entry k gets {index, seen, off}; 0x56d900 then refills its content by index
        const uint32_t n = r.u32();
        for (uint32_t k = 0; k < n && r.ok; ++k) {
            const uint32_t idx = r.u32();
            const uint8_t seen = r.u8(), off = r.u8();
            if (r.ok && idx < s.tutorials.size()) s.tutorials[idx] = {seen, off};
        }
    }
    if (r.find("HUB_INFOS")) {  // 0x567d60
        const uint32_t n = r.u32();
        for (uint32_t h = 0; h < n && r.ok; ++h) {
            const uint8_t a = r.u8(), b = r.u8();
            if (h < s.hubs.size()) s.hubs[h] = {a, b};
        }
    }
    auto perHub = [&](const char* name, std::vector<std::vector<CareerState::Flags2>>& v, std::vector<uint32_t>* crcs, bool revealIfOwned) {
        if (!r.find(name)) return;
        const uint32_t n = r.u32();
        for (uint32_t h = 0; h < n && r.ok; ++h) {
            const uint32_t c = r.u32();
            for (uint32_t k = 0; k < c && r.ok; ++k) {
                if (crcs) r.u32();
                CareerState::Flags2 f{r.u8(), r.u8()};
                if (revealIfOwned && f.b) f.a = 1;  // 0x567e00 / 0x568380 / 0x568470 [O]
                if (h < v.size() && k < v[h].size()) v[h][k] = f;
            }
        }
    };
    perHub("VISTAPOINT_INFOS", s.vistas, nullptr, true);
    if (r.find("EDITEDRACE_INFOS")) r.raw(s.editedRaces.data(), s.editedRaces.size());
    if (r.find("MISSION_INFOS")) {  // 0x567ff0: per hub its index, then the three lists [O]
        const uint32_t n = r.u32();
        for (uint32_t h = 0; h < n && r.ok; ++h) {
            const uint32_t idx = r.u32();
            if (idx != h) { notes.push_back("mission hub index"); break; }
            for (int l = 0; l < 3; ++l) {
                auto* list = l == 0 ? &s.tutorial : l == 1 ? &s.career : &s.challenges;
                const uint32_t c = r.u32();
                // FUEL stops reading on a count mismatch ("Nb de mission ... differents") [O]; the port keeps what fits.
                // P99: saves written before P42 (2026-11-03) listed only the EMD missions (0x575370 dropped the world's
                // MISSION_COURSE ones: 57 career races); their statuses go to the hub's EMD entries, in list order.
                std::vector<size_t> slot;
                if (h < list->size()) {
                    const auto& ids = l == 0 ? rules_.hubs[h].tutorial : l == 1 ? rules_.hubs[h].career : rules_.hubs[h].challenges;
                    for (size_t k = 0; k < ids.size(); ++k) slot.push_back(k);
                    if (c != slot.size()) {
                        std::vector<size_t> emd;
                        for (size_t k = 0; k < ids.size(); ++k)
                            if (!rules_.missions[size_t(ids[k])].world) emd.push_back(k);
                        if (c == emd.size()) slot = emd;
                        else notes.push_back("mission count differs in hub " + std::to_string(h));
                    }
                }
                for (uint32_t k = 0; k < c && r.ok; ++k) {
                    MissionStatus m;
                    r.raw(&m, sizeof m);
                    if (h < list->size() && k < slot.size()) (*list)[h][slot[k]] = m;
                }
            }
        }
    }
    perHub("TRAFFIC_INFOS", s.traffic, &s.trafficCrc, true);
    perHub("VEHICLES_LIVERIES_INFOS", s.liveries, &s.liveryCrc, true);
    if (r.find("PILOT_LIVERIES_INFOS")) {  // 0x568620
        const uint32_t n = r.u32();
        for (uint32_t k = 0; k < n && r.ok; ++k) {
            const uint8_t o = r.u8();
            if (k < s.pilotOwned.size()) s.pilotOwned[k] = o;
        }
        // P66: {crc(part type), chosen texture livery}; matched by crc into the script's list [port: saves before P66
        // wrote an empty list, which keeps the defaults (-1)].
        const uint32_t m = r.u32();
        for (uint32_t k = 0; k < m && r.ok; ++k) {
            const uint32_t a = r.u32(), b = r.u32();
            bool found = false;
            for (auto& pr : s.pilotPairs)
                if (pr.first == a) pr.second = b, found = true;
            if (!found && r.ok) s.pilotPairs.push_back({a, b});
        }
    }
    s.trophiesWon.clear();
    if (r.find("PORT_TROPHIES")) {
        const uint32_t n = r.u32();
        for (uint32_t k = 0; k < n && r.ok; ++k) s.trophiesWon.push_back(int(r.u32()));
    }
    if (r.find("PORT_VEHICLES")) {  // CUSTOM VEHICLES: unknown crcs (a pack removed) are dropped
        if (r.u32() == 1) {
            const uint32_t n = r.u32();
            for (uint32_t i = 0; i < n && r.ok; ++i) {
                CareerState::Vehicle v;
                v.crc = r.u32(), v.unlocked = r.u8(), v.owned = r.u8(), v.livery = r.u8(), v.color = r.u8(), v.f10 = r.f32();
                for (size_t k = 0; k < s.vehicles.size() && k < rules_.vehicles.size() && r.ok; ++k)
                    if (s.vehicles[k].crc == v.crc && rules_.vehicles[k].custom) s.vehicles[k] = v;
            }
            const uint32_t cur = r.u32();
            for (size_t k = 0; k < s.vehicles.size() && k < rules_.vehicles.size() && r.ok && cur != 0; ++k)
                if (s.vehicles[k].crc == cur && rules_.vehicles[k].custom) s.currentVehicle() = int32_t(k);
        }
    }
    s.trips.clear();
    if (r.find("PORT_TRAVEL")) {
        const uint32_t n = r.u32();
        for (uint32_t k = 0; k < n && k < 8 && r.ok; ++k) {
            CareerState::Trip t;
            t.from = int32_t(r.u32()), t.to = int32_t(r.u32()), t.valid = r.u8() != 0;
            s.trips.push_back(t);
        }
    }
    if (r.find("PORT_WARDROBE")) {  // PHYSIQUE
        if (r.u32() == 1) {
            const uint8_t o = r.u8(), h = r.u8();
            if (r.ok && o <= 3) s.riderOutfit = o, s.riderHelmet = h ? 1 : 0;
        }
    }
    // [DEVIATION upgrades U2] PORT_UPGRADES (see sections()). No section (older port builds, PC imports): all stock (newGame's).
    // A body of the wrong size or format: all stock and a note, not the corrupt-save alert (the trailer's CRC already
    // catches damage). Unknown crcs (a pack removed) and never-upgraded vehicles are dropped with a note; stages clamp to
    // 0..3, equipped <= owned; the spent fuel clamps to FUEL's 99,999,999.
    s.upgrades.assign(s.vehicles.size(), {});
    if (r.find("PORT_UPGRADES")) {
        const size_t body = r.end - r.pos;
        const uint32_t format = r.u32(), flags = r.u32(), n = r.u32();
        (void)flags;  // reserved (0)
        constexpr size_t kRecord = 4 + 2 * upg::kCategories + 4;
        if (!r.ok || format != 1 || body != 12 + size_t(n) * kRecord) {
            notes.push_back("PORT_UPGRADES: format " + std::to_string(format) + ", " + std::to_string(body) + " bytes for " + std::to_string(n) +
                            " vehicles: upgrades read as stock");
        } else {
            std::vector<CareerState::VehicleUpgrades> got(s.vehicles.size());
            int dropped = 0;
            for (uint32_t k = 0; k < n && r.ok; ++k) {
                const uint32_t crc = r.u32();
                CareerState::VehicleUpgrades u;
                r.raw(u.owned, sizeof u.owned), r.raw(u.equipped, sizeof u.equipped);
                u.spent = std::min(r.u32(), kFuelCap);
                for (int c = 0; c < upg::kCategories; ++c) {
                    u.owned[c] = uint8_t(std::min<int>(u.owned[c], upg::kMaxStage));
                    u.equipped[c] = std::min(u.equipped[c], u.owned[c]);
                }
                size_t at = s.vehicles.size();
                for (size_t i = 0; i < s.vehicles.size() && i < rules_.vehicles.size(); ++i)
                    if (s.vehicles[i].crc == crc) at = i;
                if (at == s.vehicles.size() || rules_.vehicles[at].cls < 0) ++dropped;
                else got[at] = u;
            }
            if (r.ok) s.upgrades = std::move(got);
            if (dropped) notes.push_back("PORT_UPGRADES: " + std::to_string(dropped) + " unknown or never-upgraded vehicles dropped");
        }
    }
    // [DEVIATION dev mode] PORT_DEVMODE (see sections()). No section: no dev choices. A body of the wrong size or format: none,
    // with a note (not the corrupt alert). Unknown crcs (a pack removed) are dropped with a note; stages clamp to 0..3; stages
    // of a vehicle that is never upgraded are dropped.
    s.dev.assign(s.vehicles.size(), {});
    if (r.find("PORT_DEVMODE")) {
        const size_t body = r.end - r.pos;
        const uint32_t format = r.u32(), flags = r.u32(), n = r.u32();
        (void)flags;  // reserved (0)
        constexpr size_t kRecord = 4 + 4 + upg::kCategories + 2;
        if (!r.ok || format != 1 || body != 12 + size_t(n) * kRecord) {
            notes.push_back("PORT_DEVMODE: format " + std::to_string(format) + ", " + std::to_string(body) + " bytes for " + std::to_string(n) +
                            " vehicles: no dev mode choices");
        } else {
            std::vector<CareerState::DevVehicle> got(s.vehicles.size());
            int dropped = 0;
            for (uint32_t k = 0; k < n && r.ok; ++k) {
                const uint32_t crc = r.u32();
                CareerState::DevVehicle d;
                d.flags = r.u8() & (CareerState::kDevStages | CareerState::kDevLook), d.livery = r.u8(), d.color = r.u8();
                r.u8();
                r.raw(d.equipped, sizeof d.equipped);
                r.u8(), r.u8();
                for (int c = 0; c < upg::kCategories; ++c) d.equipped[c] = uint8_t(std::min<int>(d.equipped[c], upg::kMaxStage));
                size_t at = s.vehicles.size();
                for (size_t i = 0; i < s.vehicles.size() && i < rules_.vehicles.size(); ++i)
                    if (s.vehicles[i].crc == crc) at = i;
                if (at == s.vehicles.size() || rules_.vehicles[at].cls < 0) {
                    ++dropped;
                    continue;
                }
                got[at] = d;
            }
            if (r.ok) s.dev = std::move(got);
            if (dropped) notes.push_back("PORT_DEVMODE: " + std::to_string(dropped) + " unknown or unclassed vehicles dropped");
        }
    }
    if (error && !notes.empty()) *error = notes.front();
    // After a load FUEL replays the unlock rules (CheckUnLock) [I].
    unlockDone_.assign(rules_.unlocks.size(), 0);
    checkUnlocks();
    return true;
}

// PK25: SaveGamePc_Z write 0x840000 opens the save itself for writing (CreateFileA GENERIC_WRITE, CREATE_ALWAYS), so a
// read-only file or folder fails (code 9) [O]; the port still writes a temporary file and renames it over the save (never
// a half-written save) [port], after the same write test on the file. ENOSPC / EDQUOT -> 0xe [port].
int Career::write(const std::string& path) const {
    auto code = [](int e) { return e == ENOSPC || e == EDQUOT ? int(pcsave::NoSpace) : int(pcsave::WriteFailed); };
    std::error_code ec;
    fs::create_directories(fs::path(path).parent_path(), ec);
    if (fs::exists(path, ec) && ::access(path.c_str(), W_OK) != 0) return code(errno);
    const std::vector<uint8_t> b = serialize();
    const std::string tmp = path + ".tmp";
    const int fd = ::open(tmp.c_str(), O_WRONLY | O_CREAT | O_TRUNC, 0644);
    if (fd < 0) return code(errno);
    size_t done = 0;
    while (done < b.size()) {
        const ssize_t n = ::write(fd, b.data() + done, b.size() - done);
        if (n < 0) {
            if (errno == EINTR) continue;
            const int e = errno;
            ::close(fd);
            ::unlink(tmp.c_str());
            return code(e);
        }
        done += size_t(n);
    }
    if (::close(fd) != 0) {
        const int e = errno;
        ::unlink(tmp.c_str());
        return code(e);
    }
    if (::rename(tmp.c_str(), path.c_str()) != 0) {
        const int e = errno;
        ::unlink(tmp.c_str());
        return code(e);
    }
    return pcsave::Ok;
}

int Career::read(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) return pcsave::NotFound;  // 0x840190: CreateFileA fails -> 7
    std::vector<uint8_t> b((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    if (b.empty()) return pcsave::Empty;  // GetFileSize 0 -> 8
    const CareerState keep = state_;
    std::string err;
    if (!deserialize(b, &err)) {
        state_ = keep;
        return pcsave::Corrupt;  // 0x4eb9c0: marker / CRC (the port: header, version, sections) -> 0x17
    }
    return pcsave::Ok;
}

}  // namespace fuel
