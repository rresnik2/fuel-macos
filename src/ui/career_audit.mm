// fuelview --career-audit (P99): see career_audit.h and ~/re-work/fuel/notes/p99-career-audit.md.
//
// A whole career, headless, on a scratch save, through the port's own code:
//   - the menus (fuel::ui::Menu) driven by MenuKey presses: Map tab, hub, CAREER / CHALLENGES, rows, the vehicle
//     carousel, the garage's buy boxes, camps, DATA, Profile; the confirmation popups and tutorials answered by keys;
//   - every race resolved by fuel::RaceRules (gates, laps, timers, knock-outs, hunts, the helicopter) with a scripted
//     autopilot (the player steps across each gate line; opponents and hunted racers are placed), its order turned into
//     the results rows by raceResultRows (the game's own function) and handed to Menu::setResults, then the end-race
//     pages walked with the action key up to EndRace_WhatNext2;
//   - free roam through fuel::FreeRoam (vistas, liveries, barrels, challenge discovery, special traffic catches) and the
//     challenge-arch prompt (Menu::hudTick -> 0x4f6fd0);
//   - every session is a new process in the game (exec): here a new Menu (or a reload of career.sav) per session.
// Each result is checked against an independent model of FUEL's rules (0x630dd0 / 0x630ca0 / 0x578b50 / 0x578a20 /
// 0x4ba590 / 0x4ce590 ...), not against Career's own answers.
#include "ui/career_audit.h"

#include "core/career.hpp"
#include "core/dpc.hpp"
#include "core/events.hpp"
#include "core/freeroam.hpp"
#include "core/hub_zones.hpp"
#include "core/race_editor.hpp"  // PK21
#include "core/race_rules.hpp"
#include "core/settings.hpp"
#include "ui/game_menu.h"  // [upgrades U2] sessionArgs (execSession's arguments)
#include "ui/menu.h"

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <memory>
#include <random>
#include <set>
#include <sstream>
#include <strings.h>
#include <unistd.h>

namespace fs = std::filesystem;

namespace fuel::ui {
namespace {

using Vec3 = std::array<float, 3>;

std::string up(std::string s) {
    for (char& c : s) c = char(std::toupper(static_cast<unsigned char>(c)));
    return s;
}
bool same(const std::string& a, const std::string& b) { return strcasecmp(a.c_str(), b.c_str()) == 0; }

// The menus' id of a mission the scripts name (names compare without case, _stricmp; the world's MISSION_COURSE
// names are upper case).
std::string eventId(const Menu& m, const std::string& name) {
    for (const Event& e : m.events())
        if (same(e.id, name)) return e.id;
    return name;
}

std::vector<uint8_t> readFile(const std::string& p) {
    std::ifstream f(p, std::ios::binary);
    return std::vector<uint8_t>((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
}
void writeFile(const std::string& p, const std::vector<uint8_t>& b) {
    std::ofstream f(p, std::ios::binary | std::ios::trunc);
    f.write(reinterpret_cast<const char*>(b.data()), std::streamsize(b.size()));
}

// ------------------------------------------------------------------------------------------------ the report

struct Feature {
    std::string id, title;
    int pass = 0, fail = 0;
    std::vector<std::string> fails;
    std::string note;
    bool missing = false;
};

class Report {
public:
    void add(const std::string& id, const std::string& title) {
        index_[id] = f_.size();
        f_.push_back({id, title});
    }
    bool check(const std::string& id, bool ok, const std::string& what) {
        Feature& f = f_[index_.at(id)];
        if (ok) {
            ++f.pass;
        } else {
            ++f.fail;
            if (f.fails.size() < 12) f.fails.push_back(what);
            std::printf("  FAIL [%s] %s\n", id.c_str(), what.c_str());
        }
        return ok;
    }
    void missing(const std::string& id, const std::string& why) {
        Feature& f = f_[index_.at(id)];
        f.missing = true, f.note = why;
    }
    void note(const std::string& id, const std::string& n) { f_[index_.at(id)].note = n; }
    int print() const {
        int bad = 0, pass = 0, fail = 0, miss = 0;
        std::printf("\n%-5s %-58s %6s %5s  %s\n", "id", "feature", "checks", "fail", "verdict");
        for (const Feature& f : f_) {
            const char* v = f.fail ? "FAIL" : f.missing ? "MISSING" : f.pass ? "PASS" : "n/a";
            std::printf("%-5s %-58s %6d %5d  %s%s%s\n", f.id.c_str(), f.title.c_str(), f.pass + f.fail, f.fail, v, f.note.empty() ? "" : "  - ",
                        f.note.c_str());
            for (const std::string& s : f.fails) std::printf("        %s\n", s.c_str());
            bad += f.fail;
            pass += !f.fail && !f.missing && f.pass;
            fail += f.fail != 0;
            miss += f.missing && !f.fail;
        }
        std::printf("\ncareer-audit: %d features PASS, %d FAIL, %d MISSING; %d failed checks\n", pass, fail, miss, bad);
        return bad;
    }

private:
    std::vector<Feature> f_;
    std::map<std::string, size_t> index_;
};

// ------------------------------------------------------------------------------------------------ races

enum class Plan { First, Last, Dnf };

struct RaceRun {
    bool ended = false;              // the race rules finished it (else the player quits)
    bool won = false;                // RaceRules::Won (the game's openResults argument)
    int level = -1;                  // medals: the race's level (race +0x3e8, the session's --ai-level): 0 gold .. 2 bronze
    bool playerFinished = false;
    int place = 0;
    double time = 0;
    std::string reason;
    std::vector<ResultRow> rows;
    bool stuck = false;              // the autopilot could not pass a gate (the event cannot be completed)
};

// The autopilot: racer i steps across each of its gate lines in turn (from just before the line to just past it, along
// the gate's normal oriented by the approach), `dt` seconds per step.
bool driveRoute(RaceRules& R, std::vector<Vec3>& pos, size_t i, double dt) {
    const std::vector<Vec3>& line = R.lineFor(i);
    int guard = 0;
    while (R.state != RaceRules::Finished && R.racers[i].next < line.size() && !R.racers[i].out && R.racers[i].finished < 0) {
        if (++guard > 4 * int(line.size()) + 8) return false;
        const size_t g = R.racers[i].next;
        if (!R.usesRoute(i)) {
            pos[i] = line[g];
            R.update(dt, pos);
            continue;
        }
        const RaceRules::Gate& G = R.gates[g];
        float nx = -(G.bz - G.az), nz = G.bx - G.ax;
        float l = std::hypot(nx, nz);
        if (l < 1e-4f) {
            nx = G.centre[0] - pos[i][0], nz = G.centre[2] - pos[i][2], l = std::hypot(nx, nz);
            if (l < 1e-4f) nx = 1, nz = 0, l = 1;
        }
        nx /= l, nz /= l;
        if (nx * (G.centre[0] - pos[i][0]) + nz * (G.centre[2] - pos[i][2]) < 0) nx = -nx, nz = -nz;
        const float back = std::min(4.0f, std::max(0.5f, G.half * 0.5f));
        pos[i] = {G.centre[0] - nx * back, G.centre[1], G.centre[2] - nz * back};
        R.update(dt, pos);
        if (R.racers[i].next != g || R.state == RaceRules::Finished) continue;
        pos[i] = {G.centre[0] + nx * back, G.centre[1], G.centre[2] + nz * back};
        R.update(dt, pos);
        if (R.racers[i].next == g && R.state != RaceRules::Finished) return false;
    }
    return true;
}

RaceRun runRace(const Event& e, Plan plan) {
    RaceRun out;
    RaceRules R;
    const int opp = eventOpponents(e);
    R.begin(e.race, size_t(1 + opp), e.times[0]);
    std::vector<Vec3> pos(R.racers.size(), e.race.start);
    const Vec3 hunt0 = R.aiPath.empty() ? e.race.start : R.aiPath.front();
    for (size_t i = 1; i < pos.size(); ++i)
        if (!R.usesRoute(i)) pos[i] = {hunt0[0] + 30.0f * float(i), hunt0[1], hunt0[2]};
    while (R.state == RaceRules::Countdown) R.update(0.5, pos);
    const bool hunt = R.mode == Race::GmCatchAI;
    if (plan == Plan::First) {
        if (hunt) {
            for (size_t i = 1; i < pos.size() && R.state != RaceRules::Finished; ++i) pos[0] = pos[i], R.update(0.05, pos);
        } else {
            out.stuck = !driveRoute(R, pos, 0, 0.05);
        }
    } else if (plan == Plan::Last && !hunt) {
        for (size_t i = 1; i < pos.size() && R.state != RaceRules::Finished; ++i) driveRoute(R, pos, i, 0.001);
        if (R.state != RaceRules::Finished) out.stuck = !driveRoute(R, pos, 0, 0.05);
    }
    // Waiting (DNF): timers, the helicopter and knock-outs end it; else the player has to quit.
    for (int s = 0; s < 4000 && R.state != RaceRules::Finished; ++s) R.update(1.0, pos);
    out.ended = R.state == RaceRules::Finished;
    out.won = R.outcome == RaceRules::Won;
    out.reason = R.reason;
    out.playerFinished = R.racers[0].finished >= 0;
    out.time = std::max(0.0, R.racers[0].finished);
    std::vector<std::string> names{"PLAYER"};
    std::vector<double> fin{R.racers[0].finished};
    for (size_t i = 1; i < R.racers.size(); ++i) names.push_back("AI" + std::to_string(i)), fin.push_back(R.racers[i].finished);
    const std::vector<int> p = R.positions(pos);
    out.place = p[0];
    out.rows = raceResultRows(p, names, fin);
    return out;
}

// medals: the level the menu's session launch hands over (sessionArgs --ai-level = race +0x3e8), -1 when none.
int sessionLevel(const std::string& dataDir, const MenuRequest& r, const Menu& m) {
    const std::vector<std::string> args = sessionArgs(dataDir, r, m);
    for (size_t i = 0; i + 1 < args.size(); ++i)
        if (args[i] == "--ai-level") return std::atoi(args[i + 1].c_str());
    return -1;
}

// ------------------------------------------------------------------------------------------------ FUEL's rules (model)

int starsOf(int rank) { return rank == 0 ? 3 : rank == 1 ? 2 : rank == 2 ? 1 : 0; }

int forSale(const Career& c) {  // 0x56a140: unlocked, no code, a class
    int n = 0;
    const auto& R = c.rules();
    for (size_t v = 0; v < R.vehicles.size(); ++v)
        n += c.state().vehicles[v].unlocked && (R.vehicles[v].code.empty() || up(R.vehicles[v].code) == "NONE") && R.vehicles[v].cls != -1;
    return n;
}

// What FUEL gives for a finished race (0x6300c0 -> 0x630dd0 career / 0x630ca0 challenge, 0x578aa0 hubs), from the state
// before the race.
struct Expect {
    int list = -1;
    bool counted = false;        // mission state 6 (ENDING_WON)
    int newStars = 0;
    uint32_t fuel = 0;
    std::vector<int> hubs;
    int gift = -1;               // the vehicle the race gives (0x56a320)
    std::vector<int> pilot;      // pilot liveries (0x56b830)
    bool kinds[4] = {false, false, false, false};  // traffic revealed (0x574cb0): vista, livery, challenge, vehicle
    std::vector<std::string> pages;
};

Expect expectFor(const Career& c, const std::string& event, const RaceRun& run) {
    Expect x;
    const CareerRules& R = c.rules();
    const CareerState& s = c.state();
    int h = -1, k = -1;
    if (!c.locate(event, &h, &x.list, &k)) return x;
    const int mi = R.missionIndex(event);
    const MissionStatus& m = (x.list == 1 ? s.career : s.challenges)[size_t(h)][size_t(k)];
    if (x.list == 1) {
        // Career (medals): won = mission state 6 from the end-condition list 0x62cff0 (finished 1st; bit 0x800, place < 4,
        // is in no list), = RaceRules::Won; rank = the race's level (race +0x3e8, 0x630dd0), not the place.
        x.counted = run.won;
        const int rank = std::clamp(run.level, 0, 2);
        if (x.counted && rank < m.bestRank) {
            x.newStars = starsOf(rank) - starsOf(m.bestRank);
            for (int i = rank; i < 3; ++i)
                if (!m.medal[i]) x.fuel += uint32_t(std::lround(R.missions[size_t(mi)].fuel[i]));
            const int stars = c.stars() + x.newStars;
            for (size_t hh = 0; hh < R.hubs.size(); ++hh)  // 0x578aa0 with 0x579720 (no hub needs a mission in the data)
                if (!s.hubs[hh].unlocked && R.hubs[hh].starsNeeded >= 0 && stars >= R.hubs[hh].starsNeeded) x.hubs.push_back(int(hh));
        }
    } else {
        x.counted = run.won;
        if (x.counted && m.bestRank > 0 && !m.medal[0]) x.fuel = uint32_t(std::lround(R.missions[size_t(mi)].fuel[0]));
    }
    if (!x.counted) return x;
    for (size_t v = 0; v < R.vehicles.size() && x.gift < 0; ++v)
        if (!s.vehicles[v].owned && same(R.vehicles[v].raceUnlock, event)) x.gift = int(v);
    for (size_t p = 0; p < R.pilotLiveries.size(); ++p)
        if (!s.pilotOwned[p] && same(R.pilotLiveries[p].unlock, event)) x.pilot.push_back(int(p));
    if (x.list == 1)
        for (size_t hh = 0; hh < R.hubs.size(); ++hh)
            for (size_t t = 0; t < R.hubs[hh].traffic.size(); ++t)
                if (!s.traffic[hh][t].a && same(R.hubs[hh].traffic[t].mission, event)) x.kinds[std::clamp(R.hubs[hh].traffic[t].kind(), 0, 3)] = true;
    return x;
}

// 0x4ba590 [O]: ranking 0xf/0x10 -> 0x1b (vehicles for sale changed) -> 0x13 (hubs) -> 0x14 (pilot livery) -> the first
// doppler 0x15 livery / 0x16 vista (-> 0x18 when a vehicle too) / 0x17 challenge / 0x18 Maverick -> 0x19 BonusCar
// (+0xe90) -> 0x1c.
std::vector<std::string> expectPages(const Expect& x, bool saleChanged) {
    std::vector<std::string> p;
    if (saleChanged) p.push_back("EndRaceNewVehiclesUnlocked");
    if (!x.hubs.empty()) p.push_back("EndRace_Event_Unlocked2");
    if (x.pilot.empty()) return p;
    p.push_back("EndRace_PilotLivery_Unlocked2");
    static const char* const kPage[4] = {"EndRace_VistaDoppler_Unlocked2", "EndRace_LiveryDoppler_Unlocked2", "EndRace_ChallengeDoppler_Unlocked2",
                                         "EndRace_Maverick_Unlocked2"};
    int first = -1;
    for (int k : {1, 0, 2, 3})
        if (x.kinds[k] && first < 0) first = k;
    if (first < 0) return p;
    p.push_back(kPage[first]);
    if (first == 0 && x.kinds[3]) p.push_back(kPage[3]);
    if (x.gift >= 0) p.push_back("EndRace_BonusCar_Unlocked2");
    return p;
}

// ------------------------------------------------------------------------------------------------ the audit

class Audit {
public:
    Audit(const std::string& dataDir, const std::string& scratch) : data_(dataDir), scratch_(scratch) {
        settings_ = loadSettings();
        settings_.dataDir = dataDir;
        save_ = Career::careerPath();
    }
    int run();

private:
    // A new process of the game: the menus (MenuMode::Main) or a session (MenuMode::Pause + setSession).
    std::unique_ptr<Menu> boot(MenuMode mode) {
        ++boots_;
        auto m = std::make_unique<Menu>(data_, settings_, mode);
        if (!zones_.zones.empty()) m->setHubZones(&zones_);
        return m;
    }
    void frames(Menu& m, int n = 3) {
        for (int i = 0; i < n; ++i) m.update(0.1f), m.updateTutorial(0.1f, true);
    }
    // Answers every tutorial page with the action key (OK 0x56e4c0); returns the entries shown.
    std::vector<int> okTutorials(Menu& m) {
        std::vector<int> shown;
        for (int guard = 0; guard < 80; ++guard) {
            frames(m, 1);
            if (m.tutorialShown() < 0) {
                if (m.tutorialQueue().empty()) break;
                continue;
            }
            if (shown.empty() || shown.back() != m.tutorialShown()) shown.push_back(m.tutorialShown());
            m.tutorialKey(MenuKey::Accept);
        }
        // FUEL's manager keeps +0x14 set for 0.5 s after the last page closes and that clock only runs while nothing is
        // queued (0x56e340 / 0x56e620): a tab switched sooner would wait for good [O]. A player reads; so do we.
        if (!shown.empty()) frames(m, 7);
        return shown;
    }
    bool focusLeft(Menu& m, const std::string& id) {
        const auto ids = m.leftMenuIds();
        const auto it = std::find(ids.begin(), ids.end(), id);
        if (it == ids.end()) return false;
        const int want = int(it - ids.begin());
        for (int g = 0; g < 12 && m.focusIndex() != want; ++g) m.key(MenuKey::Down);
        return m.focusIndex() == want;
    }
    bool selectHub(Menu& m, const std::string& name) {
        const Hub* want = findHub(m.hubs(), name);
        for (size_t g = 0; g <= m.hubs().size(); ++g) {
            if (&m.hubs()[size_t(m.hubIndex())] == want) return true;
            m.key(MenuKey::Right);
        }
        return false;
    }
    // A race / camp list row reached the way the keys can (0x47dfc0: Up/Down inside a page, Right to the second page).
    bool focusRow(Menu& m, int row, int perPage) {
        if (row >= perPage) m.key(MenuKey::Right);
        for (int g = 0; g < perPage && m.focusIndex() < row; ++g) m.key(MenuKey::Down);
        return m.focusIndex() == row;
    }
    bool confirmYes(Menu& m) {
        if (!m.confirmOpen()) return false;
        m.key(MenuKey::Left);  // No is focused first (0x4ce390(0)); Left -> Yes
        m.key(MenuKey::Accept);
        return true;
    }
    bool openEventFromMap(Menu& m, const std::string& hub, const std::string& event, int list, std::string* why);
    MenuRequest chooseVehicleAndStart(Menu& m, std::string* why, bool allowBuy = true);
    void checkRacePage(Menu& m, const std::string& event, int list);  // PK02
    void checkRanking(Menu& g, const std::string& what);              // PK08
    void checkFuelReward(Menu& g, const Career& c, const std::string& event, const std::string& what);
    void checkUnlockMap(Menu& g, const Career& c, const std::string& what);
    int fuelPages_ = 0;  // PK08
    int racePages_ = 0, fuelRewardPages_ = 0, warningPages_ = 0, forbiddenLists_ = 0;
    struct SessionOut {
        RaceRun run;
        Expect x;
        CareerAward award;
        std::vector<std::string> pages;
        bool quit = false;
    };
    // `level`: the race's level (--ai-level; medals); -1 = the row the career page preselects (best rank - 1, at least 0).
    SessionOut raceSession(const std::string& event, const std::string& vehicle, Plan plan, bool fresh, const std::string& tag, int level = -1);
    void checkPersisted(const Career& c, const std::string& what);

    void newCareer();
    void firstHours();
    void wholeCareer();
    void archStarts(bool fresh);
    void economy();
    void upgrades();   // [upgrades U2]
    void aiUpgrades(); // [upgrades U4]
    void aiLevel();    // cr5
    void racePages();  // PK02
    void hubMenu();    // PK11
    void travel();
    void freeRoam();
    void saves();
    void profile();
    void achievements();
    void editorRaces();  // PK21
    void crossCheck();
    void finalState();

    std::string data_, scratch_, save_;
    Settings settings_;
    Report rep_;
    HubZones zones_;
    std::vector<Event> events_;
    std::unique_ptr<CareerRules> rules_;
    std::unique_ptr<Menu> session_;  // the reused session menu (wholeCareer)
    int boots_ = 0, races_ = 0;
    std::set<std::string> completed_, startedFromMenu_, blocked_;
    std::map<int, int> modeRuns_;
    std::set<int> modeLast_, modeDnf_;
    uint64_t fuelEarned_ = 0, fuelSpent_ = 0;
};

bool Audit::openEventFromMap(Menu& m, const std::string& hub, const std::string& event, int list, std::string* why) {
    for (int g = 0; g < 8 && m.screenName().rfind("Tab", 0) != 0; ++g) m.key(MenuKey::Back);
    for (int g = 0; g < 8 && m.screenName() != "Tab:2"; ++g) m.key(MenuKey::TabRight);
    for (int g = 0; g < 8 && m.screenName() != "Tab:2"; ++g) m.key(MenuKey::TabLeft);
    if (m.screenName() != "Tab:2") return *why = "no Map tab (" + m.screenName() + ")", false;
    if (!selectHub(m, hub)) return *why = "hub " + hub + " not on the map", false;
    const std::string item = list == 1 ? "ITEM_ID_LEFT_MENU5" : "ITEM_ID_LEFT_MENU4";
    if (!focusLeft(m, item)) return *why = item + " not focusable", false;
    m.key(MenuKey::Accept);
    const std::string want = list == 1 ? "CareerList" : "ChallengeList";
    if (m.screenName() != want) return *why = "the hub's " + want + " did not open (" + m.screenName() + ")", false;
    const auto rows = m.listRows();
    const auto it = std::find_if(rows.begin(), rows.end(), [&](const std::string& r) { return same(r, event); });
    if (it == rows.end()) return *why = event + " not on the " + want, false;
    if (!focusRow(m, int(it - rows.begin()), list == 1 ? 3 : 5)) return *why = "row " + std::to_string(it - rows.begin()) + " not reachable by keys", false;
    m.key(MenuKey::Accept);
    if (!same(m.pendingEvent(), event)) return *why = "the row did not take the event (not playable?)", false;
    // PK02: the row's Accept opens the race page (0x47d550 -> state 0x56 / 0x47d660 -> 0x59) before the vehicle pick.
    if (m.screenName() != "RacePage") return *why = "the row's Accept did not open the race page (" + m.screenName() + ")", false;
    checkRacePage(m, event, list);
    return true;
}

// PK02: what the race page shows, against the rules: the medal row preselected = best rank - 1 (0x46b970, at least 0); a
// challenge page's GROUP_FUEL_REWARD = ftol(mission +0x94) while the challenge is not won (rank +0x120 == 3, 0x481280) and
// hidden once won; GROUP_CHALLENGE_WARNING only when no unlocked vehicle passes the mission's forbidden list (0x481340).
void Audit::checkRacePage(Menu& m, const std::string& event, int list) {
    Career& c = *m.career();
    const CareerRules& R = c.rules();
    const int mi = R.missionIndex(event);
    const MissionStatus* ms = c.status(event);
    const int rank = ms ? ms->bestRank : 3;
    const Event* e = nullptr;
    for (const Event& x : m.events())
        if (same(x.id, event)) e = &x;
    ++racePages_;
    rep_.check("P2", !m.raceTitle().empty(), "the race page of " + event + " shows the race name");
    if (list == 1) {
        rep_.check("P2", m.raceMedalRow() == std::max(0, rank - 1), event + ": the medal row preselected is rank - 1 (rank " + std::to_string(rank) + ", row " + std::to_string(m.raceMedalRow()) + ")");
        return;
    }
    const std::string want = rank == 3 && mi >= 0 ? std::to_string(int(R.missions[size_t(mi)].fuel[0])) : std::string();
    rep_.check("P2", m.raceFuelReward() == want, event + ": the fuel reward shows ftol(mission +0x94) while not won (\"" + m.raceFuelReward() + "\" vs \"" + want + "\")");
    if (rank == 3) ++fuelRewardPages_;
    // The warning model (0x481340): no unlocked vehicle of the garage's name list outside the forbidden list.
    bool any = false;
    for (size_t vi = 0; vi < m.vehicles().size(); ++vi) {
        const VehicleInfo& v = m.vehicles()[vi];
        if (v.family == "UNKNOWN") continue;
        if (R.vehicleIndex(v.name) >= 0 && !c.vehicleUnlocked(v.name)) continue;
        if (e && std::find(e->race.forbidden.begin(), e->race.forbidden.end(), int(vi)) != e->race.forbidden.end()) continue;
        any = true;
    }
    rep_.check("P2", m.raceWarning() == !any, event + ": GROUP_CHALLENGE_WARNING shows only when no allowed vehicle is unlocked (" + (m.raceWarning() ? "shown" : "hidden") + ")");
    if (m.raceWarning()) ++warningPages_;
}

MenuRequest Audit::chooseVehicleAndStart(Menu& m, std::string* why, bool allowBuy) {
    Career& c = *m.career();
    if (m.screenName() == "RacePage") m.key(MenuKey::Accept);  // PK02: the race page's Accept (0x47e310 -> 0xaf / 0x47ed60 -> 0xb0)
    if (m.screenName() != "VehicleList") {
        // A challenge with its own vehicle starts at once (0x47efb0), after the start tutorials.
        MenuRequest r = m.take();
        if (r.kind == MenuRequest::None) okTutorials(m), r = m.take();
        const Event* e = nullptr;
        for (const Event& x : m.events())
            if (x.id == m.pendingEvent()) e = &x;
        rep_.check("R1", e && r.kind == MenuRequest::StartEvent && r.vehicle == e->vehicle,
                   "challenge " + m.pendingEvent() + " starts with its own vehicle " + (e ? e->vehicle : std::string()) + " (got " + r.vehicle + ")");
        if (r.kind != MenuRequest::StartEvent) *why = "no StartEvent request for a challenge with its own vehicle";
        return r;
    }
    const auto rows = m.listRows();
    {   // PK02: the garage list never holds a vehicle of the mission's forbidden list (0x4fe250, table index = AddVehicleInfo order)
        const Event* pe = nullptr;
        for (const Event& x : m.events())
            if (x.id == m.pendingEvent()) pe = &x;
        bool clean = true;
        for (const std::string& name : rows)
            for (size_t vi = 0; pe && vi < m.vehicles().size(); ++vi)
                if (m.vehicles()[vi].name == name && std::find(pe->race.forbidden.begin(), pe->race.forbidden.end(), int(vi)) != pe->race.forbidden.end()) clean = false;
        rep_.check("P3", clean, "the vehicle list of " + m.pendingEvent() + " holds no forbidden vehicle");
        if (pe && !pe->race.forbidden.empty()) ++forbiddenLists_;
    }
    if (rows.empty()) return *why = "no vehicle of the event's class unlocked", MenuRequest{};
    int pick = -1;
    for (size_t i = 0; i < rows.size(); ++i)
        if (c.vehicleOwned(rows[i]) && (pick < 0 || same(rows[i], m.vehicle()))) pick = int(i);
    if (pick < 0 && allowBuy) {  // buy the cheapest one the fuel pays for (InfoBoxGarageYesNo -> 0x4ce590)
        int best = -1;
        for (size_t i = 0; i < rows.size(); ++i) {
            const int p = c.vehiclePrice(rows[i]);
            if (c.vehicleUnlocked(rows[i]) && p >= 0 && uint32_t(p) <= c.state().fuel && (best < 0 || p < c.vehiclePrice(rows[size_t(best)]))) best = int(i);
        }
        if (best >= 0) {
            for (int g = 0; g < 80 && m.focusIndex() != best; ++g) m.key(m.focusIndex() < best ? MenuKey::Right : MenuKey::Left);
            const uint32_t before = c.state().fuel;
            m.key(MenuKey::Accept);
            rep_.check("E1", m.infoBox() == "InfoBoxGarageYesNo", "buy box for " + rows[size_t(best)] + ": " + m.infoBox());
            m.key(MenuKey::Accept);
            const int price = c.vehiclePrice(rows[size_t(best)]);
            rep_.check("E1", c.vehicleOwned(rows[size_t(best)]) && c.state().fuel == before - uint32_t(price),
                       "bought " + rows[size_t(best)] + " for " + std::to_string(price) + " (fuel " + std::to_string(before) + " -> " + std::to_string(c.state().fuel) + ")");
            fuelSpent_ += uint32_t(price);
            pick = best;
        }
    }
    if (pick < 0) {
        std::string list;
        for (const auto& r : rows) list += " " + r + "(" + std::to_string(c.vehiclePrice(r)) + (c.vehicleUnlocked(r) ? "" : ",locked") + ")";
        return *why = "no owned vehicle of the class and none affordable (fuel " + std::to_string(c.state().fuel) + "):" + list, MenuRequest{};
    }
    for (int g = 0; g < 80 && m.focusIndex() != pick; ++g) m.key(m.focusIndex() < pick ? MenuKey::Right : MenuKey::Left);
    m.key(MenuKey::Accept);
    MenuRequest r = m.take();
    if (r.kind == MenuRequest::None) {  // tutorials 14 / 15 first (0x4786e0), the start after 15's OK
        okTutorials(m);
        r = m.take();
    }
    if (r.kind != MenuRequest::StartEvent) *why = "no StartEvent request after the vehicle (" + std::to_string(int(r.kind)) + ", " + m.screenName() + ")";
    return r;
}

// PK08 0x4b1c50 / 0x4b1aa0: the ranking window. Row k shows racer (sel - row + k); the highlighted row starts at 1 (+0xe70 = +0xe6c = 0
// then the draw's clamp); Down moves sel and row (the row stops at 5 / the racer count), Up the other way; the local player's
// name is the profile name or TT 1262 "User".
void Audit::checkRanking(Menu& g, const std::string& what) {
    const int n = int(g.rankRows());
    g.update(0.1f);
    rep_.check("P8", g.rankSelected() == 0 && g.rankHighlight() == (n > 0 ? 1 : 0), what + ": the ranking starts on its first row");
    int sel = 0, row = 1;
    bool ok = true;
    for (int step = 0; step < n + 2 && n > 0; ++step) {
        for (int k = 1; k <= 5; ++k) {
            const int want = sel - row + k;
            const bool shown = want >= 0 && want < n;
            ok = ok && ((g.rankName(k) != "") == shown);
        }
        ok = ok && g.rankSelected() == sel && g.rankHighlight() == row;
        g.key(MenuKey::Down);
        if (sel < n - 1) ++sel;
        row = std::min(row + 1, std::min(5, n));
        g.update(0.1f);
    }
    for (int step = 0; step < n + 2 && n > 0; ++step) {
        ok = ok && g.rankSelected() == sel && g.rankHighlight() == row;
        g.key(MenuKey::Up);
        if (sel > 0) --sel;
        row = std::max(row - 1, 1);
        g.update(0.1f);
    }
    rep_.check("P8", ok && g.rankSelected() == 0, what + ": Down / Up scroll the " + std::to_string(n) + " racers through the 5 rows");
    bool name = true;
    for (int i = 0; i < n; ++i) {
        for (int k = 1; k <= 5; ++k)
            if (g.rankIndex(k) == i && g.rowIsPlayer(i)) name = name && g.rankName(k) == g.textOf(1262);
    }
    rep_.check("P8", name && g.textOf(1262) == "User", what + ": the player's row reads TT 1262");
}

// PK08 0x4b7630 / 0x4b68b0: ITEM_ID_FUEL_WON = "+%d" of ftol(mission +0x94), ITEM_ID_FUEL_TOTAL = "YOUR FUEL: %d" of game +0x408.
void Audit::checkFuelReward(Menu& g, const Career& c, const std::string& event, const std::string& what) {
    const auto t = g.fuelRewardTexts();
    const int mi = c.rules().missionIndex(event);
    const std::string won = "+" + std::to_string(int(c.rules().missions[size_t(mi)].fuel[0]));
    rep_.check("P5", t.first == won, what + ": page 121 FUEL_WON \"" + t.first + "\" (FUEL \"" + won + "\")");
    rep_.check("P5", t.second == "YOUR FUEL: " + std::to_string(c.state().fuel), what + ": page 121 FUEL_TOTAL \"" + t.second + "\" (game +0x408 = " + std::to_string(c.state().fuel) + ")");
    ++fuelPages_;
}

// PK08 0x4b63a0 / 0x4b64e0: "<n>A" / "<n>B" (n = game +0x346c), MAP_2's alpha (sin(8 t) + 1) / 2 with t the time on the page.
void Audit::checkUnlockMap(Menu& g, const Career& c, const std::string& what) {
    float a = -1;
    const auto pics = g.unlockMapPictures(&a);
    const std::string n = std::to_string(c.state().get<int32_t>(0xb4));
    rep_.check("P6", pics[0] == n + "A" && pics[1] == n + "B" && g.assets().image(pics[0]) && g.assets().image(pics[1]),
               what + ": map pictures " + pics[0] + " / " + pics[1] + " (n = " + n + ", both exist)");
    g.update(0.25f);
    g.unlockMapPictures(&a);
    const float t = g.mapPulse();
    rep_.check("P6", std::fabs(a - (std::sin(t * 8.0f) + 1.0f) * 0.5f) < 1e-5f && t > 0, what + ": MAP_2 pulses (sin(8 t) + 1) / 2");
}

void Audit::checkPersisted(const Career& c, const std::string& what) {
    Career back(*rules_);
    const bool ok = back.load(save_);
    // The process ends with the session: what the next one loads must be what this one had.
    rep_.check("S5", ok && back.serialize() == c.serialize(), "career.sav after " + what + " holds the session's whole state");
}

Audit::SessionOut Audit::raceSession(const std::string& event, const std::string& vehicle, Plan plan, bool fresh, const std::string& tag, int level) {
    SessionOut o;
    std::unique_ptr<Menu> own;
    Menu* g = nullptr;
    if (fresh) {
        own = boot(MenuMode::Pause);
        g = own.get();
    } else {
        if (!session_) session_ = boot(MenuMode::Pause);
        g = session_.get();
        g->career()->load();  // the new process reads career.sav
        g->setMode(MenuMode::Pause);
    }
    g->setSession(event, "", vehicle);
    Career& c = *g->career();
    const Event* ev = nullptr;
    for (const Event& e : g->events())
        if (e.id == event) ev = &e;
    if (!ev) {
        rep_.check("R1", false, tag + ": " + event + " not among the session's events");
        return o;
    }
    ++races_;
    ++modeRuns_[ev->race.gameMode()];
    o.run = runRace(*ev, plan);
    const std::vector<uint8_t> before = c.serialize();
    const int starsBefore = c.stars();
    const uint32_t fuelBefore = c.state().fuel;
    const int saleBefore = forSale(c);
    const int saves0 = g->saveCount();
    int hub = -1, list = -1, idx = -1;
    c.locate(event, &hub, &list, &idx);
    const MissionStatus m0 = list >= 0 ? *c.status(event) : MissionStatus{};
    const int finishedBefore = c.state().get<int32_t>(0x88);
    if (level < 0) level = std::max(0, m0.bestRank - 1);  // 0x46b970's preselected row (m0.bestRank 3 when not listed)
    o.run.level = list == 1 ? std::clamp(level, 0, 2) : 0;  // the challenge page: TTMN ... 0
    if (!o.run.ended) {
        // The player gives up: Race tab, GO FREE RIDE, "Your progression ... will be lost" (TT_PAGE_RACE_YES_NO), Yes.
        o.quit = true;
        rep_.check("R5", focusLeft(*g, "ITEM_ID_TOPMENU_RACE_GO_FREE_RIDE"), tag + " " + event + ": the race menu has GO FREE RIDE");
        g->key(MenuKey::Accept);
        rep_.check("R5", g->confirmOpen() && g->confirmText() == "TT_PAGE_RACE_YES_NO", tag + " " + event + ": quitting asks TT_PAGE_RACE_YES_NO");
        confirmYes(*g);
        const MenuRequest r = g->take();
        rep_.check("R5", r.kind == MenuRequest::FreeRide, tag + " " + event + ": quitting ends in free ride");
        rep_.check("R5", c.serialize() == before && g->saveCount() == saves0, tag + " " + event + ": quitting changes nothing (KillMission, no state 6/7)");
        g->sessionEnd();
        return o;
    }
    o.x = expectFor(c, event, o.run);
    // GameOverlay::openResults
    g->setResults(o.run.rows, o.run.won, o.run.level);
    g->setMode(MenuMode::Results);
    o.award = g->lastAward();
    const MissionStatus* m1 = c.status(event);
    const std::string what = tag + " " + event + (list == 1 ? " level " + std::to_string(o.run.level) : std::string()) + " place " +
                             std::to_string(o.run.place) + "/" + std::to_string(o.run.rows.size()) +
                             (o.run.playerFinished ? "" : " (not finished: " + o.run.reason + ")");
    if (list >= 0 && m1) {
        rep_.check("R9", m1->flag10c == 1 && c.state().get<int32_t>(0x88) == finishedBefore + 1, what + ": played flag +0x10c and races finished +0x3440");
        const char* fid = list == 1 ? "R6" : "R10";
        rep_.check(fid, o.award.counted == o.x.counted, what + ": counted " + std::to_string(o.award.counted) + " (FUEL " + std::to_string(o.x.counted) + ")");
        rep_.check(fid, c.stars() == starsBefore + o.x.newStars,
                   what + ": stars " + std::to_string(starsBefore) + " -> " + std::to_string(c.stars()) + " (FUEL +" + std::to_string(o.x.newStars) + ")");
        rep_.check("R7", c.state().fuel == std::min<uint64_t>(99999999u, uint64_t(fuelBefore) + o.x.fuel) && o.award.fuelWon == o.x.fuel,
                   what + ": fuel +" + std::to_string(c.state().fuel - fuelBefore) + " (FUEL +" + std::to_string(o.x.fuel) + ")");
        fuelEarned_ += o.x.fuel;
        if (o.x.counted) {
            const uint32_t score = 0x7fffffffu - uint32_t(std::lround(o.run.time * 100.0));
            rep_.check("R8", m1->lastScore == score && m1->bestScore == std::max(m0.bestScore, score) && m1->lastPlace == o.run.place &&
                                 m1->bestPlace == std::min<int>(m0.bestPlace, o.run.place),
                       what + ": record (score " + std::to_string(m1->bestScore) + ", place " + std::to_string(m1->bestPlace) + ")");
            if (list == 1) {
                const int rank = o.run.level;  // medals: the race's level
                bool medals = m1->lastRank == rank && m1->bestRank == std::min(m0.bestRank, rank);
                for (int i = 0; i < 3; ++i) medals = medals && (m1->medal[i] != 0) == (m0.medal[i] || i >= std::min(m0.bestRank, rank));
                rep_.check("R6", medals, what + ": medals / ranks (best " + std::to_string(m1->bestRank) + ")");
            } else {
                rep_.check("R10", m1->bestRank == 0 && m1->medal[0], what + ": challenge won (rank 0, gold)");
            }
            rep_.check("S2", g->saveCount() == saves0 + 1, what + ": SaveGameData after the award");
        } else {
            rep_.check(fid, std::memcmp(&m1->medal, &m0.medal, 3) == 0 && m1->bestRank == m0.bestRank && m1->bestScore == m0.bestScore,
                       what + ": lost: no medal, rank or record");
            rep_.check("S2", g->saveCount() == saves0, what + ": a lost race does not save (state 7)");
        }
        std::set<std::string> gotHubs(o.award.hubsUnlocked.begin(), o.award.hubsUnlocked.end()), wantHubs;
        for (int hh : o.x.hubs) wantHubs.insert(c.rules().hubs[size_t(hh)].name);
        rep_.check("U1", gotHubs == wantHubs, what + ": hubs unlocked " + std::to_string(gotHubs.size()) + " (FUEL " + std::to_string(wantHubs.size()) + ")");
        if (o.x.gift >= 0)
            rep_.check("U3", c.state().vehicles[size_t(o.x.gift)].owned && c.state().vehicles[size_t(o.x.gift)].unlocked,
                       what + ": gives " + c.rules().vehicles[size_t(o.x.gift)].label);
        for (int p : o.x.pilot) rep_.check("U6", c.state().pilotOwned[size_t(p)] != 0, what + ": pilot livery " + std::to_string(p));
        for (int kd = 0; kd < 4; ++kd)
            if (o.x.kinds[kd]) {
                bool any = false;
                for (size_t hh = 0; hh < c.rules().hubs.size(); ++hh)
                    for (size_t t = 0; t < c.rules().hubs[hh].traffic.size(); ++t)
                        any = any || (same(c.rules().hubs[hh].traffic[t].mission, event) && c.rules().hubs[hh].traffic[t].kind() == kd && c.state().traffic[hh][t].a);
                rep_.check("U4", any, what + ": special traffic kind " + std::to_string(kd) + " revealed");
            }
    }
    // The end-race pages: states 6/7 (EndRace) -> ranking -> reward (counted career) -> unlock pages -> WhatNext (0x1c).
    o.x.pages = expectPages(o.x, forSale(c) != saleBefore);
    std::vector<std::string> seen;
    for (int guard = 0; guard < 24; ++guard) {
        const std::string sname = g->screenName();
        if (sname == "Unlock") seen.push_back(g->unlockPage());
        else if (seen.empty() || seen.back() != sname) seen.push_back(sname);
        if (sname == "WhatNext") break;
        if (sname == "Results") checkRanking(*g, what);
        if (sname == "FuelReward") checkFuelReward(*g, c, event, what);
        if (sname == "Reward") {  // medals: race_reward (0x4b4220) shows the medals of the race's level, from the state before
            static const char* const kNow[3] = {"MAP_CAREER_GOLD_ICON", "MAP_CAREER_SILVER_ICON", "MAP_CAREER_BRONZE_ICON"};
            static const char* const kHad[3] = {"MEDAL_GOLD_GRISE", "MEDAL_SILVER_GRISE", "MEDAL_BRONZE_GRISE"};
            const Menu::RaceRewardItems ri = g->raceRewardItems();
            const int rank = o.run.level;
            const bool better = rank < m0.bestRank;
            bool ok = ri.fuelWon == std::to_string(o.x.fuel) && ri.stars == std::to_string(o.x.newStars);
            std::string bars;
            for (int i = 0; i < 3; ++i) {
                const bool now = better && i >= rank && !m0.medal[i];
                const std::string want = now ? kNow[i] : m0.medal[i] ? kHad[i] : "";
                ok = ok && ri.slot[size_t(i)] == want;
                bars += " " + (ri.slot[size_t(i)].empty() ? std::string("-") : ri.slot[size_t(i)]);
            }
            rep_.check("R11", ok, what + ": race_reward bars" + bars + ", fuel " + ri.fuelWon + " (FUEL " + std::to_string(o.x.fuel) + "), stars " + ri.stars);
        }
        if (sname == "Unlock" && g->unlockPage() == "EndRace_Event_Unlocked2") checkUnlockMap(*g, c, what);
        g->key(MenuKey::Accept);
    }
    // PK08 0x4ba590: a challenge ends in state 8 / 9: 8 -> 0x1a (mission +0x50: the first gold medal, set with the fuel) else 0x1c,
    // 9 -> 0x1c; no ranking, reward or unlock pages. A career race: 6 / 7 -> ranking -> reward -> unlock pages -> 0x1c.
    std::vector<std::string> want = {"EndRace"};
    if (list == 2) {
        if (o.x.counted && o.x.fuel > 0) want.push_back("FuelReward");
        rep_.check("P5", (std::find(seen.begin(), seen.end(), "FuelReward") != seen.end()) == (o.x.counted && o.x.fuel > 0),
                   what + ": page 121 follows a challenge's first gold medal only (fuel +" + std::to_string(o.x.fuel) + ")");
    } else {
        want.push_back("Results");
        if (list == 1 && o.x.counted) want.push_back("Reward");
        for (const auto& p : o.x.pages) want.push_back(p);
    }
    want.push_back("WhatNext");
    std::string got, exp;
    for (const auto& s : seen) got += " " + s;
    for (const auto& s : want) exp += " " + s;
    rep_.check("R11", seen == want, what + ": pages" + got + " (FUEL" + exp + ")");
    o.pages = seen;
    if (g->screenName() == "WhatNext") {
        // 0x4ba590 state 0x1c: MENU_VALI2 -> TTMN "<mission>" (retry), MENU_CANCEL -> TeleportToHub, MENU_VALID ->
        // KillMission 1 (free ride where the player is).
        g->key(MenuKey::Vali2);
        rep_.check("R11", g->take().kind == MenuRequest::Restart, what + ": WhatNext's VALI2 retries the race");
        g->key(MenuKey::TabRight);
        const MenuRequest t = g->take();
        rep_.check("R11", t.kind == MenuRequest::FreeRide && !t.at, what + ": WhatNext's CANCEL goes to the hub's camp");
        g->key(MenuKey::Accept);
        const MenuRequest r = g->take();
        rep_.check("R11", r.kind == MenuRequest::FreeRide && r.at, what + ": WhatNext's action key goes on in free ride where the race ended");
    }
    g->sessionEnd();
    checkPersisted(c, what);
    if (o.x.counted) completed_.insert(event);
    return o;
}

// ---------------------------------------------------------------------------------------------- 1. new career

void Audit::newCareer() {
    std::printf("-- new career\n");
    rep_.check("N1", !fs::exists(save_), "no save before the first boot (a new career)");
    auto m = boot(MenuMode::Main);
    Career& c = *m->career();
    const CareerRules& R = c.rules();
    int unlocked = 0;
    for (size_t h = 0; h < R.hubs.size(); ++h) unlocked += c.hubUnlocked(int(h));
    rep_.check("N1", unlocked == 1 && c.hubUnlocked(0) && R.hubs[0].starsNeeded == 0, "only " + R.hubs[0].name + " is unlocked (stars needed 0, 0x578a20)");
    rep_.check("N1", c.currentHub() == 0, "the career starts in hub uid 0 (" + R.hubs[0].name + ")");
    rep_.check("N1", c.state().fuel == 0 && c.stars() == 0, "no fuel and no stars (InitEmptySave 0x54ec80)");
    // Vehicles: UnlockNeed ALWAYS / HUB "<unlocked hub>" -> unlocked (0x5641c0, every rule of a target met); price < 1 ->
    // owned (0x569e60).
    std::string owned;
    bool rule = true;
    for (size_t v = 0; v < R.vehicles.size(); ++v) {
        int rules = 0, met = 0;
        for (const auto& u : R.unlocks)
            if (u.kind == "VEHICLE" && R.vehicleIndex(u.target) == int(v)) ++rules, met += u.cond == "ALWAYS" || (u.cond == "HUB" && R.hubIndex(u.arg) == 0);
        const bool want = rules > 0 && met == rules;
        const bool ok = (c.state().vehicles[v].unlocked != 0) == want && (c.state().vehicles[v].owned != 0) == (want && R.vehicles[v].price < 1);
        if (!ok) rep_.check("N1", false, R.vehicles[v].label + ": unlocked " + std::to_string(c.state().vehicles[v].unlocked) + " owned " + std::to_string(c.state().vehicles[v].owned));
        rule = rule && ok;
        if (c.state().vehicles[v].owned && R.vehicles[v].cls >= 0) owned += " " + R.vehicles[v].label;
    }
    rep_.check("N1", rule, "starting vehicles follow VehiclesUnlock.tsc");
    rep_.check("N1", c.vehicleOwned(m->vehicle()), "the garage's vehicle " + m->vehicle() + " is owned");
    rep_.note("N1", "owned at start:" + owned);
    // The menus open on the Map tab: its tutorial (13) over the menus (0x46ee10 -> 0x482410), once.
    rep_.check("N2", m->screenName() == "Tab:2", "the menus open on the Map tab (" + m->screenName() + ")");
    const auto shown = okTutorials(*m);
    rep_.check("N2", shown.size() == 1 && shown[0] == 13, "first tutorial: 13 (the map), got " + std::to_string(shown.empty() ? -1 : shown[0]));
    rep_.check("N2", c.state().tutorials[13].seen && c.state().tutorials[13].off && fs::exists(save_), "tutorial 13 seen + off and saved (SECTION_TUTORIAL)");
    m->key(MenuKey::TabLeft);  // Garage: 12
    const auto g = okTutorials(*m);
    std::string gs;
    for (int t : g) gs += " " + std::to_string(t);
    rep_.check("N2", g.size() == 1 && g[0] == 12, "the Garage tab's tutorial 12 (" + m->screenName() + ":" + gs + ")");
    m.reset();
    auto m2 = boot(MenuMode::Main);
    rep_.check("N2", okTutorials(*m2).empty(), "a seen tutorial does not come back after a restart");
}

// ---------------------------------------------------------------------------------------------- 2. first hours

// YellowStone played the way a player does: from the Map tab, each race given up, lost and won, every step a process.
void Audit::firstHours() {
    std::printf("-- first hours (full sessions)\n");
    const CareerRules& R = *rules_;
    const int ys = 0;
    bool firstStart = true;
    for (int mi : R.hubs[size_t(ys)].career) {
        std::string ev = R.missions[size_t(mi)].name;
        for (const Event& e : events_)
            if (same(e.id, ev)) ev = e.id;
        // medals: given up and lost at the preselected bronze row, then won three times at the row the page preselects each
        // time (best rank - 1): bronze, silver, gold, each adding its own medal (0x630dd0).
        int wins = 0;
        for (Plan plan : {Plan::Dnf, Plan::Last, Plan::First, Plan::First, Plan::First}) {
            auto m = boot(MenuMode::Main);
            okTutorials(*m);
            std::string why;
            const bool opened = openEventFromMap(*m, R.hubs[size_t(ys)].name, ev, 1, &why);
            rep_.check("R1", opened, "menu start " + ev + ": " + why);
            if (!opened) { blocked_.insert(ev); break; }
            rep_.check("R1", m->career()->state().hubs[size_t(ys)].opened == 1, "the hub's CAREER list opens the hub (+0x248, 0x481cd0)");
            const MenuRequest r = chooseVehicleAndStart(*m, &why);
            if (firstStart) {
                rep_.check("N2", m->career()->state().tutorials[14].seen && m->career()->state().tutorials[15].seen, "the first race start shows tutorials 14 and 15 (0x4786e0)");
                firstStart = false;
            }
            rep_.check("R1", r.kind == MenuRequest::StartEvent && r.event == ev, "menu start " + ev + ": " + why);
            if (r.kind != MenuRequest::StartEvent) { blocked_.insert(ev); break; }
            startedFromMenu_.insert(ev);
            const int level = sessionLevel(data_, r, *m);
            m->sessionEnd();
            m.reset();
            const char* pn = plan == Plan::Dnf ? "dnf" : plan == Plan::Last ? "last" : "first";
            SessionOut o = raceSession(ev, r.vehicle, plan, true, pn, level);
            if (plan == Plan::First) {
                rep_.check("R3", o.run.ended && o.run.place == 1 && o.x.counted, ev + " finished first: " + o.run.reason);
                const int want = 2 - wins++;  // bronze, silver, gold
                Career sc(*rules_);
                sc.load(save_);  // career.sav as the next process reads it
                const MissionStatus* st = sc.status(ev);
                rep_.check("R6", level == want && st && st->bestRank == want && st->medal[want] && o.award.newMedals == 1 && o.award.medalNew[want],
                           ev + " won at the preselected row " + std::to_string(level) + " (FUEL " + std::to_string(want) + "): that medal only");
            }
            if (plan == Plan::Last && o.run.ended)
                rep_.check("R4", o.run.place == int(o.run.rows.size()), ev + " last: place " + std::to_string(o.run.place) + "/" + std::to_string(o.run.rows.size()));
            if (plan == Plan::Dnf) rep_.check("R5", !o.x.counted, ev + " DNF: nothing won (" + (o.quit ? std::string("quit") : o.run.reason) + ")");
        }
    }
    auto m = boot(MenuMode::Main);
    Career& c = *m->career();
    rep_.check("U1", c.stars() == 3 * int(R.hubs[size_t(ys)].career.size()), "all YellowStone golds: " + std::to_string(c.stars()) + " stars");
    for (size_t h = 0; h < R.hubs.size(); ++h)
        rep_.check("U1", c.hubUnlocked(int(h)) == (R.hubs[h].starsNeeded <= c.stars()),
                   R.hubs[h].name + " (needs " + std::to_string(R.hubs[h].starsNeeded) + ") unlocked = " + std::to_string(c.hubUnlocked(int(h))));
}

// ---------------------------------------------------------------------------------------------- 3. the whole career

void Audit::wholeCareer() {
    std::printf("-- the whole career (every career race and challenge)\n");
    const CareerRules& R = *rules_;
    auto m = boot(MenuMode::Main);
    okTutorials(*m);
    std::set<size_t> done;
    for (int pass = 0; pass < 30 && done.size() < R.hubs.size(); ++pass) {
        bool progress = false;
        for (size_t h = 0; h < R.hubs.size(); ++h) {
            m->career()->load();
            if (done.count(h) || !m->career()->hubUnlocked(int(h))) continue;
            progress = true;
            done.insert(h);
            // Challenges past the first three are discovered in free roam near their start (0x4c0150): drive there
            // once the hub is opened (entered from the map; the CAREER list below does that too).
            {
                Career& c = *m->career();
                std::vector<std::pair<std::string, std::array<float, 2>>> starts;
                for (const Event& e : m->events()) starts.push_back({e.id, {e.race.start[0], e.race.start[2]}});
                FreeRoam fr(c, [&](const std::string& n, float& x, float& z) {  // as FreeRoamView's
                    for (const auto& [id, p] : starts)
                        if (same(id, n)) return x = p[0], z = p[1], true;
                    return false;
                });
                c.enterHub(int(h));
                c.setCurrentHub(int(h));  // a free-roam session in the hub (TeleportToHub, game +0x33c0)
                for (int k : R.hubs[h].challenges) {
                    float x = 0, z = 0;
                    for (const auto& [id, p] : starts)
                        if (same(id, R.missions[size_t(k)].name)) x = p[0], z = p[1];
                    FreeRoamFrame f;
                    f.dt = 0.1f, f.pos = {x, 0, z};
                    for (int i = 0; i < 3; ++i) fr.update(f);
                }
                bool all = true;
                for (const auto& st : c.state().challenges[h]) all = all && st.available;
                rep_.check("U10", all, R.hubs[h].name + ": every challenge discovered by driving to its start (0x4c0150)");
                // 0x4c0150 does not save [O]: the session's end keeps it (S5).
                c.save();
            }
            for (int list : {1, 2}) {
                const auto& ids = list == 1 ? R.hubs[h].career : R.hubs[h].challenges;
                for (int mi : ids) {
                    const std::string ev = eventId(*m, R.missions[size_t(mi)].name);
                    m->career()->load();  // the menus' process after the last session
                    m->setMode(MenuMode::Main);
                    std::string why;
                    if (!openEventFromMap(*m, R.hubs[h].name, ev, list, &why)) {
                        rep_.check("X1", false, "start " + ev + " from the menu: " + why);
                        blocked_.insert(ev);
                        continue;
                    }
                    if (list == 1) m->key(MenuKey::Up), m->key(MenuKey::Up);  // medals: the gold row (0x46bd5f)
                    const MenuRequest r = chooseVehicleAndStart(*m, &why);
                    const int level = sessionLevel(data_, r, *m);
                    if (r.kind != MenuRequest::StartEvent) {
                        rep_.check("X1", false, "start " + ev + ": " + why);
                        blocked_.insert(ev);
                        continue;
                    }
                    startedFromMenu_.insert(ev);
                    m->sessionEnd();
                    int mode = -1;
                    for (const Event& e : m->events())
                        if (e.id == ev) mode = e.race.gameMode();
                    // One race of each game mode is also given up and lost.
                    if (!modeDnf_.count(mode)) {
                        modeDnf_.insert(mode);
                        SessionOut d = raceSession(ev, r.vehicle, Plan::Dnf, false, "dnf", level);
                        rep_.check("R5", !d.x.counted, ev + " (" + gameModeName(mode) + ") DNF: nothing won (" + (d.quit ? std::string("quit") : d.run.reason) + ")");
                    }
                    if (!modeLast_.count(mode)) {
                        modeLast_.insert(mode);
                        SessionOut l = raceSession(ev, r.vehicle, Plan::Last, false, "last", level);
                        if (l.run.ended && l.run.rows.size() > 1 && mode != Race::GmCatchAI)  // hunts have no order to be last in
                            rep_.check("R4", l.run.place == int(l.run.rows.size()), ev + " (" + gameModeName(mode) + ") last: place " + std::to_string(l.run.place));
                    }
                    SessionOut o = raceSession(ev, r.vehicle, Plan::First, false, "first", level);
                    rep_.check("R3", o.run.ended && o.x.counted && !o.run.stuck,
                               ev + " (" + gameModeName(mode) + ") won: " + (o.run.stuck ? std::string("autopilot stuck at a gate") : o.run.reason));
                    if (o.x.counted) {  // the same race again: nothing more (no reward twice)
                        SessionOut a = raceSession(ev, r.vehicle, Plan::First, false, "again", level);
                        rep_.check("X3", a.x.fuel == 0 && a.award.fuelWon == 0 && a.award.newMedals == 0 && a.award.hubsUnlocked.empty() &&
                                             a.award.pilotLiveries.empty() && a.x.gift < 0,
                                   ev + " won again gives nothing new");
                    }
                }
            }
        }
        if (!progress) break;
    }
    for (size_t h = 0; h < R.hubs.size(); ++h)
        rep_.check("X3", done.count(h) != 0, R.hubs[h].name + " reached in the career (needs " + std::to_string(R.hubs[h].starsNeeded) + " stars)");
}

// ---------------------------------------------------------------------------------------------- arches

void Audit::archStarts(bool fresh) {
    std::printf("-- challenge arches (free-roam starts)\n");
    const CareerRules& R = *rules_;
    if (fresh) {
        // A new career: YellowStone unlocked but not entered from the map -> no prompt (0x4f6ed0: zone hub +0x248).
        const auto keep = readFile(save_);
        fs::remove(save_);
        {
            auto m = boot(MenuMode::Pause);
            m->setSession("", R.hubs[0].name, "");
            const Event* e0 = nullptr;
            for (const Event& e : m->events())
                if (findHub(m->hubs(), e.hub) == findHub(m->hubs(), R.hubs[0].name) && e.playable && e.kind == Event::Career) {
                    e0 = &e;
                    break;
                }
            rep_.check("R2", e0 != nullptr, "a YellowStone career race");
            if (e0) {
                HudState hud;
                hud.px = e0->race.start[0], hud.pz = e0->race.start[2];
                m->hudTick(hud);
                rep_.check("R2", m->archEvent() < 0, "no arch prompt in a hub not entered from the map yet (0x4f6ed0 +0x248)");
                m->career()->enterHub(0);
                m->hudTick(hud);
                rep_.check("R2", m->archEvent() >= 0 && m->events()[size_t(m->archEvent())].id == e0->id, "the arch prompt at " + e0->id + " once the hub is opened");
                hud.race = true;
                m->hudTick(hud);
                rep_.check("R2", m->archEvent() < 0, "no arch prompt during a race");
            }
        }
        if (!keep.empty()) writeFile(save_, keep);
        return;
    }
    // Every listed event from its arch, on the played career.
    auto m = boot(MenuMode::Pause);
    int ok = 0, n = 0;
    for (const Event& e : m->events()) {
        if (!e.playable || !m->career()->locate(e.id, nullptr, nullptr, nullptr)) continue;
        ++n;
        m->setMode(MenuMode::Pause);
        m->setSession("", e.hub, "");
        HudState hud;
        hud.px = e.race.start[0], hud.pz = e.race.start[2];
        m->hudTick(hud);
        if (m->archEvent() < 0) {
            rep_.check("R2", false, "no arch prompt at the start of " + e.id);
            continue;
        }
        const std::string at = m->events()[size_t(m->archEvent())].id;
        if (!m->archAccept()) {
            rep_.check("R2", false, "arch accept refused at " + e.id);
            continue;
        }
        std::string why;
        const MenuRequest r = chooseVehicleAndStart(*m, &why, false);
        const bool good = r.kind == MenuRequest::StartEvent && r.event == at;
        ok += good;
        rep_.check("R2", good, "arch start " + at + ": " + why);
    }
    std::printf("   %d of %d events start from their arch\n", ok, n);
}

// ---------------------------------------------------------------------------------------------- PK02: race pages

void Audit::racePages() {
    std::printf("-- race pages\n");
    auto m = boot(MenuMode::Main);
    okTutorials(*m);
    Career& c = *m->career();
    const CareerRules& R = c.rules();
    const std::string hub = R.hubs[0].name;
    std::string why;
    // A career race: Accept on the row opens the page (not the garage), Back returns to the list on the same row.
    const std::string ev = R.missions[size_t(R.hubs[0].career.front())].name;
    const std::string evId = eventId(*m, ev);
    const bool open = openEventFromMap(*m, hub, ev, 1, &why);
    rep_.check("P2", open && m->screenName() == "RacePage", "CAREER row Accept opens the race page: " + why);
    if (open) {
        rep_.check("P2", m->pendingEvent() == evId, "the page is of the row's race");
        const int row0 = m->raceMedalRow();
        m->key(MenuKey::Down), m->key(MenuKey::Down), m->key(MenuKey::Down);
        rep_.check("P2", m->raceMedalRow() == 2, "Down stops at the last medal row (no wrap)");
        for (int g = 0; g < 4; ++g) m->key(MenuKey::Up);
        rep_.check("P2", m->raceMedalRow() == 0, "Up stops at the first medal row (no wrap)");
        (void)row0;
        m->key(MenuKey::Back);
        rep_.check("P2", m->screenName() == "CareerList" && m->pendingEvent().empty(), "Back returns to the CAREER list (0x47e310 -> 0x55)");
        m->key(MenuKey::Accept);
        m->key(MenuKey::Accept);
        rep_.check("P2", m->screenName() == "VehicleList" || m->screenName() == "RacePage", "Accept on the page opens the vehicle pick (" + m->screenName() + ")");
        if (m->screenName() == "VehicleList") {
            m->key(MenuKey::Back);
            rep_.check("P2", m->screenName() == "RacePage" && m->pendingEvent() == evId, "Back in the vehicle pick returns to the race page (0x47f120)");
        }
    }
    // A challenge page: the fuel reward, and the warning that takes Accept away.
    std::string chEv;
    for (const Event& e : m->events())
        if (e.kind == Event::Challenge && e.playable && same(e.hub, hub) && c.eventAvailable(e.id)) {
            chEv = e.id;
            break;
        }
    rep_.check("P2", !chEv.empty(), "a challenge of the first hub is listed");
    if (!chEv.empty()) {
        Career& cc = *m->career();
        cc.enterHub(0);
        const bool o2 = openEventFromMap(*m, hub, chEv, 2, &why);
        rep_.check("P2", o2 && m->screenName() == "RacePage", "CHALLENGES row Accept opens the challenge page: " + why);
        if (o2) {
            rep_.check("P2", !m->raceFuelReward().empty(), "an unwon challenge shows GROUP_FUEL_REWARD");
            m->setNoVehicleShot(true);
            rep_.check("P2", m->raceWarning(), "no allowed vehicle: GROUP_CHALLENGE_WARNING shows");
            m->key(MenuKey::Accept);
            rep_.check("P2", m->screenName() == "RacePage" && m->take().kind == MenuRequest::None, "no Accept on a page with the warning (0x47ed60: +0x959 == 0)");
            m->setNoVehicleShot(false);
            m->key(MenuKey::Back);
            rep_.check("P2", m->screenName() == "ChallengeList", "Back returns to the CHALLENGES list (0x47ed60 -> 0x58)");
        }
    }
    // The vehicle lists of the races with forbidden lists (EMD_SetPlayerForbiddenVehiculeFilter): every career race with a
    // non-empty list, once the whole table is owned.
    int lists = 0;
    for (const Event& e : m->events()) {
        if (e.kind != Event::Career || e.race.forbidden.empty()) continue;
        ++lists;
    }
    rep_.check("P3", lists >= 20, "the parser read the forbidden lists of " + std::to_string(lists) + " career races (miss_official.tsc has 27)");
    {
        auto g = boot(MenuMode::Main);
        okTutorials(*g);
        Career& gc = *g->career();
        for (auto& v : gc.state().vehicles) v.unlocked = v.owned = 1;
        for (auto& h : gc.state().hubs) h.unlocked = 1;
        for (const Event& e : g->events()) {
            if (e.kind != Event::Career || e.race.forbidden.empty() || !e.playable) continue;
            std::string w;
            if (!openEventFromMap(*g, R.hubs[size_t(std::max(0, g->careerHubOf(e.hub)))].name, e.id, 1, &w)) continue;  // the hub may be locked
            g->key(MenuKey::Accept);
            if (g->screenName() != "VehicleList") continue;
            const auto rows = g->listRows();
            bool clean = true;
            for (const std::string& name : rows)
                for (size_t vi = 0; vi < g->vehicles().size(); ++vi)
                    if (g->vehicles()[vi].name == name && std::find(e.race.forbidden.begin(), e.race.forbidden.end(), int(vi)) != e.race.forbidden.end()) clean = false;
            rep_.check("P3", clean, e.id + ": the vehicle list (" + std::to_string(rows.size()) + " rows) holds no vehicle of its forbidden list");
            ++forbiddenLists_;
        }
    }
    // Damocles: an owned BONUS_ROAD_3 in the free-ride garage opens the popup (TT_PAGE_VEHICLE_DAMOCLES) and is not taken.
    {
        auto g = boot(MenuMode::Main);
        okTutorials(*g);
        Career& gc = *g->career();
        const int vi = gc.rules().vehicleIndex("BONUS_ROAD_3");
        rep_.check("P4", vi >= 0, "BONUS_ROAD_3 is a career vehicle");
        if (vi >= 0) {
            gc.state().vehicles[size_t(vi)].unlocked = gc.state().vehicles[size_t(vi)].owned = 1;
            for (int t = 0; t < 6 && g->screenName() != "Tab:1"; ++t) g->key(MenuKey::TabLeft);
            focusLeft(*g, "ITEM_ID_TOPMENU_GARAGE_VEHICLE");
            g->key(MenuKey::Accept);
            const auto rows = g->listRows();
            const auto it = std::find(rows.begin(), rows.end(), "BONUS_ROAD_3");
            rep_.check("P4", it != rows.end(), "the owned Damocles is on the free-ride garage list");
            if (it != rows.end()) {
                const std::string before = g->vehicle();
                for (int t = 0; t < 80 && g->focusIndex() != int(it - rows.begin()); ++t) g->key(g->focusIndex() < int(it - rows.begin()) ? MenuKey::Right : MenuKey::Left);
                g->key(MenuKey::Accept);
                rep_.check("P4", g->confirmOpen() && g->confirmText() == "TT_PAGE_VEHICLE_DAMOCLES", "accepting the Damocles outside the TTCsaltlake garage opens TT 5981 (" + g->confirmText() + ")");
                rep_.check("P4", g->vehicle() == before, "the free-ride vehicle did not change");
                g->key(MenuKey::Accept);
                rep_.check("P4", !g->confirmOpen() && g->screenName() == "VehicleList", "the popup's OK returns to the garage list");
            }
        }
    }
    std::printf("   %d race pages checked, %d with the fuel reward, %d with the warning, %d forbidden lists\n", racePages_, fuelRewardPages_, warningPages_, forbiddenLists_);
}

// ---------------------------------------------------------------------------------------------- 4. economy

// PK11: the hub menu's item states (0x47ca70, 0x481f80), its help key (0x47cbf0), the camps list keys (0x47e610) and the
// View 13 camera slots (0x578f20 / 0x578f60).
void Audit::hubMenu() {
    std::printf("-- hub menu\n");
    auto m = boot(MenuMode::Main);
    okTutorials(*m);
    Career& c = *m->career();
    const CareerRules& R = c.rules();
    for (int g = 0; g < 6 && m->screenName() != "Tab:2"; ++g) m->key(MenuKey::TabRight);
    for (int g = 0; g < 6 && m->screenName() != "Tab:2"; ++g) m->key(MenuKey::TabLeft);
    auto has = [&](Menu& mm, const char* id) {
        const auto ids = mm.leftMenuIds();
        return std::find(ids.begin(), ids.end(), id) != ids.end();
    };
    int locked = -1;
    if (R.hubs.size() > 2) c.state().hubs[2].unlocked = 0;  // in memory: the audit's save may have opened every hub by now
    for (size_t h = 0; h < R.hubs.size(); ++h)
        if (!c.hubUnlocked(int(h))) { locked = int(h); break; }
    rep_.check("H1", locked >= 0 && selectHub(*m, R.hubs[size_t(locked)].name), "a locked hub is on the map");
    rep_.check("H1", !has(*m, "ITEM_ID_LEFT_MENU5") && !has(*m, "ITEM_ID_LEFT_MENU4") && m->hubHelpKey() == "TT_BH_HUB_LOCKED_AND_NOT_VISITED",
               "locked hub: CAREER / CHALLENGES off, help key " + m->hubHelpKey());
    // Unlocked but never entered (+0x248 == 0): off, TT_BH_HUB_OPEN_AND_NOT_VISITED; entered: on, TT_BH_MAIN. Needs a session
    // (the title menus have no hub context [port]); the hub the session runs in counts as entered (arrival 0x481cd0, PK03).
    {
        auto ps = boot(MenuMode::Pause);
        okTutorials(*ps);
        Career& pc = *ps->career();
        ps->setSession("", R.hubs[0].name, "");
        pc.state().hubs[0].opened = 0;
        pc.enterHub(0);  // PK03: the session's arrival in its hub (StartHubHUD -> 0x481cd0) sets +0x248
        pc.state().hubs[1].unlocked = 1, pc.state().hubs[1].opened = 0;
        for (int g = 0; g < 6 && ps->screenName() != "Tab:2"; ++g) ps->key(MenuKey::TabRight);
        for (int g = 0; g < 6 && ps->screenName() != "Tab:2"; ++g) ps->key(MenuKey::TabLeft);
        rep_.check("H1", selectHub(*ps, R.hubs[0].name) && has(*ps, "ITEM_ID_LEFT_MENU5") && ps->hubHelpKey() == "TT_BH_MAIN",
                   "the session's own hub: CAREER on (arrival), help key " + ps->hubHelpKey());
        rep_.check("H1", selectHub(*ps, R.hubs[1].name), "hub 1 on the map");
        rep_.check("H1", !has(*ps, "ITEM_ID_LEFT_MENU5") && !has(*ps, "ITEM_ID_LEFT_MENU4") && ps->hubHelpKey() == "TT_BH_HUB_OPEN_AND_NOT_VISITED",
                   "unlocked, not entered: CAREER / CHALLENGES off, help key " + ps->hubHelpKey());
        rep_.check("H1", has(*ps, "ITEM_ID_LEFT_MENU3") && has(*ps, "ITEM_ID_LEFT_MENU2"), "CAMPS and DATA stay on");
        pc.enterHub(1);
        rep_.check("H1", has(*ps, "ITEM_ID_LEFT_MENU5") && has(*ps, "ITEM_ID_LEFT_MENU4") && ps->hubHelpKey() == "TT_BH_MAIN",
                   "entered: CAREER / CHALLENGES on, help key " + ps->hubHelpKey());
    }
    selectHub(*m, R.hubs[0].name);
    m->setGameMode(1);
    rep_.check("H1", has(*m, "ITEM_ID_LEFT_MENU5") && !has(*m, "ITEM_ID_LEFT_MENU4"), "demo preview (game+0x32bc == 1): CHALLENGES off");
    m->setGameMode(2);
    rep_.check("H1", !has(*m, "ITEM_ID_LEFT_MENU3") && !has(*m, "ITEM_ID_LEFT_MENU2") && has(*m, "ITEM_ID_LEFT_MENU5"), "tutorial mode (== 2): CAMPS and DATA off");
    m->setGameMode(0);
    rep_.check("H1", has(*m, "ITEM_ID_LEFT_MENU3") && has(*m, "ITEM_ID_LEFT_MENU2") && has(*m, "ITEM_ID_LEFT_MENU4"), "normal mode: all items on");
    const auto s0 = Menu::hubCameraSlot(0), s1 = Menu::hubCameraSlot(1), s2 = Menu::hubCameraSlot(2), s3 = Menu::hubCameraSlot(3);
    rep_.check("H1", s0.pos == 0x1a8 && s0.target == 0x1b4 && s1.pos == 0x1f0 && s1.target == 0x1fc && s2.pos == 0x1d8 && s2.target == 0x1e4 &&
                         s3.pos == 0x1c0 && s3.target == 0x1cc && Menu::hubCameraSlot(9).pos == 0x1a8,
               "View 13 camera slots by view state (hub, career, challenge, shop)");
    // Camps list: Up/Down stay inside the page and the shown (unlocked) rows, Left/Right jump between the two pages.
    if (focusLeft(*m, "ITEM_ID_LEFT_MENU3")) {
        m->key(MenuKey::Accept);
        rep_.check("H2", m->screenName() == "CampList", "Map > camps opens the list");
        const auto rows = m->listRows();
        int shown = 0;
        for (size_t h = 0; h < R.hubs.size() && h < 10; ++h) {
            if (!c.hubUnlocked(int(h))) break;
            ++shown;
        }
        for (int g = 0; g < 12; ++g) m->key(MenuKey::Up);
        const int top = m->focusIndex();
        for (int g = 0; g < 14; ++g) m->key(MenuKey::Down);
        const int bottom = m->focusIndex();
        rep_.check("H2", top == 0 && bottom == std::max(0, shown - 1) && bottom < 10,
                   "Up/Down clamp inside the page and the unlocked rows (top " + std::to_string(top) + ", bottom " + std::to_string(bottom) + ", unlocked " + std::to_string(shown) + ")");
        if (rows.size() > 10) {
            m->key(MenuKey::Right);
            const int p2 = m->focusIndex();
            m->key(MenuKey::Left);
            rep_.check("H2", p2 == 10 && m->focusIndex() == 0, "Right -> row 10, Left -> row 0");
        }
        m->key(MenuKey::Back);
        rep_.check("H2", m->screenName() == "Tab:2", "Back returns to the hub menu");
    } else rep_.check("H2", false, "camps item not focusable");
}

void Audit::economy() {
    std::printf("-- economy\n");
    auto m = boot(MenuMode::Main);
    okTutorials(*m);
    Career& c = *m->career();
    const CareerRules& R = c.rules();
    for (int g = 0; g < 6 && m->screenName() != "Tab:1"; ++g) m->key(MenuKey::TabLeft);
    okTutorials(*m);
    rep_.check("E1", m->screenName() == "Tab:1", "Garage tab (" + m->screenName() + ")");
    rep_.check("E1", focusLeft(*m, "ITEM_ID_TOPMENU_GARAGE_VEHICLE"), "Garage > VEHICLE");
    m->key(MenuKey::Accept);
    rep_.check("E1", m->screenName() == "VehicleList", "the vehicle carousel opens");
    const auto rows = m->listRows();
    bool allUnlocked = true;
    for (const auto& r : rows) allUnlocked = allUnlocked && (R.vehicleIndex(r) < 0 || c.vehicleUnlocked(r));
    rep_.check("U2", allUnlocked, "the garage lists unlocked vehicles only (0x47a7d0)");
    for (const auto& u : R.unlocks)
        if (u.kind == "VEHICLE" && u.cond == "HUB") {
            const int h = R.hubIndex(u.arg);
            if (h >= 0 && c.hubUnlocked(h)) rep_.check("U2", c.vehicleUnlocked(u.target), u.target + " for sale once " + u.arg + " is open (UnlockNeed HUB)");
        }
    std::string dear;
    for (const auto& v : R.vehicles)
        if (v.cls >= 0 && v.price > 0 && c.vehicleUnlocked(v.label) && !c.vehicleOwned(v.label) && std::find(rows.begin(), rows.end(), v.label) != rows.end()) {
            dear = v.label;
            break;
        }
    if (!dear.empty()) {
        const uint32_t keep = c.state().fuel;
        const uint32_t price = uint32_t(c.vehiclePrice(dear));
        c.state().fuel = price - 1;
        const int i = int(std::find(rows.begin(), rows.end(), dear) - rows.begin());
        for (int g = 0; g < 80 && m->focusIndex() != i; ++g) m->key(m->focusIndex() < i ? MenuKey::Right : MenuKey::Left);
        m->key(MenuKey::Accept);
        rep_.check("E2", m->infoBox() == "InfoBoxGarageVehicleIsTooExpensive", dear + " (" + std::to_string(price) + ") with " + std::to_string(c.state().fuel) + " fuel: " + m->infoBox());
        m->key(MenuKey::Accept);
        rep_.check("E2", !c.vehicleOwned(dear) && c.state().fuel == price - 1, "too expensive: nothing bought");
        c.state().fuel = price;
        const int saves0 = m->saveCount();
        m->key(MenuKey::Accept);
        rep_.check("E1", m->infoBox() == "InfoBoxGarageYesNo", "with the price: the yes/no box (" + m->infoBox() + ")");
        m->key(MenuKey::Accept);
        rep_.check("E1", c.vehicleOwned(dear) && c.state().fuel == 0 && m->saveCount() == saves0 + 1 && c.state().currentVehicle() == R.vehicleIndex(dear),
                   "bought " + dear + ": fuel 0, the current vehicle (+0x33c8), saved");
        bool liv = true;
        for (size_t h = 0; h < R.hubs.size(); ++h)
            for (size_t k = 0; k < R.hubs[h].liveries.size(); ++k)
                if (R.hubs[h].liveries[k].kind == "ALWAYS" && same(R.hubs[h].liveries[k].mesh, R.vehicles[size_t(R.vehicleIndex(dear))].mesh))
                    liv = liv && c.state().liveries[h][k].b;
        rep_.check("U5", liv, "the bought vehicle's ALWAYS liveries are owned (0x569b20)");
        m->key(MenuKey::Accept);  // owned now: chosen
        rep_.check("E1", m->vehicle() == dear, "the bought vehicle is the garage's choice");
        c.state().fuel = keep;
        c.save();
        m.reset();
        auto m2 = boot(MenuMode::Main);
        rep_.check("E1", m2->career()->vehicleOwned(dear) && m2->vehicle() == dear, "after a restart the bought vehicle is owned and chosen (+0x33c8)");
    } else {
        rep_.check("E2", false, "no unowned priced vehicle in the garage to test");
    }
    Career fresh(*rules_);
    std::string gift;
    for (const auto& v : rules_->vehicles)
        if (v.cls >= 0 && v.price < 0 && !fresh.vehicleOwned(v.label)) { gift = v.label; break; }
    if (!gift.empty()) {
        fresh.state().fuel = 99999999;
        rep_.check("E2", fresh.buyVehicle(gift) != Career::Bought && !fresh.vehicleOwned(gift), "gift-only " + gift + " (price -1) cannot be bought");
    }
    std::string locked;
    for (const auto& v : rules_->vehicles)
        if (v.cls >= 0 && v.price > 0 && !fresh.vehicleUnlocked(v.label)) { locked = v.label; break; }
    if (!locked.empty()) rep_.check("E2", fresh.buyVehicle(locked) == Career::Locked, "locked " + locked + " cannot be bought");
    rep_.note("E1", "FUEL sells no liveries (ALWAYS ones come with the vehicle, POI pickups, CHEAT codes)");
}

// ------------------------------------------------------------------------- [DEVIATION upgrades U2] vehicle upgrades
// FUEL has no upgrades (port deviation, notes upgrades-plan.md 3 / 7, upg2.md). After the whole career (every star): a career
// race of the first hub started from the Map, stages bought on the vehicle the garage picks (prices and gates against the
// plan's table: level x category factor, rounded to 50; stage 2 = 20 stars, stage 3 = 45), and the session's arguments
// (sessionArgs = what execSession execs) carry the equipped stages; a restart keeps them; `upgrades = off`, a challenge
// that hands over its vehicle and a vehicle not owned run stock; free ride carries them too.
void Audit::upgrades() {
    std::printf("-- upgrades\n");
    const CareerRules& R = *rules_;
    auto model = [](int level, int cat, int stage) {  // plan 3.2
        static const int base[5][3] = {{500, 1500, 4000}, {800, 2500, 6500}, {1200, 4000, 10000}, {2000, 6500, 16000}, {3000, 10000, 25000}};
        static const double factor[6] = {1.40, 1.00, 1.00, 1.10, 0.90, 0.85};
        return int(std::lround(base[std::clamp(level, 1, 5) - 1][stage - 1] * factor[cat] / 50.0)) * 50;
    };
    auto stagesArg = [](const std::vector<std::string>& args) {
        for (size_t i = 0; i + 1 < args.size(); ++i)
            if (args[i] == "--upgrades") return args[i + 1];
        return std::string("(none)");
    };
    std::string ev = R.missions[size_t(R.hubs[0].career.back())].name;
    for (const Event& e : events_)
        if (same(e.id, ev)) ev = e.id;
    auto m = boot(MenuMode::Main);
    okTutorials(*m);
    std::string why;
    const bool opened = openEventFromMap(*m, R.hubs[0].name, ev, 1, &why);
    const MenuRequest r = opened ? chooseVehicleAndStart(*m, &why, false) : MenuRequest{};
    rep_.check("E3", r.kind == MenuRequest::StartEvent && !r.vehicle.empty(), "menu start " + ev + " (" + r.vehicle + "): " + why);
    if (r.kind != MenuRequest::StartEvent) return;
    Career& c = *m->career();
    const std::string v = r.vehicle;
    const int vi = R.vehicleIndex(v);
    const int level = vi >= 0 ? R.vehicles[size_t(vi)].level : -1;
    rep_.check("E3", c.vehicleOwned(v) && c.upgradable(v) && level >= 1 && c.stars() >= 45, v + ": owned, upgradable, level " + std::to_string(level) +
                                                                                          ", " + std::to_string(c.stars()) + " stars");
    rep_.check("E3", stagesArg(sessionArgs(data_, r, *m)) == "0,0,0,0,0,0", "before any purchase the session runs stock (" + stagesArg(sessionArgs(data_, r, *m)) + ")");
    const uint32_t keep = c.state().fuel;
    const int e123 = model(level, upg::Engine, 1) + model(level, upg::Engine, 2) + model(level, upg::Engine, 3);
    c.state().fuel = uint32_t(e123 - 1);
    rep_.check("E3", c.buyUpgrade(v, upg::Engine, 3) == Career::UpgradeResult::TooExpensive && c.state().fuel == uint32_t(e123 - 1) && c.upgradesOwned(v).stock(),
               "engine stage 3 (" + std::to_string(e123) + " = stages 1-3) with one fuel too few: too expensive, nothing spent");
    c.state().fuel = uint32_t(e123);
    rep_.check("E3", c.buyUpgrade(v, upg::Engine, 3) == Career::UpgradeResult::Bought && c.state().fuel == 0 && c.upgradesOwned(v)[upg::Engine] == 3,
               "engine stage 3 bought with stages 1-2 for " + std::to_string(e123) + ", fuel 0");
    c.state().fuel = uint32_t(model(level, upg::Suspension, 1));
    rep_.check("E3", c.buyUpgrade(v, upg::Suspension, 1) == Career::UpgradeResult::Bought && c.state().fuel == 0, "suspension stage 1 for its price");
    rep_.check("E3", c.equipUpgrade(v, upg::Engine, 2) && !c.equipUpgrade(v, upg::Tires, 1) && c.state().fuel == 0,
               "engine stage 2 equipped for free; tires stage 1 not owned");
    c.state().fuel = keep;
    rep_.check("E3", c.save(), "saved");
    const std::string want = "2,1,0,0,0,0";
    rep_.check("E3", stagesArg(sessionArgs(data_, r, *m)) == want, ev + " with " + v + ": the session gets --upgrades " + stagesArg(sessionArgs(data_, r, *m)) + " (" + want + ")");
    MenuRequest fr;
    fr.kind = MenuRequest::FreeRide, fr.hub = m->hubs().empty() ? std::string() : m->hubs().front().id, fr.vehicle = v;
    rep_.check("E3", stagesArg(sessionArgs(data_, fr, *m)) == want, "free ride with " + v + ": --upgrades " + stagesArg(sessionArgs(data_, fr, *m)));
    // A challenge that hands over its own vehicle runs stock even when the player owns that vehicle with stages.
    const Event* forced = nullptr;
    for (const Event& e : m->events())
        if (!forced && e.kind == Event::Challenge && !e.vehicle.empty() && c.upgradable(e.vehicle) && c.vehicleOwned(e.vehicle)) forced = &e;
    if (forced) {
        c.state().fuel = 1000000;
        c.buyUpgrade(forced->vehicle, upg::Tires, 1);
        c.state().fuel = keep;
        MenuRequest cr;
        cr.kind = MenuRequest::StartEvent, cr.event = forced->id, cr.vehicle = forced->vehicle;
        MenuRequest cf = fr;
        cf.vehicle = forced->vehicle;
        rep_.check("E3", stagesArg(sessionArgs(data_, cr, *m)) == "0,0,0,0,0,0" && stagesArg(sessionArgs(data_, cf, *m)) == "0,0,1,0,0,0",
                   "challenge " + forced->id + " hands over " + forced->vehicle + ": stock (" + stagesArg(sessionArgs(data_, cr, *m)) +
                       "); the same vehicle in free ride: " + stagesArg(sessionArgs(data_, cf, *m)));
    } else {
        rep_.check("E3", false, "no challenge with its own owned vehicle to test");
    }
    const std::string forcedText = forced ? "challenge " + forced->id + " with " + forced->vehicle + ": stock" : "no forced challenge";
    c.save();
    m.reset();
    // A restart (the next process) keeps the stages; `upgrades = off` runs stock and keeps them.
    auto m2 = boot(MenuMode::Main);
    rep_.check("E3", upg::formatStages(m2->career()->upgradesOwned(v)) == "3,1,0,0,0,0" && stagesArg(sessionArgs(data_, r, *m2)) == want,
               "after a restart: owned " + upg::formatStages(m2->career()->upgradesOwned(v)) + ", the session gets " + stagesArg(sessionArgs(data_, r, *m2)));
    m2.reset();
    Settings off = settings_;
    off.all["upgrades"] = "off";
    ++boots_;
    Menu m3(data_, off, MenuMode::Main);
    rep_.check("E3", stagesArg(sessionArgs(data_, r, m3)) == "0,0,0,0,0,0" && upg::formatStages(m3.career()->upgradesEquipped(v)) == want,
               "upgrades = off: the session runs stock (" + stagesArg(sessionArgs(data_, r, m3)) + "), the career keeps " + want);
    std::printf("   %s (level %d) in %s: --upgrades %s; %s; upgrades = off: %s\n", v.c_str(), level, ev.c_str(), want.c_str(), forcedText.c_str(),
                stagesArg(sessionArgs(data_, r, m3)).c_str());
    rep_.note("E3", "no garage page yet (U3): bought through Career::buyUpgrade, the page's call");
}

// cr5 (notes cr5-ai.md): the AI's target time is AddMissionTime value `level`, race +0x3e8 [O]: the career page 0x47eee0 runs
// TTMN "<mission>" <index> <medal row +0x88> (preselected best rank - 1, at least 0; Up / Down 0..2), the challenge page
// 0x47efb0 TTMN ... 0. sessionArgs passes it as --ai-level.
void Audit::aiLevel() {
    std::printf("-- ai level\n");
    auto levelArg = [](const std::vector<std::string>& args) {
        for (size_t i = 0; i + 1 < args.size(); ++i)
            if (args[i] == "--ai-level") return args[i + 1];
        return std::string("(none)");
    };
    auto m = boot(MenuMode::Main);
    int checked = 0, wrong = 0;
    for (const Event& e : m->events()) {
        if (e.kind == Event::Edited) continue;
        MenuRequest r;
        r.kind = MenuRequest::StartEvent, r.event = e.id, r.vehicle = e.vehicle.empty() ? "QUAD_ROAD_2" : e.vehicle;
        const std::string got = levelArg(sessionArgs(data_, r, *m)), want = e.kind == Event::Career ? std::to_string(m->raceMedalRow()) : "0";
        ++checked, wrong += got != want;
        if (got != want) std::printf("   %s: --ai-level %s (want %s)\n", e.id.c_str(), got.c_str(), want.c_str());
    }
    rep_.check("E5", checked > 200 && wrong == 0, std::to_string(checked) + " events: career the medal row, challenges 0 (" + std::to_string(wrong) + " wrong)");
    m.reset();
    // The page preselects best rank - 1 (bronze = row 2 for a race not won yet); Up moves to silver and gold, and the session follows.
    auto p = boot(MenuMode::Main);
    okTutorials(*p);
    const CareerRules& R = p->career()->rules();
    const std::string ev = R.missions[size_t(R.hubs[0].career.front())].name, evId = eventId(*p, ev);
    std::string why;
    const bool open = openEventFromMap(*p, R.hubs[0].name, ev, 1, &why);
    rep_.check("E5", open && p->screenName() == "RacePage", "the race page of " + evId + ": " + why);
    if (open) {
        MenuRequest r;
        r.kind = MenuRequest::StartEvent, r.event = evId, r.vehicle = "QUAD_ROAD_2";
        const MissionStatus* ms = p->career()->status(evId);
        const int pre = std::max(0, (ms ? ms->bestRank : 3) - 1);
        std::string seq = levelArg(sessionArgs(data_, r, *p));
        p->key(MenuKey::Up);
        seq += "," + levelArg(sessionArgs(data_, r, *p));
        p->key(MenuKey::Up), p->key(MenuKey::Up);
        seq += "," + levelArg(sessionArgs(data_, r, *p));
        const std::string want = std::to_string(pre) + "," + std::to_string(std::max(pre - 1, 0)) + ",0";
        rep_.check("E5", seq == want, evId + ": the preselected row, then Up, Up Up -> --ai-level " + seq + " (" + want + ")");
    }
}

// [upgrades U4] The opponents' stages (core/upgrades aiDraw; notes upg4.md): the menu's session launch passes the race's hub tier
// (`--ai-upgrade-tier`, an edited race none: the session finds the hub under its start), and the stages the session draws from
// it are a function of the race, the grid slot, the tier and the seed only (the same field on every start and retry).
void Audit::aiUpgrades() {
    std::printf("-- ai upgrades\n");
    auto tierArg = [](const std::vector<std::string>& args) {
        for (size_t i = 0; i + 1 < args.size(); ++i)
            if (args[i] == "--ai-upgrade-tier") return args[i + 1];
        return std::string("(none)");
    };
    auto m = boot(MenuMode::Main);
    int checked = 0, wrong = 0, forced = 0;
    std::string lowEv, highEv;
    int highTier = -1;
    for (const Event& e : m->events()) {
        if (e.kind == Event::Edited) continue;
        MenuRequest r;
        r.kind = MenuRequest::StartEvent, r.event = e.id, r.vehicle = e.vehicle.empty() ? "QUAD_ROAD_2" : e.vehicle;
        // A challenge that hands over its vehicle: the player runs stock (U2), its opponents too (-1).
        const bool handedOver = e.kind == Event::Challenge && !e.vehicle.empty();
        forced += handedOver && !e.hub.empty() && e.tier > 0 && tierArg(sessionArgs(data_, r, *m)) == "-1";
        const std::string got = tierArg(sessionArgs(data_, r, *m)), want = std::to_string(e.hub.empty() || handedOver ? -1 : e.tier);
        ++checked, wrong += got != want;
        if (got != want) std::printf("   %s: --ai-upgrade-tier %s (hub tier %s)\n", e.id.c_str(), got.c_str(), want.c_str());
        if (e.kind == Event::Career && e.tier == 0 && lowEv.empty()) lowEv = e.id;
        if (e.kind == Event::Career && e.tier >= 7 && e.tier > highTier) highEv = e.id, highTier = e.tier;
    }
    rep_.check("E4", checked > 200 && wrong == 0, std::to_string(checked) + " events: the session gets the event's hub tier (" + std::to_string(wrong) + " wrong)");
    rep_.check("E4", forced > 0, std::to_string(forced) + " challenges on a hub above tier 0 that hand over their vehicle: opponents stock (-1)");
    MenuRequest ed;
    ed.kind = MenuRequest::StartEvent, ed.event = "Edited Race 01", ed.vehicle = "QUAD_ROAD_2";
    rep_.check("E4", tierArg(sessionArgs(data_, ed, *m)) == "(none)", "an edited race passes no tier (the session takes the hub under its start)");
    m.reset();
    // The field of a race: drawn twice the same, per slot; another seed or another race gives another field.
    auto field = [](const std::string& race, int tier, uint64_t seed) {
        std::string f;
        for (int k = 0; k < 11; ++k) f += upg::formatStages(upg::aiDraw(race, k, tier, seed).stages) + ";";
        return f;
    };
    const std::string a = field(highEv, highTier, 0), b = field(highEv, highTier, 0), c = field(highEv, highTier, 1), d = field(lowEv, highTier, 0);
    rep_.check("E4", !highEv.empty() && a == b && a != c && a != d,
               highEv + " (hub tier " + std::to_string(highTier) + "): the same field drawn twice; seed 1 and " + lowEv + " give others");
    int rivals = 0, maxStage = 0;
    for (int k = 0; k < 11; ++k)
        for (int s = 0; s < 64; ++s) {
            const upg::AiDraw w = upg::aiDraw(lowEv + std::to_string(s), k, 0, 0);
            rivals += w.rival != 0;
            for (int cat = 0; cat < upg::kCategories; ++cat) maxStage = std::max(maxStage, w.stages[cat]);
        }
    rep_.check("E4", rivals == 0 && maxStage <= 1, "stock-tier hubs: no rivals, stages 0..1 (704 draws: " + std::to_string(rivals) + " rivals, max " +
                                                      std::to_string(maxStage) + ")");
    const bool wasOn = upg::enabled();
    upg::Stages max;
    max.s.fill(3);
    upg::setEnabled(false);
    const bool offIdentity = upg::aiFactors(max, upg::Quad, 1.0f).identity();
    upg::setEnabled(wasOn);
    rep_.check("E4", offIdentity && !upg::aiFactors(max, upg::Quad, 1.0f).identity(), "upgrades = off: opponents' ghosts as FUEL's");
    std::printf("   %s: %s\n", highEv.c_str(), a.c_str());
}

// ---------------------------------------------------------------------------------------------- 5. hub travel

void Audit::travel() {
    std::printf("-- hub travel\n");
    auto m = boot(MenuMode::Main);
    okTutorials(*m);
    Career& c = *m->career();
    const CareerRules& R = c.rules();
    rep_.check("T1", focusLeft(*m, "ITEM_ID_LEFT_MENU3"), "Map > camps item");
    m->key(MenuKey::Accept);
    rep_.check("T1", m->screenName() == "CampList", "the camp list opens (" + m->screenName() + ")");
    const auto rows = m->listRows();
    rep_.check("T1", rows.size() == R.hubs.size(), "the camp list has every hub (" + std::to_string(rows.size()) + ")");
    int target = -1, lockedRow = -1;
    for (size_t i = 0; i < rows.size(); ++i) {
        const int h = R.hubIndex(rows[i]);
        if (h > 0 && c.hubUnlocked(h) && target < 0) target = int(i);
        if (h >= 0 && !c.hubUnlocked(h) && lockedRow < 0) lockedRow = int(i);
    }
    if (target >= 0) {
        for (int g = 0; g < 30 && m->focusIndex() != target; ++g) {
            if (m->focusIndex() < 10 && target >= 10) m->key(MenuKey::Right);
            else m->key(m->focusIndex() < target ? MenuKey::Down : MenuKey::Up);
        }
        m->key(MenuKey::Accept);
        rep_.check("T1", m->confirmOpen() && m->confirmText() == "TT_ARE_YOU_SURE_YOU_WANT_TO_GO_THERE", "travel asks TT_ARE_YOU_SURE_YOU_WANT_TO_GO_THERE");
        confirmYes(*m);
        const MenuRequest r = m->take();
        rep_.check("T1", r.kind == MenuRequest::FreeRide && same(r.hub, rows[size_t(target)]), "camp travel: free ride at " + rows[size_t(target)]);
    } else {
        rep_.check("T1", false, "no unlocked camp to travel to");
    }
    if (lockedRow >= 0) {
        const int h = R.hubIndex(rows[size_t(lockedRow)]);
        // The locked camp's row does nothing (0x481580 does not list it).
        auto m1 = boot(MenuMode::Main);
        okTutorials(*m1);
        focusLeft(*m1, "ITEM_ID_LEFT_MENU3");
        m1->key(MenuKey::Accept);
        for (int g = 0; g < 30 && m1->focusIndex() != lockedRow; ++g) {
            if (m1->focusIndex() < 10 && lockedRow >= 10) m1->key(MenuKey::Right);
            else m1->key(m1->focusIndex() < lockedRow ? MenuKey::Down : MenuKey::Up);
        }
        m1->key(MenuKey::Accept);
        rep_.check("T2", !m1->confirmOpen() && m1->take().kind == MenuRequest::None, "a locked camp cannot be travelled to");
        auto m2 = boot(MenuMode::Main);
        okTutorials(*m2);
        std::string why;
        rep_.check("T2", !openEventFromMap(*m2, R.hubs[size_t(h)].name, R.missions[size_t(R.hubs[size_t(h)].career.front())].name, 1, &why),
                   "a locked hub's CAREER list does not open");
        rep_.check("T2", m2->career()->state().hubs[size_t(h)].opened == 0, "a locked hub is not opened");
    }
    // The map page in free roam (InGamePause INFOS -> the Map tab, 0x46e750): the cursor on an item, the action key
    // (0x482b60): a heliport (type 21) flies there (0x56ca00 + game +0x3460 + 1, saved); a camp (type 6) of an unlocked
    // hub = TeleportToHub (current hub +0x33c0, saved); a locked camp does nothing.
    {
        auto f = boot(MenuMode::Pause);
        Career& fc = *f->career();
        const int cur = fc.currentHub();
        f->setSession("", R.hubs[size_t(cur)].name, f->vehicle());
        auto aim = [&](float x, float z) {
            f->setPlayer(x, z, -1, 0);
            f->openMapTab();
            f->update(0.1f);
        };
        const Hub* here = findHub(f->hubs(), R.hubs[size_t(cur)].name);
        if (fc.state().hubs[size_t(cur)].opened && !R.hubs[size_t(cur)].teleports.empty() && here) {
            const auto& tp = R.hubs[size_t(cur)].teleports[0];
            aim(tp.x, tp.z);
            const auto sel = f->mapSelection();
            rep_.check("T1", sel.first == 21 && sel.second == 0, "map cursor on " + R.hubs[size_t(cur)].name + "'s heliport 0 (" + std::to_string(sel.first) + ")");
            f->setPlayer(float(here->camp.x), float(here->camp.z), -1, 0);  // the player at the camp, the cursor stays
            const int uses = fc.state().heliportUses(), saves0 = f->saveCount();
            f->key(MenuKey::MapAction);
            const MenuRequest r = f->take();
            rep_.check("T1", r.kind == MenuRequest::Teleport && std::fabs(r.x - tp.x) < 1 && std::fabs(r.z - tp.z) < 1 && fc.state().heliportUses() == uses + 1 &&
                                 f->saveCount() == saves0 + 1,
                       "the heliport flies there, counts (+0x3460) and saves");
        }
        int dest = -1, lockedH = -1;
        for (size_t h = 0; h < R.hubs.size(); ++h) {
            if (int(h) != cur && fc.hubUnlocked(int(h)) && dest < 0) dest = int(h);
            if (!fc.hubUnlocked(int(h)) && lockedH < 0) lockedH = int(h);
        }
        if (lockedH >= 0)
            if (const Hub* hb = findHub(f->hubs(), R.hubs[size_t(lockedH)].name)) {
                aim(float(hb->camp.x), float(hb->camp.z));
                rep_.check("T2", f->mapSelection() == std::make_pair(6, lockedH), "map cursor on the locked camp of " + R.hubs[size_t(lockedH)].name);
                f->key(MenuKey::MapAction);
                rep_.check("T2", f->take().kind == MenuRequest::None && fc.currentHub() == cur, "a locked camp on the map does nothing");
            }
        if (dest >= 0)
            if (const Hub* hb = findHub(f->hubs(), R.hubs[size_t(dest)].name)) {
                aim(float(hb->camp.x), float(hb->camp.z));
                rep_.check("T1", f->mapSelection() == std::make_pair(6, dest), "map cursor on the camp of " + R.hubs[size_t(dest)].name);
                const int saves0 = f->saveCount();
                f->key(MenuKey::MapAction);
                const MenuRequest r = f->take();
                rep_.check("T1", r.kind == MenuRequest::FreeRide && same(r.hub, R.hubs[size_t(dest)].name) && fc.currentHub() == dest && f->saveCount() == saves0 + 1,
                           "the map's camp: free ride at " + R.hubs[size_t(dest)].name + ", current hub, saved");
                f->sessionEnd();
                auto g = boot(MenuMode::Main);
                rep_.check("T1", g->career()->currentHub() == dest, "the next process starts in " + R.hubs[size_t(dest)].name);
            }
    }
}

// ---------------------------------------------------------------------------------------------- free roam

void Audit::freeRoam() {
    std::printf("-- free roam collectibles\n");
    auto m = boot(MenuMode::Pause);
    Career& c = *m->career();
    const CareerRules& R = c.rules();
    m->setSession("", R.hubs[0].name, m->vehicle());
    std::vector<std::pair<std::string, std::array<float, 2>>> starts;
    for (const Event& e : m->events()) starts.push_back({e.id, {e.race.start[0], e.race.start[2]}});
    FreeRoam fr(c, [&](const std::string& n, float& x, float& z) {
        for (const auto& [id, p] : starts)
            if (same(id, n)) return x = p[0], z = p[1], true;
        return false;
    });
    m->setFreeRoam(&fr);
    auto at = [&](float x, float z, int n = 3) {
        FreeRoamFrame f;
        f.dt = 0.1f, f.pos = {x, 0, z};
        f.onScreen = [](float, float, float) { return true; };
        ContactBody body;  // PK09: the player's body for the barrel contacts (a 1.8 x 1 x 3.6 m box)
        body.boxes.push_back({{x, 0.5f, z}, {0.9f, 0.5f, 1.8f}});
        f.bodies = {body};
        for (int i = 0; i < n; ++i) fr.update(f);
    };
    int visits = 0, total = 0;
    for (size_t h = 0; h < R.hubs.size(); ++h) {
        c.enterHub(int(h));
        for (size_t k = 0; k < R.hubs[h].vistaPoints.size(); ++k) {
            ++total;
            at(R.hubs[h].vistaPoints[k].x, R.hubs[h].vistaPoints[k].z);
            visits += c.state().vistas[h][k].b != 0;
        }
    }
    rep_.check("U7", visits == total && total == 95, "vista points visited " + std::to_string(visits) + "/" + std::to_string(total));
    rep_.check("U7", fr.takeSaveRequest(), "a vista visit asks SaveGameData");
    int liv = 0, livN = 0;
    for (size_t h = 0; h < R.hubs.size(); ++h)
        for (size_t k = 0; k < R.hubs[h].liveries.size(); ++k)
            if (R.hubs[h].liveries[k].kind == "POI") {
                ++livN;
                at(R.hubs[h].liveries[k].x, R.hubs[h].liveries[k].z);
                liv += c.state().liveries[h][k].b != 0;
            }
    rep_.check("U5", liv == livN, "POI liveries picked up " + std::to_string(liv) + "/" + std::to_string(livN));
    const uint32_t f0 = c.state().fuel, n0 = c.state().fuelFound;
    fr.setBarrels({{1000, 0, 1000, 100, 1}, {1100, 0, 1000, 400, 1}});
    at(1000, 1000), at(1100, 1000);
    rep_.check("U8", c.state().fuel == std::min<uint64_t>(99999999u, uint64_t(f0) + 500) && c.state().fuelFound == n0 + 5,
               "two barrels: +500 fuel, +5 found (" + std::to_string(c.state().fuel - f0) + ", " + std::to_string(c.state().fuelFound - n0) + ")");
    at(1000, 1000);
    rep_.check("U8", c.state().fuel == std::min<uint64_t>(99999999u, uint64_t(f0) + 500), "a taken barrel gives nothing again");
    fuelEarned_ += 500;
    {
        // PK09: barrels are taken by a contact (0x531310: the other node's +0xd8 & 0x20c), not by the player's distance.
        auto car = [](int flags, float x, float y, float z) {
            ContactBody b;
            b.flags = flags;
            b.boxes.push_back({{x, y + 0.5f, z}, {0.9f, 0.5f, 1.8f}});  // a 1.8 x 1 x 3.6 m body on the ground
            return b;
        };
        auto frame = [&](float x, float z, std::vector<ContactBody> bodies) {
            FreeRoamFrame f;
            f.dt = 0.1f, f.pos = {x, 0, z}, f.bodies = std::move(bodies);
            fr.update(f);
        };
        const uint32_t g0 = c.state().fuel, m0 = c.state().fuelFound;
        fr.setBarrels({{2000, 0, 2000, 100, 0.4f}, {2000, 0, 2100, 100, 0.4f}, {2000, 0, 2200, 400, 0.4f}, {2003, 0, 2202, 400, 0.4f}});
        frame(2000, 2003.5f, {car(0x4, 2000, 0, 2003.5f)});  // 3.5 m away: no contact (the port's old 4 m rule took it)
        const bool near = !fr.barrels()[0].taken;
        frame(2000, 2001.5f, {car(0x4, 2000, 0, 2001.5f)});  // the box's front reaches it
        rep_.check("U8", near && fr.barrels()[0].taken && c.state().fuel == std::min<uint64_t>(99999999u, uint64_t(g0) + 100),
                   "a barrel is taken on contact with the player's body, not within 4 m (0x531310)");
        frame(0, 0, {car(0x8, 2000, 0, 2100)});  // an AI vehicle drives through barrel 1
        rep_.check("U8", fr.barrels()[1].taken && c.state().fuel == std::min<uint64_t>(99999999u, uint64_t(g0) + 100) &&
                             c.state().fuelFound == m0 + 1,
                   "an AI vehicle (+0xd8 0x8) takes a barrel without credit");
        frame(0, 0, {car(0x8, 2000, 0, 2200)});  // ... and a 400 pack with it (0x51d600 runs the pack for anyone)
        frame(2003, 2202, {car(0x4, 2003, 0, 2202)});
        rep_.check("U8", fr.barrels()[2].taken && fr.barrels()[3].taken && c.state().fuelFound == m0 + 1,
                   "an AI-taken 400 barrel takes its pack (within 5 m): the player gets none of it");
        fr.setBarrels({{5000, 0, 5000, 100, 0.4f}});
        frame(5000, 5000, {ContactBody{0x20, {{5000, 0.5f, 5000, 2}}, {}}});
        rep_.check("U8", !fr.barrels()[0].taken, "a node without 0x20c (a dynamic object, 0x20) takes nothing");
        fuelEarned_ += 100;
    }
    {
        // PK09: the POI list in FUEL's index order: the append order (hubinfos.tsc liveries, poi.tsc), then 0x735ac0.
        const auto& P = fr.pois();
        int n2 = 0, n3 = 0, n4 = 0, moved = 0;
        bool sorted = true;
        for (size_t i = 0; i < P.size(); ++i) {
            n2 += P[i].type == 2, n3 += P[i].type == 3, n4 += P[i].type == 4;
            if (i && P[i].type < P[i - 1].type) sorted = false;
        }
        // A stable sort would keep the append order inside a type; 0x735ac0 does not.
        std::vector<int> stable;
        for (int t : {2, 3, 4})
            for (const auto& e : R.poiScript)
                if (e.type == t) stable.push_back(t);
        std::vector<FreeRoam::Poi> st(P.begin(), P.end());
        std::stable_sort(st.begin(), st.end(), [](const FreeRoam::Poi& a, const FreeRoam::Poi& b) { return a.order < b.order; });
        std::stable_sort(st.begin(), st.end(), [](const FreeRoam::Poi& a, const FreeRoam::Poi& b) { return a.type < b.type; });
        for (size_t i = 0; i < P.size(); ++i) moved += P[i].order != st[i].order;
        rep_.check("U11", P.size() == 433 && n2 == 148 && n3 == 95 && n4 == 190 && sorted,
                   "433 POIs (148 liveries, 95 vistas, 190 teleports), types ascending: liveries 0..147, vistas 148..242");
        // The vector: types 4 3 2 4 3 2 (records a..f) -> 0x735ac0 gives c f e b a d (a stable sort: c f b e a d).
        struct T {
            int type;
            char id;
        };
        std::vector<T> v{{4, 'a'}, {3, 'b'}, {2, 'c'}, {4, 'd'}, {3, 'e'}, {2, 'f'}};
        poiExchangeSort(v);
        std::string got;
        for (const T& t : v) got += t.id;
        rep_.check("U11", got == "cfebad", "0x735ac0 vector: 4 3 2 4 3 2 (a..f) -> " + got + " (FUEL: cfebad)");
        rep_.check("U11", moved > 0, std::to_string(moved) + " of 433 records leave the append order within their type (unstable sort)");
        // The marker path (worldPoiList over the script files) indexes the same list.
        auto text = [&](const char* f) {
            std::ifstream in(fs::path(data_) / "LocalizedComponent/GameTsc/Story" / f, std::ios::binary);
            return std::string(std::istreambuf_iterator<char>(in), {});
        };
        const auto W = worldPoiList(text("hubinfos.tsc"), text("poi.tsc"));
        bool same = W.size() == P.size();
        for (size_t i = 0; same && i < W.size(); ++i) same = W[i].type == P[i].type && W[i].x == P[i].x && W[i].z == P[i].z;
        rep_.check("U11", same, "the markers' POI list (worldPoiList) has FreeRoam's index order");
        // Grid walk (0x4edc70 / 0x4edd60): cells row by row, newest first in a cell.
        PoiGrid g;
        g.build({{100, 100}, {200, 200}, {6000, 100}, {300, 300}, {100, 6000}});
        std::string walk;
        g.query(3000, 3000, 4000, [&](int i) { return walk += char('0' + i), true; });
        rep_.check("U11", walk == "31024", "POI grid walk order " + walk + " (FUEL: 3 1 0, 2, 4)");
    }
    int caught = 0, traffic = 0, again = 0;
    for (size_t h = 0; h < R.hubs.size(); ++h)
        for (size_t k = 0; k < R.hubs[h].traffic.size(); ++k) {
            ++traffic;
            const bool was = c.trafficCaught(int(h), int(k));
            const bool now = fr.catchTraffic(int(h), int(k));
            caught += c.trafficCaught(int(h), int(k));
            again += fr.catchTraffic(int(h), int(k));
            if (!was && now) {
                const int kind = R.hubs[h].traffic[k].kind();
                bool rew = true;
                if (kind == 0)
                    for (const auto& v : c.state().vistas[h]) rew = rew && v.a;
                if (kind == 1)
                    for (const auto& v : c.state().liveries[h]) rew = rew && v.a;
                if (kind == 2)
                    for (const auto& v : c.state().challenges[h]) rew = rew && v.available;
                if (kind == 3) rew = c.vehicleOwned(R.hubs[h].traffic[k].mesh) || R.vehicleIndex(R.hubs[h].traffic[k].mesh) < 0;
                rep_.check("U9", rew, R.hubs[h].name + " traffic " + std::to_string(k) + " kind " + std::to_string(kind) + ": its reward");
            }
        }
    rep_.check("U9", caught == traffic && again == 0, "special traffic caught " + std::to_string(caught) + "/" + std::to_string(traffic) + ", none twice");
    rep_.check("U9", fr.takeSaveRequest(), "a catch asks SaveGameData");
    // PK08 MessagesHUD: kind 8 (0x4afc70) shows the bonus car's picture (record +0x4c) and name (+0x6c); the action key during the
    // hold (0.1 T < t < 0.9 T) cuts to the fade-out (0x4af920).
    {
        int bh = -1, bk = -1;
        for (size_t h = 0; h < R.hubs.size() && bh < 0; ++h)
            for (size_t k = 0; k < R.hubs[h].traffic.size(); ++k)
                if (R.hubs[h].traffic[k].kind() == 3 && R.vehicleIndex(R.hubs[h].traffic[k].mesh) >= 0) { bh = int(h), bk = int(k); break; }
        rep_.check("P7", bh >= 0, "a bonus-car traffic entry exists");
        if (bh >= 0) {
            const int vi = R.vehicleIndex(R.hubs[size_t(bh)].traffic[size_t(bk)].mesh);
            fr.messages().clear();
            fr.messages().push(8, vi);
            fr.messages().update(0);
            Page* p = m->messagePage(8, fr.messages().current().param);
            const Item* pic = p ? p->find("ITEM_ID_BONUS_CAR_PICTURE") : nullptr;
            const Item* nm = p ? p->find("ITEM_ID_POI_TT") : nullptr;
            const VehicleInfo* info = m->vehicleInfoPublic(R.vehicles[size_t(vi)].label);
            rep_.check("P7", pic && nm && info && pic->imgNormal == info->picture && !info->picture.empty() && m->assets().image(info->picture) &&
                                 nm->dynText == m->textOf(R.vehicles[size_t(vi)].nameText) && !nm->dynText.empty(),
                       "bonus-car banner: picture " + (info ? info->picture : std::string("?")) + ", name \"" + (nm ? nm->dynText : std::string()) + "\"");
            rep_.check("P7", !m->bannerAccept(), "the action key during the fade-in does not skip");
            fr.messages().update(1.0f);  // t = 4 s of 5: the steady phase
            rep_.check("P7", m->bannerAccept() && fr.messages().alpha() < 1.0f + 1e-6f, "the action key in the hold cuts to the fade-out");
            rep_.check("P7", !m->bannerAccept(), "...and not again (t = 0.1 T)");
            fr.messages().update(1.0f);
            rep_.check("P7", !fr.messages().showing(), "the banner is gone after the fade-out");
            fr.messages().clear();
        }
    }
    // PK08 0x4b16a0: MissionHUD init forces the race-info and ranking display options back on.
    {
        MenuOptions o = loadMenuOptions();
        o.raceInfo = o.rankingInfo = false;
        fs::create_directories(fs::path(settingsPath()).parent_path());
        const bool saved = saveMenuOptions(o);
        auto m2 = boot(MenuMode::Pause);
        const bool off = !m2->options().raceInfo && !m2->options().rankingInfo;
        m2->setSession("", R.hubs[0].name, m2->vehicle());
        rep_.check("P8", saved && off && m2->options().raceInfo && m2->options().rankingInfo, "the HUD init forces the race-info and ranking options on (they were off)");
        m2->sessionEnd();
    }
    c.save();
    m->sessionEnd();
    checkPersisted(c, "free roam");
}

// ---------------------------------------------------------------------------------------------- 6. saves

void Audit::saves() {
    std::printf("-- save and load\n");
    const CareerRules& R = *rules_;
    {  // Every field round-trips.
        Career a(R);
        CareerState& s = a.state();
        std::mt19937 rng(99);
        auto b = [&] { return uint8_t(rng() & 0xff); };
        for (auto& p : s.profile)
            for (int i = 0; i < 32; ++i) p[i] = char('A' + rng() % 26);
        s.fuel = 12345678, s.fuelFound = 4321;
        for (auto& x : s.stats) x = b();
        for (auto& x : s.options) x = b();
        s.options2a0 = 0xdeadbeef;
        for (auto& v : s.vehicles) v.unlocked = 1, v.owned = rng() & 1, v.livery = b() % 9, v.color = b() % 12, v.f10 = float(rng() % 10000) / 7.0f;
        for (auto& h : s.hubs) h.opened = rng() & 1, h.unlocked = 1;
        for (auto* v : {&s.vistas, &s.traffic, &s.liveries})
            for (auto& hub : *v)
                for (auto& e : hub) e.b = rng() & 1, e.a = e.b ? 1 : (rng() & 1);
        for (auto* v : {&s.tutorial, &s.career, &s.challenges})
            for (auto& hub : *v)
                for (auto& m : hub) {
                    uint8_t* p = reinterpret_cast<uint8_t*>(&m);
                    for (size_t i = 0; i < sizeof m; ++i) p[i] = b();
                }
        for (auto& o : s.pilotOwned) o = rng() & 1;
        for (auto& pr : s.pilotPairs) pr.second = rng() % 30;
        for (auto& x : s.editedRaces) x = b();
        s.trophiesWon = {1, 5, 19, 44};
        for (auto& t : s.tutorials) t.seen = rng() & 1, t.off = rng() & 1;
        s.trips = {{0, 1, true}, {1, 3, false}, {3, 3, true}};
        const auto bytes = a.serialize();
        Career back(R);
        std::string err;
        const bool ok = back.deserialize(bytes, &err);
        const CareerState& t = back.state();
        auto eq = [&](const char* what, bool v) { rep_.check("S1", v, std::string("round trip: ") + what); };
        eq("deserialize", ok && err.empty());
        eq("profile strings", std::memcmp(s.profile, t.profile, sizeof s.profile) == 0);
        eq("fuel, barrels", s.fuel == t.fuel && s.fuelFound == t.fuelFound);
        eq("stats block 0xb8", std::memcmp(s.stats, t.stats, sizeof s.stats) == 0);
        eq("options 0x38 + 0x2a0", std::memcmp(s.options, t.options, sizeof s.options) == 0 && s.options2a0 == t.options2a0);
        bool v = s.vehicles.size() == t.vehicles.size();
        for (size_t i = 0; v && i < s.vehicles.size(); ++i)
            v = s.vehicles[i].crc == t.vehicles[i].crc && s.vehicles[i].owned == t.vehicles[i].owned && s.vehicles[i].unlocked == t.vehicles[i].unlocked &&
                s.vehicles[i].livery == t.vehicles[i].livery && s.vehicles[i].color == t.vehicles[i].color && s.vehicles[i].f10 == t.vehicles[i].f10;
        eq("vehicles (unlocked, owned, livery, colour, time)", v);
        bool hb = true;
        for (size_t i = 0; i < s.hubs.size(); ++i) hb = hb && s.hubs[i].opened == t.hubs[i].opened && s.hubs[i].unlocked == t.hubs[i].unlocked;
        eq("hubs (opened, unlocked)", hb);
        auto flags = [](const std::vector<std::vector<CareerState::Flags2>>& x, const std::vector<std::vector<CareerState::Flags2>>& y) {
            if (x.size() != y.size()) return false;
            for (size_t h = 0; h < x.size(); ++h) {
                if (x[h].size() != y[h].size()) return false;
                for (size_t k = 0; k < x[h].size(); ++k)
                    if (x[h][k].a != y[h][k].a || x[h][k].b != y[h][k].b) return false;
            }
            return true;
        };
        eq("vistas", flags(s.vistas, t.vistas));
        eq("traffic", flags(s.traffic, t.traffic));
        eq("vehicle liveries", flags(s.liveries, t.liveries));
        bool ms = true;
        for (auto [x, y] : {std::pair{&s.tutorial, &t.tutorial}, std::pair{&s.career, &t.career}, std::pair{&s.challenges, &t.challenges}})
            for (size_t h = 0; h < x->size(); ++h)
                for (size_t k = 0; k < (*x)[h].size(); ++k) ms = ms && std::memcmp(&(*x)[h][k], &(*y)[h][k], sizeof(MissionStatus)) == 0;
        eq("mission status blocks (40 B each)", ms);
        eq("pilot liveries + pairs", s.pilotOwned == t.pilotOwned && s.pilotPairs == t.pilotPairs);
        eq("edited races", s.editedRaces == t.editedRaces);
        eq("achievements", s.trophiesWon == t.trophiesWon);
        bool tu = true;
        for (size_t i = 0; i < s.tutorials.size(); ++i) tu = tu && s.tutorials[i].seen == t.tutorials[i].seen && s.tutorials[i].off == t.tutorials[i].off;
        eq("tutorials", tu);
        bool tr = s.trips.size() == t.trips.size();
        for (size_t i = 0; tr && i < s.trips.size(); ++i) tr = s.trips[i].from == t.trips[i].from && s.trips[i].to == t.trips[i].to && s.trips[i].valid == t.trips[i].valid;
        eq("travel history", tr);
        eq("the same bytes again", back.serialize() == bytes);
    }
    // An old save: written before P42 (57 career races: the hub lists had no world missions), before P62/P66 (empty
    // TUTORIAL, empty pilot pair list) and before P44 (no PORT_TRAVEL).
    {
        Career cur(R);
        const auto& yl = R.hubs[0].career;
        std::vector<size_t> emd;
        for (size_t k = 0; k < yl.size(); ++k)
            if (!R.missions[size_t(yl[k])].world) emd.push_back(k);
        rep_.check("S4", !emd.empty() && emd.size() < yl.size(), "YellowStone has EMD and world races (" + std::to_string(emd.size()) + "/" + std::to_string(yl.size()) + ")");
        struct W {
            std::vector<uint8_t> b;
            size_t at = 0;
            void u32(uint32_t v) { b.insert(b.end(), reinterpret_cast<uint8_t*>(&v), reinterpret_cast<uint8_t*>(&v) + 4); }
            void u8(uint8_t v) { b.push_back(v); }
            void raw(const void* p, size_t n) { b.insert(b.end(), static_cast<const uint8_t*>(p), static_cast<const uint8_t*>(p) + n); }
            void begin(const std::string& n) {
                const std::string s = "SECTION_" + n;
                raw(s.c_str(), s.size() + 1);
                b.resize((b.size() | 3) + 1, 0);  // 0x565b70: pos = (pos | 3) + 1
                at = b.size();
                u32(0);
            }
            void end() {
                const uint32_t n = uint32_t(b.size() - at - 4);
                std::memcpy(&b[at], &n, 4);
            }
        } p;
        const CareerState& s = cur.state();
        p.begin("SAVESTRUCT_INFOS");
        p.raw(s.profile[0], 33), p.raw(s.profile[1], 33);
        p.u32(777), p.u32(3);
        p.raw(s.stats, sizeof s.stats), p.raw(s.options, sizeof s.options), p.u32(0);
        p.end();
        p.begin("VEHICLES_INFOS");
        p.u32(uint32_t(s.vehicles.size()));
        for (const auto& v : s.vehicles) p.u32(v.crc), p.u8(v.unlocked), p.u8(v.owned), p.u8(0), p.u8(0), p.u32(0);
        p.end();
        p.begin("INPUT"), p.end();
        p.begin("TUTORIAL"), p.end();
        p.begin("HUB_INFOS");
        p.u32(uint32_t(s.hubs.size()));
        for (size_t h = 0; h < s.hubs.size(); ++h) p.u8(h <= 1), p.u8(h <= 1);
        p.end();
        p.begin("VISTAPOINT_INFOS");
        p.u32(uint32_t(s.vistas.size()));
        for (const auto& h : s.vistas) {
            p.u32(uint32_t(h.size()));
            for (size_t k = 0; k < h.size(); ++k) p.u8(0), p.u8(0);
        }
        p.end();
        p.begin("EDITEDRACE_INFOS"), p.raw(s.editedRaces.data(), s.editedRaces.size()), p.end();
        p.begin("MISSION_INFOS");
        p.u32(uint32_t(s.career.size()));
        int oldCareer = 0;
        for (size_t h = 0; h < s.career.size(); ++h) {
            p.u32(uint32_t(h));
            p.u32(0);
            std::vector<int> ids;
            for (int mi : R.hubs[h].career)
                if (!R.missions[size_t(mi)].world) ids.push_back(mi);
            oldCareer += int(ids.size());
            p.u32(uint32_t(ids.size()));
            for (size_t k = 0; k < ids.size(); ++k) {
                MissionStatus m;
                if (h == 0 && k == 0) m.bestRank = 0, m.lastRank = 0, m.medal[0] = m.medal[1] = m.medal[2] = 1, m.flag10c = 1, m.available = 1;
                p.raw(&m, sizeof m);
            }
            p.u32(uint32_t(s.challenges[h].size()));
            for (size_t k = 0; k < s.challenges[h].size(); ++k) {
                MissionStatus m;
                if (h == 0 && k == 0) m.bestRank = 0, m.medal[0] = 1, m.available = 1;
                p.raw(&m, sizeof m);
            }
        }
        p.end();
        p.begin("TRAFFIC_INFOS"), p.u32(0), p.end();
        p.begin("VEHICLES_LIVERIES_INFOS"), p.u32(0), p.end();
        p.begin("PILOT_LIVERIES_INFOS");
        p.u32(uint32_t(s.pilotOwned.size()));
        for (uint8_t o : s.pilotOwned) p.u8(o);
        p.u32(0);
        p.end();
        p.begin("PORT_TROPHIES"), p.u32(1), p.u32(2), p.end();
        std::vector<uint8_t> file = {'F', 'U', 'E', 'L', 'P', 'S', 'A', 'V', 1, 0, 0, 0};
        const uint32_t n = uint32_t(p.b.size());
        file.insert(file.end(), reinterpret_cast<const uint8_t*>(&n), reinterpret_cast<const uint8_t*>(&n) + 4);
        file.insert(file.end(), p.b.begin(), p.b.end());
        rep_.check("S4", oldCareer == 57, "the old layout lists 57 career races (" + std::to_string(oldCareer) + ")");
        Career old(R);
        std::string err;
        const bool ok = old.deserialize(file, &err);
        rep_.check("S4", ok && err.empty(), "an old (57-race) save loads" + (err.empty() ? std::string() : ": " + err));
        const std::string gold = R.missions[size_t(yl[emd[0]])].name;
        rep_.check("S4", old.status(gold) && old.status(gold)->bestRank == 0 && old.stars() == 3,
                   "its YellowStone gold stays on " + gold + " (stars " + std::to_string(old.stars()) + ")");
        rep_.check("S4", old.state().fuel == 777, "the old save's fuel (" + std::to_string(old.state().fuel) + ")");
        rep_.check("S4", old.hubUnlocked(1), "the old save's hubs");
        rep_.check("S4", old.state().trophiesWon == std::vector<int>{2}, "the old save's achievements (" + std::to_string(old.state().trophiesWon.size()) + ")");
        rep_.check("S4", old.state().pilotPairs.size() == s.pilotPairs.size() && old.pilotTypeLivery("HEAD1") == cur.pilotTypeLivery("HEAD1"),
                   "an empty pilot pair list keeps the defaults");
        const auto keep = readFile(save_);
        writeFile(save_, file);
        auto m = boot(MenuMode::Main);
        rep_.check("S4", m->career()->state().fuel == 777 && m->career()->stars() == 3, "the menus load the old save");
        m.reset();
        writeFile(save_, keep);
    }
    {
        Career a(R);
        a.state().fuel = 5;
        const std::vector<uint8_t> junk = {'n', 'o', 'p', 'e'};
        rep_.check("S4", !a.deserialize(junk) && a.state().fuel == 5, "a foreign file is refused, the career kept");
        // A session over a file that does not load leaves it alone at its end.
        const auto keep = readFile(save_);
        writeFile(save_, junk);
        {
            auto g = boot(MenuMode::Pause);
            g->setSession("", R.hubs[0].name, g->vehicle());
            g->sessionEnd();
        }
        rep_.check("S4", readFile(save_) == junk, "a session does not overwrite an unreadable career.sav when it ends");
        writeFile(save_, keep);
    }
    // The DATA page (Map > DATA = HubData): SAVE asks TT_AUTO_SAVE_OVERWRITE, writes, TT_SAVE_COMPLETED_SUCCESSFULLY;
    // LOAD asks TT_LOAD_GAME_SHOW_WARNING, reads, TT_LOAD_GAME_SUCCESSFULL and restarts free ride in the saved hub.
    {
        auto m = boot(MenuMode::Main);
        okTutorials(*m);
        Career& c = *m->career();
        const uint32_t fuel = c.state().fuel;
        rep_.check("S3", focusLeft(*m, "ITEM_ID_LEFT_MENU2"), "Map > DATA");
        m->key(MenuKey::Accept);
        rep_.check("S3", focusLeft(*m, "ITEM_ID_HUB_DATA_SAVE"), "DATA > SAVE");
        c.state().fuel = fuel + 1;
        m->key(MenuKey::Accept);
        rep_.check("S3", m->confirmOpen() && m->confirmText() == "TT_AUTO_SAVE_OVERWRITE", "SAVE asks TT_AUTO_SAVE_OVERWRITE");
        confirmYes(*m);
        rep_.check("S3", m->confirmOpen() && m->confirmText() == "TT_SAVE_COMPLETED_SUCCESSFULLY", "then TT_SAVE_COMPLETED_SUCCESSFULLY");
        m->key(MenuKey::Accept);
        Career back(R);
        rep_.check("S3", back.load(save_) && back.state().fuel == fuel + 1, "SAVE wrote career.sav");
        c.state().fuel = 3;  // in memory only
        focusLeft(*m, "ITEM_ID_LEFT_MENU2");
        m->key(MenuKey::Accept);
        rep_.check("S3", focusLeft(*m, "ITEM_ID_HUB_DATA_LOAD"), "DATA > LOAD");
        m->key(MenuKey::Accept);
        rep_.check("S3", m->confirmOpen() && m->confirmText() == "TT_LOAD_GAME_SHOW_WARNING", "LOAD asks TT_LOAD_GAME_SHOW_WARNING");
        confirmYes(*m);
        rep_.check("S3", m->confirmOpen() && m->confirmText() == "TT_LOAD_GAME_SUCCESSFULL", "then TT_LOAD_GAME_SUCCESSFULL");
        m->key(MenuKey::Accept);
        const MenuRequest r = m->take();
        rep_.check("S3", c.state().fuel == fuel + 1, "LOAD restored the saved fuel");
        const int hub = c.currentHub();
        rep_.check("S3", r.kind == MenuRequest::FreeRide && hub >= 0 && findHub(m->hubs(), r.hub) == findHub(m->hubs(), R.hubs[size_t(hub)].name),
                   "LOAD restarts free ride in the saved hub (" + r.hub + ")");
        c.state().fuel = fuel;
        c.save();
    }
    {
        const auto keep = readFile(save_);
        auto m = boot(MenuMode::Main);
        okTutorials(*m);
        fs::remove(save_);
        focusLeft(*m, "ITEM_ID_LEFT_MENU2");
        m->key(MenuKey::Accept);
        focusLeft(*m, "ITEM_ID_HUB_DATA_LOAD");
        m->key(MenuKey::Accept);
        confirmYes(*m);
        rep_.check("S3", m->confirmOpen() && m->confirmText() == "TT_LOAD_GAME_NO_FUEL_DATA_WAS_FOUND", "LOAD without a save: TT_LOAD_GAME_NO_FUEL_DATA_WAS_FOUND");
        m.reset();
        writeFile(save_, keep);
    }
}

// ---------------------------------------------------------------------------------------------- 7. profile

void Audit::profile() {
    std::printf("-- profile\n");
    auto m = boot(MenuMode::Main);
    okTutorials(*m);
    const Career& c = *m->career();
    const CareerRules& R = c.rules();
    const CareerState& s = c.state();
    for (int g = 0; g < 6 && m->screenName() != "Tab:3"; ++g) m->key(MenuKey::TabRight);
    rep_.check("C1", m->screenName() == "Tab:3", "Profile tab (" + m->screenName() + ")");
    rep_.check("C1", focusLeft(*m, "ITEM_ID_TOPMENU_PROFILE_GENERAL"), "Profile > GENERAL");
    m->key(MenuKey::Accept);
    rep_.check("C1", m->screenName() == "Profile", "the General page opens");
    int hubsOpen = 0, careerN = 0, played = 0, gold = 0, chN = 0, chWon = 0, vistaN = 0, visited = 0;
    for (size_t h = 0; h < R.hubs.size(); ++h) {
        hubsOpen += c.hubUnlocked(int(h));
        for (const auto& st : s.career[h]) ++careerN, played += st.flag10c != 0, gold += st.bestRank == 0;
        for (const auto& st : s.challenges[h]) ++chN, chWon += st.bestRank != 3;
        for (const auto& v : s.vistas[h]) ++vistaN, visited += v.b != 0;
    }
    auto two = [](int a, int b) {
        char buf[32];
        std::snprintf(buf, sizeof buf, "%02d/%02d", a, b);
        return std::string(buf);
    };
    const char* P = "PageProfileGeneral_Progression";
    auto has = [&](const char* id, const std::string& want) {
        const std::string v = m->profileValue(P, id);
        rep_.check("C1", v.find(want) != std::string::npos, std::string(id) + " = '" + v + "' (progress " + want + ")");
    };
    has("ITEM_ID_EVENTS_UNLOCKED", two(hubsOpen, int(R.hubs.size())));
    has("ITEM_ID_CAREER_RACE_PLAYED", two(played, careerN));
    has("ITEM_ID_CRACE_WITH_GOLD", two(gold, careerN));
    has("ITEM_ID_CAREER_STARS", two(c.stars(), c.starsMax()));
    has("ITEM_ID_CHALLENGE_CLEARED", two(chWon, chN));
    has("ITEM_ID_VPOINT_VISITED", two(visited, vistaN));
    rep_.check("C1", careerN == 72 && chN == 190 && c.starsMax() == 216, "72 career races, 190 challenges, 216 stars");
    // Completion (0x498700 / 0x498c50): per hub, enabled = unlocked; per race, its best rank, enabled = won.
    for (int sel : {0, 1}) {
        const auto rows = m->profileRows(0x24, sel);
        size_t i = 0;
        bool ok = true;
        int lines = 0;
        for (size_t h = 0; h < R.hubs.size() && ok; ++h) {
            ok = i < rows.size() && rows[i].hub && rows[i].enabled == c.hubUnlockTest(int(h));
            ++i;
            const auto& st = sel == 0 ? s.career[h] : s.challenges[h];
            for (size_t k = 0; k < st.size() && ok; ++k, ++i, ++lines)
                ok = i < rows.size() && !rows[i].hub && rows[i].rank == st[k].bestRank && rows[i].enabled == (st[k].bestRank != 3);
        }
        rep_.check("C2", ok && i == rows.size(), std::string("Completion > ") + (sel ? "CHALLENGES" : "CAREER") + ": " + std::to_string(lines) + " race lines match the progress");
    }
    {
        const auto rows = m->profileRows(0x1e, 1);
        int en = 0;
        for (const auto& r : rows) en += !r.hub && r.enabled;
        rep_.check("C2", en == visited, "Collection > VISTAPOINT: " + std::to_string(en) + " visited lines (" + std::to_string(visited) + ")");
    }
    rep_.note("C1", "FUEL shows counts on the Progression page, no percentage");
}

// ---------------------------------------------------------------------------------------------- PK21: the race editor

// The Editor tab driven by keys (notes/pk21.md): CREATE on the first free slot, points dropped on roads of an unlocked hub
// (refusals: a second point within 30 m of the start, off road), pick / move / delete, Back -> validation (0x475d80): an empty
// CREATE leaves nothing; VALID + Accept = 0x4664d0 (new-race defaults, races created + 1, SaveGameData) -> the CREATE
// achievement; EDIT + "quit without saving" restores the slot (0x475870); a new process lists it and TRY RACE starts
// "Edited Race 01", which plays through RaceRules and ends in states 0xa / 0xb -> 0x1c with no award; DELETE + Yes empties it.
void Audit::editorRaces() {
    std::printf("-- race editor\n");
    auto toEditor = [&](Menu& m) {
        for (int g = 0; g < 8 && m.screenName() != "Tab:5"; ++g) m.key(MenuKey::TabRight), okTutorials(m);
        return m.screenName() == "Tab:5" && m.editorState() == 5;
    };
    auto saved = [&]() {
        Career c(*rules_);
        c.load(save_);
        return c;
    };
    auto m = boot(MenuMode::Main);
    okTutorials(*m);
    rep_.check("ED1", toEditor(*m), "the Editor tab opens on its top menu (state 5, 0x475e70)");
    const std::vector<uint8_t> sav0 = readFile(save_);
    const int created0 = saved().state().racesCreated();
    rep_.check("ED1", fuel::editor::freeSlots(m->career()->state().editedRaces) == 10, "no edited race yet: 10 free slots (0x569060)");
    rep_.check("ED1", m->leftMenuIds() == std::vector<std::string>{"ITEM_ID_TOPMENU_EDITOR_CREATE_RACE"},
               "only CREATE enabled while every slot is free (0x4770b0)");
    m->key(MenuKey::Accept);
    rep_.check("ED1", m->editorState() == 0xa5 && m->editorSlot() == 0, "CREATE -> map editing (0xa5) on the first free slot (0x569190)");
    rep_.check("ED1", m->editorHelpKey().rfind("TT_EDITOR_NO_CHECKPOINT_FROM_CREATEMODE", 0) == 0, "help: " + m->editorHelpKey() + " (0x475940)");
    m->key(MenuKey::Back);
    rep_.check("ED1", m->editorState() == 5 && readFile(save_) == sav0, "Back with no point: back to the top menu, nothing saved (0x475d80)");
    m->key(MenuKey::Accept);
    fuel::editor::Session& s = m->editorSession();
    const auto spots = m->editorRoadSpots(6, 500.0f);
    rep_.check("ED1", spots.size() >= 5, std::to_string(spots.size()) + " road spots near the hub camp can take a point (road, unlocked hub, >= 30 m from a junction)");
    if (spots.size() < 5) return;
    auto at = [&](const std::array<float, 2>& p, float dx = 0) {
        s.cx = p[0] + dx, s.cz = p[1];
        if (s.world.ground) s.cy = s.world.ground(s.cx, s.cz);
    };
    at(spots[0]), m->key(MenuKey::Accept);
    rep_.check("ED1", m->editorRace().count == 1, "Accept on a road: the start point (0x464b50)");
    at(spots[0], 8.0f), m->key(MenuKey::Accept);
    rep_.check("ED1", m->editorRace().count == 1, "a second point within 30 m of the start is refused (0x464b50)");
    {  // off road: no road within the pick radius
        bool off = false;
        for (int k = 1; k < 40 && !off; ++k) {
            at(spots[0], 37.0f * float(k));
            uint8_t gate = 0;
            s.snapped(&gate);
            off = gate == 1;
        }
        const int n0 = m->editorRace().count;
        if (off) m->key(MenuKey::Accept);
        rep_.check("ED1", off && m->editorRace().count == n0 && m->editorHelpKey() == "TT_EDITOR_MIDDLE_HELP2", "off road: no point, help MIDDLE_HELP2 (0x464ac0)");
    }
    at(spots[1]), m->key(MenuKey::Accept);
    at(spots[2]), m->key(MenuKey::Accept);
    rep_.check("ED1", m->editorRace().count == 3 && m->editorHelpKey() == "TT_EDITOR_MIDDLE_HELP", "three points, help MIDDLE_HELP");
    m->key(MenuKey::Vali2);
    rep_.check("ED1", s.picked == 2 && m->editorHelpKey() == "TT_EDITOR_DROP_POINT", "MENU_VALI2 picks the point under the cursor (help DROP_POINT)");
    at(spots[3]), m->key(MenuKey::Vali2);
    const auto& p2 = m->editorRace().pts[2];
    rep_.check("ED1", s.picked < 0 && std::hypot(p2.x - spots[3][0], p2.z - spots[3][1]) < 1.0f, "MENU_VALI2 again drops it on the road at the cursor (0x465660)");
    m->key(MenuKey::TabRight);
    rep_.check("ED1", m->editorRace().count == 2, "MENU_CANCEL deletes the point under the cursor (0x4646b0)");
    m->key(MenuKey::Accept);
    rep_.check("ED1", m->editorRace().count == 3, "and Accept puts it back");
    {
        const auto info = m->editorInfos();
        const float len = fuel::editor::routeLength(m->editorRace(), s.world);
        const int km = fuel::editor::lengthKm(len);
        const bool miles = info.second.find("mi") != std::string::npos;
        const std::string want = miles ? std::to_string(int(float(km) * 0.6213712f)) + " mi" : std::to_string(km) + " km";
        rep_.check("ED1", info.first == "03/32" && info.second == want && len > 0,
                   "EditorMapInfos: " + info.first + ", " + info.second + " (route " + std::to_string(int(len)) + " m, 0x4772c0)");
    }
    const int saves0 = m->saveCount();
    m->key(MenuKey::Back);
    rep_.check("ED1", m->editorState() == 0xa8, "Back: the race is valid (0xa8)");
    m->key(MenuKey::Accept);
    {
        Career c = saved();
        const fuel::EditedRace r = fuel::editor::slot(c.state().editedRaces, 0);
        int createId = -1;
        for (const auto& t : c.rules().trophies)
            if (t.verb == 8 && t.object == 9) createId = t.id;
        const bool trophy = std::find(c.state().trophiesWon.begin(), c.state().trophiesWon.end(), createId) != c.state().trophiesWon.end();
        rep_.check("ED1", m->editorState() == 5 && m->saveCount() == saves0 + 1 && r.count == 3 && r.type == 1 && r.optionFlags() == 0x816 &&
                              c.state().racesCreated() == created0 + 1,
                   "Accept: saved (0x4664d0: type 1, options 0x816, races created " + std::to_string(c.state().racesCreated()) + ")");
        rep_.check("ED1", trophy, "achievement " + std::to_string(createId) + " (CREATE Race 1) awarded and saved");
        rep_.check("ED1", c.state().editedRaces == m->career()->state().editedRaces, "EDITEDRACE_INFOS in career.sav = the session's block");
    }
    // EDIT, a fourth point, then "quit without saving" (MENU_CANCEL on the valid popup) -> the slot as it was.
    {
        const std::vector<uint8_t> before = readFile(save_);
        rep_.check("ED1", focusLeft(*m, "ITEM_ID_TOPMENU_EDITOR_EDIT_RACE"), "EDIT enabled once a slot is used");
        m->key(MenuKey::Accept);
        rep_.check("ED1", m->editorState() == 0xa2 && m->editorListRows() == std::vector<std::string>{m->textOf(182).empty() ? "" : "EDITED RACE 01"},
                   "EDIT lists \"EDITED RACE 01\" (0x477480)");
        m->key(MenuKey::Accept);
        at(spots[4]), m->key(MenuKey::Accept);
        const int n = m->editorRace().count;
        m->key(MenuKey::Back);
        m->key(MenuKey::TabRight);
        rep_.check("ED1", n == 4 && m->editorState() == 0xa2 && fuel::editor::slot(m->career()->state().editedRaces, 0).count == 3 && readFile(save_) == before,
                   "quit without saving: the slot is restored (0x475870), back to the list, nothing saved");
        m->key(MenuKey::Back);
    }
    m->sessionEnd();
    // A new process: TRY RACE starts it.
    auto t = boot(MenuMode::Main);
    okTutorials(*t);
    toEditor(*t);
    MenuRequest req;
    if (focusLeft(*t, "ITEM_ID_TOPMENU_EDITOR_TRY_RACE")) {
        t->key(MenuKey::Accept);
        t->key(MenuKey::Accept);
        req = t->take();
    }
    rep_.check("ED1", req.kind == MenuRequest::StartEvent && req.event == "Edited Race 01" && !req.vehicle.empty(),
               "reloaded: TRY RACE -> StartEvent \"Edited Race 01\" on " + req.vehicle);
    {
        const auto events = fuel::editor::editedEvents(saved().state().editedRaces, t->editorWorld());
        const Event* ev = events.empty() ? nullptr : &events.front();
        rep_.check("ED1", ev && ev->id == "Edited Race 01" && ev->race.scenario == 2 && ev->race.checkpoints.size() == 1 && ev->race.laps == 0 &&
                              ev->race.editorFlags == 0x816 && ev->race.gameMode() == Race::GmCheckpoint,
                   "the session builds it (0x465be0): checkpoint race, 1 checkpoint, no laps, mission +0x54 = 0x816");
        if (ev) {
            ++races_;
            const RaceRun run = runRace(*ev, Plan::First);
            auto g = boot(MenuMode::Pause);
            g->setSession(ev->id, "", req.vehicle);
            const std::vector<uint8_t> cs = g->career()->serialize();
            const int sv = g->saveCount();
            g->setResults(run.rows, run.won);
            g->setMode(MenuMode::Results);
            const bool endPage = g->screenName() == "EndRace";
            g->key(MenuKey::Accept);
            rep_.check("ED1", run.ended && run.won && run.playerFinished && endPage && g->screenName() == "WhatNext",
                       "played through RaceRules: " + std::string(run.won ? "won" : "lost") + " in " + std::to_string(int(run.time)) + " s; EditedRaceYouWin -> WhatNext (0xa -> 0x1c)");
            rep_.check("ED1", g->career()->serialize() == cs && g->saveCount() == sv && !g->lastAward().counted, "an edited race gives no award and does not save");
        }
    }
    t->sessionEnd();
    // DELETE: yes / no popup, Yes empties the slot, OK -> the top menu (no race left).
    auto d = boot(MenuMode::Main);
    okTutorials(*d);
    toEditor(*d);
    bool popup = false;
    if (focusLeft(*d, "ITEM_ID_TOPMENU_EDITOR_DELETE_RACE")) {
        d->key(MenuKey::Accept);
        d->key(MenuKey::Accept);
        popup = d->confirmOpen() && d->confirmText() == "TT_PAGE_EDITOR_DELETE_THIS_RACE_YESNO";
        confirmYes(*d);
    }
    const bool deleted = d->editorState() == 0xa7 && fuel::editor::slot(saved().state().editedRaces, 0).count == 0;
    d->key(MenuKey::Accept);
    rep_.check("ED1", popup && deleted && d->editorState() == 5 && d->leftMenuIds() == std::vector<std::string>{"ITEM_ID_TOPMENU_EDITOR_CREATE_RACE"},
               "DELETE: yes / no (0xa6), Yes -> slot emptied and saved (0x475b50), OK -> top menu with only CREATE (0x4767f0)");
    // CREATE again in the freed slot: two points -> type 0 (0x4664d0), races created counts every save; the career keeps it.
    d->key(MenuKey::Accept);
    {
        fuel::editor::Session& s2 = d->editorSession();
        for (int k = 0; k < 2; ++k) {
            s2.cx = spots[size_t(k)][0], s2.cz = spots[size_t(k)][1];
            d->key(MenuKey::Accept);
        }
        d->key(MenuKey::Back);
        d->key(MenuKey::Accept);
        Career c = saved();
        const fuel::EditedRace r = fuel::editor::slot(c.state().editedRaces, 0);
        rep_.check("ED1", d->editorState() == 5 && r.count == 2 && r.type == 0 && c.state().racesCreated() == created0 + 2 &&
                              fuel::editor::freeSlots(c.state().editedRaces) == 9,
                   "CREATE again in slot 0: 2 points -> type 0, races created " + std::to_string(c.state().racesCreated()));
    }
    d->sessionEnd();
}

// ---------------------------------------------------------------------------------------------- 8. achievements

void Audit::achievements() {
    std::printf("-- achievements\n");
    Career c(*rules_);
    c.load(save_);
    const auto& T = c.rules().trophies;
    std::set<int> won(c.state().trophiesWon.begin(), c.state().trophiesWon.end());
    rep_.check("A1", won.size() == c.state().trophiesWon.size(), "no achievement awarded twice");
    std::string missed;
    for (const auto& t : T) {
        std::string prog;
        const bool m = c.trophyMet(t, &prog);
        if (m && !won.count(t.id))  // GOTO HUB (the travel history) can stop holding after it was awarded: awards stay
            rep_.check("A1", false, "achievement " + std::to_string(t.id) + " (" + kTrophyVerbs[t.verb] + " " + kTrophyObjects[t.object] + "): met " + std::to_string(m) +
                                        ", awarded " + std::to_string(won.count(t.id)));
        else rep_.check("A1", true, "");
        if (!m) missed += " " + std::to_string(t.id) + ":" + kTrophyVerbs[t.verb] + "_" + kTrophyObjects[t.object];
    }
    rep_.note("A1", std::to_string(won.size()) + "/" + std::to_string(T.size()) + " earned by the audit's career; not met:" + missed);
}

// ---------------------------------------------------------------------------------------------- cross-checks

void Audit::crossCheck() {
    std::printf("-- hubinfos.tsc cross-check\n");
    const CareerRules& R = *rules_;
    const Career probe(R);  // FUEL's hub lists (locate)
    // The script itself, read here without the port's parsers.
    std::ifstream f(fs::path(data_) / "LocalizedComponent/GameTsc/Story/hubinfos.tsc");
    std::vector<std::pair<std::string, std::string>> career, challenge;
    for (std::string line; std::getline(f, line);) {
        std::istringstream ss(line);
        std::string cmd;
        ss >> cmd;
        if (cmd != "AddHubCareerRaces" && cmd != "AddHubCHallengeRaces") continue;
        std::vector<std::string> q;
        for (size_t p = line.find('"'); p != std::string::npos;) {
            const size_t e = line.find('"', p + 1);
            if (e == std::string::npos) break;
            q.push_back(line.substr(p + 1, e - p - 1));
            p = line.find('"', e + 1);
        }
        if (q.size() < 2) continue;
        (cmd == "AddHubCareerRaces" ? career : challenge).push_back({q[0], q[1]});
    }
    rep_.check("X1", career.size() == 72 && challenge.size() == 190,
               "hubinfos.tsc lists " + std::to_string(career.size()) + " career races and " + std::to_string(challenge.size()) + " challenges");
    int listed = 0, menu = 0, won = 0;
    for (auto* v : {&career, &challenge})
        for (const auto& [hub, name] : *v) {
            int h, l, k;
            const bool in = probe.locate(name, &h, &l, &k) && same(R.hubs[size_t(h)].name, hub);
            listed += in;
            auto has = [&](const std::set<std::string>& s) {
                for (const auto& x : s)
                    if (same(x, name)) return true;
                return false;
            };
            menu += has(startedFromMenu_);
            won += has(completed_);
            rep_.check("X1", in, name + " (" + hub + ") is in FUEL's hub lists");
            rep_.check("X1", has(startedFromMenu_), name + " (" + hub + ") started from the menu");
            rep_.check("X1", has(completed_), name + " (" + hub + ") completed (won)");
        }
    std::printf("   %d listed, %d started from the menu, %d won of %zu\n", listed, menu, won, career.size() + challenge.size());
    // Hub zones: every listed event starts in its own hub's zone (the arch prompt and WhatNext go by the zone).
    if (!zones_.zones.empty()) {
        int inOwn = 0, n = 0;
        std::string bad;
        for (const Event& e : events_) {
            int h = -1;
            if (!probe.locate(e.id, &h, nullptr, nullptr)) continue;
            ++n;
            const int z = zones_.find(e.race.start[0], e.race.start[2]);
            const int zh = z >= 0 ? R.hubIndex(zones_.zones[size_t(z)].name) : -1;
            inOwn += zh == h;
            if (zh != h && bad.size() < 240) bad += " " + e.id + "(" + (z >= 0 ? zones_.zones[size_t(z)].name : std::string("no zone")) + ")";
        }
        rep_.check("X1", inOwn == n, std::to_string(inOwn) + "/" + std::to_string(n) + " event starts in their own hub's zone;" + bad);
        int named = 0;
        for (const auto& z : zones_.zones) named += R.hubIndex(z.name) >= 0;
        rep_.check("X1", named == int(zones_.zones.size()), std::to_string(named) + "/" + std::to_string(zones_.zones.size()) + " hub zones name a hub");
    }
}

// Everything the scripts promise, granted by the end.
void Audit::finalState() {
    std::printf("-- final state\n");
    Career c(*rules_);
    rep_.check("X2", c.load(save_), "the final career.sav loads");
    const CareerRules& R = c.rules();
    const CareerState& s = c.state();
    rep_.check("X2", c.stars() == 216, "216 stars (" + std::to_string(c.stars()) + ")");
    rep_.check("X2", c.challengesWon() == 190, "190 challenges won (" + std::to_string(c.challengesWon()) + ")");
    for (size_t h = 0; h < R.hubs.size(); ++h) rep_.check("X2", c.hubUnlocked(int(h)), R.hubs[h].name + " unlocked");
    for (const auto& u : R.unlocks)
        if (u.kind == "VEHICLE") rep_.check("X2", c.vehicleUnlocked(u.target), "UnlockNeed " + u.target + " (" + u.cond + " " + u.arg + ") unlocked");
    for (const auto& v : R.vehicles)
        if (!v.raceUnlock.empty() && up(v.raceUnlock) != "NONE" && c.locate(v.raceUnlock, nullptr, nullptr, nullptr))
            rep_.check("X2", c.vehicleOwned(v.label), v.label + " given by " + v.raceUnlock);
    for (size_t p = 0; p < R.pilotLiveries.size(); ++p) {
        const auto& pl = R.pilotLiveries[p];
        if (up(pl.unlock) == "ALWAYS") rep_.check("X2", s.pilotOwned[p] != 0, "pilot livery " + pl.part + " " + std::to_string(pl.index) + " ALWAYS owned");
        else if (up(pl.unlock) != "NONE" && c.locate(pl.unlock, nullptr, nullptr, nullptr))
            rep_.check("X2", s.pilotOwned[p] != 0, "pilot livery " + pl.part + " " + std::to_string(pl.index) + " from " + pl.unlock);
    }
    uint64_t pay = 0;
    bool medals = true;
    for (size_t h = 0; h < R.hubs.size(); ++h) {
        for (size_t k = 0; k < R.hubs[h].career.size(); ++k) {
            const auto& m = s.career[h][k];
            medals = medals && m.medal[0] && m.medal[1] && m.medal[2] && m.bestRank == 0;
            for (int i = 0; i < 3; ++i) pay += uint64_t(std::lround(R.missions[size_t(R.hubs[h].career[k])].fuel[i]));
        }
        for (size_t k = 0; k < R.hubs[h].challenges.size(); ++k) {
            medals = medals && s.challenges[h][k].medal[0];
            pay += uint64_t(std::lround(R.missions[size_t(R.hubs[h].challenges[k])].fuel[0]));
        }
    }
    rep_.check("X2", medals, "every medal won");
    rep_.check("X3", fuelEarned_ == pay + 600, "fuel earned " + std::to_string(fuelEarned_) + " = the scripts' rewards " + std::to_string(pay) + " + 600 of barrels");
    std::printf("   fuel: earned %llu, spent %llu, now %u\n", (unsigned long long)fuelEarned_, (unsigned long long)fuelSpent_, s.fuel);
    rep_.check("X3", blocked_.empty(), std::to_string(blocked_.size()) + " events blocked" + (blocked_.empty() ? std::string() : ": " + *blocked_.begin()));
    std::string modes;
    for (const auto& [mode, n] : modeRuns_) modes += std::string(" ") + gameModeName(mode) + ":" + std::to_string(n);
    rep_.note("R3", "races by mode:" + modes);
}

int Audit::run() {
    const auto t0 = std::chrono::steady_clock::now();
    for (auto [id, title] : std::initializer_list<std::pair<const char*, const char*>>{
             {"N1", "new career: hub, vehicles, fuel"},
             {"N2", "first tutorials (map, garage, race start)"},
             {"R1", "start a race from the menu (Map > hub > list > vehicle)"},
             {"R2", "start a race from its free-roam arch"},
             {"R3", "finish first (every career race and challenge)"},
             {"R4", "finish last (one per game mode)"},
             {"R5", "DNF / quit (one per game mode)"},
             {"R6", "career medals (the race's level, won = 1st), ranks and stars"},
             {"R7", "fuel rewards"},
             {"R8", "record times and places"},
             {"R9", "played flags, races finished"},
             {"R10", "challenges: won once, gold, fuel"},
             {"R11", "end-race pages (ranking, reward, unlocks, WhatNext)"},
             {"U1", "hubs unlock by stars (tiers)"},
             {"U2", "vehicles for sale (UnlockNeed)"},
             {"U3", "bonus cars given by races"},
             {"U4", "special traffic revealed by races (dopplers, Maverick)"},
             {"U5", "vehicle liveries (ALWAYS with the vehicle, POI pickups)"},
             {"U6", "pilot liveries"},
             {"U7", "vista points"},
             {"U8", "fuel barrels"},
             {"U9", "special traffic catches and rewards"},
             {"U10", "challenge discovery in free roam"},
             {"U11", "points of interest: FUEL's index order (0x735ac0) and grid walks"},
             {"P2", "race pages 0x56 / 0x59: name, medal row, fuel reward, no-vehicle warning, Back"},
             {"P3", "garage list drops the mission's forbidden vehicles (EMD_SetPlayerForbiddenVehiculeFilter)"},
             {"P5", "challenge end (states 8 / 9): page 121 fuel reward after a first gold, no ranking"},
             {"P6", "unlocked-hub page: map pictures <n>A / <n>B, MAP_2 pulse"},
             {"P7", "MessagesHUD: bonus-car banner picture + name, Accept skips the hold"},
             {"P8", "ranking scroll (6th / 7th racers), player name TT 1262, race HUD options forced on"},
             {"P4", "Damocles lock (0x479130): popup 0x99 outside the TTCsaltlake garage"},
             {"H1", "hub menu item states (0x47ca70 / 0x481f80), help key (0x47cbf0), View 13 camera slots"},
             {"H2", "camps list keys (0x47e610): clamped Up/Down, page jumps, Back"},
             {"E1", "buy a vehicle with fuel (garage)"},
             {"E2", "too expensive / locked"},
             {"E3", "vehicle upgrades: buy, gates, equip, the session's --upgrades (port deviation)"},  // [upgrades U2]
             {"E4", "opponent upgrades: the session's --ai-upgrade-tier, deterministic stages per race and seed (port deviation)"},  // [upgrades U4]
             {"E5", "the race's AI level (race +0x3e8): the session's --ai-level = the career page's medal row, challenges 0"},  // cr5
             {"T1", "camp travel (Map > camps)"},
             {"T2", "locked hubs refused"},
             {"S1", "save: every field round-trips"},
             {"S2", "autosave points (award yes, lost race no)"},
             {"S3", "DATA page load / save"},
             {"S4", "old saves load"},
             {"S5", "a session's state survives the process restart"},
             {"C1", "profile Progression counts"},
             {"C2", "profile Completion / Collection lines"},
             {"A1", "achievements awarded locally"},
             {"ED1", "race editor: create, validate, save, reload, play and finish, delete (EDITEDRACE_INFOS, CREATE race)"},  // PK21
             {"X1", "hubinfos.tsc: every race listed, startable, completed"},
             {"X2", "every promised unlock granted"},
             {"X3", "no reward lost, doubled or blocked"},
         })
        rep_.add(id, title);
    rules_ = std::make_unique<CareerRules>(loadCareerRules(data_));
    events_ = loadEvents(data_);
    try {
        const DpcFile usa1((fs::path(data_) / "CommonComponent/WORLD/USA1.DPC").string());
        zones_ = loadHubZones(usa1);
    } catch (const std::exception& e) {
        std::printf("   no hub zones: %s\n", e.what());
    }
    std::printf("career-audit: scratch save %s, %zu hub zones\n", save_.c_str(), zones_.zones.size());
    auto lap = [&](const char* what) {
        std::printf("   [%.1f s] %s done\n", std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count(), what);
        std::fflush(stdout);
    };
    newCareer(), lap("new career");
    firstHours(), lap("first hours");
    archStarts(true), lap("arches (new career)");
    economy(), lap("economy");
    racePages(), lap("race pages");
    hubMenu(), lap("hub menu");
    travel(), lap("travel");
    wholeCareer(), lap("whole career");
    upgrades(), lap("upgrades");  // [upgrades U2]
    aiUpgrades(), lap("ai upgrades");  // [upgrades U4]
    aiLevel(), lap("ai level");  // cr5
    freeRoam(), lap("free roam");
    archStarts(false), lap("arches");
    saves(), lap("saves");
    profile(), lap("profile");
    editorRaces(), lap("race editor");  // PK21
    achievements();
    crossCheck();
    finalState();
    const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    std::printf("\ncareer-audit: %d races, %d processes, %.0f s\n", races_, boots_, secs);
    return rep_.print();
}

}  // namespace

int runCareerAudit(const std::string& dataDir) {
    // Scratch only: HOME (settings.txt) and FUEL_CAREER_SAVE point into a new temporary folder.
    const char* tmp = std::getenv("TMPDIR");
    std::string tmpl = std::string(tmp && *tmp ? tmp : "/tmp/") + "/fuel-career-audit-XXXXXX";
    std::vector<char> buf(tmpl.begin(), tmpl.end());
    buf.push_back(0);
    const char* dir = mkdtemp(buf.data());
    if (!dir) {
        std::fprintf(stderr, "career-audit: no scratch folder\n");
        return 1;
    }
    const std::string userSave = Career::careerPath(), userSettings = settingsPath();
    setenv("HOME", dir, 1);
    setenv("FUEL_CAREER_SAVE", (std::string(dir) + "/career.sav").c_str(), 1);
    if (Career::careerPath() == userSave || settingsPath() == userSettings || Career::careerPath().find(dir) != 0) {
        std::fprintf(stderr, "career-audit: refusing to run outside the scratch folder\n");
        return 1;
    }
    // FUEL_CAREER_AUDIT_SAVE=<file>: also load a player's save (read only) and report what it holds.
    std::vector<uint8_t> extra;
    if (const char* x = std::getenv("FUEL_CAREER_AUDIT_SAVE"); x && *x) extra = readFile(x);
    Audit a(dataDir, dir);
    int bad = a.run();
    if (!extra.empty()) {
        Career c(loadCareerRules(dataDir));
        std::string err;
        const bool ok = c.deserialize(extra, &err);
        int owned = 0;
        for (const auto& v : c.state().vehicles) owned += v.owned != 0;
        std::printf("career-audit: extra save %s%s: %u fuel, %d stars, %d challenges, %d vehicles owned, hub %d\n", ok ? "loads" : "REFUSED",
                    err.empty() ? "" : (" (" + err + ")").c_str(), c.state().fuel, c.stars(), c.challengesWon(), owned, c.currentHub());
        bad += !ok || !err.empty();
    }
    std::error_code ec;
    if (!std::getenv("FUEL_CAREER_AUDIT_KEEP")) fs::remove_all(dir, ec);
    else std::printf("career-audit: kept %s\n", dir);
    return bad > 255 ? 255 : bad;  // an exit status: never 0 for 256 failures
}

}  // namespace fuel::ui
