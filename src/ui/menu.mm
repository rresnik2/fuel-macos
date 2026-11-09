#include "core/custom_vehicles.hpp"  // CUSTOM VEHICLES
#include "ui/menu.h"

#include "core/assembly.hpp"
#include "core/dpc.hpp"
#include "core/vehicle_params.hpp"
#include "core/events.hpp"
#include "core/hub_zones.hpp"
#include "ui/tsc.hpp"
#include "ui/ui_renderer.h"
#include "ui/help_texts.hpp"  // PK26

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace fs = std::filesystem;

namespace fuel::ui {
namespace {

std::string upper(std::string s) {
    for (char& c : s) c = char(std::toupper(static_cast<unsigned char>(c)));
    return s;
}

// The trailing number of an item id ("ITEM_ID_HALO3" -> 3), 0 when none.
int rowOf(const std::string& id) {
    size_t k = id.size();
    while (k > 0 && std::isdigit(static_cast<unsigned char>(id[k - 1]))) --k;
    return k == id.size() ? 0 : std::atoi(id.c_str() + k);
}

std::string formatTime(float s) {
    if (s <= 0) return "--:--.--";
    const int cs = int(std::lround(s * 100.0f));
    char b[32];
    std::snprintf(b, sizeof b, "%d:%02d.%02d", cs / 6000, (cs / 100) % 60, cs % 100);
    return b;
}

// ---- PK26: the text tables' "{STR_X}" variables (TransText 0x4e2820 at load, ui/help_texts.*) expand to the bound key or
// the pad glyph (0x57bdf0; filled by 0x4e2250 / PaddleChecker 0x4e36e0). Was P78's fixed key names (ENTER, ESC, Q, ...).
std::string pcButtons(std::string s) {
    s = helpTexts().expand(s);
    // P78: '²' escapes the next glyph (0x698470 / 0x43c960: the button icons input.PC's RemapTextAdd puts there); the
    // help lines separate their entries with "²²²" [port: a space each].
    for (size_t p; (p = s.find("\xc2\xb2")) != std::string::npos;) s.replace(p, 2, " ");
    return s;
}
// ---- end PK26

const char* kTabIcon[] = {"ITEM_ID_TOP_MENU_LAST", "ITEM_ID_TOP_MENU_GARAGE", "ITEM_ID_TOP_MENU_MAP", "ITEM_ID_TOP_MENU_PROFILE",
                          "ITEM_ID_TOP_MENU_OPTIONS", "ITEM_ID_TOP_MENU_EDITOR", "ITEM_ID_TOP_MENU_ONLINE"};
const char* kTabPage[] = {"Race", "Garage", "Map", "Profile", "Options", "Editor", "Online"};
const char* kTabTitle[] = {"TT_TOP_TITLE_TOP_RACE", "TT_TOP_TITLE_GARAGE_FREERIDE", "TT_TOP_TITLE_MAP", "TT_TOP_TITLE_TOP_PROFILE",
                           "TT_TOP_TITLE_TOP_OPTIONS", "TT_TOP_TITLE_TOP_EDITOR", "TT_TOP_TITLE_ONLINE"};

// Settings-file lines for the port's menu options (beside fuel::Settings' keys).
std::vector<std::pair<std::string, std::string>> readKeyValues(const std::string& path) {
    std::vector<std::pair<std::string, std::string>> out;
    std::ifstream f(path);
    for (std::string line; std::getline(f, line);) {
        const size_t eq = line.find('=');
        auto trim = [](std::string s) {
            while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back()))) s.pop_back();
            size_t a = 0;
            while (a < s.size() && std::isspace(static_cast<unsigned char>(s[a]))) ++a;
            return s.substr(a);
        };
        if (eq == std::string::npos || line[0] == '#') out.emplace_back("", line);
        else out.emplace_back(trim(line.substr(0, eq)), trim(line.substr(eq + 1)));
    }
    return out;
}

}  // namespace

MenuOptions loadMenuOptions(const std::string& path) {
    MenuOptions o;
    // ---- PK26: SetLanguageAuto 0x54c900 (Story/init.tsc, every start): game+0x330c / +0x33f5 (km / miles) = the language
    // is 1, 10, 0x13 or 0x14 (English US / UK, 0xc0c French Canada, 0x14); the player's saved choice replaces it after
    // the save load (0x55af50). [port] settings.txt `hud_miles` is that saved choice; without it the language decides.
    {
        std::string lang = "en";
        for (const auto& [k, v] : readKeyValues(path))
            if (k == "language") lang = v;
        o.miles = languageMiles(languageIndex(lang));
    }
    // ---- end PK26
    for (const auto& [k, v] : readKeyValues(path)) {
        const bool b = v == "1" || v == "true";
        if (k == "hud_race_info") o.raceInfo = b;
        else if (k == "hud_ranking") o.rankingInfo = b;
        else if (k == "hud_gps") o.gps = b;
        else if (k == "hud_speed_meter") o.speedMeter = b;
        else if (k == "hud_miles") o.miles = b;
        else if (k == "brightness_slider") o.brightness = std::clamp(float(std::atof(v.c_str())), 0.0f, 1.0f);
        else if (k == "music_volume") o.musicVolume = std::clamp(float(std::atof(v.c_str())), 0.0f, 1.0f);
        else if (k == "tutorials_seen") o.tutorialsSeen = uint32_t(std::strtoul(v.c_str(), nullptr, 10));
    }
    return o;
}

bool saveMenuOptions(const MenuOptions& o, const std::string& path) {
    auto lines = readKeyValues(path);
    auto num = [](float f) {
        std::ostringstream s;
        s << f;
        return s.str();
    };
    const std::vector<std::pair<std::string, std::string>> values = {
        {"hud_race_info", o.raceInfo ? "1" : "0"}, {"hud_ranking", o.rankingInfo ? "1" : "0"}, {"hud_gps", o.gps ? "1" : "0"},
        {"hud_speed_meter", o.speedMeter ? "1" : "0"}, {"hud_miles", o.miles ? "1" : "0"}, {"brightness_slider", num(o.brightness)},
        {"music_volume", num(o.musicVolume)}, {"tutorials_seen", std::to_string(o.tutorialsSeen)}};
    for (const auto& [k, v] : values) {
        bool done = false;
        for (auto& l : lines)
            if (l.first == k) l.second = v, done = true;
        if (!done) lines.emplace_back(k, v);
    }
    std::ofstream f(path);
    if (!f) return false;
    for (const auto& [k, v] : lines) f << (k.empty() ? v : k + " = " + v) << "\n";
    return bool(f);
}

Menu::Menu(const std::string& dataDir, const Settings& settings, MenuMode mode)
    : dataDir_(dataDir), settings_(settings), options_(loadMenuOptions()), mode_(mode) {
    scripts_ = loadMenuScripts(dataDir);
    port_ = PortOptions::from(settings.all), startSettings_ = settings;  // P101
    addGraphicsItem();                                                  // P101
    addUpgradesItem();                                                  // [upgrades U3]
    text_ = loadText(dataDir, settings.language);
    upgradeTexts();                                                     // [upgrades U3] the page's legends / help line / title
    assets_ = std::make_unique<MenuAssets>(dataDir);
    try {
        hubs_ = loadHubs(dataDir);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "menu: hubs: %s\n", e.what());
    }
    try {
        events_ = loadEvents(dataDir);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "menu: events: %s\n", e.what());
    }
    vehicles_ = loadVehicleInfos(dataDir);
    // What the port can start: a variant of that exact name in the vehicle's archive
    // (some variants' names are not decoded yet) with driving parameters.
    {
        // CUSTOM VEHICLES: the packs' VehiclePP blocks appended; each pack's archive checked like FUEL's below.
        const auto params = parseVehiclesParam(custom::readScript(findPathNoCase((fs::path(dataDir) / "CommonComponent").string(), "Param\\VehiclesParam.tsc")));
        for (const custom::Vehicle& cv : custom::vehicles()) {
            try {
                DpcFile dpc(cv.dpc);
                VehicleAssembler va(dpc);
                if (va.find(cv.name) && params.count(cv.name)) startable_.insert(cv.name);
            } catch (const std::exception& e) {
                std::fprintf(stderr, "menu: %s: %s\n", cv.dpc.c_str(), e.what());
            }
        }
        for (const char* cls : {"QUAD", "BIKE", "BUGGY", "MUSCLE", "SUV", "TRUCK", "BONUS1"}) {
            const std::string path = findPathNoCase((fs::path(dataDir) / "CommonComponent").string(), std::string("DATAS\\") + cls + ".DPC");
            if (path.empty()) continue;
            try {
                DpcFile dpc(path);
                VehicleAssembler va(dpc);
                for (const VehicleVariant& v : va.variants())
                    if (params.count(v.label)) startable_.insert(v.label);
            } catch (const std::exception& e) {
                std::fprintf(stderr, "menu: %s: %s\n", path.c_str(), e.what());
            }
        }
    }
    // ---- PK25: the in-game alert dialogs (FrontGlobalInGameData_G vf24 0x442990) and the storage state.
    dialogs_ = std::make_unique<InGameDialogs>(dataDir, *assets_, [this](uint32_t tt) { return textOf(tt); });
    {
        McAlert::Host& h = dialogs_->mcAlert().host;
        h.pause = [this] { alertPause_ = true; };          // 0x441670
        h.autosaveOff = [this] { setAutosaveOff(true); };  // storage +0x75 = 1
        // h.unpause (0x440b50 "UnPause"): the port's overlay keeps the pause menu the alert opened, as FUEL keeps the
        // pause page set (+0x6bac > 0 -> no UnPause); h.restartFromBuffer (0x4416f0): only from MCAlert state 6 / code
        // 0x1c, which nothing sets on PC.
    }
    if (const char* a = std::getenv("FUEL_AUTOSAVE_OFF"); a && std::string(a) == "1") storage_.autosaveOff = true;
    // ---- end PK25
    // P39: the career (rules from the scripts, progress from career.sav).
    try {
        career_ = std::make_unique<Career>(loadCareerRules(dataDir));
        career_->setDevMode(port_.devMode);  // [dev mode] settings.txt `dev_mode`: the overlay (before the vehicle pick below)
        if (loadCareerFile() != 0) {  // PK25: LoadSlot -> 0x4eb9c0 (a corrupt save asks TT 9113)
            std::error_code ec;
            careerUnreadable_ = fs::exists(Career::careerPath(), ec);
            std::fprintf(stderr, "menu: %s career save at %s: new career\n", careerUnreadable_ ? "unreadable" : "no", Career::careerPath().c_str());
        }
        // The vehicle the player last bought or drove (+0x33c8), else the first owned one the port can drive.
        const auto& rv = career_->rules().vehicles;
        const int cur = career_->state().currentVehicle();
        if (cur >= 0 && size_t(cur) < rv.size() && career_->vehicleOwned(rv[size_t(cur)].label) && startable_.count(rv[size_t(cur)].label))
            vehicle_ = rv[size_t(cur)].label;
        else if (!career_->vehicleOwned(vehicle_))
            for (const auto& v : rv)
                if (v.cls >= 0 && career_->vehicleOwned(v.label) && startable_.count(v.label)) { vehicle_ = v.label; break; }
        // P62: tutorial flags moved from settings.txt (P56 `tutorials_seen` mask) to SECTION_TUTORIAL; an old mask
        // migrates in (saved with the next career save).
        auto& tf = career_->state().tutorials;
        for (size_t i = 0; i < tf.size() && i < 32; ++i)
            if (options_.tutorialsSeen >> i & 1u) tf[i].seen = tf[i].off = 1;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "menu: career: %s\n", e.what());
    }
    setMode(mode);
}

Menu::~Menu() = default;

// ---- PK26: 0x594a50 queues "PlayLevel USA1 <vehicle>" (game+0x33c8, else 0x56a7d0's default; the port's vehicle_ follows the
// same rule in the constructor) and the level's first load runs Dialog_Loading vf23 0x58fc20: SaveManager_LoadSlot(0), then
// TeleportToHub <game+0x33c0, or 0 when < 0> [O].
MenuRequest Menu::careerBootRequest() const {
    MenuRequest r;
    r.kind = MenuRequest::FreeRide, r.vehicle = vehicle_, r.careerBoot = true;
    int hub = career_ ? career_->currentHub() : -1;
    if (hub < 0) hub = 0;
    if (career_) {
        const auto& R = career_->rules();
        if (size_t(hub) < R.hubs.size())
            for (const Hub& h : hubs_)
                if (upper(h.id) == upper(R.hubs[size_t(hub)].name)) r.hub = h.id;
    }
    if (r.hub.empty() && !hubs_.empty()) r.hub = hubs_[0].id;  // [port] no career rules: the first hub
    return r;
}
// ---- end PK26

int Menu::careerHub() const {
    if (!career_ || hubs_.empty()) return -1;
    return career_->rules().hubIndex(hubs_[size_t(hubIndex_)].id);
}

bool Menu::hubLocked() const {
    const int h = careerHub();
    return h >= 0 && !career_->hubUnlocked(h);
}

// PK11 (hub +0x248, set by the first arrival 0x481cd0; Career::enterHub): CAREER and CHALLENGES need it.
bool Menu::hubOpened() const {
    const int h = careerHub();
    // PK03: FUEL's arrival (0x481cd0: driving into the camp or TeleportToHub -> StartHubHUD) sets +0x248 through
    // Career::enterHub before the camp menu opens; the gate follows the flag (PK11's session-hub shortcut removed).
    // [port] the title menus (MenuMode::Main, a port construct with no hub arrival) keep every hub entered.
    return h < 0 || career_->state().hubs[size_t(h)].opened != 0 || mode_ != MenuMode::Pause;
}

// PK11 0x47ca70 [O]: CAREER / CHALLENGES (+9) = a pending hub (+0x954; the port's browsed hub), unlocked (0x579720) and
// +0x248 set; demo preview (game+0x32bc == 1) also turns CHALLENGES off. DATA = signed in (NetPlay vt+0x64; the port's
// profile is always local). 0x481f80: tutorial mode (game+0x32bc == 2) turns CAMPS and DATA off.
bool Menu::hubItemEnabled(const std::string& id) const {
    if (id == "ITEM_ID_LEFT_MENU5" || id == "ITEM_ID_LEFT_MENU4") {
        if (hubs_.empty() || hubLocked() || !hubOpened()) return false;
        return !(id == "ITEM_ID_LEFT_MENU4" && gameMode_ == 1);
    }
    if (id == "ITEM_ID_LEFT_MENU3" || id == "ITEM_ID_LEFT_MENU2") return gameMode_ != 2 && (id != "ITEM_ID_LEFT_MENU2" || career_ != nullptr);
    return true;
}

// PK11 0x47cbf0 [O]: the BandeauHelp key of the hub menu.
std::string Menu::hubHelpKey() const {
    if (hubs_.empty()) return "TT_BH_HUB_OUT_OF_AREA";
    if (hubLocked()) return "TT_BH_HUB_LOCKED_AND_NOT_VISITED";
    if (!hubOpened()) return "TT_BH_HUB_OPEN_AND_NOT_VISITED";
    return "TT_BH_MAIN";
}

// PK11 0x578f20 / 0x578f60 [O]: 1 career +0x1f0 / +0x1fc, 2 challenge +0x1d8 / +0x1e4, 3 shop +0x1c0 / +0x1cc, else the hub
// camera +0x1a8 / +0x1b4. The view state (hub +0x15c) is only ever written 0 in the shipped game (0x578c40 reset), so View 13
// always uses CAMERA_HUB / TARGET_HUB.
Menu::HubCameraSlot Menu::hubCameraSlot(int view) {
    switch (view) {
        case 1: return {0x1f0, 0x1fc, "CAMERA_CAREER"};
        case 2: return {0x1d8, 0x1e4, "CAMERA_CHALLENGE"};
        case 3: return {0x1c0, 0x1cc, "CAMERA_SHOP"};
        default: return {0x1a8, 0x1b4, "CAMERA_HUB"};
    }
}

// ---- PK17 (menu.h)
void Menu::markDirty() {
    if (mode_ == MenuMode::Pause) pauseDirty_ = true;  // +0x80 = 1
    else saveCareer();                                 // [port] the front-end menus save at once
}
// ---- end PK17

// PK25: SaveManager_Save 0x4ebea0 (mode 0: the whole career) and the save half of MemoryCardManager_C vf17 0x4eb9c0.
void Menu::saveCareer() {
    if (!career_) return;
    // 0x4ebea0: game+0x32bc == 2 (Demo_PublicDemo, the port's "tutorial mode") or autosave off (+0x75), with no storage
    // operation running (+0x3c == 0): the save is refused (+0x64 = 1) [O].
    if ((gameMode_ == 2 || storage_.autosaveOff) && storage_.op == 0) {
        storage_.saveErr = 1;
        std::printf("menu: save refused (%s)\n", storage_.autosaveOff ? "autosave disabled" : "demo / tutorial mode");
        return;
    }
    storage_.op = 1, storage_.saveAlert = true, storage_.slot = 0;  // +0x3c = 1, +0x77 = 1, +0x60 = mode
    const int rc = career_->write();  // g_SaveGame vt+0x3c -> 0x840000 (the port writes at once)
    storage_.saveErr = rc, storage_.saveAlert = false;  // vf17: poll done, +0x64 = err, +0x77 = 0
    if (rc == 0) {
        storage_.op = 0;  // +0x3c = 0
        ++saves_, careerUnreadable_ = false;
        return;
    }
    std::fprintf(stderr, "menu: cannot save %s (code 0x%x)\n", Career::careerPath().c_str(), rc);
    storage_.saveAlert = dialogs_ && dialogs_->mcAlert().saveError(rc);  // +0x77 = 0x441320(err); +0x3c stays 1 [O]
}

// P75 (P76 decode): the race info panel of the selected race, refilled on every selection change (0x480800 / 0x480000 ->
// 0x480ee0 career page 0x57 / 0x480560 challenge page 0x54) [O]: RACE_TYPE = TT by Mission_GameMode 0x4fe2d0 (career 0,1 ->
// 6000, 2..11 -> 5999 + mode, 12 -> 6012; challenge 0 -> 6008, 1,2,3,4,7,10,12 -> 6003, 5,6,9 -> 6008, 8,11 -> 6007),
// ENTRANTS = nb IA + 1, SURFACE_TYPE = TT record +0x164, VEHICLE_TYPE = TT +0x168. PK02: also drawn under the race pages
// (0x47fd20 / 0x47fe20 draw HubSelectCareerInfo / HubSelectChallengeInfo first).
void Menu::drawRaceInfo(UiRenderer& r, const Event& e, bool career) {
    Page* p = scripts_.page(career ? "HubSelectCareerInfo" : "HubSelectChallengeInfo");
    if (!p) return;
    const int mode = e.race.gameMode();
    uint32_t tt = 0;
    if (career) tt = mode <= 1 ? 6000 : mode <= 11 ? 5999 + uint32_t(mode) : 6012;
    else tt = (mode == 0 || mode == 5 || mode == 6 || mode == 9) ? 6008 : (mode == 8 || mode == 11) ? 6007 : 6003;
    auto dyn = [&](const char* id, const std::string& v) {
        if (Item* it = p->find(id)) it->hasDynText = true, it->dynText = v;
    };
    dyn("ITEM_ID_RACE_TYPE", textOf(tt));
    dyn("ITEM_ID_ENTRANTS", std::to_string(e.race.aiCount + 1));
    dyn("ITEM_ID_SURFACE_TYPE", e.surfaceText ? textOf(e.surfaceText) : "");
    dyn("ITEM_ID_VEHICLE_TYPE", e.vehicleText ? textOf(e.vehicleText) : "");
    for (Item& it : p->items)
        if (it.id.rfind("ITEM_ID_FUEL_REWARD", 0) == 0) it.visible = false;  // off-canvas (x 1489) in the script
    drawPage(r, *p);
}

// ---- PK02: the race pages 0x56 / 0x59 -------------------------------------------------------------------------------
// FUEL goes list -> race page -> vehicle pick (garage states 0xaf / 0xb0) -> start [O: b031 / b041 / b078]:
//   0x47d550 (career Accept): 0x46b970 (page +0x84 hub, +0x88 = best rank (+0x120) - 1, at least 0 = the preselected medal
//     row, +0x8c index; fill 0x46c020), 0x46f330, state 0x56. 0x47d660 (challenge Accept): fuel reward 0x481280, texts
//     0x481430, info panel 0x480ee0, warning 0x4813a0, state 0x59. Draw 0x47fd20 / 0x47fe20: the info panel
//     (HubSelectCareerInfo / HubSelectChallengeInfo), then HubSelectCareerMood / HubChallengesDescription.
//   Keys 0x47e310 (via 0x46bc60): Up / Down move the medal row (0..2, no wrap; hover over a VALID_BOX row too); Accept ->
//     0x4814e0 (the race's garage) + state 0xaf; Back -> the list (0x55), or the menu closes when a world arch opened the
//     page (0x4819f0). 0x47ed60: Accept needs +0x959 (a vehicle the race allows, 0x4813a0) else nothing; -> 0x481530 + 0xb0;
//     Back -> the list (0x58) / close (0x4819f0).
//   0x47f120 / 0x47f200: Back in the garage pick returns to 0x56 / 0x59 (+ 0x4789e0, the garage's tab leave).
// [port] a challenge with its own vehicle starts at once from the page (P99: 0x47efb0 does not read the garage's vehicle);
// FUEL shows its garage in between. The medal row the player aimed at (+0x88) goes to the garage (0x46bb90) and from there
// to the race (0x47eee0: TTMN "<mission>" <index> <row> -> RestartMission -> race +0x3e8): the AI's target time is
// AddMissionTime value `row` (cr5: sessionArgs --ai-level).
void Menu::openRace(const Event& e, bool fromArch) {
    pendingEvent_ = e.id;
    raceFromArch_ = fromArch;
    screen_ = Screen::RacePage;
    focus_ = 0, listFirst_ = 0;
    const MissionStatus* ms = career_ ? career_->status(e.id) : nullptr;
    raceRow_ = std::max(0, (ms ? ms->bestRank : 3) - 1);
    setTitle("TT_TOP_TITLE_MAP");
}

void Menu::raceKey(MenuKey k) {
    const Event* e = eventById(pendingEvent_);
    if (!e) return goTab(TabMap);
    const bool career = e->kind == Event::Career;
    if (k == MenuKey::Back) {
        if (raceFromArch_) {  // 0x4819f0: the menu closes (the map reset 0x47ee80, the arch prompt is cleared)
            raceFromArch_ = false, pendingEvent_.clear();
            if (mode_ == MenuMode::Pause) request_.kind = MenuRequest::Resume;
            else goTab(TabMap);
            return;
        }
        const auto list = hubEvents(e->kind);
        int idx = 0;
        for (size_t i = 0; i < list.size(); ++i)
            if (list[i]->id == e->id) idx = int(i);
        pendingEvent_.clear();
        openList(career ? Screen::CareerList : Screen::ChallengeList);
        focus_ = idx, listFirst_ = idx / (career ? 3 : 5) * (career ? 3 : 5);
        return;
    }
    if (career && k == MenuKey::Up && raceRow_ > 0) --raceRow_;
    if (career && k == MenuKey::Down && raceRow_ < 2) ++raceRow_;
    if (k != MenuKey::Accept) return;
    if (!career && !raceHasVehicle(*e)) return;  // 0x47ed60: +0x959 == 0 -> nothing (the button bar has no Accept)
    if (startForced(*e)) return;
    openList(Screen::VehicleList);
}

const char* const kMoodIcon[3] = {"MAP_CAREER_GOLD_ICON", "MAP_CAREER_SILVER_ICON", "MAP_CAREER_BRONZE_ICON"};

void Menu::drawRace(UiRenderer& r) {
    const Event* e = eventById(pendingEvent_);
    if (!e) return;
    const bool career = e->kind == Event::Career;
    drawRaceInfo(r, *e, career);
    Page* p = scripts_.page(career ? "HubSelectCareerMood" : "HubChallengesDescription");
    if (!p) return;
    const MissionStatus* ms = career_ ? career_->status(e->id) : nullptr;
    const int rank = ms ? ms->bestRank : 3;  // +0x120
    const int mi = career_ ? career_->rules().missionIndex(e->id) : -1;
    const CareerRules::Mission* m = mi >= 0 ? &career_->rules().missions[size_t(mi)] : nullptr;
    auto dyn = [&](const char* id, const std::string& v) {
        if (Item* it = p->find(id)) it->hasDynText = true, it->dynText = v;
    };
    // 0x46b970 / 0x481430: ITEM_ID_RACE_NAME = TT(mission +0x15c) (the name text of the hub list), the challenge's
    // ITEM_ID_CHALLENGE_DESCRIPTION = TT(+0x160).
    if (Item* it = p->find("ITEM_ID_RACE_NAME")) it->hasDynText = true, it->dynText = e->nameText ? textOf(e->nameText) : e->id;
    if (career) {
        // 0x46c020: row i (1..3) is open while i - 1 < rank (a medal not won yet): its bar, FUEL label and reward are enabled,
        // the reward = ftol(mission +0x94 + 4 (i - 1)); a won row shows its medal (MAP_CAREER_* from the table at +0xcf8);
        // an open row shows the +0xcf4 image from the selected row down. HALO i only on the selected row (+0x88).
        for (int i = 1; i <= 3; ++i) {
            const std::string n = std::to_string(i);
            const bool open = i - 1 < rank;
            if (Item* it = p->find("ITEM_ID_HALO" + n)) it->visible = i == raceRow_ + 1, it->focus = true;
            if (Item* it = p->find("ITEM_ID_MEDAL_SLOT" + n)) {
                it->visible = open ? raceRow_ + 1 <= i : true;
                it->imgNormal = it->imgEnabled = it->imgFocus = open ? "MAP_CAREER_NONE_ICON" : kMoodIcon[i - 1];
            }
            for (const char* id : {"ITEM_ID_BARRE_STATE", "ITEM_ID_FUEL_LABEL", "ITEM_ID_FUEL_REWARD"})
                if (Item* it = p->find(id + n)) it->enabled = open;
            dyn(("ITEM_ID_FUEL_REWARD" + n).c_str(), open && m ? std::to_string(int(m->fuel[i - 1])) : "0");
        }
    } else {
        dyn("ITEM_ID_CHALLENGE_DESCRIPTION", e->descText ? textOf(e->descText) : "");
        if (Item* it = p->find("ITEM_ID_CHALLENGE_DESCRIPTION")) it->hasDynText = true;
        // 0x481280: GROUP_FUEL_REWARD while the challenge is not won (rank == 3), ITEM_ID_FUEL_REWARD = ftol(mission +0x94).
        const bool showFuel = rank == 3;
        dyn("ITEM_ID_FUEL_REWARD", m ? std::to_string(int(m->fuel[0])) : "0");
        // 0x4813a0: GROUP_CHALLENGE_WARNING while no vehicle is allowed (+0x959 == 0).
        const bool warn = !raceHasVehicle(*e);
        for (Item& it : p->items) {
            if (it.group == "GROUP_FUEL_REWARD") it.visible = showFuel;
            if (it.group == "GROUP_CHALLENGE_WARNING") it.visible = warn;
        }
    }
    for (Item& it : p->items)
        if (it.id.rfind("ITEM_ID_VALID_BOX", 0) == 0 || it.id == "ITEM_ID_BOX_VALIDATION") it.visible = false;  // click areas
    drawPage(r, *p);
}

std::string Menu::raceFuelReward() const {
    const Event* e = eventById(pendingEvent_);
    if (screen_ != Screen::RacePage || !e || e->kind != Event::Challenge) return {};
    const MissionStatus* ms = career_ ? career_->status(e->id) : nullptr;
    if ((ms ? ms->bestRank : 3) != 3) return {};
    const int mi = career_ ? career_->rules().missionIndex(e->id) : -1;
    return std::to_string(mi >= 0 ? int(career_->rules().missions[size_t(mi)].fuel[0]) : 0);
}

bool Menu::raceWarning() const {
    const Event* e = eventById(pendingEvent_);
    return screen_ == Screen::RacePage && e && e->kind == Event::Challenge && !raceHasVehicle(*e);
}

std::string Menu::raceTitle() const {
    const Event* e = eventById(pendingEvent_);
    return screen_ == Screen::RacePage && e && e->nameText ? textOf(e->nameText) : std::string();
}

// Garage list: owned vehicles are chosen; others open FUEL's info boxes (locked, too
// expensive, or buy yes/no; the buy itself is 0x4ce590).
// PK02, 0x479130 (called by the garage's accept of an owned vehicle, EDI = its record): 1 for any vehicle but the Damocles
// (MESH_ID_BONUS_ROAD_3); 2 for the Damocles in the garage of a challenge (+0xe4 == 2) whose mission is "TTCsaltlake"
// (case-insensitive) = accepted; 0 otherwise -> 0x4788a0, state 0x99: the popup TT_PAGE_VEHICLE_DAMOCLES (TT 5981).
int Menu::damoclesLock(const std::string& label) const {
    if (strcasecmp(label.c_str(), "BONUS_ROAD_3") != 0) return 1;
    const Event* e = eventById(pendingEvent_);
    return e && e->kind == Event::Challenge && strcasecmp(e->id.c_str(), "TTCsaltlake") == 0 ? 2 : 0;
}

void Menu::vehicleAccept(const std::string& label) {
    if (!career_ || career_->vehicleOwned(label) || career_->rules().vehicleIndex(label) < 0) {
        if (career_ && career_->vehicleOwned(label) && damoclesLock(label) == 0) {  // PK02: 0x479130 -> 0x4788a0
            openOk("TT_PAGE_VEHICLE_DAMOCLES", [] {});
            return;
        }
        // PK02, 0x4786e0: in the TTCsaltlake garage the player's current vehicle is stashed (+0x33c4) and restored by the
        // race kill (0x62ba30, takeRestore): the free-ride vehicle stays what it was.
        if (damoclesLock(label) == 2 && !pendingEvent_.empty()) {
            requestStart({MenuRequest::StartEvent, pendingEvent_, "", label});
            return;
        }
        vehicle_ = label;
        if (career_) markDirty();  // PK17: 0x4786e0 +0x80 = 1 (0x4787a3)
        if (!pendingEvent_.empty()) requestStart({MenuRequest::StartEvent, pendingEvent_, "", vehicle_});  // P62: 0x4786e0
        else garageReturn("ITEM_ID_TOPMENU_GARAGE_VEHICLE", true);  // PK20: 0x4791b0 Accept -> the next item (was goTab)
        return;
    }
    const int price = career_->vehiclePrice(label);
    if (!career_->vehicleUnlocked(label) || price < 0) infoBox_ = "InfoBoxGarageVehicleIsLocked";
    else if (uint32_t(price) > career_->state().fuel) infoBox_ = "InfoBoxGarageVehicleIsTooExpensive";
    else infoBox_ = "InfoBoxGarageYesNo", buyTarget_ = label;
}

std::string Menu::textOf(uint32_t tt) const {
    const std::string* s = text_.get(tt);
    return s ? pcButtons(*s) : std::string();
}

// PK08 0x4b4220 (see drawing): race_reward's filled items; medals: the bars follow the award (the race's level, 0x630dd0).
Menu::RaceRewardItems Menu::raceRewardItems() {
    RaceRewardItems out;
    Page* p = scripts_.page("race_reward");
    if (!p) return out;
    // 0x4b4220: bar i = medal i (gold, silver, bronze): won now -> MAP_CAREER_*_ICON,
    // had before -> MEDAL_*_GRISE, else hidden; FUEL_REWARD i = AddMissionFuel i;
    // bar 4: MAP_CAREER_NONE_ICON, fuel won now and the number of new stars.
    static const char* const kNow[3] = {"MAP_CAREER_GOLD_ICON", "MAP_CAREER_SILVER_ICON", "MAP_CAREER_BRONZE_ICON"};
    static const char* const kHad[3] = {"MEDAL_GOLD_GRISE", "MEDAL_SILVER_GRISE", "MEDAL_BRONZE_GRISE"};
    const int mi = career_ ? career_->rules().missionIndex(sessionEvent_) : -1;
    for (int i = 0; i < 3; ++i) {
        const std::string n = std::to_string(i + 1);
        if (Item* it = p->find("ITEM_ID_MEDAL_SLOT" + n)) {
            it->visible = award_.medalNew[i] || award_.medalHad[i];
            it->imgNormal = it->imgEnabled = it->imgFocus = award_.medalNew[i] ? kNow[i] : kHad[i];
            out.slot[size_t(i)] = it->visible ? it->imgNormal : "";
        }
        if (Item* it = p->find("ITEM_ID_FUEL_REWARD" + n)) {
            it->hasDynText = true;
            it->dynText = out.fuel[size_t(i)] = mi >= 0 ? std::to_string(std::lround(career_->rules().missions[size_t(mi)].fuel[i])) : "0";
        }
    }
    if (Item* it = p->find("ITEM_ID_MEDAL_SLOT4")) it->imgNormal = it->imgEnabled = it->imgFocus = "MAP_CAREER_NONE_ICON";
    if (Item* it = p->find("ITEM_ID_FUEL_REWARD4")) it->hasDynText = true, it->dynText = out.fuelWon = std::to_string(award_.fuelWon);
    if (Item* it = p->find("ITEM_ID_NBSTARS")) it->hasDynText = true, it->dynText = out.stars = std::to_string(award_.newMedals);
    return out;
}

// PK08 0x4b7630 (see drawing): page 121's labels.
std::pair<std::string, std::string> Menu::fuelRewardTexts() {
    Page* p = scripts_.page("EndRaceFuelChallengeReward");
    if (!p) return {};
    const int mi = career_ ? career_->rules().missionIndex(sessionEvent_) : -1;
    std::pair<std::string, std::string> out;
    if (Item* it = p->find("ITEM_ID_FUEL_WON"))
        it->hasDynText = true, it->dynText = out.first = "+" + std::to_string(mi >= 0 ? int(career_->rules().missions[size_t(mi)].fuel[0]) : int(award_.fuelWon));
    if (Item* it = p->find("ITEM_ID_FUEL_TOTAL")) {  // 0x46c900: the number through the item's own text (TT 994 "YOUR FUEL: %d")
        const std::string t = textOf(it->tt);
        char b[96];
        std::snprintf(b, sizeof b, t.find('%') != std::string::npos ? t.c_str() : "%d", int(career_ ? career_->state().fuel : 0u));
        it->hasDynText = true, it->dynText = out.second = b;
    }
    return out;
}

// PK08 0x4b63a0 / 0x4b64e0: the unlocked-hub page's map pictures "<n>A" / "<n>B" (n = game +0x346c) and MAP_2's pulse alpha.
std::array<std::string, 2> Menu::unlockMapPictures(float* alpha2) {
    Page* p = scripts_.page("EndRace_Event_Unlocked2");
    if (!p) return {};
    const int n = unlockMapN_ >= 0 ? unlockMapN_ : career_ ? career_->state().get<int32_t>(0xb4) : 0;
    std::array<std::string, 2> out;
    if (Item* it = p->find("ITEM_ID_MAP")) it->imgNormal = it->imgEnabled = it->imgFocus = out[0] = std::to_string(n) + "A";
    if (Item* it = p->find("ITEM_ID_MAP_2")) {
        it->imgNormal = it->imgEnabled = it->imgFocus = out[1] = std::to_string(n) + "B";
        it->visible = true;
        it->cNormal.a = it->cEnabled.a = it->cFocus.a = (std::sin(mapPulse_ * 8.0f) + 1.0f) * 0.5f;
        if (alpha2) *alpha2 = it->cNormal.a;
    }
    return out;
}

std::string Menu::rankName(int row) const {
    const int i = rankIndex(row);
    return i < 0 ? std::string() : results_[size_t(i)].player ? playerName() : opponentName(results_[size_t(i)]);
}

// PK16 0x620c20 / RaceRec_Reset: an opponent's ranking name is its race record's name, the localised text of the id the
// race setup gave it (TT 1500..1573); a row without one (the audit's rows) shows the vehicle's name.
std::string Menu::opponentName(const ResultRow& r) const {
    if (r.nameTT > 0) {
        const std::string n = textOf(r.nameTT);
        if (!n.empty()) return n;
    }
    return vehicleName(r.name);
}

// PK08 0x5c0d70 / 0x72ed50: a ranking row's name for the local player is the profile (gamer) name, else TT 1262 "User". The
// port has no GFWL profile, so it is always the fallback.
std::string Menu::playerName() const {
    return textOf(1262);
}

// PK08 0x4b1c50: the ranking window. Row clamp: +0xe6c above the racer count or above 5 -> min(5, count); below 1 -> 1; no racers -> 0.
void Menu::rankClamp() {
    const int count = int(results_.size());
    if (unsigned(count) < unsigned(rankRow_) || 5 < rankRow_) rankRow_ = std::min(5, count);
    if (rankRow_ < 1) rankRow_ = 1;
    if (count == 0) rankRow_ = 0;
}

int Menu::rankIndex(int row) const {
    const int i = rankSel_ - rankRow_ + row;
    return i >= 0 && i < int(results_.size()) ? i : -1;
}

std::string Menu::vehicleName(const std::string& label) const {
    for (const VehicleInfo& v : vehicles_)
        if (v.name == label && v.nameText) {
            const std::string n = textOf(v.nameText);
            if (!n.empty()) return n;
        }
    return label;
}

void Menu::setSession(const std::string& event, const std::string& hub, const std::string& vehicle) {
    sessionEvent_ = event, sessionHub_ = upper(hub), sessionVehicle_ = vehicle;
    raceRestarted();  // P99: a new session has no results yet
    for (const Event& e : events_)
        if (sessionHub_.empty() && e.id == event) sessionHub_ = upper(e.hub);
    // PK02: the TTCsaltlake race runs on the Damocles lent by the garage (0x4786e0 stashes the free-ride vehicle in +0x33c4);
    // the race kill gives it back (0x62ba30), so the free-ride vehicle never becomes the Damocles here.
    if (!vehicle.empty() && !(strcasecmp(event.c_str(), "TTCsaltlake") == 0 && strcasecmp(vehicle.c_str(), "BONUS_ROAD_3") == 0)) vehicle_ = vehicle;
    for (size_t i = 0; i < hubs_.size(); ++i)
        if (upper(hubs_[i].id) == sessionHub_) hubIndex_ = int(i);
    sessionLivery_ = career_ ? career_->vehicleLivery(vehicle_) : -1;  // P78: what the session was started with (0x47be40)
    sessionColor_ = career_ ? career_->vehicleColor(vehicle_) : -1;
    sessionStages_ = career_ ? career_->sessionUpgrades(vehicle_, port_.upgrades) : upg::Stages{};  // [upgrades U3] its --upgrades
    sessionForSale_ = vehiclesForSale();  // mission +0x54 (0x62af90's reference count)
    // PK08 0x4b16a0 (MissionHUD vf00, run whenever the in-game HUDs initialise): the race-info and ranking display options
    // (game +0x33f0 / +0x33f1) are forced back on.
    options_.raceInfo = options_.rankingInfo = true;
}

// minimap-fix: 0x577d70 -> 0x579a30 per hub (hub list order): tutorial +0x20c, career +0x214, challenge +0x21c lists by the
// mission's name CRC; 3 when no hub lists it (Career::locate walks the same lists of hubinfos.tsc).
int Menu::missionList(const std::string& id) const {
    int list = 3;
    if (career_) return career_->locate(id, nullptr, &list, nullptr) ? list : 3;
    for (const Event& e : events_)  // [port] no career loaded: the event list's kind (built from the same hub lists)
        if (e.id == id) return e.kind == Event::Career ? 1 : e.kind == Event::Challenge ? 2 : 3;
    return 3;
}

void Menu::setResults(const std::vector<ResultRow>& rows, bool won, int level) {
    results_ = rows;
    award_ = {};
    resultsWon_ = won, resultsList_ = 2, endRaceClock_ = 0;
    challengeEnd_ = fuelChallenge_ = false, rankSel_ = rankRow_ = 0;
    if (!career_ || sessionEvent_.empty()) return;
    // PK21: an edited race (mission list kind 3, 0x577d70) ends in state 0xa (won) / 0xb (lost) with no career award (0x6300c0).
    if (fuel::editor::slotOfName(sessionEvent_) >= 0 && !career_->locate(sessionEvent_, nullptr, nullptr, nullptr)) {
        resultsList_ = 3, resultsWon_ = won;
        return;
    }
    int place = 0;
    double time = 0;
    for (size_t i = 0; i < rows.size(); ++i)
        if (rows[i].player) place = int(i) + 1, time = rows[i].time;
    int hub, list, index;
    if (place == 0 || !career_->locate(sessionEvent_, &hub, &list, &index)) return;
    // medals: MISSION_ENDING_WON (state 6) is the race's end-condition list 0x62cff0 (race +0x384, tested in 0x6300c0;
    // asm 0x630285 SetState 6, the lose list +0x38c 0x630370 SetState 7) [O]: scenario (mission +0x58 -> race +0x27c,
    // 0x62e792) default (A to B, checkpoint, circuit, baja ...) 0x401 = the player finished (racer +0xdc == 4, bit 0x400)
    // AND place 1 (bit 1); knock-out the same; helico 0x400 (finished; lost when the helicopter's record ends first);
    // catch 0x1000 (all caught); time trials bit 2 (a racer of the list +0x3d8 past state 3). Bit 0x800 (place < 4) is in no list: 2nd and 3rd
    // lose (state 7: no medal, no record, no save). That is RaceRules' outcome for every kind, so `won` is used as is.
    // The medal is the race's level (race +0x3e8 = Race_Start's 4th argument, 0x62e7a1), not the place (0x630dd0).
    const bool careerWon = won;
    if (level < 0) {  // [port] no session level: the row the career page preselects (0x46b970: best rank - 1, at least 0)
        const MissionStatus* ms = career_->status(sessionEvent_);
        level = std::max(0, (ms ? ms->bestRank : 3) - 1);
    }
    level = list == 1 ? std::clamp(level, 0, 2) : 0;  // challenges: TTMN ... 0 (0x47efb0); 0x630ca0 ignores it
    resultsWon_ = careerWon, resultsList_ = list;
    // PK08: mission +0x50 (0x630ca0): a won challenge whose gold medal was not taken yet (rank before > 0, no medal 0).
    challengeEnd_ = list == 2;
    if (const MissionStatus* st = career_->status(sessionEvent_)) fuelChallenge_ = list == 2 && careerWon && st->bestRank > 0 && !st->medal[0];
    award_ = career_->finishEvent(sessionEvent_, careerWon, level, place, time);
    if (award_.counted) saveCareer();  // console "SaveGameData" after the award (0x6300c0)
    std::printf("career: %s %s place %d level %d: %d new stars, +%u fuel (now %u, %d stars)%s\n", sessionEvent_.c_str(), careerWon ? "won" : "lost", place, level,
                award_.newMedals, award_.fuelWon, career_->state().fuel, career_->stars(), award_.hubsUnlocked.empty() ? "" : ", hub unlocked");
    for (int t : award_.trophies) std::printf("career: achievement %d unlocked\n", t);
}

void Menu::setMode(MenuMode m) {
    mode_ = m;
    request_ = {};
    if (m == MenuMode::Title) screen_ = Screen::Title;
    else if (m == MenuMode::Results) screen_ = Screen::EndRace, focus_ = 0, endRaceClock_ = 0;  // P75: states 6 / 7 first
    else goTab(m == MenuMode::Pause ? TabLast : TabMap);
}

bool Menu::tabAvailable(int tab) const {
    if (tab == TabLast) return mode_ == MenuMode::Pause || mode_ == MenuMode::Results;
    // PK18 0x46f080 / 0x46f250 [O]: a race running (menu mode 3 / 4, 0x46e650) disables GARAGE, ONLINE and EDITOR; the tutorial
    // mode (game+0x32bc == 2) disables PROFILE and EDITOR.
    if (tab == TabGarage && menuMode() >= 3) return false;
    if (tab == TabProfile && gameMode_ == 2) return false;
    if (tab == TabEditor) return menuMode() < 3 && gameMode_ != 2;  // PK21: the race editor (0x46f080 / 0x46f250)
    return tab == TabGarage || tab == TabMap || tab == TabOptions || tab == TabProfile;  // P66: Profile; Online: not in the port
}

void Menu::setTitle(const std::string& key) {
    if (Page* p = scripts_.page("TopTitle"))
        if (Item* t = p->find("ITEM_ID_TITLE"))
            for (size_t i = (t->hasDynText = false, 0); i < t->entries.size(); ++i)  // P101: a port title (GRAPHICS) clears
                if (t->entries[i].key == key) t->entryIndex = int(i);
}

void Menu::goTab(int tab) {
    tab_ = std::clamp(tab, 0, TabCount - 1);
    screen_ = Screen::Tab;
    focus_ = 0;
    garage_.page.clear(), garage_.state = 1;  // P66
    profile_ = {};                            // P66
    mapData_ = false;                         // P66
    ctrlMenu_ = false;                        // P101
    setTitle(kTabTitle[tab_]);
    if (tab_ == TabGarage && !pendingEvent_.empty()) setTitle("TT_TOP_TITLE_GARAGE_RACE");
    if (tab_ == TabMap) mapOpen();  // P44: the map opens on the player
    if (tab_ == TabEditor) editorEnter();  // PK21: 0x475e70
    // P62: menu tutorials on the tab switch 0x46ee10: Race (case 0) -> 11; Garage 0x478400 -> 0x4782f0 -> 12; Map 0x482410 -> 13.
    if (tab_ == TabLast) queueTutorial(11);
    if (tab_ == TabGarage) queueTutorial(12);
    if (tab_ == TabMap) queueTutorial(13);
}

Page* Menu::leftPage() {
    if (screen_ != Screen::Tab) return nullptr;
    if (tab_ == TabGarage && !garage_.page.empty()) return scripts_.page(garage_.page);  // P66: Driver menus
    if (tab_ == TabMap && mapData_) return scripts_.page("HubData");                       // P66: DATA (0x5b)
    if (tab_ == TabOptions && ctrlMenu_) return scripts_.page("PageOptionsController");    // P101: state 0x29
    return scripts_.page(tab_ == TabMap ? "TemplateLeftMenu" : kTabPage[tab_]);
}

std::vector<Item*> Menu::focusables() {
    std::vector<Item*> out;
    Page* p = leftPage();
    if (!p) return out;
    if (tab_ == TabMap) applyHubStates();  // PK11: the item states are set when the page is set up (0x47ca70), not at draw
    if (tab_ == TabEditor) editorItemStates();  // PK21: 0x4770b0
    for (Item& it : p->items)
        if (it.isButton() && it.group == "GROUP_LEFT_MENU" && it.visible && it.enabled) out.push_back(&it);
    std::sort(out.begin(), out.end(), [](const Item* a, const Item* b) { return a->y < b->y; });
    return out;
}

std::vector<const Event*> Menu::hubEvents(Event::Kind kind) const {
    std::vector<const Event*> out;
    const std::string hub = hubs_.empty() ? std::string() : upper(hubs_[size_t(hubIndex_)].id);
    for (const Event& e : events_)
        if (e.kind == kind && upper(e.hub) == hub) {
            // The challenge list shows the hub's available challenges only (0x480800 via 0x5798b0).
            if (kind == Event::Challenge && career_ && career_->status(e.id) && !career_->eventAvailable(e.id)) continue;
            out.push_back(&e);
        }
    return out;
}

// P66: 0x47a7d0 lists the vehicles named by texts 3401..3475 (0x469db0(list, 0xd93) from 0xd49: "MESH_ID_..." each,
// stopping at an empty or "Empty Text" one), matched by mesh crc; with param 4 = 0 only unlocked records (+0) [O].
std::vector<const VehicleInfo*> Menu::vehicleOrder(bool unlockedOnly) const {
    std::vector<const VehicleInfo*> out;
    for (uint32_t t = 0xd49; t <= 0xd93; ++t) {
        const std::string* s = text_.get(t);
        if (!s || s->empty() || s->find("Empty Text") != std::string::npos) break;
        std::string n = *s;
        while (!n.empty() && std::isspace(static_cast<unsigned char>(n.back()))) n.pop_back();
        if (n.rfind("MESH_ID_", 0) == 0) n = n.substr(8);
        for (const VehicleInfo& v : vehicles_)
            if (upper(v.name) == upper(n)) {
                if (!unlockedOnly || !career_ || career_->rules().vehicleIndex(v.name) < 0 || career_->vehicleUnlocked(v.name)) out.push_back(&v);
                break;
            }
    }
    if (out.empty()) {  // [port] no order list (another text table): the script order
        for (const VehicleInfo& v : vehicles_)
            if (!unlockedOnly || !career_ || career_->rules().vehicleIndex(v.name) < 0 || career_->vehicleUnlocked(v.name)) out.push_back(&v);
    } else {  // CUSTOM VEHICLES: the packs' vehicles are not in FUEL's name list; they follow it, in script order
        for (const VehicleInfo& v : vehicles_)
            if (custom::isCustom(v.name) && std::find(out.begin(), out.end(), &v) == out.end() &&
                (!unlockedOnly || !career_ || career_->rules().vehicleIndex(v.name) < 0 || career_->vehicleUnlocked(v.name)))
                out.push_back(&v);
    }
    return out;
}

const Event* Menu::eventById(const std::string& id) const {
    if (id.empty()) return nullptr;
    for (const Event& e : events_)
        if (e.id == id) return &e;
    return nullptr;
}

// PK02, 0x4fe250: the garage list (0x47a7d0 with a mission) keeps an unlocked vehicle unless its index in the game's vehicle
// table (0x56a040: the table's mesh crc match; here the AddVehicleInfo order of VehiclesDef.tsc) is in the mission's
// forbidden list (+0xb8/+0xbc, EMD_SetPlayerForbiddenVehiculeFilter 0x5517d0).
bool Menu::vehicleForbidden(const Event* e, const VehicleInfo& v) const {
    if (!e || e->race.forbidden.empty() || vehicles_.empty()) return false;
    const int idx = int(&v - vehicles_.data());
    return std::find(e->race.forbidden.begin(), e->race.forbidden.end(), idx) != e->race.forbidden.end();
}

// PK02, 0x481340: +0x959 = the list 0x47a7d0(menu, mission, 0, 0) is not empty: unlocked vehicles of the name list (texts
// 3401..) the mission does not forbid; no class filter [O]. (The port's garage adds its class filter on top, [port].)
bool Menu::raceHasVehicle(const Event& e) const {
    if (shotNoVehicle_) return false;  // --menu-shot HubChallengesWarning
    for (const VehicleInfo* v : vehicleOrder(true))
        if (!vehicleForbidden(&e, *v)) return true;
    return false;
}

std::vector<const VehicleInfo*> Menu::vehicleChoices() const {
    std::string family;
    for (const Event& e : events_)
        if (e.id == pendingEvent_) family = upper(e.vehicleFamily);
    // The event's class ("QUAD", or several joined: "MUSCLE_BUGGY"), or one vehicle by name.
    bool byName = false;
    for (const VehicleInfo& v : vehicles_)
        if (upper(v.name) == family) byName = true;
    std::vector<const VehicleInfo*> out;
    const Event* pending = eventById(pendingEvent_);
    for (const VehicleInfo* vp : vehicleOrder(true)) {  // P66: FUEL's order and unlocked vehicles only (0x47a7d0)
        const VehicleInfo& v = *vp;
        if (v.family == "UNKNOWN") continue;  // test and traffic vehicles (VEH_TEST_*, TRAFFIC_TRUCK)
        if (pending && vehicleForbidden(pending, v)) continue;  // PK02: the mission's forbidden list (0x4fe250)
        if (!startable_.count(v.name)) continue;
        bool ok = family.empty();
        if (byName) ok = upper(v.name) == family;
        else if (!family.empty()) {
            std::stringstream ss(family);
            for (std::string part; std::getline(ss, part, '_');)
                if (part == v.family) ok = true;
        }
        if (ok) out.push_back(&v);
    }
    return out;
}

void Menu::openList(Screen s) {
    screen_ = s;
    focus_ = 0;
    listFirst_ = 0;
    if (s == Screen::VehicleList) {
        setTitle("TT_TOP_TITLE_GARAGE_VEHICLE");
        const auto list = vehicleChoices();
        for (size_t i = 0; i < list.size(); ++i)
            if (list[i]->name == vehicle_) focus_ = int(i), listFirst_ = int(i / 10) * 10;
    } else if (s == Screen::OptDisplay) {
        setTitle("TT_TOP_TITLE_OPTION_DISPLAY");
        optionsOpen(s);
    } else if (s == Screen::OptAudio) {
        setTitle("TT_TOP_TITLE_OPTION_AUDIO");
        optionsOpen(s);
    } else if (s == Screen::OptGraphics) {  // P101 [port]: FUEL has no such title text: "GRAPHICS" in the title's font
        setTitle("TT_TOP_TITLE_OPTION_DISPLAY");
        if (Page* p = scripts_.page("TopTitle"))
            if (Item* t = p->find("ITEM_ID_TITLE")) t->hasDynText = true, t->dynText = "GRAPHICS";
        optionsOpen(s);
    } else if (s == Screen::OptController) {  // P101: 0x495ca0 case 0x2c
        setTitle("TT_TOP_TITLE_OPTION_CONTROLLER_SETTINGS");
        optionsOpen(s);
    } else if (s == Screen::CampList) {
        setTitle("TT_TOP_TITLE_MAP");
        focus_ = hubIndex_, listFirst_ = (hubIndex_ / 10) * 10;
    } else {
        setTitle("TT_TOP_TITLE_MAP");
    }
}

bool Menu::showPage(const std::string& name) {
    const std::string n = upper(name);
    for (int t = 0; t < TabCount; ++t)
        if (n == upper(kTabPage[t]) || (t == TabMap && n == "TEMPLATELEFTMENU") || n == upper(std::string("TOPMENU_") + kTabPage[t])) {
            if (t == TabLast && mode_ == MenuMode::Main) mode_ = MenuMode::Pause;
            goTab(t);
            return true;
        }
    if (n.rfind("EDITOR", 0) == 0 && n != "EDITOR" && career_) return editorShot(n);  // PK21: the editor's states
    if (n == "TOPMENU") return goTab(TabMap), true;
    if (n == "RACERUNNING" || n == "TUTORIALMODE") {  // PK18: --menu-shot RaceRunning (a paused race: GARAGE off) / TutorialMode (PROFILE off)
        if (n == "TUTORIALMODE") gameMode_ = 2, mode_ = MenuMode::Pause;
        else {
            for (const Event& e : events_)
                if (e.kind == Event::Career) { setSession(e.id, e.hub, vehicle_); break; }
            mode_ = MenuMode::Pause;
        }
        return goTab(TabLast), true;
    }
    if (n == "MAPFOCUS") return goTab(TabMap), map_.focus = true, true;  // P44: the map page with the map focused
    // PK19: --menu-shot MAPZOOM:<zoom>[:x:z] (the map focused, the zoom settled, the cursor at x, z) / MAPSEL:<type>[:zoom] (the
    // cursor on the first map item of that type, e.g. 11 challenge, 13 livery, 7 career, 6 hub).
    if ((n.rfind("MAPZOOM:", 0) == 0 || n.rfind("MAPSEL:", 0) == 0) && career_) {
        goTab(TabMap);
        mapOpen();
        map_.focus = true;
        std::vector<float> a;
        for (size_t at = n.find(':'); at != std::string::npos; at = n.find(':', at + 1)) a.push_back(std::strtof(n.c_str() + at + 1, nullptr));
        float zoom = 5000;
        if (n.rfind("MAPZOOM:", 0) == 0) {
            if (!a.empty()) zoom = a[0];
            if (a.size() >= 3) map_.cx = a[1], map_.cz = a[2];
        } else {
            if (a.size() >= 2) zoom = a[1];
            const int type = a.empty() ? 6 : int(a[0]);
            for (const MapItem& it : mapItems())
                if (it.type == type) { map_.cx = it.x, map_.cz = it.z; break; }
        }
        map_.vx = map_.cx, map_.vz = map_.cz, map_.zoom = map_.zoomTarget = zoom;
        mapCursorHub(false, true);
        mapSelect();
        mapElementInfo();
        return true;
    }
    if (n == "MAP_LOCKED_HUB" && career_) {  // --menu-shot: the map on the first locked hub
        for (size_t i = 0; i < hubs_.size(); ++i) {
            hubIndex_ = int(i);
            if (hubLocked()) break;
        }
        return goTab(TabMap), true;
    }
    if (n.rfind("GARAGEVEHICLES:", 0) == 0) {  // vehicles4 (port): --menu-shot GARAGEVEHICLES:<VARIANT>, the list on that vehicle
        openList(Screen::VehicleList);
        const std::string want = upper(name.substr(15));
        const auto list = vehicleChoices();
        bool found = false;
        for (size_t i = 0; i < list.size(); ++i)
            if (upper(list[i]->name) == want) focus_ = int(i), listFirst_ = int(i / 10) * 10, found = true;
        if (!found)  // still locked in this career: preview it as the current vehicle (the list keeps its focus)
            for (const VehicleInfo& v : vehicles_)
                if (upper(v.name) == want) vehicle_ = v.name;
        return true;
    }
    if (n.rfind("GARAGEUPGRADES", 0) == 0 && career_)  // [upgrades U3] GARAGEUPGRADES[:VEHICLE[:CAT[:STAGE[:KEYS]]]]
        return upgradesShot(name.size() > 15 ? name.substr(15) : std::string());
    if (n == "GARAGEPRICE" && career_) {  // --menu-shot: the garage list on an unowned vehicle
        openList(Screen::VehicleList);
        const auto list = vehicleChoices();
        for (size_t i = 0; i < list.size(); ++i)
            if (!career_->vehicleOwned(list[i]->name)) { focus_ = int(i), listFirst_ = int(i / 10) * 10; break; }
        return true;
    }
    if (n.rfind("OK_", 0) == 0) return goTab(TabMap), openOk(name.substr(3), {}), true;  // P66: --menu-shot OK_<TT key>
    if (n == "HUBDATA") return goTab(TabMap), mapData_ = true, setTitle("TT_TOP_TITLE_HUB_DATA"), true;  // P66: the DATA page
    if (n.rfind("CONFIRM_", 0) == 0) return goTab(TabMap), openConfirm(name.substr(8), {}), true;  // P62: --menu-shot CONFIRM_<TT key>
    if (n.rfind("TABTUTORIAL", 0) == 0 && n.size() > 11) {  // P62: --menu-shot TabTutorial<N>: a tutorial over the Map tab
        goTab(TabMap);
        return showTutorial(std::atoi(n.c_str() + 11));
    }
    if (n == "PRESSSTART" || n == "TITLE") return setMode(MenuMode::Title), true;
    if (n.rfind("MCALERT", 0) == 0 || n == "PADALERT" || n == "INFOSTEXT" || n == "IGMSGBOX") return alertShot(n);  // PK25
    // PK02: --menu-shot HubSelectCareerMood / HubChallengesDescription / HubChallengesWarning (the first race of the hub's
    // list; "Warning" = a challenge whose vehicle list is empty, 0x4813a0).
    if (n == "HUBSELECTCAREERMOOD" || n == "HUBCHALLENGESDESCRIPTION" || n == "HUBCHALLENGESWARNING") {
        if (career_) career_->enterHub(careerHub());
        goTab(TabMap);
        const auto list = hubEvents(n == "HUBSELECTCAREERMOOD" ? Event::Career : Event::Challenge);
        if (list.empty()) return false;
        shotNoVehicle_ = n == "HUBCHALLENGESWARNING";
        openRace(*list.front());
        return true;
    }
    if (n == "HUBSELECTCAREER" || n == "HUBSELECTCHALLENGES") {
        if (career_) career_->enterHub(careerHub());  // in memory only (shots never write career.sav)
        return openList(n == "HUBSELECTCAREER" ? Screen::CareerList : Screen::ChallengeList), true;
    }
    if (n == "CAREERRACEREWARD" || n == "RACE_REWARD" || n == "REWARD") {
        if (career_ && award_.fuelWon == 0) {  // a sample award on a copy, for --menu-shot
            Career copy = *career_;
            const auto& hl = copy.rules().hubs[0].career;
            if (!hl.empty()) sessionEvent_ = copy.rules().missions[size_t(hl[0])].name;
            award_ = copy.finishEvent(sessionEvent_, true, 1, 2, 200.0);
        }
        mode_ = MenuMode::Results, screen_ = Screen::Reward;
        return true;
    }
    // P66: --menu-shot pages of the garage: GARAGELIVERY, GARAGEPAINT, DRIVER, DRIVERHEAD / CHEST / LEGS, DRIVER<PART>TYPE,
    // DRIVER<PART>LIVERY, DRIVERSKIN (FUEL states 0x94, 0x95, 0x3b, 0x3c.., 0x9b.., 0x3f).
    {
        static const std::pair<const char*, int> kGarage[] = {
            {"GARAGELIVERY", 0x94},     {"GARAGEPAINT", 0x95},        {"DRIVER", 0x3b},          {"DRIVERHEAD", 0x3c},
            {"DRIVERCHEST", 0x3d},      {"DRIVERLEGS", 0x3e},         {"DRIVERSKIN", 0x3f},      {"DRIVERHEADTYPE", 0x9b},
            {"DRIVERHEADLIVERY", 0x9c}, {"DRIVERCHESTTYPE", 0x9d},    {"DRIVERCHESTLIVERY", 0x9e}, {"DRIVERLEGSTYPE", 0x9f},
            {"DRIVERLEGSLIVERY", 0xa0}};
        for (const auto& [name, st] : kGarage)
            if (n == name) return goTab(TabGarage), garageEnter(st), true;
    }
    // P66: --menu-shot PROFILEGENERAL<k> (k = 0..6 sub page), PROFILECOLLECTION<k> (0..2), PROFILECOMPLETION<k> (0..1),
    // with an optional ":<scroll>" (lines scrolled).
    for (const auto& [prefix, st] : {std::pair<const char*, int>{"PROFILEGENERAL", 0x16}, {"PROFILECOLLECTION", 0x1e}, {"PROFILECOMPLETION", 0x24}})
        if (n.rfind(prefix, 0) == 0) {
            goTab(TabProfile), profileEnter(st);
            const std::string rest = n.substr(std::strlen(prefix));
            profile_.sel = std::atoi(rest.c_str());
            if (const size_t c = rest.find(':'); c != std::string::npos) profile_.scroll = std::atoi(rest.c_str() + c + 1);
            return true;
        }
    if (n == "HUBSELECTCAMPS") return openList(Screen::CampList), true;
    if (n == "GARAGEVEHICLES" || n == "VEHICLELIST") return openList(Screen::VehicleList), true;
    if (n == "PAGEOPTIONSDISPLAY") return openList(Screen::OptDisplay), true;
    if (n == "PAGEOPTIONSAUDIO") return openList(Screen::OptAudio), true;
    // P101: the GRAPHICS page, the Controller menu (state 0x29) and its settings page (0x2c); ":<row>" focuses a row,
    // ":<row>:<k>" then changes it k times (in memory: shots never save).
    for (const auto& [prefix, sc] : {std::pair<const char*, Screen>{"PAGEOPTIONSGRAPHICS", Screen::OptGraphics},
                                     {"PAGEOPTIONSCONTROLLERSETTINGS", Screen::OptController},
                                     {"PAGEOPTIONSDISPLAY:", Screen::OptDisplay}, {"PAGEOPTIONSAUDIO:", Screen::OptAudio}})
        if (n.rfind(prefix, 0) == 0) {
            goTab(TabOptions), openList(sc);
            if (const size_t c = n.find(':'); c != std::string::npos) {
                focus_ = std::clamp(std::atoi(n.c_str() + c + 1), 0, optionRows(sc) - 1);
                if (const size_t c2 = n.find(':', c + 1); c2 != std::string::npos)  // ":<row>:<k>": k presses of Right on it
                    for (int k = std::atoi(n.c_str() + c2 + 1); k > 0; --k) listPage(1);
            }
            return true;
        }
    if (n == "PAGEOPTIONSCONTROLLER") return goTab(TabOptions), ctrlMenu_ = true, setTitle("TT_TOP_TITLE_OPTION_CONTROLLER"), true;
    if (n.rfind("PAGEOPTIONSCONTROLLERCONFIG", 0) == 0) return remapShot(name.substr(std::min(name.size(), size_t(27)))), true;  // [port] remap
    if (n.rfind("CREDITS", 0) == 0) {  // P75: --menu-shot CREDITS[:seconds]
        goTab(TabOptions), screen_ = Screen::Credits;
        creditsClock_ = n.size() > 8 ? float(std::atof(n.c_str() + 8)) : 2.0f;
        return true;
    }
    if (n.rfind("ENDRACEUNLOCKS", 0) == 0) {  // P75: the unlock chain's camp page with a sample award
        award_ = {};
        award_.hubsUnlocked = {"Islands", "Mountain 3"};
        if (n.size() > 15) {  // P78: ENDRACEUNLOCKS:k = page k of a fuller sample (camps, pilot livery, vista doppler, maverick, bonus car)
            award_.pilotLiveries = {"HEAD 3"};
            award_.trafficRevealed = {"Radar_VistaPoint in Islands", "Vehicle in Islands"};
            award_.vehiclesUnlocked = {"BUGGY_ALL_2"}, award_.giftVehicle = "BUGGY_ALL_2";
        }
        unlockMapN_ = career_ && career_->state().get<int32_t>(0xb4) > 0 ? -1 : 1;  // the map pictures "<n>A" / "<n>B" (n = 0 has none)
        buildUnlocks();
        for (int k = n.size() > 15 ? std::atoi(n.c_str() + 15) : 0; k > 0 && unlocks_.size() > 1; --k) unlocks_.erase(unlocks_.begin());
        mode_ = MenuMode::Results, screen_ = Screen::Unlock;
        return true;
    }
    if (n.rfind("BONUSCODES", 0) == 0) {  // P78: BONUSCODES (list), BONUSCODES:1 (keyboard, "1234"), :2 (that code: wrong)
        goTab(TabOptions);
        const int k = n.size() > 11 ? std::atoi(n.c_str() + 11) : 0;
        bonusOpen(k == 0 ? Screen::BonusCodes : Screen::BonusKeyboard);
        if (k >= 1) bonus_.entered = "1234", bonus_.focus = 4;
        if (k >= 2) {
            bonus_.focus = 10;  // (a right code would be saved: shots never redeem one)
            bonusKey(MenuKey::Accept);
        }
        return true;
    }
    if (n == "FUELREWARD") {  // PK08: EndRaceFuelChallengeReward (state 0x1a) after a sample won challenge
        if (career_) {
            for (const Event& e : events_)
                if (e.kind == Event::Challenge) { sessionEvent_ = e.id; break; }
            award_ = {};
            award_.fuelWon = 1;
        }
        mode_ = MenuMode::Results, screen_ = Screen::FuelReward;
        return true;
    }
    if (n.rfind("BANNER:", 0) == 0) {  // PK08: BANNER:<kind>[:<param>] = MessagesHUD kind 1..12 (param: text id, 8 = vehicle index)
        shotBanner_.kind = std::atoi(n.c_str() + 7);
        if (const size_t c = n.find(':', 7); c != std::string::npos) shotBanner_.param = std::atoi(n.c_str() + c + 1);
        mode_ = MenuMode::Results, screen_ = Screen::Single, singlePage_.clear();
        return true;
    }
    if (n == "WHATNEXT") {  // P78: EndRace_WhatNext2 (state 0x1c)
        mode_ = MenuMode::Results, screen_ = Screen::WhatNext;
        if (!hubs_.empty()) playerX_ = float(hubs_[0].camp.x), playerZ_ = float(hubs_[0].camp.z);
        return true;
    }
    if (n == "ENDRACEWON" || n == "ENDRACELOST") {  // P75: the win / lose page (0x4b7830 / 0x4b82c0), fully faded in
        if (results_.empty()) results_ = {{"QUAD_ROAD_2", 245.31f, true}, {"BIKE_ROAD_1", 247.02f, false}};
        mode_ = MenuMode::Results, screen_ = Screen::EndRace, endRaceClock_ = 1, resultsWon_ = n == "ENDRACEWON";
        return true;
    }
    if (n.rfind("RANKINGSCROLL", 0) == 0) {  // PK08: seven racers, the player 3rd; RANKINGSCROLL:<k> = k presses of Down
        results_ = {{"QUAD_ROAD_2", 245.31f, false}, {"BIKE_ROAD_1", 247.02f, false}, {"BUGGY_ROAD_1", 251.9f, true}, {"MUSCLE_ROAD_1", 255.0f, false},
                    {"SUV_ALL_1", 258.4f, false}, {"BIKE_ROAD_2", 261.0f, false}, {"QUAD_OFF_1", 263.3f, false}};
        if (sessionEvent_.empty()) sessionEvent_ = "CR2-Yell-CP";
        mode_ = MenuMode::Results, screen_ = Screen::Results, rankSel_ = rankRow_ = 0;
        for (int k = n.size() > 14 ? std::atoi(n.c_str() + 14) : 0; k > 0; --k) key(MenuKey::Down);
        return true;
    }
    if (n == "RACE_RANKING_WIN" || n == "RESULTS") {
        if (results_.empty()) results_ = {{"QUAD_ROAD_2", 245.31f, true}, {"BIKE_ROAD_1", 247.02f, false}, {"BUGGY_ROAD_1", 251.9f, false}};
        if (sessionEvent_.empty()) sessionEvent_ = "CR2-Yell-CP";  // P56: a sample event for end_race_infos
        mode_ = MenuMode::Results, screen_ = Screen::Results;
        return true;
    }
    if (n.rfind("PAGETUTORIAL", 0) == 0 && n.size() > 12 && std::isdigit(static_cast<unsigned char>(n[12]))) {  // P56: PageTutorial<N>
        screen_ = Screen::Single, singlePage_.clear();
        return showTutorial(std::atoi(n.c_str() + 12));
    }
    if (scripts_.page(name)) {
        screen_ = Screen::Single, singlePage_ = name;
        return true;
    }
    return false;
}

MenuRequest Menu::take() {
    MenuRequest r = request_;
    request_ = {};
    // P78: 0x47be40 on menu close (0x46eea0) [O]: v = game+0x33c8 (the garage's vehicle), R = its record; when the
    // player's vehicle index (+0x50c) differs: rebuild the vehicle (0x429fc0) + livery R+0xc (0x42a270) + colour R+0xd
    // (0x42a110); else a colour change (+0x510) or a livery change (+0x84) is applied alone; then the meter HUD refresh.
    // [port] the free-ride session restarts where the player is with the garage's choice (vehicle, livery, colour are
    // session arguments); [I] races keep their vehicle.
    if (r.kind == MenuRequest::Resume && mode_ == MenuMode::Pause && sessionEvent_.empty() && career_ && !sessionVehicle_.empty()) {
        // [upgrades U3] stages bought / equipped (or upgrades switched) in the pause menu's garage count as a vehicle change
        // (plan 4.6: the free-ride session restarts with them; FUEL has no upgrades).
        const bool changed = upper(vehicle_) != upper(sessionVehicle_) || career_->vehicleLivery(vehicle_) != sessionLivery_ ||
                             career_->vehicleColor(vehicle_) != sessionColor_ || !(career_->sessionUpgrades(vehicle_, port_.upgrades) == sessionStages_);
        if (changed) {
            const std::string hub = sessionHub_.empty() && !hubs_.empty() ? hubs_[size_t(hubIndex_)].id : sessionHub_;
            r = {MenuRequest::FreeRide, "", hub, vehicle_};
            r.x = playerX_, r.z = playerZ_, r.at = true;
        }
    }
    return r;
}

void Menu::saveOptions() {
    if (!saveSettings(settings_)) std::fprintf(stderr, "menu: cannot save %s\n", settingsPath().c_str());
    saveMenuOptions(options_);
    savePortOptions(port_, settingsPath());  // P101
    if (bindLinesSet_ && !saveBindingLines(bindLines_, settingsPath()))  // [port] remap: key.* / pad.* (port_options.cpp)
        std::fprintf(stderr, "menu: cannot save the bindings in %s\n", settingsPath().c_str());
}

void Menu::back() {
    switch (screen_) {
        case Screen::Title: break;
        case Screen::Tab:
            if (tab_ == TabGarage && !garage_.page.empty()) garageBack();  // P66: Driver menus
            else if (tab_ == TabMap && mapData_) mapData_ = false, focus_ = 0, setTitle("TT_TOP_TITLE_MAP");  // P66: 0x47e4c0 -> 0x53
            else if (tab_ == TabOptions && ctrlMenu_) goTab(TabOptions);  // P101: 0x492950 Back -> 0x46f080, state 6
            else if (mode_ == MenuMode::Pause) request_.kind = MenuRequest::Resume;
            break;
        case Screen::GarageSelect: garageBack(); break;  // P66
        case Screen::GarageUpgrades: garageBack(); break;  // [upgrades U3]
        case Screen::Profile: profileEnter(3); break;      // P66
        case Screen::CareerList:
        case Screen::ChallengeList:
        case Screen::CampList: goTab(TabMap); break;
        case Screen::RacePage: raceKey(MenuKey::Back); break;  // PK02
        case Screen::VehicleList:
            if (!pendingEvent_.empty()) {
                // PK02: 0x47f120 / 0x47f200 (garage keys 0x478bb0 Back): state 0x56 / 0x59, the race page again.
                if (const Event* e = eventById(pendingEvent_); e) {
                    openRace(*e, raceFromArch_);
                    break;
                }
                pendingEvent_.clear();
                goTab(TabMap);
            } else {
                garageReturn("ITEM_ID_TOPMENU_GARAGE_VEHICLE", false);  // PK20: 0x4791b0 Back keeps the list's focus (was goTab)
            }
            break;
        case Screen::OptRemap: remapLeave(); break;  // [port] remap: 0x4c9190 -> 0x4ccfd0 (keep, save, state 0x29)
        case Screen::OptController:
            // P101: 0x492670 Back -> state 0xd5, the same popup (0x495ca0): Yes 0x492f00 = restore (0x495c40) + the
            // Controller menu (state 0x29), No 0x496100 = stay (state 0x2c).
            openConfirm("TT_AUDIO_DISPLAY_CONFIRMATION", [this] { optionsLeave(false); });
            break;
        case Screen::OptGraphics:
        case Screen::OptDisplay:
        case Screen::OptAudio:
            // 0x492b70 / 0x492f20: Back -> state 0xd3 / 0xd4, the popup TT_AUDIO_DISPLAY_CONFIRMATION ("Are you sure? Any
            // unsaved changes will be lost."): Yes 0x492ae0 / 0x492eb0 restores the values and returns to the options menu,
            // No 0x4960c0 / 0x4960e0 stays on the page.
            openConfirm("TT_AUDIO_DISPLAY_CONFIRMATION", [this] { optionsLeave(false); });
            break;
        case Screen::Results:
            // race_reward (CareerRaceReward.tsc) after a counted career race (0x4b4220), then the race menu.
            buildUnlocks();
            if (award_.counted && career_ && career_->status(sessionEvent_)) {
                int list = 0;
                career_->locate(sessionEvent_, nullptr, &list, nullptr);
                if (list == 1) { screen_ = Screen::Reward; break; }
            }
            if (nextUnlock()) break;
            endChain();
            break;
        case Screen::Reward:
        case Screen::Unlock:
            if (nextUnlock()) break;
            endChain();
            break;
        case Screen::FuelReward: break;  // 0x1a: the action key only
        case Screen::WhatNext: break;  // 0x1c has no back key (and PAUSE is refused while +0xd6c is 6..0x1c, 0x46e750)
        case Screen::Single: break;
    }
}

void Menu::accept(const std::string& id) {
    if (tab_ == TabGarage && garageAccept(id)) return;  // P66: livery, paintwork, driver pages
    if (tab_ == TabProfile && profileAccept(id)) return;  // P66: the Profile tab's pages
    if (tab_ == TabEditor && editorAccept(id)) return;    // PK21: 0x4761d0 -> 0x475650(1)
    if (id == "ITEM_ID_TOPMENU_RACE_CONTINUE") request_.kind = MenuRequest::Resume;
    else if (id == "ITEM_ID_TOPMENU_RACE_RESTART" || id == "ITEM_ID_TOPMENU_RACE_GO_FREE_RIDE") {
        MenuRequest r = {MenuRequest::Restart};
        if (id == "ITEM_ID_TOPMENU_RACE_GO_FREE_RIDE")
            r = {MenuRequest::FreeRide, "", sessionHub_.empty() && !hubs_.empty() ? hubs_[size_t(hubIndex_)].id : sessionHub_, vehicle_},
            r.noArrival = true;
        // P62: 0x49ade0: items 1 and 2 go to state 0xae, the popup TT_PAGE_RACE_YES_NO ("Your progression in the current
        // event will be lost...") while a mission runs (0x49b500 -> 0x421f60); Yes 0x4ceaa0 runs the item, No 0x4ceab0 stays.
        if (!sessionEvent_.empty() && results_.empty()) openConfirm("TT_PAGE_RACE_YES_NO", [this, r] { request_ = r; });
        else request_ = r;
    } else if (id == "ITEM_ID_LEFT_MENU5" || id == "ITEM_ID_LEFT_MENU4") {
        if (hubLocked()) return;  // Map_locked_hub: stars still needed
        if (career_ && career_->enterHub(careerHub())) career_->setCurrentHub(careerHub()), markDirty();  // 0x481cd0, 0x481d70 (+0x33c0, 0x56ca20); PK17: +0x80 (0x481d43)
        openList(id == "ITEM_ID_LEFT_MENU5" ? Screen::CareerList : Screen::ChallengeList);
    }
    else if (id == "ITEM_ID_LEFT_MENU3") openList(Screen::CampList);
    else if (id == "ITEM_ID_LEFT_MENU2") mapData_ = true, focus_ = 0, setTitle("TT_TOP_TITLE_HUB_DATA");  // P66: DATA -> HubData (0x5b)
    // P66: HubData (0x47e4c0): LOAD -> 0xb5 TT_LOAD_GAME_SHOW_WARNING (Yes 0x47d6d0 load, No 0x47d5e0 -> 0x5b);
    // SAVE (0x47d760) -> 0xbe TT_AUTO_SAVE_OVERWRITE (Yes 0x4ceaf0 save, No 0x4ceb50 -> 0x53).
    else if (id == "ITEM_ID_HUB_DATA_LOAD") openConfirm("TT_LOAD_GAME_SHOW_WARNING", [this] { dataLoad(); });
    else if (id == "ITEM_ID_HUB_DATA_SAVE") openConfirm("TT_AUTO_SAVE_OVERWRITE", [this] { dataSave(); }, [this] { mapData_ = false, focus_ = 0, setTitle("TT_TOP_TITLE_MAP"); });
    else if (id == "ITEM_ID_LEFT_MENU1") {
        // P62: 0x47cc70: TT_ARE_YOU_SURE_QUIT ("Are you sure you want to exit the game?"), Yes 0x4ce7d0, No 0x4ce7b0 -> state 0x53.
        const MenuRequest::Kind q = mode_ == MenuMode::Pause ? MenuRequest::QuitToTitle : MenuRequest::QuitGame;
        openConfirm("TT_ARE_YOU_SURE_QUIT", [this, q] { request_.kind = q; });
    }
    else if (id == "ITEM_ID_TOPMENU_GARAGE_VEHICLE") openList(Screen::VehicleList);
    else if (id == "ITEM_ID_TOPMENU_GARAGE_START_RACE") {
        if (!pendingEvent_.empty()) requestStart({MenuRequest::StartEvent, pendingEvent_, "", vehicle_});  // P62: 0x4786e0
        else if (!hubs_.empty()) requestStart({MenuRequest::FreeRide, "", hubs_[size_t(hubIndex_)].id, vehicle_});
    } else if (id == "ITEM_ID_TOPMENU_OPTIONS_DISPLAY") openList(Screen::OptDisplay);
    else if (id == "ITEM_ID_TOPMENU_OPTIONS_GRAPHICS") openList(Screen::OptGraphics);  // P101 [port]
    // P101: Options > CONTROLLER -> state 0x29 (left menu PageOptionsController, 0x495ca0); SETTINGS (0x491910 case 1) ->
    // +0x450 = +0x454 = 0, the snapshot 0x4946c0, state 0x2c. LAYOUT (case 0: the remap page 0x4cc340): [port] remap, un-parked
    // at the user's request (remap_page.mm; FUEL opens it only without a pad, else its pad picture page 0xad).
    else if (id == "ITEM_ID_TOPMENU_OPTIONS_CONTROLLER") ctrlMenu_ = true, focus_ = 0, setTitle("TT_TOP_TITLE_OPTION_CONTROLLER");
    else if (id == "ITEM_ID_CONTROLLER_MENU_OPTIONS_SETTINGS") openList(Screen::OptController);
    else if (id == "ITEM_ID_CONTROLLER_MENU_OPTIONS_CONFIGURATION") remapOpen();  // [port] remap: 0x491910 case 0 -> 0x4cc340
    else if (id == "ITEM_ID_TOPMENU_OPTIONS_AUDIO") openList(Screen::OptAudio);
    else if (id == "ITEM_ID_TOPMENU_OPTIONS_CREDITS") screen_ = Screen::Credits, creditsClock_ = 0;  // P75: state 0x36
    else if (id == "ITEM_ID_TOPMENU_OPTIONS_BONUS_CODES") bonusOpen(Screen::BonusCodes);  // P78: 0x491de0 -> state 0x33
}

// PK11 0x481580: a camp row is shown when its hub is unlocked (0x579720).
bool Menu::campShown(int idx) const {
    if (idx < 0 || idx >= int(hubs_.size())) return false;
    const int ch = career_ ? career_->rules().hubIndex(hubs_[size_t(idx)].id) : -1;
    return !(ch >= 0 && !career_->hubUnlocked(ch));
}

void Menu::listMove(int d) {
    int count = 0, rows = 1;
    switch (screen_) {
        case Screen::CareerList: count = int(hubEvents(Event::Career).size()), rows = 3; break;
        case Screen::ChallengeList: count = int(hubEvents(Event::Challenge).size()), rows = 5; break;
        case Screen::CampList: count = int(hubs_.size()), rows = 10; break;
        case Screen::VehicleList: count = int(vehicleChoices().size()), rows = 10; break;
        case Screen::OptDisplay:
        case Screen::OptAudio:
        case Screen::OptGraphics:
        case Screen::OptController: count = rows = optionRows(screen_); break;  // P101
        default: return;
    }
    if (count == 0) return;
    // P62: the options pages stop at their first and last rows (0x492b70 / 0x492f20: +0x414 / +0x41c only move within).
    // P75: the garage carousel stops at its ends too (0x4791b0: Left if +0xc8 > 0, Right if < count - 1). The race lists
    // (0x47dfc0 / 0x47e9f0) move Up/Down only inside the page: sel % R > 0 / sel % R < min(N, R) - 1 [O quirk: the
    // page-1 bound also on page 2].
    // P101: the controller settings page too (0x492670: Up while row > +0x418, Down while row < 2; port rows below).
    if (screen_ == Screen::OptDisplay || screen_ == Screen::OptAudio || screen_ == Screen::OptGraphics || screen_ == Screen::OptController ||
        screen_ == Screen::VehicleList)
        focus_ = std::clamp(focus_ + d, 0, count - 1);
    else if (screen_ == Screen::CareerList || screen_ == Screen::ChallengeList) {
        const int r = focus_ % rows;
        if ((d < 0 && r > 0) || (d > 0 && r < std::min(count, rows) - 1 && focus_ + 1 < count)) focus_ += d;
    } else if (screen_ == Screen::CampList) {
        // PK11 0x47e610: Up while sel % 10 > 0, Down while sel % 10 < (shown BARRE_COLOR rows, 0x60f4a0) - 1 (the rows
        // are counted from the top until the first hidden one, i.e. the first locked camp); no wrap, no page change.
        int shown = 0;
        for (int k = 0; k < rows && (focus_ / rows) * rows + k < count && campShown((focus_ / rows) * rows + k); ++k) ++shown;
        const int r = focus_ % rows;
        if ((d < 0 && r > 0) || (d > 0 && r < shown - 1)) focus_ += d;
    } else focus_ = (focus_ + d + count) % count;
    listFirst_ = (focus_ / rows) * rows;
}

void Menu::listPage(int d) {
    if (screen_ == Screen::OptGraphics || screen_ == Screen::OptController || (screen_ == Screen::OptAudio && focus_ >= 2)) {
        optionChange(d);  // P101 (options_pages.mm)
        return;
    }
    if (screen_ == Screen::OptDisplay || screen_ == Screen::OptAudio) {
        ++optSerial_;  // P101: applied live by the session
        // Left/Right change the focused option.
        auto flip = [&](bool& b) { b = !b; };
        if (screen_ == Screen::OptAudio) {
            // 0x495650 -> 0x5540e0 (SFX, game+0x258) / 0x554040 (music, +0x254): integer volume 0..16 += 1, sound
            // manager volume = n / 16.
            float& v = focus_ == 0 ? settings_.volume : options_.musicVolume;
            v = float(std::clamp(int(std::lround(v * 16.0f)) + d, 0, 16)) / 16.0f;
            return;
        }
        switch (focus_) {
            case 0: flip(options_.raceInfo); break;
            case 1: flip(options_.rankingInfo); break;
            case 2: flip(options_.gps); break;
            case 3: flip(options_.speedMeter); break;
            case 4: flip(options_.miles); break;
            // 0x495390 case 5 -> 0x495540: slider p += (+-4 px) / bar width, clamped to 0..1; brightness 0.5 + 1.5 p.
            case 5: options_.brightness = std::clamp(options_.brightness + 4.0f * float(d) / 313.0f, 0.0f, 1.0f); break;
            // P101: post effects, grass and full screen moved to the GRAPHICS page (FUEL's six rows stay here).
        }
        return;
    }
    const int rows = screen_ == Screen::CareerList ? 3 : screen_ == Screen::ChallengeList ? 5 : 10;
    int count = 0;
    if (screen_ == Screen::CareerList) count = int(hubEvents(Event::Career).size());
    if (screen_ == Screen::ChallengeList) count = int(hubEvents(Event::Challenge).size());
    if (screen_ == Screen::CampList) count = int(hubs_.size());
    if (screen_ == Screen::VehicleList) count = int(vehicleChoices().size());
    if (count <= rows) return;
    if (screen_ == Screen::CareerList || screen_ == Screen::ChallengeList || screen_ == Screen::CampList) {
        // P75 0x47dfc0 / 0x47e9f0 (and 0x481580's two camp pages): Left -> row 0 when sel >= R, Right -> row R when
        // sel < R (at most two pages, the first row of the page).
        if (d < 0 && focus_ >= rows) focus_ = 0;
        if (d > 0 && focus_ < rows) focus_ = rows;
        listFirst_ = (focus_ / rows) * rows;
        return;
    }
    const int pages = (count + rows - 1) / rows;
    const int page = ((listFirst_ / rows) + d + pages) % pages;
    listFirst_ = page * rows;
    focus_ = std::min(count - 1, listFirst_ + (focus_ % rows));
}

void Menu::key(MenuKey k) {
    if (dialogs_ && dialogs_->mcAlert().shown()) {  // PK25: MCAlert has the action keys (game+0x208 / +0x210)
        dialogs_->mcAlert().key(k == MenuKey::Accept, k == MenuKey::Back);
        return;
    }
    // PK18 0x4c9190 / 0x4c9220 / 0x4c9330 / 0x4c93c0 / 0x4c9450 / 0x4c94e0 / 0x4c92b0 [O]: the page's key tests OR the sound request
    // into the pause HUD (+0x90): 1 Back, 2 Accept, 4 Up / Down, 0x10 Left / tab left, 0x20 Right / tab right, 0x40 MENU_VALI2
    // (0x238; the screens that read it); 0x46d380 plays one sound a frame. The title screen is not the pause HUD, and the
    // confirmation popup has no Back test (0x4cdfa0).
    if (screen_ != Screen::Title && k < MenuKey::MapAction) {
        switch (k) {
            case MenuKey::Back: if (!confirm_.open && !tutorialOpen()) soundBit(1); break;
            case MenuKey::Accept: soundBit(2); break;
            case MenuKey::Up: case MenuKey::Down: soundBit(4); break;
            case MenuKey::Left: case MenuKey::TabLeft: soundBit(0x10); break;
            case MenuKey::Right: case MenuKey::TabRight: soundBit(0x20); break;
            case MenuKey::Vali2: if (screen_ == Screen::BonusKeyboard) soundBit(bonus_.entered.empty() ? 0x80 : 0x40); break;  // 0x493320: 0x4937f0 = a character was removed
            default: break;
        }
    }
    if (tutorialOpen()) return tutorialKey(k);  // P62: the tutorial page has the keys over the menus too
    if (confirm_.open) return confirmKey(k);     // P62: the confirmation popup
    if (screen_ == Screen::OptRemap) return remapKey(k);  // [port] remap (remap_page.mm)
    if (k == MenuKey::Reset) return;                     // [port] remap: MENU_RESET is the remap page's only
    if (screen_ == Screen::Tab && tab_ == TabEditor && ed_.state != 5) {  // PK21: 0x475fa0 (tabs still switch from the races list)
        const bool list = ed_.state >= 0xa1 && ed_.state <= 0xa4;
        if (!(list && (k == MenuKey::TabLeft || k == MenuKey::TabRight))) return editorKey(k);
    }
    if (k >= MenuKey::MapAction) {  // P44: the map page's own keys
        if (screen_ == Screen::Tab && tab_ == TabMap && infoBox_.empty()) mapKey(k);
        return;
    }
    if (screen_ == Screen::Tab && tab_ == TabMap && map_.focus) {  // [port] map focus: Accept = action, Back = left menu
        if (k == MenuKey::Accept) return mapKey(MenuKey::MapAction);
        if (k == MenuKey::Back) {
            map_.focus = false;
            return;
        }
        if (k == MenuKey::Up || k == MenuKey::Down || k == MenuKey::Left || k == MenuKey::Right) {
            mapKey(k == MenuKey::Up ? MenuKey::MapUp : k == MenuKey::Down ? MenuKey::MapDown : k == MenuKey::Left ? MenuKey::MapLeft : MenuKey::MapRight);
            return;
        }
    }
    if (screen_ == Screen::Title) {
        if (k == MenuKey::Accept) setMode(MenuMode::Main);
        else if (k == MenuKey::Back) request_.kind = MenuRequest::QuitGame;
        return;
    }
    if (screen_ == Screen::BonusCodes || screen_ == Screen::BonusKeyboard || screen_ == Screen::BonusResult) return bonusKey(k);  // P78
    if (screen_ == Screen::Credits) {  // P75 0x4934f0: the action key (or a click) -> the options menu (0x46f080, state 6)
        if (k == MenuKey::Accept) goTab(TabOptions);
        return;
    }
    if (screen_ == Screen::EndRace) {  // P75: 0x4ba590 states 6 / 7: the action key -> ranking 0x11 / 0x12
        if (k != MenuKey::Accept) return;
        if (resultsList_ == 3) return endChain();  // PK21: 0x4ba590 states 0xa / 0xb -> 0x1c (EndRace_WhatNext2)
        // PK08 state 8 (a challenge won): mission +0x50 ? 0x1a (page 121) : 0x1c; state 9 (lost): 0x1c. No ranking.
        if (challengeEnd_) {
            if (resultsWon_ && fuelChallenge_) screen_ = Screen::FuelReward;
            else endChain();
            return;
        }
        screen_ = Screen::Results, rankSel_ = rankRow_ = 0;  // 0x4b2b20: +0xe70 = +0xe6c = 0
        return;
    }
    if (screen_ == Screen::FuelReward) {  // 0x4ba590 state 0x1a: the action key -> 0x1c
        if (k == MenuKey::Accept) endChain();
        return;
    }
    if (screen_ == Screen::Results && (k == MenuKey::Up || k == MenuKey::Down)) {
        // 0x4b1aa0 (offline): UP (+0xe70 > 0: -1) and the row -1; DOWN (+0xe70 < count - 1: +1) and the row +1; 0x4b1c50 clamps.
        const int count = int(results_.size());
        rankClamp();  // the draw's 0x4b1c50 has run
        if (k == MenuKey::Up) {
            if (rankSel_ > 0) --rankSel_;
            --rankRow_;
        } else {
            if (rankSel_ < count - 1) ++rankSel_;
            ++rankRow_;
        }
        rankClamp();
        return;
    }
    if (screen_ == Screen::Results || screen_ == Screen::Reward || screen_ == Screen::Unlock) {
        if (k == MenuKey::Accept || k == MenuKey::Back) back();
        return;
    }
    if (screen_ == Screen::WhatNext) {
        // 0x4ba590 state 0x1c [O]: MENU_VALID -> "KillMission 1" (the race ends, free ride goes on where the player is);
        // MENU_CANCEL (+0x20c) -> TeleportToHub; MENU_VALI2 (+0x238) -> TTMN "<mission>" (back to the start: retry).
        // [port] the session is a race process: free ride restarts as a free-ride session at the player's place
        // (--at); E is both the port's next-tab key and FUEL's PC MENU_CANCEL.
        const int h = whatNextHub();
        const std::string hub = h >= 0 ? hubs_[size_t(h)].id : sessionHub_.empty() && !hubs_.empty() ? hubs_[size_t(hubIndex_)].id : sessionHub_;
        if (k == MenuKey::Accept) {
            request_ = {MenuRequest::FreeRide, "", hub, vehicle_};
            request_.x = playerX_, request_.z = playerZ_, request_.at = true;
        } else if (k == MenuKey::TabRight) {
            request_ = {MenuRequest::FreeRide, "", hub, vehicle_};
        } else if (k == MenuKey::Vali2) {
            request_ = {MenuRequest::Restart};
        }
        return;
    }
    if (!infoBox_.empty()) {
        if (k == MenuKey::Accept && infoBox_ == "InfoBoxGarageYesNo" && career_ && upg_.buyCat >= 0) {
            // [upgrades U3] the UPGRADES page's Yes: U2's buy (it equips the stage; the missing lower stages are bought with
            // it), saved as the vehicle purchase below is.
            const Career::UpgradeResult res = career_->buyUpgrade(vehicle_, upg_.buyCat, upg_.buyStage);
            if (res == Career::UpgradeResult::Bought) {
                std::printf("career: bought %s for %s, stages %s, %u fuel left\n", upgradeStageName(upg_.buyCat, upg_.buyStage).c_str(), vehicle_.c_str(),
                            upg::formatStages(career_->upgradesEquipped(vehicle_)).c_str(), career_->state().fuel);
                markDirty();
            } else {
                std::printf("career: %s %s not bought (%s)\n", vehicle_.c_str(), upgradeStageName(upg_.buyCat, upg_.buyStage).c_str(), Career::upgradeResultName(res));
            }
        } else if (k == MenuKey::Accept && infoBox_ == "InfoBoxGarageYesNo" && career_) {
            if (career_->buyVehicle(buyTarget_) == Career::Bought) {
                std::printf("career: bought %s, %u fuel left\n", buyTarget_.c_str(), career_->state().fuel);
                markDirty();  // PK17: 0x4ce590 +0x80 = 1 (saved when the pause menu closes)
            }
        }
        if (k == MenuKey::Accept || k == MenuKey::Back) infoBox_.clear(), buyTarget_.clear(), infoText_.clear(), upg_.buyCat = upg_.buyStage = -1;
        return;
    }
    if (screen_ == Screen::RacePage && k != MenuKey::TabLeft && k != MenuKey::TabRight) return raceKey(k);  // PK02
    if (screen_ == Screen::GarageSelect && k != MenuKey::TabLeft && k != MenuKey::TabRight) return garageKey(k);  // P66
    if (screen_ == Screen::GarageUpgrades && k != MenuKey::TabLeft && k != MenuKey::TabRight) return upgradesKey(k);  // [upgrades U3]
    if (screen_ == Screen::Profile && k != MenuKey::TabLeft && k != MenuKey::TabRight) return profileKey(k);       // P66
    if (k == MenuKey::Back) return back();
    if (k == MenuKey::TabLeft || k == MenuKey::TabRight) {
        if (screen_ != Screen::Tab) back();
        const int d = k == MenuKey::TabLeft ? -1 : 1;
        for (int t = tab_ + d; t >= 0 && t < TabCount; t += d)
            if (tabAvailable(t)) return goTab(t);
        return;
    }
    if (screen_ == Screen::Tab) {
        const auto f = focusables();
        if (k == MenuKey::Up && !f.empty()) focus_ = (focus_ + int(f.size()) - 1) % int(f.size());
        if (k == MenuKey::Down && !f.empty()) focus_ = (focus_ + 1) % int(f.size());
        if ((k == MenuKey::Left || k == MenuKey::Right) && tab_ == TabMap && !hubs_.empty()) {
            hubIndex_ = (hubIndex_ + (k == MenuKey::Left ? -1 : 1) + int(hubs_.size())) % int(hubs_.size());
            map_.cx = float(hubs_[size_t(hubIndex_)].camp.x), map_.cz = float(hubs_[size_t(hubIndex_)].camp.z);  // port: the map follows
            // PK19 0x483810: the cursor on the hub's camp (+0x1f0), +0x2d4 = that hub, its picture (0x483750) and stats.
            map_.hub = careerHub();
            mapHubPicture();
            // PK19 end
        }
        if (k == MenuKey::Accept && focus_ < int(f.size())) accept(f[size_t(focus_)]->id);
        return;
    }
    if (screen_ == Screen::Single) return;
    if (k == MenuKey::Up) listMove(-1);
    if (k == MenuKey::Down) listMove(1);
    if (screen_ == Screen::VehicleList && (k == MenuKey::Left || k == MenuKey::Right)) listMove(k == MenuKey::Left ? -1 : 1);  // P62: the carousel
    else if (k == MenuKey::Left) listPage(-1);
    else if (k == MenuKey::Right) listPage(1);
    if (k != MenuKey::Accept) return;
    switch (screen_) {
        case Screen::CareerList:
        case Screen::ChallengeList: {
            const auto list = hubEvents(screen_ == Screen::CareerList ? Event::Career : Event::Challenge);
            if (focus_ < int(list.size()) && list[size_t(focus_)]->playable) {
                // PK02: 0x47d550 / 0x47d660 (+0x944 = 0x56 / 0x59): the race page comes before the vehicle pick.
                pendingEvent_ = list[size_t(focus_)]->id;
                openRace(*list[size_t(focus_)]);
            }
            break;
        }
        case Screen::CampList:
            if (focus_ < int(hubs_.size())) {
                // P62: 0x47cc70: TT_ARE_YOU_SURE_YOU_WANT_TO_GO_THERE, Yes 0x47e3a0 (TeleportToHub), No 0x47db90 -> the list.
                const int h = focus_;
                if (career_) {  // P75: locked camps are not listed (0x481580) [port: nothing happens on their hidden rows]
                    const int ch = career_->rules().hubIndex(hubs_[size_t(h)].id);
                    if (ch >= 0 && !career_->hubUnlocked(ch)) break;
                }
                // P66: 0x47e460: the camp the player is in -> 0xc8 TT_HUB_ALREADY_IN_CAMP (OK 0x47db70 -> the list, 0x5a).
                if (!sessionHub_.empty() && upper(hubs_[size_t(h)].id) == sessionHub_ && sessionEvent_.empty()) {
                    openOk("TT_HUB_ALREADY_IN_CAMP", {});
                    break;
                }
                openConfirm("TT_ARE_YOU_SURE_YOU_WANT_TO_GO_THERE", [this, h] {
                    hubIndex_ = h;
                    request_ = {MenuRequest::FreeRide, "", hubs_[size_t(h)].id, vehicle_};
                });
            }
            break;
        case Screen::VehicleList: {
            const auto list = vehicleChoices();
            if (focus_ < int(list.size())) vehicleAccept(list[size_t(focus_)]->name);
            break;
        }
        case Screen::OptGraphics:
        case Screen::OptController:  // P101: 0x492670: the action key -> +0x80 = 1, state 0x29
        case Screen::OptDisplay:
        case Screen::OptAudio: optionsLeave(true); break;  // P62: the action key keeps the values (0x492b70: +0x80 = 1, state 6)
        default: break;
    }
}

void Menu::pointer(float x, float y, bool click) {
    pointerX_ = x, pointerY_ = y;
    if (alertUp()) return;  // PK25: the menus wait under an alert (0x444400)
    if (tutorialOpen()) {  // P62: a click is the OK button
        if (click) tutorialKey(MenuKey::Accept);
        return;
    }
    if (confirm_.open && confirm_.okOnly) {  // P66: the OK popup: a click is OK
        if (click) confirmKey(MenuKey::Accept);
        return;
    }
    if (confirm_.open) {  // P62: the popup's Yes / No buttons
        if (Page* yn = scripts_.page("PopUpInput_YesNo")) {
            const float dy = scripts_.page("PopUpHelp") && scripts_.page("PopUpHelp")->find("ITEM_ID_POPUPHELP_BOX") ? scripts_.page("PopUpHelp")->find("ITEM_ID_POPUPHELP_BOX")->y : 396.0f;
            for (int b = 0; b < 2; ++b)
                if (const Item* it = yn->find(b == 0 ? "ITEM_ID_YES_LABEL" : "ITEM_ID_NO_LABEL"))
                    if (x >= it->x && x < it->x + it->w && y >= it->y + dy && y < it->y + dy + it->h) {
                        confirm_.focus = b;
                        if (click) confirmKey(MenuKey::Accept);
                    }
        }
        return;
    }
    auto inside = [&](const Item& it) { return x >= it.x && x < it.x + it.w && y >= it.y && y < it.y + it.h; };
    if (screen_ == Screen::GarageUpgrades && !infoBox_.empty()) return void(upgradesPointer(x, y, click));  // [upgrades U3]: Yes / No halves
    if (screen_ == Screen::Title || screen_ == Screen::Results || screen_ == Screen::Reward || screen_ == Screen::EndRace || screen_ == Screen::Unlock ||
        screen_ == Screen::WhatNext || screen_ == Screen::FuelReward || !infoBox_.empty()) {
        if (click) key(MenuKey::Accept);
        return;
    }
    if (Page* top = scripts_.page("TopMenu"))
        for (int t = 0; t < TabCount; ++t)
            if (const Item* icon = top->find(kTabIcon[t]);
                icon && x >= icon->x - icon->w * float(icon->a7) * 0.5f && x < icon->x + icon->w * (1 - float(icon->a7) * 0.5f) &&
                y >= icon->y - icon->h * float(icon->a8) * 0.5f && y < icon->y + icon->h * (1 - float(icon->a8) * 0.5f)) {  // PK18: images are anchored (0x612020)
                if (click && tabAvailable(t)) soundBit(2), goTab(t);  // 0x46fd60: a click on a tab -> +0x90 |= 2
                return;
            }
    if (screen_ == Screen::GarageUpgrades) return void(upgradesPointer(x, y, click));  // [upgrades U3]
    if (screen_ == Screen::Tab) {
        const auto f = focusables();
        for (size_t i = 0; i < f.size(); ++i)
            if (inside(*f[i]) || (x >= f[i]->x && x < f[i]->x + 420 && y >= f[i]->y && y < f[i]->y + f[i]->h)) {
                if (focus_ != int(i)) soundBit(4);  // 0x46c510: the pointer moved the selection (page bits 0x100 / 0x200) -> +0x90 |= 4 / 8
                focus_ = int(i);
                if (click) soundBit(2), accept(f[i]->id);
                return;
            }
        if (click && tab_ == TabMap)  // P44 [port]: a click on the map moves the cursor there
            if (Page* mc = scripts_.page("Map_Cache"))
                if (const Item* c = mc->find("ITEM_ID_CLIPPING_RECT"); c && inside(*c)) {
                    // PK19: the anchored rect and the view centre +0x1b28 (as drawMap); the hub under the new cursor (0x4838d0).
                    float rx = 0, ry = 0;
                    itemRect(*c, rx, ry);
                    const hud::MapView v = hud::mapView(rx, ry, c->w, c->h, map_.zoom, map_.vx, map_.vz, map_.hx, map_.hz);
                    const float dx = (x - v.cx) / v.ppm, dy = (y - v.cy) / v.ppm;
                    map_.cx = map_.vx + dx * map_.hx - dy * map_.hz, map_.cz = map_.vz + dx * map_.hz + dy * map_.hx;
                    mapCursorHub(false, true);
                    map_.focus = true;
                    return;
                }
        if (click && tab_ == TabMap && !hubs_.empty())
            if (Page* m = scripts_.page("Map")) {
                const Item* l = m->find("ITEM_ID_FLECHE_GAUCHE");
                const Item* r = m->find("ITEM_ID_FLECHE_DROITE");
                if (l && inside(*l)) key(MenuKey::Left);
                if (r && inside(*r)) key(MenuKey::Right);
            }
        return;
    }
    if (screen_ == Screen::GarageSelect) {  // P66 [port]: a click on a side moves the carousel, on the middle accepts
        if (!click || y < 440 || y > 640) return;
        const int d = int(std::lround((x - 643.0f) / 110.0f));
        if (d == 0) garageKey(MenuKey::Accept);
        else garageKey(d < 0 ? MenuKey::Left : MenuKey::Right);
        return;
    }
    if (screen_ == Screen::VehicleList) {  // P62: the carousel's arrows and slots
        if (!click) return;
        if (Page* gc = scripts_.page("GearSelecteurCache")) {
            const Item* l = gc->find("ITEM_ID_FLECHE_GAUCHE");
            const Item* rr = gc->find("ITEM_ID_FLECHE_DROITE");
            if (l && x < l->x + l->w + 20 && y > 465 && y < 620) return listMove(-1);
            if (rr && x > rr->x - 20 && y > 465 && y < 620) return listMove(1);
        }
        if (y > 465 && y < 620) {
            const int d = int(std::lround((x - 643.0f) / 115.2f));
            if (d == 0) key(MenuKey::Accept);
            else listMove(d);
        }
        return;
    }
    if (screen_ == Screen::OptRemap) return void(remapPointer(x, y, click));  // [port] remap
    if ((screen_ == Screen::OptGraphics || screen_ == Screen::OptController || screen_ == Screen::OptAudio) && optionsPointer(x, y, click)) return;  // P101
    if (screen_ == Screen::OptGraphics || screen_ == Screen::OptController) return;  // pad-ux: no list page under these (was HubSelectCamps)
    // List pages: the VALID_BOX rectangles are the rows; the big arrows page.
    const char* name = screen_ == Screen::CareerList ? "HubSelectCareer" : screen_ == Screen::ChallengeList ? "HubSelectChallenges"
                     : screen_ == Screen::OptDisplay ? "PageOptionsDisplay" : screen_ == Screen::OptAudio ? "PageOptionsAudio" : "HubSelectCamps";
    Page* p = scripts_.page(name);
    if (!p) return;
    for (const Item& it : p->items) {
        if (!inside(it)) continue;
        if (it.id.rfind("ITEM_ID_VALID_BOX", 0) == 0) {
            const int row = rowOf(it.id) - 1;
            const int idx = (screen_ == Screen::OptDisplay || screen_ == Screen::OptAudio) ? row : listFirst_ + row;
            if (screen_ == Screen::CampList && !campShown(idx)) return;  // PK11: a hidden row cannot be hovered
            if (screen_ == Screen::CampList && idx < int(hubs_.size())) {
                // PK11 0x47e610: the hovered row becomes the selection (the second page needs no click), a click on a
                // row goes to 0x47e460 at once.
                focus_ = idx;
                if (click) key(MenuKey::Accept);
                return;
            }
            if (idx >= 0) {
                const int before = focus_;
                focus_ = idx;
                if (before != idx) soundBit(4);  // 0x46c510
                if (click && before == idx) {
                    if (screen_ == Screen::OptDisplay || screen_ == Screen::OptAudio) soundBit(2), listPage(1);  // port: a click changes the row
                    else key(MenuKey::Accept);
                }
            }
            return;
        }
        if (click && it.id == "ITEM_ID_FLECHE_GAUCHE") return listPage(-1);
        if (click && it.id == "ITEM_ID_FLECHE_DROITE") return listPage(1);
    }
}

void Menu::keyUp(MenuKey k) {
    if (k == MenuKey::Up || k == MenuKey::Down || k == MenuKey::Left || k == MenuKey::Right)  // [port] arrows pan a focused map
        k = k == MenuKey::Up ? MenuKey::MapUp : k == MenuKey::Down ? MenuKey::MapDown : k == MenuKey::Left ? MenuKey::MapLeft : MenuKey::MapRight;
    if (k >= MenuKey::MapUp && k <= MenuKey::MapZoomOut) map_.held &= ~(1u << (int(k) - int(MenuKey::MapUp)));
}

void Menu::update(float dt) {
    time_ += dt;
    if (screen_ == Screen::OptRemap) {  // [port] remap: 0x4cc870 +0x484 (the capture gate) and 0x4cd9d0 +0x498
        remap_.clock += dt, remap_.noteTime += dt;
        remap_.pulse = std::fmod(remap_.pulse + dt, 10.0f);
        if (remap_.noteTime > 6.0f) remap_.note.clear();
    }
    if (dialogs_) {  // PK25: Dialog_MCAlert vf16 0x4410d0, then the storage's answer handling (0x4eb9c0)
        if (dialogs_->mcAlert().shown()) dialogs_->mcAlert().update(dt);
        storageFrame();
    }
    // PK18 0x46f6f0 [O]: every tab update moves the focused GROUP_LEFT_MENU item by a random integer in [-1, 1] on x and y (an item
    // that loses the focus keeps its last offset).
    if (dt > 0 && screen_ == Screen::Tab && !confirm_.open && !(tab_ == TabMap && map_.focus) && !(tab_ == TabEditor && ed_.state != 5)) {  // PK21: 0x4761d0 only
        const auto f = focusables();
        if (focus_ >= 0 && focus_ < int(f.size()) && f[size_t(focus_)]->group == "GROUP_LEFT_MENU")
            f[size_t(focus_)]->offX = float(jitterRand()), f[size_t(focus_)]->offY = float(jitterRand());
    }
    if (screen_ == Screen::Tab && tab_ == TabMap) mapUpdate(dt);
    if (screen_ == Screen::Tab && tab_ == TabEditor) editorUpdate(dt);  // PK21
    if (screen_ == Screen::EndRace) endRaceClock_ += dt;
    if (screen_ == Screen::Results) rankClamp();  // 0x4b1c50 runs with every draw of state 0x11 / 0x12
    if (screen_ == Screen::Unlock && !unlocks_.empty() && unlocks_.front().first == "EndRace_Event_Unlocked2") mapPulse_ += dt;  // 0x4b64e0: +0xe94 += dt
    if (screen_ == Screen::Credits) creditsClock_ += dt;  // 0x4934f0: +0x44c += dt  // P75: MissionHUD +0xd3c ramp (0x4b6050)
}

// P75 / P78: the end-race unlock chain of MissionHUD 0x4ba590 (pages from the state draws 0x4b64e0 .. 0x4b6f30) [O]:
// ranking / reward (0xf / 0x10) -> 0x1b EndRaceNewVehiclesUnlocked (122) when the count of vehicles for sale changed
// during the race (0x62af90: 0x56a140 counts records unlocked, without a code, class != -1, against mission +0x54);
// then 0x13 EndRace_Event_Unlocked2 (110) when camps opened (+0xe74; 0x4b6150: HUB_NAME1..4 = text 938 + uid,
// SELECT_EVENT_CHOICE TT 5968 one / 5969 several); then 0x14 EndRace_PilotLivery_Unlocked2 (111) when the race gave a
// pilot livery (+0xe7c, the first, 0x56b830), else 0x1c. Only from 0x14: the first doppler of +0xe80 (traffic kind 1)
// 0x15 LiveryDoppler (112), +0xe84 (kind 0) 0x16 VistaDoppler (113), +0xe88 (kind 2) 0x17 ChallengeDoppler (114),
// +0xe8c (kind 3, a vehicle) 0x18 Maverick (115) (0x630dd0 / 0x574cb0: the first entry of each kind the race reveals),
// else 0x1c; 0x16 goes on to 0x18 when +0xe8c is set too; after a doppler page +0xe90 (the vehicle the race gives,
// 0x56a320) -> 0x19 BonusCar (116, ITEM_ID_CAR_NAME = its name upper-cased, 0x4b6d60), then 0x1c EndRace_WhatNext2.
// P99: a challenge with its own vehicle (AddMissionNeededVehicle "MESH_ID_..."; 101 of the 190) starts with it: the
// challenge page's start 0x47efb0 runs TTMN "<mission>" <index> 0 through 0x4cac00 without the garage's vehicle (the
// career page's 0x47eee0 passes the garage's +0x88) [O]; so nothing has to be owned or bought, and the bonus-car
// challenges (MESH_ID_BONUS_*) are playable. [port] the session's vehicle is that variant.
bool Menu::startForced(const Event& e) {
    if (e.kind != Event::Challenge || e.vehicle.empty() || !startable_.count(e.vehicle)) return false;
    pendingEvent_ = e.id;
    requestStart({MenuRequest::StartEvent, e.id, "", e.vehicle});
    return true;
}

int Menu::vehiclesForSale() const {
    if (!career_) return -1;
    const auto& rv = career_->rules().vehicles;
    const auto& sv = career_->state().vehicles;
    int n = 0;
    for (size_t v = 0; v < rv.size() && v < sv.size(); ++v)
        if (sv[v].unlocked && (rv[v].code.empty() || upper(rv[v].code) == "NONE") && rv[v].cls != -1) ++n;
    return n;
}

void Menu::buildUnlocks() {
    unlocks_.clear();
    if (sessionForSale_ >= 0 && vehiclesForSale() != sessionForSale_) unlocks_.push_back({"EndRaceNewVehiclesUnlocked", {}});
    if (!award_.hubsUnlocked.empty()) {
        std::vector<std::string> names;
        for (const std::string& h : award_.hubsUnlocked)
            if (const Hub* hub = findHub(hubs_, h))
                names.push_back(hub->nameText >= 1071 && hub->nameText < 1090 ? textOf(938 + hub->nameText - 1071) : textOf(hub->nameText));
        unlocks_.push_back({"EndRace_Event_Unlocked2", names});
    }
    if (award_.pilotLiveries.empty()) return;
    unlocks_.push_back({"EndRace_PilotLivery_Unlocked2", {award_.pilotLiveries.front()}});
    bool kinds[4] = {false, false, false, false};
    for (const std::string& t : award_.trafficRevealed) {
        CareerRules::Traffic tr;
        tr.radar = t.substr(0, t.find(' '));
        kinds[std::clamp(tr.kind(), 0, 3)] = true;
    }
    static const char* const kPage[4] = {"EndRace_VistaDoppler_Unlocked2", "EndRace_LiveryDoppler_Unlocked2", "EndRace_ChallengeDoppler_Unlocked2",
                                         "EndRace_Maverick_Unlocked2"};
    int first = -1;
    for (int k : {1, 0, 2, 3})
        if (kinds[k] && first < 0) first = k;
    if (first < 0) return;
    unlocks_.push_back({kPage[first], {}});
    if (first == 0 && kinds[3]) unlocks_.push_back({kPage[3], {}});
    if (!award_.giftVehicle.empty()) {  // P99: +0xe90 is the race's own gift (0x56a320), not an UnlockNeed vehicle
        std::string n = upper(vehicleName(award_.giftVehicle));
        unlocks_.push_back({"EndRace_BonusCar_Unlocked2", {n}});
    }
}

bool Menu::nextUnlock() {
    if (screen_ == Screen::Unlock && !unlocks_.empty()) unlocks_.erase(unlocks_.begin());
    if (unlocks_.empty()) return false;
    screen_ = Screen::Unlock;
    return true;
}

// P78: the hub of the zone the player is in (0x4b18c0: 0x4a7f40 Zone_FindContaining, 0x577500 hub by zone).
int Menu::whatNextHub() const {
    if (!zones_) return -1;
    const int z = zones_->find(playerX_, playerZ_);
    if (z < 0) return -1;
    for (size_t i = 0; i < hubs_.size(); ++i)
        if (strcasecmp(hubs_[i].id.c_str(), zones_->zones[size_t(z)].name.c_str()) == 0) return int(i);
    return -1;
}

// P78: the unlock chain ends in 0x1c (EndRace_WhatNext2) for the offline races (0x19 / 0x13 / 8 / 9..0xb -> 0x1c).
void Menu::endChain() {
    if (mode_ == MenuMode::Results) {
        screen_ = Screen::WhatNext;
        return;
    }
    goTab(TabLast), mode_ = MenuMode::Pause;
}

// P75: the credits (0x4938d0 from 0x4944e0) [O]: ITEM_ID_CREDITS_RECT R; text i = TT 14001 .. 14800 at y_i = (i - 14001)
// 30 + (R.y + R.h) - 50 t (enter at the bottom, 50 px/s, 30 px apart). "END OF CREDITS" above the top (-30) -> back to the
// options; a line below the rect stops the loop, one above R.y - 30 is skipped. Alpha fades over 30 px at both edges
// (x 1/30, 0x9dc01c). "$" = blank line; "BITMAP_<name>" = a picture (crc of the text) at x = 0.475 W - 2, 0.05 W wide
// ("BITMAP_GAMESPY" 0.4 W x 0.2 W) [I: argument roles], drawn when fully above the bottom. Text: box (R.x - 2, y + 2) ..
// (R.x + R.w - 2, y + 32), centred, black shadow then white offset (-2, -2) [port: font 0, 24 px].
void Menu::drawCredits(UiRenderer& r) {
    Page* p = scripts_.page("PageOptionsCredits");
    if (!p) return;
    const Item* R = p->find("ITEM_ID_CREDITS_RECT");
    if (!R) return;
    const float top = R->y, bottom = R->y + R->h;
    // CUSTOM VEHICLES [port]: the packs' attribution lines (CC-BY and such) go in before FUEL's "END OF CREDITS".
    std::vector<std::string> lines;
    for (uint32_t i = 14001; i <= 14800; ++i) {
        std::string t = textOf(i);
        if (t == "END OF CREDITS") {
            for (const std::string& c : custom::creditLines()) lines.push_back(c);
            lines.push_back(t);
            break;
        }
        lines.push_back(std::move(t));
    }
    for (size_t k = 0; k < lines.size(); ++k) {
        const float y = float(k) * 30.0f + bottom - 50.0f * creditsClock_;
        const std::string& t = lines[k];
        if (t == "END OF CREDITS") {
            if (y < top - 30.0f) goTab(TabOptions);
            break;
        }
        if (y > bottom) break;
        if (y < top - 30.0f || t.empty() || t == "$") continue;
        float a = 1;
        if (y > bottom - 30.0f) a = (bottom - y) / 30.0f;
        if (y < top) a = (y - (top - 30.0f)) / 30.0f;
        a = std::clamp(a, 0.0f, 1.0f);
        if (t.rfind("BITMAP_", 0) == 0) {
            const bool gs = t == "BITMAP_GAMESPY";
            const float w = gs ? 0.4f * 1280 : 0.05f * 1280, h = gs ? 0.2f * 1280 : 0.05f * 1280;
            if (y + h <= bottom)
                if (BitmapRef b = assets_->image(t.substr(7))) r.image(b, (gs ? 0.5f * 1280 - w * 0.5f : 0.475f * 1280) - 2, y, w, h, Rgba{1, 1, 1, a});
            continue;
        }
        TextStyle st;
        st.font = 0, st.height = 24, st.align = 1;
        st.fill = Rgba{0, 0, 0, a}, st.outline = Rgba{0, 0, 0, 0};
        r.text(t, R->x - 2, y + 2 + 3, R->w, st);
        st.fill = Rgba{1, 1, 1, a};
        r.text(t, R->x - 4, y + 3, R->w, st);
    }
}

// P75: the end-race page of MissionHUD states 6/8/10/0xc (won, 0x4b7830) and 7/9/0xb (lost, 0x4b82c0) [O]. Nothing while
// a TR_DEATH cut-scene plays (0x53ecc0 queue entry 1 / not 5; the port's results open after it). Page by the mission list
// kind 0x577d70 (InterfacesInfosPacked order): won 0 TutorialRaceYouWin (119), 1 CareerRaceYouWin2 (105), 2
// ChallengeYouWin2 (117), 3 EditedRaceYouWin (120); lost 1 CareerRaceYouLose2 (106), 2 ChallengeYouLose2 (118), others
// none. Drawn with endrace_tache_tribal (104) at alpha +0xd3c (0x46ef80(0x68, a), 0x60fe10(page, a)); help line TT_OK.
// ITEM_ID_ADD_INFOS_SENTENCE (0x4b76b0) = the player's race time (last entry of the race's time list R+0x204),
// "%02d:%02d.%02d" below an hour else "%02d:%02d:%02d:%02d" (0x484690: h, min, s, cs, each floored).
void Menu::drawEndRace(UiRenderer& r) {
    static const char* const kWon[4] = {"TutorialRaceYouWin", "CareerRaceYouWin2", "ChallengeYouWin2", "EditedRaceYouWin"};
    static const char* const kLost[4] = {nullptr, "CareerRaceYouLose2", "ChallengeYouLose2", nullptr};
    const int k = std::clamp(resultsList_, 0, 3);
    const char* name = resultsWon_ ? kWon[k] : kLost[k];
    const float a = std::clamp(endRaceClock_, 0.0f, 1.0f);
    const float keep = r.alpha;
    r.alpha = keep * a;
    if (Page* t = scripts_.page("endrace_tache_tribal")) drawPage(r, *t);
    if (Page* p = name ? scripts_.page(name) : nullptr) {
        float time = 0;
        for (const ResultRow& rr : results_)
            if (rr.player) time = rr.time;
        if (Item* it = p->find("ITEM_ID_ADD_INFOS_SENTENCE")) {
            it->visible = time > 0;
            if (time > 0) {
                const double t = time;
                const int h = int(std::floor(t / 3600)), m = int(std::floor(std::fmod(t, 3600) / 60)), sec = int(std::floor(std::fmod(t, 60)));
                const int cs = int(std::floor((t - std::floor(t)) * 100));
                char buf[64];
                if (h == 0) std::snprintf(buf, sizeof buf, "%02d:%02d.%02d", m, sec, cs);
                else std::snprintf(buf, sizeof buf, "%02d:%02d:%02d:%02d", h, m, sec, cs);
                it->hasDynText = true, it->dynText = buf;
            }
        }
        drawPage(r, *p);
    }
    drawChrome(r);  // PK18: 0x4b7830 / 0x4b82c0 end with 0x46b270 at the page alpha
    r.alpha = keep;
}

// ---- drawing ---------------------------------------------------------------

void Menu::drawItem(UiRenderer& r, const Item& it, float dx, float dy) {
    if (!it.visible) return;
    // Safe-frame snapping (FUEL.exe 0x49c0a0, mode 1 = 90 % frame on PC).
    float x = it.x + dx, y = it.y + dy;
    if (it.anchorH == 1) x += 64;
    if (it.anchorH == 2) x -= 64;
    if (it.anchorV == 1 || it.anchorV == 3) y += 36;
    if (it.anchorV == 2 || it.anchorV == 4) y -= 36;
    x += it.offX, y += it.offY;  // PK18 0x46f6f0: the focus jitter (+0x18 / +0x1c)
    const int state = it.focus ? 2 : it.enabled ? 1 : 0;
    // PK18 0x46c400 [O]: the focused row's halo (ITEM_ID_HALO<n>) pulses: alpha = 0.25 + (sin(clock x 10) + 1) x 0.25 on an image
    // (the three colours' alpha), x 5 on a plain box.
    const bool rowHalo = it.id.size() > 12 && it.id.compare(0, 12, "ITEM_ID_HALO") == 0 && it.id.find_first_not_of("0123456789", 12) == std::string::npos;
    const float pulse = 0.25f + (std::sin(time_ * (it.kind == ItemKind::Image ? 10.0f : 5.0f)) + 1.0f) * 0.25f;
    if (it.kind == ItemKind::Image) {
        const std::string& name = state == 2 ? it.imgFocus : state == 1 ? it.imgEnabled : it.imgNormal;
        Rgba c = state == 2 ? it.cFocus : state == 1 ? it.cEnabled : it.cNormal;
        if (rowHalo) c.a = pulse;
        // PK18 0x612020 [O]: the image is drawn at (x - W a7 / 2, y - H a8 / 2); W x H = +0x90 / +0x94 in the normal state (a halo
        // sets it, 0x4701a0), else the item's w x h (+0x24 / +0x28; focus +0x98 / +0x9c start equal).
        const float W = state == 0 && it.drawW > 0 ? it.drawW : it.w, H = state == 0 && it.drawH > 0 ? it.drawH : it.h;
        r.image(assets_->image(name), x - W * float(it.a7) * 0.5f, y - H * float(it.a8) * 0.5f, W, H, c);
        return;
    }
    if (it.kind == ItemKind::FillRect) {
        Rgba c = state == 2 ? it.cFocus : state == 1 ? it.cEnabled : it.cNormal;
        if (rowHalo) c.a = pulse;
        r.rect(x, y, it.w, it.h, c);
        return;
    }
    std::string s;
    if (it.hasDynText) s = it.dynText;
    else if (it.kind == ItemKind::ListBoxTT) {
        if (it.entryIndex >= 0 && size_t(it.entryIndex) < it.entries.size()) s = textOf(it.entries[size_t(it.entryIndex)].tt);
    } else if (it.kind != ItemKind::LabelDyn) {
        s = textOf(it.tt);
    }
    if (s.empty()) return;
    TextStyle st;
    st.font = it.font;
    st.height = it.h;
    st.fill = state == 2 ? it.tFocus : state == 1 ? it.tEnabled : it.tNormal;
    st.outline = state == 2 ? it.cFocus : state == 1 ? it.cEnabled : it.cNormal;
    st.align = it.align;
    st.upper = it.upper;
    st.slant = it.slant ? 0.2f : 0.0f;
    // P75: 0x614f70 -> 0x6153e0 for items with the script's cc flag (+0xcc) and no markup ('^', 'µ', '§': 0x6157c0 sends
    // those to TextGameDraw 0x615860): greedy word wrap at ' ' (no shrink; a word wider than the item stands alone), a
    // token starting with '~' breaks the line; line i at x = item.x + (w - lineW) a7 / 2 (a7 = +0x34: 0 left, 1 centre,
    // 2 right), the first line at the item's place (+0x38 = 2 puts it at the bottom of the box; the port's text height
    // is the box height) and the next ones below, one line height apart.
    // P78: texts with '§' or 'µ' markup go to TextGameDraw like FUEL's (0x6157c0 -> 0x615860): box x = item.x + offset ..
    // + w, y = item.y .. + 1000 h (no vertical limit), wrapping, h-align = a7 (+0x34: 0 left, 2 right, else centre),
    // the item's scale; '§NNN' scales the rest of the text. ('^' texts keep the port's path: same colours, no wrap.)
    if (s.find("\xc2\xb5") != std::string::npos || s.find("\xc2\xa7") != std::string::npos) {
        const int a = it.a7 == 0 ? 0 : it.a7 == 2 ? 2 : 1;
        r.richText(s, x + it.xOffset, y, x + it.xOffset + it.w, y + it.h * 1000.0f, a, st, true, false);
        return;
    }
    if (it.flagCC && s.find('^') == std::string::npos && s.find("\xc2\xb5") == std::string::npos &&
        s.find("\xc2\xa7") == std::string::npos) {
        std::vector<std::string> tokens;
        for (size_t p = 0; p <= s.size();) {
            size_t q = s.find(' ', p);
            if (q == std::string::npos) q = s.size();
            if (q > p) tokens.push_back(s.substr(p, q - p));
            p = q + 1;
        }
        std::vector<std::string> lines;
        size_t t = 0;
        while (t < tokens.size()) {
            std::string line = tokens[t] == "~" ? std::string() : tokens[t][0] == '~' ? tokens[t].substr(1) : tokens[t];
            ++t;
            while (t < tokens.size() && r.textWidth(line, st) < it.w) {
                if (tokens[t][0] == '~') {
                    if (tokens[t].size() > 1) tokens[t] = tokens[t].substr(1);  // [port] "~word": break, keep the word
                    else ++t;
                    break;
                }
                const std::string cand = line + " " + tokens[t];
                if (r.textWidth(cand, st) >= it.w) break;
                line = cand, ++t;
            }
            lines.push_back(line);
        }
        TextStyle one = st;
        one.align = 0;
        const float a7 = float(std::clamp(it.a7, 0, 2));
        for (size_t i = 0; i < lines.size(); ++i) {
            const float lw = r.textWidth(lines[i], one);
            r.text(lines[i], x + (it.w - lw) * a7 * 0.5f, y + float(i) * it.h, lw + 1, one);
        }
        return;
    }
    // PK18: BandeauHelp's scrolling help line (script b19: clip to the item rect, marquee at -32 px/s, 0x614bb0 / 0x614b70).
    if (it.b19 && (it.id == "ITEM_ID_SCROLLING_HELPS" || it.id == "ITEM_ID_SCROLLING_HELPS_DYN")) {
        TextStyle ms = st;
        ms.align = 0;
        const float tw = r.textWidth(s, ms);
        if (marqueeDt_ > 0) {
            it.scrollX += marqueeDt_ * -32.0f;
            if (it.scrollX < -tw) it.scrollX = it.w;  // it has fully left on the left: restart at the right edge
        }
        r.setClip(x, y, it.w, it.h * 1.25f);  // [port] the clip is the item rect (+0xce); the port's glyph box is 1.25 h (descenders)
        r.text(s, x + it.scrollX, y, tw + 1, ms);
        r.clearClip();
        return;
    }
    // PK18 0x615240 -> 0x6090e0 [O]: the script's F9 (+0xc8, degrees; 358 on the left-menu labels, 356 on five) turns the single
    // line about its anchor point (a7 / 2, a8 / 2 of the text box). Only the rotation is ported here: the placement (the parked
    // alignment question) is untouched.
    if (it.f9 != 0 && std::fmod(it.f9, 360.0f) != 0) {
        r.setTextRotation(it.f9, float(it.a7) * 0.5f, float(it.a8) * 0.5f);
        r.textFit(s, x + it.xOffset, y, it.w, st);
        r.clearTextRotation();
        return;
    }
    r.textFit(s, x + it.xOffset, y, it.w, st);
}

void Menu::drawPage(UiRenderer& r, Page& p, float dx, float dy) {
    for (const Item& it : p.items) drawItem(r, it, dx, dy);
}

void Menu::applyHubStates() {
    if (Page* hub = scripts_.page("TemplateLeftMenu"))  // PK11: 0x47ca70 / 0x481f80 (HubNeutre)
        for (Item& it : hub->items)
            if (it.id.rfind("ITEM_ID_LEFT_MENU", 0) == 0 && it.id.size() == 18) it.enabled = hubItemEnabled(it.id);
}

void Menu::applyItemStates() {
    // Top bar: available tabs enabled, the current one focused with its halo.
    applyTopBar();  // PK18: icons, halos (0x46f080 / 0x4701a0)
    if (Page* m = scripts_.page("Map")) {
        if (Item* hub = m->find("ITEM_ID_TITLE_HUB_INFO"); hub && !hubs_.empty()) {
            const std::string key = "TT_HUB_" + upper(hubs_[size_t(hubIndex_)].id);
            for (size_t i = 0; i < hub->entries.size(); ++i)
                if (upper(hub->entries[i].key) == key) hub->entryIndex = int(i);
        }
    }
    // Left menus: what the port can do is enabled; the rest keeps the authored look, disabled.
    // P78: BONUS_CODES is ported (bonus_codes.mm). P101: CONTROLLER is ported (its SETTINGS page); [port] remap: LAYOUT (the
    // remap page, state 0x29 -> 0x4cc340) too, so nothing is disabled here any more.
    static const char* kOff[] = {"ITEM_ID_NOTHING_DISABLED"};
    for (Page& p : scripts_.pages)
        for (Item& it : p.items) {
            it.focus = false;
            for (const char* off : kOff)
                if (it.id == off) it.enabled = false;
            if (it.id == "ITEM_ID_TOPMENU_RACE_RESTART") it.enabled = !sessionEvent_.empty();
        }
    applyHubStates();
    // P66: 0x47c050: START_RACE, LIVERY and COLOR enabled (+9) when the garage's vehicle is owned (+1).
    if (Page* g = scripts_.page("Garage")) {
        const bool owned = !career_ || career_->vehicleOwned(vehicle_);
        for (const char* id : {"ITEM_ID_TOPMENU_GARAGE_START_RACE", "ITEM_ID_TOPMENU_GARAGE_LIVERY", "ITEM_ID_TOPMENU_GARAGE_COLOR"})
            if (Item* it = g->find(id)) it->enabled = owned;
    }
    layoutGarageList();  // [upgrades U3] UPGRADES shown (`upgrades = on`, DRIVER .. PAINTWORK one row up) or FUEL's list
    if (Page* g = scripts_.page("Garage"))
        if (Item* start = g->find("ITEM_ID_TOPMENU_GARAGE_START_RACE")) {
            const Event* pe = nullptr;
            for (const Event& e : events_)
                if (e.id == pendingEvent_) pe = &e;
            start->hasDynText = pe == nullptr;
            start->dynText = textOf(10137);  // no event chosen: free ride in the current hub
            if (pe) start->entryIndex = pe->kind == Event::Career ? 0 : 1;
        }
    if (screen_ == Screen::Tab) {
        auto f = focusables();
        if (!f.empty()) f[size_t(std::min(focus_, int(f.size()) - 1))]->focus = true;
    }
}

void Menu::prepareRows() {
    auto setRow = [](Page& p, int row, bool visible) {
        for (Item& it : p.items)
            if (rowOf(it.id) == row && it.id.rfind("ITEM_ID_FLECHE", 0) != 0) it.visible = visible;
    };
    if (screen_ == Screen::CareerList || screen_ == Screen::ChallengeList) {
        const bool career = screen_ == Screen::CareerList;
        Page* p = scripts_.page(career ? "HubSelectCareer" : "HubSelectChallenges");
        if (!p) return;
        const auto list = hubEvents(career ? Event::Career : Event::Challenge);
        const int rows = career ? 3 : 5;
        for (int k = 1; k <= rows; ++k) {
            const int idx = listFirst_ + k - 1;
            const bool on = idx < int(list.size());
            setRow(*p, k, on);
            if (!on) continue;
            const Event& e = *list[size_t(idx)];
            const std::string n = std::to_string(k);
            if (Item* it = p->find("ITEM_ID_RACE_NAME" + n)) {
                it->hasDynText = true;
                it->dynText = e.nameText ? textOf(e.nameText) : e.id;
                if (it->dynText.empty()) it->dynText = e.id;
                it->enabled = false;                    // +9 turns these labels black (authored colours)
                it->tNormal.a = e.playable ? 1.0f : 0.4f;  // port: races it cannot run yet are dimmed
            }
            if (Item* it = p->find("ITEM_ID_BUTTON_NUMBER" + n)) {
                char b[8];
                std::snprintf(b, sizeof b, "%02d", idx + 1);
                it->hasDynText = true, it->dynText = b, it->focus = idx == focus_;
            }
            if (Item* it = p->find("ITEM_ID_MEDAL_SLOT" + n)) {
                // 0x480000 (career): MAP_CAREER_{GOLD,SILVER,BRONZE}_ICON by best rank, hidden at rank 3;
                // 0x480800 (challenges): MAP_CHALLENGE_GOLD_ICON once won.
                static const char* const kCareerIcon[3] = {"MAP_CAREER_GOLD_ICON", "MAP_CAREER_SILVER_ICON", "MAP_CAREER_BRONZE_ICON"};
                const MissionStatus* ms = career_ ? career_->status(e.id) : nullptr;
                const int rank = ms ? ms->bestRank : 3;
                it->visible = rank >= 0 && rank < 3;
                if (it->visible) it->imgNormal = it->imgEnabled = it->imgFocus = career ? kCareerIcon[rank] : "MAP_CHALLENGE_GOLD_ICON";
            }
            if (Item* it = p->find("ITEM_ID_FUEL_REWARD" + n)) it->visible = false;  // not set by 0x480000 [U]
            if (Item* it = p->find("ITEM_ID_HALO" + n)) it->visible = idx == focus_, it->focus = true;
        }
        for (const char* a : {"ITEM_ID_FLECHE_GAUCHE", "ITEM_ID_FLECHE_DROITE"})
            if (Item* it = p->find(a)) it->visible = int(list.size()) > rows;
    } else if (screen_ == Screen::CampList || screen_ == Screen::VehicleList) {
        Page* p = scripts_.page("HubSelectCamps");
        if (!p) return;
        const auto vehicles = vehicleChoices();
        const int count = screen_ == Screen::CampList ? int(hubs_.size()) : int(vehicles.size());
        for (int k = 1; k <= 10; ++k) {
            const int idx = listFirst_ + k - 1;
            const bool on = idx < count;
            setRow(*p, k, on);
            if (!on) continue;
            const std::string n = std::to_string(k);
            if (Item* it = p->find("ITEM_ID_HUB_NAME" + n)) {
                it->hasDynText = true;
                if (screen_ == Screen::CampList) {
                    // P75 0x481580: HUB_NAME = text 938 + uid (upper-case camp names; mode 1 10165, tutorial 10184),
                    // locked hubs' rows hidden (0x579720), the current camp's label disabled (+9 = h != current).
                    const Hub& h = hubs_[size_t(idx)];
                    const bool hasUid = h.nameText >= 1071 && h.nameText < 1071 + 19;  // name texts are 1071 + uid
                    it->dynText = hasUid ? textOf(938 + (h.nameText - 1071)) : h.nameText ? textOf(h.nameText) : h.id;
                    const int ch = career_ ? career_->rules().hubIndex(h.id) : -1;
                    if (ch >= 0 && !career_->hubUnlocked(ch)) {
                        setRow(*p, k, false);
                        continue;
                    }
                    const bool current = career_ ? ch >= 0 && ch == career_->currentHub() : upper(h.id) == sessionHub_;
                    it->enabled = !current;
                    if (Item* halo = p->find("ITEM_ID_HALO" + n)) halo->visible = idx == focus_, halo->focus = true;
                    continue;
                } else {
                    const VehicleInfo& v = *vehicles[size_t(idx)];
                    const std::string nm = textOf(v.nameText);
                    it->dynText = nm.empty() ? v.name : nm;
                }
                it->enabled = true;
                it->tEnabled.a = 1.0f;
                if (screen_ == Screen::VehicleList && career_ && !career_->vehicleOwned(vehicles[size_t(idx)]->name))
                    it->tEnabled.a = 0.45f;  // port: not owned (FUEL's garage art for this is not decoded)
            }
            if (Item* it = p->find("ITEM_ID_HALO" + n)) it->visible = idx == focus_, it->focus = true;
        }
        if (screen_ == Screen::CampList) {  // P75 0x481580: page 1 -> right arrow iff more hubs than rows; page 2 -> left arrow
            if (Item* it = p->find("ITEM_ID_FLECHE_GAUCHE")) it->visible = focus_ >= 10;
            if (Item* it = p->find("ITEM_ID_FLECHE_DROITE")) it->visible = focus_ < 10 && count > 10;
        } else
            for (const char* a : {"ITEM_ID_FLECHE_GAUCHE", "ITEM_ID_FLECHE_DROITE"})
                if (Item* it = p->find(a)) it->visible = count > 10;
    } else if (screen_ == Screen::Results) {
        Page* p = scripts_.page("race_ranking_win");
        if (!p) return;
        for (Item& it : p->items) it.visible = true;
        // PK08 0x4b1c50 (offline): row k shows racer (+0xe70 - +0xe6c + k) while it exists (+0xe6c = the highlighted row, clamped
        // to 1..5 / the racer count); the HALO belongs to that row; a row past the list stays hidden and ends the fill.
        rankClamp();
        for (int k = 1; k <= 5; ++k) {
            const int ri = rankIndex(k);
            const bool on = ri >= 0;
            setRow(*p, k, on);
            if (!on)  // the row's unnumbered bars too (those inside the row's halo band)
                if (const Item* halo = p->find("ITEM_ID_HALO" + std::to_string(k)))
                    for (Item& it : p->items)
                        if (rowOf(it.id) == 0 && it.y + it.h * 0.5f > halo->y && it.y + it.h * 0.5f < halo->y + halo->h) it.visible = false;
            if (!on) continue;
            const ResultRow& rr = results_[size_t(ri)];
            const std::string n = std::to_string(k);
            auto dyn = [&](const std::string& id, const std::string& s) {
                if (Item* it = p->find(id)) it->hasDynText = true, it->dynText = s, it->enabled = rr.player;
            };
            dyn("ITEM_ID_PLAYER_POS" + n, std::to_string(ri + 1));
            dyn("ITEM_ID_PLAYER_NAME" + n, rankName(k));
            dyn("ITEM_ID_PLAYER_TIME" + n, formatTime(rr.time));
            if (Item* it = p->find("ITEM_ID_HALO" + n)) it->visible = k == rankRow_, it->focus = true;
            if (Item* it = p->find("ITEM_ID_PLAYER_VEHICLE" + n)) it->visible = false;  // drawn below (0x4b7cf0)
        }
        for (const char* a : {"ITEM_ID_FLECHE_UP", "ITEM_ID_FLECHE_DOWN"})
            if (Item* it = p->find(a)) it->visible = results_.size() > 5;
        std::string title = sessionEvent_;
        for (const Event& e : events_)
            if (e.id == sessionEvent_ && e.nameText) title = textOf(e.nameText);
        if (Item* it = p->find("UNDEFINED_ITEM_ID")) it->hasDynText = true, it->dynText = title;
        if (Item* it = p->find("ITEM_ID_STATUS_MSG")) it->hasDynText = true, it->dynText = textOf(235), it->align = 1;
        fillEndRaceInfos();
    } else if (screen_ == Screen::OptDisplay) {
        Page* p = scripts_.page("PageOptionsDisplay");
        if (!p) return;
        const bool vals[5] = {options_.raceInfo, options_.rankingInfo, options_.gps, options_.speedMeter, options_.miles};
        const char* lists[5] = {"ITEM_ID_DISPLAY_OPTIONS_RACE_INFOS_STATE", "ITEM_ID_DISPLAY_OPTIONS_RANKING_INFOS_STATE",
                                "ITEM_ID_DISPLAY_OPTIONS_GPS_MAP_STATE", "ITEM_ID_DISPLAY_OPTIONS_SPEED_METER_STATE",
                                "ITEM_ID_DISPLAY_OPTIONS_RACE_KM_MILES"};
        for (int k = 0; k < 5; ++k)
            if (Item* it = p->find(lists[k])) it->entryIndex = vals[k] ? 1 : 0, it->focus = focus_ == k;
        if (Item* g = p->find("ITEM_ID_DISPLAY_OPTIONS_RECT_BRIGHTNESS_PERCENT")) g->w = 313.0f * options_.brightness;
        if (Item* lb = p->find("ITEM_ID_DISPLAY_OPTIONS_BRIGHTNESS_PERCENT")) lb->visible = false;
        // The arrows (authored on row 5) follow the focused row; rows are 49 apart (port rows too).
        for (const char* a : {"ITEM_ID_FLECHE_GAUCHE", "ITEM_ID_FLECHE_DROITE"})
            if (Item* it = p->find(a)) it->y = 419.0f + 49.0f * float(focus_ - 4);
        for (Item& it : p->items) {
            const int row = rowOf(it.id);
            if (it.id.rfind("ITEM_ID_HALO", 0) == 0) it.visible = row - 1 == focus_, it.focus = true;
        }
    } else if (screen_ == Screen::OptAudio) {
        Page* p = scripts_.page("PageOptionsAudio");
        if (!p) return;
        if (Item* g = p->find("ITEM_ID_SOUND_VOLUME_PERCENT")) g->w = 317.0f * settings_.volume;
        if (Item* g = p->find("ITEM_ID_MUSIC_VOLUME_PERCENT")) g->w = 317.0f * options_.musicVolume;
        for (const char* a : {"ITEM_ID_FLECHE_GAUCHE", "ITEM_ID_FLECHE_DROITE"})
            if (Item* it = p->find(a)) it->y = 307.0f + 60.0f * float(focus_), it->visible = focus_ < 2;  // P101: row 3 draws its own
        for (Item& it : p->items) {
            const int row = rowOf(it.id);
            if (it.id.rfind("ITEM_ID_HALO", 0) == 0) it.visible = row - 1 == focus_, it.focus = true;
        }
    }
}

void Menu::draw(UiRenderer& r) {
    struct AlertsOnTop {  // PK25: the Front dialogs draw over every screen (whatever return the screens take)
        Menu* m;
        UiRenderer& r;
        ~AlertsOnTop() { m->drawAlerts(r); }
    } alertsOnTop{this, r};
    applyItemStates();
    prepareRows();
    applyChrome();  // PK18: TopTitle / TopTitleDyn / BandeauHelp
    auto page = [&](const char* name, float dx = 0, float dy = 0) {
        if (Page* p = scripts_.page(name)) drawPage(r, *p, dx, dy);
    };
    if (screen_ == Screen::Title) {
        // resrce.tsc MENUDialog PressStart (CWindow_G, 0..1 coordinates): the
        // PressStartText window at 0.6, 0.8, 0.3 x 0.0833 in text_black (black, outlined
        // white, SMALL font, centred). The logo bitmap is set by code (not decoded).
        BitmapRef bg = assets_->image("PRESS_START");
        if (bg) r.image(bg, 0, 0, 1280, 720, Rgba{1, 1, 1, 1});
        TextStyle st;
        st.font = 0;
        st.height = 26;
        st.fill = Rgba{0, 0, 0, 1};
        st.outline = Rgba{1, 1, 1, 1};
        st.align = 1;
        // PK26 0x592ce0: colour (0, 0, 0, (cos 5t + 1) / 2); 0x591d60: TT 1218 with a pad (PaddleChecker +0x24), else TT 230.
        // (The boot runs ui/boot_screens.mm's PressStart dialog; this screen is the menu pages' own title.)
        const float pulse = (std::cos(time_ * 5.0f) + 1.0f) * 0.5f;
        st.fill.a = st.outline.a = pulse;
        r.textFit(textOf(helpTexts().padMode() ? 1218 : 230), 0.6f * 1280, 0.8f * 720 + 17, 0.3f * 1280, st);
        return;
    }
    if (screen_ == Screen::Single) {
        if (!singlePage_.empty()) page(singlePage_.c_str());
        drawMessages(r);  // PK08: --menu-shot BANNER:<kind>
        if (tutorialOpen()) drawTutorial(r);  // P56
        return;
    }
    if (screen_ == Screen::EndRace) return drawEndRace(r);  // P75: a MissionHUD page over the running world
    if (screen_ == Screen::WhatNext) {
        // 0x4b6f30: pages 104 (endrace_tache_tribal) and 123 (EndRace_WhatNext2) at alpha +0xd3c; 0x4b18c0 sets
        // ITEM_ID_BACK_TO_HUB = sprintf(its text TT 898, the hub name of the player's zone) (0x46cdb0, not upper-cased).
        page("endrace_tache_tribal");
        if (Page* p = scripts_.page("EndRace_WhatNext2")) {
            if (Item* it = p->find("ITEM_ID_BACK_TO_HUB")) {
                int h = whatNextHub();
                if (h < 0 && !hubs_.empty()) {  // [port] no zone data (menu shots): the session's hub
                    h = hubIndex_;
                    for (size_t i = 0; i < hubs_.size(); ++i)
                        if (!sessionHub_.empty() && upper(hubs_[i].id) == sessionHub_) h = int(i);
                }
                std::string name = h >= 0 && h < int(hubs_.size()) ? textOf(hubs_[size_t(h)].nameText) : std::string();
                std::string fmt = textOf(898);
                char buf[512];
                std::snprintf(buf, sizeof buf, fmt.find("%s") != std::string::npos ? fmt.c_str() : "%s", fmt.find("%s") != std::string::npos ? name.c_str() : fmt.c_str());
                // 0x615240 places it at item.x + (w - textW) a7 / 2 (a7 = 2: right, ending at 1216 after the -64 safe
                // frame); the port's code alignment (left) would start it off screen (P19 alignment question, as for STATUS_MSG).
                it->hasDynText = true, it->dynText = buf, it->align = std::clamp(it->a7, 0, 2);
            }
            drawPage(r, *p);
        }
        return;
    }
    if (screen_ == Screen::FuelReward) {
        // 0x4b68b0 (state 0x1a): pages 104 and 121 at +0xd3c, then 0x46b270. 0x4b7630 (run by the challenge win page 0x4b7830 while
        // mission +0x50): ITEM_ID_FUEL_WON = "+%d" of ftol(the mission's gold fuel, +0x94), ITEM_ID_FUEL_TOTAL = game +0x408.
        page("endrace_tache_tribal");
        fuelRewardTexts();
        if (Page* p = scripts_.page("EndRaceFuelChallengeReward")) drawPage(r, *p);
        drawChrome(r);
        return;
    }
    if (screen_ == Screen::Unlock) {  // P75: endrace_tache_tribal + the unlock page (0x4b64e0 ..: pages 104 and 110..122)
        if (unlocks_.empty()) return;
        page("endrace_tache_tribal");
        Page* p = scripts_.page(unlocks_.front().first);
        if (!p) return;
        const auto& texts = unlocks_.front().second;
        if (unlocks_.front().first == "EndRace_Event_Unlocked2") {  // 0x4b6150
            for (int k = 1; k <= 4; ++k)
                if (Item* it = p->find("ITEM_ID_HUB_NAME" + std::to_string(k))) {
                    it->visible = k <= int(texts.size());
                    if (it->visible) it->hasDynText = true, it->dynText = texts[size_t(k - 1)];
                }
            if (Item* it = p->find("ITEM_ID_SELECT_EVENT_CHOICE")) it->entryIndex = texts.size() < 2 ? 0 : 1;
            unlockMapPictures(nullptr);  // PK08 0x4b63a0 / 0x4b64e0
        } else {
            for (const char* id : {"ITEM_ID_LIVERY_NAME", "ITEM_ID_CAR_NAME"})
                if (Item* it = p->find(id); it && it->kind == ItemKind::LabelDyn) {
                    it->visible = !texts.empty();
                    if (!texts.empty()) it->hasDynText = true, it->dynText = texts.front();
                }
        }
        drawPage(r, *p);
        drawChrome(r);  // PK18: every unlock page draw ends with 0x46b270
        return;
    }
    page("tache_tout_menu");
    if (screen_ != Screen::Results) {  // the ranking page has its own title line
        page("TopMenu");
        page("TopTitle");
    }
    switch (screen_) {
        case Screen::Tab:
            if (tab_ == TabMap) {
                if (career_) drawMap(r);  // P44: the map under the left column
                page("Map");
                if (career_ && map_.focus) drawMapInfo(r);
                else page(mapData_ ? "HubData" : "TemplateLeftMenu");  // P66: DATA
                if (hubLocked())
                    if (Page* lp = scripts_.page("Map_locked_hub")) {
                        // 0x483dd0: stars still needed = hub +0x23c - total stars.
                        const int h = careerHub();
                        if (Item* it = lp->find("ITEM_ID_NB_STARS_NEED_TO_ENTER_HUB"))
                            it->hasDynText = true, it->dynText = std::to_string(career_->rules().hubs[size_t(h)].starsNeeded - career_->stars());
                        drawPage(r, *lp);
                    }
            } else if (tab_ == TabEditor) {
                drawEditor(r);  // PK21: 0x476b30
            } else {
                page(tab_ == TabGarage && !garage_.page.empty() ? garage_.page.c_str()
                     : tab_ == TabOptions && ctrlMenu_          ? "PageOptionsController"  // P101: state 0x29
                                                                : kTabPage[tab_]);  // P66: Driver menus
            }
            if (career_) fillEconomy(), page("economy_hud");  // P44: 0x46f4d0 -> 0x46d1e0 on the left-menu pages
            break;
        case Screen::GarageSelect: drawGarageSelect(r); break;  // P66
        case Screen::GarageUpgrades: drawUpgrades(r); break;    // [upgrades U3]
        case Screen::Profile: drawProfile(r); break;            // P66
        case Screen::CareerList:
        case Screen::ChallengeList: {
            const bool career = screen_ == Screen::CareerList;
            page(career ? "HubSelectCareer" : "HubSelectChallenges");
            const auto list = hubEvents(career ? Event::Career : Event::Challenge);
            if (focus_ < int(list.size())) drawRaceInfo(r, *list[size_t(focus_)], career);
            break;
        }
        case Screen::RacePage: drawRace(r); break;  // PK02
        case Screen::CampList: page("HubSelectCamps"); break;
        case Screen::VehicleList: {
            drawVehicleSelector(r);  // P62: GearSelecteur (0x473ea0) instead of the port's list page
            // GarageVehiclePriceInfos for the focused vehicle, then an open info box.
            const auto list = vehicleChoices();
            Page* pp = scripts_.page("GarageVehiclePriceInfos");
            if (career_ && pp && focus_ < int(list.size())) {
                const std::string& label = list[size_t(focus_)]->name;
                const bool owned = career_->vehicleOwned(label);
                const int price = career_->vehiclePrice(label);
                auto dyn = [&](const char* id, const std::string& v, bool show) {
                    if (Item* it = pp->find(id)) it->hasDynText = true, it->dynText = v, it->visible = show;
                };
                dyn("ITEM_ID_VEHICLE_NAME", vehicleName(label), true);
                dyn("ITEM_ID_YOUR_FUEL", std::to_string(career_->state().fuel), true);
                dyn("ITEM_ID_VEHICLE_FUEL_PRICE", price < 0 ? std::string("-") : std::to_string(price), !owned);
                if (Item* it = pp->find("ITEM_ID_VEHICLE_OWNED")) it->visible = owned;
                drawPage(r, *pp);
            }
            // P62: GarageVehiclePerformanceInfos (0x47b330): seven rows of six points, ITEM_ID_<ROW><i> visible when
            // i <= trunc(v x 6 + 0.5) (fmul 6.0 0x9dbf78, fadd 0.5 0x9960c0, ftol), v from the vehicle record (0x477f40):
            // SPEED +0x14, ACCELERATION +0x18, GRIP +0x1c, BRAKE +0x20, RELIABILITY +0x24, ASPHALT +0x28, OFFROAD +0x2c,
            // which AddVehicleInterfaceParams (0x56b040) fills from its arguments 1, 2, 3, 4, 7, 5, 6.
            if (Page* perf = scripts_.page("GarageVehiclePerformanceInfos"); perf && focus_ < int(list.size())) {
                const VehicleInfo& v = *list[size_t(focus_)];
                static const std::pair<const char*, int> kRows[7] = {{"ITEM_ID_SPEED", 0},       {"ITEM_ID_ACCELERATION", 1}, {"ITEM_ID_GRIP", 2},
                                                                     {"ITEM_ID_BRAKE", 3},       {"ITEM_ID_RELIABILITY", 6},  {"ITEM_ID_ASPHALT", 4},
                                                                     {"ITEM_ID_OFFROAD", 5}};
                for (const auto& [row, arg] : kRows) {
                    const float val = size_t(arg) < v.bars.size() ? v.bars[size_t(arg)] : 0.0f;
                    const int n = int(val * 6.0f + 0.5f);
                    for (int i = 1; i <= 6; ++i)
                        if (Item* it = perf->find(std::string(row) + std::to_string(i))) it->visible = i <= n;
                }
                if (Item* it = perf->find("ITEM_ID_VEHICLE_NAME")) it->hasDynText = true, it->dynText = vehicleName(v.name);
                drawPage(r, *perf);
            }
            if (!infoBox_.empty()) drawGarageInfoBox(r);  // [upgrades U3] FUEL's PopUpHelp TT_PAGE_VEHICLE_* (was the absent InfoBoxGarage* pages)
            break;
        }
        case Screen::OptDisplay: page("PageOptionsDisplay"); break;  // P101: FUEL's six rows (the port rows moved to GRAPHICS)
        case Screen::OptGraphics: drawGraphics(r); break;                // P101
        case Screen::OptController: drawController(r); break;            // P101
        case Screen::OptRemap: drawRemap(r); break;                       // [port] remap (remap_page.mm)
        case Screen::OptAudio: page("PageOptionsAudio"), drawAudioRows(r); break;  // P101: + the limiter row
        case Screen::EndRace: break;
        case Screen::Credits: drawCredits(r); break;
        case Screen::BonusCodes:
        case Screen::BonusKeyboard:
        case Screen::BonusResult: drawBonus(r); break;
        case Screen::Results:
            page("end_race_infos");  // P56: filled by 0x4b2b20 when the ranking state (0x11 / 0x12) starts
            page("race_ranking_win");
            // P75: 0x4b7cf0 (offline): ITEM_ID_PLAYER_VEHICLE<k> = the row's vehicle picture (racer vehicle +0x50c -> vehicle
            // info game+0x3258 + 0x10c i, picture crc +0x4c, the garage carousel's packed image), drawn by 0x60acf0 at the
            // item's rect in the neutral colour 0xa064a8 (white here).
            if (Page* p = scripts_.page("race_ranking_win"))
                for (int k = 1; k <= 5 && rankIndex(k) >= 0; ++k)
                    if (const Item* it = p->find("ITEM_ID_PLAYER_VEHICLE" + std::to_string(k)))
                        if (const VehicleInfo* v = vehicleInfo(results_[size_t(rankIndex(k))].name); v && !v->picture.empty()) {
                            float x = 0, y = 0;
                            itemRect(*it, x, y);
                            r.image(assets_->image(v->picture), x, y, it->w, it->h, Rgba{1, 1, 1, 1});
                        }
            break;
        case Screen::Reward:
            raceRewardItems();  // medals: also what --career-audit reads (R11)
            if (Page* p = scripts_.page("race_reward")) {  // packed name of CareerRaceReward.tsc
                drawPage(r, *p);
            }
            break;
        default: break;
    }
    drawChrome(r);  // PK18: TopTitleDyn + BandeauHelp after the page of the state (0x46b750 / 0x46b270)
    drawDevTag(r);  // [dev mode] the overlay's tag on the garage pages (dev_mode.mm)
    if (confirm_.open) drawConfirm(r);
    if (tutorialOpen()) drawTutorial(r);  // P62: menu-side tutorials (11..15) over the menus
}

// ---- P62: confirmation popup and the options pages' rules ------------------------------------------------------------

void Menu::openConfirm(const std::string& key, std::function<void()> yes, std::function<void()> no) {
    confirm_ = {true, key, std::move(yes), std::move(no), 1};  // 0x4ce390(0): No focused
}

void Menu::openOk(const std::string& key, std::function<void()> ok) {
    confirm_ = {true, key, std::move(ok), {}, 0, true};
}

// P66: LOAD. FUEL (0x47d6d0 -> 0xb1 .. 0xb6 0x47f3b0: SaveManager_LoadSlot(1) -> 0xb7 0x47f460): error 0 -> 0xb8
// TT_LOAD_GAME_SUCCESSFULL (OK 0x4ce8e0: unpack the buffer 0x567af0, "RestartFromBufferSave": the game restarts in the
// saved hub, 0x5539f0); errors 5 / 6 (no save) -> 0xb9 TT_LOAD_GAME_NO_FUEL_DATA_WAS_FOUND (OK -> 0x53); others ->
// 0x53. The PC build keeps one save per gamer profile (savedir\gamertag\name.sav), so there is no profile list or
// delete in the game; the port keeps its one career.sav the same way.
void Menu::dataLoad() {
    auto backToMap = [this] { mapData_ = false, focus_ = 0, setTitle("TT_TOP_TITLE_MAP"); };
    if (!career_) return backToMap();
    std::error_code ec;
    if (!fs::exists(Career::careerPath(), ec)) return openOk("TT_LOAD_GAME_NO_FUEL_DATA_WAS_FOUND", backToMap);
    auto loaded = [this] {
        openOk("TT_LOAD_GAME_SUCCESSFULL", [this] {
            // RestartFromBufferSave -> 0x5539f0: KillMission, TeleportToHub(game +0x33c0) with the vehicle +0x33c8.
            const auto& R = career_->rules();
            const int hub = career_->currentHub(), veh = career_->state().get<int32_t>(0x10);
            std::string hubId = hubs_.empty() ? std::string() : hubs_[size_t(hubIndex_)].id;
            if (hub >= 0 && size_t(hub) < R.hubs.size())
                for (const Hub& h : hubs_)
                    if (upper(h.id) == upper(R.hubs[size_t(hub)].name)) hubId = h.id;
            if (veh >= 0 && size_t(veh) < R.vehicles.size() && career_->vehicleOwned(R.vehicles[size_t(veh)].label) && startable_.count(R.vehicles[size_t(veh)].label))
                vehicle_ = R.vehicles[size_t(veh)].label;
            if (!hubId.empty()) request_ = {MenuRequest::FreeRide, "", hubId, vehicle_};
        });
    };
    // PK25: LoadSlot(1) -> 0x4eb9c0 -> 0x47f460: error 0 -> 0xb8, a corrupt save -> MCAlert TT 9113 first: Yes (overwrite:
    // +0x5c 0x17 -> 0) -> 0xb8 TT_LOAD_GAME_SUCCESSFULL (the restart uses the game's buffer = the career in memory);
    // No -> autosave off, TT 9023, then 0x53.
    const int rc = loadCareerFile();
    if (rc == 0) {
        std::printf("career: loaded %s\n", Career::careerPath().c_str());
        return loaded();
    }
    if (storage_.loadAlert) {
        afterAlert_ = [this, loaded, backToMap] { storage_.loadErr == 0 ? loaded() : backToMap(); };
        return;
    }
    backToMap();
}

// P66: SAVE. FUEL 0x4ceaf0 (autosave off -> back on with 0xbb TT_AUTO_SAVE_ENABLED first) -> 0xbf save -> 0xc0
// 0x47f500: error 0 -> 0xc1 TT_SAVE_COMPLETED_SUCCESSFULLY (OK 0x4ce980 -> 0x53), else 0x53.
// PK25: autosave can be off now (the corrupt-save "No"): 0x4ceaf0 clears +0x75 and shows 0xbb, whose OK (0x4ce8a0) goes
// to 0xbf; a failed save shows MCAlert (retry) before 0x47f500 decides.
void Menu::dataSave() {
    if (storage_.autosaveOff) {
        setAutosaveOff(false);
        return openOk("TT_AUTO_SAVE_ENABLED", [this] { dataSaveNow(); });
    }
    dataSaveNow();
}

void Menu::dataSaveNow() {
    auto backToMap = [this] { mapData_ = false, focus_ = 0, setTitle("TT_TOP_TITLE_MAP"); };
    auto after = [this, backToMap] {
        if (storage_.saveErr == 0) return openOk("TT_SAVE_COMPLETED_SUCCESSFULLY", backToMap);
        backToMap();
    };
    saveCareer();
    if (storage_.saveAlert) return void(afterAlert_ = after);
    after();
}

void Menu::confirmKey(MenuKey k) {  // 0x4cdfa0
    if (confirm_.okOnly) {  // P66: 0x4ce400: the action key runs the OK function
        if (k != MenuKey::Accept) return;
        Confirm c = std::move(confirm_);
        confirm_ = {};
        if (c.yes) c.yes();
        return;
    }
    if (k == MenuKey::Left || k == MenuKey::Right || k == MenuKey::Up || k == MenuKey::Down) {  // 0x4c9330 / 0x4c93c0 [Up/Down: port]
        confirm_.focus = 1 - confirm_.focus;
        return;
    }
    if (k != MenuKey::Accept) return;  // no Back test in 0x4cdfa0
    Confirm c = std::move(confirm_);
    confirm_ = {};
    if (c.focus == 0 && c.yes) c.yes();
    if (c.focus == 1 && c.no) c.no();
}

void Menu::drawConfirm(UiRenderer& r) {
    Page* help = scripts_.page("PopUpHelp");
    Page* yn = scripts_.page("PopUpInput_YesNo");
    if (!help || !yn) return;
    // 0x46b2e0: both text list boxes and ITEM_ID_BUTTONS select the key; FOND_1 and POPUP_TEXTS shown, FOND_2 and
    // POPUP_TEXTS_2 hidden (the online chat texts swap them); 0x4ce260 hides ITEM_ID_BUTTONS and moves the Yes/No page to
    // the ITEM_ID_POPUPHELP_BOX row.
    float dy = 396;
    for (Item& it : help->items) {
        if (it.kind == ItemKind::ListBoxTT)
            for (size_t i = 0; i < it.entries.size(); ++i)
                if (it.entries[i].key == confirm_.key) it.entryIndex = int(i);
        if (it.id == "ITEM_ID_FOND_2" || it.id == "ITEM_ID_POPUP_TEXTS_2" || it.id == "ITEM_ID_BUTTONS") it.visible = false;
        if (it.id == "ITEM_ID_POPUPHELP_BOX") dy = it.y;
    }
    drawPage(r, *help);
    if (confirm_.okOnly) {  // P66: 0x4ce460: PopUpInput_Continue at the ITEM_ID_POPUPHELP_BOX row
        if (Page* ok = scripts_.page("PopUpInput_Continue")) {
            for (Item& it : ok->items) it.focus = true, it.enabled = true;
            drawPage(r, *ok, 0, dy);
        }
        return;
    }
    for (Item& it : yn->items) {
        if (it.id == "ITEM_ID_YES_LABEL") it.focus = confirm_.focus == 0, it.enabled = true;
        if (it.id == "ITEM_ID_NO_LABEL") it.focus = confirm_.focus == 1, it.enabled = true;
    }
    drawPage(r, *yn, 0, dy);
}

// 0x491d90 -> 0x4947c0 (display) / 0x491db0 -> 0x494950 (audio): the page opens on row 0 with the current values kept.
void Menu::optionsOpen(Screen) {
    optSnap_.options = options_;
    optSnap_.volume = settings_.volume;
    optSnap_.postEffects = settings_.postEffects, optSnap_.grass = settings_.grass, optSnap_.fullscreen = settings_.fullscreen;
    optSnap_.width = settings_.width, optSnap_.height = settings_.height, optSnap_.port = port_;  // P101 (0x4946c0: +0x42c..+0x42e)
    scroll_ = ListScroll{};  // pad-ux: a scrolled page opens at its top
}

// Leaving an options page: keep (the action key, 0x492b70 / 0x492f20: +0x80 = 1 -> saved) or restore the values from the
// page's open (0x495b70 display, 0x495a50 audio), then the options menu (state 6).
void Menu::optionsLeave(bool keep) {
    if (!keep) {
        const uint32_t seen = options_.tutorialsSeen;
        options_ = optSnap_.options;
        options_.tutorialsSeen = seen;
        settings_.volume = optSnap_.volume;
        settings_.postEffects = optSnap_.postEffects, settings_.grass = optSnap_.grass, settings_.fullscreen = optSnap_.fullscreen;
        settings_.width = optSnap_.width, settings_.height = optSnap_.height;
        // P101: 0x495c40 (controller page "Yes"): +0x272 from +0x42c with the manager switched to match (0x557c90 /
        // 0x557d80, no test pulse), +0x271 / +0x2a0 from +0x42d / +0x42e.
        port_ = optSnap_.port;
        applyDevMode();  // [dev mode] the overlay follows the restored value
        ++optSerial_;
    } else {
        saveOptions();
        storeOptionsInCareer();
    }
    const bool ctrl = screen_ == Screen::OptController;
    goTab(TabOptions);
    if (ctrl) ctrlMenu_ = true, focus_ = 0, setTitle("TT_TOP_TITLE_OPTION_CONTROLLER");  // P101: 0x492670 / 0x492f00 -> state 0x29
}

// FUEL keeps these options in the profile: game +0x33f0 race info, +0x33f1 ranking, +0x33f3 GPS, +0x33f4 speed meter,
// +0x33f5 (= +0x330c) km/miles (0x495390), all in SAVESTRUCT_INFOS' 0xb8-byte block at +0x38..+0x3d; +0x24c brightness
// (float 0.5 + 1.5 p, 0x495540), +0x254 music and +0x258 SFX volume (int 0..16, 0x554040 / 0x5540e0) in its options block.
// The port's settings.txt stays the source; career.sav gets the same values where FUEL keeps them.
void Menu::storeOptionsInCareer() {
    if (!career_) return;
    CareerState& cs = career_->state();
    cs.at<uint8_t>(0x38) = options_.raceInfo, cs.at<uint8_t>(0x39) = options_.rankingInfo;
    cs.at<uint8_t>(0x3b) = options_.gps, cs.at<uint8_t>(0x3c) = options_.speedMeter;
    cs.at<uint8_t>(0x3d) = options_.miles;  // [I: true = miles]
    const float b = 0.5f + 1.5f * options_.brightness;
    const int32_t music = int32_t(std::lround(options_.musicVolume * 16.0f)), sfx = int32_t(std::lround(settings_.volume * 16.0f));
    std::memcpy(cs.options + 0x0, &b, 4);
    std::memcpy(cs.options + 0x8, &music, 4);
    std::memcpy(cs.options + 0xc, &sfx, 4);
    // P101: the controller page's values in the same blocks (save 0x38 bytes from game +0x24c, then u32 +0x2a0):
    // +0x271 camera X inverted = options +0x25, +0x272 vibration = options +0x26, +0x2a0 camera Y inverted.
    cs.options[0x25] = port_.invertX, cs.options[0x26] = port_.vibration;
    cs.options2a0 = port_.invertY;
    markDirty();  // PK17: the options pages' +0x80 = 1 (0x492670 / 0x492ab0 / 0x492b70 / 0x492f20)
}

// P62: the garage vehicle selector. FUEL: pages GearSelecteur (band) and GearSelecteurCache (arrows, selection square,
// reference rectangles); 0x473ea0 lays the entries out around ITEM_ID_VEHICLE_REF_ITEM (579, 499, 128 x 128): spacing
// 0.9 w, centre = ref centre - focus x spacing, focused entry 1.1 x size centred, the others 0.7 x lifted to 0.35 h,
// shown within 1.5 x (view width - ref centre) of the centre and faded by distance (colour x (1 - d / range)); arrows
// (0x47b330): left when the focus > 0, right when it is not the last.
// P66: the pictures are packed images, not renders: 0x47a7d0 fills entry +4 with crc(vehicle record +0x4c), the 5th
// argument of AddVehicleInfo ("VEH_QUAD_ROAD_2"), colour 0.5 (neutral) and +0x1c = owned; 0x60abf0 draws the
// MaterialAnim of that crc (0x6093d0); a vehicle not owned is drawn black (0x473ea0) [O]. The list holds unlocked
// vehicles only (record +0 set) [O].
void Menu::drawVehicleSelector(UiRenderer& r) {
    if (Page* g = scripts_.page("GearSelecteur")) drawPage(r, *g);
    Page* gc = scripts_.page("GearSelecteurCache");
    const auto list = vehicleChoices();
    if (!gc || list.empty()) return;
    for (Item& it : gc->items) {
        if (it.id == "ITEM_ID_FLECHE_GAUCHE") it.visible = focus_ > 0;
        else if (it.id == "ITEM_ID_FLECHE_DROITE") it.visible = focus_ < int(list.size()) - 1;
        else if (it.id == "ITEM_ID_GEAR_CARRE_SELECTION") it.visible = true;  // 0x473c70.. [I: shown on the focus]
        else continue;
        drawItem(r, it, 0, 0);
    }
    std::vector<GarageEntry> entries;
    for (const VehicleInfo* v : list) {
        GarageEntry e;
        e.layers.push_back({v->picture, Rgba{1, 1, 1, 1}, 0});
        e.owned = !career_ || career_->vehicleOwned(v->name);
        entries.push_back(e);
    }
    drawCarousel(r, entries, focus_, "ITEM_ID_VEHICLE_REF_ITEM", 0.9f, 0);
}

// Menu::drawHud and the HUD pages: hud.mm (p40).

// P56: page end_race_infos (left panel of the ranking screen), filled by 0x4b2b20 (offline branch) [O]:
//  - ITEM_ID_RACE_NAME = the mission's name text (0x577450 -> 0x677d20);
//  - ITEM_ID_RACE_TYPE_DYN = a text by the effective game mode (0x4fe2d0), jump tables 0x4b3330 (the mission is in
//    a career list, 0x577d70 == 1) / 0x4b3364 (others); ITEM_ID_RACE_TYPE_INFO (the online list box) hidden;
//  - ITEM_ID_ENTRANTS_INFO = mission nb IA + 1 (0x46c900, TT 1304 "%d");
//  - ITEM_ID_GPS_INFO entry = game+0x33f3 (the 3-D GPS preference: TT_NO / TT_YES);
//  - ITEM_ID_LOCATION / _INFO = the hub's name (0x482320) when the mission has a hub, else both hidden;
//  - ITEM_ID_VEHICLE_INFO = the player's vehicle name (0x422e80 -> 0x677d20);
//  - ITEM_ID_WORLD_RECORD_INFO = entry TT_NOT_AVAILABLE (0x613730; online records are not ported either).
void Menu::fillEndRaceInfos() {
    Page* p = scripts_.page("end_race_infos");
    if (!p) return;
    const Event* ev = nullptr;
    for (const Event& e : events_)
        if (e.id == sessionEvent_) ev = &e;
    auto dyn = [&](const char* id, const std::string& s) {
        if (Item* it = p->find(id)) it->hasDynText = true, it->dynText = s;
    };
    dyn("ITEM_ID_RACE_NAME", ev && ev->nameText ? textOf(ev->nameText) : sessionEvent_);
    const int mode = ev ? ev->race.gameMode() : -1;
    const bool careerList = ev && ev->kind == Event::Career;
    uint32_t tt = 0;
    switch (mode) {
        case 0: case 1: tt = careerList ? 6003 : 6000; break;
        case 2: tt = 6001; break;
        case 3: tt = 6002; break;
        case 4: tt = 6003; break;
        case 5: tt = 6004; break;
        case 6: tt = 6005; break;
        case 7: tt = 6006; break;
        case 8: tt = 6007; break;
        case 9: tt = 6008; break;
        case 10: tt = careerList ? 6003 : 6009; break;
        case 11: tt = careerList ? 6008 : 6010; break;
        case 12: tt = careerList ? 6003 : 6012; break;
        default: break;
    }
    dyn("ITEM_ID_RACE_TYPE_DYN", tt ? textOf(tt) : "");
    if (Item* it = p->find("ITEM_ID_RACE_TYPE_INFO")) it->visible = false;
    dyn("ITEM_ID_ENTRANTS_INFO", std::to_string(ev ? ev->race.aiCount + 1 : 1));
    if (Item* it = p->find("ITEM_ID_GPS_INFO")) it->entryIndex = options_.gps ? 1 : 0;  // port: the GPS option
    std::string hubName;
    if (ev)
        for (const Hub& h : hubs_)
            if (upper(h.id) == upper(ev->hub)) hubName = h.nameText ? textOf(h.nameText) : h.id;
    dyn("ITEM_ID_LOCATION_INFO", hubName);
    for (const char* id : {"ITEM_ID_LOCATION", "ITEM_ID_LOCATION_INFO"})
        if (Item* it = p->find(id)) it->visible = !hubName.empty();
    dyn("ITEM_ID_VEHICLE_INFO", vehicleName(sessionVehicle_.empty() ? vehicle_ : sessionVehicle_));
    if (Item* it = p->find("ITEM_ID_WORLD_RECORD_INFO")) it->entryIndex = 1;  // TT_NOT_AVAILABLE
    // The captions (RACE INFO, Race type, Max Entrants, Vehicle, GPS, Location, World Record) are LabelDyn items no code
    // writes: they show their own TT.
    for (Item& it : p->items)
        if (it.kind == ItemKind::LabelDyn && !it.hasDynText && it.tt != 1309 && it.tt != 1304 && it.tt != 1369)
            it.hasDynText = true, it.dynText = textOf(it.tt);
}

}  // namespace fuel::ui

namespace fuel::ui {

std::vector<ResultRow> raceResultRows(const std::vector<int>& pos, const std::vector<std::string>& names, const std::vector<double>& finished,
                                      const std::vector<int>* nameTT) {
    std::vector<ResultRow> rows;
    for (int p = 1; p <= int(pos.size()); ++p)
        for (size_t i = 0; i < pos.size(); ++i)
            if (pos[i] == p)
                rows.push_back({i < names.size() ? names[i] : std::string(), float(std::max(0.0, i < finished.size() ? finished[i] : -1.0)), i == 0,
                                nameTT && i < nameTT->size() ? (*nameTT)[i] : 0});
    return rows;
}

// P99: --career-audit views.
std::string Menu::screenName() const {
    static const char* const kScreen[] = {"Title", "Tab", "CareerList", "ChallengeList", "CampList", "VehicleList", "OptDisplay", "OptAudio",
                                          "Results", "Reward", "Single", "GarageSelect", "Profile", "EndRace", "Credits", "Unlock",
                                          "WhatNext", "BonusCodes", "BonusKeyboard", "BonusResult", "OptGraphics", "OptController", "OptRemap", "RacePage", "FuelReward",
                                          "GarageUpgrades"};
    const size_t s = size_t(screen_);
    std::string n = s < sizeof kScreen / sizeof kScreen[0] ? kScreen[s] : "?";
    if (screen_ == Screen::Tab) n += ":" + std::to_string(tab_);
    return n;
}

std::vector<std::string> Menu::leftMenuIds() {
    std::vector<std::string> out;
    for (const Item* it : focusables()) out.push_back(it->id);
    return out;
}

std::vector<std::string> Menu::listRows() const {
    std::vector<std::string> out;
    switch (screen_) {
        case Screen::CareerList:
        case Screen::ChallengeList:
            for (const Event* e : hubEvents(screen_ == Screen::CareerList ? Event::Career : Event::Challenge)) out.push_back(e->id);
            break;
        case Screen::VehicleList:
            for (const VehicleInfo* v : vehicleChoices()) out.push_back(v->name);
            break;
        case Screen::CampList:
            for (const Hub& h : hubs_) out.push_back(h.id);
            break;
        default: break;
    }
    return out;
}

}  // namespace fuel::ui
