// FUEL's in-game menu, driven by the original pages (menu_model.hpp): the top bar
// (TopMenu, tache_tout_menu, TopTitle), the left menus (Race, Garage, Map, Options,
// TemplateLeftMenu), the list pages (HubSelectCamps, HubSelectCareer,
// HubSelectChallenges), the options pages, the race ranking and the HUD pages
// (compteur, classement, boussole). What each item does is game code in FUEL.exe;
// the flows here follow the item ids and are the port's (see notes/p19-menus.md).
#pragma once

#include "core/career.hpp"
#include "core/custom_rider.hpp"  // PHYSIQUE
#include "core/race_editor.hpp"  // PK21
#include "core/events.hpp"
#include "core/hubs.hpp"
#include "core/settings.hpp"
#include "ui/menu_assets.hpp"
#include "ui/menu_model.hpp"
#include "ui/hud_math.hpp"
#include "ui/gps_guide.hpp"
#include "ui/port_options.hpp"
#include "ui/alerts.h"  // PK25
#include "core/input_map.hpp"  // [port] remap
#include "core/remap.hpp"      // [port] remap

#include <array>
#include <functional>
#include <map>
#include <set>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace fuel {
class RoadGraph;
struct RoadTypeParams;
class FreeRoam;
struct HubZones;
}  // namespace fuel

namespace fuel::ui {

class UiRenderer;

enum class MenuMode { Title, Main, Pause, Results };
enum class MenuKey { Up, Down, Left, Right, Accept, Back, TabLeft, TabRight,
                     Vali2 /* P78: MENU_VALI2 (game+0x238; PC key R, pad X) */,
                     Reset /* [port] remap: MENU_RESET (game+0x234), read by the remap page only (ui/menu_input.h) */,
                     // P44 map page: FUEL's action key (game+0x208 -> 0x482b60), GPS key (+0x20c -> 0x484580), cursor, zoom
                     MapAction, MapTarget, MapUp, MapDown, MapLeft, MapRight, MapZoomIn, MapZoomOut };

struct MenuRequest {
    enum Kind { None, StartEvent, FreeRide, Resume, Restart, QuitToTitle, QuitGame, Teleport };
    Kind kind = None;
    std::string event, hub, vehicle;
    float x = 0, z = 0;  // Teleport (0x4caa90): the map item the player flies / drives to
    bool at = false;     // P78: FreeRide from (x, z) instead of the hub's camp (EndRace_WhatNext2 "Go free ride")
    bool noArrival = false;  // the race tab's GO FREE RIDE (0x49b190) is no TeleportToHub: no hub arrival / camp menu
    bool careerBoot = false; // PK26: TRCMessage's Continue (0x594a50): the career world's first load (Dialog_Loading 0x58fc20)
};

struct ResultRow {
    std::string name;
    float time = 0;     // seconds, 0 = did not finish
    bool player = false;
    int nameTT = 0;     // PK16 0x620c20: an opponent's name text id (TT 1500..1573); 0: the vehicle's name
};
// The race order for the results page: racer i (0 = the player) at 1-based `positions[i]`, finished at clock
// `finished[i]` (< 0: not finished). Shared by the game (--in-game) and --career-audit (P99).
std::vector<ResultRow> raceResultRows(const std::vector<int>& positions, const std::vector<std::string>& names, const std::vector<double>& finished,
                                       const std::vector<int>* nameTT = nullptr);

// HUD values for one frame (FUEL's HUD: hud.mm, notes/p40-hud.md).
struct HudState {
    float speedKmh = 0;
    float odometerKm = 0;
    std::string hubName;  // radar caption
    bool race = false;
    // minimap-fix: the radar banner 0x4a54b0 runs in races too. Its gate 0x4a53e0: the teleport machine idle (IGP +0x908 =
    // LevelManipulator_G, +0x5c state 0; Teleport_Queue 0x57e650 sets 1) and game +0xdc (GAMEStarted); its status line reads
    // the current mission's state +0x48 (0 none, 3 STREAMING, 4 PREPARING, 5 RUNNING, 6 ENDING_WON, 7 ENDING_LOST).
    int missionState = 0;
    int missionList = 3;  // 0x577d70 of the mission: 0 tutorial, 1 career, 2 challenge, 3 in no hub list (Menu::missionList)
    bool teleporting = false;
    bool gameStarted = true;
    bool outOfArea = false;  // no hub zone holds the player: uid -1, hubName = TT 998 "OUT OF AREA", the fade restarts each frame
    int position = 0, racers = 0;
    std::vector<ResultRow> standings;  // current order (time = remaining metres for the ranking lines)
    // p40: speedometer (MeterHUD)
    float speedMs = 0;       // |body velocity| (0x4222f0 before its x 1.15)
    float rpm = 0, maxRpm = 0;  // display rpm, VehiclePP 324
    float damage = 0;        // 0..1: FUEL's damage record 1, largest slot (0x4235b0, P46)
    // Player and opponents for the minimap (world x, z; quaternion x, y, z, w of the body).
    float px = 0, pz = 0;
    std::array<float, 4> quat{0, 0, 0, 1};
    struct Blip {
        float x, z, fx, fz;  // position, forward
    };
    std::vector<Blip> opponents;
    // P78: the special traffic vehicles (AddHubTrafficVehicle entries, 0x5c57d0) for the minimap's doppler / bonus-car
    // icons (0x4ac1a0): where each is (its vehicle when it drives, else its box centre [port]), kind 0..3, hub, entry.
    struct Special {
        float x, z;
        int kind, hub, index;
        bool live = false;  // PK19: a traffic slot drives it (0x5c5690's active list), (x, z) = its vehicle
    };
    std::vector<Special> specials;
    // Race info (MissionHUD)
    hud::InfoPage page = hud::InfoPage::None;
    int topA = 0, topB = 0, bottomA = 0, bottomB = 0;
    float distance = 0;      // remaining metres
    float time = 0;          // race time, or time left on time-limit pages
    float timeTotal = 0;
    float countdown = -10;   // c of 0x4b3da0 (see hud::countdown)
    std::vector<std::array<float, 2>> raceLine;  // the race's path (minimap race line, 0x4a1bd0)
    std::vector<std::array<float, 2>> gates;     // gates in order, the last one = finish
    size_t nextGate = 0;
    std::vector<float> gateY;                    // P56: gate heights (Navigator3dHUD band icons project in 3-D)
    float py = 0;                                // P56: player height
    bool helico = false;                         // P56: helicopter race (scenario 0xb): no band icons (0x4c0ed0)
    bool gps3d = true;                           // P56: the 3-D GPS shown ([0xa6536e] == 0): ribbons, compass, icons
    std::vector<std::array<float, 3>> gps;       // GPS samples (0x4a2250): <= 30, 10 m apart
    float dt = 0;
    bool settle = false;     // port (film shots): smoothed values start at their targets
    // In-world GPS ribbons (Navigator3dHUD) and the camera that projects them onto the 1280 x 720 canvas.
    std::vector<hud::GpsGuide::Ribbon> ribbons;
    std::array<float, 3> eye{}, camF{0, 0, 1}, camR{1, 0, 0}, camU{0, 1, 0};
    float camFy = 0;         // canvas px per unit of tan (360 / tan(fovY / 2)); 0 = no projection
    // [nitro] the player's tank for the gauge beside the speedometer (ui/nitro_gauge.mm); on = false: no gauge.
    struct Nitro {
        bool on = false;
        float level = 0, amount = 0;  // 0..1
        bool active = false;
        float emptyFlash = 0, bonusGlow = 0;  // s left
        // [upgrades U1] the vehicle's tank seconds (tick marks) and the level a boost needs; 0 = nitro::params()'s.
        float tankSeconds = 0, minStartLevel = 0;
    } nitro;
};

// The port's own options kept beside fuel::Settings in the same file (unknown keys
// survive saveSettings).
struct MenuOptions {
    bool raceInfo = true, rankingInfo = true, gps = true, speedMeter = true, miles = false;
    float brightness = 1.0f / 3.0f, musicVolume = kFuelDefaultVolume;  // P103: music n / 16, FUEL's default 8; brightness = slider p: G = 0.5 + 1.5 p (0x495540); default G 1 (U2)
    uint32_t tutorialsSeen = 0;  // P56 (read only since P62: the flags moved to career.sav SECTION_TUTORIAL; old masks migrate)
};
MenuOptions loadMenuOptions(const std::string& path = settingsPath());
bool saveMenuOptions(const MenuOptions& o, const std::string& path = settingsPath());

// [port] DEVIATION from FUEL (user request 2026-11-06, notes ~/re-work/fuel/notes/pad-ux.md): the port's option pages
// that are longer than their frame (GRAPHICS, CONTROLS > SETTINGS; later the remap list) scroll. A window of `visible`
// rows over `count`; it moves by FUEL's ranking rule (0x4b1aa0 / 0x4b1c50: the list scrolls only when the selected row
// would leave the window), so the selection always stays visible; the wheel moves the window and keeps the selection
// inside it. FUEL's own scroll arrows (FLECHEUP / FLECHEDOWN of PageOptionsControllerConfigChange, FUEL's long controls
// list) show while rows lie above / below.
struct ListScroll {
    int first = 0, visible = 1, count = 0;
    void set(int n, int vis) {
        count = n, visible = vis < 1 ? 1 : vis;
        clamp();
    }
    int maxFirst() const { return count > visible ? count - visible : 0; }
    void clamp() { first = first < 0 ? 0 : first > maxFirst() ? maxFirst() : first; }
    void follow(int sel) {
        if (sel < first) first = sel;
        if (sel >= first + visible) first = sel - visible + 1;
        clamp();
    }
    // The window moves `rows` (clamped); `sel` is pulled inside it. Returns the rows it moved.
    int scroll(int rows, int& sel) {
        const int was = first;
        first += rows;
        clamp();
        const int last = (first + visible < count ? first + visible : count) - 1;
        sel = sel < first ? first : sel > last ? last : sel;
        return first - was;
    }
    bool moreUp() const { return first > 0; }
    bool moreDown() const { return first + visible < count; }
};

class Menu {
public:
    Menu(const std::string& dataDir, const Settings& settings, MenuMode mode);
    ~Menu();

    const MenuScripts& scripts() const { return scripts_; }
    const MenuAssets& assets() const { return *assets_; }
    const TextTable& text() const { return text_; }
    const std::vector<Hub>& hubs() const { return hubs_; }
    const std::vector<Event>& events() const { return events_; }
    const std::vector<VehicleInfo>& vehicles() const { return vehicles_; }
    Settings& settings() { return settings_; }
    const MenuOptions& options() const { return options_; }
    // P101: the port's settings.txt keys the options pages edit (ui/port_options.hpp), the values as shown now (the
    // pages apply as the player changes them, as FUEL's 0x495390 / 0x5540e0 do), and a counter that moves on every
    // change or restore so a session can apply them live (main.mm, game_menu.mm).
    const PortOptions& portOptions() const { return port_; }
    int optionsSerial() const { return optSerial_; }
    Settings liveSettings() const;
    // ---- [port] remap (remap_page.mm; notes/remap.md): the bindings the menus read (ui/menu_input.h) and the remap page.
    // bindings(): the live map (settings + the page's key.* / pad.* lines), reloaded when the options change.
    const fuel::input::InputMap& bindings();
    bool remapPageUp() const { return screen_ == Screen::OptRemap; }
    // Hosts give the page the raw keys first (a capture takes them; `repeat`: a held key's auto-repeat, ignored) and the
    // pad state every frame; true = taken (the host routes nothing more this time).
    bool rawKey(unsigned short macKey, bool repeat);
    bool padFrame(const fuel::input::PadState& pad, float dt);
    bool remapCapturing() const { return remap_.capture; }
    // The menu keys of a keyboard key / the pad with the bindings above (menu_input.h).
    std::vector<MenuKey> menuKeys(unsigned short macKey);
    // ---- end remap
    // P101: VIBRATION switched ON on the controller page asks for FUEL's test pulse (0x495670: IMPACT_MEDIUM, priority 2,
    // cooldown cleared): the count of such requests so far.
    int vibrationTests() const { return vibTests_; }
    // P101: the settings the running process started with (what "takes effect next time" compares against).
    void setStartSettings(const Settings& s) { startSettings_ = s; }

    // The session the game runs (pause and results menus).
    void setSession(const std::string& event, const std::string& hub, const std::string& vehicle);
    // PK11: game+0x32bc (0 game, 1 demo preview, 2 tutorial mode; none of the shipped game's flows sets 1 or 2 outside
    // the retail demo / tutorial builds). 0x47ca70 disables CHALLENGES in mode 1, 0x481f80 CAMPS and DATA in mode 2.
    void setGameMode(int m) { gameMode_ = m; }
    int gameMode() const { return gameMode_; }  // PK17: World_IsOutside 0x5280a0 reads game +0x32bc
    // PK11: the state of a hub menu item (0x47ca70 / 0x481f80) and the help-line key under it (0x47cbf0, PK18's
    // BandeauHelp): "ITEM_ID_LEFT_MENU5" career, "4" challenges, "3" camps, "2" data.
    bool hubItemEnabled(const std::string& id) const;
    std::string hubHelpKey() const;
    // ---- PK18: pause-menu chrome (menu_chrome.mm; notes/pk18.md) ----
    // The keys the pause HUD hands to TopTitleDyn / BandeauHelp for the screen shown (0x46b8f0: dyn text, TopTitle key, button
    // legend key, help-line key; every key is a TT_ id of the page's list entries, "" = the item is hidden).
    struct Chrome {
        bool hasDyn = false;  // TopTitleDyn shows `dyn` (upper-cased) and TopTitle is hidden
        std::string dyn, title, buttons, help;
    };
    Chrome chrome() const;
    // 0x46d380: the menu sound picked for what happened since the last call (index 0..11 of MenuSfx, -1 none); the host plays it.
    int takeSound();
    // 0x46f080 / 0x46f250: whether top-bar tab `tab` (TabLast .. TabOnline) is enabled now.
    bool tabEnabled(int tab) const { return tabAvailable(tab); }
    // --menu-shot: lets the help line's marquee run `seconds` before the frame is drawn.
    void previewAdvance(float seconds) { previewBoost_ = seconds; }
    // 0x46e650: 1 outside a hub camp, 2 inside one, 3 a race is running (4 online: not in the port).
    int menuMode() const;
    // PK11: the hub menu camera (View 13, 0x40e4e0): hub-record offsets from the view state +0x15c (0x578f20 / 0x578f60).
    struct HubCameraSlot { int pos, target; const char* name; };
    static HubCameraSlot hubCameraSlot(int view);
    // Race results. `won` = RaceRules::Won, FUEL's mission state 6 (the end-condition list 0x62cff0: a race is won by
    // finishing 1st). `level` = the race's level (race +0x3e8, the session's --ai-level: 0 gold, 1 silver, 2 bronze); a
    // won career race gets that medal (0x630dd0); -1 = the row the career page would preselect. Applies the career award
    // and saves career.sav.
    void setResults(const std::vector<ResultRow>& rows, bool won = false, int level = -1);
    Career* career() { return career_.get(); }
    const Career* career() const { return career_.get(); }
    const std::string& sessionVehicle() const { return sessionVehicle_.empty() ? vehicle_ : sessionVehicle_; }
    const CareerAward& lastAward() const { return award_; }
    void setMode(MenuMode m);
    MenuMode mode() const { return mode_; }
    // P75: InGamePauseHUD vf22 0x46e750: INFOS (0x4443ea) opens the in-game menu on tab 2 = Map, PAUSE (0x444384) on tab 0.
    void openMapTab() { goTab(TabMap); }
    // ---- PK03: TT_PAGE_HUB_ALERT (Map state 0xc7, 0x47da10) over the menu; OK 0x4ce850 -> 0x47f060: close and TLP
    // (0x4caa90, the map teleport) to the locked hub's END1 (x, z).
    void openHubAlert(float x, float z) { openOk("TT_PAGE_HUB_ALERT", [this, x, z] { request_ = {MenuRequest::Teleport, "", "", "", x, z}; }); }
    // ---- end PK03
    // P75: the challenge-arch start (GeneratedOfflineVisibleObject vf03 0x4f6fd0, MissionHUD +0xd6c state 1): in free
    // roam, an available race whose start is within the arch radius shows page PressStart (TT 433, 0x4b86a0); the
    // action key opens the race's page (0x481860) = the port's vehicle choice for it.
    int archEvent() const { return arch_; }
    bool archAccept();
    // PK08: MessagesHUD 0x4af920: the action key during a banner's hold cuts it to its fade-out. True when it did.
    bool bannerAccept();
    Page* messagePage(int kind, int param);  // the banner page of a message kind, texts / images filled (audit)
    int rankSelected() const { return rankSel_; }
    std::string rankName(int row) const;
    const VehicleInfo* vehicleInfoPublic(const std::string& l) const { return vehicleInfo(l); }
    int rankRows() const { return int(results_.size()); }
    bool rowIsPlayer(int i) const { return i >= 0 && i < int(results_.size()) && results_[size_t(i)].player; }
    int rankIndex(int row) const;
    float mapPulse() const { return mapPulse_; }  // the name shown on ranking row 1..5 ("" none)
    // The end-race pages' filled items (audit / shots): page 121's FUEL_WON / FUEL_TOTAL, the unlocked-hub map's two pictures.
    std::pair<std::string, std::string> fuelRewardTexts();
    // medals: race_reward (0x4b4220): bars 1..3 (gold, silver, bronze) icon ("" hidden), FUEL_REWARD1..3, FUEL_REWARD4, NBSTARS.
    struct RaceRewardItems { std::array<std::string, 3> slot, fuel; std::string fuelWon, stars; };
    RaceRewardItems raceRewardItems();
    std::array<std::string, 2> unlockMapPictures(float* alpha2);
    int rankHighlight() const { return rankRow_; }

    // Shows the screen built around a page name (--menu-shot): TopMenu tabs ("Race",
    // "Garage", "Map", "Options", ...), list pages, options pages, PressStart,
    // race_ranking_win, compteur/classement/boussole (HUD) or any other page alone.
    bool showPage(const std::string& page);
    // PK26 0x594a50 + 0x58fc20: the session the boot screens end in: the saved hub (game+0x33c0, else hub 0) with the
    // career vehicle (game+0x33c8, else the default one), arriving by TeleportToHub.
    MenuRequest careerBootRequest() const;

    void key(MenuKey k);
    void keyUp(MenuKey k);  // P75: the map's held keys (pan / zoom are continuous in FUEL)
    // pad-ux [port]: the mouse wheel / trackpad, `rows` whole rows (> 0 down). The scrolled pages move their window
    // (ListScroll); every other screen takes it as Up / Down presses under FUEL's own rules, the paged lists (races,
    // challenges, camps, edited races) turning the page at their ends.
    void wheel(int rows);
    // pad-ux: the scroll window of the page shown (count 0 when the page does not scroll) - checks and shots.
    const ListScroll& listScroll() { return syncScroll(), scroll_; }
    void pointer(float x, float y, bool click);  // canvas coordinates
    void update(float dt);
    void draw(UiRenderer& r);
    // `vignette` false: the caller draws the VignettageHUD picture itself (P102: GameOverlay draws it even while the
    // HUDs are hidden, as 0x443290 does).
    void drawHud(UiRenderer& r, const HudState& hud, bool vignette = true);
    // The world's roads for the minimap (FUEL's StockRoads graph and road-type table).
    void setRoads(const fuel::RoadGraph* graph, const std::array<fuel::RoadTypeParams, 21>* types) { roads_ = graph, roadTypes_ = types; }
    // P75: the Map page's hub zones (GenWorld_Z tail, core/hub_zones.hpp).
    void setHubZones(const fuel::HubZones* z) { zones_ = z; }
    const fuel::RoadGraph* roads() const { return roads_; }
    MenuRequest take();

    // P44 free roam: the rules object (messages, POIs), the player for the map page (position and the minimap's
    // heading h), and the map's GPS target (MapHUD +0x60 / +0x64..+0x6c, set by 0x484580).
    void setFreeRoam(fuel::FreeRoam* f) { freeRoam_ = f; }
    void setPlayer(float x, float z, float hx, float hz) { map_.px = x, map_.pz = z, map_.phx = hx, map_.phz = hz, map_.havePlayer = true; }
    bool gpsTarget(float& x, float& z) const {
        x = map_.tx, z = map_.tz;
        return map_.target;
    }
    void clearGpsTarget() { map_.target = false; }
    void setGpsTarget(float x, float z) { map_.target = true, map_.tx = x, map_.tz = z; }

    // P56/P62: tutorials (tutorial.mm; manager 0x56e340 / 0x56e420, gate 0x56e620, page PageTutorial 0x56e8a0, OK 0x56e4c0).
    void queueTutorial(int index);
    // Per frame (0x56e340). `menuOpen`: the menus are up (the menu-side entries 11..15 show over them; the in-game ones
    // wait until they close).
    void updateTutorial(float dt, bool menuOpen);
    // ---- PK17: 0x56eb20 (PAUSE key 0x4442f0, after the pause page opens): the queued tutorials without the immediate flag
    // (+0x18 == 0) are dropped (0x56ed30), back to front. setRtcFreeze: 0x534b40(2) - a freeze-player RTC runs; the
    // tutorial gate 0x56e620 refuses while it does.
    void pauseDropTutorials();
    void setRtcFreeze(bool on) { rtcFreeze_ = on; }
    // ---- end PK17
    bool tutorialOpen() const { return tutorial_.shown >= 0; }
    void tutorialKey(MenuKey k);                  // OK: next text page, then the next queued entry or close
    void drawTutorial(UiRenderer& r);
    bool showTutorial(int index);                 // --menu-shot PageTutorial<N>

    std::string textOf(uint32_t tt) const;
    // A vehicle variant label ("QUAD_ROAD_2") as the game names it ("Speed Demon").
    std::string vehicleName(const std::string& label) const;

    // ---- PK20: the 3-D garage preview (notes/pk20.md). The garage page's side of it (InGamePauseHUD: the hovered item of
    // the garage list 0x478070, the delays +0xec / +0x2d0, the restart 0x478ec0, the ticks at the end of the key handler
    // 0x478bb0) runs in garage3DUpdate and queues FUEL's calls into the two preview objects, in order; the viewer
    // (viewer/garage_preview.mm) runs them on its InGame3DVehicle (+0xf0) / InGame3DPerso (+0x214) and draws their
    // InGameObjects_Z scenes. Hosts that draw the preview call garage3DUpdate once per frame after update(); nothing is
    // queued otherwise (headless checks).
    struct Garage3DCmd {
        enum Kind {
            Setup,        // garage enter 0x478400 / 0x478490 / 0x478520: 0x4781c0 -> 0x4d1020 (vehicle viewport, orbit
                          // vectors zeroed, FOV PP66) and 0x471010 (rider set-up 0x4d0720, mode 0, framing PP300-302,
                          // rider scene off, outfit 0x471570, vf01(0))
            Release,      // garage leave 0x4789e0: 0x4712f0 (rider vf00 0x4d0a00) and 0x4788d0 (vehicle vf00 0x4d10f0)
            Restart,      // 0x478ec0 (and its copy in the enter functions): both scenes off (0x436020, AL 0)
            LoadVehicle,  // 0x47bd20 when +0xec runs out: 0x4d1110(index), 0x4cfec0, livery request 0x5dd790(record +0xc,
                          // force 1), paint 0x44afb0(record +0xd) -> Vehicle_SetDirtAndColor 0x5dd980(dirt 0)
            ShowRider,    // 0x475370 when +0x2d0 runs out: the rider scene on (0x436020, AL 1)
            Livery,       // 0x4799c0: livery request with the browsed entry (+0x148 / +0x14c = 1, else back to +0x144)
            Paint,        // 0x479a00: Vehicle_SetDirtAndColor(0, entry colour x 2)
            Look,         // the rider outfit record (page +0x270 = InGame3DPerso +0x5c) changed: 0x471570 / 0x471f40 ..
            PoseMode,     // 0x47c1e0: InGame3DPerso +0x44 through 0x4d0b90 .. 0x4d0c10
            TickVehicle,  // 0x47bd20 tail: InGame3DVehicle vf01(dt) 0x4d0df0 while +0x128
            TickRider,    // 0x475370 tail: InGame3DPerso vf01(dt) 0x4d04d0
            Upgrades,     // [upgrades U3, DEVIATION] the stages the preview vehicle shows (`stages`, upg::Category order
                          // e,s,t,w,np,nt): its equipped ones, the UPGRADES page's browsed stage in the focused category;
                          // sent when they change and after every LoadVehicle (GaragePreview::setUpgradeStages)
        };
        Kind kind = Restart;
        float dt = 0;
        std::string vehicle;          // LoadVehicle: the variant label of the +0xc8 entry
        int livery = 0;               // LoadVehicle / Livery: texture livery, 0-based (record +0xc)
        std::array<float, 3> rgb{};   // LoadVehicle / Paint: 0x44afb0's colour (palette x 2, i.e. / 255)
        PilotLook look;               // Look
        fuel::RiderOutfit outfit;     // Setup / Look: PHYSIQUE (port deviation) the outfit the preview rider wears
        int mode = 0;                 // PoseMode
        std::array<int, 6> stages{};  // Upgrades [upgrades U3]
    };
    void garage3DUpdate(float dt, bool menuVisible);
    std::vector<Garage3DCmd> takeGarage3D() { return std::exchange(g3dCmds_, {}); }
    // ---- PK20 end

    // ---- [upgrades U3, DEVIATION] the garage's UPGRADES page (garage_upgrades.mm; notes upg3.md): views for checks and the audit.
    struct UpgradePip {
        enum State { Equipped, Owned, ForSale, TooExpensive, NeedsStars, VehicleNotOwned };
        State state = ForSale;
        int price = 0;  // fuel to own it (the missing lower stages included); 0 owned
        int stars = 0;  // stars it needs (stage 2: 20, stage 3: 45)
    };
    UpgradePip upgradePip(const std::string& vehicle, int category, int stage) const;
    int upgradeCategory() const { return upg_.cat; }  // the focused category row (upg::Category)
    int upgradeStage() const { return upg_.stage; }   // the stage cursor (0 stock .. 3)
    std::vector<int> upgradeRows() const;             // the category rows shown (nitrous off: no nitro rows)
    // The stages the 3-D preview shows for `vehicle` now (its equipped ones; on the UPGRADES page the browsed stage in the
    // focused category; stock when upgrades are off or the vehicle is not owned).
    std::array<int, 6> garagePreviewStages(const std::string& vehicle) const;
    const std::string& infoText() const { return infoText_; }  // the port text of an open garage info box ("" FUEL's own)
    // The page's price box as drawn: the hovered stage's name, its price / OWNED / EQUIPPED, YOUR_FUEL (the career's fuel).
    std::array<std::string, 3> upgradeBoxTexts() const;
    // ---- end U3

    // ---- P99: read-only views for --career-audit (career_audit.mm drives the menu with keys and checks what shows) ----
    std::string screenName() const;                // "Tab:2", "CareerList", "RacePage", "VehicleList", "Reward", "Unlock", "WhatNext" ...
    // PK02: what the race page shows (the audit reads these): the focused medal row, the fuel reward label of a challenge
    // page ("" while GROUP_FUEL_REWARD is hidden), whether GROUP_CHALLENGE_WARNING shows, and the page's name text.
    void setNoVehicleShot(bool b) { shotNoVehicle_ = b; }  // PK02 test hook: the vehicle list is empty (0x4813a0)
    int careerHubOf(const std::string& hubId) const { return career_ ? career_->rules().hubIndex(hubId) : -1; }
    int raceMedalRow() const { return raceRow_; }
    std::string raceFuelReward() const;
    bool raceWarning() const;
    std::string raceTitle() const;
    // PK02 test hook (--menu-shot / audit): the vehicle ids named like the player's list (0x47a7d0 with a mission filter).
    bool raceHasVehiclePublic(const std::string& event) const { const Event* e = eventById(event); return e && raceHasVehicle(*e); }
    std::vector<std::string> leftMenuIds();        // the focusable left-menu items, in focus order
    int focusIndex() const { return focus_; }
    int hubIndex() const { return hubIndex_; }
    std::vector<std::string> listRows() const;     // ids on the open list: events, vehicle labels, hub ids
    const std::string& infoBox() const { return infoBox_; }
    bool confirmOpen() const { return confirm_.open; }
    const std::string& confirmText() const { return confirm_.key; }
    std::string unlockPage() const { return screen_ == Screen::Unlock && !unlocks_.empty() ? unlocks_.front().first : std::string(); }
    const std::string& vehicle() const { return vehicle_; }
    const std::string& pendingEvent() const { return pendingEvent_; }
    int tutorialShown() const { return tutorial_.shown; }
    const std::vector<int>& tutorialQueue() const { return tutorial_.queue; }
    // The parts of drawHud that are not drawing: the player's place and the challenge-arch prompt (0x4f6fd0).
    void hudTick(const HudState& hud) {
        hudRace_ = hud.race, playerX_ = hud.px, playerZ_ = hud.pz;
        updateArch(hud);
    }
    // Profile: the General sub pages' values (0x497920 fills them as the page opens) and the Collection (state 0x1e)
    // / Completion (0x24) lines of type `sel`.
    struct ProfileRow {
        bool hub = false, enabled = false;
        uint32_t text = 0, text2 = 0;
        int rank = 3;
    };
    std::vector<ProfileRow> profileRows(int state, int sel);
    // Map: the selected item's type (6 camp, 7..12 races, 13..20 POIs, 21 heliport) and index, -1 none.
    std::pair<int, int> mapSelection() const;
    void setMapTarget(float x, float z) { map_.target = true, map_.tx = x, map_.tz = z; }  // PK19: --menu-shot hudfree (the GPS pin)
    std::string profileValue(const std::string& page, const std::string& id);

private:
    enum class Screen { Title, Tab, CareerList, ChallengeList, CampList, VehicleList, OptDisplay, OptAudio, Results, Reward, Single,
                        GarageSelect /* P66: livery, paint and driver carousels */, Profile /* P66: General / Collection / Completion */,
                        EndRace /* P75: the win / lose page before the ranking (0x4b7830 / 0x4b82c0) */,
                        Credits /* P75: Options > Credits (state 0x36, 0x4938d0) */,
                        Unlock /* P75: the end-race unlock pages (MissionHUD 0x13..0x1b, 0x4ba590) */,
                        WhatNext /* P78: EndRace_WhatNext2 (MissionHUD state 0x1c, draw 0x4b6f30, text 0x4b18c0) */,
                        BonusCodes, BonusKeyboard, BonusResult /* P78: Options > Bonus codes, states 0x33 / 0x34 / 0x35 */,
                        OptGraphics /* P101: the port's GRAPHICS page (options_pages.mm) */,
                        OptController /* P101: Options > Controller > Settings (state 0x2c, keys 0x492670) */,
                        OptRemap /* [port] remap: Options > Controls > Layout, FUEL's remap page (state 0x2d, 0x4cc870) */,
                        RacePage /* PK02: the career race page (state 0x56, HubSelectCareerMood) / challenge page (0x59,
                                    HubChallengesDescription) between the list and the vehicle pick */,
                        FuelReward /* PK08: EndRaceFuelChallengeReward (MissionHUD state 0x1a, draw 0x4b68b0, labels 0x4b7630) */,
                        GarageUpgrades /* [upgrades U3, DEVIATION] the garage's UPGRADES page (port state 0x1a0, garage_upgrades.mm) */ };
    enum Tab { TabLast, TabGarage, TabMap, TabProfile, TabOptions, TabEditor, TabOnline, TabCount };

    void goTab(int tab);
    void openList(Screen s);
    bool tabAvailable(int tab) const;
    bool tabHaloShown(int tab) const;  // PK18: 0x46f080 / 0x46f250 hide the halos of the disabled-by-rule tabs
    void applyTopBar();                // PK18: 0x46f080 + 0x4701a0
    void applyChrome();                // PK18: TopTitle / TopTitleDyn / BandeauHelp item states from chrome()
    void drawChrome(UiRenderer& r);    // PK18: TopTitleDyn + BandeauHelp pages (0x46b750, 0x46b270)
    std::string areaName() const;      // PK18: the hub name 0x47cc70 / 0x49b500 hand to TopTitleDyn
    std::string mapLegendKey() const;  // PK18: 0x4836d0's button legend of the map view
    void soundBit(unsigned b) { snd_ |= b; }
    unsigned snd_ = 0;                 // PK18: pause HUD +0x90 (the sound request bits)
    float previewBoost_ = 0, chromeClock_ = 0, marqueeDt_ = 0;
    std::string lastHelp_, lastHelpDyn_;
    uint32_t jitterSeed_ = 0x2545f491;
    int jitterRand();                  // 0x6462e0 in [-1, 1]
    Page* leftPage();
    std::vector<Item*> focusables();
    void accept(const std::string& id);
    void listMove(int d);
    void listPage(int d);
    void applyItemStates();
    void prepareRows();
    void drawPage(UiRenderer& r, Page& p, float dx = 0, float dy = 0);
    void drawItem(UiRenderer& r, const Item& it, float dx, float dy);
    static void itemRect(const Item& it, float& x, float& y);  // safe-frame snapping (0x49c0a0)
    void drawMeter(UiRenderer& r, const HudState& hud);
    void drawMissionInfo(UiRenderer& r, const HudState& hud);
    void drawRanking(UiRenderer& r, const HudState& hud);
    void drawMinimap(UiRenderer& r, const HudState& hud);
    void drawCountdown(UiRenderer& r, const HudState& hud);
    void drawNavigator(UiRenderer& r, const HudState& hud);  // P56: Navigator3dHUD band (nav_hud.mm, 0x4c0fe0)
    // PK19: the FX_NAVIGATOR3D_LINE beams of the band's icons (0x4bfc60 flag 1 -> 0x4bf820): world-space ribbons from each
    // icon's place on the ground up to its spot on the compass band, drawn by the renderer in the scene (nav_hud.mm).
public:
    struct NavBeam {
        std::array<std::array<float, 3>, 4> points;  // the ribbon's +0xb4 points, ground first
        std::array<float, 4> colour;                 // the half-float rgb (entry rgb x 8, +0x6a..) and alpha (+0x70)
        float halfWidth = 0;                          // +0x58 = 0.008 x the horizontal distance
    };
    std::vector<NavBeam> navBeams(const HudState& hud);
    void drawVignette(UiRenderer& r);                         // P56: VignettageHUD (0x4cc0f0); P102: the whole target
private:
    void drawMessages(UiRenderer& r);         // P44: MessagesHUD pages (0x4afe00)
    void fillEconomy();                       // P44: economy_hud (0x46d1e0)
    void fillEndRaceInfos();                  // P56: end_race_infos (0x4b2b20)
    // P44 map page (map_page.mm): items 0x4a9f50, selection 0x46a4d0, view 0x4ac8a0, keys 0x4827b0.
    struct MapItem {
        float x = 0, z = 0;
        int type = 0, index = 0, hub = -1;
        bool enabled = false;
        float size = 30;
    };
    std::vector<MapItem> mapItems() const;
    void drawMap(UiRenderer& r);       // Map_Cache, roads, items, cursor (and the selection)
    void drawMapInfo(UiRenderer& r);   // the info page of the selection (0x483270)
    void mapKey(MenuKey k);
    void mapUpdate(float dt);  // P75: held map keys, zoom easing
    void mapSelect();          // P99: the selection 0x46a4d0 (per frame)
    void mapOpen();
    // PK19 (map_page.mm): the hub whose zone holds (x, z) (0x4a7f40 -> 0x577500; CareerRules index, -1 none); 0x4838d0 (the
    // hub under the cursor -> +0x2d4, its title / picture / info text; `setZoom`: +0x2d8 = PP[321 + uid]); 0x4839e0 (the
    // selection's picture, sticky until the cursor moves); InGameMapHUD vf21 0x46a8f0 (zoomed out).
    int mapHubAt(float x, float z) const;
    void mapCursorHub(bool setZoom, bool browse);
    void mapHubPicture();
    void mapElementInfo();
    bool mapZoomedOut() const;
    void drawMapBase(UiRenderer& r, float x, float y, float w, float h, const hud::MapView& v, float cx, float cz, float zoom);
    void setTitle(const std::string& key);
    std::vector<const Event*> hubEvents(Event::Kind kind) const;
    std::vector<const VehicleInfo*> vehicleChoices() const;
    void saveOptions();
    void back();
    int careerHub() const;                   // CareerRules hub index of hubs_[hubIndex_] (-1 none)
    bool hubLocked() const;
    void applyHubStates();
    bool campShown(int idx) const;  // PK11
    bool hubOpened() const;  // PK11: hub +0x248 (first entry done)
    int gameMode_ = 0;       // PK11
    void vehicleAccept(const std::string& label);
    int damoclesLock(const std::string& label) const;  // PK02: 0x479130 (0 popup 0x99, 1 any other vehicle, 2 TTCsaltlake garage)
    bool startForced(const Event& e);  // P99: challenges with their own vehicle start without the garage (0x47efb0)
    // PK02: the race pages. openRace = 0x47d550 (career, state 0x56) / 0x47d660 (challenge, 0x59); raceKey = 0x47e310 /
    // 0x47ed60 (+ the keys 0x46bc60); drawRace = the draw 0x47fd20 / 0x47fe20 with the fills 0x46c020 / 0x481280 / 0x481430 /
    // 0x4813a0.
    void openRace(const Event& e, bool fromArch = false);
    void raceKey(MenuKey k);
    void drawRace(UiRenderer& r);
    void drawRaceInfo(UiRenderer& r, const Event& e, bool career);  // the info panel (0x480ee0 / 0x480560)
    bool raceHasVehicle(const Event& e) const;                      // 0x481340: an unlocked vehicle the race does not forbid
    bool vehicleForbidden(const Event* e, const VehicleInfo& v) const;  // 0x4fe250: the vehicle-table index is in mission +0xb8
    const Event* eventById(const std::string& id) const;
    bool shotNoVehicle_ = false;  // PK02: --menu-shot HubChallengesWarning
    int raceRow_ = 0;           // PK02: page +0x88, the selected medal row of the career page (0 gold .. 2 bronze)
    bool raceFromArch_ = false; // PK02: menu +0x94c, the page was opened by a world arch (Back closes the menu, 0x4819f0)
    void saveCareer();
    bool pauseDirty_ = false;  // PK17: InGamePauseHUD +0x80
public:
    // P78: successful career saves so far (the save icon 0x5035c0 shows after each one).
    int saveCount() const { return saves_; }
    // P99: the session's process is about to be replaced (exec: next session, back to the menus). FUEL keeps its game
    // object in memory from one session to the next, so what a session changed without a "SaveGameData" (a lost race's
    // played flag +0x10c and races finished +0x3440, distances, vehicle times, tutorial and travel state) is still there
    // at the next save; the port's sessions are processes, so the career is written here [port].
    // A career.sav that exists but did not load (damaged, foreign, a newer version) is not overwritten here.
    void sessionEnd() {
        if (!careerUnreadable_) saveCareer();
    }
    // P99: the race starts again in the same process (RESTART, WhatNext's MENU_VALI2 "TTMN <mission>"): its results are
    // gone, so the race menu asks TT_PAGE_RACE_YES_NO again before leaving it (0x49ade0 while a mission runs).
    void raceRestarted() { results_.clear(), award_ = {}, unlocks_.clear(); }
    // minimap-fix: 0x577d70, the first hub list (hub order; tutorial, career, challenge) holding mission `id`: 0 / 1 / 2,
    // 3 when none does (the edited races). 0x62baa0 (offline mission kill): the radar status line back to 2, timer 5 s.
    int missionList(const std::string& id) const;
    void hudMissionKilled() { hud::bannerKilled(hudMem_.status); }
    // ---- PK25: save robustness (alerts.mm; notes/pk25.md) ----
    // The in-game alert dialogs (resGIGD.tsc). While MCAlert is up it takes the action keys, the menus wait (0x444400:
    // Front dialogs 0 and 2 hidden) and the race HUD hides (0x440a30 -> MissionHUD vf19 0x4b8cb0).
    InGameDialogs* dialogs() { return dialogs_.get(); }
    bool alertUp() const { return dialogs_ && dialogs_->anyShown(); }
    void drawAlerts(UiRenderer& r) { if (dialogs_) dialogs_->draw(r); }
    // 0x441670 (MCAlert save error): "Pause" and the pause page set opens when none is up; the overlay takes it once.
    bool takeAlertPause() { const bool p = alertPause_; alertPause_ = false; return p; }
    // The console's "SaveGameData" (free roam pickups, hub arrival): SaveManager_Save 0x4ebea0 with its refusal and alerts.
    void requestSave() { saveCareer(); }
    // ---- PK17: InGamePauseHUD +0x80, "save when the menu closes" (notes/pk17.md). The pause pages that change the career
    // (garage buy 0x4ce590, vehicle choice 0x4786e0, rider / livery / paint 0x4715c0 0x471e80 0x472550 0x472cc0 0x4796e0
    // 0x479a60, edited-race delete 0x475b50, hub pages 0x481cd0, options kept 0x492670 / 0x492ab0 / 0x492b70 / 0x492f20,
    // bonus code 0x493580) set it; vf07 0x46aca0 clears it when the menu opens. Outside the in-game pause menu (the
    // port's front-end menus, FUEL has none there) the save runs at once [port].
    void markDirty();
    bool pauseDirty() const { return pauseDirty_; }
    void clearPauseDirty() { pauseDirty_ = false; }  // vf07: +0x80 = 0
    // vf08 0x46e9b0 with +0x80: SaveGameData 0x54bbc0 unless a save is pending (+0x77), +0x98 = 1 (the menu waits),
    // +0x80 = 0.
    void closeSave() {
        if (!storage_.saveAlert) saveCareer();
        pauseDirty_ = false;
    }
    // vf02 0x46ea50's wait: MemoryCardManager_C +0x76 / +0x77 clear (and no queued save request: the port saves at once).
    bool storageIdle() const { return !storage_.loadAlert && !storage_.saveAlert; }
    // ---- end PK17
    // MemoryCardManager_C +0x75: autosave off (the corrupt-save "No", 0x440d10); kept across the port's exec'd sessions
    // in $FUEL_AUTOSAVE_OFF [port]; Map > DATA > SAVE switches it back on (0x4ceaf0, TT_AUTO_SAVE_ENABLED).
    bool autosaveOff() const { return storage_.autosaveOff; }
    void setAutosaveOff(bool off);
    int lastSaveCode() const { return storage_.saveErr; }   // +0x64: 0 ok, 1 refused, else SaveGamePc_Z codes
    int lastLoadCode() const { return storage_.loadErr; }   // +0x5c
    // --menu-shot MCALERT_<9024|9045|9113|9023|9038>, PADALERT, INFOSTEXT, IGMSGBOX.
    bool alertShot(const std::string& name);
private:
    struct Storage {             // MemoryCardManager_C (game+0x3200), the parts a Mac file save keeps
        int op = 0;              // +0x3c: 0 idle, 1 save, 2 load
        bool autosaveOff = false;  // +0x75
        int saveErr = 0;         // +0x64
        int loadErr = 0;         // +0x5c
        bool loadAlert = false;  // +0x76: a load (or its alert) is pending
        bool saveAlert = false;  // +0x77: a save (or its alert) is pending
        int slot = -1;           // +0x60: the last Save mode (SaveSlot retry target)
    } storage_;
    std::unique_ptr<InGameDialogs> dialogs_;
    bool alertPause_ = false;
    std::function<void()> afterAlert_;  // the DATA page's next state once the storage is idle (0x47f460 / 0x47f500)
    void storageFrame();                // 0x4eb9c0: the MCAlert answer (retry save / load, idle)
    int loadCareerFile();               // LoadSlot -> 0x4eb9c0 read, check, apply; the corrupt alert
    void dataSaveNow();                 // 0xbf / 0xc0 (0x47f500)
    // ---- end PK25
    int saves_ = 0;
    bool careerUnreadable_ = false;  // P99: career.sav exists but did not load
    // P78: bonus codes (bonus_codes.mm)
    struct BonusSlot {
        int kind = 0;        // 0 vehicle, 1 hub CHEAT livery, 2 pilot livery
        int a = -1, b = -1;  // vehicle / hub + livery / pilot livery index
        std::string code, picture;
        bool owned = false;
        int arg = -1;        // pilot livery text
    };
    struct BonusState {
        std::string entered;
        int focus = 0;  // 0..9 = KEY_0..KEY_9, 10 = KEY_VALID
        bool ok = false;
        int slot = -1;
    } bonus_;
    std::vector<BonusSlot> bonusSlots() const;
    int bonusRedeem(const std::string& entered);
    void bonusOpen(Screen s);
    void bonusKey(MenuKey k);
    void drawBonus(UiRenderer& r);
    int sessionForSale_ = -1;  // P78: vehicles for sale at the session start (0x62af90)
    int vehiclesForSale() const;
    int sessionLivery_ = -1, sessionColor_ = -1;  // P78: the running session's livery / colour (0x47be40)
    upg::Stages sessionStages_;                    // [upgrades U3] the stages the running session drives (its --upgrades)
    float playerX_ = 0, playerZ_ = 0;  // P78: the player's last HUD position (EndRace_WhatNext2's hub, "Go free ride")
    int whatNextHub() const;           // index in hubs_ of the zone the player is in (-1 none)
    void endChain();                   // after the ranking / reward / unlock pages: 0x1c

    std::string dataDir_;
    Settings settings_;
    MenuOptions options_;
    MenuMode mode_;
    MenuScripts scripts_;
    TextTable text_;
    std::unique_ptr<MenuAssets> assets_;
    std::vector<Hub> hubs_;
    std::vector<Event> events_;
    std::vector<VehicleInfo> vehicles_;
    std::set<std::string> startable_;  // variants the port can assemble and drive

    Screen screen_ = Screen::Title;
    std::string singlePage_;
    int tab_ = TabMap;
    int focus_ = 0;           // index into focusables() / list rows
    int listFirst_ = 0;       // first row shown
    int hubIndex_ = 0;
    std::string pendingEvent_;  // chosen event waiting for a vehicle
    std::string sessionEvent_, sessionHub_, sessionVehicle_;
    std::string vehicle_ = "QUAD_ROAD_2";
    std::vector<ResultRow> results_;
    // P75: MissionHUD states 6 (won) / 7 (lost): the end-race page (0x4b7830 / 0x4b82c0) fades in with +0xd3c (0x4b6050:
    // 0 -> 1 in 1 s); the action key (0x4ba540 -> input +0x35) moves on to the ranking (states 0x11 / 0x12).
    int arch_ = -1;            // P75: events_ index under the player in free roam (0x4f6fd0), -1 none
    float archRadius(float x, float z) const;
    void updateArch(const HudState& hud);
    bool resultsWon_ = false;
    // ---- PK08: end-race pages. A challenge ends in state 8 / 9 (0x6300c0: career 6 / 7, challenge 8 / 9, edited 0xa / 0xb,
    // tutorial 0xc): 0x4ba590 goes 8 -> 0x1a (mission +0x50: the first gold medal, set by 0x630ca0 with the fuel) else 0x1c,
    // 9 -> 0x1c, with no ranking, reward or unlock pages. Ranking window (MissionHUD +0xe70 selected racer, +0xe6c row).
    bool challengeEnd_ = false;   // the finished event is a challenge of the hub lists
    bool fuelChallenge_ = false;  // mission +0x50
    int rankSel_ = 0, rankRow_ = 0;
    struct { int kind = 0, param = -1; } shotBanner_;  // --menu-shot BANNER: a banner without a free-roam session
    int unlockMapN_ = -1;         // --menu-shot / audit override of game +0x346c for the map pictures
    float mapPulse_ = 0;          // IGP +0xe94 (the unlocked-hub map's MAP_2 pulse clock)
    std::string playerName() const;
    std::string opponentName(const ResultRow& r) const;  // PK16
    // racer shown on ranking row 1..5 (0x4b1c50), -1 none
    void rankClamp();
    int resultsList_ = 1;      // the mission list kind (0x577d70): 0 tutorial, 1 career, 2 challenge, 3 edited race
    float endRaceClock_ = 0;
    float creditsClock_ = 0;   // P75: page +0x44c (seconds scrolled)
    std::vector<std::pair<std::string, std::vector<std::string>>> unlocks_;  // P75: pages still to show, their dyn texts
    bool nextUnlock();         // P75: shows the next unlock page; false when none is left
    void buildUnlocks();
    void drawCredits(UiRenderer& r);
    void drawEndRace(UiRenderer& r);
    std::unique_ptr<Career> career_;         // P39: FUEL's career (career.sav)
    CareerAward award_;                      // the last finished event's award (CareerRaceReward)
    std::string infoBox_, buyTarget_;        // garage info box over the vehicle list (InfoBoxGarage*)
    std::string infoText_;                   // [upgrades U3] the info box's port text (its first label; "" = FUEL's TT)
    MenuRequest request_;
    float time_ = 0;
    float pointerX_ = -1, pointerY_ = -1;
    // HUD runtime state (p40)
    const fuel::RoadGraph* roads_ = nullptr;
    const fuel::HubZones* zones_ = nullptr;
    const std::array<fuel::RoadTypeParams, 21>* roadTypes_ = nullptr;
    struct HudMemory {
        float zoom = 2000, zoomTarget = 2000;  // MapHUD +0x19e8 / +0x19ec (ctor 2000)
        float hx = -1, hz = 0;                // +0x19f4 / +0x19f8
        float pulse = 0;                      // +0x1a78
        float fadeT = 0;                      // +0x1ad8 (GROUP_FADE)
        hud::BannerStatus status;             // +0x1ae4 / +0x1ae8 / +0x1ae0 (minimap-fix)
        int missionState = 0;                 // last frame's (the kill 0x62baa0 = a mission leaving)
        float markerAlpha = 0;                // +0x1af4 (free-roam markers, 0x49f6d0)
        std::string fadeName;
        float tol = -1;
        int lod = -1;
        std::vector<std::pair<int, std::vector<std::array<float, 2>>>> cache;  // record, simplified points
        float cacheX = 1e30f, cacheZ = 1e30f, cacheHalf = 0;
        std::array<float, 2> needles{-100, -130};
        int meterFrame = 0;
        float damage = 0;
        std::vector<float> pp;  // pp001.tsc (minimap zoom PP22/23, GPS styles PP840..928)
    } hudMem_;
    struct TutorialDef {
        const char* icon;
        std::vector<uint32_t> texts;
        uint32_t title;
        bool immediate = false;  // +0x18
    };
    static const TutorialDef* tutorialDef(int index);  // 0x56d900
    struct TutorialState {
        std::vector<int> queue;  // +8
        int shown = -1, page = 0;  // the open entry, its text page (+0x10)
        float clock = 0;           // +0x18
        bool busy = false;         // +0x14: a page is up, or closed less than 0.5 s ago (0x56e620 refuses)
        float closeClock = -1;     // +0x1c: time since the last page closed (-1 = idle)
        bool menuOpen = false;     // port: the menus are up (see updateTutorial)
        int lastKind = 0;          // port: the MessagesHUD kind last seen
    } tutorial_;
    // P62: FUEL's confirmation popup (state setups 0x47cc70 / 0x495ca0 / 0x49b500): page PopUpHelp with the text entry
    // `key` (0x46b2e0) and the Yes/No buttons of PopUpInput_YesNo at ITEM_ID_POPUPHELP_BOX (0x4ce260); No focused first
    // (0x4ce390(0)); Left/Right move, the action key runs the focused button's callback (0x4cdfa0); Back does nothing.
    struct Confirm {
        bool open = false;
        std::string key;
        std::function<void()> yes, no;
        int focus = 1;  // 0 Yes (+0x964), 1 No (+0x968)
        bool okOnly = false;  // P66: 0x4ce460: PopUpInput_Continue (ITEM_ID_CONTINUE_LABEL, OK fn +0x96c) instead of Yes/No
    } confirm_;
    void openConfirm(const std::string& key, std::function<void()> yes, std::function<void()> no = {});
    void openOk(const std::string& key, std::function<void()> ok);  // P66: OK-only popup (0x4ce460 / 0x4ce400)
    bool mapData_ = false;  // P66: the Map tab's DATA page (HubData: LOAD / SAVE, state 0x5b)
    void dataLoad();        // P66: 0x47d6d0 .. 0x47f460 / 0x4ce8e0
    void dataSave();        // P66: 0x4ceaf0 .. 0x47f500
    void confirmKey(MenuKey k);
    void drawConfirm(UiRenderer& r);
    // P62: the options pages' values when they opened (0x4947c0 display -> page +0x42f.., 0x494950 audio -> +0x43c..),
    // restored by "Yes" in the leave popup (0x495b70 / 0x495a50).
    struct OptionsSnapshot {
        MenuOptions options;
        float volume = 1;
        bool postEffects = true, grass = true, fullscreen = false;
        int width = 0, height = 0;   // P101
        PortOptions port;            // P101
    } optSnap_;
    void optionsOpen(Screen s);
    void optionsLeave(bool keep);
    // ---- P101 (options_pages.mm): GRAPHICS, the limiter row, the Controller menu (state 0x29) and its settings page
    PortOptions port_;
    Settings startSettings_;
    int optSerial_ = 0, vibTests_ = 0;
    bool ctrlMenu_ = false;            // the Options tab shows PageOptionsController (state 0x29)
    void addGraphicsItem();            // GRAPHICS in the Options left menu
    int optionRows(Screen s) const;
    void optionChange(int d);          // Left / Right on the focused row of an options page
    void vibrationSwitch(int d);       // 0x495670
    void drawGraphics(UiRenderer& r);
    void drawController(UiRenderer& r);
    void drawAudioRows(UiRenderer& r);
    void drawPortRow(UiRenderer& r, Page& p, const char* rowPrefix, float rowY, float dy, int row, const std::string& label,
                     const std::string& value, bool focused);
    bool optionsPointer(float x, float y, bool click);
    // ---- pad-ux [port]: the scrolled option pages (options_pages.mm)
    ListScroll scroll_;
    int scrollVisible(Screen s) const;  // rows the page shows at once (0: the page does not scroll)
    void syncScroll();                  // the window over the page's rows, following the selection
    float scrollRowY(int slot) const;   // the label y of window slot `slot` on the page shown
    void drawScrollArrows(UiRenderer& r);
    bool scrollArrowHit(float x, float y, bool click);
    // ---- end pad-ux
    std::vector<std::pair<int, int>> resolutions() const;
    std::string graphicsValue(int row) const;
    bool pendingRestart(int row) const;
    // ---- [port] remap (remap_page.mm): FUEL's PageOptionsControllerConfigChange(B / T) list with two binding columns
    struct RemapState {
        std::vector<fuel::input::remap::Row> rows;
        int column = 0;             // 0 keyboard, 1 pad
        float clock = 0;            // +0x484: time on the page / since the last capture (captures wait 0.5 s)
        float pulse = 0;            // +0x498: the focused row's pulse clock (0x4cd9d0)
        bool capture = false;       // +0x468: waiting for a key / button
        float captureTime = 0;      // [port] a pad capture gives up after 10 s
        fuel::input::remap::PadCapture pad;
        std::string note;           // [port] the line under the list (swap / refusal / reset)
        float noteTime = 0;
    } remap_;
    std::shared_ptr<fuel::input::InputMap> bindings_, bindDefaults_;
    int bindSerial_ = -1;
    std::map<std::string, std::string> bindLines_;  // key.* / pad.* lines of the page (settings.txt)
    bool bindLinesSet_ = false;                     // the page changed them (else settings_.all's own lines)
    void remapOpen();
    void remapLeave();
    void remapKey(MenuKey k);
    void remapMove(int d);
    void remapAccept();
    void remapResetAsk(int device);
    void remapApply(const fuel::input::remap::Result& r, int action, int device);
    void drawRemap(UiRenderer& r);
    bool remapPointer(float x, float y, bool click);
    std::string remapCell(int action, int device);
    const fuel::input::InputMap& bindingDefaults();
    void remapAfterScroll(int dir);
    // --menu-shot PAGEOPTIONSCONTROLLERCONFIG[:<ACTION>[:k|p[:capture|:bind=<key or pad input>|:reset]]] (in memory)
    bool remapShot(const std::string& args);
    float remapRowY(int slot) const { return 250.0f + 50.0f * float(slot); }  // ITEM_ID_OPT_CTRL_CHANGE_ITEM_FIRST 250, h 50
    // ---- end remap
public:
    friend int runRemapCheck(const std::string& dataDir);  // [port] remap (remap_page.mm)
    // P101 --menu-check: the options pages driven through key presses on a scratch settings file.
    friend int runOptionsCheck(const std::string& dataDir);
    friend int runChromeCheck(const std::string& dataDir);  // PK18 (menu_chrome.mm)
    friend int runAlertCheck(const std::string& dataDir);   // PK25 (alerts.mm)
    friend int runScrollCheck(const std::string& dataDir);  // pad-ux (options_pages.mm)
    friend int runUpgradePageCheck(const std::string& dataDir);  // [upgrades U3] (garage_upgrades.mm)
    friend int runDevModeCheck(const std::string& dataDir);      // [dev mode] (dev_mode.mm)
private:
    void storeOptionsInCareer();
    void drawVehicleSelector(UiRenderer& r);  // P62: GearSelecteur carousel (0x473ea0)  // the values at their FUEL places in career.sav (game +0x33f0.., +0x24c..)
    // ---- P66: garage livery / paint / driver pages (garage.mm; notes/p66-garage-profile.md) ----
    // FUEL's garage page state +0x16c: 1 garage, 0x94 livery, 0x95 paintwork, 0x3b Driver, 0x3c / 0x3d / 0x3e the
    // DriverHead / Chest / Legs menus, 0x3f skin, 0x9b / 0x9d / 0x9f part types, 0x9c / 0x9e / 0xa0 part liveries.
    struct GarageLayer {
        std::string image;     // the picture (MaterialAnim name; FUEL's entry +4 crc)
        Rgba colour{1, 1, 1, 1};
        int ref = 0;           // 0 ITEM_ID_BIDON_BASE, 1 ITEM_ID_LIVERIES_REF_ITEM (x 0.87 wide)
    };
    struct GarageEntry {       // FUEL's 0x24-byte carousel entry
        std::vector<GarageLayer> layers;
        int value = 0;         // +0x18
        bool owned = true;     // +0x1c (false: drawn black)
        uint32_t text = 0;     // +0x20: ITEM_ID_SLOT_OR_LIVERY_NAME
        std::string label;     // PHYSIQUE: a port text instead of `text` (the outfit carousel)
    };
    struct GarageState {
        int state = 1;
        std::string page;      // the left menu over the Garage tab (Driver, DriverHead, ...); empty = Garage
        std::vector<GarageEntry> entries;
        int focus = 0;
    } garage_;
    std::set<uint32_t> riderLiveries_;  // PILOT.DPC MaterialAnim names (livery counts, 0x5dc850)
    bool garageDataLoaded_ = false;
    void garageLoadData();
    void garageEnter(int state);                 // 0x4780c0 + the page builds 0x47aa40 / 0x47af10 / 0x4740f0 ...
    bool garageAccept(const std::string& id);    // left-menu items of Garage / Driver / DriverHead ...
    void garageKey(MenuKey k);                   // the carousels (0x4796e0 / 0x479a60 / 0x471f60 ...)
    bool garageBack();                           // Back on a garage sub page; false when not in one
    void drawGarageSelect(UiRenderer& r);        // 0x47a570 / 0x47a6a0 / 0x473c70 / 0x473e50
    void drawCarousel(UiRenderer& r, const std::vector<GarageEntry>& entries, int focus, const char* refId, float spacing,
                      int layerRef = -1, float widthFactor = -1);  // 0x473ea0
    const VehicleInfo* vehicleInfo(const std::string& label) const;
    // ---- PK20: the garage page's preview fields (InGamePauseHUD) and helpers (garage.mm)
    struct Garage3DPage {
        bool shown = false;       // the garage pages were up last frame (0x478400 .. 0x4789e0 bracket)
        int item = -1;            // 0x478070 last frame: 0 DRIVER, 1 VEHICLE, 2 LIVERY, 3 COLOR, 4 START_RACE, -1 none
        float vehicleTimer = -1;  // +0xec (0.15 s, 0x9dc0b8; -1.0 [0xa05224] idle)
        float riderTimer = -1;    // +0x2d0 (0.25 s, 0x9dbfb0)
        bool loaded = false;      // +0x128
        std::string vehicle;      // the +0xc8 entry's vehicle (the browsed one on the vehicle list)
        int livery = -1;          // the livery the 3-D vehicle was last asked for (0x4799c0 on a change)
        int paint = -1;           // the paint entry last applied (0x479a00 on a change)
        PilotLook look;           // the outfit record last sent
        fuel::RiderOutfit outfit{-1, true};  // PHYSIQUE: the outfit last sent
        int mode = -1;            // InGame3DPerso +0x44 last sent
        std::array<int, 6> stages{{-1, -1, -1, -1, -1, -1}};  // [upgrades U3] the stages last sent (Garage3DCmd::Upgrades)
    } g3d_;
    std::vector<Garage3DCmd> g3dCmds_;
    int garage3DItem();                                          // 0x478070 (the item the garage list's focus is on)
    PilotLook garagePreviewLook() const;                         // the outfit record +0x270 with the browsed entry
    // ---- PHYSIQUE (port deviation): the Driver menu's OUTFIT item (port state 0x3f0, a carousel of RiderOutfit values
    // outfit * 2 + helmet) and the outfit the preview wears (the browsed entry while that carousel is up).
    static constexpr int kOutfitState = 0x3f0;
    fuel::RiderOutfit garagePreviewOutfit() const;
    void garageAddOutfitItem();
    PilotLook garageLookWith(PilotLook look, int state, int value) const;  // 0x471f40 / 0x4721f0 / 0x471ba0 ..
    std::array<float, 3> garagePaintRgb(const std::string& vehicle, int index) const;  // 0x44afb0
    void garageRestart();                                        // 0x478ec0
    void garageReturn(const char* itemId, bool next);            // state 1 again with the garage list's focus (Back / Accept)
    // ---- PK20 end
    // ---- [upgrades U3, DEVIATION] the garage's UPGRADES page (garage_upgrades.mm; notes upg3.md, upgrades-plan.md 4)
    static constexpr int kUpgradesState = 0x1a0;  // the port's garage state (outside FUEL's numbers in the page word +0x16c)
    struct UpgradePage {
        int cat = 0;              // the focused category row (upg::Category)
        int stage = 0;            // the stage cursor 0..3
        int buyCat = -1, buyStage = -1;  // the stage the Yes/No box buys
        ListScroll scroll;        // the category rows' window (pad-ux rule): every row fits today
    } upg_;
    std::array<float, 4> garageListY_{};  // the Garage list's authored y of DRIVER .. PAINTWORK (they move up with UPGRADES)
    std::array<float, 4> garageListX_{};
    bool upgradesShown() const;           // the UPGRADES item / page exist (`upgrades = on`, a career)
    void addUpgradesItem();               // UPGRADES in the Garage list (between PAINTWORK and START THE RACE!)
    void layoutGarageList();              // the list with / without it (applyItemStates)
    void upgradesOpen(int category = -1);
    void upgradesKey(MenuKey k);
    void upgradesAccept();
    void upgradesMove(int dCat, int dStage);
    void drawUpgrades(UiRenderer& r);
    void drawGarageInfoBox(UiRenderer& r);  // FUEL's PopUpHelp TT_PAGE_VEHICLE_* boxes (the vehicle purchase too)
    bool upgradesPointer(float x, float y, bool click);
    bool upgradesShot(const std::string& args);  // --menu-shot GARAGEUPGRADES[:VEHICLE[:cat[:stage[:A|AA|B]]]]
    std::string upgradeStageName(int category, int stage) const;
    std::string upgradeEffect(const std::string& vehicle, int category, int stage) const;
    void upgradeTexts();                  // the port's legend / help line entries (BandeauHelp)
    // ---- end U3
    // ---- [DEVIATION dev mode] (dev_mode.mm; notes devmode.md) settings.txt `dev_mode`: the career's overlay of owned vehicles
    // and stages (Career::setDevMode). applyDevMode sets it from port_ (the constructor, the options row, a restored page)
    // and, turned off, moves the garage's vehicle back to one the real career owns; drawDevTag marks the garage pages.
    void applyDevMode();
    bool devTagShown() const;
    void drawDevTag(UiRenderer& r);
    // ---- end dev mode
    // ---- P66: the Profile tab (profile.mm): FUEL's state +0x3e4 = 3 menu, 0x16 General, 0x1e Collection, 0x24 Completion ----
    struct ProfileState {
        int state = 3;
        int sel = 0;     // ITEM_ID_SELECTION_TYPE entry (+0x128)
        int scroll = 0;  // first line shown (+0x3ec / +0x3f4 ..)
    } profile_;
    struct ProfileLine {
        bool hub = false;        // the hub / vehicle subtitle line (else an info or race line)
        uint32_t text = 0, text2 = 0;
        bool enabled = false, career = true;
        int rank = 3;
    };
    void profileEnter(int state);
    bool profileAccept(const std::string& id);
    void profileKey(MenuKey k);
    void profileFillGeneral();
    std::vector<ProfileLine> profileLines() const;
    void drawProfile(UiRenderer& r);
    // P66: FUEL's vehicle order for the garage carousel and the collection: the names of texts 3401..3475 (0x469db0
    // from 0x47a7d0), the vehicles found in it; `unlockedOnly`: record +0 set.
    std::vector<const VehicleInfo*> vehicleOrder(bool unlockedOnly) const;
    MenuRequest pendingStart_;     // P62: the garage start waiting for tutorial 15's OK (0x56e4c0 -> 0x47eee0 / 0x47efb0)
    bool hudRace_ = false;         // P62: the HUD last drew a race (0x56e620: race state)
    CareerState::Tutorial& tutorialFlags(int index);
    bool tutorialGate(bool flaggedFront) const;  // 0x56e620
    bool rtcFreeze_ = false;  // PK17: 0x534b40(2)
    void tutorialOpenFront();                    // 0x56e5b0 / 0x56e8a0
    void requestStart(const MenuRequest& r);     // 0x4786e0: tutorials 14 / 15 before a garage start
    struct NavMemory {
        int chevronN = -1;      // Navigator3dHUD +0x84
        float chevronClock = 0; // +0x88
    } navMem_;
    // PK19: one entry of the band's icon list (0x4c0010 -> 0x4c0a40 / 0x4c0260 -> 0x4bfc60): type, canvas x, squared
    // horizontal distance, the world place and the colour the call passes in EAX (0..4: PP2-4, PP5-7, PP11-13 / 510,
    // 0.3 grey, PP17-19 / 510).
    struct NavEntry {
        int type;
        float x, d2;
        float wx, wy, wz;
        int colour;
    };
    std::vector<NavEntry> navEntries(const HudState& hud, int& chevrons) const;
    fuel::FreeRoam* freeRoam_ = nullptr;
    struct MapState {
        bool open = false;
        bool focus = false;           // [port] the map has the keys (else the left menu); info pages only then
        float cx = 0, cz = 0;         // cursor (+0x1b38 / +0x1b40) = the map's centre
        float zoom = 6000;            // +0x19e8 displayed radius (eases: (zoom + target) / 2 per frame)
        float zoomTarget = 6000;      // +0x19f0 (P75: 0x482eb0)
        unsigned held = 0;            // P75: map keys held (bit = MenuKey - MapUp): continuous pan 0x483150 / zoom 0x482eb0
        float hx = -1, hz = 0;        // orientation (+0x1b5c / +0x1b60 = the minimap's h at open)
        int selected = -1;            // +0x1b88
        bool target = false;          // MapHUD +0x60
        float tx = 0, tz = 0;         // +0x64 / +0x6c
        float px = 0, pz = 0, phx = -1, phz = 0;  // the player and its minimap heading
        bool havePlayer = false;                  // [port] none (menu shots): the map opens on the current hub's camp
        // PK19: the view centre +0x1b28 / +0x1b30 (follows the cursor: (2 c + 2 cursor) / 4 per frame, 0x4825b0), the hub
        // under the cursor (pause +0x2d4, CareerRules index), the hub zoom pause +0x2d8 and the Map page's
        // ITEM_ID_PICT_INFO picture (0x483750 hub picture / 0x4839e0 selection).
        float vx = 0, vz = 0;
        int hub = -1;
        float hubZoom = 5000;
        std::string pict;
    } map_;
    // ---- PK21: the Editor tab (editor.mm; notes/pk21.md). InGamePauseHUD +0x934 state: 5 top menu, 0xa1..0xa4 the races
    // list (create / edit / delete / try), 0xa5 map editing, 0xa6 delete yes / no, 0xa7 deleted, 0xa8 valid, 0xa9 / 0xaa
    // invalid (km / miles), 0xab route too long, 0xac saving. The slot (+0x940) is edited in place in the career's block
    // (game +0x410) with a copy (+0x4a0) that "quit without saving" puts back.
public:
    void setGround(std::function<float(float, float)> g) { ground_ = std::move(g); }  // 0x445500 for the edited points
    int editorState() const { return ed_.state; }
    int editorSlot() const { return ed_.slot; }
    fuel::editor::Session& editorSession() { return ed_.map; }
    const fuel::EditedRace& editorRace() const { return ed_.race; }
    std::vector<std::string> editorListRows();     // the names EditorRacesList shows (0x477480)
    std::pair<std::string, std::string> editorInfos();  // EditorMapInfos: points used, estimated length (0x4772c0)
    std::string editorHelpKey() const;             // 0x475940: the button legend of the map editing
    fuel::EditorWorld editorWorld();               // roads, zones, ground of the session (or USA1 loaded once)
    // Spots (x, z) near the current hub's camp where the open map editor can drop a point (road, unlocked hub, >= 30 m from
    // a junction), `spacing` metres apart (audit / menu shots; the player moves the cursor there).
    std::vector<std::array<float, 2>> editorRoadSpots(int n, float spacing);
    struct EditorWorldData;  // editor.mm
private:
    struct EditorState {
        int state = 5;
        int action = -1;          // 0x4756d0 of the focused top item when it ran
        int slot = -1;            // +0x940
        int row = 0, perPage = 5; // +0x938, +0x93c
        int base = 5;             // the state whose title / help keys a popup keeps
        std::vector<uint8_t> backup;  // +0x4a0
        fuel::EditedRace race;        // the slot under edit (written back to the career's block after every change)
        fuel::editor::Session map;    // EditorHUD +0x49c
    } ed_;
    std::function<float(float, float)> ground_;
    std::shared_ptr<EditorWorldData> edData_;
    void editorEnter();                // 0x475e70
    void editorItemStates();           // 0x4770b0
    void editorSetState(int st);       // 0x475720
    void editorTop();                  // 0x475ce0
    int editorFocusAction();           // 0x4756d0
    void editorRun(bool fromTop);      // 0x475650
    void editorSelectSlot();           // 0x476ad0
    void editorList(int st);           // 0x4757a0 / 0x4757d0 / 0x475800
    void editorOpenMap();              // 0x4758b0
    void editorValidate();             // 0x475d80
    void editorDelete();               // 0x475b50
    void editorTry();                  // 0x476670
    void editorRestore();              // 0x475870
    void editorStore();                // the live record into the career's block
    bool editorAccept(const std::string& id);  // the top menu (0x4761d0)
    void editorKey(MenuKey k);         // 0x475fa0 per state
    bool editorListKey(MenuKey k);     // 0x4762b0
    void editorUpdate(float dt);       // 0x4769b0 per frame (cursor, zoom, the dragged point)
    void editorChrome(Chrome& c) const;
    void drawEditor(UiRenderer& r);    // 0x476b30
    void drawEditorMap(UiRenderer& r); // 0x476fe0 / 0x467090 / 0x4678d0
    void drawEditorPopup(UiRenderer& r, const std::string& key);  // PopUpHelp with its button line
    bool editorShot(const std::string& name);  // --menu-shot EDITOR* (in memory, never saved)
    // map_page.mm: the roads and hub zones of a map view (0x4a3750 + 0x4a8090), shared by the Map tab and the editor.
    // PK19: (cx, cz) culls the roads; the zone under (curX, curZ) (the cursor, M+0x1b38) is the highlighted one (NaN: cx, cz).
    void drawMapGround(UiRenderer& r, const hud::MapView& v, float cx, float cz, float zoom, float curX = NAN, float curZ = NAN);
    // ---- end PK21
};

// P101 --menu-check: the options pages (Display, Graphics, Audio, Controller) driven by keys over a scratch copy of
// settings.txt / career.sav ($FUEL_SETTINGS / $FUEL_CAREER_SAVE): navigation, the no-wrap rules, the popups, the
// values written and read back. Returns the number of failures.
int runOptionsCheck(const std::string& dataDir);

// [port] remap --menu-check: the remap page (list, capture, swap, refusal, reset, save / reload, pointer, wheel), the menus'
// routing through the bindings and the live prompt rebuild. Scratch settings.txt / career.sav. Returns the failures.
int runRemapCheck(const std::string& dataDir);

// pad-ux --menu-check: the scrolled pages (window, arrows, selection always shown, wheel, pointer rows after a scroll), the
// pad_glyphs row, and FUEL's own paging / scrolling rules on every list (camps, races, challenges, profile, ranking, edited
// races: every entry reachable with keys and with the wheel). Scratch settings.txt / career.sav. Returns the failures.
int runScrollCheck(const std::string& dataDir);

// [upgrades U3] --menu-check: the garage's UPGRADES page on a scratch career (navigation, buy / equip, the locks and their
// popups, the pad / keyboard / mouse flow, the preview's stages, the options row, the free-ride restart). Returns the failures.
int runUpgradePageCheck(const std::string& dataDir);

// [dev mode] --menu-check: dev mode on a scratch career (every vehicle in the garage carousel and the race / challenge lists
// owned, the UPGRADES page all owned with Accept equipping free, the DEV tag, the save: FUEL's sections and PORT_UPGRADES
// as the real career's, the choices in PORT_DEVMODE; off restores the real career). Returns the failures.
int runDevModeCheck(const std::string& dataDir);

// PK18 --menu-check: the pause chrome (menu_chrome.mm) on a scratch settings file / career: tab rules (a race running, the tutorial
// mode), halo sizes and alpha, BandeauHelp / TopTitleDyn keys per screen (each key resolves to an entry of the page's list),
// the sound pick priority, the focus jitter. Returns the number of failures.
int runChromeCheck(const std::string& dataDir);

// PK25 --menu-check: the save robustness flows on a scratch career.sav / settings.txt: a corrupt save asks TT 9113 (No ->
// autosave off, TT 9023, saves refused, the file kept; Yes -> the next save overwrites it), a read-only save asks TT 9024
// (Yes retries, No closes; the file kept), the demo/tutorial-mode refusal, DATA > SAVE switching autosave back on.
int runAlertCheck(const std::string& dataDir);

}  // namespace fuel::ui
