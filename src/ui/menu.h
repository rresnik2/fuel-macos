// FUEL's in-game menu, driven by the original pages (menu_model.hpp): the top bar
// (TopMenu, tache_tout_menu, TopTitle), the left menus (Race, Garage, Map, Options,
// TemplateLeftMenu), the list pages (HubSelectCamps, HubSelectCareer,
// HubSelectChallenges), the options pages, the race ranking and the HUD pages
// (compteur, classement, boussole). What each item does is game code in FUEL.exe;
// the flows here follow the item ids and are the port's (see notes/p19-menus.md).
#pragma once

#include "core/events.hpp"
#include "core/hubs.hpp"
#include "core/settings.hpp"
#include "ui/menu_assets.hpp"
#include "ui/menu_model.hpp"

#include <map>
#include <memory>
#include <string>
#include <vector>

namespace fuel::ui {

class UiRenderer;

enum class MenuMode { Title, Main, Pause, Results };
enum class MenuKey { Up, Down, Left, Right, Accept, Back, TabLeft, TabRight };

struct MenuRequest {
    enum Kind { None, StartEvent, FreeRide, Resume, Restart, QuitToTitle, QuitGame };
    Kind kind = None;
    std::string event, hub, vehicle;
};

struct ResultRow {
    std::string name;
    float time = 0;     // seconds, 0 = did not finish
    bool player = false;
};

// HUD values for one frame.
struct HudState {
    float speedKmh = 0;
    float odometerKm = 0;
    std::string hubName;  // radar caption
    bool race = false;
    int position = 0, racers = 0;
    std::vector<ResultRow> standings;  // current order (time = gap or 0)
};

// The port's own options kept beside fuel::Settings in the same file (unknown keys
// survive saveSettings).
struct MenuOptions {
    bool raceInfo = true, rankingInfo = true, gps = true, speedMeter = true, miles = false;
    float brightness = 0.5f, musicVolume = 1.0f;
};
MenuOptions loadMenuOptions(const std::string& path = settingsPath());
bool saveMenuOptions(const MenuOptions& o, const std::string& path = settingsPath());

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

    // The session the game runs (pause and results menus).
    void setSession(const std::string& event, const std::string& hub, const std::string& vehicle);
    void setResults(const std::vector<ResultRow>& rows);
    void setMode(MenuMode m);
    MenuMode mode() const { return mode_; }

    // Shows the screen built around a page name (--menu-shot): TopMenu tabs ("Race",
    // "Garage", "Map", "Options", ...), list pages, options pages, PressStart,
    // race_ranking_win, compteur/classement/boussole (HUD) or any other page alone.
    bool showPage(const std::string& page);

    void key(MenuKey k);
    void pointer(float x, float y, bool click);  // canvas coordinates
    void update(float dt);
    void draw(UiRenderer& r);
    void drawHud(UiRenderer& r, const HudState& hud);
    MenuRequest take();

    std::string textOf(uint32_t tt) const;
    // A vehicle variant label ("QUAD_ROAD_2") as the game names it ("Speed Demon").
    std::string vehicleName(const std::string& label) const;

private:
    enum class Screen { Title, Tab, CareerList, ChallengeList, CampList, VehicleList, OptDisplay, OptAudio, Results, Single };
    enum Tab { TabLast, TabGarage, TabMap, TabProfile, TabOptions, TabEditor, TabOnline, TabCount };

    void goTab(int tab);
    void openList(Screen s);
    bool tabAvailable(int tab) const;
    Page* leftPage();
    std::vector<Item*> focusables();
    void accept(const std::string& id);
    void listMove(int d);
    void listPage(int d);
    void applyItemStates();
    void prepareRows();
    void drawPage(UiRenderer& r, Page& p, float dx = 0, float dy = 0);
    void drawItem(UiRenderer& r, const Item& it, float dx, float dy);
    void setTitle(const std::string& key);
    std::vector<const Event*> hubEvents(Event::Kind kind) const;
    std::vector<const VehicleInfo*> vehicleChoices() const;
    void saveOptions();
    void back();

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
    MenuRequest request_;
    float time_ = 0;
    float pointerX_ = -1, pointerY_ = -1;
};

}  // namespace fuel::ui
