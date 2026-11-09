// PK18: the pause menu's chrome (notes/pk18.md): the BandeauHelp bottom bar (0x46b270 / 0x46b1b0 / 0x46b7c0 / 0x46b8f0) and
// TopTitleDyn (0x46b560 / 0x46b750), the top bar's tab rules (0x46e650 / 0x46f080 / 0x46f250) and halos (0x4701a0), the focus
// jitter (0x46f6f0), the row-halo pulse (0x46c400) and the menu sound pick (0x46d380). O = observed in the decompile, I = inferred,
// [port] = deliberate deviation.
#include "ui/menu.h"
#include "ui/ui_renderer.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <strings.h>
#include <unistd.h>
#include <cstdlib>

namespace fuel::ui {

namespace {

std::string upperAscii(std::string s) {
    for (char& c : s)
        if (c >= 'a' && c <= 'z') c = char(c - 32);
    return s;
}
std::string upper(std::string s) { return upperAscii(std::move(s)); }

const char* kTabIconId(int t) {
    static const char* const k[] = {"ITEM_ID_TOP_MENU_LAST",   "ITEM_ID_TOP_MENU_GARAGE", "ITEM_ID_TOP_MENU_MAP",   "ITEM_ID_TOP_MENU_PROFILE",
                                    "ITEM_ID_TOP_MENU_OPTIONS", "ITEM_ID_TOP_MENU_EDITOR", "ITEM_ID_TOP_MENU_ONLINE"};
    return k[t];
}

}  // namespace

// 0x46e650 [O]: a race running (MissionStatus +0x48 not 0 / 1) -> 3 (4 when online, mission +0x3b8: not in the port); else 1 + (the
// player is inside its hub camp polygon, 0x481a40). [port] a race runs while a paused session carries an event; the title
// menus (MenuMode::Main) have no player: they count as inside the camp, as do menu shots.
int Menu::menuMode() const {
    if (mode_ == MenuMode::Pause && !sessionEvent_.empty()) return 3;
    if (mode_ == MenuMode::Pause && zones_ && whatNextHub() < 0) return 1;
    return 2;
}

// 0x46f080 [O]: mode 3 / 4 (a race) disables the GARAGE, ONLINE and EDITOR icons and hides their halos; 0x46f250 [O]: the tutorial
// mode (game+0x32bc == 2) disables PROFILE and EDITOR and hides their halos. The enabled state of an icon the port does not
// implement (ONLINE) stays off here (PK21: EDITOR is ported).
bool Menu::tabHaloShown(int tab) const {
    if (menuMode() >= 3 && (tab == TabGarage || tab == TabOnline || tab == TabEditor)) return false;
    if (gameMode_ == 2 && (tab == TabProfile || tab == TabEditor)) return false;
    return true;
}

// 0x46f080 + 0x4701a0 [O]: every GROUP_TOP_MENU_HALO item stays visible (except the ones above); the focused tab's halo takes the
// size of the hidden ITEM_ID_HALO_SELECTED_SIZE rect (80 x 70) and alpha 1.0 on all three colours (+0x54 / +0x64 / +0x74), the
// others ITEM_ID_HALO_UNSELECTED_SIZE (66 x 56) and alpha 0.5; the size is the image's normal-state size (+0x90 / +0x94).
void Menu::applyTopBar() {
    Page* top = scripts_.page("TopMenu");
    if (!top) return;
    const Item* selR = top->find("ITEM_ID_HALO_SELECTED_SIZE");
    const Item* unR = top->find("ITEM_ID_HALO_UNSELECTED_SIZE");
    for (int t = 0; t < TabCount; ++t) {
        if (Item* icon = top->find(kTabIconId(t))) icon->enabled = tabAvailable(t), icon->focus = t == tab_;
        Item* halo = top->find(std::string(kTabIconId(t)) + "_HALO");
        if (!halo) continue;
        const bool sel = t == tab_;
        halo->visible = tabHaloShown(t);
        halo->focus = false;
        const Item* r = sel ? selR : unR;
        halo->drawW = r ? r->w : 0, halo->drawH = r ? r->h : 0;
        const float a = sel ? 1.0f : 0.5f;
        halo->cNormal.a = halo->cEnabled.a = halo->cFocus.a = a;
    }
}

// 0x46f6f0 [O]: Park-Miller rand 0x6462e0 in [-1, 1] (an integer).
int Menu::jitterRand() {
    jitterSeed_ = uint32_t((uint64_t(jitterSeed_) * 48271u) % 0x7fffffffu);
    return int(jitterSeed_ % 3) - 1;
}

// 0x46d380 [O]: the highest-priority request bit plays ONE sound per frame: bit0 BACK(0), bit1 CONFIRM(3), bit2 / bit3
// NAVIGATIONOPTIONS(7), bit4 MOVEBACKWARD(5), bit5 MOVEFORWARD(6), bit6 CANCEL(1), bit7 INVALID(4); bit8 (the GPS key, 0x4c92f0)
// has no sound. The bits clear at the next frame's 0x46fba0 (+0x90 = 0).
int Menu::takeSound() {
    const unsigned b = snd_ & 0xff;
    snd_ = 0;
    if (b & 1) return 0;
    if (b & 2) return 3;
    if (b & 4) return 7;
    if (b & 8) return 7;
    if (b & 0x10) return 5;
    if (b & 0x20) return 6;
    if (b & 0x40) return 1;
    if (b & 0x80) return 4;
    return -1;
}

// The hub name text 0x47cc70 / 0x49b500 build in local_200 (+0x954 -> hub name TT; "" without a hub) and hand to TopTitleDyn.
// 0x47cc70 (hub menu pages): +0x954 = the pending hub, the port's browsed hub. 0x49b500 (Race tab): the hub the player is in
// (0x577500), the port's zone of the player else the session's hub.
std::string Menu::areaName() const {
    if (hubs_.empty()) return "";
    int h = hubIndex_;
    if (screen_ == Screen::Tab && tab_ == TabLast) {
        h = whatNextHub();
        if (h < 0) {
            h = -1;
            for (size_t i = 0; i < hubs_.size(); ++i)
                if (!sessionHub_.empty() && upper(hubs_[i].id) == sessionHub_) h = int(i);
            if (h < 0 && !zones_) h = hubIndex_;
        }
    }
    if (h < 0 || h >= int(hubs_.size())) return "";
    return upperAscii(textOf(hubs_[size_t(h)].nameText));
}

// 0x4836d0 [O]: the map view's button legend by the selected item (type at +0x1c: 6 camp, 7..10 career, 11 / 12 challenge, 13..20
// livery / vista point, 21 heliport) and the "mission on" flag (bl = 0x4844c0: a target can be set = no race running). The
// *_TARGET entries (GPS target set) are never chosen by this function; no selection -> NO_TARGET.
std::string Menu::mapLegendKey() const {
    const bool free = sessionEvent_.empty();
    auto pick = [&](const char* base) { return std::string(base) + (free ? "" : "_MISSION_ON"); };
    const auto items = mapItems();
    if (map_.selected < 0 || map_.selected >= int(items.size())) return pick("TT_MAP_NO_TARGET");
    const MapItem& it = items[size_t(map_.selected)];
    if (it.type == 6) {
        if (!free) return "TT_MAP_HUB_NO_TARGET_MISSION_ON";  // [O] the script has no such entry: the legend hides
        return it.enabled && it.index >= 0 && career_ && career_->state().hubs.size() > size_t(it.index) && career_->state().hubs[size_t(it.index)].opened == 0
                   ? "TT_MAP_HUB_NO_TARGET_WITHOUT_VISITED_CAMP"
                   : "TT_MAP_HUB_NO_TARGET";
    }
    if (it.type >= 7 && it.type <= 10) return pick("TT_MAP_CAREER_NO_TARGET");
    if (it.type == 11 || it.type == 12) return pick("TT_MAP_CHALLENGE_NO_TARGET");
    if (it.type >= 13 && it.type <= 20) return it.enabled ? pick("TT_MAP_HELIPORT_NO_TARGET") : pick("TT_MAP_NO_TARGET");
    if (it.type == 21) return pick("TT_MAP_HELIPORT_NO_TARGET");
    return pick("TT_MAP_NO_TARGET");
}

Menu::Chrome Menu::chrome() const {
    Chrome c;
    auto top = [&]() -> std::string {  // the TopTitle entry the port's setTitle() selected
        if (const Page* p = scripts_.page("TopTitle"))
            if (const Item* t = p->find("ITEM_ID_TITLE"))
                if (!t->hasDynText && t->entryIndex >= 0 && size_t(t->entryIndex) < t->entries.size()) return t->entries[size_t(t->entryIndex)].key;
        return "";
    };
    // 0x47cc70: the hub menu pages (0x53 .. 0x5b): TopTitleDyn = the hub's name, TopTitle hidden (TT_NULL).
    auto hubPage = [&](const char* buttons, const std::string& help) {
        c.hasDyn = true, c.dyn = areaName(), c.buttons = buttons, c.help = help;
    };
    switch (screen_) {
        case Screen::Title:
        case Screen::Single:
        case Screen::WhatNext: return c;  // no BandeauHelp (0x4b6f30 draws the bars and the page only)
        case Screen::Tab:
            switch (tab_) {
                case TabLast:  // 0x49b500 state 0; the restart popup (0xae) hides the legend and the help
                    c.hasDyn = true, c.dyn = areaName();
                    if (!confirm_.open) c.buttons = "TT_MENU_MAIN", c.help = "TT_BH_RACE";
                    return c;
                case TabGarage:
                    c.title = top();
                    if (garage_.page.empty()) {  // state 1: 0x47c1e0
                        c.help = "TT_BH_GARAGE", c.buttons = pendingEvent_.empty() ? "TT_MENU_MAIN" : "TT_MENU_GARAGE";
                    } else if (garage_.page == "Driver") {
                        c.help = "TT_BH_GARAGE_DRIVER", c.buttons = "TT_MENU_PILOT";
                    } else {
                        c.help = garage_.page == "DriverHead" ? "TT_BH_GARAGE_DRIVER_HEAD" : garage_.page == "DriverChest" ? "TT_BH_GARAGE_DRIVER_CHEST" : "TT_BH_GARAGE_DRIVER_LEGS";
                        c.buttons = "TT_MENU_PILOT";
                    }
                    return c;
                case TabMap:
                    if (mapData_) {  // 0x5b
                        c.title = top(), c.buttons = "TT_ACCEPT_BACK", c.help = "TT_BH_HUB_DATA";
                    } else if (map_.focus) {  // 0xe: the map view (0x482b00 + the legend of 0x4836d0)
                        c.title = "TT_TOP_TITLE_MAP", c.help = "TT_BH_MAP", c.buttons = mapLegendKey();
                    } else {  // 0x53: TT_MENU_HUB inside the camp (mode 2), TT_ACCEPT_BACKTOGAME outside
                        hubPage(menuMode() == 2 ? "TT_MENU_HUB" : "TT_ACCEPT_BACKTOGAME", hubHelpKey());
                    }
                    return c;
                case TabProfile: c.title = top(), c.help = "TT_BH_PROFILE", c.buttons = "TT_PROFILE"; return c;
                case TabEditor: editorChrome(c); return c;  // PK21: 0x477a80 / 0x475940
                case TabOptions:
                    c.title = top();
                    if (ctrlMenu_) c.help = "TT_BH_OPTIONS_CONTROLS", c.buttons = "TT_ACCEPT_BACK";            // 0x29
                    else c.help = "TT_BH_OPTIONS", c.buttons = "TT_MENU_OPTIONS";                                // 6
                    return c;
                default: return c;
            }
        case Screen::CareerList: hubPage("TT_MENU_HUB_SOUS_PAGE", "TT_BH_SELECT_CAREER"); return c;      // 0x55
        case Screen::ChallengeList: hubPage("TT_MENU_HUB_SOUS_PAGE", "TT_BH_SELECT_CHALLENGE"); return c;  // 0x58
        case Screen::CampList: hubPage("TT_MENU_HUB_SOUS_PAGE", "TT_BH_MAIN_CHANGE_CAMP"); return c;      // 0x5a
        case Screen::RacePage: {                                                                            // 0x56 / 0x59
            const bool challenge = !pendingEvent_.empty() && eventById(pendingEvent_) && eventById(pendingEvent_)->kind == Event::Challenge;
            if (challenge) hubPage(raceWarning() ? "TT_BACK" : "TT_MENU_HUB_SOUS_PAGE", "TT_BH_CHALLENGE_DESCRIPTION");  // +0x959: Accept possible
            else hubPage("TT_MENU_HUB_SOUS_PAGE", "TT_BH_CAREER_MOOD");
            return c;
        }
        case Screen::VehicleList: {  // 0x93: 0x47c1e0 + the draw 0x479d50 (TO_BUY unless owned)
            const auto list = vehicleChoices();
            const bool owned = !career_ || focus_ >= int(list.size()) || career_->vehicleOwned(list[size_t(focus_)]->name);
            c.title = "TT_TOP_TITLE_GARAGE_VEHICLE", c.help = "TT_BH_GARAGE_VEHICLE";
            c.buttons = owned ? "TT_GARAGE_VEHICLE_OWNED" : "TT_GARAGE_VEHICLE_TO_BUY";
            return c;
        }
        case Screen::GarageSelect: {  // 0x47c1e0 per state
            c.title = top();
            switch (garage_.state) {
                case 0x94: c.help = "TT_BH_GARAGE_LIVERY", c.buttons = "TT_GARAGE_LIVERY"; break;
                case 0x95: c.help = "TT_BH_GARAGE_PAINTWORK", c.buttons = "TT_GARAGE_PAINTWORK"; break;
                case 0x3f: c.help = "TT_BH_GARAGE_DRIVER_SKIN", c.buttons = "TT_PILOT_SKIN"; break;
                case 0x9b: case 0x9d: case 0x9f: c.help = "TT_BH_GARAGE_ALL_TYPE", c.buttons = "TT_PILOT_TYPE"; break;
                case 0x9c: case 0x9e: case 0xa0: c.help = "TT_BH_GARAGE_ALL_LIVERY", c.buttons = "TT_PILOT_LIVERY"; break;
                default: break;
            }
            return c;
        }
        case Screen::GarageUpgrades: {  // [upgrades U3] the port's page: its legend follows the hovered stage (as 0x479d50's
            // TO_BUY / OWNED does for a vehicle): buy, equip, or the equipped one (nothing to do)
            c.title = top(), c.help = "TT_BH_GARAGE_UPGRADES";
            const UpgradePip p = upgradePip(vehicle_, upg_.cat, upg_.stage);
            c.buttons = p.state == UpgradePip::Equipped ? "TT_GARAGE_UPGRADES_VIEW" : p.state == UpgradePip::Owned ? "TT_GARAGE_UPGRADES_EQUIP" : "TT_GARAGE_UPGRADES_BUY";
            return c;
        }
        case Screen::Profile: {  // 0x4977c0
            c.title = top();
            if (profile_.state == 0x16) c.help = "TT_BH_PROFILE_GENERAL_INFO", c.buttons = "TT_MENU_PROFILE_GENERAL";
            else if (profile_.state == 0x1e) c.help = "TT_BH_PROFILE_COLLECTION", c.buttons = "TT_MENU_PROFILE_PROGRESSION";
            else c.help = "TT_BH_PROFILE_COMPLETION", c.buttons = "TT_MENU_PROFILE_COMPLETION";
            return c;
        }
        case Screen::OptDisplay:
        case Screen::OptGraphics: c.title = top(), c.help = "TT_BH_OPTIONS_DISPLAY", c.buttons = "TT_MENU_OPTION_SELECTED"; return c;  // 0x31 (GRAPHICS: the port's clone)
        case Screen::OptAudio: c.title = top(), c.help = "TT_BH_OPTIONS_AUDIO", c.buttons = "TT_MENU_OPTION_SELECTED"; return c;        // 0x32
        case Screen::OptController: c.title = top(), c.help = "TT_BH_OPTIONS_CONTROLS_SETTINGS", c.buttons = "TT_MENU_OPTIONS_VIBRATION"; return c;  // 0x2c
        case Screen::OptRemap:  // [port] remap: state 0x2d (0x495ca0: the LAYOUT help lines; legend = FUEL's TT 16028 + Change / columns)
            c.title = top();
            c.help = remap_.column == 1 ? "TT_BH_OPTIONS_CONTROLS_LAYOUT_PAD" : "TT_BH_OPTIONS_CONTROLS_LAYOUT_KEYBOARD";
            c.buttons = remap_.capture ? "TT_MENU_OPTIONS_REMAP_WAIT" : "TT_MENU_OPTIONS_REMAP";
            return c;
        case Screen::BonusCodes: c.title = top(), c.help = "TT_BH_OPTIONS_BONUS_CODES", c.buttons = "TT_MENU_OPTION_SELECTED"; return c;      // 0x33
        case Screen::BonusKeyboard: c.title = top(), c.help = "TT_BH_OPTIONS_BONUS_CODES", c.buttons = "TT_MENU_OPTION_BONUS_RESULT"; return c;  // 0x34
        case Screen::BonusResult: c.title = top(), c.buttons = "TT_MENU_OPTION_BONUS_RESULT"; return c;                                        // 0x35 (no help)
        case Screen::Credits: c.title = top(), c.buttons = "TT_OK"; return c;                                                                    // 0x36 (no help)
        // The end-race pages (0x4b7830 / 0x4b82c0 / 0x4b7a70 / 0x4b7b60 / 0x4b7c70 and the unlock pages): TT_OK, no help line; the
        // tutorial race's win page (list kind 0) TT_START_CAREER_AFTER_TUTORIAL.
        case Screen::EndRace:
            c.buttons = resultsWon_ && resultsList_ == 0 ? "TT_START_CAREER_AFTER_TUTORIAL" : "TT_OK";
            return c;
        case Screen::Results:
        case Screen::Reward:
        case Screen::FuelReward:  // PK08 0x4b68b0 ends with 0x46b270 like the other end-race pages
        case Screen::Unlock: c.buttons = "TT_OK"; return c;
    }
    return c;
}

// 0x46b8f0: TopTitleDyn ITEM_ID_TITLE = the text (0x46cdb0 with TT 1309 "%s", upper-cased) or hidden; TopTitle ITEM_ID_TITLE hidden
// while a dyn text shows (the "any" key matches no entry); BandeauHelp: the legend entry by key (hidden when absent) and the help
// line (0x46b7c0: the entry by key, restarting the marquee only when it changes; hidden when absent).
void Menu::applyChrome() {
    const Chrome c = chrome();
    if (Page* p = scripts_.page("TopTitle"))
        if (Item* t = p->find("ITEM_ID_TITLE")) t->visible = !c.hasDyn && !(screen_ == Screen::Results);
    if (Page* p = scripts_.page("TopTitleDyn"))
        if (Item* t = p->find("ITEM_ID_TITLE")) {
            t->visible = c.hasDyn;
            t->hasDynText = true;
            t->dynText = c.dyn;
        }
    Page* bh = scripts_.page("BandeauHelp");
    if (!bh) return;
    // [port] the legend (F7 = 2) and the PAUSE label are placed by the item's a7 (0x615240: x = item.x + (w - textW) a7 / 2); the
    // port's generic code alignment would start the legend at x = -64 (the parked alignment question, as for BACK_TO_HUB).
    for (const char* id : {"ITEM_ID_BUTTONS", "ITEM_ID_PAUSE"})
        if (Item* it = bh->find(id)) it->align = std::clamp(it->a7, 0, 2);
    auto select = [](Item* it, const std::string& key) {  // 0x613730: the entry index by key CRC, -1 when absent
        if (!it) return false;
        for (size_t i = 0; i < it->entries.size(); ++i)
            if (!key.empty() && strcasecmp(it->entries[i].key.c_str(), key.c_str()) == 0) return it->entryIndex = int(i), true;
        return false;
    };
    if (Item* b = bh->find("ITEM_ID_BUTTONS")) b->visible = select(b, c.buttons);
    Item* help = bh->find("ITEM_ID_SCROLLING_HELPS");
    Item* dyn = bh->find("ITEM_ID_SCROLLING_HELPS_DYN");
    if (dyn) dyn->visible = false;  // port: no screen passes a dynamic help text (0x46b7c0 with EAX != 0 is the BH_MAP-style lines of 0x482b00)
    if (help) {
        const bool on = select(help, c.help);
        help->visible = on;
        if (!on || lastHelp_ != c.help) help->scrollX = help->w;  // 0x614b70: restart at the right edge when the entry changes
        lastHelp_ = on ? c.help : std::string();
    }
}

// 0x46b750 (TopTitleDyn) and 0x46b270 (BandeauHelp) at the menu alpha, after the page of the state (the HUD's draw order).
// 0x46b080: the BandeauHelp page updates once the menu is faded in: the marquee 0x614bb0 moves a b19 item by
// dt x -32 px/s and restarts it at the item's width when it has left on the left.
void Menu::drawChrome(UiRenderer& r) {
    const float dt = std::clamp(time_ - chromeClock_, 0.0f, 0.25f) + previewBoost_;
    chromeClock_ = time_, previewBoost_ = 0;
    marqueeDt_ = dt;
    if (Page* p = scripts_.page("TopTitleDyn")) drawPage(r, *p);
    if (Page* p = scripts_.page("BandeauHelp")) drawPage(r, *p);
    marqueeDt_ = 0;
}


// ---- --menu-check ----------------------------------------------------------------------------------------------------------

int runChromeCheck(const std::string& dataDir) {
    int bad = 0, n = 0;
    auto expect = [&](bool ok, const std::string& what) {
        ++n;
        if (!ok) ++bad;
        std::printf("chrome check: %s %s\n", ok ? "ok  " : "FAIL", what.c_str());
    };
    // A scratch folder: the player's own settings.txt and career.sav are never read or written here.
    char tmpl[] = "/tmp/fuel-chrome-check-XXXXXX";
    const char* dir = mkdtemp(tmpl);
    if (!dir) return 1;
    const std::string settingsFile = std::string(dir) + "/settings.txt", careerFile = std::string(dir) + "/career.sav";
    setenv("FUEL_SETTINGS", settingsFile.c_str(), 1);
    setenv("FUEL_CAREER_SAVE", careerFile.c_str(), 1);
    Settings s0 = loadSettings();
    s0.dataDir = dataDir;
    // 1. Tab rules (0x46e650 / 0x46f080 / 0x46f250).
    {
        Menu fr(dataDir, s0, MenuMode::Pause);
        fr.setMode(MenuMode::Pause);
        expect(fr.menuMode() == 2, "no race: menu mode 2 (inside a camp)");
        bool all = true;
        for (int t : {Menu::TabLast, Menu::TabGarage, Menu::TabMap, Menu::TabProfile, Menu::TabOptions}) all = all && fr.tabEnabled(t);
        expect(all, "free ride: LAST, GARAGE, MAP, PROFILE, OPTIONS enabled");
        expect(fr.tabEnabled(Menu::TabEditor) && !fr.tabEnabled(Menu::TabOnline), "EDITOR enabled (PK21), ONLINE not in the port (disabled)");
        expect(fr.tabHaloShown(Menu::TabGarage) && fr.tabHaloShown(Menu::TabProfile), "free ride: every halo shown");
        Menu race(dataDir, s0, MenuMode::Pause);
        const Event* ev = nullptr;
        for (const Event& e : race.events())
            if (e.kind == Event::Career) { ev = &e; break; }
        if (ev) race.setSession(ev->id, ev->hub, race.vehicle());
        race.setMode(MenuMode::Pause);
        expect(ev && race.menuMode() == 3, "a race running: menu mode 3");
        expect(ev && !race.tabEnabled(Menu::TabGarage), "a race running: GARAGE disabled (0x46f080)");
        expect(ev && !race.tabHaloShown(Menu::TabGarage) && !race.tabHaloShown(Menu::TabOnline) && !race.tabHaloShown(Menu::TabEditor),
               "a race running: GARAGE / ONLINE / EDITOR halos hidden");
        expect(ev && race.tabEnabled(Menu::TabProfile) && race.tabEnabled(Menu::TabMap) && race.tabEnabled(Menu::TabOptions), "a race running: PROFILE, MAP, OPTIONS stay");
        race.goTab(Menu::TabLast);
        race.key(MenuKey::TabRight);
        expect(ev && race.tab_ == Menu::TabMap, "a race running: the next tab skips GARAGE (LAST -> MAP)");
        race.goTab(Menu::TabMap);
        race.key(MenuKey::TabLeft);
        expect(ev && race.tab_ == Menu::TabLast, "a race running: the previous tab from MAP is LAST");
        Menu tut(dataDir, s0, MenuMode::Pause);
        tut.setGameMode(2);
        tut.setMode(MenuMode::Pause);
        expect(!tut.tabEnabled(Menu::TabProfile) && tut.tabEnabled(Menu::TabGarage), "tutorial mode: PROFILE disabled, GARAGE stays (0x46f250)");
        expect(!tut.tabHaloShown(Menu::TabProfile) && !tut.tabHaloShown(Menu::TabEditor) && tut.tabHaloShown(Menu::TabGarage), "tutorial mode: PROFILE / EDITOR halos hidden");
        // 2. Halos (0x4701a0): the focused tab 80 x 70 alpha 1.0, the others 66 x 56 alpha 0.5.
        fr.goTab(Menu::TabMap);
        fr.applyItemStates();
        const Page* top = fr.scripts().page("TopMenu");
        const Item* hs = top ? top->find("ITEM_ID_TOP_MENU_MAP_HALO") : nullptr;
        const Item* hu = top ? top->find("ITEM_ID_TOP_MENU_GARAGE_HALO") : nullptr;
        expect(hs && hs->visible && hs->drawW == 80 && hs->drawH == 70 && hs->cNormal.a == 1.0f && hs->cEnabled.a == 1.0f && hs->cFocus.a == 1.0f,
               "focused tab halo 80 x 70, alpha 1.0");
        expect(hu && hu->visible && hu->drawW == 66 && hu->drawH == 56 && hu->cNormal.a == 0.5f, "other tab halos 66 x 56, alpha 0.5 (all stay visible)");
        // 3. Chrome keys per screen; every key must be an entry of its BandeauHelp / TopTitle list.
        const Page* bh = fr.scripts().page("BandeauHelp");
        auto has = [&](const char* item, const std::string& key) {
            const Item* it = bh ? bh->find(item) : nullptr;
            if (!it || key.empty()) return false;
            for (const ListEntry& e : it->entries)
                if (strcasecmp(e.key.c_str(), key.c_str()) == 0) return true;
            return false;
        };
        bool textsOk = true;
        auto textFor = [&](const char* item, const std::string& key) {
            const Item* it = bh ? bh->find(item) : nullptr;
            for (size_t i = 0; it && i < it->entries.size(); ++i)
                if (strcasecmp(it->entries[i].key.c_str(), key.c_str()) == 0) return !fr.textOf(it->entries[i].tt).empty();
            return key.empty();
        };
        auto expectKeys = [&](Menu& m, const char* what, const char* buttons, const char* help, bool dyn) {
            m.applyItemStates();
            const Menu::Chrome c = m.chrome();
            textsOk = textsOk && textFor("ITEM_ID_BUTTONS", c.buttons) && textFor("ITEM_ID_SCROLLING_HELPS", c.help);
            const bool ok = c.buttons == buttons && c.help == help && c.hasDyn == dyn && has("ITEM_ID_BUTTONS", c.buttons) && (c.help.empty() || has("ITEM_ID_SCROLLING_HELPS", c.help));
            expect(ok, std::string(what) + ": " + c.buttons + " / " + (c.help.empty() ? "-" : c.help) + (c.hasDyn ? " + hub name" : ""));
        };
        fr.showPage("Race");
        expectKeys(fr, "Race tab", "TT_MENU_MAIN", "TT_BH_RACE", true);
        fr.showPage("Garage");
        expectKeys(fr, "Garage tab", "TT_MENU_MAIN", "TT_BH_GARAGE", false);
        fr.showPage("Map");
        fr.mapData_ = false, fr.map_.focus = false;
        expectKeys(fr, "Map tab, hub menu", "TT_MENU_HUB", fr.hubHelpKey().c_str(), true);
        fr.showPage("HubSelectCareer");
        expectKeys(fr, "career list", "TT_MENU_HUB_SOUS_PAGE", "TT_BH_SELECT_CAREER", true);
        fr.showPage("HubSelectChallenges");
        expectKeys(fr, "challenge list", "TT_MENU_HUB_SOUS_PAGE", "TT_BH_SELECT_CHALLENGE", true);
        fr.showPage("HubSelectCamps");
        expectKeys(fr, "camp list", "TT_MENU_HUB_SOUS_PAGE", "TT_BH_MAIN_CHANGE_CAMP", true);
        fr.showPage("HubSelectCareerMood");
        expectKeys(fr, "career race page", "TT_MENU_HUB_SOUS_PAGE", "TT_BH_CAREER_MOOD", true);
        fr.showPage("HubChallengesDescription");
        expectKeys(fr, "challenge page", fr.raceWarning() ? "TT_BACK" : "TT_MENU_HUB_SOUS_PAGE", "TT_BH_CHALLENGE_DESCRIPTION", true);
        fr.showPage("HubChallengesWarning");
        expectKeys(fr, "challenge page without a vehicle: Back only", "TT_BACK", "TT_BH_CHALLENGE_DESCRIPTION", true);
        fr.showPage("HubData");
        expectKeys(fr, "hub data", "TT_ACCEPT_BACK", "TT_BH_HUB_DATA", false);
        fr.showPage("Profile");
        expectKeys(fr, "Profile tab", "TT_PROFILE", "TT_BH_PROFILE", false);
        fr.showPage("ProfileGeneral0");
        expectKeys(fr, "Profile > General", "TT_MENU_PROFILE_GENERAL", "TT_BH_PROFILE_GENERAL_INFO", false);
        fr.showPage("ProfileCollection0");
        expectKeys(fr, "Profile > Collection", "TT_MENU_PROFILE_PROGRESSION", "TT_BH_PROFILE_COLLECTION", false);
        fr.showPage("ProfileCompletion0");
        expectKeys(fr, "Profile > Completion", "TT_MENU_PROFILE_COMPLETION", "TT_BH_PROFILE_COMPLETION", false);
        fr.showPage("Options");
        expectKeys(fr, "Options tab", "TT_MENU_OPTIONS", "TT_BH_OPTIONS", false);
        fr.showPage("PageOptionsController");
        expectKeys(fr, "Options > Controller", "TT_ACCEPT_BACK", "TT_BH_OPTIONS_CONTROLS", false);
        fr.showPage("PageOptionsControllerSettings");
        expectKeys(fr, "Options > Controller > Settings", "TT_MENU_OPTIONS_VIBRATION", "TT_BH_OPTIONS_CONTROLS_SETTINGS", false);
        fr.showPage("PageOptionsDisplay");
        expectKeys(fr, "Options > Display", "TT_MENU_OPTION_SELECTED", "TT_BH_OPTIONS_DISPLAY", false);
        fr.showPage("PageOptionsAudio");
        expectKeys(fr, "Options > Audio", "TT_MENU_OPTION_SELECTED", "TT_BH_OPTIONS_AUDIO", false);
        fr.showPage("BonusCodes");
        expectKeys(fr, "Options > Bonus codes", "TT_MENU_OPTION_SELECTED", "TT_BH_OPTIONS_BONUS_CODES", false);
        fr.showPage("BonusCodes:1");
        expectKeys(fr, "Options > Bonus codes keyboard", "TT_MENU_OPTION_BONUS_RESULT", "TT_BH_OPTIONS_BONUS_CODES", false);
        fr.showPage("GarageLivery");
        expectKeys(fr, "Garage > Livery", "TT_GARAGE_LIVERY", "TT_BH_GARAGE_LIVERY", false);
        fr.showPage("GaragePaint");
        expectKeys(fr, "Garage > Paintwork", "TT_GARAGE_PAINTWORK", "TT_BH_GARAGE_PAINTWORK", false);
        fr.showPage("Driver");
        expectKeys(fr, "Garage > Driver", "TT_MENU_PILOT", "TT_BH_GARAGE_DRIVER", false);
        fr.showPage("DriverHeadType");
        expectKeys(fr, "Driver > Head type", "TT_PILOT_TYPE", "TT_BH_GARAGE_ALL_TYPE", false);
        fr.showPage("DriverChestLivery");
        expectKeys(fr, "Driver > Chest livery", "TT_PILOT_LIVERY", "TT_BH_GARAGE_ALL_LIVERY", false);
        fr.showPage("DriverSkin");
        expectKeys(fr, "Driver > Skin", "TT_PILOT_SKIN", "TT_BH_GARAGE_DRIVER_SKIN", false);
        // The garage's vehicle list: TO_BUY on a vehicle not owned, OWNED on an owned one (0x479d50).
        fr.showPage("GarageVehicles");
        const std::string vb = fr.chrome().buttons;
        expect(vb == "TT_GARAGE_VEHICLE_OWNED" || vb == "TT_GARAGE_VEHICLE_TO_BUY", "vehicle list legend " + vb);
        {
            const auto vl = fr.vehicleChoices();
            int unowned = -1;
            for (size_t i = 0; fr.career() && i < vl.size(); ++i)
                if (!fr.career()->vehicleOwned(vl[i]->name)) { unowned = int(i); break; }
            fr.showPage("GaragePrice");
            if (unowned >= 0) expect(fr.chrome().buttons == "TT_GARAGE_VEHICLE_TO_BUY" && has("ITEM_ID_BUTTONS", "TT_GARAGE_VEHICLE_TO_BUY"), "vehicle list on an unowned vehicle: TT_GARAGE_VEHICLE_TO_BUY");
        }
        // The map view's legend: no selection -> NO_TARGET, a race running -> the MISSION_ON entries (0x4836d0).
        fr.showPage("MapFocus");
        expectKeys(fr, "map view", fr.mapLegendKey().c_str(), "TT_BH_MAP", false);
        expect(fr.chrome().title == "TT_TOP_TITLE_MAP", "map view keeps the MAP title");
        // The restart popup of the Race tab hides the legend and the help (0x49b500 state 0xae).
        fr.showPage("Race");
        fr.openConfirm("TT_PAGE_RACE_YES_NO", {}, {});
        expect(fr.chrome().buttons.empty() && fr.chrome().help.empty() && fr.chrome().hasDyn, "Race tab restart popup: no legend, no help");
        fr.confirm_.open = false;
        // The end-race pages: TT_OK and no help line.
        fr.showPage("Reward");
        expectKeys(fr, "reward page", "TT_OK", "", false);
        fr.showPage("Title");
        expect(fr.chrome().buttons.empty() && fr.chrome().help.empty(), "title screen: no BandeauHelp");
        expect(textsOk, "every key the screens name has a text in the language table");
    }
    // 4. Sounds (0x46d380): one sound per frame, the highest-priority bit wins; the bits clear when read.
    {
        Menu m(dataDir, s0, MenuMode::Pause);
        m.setMode(MenuMode::Pause);
        m.goTab(Menu::TabOptions);
        m.takeSound();
        auto press = [&](std::initializer_list<MenuKey> ks) {
            for (MenuKey k : ks) m.key(k);
            return m.takeSound();
        };
        m.goTab(Menu::TabMap);
        m.map_.focus = false;
        expect(press({MenuKey::Up}) == 7, "Up -> NAVIGATIONOPTIONS (7)");
        expect(press({MenuKey::Down}) == 7, "Down -> NAVIGATIONOPTIONS (7)");
        expect(press({MenuKey::TabLeft}) == 5, "tab left -> MOVEBACKWARD (5)");
        m.goTab(Menu::TabMap);
        expect(press({MenuKey::TabRight}) == 6, "tab right -> MOVEFORWARD (6)");
        m.goTab(Menu::TabMap);
        m.focus_ = 0;
        expect(press({MenuKey::Back}) == 0 && m.takeSound() == -1, "Back -> BACK (0), the bits clear");
        m.goTab(Menu::TabMap);
        expect(press({MenuKey::Up, MenuKey::Accept}) == 3, "Accept beats Up in one frame -> CONFIRM (3)");
        m.goTab(Menu::TabMap);
        expect(press({MenuKey::Right, MenuKey::Up}) == 7, "Up beats Right -> 7");
        m.goTab(Menu::TabMap);
        expect(press({MenuKey::Accept, MenuKey::Back}) == 0, "Back beats Accept -> 0");
        m.goTab(Menu::TabMap);
        m.soundBit(0x40);
        expect(m.takeSound() == 1, "bit 0x40 -> CANCEL (1)");
        m.soundBit(0x80);
        expect(m.takeSound() == 4, "bit 0x80 -> INVALID (4)");
        m.soundBit(0x100);
        expect(m.takeSound() == -1, "bit 0x100 (the GPS key) has no sound");
        m.soundBit(8);
        expect(m.takeSound() == 7, "bit 8 (hover down) -> NAVIGATIONOPTIONS (7)");
        // The confirmation popup has no Back test (0x4cdfa0).
        m.goTab(Menu::TabMap);
        m.openConfirm("TT_ARE_YOU_SURE_QUIT", {}, {});
        expect(press({MenuKey::Back}) == -1, "popup: Back makes no sound");
        expect(press({MenuKey::Accept}) == 3, "popup: Accept -> CONFIRM (3)");
        // The bonus-code keyboard: backspace on an empty entry -> INVALID, else CANCEL.
        m.showPage("BonusCodes:1");
        m.bonus_.entered.clear();
        expect(press({MenuKey::Vali2}) == 4, "bonus keyboard: backspace on an empty code -> INVALID (4)");
        m.bonus_.entered = "12";
        expect(press({MenuKey::Vali2}) == 1, "bonus keyboard: backspace -> CANCEL (1)");
        // 5. Focus jitter (0x46f6f0): the focused left-menu item moves by -1..1 each update; shots (dt 0) stay still.
        m.showPage("Garage");
        m.update(0);
        const auto f0 = m.focusables();
        const bool still = !f0.empty() && f0[0]->offX == 0 && f0[0]->offY == 0;
        int moved = 0, range = 0;
        for (int i = 0; i < 200; ++i) {
            m.update(1.0f / 60);
            const auto f = m.focusables();
            if (f.empty()) break;
            moved += f[0]->offX != 0 || f[0]->offY != 0;
            range += std::fabs(f[0]->offX) <= 1 && std::fabs(f[0]->offY) <= 1;
        }
        expect(still && moved > 100 && range == 200, "focus jitter: integer offsets in [-1, 1] (" + std::to_string(moved) + " of 200 frames moved), none at dt 0");
    }
    std::printf("chrome check: %d checks, %d problems\n", n, bad);
    return bad;
}

}  // namespace fuel::ui
