#include "ui/menu.h"

#include "core/assembly.hpp"
#include "core/dpc.hpp"
#include "core/vehicle_params.hpp"
#include "core/events.hpp"
#include "ui/tsc.hpp"
#include "ui/ui_renderer.h"

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

// Text-table placeholders for console buttons ("STR_MENU_VALID"): the PC build swaps
// in key names; the port uses plain names.
std::string pcButtons(std::string s) {
    static const std::pair<const char*, const char*> kMap[] = {{"STR_MENU_VALID", "ENTER"}, {"STR_MENU_BACK", "ESC"}, {"STR_MENU_L1", "Q"},
                                                               {"STR_MENU_R1", "E"}};
    for (const auto& [from, to] : kMap)
        for (size_t p; (p = s.find(from)) != std::string::npos;) s.replace(p, std::strlen(from), to);
    return s;
}

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
    for (const auto& [k, v] : readKeyValues(path)) {
        const bool b = v == "1" || v == "true";
        if (k == "hud_race_info") o.raceInfo = b;
        else if (k == "hud_ranking") o.rankingInfo = b;
        else if (k == "hud_gps") o.gps = b;
        else if (k == "hud_speed_meter") o.speedMeter = b;
        else if (k == "hud_miles") o.miles = b;
        else if (k == "brightness") o.brightness = std::clamp(float(std::atof(v.c_str())), 0.0f, 1.0f);
        else if (k == "music_volume") o.musicVolume = std::clamp(float(std::atof(v.c_str())), 0.0f, 1.0f);
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
        {"hud_speed_meter", o.speedMeter ? "1" : "0"}, {"hud_miles", o.miles ? "1" : "0"}, {"brightness", num(o.brightness)},
        {"music_volume", num(o.musicVolume)}};
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
    text_ = loadText(dataDir, settings.language);
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
        std::ifstream pf(findPathNoCase((fs::path(dataDir) / "CommonComponent").string(), "Param\\VehiclesParam.tsc"));
        std::stringstream text;
        text << pf.rdbuf();
        const auto params = parseVehiclesParam(text.str());
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
    setMode(mode);
}

Menu::~Menu() = default;

std::string Menu::textOf(uint32_t tt) const {
    const std::string* s = text_.get(tt);
    return s ? pcButtons(*s) : std::string();
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
    for (const Event& e : events_)
        if (sessionHub_.empty() && e.id == event) sessionHub_ = upper(e.hub);
    if (!vehicle.empty()) vehicle_ = vehicle;
    for (size_t i = 0; i < hubs_.size(); ++i)
        if (upper(hubs_[i].id) == sessionHub_) hubIndex_ = int(i);
}

void Menu::setResults(const std::vector<ResultRow>& rows) { results_ = rows; }

void Menu::setMode(MenuMode m) {
    mode_ = m;
    request_ = {};
    if (m == MenuMode::Title) screen_ = Screen::Title;
    else if (m == MenuMode::Results) screen_ = Screen::Results, focus_ = 0;
    else goTab(m == MenuMode::Pause ? TabLast : TabMap);
}

bool Menu::tabAvailable(int tab) const {
    if (tab == TabLast) return mode_ == MenuMode::Pause || mode_ == MenuMode::Results;
    return tab == TabGarage || tab == TabMap || tab == TabOptions;  // Profile, Editor, Online: not in the port
}

void Menu::setTitle(const std::string& key) {
    if (Page* p = scripts_.page("TopTitle"))
        if (Item* t = p->find("ITEM_ID_TITLE"))
            for (size_t i = 0; i < t->entries.size(); ++i)
                if (t->entries[i].key == key) t->entryIndex = int(i);
}

void Menu::goTab(int tab) {
    tab_ = std::clamp(tab, 0, TabCount - 1);
    screen_ = Screen::Tab;
    focus_ = 0;
    setTitle(kTabTitle[tab_]);
    if (tab_ == TabGarage && !pendingEvent_.empty()) setTitle("TT_TOP_TITLE_GARAGE_RACE");
}

Page* Menu::leftPage() {
    if (screen_ != Screen::Tab) return nullptr;
    return scripts_.page(tab_ == TabMap ? "TemplateLeftMenu" : kTabPage[tab_]);
}

std::vector<Item*> Menu::focusables() {
    std::vector<Item*> out;
    Page* p = leftPage();
    if (!p) return out;
    for (Item& it : p->items)
        if (it.isButton() && it.group == "GROUP_LEFT_MENU" && it.visible && it.enabled) out.push_back(&it);
    std::sort(out.begin(), out.end(), [](const Item* a, const Item* b) { return a->y < b->y; });
    return out;
}

std::vector<const Event*> Menu::hubEvents(Event::Kind kind) const {
    std::vector<const Event*> out;
    const std::string hub = hubs_.empty() ? std::string() : upper(hubs_[size_t(hubIndex_)].id);
    for (const Event& e : events_)
        if (e.kind == kind && upper(e.hub) == hub) out.push_back(&e);
    return out;
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
    for (const VehicleInfo& v : vehicles_) {
        if (v.family == "UNKNOWN") continue;  // test and traffic vehicles (VEH_TEST_*, TRAFFIC_TRUCK)
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
    } else if (s == Screen::OptAudio) {
        setTitle("TT_TOP_TITLE_OPTION_AUDIO");
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
    if (n == "TOPMENU") return goTab(TabMap), true;
    if (n == "PRESSSTART" || n == "TITLE") return setMode(MenuMode::Title), true;
    if (n == "HUBSELECTCAREER") return openList(Screen::CareerList), true;
    if (n == "HUBSELECTCHALLENGES") return openList(Screen::ChallengeList), true;
    if (n == "HUBSELECTCAMPS") return openList(Screen::CampList), true;
    if (n == "GARAGEVEHICLES" || n == "VEHICLELIST") return openList(Screen::VehicleList), true;
    if (n == "PAGEOPTIONSDISPLAY") return openList(Screen::OptDisplay), true;
    if (n == "PAGEOPTIONSAUDIO") return openList(Screen::OptAudio), true;
    if (n == "RACE_RANKING_WIN" || n == "RESULTS") {
        if (results_.empty()) results_ = {{"Player", 245.31f, true}, {"Rival 1", 247.02f, false}, {"Rival 2", 251.9f, false}};
        mode_ = MenuMode::Results, screen_ = Screen::Results;
        return true;
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
    return r;
}

void Menu::saveOptions() {
    if (!saveSettings(settings_)) std::fprintf(stderr, "menu: cannot save %s\n", settingsPath().c_str());
    saveMenuOptions(options_);
}

void Menu::back() {
    switch (screen_) {
        case Screen::Title: break;
        case Screen::Tab:
            if (mode_ == MenuMode::Pause) request_.kind = MenuRequest::Resume;
            break;
        case Screen::CareerList:
        case Screen::ChallengeList:
        case Screen::CampList: goTab(TabMap); break;
        case Screen::VehicleList:
            if (!pendingEvent_.empty()) {
                pendingEvent_.clear();
                goTab(TabMap);
            } else {
                goTab(TabGarage);
            }
            break;
        case Screen::OptDisplay:
        case Screen::OptAudio:
            saveOptions();
            goTab(TabOptions);
            break;
        case Screen::Results: goTab(TabLast), mode_ = MenuMode::Pause; break;
        case Screen::Single: break;
    }
}

void Menu::accept(const std::string& id) {
    if (id == "ITEM_ID_TOPMENU_RACE_CONTINUE") request_.kind = MenuRequest::Resume;
    else if (id == "ITEM_ID_TOPMENU_RACE_RESTART") request_.kind = MenuRequest::Restart;
    else if (id == "ITEM_ID_TOPMENU_RACE_GO_FREE_RIDE") {
        request_ = {MenuRequest::FreeRide, "", sessionHub_.empty() && !hubs_.empty() ? hubs_[size_t(hubIndex_)].id : sessionHub_, vehicle_};
    } else if (id == "ITEM_ID_LEFT_MENU5") openList(Screen::CareerList);
    else if (id == "ITEM_ID_LEFT_MENU4") openList(Screen::ChallengeList);
    else if (id == "ITEM_ID_LEFT_MENU3") openList(Screen::CampList);
    else if (id == "ITEM_ID_LEFT_MENU1") request_.kind = mode_ == MenuMode::Pause ? MenuRequest::QuitToTitle : MenuRequest::QuitGame;
    else if (id == "ITEM_ID_TOPMENU_GARAGE_VEHICLE") openList(Screen::VehicleList);
    else if (id == "ITEM_ID_TOPMENU_GARAGE_START_RACE") {
        if (!pendingEvent_.empty()) request_ = {MenuRequest::StartEvent, pendingEvent_, "", vehicle_};
        else if (!hubs_.empty()) request_ = {MenuRequest::FreeRide, "", hubs_[size_t(hubIndex_)].id, vehicle_};
    } else if (id == "ITEM_ID_TOPMENU_OPTIONS_DISPLAY") openList(Screen::OptDisplay);
    else if (id == "ITEM_ID_TOPMENU_OPTIONS_AUDIO") openList(Screen::OptAudio);
}

void Menu::listMove(int d) {
    int count = 0, rows = 1;
    switch (screen_) {
        case Screen::CareerList: count = int(hubEvents(Event::Career).size()), rows = 3; break;
        case Screen::ChallengeList: count = int(hubEvents(Event::Challenge).size()), rows = 5; break;
        case Screen::CampList: count = int(hubs_.size()), rows = 10; break;
        case Screen::VehicleList: count = int(vehicleChoices().size()), rows = 10; break;
        case Screen::OptDisplay: count = 9, rows = 9; break;
        case Screen::OptAudio: count = 2, rows = 2; break;
        default: return;
    }
    if (count == 0) return;
    focus_ = (focus_ + d + count) % count;
    listFirst_ = (focus_ / rows) * rows;
}

void Menu::listPage(int d) {
    if (screen_ == Screen::OptDisplay || screen_ == Screen::OptAudio) {
        // Left/Right change the focused option.
        auto flip = [&](bool& b) { b = !b; };
        if (screen_ == Screen::OptAudio) {
            float& v = focus_ == 0 ? settings_.volume : options_.musicVolume;
            v = std::clamp(std::round((v + 0.1f * float(d)) * 10.0f) / 10.0f, 0.0f, 1.0f);
            return;
        }
        switch (focus_) {
            case 0: flip(options_.raceInfo); break;
            case 1: flip(options_.rankingInfo); break;
            case 2: flip(options_.gps); break;
            case 3: flip(options_.speedMeter); break;
            case 4: flip(options_.miles); break;
            case 5: options_.brightness = std::clamp(std::round((options_.brightness + 0.1f * float(d)) * 10.0f) / 10.0f, 0.0f, 1.0f); break;
            case 6: flip(settings_.postEffects); break;
            case 7: flip(settings_.grass); break;
            case 8: flip(settings_.fullscreen); break;
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
    const int pages = (count + rows - 1) / rows;
    const int page = ((listFirst_ / rows) + d + pages) % pages;
    listFirst_ = page * rows;
    focus_ = std::min(count - 1, listFirst_ + (focus_ % rows));
}

void Menu::key(MenuKey k) {
    if (screen_ == Screen::Title) {
        if (k == MenuKey::Accept) setMode(MenuMode::Main);
        else if (k == MenuKey::Back) request_.kind = MenuRequest::QuitGame;
        return;
    }
    if (screen_ == Screen::Results) {
        if (k == MenuKey::Accept || k == MenuKey::Back) back();
        return;
    }
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
        if ((k == MenuKey::Left || k == MenuKey::Right) && tab_ == TabMap && !hubs_.empty())
            hubIndex_ = (hubIndex_ + (k == MenuKey::Left ? -1 : 1) + int(hubs_.size())) % int(hubs_.size());
        if (k == MenuKey::Accept && focus_ < int(f.size())) accept(f[size_t(focus_)]->id);
        return;
    }
    if (screen_ == Screen::Single) return;
    if (k == MenuKey::Up) listMove(-1);
    if (k == MenuKey::Down) listMove(1);
    if (k == MenuKey::Left) listPage(-1);
    if (k == MenuKey::Right) listPage(1);
    if (k != MenuKey::Accept) return;
    switch (screen_) {
        case Screen::CareerList:
        case Screen::ChallengeList: {
            const auto list = hubEvents(screen_ == Screen::CareerList ? Event::Career : Event::Challenge);
            if (focus_ < int(list.size()) && list[size_t(focus_)]->playable) {
                pendingEvent_ = list[size_t(focus_)]->id;
                openList(Screen::VehicleList);
            }
            break;
        }
        case Screen::CampList:
            if (focus_ < int(hubs_.size())) {
                hubIndex_ = focus_;
                request_ = {MenuRequest::FreeRide, "", hubs_[size_t(focus_)].id, vehicle_};
            }
            break;
        case Screen::VehicleList: {
            const auto list = vehicleChoices();
            if (focus_ < int(list.size())) {
                vehicle_ = list[size_t(focus_)]->name;
                if (!pendingEvent_.empty()) request_ = {MenuRequest::StartEvent, pendingEvent_, "", vehicle_};
                else goTab(TabGarage);
            }
            break;
        }
        case Screen::OptDisplay:
        case Screen::OptAudio: listPage(1); break;
        default: break;
    }
}

void Menu::pointer(float x, float y, bool click) {
    pointerX_ = x, pointerY_ = y;
    auto inside = [&](const Item& it) { return x >= it.x && x < it.x + it.w && y >= it.y && y < it.y + it.h; };
    if (screen_ == Screen::Title || screen_ == Screen::Results) {
        if (click) key(MenuKey::Accept);
        return;
    }
    if (Page* top = scripts_.page("TopMenu"))
        for (int t = 0; t < TabCount; ++t)
            if (const Item* icon = top->find(kTabIcon[t]); icon && inside(*icon)) {
                if (click && tabAvailable(t)) goTab(t);
                return;
            }
    if (screen_ == Screen::Tab) {
        const auto f = focusables();
        for (size_t i = 0; i < f.size(); ++i)
            if (inside(*f[i]) || (x >= f[i]->x && x < f[i]->x + 420 && y >= f[i]->y && y < f[i]->y + f[i]->h)) {
                focus_ = int(i);
                if (click) accept(f[i]->id);
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
            if (idx >= 0) {
                const int before = focus_;
                focus_ = idx;
                if (click && before == idx) key(MenuKey::Accept);
            }
            return;
        }
        if (click && it.id == "ITEM_ID_FLECHE_GAUCHE") return listPage(-1);
        if (click && it.id == "ITEM_ID_FLECHE_DROITE") return listPage(1);
    }
}

void Menu::update(float dt) { time_ += dt; }

// ---- drawing ---------------------------------------------------------------

void Menu::drawItem(UiRenderer& r, const Item& it, float dx, float dy) {
    if (!it.visible) return;
    // Safe-frame snapping (FUEL.exe 0x49c0a0, mode 1 = 90 % frame on PC).
    float x = it.x + dx, y = it.y + dy;
    if (it.anchorH == 1) x += 64;
    if (it.anchorH == 2) x -= 64;
    if (it.anchorV == 1 || it.anchorV == 3) y += 36;
    if (it.anchorV == 2 || it.anchorV == 4) y -= 36;
    const int state = it.focus ? 2 : it.enabled ? 1 : 0;
    if (it.kind == ItemKind::Image) {
        const std::string& name = state == 2 ? it.imgFocus : state == 1 ? it.imgEnabled : it.imgNormal;
        const Rgba& c = state == 2 ? it.cFocus : state == 1 ? it.cEnabled : it.cNormal;
        r.image(assets_->image(name), x, y, it.w, it.h, c);
        return;
    }
    if (it.kind == ItemKind::FillRect) {
        r.rect(x, y, it.w, it.h, state == 2 ? it.cFocus : state == 1 ? it.cEnabled : it.cNormal);
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
    r.textFit(s, x + it.xOffset, y, it.w, st);
}

void Menu::drawPage(UiRenderer& r, Page& p, float dx, float dy) {
    for (const Item& it : p.items) drawItem(r, it, dx, dy);
}

void Menu::applyItemStates() {
    // Top bar: available tabs enabled, the current one focused with its halo.
    if (Page* top = scripts_.page("TopMenu")) {
        for (int t = 0; t < TabCount; ++t) {
            if (Item* icon = top->find(kTabIcon[t])) icon->enabled = tabAvailable(t), icon->focus = t == tab_;
            if (Item* halo = top->find(std::string(kTabIcon[t]) + "_HALO")) halo->visible = t == tab_, halo->focus = true;
        }
    }
    if (Page* m = scripts_.page("Map")) {
        if (Item* hub = m->find("ITEM_ID_TITLE_HUB_INFO"); hub && !hubs_.empty()) {
            const std::string key = "TT_HUB_" + upper(hubs_[size_t(hubIndex_)].id);
            for (size_t i = 0; i < hub->entries.size(); ++i)
                if (upper(hub->entries[i].key) == key) hub->entryIndex = int(i);
        }
    }
    // Left menus: what the port can do is enabled; the rest keeps the authored look, disabled.
    static const char* kOff[] = {"ITEM_ID_LEFT_MENU2",             "ITEM_ID_TOPMENU_GARAGE_DRIVER",   "ITEM_ID_TOPMENU_GARAGE_LIVERY",
                                 "ITEM_ID_TOPMENU_GARAGE_COLOR",   "ITEM_ID_TOPMENU_OPTIONS_CONTROLLER", "ITEM_ID_TOPMENU_OPTIONS_BONUS_CODES",
                                 "ITEM_ID_TOPMENU_OPTIONS_CREDITS"};
    for (Page& p : scripts_.pages)
        for (Item& it : p.items) {
            it.focus = false;
            for (const char* off : kOff)
                if (it.id == off) it.enabled = false;
            if (it.id == "ITEM_ID_TOPMENU_RACE_RESTART") it.enabled = !sessionEvent_.empty();
        }
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
            if (Item* it = p->find("ITEM_ID_MEDAL_SLOT" + n)) it->visible = false;   // no career progress yet
            if (Item* it = p->find("ITEM_ID_FUEL_REWARD" + n)) it->visible = false;
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
                    const Hub& h = hubs_[size_t(idx)];
                    it->dynText = h.nameText ? textOf(h.nameText) : h.id;
                } else {
                    const VehicleInfo& v = *vehicles[size_t(idx)];
                    const std::string nm = textOf(v.nameText);
                    it->dynText = nm.empty() ? v.name : nm;
                }
                it->enabled = true;
            }
            if (Item* it = p->find("ITEM_ID_HALO" + n)) it->visible = idx == focus_, it->focus = true;
        }
        for (const char* a : {"ITEM_ID_FLECHE_GAUCHE", "ITEM_ID_FLECHE_DROITE"})
            if (Item* it = p->find(a)) it->visible = count > 10;
    } else if (screen_ == Screen::Results) {
        Page* p = scripts_.page("race_ranking_win");
        if (!p) return;
        for (Item& it : p->items) it.visible = true;
        for (int k = 1; k <= 5; ++k) {
            const bool on = k <= int(results_.size());
            setRow(*p, k, on);
            if (!on)  // the row's unnumbered bars too (those inside the row's halo band)
                if (const Item* halo = p->find("ITEM_ID_HALO" + std::to_string(k)))
                    for (Item& it : p->items)
                        if (rowOf(it.id) == 0 && it.y + it.h * 0.5f > halo->y && it.y + it.h * 0.5f < halo->y + halo->h) it.visible = false;
            if (!on) continue;
            const ResultRow& rr = results_[size_t(k - 1)];
            const std::string n = std::to_string(k);
            auto dyn = [&](const std::string& id, const std::string& s) {
                if (Item* it = p->find(id)) it->hasDynText = true, it->dynText = s, it->enabled = rr.player;
            };
            dyn("ITEM_ID_PLAYER_POS" + n, std::to_string(k));
            dyn("ITEM_ID_PLAYER_NAME" + n, vehicleName(rr.name));
            dyn("ITEM_ID_PLAYER_TIME" + n, formatTime(rr.time));
            if (Item* it = p->find("ITEM_ID_HALO" + n)) it->visible = rr.player, it->focus = true;
            if (Item* it = p->find("ITEM_ID_PLAYER_VEHICLE" + n)) it->visible = false;
        }
        for (const char* a : {"ITEM_ID_FLECHE_UP", "ITEM_ID_FLECHE_DOWN"})
            if (Item* it = p->find(a)) it->visible = results_.size() > 5;
        std::string title = sessionEvent_;
        for (const Event& e : events_)
            if (e.id == sessionEvent_ && e.nameText) title = textOf(e.nameText);
        if (Item* it = p->find("UNDEFINED_ITEM_ID")) it->hasDynText = true, it->dynText = title;
        if (Item* it = p->find("ITEM_ID_STATUS_MSG")) it->hasDynText = true, it->dynText = textOf(235), it->align = 1;
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
            if (Item* it = p->find(a)) it->y = 419.0f + 49.0f * float(focus_ - 4) + (focus_ >= 6 ? -1.0f : 0.0f);
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
            if (Item* it = p->find(a)) it->y = 307.0f + 60.0f * float(focus_);
        for (Item& it : p->items) {
            const int row = rowOf(it.id);
            if (it.id.rfind("ITEM_ID_HALO", 0) == 0) it.visible = row - 1 == focus_, it.focus = true;
        }
    }
}

void Menu::draw(UiRenderer& r) {
    applyItemStates();
    prepareRows();
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
        const float pulse = 0.65f + 0.35f * std::cos(time_ * 3.0f);
        st.fill.a = st.outline.a = pulse;
        r.textFit(textOf(230), 0.6f * 1280, 0.8f * 720 + 17, 0.3f * 1280, st);
        return;
    }
    if (screen_ == Screen::Single) {
        page(singlePage_.c_str());
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
                page("Map");
                page("TemplateLeftMenu");
            } else {
                page(kTabPage[tab_]);
            }
            break;
        case Screen::CareerList: page("HubSelectCareer"); break;
        case Screen::ChallengeList: page("HubSelectChallenges"); break;
        case Screen::CampList:
        case Screen::VehicleList: page("HubSelectCamps"); break;
        case Screen::OptDisplay: {
            page("PageOptionsDisplay");
            // Port rows below the original six, in the same style (row 1 moved down):
            // the fuel::Settings switches the launcher also edits.
            Page* p = scripts_.page("PageOptionsDisplay");
            if (!p) break;
            const char* labels[3] = {"Post effects", "Grass", "Full screen"};
            const bool vals[3] = {settings_.postEffects, settings_.grass, settings_.fullscreen};
            for (int k = 0; k < 3; ++k) {
                const float dy = 49.0f * float(6 + k) - 1.0f;
                for (const Item& src : p->items) {
                    if (src.y < 200 || src.y > 230) continue;  // row 1 items
                    if (src.id.rfind("ITEM_ID_VALID_BOX", 0) == 0) continue;
                    Item it = src;
                    if (it.id == "ITEM_ID_HALO1") it.visible = focus_ == 6 + k, it.focus = true;
                    if (it.id == "ITEM_ID_DISPLAY_NAME1") it.hasDynText = true, it.dynText = labels[k];
                    if (it.kind == ItemKind::ListBoxTT) it.entryIndex = vals[k] ? 1 : 0, it.focus = focus_ == 6 + k;
                    drawItem(r, it, 0, dy);
                }
            }
            break;
        }
        case Screen::OptAudio: page("PageOptionsAudio"); break;
        case Screen::Results: page("race_ranking_win"); break;
        default: break;
    }
}

void Menu::drawHud(UiRenderer& r, const HudState& hud) {
    applyItemStates();
    if (options_.speedMeter)
        if (Page* p = scripts_.page("compteur")) {
            const float shown = options_.miles ? hud.speedKmh * 0.621371f : hud.speedKmh;
            const float dist = options_.miles ? hud.odometerKm * 0.621371f : hud.odometerKm;
            for (Item& it : p->items) {
                if (it.id == "ITEM_ID_SPEED") it.hasDynText = true, it.dynText = std::to_string(int(std::lround(std::fabs(shown))));
                if (it.id == "ITEM_ID_METER_NBKM") it.hasDynText = true, it.dynText = std::to_string(int(dist));
                if (it.id == "ITEM_ID_METER_NB100M") it.hasDynText = true, it.dynText = std::to_string(int(dist * 10.0f) % 10);
                if (it.id == "ITEM_ID_SPEED_UNITS") it.entryIndex = options_.miles ? 1 : 0;
            }
            drawPage(r, *p);
        }
    if (options_.gps)
        if (Page* p = scripts_.page("boussole")) {
            for (Item& it : p->items) {
                if (it.id == "ITEM_ID_HUB_NAME") it.hasDynText = true, it.dynText = hud.hubName;
                if (it.id == "ITEM_ID_STATUS") it.visible = false;
            }
            drawPage(r, *p);
        }
    if (hud.race && options_.rankingInfo)
        if (Page* p = scripts_.page("classement")) {
            // Up to four lines: the leader, then the player's neighbours (the game's
            // choice of lines is not decoded).
            std::vector<size_t> show;
            for (size_t i = 0; i < hud.standings.size() && show.size() < 4; ++i) show.push_back(i);
            for (size_t i = 0; i < hud.standings.size(); ++i)
                if (hud.standings[i].player && i >= show.size()) show.back() = i;
            for (Item& it : p->items) {
                const int row = rowOf(it.id);
                if (row < 1) continue;
                const bool on = size_t(row) <= show.size();
                it.visible = on;
                if (!on) continue;
                const size_t i = show[size_t(row - 1)];
                const ResultRow& rr = hud.standings[i];
                if (it.id.find("NUMBER") != std::string::npos) it.hasDynText = true, it.dynText = std::to_string(i + 1);
                if (it.id.find("NAME") != std::string::npos) it.hasDynText = true, it.dynText = vehicleName(rr.name);
                if (it.id.find("DIST") != std::string::npos) {
                    char b[32];
                    std::snprintf(b, sizeof b, rr.time > 0 ? "+%.1f" : "", rr.time);
                    it.hasDynText = true, it.dynText = b;
                }
                it.enabled = rr.player;  // RANKING_PLAYER (enabled image) vs RANKING_OPPONENT
            }
            drawPage(r, *p);
        }
}

}  // namespace fuel::ui
