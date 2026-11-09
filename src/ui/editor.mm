// PK21: the pause menu's Editor tab (InGamePauseHUD +0x934 states 5 / 0xa1..0xac, EditorHUD +0x49c; notes/pk21.md,
// decode p98 b030 / b040 / b041). The record rules live in core/race_editor.hpp; this file is the menu flow and drawing.
//   per frame     0x475fa0: 5 top 0x4761d0, 0xa1..0xa4 races list 0x476450 / 0x476510 / 0x4765c0 / 0x476670 (keys
//                 0x4762b0), 0xa5 map editing 0x4769b0, 0xa6 yes / no 0x4cdfa0, 0xa7 0x4767f0, 0xa8 0x4768e0, 0xa9 / 0xaa
//                 0x476860, 0xab 0x4ce400, 0xac storage wait then 0x475ce0
//   titles        0x477a80 (TopTitle / buttons / help per state), 0x475940 (map editing: TopTitleDyn "EDITED RACE %02d")
//   draw          0x476b30: 5 Editor, 0xa1..0xa4 EditorRacesList (fill 0x477480, subtitle 0x4771b0), 0xa5 the EditorHUD map
//                 (0x467090: roads, hub zones, route 0x4678d0) + Map_Cache + EditorMapInfos (0x4772c0), popups PopUpHelp
//                 (+ PopUpInput_YesNo 0xa6, PopUpRaceDeleted 0xa7, PopUpInput_Continue 0xab)
// Keys of the map editing (TT 492..495): MENU_VALID add a point, MENU_CANCEL (PC E) delete, MENU_VALI2 (R) select / drop,
// R2 / L2 zoom, MENU_BACK finish editing. [port] F / G (the map's action / GPS keys) also add / delete; I J K L and the
// arrows move the cursor, = / - zoom.
#include "ui/menu.h"

#include "core/dpc.hpp"
#include "core/hub_zones.hpp"
#include "core/road_graph.hpp"
#include "core/road_types.hpp"
#include "ui/ui_renderer.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <mutex>
#include <strings.h>

namespace fs = std::filesystem;

namespace fuel::ui {

// The world for a menus process without a session (MenuMode::Main, --menu-shot, --career-audit): USA1.DPC's road graph,
// road types and hub zones, loaded once per process.
struct Menu::EditorWorldData {
    std::unique_ptr<fuel::RoadGraph> graph;
    std::optional<std::array<fuel::RoadTypeParams, 21>> types;
    fuel::HubZones zones;
};

namespace {
constexpr int kTop = 5, kListCreate = 0xa1, kListEdit = 0xa2, kListDelete = 0xa3, kListTry = 0xa4, kMap = 0xa5, kDeleteYesNo = 0xa6,
              kDeleted = 0xa7, kValid = 0xa8, kInvalidKm = 0xa9, kInvalidMi = 0xaa, kTooLong = 0xab, kSaving = 0xac;
const char* const kActions[4] = {"ITEM_ID_TOPMENU_EDITOR_CREATE_RACE", "ITEM_ID_TOPMENU_EDITOR_EDIT_RACE", "ITEM_ID_TOPMENU_EDITOR_DELETE_RACE",
                                 "ITEM_ID_TOPMENU_EDITOR_TRY_RACE"};

std::string fmtText(const std::string& fmt, int a, int b = 0) {
    char buf[256];
    std::snprintf(buf, sizeof buf, fmt.empty() ? "%d" : fmt.c_str(), a, b);
    return buf;
}

std::shared_ptr<Menu::EditorWorldData>* worldCache() {
    static std::shared_ptr<Menu::EditorWorldData> w;
    return &w;
}
}  // namespace

fuel::EditorWorld Menu::editorWorld() {
    if (!roads_ && !edData_) {
        auto* cache = worldCache();
        if (!*cache) {
            auto d = std::make_shared<EditorWorldData>();
            try {
                const DpcFile usa1((fs::path(dataDir_) / "CommonComponent/WORLD/USA1.DPC").string());
                d->types = fuel::loadRoadTypes(usa1);
                d->graph = std::make_unique<fuel::RoadGraph>(usa1);
                d->zones = fuel::loadHubZones(usa1);
            } catch (const std::exception& e) {
                std::fprintf(stderr, "editor: world: %s\n", e.what());
            }
            *cache = d;
        }
        edData_ = *cache;
        // [port] the menus process has no session world: the editor's (and the Map tab's) roads and zones come from here.
        if (edData_->graph) roads_ = edData_->graph.get();
        if (edData_->types) roadTypes_ = &*edData_->types;
        if (!zones_ && !edData_->zones.zones.empty()) zones_ = &edData_->zones;
    }
    fuel::EditorWorld w;
    w.graph = roads_;
    if (const auto* t = roadTypes_) w.flatHalf = [t](int type) { return type >= 0 && type < 21 ? (*t)[size_t(type)].flat : 0.0f; };
    w.ground = ground_;
    const fuel::HubZones* z = zones_;
    Career* c = career_.get();
    // 0x464ac0: the hub of the point (0x4a7f40 / 0x577500) must pass the unlock test 0x579720.
    w.hubOpen = [z, c](float x, float zz) {
        if (!z || !c) return false;
        const int i = z->find(x, zz);
        if (i < 0) return false;
        const int h = c->rules().hubIndex(z->zones[size_t(i)].name);
        return h >= 0 && c->hubUnlockTest(h);
    };
    return w;
}

// 0x4770b0: CREATE iff a slot is free, EDIT / DELETE / TRY iff one is used.
void Menu::editorItemStates() {
    Page* p = scripts_.page("Editor");
    if (!p || !career_) return;
    const int free = fuel::editor::freeSlots(career_->state().editedRaces);
    for (int a = 0; a < 4; ++a)
        if (Item* it = p->find(kActions[a])) it->enabled = a == 0 ? free != 0 : free != 10;
}

// 0x475720: +0x934 = st (5 or >= 0xa1), the top bar (0x4704b0: locked while the map is edited), titles 0x477a80.
void Menu::editorSetState(int st) {
    ed_.state = st;
    if (st == kTop || (st >= kListCreate && st <= kMap)) ed_.base = st;
    static const char* const kTitles[4] = {"TT_TOP_TITLE_EDITOR_CREATE_RACE", "TT_TOP_TITLE_EDITOR_EDIT_RACE", "TT_TOP_TITLE_EDITOR_DELETE_RACE",
                                           "TT_TOP_TITLE_EDITOR_TRY_RACE"};
    if (st == kTop) setTitle("TT_TOP_TITLE_TOP_EDITOR");
    else if (st >= kListCreate && st <= kListTry) setTitle(kTitles[st - kListCreate]);
}

// 0x475e70: the tab opens on the top menu, row 0, the first enabled action focused.
void Menu::editorEnter() {
    ed_.row = 0;
    if (Page* p = scripts_.page("EditorRacesList")) {  // +0x93c = the race rows of the list page (0x60f410)
        int n = 0;
        for (int k = 1; k <= 10; ++k) n += p->find("ITEM_ID_RACE_NAME" + std::to_string(k)) != nullptr;
        ed_.perPage = std::max(1, n);
    }
    editorItemStates();
    focus_ = 0;
    ed_.action = -1, ed_.slot = -1;
    editorSetState(kTop);
}

// 0x475ce0: back to the top menu (row 0, slot reselected, enables).
void Menu::editorTop() {
    ed_.row = 0;
    editorSelectSlot();
    editorItemStates();
    editorSetState(kTop);
}

// 0x4756d0: the action of the focused top item.
int Menu::editorFocusAction() {
    if (tab_ != TabEditor) return -1;
    const auto f = focusables();
    if (focus_ < 0 || focus_ >= int(f.size())) return -1;
    for (int a = 0; a < 4; ++a)
        if (f[size_t(focus_)]->id == kActions[a]) return a;
    return -1;
}

// 0x476ad0: CREATE -> the first free slot, EDIT / DELETE / TRY -> the row-th used one.
void Menu::editorSelectSlot() {
    if (!career_) return void(ed_.slot = -1);
    const auto& b = career_->state().editedRaces;
    ed_.slot = ed_.action == 0 ? fuel::editor::firstFreeSlot(b) : ed_.action >= 1 && ed_.action <= 3 ? fuel::editor::usedSlot(b, ed_.row) : -1;
}

// 0x475650: run the focused action (CREATE: the map from the top menu, else back to the top).
void Menu::editorRun(bool fromTop) {
    switch (ed_.action) {
        case 0: fromTop ? editorOpenMap() : editorTop(); break;
        case 1: editorList(kListEdit); break;
        case 2: editorList(kListDelete); break;
        case 3: editorList(kListTry); break;
        default: break;
    }
}

// 0x4757a0 / 0x4757d0 / 0x475800: subtitle 0x4771b0, the list 0x477480 (row 0 when past the used slots), state.
void Menu::editorList(int st) {
    if (career_ && ed_.row >= 10 - fuel::editor::freeSlots(career_->state().editedRaces)) ed_.row = 0;
    editorSetState(st);
}

// 0x4758b0: the map editor on the selected slot (copy +0x4a0 for "quit without saving", EditorHUD 0x4633a0 / 0x4630a0).
void Menu::editorOpenMap() {
    editorSelectSlot();
    if (!career_ || ed_.slot < 0) return;
    auto& block = career_->state().editedRaces;
    const size_t off = size_t(ed_.slot) * fuel::EditedRace::kBytes;
    ed_.backup.assign(block.begin() + long(off), block.begin() + long(off + fuel::EditedRace::kBytes));
    ed_.race = fuel::editor::slot(block, ed_.slot);
    float px = map_.px, pz = map_.pz;  // the player (0x421f60); [port] none in the menus process: the current hub's camp
    if (!map_.havePlayer) {
        const int h = career_->currentHub();
        if (h >= 0 && size_t(h) < career_->rules().hubs.size())
            if (const Hub* hub = findHub(hubs_, career_->rules().hubs[size_t(h)].name)) px = float(hub->camp.x), pz = float(hub->camp.z);
    }
    ed_.map.open(&ed_.race, px, pz, editorWorld());
    ed_.map.zoom = ed_.map.zoomTarget = 2000.0f;  // MapHUD ctor (+0x19e8 / +0x19ec)
    editorSetState(kMap);
}

void Menu::editorStore() {
    if (career_ && ed_.slot >= 0) fuel::editor::setSlot(career_->state().editedRaces, ed_.slot, ed_.race);
}

// 0x475870 (Mem::CopyBlock): the copy taken when the map opened goes back into the slot.
void Menu::editorRestore() {
    if (!career_ || ed_.slot < 0 || ed_.backup.size() != fuel::EditedRace::kBytes) return;
    auto& block = career_->state().editedRaces;
    std::copy(ed_.backup.begin(), ed_.backup.end(), block.begin() + long(size_t(ed_.slot) * fuel::EditedRace::kBytes));
    ed_.race = fuel::editor::slot(block, ed_.slot);
}

// 0x475d80: length > 150 km -> invalid (km / miles), >= 2 points -> valid, 1 point -> invalid, none -> invalid unless
// CREATE (then back to the top: nothing to keep).
void Menu::editorValidate() {
    if (!career_) return;
    const fuel::EditedRace r = fuel::editor::slot(career_->state().editedRaces, ed_.slot);
    const bool km = !options_.miles;
    const int invalid = km ? kInvalidKm : kInvalidMi;
    if (fuel::editor::tooLong(fuel::editor::routeLength(r, ed_.map.world))) return editorSetState(invalid);
    if (r.count > 1) return editorSetState(kValid);
    if (r.count == 1 || ed_.action != 0) return editorSetState(invalid);
    editorRun(false);
}

// 0x475b50 (yes of the 0xa6 popup, 0x4ce6a0): the slot's point count = 0, the mission named "Edited Race %02d" blanked
// (0x477dc0; the port keeps no mission list), dirty +0x80 (saved when the pause menu closes, PK17), state 0xa7.
void Menu::editorDelete() {
    editorSelectSlot();
    if (!career_ || ed_.slot < 0) return;
    career_->state().editedRaces[size_t(ed_.slot) * fuel::EditedRace::kBytes + 0x10] = 0;
    markDirty();  // PK17: +0x80 = 1 (saved when the pause menu closes, 0x46e9b0)
    editorSetState(kDeleted);
}

// 0x476670 (TRY, accept on a row): the open trip no longer counts (game +0xe0 + 0xc x +0x144 = 0, 0x56ca00), the slot is
// loaded (0x4633a0), the menus close and the commit 0x465be0 starts the mission "Edited Race %02d" with the player's
// vehicle. [port] the race session is a new process: StartEvent with that name (main.mm builds it from career.sav).
void Menu::editorTry() {
    editorSelectSlot();
    if (!career_ || ed_.slot < 0) return;
    if (!career_->state().trips.empty()) career_->state().trips.back().valid = false;
    request_ = {MenuRequest::StartEvent, fuel::editor::slotName(ed_.slot), "", vehicle_};
}

// 0x4761d0: the action key on a top item -> 0x475650(1).
bool Menu::editorAccept(const std::string& id) {
    for (int a = 0; a < 4; ++a)
        if (id == kActions[a]) {
            ed_.action = a;
            editorRun(true);
            return true;
        }
    return false;
}

// 0x4762b0: Up / Down within the page, Left / Right a page (the right page only when a used slot is there).
bool Menu::editorListKey(MenuKey k) {
    if (!career_) return false;
    const int used = 10 - fuel::editor::freeSlots(career_->state().editedRaces);
    const int per = ed_.perPage, row = ed_.row;
    if (k == MenuKey::Up) {
        if (row % per > 0) return --ed_.row, true;
    } else if (k == MenuKey::Down) {
        const int onPage = std::min(per, used - (row >= per ? per : 0));
        if (row % per < onPage - 1) return ++ed_.row, true;
    } else if (k == MenuKey::Left) {
        if (row >= per) return ed_.row -= per, true;
    } else if (k == MenuKey::Right) {
        if (used <= per || row >= per) return false;
        const int left = std::min(used - per, per);
        ed_.row = fuel::editor::usedSlot(career_->state().editedRaces, per + row) < 0 ? per - 1 + left : per + row;
        return true;
    }
    return false;
}

void Menu::editorKey(MenuKey k) {
    const bool cancel = k == MenuKey::TabRight || k == MenuKey::MapTarget;  // MENU_CANCEL (game +0x20c)
    switch (ed_.state) {
        case kListCreate:
        case kListEdit:
        case kListDelete:
        case kListTry:
            if (k == MenuKey::Back) return editorTop();
            if (k == MenuKey::Accept) {
                if (ed_.state == kListDelete) {  // 0x475ad0 -> 0xa6: yes 0x4ce6a0 (0x475b50), no 0x4ce690 (0x475650: the list)
                    editorSelectSlot();
                    editorSetState(kDeleteYesNo);
                    openConfirm("TT_PAGE_EDITOR_DELETE_THIS_RACE_YESNO", [this] { editorDelete(); }, [this] { editorRun(false); });
                } else if (ed_.state == kListTry) {
                    editorTry();
                } else {
                    editorOpenMap();
                }
                return;
            }
            editorListKey(k);
            return;
        case kMap: {
            if (k == MenuKey::Back) {  // 0x4769b0: finish editing when no point is picked
                if (ed_.map.picked < 0) editorValidate();
                return;
            }
            fuel::editor::Session::Key sk;
            if (k == MenuKey::Accept || k == MenuKey::MapAction) sk = fuel::editor::Session::Add;
            else if (cancel) sk = fuel::editor::Session::Remove;
            else if (k == MenuKey::Vali2) sk = fuel::editor::Session::Pick;
            else {
                // the cursor and zoom keys are held (map_.held; Menu::keyUp releases them)
                MenuKey m = k == MenuKey::Up ? MenuKey::MapUp : k == MenuKey::Down ? MenuKey::MapDown : k == MenuKey::Left ? MenuKey::MapLeft
                          : k == MenuKey::Right ? MenuKey::MapRight : k;
                if (m >= MenuKey::MapUp && m <= MenuKey::MapZoomOut) map_.held |= 1u << (int(m) - int(MenuKey::MapUp));
                return;
            }
            const int res = ed_.map.key(sk);
            editorStore();
            if (res == 2) {  // 0x475b10 -> 0xab: the invalid text (km / miles), Continue -> 0xa5 (0x476850)
                editorSetState(kTooLong);
                openOk(options_.miles ? "TT_PAGE_EDITOR_RACE_IS_INVALID_IN_MILES" : "TT_PAGE_EDITOR_RACE_IS_INVALID_IN_KMS", [this] { editorSetState(kMap); });
            }
            return;
        }
        case kDeleted:  // 0x4767f0: the action key -> the delete list again while races are left, else the top menu
            if (k != MenuKey::Accept || !career_) return;
            if (fuel::editor::freeSlots(career_->state().editedRaces) != 10) editorRun(true);
            else editorEnter();
            return;
        case kValid:  // 0x4768e0
            if (k == MenuKey::Back) return editorSetState(kMap);
            if (k == MenuKey::Accept && career_) {
                // 0x4664d0: new-race defaults, races created + 1 (game +0x3438), SaveGameData; 0xac waits for the storage.
                if (fuel::editor::newRaceDefaults(ed_.race)) {
                    editorStore();
                    career_->state().racesCreated() += 1;
                    for (int t : career_->checkTrophies()) std::printf("career: achievement %d unlocked\n", t);  // 0x56c340 [port: at once]
                    saveCareer();
                }
                editorSetState(kSaving);
                // 0xac: the storage is idle at once in the port -> the page-5 action item again and 0x475ce0.
                editorTop();
                return;
            }
            if (cancel) editorRestore(), editorRun(false);
            return;
        case kInvalidKm:
        case kInvalidMi:  // 0x476860
            if (k == MenuKey::Back) return editorSetState(kMap);
            if (cancel) editorRestore(), editorRun(false);
            return;
        default: return;
    }
}

// 0x4769b0's tick part (0x464be0 + the dragged point of 0x4651f0) per frame.
void Menu::editorUpdate(float dt) {
    if (ed_.state != kMap) return;
    HudMemory& m = hudMem_;
    if (m.pp.empty()) m.pp = hud::loadPp(dataDir_ + "/CommonComponent/Param/pp001.tsc");
    const float pp47 = m.pp.size() > 47 && m.pp[47] != 0 ? m.pp[47] : 0.7f, pp48 = m.pp.size() > 48 && m.pp[48] != 0 ? m.pp[48] : 1.0f;
    auto held = [&](MenuKey k) { return float((map_.held >> (int(k) - int(MenuKey::MapUp))) & 1u); };
    const float sx = held(MenuKey::MapRight) - held(MenuKey::MapLeft), sz = held(MenuKey::MapUp) - held(MenuKey::MapDown);
    const int zoom = int(held(MenuKey::MapZoomIn)) - int(held(MenuKey::MapZoomOut));
    if (ed_.map.update(dt, sx, sz, zoom, pp47, pp48) == 4) editorStore();
}

std::vector<std::array<float, 2>> Menu::editorRoadSpots(int n, float spacing) {
    std::vector<std::array<float, 2>> out;
    const fuel::EditorWorld w = editorWorld();
    if (!w.graph || !career_) return out;
    fuel::EditedRace scratch;
    fuel::editor::Session probe;
    probe.open(&scratch, 0, 0, w);
    probe.zoom = probe.zoomTarget = 2000.0f;
    float ox = ed_.map.cx, oz = ed_.map.cz;
    struct Cand { float d2, x, z; };
    std::vector<Cand> cands;
    for (size_t i = 0; i < w.graph->size(); ++i) {
        const fuel::RoadGraph::Rec& R = w.graph->rec(i);
        if (R.n < 3 || R.type == 14 || (R.flags & fuel::RoadGraph::kExcluded)) continue;
        const auto p = w.graph->point(uint32_t(i), R.n / 2);
        cands.push_back({(p[0] - ox) * (p[0] - ox) + (p[1] - oz) * (p[1] - oz), p[0], p[1]});
    }
    std::sort(cands.begin(), cands.end(), [](const Cand& a, const Cand& b) { return a.d2 < b.d2; });
    for (const Cand& c : cands) {
        if (int(out.size()) >= n) break;
        bool far = true;
        for (const auto& q : out) far = far && std::hypot(q[0] - c.x, q[1] - c.z) >= spacing;
        if (!far) continue;
        probe.cx = c.x, probe.cz = c.z;
        if (probe.cursorPlaceable()) out.push_back({c.x, c.z});
    }
    return out;
}

std::vector<std::string> Menu::editorListRows() {
    std::vector<std::string> out;
    if (!career_) return out;
    const auto& b = career_->state().editedRaces;
    const int used = 10 - fuel::editor::freeSlots(b);
    const int first = ed_.row >= ed_.perPage ? ed_.perPage : 0;
    for (int i = 0; i < ed_.perPage && first + i < used; ++i) out.push_back(fmtText(textOf(182), fuel::editor::usedSlot(b, first + i) + 1));
    return out;
}

// 0x4772c0: "n/32" (TT 1305) and the estimated length: km (TT 1303, alert colour above 150) or miles (TT 1318, value
// trunc(km x 0.6213712), alert above 93.20568).
std::pair<std::string, std::string> Menu::editorInfos() {
    if (!career_ || ed_.slot < 0) return {};
    const fuel::EditedRace r = fuel::editor::slot(career_->state().editedRaces, ed_.slot);
    const int km = fuel::editor::lengthKm(fuel::editor::routeLength(r, ed_.map.world));
    const std::string pts = fmtText(textOf(1305), r.count, 32);
    if (!options_.miles) return {pts, fmtText(textOf(1303), km)};
    return {pts, fmtText(textOf(1318), int(float(km) * 0.6213712f))};
}

// 0x475940 [O asm]: d = the cursor cannot take a point (0x464a50); no point yet -> NO_CHECKPOINT_FROM_CREATEMODE / _EDITMODE;
// a point picked -> DROP_POINT; 32 points -> MAX_CHECKPOINT; else MIDDLE_HELP; "2" variants when d.
std::string Menu::editorHelpKey() const {
    if (!career_ || ed_.slot < 0) return "";
    const int n = career_->state().editedRaces[size_t(ed_.slot) * fuel::EditedRace::kBytes + 0x10];
    const bool d = !ed_.map.cursorPlaceable();
    if (n == 0) return ed_.action == 0 ? (d ? "TT_EDITOR_NO_CHECKPOINT_FROM_CREATEMODE2" : "TT_EDITOR_NO_CHECKPOINT_FROM_CREATEMODE")
                                       : (d ? "TT_EDITOR_NO_CHECKPOINT_FROM_EDITMODE2" : "TT_EDITOR_NO_CHECKPOINT_FROM_EDITMODE");
    if (ed_.map.picked >= 0) return d ? "TT_EDITOR_DROP_POINT2" : "TT_EDITOR_DROP_POINT";
    if (n == 32) return "TT_EDITOR_MAX_CHECKPOINT";
    return d ? "TT_EDITOR_MIDDLE_HELP2" : "TT_EDITOR_MIDDLE_HELP";
}

// 0x477a80 / 0x475940: the chrome keys of the state (a popup keeps the keys of the state it opened over).
void Menu::editorChrome(Chrome& c) const {
    const int st = ed_.base;
    static const char* const kTitles[4] = {"TT_TOP_TITLE_EDITOR_CREATE_RACE", "TT_TOP_TITLE_EDITOR_EDIT_RACE", "TT_TOP_TITLE_EDITOR_DELETE_RACE",
                                           "TT_TOP_TITLE_EDITOR_TRY_RACE"};
    if (st == kMap) {
        c.hasDyn = true, c.dyn = fmtText(textOf(182), ed_.slot + 1);
        c.buttons = editorHelpKey(), c.help = "TT_BH_EDITOR_MODE_EDITION";
        return;
    }
    c.help = "TT_BH_EDITOR_ALL_MODES_EXCEPT_EDITION";
    if (st >= kListCreate && st <= kListTry) c.title = kTitles[st - kListCreate], c.buttons = "TT_MENU_GARAGE";
    else c.title = "TT_TOP_TITLE_TOP_EDITOR", c.buttons = "TT_TOP_EDITOR";
}

void Menu::drawEditorPopup(UiRenderer& r, const std::string& key) {
    Page* help = scripts_.page("PopUpHelp");
    if (!help) return;
    for (Item& it : help->items) {  // 0x46b2e0: the text list boxes and the button line select the key
        if (it.kind == ItemKind::ListBoxTT)
            for (size_t i = 0; i < it.entries.size(); ++i)
                if (it.entries[i].key == key) it.entryIndex = int(i);
        if (it.id == "ITEM_ID_FOND_2" || it.id == "ITEM_ID_POPUP_TEXTS_2") it.visible = false;
        if (it.id == "ITEM_ID_BUTTONS") it.visible = true;
    }
    drawPage(r, *help);
}

// 0x476fe0: the EditorHUD map (vt+0x14 = 0x467090), Map_Cache, EditorMapInfos.
void Menu::drawEditorMap(UiRenderer& r) {
    Page* cache = scripts_.page("Map_Cache");
    if (!cache) return;
    for (Item& it : cache->items)
        if (it.id.rfind("ITEM_ID_MAP_ELEMENT_SIZE", 0) == 0) it.visible = false;
    if (const Item* clip = cache->find("ITEM_ID_CLIPPING_RECT")) {
        float x = 0, y = 0;  // PK19: the anchored rect (itemRect), as the Map tab
        itemRect(*clip, x, y);
        const float w = clip->w, h = clip->h;
        const fuel::editor::Session& s = ed_.map;
        const hud::MapView v = hud::mapView(x, y, w, h, s.zoom, s.cx, s.cz, s.hx, s.hz);
        r.setClip(x, y, w, h);
        drawMapBase(r, x, y, w, h, v, s.cx, s.cz, s.zoom);  // PK19: 0x4a85e0 background + 0x4ae930 terrain picture (centre +0x1b38)
        drawMapGround(r, v, s.cx, s.cz, s.zoom);
        // 0x4673b0: the route, point i to i + 1, DEFAULT_MAT_LINE, width +0x1a28 x 0.2, colour (0.5, 0, 0, 1) (MapHUD's
        // 2x colours: red); 0x467590: a +0x1a28 square on every point in (0.5, 0.5, 0.5) (white).
        constexpr float kSize14 = 14;
        const fuel::EditedRace& R = ed_.race;
        const BitmapRef line = assets_->image("DEFAULT_MAT_LINE");
        std::vector<UiVertex> tris, marks;
        for (size_t i = 0; i + 1 < R.count; ++i) {
            const auto A = v.toScreen(R.pts[i].x, R.pts[i].z), B = v.toScreen(R.pts[i + 1].x, R.pts[i + 1].z);
            float dx = B[0] - A[0], dy = B[1] - A[1];
            const float l = std::hypot(dx, dy);
            if (l <= 0) continue;
            dx *= 0.5f * kSize14 * 0.2f / l, dy *= 0.5f * kSize14 * 0.2f / l;
            const Rgba c{1, 0, 0, 1};
            const UiVertex p0{A[0] - dy, A[1] + dx, 0, 0, c}, p1{A[0] + dy, A[1] - dx, 0, 1, c}, p2{B[0] - dy, B[1] + dx, 1, 0, c}, p3{B[0] + dy, B[1] - dx, 1, 1, c};
            for (const UiVertex& q : {p0, p1, p2, p1, p3, p2}) tris.push_back(q);
        }
        r.triangles(line ? &line : nullptr, tris);
        for (size_t i = 0; i < R.count; ++i) {
            const auto P = v.toScreen(R.pts[i].x, R.pts[i].z);
            const float hs = kSize14 * 0.5f;
            const Rgba c{1, 1, 1, 1};
            const UiVertex a{P[0] - hs, P[1] - hs, 0, 0, c}, b{P[0] + hs, P[1] - hs, 1, 0, c}, d{P[0] - hs, P[1] + hs, 0, 1, c}, e{P[0] + hs, P[1] + hs, 1, 1, c};
            for (const UiVertex& q : {a, b, d, b, e, d}) marks.push_back(q);
        }
        r.triangles(nullptr, marks);
        // PK19 0x4aca30 (from 0x467090 after the route): MAP_TARGET_ICON at the cursor, size +0x1a34 (52), white, alpha +0x1af4
        // (1.0: EditorHUD never runs the minimap's fade 0x49f6d0).
        if (BitmapRef t = assets_->image("MAP_TARGET_ICON")) {
            const auto P = v.toScreen(s.cx, s.cz);
            r.sprite(t, P[0], P[1], 52, 52, 0.5f, 0.5f, 0, Rgba{1, 1, 1, 1});
        }
        if (map_.havePlayer)  // 0x4ab8d0
            if (BitmapRef pl = assets_->image("MAP_PLAYER_CAR_ICON")) {
                const auto P = v.toScreen(map_.px, map_.pz);
                const auto d = v.dirToScreen(map_.phz, -map_.phx);
                r.sprite(pl, P[0], P[1], 30, 30, 0.5f, 0.5f, std::atan2(d[0], -d[1]) * 57.29578f, Rgba{1, 1, 1, 1});
            }
        r.clearClip();
    }
    drawPage(r, *cache);  // 0x476fe0: Map_Cache over the map
    if (Page* p = scripts_.page("EditorMapInfos")) {
        const auto [pts, len] = editorInfos();
        float km = 0;
        if (career_ && ed_.slot >= 0) km = float(fuel::editor::lengthKm(fuel::editor::routeLength(fuel::editor::slot(career_->state().editedRaces, ed_.slot), ed_.map.world)));
        const bool alert = options_.miles ? km * 0.6213712f > 93.20568f : km > 150;
        if (Item* it = p->find("ITEM_ID_NB_RACE_POINTS_USED")) it->hasDynText = true, it->dynText = pts;
        for (const char* id : {"ITEM_ID_ESTIMATED_RACE_LENGTH_IN_KMS", "ITEM_ID_ESTIMATED_RACE_LENGTH_IN_MILES"})
            if (Item* it = p->find(id)) {
                const bool mine = (std::strstr(id, "MILES") != nullptr) == options_.miles;
                it->visible = mine;
                it->hasDynText = true, it->dynText = len;
                // +0x98 = +0x88 (alert, the second text colour) or +0xfc (the third)
                static Rgba normal{-1, 0, 0, 0}, warn{};
                if (normal.r < 0) normal = it->tFocus, warn = it->tEnabled;
                it->tNormal = it->tEnabled = it->tFocus = alert ? warn : normal;
            }
        drawPage(r, *p);
    }
}

// 0x476b30.
void Menu::drawEditor(UiRenderer& r) {
    auto page = [&](const char* name) {
        if (Page* p = scripts_.page(name)) drawPage(r, *p);
    };
    switch (ed_.state) {
        case kTop: page("Editor"); break;
        case kListCreate:
        case kListEdit:
        case kListDelete:
        case kListTry:
            if (Page* p = scripts_.page("EditorRacesList")) {
                // 0x477480: row k of the page: RECT / BARRE_START / BARRE_END / BUTTON_NUMBER ("%02d" TT 1312 of the
                // row + 1) / RACE_NAME ("EDITED RACE %02d" TT 182 of the slot + 1) shown when a used slot is there, HALO on
                // the focused row; arrows: left on the second page, right when more races are there. 0x4771b0: subtitle.
                const auto& b = career_ ? career_->state().editedRaces : std::vector<uint8_t>();
                const int used = career_ ? 10 - fuel::editor::freeSlots(b) : 0;
                const int per = ed_.perPage, first = ed_.row >= per ? per : 0;
                for (int k = 1; k <= per; ++k) {
                    const int idx = first + k - 1;
                    const bool shown = idx < used;
                    const std::string n = std::to_string(k);
                    for (const char* id : {"ITEM_ID_RECT", "ITEM_ID_BARRE_START", "ITEM_ID_BARRE_END", "ITEM_ID_BUTTON_NUMBER", "ITEM_ID_RACE_NAME"})
                        if (Item* it = p->find(id + n)) it->visible = shown;
                    if (Item* it = p->find("ITEM_ID_HALO" + n)) it->visible = shown && k == ed_.row % per + 1;
                    if (Item* it = p->find("ITEM_ID_BUTTON_NUMBER" + n)) it->hasDynText = true, it->dynText = fmtText(textOf(1312), idx + 1), it->focus = k == ed_.row % per + 1;
                    if (Item* it = p->find("ITEM_ID_RACE_NAME" + n)) it->hasDynText = true, it->dynText = fmtText(textOf(182), fuel::editor::usedSlot(b, idx) + 1);
                }
                if (Item* it = p->find("ITEM_ID_FLECHE_GAUCHE")) it->visible = ed_.row >= per;
                if (Item* it = p->find("ITEM_ID_FLECHE_DROITE")) it->visible = ed_.row < per && per < used;
                static const char* const kSub[4] = {"TT_CREATE_RACE_SUBTITLE", "TT_EDIT_RACE_SUBTITLE", "TT_DELETE_RACE_SUBTITLE", "TT_TRY_RACE_SUBTITLE"};
                if (Item* it = p->find("ITEM_ID_RACES_LIST_SUBTITLES"); it && ed_.action >= 0 && ed_.action < 4)
                    for (size_t i = 0; i < it->entries.size(); ++i)
                        if (it->entries[i].key == kSub[ed_.action]) it->entryIndex = int(i);
                drawPage(r, *p);
            }
            break;
        case kDeleteYesNo: break;  // titles and the popup (confirm_)
        case kDeleted:
            if (Page* p = scripts_.page("PopUpRaceDeleted")) {
                if (Item* it = p->find("ITEM_ID_LABEL_RACE_HAS_BEEN_DELETED")) it->hasDynText = true, it->dynText = fmtText(textOf(161), ed_.slot + 1);
                drawPage(r, *p);
            }
            break;
        case kMap:
        case kTooLong: drawEditorMap(r); break;
        case kValid:
        case kSaving: drawEditorMap(r), drawEditorPopup(r, "TT_PAGE_EDITOR_RACE_IS_VALID"); break;
        case kInvalidKm: drawEditorMap(r), drawEditorPopup(r, "TT_PAGE_EDITOR_RACE_IS_INVALID_IN_KMS"); break;
        case kInvalidMi: drawEditorMap(r), drawEditorPopup(r, "TT_PAGE_EDITOR_RACE_IS_INVALID_IN_MILES"); break;
        default: break;
    }
}

// --menu-shot EDITORMAP[:<points>] (0xa5), EDITORVALID (0xa8), EDITORINVALID (0xa9 / 0xaa: one point), EDITORTOOLONG (0xab),
// EDITORLIST:<1 edit | 2 delete | 3 try> (six sample races: two pages), EDITORDELETE (0xa6), EDITORDELETED (0xa7). The sample
// races are built with the editor's own keys at road spots near the current hub's camp; nothing is saved.
bool Menu::editorShot(const std::string& n) {
    goTab(TabEditor);
    auto sample = [&](int points) {
        ed_.action = 0;
        editorOpenMap();
        if (ed_.slot < 0) return false;
        for (const auto& p : editorRoadSpots(points, 450.0f)) {
            ed_.map.cx = p[0], ed_.map.cz = p[1];
            if (ed_.map.world.ground) ed_.map.cy = ed_.map.world.ground(p[0], p[1]);
            editorKey(MenuKey::Accept);
        }
        if (points > 0 && !ed_.race.count) return false;
        return true;
    };
    auto keep = [&]() {  // 0x4664d0 without the save
        fuel::editor::newRaceDefaults(ed_.race);
        editorStore();
    };
    int pts = 4;
    if (n.rfind("EDITORMAP", 0) == 0) {
        if (n.size() > 10) pts = std::atoi(n.c_str() + 10);
        return sample(pts);
    }
    if (n == "EDITORVALID") return sample(4) && (editorValidate(), true);
    if (n == "EDITORINVALID") return sample(1) && (editorValidate(), true);
    if (n == "EDITORTOOLONG") {
        if (!sample(3)) return false;
        editorSetState(kTooLong);
        openOk(options_.miles ? "TT_PAGE_EDITOR_RACE_IS_INVALID_IN_MILES" : "TT_PAGE_EDITOR_RACE_IS_INVALID_IN_KMS", [this] { editorSetState(kMap); });
        return true;
    }
    const bool list = n.rfind("EDITORLIST", 0) == 0, del = n == "EDITORDELETE" || n == "EDITORDELETED";
    if (!list && !del) return false;
    if (!sample(2)) return false;
    keep();
    const fuel::EditedRace first = ed_.race;
    for (int i = 1; i < (list ? 6 : 2); ++i) {
        const int s = fuel::editor::firstFreeSlot(career_->state().editedRaces);
        if (s >= 0) fuel::editor::setSlot(career_->state().editedRaces, s, first);
    }
    editorEnter();
    ed_.action = list ? std::clamp(n.size() > 11 ? std::atoi(n.c_str() + 11) : 1, 1, 3) : 2;
    editorRun(true);
    if (list && ed_.row == 0 && n.find(":P2") != std::string::npos) editorListKey(MenuKey::Right);
    if (del) {
        editorKey(MenuKey::Accept);  // 0xa6
        if (n == "EDITORDELETED") {
            confirm_ = {};
            career_->state().editedRaces[size_t(ed_.slot) * fuel::EditedRace::kBytes + 0x10] = 0;  // 0x475b50 without the save
            editorSetState(kDeleted);
        }
    }
    return true;
}

}  // namespace fuel::ui
