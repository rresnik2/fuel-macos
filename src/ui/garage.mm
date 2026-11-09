// P66: the garage's livery, paintwork and driver pages (FUEL's garage page, state word +0x16c), decoded from
// FUEL.exe; see ~/re-work/fuel/notes/p66-garage-profile.md. Tags: [O] observed, [I] inferred, [port] port choice.
//
// States and pages (titles 0x47c1e0, keys 0x478bb0, draws 0x479d50 / 0x473360):
//   1     Garage left menu            0x94  livery carousel (build 0x47aa40, keys 0x4796e0, draw 0x47a570)
//   0x95  paintwork carousel (0x47af10 / 0x479a60 / 0x47a6a0)
//   0x3b  Driver left menu (0x471650): HEAD / CHEST / LEGS -> 0x3c / 0x3d / 0x3e (0x470c50 ..), SKIN -> 0x3f (0x470e00)
//   0x3c  DriverHead menu (0x471780): TYPE -> 0x9b (0x470e90, build 0x4740f0), LIVERY -> 0x9c (0x470ed0, 0x474320);
//         chest 0x3d -> 0x9d (0x4745c0) / 0x9e (0x474880), legs 0x3e -> 0x9f (0x474c20) / 0xa0 (0x474e40)
//   0x3f  skin carousel (0x4750e0 / 0x471bd0)
// Carousel entries are 0x24 bytes (0x475440): +4 picture crc, +8..+0x14 colour, +0x18 value, +0x1c owned, +0x20 text.
// The type / livery / skin carousels lay out on ITEM_ID_GEAR_CARRE_SELECTION (spacing 0.65 .. 0.85 by carousel, 0x473c70 ..
// 0x473e50); the vehicle's on ITEM_ID_BIDON_BASE and ITEM_ID_LIVERIES_REF_ITEM (spacing 1.3, the livery 0.87 wide).
// Accept stores the value (vehicle record +0xc / +0xd, game +0x33d0.. through 0x4715c0, the pilot pair list
// 0x4721f0) and sets the page's save flag +0x80; the port writes career.sav at once. Back keeps the old values.
// FUEL previews the choice on the 3-D garage vehicle and rider while browsing: PK20 (garage3DUpdate below, the 3-D
// objects in viewer/garage_preview.mm); the choice also goes to the next session (game_menu.mm passes it on).
#include "ui/menu.h"

#include "core/rider.hpp"

#include <cstdlib>
#include <cstring>
#include "ui/ui_renderer.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <utility>

namespace fs = std::filesystem;

namespace fuel::ui {
namespace {

// FUEL's 2-D colours are half range (AddVehicleColor 0x56acb0 stores v / 510, the neutral entry colour is 0.5:
// DAT_00a064a8..b4 = 0.5, 0.5, 0.5, 1) [O]; the port's images take 1 for the neutral colour.
Rgba half(float r, float g, float b) { return Rgba{std::min(1.0f, 2 * r), std::min(1.0f, 2 * g), std::min(1.0f, 2 * b), 1}; }
const Rgba kNeutral = half(0.5f, 0.5f, 0.5f);

const char* const kPart[3] = {"HEAD", "CHEST", "LEGS"};

}  // namespace

const VehicleInfo* Menu::vehicleInfo(const std::string& label) const {
    for (const VehicleInfo& v : vehicles_)
        if (v.name == label) return &v;
    return nullptr;
}

void Menu::garageLoadData() {
    if (garageDataLoaded_) return;
    garageDataLoaded_ = true;
    try {
        fuel::DpcFile pilot((fs::path(dataDir_) / "CommonComponent/DATAS/PILOT.DPC").string());
        riderLiveries_ = fuel::riderLiveryNames(pilot);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "menu: pilot liveries: %s\n", e.what());
    }
}

// 0x4780c0 (state + titles 0x47c1e0) and the page builds.
void Menu::garageEnter(int state) {
    garageLoadData();
    garage_.state = state;
    garage_.entries.clear();
    garage_.focus = 0;
    // 0x47c1e0: the title per state [O].
    switch (state) {
        case 0x94: setTitle("TT_TOP_TITLE_GARAGE_LIVERY"); break;
        case 0x95: setTitle("TT_TOP_TITLE_GARAGE_COLOR"); break;
        case 0x3b: setTitle("TT_TOP_TITLE_DRIVER"); break;
        case 0x3c: case 0x9b: case 0x9c: setTitle("TT_TOP_TITLE_DRIVER_HEAD"); break;
        case 0x3d: case 0x9d: case 0x9e: setTitle("TT_TOP_TITLE_DRIVER_CHEST"); break;
        case 0x3e: case 0x9f: case 0xa0: setTitle("TT_TOP_TITLE_DRIVER_LEGS"); break;
        case 0x3f: setTitle("TT_TOP_TITLE_DRIVER_SKIN"); break;
        case kOutfitState: setTitle("TT_TOP_TITLE_DRIVER"); break;  // PHYSIQUE
        default: break;
    }
    if (state == 0x3b) garageAddOutfitItem();  // PHYSIQUE
    static const std::pair<int, const char*> kLeft[] = {{0x3b, "Driver"}, {0x3c, "DriverHead"}, {0x3d, "DriverChest"}, {0x3e, "DriverLegs"}};
    for (const auto& [st, page] : kLeft)
        if (st == state) {
            screen_ = Screen::Tab, tab_ = TabGarage, garage_.page = page, focus_ = 0;
            return;
        }
    if (state == 1) {
        garage_.page.clear();
        goTab(TabGarage);
        return;
    }
    screen_ = Screen::GarageSelect;
    if (!career_) return;
    const VehicleInfo* vi = vehicleInfo(vehicle_);
    const std::string mesh = "MESH_ID_" + vehicle_;
    auto& E = garage_.entries;
    int current = -1;
    if (state == 0x94 || state == 0x95) {
        const int liv = career_->vehicleLivery(vehicle_), col = career_->vehicleColor(vehicle_);
        Rgba paint = kNeutral;
        if (vi && !vi->colors.empty()) {
            const auto& c = vi->colors[size_t(col) % vi->colors.size()];
            paint = half(c[0] / 510.0f, c[1] / 510.0f, c[2] / 510.0f);
        }
        // BIDON_SHADE 0.5 grey, BIDON_LIGHT 0.5 grey (alpha 0.4, unused: the carousel draws with the menu's alpha) [O].
        auto bidon = [&](GarageEntry& e, const Rgba& base, int livery) {
            e.layers.push_back({"BIDON_BLANC", base, 0});
            e.layers.push_back({mesh + "_LIV" + std::to_string(livery + 1), kNeutral, 1});
            e.layers.push_back({"BIDON_SHADE", kNeutral, 0});
            e.layers.push_back({"BIDON_LIGHT", kNeutral, 0});
        };
        if (state == 0x94) {
            // 0x47aa40: three entries per owned livery (vehicleLiveryChoices), coloured with the vehicle's paint (+0xd).
            for (int k : career_->vehicleLiveryChoices(vehicle_)) {
                GarageEntry e;
                e.value = k;
                bidon(e, paint, k);
                if (k == liv) current = int(E.size());
                E.push_back(e);
            }
        } else if (vi) {
            // 0x47af10: every palette colour (count +0x104), each over the current livery (+0xc, < 0 -> 0).
            for (size_t c = 0; c < vi->colors.size(); ++c) {
                GarageEntry e;
                e.value = int(c);
                bidon(e, half(vi->colors[c][0] / 510.0f, vi->colors[c][1] / 510.0f, vi->colors[c][2] / 510.0f), std::max(0, liv));
                if (int(c) == col) current = int(E.size());
                E.push_back(e);
            }
        }
    } else if (state == kOutfitState) {
        // PHYSIQUE (port deviation): the outfits, pictured with FUEL's chest pictures (bare torso, sleeveless jersey, jersey).
        const fuel::RiderOutfit cur{career_->riderOutfit(), career_->riderHelmet()};
        static const struct { int outfit; bool helmet; const char* pic; } kOutfits[] = {
            {1, true, "CHEST4"}, {1, false, "CHEST4"}, {2, false, "CHEST3"}, {2, true, "CHEST3"},
            {3, false, "CHEST1"}, {3, true, "CHEST1"}, {0, true, "CHEST2"}};
        const bool custom = fuel::CustomRider::shared() != nullptr;
        for (const auto& k : kOutfits) {
            if (k.outfit > 0 && !custom) continue;
            GarageEntry e;
            e.layers.push_back({k.pic, kNeutral, -1});
            e.value = k.outfit * 2 + (k.helmet ? 1 : 0);
            e.label = fuel::riderOutfitName({k.outfit, k.helmet});
            if (k.outfit == cur.outfit && (k.outfit == 0 || k.helmet == cur.helmet)) current = int(E.size());
            E.push_back(e);
        }
    } else if (state == 0x3f) {
        // 0x4750e0: skins k = 1 .. count("LIVERY_01_BODY<k>") (0x5dc850 "BODY", 1), picture FLAMME_COULEUR_GEAR in three
        // fixed tones (0x3e9c9c9d ..), text k + 0xee8, all owned; current = game +0x33ec [O].
        static const float kTone[3][3] = {{0.30588f, 0.22147f, 0.17255f}, {0.34510f, 0.25686f, 0.20206f}, {0.11176f, 0.06275f, 0.04314f}};
        const int n = fuel::riderLiveryCount(riderLiveries_, "BODY", true);
        const PilotLook look = career_->pilotLook();
        for (int k = 1; k <= n; ++k) {
            GarageEntry e;
            const float* t = kTone[std::min(k - 1, 2)];
            e.layers.push_back({"FLAMME_COULEUR_GEAR", half(t[0], t[1], t[2]), -1});
            e.value = k, e.text = uint32_t(k + 0xee8);
            if (k == look.skin) current = int(E.size());
            E.push_back(e);
        }
    } else {
        const bool types = state == 0x9b || state == 0x9d || state == 0x9f;
        const int part = state <= 0x9c ? 0 : state <= 0x9e ? 1 : 2;
        const PilotLook look = career_->pilotLook();
        const int type = look.at(part), curLivery = look.at(3 + part);
        if (types) {
            // 0x4740f0 / 0x4745c0 / 0x474c20: types n = 1 .. the pilot's mesh count for the part, offered when an owned
            // pilot livery has that type (chest 1 = the bare torso: gated by "BODY1", picture CHEST1); text n + 0xed7
            // (head), n + 0xee0 (chest), n + 0xee5 (legs) [O]. Mesh count: the types with liveries in PILOT.DPC [port].
            static const uint32_t kText[3] = {0xed7, 0xee0, 0xee5};
            for (int n = 1; n <= 16; ++n) {
                const std::string name = kPart[part] + std::to_string(n);
                if (fuel::riderLiveryCount(riderLiveries_, name) == 0) continue;
                const bool owned = part == 1 && n == 1 ? career_->pilotTypeOwned("BODY1") : career_->pilotTypeOwned(name);
                if (!owned) continue;
                GarageEntry e;
                e.layers.push_back({name, kNeutral, -1});
                e.value = n, e.text = kText[part] + uint32_t(n);
                if (n == type) current = int(E.size());
                E.push_back(e);
            }
        } else if (part == 1 && type == 1) {
            // 0x474880, chest 1: liveries k = 1 .. count(LIVERY_%02d_BODY<skin>) (0x5dc710), entry (BODY<k>, index k)
            // owned, picture BODY<k>, text = the entry's (+0x18) [O].
            const int n = fuel::riderLiveryCount(riderLiveries_, "BODY" + std::to_string(look.skin));
            for (int k = 1; k <= n; ++k) {
                bool owned = false;
                const auto* pl = career_->pilotLivery("BODY" + std::to_string(k), k, &owned);
                if (!pl || !owned) continue;
                GarageEntry e;
                e.layers.push_back({"BODY" + std::to_string(k), kNeutral, -1});
                e.value = k, e.text = pl->text;
                if (k == curLivery) current = int(E.size());
                E.push_back(e);
            }
        } else {
            // 0x474320 / 0x474880 / 0x474e40: texture liveries k = 1 .. count(LIVERY_%02d_<PART><type>) (0x5dc530 ..),
            // entry n = (k - 1) / 3 + 1 of that type owned, picture "<PART><type>_<n>_<(k - 1) % 3 + 1>", text = the
            // entry's +0x18 [O]. (FUEL reads a missing entry through a null pointer; the port skips it.)
            const std::string name = kPart[part] + std::to_string(type);
            const int n = fuel::riderLiveryCount(riderLiveries_, name);
            for (int k = 1; k <= n; ++k) {
                bool owned = false;
                const auto* pl = career_->pilotLivery(name, (k - 1) / 3 + 1, &owned);
                if (!pl || !owned) continue;
                char pic[48];
                std::snprintf(pic, sizeof pic, "%s%d_%d_%d", kPart[part], type, (k - 1) / 3 + 1, (k - 1) % 3 + 1);
                GarageEntry e;
                e.layers.push_back({pic, kNeutral, -1});
                e.value = k, e.text = pl->text;
                if (k == curLivery) current = int(E.size());
                E.push_back(e);
            }
        }
    }
    // The entry whose value is the current one (+0xcc / +0x1dc ..); FUEL keeps -1 when none [port: the first].
    garage_.focus = current >= 0 ? current : 0;
}

// Left-menu items of the garage pages (Garage 0x478f00, Driver 0x471650 -> 0x470ae0, DriverHead 0x471780 -> 0x471470).
bool Menu::garageAccept(const std::string& id) {
    if (id == "ITEM_ID_TOPMENU_GARAGE_LIVERY") return garageEnter(0x94), true;   // 0x478640
    if (id == "ITEM_ID_TOPMENU_GARAGE_COLOR") return garageEnter(0x95), true;    // 0x478690
    if (id == "ITEM_ID_TOPMENU_GARAGE_DRIVER") return garageEnter(0x3b), true;   // 0x470bc0
    if (id == "ITEM_ID_TOPMENU_GARAGE_UPGRADES") return upgradesOpen(), true;    // [upgrades U3] port state 0x1a0 (garage_upgrades.mm)
    static const std::pair<const char*, int> kItems[] = {
        {"ITEM_ID_SUBMENU_DRIVER_HEAD", 0x3c},        {"ITEM_ID_SUBMENU_DRIVER_CHEST", 0x3d},       {"ITEM_ID_SUBMENU_DRIVER_LEGS", 0x3e},
        {"ITEM_ID_SUBMENU_DRIVER_SKIN", 0x3f},        {"ITEM_ID_SUBMENU_DRIVER_HEAD_TYPE", 0x9b},   {"ITEM_ID_SUBMENU_DRIVER_HEAD_LIVERY", 0x9c},
        {"ITEM_ID_SUBMENU_DRIVER_CHEST_TYPE", 0x9d},  {"ITEM_ID_SUBMENU_DRIVER_CHEST_LIVERY", 0x9e}, {"ITEM_ID_SUBMENU_DRIVER_LEGS_TYPE", 0x9f},
        {"ITEM_ID_SUBMENU_DRIVER_LEGS_LIVERY", 0xa0}, {"ITEM_ID_SUBMENU_DRIVER_OUTFIT", kOutfitState}};
    for (const auto& [item, st] : kItems)
        if (id == item) return garageEnter(st), true;
    return false;
}

// Back (the key handlers' first branch): carousels -> their menu, menus -> the one above [O].
bool Menu::garageBack() {
    switch (garage_.state) {
        // PK20: 0x4796e0 / 0x479a60 / 0x471650 Back: 0x4780c0(1) leaves the garage list's focus on the item (was focus 0)
        case 0x94: garageReturn("ITEM_ID_TOPMENU_GARAGE_LIVERY", false); return true;
        case 0x95: garageReturn("ITEM_ID_TOPMENU_GARAGE_COLOR", false); return true;
        case 0x3b: garageReturn("ITEM_ID_TOPMENU_GARAGE_DRIVER", false); return true;
        case kUpgradesState: garageReturn("ITEM_ID_TOPMENU_GARAGE_UPGRADES", false); return true;  // [upgrades U3]
        case 0x3c: case 0x3d: case 0x3e: case 0x3f: case kOutfitState: garageEnter(0x3b); return true;
        case 0x9b: case 0x9c: garageEnter(0x3c); return true;
        case 0x9d: case 0x9e: garageEnter(0x3d); return true;
        case 0x9f: case 0xa0: garageEnter(0x3e); return true;
        default: return false;
    }
}

void Menu::garageKey(MenuKey k) {
    auto& g = garage_;
    const int n = int(g.entries.size());
    // Left / Right: +-1 within the list, no wrap (0x4796e0: 0x4c9330 / 0x4c93c0) [O].
    if (k == MenuKey::Left || k == MenuKey::Up) g.focus = std::max(0, g.focus - 1);
    if (k == MenuKey::Right || k == MenuKey::Down) g.focus = std::min(std::max(0, n - 1), g.focus + 1);
    if (k == MenuKey::Back) {
        garageBack();
        return;
    }
    if (k != MenuKey::Accept || !career_ || g.focus >= n) return;
    const GarageEntry& e = g.entries[size_t(g.focus)];
    const int st = g.state;
    PilotLook look = career_->pilotLook();
    if (st == 0x94) career_->setVehicleLivery(vehicle_, e.value);       // 0x4796e0: +0xc = entry +0x18
    else if (st == 0x95) career_->setVehicleColor(vehicle_, e.value);   // 0x479a60: +0xd = +0xd0
    else if (st == 0x3f) look.skin = e.value, career_->setPilotLook(look);  // 0x471bd0 -> 0x4715c0
    else if (st == kOutfitState) career_->setRiderOutfit(e.value / 2, (e.value & 1) != 0);  // PHYSIQUE
    else {
        // 0x471f40 / 0x471e80 (types: the livery its pair remembers, default 3 x first owned - 2, written back into the
        // pair) or 0x4721f0 (liveries, also into the type's pair); 0x4715c0 commits the page's record +0x270 [O].
        const int part = st <= 0x9c ? 0 : st <= 0x9e ? 1 : 2;
        look = garageLookWith(look, st, e.value);
        const std::string type = kPart[part] + std::to_string(look.at(part));
        if (st == 0x9b || st == 0x9d || st == 0x9f) {
            const int liv = career_->pilotTypeLivery(type);
            if (liv > 0) career_->setPilotTypeLivery(type, liv);
        } else {
            career_->setPilotTypeLivery(type, e.value);
        }
        career_->setPilotLook(look);
    }
    markDirty();  // PK17: page +0x80 = 1 (saved when the pause menu closes, 0x46e9b0)
    // PK20: 0x4796e0 / 0x479a60 Accept: state 1 with the garage list's focus moved to the next selectable item (0x60fbf0(1)
    // -> 0x60fcf0); the driver pages go back to their menus.
    if (st == 0x94) garageReturn("ITEM_ID_TOPMENU_GARAGE_LIVERY", true);
    else if (st == 0x95) garageReturn("ITEM_ID_TOPMENU_GARAGE_COLOR", true);
    else garageBack();
}

void Menu::drawGarageSelect(UiRenderer& r) {
    if (Page* g = scripts_.page("GearSelecteur")) drawPage(r, *g);  // 0x46b6e0 [I: which page]
    const int st = garage_.state;
    if (st == 0x94 || st == 0x95) {
        // 0x47a570 / 0x47a6a0: the base (paint), the livery, the shade and the light, array by array.
        drawCarousel(r, garage_.entries, garage_.focus, "ITEM_ID_BIDON_BASE", 1.3f, 0);
        drawCarousel(r, garage_.entries, garage_.focus, "ITEM_ID_LIVERIES_REF_ITEM", 1.3f, 1, 0.87f);
        drawCarousel(r, garage_.entries, garage_.focus, "ITEM_ID_BIDON_BASE", 1.3f, 2);
        drawCarousel(r, garage_.entries, garage_.focus, "ITEM_ID_BIDON_BASE", 1.3f, 3);
    } else {
        // PK20: the spacing per carousel (0x473c70 .. 0x473e50 [O]): head types 0.65, head liveries 0.75, chest types 0.65,
        // chest liveries 0.85, legs types 0.75, legs liveries 0.75, skins 0.85 (was 0.65 / 0.85 for all).
        const float spacing = st == 0x9b || st == 0x9d ? 0.65f : st == 0x9e || st == 0x3f || st == kOutfitState ? 0.85f : 0.75f;
        drawCarousel(r, garage_.entries, garage_.focus, "ITEM_ID_GEAR_CARRE_SELECTION", spacing, 0);
        // DriverSlotsAndLiveriesInfos: ITEM_ID_SLOT_OR_LIVERY_NAME = the focused entry's text (0x472290 / 0x46cdb0).
        if (Page* p = scripts_.page("DriverSlotsAndLiveriesInfos")) {
            if (Item* it = p->find("ITEM_ID_SLOT_OR_LIVERY_NAME")) {
                it->hasDynText = true;
                const GarageEntry* fe = garage_.focus < int(garage_.entries.size()) ? &garage_.entries[size_t(garage_.focus)] : nullptr;
                it->dynText = !fe ? "" : !fe->label.empty() ? fe->label : textOf(fe->text);  // PHYSIQUE: port labels
            }
            drawPage(r, *p);
        }
    }
    if (Page* gc = scripts_.page("GearSelecteurCache")) {
        // 0x4752f0: left arrow when the focus > 0, right when it is not the last [O].
        for (Item& it : gc->items) {
            if (it.id == "ITEM_ID_FLECHE_GAUCHE") it.visible = garage_.focus > 0;
            else if (it.id == "ITEM_ID_FLECHE_DROITE") it.visible = garage_.focus < int(garage_.entries.size()) - 1;
            else continue;
            drawItem(r, it, 0, 0);
        }
    }
}

// 0x473ea0: entries around the reference item: spacing = factor x its width, the focused one 1.1 x (centred on the
// reference's middle), the others 0.7 x lifted to 0.35 of the height; shown within 1.5 x (view width - centre);
// colour x (1 - distance / range), black when the entry is not owned; alpha = the menu's [O].
void Menu::drawCarousel(UiRenderer& r, const std::vector<GarageEntry>& entries, int focus, const char* refId, float spacing, int layer,
                        float widthFactor) {
    Page* gc = scripts_.page("GearSelecteurCache");
    const Item* ref = gc ? gc->find(refId) : nullptr;
    if (!ref || entries.empty() || focus < 0) return;
    const float centre = ref->w * 0.5f + ref->x, range = (1280.0f - centre) * 1.5f, step = ref->w * spacing;
    const float w = widthFactor > 0 ? widthFactor * ref->w : ref->w, h = ref->h;
    for (int i = 0; i < int(entries.size()); ++i) {
        const GarageEntry& e = entries[size_t(i)];
        const int li = layer < 0 ? 0 : layer;
        if (li >= int(e.layers.size())) continue;
        const float cx = centre + float(i - focus) * step, d = std::fabs(cx - centre);
        if (d > range) continue;
        const bool on = i == focus;
        const float f = on ? 1.1f : 0.7f, lift = on ? 0.5f : 0.35f, fade = 1.0f - d / range;
        const GarageLayer& L = e.layers[size_t(li)];
        const Rgba c = e.owned ? Rgba{L.colour.r * fade, L.colour.g * fade, L.colour.b * fade, 1} : Rgba{0, 0, 0, 1};
        r.image(assets_->image(L.image), cx - f * w * 0.5f, (h * 0.5f + ref->y) - h * lift * f, f * w, f * h, c);
    }
}

// ---- PK20: the 3-D garage preview, page side (InGamePauseHUD; notes/pk20.md). The 3-D objects are the viewer's
// (viewer/garage_preview.mm); this queues FUEL's calls into them in FUEL's order.

// 0x478070: the focused item of the garage list (the page's list 1, its focus +0x14) among the five ids of 0xa663a8
// (filled by 0x8e4b90): 0 DRIVER, 1 VEHICLE, 2 LIVERY, 3 COLOR, 4 START_RACE, else -1 [O]. The garage list keeps its
// focus while a sub page is up (the driver menus, the carousels and the vehicle list sit on top of it), so those pages
// report the item that opened them [O: 0x478070 always reads list 1; the port's pages replace the list].
// [upgrades U3] 5 = the port's UPGRADES item (and its page): a vehicle item like LIVERY / PAINTWORK.
int Menu::garage3DItem() {
    static const char* const kItems[5] = {"ITEM_ID_TOPMENU_GARAGE_DRIVER", "ITEM_ID_TOPMENU_GARAGE_VEHICLE", "ITEM_ID_TOPMENU_GARAGE_LIVERY",
                                          "ITEM_ID_TOPMENU_GARAGE_COLOR", "ITEM_ID_TOPMENU_GARAGE_START_RACE"};
    if (screen_ == Screen::VehicleList) return 1;  // 0x93 [I: also the race page's garage]
    if (screen_ == Screen::GarageUpgrades) return 5;  // [upgrades U3]
    if (screen_ == Screen::GarageSelect) return garage_.state == 0x94 ? 2 : garage_.state == 0x95 ? 3 : 0;
    if (screen_ != Screen::Tab || tab_ != TabGarage) return -1;
    if (!garage_.page.empty()) return 0;  // Driver, DriverHead, DriverChest, DriverLegs
    const auto f = focusables();
    if (focus_ < 0 || focus_ >= int(f.size())) return -1;
    for (int k = 0; k < 5; ++k)
        if (f[size_t(focus_)]->id == kItems[k]) return k;
    if (f[size_t(focus_)]->id == "ITEM_ID_TOPMENU_GARAGE_UPGRADES") return 5;  // [upgrades U3]
    return -1;
}

// The outfit record +0x270 (= InGame3DPerso +0x5c) with one carousel value applied, as the browse handlers write it:
// types 0x471f40 (+0x270 = n when 1 <= n <= the variant count) then 0x471e80 (the pair's livery, default 3 x first
// owned - 2); liveries 0x4721f0 (+0x2a4 / +0x2ac / +0x2b4); skins 0x471ba0 (+0x2c4) [O].
PilotLook Menu::garageLookWith(PilotLook look, int st, int value) const {
    if (st == 0x3f) {
        look.skin = value;
        return look;
    }
    if (st < 0x9b || st > 0xa0) return look;
    const int part = st <= 0x9c ? 0 : st <= 0x9e ? 1 : 2;
    if (st == 0x9b || st == 0x9d || st == 0x9f) {
        look.at(part) = value;
        const int liv = career_ ? career_->pilotTypeLivery(kPart[part] + std::to_string(value)) : 0;
        if (liv > 0) look.at(3 + part) = liv;
    } else {
        look.at(3 + part) = value;
    }
    return look;
}

// The record the preview rider wears: the player's outfit (0x471570 copies it at the garage enter) with the focused
// entry of a driver carousel (the browse handlers write it as the focus moves; Back restores the saved one, Accept
// commits it through 0x4715c0) [O].
PilotLook Menu::garagePreviewLook() const {
    PilotLook look = career_ ? career_->pilotLook() : PilotLook{};
    const int st = garage_.state;
    if (screen_ == Screen::GarageSelect && (st == 0x3f || (st >= 0x9b && st <= 0xa0)) && garage_.focus >= 0 &&
        garage_.focus < int(garage_.entries.size()))
        look = garageLookWith(look, st, garage_.entries[size_t(garage_.focus)].value);
    if (const char* e = std::getenv("FUEL_SHOT_PARTS"); e && *e)  // captures (--menu-shot): HEAD,CHEST,LEGS[,SKIN] instead of the career's
        std::sscanf(e, "%d,%d,%d,%d", &look.head, &look.chest, &look.legs, &look.skin);
    return look;
}

// ---- PHYSIQUE (port deviation): the outfit the preview rider wears, and the Driver menu's OUTFIT item (a copy of SKIN,
// one row lower; its text is the port's).
fuel::RiderOutfit Menu::garagePreviewOutfit() const {
    fuel::RiderOutfit o{career_ ? career_->riderOutfit() : 1, career_ ? career_->riderHelmet() : true};
    if (screen_ == Screen::GarageSelect && garage_.state == kOutfitState && garage_.focus >= 0 && garage_.focus < int(garage_.entries.size())) {
        const int v = garage_.entries[size_t(garage_.focus)].value;
        o = {v / 2, (v & 1) != 0};
    }
    if (const char* e = std::getenv("FUEL_SHOT_OUTFIT"); e && *e)  // captures (--menu-shot): N[,H] instead of the career's
        o = {std::clamp(std::atoi(e), 0, fuel::kRiderOutfits - 1), std::strchr(e, ',') ? std::atoi(std::strchr(e, ',') + 1) != 0 : o.helmet};
    if (!fuel::CustomRider::shared()) o.outfit = 0;
    return o;
}

void Menu::garageAddOutfitItem() {
    Page* p = scripts_.page("Driver");
    if (!p || p->find("ITEM_ID_SUBMENU_DRIVER_OUTFIT") || !fuel::CustomRider::shared()) return;
    const Item* skin = p->find("ITEM_ID_SUBMENU_DRIVER_SKIN");
    if (!skin) return;
    Item it = *skin;
    it.id = "ITEM_ID_SUBMENU_DRIVER_OUTFIT";
    it.y += 37.0f;  // the rows are 37 apart (Driver.tsc)
    it.a7 += 2;
    it.hasDynText = true, it.dynText = "OUTFIT";
    p->items.push_back(it);
}
// ---- PHYSIQUE end

// 0x44afb0(index, record): palette colour `index` (record +0x8c + 12 i, count +0x104) x 2, else the index as 0xRRGGBB
// / 255 [O]. The palette holds v / 510, so x 2 = v / 255.
std::array<float, 3> Menu::garagePaintRgb(const std::string& vehicle, int index) const {
    const VehicleInfo* vi = vehicleInfo(vehicle);
    if (vi && index >= 0 && index < int(vi->colors.size())) {
        const auto& c = vi->colors[size_t(index)];
        return {c[0] / 510.0f * 2.0f, c[1] / 510.0f * 2.0f, c[2] / 510.0f * 2.0f};
    }
    const uint32_t u = uint32_t(index);
    return {float(u >> 16 & 0xff) * 0.003921569f, float(u >> 8 & 0xff) * 0.003921569f, float(u & 0xff) * 0.003921569f};
}

// 0x478ec0: +0xec = 0.15 (0x9dc0b8), the vehicle scene off (0x436020 on +0xf4, AL 0), +0x2d0 = 0.25 (0x9dbfb0),
// +0x128 = 0, the rider scene off (+0x218) [O].
void Menu::garageRestart() {
    g3d_.vehicleTimer = 0.15f;
    g3d_.riderTimer = 0.25f;
    g3d_.loaded = false;
    g3dCmds_.push_back(Garage3DCmd{Garage3DCmd::Restart});
}

// State 1 again (0x4780c0(1)). FUEL's pages sit on top of the garage list, so Back leaves its focus on the item that
// opened them; Accept on the livery / paint / vehicle pages moves it to the next selectable item (0x60fbf0(1): clamped,
// visible and enabled items only) through 0x60fcf0 [O]. (The port rebuilt the list with the focus on the first item.)
void Menu::garageReturn(const char* itemId, bool next) {
    garageEnter(1);
    const auto f = focusables();
    for (size_t i = 0; i < f.size(); ++i)
        if (f[i]->id == itemId) {
            focus_ = int(next ? std::min(i + 1, f.size() - 1) : i);
            break;
        }
}

void Menu::garage3DUpdate(float dt, bool menuVisible) {
    using C = Garage3DCmd;
    auto& g = g3d_;
    auto push = [&](C::Kind k) { g3dCmds_.push_back(C{k}); };
    const bool shown = menuVisible && mode_ != MenuMode::Title &&
                       (screen_ == Screen::VehicleList || screen_ == Screen::GarageSelect || screen_ == Screen::GarageUpgrades ||
                        (screen_ == Screen::Tab && tab_ == TabGarage));
    if (!shown) {
        if (g.shown) push(C::Release), g = Garage3DPage{};  // 0x4789e0 -> 0x4712f0 / 0x4788d0
        return;
    }
    const int item = garage3DItem();
    std::string veh = vehicle_;  // +0xc8: the current vehicle (0x4781c0), or the browsed one (0x4791b0)
    if (screen_ == Screen::VehicleList) {
        const auto list = vehicleChoices();
        if (focus_ >= 0 && focus_ < int(list.size())) veh = list[size_t(focus_)]->name;
    }
    const PilotLook look = garagePreviewLook();
    if (!g.shown) {
        // 0x478400 / 0x478490 / 0x478520: 0x4781c0 (+0xc8 = the current vehicle, 0x4d1020), 0x471010 (rider set-up, mode
        // 0, outfit 0x471570 = the player's), then the restart's fields inline.
        g = Garage3DPage{};
        g.shown = true, g.item = item, g.vehicle = veh, g.look = look, g.mode = 0;
        g.outfit = garagePreviewOutfit();  // PHYSIQUE
        C s{C::Setup};
        s.look = look;
        s.outfit = g.outfit;
        g3dCmds_.push_back(s);
        garageRestart();
    }
    // The hovered item changed (0x478f00; also the Accept handlers 0x4796e0 / 0x479a60 / 0x4791b0 after their focus
    // move): restart when the new item is DRIVER / START_RACE, or VEHICLE / COLOR coming from anything but LIVERY [O].
    if (item != g.item) {
        if (item == 0 || item == 4 || ((item == 1 || item == 3) && g.item != 2)) garageRestart();
        // [upgrades U3] UPGRADES keeps the vehicle shown by VEHICLE / LIVERY / PAINTWORK (as LIVERY does) and shows it again
        // after the rider (DRIVER) or an empty scene (START THE RACE!, nothing hovered).
        if (item == 5 && (g.item == 0 || g.item == 4 || g.item < 0)) garageRestart();
        g.item = item;
    }
    // 0x4791b0: a new vehicle on the list (Left / Right) or the old one back (Back) restarts the preview [O].
    if (veh != g.vehicle) g.vehicle = veh, g.livery = g.paint = -1, garageRestart();
    // 0x4796e0 / 0x4799c0: a livery browsed (or the committed one back on Back) is requested on the 3-D vehicle;
    // 0x479a60 / 0x479a00: the same for the paint [O]. Accept commits the browsed value (no request).
    if (career_) {
        const bool livPage = screen_ == Screen::GarageSelect && garage_.state == 0x94 && garage_.focus < int(garage_.entries.size());
        const bool paintPage = screen_ == Screen::GarageSelect && garage_.state == 0x95 && garage_.focus < int(garage_.entries.size());
        const int liv = livPage ? garage_.entries[size_t(garage_.focus)].value : career_->vehicleLivery(veh);
        const int pnt = paintPage ? garage_.entries[size_t(garage_.focus)].value : career_->vehicleColor(veh);
        if (g.livery < 0) g.livery = liv;
        if (g.paint < 0) g.paint = pnt;
        if (liv != g.livery) {
            C c{C::Livery};
            c.livery = liv;
            g3dCmds_.push_back(c);
            g.livery = liv;
        }
        if (pnt != g.paint) {
            C c{C::Paint};
            c.rgb = garagePaintRgb(veh, pnt);
            g3dCmds_.push_back(c);
            g.paint = pnt;
        }
    }
    // The outfit record changed (browse / Back / Accept on the driver carousels).
    bool lookChanged = false;
    for (int k = 0; k < 8; ++k) lookChanged = lookChanged || look.at(k) != g.look.at(k);
    const fuel::RiderOutfit outfit = garagePreviewOutfit();  // PHYSIQUE
    if (lookChanged || outfit != g.outfit) {
        C c{C::Look};
        c.look = look;
        c.outfit = outfit;
        g3dCmds_.push_back(c);
        g.look = look, g.outfit = outfit;
    }
    // 0x47c1e0 per state: 1 / 0x3b -> mode 4 (0x4d0b90), 0x3c -> 0 (0x4d0bb0), 0x3d -> 3 (0x4d0bd0), 0x3e -> 1
    // (0x4d0bf0), 0x3f -> 2 (0x4d0c10); the type / livery pages keep their menu's mode, the others leave it [O].
    int mode = g.mode;
    if (screen_ == Screen::Tab || screen_ == Screen::GarageSelect) switch (garage_.state) {
            case 1: case 0x3b: mode = 4; break;
            case 0x3c: case 0x9b: case 0x9c: mode = 0; break;
            case 0x3d: case 0x9d: case 0x9e: mode = 3; break;
            case 0x3e: case 0x9f: case 0xa0: mode = 1; break;
            case 0x3f: mode = 2; break;
            case kOutfitState: mode = 4; break;  // PHYSIQUE: the whole rider
            default: break;
        }
    if (mode != g.mode) {
        C c{C::PoseMode};
        c.mode = mode;
        g3dCmds_.push_back(c);
        g.mode = mode;
    }
    // The tail of 0x478bb0: DRIVER -> 0x475370, VEHICLE / LIVERY / COLOR -> 0x47bd20 [O].
    if (item == 0) {
        if (g.riderTimer > 0) {  // 0x475370: 0 < +0x2d0, then +0x2d0 -= dt; < 0 -> the rider scene on, +0x2d0 = -1
            g.riderTimer -= dt;
            if (g.riderTimer < 0) push(C::ShowRider), g.riderTimer = -1;
        }
        C t{C::TickRider};
        t.dt = dt;
        g3dCmds_.push_back(t);
    } else if (((item >= 1 && item <= 3) || item == 5) && !veh.empty()) {  // 0x47bd20: +0xc8 >= 0 ([upgrades U3] 5: UPGRADES)
        if (g.vehicleTimer > 0) {
            g.vehicleTimer -= dt;
            if (g.vehicleTimer < 0) {
                C l{C::LoadVehicle};
                l.vehicle = veh;
                l.livery = career_ ? career_->vehicleLivery(veh) : 0;                  // record +0xc
                l.rgb = garagePaintRgb(veh, career_ ? career_->vehicleColor(veh) : 0);  // record +0xd
                g3dCmds_.push_back(l);
                g.vehicleTimer = -1, g.loaded = true;
                g.stages.fill(-1);  // [upgrades U3] a new vehicle object: its stages again
            }
        }
        // [upgrades U3] the stages the preview vehicle shows (U5 draws them): its equipped ones, the UPGRADES page's browsed
        // stage in the focused category.
        if (g.loaded) {
            const std::array<int, 6> st = garagePreviewStages(veh);
            if (st != g.stages) {
                C u{C::Upgrades};
                u.stages = st;
                g3dCmds_.push_back(u);
                g.stages = st;
            }
        }
        if (g.loaded) {
            C t{C::TickVehicle};
            t.dt = dt;
            g3dCmds_.push_back(t);
        }
    }
}
// ---- PK20 end

}  // namespace fuel::ui
