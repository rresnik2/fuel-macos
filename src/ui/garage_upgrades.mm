// [upgrades U3, DEVIATION from FUEL, the user's request 2026-11-06] The garage's UPGRADES page. FUEL (2009) has no vehicle
// upgrades: everything here is the port's, drawn with FUEL's own garage art. Plan ~/re-work/fuel/notes/upgrades-plan.md
// (section 4); this package: notes/upg3.md. The rules (prices, gates, buying, equipping, the save) are U2's
// (core/career_upgrades.cpp); the stage tables U1's (core/upgrades.hpp); what the preview vehicle shows U5's.
//
// - The Garage list (Garage.tsc, GROUP_LEFT_MENU, bottom-anchored at START THE RACE! 567, 37-unit rows): UPGRADES is a copy
//   of PAINTWORK's button at PAINTWORK's place; DRIVER .. PAINTWORK move up one row (37, x - 2 along the slant), as GRAPHICS
//   does in the Options list (options_pages.mm). Enabled when the garage's vehicle is owned and has a class (the 0x47c050
//   rule of LIVERY / PAINTWORK); with `upgrades = off` it is hidden and the list is FUEL's.
// - The page (port state 0x1a0 in the page word +0x16c; Screen::GarageUpgrades): FUEL's GarageVehiclePerformanceInfos art,
//   twice, at 80 % (the two panels share the left column above BandeauHelp, 597): the categories (title bar = the vehicle's
//   name, one bar per category with four stage pips: stock, 1, 2, 3) and FUEL's seven performance rows with the stages' gains
//   (plan 4.3: the stock rating in FUEL's yellow, the equipped gain in white, the browsed stage's change pulsing at FUEL's
//   halo rhythm). FUEL's price box (GarageVehiclePriceInfos) at its place: the stage, its price / OWNED / EQUIPPED, YOUR_FUEL;
//   two lines under it: what the stage does in numbers and why it is locked. The 3-D preview (PK20) shows the vehicle with
//   the browsed stage (Garage3DCmd::Upgrades -> GaragePreview::setUpgradeStages: U5 draws it).
// - Keys: Up / Down the category (no wrap; the stage cursor lands on its equipped stage), Left / Right the stage (no wrap),
//   Accept: an owned stage is equipped (free, saved); else FUEL's info boxes with port texts: InfoBoxGarageVehicleIsLocked
//   (needs 20 / 45 stars, the vehicle not owned), InfoBoxGarageVehicleIsTooExpensive, InfoBoxGarageYesNo (Accept = buy:
//   Career::buyUpgrade, which equips it; saved as the vehicle purchase is, Menu::markDirty -> saveCareer). Back: the Garage
//   list with its focus on UPGRADES (garageReturn). The pointer: a row selects, a pip selects it, a click on a pip is Accept;
//   the wheel = Up / Down (the rows scroll with FUEL's arrows when they do not fit, pad-ux's rule).
// - The legend (BandeauHelp) and the help line are port texts with FUEL's {STR_*} variables: the real pad prompts that follow
//   the bindings (pad-ux / remap), or the key caps.
#include "ui/menu.h"

#include "core/upgrades.hpp"
#include "ui/help_texts.hpp"
#include "ui/menu_input.h"
#include "ui/pad_prompts.hpp"
#include "ui/ui_renderer.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <unistd.h>

namespace fuel::ui {

namespace {

// Port texts (ids outside FUEL's table, next to remap's 0x7f52....).
constexpr uint32_t kTtLegendBuy = 0x7f530001;    // "STR_MENU_VALID Buy upgrade ..."
constexpr uint32_t kTtLegendEquip = 0x7f530002;  // "STR_MENU_VALID Equip ..."
constexpr uint32_t kTtLegendView = 0x7f530003;   // the equipped stage hovered: no action
constexpr uint32_t kTtHelp = 0x7f530004;         // the page's help line
constexpr uint32_t kTtTitle = 0x7f530005;        // TopTitle "UPGRADES"
const char* const kLegendKeys[3] = {"TT_GARAGE_UPGRADES_BUY", "TT_GARAGE_UPGRADES_EQUIP", "TT_GARAGE_UPGRADES_VIEW"};

// FUEL's GarageVehiclePerformanceInfos [O, the page script]: title bar BARRE_GARAGE_PERFORMANCES_TITRE at (54, 217) with
// ITEM_ID_VEHICLE_NAME, rows of BARRE_GARAGE_PERFORMANCES_PART_GRIS (70, 253 + 37 k, 413 x 32) with the dark label part
// ITEM_ID_BARRE_GARAGE_DEGRADE<k + 1> and the label (101, 258 + 37 k), six BARRE_GARAGE_PERF_POINT pips (352 + 18 i, 267
// + 37 k; 23 x 12, centred) in 255:255:156:18. [port] Both panels at 80 % (kS) so that six categories and seven rows fit.
constexpr float kS = 0.8f;
constexpr float kPanelX = 54.0f, kPanelY = 217.0f, kRow0 = 253.0f, kPitch = 37.0f;
constexpr float kCatTop = 172.0f;    // the categories' title bar (under the tab icons, ~165)
constexpr float kStatsRow0 = 384.0f;  // the performance rows: 384 .. 587 (BandeauHelp at 597)
constexpr int kVisibleRows = 6;      // the category rows that fit (pad-ux: more would scroll)
constexpr float kStagePitch = 30.0f;  // stage pips (in FUEL's panel units): 1.4 x FUEL's pip, 30 apart
constexpr float kStagePipScale = 1.4f;
constexpr float kInfoY = 558.0f;     // the two lines under the price box (552) and above BandeauHelp (597)

const char* const kCatName[upg::kCategories] = {"ENGINE", "SUSPENSION", "TIRES", "WEIGHT REDUCTION", "NITROUS POWER", "NITROUS TANK"};

// The performance rows (0x47b330 order) and the stage gains in pips of six (plan 4.3): SPEED + 1/3 per engine stage,
// ACCELERATION + 1/4 per engine and + 1/3 per weight stage, GRIP + 1/3 and BRAKE + 1/4 per tyre stage, RELIABILITY + 1/3
// per suspension stage; ASPHALT / OFFROAD none (the tyres keep the vehicle's character).
struct StatRow {
    const char* id;
    int arg;  // VehicleInfo::bars index (AddVehicleInterfaceParams argument order, as menu.mm's kRows)
};
const StatRow kStats[7] = {{"ITEM_ID_SPEED", 0}, {"ITEM_ID_ACCELERATION", 1}, {"ITEM_ID_GRIP", 2}, {"ITEM_ID_BRAKE", 3},
                           {"ITEM_ID_RELIABILITY", 6}, {"ITEM_ID_ASPHALT", 4}, {"ITEM_ID_OFFROAD", 5}};
float statGain(int row, const upg::Stages& st) {
    switch (row) {
        case 0: return st[upg::Engine] / 3.0f;
        case 1: return st[upg::Engine] / 4.0f + st[upg::Weight] / 3.0f;
        case 2: return st[upg::Tires] / 3.0f;
        case 3: return st[upg::Tires] / 4.0f;
        case 4: return st[upg::Suspension] / 3.0f;
        default: return 0;
    }
}

// An item of the performance panel placed at 80 % with the panel's origin at (ox, oy) and `dy` panel units lower.
Item place(const Item& src, float ox, float oy, float dy = 0) {
    Item it = src;
    it.x = ox + (src.x - kPanelX) * kS;
    it.y = oy + (src.y + dy - kPanelY) * kS;
    it.w *= kS, it.h *= kS;
    it.xOffset *= kS;
    it.drawW *= kS, it.drawH *= kS;
    it.focus = false, it.offX = it.offY = 0;
    return it;
}

std::string upperOf(std::string s) {
    for (char& c : s) c = char(std::toupper(static_cast<unsigned char>(c)));
    return s;
}

std::string fmt(const char* f, double v) {
    char b[64];
    std::snprintf(b, sizeof b, f, v);
    return b;
}

}  // namespace

// ---- the Garage list item --------------------------------------------------------------------------------------------

bool Menu::upgradesShown() const { return port_.upgrades; }

// UPGRADES in the Garage list: PAINTWORK's button copied to PAINTWORK's place (530); DRIVER .. PAINTWORK move up one row
// while it is shown (layoutGarageList). The authored places are kept for `upgrades = off` (FUEL's list).
void Menu::addUpgradesItem() {
    Page* p = scripts_.page("Garage");
    if (!p || p->find("ITEM_ID_TOPMENU_GARAGE_UPGRADES")) return;
    const Item* col = p->find("ITEM_ID_TOPMENU_GARAGE_COLOR");
    if (!col) return;
    static const char* const kIds[4] = {"ITEM_ID_TOPMENU_GARAGE_DRIVER", "ITEM_ID_TOPMENU_GARAGE_VEHICLE", "ITEM_ID_TOPMENU_GARAGE_LIVERY",
                                        "ITEM_ID_TOPMENU_GARAGE_COLOR"};
    for (int k = 0; k < 4; ++k)
        if (const Item* it = p->find(kIds[k])) garageListX_[size_t(k)] = it->x, garageListY_[size_t(k)] = it->y;
    Item u = *col;
    u.id = "ITEM_ID_TOPMENU_GARAGE_UPGRADES";
    u.hasDynText = true, u.dynText = "UPGRADES";
    u.x += 1.0f;  // between PAINTWORK (6) and START THE RACE! (8) along the slant
    u.a7 += 1;
    p->items.push_back(u);
}

void Menu::layoutGarageList() {
    Page* p = scripts_.page("Garage");
    if (!p) return;
    Item* u = p->find("ITEM_ID_TOPMENU_GARAGE_UPGRADES");
    if (!u) return;
    const bool on = upgradesShown();
    static const char* const kIds[4] = {"ITEM_ID_TOPMENU_GARAGE_DRIVER", "ITEM_ID_TOPMENU_GARAGE_VEHICLE", "ITEM_ID_TOPMENU_GARAGE_LIVERY",
                                        "ITEM_ID_TOPMENU_GARAGE_COLOR"};
    for (int k = 0; k < 4; ++k)
        if (Item* it = p->find(kIds[k])) it->y = garageListY_[size_t(k)] - (on ? 37.0f : 0.0f), it->x = garageListX_[size_t(k)] - (on ? 2.0f : 0.0f);
    u->visible = on;
    // The 0x47c050 rule of LIVERY / PAINTWORK (the garage's vehicle owned) and a vehicle with a class (U2: upgradable).
    u->enabled = on && career_ && career_->vehicleOwned(vehicle_) && career_->upgradable(vehicle_);
}

// ---- texts -------------------------------------------------------------------------------------------------------------

void Menu::upgradeTexts() {
    if (text_.get(kTtLegendBuy)) return;
    const std::string sep = "\xc2\xb2\xc2\xb2\xc2\xb2";  // FUEL's legend spacing (TT 470 ..)
    const std::string nav = "STR_MENU_UP/STR_MENU_DOWN Part" + sep + "STR_MENU_LEFT/STR_MENU_RIGHT Stage" + sep + "STR_MENU_BACK Back";
    auto put = [&](uint32_t id, std::string s) {
        helpTexts().transText(s);  // TransText 0x4e2820: "STR_X" -> "{STR_X}" (the bindings' labels at draw time)
        text_.strings[id] = s;
    };
    put(kTtLegendBuy, "STR_MENU_VALID Buy upgrade" + sep + nav);
    put(kTtLegendEquip, "STR_MENU_VALID Equip" + sep + nav);
    put(kTtLegendView, nav);
    put(kTtHelp, "Upgrade this vehicle part by part. Buying a stage buys the stages below it, and any stage you own can be "
                 "equipped again for free. No refund whatsoever!");
    text_.strings[kTtTitle] = "UPGRADES";
    auto addEntry = [](Page* p, const char* item, const std::string& key, uint32_t tt) {
        if (!p) return;
        if (Item* it = p->find(item)) {
            for (const ListEntry& e : it->entries)
                if (e.key == key) return;
            it->entries.push_back(ListEntry{key, tt, false});
        }
    };
    for (int i = 0; i < 3; ++i) addEntry(scripts_.page("BandeauHelp"), "ITEM_ID_BUTTONS", kLegendKeys[i], kTtLegendBuy + uint32_t(i));
    addEntry(scripts_.page("BandeauHelp"), "ITEM_ID_SCROLLING_HELPS", "TT_BH_GARAGE_UPGRADES", kTtHelp);
    addEntry(scripts_.page("TopTitle"), "ITEM_ID_TITLE", "TT_TOP_TITLE_GARAGE_UPGRADES", kTtTitle);
}

std::string Menu::upgradeStageName(int category, int stage) const {
    const std::string c = category >= 0 && category < upg::kCategories ? kCatName[category] : "?";
    return stage <= 0 ? c + " STOCK" : c + " STAGE " + std::to_string(stage);
}

// What a stage does to `vehicle`, in numbers (plan 4.2's line; the class factors of plan 2.3 applied).
std::string Menu::upgradeEffect(const std::string& vehicle, int category, int stage) const {
    const int cls = upg::classOf(vehicle);
    if (cls < 0) return "";
    const upg::ClassFactors& k = upg::factors(cls);
    const int s = std::clamp(stage, 0, upg::kMaxStage);
    if (s == 0) {  // FUEL's own part
        static const char* const kStock[upg::kCategories] = {"STOCK ENGINE", "STOCK SUSPENSION", "STOCK TIRES", "STOCK WEIGHT", "STOCK BOOST",
                                                             "STOCK TANK 3.0 S"};
        return category >= 0 && category < upg::kCategories ? kStock[category] : "";
    }
    switch (category) {
        case upg::Engine: return "TORQUE x" + fmt("%.2f", 1.0 + k.engine * (upg::engineGain(s) - 1.0));
        case upg::Suspension:
            return "TRAVEL x" + fmt("%.2f", upg::travelScale(s)) + ", LIFT +" + fmt("%.1f", k.lift[size_t(s - 1)] * 100.0) + " CM" +
                   (k.antiRoll > 0 && !k.bike ? ", ANTI-ROLL x" + fmt("%.2f", 1.0 + k.antiRoll * (upg::antiRollScale(s) - 1.0)) : "");
        case upg::Tires: {
            const double g = (upg::tyreScale(s) - 1.0) * 100.0;
            const double lon = g * k.tyreLong, lat = g * k.tyreLat;
            if (std::fabs(lon - lat) < 0.05) return "GRIP +" + fmt("%.0f", lat) + " %";
            if (lon < 0.05) return "SIDEWAYS GRIP +" + fmt("%.0f", lat) + " %";  // bikes: lateral rows only (U1)
            return "GRIP +" + fmt("%.0f", lon) + " %, SIDEWAYS +" + fmt("%.0f", lat) + " %";
        }
        case upg::Weight: return "WEIGHT -" + fmt("%.0f", (1.0 - upg::effectiveMass(s)) * 100.0) + " %";
        case upg::NitroPower: return "BOOST +" + fmt("%.0f", k.nitroPower * (upg::nitroPowerScale(s) - 1.0) * 100.0) + " %";
        case upg::NitroTank:
            return "TANK " + fmt("%.1f", 3.0 * upg::nitroTankScale(s)) + " S, RECHARGE +" + fmt("%.0f", (upg::nitroRechargeScale(s) - 1.0) * 100.0) + " %";
        default: return "";
    }
}

// ---- state -------------------------------------------------------------------------------------------------------------

Menu::UpgradePip Menu::upgradePip(const std::string& vehicle, int category, int stage) const {
    UpgradePip p;
    if (!career_) return p;
    const upg::Stages own = career_->upgradesOwned(vehicle), eq = career_->upgradesEquipped(vehicle);
    p.stars = stage >= 1 ? upg::starsNeeded(stage) : 0;
    if (stage == eq[category]) return p.state = UpgradePip::Equipped, p;
    if (stage <= own[category]) return p.state = UpgradePip::Owned, p;
    p.price = std::max(0, career_->upgradePrice(vehicle, category, stage));
    switch (career_->upgradeLock(vehicle, category, stage)) {
        case Career::UpgradeResult::VehicleNotOwned:
        case Career::UpgradeResult::NotUpgradable: p.state = UpgradePip::VehicleNotOwned; break;
        case Career::UpgradeResult::NeedsStars: p.state = UpgradePip::NeedsStars; break;
        case Career::UpgradeResult::TooExpensive: p.state = UpgradePip::TooExpensive; break;
        default: p.state = UpgradePip::ForSale; break;
    }
    return p;
}

std::vector<int> Menu::upgradeRows() const {
    std::vector<int> rows;
    for (int c = 0; c < upg::kCategories; ++c)
        if (port_.nitrous || (c != upg::NitroPower && c != upg::NitroTank)) rows.push_back(c);  // plan 2.2.5: nitrous = off hides them
    return rows;
}

std::array<int, 6> Menu::garagePreviewStages(const std::string& vehicle) const {
    std::array<int, 6> st{};
    if (!career_ || !port_.upgrades || vehicle.empty()) return st;
    const bool owned = career_->vehicleOwned(vehicle);
    if (owned) {
        const upg::Stages eq = career_->upgradesEquipped(vehicle);
        for (int c = 0; c < upg::kCategories && c < 6; ++c) st[size_t(c)] = eq[c];
    }
    if (screen_ == Screen::GarageUpgrades && vehicle == vehicle_ && upg_.cat >= 0 && upg_.cat < 6) st[size_t(upg_.cat)] = std::clamp(upg_.stage, 0, 3);
    return st;
}

void Menu::upgradesOpen(int category) {
    garageLoadData();
    garage_.state = kUpgradesState, garage_.page.clear(), garage_.entries.clear();
    screen_ = Screen::GarageUpgrades, tab_ = TabGarage;
    infoBox_.clear(), infoText_.clear(), buyTarget_.clear();
    upg_.buyCat = upg_.buyStage = -1;
    const auto rows = upgradeRows();
    upg_.cat = category >= 0 && std::find(rows.begin(), rows.end(), category) != rows.end() ? category : rows.empty() ? 0 : rows.front();
    upg_.stage = career_ ? career_->upgradesEquipped(vehicle_)[upg_.cat] : 0;
    upg_.scroll = ListScroll{};
    upg_.scroll.set(int(rows.size()), kVisibleRows);
    setTitle("TT_TOP_TITLE_GARAGE_UPGRADES");
}

void Menu::upgradesMove(int dCat, int dStage) {
    const auto rows = upgradeRows();
    if (rows.empty()) return;
    int row = int(std::find(rows.begin(), rows.end(), upg_.cat) - rows.begin());
    if (row >= int(rows.size())) row = 0;
    if (dCat != 0) {
        const int to = std::clamp(row + dCat, 0, int(rows.size()) - 1);  // no wrap (FUEL's lists)
        if (to != row) {
            row = to, upg_.cat = rows[size_t(row)];
            upg_.stage = career_ ? career_->upgradesEquipped(vehicle_)[upg_.cat] : 0;  // the cursor on what is fitted
        }
    }
    if (dStage != 0) upg_.stage = std::clamp(upg_.stage + dStage, 0, upg::kMaxStage);
    upg_.scroll.set(int(rows.size()), kVisibleRows);
    upg_.scroll.follow(row);
}

void Menu::upgradesKey(MenuKey k) {
    switch (k) {
        case MenuKey::Up: return upgradesMove(-1, 0);
        case MenuKey::Down: return upgradesMove(1, 0);
        case MenuKey::Left: return upgradesMove(0, -1);
        case MenuKey::Right: return upgradesMove(0, 1);
        case MenuKey::Accept: return upgradesAccept();
        case MenuKey::Back: garageBack(); return;
        default: return;
    }
}

// Accept on the hovered stage: equip an owned one (free; saved), else the lock's info box or the Yes / No box (the buy runs
// in Menu::key's info-box branch, as the vehicle purchase's).
void Menu::upgradesAccept() {
    if (!career_) return;
    const int c = upg_.cat, s = upg_.stage;
    const UpgradePip p = upgradePip(vehicle_, c, s);
    const std::string name = upgradeStageName(c, s);
    switch (p.state) {
        case UpgradePip::Equipped: return;  // nothing to do
        case UpgradePip::Owned:
            if (career_->equipUpgrade(vehicle_, c, s)) {
                std::printf("career: equipped %s %s, stages %s\n", vehicle_.c_str(), name.c_str(), upg::formatStages(career_->upgradesEquipped(vehicle_)).c_str());
                markDirty();  // saved as a vehicle purchase is (0x4ce590 +0x80 / the front end at once)
            }
            return;
        case UpgradePip::VehicleNotOwned:
            infoBox_ = "InfoBoxGarageVehicleIsLocked", infoText_ = "Buy this vehicle first to upgrade it.";
            return;
        case UpgradePip::NeedsStars:
            infoBox_ = "InfoBoxGarageVehicleIsLocked";
            infoText_ = name + " needs " + std::to_string(p.stars) + " stars. You have " + std::to_string(career_->stars()) + ".";
            return;
        case UpgradePip::TooExpensive:
            infoBox_ = "InfoBoxGarageVehicleIsTooExpensive";
            infoText_ = "You don't have enough fuel to trade for " + name + "! It costs " + std::to_string(p.price) + " fuel.";
            return;
        case UpgradePip::ForSale: {
            const int own = career_->upgradesOwned(vehicle_)[c];
            std::string incl;
            if (s - own == 2) incl = " (stage " + std::to_string(own + 1) + " included)";
            if (s - own == 3) incl = " (stages 1 and 2 included)";
            infoBox_ = "InfoBoxGarageYesNo";
            infoText_ = "Are you sure you want to trade " + std::to_string(p.price) + " fuel for " + name + incl + "? No refund whatsoever!";
            upg_.buyCat = c, upg_.buyStage = s;
            return;
        }
    }
}

// ---- drawing -----------------------------------------------------------------------------------------------------------

std::array<std::string, 3> Menu::upgradeBoxTexts() const {
    if (!career_) return {};
    const UpgradePip p = upgradePip(vehicle_, upg_.cat, upg_.stage);
    const std::string price = p.state == UpgradePip::Equipped ? "EQUIPPED" : p.state == UpgradePip::Owned ? textOf(405) : std::to_string(p.price);
    return {upgradeStageName(upg_.cat, upg_.stage), price, std::to_string(career_->state().fuel)};
}

void Menu::drawUpgrades(UiRenderer& r) {
    Page* perf = scripts_.page("GarageVehiclePerformanceInfos");
    Page* opts = scripts_.page("PageOptionsDisplay");
    if (!perf || !career_) return;
    const auto rows = upgradeRows();
    upg_.scroll.set(int(rows.size()), kVisibleRows);
    const upg::Stages own = career_->upgradesOwned(vehicle_), eq = career_->upgradesEquipped(vehicle_);
    upg::Stages browsed = eq;
    browsed[upg_.cat] = upg_.stage;
    const float pulse = 0.25f + (std::sin(time_ * 10.0f) + 1.0f) * 0.25f;  // PK18 0x46c400: FUEL's halo rhythm
    // The templates: the title bar and its label, row 1's bar, its dark label part, its label, its six pips.
    const Item *title = nullptr, *titleText = perf->find("ITEM_ID_VEHICLE_NAME"), *bar = nullptr, *degrade = perf->find("ITEM_ID_BARRE_GARAGE_DEGRADE1"),
               *label = nullptr;
    for (const Item& it : perf->items) {
        if (it.kind == ItemKind::Image && it.imgNormal == "BARRE_GARAGE_PERFORMANCES_TITRE") title = &it;
        if (it.kind == ItemKind::Image && it.imgNormal == "BARRE_GARAGE_PERFORMANCES_PART_GRIS" && it.y == kRow0) bar = &it;
        if (it.kind == ItemKind::LabelTT && it.tt == 10150) label = &it;
    }
    const Item* pip = perf->find("ITEM_ID_SPEED1");
    const Item* halo = opts ? opts->find("ITEM_ID_HALO1") : nullptr;
    const Item* haloBar = nullptr;  // the options row's bar the halo surrounds (265, 218, 764 x 34)
    if (opts)
        for (const Item& it : opts->items)
            if (it.kind == ItemKind::Image && it.imgNormal == "BARRE_GRIS_STANDARD_PART_GRIS" && it.y == 218.0f) haloBar = &it;
    if (!title || !titleText || !bar || !label || !pip) return;
    auto haloAround = [&](float x, float y, float w, float h, float alpha) {
        if (!halo || !haloBar) return;
        const float sx = w / haloBar->w, sy = h / haloBar->h;
        Item hi = *halo;
        hi.x = x + (halo->x - haloBar->x) * sx, hi.y = y + (halo->y - haloBar->y) * sy, hi.w = halo->w * sx, hi.h = halo->h * sy;
        hi.visible = true, hi.focus = false, hi.enabled = true;
        hi.cEnabled.a = alpha;
        drawItem(r, hi, 0, 0);
    };

    // ---- the categories (title = the vehicle's name, FUEL's yellow)
    const float catOy = kCatTop;
    drawItem(r, place(*title, kPanelX, catOy), 0, 0);
    {
        Item t = place(*titleText, kPanelX, catOy);
        t.hasDynText = true, t.dynText = vehicleName(vehicle_), t.visible = true;
        drawItem(r, t, 0, 0);
    }
    const int focusRow = int(std::find(rows.begin(), rows.end(), upg_.cat) - rows.begin());
    for (int slot = 0; slot < upg_.scroll.visible && upg_.scroll.first + slot < int(rows.size()); ++slot) {
        const int row = upg_.scroll.first + slot, c = rows[size_t(row)];
        const float dy = kPitch * float(slot);
        const bool focused = row == focusRow;
        const Item b = place(*bar, kPanelX, catOy, dy);
        if (focused) haloAround(b.x, b.y, b.w, b.h, pulse);  // under the bar, as FUEL's option rows draw their halo
        drawItem(r, b, 0, 0);
        if (degrade) drawItem(r, place(*degrade, kPanelX, catOy, dy), 0, 0);
        Item l = place(*label, kPanelX, catOy, dy);
        l.hasDynText = true, l.dynText = kCatName[c];
        drawItem(r, l, 0, 0);
        // Four stage pips: equipped = FUEL's pip colour, owned = the same at 40 %, for sale = grey, unaffordable = darker
        // grey, locked (stars, the vehicle not owned) = a dark slot; the stage cursor: the pip larger over a white glow.
        for (int s = 0; s <= upg::kMaxStage; ++s) {
            const UpgradePip p = upgradePip(vehicle_, c, s);
            Item q = place(*pip, kPanelX, catOy, dy);
            q.x = kPanelX + (pip->x + kStagePitch * float(s) - kPanelX) * kS;
            const bool cursor = focused && s == upg_.stage;
            const float big = kStagePipScale * (cursor ? 1.2f : 1.0f);
            q.w *= big, q.h *= big;
            Rgba col = pip->cNormal;
            switch (p.state) {
                case UpgradePip::Equipped: break;
                case UpgradePip::Owned: col.a = 0.4f; break;
                case UpgradePip::ForSale: col = Rgba{0.70f, 0.70f, 0.70f, 1}; break;
                case UpgradePip::TooExpensive: col = Rgba{0.45f, 0.45f, 0.45f, 1}; break;
                default: col = Rgba{0.07f, 0.07f, 0.07f, 1}; break;  // a dark slot in the bar
            }
            if (cursor) {  // the cursor: a white glow in the pip's shape at FUEL's halo rhythm under the pip
                Item g = q;
                g.w *= 1.45f, g.h *= 1.7f;
                g.cNormal = g.cEnabled = g.cFocus = Rgba{1, 1, 1, pulse};
                drawItem(r, g, 0, 0);
            }
            q.cNormal = q.cEnabled = q.cFocus = col;
            drawItem(r, q, 0, 0);
        }
    }
    // pad-ux: FUEL's scroll arrows when the rows do not fit (none today: six rows, six slots).
    if (upg_.scroll.count > upg_.scroll.visible)
        if (Page* cc = scripts_.page("PageOptionsControllerConfigChange"))
            for (int up = 0; up < 2; ++up)
                if (const Item* a = cc->find(up ? "ITEM_ID_FLECHE_UP" : "ITEM_ID_FLECHE_DOWN"); a && (up ? upg_.scroll.moreUp() : upg_.scroll.moreDown())) {
                    Item it = *a;
                    it.visible = true, it.x = 250.0f;
                    it.y = up ? catOy + (kRow0 - kPanelY) * kS - 22.0f : catOy + (kRow0 + kPitch * kVisibleRows - kPanelY) * kS;
                    drawItem(r, it, 0, 0);
                }

    // ---- FUEL's performance rows with the stages (no title: the vehicle's name is above)
    const VehicleInfo* vi = vehicleInfo(vehicle_);
    const float statsOy = kStatsRow0 - (kRow0 - kPanelY) * kS;
    const bool owned = career_->vehicleOwned(vehicle_);
    const upg::Stages zero{};
    for (int k = 0; k < 7; ++k) {
        const float dy = kPitch * float(k);
        drawItem(r, place(*bar, kPanelX, statsOy, dy), 0, 0);
        if (degrade) drawItem(r, place(*degrade, kPanelX, statsOy, dy), 0, 0);
        Item l = place(*label, kPanelX, statsOy, dy);
        l.tt = 10150 + uint32_t(k);  // SPEED .. OFFROAD (FUEL's labels, in its row order)
        drawItem(r, l, 0, 0);
        const float v = vi && size_t(kStats[k].arg) < vi->bars.size() ? vi->bars[size_t(kStats[k].arg)] : 0.0f;
        const float base = float(int(v * 6.0f + 0.5f));  // 0x47b330: i <= trunc(v x 6 + 0.5)
        const float now = std::min(6.0f, base + statGain(k, owned ? eq : zero));
        const float then = std::min(6.0f, base + statGain(k, browsed));
        const Item* src = perf->find(std::string(kStats[k].id) + "1");
        if (!src) continue;
        for (int i = 1; i <= 6; ++i) {
            const float lo = float(i - 1);
            Item q = place(*src, kPanelX, statsOy);  // the row's own pip 1 (already on its row)
            q.x = kPanelX + (src->x + 18.0f * float(i - 1) - kPanelX) * kS;
            q.visible = true;
            auto seg = [&](float a, float b, const Rgba& c) {  // the part of pip i between a and b (pips of six), narrower when partial
                const float f = std::clamp(std::min(b, lo + 1.0f) - std::max(a, lo), 0.0f, 1.0f);
                if (f <= 0.02f) return;
                Item s = q;
                s.w = q.w * f, s.x = q.x - q.w * 0.5f + s.w * 0.5f;
                s.cNormal = s.cEnabled = s.cFocus = c;
                drawItem(r, s, 0, 0);
            };
            seg(0, base, pip->cNormal);                                        // FUEL's rating
            seg(base, std::min(now, then), Rgba{1, 1, 1, 1});                  // the equipped stages' gain (kept)
            if (then > now) seg(now, then, Rgba{1, 1, 1, pulse});              // the browsed stage adds: pulsing
            if (then < now) seg(then, now, Rgba{1, 1, 1, pulse * 0.5f});       // the browsed stage takes away: fading
        }
    }

    // ---- FUEL's price box: the hovered stage, its price / OWNED / EQUIPPED, YOUR_FUEL (a copy: the vehicle list keeps its own)
    const UpgradePip hp = upgradePip(vehicle_, upg_.cat, upg_.stage);
    if (const Page* pp = scripts_.page("GarageVehiclePriceInfos")) {
        Page box = *pp;
        const auto t = upgradeBoxTexts();
        auto dyn = [&](const char* id, const std::string& v, bool show) {
            if (Item* it = box.find(id)) it->hasDynText = true, it->dynText = v, it->visible = show;
        };
        const bool have = hp.state == UpgradePip::Equipped || hp.state == UpgradePip::Owned;
        dyn("ITEM_ID_VEHICLE_NAME", t[0], true);
        dyn("ITEM_ID_YOUR_FUEL", t[2], true);
        dyn("ITEM_ID_VEHICLE_FUEL_PRICE", t[1], !have);
        if (Item* it = box.find("ITEM_ID_VEHICLE_OWNED")) {
            it->visible = have;
            if (hp.state == UpgradePip::Equipped) it->hasDynText = true, it->dynText = t[1];  // else FUEL's TT 405 OWNED
        }
        drawPage(r, box);
    }
    // ---- what the stage does, and why it is locked (the performance label's style, under the price box)
    {
        Item line = *label;
        line.x = 726.0f, line.w = 540.0f, line.h = 17.0f, line.xOffset = 0, line.align = 0, line.f9 = 0, line.anchorH = line.anchorV = 0;
        line.hasDynText = true, line.dynText = upgradeEffect(vehicle_, upg_.cat, upg_.stage);
        line.y = kInfoY;
        drawItem(r, line, 0, 0);
        std::string why;
        switch (hp.state) {
            case UpgradePip::Owned: why = "OWNED: EQUIP IT FOR FREE"; break;
            case UpgradePip::VehicleNotOwned: why = "BUY THE VEHICLE FIRST"; break;
            case UpgradePip::NeedsStars: why = "NEEDS " + std::to_string(hp.stars) + " STARS (YOU HAVE " + std::to_string(career_->stars()) + ")"; break;
            case UpgradePip::TooExpensive: why = "NOT ENOUGH FUEL"; break;
            case UpgradePip::ForSale:
                if (upg_.stage - own[upg_.cat] > 1) why = upg_.stage - own[upg_.cat] == 3 ? "STAGES 1 AND 2 INCLUDED" : "STAGE " + std::to_string(upg_.stage - 1) + " INCLUDED";
                break;
            default: break;
        }
        if (!why.empty()) {
            line.y = kInfoY + 19.0f, line.dynText = why;
            if (hp.state == UpgradePip::NeedsStars || hp.state == UpgradePip::TooExpensive || hp.state == UpgradePip::VehicleNotOwned)
                line.tNormal = line.tEnabled = line.tFocus = pip->cNormal;  // FUEL's yellow for a lock
            drawItem(r, line, 0, 0);
        }
    }
    if (!infoBox_.empty()) drawGarageInfoBox(r);
}

// FUEL's garage popups: the packed page script (the one FUEL loads) has no InfoBoxGarage* pages (their .tsc files are loose
// sources); its garage purchase boxes are PopUpHelp's TT_PAGE_VEHICLE_YES_NO / _IS_LOCKED / _IS_TOO_EXPENSIVE entries [O]:
// the text in ITEM_ID_POPUP_TEXTS (TT 402 / 403 / 404) and the button line ITEM_ID_BUTTONS (TT 489 "{STR_MENU_VALID} Yes
// {STR_MENU_BACK} No" / 490 "{STR_MENU_VALID} OK": the bindings' prompts). The port's state keeps the InfoBoxGarage* names
// (Menu::key: Accept = yes / OK, Back = no); [upgrades U3] infoText_ replaces the text with the port's. Also drawn for the
// vehicle purchase (the vehicle list drew the absent InfoBoxGarage* pages: nothing showed).
void Menu::drawGarageInfoBox(UiRenderer& r) {
    const Page* help = scripts_.page("PopUpHelp");
    if (!help || infoBox_.empty()) return;
    const std::string key = infoBox_ == "InfoBoxGarageYesNo"                  ? "TT_PAGE_VEHICLE_YES_NO"
                            : infoBox_ == "InfoBoxGarageVehicleIsTooExpensive" ? "TT_PAGE_VEHICLE_IS_TOO_EXPENSIVE"
                                                                               : "TT_PAGE_VEHICLE_IS_LOCKED";
    Page box = *help;
    for (Item& it : box.items) {
        if (it.kind == ItemKind::ListBoxTT)
            for (size_t i = 0; i < it.entries.size(); ++i)
                if (it.entries[i].key == key) it.entryIndex = int(i);
        if (it.id == "ITEM_ID_FOND_2" || it.id == "ITEM_ID_POPUP_TEXTS_2") it.visible = false;
        if (it.id == "ITEM_ID_BUTTONS") it.visible = true, it.align = std::clamp(it.a7, 0, 2);  // centred (a7 = 1, 0x615240)
        if (it.id == "ITEM_ID_POPUP_TEXTS" && !infoText_.empty()) it.hasDynText = true, it.dynText = infoText_;
    }
    drawPage(r, box);
}

// ---- pointer -----------------------------------------------------------------------------------------------------------

bool Menu::upgradesPointer(float x, float y, bool click) {
    if (!infoBox_.empty()) {  // the box: a click on the No half (FUEL's B No at 662 .. 761) closes, elsewhere = the action key
        if (!click) return true;
        key(infoBox_ == "InfoBoxGarageYesNo" && x >= 640.0f ? MenuKey::Back : MenuKey::Accept);
        return true;
    }
    const auto rows = upgradeRows();
    const float barX = kPanelX + (70.0f - kPanelX) * kS, barW = 413.0f * kS;
    for (int slot = 0; slot < upg_.scroll.visible && upg_.scroll.first + slot < int(rows.size()); ++slot) {
        const float y0 = kCatTop + (kRow0 + kPitch * float(slot) - kPanelY) * kS;
        if (y < y0 - 2.0f || y >= y0 + kPitch * kS - 2.0f || x < barX || x > barX + barW) continue;
        const int c = rows[size_t(upg_.scroll.first + slot)];
        const float pip0 = kPanelX + (352.0f - kPanelX) * kS, pitch = kStagePitch * kS;
        int stage = -1;
        if (x >= pip0 - pitch * 0.5f) stage = std::clamp(int(std::lround((x - pip0) / pitch)), 0, upg::kMaxStage);
        const int wasCat = upg_.cat, wasStage = upg_.stage;
        if (c != upg_.cat) upg_.cat = c, upg_.stage = career_ ? career_->upgradesEquipped(vehicle_)[c] : 0;
        if (stage >= 0) upg_.stage = stage;
        if (upg_.cat != wasCat || upg_.stage != wasStage) soundBit(4);  // 0x46c510: the pointer moved the selection
        if (click && stage >= 0) soundBit(2), upgradesAccept();
        return true;
    }
    if (click && upg_.scroll.count > upg_.scroll.visible) {  // FUEL's arrows (pad-ux)
        const float upY = kCatTop + (kRow0 - kPanelY) * kS - 22.0f, downY = kCatTop + (kRow0 + kPitch * kVisibleRows - kPanelY) * kS;
        if (x >= 242 && x <= 290 && std::fabs(y - upY - 10) < 14) return upgradesMove(-1, 0), true;
        if (x >= 242 && x <= 290 && std::fabs(y - downY - 10) < 14) return upgradesMove(1, 0), true;
    }
    return false;
}

// ---- --menu-shot GARAGEUPGRADES[:VEHICLE[:CATEGORY[:STAGE[:KEYS]]]] (in memory: shots never save). CATEGORY: e s t w np nt or
// 0..5; KEYS: menu keys pressed after (U D L R A B), e.g. A = Accept (the info box of a stage that is not owned).
bool Menu::upgradesShot(const std::string& args) {
    std::vector<std::string> a;
    for (size_t p = 0; p <= args.size();) {
        const size_t q = std::min(args.find(':', p), args.size());
        a.push_back(args.substr(p, q - p));
        p = q + 1;
    }
    if (!a.empty() && !a[0].empty()) {
        for (const VehicleInfo& v : vehicles_)
            if (upperOf(v.name) == upperOf(a[0])) vehicle_ = v.name;
    }
    goTab(TabGarage);
    int cat = -1;
    if (a.size() > 1 && !a[1].empty()) cat = std::isdigit(static_cast<unsigned char>(a[1][0])) ? std::atoi(a[1].c_str()) : upg::categoryFromKey(a[1]);
    upgradesOpen(cat);
    if (a.size() > 2 && !a[2].empty()) upg_.stage = std::clamp(std::atoi(a[2].c_str()), 0, upg::kMaxStage);
    if (a.size() > 3) {
        const int saves = saves_;
        const bool off = storage_.autosaveOff;
        storage_.autosaveOff = true;  // a shot never writes career.sav (an equip / buy stays in memory)
        for (char k : a[3]) {
            static const std::pair<char, MenuKey> kMap[] = {{'L', MenuKey::Left}, {'R', MenuKey::Right}, {'U', MenuKey::Up},
                                                             {'D', MenuKey::Down}, {'A', MenuKey::Accept}, {'B', MenuKey::Back}};
            for (const auto& [c, mk] : kMap)
                if (std::toupper(static_cast<unsigned char>(k)) == c) key(mk);
        }
        storage_.autosaveOff = off, saves_ = saves;
        std::printf("menu-shot: upgrades %s %s stage %d, keys %s: box %s\n", vehicle_.c_str(), kCatName[upg_.cat], upg_.stage, a[3].c_str(),
                    infoBox_.empty() ? "none" : infoBox_.c_str());
    }
    return true;
}

// ---- --menu-check -------------------------------------------------------------------------------------------------------
// [upgrades U3] The UPGRADES page on a scratch career / settings.txt (the player's files are never read or written): the
// Garage list, navigation, the pips and locks with their info boxes, buying and equipping (U2's functions, saved), the pad /
// keyboard / mouse flow through the bindings (controls = port), the legend's prompts, the preview's stages, the options row
// and the free-ride restart after a change in the pause menu.
int runUpgradePageCheck(const std::string& dataDir) {
    namespace in = fuel::input;
    int bad = 0;
    auto expect = [&](bool ok, const std::string& what) {
        if (!ok) ++bad;
        std::printf("upgrade page check: %s %s\n", ok ? "ok  " : "FAIL", what.c_str());
    };
    char tmpl[] = "/tmp/fuel-upgrade-check-XXXXXX";
    const char* dir = mkdtemp(tmpl);
    if (!dir) return 1;
    const std::string settingsFile = std::string(dir) + "/settings.txt", careerFile = std::string(dir) + "/career.sav";
    setenv("FUEL_SETTINGS", settingsFile.c_str(), 1);
    setenv("FUEL_CAREER_SAVE", careerFile.c_str(), 1);
    { std::ofstream(settingsFile) << "data = " << dataDir << "\ncontrols = port\nengine_power = 1.25\nkey.RIGHT = e\nkey.NITRO = d\n"; }
    auto fileText = [&](const std::string& f) {
        std::ifstream in(f);
        std::stringstream t;
        t << in.rdbuf();
        return t.str();
    };
    Settings s0 = loadSettings();
    s0.dataDir = dataDir;
    Menu m(dataDir, s0, MenuMode::Main);
    m.setMode(MenuMode::Main);
    Career* c = m.career();
    expect(c != nullptr, "a scratch career");
    if (!c) return bad;
    // The vehicles: the first owned one with a class that the port drives, one that is not owned.
    std::string own, notOwned;
    for (const auto& v : c->rules().vehicles) {
        if (!c->upgradable(v.label) || !m.startable_.count(v.label)) continue;
        if (own.empty() && c->vehicleOwned(v.label)) own = v.label;
        if (notOwned.empty() && !c->vehicleOwned(v.label)) notOwned = v.label;
    }
    expect(!own.empty() && !notOwned.empty(), "vehicles: owned " + own + ", not owned " + notOwned);
    if (own.empty() || notOwned.empty()) return bad;
    m.vehicle_ = own;
    auto press = [&](MenuKey k, int n = 1) {
        for (int i = 0; i < n; ++i) m.applyItemStates(), m.key(k);
    };
    auto names = [&] {
        m.applyItemStates();
        std::string out;
        for (Item* it : m.focusables()) out += (out.empty() ? "" : ",") + (it->hasDynText ? it->dynText : m.textOf(it->tt));
        return out;
    };
    auto itemY = [&](const char* id) {
        const Item* it = m.scripts_.page("Garage") ? m.scripts_.page("Garage")->find(id) : nullptr;
        return it ? it->y : -1.0f;
    };
    auto st = [](const upg::Stages& s) { return upg::formatStages(s); };
    // ---- the Garage list
    m.goTab(Menu::TabGarage);
    expect(names() == "DRIVER,VEHICLE,LIVERY,PAINTWORK,UPGRADES,GO OFFLINE FREE RIDE", "Garage list: UPGRADES between PAINTWORK and START (" + names() + ")");
    expect(itemY("ITEM_ID_TOPMENU_GARAGE_UPGRADES") == 530.0f && itemY("ITEM_ID_TOPMENU_GARAGE_START_RACE") == 567.0f &&
               itemY("ITEM_ID_TOPMENU_GARAGE_DRIVER") == 382.0f && itemY("ITEM_ID_TOPMENU_GARAGE_COLOR") == 493.0f,
           "UPGRADES at PAINTWORK's place (530), DRIVER .. PAINTWORK one row up, START THE RACE! stays above BandeauHelp (567)");
    m.port_.upgrades = false;
    expect(names() == "DRIVER,VEHICLE,LIVERY,PAINTWORK,GO OFFLINE FREE RIDE" && itemY("ITEM_ID_TOPMENU_GARAGE_DRIVER") == 419.0f,
           "upgrades = off: FUEL's list and places (" + names() + ")");
    m.port_.upgrades = true;
    m.vehicle_ = notOwned;
    m.applyItemStates();
    const Item* ui = m.scripts_.page("Garage")->find("ITEM_ID_TOPMENU_GARAGE_UPGRADES");
    expect(ui && ui->visible && !ui->enabled && names().find("UPGRADES") == std::string::npos,
           "a vehicle not owned: UPGRADES shown disabled (0x47c050's rule, as LIVERY / PAINTWORK)");
    m.vehicle_ = own;
    // ---- the way in, the title, the legend
    m.focus_ = 4, press(MenuKey::Accept);
    expect(m.screen_ == Menu::Screen::GarageUpgrades && m.garage_.state == Menu::kUpgradesState && m.upg_.cat == upg::Engine && m.upg_.stage == 0,
           "Accept on UPGRADES: the page (state 0x1a0) on ENGINE, the cursor on the equipped stage (stock)");
    expect(m.chrome().title == "TT_TOP_TITLE_GARAGE_UPGRADES" && m.chrome().help == "TT_BH_GARAGE_UPGRADES" && m.chrome().buttons == "TT_GARAGE_UPGRADES_VIEW",
           "title UPGRADES, help line, legend without an action on the equipped stage");
    // ---- navigation (no wrap; the stage cursor on each row's equipped stage)
    press(MenuKey::Up);
    expect(m.upg_.cat == upg::Engine, "Up on ENGINE stays");
    std::string cats;
    for (int i = 0; i < 7; ++i) press(MenuKey::Down), cats += std::to_string(m.upg_.cat);
    expect(cats == "1234555", "Down: suspension, tires, weight, nitrous power, nitrous tank, stops (" + cats + ")");
    press(MenuKey::Up, 10);
    press(MenuKey::Right, 5);
    expect(m.upg_.cat == upg::Engine && m.upg_.stage == 3, "Right stops on stage 3");
    press(MenuKey::Left, 5);
    expect(m.upg_.stage == 0, "Left stops on stock");
    m.port_.nitrous = false;
    press(MenuKey::Down, 10);
    expect(m.upgradeRows().size() == 4 && m.upg_.cat == upg::Weight, "nitrous = off: the nitrous rows are hidden (4 rows, Down stops on WEIGHT)");
    m.port_.nitrous = true;
    press(MenuKey::Up, 10);
    expect(m.upg_.scroll.count == 6 && m.upg_.scroll.visible == 6 && !m.upg_.scroll.moreDown(), "six rows fit: no scroll window");
    // ---- pips and locks: a new career has no stars and no fuel
    c->state().fuel = 0;
    const int e1 = c->upgradePrice(own, upg::Engine, 1);
    expect(m.upgradePip(own, upg::Engine, 0).state == Menu::UpgradePip::Equipped && m.upgradePip(own, upg::Engine, 1).state == Menu::UpgradePip::TooExpensive &&
               m.upgradePip(own, upg::Engine, 2).state == Menu::UpgradePip::NeedsStars && m.upgradePip(own, upg::Engine, 2).stars == 20 &&
               m.upgradePip(own, upg::Engine, 3).stars == 45 && m.upgradePip(notOwned, upg::Engine, 1).state == Menu::UpgradePip::VehicleNotOwned,
           "pips: stock equipped, stage 1 too expensive, stages 2 / 3 need 20 / 45 stars, a vehicle not owned locked");
    press(MenuKey::Right);
    expect(m.chrome().buttons == "TT_GARAGE_UPGRADES_BUY" && m.upgradeBoxTexts()[0] == "ENGINE STAGE 1" && m.upgradeBoxTexts()[1] == std::to_string(e1) &&
               m.upgradeBoxTexts()[2] == "0",
           "stage 1 hovered: legend Buy, the price box ENGINE STAGE 1 / " + std::to_string(e1) + " / YOUR_FUEL 0");
    press(MenuKey::Accept);
    expect(m.infoBox() == "InfoBoxGarageVehicleIsTooExpensive" && m.infoText().find(std::to_string(e1)) != std::string::npos && c->upgradesOwned(own).stock(),
           "Accept without the fuel: FUEL's too-expensive box (" + m.infoText() + "), nothing bought");
    press(MenuKey::Down);
    expect(m.upg_.cat == upg::Engine && !m.infoBox().empty(), "the box has the keys (Down does nothing)");
    press(MenuKey::Accept);
    expect(m.infoBox().empty(), "its OK closes it");
    press(MenuKey::Right);
    press(MenuKey::Accept);
    expect(m.infoBox() == "InfoBoxGarageVehicleIsLocked" && m.infoText().find("needs 20 stars") != std::string::npos,
           "stage 2 at 0 stars: FUEL's locked box (" + m.infoText() + ")");
    press(MenuKey::Back);
    expect(m.infoBox().empty() && m.screen_ == Menu::Screen::GarageUpgrades, "Back closes the box, the page stays");
    m.vehicle_ = notOwned;
    m.upgradesOpen(upg::Tires);
    press(MenuKey::Right), press(MenuKey::Accept);
    expect(m.infoBox() == "InfoBoxGarageVehicleIsLocked" && m.infoText().find("Buy this vehicle first") != std::string::npos,
           "a vehicle not owned: \"" + m.infoText() + "\"");
    press(MenuKey::Accept);
    m.vehicle_ = own;
    m.upgradesOpen(upg::Engine);
    // ---- buying: the Yes / No box, No, Yes; saved; the fuel on screen
    c->state().fuel = 1000000;
    const int saves0 = m.saveCount();
    press(MenuKey::Right), press(MenuKey::Accept);
    expect(m.infoBox() == "InfoBoxGarageYesNo" && m.infoText().find(std::to_string(e1) + " fuel for ENGINE STAGE 1") != std::string::npos,
           "stage 1 for sale: FUEL's Yes / No box (" + m.infoText() + ")");
    press(MenuKey::Back);
    expect(m.infoBox().empty() && c->upgradesOwned(own).stock() && c->state().fuel == 1000000 && m.saveCount() == saves0, "No (Back): nothing bought, nothing saved");
    press(MenuKey::Accept), press(MenuKey::Accept);
    expect(c->upgradesOwned(own)[upg::Engine] == 1 && c->upgradesEquipped(own)[upg::Engine] == 1 && c->state().fuel == 1000000u - uint32_t(e1) &&
               m.saveCount() == saves0 + 1,
           "Yes: engine stage 1 bought and equipped, " + std::to_string(e1) + " fuel spent, career.sav written (Menu::saveCareer)");
    expect(m.upgradeBoxTexts()[2] == std::to_string(1000000 - e1) && m.upgradeBoxTexts()[1] == "EQUIPPED" && m.chrome().buttons == "TT_GARAGE_UPGRADES_VIEW",
           "the price box: EQUIPPED, YOUR_FUEL " + m.upgradeBoxTexts()[2]);
    {
        Career d(loadCareerRules(dataDir));
        expect(d.load(careerFile) && d.upgradesOwned(own)[upg::Engine] == 1 && d.state().fuel == 1000000u - uint32_t(e1), "read back from career.sav");
    }
    // Equip: stock again (free), then stage 1 again.
    press(MenuKey::Left);
    expect(m.chrome().buttons == "TT_GARAGE_UPGRADES_EQUIP" && m.upgradeBoxTexts()[1] == m.textOf(405), "stock hovered: legend Equip, price box OWNED (TT 405)");
    press(MenuKey::Accept);
    expect(c->upgradesEquipped(own)[upg::Engine] == 0 && c->upgradesOwned(own)[upg::Engine] == 1 && c->state().fuel == 1000000u - uint32_t(e1) &&
               m.saveCount() == saves0 + 2,
           "Accept on stock: equipped for free, stage 1 kept, saved");
    // Stars: 45 (15 golds) open stage 3; buying it from stage 1 buys stage 2 with it (their sum).
    {
        std::vector<std::string> races;
        for (const auto& h : c->rules().hubs)
            for (int idx : h.career) races.push_back(c->rules().missions[size_t(idx)].name);
        for (size_t k = 0; k < 15 && k < races.size(); ++k) c->finishEvent(races[k], true, 0, 1, 100.0);
    }
    const int e23 = upg::price(c->vehicleLevel(own), upg::Engine, 2) + upg::price(c->vehicleLevel(own), upg::Engine, 3);
    const uint32_t fuel1 = c->state().fuel;
    press(MenuKey::Right, 3), press(MenuKey::Accept);
    expect(c->stars() >= 45 && m.infoBox() == "InfoBoxGarageYesNo" && m.infoText().find(std::to_string(e23)) != std::string::npos &&
               m.infoText().find("stage 2 included") != std::string::npos,
           "stage 3 at " + std::to_string(c->stars()) + " stars from stage 1: " + m.infoText());
    press(MenuKey::Accept);
    expect(st(c->upgradesOwned(own)) == "3,0,0,0,0,0" && st(c->upgradesEquipped(own)) == "3,0,0,0,0,0" && c->state().fuel == fuel1 - uint32_t(e23),
           "Yes: stages 2 and 3 bought for " + std::to_string(e23) + ", stage 3 equipped");
    // ---- the 3-D preview's stages (Garage3DCmd::Upgrades -> GaragePreview::setUpgradeStages)
    {
        m.garage3DUpdate(0, true);
        for (int i = 0; i < 20; ++i) m.garage3DUpdate(1.0f / 60.0f, true);
        auto lastStages = [&](bool* any) {
            std::array<int, 6> s{{-1, -1, -1, -1, -1, -1}};
            *any = false;
            for (const auto& cmd : m.takeGarage3D())
                if (cmd.kind == Menu::Garage3DCmd::Upgrades) s = cmd.stages, *any = true;
            return s;
        };
        bool any = false;
        const auto s1 = lastStages(&any);
        expect(any && s1 == std::array<int, 6>{{3, 0, 0, 0, 0, 0}}, "preview: the equipped stages after the vehicle loads (3,0,0,0,0,0)");
        press(MenuKey::Down), press(MenuKey::Right, 2);
        m.garage3DUpdate(1.0f / 60.0f, true);
        const auto s2 = lastStages(&any);
        expect(any && s2 == std::array<int, 6>{{3, 2, 0, 0, 0, 0}}, "preview: suspension stage 2 browsed (3,2,0,0,0,0) before it is bought");
        press(MenuKey::Back);
        m.garage3DUpdate(1.0f / 60.0f, true);
        const auto s3 = lastStages(&any);
        expect(any && s3 == std::array<int, 6>{{3, 0, 0, 0, 0, 0}} && m.screen_ == Menu::Screen::Tab && m.focusables()[size_t(m.focus_)]->id == "ITEM_ID_TOPMENU_GARAGE_UPGRADES",
               "Back: the Garage list with the focus on UPGRADES; the preview back to the equipped stages");
    }
    // ---- the keyboard through the bindings (controls = port), the pad through PadMenuRouter, the mouse, the wheel
    {
        auto keys = [&](const char* name) {
            for (MenuKey k : m.menuKeys(static_cast<unsigned short>(in::macKeyByName(name)))) m.applyItemStates(), m.key(k);
        };
        keys("return");
        expect(m.screen_ == Menu::Screen::GarageUpgrades, "keyboard: Return opens UPGRADES");
        keys("down"), keys("right");
        expect(m.upg_.cat == upg::Suspension && m.upg_.stage == 1, "keyboard: Down / Right arrows (suspension, stage 1)");
        keys("escape");
        expect(m.screen_ == Menu::Screen::Tab, "keyboard: Escape = Back");
        in::PadState p;
        p.connected = true;
        PadMenuRouter router;
        auto padPress = [&](int button) {
            p.buttons.fill(false);
            p.buttons[size_t(button)] = true;
            for (MenuKey k : router.step(m.bindings(), p, false)) m.applyItemStates(), m.key(k);
            p.buttons.fill(false);
            router.step(m.bindings(), p, false);
        };
        padPress(in::PadA);
        padPress(in::PadDDown), padPress(in::PadDDown), padPress(in::PadDRight);
        expect(m.screen_ == Menu::Screen::GarageUpgrades && m.upg_.cat == upg::Tires && m.upg_.stage == 1, "pad: A, d-pad down twice, right (tires, stage 1)");
        p.ly = -1.0f;  // the left stick down
        for (MenuKey k : router.step(m.bindings(), p, false)) m.applyItemStates(), m.key(k);
        p.ly = 0;
        router.step(m.bindings(), p, false);
        expect(m.upg_.cat == upg::Weight, "pad: the left stick down (weight)");
        padPress(in::PadB);
        expect(m.screen_ == Menu::Screen::Tab, "pad: B = Back");
        m.upgradesOpen(upg::Engine);
        // The mouse: the rows of the categories' panel (80 % of FUEL's performance panel) and the stage pips.
        const float rowY = kCatTop + (kRow0 + kPitch * 2.0f - kPanelY) * kS + 10.0f;  // the TIRES row
        const float pipX = kPanelX + (352.0f + kStagePitch * 1.0f - kPanelX) * kS;     // stage 1
        m.pointer(150.0f, rowY, false);
        expect(m.upg_.cat == upg::Tires && m.upg_.stage == 0, "mouse: hovering the TIRES label selects the row (the cursor on its equipped stage)");
        m.pointer(pipX, rowY, false);
        expect(m.upg_.cat == upg::Tires && m.upg_.stage == 1, "mouse: hovering a pip selects its stage");
        const uint32_t fuel2 = c->state().fuel;
        m.pointer(pipX, rowY, true);
        expect(m.infoBox() == "InfoBoxGarageYesNo", "mouse: a click on a pip for sale opens the Yes / No box");
        m.pointer(700.0f, 410.0f, true);
        expect(m.infoBox().empty() && c->upgradesOwned(own)[upg::Tires] == 0 && c->state().fuel == fuel2, "mouse: a click on No closes it, nothing bought");
        m.pointer(pipX, rowY, true), m.pointer(540.0f, 410.0f, true);
        expect(c->upgradesOwned(own)[upg::Tires] == 1 && c->state().fuel == fuel2 - uint32_t(upg::price(c->vehicleLevel(own), upg::Tires, 1)),
               "mouse: a click on Yes buys tires stage 1");
        m.wheel(2);
        expect(m.upg_.cat == upg::NitroPower, "the wheel: two rows down (nitrous power)");
        m.wheel(-1);
        expect(m.upg_.cat == upg::Weight, "the wheel: one row up (weight)");
        press(MenuKey::TabRight);
        expect(m.screen_ == Menu::Screen::Tab && m.tab_ != Menu::TabGarage, "the tab keys leave the page (next tab)");
    }
    // ---- the legend's prompts: the pad's buttons (they follow the bindings) or the key caps
    {
        const std::string* buy = m.text_.get(kTtLegendBuy);
        expect(buy && buy->find("{STR_MENU_VALID}") != std::string::npos && buy->find("{STR_MENU_UP}") != std::string::npos &&
                   buy->find("{STR_MENU_BACK}") != std::string::npos,
               "legend: FUEL's {STR_*} variables (MENU_VALID, MENU_UP / DOWN, MENU_LEFT / RIGHT, MENU_BACK)");
        // The draw path: textOf -> pcButtons -> helpTexts().expand (the global labels; put back to the key caps after).
        Settings ls = m.liveSettings();
        helpTexts().rebuild(true, keyboardLabels(dataDir, ls.all), padLabels(dataDir, ls.all));
        const std::string padText = m.textOf(kTtLegendBuy);
        expect(padText.find(pad::iconText(in::PadA)) != std::string::npos && padText.find(pad::iconText(in::PadB)) != std::string::npos &&
                   padText.find("Buy upgrade") != std::string::npos,
               "with a pad: A Buy upgrade ... B Back (the pad's own prompts)");
        ls.all["pad.MENU_VALID"] = "X";
        helpTexts().rebuild(true, keyboardLabels(dataDir, ls.all), padLabels(dataDir, ls.all));
        expect(m.textOf(kTtLegendBuy).find(pad::iconText(in::PadX)) != std::string::npos, "pad.MENU_VALID = X: the legend shows X (follows the bindings)");
        helpTexts().rebuild(false, keyboardLabels(dataDir, m.liveSettings().all));
        const std::string keyText = m.textOf(kTtLegendBuy);
        expect(keyText.find(pad::iconText(in::PadA)) == std::string::npos && keyText.find("Buy upgrade") != std::string::npos &&
                   keyText.find("{STR") == std::string::npos && keyText.size() > std::string(" Buy upgrade  Part  Stage  Back").size(),
               "without a pad: the key caps (controls = port)");
    }
    // ---- the options row: CONTROLS > SETTINGS, row 8 "Upgrades" (U2's PortOptions::upgrades, settings.txt `upgrades`)
    {
        m.goTab(Menu::TabOptions);
        m.focus_ = 0, press(MenuKey::Accept);  // CONTROLS
        m.focus_ = 1, press(MenuKey::Accept);  // SETTINGS
        press(MenuKey::Down, 20), press(MenuKey::Up);  // [dev mode] the last row is Dev mode now; Upgrades the one above it
        expect(m.screen_ == Menu::Screen::OptController && m.focus_ == 8 && m.port_.upgrades, "options: Upgrades is the controller row above Dev mode (ON)");
        press(MenuKey::Right);
        expect(!m.port_.upgrades && m.liveSettings().all.at("upgrades") == "off", "Right: Upgrades OFF (live: upgrades = off)");
        press(MenuKey::Accept);
        expect(fileText(settingsFile).find("upgrades = off") != std::string::npos && fileText(settingsFile).find("controls = port") != std::string::npos &&
                   !PortOptions::from(loadSettings().all).upgrades,
               "saved: upgrades = off (controls = port kept)");
        m.goTab(Menu::TabGarage);
        expect(names().find("UPGRADES") == std::string::npos && c->upgradesOwned(own)[upg::Engine] == 3, "the Garage list hides UPGRADES; the stages are kept");
        m.goTab(Menu::TabOptions);
        m.focus_ = 0, press(MenuKey::Accept), m.focus_ = 1, press(MenuKey::Accept);
        m.focus_ = 8, press(MenuKey::Left), press(MenuKey::Accept);
        expect(m.port_.upgrades && PortOptions::from(loadSettings().all).upgrades, "and back ON (saved)");
    }
    // ---- the pause menu in free ride: a stage bought there restarts the session with it (plan 4.6, as a vehicle change)
    {
        Menu pm(dataDir, loadSettings(), MenuMode::Pause);
        pm.setSession("", "YELLOWSTONE", own);
        pm.setMode(MenuMode::Pause);
        pm.request_.kind = MenuRequest::Resume;
        expect(pm.take().kind == MenuRequest::Resume, "pause: nothing changed -> Resume");
        if (Career* pc = pm.career()) {
            pc->state().fuel = 1000000;
            pm.goTab(Menu::TabGarage);
            pm.upgradesOpen(upg::Weight);
            pm.applyItemStates(), pm.key(MenuKey::Right), pm.key(MenuKey::Accept), pm.key(MenuKey::Accept);
            expect(pc->upgradesOwned(own)[upg::Weight] == 1 && pm.pauseDirty(), "pause: weight stage 1 bought (saved when the menu closes, +0x80)");
            pm.request_.kind = MenuRequest::Resume;
            const MenuRequest r = pm.take();
            expect(r.kind == MenuRequest::FreeRide && r.at && r.vehicle == own, "pause: Resume after the change -> free ride again where the player is (new --upgrades)");
        }
    }
    std::remove(settingsFile.c_str()), std::remove(careerFile.c_str()), rmdir(dir);
    std::printf("upgrade page check: %s (%d failures)\n", bad ? "FAILED" : "ok", bad);
    return bad;
}

}  // namespace fuel::ui
