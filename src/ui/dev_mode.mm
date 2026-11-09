// [DEVIATION dev mode, the user's request 2026-11-07: "Unlock all vehicles and upgrades for me in dev mode."] The menus' side
// of the career's dev-mode overlay (core/career.hpp Career::setDevMode; notes ~/re-work/fuel/notes/devmode.md). FUEL has no
// such mode: everything here is the port's.
//
// - settings.txt `dev_mode = on|off` (PortOptions::devMode, default off), the last row of CONTROLS > SETTINGS ("Dev mode"),
//   live like the other rows (options_pages.mm calls applyDevMode on the change; Back / Yes restores it the same way).
// - applyDevMode: the career's overlay follows port_ (the constructor sets it before the vehicle pick). Turned off while the
//   garage's vehicle is one only the overlay owned, the garage goes back to the constructor's choice (+0x33c8 when the real
//   career owns it, else its first owned vehicle the port drives): a free ride resumed from the pause menu then restarts
//   on it (Menu::take's vehicle-change rule).
// - The DEV tag: a small "DEV" in FUEL's orange at the right end of the tab row on every garage page (the vehicle carousel
//   for free roam, a race or a challenge, LIVERY / PAINTWORK, DRIVER, UPGRADES), in the front-end and the pause menus.
// - --menu-check (runDevModeCheck, called by runMenuCheck): see the function.
#include "ui/menu.h"

#include "core/career.hpp"
#include "core/upgrades.hpp"
#include "ui/game_menu.h"  // sessionArgs
#include "ui/ui_renderer.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <sstream>
#include <unistd.h>

namespace fuel::ui {

void Menu::applyDevMode() {
    if (!career_) return;
    career_->setDevMode(port_.devMode);
    if (vehicle_.empty() || career_->rules().vehicleIndex(vehicle_) < 0 || career_->vehicleOwned(vehicle_)) return;
    // The constructor's rule: the vehicle the player last bought (+0x33c8) when the real career owns it, else the first owned
    // one the port can drive.
    const auto& rv = career_->rules().vehicles;
    const int cur = career_->state().currentVehicle();
    if (cur >= 0 && size_t(cur) < rv.size() && career_->vehicleOwned(rv[size_t(cur)].label) && startable_.count(rv[size_t(cur)].label)) {
        vehicle_ = rv[size_t(cur)].label;
        return;
    }
    for (const auto& v : rv)
        if (v.cls >= 0 && career_->vehicleOwned(v.label) && startable_.count(v.label)) {
            vehicle_ = v.label;
            return;
        }
}

bool Menu::devTagShown() const {
    if (!career_ || !career_->devMode()) return false;
    return screen_ == Screen::VehicleList || screen_ == Screen::GarageSelect || screen_ == Screen::GarageUpgrades ||
           (screen_ == Screen::Tab && tab_ == TabGarage);
}

// "DEV" right-aligned on the safe frame's right edge (1216, as FUEL's PAUSE under it) at the height of the tab icons (~139),
// in the performance labels' font at the effect line's size, FUEL's pip orange: small, and on every garage page.
void Menu::drawDevTag(UiRenderer& r) {
    if (!devTagShown()) return;
    const Page* perf = scripts_.page("GarageVehiclePerformanceInfos");
    if (!perf) return;
    const Item *label = nullptr, *pip = perf->find("ITEM_ID_SPEED1");
    for (const Item& it : perf->items)
        if (it.kind == ItemKind::LabelTT && it.tt == 10150) label = &it;
    if (!label) return;
    Item t = *label;
    t.x = 1096.0f, t.y = 130.0f, t.w = 120.0f, t.h = 17.0f;
    t.xOffset = 0, t.align = 2, t.f9 = 0, t.anchorH = t.anchorV = 0, t.offX = t.offY = 0;
    t.hasDynText = true, t.dynText = "DEV", t.visible = true, t.focus = false;
    if (pip) t.tNormal = t.tEnabled = t.tFocus = pip->cNormal;
    drawItem(r, t, 0, 0);
}

// ---- --menu-check -------------------------------------------------------------------------------------------------------
// Dev mode on a scratch career / settings.txt (the player's files are never read or written): the options row (off by
// default, live, Back / Yes restores, saved), every vehicle of the garage carousel and of a race's list owned with no buy box,
// the UPGRADES page all owned (Accept equips any stage for free, no box), the DEV tag, the session's arguments, the save
// (FUEL's sections and PORT_UPGRADES byte for byte the real career's, the choices in PORT_DEVMODE), dev mode off again (the
// real career: vehicle, list, stages), the save loaded with dev mode off, and the pause menu reading `dev_mode`.
int runDevModeCheck(const std::string& dataDir) {
    int bad = 0;
    auto expect = [&](bool ok, const std::string& what) {
        if (!ok) ++bad;
        std::printf("dev mode check: %s %s\n", ok ? "ok  " : "FAIL", what.c_str());
    };
    char tmpl[] = "/tmp/fuel-devmode-check-XXXXXX";
    const char* dir = mkdtemp(tmpl);
    if (!dir) return 1;
    const std::string settingsFile = std::string(dir) + "/settings.txt", careerFile = std::string(dir) + "/career.sav";
    setenv("FUEL_SETTINGS", settingsFile.c_str(), 1);
    setenv("FUEL_CAREER_SAVE", careerFile.c_str(), 1);
    { std::ofstream(settingsFile) << "data = " << dataDir << "\nengine_power = 1.25\n"; }
    auto fileBytes = [](const std::string& f) {
        std::ifstream in(f, std::ios::binary);
        return std::vector<uint8_t>((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
    };
    auto fileText = [&](const std::string& f) {
        const auto b = fileBytes(f);
        return std::string(b.begin(), b.end());
    };
    auto st = [](const upg::Stages& s) { return upg::formatStages(s); };
    // The section stream of a career.sav (FUELPSAV header, payload; the PK25 trailer after it) without one section.
    auto payload = [](const std::vector<uint8_t>& file) {
        uint32_t size = 0;
        if (file.size() < 16) return std::vector<uint8_t>{};
        std::memcpy(&size, &file[12], 4);
        return 16 + size_t(size) <= file.size() ? std::vector<uint8_t>(file.begin() + 16, file.begin() + 16 + long(size)) : std::vector<uint8_t>{};
    };
    auto without = [](std::vector<uint8_t> v, const std::string& name) {
        const std::string tag = "SECTION_" + name;
        const auto it = std::search(v.begin(), v.end(), tag.begin(), tag.end());
        if (it == v.end()) return v;
        const std::string next = "SECTION_";
        const auto end = std::search(it + 1, v.end(), next.begin(), next.end());
        v.erase(it, end);
        return v;
    };
    auto has = [](const std::vector<uint8_t>& v, const std::string& name) {
        const std::string tag = "SECTION_" + name;
        return std::search(v.begin(), v.end(), tag.begin(), tag.end()) != v.end();
    };
    auto section = [](const std::vector<uint8_t>& v, const std::string& name) {  // "SECTION_<name>" up to the next section
        const std::string tag = "SECTION_" + name + std::string(1, '\0');
        const auto it = std::search(v.begin(), v.end(), tag.begin(), tag.end());
        if (it == v.end()) return std::vector<uint8_t>{};
        const std::string next = "SECTION_";
        return std::vector<uint8_t>(it, std::search(it + 1, v.end(), next.begin(), next.end()));
    };
    // The sections holding the real records the overlay must never write: ownership / livery / colour / time (FUEL's
    // VEHICLES_INFOS, the packs' PORT_VEHICLES) and the stages (PORT_UPGRADES).
    auto realRecords = [&](const std::vector<uint8_t>& stream) {
        std::vector<uint8_t> out;
        for (const char* n : {"VEHICLES_INFOS", "PORT_VEHICLES", "PORT_UPGRADES"}) {
            const auto b = section(stream, n);
            out.insert(out.end(), b.begin(), b.end());
        }
        return out;
    };

    // ---- a real career: a new game, some fuel, its first owned vehicle with engine stage 1 bought (equipped stock again)
    const CareerRules rules = loadCareerRules(dataDir);
    std::string own;
    {
        Career c(rules);
        c.newGame();
        c.state().fuel = 50000;
        for (const auto& v : rules.vehicles)
            if (own.empty() && v.cls >= 0 && !v.custom && c.vehicleOwned(v.label)) own = v.label;
        c.buyUpgrade(own, upg::Engine, 1);
        c.equipUpgrade(own, upg::Engine, 0);
        c.buyUpgrade(own, upg::Tires, 1);
        expect(!own.empty() && c.write(careerFile) == 0, "a scratch career: " + own + " owned, engine stage 1 bought (stock equipped), tires stage 1");
    }
    const std::vector<uint8_t> realFile = fileBytes(careerFile);
    Career real(rules);
    expect(real.read(careerFile) == 0 && st(real.upgradesOwned(own)) == "1,0,1,0,0,0" && st(real.upgradesEquipped(own)) == "0,0,1,0,0,0",
           "the real career reads back (owned 1,0,1,0,0,0, equipped 0,0,1,0,0,0)");

    Settings s0 = loadSettings();
    s0.dataDir = dataDir;
    Menu m(dataDir, s0, MenuMode::Main);
    m.setMode(MenuMode::Main);
    Career* c = m.career();
    expect(c && !m.port_.devMode && !c->devMode(), "dev_mode absent from settings.txt: off (the default)");
    if (!c) return bad;
    auto press = [&](MenuKey k, int n = 1) {
        for (int i = 0; i < n; ++i) m.applyItemStates(), m.key(k);
    };
    // The vehicles: a classed one the port drives that the real career does not own (a pack's when there is one).
    std::string notOwned, packNotOwned;
    for (const auto& v : rules.vehicles) {
        if (v.cls < 0 || !m.startable_.count(v.label) || c->vehicleOwned(v.label)) continue;
        if (notOwned.empty() && !v.custom) notOwned = v.label;
        if (packNotOwned.empty() && v.custom) packNotOwned = v.label;
    }
    expect(!notOwned.empty(), "a vehicle the real career does not own: " + notOwned + (packNotOwned.empty() ? "" : ", pack " + packNotOwned));
    if (notOwned.empty()) return bad;
    int classed = 0;
    for (const auto& v : rules.vehicles) classed += v.cls >= 0;
    auto ownedCount = [&] {
        int n = 0;
        for (const auto& v : rules.vehicles) n += v.cls >= 0 && c->vehicleOwned(v.label);
        return n;
    };
    m.pendingEvent_.clear();
    const size_t listOff = m.vehicleChoices().size();
    const int ownedOff = ownedCount();
    // A career race whose class has vehicles the real career has not unlocked.
    const Event* race = nullptr;
    size_t raceOff = 0;
    for (const Event& e : m.events()) {
        if (e.kind != Event::Career || e.vehicleFamily.empty()) continue;
        m.pendingEvent_ = e.id;
        const size_t n = m.vehicleChoices().size();
        size_t all = 0;
        c->setDevMode(true);
        all = m.vehicleChoices().size();
        c->setDevMode(false);
        if (all > n && !race) race = &e, raceOff = n;
    }
    m.pendingEvent_.clear();
    const Event* challenge = nullptr;
    size_t challengeOff = 0;
    for (const Event& e : m.events()) {
        if (e.kind != Event::Challenge || e.vehicleFamily.empty()) continue;
        m.pendingEvent_ = e.id;
        const size_t n = m.vehicleChoices().size();
        c->setDevMode(true);
        const size_t all = m.vehicleChoices().size();
        c->setDevMode(false);
        if (all > n && !challenge) challenge = &e, challengeOff = n;
    }
    m.pendingEvent_.clear();

    // ---- the options row: CONTROLS > SETTINGS, the last row
    m.goTab(Menu::TabOptions);
    m.focus_ = 0, press(MenuKey::Accept);
    m.focus_ = 1, press(MenuKey::Accept);
    expect(m.screen_ == Menu::Screen::OptController, "CONTROLS > SETTINGS opens");
    press(MenuKey::Down, 12);
    expect(m.focus_ == 9, "Dev mode is the last row (9)");
    const int serial0 = m.optionsSerial();
    press(MenuKey::Right);
    expect(m.port_.devMode && c->devMode() && m.liveSettings().all.at("dev_mode") == "on" && m.optionsSerial() == serial0 + 1,
           "Right: Dev mode on, live (the career's overlay at once; live dev_mode = on)");
    expect(!m.devTagShown(), "no DEV tag on the options page");
    press(MenuKey::Back), press(MenuKey::Left), press(MenuKey::Accept);  // Back -> FUEL's popup -> Yes (restore)
    expect(!m.port_.devMode && !c->devMode() && m.screen_ == Menu::Screen::Tab, "Back / Yes restores: off, the overlay off");
    m.focus_ = 1, press(MenuKey::Accept);
    m.focus_ = 9, press(MenuKey::Left);
    press(MenuKey::Accept);  // the action key keeps and saves
    {
        const PortOptions ro = PortOptions::from(loadSettings().all);
        expect(ro.devMode && fileText(settingsFile).find("dev_mode = on") != std::string::npos && c->devMode(),
               "the action key: dev_mode = on saved in settings.txt and read back; the overlay on");
    }
    // The options' keep saves career.sav (storeOptionsInCareer, FUEL's options block): nothing chosen in dev mode yet, so it is
    // the real career at this moment (the reference below); its records are those of the save written without dev mode.
    const std::vector<uint8_t> refStream = payload(fileBytes(careerFile));
    expect(!refStream.empty() && !has(refStream, "PORT_DEVMODE") && realRecords(refStream) == realRecords(payload(realFile)) &&
               !section(refStream, "PORT_UPGRADES").empty() && !section(refStream, "VEHICLES_INFOS").empty(),
           "the options' save with dev mode on: no PORT_DEVMODE; VEHICLES_INFOS / PORT_VEHICLES / PORT_UPGRADES as the real career's");

    // ---- the garage: every vehicle owned
    m.goTab(Menu::TabGarage);
    m.applyItemStates();
    expect(m.devTagShown(), "the DEV tag on the Garage list");
    const auto list = m.vehicleChoices();
    bool allOwned = !list.empty();
    for (const VehicleInfo* v : list) allOwned = allOwned && c->vehicleOwned(v->name);
    bool packListed = packNotOwned.empty();
    for (const VehicleInfo* v : list) packListed = packListed || v->name == packNotOwned;
    {   // the classed vehicles the carousel cannot hold: not in FUEL's order list (TT 3401..3475) or not drivable by the port
        std::string missing;
        for (const auto& v : rules.vehicles) {
            if (v.cls < 0) continue;
            bool in = false;
            for (const VehicleInfo* x : list) in = in || x->name == v.label;
            if (!in) missing += " " + v.label + (m.startable_.count(v.label) ? "" : "(not startable)");
        }
        std::printf("dev mode check: note: classed vehicles not in the carousel:%s\n", missing.empty() ? " none" : missing.c_str());
    }
    expect(list.size() > listOff && allOwned && packListed && ownedCount() == classed,
           "garage carousel: " + std::to_string(list.size()) + " vehicles (real career " + std::to_string(listOff) + "), every one owned; all " +
               std::to_string(classed) + " classed vehicles owned (real " + std::to_string(ownedOff) + ")" + (packNotOwned.empty() ? "" : ", the packs too"));
    {
        bool trafficOff = true;
        for (const auto& v : rules.vehicles)
            if (v.cls < 0) trafficOff = trafficOff && c->vehicleOwned(v.label) == (c->state().vehicles[size_t(rules.vehicleIndex(v.label))].owned != 0);
        expect(trafficOff, "traffic / test vehicles (no class) stay as the real career has them");
    }
    if (race) {
        m.pendingEvent_ = race->id;
        const auto rl = m.vehicleChoices();
        bool ok = rl.size() > raceOff;
        for (const VehicleInfo* v : rl) ok = ok && c->vehicleOwned(v->name);
        expect(ok, "race " + race->id + " (" + race->vehicleFamily + "): " + std::to_string(rl.size()) + " vehicles to pick (real " + std::to_string(raceOff) + "), all owned");
    } else {
        expect(false, "a career race with vehicles not unlocked in the real career");
    }
    if (challenge) {
        m.pendingEvent_ = challenge->id;
        const auto cl = m.vehicleChoices();
        bool ok = cl.size() > challengeOff;
        for (const VehicleInfo* v : cl) ok = ok && c->vehicleOwned(v->name);
        expect(ok, "challenge " + challenge->id + " (" + challenge->vehicleFamily + "): " + std::to_string(cl.size()) + " vehicles (real " +
                       std::to_string(challengeOff) + "), all owned");
    }
    m.pendingEvent_.clear();
    m.openList(Menu::Screen::VehicleList);
    {
        const auto vl = m.vehicleChoices();
        for (size_t i = 0; i < vl.size(); ++i)
            if (vl[i]->name == notOwned) m.focus_ = int(i);
    }
    expect(m.devTagShown() && m.chrome().buttons == "TT_GARAGE_VEHICLE_OWNED", "VEHICLE carousel on " + notOwned + ": the DEV tag, legend OWNED");
    const uint32_t fuel0 = c->state().fuel;
    const int saves0 = m.saveCount();
    m.vehicleAccept(notOwned);
    expect(m.infoBox_.empty() && m.vehicle_ == notOwned && c->state().fuel == fuel0 && c->state().vehicles[size_t(rules.vehicleIndex(notOwned))].owned == 0,
           "Accept on " + notOwned + ": chosen, no buy box, no fuel spent, the real record still not owned");
    expect(c->buyVehicle(notOwned) == Career::AlreadyOwned && c->state().fuel == fuel0, "Career::buyVehicle: already owned in the overlay, nothing spent");
    // The race list's Accept (the vehicle for a race) works the same: StartEvent with it.
    // ---- UPGRADES on a vehicle only the overlay owns
    m.goTab(Menu::TabGarage);
    m.applyItemStates();
    const Item* ui = m.scripts_.page("Garage")->find("ITEM_ID_TOPMENU_GARAGE_UPGRADES");
    expect(ui && ui->visible && ui->enabled, "UPGRADES enabled for " + notOwned + " (owned in the overlay)");
    m.upgradesOpen(upg::Engine);
    expect(m.devTagShown(), "the DEV tag on the UPGRADES page");
    bool pipsOwned = true;
    for (int cat = 0; cat < upg::kCategories; ++cat)
        for (int s = 0; s <= upg::kMaxStage; ++s) {
            const auto p = m.upgradePip(notOwned, cat, s);
            pipsOwned = pipsOwned && (p.state == Menu::UpgradePip::Equipped || p.state == Menu::UpgradePip::Owned);
        }
    expect(pipsOwned && c->stars() == 0, "every pip owned (or equipped) with 0 stars: no star gate, no price");
    m.upg_.stage = 3;
    press(MenuKey::Accept);
    expect(m.infoBox_.empty() && st(c->upgradesEquipped(notOwned)) == "3,0,0,0,0,0" && c->state().fuel == fuel0 && m.saveCount() == saves0 + 2,
           "Accept on ENGINE STAGE 3: equipped at once, free, no box, saved (" + st(c->upgradesEquipped(notOwned)) + ")");
    m.upg_.cat = upg::NitroTank, m.upg_.stage = 2;
    press(MenuKey::Accept);
    expect(st(c->upgradesEquipped(notOwned)) == "3,0,0,0,0,2" && c->state().fuel == fuel0, "NITROUS TANK STAGE 2 equipped, free");
    expect(c->buyUpgrade(notOwned, upg::Weight, 3) == Career::UpgradeResult::AlreadyOwned && c->state().fuel == fuel0,
           "Career::buyUpgrade: already owned in the overlay, nothing spent");
    // On the really owned vehicle: equips go to the dev store too; the real equips stay.
    m.vehicle_ = own;
    m.upgradesOpen(upg::Engine);
    expect(m.upg_.stage == 0 && st(c->upgradesEquipped(own)) == "0,0,1,0,0,0", "the real vehicle opens on its real equipped stages (0,0,1,0,0,0)");
    m.upg_.stage = 2;
    press(MenuKey::Accept);
    expect(st(c->upgradesEquipped(own)) == "2,0,1,0,0,0" && c->state().upgrades[size_t(rules.vehicleIndex(own))].equipped[upg::Engine] == 0 &&
               c->state().upgrades[size_t(rules.vehicleIndex(own))].owned[upg::Engine] == 1,
           "ENGINE STAGE 2 on " + own + " (really owned: stage 1): equipped in the overlay; the real stages unchanged (owned 1, equipped stock)");
    // A colour for the vehicle only the overlay owns: the dev store; the real record's colour stays.
    const int realColour = c->state().vehicles[size_t(rules.vehicleIndex(notOwned))].color;
    c->setVehicleColor(notOwned, (realColour + 3) % 10);
    expect(c->vehicleColor(notOwned) == (realColour + 3) % 10 && c->state().vehicles[size_t(rules.vehicleIndex(notOwned))].color == realColour,
           "PAINTWORK for " + notOwned + ": shown, kept in the dev store (FUEL's record +0xd unchanged)");
    expect(!c->vehicleLiveryChoices(notOwned).empty(), "LIVERY for " + notOwned + ": its ALWAYS liveries offered with it (" +
                                                           std::to_string(c->vehicleLiveryChoices(notOwned).size()) + " entries)");
    // ---- the session's arguments (free ride, the overlay's vehicle and its stages)
    {
        MenuRequest r;
        r.kind = MenuRequest::FreeRide, r.vehicle = notOwned, r.hub = "YellowStone";
        const auto args = sessionArgs(dataDir, r, m);
        auto arg = [&](const char* k) {
            for (size_t i = 0; i + 1 < args.size(); ++i)
                if (args[i] == k) return args[i + 1];
            return std::string();
        };
        expect(arg("--assemble") == notOwned && arg("--upgrades") == "3,0,0,0,0,2" && arg("--color") == std::to_string((realColour + 3) % 10),
               "free ride: --assemble " + notOwned + " --upgrades " + arg("--upgrades") + " --color " + arg("--color"));
    }
    // ---- the save: FUEL's sections and PORT_UPGRADES as the real career's; the dev choices in PORT_DEVMODE
    m.saveCareer();
    const std::vector<uint8_t> devFile = fileBytes(careerFile);
    const std::vector<uint8_t> devStream = payload(devFile);
    expect(has(devStream, "PORT_DEVMODE") && without(devStream, "PORT_DEVMODE") == refStream && realRecords(devStream) == realRecords(payload(realFile)),
           "career.sav after choices in dev mode: every other section byte for byte the real career's (FUEL's, PORT_VEHICLES, PORT_UPGRADES); "
           "the choices only in PORT_DEVMODE (" + std::to_string(section(devStream, "PORT_DEVMODE").size()) + " bytes)");
    {
        Career offc(rules);  // dev mode off (a Career's default)
        std::string err;
        const bool ok = offc.deserialize(devFile, &err);
        bool same = ok && err.empty();
        for (const auto& v : rules.vehicles)
            same = same && offc.vehicleOwned(v.label) == real.vehicleOwned(v.label) && offc.vehicleUnlocked(v.label) == real.vehicleUnlocked(v.label) &&
                   st(offc.upgradesOwned(v.label)) == st(real.upgradesOwned(v.label)) && st(offc.upgradesEquipped(v.label)) == st(real.upgradesEquipped(v.label)) &&
                   offc.vehicleColor(v.label) == real.vehicleColor(v.label);
        expect(same && offc.state().fuel == real.state().fuel && offc.sections() == devStream && offc.devStages(notOwned),
               "the dev-mode save loads with dev mode off: the real career exactly (ownership, stages, colours, fuel); the dev choices kept");
    }
    // ---- dev mode off again (the options row): the real career
    m.goTab(Menu::TabOptions);
    m.focus_ = 0, press(MenuKey::Accept);
    m.focus_ = 1, press(MenuKey::Accept);
    m.vehicle_ = notOwned;
    m.focus_ = 9, press(MenuKey::Right);
    expect(!m.port_.devMode && !c->devMode() && c->vehicleOwned(m.vehicle_) && m.vehicle_ != notOwned,
           "Dev mode off: the garage's vehicle back to one the real career owns (" + m.vehicle_ + ")");
    press(MenuKey::Accept);
    expect(!PortOptions::from(loadSettings().all).devMode && fileText(settingsFile).find("dev_mode = off") != std::string::npos, "dev_mode = off saved");
    m.pendingEvent_.clear();
    expect(m.vehicleChoices().size() == listOff && ownedCount() == ownedOff && !c->vehicleOwned(notOwned),
           "the garage carousel is the real career's again (" + std::to_string(m.vehicleChoices().size()) + " vehicles, " + std::to_string(ownedOff) + " owned)");
    expect(st(c->upgradesOwned(own)) == "1,0,1,0,0,0" && st(c->upgradesEquipped(own)) == "0,0,1,0,0,0" && st(c->upgradesOwned(notOwned)) == "0,0,0,0,0,0" &&
               st(c->upgradesEquipped(notOwned)) == "0,0,0,0,0,0" && c->vehicleColor(notOwned) == realColour && c->state().fuel == fuel0,
           "the real stages and colours again (" + own + " owned 1,0,1,0,0,0 equipped 0,0,1,0,0,0; " + notOwned + " stock), the same fuel");
    m.goTab(Menu::TabGarage);
    expect(!m.devTagShown(), "no DEV tag with dev mode off");
    {
        MenuRequest r;
        r.kind = MenuRequest::FreeRide, r.vehicle = own, r.hub = "YellowStone";
        const auto args = sessionArgs(dataDir, r, m);
        const auto it = std::find(args.begin(), args.end(), "--upgrades");
        expect(it != args.end() && it + 1 != args.end() && *(it + 1) == "0,0,1,0,0,0", "free ride with " + own + ": the real --upgrades 0,0,1,0,0,0");
    }
    m.saveCareer();
    expect(payload(fileBytes(careerFile)) == devStream, "saved with dev mode off: the same bytes (the dev choices kept for the next time)");
    // ---- back on: the choices made before come back
    c->setDevMode(true);
    expect(st(c->upgradesEquipped(notOwned)) == "3,0,0,0,0,2" && st(c->upgradesEquipped(own)) == "2,0,1,0,0,0" && c->vehicleColor(notOwned) == (realColour + 3) % 10,
           "dev mode on again: the stages and colour chosen in dev mode are back");
    c->setDevMode(false);
    // ---- the pause menu reads dev_mode from settings.txt (a session's own Menu over the same save)
    {
        std::ofstream(settingsFile, std::ios::app) << "dev_mode = on\n";
        Settings sp = loadSettings();
        sp.dataDir = dataDir;
        Menu pm(dataDir, sp, MenuMode::Pause);
        expect(pm.career() && pm.career()->devMode() && pm.career()->vehicleOwned(notOwned) && st(pm.career()->upgradesEquipped(notOwned)) == "3,0,0,0,0,2",
               "the pause menu's Menu (dev_mode = on in settings.txt): the overlay and the dev choices from career.sav");
    }
    unlink(settingsFile.c_str()), unlink(careerFile.c_str()), rmdir(dir);
    std::printf("dev mode check: %s (%d failures)\n", bad ? "FAILED" : "ok", bad);
    return bad;
}

}  // namespace fuel::ui
