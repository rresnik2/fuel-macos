// FUEL's in-race and free-roam HUD (decoded in notes/p40-hud.md; maths in hud_math.*):
//  - speedometer: MeterHUD (vtable 0x9a545c) on page compteur: dial COMPTEUR02..05 by the engine's top rpm
//    (0x4b0ba0), rev needle COMPTEUR_AIGUILLE (0x4b1260), damage needle DEGAT_AIGUILLE (0x4b1330), speed
//    (0x4b0b30, every frame) and odometer (0x4b0d30, every 16 frames) through the labels' own TT templates;
//  - race info: MissionHUD (0x9a54b4) pages MissionInfo* (0x4b4530), ranking lines (0x4b95b0), countdown
//    (0x4b3da0);
//  - minimap: MapHUD (0x9ab75c), heading-up radar in the RADAR_FOND rect of page boussole (0x4a1020): road
//    cache 0x49f9b0/0x4a0100, strips 0x4a5f20, race line 0x4a1bd0, gates 0x4aba90, opponents 0x4ab920, player
//    0x4ab8d0, hub-name fade 0x4a54b0.
// Port choices are marked "port".
#include "ui/menu.h"

#include "core/hub_zones.hpp"

#include "core/freeroam.hpp"
#include "core/road_graph.hpp"
#include "core/road_types.hpp"
#include "ui/ui_renderer.h"
#include "ui/nitro_gauge.h"  // [nitro]

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <strings.h>

namespace fuel::ui {
namespace {

std::string fmt(const std::string& tmpl, int a) {
    char b[64];
    std::snprintf(b, sizeof b, tmpl.empty() ? "%d" : tmpl.c_str(), a);
    return b;
}
std::string fmt2(const std::string& tmpl, int a, int b2) {
    char b[64];
    std::snprintf(b, sizeof b, tmpl.empty() ? "%02d/%02d" : tmpl.c_str(), a, b2);
    return b;
}
int rowNumber(const std::string& id) {
    return id.empty() || !std::isdigit(static_cast<unsigned char>(id.back())) ? 0 : id.back() - '0';
}
// MapHUD colours use 0.5 as white (the engine's 2x modulate, notes/p40-work/minimap.md 1): the port doubles them.
Rgba mapColour(float r, float g, float b, float a) { return Rgba{std::min(1.0f, 2 * r), std::min(1.0f, 2 * g), std::min(1.0f, 2 * b), a}; }

}  // namespace

void Menu::itemRect(const Item& it, float& x, float& y) {
    x = it.x, y = it.y;
    if (it.anchorH == 1) x += 64;
    if (it.anchorH == 2) x -= 64;
    if (it.anchorV == 1 || it.anchorV == 3) y += 36;
    if (it.anchorV == 2 || it.anchorV == 4) y -= 36;
}

void Menu::drawHud(UiRenderer& r, const HudState& hud, bool vignette) {
    hudRace_ = hud.race;  // P62: the tutorial gate's race test (0x56e620)
    playerX_ = hud.px, playerZ_ = hud.pz;  // P78
    applyItemStates();
    if (vignette) drawVignette(r);  // P56; P102: 0x443290 draws it first (IGP +0x4830), under the other HUDs [O]
    // P86: the GPS arrows are drawn by the renderer in the 3-D scene (tornado_render.mm encodeGpsRibbons).
    drawNavigator(r, hud);
    if (options_.gps) drawMinimap(r, hud);
    if (options_.speedMeter) drawMeter(r, hud);
    if (hud.race && options_.raceInfo) drawMissionInfo(r, hud);
    if (hud.race && options_.rankingInfo) drawRanking(r, hud);
    drawCountdown(r, hud);
    drawMessages(r);
    updateArch(hud);
    if (arch_ >= 0)  // 0x4b86a0: page 125 (PressStart) at alpha 1 while +0xd6c is 1 (or 0x1e online)
        if (Page* p = scripts_.page("PressStart")) drawPage(r, *p);
}

// ---- Challenge-arch start (P75) ------------------------------------------------------------------------
// 0x4f6e80: the arch radius of a mission = RoadGraph_TypeFieldAt(start, field 2) 0x6d1c10 (0x4f97f0: EDI = 2): the
// nearest road record within 20 m (0x6d19f0) at squared distance <= 4 -> its road type's +0xc4 (table 0xa678a4), else 0;
// values below 0.001 (0x9dc024) -> 10 m (0x996494).
float Menu::archRadius(float x, float z) const {
    float r = 0;
    if (roads_ && roadTypes_) {
        int rec = -1, k = 0;
        float qx = 0, qz = 0;
        const float d2 = roads_->nearestRoad(x, z, 20.0f, rec, k, qx, qz);
        if (d2 >= 0 && d2 <= 4.0f && rec >= 0) {
            const int t = int(roads_->rec(size_t(rec)).type);
            if (t >= 0 && t < int(roadTypes_->size())) r = (*roadTypes_)[size_t(t)].core;
        }
    }
    return r < 0.001f ? 10.0f : r;
}

// 0x4f6fd0 (free roam only: race state 0/1, game-mode object +0x5c == 0): the missions in the grid (0x4f89e0: EMD and
// world missions) within 100 m of the player (0x421f60) (d^2 < [0xa2a4e0] = 10000) and within the radius above; state 0 -> 1 when 0x4f6ed0 allows it: not in tutorial mode
// 1, game +0x170 clear, and (edited races) always / (career, challenges) the zone's hub opened (+0x248) [O]. Leaving the radius ends the prompt [I: the state is reset by 0x4819f0 /
// 0x62baa0 when the menus or a race take over].
void Menu::updateArch(const HudState& hud) {
    arch_ = -1;
    if (hud.race || !career_) return;
    float best = 1e30f;
    for (size_t i = 0; i < events_.size(); ++i) {
        const Event& e = events_[i];
        // P99: every mission of the grid, not only those with a visible start (0x4f70dd..0x4f712b test distance and radius
        // only; career races have EMD_HideStartEnd and the world's MISSION_COURSE ones +0 = 0, yet 0x4f89e0 puts both in
        // the grid) [O].
        if (!e.playable) continue;
        const float dx = e.race.start[0] - hud.px, dz = e.race.start[2] - hud.pz, d2 = dx * dx + dz * dz;
        if (d2 >= 10000.0f || d2 >= best) continue;
        const float rad = archRadius(e.race.start[0], e.race.start[2]);
        if (d2 >= rad * rad) continue;
        int hub = -1;
        const bool found = career_->locate(e.id, &hub, nullptr, nullptr);
        // P99: 0x4f6ed0 [O]: the hub of the zone holding the mission's start (0x6ee810 -> 0x577500) must be opened
        // (+0x248: entered from the map, 0x481cd0), not only unlocked; no zone there -> no prompt. [port] without the
        // zones (headless), the event's own hub.
        int zoneHub = hub;
        if (zones_) {
            const int z = zones_->find(e.race.start[0], e.race.start[2]);
            zoneHub = z >= 0 ? career_->rules().hubIndex(zones_->zones[size_t(z)].name) : -1;
        }
        const bool opened = zoneHub >= 0 && size_t(zoneHub) < career_->state().hubs.size() && career_->state().hubs[size_t(zoneHub)].opened;
        static const bool trace = std::getenv("FUEL_ARCH_TRACE") != nullptr;
        if (trace) std::printf("arch: %s d %.1f r %.1f located %d hub %d zone hub %d opened %d\n", e.id.c_str(), std::sqrt(d2), rad, found, hub, zoneHub, opened);
        if (!found || !opened) continue;
        best = d2, arch_ = int(i);
    }
}

// 0x4f6fd0 state 1 -> 3 on the action key (input +0x35), then 0x481860: the career (list kind 1, state 0x56) or
// challenge (kind 2, 0x59) race page of the menu; edited races (kind 3) start at once (Race_Start). The port opens its
// race choice: the event's vehicle carousel in the Map tab (as the race lists do).
bool Menu::archAccept() {
    if (arch_ < 0 || size_t(arch_) >= events_.size()) return false;
    const Event& e = events_[size_t(arch_)];
    setMode(MenuMode::Pause);
    goTab(TabMap);
    for (size_t i = 0; i < hubs_.size(); ++i)
        if (strcasecmp(hubs_[i].id.c_str(), e.hub.c_str()) == 0) hubIndex_ = int(i);
    pendingEvent_ = e.id;
    openRace(e, true);  // PK02: 0x481860 -> the race page 0x56 / 0x59 (Back closes the menu, 0x4819f0); P99's own-vehicle start is its Accept
    arch_ = -1;
    return true;
}

// PK08: the page of a message kind with its texts / images filled (0x4afbe0, used by drawMessages and --career-audit).
Page* Menu::messagePage(int kind, int param) {
    static const char* const kPage[13] = {nullptr,
                                          "NewChallengeDetected",
                                          "NewLiveryDetected",
                                          "NewVistaPointDetected",
                                          "NewLiveryDiscovered",
                                          "NewLiveryDiscovered",
                                          "NewVistaPointDiscovered",
                                          "NewVistaPointDiscovered",
                                          "BANDEAU_DOPPLER_BONUS_CAR",
                                          "BANDEAU_DOPPLER_LIVERY",
                                          "BANDEAU_DOPPLER_VISTAPT",
                                          "BANDEAU_DOPPLER_CHALLENGE",
                                          "infofuel"};
    if (kind < 1 || kind > 12) return nullptr;
    Page* p = scripts_.page(kPage[kind]);
    if (!p) return nullptr;
    if ((kind == 4 || kind == 5) && param >= 0)
        if (Item* it = p->find("ITEM_ID_POI_TT")) it->hasDynText = true, it->dynText = textOf(uint32_t(param));
    if (kind == 8 && param >= 0) {  // 0x4afbe0 runs the setter only with a param
        // PK08 0x4afc70: ITEM_ID_BONUS_CAR_PICTURE (its three image states) = the picture of the vehicle record (+0x4c, the garage
        // carousel's picture; none when the record is null) and ITEM_ID_POI_TT = the vehicle's name text (+0x6c).
        const auto* rv = career_ && size_t(param) < career_->rules().vehicles.size() ? &career_->rules().vehicles[size_t(param)] : nullptr;
        const VehicleInfo* vi = rv ? vehicleInfo(rv->label) : nullptr;
        if (Item* it = p->find("ITEM_ID_BONUS_CAR_PICTURE")) it->imgNormal = it->imgEnabled = it->imgFocus = vi ? vi->picture : std::string();  // (the item is authored hidden: only the image changes)
        if (Item* it = p->find("ITEM_ID_POI_TT")) it->hasDynText = true, it->dynText = rv ? textOf(rv->nameText) : std::string();
    }
    if (kind == 12) {
        if (Item* it = p->find("ITEM_ID_FUEL_WON")) it->hasDynText = true, it->dynText = "+" + std::to_string(param);
        if (Item* it = p->find("ITEM_ID_FUEL_TOTAL")) it->hasDynText = true, it->dynText = career_ ? std::to_string(career_->state().fuel) : "0";
    }
    return p;
}

// ---- MessagesHUD (P44) -------------------------------------------------------------------------------
// 0x4afe00: the page of the shown message's kind (InterfacesInfosPacked order 126..135) with the fade alpha;
// 0x4afbe0 texts: kinds 4/5 ITEM_ID_POI_TT = text(param) (0x677d20), kind 8 (0x4afc70) the bonus car's picture and name,
// kind 12 (0x4afd80) ITEM_ID_FUEL_WON = "+%d" of param and ITEM_ID_FUEL_TOTAL = the player's fuel.
bool Menu::bannerAccept() { return freeRoam_ && freeRoam_->messages().skip(); }  // PK08 0x4af920

void Menu::drawMessages(UiRenderer& r) {
    int kind = 0, param = -1;
    float a = 0;
    if (shotBanner_.kind > 0) kind = shotBanner_.kind, param = shotBanner_.param, a = 1.0f;  // PK08: --menu-shot BANNER:<kind>[:<param>]
    else if (freeRoam_ && freeRoam_->messages().showing()) kind = freeRoam_->messages().current().kind, param = freeRoam_->messages().current().param, a = freeRoam_->messages().alpha();
    else return;
    Page* p = messagePage(kind, param);
    if (!p) return;
    for (const Item& src : p->items) {
        Item it = src;
        it.cNormal.a *= a, it.cEnabled.a *= a, it.cFocus.a *= a;
        it.tNormal.a *= a, it.tEnabled.a *= a, it.tFocus.a *= a;
        drawItem(r, it, 0, 0);
    }
}

// economy_hud (0x46d1e0): stars won / the most there are (0x578030 / 0x578070), and the fuel.
void Menu::fillEconomy() {
    Page* p = scripts_.page("economy_hud");
    if (!p || !career_) return;
    if (Item* it = p->find("ITEM_ID_ECO_STARS")) {
        char b[64];
        const std::string t = textOf(it->tt);
        std::snprintf(b, sizeof b, t.find('%') != std::string::npos ? t.c_str() : "%d/%d", career_->stars(), career_->starsMax());
        it->hasDynText = true, it->dynText = b;
    }
    if (Item* it = p->find("ITEM_ID_ECO_FUEL")) it->hasDynText = true, it->dynText = std::to_string(career_->state().fuel);
}

// ---- Speedometer ------------------------------------------------------------------------------------
void Menu::drawMeter(UiRenderer& r, const HudState& hud) {
    Page* p = scripts_.page("compteur");
    if (!p) return;
    HudMemory& m = hudMem_;
    const bool miles = options_.miles;
    // 0x4b08e0: odometer and damage every 16th frame, speed every frame.
    const bool refresh = (m.meterFrame & 15) == 0;
    ++m.meterFrame;
    if (refresh) m.damage = hud.damage;
    int km = 0, tenth = 0;
    hud::odometer(double(hud.odometerKm) * 1000.0, miles, km, tenth);
    const int dial = hud::meterDial(hud.maxRpm);
    const Item* dialItem = nullptr;
    for (Item& it : p->items) {
        if (it.id.rfind("ITEM_ID_COMPTEUR", 0) == 0 && it.id.size() == 18) {
            it.visible = std::atoi(it.id.c_str() + 16) == dial;  // 0x4b0ba0: only the chosen dial (+7)
            if (it.visible) dialItem = &it;
        }
        if (it.id == "ITEM_ID_SPEED") it.hasDynText = true, it.dynText = fmt(textOf(it.tt), hud::speedReadout(hud.speedMs, miles));
        if (refresh && it.id == "ITEM_ID_METER_NBKM") it.hasDynText = true, it.dynText = fmt(textOf(it.tt), km);
        if (refresh && it.id == "ITEM_ID_METER_NB100M") it.hasDynText = true, it.dynText = fmt(textOf(it.tt), tenth);
        if (it.id == "ITEM_ID_SPEED_UNITS") it.entryIndex = miles ? 1 : 0;  // 0x4b0810
    }
    // Dial pictures first, needles over them, labels on top (0x4b0940 draws the needles after the page).
    for (const Item& it : p->items)
        if (it.kind == ItemKind::Image) drawItem(r, it, 0, 0);
    if (dialItem) {
        float x, y;
        itemRect(*dialItem, x, y);
        if (BitmapRef dn = assets_->image("DEGAT_AIGUILLE")) {
            const hud::Needle n = hud::damageNeedle(x, y, dialItem->w, dialItem->h, float(dn.width));
            r.sprite(dn, n.px, n.py, n.size, n.size, 0.5f, 1.0f, hud::damageNeedleDeg(m.damage), Rgba{1, 1, 1, 1});
        }
        if (BitmapRef rn = assets_->image("COMPTEUR_AIGUILLE")) {
            const hud::Needle n = hud::revNeedle(x, y, dialItem->w, dialItem->h, float(rn.width));
            r.sprite(rn, n.px, n.py, n.size, n.size, 0.5f, 1.0f, hud::revNeedleDeg(hud.rpm, hud.maxRpm), Rgba{1, 1, 1, 1});
        }
        drawNitroGauge(r, hud, x, y, dialItem->w, dialItem->h);  // [nitro] the port's gauge right of the dial
    }
    for (const Item& it : p->items)
        if (it.kind != ItemKind::Image) drawItem(r, it, 0, 0);
}

// ---- Race info -------------------------------------------------------------------------------------
void Menu::drawMissionInfo(UiRenderer& r, const HudState& hud) {
    static const char* kPages[] = {"MissionInfoAtoBRace", "MissionInfoCheckpointRace", "MissionInfoCircuitRace", "MissionInfoKnockOutRace",
                                   "MissionInfoTimeTrialRace", "MissionInfoTimeTrialCheckpointRace", "MissionInfoCatchIARace", "MissionInfoHelicoRace"};
    using P = hud::InfoPage;
    if (hud.page == P::None) return;
    Page* p = scripts_.page(kPages[int(hud.page) - int(P::AtoB)]);
    if (!p) return;
    const hud::Clock c = hud::splitTime(std::max(0.0f, hud.time));
    for (Item& it : p->items) {
        const std::string t = textOf(it.tt);
        if (it.id == "ITEM_ID_TOP_INFO") {
            it.hasDynText = true;
            if (hud.page == P::AtoB || hud.page == P::TimeTrial || hud.page == P::Helico)
                it.dynText = hud::distanceText(hud.distance, options_.miles);
            else
                it.dynText = fmt2(t, hud.topA, hud.topB);
        }
        if (it.id == "ITEM_ID_BOTTOM_INFO") {
            it.hasDynText = true;
            if (hud.page == P::KnockOut) {
                it.dynText = fmt2(t, hud.bottomA, hud.bottomB);
            } else {
                char b[64];
                std::snprintf(b, sizeof b, t.empty() ? "%02d:%02d:%02d:%02d" : t.c_str(), c.h, c.m, c.s, c.cs);
                it.dynText = b;
            }
            it.enabled = hud::timeWarning(hud.page, hud.time, hud.timeTotal);  // red text colour (0x4b4e00)
        }
        if (it.id == "ITEM_ID_TOP_TITLE" && it.kind == ItemKind::LabelDyn) it.hasDynText = true, it.dynText = textOf(it.tt);
        if (it.id == "ITEM_ID_BOTTOM_TITLE" && it.kind == ItemKind::LabelDyn) it.hasDynText = true, it.dynText = textOf(it.tt);
    }
    drawPage(r, *p);
}

void Menu::drawRanking(UiRenderer& r, const HudState& hud) {
    Page* p = scripts_.page("classement");
    if (!p || hud.standings.size() < 2) return;
    int pos = 1;
    for (size_t i = 0; i < hud.standings.size(); ++i)
        if (hud.standings[i].player) pos = int(i) + 1;
    int playerLine = 0;
    const auto lines = hud::rankingLines(pos, int(hud.standings.size()), playerLine);
    const float mine = hud.standings[size_t(pos - 1)].time;
    for (Item& it : p->items) {
        const int row = rowNumber(it.id);
        if (row < 1 || row > 4) continue;
        const int shown = lines[size_t(row - 1)];
        it.visible = shown > 0;
        if (!it.visible) continue;
        const ResultRow& rr = hud.standings[size_t(shown - 1)];
        if (it.id.find("NUMBER") != std::string::npos) it.hasDynText = true, it.dynText = fmt(textOf(it.tt), shown);
        if (it.id.find("NAME") != std::string::npos) it.hasDynText = true, it.dynText = rr.player ? playerName() : opponentName(rr);  // PK08 0x5c0d70 / PK16 0x620c20
        if (it.id.find("DIST") != std::string::npos) {
            char b[32];
            const int d = int(std::fabs(rr.time - mine));  // remaining metres (racer +0x170)
            if (shown == pos) std::snprintf(b, sizeof b, "%d m", int(mine));
            else std::snprintf(b, sizeof b, shown < pos ? "- %d m" : "+ %d m", d);
            it.hasDynText = true, it.dynText = b;
        }
        it.enabled = shown == pos;  // RANKING_PLAYER bar
    }
    drawPage(r, *p);
}

// ---- Countdown -------------------------------------------------------------------------------------
void Menu::drawCountdown(UiRenderer& r, const HudState& hud) {
    const int n = hud::countdown(hud.countdown);
    if (n < 0) return;
    const std::string s = n == 0 ? textOf(5907) : std::to_string(n);
    TextStyle st;
    st.font = 0;          // 0x6978d0(0, ...)
    st.height = 80;       // 80 x HUD scale
    const float w = r.textWidth(s, st);
    const float x = (1280 - w) * 0.5f, y = (720 - st.height) * 0.5f;
    st.fill = Rgba{0, 0, 0, 1};  // shadow first, 10 % of the size down-right
    r.text(s, x + 0.1f * st.height, y + 0.1f * st.height, 0, st);
    st.fill = Rgba{1, 1, 1, 1};  // (0.5, 0.5, 0.5) in FUEL's 2x modulate
    r.text(s, x, y, 0, st);
}

// ---- Minimap ---------------------------------------------------------------------------------------
void Menu::drawMinimap(UiRenderer& r, const HudState& hud) {
    Page* p = scripts_.page("boussole");
    if (!p) return;
    HudMemory& m = hudMem_;
    // GROUP_FADE (0x4a54b0): the hub name and the status line show for 3 s, fade over the 4th; restart when either changes.
    // 0x4a53e0 gate (minimap-fix; PK03 read IGP +0x908 as "a mission runs"): the object is LevelManipulator_G's teleport
    // machine (Teleport_Queue 0x57e650 sets its +0x5c = 1), so the banner runs in races too, but not while a teleport is
    // under way, before GAMEStarted (game +0xdc), or under the level's LOADING RTC (HandleMgr +0x5b9, 0x563a70 / 0x563920;
    // the port's sessions have none): then the name, the status and the group's alpha keep their last values.
    // Status [O asm 0x4a5680..0x4a5779]: hud::bannerStatusStep (In career race / In challenge / Trying Edited Race while the
    // mission is PREPARING or RUNNING, by its hub list; In offline free ride 5 s after it); the port used to leave the
    // list box on the script's start index 5 (TT 5996 "Trying Edited Race") in every race.
    // A mission's state goes back to 0 only in 0x62baa0 (the kill: Mission::SetState(0)), which also resets the status.
    if (m.missionState != 0 && hud.missionState == 0) hud::bannerKilled(m.status);
    m.missionState = hud.missionState;
    const bool banner = !hud.teleporting && hud.gameStarted;
    if (banner) {
        bool restart = hud::bannerStatusStep(m.status, hud.missionState, hud.missionList, hud.dt);
        if (hud.hubName != m.fadeName) m.fadeName = hud.hubName, restart = true;  // +0x1adc (the hub's uid)
        if (hud.outOfArea) restart = true;  // 0x4a561a: uid -1 sets bVar1 every frame ("OUT OF AREA" stays up)
        if (restart) m.fadeT = 0;
        if (m.fadeT <= 4.0f) m.fadeT += hud.dt;
    }
    const float fade = m.fadeT < 3.0f ? 1.0f : std::max(0.0f, 4.0f - m.fadeT);
    const Item* fond = p->find("ITEM_ID_RADAR_FOND");
    for (Item& it : p->items) {
        if (it.id == "ITEM_ID_HUB_NAME") it.hasDynText = true, it.dynText = m.fadeName;
        if (it.id == "ITEM_ID_STATUS" && banner) it.entryIndex = m.status.index;  // 0x4a574f: list box +0x128 = +0x1ae4
    }
    for (const Item& src : p->items) {
        if (src.group != "GROUP_FADE") {
            if (&src == fond) drawItem(r, src, 0, 0);
            continue;
        }
    }
    if (fond) {
        float x, y;
        itemRect(*fond, x, y);
        const float w = fond->w, h = fond->h;
        // Zoom (0x49f690 target, 0x49f9b0 smoothing once per update).
        if (m.pp.empty()) m.pp = hud::loadPp(dataDir_ + "/CommonComponent/Param/pp001.tsc");
        auto PP = [&](int n, float fallback) { return size_t(n) < m.pp.size() && m.pp[size_t(n)] != 0 ? m.pp[size_t(n)] : fallback; };
        const float target = hud::zoomTarget(hud.speedMs, PP(22, 1000), PP(23, 4000));  // PP22 / PP23
        if (hud.settle) m.zoom = m.zoomTarget = target;
        m.zoom = hud::zoomStep(m.zoom, m.zoomTarget);
        m.zoomTarget = target;
        hud::mapHeading(hud.quat, m.hx, m.hz);
        const hud::MapView v = hud::mapView(x, y, w, h, m.zoom, hud.px, hud.pz, m.hx, m.hz);
        // Clip (V+0x24.., 0x4a12d1): m = 0.012 x viewport width, asymmetric as FUEL has it.
        const float mg = 0.012f * 1280.0f;
        // GPS_MASK (0x4a1ae0) covers R at depth 0.001 before the roads (z >= 0.05): its picture is opaque outside
        // the disc inscribed in R, so only the disc keeps roads and icons [inferred: depth mask]. Port: a disc clip.
        r.setClip(x + mg, y + mg, w - 3 * mg, h - 3 * mg, v.cx, v.cy, w * 0.5f);
        const float S = 1.0f;  // canvas units already (MapHUD +0x24 = screen width / 1280)
        // Road cache (0x49f9b0 / 0x4a0100): records in P +- 0.707 r with class > LOD, simplified by tol.
        if (roads_ && roadTypes_) {
            const float tol = hud::mapTolerance(m.zoom);
            const int lod = hud::mapLod(m.zoom);
            const float half = 0.707f * m.zoom;
            const bool moved = std::fabs(hud.px - m.cacheX) > 50.0f || std::fabs(hud.pz - m.cacheZ) > 50.0f;  // port: rebuild step
            if (tol != m.tol || lod != m.lod || moved) {
                m.tol = tol, m.lod = lod, m.cacheX = hud.px, m.cacheZ = hud.pz, m.cacheHalf = half;
                m.cache.clear();
                const float qx0 = (hud.px - half - 50) * 0.5f, qx1 = (hud.px + half + 50) * 0.5f;
                const float qz0 = (hud.pz - half - 50) * 0.5f, qz1 = (hud.pz + half + 50) * 0.5f;
                for (size_t i = 0; i < roads_->size(); ++i) {
                    const RoadGraph::Rec& rc = roads_->rec(i);
                    if (rc.maxX < qx0 || rc.minX > qx1 || rc.maxZ < qz0 || rc.minZ > qz1 || rc.type >= 21) continue;
                    if ((*roadTypes_)[rc.type].mapClass <= lod) continue;
                    std::vector<std::array<float, 2>> pts;
                    for (uint32_t k = 0; k < rc.n; ++k) pts.push_back(roads_->point(uint32_t(i), k));
                    m.cache.emplace_back(int(i), hud::simplify(pts, tol));
                }
            }
            // Three passes (0x4a3750): rivers (+0x1d8 == 14), off-road (+0xb4 > 0), tarmac (+0xb4 == 0).
            static const char* kTex[3] = {"GPS_ECRAN_RIVER", "GPS_ECRAN_OFFROAD", "GPS_ECRAN_ROUTE"};
            for (int pass = 0; pass < 3; ++pass) {
                const BitmapRef tex = assets_->image(kTex[pass]);
                std::vector<UiVertex> tris;
                for (const auto& [rec, pts] : m.cache) {
                    const int type = roads_->rec(size_t(rec)).type;
                    const RoadTypeParams& T = (*roadTypes_)[size_t(type)];
                    const int which = T.kind == 14 ? 0 : T.aiSpeedBlend > 0 ? 1 : 2;
                    if (which != pass || pts.size() < 2) continue;
                    const float alpha = hud::mapFade(T.mapClass, m.zoom) * (type <= 3 ? 0.4f : 1.0f);
                    const Rgba col = type <= 3 ? mapColour(0.1f, 0.1f, 0.1f, alpha) : mapColour(0.5f, 0.5f, 0.5f, alpha);
                    const float hw = hud::roadHalfWidth(type, v.ppm, S);
                    std::array<float, 2> pl{}, pr{};
                    for (size_t k = 0; k < pts.size(); ++k) {
                        // 0x4a0480 tangent (central difference), rotated into the view.
                        const auto& a = pts[k > 0 ? k - 1 : 0];
                        const auto& b = pts[k + 1 < pts.size() ? k + 1 : k];
                        auto t = v.dirToScreen(b[0] - a[0], b[1] - a[1]);
                        const float tl = std::hypot(t[0], t[1]);
                        if (tl > 0) t[0] /= tl, t[1] /= tl;
                        const auto s = v.toScreen(pts[k][0], pts[k][1]);
                        const std::array<float, 2> L{s[0] + t[1] * hw, s[1] - t[0] * hw}, R{s[0] - t[1] * hw, s[1] + t[0] * hw};
                        if (k > 0) {
                            const UiVertex a0{pl[0], pl[1], 0.5f, 0, col}, b0{pr[0], pr[1], 0.5f, 1, col}, a1{L[0], L[1], 0.5f, 0, col},
                                b1{R[0], R[1], 0.5f, 1, col};
                            for (const UiVertex& q : {a0, b0, a1, b0, b1, a1}) tris.push_back(q);
                        }
                        pl = L, pr = R;
                    }
                }
                r.triangles(tex ? &tex : nullptr, tris);
            }
        }
        // Race line (0x4a1bd0): GPS style 4 = PP920..924 (0.15, 0.15, 0.25, 0.75), half-width 3.6 x 0.8 (port: width units).
        if (hud.raceLine.size() >= 2) {
            const BitmapRef tex = assets_->image("GPS_ECRAN_ROUTE");
            const Rgba col{PP(920, 0.15f), PP(921, 0.15f), PP(922, 0.25f), PP(923, 0.75f)};
            const float hw = 0.8f * PP(924, 3.6f);
            std::vector<UiVertex> tris;
            const auto pts = hud::simplify(hud.raceLine, 4.0f);  // quantised to the 2 m pool
            std::array<float, 2> pl{}, pr{};
            for (size_t k = 0; k < pts.size(); ++k) {
                const auto& a = pts[k > 0 ? k - 1 : 0];
                const auto& b = pts[k + 1 < pts.size() ? k + 1 : k];
                auto t = v.dirToScreen(b[0] - a[0], b[1] - a[1]);
                const float tl = std::hypot(t[0], t[1]);
                if (tl > 0) t[0] /= tl, t[1] /= tl;
                const auto s = v.toScreen(pts[k][0], pts[k][1]);
                const std::array<float, 2> L{s[0] + t[1] * hw, s[1] - t[0] * hw}, R{s[0] - t[1] * hw, s[1] + t[0] * hw};
                if (k > 0) {
                    const UiVertex a0{pl[0], pl[1], 0.5f, 0, col}, b0{pr[0], pr[1], 0.5f, 1, col}, a1{L[0], L[1], 0.5f, 0, col}, b1{R[0], R[1], 0.5f, 1, col};
                    for (const UiVertex& q : {a0, b0, a1, b0, b1, a1}) tris.push_back(q);
                }
                pl = L, pr = R;
            }
            r.triangles(tex ? &tex : nullptr, tris);
        }
        // P56: free-roam markers. Alpha M+0x1af4 (0x49f6d0): + 2 dt up to 1 while no race runs, - 2 dt down to 0 in a race.
        m.markerAlpha = std::clamp(m.markerAlpha + (hud.race ? -2.0f : 2.0f) * hud.dt, 0.0f, 1.0f);
        if (hud.settle) m.markerAlpha = hud.race ? 0.0f : 1.0f;
        const int H = career_ ? career_->currentHub() : -1;
        const bool markers = !hud.race && career_ && H >= 0 && size_t(H) < career_->rules().hubs.size() && m.markerAlpha >= 1e-6f;
        auto marker = [&](const char* tex, float wx, float wz, float size) {  // 0x60af00 after the 0x4ad5e0 clip test
            const BitmapRef b = assets_->image(tex);
            if (!b) return;
            const auto s = v.toScreen(wx, wz);
            if (s[0] + size * 0.5f < x + mg || s[0] - size * 0.5f > x + w - 2 * mg || s[1] + size * 0.5f < y + mg || s[1] - size * 0.5f > y + h - 2 * mg)
                return;
            r.image(b, s[0] - size * 0.5f, s[1] - size * 0.5f, size, size, Rgba{1, 1, 1, m.markerAlpha});
        };
        auto startOf = [&](int mission, float& sx, float& sz) {
            const std::string& n = career_->rules().missions[size_t(mission)].name;
            for (const Event& e : events_)
                if (strcasecmp(e.id.c_str(), n.c_str()) == 0) return sx = e.race.start[0], sz = e.race.start[2], true;
            return false;
        };
        if (markers) {
            // 0x4aae80 / 0x4ab050 (filter bits 8 / 4 of M+0x1b7c = 0x1f): the current hub's available career races
            // (MAP_CAREER_* +0xce8 + 4 x best rank +0x120) and challenges (MAP_CHALLENGE_* +0xd04 + 4 x +0x120), size M+0x1a2c.
            static const char* kCareer[4] = {"MAP_CAREER_GOLD_ICON", "MAP_CAREER_SILVER_ICON", "MAP_CAREER_BRONZE_ICON", "MAP_CAREER_NONE_ICON"};
            static const char* kChal[4] = {"MAP_CHALLENGE_GOLD_ICON", "MAP_CHALLENGE_SILVER_ICON", "MAP_CHALLENGE_BRONZE_ICON", "MAP_CHALLENGE_NONE_ICON"};
            const auto& hub = career_->rules().hubs[size_t(H)];
            const auto& S = career_->state();
            float ex, ez;
            for (size_t k = 0; k < hub.career.size(); ++k)
                if (S.career[size_t(H)][k].available && startOf(hub.career[k], ex, ez))
                    marker(kCareer[std::clamp(S.career[size_t(H)][k].bestRank, 0, 3)], ex, ez, 30);
            for (size_t k = 0; k < hub.challenges.size(); ++k)
                if (S.challenges[size_t(H)][k].available && startOf(hub.challenges[k], ex, ez))
                    marker(kChal[std::clamp(S.challenges[size_t(H)][k].bestRank, 0, 3)], ex, ez, 30);
        }
        // Gates (0x4aba90): the GPS_ONDE pulse on the next gate, the remaining gates and the finish at size 14.
        m.pulse = hud::pulseStep(m.pulse, hud.dt);
        if (!hud.gates.empty() && hud.nextGate < hud.gates.size()) {
            const BitmapRef cp = assets_->image("MAP_CHECKPOINT_ICON"), fin = assets_->image("MAP_FINISH_ICON"), onde = assets_->image("GPS_ONDE");
            const float c2 = std::cos(2 * m.pulse);
            const auto ns = v.toScreen(hud.gates[hud.nextGate][0], hud.gates[hud.nextGate][1]);
            const float ps = 14.0f * (1 + 2 * (1 - c2));
            if (onde) r.sprite(onde, ns[0], ns[1], ps, ps, 0.5f, 0.5f, 0, mapColour(0.5f, 0.305f, 0.03f, std::max(c2, 0.0f)));
            for (size_t g = hud.nextGate; g < hud.gates.size(); ++g) {
                const bool last = g + 1 == hud.gates.size();
                const BitmapRef& b = last ? fin : cp;
                const auto s = v.toScreen(hud.gates[g][0], hud.gates[g][1]);
                if (b) r.sprite(b, s[0], s[1], 14, 14, 0.5f, 0.5f, 0, Rgba{1, 1, 1, 1});
            }
        }
        if (markers) {
            // 0x4ab450 (bits 1 / 2): the current hub's POIs (+0x3c == hub uid): liveries revealed and not owned
            // (MAP_LIVERY_LITTLE_ICON +0xd3c), vista points revealed and not visited (MAP_VISTAPOINT_LITTLE_ICON +0xd44),
            // size M+0x1a28 = 14.
            const auto& hub = career_->rules().hubs[size_t(H)];
            const auto& S = career_->state();
            for (size_t k = 0; k < hub.liveries.size(); ++k)
                if (hub.liveries[k].kind == "POI" && S.liveries[size_t(H)][k].a && !S.liveries[size_t(H)][k].b)
                    marker("MAP_LIVERY_LITTLE_ICON", hub.liveries[k].x, hub.liveries[k].z, 14);
            for (size_t k = 0; k < hub.vistaPoints.size(); ++k)
                if (S.vistas[size_t(H)][k].a && !S.vistas[size_t(H)][k].b) marker("MAP_VISTAPOINT_LITTLE_ICON", hub.vistaPoints[k].x, hub.vistaPoints[k].z, 14);
        }
        // PK19 0x4ab220 [O] (after 0x4ab450, before 0x4ac1a0): MAP_EVENT_ICON (IGP +0xce4, slot 93) at every hub's CAMERA_CAREER
        // x, z (hub list 0xa66860, record +0x1f0 / +0x1f8 = the port's camp), all hubs (no unlock test), size M+0x1a30 (35),
        // colour (0.5, 0.5, 0.5, M+0x1af4) = white x the marker alpha (skipped below 1e-6, so it fades out in races), the
        // size/2 box culled against the clip rect V+0x24..+0x30, upright (0x60af00, depth 0.002).
        if (career_ && m.markerAlpha >= 1e-6f)
            if (const BitmapRef ev = assets_->image("MAP_EVENT_ICON"))
                for (const auto& hr : career_->rules().hubs)
                    if (const Hub* hb = findHub(hubs_, hr.name)) {
                        const auto s = v.toScreen(float(hb->camp.x), float(hb->camp.z));
                        const float hs = 35.0f * 0.5f;
                        if (s[0] + hs < x + mg || s[0] - hs > x + w - 2 * mg || s[1] + hs < y + mg || s[1] - hs > y + h - 2 * mg) continue;
                        r.image(ev, s[0] - hs, s[1] - hs, 35.0f, 35.0f, Rgba{1, 1, 1, m.markerAlpha});
                    }
        // PK19 end
        // P78: 0x4ac1a0 (after 0x4ab220, before the opponents) [O draw order, icons]: in free ride, the special traffic of
        // the hub whose zone holds the player (0x4a7f40 -> 0x577500 uid), from the traffic list 0x5c57d0; an entry revealed
        // (+0) and not caught (+1) [I: the record flags] gets its kind's icon (IGP +0xd2c.., the name table at 0xa055cc:
        // kind 3 MAP_BONUS_CAR_ICON, 1 MAP_DOPPLER_LIVERY_ICON, 0 MAP_DOPPLER_VISTAPOINT_ICON, 2 MAP_DOPPLER_RACE_ICON)
        // by 0x4ab720: size M+0x1a2c (30), colour 0.5 (white, opaque), clip test 0x4ad5e0 [port: upright].
        if (!hud.race && career_ && zones_ && !hud.specials.empty()) {
            const int z = zones_->find(hud.px, hud.pz);
            int zh = -1;
            if (z >= 0)
                for (size_t i = 0; i < career_->rules().hubs.size(); ++i)
                    if (strcasecmp(career_->rules().hubs[i].name.c_str(), zones_->zones[size_t(z)].name.c_str()) == 0) zh = int(i);
            static const char* kIcon[4] = {"MAP_DOPPLER_VISTAPOINT_ICON", "MAP_DOPPLER_LIVERY_ICON", "MAP_DOPPLER_RACE_ICON", "MAP_BONUS_CAR_ICON"};
            const auto& S = career_->state();
            for (const auto& sp : hud.specials) {
                // PK19 0x4a1770 passes 1 to 0x4ac1a0 -> 0x5c5690 [O]: only the traffic manager's active slots (u16 list +0x1dc,
                // count +0x1da) whose entry +0x14 < 1e6, at the slot's vehicle (veh +0x4a8 / +0x4b0; the slot's +0x48 / +0x50
                // while it has none); the map page's 0x5c57d0 (every entry) is the one that lists them all.
                if (!sp.live) continue;
                if (sp.hub != zh || size_t(sp.hub) >= S.traffic.size() || size_t(sp.index) >= S.traffic[size_t(sp.hub)].size()) continue;
                const auto& f = S.traffic[size_t(sp.hub)][size_t(sp.index)];
                if (!f.a || f.b) continue;
                const BitmapRef b = assets_->image(kIcon[std::clamp(sp.kind, 0, 3)]);
                if (!b) continue;
                const auto s = v.toScreen(sp.x, sp.z);
                const float size = 30, hs = size * 0.5f;
                if (s[0] + hs < x + mg || s[0] - hs > x + w - 2 * mg || s[1] + hs < y + mg || s[1] - hs > y + h - 2 * mg) continue;
                r.image(b, s[0] - hs, s[1] - hs, size, size, Rgba{1, 1, 1, 1});
            }
        }
        // Opponents (0x4ab920, MAP_CONCURRENT_CAR_ICON) and the player (0x4ab8d0, MAP_PLAYER_CAR_ICON), size 25 x 1.2,
        // pointing along their forward on screen (sprite up = s).
        auto icon = [&](const BitmapRef& b, float wx, float wz, float fx, float fz) {
            if (!b) return;
            const auto s = v.toScreen(wx, wz);
            const auto d = v.dirToScreen(fx, fz);
            const float deg = std::atan2(d[0], -d[1]) * 57.29578f;
            r.sprite(b, s[0], s[1], 30, 30, 0.5f, 0.5f, deg, Rgba{1, 1, 1, 1});
        };
        const BitmapRef opp = assets_->image("MAP_CONCURRENT_CAR_ICON");
        for (const auto& o : hud.opponents) icon(opp, o.x, o.z, o.fx, o.fz);
        icon(assets_->image("MAP_PLAYER_CAR_ICON"), hud.px, hud.pz, m.hz, -m.hx);  // forward: h = (-fz, fx)
        // PK19 0x4aad40 [O] (after the player): the GPS target pin when MapHUD +0x60 is set (the map's target, +0x64 / +0x6c;
        // MapHUD vf21 0x4ac890 "zoomed out" is always 0 on the minimap): MAP_TARGET_Y (IGP +0xde8) of size M+0x1a2c (30),
        // culled at the target by 0x4ad5e0(size / 2), drawn raised by 0.6 x size / 2, white, upright, depth 0.002.
        if (map_.target)
            if (const BitmapRef pin = assets_->image("MAP_TARGET_Y")) {
                const auto s = v.toScreen(map_.tx, map_.tz);
                const float hs = 30.0f * 0.5f;
                if (!(s[0] + hs < x + mg || s[0] - hs > x + w - 2 * mg || s[1] + hs < y + mg || s[1] - hs > y + h - 2 * mg))
                    r.image(pin, s[0] - hs, s[1] - 0.6f * hs - hs, 30.0f, 30.0f, Rgba{1, 1, 1, 1});
            }
        // PK19 end
        r.clearClip();
    }
    // GROUP_FADE items (RADAR_NAME, hub name, status) over the radar.
    for (const Item& src : p->items) {
        if (src.group != "GROUP_FADE") continue;
        Item it = src;
        it.cNormal.a *= fade, it.cEnabled.a *= fade, it.cFocus.a *= fade;
        it.tNormal.a *= fade, it.tEnabled.a *= fade, it.tFocus.a *= fade;
        drawItem(r, it, 0, 0);
    }
}

}  // namespace fuel::ui
