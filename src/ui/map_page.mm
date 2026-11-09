// FUEL's full-screen map page (the Map tab's map, a MapHUD in map mode; decoded by P44 in
// ~/re-work/fuel/notes/p44-freeroam.md F10):
//  - items 0x4a9f50: every hub (0x4aa4d0), the current hub's available career races (0x4aa650) and challenges
//    (0x4aa7c0), its liveries, vistas and heliports (0x4aa930), icons by type (0x4a9c80);
//  - selection 0x46a4d0: the item nearest the cursor within zoom x 30 / PP340 metres, drawn larger (0x46a880);
//  - view 0x4ac8a0: Map_Cache ITEM_ID_CLIPPING_RECT, centred on the cursor, scale |diag| / 2 zoom, oriented as the
//    minimap was when the map opened;
//  - info page by selection (0x483270): Map_MetaInfos, Map_locked_hub / Map_unlocked_hub, Map_selection_*;
//  - keys (0x4827b0): action 0x482b60 (hub travel, teleport to an owned livery / visited vista, heliport flight),
//    GPS target 0x484580 (toggle at the cursor; the free-roam GPS then guides there).
// Port choices are marked "port".
#include "ui/menu.h"

#include "core/freeroam.hpp"
#include "core/hub_zones.hpp"
#include "core/road_graph.hpp"
#include "core/road_types.hpp"
#include "ui/ui_renderer.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <strings.h>

namespace fuel::ui {
namespace {

// 0x4a9c80: icon texture per item type (InGamePlayer textures +0xb70 + 4i, names 0xa05410).
const char* mapIcon(int type) {
    static const char* const k[22] = {"GPS_ONDE", "MAP_CAREER_NONE_ICON", "MAP_CHALLENGE_NONE_ICON", "MAP_CHECKPOINT_ICON", "MAP_FINISH_ICON",
                                      "MAP_TARGET_Y", "MAP_EVENT_ICON", "MAP_CAREER_GOLD_ICON", "MAP_CAREER_SILVER_ICON", "MAP_CAREER_BRONZE_ICON",
                                      "MAP_CAREER_NONE_ICON", "MAP_CHALLENGE_GOLD_ICON", "MAP_CHALLENGE_NONE_ICON", "MAP_GREY_LITTLE_ICON",
                                      "MAP_GREY_LITTLE_ICON", "MAP_LIVERY_DISABLE_ICON", "MAP_VISTAPOINT_DISABLE_ICON", "MAP_LIVERY_LITTLE_ICON",
                                      "MAP_LIVERY_ICON", "MAP_VISTAPOINT_LITTLE_ICON", "MAP_VISTAPOINT_ICON", "MAP_HELIPORT_ICON"};
    return type >= 0 && type < 22 ? k[type] : "";
}

constexpr float kSize14 = 14, kSize30 = 30, kSize35 = 35;  // +0x1a28, +0x1a2c, +0x1a30 (0x4acb40, Map_Cache SIZE1..3)

}  // namespace

std::vector<Menu::MapItem> Menu::mapItems() const {
    std::vector<MapItem> out;
    if (!career_ || !sessionEvent_.empty()) return out;  // 0x4a9f50: only while no mission runs
    const fuel::CareerRules& R = career_->rules();
    const fuel::CareerState& S = career_->state();
    // 0x4aa4d0: every hub at its map position (port: the camp, hubs.hpp), enabled = unlock test 0x579720.
    for (size_t h = 0; h < R.hubs.size(); ++h)
        if (const Hub* hub = findHub(hubs_, R.hubs[h].name))
            out.push_back({float(hub->camp.x), float(hub->camp.z), 6, int(h), int(h), career_->hubUnlockTest(int(h)), kSize30});
    const int H = career_->currentHub();
    if (H < 0 || size_t(H) >= R.hubs.size()) return out;
    auto startOf = [&](int mission, float& x, float& z) {
        const std::string& n = R.missions[size_t(mission)].name;
        for (const Event& e : events_)
            if (strcasecmp(e.id.c_str(), n.c_str()) == 0) return x = e.race.start[0], z = e.race.start[2], true;
        return false;
    };
    // 0x4aa650: available career races, type 7 + best rank.
    const auto& hub = R.hubs[size_t(H)];
    for (size_t k = 0; k < hub.career.size(); ++k) {
        const fuel::MissionStatus& m = S.career[size_t(H)][k];
        float x, z;
        if (m.available && startOf(hub.career[k], x, z)) out.push_back({x, z, 7 + std::clamp(m.bestRank, 0, 3), int(k), H, true, kSize30});
    }
    // 0x4aa7c0: available challenges, 11 won / 12 not.
    for (size_t k = 0; k < hub.challenges.size(); ++k) {
        const fuel::MissionStatus& m = S.challenges[size_t(H)][k];
        float x, z;
        if (m.available && startOf(hub.challenges[k], x, z)) out.push_back({x, z, m.bestRank == 3 ? 12 : 11, int(k), H, true, kSize30});
    }
    // 0x4aa930: the hub's POIs; liveries and vistas once revealed, heliports only in an opened hub.
    for (size_t k = 0; k < hub.liveries.size(); ++k) {
        if (hub.liveries[k].kind != "POI") continue;
        const auto& f = S.liveries[size_t(H)][k];
        if (f.b) out.push_back({hub.liveries[k].x, hub.liveries[k].z, 13, int(k), H, true, kSize14});
        else if (f.a) out.push_back({hub.liveries[k].x, hub.liveries[k].z, 17, int(k), H, false, kSize14});
    }
    for (size_t k = 0; k < hub.vistaPoints.size(); ++k) {
        const auto& f = S.vistas[size_t(H)][k];
        if (f.b) out.push_back({hub.vistaPoints[k].x, hub.vistaPoints[k].z, 14, int(k), H, true, kSize14});
        else if (f.a) out.push_back({hub.vistaPoints[k].x, hub.vistaPoints[k].z, 19, int(k), H, false, kSize14});
    }
    if (S.hubs[size_t(H)].opened)
        for (size_t k = 0; k < hub.teleports.size(); ++k)
            out.push_back({hub.teleports[k].x, hub.teleports[k].z, 21, int(k), H, true, kSize30 * 0.75f});
    return out;
}

void Menu::mapOpen() {
    // The map opens on the player, oriented as the minimap (+0x1b5c/+0x1b60 = +0x19f4/+0x19f8 at the first update).
    map_.open = true;
    map_.focus = false;
    if (!zones_ || !roads_) editorWorld();  // PK19 [port]: the menus process loads USA1's roads / zones once (as the editor)
    if (!map_.havePlayer && career_) {
        const int h = career_->currentHub();
        if (h >= 0 && size_t(h) < career_->rules().hubs.size())
            if (const Hub* hub = findHub(hubs_, career_->rules().hubs[size_t(h)].name)) map_.px = float(hub->camp.x), map_.pz = float(hub->camp.z);
    }
    map_.cx = map_.px, map_.cz = map_.pz;
    map_.vx = map_.cx, map_.vz = map_.cz;  // PK19: 0x4829a0 (param 0): +0x1b28 = the cursor
    map_.hx = map_.phx, map_.hz = map_.phz;
    if (std::fabs(map_.hx) + std::fabs(map_.hz) < 1e-6f) map_.hx = -1, map_.hz = 0;
    map_.selected = -1;
    // P75 0x4829a0: opens at PP320 = 140000 (the whole world) and flies in: the target becomes 5000 (0x9dc3f8) after
    // 0x482e50 [O]; the displayed radius halves its gap to the target every frame (0x4ac3d0).
    map_.zoom = 140000.0f, map_.zoomTarget = 5000.0f, map_.held = 0;
    mapCursorHub(true, false);  // PK19: 0x4829a0 -> 0x4838d0(cursor, 1)
}

// ---- PK19 ---------------------------------------------------------------------------------------------------------
// 0x4a7f40 -> 0x577500: Zone_FindContaining of (x, z), then the hub record whose +0x164 is that zone (port: the zone's
// name is the hub's).
int Menu::mapHubAt(float x, float z) const {
    if (!zones_ || !career_) return -1;
    const int zi = zones_->find(x, z);
    if (zi < 0) return -1;
    const auto& hs = career_->rules().hubs;
    for (size_t i = 0; i < hs.size(); ++i)
        if (strcasecmp(hs[i].name.c_str(), zones_->zones[size_t(zi)].name.c_str()) == 0) return int(i);
    return -1;
}

// 0x4838d0(cursor, setZoom) [O]: +0x2d4 = the hub under the cursor; when there is one: setZoom -> +0x2d8 = PP[321 + uid
// +0x168]; Map ITEM_ID_TITLE_HUB_INFO = the entry of its name TT (+0x158, 0x613730), ITEM_ID_PICT_INFO = the hub picture
// IGP +0xd78 + 4 x that entry (INT_PICTURE_EVENT_<entry + 1>, 0x483750), Map_unlocked_hub TEXT_SELECTION_INFO = the same
// entry (0x4837b0), the stats (0x483bc0, drawMapInfo). [port] the Map tab's browsed hub (hubIndex_: the title entry, the
// picture, the hub pages) follows the hub under the cursor when the player pans (0x483150), not when the map opens: the
// port's hubIndex_ also drives the hub menus (FUEL's +0x954), which opening the map must not move.
void Menu::mapCursorHub(bool setZoom, bool browse) {
    const int h = mapHubAt(map_.cx, map_.cz);
    if (h >= 0) map_.hub = h;
    else return;  // 0x4838d0 keeps the rest when no hub (+0x2d4 = 0 only stops 0x4839e0 / 0x483270)
    if (setZoom) {
        HudMemory& m = hudMem_;
        if (m.pp.empty()) m.pp = hud::loadPp(dataDir_ + "/CommonComponent/Param/pp001.tsc");
        if (size_t(321 + h) < m.pp.size() && m.pp[size_t(321 + h)] > 0) map_.hubZoom = m.pp[size_t(321 + h)];  // uid = file order
    }
    if (browse)
        for (size_t i = 0; i < hubs_.size(); ++i)
            if (career_->rules().hubIndex(hubs_[i].id) == h) hubIndex_ = int(i);
    mapHubPicture();
}

// 0x483750: Map ITEM_ID_PICT_INFO = IGP +0xd78 + 4 x (the TITLE_HUB_INFO entry of the browsed hub) = INT_PICTURE_EVENT_<i + 1>.
void Menu::mapHubPicture() {
    map_.pict.clear();
    if (Page* m = scripts_.page("Map"))
        if (Item* title = m->find("ITEM_ID_TITLE_HUB_INFO"); title && size_t(hubIndex_) < hubs_.size()) {
            std::string key = "TT_HUB_" + hubs_[size_t(hubIndex_)].id;
            for (char& ch : key) ch = char(std::toupper(static_cast<unsigned char>(ch)));
            for (size_t i = 0; i < title->entries.size(); ++i) {
                std::string k = title->entries[i].key;
                for (char& ch : k) ch = char(std::toupper(static_cast<unsigned char>(ch)));
                if (k == key) map_.pict = "INT_PICTURE_EVENT_" + std::to_string(i + 1);
            }
        }
}

// InGameMapHUD vf21 0x46a8f0 [O x87]: zoomed out when 0.5 x (PP320 + pause +0x2d8) < zoom +0x19e8.
bool Menu::mapZoomedOut() const {
    const float pp320 = hudMem_.pp.size() > 320 && hudMem_.pp[320] != 0 ? hudMem_.pp[320] : 140000.0f;
    return 0.5f * (pp320 + map_.hubZoom) < map_.zoom;
}

// 0x4839e0 [O] (per frame, after the keys and the map update): nothing without a hub under the cursor; when not zoomed out,
// the selection's type picks Map ITEM_ID_PICT_INFO: 7..10 VIGNETTE_GENERIC_CAREER (+0xdd0), 11 / 12 _CHALLENGE (+0xdd4),
// 13 / 15 / 17 / 18 the livery's own picture (0x484240: MaterialAnim crc "<MESH>_<n>", AddVehicleLivery 0x5756e0 entry +4),
// 14 / 16 / 19 / 20 _VISTA (+0xdcc), 21 _HELIPORT (+0xdd8). The texts are filled by drawMapInfo (0x483e80 / 0x484090 /
// 0x484240 / 0x4843f0 / 0x483bc0). Nothing resets the picture but 0x4838d0 (the cursor moving).
void Menu::mapElementInfo() {
    if (map_.hub < 0 || !career_ || mapZoomedOut()) return;
    const auto items = mapItems();
    if (map_.selected < 0 || map_.selected >= int(items.size())) return;
    const MapItem& it = items[size_t(map_.selected)];
    const int H = career_->currentHub();
    switch (it.type) {
        case 7: case 8: case 9: case 10: map_.pict = "VIGNETTE_GENERIC_CAREER"; break;
        case 11: case 12: map_.pict = "VIGNETTE_GENERIC_CHALLENGE"; break;
        case 13: case 15: case 17: case 18:
            if (H >= 0 && size_t(H) < career_->rules().hubs.size() && size_t(it.index) < career_->rules().hubs[size_t(H)].liveries.size()) {
                const auto& l = career_->rules().hubs[size_t(H)].liveries[size_t(it.index)];
                map_.pict = l.mesh + "_" + std::to_string(l.index);
            }
            break;
        case 14: case 16: case 19: case 20: map_.pict = "VIGNETTE_GENERIC_VISTA"; break;
        case 21: map_.pict = "VIGNETTE_GENERIC_HELIPORT"; break;
        default: break;
    }
}

// The map's ground layers, shared by the Map tab (0x46a700) and the editor map (0x467090), drawn first:
//  - 0x4a85e0: the scissor rect (shorts M+0x19c0..+0x19c6) filled with (0.06, 0.06, 0.06, 1) at depth 0.08 (0x609ad0)
//    [port: MapHUD's 2x colour -> 0.12];
//  - 0x4ae930 (centre: the Map tab's view centre +0x1b28, the editor's cursor +0x1b38): the CLIPMAPS terrain picture,
//    levels 0, L - 1, L (hud::mapTiles); 0x608560 per tile: "CLIPMAPS_%d" (base + index) as a MaterialAnim (0x6093d0),
//    drawn only when resident (0x6ddd20; port: found in the DPCs), quad 0x609a40 at depth 0.07 with corners (x0, z0),
//    (x1, z0), (x1, z1), (x0, z1) through the view, uv (0, 0) (1, 0) (1, 1) (0, 1) (0xa66200 / 0xa062f0 / 0xa062d8 /
//    0xa06300), colour (0.5, 0.5, 0.5, 1) (= white in the 2x modulate) [O asm 0x6085b8 .. 0x608880].
void Menu::drawMapBase(UiRenderer& r, float x, float y, float w, float h, const hud::MapView& v, float cx, float cz, float zoom) {
    r.rect(x, y, w, h, Rgba{0.12f, 0.12f, 0.12f, 1});
    for (const hud::MapTile& t : hud::mapTiles(zoom, cx, cz)) {
        const BitmapRef b = assets_->image("CLIPMAPS_" + std::to_string(t.texture));
        if (!b) continue;
        const auto p0 = v.toScreen(t.x0, t.z0), p1 = v.toScreen(t.x1, t.z0), p2 = v.toScreen(t.x1, t.z1), p3 = v.toScreen(t.x0, t.z1);
        const Rgba c{1, 1, 1, 1};
        const UiVertex a{p0[0], p0[1], 0, 0, c}, bq{p1[0], p1[1], 1, 0, c}, cq{p2[0], p2[1], 1, 1, c}, d{p3[0], p3[1], 0, 1, c};
        r.triangles(&b, {a, bq, cq, a, cq, d});
    }
}

// P75: 0x482eb0 / 0x483150 per frame. Zoom: while a zoom key is held, target +-= displayed x dt x PP46 (1), clamped to
// [PP340 = 1500, PP320 = 140000]; displayed = (displayed + target) / 2. Cursor: while a direction is held, cursor +=
// rot(h) (sx, -sz) x displayed x dt x PP45 (0.7) (0x4c8f80 stick, clamped to length 1), clamped to +-65530 (0x4a0e60).
void Menu::mapUpdate(float dt) {
    if (!map_.open) return;
    mapSelect();  // P99: before this frame's moves, as the draw saw it
    auto held = [&](MenuKey k) { return (map_.held >> (int(k) - int(MenuKey::MapUp))) & 1u; };
    if (held(MenuKey::MapZoomOut)) map_.zoomTarget += map_.zoom * dt;
    if (held(MenuKey::MapZoomIn)) map_.zoomTarget -= map_.zoom * dt;
    map_.zoomTarget = std::clamp(map_.zoomTarget, 1500.0f, 140000.0f);
    map_.zoom = (map_.zoom + map_.zoomTarget) * 0.5f;
    float sx = float(held(MenuKey::MapRight)) - float(held(MenuKey::MapLeft)), sz = float(held(MenuKey::MapUp)) - float(held(MenuKey::MapDown));
    const float l = std::hypot(sx, sz);
    if (l > 1) sx /= l, sz /= l;
    if (l > 0) {
        // screen right = world (hx, hz), screen up = world (hz, -hx) (see mapKey).
        const float step = map_.zoom * dt * 0.7f;
        map_.cx += step * (sx * map_.hx + sz * map_.hz);
        map_.cz += step * (sx * map_.hz - sz * map_.hx);
        map_.cx = std::clamp(map_.cx, -65530.0f, 65530.0f), map_.cz = std::clamp(map_.cz, -65530.0f, 65530.0f);
        mapCursorHub(false, true);  // PK19: 0x483150 -> 0x4838d0(cursor, 0) after every move
    }
    // PK19 0x4826a0 -> 0x4825b0 (outside the opening fly-in, +0x2dc == 0): the view centre +0x1b28 = (2 c + 2 cursor) / 4.
    map_.vx = (map_.vx * 2.0f + map_.cx * 2.0f) * 0.25f;
    map_.vz = (map_.cz * 2.0f + map_.vz * 2.0f) * 0.25f;
    mapElementInfo();  // PK19: 0x4839e0 after the map update
}

// 0x46a4d0: the selection = the item nearest the cursor (x/z), when within zoom x 30 / PP340 (P99: per frame, from
// mapUpdate, so the action key works without a draw).
void Menu::mapSelect() {
    HudMemory& m = hudMem_;
    if (m.pp.empty()) m.pp = hud::loadPp(dataDir_ + "/CommonComponent/Param/pp001.tsc");
    const float pp340 = m.pp.size() > 340 && m.pp[340] != 0 ? m.pp[340] : 1500.0f;
    const auto items = mapItems();
    map_.selected = -1;
    float best = 1e30f;
    for (size_t i = 0; i < items.size(); ++i) {
        const float d2 = (items[i].x - map_.cx) * (items[i].x - map_.cx) + (items[i].z - map_.cz) * (items[i].z - map_.cz);
        if (d2 < best) best = d2, map_.selected = int(i);
    }
    const float rad = map_.zoom * 30.0f / pp340;
    if (best > rad * rad) map_.selected = -1;
}

void Menu::mapKey(MenuKey k) {
    map_.focus = true;  // [port] the map keys move the focus from the left menu to the map
    // Pan in screen directions (0x4a0de0 "centre += rotated offset"); with X = d.x hx + d.z hz, Y = d.z hx - d.x hz,
    // screen right (1, 0) is world (hx, hz), screen up (0, -1) is world (hz, -hx). P75: held keys, moved per frame
    // by mapUpdate (0x483150 / 0x482eb0).
    switch (k) {
        case MenuKey::MapLeft:
        case MenuKey::MapRight:
        case MenuKey::MapUp:
        case MenuKey::MapDown:
        case MenuKey::MapZoomIn:
        case MenuKey::MapZoomOut: map_.held |= 1u << (int(k) - int(MenuKey::MapUp)); break;
        case MenuKey::MapTarget:
            // 0x484580 / 0x4844c0: no mission running -> toggle the target at the cursor.
            if (!sessionEvent_.empty()) break;
            map_.target = !map_.target;
            if (map_.target) map_.tx = map_.cx, map_.tz = map_.cz;
            break;
        case MenuKey::MapAction: {
            // 0x482b60 on the selected item.
            const auto items = mapItems();
            if (map_.selected < 0 || map_.selected >= int(items.size()) || !career_) break;
            const MapItem& it = items[size_t(map_.selected)];
            const float d2 = (it.x - map_.px) * (it.x - map_.px) + (it.z - map_.pz) * (it.z - map_.pz);
            if (it.type == 6) {
                if (!it.enabled || it.index == career_->currentHub()) break;
                if (!career_->state().trips.empty()) career_->state().trips.back().valid = false;  // 0x56ca00 [O]
                career_->setCurrentHub(it.index);                                                 // TeleportToHub
                saveCareer();
                request_ = {MenuRequest::FreeRide, "", career_->rules().hubs[size_t(it.index)].name, vehicle_};
            } else if (it.type >= 13 && it.type <= 20) {
                if (!it.enabled || d2 < 2500.0f) break;  // >= 50 m away
                request_ = {MenuRequest::Teleport, "", "", "", it.x, it.z};
            } else if (it.type == 21) {
                if (d2 < 2500.0f) break;
                career_->useHeliport();  // 0x56ca00 + game+0x3460 += 1
                saveCareer();
                request_ = {MenuRequest::Teleport, "", "", "", it.x, it.z};
            }
            break;
        }
        default: break;
    }
}

// PK21: the roads and hub zones of a map view (moved out of drawMap; the editor map 0x467090 draws the same layers).
void Menu::drawMapGround(UiRenderer& r, const hud::MapView& v, float cx, float cz, float zoom, float curX, float curZ) {
    if (std::isnan(curX)) curX = cx, curZ = cz;
    // Roads: the minimap's passes with the _NO_FADE set (0x4a3750 with arg 0, the big map).
    if (roads_ && roadTypes_) {
        const float tol = hud::mapTolerance(zoom);
        const int lod = hud::mapLod(zoom);
        const float half = 0.75f * zoom;
        static const char* kTex[3] = {"GPS_ECRAN_RIVER_NO_FADE", "GPS_ECRAN_OFFROAD_NO_FADE", "GPS_ECRAN_ROUTE_NO_FADE"};
        for (int pass = 0; pass < 3; ++pass) {
            const BitmapRef tex = assets_->image(kTex[pass]);
            std::vector<UiVertex> tris;
            for (size_t i = 0; i < roads_->size(); ++i) {
                const RoadGraph::Rec& rc = roads_->rec(i);
                if (rc.type >= 21 || rc.maxX < (cx - half) * 0.5f || rc.minX > (cx + half) * 0.5f || rc.maxZ < (cz - half) * 0.5f ||
                    rc.minZ > (cz + half) * 0.5f)
                    continue;
                const RoadTypeParams& T = (*roadTypes_)[size_t(rc.type)];
                if (T.mapClass <= lod) continue;
                const int which = T.kind == 14 ? 0 : T.aiSpeedBlend > 0 ? 1 : 2;
                if (which != pass) continue;
                std::vector<std::array<float, 2>> pts;
                for (uint32_t k = 0; k < rc.n; ++k) pts.push_back(roads_->point(uint32_t(i), k));
                pts = hud::simplify(pts, tol);
                if (pts.size() < 2) continue;
                const Rgba col = rc.type <= 3 ? Rgba{0.2f, 0.2f, 0.2f, 0.4f} : Rgba{1, 1, 1, 1};
                const float hw = hud::roadHalfWidth(rc.type, v.ppm, 1.0f);
                std::array<float, 2> pl{}, pr{};
                for (size_t k = 0; k < pts.size(); ++k) {
                    const auto& a = pts[k > 0 ? k - 1 : 0];
                    const auto& b = pts[k + 1 < pts.size() ? k + 1 : k];
                    auto t = v.dirToScreen(b[0] - a[0], b[1] - a[1]);
                    const float tl = std::hypot(t[0], t[1]);
                    if (tl > 0) t[0] /= tl, t[1] /= tl;
                    const auto s = v.toScreen(pts[k][0], pts[k][1]);
                    const std::array<float, 2> L{s[0] + t[1] * hw, s[1] - t[0] * hw}, Rr{s[0] - t[1] * hw, s[1] + t[0] * hw};
                    if (k > 0) {
                        const UiVertex a0{pl[0], pl[1], 0.5f, 0, col}, b0{pr[0], pr[1], 0.5f, 1, col}, a1{L[0], L[1], 0.5f, 0, col}, b1{Rr[0], Rr[1], 0.5f, 1, col};
                        for (const UiVertex& q : {a0, b0, a1, b0, b1, a1}) tris.push_back(q);
                    }
                    pl = L, pr = Rr;
                }
            }
            r.triangles(tex ? &tex : nullptr, tris);
        }
    }
    // P75: hub zones 0x4a8090 (after the roads and race line, before the icons) [O]: every zone with a hub (0x577500:
    // hub +0x164 == zone; port: the names match) except the one under the map cursor (Zone_FindContaining 0x6ee810 of
    // M+0x1b38/+0x1b40), then that one last. 0x4a81f0 per zone: locked hubs (not tutorial mode, 0x579720 false) filled
    // with (0, 0, 0, 0.5) by ear clipping 0x60a7c0; every edge of the zone's list as a segment 0x6097f0, black, 3 S wide
    // (0x9dbf40), DEFAULT_MAT_LINE; the cursor's zone also 10 S wide (0x996494) in (0.5, 0, 0, 0.35) = red at 35 %
    // in MapHUD's 2x colours [I: doubled here]. Shared borders are drawn by both zones (as FUEL does).
    if (zones_ && career_ && !zones_->zones.empty()) {
        const int cur = zones_->find(curX, curZ);  // PK19: the cursor M+0x1b38 (the Map tab's view centre may lag it)
        auto hubOf = [&](const fuel::HubZone& z) {
            const auto& hs = career_->rules().hubs;
            for (size_t i = 0; i < hs.size(); ++i)
                if (strcasecmp(hs[i].name.c_str(), z.name.c_str()) == 0) return int(i);
            return -1;
        };
        auto segment = [&](std::vector<UiVertex>& out, const std::array<float, 2>& a, const std::array<float, 2>& b, float width, const Rgba& c) {
            const auto A = v.toScreen(a[0], a[1]), B = v.toScreen(b[0], b[1]);
            float dx = B[0] - A[0], dy = B[1] - A[1];
            const float l = std::hypot(dx, dy);
            if (l <= 0) return;
            dx *= 0.5f * width / l, dy *= 0.5f * width / l;
            const UiVertex p0{A[0] - dy, A[1] + dx, 0, 0, c}, p1{A[0] + dy, A[1] - dx, 0, 1, c}, p2{B[0] - dy, B[1] + dx, 1, 0, c},
                p3{B[0] + dy, B[1] - dx, 1, 1, c};
            for (const UiVertex& q : {p0, p1, p2, p1, p3, p2}) out.push_back(q);
        };
        const BitmapRef line = assets_->image("DEFAULT_MAT_LINE");
        auto drawZone = [&](const fuel::HubZone& z, bool current) {
            const int h = hubOf(z);
            if (h < 0) return;
            if (!career_->hubUnlocked(h) && z.ring.size() >= 3) {  // tutorial mode (game+0x32bc == 2) not in the port
                std::vector<std::array<float, 2>> scr;
                for (const auto& p : z.ring) scr.push_back(v.toScreen(p[0], p[1]));
                std::vector<UiVertex> fill;
                const Rgba c{0, 0, 0, 0.5f};
                for (const auto& t : fuel::earClip(scr))
                    for (int k : t) fill.push_back({scr[size_t(k)][0], scr[size_t(k)][1], 0, 0, c});
                r.triangles(nullptr, fill);
            }
            std::vector<UiVertex> lines, band;
            for (const auto& e : z.edges) {
                const auto& a = zones_->points[size_t(e[0])];
                const auto& b = zones_->points[size_t(e[1])];
                segment(lines, a, b, 3.0f, Rgba{0, 0, 0, 1});
                if (current) segment(band, a, b, 10.0f, Rgba{1, 0, 0, 0.35f});
            }
            r.triangles(line ? &line : nullptr, lines);
            if (current) r.triangles(nullptr, band);
        };
        for (size_t i = 0; i < zones_->zones.size(); ++i)
            if (int(i) != cur) drawZone(zones_->zones[i], false);
        if (cur >= 0) drawZone(zones_->zones[size_t(cur)], true);
    }
}

void Menu::drawMap(UiRenderer& r) {
    Page* cache = scripts_.page("Map_Cache");
    if (!cache) return;
    if (!map_.open) mapOpen();
    for (Item& it : cache->items)
        if (it.id.rfind("ITEM_ID_MAP_ELEMENT_SIZE", 0) == 0) it.visible = false;  // size templates (0x4acb40)
    const Item* clip = cache->find("ITEM_ID_CLIPPING_RECT");
    if (!clip) return drawPage(r, *cache);
    // PK19: the rect as drawn (0x4ac8a0 asks the item for its rect: the anchor's 36-unit shift (ITEM_ID_CLIPPING_RECT and the
    // halo frame both have anchor V 1) included); the raw script y put the map 36 units above its frame.
    float x = 0, y = 0;
    itemRect(*clip, x, y);
    const float w = clip->w, h = clip->h;
    HudMemory& m = hudMem_;
    if (m.pp.empty()) m.pp = hud::loadPp(dataDir_ + "/CommonComponent/Param/pp001.tsc");
    // PK19: the view is centred on +0x1b28 (map_.vx / vz, following the cursor +0x1b38 = map_.cx / cz).
    const hud::MapView v = hud::mapView(x, y, w, h, map_.zoom, map_.vx, map_.vz, map_.hx, map_.hz);
    r.setClip(x, y, w, h);
    // 0x46a700 [O asm]: background 0x4a85e0, terrain picture 0x4ae930 (centre +0x1b28), roads 0x4a3750(0), [race line /
    // GPS route], hub zones 0x4a8090 (cursor), the cursor icon 0x4aca30, the items 0x4a9e20, [gates 0x4aba90], the GPS
    // target pin 0x4aad40, [specials 0x4ac1a0, opponents 0x4ab920], the player 0x4ab8d0, the selection 0x46a880.
    drawMapBase(r, x, y, w, h, v, map_.vx, map_.vz, map_.zoom);
    drawMapGround(r, v, map_.vx, map_.vz, map_.zoom, map_.cx, map_.cz);  // PK21: shared with the editor map
    // PK19 0x4aca30 [O asm]: MAP_TARGET_ICON (IGP +0xcdc, slot 91) at the cursor +0x1b38 (ECX), size +0x1a34 (SIZE4 = 52),
    // colour (0.5, 0.5, 0.5, +0x1af4) = white x alpha (InGameMapHUD's +0x1af4 stays at the ctor's 1.0: only the minimap's
    // update 0x49f6d0 fades it), skipped below 1e-6, culled by 0x4ad5e0, upright.
    constexpr float kSize52 = 52;
    auto culled = [&](float sx, float sy, float half) { return sx + half < x || sx - half > x + w || sy + half < y || sy - half > y + h; };  // 0x4ad5e0
    if (BitmapRef t = assets_->image("MAP_TARGET_ICON")) {
        const auto s = v.toScreen(map_.cx, map_.cz);
        if (!culled(s[0], s[1], kSize52 * 0.5f)) r.sprite(t, s[0], s[1], kSize52, kSize52, 0.5f, 0.5f, 0, Rgba{1, 1, 1, 1});
    }
    // Items, then the selection (0x46a4d0; P99: computed per frame by mapSelect, as FUEL's update does).
    auto items = mapItems();
    mapSelect();
    if (map_.selected >= int(items.size())) map_.selected = -1;
    if (map_.selected >= 0) {
        MapItem& sel = items[size_t(map_.selected)];
        sel.size = kSize35;
        if (sel.type == 13) sel.type = 15;
        else if (sel.type == 14) sel.type = 16;
        else if (sel.type == 17) sel.type = 18;
        else if (sel.type == 19) sel.type = 20;
    }
    auto drawItemIcon = [&](const MapItem& it, float scale) {
        const BitmapRef b = assets_->image(mapIcon(it.type));
        if (!b) return;
        const auto s = v.toScreen(it.x, it.z);
        const float sz = it.size * scale;
        if (culled(s[0], s[1], sz * 0.5f)) return;  // 0x4ad5e0
        r.sprite(b, s[0], s[1], sz, sz, 0.5f, 0.5f, 0, Rgba{1, 1, 1, it.type == 6 && !it.enabled ? 0.5f : 1.0f});
    };
    for (size_t i = 0; i < items.size(); ++i)
        if (int(i) != map_.selected) drawItemIcon(items[i], 1.0f);
    // PK19 0x4aad40 [O asm]: unless zoomed out (vt+0x54 = 0x46a8f0) and when the minimap's GPS target is set (MapHUD +0x60,
    // position +0x64 / +0x6c): MAP_TARGET_Y (IGP +0xde8, slot 158) of size +0x1a2c (SIZE2 x 1.2 = 30), culled at the
    // target by 0x4ad5e0(size / 2), drawn raised by 0.6 x size / 2 (top = Y - 0.6 h - h, h = size / 2), white, upright.
    if (map_.target && !mapZoomedOut())
        if (BitmapRef t = assets_->image("MAP_TARGET_Y")) {
            const auto s = v.toScreen(map_.tx, map_.tz);
            const float hs = kSize30 * 0.5f;
            if (!culled(s[0], s[1], hs)) r.sprite(t, s[0], s[1] - 0.6f * hs, kSize30, kSize30, 0.5f, 0.5f, 0, Rgba{1, 1, 1, 1});
        }
    if (BitmapRef pl = assets_->image("MAP_PLAYER_CAR_ICON")) {
        const auto s = v.toScreen(map_.px, map_.pz);
        const auto d = v.dirToScreen(map_.phz, -map_.phx);
        r.sprite(pl, s[0], s[1], kSize30, kSize30, 0.5f, 0.5f, std::atan2(d[0], -d[1]) * 57.29578f, Rgba{1, 1, 1, 1});
    }
    if (map_.selected >= 0) drawItemIcon(items[size_t(map_.selected)], 1.5f);  // 0x46a880 (last)
    r.clearClip();
    // PK19 0x483270 [O asm 0x483270..0x4832d4]: the map first (InGameMapHUD vt+0x14), then Map_Cache (page 12: the halo
    // frame and the ink stains over the map's edges), then Map (page 2). Drawing Map_Cache first let the map show over
    // the frame's top edge (the clip rect starts above the frame's visible border).
    drawPage(r, *cache);
    // PK19: the Map page's picture (0x483750 / 0x4839e0 write ITEM_ID_PICT_INFO's three images +0x78 / +0x7c / +0x80).
    if (!map_.pict.empty())
        if (Page* mp = scripts_.page("Map"))
            if (Item* it = mp->find("ITEM_ID_PICT_INFO")) it->imgNormal = it->imgEnabled = it->imgFocus = map_.pict;
}

// PK19: the record time of a career race / challenge (0x483e80 / 0x484090 [O asm]): rank +0x120 == 3 -> "- - - - - - - -";
// else t = float(0x7fffffff - best score +0x110) x 0.01 s, split by 0x484690 (each part fistp under RC down = floor, then
// unsigned %): h = t / 3600, m = (t x 1/60) % 60, s = t % 60, cs = (t x 100) % 100 -> "%02d:%02d.%02d" (m, s, cs) when
// h == 0, else "%02d:%02d:%02d:%02d" (h, m, s, cs).
static std::string mapRecordTime(int rank, uint32_t bestScore) {
    if (rank == 3) return "- - - - - - - -";
    const uint32_t q = 0x7fffffffu - bestScore;
    const float t = float(double(q) * double(0.01f));
    const unsigned h = unsigned(int64_t(std::floor(double(t) * double(0.00027777778f))));
    const unsigned m = unsigned(int64_t(std::floor(double(t) * double(0.016666668f)))) % 60u;
    const unsigned sec = unsigned(int64_t(std::floor(double(t)))) % 60u;
    const unsigned cs = unsigned(int64_t(std::floor(double(t) * 100.0))) % 100u;
    char b[48];
    if (h == 0) std::snprintf(b, sizeof b, "%02u:%02u.%02u", m, sec, cs);
    else std::snprintf(b, sizeof b, "%02u:%02u:%02u:%02u", h, m, sec, cs);
    return b;
}

void Menu::drawMapInfo(UiRenderer& r) {
    // Info page (0x483270) for the selection drawMap made.
    auto items = mapItems();
    if (map_.selected >= int(items.size())) map_.selected = -1;
    if (map_.selected >= 0) {
        MapItem& s = items[size_t(map_.selected)];
        s.type = s.type == 13 ? 15 : s.type == 14 ? 16 : s.type == 17 ? 18 : s.type == 19 ? 20 : s.type;
    }
    const fuel::CareerRules& R = career_->rules();
    const fuel::CareerState& S = career_->state();
    const int H = career_->currentHub();
    auto setText = [&](Page* p, const char* id, const std::string& t) {
        if (Item* it = p ? p->find(id) : nullptr) it->hasDynText = true, it->dynText = t;
    };
    const MapItem* sel = map_.selected >= 0 ? &items[size_t(map_.selected)] : nullptr;
    auto eventName = [&](int mission) {
        const std::string& n = R.missions[size_t(mission)].name;
        for (const Event& e : events_)
            if (strcasecmp(e.id.c_str(), n.c_str()) == 0) return e.nameText ? textOf(e.nameText) : n;
        return n;
    };
    // PK19: the hub pages follow the hub under the cursor (pause +0x2d4 = map_.hub, 0x4838d0), not the player's hub.
    const int HC = map_.hub >= 0 && size_t(map_.hub) < R.hubs.size() ? map_.hub : -1;
    const bool hcUnlocked = HC >= 0 && career_->hubUnlockTest(HC);  // 0x579720
    // 0x483bc0 (hub stats, Map_MetaInfos) for the hub under the cursor.
    auto drawStats = [&]() {
        Page* p = scripts_.page("Map_MetaInfos");
        if (!p) return;
        // P75 0x483bc0 [O]: career = medal points (rank +0x120: 0 -> 3, 1 -> 2, 2 -> 1, 3 -> 0, 0x5794c0) of 3 x races;
        // challenges with rank != 3 (0x579950); vistas visited (0x579b90) of +0x234; POI liveries (kind +0x20 == 1) owned (+1)
        // (0x579c90) of the POI count (0x579c50); PK19 0x579d50 / 0x579dd0: dopplers = traffic entries of kinds 1, 0, 2 caught
        // (+1) of hub +0x24c (AddHubTrafficVehicle 0x574fb0 counts Radar_VistaPoint / _VehicleLivery / _Challenge there),
        // bonus car = kind 3 caught of +0x250 (the "Vehicle" entries); GROUP_BONUS_CAR visible iff +0x250 > 0 (0x60fba0, BL =
        // seta).
        const auto& hub = R.hubs[size_t(HC)];
        int cw = 0, chw = 0, vv = 0, lo = 0, ln = 0, tc = 0, tn = 0, bc = 0, bn = 0;
        for (const auto& m : S.career[size_t(HC)]) cw += 3 - std::clamp(m.bestRank, 0, 3);
        for (const auto& m : S.challenges[size_t(HC)]) chw += m.bestRank != 3;
        for (const auto& f : S.vistas[size_t(HC)]) vv += f.b != 0;
        for (size_t k = 0; k < hub.liveries.size(); ++k)
            if (hub.liveries[k].kind == "POI") ++ln, lo += S.liveries[size_t(HC)][k].b != 0;
        for (size_t k = 0; k < hub.traffic.size(); ++k) {
            const bool caught = k < S.traffic[size_t(HC)].size() && S.traffic[size_t(HC)][k].b != 0;
            const int kind = hub.traffic[k].kind();
            if (kind == 3) ++bn, bc += caught;
            else if (kind >= 0 && kind <= 2) ++tn, tc += caught;
        }
        setText(p, "ITEM_ID_CUR_CAREER_RACE", std::to_string(cw)), setText(p, "ITEM_ID_MAX_CAREER_RACE", std::to_string(3 * hub.career.size()));
        setText(p, "ITEM_ID_CUR_CHALLENGE", std::to_string(chw)), setText(p, "ITEM_ID_MAX_CHALLENGE", std::to_string(hub.challenges.size()));
        setText(p, "ITEM_ID_CUR_VISTA_POINT", std::to_string(vv)), setText(p, "ITEM_ID_MAX_VISTA_POINT", std::to_string(hub.vistaPoints.size()));
        setText(p, "ITEM_ID_CUR_LIVERY", std::to_string(lo)), setText(p, "ITEM_ID_MAX_LIVERY", std::to_string(ln));
        setText(p, "ITEM_ID_CUR_DOPPLER", std::to_string(tc)), setText(p, "ITEM_ID_MAX_DOPPLER", std::to_string(tn));
        setText(p, "ITEM_ID_CUR_BONUS_CAR", std::to_string(bc)), setText(p, "ITEM_ID_MAX_BONUS_CAR", std::to_string(bn));
        for (Item& it : p->items)
            if (it.group == "GROUP_BONUS_CAR") it.visible = bn > 0;
        for (Item& it : p->items)  // the TT copies of the CUR items are the "/" separators (TT 987)
            if (it.kind == ItemKind::LabelTT && it.id.rfind("ITEM_ID_CUR_", 0) == 0) it.hasDynText = false;
        drawPage(r, *p);
    };
    // 0x483dd0: Map_locked_hub, stars still needed = hub +0x23c - stars (left as it was without a hub, as 0x4839e0 returns).
    auto drawLocked = [&]() {
        Page* p = scripts_.page("Map_locked_hub");
        if (!p) return;
        if (HC >= 0) setText(p, "ITEM_ID_NB_STARS_NEED_TO_ENTER_HUB", std::to_string(R.hubs[size_t(HC)].starsNeeded - career_->stars()));
        drawPage(r, *p);
    };
    // Map_unlocked_hub: TEXT_SELECTION_INFO = the hub's entry (0x4837b0, the TITLE_HUB_INFO order).
    auto drawUnlocked = [&]() {
        Page* p = scripts_.page("Map_unlocked_hub");
        if (!p) return;
        const Hub* hb = HC >= 0 ? findHub(hubs_, R.hubs[size_t(HC)].name) : nullptr;
        if (Item* it = p->find("ITEM_ID_TEXT_SELECTION_INFO"); it && hb) {
            std::string key = "TT_HUB_" + hb->id + "_INFO";
            for (char& ch : key) ch = char(std::toupper(static_cast<unsigned char>(ch)));
            for (size_t i = 0; i < it->entries.size(); ++i) {
                std::string k = it->entries[i].key;
                for (char& ch : k) ch = char(std::toupper(static_cast<unsigned char>(ch)));
                if (k == key) it->entryIndex = int(i);
            }
        }
        drawPage(r, *p);
    };
    // 0x483270 [O asm 0x483325..0x48367d]: zoomed out (vt+0x54): the hub under the cursor unlocked -> Map_MetaInfos (13),
    // else (also no hub) Map_locked_hub (14). Otherwise by the selection's type; no selection: no hub -> nothing,
    // unlocked -> Map_unlocked_hub (15), locked -> Map_locked_hub.
    if (mapZoomedOut()) {
        if (HC >= 0 && hcUnlocked) drawStats();
        else drawLocked();
        return;
    }
    if (!sel) {
        if (HC < 0) return;
        if (hcUnlocked) drawUnlocked();
        else drawLocked();
        return;
    }
    if (sel->type == 6) {
        // 0x483405 case 6: +0x2d4 == 0 or locked -> Map_locked_hub, else Map_unlocked_hub (the hub under the cursor).
        if (HC < 0 || !hcUnlocked) drawLocked();
        else drawUnlocked();
    } else if (sel->type >= 7 && sel->type <= 10) {
        Page* p = scripts_.page("Map_selection_career");
        if (!p) return;
        const fuel::MissionStatus& ms = S.career[size_t(H)][size_t(sel->index)];
        setText(p, "ITEM_ID_MAP_ELEMENT_NAME", eventName(R.hubs[size_t(H)].career[size_t(sel->index)]));
        // P75 0x483e80 [O]: CUR_MEDAL = 3 - rank (+0x120); YOUR_RECORD (PK19: mapRecordTime, 0x484690); WORLD_RECORD (always)
        // "- - - - - - - -".
        const int rank = std::clamp(ms.bestRank, 0, 3);
        setText(p, "ITEM_ID_CUR_MEDAL_NUMBER", std::to_string(3 - rank));
        setText(p, "ITEM_ID_MAX_MEDAL_NUMBER", "3");
        setText(p, "ITEM_ID_YOUR_RECORD", mapRecordTime(rank, ms.bestScore)), setText(p, "ITEM_ID_WORLD_RECORD", "- - - - - - - -");
        if (Item* it = p->find("ITEM_ID_MEDAL_ICON")) it->imgNormal = it->imgEnabled = it->imgFocus = mapIcon(sel->type);
        drawPage(r, *p);
    } else if (sel->type == 11 || sel->type == 12) {
        // PK19 0x484090 [O asm]: the hub challenge (+0x220[elem +0x1e], 0x5799b0): MAP_ELEMENT_NAME = TT(+0x15c) or the name
        // +0xda; ITEM_ID_CURRENT_STATE (list box +0x128, 0x613b50) = rank +0x120 != 3 ? 1 "Completed" : 0; YOUR_RECORD = the
        // best time (mapRecordTime) or "- - - - - - - -".
        Page* p = scripts_.page("Map_selection_challenge");
        if (!p) return;
        const fuel::MissionStatus& ms = S.challenges[size_t(H)][size_t(sel->index)];
        setText(p, "ITEM_ID_MAP_ELEMENT_NAME", eventName(R.hubs[size_t(H)].challenges[size_t(sel->index)]));
        if (Item* it = p->find("ITEM_ID_CURRENT_STATE")) it->entryIndex = ms.bestRank != 3 ? 1 : 0;
        setText(p, "ITEM_ID_YOUR_RECORD", mapRecordTime(ms.bestRank == 3 ? 3 : 0, ms.bestScore));
        drawPage(r, *p);
    } else if (sel->type == 13 || sel->type == 15 || sel->type == 17 || sel->type == 18) {
        // PK19 0x484240 [O asm]: the hub livery entry (0x578360 by the map object's TT +0x48): MAP_ELEMENT_NAME = TT(entry +0x14);
        // VEHICLE_NAME = TT(+0x6c) of the vehicle record of crc(MESH) (entry +8, 0x477f40); Map ITEM_ID_PICT_INFO = the
        // livery's picture (entry +4 = crc "<MESH>_<n>", set by mapElementInfo); Map_selection_livery ITEM_ID_TEXT_SELECTION_INFO
        // (TT 531 "You don't own this vehicle yet.") visible (+7) iff the vehicle record's owned byte (+1) is 0.
        Page* p = scripts_.page("Map_selection_livery");
        if (!p) return;
        const auto& l = R.hubs[size_t(H)].liveries[size_t(sel->index)];
        setText(p, "ITEM_ID_MAP_ELEMENT_NAME", textOf(l.text));
        const int vi = R.vehicleIndex(l.mesh);
        setText(p, "ITEM_ID_VEHICLE_NAME", vi >= 0 ? vehicleName(R.vehicles[size_t(vi)].label) : l.mesh);
        if (Item* it = p->find("ITEM_ID_TEXT_SELECTION_INFO")) it->visible = !career_->vehicleOwned(l.mesh);
        drawPage(r, *p);
    } else if (sel->type == 14 || sel->type == 16 || sel->type == 19 || sel->type == 20) {
        Page* p = scripts_.page("Map_selection_vista");
        if (!p) return;
        setText(p, "ITEM_ID_MAP_ELEMENT_NAME", textOf(R.hubs[size_t(H)].vistaPoints[size_t(sel->index)].text));
        drawPage(r, *p);
    } else if (sel->type == 21) {
        if (Page* p = scripts_.page("Map_selection_heliport")) drawPage(r, *p);
    }
}

std::pair<int, int> Menu::mapSelection() const {
    const auto items = mapItems();
    if (map_.selected < 0 || map_.selected >= int(items.size())) return {-1, -1};
    return {items[size_t(map_.selected)].type, items[size_t(map_.selected)].index};
}

}  // namespace fuel::ui
