// FUEL's screen band of Navigator3dHUD (vtable 0x9a55bc; P56, notes/p56-hud2.md; GPS ribbons: hud.mm / gps_guide.*)
// and the full-screen VignettageHUD picture:
//  - draw 0x4c0fe0: skipped when the HUDs are hidden or the 3-D GPS is off ([0xa6536e]); then the compass 0x4c1120,
//    the online name tags 0x4c1690 (not ported: no online), and by mode 0x4c0ed0 (0 in a helicopter race, else 2)
//    the icon list 0x4c1f60 and the ribbons 0x4bca50 (hud.mm);
//  - icon list, rebuilt every update by 0x4c0010: race (race state not 0/1) 0x4c0a40, the next gate (MAP_CHECKPOINT_ICON,
//    type 0x14), or the finish (MAP_FINISH_ICON, 0x15) when it is next, or the chevrons 0x12 / 0x13 when the target is
//    off screen; free roam 0x4c0260: career races (medal icon by best rank, types 0..3), challenges (4 won / 5 not),
//    the vista points and liveries not yet taken in an opened hub (0xa, 6 / 8) within 5000 m, the bonus car (0xe),
//    dopplers (0xf..0x11) and the map's GPS target (0x16 MAP_TARGET_Y);
//  - each entry goes in only when it projects on screen (0x4c1030 camera projection, 0x4c10c0 within 10 S of the
//    sides), sorted far to near (0x4c2230); drawn by 0x4c1e60 at (x, band centre), 32 S square, white,
//    alpha = edge fade.
// Port choices are marked "port".
#include "ui/menu.h"

#include "core/freeroam.hpp"
#include "core/hub_zones.hpp"
#include "ui/ui_renderer.h"

#include <algorithm>
#include <strings.h>
#include <cmath>

namespace fuel::ui {
namespace {

// 0x4c1f60: entry type -> InGamePlayer texture (+0xb70 + 4i, names table 0xa05410).
const char* bandIcon(int type) {
    switch (type) {
        case 0: return "MAP_CAREER_GOLD_ICON";          // +0xce8
        case 1: return "MAP_CAREER_SILVER_ICON";        // +0xcec
        case 2: return "MAP_CAREER_BRONZE_ICON";        // +0xcf0
        case 3: return "MAP_CAREER_NONE_ICON";          // +0xcf4
        case 4: return "MAP_CHALLENGE_GOLD_ICON";       // +0xd04
        case 5: return "MAP_CHALLENGE_NONE_ICON";       // +0xd10
        case 6: case 8: return "MAP_LIVERY_ICON";       // +0xd14
        case 10: return "MAP_VISTAPOINT_ICON";          // +0xd18
        case 0xd: return "MAP_FUEL_ICON";               // +0xd20
        case 0xe: return "MAP_BONUS_CAR_ICON";          // +0xd2c
        case 0xf: return "MAP_DOPPLER_LIVERY_ICON";     // +0xd30
        case 0x10: return "MAP_DOPPLER_VISTAPOINT_ICON";// +0xd34
        case 0x11: return "MAP_DOPPLER_RACE_ICON";      // +0xd38
        case 0x14: return "MAP_CHECKPOINT_ICON";        // +0xcd4
        case 0x15: return "MAP_FINISH_ICON";            // +0xcd8
        case 0x16: return "MAP_TARGET_Y";               // +0xde8
        default: return nullptr;
    }
}

struct Projector {
    const HudState& h;
    // 0x4c1030 -> 0x4ca890 -> 0x414b00 [O asm 0x414b49..0x414c77]: the band's projection is a LEVEL camera. Its frame is
    // Quat_FromArc 0x6443a0 from (0, 0, 1) (0xa06350) to the camera's look vector with y zeroed (Camera_Z +0x838 / +0x840,
    // [esp+0x14] = 0), at the camera position (+0x82c..); r = conj(q) (P - E) (0x410510); depth = r.z, kept when > 0.1
    // (0x99648c); x = (W/2 + X0 + S_x (-r.x) / r.z) / W with S_x = viewport +0x4c (0x6663c0: the scene's focal, H / 2 /
    // (0.75 tan(fov / 2))). The camera's pitch (and roll) do not move the band: a point's band x depends on its x z only.
    // minimap-fix: the port projected with the full (pitched) camera.
    bool operator()(float x, float y, float z, float& sx, float& depth) const {
        (void)y;
        const float fl = std::sqrt(h.camF[0] * h.camF[0] + h.camF[2] * h.camF[2]);
        if (fl < 1e-6f) return false;
        const float fx = h.camF[0] / fl, fz = h.camF[2] / fl;  // level forward; level right = (-fz, 0, fx) (the port's camR)
        const float dx = x - h.eye[0], dz = z - h.eye[2];
        depth = dx * fx + dz * fz;
        if (depth <= 0.1f) return false;
        sx = 640.0f + h.camFy * (dz * fx - dx * fz) / depth;
        return true;
    }
};

}  // namespace

void Menu::drawVignette(UiRenderer& r) {
    // VignettageHUD draw 0x4caef0 -> 0x4cc0f0: the VIGNETTAGE picture (InGamePlayer +0xccc, kept at +0x44c) over the whole
    // screen, colour (0.5, 0.5, 0.5, 1) = white in the 2x modulate. (0x4cb1d0 flashes, 0x4caf20 safe-frame bars when
    // [0xa65380] is set and 0x4cbbf0 page StatusPress when game+0x32bc == 1 are not ported.)
    // P102: the destination is the whole screen (0x4cc0f0 passes the viewport's own width and height to 0x60ae00 ->
    // 0x60b490, not the 16:9 canvas), so on a 16:10 display its dark corners are the screen's corners, not the
    // letterboxed canvas's (hud::vignetteRect).
    if (BitmapRef v = assets_->image("VIGNETTAGE")) {
        const hud::Rect q = hud::vignetteRect(r.viewLeft(), r.viewTop(), r.viewWidth(), r.viewHeight());
        r.image(v, q.x, q.y, q.w, q.h, Rgba{1, 1, 1, 1});
    }
}

// PK19: the band's icon list (split out of drawNavigator so the beams use the same entries).
std::vector<Menu::NavEntry> Menu::navEntries(const HudState& hud, int& chevrons) const {
    std::vector<NavEntry> list;
    if (!hud.gps3d || hud.camFy <= 0 || hud.helico) return list;
    const Projector proj{hud};
    // PK19: the colour pointer (EAX) of each 0x4bfc60 call [O asm 0x4c0327..0x4c03ec, the lea's at the call sites]: career
    // races / challenges / doppler 0x11 / the race gate (0x4c0a40) PP2-4, liveries 6 / 8 and doppler 0xf PP5-7, vistas and
    // doppler 0x10 PP11-13, the map target 0x16 0.3 grey, doppler 0xe PP17-19.
    auto colourOf = [](int type) {
        switch (type) {
            case 6: case 8: case 0xf: return 1;
            case 0xa: case 0x10: return 2;
            case 0x16: return 3;
            case 0xe: return 4;
            default: return 0;
        }
    };
    auto add = [&](int type, float wx, float wy, float wz) {  // 0x4bfc60 with the on-screen tests of its callers
        float sx, depth;
        if (!proj(wx, wy, wz, sx, depth) || hud::offScreen(sx) != 0) return;
        const float dx = wx - hud.eye[0], dz = wz - hud.eye[2];
        list.push_back({type, sx, dx * dx + dz * dz, wx, wy, wz, colourOf(type)});
    };
    if (hud.race) {
        // 0x4c0a40: the finish when it is the next target (0x4c09d0: (gate + 1) mod (gates + 1) == 0), else the next gate.
        if (hud.nextGate < hud.gates.size()) {
            const auto& g = hud.gates[hud.nextGate];
            const float gy = hud.nextGate < hud.gateY.size() ? hud.gateY[hud.nextGate] : hud.py;
            const bool finish = hud.nextGate + 1 == hud.gates.size();
            float sx, depth;
            const bool ok = proj(g[0], gy, g[1], sx, depth);
            if (!ok) {
                // Behind the camera: the side of the target from the player's heading (0x4217d0, cross product sign)
                // [port: the camera's right vector].
                const float d = (g[0] - hud.px) * hud.camR[0] + (g[1] - hud.pz) * hud.camR[2];
                chevrons = d > 0 ? 0x13 : 0x12;
            } else if (hud::offScreen(sx) > 0) {
                chevrons = 0x13;
            } else if (hud::offScreen(sx) < 0) {
                chevrons = 0x12;
            } else {
                // +0x84 = -1 (drawNavigator: chevrons == 0)
                list.push_back({finish ? 0x15 : 0x14, sx, 0, g[0], gy, g[1], 0});
            }
        }
    } else if (career_ && sessionEvent_.empty()) {
        // 0x4c0260 (free roam: race state 0 / 1), within 5000 m (d^2 < 2.5e7, 0x9dcfd0) of the camera.
        // minimap-fix [O asm 0x4c02da..0x4c0323, 0x4c0417]: the reference point is the camera (0x421980, [esp+0x40] /
        // [esp+0x48]) and the hub is the one whose zone holds it (0x4a7f40 Zone_FindContaining -> 0x577500: hub +0x164 ==
        // the zone), not the career's current hub (game +0x33c0): no zone hub, no list. [port] without the world's zones
        // the current hub as before.
        const fuel::CareerRules& R = career_->rules();
        const fuel::CareerState& St = career_->state();
        int H = career_->currentHub();
        if (zones_ && !zones_->zones.empty()) {
            H = -1;
            if (const int zi = zones_->find(hud.eye[0], hud.eye[2]); zi >= 0)
                for (size_t i = 0; i < R.hubs.size(); ++i)
                    if (strcasecmp(R.hubs[i].name.c_str(), zones_->zones[size_t(zi)].name.c_str()) == 0) H = int(i);
            if (H < 0) return list;  // 0x4c02ff
        }
        auto near = [&](float x, float z) { return (x - hud.eye[0]) * (x - hud.eye[0]) + (z - hud.eye[2]) * (z - hud.eye[2]) < 2.5e7f; };
        if (H >= 0 && size_t(H) < R.hubs.size()) {
            const bool opened = St.hubs[size_t(H)].opened != 0;  // hub +0x248
            const auto& hub = R.hubs[size_t(H)];
            auto startOf = [&](int mission) -> const Event* {
                const std::string& n = R.missions[size_t(mission)].name;
                for (const Event& e : events_)
                    if (strcasecmp(e.id.c_str(), n.c_str()) == 0) return &e;
                return nullptr;
            };
            for (size_t k = 0; k < hub.career.size(); ++k) {
                const fuel::MissionStatus& m = St.career[size_t(H)][k];
                if (!m.available) continue;  // +0x10d
                if (const Event* e = startOf(hub.career[k]))
                    if (near(e->race.start[0], e->race.start[2]))
                        add(std::clamp(m.bestRank, 0, 3), e->race.start[0], e->race.start[1], e->race.start[2]);  // +0x120: 0..3
            }
            for (size_t k = 0; k < hub.challenges.size(); ++k) {
                const fuel::MissionStatus& m = St.challenges[size_t(H)][k];
                if (!m.available) continue;  // (FreeRoam discovers them, 0x4c0150)
                if (const Event* e = startOf(hub.challenges[k]))
                    if (near(e->race.start[0], e->race.start[2]))
                        add(m.bestRank <= 2 ? 4 : 5, e->race.start[0], e->race.start[1], e->race.start[2]);
            }
            // POIs near the player (0x4ed950, any hub): liveries / vistas not taken, when the current hub is opened.
            if (freeRoam_ && opened)
                for (const auto& p : freeRoam_->pois())
                    if (!p.taken && (p.type == 2 || p.type == 3) && near(p.x, p.z))
                        add(p.type == 3 ? 0xa : 8, p.x, hud.py, p.z);  // port: POI height = the player's
        }
        // The map's GPS target (MapHUD +0x60, position +0x64..+0x6c) -> MAP_TARGET_Y.
        if (map_.target && near(map_.tx, map_.tz)) add(0x16, map_.tx, hud.py, map_.tz);
        // PK19 0x4c0260 -> 0x5c5690 [O asm 0x4c0780..0x4c087f]: the traffic manager's active special slots (HudState::Special
        // live) of the hub whose zone holds the player (0x4a7f40 -> 0x577500), whose hub record (+0x230, 0x30 B) is revealed
        // (+0) and not caught (+1), at (x, 0, z) (y = 0: local_470), within 5000 m and on screen: kind (+4) 0 -> 0x10, 1 ->
        // 0xf, 2 -> 0x11, 3 -> 0xe (jump table 0x4c09c0); each then 0x56e420 (the first-time tutorial 4 / 5, not ported).
        if (zones_ && !hud.specials.empty()) {
            const int zh = H;  // minimap-fix: the same zone hub (the camera's, [esp+0x30] of 0x4c0260)
            static const int kType[4] = {0x10, 0xf, 0x11, 0xe};
            for (const auto& sp : hud.specials) {
                if (!sp.live || sp.hub != zh || size_t(sp.hub) >= St.traffic.size() || size_t(sp.index) >= St.traffic[size_t(sp.hub)].size()) continue;
                const auto& f = St.traffic[size_t(sp.hub)][size_t(sp.index)];
                if (!f.a || f.b || sp.kind < 0 || sp.kind > 3 || !near(sp.x, sp.z)) continue;
                add(kType[sp.kind], sp.x, 0.0f, sp.z);
            }
        }
        // PK19 end
    }
    return list;
}

// PK19 0x4bf820 [O] (per 0x4bfc60 call with the last flag 1, i.e. every icon of the list, emitters HUD +0xbc[n] made by
// 0x4bd380: SpecialEffectsEmitter_G<SpecialEffectsRibbon_G>, atlas 0x1d FX_NAVIGATOR3D_LINE, kind 0x1e, draw flags +0x6c
// = 0): with the camera at E (0x421980) and d = the horizontal distance E -> icon, the icon's y = d < 200 ? 0x4bc8c0 (terrain
// / ray) : 0x445500 (ground) (port: the session's ground, else the icon's own y); when the icon projects on the band
// (0x4c1030), top = 0x4bf760 (the band point (x, band y) unprojected at view depth d); the ribbon particle: +0x60 = 0.05,
// +0x5c = 0.9 (u = 0.05 + 0.9 k / (n - 1)), colour (half floats +0x6a..) = the entry's rgb x 8, alpha +0x70 = its a,
// +0x58 (half width, 0x5fba20) = 0.008 d, +0x80 = 100, +0x7c = 10000; its points (+0xb4, 4: ctor 0x5deef0) spread evenly
// from the ground point up to the top. [port] the band point is unprojected with the port's camera (eye, forward, right, up,
// focal camFy) as E + d (F + (x - 640) / camFy R + (360 - y) / camFy U).
// minimap-fix [O]: 0x4bf760 = renderer vf17 0x843890 (viewport 0: S_x +0x4c, -S_y +0x50, centre +0x54 / +0x58) -> view
// ((x - cx) / S_x d, (cy - y) / S_y d, d) -> 0x4126b0 by the viewport camera node's matrix (+0xd4: the real, pitched camera),
// while x comes from the band's LEVEL projection (0x414b00, Projector above). So the top lands on the band icon, the foot
// on the icon's ground point, and the beam leans when the camera pitches: the foot's screen x is the band x times Z / z_v
// (Z its level depth, z_v its depth in the pitched view), so it sits nearer the centre when the camera looks up at a foot
// above it (ref-video m_03 / m_04) and farther out when it looks down at a foot below it (minimap-fix beamB_0001); upright
// for a level camera or an icon at the centre (m_14). When the icon's ground projects above the band (camera pitched down,
// icon higher than it), the band point at depth d lies below that ground and the strip runs from the band up off the
// screen: no beam on screen (FUEL alike).
std::vector<Menu::NavBeam> Menu::navBeams(const HudState& hud) {
    std::vector<NavBeam> out;
    int chevrons = 0;
    const std::vector<NavEntry> list = navEntries(hud, chevrons);
    if (list.empty()) return out;
    if (hudMem_.pp.empty()) hudMem_.pp = hud::loadPp(dataDir_ + "/CommonComponent/Param/pp001.tsc");
    auto PP = [&](int n) { return size_t(n) < hudMem_.pp.size() ? hudMem_.pp[size_t(n)] * 0.0019607844f : 0.0f; };  // x 1/510
    const std::array<float, 3> rgb[5] = {{PP(2), PP(3), PP(4)}, {PP(5), PP(6), PP(7)}, {PP(11), PP(12), PP(13)}, {0.3f, 0.3f, 0.3f}, {PP(17), PP(18), PP(19)}};
    for (const NavEntry& e : list) {
        const float dx = e.wx - hud.eye[0], dz = e.wz - hud.eye[2];
        const float d = std::sqrt(dx * dx + dz * dz);
        const float gy = ground_ ? ground_(e.wx, e.wz) : e.wy;
        const float ux = (e.x - 640.0f) / hud.camFy, uy = (360.0f - hud::kBandY) / hud.camFy;
        std::array<float, 3> top{}, bottom{e.wx, gy, e.wz};
        for (int k = 0; k < 3; ++k) top[size_t(k)] = hud.eye[size_t(k)] + d * (hud.camF[size_t(k)] + ux * hud.camR[size_t(k)] + uy * hud.camU[size_t(k)]);
        NavBeam b;
        for (int i = 0; i < 4; ++i) {
            const float t = float(i) / 3.0f;
            for (int k = 0; k < 3; ++k) b.points[size_t(i)][size_t(k)] = bottom[size_t(k)] - (bottom[size_t(k)] - top[size_t(k)]) * t;
        }
        const auto& c = rgb[std::clamp(e.colour, 0, 4)];
        // minimap-fix: the colour records' alpha [O asm 0x4c0358..0x4c03ec]: 1 (fld1) for the PP colours, 0.3 for the map
        // target's grey (0x9dbfa4 stored in all four components).
        b.colour = {c[0] * 8.0f, c[1] * 8.0f, c[2] * 8.0f, e.colour == 3 ? 0.3f : 1.0f};
        b.halfWidth = d * 0.008f;
        out.push_back(b);
    }
    return out;
}

void Menu::drawNavigator(UiRenderer& r, const HudState& hud) {
    if (!hud.gps3d || hud.camFy <= 0) return;
    NavMemory& nm = navMem_;
    const Projector proj{hud};
    const float S = 1.0f;  // +0x24 = W / 1280 (canvas units)
    // ---- Compass 0x4c1120 ----
    if (const BitmapRef bar = assets_->image("GPS_BARETTE")) {
        const float hf = std::hypot(hud.camF[0], hud.camF[2]);
        const int start = hud::compassStart(hf > 0 ? hud.camF[0] / hf : 0, hf > 0 ? hud.camF[2] / hf : -1);
        const float t4 = 720.0f * 0.035f * 0.2f;  // half the tick height (H x +0x7c x 0.2)
        TextStyle st;
        st.font = 0;
        st.height = 0.8f * S * assets_->font(0).refHeight;  // text scale 0.8 S (0x6090e0)
        for (int a = start; a < start + 162; a += 3) {
            const int deg = a > 359 ? a - 360 : a;
            const float rad = float(deg) * 0.017453292f;
            float sx, depth;
            if (!proj(hud.eye[0] - std::sin(rad) * 1e7f, hud.eye[1], hud.eye[2] - std::cos(rad) * 1e7f, sx, depth)) continue;
            const float fade = hud::edgeFade(sx);
            if (fade < 1e-6f) continue;
            const char* label = hud::compassLabel(deg);
            float y0 = hud::kBandY - t4, y1 = hud::kBandY + t4;
            if (label) {
                y0 = hud::kBandY - 2 * t4, y1 = hud::kBandY + 2 * t4;
                // Label under the long tick: x - width / 2, y + 2 t4 + 0.25 x text height, white with a black shadow (flag 2)
                // [port: shadow 1 px down-right; FUEL passes no edge fade to the label].
                const float w = r.textWidth(label, st);
                const float ty = y1 + 0.25f * st.height;
                st.fill = Rgba{0, 0, 0, 1};
                r.text(label, sx - w * 0.5f + 1, ty + 1, 0, st);
                st.fill = Rgba{1, 1, 1, 1};
                r.text(label, sx - w * 0.5f, ty, 0, st);
            }
            // Tick: GPS_BARETTE (+0xc30), 2 S wide from x, colour (0.5, 0.5, 0.5, 1) [no fade on the tick either].
            r.image(bar, sx, y0, 2 * S, y1 - y0, Rgba{1, 1, 1, 1});
        }
    }
    // ---- Icon list (0x4c0010 -> 0x4c0a40 / 0x4c0260) ----
    if (hud.helico) return;  // 0x4c0ed0: race scenario 0xb -> mode 0
    int chevrons = 0;  // 0x12 left, 0x13 right
    std::vector<NavEntry> list = navEntries(hud, chevrons);  // PK19: shared with the beams (navBeams)
    if (chevrons == 0) nm.chevronN = -1;
    // Far first (0x4c2230 inserts before the first nearer entry), so the nearest is drawn on top.
    std::stable_sort(list.begin(), list.end(), [](const NavEntry& a, const NavEntry& b) { return a.d2 > b.d2; });
    for (const NavEntry& e : list) {
        const BitmapRef b = assets_->image(bandIcon(e.type) ? bandIcon(e.type) : "");
        const float a = hud::edgeFade(e.x);
        if (!b || a < 1e-6f) continue;
        const float s = 32.0f * S;
        r.image(b, e.x - s * 0.5f, hud::kBandY - s * 0.5f, s, s, Rgba{1, 1, 1, a});
    }
    // Chevrons 0x4c1bf0 (FX_GPS_ARROW, colour PP2..PP4 / 510 of 0x4c0a40, i.e. / 255 in the port).
    if (chevrons) {
        hud::chevronStep(nm.chevronN, nm.chevronClock, hud.dt);
        const BitmapRef b = assets_->image("FX_GPS_ARROW");
        if (hudMem_.pp.empty()) hudMem_.pp = hud::loadPp(dataDir_ + "/CommonComponent/Param/pp001.tsc");
        auto PP = [&](int n) { return size_t(n) < hudMem_.pp.size() ? hudMem_.pp[size_t(n)] / 255.0f : 1.0f; };
        const float s = 24.0f * S;
        const bool right = chevrons == 0x13;
        for (int i = 0; i <= nm.chevronN && b; ++i) {
            const Rgba c{PP(2), PP(3), PP(4), 1.0f - 0.3f * float(i)};
            const float x = hud::chevronX(right, i, nm.chevronN, s);
            // Last argument of 0x60acf0: 0 left, 1 right [port: 1 = mirrored in u].
            if (right) r.image(b, x, hud::kBandY - s * 0.5f, s, s, c, 1, 0, 0, 1);
            else r.image(b, x, hud::kBandY - s * 0.5f, s, s, c);
        }
    }
}

}  // namespace fuel::ui
