// FUEL's in-race / free-roam HUD maths (decoded from FUEL.exe, notes/p40-hud.md): the speedometer
// (MeterHUD, vtable 0x9a545c), the race info and countdown (MissionHUD 0x9a54b4), the minimap
// (MapHUD 0x9ab75c, spec notes/p40-work/minimap.md) and the GPS road cost (0x4a1de0, notes/p40-work/gps.md).
// Pure functions: `fuelview --hudcheck` compares them with vectors from Ghidra's emulator and from the code.
// FUEL runs with the x87 rounding mode set to "down" (p21), so its float -> int stores floor.
#pragma once

#include <array>
#include <cmath>
#include <string>
#include <utility>
#include <vector>

namespace fuel {
struct SplineGraph;
}

namespace fuel::hud {

// ---- Speedometer (MeterHUD) -------------------------------------------------------------------------
// 0x4b0ba0: the dial picture COMPTEUR02..05 by the engine's top rpm (VehiclePP 324, P+0x50c):
// <= 5000 -> 2, <= 7000 -> 3, <= 8000 -> 4, above -> 5.
int meterDial(float maxRpm);
// 0x4b1260: rev needle COMPTEUR_AIGUILLE, degrees (clockwise on screen, 0 = up): -100 + 180 x rpm / maxRpm,
// held at +80 at or above maxRpm. Drawn by 0x60ae00 -> 0x60b490 (cos/sin of deg x 0.017453292).
float revNeedleDeg(float rpm, float maxRpm);
// 0x4b1330: damage needle DEGAT_AIGUILLE: -130 + 230 x damage (MeterHUD+0x60 = 0x4235b0 = the largest of
// the 10 damage values 0x41ff30, refreshed every 16 frames).
float damageNeedleDeg(float damage);
// Needle placement on the dial item (x, y, w, h) (0x4b0ed0 / 0x4b1090): pivot and square sprite size
// in canvas units (texture width x 0.8 / x 0.36), sprite pivot at its bottom centre (0.5, 1.0).
struct Needle {
    float px, py, size;
};
Needle revNeedle(float x, float y, float w, float h, float texWidth);
Needle damageNeedle(float x, float y, float w, float h, float texWidth);
// 0x4b0b30: the number shown: trunc(|v| x 1.15 x 3.6) km/h (x 0.6213712 in mph), |v| = the 3-D speed of
// the vehicle body (0x4222f0: |veh+0x820| x 1.15). FUEL's speedometer reads 15 % high.
int speedReadout(float speedMs, bool miles);
// 0x4b0d30: odometer (game+0x3408.. total metres): the big counter (km or miles) and the tenth digit.
void odometer(double metres, bool miles, int& big, int& tenth);

// ---- Race info (MissionHUD) -------------------------------------------------------------------------
// 0x484690: seconds -> h, min, s, 1/100 s (each floor, mod 60 / 100).
struct Clock {
    int h = 0, m = 0, s = 0, cs = 0;
};
Clock splitTime(float seconds);
// The page MissionHUD fills (0x4b4530), by FUEL's scenario (race +0x27c) and, for scenario 9, the EMD race
// mode (+0x280). Page index in InterfacesInfosPacked.tsc order.
enum class InfoPage { None, AtoB = 92, Checkpoint = 93, Circuit = 94, KnockOut = 95, TimeTrial = 96, TimeTrialCheckpoint = 97, CatchAI = 98, Helico = 99 };
InfoPage infoPage(int scenario, int emdMode);
// What TOP_INFO / BOTTOM_INFO show on each page (0x4b4530):
//   AtoB:      TOP remaining distance "%d.%03d km" (TT 1315 / mi 1319), BOTTOM race time "%02d:%02d:%02d:%02d" (TT 1313)
//   Checkpoint: TOP gates passed / gates (TT 1305 "%02d/%02d"), BOTTOM race time
//   Circuit:   TOP lap / laps, BOTTOM race time
//   KnockOut:  TOP gates passed / gates, BOTTOM "OPPONENTS" a / b
//   TimeTrial: TOP remaining distance, BOTTOM time left, red when < 10 % of the limit (0x4b4e00 flag 0)
//   TimeTrialCheckpoint: TOP gates, BOTTOM time left, red when < 5 s (flag 1)
//   CatchAI:   TOP caught / to catch (race +0x354 / +0x358), BOTTOM time
//   Helico:    TOP distance only
bool timeWarning(InfoPage p, float timeLeft, float timeTotal);
std::string distanceText(float metres, bool miles);  // 0x4b4d00 / 0x4b4d80 with TT 1315 / 1319

// 0x4b3da0 (from 0x4b4180): the start countdown. c = 3 - the race-state timer while counting down (state 4),
// then -timer for 2 s once running (state 5). Shown for -2 < c <= 3 (and c >= -2.0 @0xa2a56c):
// c > 0 -> the number trunc(c + 1) ("3", "2", "1"; sound 0xf7 when it changes), else TT 5907 "GO!" (sound 0xf8).
// Returns -1 = nothing, 0 = GO!, n > 0 = the number.
int countdown(float c);

// 0x4b95b0: the four ranking lines (page classement): line 1 = the leader, then the racer ahead, the player,
// the racer behind. `pos` = player's position (1-based), `n` = racers. For each line k = 0..3: the position shown
// there (0 = line hidden). The player's line is the "enabled" one (RANKING_PLAYER).
// Distances: the player's own line shows "%d m" (its distance), others "- %d m" (ahead) / "+ %d m" (behind):
// the absolute difference to the player.
std::array<int, 4> rankingLines(int pos, int n, int& playerLine);

// ---- Minimap (MapHUD) -------------------------------------------------------------------------------
// 0x49f690: zoom target r (centre -> rect corner, metres) = PP22 + (PP23 - PP22) x min(v / 100, 1),
// v = |veh+0x820| x 1.15 (0x4222f0). PP22 = 1000, PP23 = 4000 (CommonComponent/Param/pp001.tsc).
float zoomTarget(float speedMs, float pp22, float pp23);
// 0x49f9b0, once per HUD update: r = (15 r + previous target) / 16.
float zoomStep(float r, float previousTarget);
int mapLod(float r);            // 0x49f920 + caller: 0 (<= 9500) .. 3 (<= 35500), 4 above
float mapTolerance(float r);    // 0x49f980: trunc(r x 0.0025f) x 12 m
float mapFade(int cls, float r);  // 0x49f860: road class alpha by zoom
// The view of one frame (0x4a1020): rect (canvas) and zoom r -> screen px per metre ppm = |(w, h)| / 2r,
// heading h = normalize_xz(q x (-1, 0, 0)) of the player's node; screen = C + ppm (d.h, d.z h.x - d.x h.z)
// with d = W - P: heading-up, the vehicle's forward points up.
struct MapView {
    float cx = 0, cy = 0, ppm = 0;
    float px = 0, pz = 0;  // player
    float hx = -1, hz = 0;  // h
    std::array<float, 2> toScreen(float x, float z) const {
        const float dx = x - px, dz = z - pz;
        return {cx + ppm * (dx * hx + dz * hz), cy + ppm * (dz * hx - dx * hz)};
    }
    // A world direction (fx, fz) on screen (0x4ad500), not normalised.
    std::array<float, 2> dirToScreen(float fx, float fz) const { return {fx * hx + fz * hz, fz * hx - fx * hz}; }
};
// h = normalize_xz(q x (-1, 0, 0)) of the player's quaternion (x, y, z, w) (0x410510 = q v q^-1);
// returns false (h unchanged) when |h_xz|^2 <= 1e-6, as FUEL keeps the previous one.
bool mapHeading(const std::array<float, 4>& q, float& hx, float& hz);
MapView mapView(float x, float y, float w, float h, float r, float playerX, float playerZ, float hx, float hz);
// Road strip half-width in screen px (0x49efe0): max(0.8 S Wm ppm, 0.8 S Mpx) by road type.
float roadHalfWidth(int type, float ppm, float S);
// Douglas-free decimation of 0x4a0100: keep a point when it is farther than tol from the last kept one
// (first and last always kept).
std::vector<std::array<float, 2>> simplify(const std::vector<std::array<float, 2>>& pts, float tol);
// GPS_ONDE pulse on the next gate (0x4aba90): T advances with dt and restarts when cos 2T <= 0 (period pi/4 s);
// size = 14 (1 + 2 (1 - cos 2T)), alpha = max(cos 2T, 0).
float pulseStep(float T, float dt);

// ---- minimap-fix: the radar banner's status line (boussole ITEM_ID_STATUS, 0x4a54b0 [O asm 0x4a5680..0x4a5779]) ------
// The list box's entries (boussole.tsc): 0 TT 960 "In career race", 1 959 online free ride, 2 958 "In offline free
// ride", 3 997 online race, 4 5988 "In challenge", 5 5996 "Trying Edited Race" (the script's start index).
// MapHUD +0x1ae4 = the index, +0x1ae8 = its timer, +0x1ae0 = the index shown last (ctor 0x49eea0: 2, 0, 6). Offline
// (g_NGSM 0xa6538c exists, its +0x3c = 0): the current mission (0x62abd0) in state 4 PREPARING / 5 RUNNING (0x633170 and
// +0x48) -> by 0x577d70 (the first hub list holding the mission +0x294: 0 tutorial, 1 career, 2 challenge, 3 none):
// 1 -> 0, 2 -> 4, else 5, and the timer = 0; otherwise the timer += dt and past 5 s -> index 2, timer 0. The mission kill
// 0x62baa0 (offline) sets index 2 and the timer 5.0 (bannerKilled). Returns true when the index differs from the one
// shown last (+0x1ae0, then updated): that restarts the GROUP_FADE timer +0x1ad8 like a hub change does.
struct BannerStatus {
    int index = 2;    // +0x1ae4
    float timer = 0;  // +0x1ae8
    int shown = 6;    // +0x1ae0
};
bool bannerStatusStep(BannerStatus& s, int missionState, int missionList, float dt);
inline void bannerKilled(BannerStatus& s) { s.index = 2, s.timer = 5.0f; }

// ---- PK19: the big map's terrain picture (CLIPMAPS tile pyramid) ---------------------------------------
// 0x4ae7d0 (MapHUD / InGameMapHUD / EditorHUD init) builds MapHUD +0x19bc once: 0x6082e0 rect (-70000, -70000) ..
// (70000, 70000), size 140000, half 70000; 0x608370 five levels (0x64 B, 0x607d20): level n = 2^n x 2^n tiles of
// 140000 / 2^n m (mask 2^n - 1, shift n); 0x4ae7d0 names every level "CLIPMAPS_" with its first texture number +0x54 =
// 378 + sum(4^k, k < n) = 378, 379, 383, 399, 463 (0x40f970 pow(4, k)).
constexpr int kMapTileLevels = 5;
constexpr float kMapTileHalf = 70000.0f, kMapTileSize = 140000.0f;
int mapTileBase(int level);                                                // 378, 379, 383, 399, 463
// 0x607ee0: tile index -> world rect, min = ((idx & mask), idx >> shift) x cell + rect min, max = min + cell.
void mapTileRect(int level, int index, float& x0, float& z0, float& x1, float& z1);
// 0x607f40: world (x, z) -> tile index = dim x min(trunc((z + half) / h), mask) + min(trunc((x + half) / w), mask)
// (fistp with chop, unsigned compare: a negative quotient also gives `mask`).
int mapTileIndex(int level, float x, float z);
// 0x607fd0: the tiles under the square (cx, cz) +- r: the square is clipped to the pyramid rect on the side that
// leaves it (the far bound stays unclipped when the near one was clipped, as FUEL does), then sampled from the near
// corner in steps of min(cell, r) while <= the far bound; each new index is appended once (array order).
std::vector<int> mapVisibleTiles(int level, float cx, float cz, float r);
// 0x4ae930: d = sqrt(2 zoom^2) (zoom = M+0x19e8); L = how many of 175100 x 0.5^k (k = 0 .. levels - 2) d stays
// within, counted until the first one it exceeds. 0x608560 then draws level 0, L - 1 (when >= 1) and L (when >= 1),
// each over the tiles of (centre, d) (0x607fd0).
float mapTileRadius(float zoom);
int mapTileLevel(float zoom);
struct MapTile {
    int level, index, texture;  // texture = the number of "CLIPMAPS_%d" (0x608560: "%s%d" of the level name, base + index)
    float x0, z0, x1, z1;
};
std::vector<MapTile> mapTiles(float zoom, float cx, float cz);

// ---- PK19: the race line of the minimap (0x4a1bd0 -> 0x629990) ------------------------------------------
// 0x629990(path id, path manager, out[1000]): only when the path record's node (+0x3c) is a live SplineGraph_Z (the
// -CT world courses); for each spline block (kind +4 == 5, list +0x24 / next +0x20) in path order, its edge list (+0xc0
// u16: index & 0x7fff, bit 15 = walked backwards; count +0x3e): the first edge ever met gives its start point (+0x10;
// backwards: its end +0xe0); an edge equal to that first one ends the line (the loop closed); every edge then adds a
// point per 10 m of summed segment length (+0x28 of each 0x1c-byte segment, reset when a point is emitted and at each
// block): the segment's far end (forwards p1 +0x1c, segments 0..7; backwards p0 +0x10, segments 7..0); at most 1000
// points; a block without edges before the first edge stops the walk. (x, z) only.
std::vector<std::array<float, 2>> circuitLine(const fuel::SplineGraph& g, const std::vector<std::vector<std::pair<int, bool>>>& blocks);

// ---- Navigator3dHUD screen band (vtable 0x9a55bc, draw 0x4c0fe0; P56) ---------------------------------
// The band: init 0x4bf680 +0x70 = 0 (x origin), +0x78 = 1 (x scale), +0x74 = (1 - s) / 2 with s = the viewport's
// safe fraction +0x14 = 0.9 (InGamePlayer ctor 0x443980 stores 0x3f666666 in every HUD), +0x7c = 0.035 (height).
// Centre y = (0.035 / 2 + 0.05) x H; screen x of a world point = (1 x projected x + 0) x W (0x4c1030).
constexpr float kBandY = (0.035f * 0.5f + 0.05f) * 720.0f;  // canvas units
constexpr float kBandH = 0.035f * 720.0f;
// 0x4c1120 compass: h = -deg(atan2(-f.x, -f.z)) of the camera vector (camera +0x838, x 0x9dcf4c = -57.2958),
// truncated (_ftol 0x8e01d0); ticks every 3 degrees from start = 3 (-27 - h / 3) (+360 when < 0) over 162 (0xa2)
// degrees. Tick angle a points at world (-sin a, 0, -cos a): N at 0 (-z), NW 45, W 90 (-x), SW 135, S 180,
// SE 225, E 270, NE 315 (strings 0x9a5870 ...).
int compassStart(float camFx, float camFz);
const char* compassLabel(int deg);  // null between the eight labels
// Fade to the screen sides (0x4c1e60, 0x4c1120): 1 - |W/2 - x| / (W/2).
float edgeFade(float x, float W = 1280.0f);
// 0x4c10c0: -1 when x < -10 S, 1 when x > W + 10 S, else 0 (the icon is on screen).
int offScreen(float x, float W = 1280.0f, float S = 1.0f);
// 0x4c1bf0 chevrons (types 0x12 left, 0x13 right): +0x84 = n steps every 0.1 s through 0, 1, 2 (+0x88 clock,
// n < 0 restarts at 0); n + 1 chevrons of s = 24 S, alpha 1 - 0.3 i; left x_i = vx + 1.1 s + 1.1 s (i - n + 2) - s/2,
// right x_i = vx + vw - 3.3 s - s/2 + 1.1 s (n - i); top y = band centre - s/2. vx, vw = viewport (0.05 W, 0.9 W).
void chevronStep(int& n, float& clock, float dt);
float chevronX(bool right, int i, int n, float s = 24.0f, float vx = 64.0f, float vw = 1152.0f);

// CommonComponent/Param/pp001.tsc ("PP n value", array at *0xa6749c, loader 0x66b3e0): rows 0..5999, missing = 0.
std::vector<float> loadPp(const std::string& pp001Path);

// --hudcheck: the vectors (emulated in Ghidra or derived from the code) against these functions; prints and
// returns the number of failures. `pp` = loadPp of the player's pp001.tsc (empty: those rows are skipped).
// ---- P102: VignettageHUD (ctor 0x4cad10 at InGamePlayer +0x4830, draw 0x4caef0 -> 0x4cc0f0).
struct Rect { float x, y, w, h; };
// 0x4cc0f0: the VIGNETTAGE picture from (0, 0) at the viewport's own width and height, i.e. the whole target, given in
// the UI canvas units of a target whose left / top edge and size are (left, top, w, h) (UiRenderer::view*).
Rect vignetteRect(float left, float top, float w, float h);
// 0x443290 draws it first, before the SetHudsVisibleState gate; skipped with DisableOSD ([0xa65420], the whole draw) or
// while a video plays (VignettageHUD vt+0x10 = 0x4454b0, the HUD's +0x33 = 0x694290(0)).
bool vignetteShown(bool osdDisabled, bool videoPlaying);

// ---- [nitro] The nitrous gauge (the port's; FUEL has none; ui/nitro_gauge.mm): a small upright bottle right of the
// compteur dial, its body inside [x0, x1] x [y0, y1] (canvas units), the neck and valve cap above. Placed from the dial
// item's rect (Menu::itemRect: FUEL's safe-frame snapping), so it follows the HUD layout; the canvas is letterboxed the
// same at 16:9 and 16:10 (UiRenderer), so it sits the same on both.
struct NitroBottle {
    float x0 = 0, y0 = 0, x1 = 0, y1 = 0;  // body
    float neckTop = 0, capTop = 0;
};
NitroBottle nitroBottle(float dialX, float dialY, float dialW, float dialH);
Rect nitroBounds(const NitroBottle& b);                  // everything drawn, the full glow included
float nitroShown(float shown, float level, float dt);    // the displayed level: exp follow, 0.12 s
float nitroSurfaceY(const NitroBottle& b, float level);  // the liquid's top for a level (inside the 2-unit wall, no wave)
float nitroFlashAlpha(float flashLeft);                  // the empty flash: two dim blinks over its 0.6 s
float nitroGlowAlpha(float level, float t);              // the glow of a full tank (0 below full), a slow pulse

int runChecks(const std::vector<float>& pp);

// ---- GPS (0x4a1de0) ---------------------------------------------------------------------------------
// Cost of a road record for the GPS detour: its length, halved on records of the current IA road block;
// the record under the player (list[0]) costs length + 300 when driven against the player's heading.
float gpsCost(float length, bool inList, bool isPlayerRecord, bool sameDirAsPlayer);

}  // namespace fuel::hud
