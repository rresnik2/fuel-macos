// FUEL's HUD maths; see hud_math.hpp and notes/p40-hud.md.
#include "ui/hud_math.hpp"
#include "ui/gps_guide.hpp"
#include "core/spline_graph.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>

namespace fuel::hud {

int meterDial(float maxRpm) {
    if (!(maxRpm > 5000.0f)) return 2;
    if (!(maxRpm > 7000.0f)) return 3;
    return maxRpm > 8000.0f ? 5 : 4;
}

float revNeedleDeg(float rpm, float maxRpm) {
    constexpr float lo = -100.0f, hi = 80.0f;  // MeterHUD +0x64 / +0x68 (0xc2c80000 / 0x42a00000)
    if (rpm <= maxRpm && maxRpm != 0.0f) return rpm * ((hi - lo) / maxRpm) + lo;
    return (hi - lo) + lo;
}

float damageNeedleDeg(float damage) {
    constexpr float lo = -130.0f, hi = 100.0f;  // +0x6c / +0x70 (0xc3020000 / 0x42c80000)
    return (hi - lo) * damage + lo;
}

Needle revNeedle(float x, float y, float w, float h, float texWidth) {
    return {x + w * 0.5796388f, y + h * 0.5f, texWidth * 0.8f};
}

Needle damageNeedle(float x, float y, float w, float h, float texWidth) {
    return {x + w * 0.17569786f, y + h * 0.8535156f, texWidth * 0.36f};
}

int speedReadout(float speedMs, bool miles) {
    float v = std::fabs(speedMs * 1.15f * 3.6f);  // 0x4222f0 x 1.15, 0x4b0b30 x 3.6 (0x9dcb38)
    if (miles) v *= 0.6213712f;                    // 0x9dc078
    return int(v);                                 // _ftol truncates
}

void odometer(double metres, bool miles, int& big, int& tenth) {
    if (miles) {
        const double mi = metres * 0.0006213712;
        big = int(mi);
        tenth = int((mi - big) * 10.0);
    } else {
        big = int(metres * 0.001);
        tenth = int((metres - big * 1000.0) * 0.01);
    }
}

Clock splitTime(float t) {
    Clock c;
    c.h = int(std::floor(t * 0.00027777778f));
    c.m = int(std::floor(t * 0.016666668f)) % 60;
    c.s = int(std::floor(t)) % 60;
    c.cs = int(std::floor(t * 100.0f)) % 100;
    return c;
}

InfoPage infoPage(int scenario, int emdMode) {
    switch (scenario) {  // jump table 0x4b4bc0 (scenario - 1), 0x4b4bf4 (EMD mode)
        case 1: case 7: return InfoPage::AtoB;
        case 2: return InfoPage::Checkpoint;
        case 3: return InfoPage::KnockOut;
        case 4: return InfoPage::TimeTrial;
        case 5: return InfoPage::TimeTrialCheckpoint;
        case 6: case 8: return InfoPage::Circuit;
        case 9:
            switch (emdMode) {
                case 0: return InfoPage::AtoB;
                case 1: return InfoPage::Checkpoint;
                case 2: return InfoPage::KnockOut;
                case 4: return InfoPage::Circuit;
                default: return InfoPage::None;
            }
        case 11: return InfoPage::Helico;
        case 13: return InfoPage::CatchAI;
        default: return InfoPage::None;
    }
}

bool timeWarning(InfoPage p, float timeLeft, float timeTotal) {
    if (p == InfoPage::TimeTrial) return timeLeft < timeTotal * 0.1f;
    if (p == InfoPage::TimeTrialCheckpoint) return timeLeft < 5.0f;
    return false;
}

std::string distanceText(float metres, bool miles) {
    char b[48];
    if (miles) {
        const float mi = metres * 0.0006213712f;  // 0x4b4d80: whole miles, thousandths
        const int whole = int(mi);
        std::snprintf(b, sizeof b, "%d.%03d mi", whole, int(std::floor((mi - float(whole)) * 1000.0f)));
    } else {
        std::snprintf(b, sizeof b, "%d.%03d km", int(std::floor(metres * 0.001f)), int(std::floor(metres)) % 1000);
    }
    return b;
}

int countdown(float c) {
    if (!(c >= -2.0f) || c > 3.0f || !(c + 2.0f > 0.0f)) return -1;
    if (c > 0.0f) return int(c + 1.0f);
    return 0;
}

std::array<int, 4> rankingLines(int p, int n, int& playerLine) {
    std::array<int, 4> pos{1, 0, 0, 0};
    if (p > 2) pos[1] = p - 1;                     // the racer ahead
    const int afterLine = p == 1 ? 1 : p == 2 ? 2 : 3;
    if (p < n) pos[size_t(afterLine)] = p + 1;     // the racer behind
    playerLine = p == 1 ? 0 : p == 2 ? 1 : 2;
    pos[size_t(playerLine)] = p;
    return pos;
}

float zoomTarget(float speedMs, float pp22, float pp23) {
    const float v = std::fabs(speedMs * 1.15f);
    return pp22 + (pp23 - pp22) * std::min(v / 100.0f, 1.0f);
}

float zoomStep(float r, float previousTarget) { return (r * 15.0f + previousTarget) * 0.0625f; }

bool bannerStatusStep(BannerStatus& s, int missionState, int missionList, float dt) {
    if (missionState == 4 || missionState == 5) {  // 0x4a56cb: +0x48 == 5 or 4 (0x633170: not 0 / 1)
        s.index = missionList == 1 ? 0 : missionList == 2 ? 4 : 5;  // 0x4a56e3: 1 -> 0, else (list != 2) + 4
        s.timer = 0;
    } else {
        s.timer += dt;  // 0x4a5702
        if (s.timer > 5.0f) s.timer = 0, s.index = 2;  // 0x4a5710 (5.0 @0x9dbf8c), 0x4a571f
    }
    if (s.index == s.shown) return false;  // 0x4a5762
    s.shown = s.index;
    return true;
}

int mapLod(float r) {
    if (r <= 9500.0f) return 0;
    if (r <= 13500.0f) return 1;
    if (r <= 20500.0f) return 2;
    if (r <= 35500.0f) return 3;
    return 4;
}

// x87 at 53-bit precision: the product keeps 2000 x 0.0025f = 4.9999998... and the store rounds down (-> 48 m).
float mapTolerance(float r) { return float(int(std::floor(double(r) * double(0.0025f)))) * 12.0f; }

float mapFade(int cls, float r) {
    float full, zero;
    switch (cls) {
        case 1: full = 4000, zero = 6500; break;
        case 2: full = 8000, zero = 10500; break;
        case 3: full = 15000, zero = 17500; break;
        case 4: full = 30000, zero = 32500; break;
        default: return 1.0f;
    }
    if (r < full) return 1.0f;
    return std::clamp((zero - r) * 0.0004f, 0.0f, 1.0f);
}

bool mapHeading(const std::array<float, 4>& q, float& hx, float& hz) {
    // v' = q v q^-1 for v = (-1, 0, 0).
    const float x = q[0], y = q[1], z = q[2], w = q[3];
    const float vx = -(1 - 2 * (y * y + z * z));
    const float vz = -(2 * (x * z - w * y));
    const float l2 = vx * vx + vz * vz;
    if (l2 <= 1e-6f) return false;
    const float k = 1.0f / std::sqrt(l2);
    hx = vx * k, hz = vz * k;
    return true;
}

MapView mapView(float x, float y, float w, float h, float r, float playerX, float playerZ, float hx, float hz) {
    MapView v;
    v.cx = x + w * 0.5f, v.cy = y + h * 0.5f;
    v.ppm = std::sqrt(w * w + h * h) / (r + r);
    v.px = playerX, v.pz = playerZ;
    v.hx = hx, v.hz = hz;
    return v;
}

float roadHalfWidth(int t, float ppm, float S) {
    float wm = 10, mpx = 2;  // 0x49efe0 (consts from 0x9dc154): metres, minimum px
    switch (t) {
        case 4: case 5: case 14: wm = 15, mpx = 2; break;
        case 6: case 7: case 8: case 9: case 13: case 16: case 20: wm = 25, mpx = 2.5f; break;
        case 10: case 11: case 12: case 15: case 17: case 19: wm = 35, mpx = 3; break;
        case 18: wm = 50, mpx = 4; break;
        default: break;
    }
    return std::max(0.8f * S * wm * ppm, 0.8f * S * mpx);
}

std::vector<std::array<float, 2>> simplify(const std::vector<std::array<float, 2>>& pts, float tol) {
    if (pts.size() <= 2 || tol <= 0) return pts;
    std::vector<std::array<float, 2>> out{pts.front()};
    const float t2 = tol * tol;
    for (size_t i = 1; i + 1 < pts.size(); ++i) {
        const float dx = pts[i][0] - out.back()[0], dz = pts[i][1] - out.back()[1];
        if (dx * dx + dz * dz > t2) out.push_back(pts[i]);
    }
    out.push_back(pts.back());
    return out;
}

// ---- PK19: CLIPMAPS tile pyramid (0x4ae7d0, 0x6082e0, 0x608370, 0x607ee0, 0x607f40, 0x607fd0, 0x4ae930) ----
int mapTileBase(int level) {
    int base = 378;  // 0x4ae7d0: 0x17a, + round(pow(4, k)) after each level
    for (int k = 0; k < level; ++k) base += 1 << (2 * k);
    return base;
}

void mapTileRect(int level, int index, float& x0, float& z0, float& x1, float& z1) {
    const unsigned dim = 1u << level, mask = dim - 1;
    const float cell = kMapTileSize / float(dim);  // 0x608370: size x (1 / dim)
    x0 = float(unsigned(index) & mask) * cell - kMapTileHalf;
    z0 = float(unsigned(index) >> level) * cell - kMapTileHalf;
    x1 = x0 + cell, z1 = z0 + cell;
}

int mapTileIndex(int level, float x, float z) {
    const unsigned dim = 1u << level, mask = dim - 1;
    const float cell = kMapTileSize / float(dim);
    auto chop = [&](float v) {  // fistp under RC chop, then an unsigned compare against the mask
        const long long q = (long long)((kMapTileHalf + v) / cell);
        const unsigned u = unsigned(uint32_t(q));
        return u > mask ? mask : u;
    };
    return int(dim * chop(z) + chop(x));
}

std::vector<int> mapVisibleTiles(int level, float cx, float cz, float r) {
    std::vector<int> out;
    const float cell = kMapTileSize / float(1u << level);
    float x0 = cx - r, x1 = cx + r, z0 = cz - r, z1 = cz + r;
    if (-kMapTileHalf <= x0) x1 = std::min(x1, kMapTileHalf);
    else x0 = -kMapTileHalf;
    if (-kMapTileHalf <= z0) z1 = std::min(z1, kMapTileHalf);
    else z0 = -kMapTileHalf;
    const float sx = std::min(cell, r), sz = std::min(cell, r);
    if (!(sx > 0) || !(sz > 0)) return out;
    for (float z = z0; z <= z1; z += sz)
        for (float x = x0; x <= x1; x += sx) {
            const int t = mapTileIndex(level, x, z);
            if (std::find(out.begin(), out.end(), t) == out.end()) out.push_back(t);
        }
    return out;
}

float mapTileRadius(float zoom) {
    const float z2 = zoom * zoom;
    return std::sqrt(z2 + z2);
}

int mapTileLevel(float zoom) {
    const double d = std::sqrt(double(float(zoom * zoom + zoom * zoom)));  // x87: the comparisons use the unrounded root
    double t = 175100.0;  // 0x9dcd0c
    int L = 0;
    for (int k = 0; k < kMapTileLevels - 1 && d <= t; ++k, t *= 0.5) ++L;
    return L;
}

std::vector<MapTile> mapTiles(float zoom, float cx, float cz) {
    std::vector<MapTile> out;
    const int L = mapTileLevel(zoom);
    const float r = mapTileRadius(zoom);
    std::vector<int> levels{0};
    if (L - 1 >= 1) levels.push_back(L - 1);
    if (L >= 1) levels.push_back(L);
    for (int lv : levels)
        for (int idx : mapVisibleTiles(lv, cx, cz, r)) {
            MapTile t{lv, idx, mapTileBase(lv) + idx, 0, 0, 0, 0};
            mapTileRect(lv, idx, t.x0, t.z0, t.x1, t.z1);
            out.push_back(t);
        }
    return out;
}

std::vector<std::array<float, 2>> circuitLine(const fuel::SplineGraph& g, const std::vector<std::vector<std::pair<int, bool>>>& blocks) {
    std::vector<std::array<float, 2>> out;
    int first = -1;  // uVar11 (local_18 after the first edge)
    for (const auto& block : blocks) {
        if (!block.empty()) {
            float acc = 0;
            for (const auto& [e, fwd] : block) {
                if (e < 0 || size_t(e) >= g.edges.size()) return out;
                const auto& E = g.edges[size_t(e)];
                if (first < 0) {
                    const auto& p = fwd ? E.start() : E.end();
                    out.push_back({p[0], p[2]});
                    if (out.size() > 999) return out;
                } else if (first == e) {
                    return out;
                }
                for (int i = 0; i < 8; ++i) {
                    const int k = fwd ? i : 7 - i;
                    acc += E.seg[size_t(k)].len;
                    if (10.0f <= acc) {
                        const auto& p = fwd ? E.seg[size_t(k)].p1 : E.seg[size_t(k)].p0;
                        out.push_back({p[0], p[2]});
                        acc = 0;
                        if (out.size() > 999) return out;
                    }
                }
                if (first < 0) first = e;
            }
        }
        if (first < 0) return out;
    }
    return out;
}

float pulseStep(float T, float dt) {
    if (std::cos(2.0f * T) <= 0.0f) T = 0;
    return T + dt;
}

int compassStart(float camFx, float camFz) {
    const int h = int(std::atan2(-camFx, -camFz) * -57.2957763671875f);  // 0x40f550 x 0x9dcf4c, _ftol
    int s = (-27 - h / 3) * 3;                                             // imul 0x55555556: truncating / 3
    if (s < 0) s += 360;
    return s;
}

const char* compassLabel(int deg) {
    switch (deg) {
        case 0: return "N";     // 0x9a5870
        case 45: return "NW";   // 0x9abf4c
        case 90: return "W";    // 0x9a584c
        case 135: return "SW";  // 0x9abf50
        case 180: return "S";   // 0x9a585c
        case 225: return "SE";  // 0x9abf54
        case 270: return "E";   // 0x9a5894
        case 315: return "NE";  // 0x9abf58
        default: return nullptr;
    }
}

float edgeFade(float x, float W) { return 1.0f - std::fabs((W * 0.5f - x) * (1.0f / (W * 0.5f))); }

int offScreen(float x, float W, float S) {
    if (x < S * -10.0f) return -1;
    if (S * 10.0f + W < x) return 1;
    return 0;
}

void chevronStep(int& n, float& clock, float dt) {
    if (n < 0) n = 0, clock = 0;
    clock += dt;
    if (clock > 0.1f) {
        if (++n > 2) n = 0;
        clock = 0;
    }
}

float chevronX(bool right, int i, int n, float s, float vx, float vw) {
    if (!right) return s * 1.1f * float(i - n + 2) + (vx + s * 1.1f - s * 0.5f);
    return float(n - i) * s * 1.1f + ((vw + vx - s * 3.3000002f) - s * 0.5f);
}

float gpsCost(float length, bool inList, bool isPlayerRecord, bool sameDirAsPlayer) {
    if (isPlayerRecord) return sameDirAsPlayer ? length * 0.5f : length + 300.0f;
    return inList ? length * 0.5f : length;
}

std::vector<float> loadPp(const std::string& path) {
    std::vector<float> pp;
    std::ifstream f(path, std::ios::binary);
    std::string line;
    while (std::getline(f, line)) {
        std::istringstream ls(line);
        std::string tag;
        int n = -1;
        float v = 0;
        if (ls >> tag >> n >> v && tag == "PP" && n >= 0 && n < 6000) {
            if (pp.size() < 6000) pp.resize(6000, 0.0f);
            pp[size_t(n)] = v;
        }
    }
    return pp;
}

Rect vignetteRect(float left, float top, float w, float h) { return Rect{left, top, w, h}; }
bool vignetteShown(bool osdDisabled, bool videoPlaying) { return !osdDisabled && !videoPlaying; }

// ---- [nitro] the gauge's geometry and animation (hud_math.hpp)
NitroBottle nitroBottle(float dx, float dy, float dw, float dh) {
    NitroBottle b;
    b.x0 = dx + dw + 8.0f, b.x1 = b.x0 + 18.0f;  // right of the dial picture, 8 units clear
    b.y1 = dy + dh - 12.0f, b.y0 = b.y1 - 112.0f;  // its foot level with the damage gauge's
    b.neckTop = b.y0 - 7.0f, b.capTop = b.neckTop - 5.0f;
    return b;
}
Rect nitroBounds(const NitroBottle& b) { return Rect{b.x0 - 6.0f, b.capTop - 6.0f, b.x1 - b.x0 + 12.0f, b.y1 - b.capTop + 12.0f}; }
float nitroShown(float shown, float level, float dt) {
    if (!(dt > 0)) return shown;
    return shown + (level - shown) * (1.0f - std::exp(-dt / 0.12f));
}
float nitroSurfaceY(const NitroBottle& b, float level) {
    const float lo = b.y1 - 2.0f, hi = b.y0 + 2.0f;
    return lo - std::clamp(level, 0.0f, 1.0f) * (lo - hi);
}
float nitroFlashAlpha(float left) {
    if (!(left > 0)) return 0;
    const float ph = std::clamp((0.6f - left) / 0.6f, 0.0f, 1.0f);
    const float s = std::sin(ph * 6.2831853f);
    return 0.5f * s * s * (1.0f - 0.4f * ph);
}
float nitroGlowAlpha(float level, float t) { return level >= 0.999f ? 0.22f + 0.10f * std::sin(t * 2.6f) : 0.0f; }

int runChecks(const std::vector<float>& pp) {
    int fails = 0, total = 0;
    auto near = [&](const char* what, double got, double want, double tol = 1e-3) {
        ++total;
        const bool ok = std::fabs(got - want) <= tol * std::max(1.0, std::fabs(want));
        if (!ok) ++fails;
        std::printf("  %-4s %-52s got %.6g want %.6g\n", ok ? "ok" : "FAIL", what, got, want);
    };
    std::printf("hudcheck: emulated (Ghidra EmuRun, notes/p40-work/minimap_emu.cmd, gpscost.cmd)\n");
    // 0x49f690 with PP22 = 1000, PP23 = 4000; the emulator was given v directly (= speed x 1.15).
    for (auto [v, want] : std::vector<std::pair<float, float>>{{0, 1000}, {50, 2500}, {99.9f, 3997}, {250, 4000}})
        near("0x49f690 zoom target", zoomTarget(v / 1.15f, 1000, 4000), want);
    for (auto [r, want] : std::vector<std::pair<float, int>>{{9500, 0}, {9501, 1}, {20000, 2}, {40000, 4}})
        near("0x49f920 LOD (+ caller: > 35500 -> 4)", mapLod(r), want, 0);
    for (auto [r, want] : std::vector<std::pair<float, float>>{{1000, 24}, {2000, 48}, {4000, 108}, {399, 0}})
        near("0x49f980 simplification tolerance", mapTolerance(r), want);
    struct F { int c; float r, want; };
    for (const F& f : std::vector<F>{{1, 3000, 1}, {1, 5250, 0.5f}, {2, 9000, 0.6f}, {4, 32000, 0.2f}, {1, 7000, 0}})
        near("0x49f860 road fade", mapFade(f.c, f.r), f.want);
    // 0x4a1de0 on a 20 m record: list empty / in list / player record same dir / against / not in list.
    near("0x4a1de0 GPS cost, empty list", gpsCost(20, false, false, true), 20);
    near("0x4a1de0 GPS cost, in list", gpsCost(20, true, false, true), 10);
    near("0x4a1de0 GPS cost, player record same dir", gpsCost(20, true, true, true), 10);
    near("0x4a1de0 GPS cost, player record against", gpsCost(20, true, true, false), 320);
    near("0x4a1de0 GPS cost, other record", gpsCost(20, false, false, false), 20);
    near("0x4a1de0 GPS cost, partial 10 m same dir", gpsCost(10, true, true, true), 5);
    // 0x410510 (q v q^-1) for yaw 90: h = q x (-1, 0, 0) = (0, 0, 1).
    {
        float hx = 0, hz = 0;
        mapHeading({0, 0.70710678f, 0, 0.70710678f}, hx, hz);
        near("0x410510 heading x (yaw 90)", hx, 0, 1e-5);
        near("0x410510 heading z (yaw 90)", hz, 1, 1e-5);
    }
    std::printf("hudcheck: derived from the code (hand vectors, notes/p40-hud.md)\n");
    // Minimap transform (minimap.md 2): R = (-14, 485, 245, 245), r = 1000.
    {
        const MapView v0 = mapView(-14, 485, 245, 245, 1000, 0, 0, -1, 0);
        near("transform ppm", v0.ppm, 0.1732412, 1e-5);
        near("transform yaw 0, d (100, 0) -> X", v0.toScreen(100, 0)[0], 91.18, 1e-3);
        near("transform yaw 0, d (0, 100) -> Y (up)", v0.toScreen(0, 100)[1], 590.18, 1e-3);
        const MapView v1 = mapView(-14, 485, 245, 245, 1000, 0, 0, 0, 1);
        near("transform yaw 90, forward (100, 0) -> Y (up)", v1.toScreen(100, 0)[1], 590.18, 1e-3);
        near("transform yaw 90, d (0, 100) -> X", v1.toScreen(0, 100)[0], 125.82, 1e-3);
    }
    near("0x49f9b0 zoom step (2000 -> 4000)", zoomStep(2000, 4000), 2125);
    // Speedometer (MeterHUD).
    near("0x4b0ba0 dial, top rpm 4600", meterDial(4600), 2, 0);
    near("0x4b0ba0 dial, top rpm 7000", meterDial(7000), 3, 0);
    near("0x4b0ba0 dial, top rpm 8000", meterDial(8000), 4, 0);
    near("0x4b0ba0 dial, top rpm 9000", meterDial(9000), 5, 0);
    near("0x4b1260 rev needle 0 rpm", revNeedleDeg(0, 7000), -100);
    near("0x4b1260 rev needle 3500 / 7000", revNeedleDeg(3500, 7000), -10);
    near("0x4b1260 rev needle over the top", revNeedleDeg(7400, 7000), 80);
    near("0x4b1330 damage needle 0 / 0.5 / 1", damageNeedleDeg(0) + damageNeedleDeg(0.5f) * 1000 + damageNeedleDeg(1) * 1e6, -130 - 15000 + 100e6);
    near("0x4b0ed0 rev pivot x on (1020, 500, 260, 220)", revNeedle(1020, 500, 260, 220, 128).px, 1170.706);
    near("0x4b1090 damage pivot y", damageNeedle(1020, 500, 260, 220, 64).py, 687.7734);
    near("0x4b0b30 speed 20 m/s (km/h)", speedReadout(20, false), 82, 0);
    near("0x4b0b30 speed 20 m/s (mph)", speedReadout(20, true), 51, 0);
    {
        int a = 0, b = 0;
        odometer(12345.6, false, a, b);
        near("0x4b0d30 odometer km", a, 12, 0);
        near("0x4b0d30 odometer 100 m digit", b, 3, 0);
        odometer(12345.6, true, a, b);
        near("0x4b0d30 odometer miles", a, 7, 0);
        near("0x4b0d30 odometer tenth mile", b, 6, 0);
    }
    {
        const Clock c = splitTime(3725.678f);
        near("0x484690 time h:m:s:cs", c.h * 1000000 + c.m * 10000 + c.s * 100 + c.cs, 1020567, 0);
    }
    near("0x4b3da0 countdown c = 2.5", countdown(2.5f), 3, 0);
    near("0x4b3da0 countdown c = 0.2", countdown(0.2f), 1, 0);
    near("0x4b3da0 countdown c = -1 (GO!)", countdown(-1), 0, 0);
    near("0x4b3da0 countdown c = -2.5 (none)", countdown(-2.5f), -1, 0);
    {
        int pl = 0;
        const auto l = rankingLines(4, 7, pl);
        near("0x4b95b0 lines for 4th of 7", l[0] * 1000 + l[1] * 100 + l[2] * 10 + l[3], 1345, 0);
        const auto l1 = rankingLines(1, 7, pl);
        near("0x4b95b0 lines for 1st of 7", l1[0] * 1000 + l1[1] * 100 + l1[2] * 10 + l1[3], 1200, 0);
        const auto l2 = rankingLines(7, 7, pl);
        near("0x4b95b0 lines for last of 7", l2[0] * 1000 + l2[1] * 100 + l2[2] * 10 + l2[3], 1670, 0);
    }
    near("0x4b4530 page, scenario 9 mode 4", int(infoPage(9, 4)), int(InfoPage::Circuit), 0);
    near("0x4b4e00 time warning TT at 9 % left", timeWarning(InfoPage::TimeTrial, 9, 100), 1, 0);
    near("0x4bca50 ribbon fade at 25 m (L 300)", GpsGuide::fade(25, 300), 0.4995);
    near("0x4bca50 ribbon fade at 270 m (L 300)", GpsGuide::fade(270, 300), 0.6);
    near("0x4bca50 ribbon fade at 5 m", GpsGuide::fade(5, 300), 0);
    {
        bool w = false;
        near("0x4bca50 ribbon advance 299 + 40 x 0.05 wraps", GpsGuide::advance(299, 0.05f, &w), 1, 1e-4);
        near("0x4bca50 wrapped flag", w, 1, 0);
    }
    std::printf("hudcheck: Navigator3dHUD band (P56, derived from 0x4c1120 / 0x4c1bf0 / 0x4c1e60 / 0x4c10c0)\n");
    near("0x4c1120 compass start, camera to -z (N)", compassStart(0, -1), 279, 0);
    near("0x4c1120 compass start, camera to -x (W)", compassStart(-1, 0), 9, 0);
    near("0x4c1120 compass start, camera to +x (E)", compassStart(1, 0), 189, 0);
    near("0x4c1120 compass start, camera to +z (S: pi x 57.29578f = 179.99 -> 179)", compassStart(0, 1), 102, 0);
    near("0x4c1120 label 270 = E", std::string(compassLabel(270)) == "E", 1, 0);
    near("0x4c1120 no label at 30", compassLabel(30) == nullptr, 1, 0);
    near("band centre y (0.0175 + 0.05) x 720", kBandY, 48.6, 1e-4);
    near("0x4c1e60 edge fade at x = 320", edgeFade(320), 0.5, 1e-6);
    near("0x4c10c0 x = -11 -> left", offScreen(-11), -1, 0);
    near("0x4c10c0 x = 1289 -> on screen", offScreen(1289), 0, 0);
    near("0x4c1bf0 left chevron 0 of n = 2", chevronX(false, 0, 2), 64 + 26.4 - 12, 1e-4);
    near("0x4c1bf0 right chevron 0 of n = 2", chevronX(true, 0, 2), 64 + 1152 - 79.2 - 12 + 52.8, 1e-4);
    {
        int n = -1;
        float c = 0;
        chevronStep(n, c, 0.05f);
        chevronStep(n, c, 0.06f);
        near("0x4c1bf0 chevron count steps after 0.11 s", n, 1, 0);
    }
    std::printf("hudcheck: PK19 big-map tile pyramid (hand vectors from 0x4ae7d0 / 0x607ee0 / 0x607f40 / 0x607fd0 / 0x4ae930)\n");
    {
        near("0x4ae7d0 first texture of level 4 (CLIPMAPS_463)", mapTileBase(4), 463, 0);
        near("0x4ae930 level at zoom 140000 (d 197990 > 175100)", mapTileLevel(140000), 0, 0);
        near("0x4ae930 level at zoom 100000 (d 141421)", mapTileLevel(100000), 1, 0);
        near("0x4ae930 level at zoom 40000 (d 56569)", mapTileLevel(40000), 2, 0);
        near("0x4ae930 level at zoom 5000 (capped at levels - 1)", mapTileLevel(5000), 4, 0);
        near("0x607f40 level 4 tile of (0, 0)", mapTileIndex(4, 0, 0), 136, 0);
        near("0x607f40 level 2 tile of (-70000, 69999)", mapTileIndex(2, -70000, 69999), 12, 0);
        near("0x607f40 level 2 tile past the edge (clamped to the mask)", mapTileIndex(2, 80000, -71000), 3, 0);
        float x0, z0, x1, z1;
        mapTileRect(4, 136, x0, z0, x1, z1);
        near("0x607ee0 level 4 tile 136 rect x0 / z0 / x1", x0 + z0 * 1000 + x1 * 1e6, 8750e6);
        near("0x607fd0 level 1 tiles around (0, 0) r 7071", double(mapVisibleTiles(1, 0, 0, 7071).size()), 4, 0);
        near("0x607fd0 level 4 r 4000 at (0, 0): 4000 m steps", double(mapVisibleTiles(4, 0, 0, 4000).size()), 4, 0);
        near("0x4ae930 tiles drawn at zoom 5000 (levels 0, 3, 4)", double(mapTiles(5000, 0, 0).size()), 9, 0);
    }
    {
        // 0x629990 on a hand graph: edge 0 forward, 8 segments of 3 m along +x from (0, 0); edge 1 walked backwards, 8
        // segments of 5 m; then a block that meets edge 0 again (the loop closes).
        fuel::SplineGraph g;
        g.edges.resize(2);
        for (int k = 0; k < 8; ++k) {
            g.edges[0].seg[size_t(k)] = {{3.0f * k, 0, 0}, {3.0f * (k + 1), 0, 0}, 3.0f};
            g.edges[1].seg[size_t(k)] = {{24, 0, 5.0f * k}, {24, 0, 5.0f * (k + 1)}, 5.0f};
        }
        const auto L = circuitLine(g, {{{0, true}, {1, false}}, {{0, true}}});
        near("0x629990 circuit line points (start + every 10 m)", double(L.size()), 7, 0);
        near("0x629990 second point = segment 3's far end (12 m)", L.size() > 1 ? L[1][0] : -1, 12);
        near("0x629990 backwards edge: segment 6's near end (z 30)", L.size() > 3 ? L[3][1] : -1, 30);
    }
    std::printf("hudcheck: minimap-fix radar status line (hand vectors from 0x4a54b0 asm 0x4a5680..0x4a5779, 0x62baa0)\n");
    {
        BannerStatus s;  // ctor 0x49eea0: 2, 0, 6
        const bool first = bannerStatusStep(s, 0, 3, 0.1f);
        near("0x4a54b0 free roam at start: index 2, shown changed", s.index * 10 + first, 21, 0);
        bannerStatusStep(s, 4, 1, 0.1f);
        near("0x4a54b0 career list, PREPARING: 0 (In career race)", s.index, 0, 0);
        BannerStatus c, e, t;
        bannerStatusStep(c, 5, 2, 0.1f), bannerStatusStep(e, 5, 3, 0.1f), bannerStatusStep(t, 5, 0, 0.1f);
        near("0x4a54b0 challenge list, RUNNING: 4 (In challenge)", c.index, 4, 0);
        near("0x4a54b0 no hub list (edited race), RUNNING: 5", e.index, 5, 0);
        near("0x4a54b0 tutorial list: (0 != 2) + 4 = 5", t.index, 5, 0);
        BannerStatus w;
        bannerStatusStep(w, 5, 1, 0.1f);
        bannerStatusStep(w, 6, 1, 4.9f);  // ENDING_WON: the timer runs from 0
        near("0x4a54b0 4.9 s after the race ended: still 0", w.index, 0, 0);
        const bool changed = bannerStatusStep(w, 6, 1, 0.2f);
        near("0x4a54b0 past 5 s: 2, shown changed", w.index * 10 + changed, 21, 0);
        BannerStatus k;
        bannerStatusStep(k, 5, 1, 0.1f);
        bannerKilled(k);
        const bool after = bannerStatusStep(k, 0, 3, 0.0f);
        near("0x62baa0 kill (2, 5.0 s) then a frame: 2, changed", k.index * 10 + after, 21, 0);
    }
    std::printf("hudcheck: VignettageHUD (P102, derived from 0x443290 / 0x4caef0 / 0x4cc0f0)\n");
    {
        // UiRenderer's canvas at 3420 x 2224 (0x49bec0 letterbox: 1280 wide, 16:9 band centred): the target spans
        // canvas y -56.1875 .. 776.1875; the vignette covers all of it (P56 drew it on the band 0..720 only).
        const float s = 3420.0f / 1280.0f, top = -((2224.0f - 720.0f * s) * 0.5f) / s, hgt = 2224.0f / s;
        const Rect q = vignetteRect(0, top, 1280, hgt);
        near("0x4cc0f0 vignette top at 3420x2224 (canvas units)", q.y, -56.1875, 1e-5);
        near("0x4cc0f0 vignette height at 3420x2224", q.h, 832.374, 1e-5);
        near("0x4cc0f0 vignette bottom at 3420x2224", q.y + q.h, 776.1875, 1e-5);
        near("0x443290 vignette with HUDs on or off (no gate)", vignetteShown(false, false), 1, 0);
        near("0x443290 DisableOSD: no vignette", vignetteShown(true, false), 0, 0);
        near("0x4caef0 video playing (+0x33): no vignette", vignetteShown(false, true), 0, 0);
    }
    std::printf("hudcheck: [nitro] the nitrous gauge (the port's; ui/nitro_gauge.mm)\n");
    {
        const NitroBottle b = nitroBottle(956, 464, 260, 220);  // a dial at x 956..1216, y 464..684 (a 64 / 36 safe frame)
        near("nitro bottle left edge 8 clear of the dial", b.x0, 1224, 0);
        near("nitro bottle foot 12 above the dial's bottom", b.y1, 672, 0);
        const Rect r = nitroBounds(b);
        near("nitro gauge inside the 1280-wide canvas (right edge)", r.x + r.w <= 1280.0f, 1, 0);
        near("nitro gauge inside the canvas (bottom edge <= 720)", r.y + r.h <= 720.0f, 1, 0);
        near("nitro surface empty = the inner floor", nitroSurfaceY(b, 0), 670, 1e-6);
        near("nitro surface full = the inner top", nitroSurfaceY(b, 1), 562, 1e-6);
        near("nitro surface half way", nitroSurfaceY(b, 0.5f), 616, 1e-6);
        float shown = 0;
        for (int i = 0; i < 6; ++i) shown = nitroShown(shown, 1, 1.0f / 60);
        near("nitro shown level after 0.1 s (exp 0.12 s)", shown, 1 - std::exp(-0.1 / 0.12), 1e-5);
        for (int i = 0; i < 60; ++i) shown = nitroShown(shown, 1, 1.0f / 60);
        near("nitro shown level settles on the tank", shown, 1, 1e-3);
        near("nitro empty flash: off when over", nitroFlashAlpha(0), 0, 0);
        near("nitro empty flash: first blink peak (0.15 s in)", nitroFlashAlpha(0.45f), 0.5 * (1 - 0.4 * 0.25), 1e-5);
        near("nitro empty flash: dark between blinks (0.3 s in)", nitroFlashAlpha(0.3f), 0, 1e-6);
        near("nitro glow: none below full", nitroGlowAlpha(0.99f, 1), 0, 0);
        near("nitro glow: full tank pulse", nitroGlowAlpha(1, 0), 0.22, 1e-6);
    }
    if (!pp.empty()) {
        std::printf("hudcheck: pp001.tsc rows the minimap reads at run time\n");
        near("PP22 < PP23 (zoom near < far)", pp[22] < pp[23] && pp[22] > 0, 1, 0);
        near("PP924 race line width > 0", pp[924] > 0, 1, 0);
    }
    std::printf("hudcheck: %d / %d ok\n", total - fails, total);
    return fails;
}

}  // namespace fuel::hud
