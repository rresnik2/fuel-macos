// [DEVIATION, the user's request 2026-11-06] The nitrous gauge (nitro_gauge.h; geometry and timing in hud_math).
#include "ui/nitro_gauge.h"

#include "physics/nitro.hpp"
#include "ui/hud_math.hpp"
#include "ui/ui_renderer.h"

#include <algorithm>
#include <array>
#include <cmath>
#include <vector>

namespace fuel::ui {
namespace {

struct Memory {
    float shown = -1;  // the displayed level (-1: not started)
    float t = 0;       // the gauge's clock (shimmer, ripples, bubbles)
    float stir = 0;    // 0..1: how lively the liquid is (boosting, a bonus just in)
};
Memory gMem;

// FUEL's dial orange (the compteur's ticks and numerals) and the nitrous blues.
constexpr Rgba kOrange{1.0f, 0.42f, 0.06f, 1.0f};
constexpr Rgba kGlass{0.02f, 0.03f, 0.05f, 0.55f};
constexpr Rgba kDeep{0.02f, 0.15f, 0.60f, 0.93f};
constexpr Rgba kTop{0.20f, 0.58f, 1.00f, 0.96f};
constexpr Rgba kFoam{0.78f, 0.93f, 1.00f, 0.95f};
constexpr Rgba kGlow{0.35f, 0.70f, 1.00f, 1.0f};

Rgba mix(const Rgba& a, const Rgba& b, float t) {
    t = std::clamp(t, 0.0f, 1.0f);
    return Rgba{a.r + (b.r - a.r) * t, a.g + (b.g - a.g) * t, a.b + (b.b - a.b) * t, a.a + (b.a - a.a) * t};
}
Rgba scaled(Rgba c, float k, float alpha = 1) {
    c.r = std::min(1.0f, c.r * k), c.g = std::min(1.0f, c.g * k), c.b = std::min(1.0f, c.b * k), c.a *= alpha;
    return c;
}
void tri(std::vector<UiVertex>& v, float ax, float ay, const Rgba& ca, float bx, float by, const Rgba& cb, float cx, float cy, const Rgba& cc) {
    v.push_back({ax, ay, 0, 0, ca}), v.push_back({bx, by, 0, 0, cb}), v.push_back({cx, cy, 0, 0, cc});
}
// A quad with a colour per corner: (x0, y0) c00, (x1, y0) c10, (x0, y1) c01, (x1, y1) c11.
void quad(std::vector<UiVertex>& v, float x0, float y0, float x1, float y1, const Rgba& c00, const Rgba& c10, const Rgba& c01, const Rgba& c11) {
    tri(v, x0, y0, c00, x1, y0, c10, x0, y1, c01);
    tri(v, x1, y0, c10, x1, y1, c11, x0, y1, c01);
}
void box(std::vector<UiVertex>& v, float x0, float y0, float x1, float y1, const Rgba& c) { quad(v, x0, y0, x1, y1, c, c, c, c); }
// A line of width w (canvas units) from a to b.
void line(std::vector<UiVertex>& v, float ax, float ay, float bx, float by, float w, const Rgba& c) {
    float dx = bx - ax, dy = by - ay;
    const float l = std::sqrt(dx * dx + dy * dy);
    if (l < 1e-4f) return;
    const float nx = -dy / l * w * 0.5f, ny = dx / l * w * 0.5f;
    tri(v, ax + nx, ay + ny, c, bx + nx, by + ny, c, ax - nx, ay - ny, c);
    tri(v, bx + nx, by + ny, c, bx - nx, by - ny, c, ax - nx, ay - ny, c);
}

// The bottle's silhouette (clockwise from the neck's right foot): the shoulders round into the neck, the body's foot
// has rounded corners. `grow` pushes it outwards (the glow rings).
std::vector<std::array<float, 2>> outline(const hud::NitroBottle& b, float grow) {
    const float cx = (b.x0 + b.x1) * 0.5f, neck = 4.0f + grow;
    const float x0 = b.x0 - grow, x1 = b.x1 + grow, y0 = b.y0 - grow, y1 = b.y1 + grow;
    const float rs = 6.0f, rf = 5.0f;  // shoulder and foot radii
    std::vector<std::array<float, 2>> p;
    p.push_back({cx + neck, b.neckTop - grow});
    p.push_back({cx + neck, y0});
    for (int k = 0; k <= 6; ++k) {  // right shoulder: from the neck out to the wall
        const float a = float(k) / 6.0f * 1.5707963f;
        p.push_back({x1 - rs + rs * std::sin(a), y0 + rs - rs * std::cos(a)});
    }
    for (int k = 0; k <= 6; ++k) {  // right foot
        const float a = float(k) / 6.0f * 1.5707963f;
        p.push_back({x1 - rf + rf * std::cos(a), y1 - rf + rf * std::sin(a)});
    }
    for (int k = 0; k <= 6; ++k) {  // left foot
        const float a = float(k) / 6.0f * 1.5707963f;
        p.push_back({x0 + rf - rf * std::sin(a), y1 - rf + rf * std::cos(a)});
    }
    for (int k = 0; k <= 6; ++k) {  // left shoulder
        const float a = float(k) / 6.0f * 1.5707963f;
        p.push_back({x0 + rs - rs * std::cos(a), y0 + rs - rs * std::sin(a)});
    }
    p.push_back({cx - neck, y0});
    p.push_back({cx - neck, b.neckTop - grow});
    return p;
}
void stroke(std::vector<UiVertex>& v, const std::vector<std::array<float, 2>>& p, float w, const Rgba& c, bool closed) {
    for (size_t i = 0; i + 1 < p.size(); ++i) line(v, p[i][0], p[i][1], p[i + 1][0], p[i + 1][1], w, c);
    if (closed && p.size() > 2) line(v, p.back()[0], p.back()[1], p[0][0], p[0][1], w, c);
}

// The inside's vertical extent at column x: the rounded foot and shoulders (the wall is 2 units thick).
void innerSpan(const hud::NitroBottle& b, float x, float& top, float& bottom) {
    const float rs = 4.0f, rf = 3.0f;  // the inner radii (outer 6 / 5 less the wall)
    const float l = b.x0 + 2.0f, r = b.x1 - 2.0f;
    const float dl = x - l, dr = r - x, d = std::min(dl, dr);
    top = b.y0 + 2.0f, bottom = b.y1 - 2.0f;
    if (d < rs) top += rs - std::sqrt(std::max(0.0f, rs * rs - (rs - d) * (rs - d)));
    if (d < rf) bottom -= rf - std::sqrt(std::max(0.0f, rf * rf - (rf - d) * (rf - d)));
}

}  // namespace

void drawNitroGauge(UiRenderer& r, const HudState& hud, float dialX, float dialY, float dialW, float dialH) {
    const HudState::Nitro& n = hud.nitro;
    if (!n.on) {
        gMem.shown = -1;
        return;
    }
    const hud::NitroBottle b = hud::nitroBottle(dialX, dialY, dialW, dialH);
    const float dt = std::clamp(hud.dt, 0.0f, 0.1f);
    gMem.t += dt;
    gMem.shown = (gMem.shown < 0 || hud.settle) ? n.level : hud::nitroShown(gMem.shown, n.level, dt);
    const float stirTarget = std::max(n.amount, n.bonusGlow > 0 ? 0.6f : 0.0f);
    gMem.stir += (stirTarget - gMem.stir) * (1.0f - std::exp(-dt / 0.25f));
    const float level = std::clamp(gMem.shown, 0.0f, 1.0f), t = gMem.t, stir = gMem.stir;
    const float flash = hud::nitroFlashAlpha(n.emptyFlash);
    const float glow = hud::nitroGlowAlpha(n.level, t);
    const float minLevel = n.minStartLevel > 0 ? n.minStartLevel : nitro::params().fraction(nitro::params().minStart);  // [upgrades U1]
    const bool usable = n.level >= minLevel - 1e-6f || n.active;
    std::vector<UiVertex> v;
    v.reserve(4096);

    // 1. The full tank's glow: soft blue rings round the silhouette.
    if (glow > 0) {
        const float rings[3][2] = {{1.6f, 0.55f}, {3.2f, 0.30f}, {4.8f, 0.12f}};
        for (const auto& ring : rings) stroke(v, outline(b, ring[0]), 1.8f, scaled(kGlow, 1, glow * ring[1] / 0.32f), true);
    }
    // 2. The glass: dark, with the empty flash's dim red.
    const float lIn = b.x0 + 2.0f, rIn = b.x1 - 2.0f;
    constexpr int kCols = 14;
    const float colW = (rIn - lIn) / float(kCols);
    for (int c = 0; c < kCols; ++c) {
        const float xa = lIn + colW * float(c), xb = xa + colW, xm = (xa + xb) * 0.5f;
        float top, bottom;
        innerSpan(b, xm, top, bottom);
        box(v, xa, top, xb, bottom, mix(kGlass, Rgba{0.55f, 0.07f, 0.04f, 0.75f}, flash * 1.6f));
    }
    // neck and valve inside
    const float cx = (b.x0 + b.x1) * 0.5f;
    box(v, cx - 3.0f, b.neckTop + 1.0f, cx + 3.0f, b.y0 + 2.0f, kGlass);
    // 3. The liquid: columns from the floor to a rippling surface, darker with depth, a glass highlight left of the
    // middle, the walls shaded, slow bright bands rising through it (faster and stronger while boosting).
    const float base = hud::nitroSurfaceY(b, level);
    const float amp = 0.35f + 0.9f * stir;                 // ripple height (canvas units)
    const float speed = 1.6f + 4.5f * stir;                // ripple / shimmer speed
    const float shimmer = 0.16f + 0.22f * stir + 0.10f * glow;
    std::vector<float> surf(kCols + 1);
    for (int c = 0; c <= kCols; ++c) {
        const float x = lIn + colW * float(c);
        surf[size_t(c)] = base + amp * (std::sin(x * 0.9f + t * speed) * 0.7f + std::sin(x * 2.1f - t * speed * 1.37f) * 0.3f);
    }
    auto liquidAt = [&](float x, float y, float s) {
        const float depth = std::clamp((y - s) / 70.0f, 0.0f, 1.0f);
        Rgba c = mix(kTop, kDeep, depth);
        const float u = (x - lIn) / (rIn - lIn);
        const float hl = std::exp(-((u - 0.28f) * (u - 0.28f)) / 0.006f) * 0.38f;  // the glass highlight
        const float wall = 0.78f + 0.22f * std::sin(u * 3.1415927f);              // the cylinder's shading
        const float band = std::pow(std::max(0.0f, std::sin(y * 0.19f + t * speed * 1.2f - u * 1.5f)), 6.0f) * shimmer;
        c = scaled(c, wall);
        c = mix(c, kFoam, hl + band);
        return c;
    };
    if (level > 0.002f) {
        for (int c = 0; c < kCols; ++c) {
            const float xa = lIn + colW * float(c), xb = xa + colW, xm = (xa + xb) * 0.5f;
            float top, bottom;
            innerSpan(b, xm, top, bottom);
            const float sa = std::max(surf[size_t(c)], top), sb = std::max(surf[size_t(c + 1)], top);
            const float s = std::min(sa, sb);
            if (s >= bottom) continue;
            // the ripple's wedge on top of the column, then rows down to the floor
            const float sMax = std::max(sa, sb);
            tri(v, xa, sa, liquidAt(xa, sa, sa), xb, sb, liquidAt(xb, sb, sb), sa < sb ? xa : xb, sMax, liquidAt(xm, sMax, s));
            for (float y = sMax; y < bottom; y += 3.0f) {
                const float y1 = std::min(bottom, y + 3.0f);
                quad(v, xa, y, xb, y1, liquidAt(xa, y, s), liquidAt(xb, y, s), liquidAt(xa, y1, s), liquidAt(xb, y1, s));
            }
        }
        // the meniscus: a thin bright line along the surface (the foam), inside the walls
        for (int c = 0; c < kCols; ++c) {
            const float xa = lIn + colW * float(c), xb = xa + colW;
            float top, bottom;
            innerSpan(b, (xa + xb) * 0.5f, top, bottom);
            const float sa = surf[size_t(c)], sb = surf[size_t(c + 1)];
            if (std::max(sa, sb) < top + 0.5f || std::min(sa, sb) > bottom - 0.5f) continue;
            line(v, xa, sa, xb, sb, 1.3f, scaled(kFoam, 1, 0.75f + 0.25f * stir));
        }
        // bubbles while boosting (and a few after a bonus): small rising dots, from the floor to the surface
        const int bubbles = int(std::lround(7.0f * stir));
        for (int k = 0; k < bubbles; ++k) {
            const float seed = float(k) * 1.618034f;
            const float bx = lIn + 2.0f + std::fmod(seed * 7.31f, 1.0f) * (rIn - lIn - 4.0f) + std::sin(t * 3.0f + seed * 5.0f) * 0.8f;
            float top, bottom;
            innerSpan(b, bx, top, bottom);
            const float span = bottom - std::max(base, top);
            if (span < 4.0f) continue;
            const float y = bottom - std::fmod(t * (22.0f + 9.0f * std::fmod(seed * 3.7f, 1.0f)) + seed * 37.0f, span);
            const float sz = 1.1f + 0.6f * std::fmod(seed * 5.3f, 1.0f);
            box(v, bx - sz, y - sz, bx + sz, y + sz, scaled(kFoam, 1, 0.75f));
        }
    }
    // 4. The outline (FUEL's dial orange; dimmed while the tank cannot start a boost), the neck and the valve cap.
    const Rgba line0 = usable ? kOrange : scaled(kOrange, 0.75f, 0.55f);
    Rgba edge = mix(line0, kFoam, std::max(n.bonusGlow > 0 ? 0.45f * std::min(1.0f, n.bonusGlow / 0.4f) : 0.0f, 0.0f));
    edge = mix(edge, Rgba{1.0f, 0.18f, 0.08f, 1.0f}, flash * 2.0f);  // the empty flash: the outline blinks red too
    stroke(v, outline(b, 0), 1.5f, edge, false);
    line(v, cx - 4.0f, b.neckTop, cx + 4.0f, b.neckTop, 1.5f, edge);
    box(v, cx - 6.5f, b.capTop, cx + 6.5f, b.neckTop - 1.0f, Rgba{0.10f, 0.10f, 0.10f, 0.85f});
    stroke(v, {{cx - 6.5f, b.capTop}, {cx + 6.5f, b.capTop}, {cx + 6.5f, b.neckTop - 1.0f}, {cx - 6.5f, b.neckTop - 1.0f}}, 1.2f, edge, true);
    // 5. Tick marks left of the body, one per second of boost (the tank's seconds less one, as the dial's ticks).
    const int ticks = std::clamp(int(std::lround(n.tankSeconds > 0 ? n.tankSeconds : nitro::params().tankSeconds)) - 1, 0, 9);  // [upgrades U1]
    for (int k = 1; k <= ticks; ++k) {
        const float y = hud::nitroSurfaceY(b, float(k) / float(ticks + 1));
        line(v, b.x0 - 5.0f, y, b.x0 - 1.0f, y, 1.4f, line0);
    }
    r.triangles(nullptr, v);
}

}  // namespace fuel::ui
