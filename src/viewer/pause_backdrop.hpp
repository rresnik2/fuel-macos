// PK17: FUEL's pause-menu backdrop and fade (notes/pk17.md). Header-only maths shared by the in-game overlay
// (ui/game_menu.mm), the session host (viewer/main.mm) and --rendercheck.
//  - ViewportHUD fade (+0x28 alpha, +0x2c rate, +0x30 fading): vf00 0x4c89e0 rate = -4.0 (0xc0800000), alpha 0;
//    vf07 0x4c8ce0 show: rate = |rate|, alpha = 0.001 (0x3a83126f), fading; vf08 0x4c8d80 hide: rate = -|rate|, fading;
//    vf10 0x4c8c50 step(dt): alpha += rate x dt; < 0 -> alpha 0, done, returns -1; > 1 -> alpha 1, done, returns 1.
//  - InGameMenuHUD_SetPostTint 0x46af90 (per frame from vf02 0x46ae80 while +0x4c != 0 and alpha > 0):
//    gTintLerp.rgb = PP39..41, gTintLerp.a = PP43 x alpha + (1 - alpha) x saved (+0x6c), viewport +0x1dc = PP42 x alpha
//    (radial blur flag 0x8000 set when > 0, else cleared).
//  - vf10 0x46aed0: faded in -> gTintLerp.a = PP43; faded out -> gTintLerp.a = saved if World_IsOutside(player) else 0.
//  - 0x46abd0: the day clock = 43200 / PP371 (450 s -> 20:00) and World_SetTimeAndCloud(72000 = 20:00, cloud 0).
#pragma once

#include <cmath>

namespace fuel::pause {

inline constexpr float kFadeRate = 4.0f;          // ViewportHUD vf00: +0x2c = -4.0
inline constexpr float kShowAlpha = 0.001f;       // vf07: +0x28 = 0x3a83126f
inline constexpr double kBackdropTimeOfDay = 72000.0;  // 0x46abd0: World_SetCloudLayer(world, 0x478ca000, 0)
inline constexpr float kBackdropSunCloud = 0.0f;  // ... cloud argument 0 (Atmo_UpdateSun 0x780920: the intensity term only)
inline constexpr double kBackdropClockArg = 43200.0;  // 0x46abd0: SetDayClock(43200 / PP371)

// ViewportHUD's fade (+0x28 / +0x2c / +0x30).
struct HudFade {
    float alpha = 0, rate = -kFadeRate;
    bool fading = false;
    void show() {  // vf07
        if (rate < 0) rate = -rate;
        alpha = kShowAlpha, fading = true;
    }
    void hide() {  // vf08
        if (rate > 0) rate = -rate;
        fading = true;
    }
    int step(float dt) {  // vf10
        if (!fading) return 0;
        const float a = rate * dt + alpha;
        alpha = a;
        if (a < 0.0f) return fading = false, alpha = 0, -1;
        if (a > 1.0f) return alpha = 1, fading = false, 1;
        return 0;
    }
};

struct Tint {
    float r = 0, g = 0, b = 0, a = 0;  // gTintLerp
    float blur = 0;                    // viewport +0x1dc
    bool blurFlag = false;             // viewport flag 0x8000
};

// 0x46af90. `pp` = pp001.tsc PP n accessor; `saved` = +0x6c (gTintLerp.a when the menu opened).
template <class PP>
inline Tint menuTint(const PP& pp, float alpha, float saved) {
    Tint t;
    t.r = pp(39), t.g = pp(40), t.b = pp(41);
    t.a = pp(43) * alpha + (1.0f - alpha) * saved;
    t.blur = pp(42) * alpha;
    t.blurFlag = t.blur > 0.0f;
    return t;
}

// vf10 0x46aed0 after the fade-out: gTintLerp.a (`outside` = World_IsOutside 0x5280a0 at the player: Breadcrumbs::worldOutside).
inline float closedTintA(bool outside, float saved) { return outside ? saved : 0.0f; }

// 0x46abd0: SetDayClock's argument (float division as FUEL: 43200.0 / g_pp001_params[0x173]).
inline double backdropClock(float pp371) { return double(float(kBackdropClockArg) / pp371); }

}  // namespace fuel::pause
