#import "rendercheck.h"

#import <Metal/Metal.h>

#include "core/atmosphere.hpp"
#include "core/sfx.hpp"
#include "core/particles.hpp"
#include "core/weather_fx.hpp"
#include "core/weather.hpp"
#include "core/clouds.hpp"
#include "core/material_anim.hpp"
#include "core/blob_shadow.hpp"
#include "core/vehicle_lights.hpp"
#include "core/vehicle_links.hpp"  // PK22
#include "core/vehicle_fx.hpp"     // PK23
#include "core/sand_tank.hpp"      // PK12
#include "core/anim_trigger.hpp"   // PK12
#include "core/assembly.hpp"       // PK23
#include "core/wind_tank.hpp"
#include "core/vegetation.hpp"  // P94
#include "core/world.hpp"       // P94
#include "core/rtc_fade.hpp"    // PK17: readPpRows
#include "audio/sound_mixer.hpp"  // PK17: the pause mask
#include "viewer/pause_backdrop.hpp"  // PK17
#include "physics/breadcrumbs.hpp"  // PK17: World_IsOutside 0x5280a0
#include "core/f14_flyby.hpp"        // PK27
#include "viewer/f14_view.h"         // PK27
#include "core/rotshape.hpp"         // PK28
#include "core/resources.hpp"        // PK28
#include "core/genworld_noise.hpp"   // PK28: readEngineFloats
#import "renderer.h"
#include "viewer/taa_math.h"         // TAA
#include "core/upgrade_visuals.hpp"   // [upgrades U5]
#include "core/upgrades.hpp"          // [upgrades U5]
#include "core/vehicle_params.hpp"    // [upgrades U5]
#include "core/custom_vehicles.hpp"   // [upgrades U5]
#include "physics/vehicle_physics.hpp"  // [upgrades U5]

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <string>
#include <vector>

namespace fuel {

namespace {

int gPass = 0, gFail = 0;

bool near(float got, float want, float tol) {
    return std::fabs(got - want) <= tol * std::max(1.0f, std::fabs(want));
}

void check(const char* what, const float* got, const float* want, int n, float tol) {
    bool ok = true;
    for (int i = 0; i < n; ++i) ok = ok && near(got[i], want[i], tol);
    ++(ok ? gPass : gFail);
    std::printf("  %-4s %-58s got", ok ? "ok" : "FAIL", what);
    for (int i = 0; i < n; ++i) std::printf(" %.6g", got[i]);
    if (!ok) {
        std::printf("  want");
        for (int i = 0; i < n; ++i) std::printf(" %.6g", want[i]);
    }
    std::printf("\n");
}
void check1(const char* what, float got, float want, float tol) { check(what, &got, &want, 1, tol); }
void check3(const char* what, const F3& got, F3 want, float tol) { check(what, got.data(), want.data(), 3, tol); }
void check4(const char* what, const F4& got, F4 want, float tol) { check(what, got.data(), want.data(), 4, tol); }

double hm(int h, int m) { return (h * 60 + m) * 60.0; }

Atmosphere at(double t, float cloud = 0, bool weather = false, float camY = 900) {
    AtmosphereInput in;
    in.timeOfDay = t;
    in.weather.cloud = cloud;
    in.weather.apply = weather;
    in.camera = {0, camY, 0};
    return computeAtmosphere(in);
}

float smooth(float a, float b, float x) {
    const float t = std::clamp((x - a) / (b - a), 0.0f, 1.0f);
    return t * t * (3 - 2 * t);
}

}  // namespace

int runRenderCheck(const std::string& dataDir) {
    std::printf("FUEL renderer test vectors (p26-renderer.md, p26-work/post.md, shadows.md)\n");

    // ---- L2 / L3: sun path and intensity (emulated on FUEL.exe 0x780920)
    std::printf("sun path and exposure (L2, L3)\n");
    const struct { int h, m; float cosTheta, I; } sun[] = {{6, 30, 0.0717f, 0.203f}, {7, 0, 0.1425f, 0.284f}, {8, 0, 0.280f, 0.393f},
                                                         {10, 0, 0.517f, 0.478f}, {11, 59, 0.676f, -1}, {14, 0, 0.7317f, 0.498f}};
    for (const auto& v : sun) {
        const Atmosphere a = at(hm(v.h, v.m));
        char name[96];
        std::snprintf(name, sizeof name, "%02d:%02d cos(theta)", v.h, v.m);
        check1(name, std::cos(a.theta), v.cosTheta, 2e-3f);
        if (v.I >= 0) {
            std::snprintf(name, sizeof name, "%02d:%02d intensity I", v.h, v.m);
            check1(name, a.intensity, v.I, 2e-3f);
        }
    }
    // P105: the night branch of 0x780920, emulated (notes/p105-work/emu/sun.cmd -> sun.out): DLIGHT_DIR A+0xbc.
    check3("P105 02:00 DLIGHT_DIR (moon at its highest, emulated)", at(hm(2, 0)).lightDir, {3.916999e-8f, 0.82140505f, -0.5703452f}, 2e-4f);
    check3("P105 05:00 DLIGHT_DIR (emulated)", at(hm(5, 0)).lightDir, {0.8425047f, 0.48332262f, -0.23787619f}, 2e-4f);
    check3("P105 05:50 DLIGHT_DIR (moon low in the east, emulated)", at(hm(5, 50)).lightDir, {0.95677423f, 0.28767332f, -0.042745773f}, 2e-4f);
    check1("P105 05:50 blue shift strength (night, emulated)", at(hm(5, 50)).blueShift[3], 0.5f, 1e-6f);
    {
        float y0 = 0, y1 = 0;
        sunDirection(hm(5, 59) + 59, &y0);
        sunDirection(hm(6, 0) + 1, &y1);
        // The night half-turn takes 8 h, the day half-turn 16 h: the sun rises twice as fast in the last second
        // before 06:00 (-7.98e-5) as in the first second after it (+3.99e-5).
        const float g[2] = {y0, y1}, w[2] = {-7.98e-5f, 3.99e-5f};
        check("P105 sun height 1 s before / after 06:00 (continuous)", g, w, 2, 2e-6f);
    }
    check1("night intensity I (02:00)", at(hm(2, 0)).intensity, 0.100f, 1e-3f);
    check1("peak intensity I, cloud 1", at(hm(14, 0), 1.0f).intensity, 0.373f, 2e-3f);
    {
        const Atmosphere a = at(hm(8, 0));
        check3("08:00 DLIGHT_DIR (worked example L)", a.lightDir, {0.843f, 0.483f, 0.238f}, 2e-3f);
        check4("08:00 blue shift", a.blueShift, {0.4f, 0.5f, 1.0f, 10.0f}, 1e-4f);
        const Atmosphere p = at(hm(14, 0));
        check1("peak bloom luminance scale = 3.00077 I", p.bloomLumScale, 3.00077f * 0.498f, 2e-3f);
        check1("night blue shift strength", at(hm(2, 0)).blueShift[3], 0.5f, 1e-4f);
    }

    // ---- L4: sun and sky colours (emulated)
    std::printf("sun and sky colours (L4)\n");
    const struct { int h, m; float theta; F3 dl, amb, sky; } col[] = {
        {6, 30, 1.499f, {3.638f, 1.639f, 0.278f}, {2.822f, 2.356f, 1.577f}, {10.12f, 8.45f, 6.28f}},
        {7, 0, 1.428f, {3.298f, 2.120f, 0.793f}, {2.767f, 2.362f, 1.663f}, {9.39f, 8.02f, 6.27f}},
        {8, 0, 1.287f, {2.941f, 2.330f, 1.388f}, {2.689f, 2.366f, 1.781f}, {8.13f, 7.16f, 5.98f}},
        {10, 0, 1.027f, {2.687f, 2.366f, 1.783f}, {2.604f, 2.364f, 1.906f}, {6.21f, 5.63f, 5.05f}},
        {14, 0, 0.750f, {2.585f, 2.362f, 1.933f}, {2.556f, 2.359f, 1.975f}, {4.61f, 4.26f, 3.96f}},
    };
    for (const auto& v : col) {
        const Atmosphere a = at(hm(v.h, v.m));
        char name[96];
        std::snprintf(name, sizeof name, "%02d:%02d theta", v.h, v.m);
        check1(name, a.theta, v.theta, 2e-3f);
        std::snprintf(name, sizeof name, "%02d:%02d DLIGHT_COLOR", v.h, v.m);
        check3(name, a.dlightColor, v.dl, 3e-3f);
        std::snprintf(name, sizeof name, "%02d:%02d DLIGHT_AMBIENT", v.h, v.m);
        check3(name, a.dlightAmbient, v.amb, 3e-3f);
        std::snprintf(name, sizeof name, "%02d:%02d DSKY_COLOR", v.h, v.m);
        check3(name, a.dskyColor, v.sky, 3e-3f);
    }
    {
        Atmosphere n;
        n.theta = 1.521f, n.day = false, n.blueShift = {0.4f, 0.5f, 1.0f, 0.5f};
        lightColours(n, 0);
        check3("night (moon, theta 1.521 to 3 decimals) DLIGHT_COLOR", n.dlightColor, {3.719f, 1.318f, 0.656f}, 1e-2f);
        check3("night DLIGHT_AMBIENT", n.dlightAmbient, {1.327f, 1.099f, 3.610f}, 3e-3f);
        check3("night DSKY_COLOR", n.dskyColor, {4.84f, 4.01f, 14.62f}, 3e-3f);
        check3("peak DSKY_COLOR, cloud 1", at(hm(14, 0), 1.0f).dskyColor, {5.77f, 5.32f, 4.95f}, 3e-3f);
    }

    // ---- L5: scatter constants c6..c16 (emulated, parameter defaults of 0x780ce0)
    std::printf("scatter constants c6..c16 (L5)\n");
    {
        const Atmosphere a = at(hm(14, 0));
        check4("SC1", a.c[kSC1], {1, 10000, 4, 0}, 1e-5f);
        check4("SC2", a.c[kSC2], {1.442695f, 0.7f, 0.5f, 0.002f}, 1e-5f);
        check4("TERM_MULTIPLIERS", a.c[kTERM], {1, 45, 20000, 1e-4f}, 1e-5f);
        const float tol = 2e-4f;
        auto rel = [&](const char* nm, const F4& g, F4 w) {  // relative per component (tiny betas)
            F4 r{};
            for (int i = 0; i < 4; ++i) r[size_t(i)] = g[size_t(i)] / w[size_t(i)];
            check4(nm, r, {1, 1, 1, 1}, tol);
        };
        rel("BETA_1 (got / want)", a.c[kBETA1], {3.485764e-5f, 5.894569e-5f, 1.222298e-4f, 1});
        rel("BETA_DASH_1 (got / want)", a.c[kBETADASH1], {2.080412e-6f, 3.518062e-6f, 7.295053e-6f, 1});
        rel("BETA_2 (got / want)", a.c[kBETA2], {2.870299e-6f, 3.699844e-6f, 5.257156e-6f, 1});
        rel("BETA_DASH_2 (got / want)", a.c[kBETADASH2], {6.668937e-7f, 8.672288e-7f, 1.248809e-6f, 1});
        rel("BETA_1_PLUS_2 (got / want)", a.c[kBETASUM], {3.772794e-5f, 6.264553e-5f, 1.274869e-4f, 2});
        rel("ONE_OVER_BETA_1_PLUS_2 (got / want)", a.c[kONEOVER], {26505.557f, 15962.831f, 7843.9404f, 100000});
        check4("HG", a.c[kHG], {0.19f, 1.9f, 1.8f, 0}, 1e-5f);
        // DUSK: SC3 = normalize(sun position - camera), sun position = camera + S+0x4c x 10000 (0x779f99, EBX = S): the
        // un-lifted sun direction (0x780c53..0x780c7e), not DLIGHT_DIR (P26's emulation fed 0x7816c0 a DLIGHT_DIR sun).
        check4("SC3 at the peak = S+0x4c (DUSK)", a.c[kSC3], {0, 0.7316888f, 0.6816388f, 1}, 1e-5f);
        check1("sun term s (camera y 100, 08:00)", at(hm(8, 0), 0, false, 100).sunTerm, 1.0f, 1e-5f);
        // The weather hook 0x6409d0 (disassembly) with a clear sky: layer-2 density 5, no cloud slab, SC3.w 0.
        const Atmosphere w = at(hm(14, 0), 0, true);
        check4("weather hook, clear: SC2", w.c[kSC2], {1.442695f, 0.7f, 5.0f, 0.002f}, 1e-5f);
        check1("weather hook, clear: SC3.w (horizon weight)", w.c[kSC3][3], 0.0f, 1e-6f);
        Weather wt;
        wt.cloud = 0.5f, wt.fog = 0.5f, wt.wet = 0.2f;
        auto P = scatterDefaults();
        applyWeather(P, wt);
        check1("weather cloud .5 fog .5: idx12 = 0.9 c (2-c)(1-f^8)", P[12], 0.9f * 0.5f * 1.5f * (1 - std::pow(0.5f, 8.0f)), 1e-5f);
        check1("weather fog .5: idx6 = 500 sqrt(f^8) + 0.7", P[6], 500 * std::pow(0.5f, 4.0f) + 0.7f, 1e-5f);
        check1("weather cloud .5: idx9 = 50 c", P[9], 25.0f, 1e-6f);
    }

    // ---- P105: W+0xbc, the lowest ground near the camera (0x77a240) -> haze layer P7 (0x77a407) and the cloud layer
    // heights (0x6409d5 / 0x6409df); the sky colour with it (scatteringbase of scattering.h:122-136 + scatteringfinish
    // :59-117 at the 45 km dome), against an independent float64 model of the shader (notes/p105-work/py/sky.py).
    std::printf("P105 sky: lowest ground W+0xbc, haze layer P7, sky colour\n");
    {
        // A lattice of 5 x 5 samples at +-2000 m / 1000 m steps: a dip between lattice points is not seen.
        auto slope = [](double x, double z, const void*) { return float(800.0 + 0.01 * x + 0.02 * z); };
        check1("0x77a240 min over the lattice (plane: the (-2000, -2000) corner)", lowestGroundNear(0, 0, slope, nullptr), 740.0f, 1e-3f);
        auto pit = [](double x, double z, const void*) {
            if (std::fabs(x - 1000.0) < 1 && std::fabs(z + 2000.0) < 1) return 300.0f;  // on the lattice
            if (std::fabs(x - 500.0) < 1 && std::fabs(z) < 1) return 100.0f;            // between lattice points
            return 900.0f;
        };
        check1("0x77a240 sees a pit on the lattice only", lowestGroundNear(0, 0, pit, nullptr), 300.0f, 0);
        check1("0x77a240 without a heightfield (samples 0)", lowestGroundNear(0, 0, nullptr, nullptr), 0.0f, 0);
        AtmosphereInput in;
        in.timeOfDay = hm(12, 0);
        in.camera = {0, 866, 0};
        in.lowestGround = 750;
        in.weather.wet = 1, in.weather.cloudBase = 400, in.weather.cloudTopArg = 866;
        Atmosphere a = computeAtmosphere(in);
        const float pv[4] = {a.P[7], a.P[8], a.P[10], a.P[11]};
        const float pw[4] = {1750, 10, 1150, 3150};
        check("P7 = W+0xbc + 1000, P8 = 5 (wet + 1), P10 = min(max(h0 + bc, y + 100), 1500), P11 = h0 + 2000 + bc", pv, pw, 4, 1e-3f);
        check1("SC2.w = 1 / P7", a.c[kSC2][3], 1.0f / 1750.0f, 1e-9f);
        in.weather.cloudBase = 1200;
        check1("P10 clamped to 1500 (0x9dc76c)", computeAtmosphere(in).P[10], 1500.0f, 0);
        in.weather.cloudBase = 400;
        auto skyAt = [&](const Atmosphere& b, F3 d) {
            const float l = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
            const float R = 0.9f * 50000.0f;
            F3 ext, insc;
            scattering(b, in.camera, {d[0] / l * R, 866 + d[1] / l * R, d[2] / l * R}, ext, insc);
            return insc;
        };
        auto relc = [&](const char* nm, const F3& g, F3 w) {
            F3 r{};
            for (int i = 0; i < 3; ++i) r[size_t(i)] = g[size_t(i)] / w[size_t(i)];
            check3(nm, r, {1, 1, 1}, 2e-3f);
        };
        relc("12:00 haze: sky at the zenith (got / want)", skyAt(a, {0, 1, 0}), {1.4808f, 2.1256f, 3.0934f});
        relc("12:00 haze: sky toward the sun, 11 deg up (got / want)", skyAt(a, {0.3f, 0.2f, 0.9f}), {7.5991f, 8.7446f, 8.4179f});
        relc("12:00 haze: sky away from the sun, 26 deg up (got / want)", skyAt(a, {-0.7f, 0.4f, -0.3f}), {1.0690f, 1.5432f, 2.2316f});
        in.timeOfDay = hm(6, 10);
        a = computeAtmosphere(in);
        relc("06:10 haze: sky toward the sun, 3 deg up (orange) (got / want)", skyAt(a, {1, 0.05f, 0.2f}), {49.232f, 16.128f, 1.4947f});
        relc("06:10 haze: sky at the zenith (got / want)", skyAt(a, {0, 1, 0}), {0.78913f, 0.7745f, 0.95541f});
        relc("06:10 haze: sky away from the sun, 17 deg up (got / want)", skyAt(a, {-1, 0.3f, 0}), {1.2285f, 1.712f, 2.3173f});
    }

    // ---- DUSK: the sun direction S+0x4c vs DLIGHT_DIR (Atmo_UpdateSun asm 0x780c53..0x780cb5, Atmo_SetSunDir
    // 0x7808b0, GenWorld_UpdateLights 0x779f02..0x779fb0): v = normalize(x, max(|y|, 0.05), z) -> S+0x4c and theta;
    // DLIGHT_DIR = normalize(v + (0, 0.25, 0)); sun position = camera + v x 10000 -> SC3; DSKY_DIR = normalize(0.3 v.x,
    // 1, 0.3 v.z). Want values: dusk-work/py/sky.py (independent float64 model).
    std::printf("DUSK: sun direction S+0x4c (SC3, theta, DSKY_DIR) vs DLIGHT_DIR\n");
    {
        struct V { int h, m; F3 sd, ld, kd; float theta; };
        const V vs[] = {
            {5, 50, {0.9977542f, 0.0499948f, -0.0445766f}, {0.9567742f, 0.2876733f, -0.0427458f}, {0.2867321f, 0.9579251f, -0.0128103f}, 1.5207807f},
            {6, 10, {0.9985031f, 0.0499519f, 0.0222811f}, {0.9575017f, 0.2876350f, 0.0213662f}, {0.2869473f, 0.9579250f, 0.0064031f}, 1.5208236f},
            {21, 55, {-0.9986900f, 0.0499412f, 0.0111397f}, {-0.9576834f, 0.2876255f, 0.0106823f}, {-0.2870010f, 0.9579249f, 0.0032013f}, 1.5208344f},
            {12, 0, {0.3826834f, 0.6759924f, 0.6297521f}, {0.3233692f, 0.7824677f, 0.5321434f}, {0.1120984f, 0.9764242f, 0.1844716f}, 0.8284858f},
        };
        for (const V& v : vs) {
            const Atmosphere a = at(hm(v.h, v.m));
            char nm[96];
            std::snprintf(nm, sizeof nm, "%02d:%02d S+0x4c (sun position direction)", v.h, v.m);
            check3(nm, a.sunDir, v.sd, 2e-5f);
            std::snprintf(nm, sizeof nm, "%02d:%02d SC3.xyz = S+0x4c", v.h, v.m);
            check3(nm, {a.c[kSC3][0], a.c[kSC3][1], a.c[kSC3][2]}, v.sd, 2e-5f);
            std::snprintf(nm, sizeof nm, "%02d:%02d DLIGHT_DIR = normalize(S+0x4c + 0.25 up)", v.h, v.m);
            check3(nm, a.lightDir, v.ld, 2e-5f);
            std::snprintf(nm, sizeof nm, "%02d:%02d DSKY_DIR from S+0x4c", v.h, v.m);
            check3(nm, a.skyDir, v.kd, 2e-5f);
            std::snprintf(nm, sizeof nm, "%02d:%02d theta of S+0x4c (y clamped to 0.05)", v.h, v.m);
            check1(nm, a.theta, v.theta, 2e-5f);
        }
    }

    // ---- The worked ground pixel (L10): 08:00, flat ground, albedo (0.6, 0.5, 0.35), occlusion 1
    std::printf("worked ground pixel (L10)\n");
    {
        const Atmosphere a = at(hm(8, 0));
        const F3 n = {0, 1, 0}, V = {0, 0.2f, 0.98f}, alb = {0.6f, 0.5f, 0.35f};
        const F3 lit = lighting(a, n, V, alb, {0, 0, 0}, 15, 1, 1), sh = lighting(a, n, V, alb, {0, 0, 0}, 15, 0, 1);
        check3("lit HDR", lit, {8.251f, 5.802f, 3.017f}, 3e-3f);
        check3("shadowed HDR", sh, {5.183f, 3.777f, 2.173f}, 3e-3f);
        const float S = a.minExposure;
        check1("exposure at its lower clamp 0.33 I", S, 0.1295f, 2e-3f);
        const F4 notint = {1, 1, 1, 0};
        check3("lit display", toneMapPixel(lit, S, {0, 0, 0}, a.blueShift, notint), {0.826f, 0.673f, 0.385f}, 3e-3f);
        check3("shadowed display", toneMapPixel(sh, S, {0, 0, 0}, a.blueShift, notint), {0.619f, 0.474f, 0.280f}, 3e-3f);
    }

    // ---- Post chain (post.md P6)
    std::printf("post chain (post.md P6)\n");
    {
        const float xs[] = {0.05f, 0.1f, 0.18f, 0.25f, 0.5f, 1, 2, 4};
        const float ys[] = {0.0500f, 0.09999f, 0.17985f, 0.24927f, 0.48387f, 0.8f, 0.96774f, 0.99707f};
        for (int i = 0; i < 8; ++i) {
            char name[64];
            std::snprintf(name, sizeof name, "tone curve P/(1+P) at %g", xs[i]);
            check1(name, toneCurve(xs[i]), ys[i], 2e-4f);
        }
        check1("k = 0.94^(30 dt), dt 1/60", adaptationRate(1.0f / 60), 0.969536f, 1e-5f);
        check1("k, dt 1/30", adaptationRate(1.0f / 30), 0.94f, 1e-5f);
        check1("k, dt 0.1", adaptationRate(0.1f), 0.830584f, 1e-5f);
        // Ctor defaults: SCREENLUMINANCE 0.4, min 0.45, max 2.0.
        const float geo[] = {0.05f, 0.2f, 0.5f, 1.0f}, sw[] = {1.02134f, 1.00880f, 0.98103f, 0.97597f};
        for (int i = 0; i < 4; ++i) {
            char name[64];
            std::snprintf(name, sizeof name, "adaptation step from S 1, geomean %g", geo[i]);
            check1(name, exposureStep(1, std::log(geo[i]), 0.4f, 0.45f, 2.0f, 1.0f / 60), sw[i], 2e-5f);
        }
        check1("adaptation init, geomean 0.2", exposureInit(std::log(0.2f), 0.4f), 2.0f, 1e-5f);
        check1("bright pass v 1, S 2", brightPass(1, 2, 1.500385f, 5.000063f, 0.2f, 0.4f), 0.80045f, 2e-5f);
        check1("bright pass v 4, S 0.45", brightPass(4, 0.45f, 1.500385f, 5.000063f, 0.2f, 0.4f), 0.6204f, 2e-4f);
        check1("bright pass v 0.5, S 1", brightPass(0.5f, 1, 1.500385f, 5.000063f, 0.2f, 0.4f), 0.0f, 1e-6f);
        const F4 bs = {0.2f, 0.3f, 1.0f, 0.1f}, notint = {1, 1, 1, 0}, menu = {2, 0.8f, 0, 0.7f};
        check3("tone map (0.8, 0.6, 0.4), S 1", toneMapPixel({0.8f, 0.6f, 0.4f}, 1, {0, 0, 0}, bs, notint), {0.20736f, 0.23791f, 0.56653f}, 2e-4f);
        check3("tone map (2, 1.5, 1), S 0.45", toneMapPixel({2, 1.5f, 1}, 0.45f, {0, 0, 0}, bs, notint), {0.35693f, 0.34271f, 0.59099f}, 2e-4f);
        check3("tone map + menu tint", toneMapPixel({0.8f, 0.6f, 0.4f}, 1, {0, 0, 0}, bs, menu), {0.41937f, 0.21424f, 0.16996f}, 2e-4f);
        float off[11], w[11];
        gaussian11(off, w);
        const float wo[11] = {-6.5f, -5.5f, -4.5f, -3.5f, -2.5f, -1.5f, -0.5f, 0.5f, 1.5f, 2.5f, 3.5f};
        const float ww[11] = {0.035482932f, 0.058501467f, 0.0863096f, 0.113945305f, 0.13461047f, 0.14230047f,
                              0.13461047f, 0.113945305f, 0.0863096f, 0.058501467f, 0.035482932f};
        check("gaussian offsets (texels)", off, wo, 11, 1e-6f);
        check("gaussian weights", w, ww, 11, 1e-5f);
        float mb[8];
        motionBlurWeights(0.95f, mb);
        const float mbw[8] = {0.14855328f, 0.14112562f, 0.13406934f, 0.12736587f, 0.12099758f, 0.11494769f, 0.10920031f, 0.1037403f};
        check("radial blur weights (decay 0.95)", mb, mbw, 8, 1e-5f);
    }

    // ---- Shadows (shadows.md S4)
    std::printf("shadows (shadows.md S4)\n");
    {
        PostDefaults post;
        ShadowSettings ss;
        const std::string exe = (std::filesystem::path(dataDir) / "XFinalComponent" / "FUEL.exe").string();
        readEngineRenderSettings(exe.c_str(), post, ss);
        if (!ss.fromExe) {
            std::printf("  --   FUEL.exe not found under --data: shadow table and post defaults not checked\n");
        } else {
            check4("splits read from FUEL.exe 0xa05240", ss.splits, {17.5f, 77.5f, 315, 1600}, 1e-6f);
            check4("post defaults read from 0xa06680 (bloom)", post.bloom, {1.500385f, 5.000063f, 0.2f, 0.200004f}, 1e-6f);
            const float ab[3] = {float((ss.splits[1] - ss.splits[0]) * ss.zbias[1] * ss.zbiasAdd),
                                 float((ss.splits[2] - ss.splits[1]) * ss.zbias[2] * ss.zbiasAdd),
                                 float((ss.splits[3] - ss.splits[2]) * ss.zbias[3] * ss.zbiasAdd)};
            const float abw[3] = {0.140625f, 0.556640625f, 3.01171875f};
            check("ADDBIAS of slices 1, 2, 3 (m)", ab, abw, 3, 1e-5f);
            const float lo = ss.splits[2] + (ss.splits[3] - ss.splits[2]) * 0.5f;
            const float zs[5] = {900, 957.5f, 1200, 1500, 1600}, fw[5] = {0, 0, 0.3198305f, 0.9348674f, 1};
            float fg[5];
            for (int i = 0; i < 5; ++i) fg[i] = smooth(lo, ss.splits[3], zs[i]);
            check("distance fade at z 900 / 957.5 / 1200 / 1500 / 1600", fg, fw, 5, 1e-5f);
            const float vz[4] = {10, 17.5f, 100, 400}, sw[4] = {0, 1, 2, 3};
            float sl[4];
            for (int i = 0; i < 4; ++i) sl[i] = float(int(vz[i] >= ss.splits[0]) + int(vz[i] >= ss.splits[1]) + int(vz[i] >= ss.splits[2]));
            check("slice for view z 10 / 17.5 / 100 / 400", sl, sw, 4, 0);
        }
        const float nl[4] = {1, 0.5f, 0.2f, 0}, cw[4] = {0.99975592f, 3.99609756f, 24.84833778f, 4096};
        float cg[4];
        for (int i = 0; i < 4; ++i) cg[i] = 1.0f / (nl[i] * nl[i] + 1.0f / 4096);
        check("caster offset factor at |n.L| 1 / 0.5 / 0.2 / 0", cg, cw, 4, 1e-6f);
        const float so[5][2] = {{0, 0}, {0, 0.5f}, {0, 1}, {0.5f, 0.5f}, {1, 0.3f}};
        const float sw2[5] = {0, 0.05f, 0.1f, 0.525f, 1};
        float sg[5];
        for (int i = 0; i < 5; ++i) sg[i] = so[i][0] * (1 - so[i][1] * 0.1f) + so[i][1] * 0.1f;
        check("sunlighting(shadow, occ)", sg, sw2, 5, 1e-6f);
        // PixelShadow4 on a 3 x 3 neighbourhood (the shader's pcf3x3 does the same lerps).
        const float s[3][3] = {{1, 1, 0}, {1, 0, 0}, {1, 1, 1}};
        const float fx = 0.25f, fy = 0.75f;
        float sum = 0;
        for (int j = 0; j < 2; ++j)
            for (int i = 0; i < 2; ++i) {
                const float a0 = s[j][i] + (s[j][i + 1] - s[j][i]) * fx, a1 = s[j + 1][i] + (s[j + 1][i + 1] - s[j + 1][i]) * fx;
                sum += a0 + (a1 - a0) * fy;
            }
        check1("PCF 3x3 example", 0.25f * sum, 0.671875f, 1e-6f);
    }

    // ---- The Metal shaders against the CPU maths
    std::printf("Metal shader maths vs the CPU (scattering, lighting, tone curve, sky)\n");
    @autoreleasepool {
        id<MTLDevice> device = MTLCreateSystemDefaultDevice();
        if (device == nil) {
            std::printf("  FAIL no Metal device\n");
            ++gFail;
        } else {
            ResourceLibrary lib;
            MeshRenderer r(device, 1, lib);
            // P105: t 2, 3 = 06:10 and 12:00 with the W+0xbc haze layer (lowest ground 750 m, P7 1750) and a wet
            // weather hook (P8 = 10), as at the CR2 start.
            for (int t = 0; t < 4; ++t) {
                const double tod = t == 0 ? hm(8, 0) : t == 1 ? hm(20, 30) : t == 2 ? hm(6, 10) : hm(12, 0);
                const Atmosphere a = at(tod, 0, true, 860);
                const simd_float3 eye = simd_make_float3(-39570, 866, -28068);
                AtmosphereInput in;
                in.timeOfDay = tod;
                in.camera = {eye.x, eye.y, eye.z};
                if (t >= 2) {
                    in.lowestGround = 750;
                    in.weather.wet = 1, in.weather.cloudBase = 400, in.weather.cloudTopArg = eye.y;
                }
                const Atmosphere b = computeAtmosphere(in);
                std::vector<simd_float4> inputs;
                const F3 pts[4] = {{-39470, 850, -28068}, {-38570, 830, -27068}, {-29570, 1200, -28068}, {-39570, 2900, -18068}};
                for (const F3& p : pts) inputs.insert(inputs.end(), {simd_make_float4(p[0], p[1], p[2], 0), 0, 0, 0});
                inputs.insert(inputs.end(), {simd_make_float4(0.2f, 0.9f, 0.1f, 1), simd_make_float4(0.3f, 0.4f, 0.8f, 0),
                                             simd_make_float4(0.6f, 0.5f, 0.35f, 0.7f), simd_make_float4(0.8f, 0.3f, 20, 0)});
                inputs.insert(inputs.end(), {simd_make_float4(0.1f, 0.5f, 1.7f, 2), 0, 0, 0});
                const F3 dirs[2] = {{0.8f, 0.05f, 0.6f}, {-0.3f, 0.5f, 0.8f}};
                for (const F3& d : dirs) inputs.insert(inputs.end(), {simd_make_float4(d[0], d[1], d[2], 3), 0, 0, 0});
                const std::vector<simd_float4> out = r.shaderCheck(b, eye, inputs);
                if (out.size() != inputs.size() / 2) {
                    std::printf("  FAIL shader check did not run\n");
                    ++gFail;
                    break;
                }
                (void)a;
                const char* when = t == 0 ? "08:00" : t == 1 ? "20:30" : t == 2 ? "P105 06:10 haze" : "P105 12:00 haze";
                const float tolS = t >= 2 ? 1e-2f : 2e-3f;  // P105 cases reach inscatter ~30 toward the low sun
                char name[96];
                const F3 e = {eye.x, eye.y, eye.z};
                for (int k = 0; k < 4; ++k) {
                    F3 ext, insc;
                    scattering(b, e, pts[k], ext, insc);
                    const F3 ge = {out[size_t(k * 2)].x, out[size_t(k * 2)].y, out[size_t(k * 2)].z},
                             gi = {out[size_t(k * 2 + 1)].x, out[size_t(k * 2 + 1)].y, out[size_t(k * 2 + 1)].z};
                    std::snprintf(name, sizeof name, "%s GPU scattering point %d: extinction", when, k);
                    check3(name, ge, ext, 2e-3f);
                    std::snprintf(name, sizeof name, "%s GPU scattering point %d: inscatter", when, k);
                    check3(name, gi, insc, tolS);
                }
                {
                    auto nrm = [](F3 v) {
                        const float l = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
                        return F3{v[0] / l, v[1] / l, v[2] / l};
                    };
                    const F3 cpu = lighting(b, nrm({0.2f, 0.9f, 0.1f}), nrm({0.3f, 0.4f, 0.8f}), {0.6f, 0.5f, 0.35f}, {0.3f, 0.3f, 0.3f}, 20, 0.7f, 0.8f);
                    std::snprintf(name, sizeof name, "%s GPU lighting()", when);
                    check3(name, {out[8].x, out[8].y, out[8].z}, cpu, 2e-3f);
                }
                std::snprintf(name, sizeof name, "%s GPU tone curve at 0.1 / 0.5 / 1.7", when);
                check3(name, {out[10].x, out[10].y, out[10].z}, {toneCurve(0.1f), toneCurve(0.5f), toneCurve(1.7f)}, 1e-4f);
                for (int k = 0; k < 2; ++k) {
                    F3 d = dirs[k];
                    const float l = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
                    for (float& c : d) c /= l;
                    const float t2 = 0.9f * 50000.0f;  // the sky dome radius (0x7e1610: 0.9 x far, far forced to 50000)
                    F3 ext, insc;
                    scattering(b, e, {e[0] + d[0] * t2, e[1] + d[1] * t2, e[2] + d[2] * t2}, ext, insc);
                    std::snprintf(name, sizeof name, "%s GPU sky along direction %d", when, k);
                    check3(name, {out[size_t(12 + k * 2)].x, out[size_t(12 + k * 2)].y, out[size_t(12 + k * 2)].z}, insc, tolS);
                }
            }
        }
    }

    // ---- P102 [DEVIATION]: film grain (renderer.mm p102Grain*). Random by design: the GPU noise tile, noise and weight
    // are checked against this CPU mirror on fixed pixels and frames, then the noise's statistics (mean 0, spread, no
    // correlation between frames or neighbouring cells) and the luminance weight's ends.
    std::printf("Film grain (P102 deviation): GPU vs CPU mirror, statistics, luminance weight\n");
    @autoreleasepool {
        // The noise tile's texel (MeshRenderer::grainNoiseByte) and one bilinear tap of it, as p102GrainNoise.
        auto texel = [](int x, int y) { return float(MeshRenderer::grainNoiseByte(uint32_t(x & 255), uint32_t(y & 255))) / 255.0f; };
        auto noise = [&](float px, float py, float cell, float f) {
            const float ox = std::floor((f * 0.7548777f - std::floor(f * 0.7548777f)) * 256.0f) / 256.0f;
            const float oy = std::floor((f * 0.5698403f - std::floor(f * 0.5698403f)) * 256.0f) / 256.0f;
            const float tx = (px / (std::max(cell, 1.0f) * 256.0f) + ox) * 256.0f - 0.5f, ty = (py / (std::max(cell, 1.0f) * 256.0f) + oy) * 256.0f - 0.5f;
            const float ix = std::floor(tx), iy = std::floor(ty), fx = tx - ix, fy = ty - iy;
            const int x = int(ix), y = int(iy);
            const float a0 = texel(x, y) + (texel(x + 1, y) - texel(x, y)) * fx;
            const float a1 = texel(x, y + 1) + (texel(x + 1, y + 1) - texel(x, y + 1)) * fx;
            return (a0 + (a1 - a0) * fy) * 2.0f - 1.0f;
        };
        auto weight = [](float L) { L = std::clamp(L, 0.0f, 1.0f); return 2.598076f * std::sqrt(L) * (1.0f - L); };
        check1("grain weight at L = 0 (black stays black)", weight(0), 0, 1e-6f);
        check1("grain weight at L = 1/3 (peak)", weight(1.0f / 3.0f), 1.0f, 1e-5f);
        check1("grain weight at L = 1 (white stays white)", weight(1), 0, 1e-6f);
        check1("grain weight at L = 0.1", weight(0.1f), 0.739425f, 1e-5f);
        check1("film_grain = off", MeshRenderer::filmGrainAmplitude("off", 1), 0, 0);
        check1("film_grain = light", MeshRenderer::filmGrainAmplitude("light", 1), MeshRenderer::kFilmGrainLight, 0);
        check1("film_grain = strong", MeshRenderer::filmGrainAmplitude("strong", 1), MeshRenderer::kFilmGrainStrong, 0);
        check1("film_grain = 0.02", MeshRenderer::filmGrainAmplitude("0.02", 1), 0.02f, 1e-6f);
        check1("film_grain = bogus keeps the default", MeshRenderer::filmGrainAmplitude("bogus", 0.045f), 0.045f, 0);
        check1("soften = off", MeshRenderer::softenAmount("off", 1), 0, 0);
        check1("soften = light", MeshRenderer::softenAmount("light", 0), MeshRenderer::kSoftenLight, 0);
        check1("soften = strong", MeshRenderer::softenAmount("strong", 0), MeshRenderer::kSoftenStrong, 0);
        check1("soften = 2 clamps to 1", MeshRenderer::softenAmount("2", 0), 1, 0);
        id<MTLDevice> device = MTLCreateSystemDefaultDevice();
        if (device != nil) {
            ResourceLibrary lib;
            MeshRenderer r(device, 1, lib);
            const float cell = 2.0592593f, amp = 0.09f;  // the cell at 2224 lines (2224 / 1080), the strong amplitude
            // Items: 64 fixed probes, then 2 x 4096 pixels on a 64 x 64 grid 3.37 cells apart (frames 7 and 8).
            std::vector<simd_float4> in;
            auto item = [&](float px, float py, float f, simd_float3 c) {
                in.insert(in.end(), {simd_make_float4(px, py, f, 11), simd_make_float4(c.x, c.y, c.z, 0), simd_make_float4(amp, 0, cell, 0), 0});
            };
            for (int k = 0; k < 64; ++k)
                item(0.5f + float((k * 397) % 3420), 0.5f + float((k * 211) % 2224), float(k * 13 % 7 + 1), simd_make_float3(float(k % 9) / 8.0f, 0.3f, 0.2f));
            for (int fr = 7; fr <= 8; ++fr)
                for (int j = 0; j < 64; ++j)
                    for (int i = 0; i < 64; ++i) item(0.5f + float(i) * cell * 3.37f + 17, 0.5f + float(j) * cell * 3.37f + 29, float(fr), simd_make_float3(1.0f / 3, 1.0f / 3, 1.0f / 3));
            const AtmosphereInput ain;
            const std::vector<simd_float4> out = r.shaderCheck(computeAtmosphere(ain), simd_make_float3(0, 900, 0), in);
            if (out.size() != in.size() / 2) {
                std::printf("  FAIL grain shader check did not run\n");
                ++gFail;
            } else {
                int texOk = 0, noiseOk = 0, colOk = 0;
                float worst = 0;
                for (int k = 0; k < 64; ++k) {
                    const simd_float4 a = in[size_t(k * 4)], c = in[size_t(k * 4 + 1)];
                    const simd_float4 g0 = out[size_t(k * 2)], g1 = out[size_t(k * 2 + 1)];
                    texOk += std::fabs(g1.z - texel(int(a.x), int(a.y))) < 1e-6f;
                    const float n = noise(a.x, a.y, cell, a.z);
                    worst = std::max(worst, std::fabs(g1.x - n));
                    noiseOk += std::fabs(g1.x - n) <= 0.02f;  // the GPU's bilinear weights have 8 fractional bits
                    const float L = std::clamp(0.2126f * c.x + 0.7152f * c.y + 0.0722f * c.z, 0.0f, 1.0f);
                    const float want = std::clamp(c.x + amp * weight(L) * n, 0.0f, 1.0f);
                    colOk += std::fabs(g0.x - want) <= amp * 0.02f + 1e-5f;
                }
                check1("GPU grain noise tile = CPU (64 probes)", float(texOk), 64, 0);
                check1("GPU grain noise = CPU bilinear (64 probes, 0.02)", float(noiseOk), 64, 0);
                std::printf("  grain noise: largest GPU - CPU difference %.2g\n", worst);
                check1("GPU grain on colour = CPU (64 probes)", float(colOk), 64, 0);
                double m0 = 0, m1 = 0, s0 = 0, s1 = 0, c01 = 0, cx = 0;
                const size_t N = 4096, base = 64;
                for (size_t k = 0; k < N; ++k) {
                    const double a = out[(base + k) * 2 + 1].x, b = out[(base + N + k) * 2 + 1].x;
                    m0 += a, m1 += b, s0 += a * a, s1 += b * b, c01 += a * b;
                    if (k % 64 != 63) cx += a * out[(base + k + 1) * 2 + 1].x;
                }
                m0 /= N, m1 /= N;
                const double sd0 = std::sqrt(s0 / N - m0 * m0), sd1 = std::sqrt(s1 / N - m1 * m1);
                const double corr = (c01 / N - m0 * m1) / (sd0 * sd1), corrX = (cx / (N - 64) - m0 * m0) / (sd0 * sd0);
                std::printf("  grain noise: mean %.4f / %.4f, sd %.4f / %.4f, frame-to-frame r %.4f, next-sample r %.4f\n", m0, m1, sd0, sd1, corr, corrX);
                check1("grain noise mean ~ 0 (frame 7, 4096 samples)", std::fabs(m0) < 0.03 ? 0.0f : float(m0), 0, 0);
                check1("grain noise sd in [0.30, 0.50] (both frames)", sd0 >= 0.30 && sd0 <= 0.50 && sd1 >= 0.30 && sd1 <= 0.50 ? 1.0f : float(sd0), 1, 0);
                check1("grain frames uncorrelated (|r| < 0.05)", std::fabs(corr) < 0.05 ? 0.0f : float(corr), 0, 0);
                check1("grain samples 3.37 cells apart uncorrelated (|r| < 0.05)", std::fabs(corrX) < 0.05 ? 0.0f : float(corrX), 0, 0);
                // L = 1/3 grey: output sd = amplitude x noise sd (weight 1).
                double so = 0, mo = 0;
                for (size_t k = 0; k < N; ++k) {
                    const double v = out[(base + k) * 2].x;
                    mo += v, so += v * v;
                }
                mo /= N;
                check1("grain on grey 1/3: output sd = amplitude x noise sd", float(std::sqrt(so / N - mo * mo)), float(amp * sd0), 1e-3f);
                // Black and white stay put.
                std::vector<simd_float4> ends;
                for (int k = 0; k < 2; ++k)
                    ends.insert(ends.end(), {simd_make_float4(101.5f, 77.5f, 3, 11), simd_make_float4(float(k), float(k), float(k), 0), simd_make_float4(amp, 0, cell, 0), 0});
                const std::vector<simd_float4> eo = r.shaderCheck(computeAtmosphere(ain), simd_make_float3(0, 900, 0), ends);
                if (eo.size() == 4) {
                    check3("grain on black", {eo[0].x, eo[0].y, eo[0].z}, {0, 0, 0}, 0);
                    check3("grain on white", {eo[2].x, eo[2].y, eo[2].z}, {1, 1, 1}, 0);
                }
            }
        }
    }

    // ---- Ground lattice (p71): fsTerrain's chunk size from the vertex attribute and groundBand's lattice must put
    // every triangle corner on a fine-tile texel centre (FUEL's vertices, 0x7f9b50 / 0x7f92c0), or the decal uv
    // and the surface types are read from the wrong vertices.
    std::printf("Ground lattice: chunk size, lattice and fine-tile texels (groundBand, p71)\n");
    @autoreleasepool {
        id<MTLDevice> device = MTLCreateSystemDefaultDevice();
        if (device != nil) {
            ResourceLibrary lib;
            MeshRenderer r(device, 1, lib);
            const float sizes[4] = {24, 48, 96, 192}, spacing[4] = {1, 2, 4, 4};
            std::vector<simd_float4> in;
            for (int k = 0; k < 4; ++k) {
                const float sp = spacing[k];
                // A tile like WorldStreamer::buildGroundTile's: texel centres on multiples of the spacing.
                const float ox = std::round(-39570.0f / (16 * sp)) * 16 * sp - 128 * sp - sp / 2;
                const float oz = std::round(-28136.0f / (16 * sp)) * 16 * sp - 128 * sp - sp / 2;
                for (int q = 0; q < 3; ++q) {
                    const float px = -39570.37f + 7.13f * float(q) * float(k + 1), pz = -28136.71f + 5.29f * float(q);
                    // The rasteriser interpolates the size: a hair off the exact value.
                    in.insert(in.end(), {simd_make_float4(sizes[k] * (q == 1 ? 1.0001f : 0.9999f), px, pz, 6),
                                         simd_make_float4(ox, oz, 256 * sp, 1), 0, 0});
                }
            }
            const std::vector<simd_float4> out = r.shaderCheck(at(hm(8, 0)), simd_make_float3(-39570, 900, -28136), in);
            if (out.size() != in.size() / 2) {
                std::printf("  FAIL GPU ground lattice check did not run\n");
                ++gFail;
            } else {
                for (int k = 0; k < 4; ++k)
                    for (int q = 0; q < 3; ++q) {
                        const simd_float4 o = out[size_t(k * 3 + q) * 2];
                        const float got[4] = {o.x, o.y, o.z, o.w}, want[4] = {sizes[k], sizes[k] / 24, spacing[k], 0};
                        char name[96];
                        std::snprintf(name, sizeof name, "GPU %g m chunk point %d: C, lattice, tile spacing, corner off", sizes[k], q);
                        check(name, got, want, 4, 1e-2f);  // the corner offset: float32 at 40 km is ~4 mm
                    }
            }
        }
    }

    // ---- Effects (p30-effects.md): values from FUEL.exe run under Ghidra's emulator (notes/p30-work/v1.cmd, v2.cmd)
    std::printf("Effects: RNG, fade codes, fades, HSV, FX atlas (emulated FUEL.exe)\n");
    {
        namespace fs = std::filesystem;
        const std::string exe = (fs::path(dataDir) / "XFinalComponent" / "FUEL.exe").string();
        const sfx::Tables t = sfx::loadTables(exe, (fs::path(dataDir) / "CommonComponent" / "Param" / "pp001.tsc").string());
        if (!t.ok) {
            std::printf("  FAIL effect tables: %s\n", t.error.c_str());
            ++gFail;
        } else {
            sfx::Well512 rng(t.wellSeed);
            const uint32_t want[4] = {0x873a6bffu, 0xc0d9f1a8u, 0xbd6c6829u, 0x8b452b40u};  // 0x5fd710 x4 from the shipped state
            bool ok = true;
            uint32_t got[4];
            for (int i = 0; i < 4; ++i) got[i] = rng.next(), ok = ok && got[i] == want[i];
            ++(ok ? gPass : gFail);
            std::printf("  %-4s %-58s got %08x %08x %08x %08x\n", ok ? "ok" : "FAIL", "WELL512 0x5fd710 first four outputs", got[0], got[1], got[2],
                        got[3]);
            const struct { uint16_t code; F4 want; } codes[] = {
                {0x301, {0.01f, 0, 0.01f, 200}}, {0x403, {0, 0.01f, 0, 50}}, {0x101, {0.01f, 0, 0.01f, 100}}, {0xa02, {0, 100, 0, 400}}};
            for (const auto& c : codes) {
                char name[96];
                std::snprintf(name, sizeof name, "fade code 0x%03x -> GPU parameter (0x5fab60)", c.code);
                const auto p = sfx::codeParams(c.code);
                check4(name, {p[0], p[1], p[2], p[3]}, c.want, 1e-6f);
            }
            const float f[5] = {sfx::fadeValue(sfx::Fade::LogPuff, 0.5f, 2), sfx::fadeValue(sfx::Fade::OneMinusTSq, 0.5f, 2),
                                sfx::fadeValue(sfx::Fade::SqrtQuarter, 0.5f, 2), sfx::fadeValue(sfx::Fade::Quarter, 0.5f, 2),
                                sfx::fadeValue(sfx::Fade::OneMinusT, 0.5f, 2)};
            const float fw[5] = {0.998908366f, 0.5625f, 0.661437828f, 0.4375f, 0.75f};
            check("fades 0x5f47c0 0x5f4810 0x5edad0 0x5d2fd0 0x4beab0 at t .25", f, fw, 5, 1e-6f);
            // The GPU parameter of each code gives the CPU function (sfx.h GetParameterValue = 0x5fab60 inverse).
            float cg[6], cw[6];
            const sfx::Fade fs6[6] = {sfx::Fade::One, sfx::Fade::OneMinusT, sfx::Fade::OneMinusTSq, sfx::Fade::SqrtT, sfx::Fade::Quarter,
                                      sfx::Fade::SqrtQuarter};
            for (int i = 0; i < 6; ++i) {
                cg[i] = sfx::parameterValue(sfx::codeParams(sfx::fadeCode(fs6[i])), 0.3f);
                cw[i] = sfx::fadeValue(fs6[i], 0.3f, 1);
            }
            check("GetParameterValue(code) == CPU fade at t .3 (6 codes)", cg, cw, 6, 1e-5f);
            const auto hsv = sfx::rgbToHsv({0.6f, 0.5f, 0.35f, 1});
            check4("rgb -> hsv 0x5f4be0 (0.6, 0.5, 0.35)", {hsv[0], hsv[1], hsv[2], hsv[3]}, {0.6f, 0.4166667f, 0.6f, 1}, 1e-6f);
            const auto rgb = sfx::hsvToRgb({hsv[0], hsv[1], 0.22f, 1});
            // Expected from the disassembly (case 0: v, t, p): the emulator's floor/ftol (0x8e04c0 / 0x8e01d0) rounds
            // 0.6 to 1 because pcode ignores the rounding control, so its (0.2567, 0.22, 0.1283) is case 1.
            check4("hsv -> rgb 0x5f4a90 (v 0.22)", {rgb[0], rgb[1], rgb[2], rgb[3]}, {0.22f, 0.18333334f, 0.12833333f, 1}, 1e-6f);
            const bool names = t.atlas[15].material == "FX_PNEU" && t.atlas[42].material == "FX_ATLAS_0" && t.atlas[47].material == "FX_ATLAS_0";
            ++(names ? gPass : gFail);
            std::printf("  %-4s %-58s got %s %s %s\n", names ? "ok" : "FAIL", "FX atlas names 15, 42, 47 (0x8e6ae0 replay)", t.atlas[15].material.c_str(),
                        t.atlas[42].material.c_str(), t.atlas[47].material.c_str());
            auto rect = [&](int i) { return F4{t.atlas[size_t(i)].u0, t.atlas[size_t(i)].v0, t.atlas[size_t(i)].du, t.atlas[size_t(i)].dv}; };
            check4("FX atlas entry 15 rect", rect(15), {0, 0, 1, 1}, 1e-7f);
            check4("FX atlas entry 42 rect", rect(42), {0, 0, 0.5f, 0.5f}, 1e-7f);
            check4("FX atlas entry 47 rect (wheel dust)", rect(47), {0.5f, 0.75f, 0.25f, 0.25f}, 1e-7f);
            check4("FX atlas entry 49 rect", rect(49), {0.75f, 0.875f, 0.0625f, 0.0625f}, 1e-7f);
            // GPU GetParameterValue against the CPU copy.
            @autoreleasepool {
                id<MTLDevice> device = MTLCreateSystemDefaultDevice();
                if (device != nil) {
                    ResourceLibrary lib;
                    MeshRenderer r(device, 1, lib);
                    std::vector<simd_float4> in;
                    auto P = [](uint16_t c) { const auto p = sfx::codeParams(c); return simd_make_float4(p[0], p[1], p[2], p[3]); };
                    in.insert(in.end(), {simd_make_float4(0.3f, 0.7f, 0.95f, 4), P(0x301), P(0x403), P(0xa02)});
                    const std::vector<simd_float4> out = r.shaderCheck(at(hm(8, 0)), simd_make_float4(0, 900, 0, 1).xyz, in);
                    if (out.size() == 2) {
                        const float gw[3] = {sfx::parameterValue(sfx::codeParams(0x301), 0.3f), sfx::parameterValue(sfx::codeParams(0x403), 0.7f),
                                             sfx::parameterValue(sfx::codeParams(0xa02), 0.95f)};
                        const float gg[3] = {out[0].x, out[0].y, out[0].z};
                        check("GPU GetParameterValue (codes 0x301, 0x403, 0xa02)", gg, gw, 3, 1e-4f);
                    } else {
                        std::printf("  FAIL GPU sfx check did not run\n");
                        ++gFail;
                    }
                }
            }
        }
    }

    // ---- Weather fields (0x5b89e0): the altitude split and the generator maps at a few places
    std::printf("Weather amounts (0x5b89e0; maps 24 cloud, 23 precipitation, 22 fog, 21 wind, 16 sand)\n");
    {
        namespace fs = std::filesystem;
        try {
            DpcFile usa1((fs::path(dataDir) / "CommonComponent" / "WORLD" / "USA1.DPC").string());
            const auto wf = weatherFieldsFor(usa1);
            if (!wf || !wf->valid()) {
                std::printf("  FAIL weather maps missing\n");
                ++gFail;
            } else {
                const float a = wf->P(5440), b = wf->P(5441);
                // Halfway between PP5440 and PP5441: half rain, half snow of the precipitation (p26 3.1 vector).
                const WeatherAmounts mid = wf->sample(-39570, (a + b) * 0.5f, -28068);
                const float split[2] = {mid.precip > 0 ? mid.rain / mid.precip : 0.5f, mid.precip > 0 ? mid.snow / mid.precip : 0.5f};
                const float sw[2] = {0.5f, 0.5f};
                check("rain / snow split at (PP5440 + PP5441) / 2", split, sw, 2, 1e-5f);
                if (std::getenv("FUEL_WEATHER_SCAN")) {  // where the weather is (diagnostics)
                    float best[3] = {0, 0, 0};
                    double at[3][2] = {};
                    for (double x = -68000; x <= 68000; x += 1000)
                        for (double z = -68000; z <= 68000; z += 1000) {
                            const WeatherAmounts w = wf->sample(x, 500, z);
                            const float v[3] = {w.precip, w.cloud, w.fog};
                            for (int k = 0; k < 3; ++k)
                                if (v[k] > best[k]) best[k] = v[k], at[k][0] = x, at[k][1] = z;
                        }
                    std::printf("  info max precip %.3f at (%.0f, %.0f), cloud %.3f at (%.0f, %.0f), fog %.3f at (%.0f, %.0f)\n", best[0], at[0][0],
                                at[0][1], best[1], at[1][0], at[1][1], best[2], at[2][0], at[2][1]);
                }
                const double spots[][3] = {{-39570, 864, -28068}, {-20000, 900, 15000}, {30000, 1500, -30000}, {0, 3000, 0}, {40000, 400, 40000}};
                for (const auto& sp : spots) {
                    const WeatherAmounts w = wf->sample(sp[0], sp[1], sp[2]);
                    std::printf("  info at (%.0f, %.0f, %.0f): cloud %.3f precip %.3f rain %.3f snow %.3f fog %.3f wind %.1f m/s (%.2f, %.2f) sand %.3f humidity %.3f\n",
                                sp[0], sp[1], sp[2], w.cloud, w.precip, w.rain, w.snow, w.fog, w.windSpeed, w.wind[0], w.wind[2], w.sand, w.humidity);
                }
            }
        } catch (const std::exception& e) {
            std::printf("  FAIL weather: %s\n", e.what());
            ++gFail;
        }
    }

    // ---- Sky dome, cloud tank, hook arguments (p37-sky-clouds.md): emulated vectors notes/p37-work/v1.cmd
    std::printf("Sky and clouds: cloud tank 0x5fdbf0 / 0x5fde90 / 0x5fe370 (emulated FUEL.exe), vh_cloud on the GPU, hook arguments\n");
    {
        namespace fs = std::filesystem;
        const sfx::Tables t = sfx::loadTables((fs::path(dataDir) / "XFinalComponent" / "FUEL.exe").string(),
                                              (fs::path(dataDir) / "CommonComponent" / "Param" / "pp001.tsc").string());
        if (!t.ok) {
            std::printf("  FAIL tables: %s\n", t.error.c_str());
            ++gFail;
        } else {
            auto well = std::make_shared<sfx::Well512>(t.wellSeed);
            clouds::Params cp = clouds::Params::fromPP(t.pp);
            check1("cloud tank sight PP5206", cp.sight, 3000, 0);
            clouds::Tank tank(cp, [well]() { return well->uniform(); }, []() { return 0.5f; });
            tank.placeAt({-39570, 866, -28068});
            const clouds::V3 in = tank.randomPosition(true), ring = tank.randomPosition(false);
            check3("0x5fdbf0 inside the sight (WELL512 from the shipped state)", in, {-37731.832f, 0, -27035.982f}, 0.02f);
            check3("0x5fdbf0 on the ring [sight, 1.25 sight]", ring, {-42922.457f, 0, -27470.494f}, 0.02f);
            tank.placeAt({0, 0, 0});
            auto& cs = tank.mutableClouds();
            cs.assign(2, clouds::Cloud{});
            cs[1].pos = {100, 0, 100}, cs[1].active = true;
            cs[0].pos = {490, 0, 200}, cs[0].active = false;  // inactive clouds do not count
            const float acc[2] = {tank.accept({500, 0, 200}) ? 1.0f : 0.0f, tank.accept({150, 0, 200}) ? 1.0f : 0.0f};
            const float accW[2] = {1, 0};
            check("0x5fde90 accept (spacing PP5207 = 250)", acc, accW, 2, 0);
            const std::array<float, 28> k = clouds::drawConstants(0.7f, 0.6f, 3000, 600, 866);
            const float kW[28] = {0.7f, 0.6f, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, -100, 0, 0, 0, 2000, 3000, 2400, 1466};
            check("0x5fe370 draw constants C_USER0..6", k.data(), kW, 28, 1e-4f);
            check1("cloud layer height, clear (PP5203)", cloudLayerHeight(0, t.P(5202), t.P(5203)), 800, 0);
            check1("cloud layer height, overcast (PP5202)", cloudLayerHeight(1, t.P(5202), t.P(5203)), 400, 0);
            WeatherAmounts w;
            w.cloud = 0.5f, w.rain = 0.2f, w.humidity = 0.3f;
            const Weather h = weatherHook(w, 900, t.P(5202), t.P(5203));
            const float hv[4] = {h.cloudBase, h.cloudTopArg, h.wet, h.fog};
            const float hw[4] = {200, 900, 0.6f, 0.1f};
            check("0x5bba30 hook args: h0 = +0xf8 - 400, camera y, wet = max(clamp01(2 humidity), rain, snow), fog'", hv, hw, 4, 1e-5f);
            // vh_cloud corner on the GPU against the CPU copy.
            @autoreleasepool {
                id<MTLDevice> device = MTLCreateSystemDefaultDevice();
                if (device != nil) {
                    ResourceLibrary lib;
                    MeshRenderer r(device, 1, lib);
                    const simd_float3 eye = simd_make_float3(-39570, 866, -28068);
                    clouds::Cloud c;
                    c.pos = {-38900, 0, -27500}, c.width = 345, c.height = 330;
                    std::vector<simd_float4> gin;
                    const int corners[2][2] = {{-1, -1}, {1, 1}};
                    for (const auto& cr : corners)
                        gin.insert(gin.end(), {simd_make_float4(c.pos[0], c.pos[1], c.pos[2], 5), simd_make_float4(c.width, c.height, float(cr[0]), float(cr[1])),
                                               simd_make_float4(0.4f, 0.7f, 0, 0), simd_make_float4(k[24], k[25], k[26], k[27])});
                    const std::vector<simd_float4> out = r.shaderCheck(at(hm(12, 0)), eye, gin);
                    if (out.size() == 4) {
                        for (int i = 0; i < 2; ++i) {
                            const clouds::Corner cc = clouds::vertexCorner(c, corners[i][0], corners[i][1], {eye.x, eye.y, eye.z}, k[27], 3000, 0.4f);
                            const float g[5] = {out[size_t(i * 2)].x, out[size_t(i * 2)].y, out[size_t(i * 2)].z, out[size_t(i * 2)].w, out[size_t(i * 2 + 1)].x};
                            const float wv[5] = {cc.pos[0], cc.pos[1], cc.pos[2], cc.alpha, cc.shade};
                            char name[96];
                            std::snprintf(name, sizeof name, "GPU vh_cloud corner %d (position, alpha, shade)", i);
                            check(name, g, wv, 5, 0.05f);
                        }
                    } else {
                        std::printf("  FAIL GPU cloud check did not run\n");
                        ++gFail;
                    }
                }
            }
        }
    }

    // ---- p48: tyre tracks, splashes, engine particles (emulated vectors notes/p48-work/v1.cmd / v1.out)
    std::printf("Effects 2 (p48): ribbon 0x5df240 / 0x5df2c0, splash populate 0x5e0e90, Particles_Z curve 0x7d20d0 and direction 0x7d24f0 (emulated FUEL.exe)\n");
    {
        // Particles_Z 20-byte curve: keys (0; 1 2 3 4), (0.5; 3 0 1 2), (1; 1 1 1 1); cached index only moves forward.
        particles::Curve c;
        c.width = 4;
        c.keys = {0, 1, 2, 3, 4, 0.5f, 3, 0, 1, 2, 1, 1, 1, 1, 1};
        float o[4];
        int k = c.eval(1, 0.25f, o);
        const float g1[5] = {float(k), o[0], o[1], o[2], o[3]}, w1[5] = {1, 2, 1, 2, 3};
        check("0x7d20d0 t .25 from index 1 (index, values)", g1, w1, 5, 1e-6f);
        k = c.eval(1, 0.75f, o);
        const float g2[5] = {float(k), o[0], o[1], o[2], o[3]}, w2[5] = {2, 2, 0.5f, 1, 1.5f};
        check("0x7d20d0 t .75 from index 1", g2, w2, 5, 1e-6f);
        k = c.eval(2, 0.1f, o);
        const float g3[5] = {float(k), o[0], o[1], o[2], o[3]}, w3[5] = {3, 1, 1, 1, 1};
        check("0x7d20d0 t .1 from index 2 (past the end: last key)", g3, w3, 5, 1e-6f);
        check3("0x7d24f0 direction (a .4, b .7)", particles::direction(0.4f, 0.7f), {-0.29784358f, 0.921061f, -0.25087017f}, 1e-5f);

        // Ribbon of 4 points (u 0.1 at point 0, half width 0.2, normal +y, colour (.5 .4 .3 1), deep .2).
        sfx::Ribbon r;
        const sfx::V3 pts[4] = {{0, 0, 0}, {0.5f, 0, 0}, {1, 0, 0.1f}, {1.3f, 0.05f, 0.3f}};
        for (const auto& p : pts) {
            sfx::RibbonPoint q;
            q.pos = p, q.colour = {0.5f, 0.4f, 0.3f, 1}, q.deep = 0.2f, q.normal = {0, 1, 0};
            r.pts.push_back(q);
        }
        r.uBase = 0.1f, r.halfWidth = 0.2f;
        check1("0x5df240 end u (+0x60 + 0.3 x length)", sfx::ribbonEndU(r), 0.512172257f, 1e-5f);
        sfx::ribbonRebuild(r);
        const float gb[2] = {r.uBase, float(r.base)}, wb[2] = {0.25f, 1};
        check("0x5df2c0 +0x60 / +0x8c after the build", gb, wb, 2, 1e-6f);
        std::vector<sfx::DecalVertex> v;
        sfx::ribbonVertices(r, v);
        if (v.size() == 8) {
            // Emulated (half floats in FUEL: u, v, tangent x / y, rgba): vertex 2, 4, 6 and the alphas of all 8.
            const float g[] = {v[2].pos[0], v[2].pos[1], v[2].pos[2], v[2].u, v[2].tangent[0], v[2].tangent[2],
                               v[4].pos[0], v[4].pos[2], v[4].u, v[5].pos[0], v[5].pos[2], v[5].v,
                               v[6].pos[0], v[6].pos[1], v[6].pos[2], v[6].u, v[6].tangent[0], v[6].tangent[1], v[6].tangent[2]};
            const float w[] = {0.53922325f, 0, -0.19611613f, 0.25f, -0.5712f, -0.12970608f,
                               1.1109401f, -0.06641004f, 0.4028f, 0.88905996f, 0.26641005f, 1.0f,
                               1.41094f, 0.05f, 0.13358997f, 0.5117f, -0.2998f, -0.04998f, -0.2f};
            check("0x5df2c0 vertices 2, 4, 5, 6 (position, u / v, tangent)", g, w, 19, 2e-3f);
            float ga[8], wa[8] = {0, 0, 0, 0, 1, 1, 0, 0};
            for (int i = 0; i < 8; ++i) ga[i] = v[size_t(i)].rgba[3];
            check("0x5df2c0 vertex alphas (first build: pairs 0, 1 and the last 0)", ga, wa, 8, 0);
            const float gc[5] = {v[4].rgba[0], v[4].rgba[1], v[4].rgba[2], v[4].lift, v[4].deep}, wc[5] = {0.5f, 0.4f, 0.3f, 0.0625f, 0.2f};
            check("0x5df2c0 colour, lift 0.0625, deepness", gc, wc, 5, 1e-3f);
        } else {
            std::printf("  FAIL ribbon vertex count %zu\n", v.size());
            ++gFail;
        }

        namespace fs = std::filesystem;
        const sfx::Tables t = sfx::loadTables((fs::path(dataDir) / "XFinalComponent" / "FUEL.exe").string(),
                                              (fs::path(dataDir) / "CommonComponent" / "Param" / "pp001.tsc").string());
        if (t.ok) {
            wfx::SplashTank tank(wfx::SplashParams::fromPP(t.pp));
            auto well = std::make_shared<sfx::Well512>(t.wellSeed);
            tank.start(4, [well]() { return int32_t(well->next()); });
            float gs[8], ws[8] = {0.25f, -0.0625f, 0.1875f, -0.0625f, 0.125f, -0.0625f, 0.0625f, -0.0625f};
            for (int i = 0; i < 4; ++i) gs[2 * i] = tank.splashes()[size_t(i)].phase, gs[2 * i + 1] = tank.splashes()[size_t(i)].flip;
            check("0x5e0e90 splash phases (1 - i/N) PP5039 and u flips (WELL512)", gs, ws, 8, 1e-6f);
            const wfx::ThunderParams tp = wfx::ThunderParams::fromPP(t.pp);
            const float gt[6] = {tp.intervalMax, tp.distMin, tp.distMax, tp.intensity, tp.widthRatio, tp.above},
                        wt[6] = {5, 600, 3000, 3.5f, 0.9f, 200};
            check("thunder rows PP5231 / 5234 / 5235 / 5240 / 5243 / 5245", gt, wt, 6, 0);
        }
    }

    // ---- p57: materials (emulated on FUEL.exe: notes/p57-work/v1.cmd -> v1.out) and the global cube's faces
    std::printf("materials and the global cube map (p57)\n");
    {
        Material m;
        m.uvAngle = 0.5f, m.uvOffset = {0.1f, 0.2f}, m.uvScale = {2, 3};
        updateUvMatrix(m);
        const float w[6] = {1.7551651f, -1.4382766f, 0.9588511f, 2.6327477f, -0.7570081f, 0.10276449f};
        check("UpdateUVMatrix 0x695ee0 (angle .5, offset .1 .2, scale 2 3)", m.uvMatrix.data(), w, 6, 1e-6f);

        MaterialAnim::Track<2> t2;
        t2.keys = {{0, {0, 0}}, {0.5f, {1, 2}}, {1, {3, 0}}};
        float g[8];
        const float ts[3] = {0.25f, 0.75f, 2.0f};
        for (int i = 0; i < 3; ++i) {
            const auto v = evalTrack(t2, ts[i]);
            g[2 * i] = v[0], g[2 * i + 1] = v[1];
        }
        t2.mode = 8;
        const auto vh = evalTrack(t2, 0.75f);
        g[6] = vh[0], g[7] = vh[1];
        const float wc[8] = {0.5f, 1, 2, 1, 3, 0, 1, 2};
        check("Curve_Eval12 0x800000 at .25 / .75 / 2, hold at .75", g, wc, 8, 1e-6f);

        MaterialAnim::Track<1> t1;
        t1.keys = {{0, {0}}, {1, {1}}, {3, {-0.5f}}};
        const float gf[3] = {evalFloatTrack(t1, 0.25f), evalFloatTrack(t1, 2), evalFloatTrack(t1, 5)}, wf[3] = {0.25f, 0.25f, -0.5f};
        check("float track 0x72fa90 at .25 / 2 / 5 (i16 / 4096)", gf, wf, 3, 1e-6f);

        const std::vector<MaterialAnim::StepKey> st = {{0, 11}, {1, 22}, {2, 33}};
        const float gs[3] = {float(evalStepTrack(st, 0.5f)), float(evalStepTrack(st, 1.5f)), float(evalStepTrack(st, 7))}, wst[3] = {11, 22, 33};
        check("step track 0x800470 at .5 / 1.5 / 7", gs, wst, 3, 0);

        // 0x76a420 (decoded, not emulated): loop by the period, or stop at it with state 0x80 (play once).
        MaterialAnimClock c{0.9f, 0x40};
        c.advance(0.3f, 1.0f);
        MaterialAnimClock o{0.9f, 0x42};
        o.advance(0.3f, 1.0f);
        MaterialAnimClock idle{0.5f, 0x08};
        const bool moved = idle.advance(0.3f, 1.0f);
        const float gk[5] = {c.time, o.time, float(o.state), moved ? 1.0f : 0.0f, idle.time}, wk[5] = {0.2f, 1.0f, 0x82, 0, 0.5f};
        check("MaterialAnim clock 0x76a420: loop / once / not playing", gk, wk, 5, 1e-6f);

        // MaterialAnim_Z body (0x800570): no tracks = 69 bytes: material, period, state.
        std::vector<uint8_t> body(69, 0);
        const uint32_t mat = 0x12345678;
        const float period = 0.5f;
        std::memcpy(&body[60], &mat, 4);
        std::memcpy(&body[64], &period, 4);
        body[68] = 0x4c;
        const MaterialAnim a = parseMaterialAnimBody(body);
        const float ga[4] = {float(a.material == mat), a.period, float(a.state), float(a.empty())}, wa[4] = {1, 0.5f, 0x4c, 1};
        check("MaterialAnim_Z body: material, period, state (69 bytes, no keys)", ga, wa, 4, 0);

        // UpdateFlags 0x695f70 + MeshPc_Z::Draw 0x848880: mode, ALPHAREF, two-sided, cubemap.
        auto ds = [](uint32_t flags, float alpha, uint8_t kind, float f0) {
            Material mm;
            mm.flags = flags, mm.alpha = alpha, mm.fresnel = {f0, 0};
            const MaterialDrawState d = materialDrawState(mm, kind);
            return std::array<float, 4>{float(d.mode), d.alphaRef * 255.0f, float(d.twoSided), float(d.cubemap)};
        };
        const std::array<float, 4> cases[] = {ds(0, 1, 0, 0),         ds(0, 1, 2, 0),        ds(0, 0.5f, 0, 0),   ds(0x1000, 1, 0, 0),
                                              ds(0x180000 | 0x100000, 1, 2, 0), ds(0x11, 1, 2, 0.03f), ds(0x8000, 1, 1, 0), ds(0x12, 1, 1, 0)};
        const float wd[32] = {0, 0, 0, 0, 2, 0, 1, 0, 2, 0, 1, 0, 2, 0, 1, 0, 1, 128, 0, 0, 0, 0, 0, 1, 1, 0, 1, 0, 1, 0, 0, 0};
        float gd[32];
        for (int i = 0; i < 8; ++i)
            for (int k = 0; k < 4; ++k) gd[i * 4 + k] = cases[i][size_t(k)];
        check("draw state (mode, ALPHAREF, two-sided, cubemap) of 8 flag sets", gd, wd, 32, 0);

        // P72: blended phases of MeshPc_Z::Draw 0x848880 (preset byte +0xd20): phase, fade pre-pass, colour z write,
        // ignores the fade. Flag sets: plain 0x20, 0x10000, 0x40000, 0x40000|0x10000, code 0x17, code 0x12 + 0x40000,
        // two-sided 0x8000, 0x8000|0x200000, unlit 0x1000, code 1.
        const uint32_t pf[10] = {0x20, 0x10020, 0x40020, 0x50020, 0x37, 0x40032, 0x8020, 0x208020, 0x1020, 0x21};
        const float wph[40] = {0x13, 1, 0, 0,  0x12, 1, 0, 0,  0x0d, 1, 1, 0,  0x0c, 1, 1, 0,  0x13, 1, 0, 1,
                               0x13, 1, 1, 1,  0x13, 0, 0, 0,  0x13, 1, 0, 0,  0x13, 0, 0, 0,  0x13, 1, 0, 1};
        float gph[40];
        for (int i = 0; i < 10; ++i) {
            gph[i * 4] = float(blendPhase(pf[i]));
            gph[i * 4 + 1] = float(fadePrePass(pf[i]));
            gph[i * 4 + 2] = float(blendZWrite(pf[i]));
            gph[i * 4 + 3] = float(ignoresFade(pf[i] & 0x1f));
        }
        check("blended phase / fade pre-pass / z write / no fade of 10 flag sets (0x848880)", gph, wph, 40, 0);

        // Cube faces (encodeCube): the face basis must give the Metal / GL cube lookup (major axis, sc, tc):
        // direction F + 0.5 R + 0.25 U lands on that face at s = 0.75, t = 0.375.
        float gcube[18], wcube[18];
        for (int f = 0; f < 6; ++f) {
            simd_float3 F, R, U;
            cubeFaceBasis(f, F, R, U);
            const simd_float3 d = F + 0.5f * R + 0.25f * U;
            const float ax = std::fabs(d.x), ay = std::fabs(d.y), az = std::fabs(d.z);
            int face;
            float sc, tc, ma;
            if (ax >= ay && ax >= az) face = d.x > 0 ? 0 : 1, ma = ax, sc = d.x > 0 ? -d.z : d.z, tc = -d.y;
            else if (ay >= az) face = d.y > 0 ? 2 : 3, ma = ay, sc = d.x, tc = d.y > 0 ? d.z : -d.z;
            else face = d.z > 0 ? 4 : 5, ma = az, sc = d.z > 0 ? d.x : -d.x, tc = -d.y;
            gcube[3 * f] = float(face), gcube[3 * f + 1] = (sc / ma + 1) * 0.5f, gcube[3 * f + 2] = (tc / ma + 1) * 0.5f;
            wcube[3 * f] = float(f), wcube[3 * f + 1] = 0.75f, wcube[3 * f + 2] = 0.375f;
        }
        check("cube faces: F + R/2 + U/4 -> (face, s .75, t .375)", gcube, wcube, 18, 1e-6f);
    }

    // ---- P74: the car occlusion omni (Vehicle_CarOcclusion 0x5cc410, notes p74-lighting.md 1). The box rows come
    // from fillCarOcclusion (Omni_Z::vf26 0x690700: Scale(1/e) x node world, 3x3 transposed); the reference is
    // FUEL's local = (w - T) R^T / e, then 1 - f(x) f(z) f(y) with the test spot map f(u) = 2 (128 u - 0.5) / 255
    // (bilinear over texels 2i / 255). Box A turned 30 deg about y, box B axis-aligned and larger: a point in both
    // takes A (the first), a point only in B takes B, a point in neither keeps 1.
    std::printf("Car occlusion omni (0x5cc410, scat_mesh / ground bOmni)\n");
    {
        id<MTLDevice> device = MTLCreateSystemDefaultDevice();
        if (device != nil) {
            ResourceLibrary lib;
            MeshRenderer r(device, 1, lib);
            const double th = 30.0 * M_PI / 180.0, T[3] = {-39570.3, 905.2, -28136.7};
            const double ax[3][3] = {{std::cos(th), 0, -std::sin(th)}, {0, 1, 0}, {std::sin(th), 0, std::cos(th)}};
            const double eA[3] = {1.316, 1.216, 1.769}, eB[3] = {2.0, 2.0, 2.0}, TB[3] = {T[0] + 0.3, T[1], T[2]};
            MeshRenderer::CarOcclusion A, B;
            A.world = simd_matrix(simd_make_float4(float(ax[0][0]), float(ax[0][1]), float(ax[0][2]), 0),
                                  simd_make_float4(float(ax[1][0]), float(ax[1][1]), float(ax[1][2]), 0),
                                  simd_make_float4(float(ax[2][0]), float(ax[2][1]), float(ax[2][2]), 0),
                                  simd_make_float4(float(T[0]), float(T[1]), float(T[2]), 1));
            A.extents = {float(eA[0]), float(eA[1]), float(eA[2])};
            B.world = matrix_identity_float4x4;
            B.world.columns[3] = simd_make_float4(float(TB[0]), float(TB[1]), float(TB[2]), 1);
            B.extents = {2, 2, 2};
            auto worldOf = [&](const double l[3]) {  // a point given in box A's local [-1, 1] coordinates
                double w[3] = {T[0], T[1], T[2]};
                for (int k = 0; k < 3; ++k)
                    for (int j = 0; j < 3; ++j) w[j] += l[k] * eA[k] * ax[k][j];
                return simd_make_float3(float(w[0]), float(w[1]), float(w[2]));
            };
            auto f = [](double u) { return std::clamp(2.0 * (128.0 * u - 0.5) / 255.0, 0.0, 254.0 / 255.0); };
            auto want = [&](simd_float3 p) {  // FUEL: first box holding p
                const double w[3] = {p.x, p.y, p.z};
                for (int b = 0; b < 2; ++b) {
                    const double* t = b == 0 ? T : TB;
                    const double* e = b == 0 ? eA : eB;
                    double l[3];
                    for (int k = 0; k < 3; ++k) {
                        double d = 0;
                        for (int j = 0; j < 3; ++j) d += (w[j] - t[j]) * (b == 0 ? ax[k][j] : (j == k ? 1.0 : 0.0));
                        l[k] = d / e[k];
                    }
                    if (std::fabs(l[0]) < 1 && std::fabs(l[1]) < 1 && std::fabs(l[2]) < 1)
                        return float(1.0 - f(l[0] * 0.5 + 0.5) * f(l[2] * 0.5 + 0.5) * f(l[1] * 0.5 + 0.5));
                }
                return 1.0f;
            };
            const double l1[3] = {0.2, -0.4, 0.6}, l2[3] = {1.1, 0.1, -0.2}, l3[3] = {-0.05, 0.02, 0.1};
            const simd_float3 pts[4] = {worldOf(l1), worldOf(l2), worldOf(l3), worldOf(l1) + simd_make_float3(0, 30, 0)};
            std::vector<simd_float4> in;
            for (const simd_float3& p : pts) in.insert(in.end(), {simd_make_float4(p, 7), 0, 0, 0});
            const std::vector<simd_float4> out = r.shaderCheck(at(hm(12, 0)), simd_make_float3(-39570, 910, -28130), in, {A, B});
            if (out.size() != in.size() / 2) {
                std::printf("  FAIL GPU car occlusion check did not run\n");
                ++gFail;
            } else {
                const char* names[4] = {"in A (and B)", "outside A, in B", "near A's centre", "above both"};
                for (int i = 0; i < 4; ++i) {
                    const float got[2] = {out[size_t(i) * 2].x, out[size_t(i) * 2].y}, w[2] = {want(pts[i]), 2};
                    char name[96];
                    std::snprintf(name, sizeof name, "GPU car occlusion 1 - fff, boxes: point %s", names[i]);
                    check(name, got, w, 2, 1e-2f);  // the sampler's 8-bit filter weights
                }
            }
        }
    }

    // ---- P74: the sun / moon sprite quad (0x7e20f0 + SkyDome_Update 0x7e2690). Reference: the quad point
    // (x, 0.9, z) (units of R, half size 0.05) turned by the quaternion Quat_FromArc(up -> L) = normalize(1 + up.L,
    // up x L); its direction must give u = z / 0.1 + 0.5, v = -x / 0.1 + 0.5 (0x7e20f0's corner uvs).
    std::printf("Sun sprite quad (0x7e20f0)\n");
    {
        id<MTLDevice> device = MTLCreateSystemDefaultDevice();
        if (device != nil) {
            ResourceLibrary lib;
            MeshRenderer r(device, 1, lib);
            const double Lr[3] = {0.3, 0.8, -0.4}, ln = std::sqrt(0.09 + 0.64 + 0.16);
            const double L[3] = {Lr[0] / ln, Lr[1] / ln, Lr[2] / ln};
            // q = (w, v) = normalize(1 + L.y, (L.z, 0, -L.x))
            double qw = 1 + L[1], qx = L[2], qy = 0, qz = -L[0];
            const double qn = std::sqrt(qw * qw + qx * qx + qy * qy + qz * qz);
            qw /= qn, qx /= qn, qy /= qn, qz /= qn;
            auto rot = [&](const double p[3], double o[3]) {  // o = q p q*
                const double tx = 2 * (qy * p[2] - qz * p[1]), ty = 2 * (qz * p[0] - qx * p[2]), tz = 2 * (qx * p[1] - qy * p[0]);
                o[0] = p[0] + qw * tx + (qy * tz - qz * ty);
                o[1] = p[1] + qw * ty + (qz * tx - qx * tz);
                o[2] = p[2] + qw * tz + (qx * ty - qy * tx);
            };
            const double pts[3][2] = {{0, 0}, {0.03, -0.02}, {-0.045, 0.04}};
            std::vector<simd_float4> in;
            for (const auto& xz : pts) {
                const double p[3] = {xz[0], 0.9, xz[1]};
                double d[3];
                rot(p, d);
                in.insert(in.end(), {simd_make_float4(float(d[0]), float(d[1]), float(d[2]), 8), simd_make_float4(float(L[0]), float(L[1]), float(L[2]), 0), 0, 0});
            }
            // a direction 10 deg off the sun: outside the 3.2 deg quad
            in.insert(in.end(), {simd_make_float4(float(L[0] + 0.18), float(L[1]), float(L[2]), 8), simd_make_float4(float(L[0]), float(L[1]), float(L[2]), 0), 0, 0});
            const std::vector<simd_float4> out = r.shaderCheck(at(hm(12, 0)), simd_make_float3(0, 900, 0), in);
            if (out.size() != in.size() / 2) {
                std::printf("  FAIL GPU sun sprite check did not run\n");
                ++gFail;
            } else {
                for (int i = 0; i < 4; ++i) {
                    const float got[2] = {out[size_t(i) * 2].x, out[size_t(i) * 2].y};
                    const float want[2] = {i < 3 ? float(pts[i][1] / 0.1 + 0.5) : -1.0f, i < 3 ? float(-pts[i][0] / 0.1 + 0.5) : -1.0f};
                    char name[96];
                    std::snprintf(name, sizeof name, "GPU sun sprite uv, quad point %d", i);
                    check(name, got, want, 2, 1e-4f);
                }
            }
        }
    }

    // ---- P77: blob shadows (FakeShadows_G, notes p77-shadows-lights.md 1). References in double from the asm:
    // 0x509c00 / 0x509cb0 record, 0x5cb8c0 alpha factor, 0x5ca7a0 body sizes, 0x5d86b0 / 0x5d8420 gates and fades,
    // 0x5095b0 corner nudge (GPU).
    std::printf("Blob shadows (FakeShadows_G 0x509c00 / 0x5d86b0 / 0x5d8420 / 0x5095b0)\n");
    {
        using blob::V3;
        auto ref = [](const double p[3], const double n[3], const double f[3], double side, double fwd, double out[9]) {
            double v[3] = {f[0], f[1], f[2]};
            const double h = f[0] * f[0] + f[2] * f[2];
            if (h > 1e-6) v[0] = f[0] / std::sqrt(h), v[1] = 0, v[2] = f[2] / std::sqrt(h);
            double b[3] = {-(n[1] * v[2] - n[2] * v[1]), -(n[2] * v[0] - n[0] * v[2]), -(n[0] * v[1] - n[1] * v[0])};
            double l = std::sqrt(b[0] * b[0] + b[1] * b[1] + b[2] * b[2]);
            for (double& x : b) x /= l;
            double a[3] = {n[1] * b[2] - n[2] * b[1], n[2] * b[0] - n[0] * b[2], n[0] * b[1] - n[1] * b[0]};
            l = std::sqrt(a[0] * a[0] + a[1] * a[1] + a[2] * a[2]);
            for (int k = 0; k < 3; ++k) out[k] = a[k] / l * fwd, out[3 + k] = b[k] * side, out[6 + k] = p[k] + n[k] * 0.01;
        };
        const double p[3] = {-39565.5, 863.7, -28070.5};
        const double n1[3] = {0.1, 0.98, -0.17}, f1[3] = {0.6, 0.2, 0.77};  // tilted body, heading with pitch
        const double n2[3] = {0, 1, 0}, f2[3] = {0.0, 1.0, 0.0005};          // vertical f: kept unflattened
        const double* cases[2][2] = {{n1, f1}, {n2, f2}};
        for (int c = 0; c < 2; ++c) {
            const double* n = cases[c][0];
            const double* f = cases[c][1];
            const double nl = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
            const double nn[3] = {n[0] / nl, n[1] / nl, n[2] / nl};
            double w[9];
            ref(p, nn, f, 1.10, 2.19, w);
            const blob::Record r = blob::makeRecord({float(p[0]), float(p[1]), float(p[2])}, {float(nn[0]), float(nn[1]), float(nn[2])},
                                                    {float(f[0]), float(f[1]), float(f[2])}, 1.10f, 2.19f, 0.5f);
            const float got[9] = {r.a[0], r.a[1], r.a[2], r.b[0], r.b[1], r.b[2], r.centre[0], r.centre[1], r.centre[2]};
            float want[9];
            for (int k = 0; k < 9; ++k) want[k] = float(w[k]);
            check(c == 0 ? "0x509cb0 record a, b, centre (tilted normal, pitched heading)" : "0x509cb0 record (vertical heading, not flattened)", got, want, 9,
                  2e-3f);
        }
        const float fa[4] = {blob::alphaFactor(0.156f, 0.0f), blob::alphaFactor(0.9f, 0.0f), blob::alphaFactor(-0.2f, 0.5f), blob::alphaFactor(0.0f, 0.95f)};
        const float fw[4] = {0.844f, 0.3f, 0.5f, 0.2f};
        check("0x5cb8c0 alpha factor clamp(1 - inv, .3, 1) x clamp(1 - fog, .2, 1)", fa, fw, 4, 1e-6f);
        const auto s4 = blob::bodySizes(0.9f, 1.6f, {{0.6f, 0.3f, 1.3f}, {0.6f, 0.3f, -1.2f}, {-0.62f, 0.3f, 1.3f}, {-0.6f, 0.3f, -1.2f}});
        const auto s2 = blob::bodySizes(0.4f, 1.1f, {{0, 0.3f, 0.7f}, {0, 0.3f, -0.75f}});
        const float gs[4] = {s4[0], s4[1], s2[0], s2[1]}, ws[4] = {1.22f, 2.5f, 0.6f, 1.45f};
        check("0x5ca7a0 body sizes: 4 wheels (track, wheelbase), 2 wheels (1.5 min box, wheelbase)", gs, ws, 4, 1e-6f);
        // Producer: an opponent quad, all wheels loaded, 10 m from the camera: 4 wheel records then the body; fades
        // after 2 frames of 0.1 s = 0.6; wheel alpha 0.5 x fade; body alpha fade x 1.0; sizes PP 51/52, 1.6 PP 49/50.
        blob::Producer prod;
        blob::VehicleInput in;
        in.cameraDistance = 10;
        in.contacts = {{1, 0, 1}, {1, 0, -1}, {-1, 0, 1}, {-1, 0, -1}};
        in.loads = {0.1f, 0.1f, 0.1f, 0.1f};
        in.bodySize = {1.2f, 2.0f};
        in.pp = {0.3f, 0.32f, 0.2f, 0.22f};
        std::vector<blob::Record> recs;
        prod.update(1, 0.1f, 0, in, recs);
        recs.clear();
        prod.update(1, 0.1f, 0, in, recs);
        const float gp[10] = {float(recs.size()), recs[0].alpha, recs[0].side, recs[0].fwd, recs[1].side, recs[1].fwd, recs[4].alpha, recs[4].side,
                              recs[4].fwd, recs[4].centre[1]};
        const float wp[10] = {5, 0.3f, 0.2f, 0.48f, 0.22f, 0.512f, 0.6f, 0.96f, 1.6f, 0.11f};
        check("0x5d86b0 / 0x5d8420 records: count, wheel 0 alpha / side / fwd, wheel 1 side / fwd, body alpha / side / fwd / y", gp, wp, 10, 1e-5f);
        // Gates: 25 m away -> no wheel blobs (fades reset), body kept; behind 210 m -> nothing; wheel 2 unloaded ->
        // its blob skipped; the player -> nothing; a bike (2 wheels) -> body factors 0.5 / 0.6.
        recs.clear();
        in.cameraDistance = 25;
        prod.update(1, 0.1f, 0, in, recs);
        const float g1 = float(recs.size());
        recs.clear();
        in.cameraDistance = 210;
        prod.update(1, 0.1f, 0, in, recs);
        const float g2 = float(recs.size());
        recs.clear();
        in.cameraDistance = 210;
        prod.update(1, 0.1f, 4, in, recs);  // race intro: no distance / load gates
        const float g3 = float(recs.size());
        recs.clear();
        in.cameraDistance = 10, in.loads[2] = 0;
        prod.update(1, 0.1f, 0, in, recs);
        const float g4 = float(recs.size()), g4a = recs[0].alpha;  // wheel fade: 0 after 210 m, 0.3 intro, 0.6 now -> 0.3
        recs.clear();
        in.player = true;
        prod.update(0, 0.1f, 0, in, recs);
        const float g5 = float(recs.size());
        recs.clear();
        blob::VehicleInput bike = in;
        bike.player = false, bike.contacts.resize(2), bike.loads = {0.1f, 0.1f}, bike.bodySize = {0.6f, 1.45f};
        blob::Producer pb;
        for (int k = 0; k < 4; ++k) recs.clear(), pb.update(2, 0.1f, 0, bike, recs);
        const float g6a = recs.back().alpha, g6f = recs.back().fwd, g6s = recs.back().side;
        const float gg[9] = {g1, g2, g3, g4, g4a, g5, g6a, g6f, g6s}, ww[9] = {1, 0, 5, 4, 0.3f, 0, 0.6f, 0.725f, 0.48f};
        check("gates: 25 m, 210 m, intro, unloaded wheel (+ fade), player, bike body alpha / fwd / side", gg, ww, 9, 1e-5f);

        id<MTLDevice> device = MTLCreateSystemDefaultDevice();
        if (device != nil) {
            ResourceLibrary lib;
            MeshRenderer r(device, 1, lib);
            const double P[3][3] = {{-39565.0, 863.7, -28070.0}, {-39565.0, 863.7, -28070.0}, {10.0, 0.0, 5.0}};
            const double E[3][3] = {{-39572.5, 865.7, -28069.8}, {-39565.0, 873.7, -28070.0}, {10.0, 0.0, -20.0}};
            const double U[3][3] = {{0.0, 0.966, 0.259}, {0.0, 0.0, 1.0}, {0.0, 1.0, 0.0}};
            std::vector<simd_float4> gin;
            for (int i = 0; i < 3; ++i)
                gin.insert(gin.end(), {simd_make_float4(float(P[i][0]), float(P[i][1]), float(P[i][2]), 9),
                                       simd_make_float4(float(E[i][0]), float(E[i][1]), float(E[i][2]), 0), simd_make_float4(float(U[i][0]), float(U[i][1]), float(U[i][2]), 0), 0});
            const std::vector<simd_float4> out = r.shaderCheck(at(hm(12, 0)), simd_make_float3(0, 900, 0), gin);
            if (out.size() != gin.size() / 2) {
                std::printf("  FAIL GPU blob nudge check did not run\n");
                ++gFail;
            } else {
                for (int i = 0; i < 3; ++i) {
                    double d[3] = {E[i][0] - P[i][0], E[i][1] - P[i][1], E[i][2] - P[i][2]};
                    const double l = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
                    for (double& x : d) x /= l;
                    const double ul = std::sqrt(U[i][0] * U[i][0] + U[i][1] * U[i][1] + U[i][2] * U[i][2]);
                    const double dy = std::fabs((d[0] * U[i][0] + d[1] * U[i][1] + d[2] * U[i][2]) / ul);
                    const double k = (1 - dy) * (1 - dy) * 0.15 + 0.05;
                    const float want[3] = {float(P[i][0] + d[0] * k), float(P[i][1] + d[1] * k), float(P[i][2] + d[2] * k)};
                    const float got[3] = {out[size_t(i) * 2].x, out[size_t(i) * 2].y, out[size_t(i) * 2].z};
                    const V3 cpu = blob::nudge({float(P[i][0]), float(P[i][1]), float(P[i][2])}, {float(E[i][0]), float(E[i][1]), float(E[i][2])},
                                               {float(U[i][0] / ul), float(U[i][1] / ul), float(U[i][2] / ul)});
                    char name[96];
                    std::snprintf(name, sizeof name, "GPU 0x5095b0 corner nudge %d", i);
                    check(name, got, want, 3, 5e-3f);  // float32 at 40 km: ~4 mm
                    std::snprintf(name, sizeof name, "CPU 0x5095b0 corner nudge %d", i);
                    check(name, cpu.data(), want, 3, 5e-3f);
                }
            }
        }
    }

    // ---- PK22: the lamp groups (0x69da00 / 0x5cc780) and the live dirt's C_USER0 select (vsMain / fsMain).
    std::printf("PK22 vehicle lamp groups and dirt\n");
    {
        namespace vl = fuel::vlinks;
        const float gl[5] = {float(vl::lampOfMaterial(asoboCrc("LIGHT_BACK"))), float(vl::lampOfMaterial(asoboCrc("LIGHT_STOP"))),
                             float(vl::lampOfMaterial(asoboCrc("LIGHT_POSITION"))), float(vl::lampOfMaterial(asoboCrc("LIGHT_HEAD"))),
                             float(vl::lampOfMaterial(asoboCrc("LIGHT_FOG")))};
        const float wl[5] = {0, 1, 2, 3, -1};
        check("PK22 lamp group of the LIGHT_* material names", gl, wl, 5, 0);
        const auto e = vl::lampEmissive({1.0f, 0.2f, 0.0f});
        check3("PK22 0x69da00 clone colour x 5", {e[0], e[1], e[2]}, {5.0f, 1.0f, 0.0f}, 0);
        auto dirtSel = [](float global, float packed) { return global > 0.0f || packed <= 0.0f ? global : packed - 1.0f; };  // fsMain
        const float gd[3] = {dirtSel(0, 1.3f), dirtSel(0.5f, 1.3f), dirtSel(0, 0)}, wd[3] = {0.3f, 0.5f, 0};
        check("PK22 dirt: the vehicle's C_USER0, --dirt wins, unset = global", gd, wd, 3, 1e-6f);
        // Data: QUAD.DPC's LIGHT_* materials (link name) by group: 7 BACK, 39 STOP, 39 POSITION, 38 HEAD.
        try {
            DpcFile quad((std::filesystem::path(dataDir) / "CommonComponent" / "DATAS" / "QUAD.DPC").string());
            float n[4] = {0, 0, 0, 0};
            for (uint32_t id : quad.ids(cls::Material)) {
                const DpcFile::Entry* en = quad.find(id);
                if (en == nullptr) continue;
                const std::vector<uint8_t> link = quad.link(*en);
                uint32_t name = 0;
                if (link.size() >= 4) std::memcpy(&name, link.data(), 4);
                const int g = vl::lampOfMaterial(name);
                if (g >= 0) n[g] += 1;
            }
            const float wn[4] = {7, 39, 39, 38};
            check("PK22 QUAD.DPC LIGHT_* materials (BACK, STOP, POSITION, HEAD)", n, wn, 4, 0);
        } catch (const std::exception& ex) {
            std::printf("  skip PK22 QUAD.DPC: %s\n", ex.what());
        }
    }
    // ---- PK22 end

    // ---- P77: vehicle headlights (V:HEADLIGHT + Omni_Z::vf43 0x798050 + omni.phl, notes p77-shadows-lights.md 2).
    // Reference: omni.phl in double for the QUAD_ROAD_2 headlight (Omni_Z b1f00075: colour 20 / 17.25 / 9.80, r 5,
    // R 250, cut -3.8637 / 3.7320) and a second light; day factor 0x5b98d0 and the 0x5cbde0 switch.
    std::printf("Vehicle headlights (0x5cbde0 / 0x5b98d0 / Omni_Z::vf43 omni.phl)\n");
    {
        const double ts[7] = {0, 19800, 21600, 23400, 43200, 64800, 75600};
        float gd[7], wd[7] = {0, 0, 0.5f, 1, 1, 0.5f, 0};
        for (int i = 0; i < 7; ++i) gd[i] = omni::dayFactor(ts[i]);
        check("0x5b98d0 day factor at 00:00 05:30 06:00 06:30 12:00 18:00 21:00", gd, wd, 7, 1e-6f);
        const float gs[4] = {float(omni::headlightsOn(23300)), float(omni::headlightsOn(23400)), float(omni::headlightsOn(63000)),
                             float(omni::headlightsOn(63100))};
        const float ws[4] = {1, 0, 0, 1};
        check("0x5cbde0 headlights on at 06:28, off 06:30 / 17:30, on 17:31", gs, ws, 4, 0);
        omni::Params qp;
        qp.colour = {20.0f, 17.2549f, 9.80392f}, qp.inner = 5, qp.outer = 250, qp.cutZ = -3.8637f, qp.cutW = 3.73205f;
        const auto val = omni::value(qp);
        const float gv[2] = {val[0], val[1]}, wv[2] = {25.0f, float(1.0 / 62475.0)};
        check("gDOmniValue (r^2, 1 / (R^2 - r^2))", gv, wv, 2, 1e-9f);
        // omni.phl in double
        auto ref = [&](const double P[3], const double N[3], const double p[3], const double dir[3], const double up[3], double out[3]) {
            double o[3] = {p[0] - P[0], p[1] - P[1], p[2] - P[2]};
            const double d = o[0] * o[0] + o[1] * o[1] + o[2] * o[2];
            double v[3];
            for (int k = 0; k < 3; ++k) v[k] = o[k] / std::sqrt(d);
            auto sat = [](double x) { return std::clamp(x, 0.0, 1.0); };
            const double f = dir[0] * v[0] + dir[1] * v[1] + dir[2] * v[2];
            double n = sat(dir[0] * N[0] + dir[1] * N[1] + dir[2] * N[2]);
            if (f < 0) n = sat(v[0] * N[0] + v[1] * N[1] + v[2] * N[2]);
            double r = sat(1 - (d - 25.0) / 62475.0);
            r = std::pow(r, 8.0);
            const double u = sat(0.33 - (v[0] * up[0] + v[1] * up[1] + v[2] * up[2]) * 0.67 - d * 0.5);
            const double c = 2 * u + (1 - 2 * u) * sat(1 - (f * -3.8637 + 3.73205));
            const double col[3] = {20.0, 17.2549, 9.80392};
            for (int k = 0; k < 3; ++k) out[k] = r * n * c * col[k];
        };
        // the light 0.835 m up at the quad's nose, cone axis forward and 16 deg down: dir = -(0, -0.28, 0.96)
        const double L[3] = {-39570.0, 864.0, -28070.0}, D[3] = {0.0, 0.28, -0.96};
        const double up[3] = {0, 1, 0}, Ny[3] = {0, 1, 0}, Nb[3] = {0, 0, -1};
        const double pts[5][3] = {{L[0], L[1] - 0.835, L[2] + 10},     // ground 10 m ahead, in the cone
                                  {L[0] + 3, L[1] - 0.835, L[2] + 40},  // 40 m ahead, 4 deg off axis
                                  {L[0] + 12, L[1] - 0.835, L[2] + 10}, // 50 deg off axis: outside the cone
                                  {L[0], L[1] - 0.4, L[2] - 1.5},       // behind the light (f < 0): the cut-off term is 0
                                  {L[0], L[1] - 0.2, L[2] + 0.3}};      // 0.36 m in front: the BACKOMNIPOWER "up" term
        const double* nrm[5] = {Ny, Ny, Ny, Nb, Ny};
        double D1[3];
        const double dl = std::sqrt(D[0] * D[0] + D[1] * D[1] + D[2] * D[2]);
        for (int k = 0; k < 3; ++k) D1[k] = D[k] / dl;
        for (int i = 0; i < 5; ++i) {
            double w[3];
            ref(pts[i], nrm[i], L, D1, up, w);
            const omni::V3 g = omni::term({float(pts[i][0]), float(pts[i][1]), float(pts[i][2])}, {float(nrm[i][0]), float(nrm[i][1]), float(nrm[i][2])},
                                          {float(L[0]), float(L[1]), float(L[2])}, {float(D1[0]), float(D1[1]), float(D1[2])}, val, qp.colour, {0, 1, 0});
            const float want[3] = {float(w[0]), float(w[1]), float(w[2])};
            char name[96];
            std::snprintf(name, sizeof name, "CPU omni.phl point %d", i);
            check(name, g.data(), want, 3, 2e-3f);
        }
        id<MTLDevice> device = MTLCreateSystemDefaultDevice();
        if (device != nil) {
            ResourceLibrary lib;
            MeshRenderer r(device, 1, lib);
            r.forcedTimeOfDay = 21 * 3600.0;
            MeshRenderer::VehicleOmni o;
            o.pos = simd_make_float3(float(L[0]), float(L[1]), float(L[2]));
            o.dir = simd_make_float3(float(D1[0]), float(D1[1]), float(D1[2]));
            o.value = simd_make_float4(val[0], val[1], val[2], val[3]);
            o.colour = simd_make_float3(qp.colour[0], qp.colour[1], qp.colour[2]);
            MeshRenderer::VehicleOmni o2 = o;  // a second light 80 m away facing away: adds (nearly) nothing in front
            o2.pos += simd_make_float3(0, 0, 80), o2.dir = -o2.dir;
            r.setVehicleLights({o2, o});
            std::vector<simd_float4> gin;
            for (int i = 0; i < 5; ++i)
                gin.insert(gin.end(), {simd_make_float4(float(pts[i][0]), float(pts[i][1]), float(pts[i][2]), 10),
                                       simd_make_float4(float(nrm[i][0]), float(nrm[i][1]), float(nrm[i][2]), 0), 0, 0});
            std::vector<simd_float4> out = r.shaderCheck(at(hm(21, 0)), simd_make_float3(-39570, 866, -28075), gin);
            if (out.size() != gin.size() / 2) {
                std::printf("  FAIL GPU omni check did not run\n");
                ++gFail;
            } else {
                for (int i = 0; i < 5; ++i) {
                    double w[3], w2[3];
                    ref(pts[i], nrm[i], L, D1, up, w);
                    const double L2[3] = {L[0], L[1], L[2] + 80}, D2[3] = {-D1[0], -D1[1], -D1[2]};
                    ref(pts[i], nrm[i], L2, D2, up, w2);
                    const float got[4] = {out[size_t(i) * 2].x, out[size_t(i) * 2].y, out[size_t(i) * 2].z, out[size_t(i) * 2].w};
                    const float want[4] = {float(w[0] + w2[0]), float(w[1] + w2[1]), float(w[2] + w2[2]), 2};
                    char name[96];
                    std::snprintf(name, sizeof name, "GPU omni sum (2 lights) point %d, count", i);
                    check(name, got, want, 4, 2e-2f);  // float32 world positions at 40 km
                }
            }
            r.forcedTimeOfDay = 12 * 3600.0;  // daytime: switched off
            out = r.shaderCheck(at(hm(12, 0)), simd_make_float3(-39570, 866, -28075), gin);
            const float gday[2] = {out.empty() ? -1 : out[0].x, out.empty() ? -1 : out[0].w}, wday[2] = {0, 0};
            check("GPU omni at 12:00 (lights off by 0x5cbde0)", gday, wday, 2, 0);
        }
    }

    {  // P83: the race wind flag of AddMissionParam (0x575b00: start and end both >= 0) and the tank amounts 0x5bba30
        auto rw = [](const char* a, const char* b, float t) { return storm::raceWind(a, b, t).value_or(-1.0f); };
        const float g[6] = {rw("C#30_R#0_W#40", "C#30_R#0_W#40", 0.5f), rw("C#60_S#60_W#60", "C#40_S#10", 0.0f), rw("C#80_W#30", "C#80_W#60", 0.5f),
                            rw("SUN_W#40", "SUN_W#40", 0.0f), rw("SUN", "C#10", 0.0f), rw("C#0_TOR_W#100", "C#0_R#25_W#100", 0.3f)};
        const float w[6] = {0.4f, -1.0f, 0.45f, 0.4f, -1.0f, 1.0f};
        check("race wind: CR2 W#40, one-sided W# (none), lerp, SUN_W#40, SUN alone, CR21", g, w, 6, 1e-6f);
        const auto a40 = storm::windTankAmounts(0.4f * 27, 27), a100 = storm::windTankAmounts(27, 27), a20 = storm::windTankAmounts(0.2f * 27, 27);
        const float ga[6] = {a40[0], a40[1], a100[0], a100[1], a20[0], a20[1]};
        const float wa[6] = {1.0f - 0.1f * 1.4285715f, 0.1f * 1.4285715f, 0, 1, 0.2f * 3.3333333f, 0};
        check("0x5bba30 wind / debris amounts at w .4, 1, .2", ga, wa, 6, 1e-5f);
        {  // P88: tank vf05 0x5e7400 (asm 0x5e74ca..0x5e754d) for 1 s at 60 Hz and vh_debris (wind.vhl) for one particle.
            storm::WindTankLook l;
            l.count = 1, l.scale = 70, l.particle = 0.5f, l.frequency = 10, l.amplitude = 40, l.distRatio = 2, l.speedMin = 10, l.speedMax = 20;
            storm::WindTank t(l, 1);
            for (int i = 0; i < 60; ++i) t.update(1.0f / 60, {-2.7f, 0, 10.46f});
            const auto& d = t.displacement();
            const float gd[3] = {d[0], d[1], d[2]}, wd[3] = {-0.81f, 0, 3.138f};  // wind x 0.3 tank units per second (|wind| 10.8 in [10, 20])
            check("tank displacement: norm(wind) clamp(|wind|, lo, hi) 0.3 dt, tank units", gd, wd, 3, 1e-4f);
            const auto pl = t.place({0.5f, -0.25f, 0.75f, 0.2f, 0}, {0, 0, 0});
            const float gp[5] = {pl.pos[0], pl.pos[1], pl.pos[2], pl.angle, float(pl.cell[2])};
            const float wp[5] = {-50.914f, 11.714f, -56.281f, 41.0834f, 2};
            check("vh_debris MapTankCoordinate + 40 m wave at angle 10 |q + disp| + 10 w", gp, wp, 5, 2e-3f);
            // P90 [DEVIATION, wind_debris = smooth]: the same second in metres (/ TankScale), FUEL's wave held at its
            // starting phase + a 1.5 m swirl at 1.2 (0.5 + |w|) rad/s, alpha by the distance to the box faces (10 m).
            const auto& sd = t.smoothDisplacement();
            const float gs[4] = {sd[0], sd[1], sd[2], t.smoothClock()}, ws[4] = {-0.81f / 70, 0, 3.138f / 70, 1};
            check("P90 smooth drift: FUEL's tank displacement / TankScale (m/s = 0.3 clamp(|wind|)), clock", gs, ws, 4, 2e-6f);
            const auto ps = t.placeSmooth({0.5f, -0.25f, 0.75f, 0.2f, 0}, {0, 0, 0});
            const auto pf = t.placeSmooth({0.95f, 0.1f, -0.2f, 0.0f, 0}, {0, 0, 0});
            const float gq[7] = {ps.pos[0], ps.pos[1], ps.pos[2], ps.angle, float(ps.cell[2]), ps.alpha, pf.alpha};
            const float wq[7] = {87.6262f, -70.9362f, 33.0711f, 12.19414f, 0, 1, 0.431f};
            check("P90 smooth place: box wrap, 40 m wave at 10 |q| + 10 w, swirl, face fade", gq, wq, 7, 2e-3f);
            using storm::DebrisMotion;
            const float gm[8] = {float(storm::debrisMotionFrom("fuel")), float(storm::debrisMotionFrom(" Smooth ")),
                                 float(storm::debrisMotionFrom("junk", DebrisMotion::Fuel)), storm::debrisFuelWeight(DebrisMotion::Smooth, -1),
                                 storm::debrisFuelWeight(DebrisMotion::Smooth, 100), storm::debrisFuelWeight(DebrisMotion::Smooth, 150),
                                 storm::debrisFuelWeight(DebrisMotion::Smooth, 400), storm::debrisFuelWeight(DebrisMotion::Fuel, 400)};
            const float wm[8] = {1, 0, 1, 0, 1, 0.5f, 0, 1};
            check("P90 wind_debris parse (fuel / smooth / fallback), FUEL's weight: none, 100 m, 150 m, 400 m, fuel mode", gm, wm, 8, 1e-6f);
        }
    }

    {  // ---- P94: Lod_Z::Draw 0x884170 + Mesh_Z::Draw 0x80a280 fades; FUEL's decor kinds and fades (vegetation.hpp decorAtt).
        PlacedMesh p;
        p.lod = true, p.viewDist = 10, p.levels = {1}, p.levelView = {0};
        const LodView v;  // fade 39.72 .. 59.72 m (10 x 10 x 0.3972, + 20)
        const float a0 = placementAlpha(p, 0, 49.72f, v);
        p.levelView = {5};  // the mesh's own view distance: its fade wins (19.86 .. 39.86 m: gone at 49.72)
        const float a1 = placementAlpha(p, 0, 49.72f, v);
        const float ga[2] = {a0, a1}, wa[2] = {0.5f, 0.0f};
        check("P94 Lod alpha kept when the level mesh has no fade (0x80a280 returns), else the mesh's", ga, wa, 2, 1e-3f);
        // Eye at (1000, 1000), plants east of it: leaves 24 (20 m), 48 (140, 150 m), 96 (280 m), 384 (1250 m), 192 (400 m).
        const auto at = [](DecorFade k, float dx) { return decorAtt(k, 1000.0 + dx, 1000.0, 1000.0, 1000.0, dx); };
        const float gd[9] = {at(DecorFade::Mesh, 20), at(DecorFade::Mesh, 140), at(DecorFade::TreeCard, 140), at(DecorFade::BushCard, 140),
                             at(DecorFade::Mesh, 150), at(DecorFade::TreeCard, 150), at(DecorFade::BushCard, 280), at(DecorFade::TreeCard, 1250),
                             at(DecorFade::Mesh, 400)};
        const float wd[9] = {1, 0.75f, 0.75f, 0.75f, 0.416667f, 1, 0.4f, 0.5f, 0};
        check("P94 decor att: kinds 0/1 out 132.5..162.5, 0x16 in 117.5..147.5, 0x15 out 250..300, 0x14 out 1200..1300, by leaf", gd, wd, 9, 1e-4f);
    }

    {  // ---- P104: vehicle LOD (notes p104-vehicle-lod.md): Lod_Z::Draw 0x884170 levels with the LodData flags of 0x5c7fe0.
        const LodView v;  // K D s = 15.888 m for D 4
        const float gl[11] = {float(lodSwitchLevel(5, 4, 1, 1, 3, v)),     float(lodSwitchLevel(20, 4, 1, 1, 3, v)),
                              float(lodSwitchLevel(32, 4, 1, 1, 3, v)),    float(lodSwitchLevel(70, 4, 1, 1, 3, v)),
                              float(lodSwitchLevel(1000, 4, 1, 1, 3, v)),  float(lodSwitchLevel(100, 30, 1, 0, 2, v)),
                              float(lodSwitchLevel(150, 30, 1, 0, 2, v)),  float(lodSwitchLevel(300, 30, 1, 0, 2, v)),
                              float(lodSwitchLevel(500, 4, 1, 0, 0, v)),   float(lodSwitchLevel(7, 2, 1, 0, 2, v)),
                              float(lodSwitchLevel(20, 2, 1, 0, 2, v))};
        const float wl[11] = {1, 1, 2, 3, 3, 0, 1, 2, 0, 0, 2};
        check("P104 vehicle levels: AI quad D 4 (never LOD1), FLOTTE D 30, player (always LOD1), rider D 2", gl, wl, 11, 0);
        const float gf[2] = {lodFade(15, 0, 0, v.viewScale, 69.58f, v), lodFade(50, 0, 0, v.viewScale, 208.6f, v)};
        const float wf[2] = {0.5f, 0.5f};
        check("P104 Lod fade: rider (view 15) and BIKE (view 50) half gone at 69.58 / 208.6 m", gf, wf, 2, 1e-3f);
    }

    {  // ---- PK04: race time of day 0x631aa0, AddMissionParam 0x575b00 weather records, 0x631bd0 / 0x632250 / 0x5b89e0
       // overrides, SetDayClock 0x46ab10.
        const float gf[4] = {scenarioDayFraction(0) * 86400.0f, scenarioDayFraction(1) * 86400.0f, scenarioDayFraction(2) * 86400.0f,
                             scenarioDayFraction(3) * 86400.0f};
        const float wf[4] = {25228.8f, 39571.2f, 57602.88f, 77760};  // 0.292, 0.458, 0.6667, 0.9: 07:00:29, 10:59:31, 16:00:03, 21:36
        check("PK04 0x631aa0 scenario day fractions x 86400 (~07:00, 11:00, 16:00, 21:36)", gf, wf, 4, 0.05f);
        const MissionWeather cr2 = parseAddMissionParam({"CR2-Yell-CP", "18_61_33_61_33_61_18", "7.3", "7.3", "C#30_R#0_W#40", "0", "C#30_R#0_W#40", "0",
                                                         "NONE", "NONE"});
        const float g2[14] = {float(cr2.hours), cr2.day0, cr2.day1, float(cr2.cloud), cr2.cloud0, float(cr2.rain), cr2.rain0, float(cr2.wind),
                              cr2.wind1, float(cr2.fog), cr2.fog0, float(cr2.snow), float(cr2.sand), float(cr2.tornado)};
        const float w2[14] = {1, 7.3f / 24, 7.3f / 24, 1, 0.3f, 1, 0, 1, 0.4f, 1, 0, 0, 0, 0};
        check("PK04 AddMissionParam CR2: hours 7.3 / 24, C#30 R#0 W#40 both, fog numbers 0 / 0 (flag set), no snow / sand / TOR", g2, w2, 14,
              1e-6f);
        const MissionWeather cr4 = parseAddMissionParam({"CR4-Isla-RD", "x", "18.4", "19.3333333333333", "C#0", "-1", "C#0", "-1", "NONE", "NONE"});
        const MissionWeather one = parseAddMissionParam({"m", "NONE", "15", "NONE", "C#60_S#60_W#60", "0.25", "C#40_S#10_G#20", "0.75", "NONE", "NONE"});
        const MissionWeather tor = parseAddMissionParam({"m", "NONE", "NONE", "9", "C#0_TOR_W#100_G#50", "NONE", "c#0_r#25_w#100", "NONE", "NONE", "NONE"});
        const MissionWeather sun = parseAddMissionParam({"m", "NONE", "12", "12", "SUN", "0", "SUN", "0", "NONE", "NONE"});
        const float g3[16] = {float(cr4.fog), cr4.day1 * 24, float(one.hours), one.day1, float(one.cloud), one.cloud1, float(one.snow), one.snow1,
                              float(one.wind), float(one.sand), one.fog1, float(tor.hours), float(tor.tornado), float(tor.rain), float(tor.sand),
                              float(sun.cloud) + float(sun.rain) + float(sun.snow) + float(sun.wind) - sun.cloud0};
        const float w3[16] = {0, 19.3333333f, 1, 15.0f / 24 + 0.05f, 1, 0.4f, 1, 0.1f, 0, 0, 0.75f, 0, 1, 0, 0, 4};
        check("PK04 AddMissionParam: fog -1 (no flag), end hour, NONE end = +0.05, one-sided W# / G#, NONE hours, TOR (start only), "
              "one-sided R#, SUN",
              g3, w3, 16, 1e-5f);
        // 0x631bd0 + 0x631aa0 then 0x632250: day 7.3 -> 8.0 h (furthest t only), cloud .6 -> .4, fog .25 -> .75, snow fixed.
        MissionWeather m = one;
        m.day0 = 7.3f / 24, m.day1 = 8.0f / 24;
        WeatherOverrides o;
        o.windValue = 9;  // vf14 clears it (port: the whole block)
        RaceWeather rw;
        WeatherAmounts now;
        now.cloud = 0.8f;
        const bool repop = rw.begin(m, 0, now, o);
        const float g4[8] = {float(repop), float(o.time), o.dayFraction, float(o.cloud), o.cloudValue, float(o.snow), o.snowValue, float(o.fog) + o.fogValue};
        const float w4[8] = {1, 1, 7.3f / 24, 1, 0.6f, 1, 0.6f, 1.25f};
        check("PK04 0x631bd0 / 0x631aa0 setup: 0x5b7bb0 repopulate, time, start fraction, cloud .6, snow .6, fog .25", g4, w4, 8, 1e-6f);
        rw.step(0.5f, o);
        const float d05 = o.dayFraction;
        rw.step(0.25f, o);
        const float g5[6] = {d05, o.dayFraction, o.cloudValue, o.fogValue, o.snowValue, rw.dayProgress()};
        const float w5[6] = {0.5f * (7.3f + 8.0f) / 24, 0.5f * (7.3f + 8.0f) / 24, 0.55f, 0.375f, 0.6f, 0.5f};
        check("PK04 0x632250: time at the furthest t (0.5 then 0.25), cloud / fog by t, snow not lerped", g5, w5, 6, 1e-6f);
        WeatherOverrides o2;
        const bool repop2 = rw.begin(MissionWeather{}, 0x2 | (2 << 2) | (3 << 4) | 0x800 | 0x200 | 0x80 | 0x10000 | (1 << 12) | (2 << 14) | 1, now, o2);
        const float g6[11] = {float(repop2), o2.dayFraction * 24, rw.dayEnd() * 24, o2.cloudValue, float(o2.rain), o2.rainValue, float(o2.wind),
                              float(o2.snow), o2.fogValue, float(o2.storm), float(o2.sand)};
        const float w6[11] = {1, 16.0008f, 21.6f, 1, 1, 1, 1, 1, 0.33f, 1, 0};
        check("PK04 scenario flags: time bits 2-3 / 4-5, 0x800 with 0x200 / 0x80, fog bits 12 / 14, storm bit 0", g6, w6, 11, 1e-4f);
        const float g7[4] = {raceWeatherT({0, 0, 0}, {100, 5, 0}, {50, 0, 0}), raceWeatherT({0, 0, 0}, {100, 0, 0}, {-300, 0, 0}),
                             raceWeatherT({1, 0, 1}, {1, 9, 1}, {50, 0, 0}), raceWeatherT({0, 0, 0}, {0, 0, 200}, {0, 50, 100})};
        const float w7[4] = {0.25f, 1, 0, 0.25f};
        check("PK04 0x632250 t = |p - start|^2 / |finish - start|^2 (x z, at most 1, 0 when start = finish)", g7, w7, 4, 1e-6f);
        // 0x5b89e0: rain .5 -> fog max(field .2, PP5444 x snow, PP5445 x rain) = .4; fog override; storm 0.1, Cloudness kept.
        WeatherAmounts a;
        a.cloud = 0.3f, a.cloudness = 0.6f, a.fogField = 0.2f, a.fog = 0.2f;
        WeatherOverrides ov;
        ov.rain = true, ov.rainValue = 0.5f;
        WeatherAmounts a1 = a;
        applyOverrides(a1, ov, 0.5f, 0.8f);
        ov.fog = true, ov.fogValue = 1.5f, ov.storm = true, ov.sand = true, ov.sandValue = 0.3f;
        WeatherAmounts a2 = a;
        applyOverrides(a2, ov, 0.5f, 0.8f);
        ov.cloud = true, ov.cloudValue = 0.7f, ov.storm = false;
        WeatherAmounts a3 = a;
        applyOverrides(a3, ov, 0.5f, 0.8f);
        const float g8[9] = {a1.rain, a1.fog, a2.fog, a2.cloud, a2.cloudness, a2.sand, a3.cloud, a3.cloudness, a1.cloud};
        const float w8[9] = {0.5f, 0.4f, 1, 0.1f, 0.6f, 0.3f, 0.7f, 0.7f, 0.3f};
        check("PK04 0x5b89e0 overrides: rain -> fog rule, fog (clamped), storm cloud 0.1 (Cloudness kept), sand, cloud + 0x5b7e40", g8, w8, 9, 1e-6f);
        // SetDayClock 0x46ab10 / 0x6ddfa0 and the 0x5b89e0 clock read: 43200 -> 08:00, 450 (pause menu) -> 20:00.
        const float g9[6] = {float(setDayClockMs(43200.0)), float(setDayClockMs(450.0)), float(setDayClockMs(0.0004)),
                             float(int64_t(setDayClockMs(-1.0)) - int64_t(0x10000000000ll)),
                             float(std::fmod(dayClockTimeOfDay(setDayClockMs(43200.0), 96.0f), 86400.0)),
                             float(dayClockTimeOfDay(setDayClockMs(450.0), 96.0f))};
        const float w9[6] = {43200000, 450000, 0, -1000, 28800, 72000};
        check("PK04 SetDayClock ms (43200, 450, 0.0004, -1 wraps 40 bits) and time of day 08:00 / 20:00", g9, w9, 6, 0);
        id<MTLDevice> device = MTLCreateSystemDefaultDevice();
        if (device != nil) {
            ResourceLibrary lib;
            MeshRenderer r(device, 1, lib);
            r.gameClock = 1000;  // 1000 s of game: 08:00 + 26:40:00
            const double t0 = r.timeOfDay();
            r.setDayClock(43200);
            const double t1 = r.timeOfDay();
            r.gameClock = 1010;
            const double t2 = r.timeOfDay();
            r.beginRaceWeather(cr2, {0, 0, 0}, {100, 0, 0}, [] { return std::array<double, 3>{0, 0, 0}; });
            const double t3 = r.timeOfDay();
            r.endRaceWeather();
            const double t4 = r.timeOfDay();
            r.forcedTimeOfDay = 3600;
            const double t5 = r.timeOfDay();
            const float gr[6] = {float(std::fmod(t0, 86400.0)), float(std::fmod(t1, 86400.0)), float(std::fmod(t2, 86400.0)), float(t3),
                                 float(std::fmod(t4, 86400.0)), float(t5)};
            const float wr[6] = {float(std::fmod(1000.0 * 96 + 28800, 86400.0)), 28800, 28800 + 960, float(double(float(7.3f / 24) * 86400.0f)),
                                 28800 + 960, 3600};
            check("PK04 renderer: clock, setDayClock(43200) = 08:00 and runs on, CR2 race 07:18, race end, ForceDayTime wins", gr, wr, 6, 0.01f);
        }
    }

    {  // ---- PK24: lens projections (0x5fd390 / 0x5fd270 / 0x5ff8b0, 0x600a40 / 0x600d10 / 0x5fff30, 0x600f70 / 0x600160)
        std::vector<float> pp(6000, 0.0f);
        const float rows[14] = {0, 0.05f, 15, 0.1f, 0.5f, 3, 0.01f, 0.03f, 0.3f, 0.6f, 0.7f, 1, 0.3f, 0.5f};  // PP5510..5523
        for (int i = 0; i < 14; ++i) pp[size_t(5510 + i)] = rows[i];
        const wfx::LensParams lp = wfx::LensParams::fromPP(pp);
        auto seq = [](std::vector<int32_t> v) {  // a scripted WELL512: the values, then 0x400000 (0.5)
            auto i = std::make_shared<size_t>(0);
            return wfx::RndInt([v, i]() { return *i < v.size() ? v[(*i)++] : int32_t(0x400000); });
        };
        const wfx::ScreenFx::AtlasRect rect = [](int a) { return a >= 38 ? std::array<float, 4>{0.5f, 0.0f, 0.5f, 0.5f} : std::array<float, 4>{0, 0, 1, 1}; };
        {  // rain drop: size U(PP5516, 5517), +-sqrt(U) position, angle U 2 pi - pi, atlas 38 + (|r| & 3), uv from the atlas rect, alpha, life 4 s
            wfx::ScreenFx fx(lp);
            fx.update(1e-6f, 0.85f, wfx::LensVehicle{}, seq({0x200000, 0x400000, 0x400000, 0x400000, 0x400000, 0x400000, 0x400000, 7}), rect);
            const auto& e = fx.entries()[0];
            const float g[11] = {float(e.kind), e.size[0], e.pos[0], e.pos[1], e.angle, float(e.atlas), e.uv[1], e.uv[2], e.uv[5], e.life,
                                 float(fx.entries().size())};
            const float w[11] = {1, 0.02f, 0.70710678f, 0.70710678f, 0.0f, 38 + (7 & 3), 0.5f, 1.0f, 0.0f, 4, 1};
            check("PK24 0x600a40 rain drop: size, +-sqrt(U) position, angle, atlas 38 + (r & 3), atlas-rect uv, life 4 s", g, w, 11, 1e-5f);
            wfx::ScreenFx slow(lp), fast(lp);  // 0x600f70: slides down at rest (v.y -0.05), streams outward at speed (k = 1: 0.05 n)
            const auto drop = seq({0x200000, 0x400000, 0x400000, 0x400000, 0x400000, 0x400000, 0x400000, 7});
            slow.update(1e-6f, 0.85f, wfx::LensVehicle{}, drop, rect);
            slow.update(0.1f, 0.0f, wfx::LensVehicle{}, drop, rect);
            wfx::LensVehicle mv;
            mv.present = true, mv.velocity = {4, 0, 0};
            fast.update(1e-6f, 0.85f, mv, seq({0x200000, 0x400000, 0x400000, 0x400000, 0x400000, 0x400000, 0x400000, 7}), rect);
            fast.update(0.1f, 0.0f, mv, drop, rect);
            const float g2[5] = {slow.entries()[0].pos[0], slow.entries()[0].pos[1], fast.entries()[0].pos[0], fast.entries()[0].pos[1], slow.entries()[0].rgba[3]};
            const float w2[5] = {0.70710678f, 0.70710678f - 0.005f, 0.70710678f + 0.0035355f, 0.70710678f + 0.0035355f, 1 - 0.1f / 4};
            check("PK24 0x600f70 drop motion: slides down at rest, streams outward at speed, alpha 1 - age / 4", g2, w2, 5, 2e-4f);
            slow.update(5.0f, 0.0f, wfx::LensVehicle{}, drop, rect);
            check1("PK24 0x6005c0 cull: a drop older than its 4 s life is removed", float(slow.entries().size()), 0, 0);
        }
        {  // wheel splash 0x600d10: speed 5 >= PP5515, only wheel 2 wet (flag B): x >= 0 without the first pair, y = (0.7 |x| + 0.3) U - 1
            wfx::ScreenFx fx(lp);
            wfx::LensVehicle v;
            v.present = true, v.velocity = {5, 0, 0}, v.wheels = 4, v.waterDepth[2] = 0.05f;
            fx.update(1e-6f, 0.0f, v, seq({0x200000, 0x200000, 0x100000, 0x600000, 7}), rect);
            const auto& e = fx.entries()[0];
            const float g[6] = {float(fx.entries().size()), e.size[0], e.atlas * 1.0f, e.life, e.spawn, 0};
            const float w[6] = {1, 0.375f, 0x23 + 7 % 3, 4, 1e-6f - 4.0f / 256, 0};
            check("PK24 0x600d10 wheel splash: size U(PP5518, 5519), atlas 35 + (r % 3), life 4 s back-dated by life / 256", g, w, 5, 1e-5f);
            // the first update already moved it by one 1e-6 step: x 0.75 (forced >= 0), y -0.38125
            const float gp[2] = {e.pos[0], e.pos[1]}, wp[2] = {0.75f, -0.38125f};
            check("PK24 0x600d10 splash position: x forced >= 0 without the first wheel pair, y bottom band", gp, wp, 2, 1e-4f);
            wfx::ScreenFx dry(lp);
            wfx::LensVehicle d = v;
            d.waterDepth[2] = 0.005f;  // <= 0.01: no splash
            dry.update(1e-6f, 0.0f, d, seq({}), rect);
            wfx::ScreenFx slowCar(lp);
            d.waterDepth[2] = 0.05f, d.velocity = {2, 0, 0};  // below PP5515
            slowCar.update(1e-6f, 0.0f, d, seq({}), rect);
            check1("PK24 0x5fd390 splash gates: wheel depth > 0.01 and speed >= PP5515", float(dry.entries().size() + slowCar.entries().size()), 0, 0);
        }
        {  // mud 0x5fd270 / 0x5fff30: speed >= 3 and a wheel's dust running; size U(PP5522, 5523), x = 2U - 1, atlas 32 + (r % 3), static, life 4 s
            wfx::ScreenFx fx(lp);
            wfx::LensVehicle v;
            v.present = true, v.velocity = {5, 0, 0}, v.wheels = 4, v.dust = true;
            fx.update(1e-6f, 0.0f, v, seq({0x200000, 0x200000, 0x600000, 0x400000, 0x400000, 5}), rect);
            const auto& e = fx.entries()[0];
            const float g[7] = {float(e.kind), e.size[0], e.pos[0], e.pos[1], float(e.atlas), e.life, e.rgba[3]};
            const float w[7] = {0, 0.35f, 0.5f, -0.675f, 34, 4, 1};
            check("PK24 0x5fff30 mud splat: size, x = 2U - 1, bottom band y, atlas 32 + (r % 3), life 4 s", g, w, 7, 1e-5f);
            wfx::ScreenFx noDust(lp), slowCar(lp);
            v.dust = false;
            noDust.update(1e-6f, 0.0f, v, seq({}), rect);
            v.dust = true, v.velocity = {2.9f, 0, 0};
            slowCar.update(1e-6f, 0.0f, v, seq({}), rect);
            check1("PK24 0x5fd270 mud gates: dust running and speed >= 3 m/s", float(noDust.entries().size() + slowCar.entries().size()), 0, 0);
        }
        {  // dirt strip 0x5ff8b0 / 0x5ffa70: four strips above 0.1, alpha 2 d (<= 0.5) or 2 (d - 0.5) + 1, rgb 3 x 0.5, removed at <= 0.1
            wfx::ScreenFx fx(lp);
            wfx::LensVehicle v;
            v.present = true, v.dirtValid = true, v.dirt = 0.3f;
            fx.update(0.01f, 0.0f, v, seq({}), rect);
            const auto& e = fx.entries();
            const float g[8] = {float(e.size()), e[0].rgba[3], e[0].rgba[0], e[0].pos[1], e[1].pos[1], e[2].pos[0], e[3].pos[0], e[0].size[0]};
            const float w[8] = {4, 0.6f, 1.5f, 0.95f, -0.75f, -0.9f, 0.9f, 16.0f / 9};
            check("PK24 0x5ffa70 dirt strips: top / bottom / left / right, alpha 2 d, rgb 1.5", g, w, 8, 1e-6f);
            v.dirt = 0.8f;
            fx.update(0.01f, 0.0f, v, seq({}), rect);
            const float a2 = fx.entries()[0].rgba[3];
            v.dirt = 0.1f;
            fx.update(0.01f, 0.0f, v, seq({}), rect);
            const float g2[3] = {a2, float(fx.entries().size()), 0};
            const float w2[3] = {1.6f, 0, 0};
            check("PK24 0x5ff8b0 dirt alpha 2 (d - 0.5) + 1 above 0.5, strips removed at d <= 0.1", g2, w2, 2, 1e-6f);
            const float gp2[2] = {wfx::LensParams::fromPP(pp).mudSizeMax, wfx::LensParams::fromPP(pp).splashSpeed};
            const float wp2[2] = {0.5f, 3};
            check("PK24 PP5510..5523 -> LensParams", gp2, wp2, 2, 0);
        }
    }

    {  // ---- PK23: vehicle FX (0x5cf2d0, 0x5edb20, 0x5d07f0, 0x4c5e60, 0x5defc0, 0x5f0b70, 0x5eeb20; notes pk23.md)
        namespace fs = std::filesystem;
        std::printf("PK23 vehicle effects (exhaust smoke / flames / embers, scrape sparks, track head snap)\n");
        const float lv[4] = {vfx::flameLevel(0, 0.95f, 0.01f), vfx::flameLevel(0.7f, 0.92f, 1.0f), vfx::flameLevel(0, 0.89f, 1.0f),
                             vfx::flameLevel(0, 1.0f, -0.2f)};
        const float lw[4] = {0.5f, 0.7f, 0, 0};
        check("PK23 0x5cf2d0 flame level max(flash, clamp((frac-.9)20) clamp(50 load))", lv, lw, 4, 1e-6f);
        const float nt[3] = {float(vfx::nearTopSpeed(23.5f, 100)), float(vfx::nearTopSpeed(22.0f, 100)), float(vfx::nearTopSpeed(0, 100))};
        const float ntw[3] = {1, 0, 0};
        check("PK23 0x5cf2d0 near top speed |v| / (P619 / 1.15 / 3.6) > 0.95", nt, ntw, 3, 0);
        const float sa[3] = {vfx::smokeAlpha(7.5f, 0.6f), vfx::smokeAlpha(15, 0.6f), vfx::smokeAlpha(16, 0.6f)};
        const float saw[3] = {0.3f, 0, 0};
        check("PK23 0x5cf2d0 smoke alpha (1 - v / 15) PP5329, 0 above 15 m/s", sa, saw, 3, 1e-6f);
        const float al[3] = {vfx::sparkAlpha(0.05f, 0.1f), vfx::sparkAlpha(0.2f, 0.1f), vfx::sparkAlpha(0, 0)};
        const float alw[3] = {0.840896415f, 0, 0};
        check("PK23 0x4c5e60 spark alpha (1 - t)^0.25", al, alw, 3, 1e-6f);
        {  // 0x5edb20: dir (0, 0, -1), size 0.4, length 0.5 at t = 0
            vfx::FlameEmitter fe;
            fe.start();
            fe.dir = {0, 0, -1}, fe.size = 0.4f, fe.length = 0.5f, fe.pos = {0, 0, 0};
            fe.update(0, {0.75f, 0.75f, 0.25f, 0.125f});
            const float g[12] = {fe.quads[0].v[0][0], fe.quads[0].v[0][1], fe.quads[0].v[0][2], fe.quads[1].v[2][0], fe.quads[1].v[2][1], fe.quads[1].v[2][2],
                                 fe.quads[2].v[3][0], fe.quads[2].v[3][1], fe.quads[2].v[3][2], fe.colour[0], fe.colour[1], fe.colour[2]};
            const float w[12] = {-0.282842712f, 0.282842712f, -0.05f, -0.282842712f, 0.282842712f, -0.45f, 0.282842712f, 0.282842712f, -0.45f, 2, 1.6f, 0.4f};
            check("PK23 0x5edb20 flame cap / planes (pos +- 0.05 d, +-|b-c|, |b+c| size, L d), orange", g, w, 12, 1e-6f);
            const float u[6] = {fe.quads[0].uv[0][0], fe.quads[0].uv[0][1], fe.quads[0].uv[3][0], fe.quads[1].uv[0][0], fe.quads[1].uv[0][1], fe.quads[2].uv[3][0]};
            const float uw[6] = {0.875f, 0.75f, 1.0f, 0.75f, 0.875f, 0.875f};
            check("PK23 0x5edb20 flame uvs: cap = right half of the atlas rect, planes = left half", u, uw, 6, 1e-6f);
            fe.blue = true, fe.t = 0.19634954f;  // 8 t = pi / 2: L = 1.1 x 0.5
            fe.update(0, {0.75f, 0.75f, 0.25f, 0.125f});
            const float b[4] = {fe.colour[0], fe.colour[2], fe.quads[1].v[2][2], float(fe.stopped)};
            const float bw[4] = {1.2f, 3, 0.05f - 0.55f, 0};
            check("PK23 0x5edb20 blue near top speed, length (1 + 0.1 sin 8t)", b, bw, 4, 1e-5f);
        }
        {  // 0x5d07f0 velocity range, 0x5defc0 rope
            vfx::V3 lo, hi;
            vfx::sparkVelocity({0, 0, 0, 1}, {0, 0, -1}, lo, hi);
            vfx::V3 lo2, hi2;
            vfx::sparkVelocity({0, 0.70710678f, 0, 0.70710678f}, {0, 0, 1}, lo2, hi2);  // 90 deg about y: (0,0,-10) -> (-10,0,0)
            const float g[9] = {lo[2], hi[1], hi[2], lo2[0], lo2[2], hi2[1], 0, 0, 0};
            const float w[9] = {10, 10, 0, -10, 0, 10, 0, 0, 0};
            check("PK23 0x5d07f0 velocity lo q(0,0,-10) flipped against the slide, hi q(0,10,0)", g, w, 9, 1e-5f);
            std::array<vfx::V3, 4> rope{};
            vfx::ropeStep(rope, {1, 0, 0}, 0.1f, 0.000841f);
            const float r[4] = {rope[0][0], rope[1][0], rope[2][0], rope[3][0]};
            const float rw[4] = {1, 0.9f, 0.8f, 0.7f};
            check("PK23 0x5defc0 rope: head, each point pulled to +0x80 of the one before", r, rw, 4, 1e-6f);
            std::array<vfx::V3, 4> r2{};
            r2.fill({0, 0, 0});
            vfx::ropeStep(r2, {0.02f, 0, 0}, 0.1f, 0.000841f);  // 0.02 < 0.029: the points stay
            check1("PK23 0x5defc0 rope: a point within sqrt(+0x7c) stays", r2[1][0], 0, 0);
        }
        {  // 0x5f0b70 / 0x5eeb20
            sfx::TrackEmitter t;
            t.start();
            sfx::Ribbon rb;
            rb.motion = 1;
            rb.pts.resize(2);
            rb.pts[1].pos = {1, 0, 0};
            t.ribbons.push_back(rb);
            sfx::trackHeadSnap(t, {1.5f, 0, 0}, {1.4f, 0.1f, 0});  // 0.5 m: snaps
            const float a = t.ribbons.back().pts[1].pos[0];
            t.ribbons.back().pts[1].pos = {1, 0, 0};
            sfx::trackHeadSnap(t, {3.5f, 0, 0}, {3.4f, 0, 0});  // 2.5 m: stays
            const float b = t.ribbons.back().pts[1].pos[0];
            t.ribbons.back().motion = 2;
            sfx::trackHeadSnap(t, {1.5f, 0, 0}, {1.4f, 0, 0});  // frozen: stays
            const float c = t.ribbons.back().pts[1].pos[0];
            t.kill();
            const float g[5] = {a, b, c, float(t.ribbons.size()), float(t.stopped)};
            const float w[5] = {1.4f, 1, 1, 0, 1};
            check("PK23 0x5f0b70 head snap within 2 m (running, not frozen); 0x5eeb20 kill frees (flags & 4)", g, w, 5, 1e-6f);
        }
        const sfx::Tables t = sfx::loadTables((fs::path(dataDir) / "XFinalComponent" / "FUEL.exe").string(),
                                              (fs::path(dataDir) / "CommonComponent" / "Param" / "pp001.tsc").string());
        if (t.ok) {
            const vfx::ExhaustParams ep = vfx::ExhaustParams::fromTables(t);
            const float g[12] = {ep.lifeLo, ep.lifeHi, ep.sizeLo, ep.sizeHi, ep.rgbA, ep.rgbB, ep.intervalLo, ep.intervalHi, ep.alpha, ep.rot, ep.push, ep.drag};
            const float w[12] = {0.5f, 1, 0.15f, 0.3f, 0, 0.1f, 0.05f, 0.1f, 0.6f, 1, 1, 1};
            check("PK23 PP5320..5333 -> exhaust smoke tuning", g, w, 12, 1e-6f);
            const bool names = t.atlas.size() > 0x30 && t.atlas[0].material == "FX_SPARKLE" && t.atlas[0x30].material == "FX_ATLAS_0";
            ++(names ? gPass : gFail);
            std::printf("  %-4s %-58s got %s %s\n", names ? "ok" : "FAIL", "PK23 FX atlas 0 (sparks) and 0x30 (flames)", t.atlas.empty() ? "-" : t.atlas[0].material.c_str(),
                        t.atlas.size() > 0x30 ? t.atlas[0x30].material.c_str() : "-");
            // The driver: one vehicle, one slot, the player at gear 1 then 2 (throttle 0.5 / 0.3), sparks, smoke, flames.
            sfx::System sys(t);
            vfx::VehicleFx fx(sys);
            std::vector<vfx::VehicleIn> in(1);
            in[0].slotPos = {{0, 1, 0}};
            in[0].slotDir = {{0, 0, -1}};
            in[0].visible = true, in[0].topSpeed = 100, in[0].throttle = 0.5f, in[0].cameraDistance = 10;
            vfx::PlayerIn pl;
            pl.valid = true, pl.gear = 1;
            fx.update(1.0f / 60, in, pl, {0, 2, -10});
            const int e0 = fx.stats().embersEmitted;  // the first frame: gear 0 -> 1 with throttle 0.5
            pl.gear = 2;
            fx.update(1.0f / 60, in, pl, {0, 2, -10});
            const int e1 = fx.stats().embersEmitted;
            in[0].throttle = 0.3f, pl.gear = 3;
            fx.update(1.0f / 60, in, pl, {0, 2, -10});
            const int e2 = fx.stats().embersEmitted;
            for (int k = 0; k < 30; ++k) fx.update(1.0f / 60, in, pl, {0, 2, -10});
            const int smokeSlow = fx.stats().smokeLive;
            const int flamesOff = fx.stats().flamesOn;
            in[0].flash = 1;
            fx.update(1.0f / 60, in, pl, {0, 2, -10});
            const int flamesOn = fx.stats().flamesOn;
            in[0].flash = 0.05f;  // level^2 0.0025 < 0.01
            fx.update(1.0f / 60, in, pl, {0, 2, -10});
            const int flamesLow = fx.stats().flamesOn;
            in[0].flash = 0;
            vfx::SparkHit h;
            h.a = 1, h.point = {0, 0, 0}, h.tangent = {0, 0, 1};
            in[0].hits = {h};
            fx.update(1.0f / 60, in, pl, {0, 2, -10});
            const int rec1 = fx.stats().sparkRecords, live1 = fx.stats().sparkLive;
            const bool flag = fx.fxFlag(0);
            in[0].hits.clear();
            fx.update(1.0f / 60, in, pl, {0, 2, -10});
            const int rec2 = fx.stats().sparkRecords, live2 = fx.stats().sparkLive;
            in[0].cameraDistance = 150, in[0].hits = {h};
            fx.update(1.0f / 60, in, pl, {0, 2, -10});
            const int rec3 = fx.stats().sparkRecords;
            in[0].hits.clear(), in[0].cameraDistance = 10, in[0].velocity = {20, 0, 0};
            for (int k = 0; k < 120; ++k) fx.update(1.0f / 60, in, pl, {0, 2, -10});
            const int smokeFast = fx.stats().smokeLive;
            const float g2[11] = {float(e0), float(e1), float(e2), float(smokeSlow > 0), float(flamesOff), float(flamesOn), float(flamesLow),
                                  float(rec1), float(live1 >= 20), float(flag), float(rec2 == 0 && live2 >= 20) + float(rec3)};
            const float w2[11] = {1, 2, 2, 1, 0, 1, 0, 1, 1, 1, 1};
            check("PK23 0x5cf2d0 embers on a gear change (throttle > .4), smoke below 15 m/s, flames at level^2 >= .01; 0x5d07f0 records", g2, w2, 11, 0);
            check1("PK23 0x5cf2d0 smoke stops above 15 m/s (the puffs die out)", float(smokeFast), 0, 0);
        } else {
            std::printf("  skip PK23 tables: %s\n", t.error.c_str());
        }
        try {  // 0x5cf170 on the shipped data: SFX_EXHAUSTn boxes of the parts (LOD1)
            float n[4] = {0, 0, 0, 0}, dz = 0;
            const char* sel[4][2] = {{"BUGGY", "BUGGY_ROAD_1"}, {"QUAD", "QUAD_ROAD_2"}, {"MUSCLE", "MUSCLE_ALL_3"}, {"SUV", "SUV_OFF_2"}};
            for (int k = 0; k < 4; ++k) {
                DpcFile dpc((fs::path(dataDir) / "CommonComponent" / "DATAS" / (std::string(sel[k][0]) + ".DPC")).string());
                VehicleAssembler as(dpc);
                const VehicleVariant* v = as.find(sel[k][1]);
                if (v == nullptr) continue;
                const AssembledVehicle av = as.assemble(*v, 1);
                n[k] = float(av.exhausts.size());
                if (k == 3 && !av.exhausts.empty()) dz = av.exhausts[0].dir[2];
            }
            const float wn[4] = {2, 0, 8, 2};
            check("PK23 0x5cf170 SFX_EXHAUST boxes: BUGGY_ROAD_1, QUAD_ROAD_2, MUSCLE_ALL_3, SUV_OFF_2", n, wn, 4, 0);
            check1("PK23 0x5cf170 SUV_OFF_2 exhaust 1 points back (-z of the box)", dz, -1, 1e-3f);
        } catch (const std::exception& ex) {
            std::printf("  skip PK23 exhaust boxes: %s\n", ex.what());
        }
    }  // ---- PK23 end
    {  // ---- PK12: STARTFX records and the sand-storm walls (0x5eca40, 0x5ec9a0, 0x5ea870, 0x5ecb20, 0x5c08b0; notes pk12.md)
        std::printf("PK12 STARTFX / sand storm (expected values: FUEL.exe under EmuRun, notes/pk12-work/emu)\n");
        namespace fs = std::filesystem;
#include "rendercheck_sand.inc"
        using sand::V3;
        const V3 seg[4][3] = {{{5, 0, 3}, {0, 0, 0}, {10, 0, 0}}, {{-4, 0, 3}, {0, 0, 0}, {10, 0, 0}}, {{14, 0, -3}, {0, 0, 0}, {10, 0, 0}},
                              {{3, 0, 7}, {2, 0, 2}, {2, 0, 2}}};
        float gs[4];
        for (int k = 0; k < 4; ++k) gs[k] = sand::Tank::segmentDistance(seg[k][0], seg[k][1], seg[k][2]);
        check("PK12 0x5eca40 xz distance to a segment (inside, before, after, degenerate)", gs, kSandSeg, 4, 1e-6f);
        {
            std::vector<V3> ring;
            for (int j = 0; j < 6; ++j) ring.push_back({float(std::cos(2 * M_PI * j / 5) * 50), 0.0f, float(std::sin(2 * M_PI * j / 5) * 50)});
            sand::Obb m{};
            m[3] = 100, m[11] = 200;
            const float gi[2] = {float(sand::Tank::insidePolygon({100, 0, 200}, ring, 6, m)), float(sand::Tank::insidePolygon({180, 0, 200}, ring, 6, m))};
            check("PK12 0x5ec9a0 camera inside the wall's ring (centre / outside)", gi, kSandInside, 2, 0);
        }
        std::vector<float> pp(5400, 0.0f);
        const std::pair<int, float> rows[] = {{5380, 3}, {5381, 6}, {5382, 0.15f}, {5383, 0.4f}, {5387, 0}, {5388, 1.5f}, {5389, 180}, {5390, 20},
                                              {5392, 250}, {5393, 20}, {5397, 100}, {5398, 10}, {5399, 1}};
        for (const auto& [n, v] : rows) pp[size_t(n)] = v;
        const sand::Params sp = sand::Params::fromPP(pp);
        const float ys[6] = {10, 20, 30, 40, 50, 60};
        auto seed = [&](sand::Mesh& m) {
            for (int j = 0; j < 6; ++j) {
                m.points[size_t(j)] = {0, ys[j], 0};
                m.colours[size_t(j)] = {0.2f + 0.05f * j, 0.3f + 0.02f * j, 0.4f - 0.03f * j, 1.0f};
            }
        };
        auto flat = [](const std::vector<sand::Vertex>& v) {
            std::vector<float> f(v.size() * 19);
            std::memcpy(f.data(), v.data(), f.size() * 4);
            return f;
        };
        auto flatP = [](const std::vector<V3>& p) {
            std::vector<float> f;
            for (const V3& x : p) f.insert(f.end(), x.begin(), x.end());
            return f;
        };
        {  // 0x5ea870(0, 6), then (2, 1) after point 1's z moved: columns 2.. rebuilt, point 1 kept
            sand::Mesh m(sp);
            seed(m);
            m.matrix[12] = 100, m.matrix[13] = 10, m.matrix[14] = 120, m.matrix[15] = 1;
            m.fill(0, 6);
            check("PK12 0x5ea870 wall vertices (3 rings x 6 columns: pos, normal + u, tangent + v, colour, binormal)", flat(m.cpu()).data(), kSandFill1V, 342, 2e-5f);
            check("PK12 0x5ea870 ring-0 points (x, z of ring 0; y kept)", flatP(m.points).data(), kSandFill1P, 18, 2e-5f);
            m.points[1][2] = 95;
            m.fill(2, 1);
            check("PK12 0x5ea870(2, 1): columns 2..5 rebuilt, column 1 kept", flat(m.cpu()).data(), kSandFill2V, 342, 2e-5f);
            check("PK12 0x5ea870(2, 1): ring-0 points", flatP(m.points).data(), kSandFill2P, 18, 2e-5f);
            // The drawn copy: only the uploaded columns change.
            sand::Mesh u(sp);
            seed(u);
            u.matrix[12] = 100, u.matrix[13] = 10, u.matrix[14] = 120;
            u.fill(0, 1);
            const float gu[2] = {u.vertices()[0].pos[0], u.vertices()[3].pos[0]};  // column 0 uploaded, column 1 not
            const float wu[2] = {kSandFill1V[0], 0};
            check1("PK12 0x5ea870 upload: columns [a, a + b) only", gu[1] == wu[1] && std::fabs(gu[0] - wu[0]) < 1e-3f ? 1.0f : 0.0f, 1, 0);
        }
        {  // tank vf05 0x5ecb20: one slot in state 1, no GenWorld (the ground loops skipped), camera outside; then inside; then near
            sand::Tank t(sp);
            const int s0 = t.addSlot();
            seed(*t.mesh(s0));
            t.state(s0) = 1;
            t.matrix(s0) = {1, 0, 0, 1000, 0, 1, 0, 50, 0, 0, 1, 2000, 300, 40, 350, float(std::sqrt(300.0 * 300 + 40 * 40 + 350 * 350))};
            t.update(0.02f, {1800, 60, 2000}, sand::Ground{});
            const sand::Mesh& m = *t.mesh(s0);
            check("PK12 0x5ecb20 state 1: the wall rebuilt at 0.8 x the OBB extents", flat(m.cpu()).data(), kSandTankV, 342, 2e-5f);
            check("PK12 0x5ecb20 state 1: ring points", flatP(m.points).data(), kSandTankP, 18, 2e-5f);
            std::vector<float> gc;
            for (const auto& c : m.colours) gc.insert(gc.end(), c.begin(), c.end());
            check("PK12 0x5ecb20 state 1: mean colour (rgb / n, alpha 1 + the sum)", gc.data(), kSandTankC, 24, 1e-6f);
            check("PK12 0x5ecb20 mesh +0x44: the slot OBB, extents x 0.8", m.matrix.data(), kSandTankM, 16, 1e-6f);
            const float g1[9] = {m.edge, m.alpha, float(t.state(s0)), t.fog, float(t.fogSet), float(m.instances.size()), m.instances[0][0], m.instances[0][1],
                                 m.instances[0][2]};
            check("PK12 0x5ecb20 edge distance, alpha, state 2, fog floor + flag, instance", g1, kSandTankS, 9, 2e-6f);
            t.fogSet = false;
            t.update(0.02f, {1000, 60, 2000}, sand::Ground{});
            const float g2[5] = {m.edge, m.alpha, float(t.state(s0)), t.fog, float(t.fogSet)};
            check("PK12 0x5ecb20 inside: +0x94 = 10000, alpha 0, fog PP5399", g2, kSandTankIn, 5, 1e-6f);
            t.fogSet = false;
            t.update(0.02f, {1500, 60, 2100}, sand::Ground{});
            const float g3[5] = {m.edge, m.alpha, float(t.state(s0)), t.fog, float(t.fogSet)};
            check("PK12 0x5ecb20 at 194 m: alpha 1, fog (1 - (D - 20) / 230)", g3, kSandTankOut, 5, 2e-6f);
        }
        {  // STARTFX records (0x5bc340 / 0x5c0710 / 0x5c0850 / 0x5c08b0 / 0x5bc3b0 / 0x5bc290)
            sand::StartFx fx(sp);
            sand::Fire f;
            f.node = 0xb7b97ea7, f.hasTime = true, f.time = 60, f.sand = true;
            f.obb = {1, 0, 0, 5, 0, 1, 0, 6, 0, 0, 1, 7, 1, 1, 1, 1};
            sand::Fire g = f;
            g.node = 0x1234, g.sand = false, g.hasTime = false;
            fx.fire(f), fx.fire(g);
            bool hide = false;
            auto hidden = [&](uint32_t n) { return hide && n == 0xb7b97ea7; };
            fx.step(0.5f, hidden);
            const int st1 = fx.tank().state(fx.records()[0].slot);
            fx.step(0.5f, hidden);
            const float ga[9] = {float(fx.records().size()), float(fx.tank().count()), fx.records()[0].duration, fx.records()[1].duration,
                                 fx.records()[0].elapsed, float(st1), float(fx.tank().state(0)), fx.tank().matrix(0)[3], fx.records()[0].centre[2]};
            const float wa[9] = {2, 1, 60, 120, 1.0f, 1, 2, 5, 7};
            check("PK12 STARTFX records: TIME / default 120, SAND slot, state 1 then 2, OBB copied, centre", ga, wa, 9, 1e-6f);
            hide = true;
            fx.step(0.1f, hidden);  // dead
            const float d1 = float(fx.records().size());
            fx.step(0.1f, hidden);  // dropped, its slot freed
            const float gb[4] = {d1, float(fx.records().size()), float(fx.tank().count()), float(fx.tank().slots())};
            const float wb[4] = {2, 1, 0, 0};
            check("PK12 0x5c08b0 hidden node -> dead, 0x5bc3b0 drops it and frees the slot (count 0 empties the arrays)", gb, wb, 4, 0);
            fx.release(0x1234);
            check1("PK12 0x5bc290 release by node", float(fx.records().size()), 0, 0);
        }
        try {  // the shipped data: which STARTFX volumes find a Spline_Z + CollisionVol_Z child (0x570ae0), TIME, SAND, OBB
            const fs::path w = fs::path(dataDir) / "CommonComponent" / "WORLD";
            DpcFile usa1((w / "USA1.DPC").string());
            float g[9] = {0, 0, 0, 0, 0, 0, 0, 0, 0};
            const char* subs[3] = {"USA1_CR9_3924B666.DPC", "USA1_CR36_217ACD30.DPC", "USA1_CR54_9F099175.DPC"};
            for (int k = 0; k < 3; ++k) {
                DpcFile sub((w / subs[k]).string());
                const std::vector<const DpcFile*> lookup = {&sub, &usa1};
                for (const AnimStartVolume& v : loadAnimStartVolumes(sub, lookup)) {
                    g[k] += float(v.startFx.size());
                    for (const StartFxChild& c : v.startFx) {
                        g[3] += float(c.fire.sand), g[4] += float(c.fire.hasTime && c.fire.time == 60);
                        if (c.fire.node == 0xb7b97ea7) g[5] = c.fire.obb[12], g[6] = c.fire.obb[13], g[7] = c.fire.obb[14];
                    }
                    if (v.node == 0x4c1a82d3) g[8] = v.half[0];
                }
            }
            const float wd[9] = {0, 0, 4, 4, 4, 538.66f, 67.82f, 548.33f, 163.5f};
            check("PK12 data: STARTFX children CR9 / CR36 / CR54, SAND, TIME 60, wall OBB extents, trigger half", g, wd, 9, 1e-3f);
        } catch (const std::exception& ex) {
            std::printf("  skip PK12 data: %s\n", ex.what());
        }
    }  // ---- PK12 end

    {  // ---- PK17: the pause backdrop (0x46aca0 / 0x46abd0 / 0x46af90 / 0x46aed0 / 0x4c8c50, WinSound vf15 0x833cc0)
        using namespace fuel::pause;
        HudFade f;
        f.show();
        const float a0 = f.alpha;
        const int r1 = f.step(0.1f);
        const float a1 = f.alpha;
        f.step(0.1f);
        const float a2 = f.alpha;
        const int r3 = f.step(0.1f);
        const float a3 = f.alpha;
        f.hide();
        f.step(0.1f);
        const float a4 = f.alpha;
        f.step(0.1f);
        const int r6 = f.step(0.1f);
        const float gf[8] = {a0, float(r1), a1, a2, float(r3), a3, a4, float(r6)};
        const float wf[8] = {0.001f, 0, 0.401f, 0.801f, 1, 1, 0.6f, -1};
        check("PK17 ViewportHUD fade 0x4c8c50: show 0.001, +4/s, clamp 1 (ret 1), hide -4/s, < 0 ret -1", gf, wf, 8, 1e-6f);
        // 0x46af90 with the shipped PP39..43 = (2, 0.8, 0), 0.005, 0.7 (pp001.tsc; synthetic when the data is absent).
        std::vector<float> pp = fuel::rtc::readPpRows((std::filesystem::path(dataDir) / "CommonComponent/Param/pp001.tsc").string(), 64);
        if (pp.size() < 44) pp.assign(64, 0.0f), pp[39] = 2, pp[40] = 0.8f, pp[41] = 0, pp[42] = 0.005f, pp[43] = 0.7f;
        auto P = [&](int n) { return pp[size_t(n)]; };
        const float gp[5] = {P(39), P(40), P(41), P(42), P(43)};
        const float wp[5] = {2, 0.8f, 0, 0.005f, 0.7f};
        check("PK17 pp001 PP39..43 (menu tint rgb, blur, tint lerp)", gp, wp, 5, 1e-6f);
        const Tint t1 = menuTint(P, 0.5f, 0.2f), t0 = menuTint(P, 0.0f, 0.2f), tf = menuTint(P, 1.0f, 0.0f);
        const float gt[9] = {t1.r, t1.g, t1.b, t1.a, t1.blur, float(t1.blurFlag), t0.a, float(t0.blurFlag), tf.a};
        const float wt[9] = {2, 0.8f, 0, 0.7f * 0.5f + 0.5f * 0.2f, 0.0025f, 1, 0.2f, 0, 0.7f};
        check("PK17 0x46af90 tint lerp PP43 a + (1 - a) +0x6c, blur PP42 a, flag 0x8000 > 0", gt, wt, 9, 1e-6f);
        auto out = [](double x, double z, int mode) { return fuel::fv::Breadcrumbs::worldOutside(x, z, mode == 2); };
        const float go[8] = {float(out(65501, 0, 0)), float(out(65500, -65500, 0)), float(out(0, -65600, 1)), float(out(-50875, -59875, 2)),
                             float(out(-50875 + 2626, -59875, 2)), float(out(-50875, -59875 - 2625, 2)), closedTintA(true, 0.3f),
                             closedTintA(false, 0.3f)};
        const float wo[8] = {1, 0, 1, 0, 1, 0, 0.3f, 0};
        check("PK17 World_IsOutside 0x5280a0 (|x|,|z| > 65500; mode 2 box 2625) and 0x46aed0 close tint", go, wo, 8, 0);
        // 0x780920's cloud argument: intensity only; the sky colour keeps the weather's S+0x68.
        AtmosphereInput in;
        in.timeOfDay = kBackdropTimeOfDay, in.weather.cloud = 0.8f, in.sunCloud = kBackdropSunCloud;
        const Atmosphere ab = computeAtmosphere(in);
        const Atmosphere c0 = at(kBackdropTimeOfDay, 0.0f), c8 = at(kBackdropTimeOfDay, 0.8f);
        const float ga[4] = {ab.intensity, ab.bloomLumScale, ab.dskyColor[0], float(backdropClock(96.0f))};
        const float wa[4] = {c0.intensity, c0.bloomLumScale, c8.dskyColor[0], 450};
        check("PK17 0x46abd0 sky: Atmo_UpdateSun(72000, cloud 0) intensity, dsky by weather, clock 43200/PP371", ga, wa, 4, 1e-6f);
        id<MTLDevice> device = MTLCreateSystemDefaultDevice();
        if (device != nil) {
            ResourceLibrary lib;
            MeshRenderer r(device, 1, lib);
            r.gameClock = 1000;
            r.setDayClock(43200);  // 08:00
            r.gameClock = 1005;    // 08:08
            const double t0 = r.timeOfDay();
            r.setPauseBackdrop(true);
            const double t1 = r.timeOfDay(), c1 = r.dayClockSeconds();
            r.gameClock = 1100;
            r.forcedTimeOfDay = 3600;
            const double t2 = r.timeOfDay(), c2 = r.dayClockSeconds();
            r.forcedTimeOfDay = -1;
            r.setPauseBackdrop(false);
            const double t3 = r.timeOfDay();
            r.gameClock = 1110;
            const double t4 = r.timeOfDay();
            const float gr[6] = {float(std::fmod(t0, 86400.0)), float(t1), float(c1), float(t2), float(c2), float(std::fmod(t3, 86400.0))};
            const float wr[6] = {28800 + 480, 72000, 450, 72000, 450, 28800 + 480};
            check("PK17 renderer pause: 20:00, clock 450 frozen (ForceDayTime too), then the saved clock back", gr, wr, 6, 1e-4f);
            check1("PK17 renderer: after the pause the clock runs on (10 s = 16 min)", float(std::fmod(t4, 86400.0)), 28800 + 480 + 960, 1e-4f);
        }
        // WinSound mask 7: the SFX voices stop where they are and resume from there; mask 0 runs them again.
        {
            fuel::snd::Mixer::setLimiterDefault(false);  // the limiter's 2.5 ms lookahead delays the output (P100)
            fuel::snd::Mixer mx;
            auto pcm = std::make_shared<fuel::snd::Pcm>();
            pcm->rate = fuel::snd::Mixer::kRate, pcm->loops = true;
            for (int i = 0; i < 1000; ++i) pcm->data.push_back(0.001f * float(i % 100));
            fuel::snd::Mixer::PlayParams pa;
            pa.positional = false, pa.attenuate = false;
            const int h = mx.play(pcm, pa);
            mx.setVolume(h, 1.0f);
            mx.update();
            std::vector<float> L(64), R(64), L2(64), R2(64), ref(64), refR(64);
            mx.render(L.data(), R.data(), 64);
            mx.setPauseMask(7);
            mx.render(L2.data(), R2.data(), 64);
            float paused = 0;
            for (float v : L2) paused = std::max(paused, std::fabs(v));
            mx.setPauseMask(0);
            mx.render(L2.data(), R2.data(), 64);
            fuel::snd::Mixer my;
            const int h2 = my.play(pcm, pa);
            my.setVolume(h2, 1.0f);
            my.update();
            my.render(ref.data(), refR.data(), 64);
            my.render(ref.data(), refR.data(), 64);
            float diff = 0;
            for (int i = 0; i < 64; ++i) diff = std::max(diff, std::fabs(L2[size_t(i)] - ref[size_t(i)]));
            const float gm[3] = {paused, diff, float(mx.pauseMask())};
            const float wm[3] = {0, 0, 0};
            check("PK17 WinSound vf15 mask 7: silent, cursor held; mask 0: resumes where it stopped", gm, wm, 3, 1e-6f);
            fuel::snd::Mixer::setLimiterDefault(true);
        }
    }

    {  // ---- PK27: the F14 fly-by (core/f14_flyby, viewer/f14_view; 0x51a380, 0x51a050, 0x4ea560; fuelphysvec FS/FM/FD)
        std::printf("PK27 F14 fly-by (jet node, pass geometry, doppler, the loop under the pause mask)\n");
        try {
            DpcFile usa1((std::filesystem::path(dataDir) / "CommonComponent" / "WORLD" / "USA1.DPC").string());
            F14View v;
            const bool ok = v.load(usa1);
            const auto* e = ok ? usa1.find(v.meshes()[0]) : nullptr;
            const float g[2] = {float(v.meshes().size()), e && e->classCrc == fuel::cls::Mesh ? 1.0f : 0.0f};
            const float w[2] = {1, 1};
            check("PK27 USA1 F14 Lod_Z (name crc 0x410d5e27): one Mesh_Z drawn at level 0", g, w, 2, 0);
        } catch (const std::exception& ex) {
            std::printf("  FAIL PK27 USA1: %s\n", ex.what());
            ++gFail;
        }
        // A pass over flat ground (y 500): starts 1200 m before the centre 300 m ahead of the player, 80 m up, hidden after
        // 2400 m at 250 m/s (9.6 s); the node's z axis follows the velocity; the next pass PP80 later.
        fuel::F14Flyby f;
        f.init(900, 250, 80);
        f.ground = [](float, float, float) { return 500.0f; };
        f.rand01 = [] { return 0.125f; };  // t = pi / 4: heading (sin, 0, cos) = (0.707, 0, 0.707)
        int frames = 0, shownFrames = 0;
        float yMin = 1e9f, yMax = -1e9f, zDot = 2;
        std::array<float, 3> start{};
        bool started = false;
        while (frames < 60 * 900) {
            f.step(1.0f / 60, true, true, {0, 500, 0}, {0, 0, 1});
            ++frames;
            if (f.shown) {
                if (!started) started = true, start = f.nodePos;
                ++shownFrames;
                yMin = std::min(yMin, f.nodePos[1]), yMax = std::max(yMax, f.nodePos[1]);
                const simd_float4x4 m = F14View::world(f);
                const float l = std::hypot(f.vel[0], f.vel[2]);
                zDot = std::min(zDot, (m.columns[2].x * f.vel[0] + m.columns[2].z * f.vel[2]) / l);
            } else if (started) break;
        }
        const float cx = f.end[0] - 1200 * 0.70710677f, cz = f.end[2] - 1200 * 0.70710677f;  // +0x30 = centre + d 1200
        (void)start;
        // the first pass at PP80 - 45 = 855 s on the timer: 45 s after the world loads (frames counts the pass too)
        const float gp2[7] = {(frames - shownFrames) / 60.0f, shownFrames / 60.0f, cx, cz, yMin, yMax, zDot};
        const float wp2[7] = {45.0f, 9.6f, 0, 300, 580, 580, 1};
        check("PK27 0x51a050 first pass 45 s in, 9.6 s over 2400 m, centre 300 m ahead, ground + 80, nose along velocity", gp2, wp2, 7, 0.02f);
        f.step(1.0f / 60, true, true, {0, 500, 0}, {0, 0, 1});
        check1("PK27 0x51a050 after a pass the timer counts from 0 (next pass PP80 = 900 s later)", f.timer, 1.0f / 60, 1e-4f);
        const float gd[4] = {fuel::F14Flyby::dopplerPitch(1000.0, 1000.0 + 250.0 / 60, 1.0f / 60), fuel::F14Flyby::dopplerPitch(1000.0 + 250.0 / 60, 1000.0, 1.0f / 60),
                             fuel::F14Flyby::dopplerPitch(1000.0, 1000.0, 1.0f / 60), fuel::F14Flyby::dopplerPitch(500.0, 500.0 + 0.05 / 60, 1.0f / 60)};
        const float wd[4] = {1.1f, 0.9f, 1.0f, 1.0005f};
        check("PK27 0x4ea560 doppler 1 + clamp(-0.01 dd/dt, +-0.1): approach 1.1, leave 0.9, still 1, 5 cm/s 1.0005", gd, wd, 4, 1e-6f);
        {  // the jet's loop is a plain SFX voice: WinSound's pause mask 7 (PK17) holds it, mask 0 resumes it
            fuel::snd::Mixer::setLimiterDefault(false);  // no lookahead delay in the comparison
            auto pcm = std::make_shared<fuel::snd::Pcm>();
            pcm->channels = 1, pcm->loops = true;
            pcm->data.resize(4410);
            for (size_t i = 0; i < pcm->data.size(); ++i) pcm->data[i] = 0.25f * std::sin(6.2831853f * 441.0f * float(i) / 44100.0f);
            fuel::snd::Mixer mx;
            mx.setListeners({fuel::snd::Listener{}});
            fuel::snd::Mixer::PlayParams pa;
            pa.minDist = fuel::F14Flyby::kSoundMin, pa.maxDist = fuel::F14Flyby::kSoundMax, pa.pos = {0, 80, 200};
            const int h = mx.play(pcm, pa);
            mx.setVolume(h, 1.0f), mx.setPitch(h, 1.1f);
            mx.update();
            std::vector<float> L(256), R(256);
            mx.render(L.data(), R.data(), 256);
            float playing = 0, held = 0;
            for (float x : L) playing = std::max(playing, std::fabs(x));
            mx.setPauseMask(7);
            mx.update();
            mx.render(L.data(), R.data(), 256);
            for (float x : L) held = std::max(held, std::fabs(x));
            const float far = fuel::snd::distanceGain(1500, fuel::F14Flyby::kSoundMin, fuel::F14Flyby::kSoundMax);
            const float near = fuel::snd::distanceGain(216, fuel::F14Flyby::kSoundMin, fuel::F14Flyby::kSoundMax);
            const float gm[4] = {playing > 0.05f ? 1.0f : 0.0f, held, near, far};
            const float wm[4] = {1, 0, 1, 0};
            check("PK27 F14 loop (300 / 1200 m): plays, silent under pause mask 7; gain 1 at 216 m, 0 at 1500 m", gm, wm, 4, 1e-6f);
            fuel::snd::Mixer::setLimiterDefault(true);
        }
    }  // ---- PK27 end

    {  // ---- PK28: material draw state (0x856160 blend, 0x84ca80 LOD bias), RotShape_Z (0x87c8c0 / 0x8834f0 / 0x835a00),
       // Omni_Z light records (0x7984c0 / 0x821830 / 0x6a2ed0) and the RTC switch (0x536d40 0xc6 / 0xc7)
        std::printf("PK28 material draw state, RotShape, omni records\n");
        namespace fs = std::filesystem;
        {  // 0x856160: D3DRS_BLENDOP / SRCBLEND / DESTBLEND per flag set, tested 0x1000, then 0x4000, then 0x2000
            const uint32_t fl[6] = {0x1000, 0x4000, 0x2000, 0x3000, 0x6000, 0x20};
            const uint32_t want[6][3] = {{1, 5, 2}, {1, 7, 2}, {3, 5, 2}, {1, 5, 2}, {1, 7, 2}, {1, 5, 6}};
            float g[18], w[18];
            for (int i = 0; i < 6; ++i) {
                const BlendState b = blendState(fl[i]);
                g[3 * i] = float(b.op), g[3 * i + 1] = float(b.src), g[3 * i + 2] = float(b.dst);
                for (int k = 0; k < 3; ++k) w[3 * i + k] = float(want[i][k]);
            }
            check("PK28 0x856160 blend (op, src, dst): 0x1000, 0x4000, 0x2000, 0x3000, 0x6000, plain", g, w, 18, 0);
            const MetalBlend a = metalBlend(BlendKind::Add), da = metalBlend(BlendKind::AddDestAlpha), rs = metalBlend(BlendKind::RevSubtract),
                             al = metalBlend(BlendKind::Alpha);
            const float gm[12] = {float(a.op), float(a.src), float(a.dst), float(da.op), float(da.src), float(da.dst),
                                  float(rs.op), float(rs.src), float(rs.dst), float(al.op), float(al.src), float(al.dst)};
            const float wm[12] = {float(MTLBlendOperationAdd), float(MTLBlendFactorSourceAlpha), float(MTLBlendFactorOne),
                                  float(MTLBlendOperationAdd), float(MTLBlendFactorDestinationAlpha), float(MTLBlendFactorOne),
                                  float(MTLBlendOperationReverseSubtract), float(MTLBlendFactorSourceAlpha), float(MTLBlendFactorOne),
                                  float(MTLBlendOperationAdd), float(MTLBlendFactorSourceAlpha), float(MTLBlendFactorOneMinusSourceAlpha)};
            check("PK28 Metal blend: add, add by dest alpha, reverse subtract (dest - src a), alpha", gm, wm, 12, 0);
            // reverse subtract on numbers: dest 0.8, src 0.5 at alpha 0.6 -> 0.8 - 0.3; add -> 1.1
            const float gr[2] = {0.8f - 0.5f * 0.6f, 0.8f + 0.5f * 0.6f}, wr[2] = {0.5f, 1.1f};
            check("PK28 0x2000 dest - src x a, 0x1000 dest + src x a (0.8, 0.5, a 0.6)", gr, wr, 2, 1e-6f);
        }
        {  // 0x84ca80: MIPMAPLODBIAS = renderer +0x1104 ([0xa05224] in Renderer_RegisterTweaks 0x6500d8) + +0x1100 (fldz) + Bitmap_Z +0x24
            const std::vector<float> c = readEngineFloats((fs::path(dataDir) / "XFinalComponent" / "FUEL.exe").string(), 0xa05224, 1);
            check1("PK28 0x6500d8 renderer +0x1104 = [0xa05224] (the shaders' kTexBias -1)", c.empty() ? 99.0f : c[0], -1.0f, 0);
            std::vector<uint8_t> link(32, 0);
            const float b = -2.0f;
            std::memcpy(&link[22], &b, 4);
            link[0x1E] = 0x0C;  // any format code
            float got = 99;
            try {
                got = parseBitmapLink(link).lodBias;
            } catch (const std::exception&) {
            }
            check1("PK28 Bitmap_Z +0x24 = link bytes 22..25 (synthetic -2)", got, -2.0f, 0);
            try {  // shipped: PILOT.DPC 344 bitmaps at -1 and 17 at 0, TRUCK.DPC 12 at -2 and 724 at 0
                float h[5] = {0, 0, 0, 0, 0};
                for (int f = 0; f < 2; ++f) {
                    DpcFile d((fs::path(dataDir) / "CommonComponent" / "DATAS" / (f == 0 ? "PILOT.DPC" : "TRUCK.DPC")).string());
                    for (uint32_t id : d.ids(cls::Bitmap)) {
                        const float lb = parseBitmapLink(d.link(*d.find(id))).lodBias;
                        if (f == 0) h[lb == -1.0f ? 0 : lb == 0.0f ? 1 : 4] += 1;
                        else h[lb == -2.0f ? 2 : lb == 0.0f ? 3 : 4] += 1;
                    }
                }
                const float w[5] = {344, 17, 12, 724, 0};
                check("PK28 shipped Bitmap_Z +0x24: PILOT -1 / 0, TRUCK -2 / 0, other values", h, w, 5, 0);
            } catch (const std::exception& e) {
                std::printf("  FAIL PK28 bitmap bias data: %s\n", e.what());
                ++gFail;
            }
        }
        // RotShape_Z body (0x87c8c0): pos[n] (12 B), +0x74 [n] (16 B), material index [n] (flag 2: read twice), axes [n] (12 B),
        // +0x8c [n] (8 B), handles [n], +0x9c f32, +0xa0 u16
        auto put = [](std::vector<uint8_t>& v, const void* p, size_t n) { v.insert(v.end(), (const uint8_t*)p, (const uint8_t*)p + n); };
        auto u32 = [&](std::vector<uint8_t>& v, uint32_t x) { put(v, &x, 4); };
        auto f32 = [&](std::vector<uint8_t>& v, float x) { put(v, &x, 4); };
        std::vector<uint8_t> link(98, 0), body;
        {
            const uint32_t flags = 0x001ffc30;
            const float sph[4] = {0, 1.2f, 0, 1.5f}, view = 300;
            std::memcpy(&link[8], sph, 16), std::memcpy(&link[88], &view, 4), std::memcpy(&link[92], &flags, 4);
            u32(body, 2), f32(body, 1), f32(body, 2), f32(body, 3), f32(body, 4), f32(body, 5), f32(body, 6);
            u32(body, 1);
            for (int k = 0; k < 4; ++k) f32(body, 0);
            u32(body, 2), u32(body, 1), u32(body, 0);
            u32(body, 4);
            const float ax[12] = {-0.69f, 1.04f, 0, 0.69f, -1.04f, 0, -1, 2, 0, 1, 0, 0};
            for (float x : ax) f32(body, x);
            u32(body, 1), f32(body, 0), f32(body, 1);
            u32(body, 2), u32(body, 0xaaaa0001), u32(body, 0xbbbb0002);
            f32(body, 200), body.push_back(1), body.push_back(0);
        }
        try {
            const rotshape::Shape sh = rotshape::parse(0x1234, link, body);
            const float g[14] = {float(sh.points.size()), sh.points[1].pos[2], sh.points[0].upLeft[1], sh.points[0].downRight[0], sh.points[1].upLeft[1],
                                 float(sh.points[0].material), float(sh.points[1].material), float(sh.materials[1] == 0xbbbb0002u), sh.nearFade,
                                 sh.viewDist, sh.front ? 1.0f : 0.0f, float(sh.flags == 0x001ffc30u), sh.sphere[3], float(sh.unused74 + sh.unused8c)};
            const float w[14] = {2, 6, 1.04f, 0.69f, 2, 1, 0, 1, 200, 300, 1, 1, 1.5f, 2};
            check("PK28 0x87c8c0 RotShape body: points, pivots, axis pairs, material index, handles, near fade, front", g, w, 14, 1e-6f);
        } catch (const std::exception& e) {
            std::printf("  FAIL PK28 RotShape parse: %s\n", e.what());
            ++gFail;
        }
        {
            std::vector<uint8_t> extra = body;
            extra.push_back(0);
            bool threw = false;
            try {
                (void)rotshape::parse(1, link, extra);
            } catch (const std::exception&) {
                threw = true;
            }
            check1("PK28 RotShape body with a trailing byte is rejected", threw ? 1.0f : 0.0f, 1.0f, 0);
        }
        try {  // shipped: CR2's start-line glow (USA1_CR2_3AF3648E.DPC ad7d7989: 1 point, material fa19df31, vh_front)
            DpcFile cr2((fs::path(dataDir) / "CommonComponent" / "WORLD" / "USA1_CR2_3AF3648E.DPC").string());
            const DpcFile::Entry* e = cr2.find(0xad7d7989u);
            const rotshape::Shape sh = rotshape::parse(0xad7d7989u, cr2.link(*e), cr2.body(*e).value());
            const float g[8] = {float(sh.points.size()), sh.points[0].upLeft[0], sh.points[0].upLeft[1], sh.points[0].downRight[0],
                                sh.points[0].downRight[1], float(sh.materials.at(0) == 0xfa19df31u), sh.front ? 1.0f : 0.0f, sh.nearFade + sh.viewDist};
            const float w[8] = {1, -0.689453f, 1.037109f, 0.689453f, -1.037109f, 1, 1, 0};
            check("PK28 shipped RotShape ad7d7989 (CR2): 1 point, UPLEFT / DWRIGHT, material fa19df31, front, no fade", g, w, 8, 2e-3f);
        } catch (const std::exception& e) {
            std::printf("  FAIL PK28 shipped RotShape: %s\n", e.what());
            ++gFail;
        }
        {  // 0x8834f0 -> ComputeFade(+0x5c 300, +0x9c 200, 0, K' 10, s = 1 x 0.3972, d): far fade 20 m from 1191.6 m, near (d / 794.4)^3
            rotshape::Shape sh;
            sh.viewDist = 300, sh.nearFade = 200;
            rotshape::Shape none;
            const float g[6] = {rotshape::fade(sh, 400, 1), rotshape::fade(sh, 1000, 1), rotshape::fade(sh, 1201.6f, 1), rotshape::fade(sh, 1300, 1),
                                rotshape::fade(none, 5000, 1), rotshape::fade(none, 10, 1, 0.3972f, 10, 0)};
            const float q = 400.0f / (200 * 10 * 0.3972f);
            const float w[6] = {q * q * q, 1, 0.5f, 20 - (1300 - 1191.6f), 1, 0};
            check("PK28 0x8834f0 RotShape fade: near cube, 1, far half, past the far end, no fade; node +0x5c 0 -> 0", g, w, 6, 2e-4f);
        }
        {  // scat_rotshape.vhl corners, CPU (rotshape::corner) against the GPU (rotShapeCorner, shaderCheck case 12)
            rotshape::Shape sh;
            rotshape::Point pt;
            pt.upLeft = {-0.69f, 1.04f, 0}, pt.downRight = {0.69f, -1.04f, 0};
            const std::array<float, 3> pv = {3.0f, -2.0f, 30.0f}, up = {0.1f, 0.98f, -0.17f};
            std::vector<simd_float4> gin;
            std::vector<std::array<float, 3>> want;
            for (int front = 0; front < 2; ++front)
                for (int k = 0; k < 4; ++k) {
                    sh.front = front != 0;
                    want.push_back(rotshape::corner(sh, pt, k, pv, up, 1.6f, 1.0f));
                    gin.insert(gin.end(), {simd_make_float4(pv[0], pv[1], pv[2], 12), simd_make_float4(pt.upLeft[0], pt.upLeft[1], pt.upLeft[2], float(front + 2 * k)),
                                           simd_make_float4(pt.downRight[0], pt.downRight[1], pt.downRight[2], 1.0f), simd_make_float4(up[0], up[1], up[2], 1.6f)});
                }
            sh.front = true;
            want.push_back(rotshape::corner(sh, pt, 0, pv, up, 1.6f, 1.0f / 512));  // collapsed: the pivot
            gin.insert(gin.end(), {simd_make_float4(pv[0], pv[1], pv[2], 12), simd_make_float4(pt.upLeft[0], pt.upLeft[1], pt.upLeft[2], 1),
                                   simd_make_float4(pt.downRight[0], pt.downRight[1], pt.downRight[2], 1.0f / 512), simd_make_float4(up[0], up[1], up[2], 1.6f)});
            // vh_front at the screen centre (pivot on +z, up = y): corner 0 (UPLEFT) = pivot + 1.6 (-0.69, 1.04, 0) seen from the eye
            const auto c0 = rotshape::corner(sh, pt, 0, {0, 0, 10}, {0, 1, 0}, 1.6f, 1.0f);
            check3("PK28 vh_front corner 0 = pivot + s UPLEFT (camera-facing, x right)", c0, {-0.69f * 1.6f, 1.04f * 1.6f, 10}, 1e-5f);
            id<MTLDevice> device = MTLCreateSystemDefaultDevice();
            if (device != nil) {
                ResourceLibrary lib;
                MeshRenderer r(device, 1, lib);
                const std::vector<simd_float4> out = r.shaderCheck(at(hm(12, 0)), simd_make_float3(0, 900, 0), gin);
                if (out.size() != gin.size() / 2) {
                    std::printf("  FAIL PK28 GPU RotShape corners did not run\n");
                    ++gFail;
                } else {
                    float g[27], w[27];
                    for (size_t i = 0; i < 9; ++i)
                        for (int k = 0; k < 3; ++k) g[3 * i + size_t(k)] = out[2 * i][k], w[3 * i + size_t(k)] = want[i][size_t(k)];
                    check("PK28 GPU vh_normal x4, vh_front x4, collapsed corner = CPU", g, w, 27, 1e-5f);
                }
            }
        }
        {  // 0x7984c0 records (headlight Omni_Z: +0x114 0.262, +0x118 0.785, +0x124 r 5, flags 0x28400028)
            omni::Params p;
            p.inner = 5, p.outer = 250;
            omni::ObjectInfo o;
            o.flags = 0x28400028u, o.bound = 250, o.inner = 0.262f, o.outer = 0.785f;
            const omni::Record s = omni::makeRecord(o, p, {1, 2, 3}, {0, 0, -1}, 1.5f);
            o.flags = 0x08400028u;
            const omni::Record n = omni::makeRecord(o, p, {1, 2, 3}, {0, 0, -1}, 1.5f);
            const float g[9] = {s.range2, s.invSin, s.sin2, s.cos2, s.dir[2], s.radius, n.invSin, n.dir[2], n.spot ? 1.0f : 0.0f};
            const float w[9] = {56.25f, 1.0f / std::sin(0.785f), std::sin(0.785f) * std::sin(0.785f), std::cos(0.785f) * std::cos(0.785f), -1, 375, 0, 0, 0};
            check("PK28 0x7984c0 record: (r s)^2, 1 / sin o, sin^2, cos^2, spot dir, bound x s; omni: 0 / dir 0", g, w, 9, 1e-5f);
        }
        {  // 0x821830 sphere vs view (planes: a 90 deg frustum through the eye, near 0.5, far 45 km)
            const float h = std::sqrt(0.5f);
            const std::array<omni::V3, 4> pl = {{{h, 0, h}, {-h, 0, h}, {0, h, h}, {0, -h, h}}};
            const float g[5] = {omni::sphereVisible({0, 0, 10}, 1, pl, 0.5f, 45000) ? 1.0f : 0.0f, omni::sphereVisible({0, 0, -10}, 1, pl, 0.5f, 45000) ? 1.0f : 0.0f,
                                omni::sphereVisible({0, 0, -10}, 250, pl, 0.5f, 45000) ? 1.0f : 0.0f, omni::sphereVisible({30, 0, 10}, 5, pl, 0.5f, 45000) ? 1.0f : 0.0f,
                                omni::sphereVisible({0, 0, 45100}, 250, pl, 0.5f, 45000) ? 1.0f : 0.0f};
            const float w[5] = {1, 0, 1, 0, 1};
            check("PK28 0x821830: in front, behind, behind but 250 m reach, off the side, past far within r", g, w, 5, 0);
        }
        {  // 0x6a2ed0: 64 records, later ones overwrite slot 63; RTC 0xc6 / 0xc7 (0x536d40) on +0x60
            std::vector<int> list;
            for (int i = 0; i < 70; ++i) omni::appendRecord(list, i);
            const float g[7] = {float(list.size()), float(list[62]), float(list[63]), float(omni::rtcSwitch(0x08400028u, false)), float(omni::rtcSwitch(0x08000028u, true)),
                                float(omni::rtcSwitch(0x18400028u, false)), float(omni::rtcSwitch(0x00400028u, false))};
            const float w[7] = {64, 62, 69, float(0x08000028u), float(0x08400028u), float(0x18400028u), float(0x00400028u)};
            check("PK28 0x6a2ed0 cap 64 (slot 63 overwritten); 0xc6 / 0xc7: 0x8000000 switches, 0x10000000 keeps", g, w, 7, 0);
        }
        {  // the gathered headlights on the GPU: 70 spot records -> 64 drawn (0x6a2ed0), a non-spot record is not drawn (0x7fdd80)
            id<MTLDevice> device = MTLCreateSystemDefaultDevice();
            if (device != nil) {
                ResourceLibrary lib;
                MeshRenderer r(device, 1, lib);
                r.forcedTimeOfDay = 21 * 3600.0;
                std::vector<MeshRenderer::VehicleOmni> ls;
                for (int i = 0; i < 70; ++i) {
                    MeshRenderer::VehicleOmni o;
                    o.pos = simd_make_float3(float(i) * 3, 900, 500);
                    o.value = simd_make_float4(25, 1.0f / (62500 - 25), -3.864f, 3.732f);
                    ls.push_back(o);
                }
                MeshRenderer::VehicleOmni nonSpot = ls[0];
                nonSpot.flags = 0x08400028u;
                std::vector<simd_float4> gin = {simd_make_float4(0, 900, 0, 10), simd_make_float4(0, 1, 0, 0), 0, 0};
                r.setVehicleLights(ls);
                const std::vector<simd_float4> o64 = r.shaderCheck(at(hm(21, 0)), simd_make_float3(0, 900, 0), gin);
                r.setVehicleLights({nonSpot, ls[1]});
                const std::vector<simd_float4> o1 = r.shaderCheck(at(hm(21, 0)), simd_make_float3(0, 900, 0), gin);
                const float g[2] = {o64.empty() ? -1 : o64[0].w, o1.empty() ? -1 : o1[0].w}, w[2] = {64, 1};
                check("PK28 GPU light count: 70 gathered -> 64, a non-spot Omni_Z is not drawn", g, w, 2, 0);
            }
        }
    }  // ---- PK28 end

    // ---- TAA [port option] (taa_math.h, taa_render.mm): the jitter, the jitter-aware projection, the LOD bias and the
    // motion vectors (GPU kTaaCheck = the shaders' taaCameraMotion / taaMotion / taaViewDistance against double maths).
    {
        std::printf("TAA (MetalFX temporal: jitter, motion vectors):\n");
        {  // Halton (2, 3) from index 1, minus 0.5
            float g[8], w[8] = {0.0f, 1.0f / 3 - 0.5f, -0.25f, 2.0f / 3 - 0.5f, 0.25f, 1.0f / 9 - 0.5f, -0.375f, 4.0f / 9 - 0.5f};
            for (int k = 0; k < 4; ++k) g[2 * k] = taa::jitter(uint32_t(k)).x, g[2 * k + 1] = taa::jitter(uint32_t(k)).y;
            check("TAA jitter: Halton (2, 3) phases 0..3 (render px, -0.5)", g, w, 8, 1e-6f);
            double sx = 0, sy = 0, lo = 1, hi = -1;
            for (uint32_t k = 0; k < taa::kPhases; ++k) {
                const simd_float2 j = taa::jitter(k);
                sx += j.x, sy += j.y, lo = std::min({lo, double(j.x), double(j.y)}), hi = std::max({hi, double(j.x), double(j.y)});
            }
            const float g2[4] = {float(sx / taa::kPhases), float(sy / taa::kPhases), float(lo >= -0.5 ? 1 : 0), float(hi < 0.5 ? 1 : 0)};
            const float w2[4] = {0, 0, 1, 1};
            check("TAA jitter: 32 phases centred (mean x, y within 0.02), all in [-0.5, 0.5)", g2, w2, 4, 0.02f);
            const simd_float2 j32 = taa::jitter(32), j0 = taa::jitter(0);
            const float g3[2] = {j32.x - j0.x, j32.y - j0.y}, w3[2] = {0, 0};
            check("TAA jitter: the cycle repeats after 32 frames", g3, w3, 2, 0);
        }
        {  // LOD bias: FUEL's -1 at native, + log2(scale) on a scaled temporal frame
            const float g[4] = {taa::lodBias(-1, 1.0f), taa::lodBias(-1, 0.8f), taa::lodBias(-1, 0.67f), taa::lodBias(-1, 0.5f)};
            const float w[4] = {-1.0f, -1.0f + std::log2(0.8f), -1.0f + std::log2(0.67f), -2.0f};
            check("TAA LOD bias: -1 native, -1 + log2(s) at 0.8 / 0.67 / 0.5", g, w, 4, 1e-6f);
        }
        // Two cameras as the renderer makes them (lookAtRH x perspectiveRH, fuelClip camera-relative): the chase camera
        // 2.0 m further on and turned 3 degrees, a 1710 x 1112 render area at 0.67 of 2552 x 1660.
        const float W = 1710, H = 1112, fovY = 0.9f, aspect = W / H, zn = 0.4f, zf = 120000.0f, q = zf / (zn - zf);
        auto camera = [&](simd_float3 eye, simd_float3 target) {
            const simd_float3 z = simd_normalize(eye - target), x = simd_normalize(simd_cross(simd_make_float3(0, 1, 0), z)), y = simd_cross(z, x);
            const simd_float4x4 view = simd_matrix(simd_make_float4(x.x, y.x, z.x, 0), simd_make_float4(x.y, y.y, z.y, 0), simd_make_float4(x.z, y.z, z.z, 0),
                                                   simd_make_float4(-simd_dot(x, eye), -simd_dot(y, eye), -simd_dot(z, eye), 1));
            const float f = 1.0f / std::tan(fovY * 0.5f);
            const simd_float4x4 proj = simd_matrix(simd_make_float4(f / aspect, 0, 0, 0), simd_make_float4(0, f, 0, 0), simd_make_float4(0, 0, q, -1),
                                                   simd_make_float4(0, 0, zn * q, 0));
            return simd_mul(proj, view);
        };
        const simd_float3 e0 = simd_make_float3(-39700.0f, 905.5f, -28300.0f), e1 = e0 + simd_make_float3(0.6f, 0.1f, 1.9f);
        const simd_float3 t0 = e0 + simd_make_float3(0, -1.2f, 10), t1 = e1 + simd_make_float3(std::sin(0.0524f) * 10, -1.25f, std::cos(0.0524f) * 10);
        const simd_float4x4 vpPrev = camera(e0, t0), vpCur = camera(e1, t1);
        const simd_float2 jpx = taa::jitter(5);
        const simd_float4x4 vpJit = taa::jittered(vpCur, jpx, W, H);
        auto clipOf = [&](const simd_float4x4& vp, simd_float3 eye, simd_float3 p) {  // fuelClip in float
            simd_float4 c = simd_mul(vp, simd_make_float4(p - eye, 0));
            c.z += zn * q;
            return c;
        };
        auto ndcD = [](const simd_float4x4& vp, simd_float3 eye, simd_float3 p) {  // the same in double
            double c[4] = {0, 0, 0, 0};
            for (int k = 0; k < 4; ++k)
                for (int i = 0; i < 3; ++i) c[k] += double(vp.columns[i][k]) * (double(p[i]) - double(eye[i]));
            return simd_make_double2(c[0] / c[3], c[1] / c[3]);
        };
        {  // the jitter moves the picture by (jx, jy) render pixels and leaves the depth
            const simd_float3 p = e1 + simd_make_float3(3.0f, -0.4f, 25.0f);
            const simd_float4 a = clipOf(vpCur, e1, p), b = clipOf(vpJit, e1, p);
            const float g[3] = {(b.x / b.w - a.x / a.w) * 0.5f * W, -(b.y / b.w - a.y / a.w) * 0.5f * H, b.z / b.w - a.z / a.w};
            const float w[3] = {jpx.x, jpx.y, 0.0f};
            check("TAA jittered projection: image moved by the jitter (px), depth kept", g, w, 3, 2e-3f);
        }
        // Points: ground, a sign, far hills, the sky (depth 1), and a vehicle part that moved 0.8 m.
        struct Pt { simd_float3 p, prevP; bool sky; };
        const std::vector<Pt> pts = {{e1 + simd_make_float3(1.5f, -1.6f, 6.0f), {}, false},
                                     {e1 + simd_make_float3(-4.0f, 1.0f, 30.0f), {}, false},
                                     {e1 + simd_make_float3(400.0f, 80.0f, 2500.0f), {}, false},
                                     {e1 + simd_make_float3(0.3f, 0.05f, 1.0f) * 500000.0f, {}, true},
                                     {e1 + simd_make_float3(0.2f, -0.9f, 5.0f), e1 + simd_make_float3(0.2f, -0.9f, 5.0f) - simd_make_float3(0.1f, 0.0f, 0.8f), false}};
        std::vector<simd_float4> in;
        for (const Pt& pt : pts) {
            simd_float4 c = clipOf(vpJit, e1, pt.p);
            if (pt.sky) c = simd_make_float4(c.x / c.w, c.y / c.w, 1.0f, 1.0f);  // the sky: depth 1 (the far plane), its ray
            in.push_back(simd_make_float4(c.x / c.w, c.y / c.w, c.z / c.w, 0));
            in.push_back(simd_make_float4(pt.p, 1));
            in.push_back(simd_make_float4(simd_length(pt.prevP) > 0 ? pt.prevP : pt.p, 1));
        }
        id<MTLDevice> device = MTLCreateSystemDefaultDevice();
        if (device != nil) {
            ResourceLibrary lib;
            MeshRenderer r(device, 1, lib);
            MeshRenderer::TaaCheckCamera cc, pc;
            cc.vp = vpCur, cc.jitteredVp = vpJit, cc.eye = e1, cc.zn = zn, cc.q = q;
            pc.vp = vpPrev, pc.jitteredVp = vpPrev, pc.eye = e0, pc.zn = zn, pc.q = q;
            const std::vector<simd_float4> out = r.taaCheck(cc, pc, W, H, in);
            const char* names[5] = {"ground 6 m", "sign 30 m", "hills 2.5 km", "sky (depth 1)", "moving part 5 m"};
            for (size_t i = 0; i < pts.size() && out.size() == 2 * pts.size(); ++i) {
                const Pt& pt = pts[i];
                // Expected (double): where the static point was minus where it is now (unjittered), uv of the render area.
                const simd_double2 nowN = ndcD(vpCur, e1, pt.p), wasN = ndcD(vpPrev, e0, pt.p);
                const float wantCam[2] = {float((wasN.x - nowN.x) * 0.5 * W), float(-(wasN.y - nowN.y) * 0.5 * H)};
                const float gotCam[2] = {out[2 * i].x * W, out[2 * i].y * H};
                char what[160];
                auto err = [](const float* g, const float* w) { return std::max(std::fabs(g[0] - w[0]), std::fabs(g[1] - w[1])); };
                // Absolute: the error in render pixels (0.02 px), the motion itself in the label.
                std::snprintf(what, sizeof what, "TAA GPU camera motion %s (%.2f, %.2f px): error", names[i], wantCam[0], wantCam[1]);
                if (simd_length(pt.prevP) == 0) check1(what, err(gotCam, wantCam), 0, 0.02f);
                const simd_double2 wasP = ndcD(vpPrev, e0, simd_length(pt.prevP) > 0 ? pt.prevP : pt.p);
                const float wantObj[2] = {float((wasP.x - nowN.x) * 0.5 * W), float(-(wasP.y - nowN.y) * 0.5 * H)};
                const float gotObj[2] = {out[2 * i + 1].x * W, out[2 * i + 1].y * H};
                std::snprintf(what, sizeof what, "TAA GPU object motion %s (%.2f, %.2f px): error", names[i], wantObj[0], wantObj[1]);
                if (!pt.sky) check1(what, err(gotObj, wantObj), 0, 0.02f);
                if (!pt.sky && i < 3) {  // the object pass's depth match: the resolved depth's view distance = the clip w
                    std::snprintf(what, sizeof what, "TAA view distance from depth = clip w, %s", names[i]);
                    check1(what, out[2 * i].z, out[2 * i + 1].z, 1e-3f);
                }
            }
            if (out.size() != 2 * pts.size()) check1("TAA kTaaCheck ran", 0, 1, 0);
        }
        {  // MetalFX's convention: a thing that moved 10 px right and 10 px down since the last frame has motion (-10, -10) px
            const simd_float4 cur = simd_make_float4(0.25f, -0.5f, 0.3f, 1.0f);
            const simd_float4 prev = simd_make_float4(0.25f - 20.0f / W, -0.5f + 20.0f / H, 0.3f, 1.0f);
            const float g[2] = {(prev.x / prev.w - cur.x / cur.w) * 0.5f * W, (prev.y / prev.w - cur.y / cur.w) * -0.5f * H}, w[2] = {-10, -10};
            check("TAA motion convention: moved 10 px right / down -> (-10, -10) px", g, w, 2, 1e-3f);
        }
    }  // ---- TAA end

    {  // ---- beam-colour: the navigator beams' texture lookup and colour (0x4bf820, 0x5fba20, sfx.vhl/phl ph_billboard)
        namespace fs = std::filesystem;
        std::printf("beam-colour navigator beams (FX_NAVIGATOR3D_LINE)\n");
        const float gu[4] = {sfx::navBeamU(0, 8), sfx::navBeamU(7, 8), sfx::navBeamU(1, 3), sfx::navBeamU(1, 2)};
        const float wu[4] = {0.05f, 0.95f, 0.5f, 0.95f};
        check("beam-colour 0x5fba20 u = 0.05 + 0.9 k / (n - 1): ground 0.05, band 0.95", gu, wu, 4, 1e-6f);
        const std::array<float, 4> pc = sfx::navBeamParticleColour({1.0f / 510, 127.0f / 510, 196.0f / 510, 1.0f});
        const std::array<float, 4> px = sfx::navBeamPixel(pc, {0.5f, 0.25f, 1.0f, 0.5f});
        const float gp[4] = {px[0], px[1], px[2], px[3]}, wp[4] = {8.0f / 510, 127.0f * 8 / 510 * 0.5f, 196.0f * 8 / 510 * 2, 0.5f};
        check("beam-colour ph_billboard: rgb = record x 8 x texel x 2, a = record a x texel a", gp, wp, 4, 1e-6f);
        try {
            const std::string exe = (fs::path(dataDir) / "XFinalComponent" / "FUEL.exe").string();
            const sfx::Tables t = sfx::loadTables(exe, (fs::path(dataDir) / "CommonComponent" / "Param" / "pp001.tsc").string());
            if (!t.ok || t.atlas.size() <= 0x1d) throw std::runtime_error(t.error.empty() ? "no FX atlas" : t.error);
            const sfx::AtlasEntry& e = t.atlas[0x1d];
            // The atlas entry's rect is mirrored in u: applying it (as the port did) puts the texture's dim, transparent end
            // at the band. FUEL's ribbons never apply it.
            const float gr[4] = {e.u0, e.v0, e.du, e.dv}, wr[4] = {1, 0, -1, 1};
            check("beam-colour FX atlas 0x1d rect (u0, v0, du, dv): mirrored, not used by ribbons", gr, wr, 4, 0);
            std::printf("  %-4s %-58s got %s\n", e.material == "FX_NAVIGATOR3D_LINE" ? "ok" : "FAIL", "beam-colour FX atlas 0x1d material", e.material.c_str());
            ++(e.material == "FX_NAVIGATOR3D_LINE" ? gPass : gFail);
            const float gc[3] = {t.P(5), t.P(6), t.P(7)}, wc[3] = {1, 127, 196};
            check("beam-colour PP5-7: the livery / doppler 0xf beam record x 510", gc, wc, 3, 0);
            // The texture: MaterialAnim FX_NAVIGATOR3D_LINE -> Material -> slot 0 Bitmap (SHARED.DPC 8e46e3e3, 32 x 8 DXT5).
            DpcFile d((fs::path(dataDir) / "CommonComponent" / "DATAS" / "SHARED.DPC").string());
            std::optional<std::vector<uint8_t>> bmBody;
            BitmapInfo bi;
            for (uint32_t oid : d.ids(cls::MaterialAnimZ)) {
                const DpcFile::Entry* me = d.find(oid);
                const std::vector<uint8_t> link = d.link(*me);
                uint32_t name = 0;
                if (link.size() >= 4) std::memcpy(&name, link.data(), 4);
                if (name != asoboCrc("FX_NAVIGATOR3D_LINE")) continue;
                const auto body = d.body(*me);
                if (!body) break;
                const MaterialAnim ma = parseMaterialAnimBody(*body);
                const DpcFile::Entry* mat = d.find(ma.material);
                if (!mat) break;
                const Material m = parseMaterialBody(d.body(*mat).value());
                const DpcFile::Entry* be = d.find(m.textures[0]);
                if (!be) break;
                bi = parseBitmapLink(d.link(*be));
                bmBody = d.body(*be);
                break;
            }
            if (!bmBody || bi.format != PixelFormat::DXT5 || bi.width != 32 || bi.height != 8 || bmBody->size() < 256)
                throw std::runtime_error("FX_NAVIGATOR3D_LINE bitmap not found as 32 x 8 DXT5");
            // Nearest texel (x, y) of mip 0, DXT5 decoded as D3D (565 colours expanded, 1/3 2/3 interpolation, 8-step alpha).
            auto texel = [&](int x, int y) {
                const uint8_t* b = bmBody->data() + ((y / 4) * (32 / 4) + x / 4) * 16;
                const int i = (y % 4) * 4 + x % 4;
                const int a0 = b[0], a1 = b[1];
                uint64_t abits = 0;
                for (int k = 0; k < 6; ++k) abits |= uint64_t(b[2 + k]) << (8 * k);
                const int ai = int((abits >> (3 * i)) & 7);
                float a;
                if (ai == 0) a = float(a0);
                else if (ai == 1) a = float(a1);
                else if (a0 > a1) a = float((8 - ai) * a0 + (ai - 1) * a1) / 7.0f;
                else a = ai == 6 ? 0.0f : ai == 7 ? 255.0f : float((6 - ai) * a0 + (ai - 1) * a1) / 5.0f;
                const uint16_t c0 = uint16_t(b[8] | (b[9] << 8)), c1 = uint16_t(b[10] | (b[11] << 8));
                const uint32_t ci = (uint32_t(b[12]) | (uint32_t(b[13]) << 8) | (uint32_t(b[14]) << 16) | (uint32_t(b[15]) << 24)) >> (2 * i) & 3;
                auto ex = [](uint16_t c) { return std::array<float, 3>{float(c >> 11) * 255 / 31, float((c >> 5) & 63) * 255 / 63, float(c & 31) * 255 / 31}; };
                const std::array<float, 3> p0 = ex(c0), p1 = ex(c1);
                std::array<float, 4> r{};
                for (int k = 0; k < 3; ++k)
                    r[size_t(k)] = (ci == 0 ? p0[size_t(k)] : ci == 1 ? p1[size_t(k)] : ci == 2 ? (2 * p0[size_t(k)] + p1[size_t(k)]) / 3 : (p0[size_t(k)] + 2 * p1[size_t(k)]) / 3) / 255.0f;
                r[3] = a / 255.0f;
                return r;
            };
            // Centre row (v 0.5 -> row 4) at the ground end (u 0.05 -> column 1) and the band end (u 0.95 -> column 30).
            const std::array<float, 4> lo = texel(int(sfx::navBeamU(0, 8) * 32), 4), hi = texel(int(sfx::navBeamU(7, 8) * 32), 4);
            const float gt[8] = {lo[0], lo[1], lo[2], lo[3], hi[0], hi[1], hi[2], hi[3]};
            const float wt[8] = {24.0f / 255, 25.0f / 255, 24.0f / 255, 29.0f / 255, 213.0f / 255, 214.0f / 255, 213.0f / 255, 233.0f / 255};
            check("beam-colour texel at the ground (u .05) dim / clear, at the band (u .95) bright / opaque", gt, wt, 8, 0.02f);
            // The livery beam's HDR value at the band: record PP5-7 / 510 x 8, texel, x 2 = (0.03, 3.33, 5.12), alpha 0.91.
            const std::array<float, 4> band = sfx::navBeamPixel(sfx::navBeamParticleColour({t.P(5) / 510, t.P(6) / 510, t.P(7) / 510, 1}), hi);
            const float gb[4] = {band[0], band[1], band[2], band[3]};
            const float wb[4] = {8.0f / 510 * 2 * 213 / 255, 127.0f * 8 / 510 * 2 * 214 / 255, 196.0f * 8 / 510 * 2 * 213 / 255, 233.0f / 255};
            check("beam-colour livery beam HDR at the band (no atlas flip)", gb, wb, 4, 0.02f);
        } catch (const std::exception& ex) {
            std::printf("  skip beam-colour data vectors: %s\n", ex.what());
        }
    }  // ---- beam-colour end

    // ---- [upgrades U5, DEVIATION] the visible upgrades (core/upgrade_visuals; notes/upg5.md)
    std::printf("upgrades U5: visible upgrades\n");
    {
        namespace fs = std::filesystem;
        // The segment map: C fixed, B -> B', the across scale about the C-B line.
        const Mat4 m = upgvis::segmentMap({0, 1, 0}, {1, 0.5f, 0}, {1, 0.4f, 0}, 1.2f);
        const auto c = upgvis::transformPoint(m, {0, 1, 0}), b = upgvis::transformPoint(m, {1, 0.5f, 0});
        const auto q = upgvis::transformPoint(m, {0.5f, 0.75f, 0.1f});  // the mid point, 0.1 m across (along the hinge axis z)
        const float gs[9] = {c[0], c[1], c[2], b[0], b[1], b[2], q[0], q[1], q[2]};
        const float ws[9] = {0, 1, 0, 1, 0.4f, 0, 0.5f, 0.7f, 0.12f};
        check("U5 segment map: C fixed, B -> B', mid point + 0.1 across -> mid of C B' + 0.12 (x1.2)", gs, ws, 9, 1e-5f);
        // The engine stage's flame: the nitro flame's length x1.8 at nitrous power stage 3 (VehicleFx, one slot).
        const sfx::Tables t = sfx::loadTables((fs::path(dataDir) / "XFinalComponent" / "FUEL.exe").string(),
                                              (fs::path(dataDir) / "CommonComponent" / "Param" / "pp001.tsc").string());
        if (t.ok) {
            auto flameLength = [&](float k) {
                sfx::System sys(t);
                vfx::VehicleFx fx(sys);
                std::vector<vfx::VehicleIn> in(1);
                in[0].slotPos = {{0, 1, 0}}, in[0].slotDir = {{0, 0, -1}};
                in[0].visible = true, in[0].topSpeed = 100, in[0].cameraDistance = 10, in[0].nitro = 1, in[0].nitroLength = k, in[0].nitroSize = k;
                vfx::PlayerIn pl;
                pl.valid = true, pl.gear = 1;
                for (int i = 0; i < 3; ++i) fx.update(1.0f / 60, in, pl, {0, 2, -10});
                std::vector<vfx::SmokeSprite> sm;
                std::vector<vfx::FlameQuad> fl;
                std::vector<vfx::Streak> st;
                fx.collect(sm, fl, st, {0, 2, -10});
                float lo = 1e9f;
                for (const vfx::FlameQuad& f : fl)
                    for (const auto& v : f.v) lo = std::min(lo, v[2]);
                return fl.empty() ? 0.0f : -(lo) + 0.05f;  // the planes reach P- + L d (P- = pos - 0.05 d, d = -z)
            };
            const float l1 = flameLength(1.0f), l3 = flameLength(1.8f);
            check1("U5 nitro flame length at nitrous power S3 / stock (x1.8)", l1 > 0 ? l3 / l1 : 0.0f, 1.8f, 1e-3f);
        }
        // The plans on FUEL's data: stock = FUEL's parts; QUAD_ROAD_2 / SUV_OFF_4 at stage 3 (player and opponent).
        try {
            const auto params = parseVehiclesParam(custom::readScript((fs::path(dataDir) / "CommonComponent/Param/VehiclesParam.tsc").string()));
            auto setupOf = [&](const DpcFile& dpc, const AssembledVehicle& av, const char* name, const char* stages) {
                std::vector<std::array<float, 3>> hubs;
                for (const PlacedPart& p : av.parts)
                    if (p.drawn() && (p.placement == Placement::Wheel || p.placement == Placement::WheelFlipped))
                        hubs.push_back({float(p.world[12]), float(p.world[13]), float(p.world[14])});
                VehicleSetup s = makeVehicleSetup(dpc, name, params.at(name), hubs);
                upg::Stages st;
                upg::parseStages(stages, st);
                upg::apply(s, st, 1.0f);
                return s;
            };
            DpcFile quad((fs::path(dataDir) / "CommonComponent/DATAS/QUAD.DPC").string());
            VehicleAssembler qa(quad);
            const AssembledVehicle qav = qa.assemble(*qa.find("QUAD_ROAD_2"), 1);
            const VehicleSetup s0 = setupOf(quad, qav, "QUAD_ROAD_2", "stock");
            const upgvis::Plan p0 = upgvis::makePlan(qav, s0);
            const auto r0 = upgvis::restScene(qav, s0, p0);
            bool same = !p0.active;
            size_t drawn = 0;
            for (const PlacedPart& p : qav.parts) drawn += p.drawn();
            same = same && r0.size() == drawn;
            for (const auto& sp : r0) same = same && sp.mesh == qav.parts[size_t(sp.part)].meshId && sp.world == qav.parts[size_t(sp.part)].world;
            check1("U5 stock stages: no plan, the rest scene is FUEL's parts (meshes and matrices bit for bit)", same ? 1.0f : 0.0f, 1, 0);
            const VehicleSetup s3 = setupOf(quad, qav, "QUAD_ROAD_2", "max");
            const upgvis::Plan p3 = upgvis::makePlan(qav, s3);
            int wheels = 0, uprights = 0, uprightOk = 0, extrasWheel = 0, chassis = 0;
            for (const upgvis::PartEdit& e : p3.parts) {
                if (e.wheel >= 0 && e.axle == 1.25f && e.radial >= 1.0f) ++wheels;
                if (e.why == "upright") ++uprights, uprightOk += std::fabs(e.adjust[13] + p3.lift) < 1e-6 && e.adjust[12] == 0 && e.adjust[14] == 0;
            }
            for (const upgvis::Extra& x : p3.extras) (x.wheel >= 0 ? extrasWheel : chassis)++;
            const float gq[8] = {float(p3.active), p3.lift, p3.widenFront, float(p3.carbon), float(wheels), float(uprights > 0 && uprightOk == uprights),
                                 float(extrasWheel), float(chassis)};
            const float wq[8] = {1, 0.06f, 0.06f, 3, 4, 1, 4, 1};
            check("U5 QUAD_ROAD_2 all S3: plan, lift 6 cm, widen 6 cm, carbon 3, 4 wheels x1.25, uprights down the lift, 4 wheel + 1 chassis extras", gq, wq, 8, 1e-6f);
            upgvis::Options op;
            op.opponent = true;
            const upgvis::Plan po = upgvis::makePlan(qav, s3, op);
            int poChassis = 0;
            for (const upgvis::Extra& x : po.extras) poChassis += x.wheel < 0;
            const float go[4] = {float(po.active), float(poChassis), po.engineGainDb, po.flameLength};
            const float wo[4] = {1, 0, 0, 1.8f};
            check("U5 opponent subset: no chassis extras (bottles, tips, coils), no engine note, the flame x1.8", go, wo, 4, 1e-6f);
            upg::setEnabled(false);
            const VehicleSetup soff = setupOf(quad, qav, "QUAD_ROAD_2", "max");
            const bool offInactive = !upgvis::makePlan(qav, soff).active;
            upg::setEnabled(true);
            upgvis::setEnabled(false);
            const bool visOff = !upgvis::makePlan(qav, s3).active;
            upgvis::setEnabled(true);
            const float gf[2] = {float(offInactive), float(visOff)};
            const float wf[2] = {1, 1};
            check("U5 upgrades = off / FUEL_UPGRADE_VISUALS=0: no plan", gf, wf, 2, 0);
            // SUV_OFF_4 weight S3: the spare wheel part and the BUMPER batch go, the hull is a derived mesh; suspension S3: two flares
            DpcFile suv((fs::path(dataDir) / "CommonComponent/DATAS/SUV.DPC").string());
            VehicleAssembler sa(suv);
            const AssembledVehicle sav = sa.assemble(*sa.find("SUV_OFF_4"), 1);
            const VehicleSetup ss = setupOf(suv, sav, "SUV_OFF_4", "0,3,0,3,0,0");
            const upgvis::Plan ps = upgvis::makePlan(sav, ss);
            int hidden = 0, bumper = 0, flares = 0;
            for (size_t k = 0; k < ps.parts.size(); ++k) {
                hidden += ps.parts[k].hide;
                if (const upgvis::Derived* d = ps.parts[k].mesh ? upgvis::derivedMesh(ps.parts[k].mesh) : nullptr)
                    bumper += std::count(d->hiddenMaterials.begin(), d->hiddenMaterials.end(), asoboCrc("BUMPER")) > 0 && d->carbon == 3 && d->base == sav.parts[k].meshId;
            }
            for (const std::string& n : ps.notes) flares += n.rfind("flare over wheel", 0) == 0;
            const float gv[4] = {float(hidden), float(bumper), float(flares), ps.widenFront};
            const float wv[4] = {1, 1, 2, 0.3f};
            check("U5 SUV_OFF_4 S3 weight + suspension: WHEELADD hidden, hull derived (BUMPER out, carbon 3), 2 front flares, widen 0.3", gv, wv, 4, 2e-3f);
        } catch (const std::exception& ex) {
            std::printf("  skip U5 data vectors: %s\n", ex.what());
        }
    }
    // ---- [upgrades U5] end

    std::printf("rendercheck: %d passed, %d failed\n", gPass, gFail);
    return gFail == 0 ? 0 : 1;
}

}  // namespace fuel
