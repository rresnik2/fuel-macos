#import "rendercheck.h"

#import <Metal/Metal.h>

#include "core/atmosphere.hpp"
#include "core/sfx.hpp"
#include "core/weather.hpp"
#include "core/clouds.hpp"
#import "renderer.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
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
        check4("SC3 at the peak", a.c[kSC3], {0, 0.8214052f, 0.5703452f, 1}, 1e-5f);
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
            for (int t = 0; t < 2; ++t) {
                const Atmosphere a = at(t == 0 ? hm(8, 0) : hm(20, 30), 0, true, 860);
                const simd_float3 eye = simd_make_float3(-39570, 866, -28068);
                AtmosphereInput in;
                in.timeOfDay = t == 0 ? hm(8, 0) : hm(20, 30);
                in.camera = {eye.x, eye.y, eye.z};
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
                const char* when = t == 0 ? "08:00" : "20:30";
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
                    check3(name, gi, insc, 2e-3f);
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
                    check3(name, {out[size_t(12 + k * 2)].x, out[size_t(12 + k * 2)].y, out[size_t(12 + k * 2)].z}, insc, 2e-3f);
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

    std::printf("rendercheck: %d passed, %d failed\n", gPass, gFail);
    return gFail == 0 ? 0 : 1;
}

}  // namespace fuel
