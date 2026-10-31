// FUEL's sun, sky light, atmospheric scattering and HDR post maths on the CPU (P28 port of the P26
// renderer decode, ~/re-work/fuel/notes/p26-renderer.md and p26-work/post.md, shadows.md).
//
// Per frame the game turns the time of day and the weather into:
//   - the sun / moon position (Atmo_UpdateSun 0x780920), the exposure range, bloom scale and blue shift
//     that GenWorld_FrameUpdate 0x77a3b0 copies into the viewports every frame;
//   - the sun and sky light colours (Preetham transmittance 0x781d70, 0x782030 / 0x782080 / 0x782200) and
//     directions (GenWorld_UpdateLights 0x779d40);
//   - the eleven scattering constants VS c6..c16 (Scatter_BuildShaderConstants 0x7816c0) from the 13
//     scatter parameters (defaults 0x780ce0, weather overrides 0x6409d0).
// The shaders (renderer.mm) run the same scattering / lighting / tone mapping; this file is the CPU copy
// the renderer fills its uniforms from and `fuelview --rendercheck` checks against the decode's vectors.
// Scalar constants are the engine's (cited by address); nothing here is a copied table.
#pragma once

#include <array>
#include <cstdint>

namespace fuel {

using F3 = std::array<float, 3>;
using F4 = std::array<float, 4>;

// Weather inputs of World_SetWeatherAtmosphere 0x6409d0 (all 0..1). The port has no weather fields yet
// (WeatherManager_G samples world-generator fields), so the default is a clear, dry sky.
struct Weather {
    float cloud = 0;      // arg1: cloud amount
    float fog = 0;        // arg2: fogPrime = max(fog, 0.5 max(rain, snow))
    float wet = 0;        // arg6: max(rain, snow, ...)
    float cloudBase = 0;  // arg3 h0 (+ W+0xbc): cloud layer bottom candidate, metres
    float cloudTopArg = 0;  // arg5: lower bound - 100 of the cloud layer bottom
    bool apply = true;    // false: the parameter defaults of 0x780ce0 untouched (as the decode's L5 vectors)
};

struct AtmosphereInput {
    double timeOfDay = 8 * 3600;  // seconds (WeatherManager +0xac)
    Weather weather;
    float brightness = 1.0f;      // the Brightness option G = 0.5 + 1.5 slider (A+0); default Unknown (U2): 1
    F3 camera = {0, 0, 0};        // world position (metres)
};

// Everything the shaders take per frame.
struct Atmosphere {
    // Sun (0x780920)
    F3 sunRaw{};            // un-lifted unit direction towards the sun (or the moon at night, y mirrored)
    float sunHeight = 0;    // signed y of the sun before the night mirror
    bool day = true;
    float theta = 0;        // zenith angle of sunRaw (S+0x40)
    F3 lightDir{};          // DLIGHT_DIR: normalize(x, max(|y|, 0.05) + 0.25, z) (A+0xbc)
    F3 skyDir{};            // DSKY_DIR: normalize(0.3 L.x, 1, 0.3 L.z)
    // Exposure, bloom, blue shift (A+0x4..0x24 -> viewport, 0x77a3b0)
    float intensity = 0;    // I (A+0x8)
    float screenLuminance = 0, minExposure = 0, maxExposure = 0, bloomLumScale = 0;
    F4 blueShift{};
    // Light colours
    F3 tau{};               // Preetham transmittance at theta (night tint applied)
    F3 dlightColor{};       // c66 DLIGHT_COLOR = 4 tau / |tau|
    F3 dlightAmbient{};     // c65 DLIGHT_AMBIENT = 4 tau(0.8 theta) / |...|
    F3 dskyColor{};         // c68 DSKY_COLOR
    F3 dskyAmbient{};       // c69 = 0
    float sunLuminance = 0; // S+0x48 = max(|tau| (halved at night), 0.5)
    float invIntensity = 0; // DLIGHT_INVINTENSITY = 1 / S+0x48 / blueShift.w
    // Scattering
    std::array<float, 13> P{};  // scatter parameters by index (setter 0x780f20)
    float sunTerm = 1;          // s of SC1 (camera -> sun transmittance term)
    // c6..c16: SC1, SC2, TERM, BETA_1, BETA_DASH_1, BETA_2, BETA_DASH_2, BETA_1_PLUS_2, ONE_OVER, HG, SC3
    std::array<F4, 11> c{};
    F3 sunColour{};         // gDSunColor = DLIGHT_COLOR * SC1.z: the "sunlight" of every lit shader
};

enum ScatterConst { kSC1, kSC2, kTERM, kBETA1, kBETADASH1, kBETA2, kBETADASH2, kBETASUM, kONEOVER, kHG, kSC3 };

// The full per-frame evaluation.
Atmosphere computeAtmosphere(const AtmosphereInput& in);

// Pieces (exposed for the checks).
F3 sunDirection(double timeOfDay, float* signedY);         // 0x780920 sun path (un-mirrored)
F3 preethamTau(float theta);                               // 0x781d70, turbidity 3
void lightColours(Atmosphere& a, float cloud);             // DLIGHT / DSKY colours from a.theta, a.day (a.blueShift set)
std::array<float, 13> scatterDefaults();                   // 0x780ce0
void applyWeather(std::array<float, 13>& P, const Weather& w);  // 0x6409d0
// scatterintegral (scattering.h / 0x7813d0): density integral along a segment of length `len` between heights.
float scatterIntegral(const Atmosphere& a, float len, float y1, float y2);
// scatteringbase: extinction and inscatter seen from `eye` for a point `p`.
void scattering(const Atmosphere& a, const F3& eye, const F3& p, F3& extinction, F3& inscatter);

// lighting() of scattering_light.h (no omnis), for the checks.
F3 lighting(const Atmosphere& a, const F3& n, const F3& V, const F3& albedo, const F3& specCol, float power, float shadow, float occ);

// ---- Post chain (post.md P3), per pixel / per frame
float toneCurve(float x);  // P / (1 + P), P = x + x^2 + x^3 + x^4
// k = 0.94^(30 dt) (0x843254)
float adaptationRate(float dt);
// Exposure S (= 1 / adapted luminance): init and one step (ph_lumadaptationinit / ph_lumadaptation).
float exposureInit(float avgLogLum, float screenLum);
float exposureStep(float prevS, float avgLogLum, float screenLum, float minExp, float maxExp, float dt);
// Bright pass (ph_hdrbloominit) for one channel value v.
float brightPass(float v, float S, float lumScale, float bias, float scale, float screenLum);
// Blue shift then exposure, bloom and the curve (ph_tonemapping), tint lerp. bloom = bloom + star sum.
F3 toneMapPixel(F3 c, float S, F3 bloom, const F4& blueShift, const F4& tint);
// 11-tap gaussian (0x657450): offsets in texels (x128 of the uv offsets) and weights.
void gaussian11(float offsets[11], float weights[11]);
// Radial blur weights (0x657690).
void motionBlurWeights(float decay, float w[8]);

// The post settings record (Viewport ctor 0x665f40, .data 0xa06680, 16 floats) and the shadow table
// (0xa05240: splits, zbias x4, ZBiasAdd, ZExtent), read from the player's FUEL.exe at run time.
// Without the exe the bloom is off and the shadow splits are the port's own guesses (fromExe false).
struct PostDefaults {
    F4 bloom = {0, 0, 0, 0};  // LumScale, Bias, Scale, Star
    F4 tint = {1, 1, 1, 0};   // Tint rgb, lerp
    bool fromExe = false;
};
struct ShadowSettings {
    F4 splits = {20, 80, 320, 1600};  // view-space z where slices 1, 2, 3 start and the shadows end
    F4 zbias = {0.08f, 0.08f, 0.08f, 0.08f};
    float zbiasAdd = 0.03f, zExtent = 10000.0f;
    bool fromExe = false;
};
// Reads both from the FUEL.exe at `exePath` (read only); leaves the fallbacks when it cannot.
void readEngineRenderSettings(const char* exePath, PostDefaults& post, ShadowSettings& shadow);

}  // namespace fuel
