#include "atmosphere.hpp"

#include "genworld_noise.hpp"

#include <algorithm>
#include <cmath>
#include <string>

namespace fuel {

namespace {

constexpr double kPi = 3.14159265358979323846;

F3 normalize3(F3 v) {
    const float l = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    return l > 0 ? F3{v[0] / l, v[1] / l, v[2] / l} : v;
}
float len3(const F3& v) { return std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]); }
float clamp01(float v) { return std::min(1.0f, std::max(0.0f, v)); }

// Rotation of v about the unit axis k by `angle` (Quat_FromAxisAngle 0x40f8b0 + Quat_RotateVector 0x410510:
// q v q*, i.e. v cos + (k x v) sin + k (k.v)(1 - cos)).
F3 rotate(const F3& v, const F3& k, double angle) {
    const double c = std::cos(angle), s = std::sin(angle);
    const double kv = double(k[0]) * v[0] + double(k[1]) * v[1] + double(k[2]) * v[2];
    const double cx = double(k[1]) * v[2] - double(k[2]) * v[1], cy = double(k[2]) * v[0] - double(k[0]) * v[2],
                 cz = double(k[0]) * v[1] - double(k[1]) * v[0];
    return {float(v[0] * c + cx * s + k[0] * kv * (1 - c)), float(v[1] * c + cy * s + k[1] * kv * (1 - c)),
            float(v[2] * c + cz * s + k[2] * kv * (1 - c))};
}

// Wavelengths of the three channels in micrometres ([0x9dc240], [0x9dc974], [0x9dc970]).
constexpr double kLambda[3] = {0.65, 0.57, 0.475};

}  // namespace

F3 sunDirection(double t, float* signedY) {
    // f = frac(t / 86400) (the game's FPU rounds down); 16 hours of day 06:00..22:00 -> phase 0.25..0.75.
    const double f = t / 86400.0 - std::floor(t / 86400.0);
    const double p = f >= 0.25 ? 0.75 * f + 0.0625 : 0.75 * f + 0.8125;
    // R1 = (0, -1, 0) turned 0.75 rad about +x ([0xa0632c], [0xa06338]); then about normalize(R1 x X) by 2 pi p.
    const F3 r1 = rotate({0, -1, 0}, {1, 0, 0}, 0.75);
    const F3 axis = normalize3({r1[1] * 0 - r1[2] * 0, r1[2] * 1 - r1[0] * 0, r1[0] * 0 - r1[1] * 1});
    const F3 sun = rotate(r1, axis, 2 * kPi * p);
    if (signedY) *signedY = sun[1];
    return sun;
}

F3 preethamTau(float theta) {
    // Preetham sun transmittance, turbidity 3 (0x781d70): Rayleigh and aerosol terms only.
    double th = theta;
    if (std::cos(th) < 0) th = kPi - th;
    const double deg = th * 180.0 / kPi;
    const double m = 1.0 / (std::cos(th) + 0.15 * std::pow(93.885 - deg, -1.253));
    F3 tau{};
    for (int i = 0; i < 3; ++i)
        tau[size_t(i)] = float(std::exp(-m * 0.008735 * std::pow(kLambda[i], -4.08)) * std::exp(-m * 0.0923907 * std::pow(kLambda[i], -1.3)));
    return tau;
}

std::array<float, 13> scatterDefaults() {
    // Scatter_InitParams 0x780ce0 (index order of the setter 0x780f20).
    return {0.05f, 0.0005f, 45.0f, 1.0f, 0.9f, 10000.0f, 0.7f, 500.0f, 0.5f, 0.0f, 10000.0f, 20000.0f, 1.0f};
}

void applyWeather(std::array<float, 13>& P, const Weather& w) {
    // World_SetWeatherAtmosphere 0x6409d0 (disassembly read: idx 10, 11, 9, 12, 6, 8 in that order).
    P[10] = std::min(std::max(w.cloudBase, w.cloudTopArg + 100.0f), 1500.0f);  // cloud layer bottom
    P[11] = w.cloudBase + 2000.0f;                                              // cloud layer top
    P[9] = w.cloud * 50.0f;                                                     // cloud layer density
    const float a = w.fog, c = w.cloud;
    const float a8 = std::fabs(a) * a * a * a * a * a * a * a;  // |a| a^7
    P[12] = 0.9f * c * (2.0f - std::fabs(c)) * (1.0f - a8);     // horizon thickness weight
    P[6] = std::sqrt(std::max(a8, 0.0f)) * 500.0f + 0.7f;        // layer-1 density
    P[8] = (w.wet + 1.0f) * 5.0f;                                // layer-2 density
}

float scatterIntegral(const Atmosphere& a, float len, float y1, float y2) {
    // scattering.h scatterintegral (and the CPU copy 0x7813d0).
    const F4 &SC1 = a.c[kSC1], &SC2 = a.c[kSC2], &TERM = a.c[kTERM], &HG = a.c[kHG];
    const float hi = std::max(y1, y2), lo = std::min(y1, y2);
    float r11[4] = {hi * TERM[3], lo * TERM[3], hi * SC2[3], lo * SC2[3]};
    float r10[4];
    for (int i = 0; i < 4; ++i) r10[i] = std::min(r11[i], 1.0f);
    float d[4] = {r11[0] - r11[1], r11[2] - r11[3], r10[0] - r10[1], r10[2] - r10[3]};
    for (float& v : d) v = std::max(v, TERM[3]);
    const float f1 = d[2] / d[0], f2 = d[3] / d[1];
    const float dens1 = (1.0f - (r10[1] + r10[0]) * 0.5f) * f1 * SC2[1];
    const float dens2 = (1.0f - (r10[3] + r10[2]) * 0.5f) * f2 * SC2[2];
    float integral = (dens1 + dens2) * len;
    const float chi = std::min(std::max(hi, SC1[1]), TERM[2]), clo = std::min(std::max(lo, SC1[1]), TERM[2]);
    const float tot = hi - lo, cloudH = chi - clo;
    integral += len * cloudH / std::max(tot, 1.0f / 16384.0f) * HG[3];
    return integral;
}

namespace {

void scatteringFinish(const Atmosphere& a, float lookY, float density, float cosTheta, F3& ext, F3& insc) {
    const F4 &SC2 = a.c[kSC2], &TERM = a.c[kTERM], &HG = a.c[kHG], &SC3 = a.c[kSC3];
    const float cc = std::min(0.0f, cosTheta);
    const float phase1 = 1.0f + cc * cc;
    for (int i = 0; i < 3; ++i) ext[size_t(i)] = std::exp2(-a.c[kBETASUM][size_t(i)] * SC2[0] * density);
    const float r = 1.0f / std::sqrt(HG[1] + HG[2] * cosTheta);
    const float phase2 = HG[0] * (r * r * r * r);
    float ep = std::max(0.0f, 1.0f + lookY);
    ep *= ep, ep *= ep, ep *= ep;
    ep = 1.0f - SC3[3] + ep * SC3[3];
    float tw = 0.5f - cosTheta * 0.5f;
    tw = clamp01(tw * tw);
    for (int i = 0; i < 3; ++i) {
        const size_t k = size_t(i);
        float v = (a.c[kBETADASH1][k] * phase1 + a.c[kBETADASH2][k] * phase2) * a.c[kONEOVER][k];
        v *= (1.0f - ext[k]) * ep;
        v *= (a.dlightColor[k] * tw + (1.0f - tw)) * TERM[1];
        insc[k] = v;
    }
    const float l = std::sqrt(len3(ext));
    for (float& e : ext) e *= l;
}

}  // namespace

void scattering(const Atmosphere& a, const F3& eye, const F3& p, F3& ext, F3& insc) {
    F3 look = {eye[0] - p[0], eye[1] - p[1], eye[2] - p[2]};
    const float len = len3(look);
    if (len > 0)
        for (float& v : look) v /= len;
    const F4& SC3 = a.c[kSC3];
    const float cosT = look[0] * SC3[0] + look[1] * SC3[1] + look[2] * SC3[2];
    const float density = scatterIntegral(a, len, 0.25f + std::max(p[1], 0.0f), eye[1]);
    scatteringFinish(a, std::min(look[1], 0.0f), density, cosT, ext, insc);
}

void lightColours(Atmosphere& a, float cloud) {
    // Atmo_ComputeSunColours 0x781bd0 and 0x782030 / 0x782080 / 0x782200 (needs a.theta, a.day, a.blueShift).
    const F3 night = {0.1f, 0.1f, 0.5f};
    auto tauAt = [&](float th) {
        F3 t = preethamTau(th);
        if (!a.day)
            for (int i = 0; i < 3; ++i) t[size_t(i)] *= night[size_t(i)];
        return t;
    };
    a.tau = tauAt(a.theta);
    const F3 tau8 = tauAt(0.8f * a.theta);
    const float lt = len3(a.tau), l8 = len3(tau8);
    for (int i = 0; i < 3; ++i) {
        a.dlightColor[size_t(i)] = 4 * a.tau[size_t(i)] / lt;
        a.dlightAmbient[size_t(i)] = 4 * tau8[size_t(i)] / l8;
    }
    const float cth = std::fabs(std::cos(a.theta));
    const F3 skyTint = {0.9f, 0.9f, 1.0f};
    for (int i = 0; i < 3; ++i)
        a.dskyColor[size_t(i)] = a.dlightAmbient[size_t(i)] * (1 + 2.5f * (1 - cth)) * skyTint[size_t(i)] * (1.2f + 0.3f * cloud);
    a.dskyAmbient = {0, 0, 0};
    a.sunLuminance = std::max(a.day ? lt : lt * 0.5f, 0.5f);
    a.invIntensity = 1.0f / a.sunLuminance / a.blueShift[3];

}

Atmosphere computeAtmosphere(const AtmosphereInput& in) {
    Atmosphere a;
    // ---- Sun (Atmo_UpdateSun 0x780920)
    float y = 0;
    F3 sun = sunDirection(in.timeOfDay, &y);
    a.sunHeight = y;
    a.day = y > 0;
    if (!a.day) sun[1] = -sun[1];  // the moon: the sun mirrored below the horizon
    a.sunRaw = sun;
    a.theta = float(kPi / 2 - std::atan2(double(sun[1]), std::sqrt(double(sun[0]) * sun[0] + double(sun[2]) * sun[2])));
    a.lightDir = normalize3({sun[0], std::max(std::fabs(sun[1]), 0.05f) + 0.25f, sun[2]});
    a.skyDir = normalize3({0.3f * a.lightDir[0], 1.0f, 0.3f * a.lightDir[2]});

    // ---- Intensity, exposure range, bloom scale, blue shift
    const float cloud = clamp01(in.weather.cloud);
    const float u = clamp01(y), om = 1 - u, fu = 1 - om * om * om * om;
    a.intensity = (0.1f * (1 - fu) + fu * (y > 0 ? 0.5f : 0.1f)) * (1 - (2 - fu) * cloud * fu * 0.25f);
    a.bloomLumScale = 3.00077f * a.intensity;
    // GenWorld_FrameUpdate 0x77a3b0: the viewport values are these times the Brightness option A+0.
    a.screenLuminance = a.intensity * in.brightness;
    a.minExposure = 0.33f * a.intensity * in.brightness;
    a.maxExposure = 20.0f * a.intensity * in.brightness;
    const float v = 1 - clamp01(4 * y);
    a.blueShift = {0.4f, 0.5f, 1.0f, 10.0f - 9.5f * std::pow(v, 64.0f)};

    lightColours(a, cloud);

    // ---- Scatter parameters and constants (0x780ce0, 0x6409d0, 0x780e10, 0x7816c0)
    a.P = scatterDefaults();
    if (in.weather.apply) applyWeather(a.P, in.weather);
    const auto& P = a.P;
    F3 bR, bRd, bM, bMd;
    const double K[3] = {0.685, 0.679, 0.67};
    for (int i = 0; i < 3; ++i) {
        const double l = kLambda[i] * 1e-6, l2 = l * l, l4 = l2 * l2;
        bR[size_t(i)] = float(1.2444614e-28 / l4 * P[0]);
        bRd[size_t(i)] = float(7.4273315e-30 / l4 * P[0]);
        bM[size_t(i)] = float(3.5407335e-15 * K[i] / l2 * P[1]);
        bMd[size_t(i)] = float(5.635252e-16 / l2 * P[1]);
    }
    const float g = P[4];
    a.c[kSC2] = {1.442695f, P[6], P[8], 1.0f / P[7]};
    a.c[kTERM] = {P[3], P[2], P[11], 1.0f / P[5]};
    a.c[kBETA1] = {bR[0], bR[1], bR[2], 1};
    a.c[kBETADASH1] = {bRd[0], bRd[1], bRd[2], 1};
    a.c[kBETA2] = {bM[0], bM[1], bM[2], 1};
    a.c[kBETADASH2] = {bMd[0], bMd[1], bMd[2], 1};
    a.c[kBETASUM] = {bR[0] + bM[0], bR[1] + bM[1], bR[2] + bM[2], 2};
    a.c[kONEOVER] = {1 / a.c[kBETASUM][0], 1 / a.c[kBETASUM][1], 1 / a.c[kBETASUM][2], 100000};
    a.c[kHG] = {1 - g * g, 1 + g, 2 * g, P[9]};
    // Sun position = camera + DLIGHT_DIR * 10000 (0x779d40): SC3 = its direction from the camera.
    a.c[kSC3] = {a.lightDir[0], a.lightDir[1], a.lightDir[2], P[12]};
    a.c[kSC1] = {1, P[10], 4, 0};
    // Sun term s: the camera -> sun density integral (0x7813d0), T = 2^(-0.33 log2e sum(beta) d).
    const float sunY = in.camera[1] + a.lightDir[1] * 10000.0f;
    const float d = scatterIntegral(a, 10000.0f, in.camera[1], sunY);
    const float sb = a.c[kBETASUM][0] + a.c[kBETASUM][1] + a.c[kBETASUM][2];
    const float T = std::exp2(-0.33f * 1.442695f * sb * d);
    a.sunTerm = std::max(1.0f - std::pow(1.0f - T, 64.0f), 0.1f);
    a.c[kSC1] = {a.sunTerm, P[10], 4 * a.sunTerm, 0};
    for (int i = 0; i < 3; ++i) a.sunColour[size_t(i)] = a.dlightColor[size_t(i)] * a.c[kSC1][2];
    return a;
}

// ---- Post chain

float toneCurve(float x) {
    const float p = x + x * x + x * x * x + x * x * x * x;
    return p / (1 + p);
}

float adaptationRate(float dt) { return float(std::pow(0.94, 30.0 * double(dt))); }

float exposureInit(float A, float sl) { return sl / std::exp(A); }

float exposureStep(float prevS, float A, float sl, float minExp, float maxExp, float dt) {
    const float c = std::min(std::max(1.5f * std::exp(A) / sl, 1.0f / maxExp), 1.0f / minExp);
    const float a = 1.0f / prevS;
    const float an = c * std::pow(a / c, adaptationRate(dt));
    float S = 1.0f / an;
    if (!(S > 0.01f && S < 100.0f)) S = 1.0f / c;
    return S;
}

float brightPass(float v, float S, float lumScale, float bias, float scale, float sl) {
    return std::min(1.0f, std::max(0.0f, (4 * v * lumScale * (0.3f / sl) * S - bias) * scale));
}

F3 toneMapPixel(F3 c, float S, F3 bloom, const F4& bs, const F4& tint) {
    const float bw = 0.2125f * c[0] + 0.7154f * c[1] + 0.0721f * c[2];
    float t = 1 - std::min(1.0f, bs[3] * bw);
    t *= t;
    for (int i = 0; i < 3; ++i) {
        const size_t k = size_t(i);
        c[k] = c[k] + (bs[k] * bw - c[k]) * t;
        c[k] = toneCurve(c[k] * S + bloom[k]);
    }
    const float br = 0.2125f * c[0] + 0.7154f * c[1] + 0.0721f * c[2];
    for (int i = 0; i < 3; ++i) c[size_t(i)] += (br * tint[size_t(i)] - c[size_t(i)]) * tint[3];
    return c;
}

void gaussian11(float off[11], float w[11]) {
    // Post_MakeGaussian11 0x657450: sigma 3 texels, normalised; centred on -1.5 texels.
    double sum = 0, raw[11];
    for (int k = -5; k <= 5; ++k) sum += raw[k + 5] = std::exp(-double(k * k) / 18.0) / std::sqrt(2 * kPi * 9);
    for (int k = -5; k <= 5; ++k) {
        off[k + 5] = float(k) - 1.5f;
        w[k + 5] = float(raw[k + 5] / sum);
    }
}

void motionBlurWeights(float d, float w[8]) {
    double sum = 0;
    for (int i = 0; i < 8; ++i) sum += std::pow(double(d), i);
    for (int i = 0; i < 8; ++i) w[i] = float(std::pow(double(d), i) / sum);
}

void readEngineRenderSettings(const char* exePath, PostDefaults& post, ShadowSettings& shadow) {
    const std::string exe = exePath ? exePath : "";
    // Viewport post defaults (Viewport_Ctor 0x665f40 copies .data 0xa06680..0xa066bc).
    const std::vector<float> p = readEngineFloats(exe, 0xa06680, 16);
    if (p.size() == 16) {
        post.bloom = {p[0], p[1], p[2], p[3]};
        post.tint = {p[12], p[13], p[14], p[15]};
        post.fromExe = true;
    }
    // Shadow splits 0xa05240, per-slice z bias 0xa05250, ZBiasAdd 0xa05260, ZExtent 0xa05264.
    const std::vector<float> s = readEngineFloats(exe, 0xa05240, 10);
    if (s.size() == 10 && s[3] > s[2] && s[2] > s[1] && s[1] > s[0] && s[0] > 0) {
        shadow.splits = {s[0], s[1], s[2], s[3]};
        shadow.zbias = {s[4], s[5], s[6], s[7]};
        shadow.zbiasAdd = s[8];
        shadow.zExtent = s[9];
        shadow.fromExe = true;
    }
}

}  // namespace fuel

namespace fuel {

F3 lighting(const Atmosphere& a, const F3& n, const F3& V, const F3& albedo, const F3& specCol, float power, float shadow, float occ) {
    // scattering_light.h lighting() with sun = gDSunColor, sky = lerp(DSKY_AMBIENT, DSKY_COLOR, occlusion), no omnis.
    auto dot = [](const F3& x, const F3& y) { return x[0] * y[0] + x[1] * y[1] + x[2] * y[2]; };
    const float nv = dot(n, V);
    const F3 r = {nv * n[0] * 2 - V[0], nv * n[1] * 2 - V[1], nv * n[2] * 2 - V[2]};
    const float s = std::pow(std::max(clamp01(dot(r, a.lightDir)), 1e-4f), power) * shadow;
    const float sun = clamp01(dot(n, a.lightDir)) * (shadow * (1 - occ * 0.1f) + occ * 0.1f);
    const float sky = std::sqrt(clamp01(dot(a.skyDir, n) * 0.5f + 0.5f)) * 0.9f + 0.1f;
    F3 out{};
    for (int i = 0; i < 3; ++i) {
        const size_t k = size_t(i);
        const float skyCol = a.dskyAmbient[k] + (a.dskyColor[k] - a.dskyAmbient[k]) * occ;
        out[k] = albedo[k] * (sky * skyCol + sun * a.sunColour[k]) + s * specCol[k] * a.sunColour[k];
    }
    return out;
}

}  // namespace fuel
