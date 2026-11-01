#import "renderer.h"

#include <chrono>

#include "core/genworld_noise.hpp"
#include "core/vehicle.hpp"
#include "core/weather.hpp"

#include <algorithm>
#include <filesystem>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace fuel {

namespace {

// Vertex layouts used by meshes (VertexND_Z, VertexGouraud_Z and the skinned
// Vertex1Blend_Z/Vertex4Blend_Z) all start the same way: float3 position at 0,
// a D3DCOLOR tangent at 12, a D3DCOLOR normal at 16 and float2 uv0 at 20.
// Only the stride differs.
constexpr uint32_t kMinDrawableStride = 28;
constexpr NSUInteger kShadowMapSize = 1024;  // slice size W (FUEL default; "-HS" doubles it)
constexpr float kVehicleDrawDistance = 2500.0f;  // metres from the focus (setVehicles)

const char* kShaderSource = R"METAL(
#include <metal_stdlib>
using namespace metal;

struct VertexIn {
    float3 position [[attribute(0)]];
    float4 tangent  [[attribute(1)]];  // D3DCOLOR read as BGRA: xyz direction, w handedness
    float4 normal   [[attribute(2)]];
    float2 uv       [[attribute(3)]];
    float2 corner   [[attribute(4)]];  // sprite batches: corner offset in metres (else uv1, unused)
};

struct FrameUniforms {
    float4x4 viewProj;
    float4x4 sunViewProj;
    float4x4 invViewProj;
    float4 sunDir;        // xyz towards the sun, w intensity
    float4 paint;         // linear rgb
    float4 eye;           // camera position
    float4 params;        // x dirt, y exposure, z ground height, w ground radius
    float4 footprint;     // xy scene centre (x, z), zw half extents
    float4 terrain;       // origin x, origin z, sample spacing, metres per texel value 1.0
    float4 terrainGrid;   // grid centre x, z, radius, vertices per side
    float4 fog;           // x haze density per metre (0 = none), y = 1 when ground maps are loaded,
                          // z near plane, w projection q = far / (near - far) (for view distances from depth)
    float4 camRight;      // camera axes in world space, for sprites
    float4 camUp;
    float4 groundFar;     // rgb: far-band forest colour (palette 57, linear); w = 1 when the colour map is
                          // FUEL's decoded point colour (alpha = forest weight), 0 for the old heuristic map
    float4 groundFine[3]; // fine ground tiles around the player (1, 2, 4 m): origin x, z, extent, w = 1 when loaded
    // ---- FUEL's lights and scattering (core/atmosphere.hpp; p26-renderer.md L1-L8)
    float4 lightDir;      // DLIGHT_DIR (towards the sun, lifted), w DLIGHT_INVINTENSITY
    float4 skyDir;        // DSKY_DIR
    float4 sunColour;     // gDSunColor = DLIGHT_COLOR * SC1.z
    float4 dlight;        // DLIGHT_COLOR (inscatter tint, sun disc), w blue shift strength (day 10, night 0.5)
    float4 skyColour;     // DSKY_COLOR
    float4 skyAmbient;    // DSKY_AMBIENT (0)
    float4 sunRaw;        // un-lifted direction of the sun / moon disc
    float4 scat[11];      // VS c6..c16: SC1, SC2, TERM, BETA_1, BETA_DASH_1, BETA_2, BETA_DASH_2, BETA_1_PLUS_2,
                          // ONE_OVER_BETA_1_PLUS_2, HG, SC3
    float4 shadowSplit;   // gShadowSliceDist: view z where slices 1..3 start, w = end of the shadows
    float4 shadowSize;    // TextureSize: (W, 1/W, 1/(4W), 4W) of the atlas, W = slice size
    float4x4 shadowMat[4];// view space -> (u, v in the atlas, light depth) per slice
    float4x4 view;        // world -> view space (FUEL's view: +z forward)
};

struct MaterialUniforms {
    float4 diffuse;
    float4 specular;      // rgb, power
    float4 emissive;
    float2 fresnel;       // reflectance at normal and grazing incidence
    uint alphaMode;       // 0 opaque, 1 alpha test, 2 blend
    uint lit;
    uint code;            // MatCodeId: 17 painted + two-state sheet, 19 painted, 20 two-state sheet
    uint features;
};

constant uint PaintOnly = 1, HasOcclusion = 2, HasNormal = 4, HasSpecular = 8, HasNormalLocal = 16, HasDirt = 32,
              Reflective = 64, GroundTint = 128, Foliage = 256;

struct VertexOut {
    float4 position [[position]];
    float3 world;
    float3 normal;
    float4 tangent;
    float2 uv;
    float3 ext;         // scattering at the vertex (scat_mesh.vhl scatteringbase)
    float3 insc;
    float3 m0, m1, m2;  // the instance's rotation (for object-space normal maps)
    float3 tint;        // local ground colour x2, for grey foliage and rock textures
    uint sprite [[flat]];
    float fade [[flat]];  // 1 - the object's fade alpha (Lod_Z / Mesh_Z ComputeFade, p36); 0 = opaque
};

// ---- FUEL's cascaded sun shadow (p26-work/shadows.md S1): 4 orthographic slices stacked in one
// W x 4W atlas, depth reversed (1 at the light camera; lit when the receiver depth >= the stored one).
// Port: the atlas is a 4-slice depth array (slice i = FUEL's atlas rows [iW, (i+1)W)), so the receiver
// matrices end at the slice's (u, v) instead of the atlas v = 0.25 v + 0.25 i; the taps are the same.
float shadowTap(depth2d_array<float> map, float2 uv, uint slice, float depth) {
    const int W = int(map.get_width());
    const int2 p = clamp(int2(floor(uv * float(W))), int2(0), int2(W - 1));
    return step(map.read(uint2(p), slice), depth);
}

// PixelShadow4: four bilinear PCF lookups over the 3x3 neighbourhood (a 2-texel tent).
float pcf3x3(depth2d_array<float> map, float3 tc, uint slice, float4 ts) {
    const float2 S = float2(ts.y, ts.z);
    const float2 fr = fract(tc.xy * float2(ts.x, ts.w));
    float s[3][3];
    for (int j = -1; j <= 1; ++j)
        for (int i = -1; i <= 1; ++i) s[j + 1][i + 1] = shadowTap(map, tc.xy + float2(i, j) * S, slice, tc.z);
    float sum = 0.0;
    for (int j = 0; j < 2; ++j)
        for (int i = 0; i < 2; ++i) {
            const float a0 = mix(s[j][i], s[j][i + 1], fr.x), a1 = mix(s[j + 1][i], s[j + 1][i + 1], fr.x);
            sum += mix(a0, a1, fr.y);
        }
    return 0.25 * sum;
}

// ProjShadowPS + ComputeShadow + ShadowAttenuation: 1 lit, 0 shadowed; fades to lit from 957.5 to 1600 m.
float fuelShadow(depth2d_array<float> map, float3 world, constant FrameUniforms& f) {
    if (f.shadowSize.x <= 0.0 || (uint(f.fog.x) & 1u)) return 1.0;
    const float3 pv = (f.view * float4(world, 1.0)).xyz;
    const float vz = pv.z;
    const float4 d = f.shadowSplit;
    const int i = int(vz >= d.x) + int(vz >= d.y) + int(vz >= d.z);
    float3 tc = (f.shadowMat[i] * float4(pv, 1.0)).xyz;
    tc.xy = saturate(tc.xy);
    const float s = pcf3x3(map, tc, uint(i), f.shadowSize);
    const float lo = d.z + (d.w - d.z) * 0.5;
    return saturate(s + smoothstep(lo, d.w, vz));
}

// ---- Atmospheric scattering (scattering.h; the fog of every lit shader). Index names as VS c6..c16.
constant int SC1 = 0, SC2 = 1, TERM = 2, BETA_DASH_1 = 4, BETA_DASH_2 = 6, BETA_SUM = 7, ONE_OVER = 8, HG = 9, SC3 = 10;

// scatterintegral: atmospheric density integral of a segment of length len between two heights.
float fuelScatterIntegral(constant FrameUniforms& f, float len, float y1, float y2) {
    const float4 sc1 = f.scat[SC1], sc2 = f.scat[SC2], term = f.scat[TERM];
    float4 r11 = float4(max(y1, y2), min(y1, y2), max(y1, y2), min(y1, y2));
    const float4 o = r11;
    r11.xy *= term.w;
    r11.zw *= sc2.w;
    float4 r10 = min(r11, 1.0);
    float4 d = float4(r11.x - r11.y, r11.z - r11.w, r10.x - r10.y, r10.z - r10.w);
    d = max(d, term.w);
    const float f1 = d.z / d.x, f2 = d.w / d.y;
    const float dens = (1.0 - (r10.x + r10.y) * 0.5) * f1 * sc2.y + (1.0 - (r10.z + r10.w) * 0.5) * f2 * sc2.z;
    float integral = dens * len;
    const float2 c = clamp(o.xy, sc1.y, term.z);
    integral += len * (c.x - c.y) / max(o.x - o.y, 1.0 / 16384.0) * f.scat[HG].w;
    return integral;
}

struct Scatter {
    float3 ext;
    float3 insc;
};

// scatteringfinish: extinction and inscatter for a density and the view/sun angle.
Scatter fuelScatterFinish(constant FrameUniforms& f, float lookY, float density, float cosT) {
    const float cc = min(0.0, cosT);
    const float phase1 = 1.0 + cc * cc;
    Scatter s;
    s.ext = exp2(-f.scat[BETA_SUM].xyz * f.scat[SC2].x * density);
    const float r = rsqrt(f.scat[HG].y + f.scat[HG].z * cosT);
    const float phase2 = f.scat[HG].x * (r * r * r * r);
    float ep = max(0.0, 1.0 + lookY);
    ep *= ep;
    ep *= ep;
    ep *= ep;
    ep = 1.0 - f.scat[SC3].w + ep * f.scat[SC3].w;
    float tw = 0.5 - cosT * 0.5;
    tw = saturate(tw * tw);
    s.insc = (f.scat[BETA_DASH_1].xyz * phase1 + f.scat[BETA_DASH_2].xyz * phase2) * f.scat[ONE_OVER].xyz;
    s.insc *= (1.0 - s.ext) * ep;
    s.insc *= (f.dlight.xyz * tw + (1.0 - tw)) * f.scat[TERM].y;
    s.ext *= sqrt(length(s.ext));
    return s;
}

// scatteringbase: what the camera sees of the air between it and the world point p.
Scatter fuelScattering(constant FrameUniforms& f, float3 p) {
    if (uint(f.fog.x) & 2u) return Scatter{float3(1.0), float3(0.0)};
    float3 look = f.eye.xyz - p;
    const float len = length(look);
    look /= max(len, 1e-6);
    const float cosT = dot(look, f.scat[SC3].xyz);
    const float density = fuelScatterIntegral(f, len, 0.25 + max(p.y, 0.0), f.eye.y);
    return fuelScatterFinish(f, min(look.y, 0.0), density, cosT);
}

// The sky seen along `dir` (vh_sky / ph_sky: scatteringbase to the sky dome vertex). The dome (SkyDome_Update
// 0x7e2690, tiles 0x7e1610 + 0x7e1340) is a sphere of radius 0.9 x the GenWorld camera's far plane (frustum +0x6c4,
// forced to 50000 at 0x77ad73) = 45 km, centred on the camera (frustum +0x6e0, re-placed every 5th frame), scale
// 1 - 0.75 sky+8 (0: no writer is called). FUEL evaluates it per dome vertex; the port per pixel.
constant float kSkyDomeRadius = 0.9 * 50000.0;
float3 fuelSky(constant FrameUniforms& f, float3 dir) {
    return fuelScattering(f, f.eye.xyz + dir * kSkyDomeRadius).insc;
}

// ---- Lighting (scattering_light.h)
float fuelSkylighting(float3 n, float3 K, thread float& skydot) {
    skydot = sqrt(saturate(dot(K, n) * 0.5 + 0.5));
    return skydot * 0.9 + 0.1;
}

float fuelSunlighting(float shadow, float occ) { return shadow * (1.0 - occ * 0.1) + occ * 0.1; }

// lighting(): vtx (sky + sun + omni) + spec * sunCol (with the shadow map: spec and sun shadowed).
float3 fuelLighting(float3 n, float3 L, float3 K, float3 V, float3 sunCol, float3 skyCol, float3 vtx, float3 specCol,
                    float power, float shadow, float3 omni, float occ) {
    const float3 r = dot(n, V) * n * 2.0 - V;
    float s = pow(max(saturate(dot(r, L)), 1e-4), power) * shadow;
    const float sun = saturate(dot(n, L)) * fuelSunlighting(shadow, occ);
    float skydot;
    const float sky = fuelSkylighting(n, K, skydot);
    return vtx * (sky * skyCol + sun * sunCol + omni) + s * specCol * sunCol;
}

// lightingmaterial(): with the material's diffuse and emissive.
float3 fuelLightingMaterial(float3 n, float3 L, float3 K, float3 V, float3 sunCol, float3 skyCol, float3 vtx, float3 specCol,
                            float power, float shadow, float3 omni, float occ, float4 diffuse, float3 emissive) {
    const float3 r = dot(n, V) * n * 2.0 - V;
    float s = pow(max(saturate(dot(r, L)), 1e-4), power) * shadow;
    const float sun = saturate(dot(n, L)) * fuelSunlighting(shadow, occ);
    float skydot;
    const float sky = fuelSkylighting(n, K, skydot);
    return vtx * (emissive * diffuse.a + diffuse.rgb * (sky * skyCol + sun * sunCol + omni)) + s * specCol * sunCol;
}

// cubelighting(): lighting() blended towards the reflection by the cube map influence.
float3 fuelCubeLighting(float3 n, float3 L, float3 K, float3 V, float3 sunCol, float3 skyCol, float3 vtx, float3 specCol,
                        float power, float3 cube, float cubePower, float shadow, float3 omni, float occ) {
    const float3 r = dot(n, V) * n * 2.0 - V;
    float s = pow(max(saturate(dot(r, L)), 1e-4), power) * shadow;
    const float sun = saturate(dot(n, L)) * fuelSunlighting(shadow, occ);
    float skydot;
    const float sky = fuelSkylighting(n, K, skydot);
    const float3 base = vtx * (sky * skyCol + sun * sunCol + omni);
    const float3 refl = skydot * (cube + s * specCol * sunCol);
    return mix(base, refl, cubePower);
}

// The world palette is a bright spring green; the port's old heuristic colour map (no decoded
// ground colours) is pulled towards olive. Not used with FUEL's decoded colours.
float3 groundGrade(float3 c) {
    float l = dot(c, float3(0.30, 0.59, 0.11));
    float green = saturate((c.g - max(c.r, c.b)) / max(c.g, 1e-3) * 2.0);
    return mix(float3(l), c, mix(0.75, 0.45, green)) * float3(0.92, 0.86, 0.76);
}

// Instanced: one world matrix per instance in buffer 2.
vertex VertexOut vsMain(VertexIn in [[stage_in]], constant FrameUniforms& f [[buffer(1)]],
                        const device float4x4* models [[buffer(2)]], constant uint& sprite [[buffer(3)]],
                        texture2d<float> groundColour [[texture(0)]], uint iid [[instance_id]]) {
    // Placed objects carry 1 - fade alpha in column 0's w (0 for every affine world matrix).
    float4x4 model = models[iid];
    VertexOut o;
    o.fade = model[0].w;
    model[0].w = 0.0;
    o.tint = float3(1.0);
    o.sprite = sprite;
    if (f.fog.y > 0.0) {
        // Like FUEL's ground, foliage and rocks take the palette colour under them
        // (ground x 2; the fragment shader decides how to apply it).
        constexpr sampler lin(coord::normalized, filter::linear, address::clamp_to_edge);
        const float3 gc = groundColour.sample(lin, (model[3].xz + 70000.0) / 140000.0, level(0)).rgb;
        o.tint = (f.groundFar.w > 0.0 ? gc : groundGrade(gc)) * 2.0;
    }
    o.m0 = model[0].xyz;
    o.m1 = model[1].xyz;
    o.m2 = model[2].xyz;
    float4 w = model * float4(in.position, 1.0);
    if (sprite != 0) {
        // Leaf and impostor sprites: all four corners share one position; expand
        // them towards the camera by the stored offsets (scaled with the instance).
        float scale = length(model[1].xyz);
        w.xyz += (f.camRight.xyz * in.corner.x + f.camUp.xyz * in.corner.y) * scale;
    }
    o.position = f.viewProj * w;
    o.world = w.xyz;
    o.normal = (model * float4(in.normal.xyz * 2.0 - 1.0, 0.0)).xyz;
    o.tangent = float4((model * float4(in.tangent.xyz * 2.0 - 1.0, 0.0)).xyz, in.tangent.w > 0.5 ? 1.0 : -1.0);
    o.uv = in.uv;
    const Scatter sc = fuelScattering(f, w.xyz);
    o.ext = sc.ext;
    o.insc = sc.insc;
    return o;
}

// ---- Shadow casters (shadow.phl ph_shadow, mesh_shadow.vhl vh_shadow): pushed away from the light by
// ADDBIAS / (|n.L|^2 + 1/4096) (ReAddShadowBiasNormal), depth z = 1 - zLight / ZExtent, cut-outs alpha-tested.
struct ShadowUniforms {
    float4x4 lightViewProj;
    float4 light;  // xyz towards the sun (DLIGHT_DIR), w ADDBIAS of the slice
    float4 eye;    // (terrain casters)
    float4 lightRight, lightUp;  // the light camera's axes: sprites face it (rotshape_shadow.vhl)
};

struct ShadowOut {
    float4 position [[position]];
    float2 uv;
};

vertex ShadowOut vsShadow(VertexIn in [[stage_in]], constant ShadowUniforms& su [[buffer(1)]],
                          const device float4x4* models [[buffer(2)]], constant uint& sprite [[buffer(3)]], uint iid [[instance_id]]) {
    float4x4 model = models[iid];
    model[0].w = 0.0;  // the fade slot (vsMain)
    float4 w = model * float4(in.position, 1.0);
    if (sprite != 0) {
        // Billboards face the light camera and move by the constant ReAddShadowBias.
        w.xyz += (su.lightRight.xyz * in.corner.x + su.lightUp.xyz * in.corner.y) * length(model[1].xyz);
        w.xyz -= su.light.xyz * su.light.w;
    } else {
        float3 n = (model * float4(in.normal.xyz * 2.0 - 1.0, 0.0)).xyz;
        n = dot(n, n) > 1e-8 ? normalize(n) : float3(0, 1, 0);
        const float nl = dot(n, su.light.xyz);
        w.xyz -= su.light.xyz * (su.light.w / (nl * nl + 1.0 / 4096.0));
    }
    ShadowOut o;
    o.position = su.lightViewProj * w;
    o.uv = in.uv;
    return o;
}

fragment void fsShadowCut(ShadowOut in [[stage_in]], texture2d<float> tDiffuse [[texture(0)]], sampler smp [[sampler(0)]],
                          constant uint& halfAtlas [[buffer(0)]]) {
    const float2 uv = halfAtlas != 0 ? float2(in.uv.x * 0.5, in.uv.y) : in.uv;
    if (tDiffuse.sample(smp, uv).a - 0.5 < 0.0) discard_fragment();
}

// ---- Objects, vehicles, rider (scat_mesh.phl ph_base) and leaf / impostor sprites (scat_rotshape).
fragment float4 fsMain(VertexOut in [[stage_in]], bool front [[front_facing]],
                       constant FrameUniforms& f [[buffer(1)]],
                       constant MaterialUniforms& m [[buffer(2)]],
                       texture2d<float> tDiffuse [[texture(0)]],
                       texture2d<float> tOcclusion [[texture(2)]],
                       texture2d<float> tNormal [[texture(3)]],
                       texture2d<float> tSpecular [[texture(4)]],
                       texture2d<float> tNormalLocal [[texture(5)]],
                       texture2d<float> tDirt [[texture(6)]],
                       depth2d_array<float> shadowMap [[texture(7)]],
                       sampler smp [[sampler(0)]]) {
    // Codes 17 and 20 use sheets whose left half is the intact part and the
    // right half the damaged one; draw the intact half (bBroken, break amount 0).
    float2 uvHalf = (m.code == 17 || m.code == 20) ? float2(in.uv.x * 0.5, in.uv.y) : in.uv;
    if (in.fade > 0.0) {
        // Port: FUEL fades objects over the last 20 m of their view distance (view+0x160 alpha); the
        // port dissolves them with a 4x4 ordered dither instead of sorting blended objects.
        const uint2 q = uint2(in.position.xy) & 3u;
        const float bayer[16] = {0, 8, 2, 10, 12, 4, 14, 6, 3, 11, 1, 9, 15, 7, 13, 5};
        if ((bayer[q.y * 4 + q.x] + 0.5) / 16.0 < in.fade) discard_fragment();
    }
    float4 d = tDiffuse.sample(smp, uvHalf);
    if (in.sprite != 0) {
        // Port: leaves take the ground's hue with only mild darkening (some leaf textures are dark).
        const float lum = max(dot(in.tint, float3(0.3, 0.55, 0.15)), 1e-3);
        d.rgb *= in.tint / lum * clamp(lum, 0.6, 1.0);
    }
    else if (m.features & GroundTint)  // port: rock, a muted version of the ground colour
        d.rgb *= mix(float3(dot(in.tint, float3(0.3, 0.55, 0.15))), in.tint, 0.35);
    if (m.code == 17 || m.code == 19) {
        // bColoured: lerp(gGouraudDefault (the paint), tex, tex.a).
        d.rgb = mix(f.paint.rgb, d.rgb, (m.features & PaintOnly) ? 0.0 : d.a);
        d.a = 1.0;
    }
    float dirtiness = 0.0;
    if (m.features & HasDirt) {
        float4 dirt = tDirt.sample(smp, in.uv);
        float edge = 1.0 - f.params.x;
        dirtiness = smoothstep(edge, edge + 0.15, dirt.a);
        d.rgb = mix(d.rgb, dirt.rgb, dirtiness);
    }
    float4 c = d * float4(1.0, 1.0, 1.0, m.diffuse.a);
    float alpha = m.alphaMode == 2 ? c.a : 1.0;
    if (m.alphaMode == 1) {
        // Cut-outs: the 0.5 edge sharpened to about a pixel; with MSAA the alpha
        // becomes coverage (alpha to coverage), so edges stay soft and depth sorts them.
        alpha = saturate((c.a - 0.5) / max(fwidth(c.a), 1e-4) + 0.5);
        // Leaves at the lens would fill the screen: thin them out within 1.5..4 m.
        if (m.features & Foliage) alpha *= saturate((distance(in.world, f.eye.xyz) - 1.5) / 2.5);
        if (alpha <= 0.0) discard_fragment();
    }
    if (m.lit == 0) return float4(c.rgb * m.diffuse.rgb + m.emissive.rgb, alpha);

    const float shadow = fuelShadow(shadowMap, in.world, f);
    const float3 sunCol = f.sunColour.rgb, L = f.lightDir.xyz, K = f.skyDir.xyz;
    if (in.sprite != 0) {
        // scat_rotshape: normal up; sky = DSKY_COLOR + DSKY_AMBIENT (K.y/2 + 1/2), sunlit = sky + sun sat(L.y),
        // both x MTL_DIFFUSE x GOURAUD x extinction; lerp(sky, sunlit, sunlighting(shadow, 1)) + emissive.
        const float3 sky = f.skyColour.rgb + f.skyAmbient.rgb * (K.y * 0.5 + 0.5);
        const float3 sunlit = sky + sunCol * saturate(L.y);
        const float3 vcol = m.diffuse.rgb * in.ext;
        const float3 emis = m.emissive.rgb * f.lightDir.w * in.ext;
        const float3 rgb = c.rgb * (mix(sky, sunlit, fuelSunlighting(shadow, 1.0)) * vcol + emis) + in.insc;
        return float4(rgb, alpha);
    }

    // Some foliage stores no normal at all: fall back to up.
    const float3 n0 = dot(in.normal, in.normal) > 1e-8 ? normalize(in.normal) : float3(0, 1, 0);
    float3 N = n0 * (front ? 1.0 : -1.0);
    if (m.features & HasNormalLocal) {
        // bNormalLocal: object-space normal map (compressed: .xyw).
        const float3 nl = tNormalLocal.sample(smp, uvHalf).xyw * 2.0 - 1.0;
        N = normalize(in.m0 * nl.x + in.m1 * nl.y + in.m2 * nl.z);
    } else if (m.features & HasNormal) {
        // bNormal: tangent-space normal map, x in alpha and y in green, z rebuilt.
        float2 xy = tNormal.sample(smp, uvHalf).wy * 2.0 - 1.0;
        float3 T = in.tangent.xyz - N * dot(N, in.tangent.xyz);
        T = dot(T, T) > 1e-8 ? normalize(T) : normalize(cross(N, abs(N.y) < 0.9 ? float3(0, 1, 0) : float3(1, 0, 0)));
        float3 B = cross(N, T) * in.tangent.w;
        N = normalize(T * xy.x + B * xy.y + N * sqrt(saturate(1.0 - dot(xy, xy))));
    }
    // occlusion = sOcclusion x gDLight.Ambient.w (1); skylight = lerp(DSKY_AMBIENT, DSKY_COLOR, occlusion).
    const float occ = (m.features & HasOcclusion) ? tOcclusion.sample(smp, in.uv).r : 1.0;
    const float3 skyCol = mix(f.skyAmbient.rgb, f.skyColour.rgb, occ);
    float4 spec = m.specular * ((m.features & HasSpecular) ? tSpecular.sample(smp, in.uv) : float4(1.0));
    if (m.features & HasOcclusion) spec.rgb *= occ;
    spec.rgb *= shadow * 0.5 + 0.5;
    spec.rgb *= 1.0 - dirtiness;
    const float3 V = normalize(f.eye.xyz - in.world);
    float3 rgb;
    if (m.features & Reflective) {
        // bCubemap: Fresnel(FallOff) x 2 (r + g + b) of the specular, x (0.33 shadow + 0.67). The cube is
        // sGlobalCube in FUEL; the port reflects its sky (approximation).
        const float fres = mix(m.fresnel.x, m.fresnel.y, pow(1.0 - saturate(dot(V, N)), 5.0));
        float infl = saturate(fres * dot(spec.rgb, float3(2.0)));
        infl *= shadow * 0.33 + 0.67;
        const float3 r = reflect(-V, N);
        const float3 cube = fuelSky(f, float3(r.x, max(r.y, 0.02), r.z));
        rgb = fuelCubeLighting(N, L, K, V, sunCol, skyCol, c.rgb, spec.rgb, spec.a, cube, infl, shadow, float3(0), occ);
    } else {
        rgb = fuelLightingMaterial(N, L, K, V, sunCol, skyCol, c.rgb, spec.rgb, spec.a, shadow, float3(0), occ, m.diffuse,
                                   m.emissive.rgb);
    }
    return float4(rgb * in.ext + in.insc, alpha);
}

struct GroundOut {
    float4 position [[position]];
    float3 world;
};

vertex GroundOut vsGround(uint vid [[vertex_id]], constant FrameUniforms& f [[buffer(1)]]) {
    const float2 corners[4] = {float2(-1, -1), float2(1, -1), float2(-1, 1), float2(1, 1)};
    float3 w = float3(f.footprint.x + corners[vid].x * f.params.w, f.params.z, f.footprint.y + corners[vid].y * f.params.w);
    GroundOut o;
    o.position = f.viewProj * float4(w, 1.0);
    o.world = w;
    return o;
}

fragment float4 fsGround(GroundOut in [[stage_in]], constant FrameUniforms& f [[buffer(1)]],
                         depth2d_array<float> shadowMap [[texture(7)]]) {
    float2 rel = in.world.xz - f.footprint.xy;
    // Soft contact darkening under the vehicle's footprint.
    float ao = mix(0.45, 1.0, smoothstep(0.5, 1.4, length(rel / max(f.footprint.zw, 0.05))));
    const float shadow = fuelShadow(shadowMap, in.world, f);
    const float3 V = normalize(f.eye.xyz - in.world);
    const float3 rgb = fuelLighting(float3(0, 1, 0), f.lightDir.xyz, f.skyDir.xyz, V, f.sunColour.rgb, f.skyColour.rgb * ao,
                                    float3(0.55, 0.52, 0.48), float3(0), 1.0, shadow, float3(0), ao);
    float fade = smoothstep(1.0, 0.55, length(rel) / f.params.w);
    return float4(rgb, fade);
}

constexpr sampler heightSampler(coord::normalized, filter::linear, address::clamp_to_edge);

// Heights are 12-bit (2.4 m steps) on a 30 m grid; like the original engine,
// smooth them with a cubic B-spline (four bilinear taps).
float terrainHeight(texture2d<float> hm, float2 xz, constant FrameUniforms& f) {
    float2 size = float2(hm.get_width(), hm.get_height());
    float2 t = (xz - f.terrain.xy) / f.terrain.z;
    float2 i = floor(t), a = t - i;
    float2 w0 = (1.0 - a) * (1.0 - a) * (1.0 - a) / 6.0;
    float2 w1 = (4.0 - 6.0 * a * a + 3.0 * a * a * a) / 6.0;
    float2 w3 = a * a * a / 6.0;
    float2 w2 = 1.0 - w0 - w1 - w3;
    float2 g0 = w0 + w1, g1 = w2 + w3;
    float2 p0 = (i - 1.0 + w1 / g0 + 0.5) / size, p1 = (i + 1.0 + w3 / g1 + 0.5) / size;
    float h = g0.y * (g0.x * hm.sample(heightSampler, float2(p0.x, p0.y), level(0)).r +
                      g1.x * hm.sample(heightSampler, float2(p1.x, p0.y), level(0)).r) +
              g1.y * (g0.x * hm.sample(heightSampler, float2(p0.x, p1.y), level(0)).r +
                      g1.x * hm.sample(heightSampler, float2(p1.x, p1.y), level(0)).r);
    return h * f.terrain.w;
}

float hash12(float2 p) {
    float3 p3 = fract(float3(p.xyx) * 0.1031);
    p3 += dot(p3, p3.yzx + 33.33);
    return fract((p3.x + p3.y) * p3.z);
}

float valueNoise(float2 p) {
    float2 i = floor(p), u = fract(p);
    u = u * u * (3.0 - 2.0 * u);
    return mix(mix(hash12(i), hash12(i + float2(1, 0)), u.x), mix(hash12(i + float2(0, 1)), hash12(i + float2(1, 1)), u.x), u.y);
}

// Size of the leaf chunk of FUEL's camera-centred quadtree holding xz: 12288 m
// roots on a 12288 m lattice, split into four while the camera is within 2.0625 x
// the chunk size of its centre, down to 24 m (FUN_007dfdc0; drive.mm has the same).
float groundChunk(float2 xz, float2 eye) {
    float size = 12288.0;
    float2 o = round(xz / size) * size;
    for (int k = 0; k < 9 && size > 24.01; ++k) {
        if (distance(o, eye) >= size * 2.0625) break;
        o += select(float2(-0.25), float2(0.25), xz >= o) * size;
        size *= 0.5;
    }
    return size;
}

struct TerrainOut {
    float4 position [[position]];
    float3 world;
    float3 ext;      // ground.vhl scatteringbase at the vertex
    float3 insc;
    float3 normal;   // FUEL's ground normal at the vertex (central differences of the chunk's points)
    float chunk;     // size of the ground chunk the vertex belongs to
};

// FUEL's ground chunks (terrain_mesh.hpp, TerrainVertex): 25 x 25 points per quadtree
// chunk, morphed towards the half-resolution grid between 2.375 and 2.875 chunk sizes
// from the camera (horizontal distance; Paysage_BuildVertices 0x7f9b50, ground.vhl).
struct TerrainVertexIn {
    packed_float3 p0;
    float tweenStart;
    packed_float3 p1;
    float tweenInv;
    packed_float3 n;
    float size;
    packed_float3 n1;  // morph target normal
    float pad;
};

vertex TerrainOut vsTerrain(uint vid [[vertex_id]], constant FrameUniforms& f [[buffer(1)]],
                            const device TerrainVertexIn* verts [[buffer(0)]]) {
    const TerrainVertexIn v = verts[vid];
    const float3 p0 = float3(v.p0), p1 = float3(v.p1);
    const float t = clamp((distance(f.eye.xz, p0.xz) - v.tweenStart) * v.tweenInv, 0.0, 1.0);
    const float3 w = mix(p0, p1, t);
    TerrainOut o;
    o.position = f.viewProj * float4(w, 1.0);
    o.world = w;
    const Scatter sc = fuelScattering(f, w);
    o.ext = sc.ext;
    o.insc = sc.insc;
    o.normal = normalize(mix(float3(v.n), float3(v.n1), t));  // ground.vhl lerps the normal with the position
    o.chunk = v.size;
    return o;
}

// The 60 m colour map refined by the fine tiles around the player (4 m, 2 m, then 1 m texels),
// each fading out over the outer eighth of its square. FUEL colours its terrain per
// vertex: 1 m apart in the 24 m chunks near the camera, coarser further out.
float4 groundColourAt(float2 xz, float4 gc, texture2d_array<float> fine, constant FrameUniforms& f) {
    constexpr sampler lin(coord::normalized, filter::linear, mip_filter::linear, address::clamp_to_edge);
    for (int l = 2; l >= 0; --l) {
        const float4 t = f.groundFine[l];
        if (t.w <= 0.0) continue;
        const float2 u = (xz - t.xy) / t.z;
        const float edge = min(min(u.x, 1.0 - u.x), min(u.y, 1.0 - u.y));
        const float w = saturate(edge * 8.0);
        if (w > 0.0) gc = mix(gc, fine.sample(lin, u, uint(l)), w);
    }
    return gc;
}

// The ground's sky occlusion (ground.vhl SetOcclusion: the vertex occlusion byte / 256) from the fine tiles'
// alpha; 1 outside them.
float groundOcclusionAt(float2 xz, texture2d_array<float> fine, constant FrameUniforms& f) {
    constexpr sampler lin(coord::normalized, filter::linear, mip_filter::linear, address::clamp_to_edge);
    float occ = 1.0;
    for (int l = 2; l >= 0; --l) {
        const float4 t = f.groundFine[l];
        if (t.w <= 0.0) continue;
        const float2 u = (xz - t.xy) / t.z;
        const float edge = min(min(u.x, 1.0 - u.x), min(u.y, 1.0 - u.y));
        const float w = saturate(edge * 8.0);
        if (w > 0.0) occ = mix(occ, fine.sample(lin, u, uint(l)).a, w);
    }
    return occ;
}

// ---- ground.phl / ground.vhl and the ground vertices of Paysage_BuildVertices (FUEL.exe
// 0x7f9b50), decoded in ~/re-work/fuel/notes/p22-ground-roads-water.md (G1, G2, R4). The CPU
// tiles (setGroundTile) hold FUEL's per-vertex inputs on its vertex lattice; the shader
// rebuilds each lattice triangle the way the engine builds its draw sets.
struct GroundVert {
    ushort4 tw;  // surface type | weight byte << 8
    float4 dc;   // decal u, v, weight, slot (-1 none)
};

bool groundVertex(texture2d_array<ushort> types, texture2d_array<float> decal, float4 rect, uint slice, float sp, float2 wp,
                  thread GroundVert& v) {
    if (rect.w <= 0.0) return false;
    const int2 it = int2(floor((wp - rect.xy) / sp - 0.5 + 0.5));
    if (any(it < 0) || any(it >= int2(256))) return false;
    v.tw = types.read(uint2(it), slice);
    v.dc = decal.read(uint2(it), slice);
    return true;
}

uint typeWeight(ushort4 tw, uint k) {
    for (int c = 0; c < 4; ++c)
        if (uint(tw[c] & 0xff) == k) return uint(tw[c] >> 8);
    return 0;
}

// The lattice triangle holding xz (lattice spacing L): FUEL splits each quad along the
// (x+1, z)-(x, z+1) diagonal; vertices in the order the decal material is picked.
void groundTriangle(float2 xz, float L, thread float2* p, thread float3& bary) {
    const float2 g = xz / L, i0 = floor(g), fr = g - i0;
    if (fr.x + fr.y <= 1.0) {
        p[0] = i0, p[1] = i0 + float2(0, 1), p[2] = i0 + float2(1, 0);
        bary = float3(1.0 - fr.x - fr.y, fr.y, fr.x);
    } else {
        p[0] = i0 + float2(1, 0), p[1] = i0 + float2(0, 1), p[2] = i0 + float2(1, 1);
        bary = float3(1.0 - fr.y, 1.0 - fr.x, fr.x + fr.y - 1.0);
    }
    for (int i = 0; i < 3; ++i) p[i] *= L;
}

// One band of ground.phl: bump normal + grey detail (n.xyz, n.w) and the decal (rgb, blend).
struct GroundBand {
    float4 n;
    float4 decal;
    bool ok;
};

GroundBand groundBand(float C, float2 xz, float2 warp, float steep, float sandy, texture2d_array<ushort> types,
                      texture2d_array<float> decalTex, texture2d_array<float> detail, sampler rep, constant FrameUniforms& f,
                      texture2d<float> d0, texture2d<float> d1, texture2d<float> d2, texture2d<float> d3, texture2d<float> d4,
                      texture2d<float> d5) {
    GroundBand out;
    out.ok = true;
    out.decal = float4(0.0);
    // Lattice and tile slice of the chunk's vertices: C/24 apart (1, 2, 4, 8 m).
    const float L = C / 24.0;
    const uint slice = C <= 24.5 ? 0u : (C <= 48.5 ? 1u : 2u);
    const float sp = slice == 0u ? 1.0 : (slice == 1u ? 2.0 : 4.0);
    float2 p[3];
    float3 bary;
    groundTriangle(xz, L, p, bary);
    GroundVert v[3];
    bool have = C <= 192.5;
    for (int i = 0; i < 3 && have; ++i) have = groundVertex(types, decalTex, f.groundFine[slice], slice, sp, p[i], v[i]);
    if (C <= 48.5) {
        if (!have) {
            out.ok = false;
            return out;
        }
        // Draw set of the triangle (0x7f9b50 near loop): per type the largest vertex weight; the
        // three largest (ties keep the lower type) if above the threshold, sorted by type.
        const uint thr = C <= 30.0 ? 32u : 64u;
        uint m0 = 0, m1 = 0, m2 = 0, k0 = 0, k1 = 0, k2 = 0, last = 0;
        bool first = true;
        for (int it = 0; it < 12; ++it) {
            uint next = 0x100;
            for (int q = 0; q < 12; ++q) {
                const uint k = uint(v[q / 4].tw[q % 4] & 0xff);
                if (k != 0xff && (first || k > last) && k < next) next = k;
            }
            if (next == 0x100) break;
            first = false;
            last = next;
            const uint m = max(typeWeight(v[0].tw, next), max(typeWeight(v[1].tw, next), typeWeight(v[2].tw, next)));
            if (m2 < m) {
                if (m1 < m) {
                    m2 = m1, k2 = k1, m1 = m, k1 = next;
                    if (m0 < m) m1 = m0, k1 = k0, m0 = m, k0 = next;
                } else {
                    m2 = m, k2 = next;
                }
            }
        }
        if (m2 > thr) {
            if (k1 < k0) { const uint t = k0; k0 = k1; k1 = t; }
            if (k2 < k1) { const uint t = k1; k1 = k2; k2 = t; }
            if (k1 < k0) { const uint t = k0; k0 = k1; k1 = t; }
        } else {
            k2 = 0xff;
            if (m1 > thr) {
                if (k1 < k0) { const uint t = k0; k0 = k1; k1 = t; }
            } else {
                k1 = 0xff;
            }
        }
        // Per-vertex lerp weights (0x7f8be0), interpolated like color0.w / color1.w.
        float3 b1 = 0.0, b2 = 0.0;
        for (int i = 0; i < 3; ++i) {
            const float w0 = float(typeWeight(v[i].tw, k0)) / 255.0;
            const float a = k1 != 0xff ? float(typeWeight(v[i].tw, k1)) / 255.0 : 0.0;
            const float b = k2 != 0xff ? float(typeWeight(v[i].tw, k2)) / 255.0 : 0.0;
            const float sum = w0 + a;
            b1[i] = a > 0.0 ? floor((sum > 0.0 ? a / sum : 0.0) * 255.0) / 255.0 : 0.0;
            b2[i] = b > 0.0 ? floor(saturate(max(b, 1.0 - sum)) * 255.0) / 255.0 : 0.0;
        }
        const float B1 = dot(b1, bary), B2 = dot(b2, bary);
        const float2 uv = warp * 0.2;  // W+0x4832c = 0.2 repeats per metre
        float4 t = detail.sample(rep, uv, k0);
        if (k1 != 0xff) t = mix(t, detail.sample(rep, uv, k1), B1);
        if (k2 != 0xff) t = mix(t, detail.sample(rep, uv, k2), B2);
        out.n = t;
    } else {
        // Chunks of 96 m and more use one fixed set for every mid/far band (0x7f7ea0 / 0x7f8390 /
        // 0x7f8850 store bump slots 0x11, 0x14, 0x15 at 0x7f820e / 0x7f86f0 / 0x7f8af1; p32):
        // T0 = 17 STEPPENL, T1 = 20 ROCKNL (steepness w1), T2 = 21 PLAINNL (sand / snow share w2).
        const float scale = C < 383.0 ? 0.015 : 0.0015, extra = C < 383.0 ? 0.75 : 0.0;
        const float2 uvF = warp * scale;
        const float4 t0 = detail.sample(rep, uvF, 17), t1 = detail.sample(rep, uvF, 20);
        const float4 t2 = detail.sample(rep, uvF, 21);
        out.n = mix(mix(t0, t1, steep), t2, saturate(saturate(sandy + extra) - steep));
    }
    // Decal (ground.phl !bNoDecal): material of the first vertex that has one; uv and
    // weight interpolated across the triangle. Chunks up to 192 m carry decals.
    if (have) {
        int slot = -1;
        for (int i = 0; i < 3; ++i)
            if (slot < 0 && v[i].dc.w >= 0.0) slot = int(v[i].dc.w + 0.5);
        const float w = dot(float3(v[0].dc.z, v[1].dc.z, v[2].dc.z), bary);
        if (slot >= 0 && w > 0.0) {
            const float2 duv = v[0].dc.xy * bary.x + v[1].dc.xy * bary.y + v[2].dc.xy * bary.z;
            float4 td;
            switch (slot) {
                case 0: td = d0.sample(rep, duv); break;
                case 1: td = d1.sample(rep, duv); break;
                case 2: td = d2.sample(rep, duv); break;
                case 3: td = d3.sample(rep, duv); break;
                case 4: td = d4.sample(rep, duv); break;
                default: td = d5.sample(rep, duv); break;
            }
            out.decal = float4(td.rgb, td.a * w);
        }
    }
    return out;
}

fragment float4 fsTerrain(TerrainOut in [[stage_in]], constant FrameUniforms& f [[buffer(1)]],
                          texture2d<float> hm [[texture(0)]], texture2d<float> groundColour [[texture(1)]],
                          texture2d<uint> groundSurface [[texture(2)]], texture2d_array<float> detail [[texture(3)]],
                          texture2d_array<float> groundFine [[texture(4)]], texture2d_array<ushort> groundTypes [[texture(5)]],
                          texture2d_array<float> groundDecal [[texture(6)]], depth2d_array<float> shadowMap [[texture(7)]],
                          texture2d<float> decal0 [[texture(8)]], texture2d<float> decal1 [[texture(9)]],
                          texture2d<float> decal2 [[texture(10)]], texture2d<float> decal3 [[texture(11)]],
                          texture2d<float> decal4 [[texture(12)]], texture2d<float> decal5 [[texture(13)]]) {
    float e = f.terrain.z;
    float2 xz = in.world.xz;
    (void)e;
    // FUEL's ground normal from the chunk's points (P21's ground mesh), interpolated.
    float3 N = normalize(in.normal);
    float slope = 1.0 - N.y;
    float n = valueNoise(xz * 0.004) * 0.6 + valueNoise(xz * 0.03) * 0.3 + valueNoise(xz * 0.25) * 0.1;
    float3 c;
    float occ = 1.0;
    if (f.fog.y > 0.0) {
        constexpr sampler lin(coord::normalized, filter::linear, mip_filter::linear, address::clamp_to_edge);
        constexpr sampler rep(coord::normalized, filter::linear, mip_filter::linear, address::repeat, max_anisotropy(8));
        float2 uv = (xz + 70000.0) / 140000.0;
        const float4 map60 = groundColour.sample(lin, uv);
        const float4 gc = groundColourAt(xz, map60, groundFine, f);
        occ = groundOcclusionAt(xz, groundFine, f);
        // Decoded map (p16): the point colour ctx+0x70 as is; the heuristic map gets the old grade.
        c = f.groundFar.w > 0.0 ? gc.rgb : groundGrade(gc.rgb);
        // Bump uv: the world position warped by two sines (0x7f9b50).
        const float2 warp = float2(xz.x + 5.0 * sin(0.02 * xz.y) + 2.0 * sin(0.2 * xz.y),
                                   xz.y + 5.0 * sin(0.02 * xz.x) + 2.0 * sin(0.2 * xz.x));
        // Sand / snow / cracked share of the 60 m surface map (FUN_007e4950 input of the far sets).
        float size = float(groundSurface.get_width());
        float2 t = uv * size - 0.5;
        float2 i0 = floor(t), a = t - i0;
        float sandy = 0.0;
        uint nearest = 0;
        float wbest = -1.0;
        for (int q = 0; q < 4; ++q) {
            uint2 cell = uint2(clamp(i0 + float2(q & 1, q >> 1), 0.0, size - 1.0));
            float w = ((q & 1) ? a.x : 1.0 - a.x) * ((q >> 1) ? a.y : 1.0 - a.y);
            const uint layer = groundSurface.read(cell).r;
            if (w > wbest) wbest = w, nearest = layer;
            sandy += w * float((layer >= 6 && layer <= 9) || layer == 24 || layer >= 36);
        }
        // Steepness weight of the far sets: a = clamp(2 n.y - 1), s = 2a - 1, w1 = 1 - S2(s).
        const float sa = 2.0 * saturate(2.0 * N.y - 1.0) - 1.0;
        const float steep = 1.0 - ((2.0 - abs(sa)) * sa * 0.5 + 0.5);
        // The chunk under the point and FUEL's geomorph tween toward its parent chunk
        // (ground.vhl: (d - 2.375 C) * 2 / C, distance in xz); the port blends the parent's
        // textures with the same weight (the engine switches them per chunk).
        const float dist = length(f.eye.xz - xz);
        const float C = in.chunk > 0.0 ? exp2(round(log2(in.chunk))) : groundChunk(xz, f.eye.xz);  // the ground chunk drawing this pixel
        const float tw = saturate((dist - 2.375 * C) * 2.0 / C);
        GroundBand b = groundBand(C, xz, warp, steep, sandy, groundTypes, groundDecal, detail, rep, f, decal0, decal1, decal2, decal3,
                                  decal4, decal5);
        if (!b.ok) {  // no tile yet: the 60 m surface map's nearest type
            b.n = detail.sample(rep, warp * 0.2, nearest);
            b.ok = true;
        }
        if (tw > 0.0) {
            GroundBand pb = groundBand(2.0 * C, xz, warp, steep, sandy, groundTypes, groundDecal, detail, rep, f, decal0, decal1, decal2,
                                       decal3, decal4, decal5);
            b.n = mix(b.n, pb.n, tw);
            b.decal = mix(b.decal, pb.decal, tw);
        }
        // Vertex colour by band (0x7f9b50): chunks >= 600 m (the 768 m leaves, from about
        // 1.6 km) use ctx+0x90, the colour with the forest floor (palette 57) by weight k (map
        // alpha); nearer ones ctx+0x70/0x80 (equal on land). Faded by distance.
        if (f.groundFar.w > 0.0) c = mix(c, f.groundFar.rgb, map60.a * smoothstep(1400.0, 1800.0, dist));
        // ground.phl: albedo = lerp(vcol * tN.w * 2, tDiffuse.rgb, tDiffuse.a * decal weight), all in gamma
        // space like the game (textures and vertex colours are read without sRGB decoding).
        c = mix(c * b.n.b * 2.0, b.decal.rgb, saturate(b.decal.a));
        float3 nt = normalize(float3(b.n.r, b.n.g, b.n.a) * 2.0 - 1.0);
        // ground.vhl tangent frame: x = cross(z, N), y = N, z = cross(x, N).
        float3 T = normalize(cross(float3(0, 0, 1), N)), B = normalize(cross(T, N));
        N = normalize(T * nt.x + N * nt.y + B * nt.z);
    } else {
        // Procedural fallback: grass and dry ground by altitude, rock on steep slopes, snow up high.
        float3 grass = float3(0.035, 0.060, 0.018), dry = float3(0.11, 0.085, 0.045);
        float3 rock = float3(0.10, 0.092, 0.085), snow = float3(0.80, 0.82, 0.86);
        c = mix(grass, dry, saturate(n * 0.9 + (in.world.y - 1500.0) / 1800.0));
        c = mix(c, rock, smoothstep(0.16, 0.32, slope + (n - 0.5) * 0.08));
        c = mix(c, snow, smoothstep(3100.0, 3500.0, in.world.y + n * 300.0) * (1.0 - smoothstep(0.35, 0.6, slope)));
    }
    // ground.phl lighting(): sun / sky in the bump frame, skylight = lerp(DSKY_AMBIENT, DSKY_COLOR, occlusion),
    // specular power 15 with the per-vertex specular (port: 0, not carried by the tiles), no omnis, then
    // ApplyScattering with the vertex's extinction / inscatter.
    const float shadow = fuelShadow(shadowMap, in.world, f);
    const float3 V = normalize(f.eye.xyz - in.world);
    const float3 rgb = fuelLighting(N, f.lightDir.xyz, f.skyDir.xyz, V, f.sunColour.rgb, mix(f.skyAmbient.rgb, f.skyColour.rgb, occ), c,
                                    float3(0.0), 15.0, shadow, float3(0), occ);
    return float4(rgb * in.ext + in.insc, 1.0);
}


// ground.vhl vh_shadow: the terrain chunk with the colour pass's geomorph, pushed away from the light
// along its normal (ReAddShadowBiasNormal); no alpha test.
struct TerrainShadowUniforms {
    float4x4 lightViewProj;
    float4 light;  // xyz towards the sun, w ADDBIAS
    float4 eye;    // camera position (the geomorph distance)
};

vertex float4 vsTerrainShadow(uint vid [[vertex_id]], constant TerrainShadowUniforms& su [[buffer(1)]],
                              const device TerrainVertexIn* verts [[buffer(0)]]) {
    const TerrainVertexIn v = verts[vid];
    const float3 p0 = float3(v.p0), p1 = float3(v.p1);
    const float t = clamp((distance(su.eye.xz, p0.xz) - v.tweenStart) * v.tweenInv, 0.0, 1.0);
    float3 w = mix(p0, p1, t);
    const float3 n = normalize(float3(v.n));
    const float nl = dot(n, su.light.xyz);
    w -= su.light.xyz * (su.light.w / (nl * nl + 1.0 / 4096.0));
    return su.lightViewProj * float4(w, 1.0);
}

// ---- Lake water: ground_water.vhl / .phl (vh_water_anim, ph_water), notes p22 / water.md 3.3.
// Port: no scene-colour copy or depth texture; the refraction lerps are done by blending
// (refraction unperturbed) and the water depth comes from the vertices.
struct WaterVertex {
    packed_float3 position;
    uchar4 colour;  // B, G, R, A bytes
    float2 uv;
    float depth;
    float kind;  // 0 lake (category 0xe), 1 river (0xf)
};

struct WaterOut {
    float4 position [[position]];
    float3 world;
    float4 colour;
    float4 uv;
    float depth;
    float3 ext;   // scattering() at the vertex (ground_water.vhl)
    float3 insc;
    float kind;
};

vertex WaterOut vsWater(uint vid [[vertex_id]], const device WaterVertex* v [[buffer(0)]], constant FrameUniforms& f [[buffer(1)]],
                        constant float4* W [[buffer(2)]], constant float& t [[buffer(3)]]) {
    WaterOut o;
    float3 w = float3(v[vid].position);
    const bool river = v[vid].kind > 0.5;
    // Wave (vh_water_anim, lakes): y -= W3.y (sin(-x W3.x + W3.z t) 0.5 + 0.5). Rivers
    // (vh_water_noanim) have none.
    if (!river) w.y -= W[3].y * (sin(-w.x * W[3].x + W[3].z * t) * 0.5 + 0.5);
    o.kind = river ? 1.0 : 0.0;
    o.position = f.viewProj * float4(w, 1.0);
    o.world = w;
    o.colour = float4(float3(v[vid].colour.zyx), float(v[vid].colour.w)) / 255.0;
    const float2 uv = v[vid].uv;
    o.uv = float4(uv * W[1].z + W[1].xy * t, uv * W[2].z + W[2].xy * t);
    if (river) o.uv.yw += 0.25 * t;  // C_QUARTER flow
    o.depth = v[vid].depth;
    const Scatter sc = fuelScattering(f, w);
    o.ext = sc.ext;
    o.insc = sc.insc;
    return o;
}

fragment float4 fsWater(WaterOut in [[stage_in]], constant FrameUniforms& f [[buffer(1)]], constant float4* W [[buffer(2)]],
                        texture2d<float> foamTex [[texture(0)]], texture2d<float> normalTex [[texture(1)]],
                        depth2d_array<float> shadowMap [[texture(7)]]) {
    constexpr sampler rep(coord::normalized, filter::linear, mip_filter::linear, address::repeat, max_anisotropy(4));
    const float4 S0 = W[4], S1 = W[5], S2 = W[6], S3 = W[7];
    const float3 V = normalize(f.eye.xyz - in.world);
    // Depth of water along the view ray (the engine reads the depth buffer), x Settings1.z.
    const float waterDepth = max(in.depth, 0.0) / max(abs(V.y), 0.1) * S1.z;
    float3 n = (normalTex.sample(rep, in.uv.xy).xyw * 2.0 - 1.0) + (normalTex.sample(rep, in.uv.zw).xyw * 2.0 - 1.0);
    const float3 ns = normalize(float3(n.x, n.y * S3.x * 3.0, n.z));
    const bool river = in.kind > 0.5;
    // ph_water_river: n.y *= S0.w (lakes: S0.w * 0.2), spec floor 0.001, no foam, own border.
    n = normalize(float3(n.x, n.y * S0.w * (river ? 1.0 : 0.2), n.z));
    const float fog = saturate(1.0 - exp(-waterDepth * 0.1 * S2.z * in.colour.a));
    const float shadow = fuelShadow(shadowMap, in.world, f);
    // light = shadow * sunlight + skylight (scattering(): DLIGHT_COLOR * SC1.z and DSKY_COLOR).
    const float3 L = f.lightDir.xyz;
    const float3 light = f.sunColour.rgb * shadow + f.skyColour.rgb;
    const float3 wc = in.colour.rgb * light * S0.x;
    // c = refr * k + K, tracked through the shader's lerps with the refraction.
    float k = 1.0 - fog;
    float3 K = wc * fog;
    const float3 r = reflect(-V, n);
    // sGlobalCube in FUEL; the port reflects its sky (approximation).
    const float3 env = fuelSky(f, float3(r.x, max(abs(r.y), 0.02), r.z)) * (0.25 + 0.75 * shadow);
    const float F = saturate(pow(1.0 - saturate(dot(V, n)), 5.0) * (1.0 - saturate(S1.w)) + S1.w) * S2.y;
    k *= 1.0 - F;
    K = K * (1.0 - F) + env * F;
    const float spec = pow(max(river ? 0.001 : 1e-4, dot(reflect(-V, ns), L)), S3.y * 2.0) * S3.z * 0.3;
    K += light * spec;
    const float fo = saturate(1.1 - waterDepth * 0.002 * S2.x);
    const float ff = river ? 0.0 : fo * fo * foamTex.sample(rep, in.uv.xy - n.xz * 0.5).a;
    k *= 1.0 - ff;
    K = K * (1.0 - ff) + light * ff;
    const float b = river ? saturate(saturate(waterDepth * 0.01 * S2.x) - length(f.eye.xyz - in.world) * 0.002)
                          : saturate(waterDepth * 0.02 * S2.x);
    // out = lerp(refr, c, b): premultiplied, blended One / OneMinusSourceAlpha.
    return float4((K * in.ext + in.insc) * b, b * (1.0 - k));
}

struct SkyOut {
    float4 position [[position]];
    float2 ndc;
};

vertex SkyOut vsSky(uint vid [[vertex_id]]) {
    const float2 p[3] = {float2(-1, -1), float2(3, -1), float2(-1, 3)};
    SkyOut o;
    o.position = float4(p[vid], 1.0, 1.0);
    o.ndc = p[vid];
    return o;
}

fragment float4 fsSky(SkyOut in [[stage_in]], constant FrameUniforms& f [[buffer(1)]]) {
    float4 w = f.invViewProj * float4(in.ndc, 1.0, 1.0);
    float3 dir = normalize(w.xyz / w.w - f.eye.xyz);
    // ph_sky: the inscatter of the sky box (no texture).
    float3 c = fuelSky(f, dir);
    // ph_skysun: the sun (or moon) sprite: DLIGHT_COLOR x tex.a x extinction x 1000 (moon: x tex.rgb x 100),
    // picked by saturate(blueShift.w - 0.5). Port: a soft disc of 0.5 degrees radius stands in for the
    // sprite texture, the extinction is taken along the sun direction.
    const float3 sd = normalize(f.sunRaw.xyz);
    const float disc = smoothstep(0.99996, 0.999975, dot(dir, sd));
    if (disc > 0.0) {
        const Scatter sc = fuelScattering(f, f.eye.xyz + sd * kSkyDomeRadius);
        const float3 sunDisc = f.dlight.rgb * disc * sc.ext * 1000.0, moonDisc = f.dlight.rgb * disc * sc.ext * 100.0;
        c += mix(moonDisc, sunDisc, saturate(f.dlight.w - 0.5));
    }
    return float4(c, 1.0);
}


struct MarkerOut {
    float4 position [[position]];
    float3 color;
    float glow;
};

// A beacon: an open cylinder from the ground to 150 m, fading upwards.
vertex MarkerOut vsMarker(uint vid [[vertex_id]], uint iid [[instance_id]], constant FrameUniforms& f [[buffer(1)]],
                          constant float4* markers [[buffer(2)]]) {
    float4 m = markers[iid];
    float a = float(vid / 2) / 32.0 * 2.0 * M_PI_F;
    bool top = (vid & 1) != 0;
    float radius = m.w == 2.0 ? 6.0 : 4.0;
    float3 w = m.xyz + float3(cos(a) * radius, top ? 150.0 : -3.0, sin(a) * radius);
    MarkerOut o;
    o.position = f.viewProj * float4(w, 1.0);
    o.color = m.w == 0.0 ? float3(1.0, 0.55, 0.1) : m.w == 1.0 ? float3(0.35, 0.6, 1.0) : float3(0.3, 1.0, 0.35);
    o.glow = top ? 0.0 : (m.w == 1.0 ? 0.5 : 1.0);
    return o;
}

fragment float4 fsMarker(MarkerOut in [[stage_in]]) {
    float g = in.glow * in.glow;
    return float4(in.color * g, g * 0.7);
}

// ---- FUEL's effects billboards (sfx.vhl / sfx.phl, p30-effects.md 2.5): vh_bboard_soft_static (GPU-animated
// wheel particles, SubCode 0xe) and vh_billboard_soft (CPU particles, 0xb), both lit by scattering() and drawn
// with ph_billboard_soft over the resolved scene.

struct SfxSprite {
    float4 posAngle;
    float4 uv;
    float4 colour;
    float4 vel;
    float4 acc;
    float4 life;
    float4 alphaP;
    float4 scaleP;
};

struct SfxParams {
    float4 user0;  // C_USER0: x FX clock, y 100 ("hundred" of GetParameterValue)
    float4 user1;  // C_USER1: x fade-to-camera distance (PP5530), y fade-out distance (PP5531), z 0.01 collapse
    float4 user2;  // C_USER2: x 0.8 FastFade frontier, y 0.1 (0x5f6550)
};

struct SfxOut {
    float4 position [[position]];
    float2 uv;
    float4 sun;      // (sky + sun saturate(L.y)) * vcol, a = vertex alpha
    float4 sky;      // sky * vcol
    float3 insc;
    float viewZ;
    float shadow;    // SFX_VERTEX_SHADOWS: the shadow term at the vertex
};

// sfx.h GetParameterValue: integer part / 100 and fraction x 100 of each component weight 1, t, sin(pi t),
// 0.25 + 0.75 t, 0.25 + 0.75 sin(pi t), an inverting lerp and a final power.
float sfxParameter(float4 P, float life, float hundred) {
    const float4 lo = fract(P) * hundred;
    const float4 hi = (P - fract(P)) / hundred;
    const float sl = sin(life * M_PI_F);
    float v = hi.x + lo.x * life + hi.y * sl + lo.y * (0.25 + 0.75 * life) + hi.z * (0.25 + 0.75 * sl);
    v = v + lo.z * ((1.0 - v) - v);
    return pow(v, hi.w);
}

vertex SfxOut vsSfx(uint vid [[vertex_id]], uint iid [[instance_id]], const device SfxSprite* ps [[buffer(0)]],
                    constant FrameUniforms& f [[buffer(1)]], constant SfxParams& u [[buffer(2)]],
                    depth2d_array<float> shadowMap [[texture(0)]]) {
    const SfxSprite p = ps[iid];
    const float2 corner = float2(vid & 1, vid >> 1) * 2.0 - 1.0;  // (-1,-1) (1,-1) (-1,1) (1,1)
    float3 pos = p.posAngle.xyz;
    float4 diffuse = p.colour;
    float angle = p.posAngle.w;
    float scale = 1.0;
    float2 vert = corner * p.vel.w;
    if (p.life.z > 0.5) {  // vh_bboard_soft_static
        const float dT = u.user0.x - p.life.x;
        const float life = dT / p.life.y;
        pos += dT * p.vel.xyz + dT * dT * 0.5 * p.acc.xyz;
        if (life > 1.0) {
            diffuse = float4(0.0);
        } else {
            const float dist = length(pos - f.eye.xyz);
            const float alphaCamera = saturate((dist - u.user1.y) / (u.user1.x - u.user1.y));
            diffuse.a *= sfxParameter(p.alphaP, life, u.user0.y) * alphaCamera;
            diffuse.a *= 1.0 - saturate((life - u.user2.x) / u.user2.y);  // FastFade
            angle += dT * p.acc.w;
            vert = clamp(vert, -2.5, 2.5) * sfxParameter(p.scaleP, life, u.user0.y);
        }
    } else {  // vh_billboard_soft
        const float dist = length(pos - f.eye.xyz);
        diffuse.a *= saturate((dist - u.user1.y) / (u.user1.x - u.user1.y));
    }
    if (diffuse.a > u.user1.z) {
        const float s = sin(angle), c = cos(angle);
        const float2 r = float2(vert.x * c - vert.y * s, vert.x * s + vert.y * c);
        pos += r.x * f.camRight.xyz + r.y * f.camUp.xyz;
    }
    SfxOut o;
    o.position = f.viewProj * float4(pos, 1.0);
    // Corner -> atlas uv (0x5fac70: -x corners take u0, +x corners u1; the top takes v0).
    o.uv = float2(corner.x < 0.0 ? p.uv.x : p.uv.z, corner.y > 0.0 ? p.uv.y : p.uv.w);
    const float3 fwd = cross(f.camUp.xyz, f.camRight.xyz);
    o.viewZ = dot(pos - f.eye.xyz, fwd);
    const Scatter sc = fuelScattering(f, pos);
    const float3 vcol = diffuse.rgb * 2.0 * sc.ext;
    o.sun = float4((f.skyColour.rgb + f.sunColour.rgb * saturate(f.lightDir.y)) * vcol, diffuse.a);
    o.sky = float4(f.skyColour.rgb * vcol, diffuse.a);
    o.insc = sc.insc;
    o.shadow = fuelShadow(shadowMap, pos, f);
    if (u.user2.z > 0.0) o.sun = o.sky = float4(4.0, 0.0, 0.0, 1.0);  // FUEL_SFX_DEBUG=2: solid red sprites
    return o;
}

fragment float4 fsSfx(SfxOut in [[stage_in]], depth2d<float> depth [[texture(0)]], texture2d<float> atlas [[texture(1)]],
                      constant FrameUniforms& f [[buffer(1)]]) {
    constexpr sampler smp(coord::normalized, filter::linear, mip_filter::linear, address::clamp_to_edge);
    float4 c = atlas.sample(smp, in.uv);
    if (c.a < 0.01) discard_fragment();  // sfxClip
    c *= mix(in.sky, in.sun, in.shadow);
    c.rgb += in.insc;
    // ph_billboard_soft: a *= saturate((sceneViewZ - particleViewZ) * 4); the resolved depth -> view distance.
    const float d = depth.read(uint2(in.position.xy));
    const float scene = f.fog.z * f.fog.w / (d + f.fog.w);
    c.a *= saturate((scene - in.viewZ) * 4.0);
    return float4(max(c.rgb, 0.0), saturate(c.a));
}

// ---- FUEL's billboard clouds (SpecialEffectsCloudTank_G, SubCode 0x27): sfx.vhl vh_cloud + sfx.phl
// ph_sfxScattering (the variant without bShadowMap). One instance per active cloud; the constants are 0x5fe370's.

struct CloudSprite {
    float4 pos;  // xyz cloud position (y 0), w half width (+0x14)
    float4 uv;   // u at -x, u at +x, v at the top, v at the bottom (0x5fe580)
    float4 size; // x half height (+0x18)
};

struct CloudParams {
    float4 user0;  // C_USER0: x Cloudness (+0x94), y CloudsAlpha (+0xf4 = (cloud + 1) / 2)
    float4 user5;  // C_USER5: x downHeight -100
    float4 user6;  // C_USER6: x lightning range 2000, y AppearanceZone (sight), z SightLine (0.8 sight), w layer + tank y
    float4 rgba;   // the vertex colour: 1.25 PP5201 x3, 0.5
};

struct CloudOut {
    float4 position [[position]];
    float2 uv;
    float4 sun;  // (sky + sun saturate(L.y)) x vcol, a = the distance alpha
    float3 insc;
    float viewZ;
};

// vh_cloud's corner: world position, distance alpha (o.diffuse.a) and the bottom darkening 1 - Cloudness s.
float3 fuelCloudCorner(constant FrameUniforms& f, float3 cpos, float2 halfSize, float2 corner, float4 user0, float4 user5,
                       float4 user6, thread float& alpha, thread float& shade) {
    const float Cloudness = user0.x, zone = user6.y, sightLine = user6.z;
    float3 position = float3(cpos.x, cpos.y + user6.w, cpos.z);
    const float3 vEye = position - f.eye.xyz;
    const float len = length(vEye);
    float scaleFactor = mix(1.0, 0.75, len / zone);
    if (len > zone) scaleFactor = 0.0;
    position.y += mix(0.0, user5.x, len / zone);
    const float3 viewDir = normalize(position - f.eye.xyz);
    const float3 right = normalize(cross(viewDir, float3(0, 1, 0)));
    const float3 up = cross(right, viewDir);
    const float2 uvscale = corner * halfSize;
    shade = 1.0 - Cloudness * (1.0 - (1.0 + sign(uvscale.y)) * 0.5);
    // No lightning flashes in the port (C_USER1..4 w = 0): GetLightningInfluence = 0.
    alpha = saturate(1.0 - (len - (sightLine - zone)) / zone);
    return position + uvscale.x * right * scaleFactor + uvscale.y * up * scaleFactor;
}

vertex CloudOut vsCloud(uint vid [[vertex_id]], uint iid [[instance_id]], const device CloudSprite* cs [[buffer(0)]],
                        constant FrameUniforms& f [[buffer(1)]], constant CloudParams& u [[buffer(2)]]) {
    const CloudSprite c = cs[iid];
    const float2 corner = float2(vid & 1, vid >> 1) * 2.0 - 1.0;
    float alpha, shade;
    const float3 v = fuelCloudCorner(f, c.pos.xyz, float2(c.pos.w, c.size.x), corner, u.user0, u.user5, u.user6, alpha, shade);
    CloudOut o;
    o.position = f.viewProj * float4(v, 1.0);
    o.uv = float2(corner.x < 0.0 ? c.uv.x : c.uv.y, corner.y > 0.0 ? c.uv.z : c.uv.w);
    // o.diffuse = rgba x CloudsAlpha, a = 1, rgb x (1 - Cloudness s), a x the distance fade.
    const float3 diffuse = u.rgba.rgb * u.user0.y * shade;
    const Scatter sc = fuelScattering(f, v);
    const float3 vcol = diffuse * sc.ext * 2.0;
    o.sun = float4((f.skyColour.rgb + f.sunColour.rgb * saturate(f.lightDir.y)) * vcol, alpha);
    o.insc = sc.insc;
    o.viewZ = dot(v - f.eye.xyz, cross(f.camUp.xyz, f.camRight.xyz));
    return o;
}

fragment float4 fsCloud(CloudOut in [[stage_in]], depth2d<float> depth [[texture(0)]], texture2d<float> tex [[texture(1)]],
                        constant FrameUniforms& f [[buffer(1)]]) {
    constexpr sampler smp(coord::normalized, filter::linear, mip_filter::linear, address::clamp_to_edge);
    float4 c = tex.sample(smp, in.uv);
    c.a *= in.sun.a;                     // color.w *= GetShadowLight(i).w
    if (c.a < 0.01) discard_fragment();  // sfxClip
    c.rgb *= in.sun.rgb;
    c.rgb += in.insc;
    const float d = depth.read(uint2(in.position.xy));
    if (in.viewZ > f.fog.z * f.fog.w / (d + f.fog.w)) discard_fragment();  // behind the scene
    return float4(max(c.rgb, 0.0), saturate(c.a));
}

// ---- Weather tanks (rain.vhl vh_rain, snow.vhl vh_snow, drawn with ph_sfxBase; p30-effects.md 6): N drops in a
// [-1, 1]^3 tank of half size TankScale whose centre sits ahead of the camera, wrapped by MapTankCoordinate.

struct TankParams {
    float4 user[8];  // C_USER0..7 as the tank's constants function writes them (0x5e21a0 rain, 0x5dffc0 snow)
    float4 mode;     // x 0 rain / 1 snow
    float4 uv;       // atlas rect of the tank (u0, v0, du, dv)
};

struct TankOut {
    float4 position [[position]];
    float4 diffuse;
    float2 uv;
    float viewZ;
};

float3 mapTankCoordinate(float3 position, float3 tankPosition, float tankScale) {
    const float3 p = position - tankPosition / tankScale;  // (sic: the shader divides only the tank position)
    const float3 sp = sign(p);
    return (fmod(p + sp, 2.0) - sp) * tankScale + tankPosition;
}

vertex TankOut vsTank(uint vid [[vertex_id]], uint iid [[instance_id]], const device float4* drops [[buffer(0)]],
                      constant FrameUniforms& f [[buffer(1)]], constant TankParams& t [[buffer(2)]]) {
    const float4 d = drops[iid];
    const float2 corner = float2(vid & 1, vid >> 1) * 2.0 - 1.0;
    const float3 fwd = cross(f.camUp.xyz, f.camRight.xyz);
    TankOut o;
    float3 world;
    if (t.mode.x < 0.5) {  // vh_rain
        const float4 RainColor = t.user[0];
        const float3 Speed = t.user[1].xyz;
        const float TankScale = t.user[1].w;
        const float WidthFar = t.user[2].x, WidthNear = t.user[2].y, Height = t.user[2].z, Individual = t.user[2].w;
        const float AlphaNear = t.user[3].x, AlphaFar = t.user[3].y;
        const float3 position = mapTankCoordinate(d.xyz + t.user[5].xyz, t.user[4].xyz, TankScale);
        const float speedz = fract(d.w), speedx = d.w / 1000.0;
        const float2 individual = float2(speedx - 0.5, speedz - 0.5) * 2.0;
        const float2 xz = (Speed.xz + individual * Individual) * -corner.y;
        world = float3(position.x + xz.x, position.y + Height * corner.y, position.z + xz.y);
        const float depth = abs(dot(world - f.eye.xyz, fwd) / TankScale);
        world += f.camRight.xyz * (mix(WidthNear, WidthFar, depth) * corner.x);  // view-space x
        o.diffuse = float4(RainColor.rgb, mix(AlphaNear, AlphaFar, depth));
        // u: the quad's two corner columns are the drop's own quarter column of the atlas rect (0x5e2490)
        const float col = floor(fract(d.w * 7.31) * 4.0);
        const float u = t.uv.x + col * t.uv.z * 0.25 + t.uv.z * 0.0625 + (corner.x > 0.0 ? t.uv.z * 0.125 : 0.0);
        o.uv = float2(u, corner.y > 0.0 ? t.uv.y : t.uv.y + t.uv.w);
    } else {  // vh_snow
        const float4 SnowColor = float4(t.user[0].rgb, 0.3);
        const float3 Speed = t.user[1].xyz * 0.6;
        const float TankScale = t.user[1].w;
        const float ParticleScale = t.user[2].x, WaveAmplitude = t.user[2].y, WaveFrequency = t.user[2].z;
        const float scaleRatio = t.user[3].x;
        const float3 current = d.xyz + Speed;
        float3 position = mapTankCoordinate(current, t.user[4].xyz, TankScale);
        const float angle = WaveFrequency * length(current) + WaveFrequency * d.w;
        const float s = sin(angle), c = cos(angle);
        position += WaveAmplitude * float3(c - s, s - c, s + c);
        const float depth = abs(dot(position - f.eye.xyz, fwd) / TankScale);
        const float scale = ParticleScale * (1.0 + scaleRatio * depth);
        world = position + (f.camRight.xyz * corner.x + f.camUp.xyz * corner.y) * scale;
        o.diffuse = SnowColor;
        o.uv = float2(corner.x < 0.0 ? t.uv.x : t.uv.x + t.uv.z, corner.y > 0.0 ? t.uv.y : t.uv.y + t.uv.w);
    }
    o.position = f.viewProj * float4(world, 1.0);
    o.viewZ = dot(world - f.eye.xyz, fwd);
    return o;
}

// ph_sfxBase: rgb = vcol x tex x 2, a = vcol.a x tex.a, clip at 0.01; hidden behind the scene (resolved depth).
fragment float4 fsTank(TankOut in [[stage_in]], depth2d<float> depth [[texture(0)]], texture2d<float> atlas [[texture(1)]],
                       constant FrameUniforms& f [[buffer(1)]]) {
    constexpr sampler smp(coord::normalized, filter::linear, mip_filter::linear, address::clamp_to_edge);
    const float4 tex = atlas.sample(smp, in.uv);
    float4 c = float4(in.diffuse.rgb * tex.rgb * 2.0, in.diffuse.a * tex.a);
    if (c.a < 0.01) discard_fragment();
    const float d = depth.read(uint2(in.position.xy));
    const float scene = f.fog.z * f.fog.w / (d + f.fog.w);
    if (in.viewZ > scene) discard_fragment();
    return float4(c.rgb, saturate(c.a));
}

// ---- FUEL's HDR post chain (fx.phl; p26-work/post.md P3): 128 x 128 scene shrink, log luminance,
// 2 x 2 reductions to 1 x 1, eye adaptation, bright pass, 11-tap gaussian, two star streaks, tone map
// (blue shift, exposure, bloom + star, x + x^2 + x^3 + x^4 curve, tint), radial speed blur.

struct PostUniforms {
    float4 bloom;      // gBloom: LumScale, Bias, Scale, Star amount
    float4 hdr;        // x k = 0.94^(30 dt), y SCREENLUMINANCE, z min exposure, w max exposure
    float4 blueShift;  // gBlueShift
    float4 tint;       // gTintLerp
    float4 mb0;        // gMotionBlurSettings0 (1/(Max-Min), -Min/(Max-Min), FallOff, scale)
    float4 texel;      // xy: 1 / source size of the pass; zw: 1 / scene size
    float4 flags;      // x 1: adaptation init; y 1: bloom and star; z 1: the output target is sRGB-encoded
    float4 taps[11];   // gSampleOffsetWeight: xy offset (uv), z weight
    float4 mbw[8];     // radial blur weights in .x
};

struct FullOut {
    float4 position [[position]];
    float2 uv;
};

vertex FullOut vsFull(uint vid [[vertex_id]]) {
    const float2 p[3] = {float2(-1, -1), float2(3, -1), float2(-1, 3)};
    FullOut o;
    o.position = float4(p[vid], 0.0, 1.0);
    o.uv = p[vid] * float2(0.5, -0.5) + 0.5;
    return o;
}

constexpr sampler postLin(coord::normalized, filter::linear, address::clamp_to_edge);
constexpr sampler postPoint(coord::normalized, filter::nearest, address::clamp_to_edge);

// 1. The scene shrunk to 128 x 128 (ph_2d, one bilinear tap).
fragment float4 fsShrink(FullOut in [[stage_in]], texture2d<float> src [[texture(0)]]) { return src.sample(postLin, in.uv); }

// 2. ph_lumlogavg: mean of ln(max(lum, 0.001)) over four point taps of the full scene at +-half a scene pixel.
fragment float4 fsLumLog(FullOut in [[stage_in]], texture2d<float> src [[texture(0)]], constant PostUniforms& u [[buffer(0)]]) {
    float s = 0.0;
    for (int k = 0; k < 4; ++k) {
        const float2 o = float2((k & 1) ? 0.5 : -0.5, (k & 2) ? 0.5 : -0.5) * u.texel.zw;
        s += 0.25 * log(max(dot(src.sample(postPoint, in.uv + o).rgb, float3(0.3, 0.59, 0.11)), 0.001));
    }
    return float4(s, 0, 0, 1);
}

// 3. ph_lumavg: exact 2 x 2 box mean of the level above.
fragment float4 fsLumAvg(FullOut in [[stage_in]], texture2d<float> src [[texture(0)]], constant PostUniforms& u [[buffer(0)]]) {
    float s = 0.0;
    for (int k = 0; k < 4; ++k) {
        const float2 o = float2((k & 1) ? 0.5 : -0.5, (k & 2) ? 0.5 : -0.5) * u.texel.xy;
        s += 0.25 * src.sample(postPoint, in.uv + o).r;
    }
    return float4(s, 0, 0, 1);
}

// 4. ph_lumadaptation(init): the exposure S = 1 / adapted luminance.
fragment float4 fsAdapt(FullOut in [[stage_in]], texture2d<float> avg [[texture(0)]], texture2d<float> prev [[texture(1)]],
                        constant PostUniforms& u [[buffer(0)]]) {
    const float A = avg.read(uint2(0, 0)).r;
    const float sl = u.hdr.y;
    if (u.flags.x > 0.0) return float4(sl / exp(A), 0, 0, 1);
    const float c = clamp(1.5 * exp(A) / sl, 1.0 / u.hdr.w, 1.0 / u.hdr.z);
    const float a = 1.0 / prev.read(uint2(0, 0)).r;
    const float an = c * pow(a / c, u.hdr.x);
    float S = 1.0 / an;
    if (!(S > 0.01 && S < 100.0)) S = 1.0 / c;
    return float4(S, 0, 0, 1);
}

// 5. ph_hdrbloominit: bright pass into the 8-bit bloom target (clamped to [0, 1] by the format).
fragment float4 fsBright(FullOut in [[stage_in]], texture2d<float> src [[texture(0)]], texture2d<float> expo [[texture(1)]],
                         constant PostUniforms& u [[buffer(0)]]) {
    const float S = expo.read(uint2(0, 0)).r;
    const float3 c = src.sample(postLin, in.uv).rgb;
    return float4(max(float3(0.0), (4.0 * c * u.bloom.x * (0.3 / u.hdr.y) * S - u.bloom.y) * u.bloom.z), 1.0);
}

// 6, 7. ph_hdrbloom: 11 weighted bilinear taps (gaussian, or a diagonal star streak with scaled offsets).
fragment float4 fsGauss(FullOut in [[stage_in]], texture2d<float> src [[texture(0)]], constant PostUniforms& u [[buffer(0)]]) {
    float3 c = 0.0;
    for (int k = 0; k < 11; ++k) c += u.taps[k].z * src.sample(postLin, in.uv + u.taps[k].xy).rgb;
    return float4(c, 1.0);
}

// The display value v goes out through the inverse sRGB curve when the target encodes sRGB, so the
// stored byte is v itself (FUEL writes the tone-mapped value straight into an 8-bit buffer).
float3 displayOut(float3 v, constant PostUniforms& u) {
    v = saturate(v);
    if (u.flags.z <= 0.0) return v;
    return select(pow((v + 0.055) / 1.055, 2.4), v / 12.92, v <= 0.04045);
}

// 8. ph_tonemapping.
float3 toneMapFuel(float3 c, float S, float3 bloom, constant PostUniforms& u) {
    const float bw = dot(c, float3(0.2125, 0.7154, 0.0721));
    float t = 1.0 - min(1.0, u.blueShift.w * bw);
    t *= t;
    c = mix(c, u.blueShift.rgb * bw, t);
    c = c * S + bloom;
    const float3 P = c + c * c + c * c * c + c * c * c * c;
    c = P / (1.0 + P);
    const float br = dot(c, float3(0.2125, 0.7154, 0.0721));
    return mix(c, br * u.tint.rgb, u.tint.a);
}

fragment float4 fsToneMap(FullOut in [[stage_in]], texture2d<float> scene [[texture(0)]], texture2d<float> expo [[texture(1)]],
                          texture2d<float> bloom [[texture(2)]], texture2d<float> starA [[texture(3)]], texture2d<float> starB [[texture(4)]],
                          constant PostUniforms& u [[buffer(0)]]) {
    const float3 c = scene.read(uint2(in.position.xy)).rgb;
    const float S = expo.read(uint2(0, 0)).r;
    float3 b = 0.0;
    if (u.flags.y > 0.0) {
        const float2 buv = in.uv + 1.0 / 128.0;
        b = bloom.sample(postLin, buv).rgb + u.bloom.w * (starA.sample(postLin, buv).rgb + starB.sample(postLin, buv).rgb);
    }
    return float4(displayOut(toneMapFuel(c, S, b, u), u), 1.0);
}

// 9. ph_motionblur: 8 taps towards the screen centre, stronger at the edges and the bottom.
fragment float4 fsRadial(FullOut in [[stage_in]], texture2d<float> src [[texture(0)]], constant PostUniforms& u [[buffer(0)]]) {
    const float2 H = in.uv * 2.0 - 1.0;
    const float l = length(H);
    const float2 dir = l > 1e-6 ? H / l : float2(0.0);
    float a = saturate(l * u.mb0.x + u.mb0.y);
    a = a * a * a * a * u.mb0.w * in.uv.y;
    const float2 step = -dir * a;
    float3 c = 0.0;
    for (int i = 0; i < 8; ++i) c += u.mbw[i].x * src.sample(postLin, in.uv + float(i) * step).rgb;
    return float4(displayOut(c, u), 1.0);
}

// ---- --rendercheck: the shader maths on test inputs (see rendercheck.mm for the input layout).
kernel void kRenderCheck(constant FrameUniforms& f [[buffer(0)]], const device float4* in [[buffer(1)]], device float4* out [[buffer(2)]],
                         uint id [[thread_position_in_grid]]) {
    const float4 a = in[id * 4], b = in[id * 4 + 1], c = in[id * 4 + 2], d = in[id * 4 + 3];
    float4 r0 = 0.0, r1 = 0.0;
    switch (int(a.w)) {
        case 0: {  // scattering of point a.xyz
            const Scatter s = fuelScattering(f, a.xyz);
            r0 = float4(s.ext, 0);
            r1 = float4(s.insc, 0);
            break;
        }
        case 1: {  // lighting(): n = a.xyz, V = b.xyz, albedo = c.xyz, shadow = c.w, occ = d.x, spec = d.y, power = d.z
            const float3 col = fuelLighting(normalize(a.xyz), f.lightDir.xyz, f.skyDir.xyz, normalize(b.xyz), f.sunColour.rgb,
                                            mix(f.skyAmbient.rgb, f.skyColour.rgb, d.x), c.xyz, float3(d.y), d.z, c.w, float3(0), d.x);
            r0 = float4(col, 0);
            break;
        }
        case 2: {  // tone curve of a.x, a.y, a.z
            const float3 x = a.xyz;
            const float3 P = x + x * x + x * x * x + x * x * x * x;
            r0 = float4(P / (1.0 + P), 0);
            break;
        }
        case 4: {  // sfx.h GetParameterValue(P, t): P = b, c, d at t = a.x, a.y, a.z (hundred = 100)
            r0 = float4(sfxParameter(b, a.x, 100.0), sfxParameter(c, a.y, 100.0), sfxParameter(d, a.z, 100.0), 0.0);
            break;
        }
        case 3: {  // the sky along a.xyz
            r0 = float4(fuelSky(f, normalize(a.xyz)), 0);
            break;
        }
        case 5: {  // vh_cloud corner: cloud a.xyz, half size b.xy, corner b.zw, C_USER0 = c, C_USER6 = d, C_USER5.x -100
            float alpha, shade;
            r0 = float4(fuelCloudCorner(f, a.xyz, b.xy, b.zw, c, float4(-100, 0, 0, 0), d, alpha, shade), alpha);
            r1 = float4(shade, 0, 0, 0);
            break;
        }
    }
    out[id * 2] = r0;
    out[id * 2 + 1] = r1;
}
)METAL";

// ---- Grass tufts (p14) -------------------------------------------------------
// FUEL's Grass_EmitTuft (0x7dce50): a 5-vertex card (3 triangles) per tuft from a
// cell of the Grass material's 4x4 atlas; 1.5 x size wide, size tall, facing the
// camera around the vertical; the two upper-left vertices sway with the wind.
// Appended to kShaderSource (it uses its helpers and FrameUniforms).
const char* kGrassShaderSource = R"METAL(
struct GrassInstance {
    float4 posSize;  // base xyz, card height
    uchar4 info;     // atlas cell, wind amplitude x 100, road weight x 255, unused
    uchar4 colour;   // the point's ground colour (sRGB) and sky occlusion (p16)
};

struct GrassParams {
    float4 a;  // x time (s), y draw radius, z fade start, w wind strength
};

struct GrassOut {
    float4 position [[position]];
    float3 world;
    float2 uv;
    float3 vcol;      // colour x 2 x extinction (ground_rotshape.vhl)
    float3 sky;       // lerp(DSKY_AMBIENT, DSKY_COLOR, occlusion)
    float3 sunlit;    // sky + sun x saturate(L.n), n = up (port: the tufts carry no normal)
    float3 insc;
};

constant uint kGrassCorner[9] = {0, 1, 2, 0, 2, 3, 0, 3, 4};
constant float2 kGrassOffset[5] = {float2(-0.75, 0), float2(-0.75, 0.5), float2(0, 1), float2(0.75, 0.5), float2(0.75, 0)};
constant float2 kGrassUv[5] = {float2(0, 0.25), float2(0, 0.125), float2(0.125, 0), float2(0.25, 0.125), float2(0.25, 0.25)};

vertex GrassOut vsGrass(uint vid [[vertex_id]], uint iid [[instance_id]], constant FrameUniforms& f [[buffer(1)]],
                        const device GrassInstance* tufts [[buffer(2)]], constant GrassParams& g [[buffer(3)]],
                        texture2d<float> groundColour [[texture(0)]]) {
    const GrassInstance t = tufts[iid];
    const uint c = kGrassCorner[vid];
    const float3 base = t.posSize.xyz;
    // Shrink into the ground towards the edge of the grass radius instead of popping.
    const float dist = distance(base.xz, f.eye.xz);
    const float size = t.posSize.w * (1.0 - smoothstep(g.a.z, g.a.y, dist));
    float3 right = float3(f.camRight.x, 0.0, f.camRight.z);
    right = dot(right, right) > 1e-6 ? normalize(right) : float3(1, 0, 0);
    float3 w = base + right * (kGrassOffset[c].x * size) + float3(0, kGrassOffset[c].y * size, 0);
    if (c == 1 || c == 2) {
        // Engine wind weights: wb from sin(x+z) and sin(0.25(x+z)); amplitude (r/255+1)/2 x 2.25.
        const float s = base.x + base.z;
        const float wb = 0.25 * (sin(s) + 1.0) + 0.25 * (sin(0.25 * s) + 1.0);
        const float amp = float(t.info.y) * 0.01 * size * g.a.w;
        const float sway = wb * sin(g.a.x * 1.9 + s * 0.31) + (1.0 - wb) * sin(g.a.x * 1.3 + s * 0.17);
        w += float3(0.8, 0.0, 0.6) * (sway * amp);
    }
    GrassOut o;
    o.position = f.viewProj * float4(w, 1.0);
    o.world = w;
    const uint tile = t.info.x;
    o.uv = float2(float(tile & 3u), float(tile >> 2u)) * 0.25 + kGrassUv[c];
    // ground_rotshape.vhl: vcolor = colour x 2 x extinction (gamma space, like the game); the colour is the
    // decoded point colour lerp(ctx+0x80, ctx+0x50, road) and its alpha the sky occlusion.
    const Scatter sc = fuelScattering(f, w);
    o.vcol = 2.0 * float3(t.colour.rgb) / 255.0 * sc.ext;
    o.insc = sc.insc;
    const float occ = float(t.colour.a) / 255.0;
    o.sky = mix(f.skyAmbient.rgb, f.skyColour.rgb, occ);
    o.sunlit = o.sky + f.sunColour.rgb * saturate(f.lightDir.y);
    return o;
}

fragment float4 fsGrass(GrassOut in [[stage_in]], constant FrameUniforms& f [[buffer(1)]], texture2d<float> atlas [[texture(0)]],
                        depth2d_array<float> shadowMap [[texture(7)]]) {
    constexpr sampler smp(coord::normalized, filter::linear, mip_filter::linear, address::clamp_to_edge, max_anisotropy(4));
    float4 d = atlas.sample(smp, in.uv);
    if (d.a < 0.4) discard_fragment();
    // ground_rotshape.phl: vcol x tex x lerp(sky, sunlit, sunlighting(shadow, 0.5)) + inscatter.
    const float shadow = fuelShadow(shadowMap, in.world, f);
    const float3 rgb = d.rgb * in.vcol * mix(in.sky, in.sunlit, fuelSunlighting(shadow, 0.5)) + in.insc;
    return float4(rgb, d.a);
}
)METAL";

}  // namespace

struct FrameUniforms {
    simd_float4x4 viewProj;
    simd_float4x4 sunViewProj;
    simd_float4x4 invViewProj;
    simd_float4 sunDir;
    simd_float4 paint;
    simd_float4 eye;
    simd_float4 params;
    simd_float4 footprint;
    simd_float4 terrain;
    simd_float4 terrainGrid;
    simd_float4 fog;
    simd_float4 camRight;
    simd_float4 camUp;
    simd_float4 groundFar;
    simd_float4 groundFine[3];
    simd_float4 lightDir, skyDir, sunColour, dlight, skyColour, skyAmbient, sunRaw;
    simd_float4 scat[11];
    simd_float4 shadowSplit, shadowSize;
    simd_float4x4 shadowMat[4];
    simd_float4x4 view;
};

namespace {

constexpr uint32_t kTerrainGridSize = 1024;  // vertices per side

struct MaterialUniforms {
    simd_float4 diffuse;
    simd_float4 specular;
    simd_float4 emissive;
    simd_float2 fresnel;
    uint32_t alphaMode;
    uint32_t lit;
    uint32_t code;
    uint32_t features;
};

struct PostUniforms {
    simd_float4 bloom, hdr, blueShift, tint, mb0, texel, flags;
    simd_float4 taps[11];
    simd_float4 mbw[8];
};

struct ShadowUniforms {
    simd_float4x4 lightViewProj;
    simd_float4 light;
    simd_float4 eye;  // the terrain caster's geomorph (unused by mesh casters)
    simd_float4 lightRight, lightUp;
};

// Right-handed view and projection with clip z in [0, 1]. FUEL's data is
// right-handed: with left-handed matrices, billboard text shows mirrored and
// front faces wind counter-clockwise. With these, text reads correctly and
// front faces are clockwise, the Direct3D 9 default cull convention.
simd_float4x4 lookAtRH(simd_float3 eye, simd_float3 target, simd_float3 up) {
    const simd_float3 z = simd_normalize(eye - target);
    const simd_float3 x = simd_normalize(simd_cross(up, z));
    const simd_float3 y = simd_cross(z, x);
    return simd_matrix(simd_make_float4(x.x, y.x, z.x, 0), simd_make_float4(x.y, y.y, z.y, 0),
                       simd_make_float4(x.z, y.z, z.z, 0),
                       simd_make_float4(-simd_dot(x, eye), -simd_dot(y, eye), -simd_dot(z, eye), 1));
}

simd_float4x4 perspectiveRH(float fovY, float aspect, float zn, float zf) {
    const float f = 1.0f / std::tan(fovY * 0.5f);
    const float q = zf / (zn - zf);
    return simd_matrix(simd_make_float4(f / aspect, 0, 0, 0), simd_make_float4(0, f, 0, 0), simd_make_float4(0, 0, q, -1),
                       simd_make_float4(0, 0, zn * q, 0));
}

simd_float4x4 orthoRH(float halfSize, float zn, float zf) {
    const float q = 1.0f / (zn - zf);
    return simd_matrix(simd_make_float4(1 / halfSize, 0, 0, 0), simd_make_float4(0, 1 / halfSize, 0, 0),
                       simd_make_float4(0, 0, q, 0), simd_make_float4(0, 0, zn * q, 1));
}

float srgbToLinear(float c) { return c <= 0.04045f ? c / 12.92f : std::pow((c + 0.055f) / 1.055f, 2.4f); }

uint8_t expand(uint32_t v, int bits) { return uint8_t((v * 255 + ((1u << bits) - 1) / 2) / ((1u << bits) - 1)); }

// Converts one mip level of an uncompressed format to BGRA8. Returns false
// for formats the viewer does not display.
bool toBgra8(PixelFormat fmt, const uint8_t* src, uint32_t w, uint32_t h, std::vector<uint8_t>& out) {
    const size_t n = size_t(w) * h;
    out.resize(n * 4);
    for (size_t i = 0; i < n; ++i) {
        uint8_t* d = &out[i * 4];
        uint16_t v16 = 0;
        if (fmt == PixelFormat::A1R5G5B5 || fmt == PixelFormat::R5G6B5 || fmt == PixelFormat::A4R4G4B4) std::memcpy(&v16, src + i * 2, 2);
        switch (fmt) {
            case PixelFormat::A8R8G8B8: std::memcpy(d, src + i * 4, 4); break;
            case PixelFormat::R8G8B8: d[0] = src[i * 3]; d[1] = src[i * 3 + 1]; d[2] = src[i * 3 + 2]; d[3] = 255; break;
            case PixelFormat::A1R5G5B5:
                d[0] = expand(v16 & 31, 5); d[1] = expand((v16 >> 5) & 31, 5); d[2] = expand((v16 >> 10) & 31, 5);
                d[3] = (v16 & 0x8000) ? 255 : 0;
                break;
            case PixelFormat::R5G6B5:
                d[0] = expand(v16 & 31, 5); d[1] = expand((v16 >> 5) & 63, 6); d[2] = expand((v16 >> 11) & 31, 5); d[3] = 255;
                break;
            case PixelFormat::A4R4G4B4:
                d[0] = expand(v16 & 15, 4); d[1] = expand((v16 >> 4) & 15, 4); d[2] = expand((v16 >> 8) & 15, 4);
                d[3] = expand((v16 >> 12) & 15, 4);
                break;
            case PixelFormat::L8: d[0] = d[1] = d[2] = src[i]; d[3] = 255; break;
            default: return false;
        }
    }
    return true;
}

MTLVertexDescriptor* meshVertexDescriptor(uint32_t stride) {
    MTLVertexDescriptor* vd = [MTLVertexDescriptor vertexDescriptor];
    vd.attributes[0].format = MTLVertexFormatFloat3;
    vd.attributes[0].offset = 0;
    vd.attributes[1].format = MTLVertexFormatUChar4Normalized_BGRA;
    vd.attributes[1].offset = 12;
    vd.attributes[2].format = MTLVertexFormatUChar4Normalized_BGRA;
    vd.attributes[2].offset = 16;
    vd.attributes[3].format = MTLVertexFormatFloat2;
    vd.attributes[3].offset = 20;
    vd.attributes[4].format = MTLVertexFormatFloat2;
    vd.attributes[4].offset = 28;
    for (int i = 0; i < 5; ++i) vd.attributes[i].bufferIndex = 0;
    vd.layouts[0].stride = stride;
    return vd;
}

}  // namespace

ResourceLibrary::Hit ResourceLibrary::find(uint32_t id, uint32_t classCrc) const {
    for (const auto& f : files_) {
        const DpcFile::Entry* e = f->find(id);
        if (e != nullptr && e->classCrc == classCrc) return {f.get(), e};
    }
    return {};
}

MeshRenderer::MeshRenderer(id<MTLDevice> device, NSUInteger sampleCount, const ResourceLibrary& library)
    : device_(device), sampleCount_(sampleCount), library_(library) {
    NSError* err = nil;
    shaders_ = [device newLibraryWithSource:[NSString stringWithFormat:@"%s\n%s", kShaderSource, kGrassShaderSource] options:nil error:&err];
    if (shaders_ == nil) [NSException raise:@"shader" format:@"%@", err];

    MTLDepthStencilDescriptor* ds = [MTLDepthStencilDescriptor new];
    ds.depthCompareFunction = MTLCompareFunctionLessEqual;
    ds.depthWriteEnabled = YES;
    depthWrite_ = [device newDepthStencilStateWithDescriptor:ds];
    ds.depthWriteEnabled = NO;
    depthNoWrite_ = [device newDepthStencilStateWithDescriptor:ds];

    for (int i = 0; i < 4; ++i) {
        MTLSamplerDescriptor* sd = [MTLSamplerDescriptor new];
        sd.minFilter = sd.magFilter = MTLSamplerMinMagFilterLinear;
        sd.mipFilter = MTLSamplerMipFilterLinear;
        sd.maxAnisotropy = 8;
        sd.sAddressMode = (i & 1) ? MTLSamplerAddressModeClampToEdge : MTLSamplerAddressModeRepeat;
        sd.tAddressMode = (i & 2) ? MTLSamplerAddressModeClampToEdge : MTLSamplerAddressModeRepeat;
        samplers_[i] = [device newSamplerStateWithDescriptor:sd];
    }

    MTLTextureDescriptor* td = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatBGRA8Unorm
                                                                                    width:1
                                                                                   height:1
                                                                                mipmapped:NO];
    white_ = [device newTextureWithDescriptor:td];
    const uint32_t px = 0xFFFFFFFF;
    [white_ replaceRegion:MTLRegionMake2D(0, 0, 1, 1) mipmapLevel:0 withBytes:&px bytesPerRow:4];

    // FUEL's shadow atlas: 4 slices of W x W (0x837660: 1024 x 4096 R32F; 2048 with -HS), here a depth array.
    MTLTextureDescriptor* sd = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatDepth32Float
                                                                                    width:kShadowMapSize
                                                                                   height:kShadowMapSize
                                                                                mipmapped:NO];
    sd.textureType = MTLTextureType2DArray;
    sd.arrayLength = 4;
    sd.usage = MTLTextureUsageRenderTarget | MTLTextureUsageShaderRead;
    sd.storageMode = MTLStorageModePrivate;
    shadowMap_ = [device newTextureWithDescriptor:sd];

    MTLRenderPipelineDescriptor* gd = [MTLRenderPipelineDescriptor new];
    gd.vertexFunction = [shaders_ newFunctionWithName:@"vsGround"];
    gd.fragmentFunction = [shaders_ newFunctionWithName:@"fsGround"];
    gd.rasterSampleCount = sampleCount_;
    gd.colorAttachments[0].pixelFormat = kRenderColorFormat;
    gd.colorAttachments[0].blendingEnabled = YES;
    gd.colorAttachments[0].sourceRGBBlendFactor = MTLBlendFactorSourceAlpha;
    gd.colorAttachments[0].destinationRGBBlendFactor = MTLBlendFactorOneMinusSourceAlpha;
    gd.depthAttachmentPixelFormat = kRenderDepthFormat;
    groundPipeline_ = [device newRenderPipelineStateWithDescriptor:gd error:&err];
    if (groundPipeline_ == nil) [NSException raise:@"pipeline" format:@"%@", err];

    auto makeSimple = [&](NSString* vs, NSString* fs) {
        MTLRenderPipelineDescriptor* d = [MTLRenderPipelineDescriptor new];
        d.vertexFunction = [shaders_ newFunctionWithName:vs];
        d.fragmentFunction = [shaders_ newFunctionWithName:fs];
        d.rasterSampleCount = sampleCount_;
        d.colorAttachments[0].pixelFormat = kRenderColorFormat;
        d.depthAttachmentPixelFormat = kRenderDepthFormat;
        NSError* e = nil;
        id<MTLRenderPipelineState> ps = [device newRenderPipelineStateWithDescriptor:d error:&e];
        if (ps == nil) [NSException raise:@"pipeline" format:@"%@", e];
        return ps;
    };
    terrainPipeline_ = makeSimple(@"vsTerrain", @"fsTerrain");
    {
        MTLRenderPipelineDescriptor* d = [MTLRenderPipelineDescriptor new];
        d.vertexFunction = [shaders_ newFunctionWithName:@"vsWater"];
        d.fragmentFunction = [shaders_ newFunctionWithName:@"fsWater"];
        d.rasterSampleCount = sampleCount_;
        d.colorAttachments[0].pixelFormat = kRenderColorFormat;
        d.colorAttachments[0].blendingEnabled = YES;
        d.colorAttachments[0].sourceRGBBlendFactor = MTLBlendFactorOne;
        d.colorAttachments[0].destinationRGBBlendFactor = MTLBlendFactorOneMinusSourceAlpha;
        d.colorAttachments[0].sourceAlphaBlendFactor = MTLBlendFactorZero;
        d.colorAttachments[0].destinationAlphaBlendFactor = MTLBlendFactorOne;
        d.depthAttachmentPixelFormat = kRenderDepthFormat;
        NSError* e = nil;
        waterPipeline_ = [device newRenderPipelineStateWithDescriptor:d error:&e];
        if (waterPipeline_ == nil) [NSException raise:@"pipeline" format:@"%@", e];
    }
    {
        MTLRenderPipelineDescriptor* d = [MTLRenderPipelineDescriptor new];
        d.vertexFunction = [shaders_ newFunctionWithName:@"vsMarker"];
        d.fragmentFunction = [shaders_ newFunctionWithName:@"fsMarker"];
        d.rasterSampleCount = sampleCount_;
        d.colorAttachments[0].pixelFormat = kRenderColorFormat;
        d.colorAttachments[0].blendingEnabled = YES;  // additive glow
        d.colorAttachments[0].sourceRGBBlendFactor = MTLBlendFactorOne;
        d.colorAttachments[0].destinationRGBBlendFactor = MTLBlendFactorOne;
        d.depthAttachmentPixelFormat = kRenderDepthFormat;
        NSError* e = nil;
        markerPipeline_ = [device newRenderPipelineStateWithDescriptor:d error:&e];
        if (markerPipeline_ == nil) [NSException raise:@"pipeline" format:@"%@", e];
    }
    skyPipeline_ = makeSimple(@"vsSky", @"fsSky");
    createGrassPipeline();
    ds.depthCompareFunction = MTLCompareFunctionAlways;
    ds.depthWriteEnabled = NO;
    depthAlways_ = [device newDepthStencilStateWithDescriptor:ds];

    // Dust and post passes: single-sample, no depth attachment.
    auto makePost = [&](NSString* vs, NSString* fs, MTLPixelFormat format, int blend) {
        MTLRenderPipelineDescriptor* d = [MTLRenderPipelineDescriptor new];
        d.vertexFunction = [shaders_ newFunctionWithName:vs];
        d.fragmentFunction = [shaders_ newFunctionWithName:fs];
        d.colorAttachments[0].pixelFormat = format;
        if (blend == 1) {  // alpha blending
            d.colorAttachments[0].blendingEnabled = YES;
            d.colorAttachments[0].sourceRGBBlendFactor = MTLBlendFactorSourceAlpha;
            d.colorAttachments[0].destinationRGBBlendFactor = MTLBlendFactorOneMinusSourceAlpha;
            d.colorAttachments[0].sourceAlphaBlendFactor = MTLBlendFactorZero;
            d.colorAttachments[0].destinationAlphaBlendFactor = MTLBlendFactorOne;
        } else if (blend == 2) {  // added to the colour, alpha kept
            d.colorAttachments[0].blendingEnabled = YES;
            d.colorAttachments[0].sourceRGBBlendFactor = MTLBlendFactorOne;
            d.colorAttachments[0].destinationRGBBlendFactor = MTLBlendFactorOne;
            d.colorAttachments[0].writeMask = MTLColorWriteMaskRed | MTLColorWriteMaskGreen | MTLColorWriteMaskBlue;
        }
        NSError* e = nil;
        id<MTLRenderPipelineState> ps = [device newRenderPipelineStateWithDescriptor:d error:&e];
        if (ps == nil) [NSException raise:@"pipeline" format:@"%@", e];
        return ps;
    };
    dustPipeline_ = makePost(@"vsSfx", @"fsSfx", kRenderColorFormat, 1);
    tankPipeline_ = makePost(@"vsTank", @"fsTank", kRenderColorFormat, 1);
    cloudPipeline_ = makePost(@"vsCloud", @"fsCloud", kRenderColorFormat, 1);
    shrinkPipeline_ = makePost(@"vsFull", @"fsShrink", kRenderColorFormat, 0);
    lumLogPipeline_ = makePost(@"vsFull", @"fsLumLog", MTLPixelFormatR32Float, 0);
    lumAvgPipeline_ = makePost(@"vsFull", @"fsLumAvg", MTLPixelFormatR32Float, 0);
    adaptPipeline_ = makePost(@"vsFull", @"fsAdapt", MTLPixelFormatR32Float, 0);
    brightPipeline_ = makePost(@"vsFull", @"fsBright", MTLPixelFormatRGBA8Unorm, 0);
    gaussPipeline_ = makePost(@"vsFull", @"fsGauss", MTLPixelFormatRGBA8Unorm, 0);
    toneMapPipeline_ = makePost(@"vsFull", @"fsToneMap", kOutputColorFormat, 0);
    toneMapFxPipeline_ = makePost(@"vsFull", @"fsToneMap", MTLPixelFormatRGBA8Unorm, 0);
    radialPipeline_ = makePost(@"vsFull", @"fsRadial", kOutputColorFormat, 0);
    {
        MTLRenderPipelineDescriptor* d = [MTLRenderPipelineDescriptor new];
        d.vertexFunction = [shaders_ newFunctionWithName:@"vsTerrainShadow"];
        d.depthAttachmentPixelFormat = MTLPixelFormatDepth32Float;
        NSError* e = nil;
        terrainShadowPipeline_ = [device newRenderPipelineStateWithDescriptor:d error:&e];
        if (terrainShadowPipeline_ == nil) [NSException raise:@"pipeline" format:@"%@", e];
        // Reversed depth: 1 at the light camera, cleared to 0, the nearest caster to the sun wins.
        MTLDepthStencilDescriptor* sds = [MTLDepthStencilDescriptor new];
        sds.depthCompareFunction = MTLCompareFunctionGreaterEqual;
        sds.depthWriteEnabled = YES;
        shadowDepth_ = [device newDepthStencilStateWithDescriptor:sds];
    }

    sunDir_ = simd_make_float3(0, 1, 0);  // replaced every frame by FUEL's sun (fillAtmosphere)
}

void MeshRenderer::setGround(const GroundMaps& maps, const DpcFile& world) {
    // Engine settings read from the player's files: the post defaults and shadow table from FUEL.exe,
    // the clock scale PP371 from pp001.tsc.
    if (!engineSettingsRead_) {
        engineSettingsRead_ = true;
        loadSfxAtlas(world);
        {
            std::string err;
            auto atlas = sfx::replayAtlas(engineExePath(world), &err);
            if (!atlas.empty()) sfxTables_ = std::make_shared<const std::vector<sfx::AtlasEntry>>(std::move(atlas));
        }
        weatherFields_ = weatherFieldsFor(world);
        const std::string exe = engineExePath(world);
        readEngineRenderSettings(exe.c_str(), postDefaults_, shadowSettings_);
        const std::filesystem::path pp = std::filesystem::path(world.path()).parent_path().parent_path().parent_path() / "CommonComponent" /
                                         "Param" / "pp001.tsc";
        if (FILE* fp = std::fopen(pp.string().c_str(), "r")) {
            char line[256];
            while (std::fgets(line, sizeof line, fp)) {
                int row = 0;
                float v = 0;
                if (std::sscanf(line, "PP %d %f", &row, &v) == 2 && row == 371) {
                    clockScale = v;
                    break;
                }
            }
            std::fclose(fp);
        }
    }
    // Colour map (gamma-space bytes read as they are, mipmapped) and surface-type map (one byte per sample).
    MTLTextureDescriptor* td = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatRGBA8Unorm
                                                                                    width:maps.size
                                                                                   height:maps.size
                                                                                mipmapped:YES];
    groundColour_ = [device_ newTextureWithDescriptor:td];
    groundFar_ = simd_make_float4(maps.farColour[0], maps.farColour[1], maps.farColour[2], maps.decoded ? 1.0f : 0.0f);
    [groundColour_ replaceRegion:MTLRegionMake2D(0, 0, maps.size, maps.size) mipmapLevel:0 withBytes:maps.colour.data() bytesPerRow:maps.size * 4];
    {
        MTLTextureDescriptor* fd = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatRGBA8Unorm
                                                                                      width:kGroundTile
                                                                                     height:kGroundTile
                                                                                  mipmapped:YES];
        fd.textureType = MTLTextureType2DArray;
        fd.arrayLength = kGroundTileLevels;
        groundFine_ = [device_ newTextureWithDescriptor:fd];
        for (auto& r : groundFineRect_) r = simd_make_float4(0, 0, 1, 0);
        MTLTextureDescriptor* ud = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatRGBA16Uint
                                                                                      width:kGroundTile
                                                                                     height:kGroundTile
                                                                                  mipmapped:NO];
        ud.textureType = MTLTextureType2DArray;
        ud.arrayLength = kGroundTileLevels;
        groundTypes_ = [device_ newTextureWithDescriptor:ud];
        ud.pixelFormat = MTLPixelFormatRGBA32Float;
        groundDecal_ = [device_ newTextureWithDescriptor:ud];
        // Road decal textures (sDiffuse of ground.phl's decal path), by slot.
        const uint16_t enums[kDecalSlots] = {0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x34};
        for (int k = 0; k < kDecalSlots; ++k) {
            const uint32_t bm = genWorldEnumTexture(world, enums[k]);
            const DpcFile::Entry* e = bm != 0 ? world.find(bm) : nullptr;
            uint8_t alpha = 0;
            decalTex_[k] = e != nullptr ? uploadBitmap(world, *e, false, &alpha) : nil;
        }
    }
    td = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatR8Uint width:maps.size height:maps.size mipmapped:NO];
    groundSurface_ = [device_ newTextureWithDescriptor:td];
    [groundSurface_ replaceRegion:MTLRegionMake2D(0, 0, maps.size, maps.size) mipmapLevel:0 withBytes:maps.surface.data() bytesPerRow:maps.size];

    // Detail textures: one 256x256 DXT5 slice per surface type, taken from the
    // mip level of each source that is 256 wide (sources are 256-1024).
    constexpr uint32_t kSide = 256, kLevels = 9;
    td = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatBC3_RGBA width:kSide height:kSide mipmapped:YES];
    td.textureType = MTLTextureType2DArray;
    td.arrayLength = kSurfaceTypes;
    td.mipmapLevelCount = kLevels;
    groundDetail_ = [device_ newTextureWithDescriptor:td];
    for (int s = 0; s < kSurfaceTypes; ++s) {
        const DpcFile::Entry* e = world.find(maps.textures[size_t(s)]);
        std::optional<std::vector<uint8_t>> body;
        BitmapInfo bm;
        try {
            if (e != nullptr) {
                bm = parseBitmapLink(world.link(*e));
                body = world.body(*e);
            }
        } catch (const std::exception&) {
            body.reset();
        }
        const bool usable = body && bm.format == PixelFormat::DXT5 && bm.width >= kSide && bm.width == bm.height &&
                            checkBitmap(bm, uint32_t(body->size())).empty();
        const std::vector<uint32_t> sizes = usable ? bm.mipSizes() : std::vector<uint32_t>{};
        size_t offset = 0;
        uint32_t level = 0, w = usable ? bm.width : 0;
        while (usable && w > kSide) offset += sizes[level++], w >>= 1;
        std::vector<uint8_t> flat(16, 0);  // neutral block fallback: normal up, detail 0.5
        for (uint32_t m = 0; m < kLevels; ++m) {
            const uint32_t side = std::max(1u, kSide >> m), blocks = std::max(1u, side / 4);
            const uint8_t* src;
            if (usable && level + m < bm.mips) {
                src = body->data() + offset;
                offset += sizes[level + m];
            } else {
                // DXT5 block: alpha 0.5, colour (0.5, 1, 0.5) = flat normal with neutral detail.
                static const uint8_t kFlat[16] = {128, 128, 0, 0, 0, 0, 0, 0, 0xEF, 0x87, 0xEF, 0x87, 0, 0, 0, 0};
                flat.assign(size_t(blocks) * blocks * 16, 0);
                for (size_t b = 0; b < size_t(blocks) * blocks; ++b) std::memcpy(&flat[b * 16], kFlat, 16);
                src = flat.data();
            }
            [groundDetail_ replaceRegion:MTLRegionMake2D(0, 0, side, side)
                             mipmapLevel:m
                                   slice:NSUInteger(s)
                               withBytes:src
                             bytesPerRow:blocks * 16
                           bytesPerImage:size_t(blocks) * blocks * 16];
        }
    }
    id<MTLCommandQueue> q = [device_ newCommandQueue];
    id<MTLCommandBuffer> cb = [q commandBuffer];
    id<MTLBlitCommandEncoder> blit = [cb blitCommandEncoder];
    [blit generateMipmapsForTexture:groundColour_];
    [blit endEncoding];
    [cb commit];
    [cb waitUntilCompleted];
}


void MeshRenderer::setTerrain(const Heightfield& hf) {
    MTLTextureDescriptor* td = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatR16Unorm
                                                                                    width:hf.width
                                                                                   height:hf.height
                                                                                mipmapped:NO];
    heightTex_ = [device_ newTextureWithDescriptor:td];
    [heightTex_ replaceRegion:MTLRegionMake2D(0, 0, hf.width, hf.height) mipmapLevel:0 withBytes:hf.raw.data() bytesPerRow:hf.width * 2];
    terrainLod_.reset();
    chunkGpu_.clear();
    terrainData_ = hf;
    terrainInfo_ = simd_make_float4(hf.originX, hf.originZ, hf.spacing, hf.metresPerUnit * 65535.0f);
    terrainLod_ = std::make_unique<TerrainLod>(terrainData_);
    lastTerrainEye_ = simd_make_float3(1e9f, 0, 1e9f);
    if (terrainIndices_ == nil) {
        const std::vector<uint32_t>& idx = terrainChunkIndices();
        terrainIndices_ = [device_ newBufferWithBytes:idx.data() length:idx.size() * 4 options:MTLResourceStorageModeShared];
        terrainIndexCount_ = uint32_t(idx.size());
    }
}

id<MTLRenderPipelineState> MeshRenderer::pipeline(uint32_t stride, AlphaMode mode) {
    const uint64_t key = (uint64_t(stride) << 8) | uint64_t(mode);
    auto it = pipelines_.find(key);
    if (it != pipelines_.end()) return it->second;

    MTLRenderPipelineDescriptor* pd = [MTLRenderPipelineDescriptor new];
    pd.vertexFunction = [shaders_ newFunctionWithName:@"vsMain"];
    pd.fragmentFunction = [shaders_ newFunctionWithName:@"fsMain"];
    pd.vertexDescriptor = meshVertexDescriptor(stride);
    pd.rasterSampleCount = sampleCount_;
    pd.colorAttachments[0].pixelFormat = kRenderColorFormat;
    pd.depthAttachmentPixelFormat = kRenderDepthFormat;
    if (mode == AlphaMode::Test) pd.alphaToCoverageEnabled = sampleCount_ > 1;
    if (mode == AlphaMode::Blend) {
        auto* ca = pd.colorAttachments[0];
        ca.blendingEnabled = YES;
        ca.sourceRGBBlendFactor = MTLBlendFactorSourceAlpha;
        ca.destinationRGBBlendFactor = MTLBlendFactorOneMinusSourceAlpha;
        ca.sourceAlphaBlendFactor = MTLBlendFactorOne;
        ca.destinationAlphaBlendFactor = MTLBlendFactorOneMinusSourceAlpha;
    }
    NSError* err = nil;
    id<MTLRenderPipelineState> ps = [device_ newRenderPipelineStateWithDescriptor:pd error:&err];
    if (ps == nil) [NSException raise:@"pipeline" format:@"%@", err];
    pipelines_[key] = ps;
    return ps;
}

id<MTLRenderPipelineState> MeshRenderer::shadowPipeline(uint32_t stride, bool cut) {
    const uint32_t key = stride << 1 | (cut ? 1u : 0u);
    if (auto it = shadowPipelines_.find(key); it != shadowPipelines_.end()) return it->second;
    MTLRenderPipelineDescriptor* pd = [MTLRenderPipelineDescriptor new];
    pd.vertexFunction = [shaders_ newFunctionWithName:@"vsShadow"];
    if (cut) pd.fragmentFunction = [shaders_ newFunctionWithName:@"fsShadowCut"];
    pd.vertexDescriptor = meshVertexDescriptor(stride);
    pd.depthAttachmentPixelFormat = MTLPixelFormatDepth32Float;
    NSError* err = nil;
    id<MTLRenderPipelineState> ps = [device_ newRenderPipelineStateWithDescriptor:pd error:&err];
    if (ps == nil) [NSException raise:@"pipeline" format:@"%@", err];
    shadowPipelines_[key] = ps;
    return ps;
}

// Like the ground's detail maps, some world textures (rocks, cliffs) are pure
// grey around a mean of 0.5 and take their colour from the ground palette.
// Photographic grey textures always carry a little colour, so the test is
// strict: (nearly) every DXT colour endpoint must be grey (textureStats).
const MeshRenderer::TextureStats& MeshRenderer::textureStats(uint32_t bitmapId) {
    if (auto it = textureStats_.find(bitmapId); it != textureStats_.end()) return it->second;
    TextureStats st;
    if (auto hit = bitmapId ? library_.find(bitmapId, cls::Bitmap) : ResourceLibrary::Hit{}) {
        try {
            const BitmapInfo bi = parseBitmapLink(hit.file->link(*hit.entry));
            const auto body = hit.file->body(*hit.entry);
            const size_t block = bi.format == PixelFormat::DXT1 ? 8 : bi.format == PixelFormat::DXT5 ? 16 : 0;
            const size_t blocks = size_t(std::max(1u, bi.width / 4)) * std::max(1u, bi.height / 4);
            if (block && body && body->size() >= blocks * block) {
                size_t coloured = 0;
                simd_float3 sum = 0;
                for (size_t q = 0; q < blocks; ++q)
                    for (size_t c = 0; c < 2; ++c) {
                        const uint8_t* e = body->data() + q * block + (block - 8) + c * 2;
                        const int v = e[0] | (e[1] << 8), r = v >> 11, g = (v >> 5) & 63, b = v & 31;
                        if (std::abs(r - b) > 1 || std::abs(g - 2 * r) > 2) ++coloured;
                        sum += simd_make_float3(r / 31.0f, g / 63.0f, b / 31.0f);
                    }
                st.mean = sum / float(blocks * 2);
                st.greyDetail = coloured * 100 <= blocks * 2 && st.mean.y > 0.4f && st.mean.y < 0.6f;  // at most 1% off-grey
            }
        } catch (const std::exception&) {
        }
    }
    return textureStats_[bitmapId] = st;
}

id<MTLTexture> MeshRenderer::uploadBitmap(const DpcFile& file, const DpcFile::Entry& entry, bool srgb, uint8_t* alphaKind) {
    id<MTLTexture> result = nil;
    uint8_t alpha = 0;
        try {
            const BitmapInfo bm = parseBitmapLink(file.link(entry));
            const auto body = file.body(entry);
            if (body && checkBitmap(bm, uint32_t(body->size())).empty() && bm.type == BitmapInfo::Tex2D) {
                MTLPixelFormat pf = srgb ? MTLPixelFormatBGRA8Unorm_sRGB : MTLPixelFormatBGRA8Unorm;
                if (bm.format == PixelFormat::DXT1) pf = srgb ? MTLPixelFormatBC1_RGBA_sRGB : MTLPixelFormatBC1_RGBA;
                if (bm.format == PixelFormat::DXT3) pf = srgb ? MTLPixelFormatBC2_RGBA_sRGB : MTLPixelFormatBC2_RGBA;
                if (bm.format == PixelFormat::DXT5) pf = srgb ? MTLPixelFormatBC3_RGBA_sRGB : MTLPixelFormatBC3_RGBA;
                MTLTextureDescriptor* td = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:pf
                                                                                                width:bm.width
                                                                                               height:bm.height
                                                                                            mipmapped:bm.mips > 1];
                td.mipmapLevelCount = bm.mips;
                id<MTLTexture> t = [device_ newTextureWithDescriptor:td];
                const std::vector<uint32_t> sizes = bm.mipSizes();
                size_t offset = 0;
                uint32_t w = bm.width, h = bm.height;
                bool ok = true;
                std::vector<uint8_t> converted;
                for (uint32_t level = 0; level < bm.mips && ok; ++level) {
                    const uint8_t* src = body->data() + offset;
                    if (bm.isBlockCompressed()) {
                        const uint32_t blockBytes = bm.format == PixelFormat::DXT1 ? 8 : 16;
                        [t replaceRegion:MTLRegionMake2D(0, 0, w, h)
                             mipmapLevel:level
                               withBytes:src
                             bytesPerRow:std::max(1u, (w + 3) / 4) * blockBytes];
                    } else if ((ok = toBgra8(bm.format, src, w, h, converted))) {
                        [t replaceRegion:MTLRegionMake2D(0, 0, w, h) mipmapLevel:level withBytes:converted.data() bytesPerRow:w * 4];
                    }
                    offset += sizes[level];
                    w = std::max(1u, w >> 1);
                    h = std::max(1u, h >> 1);
                }
                if (ok) {
                    result = t;
                    alpha = bm.alphaKind;
                }
            }
        } catch (const std::exception&) {
            // Textures that do not decode are treated as missing.
        }
        *alphaKind = alpha;
    return result;
}

id<MTLTexture> MeshRenderer::texture(uint32_t bitmapId, bool srgb, uint8_t* alphaKind) {
    *alphaKind = 0;
    if (bitmapId == 0) return nil;
    const uint64_t key = (uint64_t(bitmapId) << 1) | (srgb ? 1u : 0u);
    if (auto it = textures_.find(key); it != textures_.end()) {
        *alphaKind = textureAlpha_[bitmapId];
        return it->second;
    }
    id<MTLTexture> result = nil;
    uint8_t alpha = 0;
    if (auto hit = library_.find(bitmapId, cls::Bitmap)) result = uploadBitmap(*hit.file, *hit.entry, srgb, &alpha);
    textures_[key] = result;
    textureAlpha_[bitmapId] = alpha;
    *alphaKind = alpha;
    return result;
}

MeshRenderer::GpuMaterial MeshRenderer::makeMaterial(uint32_t materialId, uint32_t diffuseOverride, uint32_t specularOverride) {
    GpuMaterial gm;
    gm.textures.fill(white_);
    auto hit = library_.find(materialId, cls::Material);
    if (!hit) return gm;
    try {
        const auto body = hit.file->body(*hit.entry);
        if (!body) return gm;
        Material m = parseMaterialBody(*body);
        if (diffuseOverride) m.textures[kDiffuse] = diffuseOverride;
        if (specularOverride) m.textures[kSpecular] = specularOverride;
        const std::vector<uint8_t> link = hit.file->link(*hit.entry);
        uint32_t name = 0;
        if (link.size() >= 4) std::memcpy(&name, link.data(), 4);

        uint32_t diffuseTex = m.textures[kDiffuse];
        gm.code = m.flags & 0x1F;
        for (int lod = 1; lod <= 3; ++lod) {
            if (name != liveryMaterialName(lod)) continue;
            if (paint_.livery[size_t(lod - 1)] != 0)
                diffuseTex = paint_.livery[size_t(lod - 1)];
            else
                gm.features |= PaintOnly;
        }
        uint8_t alphaKind = 0, ignored = 0;
        // Gamma space like the game: colour textures are read without sRGB decoding.
        if (id<MTLTexture> t = texture(diffuseTex, false, &alphaKind)) gm.textures[kDiffuse] = t;
        const TextureStats& ts = textureStats(diffuseTex);
        if (ts.greyDetail) gm.features |= GroundTint;
        const struct { Slot slot; bool srgb; uint32_t feature; } maps[] = {
            {kOcclusion, false, HasOcclusion}, {kNormal, false, HasNormal},     {kSpecular, false, HasSpecular},
            {kNormalLocal, false, HasNormalLocal}, {kDirt, false, HasDirt},
        };
        for (const auto& mp : maps) {
            if (id<MTLTexture> t = texture(m.textures[size_t(mp.slot)], mp.srgb, &ignored)) {
                gm.textures[size_t(mp.slot)] = t;
                gm.features |= mp.feature;
            }
        }
        gm.diffuse = simd_make_float4(m.diffuse[0], m.diffuse[1], m.diffuse[2], m.alpha);
        gm.specular = simd_make_float4(m.specular[0], m.specular[1], m.specular[2], m.specular[3]);
        gm.emissive = simd_make_float4(m.emissive[0], m.emissive[1], m.emissive[2], 0);
        gm.fresnel = simd_make_float2(m.fresnel[0], m.fresnel[1]);
        // Texture mask bit 8 turns on reflections; that path ignores the material colour.
        if (m.textureMask & 0x100) {
            gm.features |= Reflective;
            gm.diffuse = simd_make_float4(1, 1, 1, m.alpha);
        }
        // The engine derives the blend mode from the diffuse bitmap's alpha
        // kind and the material opacity; flags 0x180000 force alpha test.
        // Painted car parts (codes 17 and 19) are always opaque.
        if (gm.code == 17 || gm.code == 19)
            gm.alpha = AlphaMode::Opaque;
        else if (alphaKind == 2 || m.alpha < 1.0f)
            gm.alpha = AlphaMode::Blend;
        else if (alphaKind == 1 || (m.flags & 0x180000))
            gm.alpha = AlphaMode::Test;
        // (P28: the port's old lift of dark green cut-outs is gone: in FUEL's gamma-space lighting the leaf
        // textures are read as stored and lit by the strong sky light, as in the game.)
        gm.lit = (m.flags & 0x3000) == 0;
        gm.skip = m.skipDraw();
        gm.samplerIndex = (m.clampU() ? 1u : 0u) | (m.clampV() ? 2u : 0u);
    } catch (const std::exception&) {
    }
    return gm;
}

void MeshRenderer::setPaint(const Paint& paint) {
    if (paint.livery != paint_.livery) meshes_.clear();  // materials bake in the livery texture
    paint_ = paint;
}

const MeshRenderer::GpuMesh* MeshRenderer::loadMesh(uint32_t meshId, std::string* error) {
    if (auto it = bakedMeshes_.find(meshId); it != bakedMeshes_.end()) return it->second.get();
    if (auto it = meshes_.find(meshId); it != meshes_.end()) return it->second.get();
    std::unique_ptr<GpuMesh> gm = buildMesh(meshId, error);
    return gm ? (meshes_[meshId] = std::move(gm)).get() : nullptr;
}

const MeshRenderer::GpuMesh* MeshRenderer::loadMesh(uint32_t meshId, std::string* error, const std::array<uint32_t, 3>& livery) {
    if (auto it = bakedMeshes_.find(meshId); it != bakedMeshes_.end()) return it->second.get();
    const auto key = std::make_pair(meshId, livery);
    if (auto it = liveryMeshes_.find(key); it != liveryMeshes_.end()) return it->second.get();
    // makeMaterial substitutes the current paint's livery.
    const std::array<uint32_t, 3> own = paint_.livery;
    paint_.livery = livery;
    std::unique_ptr<GpuMesh> gm = buildMesh(meshId, error);
    paint_.livery = own;
    return gm ? (liveryMeshes_[key] = std::move(gm)).get() : nullptr;
}

std::unique_ptr<MeshRenderer::GpuMesh> MeshRenderer::buildMesh(uint32_t meshId, std::string* error) {
    auto hit = library_.find(meshId, cls::Mesh);
    if (!hit) {
        *error = "no such Mesh_Z";
        return nullptr;
    }
    Mesh mesh;
    MeshStats stats;
    try {
        const MeshLink link = parseMeshLink(hit.file->link(*hit.entry));
        const auto body = hit.file->body(*hit.entry);
        if (!body) {
            *error = "mesh has no body";
            return nullptr;
        }
        mesh = parseMeshBody(*body, link.flags);
        const auto problems = checkMesh(mesh, &stats);
        if (!problems.empty()) {
            *error = problems.front();
            return nullptr;
        }
    } catch (const std::exception& e) {
        *error = e.what();
        return nullptr;
    }

    auto gm = std::make_unique<GpuMesh>();
    for (const VertexBuffer& vb : mesh.vbs) {
        gm->vbs.push_back(vb.data.empty() ? nil
                                          : [device_ newBufferWithBytes:vb.data.data()
                                                                 length:vb.data.size()
                                                                options:MTLResourceStorageModeShared]);
        gm->vbStrides.push_back(vb.stride);
    }
    GpuMaterial none;
    none.textures.fill(white_);
    gm->materials.push_back(none);
    for (uint32_t id : mesh.materials) gm->materials.push_back(makeMaterial(id));

    std::vector<uint32_t> indices;
    for (const Batch& b : mesh.batches) {
        if (b.vb >= mesh.vbs.size() || mesh.vbs[b.vb].stride < kMinDrawableStride) continue;
        const uint32_t mat = b.material < mesh.materials.size() ? b.material + 1u : 0u;
        if (gm->materials[mat].skip) continue;
        const std::vector<uint32_t> tris = batchTriangles(mesh, b);
        Draw d{b.vb, uint32_t(indices.size()), uint32_t(tris.size()), mat, false};
        // Sprite batches (leaves, impostors): vertex type 7 whose triangle corners share a position.
        if ((b.vertexType == 7 || mesh.vbs[b.vb].count <= 8) && tris.size() >= 3) {
            const VertexBuffer& vb = mesh.vbs[b.vb];
            d.sprite = std::memcmp(&vb.data[size_t(tris[0]) * vb.stride], &vb.data[size_t(tris[1]) * vb.stride], 12) == 0;
        }
        indices.insert(indices.end(), tris.begin(), tris.end());
        // A distant tree / bush card (a lone 4-vertex sprite) is cut out and writes
        // depth: thousands of overlapping blended cards cannot be sorted.
        // Foliage too: unsorted blended leaves let a farther tree show through a nearer one.
        if (gm->materials[mat].alpha == AlphaMode::Blend &&
            ((d.sprite && mesh.vbs.size() == 1 && mesh.vbs[b.vb].count == 4) || foliage_.count(meshId))) {
            gm->materials[mat].alpha = AlphaMode::Test;
            gm->materials[mat].features |= Foliage;
        }
        gm->draws[uint32_t(gm->materials[mat].alpha)].push_back(d);
        gm->triangles += uint32_t(tris.size() / 3);
    }
    gm->indices = indices.empty() ? nil
                                  : [device_ newBufferWithBytes:indices.data()
                                                         length:indices.size() * 4
                                                        options:MTLResourceStorageModeShared];
    gm->boundsMin = simd_make_float3(stats.boundsMin[0], stats.boundsMin[1], stats.boundsMin[2]);
    gm->boundsMax = simd_make_float3(stats.boundsMax[0], stats.boundsMax[1], stats.boundsMax[2]);
    return gm;
}

bool MeshRenderer::addBakedMesh(uint32_t key, const BakedMesh& mesh, std::string* error) {
    if (mesh.vertices.empty() || mesh.indices.empty() || mesh.vertices.size() % BakedMesh::kStride != 0) {
        *error = "empty baked mesh";
        return false;
    }
    const uint32_t vertexCount = mesh.vertexCount();
    for (uint32_t i : mesh.indices)
        if (i >= vertexCount) {
            *error = "baked mesh index out of range";
            return false;
        }
    for (const BakedBatch& b : mesh.batches)
        if (b.material >= mesh.materials.size() || uint64_t(b.firstIndex) + b.indexCount > mesh.indices.size()) {
            *error = "baked mesh batch out of range";
            return false;
        }
    auto gm = std::make_unique<GpuMesh>();
    gm->vbs.push_back([device_ newBufferWithBytes:mesh.vertices.data() length:mesh.vertices.size() options:MTLResourceStorageModeShared]);
    gm->vbStrides.push_back(BakedMesh::kStride);
    gm->indices = [device_ newBufferWithBytes:mesh.indices.data() length:mesh.indices.size() * 4 options:MTLResourceStorageModeShared];
    GpuMaterial none;
    none.textures.fill(white_);
    gm->materials.push_back(none);
    for (const BakedMaterial& m : mesh.materials) gm->materials.push_back(makeMaterial(m.materialId, m.diffuseTex, m.specularTex));
    for (const BakedBatch& b : mesh.batches) {
        const uint32_t mat = b.material + 1;
        if (gm->materials[mat].skip) continue;
        gm->draws[uint32_t(gm->materials[mat].alpha)].push_back({0, b.firstIndex, b.indexCount, mat, false});
        gm->triangles += b.indexCount / 3;
    }
    simd_float3 lo = simd_make_float3(1e30f, 1e30f, 1e30f), hi = -lo;
    for (uint32_t i = 0; i < vertexCount; ++i) {
        const auto p = mesh.position(i);
        const simd_float3 v = simd_make_float3(p[0], p[1], p[2]);
        lo = simd_min(lo, v);
        hi = simd_max(hi, v);
    }
    gm->boundsMin = lo;
    gm->boundsMax = hi;
    bakedMeshes_[key] = std::move(gm);
    return true;
}

void MeshRenderer::updateBakedVertices(uint32_t key, const std::vector<uint8_t>& vertices) {
    auto it = bakedMeshes_.find(key);
    if (it == bakedMeshes_.end() || vertices.empty() || it->second->vbs.empty()) return;
    GpuMesh& gm = *it->second;
    if (gm.vbs[0].length != vertices.size()) return;
    // A fresh buffer each frame: the command buffers still in flight keep the previous one alive.
    gm.vbs[0] = [device_ newBufferWithBytes:vertices.data() length:vertices.size() options:MTLResourceStorageModeShared];
    simd_float3 lo = simd_make_float3(1e30f, 1e30f, 1e30f), hi = -lo;
    for (size_t o = 0; o + BakedMesh::kStride <= vertices.size(); o += BakedMesh::kStride) {
        float p[3];
        std::memcpy(p, &vertices[o], 12);
        const simd_float3 v = simd_make_float3(p[0], p[1], p[2]);
        lo = simd_min(lo, v), hi = simd_max(hi, v);
    }
    gm.boundsMin = lo, gm.boundsMax = hi;
}

bool MeshRenderer::setMesh(uint32_t meshId, std::string* error) {
    return setScene({MeshInstance{meshId, matrix_identity_float4x4}}, error);
}

bool MeshRenderer::setScene(const std::vector<MeshInstance>& instances, std::string* error) {
    instances_.clear();
    inputToInstance_.assign(instances.size(), -1);
    triangles_ = 0;
    error->clear();
    simd_float3 lo = simd_make_float3(1e30f, 1e30f, 1e30f), hi = -lo;
    for (size_t k = 0; k < instances.size(); ++k) {
        const MeshInstance& in = instances[k];
        std::string err;
        const GpuMesh* gm = loadMesh(in.meshId, &err);
        if (gm == nullptr) {
            char buf[32];
            std::snprintf(buf, sizeof buf, "%08x", in.meshId);
            *error += (error->empty() ? "" : "; ") + std::string(buf) + ": " + err;
            continue;
        }
        if (gm->indices == nil) continue;
        const simd_float4x4 world = simd_mul(placement, in.world);
        inputToInstance_[k] = int(instances_.size());
        instances_.push_back({gm, world});
        triangles_ += gm->triangles;
        for (int c = 0; c < 8; ++c) {
            const simd_float3 corner = simd_make_float3((c & 1) ? gm->boundsMax.x : gm->boundsMin.x,
                                                        (c & 2) ? gm->boundsMax.y : gm->boundsMin.y,
                                                        (c & 4) ? gm->boundsMax.z : gm->boundsMin.z);
            const simd_float4 w = simd_mul(world, simd_make_float4(corner, 1.0f));
            lo = simd_min(lo, w.xyz);
            hi = simd_max(hi, w.xyz);
        }
    }
    if (instances_.empty()) {
        if (error->empty()) *error = "nothing to draw";
        return false;
    }
    sceneMin_ = lo;
    sceneMax_ = hi;
    center_ = (lo + hi) * 0.5f;
    radius_ = std::max(0.05f, simd_length(hi - lo) * 0.5f);
    return true;
}

size_t MeshRenderer::setWorldInstances(const std::vector<MeshInstance>& instances) {
    worldInstances_.clear();
    std::unordered_map<const GpuMesh*, std::vector<simd_float4x4>> byMesh;
    for (const MeshInstance& in : instances) {
        std::string err;
        const GpuMesh* gm = loadMesh(in.meshId, &err);
        if (gm == nullptr || gm->indices == nil) continue;
        worldInstances_.push_back({gm, in.world});
        byMesh[gm].push_back(in.world);
    }
    worldGroups_.clear();
    for (const auto& [gm, worlds] : byMesh)
        worldGroups_.push_back({gm, [device_ newBufferWithBytes:worlds.data() length:worlds.size() * sizeof(simd_float4x4)
                                                        options:MTLResourceStorageModeShared],
                                uint32_t(worlds.size())});
    return worldInstances_.size();
}

size_t MeshRenderer::setLodObjects(const std::vector<PlacedMesh>& objects, int set) {
    std::vector<LodObject>& list = lodObjects_[size_t(set & 1)];
    list.clear();
    list.reserve(objects.size());
    size_t drawable = 0;
    for (const PlacedMesh& p : objects) {
        LodObject o;
        o.p = p;
        for (size_t c = 0; c < 4; ++c) o.world.columns[c] = simd_make_float4(p.world[c * 4], p.world[c * 4 + 1], p.world[c * 4 + 2], p.world[c * 4 + 3]);
        bool any = false;
        for (uint32_t id : p.levels) {
            std::string err;
            const GpuMesh* gm = loadMesh(id, &err);
            if (gm != nullptr && gm->indices == nil) gm = nullptr;
            o.levels.push_back(gm);
            any = any || gm != nullptr;
        }
        drawable += any;
        list.push_back(std::move(o));  // kept in input order (setLodHidden indexes)
    }
    if (set == 0) lodHidden_.assign(list.size(), 0);
    return drawable;
}

// Per frame: FUEL's Lod_Z::Draw level and fade for every placed object (world.cpp lodLevel /
// placementAlpha), grouped by mesh into one buffer per frame.
void MeshRenderer::evaluateLodObjects(simd_float3 eye) {
    const auto t0 = std::chrono::steady_clock::now();
    lodVisible_.clear();
    lodGroups_.clear();
    lodDrawn_ = lodFading_ = 0;
    lodSetDrawn_[0] = lodSetDrawn_[1] = 0;
    if (lodObjects_[0].empty() && lodObjects_[1].empty()) return;
    std::unordered_map<const GpuMesh*, std::vector<simd_float4x4>> byMesh;
    for (size_t set = 0; set < 2; ++set)
    for (size_t i = 0; i < lodObjects_[set].size(); ++i) {
        const LodObject& o = lodObjects_[set][i];
        if (set == 0 && i < lodHidden_.size() && lodHidden_[i]) continue;
        const simd_float3 t = o.world.columns[3].xyz;  // node world T (node+0xd4 row 3)
        const float d = simd_distance(t, eye);
        const int level = lodLevel(o.p, d, lodView);
        const GpuMesh* gm = level < int(o.levels.size()) ? o.levels[size_t(level)] : nullptr;
        const float a = placementAlpha(o.p, level, d, lodView);
        if (gm == nullptr) continue;
        if (!(a >= 0.01f)) continue;
        simd_float4x4 w = o.world;
        w.columns[0].w = a >= 1.0f ? 0.0f : 1.0f - a;
        lodFading_ += a < 1.0f;
        lodVisible_.push_back({gm, w});
        byMesh[gm].push_back(w);
        lodSetDrawn_[set]++;
    }
    lodDrawn_ = lodVisible_.size();
    static const bool trace = std::getenv("FUEL_LOD_TRACE") != nullptr;
    if (trace && frame_ % 30 == 0)
        std::printf("lod frame %llu: %zu + %zu objects, %zu + %zu drawn, %zu fading, %.3f ms\n", (unsigned long long)frame_, lodObjects_[0].size(),
                    lodObjects_[1].size(), lodSetDrawn_[0], lodSetDrawn_[1], lodFading_,
                    std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - t0).count());
    const size_t bytes = std::max<size_t>(1, lodVisible_.size()) * sizeof(simd_float4x4);
    id<MTLBuffer> buf = [device_ newBufferWithLength:bytes options:MTLResourceStorageModeShared];
    auto* dst = static_cast<simd_float4x4*>(buf.contents);
    size_t at = 0;
    for (const auto& [gm, worlds] : byMesh) {
        std::memcpy(dst + at, worlds.data(), worlds.size() * sizeof(simd_float4x4));
        lodGroups_.push_back({gm, buf, uint32_t(worlds.size()), uint32_t(at * sizeof(simd_float4x4))});
        at += worlds.size();
    }
}

size_t MeshRenderer::setPropInstances(const std::vector<MeshInstance>& instances) {
    propInstances_.clear();
    std::unordered_map<const GpuMesh*, std::vector<simd_float4x4>> byMesh;
    for (const MeshInstance& in : instances) {
        std::string err;
        const GpuMesh* gm = loadMesh(in.meshId, &err);
        if (gm == nullptr || gm->indices == nil) continue;
        propInstances_.push_back({gm, in.world});
        byMesh[gm].push_back(in.world);
    }
    propGroups_.clear();
    for (const auto& [gm, worlds] : byMesh)
        propGroups_.push_back({gm, [device_ newBufferWithBytes:worlds.data() length:worlds.size() * sizeof(simd_float4x4)
                                                       options:MTLResourceStorageModeShared],
                               uint32_t(worlds.size())});
    return propInstances_.size();
}

void MeshRenderer::setInstanceWorlds(const std::vector<simd_float4x4>& worlds) {
    for (size_t k = 0; k < worlds.size() && k < inputToInstance_.size(); ++k)
        if (inputToInstance_[k] >= 0) instances_[size_t(inputToInstance_[k])].world = worlds[k];
}

void MeshRenderer::setVehicles(const std::vector<std::vector<MeshInstance>>& parts, const std::vector<Paint>& paints) {
    vehicles_.clear();
    for (size_t v = 0; v < parts.size(); ++v) {
        const Paint paint = v < paints.size() ? paints[v] : paint_;
        OtherVehicle ov;
        ov.paint = paint.color;
        for (const MeshInstance& in : parts[v]) {
            std::string err;
            const GpuMesh* gm = loadMesh(in.meshId, &err, paint.livery);
            ov.inputToInstance.push_back(gm && gm->indices != nil ? int(ov.instances.size()) : -1);
            if (gm && gm->indices != nil) ov.instances.push_back({gm, in.world});
        }
        vehicles_.push_back(std::move(ov));
    }
}

void MeshRenderer::setVehicleWorlds(size_t k, const std::vector<simd_float4x4>& worlds) {
    if (k >= vehicles_.size()) return;
    OtherVehicle& ov = vehicles_[k];
    for (size_t i = 0; i < worlds.size() && i < ov.inputToInstance.size(); ++i)
        if (ov.inputToInstance[i] >= 0) ov.instances[size_t(ov.inputToInstance[i])].world = worlds[i];
}

simd_float4x4 MeshRenderer::sunViewProj() const {
    // Legacy single-map matrix (kept for FrameUniforms.sunViewProj; the shaders use the 4 slices).
    const simd_float3 up = std::fabs(sunDir_.y) > 0.99f ? simd_make_float3(0, 0, 1) : simd_make_float3(0, 1, 0);
    const float r = shadowHalfSize() / 1.05f;
    const simd_float3 eye = center_ + sunDir_ * (r * 3.0f);
    return simd_mul(orthoRH(r * 1.05f, r * 0.5f, r * 6.0f), lookAtRH(eye, center_, up));
}

double MeshRenderer::timeOfDay() const {
    if (forcedTimeOfDay >= 0) return forcedTimeOfDay;  // ForceDayTime
    // WeatherManager_G 0x5b89e0: t = game clock (s) x PP371 + 28800 (08:00).
    const double clock = gameClock >= 0 ? gameClock : std::chrono::duration<double>(std::chrono::steady_clock::now() - epoch_).count();
    return clock * double(clockScale) + 28800.0;
}

namespace {

simd_float4 f4(const F3& v, float w) { return simd_make_float4(v[0], v[1], v[2], w); }

using Mat4d = std::array<double, 16>;  // row-major
Mat4d mul(const Mat4d& a, const Mat4d& b) {
    Mat4d r{};
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) {
            double v = 0;
            for (int k = 0; k < 4; ++k) v += a[size_t(i * 4 + k)] * b[size_t(k * 4 + j)];
            r[size_t(i * 4 + j)] = v;
        }
    return r;
}
simd_float4x4 toSimd(const Mat4d& m) {  // simd is column-major
    simd_float4x4 r;
    for (int c = 0; c < 4; ++c)
        r.columns[c] = simd_make_float4(float(m[size_t(c)]), float(m[size_t(4 + c)]), float(m[size_t(8 + c)]), float(m[size_t(12 + c)]));
    return r;
}

}  // namespace

void MeshRenderer::fillAtmosphere(FrameUniforms& fu, simd_float3 eye, const simd_float4x4&, float, float, float) {
    AtmosphereInput in;
    in.timeOfDay = timeOfDay();
    in.weather = weather;
    // FUEL's weather at the camera (WeatherManager_G 0x5b89e0 -> apply 0x5bba30 -> hook 0x6409d0), unless --weather.
    if (weatherFromWorld && weatherFields_ && weatherFields_->valid()) {
        weatherNow_ = weatherFields_->sample(eye.x, eye.y, eye.z);
        const Weather h = weatherHook(weatherNow_, eye.y, weatherFields_->P(5202), weatherFields_->P(5203));
        in.weather = h;
    }
    in.brightness = brightness;
    in.camera = {eye.x, eye.y, eye.z};
    atmo_ = computeAtmosphere(in);
    const Atmosphere& a = atmo_;
    sunDir_ = simd_make_float3(a.lightDir[0], a.lightDir[1], a.lightDir[2]);
    fu.sunDir = simd_make_float4(sunDir_, 1.0f);
    fu.lightDir = f4(a.lightDir, a.invIntensity);
    fu.skyDir = f4(a.skyDir, 0);
    fu.sunColour = f4(a.sunColour, 1);
    fu.dlight = f4(a.dlightColor, a.blueShift[3]);
    fu.skyColour = f4(a.dskyColor, 0);
    fu.skyAmbient = f4(a.dskyAmbient, 0);
    fu.sunRaw = f4(a.sunRaw, a.day ? 1.0f : 0.0f);
    for (int i = 0; i < 11; ++i) fu.scat[i] = simd_make_float4(a.c[size_t(i)][0], a.c[size_t(i)][1], a.c[size_t(i)][2], a.c[size_t(i)][3]);
    // FUEL's view space: camera-relative, +z forward (WorldToViewVertex).
    const simd_float3 r = fu.camRight.xyz, u = fu.camUp.xyz, fwd = simd_cross(fu.camUp.xyz, fu.camRight.xyz);
    fu.view = simd_matrix(simd_make_float4(r.x, u.x, fwd.x, 0), simd_make_float4(r.y, u.y, fwd.y, 0), simd_make_float4(r.z, u.z, fwd.z, 0),
                          simd_make_float4(-simd_dot(r, eye), -simd_dot(u, eye), -simd_dot(fwd, eye), 1));
}

// FUEL's cascaded shadow map (p26-work/shadows.md S2, ShadowMap_RenderSlices 0x7566b0 / FitOrtho 0x753740).
void MeshRenderer::encodeShadows(id<MTLCommandBuffer> cb, FrameUniforms& fu, const std::vector<std::shared_ptr<const TerrainChunk>>& chunks) {
    const ShadowSettings& ss = shadowSettings_;
    const double W = double(kShadowMapSize);
    fu.shadowSplit = simd_make_float4(ss.splits[0], ss.splits[1], ss.splits[2], ss.splits[3]);
    fu.shadowSize = simd_make_float4(float(W), float(1 / W), float(1 / W), float(W));
    // Slice 0 every frame, 1..3 in turn (all four on the first frame).
    unsigned mask;
    if (shadowCounter_ < 0) {
        mask = 0xF;
        shadowCounter_ = 0;
    } else {
        shadowCounter_ = (shadowCounter_ + 1) % 3;
        mask = (1u << (shadowCounter_ + 1)) | 1u;
    }
    for (int i = 0; i < 4; ++i)
        if (!sliceValid_[size_t(i)]) mask |= 1u << i;

    // Camera: eye, axes and the projection (from the frame's matrices).
    const simd_float3 eyeF = fu.eye.xyz;
    const double ex = eyeF.x, ey = eyeF.y, ez = eyeF.z;
    const simd_float3 R = fu.camRight.xyz, U = fu.camUp.xyz, F = simd_cross(fu.camUp.xyz, fu.camRight.xyz);
    const simd_float4x4 proj = simd_mul(fu.viewProj, simd_inverse(simd_mul(simd_matrix(simd_make_float4(1, 0, 0, 0), simd_make_float4(0, 1, 0, 0),
                                                                                         simd_make_float4(0, 0, -1, 0), simd_make_float4(0, 0, 0, 1)),
                                                                             fu.view)));
    const double tanY = 1.0 / double(proj.columns[1].y), tanX = 1.0 / double(proj.columns[0].x);
    const double zn = double(fu.fog.z);
    // Light basis: towards the sun L, right = normalize(cross(-L, up)) (fallback -x), up = cross(right, -L).
    const double L[3] = {fu.lightDir.x, fu.lightDir.y, fu.lightDir.z};
    double rt[3] = {L[2], 0, -L[0]};  // cross(-L, (0,1,0))
    double rl = std::sqrt(rt[0] * rt[0] + rt[2] * rt[2]);
    if (rl * rl <= 0.001) rt[0] = -1, rt[1] = 0, rt[2] = 0, rl = 1;
    for (double& v : rt) v /= rl;
    // up = cross(-L, right)
    const double up[3] = {-(L[1] * rt[2] - L[2] * rt[1]), -(L[2] * rt[0] - L[0] * rt[2]), -(L[0] * rt[1] - L[1] * rt[0])};

    // Receiver matrices need view -> world (double).
    const Mat4d invView = {R.x, U.x, F.x, ex, R.y, U.y, F.y, ey, R.z, U.z, F.z, ez, 0, 0, 0, 1};
    const Mat4d tex = {0.5, 0, 0, 0.5, 0, -0.5, 0, 0.5, 0, 0, 1, 0, 0, 0, 0, 1};

    struct Box { double cx, cy, hx, hy, minH; };
    std::array<Box, 4> boxes{};
    for (int i = 3; i >= 0; --i) {
        if (!(mask & (1u << i))) continue;
        const double nearZ = i == 0 ? zn : ss.splits[size_t(i - 1)], farZ = ss.splits[size_t(i)];
        double minX = 1e30, maxX = -1e30, minY = 1e30, maxY = -1e30, minH = 1e30;
        for (int c = 0; c < 8; ++c) {
            const double z = (c & 4) ? farZ : nearZ, x = ((c & 1) ? 1 : -1) * z * tanX, y = ((c & 2) ? 1 : -1) * z * tanY;
            const double p[3] = {ex + R.x * x + U.x * y + F.x * z, ey + R.y * x + U.y * y + F.y * z, ez + R.z * x + U.z * y + F.z * z};
            const double lx = p[0] * rt[0] + p[1] * rt[1] + p[2] * rt[2], ly = p[0] * up[0] + p[1] * up[1] + p[2] * up[2],
                         lh = p[0] * L[0] + p[1] * L[1] + p[2] * L[2];
            minX = std::min(minX, lx), maxX = std::max(maxX, lx), minY = std::min(minY, ly), maxY = std::max(maxY, ly), minH = std::min(minH, lh);
        }
        const double p7 = i == 0 ? 1.0 : 0.95;  // border factor
        const double sx = 2.0 / ((2.0 - p7) * (maxX - minX)), sy = 2.0 / ((2.0 - p7) * (maxY - minY));
        const double cx = (minX + maxX) * 0.5, cy = (minY + maxY) * 0.5, ZE = ss.zExtent;
        // Camera ZExtent towards the sun from just behind the box (maxZ + 0.1 away from the sun):
        // z_clip = 1 - zLight / ZExtent = (h - minH + 0.1) / ZExtent.
        sliceLvp_[size_t(i)] = {sx * rt[0], sx * rt[1], sx * rt[2], -sx * cx, sy * up[0], sy * up[1], sy * up[2], -sy * cy,
                                L[0] / ZE, L[1] / ZE, L[2] / ZE, (-minH + 0.1) / ZE, 0, 0, 0, 1};
        sliceBias_[size_t(i)] = float((farZ - nearZ) * ss.zbias[size_t(i)] * ss.zbiasAdd);
        sliceValid_[size_t(i)] = true;
        boxes[size_t(i)] = {cx, cy, 1.0 / sx, 1.0 / sy, minH};
    }
    for (int i = 0; i < 4; ++i) fu.shadowMat[i] = toSimd(mul(tex, mul(sliceLvp_[size_t(i)], invView)));

    for (int i = 3; i >= 0; --i) {
        if (!(mask & (1u << i))) continue;
        const Box& bx = boxes[size_t(i)];
        auto visible = [&](double px, double py, double pz, double rad) {
            const double lx = px * rt[0] + py * rt[1] + pz * rt[2], ly = px * up[0] + py * up[1] + pz * up[2], lh = px * L[0] + py * L[1] + pz * L[2];
            return std::fabs(lx - bx.cx) <= bx.hx + rad && std::fabs(ly - bx.cy) <= bx.hy + rad && lh >= bx.minH - rad &&
                   lh <= bx.minH + ss.zExtent + rad;
        };
        const float texelHalf = float(std::max(bx.hx, bx.hy) / W);
        // Casters grouped by mesh: one instanced draw each.
        std::unordered_map<const GpuMesh*, std::vector<simd_float4x4>> groups;
        auto consider = [&](const Instance& inst) {
            const GpuMesh& gm = *inst.mesh;
            const simd_float3 c = (gm.boundsMin + gm.boundsMax) * 0.5f;
            const simd_float4 w = simd_mul(inst.world, simd_make_float4(c, 1));
            const float sc = std::max(simd_length(inst.world.columns[0].xyz), std::max(simd_length(inst.world.columns[1].xyz), simd_length(inst.world.columns[2].xyz)));
            const float size = simd_length(gm.boundsMax - gm.boundsMin) * 0.5f * sc, rad = size + 2.0f;
            // Port: casters smaller than one shadow texel of the slice are skipped (their shadow would not show).
            if (size < texelHalf * 2.0f) return;
            if (visible(w.x, w.y, w.z, rad)) groups[&gm].push_back(inst.world);
        };
        for (const Instance& inst : instances_) consider(inst);
        for (const OtherVehicle& ov : vehicles_)
            if (!ov.instances.empty() && simd_distance(ov.instances[0].world.columns[3].xyz, center_) <= kVehicleDrawDistance)
                for (const Instance& inst : ov.instances) consider(inst);
        for (const Instance& inst : worldInstances_) consider(inst);
        for (const Instance& inst : propInstances_) consider(inst);
        for (const Instance& inst : lodVisible_) consider(inst);

        MTLRenderPassDescriptor* rp = [MTLRenderPassDescriptor renderPassDescriptor];
        rp.depthAttachment.texture = shadowMap_;
        rp.depthAttachment.slice = NSUInteger(i);
        rp.depthAttachment.loadAction = MTLLoadActionClear;
        rp.depthAttachment.clearDepth = 0.0;  // reversed depth
        rp.depthAttachment.storeAction = MTLStoreActionStore;
        id<MTLRenderCommandEncoder> enc = [cb renderCommandEncoderWithDescriptor:rp];
        [enc setDepthStencilState:shadowDepth_];
        [enc setCullMode:MTLCullModeNone];
        ShadowUniforms su{};
        su.lightViewProj = toSimd(sliceLvp_[size_t(i)]);
        su.light = simd_make_float4(float(L[0]), float(L[1]), float(L[2]), sliceBias_[size_t(i)]);
        su.eye = fu.eye;
        su.lightRight = simd_make_float4(float(rt[0]), float(rt[1]), float(rt[2]), 0);
        su.lightUp = simd_make_float4(float(up[0]), float(up[1]), float(up[2]), 0);
        [enc setVertexBytes:&su length:sizeof su atIndex:1];

        size_t total = 0;
        for (const auto& g : groups) total += g.second.size();
        if (total > 0) {
            id<MTLBuffer> mats = [device_ newBufferWithLength:total * sizeof(simd_float4x4) options:MTLResourceStorageModeShared];
            auto* out = static_cast<simd_float4x4*>(mats.contents);
            size_t at = 0;
            for (const auto& [gm, worlds] : groups) {
                std::memcpy(out + at, worlds.data(), worlds.size() * sizeof(simd_float4x4));
                [enc setVertexBuffer:mats offset:at * sizeof(simd_float4x4) atIndex:2];
                at += worlds.size();
                for (uint32_t pass = 0; pass < 3; ++pass) {
                    for (const Draw& d : gm->draws[pass]) {
                        const GpuMaterial& m = gm->materials[d.material];
                        if (!m.lit) continue;  // unlit / additive materials do not cast (pass 7 skips flag 0x1000)
                        const bool cut = pass != 0;
                        [enc setRenderPipelineState:shadowPipeline(gm->vbStrides[d.vb], cut)];
                        [enc setVertexBuffer:gm->vbs[d.vb] offset:0 atIndex:0];
                        const uint32_t sprite = d.sprite ? 1u : 0u;
                        [enc setVertexBytes:&sprite length:sizeof sprite atIndex:3];
                        if (cut) {
                            const uint32_t half = (m.code == 17 || m.code == 20) ? 1u : 0u;
                            [enc setFragmentBytes:&half length:sizeof half atIndex:0];
                            [enc setFragmentTexture:m.textures[kDiffuse] atIndex:0];
                            [enc setFragmentSamplerState:samplers_[m.samplerIndex] atIndex:0];
                        }
                        [enc drawIndexedPrimitives:MTLPrimitiveTypeTriangle
                                        indexCount:d.indexCount
                                         indexType:MTLIndexTypeUInt32
                                       indexBuffer:gm->indices
                                 indexBufferOffset:size_t(d.firstIndex) * 4
                                     instanceCount:worlds.size()];
                    }
                }
            }
        }
        // Terrain chunks cast too (ground.vhl vh_shadow).
        if (!chunks.empty() && terrainIndices_ != nil) {
            [enc setRenderPipelineState:terrainShadowPipeline_];
            for (const auto& c : chunks) {
                auto git = chunkGpu_.find(c->key);
                if (git == chunkGpu_.end() || git->second.vb == nil) continue;
                ChunkBounds& b = chunkBounds_[c->key];
                if (b.maxY <= b.minY) {
                    b.minY = 1e30f, b.maxY = -1e30f;
                    for (const TerrainVertex& v : c->verts) b.minY = std::min(b.minY, v.p0[1]), b.maxY = std::max(b.maxY, v.p0[1]);
                    b.maxY = std::max(b.maxY, b.minY + 0.01f);
                }
                const double rad = c->size * 0.7072 + (b.maxY - b.minY) * 0.5;
                if (!visible(c->cx, (b.minY + b.maxY) * 0.5, c->cz, rad)) continue;
                [enc setVertexBuffer:git->second.vb offset:0 atIndex:0];
                [enc drawIndexedPrimitives:MTLPrimitiveTypeTriangle
                                indexCount:terrainIndexCount_
                                 indexType:MTLIndexTypeUInt32
                               indexBuffer:terrainIndices_
                         indexBufferOffset:0];
            }
        }
        [enc endEncoding];
    }
    if (chunkBounds_.size() > 4 * chunkGpu_.size() + 256)
        for (auto it = chunkBounds_.begin(); it != chunkBounds_.end();)
            it = chunkGpu_.count(it->first) ? std::next(it) : chunkBounds_.erase(it);
}

void MeshRenderer::encode(id<MTLRenderCommandEncoder> enc, float aspect, const OrbitCamera& cam) {
    if (instances_.empty()) return;
    float dist = cam.distance * radius_;
    simd_float3 eye = center_ + dist * simd_make_float3(std::cos(cam.pitch) * std::sin(cam.yaw), std::sin(cam.pitch),
                                                        std::cos(cam.pitch) * std::cos(cam.yaw));
    if (cam.explicitPose) eye = cam.eye, dist = simd_distance(eye, center_);
    if (showTerrain && !terrainData_.raw.empty() && !cam.explicitPose) {
        // Keep the camera above the (smoothed, as drawn) ground, and lift it until
        // no hill between it and the target hides the target.
        const float clear = std::max(1.5f, radius_ * 0.3f);
        eye.y = std::max(eye.y, terrainData_.smoothHeightAt(eye.x, eye.z) + clear);
        for (int k = 1; k < 8; ++k) {
            const float t = float(k) / 8.0f;
            const simd_float3 p = eye + (center_ - eye) * t;
            const float need = terrainData_.smoothHeightAt(p.x, p.z) + 0.6f - p.y;
            if (need > 0) eye.y += need / (1.0f - t);  // raise the eye so the line clears this point
        }
    }
    if (cameraClearance && !cam.explicitPose) {
        const float t = cameraClearance(center_, eye);
        if (t < 1.0f) eye = center_ + (eye - center_) * std::max(0.15f, t - 0.4f / std::max(dist, 0.5f));
    }
    FrameUniforms fu;
    const bool terrain = showTerrain && heightTex_ != nil;
    float zn = terrain ? std::max(0.05f, dist * 0.02f) : radius_ * 0.02f;
    const float zf = terrain ? std::max(120000.0f, dist * 4.0f) : radius_ * 40.0f;
    simd_float3 target = center_ + simd_make_float3(0, cam.aim, 0);
    simd_float3 upVec = simd_make_float3(0, 1, 0);
    float fovY = cam.fovY;
    if (cam.explicitPose) {
        target = cam.target;
        zn = std::max(cam.zNear, 0.06f);  // Camera_Z near >= 0.06 (0x4156e0)
        // FUEL's camera up follows its roll about the view axis (ctx+0x8c, Camera_Z +0x81c): the camera banks with
        // the vehicle when roll = -(vehicle bank).
        const simd_float3 f = simd_normalize(target - eye);
        const simd_float3 r0 = simd_normalize(simd_cross(f, upVec));
        const simd_float3 u0 = simd_cross(r0, f);
        upVec = u0 * std::cos(cam.roll) + r0 * std::sin(-cam.roll);
    }
    if (cam.fovH43 > 0) fovY = 2.0f * std::atan(std::tan(cam.fovH43 * 0.5f) * 0.75f);  // horizontal FOV of a 4:3 frame
    fu.viewProj = simd_mul(perspectiveRH(fovY, aspect, zn, zf), lookAtRH(eye, target, upVec));
    fu.invViewProj = simd_inverse(fu.viewProj);
    {
        const simd_float3 fwd = simd_normalize(target - eye);
        const simd_float3 right = simd_normalize(simd_cross(fwd, upVec));
        fu.camRight = simd_make_float4(right, 0);
        fu.camUp = simd_make_float4(simd_cross(right, fwd), 0);
    }
    fu.sunViewProj = sunViewProj();
    fu.terrain = terrainInfo_;
    fu.terrainGrid = simd_make_float4(center_.x, center_.z, std::max(70000.0f, dist * 6.0f), float(kTerrainGridSize));
    fu.fog = simd_make_float4(terrain ? 1.0f / 3600.0f : 0.0f, groundColour_ != nil ? 1.0f : 0.0f, zn, zf / (zn - zf));
    for (int l = 0; l < kGroundTileLevels; ++l) fu.groundFine[l] = groundFineRect_[l];
    fu.groundFar = groundFar_;
    fu.sunDir = simd_make_float4(sunDir_, 4.2f);
    fu.paint = simd_make_float4(srgbToLinear(paint_.color.x), srgbToLinear(paint_.color.y), srgbToLinear(paint_.color.z), 1);
    fu.eye = simd_make_float4(eye, 1);
    fu.params = simd_make_float4(dirt, 1.0f, sceneMin_.y, radius_ * 4.0f);
    fu.footprint = simd_make_float4(center_.x, center_.z, (sceneMax_.x - sceneMin_.x) * 0.5f, (sceneMax_.z - sceneMin_.z) * 0.5f);
    lastFrame_ = std::make_shared<FrameUniforms>(fu);

    // FUEL's ground chunks around the camera. A first frame or a jump of more than 2 km
    // waits for the chunks (headless shots), otherwise they stream in from the workers.
    std::vector<std::shared_ptr<const TerrainChunk>> terrainChunks;
    if (terrain && terrainLod_) {
        const bool jump = simd_distance(simd_make_float2(eye.x, eye.z), simd_make_float2(lastTerrainEye_.x, lastTerrainEye_.z)) > 2000.0f;
        lastTerrainEye_ = eye;
        if (jump)
            for (int k = 0; k < 14; ++k) {
                terrainLod_->update(eye.x, eye.z);
                terrainLod_->settle();
            }
        const simd_float3 look = (cam.explicitPose ? cam.target : center_) - eye;
        terrainChunks = terrainLod_->update(eye.x, eye.z, look.x, look.z);
        ++chunkFrame_;
        for (const auto& c : terrainChunks) {
            ChunkGpu& g = chunkGpu_[c->key];
            if (g.vb == nil)
                g.vb = [device_ newBufferWithBytes:c->verts.data() length:c->verts.size() * sizeof(TerrainVertex)
                                           options:MTLResourceStorageModeShared];
            g.lastFrame = chunkFrame_;
        }
        for (auto it = chunkGpu_.begin(); it != chunkGpu_.end();)
            it = chunkFrame_ - it->second.lastFrame > 240 ? chunkGpu_.erase(it) : std::next(it);
    }

    evaluateLodObjects(eye);  // p36: per-object LOD level and fade from this frame's camera
    // ---- FUEL's lights, scattering and shadow slices for this frame (P28), then the scene pass.
    fu.paint = simd_make_float4(paint_.color.x, paint_.color.y, paint_.color.z, 1);  // gamma space, like the game
    fillAtmosphere(fu, eye, fu.viewProj, 0, aspect, zn);
    const bool ownEncoder = enc == nil;
    if (ownEncoder) {
        // FUEL_PROFILE=1: frames cycle full / no shadow lookups / no vertex scattering / no shadow slices.
        static const bool profile = std::getenv("FUEL_PROFILE") != nullptr;
        const uint32_t pm = profile ? frame_ % 4 : 0;
        fu.fog.x = pm == 1 ? 1.0f : pm == 2 ? 2.0f : 0.0f;
        if (pm != 3) encodeShadows(frameCb_, fu, terrainChunks);
        enc = [frameCb_ renderCommandEncoderWithDescriptor:frameRp_];
    }
    lastFrame_ = std::make_shared<FrameUniforms>(fu);

    [enc setVertexBytes:&fu length:sizeof fu atIndex:1];
    [enc setFragmentBytes:&fu length:sizeof fu atIndex:1];
    [enc setFragmentTexture:shadowMap_ atIndex:7];
    [enc setFrontFacingWinding:MTLWindingClockwise];

    if (terrain) {
        [enc setRenderPipelineState:skyPipeline_];
        [enc setDepthStencilState:depthAlways_];
        [enc setCullMode:MTLCullModeNone];
        [enc drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:0 vertexCount:3];
    }

    auto drawPass = [&](uint32_t pass) {
        const AlphaMode mode = AlphaMode(pass);
        [enc setDepthStencilState:mode == AlphaMode::Blend ? depthNoWrite_ : depthWrite_];
        [enc setCullMode:cullBackFaces ? MTLCullModeBack : MTLCullModeNone];
        [enc setTriangleFillMode:wireframe ? MTLTriangleFillModeLines : MTLTriangleFillModeFill];
        auto drawMesh = [&](const GpuMesh& gm, NSUInteger count) {
            for (const Draw& d : gm.draws[pass]) {
                const GpuMaterial& m = gm.materials[d.material];
                // Sprites face the camera with either winding (tree cards wind
                // counter-clockwise on screen), so they are never culled.
                [enc setCullMode:cullBackFaces && !d.sprite ? MTLCullModeBack : MTLCullModeNone];
                [enc setRenderPipelineState:pipeline(gm.vbStrides[d.vb], mode)];
                [enc setVertexBuffer:gm.vbs[d.vb] offset:0 atIndex:0];
                const uint32_t sprite = d.sprite ? 1u : 0u;
                [enc setVertexBytes:&sprite length:sizeof sprite atIndex:3];
                [enc setVertexTexture:groundColour_ != nil ? groundColour_ : white_ atIndex:0];
                MaterialUniforms mu{m.diffuse, m.specular, m.emissive, m.fresnel, uint32_t(m.alpha), m.lit ? 1u : 0u, m.code,
                                    m.features};
                [enc setFragmentBytes:&mu length:sizeof mu atIndex:2];
                for (size_t slot = 0; slot < m.textures.size(); ++slot) [enc setFragmentTexture:m.textures[slot] atIndex:slot];
                [enc setFragmentSamplerState:samplers_[m.samplerIndex] atIndex:0];
                [enc drawIndexedPrimitives:MTLPrimitiveTypeTriangle
                                indexCount:d.indexCount
                                 indexType:MTLIndexTypeUInt32
                               indexBuffer:gm.indices
                         indexBufferOffset:size_t(d.firstIndex) * 4
                             instanceCount:count];
            }
        };
        for (const Instance& inst : instances_) {
            if (inst.mesh->draws[pass].empty()) continue;
            [enc setVertexBytes:&inst.world length:sizeof inst.world atIndex:2];
            drawMesh(*inst.mesh, 1);
        }
        for (const OtherVehicle& ov : vehicles_) {  // each with its own paint colour
            if (ov.instances.empty() || simd_distance(ov.instances[0].world.columns[3].xyz, center_) > kVehicleDrawDistance) continue;
            FrameUniforms vu = fu;
            vu.paint = simd_make_float4(ov.paint.x, ov.paint.y, ov.paint.z, 1);  // gamma space
            [enc setFragmentBytes:&vu length:sizeof vu atIndex:1];
            for (const Instance& inst : ov.instances) {
                if (inst.mesh->draws[pass].empty()) continue;
                [enc setVertexBytes:&inst.world length:sizeof inst.world atIndex:2];
                drawMesh(*inst.mesh, 1);
            }
            [enc setFragmentBytes:&fu length:sizeof fu atIndex:1];
        }
        for (const WorldGroup& g : worldGroups_) {  // one instanced draw per mesh
            if (g.mesh->draws[pass].empty()) continue;
            [enc setVertexBuffer:g.matrices offset:0 atIndex:2];
            drawMesh(*g.mesh, g.count);
        }
        for (const WorldGroup& g : propGroups_) {
            if (g.mesh->draws[pass].empty()) continue;
            [enc setVertexBuffer:g.matrices offset:0 atIndex:2];
            drawMesh(*g.mesh, g.count);
        }
        for (const WorldGroup& g : lodGroups_) {  // placed objects at this frame's level and fade (p36)
            if (g.mesh->draws[pass].empty()) continue;
            [enc setVertexBuffer:g.matrices offset:g.offset atIndex:2];
            drawMesh(*g.mesh, g.count);
        }
    };
    if (terrain) {
        [enc setRenderPipelineState:terrainPipeline_];
        [enc setDepthStencilState:depthWrite_];
        [enc setCullMode:MTLCullModeNone];
        [enc setTriangleFillMode:wireframe ? MTLTriangleFillModeLines : MTLTriangleFillModeFill];
        [enc setFragmentTexture:heightTex_ atIndex:0];
        if (groundColour_ != nil) {
            [enc setFragmentTexture:groundColour_ atIndex:1];
            [enc setFragmentTexture:groundSurface_ atIndex:2];
            [enc setFragmentTexture:groundDetail_ atIndex:3];
            [enc setFragmentTexture:groundFine_ atIndex:4];
            [enc setFragmentTexture:groundTypes_ atIndex:5];
            [enc setFragmentTexture:groundDecal_ atIndex:6];
            for (int k = 0; k < kDecalSlots; ++k) [enc setFragmentTexture:decalTex_[k] != nil ? decalTex_[k] : white_ atIndex:NSUInteger(8 + k)];
        }
        for (const auto& c : terrainChunks) {
            [enc setVertexBuffer:chunkGpu_[c->key].vb offset:0 atIndex:0];
            [enc drawIndexedPrimitives:MTLPrimitiveTypeTriangle
                            indexCount:terrainIndexCount_
                             indexType:MTLIndexTypeUInt32
                           indexBuffer:terrainIndices_
                     indexBufferOffset:0];
        }
    }
    // Terrain first (roads are painted into it, ground.phl decal path).
    drawPass(0);
    drawPass(1);
    if (terrain) encodeGrass(enc);
    if (terrain && waterVertices_ != nil && waterIndexCount_ > 0 && haveWaterLook_) {
        [enc setRenderPipelineState:waterPipeline_];
        [enc setDepthStencilState:depthNoWrite_];
        [enc setCullMode:MTLCullModeNone];
        [enc setTriangleFillMode:MTLTriangleFillModeFill];
        const float t = std::chrono::duration<float>(std::chrono::steady_clock::now() - grassEpoch_).count();
        [enc setVertexBuffer:waterVertices_ offset:0 atIndex:0];
        [enc setVertexBytes:waterConst_.data() length:sizeof(waterConst_) atIndex:2];
        [enc setVertexBytes:&t length:sizeof t atIndex:3];
        [enc setFragmentBytes:waterConst_.data() length:sizeof(waterConst_) atIndex:2];
        [enc setFragmentTexture:waterFoam_ != nil ? waterFoam_ : white_ atIndex:0];
        [enc setFragmentTexture:waterNormal_ != nil ? waterNormal_ : white_ atIndex:1];
        [enc setFragmentTexture:shadowMap_ atIndex:7];
        [enc drawIndexedPrimitives:MTLPrimitiveTypeTriangle
                        indexCount:waterIndexCount_
                         indexType:MTLIndexTypeUInt32
                       indexBuffer:waterIndices_
                 indexBufferOffset:0];
    }
    if (!terrain && showGround) {
        [enc setRenderPipelineState:groundPipeline_];
        [enc setDepthStencilState:depthWrite_];
        [enc setCullMode:MTLCullModeNone];
        [enc setTriangleFillMode:MTLTriangleFillModeFill];
        [enc drawPrimitives:MTLPrimitiveTypeTriangleStrip vertexStart:0 vertexCount:4];
    }
    drawPass(2);
    if (!markers_.empty()) {
        [enc setRenderPipelineState:markerPipeline_];
        [enc setDepthStencilState:depthNoWrite_];
        [enc setCullMode:MTLCullModeNone];
        [enc setTriangleFillMode:MTLTriangleFillModeFill];
        [enc setVertexBytes:markers_.data() length:markers_.size() * sizeof(simd_float4) atIndex:2];
        [enc drawPrimitives:MTLPrimitiveTypeTriangleStrip vertexStart:0 vertexCount:66 instanceCount:markers_.size()];
    }
    if (ownEncoder) [enc endEncoding];
}


void MeshRenderer::ensureTargets(NSUInteger width, NSUInteger height) {
    auto make = [&](MTLPixelFormat pf, NSUInteger w, NSUInteger h, NSUInteger samples, MTLTextureUsage usage) {
        MTLTextureDescriptor* td = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:pf
                                                                                      width:std::max<NSUInteger>(w, 1)
                                                                                     height:std::max<NSUInteger>(h, 1)
                                                                                  mipmapped:NO];
        td.textureType = samples > 1 ? MTLTextureType2DMultisample : MTLTextureType2D;
        td.sampleCount = samples;
        td.usage = usage;
        td.storageMode = samples > 1 ? MTLStorageModeMemoryless : MTLStorageModePrivate;
        return [device_ newTextureWithDescriptor:td];
    };
    const MTLTextureUsage rt = MTLTextureUsageRenderTarget, rw = MTLTextureUsageRenderTarget | MTLTextureUsageShaderRead;
    if (shrink_ == nil) {
        // The fixed-size HDR chain targets (Renderer_CreatePostTargets 0x837010).
        shrink_ = make(kRenderColorFormat, 128, 128, 1, rw);
        for (size_t k = 0; k < lum_.size(); ++k) lum_[k] = make(MTLPixelFormatR32Float, 128 >> k, 128 >> k, 1, rw);
        for (auto& t : adapted_) t = make(MTLPixelFormatR32Float, 1, 1, 1, rw);
        for (auto& t : bloomRt_) t = make(MTLPixelFormatRGBA8Unorm, 128, 128, 1, rw);
    }
    if (width == targetWidth_ && height == targetHeight_) return;
    targetWidth_ = width, targetHeight_ = height;
    hdrMsaa_ = make(kRenderColorFormat, width, height, sampleCount_, rt);
    depthMsaa_ = make(kRenderDepthFormat, width, height, sampleCount_, rt);
    hdr_ = make(kRenderColorFormat, width, height, 1, rw);
    depth_ = make(kRenderDepthFormat, width, height, 1, rw);
    postFx_ = make(MTLPixelFormatRGBA8Unorm, width, height, 1, rw);
}

void MeshRenderer::render(id<MTLCommandBuffer> cb, id<MTLTexture> target, const OrbitCamera& cam) {
    if (frame_ == 0) epoch_ = std::chrono::steady_clock::now();  // the running clock starts with the first frame
    ensureTargets(target.width, target.height);

    // The scene, multisampled; colour and depth resolve for the passes after it. The scene encoder is
    // made inside encode(), after the frame's shadow slices.
    MTLRenderPassDescriptor* rp = [MTLRenderPassDescriptor renderPassDescriptor];
    rp.colorAttachments[0].texture = hdrMsaa_;
    rp.colorAttachments[0].resolveTexture = hdr_;
    rp.colorAttachments[0].loadAction = MTLLoadActionClear;
    rp.colorAttachments[0].clearColor = MTLClearColorMake(0.10, 0.12, 0.15, 1.0);  // behind single meshes (no sky)
    rp.colorAttachments[0].storeAction = MTLStoreActionMultisampleResolve;
    rp.depthAttachment.texture = depthMsaa_;
    rp.depthAttachment.resolveTexture = depth_;
    rp.depthAttachment.loadAction = MTLLoadActionClear;
    rp.depthAttachment.clearDepth = 1.0;
    rp.depthAttachment.storeAction = MTLStoreActionMultisampleResolve;
    rp.depthAttachment.depthResolveFilter = MTLMultisampleDepthResolveFilterMin;  // the nearest sample
    lastFrame_.reset();
    frameCb_ = cb;
    frameRp_ = rp;
    if (instances_.empty()) {
        id<MTLRenderCommandEncoder> enc = [cb renderCommandEncoderWithDescriptor:rp];
        [enc endEncoding];
    } else {
        encode(nil, float(target.width) / float(std::max<NSUInteger>(target.height, 1)), cam);
    }
    frameCb_ = nil;
    frameRp_ = nil;

    if (lastFrame_) encodeDust(cb);
    encodePost(cb, target);
    ++frame_;
    [cb addCompletedHandler:^(id<MTLCommandBuffer> b) {
        if (b.GPUEndTime > b.GPUStartTime) gpuMs_ = (b.GPUEndTime - b.GPUStartTime) * 1000.0;
        static const bool profile = std::getenv("FUEL_PROFILE") != nullptr;
        if (profile) {
            static double sum[4] = {0, 0, 0, 0};
            static int n[4] = {0, 0, 0, 0}, total = 0;
            const int k = (total++) % 4;  // frames complete in order
            sum[k] += (b.GPUEndTime - b.GPUStartTime) * 1000.0, ++n[k];
            if (total % 40 == 0)
                std::printf("profile: full %.2f, no shadow lookups %.2f, no vertex scattering %.2f, no shadow slices %.2f ms\n",
                            sum[0] / std::max(1, n[0]), sum[1] / std::max(1, n[1]), sum[2] / std::max(1, n[2]), sum[3] / std::max(1, n[3]));
        }
    }];
}

void MeshRenderer::encodeDust(id<MTLCommandBuffer> cb) {
    if (sfxAtlas_ == nil) return;
    encodeClouds(cb);
    encodeTanks(cb);
    if (dust_.sprites.empty()) return;
    // Back to front from the camera (FUEL sorts its 32-particle buckets by depth, 0x5fee80; the port sorts sprites).
    const simd_float3 eye = lastFrame_->eye.xyz;
    const std::vector<SfxSprite>& sp = dust_.sprites;
    std::vector<std::pair<float, uint32_t>> order;
    order.reserve(sp.size());
    for (uint32_t i = 0; i < sp.size(); ++i) {
        simd_float3 p = sp[i].posAngle.xyz;
        if (sp[i].life.z > 0.5f) {
            const float t = dust_.clock - sp[i].life.x;
            p += t * sp[i].vel.xyz + 0.5f * t * t * sp[i].acc.xyz;
        }
        order.emplace_back(-simd_distance_squared(p, eye), i);
    }
    std::sort(order.begin(), order.end());
    id<MTLBuffer> buf = [device_ newBufferWithLength:order.size() * sizeof(SfxSprite) options:MTLResourceStorageModeShared];
    auto* out = static_cast<SfxSprite*>(buf.contents);
    for (size_t k = 0; k < order.size(); ++k) out[k] = sp[order[k].second];
    struct {
        simd_float4 user0, user1, user2;
    } params = {simd_make_float4(dust_.clock, 100.0f, 0, 0), simd_make_float4(dust_.fadeTo, dust_.fadeOut, 0.01f, 0),
                simd_make_float4(0.8f, 0.1f, std::getenv("FUEL_SFX_DEBUG") && std::atoi(std::getenv("FUEL_SFX_DEBUG")) == 2 ? 1.0f : 0.0f, 0)};  // 0x5f6550

    MTLRenderPassDescriptor* rp = [MTLRenderPassDescriptor renderPassDescriptor];
    rp.colorAttachments[0].texture = hdr_;
    rp.colorAttachments[0].loadAction = MTLLoadActionLoad;
    rp.colorAttachments[0].storeAction = MTLStoreActionStore;
    id<MTLRenderCommandEncoder> enc = [cb renderCommandEncoderWithDescriptor:rp];
    [enc setRenderPipelineState:dustPipeline_];
    [enc setVertexBuffer:buf offset:0 atIndex:0];
    [enc setVertexBytes:lastFrame_.get() length:sizeof(FrameUniforms) atIndex:1];
    [enc setVertexBytes:&params length:sizeof params atIndex:2];
    [enc setVertexTexture:shadowMap_ atIndex:0];
    [enc setFragmentBytes:lastFrame_.get() length:sizeof(FrameUniforms) atIndex:1];
    [enc setFragmentTexture:depth_ atIndex:0];
    [enc setFragmentTexture:sfxAtlas_ atIndex:1];
    [enc drawPrimitives:MTLPrimitiveTypeTriangleStrip vertexStart:0 vertexCount:4 instanceCount:order.size()];
    [enc endEncoding];
}

// FUEL's billboard clouds (SpecialEffectsCloudTank_G, p37-sky-clouds.md): the tank follows the camera (vf+0x24
// 0x5fd890), refreshes every 10 s (0x5fe040), and draws its active clouds with vh_cloud (constants 0x5fe370).
void MeshRenderer::encodeClouds(id<MTLCommandBuffer> cb) {
    const auto now = std::chrono::steady_clock::now();
    float dt = haveCloudClock_ ? std::min(0.25f, std::chrono::duration<float>(now - cloudClock_).count()) : 0.0f;
    if (exposureDt >= 0) dt = exposureDt;
    cloudClock_ = now, haveCloudClock_ = true;
    if (!weatherFields_ || cloudTex_ == nil || !weatherFromWorld) return;
    const WeatherAmounts& w = weatherNow_;
    const float cloud = w.cloud;
    const float cloudness = std::min(std::max(2.0f * cloud, 0.0f), 1.0f);  // 0x5b7e40: clamp01(2 x 0x5bac10)
    if (!cloudTank_) {
        auto rnd = [this]() {  // port generator (FUEL: WELL512 0x5fd710 and 0x6460f0, shared with other systems)
            cloudRng_ ^= cloudRng_ << 13, cloudRng_ ^= cloudRng_ >> 17, cloudRng_ ^= cloudRng_ << 5;
            return float(cloudRng_ >> 8) / 16777216.0f;
        };
        cloudTank_ = std::make_unique<clouds::Tank>(clouds::Params::fromPP(weatherFields_->pp()), rnd, rnd);
    }
    const simd_float3 eye = lastFrame_->eye.xyz;
    cloudTank_->setPosition({eye.x, eye.y, eye.z}, cloudness);
    cloudTank_->update(dt, cloud, cloudness);
    const int n = cloudTank_->activeCount();  // +8 = +0x98: the active clouds lead the sorted array
    if (n <= 0) return;
    const auto& cs = cloudTank_->clouds();
    struct CloudSprite {
        simd_float4 pos, uv, size;
    };
    if (cloudBuf_ == nil) cloudBuf_ = [device_ newBufferWithLength:cs.size() * sizeof(CloudSprite) options:MTLResourceStorageModeShared];
    if (cloudTank_->dirty()) {  // 0x5fe580: uv quarter by variant, u swapped when mirrored
        auto* out = static_cast<CloudSprite*>(cloudBuf_.contents);
        for (size_t i = 0; i < cs.size(); ++i) {
            const clouds::Cloud& c = cs[i];
            float u0 = 0, v0 = 0;
            if (c.variant == 1) u0 = 0.5f;
            else if (c.variant == 2) u0 = 0.5f, v0 = 0.5f;
            else if (c.variant == 3) v0 = 0.5f;
            float ua = u0, ub = u0 + 0.5f;
            if (c.mirror) std::swap(ua, ub);
            out[i] = {simd_make_float4(c.pos[0], c.pos[1], c.pos[2], c.width), simd_make_float4(ua, ub, v0, v0 + 0.5f),
                      simd_make_float4(c.height, 0, 0, 0)};
        }
        cloudTank_->clearDirty();
    }
    const clouds::Params& p = cloudTank_->params();
    // CloudsAlpha = (cloud + 1) / 2 (0x5bba30 -> +0xf4); vertex colour 1.25 PP5201 (0x5fe580).
    const std::array<float, 28> k = clouds::drawConstants(cloudness, (cloud + 1.0f) * 0.5f, p.sight, cloudTank_->layerHeight(),
                                                          cloudTank_->position()[1]);
    struct {
        simd_float4 user0, user5, user6, rgba;
    } params = {simd_make_float4(k[0], k[1], k[2], k[3]), simd_make_float4(k[20], k[21], k[22], k[23]),
                simd_make_float4(k[24], k[25], k[26], k[27]), simd_make_float4(1.25f * p.colour, 1.25f * p.colour, 1.25f * p.colour, 0.5f)};
    MTLRenderPassDescriptor* rp = [MTLRenderPassDescriptor renderPassDescriptor];
    rp.colorAttachments[0].texture = hdr_;
    rp.colorAttachments[0].loadAction = MTLLoadActionLoad;
    rp.colorAttachments[0].storeAction = MTLStoreActionStore;
    id<MTLRenderCommandEncoder> enc = [cb renderCommandEncoderWithDescriptor:rp];
    [enc setRenderPipelineState:cloudPipeline_];
    [enc setVertexBuffer:cloudBuf_ offset:0 atIndex:0];
    [enc setVertexBytes:lastFrame_.get() length:sizeof(FrameUniforms) atIndex:1];
    [enc setVertexBytes:&params length:sizeof params atIndex:2];
    [enc setFragmentBytes:lastFrame_.get() length:sizeof(FrameUniforms) atIndex:1];
    [enc setFragmentTexture:depth_ atIndex:0];
    [enc setFragmentTexture:cloudTex_ atIndex:1];
    [enc drawPrimitives:MTLPrimitiveTypeTriangleStrip vertexStart:0 vertexCount:4 instanceCount:NSUInteger(n)];
    [enc endEncoding];
}

// FUEL's rain and snow tanks (SpecialEffectsRainTank_G 0x5e1920 / SnowTank_G 0x5dfbd0, started by the weather apply
// 0x5bba30 with 1024 x 12 rain drops and 12000 flakes; drawn count = floor(amount x N) 0x5de850).
void MeshRenderer::encodeTanks(id<MTLCommandBuffer> cb) {
    const WeatherAmounts& w = weatherNow_;
    if (std::getenv("FUEL_SFX_DEBUG") && frame_ % 30 == 0)
        std::fprintf(stderr, "weather at camera: cloud %.2f rain %.2f snow %.2f fog %.2f wind %.1f (%.2f, %.2f) humidity %.2f clouds %d\n", w.cloud,
                     w.rain, w.snow, w.fog, w.windSpeed, wind_.x, wind_.z, w.humidity, cloudTank_ ? cloudTank_->activeCount() : -1);
    if (weatherFields_) {  // 0x5b8680: the field wind with gusts (WELL512 draws; port generator), on the game clock
        windClock_ += exposureDt >= 0 ? exposureDt : 1.0 / 60.0;
        const std::array<float, 3> g = gusts_.apply(w.wind, windClock_, weatherFields_->pp(), [this]() {
            cloudRng_ ^= cloudRng_ << 13, cloudRng_ ^= cloudRng_ >> 17, cloudRng_ ^= cloudRng_ << 5;
            return float(cloudRng_ >> 8) / 16777216.0f;
        });
        wind_ = simd_make_float3(g[0], g[1], g[2]);
    }
    if (!weatherFields_ || (w.rain <= 0 && w.snow <= 0)) {
        tankClock_ = std::chrono::steady_clock::now(), haveTankClock_ = false;
        return;
    }
    auto P = [&](int n) { return weatherFields_->P(n); };
    const auto now = std::chrono::steady_clock::now();
    float dt = haveTankClock_ ? std::min(0.25f, std::chrono::duration<float>(now - tankClock_).count()) : 0.0f;
    if (exposureDt >= 0) dt = exposureDt;
    tankClock_ = now, haveTankClock_ = true;
    const simd_float3 eye = lastFrame_->eye.xyz;
    const simd_float3 fwd = simd_cross(lastFrame_->camUp.xyz, lastFrame_->camRight.xyz);
    // Rain velocity (0x5e1fc0): the player's velocity smoothed 0.25 per frame, x PP5043, + 0.3 wind, at most PP5042.
    // Port: the camera's velocity stands in for the player vehicle's (it follows it). The wind is the weather
    // manager's +0xc4 (0x5b8330 field x speed, 0x5b8680 gusts; the apply 0x5bba30 copies it into the tanks' +0x20).
    simd_float3 v = haveTankEye_ && dt > 0 ? (eye - tankEye_) / dt : simd_make_float3(0, 0, 0);
    if (simd_length(v) > 200.0f) v = 0;
    tankEye_ = eye, haveTankEye_ = true;
    rainSmooth_ += (v - rainSmooth_) * 0.25f;
    simd_float3 rv = rainSmooth_ * P(5043) + 0.3f * wind_;
    const float rl = simd_length(rv);
    if (P(5042) < rl) rv *= P(5042) / rl;
    const simd_float3 half = simd_make_float3(rv.x * 0.5f, rv.y + P(5058), rv.z * 0.5f);  // +0xb4..+0xbc (0x5e1cb0)
    rainShift_ += half * dt;
    // Snow drift (0x5e0240): 0.5 wind + a fall of lerp(PP5027, PP5026, drawn / N).
    const int rainN = 1024 * 12, snowN = 12000;
    const int rainDrawn = int(std::floor(w.rain * float(rainN))), snowDrawn = int(std::floor(w.snow * float(snowN)));
    snowShift_ += simd_make_float3(0.5f * wind_.x, 0.5f * wind_.y + P(5027) + (P(5026) - P(5027)) * (float(snowDrawn) / float(snowN)),
                                   0.5f * wind_.z) * dt;
    auto ensure = [&](__strong id<MTLBuffer>& buf, int n, bool rain) {
        if (buf != nil) return;
        std::vector<simd_float4> d(static_cast<size_t>(n));
        uint32_t r = rain ? 0x2545f491u : 0x9e3779b9u;  // port RNG (FUEL fills the buffer with rand 0x6460f0)
        auto u = [&]() {
            r ^= r << 13, r ^= r >> 17, r ^= r << 5;
            return float(r >> 8) / 16777216.0f;
        };
        for (auto& q : d) {
            q.x = u() * 2 - 1, q.y = u() * 2 - 1, q.z = u() * 2 - 1;
            q.w = rain ? float(int(u() * 1000.0f) + 1) + u() : u();  // vh_rain: x speed = w / 1000, z speed = frac(w)
        }
        buf = [device_ newBufferWithBytes:d.data() length:d.size() * sizeof(simd_float4) options:MTLResourceStorageModeShared];
    };
    struct TankParams {
        simd_float4 user[8];
        simd_float4 mode, uv;
    };
    auto atlasRect = [&](int i) {
        if (!sfxTables_ || size_t(i) >= sfxTables_->size()) return simd_make_float4(0, 0, 1, 1);
        const auto& e = (*sfxTables_)[size_t(i)];
        return simd_make_float4(e.u0, e.v0, e.du, e.dv);
    };
    MTLRenderPassDescriptor* rp = [MTLRenderPassDescriptor renderPassDescriptor];
    rp.colorAttachments[0].texture = hdr_;
    rp.colorAttachments[0].loadAction = MTLLoadActionLoad;
    rp.colorAttachments[0].storeAction = MTLStoreActionStore;
    id<MTLRenderCommandEncoder> enc = [cb renderCommandEncoderWithDescriptor:rp];
    [enc setRenderPipelineState:tankPipeline_];
    [enc setVertexBytes:lastFrame_.get() length:sizeof(FrameUniforms) atIndex:1];
    [enc setFragmentBytes:lastFrame_.get() length:sizeof(FrameUniforms) atIndex:1];
    [enc setFragmentTexture:depth_ atIndex:0];
    [enc setFragmentTexture:sfxAtlas_ atIndex:1];
    if (rainDrawn > 0) {  // 0x5e21a0
        ensure(rainDrops_, rainN, true);
        TankParams t{};
        t.user[0] = simd_make_float4(P(5040), P(5040), P(5040), 0);
        t.user[1] = simd_make_float4(rv.x, rv.y + P(5058), rv.z, 30.0f);  // tank scale .rdata 0x9dbf90 (30)
        t.user[2] = simd_make_float4(P(5044), P(5045), P(5046), 0.3f);
        t.user[3] = simd_make_float4(P(5047), P(5048), P(5048), P(5048));
        // Centre: camera + camera rotation x (0.5 vx, 0.5 (vy + PP5058), 0.5 vz + PP5049) (tank +0x38 = PP5049).
        const simd_float3 local = simd_make_float3(half.x, half.y * 0.5f, half.z + P(5049));
        t.user[4] = simd_make_float4(eye + lastFrame_->camRight.xyz * local.x + lastFrame_->camUp.xyz * local.y + fwd * local.z, 0);
        t.user[5] = simd_make_float4(rainShift_, 1);
        t.mode = simd_make_float4(0, 0, 0, 0);
        t.uv = atlasRect(50);  // the splash tank's rain sub-tank: atlas 0x32
        [enc setVertexBuffer:rainDrops_ offset:0 atIndex:0];
        [enc setVertexBytes:&t length:sizeof t atIndex:2];
        [enc setFragmentBytes:&t length:sizeof t atIndex:2];
        [enc drawPrimitives:MTLPrimitiveTypeTriangleStrip vertexStart:0 vertexCount:4 instanceCount:NSUInteger(rainDrawn)];
    }
    if (snowDrawn > 0) {  // 0x5dffc0
        ensure(snowDrops_, snowN, false);
        TankParams t{};
        const float scale = P(5009);  // snow tank +0x38
        t.user[0] = simd_make_float4(P(5001), P(5001), P(5001), P(5004));
        t.user[1] = simd_make_float4(snowShift_, scale);
        t.user[2] = simd_make_float4(P(5005), P(5006), P(5007), P(5008));
        t.user[3] = simd_make_float4(P(5002), 0, 0, 0);
        t.user[4] = simd_make_float4(eye + fwd * scale, 0);
        t.mode = simd_make_float4(1, 0, 0, 0);
        t.uv = atlasRect(49);  // 0x31
        [enc setVertexBuffer:snowDrops_ offset:0 atIndex:0];
        [enc setVertexBytes:&t length:sizeof t atIndex:2];
        [enc drawPrimitives:MTLPrimitiveTypeTriangleStrip vertexStart:0 vertexCount:4 instanceCount:NSUInteger(snowDrawn)];
    }
    [enc endEncoding];
}

void MeshRenderer::loadSfxAtlas(const DpcFile& world) {
    // FX_ATLAS_0: MaterialAnim_Z (link = name CRC) in DATAS/SHARED.DPC -> body + 0x3c Material_Z -> slot 0 bitmap.
    namespace fs = std::filesystem;
    const fs::path shared = fs::path(world.path()).parent_path().parent_path() / "DATAS" / "SHARED.DPC";
    try {
        if (!fs::exists(shared)) return;
        sfxShared_ = std::make_unique<DpcFile>(shared.string());
        const DpcFile& d = *sfxShared_;
        auto load = [&](const char* nameStr) -> id<MTLTexture> {
            const uint32_t want = asoboCrc(nameStr);
            for (uint32_t oid : d.ids(cls::MaterialAnim)) {
                const DpcFile::Entry* e = d.find(oid);
                const std::vector<uint8_t> link = d.link(*e);
                uint32_t name = 0;
                if (link.size() >= 4) std::memcpy(&name, link.data(), 4);
                if (name != want) continue;
                const auto body = d.body(*e);
                if (!body || body->size() < 0x40) return nil;
                uint32_t matId;
                std::memcpy(&matId, body->data() + 0x3c, 4);
                const DpcFile::Entry* me = d.find(matId);
                if (!me) return nil;
                const Material mat = parseMaterialBody(d.body(*me).value());
                const DpcFile::Entry* be = d.find(mat.textures[0]);
                if (!be) return nil;
                uint8_t alphaKind = 0;
                id<MTLTexture> t = uploadBitmap(d, *be, false, &alphaKind);
                if (std::getenv("FUEL_SFX_DEBUG"))
                    std::fprintf(stderr, "sfx %s: material %08x bitmap %08x -> %s\n", nameStr, matId, mat.textures[0], t ? "loaded" : "failed");
                return t;
            }
            return nil;
        };
        sfxAtlas_ = load("FX_ATLAS_0");
        cloudTex_ = load("FX_BILLBOARD_CLOUDS");  // the cloud tank's atlas entry 14 (whole texture)
    } catch (const std::exception& e) {
        std::fprintf(stderr, "fuelview: no FX atlas: %s\n", e.what());
    }
}

// FUEL's HDR chain (post.md P3 / P6), with the per-frame exposure, bloom scale and blue shift of
// GenWorld_FrameUpdate 0x77a3b0 (p26-renderer.md L1 step 2).
void MeshRenderer::encodePost(id<MTLCommandBuffer> cb, id<MTLTexture> target) {
    const float w = float(target.width), h = float(std::max<NSUInteger>(target.height, 1));
    // Seconds since the last frame (the adaptation rate k = 0.94^(30 dt)).
    const auto now = std::chrono::steady_clock::now();
    float dt = haveLastRender_ ? std::min(0.25f, std::chrono::duration<float>(now - lastRender_).count()) : 1.0f / 60;
    lastRender_ = now, haveLastRender_ = true;
    if (exposureDt >= 0) dt = exposureDt;

    const Atmosphere& a = atmo_;
    PostUniforms pu{};
    pu.bloom = simd_make_float4(a.bloomLumScale, postDefaults_.bloom[1], postDefaults_.bloom[2], postDefaults_.bloom[3]);
    pu.hdr = simd_make_float4(adaptationRate(dt), a.screenLuminance, a.minExposure, a.maxExposure);
    pu.blueShift = simd_make_float4(a.blueShift[0], a.blueShift[1], a.blueShift[2], a.blueShift[3]);
    pu.tint = simd_make_float4(postDefaults_.tint[0], postDefaults_.tint[1], postDefaults_.tint[2], postDefaults_.tint[3]);
    // gMotionBlurSettings0 from MinRadius 0, MaxRadius 1, FallOff 6 (0x64ff38), scale = viewport +0x1dc.
    const float blur = !postEffects ? 0.0f : speedBlurAbsolute >= 0 ? speedBlurAbsolute : std::clamp(speedBlur, 0.0f, 1.0f) * speedBlurMax;
    pu.mb0 = simd_make_float4(1.0f, 0.0f, 6.0f, blur);
    pu.texel = simd_make_float4(0, 0, 1.0f / w, 1.0f / h);
    const bool srgbOut = target.pixelFormat == MTLPixelFormatBGRA8Unorm_sRGB || target.pixelFormat == MTLPixelFormatRGBA8Unorm_sRGB;
    pu.flags = simd_make_float4(0, postEffects && postDefaults_.fromExe ? 1.0f : 0.0f, srgbOut ? 1.0f : 0.0f, 0);
    float mbw[8];
    motionBlurWeights(0.95f, mbw);  // WeightDecrease [0x9dc5cc]
    for (int i = 0; i < 8; ++i) pu.mbw[i] = simd_make_float4(mbw[i], 0, 0, 0);
    float goff[11], gw[11];
    gaussian11(goff, gw);

    auto pass = [&](id<MTLTexture> dst, id<MTLRenderPipelineState> ps, std::initializer_list<id<MTLTexture>> textures, const PostUniforms& u) {
        MTLRenderPassDescriptor* rp = [MTLRenderPassDescriptor renderPassDescriptor];
        rp.colorAttachments[0].texture = dst;
        rp.colorAttachments[0].loadAction = MTLLoadActionDontCare;
        rp.colorAttachments[0].storeAction = MTLStoreActionStore;
        id<MTLRenderCommandEncoder> enc = [cb renderCommandEncoderWithDescriptor:rp];
        [enc setRenderPipelineState:ps];
        [enc setFragmentBytes:&u length:sizeof u atIndex:0];
        NSUInteger slot = 0;
        for (id<MTLTexture> t : textures) [enc setFragmentTexture:t atIndex:slot++];
        [enc drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:0 vertexCount:3];
        [enc endEncoding];
    };
    // 1-3: shrink, log luminance, reductions to 1 x 1.
    pass(shrink_, shrinkPipeline_, {hdr_}, pu);
    pass(lum_[0], lumLogPipeline_, {hdr_}, pu);
    for (size_t k = 1; k < lum_.size(); ++k) {
        PostUniforms u = pu;
        u.texel.x = 1.0f / float(lum_[k - 1].width), u.texel.y = 1.0f / float(lum_[k - 1].height);
        pass(lum_[k], lumAvgPipeline_, {lum_[k - 1]}, u);
    }
    // 4: adaptation. The first frame initialises (S = SCREENLUMINANCE / e^A) and, so that single shots
    // show the settled exposure, then takes one step with k = 0 (port).
    auto adapt = [&](bool init, float k) {
        PostUniforms u = pu;
        u.flags.x = init ? 1.0f : 0.0f;
        u.hdr.x = k;
        const int prev = adaptedIndex_;
        adaptedIndex_ ^= 1;
        pass(adapted_[size_t(adaptedIndex_)], adaptPipeline_, {lum_.back(), adapted_[size_t(prev)]}, u);
    };
    if (adaptInit_) {
        adapt(true, pu.hdr.x);
        adapt(false, 0.0f);
        adaptInit_ = false;
    } else {
        adapt(false, pu.hdr.x);
    }
    id<MTLTexture> S = adapted_[size_t(adaptedIndex_)];
    // 5-7: bright pass, gaussian H / V, the two star streaks.
    if (pu.flags.y > 0) {
        pass(bloomRt_[0], brightPipeline_, {shrink_, S}, pu);
        auto gauss = [&](id<MTLTexture> src, id<MTLTexture> dst, float dx, float dy) {
            PostUniforms u = pu;
            for (int k = 0; k < 11; ++k) u.taps[k] = simd_make_float4(goff[k] / 128.0f * dx, goff[k] / 128.0f * dy, gw[k], 0);
            pass(dst, gaussPipeline_, {src}, u);
        };
        gauss(bloomRt_[0], bloomRt_[1], 1, 0);
        gauss(bloomRt_[1], bloomRt_[0], 0, 1);
        gauss(bloomRt_[0], bloomRt_[1], 3, 3);
        gauss(bloomRt_[1], bloomRt_[2], 6, 6);
        gauss(bloomRt_[0], bloomRt_[1], -3, 3);
        gauss(bloomRt_[1], bloomRt_[3], -6, 6);
    }
    // 8-9: tone map, then the radial speed blur when it is on (viewport flag 0x8000).
    if (blur > 0) {
        PostUniforms u = pu;
        u.flags.z = 0;  // the intermediate stores the display value as is
        pass(postFx_, toneMapFxPipeline_, {hdr_, S, bloomRt_[0], bloomRt_[2], bloomRt_[3]}, u);
        pass(target, radialPipeline_, {postFx_}, pu);
    } else {
        pass(target, toneMapPipeline_, {hdr_, S, bloomRt_[0], bloomRt_[2], bloomRt_[3]}, pu);
    }
}

std::vector<simd_float4> MeshRenderer::shaderCheck(const Atmosphere& a, simd_float3 eye, const std::vector<simd_float4>& inputs) {
    FrameUniforms fu{};
    fu.eye = simd_make_float4(eye, 1);
    fu.lightDir = f4(a.lightDir, a.invIntensity);
    fu.skyDir = f4(a.skyDir, 0);
    fu.sunColour = f4(a.sunColour, 1);
    fu.dlight = f4(a.dlightColor, a.blueShift[3]);
    fu.skyColour = f4(a.dskyColor, 0);
    fu.skyAmbient = f4(a.dskyAmbient, 0);
    for (int i = 0; i < 11; ++i) fu.scat[i] = simd_make_float4(a.c[size_t(i)][0], a.c[size_t(i)][1], a.c[size_t(i)][2], a.c[size_t(i)][3]);
    const size_t n = inputs.size() / 4;
    std::vector<simd_float4> out(n * 2);
    if (n == 0) return out;
    NSError* err = nil;
    id<MTLComputePipelineState> ps = [device_ newComputePipelineStateWithFunction:[shaders_ newFunctionWithName:@"kRenderCheck"] error:&err];
    if (ps == nil) return {};
    id<MTLBuffer> inb = [device_ newBufferWithBytes:inputs.data() length:inputs.size() * sizeof(simd_float4) options:MTLResourceStorageModeShared];
    id<MTLBuffer> outb = [device_ newBufferWithLength:out.size() * sizeof(simd_float4) options:MTLResourceStorageModeShared];
    id<MTLCommandQueue> q = [device_ newCommandQueue];
    id<MTLCommandBuffer> cb = [q commandBuffer];
    id<MTLComputeCommandEncoder> enc = [cb computeCommandEncoder];
    [enc setComputePipelineState:ps];
    [enc setBytes:&fu length:sizeof fu atIndex:0];
    [enc setBuffer:inb offset:0 atIndex:1];
    [enc setBuffer:outb offset:0 atIndex:2];
    [enc dispatchThreads:MTLSizeMake(n, 1, 1) threadsPerThreadgroup:MTLSizeMake(std::min<NSUInteger>(n, 64), 1, 1)];
    [enc endEncoding];
    [cb commit];
    [cb waitUntilCompleted];
    std::memcpy(out.data(), outb.contents, out.size() * sizeof(simd_float4));
    return out;
}

// ---- Grass tufts (p14) -------------------------------------------------------

void MeshRenderer::createGrassPipeline() {
    MTLRenderPipelineDescriptor* d = [MTLRenderPipelineDescriptor new];
    d.vertexFunction = [shaders_ newFunctionWithName:@"vsGrass"];
    d.fragmentFunction = [shaders_ newFunctionWithName:@"fsGrass"];
    d.rasterSampleCount = sampleCount_;
    d.alphaToCoverageEnabled = sampleCount_ > 1;
    d.colorAttachments[0].pixelFormat = kRenderColorFormat;
    d.depthAttachmentPixelFormat = kRenderDepthFormat;
    NSError* e = nil;
    grassPipeline_ = [device_ newRenderPipelineStateWithDescriptor:d error:&e];
    if (grassPipeline_ == nil) [NSException raise:@"pipeline" format:@"%@", e];
    grassEpoch_ = std::chrono::steady_clock::now();
}


void MeshRenderer::setGroundTile(int level, double originX, double originZ, float extent, const std::vector<uint8_t>& rgba,
                                 const std::vector<uint16_t>& types, const std::vector<float>& decal) {
    const size_t texels = size_t(kGroundTile) * kGroundTile;
    if (groundFine_ == nil || level < 0 || level >= kGroundTileLevels || rgba.size() != texels * 4 || types.size() != texels * 4 ||
        decal.size() != texels * 4)
        return;
    [groundTypes_ replaceRegion:MTLRegionMake2D(0, 0, kGroundTile, kGroundTile)
                    mipmapLevel:0
                          slice:NSUInteger(level)
                      withBytes:types.data()
                    bytesPerRow:kGroundTile * 8
                  bytesPerImage:0];
    [groundDecal_ replaceRegion:MTLRegionMake2D(0, 0, kGroundTile, kGroundTile)
                    mipmapLevel:0
                          slice:NSUInteger(level)
                      withBytes:decal.data()
                    bytesPerRow:kGroundTile * 16
                  bytesPerImage:0];
    [groundFine_ replaceRegion:MTLRegionMake2D(0, 0, kGroundTile, kGroundTile)
                   mipmapLevel:0
                         slice:NSUInteger(level)
                     withBytes:rgba.data()
                   bytesPerRow:kGroundTile * 4
                 bytesPerImage:0];
    if (blitQueue_ == nil) blitQueue_ = [device_ newCommandQueue];
    id<MTLCommandBuffer> cb = [blitQueue_ commandBuffer];
    id<MTLBlitCommandEncoder> blit = [cb blitCommandEncoder];
    [blit generateMipmapsForTexture:groundFine_];
    [blit endEncoding];
    [cb commit];
    groundFineRect_[level] = simd_make_float4(float(originX), float(originZ), extent, 1);
}

void MeshRenderer::setWater(const std::vector<WaterVertex>& vertices, const std::vector<uint32_t>& indices) {
    waterIndexCount_ = uint32_t(indices.size());
    waterVertices_ = vertices.empty() ? nil : [device_ newBufferWithBytes:vertices.data() length:vertices.size() * sizeof(WaterVertex)
                                                                  options:MTLResourceStorageModeShared];
    waterIndices_ = indices.empty() ? nil : [device_ newBufferWithBytes:indices.data() length:indices.size() * 4
                                                                options:MTLResourceStorageModeShared];
}

void MeshRenderer::setWaterLook(const std::array<simd_float4, 8>& constants, const DpcFile& world) {
    waterConst_ = constants;
    uint8_t alpha = 0;
    const uint32_t foam = genWorldEnumTexture(world, 0x37, 0), normal = genWorldEnumTexture(world, 0x37, 5);
    if (const DpcFile::Entry* e = foam ? world.find(foam) : nullptr) waterFoam_ = uploadBitmap(world, *e, false, &alpha);
    if (const DpcFile::Entry* e = normal ? world.find(normal) : nullptr) waterNormal_ = uploadBitmap(world, *e, false, &alpha);
    haveWaterLook_ = true;
}

void MeshRenderer::setGrassAtlas(uint32_t bitmapId) {
    uint8_t alpha = 0;
    grassAtlas_ = texture(bitmapId, false, &alpha);
}

void MeshRenderer::setGrass(const std::vector<GrassTuft>& tufts) {
    struct Gpu {
        simd_float4 posSize;
        uint8_t info[4];
        uint8_t colour[4];
    };
    static_assert(sizeof(Gpu) == 32, "matches GrassInstance");
    grassCount_ = tufts.size();
    if (tufts.empty()) return;
    // A fresh buffer each time: frames in flight may still read the previous one.
    std::vector<Gpu> data(tufts.size());
    for (size_t i = 0; i < tufts.size(); ++i) {
        const GrassTuft& t = tufts[i];
        data[i] = {simd_make_float4(t.x, t.y, t.z, t.size), {t.tile, t.windAmp, t.tint, 0}, {t.rgb[0], t.rgb[1], t.rgb[2], t.occlusion}};
    }
    grassBuffer_ = [device_ newBufferWithBytes:data.data() length:data.size() * sizeof(Gpu) options:MTLResourceStorageModeShared];
}

void MeshRenderer::encodeGrass(id<MTLRenderCommandEncoder> enc) {
    static const bool disabled = std::getenv("FUEL_NO_GRASS") != nullptr;  // FUEL_NO_GRASS=1: measure without
    if (disabled || !grass || grassCount_ == 0 || grassAtlas_ == nil || grassPipeline_ == nil) return;
    struct {
        simd_float4 a;
    } params;
    const float t = std::chrono::duration<float>(std::chrono::steady_clock::now() - grassEpoch_).count();
    params.a = simd_make_float4(std::fmod(t, 1000.0f), grassRadius, grassRadius * 0.75f, 0.06f);
    [enc setRenderPipelineState:grassPipeline_];
    [enc setDepthStencilState:depthWrite_];
    [enc setCullMode:MTLCullModeNone];
    [enc setTriangleFillMode:wireframe ? MTLTriangleFillModeLines : MTLTriangleFillModeFill];
    [enc setVertexBuffer:grassBuffer_ offset:0 atIndex:2];
    [enc setVertexBytes:&params length:sizeof params atIndex:3];
    [enc setVertexTexture:groundColour_ != nil ? groundColour_ : white_ atIndex:0];
    [enc setFragmentTexture:grassAtlas_ atIndex:0];
    [enc setFragmentTexture:shadowMap_ atIndex:7];
    [enc drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:0 vertexCount:9 instanceCount:grassCount_];
}

}  // namespace fuel
