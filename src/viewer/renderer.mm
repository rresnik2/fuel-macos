#import "renderer.h"
#import "gpu_profile.h"
#import <MetalFX/MetalFX.h>  // P93: the dynamic resolution's spatial upscaler

#include <chrono>

#include "core/automesh.hpp"  // P94: autoCellHolds
#include "core/vegetation.hpp"  // P94: decorAtt
#include "core/genworld_noise.hpp"
#include "core/surface_patch.hpp"
#include "core/vehicle.hpp"
#include "core/weather.hpp"
#include "core/vehicle_lights.hpp"
#include "core/vehicle_links.hpp"  // PK22
#include "core/upgrade_visuals.hpp"  // [upgrades U5]
#include "viewer/pause_backdrop.hpp"  // PK17
#include "viewer/taa_math.h"         // TAA

#include <algorithm>
#include <filesystem>
#include <fstream>
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

// ---- PK28: FUEL's mip LOD bias. 0x84ca80 sets D3DSAMP_MIPMAPLODBIAS = renderer +0x1104 (-1.0, Renderer_RegisterTweaks
// 0x64faf0) + +0x1100 (0.0) + the bitmap's own +0x24 on every material texture it binds (BindShaderAndSamplers
// 0x855300). FUEL_TEX_BIAS comes from MTLCompileOptions (the two renderer terms; FUEL_LOD_BIAS overrides it).
// TAA: the scene's and the effects' samples read the same value per frame from FrameUniforms.taa.x (texBiasBase_, plus
// log2(render scale) while the temporal upscaler runs); the constant stays for the shadow casters' cut-out test.
#ifndef FUEL_TEX_BIAS
#define FUEL_TEX_BIAS -1.0
#endif
constant float kTexBias = FUEL_TEX_BIAS;
// ---- PK28 end

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
    // P74: the CAR_OCCLUSION omnis (C_OMNI_0_WORLDTOLOCAL per vehicle, Omni_Z::vf26 0x690700): 3 rows each,
    // local_k = dot(row_k.xyz, w) + row_k.w, the box = [-1, 1]^3. lightInfo: x = their count, w = 1 when the
    // sun / moon sprite texture is loaded (fsSky).
    float4 carOcc[48];
    float4 lightInfo;     // y: P77 vehicle omni count
    // P77: the vehicles' headlight omnis (Omni_Z::vf43 0x798050 constants, in world space): per light 4 rows:
    // position (w 1 = gDOmniPos.w), gDOmniDir, gDOmniValue (r^2, 1 / (R^2 - r^2), +0x11c, +0x120), gDOmniColor.
    // ---- PK28: the rows moved to their own buffer (`omnis`, up to 64 lights = FUEL's list +0x3178, 0x6a2ed0); here the
    // decal bitmaps' own LOD bias (0x84ca80, slots 0..5 of fsTerrain's decal path).
    float4 decalBias[2];
    // P92: the headlight clusters (omniMask, kOmniClusters): x tile size in pixels (0 = off: every light is tried),
    // y / z tiles across / down, w depth slices; clusterZ: x the far end of slice 0 (m), y slices per doubling of the
    // view depth, z the far end of the last slice; clusterView: the target's width, height and the view's x / z and
    // y / z at the screen edge (tan of the half angles); camFwd: the view axis (view depth = dot(P - eye, camFwd)).
    float4 cluster;
    float4 clusterZ;
    float4 clusterView;
    float4 camFwd;
    // ---- TAA (taa_render.mm): viewProj without the temporal jitter (= viewProj when temporal upscaling is off), and
    // taa: x the texture LOD bias of the scene's and the effects' samples (PK28's kTexBias, the renderer's -1.0; plus
    // log2(render scale) on a temporal frame), yz the jitter (ndc), w 1 on a temporal frame. The shadow casters' cut-out
    // test (fsShadowCut) keeps the constant: the shadow map's size does not follow the render scale.
    float4x4 viewProjStable;
    float4 taa;
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
    float4 uv0;           // MaterialMat (hlsl_math.h): u' = (u, v, 1).(x, z, uv1.x), v' = (u, v, 1).(y, w, uv1.y)
    float4 uv1;           // xy translation, z ALPHAREF (D3DCMP_GREATER), w unused
    // ---- PK28: the bound bitmaps' own LOD bias (Bitmap_Z +0x24, 0x84ca80), x slot 0 diffuse, y 2 occlusion, z 3 normal,
    // w 4 specular; lodBias2 x 5 normal-local, y 6 dirt. FrameUniforms.taa.x (the renderer's -1.0) is added in the shader.
    float4 lodBias;
    float4 lodBias2;
};

constant uint PaintOnly = 1, HasOcclusion = 2, HasNormal = 4, HasSpecular = 8, HasNormalLocal = 16, HasDirt = 32,
              Reflective = 64, GroundTint = 128, Foliage = 256, TwoSided = 512,
              NoCarOcclusion = 1024,  // P74: skinned batches (scat_skin.vhl writes omni = 0: no car occlusion)
              SkinShade = 2048, HairShade = 4096, EyeShade = 8192,  // PHYSIQUE (port deviation): the custom rider
              Carbon = 16384;  // [upgrades U5] the weight stage's carbon paint (variant bit 16)

// ---- P95: vsMain / fsMain variants (function constants 10..12, all true when not given: the full set) leave out the
// outputs a draw's material never reads: the instance rotation (only object-space normal maps), the ground tint (only
// sprites and GroundTint materials) and the tangent (only tangent-space normal maps). Fewer outputs per vertex is less
// for the tiler to store and the fragments to load; every output that is kept is computed as before.
constant bool kVsInstFC [[function_constant(10)]];
constant bool kVsTintFC [[function_constant(11)]];
constant bool kVsTangentFC [[function_constant(12)]];
constant bool kVsNormalFC [[function_constant(14)]];
constant bool kVsInst = is_function_constant_defined(kVsInstFC) ? kVsInstFC : true;
constant bool kVsTint = is_function_constant_defined(kVsTintFC) ? kVsTintFC : true;
constant bool kVsTangent = is_function_constant_defined(kVsTangentFC) ? kVsTangentFC : true;
constant bool kVsNormal = is_function_constant_defined(kVsNormalFC) ? kVsNormalFC : true;  // lit non-sprite draws only
// [upgrades U5] the carbon paint (a derived vehicle mesh, core/upgrade_visuals): the mesh-space position and normal reach
// fsMain. Off unless given (the full variant 15 and every FUEL draw leave it out).
constant bool kCarbonFC [[function_constant(16)]];
constant bool kCarbon = is_function_constant_defined(kCarbonFC) ? kCarbonFC : false;

struct VertexOut {
    float4 position [[position, invariant]];  // P72: the fade pre-pass and its colour pass must give equal depths
    float3 world;
    float3 normal [[function_constant(kVsNormal)]];
    float4 tangent [[function_constant(kVsTangent)]];
    float2 uv;
    float3 ext;         // scattering at the vertex (scat_mesh.vhl scatteringbase)
    float3 insc;
    float3 m0 [[function_constant(kVsInst)]];  // the instance's rotation (for object-space normal maps)
    float3 m1 [[function_constant(kVsInst)]];
    float3 m2 [[function_constant(kVsInst)]];
    float3 tint [[function_constant(kVsTint)]];  // local ground colour x2, for grey foliage and rock textures
    uint sprite [[flat]];
    float fade [[flat]];  // 1 - the object's fade alpha (Lod_Z / Mesh_Z ComputeFade, p36); 0 = opaque
    float att;            // P94: a plant's decor fade (ground_rotshape.vhl att, per vertex); 1 = none
    float dirt [[flat]];  // PK22: the instance's C_USER0.x + 1 (world column 2 w), 0 = the global dirt
    float3 local [[function_constant(kCarbon)]];    // [upgrades U5] mesh space (the carbon weave and its region)
    float3 lnormal [[function_constant(kCarbon)]];
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
    const float2 fr = fract(tc.xy * float2(ts.x, ts.w));
    float s[3][3];
    // P92: the nine texels by four gathers (each the 2 x 2 block below / left of a texel corner) instead of nine reads:
    // the taps are the centre texel b = floor(uv W) and its neighbours b + (i, j), each clamped to the map like
    // shadowTap (clamp_to_edge). Same texels and comparisons (shadowTap floors uv + i / W, which equals b + i but for
    // float rounding a hair from a texel edge).
    constexpr sampler gs(coord::normalized, filter::nearest, address::clamp_to_edge);
    const float W = float(map.get_width());
    const float2 b = floor(tc.xy * W);  // 0 .. W (tc is saturated): the clamp is the sampler's, per texel
    const float4 g00 = map.gather(gs, b / W, slice);                       // texels (b-1, b) x (b-1, b)
    const float4 g10 = map.gather(gs, (b + float2(1, 0)) / W, slice);       // (b, b+1) x (b-1, b)
    const float4 g01 = map.gather(gs, (b + float2(0, 1)) / W, slice);       // (b-1, b) x (b, b+1)
    const float4 g11 = map.gather(gs, (b + float2(1, 1)) / W, slice);       // (b, b+1) x (b, b+1)
    // gather order: x = (i0, j1), y = (i1, j1), z = (i1, j0), w = (i0, j0).
    const float d = tc.z;
    s[0][0] = step(g00.w, d), s[0][1] = step(g00.z, d), s[0][2] = step(g10.z, d);
    s[1][0] = step(g00.x, d), s[1][1] = step(g00.y, d), s[1][2] = step(g10.y, d);
    s[2][0] = step(g01.x, d), s[2][1] = step(g01.y, d), s[2][2] = step(g11.y, d);
    float sum = 0.0;
    for (int j = 0; j < 2; ++j)
        for (int i = 0; i < 2; ++i) {
            const float a0 = mix(s[j][i], s[j][i + 1], fr.x), a1 = mix(s[j + 1][i], s[j + 1][i + 1], fr.x);
            sum += mix(a0, a1, fr.y);
        }
    return 0.25 * sum;
}

// P93: settings.txt shadow_filter = 2x2 (FrameUniforms fog.x bit 16): one bilinear PCF lookup over the 2 x 2 texels
// around the receiver (texel centres at k + 0.5), the same centre as pcf3x3's tent but half its width: a quarter of
// its gathers. Same texels / clamp convention as pcf3x3 (a gather at a texel corner c returns texels c-1 and c).
float pcf2x2(depth2d_array<float> map, float3 tc, uint slice) {
    constexpr sampler gs(coord::normalized, filter::nearest, address::clamp_to_edge);
    const float W = float(map.get_width());
    const float2 t = tc.xy * W - 0.5;
    const float2 b = floor(t);  // the lower-left texel of the 2 x 2 block (-1 .. W-1; the sampler clamps)
    const float2 fr = t - b;
    const float4 g = map.gather(gs, (b + 1.0) / W, slice);  // texels (b, b+1) x (b, b+1)
    // gather order: x = (i0, j1), y = (i1, j1), z = (i1, j0), w = (i0, j0).
    const float d = tc.z;
    const float a0 = mix(step(g.w, d), step(g.z, d), fr.x), a1 = mix(step(g.x, d), step(g.y, d), fr.x);
    return mix(a0, a1, fr.y);
}

// World -> clip, camera-relative (FUEL's WorldToViewVertex works from the camera too). viewProj * (w, 1) in float
// cancels ~40 km world terms in the z and w rows separately: a few mm of error each, which at a 0.06 m near plane
// moves depth more than 0.2 m at 40 m, so a billboard's poster lost to the planks 0.2 m behind it on some frames
// (P60). The rotation part takes the small offset w - eye; the eye's own clip position is (0, 0, near * q, 0).
float4 fuelClip(constant FrameUniforms& f, float3 w) {
    float4 c = f.viewProj * float4(w - f.eye.xyz, 0.0);
    c.z += f.fog.z * f.fog.w;
    return c;
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
    const float s = (uint(f.fog.x) & 16u) ? pcf2x2(map, tc, uint(i)) : pcf3x3(map, tc, uint(i), f.shadowSize);  // P93
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

// scatteringbase: what the camera sees of the air between it and the world point p. `sunDir` = the shader's
// sunlightdir argument: C_SCATTERCONSTANTS3 (the sun position S+0x4c) for every shader but sfx.vhl's vh_billboard_soft,
// vh_cloud, vh_bboard_soft_static and vh_lightning, which pass DLIGHT_DIR (sfx.vhl:108 / 207 / 290 / 393). DUSK: the
// two differ since SC3 is the un-lifted S+0x4c (before, the port used DLIGHT_DIR for SC3, so they were equal).
Scatter fuelScatteringDir(constant FrameUniforms& f, float3 p, float3 sunDir) {
    if (uint(f.fog.x) & 2u) return Scatter{float3(1.0), float3(0.0)};
    float3 look = f.eye.xyz - p;
    const float len = length(look);
    look /= max(len, 1e-6);
    const float cosT = dot(look, sunDir);
    const float density = fuelScatterIntegral(f, len, 0.25 + max(p.y, 0.0), f.eye.y);
    return fuelScatterFinish(f, min(look.y, 0.0), density, cosT);
}
Scatter fuelScattering(constant FrameUniforms& f, float3 p) { return fuelScatteringDir(f, p, f.scat[SC3].xyz); }

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

// ---- PHYSIQUE (port deviation): the player's custom rider within FUEL's lighting terms (same sun, sky, omni, shadow and
// gamma-space sum as lightingmaterial()). Skin: the sun's Lambert term mixed with a wrapped one, the difference tinted red
// (light scattered under the skin at the terminator), a soft sky-lit sheen at grazing angles and a broad + tight Phong
// pair. Hair: Kajiya-Kay-like highlights along the strands (the bitangent, v of the hair's uv). Eyes: a sharp wet highlight.
float3 physiqueLighting(uint features, float3 n, float3 L, float3 K, float3 V, float4 tangent, float3 sunCol, float3 skyCol,
                        float3 vtx, float3 specCol, float power, float shadow, float3 omni, float occ, float4 diffuse) {
    const float ndl = dot(n, L);
    const float sunVis = fuelSunlighting(shadow, occ);
    float skydot;
    const float sky = fuelSkylighting(n, K, skydot);
    const float3 r = dot(n, V) * n * 2.0 - V;
    float3 lit;
    float3 specular;
    if (features & SkinShade) {
        const float lambert = saturate(ndl);
        const float wrap = saturate((ndl + 0.42) / 1.42);
        const float3 sss = float3(0.62, 0.20, 0.12) * saturate(wrap - lambert) * 1.6;
        const float3 sunTerm = (mix(float3(lambert), float3(wrap), 0.32) + sss) * sunVis;
        const float fres = pow(1.0 - saturate(dot(n, V)), 4.0);
        lit = diffuse.rgb * (sky * skyCol * (1.0 + 0.35 * fres) + sunTerm * sunCol + omni);
        const float broad = pow(max(saturate(dot(r, L)), 1e-4), max(power * 0.18, 2.0)) * 0.35;
        const float tight = pow(max(saturate(dot(r, L)), 1e-4), power);
        specular = (broad + tight) * shadow * specCol * sunCol + fres * 0.10 * specCol * skyCol * occ;
    } else if (features & HairShade) {
        const float3 T = normalize(cross(n, tangent.xyz) * tangent.w);  // the strand direction (uv v)
        const float3 H = normalize(L + V);
        const float th = dot(T, H);
        const float aniso = pow(sqrt(saturate(1.0 - th * th)), power);
        const float th2 = dot(T, normalize(H + n * 0.25));
        const float aniso2 = pow(sqrt(saturate(1.0 - th2 * th2)), power * 0.35) * 0.35;
        const float sunTerm = saturate(ndl * 0.75 + 0.25) * sunVis;
        lit = diffuse.rgb * (sky * skyCol + sunTerm * sunCol + omni);
        specular = (aniso + aniso2 * vtx * 2.0) * shadow * saturate(ndl + 0.3) * specCol * sunCol;
    } else {  // EyeShade
        lit = diffuse.rgb * (sky * skyCol + saturate(ndl) * sunVis * sunCol + omni);
        const float s = pow(max(saturate(dot(r, L)), 1e-4), power) * shadow;
        specular = s * specCol * sunCol + specCol * skyCol * pow(1.0 - saturate(dot(n, V)), 3.0) * 0.4 * occ;
    }
    return vtx * lit + specular;
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

// ---- P74: FUEL's car occlusion omni (Vehicle_CarOcclusion 0x5cc410, notes p74-lighting.md 1). bOmni variants of
// scat_mesh / ground: VS omni = mul(w, C_OMNI_0_WORLDTOLOCAL) * 0.5 + 0.5, PS occlusion *= 1 - f(x) f(z) f(y) with
// f = sProjSpotMap0 at (t, 0): the OMBREVOITURES Omni_Z's spot map (a ramp 0 at the box faces, 1 inside). FUEL reads
// slot 0 only, i.e. one omni per object draw (the first gathered, 0x798640 / 0x798970); the port takes the first box
// holding the pixel (vehicle order). Port: mip 0 (no derivatives inside the loop); FUEL's sampler picks the mip.
float carOcclusion(constant FrameUniforms& f, float3 w, texture2d<float> spot) {
    constexpr sampler s(coord::normalized, filter::linear, address::clamp_to_edge);
    const int n = int(f.lightInfo.x);
    for (int i = 0; i < n; ++i) {
        const float4 r0 = f.carOcc[3 * i], r1 = f.carOcc[3 * i + 1], r2 = f.carOcc[3 * i + 2];
        const float3 l = float3(dot(r0.xyz, w) + r0.w, dot(r1.xyz, w) + r1.w, dot(r2.xyz, w) + r2.w);
        if (any(abs(l) >= 1.0)) continue;  // outside the box f = 0 (the ramp's ends): no change
        const float3 t = l * 0.5 + 0.5;
        float v = spot.sample(s, float2(t.x, 0.0), level(0.0)).r;
        v *= spot.sample(s, float2(t.z, 0.0), level(0.0)).r;
        v *= spot.sample(s, float2(t.y, 0.0), level(0.0)).r;
        return 1.0 - v;
    }
    return 1.0;
}

// ---- P77: the deferred omnis (Omni_Z::vf43 0x798050 + omni.phl) evaluated per pixel instead of through a G-buffer
// and a light buffer: the same sum lands in lighting()'s `omni`. Everything in world space; omni.phl's v.y is a view-
// space value, taken on the camera's up axis. P = the lit point, N = the lit normal (FUEL's G-buffer normal is the
// same bump-mapped normal).
float3 fuelOmni(float3 P, float3 N, float3 pos, float3 dir, float4 val, float3 col, float3 viewUp) {
    const float3 o = pos - P;
    const float d = dot(o, o);
    const float3 v = o * rsqrt(d);
    const float fd = dot(dir, v);
    float n = saturate(dot(dir, N));
    if (fd < 0.0) n = saturate(dot(v, N));
    float r = saturate(1.0 - (d - val.x) * val.y);
    r *= r; r *= r; r *= r;
    const float up = saturate(0.33 - dot(v, viewUp) * 0.67 - d * 0.5);
    const float c = 2.0 * up + (1.0 - up * 2.0) * saturate(1.0 - (fd * val.z + val.w));  // BACKOMNIPOWER 2
    return r * n * c * col;
}

float3 fuelOmnis(constant FrameUniforms& f, constant float4* omni, float3 P, float3 N) {  // PK28: rows in `omni` (64 lights)
    float3 sum = float3(0.0);
    const int n = int(f.lightInfo.y);
    for (int i = 0; i < n; ++i) {
        const float3 o = omni[4 * i].xyz - P;
        const float4 val = omni[4 * i + 2];
        if ((dot(o, o) - val.x) * val.y >= 1.0) continue;  // past R: the range term is 0 (vf43 culls by the sphere too)
        sum += fuelOmni(P, N, omni[4 * i].xyz, omni[4 * i + 1].xyz, val, omni[4 * i + 3].rgb, f.camUp.xyz) * omni[4 * i].w;
    }
    return sum;
}

// ---- P92: clustered headlights. Every pixel used to evaluate all 16 omnis (7.6 ms of a 3420x2224 night frame): their
// range is 250 m, so nearly every pixel is in range of all of them, but they are spots: omni.phl's cone term
// saturate(1 - (fd z + w)) is 0 more than 45 degrees off the light's axis (gDOmniValue z = -3.86, w = 3.73), and the
// "up" term is 0 beyond 1.42 m of the light (d^2 x 0.5 >= 1). kOmniClusters marks, per 64-pixel screen tile and view-
// depth slice, the omnis whose range sphere, cone or near sphere can reach the cluster (conservatively: the cluster's
// bounding sphere, two pixels and 0.03 rad of margin); a pixel evaluates only those, in the same ascending order and
// with the same terms, so the sum is fuelOmnis's: an omni left out adds exactly 0 at every point of the cluster.
ulong omniMask(constant FrameUniforms& f, const device ulong* clusters, float2 pix, float3 P) {  // PK28: 64 lights
    if (f.cluster.x <= 0.0) return ~0ul;
    const uint tx = min(uint(pix.x / f.cluster.x), uint(f.cluster.y) - 1u);
    const uint ty = min(uint(pix.y / f.cluster.x), uint(f.cluster.z) - 1u);
    const float z = dot(P - f.eye.xyz, f.camFwd.xyz);
    const int s = clamp(int(floor(log2(max(z, 1e-3) / f.clusterZ.x) * f.clusterZ.y)), 0, int(f.cluster.w) - 1);
    return clusters[(uint(s) * uint(f.cluster.z) + ty) * uint(f.cluster.y) + tx];
}

float3 fuelOmnisMasked(constant FrameUniforms& f, constant float4* omni, float3 P, float3 N, ulong mask) {
    float3 sum = float3(0.0);
    const int n = int(f.lightInfo.y);
    // PK28: the set bits in ascending order (the clusters leave out the lights that cannot reach the pixel, P92)
    ulong m = n >= 64 ? mask : (mask & ((1ul << ulong(n)) - 1ul));
    while (m != 0ul) {
        const int i = int(ctz(m));
        m &= m - 1ul;
        const float3 o = omni[4 * i].xyz - P;
        const float4 val = omni[4 * i + 2];
        const float d = dot(o, o);
        if ((d - val.x) * val.y >= 1.0) continue;  // past R: the range term is 0 (vf43 culls by the sphere too)
        // P92: fuelOmni's cone and "up" terms first: both 0 (off the spot's cone, more than 1.42 m from the light)
        // makes its product 0, so the light adds nothing; skip the rest of it.
        const float3 v = o * rsqrt(d);
        const float cone = 1.0 - (dot(omni[4 * i + 1].xyz, v) * val.z + val.w);
        const float up = 0.33 - dot(v, f.camUp.xyz) * 0.67 - d * 0.5;
        if (cone <= 0.0 && up <= 0.0) continue;
        sum += fuelOmni(P, N, omni[4 * i].xyz, omni[4 * i + 1].xyz, val, omni[4 * i + 3].rgb, f.camUp.xyz) * omni[4 * i].w;
    }
    return sum;
}

kernel void kOmniClusters(constant FrameUniforms& f [[buffer(0)]], device ulong* out [[buffer(1)]], constant float4* omni [[buffer(2)]],
                          uint3 gid [[thread_position_in_grid]]) {  // PK28: 64 lights (ulong masks, rows in buffer 2)
    const uint tx = uint(f.cluster.y), ty = uint(f.cluster.z), ns = uint(f.cluster.w);
    if (gid.x >= tx || gid.y >= ty || gid.z >= ns) return;
    // The cluster: its depth range and its pixels (2 px of margin) as view slopes x / z, y / z.
    const float za = gid.z == 0 ? 0.0 : f.clusterZ.x * exp2(float(gid.z) / f.clusterZ.y);
    const float zb = gid.z + 1 >= ns ? f.clusterZ.z : f.clusterZ.x * exp2(float(gid.z + 1) / f.clusterZ.y);
    const float T = f.cluster.x, W = f.clusterView.x, H = f.clusterView.y;
    const float sx0 = ((float(gid.x) * T - 2.0) / W * 2.0 - 1.0) * f.clusterView.z, sx1 = ((float(gid.x + 1) * T + 2.0) / W * 2.0 - 1.0) * f.clusterView.z;
    const float sy0 = (1.0 - (float(gid.y + 1) * T + 2.0) / H * 2.0) * f.clusterView.w, sy1 = (1.0 - (float(gid.y) * T - 2.0) / H * 2.0) * f.clusterView.w;
    const float3 R = f.camRight.xyz, U = f.camUp.xyz, F = f.camFwd.xyz;
    float3 c[8];
    for (int k = 0; k < 8; ++k) {
        const float z = (k & 4) ? zb : za;
        c[k] = (R * ((k & 1) ? sx1 : sx0) + U * ((k & 2) ? sy1 : sy0) + F) * z;  // eye-relative world
    }
    float3 centre = float3(0.0);
    for (int k = 0; k < 8; ++k) centre += c[k];
    centre *= 0.125;
    float rho = 0.0;
    for (int k = 0; k < 8; ++k) rho = max(rho, distance(centre, c[k]));
    rho = rho * 1.01 + 0.05;
    ulong mask = 0;
    const int n = min(int(f.lightInfo.y), 64);
    for (int i = 0; i < n; ++i) {
        const ulong bit = 1ul << ulong(i);
        const float4 val = omni[4 * i + 2];
        const float3 L = omni[4 * i].xyz - f.eye.xyz;  // eye-relative
        const float3 V = centre - L;
        const float lenV = length(V);
        if (!(val.y > 0.0) || !isfinite(val.y) || lenV <= rho + 1.5) {  // no usable range, or the near ("up") term
            mask |= bit;
            continue;
        }
        const float range = sqrt(max(val.x + 1.0 / val.y, 0.0)) * 1.01 + 0.05;
        if (lenV - rho > range) continue;  // out of range everywhere in the cluster
        const float lz = dot(L, F);
        if (lz + range < za || lz - range > zb) continue;
        // The cone: fd = dot(dir, v) > (1 - w) / z (z < 0), v towards the light, i.e. within acos(ct / |dir|) of -dir
        // from the light. z >= 0 would light the outside of a cone: kept.
        const float3 dir = omni[4 * i + 1].xyz;
        const float dl = length(dir);
        if (val.z >= 0.0 || dl <= 1e-6) {
            mask |= bit;
            continue;
        }
        const float ct = clamp((1.0 - val.w) / val.z / dl, -1.0, 1.0);
        const float theta = acos(ct) + asin(min(rho / lenV, 1.0)) + 0.03;
        if (theta >= 3.14159 || dot(V, -dir / dl) / lenV > cos(theta)) mask |= bit;
    }
    out[(gid.z * ty + gid.y) * tx + gid.x] = mask;
}

// The world palette is a bright spring green; the port's old heuristic colour map (no decoded
// ground colours) is pulled towards olive. Not used with FUEL's decoded colours.
float3 groundGrade(float3 c) {
    float l = dot(c, float3(0.30, 0.59, 0.11));
    float green = saturate((c.g - max(c.r, c.b)) / max(c.g, 1e-3) * 2.0);
    return mix(float3(l), c, mix(0.75, 0.45, green)) * float3(0.92, 0.86, 0.76);
}

// ---- P94: FUEL's decor kinds and fades for a plant instance (core/vegetation.hpp decorAtt, the same rule): the decor
// quadtree leaf under the plant's pivot (12288 m roots split while the eye is within 2.0625 x the size, 24 m leaves)
// picks which ground element kinds hold it; each fades with the horizontal eye distance (ground_rotshape.vhl):
// kind 1 = mesh (chunks < 60: fade out 132.5..162.5 m), 2 / 3 = tree / bush card (chunks 30..60: kind 0x16, fade in
// 117.5..147.5 m; 60 up to 600 / 100: kinds 0x14 / 0x15, fade out 1200..1300 / 250..300 m). 0 = not queued.
float decorLeaf(float2 p, float2 eye) {
    float size = 12288.0;
    float2 o = round(p / 12288.0) * 12288.0;
    while (size > 24.01) {
        const float r = size * 2.0625;
        const float2 v = o - eye;
        if (dot(v, v) >= r * r) break;
        o += select(float2(-0.25), float2(0.25), p >= o) * size;
        size *= 0.5;
    }
    return size;
}
float decorAtt(uint kind, float C, float dxz) {
    if (kind == 1u) return C < 60.0 ? 1.0 - saturate((dxz - 132.5) / 30.0) : 0.0;
    if (C < 30.0) return 0.0;
    if (C < 60.0) return 1.0 - saturate((147.5 - dxz) / 30.0);
    if (kind == 2u) return C < 600.0 ? 1.0 - saturate((dxz - 1200.0) / 100.0) : 0.0;
    return C < 100.0 ? 1.0 - saturate((dxz - 250.0) / 50.0) : 0.0;
}
// ---- P94 end

// Instanced: one world matrix per instance in buffer 2.
vertex VertexOut vsMain(VertexIn in [[stage_in]], constant FrameUniforms& f [[buffer(1)]],
                        const device float4x4* models [[buffer(2)]], constant uint& sprite [[buffer(3)]],
                        const device packed_float3* morph [[buffer(4)]],
                        texture2d<float> groundColour [[texture(0)]], uint iid [[instance_id]], uint vid [[vertex_id]]) {
    // Placed objects carry 1 - fade alpha in column 0's w (0 for every affine world matrix).
    float4x4 model = models[iid];
    VertexOut o;
    o.fade = model[0].w;
    model[0].w = 0.0;
    const uint decor = uint(model[1].w);  // P94: a plant's decor kind (drive.mm buildBackground), 0 = none
    model[1].w = 0.0;
    o.dirt = model[2].w;  // PK22: a vehicle's own dirt (0x5dd8c0 / 0x5dd300)
    model[2].w = 0.0;
    float3 tint = float3(1.0);
    o.sprite = sprite & 1u;
    // ---- P85: the damage morph (MeshPc_Z::Draw 0x848880, vertex type 8): the instance's morph VB of 0x7cb550 as
    // the second stream with weight 1 (bit 1 of the draw flags); FUEL's static weight (+0x4c) is never set: 0.
    if ((sprite & 2u) != 0) in.position += float3(morph[vid]);
    // ---- P85 end
    if (kVsTint && f.fog.y > 0.0) {
        // Like FUEL's ground, foliage and rocks take the palette colour under them
        // (ground x 2; the fragment shader decides how to apply it).
        constexpr sampler lin(coord::normalized, filter::linear, address::clamp_to_edge);
        const float3 gc = groundColour.sample(lin, (model[3].xz + 70000.0) / 140000.0, level(0)).rgb;
        tint = (f.groundFar.w > 0.0 ? gc : groundGrade(gc)) * 2.0;
    }
    if (kVsTint) o.tint = tint;
    if (kVsInst) {
        o.m0 = model[0].xyz;
        o.m1 = model[1].xyz;
        o.m2 = model[2].xyz;
    }
    float4 w = model * float4(in.position, 1.0);
    // ---- P94: the decor fade at the vertex (the sprite's pivot), FUEL's ground_rotshape att; a sprite with att <= 0.5
    // is not expanded ("shrink le rotshape si l'alpha est trop petit"), and an instance its chunk does not queue, or a
    // mesh past its fade by more than a tree's reach, is dropped whole.
    o.att = 1.0;
    bool dropped = false;
    if (decor != 0u) {
        const float C = decorLeaf(model[3].xz, f.eye.xz);
        o.att = decorAtt(decor, C, distance(w.xz, f.eye.xz));
        dropped = decorAtt(decor, C, max(distance(model[3].xz, f.eye.xz) - 40.0, 0.0)) <= 0.5 &&
                  decorAtt(decor, C, distance(model[3].xz, f.eye.xz) + 40.0) <= 0.5;
    }
    // ---- P94 end
    if ((sprite & 1u) != 0 && o.att > 0.5) {
        // Leaf and impostor sprites: all four corners share one position; expand
        // them towards the camera by the stored offsets (scaled with the instance).
        float scale = length(model[1].xyz);
        w.xyz += (f.camRight.xyz * in.corner.x + f.camUp.xyz * in.corner.y) * scale;
    }
    o.position = dropped ? float4(2.0, 2.0, 0.5, 1.0) : fuelClip(f, w.xyz);  // P94: dropped = outside the clip volume
    o.world = w.xyz;
    if (kVsNormal) o.normal = (model * float4(in.normal.xyz * 2.0 - 1.0, 0.0)).xyz;
    if (kVsTangent) o.tangent = float4((model * float4(in.tangent.xyz * 2.0 - 1.0, 0.0)).xyz, in.tangent.w > 0.5 ? 1.0 : -1.0);
    o.uv = in.uv;
    if (kCarbon) {  // [upgrades U5]
        o.local = in.position;
        o.lnormal = in.normal.xyz * 2.0 - 1.0;
    }
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
    // ---- P94: a plant casts while its decor kind shows at its pivot (vsMain decorAtt; port: per instance)
    const uint decor = uint(model[1].w);
    model[1].w = 0.0;
    model[2].w = 0.0;  // PK22: the dirt slot (vsMain)
    if (decor != 0u && decorAtt(decor, decorLeaf(model[3].xz, su.eye.xz), distance(model[3].xz, su.eye.xz)) <= 0.5) {
        ShadowOut gone;
        gone.position = float4(2.0, 2.0, 0.5, 1.0);
        gone.uv = in.uv;
        return gone;
    }
    // ---- P94 end
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
    o.position = su.lightViewProj * float4(w.xyz - su.eye.xyz, 1.0);  // P72: camera-relative (P60's fuelClip for casters)
    o.uv = in.uv;
    return o;
}

fragment void fsShadowCut(ShadowOut in [[stage_in]], texture2d<float> tDiffuse [[texture(0)]], sampler smp [[sampler(0)]],
                          constant float2& halfAtlas [[buffer(0)]]) {  // PK28: y = the diffuse bitmap's own LOD bias
    const float2 uv = halfAtlas.x != 0.0 ? float2(in.uv.x * 0.5, in.uv.y) : in.uv;
    if (tDiffuse.sample(smp, uv, bias(kTexBias + halfAtlas.y)).a - 0.5 < 0.0) discard_fragment();
}

// ---- [upgrades U5] a 2 x 2 twill carbon weave (gamma space) in mesh space, triplanar on the mesh normal: tows of `tow`
// metres, each lit across its width; faded to the weave's mean where a tow is under about a pixel.
float3 carbonWeave(float3 p, float3 n, float tow) {
    const float3 an = abs(n);
    const float2 q = (an.x > an.y && an.x > an.z ? p.zy : (an.y > an.z ? p.xz : p.xy)) / tow;
    const float2 cell = floor(q), fr = fract(q);
    const bool alongU = fmod(cell.x + cell.y + 1024.0, 4.0) < 2.0;
    const float across = alongU ? fr.y : fr.x, along = alongU ? fr.x : fr.y;
    const float lit = sin(3.14159265 * across) * (0.82 + 0.18 * sin(3.14159265 * along));
    const float3 c = mix(float3(0.030, 0.031, 0.034), float3(0.150, 0.152, 0.160), lit * (alongU ? 1.0 : 0.72));
    const float2 w = fwidth(q);
    return mix(float3(0.070, 0.071, 0.076), c, saturate(1.5 - 2.5 * max(w.x, w.y)));  // gone at 0.6 tow per pixel (no moire)
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
                       texturecube<float> globalCube [[texture(8)]],
                       texture2d<float> carSpot [[texture(14)]],
                       sampler smp [[sampler(0)]],
                       const device ulong* omniClusters [[buffer(5)]], constant float4* omnis [[buffer(6)]]) {  // P92 (PK28: 64)
    constexpr sampler cubeSmp(filter::linear, address::clamp_to_edge);
    // GetDiffuseUV = MaterialMat(uv0) (scat_mesh.vhl): the material's UV transform (static, or a MaterialAnim_Z's).
    const float2 uv = float2(dot(float3(in.uv, 1.0), float3(m.uv0.xz, m.uv1.x)), dot(float3(in.uv, 1.0), float3(m.uv0.yw, m.uv1.y)));
    // Codes 17 and 20 use sheets whose left half is the intact part and the
    // right half the damaged one; draw the intact half (bBroken, break amount 0).
    float2 uvHalf = (m.code == 17 || m.code == 20) ? float2(uv.x * 0.5, uv.y) : uv;
    if (in.fade > 0.0 && m.uv1.w == 0.0) {
        // (P72: fading objects now take FUEL's pre-pass + blended path, uv1.w = 1, below; this dither is a fallback.)
        // Port: FUEL fades objects over the last 20 m of their view distance (view+0x160 alpha); the
        // port dissolves them with a 4x4 ordered dither instead of sorting blended objects.
        const uint2 q = uint2(in.position.xy) & 3u;
        const float bayer[16] = {0, 8, 2, 10, 12, 4, 14, 6, 3, 11, 1, 9, 15, 7, 13, 5};
        if ((bayer[q.y * 4 + q.x] + 0.5) / 16.0 < in.fade) discard_fragment();
    }
    float4 d = tDiffuse.sample(smp, uvHalf, bias(f.taa.x + m.lodBias.x));  // PK28: 0x84ca80 LOD bias
    if (kVsTint) {  // P95: else no draw of this variant is a sprite or a GroundTint material
        if (in.sprite != 0) {
            // Port: leaves take the ground's hue with only mild darkening (some leaf textures are dark).
            const float lum = max(dot(in.tint, float3(0.3, 0.55, 0.15)), 1e-3);
            d.rgb *= in.tint / lum * clamp(lum, 0.6, 1.0);
        }
        else if (m.features & GroundTint)  // port: rock, a muted version of the ground colour
            d.rgb *= mix(float3(dot(in.tint, float3(0.3, 0.55, 0.15))), in.tint, 0.35);
    }
    // [upgrades U5] the carbon region of a derived vehicle mesh: S1 the bonnet, S2 + every top surface, S3 all
    bool carbonHere = false;
    float3 cw = float3(0.0);
    if (kCarbon && (m.features & Carbon)) {
        const float3 ln = normalize(in.lnormal);
        const float level = m.lodBias2.z;
        carbonHere = level >= 3.0 || (ln.y > 0.35 && (level >= 2.0 || in.local.z > m.lodBias2.w));
        if (carbonHere) cw = level > 8.0 ? float3(1.0, 0.0, 0.0) : carbonWeave(in.local, ln, 0.012);  // (9: FUEL_U5_CARBON_DEBUG)
    }
    if (m.code == 17 || m.code == 19) {
        // bColoured: lerp(gGouraudDefault (the paint), tex, tex.a).
        if (kCarbon && carbonHere) {
            const float tex = (m.features & PaintOnly) ? 0.0 : d.a;
            d.rgb = mix(cw, mix(d.rgb, cw, 0.7), tex);  // [upgrades U5] the paint becomes carbon; decals / liveries tinted toward it
        } else {
            d.rgb = mix(f.paint.rgb, d.rgb, (m.features & PaintOnly) ? 0.0 : d.a);
        }
        d.a = 1.0;
    } else if (kCarbon && carbonHere) {  // [upgrades U5] a textured body skin: carbon, the texture's shading kept
        const float l = dot(d.rgb, float3(0.30, 0.59, 0.11));
        d.rgb = mix(d.rgb, cw * (0.45 + 1.1 * l), 0.85);
    }
    float dirtiness = 0.0;
    if (m.features & HasDirt) {
        float4 dirt = tDirt.sample(smp, uv, bias(f.taa.x + m.lodBias2.y));  // PK28
        // PK22: C_USER0.x = the vehicle's live dirt (in.dirt - 1); the global value (--dirt, the d key) wins when set.
        const float dirtiness0 = f.params.x > 0.0 || in.dirt <= 0.0 ? f.params.x : in.dirt - 1.0;
        float edge = 1.0 - dirtiness0;
        dirtiness = smoothstep(edge, edge + 0.15, dirt.a);
        d.rgb = mix(d.rgb, dirt.rgb, dirtiness);
    }
    float4 c = d * float4(1.0, 1.0, 1.0, m.diffuse.a);
    // ---- P94: a plant's decor fade (ground_rotshape ph_base: vcolor.a = att times the texture alpha, alpha test 0.5)
    if (in.att < 1.0) {
        if ((m.alphaMode != 0 ? c.a : 1.0) * in.att <= 0.5) discard_fragment();
        if (m.alphaMode == 1) c.a *= in.att;  // the cut-out edge (coverage) from the faded alpha
    }
    // ---- P94 end
    float alpha = m.alphaMode == 2 ? c.a : 1.0;
    // Every code-0/18/20 draw keeps D3D's alpha test (pass bit 0x20 clear): ALPHAFUNC GREATER (0x838160), ALPHAREF 0,
    // or 128 with material flag 0x100000 (0x848880 / 0x858810). Cut-outs (mode 1) do it by coverage below.
    if (m.alphaMode == 2 && c.a <= m.uv1.z) discard_fragment();
    if (m.alphaMode == 1) {
        // Cut-outs: the 0.5 edge sharpened to about a pixel; with MSAA the alpha
        // becomes coverage (alpha to coverage), so edges stay soft and depth sorts them.
        alpha = saturate((c.a - 0.5) / max(fwidth(c.a), 1e-4) + 0.5);
        // Leaves at the lens would fill the screen: thin them out within 1.5..4 m.
        if (m.features & Foliage) alpha *= saturate((distance(in.world, f.eye.xyz) - 1.5) / 2.5);
        if (alpha <= 0.0) discard_fragment();
    }
    // P72: a fading object's colour pass (0x848880 pass bits 0x1d): the view alpha (view+0x160, 0x84ec60 -> the
    // material constant) scales the output alpha, blended SRCALPHA / INVSRCALPHA over its own depth pre-pass.
    if (m.uv1.w != 0.0) alpha *= 1.0 - in.fade;
    // ---- PK28: materials with flag 0x1000 / 0x2000 are not unlit: the constant 0x84f060 / 0x84f120 zero is MTL_ADDSOUS_NULL
    // (VS_Const.h c[C_MTL_CONTEXT+2].w, "==0.f if Additif or Soustractif"), which scat_mesh / scat_skin / scat_rotshape
    // .vhl multiply into the inscatter only; the lighting is the same as every material's.
    const float3 insc = m.lit == 0 ? float3(0.0) : in.insc;
    // ---- PK28 end

    const float shadow = fuelShadow(shadowMap, in.world, f);
    const float3 sunCol = f.sunColour.rgb, L = f.lightDir.xyz, K = f.skyDir.xyz;
    if (in.sprite != 0) {
        // scat_rotshape: normal up; sky = DSKY_COLOR + DSKY_AMBIENT (K.y/2 + 1/2), sunlit = sky + sun sat(L.y),
        // both x MTL_DIFFUSE x GOURAUD x extinction; lerp(sky, sunlit, sunlighting(shadow, 1)) + emissive.
        const float3 sky = f.skyColour.rgb + f.skyAmbient.rgb * (K.y * 0.5 + 0.5);
        const float3 sunlit = sky + sunCol * saturate(L.y);
        const float3 vcol = m.diffuse.rgb * in.ext;
        const float3 emis = m.emissive.rgb * f.lightDir.w * in.ext;
        const float3 rgb = c.rgb * (mix(sky, sunlit, fuelSunlighting(shadow, 1.0)) * vcol + emis) + insc;  // PK28: x MTL_ADDSOUS_NULL
        return float4(rgb, alpha);
    }

    // Some foliage stores no normal at all: fall back to up.
    if (!kVsNormal) return float4(0.0);  // P95: never reached (the variant is only for sprites and unlit materials)
    const float3 n0 = dot(in.normal, in.normal) > 1e-8 ? normalize(in.normal) : float3(0, 1, 0);
    // FUEL's two-sided draws (CULLMODE none) light back faces with the vertex normal as stored (ph_base reads no
    // VFACE); the port's flip stays for the viewer's no-cull mode on single-sided materials.
    float3 N = (m.features & TwoSided) ? n0 : n0 * (front ? 1.0 : -1.0);
    if (kVsInst && (m.features & HasNormalLocal)) {  // P95: else no draw of this variant has the feature
        // bNormalLocal: object-space normal map (compressed: .xyw).
        const float3 nl = tNormalLocal.sample(smp, uvHalf, bias(f.taa.x + m.lodBias2.x)).xyw * 2.0 - 1.0;  // PK28
        N = normalize(in.m0 * nl.x + in.m1 * nl.y + in.m2 * nl.z);
    } else if (kVsTangent && (m.features & HasNormal)) {
        // bNormal: tangent-space normal map, x in alpha and y in green, z rebuilt.
        float2 xy = tNormal.sample(smp, uvHalf, bias(f.taa.x + m.lodBias.z)).wy * 2.0 - 1.0;  // PK28
        float3 T = in.tangent.xyz - N * dot(N, in.tangent.xyz);
        T = dot(T, T) > 1e-8 ? normalize(T) : normalize(cross(N, abs(N.y) < 0.9 ? float3(0, 1, 0) : float3(1, 0, 0)));
        float3 B = cross(N, T) * in.tangent.w;
        N = normalize(T * xy.x + B * xy.y + N * sqrt(saturate(1.0 - dot(xy, xy))));
    }
    // occlusion = sOcclusion x gDLight.Ambient.w (1); skylight = lerp(DSKY_AMBIENT, DSKY_COLOR, occlusion).
    float occ = (m.features & HasOcclusion) ? tOcclusion.sample(smp, uv, bias(f.taa.x + m.lodBias.y)).r : 1.0;  // PK28
    // P74: bOmni (scat_mesh.phl): occlusion *= 1 - f f f before the skylight, the specular and sunlighting use it.
    if (!(m.features & NoCarOcclusion)) occ *= carOcclusion(f, in.world, carSpot);
    const float3 skyCol = mix(f.skyAmbient.rgb, f.skyColour.rgb, occ);
    float4 spec = m.specular * ((m.features & HasSpecular) ? tSpecular.sample(smp, uv, bias(f.taa.x + m.lodBias.w)) : float4(1.0));  // PK28
    if (m.features & HasOcclusion) spec.rgb *= occ;
    spec.rgb *= shadow * 0.5 + 0.5;
    spec.rgb *= 1.0 - dirtiness;
    const float3 V = normalize(f.eye.xyz - in.world);
    const float3 omni = fuelOmnisMasked(f, omnis, in.world, N, omniMask(f, omniClusters, in.position.xy, in.world));  // P77 (P92: clustered)
    float3 rgb;
    if (m.features & Reflective) {
        // bCubemap: Fresnel(FallOff) x 2 (r + g + b) of the specular, x (0.33 shadow + 0.67); the reflection is
        // sGlobalCube (GetCubeMap; the port's cube is in world axes, so no z flip), rendered by encodeCube.
        const float fres = mix(m.fresnel.x, m.fresnel.y, pow(1.0 - saturate(dot(V, N)), 5.0));
        float infl = saturate(fres * dot(spec.rgb, float3(2.0)));
        if (kCarbon && carbonHere) infl *= 0.12;  // [upgrades U5] carbon under a clear coat: far less mirror than paint
        infl *= shadow * 0.33 + 0.67;
        float3 r = reflect(-V, N);
        if (uint(f.fog.x) & 32u) r.z = -r.z;  // ---- PK20: a game cube (CUBEMAP_GARAGE): GetCubeMap's uvw.z = -uvw.z
        float3 cube = globalCube.sample(cubeSmp, r).rgb;
        if (uint(f.fog.x) & 8u) return float4(globalCube.sample(cubeSmp, -V).rgb, 1.0);  // FUEL_CUBE_DEBUG: look through
        rgb = fuelCubeLighting(N, L, K, V, sunCol, skyCol, c.rgb, spec.rgb, spec.a, cube, infl, shadow, omni, occ);
    } else if (m.features & (SkinShade | HairShade | EyeShade)) {
        rgb = physiqueLighting(m.features, N, L, K, V, kVsTangent ? in.tangent : float4(1, 0, 0, 1), sunCol, skyCol, c.rgb, spec.rgb, spec.a, shadow, omni, occ, m.diffuse);
    } else {
        rgb = fuelLightingMaterial(N, L, K, V, sunCol, skyCol, c.rgb, spec.rgb, spec.a, shadow, omni, occ, m.diffuse,
                                   m.emissive.rgb);
    }
    return float4(rgb * in.ext + insc, alpha);  // PK28: x MTL_ADDSOUS_NULL
}

struct GroundOut {
    float4 position [[position]];
    float3 world;
};

vertex GroundOut vsGround(uint vid [[vertex_id]], constant FrameUniforms& f [[buffer(1)]]) {
    const float2 corners[4] = {float2(-1, -1), float2(1, -1), float2(-1, 1), float2(1, 1)};
    float3 w = float3(f.footprint.x + corners[vid].x * f.params.w, f.params.z, f.footprint.y + corners[vid].y * f.params.w);
    GroundOut o;
    o.position = fuelClip(f, w);
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

// The chunk size C of a ground vertex (TerrainVertex::size, interpolated). Chunk sizes are 12288 / 2^k =
// 3 * 2^n (24, 48, 96, 192 m), so the size is snapped in those steps. (p71: a power-of-two snap made 24 m
// chunks 32 m ones, i.e. a 1.33 m lattice reading the 1 m / 2 m tiles at the wrong texels: the decal uv
// came out in flat blocks with stepped lane lines.)
float groundBandChunk(float size) {
    return 3.0 * exp2(round(log2(size / 3.0)));
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
    o.position = fuelClip(f, w);
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

// P92: groundColourAt and groundOcclusionAt in one pass: one fetch per fine level for both (they sampled the same
// texels twice). Same values.
float4 groundColourOccAt(float2 xz, float4 gc, texture2d_array<float> fine, constant FrameUniforms& f, thread float& occ) {
    constexpr sampler lin(coord::normalized, filter::linear, mip_filter::linear, address::clamp_to_edge);
    occ = 1.0;
    for (int l = 2; l >= 0; --l) {
        const float4 t = f.groundFine[l];
        if (t.w <= 0.0) continue;
        const float2 u = (xz - t.xy) / t.z;
        const float edge = min(min(u.x, 1.0 - u.x), min(u.y, 1.0 - u.y));
        const float w = saturate(edge * 8.0);
        if (w > 0.0) {
            const float4 s = fine.sample(lin, u, uint(l));
            gc = mix(gc, s, w);
            occ = mix(occ, s.a, w);
        }
    }
    return gc;
}

// P92: the 60 m surface map around uv: the sand / snow / cracked share (the far sets' input) and the nearest cell's
// type (the fallback before a fine tile arrives). fsTerrain reads it only when a band needs either.
void groundSurfaceAt(texture2d<uint> groundSurface, float2 uv, thread float& sandy, thread uint& nearest) {
    float size = float(groundSurface.get_width());
    float2 t = uv * size - 0.5;
    float2 i0 = floor(t), a = t - i0;
    sandy = 0.0;
    nearest = 0;
    float wbest = -1.0;
    for (int q = 0; q < 4; ++q) {
        uint2 cell = uint2(clamp(i0 + float2(q & 1, q >> 1), 0.0, size - 1.0));
        float w = ((q & 1) ? a.x : 1.0 - a.x) * ((q >> 1) ? a.y : 1.0 - a.y);
        const uint layer = groundSurface.read(cell).r;
        if (w > wbest) wbest = w, nearest = layer;
        sandy += w * float((layer >= 6 && layer <= 9) || layer == 24 || layer >= 36);
    }
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
// A lattice vertex's tile texel, false outside the tile. P92: its surface types are no longer read per pixel: the
// draw set of each lattice triangle is made once per tile by kGroundDrawSet (below) and read whole; the vertex's
// decal (u, v, weight, slot (-1 none)) is read only when the triangle has one (or its band has no draw set).
bool groundVertex(float4 rect, float sp, float2 wp, thread int2& it) {
    if (rect.w <= 0.0) return false;
    it = int2(floor((wp - rect.xy) / sp - 0.5 + 0.5));
    if (any(it < 0) || any(it >= int2(256))) return false;
    return true;
}

uint typeWeight(ushort4 tw, uint k) {
    for (int c = 0; c < 4; ++c)
        if (uint(tw[c] & 0xff) == k) return uint(tw[c] >> 8);
    return 0;
}

// ---- P92: the draw set of the near bands' lattice triangles (0x7f9b50 near loop), computed per triangle when a fine
// tile arrives instead of per pixel in fsTerrain (it was ~1.8 ms of a 3420x2224 frame). The selection is the one
// groundBand ran, unchanged and in integers, so every pixel gets the same types and weights as before. Per tile
// level (0 = 1 m lattice, 24 m chunks; 1 = 2 m, 48 m chunks) and cell (x, y) two triangles, slice = level * 2 + half
// (0: (x, y) (x, y+1) (x+1, y); 1: (x+1, y) (x, y+1) (x+1, y+1), groundTriangle's vertex order):
//   x = k0 | k1 << 8 | k2 << 16 (k1 / k2 = 0xff when unused) | 1 << 24 when a vertex has a road decal, y / z / w =
//   vertex 0 / 1 / 2's weights of k0, k1, k2 (bytes 0, 1, 2; 0 for an unused k).
kernel void kGroundDrawSet(texture2d_array<ushort, access::read> types [[texture(0)]],
                           texture2d_array<uint, access::write> out [[texture(1)]], constant uint& level [[buffer(0)]],
                           texture2d_array<float, access::read> decal [[texture(2)]],
                           uint3 gid [[thread_position_in_grid]]) {
    if (gid.x >= 256 || gid.y >= 256 || gid.z >= 2) return;
    uint4 res = uint4(0);
    if (gid.x < 255 && gid.y < 255) {
        const uint2 c = gid.xy;
        uint2 t[3];
        if (gid.z == 0) t[0] = c, t[1] = c + uint2(0, 1), t[2] = c + uint2(1, 0);
        else t[0] = c + uint2(1, 0), t[1] = c + uint2(0, 1), t[2] = c + uint2(1, 1);
        ushort4 tw[3];
        for (int i = 0; i < 3; ++i) tw[i] = types.read(t[i], level);
        const uint thr = level == 0 ? 32u : 64u;  // C <= 30 (24 m chunks) : 48 m chunks
        uint m0 = 0, m1 = 0, m2 = 0, k0 = 0, k1 = 0, k2 = 0, last = 0;
        bool first = true;
        for (int it = 0; it < 12; ++it) {
            uint next = 0x100;
            for (int q = 0; q < 12; ++q) {
                const uint k = uint(tw[q / 4][q % 4] & 0xff);
                if (k != 0xff && (first || k > last) && k < next) next = k;
            }
            if (next == 0x100) break;
            first = false;
            last = next;
            const uint m = max(typeWeight(tw[0], next), max(typeWeight(tw[1], next), typeWeight(tw[2], next)));
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
            if (k1 < k0) { const uint s = k0; k0 = k1; k1 = s; }
            if (k2 < k1) { const uint s = k1; k1 = k2; k2 = s; }
            if (k1 < k0) { const uint s = k0; k0 = k1; k1 = s; }
        } else {
            k2 = 0xff;
            if (m1 > thr) {
                if (k1 < k0) { const uint s = k0; k0 = k1; k1 = s; }
            } else {
                k1 = 0xff;
            }
        }
        res.x = k0 | (k1 << 8) | (k2 << 16);
        // Bit 24: a vertex of the triangle carries a road decal (slot >= 0); without one groundBand skips the decal reads.
        bool anyDecal = false;
        for (int i = 0; i < 3; ++i) anyDecal = anyDecal || decal.read(t[i], level).w >= 0.0;
        if (anyDecal) res.x |= 1u << 24;
        for (int i = 0; i < 3; ++i)
            res[1 + i] = typeWeight(tw[i], k0) | ((k1 != 0xff ? typeWeight(tw[i], k1) : 0u) << 8) |
                         ((k2 != 0xff ? typeWeight(tw[i], k2) : 0u) << 16);
    }
    out.write(res, gid.xy, level * 2 + gid.z);
}

// The lattice triangle holding xz (lattice spacing L): FUEL splits each quad along the
// (x+1, z)-(x, z+1) diagonal; vertices in the order the decal material is picked.
void groundTriangle(float2 xz, float L, thread float2* p, thread float3& bary, thread uint& upper) {
    const float2 g = xz / L, i0 = floor(g), fr = g - i0;
    if (fr.x + fr.y <= 1.0) {
        p[0] = i0, p[1] = i0 + float2(0, 1), p[2] = i0 + float2(1, 0);
        bary = float3(1.0 - fr.x - fr.y, fr.y, fr.x);
        upper = 0;  // P92: kGroundDrawSet's slice (0 lower, 1 upper triangle)
    } else {
        p[0] = i0 + float2(1, 0), p[1] = i0 + float2(0, 1), p[2] = i0 + float2(1, 1);
        bary = float3(1.0 - fr.y, 1.0 - fr.x, fr.x + fr.y - 1.0);
        upper = 1;
    }
    for (int i = 0; i < 3; ++i) p[i] *= L;
}

// One band of ground.phl: bump normal + grey detail (n.xyz, n.w) and the decal (rgb, blend).
struct GroundBand {
    float4 n;
    float4 decal;
    bool ok;
};

GroundBand groundBand(float C, float2 xz, float2 warp, float steep, float sandy, texture2d_array<uint> drawSet,
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
    uint upper;
    groundTriangle(xz, L, p, bary, upper);
    float4 dc[3];
    int2 it[3];
    bool have = C <= 192.5;
    for (int i = 0; i < 3 && have; ++i) have = groundVertex(f.groundFine[slice], sp, p[i], it[i]);
    bool decals = have;  // P92: read the vertices' decals (the near bands: only when the triangle has one)
    if (C <= 48.5) {
        if (!have) {
            out.ok = false;
            return out;
        }
        // Draw set of the triangle (0x7f9b50 near loop): per type the largest vertex weight; the three largest (ties
        // keep the lower type) if above the threshold, sorted by type. P92: made per triangle by kGroundDrawSet when
        // the tile arrived; the cell is the lower triangle's first vertex (the upper one's minus one in x).
        const int2 cell = upper != 0u ? it[0] - int2(1, 0) : it[0];
        const uint4 ds = drawSet.read(uint2(cell), slice * 2u + upper);
        const uint k0 = ds.x & 0xffu, k1 = (ds.x >> 8) & 0xffu, k2 = (ds.x >> 16) & 0xffu;
        decals = (ds.x >> 24) != 0u;
        // Per-vertex lerp weights (0x7f8be0), interpolated like color0.w / color1.w.
        float3 b1 = 0.0, b2 = 0.0;
        for (int i = 0; i < 3; ++i) {
            const uint wk = ds[1 + i];
            const float w0 = float(wk & 0xffu) / 255.0;
            const float a = k1 != 0xff ? float((wk >> 8) & 0xffu) / 255.0 : 0.0;
            const float b = k2 != 0xff ? float((wk >> 16) & 0xffu) / 255.0 : 0.0;
            const float sum = w0 + a;
            b1[i] = a > 0.0 ? floor((sum > 0.0 ? a / sum : 0.0) * 255.0) / 255.0 : 0.0;
            b2[i] = b > 0.0 ? floor(saturate(max(b, 1.0 - sum)) * 255.0) / 255.0 : 0.0;
        }
        const float B1 = dot(b1, bary), B2 = dot(b2, bary);
        const float2 uv = warp * 0.2;  // W+0x4832c = 0.2 repeats per metre
        float4 t = detail.sample(rep, uv, k0, bias(f.taa.x));  // PK28: 0x84ca80 LOD bias
        if (k1 != 0xff) t = mix(t, detail.sample(rep, uv, k1, bias(f.taa.x)), B1);  // PK28: 0x84ca80 LOD bias
        if (k2 != 0xff) t = mix(t, detail.sample(rep, uv, k2, bias(f.taa.x)), B2);  // PK28: 0x84ca80 LOD bias
        out.n = t;
    } else {
        // Chunks of 96 m and more use one fixed set for every mid/far band (0x7f7ea0 / 0x7f8390 /
        // 0x7f8850 store bump slots 0x11, 0x14, 0x15 at 0x7f820e / 0x7f86f0 / 0x7f8af1; p32):
        // T0 = 17 STEPPENL, T1 = 20 ROCKNL (steepness w1), T2 = 21 PLAINNL (sand / snow share w2).
        const float scale = C < 383.0 ? 0.015 : 0.0015, extra = C < 383.0 ? 0.75 : 0.0;
        const float2 uvF = warp * scale;
        const float4 t0 = detail.sample(rep, uvF, 17, bias(f.taa.x)), t1 = detail.sample(rep, uvF, 20, bias(f.taa.x));  // PK28: 0x84ca80 LOD bias
        const float4 t2 = detail.sample(rep, uvF, 21, bias(f.taa.x));  // PK28: 0x84ca80 LOD bias
        out.n = mix(mix(t0, t1, steep), t2, saturate(saturate(sandy + extra) - steep));
    }
    // Decal (ground.phl !bNoDecal): material of the first vertex that has one; uv and
    // weight interpolated across the triangle. Chunks up to 192 m carry decals.
    if (decals) {  // (without a decal slot on any vertex this block draws nothing)
        for (int i = 0; i < 3; ++i) dc[i] = decalTex.read(uint2(it[i]), slice);
        int slot = -1;
        for (int i = 0; i < 3; ++i)
            if (slot < 0 && dc[i].w >= 0.0) slot = int(dc[i].w + 0.5);
        const float w = dot(float3(dc[0].z, dc[1].z, dc[2].z), bary);
        if (slot >= 0 && w > 0.0) {
            const float2 duv = dc[0].xy * bary.x + dc[1].xy * bary.y + dc[2].xy * bary.z;
            float4 td;
            switch (slot) {
                case 0: td = d0.sample(rep, duv, bias(f.taa.x + f.decalBias[0].x)); break;  // PK28: 0x84ca80 LOD bias
                case 1: td = d1.sample(rep, duv, bias(f.taa.x + f.decalBias[0].y)); break;  // PK28: 0x84ca80 LOD bias
                case 2: td = d2.sample(rep, duv, bias(f.taa.x + f.decalBias[0].z)); break;  // PK28: 0x84ca80 LOD bias
                case 3: td = d3.sample(rep, duv, bias(f.taa.x + f.decalBias[0].w)); break;  // PK28: 0x84ca80 LOD bias
                case 4: td = d4.sample(rep, duv, bias(f.taa.x + f.decalBias[1].x)); break;  // PK28: 0x84ca80 LOD bias
                default: td = d5.sample(rep, duv, bias(f.taa.x + f.decalBias[1].y)); break;  // PK28: 0x84ca80 LOD bias
            }
            out.decal = float4(td.rgb, td.a * w);
        }
    }
    return out;
}

fragment float4 fsTerrain(TerrainOut in [[stage_in]], constant FrameUniforms& f [[buffer(1)]],
                          texture2d<float> hm [[texture(0)]], texture2d<float> groundColour [[texture(1)]],
                          texture2d<uint> groundSurface [[texture(2)]], texture2d_array<float> detail [[texture(3)]],
                          texture2d_array<float> groundFine [[texture(4)]], texture2d_array<uint> groundDrawSet [[texture(5)]],
                          texture2d_array<float> groundDecal [[texture(6)]], depth2d_array<float> shadowMap [[texture(7)]],
                          texture2d<float> decal0 [[texture(8)]], texture2d<float> decal1 [[texture(9)]],
                          texture2d<float> decal2 [[texture(10)]], texture2d<float> decal3 [[texture(11)]],
                          texture2d<float> decal4 [[texture(12)]], texture2d<float> decal5 [[texture(13)]],
                          texture2d<float> carSpot [[texture(14)]], const device ulong* omniClusters [[buffer(5)]],
                          constant float4* omnis [[buffer(6)]]) {  // PK28
    float e = f.terrain.z;
    float2 xz = in.world.xz;
    (void)e;
    // FUEL's ground normal from the chunk's points (P21's ground mesh), interpolated.
    float3 N = normalize(in.normal);
    float3 c;
    float occ = 1.0;
    if (f.fog.y > 0.0) {
        constexpr sampler lin(coord::normalized, filter::linear, mip_filter::linear, address::clamp_to_edge);
        constexpr sampler rep(coord::normalized, filter::linear, mip_filter::linear, address::repeat, max_anisotropy(8));
        float2 uv = (xz + 70000.0) / 140000.0;
        const float4 map60 = groundColour.sample(lin, uv);
        const float4 gc = groundColourOccAt(xz, map60, groundFine, f, occ);  // P92: one fetch per level for both
        // Decoded map (p16): the point colour ctx+0x70 as is; the heuristic map gets the old grade.
        c = f.groundFar.w > 0.0 ? gc.rgb : groundGrade(gc.rgb);
        // Bump uv: the world position warped by two sines (0x7f9b50).
        const float2 warp = float2(xz.x + 5.0 * sin(0.02 * xz.y) + 2.0 * sin(0.2 * xz.y),
                                   xz.y + 5.0 * sin(0.02 * xz.x) + 2.0 * sin(0.2 * xz.x));
        // Steepness weight of the far sets: a = clamp(2 n.y - 1), s = 2a - 1, w1 = 1 - S2(s).
        const float sa = 2.0 * saturate(2.0 * N.y - 1.0) - 1.0;
        const float steep = 1.0 - ((2.0 - abs(sa)) * sa * 0.5 + 0.5);
        // The chunk under the point and FUEL's geomorph tween toward its parent chunk
        // (ground.vhl: (d - 2.375 C) * 2 / C, distance in xz); the port blends the parent's
        // textures with the same weight (the engine switches them per chunk).
        const float dist = length(f.eye.xz - xz);
        const float C = in.chunk > 0.0 ? groundBandChunk(in.chunk) : groundChunk(xz, f.eye.xz);  // the ground chunk drawing this pixel
        const float tw = saturate((dist - 2.375 * C) * 2.0 / C);
        // Sand / snow / cracked share of the 60 m surface map (FUN_007e4950 input of the far sets). P92: read only
        // when a far band (chunks over 48 m, or the tween's parent) uses it, or below for the no-tile fallback.
        float sandy = 0.0;
        uint nearest = 0;
        const bool surfaceRead = C > 48.5 || (tw > 0.0 && 2.0 * C > 48.5);
        if (surfaceRead) groundSurfaceAt(groundSurface, uv, sandy, nearest);
        GroundBand b = groundBand(C, xz, warp, steep, sandy, groundDrawSet, groundDecal, detail, rep, f, decal0, decal1, decal2, decal3,
                                  decal4, decal5);
        if (!b.ok) {  // no tile yet: the 60 m surface map's nearest type
            if (!surfaceRead) groundSurfaceAt(groundSurface, uv, sandy, nearest);
            b.n = detail.sample(rep, warp * 0.2, nearest, bias(f.taa.x));  // PK28: 0x84ca80 LOD bias
            b.ok = true;
        }
        if (tw > 0.0) {
            GroundBand pb = groundBand(2.0 * C, xz, warp, steep, sandy, groundDrawSet, groundDecal, detail, rep, f, decal0, decal1, decal2,
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
        // Procedural fallback: grass and dry ground by altitude, rock on steep slopes, snow up high. (P92: the noise and
        // slope only here, where they are used.)
        const float slope = 1.0 - N.y;
        const float n = valueNoise(xz * 0.004) * 0.6 + valueNoise(xz * 0.03) * 0.3 + valueNoise(xz * 0.25) * 0.1;
        float3 grass = float3(0.035, 0.060, 0.018), dry = float3(0.11, 0.085, 0.045);
        float3 rock = float3(0.10, 0.092, 0.085), snow = float3(0.80, 0.82, 0.86);
        c = mix(grass, dry, saturate(n * 0.9 + (in.world.y - 1500.0) / 1800.0));
        c = mix(c, rock, smoothstep(0.16, 0.32, slope + (n - 0.5) * 0.08));
        c = mix(c, snow, smoothstep(3100.0, 3500.0, in.world.y + n * 300.0) * (1.0 - smoothstep(0.35, 0.6, slope)));
    }
    // P74: ground.phl bOmni: occlusion *= 1 - f f f (the car occlusion omni of the vehicle over this ground).
    occ *= carOcclusion(f, in.world, carSpot);
    // ground.phl lighting(): sun / sky in the bump frame, skylight = lerp(DSKY_AMBIENT, DSKY_COLOR, occlusion),
    // specular power 15 with the per-vertex specular (port: 0, not carried by the tiles), the deferred omnis (P77,
    // GetDeferredLights), then ApplyScattering with the vertex's extinction / inscatter.
    const float shadow = fuelShadow(shadowMap, in.world, f);
    const float3 V = normalize(f.eye.xyz - in.world);
    const float3 rgb = fuelLighting(N, f.lightDir.xyz, f.skyDir.xyz, V, f.sunColour.rgb, mix(f.skyAmbient.rgb, f.skyColour.rgb, occ), c,
                                    float3(0.0), 15.0, shadow, fuelOmnisMasked(f, omnis, in.world, N, omniMask(f, omniClusters, in.position.xy, in.world)),
                                    occ);
    if (f.lightInfo.z > 0.5) return float4(fuelOmnis(f, omnis, in.world, N) * 0.05, 1.0);  // FUEL_OMNI_DEBUG: the omni sum alone
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
    return su.lightViewProj * float4(w - su.eye.xyz, 1.0);  // P72: camera-relative
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
    o.position = fuelClip(f, w);
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
                        depth2d_array<float> shadowMap [[texture(7)]], texturecube<float> globalCube [[texture(8)]]) {
    constexpr sampler cubeSmp(filter::linear, address::clamp_to_edge);
    constexpr sampler rep(coord::normalized, filter::linear, mip_filter::linear, address::repeat, max_anisotropy(4));
    const float4 S0 = W[4], S1 = W[5], S2 = W[6], S3 = W[7];
    const float3 V = normalize(f.eye.xyz - in.world);
    // Depth of water along the view ray (the engine reads the depth buffer), x Settings1.z.
    const float waterDepth = max(in.depth, 0.0) / max(abs(V.y), 0.1) * S1.z;
    float3 n = (normalTex.sample(rep, in.uv.xy, bias(f.taa.x)).xyw * 2.0 - 1.0) + (normalTex.sample(rep, in.uv.zw, bias(f.taa.x)).xyw * 2.0 - 1.0);  // PK28: 0x84ca80 LOD bias
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
    // sGlobalCube (the port's encodeCube: sky + ground from the camera, world axes).
    const float3 env = globalCube.sample(cubeSmp, r).rgb * (0.25 + 0.75 * shadow);
    const float F = saturate(pow(1.0 - saturate(dot(V, n)), 5.0) * (1.0 - saturate(S1.w)) + S1.w) * S2.y;
    k *= 1.0 - F;
    K = K * (1.0 - F) + env * F;
    const float spec = pow(max(river ? 0.001 : 1e-4, dot(reflect(-V, ns), L)), S3.y * 2.0) * S3.z * 0.3;
    K += light * spec;
    const float fo = saturate(1.1 - waterDepth * 0.002 * S2.x);
    const float ff = river ? 0.0 : fo * fo * foamTex.sample(rep, in.uv.xy - n.xz * 0.5, bias(f.taa.x)).a;  // PK28: 0x84ca80 LOD bias
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

// P74: the sun / moon sprite quad (SkyDome_BuildSunQuad 0x7e20f0, GenWorld kind 0x19): built once at the zenith,
// centre 0.9 R up, half size 0.05 R (R = 0.9 far), corners pulled in x 0.95; uv (0,0) at (+h, -h), (1,0) at (+h, +h),
// (1,1) at (-h, +h), (0,1) at (-h, -h) in (x, z), i.e. u = z / 2h + 1/2, v = -x / 2h + 1/2. SkyDome_Update 0x7e2690
// draws it with Quat_FromArc(up -> GenWorld_GetLightDirection) about the camera. Returns uv, or -1 off the quad.
float2 sunSpriteUv(float3 dir, float3 L) {
    // d' = R^T d, R the shortest arc up -> L (Rodrigues about k = up x L / |up x L| by -theta, cos theta = L.y).
    const float c = clamp(L.y, -1.0, 1.0), s = length(L.xz);
    float3 d = dir;
    if (s > 1e-6) {
        const float3 k = float3(L.z, 0.0, -L.x) / s;
        d = dir * c - cross(k, dir) * s + k * dot(k, dir) * (1.0 - c);
    } else if (c < 0.0) {
        d = float3(dir.x, -dir.y, -dir.z);  // L straight down: half a turn about x
    }
    if (d.y <= 1e-4) return float2(-1.0);
    const float2 q = d.xz / d.y * (0.9 / 0.05);  // (x, z) / h on the quad's plane
    if (any(abs(q) > 1.0)) return float2(-1.0);
    return float2(q.y * 0.5 + 0.5, -q.x * 0.5 + 0.5);
}

fragment float4 fsSky(SkyOut in [[stage_in]], constant FrameUniforms& f [[buffer(1)]], texture2d<float> sunTex [[texture(0)]]) {
    float4 w = f.invViewProj * float4(in.ndc, 1.0, 1.0);
    float3 dir = normalize(w.xyz / w.w - f.eye.xyz);
    // ph_sky: the inscatter of the sky box (no texture).
    float3 c = fuelSky(f, dir);
    // ph_skysun: the sun (or moon) sprite: DLIGHT_COLOR x tex.a x extinction x 1000 (moon: x tex.rgb x 100),
    // picked by saturate(blueShift.w - 0.5), alpha tex.a, blended over the sky (material 0x36 flags 0x20). P74: the
    // textured quad of 0x7e20f0 (GenWorld material 0x36's texture, sunRaw.w = 1 when loaded); the extinction is
    // taken along the sun direction at the dome radius (FUEL: per quad vertex).
    const float3 sd = normalize(f.sunRaw.xyz);
    constexpr sampler sunSmp(filter::linear, mip_filter::linear, address::clamp_to_edge);
    const float2 uv = sunSpriteUv(dir, sd);
    const float4 t = sunTex.sample(sunSmp, uv);  // outside the texture's disc alpha is 0
    // The cube pass (fog.x bit 2) draws GenWorld kinds 3..8 and 24 only: no sun sprite (kind 25).
    const float a = ((uint(f.fog.x) & 4u) || uv.x < 0.0 || f.lightInfo.w == 0.0) ? 0.0 : t.a;
    if (a > 0.0) {
        const Scatter sc = fuelScattering(f, f.eye.xyz + sd * kSkyDomeRadius);
        const float3 sunDisc = f.dlight.rgb * a * sc.ext * 1000.0, moonDisc = f.dlight.rgb * a * t.rgb * sc.ext * 100.0;
        c = mix(c, c + mix(moonDisc, sunDisc, saturate(f.dlight.w - 0.5)), a);  // SRCALPHA over the dome's inscatter
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
    MarkerOut o;
    if (m.w >= 10.0) {  // debug trunk (FUEL_P52_TRUNKS): w = 10 + r (tree, 6 m tall) or 20 + r (bush, 11 m), base at xyz
        float kind = floor(m.w / 10.0), r = m.w - kind * 10.0;
        float3 w = m.xyz + float3(cos(a) * r, top ? (kind == 1.0 ? 6.0 : 11.0) : 0.0, sin(a) * r);
        o.position = fuelClip(f, w);
        o.color = float3(6.0, 0.3, 5.0);  // HDR: stays visible after the exposure
        o.glow = 1.0;
        return o;
    }
    float radius = m.w == 2.0 ? 6.0 : 4.0;
    float3 w = m.xyz + float3(cos(a) * radius, top ? 150.0 : -3.0, sin(a) * radius);
    o.position = fuelClip(f, w);
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
    o.position = fuelClip(f, pos);
    // Corner -> atlas uv (0x5fac70: -x corners take u0, +x corners u1; the top takes v0).
    o.uv = float2(corner.x < 0.0 ? p.uv.x : p.uv.z, corner.y > 0.0 ? p.uv.y : p.uv.w);
    const float3 fwd = cross(f.camUp.xyz, f.camRight.xyz);
    o.viewZ = dot(pos - f.eye.xyz, fwd);
    const Scatter sc = fuelScatteringDir(f, pos, f.lightDir.xyz)  /* DUSK: sfx.vhl:207 / 393 */;
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
    float4 c = atlas.sample(smp, in.uv, bias(f.taa.x));  // PK28: 0x84ca80 LOD bias
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
    float4 flash[4];  // C_USER1..4: the thunder's flashes (cloud +0xa4 position, +0xd4 strength; p48)
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
                       float4 user6, thread float& alpha, thread float& shade, constant float4* flash, thread float& lightning) {
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
    const float3 v = position + uvscale.x * right * scaleFactor + uvscale.y * up * scaleFactor;
    // GetLightningInfluence over C_USER1..4 with the range C_USER6.x (2000): w x (1 - saturate(d / range)).
    lightning = 0.0;
    for (int k = 0; k < 4; ++k) lightning += flash[k].w * (1.0 - saturate(length(v - flash[k].xyz) / user6.x));
    alpha = saturate(1.0 - (len - (sightLine - zone)) / zone);
    return v;
}

vertex CloudOut vsCloud(uint vid [[vertex_id]], uint iid [[instance_id]], const device CloudSprite* cs [[buffer(0)]],
                        constant FrameUniforms& f [[buffer(1)]], constant CloudParams& u [[buffer(2)]]) {
    const CloudSprite c = cs[iid];
    const float2 corner = float2(vid & 1, vid >> 1) * 2.0 - 1.0;
    float alpha, shade, lightning;
    const float3 v = fuelCloudCorner(f, c.pos.xyz, float2(c.pos.w, c.size.x), corner, u.user0, u.user5, u.user6, alpha, shade, u.flash,
                                     lightning);
    CloudOut o;
    o.position = fuelClip(f, v);
    o.uv = float2(corner.x < 0.0 ? c.uv.x : c.uv.y, corner.y > 0.0 ? c.uv.z : c.uv.w);
    // o.diffuse = rgba x CloudsAlpha, a = 1, rgb x (1 - Cloudness s), a x the distance fade.
    const float3 diffuse = u.rgba.rgb * u.user0.y * shade;
    const Scatter sc = fuelScatteringDir(f, v, f.lightDir.xyz)  /* DUSK: sfx.vhl:290 */;
    const float3 vcol = (diffuse * sc.ext + lightning) * 2.0;  // vcolor.xyz += intensityModulated; x C_TWO
    o.sun = float4((f.skyColour.rgb + f.sunColour.rgb * saturate(f.lightDir.y)) * vcol, alpha);
    o.insc = sc.insc;
    o.viewZ = dot(v - f.eye.xyz, cross(f.camUp.xyz, f.camRight.xyz));
    return o;
}

fragment float4 fsCloud(CloudOut in [[stage_in]], depth2d<float> depth [[texture(0)]], texture2d<float> tex [[texture(1)]],
                        constant FrameUniforms& f [[buffer(1)]]) {
    constexpr sampler smp(coord::normalized, filter::linear, mip_filter::linear, address::clamp_to_edge);
    float4 c = tex.sample(smp, in.uv, bias(f.taa.x));  // PK28: 0x84ca80 LOD bias
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
    o.position = fuelClip(f, world);
    o.viewZ = dot(world - f.eye.xyz, fwd);
    return o;
}

// ph_sfxBase: rgb = vcol x tex x 2, a = vcol.a x tex.a, clip at 0.01; hidden behind the scene (resolved depth).
fragment float4 fsTank(TankOut in [[stage_in]], depth2d<float> depth [[texture(0)]], texture2d<float> atlas [[texture(1)]],
                       constant FrameUniforms& f [[buffer(1)]]) {
    constexpr sampler smp(coord::normalized, filter::linear, mip_filter::linear, address::clamp_to_edge);
    const float4 tex = atlas.sample(smp, in.uv, bias(f.taa.x));  // PK28: 0x84ca80 LOD bias
    float4 c = float4(in.diffuse.rgb * tex.rgb * 2.0, in.diffuse.a * tex.a);
    if (c.a < 0.01) discard_fragment();
    const float d = depth.read(uint2(in.position.xy));
    const float scene = f.fog.z * f.fog.w / (d + f.fog.w);
    if (in.viewZ > scene) discard_fragment();
    return float4(c.rgb, saturate(c.a));
}

// ---- FUEL's tyre tracks (ribbon decals, SubCode 0x1b): decal.vhl vh_decal + decal.phl ph_decal (p48). The
// strips are 0x5df2c0's: two vertices per point, lifted 0.0625 m along the vehicle's up axis; the colour is the
// ground colour lit through the FX_PNEU normal map, and on tarmac the mark turns black with the normal map's z
// as its alpha.

struct DecalVert {
    packed_float3 pos;
    float u, v;
    packed_float3 tangent;
    packed_float4 rgba;
    packed_float3 normal;
    float lift, deep, tarmac;
};

struct DecalOut {
    float4 position [[position]];
    float2 uv;
    float3 T, B, N;
    float3 world;
    float3 eyevec;
    float4 diffuse;
    float tarmac;
    float3 ext, insc;
    float viewZ;
};

vertex DecalOut vsDecal(uint vid [[vertex_id]], const device DecalVert* vs [[buffer(0)]], constant FrameUniforms& f [[buffer(1)]]) {
    const DecalVert v = vs[vid];
    float3 p = float3(v.pos);
    DecalOut o;
    o.eyevec = f.eye.xyz - p;
    p += float3(v.normal) * v.lift;
    o.position = fuelClip(f, p);
    const float3 T0 = float3(v.tangent);
    o.T = length(T0) > 1e-6 ? normalize(T0) : float3(1, 0, 0);
    o.N = normalize(float3(v.normal));
    o.B = normalize(cross(o.N, o.T));
    o.world = p;
    o.diffuse = float4(v.rgba);
    o.uv = float2(v.u, v.v);
    o.tarmac = v.tarmac;
    const Scatter sc = fuelScattering(f, p);
    o.ext = sc.ext, o.insc = sc.insc;
    o.viewZ = dot(p - f.eye.xyz, cross(f.camUp.xyz, f.camRight.xyz));
    return o;
}

fragment float4 fsDecal(DecalOut in [[stage_in]], depth2d<float> depth [[texture(0)]], texture2d<float> diffuseTex [[texture(1)]],
                        texture2d<float> normalTex [[texture(2)]], depth2d_array<float> shadowMap [[texture(3)]],
                        constant FrameUniforms& f [[buffer(1)]], constant float4& opts [[buffer(2)]]) {
    constexpr sampler smp(coord::normalized, filter::linear, mip_filter::linear, address::repeat);
    const float d = depth.read(uint2(in.position.xy));
    const float scene = f.fog.z * f.fog.w / (d + f.fog.w);
    if (in.viewZ > scene + opts.y) discard_fragment();  // the ground in front of the mark
    if (opts.z > 0.5) return float4(4.0, 0.0, 4.0, 1.0);  // FUEL_SFX_DEBUG=6: solid magenta strips
    const float4 Diffuse = diffuseTex.sample(smp, in.uv, bias(f.taa.x));  // PK28: 0x84ca80 LOD bias
    const float4 ns = normalTex.sample(smp, in.uv, bias(f.taa.x));  // PK28: 0x84ca80 LOD bias
    float3 t;
    if (opts.x > 0.5) {  // bCompression: wyz, z rebuilt
        t = ns.wyz * 2.0 - 1.0;
        t.z = sqrt(saturate(1.0 - (t.x * t.x + t.y * t.y)));
    } else {
        t = ns.xyz * 2.0 - 1.0;
    }
    const float3 n = normalize(t.x * in.T + t.y * in.B + t.z * in.N);
    const float3 eye = normalize(in.eyevec);
    const float shadow = fuelShadow(shadowMap, in.world, f);
    float4 c;
    c.rgb = fuelLighting(n, f.lightDir.xyz, f.skyDir.xyz, eye, f.sunColour.rgb, f.skyColour.rgb, in.diffuse.rgb, in.diffuse.rgb * 0.3,
                         0.9, shadow, float3(0), 1.0);
    c.a = Diffuse.a * in.diffuse.a;
    const float4 tarmacColor = float4(0, 0, 0, ns.z * in.diffuse.a);
    c = mix(c, tarmacColor, in.tarmac);
    c.rgb = c.rgb * in.ext + in.insc;
    return float4(max(c.rgb, 0.0), saturate(c.a));
}

// ---- P77: blob shadows (FakeShadows_G::vf19 0x509630): one quad per record, corners C -+ b +- a (uv (0,0) (1,0)
// (0,1) (1,1)) moved towards the camera by 0x5095b0, vertex colour (0.5, 0.5, 0.5, alpha), FAKESHADOW texture
// (black, alpha disc), SRCALPHA / INVSRCALPHA, z test without z write.

struct BlobQuad {
    float4 centre;  // xyz; w alpha (record +0x2c x the renderer's light / weather factor)
    float4 a;       // xyz forward half axis (+0x00)
    float4 b;       // xyz side half axis (+0x0c)
};

struct BlobOut {
    float4 position [[position]];
    float2 uv;
    float alpha;
    float viewZ;
};

// 0x5095b0: P moved towards the eye E by (1 - |d.y|)^2 0.15 + 0.05 of d = normalize(E - P), d.y on the view's up axis.
float3 blobNudge(float3 P, float3 E, float3 up) {
    const float3 d = normalize(E - P);
    const float k = (1.0 - abs(dot(d, up))) * (1.0 - abs(dot(d, up))) * 0.15 + 0.05;
    return P + d * k;
}

vertex BlobOut vsBlob(uint vid [[vertex_id]], uint iid [[instance_id]], const device BlobQuad* qs [[buffer(0)]],
                      constant FrameUniforms& f [[buffer(1)]]) {
    const BlobQuad q = qs[iid];
    const float su = (vid & 1) ? 1.0 : -1.0, sv = (vid & 2) ? -1.0 : 1.0;
    float3 p = q.centre.xyz + su * q.b.xyz + sv * q.a.xyz;
    p = blobNudge(p, f.eye.xyz, f.camUp.xyz);
    BlobOut o;
    o.position = fuelClip(f, p);
    o.uv = float2(vid & 1, vid >> 1);
    o.alpha = q.centre.w;
    o.viewZ = dot(p - f.eye.xyz, cross(f.camUp.xyz, f.camRight.xyz));
    return o;
}

fragment float4 fsBlob(BlobOut in [[stage_in]], depth2d<float> depth [[texture(0)]], texture2d<float> tex [[texture(1)]],
                       constant FrameUniforms& f [[buffer(1)]], constant float4& opts [[buffer(2)]]) {
    constexpr sampler smp(coord::normalized, filter::linear, mip_filter::linear, address::clamp_to_edge);  // flags 0x300: clamp u / v
    const float d = depth.read(uint2(in.position.xy));
    // ZENABLE, no z write. opts.y: like the tyre marks, the records sit on the physics ground and the drawn terrain may
    // lie a little above it (FUEL draws one ground), so a blob still shows that far behind the scene depth.
    const bool hidden = in.viewZ > f.fog.z * f.fog.w / (d + f.fog.w) + opts.y;
    if (opts.x > 0.5) return hidden ? float4(0, 0, 4, 1) : float4(4, 0, 4, 1);  // FUEL_BLOB_DEBUG: magenta drawn, blue hidden
    if (hidden) discard_fragment();
    const float4 t = tex.sample(smp, in.uv, bias(f.taa.x));  // PK28: 0x84ca80 LOD bias
    const float a = t.a * in.alpha;
    if (a <= 0.0) discard_fragment();  // ALPHAFUNC GREATER, ref 0
    return float4(t.rgb * 0.5, saturate(a));
}

// ---- Splashes (rain.vhl vh_splash), lightning bolts (sfx.vhl vh_lightning + ph_lightning) and the thunder's
// screen flash (the HUD rectangle of 0x5e3632), one instance per quad (p48).

struct FxQuad {
    float4 pos;   // xyz; w: splash phase (+0x50) / bolt fog (weather +0xb4)
    float4 uv;    // u at -x, v at the top, u at +x, v at the bottom
    float4 size;  // xy: half extents (splash: ParticleScale xy)
    float4 rgba;
};

struct FxParams {
    float4 user0;  // splash: x FX clock, y AnimTime (PP5039), z TextureSteps 16, w uvSingleStep.x
    float4 mode;   // x 0 splash, 1 lightning, 2 screen flash
};

struct FxOut {
    float4 position [[position]];
    float2 uv;
    float4 diffuse;
    float fog;
    float viewZ;
};

vertex FxOut vsFx(uint vid [[vertex_id]], uint iid [[instance_id]], const device FxQuad* qs [[buffer(0)]],
                  constant FrameUniforms& f [[buffer(1)]], constant FxParams& u [[buffer(2)]]) {
    const FxQuad q = qs[iid];
    const float2 corner = float2(vid & 1, vid >> 1) * 2.0 - 1.0;
    FxOut o;
    o.fog = q.pos.w;
    o.diffuse = q.rgba;
    if (u.mode.x > 1.5) {  // full screen
        o.position = float4(corner, 0.0, 1.0);
        o.uv = float2(0);
        o.viewZ = 0;
        return o;
    }
    float3 world = q.pos.xyz;
    float2 uv = float2(corner.x < 0.0 ? q.uv.x : q.uv.z, corner.y > 0.0 ? q.uv.y : q.uv.w);
    if (u.mode.x < 0.5) {  // vh_splash: world y offset, view-space x offset; 16 animation frames along u
        const float anim = floor(fmod((q.pos.w + u.user0.x) / u.user0.y, 1.0) * u.user0.z);
        uv.x += anim * u.user0.w;
        world.y += q.size.y * corner.y;
        world += f.camRight.xyz * (q.size.x * corner.x);
    } else {  // vh_lightning: CAM_WSIDE / CAM_WUP
        world += corner.x * q.size.x * f.camRight.xyz + corner.y * q.size.y * f.camUp.xyz;
    }
    o.uv = uv;
    o.position = fuelClip(f, world);
    o.viewZ = dot(world - f.eye.xyz, cross(f.camUp.xyz, f.camRight.xyz));
    return o;
}

fragment float4 fsFx(FxOut in [[stage_in]], depth2d<float> depth [[texture(0)]], texture2d<float> tex [[texture(1)]],
                     constant FrameUniforms& f [[buffer(1)]], constant FxParams& u [[buffer(2)]]) {
    constexpr sampler smp(coord::normalized, filter::linear, mip_filter::linear, address::repeat);
    if (u.mode.x > 1.5) return float4(in.diffuse.rgb, saturate(in.diffuse.a));
    const float d = depth.read(uint2(in.position.xy));
    if (in.viewZ > f.fog.z * f.fog.w / (d + f.fog.w)) discard_fragment();
    const float4 t = tex.sample(smp, in.uv, bias(f.taa.x));  // PK28: 0x84ca80 LOD bias
    float4 c;
    if (u.mode.x < 0.5) {  // ph_sfxBase
        c = float4(in.diffuse.rgb * t.rgb * 2.0, in.diffuse.a * t.a);
        if (u.mode.y > 0.5) return float4(4.0, 0.0, 4.0, 1.0);  // FUEL_SFX_DEBUG=6: solid magenta splashes
    } else {  // ph_lightning: alpha map without fog (a), with fog (r)
        c = float4(in.diffuse.rgb * 2.0, in.diffuse.a * mix(t.a, t.r, in.fog));
    }
    if (c.a < 0.01) discard_fragment();  // sfxClip
    return float4(c.rgb, saturate(c.a));
}

// ---- Engine particles (Particles_Z, particle.vhl vh_particle, p26-work/particles.md 1.3): one quad per particle,
// corners (+-0.5 size) rotated by the particle angle in the quad's plane; no lighting, no fog.

struct PQuad {
    float4 pos;    // render position, w angle
    float4 size;   // x, y size; z orientation (0 camera, 1 vertical: flags 4, 2 ground: flags 2)
    float4 rgba;   // colour x (MTL_DIFFUSE + MTL_EMISSIVE), alpha x opacity x the distance fade
};

struct POut {
    float4 position [[position]];
    float2 uv;
    float4 colour;
    float viewZ;
};

vertex POut vsParticle(uint vid [[vertex_id]], uint iid [[instance_id]], const device PQuad* qs [[buffer(0)]],
                       constant FrameUniforms& f [[buffer(1)]]) {
    const PQuad q = qs[iid];
    // Corner table .data 0xa26ab8 in triangle-strip order: (-0.5, 0.5 | 0, 0), (0.5, 0.5 | 1, 0), (-0.5, -0.5 | 0, 1), (0.5, -0.5 | 1, 1).
    const float2 c = float2((vid & 1) ? 0.5 : -0.5, (vid >> 1) ? -0.5 : 0.5);
    const float2 uv = float2((vid & 1) ? 1.0 : 0.0, (vid >> 1) ? 1.0 : 0.0);
    const float2 v = c * q.size.xy;
    const float s = sin(q.pos.w), co = cos(q.pos.w);
    const float2 r = float2(v.x * co + v.y * s, v.y * co - v.x * s);
    float3 X = f.camRight.xyz, Y = f.camUp.xyz;
    if (q.size.z > 1.5) X = float3(-1, 0, 0), Y = float3(0, 0, 1);
    else if (q.size.z > 0.5) Y = float3(0, 1, 0);
    const float3 world = q.pos.xyz + r.x * X + r.y * Y;
    POut o;
    o.position = fuelClip(f, world);
    o.uv = uv;
    o.colour = q.rgba;
    o.viewZ = dot(world - f.eye.xyz, cross(f.camUp.xyz, f.camRight.xyz));
    return o;
}

fragment float4 fsParticle(POut in [[stage_in]], depth2d<float> depth [[texture(0)]], texture2d<float> tex [[texture(1)]],
                           constant FrameUniforms& f [[buffer(1)]], constant float4& tint [[buffer(2)]]) {
    constexpr sampler smp(coord::normalized, filter::linear, mip_filter::linear, address::clamp_to_edge);
    const float d = depth.read(uint2(in.position.xy));
    if (in.viewZ > f.fog.z * f.fog.w / (d + f.fog.w)) discard_fragment();
    const float4 c = tex.sample(smp, in.uv, bias(f.taa.x)) * in.colour * tint;  // x (MTL_DIFFUSE + MTL_EMISSIVE), a x MTL_OPACITY
    if (c.a < 0.004) discard_fragment();
    return float4(max(c.rgb, 0.0), saturate(c.a));
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
    float4 fade;       // P67: the screen fader's quad (rgb, alpha), over the final image
    float4 view;       // P93: xy the rendered part of the scene targets (uv scale), zw the largest uv read from them
    float4 grain;      // P102 [DEVIATION]: film grain: x amplitude (0 off), y frame seed, z grain cell (output pixels);
                       // w soften (0 off .. 1: the scene through a 3 x 3 [1 2 1] tent before the tone map)
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
// P93: the scene targets' rendered part only (view: 1, 1, 2, 2 at native, where the uv are unchanged).
fragment float4 fsShrink(FullOut in [[stage_in]], texture2d<float> src [[texture(0)]], constant PostUniforms& u [[buffer(0)]]) {
    return src.sample(postLin, min(in.uv * u.view.xy, u.view.zw));
}

// 2. ph_lumlogavg: mean of ln(max(lum, 0.001)) over four point taps of the full scene at +-half a scene pixel.
fragment float4 fsLumLog(FullOut in [[stage_in]], texture2d<float> src [[texture(0)]], constant PostUniforms& u [[buffer(0)]]) {
    float s = 0.0;
    for (int k = 0; k < 4; ++k) {
        const float2 o = float2((k & 1) ? 0.5 : -0.5, (k & 2) ? 0.5 : -0.5) * u.texel.zw;
        s += 0.25 * log(max(dot(src.sample(postPoint, min(in.uv * u.view.xy + o, u.view.zw)).rgb, float3(0.3, 0.59, 0.11)), 0.001));
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

// ---- P95: 1 and 2 in one pass (two targets, the same expressions), and the 2 x 2 reductions (3) from 128 x 128 down
// to 1 x 1 in one compute pass: every level as fsLumAvg (s = 0, then s += 0.25 x the four texels in the order (0, 0),
// (1, 0), (0, 1), (1, 1); fsLumAvg's point taps land on those texel centres), so the same values, without the seven
// small render passes.
struct ShrinkLumOut {
    float4 shrink [[color(0)]];
    float4 lum [[color(1)]];
};
fragment ShrinkLumOut fsShrinkLumLog(FullOut in [[stage_in]], texture2d<float> src [[texture(0)]], constant PostUniforms& u [[buffer(0)]]) {
    ShrinkLumOut o;
    o.shrink = src.sample(postLin, min(in.uv * u.view.xy, u.view.zw));
    float s = 0.0;
    for (int k = 0; k < 4; ++k) {
        const float2 o2 = float2((k & 1) ? 0.5 : -0.5, (k & 2) ? 0.5 : -0.5) * u.texel.zw;
        s += 0.25 * log(max(dot(src.sample(postPoint, min(in.uv * u.view.xy + o2, u.view.zw)).rgb, float3(0.3, 0.59, 0.11)), 0.001));
    }
    o.lum = float4(s, 0, 0, 1);
    return o;
}
kernel void kLumReduce(texture2d<float, access::read> src [[texture(0)]], texture2d<float, access::write> dst [[texture(1)]],
                       uint t [[thread_index_in_threadgroup]], uint nt [[threads_per_threadgroup]]) {
    threadgroup float a[64 * 64], b[32 * 32];
    for (uint i = t; i < 64u * 64u; i += nt) {  // 128 -> 64 from the texture
        const uint2 p = uint2(i % 64u, i / 64u) * 2u;
        float s = 0.0;
        for (int k = 0; k < 4; ++k) s += 0.25 * src.read(p + uint2(uint(k) & 1u, uint(k) >> 1u)).r;
        a[i] = s;
    }
    threadgroup_barrier(mem_flags::mem_threadgroup);
    threadgroup float* from = a;
    threadgroup float* to = b;
    for (uint n = 32u; n >= 1u; n /= 2u) {  // 64 -> 32 -> ... -> 1, ping-pong
        for (uint i = t; i < n * n; i += nt) {
            const uint2 p = uint2(i % n, i / n) * 2u;
            float s = 0.0;
            for (int k = 0; k < 4; ++k) s += 0.25 * from[(p.y + (uint(k) >> 1u)) * (2u * n) + p.x + (uint(k) & 1u)];
            to[i] = s;
        }
        threadgroup_barrier(mem_flags::mem_threadgroup);
        threadgroup float* x = from;
        from = to, to = x;
    }
    if (t == 0) dst.write(float4(from[0], 0, 0, 1), uint2(0, 0));
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

// ---- P102 [DEVIATION, the user's request]: film grain on the display value, after the tone map and before the HUD.
// FUEL has none: the only noise in its post chain, a screen-space uv jitter from sampler s5 in ph_tonemapping
// (ShaderComponent/shaders/fx.phl lines 139-142), is commented out in the source, ph_tonemapping has a single compiled
// variant (bNoPartialPrecision, notes/p5-shader-variants.tsv) and no shader calls fx.phl's vignette() (line 250).
// Monochrome value noise: a tiled 256 x 256 R8 texture of white noise (grainTex_, bytes from a fixed hash, made once)
// read with one bilinear tap per pixel at one texel per grain cell (u.grain.z output pixels: ~1080p grain at any size),
// shifted every frame by a whole-texel R2-sequence offset of the frame number u.grain.y (0..4095); noise in [-1, 1]; amplitude
// u.grain.x times the luminance weight w(L) = 2.598 sqrt(L) (1 - L) (peak 1 at L = 1/3; 0 at black and white, so
// fades, black bars and clipped highlights stay clean). (P102's first version hashed four cell corners per pixel:
// +0.68 ms in the radial-blur pass at 3420x2224; one tap is cheaper.) The CPU mirror is in rendercheck.mm.
constexpr sampler p102GrainSmp(coord::normalized, filter::linear, address::repeat);
float p102GrainNoise(texture2d<float> noise, float2 pos, float cell, float frame) {
    // Whole-texel offsets: the sub-texel phase (so the noise's spread) depends on the pixel only, not on the frame.
    const float2 uv = pos / (max(cell, 1.0) * 256.0) + floor(fract(frame * float2(0.7548777, 0.5698403)) * 256.0) / 256.0;
    return noise.sample(p102GrainSmp, uv).r * 2.0 - 1.0;
}
float p102GrainWeight(float L) { return 2.598076 * sqrt(saturate(L)) * (1.0 - saturate(L)); }
float3 p102Grain(float3 v, float2 pos, float4 g, texture2d<float> noise) {
    if (g.x <= 0.0) return v;
    const float L = dot(saturate(v), float3(0.2126, 0.7152, 0.0722));
    return saturate(v + g.x * p102GrainWeight(L) * p102GrainNoise(noise, pos, g.z, g.y));
}
// ---- end P102

// The display value v goes out through the inverse sRGB curve when the target encodes sRGB, so the
// stored byte is v itself (FUEL writes the tone-mapped value straight into an 8-bit buffer).
float3 displayOut(float3 v, constant PostUniforms& u, float2 pos, texture2d<float> grain) {
    v = p102Grain(saturate(v), pos, u.grain, grain);  // P102: grain (u.grain.x 0 on intermediate targets)
    v = mix(v, u.fade.rgb, u.fade.a);  // P67: BaseInGameDatas_G's fade quad (0x438be0), 0 when off
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
                          texture2d<float> grain [[texture(5)]], constant PostUniforms& u [[buffer(0)]]) {
    float3 c = scene.read(uint2(in.position.xy)).rgb;
    // ---- P102 [DEVIATION, the user's request]: soften (settings.txt soften). FUEL has no such pass (its only soften is
    // the LDR fallback's 1-pixel "Depth of Field" blend 0x8778c0, not run with HDR); the reference video's softness is
    // its own scaling and compression. Four bilinear taps half a pixel off diagonally = a 3 x 3 [1 2 1] tent of the HDR
    // scene, mixed in by grain.w, before the tone map (so FUEL's bloom, curve and blue shift act on it as on the scene).
    if (u.grain.w > 0.0) {
        const float2 px = 1.0 / float2(scene.get_width(), scene.get_height()), p = in.position.xy * px;
        float3 t = 0.0;
        for (int k = 0; k < 4; ++k)
            t += scene.sample(postLin, min(p + float2((k & 1) ? 0.5 : -0.5, (k & 2) ? 0.5 : -0.5) * px, u.view.zw)).rgb;
        c = mix(c, 0.25 * t, u.grain.w);
    }
    // ---- end P102
    const float S = expo.read(uint2(0, 0)).r;
    float3 b = 0.0;
    if (u.flags.y > 0.0) {
        const float2 buv = in.uv + 1.0 / 128.0;
        b = bloom.sample(postLin, buv).rgb + u.bloom.w * (starA.sample(postLin, buv).rgb + starB.sample(postLin, buv).rgb);
    }
    return float4(displayOut(toneMapFuel(c, S, b, u), u, in.position.xy, grain), 1.0);
}

// 9. ph_motionblur: 8 taps towards the screen centre, stronger at the edges and the bottom.
fragment float4 fsRadial(FullOut in [[stage_in]], texture2d<float> src [[texture(0)]], texture2d<float> grain [[texture(1)]],
                         constant PostUniforms& u [[buffer(0)]]) {
    const float2 H = in.uv * 2.0 - 1.0;
    const float l = length(H);
    const float2 dir = l > 1e-6 ? H / l : float2(0.0);
    float a = saturate(l * u.mb0.x + u.mb0.y);
    a = a * a * a * a * u.mb0.w * in.uv.y;
    const float2 step = -dir * a;
    float3 c = 0.0;
    for (int i = 0; i < 8; ++i) c += u.mbw[i].x * src.sample(postLin, min((in.uv + float(i) * step) * u.view.xy, u.view.zw)).rgb;
    return float4(displayOut(c, u, in.position.xy, grain), 1.0);
}

// ---- P93: dynamic resolution's last passes. The upscaled frame (display values) into the output: through the inverse
// sRGB curve when the output encodes sRGB (flags.z), as displayOut (the fade is already in).
float3 presentOut(float3 v, constant PostUniforms& u, float2 pos, texture2d<float> grain) {
    v = p102Grain(saturate(v), pos, u.grain, grain);  // P102: a scaled frame's grain, at the output's pixels
    if (u.flags.z <= 0.0) return v;
    return select(pow((v + 0.055) / 1.055, 2.4), v / 12.92, v <= 0.04045);
}
fragment float4 fsPresent(FullOut in [[stage_in]], texture2d<float> src [[texture(0)]], texture2d<float> grain [[texture(1)]],
                          constant PostUniforms& u [[buffer(0)]]) {
    return float4(presentOut(src.read(uint2(in.position.xy)).rgb, u, in.position.xy, grain), 1.0);
}
// The upscaler (mb0.y = 1, the default: bilinear + sharpen in five taps; else:) Catmull-Rom from the rendered part (view.xy of the source; taps clamped to [0, view.zw]) in five
// bilinear taps (the four corner taps of the 4 x 4 kernel have weights ~0 and are left out), then sharpened by its
// difference to the mean of four bilinear taps one source texel away (mb0.x) and limited to those taps' range (no halos).
// texel.xy: 1 / source size. The source holds display values, as MetalFX's perceptual mode.
float3 upTap(texture2d<float> src, float2 p, constant PostUniforms& u) { return src.sample(postLin, clamp(p, float2(0.0), u.view.zw)).rgb; }
fragment float4 fsUpscale(FullOut in [[stage_in]], texture2d<float> src [[texture(0)]], texture2d<float> grain [[texture(1)]],
                          constant PostUniforms& u [[buffer(0)]]) {
    const float2 uv = in.uv * u.view.xy;
    if (u.mb0.y > 0.0) {  // bilinear + sharpen (5 taps)
        const float3 c = upTap(src, uv, u);
        const float3 a = upTap(src, uv + float2(u.texel.x, 0), u), b = upTap(src, uv - float2(u.texel.x, 0), u);
        const float3 d = upTap(src, uv + float2(0, u.texel.y), u), e = upTap(src, uv - float2(0, u.texel.y), u);
        const float3 lo = min(c, min(min(a, b), min(d, e))), hi = max(c, max(max(a, b), max(d, e)));
        return float4(presentOut(clamp(c + u.mb0.x * (c - 0.25 * (a + b + d + e)), lo, hi), u, in.position.xy, grain), 1.0);
    }
    const float2 size = 1.0 / u.texel.xy;
    const float2 sp = uv * size;
    const float2 t1 = floor(sp - 0.5) + 0.5;
    const float2 f = sp - t1;
    const float2 w0 = f * (-0.5 + f * (1.0 - 0.5 * f));
    const float2 w1 = 1.0 + f * f * (-2.5 + 1.5 * f);
    const float2 w2 = f * (0.5 + f * (2.0 - 1.5 * f));
    const float2 w3 = f * f * (-0.5 + 0.5 * f);
    const float2 w12 = w1 + w2;
    const float2 t0 = (t1 - 1.0) * u.texel.xy, t3 = (t1 + 2.0) * u.texel.xy, t12 = (t1 + w2 / w12) * u.texel.xy;
    const float k0 = w12.x * w0.y, k1 = w0.x * w12.y, k2 = w12.x * w12.y, k3 = w3.x * w12.y, k4 = w12.x * w3.y;
    const float3 c = (upTap(src, float2(t12.x, t0.y), u) * k0 + upTap(src, float2(t0.x, t12.y), u) * k1 + upTap(src, t12, u) * k2 + upTap(src, float2(t3.x, t12.y), u) * k3 +
                      upTap(src, float2(t12.x, t3.y), u) * k4) / (k0 + k1 + k2 + k3 + k4);
    if (u.mb0.x <= 0.0) return float4(presentOut(c, u, in.position.xy, grain), 1.0);  // Catmull-Rom alone (5 taps)
    const float3 a = upTap(src, uv + float2(u.texel.x, 0), u), b = upTap(src, uv - float2(u.texel.x, 0), u);
    const float3 d = upTap(src, uv + float2(0, u.texel.y), u), e = upTap(src, uv - float2(0, u.texel.y), u);
    const float3 lo = min(min(a, b), min(d, e)), hi = max(max(a, b), max(d, e));
    const float3 s = clamp(c + u.mb0.x * (c - 0.25 * (a + b + d + e)), min(lo, c), max(hi, c));
    return float4(presentOut(s, u, in.position.xy, grain), 1.0);
}

// ---- --rendercheck: the shader maths on test inputs (see rendercheck.mm for the input layout).
// ---- PK28: scat_rotshape.vhl vh_front / vh_normal in view space (+z forward, y up) for corner k of the quad 0xa26a80
// (0x859bc0's strip: P = (1,1,1), (0,1,1), (1,0,0), (0,0,0)): pivot pv, world up in view space `up`, RSHAPE_UPLEFT /
// RSHAPE_DWRIGHT, RSHAPE_SCALE s and GOURAUD_DFLT.w alpha (at or below C_INV256 the quad collapses to the pivot).
float3 rotShapeCorner(float3 pv, float3 up, float3 upLeft, float3 downRight, float s, float alpha, bool front, uint k) {
    if (!(alpha > 1.0 / 256.0)) return pv;
    const float3 Pw[4] = {float3(1, 1, 1), float3(0, 1, 1), float3(1, 0, 0), float3(0, 0, 0)};
    const float3 P = Pw[k & 3u];
    const float3 pos = (1.0 - P) * downRight + P * upLeft;
    float3 X, Y, Z;
    if (front) {  // vh_front: Z = normalize(pivot), X = viewAxis x Z, Y = Z x X; Z *= -scale
        Z = normalize(pv);
        X = cross(normalize(up), Z);
        Y = cross(Z, X);
        X *= s, Y *= s, Z *= -s;
    } else {  // vh_normal: Y = normalize(viewAxis), X = normalize(Y x pivot), Z = X x Y
        Y = normalize(up);
        X = normalize(cross(Y, pv));
        Z = cross(X, Y);
        X *= s, Y *= s, Z *= s;
    }
    return X * pos.x + Y * pos.y + Z * pos.z + pv;
}
// ---- PK28 end

constant float4 kNoFlash[4] = {float4(0.0), float4(0.0), float4(0.0), float4(0.0)};

kernel void kRenderCheck(constant FrameUniforms& f [[buffer(0)]], const device float4* in [[buffer(1)]], device float4* out [[buffer(2)]],
                         constant float4* omnis [[buffer(3)]],  // PK28: the omni rows
                         texture2d<float> spot [[texture(0)]], texture2d<float> grainTex [[texture(1)]], uint id [[thread_position_in_grid]]) {
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
        case 6: {  // ground band of a vertex of chunk size a.x at xz = a.yz with the fine tile rect b (groundBand's lattice):
                   // C, lattice L, tile spacing, max distance of the pixel's triangle corners from their tile texel centres
            const float C = groundBandChunk(a.x), L = C / 24.0;
            const float sp = C <= 24.5 ? 1.0 : (C <= 48.5 ? 2.0 : 4.0);
            float2 p[3];
            float3 bary;
            uint upper;
            groundTriangle(a.yz, L, p, bary, upper);
            float off = 0.0;
            for (int i = 0; i < 3; ++i) {
                const float2 it = floor((p[i] - b.xy) / sp - 0.5 + 0.5);
                const float2 centre = b.xy + (it + 0.5) * sp;
                off = max(off, max(abs(centre.x - p[i].x), abs(centre.y - p[i].y)));
            }
            r0 = float4(C, L, sp, off);
            break;
        }
        case 7: {  // P74: car occlusion factor 1 - f f f at world point a.xyz (boxes from shaderCheck), box count
            r0 = float4(carOcclusion(f, a.xyz, spot), f.lightInfo.x, 0, 0);
            break;
        }
        case 8: {  // P74: sun sprite uv of view direction a.xyz for light direction b.xyz (-1 off the quad)
            r0 = float4(sunSpriteUv(normalize(a.xyz), normalize(b.xyz)), 0, 0);
            break;
        }
        case 10: {  // P77: the vehicle omnis' sum at a.xyz for the normal b.xyz (lights from setVehicleLights), count
            r0 = float4(fuelOmnis(f, omnis, a.xyz, normalize(b.xyz)), f.lightInfo.y);
            break;
        }
        case 9: {  // P77: 0x5095b0 blob corner nudge of a.xyz towards b.xyz, view up c.xyz
            r0 = float4(blobNudge(a.xyz, b.xyz, normalize(c.xyz)), 0);
            break;
        }
        case 5: {  // vh_cloud corner: cloud a.xyz, half size b.xy, corner b.zw, C_USER0 = c, C_USER6 = d, C_USER5.x -100
            float alpha, shade, lightning;
            r0 = float4(fuelCloudCorner(f, a.xyz, b.xy, b.zw, c, float4(-100, 0, 0, 0), d, alpha, shade, kNoFlash, lightning), alpha);
            r1 = float4(shade, 0, 0, 0);
            break;
        }
        case 11: {  // P102: film grain of display colour b.rgb at pixel a.xy, frame a.z, cell c.z, amplitude c.x
            r0 = float4(p102Grain(b.rgb, a.xy, float4(c.x, a.z, c.z, 0), grainTex), 0);
            r1 = float4(p102GrainNoise(grainTex, a.xy, c.z, a.z), p102GrainWeight(b.x), grainTex.read(uint2(a.xy) & 255u).r, 0);
            break;
        }
        case 12: {  // PK28: RotShape corner of pivot a.xyz (view space): UPLEFT b.xyz, b.w = front (1 = vh_front) + 2 x corner,
                    // DWRIGHT c.xyz, alpha c.w, up d.xyz, scale d.w
            const uint bw = uint(b.w + 0.5);
            r0 = float4(rotShapeCorner(a.xyz, d.xyz, b.xyz, c.xyz, d.w, c.w, (bw & 1u) != 0u, bw >> 1), 0);
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
    o.position = fuelClip(f, w);
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
                        depth2d_array<float> shadowMap [[texture(7)]], const device ulong* omniClusters [[buffer(5)]],
                        constant float4* omnis [[buffer(6)]]) {  // PK28
    constexpr sampler smp(coord::normalized, filter::linear, mip_filter::linear, address::clamp_to_edge, max_anisotropy(4));
    float4 d = atlas.sample(smp, in.uv, bias(f.taa.x));  // PK28: 0x84ca80 LOD bias
    if (d.a < 0.4) discard_fragment();
    // ground_rotshape.phl: vcol x tex x lerp(sky, sunlit + omni, sunlighting(shadow, 0.5)) + inscatter; omni = the
    // deferred lights (P77) with the tuft's normal (port: up, as for its sunlit term).
    const float shadow = fuelShadow(shadowMap, in.world, f);
    const float3 omni = fuelOmnisMasked(f, omnis, in.world, float3(0, 1, 0), omniMask(f, omniClusters, in.position.xy, in.world));  // P92
    const float3 rgb = d.rgb * in.vcol * mix(in.sky, in.sunlit + omni, fuelSunlighting(shadow, 0.5)) + in.insc;
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
    simd_float4 carOcc[48];  // P74: 3 rows per car occlusion omni (see the MSL struct)
    simd_float4 lightInfo;   // x car occlusion count, y vehicle omni count, w sun sprite texture loaded
    simd_float4 decalBias[2];  // PK28: the decal bitmaps' +0x24 (the omni rows moved to omniRows_ / buffer 6)
    simd_float4 cluster, clusterZ, clusterView, camFwd;  // P92: headlight clusters (see the MSL struct; cluster.x = 0: off)
    simd_float4x4 viewProjStable;      // TAA: viewProj without the temporal jitter (see the MSL struct)
    simd_float4 taa = {-1, 0, 0, 0};   // TAA: x the samples' LOD bias (FUEL's -1 by default), yz jitter (ndc), w temporal
};
static_assert(sizeof(FrameUniforms) <= 4096, "FrameUniforms goes through setVertexBytes");

void frameCameraOf(const FrameUniforms& f, simd_float4x4& viewProj, simd_float3& eye, simd_float3& right, simd_float3& up) {  // P88 traces
    viewProj = f.viewProjStable, eye = f.eye.xyz, right = f.camRight.xyz, up = f.camUp.xyz;  // TAA: without the jitter
}

namespace {

constexpr uint32_t kTerrainGridSize = 1024;  // vertices per side
constexpr NSUInteger kCubeSize = 128;         // gCubeMapZ / viewport 0x80 x 0x80 (0x837950, 0x757c20)

struct MaterialUniforms {
    simd_float4 diffuse;
    simd_float4 specular;
    simd_float4 emissive;
    simd_float2 fresnel;
    uint32_t alphaMode;
    uint32_t lit;
    uint32_t code;
    uint32_t features;
    simd_float4 uv0;
    simd_float4 uv1;
    simd_float4 lodBias;   // PK28: Bitmap_Z +0x24 of slots 0 / 2 / 3 / 4
    simd_float4 lodBias2;  // PK28: slots 5 / 6
};

struct PostUniforms {
    simd_float4 bloom, hdr, blueShift, tint, mb0, texel, flags;
    simd_float4 taps[11];
    simd_float4 mbw[8];
    simd_float4 fade;
    simd_float4 view;  // P93
    simd_float4 grain;  // P102 [DEVIATION]: film grain (amplitude, frame seed, cell px, 0)
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

// ---- P92: conservative view culling for the scene pass. The four side planes of the view through the eye (normals
// pointing out, unit length); a box or sphere wholly outside one of them cannot put a fragment on screen (with the
// eye on every plane, a point behind the eye is outside one of them too). What is drawn keeps its order, so the
// picture is the same; only the vertex work of what is off screen goes.
struct ViewCull {
    bool on = false;
    simd_float3 eye = {0, 0, 0};
    std::array<simd_float3, 4> n{};
    void set(simd_float3 e, simd_float3 right, simd_float3 up, simd_float3 fwd, float tanX, float tanY) {
        eye = e;
        n[0] = simd_normalize(right - fwd * tanX), n[1] = simd_normalize(-right - fwd * tanX);
        n[2] = simd_normalize(up - fwd * tanY), n[3] = simd_normalize(-up - fwd * tanY);
        on = std::isfinite(tanX) && std::isfinite(tanY) && tanX > 0 && tanY > 0;
    }
    bool sphereOutside(simd_float3 c, float r, float margin = 1.0f) const {
        if (!on) return false;
        const simd_float3 d = c - eye;
        for (const simd_float3& p : n)
            if (simd_dot(d, p) > r + margin) return true;
        return false;
    }
    // Margin in metres on top (float rounding of the box and of the shader's clip maths).
    bool boxOutside(simd_float3 lo, simd_float3 hi, float margin = 1.0f) const {
        if (!on) return false;
        const simd_float3 c = (lo + hi) * 0.5f - eye, h = (hi - lo) * 0.5f;
        for (const simd_float3& p : n)
            if (simd_dot(c, p) - simd_dot(h, simd_abs(p)) > margin) return true;
        return false;
    }
};

ResourceLibrary::Hit ResourceLibrary::find(uint32_t id, uint32_t classCrc) const {
    for (const auto& f : files_) {
        const DpcFile::Entry* e = f->find(id);
        if (e != nullptr && e->classCrc == classCrc) return {f.get(), e};
    }
    return {};
}

extern const char* kTornadoShaderSource;  // P65: tornado_render.mm (uses kShaderSource's helpers)
extern const char* kLensShaderSource;     // PK24: lens_render.mm
extern const char* kVehicleFxShaderSource;  // PK23: vehicle_fx_render.mm
extern const char* kSandShaderSource;       // PK12: sand_render.mm (after the tornado source: uses its helpers)
extern const char* kTaaShaderSource;        // TAA: taa_render.mm (after the grass / RotShape sources: uses their structs)

// ---- PK28: RotShape_Z (scat_rotshape.vhl vh_front / vh_normal + scat_rotshape.phl ph_base, written from their decode).
// One record = the quad 0xa26a80 (0x859bc0: strip of P = (1,1,1) uv (1,0), (0,1,1) (0,0), (1,0,0) (1,1), (0,0,0) (0,1))
// expanded in view space around the pivot: pos = (1 - P) DWRIGHT + P UPLEFT; front: Z = normalize(pivot), X = up x Z,
// Y = Z x X, Z *= -scale; normal: Y = up, X = normalize(Y x pivot), Z = X x Y (up = world y in view space, the view's
// y column). GOURAUD_DFLT.w (the view alpha) <= 1/256 collapses the quad. Lighting at the pivot (vh_base_lightning):
// sky = skylight + DSKY_AMBIENT (K.y / 2 + 1 / 2), sun = sky + sunlight sat(L.y), both x MTL_DIFFUSE x extinction;
// emissive = MTL_EMISSIVE x DLIGHT_INVINTENSITY x extinction; inscatter x MTL_ADDSOUS_NULL; ph_base: tex.a x alpha,
// rgb = tex (lerp(sky, sun, sunlighting(shadow, 1)) + emissive) + inscatter.
const char* kRotShapeShaderSource = R"METAL(
struct RotRecordGpu {
    float4 pivot, upLeft, downRight, extra;
};
struct RotShapeOut {
    float4 position [[position]];
    float2 uv;
    float3 world;
    float3 ext;
    float3 insc;
    float alpha [[flat]];
};
vertex RotShapeOut vsRotShape(uint vid [[vertex_id]], constant FrameUniforms& f [[buffer(1)]], constant RotRecordGpu& r [[buffer(2)]],
                              constant MaterialUniforms& m [[buffer(3)]]) {
    const float2 UV[4] = {float2(1, 0), float2(0, 0), float2(1, 1), float2(0, 1)};  // 0x859bc0's uv (u = 1 at UPLEFT)
    const float3 R = float3(f.view[0][0], f.view[1][0], f.view[2][0]);
    const float3 U = float3(f.view[0][1], f.view[1][1], f.view[2][1]);
    const float3 F = float3(f.view[0][2], f.view[1][2], f.view[2][2]);
    const float3 d = r.pivot.xyz - f.eye.xyz;
    const float3 pv = float3(dot(R, d), dot(U, d), dot(F, d));  // WorldToViewVertex(RSHAPE_PIVOT)
    const float3 up = float3(R.y, U.y, F.y);                     // c[C_VIEW_CONTEXT + k].y: world y in view space
    const float3 v = rotShapeCorner(pv, up, r.upLeft.xyz, r.downRight.xyz, r.pivot.w, r.downRight.w, r.upLeft.w > 0.5, vid);
    RotShapeOut o;
    const float3 w = f.eye.xyz + R * v.x + U * v.y + F * v.z;
    o.position = fuelClip(f, w);
    o.world = w;
    const float2 uv = UV[vid & 3u];
    o.uv = float2(dot(float3(uv, 1.0), float3(m.uv0.xz, m.uv1.x)), dot(float3(uv, 1.0), float3(m.uv0.yw, m.uv1.y)));  // MaterialMat
    const Scatter sc = fuelScattering(f, r.pivot.xyz);  // at the pivot (vh_base_lightning: wPos = wPivotPos)
    o.ext = sc.ext;
    o.insc = sc.insc * r.extra.x;  // x MTL_ADDSOUS_NULL (0 for 0x1000 / 0x2000 materials)
    o.alpha = r.downRight.w;
    return o;
}
fragment float4 fsRotShape(RotShapeOut in [[stage_in]], constant FrameUniforms& f [[buffer(1)]], constant MaterialUniforms& m [[buffer(2)]],
                           texture2d<float> tDiffuse [[texture(0)]], depth2d_array<float> shadowMap [[texture(7)]], sampler smp [[sampler(0)]]) {
    float4 t = tDiffuse.sample(smp, in.uv, bias(f.taa.x + m.lodBias.x));  // 0x84ca80 LOD bias
    t.a *= in.alpha;                                                          // tDiffuse.a *= GetAlpha(i)
    // ALPHATEST GREATER: 0x835a00's pass bits (0xf / 0x1d / 0x03) leave the test on (no 0x20) with ALPHAREF 0 (no 0x40)
    if (!(t.a > m.uv1.z)) discard_fragment();
    const float shadow = fuelShadow(shadowMap, in.world, f);
    const float3 L = f.lightDir.xyz, K = f.skyDir.xyz;
    const float3 sky = f.skyColour.rgb + f.skyAmbient.rgb * (K.y * 0.5 + 0.5);
    const float3 sun = sky + f.sunColour.rgb * saturate(L.y);
    const float3 vcol = m.diffuse.rgb * in.ext;
    const float3 emis = m.emissive.rgb * f.lightDir.w * in.ext;
    const float3 rgb = t.rgb * (mix(sky, sun, fuelSunlighting(shadow, 1.0)) * vcol + emis) + in.insc;
    return float4(rgb, t.a);  // PackHDRSample(rgb, a): the blend state decides what the alpha does
}
)METAL";
// ---- PK28 end
extern const size_t kFrameUniformsSize;
const size_t kFrameUniformsSize = sizeof(FrameUniforms);

MeshRenderer::MeshRenderer(id<MTLDevice> device, NSUInteger sampleCount, const ResourceLibrary& library)
    : device_(device), sampleCount_(sampleCount), library_(library) {
    NSError* err = nil;
    MTLCompileOptions* shaderOptions = [MTLCompileOptions new];
    shaderOptions.preserveInvariance = YES;  // P72: [[invariant]] positions (fade pre-pass depth == colour pass depth)
    {  // ---- PK28: the renderer's mip LOD bias +0x1104 + +0x1100 (Renderer_RegisterTweaks 0x64faf0: -1.0 + 0.0, read by
       // 0x84ca80). FUEL_LOD_BIAS=<x> replaces it (A / B shots only).
        double texBias = -1.0 + 0.0;
        if (const char* e = std::getenv("FUEL_LOD_BIAS")) texBias = std::atof(e);
        shaderOptions.preprocessorMacros = @{@"FUEL_TEX_BIAS" : [NSString stringWithFormat:@"(%.4f)", texBias]};
        texBiasBase_ = float(std::atof([NSString stringWithFormat:@"%.4f", texBias].UTF8String));  // TAA: FrameUniforms.taa.x, the same value
    }  // ---- PK28 end
    shaders_ = [device newLibraryWithSource:[NSString stringWithFormat:@"%s\n%s\n%s\n%s\n%s\n%s\n%s\n%s", kShaderSource, kGrassShaderSource, kTornadoShaderSource, kLensShaderSource,
                                                                       kVehicleFxShaderSource /* PK23 */, kSandShaderSource /* PK12 */,
                                                                       kRotShapeShaderSource /* PK28 */, kTaaShaderSource /* TAA */]
                                    options:shaderOptions
                                      error:&err];
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
    {  // P102: the film grain's 256 x 256 white-noise tile (renderer.h grainNoiseByte)
        MTLTextureDescriptor* gd = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatR8Unorm width:256 height:256 mipmapped:NO];
        grainTex_ = [device newTextureWithDescriptor:gd];
        std::vector<uint8_t> g(256 * 256);
        for (uint32_t y = 0; y < 256; ++y)
            for (uint32_t x = 0; x < 256; ++x) g[y * 256 + x] = grainNoiseByte(x, y);
        [grainTex_ replaceRegion:MTLRegionMake2D(0, 0, 256, 256) mipmapLevel:0 withBytes:g.data() bytesPerRow:256];
    }

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
    {
        // p57: the global cube (gCubeMapZ 0x837950: 128 x 128) and its single-sample pipelines.
        auto makeCube = [&](NSString* vs, NSString* fs) {
            MTLRenderPipelineDescriptor* d = [MTLRenderPipelineDescriptor new];
            d.vertexFunction = [shaders_ newFunctionWithName:vs];
            d.fragmentFunction = [shaders_ newFunctionWithName:fs];
            d.rasterSampleCount = 1;
            d.colorAttachments[0].pixelFormat = kRenderColorFormat;
            d.depthAttachmentPixelFormat = kRenderDepthFormat;
            NSError* e = nil;
            id<MTLRenderPipelineState> ps = [device newRenderPipelineStateWithDescriptor:d error:&e];
            if (ps == nil) [NSException raise:@"pipeline" format:@"%@", e];
            return ps;
        };
        cubeSkyPipeline_ = makeCube(@"vsSky", @"fsSky");
        cubeTerrainPipeline_ = makeCube(@"vsTerrain", @"fsTerrain");
        MTLTextureDescriptor* cd = [MTLTextureDescriptor textureCubeDescriptorWithPixelFormat:kRenderColorFormat
                                                                                         size:kCubeSize
                                                                                    mipmapped:NO];
        cd.usage = MTLTextureUsageRenderTarget | MTLTextureUsageShaderRead;
        cd.storageMode = MTLStorageModePrivate;
        globalCube_ = [device newTextureWithDescriptor:cd];
        MTLTextureDescriptor* dd = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:kRenderDepthFormat
                                                                                       width:kCubeSize
                                                                                      height:kCubeSize
                                                                                   mipmapped:NO];
        dd.usage = MTLTextureUsageRenderTarget;
        dd.storageMode = MTLStorageModePrivate;
        cubeDepth_ = [device newTextureWithDescriptor:dd];
    }
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
        } else if (blend == 3) {  // additive by alpha, alpha kept
            d.colorAttachments[0].blendingEnabled = YES;
            d.colorAttachments[0].sourceRGBBlendFactor = MTLBlendFactorSourceAlpha;
            d.colorAttachments[0].destinationRGBBlendFactor = MTLBlendFactorOne;
            d.colorAttachments[0].writeMask = MTLColorWriteMaskRed | MTLColorWriteMaskGreen | MTLColorWriteMaskBlue;
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
    makeTornadoPipelines();  // P65
    makeSandPipeline();      // PK12
    createLensPipeline();    // PK24
    createVehicleFxPipeline();  // PK23
    decalPipeline_ = makePost(@"vsDecal", @"fsDecal", kRenderColorFormat, 1);
    blobPipeline_ = makePost(@"vsBlob", @"fsBlob", kRenderColorFormat, 1);  // P77
    fxPipeline_ = makePost(@"vsFx", @"fsFx", kRenderColorFormat, 1);
    particlePipeline_ = makePost(@"vsParticle", @"fsParticle", kRenderColorFormat, 1);
    particleAddPipeline_ = makePost(@"vsParticle", @"fsParticle", kRenderColorFormat, 3);
    shrinkPipeline_ = makePost(@"vsFull", @"fsShrink", kRenderColorFormat, 0);
    lumLogPipeline_ = makePost(@"vsFull", @"fsLumLog", MTLPixelFormatR32Float, 0);
    {  // P95: shrink + log luminance (two targets) and the reductions in one compute pass
        MTLRenderPipelineDescriptor* d = [MTLRenderPipelineDescriptor new];
        d.vertexFunction = [shaders_ newFunctionWithName:@"vsFull"];
        d.fragmentFunction = [shaders_ newFunctionWithName:@"fsShrinkLumLog"];
        d.colorAttachments[0].pixelFormat = kRenderColorFormat;
        d.colorAttachments[1].pixelFormat = MTLPixelFormatR32Float;
        NSError* e = nil;
        shrinkLumPipeline_ = [device newRenderPipelineStateWithDescriptor:d error:&e];
        lumReducePipeline_ = [device newComputePipelineStateWithFunction:[shaders_ newFunctionWithName:@"kLumReduce"] error:&e];
        if (shrinkLumPipeline_ == nil || lumReducePipeline_ == nil) [NSException raise:@"pipeline" format:@"%@", e];
    }
    lumAvgPipeline_ = makePost(@"vsFull", @"fsLumAvg", MTLPixelFormatR32Float, 0);
    adaptPipeline_ = makePost(@"vsFull", @"fsAdapt", MTLPixelFormatR32Float, 0);
    brightPipeline_ = makePost(@"vsFull", @"fsBright", MTLPixelFormatRGBA8Unorm, 0);
    gaussPipeline_ = makePost(@"vsFull", @"fsGauss", MTLPixelFormatRGBA8Unorm, 0);
    toneMapPipeline_ = makePost(@"vsFull", @"fsToneMap", kOutputColorFormat, 0);
    toneMapFxPipeline_ = makePost(@"vsFull", @"fsToneMap", MTLPixelFormatRGBA8Unorm, 0);
    radialPipeline_ = makePost(@"vsFull", @"fsRadial", kOutputColorFormat, 0);
    // P93: dynamic resolution: the radial blur into the scaled frame (display values), the present copy and the
    // fallback upscaler into the output.
    radialLowPipeline_ = makePost(@"vsFull", @"fsRadial", MTLPixelFormatRGBA8Unorm, 0);
    presentPipeline_ = makePost(@"vsFull", @"fsPresent", kOutputColorFormat, 0);
    sharpenPipeline_ = makePost(@"vsFull", @"fsUpscale", kOutputColorFormat, 0);
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

// P74: sProjSpotMap0 of the car occlusion omni: the world's Omni_Z named OMBREVOITURES (0x5cc410 instances it), its
// first texture handle (Omni_Z +0x12c, body 0xc0 after the 0xc0 bytes of +0x6c..+0x12b, vf03 0x690b50) -> the
// MaterialAnim_Z's material (+0x84, 0x690570) -> that Material_Z's diffuse bitmap (+0x9c, bound by 0x84f240).
void MeshRenderer::loadCarSpot(const DpcFile& world) {
    carSpot_ = nil;
    const uint32_t name = asoboCrc("OMBREVOITURES");
    try {
        for (uint32_t id : world.ids(cls::Omni)) {
            const DpcFile::Entry* e = world.find(id);
            if (e == nullptr || parseObjectLink(world.link(*e)).name != name) continue;
            const auto body = world.body(*e);
            if (!body || body->size() < 0xc4) return;
            uint32_t handle = 0;
            std::memcpy(&handle, body->data() + 0xc0, 4);
            uint32_t material = handle;
            const DpcFile::Entry* h = world.find(handle);
            if (h != nullptr && h->classCrc == cls::MaterialAnimZ) {
                const auto ab = world.body(*h);
                if (!ab) return;
                material = parseMaterialAnimBody(*ab).material;
            }
            const DpcFile::Entry* me = world.find(material);
            if (me == nullptr || me->classCrc != cls::Material) return;
            const auto mb = world.body(*me);
            if (!mb) return;
            const Material m = parseMaterialBody(*mb);
            const DpcFile::Entry* be = world.find(m.textures[0]);
            uint8_t alpha = 0;
            if (be != nullptr) carSpot_ = uploadBitmap(world, *be, false, &alpha);
            return;
        }
    } catch (const std::exception&) {
        carSpot_ = nil;  // no car occlusion without the spot map
    }
}

void MeshRenderer::setGround(const GroundMaps& maps, const DpcFile& world) {
    loadCarSpot(world);
    {  // P74: the sun / moon sprite: GenWorld material enum 0x36 (dispatcher 0x779440 case 0x19, W+0xb75ac[0x36])
        sunTex_ = nil;
        uint8_t alpha = 0;
        if (const uint32_t bm = genWorldEnumTexture(world, 0x36))
            if (const DpcFile::Entry* e = world.find(bm)) sunTex_ = uploadBitmap(world, *e, false, &alpha);
    }
    // Engine settings read from the player's files: the post defaults and shadow table from FUEL.exe,
    // the clock scale PP371 from pp001.tsc.
    if (!engineSettingsRead_) {
        engineSettingsRead_ = true;
        {
            std::string err;
            auto atlas = sfx::replayAtlas(engineExePath(world), &err);
            if (!atlas.empty()) sfxTables_ = std::make_shared<const std::vector<sfx::AtlasEntry>>(std::move(atlas));
        }
        loadSfxAtlas(world);
        weatherFields_ = weatherFieldsFor(world);
        // ---- PK04: the game folder (CommonComponent's parent) for the race weather lines of hubinfos.tsc
        gameRoot_ = std::filesystem::path(world.path()).parent_path().parent_path().parent_path().string();
        // ---- end PK04
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
        // P92: kGroundDrawSet's output (two triangles per cell of levels 0 and 1).
        ud.pixelFormat = MTLPixelFormatRGBA32Uint;
        ud.arrayLength = 4;
        ud.usage = MTLTextureUsageShaderRead | MTLTextureUsageShaderWrite;
        ud.storageMode = MTLStorageModePrivate;
        groundDrawSet_ = [device_ newTextureWithDescriptor:ud];
        if (drawSetPipeline_ == nil) {
            NSError* e = nil;
            drawSetPipeline_ = [device_ newComputePipelineStateWithFunction:[shaders_ newFunctionWithName:@"kGroundDrawSet"] error:&e];
            if (drawSetPipeline_ == nil) [NSException raise:@"pipeline" format:@"%@", e];
        }
        // Road decal textures (sDiffuse of ground.phl's decal path), by slot.
        const uint16_t enums[kDecalSlots] = {0x2a, 0x2b, 0x2c, 0x2d, 0x2e, 0x34};
        for (int k = 0; k < kDecalSlots; ++k) {
            const uint32_t bm = genWorldEnumTexture(world, enums[k]);
            const DpcFile::Entry* e = bm != 0 ? world.find(bm) : nullptr;
            uint8_t alpha = 0;
            decalTex_[k] = e != nullptr ? uploadBitmap(world, *e, false, &alpha) : nil;
            decalLodBias_[k] = 0.0f;  // ---- PK28: the decal bitmap's own +0x24 (0x84ca80)
            if (e != nullptr) try {
                    decalLodBias_[k] = parseBitmapLink(world.link(*e)).lodBias;
                } catch (const std::exception&) {
                }  // ---- PK28 end
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
    // P92: the ground's CPU-filled textures in the GPU's layout (the same texels; faster to sample).
    for (id<MTLTexture> t : {groundColour_, groundSurface_, groundDetail_}) [blit optimizeContentsForGPUAccess:t];
    [blit endEncoding];
    // P92: the headlight cluster kernel made and run once now (one cluster, no omnis, into a scratch buffer): its first
    // dispatch cost the GPU ~45 ms, in the frame the headlights first came on.
    if (omniClusterPipeline_ == nil) {
        NSError* e = nil;
        omniClusterPipeline_ = [device_ newComputePipelineStateWithFunction:[shaders_ newFunctionWithName:@"kOmniClusters"] error:&e];
        if (omniClusterPipeline_ == nil) [NSException raise:@"pipeline" format:@"%@", e];
    }
    {
        FrameUniforms warm{};
        warm.cluster = simd_make_float4(64, 1, 1, 1);
        warm.clusterZ = simd_make_float4(1, 1, 1000, 0);
        warm.clusterView = simd_make_float4(64, 64, 1, 1);
        id<MTLBuffer> scratch = [device_ newBufferWithLength:16 options:MTLResourceStorageModePrivate];
        id<MTLComputeCommandEncoder> ce = [cb computeCommandEncoder];
        [ce setComputePipelineState:omniClusterPipeline_];
        [ce setBytes:&warm length:sizeof warm atIndex:0];
        [ce setBuffer:scratch offset:0 atIndex:1];
        [ce setBuffer:scratch offset:0 atIndex:2];  // PK28: the light rows (none: lightInfo.y = 0)
        [ce dispatchThreads:MTLSizeMake(1, 1, 1) threadsPerThreadgroup:MTLSizeMake(1, 1, 1)];
        [ce endEncoding];
    }
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

// P95: the vsMain / fsMain variant a draw needs (bits: 1 the instance rotation, 2 the ground tint, 4 the tangent, 8 the
// normal; 15 = all outputs, the shaders as they were). See VertexOut.
uint32_t MeshRenderer::vsVariant(const GpuMaterial& m, bool sprite) const {
    if (p95Off & P95SlimVaryings) return 15u;
    uint32_t v = 0;
    if (!sprite) v |= 8u;  // PK28: 0x1000 / 0x2000 materials are lit too (MTL_ADDSOUS_NULL only drops the inscatter)
    if (m.features & HasNormalLocal) v |= 1u;
    if (sprite || (m.features & GroundTint)) v |= 2u;
    if ((m.features & HasNormal) && !(m.features & HasNormalLocal)) v |= 4u;
    if (m.features & Carbon) v |= 16u | 8u;  // [upgrades U5] the carbon paint (needs the normal too)
    return v;
}

// P95: a pipeline with a variant other than 15 compiles in the background (no frame waits for it); until it is ready the
// draw takes the full variant (the same picture). FUEL_FIXED_CLOCK / FUEL_SYNC_PIPELINES (shots) compile it at once.
id<MTLRenderPipelineState> MeshRenderer::mainPipeline(uint64_t key, uint32_t variant, MTLRenderPipelineDescriptor* (^describe)(void)) {
    if (auto it = pipelines_.find(key); it != pipelines_.end()) return it->second;
    auto make = [&](MTLRenderPipelineDescriptor* pd) {
        if (variant != 15u) {
            MTLFunctionConstantValues* fc = [MTLFunctionConstantValues new];
            const bool inst = variant & 1u, tint = variant & 2u, tangent = variant & 4u, normal = variant & 8u;
            [fc setConstantValue:&inst type:MTLDataTypeBool atIndex:10];
            [fc setConstantValue:&tint type:MTLDataTypeBool atIndex:11];
            [fc setConstantValue:&tangent type:MTLDataTypeBool atIndex:12];
            [fc setConstantValue:&normal type:MTLDataTypeBool atIndex:14];
            if (variant & 16u) {  // [upgrades U5] only the carbon variants define it (the others compile as before)
                const bool carbon = true;
                [fc setConstantValue:&carbon type:MTLDataTypeBool atIndex:16];
            }
            NSError* e = nil;
            pd.vertexFunction = [shaders_ newFunctionWithName:@"vsMain" constantValues:fc error:&e];
            pd.fragmentFunction = [shaders_ newFunctionWithName:@"fsMain" constantValues:fc error:&e];
            if (pd.vertexFunction == nil || pd.fragmentFunction == nil) [NSException raise:@"pipeline" format:@"%@", e];
        } else {  // (functions with function constants are made through constantValues: none given = every output)
            MTLFunctionConstantValues* none = [MTLFunctionConstantValues new];
            NSError* e = nil;
            pd.vertexFunction = [shaders_ newFunctionWithName:@"vsMain" constantValues:none error:&e];
            pd.fragmentFunction = [shaders_ newFunctionWithName:@"fsMain" constantValues:none error:&e];
            if (pd.vertexFunction == nil || pd.fragmentFunction == nil) [NSException raise:@"pipeline" format:@"%@", e];
        }
    };
    static const bool sync = std::getenv("FUEL_FIXED_CLOCK") != nullptr || std::getenv("FUEL_SYNC_PIPELINES") != nullptr;
    if (variant == 15u || sync || (variant & 16u)) {  // [upgrades U5] a carbon variant has no full-variant stand-in: made now
        MTLRenderPipelineDescriptor* pd = describe();
        make(pd);
        NSError* err = nil;
        id<MTLRenderPipelineState> ps = [device_ newRenderPipelineStateWithDescriptor:pd error:&err];
        if (ps == nil) [NSException raise:@"pipeline" format:@"%@", err];
        pipelines_[key] = ps;
        ++oneOff_.pipelines;  // P92: a compile at first use (a hitch cause)
        return ps;
    }
    {
        std::lock_guard<std::mutex> lk(asyncPipeMutex_);
        if (auto it = asyncPipes_.find(key); it != asyncPipes_.end()) {
            if (it->second != nil) {
                pipelines_[key] = it->second;
                return it->second;
            }
            return nil;  // still compiling
        }
        asyncPipes_[key] = nil;
    }
    MTLRenderPipelineDescriptor* pd = describe();
    make(pd);  // (the specialised functions; the pipeline itself compiles off this thread)
    std::mutex* mu = &asyncPipeMutex_;
    auto* ready = &asyncPipes_;
    [device_ newRenderPipelineStateWithDescriptor:pd completionHandler:^(id<MTLRenderPipelineState> ps, NSError* err) {
        if (ps == nil) std::fprintf(stderr, "fuelview: pipeline variant %u: %s\n", variant, err.localizedDescription.UTF8String);
        std::lock_guard<std::mutex> lk(*mu);
        (*ready)[key] = ps;  // nil on failure: the full variant stays in use
    }];
    return nil;
}

id<MTLRenderPipelineState> MeshRenderer::pipeline(uint32_t stride, AlphaMode mode, uint32_t variant) {
    const uint64_t key = (uint64_t(variant) << 40) | (uint64_t(stride) << 8) | uint64_t(mode);
    id<MTLRenderPipelineState> ps = mainPipeline(key, variant, ^MTLRenderPipelineDescriptor*() {
        MTLRenderPipelineDescriptor* pd = [MTLRenderPipelineDescriptor new];
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
        return pd;
    });
    return ps != nil ? ps : pipeline(stride, mode, 15u);
}

// ---- PK28: Metal factors of 0x856160's D3D9 blend states (BLENDOP / SRCBLEND / DESTBLEND; D3D9 without
// SEPARATEALPHABLENDENABLE blends the alpha channel with the same factors). Alpha keeps P72's alpha channel (ONE /
// INVSRCALPHA): nothing reads the scene alpha after the colour pass, and no shipped material is 0x4000 (DESTALPHA).
MetalBlend metalBlend(fuel::BlendKind kind) {
    const fuel::BlendState b = fuel::blendState(kind == fuel::BlendKind::Add ? 0x1000u : kind == fuel::BlendKind::AddDestAlpha ? 0x4000u
                                                : kind == fuel::BlendKind::RevSubtract ? 0x2000u : 0u);
    auto factor = [](uint32_t d3d) {
        switch (d3d) {
            case 2: return MTLBlendFactorOne;
            case 5: return MTLBlendFactorSourceAlpha;
            case 6: return MTLBlendFactorOneMinusSourceAlpha;
            case 7: return MTLBlendFactorDestinationAlpha;
            default: return MTLBlendFactorZero;
        }
    };
    MetalBlend m;
    m.op = b.op == 3 ? MTLBlendOperationReverseSubtract : b.op == 2 ? MTLBlendOperationSubtract : MTLBlendOperationAdd;
    m.src = factor(b.src), m.dst = factor(b.dst);
    m.srcAlpha = kind == fuel::BlendKind::Alpha ? MTLBlendFactorOne : m.src;
    m.dstAlpha = m.dst;
    return m;
}

static void applyBlend(MTLRenderPipelineColorAttachmentDescriptor* ca, fuel::BlendKind kind) {
    const MetalBlend m = metalBlend(kind);
    ca.blendingEnabled = YES;
    ca.rgbBlendOperation = ca.alphaBlendOperation = m.op;
    ca.sourceRGBBlendFactor = m.src, ca.destinationRGBBlendFactor = m.dst;
    ca.sourceAlphaBlendFactor = m.srcAlpha, ca.destinationAlphaBlendFactor = m.dstAlpha;
}

// ---- PK28: a blended draw's pipeline with the material's 0x856160 blend mode (fuel::blendKind): Alpha is the P72 Blend
// pipeline; the others get their own key (low byte 0x10 | kind).
id<MTLRenderPipelineState> MeshRenderer::blendPipeline(uint32_t stride, uint32_t flags, uint32_t variant, bool warmOnly) {
    const fuel::BlendKind kind = fuel::blendKind(flags);
    if (kind == fuel::BlendKind::Alpha) return warmOnly ? nil : pipeline(stride, AlphaMode::Blend, variant);
    MTLRenderPipelineDescriptor* (^describe)(void) = ^MTLRenderPipelineDescriptor*() {
        MTLRenderPipelineDescriptor* pd = [MTLRenderPipelineDescriptor new];
        pd.vertexDescriptor = meshVertexDescriptor(stride);
        pd.rasterSampleCount = sampleCount_;
        pd.colorAttachments[0].pixelFormat = kRenderColorFormat;
        pd.depthAttachmentPixelFormat = kRenderDepthFormat;
        applyBlend(pd.colorAttachments[0], kind);
        return pd;
    };
    const uint64_t key = (uint64_t(variant) << 40) | (uint64_t(stride) << 8) | (0x10u | uint64_t(kind));
    if (warmOnly) {  // buildMesh: start the specialised variant's compile off the main thread
        if (variant != 15u) (void)mainPipeline(key, variant, describe);
        return nil;
    }
    if (id<MTLRenderPipelineState> ps = mainPipeline(key, variant, describe)) return ps;
    // Still compiling: the full variant of this blend if it exists; else the alpha-blend pipeline for the few frames until
    // it is ready [P] (compiling the full additive variant here cost a 190 ms frame in --perf-check).
    const uint64_t full = (uint64_t(15u) << 40) | (uint64_t(stride) << 8) | (0x10u | uint64_t(kind));
    if (auto it = pipelines_.find(full); it != pipelines_.end()) return it->second;
    return pipeline(stride, AlphaMode::Blend, variant);
}
// ---- PK28 end

id<MTLRenderPipelineState> MeshRenderer::prepassPipeline(uint32_t stride, uint32_t variant) {
    const uint64_t key = (uint64_t(variant) << 40) | (uint64_t(stride) << 8) | 0xffu;
    id<MTLRenderPipelineState> ps = mainPipeline(key, variant, ^MTLRenderPipelineDescriptor*() {
        MTLRenderPipelineDescriptor* pd = [MTLRenderPipelineDescriptor new];  // fsMain keeps the alpha test / cut-out discards
        pd.vertexDescriptor = meshVertexDescriptor(stride);
        pd.rasterSampleCount = sampleCount_;
        pd.colorAttachments[0].pixelFormat = kRenderColorFormat;
        pd.colorAttachments[0].writeMask = MTLColorWriteMaskNone;  // pass bits 0x83: ZENABLE | ZWRITE, no COLORWRITE
        pd.depthAttachmentPixelFormat = kRenderDepthFormat;
        return pd;
    });
    return ps != nil ? ps : prepassPipeline(stride, 15u);
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
    ++oneOff_.pipelines;  // P92
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
                    toOptimize_.push_back(t);  // P92: GPU layout at the next frame
                }
            }
        } catch (const std::exception&) {
            // Textures that do not decode are treated as missing.
        }
        *alphaKind = alpha;
    return result;
}

// ---- PK28: Bitmap_Z +0x24, the per-texture term of 0x84ca80's D3DSAMP_MIPMAPLODBIAS (the shaders add FrameUniforms.taa.x).
float MeshRenderer::bitmapLodBias(uint32_t bitmapId) {
    if (bitmapId == 0) return 0.0f;
    if (auto it = textureBias_.find(bitmapId); it != textureBias_.end()) return it->second;
    float b = 0.0f;
    if (auto hit = library_.find(bitmapId, cls::Bitmap)) {
        try {
            b = parseBitmapLink(hit.file->link(*hit.entry)).lodBias;
        } catch (const std::exception&) {
        }
    }
    if (!std::isfinite(b)) b = 0.0f;
    return textureBias_[bitmapId] = b;
}
// ---- PK28 end

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
    if (auto hit = library_.find(bitmapId, cls::Bitmap)) result = uploadBitmap(*hit.file, *hit.entry, srgb, &alpha), ++oneOff_.textures;  // P92
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
        // Draw state from the flags (core/material_anim.hpp materialDrawState: UpdateFlags 0x695f70, MeshPc_Z::Draw
        // 0x848880). The cubelighting path (bCubemap) ignores the material colour.
        const MaterialDrawState ds = materialDrawState(m, alphaKind);
        const bool unlit = (m.flags & 0x3000) != 0;
        if (ds.cubemap) {
            gm.features |= Reflective;
            gm.diffuse = simd_make_float4(1, 1, 1, m.alpha);
        }
        // UV transform: the stored matrix (UpdateUVMatrix runs only when a MaterialAnim_Z drives it).
        gm.uv0 = simd_make_float4(m.uvMatrix[0], m.uvMatrix[1], m.uvMatrix[2], m.uvMatrix[3]);
        gm.uv1 = simd_make_float2(m.uvMatrix[4], m.uvMatrix[5]);
        gm.alpha = AlphaMode(uint32_t(ds.mode));
        gm.alphaRef = ds.alphaRef;
        gm.twoSided = ds.twoSided;
        gm.fxAdditive = alphaKind == 0 && m.alpha >= 1.0f && (m.flags & 0x180000) == 0;
        gm.flags = m.flags;
        if (gm.twoSided) gm.features |= TwoSided;
        static const bool matDebug = std::getenv("FUEL_MAT_DEBUG") != nullptr;
        if (matDebug)
            std::printf("material %08x code %u flags %08x alpha %.3f alphaKind %u -> mode %u twoSided %d lit %d reflective %d tex0 %08x\n",
                        materialId, gm.code, m.flags, m.alpha, alphaKind, unsigned(gm.alpha), gm.twoSided ? 1 : 0, unlit ? 0 : 1,
                        (gm.features & Reflective) ? 1 : 0, diffuseTex);
        scanMaterialAnims();
        if (auto it = animByMaterial_.find(materialId); it != animByMaterial_.end()) gm.anim = it->second;
        // (P28: the port's old lift of dark green cut-outs is gone: in FUEL's gamma-space lighting the leaf
        // textures are read as stored and lit by the strong sky light, as in the game.)
        gm.lit = (m.flags & 0x3000) == 0;
        gm.skip = m.skipDraw();
        gm.lamp = fuel::vlinks::lampOfMaterial(name);  // PK22: a lamp group's material (0x69da00)
        gm.nameCrc = name;  // [upgrades U5]
        gm.samplerIndex = (m.clampU() ? 1u : 0u) | (m.clampV() ? 2u : 0u);
        // ---- PK28: each slot's own LOD bias (0x84ca80; flip-book frames keep slot 0's, see notes pk28.md)
        gm.lodBias = simd_make_float4(bitmapLodBias(diffuseTex), bitmapLodBias(m.textures[kOcclusion]), bitmapLodBias(m.textures[kNormal]),
                                      bitmapLodBias(m.textures[kSpecular]));
        gm.lodBias2 = simd_make_float4(bitmapLodBias(m.textures[kNormalLocal]), bitmapLodBias(m.textures[kDirt]), 0, 0);
    } catch (const std::exception&) {
    }
    return gm;
}

// ---- PHYSIQUE (port deviation): the custom rider's materials. DDS (DXT1 / DXT5 / BGRA8 with their mip chains), read as
// stored like FUEL's bitmaps (gamma space, no sRGB decode); normal maps in FUEL's layout (x in alpha, y in green).
id<MTLTexture> MeshRenderer::ddsTexture(const std::string& path, bool* hasAlpha) {
    if (hasAlpha) *hasAlpha = false;
    if (path.empty()) return nil;
    if (auto it = ddsTextures_.find(path); it != ddsTextures_.end()) {
        if (hasAlpha) *hasAlpha = it->second != nil && it->second.pixelFormat != MTLPixelFormatBC1_RGBA;
        return it->second;
    }
    id<MTLTexture> t = nil;
    std::ifstream f(path, std::ios::binary);
    std::vector<uint8_t> b((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    uint32_t h[32] = {};
    if (b.size() > 128) std::memcpy(h, b.data(), 128);
    if (h[0] == 0x20534444 && h[1] == 124) {
        const uint32_t height = h[3], width = h[4], mips = std::max(1u, h[7]), fourcc = h[21], bits = h[22];
        MTLPixelFormat pf = MTLPixelFormatInvalid;
        uint32_t block = 0;
        if (fourcc == 0x31545844) pf = MTLPixelFormatBC1_RGBA, block = 8;
        else if (fourcc == 0x35545844) pf = MTLPixelFormatBC3_RGBA, block = 16;
        else if (fourcc == 0 && bits == 32) pf = MTLPixelFormatBGRA8Unorm;
        if (pf != MTLPixelFormatInvalid && width > 0 && height > 0 && width <= 8192 && height <= 8192) {
            MTLTextureDescriptor* td = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:pf width:width height:height mipmapped:mips > 1];
            td.mipmapLevelCount = mips;
            t = [device_ newTextureWithDescriptor:td];
            size_t off = 128;
            uint32_t w = width, hh = height;
            for (uint32_t level = 0; level < mips && t != nil; ++level) {
                const size_t row = block ? size_t(std::max(1u, (w + 3) / 4)) * block : size_t(w) * 4;
                const size_t size = block ? row * std::max(1u, (hh + 3) / 4) : row * hh;
                if (off + size > b.size()) {
                    t = nil;
                    break;
                }
                [t replaceRegion:MTLRegionMake2D(0, 0, w, hh) mipmapLevel:level withBytes:b.data() + off bytesPerRow:row];
                off += size;
                w = std::max(1u, w >> 1), hh = std::max(1u, hh >> 1);
            }
            if (t != nil) toOptimize_.push_back(t);
        }
    }
    if (t == nil) std::fprintf(stderr, "renderer: custom rider texture %s does not load\n", path.c_str());
    ddsTextures_[path] = t;
    if (hasAlpha) *hasAlpha = t != nil && t.pixelFormat != MTLPixelFormatBC1_RGBA;
    return t;
}

MeshRenderer::GpuMaterial MeshRenderer::makeCustomMaterial(const CustomMaterial& c) {
    GpuMaterial gm;
    gm.textures.fill(white_);
    if (id<MTLTexture> t = ddsTexture(c.diffuse)) gm.textures[kDiffuse] = t;
    if (id<MTLTexture> t = ddsTexture(c.normal)) gm.textures[kNormal] = t, gm.features |= HasNormal;
    if (id<MTLTexture> t = ddsTexture(c.specular)) gm.textures[kSpecular] = t, gm.features |= HasSpecular;
    gm.diffuse = simd_make_float4(1, 1, 1, 1);
    gm.specular = simd_make_float4(c.spec[0], c.spec[1], c.spec[2], c.spec[3]);
    gm.alpha = (c.flags & CustomMaterial::AlphaTest) ? AlphaMode::Test : AlphaMode::Opaque;
    gm.twoSided = (c.flags & CustomMaterial::TwoSided) != 0;
    if (gm.twoSided) gm.features |= TwoSided;
    if (c.flags & CustomMaterial::Skin) gm.features |= SkinShade;
    if (c.flags & CustomMaterial::Hair) gm.features |= HairShade;
    if (c.flags & CustomMaterial::Eye) gm.features |= EyeShade;
    // ---- [upgrades U5] the procedural parts' materials (core/upgrade_visuals): a solid colour, chrome (FUEL's cube path) or
    // the vehicle's paint (code 19 with the diffuse alpha 0)
    if (c.flags & CustomMaterial::Solid) gm.textures[kDiffuse] = solidTexture(c.rgba);
    if (c.flags & CustomMaterial::Chrome) gm.features |= Reflective, gm.fresnel = simd_make_float2(c.fresnel[0], c.fresnel[1]);
    if (c.flags & CustomMaterial::Paint) gm.code = 19;
    if (c.flags & CustomMaterial::Carbon) gm.features |= Carbon, gm.lodBias2.z = 3.0f;
    return gm;
}

id<MTLTexture> MeshRenderer::solidTexture(const std::array<float, 4>& rgba) {
    uint8_t px[4];
    for (int i = 0; i < 4; ++i) px[i] = uint8_t(std::lround(std::clamp(rgba[size_t(i)], 0.0f, 1.0f) * 255.0f));
    uint32_t key;
    std::memcpy(&key, px, 4);
    if (auto it = solidTextures_.find(key); it != solidTextures_.end()) return it->second;
    MTLTextureDescriptor* td = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatRGBA8Unorm width:4 height:4 mipmapped:NO];
    id<MTLTexture> t = [device_ newTextureWithDescriptor:td];
    uint8_t all[64];
    for (int i = 0; i < 16; ++i) std::memcpy(all + i * 4, px, 4);
    [t replaceRegion:MTLRegionMake2D(0, 0, 4, 4) mipmapLevel:0 withBytes:all bytesPerRow:16];
    solidTextures_[key] = t;
    return t;
}

// [upgrades U5] A procedural mesh (added as a baked mesh) or a derived one (a copy of the base without the hidden batches,
// the paint with the carbon weave). `handled` = the key is one of the upgrades' (the result can still be null on failure).
const MeshRenderer::GpuMesh* MeshRenderer::loadUpgradeMesh(uint32_t meshId, std::string* error, const std::array<uint32_t, 3>* livery,
                                                           bool* handled) {
    *handled = false;
    if (std::shared_ptr<const BakedMesh> pm = fuel::upgvis::proceduralMesh(meshId)) {
        *handled = true;
        if (!addBakedMesh(meshId, *pm, error)) return nullptr;
        const GpuMesh* gm = bakedMeshes_[meshId].get();
        for (int pass = 0; pass < 3; ++pass)  // its pipelines now, not in a race frame
            for (const Draw& d : gm->draws[pass]) (void)pipeline(gm->vbStrides[d.vb], AlphaMode(pass), vsVariant(gm->materials[d.material], d.sprite));
        return gm;
    }
    const fuel::upgvis::Derived* dm = fuel::upgvis::derivedMesh(meshId);
    if (dm == nullptr) return nullptr;
    *handled = true;
    const GpuMesh* base = livery ? loadMesh(dm->base, error, *livery) : loadMesh(dm->base, error);
    if (base == nullptr) return nullptr;
    std::unique_ptr<GpuMesh> gm = deriveMesh(*base, meshId);
    const GpuMesh* out = gm.get();
    if (livery) liveryMeshes_[std::make_pair(meshId, *livery)] = std::move(gm);
    else meshes_[meshId] = std::move(gm);
    return out;
}

std::unique_ptr<MeshRenderer::GpuMesh> MeshRenderer::deriveMesh(const GpuMesh& base, uint32_t key) {
    const fuel::upgvis::Derived& dm = *fuel::upgvis::derivedMesh(key);
    auto gm = std::make_unique<GpuMesh>(base);
    gm->triangles = 0;
    for (auto& list : gm->draws) {
        std::vector<Draw> kept;
        for (const Draw& d : list)
            if (std::find(dm.hiddenMaterials.begin(), dm.hiddenMaterials.end(), gm->materials[d.material].nameCrc) == dm.hiddenMaterials.end())
                kept.push_back(d), gm->triangles += d.indexCount / 3;
        list = std::move(kept);
    }
    if (dm.carbon > 0)
        for (GpuMaterial& m : gm->materials)
            if (m.code == 17 || m.code == 19 ||
                (m.nameCrc != 0 && std::find(dm.carbonMaterials.begin(), dm.carbonMaterials.end(), m.nameCrc) != dm.carbonMaterials.end())) {
                m.features |= Carbon;
                m.lodBias2.z = dm.carbonParams[0], m.lodBias2.w = dm.carbonParams[1];
            }
    if (std::getenv("FUEL_U5_TRACE")) {
        int c = 0;
        for (const GpuMaterial& m : gm->materials) c += (m.features & Carbon) ? 1 : 0;
        std::printf("u5: derived mesh %08x from %08x: %u triangles, %d carbon materials, %zu hidden names\n", key, dm.base, gm->triangles, c, dm.hiddenMaterials.size());
    }
    for (int pass = 0; pass < 3; ++pass)  // the carbon pipelines now (made synchronously: no stand-in), not in a race frame
        for (const Draw& d : gm->draws[pass])
            if (gm->materials[d.material].features & Carbon) (void)pipeline(gm->vbStrides[d.vb], AlphaMode(pass), vsVariant(gm->materials[d.material], d.sprite));
    return gm;
}
// ---- PHYSIQUE end

void MeshRenderer::setPaint(const Paint& paint) {
    // Materials bake in the livery texture, so setScene loads the vehicle's parts through the
    // (mesh, livery) cache. meshes_ is never cleared here: world objects, LOD groups, props and
    // cut-scene objects keep pointers into it (clearing it on a vehicle switch left them dangling).
    paint_ = paint;
}

const MeshRenderer::GpuMesh* MeshRenderer::loadMesh(uint32_t meshId, std::string* error) {
    if (auto it = bakedMeshes_.find(meshId); it != bakedMeshes_.end()) return it->second.get();
    if (auto it = meshes_.find(meshId); it != meshes_.end()) return it->second.get();
    {  // [upgrades U5]
        bool handled = false;
        const GpuMesh* u = loadUpgradeMesh(meshId, error, nullptr, &handled);
        if (handled) return u;
    }
    if (library_.find(meshId, cls::Surface)) {  // P73: a Surface_Z's Bezier patches, baked once (core/surface_patch.hpp)
        std::vector<const DpcFile*> lookup;
        for (const auto& f : library_.files()) lookup.push_back(f.get());
        BakedMesh b;
        if (bakeSurface(lookup, meshId, b, error) && addBakedMesh(meshId, b, error)) return bakedMeshes_[meshId].get();
        meshes_[meshId] = nullptr;  // do not retry every frame
        return nullptr;
    }
    std::unique_ptr<GpuMesh> gm = buildMesh(meshId, error);
    return gm ? (meshes_[meshId] = std::move(gm)).get() : nullptr;
}

const MeshRenderer::GpuMesh* MeshRenderer::loadMesh(uint32_t meshId, std::string* error, const std::array<uint32_t, 3>& livery) {
    if (auto it = bakedMeshes_.find(meshId); it != bakedMeshes_.end()) return it->second.get();
    const auto key = std::make_pair(meshId, livery);
    if (auto it = liveryMeshes_.find(key); it != liveryMeshes_.end()) return it->second.get();
    {  // [upgrades U5]
        bool handled = false;
        const GpuMesh* u = loadUpgradeMesh(meshId, error, &livery, &handled);
        if (handled) return u;
    }
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
    ++oneOff_.meshes;  // P92: a mesh built at first use
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
        // P74: CodeId 2 / 3 / 4 / 5 / 6 (1- and 4-blend, volume, morph) draw with scat_skin.vhl, whose omni is 0.
        d.skinned = b.vertexType >= 2 && b.vertexType <= 6;
        d.morph = b.vertexType == 8;  // P85: the damage morph's batches
        // Sprite batches (leaves, impostors): vertex type 7 whose triangle corners share a position.
        if ((b.vertexType == 7 || mesh.vbs[b.vb].count <= 8) && tris.size() >= 3) {
            const VertexBuffer& vb = mesh.vbs[b.vb];
            d.sprite = std::memcmp(&vb.data[size_t(tris[0]) * vb.stride], &vb.data[size_t(tris[1]) * vb.stride], 12) == 0;
            if (d.sprite && vb.stride >= 36)  // P92: how far vsMain moves a sprite corner (the corner attribute at +28)
                for (uint32_t t : tris) {
                    float corner[2];
                    std::memcpy(corner, &vb.data[size_t(t) * vb.stride + 28], 8);
                    gm->spriteReach = std::max(gm->spriteReach, std::hypot(corner[0], corner[1]));
                }
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
    for (const Draw& d : gm->draws[uint32_t(AlphaMode::Blend)]) {  // ---- PK28: the 0x856160 blend pipelines it will need
        const GpuMaterial& m = gm->materials[d.material];
        (void)blendPipeline(gm->vbStrides[d.vb], m.flags, vsVariant(m, d.sprite), true);
    }  // ---- PK28 end
    return gm;
}

bool MeshRenderer::addBakedMesh(uint32_t key, const BakedMesh& mesh, std::string* error) {
    bakedRing_.erase(key);  // P92: a new mesh under the key starts a new ring
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
    for (const BakedMaterial& m : mesh.materials)  // PHYSIQUE: custom rider materials from their own textures
        gm->materials.push_back(m.custom ? makeCustomMaterial(*m.custom) : makeMaterial(m.materialId, m.diffuseTex, m.specularTex));
    for (const BakedBatch& b : mesh.batches) {
        const uint32_t mat = b.material + 1;
        if (gm->materials[mat].skip) continue;
        gm->draws[uint32_t(gm->materials[mat].alpha)].push_back({0, b.firstIndex, b.indexCount, mat, false, true});  // P74: skinned
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

void MeshRenderer::setBakedMaterial(uint32_t key, size_t slot, const Material& m) {  // PK29
    auto it = bakedMeshes_.find(key);
    if (it == bakedMeshes_.end() || slot + 1 >= it->second->materials.size()) return;
    GpuMaterial& g = it->second->materials[slot + 1];  // [0] = no material
    if (!(g.features & Reflective)) g.diffuse = simd_make_float4(m.diffuse[0], m.diffuse[1], m.diffuse[2], g.diffuse.w);
    g.emissive = simd_make_float4(m.emissive[0], m.emissive[1], m.emissive[2], 0);
    g.uv0 = simd_make_float4(m.uvMatrix[0], m.uvMatrix[1], m.uvMatrix[2], m.uvMatrix[3]);
    g.uv1 = simd_make_float2(m.uvMatrix[4], m.uvMatrix[5]);
}

void MeshRenderer::updateBakedVertices(uint32_t key, const std::vector<uint8_t>& vertices) {
    auto it = bakedMeshes_.find(key);
    if (it == bakedMeshes_.end() || vertices.empty() || it->second->vbs.empty()) return;
    GpuMesh& gm = *it->second;
    if (gm.vbs[0].length != vertices.size()) return;
    // A buffer the frames in flight do not read: P92: from a ring per mesh (was a fresh buffer each frame, whose new
    // pages the GPU had to map at its first use). A slot comes back kBakedRing updates later: at most one update per
    // mesh per drawn frame, and at most three frames in flight (MTKView's drawables, --perf-check), so the GPU is done.
    auto& ring = bakedRing_[key];
    if (ring.buffers.empty()) {
        ring.buffers.push_back(gm.vbs[0]);  // the buffer addBakedMesh made joins the ring
        for (size_t k = 1; k < kBakedRing; ++k)
            ring.buffers.push_back([device_ newBufferWithLength:vertices.size() options:MTLResourceStorageModeShared]);
    }
    ring.next = (ring.next + 1) % ring.buffers.size();
    id<MTLBuffer> b = ring.buffers[ring.next];
    std::memcpy(b.contents, vertices.data(), vertices.size());
    gm.vbs[0] = b;
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
    ++morphEpoch_;  // P85: the damage morphs went with the instances
    if (!visuals_.empty()) visuals_[0] = VehicleVisual{};  // PK22: the player's links went with them
    inputToInstance_.assign(instances.size(), -1);
    triangles_ = 0;
    error->clear();
    simd_float3 lo = simd_make_float3(1e30f, 1e30f, 1e30f), hi = -lo;
    for (size_t k = 0; k < instances.size(); ++k) {
        const MeshInstance& in = instances[k];
        std::string err;
        const GpuMesh* gm = loadMesh(in.meshId, &err, paint_.livery);
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
    worldCull_.clear();
    for (const auto& [gm, worlds] : byMesh) {
        worldGroups_.push_back({gm, [device_ newBufferWithBytes:worlds.data() length:worlds.size() * sizeof(simd_float4x4)
                                                        options:MTLResourceStorageModeShared],
                                uint32_t(worlds.size())});
        worldCull_.push_back(cullGroupOf(gm, worlds));  // P95: in the groups' order
    }
    return worldInstances_.size();
}

size_t MeshRenderer::setLodObjects(const std::vector<PlacedMesh>& objects, int set) {
    lodIndexDirty_[size_t(std::clamp(set, 0, int(kLodSets) - 1))] = true;  // P92
    std::vector<LodObject>& list = lodObjects_[size_t(std::clamp(set, 0, int(kLodSets) - 1))];
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
bool MeshRenderer::bakedMeshDrawable(uint32_t key, const OrbitCamera& cam) {
    if (!cam.explicitPose) return true;
    auto bit = bakedMeshes_.find(key);
    if (bit == bakedMeshes_.end()) return true;
    const GpuMesh* gm = bit->second.get();
    for (const auto* list : {&propInputs_, &rtcInputs_})  // props and cut-scene objects: drawn as they are
        for (const MeshInstance& m : *list)
            if (m.meshId == key) return true;
    for (size_t set = 0; set < kLodSets; ++set) {
        auto& index = lodIndex_[set];
        if (lodIndexDirty_[set]) {
            index.clear();
            for (uint32_t i = 0; i < lodObjects_[set].size(); ++i)
                for (const GpuMesh* l : lodObjects_[set][i].levels)
                    if (l != nullptr) {
                        auto& v = index[l];
                        if (v.empty() || v.back() != i) v.push_back(i);
                    }
            lodIndexDirty_[set] = false;
        }
        auto it = index.find(gm);
        if (it == index.end()) continue;
        for (uint32_t i : it->second) {  // evaluateLodObjects's test for this object
            const LodObject& o = lodObjects_[set][i];
            if (set == 0 && i < lodHidden_.size() && lodHidden_[i]) continue;
            // At the camera's distance and 2 m either side: the window shakes the camera (P67) after this is asked.
            const float d0 = simd_distance(o.world.columns[3].xyz, cam.eye);
            for (const float d : {d0, std::max(0.0f, d0 - 2.0f), d0 + 2.0f}) {
                const int level = lodLevel(o.p, d, lodView);
                if (level >= int(o.levels.size()) || o.levels[size_t(level)] != gm) continue;
                if (placementAlpha(o.p, level, d, lodView) >= 0.01f) return true;
            }
        }
    }
    return false;
}

// ---- P94: one object's Lod_Z::Draw decision (0x884170: level, ComputeFade alpha >= 0.01), shared by the draw
// (evaluateLodObjects) and the pop probe.
bool MeshRenderer::lodDecide(size_t set, size_t i, simd_float3 eye, const GpuMesh*& gm, float& alpha) const {
    const LodObject& o = lodObjects_[set][i];
    if (set == 0 && i < lodHidden_.size() && lodHidden_[i]) return false;
    // AutoMesh / RoadMesh spawns: FUEL holds them while the camera's decor leaf over their cell generates them (the
    // worker's list is a superset around the player, core/automesh.hpp autoCellHolds).
    if (set == 1 && o.p.autoCell.on && !fuel::autoCellHolds(o.p.autoCell, eye.x, eye.z)) return false;
    const simd_float3 t = o.world.columns[3].xyz;  // node world T (node+0xd4 row 3)
    const float d = simd_distance(t, eye);
    const int level = lodLevel(o.p, d, lodView);
    gm = level < int(o.levels.size()) ? o.levels[size_t(level)] : nullptr;
    alpha = placementAlpha(o.p, level, d, lodView);
    return gm != nullptr && alpha >= 0.01f;
}

// PK15: 0x57f050 (the generated ground is settled).
bool MeshRenderer::terrainIdle() const { return !terrainLod_ || terrainLod_->idle(); }

void MeshRenderer::popProbe(simd_float3 eye, std::vector<PopSample>& out) const {
    out.clear();
    auto radiusOf = [](const GpuMesh* gm, const simd_float4x4& w) {
        const float sc = std::max(simd_length(w.columns[0].xyz), std::max(simd_length(w.columns[1].xyz), simd_length(w.columns[2].xyz)));
        return simd_length(gm->boundsMax - gm->boundsMin) * 0.5f * sc + gm->spriteReach * simd_length(w.columns[1].xyz);
    };
    static constexpr PopSample::Kind kSetKind[kLodSets] = {PopSample::Placed, PopSample::AutoMesh, PopSample::Marker, PopSample::Prop};
    for (size_t set = 0; set < kLodSets; ++set)
        for (size_t i = 0; i < lodObjects_[set].size(); ++i) {
            const LodObject& o = lodObjects_[set][i];
            const GpuMesh* gm = nullptr;
            float a = 0;
            if (!lodDecide(set, i, eye, gm, a)) {
                if (o.levels.empty() || o.levels[0] == nullptr) continue;
                // FUEL's own rules hold it back: its cell (AutoMesh), or a Lod level without a drawable mesh. Reported
                // with alpha 0, so that the change is told apart.
                const bool gated = set == 1 && o.p.autoCell.on && !fuel::autoCellHolds(o.p.autoCell, eye.x, eye.z);
                const bool hidden = set == 0 && i < lodHidden_.size() && lodHidden_[i];
                if (!gated && (hidden || gm != nullptr || !(a >= 0.01f))) continue;
                out.push_back({kSetKind[set], o.levels[0], gated ? o.levels[0] : nullptr, o.world.columns[3].xyz, 0.0f, radiusOf(o.levels[0], o.world),
                               o.p.objectId, o.p.viewDist, gated, true});
                continue;
            }
            out.push_back({kSetKind[set], o.levels.empty() || o.levels[0] == nullptr ? static_cast<const void*>(gm) : o.levels[0], gm, o.world.columns[3].xyz, a, radiusOf(gm, o.world), o.p.objectId, o.p.lod ? o.p.viewDist : (o.p.levelView.empty() ? 0.0f : o.p.levelView[0]), false, true});
        }
    for (const Instance& inst : propInstances_)  // the plain prop list (and cut-scene objects): drawn as they are
        out.push_back({PopSample::Prop, inst.mesh, inst.mesh, inst.world.columns[3].xyz, 1.0f, radiusOf(inst.mesh, inst.world)});
    for (const Instance& inst : worldInstances_) {  // plants: their decor kind, else a lone sprite quad is a card
        const auto kind = fuel::DecorFade(uint8_t(inst.world.columns[1].w));
        const bool card = kind != fuel::DecorFade::None ? kind != fuel::DecorFade::Mesh : inst.mesh->triangles <= 2 && inst.mesh->spriteReach > 0;
        const simd_float3 p = inst.world.columns[3].xyz;
        const float a = fuel::decorAtt(kind, p.x, p.z, eye.x, eye.z, simd_distance(p.xz, eye.xz));
        simd_float4x4 w = inst.world;
        w.columns[1].w = 0;
        // FUEL's alpha test at 0.5 on att x the texture alpha: texels thin out as att falls from 1 to 0.5 (the probe's
        // alpha: the share of opaque-to-half-alpha texels left, 2 att - 1).
        out.push_back({card ? PopSample::PlantCard : PopSample::PlantMesh, nullptr, inst.mesh, p, std::clamp(2.0f * a - 1.0f, 0.0f, 1.0f), radiusOf(inst.mesh, w), 0,
                       kind != fuel::DecorFade::None ? float(fuel::decorLeafSize(p.x, p.z, eye.x, eye.z)) : 0.0f});  // view = the leaf
    }
    // ---- P104: every LOD-switched vehicle and its rider, as setVehicleLod left them (one sample each; the level is
    // Lod_Z::Draw's, so a level change is FUEL's own switch).
    for (const OtherVehicle& ov : vehicles_) {
        if (ov.lod.level < 0) continue;
        float r = 1.0f;
        for (size_t i = 0; i < ov.instances.size(); ++i)
            if (ov.drawn(i)) {
                const Instance& inst = ov.instances[i];
                const simd_float3 c = simd_mul(inst.world, simd_make_float4((inst.mesh->boundsMin + inst.mesh->boundsMax) * 0.5f, 1)).xyz;
                r = std::max(r, simd_distance(c, ov.lod.anchor) + radiusOf(inst.mesh, inst.world));
            }
        const auto tag = [](uintptr_t base, int level) { return reinterpret_cast<const void*>(base + uintptr_t(level)); };
        out.push_back({PopSample::Vehicle, &ov, tag(0x1000, ov.lod.level), ov.lod.anchor, ov.lod.alpha, std::min(r, 6.0f), 0, 0, false, true});
        if (ov.lod.riderLevel >= 0)
            out.push_back({PopSample::Vehicle, reinterpret_cast<const char*>(&ov) + 1, tag(0x2000, ov.lod.riderLevel), ov.lod.riderAnchor,
                           ov.lod.riderAlpha, 1.0f, 0, 0, false, true});
    }
    // ---- P104 end
}
// ---- P94 end

void MeshRenderer::evaluateLodObjects(simd_float3 eye, const ViewCull& cull) {
    const auto t0 = std::chrono::steady_clock::now();
    lodVisible_.clear();
    lodGroups_.clear();
    lodFadingObjs_.clear();
    lodDrawn_ = lodFading_ = 0;
    for (size_t set = 0; set < kLodSets; ++set) lodSetDrawn_[set] = 0;
    if (std::all_of(lodObjects_.begin(), lodObjects_.end(), [](const auto& l) { return l.empty(); })) return;
    std::unordered_map<const GpuMesh*, std::vector<simd_float4x4>> byMesh;
    for (size_t set = 0; set < kLodSets; ++set)
    for (size_t i = 0; i < lodObjects_[set].size(); ++i) {
        const LodObject& o = lodObjects_[set][i];
        const GpuMesh* gm = nullptr;
        float a = 0;
        if (!lodDecide(set, i, eye, gm, a)) continue;  // P94: shared with popProbe
        simd_float4x4 w = o.world;
        w.columns[0].w = a >= 1.0f ? 0.0f : 1.0f - a;
        lodFading_ += a < 1.0f;
        lodVisible_.push_back({gm, w});  // (shadow casters: every object in range)
        lodSetDrawn_[set]++;
        // P92: objects wholly off screen go to the shadows only: the mesh's box as a sphere (with the leaf / impostor
        // sprites' corner reach), through the instance's scale. The mesh's group is still made (maybe empty), so the
        // groups come out of byMesh in the order they did without culling.
        bool off = false;
        if (cull.on) {
            const simd_float3 c = simd_mul(o.world, simd_make_float4((gm->boundsMin + gm->boundsMax) * 0.5f, 1)).xyz;
            const float sc = std::max(simd_length(o.world.columns[0].xyz), std::max(simd_length(o.world.columns[1].xyz), simd_length(o.world.columns[2].xyz)));
            const float r = simd_length(gm->boundsMax - gm->boundsMin) * 0.5f * sc + gm->spriteReach * simd_length(o.world.columns[1].xyz);
            off = cull.sphereOutside(c, r);
        }
        static const bool ditherFade = std::getenv("FUEL_FADE_DITHER") != nullptr;  // the pre-P72 dither, to compare
        if (a < 1.0f && !ditherFade) {
            if (!off) lodFadingObjs_.push_back({gm, w});  // P72: fading (0x84ec60 alpha < 1) -> the sorted blended phase
        } else {
            auto& group = byMesh[gm];
            if (!off) group.push_back(w);
        }
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
        if (worlds.empty()) continue;  // P92: every instance off screen
        std::memcpy(dst + at, worlds.data(), worlds.size() * sizeof(simd_float4x4));
        lodGroups_.push_back({gm, buf, uint32_t(worlds.size()), uint32_t(at * sizeof(simd_float4x4))});
        at += worlds.size();
    }
}

size_t MeshRenderer::setPropInstances(const std::vector<MeshInstance>& instances) {
    propInputs_ = instances;
    return rebuildProps();
}

size_t MeshRenderer::setRtcInstances(const std::vector<MeshInstance>& instances) {
    if (instances.empty() && rtcInputs_.empty()) return propInstances_.size();
    rtcInputs_ = instances;
    return rebuildProps();
}

size_t MeshRenderer::setHeliInstances(const std::vector<MeshInstance>& instances) {  // PK07
    if (instances.empty() && heliInputs_.empty()) return propInstances_.size();
    heliInputs_ = instances;
    return rebuildProps();
}

// ---- PK27: the F14 fly-by (f14_view.mm)
size_t MeshRenderer::setF14Instances(const std::vector<MeshInstance>& instances) {
    if (instances.empty() && f14Inputs_.empty()) return propInstances_.size();
    f14Inputs_ = instances;
    return rebuildProps();
}
// ---- PK27 end

size_t MeshRenderer::rebuildProps() {
    propInstances_.clear();
    std::unordered_map<const GpuMesh*, std::vector<simd_float4x4>> byMesh;
    std::vector<MeshInstance> instances = propInputs_;
    instances.insert(instances.end(), rtcInputs_.begin(), rtcInputs_.end());
    instances.insert(instances.end(), heliInputs_.begin(), heliInputs_.end());  // PK07
    instances.insert(instances.end(), f14Inputs_.begin(), f14Inputs_.end());    // PK27
    for (const MeshInstance& in : instances) {
        std::string err;
        const GpuMesh* gm = loadMesh(in.meshId, &err);
        if (gm == nullptr || gm->indices == nil) continue;
        propInstances_.push_back({gm, in.world});
        byMesh[gm].push_back(in.world);
    }
    propGroups_.clear();
    propCull_.clear();
    for (const auto& [gm, worlds] : byMesh) {
        propGroups_.push_back({gm, [device_ newBufferWithBytes:worlds.data() length:worlds.size() * sizeof(simd_float4x4)
                                                       options:MTLResourceStorageModeShared],
                               uint32_t(worlds.size())});
        propCull_.push_back(cullGroupOf(gm, worlds));  // P95
    }
    return propInstances_.size();
}

// ---- P95: instances wholly off screen left out of the scene pass (P92 did it for the placed LOD objects). The sphere is
// P92's (the mesh box through the instance's largest axis scale, plus the sprite corner reach through its y scale) and
// ViewCull keeps a metre of margin, so a left-out instance has no pixel on screen: the same picture.
MeshRenderer::CullGroup MeshRenderer::cullGroupOf(const GpuMesh* gm, const std::vector<simd_float4x4>& worlds) {
    CullGroup g{gm, worlds, {}};
    g.spheres.reserve(worlds.size());
    const simd_float3 c0 = (gm->boundsMin + gm->boundsMax) * 0.5f;
    const float half = simd_length(gm->boundsMax - gm->boundsMin) * 0.5f;
    for (const simd_float4x4& w : worlds) {
        const simd_float3 c = simd_mul(w, simd_make_float4(c0, 1)).xyz;  // (the flag slots in the columns' w do not reach xyz)
        const float sc = std::max(simd_length(w.columns[0].xyz), std::max(simd_length(w.columns[1].xyz), simd_length(w.columns[2].xyz)));
        g.spheres.push_back(simd_make_float4(c, half * sc + gm->spriteReach * simd_length(w.columns[1].xyz)));
    }
    return g;
}

void MeshRenderer::cullGroups(const std::vector<CullGroup>& in, std::vector<WorldGroup>& out, const ViewCull& cull) {
    out.clear();
    size_t total = 0;
    for (const CullGroup& g : in) total += g.worlds.size();
    if (total == 0) return;
    id<MTLBuffer> buf = [device_ newBufferWithLength:total * sizeof(simd_float4x4) options:MTLResourceStorageModeShared];
    auto* dst = static_cast<simd_float4x4*>(buf.contents);
    size_t at = 0;
    for (const CullGroup& g : in) {  // the groups' order, and inside a group the instances' order
        const size_t first = at;
        for (size_t i = 0; i < g.worlds.size(); ++i)
            if (!cull.sphereOutside(g.spheres[i].xyz, g.spheres[i].w)) dst[at++] = g.worlds[i];
        if (at > first) out.push_back({g.mesh, buf, uint32_t(at - first), uint32_t(first * sizeof(simd_float4x4))});
    }
}

bool MeshRenderer::vehicleOffScreen(size_t v, const ViewCull& cull) const {
    if ((p95Off & P95CullInstances) || !cull.on) return false;
    // Each part's box (through its scale) plus 3 m: the damage morph (P85) and the rider's skinned pose reach past the
    // undamaged / bind-pose box, by far less than that.
    for (size_t i = 0; i < vehicles_[v].instances.size(); ++i) {
        if (!vehicles_[v].drawn(i)) continue;  // P104: another LOD level
        const Instance& inst = vehicles_[v].instances[i];
        const GpuMesh& gm = *inst.mesh;
        const simd_float3 c = simd_mul(inst.world, simd_make_float4((gm.boundsMin + gm.boundsMax) * 0.5f, 1)).xyz;
        const float sc = std::max(simd_length(inst.world.columns[0].xyz), std::max(simd_length(inst.world.columns[1].xyz), simd_length(inst.world.columns[2].xyz)));
        const float r = simd_length(gm.boundsMax - gm.boundsMin) * 0.5f * sc + gm.spriteReach * simd_length(inst.world.columns[1].xyz) + 3.0f;
        if (!cull.sphereOutside(c, r)) return false;
    }
    return true;
}

// ---- P85: the damage morph VBs (vehicle_damage.hpp). A fresh buffer per change: frames in flight keep the old one.
void MeshRenderer::setPartMorph(int vehicle, size_t part, const std::vector<float>& deltas) {
    Instance* inst = nullptr;
    if (vehicle < 0) {
        if (part < inputToInstance_.size() && inputToInstance_[part] >= 0) inst = &instances_[size_t(inputToInstance_[part])];
    } else if (size_t(vehicle) < vehicles_.size()) {
        OtherVehicle& ov = vehicles_[size_t(vehicle)];
        if (part < ov.inputToInstance.size() && ov.inputToInstance[part] >= 0) inst = &ov.instances[size_t(ov.inputToInstance[part])];
    }
    if (inst == nullptr) return;
    inst->morph = deltas.empty() ? nil
                                 : [device_ newBufferWithBytes:deltas.data() length:deltas.size() * sizeof(float)
                                                       options:MTLResourceStorageModeShared];
}
// ---- P85 end

// ---- PK22: the vehicle visual links (renderer.h)
void MeshRenderer::setPartAdjust(int vehicle, size_t part, const simd_float4x4* m) {
    VehicleVisual& v = visual(vehicle);
    if (v.adjust.size() <= part) v.adjust.resize(part + 1, matrix_identity_float4x4), v.adjusted.resize(part + 1, 0);
    v.adjusted[part] = m != nullptr;
    if (m != nullptr) v.adjust[part] = *m;
}
void MeshRenderer::setVehicleLamps(int vehicle, const std::array<float, 4>& alpha) {
    VehicleVisual& v = visual(vehicle);
    v.lampAlpha = alpha;
    v.lampsSet = true;
    if (!v.emissiveFound) findLampEmissive(vehicle);
}
void MeshRenderer::setVehicleDirt(int vehicle, float dirt) { visual(vehicle).dirt = dirt; }
// 0x69da00: the clone of the first part (LOD record 0's link records in order) that has the group's material.
void MeshRenderer::findLampEmissive(int vehicle) {
    VehicleVisual& v = visual(vehicle);
    const std::vector<Instance>* list = nullptr;
    const std::vector<int>* map = nullptr;
    if (vehicle < 0) list = &instances_, map = &inputToInstance_;
    else if (size_t(vehicle) < vehicles_.size()) list = &vehicles_[size_t(vehicle)].instances, map = &vehicles_[size_t(vehicle)].inputToInstance;
    if (list == nullptr) return;
    std::array<bool, 4> found{};
    for (int k : *map) {
        if (k < 0 || size_t(k) >= list->size()) continue;
        const GpuMesh& gm = *(*list)[size_t(k)].mesh;
        for (const GpuMaterial& m : gm.materials)
            if (m.lamp >= 0 && !found[size_t(m.lamp)]) {
                found[size_t(m.lamp)] = true;
                const auto e = fuel::vlinks::lampEmissive({m.emissive.x, m.emissive.y, m.emissive.z});
                v.lampEmissive[size_t(m.lamp)] = simd_make_float3(e[0], e[1], e[2]);
            }
    }
    v.emissiveFound = true;
}
std::array<simd_float3, 4> MeshRenderer::vehicleLampEmissive(int vehicle) {
    if (!visual(vehicle).emissiveFound) findLampEmissive(vehicle);
    return visual(vehicle).lampEmissive;
}
namespace {
// World x the V:DYN factor, and the dirt in column 2's w (dirt + 1; 0 = the global value, vsMain).
simd_float4x4 pk22World(const simd_float4x4& w, const std::vector<simd_float4x4>& adjust, const std::vector<char>& adjusted, size_t k, float dirt) {
    simd_float4x4 out = k < adjusted.size() && adjusted[k] ? simd_mul(w, adjust[k]) : w;
    out.columns[2].w = dirt >= 0 ? dirt + 1.0f : 0.0f;
    return out;
}
}  // namespace
// ---- PK22 end

void MeshRenderer::setInstanceWorlds(const std::vector<simd_float4x4>& worlds) {
    const VehicleVisual& vv = visual(-1);  // PK22
    for (size_t k = 0; k < worlds.size() && k < inputToInstance_.size(); ++k)
        if (inputToInstance_[k] >= 0) instances_[size_t(inputToInstance_[k])].world = pk22World(worlds[k], vv.adjust, vv.adjusted, k, vv.dirt);
}

void MeshRenderer::setVehicles(const std::vector<std::vector<MeshInstance>>& parts, const std::vector<Paint>& paints) {
    vehicles_.clear();
    if (visuals_.size() > 1) visuals_.resize(1);  // PK22: the other vehicles' links go with them
    ++morphEpoch_;  // P85
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
        ov.alpha.assign(ov.instances.size(), 1.0f);  // P104
        ov.rider.assign(ov.instances.size(), 0);
        vehicles_.push_back(std::move(ov));
    }
}

// ---- P104: FUEL's vehicle LOD (setVehicleLod)
void MeshRenderer::setVehicleLod(size_t k, const std::vector<float>& partAlphas, const VehicleLodState& st, const std::vector<float>& lod1Alphas) {
    if (k >= vehicles_.size()) return;
    OtherVehicle& ov = vehicles_[k];
    ov.alphaLod.assign(ov.instances.size(), 1.0f), ov.alphaLod1.assign(ov.instances.size(), 1.0f);
    for (size_t i = 0; i < partAlphas.size() && i < ov.inputToInstance.size(); ++i)
        if (ov.inputToInstance[i] >= 0) ov.alphaLod[size_t(ov.inputToInstance[i])] = partAlphas[i];
    for (size_t i = 0; i < lod1Alphas.size() && i < ov.inputToInstance.size(); ++i)
        if (ov.inputToInstance[i] >= 0) ov.alphaLod1[size_t(ov.inputToInstance[i])] = lod1Alphas[i];
    ov.alpha = lod1Alphas.empty() || !vehicleLodOff ? ov.alphaLod : ov.alphaLod1;
    ov.lod = st;
}
// ---- P104 end

void MeshRenderer::setVehicleWorlds(size_t k, const std::vector<simd_float4x4>& worlds) {
    if (k >= vehicles_.size()) return;
    OtherVehicle& ov = vehicles_[k];
    const VehicleVisual& vv = visual(int(k));  // PK22
    for (size_t i = 0; i < worlds.size() && i < ov.inputToInstance.size(); ++i)
        if (ov.inputToInstance[i] >= 0) ov.instances[size_t(ov.inputToInstance[i])].world = pk22World(worlds[i], vv.adjust, vv.adjusted, i, vv.dirt);
}

simd_float4x4 MeshRenderer::sunViewProj() const {
    // Legacy single-map matrix (kept for FrameUniforms.sunViewProj; the shaders use the 4 slices).
    const simd_float3 up = std::fabs(sunDir_.y) > 0.99f ? simd_make_float3(0, 0, 1) : simd_make_float3(0, 1, 0);
    const float r = shadowHalfSize() / 1.05f;
    const simd_float3 eye = center_ + sunDir_ * (r * 3.0f);
    return simd_mul(orthoRH(r * 1.05f, r * 0.5f, r * 6.0f), lookAtRH(eye, center_, up));
}

double MeshRenderer::timeOfDay() const {
    // ---- PK17: 0x46abd0's World_SetTimeAndCloud(72000): the weather apply (the only other writer) does not run while
    // the pause menu holds the world, so the menu's 20:00 wins over ForceDayTime and the race's fixed time too.
    if (pauseSky_) return pause::kBackdropTimeOfDay;
    // ---- end PK17
    if (forcedTimeOfDay >= 0) return forcedTimeOfDay;  // ForceDayTime
    // ---- PK04: the race's fixed time of day (0x5b89e0 with +0x114: +0xe0 x 86400; ForceDayTime still wins)
    if (weatherOverrides_.time) return overrideTimeOfDay(weatherOverrides_);
    // WeatherManager_G 0x5b89e0: t = game clock (s) x PP371 + 28800 (08:00); the clock as SetDayClock left it.
    return dayClockSeconds() * double(clockScale) + 28800.0;
    // ---- end PK04
}

// ---- PK04: SetDayClock 0x46ab10 and the race weather (weather.hpp).
double MeshRenderer::elapsedClock() const {
    // PK17: the day clock does not run while the world is paused (LevelManipulator advances it only when not paused).
    return (pausedAt_ >= 0 ? pausedAt_ : rawElapsed()) - pausedTotal_;
}

double MeshRenderer::rawElapsed() const {
    if (gameClock >= 0) return gameClock;
    if (frame_ == 0) return 0.0;  // the running clock starts with the first frame (epoch_)
    return std::chrono::duration<double>(std::chrono::steady_clock::now() - epoch_).count();
}

float MeshRenderer::tintLerpA() const { return tintOverride.w >= 0 ? tintOverride.w : postDefaults_.tint[3]; }  // PK17

// ---- PK17: the pause backdrop (renderer.h). 0x46abd0: +0x70 / +0x74 = the clock, SetDayClock(43200 / PP371) (450 s:
// 08:00 + 450 x 96 = 20:00), World_SetTimeAndCloud(72000, 0); 0x46ade0: the clock bytes put back. The weather hook's
// parameters of the last running frame stay (0x6409d0 runs only from the weather apply).
void MeshRenderer::setPauseBackdrop(bool on) {
    taaReset_ = true;  // TAA: the backdrop is a cut (another sky, the world stopped)
    if (on == pauseSky_) return;
    if (on) {
        pauseSavedClock_ = dayClockSeconds();
        pausedAt_ = rawElapsed();
        animPausedAt_ = animSeconds();
        setDayClock(pause::backdropClock(clockScale));
        pauseSky_ = true;
    } else {
        pauseSky_ = false;
        pausedTotal_ += rawElapsed() - pausedAt_;
        pausedAt_ = -1;
        const float a = animSeconds();  // still the frozen value (animPausedAt_ >= 0)
        animPausedAt_ = -1;
        animPausedTotal_ += animSeconds() - a;
        dayClockBase_ = pauseSavedClock_, dayClockAt_ = elapsedClock();  // the raw clock back, running from here
        haveAnimClock_ = false;  // material anims: no catch-up step for the paused time
    }
    if (std::getenv("FUEL_ATMO_TRACE"))
        std::printf("atmo: pause backdrop %s: clock %.3f s, time of day %.0f s\n", on ? "on" : "off", dayClockSeconds(), timeOfDay());
}
// ---- end PK17

double MeshRenderer::dayClockSeconds() const { return dayClockBase_ + (elapsedClock() - dayClockAt_); }

void MeshRenderer::setDayClock(double t) {
    dayClockBase_ = double(setDayClockMs(t)) * 0.001;
    dayClockAt_ = elapsedClock();
    if (std::getenv("FUEL_ATMO_TRACE")) std::printf("atmo: SetDayClock(%.1f) -> clock %.3f s, time of day %.0f s\n", t, dayClockBase_, timeOfDay());
}

void MeshRenderer::beginRaceWeather(const std::string& missionId, const std::array<float, 3>& start, const std::array<float, 3>& finish,
                                    std::function<std::array<double, 3>()> player, uint32_t flags) {
    const std::optional<MissionWeather> m = gameRoot_.empty() ? std::nullopt : missionWeatherFor(gameRoot_, missionId);
    if (std::getenv("FUEL_ATMO_TRACE"))
        std::printf("atmo: race %s: %s\n", missionId.c_str(), m ? "AddMissionParam weather" : "no AddMissionParam line (overrides cleared)");
    beginRaceWeather(m.value_or(MissionWeather{}), start, finish, std::move(player), flags);
}

void MeshRenderer::beginRaceWeather(const MissionWeather& m, const std::array<float, 3>& start, const std::array<float, 3>& finish,
                                    std::function<std::array<double, 3>()> player, uint32_t flags) {
    if (std::getenv("FUEL_NO_RACE_WEATHER")) return;  // A/B captures: the pre-PK04 running clock and field weather
    cloudRepopulate_ = raceWeather_.begin(m, flags, weatherNow_, weatherOverrides_);
    raceStart_ = start, raceFinish_ = finish, racePlayer_ = std::move(player);
    raceWeatherOn_ = true;
    if (std::getenv("FUEL_ATMO_TRACE"))
        std::printf("atmo: race weather: time %d %.4f..%.4f cloud %d %.2f rain %d %.2f snow %d %.2f fog %d %.2f sand %d %.2f storm %d\n",
                    int(raceWeather_.timeOfDay()), raceWeather_.dayStart(), raceWeather_.dayEnd(), int(weatherOverrides_.cloud),
                    weatherOverrides_.cloudValue, int(weatherOverrides_.rain), weatherOverrides_.rainValue, int(weatherOverrides_.snow),
                    weatherOverrides_.snowValue, int(weatherOverrides_.fog), weatherOverrides_.fogValue, int(weatherOverrides_.sand),
                    weatherOverrides_.sandValue, int(weatherOverrides_.storm));
}

void MeshRenderer::endRaceWeather() {
    weatherOverrides_.clear();
    raceWeatherOn_ = false;
    racePlayer_ = nullptr;
}

void MeshRenderer::stepRaceWeather() {
    if (!raceWeatherOn_ || !racePlayer_) return;
    const float t = raceWeatherT(raceStart_, raceFinish_, racePlayer_());  // 0x632250 (EDI = the player)
    raceWeather_.step(t, weatherOverrides_);
}
// ---- end PK04

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
    if (!pauseSky_) stepRaceWeather();  // ---- PK04: 0x632250 before the weather read (race frame, then WeatherManager vf17)
    in.timeOfDay = timeOfDay();
    in.weather = weather;
    // ---- PK17: paused: the weather manager does not run (its hook's parameters stay); the sun gets 0x46abd0's cloud 0.
    if (pauseSky_) {
        in.sunCloud = pause::kBackdropSunCloud;
        if (havePauseHook_) in.weather = pauseHook_;
    } else
    // ---- end PK17
    // FUEL's weather at the camera (WeatherManager_G 0x5b89e0 -> apply 0x5bba30 -> hook 0x6409d0), unless --weather.
    if (weatherFromWorld && weatherFields_ && weatherFields_->valid()) {
        weatherNow_ = weatherFields_->sample(eye.x, eye.y, eye.z);
        // ---- PK04: 0x5b89e0's race overrides (cloud, rain, snow, fog, sand, storm); FUEL_NO_RACE_WEATHER=1 for A/B
        if (weatherOverrides_.any())
            applyOverrides(weatherNow_, weatherOverrides_, weatherFields_->P(5444), weatherFields_->P(5445));
        // ---- end PK04
        if (weatherOverride) weatherOverride(weatherNow_);  // P65: race weather
        // ---- PK12: 0x5bba30: the sand tank's fog floor (+0x120 with +0x11c, set by 0x5ecb20 last frame) raises the fog
        if (sandFogSet_) weatherNow_.fog = std::max(weatherNow_.fog, sandFog_), sandFogSet_ = false;
        // ---- PK12 end
        const Weather h = weatherHook(weatherNow_, eye.y, weatherFields_->P(5202), weatherFields_->P(5203));
        in.weather = h;
    }
    if (!pauseSky_) pauseHook_ = in.weather, havePauseHook_ = true;  // PK17
    in.brightness = brightness;
    in.camera = {eye.x, eye.y, eye.z};
    // ---- P105: W+0xbc (0x77a240), the lowest coarse ground around the camera -> haze layer P7, cloud P10 / P11.
    // FUEL samples it in every GenWorld frame; without a loaded world the samples are 0 (P7 = 1000).
    if (terrainLowestGround) {
        auto h = [](double x, double z, const void* ctx) {
            const Heightfield& t = *static_cast<const Heightfield*>(ctx);
            return t.raw.empty() ? 0.0f : t.baseHeightAt(x, z);
        };
        in.lowestGround = lowestGroundNear(eye.x, eye.z, h, &terrainData_);
    }
    // ---- end P105
    in.legacyLiftedSun = std::getenv("FUEL_DUSK_OLD_SUN") != nullptr;  // DUSK: A/B captures only
    atmo_ = computeAtmosphere(in);
    const Atmosphere& a = atmo_;
    // ---- P105: FUEL_ATMO_TRACE=1 prints the atmosphere inputs and the scatter parameters of the frame.
    if (static const bool trace = std::getenv("FUEL_ATMO_TRACE") != nullptr; trace)
        std::printf("atmo: t %.0f s sun.y %.4f blueShift.w %.3f I %.3f | weather cloud %.3f fog %.3f wet %.3f base %.0f arg5 %.0f | "
                    "P6 %.3f P7 %.0f P8 %.3f P9 %.3f P10 %.0f P11 %.0f P12 %.3f | eye y %.1f\n",
                    in.timeOfDay, a.sunHeight, a.blueShift[3], a.intensity, in.weather.cloud, in.weather.fog, in.weather.wet,
                    in.weather.cloudBase, in.weather.cloudTopArg, a.P[6], a.P[7], a.P[8], a.P[9], a.P[10], a.P[11], a.P[12], eye.y);
    // ---- end P105
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
    fu.decalBias[0] = simd_make_float4(decalLodBias_[0], decalLodBias_[1], decalLodBias_[2], decalLodBias_[3]);  // PK28: 0x84ca80
    fu.decalBias[1] = simd_make_float4(decalLodBias_[4], decalLodBias_[5], 0, 0);
    fillCarOcclusion(fu, eye);
}

// P74: C_OMNI_0_WORLDTOLOCAL of each car occlusion omni (Omni_Z::vf26 0x690700 tail): M = Scale(1/e) x the node's
// world, "inverted" by transposing its 3x3 (0x5f9a70) and moving the translation (0x5f9a00), so with an unscaled
// node local = (w - T) R^T / e. Row k = (axis_k / e_k, -axis_k.T / e_k), in double. The nearest kMaxCarOcclusion
// vehicles to the eye are kept (FUEL hides the omni of vehicles failing 0x771fa0, 0x5cbf10), in input order.
void MeshRenderer::fillCarOcclusion(FrameUniforms& fu, simd_float3 eye) const {
    fu.lightInfo = simd_make_float4(0, 0, 0, sunTex_ != nil ? 1.0f : 0.0f);
    fillVehicleOmnis(fu, eye);  // P77
    if (!carOcclusionOn || carSpot_ == nil) return;
    if (!(carOccOmniFlags_ & omni::kOmniGathered)) return;  // PK28: switched off by RTC message 0xc6
    std::vector<size_t> order(carOcclusion_.size());
    for (size_t i = 0; i < order.size(); ++i) order[i] = i;
    auto dist2 = [&](size_t i) { return simd_distance_squared(carOcclusion_[i].world.columns[3].xyz, eye); };
    if (order.size() > kMaxCarOcclusion) {
        std::nth_element(order.begin(), order.begin() + kMaxCarOcclusion, order.end(), [&](size_t a, size_t b) { return dist2(a) < dist2(b); });
        order.resize(kMaxCarOcclusion);
        std::sort(order.begin(), order.end());
    }
    int n = 0;
    for (size_t i : order) {
        const CarOcclusion& c = carOcclusion_[i];
        const simd_float4x4& w = c.world;
        for (int k = 0; k < 3; ++k) {
            const double e = std::max(1e-4, double(c.extents[k]));
            const double ax = w.columns[k].x / e, ay = w.columns[k].y / e, az = w.columns[k].z / e;
            const double t = ax * w.columns[3].x + ay * w.columns[3].y + az * w.columns[3].z;
            fu.carOcc[n * 3 + k] = simd_make_float4(float(ax), float(ay), float(az), float(-t));
        }
        ++n;
    }
    fu.lightInfo.x = float(n);
}

// P77: the headlights of the frame (setVehicleLights), switched by the day factor (0x5cbde0: on while 0x5b98d0 < 1).
// ---- PK28: FUEL's gather instead of P77's 16 nearest. Skel_Z's link update 0x6a0200 appends, per viewport, a 0x68-byte
// record (0x7984c0) to the list +0x3178 for every light link whose node is an Omni_Z (type 0x10), not hidden (+0x2c
// 0x40000: 0x5cbde0 hides the lights by day) and whose bound sphere (+0xc x node scale) passes 0x821830 against the
// view; 0x6a2ed0 caps the list at 64 (later records overwrite slot 63). The deferred pass 0x7fdd80 runs Omni_Z vf43
// for the records whose Omni_Z is a spot (+0x60 0x20000000). Order: vehicles as given (player first), links in order.
void MeshRenderer::fillVehicleOmnis(FrameUniforms& fu, simd_float3 eye) const {
    fu.lightInfo.y = 0;
    omniRows_.clear();
    omniGathered_ = 0;
    static const bool off = std::getenv("FUEL_NO_HEADLIGHTS") != nullptr, trace = std::getenv("FUEL_OMNI_TRACE") != nullptr;
    const int cap = std::clamp(omniCap, 0, omni::kMaxLightRecords);  // FUEL_OMNI_MAX / --perf-alternate omni=N (A / B only)
    if (off || vehicleLights_.empty() || !omni::headlightsOn(timeOfDay())) return;
    // 0x821830's side planes from the frame's projection (camera-relative rows; inward normals through the eye) and the
    // view axis. Without a projection (shaderCheck) every light passes.
    const simd_float4x4& M = fu.viewProj;
    auto row = [&](int k) { return simd_make_float3(M.columns[0][k], M.columns[1][k], M.columns[2][k]); };
    const simd_float3 r0 = row(0), r1 = row(1), r3 = row(3);
    std::array<simd_float3, 4> planes = {r3 + r0, r3 - r0, r3 + r1, r3 - r1};
    bool haveView = true;
    for (simd_float3& p : planes) {
        const float l = simd_length(p);
        if (!(l > 1e-6f)) haveView = false;
        p = l > 1e-6f ? p / l : p;
    }
    const simd_float3 fwd = simd_make_float3(fu.view.columns[0][2], fu.view.columns[1][2], fu.view.columns[2][2]);
    std::vector<const VehicleOmni*> list;
    for (const VehicleOmni& l : vehicleLights_) {
        if (haveView) {
            const simd_float3 d = l.pos - eye;
            // view space as 0x821830 sees it: z along the view axis; the side planes are rotation-invariant (world axes)
            // [I] near = 0, far = infinite: the camera's +0x6bc / +0x6c0 (0.5 / 45 km) are not passed down here.
            const float z = simd_dot(d, fwd);
            bool vis = z >= -l.radius;
            for (int k = 0; k < 4 && vis; ++k) vis = simd_dot(planes[size_t(k)], d) >= -l.radius;
            if (!vis) continue;
        }
        ++omniGathered_;
        omni::appendRecord(list, &l);
    }
    int n = 0;
    for (const VehicleOmni* lp : list) {  // 0x7fdd80: vf43 only for spot omnis
        if (n >= cap) break;
        if (!(lp->flags & omni::kOmniSpot)) continue;
        omniRows_.push_back(simd_make_float4(lp->pos, 1.0f));
        omniRows_.push_back(simd_make_float4(lp->dir, 0.0f));
        omniRows_.push_back(lp->value);
        omniRows_.push_back(simd_make_float4(lp->colour, 1.0f));
        ++n;
    }
    fu.lightInfo.y = float(n);
    if (trace && frame_ % 60 == 0)
        std::printf("omnis: %zu lights, %d gathered (cap 64), %d drawn, time %.0f s, day factor %.2f, eye (%.1f %.1f %.1f)\n",
                    vehicleLights_.size(), omniGathered_, n, timeOfDay(), omni::dayFactor(timeOfDay()), eye.x, eye.y, eye.z);
    static const bool debug = std::getenv("FUEL_OMNI_DEBUG") != nullptr;
    fu.lightInfo.z = debug ? 1.0f : 0.0f;
}

// ---- PK28: RotShape_Z (core/rotshape.hpp). RotShapePc_Z::vf16 0x835a00 per placed node: the fade 0x8834f0 (drawn at
// view alpha >= 0.01), then per point: the material's slot textures streamed in (0x6ddd20: always in the port), flag
// 0x80 skips, one record of vertex code 0xb / 0xc. Opaque phase 2 (bits 0xf) when the material is not blended
// (0x20) and the object is not fading; code 4 = the occlusion-sized flare (no shipped RotShape material has it: not
// ported); else the blended phase 0x12 | (~flags >> 16 & 1) with key 65535 / (z + 1), a fading object's depth
// pre-pass (bits 0x83 / 0x03) when (flags & 0xb000) == 0 or flags & 0x200000, colour bits 0x1d.
const MeshRenderer::GpuMaterial& MeshRenderer::rotMaterial(uint32_t id) {
    if (auto it = rotMats_.find(id); it != rotMats_.end()) return it->second;
    return rotMats_[id] = makeMaterial(id);
}

void MeshRenderer::gatherRotShapes(simd_float3 eye, const ViewCull& cull) {
    rotRecords_.clear();
    if (!rotShapesOn) return;  // FUEL_NO_ROTSHAPES / --perf-alternate rotshapes
    const LodView view;  // the main view's K' and view scale (p36)
    for (const rotshape::Placement& p : rotShapes_) {
        if (!p.sub.empty() && rotSubActive_ && !rotSubActive_(p.sub)) continue;
        const rotshape::Shape& sh = *p.shape;
        const simd_float3 T = simd_make_float3(p.world[12], p.world[13], p.world[14]);
        const float d = simd_distance(T, eye);
        const float alpha = rotshape::fade(sh, d, p.lodScale, view.viewScale, view.Kfade);
        if (!(alpha >= 0.01f)) continue;  // [0x99614c]
        // the node's bound sphere (object sphere x scale) against the view, as the scene draw 0x752bf0 culls objects
        const simd_float3 sc = simd_make_float3(sh.sphere[0], sh.sphere[1], sh.sphere[2]);
        const simd_float3 cw = sc.x * simd_make_float3(p.world[0], p.world[1], p.world[2]) + sc.y * simd_make_float3(p.world[4], p.world[5], p.world[6]) +
                               sc.z * simd_make_float3(p.world[8], p.world[9], p.world[10]) + T;
        if (cull.sphereOutside(cw, sh.sphere[3] * p.lodScale)) continue;  // the node's bound sphere (0x752bf0 -> 0x821620)
        for (const rotshape::Point& pt : sh.points) {
            if (pt.material >= sh.materials.size()) continue;
            const uint32_t matId = sh.materials[pt.material];
            const GpuMaterial& m = rotMaterial(matId);
            if (m.skip) continue;  // flag 0x80
            if ((m.flags & 0x1f) == 4) continue;  // the type-4 flare (occlusion query 0x84f6c0): not ported, none shipped
            RotRecord r;
            const simd_float3 pw = pt.pos[0] * simd_make_float3(p.world[0], p.world[1], p.world[2]) +
                                   pt.pos[1] * simd_make_float3(p.world[4], p.world[5], p.world[6]) +
                                   pt.pos[2] * simd_make_float3(p.world[8], p.world[9], p.world[10]) + T;
            static const float dbgScale = std::getenv("FUEL_ROTSHAPE_SCALE") ? float(std::atof(std::getenv("FUEL_ROTSHAPE_SCALE"))) : 1.0f;  // debug
            r.pivot = simd_make_float4(pw, p.lodScale * dbgScale);
            r.upLeft = simd_make_float4(pt.upLeft[0], pt.upLeft[1], pt.upLeft[2], sh.front ? 1.0f : 0.0f);
            r.downRight = simd_make_float4(pt.downRight[0], pt.downRight[1], pt.downRight[2], alpha);
            r.extra = simd_make_float4((m.flags & 0x3000) ? 0.0f : 1.0f, 0, 0, 0);  // MTL_ADDSOUS_NULL (SetUnlit 0x84f060)
            r.material = matId;
            r.dist = simd_distance(pw, eye);
            r.fading = alpha < 1.0f;
            rotRecords_.push_back(r);
        }
    }
    rotDrawn_ = rotRecords_.size();
    static const char* traceEnv = std::getenv("FUEL_ROTSHAPE_TRACE");  // 1: every 60th frame, 2: every frame
    if (traceEnv && (frame_ % 60 == 0 || traceEnv[0] == '2')) {
        std::printf("rotshapes: %zu placed, %zu records\n", rotShapes_.size(), rotRecords_.size());
        for (const RotRecord& r : rotRecords_)
            std::printf("  rot %08x at %.1f %.1f %.1f dist %.0f alpha %.2f\n", r.material, r.pivot.x, r.pivot.y, r.pivot.z, r.dist, r.downRight.w);
    }
}

void MeshRenderer::setRotShapes(std::vector<rotshape::Placement> list, std::function<bool(const std::string&)> subActive) {
    rotShapes_ = std::move(list), rotSubActive_ = std::move(subActive);
    for (const rotshape::Placement& p : rotShapes_)
        for (uint32_t id : p.shape->materials) {
            const GpuMaterial& m = rotMaterial(id);
            (void)rotPipeline(m.alpha == AlphaMode::Blend ? 1u + uint32_t(fuel::blendKind(m.flags)) : 0u);
            if (m.alpha != AlphaMode::Blend) (void)rotPipeline(1u);  // a fading opaque shape draws alpha-blended
            if (fuel::fadePrePass(m.flags)) (void)rotPipeline(0xffu);
        }
}

id<MTLRenderPipelineState> MeshRenderer::rotPipeline(uint32_t kind) {
    if (auto it = rotPipes_.find(kind); it != rotPipes_.end()) return it->second;
    MTLRenderPipelineDescriptor* pd = [MTLRenderPipelineDescriptor new];
    pd.vertexFunction = [shaders_ newFunctionWithName:@"vsRotShape"];
    pd.fragmentFunction = [shaders_ newFunctionWithName:@"fsRotShape"];
    pd.rasterSampleCount = sampleCount_;
    pd.colorAttachments[0].pixelFormat = kRenderColorFormat;
    pd.depthAttachmentPixelFormat = kRenderDepthFormat;
    auto* ca = pd.colorAttachments[0];
    if (kind == 0xffu) {
        ca.writeMask = MTLColorWriteMaskNone;
    } else if (kind >= 1) {
        applyBlend(ca, fuel::BlendKind(kind - 1));
    }
    NSError* err = nil;
    id<MTLRenderPipelineState> ps = [device_ newRenderPipelineStateWithDescriptor:pd error:&err];
    if (ps == nil) [NSException raise:@"pipeline" format:@"%@", err];
    ++oneOff_.pipelines;
    return rotPipes_[kind] = ps;
}

void MeshRenderer::drawRotRecord(id<MTLRenderCommandEncoder> enc, const RotRecord& r, bool prepass) {
    const GpuMaterial& m = rotMaterial(r.material);
    const bool blended = m.alpha == AlphaMode::Blend || r.fading;
    const uint32_t kind = prepass ? 0xffu : blended ? 1u + uint32_t(fuel::blendKind(m.flags)) : 0u;
    [enc setRenderPipelineState:rotPipeline(kind)];
    [enc setDepthStencilState:prepass || !blended || fuel::blendZWrite(m.flags) ? depthWrite_ : depthNoWrite_];
    [enc setCullMode:MTLCullModeNone];  // pass bits 0xf / 0x1d / 0x03: no cull bit
    // ALPHAREF 0: 0x835a00's pass bits never carry 0x40 (MeshPc's flag-0x100000 ALPHAREF 128 is not applied here)
    MaterialUniforms mu{m.diffuse, m.specular, m.emissive, m.fresnel, uint32_t(blended ? AlphaMode::Blend : m.alpha), m.lit ? 1u : 0u, m.code,
                        m.features, m.uv0, simd_make_float4(m.uv1.x, m.uv1.y, 0.0f, 0.0f), m.lodBias, m.lodBias2};
    id<MTLTexture> tex = m.textures[0];
    if (m.anim >= 0) {  // an auto-playing MaterialAnim_Z (as bindDraw)
        const AnimatedMaterial& a = materialAnims_[size_t(m.anim)];
        mu.uv0 = a.uv0;
        mu.uv1 = simd_make_float4(a.uv1.x, a.uv1.y, 0.0f, 0.0f);
        if (!a.anim.diffuse.keys.empty() || !a.anim.alpha.keys.empty()) mu.diffuse = a.diffuse;
        if (!a.anim.emissive.keys.empty()) mu.emissive = a.emissive;
        if (a.texture != nil) tex = a.texture;
    }
    [enc setVertexBytes:&r length:sizeof(simd_float4) * 4 atIndex:2];
    [enc setVertexBytes:&mu length:sizeof mu atIndex:3];
    [enc setFragmentBytes:&mu length:sizeof mu atIndex:2];
    [enc setFragmentTexture:tex atIndex:0];
    [enc setFragmentSamplerState:samplers_[m.samplerIndex] atIndex:0];
    [enc drawPrimitives:MTLPrimitiveTypeTriangleStrip vertexStart:0 vertexCount:4];
}
// ---- PK28 end

// ---- PK28: the gathered lights' rows as inline bytes (at most 64 x 4 rows = 4 KB, setBytes' limit; one zero row
// when there are none): no buffer is made per frame.
const void* MeshRenderer::omniBytes(NSUInteger* length) const {
    static const simd_float4 zero[1] = {{0, 0, 0, 0}};
    *length = std::max<NSUInteger>(sizeof(simd_float4), NSUInteger(omniRows_.size()) * sizeof(simd_float4));
    return omniRows_.empty() ? static_cast<const void*>(zero) : static_cast<const void*>(omniRows_.data());
}
// ---- PK28 end

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
    // ---- P94: a plant casts while its decor kind shows (vsShadow drops the others; leaving them out here saves their
    // draws), decided once per frame, lazily for the plants a slice reaches.
    std::vector<int8_t> plantShows(worldInstances_.size(), -1);
    auto plantCasts = [&](size_t k) {
        const Instance& inst = worldInstances_[k];
        const auto kind = fuel::DecorFade(uint8_t(inst.world.columns[1].w));
        if (kind == fuel::DecorFade::None) return true;
        int8_t& v = plantShows[k];
        if (v < 0) {
            const simd_float3 p = inst.world.columns[3].xyz;
            v = fuel::decorAtt(kind, p.x, p.z, fu.eye.x, fu.eye.z, std::hypot(p.x - fu.eye.x, p.z - fu.eye.z)) > 0.5f;
        }
        return v != 0;
    };
    // ---- P94 end

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
        if (!(perfSkip & SkipShadowObjects))  // P95 profiling: the indented block below
        for (const Instance& inst : instances_) consider(inst);
        if (!(perfSkip & SkipShadowObjects))
        for (const OtherVehicle& ov : vehicles_)
            if (!ov.instances.empty() && simd_distance(ov.instances[0].world.columns[3].xyz, center_) <= kVehicleDrawDistance)
                for (size_t i = 0; i < ov.instances.size(); ++i)
                    if (ov.drawn(i)) consider(ov.instances[i]);  // P104: the drawn LOD level only (the same view, 0x7566b0)
        if (!(perfSkip & SkipShadowObjects)) {  // P95 profiling
            for (size_t k = 0; k < worldInstances_.size(); ++k)
                if (plantCasts(k)) consider(worldInstances_[k]);  // P94
            for (const Instance& inst : propInstances_) consider(inst);
            for (const Instance& inst : lodVisible_) consider(inst);
        }

        MTLRenderPassDescriptor* rp = [MTLRenderPassDescriptor renderPassDescriptor];
        rp.depthAttachment.texture = shadowMap_;
        rp.depthAttachment.slice = NSUInteger(i);
        rp.depthAttachment.loadAction = MTLLoadActionClear;
        rp.depthAttachment.clearDepth = 0.0;  // reversed depth
        rp.depthAttachment.storeAction = MTLStoreActionStore;
        gpuprof::tag(rp, "shadow");  // P92: per-pass GPU timing (no-op unless profiling)
        id<MTLRenderCommandEncoder> enc = [cb renderCommandEncoderWithDescriptor:rp];
        [enc setDepthStencilState:shadowDepth_];
        [enc setCullMode:MTLCullModeNone];
        ShadowUniforms su{};
        // P72: casters are camera-relative like fuelClip (P60): lightViewProj x translate(eye) folded in double, the
        // shaders subtract the eye in float, so ~40 km world coordinates no longer cost light-depth precision.
        Mat4d rel = sliceLvp_[size_t(i)];
        for (int r = 0; r < 4; ++r)
            rel[size_t(r * 4 + 3)] += rel[size_t(r * 4)] * double(fu.eye.x) + rel[size_t(r * 4 + 1)] * double(fu.eye.y) +
                                      rel[size_t(r * 4 + 2)] * double(fu.eye.z);
        su.lightViewProj = toSimd(rel);
        su.light = simd_make_float4(float(L[0]), float(L[1]), float(L[2]), sliceBias_[size_t(i)]);
        su.eye = fu.eye;
        su.lightRight = simd_make_float4(float(rt[0]), float(rt[1]), float(rt[2]), 0);
        su.lightUp = simd_make_float4(float(up[0]), float(up[1]), float(up[2]), 0);
        [enc setVertexBytes:&su length:sizeof su atIndex:1];

        size_t total = 0;
        for (const auto& g : groups) total += g.second.size();
        cpu_.shadowCasters += double(total);  // P92 statistics
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
                            const simd_float2 half = simd_make_float2((m.code == 17 || m.code == 20) ? 1.0f : 0.0f, m.lodBias.x);  // PK28: + bias
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
                        cpu_.shadowTriangles += double(d.indexCount / 3) * double(worlds.size());  // P92 statistics
                    }
                }
            }
        }
        // Terrain chunks cast too (ground.vhl vh_shadow).
        if (!chunks.empty() && terrainIndices_ != nil && !(perfSkip & SkipShadowTerrain)) {
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
                cpu_.shadowChunks += 1, cpu_.shadowTriangles += double(terrainIndexCount_ / 3);  // P92 statistics
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
    fu.cluster = fu.clusterZ = fu.clusterView = fu.camFwd = simd_make_float4(0, 0, 0, 0);  // P92: no clusters (the cube faces copy fu early)
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
    if (preview.on) {  // ---- PK20: 0x845ac0's projection (1/tanx, 1/tany, far / (far - near)) with the preview camera's planes
        const float pn = preview.zNear, pf = preview.zFar, q = pf / (pn - pf);
        const simd_float4x4 P = simd_matrix(simd_make_float4(1.0f / preview.tanX, 0, 0, 0), simd_make_float4(0, 1.0f / preview.tanY, 0, 0),
                                            simd_make_float4(0, 0, q, -1), simd_make_float4(0, 0, pn * q, 0));
        fu.viewProj = simd_mul(P, lookAtRH(eye, target, upVec));
    }  // ---- PK20 end
    fu.invViewProj = simd_inverse(fu.viewProj);
    fu.viewProjStable = fu.viewProj;               // TAA: the jitter (taaFrameUniforms) goes on after the shadows / cube / clusters
    fu.taa = simd_make_float4(texBiasBase_, 0, 0, 0);  // TAA: FUEL's LOD bias (PK28's kTexBias)
    {
        const simd_float3 fwd = simd_normalize(target - eye);
        const simd_float3 right = simd_normalize(simd_cross(fwd, upVec));
        fu.camRight = simd_make_float4(right, 0);
        fu.camUp = simd_make_float4(simd_cross(right, fwd), 0);
    }
    ViewCull cull;  // P92: the scene pass skips terrain chunks / grass runs wholly off screen
    if (!std::getenv("FUEL_NO_VIEW_CULL"))
        cull.set(eye, fu.camRight.xyz, fu.camUp.xyz, simd_cross(fu.camUp.xyz, fu.camRight.xyz), std::tan(fovY * 0.5f) * aspect, std::tan(fovY * 0.5f));
    if (preview.on && cull.on)  // ---- PK20
        cull.set(eye, fu.camRight.xyz, fu.camUp.xyz, simd_cross(fu.camUp.xyz, fu.camRight.xyz), preview.tanX, preview.tanY);
    fu.sunViewProj = sunViewProj();
    fu.terrain = terrainInfo_;
    fu.terrainGrid = simd_make_float4(center_.x, center_.z, std::max(70000.0f, dist * 6.0f), float(kTerrainGridSize));
    fu.fog = simd_make_float4(terrain ? 1.0f / 3600.0f : 0.0f, groundColour_ != nil ? 1.0f : 0.0f, zn, zf / (zn - zf));
    if (preview.on) fu.fog.z = preview.zNear, fu.fog.w = preview.zFar / (preview.zNear - preview.zFar);  // ---- PK20
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
    using PerfClock = std::chrono::steady_clock;  // P92: CPU timings (cpu_)
    auto msSince = [](PerfClock::time_point t) { return std::chrono::duration<double, std::milli>(PerfClock::now() - t).count(); };
    auto tPhase = PerfClock::now();
    if (terrain && terrainLod_) {
        const bool jump = simd_distance(simd_make_float2(eye.x, eye.z), simd_make_float2(lastTerrainEye_.x, lastTerrainEye_.z)) > 2000.0f ||
                          std::exchange(terrainSettle_, false);  // PK15: a teleport's regeneration (0x640fe0)
        lastTerrainEye_ = eye;
        if (jump) cubeFresh_ = true;
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
            if (g.vb == nil) {
                g.vb = [device_ newBufferWithBytes:c->verts.data() length:c->verts.size() * sizeof(TerrainVertex)
                                           options:MTLResourceStorageModeShared];
                ++oneOff_.chunks;  // P92
                // P92: the chunk's box over every vertex and morph target (the geomorph moves between the two).
                simd_float3 lo = simd_make_float3(1e30f, 1e30f, 1e30f), hi = -lo;
                for (const TerrainVertex& v : c->verts) {
                    const simd_float3 a = simd_make_float3(v.p0[0], v.p0[1], v.p0[2]), b = simd_make_float3(v.p1[0], v.p1[1], v.p1[2]);
                    lo = simd_min(lo, simd_min(a, b)), hi = simd_max(hi, simd_max(a, b));
                }
                g.lo = lo, g.hi = hi;
            }
            g.lastFrame = chunkFrame_;
        }
        for (auto it = chunkGpu_.begin(); it != chunkGpu_.end();)
            it = chunkFrame_ - it->second.lastFrame > 240 ? chunkGpu_.erase(it) : std::next(it);
    }

    cpu_.terrain = msSince(tPhase), tPhase = PerfClock::now();
    evaluateLodObjects(eye, cull);  // p36: per-object LOD level and fade from this frame's camera (P92: off-screen ones culled)
    cpu_.lod = msSince(tPhase);
    {
        // MaterialManager 0x7acfc0 advances every registered MaterialAnim_Z by the frame time.
        const auto now = std::chrono::steady_clock::now();
        float dt = haveAnimClock_ ? std::min(std::chrono::duration<float>(now - animClock_).count(), 0.1f) : 0.0f;
        if (fixedClock_) dt = haveAnimClock_ ? 1.0f / 60.0f : 0.0f;  // P92: FUEL_FIXED_CLOCK (repeatable shots)
        if (pauseSky_) dt = 0;  // PK17: GameManager_Z::Update 0x660aa0 skips 0x7acfc0 while paused (+0x40)
        animClock_ = now, haveAnimClock_ = true;
        updateMaterialAnims(dt);
    }
    // ---- FUEL's lights, scattering and shadow slices for this frame (P28), then the scene pass.
    fu.paint = simd_make_float4(paint_.color.x, paint_.color.y, paint_.color.z, 1);  // gamma space, like the game
    fillAtmosphere(fu, eye, fu.viewProj, 0, aspect, zn);
    const bool ownEncoder = enc == nil;
    id<MTLBuffer> omniClusters = nil;  // P92: headlight clusters (with the frame's own encoder)
    if (ownEncoder) {
        // FUEL_PROFILE=1: frames cycle full / no shadow lookups / no vertex scattering / no shadow slices.
        static const bool profile = std::getenv("FUEL_PROFILE") != nullptr;
        const uint32_t pm = profile ? frame_ % 4 : 0;
        fu.fog.x = pm == 1 ? 1.0f : pm == 2 ? 2.0f : 0.0f;
        static const bool cubeDebug = std::getenv("FUEL_CUBE_DEBUG") != nullptr;  // reflective surfaces show the cube behind them
        if (cubeDebug) fu.fog.x += 8.0f;
        if (shadow2x2) fu.fog.x += 16.0f;  // P93: settings.txt shadow_filter = 2x2 (fuelShadow: pcf2x2)
        if (preview.on) {  // ---- PK20: the preview's own lights (0x68ffd0 + 0x4cfec0 -> light blocks +0x268 / +0x2b0 and the
            // scatter of its Atmo, 0x7ca0a0 mode 6), no shadow slices (fuelShadow: 1 when shadowSize.x <= 0), a game cube
            fu.lightDir = preview.lightDir, fu.skyDir = preview.skyDir, fu.sunColour = preview.sunColour, fu.dlight = preview.dlight;
            fu.skyColour = preview.skyColour, fu.skyAmbient = preview.skyAmbient;
            for (size_t i = 0; i < 11; ++i) fu.scat[i] = preview.scat[i];
            fu.shadowSize = fu.shadowSplit = simd_make_float4(0, 0, 0, 0);
            if (preview.envCube != nil) fu.fog.x += 32.0f;
        }  // ---- PK20 end
        tPhase = PerfClock::now();
        if (pm != 3 && !preview.on) encodeShadows(frameCb_, fu, terrainChunks);  // PK20: no slices in the preview
        cpu_.shadows = msSince(tPhase), tPhase = PerfClock::now();
        if (!(perfSkip & SkipCube) && !preview.on)  // P95 profiling; PK20: no cube faces in the preview
            encodeCube(frameCb_, fu, terrain ? terrainChunks : std::vector<std::shared_ptr<const TerrainChunk>>{}, cubeFresh_);
        cpu_.cube = msSince(tPhase);
        cubeFresh_ = false;
        if (perfSkip & SkipOmnis) fu.lightInfo.y = 0;
        omniClusters = encodeOmniClusters(frameCb_, fu, fovY, aspect);  // P92: after the cube faces (they try every omni)
        gpuprof::tag(frameRp_, "scene");  // P92: per-pass GPU timing (no-op unless profiling)
        enc = [frameCb_ renderCommandEncoderWithDescriptor:frameRp_];
        applyView(enc);  // P93: the scaled render area
    }
    tPhase = PerfClock::now();
    // P92 profiling (--perf-breakdown): the shadow lookups / headlight omnis left out (fsMain, fsTerrain, fsGrass).
    if (perfSkip & SkipShadowLookup) fu.fog.x = float(uint32_t(fu.fog.x) | 1u);
    stableFrame_.reset();
    if (temporalFrame_ && ownEncoder) taaFrameUniforms(fu, lastScale_);  // TAA: jitter + LOD bias (stableFrame_ keeps the unjittered copy)
    lastFrame_ = std::make_shared<FrameUniforms>(fu);

    [enc setVertexBytes:&fu length:sizeof fu atIndex:1];
    [enc setFragmentBytes:&fu length:sizeof fu atIndex:1];
    [enc setFragmentBuffer:omniClusters != nil ? omniClusters : noMorph_ offset:0 atIndex:5];  // P92: clusters (fsMain, fsTerrain, fsGrass)
    {  // PK28: the gathered lights' rows (up to 64)
        NSUInteger n = 0;
        const void* rows = omniBytes(&n);
        [enc setFragmentBytes:rows length:n atIndex:6];
    }
    [enc setFragmentTexture:shadowMap_ atIndex:7];
    [enc setFragmentTexture:preview.on && preview.envCube != nil ? preview.envCube : globalCube_ atIndex:8];  // PK20: mode 6's cube
    [enc setFragmentTexture:carSpot_ != nil ? carSpot_ : white_ atIndex:14];  // P74: sProjSpotMap0 (fsMain, fsTerrain)
    [enc setFrontFacingWinding:MTLWindingClockwise];

    if (terrain && !(perfSkip & SkipSky)) {
        [enc setRenderPipelineState:skyPipeline_];
        [enc setFragmentTexture:sunTex_ != nil ? sunTex_ : white_ atIndex:0];  // P74: the sun / moon sprite
        [enc setDepthStencilState:depthAlways_];
        [enc setCullMode:MTLCullModeNone];
        [enc drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:0 vertexCount:3];
    }

    // One Draw's vertex buffer, material constants and textures (BindMaterial 0x84f470). fadeBlend = 1 marks a fading
    // object's pre-pass / colour pass (fsMain: alpha x the view alpha instead of the dither).
    // P85: the drawn vehicle part's damage morph (Instance::morph), nil for everything else.
    id<MTLBuffer> curMorph = nil;
    const VehicleVisual* curLamps = nullptr;  // PK22: the drawn vehicle's lamp clones (blended phases)
    if (noMorph_ == nil) noMorph_ = [device_ newBufferWithLength:16 options:MTLResourceStorageModeShared];
    auto bindDraw = [&](const GpuMesh& gm, const Draw& d, float fadeBlend) {
        const GpuMaterial& m = gm.materials[d.material];
        [enc setVertexBuffer:gm.vbs[d.vb] offset:0 atIndex:0];
        const bool morphed = d.morph && curMorph != nil;  // P85
        const uint32_t sprite = (d.sprite ? 1u : 0u) | (morphed ? 2u : 0u);
        [enc setVertexBytes:&sprite length:sizeof sprite atIndex:3];
        [enc setVertexBuffer:morphed ? curMorph : noMorph_ offset:0 atIndex:4];  // P85
        [enc setVertexTexture:groundColour_ != nil ? groundColour_ : white_ atIndex:0];
        MaterialUniforms mu{m.diffuse, m.specular, m.emissive, m.fresnel, uint32_t(m.alpha), m.lit ? 1u : 0u, m.code,
                            m.features | (d.skinned ? uint32_t(NoCarOcclusion) : 0u), m.uv0,
                            simd_make_float4(m.uv1.x, m.uv1.y, m.alphaRef, fadeBlend), m.lodBias, m.lodBias2};  // PK28: + LOD bias
        id<MTLTexture> diffuseTex = m.textures[0];
        if (m.anim >= 0) {  // an auto-playing MaterialAnim_Z: this frame's fields (updateMaterialAnims)
            const AnimatedMaterial& a = materialAnims_[size_t(m.anim)];
            mu.uv0 = a.uv0;
            mu.uv1 = simd_make_float4(a.uv1.x, a.uv1.y, m.alphaRef, fadeBlend);
            if (!a.anim.diffuse.keys.empty() || !a.anim.alpha.keys.empty())
                mu.diffuse = (m.features & Reflective) ? simd_make_float4(1, 1, 1, a.diffuse.w) : a.diffuse;
            if (!a.anim.specular.keys.empty()) mu.specular = a.specular;
            if (!a.anim.emissive.keys.empty()) mu.emissive = a.emissive;
            if (a.texture != nil) diffuseTex = a.texture;
        }
        if (m.lamp >= 0 && curLamps != nullptr) {  // PK22: the clone (0x69da00 colour x 5, 0x5cc780 alpha I)
            mu.diffuse.w = curLamps->lampAlpha[size_t(m.lamp)];
            mu.emissive = simd_make_float4(curLamps->lampEmissive[size_t(m.lamp)], 0);
        }
        [enc setFragmentBytes:&mu length:sizeof mu atIndex:2];
        for (size_t slot = 0; slot < m.textures.size(); ++slot)
            [enc setFragmentTexture:slot == 0 ? diffuseTex : m.textures[slot] atIndex:slot];
        [enc setFragmentSamplerState:samplers_[m.samplerIndex] atIndex:0];
    };
    // P72: MeshPc_Z::Draw 0x848880 takes the fade path (view alpha < 1, 0x84ec60) only for the default material
    // codes; codes 1, 0x11, 0x13 (pass bits 0x2f), 0x12 / 0x14 (0xf / 0x1f), 0xd (refraction) and 0x17 (0x9d) draw
    // as if opaque whatever the alpha.
    auto ignoresFade = [](uint32_t code) { return fuel::ignoresFade(code); };  // core/material_anim
    // P95: this frame's on-screen world / prop instances and opponents (the scene pass only).
    const bool cullOn = !(p95Off & P95CullInstances) && cull.on;
    if (cullOn) cullGroups(worldCull_, worldVis_, cull), cullGroups(propCull_, propVis_, cull);
    std::vector<char> vehicleOff(vehicles_.size(), 0);
    for (size_t v = 0; v < vehicles_.size(); ++v) vehicleOff[v] = vehicleOffScreen(v, cull) ? 1 : 0;
    if (temporalFrame_) {  // TAA: the velocity pass draws the opponents the scene pass draws
        taaVehicleOff_ = vehicleOff;
        for (size_t v = 0; v < vehicles_.size(); ++v)
            if (vehicles_[v].instances.empty() || simd_distance(vehicles_[v].instances[0].world.columns[3].xyz, center_) > kVehicleDrawDistance)
                taaVehicleOff_[v] = 1;
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
                [enc setCullMode:cullBackFaces && !d.sprite && !m.twoSided ? MTLCullModeBack : MTLCullModeNone];
                [enc setRenderPipelineState:pipeline(gm.vbStrides[d.vb], mode, vsVariant(m, d.sprite))];
                bindDraw(gm, d, 0.0f);
                [enc drawIndexedPrimitives:MTLPrimitiveTypeTriangle
                                indexCount:d.indexCount
                                 indexType:MTLIndexTypeUInt32
                               indexBuffer:gm.indices
                         indexBufferOffset:size_t(d.firstIndex) * 4
                             instanceCount:count];
                cpu_.sceneDraws += 1, cpu_.sceneTriangles += double(d.indexCount / 3) * double(count);  // P92 statistics
            }
        };
        for (const Instance& inst : instances_) {
            if (inst.mesh->draws[pass].empty()) continue;
            [enc setVertexBytes:&inst.world length:sizeof inst.world atIndex:2];
            curMorph = inst.morph;  // P85
            drawMesh(*inst.mesh, 1);
        }
        for (size_t v = 0; v < vehicles_.size(); ++v) {  // each with its own paint colour
            const OtherVehicle& ov = vehicles_[v];
            if (ov.instances.empty() || simd_distance(ov.instances[0].world.columns[3].xyz, center_) > kVehicleDrawDistance) continue;
            if (vehicleOff[v]) continue;  // P95: wholly off screen
            FrameUniforms vu = fu;
            vu.paint = simd_make_float4(ov.paint.x, ov.paint.y, ov.paint.z, 1);  // gamma space
            [enc setFragmentBytes:&vu length:sizeof vu atIndex:1];
            for (size_t i = 0; i < ov.instances.size(); ++i) {
                const Instance& inst = ov.instances[i];
                if (inst.mesh->draws[pass].empty() || !ov.drawn(i)) continue;  // P104: another LOD level
                if (ov.fadeOf(i) > 0) {  // P104: fading (the Lod alpha): only the draws whose code ignores the fade (P72)
                    simd_float4x4 w = inst.world;
                    w.columns[0].w = 0.0f;
                    curMorph = inst.morph;
                    for (const Draw& d : inst.mesh->draws[pass]) {
                        const GpuMaterial& m = inst.mesh->materials[d.material];
                        if (!ignoresFade(m.code)) continue;
                        [enc setCullMode:cullBackFaces && !d.sprite && !m.twoSided ? MTLCullModeBack : MTLCullModeNone];
                        [enc setRenderPipelineState:pipeline(inst.mesh->vbStrides[d.vb], mode, vsVariant(m, d.sprite))];
                        [enc setVertexBytes:&w length:sizeof w atIndex:2];
                        bindDraw(*inst.mesh, d, 0.0f);
                        [enc drawIndexedPrimitives:MTLPrimitiveTypeTriangle
                                        indexCount:d.indexCount
                                         indexType:MTLIndexTypeUInt32
                                       indexBuffer:inst.mesh->indices
                                 indexBufferOffset:size_t(d.firstIndex) * 4];
                    }
                    continue;
                }
                [enc setVertexBytes:&inst.world length:sizeof inst.world atIndex:2];
                curMorph = inst.morph;  // P85
                drawMesh(*inst.mesh, 1);
            }
            [enc setFragmentBytes:&fu length:sizeof fu atIndex:1];
        }
        curMorph = nil;  // P85
        for (const WorldGroup& g : cullOn ? worldVis_ : worldGroups_) {  // one instanced draw per mesh (P95: on screen)
            if (g.mesh->draws[pass].empty()) continue;
            [enc setVertexBuffer:g.matrices offset:g.offset atIndex:2];
            drawMesh(*g.mesh, g.count);
        }
        for (const WorldGroup& g : cullOn ? propVis_ : propGroups_) {
            if (g.mesh->draws[pass].empty()) continue;
            [enc setVertexBuffer:g.matrices offset:g.offset atIndex:2];
            drawMesh(*g.mesh, g.count);
        }
        for (const WorldGroup& g : lodGroups_) {  // placed objects at this frame's level, full alpha (p36)
            if (g.mesh->draws[pass].empty()) continue;
            [enc setVertexBuffer:g.matrices offset:g.offset atIndex:2];
            drawMesh(*g.mesh, g.count);
        }
        if (mode == AlphaMode::Blend) return;
        // P72: the draws of fading objects whose material code ignores the fade stay opaque (fade slot cleared).
        for (const Instance& inst : lodFadingObjs_) {
            simd_float4x4 w = inst.world;
            w.columns[0].w = 0.0f;
            for (const Draw& d : inst.mesh->draws[pass]) {
                const GpuMaterial& m = inst.mesh->materials[d.material];
                if (!ignoresFade(m.code)) continue;
                [enc setCullMode:cullBackFaces && !d.sprite && !m.twoSided ? MTLCullModeBack : MTLCullModeNone];
                [enc setRenderPipelineState:pipeline(inst.mesh->vbStrides[d.vb], mode, vsVariant(m, d.sprite))];
                [enc setVertexBytes:&w length:sizeof w atIndex:2];
                bindDraw(*inst.mesh, d, 0.0f);
                [enc drawIndexedPrimitives:MTLPrimitiveTypeTriangle
                                indexCount:d.indexCount
                                 indexType:MTLIndexTypeUInt32
                               indexBuffer:inst.mesh->indices
                         indexBufferOffset:size_t(d.firstIndex) * 4];
            }
        }
    };
    // ---- P72: FUEL's blended phases. MeshPc_Z::Draw 0x848880 queues blended draws (material flag 0x20 = the port's
    // AlphaMode::Blend) and every draw of a fading object (view alpha < 1) in the queue preset / frame phase
    // 0x12 | (~flags >> 16 & 1) (0xc | .. with flag 0x40000); code 0x17 in 0x13. The frame flush 0x858e60 runs the
    // phases in ascending order (Renderer_FramePhase per phase change) and inside a phase sorts the records by the
    // key 65535 / (depth + 1) (depth = record +0; 0 only for the opaque phase 2): back to front. Pass bits: colour
    // 0x1d (ZENABLE, COLORWRITE, ALPHABLEND, no cull), 0x1f (+ ZWRITE) with flag 0x40000 or codes 0x12 / 0x14, 0x9d
    // (+ cull) for code 0x17; a fading object first lays its depth with 0x83 (ZWRITE, no colour, cull; 0x03 with the
    // two-sided flag 0x8000) when (flags & 0xb000) == 0 or flags & 0x200000, so it fades as one surface.
    // Port: depth = the object origin's distance from the eye; the record order inside one object is kept.
    auto drawBlendedPhases = [&]() {
        struct BlendItem {
            uint32_t phase;
            float dist;
            const GpuMesh* gm;
            const Draw* d;
            simd_float4x4 world;
            int vehicle;  // index into vehicles_ (its paint), or -1
            bool fading;
            id<MTLBuffer> morph = nil;  // P85: the vehicle part's damage morph
            const VehicleVisual* lamps = nullptr;  // PK22
            int rot = -1;  // PK28: a RotShape record (rotRecords_), gm / d unused
        };
        std::vector<BlendItem> items;
        auto phaseOf = [](const GpuMaterial& m) { return fuel::blendPhase(m.flags); };
        auto add = [&](const GpuMesh* gm, const simd_float4x4& w, int vehicle, bool fading, id<MTLBuffer> morph = nil,
                       const VehicleVisual* lamps = nullptr) {
            const float dist = simd_distance(w.columns[3].xyz, eye);
            // PK22: a lamp group switched off (0x5cdfc0: alpha <= 0 -> flag 0x80) is not drawn
            auto lampOff = [&](const GpuMaterial& m) { return m.lamp >= 0 && lamps != nullptr && !(lamps->lampAlpha[size_t(m.lamp)] > 0); };
            for (uint32_t pass = fading ? 0u : 2u; pass < 3; ++pass)
                for (const Draw& d : gm->draws[pass]) {
                    const GpuMaterial& m = gm->materials[d.material];
                    if (fading && ignoresFade(m.code)) continue;  // drawn opaque above (blended ones: below)
                    if (lampOff(m)) continue;                     // PK22
                    items.push_back({phaseOf(m), dist, gm, &d, w, vehicle, fading, morph, lamps});
                }
            if (fading)  // blended draws of codes that ignore the fade: their own blended record, full alpha
                for (const Draw& d : gm->draws[2])
                    if (ignoresFade(gm->materials[d.material].code) && !lampOff(gm->materials[d.material])) {
                        simd_float4x4 w0 = w;
                        w0.columns[0].w = 0.0f;
                        items.push_back({phaseOf(gm->materials[d.material]), dist, gm, &d, w0, vehicle, false, morph, lamps});  // P104: with its morph
                    }
        };
        const VehicleVisual* playerLamps = lampState(-1);  // PK22
        for (const Instance& inst : instances_)
            if (!inst.mesh->draws[2].empty()) add(inst.mesh, inst.world, -1, false, inst.morph, playerLamps);
        for (size_t v = 0; v < vehicles_.size(); ++v) {
            const OtherVehicle& ov = vehicles_[v];
            if (ov.instances.empty() || simd_distance(ov.instances[0].world.columns[3].xyz, center_) > kVehicleDrawDistance) continue;
            if (vehicleOff[v]) continue;  // P95: wholly off screen
            for (size_t i = 0; i < ov.instances.size(); ++i) {
                const Instance& inst = ov.instances[i];
                if (!ov.drawn(i)) continue;  // P104: another LOD level
                if (ov.fadeOf(i) > 0) {      // P104: the Lod fade: every draw in the blended phases (as lodFadingObjs_)
                    simd_float4x4 w = inst.world;
                    w.columns[0].w = ov.fadeOf(i);
                    add(inst.mesh, w, int(v), true, inst.morph, lampState(int(v)));
                    continue;
                }
                if (!inst.mesh->draws[2].empty()) add(inst.mesh, inst.world, int(v), false, inst.morph, lampState(int(v)));
            }
        }
        for (const std::vector<Instance>* list : {&worldInstances_, &propInstances_})
            for (const Instance& inst : *list)
                if (!inst.mesh->draws[2].empty()) add(inst.mesh, inst.world, -1, false);
        for (const WorldGroup& g : lodGroups_) {
            if (g.mesh->draws[2].empty()) continue;
            const auto* worlds = reinterpret_cast<const simd_float4x4*>(static_cast<const uint8_t*>(g.matrices.contents) + g.offset);
            for (uint32_t k = 0; k < g.count; ++k) add(g.mesh, worlds[k], -1, false);
        }
        for (const Instance& inst : lodFadingObjs_) add(inst.mesh, inst.world, -1, true);
        for (size_t i = 0; i < rotRecords_.size(); ++i) {  // ---- PK28: blended / fading RotShape records (0x835a00)
            const RotRecord& r = rotRecords_[i];
            const GpuMaterial& m = rotMaterial(r.material);
            if (m.alpha != AlphaMode::Blend && !r.fading) continue;  // drawn in the opaque phase
            BlendItem bi{fuel::blendPhase(m.flags), r.dist, nullptr, nullptr, matrix_identity_float4x4, -1, r.fading};
            bi.rot = int(i);
            items.push_back(bi);
        }  // ---- PK28 end
        std::stable_sort(items.begin(), items.end(), [](const BlendItem& a, const BlendItem& b) {
            return a.phase != b.phase ? a.phase < b.phase : a.dist > b.dist;
        });
        [enc setTriangleFillMode:wireframe ? MTLTriangleFillModeLines : MTLTriangleFillModeFill];
        int boundVehicle = -1;
        for (const BlendItem& it : items) {
            if (it.rot >= 0) {  // ---- PK28: a RotShape record (its own pipeline and bindings)
                const RotRecord& r = rotRecords_[size_t(it.rot)];
                if (r.fading && fuel::fadePrePass(rotMaterial(r.material).flags)) drawRotRecord(enc, r, true);
                drawRotRecord(enc, r, false);
                continue;
            }  // ---- PK28 end
            const GpuMaterial& m = it.gm->materials[it.d->material];
            if (it.vehicle != boundVehicle) {
                FrameUniforms vu = fu;
                if (it.vehicle >= 0) {
                    const simd_float3 p = vehicles_[size_t(it.vehicle)].paint;
                    vu.paint = simd_make_float4(p.x, p.y, p.z, 1);  // gamma space
                }
                [enc setFragmentBytes:&vu length:sizeof vu atIndex:1];
                boundVehicle = it.vehicle;
            }
            [enc setVertexBytes:&it.world length:sizeof it.world atIndex:2];
            const uint32_t stride = it.gm->vbStrides[it.d->vb];
            const auto draw = [&] {
                [enc drawIndexedPrimitives:MTLPrimitiveTypeTriangle
                                indexCount:it.d->indexCount
                                 indexType:MTLIndexTypeUInt32
                               indexBuffer:it.gm->indices
                         indexBufferOffset:size_t(it.d->firstIndex) * 4];
            };
            curMorph = it.morph;  // P85
            curLamps = it.lamps;  // PK22
            bindDraw(*it.gm, *it.d, it.fading ? 1.0f : 0.0f);
            if (it.fading && fuel::fadePrePass(m.flags)) {
                [enc setRenderPipelineState:prepassPipeline(stride, vsVariant(m, it.d->sprite))];  // 0x83 / 0x03
                [enc setDepthStencilState:depthWrite_];
                [enc setCullMode:cullBackFaces && !it.d->sprite && !(m.flags & 0x8000) ? MTLCullModeBack : MTLCullModeNone];
                draw();
            }
            const bool zWrite = fuel::blendZWrite(m.flags);
            [enc setRenderPipelineState:blendPipeline(stride, m.flags, vsVariant(m, it.d->sprite))];  // PK28: 0x856160's mode
            [enc setDepthStencilState:zWrite ? depthWrite_ : depthNoWrite_];
            [enc setCullMode:cullBackFaces && !it.d->sprite && m.code == 0x17 ? MTLCullModeBack : MTLCullModeNone];
            draw();
        }
        if (boundVehicle != -1) [enc setFragmentBytes:&fu length:sizeof fu atIndex:1];
        curLamps = nullptr;  // PK22
    };
    if (terrain && !(perfSkip & SkipTerrain)) {
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
            [enc setFragmentTexture:groundDrawSet_ atIndex:5];  // P92: fsTerrain reads the draw sets (was the types)
            [enc setFragmentTexture:groundDecal_ atIndex:6];
            for (int k = 0; k < kDecalSlots; ++k) [enc setFragmentTexture:decalTex_[k] != nil ? decalTex_[k] : white_ atIndex:NSUInteger(8 + k)];
        }
        for (const auto& c : terrainChunks) {
            const ChunkGpu& g = chunkGpu_[c->key];
            if (cull.boxOutside(g.lo, g.hi)) continue;  // P92: off screen
            cpu_.sceneChunks += 1;  // P92 statistics
            [enc setVertexBuffer:g.vb offset:0 atIndex:0];
            [enc drawIndexedPrimitives:MTLPrimitiveTypeTriangle
                            indexCount:terrainIndexCount_
                             indexType:MTLIndexTypeUInt32
                           indexBuffer:terrainIndices_
                     indexBufferOffset:0];
        }
    }
    // Terrain first (roads are painted into it, ground.phl decal path).
    [enc setFragmentTexture:preview.on && preview.envCube != nil ? preview.envCube : globalCube_ atIndex:8];  // the terrain bound its decal textures there (PK20: mode 6's cube)
    if (!(perfSkip & SkipOpaque)) drawPass(0);
    if (!(perfSkip & SkipCutout)) drawPass(1);
    // ---- PK28: RotShape_Z records (0x835a00): gathered for this view; the opaque ones (phase 2) here
    if (!preview.on) {
        gatherRotShapes(eye, cull);
        for (const RotRecord& r : rotRecords_)
            if (rotMaterial(r.material).alpha != AlphaMode::Blend && !r.fading) drawRotRecord(enc, r, false);
    } else {
        rotRecords_.clear();
    }
    // ---- PK28 end
    if (terrain && !(perfSkip & SkipGrass)) encodeGrass(enc, cull);
    if (terrain && waterVertices_ != nil && waterIndexCount_ > 0 && haveWaterLook_ && !(perfSkip & SkipWater)) {
        [enc setRenderPipelineState:waterPipeline_];
        [enc setDepthStencilState:depthNoWrite_];
        [enc setCullMode:MTLCullModeNone];
        [enc setTriangleFillMode:MTLTriangleFillModeFill];
        const float t = animSeconds();
        waterClock_ = t, taaWaterDrawn_ = true;  // TAA: the velocity pass's water clock
        [enc setVertexBuffer:waterVertices_ offset:0 atIndex:0];
        [enc setVertexBytes:waterConst_.data() length:sizeof(waterConst_) atIndex:2];
        [enc setVertexBytes:&t length:sizeof t atIndex:3];
        [enc setFragmentBytes:waterConst_.data() length:sizeof(waterConst_) atIndex:2];
        [enc setFragmentTexture:waterFoam_ != nil ? waterFoam_ : white_ atIndex:0];
        [enc setFragmentTexture:waterNormal_ != nil ? waterNormal_ : white_ atIndex:1];
        [enc setFragmentTexture:shadowMap_ atIndex:7];
        [enc setFragmentTexture:globalCube_ atIndex:8];
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
    if (!(perfSkip & SkipBlend)) drawBlendedPhases();  // P72: was drawPass(2), unsorted
    for (const std::vector<simd_float4>* list : {&markers_, &debugTrunks_}) {
        if (list->empty()) continue;
        [enc setRenderPipelineState:markerPipeline_];
        [enc setDepthStencilState:depthNoWrite_];
        [enc setCullMode:MTLCullModeNone];
        [enc setTriangleFillMode:MTLTriangleFillModeFill];
        for (size_t at = 0; at < list->size(); at += 256) {  // setVertexBytes takes at most 4 KB
            const size_t n = std::min<size_t>(256, list->size() - at);
            [enc setVertexBytes:list->data() + at length:n * sizeof(simd_float4) atIndex:2];
            [enc drawPrimitives:MTLPrimitiveTypeTriangleStrip vertexStart:0 vertexCount:66 instanceCount:n];
        }
    }
    if (temporalFrame_ && ownEncoder) encodeTaaLumCapture(enc);  // TAA: the pre-effects luminance into the HDR alpha (reactive mask)
    if (ownEncoder) [enc endEncoding];
    cpu_.scene = msSince(tPhase);
}

// ---- TAA [port performance option] (taa_render.mm): a temporal frame's uniforms. The jitter (Halton 2, 3) goes on the
// scene's projection only after the shadow slices, the cube face, the light gather and the headlight clusters were
// made from the unjittered one; viewProjStable keeps it (frameCameraOf, CPU culling and sorting in the effects) and
// stableFrame_ is a copy without the jitter for the screen-stable 3-D markers (GPS arrows, navigator beams). The
// samples' LOD bias becomes FUEL's -1 + log2(render scale); taaCur_ holds the camera the motion vectors use.
void MeshRenderer::taaFrameUniforms(FrameUniforms& fu, float scale) {
    static const bool noJitter = std::getenv("FUEL_TAA_NO_JITTER") != nullptr;  // A / B only
    taaGrassDraws_.clear(), taaWaterDrawn_ = false;  // filled by encodeGrass / the water draw of this frame
    taaJitterPx_ = noJitter ? simd_make_float2(0, 0) : taa::jitter(taaIndex_);
    ++taaIndex_;
    const float W = float(std::max<NSUInteger>(viewW_, 1)), H = float(std::max<NSUInteger>(viewH_, 1));
    fu.taa = simd_make_float4(taa::lodBias(texBiasBase_, scale), 2.0f * taaJitterPx_.x / W, -2.0f * taaJitterPx_.y / H, 1.0f);
    stableFrame_ = std::make_shared<FrameUniforms>(fu);
    stableFrame_->taa.y = stableFrame_->taa.z = 0;
    fu.viewProj = taaJitteredVp_ = taa::jittered(fu.viewProjStable, taaJitterPx_, W, H);
    fu.invViewProj = simd_inverse(fu.viewProj);
    TaaCam& c = taaCur_;
    c.vp = fu.viewProjStable;
    c.eye = fu.eye.xyz, c.right = fu.camRight.xyz;
    c.R = simd_make_float3(fu.view.columns[0].x, fu.view.columns[1].x, fu.view.columns[2].x);
    c.U = simd_make_float3(fu.view.columns[0].y, fu.view.columns[1].y, fu.view.columns[2].y);
    c.F = simd_make_float3(fu.view.columns[0].z, fu.view.columns[1].z, fu.view.columns[2].z);
    c.zn = fu.fog.z, c.q = fu.fog.w;
    c.valid = true;
}

const FrameUniforms* MeshRenderer::stableFrame() const { return stableFrame_ ? stableFrame_.get() : lastFrame_.get(); }
// ---- TAA end

// ---- P92: the headlight clusters of this frame's view (omniMask in the shaders), made by kOmniClusters on the GPU
// before the scene pass. Off (cluster.x = 0: every omni tried) without omnis or without a target size.
id<MTLBuffer> MeshRenderer::encodeOmniClusters(id<MTLCommandBuffer> cb, FrameUniforms& fu, float fovY, float aspect) {
    if (noMorph_ == nil) noMorph_ = [device_ newBufferWithLength:16 options:MTLResourceStorageModeShared];
    fu.cluster = fu.clusterZ = fu.clusterView = fu.camFwd = simd_make_float4(0, 0, 0, 0);
    const int n = std::min(64, int(fu.lightInfo.y));  // PK28: 64
    if (n <= 0 || viewW_ == 0 || viewH_ == 0 || cb == nil) return noMorph_;
    if (omniClusterPipeline_ == nil) {
        NSError* e = nil;
        omniClusterPipeline_ = [device_ newComputePipelineStateWithFunction:[shaders_ newFunctionWithName:@"kOmniClusters"] error:&e];
        if (omniClusterPipeline_ == nil) [NSException raise:@"pipeline" format:@"%@", e];
    }
    constexpr int kTile = 64, kSlices = 16;
    constexpr float kZ0 = 1.0f, kZ1 = 1024.0f;  // slice 0 ends at 1 m, slice 15 starts at 1024 m (and runs to 200 km)
    const int tx = int((viewW_ + kTile - 1) / kTile), ty = int((viewH_ + kTile - 1) / kTile);  // P93: the rendered size
    fu.cluster = simd_make_float4(float(kTile), float(tx), float(ty), float(kSlices));
    fu.clusterZ = simd_make_float4(kZ0, float(kSlices) / std::log2(kZ1 / kZ0), 200000.0f, 0);
    fu.clusterView = simd_make_float4(float(viewW_), float(viewH_), std::tan(fovY * 0.5f) * aspect, std::tan(fovY * 0.5f));
    fu.camFwd = simd_make_float4(simd_cross(fu.camUp.xyz, fu.camRight.xyz), 0);
    id<MTLBuffer> buf = [device_ newBufferWithLength:size_t(tx) * size_t(ty) * kSlices * sizeof(uint64_t) options:MTLResourceStorageModePrivate];  // PK28
    id<MTLComputeCommandEncoder> ce = gpuprof::compute(cb, "omni clusters");
    [ce setComputePipelineState:omniClusterPipeline_];
    [ce setBytes:&fu length:sizeof fu atIndex:0];
    [ce setBuffer:buf offset:0 atIndex:1];
    {  // PK28: the light rows
        NSUInteger n = 0;
        const void* rows = omniBytes(&n);
        [ce setBytes:rows length:n atIndex:2];
    }
    [ce dispatchThreads:MTLSizeMake(NSUInteger(tx), NSUInteger(ty), kSlices) threadsPerThreadgroup:MTLSizeMake(8, 8, 1)];
    [ce endEncoding];
    if (static const bool trace = std::getenv("FUEL_OMNI_TRACE") != nullptr; trace && frame_ % 60 == 0)
        std::printf("omni clusters: %d omnis, range %.1f m (light 0: value %g %g %g %g)\n", n, std::sqrt(omniRows_[2].x + 1.0f / omniRows_[2].y),
                    omniRows_[2].x, omniRows_[2].y, omniRows_[2].z, omniRows_[2].w);
    return buf;
}

// ---- p57: FUEL's global cube map. Scene_RenderShadowedViewport 0x758c70 calls 0x757c20 when the viewport has flag
// 0x40000 and the renderer's cube bitmap exists (vf+0xd0 = 0x837a80). 0x757c20: face = (scene+0x194 + 1) % 6, drawn
// alone unless renderer[0x43a] forces all six; per face a 90 degree (cot(pi/4) = 1) square projection with the
// camera's near / far, centred on the camera position, viewport 0,0,128,128 cleared to (0,0,0,1) (0xa064c8),
// render target 10 + face (renderer+0xd22), light type 7 -> 6 (no shadow map), GenWorld element mask
// W+0xb7474 = 0x10001f8 (kinds 3..8 = the ground chunks <= 30 m and the mid / far ground meshes, 24 = the sky dome;
// not trees 0..2/0x14..0x16, objects 0x17, water 0xe..0x10, the sun sprite 0x19) and scene flags 0x247 (bit 0x200:
// 0x752bf0 skips every scene object, vehicles and effects). Port: the sky (fsSky without the sun disc) and the
// port's terrain chunks (fsTerrain, unshadowed) inside each face's frustum; faces in Metal's cube convention, world
// axes, so the shaders sample with the world direction (FUEL flips z for D3D's left-handed cube in GetCubeMap).
void cubeFaceBasis(int face, simd_float3& F, simd_float3& R, simd_float3& U) {
    static const simd_float3 f[6] = {{1, 0, 0}, {-1, 0, 0}, {0, 1, 0}, {0, -1, 0}, {0, 0, 1}, {0, 0, -1}};
    static const simd_float3 r[6] = {{0, 0, -1}, {0, 0, 1}, {1, 0, 0}, {1, 0, 0}, {1, 0, 0}, {-1, 0, 0}};
    static const simd_float3 u[6] = {{0, 1, 0}, {0, 1, 0}, {0, 0, -1}, {0, 0, 1}, {0, 1, 0}, {0, 1, 0}};
    F = f[face], R = r[face], U = u[face];
}

void MeshRenderer::encodeCube(id<MTLCommandBuffer> cb, const FrameUniforms& fu0, const std::vector<std::shared_ptr<const TerrainChunk>>& chunks,
                              bool allFaces) {
    if (globalCube_ == nil) return;
    cubeFace_ = (cubeFace_ + 1) % 6;
    const simd_float3 eye = fu0.eye.xyz;
    const float zn = 0.5f, zf = 120000.0f;
    for (int face = 0; face < 6; ++face) {
        if (!allFaces && face != cubeFace_) continue;
        FrameUniforms fu = fu0;
        simd_float3 f, r, u;
        cubeFaceBasis(face, f, r, u);
        // View rows R, U, -F (a mirror: cube faces are left-handed), then FUEL's RH projection with fov 90 degrees.
        const simd_float3 b = -f;
        const simd_float4x4 view = simd_matrix(simd_make_float4(r.x, u.x, b.x, 0), simd_make_float4(r.y, u.y, b.y, 0),
                                               simd_make_float4(r.z, u.z, b.z, 0),
                                               simd_make_float4(-simd_dot(r, eye), -simd_dot(u, eye), -simd_dot(b, eye), 1));
        fu.viewProj = simd_mul(perspectiveRH(float(M_PI) * 0.5f, 1.0f, zn, zf), view);
        fu.invViewProj = simd_inverse(fu.viewProj);
        fu.camRight = simd_make_float4(r, 0);
        fu.camUp = simd_make_float4(u, 0);
        fu.fog.x = 5.0f;  // bit 0: no shadow lookups (light type 6), bit 2: the cube pass (no sun sprite)
        fu.fog.z = zn;
        fu.fog.w = zf / (zn - zf);

        MTLRenderPassDescriptor* rp = [MTLRenderPassDescriptor renderPassDescriptor];
        rp.colorAttachments[0].texture = globalCube_;
        rp.colorAttachments[0].slice = NSUInteger(face);
        rp.colorAttachments[0].loadAction = MTLLoadActionClear;
        rp.colorAttachments[0].clearColor = MTLClearColorMake(0, 0, 0, 1);
        rp.colorAttachments[0].storeAction = MTLStoreActionStore;
        rp.depthAttachment.texture = cubeDepth_;
        rp.depthAttachment.loadAction = MTLLoadActionClear;
        rp.depthAttachment.clearDepth = 1.0;
        rp.depthAttachment.storeAction = MTLStoreActionDontCare;
        gpuprof::tag(rp, "cube");  // P92: per-pass GPU timing (no-op unless profiling)
        id<MTLRenderCommandEncoder> enc = [cb renderCommandEncoderWithDescriptor:rp];
        enc.label = @"Cube-Map";
        [enc setVertexBytes:&fu length:sizeof fu atIndex:1];
        [enc setFragmentBytes:&fu length:sizeof fu atIndex:1];
        [enc setFragmentBuffer:noMorph_ offset:0 atIndex:5];  // P92: fsTerrain's cluster buffer (unused: cluster.x = 0 here)
        {  // PK28: the light rows
            NSUInteger n = 0;
            const void* rows = omniBytes(&n);
            [enc setFragmentBytes:rows length:n atIndex:6];
        }
        [enc setFragmentTexture:shadowMap_ atIndex:7];
        [enc setCullMode:MTLCullModeNone];
        [enc setRenderPipelineState:cubeSkyPipeline_];
        [enc setFragmentTexture:white_ atIndex:0];  // no sprite in the cube (kind 25 is not drawn there)
        [enc setDepthStencilState:depthAlways_];
        [enc drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:0 vertexCount:3];
        if (!chunks.empty() && terrainIndices_ != nil) {
            [enc setRenderPipelineState:cubeTerrainPipeline_];
            [enc setDepthStencilState:depthWrite_];
            [enc setFragmentTexture:heightTex_ atIndex:0];
            if (groundColour_ != nil) {
                [enc setFragmentTexture:groundColour_ atIndex:1];
                [enc setFragmentTexture:groundSurface_ atIndex:2];
                [enc setFragmentTexture:groundDetail_ atIndex:3];
                [enc setFragmentTexture:groundFine_ atIndex:4];
                [enc setFragmentTexture:groundDrawSet_ atIndex:5];  // P92: fsTerrain reads the draw sets (was the types)
                [enc setFragmentTexture:groundDecal_ atIndex:6];
                for (int k = 0; k < kDecalSlots; ++k)
                    [enc setFragmentTexture:decalTex_[k] != nil ? decalTex_[k] : white_ atIndex:NSUInteger(8 + k)];
            }
            for (const auto& c : chunks) {
                auto git = chunkGpu_.find(c->key);
                if (git == chunkGpu_.end() || git->second.vb == nil) continue;
                // The face's frustum is |d.R| <= d.F and |d.U| <= d.F: skip chunks whose box is outside one plane.
                ChunkBounds& cbnd = chunkBounds_[c->key];
                if (cbnd.maxY <= cbnd.minY) {
                    cbnd.minY = 1e30f, cbnd.maxY = -1e30f;
                    for (const TerrainVertex& v : c->verts) cbnd.minY = std::min(cbnd.minY, v.p0[1]), cbnd.maxY = std::max(cbnd.maxY, v.p0[1]);
                    cbnd.maxY = std::max(cbnd.maxY, cbnd.minY + 0.01f);
                }
                const float h = float(c->size) * 0.5f;
                bool out[4] = {true, true, true, true};
                for (int k = 0; k < 8; ++k) {
                    const simd_float3 d = simd_make_float3(float(c->cx) + ((k & 1) ? h : -h) - eye.x, ((k & 2) ? cbnd.maxY : cbnd.minY) - eye.y,
                                                           float(c->cz) + ((k & 4) ? h : -h) - eye.z);
                    const float df = simd_dot(d, f), dr = simd_dot(d, r), du = simd_dot(d, u);
                    out[0] = out[0] && df - dr < 0, out[1] = out[1] && df + dr < 0;
                    out[2] = out[2] && df - du < 0, out[3] = out[3] && df + du < 0;
                }
                if (out[0] || out[1] || out[2] || out[3]) continue;
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
}

// ---- p57: MaterialAnim_Z. MaterialManager_Z keeps every loaded anim (registered by MaterialAnim_Z::vf13 0x76a130
// -> 0x7ffbe0) and advances them each frame (0x7acfc0 -> 0x76a420); only anims whose state has 0x40 (playing) move.
// In the shipped data the auto-playing ones (state 0x4c) all scroll a texture (uvOffset tracks); the others are
// started by game code (not ported) or are static livery / FX-atlas records.
void MeshRenderer::scanMaterialAnims() {
    const auto& files = library_.files();
    for (; animFilesScanned_ < files.size(); ++animFilesScanned_) {
        const DpcFile& file = *files[animFilesScanned_];
        for (uint32_t id : file.ids(cls::MaterialAnimZ)) {
            try {
                const auto body = file.body(*file.find(id));
                if (!body || body->size() <= 69) continue;  // 69 bytes: no keys at all
                MaterialAnim a = parseMaterialAnimBody(*body);
                if ((a.state & 0x40) == 0 || a.empty() || animByMaterial_.count(a.material)) continue;
                auto hit = library_.find(a.material, cls::Material);
                if (!hit) continue;
                const auto mb = hit.file->body(*hit.entry);
                if (!mb) continue;
                AnimatedMaterial am;
                am.base = parseMaterialBody(*mb);
                am.clock.state = a.state;
                am.anim = std::move(a);
                if (std::getenv("FUEL_MAT_DEBUG"))
                    std::printf("material anim %08x -> material %08x state %02x period %.3f\n", id, am.anim.material, am.anim.state,
                                am.anim.period);
                animByMaterial_[am.anim.material] = int(materialAnims_.size());
                materialAnims_.push_back(std::move(am));
            } catch (const std::exception&) {
            }
        }
    }
}

void MeshRenderer::updateMaterialAnims(float dt) {
    for (AnimatedMaterial& a : materialAnims_) {
        if (!a.clock.advance(dt, a.anim.period) && a.evaluated) continue;
        a.evaluated = true;
        Material m = a.base;
        evaluateMaterialAnim(a.anim, a.clock.time, m);
        a.uv0 = simd_make_float4(m.uvMatrix[0], m.uvMatrix[1], m.uvMatrix[2], m.uvMatrix[3]);
        a.uv1 = simd_make_float2(m.uvMatrix[4], m.uvMatrix[5]);
        a.diffuse = simd_make_float4(m.diffuse[0], m.diffuse[1], m.diffuse[2], m.alpha);
        a.specular = simd_make_float4(m.specular[0], m.specular[1], m.specular[2], m.specular[3]);
        a.emissive = simd_make_float4(m.emissive[0], m.emissive[1], m.emissive[2], 0);
        if (!a.anim.texture.empty()) {
            uint8_t ignored = 0;
            a.texture = texture(m.textures[0], false, &ignored);
        }
    }
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
        lumOne_ = make(MTLPixelFormatR32Float, 1, 1, 1, MTLTextureUsageShaderRead | MTLTextureUsageShaderWrite);  // P95: kLumReduce's
        for (auto& t : bloomRt_) t = make(MTLPixelFormatRGBA8Unorm, 128, 128, 1, rw);
    }
    if (width == targetWidth_ && height == targetHeight_) return;
    targetWidth_ = width, targetHeight_ = height;
    hdrMsaa_ = make(kRenderColorFormat, width, height, sampleCount_, rt);
    depthMsaa_ = make(kRenderDepthFormat, width, height, sampleCount_, rt);
    hdr_ = make(kRenderColorFormat, width, height, 1, rw | targetExtraUsage_);    // TAA: + what the temporal scaler asks (0 otherwise)
    depth_ = make(kRenderDepthFormat, width, height, 1, rw | targetExtraUsage_);
    postFx_ = make(MTLPixelFormatRGBA8Unorm, width, height, 1, rw);
}

void MeshRenderer::render(id<MTLCommandBuffer> cb, id<MTLTexture> target, const OrbitCamera& cam) {
    if (!toOptimize_.empty() && !std::getenv("FUEL_NO_TEX_OPTIMIZE")) {  // P92: textures uploaded since the last frame
        id<MTLBlitCommandEncoder> blit = gpuprof::blit(cb, "tiles");
        for (id<MTLTexture> t : toOptimize_) [blit optimizeContentsForGPUAccess:t];
        [blit endEncoding];
    }
    toOptimize_.clear();
    if (frame_ == 0) epoch_ = std::chrono::steady_clock::now();  // the running clock starts with the first frame
    // P92: per-pass GPU timing of this frame (the caller ends it with gpuprof::endFrame) and the CPU timings.
    gpuprof::beginFrame();
    cpu_ = CpuTimes{};
    const auto tRender = std::chrono::steady_clock::now();
    ensureTargets(target.width, target.height);
    flushGroundTiles(cb);  // fine ground tiles staged since the last frame, before this frame samples them
    // P93: this frame's render size (dynamic resolution): the top-left viewW_ x viewH_ of the scene targets.
    float scale = chooseScale(target.width, target.height);
    const bool cut = cameraCut(cam);
    if (cut) taaReset_ = true, ++cutSerial_;  // TAA: a cut drops the temporal history (PHYSIQUE: and the rider's springs)
    cutFrame_ = cut && dynamicResolution && fixedRenderScale <= 0 && !std::getenv("FUEL_NO_CUT_SCALE");
    if (cutFrame_) {
        ++cutEpoch_;
        dynres_.cut();
        scale = std::min(scale, std::clamp(dynamicResolutionParams.minScale, 0.5f, 1.0f));
        if (scale < 1.0f) ensureUpscaler(target.width, target.height);
    }
    // TAA: every frame of the temporal upscaler goes through it (native ones too), while it runs at all.
    const bool wasTemporal = temporalFrame_;
    temporalFrame_ = upscaler == Upscaler::MetalFXTemporal && (dynamicResolution || fixedRenderScale > 0) && !instances_.empty() &&
                     ensureTemporal(target.width, target.height);
    if (temporalFrame_) scale = std::max(scale, taaMinScale_);
    if (temporalFrame_ && !wasTemporal) taaReset_ = true, taaPrev_.valid = false;  // (re)started: no history, no last frame
    scaledFrame_ = scale < 1.0f;
    lastScale_ = scaledFrame_ ? scale : 1.0f;
    if (!temporalFrame_ && scaledFrame_) ensureUpscaler(target.width, target.height, false);  // (no temporal frame: the sharpened bilinear)
    viewW_ = scaledFrame_ ? std::max<NSUInteger>(16, NSUInteger(std::lround(double(target.width) * scale))) : target.width;
    viewH_ = scaledFrame_ ? std::max<NSUInteger>(16, NSUInteger(std::lround(double(target.height) * scale))) : target.height;
    cpu_.renderScale = lastScale_;

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
    if (scaledFrame_) rp.renderTargetWidth = viewW_, rp.renderTargetHeight = viewH_;  // P93
    lastFrame_.reset();
    frameCb_ = cb;
    frameRp_ = rp;
    if (instances_.empty()) {
        gpuprof::tag(rp, "scene");  // P92: per-pass GPU timing (no-op unless profiling)
        id<MTLRenderCommandEncoder> enc = [cb renderCommandEncoderWithDescriptor:rp];
        applyView(enc);  // P93
        [enc endEncoding];
    } else {
        encode(nil, float(target.width) / float(std::max<NSUInteger>(target.height, 1)), cam);
    }
    frameCb_ = nil;
    frameRp_ = nil;

    auto tPart = std::chrono::steady_clock::now();
    if (lastFrame_) encodeDust(cb);
    if (lastFrame_) encodeLens(cb);  // PK24: the lens projections over everything drawn in the scene
    fxEnd();  // P92: closes the shared effects pass
    if (temporalFrame_) encodeTaaVelocity(cb);  // TAA: motion vectors and the reactive mask
    cpu_.effects = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - tPart).count();
    tPart = std::chrono::steady_clock::now();
    encodePost(cb, target);
    cpu_.post = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - tPart).count();
    cpu_.total = std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now() - tRender).count();
    // The one-off work since the last frame (set-up calls between frames included).
    cpu_.pipelines = oneOff_.pipelines - oneOffSeen_.pipelines, cpu_.textures = oneOff_.textures - oneOffSeen_.textures;
    cpu_.meshes = oneOff_.meshes - oneOffSeen_.meshes, cpu_.chunks = oneOff_.chunks - oneOffSeen_.chunks;
    oneOffSeen_ = oneOff_;
    if (temporalFrame_) taaEndFrame();  // TAA: this frame's camera and instances become the next one's "previous"
    ++frame_;
    const float frameScale = cutFrame_ ? -1.0f : lastScale_;  // P93: a cut's frame is not the controller's
    const uint32_t frameEpoch = cutEpoch_;
    [cb addCompletedHandler:^(id<MTLCommandBuffer> b) {
        if (b.GPUEndTime > b.GPUStartTime) gpuMs_ = (b.GPUEndTime - b.GPUStartTime) * 1000.0;
        if (b.GPUEndTime > b.GPUStartTime) {  // P93: the GPU time this frame added (not the wait for the frame before)
            std::lock_guard<std::mutex> lk(dynresMutex_);
            const double net = (b.GPUEndTime - std::max(b.GPUStartTime, dynresLastEnd_)) * 1000.0;
            dynresLastEnd_ = std::max(dynresLastEnd_, b.GPUEndTime);
            if (net > 0 && frameScale > 0 && dynresDone_.size() < 64) dynresDone_.push_back({net, frameScale, frameEpoch});
        }
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

// ---- P92: the effects passes (blob shadows, tyre tracks, particles, clouds, tornadoes, rain / snow, wind tanks, GPS
// arrows, splashes / bolts, dust) used to be one render pass each over the resolved HDR colour, every one loading and
// storing the tiles it touched (61 MB at 3420x2224 for a full-screen one). They draw in the same order into one pass
// now: the same draws, blending and state (each sets what it uses; none sets more than the cull mode it leaves at
// the default), and none reads the colour target, so the picture is the same. FUEL_FX_SPLIT=1 keeps one pass per
// effect (for per-effect GPU timing).
id<MTLRenderCommandEncoder> MeshRenderer::fxEncoder(id<MTLCommandBuffer> cb, const char* name) {
    static const bool split = std::getenv("FUEL_FX_SPLIT") != nullptr;
    if (fxEnc_ != nil && !split) return fxEnc_;
    fxEnd();
    MTLRenderPassDescriptor* rp = [MTLRenderPassDescriptor renderPassDescriptor];
    rp.colorAttachments[0].texture = hdr_;
    rp.colorAttachments[0].loadAction = MTLLoadActionLoad;
    rp.colorAttachments[0].storeAction = MTLStoreActionStore;
    if (scaledFrame_) rp.renderTargetWidth = viewW_, rp.renderTargetHeight = viewH_;  // P93
    gpuprof::tag(rp, split ? name : "fx");  // per-pass GPU timing (no-op unless profiling)
    fxEnc_ = [cb renderCommandEncoderWithDescriptor:rp];
    applyView(fxEnc_);  // P93
    return fxEnc_;
}

void MeshRenderer::fxEnd() {
    if (fxEnc_ != nil) [fxEnc_ endEncoding];
    fxEnc_ = nil;
}

void MeshRenderer::encodeDust(id<MTLCommandBuffer> cb) {
    if (sfxAtlas_ == nil) return;
    const auto now = std::chrono::steady_clock::now();
    float fxDt = haveFxWall_ ? std::min(0.25f, std::chrono::duration<float>(now - fxWall_).count()) : 0.0f;
    if (exposureDt >= 0) fxDt = exposureDt;
    if (pauseSky_) fxDt = 0;  // PK17: the weather FX, lens drops (PK24) and wind tanks are frozen while paused
    fxWall_ = now, haveFxWall_ = true;
    encodeBlobs(cb);  // P77: preset 3, before the blended presets
    encodeDecals(cb);
    encodeEngineParticles(cb);
    updateWeatherFx(fxDt);
    updateLens(fxDt);  // PK24
    updateSandFx(fxDt);  // PK12: STARTFX records + sand tank (0x5bc3b0, 0x5ecb20)
    encodeClouds(cb);
    encodeTornadoes(cb);  // P65
    encodeSandWalls(cb);  // PK12: the SandMesh walls (SubCode 0x14), with the FX manager's other meshes
    encodeTanks(cb);
    encodeWindTanks(cb, fxDt);  // P65
    encodeGpsRibbons(cb);       // P86
    encodeNavBeams(cb);         // PK19
    encodeWeatherFx(cb, fxDt);
    static const bool hideSprites = std::getenv("FUEL_SFX_HIDE") != nullptr;  // look at the marks under the dust
    if (dust_.sprites.empty() || hideSprites) {
        encodeVehicleFx(cb);  // PK23: SubCodes 0x15 / 0x16 draw after the wheel dust (0xb / 0xe)
        return;
    }
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

    id<MTLRenderCommandEncoder> enc = fxEncoder(cb, "fx.dust");  // P92: the shared effects pass (fxEncoder)
    [enc setRenderPipelineState:dustPipeline_];
    [enc setVertexBuffer:buf offset:0 atIndex:0];
    [enc setVertexBytes:lastFrame_.get() length:sizeof(FrameUniforms) atIndex:1];
    [enc setVertexBytes:&params length:sizeof params atIndex:2];
    [enc setVertexTexture:shadowMap_ atIndex:0];
    [enc setFragmentBytes:lastFrame_.get() length:sizeof(FrameUniforms) atIndex:1];
    [enc setFragmentTexture:depth_ atIndex:0];
    [enc setFragmentTexture:sfxAtlas_ atIndex:1];
    [enc drawPrimitives:MTLPrimitiveTypeTriangleStrip vertexStart:0 vertexCount:4 instanceCount:order.size()];
    encodeVehicleFx(cb);  // PK23
}

void MeshRenderer::lightningStrike(const std::array<float, 3>& groundPos) {
    std::lock_guard<std::mutex> lk(strikeMu_);
    strikes_.push_back(groundPos);
    externalThunder_ = true;
}

// p48: the splash tank (+0x13c) and the thunder tank (+0x140) as the weather apply 0x5bba30 drives them: the splash
// tank runs (vf+0xc(1024)) while it rains, its amount is the rain; the thunder's amount is the cloud cover (1 with
// the debug Force Rain, FUEL_FORCE_RAIN); both sit at the reference position (the camera).
void MeshRenderer::updateWeatherFx(float dt) {
    fxClock_ += dt;
    if (!weatherFields_ || !weatherFromWorld || !lastFrame_) return;
    const WeatherAmounts& w = weatherNow_;
    auto P = [&](int n) { return weatherFields_->P(n); };
    if (!wfxWell_) {
        std::array<uint32_t, 16> seed{};
        for (size_t i = 0; i < seed.size(); ++i) seed[i] = 0x9e3779b9u * uint32_t(i + 1) ^ 0x5bd1e995u;  // port state
        wfxWell_ = std::make_unique<sfx::Well512>(seed);
    }
    const wfx::RndInt well = [this]() { return int32_t(wfxWell_->next()); };
    const wfx::Rnd game = [this]() {
        wfxGame_ ^= wfxGame_ << 13, wfxGame_ ^= wfxGame_ >> 17, wfxGame_ ^= wfxGame_ << 5;
        return float(wfxGame_ >> 8) / 16777216.0f;
    };
    const wfx::Ground ground = [this](float x, float z) { return terrainData_.width > 0 ? terrainData_.heightAt(x, z) : 0.0f; };
    const simd_float3 eye = lastFrame_->eye.xyz;
    const wfx::V3 tankPos{eye.x, eye.y, eye.z};
    static const bool forceRain = std::getenv("FUEL_FORCE_RAIN") != nullptr;  // 0xa654d9: thunder amount 1 too
    // Splashes.
    if (!splash_) splash_ = std::make_unique<wfx::SplashTank>(wfx::SplashParams::fromPP(weatherFields_->pp()));
    if (w.rain > 0) {
        if (!splash_->started()) splash_->start(1024, well);
        splash_->setAmount(w.rain);
        splash_->update(dt, tankPos, game, ground);
        if (std::getenv("FUEL_HOLD_BOLTS"))  // shots only (headless films render once per frame shot): cycle the ring
            for (int k = 0; k < 120; ++k) splash_->update(1.0f / 60, tankPos, game, ground);
    } else if (splash_->started()) {
        splash_->setAmount(0);
    }
    // Thunder.
    if (!thunder_) {
        wfx::ThunderParams tp = wfx::ThunderParams::fromPP(weatherFields_->pp());
        if (std::getenv("FUEL_HOLD_BOLTS")) tp.lifeMin = tp.lifeMax = 1000.0f, tp.distMin = 600, tp.distMax = 900;  // shots only
        thunder_ = std::make_unique<wfx::ThunderTank>(tp);
    }
    thunder_->setAmount(forceRain ? 1.0f : w.cloud);
    {
        std::lock_guard<std::mutex> lk(strikeMu_);
        thunder_->external = externalThunder_;
        for (const auto& g : strikes_) thunder_->strike({g[0], g[1], g[2]});
        strikes_.clear();
    }
    const simd_float3 fwd = simd_cross(lastFrame_->camUp.xyz, lastFrame_->camRight.xyz);
    const float layer = cloudTank_ ? cloudTank_->layerHeight() : (P(5202) - P(5203)) * w.cloud + P(5203);  // cloud tank +0xf8
    thunder_->lastFlash = -1;
    if (std::getenv("FUEL_HOLD_BOLTS") && thunder_->storm())  // shots only: a ring of held bolts around the camera
        while (thunder_->bolts().size() < 40) thunder_->spawn(tankPos, layer, well, game, ground);
    thunder_->update(dt, tankPos, {fwd.x, fwd.y, fwd.z}, layer, well, game, ground, nullptr);
    if (thunder_->lastFlash > 0) flashAlpha_ = thunder_->lastFlash, flashTime_ = 0;
    else flashTime_ += dt;
    if (std::getenv("FUEL_SFX_DEBUG") && frame_ % 30 == 0)
        std::fprintf(stderr, "p48 weather fx: splashes %d, bolts %zu (drawn %d), storm %d, external %d, flash %.2f\n",
                     splash_->drawn(), thunder_->bolts().size(), thunder_->drawn(), int(thunder_->storm()), int(thunder_->external),
                     flashTime_ < 1 ? flashAlpha_ : 0.0f);
}

void MeshRenderer::encodeWeatherFx(id<MTLCommandBuffer> cb, float) {
    if (!lastFrame_) return;
    struct FxQuad {
        simd_float4 pos, uv, size, rgba;
    };
    struct FxParams {
        simd_float4 user0, mode;
    };
    std::vector<FxQuad> splashQ, boltQ;
    if (splash_ && splash_->drawn() > 0 && splashTex_ != nil && sfxTables_ && sfxTables_->size() > 9) {
        // 0x5e0bc0 corners: u 0 .. 0.0625 (one of 16 frames), v = atlas 9's rect; u swaps when +0x5c > 0.
        const auto& e = (*sfxTables_)[9];
        const wfx::SplashParams& sp = splash_->params();
        const int n = std::min(splash_->drawn(), int(splash_->splashes().size()));
        splashQ.reserve(size_t(n));
        for (int i = 0; i < n; ++i) {
            const wfx::Splash& q = splash_->splashes()[size_t(i)];
            const bool mirror = q.flip > 0;
            splashQ.push_back({simd_make_float4(q.pos[0], q.pos[1], q.pos[2], q.phase),
                               simd_make_float4(mirror ? 0.0625f : 0.0f, e.v0, mirror ? 0.0f : 0.0625f, e.v0 + e.dv),
                               simd_make_float4(sp.scale[0], sp.scale[1], 0, 0),
                               simd_make_float4(sp.colour[0], sp.colour[1], sp.colour[2], sp.colour[3])});
        }
    }
    if (thunder_ && boltTex_ != nil) {
        const float fog = weatherNow_.fog;  // weather +0xb4 -> particle +0x48 -> positionRotation.w
        for (const wfx::BoltQuad& b : wfx::boltQuads(*thunder_))
            boltQ.push_back({simd_make_float4(b.centre[0], b.centre[1], b.centre[2], fog), simd_make_float4(b.u0, 0, b.u1, 1),
                             simd_make_float4(b.halfW, b.halfH, 0, 0), simd_make_float4(b.rgb, b.rgb, b.rgb, b.alpha)});
    }
    const float flash = flashTime_ < 1.0f ? flashAlpha_ * (1.0f - flashTime_) : 0.0f;  // I: linear over the 1 s
    if (std::getenv("FUEL_SFX_DEBUG") && !boltQ.empty())
        std::fprintf(stderr, "p48 bolts: %zu quads, first at %.0f %.0f %.0f half %.0f x %.0f, uv %.2f..%.2f, a %.2f; eye %.0f %.0f %.0f\n", boltQ.size(),
                     boltQ[0].pos.x, boltQ[0].pos.y, boltQ[0].pos.z, boltQ[0].size.x, boltQ[0].size.y, boltQ[0].uv.x, boltQ[0].uv.z, boltQ[0].rgba.w,
                     lastFrame_->eye.x, lastFrame_->eye.y, lastFrame_->eye.z);
    if (splashQ.empty() && boltQ.empty() && flash <= 0) return;
    id<MTLRenderCommandEncoder> enc = fxEncoder(cb, "fx.weather");  // P92: the shared effects pass (fxEncoder)
    [enc setRenderPipelineState:fxPipeline_];
    [enc setVertexBytes:lastFrame_.get() length:sizeof(FrameUniforms) atIndex:1];
    [enc setFragmentBytes:lastFrame_.get() length:sizeof(FrameUniforms) atIndex:1];
    [enc setFragmentTexture:depth_ atIndex:0];
    auto draw = [&](const std::vector<FxQuad>& q, id<MTLTexture> tex, const FxParams& prm) {
        if (q.empty()) return;
        id<MTLBuffer> buf = [device_ newBufferWithBytes:q.data() length:q.size() * sizeof(FxQuad) options:MTLResourceStorageModeShared];
        [enc setVertexBuffer:buf offset:0 atIndex:0];
        [enc setVertexBytes:&prm length:sizeof prm atIndex:2];
        [enc setFragmentBytes:&prm length:sizeof prm atIndex:2];
        [enc setFragmentTexture:tex atIndex:1];
        [enc drawPrimitives:MTLPrimitiveTypeTriangleStrip vertexStart:0 vertexCount:4 instanceCount:q.size()];
    };
    if (splash_) {
        // C_USER4 = (FX clock [manager +0xd8], AnimTime PP5039, TextureSteps 16), C_USER3.zw = (0.0625, 0).
        static const bool magenta = std::getenv("FUEL_SFX_DEBUG") && std::atoi(std::getenv("FUEL_SFX_DEBUG")) == 6;
        draw(splashQ, splashTex_, FxParams{simd_make_float4(fxClock_, splash_->params().life, 16.0f, 0.0625f), simd_make_float4(0, magenta ? 1 : 0, 0, 0)});
    }
    draw(boltQ, boltTex_, FxParams{simd_make_float4(0, 1, 1, 0), simd_make_float4(1, 0, 0, 0)});
    if (flash > 0) {
        std::vector<FxQuad> f = {{simd_make_float4(0, 0, 0, 0), simd_make_float4(0), simd_make_float4(0), simd_make_float4(1, 1, 1, flash)}};
        draw(f, boltTex_ ? boltTex_ : sfxAtlas_, FxParams{simd_make_float4(0), simd_make_float4(2, 0, 0, 0)});
    }
}

// p48: tyre tracks (0x5df2c0 strips, decal.vhl / decal.phl) over the resolved scene.
void MeshRenderer::encodeBlobs(id<MTLCommandBuffer> cb) {
    static const bool off = std::getenv("FUEL_NO_BLOBS") != nullptr, trace = std::getenv("FUEL_BLOB_TRACE") != nullptr;
    if (trace) {
        std::printf("blobs: %zu records, texture %s, factor %.3f (inv %.3f fog %.3f), eye (%.1f %.1f %.1f)\n", dust_.blobs.size(), blobTex_ ? "ok" : "-",
                    lastFrame_ ? blob::alphaFactor(lastFrame_->lightDir.w, weatherNow_.fog) : 0.0f, lastFrame_ ? lastFrame_->lightDir.w : 0.0f,
                    weatherNow_.fog, lastFrame_ ? lastFrame_->eye.x : 0.0f, lastFrame_ ? lastFrame_->eye.y : 0.0f, lastFrame_ ? lastFrame_->eye.z : 0.0f);
        for (size_t i = 0; i < dust_.blobs.size() && i < 6; ++i) {
            const blob::Record& r = dust_.blobs[i];
            std::printf("  blob %zu at (%.2f %.2f %.2f) side %.2f fwd %.2f alpha %.2f\n", i, r.centre[0], r.centre[1], r.centre[2], r.side, r.fwd, r.alpha);
        }
    }
    if (off || dust_.blobs.empty() || blobTex_ == nil || !lastFrame_) return;
    // 0x5cb8c0's factor: clamp(1 - DLIGHT_INVINTENSITY, 0.3, 1) x clamp(1 - fog (WeatherManager +0xb4), 0.2, 1).
    const float k = blob::alphaFactor(lastFrame_->lightDir.w, weatherNow_.fog);
    struct Q {
        simd_float4 c, a, b;
    };
    std::vector<Q> qs;
    qs.reserve(dust_.blobs.size());
    for (const blob::Record& r : dust_.blobs)
        qs.push_back({simd_make_float4(r.centre[0], r.centre[1], r.centre[2], r.alpha * k), simd_make_float4(r.a[0], r.a[1], r.a[2], 0),
                      simd_make_float4(r.b[0], r.b[1], r.b[2], 0)});
    id<MTLBuffer> buf = [device_ newBufferWithBytes:qs.data() length:qs.size() * sizeof(Q) options:MTLResourceStorageModeShared];
    id<MTLRenderCommandEncoder> enc = fxEncoder(cb, "fx.blobs");  // P92: the shared effects pass (fxEncoder)
    [enc setRenderPipelineState:blobPipeline_];
    [enc setVertexBuffer:buf offset:0 atIndex:0];
    [enc setVertexBytes:lastFrame_.get() length:sizeof(FrameUniforms) atIndex:1];
    [enc setFragmentBytes:lastFrame_.get() length:sizeof(FrameUniforms) atIndex:1];
    [enc setFragmentTexture:depth_ atIndex:0];
    [enc setFragmentTexture:blobTex_ atIndex:1];
    static const bool debug = std::getenv("FUEL_BLOB_DEBUG") != nullptr;
    const simd_float4 opts = simd_make_float4(debug ? 1.0f : 0.0f, 0.35f, 0, 0);  // y: as encodeDecals
    [enc setFragmentBytes:&opts length:sizeof opts atIndex:2];
    [enc drawPrimitives:MTLPrimitiveTypeTriangleStrip vertexStart:0 vertexCount:4 instanceCount:qs.size()];
}

void MeshRenderer::encodeDecals(id<MTLCommandBuffer> cb) {
    if (dust_.decals.empty() || pneuDiffuse_ == nil || !lastFrame_) return;
    id<MTLBuffer> buf = [device_ newBufferWithBytes:dust_.decals.data() length:dust_.decals.size() * sizeof(sfx::DecalVertex)
                                            options:MTLResourceStorageModeShared];
    id<MTLRenderCommandEncoder> enc = fxEncoder(cb, "fx.decals");  // P92: the shared effects pass (fxEncoder)
    [enc setRenderPipelineState:decalPipeline_];
    [enc setVertexBuffer:buf offset:0 atIndex:0];
    [enc setVertexBytes:lastFrame_.get() length:sizeof(FrameUniforms) atIndex:1];
    [enc setFragmentBytes:lastFrame_.get() length:sizeof(FrameUniforms) atIndex:1];
    // x: bCompression (DXT5 normal map, wyz); y: how far behind the scene depth a mark still shows (metres): the
    // marks lie on the physics ground, the drawn terrain may sit a little above it.
    static const bool magenta = std::getenv("FUEL_SFX_DEBUG") && std::atoi(std::getenv("FUEL_SFX_DEBUG")) == 6;
    const simd_float4 opts = simd_make_float4(pneuCompressed_ ? 1.0f : 0.0f, 0.35f, magenta ? 1.0f : 0.0f, 0);
    [enc setFragmentBytes:&opts length:sizeof opts atIndex:2];
    [enc setFragmentTexture:depth_ atIndex:0];
    [enc setFragmentTexture:pneuDiffuse_ atIndex:1];
    [enc setFragmentTexture:pneuNormal_ ? pneuNormal_ : pneuDiffuse_ atIndex:2];
    [enc setFragmentTexture:shadowMap_ atIndex:3];
    uint32_t first = 0;
    for (uint32_t n : dust_.decalCounts) {
        [enc drawPrimitives:MTLPrimitiveTypeTriangleStrip vertexStart:first vertexCount:n];
        first += n;
    }
}

// p48: placed engine particles (Particles_Z), one draw per Material_Z; blended like the material says.
void MeshRenderer::encodeEngineParticles(id<MTLCommandBuffer> cb) {
    if (engineQuads_.empty() || !lastFrame_) return;
    id<MTLBuffer> buf = [device_ newBufferWithBytes:engineQuads_.data() length:engineQuads_.size() * sizeof(EngineParticle)
                                            options:MTLResourceStorageModeShared];
    id<MTLRenderCommandEncoder> enc = fxEncoder(cb, "fx.particles");  // P92: the shared effects pass (fxEncoder)
    [enc setVertexBuffer:buf offset:0 atIndex:0];
    [enc setVertexBytes:lastFrame_.get() length:sizeof(FrameUniforms) atIndex:1];
    [enc setFragmentBytes:lastFrame_.get() length:sizeof(FrameUniforms) atIndex:1];
    [enc setFragmentTexture:depth_ atIndex:0];
    for (const EngineBatch& b : engineBatches_) {
        auto it = particleMats_.find(b.material);
        if (it == particleMats_.end()) {
            // The element's material is a MaterialAnim_Z: its body +0x3c names the Material_Z (as FX_ATLAS_0).
            uint32_t matId = b.material;
            if (auto hit = library_.find(b.material, cls::MaterialAnim))
                if (const auto body = hit.file->body(*hit.entry); body && body->size() >= 0x40) std::memcpy(&matId, body->data() + 0x3c, 4);
            it = particleMats_.emplace(b.material, makeMaterial(matId)).first;
            if (std::getenv("FUEL_SFX_DEBUG")) {
                const GpuMaterial& m = it->second;
                const EngineParticle& q0 = engineQuads_[b.first];
                std::fprintf(stderr, "p48 particle material %08x: flags %08x fxAdditive %d code %u alpha %d tex %lux%lu fmt %lu diff %.2f %.2f %.2f %.2f emis %.2f %.2f %.2f; quad rgba %.2f %.2f %.2f %.2f size %.2f %.2f\n",
                             b.material, m.flags, m.fxAdditive ? 1 : 0, m.code, int(m.alpha), (unsigned long)m.textures[0].width, (unsigned long)m.textures[0].height,
                             (unsigned long)m.textures[0].pixelFormat, m.diffuse.x, m.diffuse.y, m.diffuse.z, m.diffuse.w, m.emissive.x,
                             m.emissive.y, m.emissive.z, q0.rgba.x, q0.rgba.y, q0.rgba.z, q0.rgba.w, q0.size.x, q0.size.y);
            }
        }
        const GpuMaterial& m = it->second;
        if (m.skip || b.count == 0) continue;
        // ---- PK28: the record's blend is its material's (0x856160 from the frame flush 0x858e60): flag 0x1000 -> SRCALPHA /
        // ONE, else SRCALPHA / INVSRCALPHA (p48's alpha-kind rule agreed on every shipped particle material seen).
        // 0x4000 / 0x2000 have no particle pipeline here (no shipped material has them).
        const bool additive = fuel::blendKind(m.flags) == fuel::BlendKind::Add;
        [enc setRenderPipelineState:additive ? particleAddPipeline_ : particlePipeline_];
        [enc setFragmentTexture:m.textures[0] atIndex:1];
        const simd_float4 tint = simd_make_float4(m.diffuse.xyz + m.emissive.xyz, m.diffuse.w);
        [enc setFragmentBytes:&tint length:sizeof tint atIndex:2];
        [enc setVertexBufferOffset:b.first * sizeof(EngineParticle) atIndex:0];
        [enc drawPrimitives:MTLPrimitiveTypeTriangleStrip vertexStart:0 vertexCount:4 instanceCount:b.count];
    }
}

// FUEL's billboard clouds (SpecialEffectsCloudTank_G, p37-sky-clouds.md): the tank follows the camera (vf+0x24
// 0x5fd890), refreshes every 10 s (0x5fe040), and draws its active clouds with vh_cloud (constants 0x5fe370).
void MeshRenderer::encodeClouds(id<MTLCommandBuffer> cb) {
    const auto now = std::chrono::steady_clock::now();
    float dt = haveCloudClock_ ? std::min(0.25f, std::chrono::duration<float>(now - cloudClock_).count()) : 0.0f;
    if (exposureDt >= 0) dt = exposureDt;
    if (pauseSky_) dt = 0;  // PK17: the cloud tank (manipulator) is not updated while paused
    cloudClock_ = now, haveCloudClock_ = true;
    if (!weatherFields_ || cloudTex_ == nil || !weatherFromWorld) return;
    const WeatherAmounts& w = weatherNow_;
    const float cloud = w.cloud;
    const float cloudness = w.cloudness;  // ---- PK04: 0x5b7e40 (clamp01(2 x 0x5bac10), or the race's +0xe4)
    if (!cloudTank_) {
        auto rnd = [this]() {  // port generator (FUEL: WELL512 0x5fd710 and 0x6460f0, shared with other systems)
            cloudRng_ ^= cloudRng_ << 13, cloudRng_ ^= cloudRng_ >> 17, cloudRng_ ^= cloudRng_ << 5;
            return float(cloudRng_ >> 8) / 16777216.0f;
        };
        cloudTank_ = std::make_unique<clouds::Tank>(clouds::Params::fromPP(weatherFields_->pp()), rnd, rnd);
    }
    const simd_float3 eye = lastFrame_->eye.xyz;
    cloudTank_->setPosition({eye.x, eye.y, eye.z}, cloudness);
    // ---- PK04: 0x5b7bb0 at race start: the tank's vf+0x10 (0x5fdd10) rebuilds at once with the race's cover.
    if (cloudRepopulate_) cloudTank_->populate(cloudness), cloudRepopulate_ = false;
    // ---- end PK04
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
        simd_float4 user0, flash[4], user5, user6, rgba;
    } params = {simd_make_float4(k[0], k[1], k[2], k[3]), {}, simd_make_float4(k[20], k[21], k[22], k[23]),
                simd_make_float4(k[24], k[25], k[26], k[27]), simd_make_float4(1.25f * p.colour, 1.25f * p.colour, 1.25f * p.colour, 0.5f)};
    if (thunder_) {  // C_USER1..4: the thunder tank's flashes (0x5e32a0 -> cloud +0xa4 / +0xd4)
        const auto fl = thunder_->flashes();
        for (int i = 0; i < 4; ++i) params.flash[i] = simd_make_float4(fl[size_t(i)].pos[0], fl[size_t(i)].pos[1], fl[size_t(i)].pos[2], fl[size_t(i)].strength);
    }
    id<MTLRenderCommandEncoder> enc = fxEncoder(cb, "fx.clouds");  // P92: the shared effects pass (fxEncoder)
    [enc setRenderPipelineState:cloudPipeline_];
    [enc setVertexBuffer:cloudBuf_ offset:0 atIndex:0];
    [enc setVertexBytes:lastFrame_.get() length:sizeof(FrameUniforms) atIndex:1];
    [enc setVertexBytes:&params length:sizeof params atIndex:2];
    [enc setFragmentBytes:lastFrame_.get() length:sizeof(FrameUniforms) atIndex:1];
    [enc setFragmentTexture:depth_ atIndex:0];
    [enc setFragmentTexture:cloudTex_ atIndex:1];
    [enc drawPrimitives:MTLPrimitiveTypeTriangleStrip vertexStart:0 vertexCount:4 instanceCount:NSUInteger(n)];
}

// FUEL's rain and snow tanks (SpecialEffectsRainTank_G 0x5e1920 / SnowTank_G 0x5dfbd0, started by the weather apply
// 0x5bba30 with 1024 x 12 rain drops and 12000 flakes; drawn count = floor(amount x N) 0x5de850).
void MeshRenderer::encodeTanks(id<MTLCommandBuffer> cb) {
    const WeatherAmounts& w = weatherNow_;
    if (std::getenv("FUEL_SFX_DEBUG") && frame_ % 30 == 0)
        std::fprintf(stderr, "weather at camera: cloud %.2f rain %.2f snow %.2f fog %.2f wind %.1f (%.2f, %.2f) humidity %.2f clouds %d\n", w.cloud,
                     w.rain, w.snow, w.fog, w.windSpeed, wind_.x, wind_.z, w.humidity, cloudTank_ ? cloudTank_->activeCount() : -1);
    if (weatherFields_ && !pauseSky_) {  // 0x5b8680: the field wind with gusts (WELL512 draws; port generator), on the game clock
        windClock_ += exposureDt >= 0 ? exposureDt : 1.0 / 60.0;  // PK17: the weather manager is not updated while paused
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
    if (pauseSky_) dt = 0;  // PK17: the rain / snow tanks are not updated while paused
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
    id<MTLRenderCommandEncoder> enc = fxEncoder(cb, "fx.tanks");  // P92: the shared effects pass (fxEncoder)
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
}

void MeshRenderer::loadSfxAtlas(const DpcFile& world) {
    // FX_ATLAS_0: MaterialAnim_Z (link = name CRC) in DATAS/SHARED.DPC -> body + 0x3c Material_Z -> slot 0 bitmap.
    namespace fs = std::filesystem;
    const fs::path shared = fs::path(world.path()).parent_path().parent_path() / "DATAS" / "SHARED.DPC";
    try {
        if (!fs::exists(shared)) return;
        sfxShared_ = std::make_unique<DpcFile>(shared.string());
        const DpcFile& d = *sfxShared_;
        auto load = [&](const char* nameStr, int slot = 0, bool* dxt5 = nullptr) -> id<MTLTexture> {
            const uint32_t want = asoboCrc(nameStr);
            for (uint32_t oid : d.ids(cls::MaterialAnim)) {
                const DpcFile::Entry* e = d.find(oid);
                const std::vector<uint8_t> link = d.link(*e);
                uint32_t name = 0;
                if (link.size() >= 4) std::memcpy(&name, link.data(), 4);
                if (name != want) continue;
                const auto body = d.body(*e);
                if (!body) return nil;
                uint32_t matId = 0;
                if (body->size() >= 0x40) std::memcpy(&matId, body->data() + 0x3c, 4);
                if (d.find(matId) == nullptr) {  // P77: FAKESHADOW's material sits after its tracks (+0x5d)
                    try {
                        matId = parseMaterialAnimBody(*body).material;
                    } catch (const std::exception&) {
                        return nil;
                    }
                }
                const DpcFile::Entry* me = d.find(matId);
                if (!me) return nil;
                const Material mat = parseMaterialBody(d.body(*me).value());
                const DpcFile::Entry* be = d.find(mat.textures[size_t(slot)]);
                if (!be) return nil;
                uint8_t alphaKind = 0;
                id<MTLTexture> t = uploadBitmap(d, *be, false, &alphaKind);
                if (dxt5 && t) *dxt5 = t.pixelFormat == MTLPixelFormatBC3_RGBA;
                if (std::getenv("FUEL_SFX_DEBUG"))
                    std::fprintf(stderr, "sfx %s: material %08x bitmap %08x -> %s\n", nameStr, matId, mat.textures[0], t ? "loaded" : "failed");
                return t;
            }
            return nil;
        };
        sfxAtlas_ = load("FX_ATLAS_0");
        blobTex_ = load("FAKESHADOW");  // P77: g_Game+0x3310 obj +0xde4 (MaterialAnim 0x76f68eb8 -> Material 0xa23e1f7a)
        cloudTex_ = load("FX_BILLBOARD_CLOUDS");  // the cloud tank's atlas entry 14 (whole texture)
        // p48: the tracks' FX_PNEU (atlas 15: diffuse slot 0, normal slot 3), the splash tank's atlas 9 and the
        // thunder's atlas 0x19 materials (names from the FX atlas replay).
        auto atlasName = [&](size_t i) -> std::string {
            return sfxTables_ && i < sfxTables_->size() ? (*sfxTables_)[i].material : std::string();
        };
        const std::string pneu = atlasName(15), splash = atlasName(9), bolt = atlasName(0x19);
        if (!pneu.empty()) pneuDiffuse_ = load(pneu.c_str()), pneuNormal_ = load(pneu.c_str(), 3, &pneuCompressed_);
        if (!splash.empty()) splashTex_ = load(splash.c_str());
        if (!bolt.empty()) boltTex_ = load(bolt.c_str());
        // ---- PK24: the lens projections' materials (FX atlas 30..41: dirt, mud, wheel splash, rain drop maps)
        for (size_t i = 0; i < lensTex_.size(); ++i) {
            const std::string name = atlasName(30 + i);
            if (name.empty()) continue;
            bool reused = false;
            for (size_t j = 0; j < i; ++j)
                if (atlasName(30 + j) == name) lensTex_[i] = lensTex_[j], reused = true;
            if (reused) continue;
            lensTex_[i].diffuse = load(name.c_str());
            lensTex_[i].normal = load(name.c_str(), 3, &lensTex_[i].compressed);
            if (std::getenv("FUEL_SFX_DEBUG"))
                std::fprintf(stderr, "pk24 lens texture %zu %s: diffuse %s normal %s (dxt5 %d)\n", 30 + i, name.c_str(), lensTex_[i].diffuse ? "ok" : "-",
                             lensTex_[i].normal ? "ok" : "-", int(lensTex_[i].compressed));
        }
        // ---- PK24 end
        // P65: the tornado (SpecialEffectsTornadoMesh_G / CloudTop_G materials, the base smoke's atlas entry 6).
        tornadoTex_[0] = load("TORNADE_CYLINDRE"), tornadoTex_[1] = load("TORNADE_CYLINDRE", 3, &tornadoNormalBC3_[0]);
        tornadoTex_[2] = load("FX_TORNADEHAUT"), tornadoTex_[3] = load("FX_TORNADEHAUT", 3, &tornadoNormalBC3_[1]);
        if (const std::string smoke = atlasName(6); !smoke.empty()) tornadoTex_[4] = load(smoke.c_str());
        if (const std::string wt = atlasName(8); !wt.empty()) windTankTex_[0] = load(wt.c_str());      // wind tank
        if (const std::string dt = atlasName(0x1c); !dt.empty()) windTankTex_[1] = load(dt.c_str());  // tornado debris tank
        if (const std::string ga = atlasName(0x15); !ga.empty()) gpsArrowTex_ = load(ga.c_str());     // P86: GPS arrows
        // ---- PK12: the sand-storm walls' material (SandMesh vf04 0x5ea5f0: "FX_SANDSTORM01" 0x9c4204), slots 0 and 3
        sandTex_[0] = load("FX_SANDSTORM01"), sandTex_[1] = load("FX_SANDSTORM01", 3, &sandNormalBC3_);
        if (std::getenv("FUEL_SFX_DEBUG"))
            std::fprintf(stderr, "pk12 textures: FX_SANDSTORM01 %s/%s (dxt5 normal %d)\n", sandTex_[0] ? "ok" : "-", sandTex_[1] ? "ok" : "-", int(sandNormalBC3_));
        // ---- PK12 end
        // ---- PK23: the spark streaks' and embers' atlas 0 (FX_SPARKLE); the smoke's atlas 6 is tornadoTex_[4], the flames' 0x30 FX_ATLAS_0
        if (const std::string sp = atlasName(0); !sp.empty()) sparkleTex_ = load(sp.c_str());
        // ---- PK23 end
        // PK19: the navigator beams' FX_NAVIGATOR3D_LINE (atlas 0x1d, names 0x8e6ae0). beam-colour: its atlas rect
        // (u0 1, du -1) is not used: ribbons keep their own uv (tornado_render.mm fsNavBeam).
        if (const std::string nl = atlasName(0x1d); !nl.empty()) {
            navLineTex_ = load(nl.c_str());
            if (std::getenv("FUEL_SFX_DEBUG")) {
                const auto& e = (*sfxTables_)[0x1d];
                std::fprintf(stderr, "pk19 nav line texture %s (atlas rect %.2f %.2f %.2f %.2f, unused)\n", nl.c_str(), e.u0, e.v0, e.du, e.dv);
            }
        }
        if (std::getenv("FUEL_SFX_DEBUG"))
            std::fprintf(stderr, "p65 textures: cylinder %s/%s (dxt5 %d), cloud top %s/%s (dxt5 %d), smoke %s %s\n", tornadoTex_[0] ? "ok" : "-",
                         tornadoTex_[1] ? "ok" : "-", int(tornadoNormalBC3_[0]), tornadoTex_[2] ? "ok" : "-", tornadoTex_[3] ? "ok" : "-",
                         int(tornadoNormalBC3_[1]), atlasName(6).c_str(), tornadoTex_[4] ? "ok" : "-");
        if (std::getenv("FUEL_SFX_DEBUG"))
            std::fprintf(stderr, "p48 textures: %s %s/%s (dxt5 normal %d), %s %s, %s %s\n", pneu.c_str(), pneuDiffuse_ ? "ok" : "-",
                         pneuNormal_ ? "ok" : "-", int(pneuCompressed_), splash.c_str(), splashTex_ ? "ok" : "-", bolt.c_str(), boltTex_ ? "ok" : "-");
    } catch (const std::exception& e) {
        std::fprintf(stderr, "fuelview: no FX atlas: %s\n", e.what());
    }
}

// P102: settings.txt `soften` value -> tent mix (renderer.h).
float MeshRenderer::softenAmount(const std::string& v, float fallback) {
    if (v == "off" || v == "0" || v == "no" || v == "false") return 0.0f;
    if (v == "light" || v == "on") return kSoftenLight;
    if (v == "strong") return kSoftenStrong;
    char* end = nullptr;
    const float f = std::strtof(v.c_str(), &end);
    if (end != v.c_str() && *end == 0 && std::isfinite(f)) return std::clamp(f, 0.0f, 1.0f);
    return fallback;
}

// P102: settings.txt `film_grain` value -> amplitude (renderer.h).
float MeshRenderer::filmGrainAmplitude(const std::string& v, float fallback) {
    if (v == "off" || v == "0" || v == "no" || v == "false") return 0.0f;
    if (v == "light" || v == "on") return kFilmGrainLight;
    if (v == "strong") return kFilmGrainStrong;
    char* end = nullptr;
    const float f = std::strtof(v.c_str(), &end);
    if (end != v.c_str() && *end == 0 && std::isfinite(f)) return std::clamp(f, 0.0f, 0.5f);
    return fallback;
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
    if (tintOverride.w >= 0) pu.tint = tintOverride;  // P67: the race-end grade (MissionHUD 0x4b6050)
    pu.fade = simd_make_float4(screenFade.x, screenFade.y, screenFade.z, std::clamp(screenFade.w, 0.0f, 1.0f));
    // gMotionBlurSettings0 from MinRadius 0, MaxRadius 1, FallOff 6 (0x64ff38), scale = viewport +0x1dc.
    const float blur = !postEffects ? 0.0f : speedBlurAbsolute >= 0 ? speedBlurAbsolute : std::clamp(speedBlur, 0.0f, 1.0f) * speedBlurMax;
    pu.mb0 = simd_make_float4(1.0f, 0.0f, 6.0f, blur);
    pu.texel = simd_make_float4(0, 0, 1.0f / w, 1.0f / h);
    // P93: the part of the scene targets this frame drew (uv scale) and the largest uv the post chain reads from them
    // (half a texel inside the drawn part: no bilinear tap reaches the stale rest). At native (1, 1, 2, 2): unchanged uv.
    pu.view = scaledFrame_ ? simd_make_float4(float(viewW_) / w, float(viewH_) / h, (float(viewW_) - 0.5f) / w, (float(viewH_) - 0.5f) / h)
                           : simd_make_float4(1, 1, 2, 2);
    const bool srgbOut = target.pixelFormat == MTLPixelFormatBGRA8Unorm_sRGB || target.pixelFormat == MTLPixelFormatRGBA8Unorm_sRGB;
    pu.flags = simd_make_float4(0, postEffects && postDefaults_.fromExe ? 1.0f : 0.0f, srgbOut ? 1.0f : 0.0f, 0);
    // ---- P102 [DEVIATION]: film grain on the last pass into the output (u.grain.x 0 on the intermediates), with a new
    // seed every frame and one grain cell per 1/1080 of the output height (FUEL_FILM_GRAIN=off|light|strong|<amplitude>
    // overrides the setting for one run).
    {
        float amp = filmGrain;
        if (const char* e = std::getenv("FUEL_FILM_GRAIN")) amp = filmGrainAmplitude(e, amp);
        float soft = soften;
        if (const char* e = std::getenv("FUEL_SOFTEN")) soft = softenAmount(e, soft);
        grainNow_ = amp > 0 ? simd_make_float4(amp, float(grainFrame_++ & 4095), std::max(1.0f, h / 1080.0f), 0) : simd_make_float4(0, 0, 1, 0);
        pu.grain = grainNow_;
        pu.grain.w = soft;  // the tone map's soften (grainNow_.w stays 0: the upscale does not soften again)
    }
    // ---- end P102
    float mbw[8];
    motionBlurWeights(0.95f, mbw);  // WeightDecrease [0x9dc5cc]
    for (int i = 0; i < 8; ++i) pu.mbw[i] = simd_make_float4(mbw[i], 0, 0, 0);
    float goff[11], gw[11];
    gaussian11(goff, gw);

    auto pass = [&](id<MTLTexture> dst, id<MTLRenderPipelineState> ps, std::initializer_list<id<MTLTexture>> textures, const PostUniforms& u,
                    const char* name) {
        MTLRenderPassDescriptor* rp = [MTLRenderPassDescriptor renderPassDescriptor];
        rp.colorAttachments[0].texture = dst;
        rp.colorAttachments[0].loadAction = MTLLoadActionDontCare;
        rp.colorAttachments[0].storeAction = MTLStoreActionStore;
        const bool scaledDst = scaledFrame_ && !temporalFrame_ && (dst == postFx_ || dst == lowOut_);  // P93: the scaled frame's own size (TAA: output size)
        if (scaledDst) rp.renderTargetWidth = viewW_, rp.renderTargetHeight = viewH_;
        gpuprof::tag(rp, name);  // P92
        id<MTLRenderCommandEncoder> enc = [cb renderCommandEncoderWithDescriptor:rp];
        if (scaledDst) applyView(enc);
        [enc setRenderPipelineState:ps];
        [enc setFragmentBytes:&u length:sizeof u atIndex:0];
        NSUInteger slot = 0;
        for (id<MTLTexture> t : textures) [enc setFragmentTexture:t atIndex:slot++];
        [enc drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:0 vertexCount:3];
        [enc endEncoding];
    };
    // 1-3: shrink, log luminance, reductions to 1 x 1.
    id<MTLTexture> lumAvg = lum_.back();
    if (!(p95Off & P95LumCompute) && shrinkLumPipeline_ != nil && lumReducePipeline_ != nil) {
        // P95: shrink + log luminance in one pass (two targets), the reductions in one compute pass (same values).
        MTLRenderPassDescriptor* rp = [MTLRenderPassDescriptor renderPassDescriptor];
        rp.colorAttachments[0].texture = shrink_, rp.colorAttachments[1].texture = lum_[0];
        for (int k = 0; k < 2; ++k) rp.colorAttachments[k].loadAction = MTLLoadActionDontCare, rp.colorAttachments[k].storeAction = MTLStoreActionStore;
        gpuprof::tag(rp, "post.lum");
        id<MTLRenderCommandEncoder> enc = [cb renderCommandEncoderWithDescriptor:rp];
        [enc setRenderPipelineState:shrinkLumPipeline_];
        [enc setFragmentBytes:&pu length:sizeof pu atIndex:0];
        [enc setFragmentTexture:hdr_ atIndex:0];
        [enc drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:0 vertexCount:3];
        [enc endEncoding];
        id<MTLComputeCommandEncoder> ce = gpuprof::compute(cb, "post.lum");
        [ce setComputePipelineState:lumReducePipeline_];
        [ce setTexture:lum_[0] atIndex:0];
        [ce setTexture:lumOne_ atIndex:1];
        [ce dispatchThreadgroups:MTLSizeMake(1, 1, 1)
            threadsPerThreadgroup:MTLSizeMake(std::min<NSUInteger>(1024, lumReducePipeline_.maxTotalThreadsPerThreadgroup), 1, 1)];
        [ce endEncoding];
        lumAvg = lumOne_;
    } else {
        pass(shrink_, shrinkPipeline_, {hdr_}, pu, "post.lum");  // P93: pu.view (the scaled frame's part)
        pass(lum_[0], lumLogPipeline_, {hdr_}, pu, "post.lum");
        for (size_t k = 1; k < lum_.size(); ++k) {
            PostUniforms u = pu;
            u.texel.x = 1.0f / float(lum_[k - 1].width), u.texel.y = 1.0f / float(lum_[k - 1].height);
            pass(lum_[k], lumAvgPipeline_, {lum_[k - 1]}, u, "post.lum");
        }
    }
    // 4: adaptation. The first frame initialises (S = SCREENLUMINANCE / e^A) and, so that single shots
    // show the settled exposure, then takes one step with k = 0 (port).
    auto adapt = [&](bool init, float k) {
        PostUniforms u = pu;
        u.flags.x = init ? 1.0f : 0.0f;
        u.hdr.x = k;
        const int prev = adaptedIndex_;
        adaptedIndex_ ^= 1;
        pass(adapted_[size_t(adaptedIndex_)], adaptPipeline_, {lumAvg, adapted_[size_t(prev)]}, u, "post.lum");
    };
    if (adaptInit_) {
        adapt(true, pu.hdr.x);
        adapt(false, 0.0f);
        adaptInit_ = false;
    } else {
        adapt(false, pu.hdr.x);
    }
    id<MTLTexture> S = adapted_[size_t(adaptedIndex_)];
    // FUEL_EXPOSURE_TRACE=1 (debug): print the frame's mean log luminance A, the adapted exposure S and the
    // per-frame inputs (I, SCREENLUMINANCE, clamps, blue shift w) once the GPU has run the chain.
    if (std::getenv("FUEL_EXPOSURE_TRACE")) {
        static id<MTLBuffer> rb = [device_ newBufferWithLength:16 options:MTLResourceStorageModeShared];
        id<MTLBlitCommandEncoder> be = [cb blitCommandEncoder];
        [be copyFromTexture:lumAvg sourceSlice:0 sourceLevel:0 sourceOrigin:MTLOriginMake(0, 0, 0) sourceSize:MTLSizeMake(1, 1, 1)
                   toBuffer:rb destinationOffset:0 destinationBytesPerRow:4 destinationBytesPerImage:4];
        [be copyFromTexture:S sourceSlice:0 sourceLevel:0 sourceOrigin:MTLOriginMake(0, 0, 0) sourceSize:MTLSizeMake(1, 1, 1)
                   toBuffer:rb destinationOffset:4 destinationBytesPerRow:4 destinationBytesPerImage:4];
        [be endEncoding];
        const float I = a.intensity, sl = a.screenLuminance, mn = a.minExposure, mx = a.maxExposure, bw = a.blueShift[3];
        const double tod = timeOfDay();
        [cb addCompletedHandler:^(id<MTLCommandBuffer>) {
            const float* v = static_cast<const float*>(rb.contents);
            std::printf("exposure: tod %.0f s A %.4f (e^A %.4f) S %.4f I %.4f SL %.4f clamp [%.4f, %.4f] w %.3f\n", tod, v[0],
                        std::exp(v[0]), v[1], I, sl, mn, mx, bw);
        }];
    }
    // 5-7: bright pass, gaussian H / V, the two star streaks.
    if (pu.flags.y > 0) {
        pass(bloomRt_[0], brightPipeline_, {shrink_, S}, pu, "post.bloom");
        auto gauss = [&](id<MTLTexture> src, id<MTLTexture> dst, float dx, float dy) {
            PostUniforms u = pu;
            for (int k = 0; k < 11; ++k) u.taps[k] = simd_make_float4(goff[k] / 128.0f * dx, goff[k] / 128.0f * dy, gw[k], 0);
            pass(dst, gaussPipeline_, {src}, u, "post.bloom");
        };
        gauss(bloomRt_[0], bloomRt_[1], 1, 0);
        gauss(bloomRt_[1], bloomRt_[0], 0, 1);
        gauss(bloomRt_[0], bloomRt_[1], 3, 3);
        gauss(bloomRt_[1], bloomRt_[2], 6, 6);
        gauss(bloomRt_[0], bloomRt_[1], -3, 3);
        gauss(bloomRt_[1], bloomRt_[3], -6, 6);
    }
    // 8-9: tone map, then the radial speed blur when it is on (viewport flag 0x8000).
    id<MTLTexture> hdrSrc = hdr_;
    if (temporalFrame_) {  // TAA: the HDR scene through MetalFX temporal to the output size, then the native passes from it
        encodeTemporal(cb, S);
        hdrSrc = upHdr_;
        pu.view = simd_make_float4(1, 1, 2, 2);  // the output-size source: unchanged uv (as native)
    } else if (scaledFrame_) {  // P93: the same passes into the scaled frame (display values, no sRGB curve), then upscaled
        PostUniforms lo = pu;
        lo.flags.z = 0;
        lo.grain.x = 0;  // P102: the grain goes on in the upscale pass, at the output's pixels
        if (blur > 0) {
            PostUniforms u = lo;
            u.fade = simd_make_float4(0, 0, 0, 0);
            pass(postFx_, toneMapFxPipeline_, {hdr_, S, bloomRt_[0], bloomRt_[2], bloomRt_[3], grainTex_}, u, "post.tonemap");
            pass(lowOut_, radialLowPipeline_, {postFx_, grainTex_}, lo, "post.radial");
        } else {
            pass(lowOut_, toneMapFxPipeline_, {hdr_, S, bloomRt_[0], bloomRt_[2], bloomRt_[3], grainTex_}, lo, "post.tonemap");
        }
        encodeUpscale(cb, target);
        return;
    }
    if (blur > 0) {
        PostUniforms u = pu;
        u.flags.z = 0;  // the intermediate stores the display value as is
        u.fade = simd_make_float4(0, 0, 0, 0);  // the fade goes on once, in the radial pass
        u.grain.x = 0;  // P102: the grain too
        pass(postFx_, toneMapFxPipeline_, {hdrSrc, S, bloomRt_[0], bloomRt_[2], bloomRt_[3], grainTex_}, u, "post.tonemap");
        pass(target, radialPipeline_, {postFx_, grainTex_}, pu, "post.radial");
    } else {
        pass(target, toneMapPipeline_, {hdrSrc, S, bloomRt_[0], bloomRt_[2], bloomRt_[3], grainTex_}, pu, "post.tonemap");
    }
}

// ---- P93: dynamic resolution.
// A camera cut: the eye jumps more than 5 m in a frame (300 m/s at 60 fps; no vehicle moves the chase camera that far) or
// the view turns more than 30 degrees. Only explicit poses (the game's chase / cut-scene cameras) count.
bool MeshRenderer::cameraCut(const OrbitCamera& cam) {
    if (!cam.explicitPose) {
        haveCutCam_ = false;
        return false;
    }
    const simd_float3 fwd = simd_normalize(cam.target - cam.eye);
    const bool cut = haveCutCam_ && (simd_distance(cam.eye, cutEye_) > 5.0f || simd_dot(fwd, cutFwd_) < 0.866f);
    cutEye_ = cam.eye, cutFwd_ = fwd, haveCutCam_ = true;
    return cut;
}

bool MeshRenderer::metalFxAvailable() const {
    if (@available(macOS 13.0, *)) return [MTLFXSpatialScalerDescriptor supportsDevice:device_];
    return false;
}

void MeshRenderer::applyView(id<MTLRenderCommandEncoder> enc) const {
    if (!scaledFrame_ || enc == nil) return;
    [enc setViewport:(MTLViewport){0, 0, double(viewW_), double(viewH_), 0, 1}];
}

// The upscale targets and the MetalFX scaler for an output of width x height (made when dynamic resolution or a fixed
// scale is first used at that size).
void MeshRenderer::ensureUpscaler(NSUInteger width, NSUInteger height, bool allowTemporal) {
    if (allowTemporal && upscaler == Upscaler::MetalFXTemporal && ensureTemporal(width, height)) return;  // TAA: no display-value targets
    if (lowOut_ != nil && lowOut_.width == width && lowOut_.height == height && builtUpscaler_ == upscaler) return;
    builtUpscaler_ = upscaler;  // P101: the GRAPHICS page switches it live
    auto make = [&](MTLTextureUsage usage) {
        MTLTextureDescriptor* td = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatRGBA8Unorm
                                                                                      width:width
                                                                                     height:height
                                                                                  mipmapped:NO];
        td.usage = usage;
        td.storageMode = MTLStorageModePrivate;
        return [device_ newTextureWithDescriptor:td];
    };
    fxScaler_ = nil;
    MTLTextureUsage lowUsage = MTLTextureUsageRenderTarget | MTLTextureUsageShaderRead, upUsage = MTLTextureUsageShaderRead;
    static const bool noFx = std::getenv("FUEL_NO_METALFX") != nullptr;
    if (upscaler == Upscaler::MetalFX && !noFx && metalFxAvailable()) {
        if (@available(macOS 13.0, *)) {
            MTLFXSpatialScalerDescriptor* d = [MTLFXSpatialScalerDescriptor new];
            d.colorTextureFormat = MTLPixelFormatRGBA8Unorm;
            d.outputTextureFormat = MTLPixelFormatRGBA8Unorm;
            d.inputWidth = width, d.inputHeight = height;  // the content size is set per frame (inputContentWidth)
            d.outputWidth = width, d.outputHeight = height;
            d.colorProcessingMode = MTLFXSpatialScalerColorProcessingModePerceptual;  // display values
            id<MTLFXSpatialScaler> sc = [d newSpatialScalerWithDevice:device_];
            if (sc != nil) {
                fxScaler_ = sc;
                lowUsage |= sc.colorTextureUsage, upUsage |= sc.outputTextureUsage;
            }
        }
    }
    if (!fxScalerTried_)
        std::fprintf(stderr, "fuelview: dynamic resolution: %s\n", fxScaler_ == nil ? "MetalFX unavailable, sharpened bilinear upscaler"
                                                                   : upscaler == Upscaler::MetalFX ? "MetalFX spatial upscaler" : "sharpened bilinear upscaler");
    fxScalerTried_ = true;
    lowOut_ = make(lowUsage);
    upOut_ = fxScaler_ != nil ? make(upUsage) : nil;
}

float MeshRenderer::chooseScale(NSUInteger width, NSUInteger height) {
    std::vector<DynresSample> done;
    {
        std::lock_guard<std::mutex> lk(dynresMutex_);
        done.swap(dynresDone_);
    }
    if (fixedRenderScale > 0) {
        if (fixedRenderScale >= 0.999f) return 1.0f;
        ensureUpscaler(width, height);
        return std::clamp(fixedRenderScale, 0.25f, 1.0f);
    }
    if (!dynamicResolution) {
        dynres_.reset();
        return 1.0f;
    }
    dynres_.params = dynamicResolutionParams;
    if (upscaler == Upscaler::MetalFXTemporal && fxTemporal_ != nil) {  // TAA: the temporal scaler runs at every scale
        dynres_.params.alwaysUpscale = true;
        if (!std::getenv("FUEL_DYNRES_UPSCALE")) dynres_.params.upscaleMs = temporalUpscaleMs;
    }
    for (const DynresSample& d : done)
        if (d.epoch == cutEpoch_) dynres_.report(d.ms, d.scale);  // frames encoded before the last cut are the old view's
    const float s = dynres_.next();
    if (s < 1.0f) ensureUpscaler(width, height);
    return s;
}

void MeshRenderer::encodeUpscale(id<MTLCommandBuffer> cb, id<MTLTexture> target) {
    const float w = float(target.width), h = float(std::max<NSUInteger>(target.height, 1));
    PostUniforms pu{};
    const bool srgbOut = target.pixelFormat == MTLPixelFormatBGRA8Unorm_sRGB || target.pixelFormat == MTLPixelFormatRGBA8Unorm_sRGB;
    pu.flags = simd_make_float4(0, 0, srgbOut ? 1.0f : 0.0f, 0);
    pu.view = simd_make_float4(float(viewW_) / w, float(viewH_) / h, (float(viewW_) - 0.5f) / w, (float(viewH_) - 0.5f) / h);
    pu.texel = simd_make_float4(1.0f / w, 1.0f / h, 0, 0);  // one source texel (the source has the output's size)
    // The default is the bilinear + sharpen mode (0.6 ms at 3420x2224 on an M2); FUEL_UPSCALE_CATMULL=1 takes the
    // Catmull-Rom + sharpen one (1.3 ms, a little crisper; P93 notes). FUEL_UPSCALE_SHARPEN sets the amount.
    static const bool catmull = std::getenv("FUEL_UPSCALE_CATMULL") != nullptr;
    static const float sharpen = std::getenv("FUEL_UPSCALE_SHARPEN") ? std::stof(std::getenv("FUEL_UPSCALE_SHARPEN")) : catmull ? 0.5f : 0.8f;
    pu.mb0 = simd_make_float4(sharpen, catmull ? 0.0f : 1.0f, 0, 0);  // fsUpscale: x the sharpening, y 1 = bilinear
    pu.grain = grainNow_;  // P102: this frame's grain (set by encodePost), after the upscale
    id<MTLTexture> src = lowOut_;
    id<MTLRenderPipelineState> ps = sharpenPipeline_;
    bool afterScaler = false;
    if (upscaler == Upscaler::MetalFX && fxScaler_ != nil && upOut_ != nil) {
        if (@available(macOS 13.0, *)) {
            id<MTLFXSpatialScaler> sc = fxScaler_;
            sc.inputContentWidth = viewW_, sc.inputContentHeight = viewH_;
            sc.colorTexture = lowOut_;
            sc.outputTexture = upOut_;
            [sc encodeToCommandBuffer:cb];
            src = upOut_, ps = presentPipeline_, afterScaler = true;
        }
    }
    MTLRenderPassDescriptor* rp = [MTLRenderPassDescriptor renderPassDescriptor];
    rp.colorAttachments[0].texture = target;
    rp.colorAttachments[0].loadAction = MTLLoadActionDontCare;
    rp.colorAttachments[0].storeAction = MTLStoreActionStore;
    // The MetalFX passes are not tagged (they are MetalFX's): the present pass is charged from the end of the pass
    // before them, so the upscale's GPU time counts in the frame.
    gpuprof::tag(rp, "post.upscale", afterScaler);
    id<MTLRenderCommandEncoder> enc = [cb renderCommandEncoderWithDescriptor:rp];
    [enc setRenderPipelineState:ps];
    [enc setFragmentBytes:&pu length:sizeof pu atIndex:0];
    [enc setFragmentTexture:src atIndex:0];
    [enc setFragmentTexture:grainTex_ atIndex:1];  // P102
    [enc drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:0 vertexCount:3];
    [enc endEncoding];
}

std::vector<simd_float4> MeshRenderer::shaderCheck(const Atmosphere& a, simd_float3 eye, const std::vector<simd_float4>& inputs,
                                                   const std::vector<CarOcclusion>& boxes) {
    FrameUniforms fu{};
    // P74: the car occlusion boxes through fillCarOcclusion, with a test spot map: 128 x 32 R8, texel i = 2i / 255.
    id<MTLTexture> spot = white_;
    if (!boxes.empty()) {
        MTLTextureDescriptor* td = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatR8Unorm width:128 height:32 mipmapped:NO];
        spot = [device_ newTextureWithDescriptor:td];
        std::vector<uint8_t> px(128 * 32);
        for (size_t i = 0; i < px.size(); ++i) px[i] = uint8_t(2 * (i % 128));
        [spot replaceRegion:MTLRegionMake2D(0, 0, 128, 32) mipmapLevel:0 withBytes:px.data() bytesPerRow:128];
        const std::vector<CarOcclusion> saved = carOcclusion_;
        const bool on = carOcclusionOn;
        carOcclusion_ = boxes, carOcclusionOn = true;
        std::swap(carSpot_, spot);
        fillCarOcclusion(fu, eye);
        std::swap(carSpot_, spot);
        carOcclusion_ = saved, carOcclusionOn = on;
    }
    fu.eye = simd_make_float4(eye, 1);
    fu.camUp = simd_make_float4(0, 1, 0, 0);  // P77: omni.phl's view-space v.y
    fillVehicleOmnis(fu, eye);                // P77: the lights given to setVehicleLights, at timeOfDay()
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
    {  // PK28: the omni rows
        NSUInteger n = 0;
        const void* rows = omniBytes(&n);
        [enc setBytes:rows length:n atIndex:3];
    }
    [enc setTexture:spot atIndex:0];
    [enc setTexture:grainTex_ atIndex:1];  // P102: the film grain's noise tile (case 11)
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
    PendingTile p;
    p.set = true;
    // P92: the staging buffers come from a ring per level, made once: a new buffer per tile cost ~2 ms of GPU time at
    // its first use (5.5 MB of fresh pages for the GPU to map). A ring slot comes back after kTileRing more tiles of its
    // level, seconds after the GPU copied it (each tile is a worker job; one upload per frame at most).
    std::lock_guard<std::mutex> lock(pendingTilesMutex_);
    TileStaging& st = tileStaging_[size_t(level)][tileStagingNext_[size_t(level)]++ % kTileRing];
    if (st.rgba == nil) {
        st.rgba = [device_ newBufferWithLength:rgba.size() options:MTLResourceStorageModeShared];
        st.types = [device_ newBufferWithLength:types.size() * sizeof(uint16_t) options:MTLResourceStorageModeShared];
        st.decal = [device_ newBufferWithLength:decal.size() * sizeof(float) options:MTLResourceStorageModeShared];
    }
    std::memcpy(st.rgba.contents, rgba.data(), rgba.size());
    std::memcpy(st.types.contents, types.data(), types.size() * sizeof(uint16_t));
    std::memcpy(st.decal.contents, decal.data(), decal.size() * sizeof(float));
    p.rgba = st.rgba, p.types = st.types, p.decal = st.decal;
    p.rect = simd_make_float4(float(originX), float(originZ), extent, 1);
    pendingTiles_[size_t(level)] = p;  // a newer tile for the same level replaces one not yet uploaded
}

void MeshRenderer::flushGroundTiles(id<MTLCommandBuffer> cb) {
    std::array<PendingTile, kGroundTileLevels> tiles;
    {
        std::lock_guard<std::mutex> lock(pendingTilesMutex_);
        tiles = pendingTiles_;
        for (PendingTile& p : pendingTiles_) p = PendingTile{};
    }
    bool any = false;
    for (const PendingTile& p : tiles) any = any || p.set;
    if (!any) return;
    const MTLSize size = MTLSizeMake(kGroundTile, kGroundTile, 1);
    id<MTLBlitCommandEncoder> blit = gpuprof::blit(cb, "tiles");  // P92
    for (size_t l = 0; l < tiles.size(); ++l) {
        const PendingTile& p = tiles[l];
        if (!p.set) continue;
        [blit copyFromBuffer:p.types sourceOffset:0 sourceBytesPerRow:kGroundTile * 8 sourceBytesPerImage:kGroundTile * kGroundTile * 8
                  sourceSize:size toTexture:groundTypes_ destinationSlice:l destinationLevel:0 destinationOrigin:MTLOriginMake(0, 0, 0)];
        [blit copyFromBuffer:p.decal sourceOffset:0 sourceBytesPerRow:kGroundTile * 16 sourceBytesPerImage:kGroundTile * kGroundTile * 16
                  sourceSize:size toTexture:groundDecal_ destinationSlice:l destinationLevel:0 destinationOrigin:MTLOriginMake(0, 0, 0)];
        [blit copyFromBuffer:p.rgba sourceOffset:0 sourceBytesPerRow:kGroundTile * 4 sourceBytesPerImage:kGroundTile * kGroundTile * 4
                  sourceSize:size toTexture:groundFine_ destinationSlice:l destinationLevel:0 destinationOrigin:MTLOriginMake(0, 0, 0)];
    }
    [blit generateMipmapsForTexture:groundFine_];
    [blit endEncoding];
    // P92: the draw sets of the new level-0 / level-1 tiles' lattice triangles (fsTerrain's near bands read them).
    if (groundDrawSet_ != nil && drawSetPipeline_ != nil && (tiles[0].set || tiles[1].set)) {
        id<MTLComputeCommandEncoder> ce = gpuprof::compute(cb, "tiles");
        [ce setComputePipelineState:drawSetPipeline_];
        [ce setTexture:groundTypes_ atIndex:0];
        [ce setTexture:groundDrawSet_ atIndex:1];
        [ce setTexture:groundDecal_ atIndex:2];
        for (uint32_t l = 0; l < 2; ++l) {
            if (!tiles[l].set) continue;
            [ce setBytes:&l length:sizeof l atIndex:0];
            [ce dispatchThreads:MTLSizeMake(kGroundTile, kGroundTile, 2) threadsPerThreadgroup:MTLSizeMake(16, 16, 1)];
        }
        [ce endEncoding];
    }
    // The new rects take effect in this frame's uniforms, together with the data uploaded above.
    for (size_t l = 0; l < tiles.size(); ++l)
        if (tiles[l].set) groundFineRect_[l] = tiles[l].rect;
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

// P92: the grass / water sway clock. FUEL_FIXED_CLOCK=1 (image comparisons) runs it at 60 frames a second of
// rendered frames instead of the wall clock, so two runs of the same shot give the same picture.
float MeshRenderer::animSeconds() const {
    if (animPausedAt_ >= 0) return animPausedAt_;  // PK17: frozen while the pause menu holds the world
    if (fixedClock_) return float(frame_) / 60.0f - animPausedTotal_;
    return std::chrono::duration<float>(std::chrono::steady_clock::now() - grassEpoch_).count() - animPausedTotal_;
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
    // P92: runs of one 24 m chunk each (the streamer appends the tufts chunk by chunk; a run also ends at 4096 tufts).
    // The box holds a card at full size: 0.75 x size to the side, size up, and its sway (at most 2.55 x 0.06 x size).
    grassRuns_.clear();
    auto cellOf = [](const GrassTuft& t) { return std::make_pair(int(std::floor(t.x / 24.0f)), int(std::floor(t.z / 24.0f))); };
    for (size_t i = 0; i < tufts.size();) {
        const auto cell = cellOf(tufts[i]);
        GrassRun r;
        r.first = uint32_t(i);
        simd_float3 lo = simd_make_float3(1e30f, 1e30f, 1e30f), hi = -lo;
        size_t j = i;
        for (; j < tufts.size() && j - i < 4096 && cellOf(tufts[j]) == cell; ++j) {
            const GrassTuft& t = tufts[j];
            const float reach = std::fabs(t.size) * 1.0f + 0.25f;
            lo = simd_min(lo, simd_make_float3(t.x - reach, t.y - 0.25f, t.z - reach));
            hi = simd_max(hi, simd_make_float3(t.x + reach, t.y + std::fabs(t.size) + 0.25f, t.z + reach));
        }
        r.count = uint32_t(j - i), r.lo = lo, r.hi = hi;
        grassRuns_.push_back(r);
        i = j;
    }
}

void MeshRenderer::encodeGrass(id<MTLRenderCommandEncoder> enc, const ViewCull& cull) {
    static const bool disabled = std::getenv("FUEL_NO_GRASS") != nullptr;  // FUEL_NO_GRASS=1: measure without
    if (disabled || !grass || grassCount_ == 0 || grassAtlas_ == nil || grassPipeline_ == nil) return;
    struct {
        simd_float4 a;
    } params;
    const float t = animSeconds();
    // PK17: 0x46aca0 sets the world's sway factor W+0xfb4 / W+0x5c040 to 0 for the pause backdrop [I: the factor scales
    // the sway; the port's 0.06 is its own grass amplitude].
    params.a = simd_make_float4(std::fmod(t, 1000.0f), grassRadius, grassRadius * 0.75f, pauseSky_ ? 0.0f : 0.06f);
    grassClock_ = params.a.x, grassAmp_ = params.a.w;  // TAA: the velocity pass's sway clock
    [enc setRenderPipelineState:grassPipeline_];
    [enc setDepthStencilState:depthWrite_];
    [enc setCullMode:MTLCullModeNone];
    [enc setTriangleFillMode:wireframe ? MTLTriangleFillModeLines : MTLTriangleFillModeFill];
    [enc setVertexBuffer:grassBuffer_ offset:0 atIndex:2];
    [enc setVertexBytes:&params length:sizeof params atIndex:3];
    [enc setVertexTexture:groundColour_ != nil ? groundColour_ : white_ atIndex:0];
    [enc setFragmentTexture:grassAtlas_ atIndex:0];
    [enc setFragmentTexture:shadowMap_ atIndex:7];
    // P92: the runs (24 m grass chunks) that can show, in their order: the drawn tufts keep the order of the whole
    // list, so the picture is the one a single draw of every tuft makes.
    if (grassRuns_.empty()) {
        [enc drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:0 vertexCount:9 instanceCount:grassCount_];
        cpu_.grassTufts += double(grassCount_);  // P92 statistics
        if (temporalFrame_) taaGrassDraws_.emplace_back(0u, uint32_t(grassCount_));  // TAA
        return;
    }
    uint32_t first = 0, count = 0;  // consecutive visible runs merge into one draw
    auto flush = [&] {
        if (count > 0) {
            [enc drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:0 vertexCount:9 instanceCount:count baseInstance:first];
            if (temporalFrame_) taaGrassDraws_.emplace_back(first, count);  // TAA: the velocity pass draws the same runs
        }
        cpu_.grassTufts += double(count);
        count = 0;
    };
    for (const GrassRun& r : grassRuns_) {
        if (cull.boxOutside(r.lo, r.hi)) {
            flush();
            continue;
        }
        if (count == 0) first = r.first;
        count += r.count;
    }
    flush();
}

}  // namespace fuel
