#import "renderer.h"

#include <algorithm>
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
constexpr NSUInteger kShadowMapSize = 2048;
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
              Reflective = 64, GroundTint = 128;

struct VertexOut {
    float4 position [[position]];
    float3 world;
    float3 normal;
    float4 tangent;
    float2 uv;
    float4 sunPos;
    float3 m0, m1, m2;  // the instance's rotation (for object-space normal maps)
    float3 tint;        // local ground colour x2, for grey foliage and rock textures
    uint sprite [[flat]];
};

constexpr sampler shadowSampler(coord::normalized, filter::linear, address::clamp_to_edge, compare_func::less_equal);

float shadowFactor(depth2d<float> map, float4 sunPos) {
    float3 p = sunPos.xyz / sunPos.w;
    float2 uv = p.xy * float2(0.5, -0.5) + 0.5;
    if (any(uv < 0.0) || any(uv > 1.0) || p.z >= 1.0) return 1.0;
    float texel = 1.0 / float(map.get_width());
    float s = 0.0;
    for (int y = -1; y <= 1; ++y)
        for (int x = -1; x <= 1; ++x) s += map.sample_compare(shadowSampler, uv + float2(x, y) * texel, p.z - 0.001);
    return s / 9.0;
}

// A late-afternoon sky like FUEL's: a warm, hazy horizon under a dusty blue
// zenith, with a broad glow around the low sun. Linear HDR; the post pass tone maps.
constant float3 kSunColour = float3(1.0, 0.74, 0.46);

float3 skyColor(float3 d, float3 sun) {
    float3 zenith = float3(0.14, 0.21, 0.36), horizon = float3(0.74, 0.57, 0.39), ground = float3(0.20, 0.15, 0.10);
    float3 c = d.y >= 0.0 ? mix(horizon, zenith, pow(d.y, 0.5)) : mix(horizon * 0.75, ground, saturate(-d.y * 4.0));
    float mu = saturate(dot(d, sun));
    return c + float3(1.0, 0.62, 0.30) * (pow(mu, 6.0) * 0.40 + pow(mu, 48.0) * 0.9);
}

// The world palette is a bright spring green; FUEL's evening ground reads olive
// and brown. Pull greens towards their luminance and warm them a little.
float3 groundGrade(float3 c) {
    float l = dot(c, float3(0.30, 0.59, 0.11));
    float green = saturate((c.g - max(c.r, c.b)) / max(c.g, 1e-3) * 2.0);
    return mix(float3(l), c, mix(0.75, 0.45, green)) * float3(1.06, 0.98, 0.84);
}

// Aerial perspective: distance haze in the colour of the sky near the horizon in
// that direction, so it glows warm towards the sun.
float3 haze(float3 rgb, float3 world, constant FrameUniforms& f) {
    if (f.fog.x <= 0.0) return rgb;
    float3 d = world - f.eye.xyz;
    float k = (1.0 - exp(-length(d) * f.fog.x)) * 0.92;
    return mix(rgb, skyColor(normalize(float3(d.x, max(d.y, 0.0) * 0.15, d.z)), normalize(f.sunDir.xyz)), k);
}

// Sun (GGX highlight, shadowed), hemispheric sky light and sky reflections.
float3 lighting(float3 albedo, float3 N, float3 V, float ao, float shadow, float specStrength, float rough,
                float2 fresnel, bool reflective, constant FrameUniforms& f) {
    float3 L = normalize(f.sunDir.xyz);
    float3 sun = kSunColour * f.sunDir.w;
    float NdotL = saturate(dot(N, L));
    float NdotV = saturate(dot(N, V)) + 1e-4;
    float3 H = normalize(L + V);
    float NdotH = saturate(dot(N, H));
    float VdotH = saturate(dot(V, H));

    float3 ambient = mix(float3(0.13, 0.10, 0.07), float3(0.30, 0.31, 0.34), N.y * 0.5 + 0.5) * ao;
    float3 color = albedo * (sun * NdotL * shadow + ambient);

    float a2 = pow(rough, 4.0);
    float dn = NdotH * NdotH * (a2 - 1.0) + 1.0;
    float D = a2 / (M_PI_F * dn * dn);
    float k = (rough + 1.0) * (rough + 1.0) / 8.0;
    float G = NdotL / (NdotL * (1.0 - k) + k) * NdotV / (NdotV * (1.0 - k) + k);
    float f0 = mix(0.02, 0.08, specStrength);
    float F = f0 + (1.0 - f0) * pow(1.0 - VdotH, 5.0);
    color += sun * (D * G * F / max(4.0 * NdotV, 1e-3)) * shadow * specStrength;

    float3 env = mix(skyColor(reflect(-V, N), L), float3(0.40, 0.38, 0.36), rough);
    float fr = reflective ? mix(fresnel.x, fresnel.y, pow(1.0 - NdotV, 5.0)) * 2.0
                          : (f0 + (1.0 - f0) * pow(1.0 - NdotV, 5.0)) * (1.0 - rough) * specStrength;
    return color + env * saturate(fr) * ao * mix(0.5, 1.0, shadow);
}

// Instanced: one world matrix per instance in buffer 2.
vertex VertexOut vsMain(VertexIn in [[stage_in]], constant FrameUniforms& f [[buffer(1)]],
                        const device float4x4* models [[buffer(2)]], constant uint& sprite [[buffer(3)]],
                        texture2d<float> groundColour [[texture(0)]], uint iid [[instance_id]]) {
    const float4x4 model = models[iid];
    VertexOut o;
    o.tint = float3(1.0);
    o.sprite = sprite;
    if (f.fog.y > 0.0) {
        // Like FUEL's ground, foliage and rocks take the palette colour under them
        // (ground x 2; the fragment shader decides how to apply it).
        constexpr sampler lin(coord::normalized, filter::linear, address::clamp_to_edge);
        o.tint = groundGrade(groundColour.sample(lin, (model[3].xz + 70000.0) / 140000.0, level(0)).rgb) * 2.0;
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
    o.sunPos = f.sunViewProj * w;
    return o;
}

vertex float4 vsShadow(VertexIn in [[stage_in]], constant FrameUniforms& f [[buffer(1)]],
                       const device float4x4* models [[buffer(2)]], uint iid [[instance_id]]) {
    return f.sunViewProj * (models[iid] * float4(in.position, 1.0));
}

fragment float4 fsMain(VertexOut in [[stage_in]], bool front [[front_facing]],
                       constant FrameUniforms& f [[buffer(1)]],
                       constant MaterialUniforms& m [[buffer(2)]],
                       texture2d<float> tDiffuse [[texture(0)]],
                       texture2d<float> tOcclusion [[texture(2)]],
                       texture2d<float> tNormal [[texture(3)]],
                       texture2d<float> tSpecular [[texture(4)]],
                       texture2d<float> tNormalLocal [[texture(5)]],
                       texture2d<float> tDirt [[texture(6)]],
                       depth2d<float> shadowMap [[texture(7)]],
                       sampler smp [[sampler(0)]]) {
    // Codes 17 and 20 use sheets whose left half is the intact part and the
    // right half the damaged one; draw the intact half.
    float2 uvHalf = (m.code == 17 || m.code == 20) ? float2(in.uv.x * 0.5, in.uv.y) : in.uv;
    float4 d = tDiffuse.sample(smp, uvHalf);
    if (in.sprite != 0) {
        // Leaves: the ground's hue with only mild darkening; some leaf textures are
        // already dark, and a plain multiply turns them black.
        const float lum = max(dot(in.tint, float3(0.3, 0.55, 0.15)), 1e-3);
        d.rgb *= in.tint / lum * clamp(lum, 0.6, 1.0);
    }
    else if (m.features & GroundTint)  // rock: a muted version of the ground colour
        d.rgb *= mix(float3(dot(in.tint, float3(0.3, 0.55, 0.15))), in.tint, 0.35);
    if (m.code == 17 || m.code == 19) {
        // Car paint shows where the livery's alpha is 0.
        d.rgb = mix(f.paint.rgb, d.rgb, (m.features & PaintOnly) ? 0.0 : d.a);
        d.a = 1.0;
    }
    if (m.features & HasDirt) {
        float4 dirt = tDirt.sample(smp, in.uv);
        float edge = 1.0 - f.params.x;
        d.rgb = mix(d.rgb, dirt.rgb, smoothstep(edge, edge + 0.15, dirt.a));
    }
    float4 c = d * m.diffuse;
    if (m.alphaMode == 1 && c.a < 0.5) discard_fragment();
    float alpha = m.alphaMode == 2 ? c.a : 1.0;
    if (m.lit == 0) return float4(c.rgb + m.emissive.rgb, alpha);

    // Some foliage stores no normal at all: fall back to up.
    const float3 n0 = dot(in.normal, in.normal) > 1e-8 ? normalize(in.normal) : float3(0, 1, 0);
    float3 N = n0 * (front ? 1.0 : -1.0);
    // Leaf cards lit as flat quads go black on the side away from the sun (and
    // many store no useful normal); light cut-outs like a canopy instead:
    // mostly up, a little of their own direction.
    const bool foliage = in.sprite != 0 || (m.alphaMode != 0 && !(m.features & Reflective));
    if (foliage) N = normalize(mix(n0, float3(0, 1, 0), 0.7));
    if (m.features & HasNormalLocal) {
        // Object-space normal map, two channels plus alpha.
        const float3 nl = tNormalLocal.sample(smp, uvHalf).xyw * 2.0 - 1.0;
        N = normalize(in.m0 * nl.x + in.m1 * nl.y + in.m2 * nl.z);
    } else if ((m.features & HasNormal) && in.sprite == 0) {  // sprites have no usable tangent frame
        // Tangent-space normal map with x in alpha and y in green.
        float2 xy = tNormal.sample(smp, uvHalf).wy * 2.0 - 1.0;
        float3 T = in.tangent.xyz - N * dot(N, in.tangent.xyz);
        T = dot(T, T) > 1e-8 ? normalize(T) : normalize(cross(N, abs(N.y) < 0.9 ? float3(0, 1, 0) : float3(1, 0, 0)));
        float3 B = cross(N, T) * in.tangent.w;
        N = normalize(T * xy.x + B * xy.y + N * sqrt(saturate(1.0 - dot(xy, xy))));
    }
    float ao = (m.features & HasOcclusion) ? tOcclusion.sample(smp, in.uv).r : 1.0;
    float3 spec = m.specular.rgb * ((m.features & HasSpecular) ? tSpecular.sample(smp, in.uv).rgb : float3(1.0));
    float specStrength = saturate(dot(spec, float3(1.0 / 6.0)));
    float rough = clamp(sqrt(2.0 / (max(m.specular.w, 0.0) + 2.0)), 0.08, 1.0);
    float shadow = shadowFactor(shadowMap, in.sunPos);
    if (foliage) shadow = mix(0.55, 1.0, shadow);  // leaves let light through
    float3 V = normalize(f.eye.xyz - in.world);
    float3 rgb = lighting(c.rgb, N, V, ao, shadow, specStrength, rough, m.fresnel, (m.features & Reflective) != 0, f);
    return float4(haze(rgb + m.emissive.rgb, in.world, f), alpha);
}

struct GroundOut {
    float4 position [[position]];
    float3 world;
    float4 sunPos;
};

vertex GroundOut vsGround(uint vid [[vertex_id]], constant FrameUniforms& f [[buffer(1)]]) {
    const float2 corners[4] = {float2(-1, -1), float2(1, -1), float2(-1, 1), float2(1, 1)};
    float3 w = float3(f.footprint.x + corners[vid].x * f.params.w, f.params.z, f.footprint.y + corners[vid].y * f.params.w);
    GroundOut o;
    o.position = f.viewProj * float4(w, 1.0);
    o.world = w;
    o.sunPos = f.sunViewProj * float4(w, 1.0);
    return o;
}

fragment float4 fsGround(GroundOut in [[stage_in]], constant FrameUniforms& f [[buffer(1)]],
                         depth2d<float> shadowMap [[texture(7)]]) {
    float2 rel = in.world.xz - f.footprint.xy;
    // Soft contact darkening under the vehicle's footprint.
    float ao = mix(0.45, 1.0, smoothstep(0.5, 1.4, length(rel / max(f.footprint.zw, 0.05))));
    float shadow = shadowFactor(shadowMap, in.sunPos);
    float3 V = normalize(f.eye.xyz - in.world);
    float3 rgb = lighting(float3(0.30, 0.27, 0.23), float3(0, 1, 0), V, ao, shadow, 0.0, 1.0, float2(0), false, f);
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

struct TerrainOut {
    float4 position [[position]];
    float3 world;
    float4 sunPos;
};

// One grid around the camera target whose spacing grows with the square of
// the distance from the centre: dense up close, coarse at the horizon.
vertex TerrainOut vsTerrain(uint vid [[vertex_id]], constant FrameUniforms& f [[buffer(1)]],
                            texture2d<float> hm [[texture(0)]]) {
    uint n = uint(f.terrainGrid.w);
    float2 q = float2(vid % n, vid / n) / float(n - 1) * 2.0 - 1.0;
    float2 xz = f.terrainGrid.xy + q * abs(q) * f.terrainGrid.z;
    // Snap to a world-fixed lattice (power-of-two spacing near the local vertex
    // spacing) so the ground under a vertex does not change as the grid follows
    // the camera; otherwise hills visibly swim and ripple while driving.
    float2 spacing = max(2.0 * abs(q) * f.terrainGrid.z * (2.0 / float(n - 1)), float2(1.0));
    float2 lattice = exp2(ceil(log2(spacing)));
    xz = round(xz / lattice) * lattice;
    float3 w = float3(xz.x, terrainHeight(hm, xz, f), xz.y);
    TerrainOut o;
    o.position = f.viewProj * float4(w, 1.0);
    o.world = w;
    o.sunPos = f.sunViewProj * float4(w, 1.0);
    return o;
}

fragment float4 fsTerrain(TerrainOut in [[stage_in]], constant FrameUniforms& f [[buffer(1)]],
                          texture2d<float> hm [[texture(0)]], texture2d<float> groundColour [[texture(1)]],
                          texture2d<uint> groundSurface [[texture(2)]], texture2d_array<float> detail [[texture(3)]],
                          depth2d<float> shadowMap [[texture(7)]]) {
    float e = f.terrain.z;
    float2 xz = in.world.xz;
    float hl = terrainHeight(hm, xz - float2(e, 0), f), hr = terrainHeight(hm, xz + float2(e, 0), f);
    float hd = terrainHeight(hm, xz - float2(0, e), f), hu = terrainHeight(hm, xz + float2(0, e), f);
    float3 N = normalize(float3(hl - hr, 2.0 * e, hd - hu));
    float slope = 1.0 - N.y;
    float n = valueNoise(xz * 0.004) * 0.6 + valueNoise(xz * 0.03) * 0.3 + valueNoise(xz * 0.25) * 0.1;
    float3 c;
    if (f.fog.y > 0.0) {
        // FUEL's ground: palette colour from the world map, modulated by the local
        // surface's detail texture (B = grey detail x2, R/G/A = normal; y up).
        constexpr sampler lin(coord::normalized, filter::linear, mip_filter::linear, address::clamp_to_edge);
        constexpr sampler rep(coord::normalized, filter::linear, mip_filter::linear, address::repeat, max_anisotropy(8));
        float2 uv = (xz + 70000.0) / 140000.0;
        c = groundGrade(groundColour.sample(lin, uv).rgb);
        float dist = length(f.eye.xyz - in.world);
        float near = 1.0 - smoothstep(150.0, 600.0, dist);
        if (near > 0.0) {
            float size = float(groundSurface.get_width());
            float2 t = uv * size - 0.5;
            float2 i0 = floor(t), a = t - i0;
            float2 duv = xz / 4.0;  // one repeat per 4 m
            float4 d = 0.0;
            for (int k = 0; k < 4; ++k) {
                uint2 cell = uint2(clamp(i0 + float2(k & 1, k >> 1), 0.0, size - 1.0));
                float w = ((k & 1) ? a.x : 1.0 - a.x) * ((k >> 1) ? a.y : 1.0 - a.y);
                const uint layer = groundSurface.read(cell).r;
                // Two scales (4 m and 17.3 m) so the repeat does not show.
                d += w * (0.6 * detail.sample(rep, duv, layer) + 0.4 * detail.sample(rep, duv * 0.231 + 0.37, layer));
            }
            c *= mix(1.0, d.b * 2.0, near);
            float3 nt = float3(d.r, d.g, d.a) * 2.0 - 1.0;
            float3 T = normalize(float3(1, 0, 0) - N * N.x), B = cross(T, N);
            N = normalize(mix(N, normalize(T * nt.x + N * max(nt.y, 0.2) - B * nt.z), near * 0.8));
        }
    } else {
        // Procedural fallback: grass and dry ground by altitude, rock on steep slopes, snow up high.
        float3 grass = float3(0.035, 0.060, 0.018), dry = float3(0.11, 0.085, 0.045);
        float3 rock = float3(0.10, 0.092, 0.085), snow = float3(0.80, 0.82, 0.86);
        c = mix(grass, dry, saturate(n * 0.9 + (in.world.y - 1500.0) / 1800.0));
        c = mix(c, rock, smoothstep(0.16, 0.32, slope + (n - 0.5) * 0.08));
        c = mix(c, snow, smoothstep(3100.0, 3500.0, in.world.y + n * 300.0) * (1.0 - smoothstep(0.35, 0.6, slope)));
    }
    float shadow = shadowFactor(shadowMap, in.sunPos);
    float3 V = normalize(f.eye.xyz - in.world);
    float3 rgb = lighting(c, N, V, 1.0, shadow, 0.0, 1.0, float2(0), false, f);
    return float4(haze(rgb, in.world, f), 1.0);
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
    const float3 sun = normalize(f.sunDir.xyz);
    float3 c = skyColor(dir, sun) * 1.3 + float3(1.0, 0.85, 0.6) * smoothstep(0.99975, 0.99990, dot(dir, sun)) * 40.0;
    return float4(c, 1.0);
}

struct RoadVertex {
    float4 position;  // w = opacity
    float4 uv;        // x across (0..1), y along
};

struct RoadOut {
    float4 position [[position]];
    float3 world;
    float2 uv;
    float opacity;
    float4 sunPos;
};

vertex RoadOut vsRoad(uint vid [[vertex_id]], const device RoadVertex* v [[buffer(0)]], constant FrameUniforms& f [[buffer(1)]]) {
    RoadOut o;
    float3 w = v[vid].position.xyz;
    o.position = f.viewProj * float4(w, 1.0);
    o.world = w;
    o.uv = v[vid].uv.xy;
    o.opacity = v[vid].position.w;
    o.sunPos = f.sunViewProj * float4(w, 1.0);
    return o;
}

fragment float4 fsRoad(RoadOut in [[stage_in]], constant FrameUniforms& f [[buffer(1)]], texture2d<float> tex [[texture(0)]],
                       depth2d<float> shadowMap [[texture(7)]], constant uint& textured [[buffer(2)]],
                       constant float4& tint [[buffer(3)]]) {
    constexpr sampler rep(coord::normalized, filter::linear, mip_filter::linear, address::repeat, max_anisotropy(8));
    // Dirt road textures are grey detail maps over the dirt colour; tarmac is a colour map (tint 1).
    float3 albedo = (textured != 0 ? tex.sample(rep, float2(in.uv.x, in.uv.y)).rgb : float3(0.5)) * tint.rgb;
    // Soft edges so the ribbon melts into the ground.
    float edge = smoothstep(0.0, 0.18, in.uv.x) * smoothstep(1.0, 0.82, in.uv.x);
    float shadow = shadowFactor(shadowMap, in.sunPos);
    float3 V = normalize(f.eye.xyz - in.world);
    float3 rgb = lighting(albedo, float3(0, 1, 0), V, 1.0, shadow, 0.1, 0.8, float2(0), false, f);
    return float4(haze(rgb, in.world, f), in.opacity * edge);
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

// ---- dust: soft camera-facing sprites over the resolved scene ----

struct DustParticle {
    float4 position;  // xyz, w radius in metres
    float4 colour;    // linear albedo, a opacity
    float4 extra;     // x rotation (radians), y noise seed
};

struct DustOut {
    float4 position [[position]];
    float2 uv;
    float3 rgb;
    float alpha;
    float depth;   // view-space distance of the sprite
    float radius;
    float seed;
};

vertex DustOut vsDust(uint vid [[vertex_id]], uint iid [[instance_id]], const device DustParticle* ps [[buffer(0)]],
                      constant FrameUniforms& f [[buffer(1)]]) {
    const DustParticle p = ps[iid];
    float2 c = float2(vid & 1, vid >> 1) * 2.0 - 1.0;
    float s = sin(p.extra.x), k = cos(p.extra.x);
    float2 r = float2(c.x * k - c.y * s, c.x * s + c.y * k);
    float3 w = p.position.xyz + (f.camRight.xyz * r.x + f.camUp.xyz * r.y) * p.position.w;
    DustOut o;
    o.position = f.viewProj * float4(w, 1.0);
    o.uv = c;
    const float3 fwd = cross(f.camUp.xyz, f.camRight.xyz);
    o.depth = dot(p.position.xyz - f.eye.xyz, fwd);
    o.radius = p.position.w;
    o.seed = p.extra.y;
    // Lit by the sun through the cloud: strong forward scattering (Henyey-Greenstein,
    // g = 0.55) makes plumes glow when the camera looks towards the low sun.
    const float3 L = normalize(f.sunDir.xyz);
    const float mu = dot(normalize(p.position.xyz - f.eye.xyz), L);
    const float g = 0.55;
    const float hg = (1.0 - g * g) / pow(1.0 + g * g - 2.0 * g * mu, 1.5) / (4.0 * M_PI_F);
    float3 light = kSunColour * f.sunDir.w * (0.18 + hg * 1.6) + float3(0.28, 0.27, 0.27);
    o.rgb = haze(p.colour.rgb * light, p.position.xyz, f);
    o.alpha = p.colour.a;
    return o;
}

fragment float4 fsDust(DustOut in [[stage_in]], depth2d<float> depth [[texture(0)]], constant FrameUniforms& f [[buffer(1)]]) {
    float r2 = dot(in.uv, in.uv);
    if (r2 >= 1.0) discard_fragment();
    // Lumpy edges: a little value noise over the round falloff.
    float n = valueNoise(in.uv * 2.3 + in.seed * 17.0) * 0.6 + valueNoise(in.uv * 5.1 - in.seed * 9.0) * 0.4;
    float blob = pow(1.0 - r2, 1.5) * mix(0.55, 1.25, n);
    // Soft particles: fade out where the sprite meets the scene, and near the camera.
    // The resolved depth turns back into a view distance with zn and q (fog.zw).
    float d = depth.read(uint2(in.position.xy));
    float scene = f.fog.z * f.fog.w / (d + f.fog.w);
    float fade = saturate((scene - in.depth) / max(in.radius * 0.6, 0.1)) * saturate((in.depth - 1.0) / 3.0);
    float a = saturate(in.alpha * blob * fade);
    return float4(in.rgb, a);
}

// ---- post-processing: bloom, light shafts, radial blur, tone mapping and grade ----

struct PostUniforms {
    float4 sun;      // xy the sun in uv, z 0..1 how far it is on screen and in front, w glare strength
    float4 texel;    // xy 1 / source size, z aspect (width / height), w frame number
    float4 params;   // x exposure, y radial blur strength, z bloom strength, w light shaft strength
    float4 grade;    // x saturation, y contrast, z vignette, w 1 = grade (0: plain tone map)
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

// Half resolution: a 4x4 box of the scene above a soft threshold; alpha is the
// share of sky (depth at the far plane), which the shafts and the glare use.
fragment float4 fsBright(FullOut in [[stage_in]], texture2d<float> src [[texture(0)]], depth2d<float> depth [[texture(1)]],
                         constant PostUniforms& u [[buffer(0)]]) {
    float2 t = u.texel.xy;
    float3 c = 0.0;
    float sky = 0.0;
    for (int k = 0; k < 4; ++k) {
        float2 o = float2((k & 1) ? 1.0 : -1.0, (k & 2) ? 1.0 : -1.0) * t;
        c += min(src.sample(postLin, in.uv + o).rgb, float3(60.0));
        sky += depth.sample(postLin, in.uv + o) >= 1.0 ? 0.25 : 0.0;
    }
    c *= 0.25;
    const float threshold = 1.4, knee = 0.7;
    float lum = max(c.r, max(c.g, c.b));
    float soft = clamp(lum - threshold + knee, 0.0, 2.0 * knee);
    soft = soft * soft / (4.0 * knee + 1e-4);
    float w = max(soft, lum - threshold) / max(lum, 1e-4);
    return float4(c * w, sky);
}

fragment float4 fsDown(FullOut in [[stage_in]], texture2d<float> src [[texture(0)]], constant PostUniforms& u [[buffer(0)]]) {
    float2 t = u.texel.xy;
    return 0.25 * (src.sample(postLin, in.uv + float2(-t.x, -t.y)) + src.sample(postLin, in.uv + float2(t.x, -t.y)) +
                   src.sample(postLin, in.uv + float2(-t.x, t.y)) + src.sample(postLin, in.uv + float2(t.x, t.y)));
}

// A 3x3 tent of the smaller level, added onto the larger one (rgb only).
fragment float4 fsUp(FullOut in [[stage_in]], texture2d<float> src [[texture(0)]], constant PostUniforms& u [[buffer(0)]]) {
    float2 t = u.texel.xy;
    float3 c = src.sample(postLin, in.uv).rgb * 4.0;
    c += (src.sample(postLin, in.uv + float2(-t.x, 0)).rgb + src.sample(postLin, in.uv + float2(t.x, 0)).rgb +
          src.sample(postLin, in.uv + float2(0, -t.y)).rgb + src.sample(postLin, in.uv + float2(0, t.y)).rgb) * 2.0;
    c += src.sample(postLin, in.uv + float2(-t.x, -t.y)).rgb + src.sample(postLin, in.uv + float2(t.x, -t.y)).rgb +
         src.sample(postLin, in.uv + float2(-t.x, t.y)).rgb + src.sample(postLin, in.uv + float2(t.x, t.y)).rgb;
    return float4(c / 16.0, 0.0);
}

// Quarter resolution: light shafts, marching from each pixel towards the sun
// through the sky mask (sky near the sun shines through gaps in the trees).
fragment float4 fsShafts(FullOut in [[stage_in]], texture2d<float> src [[texture(0)]], constant PostUniforms& u [[buffer(0)]]) {
    if (u.sun.z <= 0.0) return float4(0.0);
    const int n = 28;
    float2 aspect = float2(u.texel.z, 1.0);
    float2 delta = (u.sun.xy - in.uv) / float(n) * 0.9;
    float2 p = in.uv + delta * hash12(in.position.xy + u.texel.w);  // jitter against banding
    float sum = 0.0, decay = 1.0;
    for (int i = 0; i < n; ++i) {
        float near = exp(-length((p - u.sun.xy) * aspect) * 3.5);
        sum += src.sample(postLin, p).a * near * decay;
        decay *= 0.965;
        p += delta;
    }
    return float4(kSunColour * sum / float(n) * u.sun.z, 0.0);
}

float3 acesFitted(float3 x) {  // Narkowicz
    return saturate((x * (2.51 * x + 0.03)) / (x * (2.43 * x + 0.59) + 0.14));
}

fragment float4 fsComposite(FullOut in [[stage_in]], texture2d<float> hdr [[texture(0)]], texture2d<float> bloom [[texture(1)]],
                            texture2d<float> shafts [[texture(2)]], texture2d<float> skyShare [[texture(3)]],
                            constant PostUniforms& u [[buffer(0)]]) {
    float2 uv = in.uv;
    float2 fromCentre = uv - float2(0.5, 0.48);
    float radial = length(fromCentre * float2(u.texel.z, 1.0)) / (0.5 * u.texel.z);  // ~1 at the side edges
    // Speed blur: samples along the ray from the centre, longer towards the edges.
    float blur = u.params.y * smoothstep(0.15, 1.0, radial);
    float3 c;
    if (blur > 0.002) {
        const int n = 10;
        c = 0.0;
        float j = hash12(in.position.xy + u.texel.w) - 0.5;
        for (int i = 0; i < n; ++i) c += hdr.sample(postLin, uv - fromCentre * blur * ((float(i) + j) / float(n))).rgb;
        c /= float(n);
    } else {
        c = hdr.sample(postLin, uv).rgb;
    }
    if (u.params.z > 0.0) c += bloom.sample(postLin, uv).rgb * u.params.z;
    if (u.params.w > 0.0) c += shafts.sample(postLin, uv).rgb * u.params.w;
    if (u.sun.z > 0.0) {
        // Glare around the sun, as strong as there is open sky round it.
        float vis = skyShare.sample(postLin, u.sun.xy).a;
        float d = length((uv - u.sun.xy) * float2(u.texel.z, 1.0));
        c += kSunColour * vis * u.sun.z * u.sun.w * (exp(-d * 6.0) * 0.3 + exp(-d * 24.0) * 1.2);
    }
    c *= u.params.x;
    if (u.grade.w > 0.0) {
        c *= float3(1.07, 1.0, 0.86);  // warm white balance
        c = acesFitted(c);
        // Grade in display space: contrast around mid grey, lift the blacks a touch
        // towards brown, golden highlights, then saturation.
        float3 g = pow(c, 1.0 / 2.2);
        g = saturate((g - 0.45) * u.grade.y + 0.45);
        g = g * float3(1.03, 0.99, 0.90) + float3(0.018, 0.012, 0.004) * (1.0 - g);
        float l = dot(g, float3(0.2126, 0.7152, 0.0722));
        g = mix(float3(l), g, u.grade.x);
        // Vignette: dark corners and edges.
        float v = smoothstep(1.25, 0.35, length(fromCentre * float2(1.0, 1.15)) * 1.6);
        g *= mix(1.0, v, u.grade.z);
        g += (hash12(in.position.xy * 1.37 + u.texel.w * 7.0) - 0.5) / 255.0;  // dither
        c = pow(saturate(g), 2.2);
    } else {
        c = acesFitted(c);
    }
    return float4(c, 1.0);
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
    simd_float4 sun;
    simd_float4 texel;
    simd_float4 params;
    simd_float4 grade;
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
    shaders_ = [device newLibraryWithSource:@(kShaderSource) options:nil error:&err];
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

    MTLTextureDescriptor* sd = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatDepth32Float
                                                                                    width:kShadowMapSize
                                                                                   height:kShadowMapSize
                                                                                mipmapped:NO];
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
        d.vertexFunction = [shaders_ newFunctionWithName:@"vsRoad"];
        d.fragmentFunction = [shaders_ newFunctionWithName:@"fsRoad"];
        d.rasterSampleCount = sampleCount_;
        d.colorAttachments[0].pixelFormat = kRenderColorFormat;
        d.colorAttachments[0].blendingEnabled = YES;
        d.colorAttachments[0].sourceRGBBlendFactor = MTLBlendFactorSourceAlpha;
        d.colorAttachments[0].destinationRGBBlendFactor = MTLBlendFactorOneMinusSourceAlpha;
        d.depthAttachmentPixelFormat = kRenderDepthFormat;
        NSError* e = nil;
        roadPipeline_ = [device newRenderPipelineStateWithDescriptor:d error:&e];
        if (roadPipeline_ == nil) [NSException raise:@"pipeline" format:@"%@", e];
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
    dustPipeline_ = makePost(@"vsDust", @"fsDust", kRenderColorFormat, 1);
    brightPipeline_ = makePost(@"vsFull", @"fsBright", kRenderColorFormat, 0);
    downPipeline_ = makePost(@"vsFull", @"fsDown", kRenderColorFormat, 0);
    upPipeline_ = makePost(@"vsFull", @"fsUp", kRenderColorFormat, 2);
    shaftsPipeline_ = makePost(@"vsFull", @"fsShafts", kRenderColorFormat, 0);
    compositePipeline_ = makePost(@"vsFull", @"fsComposite", kOutputColorFormat, 0);

    // A low, warm evening sun (about 18 degrees up), as in most of FUEL's races.
    sunDir_ = simd_normalize(simd_make_float3(0.736f, 0.31f, 0.602f));
}

void MeshRenderer::setGround(const GroundMaps& maps, const DpcFile& world) {
    // Colour map (sRGB, mipmapped) and surface-type map (one byte per sample).
    MTLTextureDescriptor* td = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatRGBA8Unorm_sRGB
                                                                                    width:maps.size
                                                                                   height:maps.size
                                                                                mipmapped:YES];
    groundColour_ = [device_ newTextureWithDescriptor:td];
    [groundColour_ replaceRegion:MTLRegionMake2D(0, 0, maps.size, maps.size) mipmapLevel:0 withBytes:maps.colour.data() bytesPerRow:maps.size * 4];
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

void MeshRenderer::setRoads(const std::vector<RoadVertex>& vertices, const std::vector<uint32_t>& indices,
                            const std::vector<RoadBatch>& batches) {
    roadBatches_.clear();
    roadVertices_ = vertices.empty() ? nil : [device_ newBufferWithBytes:vertices.data() length:vertices.size() * sizeof(RoadVertex)
                                                                 options:MTLResourceStorageModeShared];
    roadIndices_ = indices.empty() ? nil : [device_ newBufferWithBytes:indices.data() length:indices.size() * 4
                                                               options:MTLResourceStorageModeShared];
    for (const RoadBatch& b : batches) {
        uint8_t alphaKind = 0;
        roadBatches_.push_back({texture(b.texture, true, &alphaKind), b});
    }
}

void MeshRenderer::setTerrain(const Heightfield& hf) {
    MTLTextureDescriptor* td = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatR16Unorm
                                                                                    width:hf.width
                                                                                   height:hf.height
                                                                                mipmapped:NO];
    heightTex_ = [device_ newTextureWithDescriptor:td];
    [heightTex_ replaceRegion:MTLRegionMake2D(0, 0, hf.width, hf.height) mipmapLevel:0 withBytes:hf.raw.data() bytesPerRow:hf.width * 2];
    terrainData_ = hf;
    terrainInfo_ = simd_make_float4(hf.originX, hf.originZ, hf.spacing, hf.metresPerUnit * 65535.0f);
    if (terrainIndices_ == nil) {
        std::vector<uint32_t> idx;
        const uint32_t n = kTerrainGridSize;
        idx.reserve(size_t(n - 1) * (n - 1) * 6);
        for (uint32_t r = 0; r + 1 < n; ++r)
            for (uint32_t c = 0; c + 1 < n; ++c) {
                const uint32_t a = r * n + c, b = a + 1, d = a + n, e = d + 1;
                idx.insert(idx.end(), {a, b, d, b, e, d});
            }
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

id<MTLRenderPipelineState> MeshRenderer::shadowPipeline(uint32_t stride) {
    if (auto it = shadowPipelines_.find(stride); it != shadowPipelines_.end()) return it->second;
    MTLRenderPipelineDescriptor* pd = [MTLRenderPipelineDescriptor new];
    pd.vertexFunction = [shaders_ newFunctionWithName:@"vsShadow"];
    pd.vertexDescriptor = meshVertexDescriptor(stride);
    pd.depthAttachmentPixelFormat = MTLPixelFormatDepth32Float;
    NSError* err = nil;
    id<MTLRenderPipelineState> ps = [device_ newRenderPipelineStateWithDescriptor:pd error:&err];
    if (ps == nil) [NSException raise:@"pipeline" format:@"%@", err];
    shadowPipelines_[stride] = ps;
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
    if (auto hit = library_.find(bitmapId, cls::Bitmap)) {
        try {
            const BitmapInfo bm = parseBitmapLink(hit.file->link(*hit.entry));
            const auto body = hit.file->body(*hit.entry);
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
    }
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
        if (id<MTLTexture> t = texture(diffuseTex, true, &alphaKind)) gm.textures[kDiffuse] = t;
        const TextureStats& ts = textureStats(diffuseTex);
        if (ts.greyDetail) gm.features |= GroundTint;
        const struct { Slot slot; bool srgb; uint32_t feature; } maps[] = {
            {kOcclusion, false, HasOcclusion}, {kNormal, false, HasNormal},     {kSpecular, false, HasSpecular},
            {kNormalLocal, false, HasNormalLocal}, {kDirt, true, HasDirt},
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
        // Some leaf textures are very dark and go black once tinted by the
        // ground colour; lift dark green cut-outs to a common brightness.
        if (gm.alpha != AlphaMode::Opaque && ts.mean.y > ts.mean.x && ts.mean.y > ts.mean.z && ts.mean.y < 0.22f) {
            const float gain = std::min(2.5f, 0.24f / std::max(ts.mean.y, 0.02f));
            gm.diffuse = simd_make_float4(gm.diffuse.xyz * gain, gm.diffuse.w);
        }
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
    const simd_float3 up = std::fabs(sunDir_.y) > 0.99f ? simd_make_float3(0, 0, 1) : simd_make_float3(0, 1, 0);
    const float r = shadowHalfSize() / 1.05f;
    const simd_float3 eye = center_ + sunDir_ * (r * 3.0f);
    return simd_mul(orthoRH(r * 1.05f, r * 0.5f, r * 6.0f), lookAtRH(eye, center_, up));
}

void MeshRenderer::encodeShadows(id<MTLCommandBuffer> cb) {
    MTLRenderPassDescriptor* rp = [MTLRenderPassDescriptor renderPassDescriptor];
    rp.depthAttachment.texture = shadowMap_;
    rp.depthAttachment.loadAction = MTLLoadActionClear;
    rp.depthAttachment.clearDepth = 1.0;
    rp.depthAttachment.storeAction = MTLStoreActionStore;
    id<MTLRenderCommandEncoder> enc = [cb renderCommandEncoderWithDescriptor:rp];
    FrameUniforms fu{};
    fu.sunViewProj = sunViewProj();
    [enc setVertexBytes:&fu length:sizeof fu atIndex:1];
    [enc setDepthStencilState:depthWrite_];
    [enc setDepthBias:1.0f slopeScale:2.0f clamp:0.01f];
    [enc setCullMode:MTLCullModeNone];
    // World objects cast shadows only near the focus (the shadow map covers the scene radius).
    std::vector<const Instance*> casters;
    for (const Instance& inst : instances_) casters.push_back(&inst);
    for (const Instance& inst : worldInstances_)
        if (simd_distance(inst.world.columns[3].xyz, center_) < radius_ * 3.0f + 30.0f) casters.push_back(&inst);
    for (const Instance& inst : propInstances_)
        if (simd_distance(inst.world.columns[3].xyz, center_) < radius_ * 3.0f + 30.0f) casters.push_back(&inst);
    for (const OtherVehicle& ov : vehicles_)
        for (const Instance& inst : ov.instances)
            if (simd_distance(inst.world.columns[3].xyz, center_) < shadowHalfSize() * 1.5f + 5.0f) casters.push_back(&inst);
    for (const Instance* ip : casters) {
        const Instance& inst = *ip;
        const GpuMesh& gm = *inst.mesh;
        [enc setVertexBytes:&inst.world length:sizeof inst.world atIndex:2];
        for (uint32_t pass = 0; pass < 2; ++pass) {  // opaque and alpha-tested parts cast shadows
            for (const Draw& d : gm.draws[pass]) {
                [enc setRenderPipelineState:shadowPipeline(gm.vbStrides[d.vb])];
                [enc setVertexBuffer:gm.vbs[d.vb] offset:0 atIndex:0];
                [enc drawIndexedPrimitives:MTLPrimitiveTypeTriangle
                                indexCount:d.indexCount
                                 indexType:MTLIndexTypeUInt32
                               indexBuffer:gm.indices
                         indexBufferOffset:size_t(d.firstIndex) * 4];
            }
        }
    }
    [enc endEncoding];
}

void MeshRenderer::encode(id<MTLRenderCommandEncoder> enc, float aspect, const OrbitCamera& cam) {
    if (instances_.empty()) return;
    const float dist = cam.distance * radius_;
    simd_float3 eye = center_ + dist * simd_make_float3(std::cos(cam.pitch) * std::sin(cam.yaw), std::sin(cam.pitch),
                                                        std::cos(cam.pitch) * std::cos(cam.yaw));
    if (showTerrain && !terrainData_.raw.empty()) {
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
    if (cameraClearance) {
        const float t = cameraClearance(center_, eye);
        if (t < 1.0f) eye = center_ + (eye - center_) * std::max(0.15f, t - 0.4f / std::max(dist, 0.5f));
    }
    FrameUniforms fu;
    const bool terrain = showTerrain && heightTex_ != nil;
    const float zn = terrain ? std::max(0.05f, dist * 0.02f) : radius_ * 0.02f;
    const float zf = terrain ? std::max(120000.0f, dist * 4.0f) : radius_ * 40.0f;
    const simd_float3 target = center_ + simd_make_float3(0, cam.aim, 0);
    fu.viewProj = simd_mul(perspectiveRH(cam.fovY, aspect, zn, zf), lookAtRH(eye, target, simd_make_float3(0, 1, 0)));
    fu.invViewProj = simd_inverse(fu.viewProj);
    {
        const simd_float3 fwd = simd_normalize(target - eye);
        const simd_float3 right = simd_normalize(simd_cross(fwd, simd_make_float3(0, 1, 0)));
        fu.camRight = simd_make_float4(right, 0);
        fu.camUp = simd_make_float4(simd_cross(right, fwd), 0);
    }
    fu.sunViewProj = sunViewProj();
    fu.terrain = terrainInfo_;
    fu.terrainGrid = simd_make_float4(center_.x, center_.z, std::max(70000.0f, dist * 6.0f), float(kTerrainGridSize));
    fu.fog = simd_make_float4(terrain ? 1.0f / 3600.0f : 0.0f, groundColour_ != nil ? 1.0f : 0.0f, zn, zf / (zn - zf));
    fu.sunDir = simd_make_float4(sunDir_, 4.2f);
    fu.paint = simd_make_float4(srgbToLinear(paint_.color.x), srgbToLinear(paint_.color.y), srgbToLinear(paint_.color.z), 1);
    fu.eye = simd_make_float4(eye, 1);
    fu.params = simd_make_float4(dirt, 1.0f, sceneMin_.y, radius_ * 4.0f);
    fu.footprint = simd_make_float4(center_.x, center_.z, (sceneMax_.x - sceneMin_.x) * 0.5f, (sceneMax_.z - sceneMin_.z) * 0.5f);
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
            vu.paint = simd_make_float4(srgbToLinear(ov.paint.x), srgbToLinear(ov.paint.y), srgbToLinear(ov.paint.z), 1);
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
    };
    if (terrain) {
        [enc setRenderPipelineState:terrainPipeline_];
        [enc setDepthStencilState:depthWrite_];
        [enc setCullMode:MTLCullModeNone];
        [enc setTriangleFillMode:wireframe ? MTLTriangleFillModeLines : MTLTriangleFillModeFill];
        [enc setVertexTexture:heightTex_ atIndex:0];
        [enc setFragmentTexture:heightTex_ atIndex:0];
        if (groundColour_ != nil) {
            [enc setFragmentTexture:groundColour_ atIndex:1];
            [enc setFragmentTexture:groundSurface_ atIndex:2];
            [enc setFragmentTexture:groundDetail_ atIndex:3];
        }
        [enc drawIndexedPrimitives:MTLPrimitiveTypeTriangle
                        indexCount:terrainIndexCount_
                         indexType:MTLIndexTypeUInt32
                       indexBuffer:terrainIndices_
                 indexBufferOffset:0];
        if (roadVertices_ != nil && roadIndices_ != nil) {
            [enc setRenderPipelineState:roadPipeline_];
            [enc setDepthStencilState:depthNoWrite_];
            [enc setDepthBias:-8.0f slopeScale:-24.0f clamp:0.0f];
            [enc setVertexBuffer:roadVertices_ offset:0 atIndex:0];
            [enc setFragmentTexture:shadowMap_ atIndex:7];
            for (const auto& [tex, b] : roadBatches_) {
                const uint32_t textured = tex != nil ? 1u : 0u;
                [enc setFragmentBytes:&textured length:sizeof textured atIndex:2];
                const simd_float4 tint = b.dirt ? simd_make_float4(0.42f, 0.30f, 0.20f, 1) : simd_make_float4(1, 1, 1, 1);
                [enc setFragmentBytes:&tint length:sizeof tint atIndex:3];
                [enc setFragmentTexture:tex != nil ? tex : white_ atIndex:0];
                [enc drawIndexedPrimitives:MTLPrimitiveTypeTriangle
                                indexCount:b.indexCount
                                 indexType:MTLIndexTypeUInt32
                               indexBuffer:roadIndices_
                         indexBufferOffset:size_t(b.firstIndex) * 4];
            }
            [enc setDepthBias:0 slopeScale:0 clamp:0];
        }
    }
    // Terrain and roads first, so vehicles and objects cover the road ribbons.
    drawPass(0);
    drawPass(1);
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
}


void MeshRenderer::ensureTargets(NSUInteger width, NSUInteger height) {
    if (width == targetWidth_ && height == targetHeight_) return;
    targetWidth_ = width, targetHeight_ = height;
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
    hdrMsaa_ = make(kRenderColorFormat, width, height, sampleCount_, rt);
    depthMsaa_ = make(kRenderDepthFormat, width, height, sampleCount_, rt);
    hdr_ = make(kRenderColorFormat, width, height, 1, rw);
    depth_ = make(kRenderDepthFormat, width, height, 1, rw);
    for (int i = 0; i < kBloomLevels; ++i) bloom_[size_t(i)] = make(kRenderColorFormat, width >> (i + 1), height >> (i + 1), 1, rw);
    shafts_ = make(kRenderColorFormat, width >> 2, height >> 2, 1, rw);
}

void MeshRenderer::render(id<MTLCommandBuffer> cb, id<MTLTexture> target, const OrbitCamera& cam) {
    ensureTargets(target.width, target.height);
    encodeShadows(cb);

    // The scene, multisampled; colour and depth resolve for the passes after it.
    MTLRenderPassDescriptor* rp = [MTLRenderPassDescriptor renderPassDescriptor];
    rp.colorAttachments[0].texture = hdrMsaa_;
    rp.colorAttachments[0].resolveTexture = hdr_;
    rp.colorAttachments[0].loadAction = MTLLoadActionClear;
    rp.colorAttachments[0].clearColor = MTLClearColorMake(0.02, 0.022, 0.026, 1.0);
    rp.colorAttachments[0].storeAction = MTLStoreActionMultisampleResolve;
    rp.depthAttachment.texture = depthMsaa_;
    rp.depthAttachment.resolveTexture = depth_;
    rp.depthAttachment.loadAction = MTLLoadActionClear;
    rp.depthAttachment.clearDepth = 1.0;
    rp.depthAttachment.storeAction = MTLStoreActionMultisampleResolve;
    rp.depthAttachment.depthResolveFilter = MTLMultisampleDepthResolveFilterMin;  // the nearest sample
    id<MTLRenderCommandEncoder> enc = [cb renderCommandEncoderWithDescriptor:rp];
    lastFrame_.reset();
    encode(enc, float(target.width) / float(std::max<NSUInteger>(target.height, 1)), cam);
    [enc endEncoding];

    if (lastFrame_ && !dust_.empty()) encodeDust(cb);
    encodePost(cb, target);
    ++frame_;
    [cb addCompletedHandler:^(id<MTLCommandBuffer> b) {
        if (b.GPUEndTime > b.GPUStartTime) gpuMs_ = (b.GPUEndTime - b.GPUStartTime) * 1000.0;
    }];
}

void MeshRenderer::encodeDust(id<MTLCommandBuffer> cb) {
    // Back to front from the camera, for alpha blending.
    const simd_float3 eye = lastFrame_->eye.xyz;
    std::vector<std::pair<float, uint32_t>> order;
    order.reserve(dust_.size());
    for (uint32_t i = 0; i < dust_.size(); ++i) order.emplace_back(-simd_distance_squared(dust_[i].position.xyz, eye), i);
    std::sort(order.begin(), order.end());
    id<MTLBuffer> buf = [device_ newBufferWithLength:order.size() * sizeof(DustParticle) options:MTLResourceStorageModeShared];
    auto* out = static_cast<DustParticle*>(buf.contents);
    for (size_t k = 0; k < order.size(); ++k) out[k] = dust_[order[k].second];

    MTLRenderPassDescriptor* rp = [MTLRenderPassDescriptor renderPassDescriptor];
    rp.colorAttachments[0].texture = hdr_;
    rp.colorAttachments[0].loadAction = MTLLoadActionLoad;
    rp.colorAttachments[0].storeAction = MTLStoreActionStore;
    id<MTLRenderCommandEncoder> enc = [cb renderCommandEncoderWithDescriptor:rp];
    [enc setRenderPipelineState:dustPipeline_];
    [enc setVertexBuffer:buf offset:0 atIndex:0];
    [enc setVertexBytes:lastFrame_.get() length:sizeof(FrameUniforms) atIndex:1];
    [enc setFragmentBytes:lastFrame_.get() length:sizeof(FrameUniforms) atIndex:1];
    [enc setFragmentTexture:depth_ atIndex:0];
    [enc drawPrimitives:MTLPrimitiveTypeTriangleStrip vertexStart:0 vertexCount:4 instanceCount:order.size()];
    [enc endEncoding];
}

void MeshRenderer::encodePost(id<MTLCommandBuffer> cb, id<MTLTexture> target) {
    const float w = float(target.width), h = float(std::max<NSUInteger>(target.height, 1));
    PostUniforms pu{};
    pu.texel = simd_make_float4(1.0f / w, 1.0f / h, w / h, float(frame_ % 1024));
    pu.params = postEffects ? simd_make_float4(1.0f, std::clamp(speedBlur, 0.0f, 1.0f) * 0.11f, 0.10f, 0.55f) : simd_make_float4(1, 0, 0, 0);
    pu.grade = simd_make_float4(0.86f, 1.12f, 0.75f, postEffects ? 1.0f : 0.0f);
    if (postEffects && lastFrame_ && lastFrame_->fog.x > 0) {
        // Where the sun is on screen; glare and shafts fade out as it leaves the frame.
        const simd_float4 clip = simd_mul(lastFrame_->viewProj, simd_make_float4(lastFrame_->eye.xyz + sunDir_ * 1000.0f, 1.0f));
        if (clip.w > 0) {
            const simd_float2 uv = simd_make_float2(clip.x / clip.w * 0.5f + 0.5f, 0.5f - clip.y / clip.w * 0.5f);
            const float out = std::max(std::max(-uv.x, uv.x - 1.0f), std::max(-uv.y, uv.y - 1.0f));
            pu.sun = simd_make_float4(uv, std::clamp(1.0f - out / 0.35f, 0.0f, 1.0f), 1.0f);
        }
    }
    auto pass = [&](id<MTLTexture> dst, MTLLoadAction load, id<MTLRenderPipelineState> ps, std::initializer_list<id<MTLTexture>> textures,
                    id<MTLTexture> texelOf) {
        MTLRenderPassDescriptor* rp = [MTLRenderPassDescriptor renderPassDescriptor];
        rp.colorAttachments[0].texture = dst;
        rp.colorAttachments[0].loadAction = load;
        rp.colorAttachments[0].clearColor = MTLClearColorMake(0, 0, 0, 0);
        rp.colorAttachments[0].storeAction = MTLStoreActionStore;
        id<MTLRenderCommandEncoder> enc = [cb renderCommandEncoderWithDescriptor:rp];
        if (ps == nil) {  // just clear
            [enc endEncoding];
            return;
        }
        [enc setRenderPipelineState:ps];
        PostUniforms u = pu;
        if (texelOf != nil) u.texel.x = 1.0f / float(texelOf.width), u.texel.y = 1.0f / float(texelOf.height);
        [enc setFragmentBytes:&u length:sizeof u atIndex:0];
        NSUInteger slot = 0;
        for (id<MTLTexture> t : textures) [enc setFragmentTexture:t atIndex:slot++];
        [enc drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:0 vertexCount:3];
        [enc endEncoding];
    };
    if (postEffects) {
        pass(bloom_[0], MTLLoadActionDontCare, brightPipeline_, {hdr_, depth_}, hdr_);
        for (int i = 1; i < kBloomLevels; ++i) pass(bloom_[size_t(i)], MTLLoadActionDontCare, downPipeline_, {bloom_[size_t(i - 1)]}, bloom_[size_t(i - 1)]);
        if (pu.sun.z > 0) pass(shafts_, MTLLoadActionDontCare, shaftsPipeline_, {bloom_[1]}, nil);
        else pass(shafts_, MTLLoadActionClear, nil, {}, nil);
        for (int i = kBloomLevels - 1; i > 0; --i) pass(bloom_[size_t(i - 1)], MTLLoadActionLoad, upPipeline_, {bloom_[size_t(i)]}, bloom_[size_t(i)]);
    }
    pass(target, MTLLoadActionDontCare, compositePipeline_, {hdr_, bloom_[0], shafts_, bloom_[kBloomLevels - 1]}, nil);
}

}  // namespace fuel
