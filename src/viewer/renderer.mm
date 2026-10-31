#import "renderer.h"

#include <chrono>

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
    float4 groundFar;     // rgb: far-band forest colour (palette 57, linear); w = 1 when the colour map is
                          // FUEL's decoded point colour (alpha = forest weight), 0 for the old heuristic map
    float4 groundFine[3]; // fine ground tiles around the player (1, 2, 4 m): origin x, z, extent, w = 1 when loaded
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
    float3 zenith = float3(0.11, 0.19, 0.38), horizon = float3(0.70, 0.56, 0.42), ground = float3(0.20, 0.15, 0.10);
    float3 c = d.y >= 0.0 ? mix(horizon, zenith, pow(d.y, 0.5)) : mix(horizon * 0.75, ground, saturate(-d.y * 4.0));
    float mu = saturate(dot(d, sun));
    return c + float3(1.0, 0.62, 0.30) * (pow(mu, 6.0) * 0.40 + pow(mu, 48.0) * 0.9);
}

// The world palette is a bright spring green; FUEL's evening ground reads olive
// and brown. Pull greens towards their luminance and warm them a little.
float3 groundGrade(float3 c) {
    float l = dot(c, float3(0.30, 0.59, 0.11));
    float green = saturate((c.g - max(c.r, c.b)) / max(c.g, 1e-3) * 2.0);
    return mix(float3(l), c, mix(0.75, 0.45, green)) * float3(0.92, 0.86, 0.76);
}

// Aerial perspective: distance haze in the colour of the sky near the horizon in
// that direction, so it glows warm towards the sun.
float3 haze(float3 rgb, float3 world, constant FrameUniforms& f) {
    if (f.fog.x <= 0.0) return rgb;
    float3 d = world - f.eye.xyz;
    float k = (1.0 - exp(-length(d) * f.fog.x)) * 0.85;
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

    float3 ambient = mix(float3(0.15, 0.12, 0.08), float3(0.38, 0.38, 0.42), N.y * 0.5 + 0.5) * ao;
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
    float alpha = m.alphaMode == 2 ? c.a : 1.0;
    if (m.alphaMode == 1) {
        // Cut-outs: the 0.5 edge sharpened to about a pixel; with MSAA the alpha
        // becomes coverage (alpha to coverage), so edges stay soft and depth sorts them.
        alpha = saturate((c.a - 0.5) / max(fwidth(c.a), 1e-4) + 0.5);
        // Leaves at the lens would fill the screen: thin them out within 1.5..4 m.
        if (m.features & Foliage) alpha *= saturate((distance(in.world, f.eye.xyz) - 1.5) / 2.5);
        if (alpha <= 0.0) discard_fragment();
    }
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
    float4 sunPos;
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
    o.sunPos = f.sunViewProj * float4(w, 1.0);
    o.normal = float3(v.n);
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
        // Chunks of 96 m and more use fixed sets (0x7f7ea0 / 0x7f8850): steepness w1 and the
        // sand / snow / cracked share w2. Inferred: the *NL textures (types 17-21).
        const float scale = C < 383.0 ? 0.015 : 0.0015, extra = C < 383.0 ? 0.75 : 0.0;
        const float2 uvF = warp * scale;
        const float4 t0 = detail.sample(rep, uvF, 21), t1 = detail.sample(rep, uvF, 20);
        const float4 t2 = detail.sample(rep, uvF, extra > 0.0 ? 19 : 18);
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
                          texture2d_array<float> groundDecal [[texture(6)]], depth2d<float> shadowMap [[texture(7)]],
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
    if (f.fog.y > 0.0) {
        constexpr sampler lin(coord::normalized, filter::linear, mip_filter::linear, address::clamp_to_edge);
        constexpr sampler rep(coord::normalized, filter::linear, mip_filter::linear, address::repeat, max_anisotropy(8));
        float2 uv = (xz + 70000.0) / 140000.0;
        const float4 gc = groundColourAt(xz, groundColour.sample(lin, uv), groundFine, f);
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
        if (f.groundFar.w > 0.0) c = mix(c, f.groundFar.rgb, gc.a * smoothstep(1400.0, 1800.0, dist));
        // ground.phl: albedo = lerp(vcol * tN.w * 2, tDiffuse.rgb, tDiffuse.a * decal weight), in gamma
        // space (DX9 without sRGB reads, see vsGrass).
        if (f.groundFar.w > 0.0) {
            float3 g = pow(c, 1.0 / 2.2) * b.n.b * 2.0;
            g = mix(g, pow(b.decal.rgb, 1.0 / 2.2), saturate(b.decal.a));
            c = pow(g, 2.2);
        } else {
            c = mix(c * b.n.b * 2.0, b.decal.rgb, saturate(b.decal.a));
        }
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
    float shadow = shadowFactor(shadowMap, in.sunPos);
    float3 V = normalize(f.eye.xyz - in.world);
    float3 rgb = lighting(c, N, V, 1.0, shadow, 0.0, 1.0, float2(0), false, f);
    return float4(haze(rgb, in.world, f), 1.0);
}


// ---- Lake water: ground_water.vhl / .phl (vh_water_anim, ph_water), notes p22 / water.md 3.3.
// Port: no scene-colour copy or depth texture; the refraction lerps are done by blending
// (refraction unperturbed) and the water depth comes from the vertices.
struct WaterVertex {
    packed_float3 position;
    uchar4 colour;  // B, G, R, A bytes
    float2 uv;
    float depth;
    float pad;
};

struct WaterOut {
    float4 position [[position]];
    float3 world;
    float4 colour;
    float4 uv;
    float depth;
    float4 sunPos;
};

vertex WaterOut vsWater(uint vid [[vertex_id]], const device WaterVertex* v [[buffer(0)]], constant FrameUniforms& f [[buffer(1)]],
                        constant float4* W [[buffer(2)]], constant float& t [[buffer(3)]]) {
    WaterOut o;
    float3 w = float3(v[vid].position);
    // Wave: y -= W3.y (sin(-x W3.x + W3.z t) 0.5 + 0.5).
    w.y -= W[3].y * (sin(-w.x * W[3].x + W[3].z * t) * 0.5 + 0.5);
    o.position = f.viewProj * float4(w, 1.0);
    o.world = w;
    o.colour = float4(float3(v[vid].colour.zyx), float(v[vid].colour.w)) / 255.0;
    const float2 uv = v[vid].uv;
    o.uv = float4(uv * W[1].z + W[1].xy * t, uv * W[2].z + W[2].xy * t);
    o.depth = v[vid].depth;
    o.sunPos = f.sunViewProj * float4(w, 1.0);
    return o;
}

fragment float4 fsWater(WaterOut in [[stage_in]], constant FrameUniforms& f [[buffer(1)]], constant float4* W [[buffer(2)]],
                        texture2d<float> foamTex [[texture(0)]], texture2d<float> normalTex [[texture(1)]],
                        depth2d<float> shadowMap [[texture(7)]]) {
    constexpr sampler rep(coord::normalized, filter::linear, mip_filter::linear, address::repeat, max_anisotropy(4));
    const float4 S0 = W[4], S1 = W[5], S2 = W[6], S3 = W[7];
    const float3 V = normalize(f.eye.xyz - in.world);
    // Depth of water along the view ray (the engine reads the depth buffer), x Settings1.z.
    const float waterDepth = max(in.depth, 0.0) / max(abs(V.y), 0.1) * S1.z;
    float3 n = (normalTex.sample(rep, in.uv.xy).xyw * 2.0 - 1.0) + (normalTex.sample(rep, in.uv.zw).xyw * 2.0 - 1.0);
    const float3 ns = normalize(float3(n.x, n.y * S3.x * 3.0, n.z));
    n = normalize(float3(n.x, n.y * S0.w * 0.2, n.z));
    const float fog = saturate(1.0 - exp(-waterDepth * 0.1 * S2.z * in.colour.a));
    const float shadow = shadowFactor(shadowMap, in.sunPos);
    // Light = shadow * sunlight + skylight (scattering terms of the engine; port sun and sky).
    const float3 L = normalize(f.sunDir.xyz);
    const float3 light = kSunColour * f.sunDir.w * shadow * 0.6 + skyColor(float3(0, 1, 0), L) * 0.6;
    const float3 wc = in.colour.rgb * light * S0.x;
    // c = refr * k + K, tracked through the shader's lerps with the refraction.
    float k = 1.0 - fog;
    float3 K = wc * fog;
    const float3 r = reflect(-V, n);
    const float3 env = skyColor(float3(r.x, abs(r.y), r.z), L) * (0.25 + 0.75 * shadow);
    const float F = saturate(pow(1.0 - saturate(dot(V, n)), 5.0) * (1.0 - saturate(S1.w)) + S1.w) * S2.y;
    k *= 1.0 - F;
    K = K * (1.0 - F) + env * F;
    const float spec = pow(max(1e-4, dot(reflect(-V, ns), L)), S3.y * 2.0) * S3.z * 0.3;
    K += light * spec;
    const float fo = saturate(1.1 - waterDepth * 0.002 * S2.x);
    const float ff = fo * fo * foamTex.sample(rep, in.uv.xy - n.xz * 0.5).a;
    k *= 1.0 - ff;
    K = K * (1.0 - ff) + light * ff;
    const float b = saturate(waterDepth * 0.02 * S2.x);
    // out = lerp(refr, c, b): premultiplied, blended One / OneMinusSourceAlpha.
    return float4(haze(K, in.world, f) * b, b * (1.0 - k));
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
    float3 c = skyColor(dir, sun) * 1.05 + float3(1.0, 0.85, 0.6) * smoothstep(0.99975, 0.99990, dot(dir, sun)) * 40.0;
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
    // Lit by the sun through the cloud (no normal: a thin cloud scatters light from
    // every side); strong forward scattering (Henyey-Greenstein, g = 0.55) makes
    // plumes glow when the camera looks towards the low sun.
    const float3 L = normalize(f.sunDir.xyz);
    const float mu = dot(normalize(p.position.xyz - f.eye.xyz), L);
    const float g = 0.55;
    const float hg = (1.0 - g * g) / pow(1.0 + g * g - 2.0 * g * mu, 1.5) / (4.0 * M_PI_F);
    float3 light = kSunColour * f.sunDir.w * (0.35 + hg * 1.2) + float3(0.28, 0.28, 0.29);
    o.rgb = haze(p.colour.rgb * light, p.position.xyz, f);
    o.alpha = p.colour.a;
    return o;
}

fragment float4 fsDust(DustOut in [[stage_in]], depth2d<float> depth [[texture(0)]], constant FrameUniforms& f [[buffer(1)]]) {
    float r2 = dot(in.uv, in.uv);
    if (r2 >= 1.0) discard_fragment();
    // Lumpy edges: a little value noise over the round falloff.
    float n = valueNoise(in.uv * 2.3 + in.seed * 17.0) * 0.6 + valueNoise(in.uv * 5.1 - in.seed * 9.0) * 0.4;
    float blob = saturate(pow(1.0 - r2, 1.2) * mix(0.3, 1.7, n));
    // Soft particles: fade out where the sprite meets the scene, and near the camera.
    // The resolved depth turns back into a view distance with zn and q (fog.zw).
    float d = depth.read(uint2(in.position.xy));
    float scene = f.fog.z * f.fog.w / (d + f.fog.w);
    float fade = saturate((scene - in.depth) / max(in.radius * 0.6, 0.1)) * saturate((in.depth - 0.4) / max(1.2, in.radius * 1.2));
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
        c *= float3(1.03, 1.0, 0.94);  // warm white balance
        c = acesFitted(c);
        // Grade in display space: contrast around mid grey, lift the blacks a touch
        // towards brown, golden highlights, then saturation.
        float3 g = pow(c, 1.0 / 2.2);
        g = saturate((g - 0.45) * u.grade.y + 0.45);
        g = g * mix(float3(0.97, 1.0, 1.02), float3(1.04, 0.99, 0.90), g) + float3(0.016, 0.011, 0.004) * (1.0 - g);
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
    float3 tint;
    float occlusion;
    float4 sunPos;
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
    // ground_rotshape.vhl: vcolor = colour x 2 (x the scattering extinction, which the port's haze
    // applies later); the colour is the decoded point colour lerp(ctx+0x80, ctx+0x50, road) and its
    // alpha the sky occlusion. The texel multiplies it in the pixel shader.
    // FUEL multiplies in gamma space (DX9, no sRGB reads), so x2 is applied before linearising:
    // linear(2 c x tex) = linear(2 c) x linear(tex) (power law).
    o.tint = pow(2.0 * float3(t.colour.rgb) / 255.0, 2.2);
    o.occlusion = float(t.colour.a) / 255.0;
    o.sunPos = f.sunViewProj * float4(w, 1.0);
    return o;
}

fragment float4 fsGrass(GrassOut in [[stage_in]], constant FrameUniforms& f [[buffer(1)]], texture2d<float> atlas [[texture(0)]],
                        depth2d<float> shadowMap [[texture(7)]]) {
    constexpr sampler smp(coord::normalized, filter::linear, mip_filter::linear, address::clamp_to_edge, max_anisotropy(4));
    float4 d = atlas.sample(smp, in.uv);
    if (d.a < 0.4) discard_fragment();
    d.rgb *= in.tint;
    const float shadow = mix(0.55, 1.0, shadowFactor(shadowMap, in.sunPos));
    const float3 V = normalize(f.eye.xyz - in.world);
    // The occlusion scales the sky light only (ground_rotshape.vhl: lerp(DSKY_AMBIENT, sky, occlusion)).
    const float3 rgb = lighting(d.rgb, float3(0, 1, 0), V, in.occlusion, shadow, 0.0, 1.0, float2(0), false, f);
    return float4(haze(rgb, in.world, f), d.a);
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
    dustPipeline_ = makePost(@"vsDust", @"fsDust", kRenderColorFormat, 1);
    brightPipeline_ = makePost(@"vsFull", @"fsBright", kRenderColorFormat, 0);
    downPipeline_ = makePost(@"vsFull", @"fsDown", kRenderColorFormat, 0);
    upPipeline_ = makePost(@"vsFull", @"fsUp", kRenderColorFormat, 2);
    shaftsPipeline_ = makePost(@"vsFull", @"fsShafts", kRenderColorFormat, 0);
    compositePipeline_ = makePost(@"vsFull", @"fsComposite", kOutputColorFormat, 0);

    // A low, warm evening sun (about 22 degrees up), as in most of FUEL's races.
    sunDir_ = simd_normalize(simd_make_float3(0.718f, 0.375f, 0.587f));
}

void MeshRenderer::setGround(const GroundMaps& maps, const DpcFile& world) {
    // Colour map (sRGB, mipmapped) and surface-type map (one byte per sample).
    MTLTextureDescriptor* td = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatRGBA8Unorm_sRGB
                                                                                    width:maps.size
                                                                                   height:maps.size
                                                                                mipmapped:YES];
    groundColour_ = [device_ newTextureWithDescriptor:td];
    groundFar_ = simd_make_float4(srgbToLinear(maps.farColour[0]), srgbToLinear(maps.farColour[1]), srgbToLinear(maps.farColour[2]),
                                  maps.decoded ? 1.0f : 0.0f);
    [groundColour_ replaceRegion:MTLRegionMake2D(0, 0, maps.size, maps.size) mipmapLevel:0 withBytes:maps.colour.data() bytesPerRow:maps.size * 4];
    {
        MTLTextureDescriptor* fd = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatRGBA8Unorm_sRGB
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
            decalTex_[k] = e != nullptr ? uploadBitmap(world, *e, true, &alpha) : nil;
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
        terrainChunks = terrainLod_->update(eye.x, eye.z);
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
    rp.colorAttachments[0].clearColor = MTLClearColorMake(0.10, 0.12, 0.15, 1.0);  // behind single meshes (no sky)
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
    pu.grade = simd_make_float4(1.0f, 1.22f, 0.8f, postEffects ? 1.0f : 0.0f);
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
    grassAtlas_ = texture(bitmapId, true, &alpha);
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
