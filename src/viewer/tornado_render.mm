// P65: FUEL's tornado draws (SpecialEffectsTornado_G, notes ~/re-work/fuel/notes/p65-storm-visuals.md).
//
// Three SubCodes of tornado.vhl / tornado.phl (the shaders are written here from their decode, no game shader code):
//   0x26 vh_tornadoclouds / ph_tornadoclouds: the cloud-top discs (SpecialEffectsCloudTop_G vf04 0x5e5e40);
//   0x23 vh_tornado / ph_tornado: the funnel (SpecialEffectsTornadoMesh_G vf04 0x5e9600), uv + C_USER0.yz;
//   0x25 vh_tornadobasesmoke / ph_tornadosoftsmoke: the base smoke (0x5e8ca0 quads).
// Like the other effects passes they draw over the resolved scene and test the resolved depth in the fragment.
#import "renderer.h"
#import "gpu_profile.h"

#include <cmath>

namespace fuel {

extern const size_t kFrameUniformsSize;  // renderer.mm
void frameCameraOf(const FrameUniforms& f, simd_float4x4& viewProj, simd_float3& eye, simd_float3& right, simd_float3& up);  // renderer.mm

const char* kTornadoShaderSource = R"METAL(
struct TornadoVtx {        // TornadoVertex_Z (0x40)
    packed_float3 pos;     // relative to the base
    packed_float3 normal;
    float2 uv;
    float alpha;
    float add;
    packed_float3 tangent;
    packed_float3 binormal;
};

struct TornadoParams {
    float4 base;     // xyz: the tornado base (funnel) or the cloud-top centre; w: the disc's half size
    float4 user0;    // C_USER0 (1, u scroll, v scroll, 0)
    float4 flags;    // x: normal map is DXT5 (wyz), y: funnel side (0 far, 1 near), z: disc angle, w: disc lift
    float4 uvRect;   // the smoke's atlas rect (u0, v0, du, dv)
};

struct TornadoOut {
    float4 position [[position]];
    float4 eyevec;   // xyz towards C_USER7 (the camera), w = 1 - saturate(len / 500)
    float3 T;
    float3 B;
    float3 N;
    float4 uvAlpha;
    float3 ext;
    float3 insc;
    float viewZ;
};

float tornadoViewZ(constant FrameUniforms& f, float3 w) {
    return dot(w - f.eye.xyz, cross(f.camUp.xyz, f.camRight.xyz));
}

// The scene's view distance at this pixel (the resolved depth), as fsSfx.
float tornadoSceneZ(constant FrameUniforms& f, depth2d<float> depth, float4 pos) {
    const float d = depth.read(uint2(pos.xy));
    return f.fog.z * f.fog.w / (d + f.fog.w);
}

// h4tex2D(sNormal).wyz * 2 - 1 with z rebuilt (bCompression), else rgb * 2 - 1.
float3 tornadoNormal(texture2d<float> t, sampler s, float2 uv, bool dxt5) {
    const float4 c = t.sample(s, uv);
    if (!dxt5) return c.rgb * 2.0 - 1.0;
    float3 n = c.wyz * 2.0 - 1.0;
    n.z = sqrt(saturate(1.0 - (n.x * n.x + n.y * n.y)));
    return n;
}

// tornado.phl lightingtornado (no specular: the callers pass specular 0, no shadow, no omni).
float3 tornadoLighting(constant FrameUniforms& f, float3 n, float3 L, float3 E, float3 vtx) {
    const float fake = dot(L, E);
    const float nEye = 1.0 - abs(dot(n, E));
    const float sun = saturate(dot(n, L) - saturate(fake) * nEye);
    float skydot;
    const float sky = fuelSkylighting(n, f.skyDir.xyz, skydot);
    return vtx * (sky * f.skyColour.rgb + sun * f.sunColour.rgb);
}

vertex TornadoOut vsTornado(uint vid [[vertex_id]], const device TornadoVtx* vs [[buffer(0)]], constant FrameUniforms& f [[buffer(1)]],
                            constant TornadoParams& p [[buffer(2)]]) {
    const TornadoVtx i = vs[vid];
    const float3 w = p.base.xyz + float3(i.pos);
    TornadoOut o;
    o.position = fuelClip(f, w);
    o.uvAlpha = float4(i.uv + p.user0.yz, i.alpha, i.alpha);
    o.T = -normalize(float3(i.tangent));
    o.N = normalize(float3(i.normal));
    o.B = normalize(float3(i.binormal));
    const float3 vec = f.eye.xyz - w;
    const float len = length(vec);
    o.eyevec = float4(vec / max(len, 1e-3), 1.0 - saturate(len / 500.0));
    const Scatter sc = fuelScattering(f, w);  // scatteringbase
    o.ext = sc.ext;
    o.insc = sc.insc;
    o.viewZ = tornadoViewZ(f, w);
    return o;
}

fragment float4 fsTornado(TornadoOut in [[stage_in]], depth2d<float> depth [[texture(0)]], texture2d<float> tDiffuse [[texture(1)]],
                          texture2d<float> tNormal [[texture(2)]], constant FrameUniforms& f [[buffer(1)]],
                          constant TornadoParams& p [[buffer(2)]]) {
    if (in.viewZ > tornadoSceneZ(f, depth, in.position)) discard_fragment();
    const float3 eye = normalize(in.eyevec.xyz);
    // [port] the tube is drawn far side first, then near side (by the vertex normal), for the blending order.
    const bool near = dot(in.N, eye) >= 0.0;
    if (near != (p.flags.y > 0.5)) discard_fragment();
    constexpr sampler smp(coord::normalized, filter::linear, mip_filter::linear, address::repeat);
    const float2 uv = in.uvAlpha.xy;
    const float2 uv2 = float2(-uv.x * 4.0 + 0.21, uv.y * 4.0);
    const float4 diffuse = tDiffuse.sample(smp, uv);
    const bool dxt5 = p.flags.x > 0.5;
    float3 t = mix(tornadoNormal(tNormal, smp, uv, dxt5), tornadoNormal(tNormal, smp, uv2, dxt5), 0.2 + in.eyevec.w * 0.3);
    const float3 n = normalize(t.x * in.T + t.y * in.B + t.z * in.N);
    const float magic = (eye.x * n.x + eye.z * n.z) * 1.0 + abs(eye.y * n.y) * 1.5;
    float alpha = magic;
    alpha *= alpha * alpha * alpha * 4.0;
    alpha = saturate(alpha);
    float3 c = tornadoLighting(f, n, f.lightDir.xyz, eye, diffuse.rgb);
    c = c * in.ext + in.insc;
    return float4(max(c, 0.0), saturate(alpha * in.uvAlpha.z));
}

// The cloud-top disc: 0x5e59d0's 5 x 5 grid (4 x 4 cells, 6 vertices each), local x = -h (2u - 1), z = h (2v - 1),
// y = lift, uv (1 - u, v), normal (0, -1, 0), tangent (1, 0, 0), binormal (0, 0, 1); turned about y by the angle.
vertex TornadoOut vsTornadoCloud(uint vid [[vertex_id]], constant FrameUniforms& f [[buffer(1)]], constant TornadoParams& p [[buffer(2)]]) {
    const uint cell = vid / 6u, k = vid % 6u;
    const uint2 corner[6] = {uint2(0, 0), uint2(1, 0), uint2(1, 1), uint2(0, 0), uint2(1, 1), uint2(0, 1)};
    const float2 g = float2(float(cell % 4u + corner[k].x), float(cell / 4u + corner[k].y)) / 4.0;  // (u, v)
    const float h = p.base.w;
    const float3 local = float3(-h * (2.0 * g.x - 1.0), p.flags.w, h * (2.0 * g.y - 1.0));
    const float s = sin(p.flags.z), c = cos(p.flags.z);
    const float3 w = p.base.xyz + float3(local.x * c + local.z * s, local.y, -local.x * s + local.z * c);
    TornadoOut o;
    o.position = fuelClip(f, w);
    o.uvAlpha = float4(1.0 - g.x, g.y, 1.0, 0.0);
    o.T = float3(1, 0, 0);  // vh_tornadoclouds: the vertex frame as stored (not turned by the world matrix)
    o.N = float3(0, -1, 0);
    o.B = cross(o.N, o.T);
    const float3 vec = f.eye.xyz - w;
    const float len = length(vec);
    o.eyevec = float4(vec / max(len, 1e-3), 1.0 - saturate(len / 500.0));
    const Scatter sc = fuelScattering(f, w);
    o.ext = sc.ext;
    o.insc = sc.insc;
    o.viewZ = tornadoViewZ(f, w);
    return o;
}

fragment float4 fsTornadoCloud(TornadoOut in [[stage_in]], depth2d<float> depth [[texture(0)]], texture2d<float> tDiffuse [[texture(1)]],
                               texture2d<float> tNormal [[texture(2)]], constant FrameUniforms& f [[buffer(1)]],
                               constant TornadoParams& p [[buffer(2)]]) {
    if (in.viewZ > tornadoSceneZ(f, depth, in.position)) discard_fragment();
    constexpr sampler smp(coord::normalized, filter::linear, mip_filter::linear, address::repeat);
    const float4 diffuse = tDiffuse.sample(smp, in.uvAlpha.xy);
    const float3 t = tornadoNormal(tNormal, smp, in.uvAlpha.xy, p.flags.x > 0.5);
    float3 n = normalize(t.x * in.T + t.y * in.B + t.z * in.N);
    n.y = -n.y;
    const float3 eye = normalize(in.eyevec.xyz);
    float3 c = tornadoLighting(f, n, normalize(f.lightDir.xyz), eye, diffuse.rgb);
    c = c * in.ext + in.insc;
    return float4(max(c, 0.0), saturate(diffuse.a));
}

struct TornadoSmokeIn {
    float y, diam, angle, rot, size, alpha;
};

struct TornadoSmokeOut {
    float4 position [[position]];
    float2 uv;
    float alpha;
    float3 normal;
    float3 ext;
    float3 insc;
    float viewZ;
};

vertex TornadoSmokeOut vsTornadoSmoke(uint vid [[vertex_id]], uint iid [[instance_id]], const device TornadoSmokeIn* ps [[buffer(0)]],
                                      constant FrameUniforms& f [[buffer(1)]], constant TornadoParams& p [[buffer(2)]]) {
    const TornadoSmokeIn s = ps[iid];
    // FromAxisAngle((0, 1, 0), angle) x (-1, 0, 0), pushed out by the diameter (TornadoSmokeVertex_Z).
    const float3 normal = normalize(float3(-cos(s.angle), 0.0, sin(s.angle)));
    const float3 centre = p.base.xyz + float3(0.0, s.y, 0.0);
    float3 w = centre + normal * s.diam;
    const float edge = sqrt(saturate(dot(normalize(w - f.eye.xyz), normalize(centre - w))));
    // 0x5e8ca0's corners: (u0, v1) (u0, v0) (u1, v1) (u1, v0); partScale = (uv x 2 - 1) x size over a whole texture.
    const float2 corner = float2(vid >> 1, (vid & 1) ^ 1) * 2.0 - 1.0;
    const float2 ps2 = corner * s.size;
    const float sn = sin(s.rot), cs = cos(s.rot);
    const float2 off = edge * float2(ps2.x * cs - ps2.y * sn, ps2.x * sn + ps2.y * cs);
    const float3 sprite = w + off.x * f.camRight.xyz + off.y * f.camUp.xyz;
    TornadoSmokeOut o;
    o.position = fuelClip(f, sprite);
    o.uv = float2(p.uvRect.x + (corner.x * 0.5 + 0.5) * p.uvRect.z, p.uvRect.y + (0.5 - corner.y * 0.5) * p.uvRect.w);
    o.alpha = s.alpha * edge;
    o.normal = normal;
    const Scatter sc = fuelScattering(f, w);
    o.ext = sc.ext;
    o.insc = sc.insc;
    o.viewZ = tornadoViewZ(f, sprite);
    return o;
}

fragment float4 fsTornadoSmoke(TornadoSmokeOut in [[stage_in]], depth2d<float> depth [[texture(0)]], texture2d<float> tDiffuse [[texture(1)]],
                               constant FrameUniforms& f [[buffer(1)]]) {
    constexpr sampler smp(coord::normalized, filter::linear, mip_filter::linear, address::clamp_to_edge);
    const float4 tex = tDiffuse.sample(smp, in.uv);
    float3 c = 0.3 * tex.rgb * 2.0;
    float a = in.alpha * tex.a;
    float3 n = in.normal;
    n.y = 0.0;
    n = normalize(n);
    c = fuelLighting(n, f.lightDir.xyz, f.skyDir.xyz, f.lightDir.xyz, f.sunColour.rgb, f.skyColour.rgb, c, float3(0.0), 1.0, 1.0,
                     float3(0.0), 1.0);
    c = c * in.ext + in.insc;
    // Soft over 50 m; behind the scene only where the scene is nearer than 2 m.
    const float scene = tornadoSceneZ(f, depth, in.position);
    float az = saturate((scene - in.viewZ) / 50.0);
    if (scene < in.viewZ) az = scene < 2.0 ? 1.0 : 0.0;
    a *= az;
    return float4(max(c, 0.0), saturate(a));
}

// ---- P65: the wind (SubCode 0x19, vh_wind / ph_wind) and tornado debris (0x1a, vh_debris / ph_sfxScattering) tanks.
struct WindTankParticle {
    float4 pos;     // unit position, w the wave phase
    float4 extra;   // x: atlas column 0..3
};

struct WindTankParams {
    float4 user1;   // C_USER1: displacement xyz, TankScale
    float4 user2;   // C_USER2: ParticleScale, WaveFrequency, WaveAmplitude, DistanceScaleRatio
    float4 uv;      // the tank atlas rect (u0, v0, du, dv)
    float4 mode;    // x: 0 wind, 1 debris
    float4 smooth;  // P90: the smooth mode's displacement xyz (tank units = m / TankScale), w its clock (s)
    float4 swirl;   // P90: swirl radius (m), swirl rate (rad/s), face fade width (m), w the weight of FUEL's motion (1: FUEL)
};

struct WindTankOut {
    float4 position [[position]];
    float2 uv;
    float3 sun;
    float3 insc;
    float viewZ;
    float debris;
    float alpha;
};

// MapTankCoordinate (wind.vhl): wrap into the box of half size TankScale round adj; `local` the place in it (-1, 1).
float3 windTankMap(float3 current, float3 adj, float scale, thread float3& local) {
    const float3 m = current - adj / scale;
    const float3 sg = sign(m);
    local = fmod(m + sg, 2.0) - sg;
    return local * scale + adj;
}

vertex WindTankOut vsWindTank(uint vid [[vertex_id]], uint iid [[instance_id]], const device WindTankParticle* ps [[buffer(0)]],
                              constant FrameUniforms& f [[buffer(1)]], constant WindTankParams& p [[buffer(2)]]) {
    const WindTankParticle q = ps[iid];
    const float scale = p.user1.w;
    const float3 fwd = cross(f.camUp.xyz, f.camRight.xyz);
    const float3 adj = f.eye.xyz + fwd * scale;  // C_USER0: the camera + its rotation x (0, 0, TankScale) (vf10)
    const float A = p.user2.z, F = p.user2.y;
    const float k = p.swirl.w;  // P90: 1 = FUEL's motion exactly, 0 = the smooth deviation, between: mixed
    float3 local;
    // vh_wind / vh_debris: FUEL's motion.
    float3 posF = float3(0.0);
    float angleF = 0.0;
    if (k > 0.0) {
        const float3 current = q.pos.xyz + p.user1.xyz;
        posF = windTankMap(current, adj, scale, local);
        angleF = F * length(current) + F * q.pos.w;
        const float s = sin(angleF), c = cos(angleF);
        posF += A * float3(c - s, s - c, s + c);
    }
    // P90 [DEVIATION from FUEL] (core/wind_tank.cpp WindTank::placeSmooth): the drift in metres, FUEL's 40 m wave held
    // at its starting phase plus a small swirl that also spins the sprite, and a fade towards the box faces.
    float3 posS = float3(0.0);
    float angleS = 0.0, alphaS = 1.0;
    if (k < 1.0) {
        posS = windTankMap(q.pos.xyz + p.smooth.xyz, adj, scale, local);
        const float3 al = abs(local);
        alphaS = p.swirl.z > 0.0 ? saturate((1.0 - max(al.x, max(al.y, al.z))) * scale / p.swirl.z) : 1.0;
        const float a0 = F * length(q.pos.xyz) + F * q.pos.w;
        const float rate = p.swirl.y * (0.5 + abs(q.pos.w)) * (q.pos.w < 0.0 ? -1.0 : 1.0);
        angleS = a0 + rate * p.smooth.w;
        const float s0 = sin(a0), c0 = cos(a0), s = sin(angleS), c = cos(angleS);
        posS += A * float3(c0 - s0, s0 - c0, s0 + c0) + p.swirl.x * float3(c - s, s - c, s + c);
    }
    const float3 pos = k >= 1.0 ? posF : (k <= 0.0 ? posS : mix(posS, posF, k));
    const float angle = k >= 0.5 ? angleF : angleS;
    const float s = sin(angle), c = cos(angle);
    const float vz = dot(pos - f.eye.xyz, fwd);
    float ps2 = p.user2.x * (1.0 + p.user2.w * abs(vz / scale));
    // uvscale.zw: (-1, -1) (-1, 1) (1, -1) (1, 1); uv (u, v0) (u, v1) (u + 1/4, v0) (u + 1/4, v1).
    const float2 corner = float2(vid >> 1, vid & 1) * 2.0 - 1.0;
    const float2 off = ps2 * float2(corner.x * c - corner.y * s, corner.x * s + corner.y * c);
    const float3 w = pos + off.x * f.camRight.xyz + off.y * f.camUp.xyz;
    WindTankOut o;
    o.position = fuelClip(f, w);
    o.uv = float2(q.extra.x * 0.25 + (corner.x > 0.0 ? 0.25 : 0.0), p.uv.y + (corner.y > 0.0 ? p.uv.w : 0.0));
    o.viewZ = dot(w - f.eye.xyz, fwd);
    o.debris = p.mode.x;
    o.alpha = k >= 1.0 ? 1.0 : mix(alphaS, 1.0, k);
    if (p.mode.x > 0.5) {
        // P90: vh_debris + ph_sfxScattering (wind.vhl, sfx.phl, scattering.h) exactly: scattering(pos, 1, C_SCATTERCONSTANTS3)
        // gives sunlight = DLIGHT_COLOR x C_SCATTERCONSTANTS1.z (f.sunColour), skylight = DSKY_COLOR (w 1) and the
        // inscatter; the pixel shader does colour x sunlight + inscatter. The extinction is computed but never read,
        // there is no sky term and no N.L. (The bShadowMap variant would lerp(skylight, sunlight, the vertex shadow),
        // but vh_debris never writes the shadowtcproj0..3 it would read; the port takes the sun-lit path, P88 / P90.)
        o.sun = f.sunColour.rgb;
        o.insc = fuelScattering(f, pos).insc;
    } else {
        o.sun = float3(8.0);  // ph_wind: diffuse (1) x tex x 2 x 4, no lighting
        o.insc = float3(0.0);
    }
    return o;
}

fragment float4 fsWindTank(WindTankOut in [[stage_in]], depth2d<float> depth [[texture(0)]], texture2d<float> tex [[texture(1)]],
                           constant FrameUniforms& f [[buffer(1)]]) {
    if (in.viewZ > tornadoSceneZ(f, depth, in.position)) discard_fragment();
    constexpr sampler smp(coord::normalized, filter::linear, mip_filter::linear, address::clamp_to_edge);
    const float4 t = tex.sample(smp, in.uv);
    if (in.debris > 0.5 && t.a < 0.01) discard_fragment();  // sfxClip
    // ph_wind / ph_sfxScattering: the alpha is the texture's (the vertex alpha of vh_wind / vh_debris is not read).
    // P90: in.alpha is the smooth mode's face fade (1 in FUEL's motion), applied after sfxClip like a blend factor.
    return float4(max(t.rgb * in.sun + in.insc, 0.0), saturate(t.a * in.alpha));
}
// ---- P86: Navigator3dHUD's GPS arrows (SpecialEffectsRibbon_G, SubCode 0x1e): sfx.vhl vh_billboard (the strip's
// vertices are world positions from 0x5fba20, angle 0 and uvscale.zw 0, so no camera offset) and sfx.phl
// ph_billboard: colour = vertex rgba x texture, rgb x 2, sfxClip. No lighting, no scattering.
struct GpsRibbonVertex {
    float4 pos;  // xyz world, w alpha
    float4 uv;   // xy uv
};

struct GpsRibbonOut {
    float4 position [[position]];
    float2 uv;
    float alpha;
    float viewZ;
    float debug;
};

vertex GpsRibbonOut vsGpsRibbon(uint vid [[vertex_id]], const device GpsRibbonVertex* vs [[buffer(0)]],
                                constant FrameUniforms& f [[buffer(1)]]) {
    const GpsRibbonVertex v = vs[vid];
    GpsRibbonOut o;
    o.position = fuelClip(f, v.pos.xyz);
    o.uv = v.uv.xy;
    o.alpha = v.pos.w;
    o.viewZ = tornadoViewZ(f, v.pos.xyz);
    o.debug = v.uv.z;
    return o;
}

fragment float4 fsGpsRibbon(GpsRibbonOut in [[stage_in]], depth2d<float> depth [[texture(0)]], texture2d<float> tex [[texture(1)]],
                            constant FrameUniforms& f [[buffer(1)]]) {
    if (in.viewZ > tornadoSceneZ(f, depth, in.position)) discard_fragment();
    if (in.debug > 0.5) return float4(4.0, 0.0, 4.0, 1.0);  // FUEL_SFX_DEBUG=2: solid magenta strips
    constexpr sampler smp(coord::normalized, filter::linear, mip_filter::linear, address::clamp_to_edge);
    const float4 t = tex.sample(smp, in.uv);
    const float a = t.a * in.alpha;
    if (a < 0.01) discard_fragment();  // sfxClip
    return float4(t.rgb * 2.0, saturate(a));
}

// PK19: the navigator beams (0x4bf820): the same billboard strip, the particle's colour (rgb x 8, alpha) times the
// texel, rgb x 2 (ph_billboard). beam-colour: the uv is the vertex's own (u = 0.05 + 0.9 k / (n - 1), 0 at the icon's
// ground and n - 1 at the band; v 0 / 1): 0x5fba20 writes particle +0x60 + +0x5c k / (n - 1) and vh_billboard passes
// uvscale.xy through, so FUEL never applies the FX atlas rect of entry 0x1d (u0 1, du -1: mirrored) to a ribbon.
// Remapping by it flipped u: the beam was dim and transparent at the band (texel ~0.1, alpha ~0.12) and bright at the
// ground, where FUEL's is brightest and most opaque at the band (texel 0.84, alpha 0.91; ref-video m_14 / m_16).
struct NavBeamParams {
    float4 colour;
};
fragment float4 fsNavBeam(GpsRibbonOut in [[stage_in]], depth2d<float> depth [[texture(0)]], texture2d<float> tex [[texture(1)]],
                          constant FrameUniforms& f [[buffer(1)]], constant NavBeamParams& p [[buffer(2)]]) {
    if (in.viewZ > tornadoSceneZ(f, depth, in.position)) discard_fragment();
    if (in.debug > 0.5) return float4(0.0, 4.0, 4.0, 1.0);  // FUEL_SFX_DEBUG=2: solid cyan strips
    constexpr sampler smp(coord::normalized, filter::linear, mip_filter::linear, address::clamp_to_edge);
    const float4 t = tex.sample(smp, in.uv);
    const float a = t.a * p.colour.a * in.alpha;
    if (a < 0.01) discard_fragment();  // sfxClip
    return float4(t.rgb * p.colour.rgb * 2.0, saturate(a));
}
)METAL";

void MeshRenderer::makeTornadoPipelines() {
    auto make = [&](NSString* vs, NSString* fs) -> id<MTLRenderPipelineState> {
        MTLRenderPipelineDescriptor* d = [MTLRenderPipelineDescriptor new];
        d.vertexFunction = [shaders_ newFunctionWithName:vs];
        d.fragmentFunction = [shaders_ newFunctionWithName:fs];
        d.colorAttachments[0].pixelFormat = kRenderColorFormat;
        d.colorAttachments[0].blendingEnabled = YES;  // alpha blending, the destination alpha kept
        d.colorAttachments[0].sourceRGBBlendFactor = MTLBlendFactorSourceAlpha;
        d.colorAttachments[0].destinationRGBBlendFactor = MTLBlendFactorOneMinusSourceAlpha;
        d.colorAttachments[0].sourceAlphaBlendFactor = MTLBlendFactorZero;
        d.colorAttachments[0].destinationAlphaBlendFactor = MTLBlendFactorOne;
        NSError* e = nil;
        id<MTLRenderPipelineState> ps = [device_ newRenderPipelineStateWithDescriptor:d error:&e];
        if (ps == nil) [NSException raise:@"pipeline" format:@"%@", e];
        return ps;
    };
    tornadoPipeline_ = make(@"vsTornado", @"fsTornado");
    tornadoCloudPipeline_ = make(@"vsTornadoCloud", @"fsTornadoCloud");
    tornadoSmokePipeline_ = make(@"vsTornadoSmoke", @"fsTornadoSmoke");
    windTankPipeline_ = make(@"vsWindTank", @"fsWindTank");
    gpsRibbonPipeline_ = make(@"vsGpsRibbon", @"fsGpsRibbon");  // P86
    navBeamPipeline_ = make(@"vsGpsRibbon", @"fsNavBeam");     // PK19
}

// PK06: the tank wind vector (+0x20, the weather's +0xc4 after the gusts) of the last frame, for StormView's wind-gust
// emitter (0x5e4d00 reads it from the same tank).
simd_float3 gStormWind = {0, 0, 0};

void MeshRenderer::encodeWindTanks(id<MTLCommandBuffer> cb, float dt) {
    if (!lastFrame_ || !weatherFields_ || windTankPipeline_ == nil || std::getenv("FUEL_NO_WIND_TANKS")) return;
    auto P = [this](int n) { return weatherFields_->P(n); };
    if (!windTank_) {  // 0x5bb680 starts both at load
        windTank_ = std::make_unique<storm::WindTank>(storm::WindTankLook::load(P, false), 0x5e4560u);
        debrisTank_ = std::make_unique<storm::WindTank>(storm::WindTankLook::load(P, true), 0x5e71a0u);
        auto upload = [&](const storm::WindTank& t) {
            std::vector<simd_float4> v;
            for (const auto& q : t.particles()) v.push_back(simd_make_float4(q.x, q.y, q.z, q.w)), v.push_back(simd_make_float4(q.column, 0, 0, 0));
            return v.empty() ? nil : [device_ newBufferWithBytes:v.data() length:v.size() * sizeof(simd_float4) options:MTLResourceStorageModeShared];
        };
        windTankBuf_ = upload(*windTank_), debrisTankBuf_ = upload(*debrisTank_);
    }
    const std::array<float, 3> wind = {wind_.x, wind_.y, wind_.z};  // tank +0x20 = the weather wind +0xc4 (0x5bba30)
    gStormWind = wind_;
    windTank_->update(dt, wind);
    debrisTank_->update(dt, wind);
    const std::array<float, 2> ab = storm::windTankAmounts(weatherNow_.windSpeed, P(5460));
    const int nWind = windTank_->drawn(ab[0]), nDebris = debrisTank_->drawn(ab[1]);
    // P90 [DEVIATION]: the debris motion (settings.txt wind_debris, FUEL_WIND_DEBRIS overrides) and FUEL's weight by
    // the camera's x z distance to the nearest tornado base (FUEL's whirl near a funnel, smooth away from one).
    static const char* envMotion = std::getenv("FUEL_WIND_DEBRIS");
    const storm::DebrisMotion motion = envMotion ? storm::debrisMotionFrom(envMotion, windDebris) : windDebris;
    storm::SmoothDebris smooth;
    if (static const char* tz = std::getenv("FUEL_DEBRIS_TORNADO_RANGE"); tz)  // diagnostics: NEAR,FAR metres of FUEL's weight
        std::sscanf(tz, "%f,%f", &smooth.fuelNear, &smooth.fuelFar);
    simd_float3 camEye;
    {
        simd_float4x4 vp;
        simd_float3 r, u;
        frameCameraOf(*lastFrame_, vp, camEye, r, u);
    }
    float tornadoDist = -1;
    for (const TornadoDraw& t : tornadoes_) {
        const float d = float(std::hypot(t.base[0] - camEye.x, t.base[2] - camEye.z));
        if (tornadoDist < 0 || d < tornadoDist) tornadoDist = d;
    }
    const float fuelWeight = storm::debrisFuelWeight(motion, tornadoDist, smooth);
    if (std::getenv("FUEL_SFX_DEBUG") && frame_ % 30 == 0)
        std::fprintf(stderr, "p65 wind tanks: wind %.1f m/s -> wind %d, debris %d particles\n", weatherNow_.windSpeed, nWind, nDebris);
    if (static const bool trace = std::getenv("FUEL_TANK_TRACE") != nullptr; trace) {  // P88: per-frame particle trace
        static bool once = false;
        if (!once) {
            once = true;
            std::fprintf(stderr, "p88 pp:");
            for (int n : {5270, 5271, 5272, 5273, 5274, 5275, 5276, 5460, 5500, 5501, 5502, 5503, 5504, 5505, 5506}) std::fprintf(stderr, " %d=%g", n, P(n));
            std::fprintf(stderr, "\n");
        }
        simd_float4x4 vp;
        simd_float3 eye, right, up;
        frameCameraOf(*lastFrame_, vp, eye, right, up);
        const simd_float3 fwd = simd_cross(up, right);
        std::fprintf(stderr, "p88 frame %llu dt %.4f wind %.2f (%.2f %.2f %.2f) n %d %d eye %.2f %.2f %.2f fwd %.3f %.3f %.3f\n", (unsigned long long)frame_, dt,
                     weatherNow_.windSpeed, wind[0], wind[1], wind[2], nWind, nDebris, eye.x, eye.y, eye.z, fwd.x, fwd.y, fwd.z);
        std::fprintf(stderr, "p90 motion %s tornado %.1f m fuel weight %.3f\n", storm::debrisMotionName(motion), tornadoDist, fuelWeight);
        if (frame_ % 60 == 0) {  // P90: the debris light, FUEL's (sunlight) and the port's before P90 ((sky + sun sat(L.y)) x ext)
            const Atmosphere& at = atmo_;
            const float ly = std::clamp(float(at.lightDir[1]), 0.0f, 1.0f);
            std::fprintf(stderr, "p90 light: sunlight %.3f %.3f %.3f, sky %.3f %.3f %.3f, L.y %.3f -> before (sky + sun L.y) %.3f %.3f %.3f (x ext)\n",
                         float(at.sunColour[0]), float(at.sunColour[1]), float(at.sunColour[2]), float(at.dskyColor[0]), float(at.dskyColor[1]),
                         float(at.dskyColor[2]), ly, float(at.dskyColor[0] + at.sunColour[0] * ly), float(at.dskyColor[1] + at.sunColour[1] * ly),
                         float(at.dskyColor[2] + at.sunColour[2] * ly));
        }
        for (int tk = 0; tk < 2; ++tk) {
            const storm::WindTank& t = tk ? *debrisTank_ : *windTank_;
            const simd_float3 a = eye + fwd * t.look().scale;
            const auto& d = t.displacement();
            const int nTrace = std::getenv("FUEL_TANK_TRACE") ? std::max(4, std::atoi(std::getenv("FUEL_TANK_TRACE"))) : 4;  // P90: =N traces N
            for (int k = 0; k < nTrace && k < int(t.particles().size()); ++k) {
                // P90: the debris tank's trace follows the drawn motion (the smooth one only when FUEL's weight is 0).
                const auto pl = (tk && fuelWeight <= 0) ? t.placeSmooth(t.particles()[size_t(k)], {a.x, a.y, a.z}, smooth)
                                                        : t.place(t.particles()[size_t(k)], {a.x, a.y, a.z});
                const simd_float4 c = simd_mul(vp, simd_make_float4(pl.pos[0], pl.pos[1], pl.pos[2], 1));
                std::fprintf(stderr, "p88 %s %d disp %.3f %.3f %.3f pos %.2f %.2f %.2f cell %d %d %d ang %.3f ndc %.4f %.4f w %.2f a %.3f\n", tk ? "debris" : "wind", k, d[0],
                             d[1], d[2], pl.pos[0], pl.pos[1], pl.pos[2], pl.cell[0], pl.cell[1], pl.cell[2], pl.angle, c.x / c.w, c.y / c.w, c.w, pl.alpha);
            }
        }
    }
    if (nWind <= 0 && nDebris <= 0) return;
    id<MTLRenderCommandEncoder> enc = fxEncoder(cb, "fx.wind");  // P92: the shared effects pass (fxEncoder)
    [enc setRenderPipelineState:windTankPipeline_];
    [enc setVertexBytes:lastFrame_.get() length:kFrameUniformsSize atIndex:1];
    [enc setFragmentBytes:lastFrame_.get() length:kFrameUniformsSize atIndex:1];
    [enc setFragmentTexture:depth_ atIndex:0];
    struct Params {
        simd_float4 user1, user2, uv, mode, smooth, swirl;
    };
    auto draw = [&](const storm::WindTank& t, id<MTLBuffer> buf, id<MTLTexture> tex, int n, float mode) {
        if (n <= 0 || buf == nil || tex == nil) return;
        const storm::WindTankLook& l = t.look();
        simd_float4 rect = simd_make_float4(0, 0, 1, 1);
        if (sfxTables_ && size_t(l.atlas) < sfxTables_->size()) {
            const auto& e = (*sfxTables_)[size_t(l.atlas)];
            rect = simd_make_float4(e.u0, e.v0, e.du, e.dv);
        }
        const auto& d = t.displacement();
        const auto& sd = t.smoothDisplacement();
        // P90: the wind tank (mode 0) is always FUEL's motion; the debris tank's by `fuelWeight`.
        Params p{simd_make_float4(d[0], d[1], d[2], l.scale), simd_make_float4(l.particle, l.frequency, l.amplitude, l.distRatio), rect,
                 simd_make_float4(mode, 0, 0, 0), simd_make_float4(sd[0], sd[1], sd[2], t.smoothClock()),
                 simd_make_float4(smooth.swirlRadius, smooth.swirlRate, smooth.fadeWidth, mode > 0.5f ? fuelWeight : 1.0f)};
        [enc setVertexBuffer:buf offset:0 atIndex:0];
        [enc setVertexBytes:&p length:sizeof p atIndex:2];
        [enc setFragmentTexture:tex atIndex:1];
        [enc drawPrimitives:MTLPrimitiveTypeTriangleStrip vertexStart:0 vertexCount:4 instanceCount:NSUInteger(n)];
    };
    static const int hide = std::getenv("FUEL_TANK_HIDE") ? std::atoi(std::getenv("FUEL_TANK_HIDE")) : 0;  // P88 diagnostics: 1 wind, 2 debris
    if (!(hide & 1)) draw(*windTank_, windTankBuf_, windTankTex_[0], nWind, 0.0f);
    if (!(hide & 2)) draw(*debrisTank_, debrisTankBuf_, windTankTex_[1], nDebris, 1.0f);
}

// P86: the GPS arrows. 0x5fba20 builds each ribbon as a strip: at point k the side s = normalise(up x (p[k+1] - p[k]))
// (the last point keeps the previous side), scaled by the size +0x58 (2.25, 0x4bbba0) x Fade_One; vertices p + s with
// uv (u, 0) and p - s with (u, 1), u = +0x60 + +0x5c k / (n - 1) = 0.25 k / 3 (flags 0x20: no taper, no length u);
// 0x5fae60 joins the ribbons with degenerate vertices into one strip.
void MeshRenderer::encodeGpsRibbons(id<MTLCommandBuffer> cb) {
    if (!lastFrame_ || gpsRibbonPipeline_ == nil || gpsArrowTex_ == nil || gpsRibbons.empty()) return;
    struct V {
        simd_float4 pos, uv;
    };
    static const float debug = std::getenv("FUEL_SFX_DEBUG") && std::atoi(std::getenv("FUEL_SFX_DEBUG")) == 2 ? 1.0f : 0.0f;
    std::vector<V> strip;
    for (const GpsRibbonDraw& r : gpsRibbons) {
        const size_t n = r.points.size();
        if (n < 2 || r.alpha <= 0) continue;
        const simd_float3 up = simd_make_float3(r.up[0], r.up[1], r.up[2]);
        simd_float3 side = simd_make_float3(0, 0, 0);
        std::vector<V> rib;
        for (size_t k = 0; k < n; ++k) {
            const simd_float3 p = simd_make_float3(r.points[k][0], r.points[k][1], r.points[k][2]);
            if (k + 1 < n) {
                const simd_float3 d = simd_make_float3(r.points[k + 1][0], r.points[k + 1][1], r.points[k + 1][2]) - p;
                const float l = simd_length(d);
                if (l > 1e-4f) {
                    const simd_float3 c = simd_cross(up, d / l);
                    const float cl = simd_length(c);
                    side = cl > 1e-6f ? c / cl : c;
                } else if (k < 2) {
                    side = simd_make_float3(0, 0, 0);
                }
            }
            const simd_float3 w = side * 2.25f;
            const float u = 0.25f * float(k) / float(n - 1);
            rib.push_back({simd_make_float4(p + w, r.alpha), simd_make_float4(u, 0, debug, 0)});
            rib.push_back({simd_make_float4(p - w, r.alpha), simd_make_float4(u, 1, debug, 0)});
        }
        if (!strip.empty()) strip.push_back(strip.back()), strip.push_back(rib.front());  // degenerate join
        strip.insert(strip.end(), rib.begin(), rib.end());
    }
    if (std::getenv("FUEL_GPS_TRACE"))
        for (const GpsRibbonDraw& r : gpsRibbons)
            std::fprintf(stderr, "p86 gps arrow at (%.0f, %.0f, %.0f) up (%.2f, %.2f, %.2f) alpha %.2f, %zu points\n", r.points[0][0], r.points[0][1],
                         r.points[0][2], r.up[0], r.up[1], r.up[2], r.alpha, r.points.size());
    if (strip.size() < 4) return;
    id<MTLBuffer> buf = [device_ newBufferWithBytes:strip.data() length:strip.size() * sizeof(V) options:MTLResourceStorageModeShared];
    id<MTLRenderCommandEncoder> enc = fxEncoder(cb, "fx.gps");  // P92: the shared effects pass (fxEncoder)
    [enc setRenderPipelineState:gpsRibbonPipeline_];
    [enc setCullMode:MTLCullModeNone];
    [enc setVertexBuffer:buf offset:0 atIndex:0];
    // TAA: the arrows are a 3-D HUD element: drawn without the temporal jitter (stableFrame(): lastFrame_ when off) so
    // they keep still on screen; the reactive mask gives them this frame's pixels.
    [enc setVertexBytes:stableFrame() length:kFrameUniformsSize atIndex:1];
    [enc setFragmentBytes:stableFrame() length:kFrameUniformsSize atIndex:1];
    [enc setFragmentTexture:depth_ atIndex:0];
    [enc setFragmentTexture:gpsArrowTex_ atIndex:1];
    [enc drawPrimitives:MTLPrimitiveTypeTriangleStrip vertexStart:0 vertexCount:strip.size()];
}

// PK19: the navigator beams. 0x5fae60 without flag 0x20 gives every particle the manager's +0xb8 as its bank (+0xbc) [I:
// the camera's view direction], so 0x5fba20's side = normalise(up x (p[k+1] - p[k])) faces the camera; x the half width
// +0x58 (0.008 d) x Fade_One (1); vertices p + side (u, 0) and p - side (u, 1), u = +0x60 + +0x5c k / (n - 1) = 0.05 +
// 0.9 k / 3 (flags 0: no taper, no length u).
void MeshRenderer::encodeNavBeams(id<MTLCommandBuffer> cb) {
    if (!lastFrame_ || navBeamPipeline_ == nil || navLineTex_ == nil || navBeams.empty()) return;
    struct V {
        simd_float4 pos, uv;
    };
    struct Params {
        simd_float4 colour;
    };
    static const float debug = std::getenv("FUEL_SFX_DEBUG") && std::atoi(std::getenv("FUEL_SFX_DEBUG")) == 2 ? 1.0f : 0.0f;
    simd_float4x4 vp;
    simd_float3 eye, right, up;
    frameCameraOf(*lastFrame_, vp, eye, right, up);
    const simd_float3 fwd = simd_cross(up, right);
    id<MTLRenderCommandEncoder> enc = nil;
    for (const NavBeamDraw& b : navBeams) {
        const size_t n = b.points.size();
        if (n < 2 || b.colour[3] <= 0 || b.halfWidth <= 0) continue;
        std::vector<V> strip;
        simd_float3 side = simd_make_float3(0, 0, 0);
        for (size_t k = 0; k < n; ++k) {
            const simd_float3 p = simd_make_float3(b.points[k][0], b.points[k][1], b.points[k][2]);
            if (k + 1 < n) {
                const simd_float3 d = simd_make_float3(b.points[k + 1][0], b.points[k + 1][1], b.points[k + 1][2]) - p;
                const float l = simd_length(d);
                if (l > 1e-4f) {
                    const simd_float3 c = simd_cross(fwd, d / l);
                    const float cl = simd_length(c);
                    side = cl > 1e-6f ? c / cl : c;
                } else if (k < 2) {
                    side = simd_make_float3(0, 0, 0);
                }
            }
            const simd_float3 w = side * b.halfWidth;
            const float u = sfx::navBeamU(k, n);  // the texture's own u (no atlas rect)
            strip.push_back({simd_make_float4(p + w, 1.0f), simd_make_float4(u, 0, debug, 0)});
            strip.push_back({simd_make_float4(p - w, 1.0f), simd_make_float4(u, 1, debug, 0)});
        }
        if (enc == nil) {
            enc = fxEncoder(cb, "fx.navbeams");
            [enc setRenderPipelineState:navBeamPipeline_];
            [enc setCullMode:MTLCullModeNone];
            [enc setVertexBytes:stableFrame() length:kFrameUniformsSize atIndex:1];  // TAA: without the jitter (as the GPS arrows)
            [enc setFragmentBytes:stableFrame() length:kFrameUniformsSize atIndex:1];
            [enc setFragmentTexture:depth_ atIndex:0];
            [enc setFragmentTexture:navLineTex_ atIndex:1];
        }
        const Params prm{simd_make_float4(b.colour[0], b.colour[1], b.colour[2], b.colour[3])};
        [enc setVertexBytes:strip.data() length:strip.size() * sizeof(V) atIndex:0];
        [enc setFragmentBytes:&prm length:sizeof prm atIndex:2];
        [enc drawPrimitives:MTLPrimitiveTypeTriangleStrip vertexStart:0 vertexCount:strip.size()];
    }
    if (std::getenv("FUEL_GPS_TRACE"))
        for (const NavBeamDraw& b : navBeams)
            std::fprintf(stderr, "pk19 nav beam (%.0f, %.0f, %.0f) -> (%.0f, %.0f, %.0f) half %.2f colour %.2f %.2f %.2f\n", b.points.front()[0], b.points.front()[1],
                         b.points.front()[2], b.points.back()[0], b.points.back()[1], b.points.back()[2], b.halfWidth, b.colour[0], b.colour[1], b.colour[2]);
}

void MeshRenderer::encodeTornadoes(id<MTLCommandBuffer> cb) {
    if (tornadoes_.empty() || !lastFrame_ || tornadoPipeline_ == nil || tornadoTex_[0] == nil) return;
    struct Params {
        simd_float4 base, user0, flags, uvRect;
    };
    simd_float4 smokeRect = simd_make_float4(0, 0, 1, 1);
    if (sfxTables_ && sfxTables_->size() > 6) {
        const auto& e = (*sfxTables_)[6];
        smokeRect = simd_make_float4(e.u0, e.v0, e.du, e.dv);
    }
    static const std::vector<uint16_t>& idx = storm::TornadoEffect::funnelIndices();
    static id<MTLBuffer> idxBuf = [device_ newBufferWithBytes:idx.data() length:idx.size() * sizeof(uint16_t) options:MTLResourceStorageModeShared];
    id<MTLRenderCommandEncoder> enc = fxEncoder(cb, "fx.tornado");  // P92: the shared effects pass (fxEncoder)
    [enc setVertexBytes:lastFrame_.get() length:kFrameUniformsSize atIndex:1];
    [enc setFragmentBytes:lastFrame_.get() length:kFrameUniformsSize atIndex:1];
    [enc setFragmentTexture:depth_ atIndex:0];
    for (const TornadoDraw& t : tornadoes_) {  // far to near (StormView sorts)
        // 0x26 cloud tops (CloudTop_G vf04 0x5e5e40).
        if (tornadoTex_[2] != nil) {
            [enc setRenderPipelineState:tornadoCloudPipeline_];
            [enc setFragmentTexture:tornadoTex_[2] atIndex:1];
            [enc setFragmentTexture:(tornadoTex_[3] ? tornadoTex_[3] : tornadoTex_[2]) atIndex:2];
            for (const storm::TornadoCloudTop& c : t.clouds) {
                Params p{simd_make_float4(float(c.centre[0]), float(c.centre[1]), float(c.centre[2]), c.half), simd_make_float4(1, 0, 0, 0),
                         simd_make_float4(tornadoNormalBC3_[1] ? 1.0f : 0.0f, 0, c.angle, c.lift), smokeRect};
                [enc setVertexBytes:&p length:sizeof p atIndex:2];
                [enc setFragmentBytes:&p length:sizeof p atIndex:2];
                [enc drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:0 vertexCount:96];
            }
        }
        // 0x23 the funnel.
        if (!t.funnel.empty()) {
            [enc setRenderPipelineState:tornadoPipeline_];
            id<MTLBuffer> vb = [device_ newBufferWithBytes:t.funnel.data() length:t.funnel.size() * sizeof(storm::TornadoVertex)
                                                   options:MTLResourceStorageModeShared];
            [enc setVertexBuffer:vb offset:0 atIndex:0];
            [enc setFragmentTexture:tornadoTex_[0] atIndex:1];
            [enc setFragmentTexture:(tornadoTex_[1] ? tornadoTex_[1] : tornadoTex_[0]) atIndex:2];
            for (int side = 0; side < 2; ++side) {
                Params p{simd_make_float4(float(t.base[0]), float(t.base[1]), float(t.base[2]), 0), simd_make_float4(1, t.scroll[0], t.scroll[1], 0),
                         simd_make_float4(tornadoNormalBC3_[0] ? 1.0f : 0.0f, float(side), 0, 0), smokeRect};
                [enc setVertexBytes:&p length:sizeof p atIndex:2];
                [enc setFragmentBytes:&p length:sizeof p atIndex:2];
                [enc drawIndexedPrimitives:MTLPrimitiveTypeTriangle indexCount:idx.size() indexType:MTLIndexTypeUInt16 indexBuffer:idxBuf
                         indexBufferOffset:0];
            }
        }
        // 0x25 the base smoke.
        if (!t.smoke.empty() && tornadoTex_[4] != nil) {
            [enc setRenderPipelineState:tornadoSmokePipeline_];
            Params p{simd_make_float4(float(t.base[0]), float(t.base[1]), float(t.base[2]), 0), simd_make_float4(1, 0, 0, 0), simd_make_float4(0),
                     smokeRect};
            id<MTLBuffer> sb = [device_ newBufferWithBytes:t.smoke.data() length:t.smoke.size() * sizeof(storm::TornadoSmoke)
                                                   options:MTLResourceStorageModeShared];
            [enc setVertexBuffer:sb offset:0 atIndex:0];
            [enc setVertexBytes:&p length:sizeof p atIndex:2];
            [enc setFragmentTexture:tornadoTex_[4] atIndex:1];
            [enc drawPrimitives:MTLPrimitiveTypeTriangleStrip vertexStart:0 vertexCount:4 instanceCount:t.smoke.size()];
        }
    }
}

}  // namespace fuel
