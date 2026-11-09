// TAA [port performance option, not a FUEL behaviour]: MetalFX temporal upscaling (settings.txt
// `dynamic_resolution_upscaler = metalfx-temporal`). FUEL renders at the window's size with 4x MSAA and has no temporal
// anti-aliasing; this path lets the port render the 3-D scene below the output size and rebuild the output from the
// jittered frames over time (MTLFXTemporalScaler), as the dynamic resolution's other upscalers do from one frame.
// Notes: ~/re-work/fuel/notes/taa.md. Off (the default upscaler is the sharpened bilinear), none of this runs.
//
// What a temporal frame adds to the renderer's passes:
//  - the scene's projection carries a Halton (2, 3) sub-pixel jitter (renderer.mm taaFrameUniforms); the shadow slices,
//    the cube face, the light gather, the headlight clusters, the CPU culling and the screen-stable 3-D markers (GPS
//    arrows, navigator beams) use the matrix without it; the samples' LOD bias is FUEL's -1 + log2(render scale);
//  - the scene pass ends with a per-sample draw that keeps each pixel's luminance in the HDR alpha (no effect writes
//    the alpha: their blend states keep the destination's), so the velocity pass can tell where the effects changed
//    the picture: the reactive mask (particles, smoke, sparks, rain, splashes, tornado, sand walls, lens drops, blob
//    shadows, GPS arrows: everything drawn after the scene, without motion vectors of its own);
//  - the velocity pass (render size, RG16Float): first the camera motion of every pixel from the resolved depth (terrain
//    with its painted roads, static objects, trees and their cards, markers; the sky at the far plane turns with the
//    camera only), then the moving opaque things over it with their last frame's pose: the player's and the opponents'
//    vehicle parts (wheels included) and riders (the skinned vertex buffer of the last frame), the props, the
//    helicopter, the F14 and cut-scene objects, LOD set 3 (props drawn with their Lod_Z), the animated world objects,
//    the grass (the last frame's sway clock and camera, the cards face it), the lakes (their wave; the water writes
//    no depth, so its test is "in front of the scene's depth") and the opaque RotShape quads (built from the camera:
//    the last frame's view axes and eye). Each object fragment writes only where its own view distance matches the
//    resolved depth's (a moving thing hidden by a tree keeps the tree's motion);
//  - the post chain takes the exposure and the bloom from the render-size HDR as before, then MetalFX temporal turns the
//    HDR scene into the output size, and the tone map, radial blur and grain run there (Apple's order for temporal
//    upscalers: before tone mapping, with the exposure the tone map will use).
#import "renderer.h"
#import "gpu_profile.h"
#import <MetalFX/MetalFX.h>

#include "viewer/taa_math.h"

#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace fuel {

extern const size_t kFrameUniformsSize;  // renderer.mm

const char* kTaaShaderSource = R"METAL(
// ---- TAA (taa_render.mm): the MetalFX temporal scaler's inputs.
struct TaaUniforms {
    float4x4 curVP;      // this frame's viewProj without the jitter (fuelClip form: applied to (w - eye, 0))
    float4x4 prevVP;     // the last frame's
    float4x4 reproj;     // camera motion: this frame's (ndc x, ndc y, depth, 1) -> the static point's last-frame clip
    float4x4 unjit;      // this frame's (ndc x, ndc y, depth, 1) -> its clip position without the jitter
    float4 eye;          // this frame's eye (FrameUniforms.eye)
    float4 prevEye;      // the last frame's
    float4 view;         // xy the render size (pixels), z depth tolerance (share of the view distance), w (metres)
    float4 depth;        // x zn, y q (view distance L = zn q / (d + q)), z reactive gain, w reactive cap
    float4 prevRight;    // the last frame's camRight (grass cards turn about the vertical to face the camera)
    float4 clocks;       // x grass sway clock now, y the last frame's, z sway strength now, w the last frame's
    float4 water;        // x water clock now, y the last frame's
    float4 prevR, prevU, prevF;  // the last frame's view axes (RotShape corners)
};

// Motion in uv of the render area (x right, y down), from this frame's position to the last frame's.
float2 taaMotion(float4 cur, float4 prev) { return (prev.xy / prev.w - cur.xy / cur.w) * float2(0.5, -0.5); }

// The camera motion of the static point a pixel shows: ndc (with the jitter) and its resolved depth (1 = the sky).
float2 taaCameraMotion(constant TaaUniforms& t, float2 ndc, float d) {
    const float4 h = float4(ndc, d, 1.0);
    return taaMotion(t.unjit * h, t.reproj * h);
}

// The view distance of a depth value (fsParticle's form).
float taaViewDistance(constant TaaUniforms& t, float d) { return t.depth.x * t.depth.y / (d + t.depth.y); }

// The scene pass's last draw, per sample (the colour attachment read in the tile): the luminance into the alpha.
fragment float4 fsTaaLumCapture(FullOut in [[stage_in]], float4 c [[color(0)]], uint sid [[sample_id]]) {
    (void)sid;
    return float4(0.0, 0.0, 0.0, dot(c.rgb, float3(0.2125, 0.7154, 0.0721)));
}

struct TaaCameraOut {
    float2 motion [[color(0)]];
    float reactive [[color(1)]];
};
fragment TaaCameraOut fsTaaCamera(FullOut in [[stage_in]], depth2d<float> depth [[texture(0)]], texture2d<float> hdr [[texture(1)]],
                                  constant TaaUniforms& t [[buffer(0)]]) {
    const uint2 p = uint2(in.position.xy);
    const float2 ndc = float2(in.position.x / t.view.x * 2.0 - 1.0, 1.0 - in.position.y / t.view.y * 2.0);
    TaaCameraOut o;
    o.motion = taaCameraMotion(t, ndc, depth.read(p));
    // The reactive mask: how much the effects changed the pixel's luminance (alpha = the luminance before them).
    const float4 c = hdr.read(p);
    const float L1 = dot(c.rgb, float3(0.2125, 0.7154, 0.0721)), L0 = c.a;
    o.reactive = min(saturate(abs(L1 - L0) / max(max(L1, L0), 1e-4) * t.depth.z), t.depth.w);
    return o;
}

struct TaaObjectOut {
    float4 position [[position]];
    float4 cur;    // clip without the jitter, this frame
    float4 prev;   // the last frame's
    float2 uv;
    float alpha [[flat]];
};
struct TaaDraw {
    uint stride;  // vertex bytes (the position: float3 at 0)
    uint flags;   // 1: the damage morph at buffer 4 (vsMain's P85 stream)
    uint pad0, pad1;
};
float4x4 taaModel(float4x4 m) {  // vsMain's flag slots (fade, decor kind, dirt) cleared
    m[0].w = 0.0;
    m[1].w = 0.0;
    m[2].w = 0.0;
    return m;
}
TaaObjectOut taaObject(constant FrameUniforms& f, constant TaaUniforms& t, float3 w, float3 wp) {
    TaaObjectOut o;
    o.position = fuelClip(f, w);  // the scene's (jittered) raster position
    o.cur = t.curVP * float4(w - t.eye.xyz, 0.0);
    o.prev = t.prevVP * float4(wp - t.prevEye.xyz, 0.0);
    o.uv = float2(0.0);
    o.alpha = 1.0;
    return o;
}
// Rigid and skinned meshes: models[2 i] this frame's world, [2 i + 1] the last frame's; prevVb the last frame's vertices.
vertex TaaObjectOut vsTaaMesh(uint vid [[vertex_id]], uint iid [[instance_id]], const device uchar* vb [[buffer(0)]],
                              constant FrameUniforms& f [[buffer(1)]], const device float4x4* models [[buffer(2)]],
                              constant TaaDraw& dr [[buffer(3)]], const device packed_float3* morph [[buffer(4)]],
                              const device uchar* prevVb [[buffer(5)]], constant TaaUniforms& t [[buffer(6)]]) {
    float3 p = float3(*(const device packed_float3*)(vb + vid * dr.stride));
    float3 pp = float3(*(const device packed_float3*)(prevVb + vid * dr.stride));
    if ((dr.flags & 1u) != 0u) {
        p += float3(morph[vid]);
        pp += float3(morph[vid]);
    }
    const float3 w = (taaModel(models[2 * iid]) * float4(p, 1.0)).xyz;
    const float3 wp = (taaModel(models[2 * iid + 1]) * float4(pp, 1.0)).xyz;
    return taaObject(f, t, w, wp);
}
// Writes where the object is what the scene shows: its view distance within the tolerance of the resolved depth's.
fragment float2 fsTaaObject(TaaObjectOut in [[stage_in]], depth2d<float> depth [[texture(0)]], constant TaaUniforms& t [[buffer(6)]]) {
    const float Ls = taaViewDistance(t, depth.read(uint2(in.position.xy))), Lf = in.cur.w;
    if (abs(Lf - Ls) > max(t.view.w, t.view.z * Lf)) discard_fragment();
    return taaMotion(in.cur, in.prev);
}

// Grass tufts: vsGrass's card with this frame's and the last frame's sway clock and camera.
float3 taaGrassVertex(GrassInstance g, uint c, float3 eye, float3 camRight, float clock, float strength, float radius, float fadeStart) {
    const float3 base = g.posSize.xyz;
    const float dist = distance(base.xz, eye.xz);
    const float size = g.posSize.w * (1.0 - smoothstep(fadeStart, radius, dist));
    float3 right = float3(camRight.x, 0.0, camRight.z);
    right = dot(right, right) > 1e-6 ? normalize(right) : float3(1, 0, 0);
    float3 w = base + right * (kGrassOffset[c].x * size) + float3(0, kGrassOffset[c].y * size, 0);
    if (c == 1 || c == 2) {
        const float s = base.x + base.z;
        const float wb = 0.25 * (sin(s) + 1.0) + 0.25 * (sin(0.25 * s) + 1.0);
        const float amp = float(g.info.y) * 0.01 * size * strength;
        const float sway = wb * sin(clock * 1.9 + s * 0.31) + (1.0 - wb) * sin(clock * 1.3 + s * 0.17);
        w += float3(0.8, 0.0, 0.6) * (sway * amp);
    }
    return w;
}
vertex TaaObjectOut vsTaaGrass(uint vid [[vertex_id]], uint iid [[instance_id]], constant FrameUniforms& f [[buffer(1)]],
                               const device GrassInstance* tufts [[buffer(2)]], constant GrassParams& g [[buffer(3)]],
                               constant TaaUniforms& t [[buffer(6)]]) {
    const GrassInstance tf = tufts[iid];
    const uint c = kGrassCorner[vid];
    const float3 w = taaGrassVertex(tf, c, f.eye.xyz, f.camRight.xyz, t.clocks.x, t.clocks.z, g.a.y, g.a.z);
    const float3 wp = taaGrassVertex(tf, c, t.prevEye.xyz, t.prevRight.xyz, t.clocks.y, t.clocks.w, g.a.y, g.a.z);
    TaaObjectOut o = taaObject(f, t, w, wp);
    const uint tile = tf.info.x;
    o.uv = float2(float(tile & 3u), float(tile >> 2u)) * 0.25 + kGrassUv[c];
    return o;
}
fragment float2 fsTaaGrass(TaaObjectOut in [[stage_in]], depth2d<float> depth [[texture(0)]], texture2d<float> atlas [[texture(1)]],
                           constant TaaUniforms& t [[buffer(6)]]) {
    constexpr sampler smp(coord::normalized, filter::linear, mip_filter::linear, address::clamp_to_edge);
    if (atlas.sample(smp, in.uv).a < 0.4) discard_fragment();  // fsGrass's cut-out
    const float Ls = taaViewDistance(t, depth.read(uint2(in.position.xy))), Lf = in.cur.w;
    if (abs(Lf - Ls) > max(t.view.w, t.view.z * Lf)) discard_fragment();
    return taaMotion(in.cur, in.prev);
}

// Lakes: vsWater's wave at this frame's and the last frame's clock (rivers have none).
vertex TaaObjectOut vsTaaWater(uint vid [[vertex_id]], const device WaterVertex* v [[buffer(0)]], constant FrameUniforms& f [[buffer(1)]],
                               constant float4* W [[buffer(2)]], constant TaaUniforms& t [[buffer(6)]]) {
    float3 w = float3(v[vid].position);
    float3 wp = w;
    if (!(v[vid].kind > 0.5)) {
        w.y -= W[3].y * (sin(-w.x * W[3].x + W[3].z * t.water.x) * 0.5 + 0.5);
        wp.y -= W[3].y * (sin(-wp.x * W[3].x + W[3].z * t.water.y) * 0.5 + 0.5);
    }
    return taaObject(f, t, w, wp);
}
// The water writes no depth (the resolved depth is the bed): it shows where it is in front of the scene's depth.
fragment float2 fsTaaWater(TaaObjectOut in [[stage_in]], depth2d<float> depth [[texture(0)]], constant TaaUniforms& t [[buffer(6)]]) {
    const float Ls = taaViewDistance(t, depth.read(uint2(in.position.xy))), Lf = in.cur.w;
    if (Lf > Ls + max(t.view.w, t.view.z * Lf)) discard_fragment();
    return taaMotion(in.cur, in.prev);
}

// Opaque RotShape quads: vsRotShape's corner from this frame's view and from the last frame's.
float3 taaRotCorner(constant RotRecordGpu& r, float3 R, float3 U, float3 F, float3 eye, uint vid) {
    const float3 d = r.pivot.xyz - eye;
    const float3 pv = float3(dot(R, d), dot(U, d), dot(F, d));
    const float3 up = float3(R.y, U.y, F.y);
    const float3 v = rotShapeCorner(pv, up, r.upLeft.xyz, r.downRight.xyz, r.pivot.w, r.downRight.w, r.upLeft.w > 0.5, vid);
    return eye + R * v.x + U * v.y + F * v.z;
}
struct TaaRotParams {
    float4 uv0, uv1;  // the material's uv transform (MaterialMat: uv0 as MaterialUniforms.uv0, uv1.xy its translation)
};
vertex TaaObjectOut vsTaaRot(uint vid [[vertex_id]], constant FrameUniforms& f [[buffer(1)]], constant RotRecordGpu& r [[buffer(2)]],
                             constant TaaRotParams& m [[buffer(3)]], constant TaaUniforms& t [[buffer(6)]]) {
    const float3 R = float3(f.view[0][0], f.view[1][0], f.view[2][0]);
    const float3 U = float3(f.view[0][1], f.view[1][1], f.view[2][1]);
    const float3 F = float3(f.view[0][2], f.view[1][2], f.view[2][2]);
    const float3 w = taaRotCorner(r, R, U, F, f.eye.xyz, vid);
    const float3 wp = taaRotCorner(r, t.prevR.xyz, t.prevU.xyz, t.prevF.xyz, t.prevEye.xyz, vid);
    TaaObjectOut o = taaObject(f, t, w, wp);
    const float2 UV[4] = {float2(1, 0), float2(0, 0), float2(1, 1), float2(0, 1)};
    const float2 uv = UV[vid & 3u];
    o.uv = float2(dot(float3(uv, 1.0), float3(m.uv0.xz, m.uv1.x)), dot(float3(uv, 1.0), float3(m.uv0.yw, m.uv1.y)));
    o.alpha = r.downRight.w;
    return o;
}
fragment float2 fsTaaRot(TaaObjectOut in [[stage_in]], depth2d<float> depth [[texture(0)]], texture2d<float> tex [[texture(1)]],
                         sampler smp [[sampler(0)]], constant TaaUniforms& t [[buffer(6)]]) {
    if (!(tex.sample(smp, in.uv).a * in.alpha > 0.0)) discard_fragment();  // fsRotShape's ALPHATEST GREATER 0
    const float Ls = taaViewDistance(t, depth.read(uint2(in.position.xy))), Lf = in.cur.w;
    if (abs(Lf - Ls) > max(t.view.w, t.view.z * Lf)) discard_fragment();
    return taaMotion(in.cur, in.prev);
}

// The exposure S of this frame (fsAdapt's 1 x 1 R32Float) into the 1 x 1 R16Float MetalFX reads.
fragment float4 fsTaaExposure(FullOut in [[stage_in]], texture2d<float> S [[texture(0)]]) {
    return float4(S.read(uint2(0, 0)).r, 0.0, 0.0, 1.0);
}

// --rendercheck: in[3 i] = (ndc x, ndc y, depth) of a point in this frame's jittered projection, in[3 i + 1] its world
// position this frame, in[3 i + 2] the last frame's -> out[2 i] = (camera motion, view distance), out[2 i + 1] = (object
// motion, the clip w of this frame).
kernel void kTaaCheck(constant TaaUniforms& t [[buffer(0)]], const device float4* in [[buffer(1)]], device float4* out [[buffer(2)]],
                      uint i [[thread_position_in_grid]]) {
    const float4 h = in[3 * i];
    out[2 * i] = float4(taaCameraMotion(t, h.xy, h.z), taaViewDistance(t, h.z), 0.0);
    const float4 cur = t.curVP * float4(in[3 * i + 1].xyz - t.eye.xyz, 0.0);
    const float4 prev = t.prevVP * float4(in[3 * i + 2].xyz - t.prevEye.xyz, 0.0);
    out[2 * i + 1] = float4(taaMotion(cur, prev), cur.w, 0.0);
}
// ---- TAA end
)METAL";

namespace {

struct TaaUniforms {  // matches the MSL struct
    simd_float4x4 curVP, prevVP, reproj, unjit;
    simd_float4 eye, prevEye, view, depth, prevRight, clocks, water, prevR, prevU, prevF;
};

// Reactive mask: a luminance change of 1 / gain of the larger value gives 1 (gain 2: the effects doubled or halved the
// pixel); capped at 0.9 so some history stays even under a dense effect (MetalFX: 1 = this frame only).
constexpr float kReactiveGain = 2.0f, kReactiveCap = 0.9f;
// Depth match of the object pass: the larger of 4 cm and 1% of the view distance (the resolved depth is the nearest of
// the 4 MSAA samples, up to half a pixel off the pixel centre on a slope; a wheel sits 10+ cm behind its arch).
constexpr float kDepthTolShare = 0.01f, kDepthTolMetres = 0.04f;

// The camera part of the uniforms (shared with --rendercheck).
TaaUniforms cameraUniforms(const simd_float4x4& jitteredVp, const simd_float4x4& vp, const simd_float4x4& prevVp, simd_float3 eye, simd_float3 prevEye,
                           float zn, float q, float W, float H) {
    TaaUniforms tu{};
    tu.curVP = vp, tu.prevVP = prevVp;
    const taa::Reprojection rj = taa::reprojection(jitteredVp, vp, prevVp, zn, q, eye, prevEye);
    tu.reproj = rj.ok ? rj.reproj : matrix_identity_float4x4;
    tu.unjit = rj.ok ? rj.unjit : matrix_identity_float4x4;
    tu.eye = simd_make_float4(eye, 1), tu.prevEye = simd_make_float4(prevEye, 1);
    tu.view = simd_make_float4(W, H, kDepthTolShare, kDepthTolMetres);
    tu.depth = simd_make_float4(zn, q, kReactiveGain, kReactiveCap);
    return tu;
}


}  // namespace

const char* MeshRenderer::upscalerName(Upscaler u) {
    return u == Upscaler::MetalFX ? "metalfx" : u == Upscaler::MetalFXTemporal ? "metalfx-temporal" : "sharpen";
}

MeshRenderer::Upscaler MeshRenderer::upscalerFromName(const std::string& v, Upscaler fallback) {
    if (v == "sharpen") return Upscaler::Sharpen;
    if (v == "metalfx" || v == "metalfx-spatial") return Upscaler::MetalFX;
    if (v == "metalfx-temporal" || v == "temporal") return Upscaler::MetalFXTemporal;
    return fallback;
}

bool MeshRenderer::metalFxTemporalAvailable() const {
    if (@available(macOS 14.4, *)) return [MTLFXTemporalScalerDescriptor supportsDevice:device_];
    return false;
}

void MeshRenderer::makeTaaPipelines() {
    if (taaMeshPipeline_ != nil) return;
    NSError* e = nil;
    auto check = [&](id<MTLRenderPipelineState> ps) {
        if (ps == nil) [NSException raise:@"pipeline" format:@"%@", e];
        ++oneOff_.pipelines;
        return ps;
    };
    {  // the scene pass's last draw: MSAA colour + depth, alpha only
        MTLRenderPipelineDescriptor* d = [MTLRenderPipelineDescriptor new];
        d.vertexFunction = [shaders_ newFunctionWithName:@"vsFull"];
        d.fragmentFunction = [shaders_ newFunctionWithName:@"fsTaaLumCapture"];
        d.rasterSampleCount = sampleCount_;
        d.colorAttachments[0].pixelFormat = kRenderColorFormat;
        d.colorAttachments[0].writeMask = MTLColorWriteMaskAlpha;
        d.depthAttachmentPixelFormat = kRenderDepthFormat;
        taaLumPipeline_ = check([device_ newRenderPipelineStateWithDescriptor:d error:&e]);
    }
    auto velocity = [&](NSString* vs, NSString* fs, bool reactive) {
        MTLRenderPipelineDescriptor* d = [MTLRenderPipelineDescriptor new];
        d.vertexFunction = [shaders_ newFunctionWithName:vs];
        d.fragmentFunction = [shaders_ newFunctionWithName:fs];
        d.colorAttachments[0].pixelFormat = MTLPixelFormatRG16Float;
        d.colorAttachments[1].pixelFormat = MTLPixelFormatR8Unorm;
        if (!reactive) d.colorAttachments[1].writeMask = MTLColorWriteMaskNone;
        return check([device_ newRenderPipelineStateWithDescriptor:d error:&e]);
    };
    taaCameraPipeline_ = velocity(@"vsFull", @"fsTaaCamera", true);
    taaMeshPipeline_ = velocity(@"vsTaaMesh", @"fsTaaObject", false);
    taaGrassPipeline_ = velocity(@"vsTaaGrass", @"fsTaaGrass", false);
    taaWaterPipeline_ = velocity(@"vsTaaWater", @"fsTaaWater", false);
    taaRotPipeline_ = velocity(@"vsTaaRot", @"fsTaaRot", false);
    {
        MTLRenderPipelineDescriptor* d = [MTLRenderPipelineDescriptor new];
        d.vertexFunction = [shaders_ newFunctionWithName:@"vsFull"];
        d.fragmentFunction = [shaders_ newFunctionWithName:@"fsTaaExposure"];
        d.colorAttachments[0].pixelFormat = MTLPixelFormatR16Float;
        taaExposurePipeline_ = check([device_ newRenderPipelineStateWithDescriptor:d error:&e]);
    }
}

// The temporal scaler and its targets for an output of width x height (made at the first temporal frame, again when the
// size changes: a new scaler starts without history). False when MetalFX temporal is unavailable (the sharpened
// bilinear takes over).
bool MeshRenderer::ensureTemporal(NSUInteger width, NSUInteger height) {
    if (fxTemporal_ != nil && temporalW_ == width && temporalH_ == height) return true;
    static const bool noFx = std::getenv("FUEL_NO_METALFX") != nullptr;
    if (fxTemporalFailed_ || noFx || !metalFxTemporalAvailable()) {
        if (!fxTemporalTried_) std::fprintf(stderr, "fuelview: dynamic resolution: MetalFX temporal unavailable, sharpened bilinear upscaler\n");
        fxTemporalTried_ = true;
        return false;
    }
    if (@available(macOS 14.4, *)) {
        makeTaaPipelines();
        const float supMin = [MTLFXTemporalScalerDescriptor supportedInputContentMinScaleForDevice:device_];
        const float supMax = [MTLFXTemporalScalerDescriptor supportedInputContentMaxScaleForDevice:device_];
        MTLFXTemporalScalerDescriptor* d = [MTLFXTemporalScalerDescriptor new];
        d.colorTextureFormat = kRenderColorFormat;  // the HDR scene (before the tone map)
        d.depthTextureFormat = kRenderDepthFormat;
        d.motionTextureFormat = MTLPixelFormatRG16Float;
        d.outputTextureFormat = kRenderColorFormat;
        d.inputWidth = width, d.inputHeight = height;  // the scene targets keep the output size; the content size is per frame
        d.outputWidth = width, d.outputHeight = height;
        d.autoExposureEnabled = NO;  // the port's own exposure (fsAdapt's S) is given per frame
        d.inputContentPropertiesEnabled = YES;  // dynamic resolution
        d.inputContentMinScale = std::max(1.0f, supMin);
        d.inputContentMaxScale = std::min(std::max(supMax, 1.0f), 2.0f);  // render scales 0.5 .. 1 (dynamic_resolution_min >= 0.5)
        d.reactiveMaskTextureEnabled = YES;
        d.reactiveMaskTextureFormat = MTLPixelFormatR8Unorm;
        // Compile the fast upscaler now (one hitch when the option is first used: the menu, or the first frame) instead of
        // running MetalFX's slower interim one for the first seconds (a 105 ms frame and slower frames in the first --perf-check).
        d.requiresSynchronousInitialization = YES;
        id<MTLFXTemporalScaler> sc = [d newTemporalScalerWithDevice:device_];
        if (sc == nil) {
            std::fprintf(stderr, "fuelview: dynamic resolution: MetalFX temporal scaler could not be made, sharpened bilinear upscaler\n");
            fxTemporalFailed_ = fxTemporalTried_ = true;
            return false;
        }
        taaMinScale_ = 1.0f / d.inputContentMaxScale;
        // The scene targets must carry the usage the scaler asks for (ensureTargets adds it; rebuilt once).
        const MTLTextureUsage need = (sc.colorTextureUsage | sc.depthTextureUsage) & ~(MTLTextureUsageRenderTarget | MTLTextureUsageShaderRead);
        if ((need & ~targetExtraUsage_) != 0) {
            targetExtraUsage_ |= need;
            targetWidth_ = targetHeight_ = 0;
            ensureTargets(width, height);
        }
        auto make = [&](MTLPixelFormat pf, NSUInteger w, NSUInteger h, MTLTextureUsage usage) {
            MTLTextureDescriptor* td = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:pf width:w height:h mipmapped:NO];
            td.usage = usage;
            td.storageMode = MTLStorageModePrivate;
            return [device_ newTextureWithDescriptor:td];
        };
        const MTLTextureUsage rw = MTLTextureUsageRenderTarget | MTLTextureUsageShaderRead;
        vel_ = make(MTLPixelFormatRG16Float, width, height, rw | sc.motionTextureUsage);
        react_ = make(MTLPixelFormatR8Unorm, width, height, rw | sc.reactiveTextureUsage);
        upHdr_ = make(kRenderColorFormat, width, height, MTLTextureUsageShaderRead | sc.outputTextureUsage);
        expo16_ = make(MTLPixelFormatR16Float, 1, 1, rw);
        taaMark_ = make(MTLPixelFormatR16Float, 1, 1, rw);
        fxTemporal_ = sc;
        temporalW_ = width, temporalH_ = height;
        taaReset_ = true;
        taaPrev_.valid = false;
        std::fprintf(stderr, "fuelview: dynamic resolution: MetalFX temporal upscaler (%lux%lu, device scales %.2f..%.2f, render scale >= %.2f)\n",
                     (unsigned long)width, (unsigned long)height, supMin, supMax, taaMinScale_);
        fxTemporalTried_ = true;
        return true;
    }
    return false;
}

// The scene pass's last draw (temporal frames): each sample's luminance into the HDR alpha, read back by the velocity
// pass after the effects (which keep the destination alpha) to make the reactive mask.
void MeshRenderer::encodeTaaLumCapture(id<MTLRenderCommandEncoder> enc) {
    [enc setRenderPipelineState:taaLumPipeline_];
    [enc setDepthStencilState:depthAlways_];
    [enc setCullMode:MTLCullModeNone];
    [enc setTriangleFillMode:MTLTriangleFillModeFill];
    [enc drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:0 vertexCount:3];
}

void MeshRenderer::encodeTaaVelocity(id<MTLCommandBuffer> cb) {
    if (!lastFrame_ || vel_ == nil || !taaCur_.valid) return;
    const TaaCam& c = taaCur_;
    const TaaCam& p = taaPrev_.valid ? taaPrev_ : taaCur_;  // no last frame: no motion (the history restarts anyway)
    if (!taaPrev_.valid) taaReset_ = true;
    TaaUniforms tu = cameraUniforms(taaJitteredVp_, c.vp, p.vp, c.eye, p.eye, c.zn, c.q, float(viewW_), float(viewH_));
    static const float gainEnv = std::getenv("FUEL_TAA_REACTIVE") ? std::stof(std::getenv("FUEL_TAA_REACTIVE")) : -1.0f;  // A / B: the gain
    if (gainEnv >= 0) tu.depth.z = gainEnv;
    tu.prevRight = simd_make_float4(p.right, 0);
    const bool prevClocks = taaPrev_.valid;
    tu.clocks = simd_make_float4(grassClock_, prevClocks ? p.grassT : grassClock_, grassAmp_, prevClocks ? p.grassAmp : grassAmp_);
    tu.water = simd_make_float4(waterClock_, prevClocks ? p.waterT : waterClock_, 0, 0);
    tu.prevR = simd_make_float4(p.R, 0), tu.prevU = simd_make_float4(p.U, 0), tu.prevF = simd_make_float4(p.F, 0);

    MTLRenderPassDescriptor* rp = [MTLRenderPassDescriptor renderPassDescriptor];
    rp.colorAttachments[0].texture = vel_;
    rp.colorAttachments[1].texture = react_;
    for (int k = 0; k < 2; ++k) rp.colorAttachments[k].loadAction = MTLLoadActionDontCare, rp.colorAttachments[k].storeAction = MTLStoreActionStore;
    if (scaledFrame_) rp.renderTargetWidth = viewW_, rp.renderTargetHeight = viewH_;
    gpuprof::tag(rp, "taa.velocity");
    id<MTLRenderCommandEncoder> enc = [cb renderCommandEncoderWithDescriptor:rp];
    applyView(enc);
    // 1. Every pixel: the camera motion of what the resolved depth shows, and the reactive mask.
    [enc setRenderPipelineState:taaCameraPipeline_];
    [enc setFragmentBytes:&tu length:sizeof tu atIndex:0];
    [enc setFragmentTexture:depth_ atIndex:0];
    [enc setFragmentTexture:hdr_ atIndex:1];
    [enc drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:0 vertexCount:3];
    // 2. The moving opaque things over it.
    [enc setCullMode:MTLCullModeNone];
    [enc setVertexBytes:lastFrame_.get() length:kFrameUniformsSize atIndex:1];
    [enc setVertexBytes:&tu length:sizeof tu atIndex:6];
    [enc setFragmentBytes:&tu length:sizeof tu atIndex:6];
    [enc setFragmentTexture:depth_ atIndex:0];
    if (noMorph_ == nil) noMorph_ = [device_ newBufferWithLength:16 options:MTLResourceStorageModeShared];
    // The vertex buffer a skinned mesh drew in the last frame (nil: not skinned, or not animated since).
    auto prevVbOf = [&](const GpuMesh* gm) -> id<MTLBuffer> {
        const auto it = taaPrevBaked_.find(gm);
        if (it == taaPrevBaked_.end() || gm->vbs.empty() || it->second == nil || it->second.length != gm->vbs[0].length) return nil;
        return it->second;
    };
    // The last frame's world of instance i of `list`: the same index in a list of the same length, the same mesh and an
    // origin within 30 m (a list rebuilt with other objects gives no motion rather than a wrong one).
    auto prevWorld = [](const std::vector<Instance>& prev, size_t n, size_t i, const Instance& cur) {
        if (prev.size() == n && i < n && prev[i].mesh == cur.mesh &&
            simd_distance(prev[i].world.columns[3].xyz, cur.world.columns[3].xyz) < 30.0f)
            return prev[i].world;
        return cur.world;
    };
    bool meshBound = false;
    auto drawMesh = [&](const GpuMesh& gm, const simd_float4x4& w, const simd_float4x4& pw, id<MTLBuffer> morph) {
        if (gm.indices == nil) return;
        if (!meshBound) [enc setRenderPipelineState:taaMeshPipeline_], meshBound = true;
        const simd_float4x4 pair[2] = {w, pw};
        [enc setVertexBytes:pair length:sizeof pair atIndex:2];
        id<MTLBuffer> prevVb0 = prevVbOf(&gm);
        for (uint32_t pass = 0; pass < 2; ++pass)  // opaque and cut-out draws (blended ones have no motion of their own)
            for (const Draw& d : gm.draws[pass]) {
                if (d.sprite) continue;  // camera-facing sprites: the camera motion of their pixels
                const uint32_t dr[4] = {gm.vbStrides[d.vb], d.morph && morph != nil ? 1u : 0u, 0, 0};
                [enc setVertexBuffer:gm.vbs[d.vb] offset:0 atIndex:0];
                [enc setVertexBuffer:d.vb == 0 && prevVb0 != nil ? prevVb0 : gm.vbs[d.vb] offset:0 atIndex:5];
                [enc setVertexBuffer:d.morph && morph != nil ? morph : noMorph_ offset:0 atIndex:4];
                [enc setVertexBytes:dr length:sizeof dr atIndex:3];
                [enc drawIndexedPrimitives:MTLPrimitiveTypeTriangle
                                indexCount:d.indexCount
                                 indexType:MTLIndexTypeUInt32
                               indexBuffer:gm.indices
                         indexBufferOffset:size_t(d.firstIndex) * 4];
            }
    };
    for (size_t i = 0; i < instances_.size(); ++i)  // the player's vehicle and rider
        drawMesh(*instances_[i].mesh, instances_[i].world, prevWorld(taaPrevPlayer_, instances_.size(), i, instances_[i]), instances_[i].morph);
    for (size_t v = 0; v < vehicles_.size(); ++v) {  // the opponents the scene pass drew
        if (v < taaVehicleOff_.size() && taaVehicleOff_[v]) continue;
        const OtherVehicle& ov = vehicles_[v];
        static const std::vector<Instance> none;
        const std::vector<Instance>& prev = v < taaPrevVehicles_.size() ? taaPrevVehicles_[v] : none;
        for (size_t i = 0; i < ov.instances.size(); ++i)
            if (ov.drawn(i)) drawMesh(*ov.instances[i].mesh, ov.instances[i].world, prevWorld(prev, ov.instances.size(), i, ov.instances[i]), ov.instances[i].morph);
    }
    for (size_t i = 0; i < propInstances_.size(); ++i)  // props, cut-scene objects, the helicopter, the F14
        drawMesh(*propInstances_[i].mesh, propInstances_[i].world, prevWorld(taaPrevProps_, propInstances_.size(), i, propInstances_[i]), nil);
    {  // LOD set 3: the props drawn with their Lod_Z (moved by setLodObjects every frame)
        const auto& objs = lodObjects_[3];
        for (size_t i = 0; i < objs.size(); ++i) {
            const GpuMesh* gm = nullptr;
            float a = 0;
            if (!lodDecide(3, i, c.eye, gm, a) || gm == nullptr) continue;
            simd_float4x4 pw = objs[i].world;
            if (taaPrevLod3_.size() == objs.size() && simd_distance(taaPrevLod3_[i].columns[3].xyz, pw.columns[3].xyz) < 30.0f) pw = taaPrevLod3_[i];
            drawMesh(*gm, objs[i].world, pw, nil);
        }
    }
    for (const Instance& inst : lodVisible_)  // placed objects with an animated skin (world animations): their vertices moved
        if (const id<MTLBuffer> pv = prevVbOf(inst.mesh); pv != nil && pv != inst.mesh->vbs[0]) drawMesh(*inst.mesh, inst.world, inst.world, nil);
    if (!taaGrassDraws_.empty() && grassBuffer_ != nil && grassAtlas_ != nil) {  // the grass runs the scene pass drew
        const simd_float4 params = simd_make_float4(grassClock_, grassRadius, grassRadius * 0.75f, grassAmp_);
        [enc setRenderPipelineState:taaGrassPipeline_];
        meshBound = false;
        [enc setVertexBuffer:grassBuffer_ offset:0 atIndex:2];
        [enc setVertexBytes:&params length:sizeof params atIndex:3];
        [enc setFragmentTexture:grassAtlas_ atIndex:1];
        for (const auto& [first, count] : taaGrassDraws_)
            [enc drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:0 vertexCount:9 instanceCount:count baseInstance:first];
    }
    if (taaWaterDrawn_ && waterVertices_ != nil && waterIndexCount_ > 0) {  // the lakes' wave
        [enc setRenderPipelineState:taaWaterPipeline_];
        meshBound = false;
        [enc setVertexBuffer:waterVertices_ offset:0 atIndex:0];
        [enc setVertexBytes:waterConst_.data() length:sizeof(waterConst_) atIndex:2];
        [enc drawIndexedPrimitives:MTLPrimitiveTypeTriangle indexCount:waterIndexCount_ indexType:MTLIndexTypeUInt32 indexBuffer:waterIndices_ indexBufferOffset:0];
    }
    bool rotBound = false;  // the opaque RotShape records (the blended glows are effects: the camera motion under them)
    for (const RotRecord& r : rotRecords_) {
        const GpuMaterial& m = rotMaterial(r.material);
        if (m.alpha == AlphaMode::Blend || r.fading) continue;
        if (!rotBound) [enc setRenderPipelineState:taaRotPipeline_], rotBound = true, meshBound = false;
        const simd_float4 prm[2] = {m.uv0, simd_make_float4(m.uv1.x, m.uv1.y, 0, 0)};
        [enc setVertexBytes:&r length:sizeof(simd_float4) * 4 atIndex:2];
        [enc setVertexBytes:prm length:sizeof prm atIndex:3];
        [enc setFragmentTexture:m.textures[0] atIndex:1];
        [enc setFragmentSamplerState:samplers_[m.samplerIndex] atIndex:0];
        [enc drawPrimitives:MTLPrimitiveTypeTriangleStrip vertexStart:0 vertexCount:4];
    }
    [enc endEncoding];
    static const bool stats = std::getenv("FUEL_TAA_STATS") != nullptr;  // debug: the mask's and the motion's means
    if (stats && frame_ % 30 == 0) {
        const NSUInteger w = viewW_, h = viewH_;
        id<MTLBuffer> rb = [device_ newBufferWithLength:w * h options:MTLResourceStorageModeShared];
        id<MTLBuffer> vb = [device_ newBufferWithLength:w * h * 4 options:MTLResourceStorageModeShared];
        id<MTLBlitCommandEncoder> bl = [cb blitCommandEncoder];
        [bl copyFromTexture:react_ sourceSlice:0 sourceLevel:0 sourceOrigin:MTLOriginMake(0, 0, 0) sourceSize:MTLSizeMake(w, h, 1) toBuffer:rb
                 destinationOffset:0 destinationBytesPerRow:w destinationBytesPerImage:w * h];
        [bl copyFromTexture:vel_ sourceSlice:0 sourceLevel:0 sourceOrigin:MTLOriginMake(0, 0, 0) sourceSize:MTLSizeMake(w, h, 1) toBuffer:vb
                 destinationOffset:0 destinationBytesPerRow:w * 4 destinationBytesPerImage:w * h * 4];
        [bl endEncoding];
        const uint32_t f = frame_;
        [cb addCompletedHandler:^(id<MTLCommandBuffer>) {
            const auto* r = static_cast<const uint8_t*>(rb.contents);
            const auto* v = static_cast<const __fp16*>(vb.contents);
            double rs = 0, ms = 0;
            size_t hot = 0;
            for (size_t i = 0; i < size_t(w) * h; ++i) {
                rs += r[i] / 255.0, hot += r[i] > 128;
                ms += std::hypot(double(v[2 * i]) * w, double(v[2 * i + 1]) * h);
            }
            std::printf("taa stats frame %u: reactive mean %.3f, > 0.5 on %.1f%%, motion mean %.2f px\n", f, rs / (w * h), 100.0 * hot / (w * h), ms / (w * h));
        }];
    }
}

// MetalFX temporal: the render-size HDR scene (hdr_), its depth, motion and reactive mask, and this frame's exposure ->
// upHdr_ at the output size, which the tone map then reads as a native frame's scene.
void MeshRenderer::encodeTemporal(id<MTLCommandBuffer> cb, id<MTLTexture> S) {
    {  // the exposure the tone map uses (S multiplies the scene before its curve): MetalFX wants it as 1 x 1 R16Float
        MTLRenderPassDescriptor* rp = [MTLRenderPassDescriptor renderPassDescriptor];
        rp.colorAttachments[0].texture = expo16_;
        rp.colorAttachments[0].loadAction = MTLLoadActionDontCare;
        rp.colorAttachments[0].storeAction = MTLStoreActionStore;
        gpuprof::tag(rp, "post.lum");
        id<MTLRenderCommandEncoder> enc = [cb renderCommandEncoderWithDescriptor:rp];
        [enc setRenderPipelineState:taaExposurePipeline_];
        [enc setFragmentTexture:S atIndex:0];
        [enc drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:0 vertexCount:3];
        [enc endEncoding];
    }
    if (@available(macOS 14.4, *)) {
        id<MTLFXTemporalScaler> sc = fxTemporal_;
        sc.colorTexture = hdr_;
        sc.depthTexture = depth_;
        sc.motionTexture = vel_;
        sc.reactiveMaskTexture = react_;
        sc.exposureTexture = expo16_;
        sc.outputTexture = upHdr_;
        sc.inputContentWidth = viewW_, sc.inputContentHeight = viewH_;
        // The jitter as MetalFX wants it: the sample's offset from the pixel centre in render pixels, x right / y down (the
        // same sign as taaJitterPx_; measured on a static capture spot at 0.5: +1 converges onto the native picture, 24.9 dB,
        // -1 lands half a pixel off, 21.1 dB and visibly shifted; taa.md). FUEL_TAA_JITTER_SIGN=-1 flips it to check.
        static const float sign = std::getenv("FUEL_TAA_JITTER_SIGN") ? std::stof(std::getenv("FUEL_TAA_JITTER_SIGN")) : 1.0f;
        sc.jitterOffsetX = sign * taaJitterPx_.x;
        sc.jitterOffsetY = sign * taaJitterPx_.y;
        static const float mvSign = std::getenv("FUEL_TAA_MV_SIGN") ? std::stof(std::getenv("FUEL_TAA_MV_SIGN")) : 1.0f;  // A / B only
        sc.motionVectorScaleX = mvSign * float(viewW_), sc.motionVectorScaleY = mvSign * float(viewH_);  // uv -> render pixels
        sc.depthReversed = NO;  // clear 1, LessEqual
        sc.reset = taaReset_;
        taaReset_ = false;
        [sc encodeToCommandBuffer:cb];
    }
    if (gpuprof::enabled()) {  // profiling: a 1 x 1 pass that reads MetalFX's output (so it waits for it) charges MetalFX's
        // untagged GPU time to "post.temporal" (else the tone map's span would hold it)
        MTLRenderPassDescriptor* rp = [MTLRenderPassDescriptor renderPassDescriptor];
        rp.colorAttachments[0].texture = taaMark_;
        rp.colorAttachments[0].loadAction = MTLLoadActionDontCare;
        rp.colorAttachments[0].storeAction = MTLStoreActionStore;
        gpuprof::tag(rp, "post.temporal", true);
        id<MTLRenderCommandEncoder> enc = [cb renderCommandEncoderWithDescriptor:rp];
        [enc setRenderPipelineState:taaExposurePipeline_];
        [enc setFragmentTexture:upHdr_ atIndex:0];
        [enc drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:0 vertexCount:3];
        [enc endEncoding];
    }
}

std::vector<simd_float4> MeshRenderer::taaCheck(const TaaCheckCamera& cur, const TaaCheckCamera& prev, float W, float H,
                                                const std::vector<simd_float4>& in) {
    std::vector<simd_float4> out;
    const size_t n = in.size() / 3;
    if (n == 0) return out;
    NSError* e = nil;
    id<MTLComputePipelineState> ps = [device_ newComputePipelineStateWithFunction:[shaders_ newFunctionWithName:@"kTaaCheck"] error:&e];
    if (ps == nil) return out;
    const TaaUniforms tu = cameraUniforms(cur.jitteredVp, cur.vp, prev.vp, cur.eye, prev.eye, cur.zn, cur.q, W, H);
    id<MTLBuffer> bin = [device_ newBufferWithBytes:in.data() length:in.size() * sizeof(simd_float4) options:MTLResourceStorageModeShared];
    id<MTLBuffer> bout = [device_ newBufferWithLength:2 * n * sizeof(simd_float4) options:MTLResourceStorageModeShared];
    id<MTLCommandQueue> q = [device_ newCommandQueue];
    id<MTLCommandBuffer> cb = [q commandBuffer];
    id<MTLComputeCommandEncoder> ce = [cb computeCommandEncoder];
    [ce setComputePipelineState:ps];
    [ce setBytes:&tu length:sizeof tu atIndex:0];
    [ce setBuffer:bin offset:0 atIndex:1];
    [ce setBuffer:bout offset:0 atIndex:2];
    [ce dispatchThreads:MTLSizeMake(n, 1, 1) threadsPerThreadgroup:MTLSizeMake(std::min<NSUInteger>(n, 64), 1, 1)];
    [ce endEncoding];
    [cb commit];
    [cb waitUntilCompleted];
    const auto* p = static_cast<const simd_float4*>(bout.contents);
    out.assign(p, p + 2 * n);
    return out;
}

// This frame's camera, clocks and moving instances become the next frame's "last frame".
void MeshRenderer::taaEndFrame() {
    taaPrev_ = taaCur_;
    taaPrev_.grassT = grassClock_, taaPrev_.grassAmp = grassAmp_, taaPrev_.waterT = waterClock_;
    taaPrevPlayer_ = instances_;
    taaPrevProps_ = propInstances_;
    taaPrevVehicles_.resize(vehicles_.size());
    for (size_t v = 0; v < vehicles_.size(); ++v) taaPrevVehicles_[v] = vehicles_[v].instances;
    taaPrevLod3_.clear();
    for (const LodObject& o : lodObjects_[3]) taaPrevLod3_.push_back(o.world);
    taaPrevBaked_.clear();
    for (const auto& [key, ring] : bakedRing_)  // the skinned meshes updated by updateBakedVertices
        if (const auto it = bakedMeshes_.find(key); it != bakedMeshes_.end() && !it->second->vbs.empty())
            taaPrevBaked_[it->second.get()] = it->second->vbs[0];
}

}  // namespace fuel
