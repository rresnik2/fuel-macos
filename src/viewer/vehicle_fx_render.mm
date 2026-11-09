// PK23: the vehicle effects of core/vehicle_fx (notes ~/re-work/fuel/notes/pk23.md): exhaust smoke (CPU particles,
// atlas 6), exhaust flames (three prebuilt quads, atlas 0x30), scrape spark streaks and exhaust embers (ribbon ropes,
// atlas 0 FX_SPARKLE). All are SubCodes 0x15 / 0x16 of the SFX manager, i.e. smoke.vhl vh_smoke + smoke.phl ph_smoke:
// the vertex is moved by uvscale.zw rotated by positionRotation.w along CAM_WSIDE / CAM_WUP (0 for the prebuilt quads and
// strips), its alpha fades within 2 m of the eye (d^2 / 4), the colour is lit like the other SFX (sky + sun saturate(L.y)
// times the vertex colour x 2 x extinction, inscatter added) and lerped by the shadow; the pixel clips at alpha 0.025.
// Written from the shader text's maths; no game shader code. The ribbon strips are built here (0x5fba20 / 0x5fae60 with
// draw flags 0: side = view direction x segment, u = 0.3 k / (n - 1), v 0 / 1, half width = size).
#import "renderer.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace fuel {

void frameCameraOf(const FrameUniforms& f, simd_float4x4& viewProj, simd_float3& eye, simd_float3& right, simd_float3& up);  // renderer.mm
extern const size_t kFrameUniformsSize;  // renderer.mm

const char* kVehicleFxShaderSource = R"METAL(
struct VfxVertex {
    float4 pos;  // xyz world, w angle (positionRotation.w)
    float4 uv;   // xy texture uv, zw uvscale.zw (the camera-plane offset in metres)
    float4 rgba;
};

struct VfxOut {
    float4 position [[position]];
    float2 uv;
    float4 sun;    // GetSunLight: (sky + sun saturate(L.y)) x vcolor, a = alpha
    float4 sky;    // GetShadowLight: sky x vcolor
    float3 insc;
    float viewZ;
    float shadow;  // SFX_VERTEX_SHADOWS: the shadow term at the vertex
};

vertex VfxOut vsVehicleFx(uint vid [[vertex_id]], const device VfxVertex* vs [[buffer(0)]], constant FrameUniforms& f [[buffer(1)]],
                          depth2d_array<float> shadowMap [[texture(0)]]) {
    const VfxVertex v = vs[vid];
    float3 pos = v.pos.xyz;
    const float s = sin(v.pos.w), c = cos(v.pos.w);
    const float2 scale = float2(v.uv.z * c - v.uv.w * s, v.uv.z * s + v.uv.w * c);
    pos += scale.x * f.camRight.xyz + scale.y * f.camUp.xyz;
    VfxOut o;
    o.position = fuelClip(f, pos);
    o.uv = v.uv.xy;
    float4 diffuse = v.rgba;
    const float3 toEye = f.eye.xyz - v.pos.xyz;
    const float d2 = dot(toEye, toEye);
    if (d2 < 4.0) diffuse.a *= d2 / 4.0;  // fadeDistance C_TWO
    const Scatter sc = fuelScattering(f, pos);
    const float3 vcol = diffuse.rgb * 2.0 * sc.ext;
    o.sun = float4((f.skyColour.rgb + f.sunColour.rgb * saturate(f.lightDir.y)) * vcol, diffuse.a);
    o.sky = float4(f.skyColour.rgb * vcol, diffuse.a);
    o.insc = sc.insc;
    o.viewZ = tornadoViewZ(f, pos);
    o.shadow = fuelShadow(shadowMap, pos, f);
    return o;
}

fragment float4 fsVehicleFx(VfxOut in [[stage_in]], depth2d<float> depth [[texture(0)]], texture2d<float> tex [[texture(1)]],
                            constant FrameUniforms& f [[buffer(1)]], constant float4& debug [[buffer(2)]]) {
    if (in.viewZ > tornadoSceneZ(f, depth, in.position)) discard_fragment();  // the scene's depth (no depth attachment here)
    if (debug.x > 0.5) return float4(0.0, 4.0, 0.0, 1.0);  // FUEL_SFX_DEBUG=2: solid green
    constexpr sampler smp(coord::normalized, filter::linear, mip_filter::linear, address::repeat);
    float4 c = tex.sample(smp, in.uv);
    c.a *= in.sky.a;
    if (c.a < 0.025) discard_fragment();  // clip(color.w - 0.025)
    c.rgb *= mix(in.sky.rgb, in.sun.rgb, in.shadow);
    c.rgb += in.insc;
    return float4(max(c.rgb, 0.0), saturate(c.a));
}
)METAL";

void MeshRenderer::createVehicleFxPipeline() {
    MTLRenderPipelineDescriptor* d = [MTLRenderPipelineDescriptor new];
    d.vertexFunction = [shaders_ newFunctionWithName:@"vsVehicleFx"];
    d.fragmentFunction = [shaders_ newFunctionWithName:@"fsVehicleFx"];
    d.colorAttachments[0].pixelFormat = kRenderColorFormat;
    d.colorAttachments[0].blendingEnabled = YES;  // alpha blending, the destination alpha kept (as the other SFX)
    d.colorAttachments[0].sourceRGBBlendFactor = MTLBlendFactorSourceAlpha;
    d.colorAttachments[0].destinationRGBBlendFactor = MTLBlendFactorOneMinusSourceAlpha;
    d.colorAttachments[0].sourceAlphaBlendFactor = MTLBlendFactorZero;
    d.colorAttachments[0].destinationAlphaBlendFactor = MTLBlendFactorOne;
    NSError* e = nil;
    vfxPipeline_ = [device_ newRenderPipelineStateWithDescriptor:d error:&e];
    if (vfxPipeline_ == nil) [NSException raise:@"pipeline" format:@"%@", e];
}

void MeshRenderer::encodeVehicleFx(id<MTLCommandBuffer> cb) {
    static const bool off = std::getenv("FUEL_NO_VEHICLE_FX") != nullptr;  // A/B
    if (off || !lastFrame_ || vfxPipeline_ == nil) return;
    if (dust_.exhaustSmoke.empty() && dust_.exhaustFlames.empty() && dust_.streaks.empty()) return;
    struct V {
        simd_float4 pos, uv, rgba;
    };
    simd_float4x4 vp;
    simd_float3 eye, right, up;
    frameCameraOf(*lastFrame_, vp, eye, right, up);
    const simd_float3 fwd = simd_cross(up, right);
    std::vector<V> verts;
    verts.reserve(dust_.exhaustSmoke.size() * 6 + dust_.exhaustFlames.size() * 6 + dust_.streaks.size() * 18);
    auto rgba = [](const vfx::C4& c) { return simd_make_float4(c[0], c[1], c[2], c[3]); };
    // 1. smoke (atlas 6), far to near: the CPU particle's four corners (+-size, uv per corner as vsSfx).
    const size_t smokeFirst = verts.size();
    for (const vfx::SmokeSprite& s : dust_.exhaustSmoke) {
        V q[4];
        for (int k = 0; k < 4; ++k) {
            const float cx = (k & 1) ? 1.0f : -1.0f, cy = (k >> 1) ? 1.0f : -1.0f;
            q[k].pos = simd_make_float4(s.pos[0], s.pos[1], s.pos[2], s.angle);
            q[k].uv = simd_make_float4(cx < 0 ? s.uv[0] : s.uv[2], cy > 0 ? s.uv[1] : s.uv[3], cx * s.size, cy * s.size);
            q[k].rgba = rgba(s.rgba);
        }
        for (int k : {0, 1, 2, 1, 3, 2}) verts.push_back(q[k]);
    }
    const size_t smokeCount = verts.size() - smokeFirst;
    // 2. flames (atlas 0x30 rect of FX_ATLAS_0): indices 0, 1, 2, 1, 2, 3 per quad (0x5f6150), vertex 0's colour.
    const size_t flameFirst = verts.size();
    for (const vfx::FlameQuad& fq : dust_.exhaustFlames)
        for (int k : {0, 1, 2, 1, 2, 3})
            verts.push_back({simd_make_float4(fq.v[size_t(k)][0], fq.v[size_t(k)][1], fq.v[size_t(k)][2], 0),
                             simd_make_float4(fq.uv[size_t(k)][0], fq.uv[size_t(k)][1], 0, 0), rgba(fq.rgba)});
    const size_t flameCount = verts.size() - flameFirst;
    // 3. streaks (FX_SPARKLE): 0x5fba20 with draw flags 0 (no taper, u from +0x5c = 0.3 and +0x60 = 0).
    const size_t streakFirst = verts.size();
    for (const vfx::Streak& st : dust_.streaks) {
        const size_t n = st.rope.size();
        simd_float3 side = simd_make_float3(0, 0, 0);
        V prevA{}, prevB{};
        for (size_t k = 0; k < n; ++k) {
            const simd_float3 p = simd_make_float3(st.rope[k][0], st.rope[k][1], st.rope[k][2]);
            if (k + 1 < n) {
                const simd_float3 d = simd_make_float3(st.rope[k + 1][0], st.rope[k + 1][1], st.rope[k + 1][2]) - p;
                const float l = simd_length(d);
                if (l > 1e-4f) {
                    const simd_float3 c = simd_cross(fwd, d / l);
                    const float cl = simd_length(c);
                    side = cl > 1e-6f ? c / cl : c;
                } else if (k < 2) {
                    side = simd_make_float3(0, 0, 0);
                }
            }
            const simd_float3 w = side * st.halfWidth;
            const float u = 0.3f * float(k) / float(n - 1);
            const V a{simd_make_float4(p + w, 0), simd_make_float4(u, 0, 0, 0), rgba(st.rgba)};
            const V b{simd_make_float4(p - w, 0), simd_make_float4(u, 1, 0, 0), rgba(st.rgba)};
            if (k > 0) {
                verts.push_back(prevA), verts.push_back(prevB), verts.push_back(a);
                verts.push_back(prevB), verts.push_back(b), verts.push_back(a);
            }
            prevA = a, prevB = b;
        }
    }
    const size_t streakCount = verts.size() - streakFirst;
    if (verts.empty()) return;
    id<MTLBuffer> buf = [device_ newBufferWithBytes:verts.data() length:verts.size() * sizeof(V) options:MTLResourceStorageModeShared];
    static const simd_float4 debug = simd_make_float4(std::getenv("FUEL_SFX_DEBUG") && std::atoi(std::getenv("FUEL_SFX_DEBUG")) == 2 ? 1.0f : 0.0f, 0, 0, 0);
    id<MTLRenderCommandEncoder> enc = fxEncoder(cb, "fx.vehicle");  // the shared effects pass
    [enc setRenderPipelineState:vfxPipeline_];
    [enc setCullMode:MTLCullModeNone];
    [enc setVertexBuffer:buf offset:0 atIndex:0];
    [enc setVertexBytes:lastFrame_.get() length:kFrameUniformsSize atIndex:1];
    [enc setVertexTexture:shadowMap_ atIndex:0];
    [enc setFragmentBytes:lastFrame_.get() length:kFrameUniformsSize atIndex:1];
    [enc setFragmentBytes:&debug length:sizeof debug atIndex:2];
    [enc setFragmentTexture:depth_ atIndex:0];
    auto draw = [&](id<MTLTexture> t, size_t first, size_t count) {
        if (t == nil || count == 0) return;
        [enc setFragmentTexture:t atIndex:1];
        [enc drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:first vertexCount:count];
    };
    draw(tornadoTex_[4], smokeFirst, smokeCount);  // atlas 6 (P65 loads the same material for the tornado base smoke)
    draw(sfxAtlas_, flameFirst, flameCount);
    draw(sparkleTex_, streakFirst, streakCount);
    if (const char* tr = std::getenv("FUEL_PK23_TRACE"); tr && (frame_ % 30 == 0 || std::atoi(tr) == 2))
        std::fprintf(stderr, "pk23 draw: smoke %zu, flame quads %zu, streaks %zu (textures smoke %d atlas %d sparkle %d)\n", dust_.exhaustSmoke.size(),
                     dust_.exhaustFlames.size(), dust_.streaks.size(), tornadoTex_[4] != nil, sfxAtlas_ != nil, sparkleTex_ != nil);
}

}  // namespace fuel
