// PK24: FUEL's screen projections on the camera lens (SpecialEffectsManager_G list +0x184: MudProjection 0x5ffe40,
// WaterProjection, DirtScreen; notes ~/re-work/fuel/notes/pk24.md). The logic is core/weather_fx (ScreenFx); this file
// draws it: three SubCodes of sfx.vhl / sfx.phl (0x20 ph_screendirt, 0x21 ph_screenmud, 0x22 ph_screenwater), written
// from their decode (p26-work/post.md P5), no game shader code. The vertex stage places each quad in clip space,
// rotated by the entry's angle, its x extent divided by the viewport aspect.
#import "renderer.h"
#import "gpu_profile.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

namespace fuel {

const char* kLensShaderSource = R"METAL(
struct LensQuad {
    float4 posSize;  // pos xy, half size xy (clip units; x is divided by the aspect)
    float4 rgba;     // current colour (+0x64)
    float4 uvA;      // corners (-,-) and (+,-)
    float4 uvB;      // corners (+,+) and (-,+)
    float4 misc;     // x angle
};

struct LensUniforms {
    float4 view;     // x aspect, yz view size in pixels, w 1 when the normal map is DXT5 (wyz)
    float4 copy;     // xy copy texture size, zw unused
    float4 mode;     // x 0 mud, 1 water, 2 dirt
};

struct LensOut {
    float4 position [[position]];
    float2 uv;
    float4 colour;
};

vertex LensOut vsLens(uint vid [[vertex_id]], uint iid [[instance_id]], const device LensQuad* qs [[buffer(0)]],
                      constant LensUniforms& u [[buffer(1)]]) {
    const LensQuad q = qs[iid];
    const float2 c = float2((vid & 1) ? 1.0 : -1.0, (vid >> 1) ? 1.0 : -1.0);
    float2 uv;
    if (vid == 0) uv = q.uvA.xy;
    else if (vid == 1) uv = q.uvA.zw;
    else if (vid == 2) uv = q.uvB.zw;
    else uv = q.uvB.xy;
    const float2 o = c * q.posSize.zw;
    const float s = sin(q.misc.x), co = cos(q.misc.x);
    const float2 r = float2(o.x * co + o.y * s, o.y * co - o.x * s);
    LensOut out;
    out.position = float4(q.posSize.x + r.x / u.view.x, q.posSize.y + r.y, 0.0, 1.0);
    out.uv = uv;
    out.colour = q.rgba;
    return out;
}

fragment float4 fsLens(LensOut in [[stage_in]], texture2d<float> tex [[texture(0)]], texture2d<float> normalTex [[texture(1)]],
                       texture2d<float> sceneCopy [[texture(2)]], constant LensUniforms& u [[buffer(1)]]) {
    constexpr sampler smp(coord::normalized, filter::linear, mip_filter::linear, address::clamp_to_edge);
    if (u.mode.y > 0.5 && u.mode.y < 1.5) return float4(4.0, 0.0, 4.0, 1.0);  // FUEL_SFX_DEBUG=7: solid magenta
    if (u.mode.x < 0.5) {  // ph_screenmud
        const float4 t = tex.sample(smp, in.uv);
        const float a = in.colour.a * t.a;
        if (a <= 0.0) discard_fragment();
        return float4(2.0 * in.colour.rgb * t.rgb, saturate(a));
    }
    if (u.mode.x > 1.5) {  // ph_screendirt
        const float4 t = tex.sample(smp, in.uv);
        const float a = in.colour.a <= 1.0 ? mix(0.0, t.r, in.colour.a) : mix(t.r, t.g, in.colour.a - 1.0);
        if (a <= 0.0) discard_fragment();
        return float4(t.a * in.colour.rgb, saturate(a));
    }
    // ph_screenwater: the drop's normal map decides where it is (alpha threshold on its blue channel), the scene copy is
    // refracted by the normal.
    const float4 ns = normalTex.sample(smp, in.uv);
    float3 n;
    if (u.view.w > 0.5) {
        n = ns.wyz * 2.0 - 1.0;
        n.z = sqrt(saturate(1.0 - dot(n.xy, n.xy)));
    } else {
        n = ns.xyz * 2.0 - 1.0;
    }
    const float mask = ns.z >= 1.0 - in.colour.a ? 1.0 : 0.0;  // the map's own blue (the drop's height): the drop dissolves from its edge as the alpha falls
    if (mask < 0.5) discard_fragment();
    if (u.mode.y > 2.5) return float4(ns.aaa, 1.0);          // FUEL_SFX_DEBUG=9: the map's alpha
    if (u.mode.y > 1.5) return float4(ns.rgb, 1.0);          // FUEL_SFX_DEBUG=8: the map's rgb
    const float2 screen = in.position.xy / u.view.yz;
    const float2 uv = (screen + n.xy * 0.25 * mask) * (u.view.yz / u.copy.xy);
    float3 rgb = sceneCopy.sample(smp, uv).rgb;
    rgb *= saturate(0.9 + 0.1 * dot(n, float3(0.0, -1.0, 0.0)));
    return float4(rgb, 1.0);
}
)METAL";

void MeshRenderer::createLensPipeline() {
    MTLRenderPipelineDescriptor* d = [MTLRenderPipelineDescriptor new];
    d.vertexFunction = [shaders_ newFunctionWithName:@"vsLens"];
    d.fragmentFunction = [shaders_ newFunctionWithName:@"fsLens"];
    d.colorAttachments[0].pixelFormat = kRenderColorFormat;
    d.colorAttachments[0].blendingEnabled = YES;  // alpha blending, the destination alpha kept
    d.colorAttachments[0].sourceRGBBlendFactor = MTLBlendFactorSourceAlpha;
    d.colorAttachments[0].destinationRGBBlendFactor = MTLBlendFactorOneMinusSourceAlpha;
    d.colorAttachments[0].sourceAlphaBlendFactor = MTLBlendFactorZero;
    d.colorAttachments[0].destinationAlphaBlendFactor = MTLBlendFactorOne;
    NSError* e = nil;
    lensPipeline_ = [device_ newRenderPipelineStateWithDescriptor:d error:&e];
    if (lensPipeline_ == nil) [NSException raise:@"pipeline" format:@"%@", e];
}

// Manager vf17 (0x5f62c0) for the projections: the player's vehicle feed arrives through setLensVehicle (drive loop).
void MeshRenderer::updateLens(float dt) {
    if (!sfxTables_ || sfxTables_->size() < 42) return;
    static const bool off = std::getenv("FUEL_NO_LENS") != nullptr;
    if (off) return;
    if (!lens_) lens_ = std::make_unique<wfx::ScreenFx>(wfx::LensParams::fromPP(weatherFields_ ? weatherFields_->pp() : std::vector<float>()));
    if (!lensWell_) {  // the lens's own WELL512 draws (port: own state, so the other tanks' draws do not depend on it)
        std::array<uint32_t, 16> seed{};
        for (size_t i = 0; i < seed.size(); ++i) seed[i] = 0x7f4a7c15u * uint32_t(i + 3) ^ 0x2545f491u;
        lensWell_ = std::make_unique<sfx::Well512>(seed);
    }
    wfx::LensVehicle veh = lensVeh_;
    if (frame_ > lensVehFrame_ + 2) veh = wfx::LensVehicle{};  // no drive loop feeding this frame (menus, garage)
    static const float forceRain = std::getenv("FUEL_LENS_RAIN") ? float(std::atof(std::getenv("FUEL_LENS_RAIN"))) : -1.0f;  // shots
    const float rain = forceRain >= 0 ? forceRain : (weatherFromWorld ? weatherNow_.rain : 0.0f);  // WeatherManager +0xb8
    const wfx::RndInt well = [this]() { return int32_t(lensWell_->next()); };
    const auto rect = [this](int i) -> std::array<float, 4> {
        const auto& e = (*sfxTables_)[size_t(i)];
        return {e.u0, e.v0, e.du, e.dv};
    };
    static const float warm = std::getenv("FUEL_LENS_WARM") ? float(std::atof(std::getenv("FUEL_LENS_WARM"))) : 0.0f;  // shots only
    if (warm > 0 && !lensWarmed_) {  // headless films render one frame per shot: run the projections' history first
        lensWarmed_ = true;
        for (float t = 0; t < warm; t += 1.0f / 30) lens_->update(1.0f / 30, rain, veh, well, rect);
    }
    static const bool substep = std::getenv("FUEL_LENS_SUBSTEP") != nullptr;  // shots only: a film step of seconds runs as 1/30 s frames
    if (substep && dt > 1.0f / 30) {
        for (float t = 0; t < dt; t += 1.0f / 30) lens_->update(std::min(1.0f / 30, dt - t), rain, veh, well, rect);
    } else {
        lens_->update(dt, rain, veh, well, rect);
    }
    if (std::getenv("FUEL_SFX_DEBUG") && frame_ % 30 == 0) {
        int n[3] = {0, 0, 0};
        for (const auto& e : lens_->entries()) ++n[int(e.kind)];
        std::fprintf(stderr, "pk24 lens: mud %d, water %d, dirt strips %d (rain %.2f, dirt %.2f, present %d, speed %.1f, dust %d)\n", n[0], n[1], n[2], rain,
                     veh.dirt, int(veh.present), std::sqrt(veh.velocity[0] * veh.velocity[0] + veh.velocity[1] * veh.velocity[1] + veh.velocity[2] * veh.velocity[2]),
                     int(veh.dust));
    }
}

void MeshRenderer::encodeLens(id<MTLCommandBuffer> cb) {
    if (!lens_ || lens_->entries().empty() || lensPipeline_ == nil || !lastFrame_ || hdr_ == nil) return;
    struct Quad {
        simd_float4 posSize, rgba, uvA, uvB, misc;
    };
    struct Uniforms {
        simd_float4 view, copy, mode;
    };
    struct Run {
        size_t first, count;
        int atlas, kind;
    };
    std::vector<Quad> quads;
    std::vector<Run> runs;
    quads.reserve(lens_->entries().size());
    for (int kind = 0; kind < 3; ++kind) {  // the projections draw in list order: mud, water, dirt
        for (const wfx::LensEntry& e : lens_->entries()) {
            if (int(e.kind) != kind || e.rgba[3] <= 0) continue;
            const int slot = e.atlas - 30;
            if (slot < 0 || size_t(slot) >= lensTex_.size()) continue;
            if (kind == 1 ? lensTex_[size_t(slot)].normal == nil : lensTex_[size_t(slot)].diffuse == nil) continue;  // splash maps: slot 3 only
            quads.push_back({simd_make_float4(e.pos[0], e.pos[1], e.size[0], e.size[1]), simd_make_float4(e.rgba[0], e.rgba[1], e.rgba[2], e.rgba[3]),
                             simd_make_float4(e.uv[0], e.uv[1], e.uv[2], e.uv[3]), simd_make_float4(e.uv[4], e.uv[5], e.uv[6], e.uv[7]),
                             simd_make_float4(e.angle, 0, 0, 0)});
            if (!runs.empty() && runs.back().atlas == e.atlas && runs.back().kind == kind)
                ++runs.back().count;
            else
                runs.push_back({quads.size() - 1, 1, e.atlas, kind});
        }
    }
    static const int showTex = std::getenv("FUEL_LENS_SHOWTEX") ? std::atoi(std::getenv("FUEL_LENS_SHOWTEX")) : 0;  // shots: one whole map in a corner
    if (showTex >= 30 && showTex < 42) {
        quads.push_back({simd_make_float4(-0.6f, 0.4f, 0.5f * float(viewH_ ? viewH_ : hdr_.height) / float(viewW_ ? viewW_ : hdr_.width) * 1.7778f, 0.5f),
                         simd_make_float4(1, 1, 1, 1), simd_make_float4(0, 1, 1, 1), simd_make_float4(1, 0, 0, 0), simd_make_float4(0)});
        runs.push_back({quads.size() - 1, 1, showTex, 1});
    }
    if (quads.empty()) return;
    const bool water = std::any_of(runs.begin(), runs.end(), [](const Run& r) { return r.kind == 1; });
    if (water) {  // ph_screenwater samples the scene: copy the colour drawn so far
        fxEnd();
        if (lensCopy_ == nil || lensCopy_.width != hdr_.width || lensCopy_.height != hdr_.height) {
            MTLTextureDescriptor* td = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:hdr_.pixelFormat width:hdr_.width height:hdr_.height mipmapped:NO];
            td.usage = MTLTextureUsageShaderRead;
            td.storageMode = MTLStorageModePrivate;
            lensCopy_ = [device_ newTextureWithDescriptor:td];
        }
        MTLBlitPassDescriptor* bp = [MTLBlitPassDescriptor blitPassDescriptor];
        gpuprof::tag(bp, "lens.copy");
        id<MTLBlitCommandEncoder> blit = [cb blitCommandEncoderWithDescriptor:bp];
        [blit copyFromTexture:hdr_ sourceSlice:0 sourceLevel:0 sourceOrigin:MTLOriginMake(0, 0, 0)
                   sourceSize:MTLSizeMake(std::min(viewW_ ? viewW_ : hdr_.width, hdr_.width), std::min(viewH_ ? viewH_ : hdr_.height, hdr_.height), 1)
                    toTexture:lensCopy_ destinationSlice:0 destinationLevel:0 destinationOrigin:MTLOriginMake(0, 0, 0)];
        [blit endEncoding];
    }
    id<MTLRenderCommandEncoder> enc = fxEncoder(cb, "fx.lens");
    [enc setRenderPipelineState:lensPipeline_];
    id<MTLBuffer> buf = [device_ newBufferWithBytes:quads.data() length:quads.size() * sizeof(Quad) options:MTLResourceStorageModeShared];
    [enc setVertexBuffer:buf offset:0 atIndex:0];
    const float w = float(viewW_ ? viewW_ : hdr_.width), h = float(viewH_ ? viewH_ : hdr_.height);
    for (const Run& r : runs) {
        const LensTex& t = lensTex_[size_t(r.atlas - 30)];
        Uniforms u;
        u.view = simd_make_float4(w / h, w, h, t.compressed ? 1.0f : 0.0f);
        u.copy = simd_make_float4(float(hdr_.width), float(hdr_.height), 0, 0);
        static const int dbg = std::getenv("FUEL_SFX_DEBUG") ? std::atoi(std::getenv("FUEL_SFX_DEBUG")) : 0;
        u.mode = simd_make_float4(float(r.kind), dbg == 7 ? 1.0f : dbg == 8 ? 2.0f : dbg == 9 ? 3.0f : 0.0f, 0, 0);
        [enc setVertexBytes:&u length:sizeof u atIndex:1];
        [enc setFragmentBytes:&u length:sizeof u atIndex:1];
        id<MTLTexture> any = t.diffuse != nil ? t.diffuse : t.normal;
        [enc setFragmentTexture:any atIndex:0];
        [enc setFragmentTexture:t.normal != nil ? t.normal : any atIndex:1];
        [enc setFragmentTexture:lensCopy_ != nil ? lensCopy_ : any atIndex:2];
        [enc drawPrimitives:MTLPrimitiveTypeTriangleStrip vertexStart:0 vertexCount:4 instanceCount:r.count baseInstance:r.first];
    }
}

}  // namespace fuel
