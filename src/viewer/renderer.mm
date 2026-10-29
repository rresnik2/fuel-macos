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

const char* kShaderSource = R"METAL(
#include <metal_stdlib>
using namespace metal;

struct VertexIn {
    float3 position [[attribute(0)]];
    float4 tangent  [[attribute(1)]];  // D3DCOLOR read as BGRA: xyz direction, w handedness
    float4 normal   [[attribute(2)]];
    float2 uv       [[attribute(3)]];
};

struct FrameUniforms {
    float4x4 viewProj;
    float4x4 sunViewProj;
    float4 sunDir;        // xyz towards the sun, w intensity
    float4 paint;         // linear rgb
    float4 eye;           // camera position
    float4 params;        // x dirt, y exposure, z ground height, w ground radius
    float4 footprint;     // xy scene centre (x, z), zw half extents
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
              Reflective = 64;

struct VertexOut {
    float4 position [[position]];
    float3 world;
    float3 normal;
    float4 tangent;
    float2 uv;
    float4 sunPos;
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

float3 skyColor(float3 d) {
    float3 zenith = float3(0.18, 0.34, 0.68), horizon = float3(0.68, 0.74, 0.80), ground = float3(0.20, 0.18, 0.15);
    if (d.y >= 0.0) return mix(horizon, zenith, sqrt(d.y));
    return mix(horizon * 0.6, ground, saturate(-d.y * 4.0));
}

float3 tonemap(float3 x) {  // ACES fit (Narkowicz)
    return saturate((x * (2.51 * x + 0.03)) / (x * (2.43 * x + 0.59) + 0.14));
}

// Sun (GGX highlight, shadowed), hemispheric sky light and sky reflections.
float3 lighting(float3 albedo, float3 N, float3 V, float ao, float shadow, float specStrength, float rough,
                float2 fresnel, bool reflective, constant FrameUniforms& f) {
    float3 L = normalize(f.sunDir.xyz);
    float3 sun = float3(1.0, 0.95, 0.86) * f.sunDir.w;
    float NdotL = saturate(dot(N, L));
    float NdotV = saturate(dot(N, V)) + 1e-4;
    float3 H = normalize(L + V);
    float NdotH = saturate(dot(N, H));
    float VdotH = saturate(dot(V, H));

    float3 ambient = mix(float3(0.14, 0.12, 0.10), float3(0.40, 0.47, 0.58), N.y * 0.5 + 0.5) * ao;
    float3 color = albedo * (sun * NdotL * shadow + ambient);

    float a2 = pow(rough, 4.0);
    float dn = NdotH * NdotH * (a2 - 1.0) + 1.0;
    float D = a2 / (M_PI_F * dn * dn);
    float k = (rough + 1.0) * (rough + 1.0) / 8.0;
    float G = NdotL / (NdotL * (1.0 - k) + k) * NdotV / (NdotV * (1.0 - k) + k);
    float f0 = mix(0.02, 0.08, specStrength);
    float F = f0 + (1.0 - f0) * pow(1.0 - VdotH, 5.0);
    color += sun * (D * G * F / max(4.0 * NdotV, 1e-3)) * shadow * specStrength;

    float3 env = mix(skyColor(reflect(-V, N)), float3(0.42, 0.47, 0.53), rough);
    float fr = reflective ? mix(fresnel.x, fresnel.y, pow(1.0 - NdotV, 5.0)) * 2.0
                          : (f0 + (1.0 - f0) * pow(1.0 - NdotV, 5.0)) * (1.0 - rough) * specStrength;
    return color + env * saturate(fr) * ao * mix(0.5, 1.0, shadow);
}

vertex VertexOut vsMain(VertexIn in [[stage_in]], constant FrameUniforms& f [[buffer(1)]],
                        constant float4x4& model [[buffer(2)]]) {
    VertexOut o;
    float4 w = model * float4(in.position, 1.0);
    o.position = f.viewProj * w;
    o.world = w.xyz;
    o.normal = (model * float4(in.normal.xyz * 2.0 - 1.0, 0.0)).xyz;
    o.tangent = float4((model * float4(in.tangent.xyz * 2.0 - 1.0, 0.0)).xyz, in.tangent.w > 0.5 ? 1.0 : -1.0);
    o.uv = in.uv;
    o.sunPos = f.sunViewProj * w;
    return o;
}

vertex float4 vsShadow(VertexIn in [[stage_in]], constant FrameUniforms& f [[buffer(1)]],
                       constant float4x4& model [[buffer(2)]]) {
    return f.sunViewProj * (model * float4(in.position, 1.0));
}

fragment float4 fsMain(VertexOut in [[stage_in]], bool front [[front_facing]],
                       constant FrameUniforms& f [[buffer(1)]],
                       constant MaterialUniforms& m [[buffer(2)]],
                       constant float4x4& model [[buffer(3)]],
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
    if (m.lit == 0) return float4(tonemap((c.rgb + m.emissive.rgb) * f.params.y), alpha);

    float3 N = normalize(in.normal) * (front ? 1.0 : -1.0);
    if (m.features & HasNormalLocal) {
        // Object-space normal map, two channels plus alpha.
        N = normalize((model * float4(tNormalLocal.sample(smp, uvHalf).xyw * 2.0 - 1.0, 0.0)).xyz);
    } else if (m.features & HasNormal) {
        // Tangent-space normal map with x in alpha and y in green.
        float2 xy = tNormal.sample(smp, uvHalf).wy * 2.0 - 1.0;
        float3 T = normalize(in.tangent.xyz - N * dot(N, in.tangent.xyz));
        float3 B = cross(N, T) * in.tangent.w;
        N = normalize(T * xy.x + B * xy.y + N * sqrt(saturate(1.0 - dot(xy, xy))));
    }
    float ao = (m.features & HasOcclusion) ? tOcclusion.sample(smp, in.uv).r : 1.0;
    float3 spec = m.specular.rgb * ((m.features & HasSpecular) ? tSpecular.sample(smp, in.uv).rgb : float3(1.0));
    float specStrength = saturate(dot(spec, float3(1.0 / 6.0)));
    float rough = clamp(sqrt(2.0 / (max(m.specular.w, 0.0) + 2.0)), 0.08, 1.0);
    float shadow = shadowFactor(shadowMap, in.sunPos);
    float3 V = normalize(f.eye.xyz - in.world);
    float3 rgb = lighting(c.rgb, N, V, ao, shadow, specStrength, rough, m.fresnel, (m.features & Reflective) != 0, f);
    return float4(tonemap((rgb + m.emissive.rgb) * f.params.y), alpha);
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
    return float4(tonemap(rgb * f.params.y), fade);
}
)METAL";

struct FrameUniforms {
    simd_float4x4 viewProj;
    simd_float4x4 sunViewProj;
    simd_float4 sunDir;
    simd_float4 paint;
    simd_float4 eye;
    simd_float4 params;
    simd_float4 footprint;
};

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
    for (int i = 0; i < 4; ++i) vd.attributes[i].bufferIndex = 0;
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

    sunDir_ = simd_normalize(simd_make_float3(0.55f, 0.80f, 0.45f));
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

MeshRenderer::GpuMaterial MeshRenderer::makeMaterial(uint32_t materialId) {
    GpuMaterial gm;
    gm.textures.fill(white_);
    auto hit = library_.find(materialId, cls::Material);
    if (!hit) return gm;
    try {
        const auto body = hit.file->body(*hit.entry);
        if (!body) return gm;
        const Material m = parseMaterialBody(*body);
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
    if (auto it = meshes_.find(meshId); it != meshes_.end()) return it->second.get();
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
        Draw d{b.vb, uint32_t(indices.size()), uint32_t(tris.size()), mat};
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
    return (meshes_[meshId] = std::move(gm)).get();
}

bool MeshRenderer::setMesh(uint32_t meshId, std::string* error) {
    return setScene({MeshInstance{meshId, matrix_identity_float4x4}}, error);
}

bool MeshRenderer::setScene(const std::vector<MeshInstance>& instances, std::string* error) {
    instances_.clear();
    triangles_ = 0;
    error->clear();
    simd_float3 lo = simd_make_float3(1e30f, 1e30f, 1e30f), hi = -lo;
    for (const MeshInstance& in : instances) {
        std::string err;
        const GpuMesh* gm = loadMesh(in.meshId, &err);
        if (gm == nullptr) {
            char buf[32];
            std::snprintf(buf, sizeof buf, "%08x", in.meshId);
            *error += (error->empty() ? "" : "; ") + std::string(buf) + ": " + err;
            continue;
        }
        if (gm->indices == nil) continue;
        instances_.push_back({gm, in.world});
        triangles_ += gm->triangles;
        for (int c = 0; c < 8; ++c) {
            const simd_float3 corner = simd_make_float3((c & 1) ? gm->boundsMax.x : gm->boundsMin.x,
                                                        (c & 2) ? gm->boundsMax.y : gm->boundsMin.y,
                                                        (c & 4) ? gm->boundsMax.z : gm->boundsMin.z);
            const simd_float4 w = simd_mul(in.world, simd_make_float4(corner, 1.0f));
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

simd_float4x4 MeshRenderer::sunViewProj() const {
    const simd_float3 up = std::fabs(sunDir_.y) > 0.99f ? simd_make_float3(0, 0, 1) : simd_make_float3(0, 1, 0);
    const simd_float3 eye = center_ + sunDir_ * (radius_ * 3.0f);
    return simd_mul(orthoRH(radius_ * 1.05f, radius_ * 0.5f, radius_ * 6.0f), lookAtRH(eye, center_, up));
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
    for (const Instance& inst : instances_) {
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
    const simd_float3 eye = center_ + dist * simd_make_float3(std::cos(cam.pitch) * std::sin(cam.yaw), std::sin(cam.pitch),
                                                              std::cos(cam.pitch) * std::cos(cam.yaw));
    FrameUniforms fu;
    fu.viewProj = simd_mul(perspectiveRH(0.70f, aspect, radius_ * 0.02f, radius_ * 40.0f),
                           lookAtRH(eye, center_, simd_make_float3(0, 1, 0)));
    fu.sunViewProj = sunViewProj();
    fu.sunDir = simd_make_float4(sunDir_, 3.0f);
    fu.paint = simd_make_float4(srgbToLinear(paint_.color.x), srgbToLinear(paint_.color.y), srgbToLinear(paint_.color.z), 1);
    fu.eye = simd_make_float4(eye, 1);
    fu.params = simd_make_float4(dirt, 1.0f, sceneMin_.y, radius_ * 4.0f);
    fu.footprint = simd_make_float4(center_.x, center_.z, (sceneMax_.x - sceneMin_.x) * 0.5f, (sceneMax_.z - sceneMin_.z) * 0.5f);

    [enc setVertexBytes:&fu length:sizeof fu atIndex:1];
    [enc setFragmentBytes:&fu length:sizeof fu atIndex:1];
    [enc setFragmentTexture:shadowMap_ atIndex:7];
    [enc setFrontFacingWinding:MTLWindingClockwise];

    auto drawPass = [&](uint32_t pass) {
        const AlphaMode mode = AlphaMode(pass);
        [enc setDepthStencilState:mode == AlphaMode::Blend ? depthNoWrite_ : depthWrite_];
        [enc setCullMode:cullBackFaces ? MTLCullModeBack : MTLCullModeNone];
        [enc setTriangleFillMode:wireframe ? MTLTriangleFillModeLines : MTLTriangleFillModeFill];
        for (const Instance& inst : instances_) {
            const GpuMesh& gm = *inst.mesh;
            if (gm.draws[pass].empty()) continue;
            [enc setVertexBytes:&inst.world length:sizeof inst.world atIndex:2];
            [enc setFragmentBytes:&inst.world length:sizeof inst.world atIndex:3];
            for (const Draw& d : gm.draws[pass]) {
                const GpuMaterial& m = gm.materials[d.material];
                [enc setRenderPipelineState:pipeline(gm.vbStrides[d.vb], mode)];
                [enc setVertexBuffer:gm.vbs[d.vb] offset:0 atIndex:0];
                MaterialUniforms mu{m.diffuse, m.specular, m.emissive, m.fresnel, uint32_t(m.alpha), m.lit ? 1u : 0u, m.code,
                                    m.features};
                [enc setFragmentBytes:&mu length:sizeof mu atIndex:2];
                for (size_t slot = 0; slot < m.textures.size(); ++slot) [enc setFragmentTexture:m.textures[slot] atIndex:slot];
                [enc setFragmentSamplerState:samplers_[m.samplerIndex] atIndex:0];
                [enc drawIndexedPrimitives:MTLPrimitiveTypeTriangle
                                indexCount:d.indexCount
                                 indexType:MTLIndexTypeUInt32
                               indexBuffer:gm.indices
                         indexBufferOffset:size_t(d.firstIndex) * 4];
            }
        }
    };
    drawPass(0);
    drawPass(1);
    if (showGround) {
        [enc setRenderPipelineState:groundPipeline_];
        [enc setDepthStencilState:depthWrite_];
        [enc setCullMode:MTLCullModeNone];
        [enc setTriangleFillMode:MTLTriangleFillModeFill];
        [enc drawPrimitives:MTLPrimitiveTypeTriangleStrip vertexStart:0 vertexCount:4];
    }
    drawPass(2);
}

}  // namespace fuel
