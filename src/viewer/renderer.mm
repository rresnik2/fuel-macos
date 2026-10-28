#import "renderer.h"

#include <algorithm>
#include <cmath>
#include <cstring>

namespace fuel {

namespace {

// Vertex layouts used by meshes (VertexND_Z, VertexGouraud_Z and the skinned
// Vertex1Blend_Z/Vertex4Blend_Z) all start the same way: float3 position at 0,
// a D3DCOLOR normal at 16 and float2 uv0 at 20. Only the stride differs.
constexpr uint32_t kMinDrawableStride = 28;

const char* kShaderSource = R"METAL(
#include <metal_stdlib>
using namespace metal;

struct VertexIn {
    float3 position [[attribute(0)]];
    float4 normal   [[attribute(1)]];  // D3DCOLOR read as BGRA, so .xyz = bytes 2,1,0
    float2 uv       [[attribute(2)]];
};

struct FrameUniforms {
    float4x4 viewProj;
    float4 lightDir;
};

struct MaterialUniforms {
    float4 diffuse;
    uint alphaMode;  // 0 opaque, 1 alpha test, 2 blend
    uint lit;
    uint pad0, pad1;
};

struct VertexOut {
    float4 position [[position]];
    float3 normal;
    float2 uv;
};

vertex VertexOut vsMain(VertexIn in [[stage_in]], constant FrameUniforms& f [[buffer(1)]]) {
    VertexOut o;
    o.position = f.viewProj * float4(in.position, 1.0);
    o.normal = in.normal.xyz * 2.0 - 1.0;
    o.uv = in.uv;
    return o;
}

fragment float4 fsMain(VertexOut in [[stage_in]],
                       constant FrameUniforms& f [[buffer(1)]],
                       constant MaterialUniforms& m [[buffer(2)]],
                       texture2d<float> tex [[texture(0)]],
                       sampler smp [[sampler(0)]]) {
    float4 c = tex.sample(smp, in.uv) * m.diffuse;
    if (m.alphaMode == 1 && c.a < 0.5) discard_fragment();
    float3 rgb = c.rgb;
    if (m.lit != 0) {
        float3 n = normalize(in.normal);
        float hemi = n.y * 0.5 + 0.5;
        float3 ambient = mix(float3(0.20, 0.18, 0.16), float3(0.45, 0.48, 0.55), hemi);
        float sun = max(dot(n, normalize(f.lightDir.xyz)), 0.0);
        rgb *= ambient + float3(1.0, 0.94, 0.84) * sun;
    }
    return float4(rgb, m.alphaMode == 2 ? c.a : 1.0);
}
)METAL";

struct FrameUniforms {
    simd_float4x4 viewProj;
    simd_float4 lightDir;
};

struct MaterialUniforms {
    simd_float4 diffuse;
    uint32_t alphaMode;
    uint32_t lit;
    uint32_t pad0, pad1;
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

}  // namespace

ResourceLibrary::Hit ResourceLibrary::find(uint32_t id, uint32_t classCrc) const {
    for (const auto& f : files_) {
        const DpcFile::Entry* e = f->find(id);
        if (e != nullptr && e->classCrc == classCrc) return {f.get(), e};
    }
    return {};
}

MeshRenderer::MeshRenderer(id<MTLDevice> device, MTLPixelFormat colorFormat, MTLPixelFormat depthFormat,
                           NSUInteger sampleCount, const ResourceLibrary& library)
    : device_(device), colorFormat_(colorFormat), depthFormat_(depthFormat), sampleCount_(sampleCount), library_(library) {
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
}

id<MTLRenderPipelineState> MeshRenderer::pipeline(uint32_t stride, AlphaMode mode) {
    const uint64_t key = (uint64_t(stride) << 8) | uint64_t(mode);
    auto it = pipelines_.find(key);
    if (it != pipelines_.end()) return it->second;

    MTLVertexDescriptor* vd = [MTLVertexDescriptor vertexDescriptor];
    vd.attributes[0].format = MTLVertexFormatFloat3;
    vd.attributes[0].offset = 0;
    vd.attributes[1].format = MTLVertexFormatUChar4Normalized_BGRA;
    vd.attributes[1].offset = 16;
    vd.attributes[2].format = MTLVertexFormatFloat2;
    vd.attributes[2].offset = 20;
    for (int i = 0; i < 3; ++i) vd.attributes[i].bufferIndex = 0;
    vd.layouts[0].stride = stride;

    MTLRenderPipelineDescriptor* pd = [MTLRenderPipelineDescriptor new];
    pd.vertexFunction = [shaders_ newFunctionWithName:@"vsMain"];
    pd.fragmentFunction = [shaders_ newFunctionWithName:@"fsMain"];
    pd.vertexDescriptor = vd;
    pd.rasterSampleCount = sampleCount_;
    pd.colorAttachments[0].pixelFormat = colorFormat_;
    pd.depthAttachmentPixelFormat = depthFormat_;
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

id<MTLTexture> MeshRenderer::texture(uint32_t bitmapId, uint8_t* alphaKind) {
    *alphaKind = 0;
    if (bitmapId == 0) return white_;
    if (auto it = textures_.find(bitmapId); it != textures_.end()) {
        *alphaKind = textureAlpha_[bitmapId];
        return it->second;
    }
    id<MTLTexture> result = white_;
    uint8_t alpha = 0;
    if (auto hit = library_.find(bitmapId, cls::Bitmap)) {
        try {
            const BitmapInfo bm = parseBitmapLink(hit.file->link(*hit.entry));
            const auto body = hit.file->body(*hit.entry);
            if (body && checkBitmap(bm, uint32_t(body->size())).empty() && bm.type == BitmapInfo::Tex2D) {
                MTLPixelFormat pf = MTLPixelFormatBGRA8Unorm;
                if (bm.format == PixelFormat::DXT1) pf = MTLPixelFormatBC1_RGBA;
                if (bm.format == PixelFormat::DXT3) pf = MTLPixelFormatBC2_RGBA;
                if (bm.format == PixelFormat::DXT5) pf = MTLPixelFormatBC3_RGBA;
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
            // Leave the white fallback in place for textures that do not decode.
        }
    }
    textures_[bitmapId] = result;
    textureAlpha_[bitmapId] = alpha;
    *alphaKind = alpha;
    return result;
}

MeshRenderer::GpuMaterial MeshRenderer::makeMaterial(uint32_t materialId) {
    GpuMaterial gm;
    gm.texture = white_;
    auto hit = library_.find(materialId, cls::Material);
    if (!hit) return gm;
    try {
        const auto body = hit.file->body(*hit.entry);
        if (!body) return gm;
        const Material m = parseMaterialBody(*body);
        uint8_t alphaKind = 0;
        gm.texture = texture(m.textures[0], &alphaKind);
        gm.diffuse = simd_make_float4(m.diffuse[0], m.diffuse[1], m.diffuse[2], m.alpha);
        // The engine derives the blend mode from the diffuse bitmap's alpha
        // kind and the material opacity; flags 0x180000 force alpha test.
        if (alphaKind == 2 || m.alpha < 1.0f)
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

bool MeshRenderer::setMesh(uint32_t meshId, std::string* error) {
    auto hit = library_.find(meshId, cls::Mesh);
    if (!hit) {
        *error = "no such Mesh_Z";
        return false;
    }
    Mesh mesh;
    MeshStats stats;
    try {
        const MeshLink link = parseMeshLink(hit.file->link(*hit.entry));
        const auto body = hit.file->body(*hit.entry);
        if (!body) {
            *error = "mesh has no body";
            return false;
        }
        mesh = parseMeshBody(*body, link.flags);
        const auto problems = checkMesh(mesh, &stats);
        if (!problems.empty()) {
            *error = problems.front();
            return false;
        }
    } catch (const std::exception& e) {
        *error = e.what();
        return false;
    }

    vbs_.clear();
    vbStrides_.clear();
    for (const VertexBuffer& vb : mesh.vbs) {
        vbs_.push_back(vb.data.empty() ? nil
                                       : [device_ newBufferWithBytes:vb.data.data()
                                                              length:vb.data.size()
                                                             options:MTLResourceStorageModeShared]);
        vbStrides_.push_back(vb.stride);
    }

    materials_.clear();
    GpuMaterial none;
    none.texture = white_;
    materials_.push_back(none);  // index 0: batches without a material
    for (uint32_t id : mesh.materials) materials_.push_back(makeMaterial(id));

    for (auto& d : draws_) d.clear();
    std::vector<uint32_t> indices;
    triangles_ = 0;
    for (const Batch& b : mesh.batches) {
        if (b.vb >= mesh.vbs.size() || mesh.vbs[b.vb].stride < kMinDrawableStride) continue;
        const uint32_t mat = b.material < mesh.materials.size() ? b.material + 1u : 0u;
        if (materials_[mat].skip) continue;
        const std::vector<uint32_t> tris = batchTriangles(mesh, b);
        Draw d{b.vb, uint32_t(indices.size()), uint32_t(tris.size()), mat};
        indices.insert(indices.end(), tris.begin(), tris.end());
        draws_[uint32_t(materials_[mat].alpha)].push_back(d);
        triangles_ += uint32_t(tris.size() / 3);
    }
    indices_ = indices.empty() ? nil
                               : [device_ newBufferWithBytes:indices.data()
                                                      length:indices.size() * 4
                                                     options:MTLResourceStorageModeShared];

    const simd_float3 lo = simd_make_float3(stats.boundsMin[0], stats.boundsMin[1], stats.boundsMin[2]);
    const simd_float3 hi = simd_make_float3(stats.boundsMax[0], stats.boundsMax[1], stats.boundsMax[2]);
    center_ = (lo + hi) * 0.5f;
    radius_ = std::max(0.05f, simd_length(hi - lo) * 0.5f);
    return true;
}

void MeshRenderer::encode(id<MTLRenderCommandEncoder> enc, float aspect, const OrbitCamera& cam) {
    if (indices_ == nil) return;
    const float dist = cam.distance * radius_;
    const simd_float3 eye = center_ + dist * simd_make_float3(std::cos(cam.pitch) * std::sin(cam.yaw), std::sin(cam.pitch),
                                                              std::cos(cam.pitch) * std::cos(cam.yaw));
    FrameUniforms fu;
    fu.viewProj = simd_mul(perspectiveRH(0.70f, aspect, radius_ * 0.02f, radius_ * 40.0f),
                           lookAtRH(eye, center_, simd_make_float3(0, 1, 0)));
    // Sun from above, slightly behind the camera's right shoulder.
    fu.lightDir = simd_make_float4(simd_normalize(simd_make_float3(std::sin(cam.yaw + 0.6f), 1.3f, std::cos(cam.yaw + 0.6f))), 0);

    [enc setVertexBytes:&fu length:sizeof fu atIndex:1];
    [enc setFragmentBytes:&fu length:sizeof fu atIndex:1];
    [enc setFrontFacingWinding:MTLWindingClockwise];
    [enc setCullMode:cullBackFaces ? MTLCullModeBack : MTLCullModeNone];
    [enc setTriangleFillMode:wireframe ? MTLTriangleFillModeLines : MTLTriangleFillModeFill];

    for (uint32_t pass = 0; pass < 3; ++pass) {
        const AlphaMode mode = AlphaMode(pass);
        [enc setDepthStencilState:mode == AlphaMode::Blend ? depthNoWrite_ : depthWrite_];
        for (const Draw& d : draws_[pass]) {
            const GpuMaterial& m = materials_[d.material];
            [enc setRenderPipelineState:pipeline(vbStrides_[d.vb], mode)];
            [enc setVertexBuffer:vbs_[d.vb] offset:0 atIndex:0];
            MaterialUniforms mu{m.diffuse, uint32_t(m.alpha), m.lit ? 1u : 0u, 0, 0};
            [enc setFragmentBytes:&mu length:sizeof mu atIndex:2];
            [enc setFragmentTexture:m.texture atIndex:0];
            [enc setFragmentSamplerState:samplers_[m.samplerIndex] atIndex:0];
            [enc drawIndexedPrimitives:MTLPrimitiveTypeTriangle
                            indexCount:d.indexCount
                             indexType:MTLIndexTypeUInt32
                           indexBuffer:indices_
                     indexBufferOffset:size_t(d.firstIndex) * 4];
        }
    }
}

}  // namespace fuel
