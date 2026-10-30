#include "ui/ui_renderer.h"

#include "core/resources.hpp"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace fuel::ui {
namespace {

const char* kUiShader = R"(
#include <metal_stdlib>
using namespace metal;
struct VIn { float2 pos [[attribute(0)]]; float2 uv [[attribute(1)]]; float4 col [[attribute(2)]]; };
struct VOut { float4 pos [[position]]; float2 uv; float4 col; };
// The menu colours are authored in gamma space; the target is sRGB, so they are
// linearised here (the textures are sampled through sRGB formats).
static float3 toLinear(float3 c) { return select(pow((c + 0.055) / 1.055, 2.4), c / 12.92, c <= 0.04045); }
vertex VOut uiVertex(VIn in [[stage_in]], constant float2& target [[buffer(1)]]) {
    VOut o;
    o.pos = float4(in.pos.x / target.x * 2.0 - 1.0, 1.0 - in.pos.y / target.y * 2.0, 0, 1);
    o.uv = in.uv;
    o.col = float4(toLinear(in.col.rgb), in.col.a);
    return o;
}
fragment float4 uiFragment(VOut in [[stage_in]], texture2d<float> tex [[texture(0)]], sampler s [[sampler(0)]]) {
    return tex.sample(s, in.uv) * in.col;
}
)";

// "^RGB": a caret and three digits (TextGameDraw colour code).
bool colourCode(const std::vector<uint32_t>& c, size_t i) {
    if (c[i] != '^' || i + 3 >= c.size()) return false;
    for (size_t k = 1; k <= 3; ++k)
        if (c[i + k] < '0' || c[i + k] > '9') return false;
    return true;
}

}  // namespace

UiRenderer::UiRenderer(id<MTLDevice> device, MTLPixelFormat target, const MenuAssets& assets) : device_(device), assets_(assets) {
    NSError* err = nil;
    id<MTLLibrary> lib = [device newLibraryWithSource:@(kUiShader) options:nil error:&err];
    if (!lib) throw std::runtime_error(std::string("ui shader: ") + err.localizedDescription.UTF8String);
    MTLVertexDescriptor* vd = [MTLVertexDescriptor vertexDescriptor];
    vd.attributes[0].format = MTLVertexFormatFloat2, vd.attributes[0].offset = 0;
    vd.attributes[1].format = MTLVertexFormatFloat2, vd.attributes[1].offset = 8;
    vd.attributes[2].format = MTLVertexFormatFloat4, vd.attributes[2].offset = 16;
    vd.layouts[0].stride = sizeof(Vertex);
    MTLRenderPipelineDescriptor* pd = [MTLRenderPipelineDescriptor new];
    pd.vertexFunction = [lib newFunctionWithName:@"uiVertex"];
    pd.fragmentFunction = [lib newFunctionWithName:@"uiFragment"];
    pd.vertexDescriptor = vd;
    pd.colorAttachments[0].pixelFormat = target;
    pd.colorAttachments[0].blendingEnabled = YES;
    pd.colorAttachments[0].sourceRGBBlendFactor = MTLBlendFactorSourceAlpha;
    pd.colorAttachments[0].destinationRGBBlendFactor = MTLBlendFactorOneMinusSourceAlpha;
    pd.colorAttachments[0].sourceAlphaBlendFactor = MTLBlendFactorOne;
    pd.colorAttachments[0].destinationAlphaBlendFactor = MTLBlendFactorOneMinusSourceAlpha;
    pipeline_ = [device newRenderPipelineStateWithDescriptor:pd error:&err];
    if (!pipeline_) throw std::runtime_error(std::string("ui pipeline: ") + err.localizedDescription.UTF8String);
    MTLSamplerDescriptor* sd = [MTLSamplerDescriptor new];
    sd.minFilter = sd.magFilter = MTLSamplerMinMagFilterLinear;
    sd.mipFilter = MTLSamplerMipFilterLinear;
    sd.sAddressMode = sd.tAddressMode = MTLSamplerAddressModeClampToEdge;
    sampler_ = [device newSamplerStateWithDescriptor:sd];
    MTLTextureDescriptor* td = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatRGBA8Unorm width:1 height:1 mipmapped:NO];
    white_ = [device newTextureWithDescriptor:td];
    const uint32_t px = 0xffffffff;
    [white_ replaceRegion:MTLRegionMake2D(0, 0, 1, 1) mipmapLevel:0 withBytes:&px bytesPerRow:4];
}

id<MTLTexture> UiRenderer::texture(const BitmapRef& b) {
    if (!b) return nil;
    const uint64_t key = (uint64_t(reinterpret_cast<uintptr_t>(b.file)) << 32) ^ b.bitmapId;
    if (auto it = textures_.find(key); it != textures_.end()) return it->second;
    id<MTLTexture> t = nil;
    try {
        const DpcFile::Entry* e = b.file->find(b.bitmapId);
        const BitmapInfo bm = parseBitmapLink(b.file->link(*e));
        const auto body = b.file->body(*e);
        MTLPixelFormat pf = MTLPixelFormatInvalid;
        if (bm.format == PixelFormat::DXT1) pf = MTLPixelFormatBC1_RGBA_sRGB;
        if (bm.format == PixelFormat::DXT3) pf = MTLPixelFormatBC2_RGBA_sRGB;
        if (bm.format == PixelFormat::DXT5) pf = MTLPixelFormatBC3_RGBA_sRGB;
        if (bm.format == PixelFormat::A8R8G8B8) pf = MTLPixelFormatBGRA8Unorm_sRGB;
        if (body && pf != MTLPixelFormatInvalid && bm.type == BitmapInfo::Tex2D && checkBitmap(bm, uint32_t(body->size())).empty()) {
            MTLTextureDescriptor* td = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:pf width:bm.width height:bm.height mipmapped:bm.mips > 1];
            td.mipmapLevelCount = bm.mips;
            t = [device_ newTextureWithDescriptor:td];
            const auto sizes = bm.mipSizes();
            size_t off = 0;
            uint32_t w = bm.width, h = bm.height;
            for (uint32_t level = 0; level < bm.mips; ++level) {
                const NSUInteger row = bm.isBlockCompressed() ? std::max(1u, (w + 3) / 4) * (bm.format == PixelFormat::DXT1 ? 8 : 16) : w * 4;
                [t replaceRegion:MTLRegionMake2D(0, 0, w, h) mipmapLevel:level withBytes:body->data() + off bytesPerRow:row];
                off += sizes[level];
                w = std::max(1u, w >> 1), h = std::max(1u, h >> 1);
            }
        }
    } catch (const std::exception&) {
        t = nil;
    }
    textures_[key] = t;
    return t;
}

void UiRenderer::begin(float w, float h) {
    targetW_ = w, targetH_ = h;
    // FUEL.exe 0x49bec0: full width, height = min(H, W * 9/16), centred vertically.
    // (A window narrower than 16:9 is pillarboxed instead so nothing is cut off.)
    float cw = w, ch = std::min(h, w * 0.5625f);
    if (h < w * 0.5625f) cw = h / 0.5625f, ch = h;
    offX_ = (w - cw) * 0.5f, offY_ = (h - ch) * 0.5f;
    scale_ = cw / 1280.0f, scaleY_ = ch / 720.0f;
    verts_.clear();
    batches_.clear();
}

void UiRenderer::quad(id<MTLTexture> tex, float x0, float y0, float x1, float y1, float u0, float v0, float u1, float v1, const Rgba& c, float shear) {
    if (c.a <= 0.002f) return;
    if (!tex) tex = white_;
    if (batches_.empty() || batches_.back().tex != tex) batches_.push_back({tex, verts_.size(), 0});
    const float s = shear * (y1 - y0);
    const Vertex a{x0 - s, y0, u0, v0, c.r, c.g, c.b, c.a}, b{x1 - s, y0, u1, v0, c.r, c.g, c.b, c.a};
    const Vertex d{x0, y1, u0, v1, c.r, c.g, c.b, c.a}, e{x1, y1, u1, v1, c.r, c.g, c.b, c.a};
    for (const Vertex& v : {a, b, d, b, e, d}) verts_.push_back(v);
    batches_.back().count += 6;
}

void UiRenderer::rect(float x, float y, float w, float h, const Rgba& c) {
    quad(nil, sx(x), sy(y), sx(x + w), sy(y + h), 0, 0, 1, 1, c);
}

void UiRenderer::image(const BitmapRef& b, float x, float y, float w, float h, const Rgba& c, float u0, float v0, float u1, float v1) {
    id<MTLTexture> t = texture(b);
    if (!t) return;
    quad(t, sx(x), sy(y), sx(x + w), sy(y + h), u0, v0, u1, v1, c);
}

std::string UiRenderer::displayText(const std::string& s, bool upper) const {
    if (!upper) return s;
    std::string out = s;
    // 0x8dfef9 upper-cases byte-wise in the C locale: ASCII only.
    for (char& c : out)
        if (c >= 'a' && c <= 'z') c = char(c - 32);
    return out;
}

float UiRenderer::textWidth(const std::string& s, const TextStyle& st) const {
    const Font& f = assets_.font(st.font);
    const float k = st.height / f.refHeight;
    float w = 0, best = 0;
    const auto codes = utf8Codes(displayText(s, st.upper));
    for (size_t i = 0; i < codes.size(); ++i) {
        const uint32_t c = codes[i];
        if (colourCode(codes, i)) {
            i += 3;
            continue;
        }
        if (c == '\n') {
            best = std::max(best, w), w = 0;
            continue;
        }
        if (const Glyph* g = f.glyph(c)) w += (g->x1 - g->x0) * k;
    }
    return std::max(best, w);
}

void UiRenderer::text(const std::string& s, float x, float y, float w, const TextStyle& st) {
    const Font& f = assets_.font(st.font);
    if (f.pages.empty()) return;
    const float k = st.height / f.refHeight;
    const std::string shown = displayText(s, st.upper);
    // Lines (a literal "\n" in the tables also breaks a line).
    std::vector<std::string> lines;
    std::string cur;
    for (size_t i = 0; i < shown.size(); ++i) {
        if (shown[i] == '\n' || (shown[i] == '\\' && i + 1 < shown.size() && shown[i + 1] == 'n')) {
            lines.push_back(cur), cur.clear();
            if (shown[i] == '\\') ++i;
            continue;
        }
        cur += shown[i];
    }
    lines.push_back(cur);
    const float lineH = st.height * 1.25f;
    const float px = 1.0f / scale_;  // one target pixel in canvas units
    for (size_t li = 0; li < lines.size(); ++li) {
        const auto codes = utf8Codes(lines[li]);
        TextStyle one = st;
        float lw = textWidth(lines[li], one);
        float cx = st.align == 1 ? x + (w - lw) * 0.5f : st.align == 2 ? x + w - lw : x;
        const float top = y + float(li) * lineH;
        Rgba fill = st.fill;
        struct G {
            const Glyph* g;
            float x;
            Rgba c;
        };
        std::vector<G> gs;
        for (size_t i = 0; i < codes.size(); ++i) {
            const uint32_t c = codes[i];
            if (colourCode(codes, i)) {
                // ^RGB colour code (digits 0-9); ^000 returns to the item colour [inferred].
                auto d = [&](size_t j) { return float(codes[j] - '0') / 9.0f; };
                if (codes[i + 1] == '0' && codes[i + 2] == '0' && codes[i + 3] == '0')
                    fill = st.fill;
                else
                    fill = Rgba{d(i + 1), d(i + 2), d(i + 3), st.fill.a};
                i += 3;
                continue;
            }
            const Glyph* g = f.glyph(c);
            if (!g) continue;
            gs.push_back({g, cx, fill});
            cx += (g->x1 - g->x0) * k;
        }
        auto draw = [&](float dx, float dy, const Rgba* only) {
            for (const G& e : gs) {
                const Glyph& g = *e.g;
                if (g.page >= f.pages.size()) continue;
                const BitmapRef& pg = f.pages[g.page];
                id<MTLTexture> t = texture(pg);
                if (!t) continue;
                const float gx = e.x + dx, gy = top + (g.top - f.refTop) * k + dy;
                const float gw = (g.x1 - g.x0) * k, gh = (g.y1 - g.y0) * k;
                const float iw = 1.0f / float(pg.width), ih = 1.0f / float(pg.height);
                quad(t, sx(gx), sy(gy), sx(gx + gw), sy(gy + gh), g.x0 * iw, g.y0 * ih, g.x1 * iw, g.y1 * ih, only ? *only : e.c, st.slant);
            }
        };
        if (st.outline.a > 0.01f) {
            const float o = px * std::max(1.0f, std::round(scale_));
            for (int oy = -1; oy <= 1; ++oy)
                for (int ox = -1; ox <= 1; ++ox)
                    if (ox || oy) draw(ox * o, oy * o, &st.outline);
        }
        draw(0, 0, nullptr);
    }
}

void UiRenderer::textFit(const std::string& s, float x, float y, float w, TextStyle st) {
    const float tw = textWidth(s, st);
    if (w > 0 && tw > w) {
        const float k = w / tw;
        const float h0 = st.height;
        st.height *= k;
        y += (h0 - st.height) * 0.5f;
    }
    text(s, x, y, w, st);
}

void UiRenderer::encode(id<MTLCommandBuffer> cb, id<MTLTexture> target, bool clear) {
    MTLRenderPassDescriptor* rp = [MTLRenderPassDescriptor renderPassDescriptor];
    rp.colorAttachments[0].texture = target;
    rp.colorAttachments[0].loadAction = clear ? MTLLoadActionClear : MTLLoadActionLoad;
    rp.colorAttachments[0].clearColor = MTLClearColorMake(0, 0, 0, 1);
    rp.colorAttachments[0].storeAction = MTLStoreActionStore;
    id<MTLRenderCommandEncoder> enc = [cb renderCommandEncoderWithDescriptor:rp];
    if (!verts_.empty()) {
        [enc setRenderPipelineState:pipeline_];
        id<MTLBuffer> vb = [device_ newBufferWithBytes:verts_.data() length:verts_.size() * sizeof(Vertex) options:MTLResourceStorageModeShared];
        [enc setVertexBuffer:vb offset:0 atIndex:0];
        const float tsize[2] = {targetW_, targetH_};
        [enc setVertexBytes:tsize length:sizeof tsize atIndex:1];
        [enc setFragmentSamplerState:sampler_ atIndex:0];
        for (const Batch& b : batches_) {
            [enc setFragmentTexture:b.tex atIndex:0];
            [enc drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:b.first vertexCount:b.count];
        }
    }
    [enc endEncoding];
}

}  // namespace fuel::ui
