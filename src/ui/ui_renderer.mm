#include "ui/ui_renderer.h"
#include "viewer/gpu_profile.h"  // P92: pass timing

#include "core/resources.hpp"
#include "ui/pad_glyphs.h"

#include <algorithm>
#include <cmath>
#include <stdexcept>

namespace fuel::ui {

bool UiRenderer::padIcons = false;  // PK26

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
// disc: x, y, radius in target pixels (radius 0 = no disc); the minimap keeps its layers inside the radar.
fragment float4 uiFragment(VOut in [[stage_in]], texture2d<float> tex [[texture(0)]], sampler s [[sampler(0)]],
                           constant float4& disc [[buffer(0)]]) {
    if (disc.z > 0 && distance(in.pos.xy, disc.xy) > disc.z) discard_fragment();
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

// ---- pad-ux [port]: the pad prompts (ui/pad_glyphs.*) as glyphs of any font. Their height and vertical centre follow the
// small font's old pad icon (SMALL_FONT's A at 0x101) relative to the font's reference "É", so a prompt sits in FUEL's text
// as the old icons did; the face buttons are as tall as that icon, the other pictures keep their size relative to them.
const Glyph* UiRenderer::glyphAt(const Font& f, uint32_t c) const {
    if (c == 0xc28a) c = 10;
    int icon = pad::iconOfCode(c);
    if (icon < 0 && padIcons && c >= 1 && c < 0x20) icon = pad::iconForGlyph(int(c));  // FUEL's glyph bytes in pad mode
    if (icon < 0) return f.glyph(c);
    const pad::Atlas* a = pad::atlas(device_);
    if (!a || !a->cells[icon].ok) return c < 0x20 ? f.glyph(0x100 + c) : nullptr;  // no atlas: FUEL's own icons
    if (a->serial != padSerial_) padGlyphs_.clear(), padSerial_ = a->serial, padTex_ = a->texture, padW_ = a->width, padH_ = a->height;
    auto it = padGlyphs_.find(&f);
    if (it == padGlyphs_.end()) {
        // The old icon's height and centre in units of the small font's reference height.
        float hRel = 1.1f, cRel = 0.5f;
        const Font& sf = assets_.font(0);
        if (const Glyph* old = sf.glyph(0x101); old && sf.refHeight > 0) {
            hRel = (old->y1 - old->y0) / sf.refHeight;
            cRel = (old->top + 0.5f * (old->y1 - old->y0) - sf.refTop) / sf.refHeight;
        }
        constexpr float kFill = 0.92f;  // the old icon's circle fills ~92 % of its glyph box [port: matched by eye, GLYPHS shot]
        const float scale = hRel * f.refHeight * kFill / std::max(1.0f, a->ref);
        const float centre = f.refTop + cRel * f.refHeight;
        std::vector<Glyph> gs(pad::kIconCount);
        for (int i = 0; i < pad::kIconCount; ++i) {
            const pad::Atlas::Cell& cell = a->cells[i];
            Glyph& g = gs[size_t(i)];
            g.page = kPadPage, g.scale = scale;
            // Side bearings: a tenth of the face height each side, inside the advance (fonts keep theirs in the rect).
            const float side = 0.1f * a->ref;
            g.x0 = cell.x0 - side, g.x1 = cell.x1 + side, g.y0 = cell.y0, g.y1 = cell.y1;
            g.top = centre - cell.centre * scale;
        }
        it = padGlyphs_.emplace(&f, std::move(gs)).first;
    }
    return &it->second[size_t(icon)];
}

id<MTLTexture> UiRenderer::glyphTexture(const Font& f, const Glyph& g, float& iw, float& ih) {
    if (g.page == kPadPage) {
        iw = 1.0f / padW_, ih = 1.0f / padH_;
        return padTex_;
    }
    if (g.page >= f.pages.size()) return nil;
    const BitmapRef& pg = f.pages[g.page];
    iw = 1.0f / float(pg.width), ih = 1.0f / float(pg.height);
    return texture(pg);
}
// ---- end pad-ux

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
    clip_ = Clip{};
}

void UiRenderer::batchFor(id<MTLTexture> tex) {
    if (batches_.empty() || batches_.back().tex != tex || !sameClip(batches_.back().clip, clip_))
        batches_.push_back({tex, verts_.size(), 0, clip_});
}

void UiRenderer::setClip(float x, float y, float w, float h, float discX, float discY, float discR) {
    clip_.on = true;
    clip_.x = sx(x), clip_.y = sy(y), clip_.w = w * scale_, clip_.h = h * scaleY_;
    clip_.disc[0] = sx(discX), clip_.disc[1] = sy(discY), clip_.disc[2] = discR * scale_, clip_.disc[3] = 0;
}

void UiRenderer::clearClip() { clip_ = Clip{}; }

bool UiRenderer::sameClip(const Clip& a, const Clip& b) {
    return a.on == b.on && a.x == b.x && a.y == b.y && a.w == b.w && a.h == b.h && a.disc[0] == b.disc[0] && a.disc[1] == b.disc[1] &&
           a.disc[2] == b.disc[2];
}

void UiRenderer::sprite(const BitmapRef& b, float px, float py, float w, float h, float pivotU, float pivotV, float deg, const Rgba& c) {
    sprite(texture(b), px, py, w, h, pivotU, pivotV, deg, c);
}

void UiRenderer::sprite(id<MTLTexture> t, float px, float py, float w, float h, float pivotU, float pivotV, float deg, const Rgba& c) {
    if (!t || c.a * alpha <= 0.002f) return;
    // 0x60b490: corners relative to the pivot rotated by deg (cos/sin of deg x 0.017453292); screen y points down,
    // so a positive angle turns clockwise [inferred from the needle sweep -100 -> 80].
    const float a = deg * 0.017453292f, cs = std::cos(a), sn = std::sin(a);
    const float x0 = -pivotU * w, x1 = (1 - pivotU) * w, y0 = -pivotV * h, y1 = (1 - pivotV) * h;
    auto P = [&](float x, float y, float u, float v) {
        return Vertex{sx(px + x * cs - y * sn), sy(py + x * sn + y * cs), u, v, c.r, c.g, c.b, c.a * alpha};
    };
    const Vertex A = P(x0, y0, 0, 0), B = P(x1, y0, 1, 0), C = P(x0, y1, 0, 1), D = P(x1, y1, 1, 1);
    batchFor(t);
    for (const Vertex& v : {A, B, C, B, D, C}) verts_.push_back(v);
    batches_.back().count += 6;
}

void UiRenderer::triangles(const BitmapRef* b, const std::vector<UiVertex>& v) {
    id<MTLTexture> t = b ? texture(*b) : white_;
    if (!t || v.size() < 3) return;
    batchFor(t);
    for (size_t i = 0; i + 2 < v.size(); i += 3)
        for (size_t k = 0; k < 3; ++k) {
            const UiVertex& q = v[i + k];
            verts_.push_back({sx(q.x), sy(q.y), q.u, q.v, q.c.r, q.c.g, q.c.b, q.c.a * alpha});
        }
    batches_.back().count += v.size() / 3 * 3;
}

void UiRenderer::quad(id<MTLTexture> tex, float x0, float y0, float x1, float y1, float u0, float v0, float u1, float v1, const Rgba& c0, float shear) {
    Rgba c = c0;
    c.a *= alpha;
    if (c.a <= 0.002f) return;
    if (!tex) tex = white_;
    batchFor(tex);
    const float s = shear * (y1 - y0);
    Vertex a{x0 - s, y0, u0, v0, c.r, c.g, c.b, c.a}, b{x1 - s, y0, u1, v0, c.r, c.g, c.b, c.a};
    Vertex d{x0, y1, u0, v1, c.r, c.g, c.b, c.a}, e{x1, y1, u1, v1, c.r, c.g, c.b, c.a};
    if (rotDeg_ != 0) {  // PK18: text rotation (F9) about the text's pivot
        const float ang = rotDeg_ * 0.017453292f, cs = std::cos(ang), sn = std::sin(ang);
        for (Vertex* v : {&a, &b, &d, &e}) {
            const float dx = v->x - rotPx_, dy = v->y - rotPy_;
            v->x = rotPx_ + dx * cs - dy * sn, v->y = rotPy_ + dx * sn + dy * cs;
        }
    }
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

std::string UiRenderer::displayText(const std::string& s0, bool upper) const {
    // P75: TextGameDraw's line breaker 0x43c960: '~' = forced new line; 'µ' (C2 B5) is a zero-width marker; '§NNN'
    // (C2 A7 + 3 digits) sets the text scale NNN/100 from there on [port: dropped, the line keeps the item scale].
    std::string out;
    out.reserve(s0.size());
    for (size_t i = 0; i < s0.size(); ++i) {
        const unsigned char c = static_cast<unsigned char>(s0[i]);
        if (c == '~') { out += '\n'; continue; }
        if (c == 0xc2 && i + 1 < s0.size() && static_cast<unsigned char>(s0[i + 1]) == 0xb5) { ++i; continue; }
        if (c == 0xc2 && i + 1 < s0.size() && static_cast<unsigned char>(s0[i + 1]) == 0xa7) {
            i += 1;
            for (int d = 0; d < 3 && i + 1 < s0.size() && std::isdigit(static_cast<unsigned char>(s0[i + 1])); ++d) ++i;
            continue;
        }
        out += char(c);
    }
    if (!upper) return out;
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
        if (const Glyph* g = glyphAt(f, c)) w += (g->x1 - g->x0) * k * g->scale;  // PK26: glyphCode; pad-ux: glyphAt
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
    if (rotDeg_ != 0) {  // PK18: pivot = the text box at (fx, fy) of its width / height (first line)
        const float lw0 = textWidth(lines[0], st);
        const float cx0 = st.align == 1 ? x + (w - lw0) * 0.5f : st.align == 2 ? x + w - lw0 : x;
        rotPx_ = sx(cx0 + rotFx_ * lw0), rotPy_ = sy(y + rotFy_ * st.height);
    }
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
            const Glyph* g = glyphAt(f, c);  // PK26: pad icons; pad-ux: the pad's own
            if (!g) continue;
            gs.push_back({g, cx, fill});
            cx += (g->x1 - g->x0) * k * g->scale;
        }
        auto draw = [&](float dx, float dy, const Rgba* only) {
            for (const G& e : gs) {
                const Glyph& g = *e.g;
                float iw = 1, ih = 1;
                id<MTLTexture> t = glyphTexture(f, g, iw, ih);
                if (!t) continue;
                const float gx = e.x + dx, gy = top + (g.top - f.refTop) * k + dy;
                const float gw = (g.x1 - g.x0) * k * g.scale, gh = (g.y1 - g.y0) * k * g.scale;
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

float UiRenderer::richText(const std::string& s0, float bx0, float by0, float bx1, float by1, int align0, const TextStyle& st0, bool wrap, bool shrink) {
    const Font& f = assets_.font(st0.font);
    if (f.pages.empty() || s0.empty()) return 0;
    std::string s = s0;
    if (st0.upper)
        for (char& c : s)
            if (c >= 'a' && c <= 'z') c = char(c - 32);
    const std::vector<uint32_t> codes = utf8Codes(s);
    const size_t n = codes.size();
    auto digit = [&](size_t i) { return i < n && codes[i] >= '0' && codes[i] <= '9'; };
    auto isScale = [&](size_t i) { return codes[i] == 0xC2A7 && digit(i + 1) && digit(i + 2) && digit(i + 3); };  // '§NNN'
    auto isAlign = [&](size_t i) { return codes[i] == 0xC2B5 && i + 1 < n; };                                       // 'µX'
    auto scaleAt = [&](size_t i) { return float((codes[i + 1] - '0') * 100 + (codes[i + 2] - '0') * 10 + (codes[i + 3] - '0')) / 100.0f; };
    struct Line {
        size_t from, to;  // [from, to) in codes
        float width, maxScale, carried;  // carried: the '§' scale in force at `from`
    };
    float scale = 1;  // shrink-to-fit factor (0x43ce60: x 0.95 per pass)
    std::vector<Line> lines;
    float lineH = 0, total = 0;
    for (int pass = 0; pass < 40; ++pass) {
        const float k = st0.height * scale / f.refHeight;
        lines.clear();
        float seg = 1;  // '§' scale carries over lines
        size_t i = 0;
        while (i < n) {
            // 0x43c960: one line from i.
            Line L{i, n, 0, seg, seg};
            float w = 0;
            size_t lastBreak = SIZE_MAX;  // index of the break character (space or '|')
            float lineSeg = seg, segAtBreak = seg, maxAtBreak = seg;
            size_t j = i;
            bool done = false;
            for (; j < n; ++j) {
                const uint32_t c = codes[j];
                if (colourCode(codes, j)) { j += 3; continue; }
                if (isScale(j)) {
                    lineSeg = scaleAt(j);
                    L.maxScale = std::max(L.maxScale, lineSeg);
                    j += 3;
                    continue;
                }
                if (isAlign(j)) { ++j; continue; }
                if (c == '~') {
                    if (j == i || j + 1 == n) continue;  // not at the line start nor the last char
                    L.to = j, L.width = w, seg = lineSeg, i = j + 1, done = true;
                    break;
                }
                if ((c == ' ' && !(j + 1 < n && (codes[j + 1] == '!' || codes[j + 1] == '?'))) || c == '|')
                    lastBreak = j, segAtBreak = lineSeg, maxAtBreak = L.maxScale;
                if (c == '|') continue;
                const Glyph* g = glyphAt(f, c);  // PK26: pad icons; pad-ux: the pad's own
                const float adv = g ? (g->x1 - g->x0) * k * g->scale * lineSeg : 0;
                if (wrap && w + adv > bx1 - bx0 && j > i) {
                    if (lastBreak != SIZE_MAX && lastBreak > i) {
                        L.to = lastBreak, seg = segAtBreak, L.maxScale = maxAtBreak, i = lastBreak + 1;
                    } else {
                        L.to = j, seg = lineSeg, i = j;
                    }
                    done = true;
                    break;
                }
                w += adv;
            }
            if (!done) L.to = n, seg = lineSeg, i = n;
            // Width of [from, to) (the break character is not part of the line).
            {
                float ww = 0, cur = L.carried;
                for (size_t q = L.from; q < L.to; ++q) {
                    if (colourCode(codes, q)) { q += 3; continue; }
                    if (isScale(q)) { cur = scaleAt(q), q += 3; continue; }
                    if (isAlign(q)) { ++q; continue; }
                    if (codes[q] == '|' || codes[q] == '~') continue;
                    if (const Glyph* g = glyphAt(f, codes[q])) ww += (g->x1 - g->x0) * k * g->scale * cur;
                }
                L.width = ww;
            }
            lines.push_back(L);
            if (lines.size() >= 32) break;  // 0x43ce60 keeps 32 lines
        }
        lineH = st0.height * scale * 1.25f;  // [port] the port's line pitch (UiRenderer::text)
        total = 0;
        for (const Line& L : lines) total += lineH * L.maxScale;
        if (!shrink || total <= by1 - by0) break;
        scale *= 0.95f;
    }
    // Draw (0x43d3c0).
    const float k = st0.height * scale / f.refHeight;
    const float px = 1.0f / scale_;
    int align = align0;
    float cur = 1;
    Rgba fill = st0.fill;
    float top = by0;
    for (const Line& L : lines) {
        // State changes before the line (markup skipped by a break).
        struct G {
            const Glyph* g;
            float x, s;
            Rgba c;
        };
        std::vector<G> gs;
        const float lx = align == 1 ? (bx0 + bx1) * 0.5f - L.width * 0.5f : align == 2 ? bx1 - L.width : bx0;
        const float lh = lineH * L.maxScale;
        float cx = lx;
        for (size_t q = L.from; q < L.to; ++q) {
            const uint32_t c = codes[q];
            if (colourCode(codes, q)) {
                auto d = [&](size_t j) { return float(codes[j] - '0') / 9.0f; };
                fill = (codes[q + 1] == '0' && codes[q + 2] == '0' && codes[q + 3] == '0') ? st0.fill : Rgba{d(q + 1), d(q + 2), d(q + 3), st0.fill.a};
                q += 3;
                continue;
            }
            if (isScale(q)) { cur = scaleAt(q), q += 3; continue; }
            if (isAlign(q)) { align = codes[q + 1] == 'L' ? 0 : codes[q + 1] == 'C' ? 1 : 2, ++q; continue; }
            if (c == '|' || c == '~') continue;
            const Glyph* g = glyphAt(f, c);  // PK26: pad icons; pad-ux: the pad's own
            if (!g) continue;
            gs.push_back({g, cx, cur, fill});
            cx += (g->x1 - g->x0) * k * g->scale * cur;
        }
        auto draw = [&](float dx, float dy, const Rgba* only) {
            for (const G& e : gs) {
                const Glyph& g = *e.g;
                float iw = 1, ih = 1;
                id<MTLTexture> t = glyphTexture(f, g, iw, ih);
                if (!t) continue;
                const float kk = k * e.s;
                // [port] glyphs of a smaller '§' scale sit on the line's bottom
                const float gx = e.x + dx, gy = top + (lh - lineH * e.s) + (g.top - f.refTop) * kk + dy;
                const float gw = (g.x1 - g.x0) * kk * g.scale, gh = (g.y1 - g.y0) * kk * g.scale;
                quad(t, sx(gx), sy(gy), sx(gx + gw), sy(gy + gh), g.x0 * iw, g.y0 * ih, g.x1 * iw, g.y1 * ih, only ? *only : e.c, st0.slant);
            }
        };
        if (st0.outline.a > 0.01f) {
            const float o = px * std::max(1.0f, std::round(scale_));
            for (int oy = -1; oy <= 1; ++oy)
                for (int ox = -1; ox <= 1; ++ox)
                    if (ox || oy) draw(ox * o, oy * o, &st0.outline);
        }
        draw(0, 0, nullptr);
        top += lh;
    }
    return total;
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
    gpuprof::tag(rp, "hud");  // P92: per-pass GPU timing (no-op unless profiling)
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
            [enc setFragmentBytes:b.clip.disc length:sizeof b.clip.disc atIndex:0];
            MTLScissorRect sc{0, 0, NSUInteger(targetW_), NSUInteger(targetH_)};
            if (b.clip.on) {
                const float x0 = std::clamp(b.clip.x, 0.0f, targetW_), y0 = std::clamp(b.clip.y, 0.0f, targetH_);
                const float x1 = std::clamp(b.clip.x + b.clip.w, x0, targetW_), y1 = std::clamp(b.clip.y + b.clip.h, y0, targetH_);
                sc = {NSUInteger(x0), NSUInteger(y0), NSUInteger(x1 - x0), NSUInteger(y1 - y0)};
                if (sc.width == 0 || sc.height == 0) continue;
            }
            [enc setScissorRect:sc];
            [enc drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:b.first vertexCount:b.count];
        }
    }
    [enc endEncoding];
}

}  // namespace fuel::ui
