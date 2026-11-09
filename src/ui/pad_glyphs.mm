// [port] DEVIATION (user request 2026-11-06): real Xbox / PlayStation button prompts. See pad_glyphs.h, pad_prompts.hpp
// and ~/re-work/fuel/notes/pad-ux.md.
#include "ui/pad_glyphs.h"

#include "core/input_map.hpp"
#include "ui/menu_assets.hpp"
#include "ui/ui_renderer.h"

#import <Cocoa/Cocoa.h>
#import <GameController/GameController.h>
#import <ImageIO/ImageIO.h>
#import <UniformTypeIdentifiers/UniformTypeIdentifiers.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <mutex>
#include <vector>

namespace fuel::ui::pad {
namespace {

std::string str(NSString* s) { return s ? std::string(s.UTF8String) : std::string(); }

NSColor* colour(uint32_t rgba) {
    return [NSColor colorWithSRGBRed:double((rgba >> 24) & 0xff) / 255.0 green:double((rgba >> 16) & 0xff) / 255.0
                                blue:double((rgba >> 8) & 0xff) / 255.0 alpha:double(rgba & 0xff) / 255.0];
}

// The symbol image at `pointSize` with the look's palette (nil when this macOS has no such symbol).
NSImage* symbolImage(const std::string& name, const Look& l, CGFloat pointSize) {
    if (name.empty()) return nil;
    NSImage* img = [NSImage imageWithSystemSymbolName:@(name.c_str()) accessibilityDescription:nil];
    if (!img) return nil;
    NSImageSymbolConfiguration* cfg = [NSImageSymbolConfiguration configurationWithPointSize:pointSize weight:NSFontWeightSemibold
                                                                                       scale:NSImageSymbolScaleMedium];
    cfg = [cfg configurationByApplyingConfiguration:[NSImageSymbolConfiguration configurationWithPaletteColors:@[ colour(l.primary), colour(l.secondary) ]]];
    return [img imageWithSymbolConfiguration:cfg];
}

constexpr CGFloat kPoint = 56;      // symbol point size (x 2 texels): a face button is ~112 texels high
constexpr int kAtlasW = 2048, kRowH = 176, kGutter = 6;

// A pill (rounded outline + caption), PlayStation's Options / Create and the fallback for a missing symbol.
NSSize pillSize(const std::string& caption, CGFloat ref, NSDictionary** attrs) {
    NSFont* font = [NSFont systemFontOfSize:ref * 0.40 weight:NSFontWeightBold];
    *attrs = @{NSFontAttributeName : font, NSForegroundColorAttributeName : NSColor.whiteColor, NSKernAttributeName : @(ref * 0.02)};
    const NSSize t = [@(caption.c_str()) sizeWithAttributes:*attrs];
    return NSMakeSize(std::ceil(t.width + ref * 0.48), std::ceil(ref * 0.72));
}

void drawPill(const std::string& caption, NSRect r, CGFloat ref, NSDictionary* attrs) {
    const CGFloat line = ref * 0.075;
    NSBezierPath* p = [NSBezierPath bezierPathWithRoundedRect:NSInsetRect(r, line * 0.5, line * 0.5) xRadius:r.size.height * 0.5 - line * 0.5
                                                      yRadius:r.size.height * 0.5 - line * 0.5];
    [[NSColor colorWithSRGBRed:0.08 green:0.08 blue:0.09 alpha:0.85] setFill];
    [p fill];
    p.lineWidth = line;
    [NSColor.whiteColor setStroke];
    [p stroke];
    NSString* s = @(caption.c_str());
    const NSSize t = [s sizeWithAttributes:attrs];
    [s drawAtPoint:NSMakePoint(r.origin.x + (r.size.width - t.width) * 0.5, r.origin.y + (r.size.height - t.height) * 0.5) withAttributes:attrs];
}

struct AtlasStore {
    std::mutex m;
    std::map<void*, Atlas> byDevice;
};
AtlasStore& store() {
    static AtlasStore s;
    return s;
}

void build(Atlas& a, id<MTLDevice> device) {
    @autoreleasepool {
        const State& st = state();
        // Pictures first (sizes), then the packing.
        struct Pic {
            NSImage* img = nil;
            NSImage* img2 = nil;
            std::string pill;
            NSDictionary* attrs = nil;
            NSSize size = NSZeroSize;
            int pair = 0;          // 1: img over img2 (stick up / down), 2: img left of img2 (left / right)
            CGFloat centre = -1;   // the shape's vertical centre in the cell (texels), < 0 = the cell's middle
        };
        std::vector<Pic> pics(kIconCount);
        // The reference height: a face button (the circle) at the symbol size.
        Look faceLook = look(Family::Xbox, fuel::input::PadA);
        NSImage* face = symbolImage(faceLook.symbol, faceLook, kPoint);
        const CGFloat ref = face ? face.size.height * 2.0 : kPoint * 2.4;  // texels (2x: the image size is in points)
        a.ref = float(ref);
        a.symbols = a.pills = a.missing = 0;
        for (int i = 0; i < kIconCount; ++i) {
            const Look l = currentLook(i);
            Pic& p = pics[size_t(i)];
            if (l.pill.empty()) {
                p.img = symbolImage(l.symbol, l, kPoint * l.scale);
                if (!p.img) {  // a pad's own symbol this macOS lacks: the table's
                    const Look t = look(st.family, i);
                    p.img = symbolImage(t.symbol, t, kPoint * t.scale);
                }
                p.img2 = symbolImage(l.symbol2, l, kPoint * l.scale);
            }
            if (p.img) {
                const NSSize s1 = NSMakeSize(std::ceil(p.img.size.width * 2.0), std::ceil(p.img.size.height * 2.0));
                p.size = s1;
                // The stick-direction symbols carry their arrow on one side of the ring: the ring is the shape to centre on
                // the text, and the two-arrow pictures (FUEL's UPDOWN / LEFTRIGHT) overlap two of them on the ring.
                const bool up = l.symbol.find("tilt.up") != std::string::npos, down = l.symbol.find("tilt.down") != std::string::npos;
                if (up) p.centre = s1.height - s1.width * 0.5;  // ring diameter = the image width
                if (down) p.centre = s1.width * 0.5;
                if (p.img2) {
                    const NSSize s2 = NSMakeSize(std::ceil(p.img2.size.width * 2.0), std::ceil(p.img2.size.height * 2.0));
                    if (up) {
                        p.pair = 1;
                        p.size = NSMakeSize(std::max(s1.width, s2.width), s1.height + s2.height - s1.width);
                        p.centre = s1.height - s1.width * 0.5;
                    } else {
                        p.pair = 2;
                        p.size = NSMakeSize(s1.width + s2.width - s1.height, std::max(s1.height, s2.height));
                    }
                }
                ++a.symbols;
            } else {
                // [port] Options / Create pills, or the button's name when the symbol is missing (older macOS).
                p.pill = l.pill.empty() ? buttonName(i) : l.pill;
                for (char& c : p.pill) c = char(std::toupper(static_cast<unsigned char>(c)));
                NSDictionary* attrs = nil;
                p.size = pillSize(p.pill, ref, &attrs);
                p.attrs = attrs;
                l.pill.empty() ? ++a.missing : ++a.pills;
            }
        }
        // Pack: rows of kRowH texels; each picture keeps a gutter around it (mip levels do not bleed).
        int x = kGutter, y = kGutter;
        std::vector<NSRect> rects(kIconCount);
        for (int i = 0; i < kIconCount; ++i) {
            const NSSize s = pics[size_t(i)].size;
            if (x + int(s.width) + kGutter > kAtlasW) x = kGutter, y += kRowH;
            rects[size_t(i)] = NSMakeRect(x, y, s.width, s.height);
            x += int(s.width) + 2 * kGutter;
        }
        int h = y + kRowH;
        h = (h + 63) / 64 * 64;
        // Draw into an sRGB bitmap (premultiplied), y down as the texture.
        std::vector<uint8_t> px(size_t(kAtlasW) * size_t(h) * 4, 0);
        CGColorSpaceRef cs = CGColorSpaceCreateWithName(kCGColorSpaceSRGB);
        CGContextRef cg = CGBitmapContextCreate(px.data(), kAtlasW, size_t(h), 8, size_t(kAtlasW) * 4, cs, kCGImageAlphaPremultipliedLast);
        CGColorSpaceRelease(cs);
        if (!cg) return;
        CGContextTranslateCTM(cg, 0, h);
        CGContextScaleCTM(cg, 1, -1);  // y down
        NSGraphicsContext* ctx = [NSGraphicsContext graphicsContextWithCGContext:cg flipped:YES];
        [NSGraphicsContext saveGraphicsState];
        NSGraphicsContext.currentContext = ctx;
        for (int i = 0; i < kIconCount; ++i) {
            const Pic& p = pics[size_t(i)];
            const NSRect r = rects[size_t(i)];
            if (p.img) {
                int k = 0;
                for (NSImage* im : {p.img, p.img2}) {
                    if (!im) continue;
                    const NSSize s = NSMakeSize(im.size.width * 2.0, im.size.height * 2.0);
                    NSRect d = NSMakeRect(r.origin.x + (r.size.width - s.width) * 0.5, r.origin.y + (r.size.height - s.height) * 0.5, s.width, s.height);
                    if (p.pair == 1) d.origin.y = k == 0 ? r.origin.y : r.origin.y + r.size.height - s.height;  // up on top, down at the bottom
                    if (p.pair == 2) d.origin.x = k == 0 ? r.origin.x : r.origin.x + r.size.width - s.width;    // left, right
                    [im drawInRect:d fromRect:NSZeroRect operation:NSCompositingOperationSourceOver fraction:1 respectFlipped:YES hints:nil];
                    ++k;
                }
            } else {
                drawPill(p.pill, r, ref, p.attrs);
            }
            Atlas::Cell& c = a.cells[i];
            c.ok = true;
            c.x0 = float(r.origin.x), c.y0 = float(r.origin.y), c.x1 = float(r.origin.x + r.size.width), c.y1 = float(r.origin.y + r.size.height);
            c.centre = float(p.centre >= 0 ? p.centre : r.size.height * 0.5);
        }
        [NSGraphicsContext restoreGraphicsState];
        CGContextRelease(cg);
        // Tight boxes: an SF Symbol image carries the font's padding around the shape; the cells keep the drawn shape
        // (+2 texels for the filtering) so sizes and centres are the shapes'. The face button's tight height is the
        // reference every prompt is scaled by.
        for (int i = 0; i < kIconCount; ++i) {
            Atlas::Cell& c = a.cells[i];
            const int x0 = int(c.x0), y0 = int(c.y0), x1 = int(c.x1), y1 = int(c.y1);
            int bx0 = x1, by0 = y1, bx1 = x0 - 1, by1 = y0 - 1;
            for (int yy = y0; yy < y1 && yy < h; ++yy)
                for (int xx = x0; xx < x1 && xx < kAtlasW; ++xx)
                    if (px[(size_t(yy) * kAtlasW + size_t(xx)) * 4 + 3] > 8) bx0 = std::min(bx0, xx), bx1 = std::max(bx1, xx), by0 = std::min(by0, yy), by1 = std::max(by1, yy);
            if (bx1 < bx0) continue;  // nothing drawn: keep the box
            const Pic& p = pics[size_t(i)];
            const float tw = float(bx1 - bx0 + 1), th = float(by1 - by0 + 1);
            float centre = float(by0) + th * 0.5f;  // the shape's middle (absolute texels)
            const Look l = currentLook(i);
            if (p.pair == 0 && l.symbol.find("tilt.up") != std::string::npos) centre = float(by1 + 1) - tw * 0.5f;    // the ring under the arrow
            if (p.pair == 0 && l.symbol.find("tilt.down") != std::string::npos) centre = float(by0) + tw * 0.5f;     // the ring over the arrow
            c.x0 = float(bx0 - 2), c.y0 = float(by0 - 2), c.x1 = float(bx1 + 3), c.y1 = float(by1 + 3);
            c.centre = centre - c.y0;
            if (i == fuel::input::PadA) a.ref = th;
        }
        // Straight alpha (the UI pipeline blends source-alpha), the colour of the nearest drawn texel under transparent
        // ones (no dark fringes when filtered or mip-mapped).
        const int W = kAtlasW, H = h;
        for (size_t i = 0; i < px.size(); i += 4)
            if (const int al = px[i + 3]; al > 0 && al < 255)
                for (int k = 0; k < 3; ++k) px[i + size_t(k)] = uint8_t(std::min(255, px[i + size_t(k)] * 255 / al));
        std::vector<uint8_t> filled(size_t(W) * size_t(H), 0);
        for (size_t i = 0; i < filled.size(); ++i) filled[i] = px[i * 4 + 3] > 0;
        for (int pass = 0; pass < 6; ++pass) {
            std::vector<uint8_t> next = filled;
            for (int yy = 0; yy < H; ++yy)
                for (int xx = 0; xx < W; ++xx) {
                    const size_t o = size_t(yy) * size_t(W) + size_t(xx);
                    if (filled[o]) continue;
                    int r = 0, g = 0, b = 0, n = 0;
                    for (int dy = -1; dy <= 1; ++dy)
                        for (int dx = -1; dx <= 1; ++dx) {
                            const int nx = xx + dx, ny = yy + dy;
                            if (nx < 0 || ny < 0 || nx >= W || ny >= H) continue;
                            const size_t q = size_t(ny) * size_t(W) + size_t(nx);
                            if (!filled[q]) continue;
                            r += px[q * 4], g += px[q * 4 + 1], b += px[q * 4 + 2], ++n;
                        }
                    if (!n) continue;
                    px[o * 4] = uint8_t(r / n), px[o * 4 + 1] = uint8_t(g / n), px[o * 4 + 2] = uint8_t(b / n);
                    next[o] = 1;
                }
            filled.swap(next);
        }
        MTLTextureDescriptor* td = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatRGBA8Unorm_sRGB width:NSUInteger(W)
                                                                                    height:NSUInteger(H) mipmapped:YES];
        td.usage = MTLTextureUsageShaderRead;
        id<MTLTexture> t = [device newTextureWithDescriptor:td];
        [t replaceRegion:MTLRegionMake2D(0, 0, NSUInteger(W), NSUInteger(H)) mipmapLevel:0 withBytes:px.data() bytesPerRow:NSUInteger(W) * 4];
        id<MTLCommandQueue> q = [device newCommandQueue];
        id<MTLCommandBuffer> cb = [q commandBuffer];
        id<MTLBlitCommandEncoder> bl = [cb blitCommandEncoder];
        [bl generateMipmapsForTexture:t];
        [bl endEncoding];
        [cb commit];
        [cb waitUntilCompleted];
        a.texture = t;
        a.width = float(W), a.height = float(H);
    }
}

}  // namespace

State detect(Style style) {
    State s = state();  // keeps the serial; setState compares the rest
    s.liveSymbol.clear(), s.liveName.clear(), s.padName.clear();
    GCController* pick = nil;
    for (GCController* c in GCController.controllers)
        if (c.extendedGamepad) {
            pick = c;
            break;
        }
    s.present = pick != nil;
    bool ps = false;
    if (pick) {
        GCExtendedGamepad* g = pick.extendedGamepad;
        if (@available(macOS 11.0, *)) ps = [g isKindOfClass:[GCDualSenseGamepad class]] || [g isKindOfClass:[GCDualShockGamepad class]];
        s.padName = str(pick.vendorName);
        if (@available(macOS 11.0, *)) {
            using namespace fuel::input;
            const std::pair<int, GCControllerElement*> els[] = {
                {PadA, g.buttonA},           {PadB, g.buttonB},          {PadX, g.buttonX},
                {PadY, g.buttonY},           {PadLB, g.leftShoulder},    {PadRB, g.rightShoulder},
                {PadLT, g.leftTrigger},      {PadRT, g.rightTrigger},    {PadLStick, g.leftThumbstickButton},
                {PadRStick, g.rightThumbstickButton}, {PadBack, g.buttonOptions}, {PadStart, g.buttonMenu}};
            for (const auto& [icon, el] : els) {
                if (!el) continue;
                if (el.sfSymbolsName) s.liveSymbol[icon] = str(el.sfSymbolsName);
                if (el.localizedName) s.liveName[icon] = str(el.localizedName);
            }
        }
    }
    // $FUEL_PAD_FAMILY = playstation | xbox: test a family without its pad (the pad's presence still counts).
    Style eff = style;
    if (const char* f = std::getenv("FUEL_PAD_FAMILY"); f && *f) eff = styleFromSetting(f);
    s.family = resolve(eff, ps);
    s.forced = eff != Style::Auto;
    return s;
}

State shotState(Style style) {
    State s = state();
    s.liveSymbol.clear(), s.liveName.clear(), s.padName.clear();
    const char* v = std::getenv("FUEL_SHOT_PAD");
    const std::string e = v ? v : "";
    s.present = !e.empty() && e != "0";
    Style eff = style;
    if (e != "1" && !e.empty()) eff = styleFromSetting(e) == Style::Auto ? style : styleFromSetting(e);
    s.family = resolve(eff, false);
    s.forced = true;
    s.padName = s.present ? std::string(familyName(s.family)) + " (shot)" : std::string();
    return s;
}

const Atlas* atlas(id<MTLDevice> device) {
    if (!device) return nullptr;
    AtlasStore& st = store();
    std::lock_guard<std::mutex> lk(st.m);
    Atlas& a = st.byDevice[(__bridge void*)device];
    if (a.serial != state().serial || !a.texture) {
        a = Atlas{};
        build(a, device);
        a.serial = state().serial;
        std::printf("pad prompts: %s atlas (%d symbols, %d pills, %d by name) %.0fx%.0f\n", familyName(state().family), a.symbols, a.pills,
                    a.missing, a.width, a.height);
    }
    return a.texture ? &a : nullptr;
}

bool writeAtlasPng(id<MTLDevice> device, const std::string& path) {
    const Atlas* a = atlas(device);
    if (!a) return false;
    const size_t W = size_t(a->width), H = size_t(a->height);
    std::vector<uint8_t> px(W * H * 4);
    // The texture is shared-storage by default on Apple silicon (newTextureWithDescriptor without a storage mode).
    [a->texture getBytes:px.data() bytesPerRow:W * 4 fromRegion:MTLRegionMake2D(0, 0, W, H) mipmapLevel:0];
    CGColorSpaceRef cs = CGColorSpaceCreateWithName(kCGColorSpaceSRGB);
    CGDataProviderRef dp = CGDataProviderCreateWithData(nullptr, px.data(), px.size(), nullptr);
    CGImageRef img = CGImageCreate(W, H, 8, 32, W * 4, cs, kCGImageAlphaLast, dp, nullptr, false, kCGRenderingIntentDefault);
    CFURLRef url = CFURLCreateFromFileSystemRepresentation(nullptr, reinterpret_cast<const UInt8*>(path.c_str()), CFIndex(path.size()), false);
    CGImageDestinationRef d = CGImageDestinationCreateWithURL(url, (__bridge CFStringRef)UTTypePNG.identifier, 1, nullptr);
    bool ok = false;
    if (d) {
        CGImageDestinationAddImage(d, img, nullptr);
        ok = CGImageDestinationFinalize(d);
        CFRelease(d);
    }
    CFRelease(url), CGImageRelease(img), CGDataProviderRelease(dp), CGColorSpaceRelease(cs);
    return ok;
}

bool paddleCheck(const State& s, const std::function<std::string(int)>& keyLabels, const HelpTexts::PadLabelFn& padLabels) {
    if (setState(s) && s.present)
        std::printf("pad prompts: %s%s (%s)\n", familyName(s.family), s.forced ? " (forced)" : "", s.padName.empty() ? "pad" : s.padName.c_str());
    const int variant = s.present ? 1 + int(s.family) : 0;
    const bool rebuilt = helpTexts().padCheck(s.present, keyLabels, padLabels, variant);
    UiRenderer::padIcons = helpTexts().padMode();
    return rebuilt;
}

}  // namespace fuel::ui::pad

namespace fuel::ui {

int runPadAtlasCheck(const std::string& dataDir) {
    int bad = 0;
    auto expect = [&](bool ok, const std::string& what) {
        if (!ok) ++bad, std::printf("pad atlas: FAIL %s\n", what.c_str());
    };
    id<MTLDevice> device = MTLCreateSystemDefaultDevice();
    if (!device) {
        std::printf("pad atlas: no Metal device (skipped)\n");
        return 0;
    }
    const pad::State saved = pad::state();
    const bool savedIcons = UiRenderer::padIcons;
    MenuAssets assets(dataDir, {}, false);  // the fonts
    UiRenderer ui(device, MTLPixelFormatBGRA8Unorm_sRGB, assets);
    const Font& f0 = assets.font(0);
    for (pad::Family fam : {pad::Family::PlayStation, pad::Family::Xbox}) {
        pad::State s;
        s.present = true, s.family = fam, s.forced = true;
        pad::setState(s);
        const pad::Atlas* a = pad::atlas(device);
        const bool ps = fam == pad::Family::PlayStation;
        int cells = 0;
        if (a)
            for (const auto& c : a->cells) cells += c.ok && c.x1 > c.x0 && c.y1 > c.y0 && c.x1 <= a->width && c.y1 <= a->height;
        expect(a && cells == pad::kIconCount && a->missing == 0 && a->pills == (ps ? 2 : 0) && a->symbols == pad::kIconCount - (ps ? 2 : 0),
               std::string(pad::familyName(fam)) + ": every icon in the atlas (" + std::to_string(cells) + " cells, " +
                   std::to_string(a ? a->symbols : 0) + " symbols, " + std::to_string(a ? a->pills : 0) + " pills)");
        if (!a) continue;
        // Glyphs: a prompt character, and FUEL's glyph byte 1 (CROSS) in pad mode, give the atlas glyph of A.
        UiRenderer::padIcons = true;
        const std::string aText = pad::iconText(fuel::input::PadA);
        const uint32_t aCode = (uint32_t(uint8_t(aText[0])) << 16) | (uint32_t(uint8_t(aText[1])) << 8) | uint8_t(aText[2]);
        const Glyph* g = ui.glyphAt(f0, aCode);
        const Glyph* g1 = ui.glyphAt(f0, 1);
        expect(g && g->page == UiRenderer::kPadPage && g1 == g, std::string(pad::familyName(fam)) + ": the prompt A and FUEL's CROSS byte share the atlas glyph");
        UiRenderer::padIcons = false;
        const Glyph* cap = ui.glyphAt(f0, 1);
        expect(cap && cap->page != UiRenderer::kPadPage, "no pad: byte 1 stays SMALL_FONT_PC's Enter key cap");
        if (const Glyph* old = f0.glyph(0x101); g && old) {
            const pad::Atlas::Cell& c = a->cells[fuel::input::PadA];
            // The drawn face height (the symbol's own box in the cell) against FUEL's A icon x 0.92.
            const float faceH = a->ref * g->scale, want = (old->y1 - old->y0) * 0.92f;
            const float centre = g->top + c.centre * g->scale, oldCentre = old->top + 0.5f * (old->y1 - old->y0);
            expect(std::fabs(faceH - want) < 0.02f * want && std::fabs(centre - oldCentre) < 0.5f,
                   std::string(pad::familyName(fam)) + ": face prompt " + std::to_string(faceH) + " texels high (FUEL's icon x 0.92 = " + std::to_string(want) +
                       "), centre " + std::to_string(centre) + " vs " + std::to_string(oldCentre));
        }
        float w = 0;
        TextStyle st;
        st.font = 0, st.height = 22;
        for (int i = 0; i < pad::kIconCount; ++i) w = std::max(w, ui.textWidth(pad::iconText(i), st));
        expect(w > 10 && w < 120, std::string(pad::familyName(fam)) + ": prompts 10..120 canvas units wide at 22 high (widest " + std::to_string(w) + ")");
    }
    pad::setState(saved);
    UiRenderer::padIcons = savedIcons;
    std::printf("pad atlas: %s\n", bad ? "FAILED" : "ok");
    return bad;
}

}  // namespace fuel::ui
