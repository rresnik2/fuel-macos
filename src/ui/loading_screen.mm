#include "ui/loading_screen.h"
#include "ui/help_texts.hpp"  // PK26

#include "ui/ui_renderer.h"

#import <QuartzCore/CAMetalLayer.h>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <filesystem>
#include <fstream>
#include <optional>

namespace fuel::ui {
namespace {

// FUEL's 2x colour convention: 0.5 = the texture / font as is (white) [O 0xa064a8 = (0.5, 0.5, 0.5, 1)].
const Rgba kWhite{1, 1, 1, 1};
const Rgba kBlack{0, 0, 0, 1};
constexpr float kW = 1280, kH = 720;  // the canvas (Menu_CanvasRect 0x49bec0) [port: W x H of the formulas = the 16:9 canvas]

TextStyle smallText(const MenuAssets& a, float scale) {
    TextStyle st;
    st.font = 0;  // SMALL_FONT (SMALL_FONT_PC)
    st.height = scale * a.font(0).refHeight;
    st.fill = kWhite;
    st.outline = kBlack;  // outline on, black, offset 2.0 [port: the renderer's 1-pixel ring]
    return st;
}

// `n` bytes at virtual address `va` of the player's FUEL.exe (PE sections), or nothing.
std::optional<std::vector<uint8_t>> readExe(const std::filesystem::path& exe, uint32_t va, size_t n) {
    std::ifstream f(exe, std::ios::binary);
    if (!f) return std::nullopt;
    const std::vector<uint8_t> b((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
    auto u16 = [&](size_t o) { return o + 2 <= b.size() ? uint32_t(b[o] | b[o + 1] << 8) : 0u; };
    auto u32 = [&](size_t o) { return o + 4 <= b.size() ? uint32_t(b[o] | b[o + 1] << 8 | b[o + 2] << 16 | uint32_t(b[o + 3]) << 24) : 0u; };
    const size_t pe = u32(0x3c);
    if (u32(pe) != 0x00004550) return std::nullopt;
    const uint32_t sections = u16(pe + 6), optSize = u16(pe + 20), imageBase = u32(pe + 24 + 28);
    const uint32_t rva = va - imageBase;
    for (uint32_t k = 0; k < sections; ++k) {
        const size_t sh = pe + 24 + optSize + size_t(k) * 40;
        const uint32_t sva = u32(sh + 12), vsize = std::max(u32(sh + 8), u32(sh + 16)), raw = u32(sh + 20);
        if (rva < sva || rva >= sva + vsize) continue;
        const size_t off = size_t(raw) + (rva - sva);
        const size_t m = std::min(n, b.size() > off ? b.size() - off : 0);
        return std::vector<uint8_t>(b.begin() + std::ptrdiff_t(off), b.begin() + std::ptrdiff_t(off + m));
    }
    return std::nullopt;
}

}  // namespace

// ---- Tips 0x503110 / 0x503200 / 0x503490 ----

void LoadingTips::build(const TextTable& text) {
    ids_.clear();
    for (uint32_t id = 4150;; ++id) {  // TT 4150, 4151, ... until "" or "Empty Text" [O 0x503110]
        const std::string* s = text.get(id);
        if (!s || s->empty() || s->find("Empty Text") != std::string::npos) break;
        ids_.push_back(id);
    }
    ages_.assign(ids_.size(), float(ids_.size()) * 5.0f);  // ages start at n x 5 s
    current_ = 0, timer_ = 5.0f;
}

void LoadingTips::pick() {
    // 0x503200: a random tip whose age >= (n - 1) x 5 s (up to 1000 tries), age 0 [I: residual].
    const size_t n = ids_.size();
    if (n == 0) return;
    std::uniform_int_distribution<size_t> d(0, n - 1);
    for (int tries = 0; tries < 1000; ++tries) {
        const size_t k = d(rng_);
        if (ages_[k] >= float(n - 1) * 5.0f) {
            ages_[k] = 0, current_ = ids_[k];
            return;
        }
    }
}

void LoadingTips::update(float dt) {
    timer_ += dt;
    for (float& a : ages_) a += dt;
    if (timer_ > 5.0f) pick(), timer_ = 0;
}

// ---- Draw helpers ----

void drawLoadingWord(UiRenderer& r, const MenuAssets& a, const std::string& word, double t) {
    // 0x503ce0: w = width("LOADING...") at scale 1; r = w / W; s = 0.75 if r <= 0.175 else min(0.75, 0.175 / r);
    // scale = 0.75 s; right edge at 92.5 % of the canvas, y = top + 0.925 Hc - 20; dots = int(t x 5) & 3.
    if (word.empty()) return;
    TextStyle one = smallText(a, 1.0f);
    const float w = r.textWidth(word + "...", one);
    const float ratio = w / kW;
    const float s = ratio <= 0.175f ? 0.75f : std::min(0.75f, 0.175f / ratio);
    const float scale = 0.75f * s;
    const float x = kW - 0.075f * kW - w * scale, y = 0.925f * kH - 20.0f;
    std::string text = word;
    const int dots = int(std::floor(t * 5.0)) & 3;
    text.append(size_t(dots), '.');
    TextStyle st = smallText(a, scale);
    r.text(text, x, y, w * scale + 2, st);
}

void drawLoadingScreen(UiRenderer& r, const MenuAssets& a, const BitmapRef& picture, const std::string& tip, const std::string& word, double t) {
    // 0x502b70: the black letterbox over the whole viewport (when the canvas has bars), LOADING_BITMAP over the canvas
    // unmodulated, the tip, "LOADING...". The title text from the level's "LOADING" record [U which levels carry one] is not drawn.
    r.rect(r.viewLeft(), r.viewTop(), r.viewWidth(), r.viewHeight(), kBlack);
    if (picture) r.image(picture, 0, 0, kW, kH, kWhite);
    if (!tip.empty()) {
        // 0x5032e0: box x = left + 0.09 W .. left + 0.65 W, y = 0.825 H .. 0.925 H, scale 0.65, colour (0.82, 0.85, 0.85)
        // (2x: white), left, wrapped, outline on, SMALL_FONT; shrink to fit the box height.
        TextStyle st = smallText(a, 0.65f);
        r.richText(tip, 0.09f * kW, 0.825f * kH, 0.65f * kW, 0.925f * kH, 0, st, true, true);
    }
    drawLoadingWord(r, a, word, t);
}

void drawSpinner(UiRenderer& r, float cx, float cy, float rW, float angle) {
    // 0x4feed0: 16 wedges pi/8 apart, each 0.2356 rad wide, outer radius rW x W, inner 0.7 of it, round on screen;
    // the head at the spinner angle quantised to pi/8, wedge i (from the head) brightness 1 - i/15 (comet tail),
    // untextured white x brightness [I: the tail trails behind the turning direction].
    constexpr float kStep = 0.3926991f, kWidth = 0.2356f;
    const float ro = rW * kW, ri = 0.7f * ro;
    const float x0 = cx * kW, y0 = cy * kH;
    const float head = std::floor(angle / kStep) * kStep;
    std::vector<UiVertex> v;
    for (int i = 0; i < 16; ++i) {
        const float b = 1.0f - float(i) / 15.0f;
        const Rgba c{b, b, b, 1};
        const float a0 = head - float(i) * kStep, a1 = a0 + kWidth;
        constexpr int kSeg = 3;
        for (int k = 0; k < kSeg; ++k) {
            const float u0 = a0 + (a1 - a0) * float(k) / kSeg, u1 = a0 + (a1 - a0) * float(k + 1) / kSeg;
            const UiVertex p0{x0 + ri * std::cos(u0), y0 + ri * std::sin(u0), 0, 0, c}, p1{x0 + ro * std::cos(u0), y0 + ro * std::sin(u0), 0, 0, c};
            const UiVertex p2{x0 + ri * std::cos(u1), y0 + ri * std::sin(u1), 0, 0, c}, p3{x0 + ro * std::cos(u1), y0 + ro * std::sin(u1), 0, 0, c};
            v.insert(v.end(), {p0, p1, p3, p0, p3, p2});
        }
    }
    r.triangles(nullptr, v);
}

id<MTLTexture> loadExeImage(id<MTLDevice> device, const std::string& dataDir, ExeImage which) {
    const bool icon = which == ExeImage::SaveIcon;
    const uint32_t va = icon ? 0x9a0584u : 0x99673cu;  // 0x502136.. / 0x504269
    const int w = icon ? 64 : 256, h = icon ? 64 : 128, bpp = icon ? 4 : 3;
    const auto src = readExe(std::filesystem::path(dataDir) / "XFinalComponent" / "FUEL.exe", va, 64 * 1024);
    if (!src) return nil;
    // Mem::CopyBlock 0x504170: a count byte; bit 7 set = (n - 0x80) copies of one pixel, else n literal pixels;
    // 4-byte pixels are copied as they are (D3D A8R8G8B8 = B, G, R, A in memory); 3-byte ones are R, G, B (stored
    // B, G, R, 0xff), and 0x504250 then clears the alpha of pixels whose R + G + B < 5.
    std::vector<uint8_t> out;
    const size_t want = size_t(w) * size_t(h) * 4;
    out.reserve(want);
    size_t i = 0;
    const std::vector<uint8_t>& s = *src;
    auto pixel = [&](size_t at) {
        if (bpp == 4) {
            out.insert(out.end(), s.begin() + std::ptrdiff_t(at), s.begin() + std::ptrdiff_t(at + 4));
        } else {
            const uint8_t R = s[at], G = s[at + 1], B = s[at + 2];
            out.insert(out.end(), {B, G, R, uint8_t(R + G + B < 5 ? 0 : 0xff)});
        }
    };
    while (out.size() < want && i < s.size()) {
        const uint8_t n = s[i++];
        if (n & 0x80) {
            if (i + size_t(bpp) > s.size()) break;
            for (int k = 0; k < n - 0x80; ++k) pixel(i);
            i += size_t(bpp);
        } else {
            if (i + size_t(n) * size_t(bpp) > s.size()) break;
            for (int k = 0; k < n; ++k) pixel(i), i += size_t(bpp);
        }
    }
    if (out.size() < want) return nil;
    out.resize(want);
    MTLTextureDescriptor* td = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:MTLPixelFormatBGRA8Unorm_sRGB width:NSUInteger(w)
                                                                                 height:NSUInteger(h) mipmapped:NO];
    id<MTLTexture> t = [device newTextureWithDescriptor:td];
    [t replaceRegion:MTLRegionMake2D(0, 0, NSUInteger(w), NSUInteger(h)) mipmapLevel:0 withBytes:out.data() bytesPerRow:NSUInteger(w) * 4];
    return t;
}

void drawSaveIcon(UiRenderer& r, id<MTLTexture> icon, float angle, bool orange) {
    // 0x5035c0: side 0.06 W centred at (0.105 W, 0.075 H + 0.03 W), turned by -2 x the spinner angle; a black copy
    // offset (+2, +2) first, then white (0.5 = as is) or orange (0.747, 0.234, 0) x 2, alpha 0.999.
    if (!icon) return;
    const float side = 0.06f * kW, cx = 0.105f * kW, cy = 0.075f * kH + 0.03f * kW;
    const float deg = -2.0f * angle * 57.29578f;
    r.sprite(icon, cx + 2, cy + 2, side, side, 0.5f, 0.5f, deg, Rgba{0, 0, 0, 0.999f});
    r.sprite(icon, cx, cy, side, side, 0.5f, 0.5f, deg, orange ? Rgba{1.0f, 0.468f, 0, 0.999f} : Rgba{1, 1, 1, 0.999f});
}

// ---- PK26: 0x5035c0 with +0x280 / +0x284 both >= 0 [O]: the centre is (x W, y H); side and colours as above.
void drawSaveIcon(UiRenderer& r, id<MTLTexture> icon, float angle, bool orange, float fx, float fy) {
    if (!icon) return;
    const float side = 0.06f * kW, cx = fx * kW, cy = fy * kH;
    const float deg = -2.0f * angle * 57.29578f;
    r.sprite(icon, cx + 2, cy + 2, side, side, 0.5f, 0.5f, deg, Rgba{0, 0, 0, 0.999f});
    r.sprite(icon, cx, cy, side, side, 0.5f, 0.5f, deg, orange ? Rgba{1.0f, 0.468f, 0, 0.999f} : Rgba{1, 1, 1, 0.999f});
}
// ---- end PK26

void drawBootLoading(UiRenderer& r, const MenuAssets& a, id<MTLTexture> logo, const std::string& word, double t, float angle) {
    // 0x502710 step 1: SplashScreenFx 0x60ae00 W/4 wide at (W/2, 0.4 H) [I: centred, the picture's 2:1 aspect],
    // spinner 0x4feed0(0.5, 0.55, 0.02), "LOADING..." 0x503ce0.
    r.rect(r.viewLeft(), r.viewTop(), r.viewWidth(), r.viewHeight(), kBlack);
    if (logo) r.sprite(logo, 0.5f * kW, 0.4f * kH, 0.25f * kW, 0.125f * kW, 0.5f, 0.5f, 0, kWhite);
    drawSpinner(r, 0.5f, 0.55f, 0.02f, angle);
    drawLoadingWord(r, a, word, t);
}

// ---- Logo stencil (LevelManipulator 0x57e650 / 0x57e0f0, LoadingDraw 0x503860) ----

void LogoTransition::start(float extra) { state_ = 1, p_ = 0, q_ = -1, frames_ = 0, stable_ = 0, extra_ = extra; }

void LogoTransition::update(float dt, bool ready) {
    if (state_ == 0) return;
    frames_ += dt * 60.0f;  // FUEL counts frames (+0x50) [port: 60 per second]
    p_ += dt;
    if (state_ == 1) {
        if (frames_ > 60) state_ = 2;  // 0x3c frames (the queued console lines, CloseBF ...)
    } else if (state_ == 2) {
        // >= 150 frames and >= 0x9c, streaming idle, then 1.0 s stable -> 3; q = PP52 + PP50 + extra (0x57dfc0)
        if (frames_ >= 156 && ready) {
            stable_ += dt;
            if (stable_ > 1.0f) state_ = 3, q_ = 0.1f + 0.6f + extra_;
        } else {
            stable_ = 0;
        }
    } else if (state_ == 3) {
        q_ -= dt;
        if (q_ <= 0) state_ = 0;  // 0x57dff0
    }
}

void LogoTransition::draw(UiRenderer& r, const MenuAssets& a, const std::string& word, double t) const {
    if (state_ == 0) return;
    const BitmapRef pic = a.image("FUEL_POCHOIR_LOAD"), mask = a.image("FUEL_POCHOIR_LOAD_ALPHA");
    if (!pic || !mask) return;
    constexpr float PP50 = 0.6f, PP51 = 50.0f, PP52 = 0.1f;  // pp001.tsc
    float s = 1;
    if (p_ < PP50 + PP52 && p_ < PP50) s = std::min(PP51, (PP50 + 0.01f) / (p_ + 0.01f));
    float alpha = 1;
    if (q_ >= 0 && q_ < PP50 + PP52) {
        alpha = 0;
        if (q_ < PP50) s = std::min(PP51, (PP50 + 0.01f) / (q_ + 0.01f));
    }
    // (the sound gain 0x554200(1 - p / PP50) of the same branches is not ported)
    const float w = kW * s, h = kH * s, x = kW * 0.5f - w * 0.5f, y = kH * 0.5f - h * 0.5f;
    r.image(pic, x, y, w, h, Rgba{1, 1, 1, alpha});
    r.image(mask, x, y, w, h, Rgba{0, 0, 0, 0.999f});
    r.image(mask, x, y, w, h, Rgba{0, 0, 0, 0.999f});
    // 0x503f70: black bars above and below the canvas.
    r.rect(r.viewLeft(), r.viewTop(), r.viewWidth(), -r.viewTop(), kBlack);
    r.rect(r.viewLeft(), kH, r.viewWidth(), r.viewTop() + r.viewHeight() - kH, kBlack);
    if (alpha >= 1) drawLoadingWord(r, a, word, t);
}

// ---- The loading window ----

LoadingWindow::LoadingWindow(const std::string& dataDir, const std::string& language, NSSize size, Mode mode) {
    [NSApplication sharedApplication];
    [NSApp setActivationPolicy:NSApplicationActivationPolicyRegular];
    const NSRect frame = NSMakeRect(0, 0, size.width, size.height);
    window_ = [[NSWindow alloc] initWithContentRect:frame styleMask:NSWindowStyleMaskTitled backing:NSBackingStoreBuffered defer:NO];
    window_.title = @"FUEL";
    window_.releasedWhenClosed = NO;
    window_.backgroundColor = NSColor.blackColor;
    // The plain label stays underneath: it is what shows if the game data or Metal is missing.
    NSTextField* label = [NSTextField labelWithString:@"Loading…"];
    label.font = [NSFont boldSystemFontOfSize:28];
    label.textColor = [NSColor colorWithCalibratedWhite:0.85 alpha:1];
    [label sizeToFit];
    [label setFrameOrigin:NSMakePoint((frame.size.width - label.frame.size.width) / 2, (frame.size.height - label.frame.size.height) / 2)];
    [window_.contentView addSubview:label];
    id<MTLDevice> device = MTLCreateSystemDefaultDevice();
    CAMetalLayer* layer = nil;
    if (device && device.supportsBCTextureCompression) {
        NSView* v = [[NSView alloc] initWithFrame:frame];
        v.wantsLayer = YES;
        layer = [CAMetalLayer layer];
        layer.device = device;
        layer.pixelFormat = MTLPixelFormatBGRA8Unorm_sRGB;
        layer.framebufferOnly = YES;
        layer.opaque = YES;
        const CGFloat k = NSScreen.mainScreen.backingScaleFactor;
        layer.contentsScale = k;
        layer.drawableSize = CGSizeMake(frame.size.width * k, frame.size.height * k);
        v.layer = layer;
        v.hidden = YES;  // shown by the thread's first frame
        [window_.contentView addSubview:v];
    }
    [window_ center];
    [window_ makeKeyAndOrderFront:nil];
    [NSApp activateIgnoringOtherApps:YES];
    [window_ displayIfNeeded];
    if (layer) {
        thread_ = std::thread([this, device, layer, dataDir, language, mode] {
            @autoreleasepool {
                // Everything is read here so the window is up at once; the main thread loads the world meanwhile.
                std::unique_ptr<MenuAssets> assets;
                std::unique_ptr<UiRenderer> ui;
                TextTable text;
                BitmapRef picture;
                id<MTLTexture> logo = nil;
                try {
                    assets = std::make_unique<MenuAssets>(dataDir, mode == World ? std::vector<std::string>{"DATAS\\LWYOMING.DPC"} : std::vector<std::string>{}, false);
                    if (assets->font(0).pages.empty()) {
                        ready_ = 2;
                        return;
                    }
                    ui = std::make_unique<UiRenderer>(device, layer.pixelFormat, *assets);
                    text = loadText(dataDir, language);
                    picture = assets->image("LOADING_BITMAP");
                    if (mode == Boot) logo = loadExeImage(device, dataDir, ExeImage::BootLogo);
                } catch (const std::exception&) {
                    ready_ = 2;
                    return;
                }
                const std::string word = text.get(300) ? *text.get(300) : std::string("LOADING");  // TT 300 "LOADING"
                LoadingTips tips;
                if (mode == World) tips.build(text);
                id<MTLCommandQueue> queue = [device newCommandQueue];
                auto last = std::chrono::steady_clock::now();
                const auto t0 = last;
                float angle = 0;
                bool shown = false;
                while (!stop_) {
                    @autoreleasepool {
                        const auto now = std::chrono::steady_clock::now();
                        const float dt = std::chrono::duration<float>(now - last).count();
                        last = now;
                        const double t = std::chrono::duration<double>(now - t0).count();
                        tips.update(dt);
                        angle = std::fmod(angle + dt, 6.2831855f);  // +0x38 += +0x40 (1.0) x dt
                        id<CAMetalDrawable> d = [layer nextDrawable];
                        if (d) {
                            ui->begin(float(d.texture.width), float(d.texture.height));
                            if (mode == World) {
                                const std::string* tip = tips.current() ? text.get(tips.current()) : nullptr;
                                drawLoadingScreen(*ui, *assets, picture, tip ? helpTexts().expand(*tip) : std::string(), word, t);  // PK26: {STR_X}
                            } else {
                                drawBootLoading(*ui, *assets, logo, word, t, angle);
                            }
                            id<MTLCommandBuffer> cb = [queue commandBuffer];
                            ui->encode(cb, d.texture, true);
                            [cb presentDrawable:d];
                            [cb commit];
                            if (!shown) {
                                [cb waitUntilCompleted];
                                ready_ = 1;
                                shown = true;
                            }
                        }
                    }
                    std::this_thread::sleep_for(std::chrono::milliseconds(33));
                }
            }
        });
    }
    // Let the window server show it before the loading blocks the main thread; the Metal view replaces the label once
    // the thread has drawn its first frame (it keeps drawing on its own after that).
    const auto until = std::chrono::steady_clock::now() + std::chrono::seconds(3);
    for (int i = 0; i < 5 || (layer && ready_ == 0 && std::chrono::steady_clock::now() < until); ++i) {
        NSEvent* e = [NSApp nextEventMatchingMask:NSEventMaskAny untilDate:[NSDate dateWithTimeIntervalSinceNow:0.02] inMode:NSDefaultRunLoopMode dequeue:YES];
        if (e) [NSApp sendEvent:e];
    }
    if (layer && ready_ == 1) {
        window_.contentView.subviews.lastObject.hidden = NO;
        [window_ displayIfNeeded];
        for (int i = 0; i < 3; ++i) {
            NSEvent* e = [NSApp nextEventMatchingMask:NSEventMaskAny untilDate:[NSDate dateWithTimeIntervalSinceNow:0.02] inMode:NSDefaultRunLoopMode dequeue:YES];
            if (e) [NSApp sendEvent:e];
        }
    }
}

LoadingWindow::~LoadingWindow() { close(); }

void LoadingWindow::close() {
    stop_ = true;
    if (thread_.joinable()) thread_.join();
    if (window_) [window_ close];
    window_ = nil;
}

}  // namespace fuel::ui
