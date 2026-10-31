#include "ui/game_menu.h"

#include "ui/ui_renderer.h"
#include "viewer/game_settings.h"

#import <Cocoa/Cocoa.h>
#import <GameController/GameController.h>
#import <ImageIO/ImageIO.h>
#import <MetalKit/MetalKit.h>
#import <QuartzCore/QuartzCore.h>
#import <UniformTypeIdentifiers/UniformTypeIdentifiers.h>

#include <mach-o/dyld.h>
#include <unistd.h>

#include <cstdio>
#include <filesystem>

namespace fs = std::filesystem;

namespace fuel::ui {
namespace {

constexpr MTLPixelFormat kFormat = MTLPixelFormatBGRA8Unorm_sRGB;

bool writePng(id<MTLDevice> device, id<MTLTexture> tex, const std::string& path) {
    const NSUInteger w = tex.width, h = tex.height, row = w * 4;
    id<MTLBuffer> buf = [device newBufferWithLength:row * h options:MTLResourceStorageModeShared];
    id<MTLCommandQueue> q = [device newCommandQueue];
    id<MTLCommandBuffer> cb = [q commandBuffer];
    id<MTLBlitCommandEncoder> blit = [cb blitCommandEncoder];
    [blit copyFromTexture:tex sourceSlice:0 sourceLevel:0 sourceOrigin:MTLOriginMake(0, 0, 0) sourceSize:MTLSizeMake(w, h, 1)
                 toBuffer:buf destinationOffset:0 destinationBytesPerRow:row destinationBytesPerImage:row * h];
    [blit endEncoding];
    [cb commit];
    [cb waitUntilCompleted];
    CGColorSpaceRef cs = CGColorSpaceCreateWithName(kCGColorSpaceSRGB);
    CGDataProviderRef dp = CGDataProviderCreateWithData(nullptr, buf.contents, row * h, nullptr);
    CGImageRef img = CGImageCreate(w, h, 8, 32, row, cs, kCGBitmapByteOrder32Little | kCGImageAlphaNoneSkipFirst, dp, nullptr, false,
                                   kCGRenderingIntentDefault);
    NSURL* url = [NSURL fileURLWithPath:@(path.c_str())];
    CGImageDestinationRef dst = CGImageDestinationCreateWithURL((__bridge CFURLRef)url, (__bridge CFStringRef)UTTypePNG.identifier, 1, nullptr);
    bool ok = dst != nullptr;
    if (dst) {
        CGImageDestinationAddImage(dst, img, nullptr);
        ok = CGImageDestinationFinalize(dst);
        CFRelease(dst);
    }
    CGImageRelease(img);
    CGDataProviderRelease(dp);
    CGColorSpaceRelease(cs);
    return ok;
}

// The menus' backdrop when no game world is behind them: dark, like the game's
// loading screens (the MENU level's own scene, WORLD/MENU.DPC, is not drawn yet).
void backdrop(UiRenderer& r) {
    r.rect(-400, -400, 2080, 1520, Rgba{0.05f, 0.045f, 0.04f, 1});
}

std::string vehicleDpc(const std::string& dataDir, const Menu& menu, const std::string& vehicle) {
    std::string cls;
    for (const VehicleInfo& v : menu.vehicles())
        if (v.name == vehicle) cls = v.family;
    // The class archives (QUAD, BIKE, BUGGY, MUSCLE, SUV, TRUCK); bonus vehicles (BONUS_*,
    // FLOTTE_*) are in BONUS1.DPC (checked with fuelscene).
    if (vehicle.rfind("BONUS_", 0) == 0 || vehicle.rfind("FLOTTE_", 0) == 0) cls = "BONUS1";
    const std::string rel = "CommonComponent/DATAS/" + cls + ".DPC";
    if (!cls.empty() && fs::exists(fs::path(dataDir) / rel)) return rel;
    return "CommonComponent/DATAS/QUAD.DPC";
}

[[noreturn]] void execArgs(const std::string& self, const std::vector<std::string>& args) {
    std::vector<char*> argv;
    argv.push_back(const_cast<char*>(self.c_str()));
    for (const std::string& a : args) argv.push_back(const_cast<char*>(a.c_str()));
    argv.push_back(nullptr);
    // The session's command, for replaying a menu choice by hand.
    std::string line = "fuelview: exec";
    for (const std::string& a : args) line += " '" + a + "'";
    std::fprintf(stderr, "%s\n", line.c_str());
    if (const char* home = std::getenv("HOME"))
        if (FILE* f = std::fopen((std::string(home) + "/Library/Application Support/FUEL Port/sessions.log").c_str(), "a")) {
            std::fprintf(f, "%s\n", line.c_str());
            std::fclose(f);
        }
    std::fflush(stdout);
    std::fflush(stderr);
    execv(self.c_str(), argv.data());
    std::perror("fuelview: exec");
    std::exit(1);
}

}  // namespace

bool menuKeyFor(unsigned short code, MenuKey* k) {
    switch (code) {
        case 126: *k = MenuKey::Up; return true;
        case 125: *k = MenuKey::Down; return true;
        case 123: *k = MenuKey::Left; return true;
        case 124: *k = MenuKey::Right; return true;
        case 36:
        case 76:
        case 49: *k = MenuKey::Accept; return true;  // Return, Enter, Space
        case 53:
        case 51: *k = MenuKey::Back; return true;  // Escape, Backspace
        case 12:
        case 116: *k = MenuKey::TabLeft; return true;  // Q, Page Up
        case 14:
        case 121: *k = MenuKey::TabRight; return true;  // E, Page Down
        case 13: *k = MenuKey::Up; return true;      // W
        case 1: *k = MenuKey::Down; return true;     // S
        case 0: *k = MenuKey::Left; return true;     // A
        case 2: *k = MenuKey::Right; return true;    // D
        default: return false;
    }
}

std::vector<MenuKey> gamepadMenuKeys(unsigned* last) {
    std::vector<MenuKey> out;
    GCExtendedGamepad* pad = GCController.controllers.firstObject.extendedGamepad;
    if (!pad) return out;
    const float sx = pad.leftThumbstick.xAxis.value, sy = pad.leftThumbstick.yAxis.value;
    const bool down[] = {pad.dpad.up.pressed || sy > 0.6f, pad.dpad.down.pressed || sy < -0.6f, pad.dpad.left.pressed || sx < -0.6f,
                         pad.dpad.right.pressed || sx > 0.6f, pad.buttonA.pressed, pad.buttonB.pressed || pad.buttonMenu.pressed,
                         pad.leftShoulder.pressed, pad.rightShoulder.pressed};
    const MenuKey keys[] = {MenuKey::Up, MenuKey::Down, MenuKey::Left, MenuKey::Right, MenuKey::Accept, MenuKey::Back, MenuKey::TabLeft, MenuKey::TabRight};
    unsigned now = 0;
    for (int i = 0; i < 8; ++i) {
        if (down[i]) now |= 1u << i;
        if (down[i] && !(*last & (1u << i))) out.push_back(keys[i]);
    }
    *last = now;
    return out;
}

void execSession(const std::string& self, const std::string& dataDir, const MenuRequest& r, const Menu& menu) {
    std::vector<std::string> args = {"--settings", "--data", dataDir, "--in-game", "--assemble", r.vehicle};
    if (r.kind == MenuRequest::StartEvent) {
        args.insert(args.end(), {"--race", r.event});
    } else {
        args.insert(args.end(), {"--hub", r.hub, "--drive"});
    }
    args.push_back(vehicleDpc(dataDir, menu, r.vehicle));
    std::printf("menu: starting %s\n", r.kind == MenuRequest::StartEvent ? ("race " + r.event).c_str() : ("free ride in " + r.hub).c_str());
    execArgs(self, args);
}

void execMenu(const std::string& self, const std::string& dataDir, const std::string& page) {
    std::vector<std::string> args = {"--game", "--data", dataDir};
    if (!page.empty()) args.insert(args.end(), {"--menu-page", page});
    execArgs(self, args);
}

int runMenuShot(const std::string& dataDir, const Settings& s, const std::string& page, const std::string& out, int w, int h) {
    @autoreleasepool {
        id<MTLDevice> device = MTLCreateSystemDefaultDevice();
        Menu menu(dataDir, s, MenuMode::Main);
        const bool imageOnly = page.rfind("image:", 0) == 0;  // debugging: one picture on a grey ground
        if (!imageOnly && !menu.showPage(page)) {
            std::fprintf(stderr, "menu-shot: no page '%s'\n", page.c_str());
            return 2;
        }
        UiRenderer ui(device, kFormat, menu.assets());
        MTLTextureDescriptor* td = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:kFormat width:NSUInteger(w) height:NSUInteger(h) mipmapped:NO];
        td.usage = MTLTextureUsageRenderTarget | MTLTextureUsageShaderRead;
        td.storageMode = MTLStorageModePrivate;
        id<MTLTexture> target = [device newTextureWithDescriptor:td];
        ui.begin(float(w), float(h));
        backdrop(ui);
        const std::string p = page;
        if (imageOnly) {
            ui.rect(0, 0, 1280, 720, Rgba{0.5f, 0.5f, 0.5f, 1});
            const BitmapRef b = menu.assets().image(p.substr(6));
            std::printf("menu-shot: %s: %ux%u\n", p.c_str() + 6, b.width, b.height);
            const float k = std::min(1200.0f / float(std::max(1u, b.width)), 680.0f / float(std::max(1u, b.height)));
            ui.image(b, 40, 20, float(b.width) * k, float(b.height) * k, Rgba{1, 1, 1, 1});
        } else if (p == "compteur" || p == "classement" || p == "boussole" || p == "hud") {
            HudState hud;
            hud.speedKmh = 87, hud.odometerKm = 12.3f, hud.race = true, hud.position = 2, hud.racers = 4;
            hud.hubName = menu.hubs().empty() ? "" : menu.textOf(menu.hubs()[0].nameText);
            hud.standings = {{"Rival 1", 0, false}, {"Player", 1.2f, true}, {"Rival 2", 3.4f, false}, {"Rival 3", 7.9f, false}};
            menu.drawHud(ui, hud);
        } else {
            menu.update(0);
            menu.draw(ui);
        }
        id<MTLCommandQueue> q = [device newCommandQueue];
        id<MTLCommandBuffer> cb = [q commandBuffer];
        ui.encode(cb, target, true);
        [cb commit];
        [cb waitUntilCompleted];
        // Private textures cannot be blitted to a shared buffer directly on all GPUs: copy via a shared one.
        MTLTextureDescriptor* sd = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:kFormat width:NSUInteger(w) height:NSUInteger(h) mipmapped:NO];
        sd.storageMode = MTLStorageModeShared;
        id<MTLTexture> shared = [device newTextureWithDescriptor:sd];
        cb = [q commandBuffer];
        id<MTLBlitCommandEncoder> blit = [cb blitCommandEncoder];
        [blit copyFromTexture:target toTexture:shared];
        [blit endEncoding];
        [cb commit];
        [cb waitUntilCompleted];
        if (!writePng(device, shared, out)) {
            std::fprintf(stderr, "menu-shot: cannot write %s\n", out.c_str());
            return 1;
        }
        std::printf("menu-shot: %s -> %s (%dx%d)\n", page.c_str(), out.c_str(), w, h);
        return 0;
    }
}

}  // namespace fuel::ui

using namespace fuel::ui;

@interface FuelMenuView : MTKView <MTKViewDelegate>
@end

@implementation FuelMenuView {
    std::unique_ptr<Menu> _menu;
    std::unique_ptr<UiRenderer> _ui;
    id<MTLCommandQueue> _queue;
    std::string _self, _dataDir;
    double _last;
    unsigned _pad;
}

- (instancetype)initWithFrame:(NSRect)frame device:(id<MTLDevice>)device menu:(std::unique_ptr<Menu>)menu self:(std::string)selfPath data:(std::string)dataDir {
    self = [super initWithFrame:frame device:device];
    if (self) {
        _menu = std::move(menu);
        _ui = std::make_unique<UiRenderer>(device, kFormat, _menu->assets());
        _queue = [device newCommandQueue];
        _self = selfPath, _dataDir = dataDir;
        self.colorPixelFormat = kFormat;
        self.delegate = self;
        self.preferredFramesPerSecond = 60;
        NSTrackingArea* ta = [[NSTrackingArea alloc] initWithRect:NSZeroRect
                                                          options:NSTrackingMouseMoved | NSTrackingActiveInKeyWindow | NSTrackingInVisibleRect
                                                            owner:self
                                                         userInfo:nil];
        [self addTrackingArea:ta];
    }
    return self;
}

- (BOOL)acceptsFirstResponder { return YES; }

- (void)keyDown:(NSEvent*)e {
    MenuKey k;
    if (menuKeyFor(e.keyCode, &k)) _menu->key(k);
}

- (void)pointerAt:(NSEvent*)e click:(bool)click {
    const NSPoint p = [self convertPoint:e.locationInWindow fromView:nil];
    const CGSize ds = self.drawableSize;
    const float kx = float(ds.width / self.bounds.size.width), ky = float(ds.height / self.bounds.size.height);
    _ui->begin(float(ds.width), float(ds.height));
    const float px = float(p.x) * kx, py = float(self.bounds.size.height - p.y) * ky;
    _menu->pointer((px - _ui->sx(0)) / _ui->canvasScale(), (py - _ui->sy(0)) / (_ui->sy(720) - _ui->sy(0)) * 720.0f, click);
}
- (void)mouseMoved:(NSEvent*)e { [self pointerAt:e click:false]; }
- (void)mouseDown:(NSEvent*)e { [self pointerAt:e click:true]; }

- (void)mtkView:(MTKView*)view drawableSizeWillChange:(CGSize)size {}

- (void)drawInMTKView:(MTKView*)view {
    const double now = CACurrentMediaTime();
    const float dt = _last > 0 ? float(std::min(0.1, now - _last)) : 0.0f;
    _last = now;
    for (MenuKey k : gamepadMenuKeys(&_pad)) _menu->key(k);
    _menu->update(dt);
    const MenuRequest r = _menu->take();
    if (r.kind == MenuRequest::QuitGame) {
        [NSApp terminate:nil];
        return;
    }
    if (r.kind == MenuRequest::StartEvent || r.kind == MenuRequest::FreeRide) execSession(_self, _dataDir, r, *_menu);
    if (r.kind == MenuRequest::QuitToTitle) _menu->setMode(MenuMode::Title);
    id<CAMetalDrawable> drawable = view.currentDrawable;
    if (!drawable) return;
    const CGSize ds = view.drawableSize;
    _ui->begin(float(ds.width), float(ds.height));
    backdrop(*_ui);
    _menu->draw(*_ui);
    id<MTLCommandBuffer> cb = [_queue commandBuffer];
    _ui->encode(cb, drawable.texture, true);
    [cb presentDrawable:drawable];
    [cb commit];
}
@end

@interface FuelMenuAppDelegate : NSObject <NSApplicationDelegate>
@end
@implementation FuelMenuAppDelegate
- (BOOL)applicationShouldTerminateAfterLastWindowClosed:(NSApplication*)app { return YES; }
@end

namespace fuel::ui {

int runGameMenu(const std::string& selfPath, const std::string& dataDir, const Settings& s, const std::string& startPage) {
    @autoreleasepool {
        std::string exe = selfPath;
        char buf[4096];
        uint32_t size = sizeof buf;
        if (_NSGetExecutablePath(buf, &size) == 0) exe = fs::weakly_canonical(buf).string();
        id<MTLDevice> device = MTLCreateSystemDefaultDevice();
        auto menu = std::make_unique<Menu>(dataDir, s, MenuMode::Title);
        if (!menu->scripts().problems.empty() && menu->scripts().pages.empty()) {
            std::fprintf(stderr, "fuelview: no menu pages under %s\n", dataDir.c_str());
            return 1;
        }
        if (!startPage.empty()) menu->showPage(startPage);
        [NSApplication sharedApplication];
        [NSApp setActivationPolicy:NSApplicationActivationPolicyRegular];
        FuelMenuAppDelegate* delegate = [FuelMenuAppDelegate new];
        NSApp.delegate = delegate;
        const NSSize sz = gameWindowSize(s);
        NSWindow* window = [[NSWindow alloc] initWithContentRect:NSMakeRect(0, 0, sz.width, sz.height)
                                                       styleMask:NSWindowStyleMaskTitled | NSWindowStyleMaskClosable | NSWindowStyleMaskResizable |
                                                                 NSWindowStyleMaskMiniaturizable
                                                         backing:NSBackingStoreBuffered
                                                           defer:NO];
        window.title = @"FUEL";
        FuelMenuView* view = [[FuelMenuView alloc] initWithFrame:window.contentView.bounds device:device menu:std::move(menu) self:exe data:dataDir];
        window.contentView = view;
        [window center];
        [window makeKeyAndOrderFront:nil];
        [window makeFirstResponder:view];
        applyWindowSettings(s, window, view);
        [NSApp activateIgnoringOtherApps:YES];
        [NSApp run];
    }
    return 0;
}

GameOverlay::GameOverlay(id<MTLDevice> device, MTLPixelFormat format, const std::string& self, const std::string& dataDir, const Settings& s,
                         const std::string& event, const std::string& hub, const std::string& vehicle)
    : self_(self), dataDir_(dataDir) {
    char buf[4096];
    uint32_t size = sizeof buf;
    if (_NSGetExecutablePath(buf, &size) == 0) self_ = fs::weakly_canonical(buf).string();
    menu_ = std::make_unique<Menu>(dataDir, s, MenuMode::Pause);
    menu_->setSession(event, hub, vehicle);
    ui_ = std::make_unique<UiRenderer>(device, format, menu_->assets());
}

GameOverlay::~GameOverlay() = default;

void GameOverlay::openPause() {
    menu_->setMode(MenuMode::Pause);
    open_ = true;
}

void GameOverlay::openResults(const std::vector<ResultRow>& rows) {
    menu_->setResults(rows);
    menu_->setMode(MenuMode::Results);
    open_ = true;
}

bool GameOverlay::keyDown(unsigned short code) {
    if (!open_) {
        if (code == 53) return openPause(), true;  // Escape opens the menu
        return false;
    }
    MenuKey k;
    if (menuKeyFor(code, &k)) menu_->key(k);
    return true;
}

void GameOverlay::pointer(float x, float y, bool click, float viewW, float viewH) {
    if (!open_) return;
    ui_->begin(viewW, viewH);
    menu_->pointer((x - ui_->sx(0)) / ui_->canvasScale(), (y - ui_->sy(0)) / (ui_->sy(720) - ui_->sy(0)) * 720.0f, click);
}

void GameOverlay::pollGamepad() {
    const std::vector<MenuKey> keys = gamepadMenuKeys(&padButtons_);
    GCExtendedGamepad* pad = GCController.controllers.firstObject.extendedGamepad;
    if (!open_) {
        if (pad && pad.buttonMenu.pressed && !(padButtons_ & 0x100)) openPause();
        padButtons_ = (padButtons_ & 0xff) | (pad && pad.buttonMenu.pressed ? 0x100 : 0);
        return;
    }
    for (MenuKey k : keys) menu_->key(k);
}

GameOverlay::Action GameOverlay::update(float dt) {
    menu_->update(dt);
    const MenuRequest r = menu_->take();
    switch (r.kind) {
        case MenuRequest::Resume: open_ = false; return Resume;
        case MenuRequest::Restart: open_ = false; return Restart;
        case MenuRequest::StartEvent:
        case MenuRequest::FreeRide: execSession(self_, dataDir_, r, *menu_);
        case MenuRequest::QuitToTitle: execMenu(self_, dataDir_);
        case MenuRequest::QuitGame: [NSApp terminate:nil]; return None;
        default: return None;
    }
}

void GameOverlay::draw(id<MTLCommandBuffer> cb, id<MTLTexture> target, const HudState& hud) {
    ui_->begin(float(target.width), float(target.height));
    if (open_) {
        ui_->rect(-400, -400, 2080, 1520, Rgba{0, 0, 0, 0.45f});  // the world dims behind the menu [port]
        menu_->draw(*ui_);
    } else {
        menu_->drawHud(*ui_, hud);
    }
    ui_->encode(cb, target, false);
}

}  // namespace fuel::ui
