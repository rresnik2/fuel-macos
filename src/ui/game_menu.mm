#include "core/custom_vehicles.hpp"  // CUSTOM VEHICLES
#include "core/upgrades.hpp"  // [upgrades U2]
#include "ui/game_menu.h"

#include <algorithm>

#include "audio/menu_audio.h"
#include "ui/boot_screens.h"  // PK26
#include "ui/help_texts.hpp"   // PK26
#include "ui/loading_screen.h"
#include "ui/pad_glyphs.h"  // pad-ux
#include "ui/ui_renderer.h"
#include "viewer/game_settings.h"
#include "viewer/garage_preview.h"  // PK20
#include "viewer/pad_haptics.h"

#import <Cocoa/Cocoa.h>
#import <GameController/GameController.h>
#import <ImageIO/ImageIO.h>
#import <MetalKit/MetalKit.h>
#import <QuartzCore/QuartzCore.h>
#import <UniformTypeIdentifiers/UniformTypeIdentifiers.h>

#include <mach-o/dyld.h>
#include <unistd.h>

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>   // [upgrades U2]
#include <strings.h>  // [upgrades U2] strcasecmp
#include <sys/stat.h>

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

std::string vehicleDpc(const std::string& dataDir, const Menu& menu, const std::string& vehicle) {
    if (const fuel::custom::Vehicle* cv = fuel::custom::find(vehicle)) return cv->dpc;  // CUSTOM VEHICLES: absolute path
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

// ---- PK20: the 3-D garage preview of a menu host (made when a garage page first queues calls into it)
void runGaragePreview(std::unique_ptr<fuel::GaragePreview>& g, id<MTLDevice> device, const std::string& dataDir, Menu& menu, float dt, bool visible) {
    menu.garage3DUpdate(dt, visible);
    const auto cmds = menu.takeGarage3D();
    if (!g && !cmds.empty()) {
        const Menu* m = &menu;
        g = std::make_unique<fuel::GaragePreview>(device, dataDir, [dataDir, m](const std::string& label) { return vehicleDpc(dataDir, *m, label); });
    }
    if (g) g->run(cmds);
}
// ---- PK20 end

// [port] remap: menuKeyFor (the fixed keys: arrows / W A S D, Return / Enter / Space, Escape / Backspace, Q / E / Page Up /
// Down, R, F, G, I J K L, = / -), releasePadDirections and gamepadMenuKeys (d-pad / stick, A, B / Menu, LB / RB, X) are
// replaced by the bindings (ui/menu_input.*): the same keys and buttons are the menu actions' defaults.

std::vector<std::string> sessionArgs(const std::string& dataDir, const MenuRequest& r, const Menu& menu) {
    std::vector<std::string> args = {"--settings", "--data", dataDir, "--in-game", "--assemble", r.vehicle};
    if (r.kind == MenuRequest::StartEvent) {
        args.insert(args.end(), {"--race", r.event});
        // [upgrades U4] the opponents' stages come from the race's hub tier (an edited race: the session finds the hub under its start);
        // a challenge that hands over its vehicle runs the player stock (U2), so its opponents too: -1
        for (const Event& e : menu.events())
            if (strcasecmp(e.id.c_str(), r.event.c_str()) == 0) {
                const bool handedOver = e.kind == Event::Challenge && !e.vehicle.empty() && strcasecmp(e.vehicle.c_str(), r.vehicle.c_str()) == 0;
                args.insert(args.end(), {"--ai-upgrade-tier", std::to_string(e.hub.empty() || handedOver ? -1 : e.tier)});
                // cr5: the race's AI level (race manager +0x3e8): TTMN "<mission>" <index> <level> -> RestartMission; the career page
                // 0x47eee0 passes the medal row +0x88 (0 gold, 1 silver, 2 bronze), the challenge page 0x47efb0 passes 0.
                args.insert(args.end(), {"--ai-level", std::to_string(e.kind == Event::Career ? std::clamp(menu.raceMedalRow(), 0, 2) : 0)});
            }
    } else {
        args.insert(args.end(), {"--hub", r.hub, "--drive"});
        if (r.at) args.insert(args.end(), {"--at", std::to_string(r.x) + "," + std::to_string(r.z)});  // P78: KillMission keeps the place
        if (r.noArrival) args.push_back("--no-hub-arrival");  // PK03 gap: the race tab's free ride runs no StartHubHUD
        if (r.careerBoot) args.push_back("--career-boot");     // PK26: Dialog_Loading 0x58fc20 runs the boot order
    }
    // P66: the garage's choices: vehicle record +0xc (texture livery, 0-based; fuelview --livery is 1-based) and +0xd
    // (palette colour), and the rider (game +0x33d0..+0x33ec as 0x42b420 resolves them).
    if (const Career* c = menu.career()) {
        args.insert(args.end(), {"--livery", std::to_string(c->vehicleLivery(r.vehicle) + 1), "--color", std::to_string(c->vehicleColor(r.vehicle))});
        const PilotLook p = c->pilotLook();
        std::string pilot;
        for (int k = 0; k < 8; ++k) pilot += (k ? "," : "") + std::to_string(p.at(k));
        args.insert(args.end(), {"--pilot", pilot});
        args.insert(args.end(), {"--outfit", std::to_string(c->riderOutfit()) + "," + (c->riderHelmet() ? "1" : "0")});  // PHYSIQUE
        // [DEVIATION upgrades U2] the driven vehicle's equipped stages, e,s,t,w,np,nt (upgrades-plan.md 7): career races,
        // challenges with the player's own vehicle and free ride. A challenge that hands over its vehicle
        // (AddMissionNeededVehicle, Menu::startForced) and `upgrades = off` run stock; the career keeps the stages.
        bool handedOver = false;
        if (r.kind == MenuRequest::StartEvent)
            for (const Event& e : menu.events())
                if (strcasecmp(e.id.c_str(), r.event.c_str()) == 0 && e.kind == Event::Challenge && !e.vehicle.empty() &&
                    strcasecmp(e.vehicle.c_str(), r.vehicle.c_str()) == 0)
                    handedOver = true;
        args.insert(args.end(), {"--upgrades", fuel::upg::formatStages(c->sessionUpgrades(r.vehicle, menu.portOptions().upgrades, handedOver))});
    }
    args.push_back(vehicleDpc(dataDir, menu, r.vehicle));
    return args;
}

void execSession(const std::string& self, const std::string& dataDir, const MenuRequest& r, const Menu& menu) {
    const std::vector<std::string> args = sessionArgs(dataDir, r, menu);
    std::printf("menu: starting %s\n", r.kind == MenuRequest::StartEvent ? ("race " + r.event).c_str() : ("free ride in " + r.hub).c_str());
    execArgs(self, args);
}

// ---- [upgrades U2] fuelview --upgrade-cmd (game_menu.h): a dev tool beside the garage's UPGRADES page (U3, garage_upgrades.mm).
int runUpgradeCommand(const std::string& dataDir, const Settings& settings, const std::string& spec) {
    Career c(loadCareerRules(dataDir));
    const std::string path = Career::careerPath();
    const int rd = c.read(path);
    if (rd == 7) c.newGame();  // no save yet: a new career (as the menus start one)
    else if (rd != 0) return std::fprintf(stderr, "upgrade-cmd: %s does not load (code %d)\n", path.c_str(), rd), 2;
    std::printf("upgrade-cmd: %s: %u fuel, %d stars, upgrades %s\n", path.c_str(), c.state().fuel, c.stars(),
                PortOptions::from(settings.all).upgrades ? "on" : "off");
    auto label = [&](const std::string& v) {
        const int i = c.rules().vehicleIndex(v);
        return i < 0 ? v : c.rules().vehicles[size_t(i)].label;
    };
    auto show = [&](const std::string& v) {
        const upg::Stages o = c.upgradesOwned(v), e = c.upgradesEquipped(v);
        std::printf("  %-22s level %d  owned %s  equipped %s  spent %u\n", label(v).c_str(), c.vehicleLevel(v), upg::formatStages(o).c_str(),
                    upg::formatStages(e).c_str(), c.upgradesSpent(v));
        for (int k = 0; k < upg::kCategories; ++k)
            if (o[k] < upg::kMaxStage)
                std::printf("      %-11s stage %d: %d fuel (%s)\n", upg::categoryName(k), o[k] + 1, c.upgradePrice(v, k, o[k] + 1),
                            c.upgradeLock(v, k, o[k] + 1) == Career::UpgradeResult::Bought ? "for sale" : Career::upgradeResultName(c.upgradeLock(v, k, o[k] + 1)));
    };
    int bad = 0;
    bool dirty = false;
    std::stringstream all(spec);
    for (std::string line; std::getline(all, line, ';');) {
        std::istringstream in(line);
        std::vector<std::string> w;
        for (std::string t; in >> t;) w.push_back(t);
        if (w.empty()) continue;
        const std::string op = w[0];
        if (op == "list") {
            for (size_t i = 0; i < c.rules().vehicles.size(); ++i) {
                const std::string& v = c.rules().vehicles[i].label;
                if (w.size() > 1 ? strcasecmp(v.c_str(), w[1].c_str()) == 0 : (c.vehicleOwned(v) && c.upgradable(v))) show(v);
            }
        } else if ((op == "buy" || op == "equip") && w.size() == 4) {
            const int cat = upg::categoryFromKey(w[2]);
            const int stage = std::atoi(w[3].c_str());
            if (cat < 0 || c.rules().vehicleIndex(w[1]) < 0) {
                std::printf("  %s: unknown %s\n", line.c_str(), cat < 0 ? "category" : "vehicle");
                ++bad;
                continue;
            }
            if (op == "buy") {
                const uint32_t before = c.state().fuel;
                const int price = c.upgradePrice(w[1], cat, stage);
                const Career::UpgradeResult r = c.buyUpgrade(w[1], cat, stage);
                std::printf("  buy %s %s stage %d (%d fuel): %s, fuel %u -> %u\n", label(w[1]).c_str(), upg::categoryName(cat), stage, price,
                            Career::upgradeResultName(r), before, c.state().fuel);
                if (r == Career::UpgradeResult::Bought) dirty = true;
                else ++bad;
            } else {
                const bool ok = c.equipUpgrade(w[1], cat, stage);
                std::printf("  equip %s %s stage %d: %s\n", label(w[1]).c_str(), upg::categoryName(cat), stage, ok ? "equipped" : "not owned");
                dirty |= ok, bad += !ok;
            }
            show(w[1]);
        } else if (op == "session" && (w.size() == 2 || w.size() == 3)) {
            // The menu's own sessionArgs on a Menu over the same career.sav (it loads it as the game's menu does).
            Menu menu(dataDir, settings, MenuMode::Main);
            if (dirty && menu.career()) menu.career()->state() = c.state();  // what this command changed, not yet saved
            MenuRequest r;
            r.vehicle = label(w[1]);
            if (w.size() == 3) {
                r.kind = MenuRequest::StartEvent, r.event = w[2];
                for (const Event& e : menu.events())
                    if (strcasecmp(e.id.c_str(), w[2].c_str()) == 0) r.event = e.id;
            } else {
                r.kind = MenuRequest::FreeRide, r.hub = "YELLOWSTONE";
            }
            std::string out;
            for (const std::string& a : sessionArgs(dataDir, r, menu)) out += " " + a;
            std::printf("  session:%s\n", out.c_str());
        } else {
            std::printf("  %s: unknown operation (list | buy V C S | equip V C S | session V [EVENT])\n", line.c_str());
            ++bad;
        }
    }
    if (dirty) {
        const int w = c.write(path);
        std::printf("upgrade-cmd: saved %s (%s)\n", path.c_str(), w == 0 ? "ok" : ("code " + std::to_string(w)).c_str());
        bad += w != 0;
    }
    return bad;
}
// ---- [upgrades U2] end

void execMenu(const std::string& self, const std::string& dataDir, const std::string& page) {
    std::vector<std::string> args = {"--game", "--data", dataDir};
    if (!page.empty()) args.insert(args.end(), {"--menu-page", page});
    execArgs(self, args);
}

int runMenuShot(const std::string& dataDir, const Settings& s, const std::string& page, const std::string& out, int w, int h) {
    @autoreleasepool {
        id<MTLDevice> device = MTLCreateSystemDefaultDevice();
        Menu menu(dataDir, s, MenuMode::Main);
        // ---- PK26: the {STR_*} labels as PaddleChecker builds them: key names, or pad glyphs with FUEL_SHOT_PAD=1.
        // pad-ux [port]: FUEL_SHOT_PAD = 1 (a pad of pad_glyphs' family, auto = Xbox), playstation / ps, xbox.
        const pad::State shotState = pad::shotState(pad::Style(menu.portOptions().padGlyphs));
        pad::paddleCheck(shotState, keyboardLabels(dataDir, s.all), padLabels(dataDir, s.all));
        const bool shotPad = shotState.present;
        // LEGAL[:t], PRESS[:t], TRC[:t]: the boot dialogs as after t seconds open (ui/boot_screens.mm).
        auto bootShot = [&](const std::string& pg) -> int {
            const size_t c = pg.find(':');
            const std::string n = pg.substr(0, c);
            if (n != "LEGAL" && n != "PRESS" && n != "TRC") return -1;
            return n == "LEGAL" ? BootScreens::LegalText : n == "PRESS" ? BootScreens::PressStart : BootScreens::TRCMessage;
        };
        const int bootDialog = bootShot(page);
        // ---- end PK26
        const bool imageOnly = page.rfind("image:", 0) == 0;  // debugging: one picture on a grey ground
        // P78: LOADING[:t] (the world's loading screen, tip and dots at time t), BOOT[:t] (the boot loading draw),
        // LOGO:p:q (the logo stencil with those timers; -1 = unset), SAVEICON[:angle] (over the HUD sample).
        const bool loadingPage = page.rfind("LOADING", 0) == 0 || page.rfind("BOOT", 0) == 0 || page.rfind("LOGO:", 0) == 0 ||
                                 page.rfind("SAVEICON", 0) == 0 || page.rfind("RICHTEXT:", 0) == 0;  // RICHTEXT:<tt>: the rich-text engine in a box
        const bool hudFree = page.rfind("hudfree", 0) == 0;  // PK19
        if (!imageOnly && !loadingPage && !hudFree && bootDialog < 0 && page != "GLYPHS" && page.rfind("PADGLYPHS", 0) != 0 && !menu.showPage(page)) {
            std::fprintf(stderr, "menu-shot: no page '%s'\n", page.c_str());
            return 2;
        }
        if (menu.optionsSerial() != 0) {  // [port] remap: a shot that rebinds: the labels from the page's bindings (0x4e2250(1))
            const Settings ls = menu.liveSettings();
            helpTexts().rebuild(shotPad, keyboardLabels(dataDir, ls.all), shotPad ? padLabels(dataDir, ls.all) : HelpTexts::PadLabelFn{});
        }
        UiRenderer ui(device, kFormat, menu.assets());
        MTLTextureDescriptor* td = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:kFormat width:NSUInteger(w) height:NSUInteger(h) mipmapped:NO];
        td.usage = MTLTextureUsageRenderTarget | MTLTextureUsageShaderRead;
        td.storageMode = MTLStorageModePrivate;
        id<MTLTexture> target = [device newTextureWithDescriptor:td];
        ui.begin(float(w), float(h));
        backdrop(ui);
        const std::string p = page;
        std::unique_ptr<fuel::GaragePreview> garage;  // PK20
        std::unique_ptr<MenuAssets> worldAssets;
        std::unique_ptr<UiRenderer> worldUi;
        auto arg = [&](int k, float def) {
            size_t at = 0;
            for (int i = 0; i < k; ++i) {
                at = page.find(':', at);
                if (at == std::string::npos) return def;
                ++at;
            }
            return std::strtof(page.c_str() + at, nullptr);
        };
        const std::string word = menu.textOf(300);
        if (page == "GLYPHS") {  // PK26: SMALL_FONT_PC's codes 1..19 = the 0xa675c8 glyph names (the {STR_*} pad glyphs)
            ui.rect(0, 0, 1280, 720, Rgba{0.25f, 0.25f, 0.28f, 1});
            TextStyle st;
            st.font = 0, st.height = 30, st.fill = Rgba{1, 1, 1, 1}, st.outline = Rgba{0, 0, 0, 1};
            for (int i = 1; i < 20; ++i) {
                const float x = 60 + float((i - 1) % 5) * 240, y = 60 + float((i - 1) / 5) * 150;
                ui.text(i == 10 ? std::string("\xc2\x8a") : std::string(1, char(i)), x, y, 200, st);  // 10: help_texts glyphByte
                ui.text(std::to_string(i) + " " + HelpTexts::glyphName(i), x, y + 50, 220, st);
            }
        } else if (page.rfind("PADGLYPHS", 0) == 0) {  // pad-ux: every pad prompt of the shot's family, in text, beside FUEL's own icons
            ui.rect(0, 0, 1280, 720, Rgba{0.12f, 0.12f, 0.13f, 1});
            TextStyle st;
            st.font = 0, st.height = 26, st.fill = Rgba{1, 1, 1, 1}, st.outline = Rgba{0, 0, 0, 1};
            const Font& f0 = menu.assets().font(0);
            if (const Glyph* old = f0.glyph(0x101))
                std::printf("menu-shot: small font ref top %.1f height %.1f; old pad icon A: top %.1f, %.1f x %.1f\n", f0.refTop, f0.refHeight, old->top,
                            old->x1 - old->x0, old->y1 - old->y0);
            ui.text(std::string("Prompts: ") + pad::familyName(pad::state().family) + (shotPad ? "" : " (no pad: key caps)"), 40, 24, 1200, st);
            for (int i = 0; i < pad::kIconCount; ++i) {
                const float x = 40 + float(i % 5) * 244, y = 66 + float(i / 5) * 56;
                ui.text(pad::iconText(i) + " " + pad::buttonName(i), x, y, 240, st);
            }
            // FUEL's glyphs 1..19 as prompts, then FUEL's own SMALL_FONT icons (0x101..) on the same line height for size
            // and baseline, then the key caps: "Xg" on both sides of each.
            const float y = 66 + float((pad::kIconCount + 4) / 5) * 56 + 20;
            std::string row;
            for (int i = 1; i < 20; ++i) row += pad::iconText(pad::iconForGlyph(i));
            ui.text("Xg" + row + "Xg", 40, y, 1200, st);
            {
                const float k = st.height / f0.refHeight;
                float x = 40;
                const float yy = y + 56;
                ui.text("Xg", x, yy, 100, st);
                x += ui.textWidth("Xg", st);
                for (int i = 1; i < 20; ++i)
                    if (const Glyph* g = f0.glyph(0x100 + uint32_t(i)); g && g->page < f0.pages.size()) {
                        const BitmapRef& pg = f0.pages[g->page];
                        const float iw = 1.0f / float(pg.width), ih = 1.0f / float(pg.height);
                        ui.image(pg, x, yy + (g->top - f0.refTop) * k, (g->x1 - g->x0) * k, (g->y1 - g->y0) * k, Rgba{1, 1, 1, 1}, g->x0 * iw, g->y0 * ih,
                                 g->x1 * iw, g->y1 * ih);
                        x += (g->x1 - g->x0) * k;
                    }
                ui.text("Xg  (FUEL)", x, yy, 300, st);
            }
            const bool was = UiRenderer::padIcons;
            UiRenderer::padIcons = false;
            std::string caps;
            for (int i = 1; i < 19; ++i) caps += i == 10 ? std::string("\xc2\x8a") : std::string(1, char(i));
            ui.text("Xg" + caps + "Xg  (key caps)", 40, y + 112, 1200, st);
            UiRenderer::padIcons = was;
            if (const char* ap = std::getenv("FUEL_SHOT_ATLAS")) std::printf("menu-shot: atlas -> %s %s\n", ap, pad::writeAtlasPng(device, ap) ? "ok" : "failed");
        } else if (bootDialog >= 0) {  // PK26
            BootScreens b(dataDir, menu.assets(), [&](uint32_t tt) { return menu.textOf(tt); }, BootScreens::Dialog(bootDialog));
            const float t = arg(1, bootDialog == BootScreens::TRCMessage ? 3.5f : 1.0f);
            b.preview(BootScreens::Dialog(bootDialog), t, shotPad);
            ui.rect(-400, -400, 2080, 1520, Rgba{0, 0, 0, 1});
            b.draw(ui, loadExeImage(device, dataDir, ExeImage::SaveIcon), 0.7f);
            std::printf("menu-shot: boot dialog %d at %.2f s: pulse %.2f, TT %u, continue %d, save icon %d\n", bootDialog, t, b.pulseAlpha(),
                        b.pressStartText(), int(b.continueShown()), int(b.saveIconOn()));
        } else if (loadingPage && page.rfind("LOADING", 0) == 0) {
            worldAssets = std::make_unique<MenuAssets>(dataDir, std::vector<std::string>{"DATAS\\LWYOMING.DPC"}, false);
            worldUi = std::make_unique<UiRenderer>(device, kFormat, *worldAssets);
            worldUi->begin(float(w), float(h));
            LoadingTips tips;
            tips.build(menu.text());
            tips.update(0.016f);
            const float t = arg(1, 0.45f);
            const std::string* tip = tips.current() ? menu.text().get(tips.current()) : nullptr;
            std::printf("menu-shot: LOADING_BITMAP %ux%u, tip TT %u\n", worldAssets->image("LOADING_BITMAP").width, worldAssets->image("LOADING_BITMAP").height, tips.current());
            drawLoadingScreen(*worldUi, *worldAssets, worldAssets->image("LOADING_BITMAP"), tip ? helpTexts().expand(*tip) : std::string(), word, t);  // PK26: {STR_X}
        } else if (loadingPage && page.rfind("BOOT", 0) == 0) {
            const float t = arg(1, 0.45f);
            drawBootLoading(ui, menu.assets(), loadExeImage(device, dataDir, ExeImage::BootLogo), word, t, std::fmod(t, 6.2831855f));
        } else if (loadingPage && page.rfind("LOGO:", 0) == 0) {
            ui.rect(0, 0, 1280, 720, Rgba{0.3f, 0.45f, 0.6f, 1});  // stands for the world behind
            LogoTransition lt;
            lt.start();
            const float p = arg(1, 0), q = arg(2, -1);
            // Advance to the asked timers: p grows from the start; q only exists in state 3.
            lt.update(p, false);
            if (q >= 0) {
                for (int i = 0; i < 400 && lt.holding(); ++i) lt.update(0.05f, true);
                lt.update(std::max(0.0f, lt.q() - q), true);
            }
            std::printf("menu-shot: logo p %.3f q %.3f\n", lt.p(), lt.q());
            lt.draw(ui, menu.assets(), word, 0.45);
        } else if (loadingPage && page.rfind("RICHTEXT:", 0) == 0) {
            const uint32_t tt = uint32_t(arg(1, 767));
            std::string text = menu.textOf(tt);
            if (tt == 767) {  // "%s~§075Time playing: %s~§075Advancement: %s" with sample values
                char b[256];
                std::snprintf(b, sizeof b, text.c_str(), "PROFILE 1", "12:34:56", "42 %");
                text = b;
            }
            ui.rect(300, 200, 680, 320, Rgba{0.2f, 0.2f, 0.25f, 1});
            TextStyle st;
            st.font = 0, st.height = 30, st.fill = Rgba{1, 1, 1, 1}, st.outline = Rgba{0, 0, 0, 1};
            const float hgt = ui.richText(text, 300, 200, 980, 520, 0, st, true, true);
            std::printf("menu-shot: rich text TT %u, %.1f high\n", tt, hgt);
        } else if (hudFree) {
            // PK19: the free-roam HUD on the current hub's camp (minimap event icons 0x4ab220; "hudfree:x:z" puts the player at
            // x, z; "hudfree:x:z:tx:tz" also sets the map's GPS target for the pin 0x4aad40).
            HudState hud;
            hud.speedKmh = 0, hud.race = false, hud.settle = true, hud.dt = 0.016f;
            hud.quat = {0, 0, 0, 1};
            std::vector<float> a;
            for (size_t at = p.find(':'); at != std::string::npos; at = p.find(':', at + 1)) a.push_back(std::strtof(p.c_str() + at + 1, nullptr));
            if (a.size() >= 2) hud.px = a[0], hud.pz = a[1];
            else if (!menu.hubs().empty()) hud.px = float(menu.hubs()[0].camp.x), hud.pz = float(menu.hubs()[0].camp.z);
            if (a.size() >= 4) menu.setMapTarget(a[2], a[3]);
            hud.hubName = menu.hubs().empty() ? "" : menu.textOf(menu.hubs()[0].nameText);
            std::printf("menu-shot: hudfree player at %.0f, %.0f\n", hud.px, hud.pz);
            menu.drawHud(ui, hud);
        } else if (loadingPage) {
            HudState hud;
            hud.speedKmh = 87, hud.race = false;
            menu.drawHud(ui, hud);
            drawSaveIcon(ui, loadExeImage(device, dataDir, ExeImage::SaveIcon), arg(1, 0.3f), false);
        } else if (imageOnly) {
            ui.rect(0, 0, 1280, 720, Rgba{0.5f, 0.5f, 0.5f, 1});
            const BitmapRef b = menu.assets().image(p.substr(6));
            std::printf("menu-shot: %s: %ux%u\n", p.c_str() + 6, b.width, b.height);
            const float k = std::min(1200.0f / float(std::max(1u, b.width)), 680.0f / float(std::max(1u, b.height)));
            ui.image(b, 40, 20, float(b.width) * k, float(b.height) * k, Rgba{1, 1, 1, 1});
        } else if (p == "compteur" || p == "classement" || p == "boussole" || p == "hud") {
            HudState hud;
            hud.speedKmh = 87, hud.odometerKm = 12.3f, hud.race = true, hud.position = 2, hud.racers = 4;
            hud.hubName = menu.hubs().empty() ? "" : menu.textOf(menu.hubs()[0].nameText);
            hud.standings = {{"Rival 1", 1180, false}, {"Player", 1212, true}, {"Rival 2", 1240, false}, {"Rival 3", 1301, false}};
            hud.speedMs = 87 / 3.6f / 1.15f, hud.rpm = 4200, hud.maxRpm = 7000, hud.settle = true;  // p40 sample
            hud.page = hud::InfoPage::Checkpoint, hud.topA = 3, hud.topB = 13, hud.time = 83.47f;
            menu.drawHud(ui, hud);
        } else {
            menu.update(0);
            menu.previewAdvance(9.0f);  // PK18: the BandeauHelp marquee has run 9 s (it restarts off the right edge)
            menu.draw(ui);
            // ---- PK20: the garage pages' 3-D preview after FUEL_GARAGE_SHOT_T seconds (default 1.5) of 60 Hz frames
            const char* gt = std::getenv("FUEL_GARAGE_SHOT_T");
            const int frames = int(std::lround((gt ? std::max(0.0, std::atof(gt)) : 1.5) * 60.0));
            // FUEL_GARAGE_SHOT_KEYS (L R U D A B, e.g. "RR"): keys pressed at a third of the frames (browse live).
            const char* keys = std::getenv("FUEL_GARAGE_SHOT_KEYS");
            for (int i = 0; i < frames; ++i) {
                if (keys && i == frames / 3)
                    for (const char* k = keys; *k; ++k) {
                        static const std::pair<char, MenuKey> kMap[] = {{'L', MenuKey::Left}, {'R', MenuKey::Right}, {'U', MenuKey::Up},
                                                                         {'D', MenuKey::Down}, {'A', MenuKey::Accept}, {'B', MenuKey::Back}};
                        for (const auto& [c, mk] : kMap)
                            if (*k == c) menu.key(mk);
                        runGaragePreview(garage, device, dataDir, menu, 1.0f / 60.0f, true);
                    }
                runGaragePreview(garage, device, dataDir, menu, 1.0f / 60.0f, true);
            }
            if (garage) garage->waitReady();
            // ---- PK20 end
        }
        id<MTLCommandQueue> q = [device newCommandQueue];
        id<MTLCommandBuffer> cb = [q commandBuffer];
        if (garage && garage->scenesOn()) {
            // PK20: the backdrop (the world), the preview scenes, then the menus: the scenes are in the viewport's list
            // with priority +0x14 = 0x17 (0x68f910), sorted (0x666990) before the HUDs' (the pause HUD's +0x14 is
            // 0x3f666666), and queue their draws with preset +0xd21 = 2 against the HUDs' 4 (0x845ac0, 0x66690b).
            ui.begin(float(w), float(h));
            backdrop(ui);
            ui.encode(cb, target, true);
            garage->draw(cb, target);
            ui.begin(float(w), float(h));
            menu.draw(ui);
            ui.encode(cb, target, false);
            std::printf("menu-shot: garage preview scenes %d (1 vehicle, 2 rider)\n", garage->scenesOn());
        } else {
            (worldUi ? *worldUi : ui).encode(cb, target, true);
        }
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
- (void)startBoot:(int)first;  // PK26
@end

@implementation FuelMenuView {
    std::unique_ptr<Menu> _menu;
    std::unique_ptr<UiRenderer> _ui;
    id<MTLCommandQueue> _queue;
    std::string _self, _dataDir;
    std::unique_ptr<fuel::MenuAudio> _audio;  // the MENU music (MusicManager_G, menu context)
    std::unique_ptr<fuel::MenuSfx> _sfx;      // PK18: the menu click sounds (0x46d380)
    double _last;
    PadMenuRouter _padRouter;  // [port] remap
    int _optSerial, _vibTests;                      // P101: options-page changes applied so far
    std::unique_ptr<fuel::PadHaptics> _haptics;     // P101: the Controller page's test pulse (0x495670) in the menus
    std::unique_ptr<fuel::GaragePreview> _garage;   // PK20
    // ---- PK26: the boot screens (MenuManager_G boot dialogs) before the career world, and the help texts' pad monitor.
    std::unique_ptr<BootScreens> _boot;
    BootScreens::Input _bootIn;
    unsigned _bootPad;
    float _spin;
    id<MTLTexture> _saveIcon;
    std::function<std::string(int)> _keyLabels;
    // ---- end PK26
    HelpTexts::PadLabelFn _padLabels;  // pad-ux: the pad prompts from the bindings
    float _wheel;                      // pad-ux: wheel / trackpad rows not yet applied
}

- (instancetype)initWithFrame:(NSRect)frame device:(id<MTLDevice>)device menu:(std::unique_ptr<Menu>)menu self:(std::string)selfPath data:(std::string)dataDir {
    self = [super initWithFrame:frame device:device];
    if (self) {
        _menu = std::move(menu);
        _ui = std::make_unique<UiRenderer>(device, kFormat, _menu->assets());
        _queue = [device newCommandQueue];
        _self = selfPath, _dataDir = dataDir;
        // P101: the menu music plays at the Music Volume option (0x554040), not the SFX one.
        if (!std::getenv("FUEL_NO_AUDIO")) _audio = std::make_unique<fuel::MenuAudio>(dataDir, loadMenuOptions().musicVolume);
        if (!std::getenv("FUEL_NO_AUDIO")) _sfx = std::make_unique<fuel::MenuSfx>(dataDir, _menu->settings().volume);  // PK18: the SFX bus
        _keyLabels = keyboardLabels(dataDir, _menu->settings().all);  // PK26: 0x4e27c0's key names for the {STR_*} labels
        _padLabels = padLabels(dataDir, _menu->settings().all);       // pad-ux
        _bootPad = 0, _spin = 0, _wheel = 0;
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
    if (_boot && _boot->active()) {  // PK26: MenuManager +0x1426 Validate (Return / Enter / Space) and +0x1429 Back (Escape)
        _bootIn.any = true;
        for (MenuKey k : _menu->menuKeys(e.keyCode)) {  // [port] remap: MENU_VALID / MENU_BACK's keys
            if (k == MenuKey::Accept) _bootIn.validate = true;
            if (k == MenuKey::Back) _bootIn.back = true;
        }
        return;
    }
    if (_menu->rawKey(e.keyCode, e.isARepeat)) return;  // [port] remap: the remap page's capture
    for (MenuKey k : _menu->menuKeys(e.keyCode)) _menu->key(k);
}

// ---- PK26: starts the boot screens at `first` (LegalText for the boot, PressStart for a return to the title).
- (void)startBoot:(int)first {
    _boot = std::make_unique<BootScreens>(_dataDir, _menu->assets(), [m = _menu.get()](uint32_t tt) { return m->textOf(tt); },
                                          BootScreens::Dialog(first));
    _bootIn = {}, _bootPad = 0, _spin = 0;
    if (!_saveIcon) _saveIcon = loadExeImage(_ui->device(), _dataDir, ExeImage::SaveIcon);
    _audio.reset();  // MusicManager +0x20 = 1 from the boot open (0x589930) until PressStart opens (0x591b90)
}
// ---- end PK26

- (void)keyUp:(NSEvent*)e {  // P75: the map's held keys
    for (MenuKey k : _menu->menuKeys(e.keyCode)) _menu->keyUp(k);
}

- (void)pointerAt:(NSEvent*)e click:(bool)click {
    const NSPoint p = [self convertPoint:e.locationInWindow fromView:nil];
    const CGSize ds = self.drawableSize;
    const float kx = float(ds.width / self.bounds.size.width), ky = float(ds.height / self.bounds.size.height);
    _ui->begin(float(ds.width), float(ds.height));
    const float px = float(p.x) * kx, py = float(self.bounds.size.height - p.y) * ky;
    if (_boot && _boot->active()) {  // PK26: the mouse position in 0..1 of the screen ([0xa7c0c0] +0xc / +0x10)
        _bootIn.mouseX = (px - _ui->sx(0)) / _ui->canvasScale() / 1280.0f;
        _bootIn.mouseY = (py - _ui->sy(0)) / (_ui->sy(720) - _ui->sy(0));
        if (click) _bootIn.any = true;
        return;
    }
    _menu->pointer((px - _ui->sx(0)) / _ui->canvasScale(), (py - _ui->sy(0)) / (_ui->sy(720) - _ui->sy(0)) * 720.0f, click);
}
- (void)mouseMoved:(NSEvent*)e { [self pointerAt:e click:false]; }
- (void)mouseDown:(NSEvent*)e { [self pointerAt:e click:true]; }
// pad-ux [port]: the mouse wheel and trackpad scroll the menus (Menu::wheel, whole rows: a wheel notch = one row, a
// trackpad moves one row per row height swept). The hover follows the next mouse move, as FUEL's pointer does.
- (void)scrollWheel:(NSEvent*)e {
    if (_boot && _boot->active()) return;
    const CGSize ds = self.drawableSize;
    _ui->begin(float(ds.width), float(ds.height));
    const float rowPx = 49.0f * _ui->canvasScale() * float(self.bounds.size.height / std::max(1.0, ds.height));  // a row in view points
    _wheel += e.hasPreciseScrollingDeltas ? -float(e.scrollingDeltaY) / std::max(1.0f, rowPx) : -float(e.scrollingDeltaY);
    const int rows = int(_wheel);  // toward zero: the rest waits for the next event
    if (rows == 0) return;
    _wheel -= float(rows);
    _menu->wheel(rows);
}

- (void)mtkView:(MTKView*)view drawableSizeWillChange:(CGSize)size {}

- (void)drawInMTKView:(MTKView*)view {
    const double now = CACurrentMediaTime();
    const float dt = _last > 0 ? float(std::min(0.1, now - _last)) : 0.0f;
    _last = now;
    // ---- PK26: PaddleChecker_G vf17 0x4e36e0: the {STR_*} labels follow the pad (GameController) coming and going.
    // pad-ux [port]: and its family (pad_glyphs); the labels are the bound pad inputs' icons.
    {
        const pad::State ps = pad::detect(pad::Style(_menu->portOptions().padGlyphs));
        if (pad::paddleCheck(ps, _keyLabels, _padLabels)) std::printf("help texts: rebuilt (%s)\n", ps.present ? "pad prompts" : "key names");
    }
    if (_boot && _boot->active()) {
        GCExtendedGamepad* pad = GCController.controllers.firstObject.extendedGamepad;
        // [port] remap: MENU_VALID / MENU_BACK / MENU_START's pad inputs (input.PC: A, B, START by default)
        const fuel::input::PadState ps = readControllerPad();
        auto bound = [&](int action) {
            for (int c : _menu->bindings().padsOf(action))
                if (c >= 0 && fuel::input::padValue(ps, c) > 0.5f) return true;
            return false;
        };
        const unsigned nowPad = pad ? (bound(fuel::input::MenuValid) ? 1u : 0u) | (bound(fuel::input::MenuBack) ? 2u : 0u) |
                                          (bound(fuel::input::MenuStart) ? 4u : 0u) |
                                          (pad.dpad.up.pressed || pad.dpad.down.pressed || pad.leftThumbstick.xAxis.value * pad.leftThumbstick.xAxis.value +
                                                                                                    pad.leftThumbstick.yAxis.value * pad.leftThumbstick.yAxis.value > 0.25f ? 8u : 0u)
                                    : 0u;
        const unsigned edge = nowPad & ~_bootPad;
        _bootPad = nowPad;
        if (edge & 1) _bootIn.validate = true;  // pad A = MENU_VALID
        if (edge & 2) _bootIn.back = true;      // pad B = MENU_BACK
        if (edge & 4) _bootIn.start = true;     // pad Start (+0x142a)
        if (edge) _bootIn.any = true;
        _boot->update(dt, _bootIn, helpTexts().padMode());
        const float mx = _bootIn.mouseX, my = _bootIn.mouseY;
        _bootIn = {};
        _bootIn.mouseX = mx, _bootIn.mouseY = my;
        _spin = std::fmod(_spin + dt, 6.2831855f);  // LoadingDraw +0x38
        if (const int snd = _boot->takeSound(); _sfx) {
            if (snd >= 0) _sfx->play(snd);
            _sfx->update();
        }
        if (_boot->musicOn() && !_audio && !std::getenv("FUEL_NO_AUDIO")) _audio = std::make_unique<fuel::MenuAudio>(_dataDir, loadMenuOptions().musicVolume);
        if (_audio) _audio->update(dt);
        if (_boot->launch()) {  // 0x594a50: PlayLevel USA1 -> the career world (Dialog_Loading 0x58fc20 there)
            const MenuRequest r = _menu->careerBootRequest();
            _menu->sessionEnd(), execSession(_self, _dataDir, r, *_menu);
        }
        id<CAMetalDrawable> drawable = view.currentDrawable;
        if (!drawable) return;
        const CGSize ds = view.drawableSize;
        _ui->begin(float(ds.width), float(ds.height));
        _boot->draw(*_ui, _saveIcon, _spin);
        id<MTLCommandBuffer> cb = [_queue commandBuffer];
        _ui->encode(cb, drawable.texture, true);
        [cb presentDrawable:drawable];
        [cb commit];
        return;
    }
    // ---- end PK26
    {  // [port] remap: the pad's menu keys from the bindings; the remap page's capture takes the pad first
        const fuel::input::PadState ps = readControllerPad();
        std::vector<MenuKey> up;
        std::vector<MenuKey> keys = _padRouter.step(_menu->bindings(), ps, _menu->remapPageUp(), &up);
        if (_menu->padFrame(ps, dt)) keys.clear(), up.clear();
        for (MenuKey k : keys) _menu->key(k);
        for (MenuKey k : up) _menu->keyUp(k);  // P75: the map's held keys
    }
    _menu->update(dt);
    runGaragePreview(_garage, self.device, _dataDir, *_menu, dt, true);  // PK20
    _menu->updateTutorial(dt, true);  // P62: menu-side tutorials (11..15)
    if (_menu->optionsSerial() != _optSerial) {  // P101: the options pages apply live here too
        _optSerial = _menu->optionsSerial();
        {  // pad-ux: the controls layout / pad_glyphs may have changed: the labels again (the keys, the pad prompts)
            const fuel::Settings ls = _menu->liveSettings();
            _keyLabels = keyboardLabels(_dataDir, ls.all), _padLabels = padLabels(_dataDir, ls.all);
            const pad::State ps = pad::detect(pad::Style(_menu->portOptions().padGlyphs));
            pad::setState(ps);
            helpTexts().rebuild(ps.present, _keyLabels, ps.present ? _padLabels : HelpTexts::PadLabelFn{});
        }
        if (_audio) _audio->setVolume(_menu->options().musicVolume);
        if (_haptics) _haptics->setOption(_menu->portOptions().vibration);
        applyFullscreenLive(_menu->settings().fullscreen, self.window, self);
    }
    if (_menu->vibrationTests() != _vibTests) {  // 0x495670: VIBRATION switched on -> IMPACT_MEDIUM on the pad
        _vibTests = _menu->vibrationTests();
        if (!_haptics) _haptics = std::make_unique<fuel::PadHaptics>(_dataDir, _menu->liveSettings());
        _haptics->testPulse();
    }
    if (_haptics) _haptics->frame(dt, false);
    if (_audio) _audio->update(dt);
    if (const int s = _menu->takeSound(); _sfx) {  // PK18: one menu sound a frame (0x46d380)
        if (s >= 0) _sfx->play(s);
        _sfx->setVolume(_menu->liveSettings().volume);
        _sfx->update();
    }
    const MenuRequest r = _menu->take();
    if (r.kind == MenuRequest::QuitGame) {
        [NSApp terminate:nil];
        return;
    }
    if (r.kind == MenuRequest::StartEvent || r.kind == MenuRequest::FreeRide) _menu->sessionEnd(), execSession(_self, _dataDir, r, *_menu);
    if (r.kind == MenuRequest::QuitToTitle) [self startBoot:BootScreens::PressStart];  // PK26: back to the PressStart dialog
    id<CAMetalDrawable> drawable = view.currentDrawable;
    if (!drawable) return;
    const CGSize ds = view.drawableSize;
    _ui->begin(float(ds.width), float(ds.height));
    backdrop(*_ui);
    id<MTLCommandBuffer> cb = [_queue commandBuffer];
    if (_garage && _garage->scenesOn()) {  // PK20: the preview scenes between the backdrop and the menus (runMenuShot)
        _ui->encode(cb, drawable.texture, true);
        _garage->draw(cb, drawable.texture);
        _ui->begin(float(ds.width), float(ds.height));
        _menu->draw(*_ui);
        _ui->encode(cb, drawable.texture, false);
    } else {
        _menu->draw(*_ui);
        _ui->encode(cb, drawable.texture, true);
    }
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
        // P78: the boot loading draw (usergame.tsc ForceCDLoadingDraw ON: logo, spinner, LOADING...) while the menu loads.
        auto boot = std::make_unique<LoadingWindow>(dataDir, s.language, gameWindowSize(s), LoadingWindow::Boot);
        auto menu = std::make_unique<Menu>(dataDir, s, MenuMode::Title);
        if (!menu->scripts().problems.empty() && menu->scripts().pages.empty()) {
            std::fprintf(stderr, "fuelview: no menu pages under %s\n", dataDir.c_str());
            return 1;
        }
        // PK26: no page -> the boot screens from LegalText (0x589930 mode 0); PRESSSTART / TITLE (a return to the title) ->
        // the PressStart dialog; any other page opens as before (--menu-page).
        const bool titlePage = startPage == "PRESSSTART" || startPage == "TITLE";
        if (!startPage.empty() && !titlePage) menu->showPage(startPage);
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
        if (startPage.empty() || titlePage) [view startBoot:startPage.empty() ? BootScreens::LegalText : BootScreens::PressStart];  // PK26
        [window center];
        [window makeKeyAndOrderFront:nil];
        boot->close();
        [window makeFirstResponder:view];
        applyWindowSettings(s, window, view);
        [NSApp activateIgnoringOtherApps:YES];
        [NSApp run];
    }
    return 0;
}

GameOverlay::GameOverlay(id<MTLDevice> device, MTLPixelFormat format, const std::string& self, const std::string& dataDir, const Settings& s,
                         const std::string& event, const std::string& hub, const std::string& vehicle)
    : self_(self), dataDir_(dataDir), device_(device) {
    char buf[4096];
    uint32_t size = sizeof buf;
    if (_NSGetExecutablePath(buf, &size) == 0) self_ = fs::weakly_canonical(buf).string();
    menu_ = std::make_unique<Menu>(dataDir, s, MenuMode::Pause);
    menu_->setSession(event, hub, vehicle);
    ui_ = std::make_unique<UiRenderer>(device, format, menu_->assets());
    format_ = format;  // PK26
    saveIcon_ = loadExeImage(device, dataDir, ExeImage::SaveIcon);
    savesSeen_ = menu_->saveCount();
    if (!std::getenv("FUEL_NO_AUDIO")) sfx_ = std::make_unique<fuel::MenuSfx>(dataDir, s.volume);  // PK18: the menu click sounds
    keyLabels_ = keyboardLabels(dataDir, s.all);  // PK26: 0x4e27c0's key names for the {STR_*} labels
    padLabels_ = padLabels(dataDir, s.all);       // pad-ux
    labelSerial_ = menu_->optionsSerial();
}

// pad-ux [port]: the options pages changed the bindings (controls layout) or pad_glyphs: the labels again.
void GameOverlay::refreshLabels() {
    if (labelSerial_ == menu_->optionsSerial()) return;
    labelSerial_ = menu_->optionsSerial();
    const Settings ls = menu_->liveSettings();
    keyLabels_ = keyboardLabels(dataDir_, ls.all), padLabels_ = padLabels(dataDir_, ls.all);
    const pad::State ps = pad::detect(pad::Style(menu_->portOptions().padGlyphs));
    pad::setState(ps);
    helpTexts().rebuild(ps.present, keyLabels_, ps.present ? padLabels_ : HelpTexts::PadLabelFn{});
}

void GameOverlay::wheel(float rows) {
    if (!open_ || (waitSave_ && !menu_->alertUp())) return;
    menu_->wheel(int(rows));
}

// ---- PK26: the career boot, see game_menu.h.
void GameOverlay::startCareerBoot() {
    bootTimer_ = 0.5f, bootDue_ = false, gameStarted_ = false;
    bootAssets_ = std::make_unique<MenuAssets>(dataDir_, std::vector<std::string>{"DATAS\\LWYOMING.DPC"}, false);
    bootUi_ = std::make_unique<UiRenderer>(ui_->device(), format_, *bootAssets_);
    std::printf("boot: Dialog_Loading (career save loaded: SaveManager_LoadSlot(0); CheckUnLock re-evaluated at the load)\n");
}

bool GameOverlay::careerBootDue() { return std::exchange(bootDue_, false); }
// ---- end PK26

GameOverlay::~GameOverlay() = default;

void GameOverlay::openPause() {
    menu_->setMode(MenuMode::Pause);
    open_ = true;
}

void GameOverlay::openInfos() {
    menu_->setMode(MenuMode::Pause);
    menu_->openMapTab();
    open_ = true;
}

bool GameOverlay::openArch() {
    if (open_ || !menu_->archAccept()) return false;
    open_ = true;
    return true;
}

// PK25: an MCAlert over the end-race pages runs "Pause" (0x441670) [port: the world stops under the box; FUEL also opens
// the pause page set there] and its close "UnPause" (0x440b50: race / end-race HUD screens).
bool GameOverlay::worldRuns() const { return open_ && menu_->mode() == MenuMode::Results && !menu_->alertUp(); }

void GameOverlay::openResults(const std::vector<ResultRow>& rows, bool won, int level) {
    menu_->setResults(rows, won, level);
    menu_->setMode(MenuMode::Results);
    open_ = true;
}

bool GameOverlay::keyDown(unsigned short code, bool repeat) {
    if (!open_ && menu_->alertUp()) {  // PK25: a load-error MCAlert does not pause (0x4411a0); it still has the keys
        for (MenuKey k : menu_->menuKeys(code)) menu_->key(k);
        return true;
    }
    if (!open_ && menu_->tutorialOpen()) {  // P56: the tutorial page has the keys (Return / Escape)
        for (MenuKey k : menu_->menuKeys(code)) menu_->tutorialKey(k);
        return true;
    }
    if (!open_) {
        for (MenuKey bk : menu_->menuKeys(code))
            if (bk == MenuKey::Accept) menu_->bannerAccept();  // PK08 0x4af920: skip a banner's hold
        const auto& pk = menu_->bindings().keysOf(fuel::input::Pause);  // [port] remap: PAUSE_MODE's keys (Escape)
        if (std::find(pk.begin(), pk.end(), int(code)) != pk.end()) {  // the pause key opens the menu
            if (!gameStarted_) return true;  // PK26: 0x444400 keeps the pause HUD locked until GAMEStarted
            if (!pauseAllowed()) return false;  // PK17: 0x539ac0 (an RTC runs): the key goes on to the RTC skip
            openPause();
            menu_->pauseDropTutorials();  // PK17: 0x4442f0 -> 0x56eb20
            return true;
        }
        return false;
    }
    if (waitSave_ && !menu_->alertUp()) return true;  // PK17: 0x46ea50 waits for the save: no tab tick, no keys
    if (menu_->rawKey(code, repeat)) return true;  // [port] remap: the remap page's capture
    for (MenuKey k : menu_->menuKeys(code)) menu_->key(k);
    return true;
}

void GameOverlay::keyUp(unsigned short code) {
    for (MenuKey k : menu_->menuKeys(code)) menu_->keyUp(k);
}

void GameOverlay::pointer(float x, float y, bool click, float viewW, float viewH) {
    if (!open_ || (waitSave_ && !menu_->alertUp())) return;  // PK17: the save-on-close wait
    ui_->begin(viewW, viewH);
    menu_->pointer((x - ui_->sx(0)) / ui_->canvasScale(), (y - ui_->sy(0)) / (ui_->sy(720) - ui_->sy(0)) * 720.0f, click);
}

void GameOverlay::pollGamepad() {
    // [port] remap: the pad's menu keys from the bindings (PadMenuRouter); the remap page's capture takes the pad first.
    const fuel::input::PadState padState = readControllerPad();
    std::vector<MenuKey> padUp;
    std::vector<MenuKey> keys = padRouter_.step(menu_->bindings(), padState, menu_->remapPageUp(), &padUp);
    if (open_ && menu_->padFrame(padState, 1.0f / 60.0f)) keys.clear(), padUp.clear();
    bool pauseDown = false;  // PAUSE_MODE's pad input (input.PC: START)
    for (int c : menu_->bindings().padsOf(fuel::input::Pause)) pauseDown = pauseDown || (c >= 0 && fuel::input::padValue(padState, c) > 0.5f);
    {  // PK26: PaddleChecker_G vf17 0x4e36e0 rebuilds the {STR_*} labels when a pad comes or goes
        // pad-ux [port]: or changes family (pad_glyphs); the labels are the bound pad inputs' icons.
        refreshLabels();
        const pad::State ps = pad::detect(pad::Style(menu_->portOptions().padGlyphs));
        if (pad::paddleCheck(ps, keyLabels_, padLabels_)) std::printf("help texts: rebuilt (%s)\n", ps.present ? "pad prompts" : "key names");
    }
    // ---- PK25: LevelManipulator vf17 0x57dd10. A pad slot that lost its pad (game+0x3214 +0x30 + 0x18 i == 1) pauses the
    // game once the player is free (Game_IsPlayerFree 0x57db50: no race running, no cut-scene): 0x57df10 opens the pause
    // page set when none is up, and MenuManager open(4, 0x14) shows the empty TestMenu dialog over it (25 % black fade)
    // [port: the fade is not drawn]. The Guide / sign-in / NGSM branches are Games for Windows - LIVE (dropped).
    // [port] the pause comes once per disconnect (a reconnect clears it); GameController reports the pads.
    {
        const int pads = int(GCController.controllers.count);
        if (padsSeen_ >= 0 && pads < padsSeen_) padLost_ = true, std::printf("pad: controller disconnected\n");
        if (pads > padsSeen_ && padsSeen_ >= 0) padLost_ = false;
        padsSeen_ = pads;
        if (padLost_ && playerFree_ && !logo_.active() && !rtcLoading_) {
            padLost_ = false;
            if (!open_) openPause(), std::printf("pad: game paused (0x57df10)\n");
        }
    }
    if (!open_ && menu_->alertUp()) {  // PK25: the alert has the pad's action keys
        for (MenuKey k : keys) menu_->key(k);
        return;
    }
    // ---- end PK25
    if (!open_ && menu_->tutorialOpen()) {
        for (MenuKey k : keys) menu_->tutorialKey(k);
        return;
    }
    if (!open_) {
        for (MenuKey k : keys)
            if (k == MenuKey::Accept) menu_->bannerAccept();  // PK08 0x4af920
        if (pauseDown && !padPause_ && pauseAllowed())  // PK26: gameStarted_; PK17: 0x539ac0. [port] remap: PAUSE_MODE's pad input
            openPause(), menu_->pauseDropTutorials();  // PK17: 0x4442f0 -> 0x56eb20
        padPause_ = pauseDown;
        return;
    }
    padPause_ = pauseDown;
    if (waitSave_ && !menu_->alertUp()) return;  // PK17: the save-on-close wait
    for (MenuKey k : keys) menu_->key(k);
    for (MenuKey k : padUp) menu_->keyUp(k);  // P75: the map's held keys
}

bool GameOverlay::holdRaceStart() {
    if (!logoArmed_) logoArmed_ = true, logo_.start(1.0f);  // PK15: 0x6333b0 holds q at PP52 + PP50 + 1 while PREPARING
    if (logo_.holding()) return true;
    logoArmed_ = false;
    return false;
}

GameOverlay::Action GameOverlay::update(float dt) {
    clock_ += dt;
    if (bootTimer_ >= 0) {  // PK26 0x58fc20: +0xe8 -= dt in states 6/0xc/0xd/0xe/0xf; at <= 0 in state 0xf -> 0x590300
        bootTimer_ -= dt;
        if (bootTimer_ <= 0) bootTimer_ = -1, bootDue_ = true;
    }
    logo_.update(dt, logoReady_);  // PK15: ready = the streamer and the terrain workers are idle (main.mm tlpReady)
    // 0x502580: the spinner angle +0x38 += 1.0 x dt (wrapped); 0x503500: the icon comes on while the storage saves
    // (+0x77) and stays 1.0 s after it is idle [port: the save is instant, so 1 s after each one].
    spinAngle_ = std::fmod(spinAngle_ + dt, 6.2831855f);
    if (menu_->saveCount() != savesSeen_) savesSeen_ = menu_->saveCount(), saveIconOn_ = true, saveIconTime_ = 1.0f;
    else if (saveIconOn_ && (saveIconTime_ -= dt) < 0) saveIconOn_ = false;
    menu_->update(dt);
    if (menu_->takeAlertPause() && !open_) openPause();  // PK25 0x441670: "Pause" + the pause page set
    runGaragePreview(garage_, device_, dataDir_, *menu_, dt, open_);  // PK20: only while the pause menu is up
    if (const int snd = menu_->takeSound(); sfx_) {  // PK18: 0x46d380 (also the sound of the key that closes the menu)
        if (snd >= 0 && (open_ || snd == 0)) sfx_->play(snd);
        sfx_->setVolume(menu_->liveSettings().volume);
        sfx_->update();
    }
    menu_->updateTutorial(dt, open_);  // P56 / P62: menuOpen
    // P66: the player's vehicle record +0x10 collects the time driven in it (0x5cb8c0 / 0x5ac3a0: the frame times, added
    // in batches of up to 0.5 s; < 0 restarts at 0) [O]; the profile's favourite vehicle is the largest (0x56a700).
    if (!open() && dt > 0)
        if (Career* c = menu_->career()) c->addVehicleTime(menu_->sessionVehicle(), dt);
    MenuRequest r = menu_->take();
    // ---- PK17: InGamePauseHUD vf08 0x46e9b0: closing with +0x80 set runs SaveGameData 0x54bbc0 (unless a save is
    // pending, +0x77) and parks the menu (+0x98 = 1); vf02 0x46ea50 closes it on a later frame once the storage is idle
    // (+0x76 / +0x77 clear). A failed save keeps it parked under PK25's MCAlert until the alert is answered. Every way
    // out of the pause page set goes through vf08; the port's quit-game path saves without waiting (the process ends).
    if (waitSave_) {
        r = {};
        if (menu_->storageIdle()) waitSave_ = false, r = std::exchange(heldClose_, MenuRequest{}), std::printf("pause: save idle, menu closes\n");
    } else if (r.kind != MenuRequest::None && open_ && menu_->mode() == MenuMode::Pause && menu_->pauseDirty()) {
        if (r.kind == MenuRequest::QuitGame) {
            menu_->closeSave();
        } else {
            menu_->closeSave();
            waitSave_ = true, heldClose_ = r, r = {};
            std::printf("pause: +0x80 set: SaveGameData on close, the menu waits for the storage\n");
        }
    }
    const Action act = finishClose(r);
    backdropFrame(dt);
    return act;
    // ---- end PK17
}

GameOverlay::Action GameOverlay::finishClose(const MenuRequest& r) {
    switch (r.kind) {
        case MenuRequest::Resume: open_ = false; return Resume;
        case MenuRequest::Restart: open_ = false, menu_->raceRestarted(); return Restart;
        case MenuRequest::StartEvent:
        case MenuRequest::FreeRide: menu_->sessionEnd(), execSession(self_, dataDir_, r, *menu_);  // P99: keep the session's state
        case MenuRequest::QuitToTitle: menu_->sessionEnd(), execMenu(self_, dataDir_, "PRESSSTART");  // PK26: PressStart, not the legal screen
        case MenuRequest::QuitGame: [NSApp terminate:nil]; return None;
        case MenuRequest::Teleport:  // P44 (0x482b60 -> 0x4caa90); P78: through the teleport queue 0x57e650 (the logo stencil)
            open_ = false, teleport_ = {r.x, r.z};
            logo_.start();
            return Teleport;
        default: return None;
    }
}

// ---- PK17: InGameMenuHUD's show / hide / fade (game_menu.h). vf07 0x46aca0 (shown with the Pause mode, +0x4c = 1):
// fade in from 0.001, "SetHudsVisibleState OFF", +0x80 = 0 (the pause page set's save flag); vf08 0x46ade0: fade out
// ("UnPause" at once: the world runs under the fading page); vf10 0x46aed0 at the end of the fade-out: +0x4c = 0,
// "SetHudsVisibleState ON". The pause pages (not the end-race MissionHUD pages) and an in-game PageTutorial (vf09
// 0x4cf2d0: Pause unless an overlay is up) are InGameMenuHUDs.
void GameOverlay::backdropFrame(float dt) {
    const bool shown = (open_ && menu_->mode() != MenuMode::Results) || (!open_ && menu_->tutorialOpen());
    if (shown && !wasShown_) {
        fade_.show();
        pauseMode_ = 1, hudsOff_ = true;
        if (open_) menu_->clearPauseDirty();
        backdropEvents_ |= Opened;
    } else if (!shown && wasShown_ && pauseMode_ != 0) {
        fade_.hide();
        backdropEvents_ |= Closed;
    }
    wasShown_ = shown;
    if (fade_.step(dt) < 0 && pauseMode_ != 0) {
        pauseMode_ = 0, hudsOff_ = false;
        backdropEvents_ |= FadedOut;
    }
}
// ---- end PK17

void GameOverlay::draw(id<MTLCommandBuffer> cb, id<MTLTexture> target, const HudState& hud, bool hudShown) {
    ui_->begin(float(target.width), float(target.height));
    // P102: InGamePlayer's HUD draw 0x443290 draws VignettageHUD (IGP +0x4830) first and before its SetHudsVisibleState
    // gate ([0x9ff9f2] at 0x4433d4), so the vignette stays on while the HUDs are hidden (cut-scenes such as the
    // TELEPORT_HELICO intro, the pause menu, which runs `SetHudsVisibleState OFF`); only DisableOSD ([0xa65420]) and a
    // playing video (each HUD's +0x33, 0x694290) remove it. The port has neither in a session.
    if (hud::vignetteShown(false, false)) menu_->drawVignette(*ui_);
    if (open_) {
        // PK17: with the host drawing FUEL's backdrop (20:00 sky, tint, blur: 0x46aca0 / 0x46af90) the port's dim goes.
        if (!worldRuns() && !backdropHosted_) ui_->rect(-400, -400, 2080, 1520, Rgba{0, 0, 0, 0.45f});  // the world dims behind the menu [port]
        if (garage_ && garage_->scenesOn()) {  // PK20: the preview scenes over the world, under the menus (runMenuShot)
            ui_->encode(cb, target, false);
            garage_->draw(cb, target);
            ui_->begin(float(target.width), float(target.height));
        }
        menu_->draw(*ui_);
    } else {
        if (hudShown && !menu_->alertUp() && !hudsOff_) menu_->drawHud(*ui_, hud, false);  // PK25: MissionHUD vf19 0x4b8cb0 (+0xd20); PK17: SetHudsVisibleState
        if (menu_->tutorialOpen()) menu_->drawTutorial(*ui_);  // P56
        menu_->drawAlerts(*ui_);  // PK25: a load-error MCAlert without the pause menu
    }
    // PK13: LoadingDraw vf18 0x502710: while a running RTC's player holds 0x10e (0x539a50, TR_DEATH respawn) the
    // "LOADING..." line 0x503ce0 is drawn (before the stencil step).
    if (rtcLoading_) drawLoadingWord(*ui_, menu_->assets(), menu_->textOf(300), clock_);
    // LoadingDraw 0x502710 step 6: the stencil unless a menu page is up (0x694290(0)).
    if (!open_ && logo_.active()) logo_.draw(*ui_, menu_->assets(), menu_->textOf(300), clock_);
    if ((!open_ || worldRuns()) && saveIconOn_) drawSaveIcon(*ui_, saveIcon_, spinAngle_, false);  // [I] the end-race pages are HUD pages
    ui_->encode(cb, target, false);
    if (bootTimer_ >= 0 && bootUi_) {  // PK26: the Loading dialog (mode 0x13): the 0.25 black fade, then LOADING_BITMAP 0 0 1 1
        bootUi_->begin(float(target.width), float(target.height));
        bootUi_->rect(bootUi_->viewLeft(), bootUi_->viewTop(), bootUi_->viewWidth(), bootUi_->viewHeight(), Rgba{0, 0, 0, 1});  // [port] no world under it yet
        if (const BitmapRef b = bootAssets_->image("LOADING_BITMAP")) bootUi_->image(b, 0, 0, 1280, 720, Rgba{1, 1, 1, 1});
        bootUi_->encode(cb, target, false);
    }
}

// ---- PK17: runPauseCheck (game_menu.h)
int runPauseCheck(const std::string& dataDir) {
    int bad = 0;
    auto expect = [&](bool ok, const std::string& what) {
        if (!ok) ++bad;
        std::printf("pause check: %s %s\n", ok ? "ok  " : "FAIL", what.c_str());
    };
    id<MTLDevice> device = MTLCreateSystemDefaultDevice();
    if (device == nil) {
        std::printf("pause check: no Metal device (skipped)\n");
        return 0;
    }
    // A scratch folder: the player's settings.txt and career.sav are never read or written here.
    char tmpl[] = "/tmp/fuel-pause-check-XXXXXX";
    const char* dir = mkdtemp(tmpl);
    if (!dir) return 1;
    const std::string settingsFile = std::string(dir) + "/settings.txt", save = std::string(dir) + "/career.sav";
    ::setenv("FUEL_SETTINGS", settingsFile.c_str(), 1);
    ::setenv("FUEL_CAREER_SAVE", save.c_str(), 1);
    ::setenv("FUEL_NO_AUDIO", "1", 1);
    ::unsetenv("FUEL_AUTOSAVE_OFF");
    { std::ofstream(settingsFile) << "data = " << dataDir << "\n"; }
    Settings s = loadSettings();
    s.dataDir = dataDir;
    {
        Menu m(dataDir, s, MenuMode::Main);
        m.requestSave();
        const int n = m.saveCount();
        m.markDirty();
        expect(m.lastSaveCode() == 0 && m.saveCount() == n + 1 && !m.pauseDirty(), "front-end menus: markDirty saves at once [port]");
    }
    GameOverlay o(device, MTLPixelFormatBGRA8Unorm_sRGB, "fuelview", dataDir, s, "", "", "QUAD_ROAD_2");
    Menu& m = o.menu();
    if (Career* c = m.career())  // the menu-side tutorials (11..15) would take the keys on a fresh career
        for (auto& t : c->state().tutorials) t.seen = t.off = 1;
    auto near = [](float a, float b) { return std::fabs(a - b) < 1e-5f; };
    // vf07 / vf10: the pause page fades in at 4 per second from 0.001; the HUDs go off.
    o.openPause();
    o.update(0.1f);
    int ev = o.takeBackdropEvents();
    expect(ev == GameOverlay::Opened && o.backdropShown() && near(o.backdropAlpha(), 0.401f), "open: vf07 (Opened), alpha 0.001 + 4 x 0.1");
    o.update(0.1f), o.update(0.1f);
    expect(o.takeBackdropEvents() == 0 && o.backdropAlpha() == 1.0f, "vf10: alpha clamps at 1 (0x4c8c50)");
    // vf08 0x46e9b0: +0x80 set -> SaveGameData, the menu parked (+0x98) until the storage is idle (0x46ea50).
    const int saves0 = m.saveCount();
    m.markDirty();
    expect(m.pauseDirty() && m.saveCount() == saves0, "pause page change: +0x80 = 1, nothing saved yet");
    o.keyDown(53);  // Back: Resume
    GameOverlay::Action a = o.update(0.016f);
    expect(a == GameOverlay::None && o.open() && o.savingOnClose() && m.saveCount() == saves0 + 1 && !m.pauseDirty(),
           "close with +0x80: SaveGameData, the menu stays (+0x98 = 1, +0x80 = 0)");
    a = o.update(0.016f);
    ev = o.takeBackdropEvents();
    expect(a == GameOverlay::Resume && !o.open() && !o.savingOnClose() && ev == GameOverlay::Closed && o.backdropShown(),
           "storage idle: vf08 again, the menu closes (Closed; +0x4c kept while it fades)");
    o.update(0.1f), o.update(0.1f);
    ev = o.takeBackdropEvents();
    expect(ev == 0 && o.backdropShown(), "fading out over 0.25 s");
    o.update(0.1f);
    ev = o.takeBackdropEvents();
    expect(ev == GameOverlay::FadedOut && !o.backdropShown() && o.backdropAlpha() == 0, "vf10 < 0: FadedOut, +0x4c = 0 (0x46aed0)");
    // No change: the close is immediate.
    o.openPause(), o.update(0.3f), o.takeBackdropEvents();
    o.keyDown(53);
    a = o.update(0.016f);
    expect(a == GameOverlay::Resume && !o.open() && m.saveCount() == saves0 + 1, "close without +0x80: at once, no save");
    o.update(0.3f), o.takeBackdropEvents();
    // A failed save keeps the menu parked under PK25's MCAlert until it is answered.
    if (::geteuid() != 0) {
        o.openPause(), o.update(0.3f), o.takeBackdropEvents();
        m.markDirty();
        ::chmod(save.c_str(), 0444);
        o.keyDown(53);
        o.update(0.016f);
        for (int i = 0; i < 10; ++i) o.update(0.1f);
        expect(o.open() && o.savingOnClose() && m.alertUp() && m.dialogs()->mcAlert().messageTT() == 9024,
               "read-only save: MCAlert TT 9024, the pause page stays parked (+0x77 set)");
        o.keyDown(53);  // No (Back) on the alert
        for (int i = 0; i < 10 && o.open(); ++i) o.update(0.1f);
        expect(!m.alertUp() && !o.open() && !o.savingOnClose(), "No: the alert closes, the storage is idle, the menu closes");
        ::chmod(save.c_str(), 0644);
        o.update(0.3f), o.takeBackdropEvents();
    }
    // 0x444400 -> 0x539ac0: no pause page while an RTC runs; the key goes on (the RTC's PAUSE skip).
    o.setRtcGate(true, true);
    const bool used = o.keyDown(53);
    expect(!used && !o.open(), "RTC running: Escape opens nothing and is not used (0x539ac0)");
    o.setRtcGate(false, false);
    expect(o.keyDown(53) && o.open(), "no RTC: Escape opens the pause page");
    o.update(0.016f);
    expect(o.takeBackdropEvents() == GameOverlay::Opened, "the pause page reports vf07 again");
    std::printf("pause check: %s (%d failures)\n", bad ? "FAILED" : "ok", bad);
    ::unsetenv("FUEL_NO_AUDIO");
    return bad;
}
// ---- end PK17

}  // namespace fuel::ui
