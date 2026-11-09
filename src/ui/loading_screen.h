// FUEL's loading screen and level transitions (LoadingDraw_C, FUEL.exe vt 0x9adfd4: ctor 0x502060, update 0x502580,
// draw 0x502710), see ~/re-work/fuel/notes/p75-work/A.md sec. 2 and notes/p78-ui-gaps.md:
// - the loading screen 0x502b70: the level's LOADING_BITMAP over the canvas, a rotating tip 0x5032e0 and "LOADING..."
//   0x503ce0 (dots at 5 Hz);
// - the busy spinner 0x4feed0 and the save icon 0x5035c0;
// - the FUEL-logo stencil 0x503860 (FUEL_POCHOIR_LOAD / _ALPHA) that closes and opens around a teleport (LevelManipulator
//   0x57e650 / 0x57e0f0 timers), e.g. a race start;
// - LoadingWindow: the --in-game process's window while the world loads (the main thread is busy, so it draws from its
//   own thread into a CAMetalLayer).
#pragma once

#import <AppKit/AppKit.h>
#import <Metal/Metal.h>

#include "ui/menu_assets.hpp"
#include "ui/menu_model.hpp"

#include <atomic>
#include <cstdint>
#include <memory>
#include <random>
#include <string>
#include <thread>
#include <vector>

namespace fuel::ui {

class UiRenderer;

// Tips TT 4150.. (0x503110), one every 5 s at random among those not shown in the last n-1 picks (0x503490 / 0x503200).
class LoadingTips {
public:
    void build(const TextTable& text);
    void update(float dt);
    uint32_t current() const { return current_; }  // 0 = none

private:
    void pick();
    std::vector<uint32_t> ids_;
    std::vector<float> ages_;
    uint32_t current_ = 0;
    float timer_ = 5.0f;  // +0x278 starts at 5: the first tip shows at once
    std::mt19937 rng_{std::random_device{}()};
};

// "LOADING..." 0x503ce0: `word` (TT 300) + 0..3 dots cycling at 5 Hz with `t` (global time), right at 92.5 % of the canvas.
void drawLoadingWord(UiRenderer& r, const MenuAssets& a, const std::string& word, double t);
// The loading screen 0x502b70 (`tip` empty = none).
void drawLoadingScreen(UiRenderer& r, const MenuAssets& a, const BitmapRef& picture, const std::string& tip, const std::string& word, double t);
// Busy spinner 0x4feed0 at (cx, cy) x the viewport, outer radius rW x W; `angle` = LoadingDraw +0x38 (1 rad/s, wrapped).
void drawSpinner(UiRenderer& r, float cx, float cy, float rW, float angle);
// Pictures FUEL.exe packs itself (RLE, decoded by Mem::CopyBlock 0x504170), read from the player's XFinalComponent/FUEL.exe:
// the 64 x 64 save icon (LoadingDraw +0x50, VA 0x9a0584) and the 256 x 128 FUEL logo of the boot loading screen (+0x4c,
// VA 0x99673c, 0x504250: near-black pixels transparent). nil when the exe is missing.
enum class ExeImage { SaveIcon, BootLogo };
id<MTLTexture> loadExeImage(id<MTLDevice> device, const std::string& dataDir, ExeImage which);
// Save icon 0x5035c0 (`angle` as above, `orange` = +0x48: the storage is loading rather than saving).
void drawSaveIcon(UiRenderer& r, id<MTLTexture> icon, float angle, bool orange);
// PK26: the same at a custom centre (LoadingDraw +0x280 / +0x284 >= 0: x W, y H; TRCMessage sets (0.5, 0.55), 0x5948d0).
void drawSaveIcon(UiRenderer& r, id<MTLTexture> icon, float angle, bool orange, float cx, float cy);
// The boot ("CD") loading draw, 0x502710 step 1 (ForceCDLoadingDraw ON in usergame.tsc): the logo W/4 wide at
// (W/2, 0.4 H), the spinner at (0.5, 0.55) radius 0.02 W and "LOADING...".
void drawBootLoading(UiRenderer& r, const MenuAssets& a, id<MTLTexture> logo, const std::string& word, double t, float angle);

// The teleport timers that drive the logo stencil (LevelManipulator +0x54 p, +0x58 q, +0x5c state).
class LogoTransition {
public:
    // 0x57e650: p = 0, q = -1, state 1. PK15: `extra` = what 0x57dfc0 adds to PP52 + PP50 when state 3 starts: 0.25
    // ([0x9dbfb0], 0x57e0f0) for a teleport, 1.0 for a race start (0x6333b0 / 0x632670 re-arm it while PREPARING).
    void start(float extra = 0.25f);
    // 0x57e0f0 per frame; `ready` = the world around the target is streamed (PK15: the streamer and the terrain workers
    // are idle, 0x57f050 / GenWorld +0x5b4b4 / 0x748ef0).
    void update(float dt, bool ready = true);
    bool active() const { return state_ != 0; }
    int state() const { return state_; }  // PK15: +0x5c (1 closing / waiting, 2 streaming, 3 opening)
    bool holding() const { return state_ == 1 || state_ == 2; }  // the logo covers the screen; the target is not shown yet
    float p() const { return p_; }
    float q() const { return q_; }
    // 0x503860 (+ the "LOADING..." it draws while the picture shows).
    void draw(UiRenderer& r, const MenuAssets& a, const std::string& word, double t) const;

private:
    int state_ = 0;
    float p_ = 0, q_ = -1, frames_ = 0, stable_ = 0;
    float extra_ = 0.25f;  // PK15
};

// The --in-game loading window: FUEL's loading screen for the world level (DATAS/LWYOMING's LOADING_BITMAP), tips and
// "LOADING...", drawn by its own thread until close(). Falls back to a plain "Loading…" label without the game data.
class LoadingWindow {
public:
    enum Mode { World, Boot };  // World: 0x502b70 with the world level's picture; Boot: drawBootLoading (the menu level)
    LoadingWindow(const std::string& dataDir, const std::string& language, NSSize size, Mode mode);
    ~LoadingWindow();
    void close();

private:
    NSWindow* window_ = nil;
    std::thread thread_;
    std::atomic<bool> stop_{false};
    std::atomic<int> ready_{0};  // 1: the thread drew its first frame, 2: it gave up (the label stays)
};

}  // namespace fuel::ui
