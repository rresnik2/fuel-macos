// The game's front end for `fuelview --game`, the in-game pause/results overlay and
// HUD, and the headless --menu-shot / --menu-check. A session (race or free ride) runs
// in a fresh fuelview process: the menu exec()s `fuelview --in-game ...`, and the
// in-game menu exec()s back to `fuelview --game` (or into the next session).
#pragma once

#import <Metal/Metal.h>

#include "core/settings.hpp"
#include "ui/loading_screen.h"
#include "ui/menu.h"
#include "ui/menu_input.h"  // [port] remap
#include "viewer/pause_backdrop.hpp"  // PK17

#include <array>
#include <functional>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace fuel {
class MenuSfx;  // PK18
class GaragePreview;  // PK20
}

namespace fuel::ui {

class UiRenderer;

// fuelview --menu-shot PAGE OUT.png [--size WxH]
int runMenuShot(const std::string& dataDir, const Settings& s, const std::string& page, const std::string& out, int w, int h);

// fuelview --game: title screen and menus in a window until the player starts a session
// (then this process is replaced) or quits.
int runGameMenu(const std::string& self, const std::string& dataDir, const Settings& s, const std::string& startPage);

// The data-relative DPC that holds a vehicle variant (the class archive; bonus vehicles in BONUS1.DPC).
std::string vehicleDpc(const std::string& dataDir, const Menu& menu, const std::string& vehicle);

// Replaces this process with the session `r` asks for (never returns unless exec fails).
void execSession(const std::string& self, const std::string& dataDir, const MenuRequest& r, const Menu& menu);
// The arguments execSession gives `fuelview` for `r` (the garage's choices, the rider, [upgrades U2] the vehicle's stages).
std::vector<std::string> sessionArgs(const std::string& dataDir, const MenuRequest& r, const Menu& menu);
// [upgrades U2] fuelview --upgrade-cmd "OP ...; OP ...": a dev tool (U3's garage page is the player's way) on career.sav ($FUEL_CAREER_SAVE):
//   list [VEHICLE]                      owned vehicles' stages, spent fuel, the next stages' prices and gates
//   buy VEHICLE CATEGORY STAGE           Career::buyUpgrade (stage n buys the ones below and equips it), saved
//   equip VEHICLE CATEGORY STAGE         Career::equipUpgrade (any owned stage, 0 = stock), saved
//   session VEHICLE [EVENT]              the session arguments the menu would exec (sessionArgs; free ride without EVENT)
// CATEGORY: e s t w np nt or a name (engine, tires, ...). Returns 0 when every operation did what it asked.
int runUpgradeCommand(const std::string& dataDir, const Settings& s, const std::string& spec);
// Replaces this process with the menu (`--game`, optionally opening `page`).
void execMenu(const std::string& self, const std::string& dataDir, const std::string& page = "");

// The pause / results menu and the HUD inside a session (`--in-game`).
class GameOverlay {
public:
    GameOverlay(id<MTLDevice> device, MTLPixelFormat format, const std::string& self, const std::string& dataDir, const Settings& s,
                const std::string& event, const std::string& hub, const std::string& vehicle);
    ~GameOverlay();

    bool open() const { return open_ || menu_->tutorialOpen(); }  // P56: a tutorial page pauses the session too
    void openPause();
    void openInfos();
    bool openArch();   // P75: the challenge-arch prompt's action key (0x4f6fd0 state 1 -> 3)  // P75: 0x46e750 with tab 2 (Map)
    // P75: the end-race pages are MissionHUD pages (0x4ba590), not the pause menu: the world, the end grade and VI_BIKE
    // keep running under them with the player's input neutral (Input_NeutralBlock at the top of 0x4ba590).
    bool worldRuns() const;
    // `won`: the event's rule outcome (RaceRules::Won); `level`: the race's level (race +0x3e8); the career award is
    // applied here.
    void openResults(const std::vector<ResultRow>& rows, bool won = false, int level = -1);
    // Keyboard (macOS virtual key codes); true when the overlay used the key.
    bool keyDown(unsigned short code, bool repeat = false);  // [port] remap: `repeat` = an auto-repeat (the capture ignores it)
    void keyUp(unsigned short code);  // P75: held map keys
    void pointer(float x, float y, bool click, float viewW, float viewH);
    void pollGamepad();
    // Requests the host must act on in-process; others exec() away.
    enum Action { None, Resume, Restart, Teleport };
    // Teleport: where the map sent the player (x, z).
    std::array<float, 2> teleportTarget() const { return teleport_; }
    Action update(float dt);
    // `hudShown` false: no HUD this frame (cut-scenes); the menus and the logo stencil still draw.
    void draw(id<MTLCommandBuffer> cb, id<MTLTexture> target, const HudState& hud, bool hudShown = true);
    // P78: the FUEL-logo stencil around a teleport (LevelManipulator 0x57e650 .. 0x57e0f0, LoadingDraw 0x503860). A race
    // start teleports the player: holdRaceStart() starts the stencil once per start and is true while the logo covers
    // the screen (the race and its TELEPORT_HELICO wait); the map teleport starts it too.
    bool holdRaceStart();
    bool logoHolds() const { return logo_.holding(); }
    bool logoActive() const { return logo_.active(); }
    // PK13: 0x539a50 (the TR_DEATH 0x10e hold): "LOADING..." over the game this frame.
    void setRtcLoading(bool on) { rtcLoading_ = on; }
    void startLogo() { logo_.start(); }  // PK03: TeleportToHub / the END1 TLP go through the teleport queue too (0x57e650)
    // PK15: 0x57e0f0 state 2 waits for the streaming to be idle (set before update()); the machine's state +0x5c.
    void setLogoReady(bool ready) { logoReady_ = ready; }
    int logoState() const { return logo_.state(); }
    Menu& menu() { return *menu_; }
    // PK25 0x57dd10: Game_IsPlayerFree 0x57db50 (no race running, no cut-scene) for the pad-disconnect pause; set by the host.
    void setPlayerFree(bool free) { playerFree_ = free; }
    // ---- PK26: the career boot (0x594a50 PlayLevel USA1 -> Dialog_Loading vf23 0x58fc20 on the level's first load).
    // startCareerBoot: the Loading dialog (mode 0x13: LOADING_BITMAP over a 0.25 black fade) shows for its 0.5 s (+0xe8);
    // careerBootDue() turns true once when that ran out (then CheckUnLock, TeleportToHub <saved hub>, UnPause and
    // AddCommandAfterTeleport GAMEStarted, 0x590300). gameStarted (GameManager +0xdc): the free-roam HUD lock 0x444400
    // keeps the pause menu / map shut until GAMEStarted; sessions not started by the boot are started already.
    void startCareerBoot();
    bool careerBootDue();
    bool careerBooting() const { return bootTimer_ >= 0; }
    void setGameStarted(bool on) { gameStarted_ = on; }
    bool gameStarted() const { return gameStarted_; }
    // ---- end PK26
    // ---- PK17: pause open / close semantics (notes/pk17.md)
    // InGameMenuHUD (the pause page set, PageTutorial): vf07 0x46aca0 show, vf08 0x46ade0 / InGamePauseHUD vf08 0x46e9b0
    // close (with the save-on-close wait 0x46ea50), vf10 0x46aed0 fade step (ViewportHUD 0x4c8c50, 4 per second).
    // The host applies what the overlay reports: backdropEvents() once per frame after update() (bit Opened: vf07 with
    // the Pause mode -> renderer.setPauseBackdrop(true), sounds 0x5d8e60 + mask 7, the tint saved; Closed: vf08 ->
    // setPauseBackdrop(false), mask 0; FadedOut: vf10 < 0 -> the tint restored (0x46aed0), HUDs on), and while
    // backdropShown() the tint / blur of 0x46af90 at backdropAlpha().
    enum BackdropEvent { Opened = 1, Closed = 2, FadedOut = 4 };
    int takeBackdropEvents() { return std::exchange(backdropEvents_, 0); }
    bool backdropShown() const { return pauseMode_ != 0; }  // +0x4c
    float backdropAlpha() const { return fade_.alpha; }     // +0x28
    void setBackdropHosted(bool on) { backdropHosted_ = on; }  // the host draws FUEL's backdrop: no port dim
    // 0x444400 / 0x539ac0: no pause page while an RTC runs or is requested (the race intro's TELEPORT_HELICO included);
    // 0x534b40(2): a freeze-player RTC suspends the tutorial gate.
    void setRtcGate(bool running, bool freeze) { rtcGate_ = running, menu_->setRtcFreeze(freeze); }
    bool pauseAllowed() const { return gameStarted_ && !rtcGate_; }
    bool savingOnClose() const { return waitSave_; }  // +0x98
    void stepBackdrop(float dt) { backdropFrame(dt); }  // --film (FUEL_MENU_FILM): the overlay is not update()d there
    // ---- end PK17

private:
    std::string self_, dataDir_;
    std::unique_ptr<Menu> menu_;
    std::unique_ptr<UiRenderer> ui_;
    std::unique_ptr<fuel::MenuSfx> sfx_;  // PK18
    bool open_ = false;
    unsigned padButtons_ = 0;
    PadMenuRouter padRouter_;  // [port] remap: the pad's menu keys from the bindings
    bool padPause_ = false;    // [port] remap: PAUSE_MODE's pad input held last frame
    std::array<float, 2> teleport_{};
    LogoTransition logo_;
    bool logoArmed_ = false;
    bool logoReady_ = true;  // PK15
    // PK25: the pad monitor (LevelManipulator vf17 0x57dd10): pads seen last frame, a lost pad waiting for the player to
    // be free, the host's Game_IsPlayerFree.
    int padsSeen_ = -1;
    bool padLost_ = false, playerFree_ = true;
    double clock_ = 0;  // global time for the "LOADING..." dots
    bool rtcLoading_ = false;  // PK13
    // P78: the save icon (LoadingDraw +0x35 / +0x44 / +0x38, 0x503500 / 0x5035c0).
    id<MTLTexture> saveIcon_ = nil;
    int savesSeen_ = 0;
    bool saveIconOn_ = false;
    float saveIconTime_ = 0, spinAngle_ = 0;
    // ---- PK20: the 3-D garage preview (made when the pause menu's garage first opens)
    id<MTLDevice> device_ = nil;
    std::unique_ptr<fuel::GaragePreview> garage_;
    // ---- PK20 end
    // ---- PK26
    float bootTimer_ = -1;   // Dialog_Loading +0xe8 (0.5 s), -1 = no career boot
    bool bootDue_ = false;
    bool gameStarted_ = true;
    // ---- PK17
    fuel::pause::HudFade fade_;   // ViewportHUD +0x28 / +0x2c / +0x30
    int pauseMode_ = 0;           // +0x4c: 1 = "Pause" (0 none)
    bool waitSave_ = false;       // InGamePauseHUD +0x98
    MenuRequest heldClose_;       // the request that closes the menu once the save is idle
    bool hudsOff_ = false;        // SetHudsVisibleState OFF (vf07) .. ON (vf10 < 0)
    bool backdropHosted_ = false, rtcGate_ = false;
    int backdropEvents_ = 0;
    bool wasShown_ = false;
    void backdropFrame(float dt);  // vf07 / vf08 on the overlay's open state, then vf10
    Action finishClose(const MenuRequest& r);
    // ---- end PK17
    std::unique_ptr<MenuAssets> bootAssets_;  // DATAS/LWYOMING's LOADING_BITMAP (the world level's, as the loading window)
    std::unique_ptr<UiRenderer> bootUi_;
    std::function<std::string(int)> keyLabels_;
    MTLPixelFormat format_ = MTLPixelFormatBGRA8Unorm_sRGB;
    // ---- end PK26
    // ---- pad-ux [port]: the pad prompts follow the bindings (help_texts padLabels) and the pad's family; the labels
    // are made again when the options pages change the bindings (controls layout) or pad_glyphs.
    std::function<std::string(int action, int glyph)> padLabels_;
    int labelSerial_ = -1;
    void refreshLabels();
public:
    // pad-ux: the mouse wheel / trackpad over the menu (canvas rows; > 0 = down).
    void wheel(float rows);
};

// ---- PK17: --menu-check part: the pause page set's open / close semantics on a scratch career.sav (save-on-close wait
// 0x46e9b0 / 0x46ea50 with PK25's MCAlert, fade 0x4c8c50, backdrop events, RTC gate 0x539ac0). Returns failures.
int runPauseCheck(const std::string& dataDir);
// ---- end PK17

// [port] remap: the fixed menuKeyFor / gamepadMenuKeys are gone; the menus read the bindings (ui/menu_input.h,
// Menu::menuKeys / PadMenuRouter).

}  // namespace fuel::ui
