// PK26: FUEL's boot screens, the MenuManager_G dialogs of CommonComponent/resrce.tsc that run before the career world
// (decoded from FUEL.exe; notes ~/re-work/fuel/notes/pk26.md, p98/b037-b039, p75-work/E.md):
//   boot open 0x589930 (mode 0: black fade colour, dialog 10) -> LegalText (FOND_LEGAL + TT 16032, >= 6.0 s 0x58f980)
//   -> CompileShader (Background; the port compiles no shaders) -> Profile (0.1 s, 0x593020) -> PressStart (TT 1218 / 230,
//   pulse (cos 5t + 1) / 2 0x592ce0, input 0x591ae0) -> TRCMessage (TT 231, Continue after 3.0 s 0x5948d0, save icon at
//   (0.5, 0.55)) -> 0x594a50 (PlayLevel USA1: the career world).
// Transitions go through 0x58a470 (the leaving dialog closes, MenuManager waits 0.366 s 0x589ea0, then opens the next);
// widgets animate with CGuiSmoothAnimation_G (0x596ce0: exponential smoothing at rate 1.1 (dialogs) / 100 (widgets),
// StaticText zooming in from scale 0 0x5a0320, CWindow_G PlayShow 0x5a21c0). Tags: [O] observed, [I] inferred, [port].
#pragma once

#import <Metal/Metal.h>

#include "ui/alerts.h"
#include "ui/menu_assets.hpp"

#include <array>
#include <functional>
#include <map>
#include <string>
#include <utility>
#include <vector>

namespace fuel::ui {

class UiRenderer;

// CGuiAnimations_G / CGuiSmoothAnimation_G (0x596340 ctor, 200 bytes): blocks of 10 floats {scale, 4 offsets, r, g, b, a,
// extra}: live +4, shown +0x2c, hidden +0x54, target +0x7c; +0xa8 hiding, +0xa9 showing; rate +0xc4.
struct GuiAnimation {
    using Block = std::array<float, 10>;
    static constexpr Block kDefault = {1, 0, 0, 0, 0, 0.5f, 0.5f, 0.5f, 1, 0};
    Block live = kDefault, shown = kDefault, hidden = kDefault, target = kDefault;
    bool showing = false, hiding = false;
    float rate = 1.1f;
    void show();                 // 0x596690: live := hidden (unless +0xab), target := shown
    void hide();                 // 0x596630: the reverse
    bool step(float dt);         // 0x596500 + 0x596ce0: true when it reached the target this frame
    bool playing() const { return showing || hiding; }
    static bool atTarget(const Block& a, const Block& b);  // 0x596900 / 0x5969a0
};

class BootScreens {
public:
    // MenuManager_G dialog indices (vf24 0x589180 order).
    enum Dialog : int { None = -1, Background = 0, PressStart = 2, CompileShader = 3, Profile = 4, LegalText = 10, TRCMessage = 11 };
    struct Input {
        bool validate = false;  // MenuManager +0x1426 (Enter / pad A)
        bool back = false;      // +0x1429 (Escape / pad B)
        bool start = false;     // +0x142a (pad Start)
        bool any = false;       // 0x58a9e0: any input this frame (the idle timer)
        float mouseX = -1, mouseY = -1;  // [0xa7c0c0] +0xc / +0x10, 0..1 of the screen (-1: none)
    };

    // `first`: LegalText for the boot (0x589930 mode 0); PressStart for a return to the title [port].
    BootScreens(const std::string& dataDir, const MenuAssets& assets, std::function<std::string(uint32_t)> textOf, Dialog first = LegalText);

    // One frame: MenuManager_G vf17/0x589ea0 and the dialogs' vf23. `padMode` = PaddleChecker_G +0x24 (an XInput pad).
    void update(float dt, const Input& in, bool padMode);
    // MenuManager_G vf19 0x588b30 (fade quad, dialogs in index order) and LoadingDraw's save icon (0x5035c0) while
    // TRCMessage forces it on. `saveIcon` from loadExeImage; `spin` = LoadingDraw +0x38.
    void draw(UiRenderer& r, id<MTLTexture> saveIcon, float spin) const;

    bool launch() const { return launch_; }          // 0x594a50 ran and its 0.2 s fade ended: start the career world
    bool active() const { return !launch_; }
    bool musicOn() const { return musicOn_; }        // MusicManager +0x20 cleared (PressStart enter 0x591b90)
    int takeSound() { return std::exchange(sound_, -1); }  // 0x55a1e0([0xa02800]) = the CONFIRM menu sound (index 3), -1 none
    Dialog current() const { return Dialog(current_); }  // MenuManager +0x15e4
    int dialogState(Dialog d) const;                 // +0x3c
    float dialogTime(Dialog d) const;                // +0x84
    int menuState() const { return mmState_; }       // +0x15b4
    float trcTimer() const { return trcTimer_; }     // TRCMessage +0xe0
    bool continueShown() const;                      // TRCMessage child PressX (0x9c) visible
    bool saveIconOn() const { return saveIcon_; }    // LoadingDraw +0x35 (TRCMessage vf23 / vf20)
    float hover() const { return hover_; }           // PressStart +0x114 (0x5927b0; no reader in FUEL.exe)
    float pulseAlpha() const;                        // 0x592ce0's alpha now
    uint32_t pressStartText() const { return pressText_; }  // 230 / 1218
    float idleTimer() const { return idle_; }        // MenuManager +0x15fc
    // --menu-shot / checks: show `d` as if it had been open `t` seconds (TRCMessage: its timer runs down from 3.0).
    void preview(Dialog d, float t, bool padMode);

private:
    struct Widget {
        const DialogWindow* def = nullptr;
        int id = 0;            // +0x3c: running index in the dialog (resrce.tsc order, from 2) [I]
        bool visible = true;   // +0x4d
        bool text = false;     // CStaticText_G
        GuiAnimation anim;     // +0x24
        Rgba fill{1, 1, 1, 1}; // StaticText colour +0x94..+0xa0 (0x592ce0 writes it)
        bool fillSet = false;
        uint32_t tt = 0;       // +0xe0
        // typewriter (+0xf0 mode, +0xf4 chars, +0xf8 0.2 s a char, +0xfc timer, +0x100 done): no live setter of +0xf0 on PC
        bool typewriter = false, typeDone = true;
        float typeTime = 0, typeRate = 0.2f;
        std::vector<Widget> children;
    };
    struct Dlg {
        const DialogDef* def = nullptr;
        int state = 0;          // +0x3c
        float stateTime = 0;    // +0x40
        float time = 0;         // +0x84
        int back = -1;          // +0x6c
        GuiAnimation anim;      // +0x24 (rate 1.1, 0x4475d0)
        std::vector<Widget> widgets;
    };
    void go(int index, bool keepBack);          // 0x58a470
    void setMenuState(int s);                   // 0x589c40
    void openDialog(int index);                 // dialog -> state 1 (vf19 enter + show animation)
    void dialogFrame(int index, float dt, const Input& in, bool padMode);  // vf23
    void enter(int index, bool padMode);        // vf19 per class
    void leave(int index);                      // vf20 per class
    void input(int index, const Input& in, bool padMode);  // vf15 per class
    void showWidget(Widget& w);                 // 0x5a1560 + PlayShow 0x5a21c0
    void hideWidget(Widget& w);
    Widget* find(int dialog, const std::string& name);
    void stepWidgets(std::vector<Widget>& ws, float dt);
    void drawWidgets(UiRenderer& r, const std::vector<Widget>& ws, float x, float y, float w, float h) const;
    void build(Widget& w, const DialogWindow& d, int& id);

    const MenuAssets& assets_;
    std::function<std::string(uint32_t)> textOf_;
    DialogScript script_;
    std::map<int, Dlg> dialogs_;
    int mmState_ = 0, current_ = -1, leaving_ = -1;
    float mmTimer_ = 0;
    Rgba fade_{0, 0, 0, 1};     // MenuManager +0x15c0..+0x15cc
    float launchFade_ = -1;     // 0x438190 fade out 0.2 s ([0x9dbfa8]) after 0x594a50; -1 none
    bool launch_ = false;
    bool musicOn_ = false;
    int sound_ = -1;
    float legalTimer_ = 0;      // LegalText +0xe0
    float profileTimer_ = 0;    // Profile +0xe4
    float trcTimer_ = 3.0f;     // TRCMessage +0xe0 (ctor 3.0; not reset by its enter)
    bool saveIcon_ = false;
    float hover_ = 0;           // PressStart +0x114
    bool pressBlocked_ = false; // PressStart +0x100
    float idle_ = 17.0f;        // MenuManager +0x15fc
    bool idleOn_ = false;       // +0x1600
    uint32_t pressText_ = 230;
    bool helpPad_ = false;      // the pad mode of the frame (for the enter of a dialog opened by MenuManager)
};

// PK26 --menu-check: the boot flow driven frame by frame (timings, inputs, texts, animation maths). Returns failures.
int runBootCheck(const std::string& dataDir, const std::string& language);

}  // namespace fuel::ui
