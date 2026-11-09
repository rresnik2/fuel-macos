// PK25: FUEL's in-game alert dialogs (FrontGlobalInGameData_G vf24 0x442990: PADAlert, InfosText, MCAlert, IGMsgBox from
// CommonComponent/resGIGD.tsc, CWindow_G framework with 0..1 coordinates) and the MCAlert flows of the save manager
// (MemoryCardManager_C) adapted to a Mac file save. Decoded from FUEL.exe; notes/pk25.md has the addresses and the
// approximations. Tags: [O] observed, [I] inferred, [U] unknown, [port] port choice.
#pragma once

#include "ui/menu_assets.hpp"
#include "ui/menu_model.hpp"

#include <cstdint>
#include <functional>
#include <map>
#include <string>
#include <vector>

namespace fuel::ui {

class UiRenderer;

// ---- the dialog script (MENUSTyleTextStruct / MENUStyleBox / MENUDialog / MENUWindow ...) --------------------------
struct DialogTextStyle {
    float scale = 1;          // MENUSTyleTextStruct name SCALE ...
    bool big = false;         // BIG / SMALL font
    Rgba fill{1, 1, 1, 1}, outline{0, 0, 0, 1};
};
struct DialogBoxStyle {
    std::string type;         // MENUStyleBox name TYPE colour: INT_BOXBASE, MSGBOX, ...
    Rgba colour{1, 1, 1, 1};
};
struct DialogWindow {
    std::string type, name;   // StaticText, Bitmap, Box
    float x = 0, y = 0, w = 0, h = 0;  // relative to the parent window (or dialog), 0..1
    std::string textStyle, boxStyle, image, align;
    uint32_t tt = 0;
    bool outlined = false;
    Rgba colour{1, 1, 1, 1};
    std::vector<DialogWindow> children;
    // ---- PK26: MENUPropertyTextScale (+0xd8 scale of the text style), MENUPropertyTextShadowed (+0xd1), MENUPropertyColor given
    float textScale = 1;
    bool shadowed = false, hasColour = false;
    // ---- end PK26
};
struct DialogDef {
    std::string name;
    float x = 0, y = 0, w = 1, h = 1;
    std::vector<DialogWindow> windows;
};
struct DialogScript {
    std::map<std::string, DialogTextStyle> textStyles;
    std::map<std::string, DialogBoxStyle> boxStyles;
    std::vector<DialogDef> dialogs;
    std::vector<std::string> problems;
    const DialogDef* find(const std::string& name) const;
};
// Reads CommonComponent/<file> (resGIGD.tsc, resrce.tsc).
DialogScript loadDialogScript(const std::string& dataDir, const std::string& file);

// ---- Dialog_MCAlert (vtable 0x9a5034, 0xb0 bytes; built by 0x442990 with +0xa4 = 6) -----------------------------------
class McAlert {
public:
    // What the alert asks of the game around it.
    struct Host {
        std::function<void()> pause;              // 0x441670: "Pause" + the pause page set opens (when none is up)
        std::function<void()> unpause;            // 0x440b50: "UnPause" (the host applies FUEL's conditions)
        std::function<void()> autosaveOff;        // storage +0x75 = 1 (0x440d10 / 0x440ff0 No)
        std::function<void()> restartFromBuffer;  // 0x4416f0: the next restart uses RestartFromBufferSave
    };
    Host host;

    // 0x441320: save error alert. 9 / 10 / 0xc / 0xd: TT 9024 "Save failed! ... try again?" Yes/No (TT 489); 0xe: TT 9045
    // with "%d" = 0x118 (280 kB), buttons TT 9118; 0xb / 0x1b: shown with the current texts; other codes: false (none).
    bool saveError(int code);
    // 0x4411a0: load error alert. 0x17: TT 9113 (corrupt, overwrite?), 0x1c: TT 9034 (another user's save); Yes/No.
    bool loadError(int code);
    // 0x441540: "no storage device" (Xbox): TT 9038 (absent on PC) + TT 490 OK, mode 2 -> 5 (0x440ff0) [port: the
    // selector does not exist on a Mac; its "no device chosen" result (autosave disabled) is taken at once].
    bool noDevice(int arg);
    // The action keys of this frame (FUEL game+0x208 = MENU_VALID: Return / pad A, game+0x210 = MENU_BACK: Escape / pad B).
    void key(bool valid, bool back);
    // Dialog_MCAlert vf16 0x4410d0 (per frame while shown).
    void update(float dt);
    // 0x440cd0: the code (+0xa0) and the answer (+0xac); true once when the alert has closed (state 7).
    bool result(int* code, int* answer);

    bool shown() const { return shown_; }
    int code() const { return code_; }
    int state() const { return state_; }
    int answer() const { return answer_; }
    uint32_t messageTT() const { return messageTT_; }
    uint32_t buttonsTT() const { return buttonsTT_; }
    const std::string& messageText() const { return messageText_; }  // a formatted message (+0xe8), "" = use the TT
    // For --menu-shot: show with these texts (no flow).
    void preview(uint32_t message, uint32_t buttons, const std::string& text = "") {
        messageTT_ = message, buttonsTT_ = buttons, messageText_ = text, shown_ = true;
    }
    // The text of a TT id (for 0x441320 case 0xe's sprintf).
    std::function<std::string(uint32_t)> textOf;

private:
    void show();
    void close(bool keepAnswer);         // 0x440b50
    bool pressed(bool& which);           // 0x4400c0: the 0.4 s input lock
    void setMessage(uint32_t tt) { messageTT_ = tt, messageText_.clear(); }  // 0x43feb0
    void setButtons(uint32_t tt) { buttonsTT_ = tt; }                        // 0x43fee0
    void loadErrorFlow();    // 0x440f90 -> 0x440d10 / 0x440df0
    void saveErrorFlow();    // 0x440fb0
    void noDeviceFlow();     // 0x440ff0
    int code_ = 0;           // +0xa0
    int mode_ = 6;           // +0xa4 (0/1/2 arm 3/4/5 the next frame; 6 idle)
    int state_ = 0;          // +0xa8 (1 asking, 2 selector, 4/5 OK only, 7 done)
    int answer_ = 0;         // +0xac (1 save again / overwrite, 2 load again)
    float timer_ = 0;        // +0x98
    bool lock_ = false;      // +0x9c
    bool shown_ = false;     // dialog state +0x3c != 0
    bool valid_ = false, back_ = false;  // the frame's MENU_VALID / MENU_BACK
    uint32_t messageTT_ = 0, buttonsTT_ = 0;
    std::string messageText_;
};

// ---- the four dialogs and their drawing ----------------------------------------------------------------------------
class InGameDialogs {
public:
    InGameDialogs(const std::string& dataDir, const MenuAssets& assets, std::function<std::string(uint32_t)> textOf);
    const DialogScript& script() const { return script_; }
    McAlert& mcAlert() { return mc_; }
    const McAlert& mcAlert() const { return mc_; }
    // PADAlert (0x4424a0 enter: "padid" = pad + 1, button 1 TT 9019 (absent on PC), button 2 cleared). FUEL PC never
    // shows it (no show path; 0x57dd10 pauses instead) [O]; the port keeps it for --menu-shot only.
    void previewPad(int pad) { padShown_ = true, pad_ = pad; }
    void previewInfos(const std::string& text) { infosShown_ = true, infos_ = text; }  // InfosText (0x440710: dead on PC)
    void previewMsgBox() { msgBoxShown_ = true; }  // IGMsgBox (GFWL profile change 0x4401a0: dropped)
    bool anyShown() const { return mc_.shown() || padShown_ || infosShown_ || msgBoxShown_; }
    void draw(UiRenderer& r) const;

private:
    void drawDialog(UiRenderer& r, const DialogDef& d, const std::function<std::string(const DialogWindow&)>& textFor) const;
    void drawWindow(UiRenderer& r, const DialogWindow& w, float px, float py, float pw, float ph,
                    const std::function<std::string(const DialogWindow&)>& textFor) const;
    void drawBox(UiRenderer& r, const DialogBoxStyle& s, float x0, float y0, float x1, float y1) const;  // 0x439330
    std::string text(uint32_t tt) const;
    DialogScript script_;
    const MenuAssets& assets_;
    std::function<std::string(uint32_t)> textOf_;
    McAlert mc_;
    bool padShown_ = false, infosShown_ = false, msgBoxShown_ = false;
    int pad_ = 0;
    std::string infos_;
};

}  // namespace fuel::ui
