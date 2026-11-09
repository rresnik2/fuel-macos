// PK26: FUEL's binding-aware help texts ({STR_*} text variables), decoded from FUEL.exe (notes ~/re-work/fuel/notes/pk26.md):
//  - `RemapTextAdd STR_X ACTION GLYPH GROUP` (script command 0x4e2d00, 79 lines in CommonComponent/Input/input.PC): records
//    of 0x28 bytes at g_Game +0x31f0 {char name[24], u8 glyph = 0x677c80(GLYPH), +0x1c long atol(GLYPH) or -1, +0x20
//    action index ("NONE" -> -1), +0x24 group crc (ALL = [0xa67434])};
//  - TransText 0x677dd0 -> 0x4e2820 on every loaded TT line: each record name found in the text and not already after '{'
//    becomes "{STR_X}", a LevelAgent text variable (created on first use);
//  - 0x4e2250 fills every variable: the record's pad glyph (one byte, the index in the 20-name table 0xa675c8 =
//    SMALL_FONT_PC glyph code) while an XInput pad is present (PaddleChecker_G +0x24), else the key label 0x4e27c0 of the
//    record's action: a glyph "^000<c>^000 " for the 18 DIK codes of 0x4e2580's table, else "^951" KEY "^999";
//    rebuilt by PaddleChecker_G vf17 0x4e36e0 on the first frame and whenever a pad comes or goes;
//  - the draw-time expansion 0x57bdf0 of "{NAME}" (CStaticText_G 0x5a05b0, the TSC pages).
// Tags: [O] observed, [I] inferred, [port] port choice.
#pragma once

#include <cstdint>
#include <functional>
#include <map>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace fuel::input {
class InputMap;
}

namespace fuel::ui {

class HelpTexts {
public:
    struct Record {           // 0x28 bytes at g_Game +0x31f0
        std::string name;     // +0 char[24]
        int glyph = 0xff;     // +0x18 u8 (0x677c80 of the GLYPH argument; 0xff = none)
        int tt = -1;          // +0x1c atol(GLYPH) or -1 (always -1 in input.PC)
        int action = -1;      // +0x20 Input_ActionIndexByName (fuel::input::Action order), -1 = NONE
        std::string group = "ALL";  // +0x24 (crc; kept as the name)
    };

    // 0x677c80: the index of a glyph name in the 20-entry table 0xa675c8 (EMPTY CROSS CIRCLE SQUARE TRIANGLE UPDOWN
    // LEFTRIGHT DIRPAD L1 R1 L2 R2 L3 R3 UP DOWN LEFT RIGHT SELECT START), 0xff if none.
    static int glyphIndex(const std::string& name);
    static const char* glyphName(int index);
    // 0x4e2580: the glyph name a keyboard binding (dev -1) shows instead of its key name, by DirectInput scan code
    // (RETURN -> CROSS, ESCAPE -> CIRCLE, ...); nullptr when the key has none.
    static const char* dikGlyph(int dik);
    // 0x4e27c0 for a keyboard key: "^000<c>^000 " when 0x4e2580 has a glyph for it, else "^951" + name + "^999".
    static std::string keyLabel(int dik, const std::string& keyName);

    // Reads CommonComponent/Input/input.PC once (later calls are no-ops). Returns the record count.
    size_t load(const std::string& dataDir);
    bool loaded() const;
    std::vector<Record> records() const;

    // 0x4e2820 (TransText hook): rewrites STR_X tokens of one TT line in place and creates their variables.
    void transText(std::string& line);
    // [port] the pad label of a record: `padLabel(action, glyph)` (action -1 for NONE records, glyph = the record's
    // 0xa675c8 index). Empty function: FUEL's own glyph byte.
    using PadLabelFn = std::function<std::string(int action, int glyph)>;
    // 0x4e2250: refill every variable. `pad` = an XInput pad is present (0x4e3e60 > 0); `keyLabel(action)` = 0x4e27c0 of
    // the action's keyboard binding. Returns true (0x4e2250 returns 1 once the binding sets are read).
    bool rebuild(bool pad, const std::function<std::string(int action)>& keyLabel, const PadLabelFn& padLabel = {});
    // PaddleChecker_G vf17 0x4e36e0 (+0x24 / +0x25): call every frame with the pad presence; rebuilds on the first call
    // and when it changes. Returns true when it rebuilt. [port] `variant` (the prompt family, pad_prompts.hpp) also
    // rebuilds when it changes while a pad is present.
    bool padCheck(bool pad, const std::function<std::string(int action)>& keyLabel, const PadLabelFn& padLabel = {}, int variant = 0);
    bool padMode() const;  // PaddleChecker_G +0x24 (Dialog_PressStart reads it through game+0x3218)
    // 0x57bdf0: "{NAME}" -> the variable's value. [port] an unknown name stays as written (FUEL dereferences a null record).
    std::string expand(const std::string& text) const;
    std::string value(const std::string& name) const;  // "" when no such variable
    size_t variableCount() const;
    int rebuilds() const;

private:
    mutable std::mutex m_;
    bool loaded_ = false;
    std::vector<Record> records_;
    std::map<std::string, std::string> vars_;  // LevelAgent text variables (game+0x31fc obj +0x90c/+0x910) by name
    int padState_ = -1;                        // +0x24 with +0x25 = "not built yet" as -1
    int padVariant_ = 0;                       // [port] the prompt family the labels were built for
    int rebuilds_ = 0;
};

// The process's help texts (g_Game +0x31f0 and the LevelAgent variables are game globals).
HelpTexts& helpTexts();

// [port] the keyboard labels of the port's bindings: the user's controls layout (settings.txt `controls`, `key.*`) for
// every action that has keys there, else FUEL's DftMap.map keys (the menu keys of the port follow those, game_menu.mm).
// Key names are the port's (FUEL asks DirectInput, DIPROP_KEYNAME 0x83be30), upper-cased.
std::function<std::string(int action)> keyboardLabels(const std::string& dataDir, const std::map<std::string, std::string>& settings);

// [port] DEVIATION (user request 2026-11-06, notes/pad-ux.md): the pad labels follow the bindings. A record with an
// action shows the pad input that action is bound to (InputMap::padsOf: input.PC's InputDefAdd, later the remap page's
// bindings) as a pad icon (ui/pad_prompts.hpp, drawn for the connected pad's family); a NONE record shows the input of
// its FUEL glyph (CROSS = A ...). FUEL draws the record's fixed glyph byte.
HelpTexts::PadLabelFn padLabels(const std::string& dataDir, const std::map<std::string, std::string>& settings);
HelpTexts::PadLabelFn padLabels(std::shared_ptr<const fuel::input::InputMap> bindings);

// PK26 --menu-check: decode checks of the above (records, rewrite, labels, PaddleChecker). Returns the failure count.
int runHelpTextCheck(const std::string& dataDir);

}  // namespace fuel::ui
