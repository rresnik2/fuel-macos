// [port] DEVIATION from FUEL (user request 2026-11-06, notes ~/re-work/fuel/notes/pad-ux.md): the on-screen button
// prompts show the connected controller's own buttons. FUEL PC draws its Xbox pad icons (SMALL_FONT 1..19) for every
// pad; the port picks the controller family from GameController (GCDualSenseGamepad / GCDualShockGamepad = PlayStation,
// GCXboxGamepad and anything else = Xbox style), or from settings.txt `pad_glyphs = auto | playstation | xbox`, and draws
// Apple's SF Symbols for the bound pad input (ui/pad_glyphs.mm rasterizes them at run time; no image files).
//
// This part is plain C++ (the fuelui library, the checks): the icon ids, their text form inside FUEL's strings, the
// FUEL glyph -> pad input table, the per-family symbol / name tables and the process-wide prompt state.
//
// Icons are the physical pad inputs of WinInput_Z (fuel::input::Pad, 0..23: A B X Y LB RB LT RT LSTICK RSTICK BACK
// START, d-pad LEFT RIGHT UP DOWN, the stick half-axes) plus five compound ones FUEL's glyph names ask for (DIRPAD,
// UPDOWN, LEFTRIGHT; the right stick and the d-pad as a whole). In text an icon is one private-use character
// (U+E000 + id, 3 UTF-8 bytes); ui/ui_renderer.mm draws it from the atlas with the font's old pad-icon size and baseline.
#pragma once

#include <cstdint>
#include <map>
#include <string>

namespace fuel::ui::pad {

enum class Family { Xbox, PlayStation };
enum class Style { Auto, PlayStation, Xbox };  // settings.txt pad_glyphs

Style styleFromSetting(const std::string& v);  // "auto" (default) | "playstation" / "ps" | "xbox"
const char* settingOf(Style s);                // "auto" | "playstation" | "xbox"
const char* familyName(Family f);              // "PlayStation" | "Xbox"
// The family the prompts use: a forced style wins; auto follows the pad (PlayStation pads -> PlayStation, else Xbox).
Family resolve(Style s, bool playStationPad);

enum Icon : int {
    // 0..23 = fuel::input::Pad
    LStickAny = 24,      // FUEL DIRPAD (the left stick, any direction)
    LStickVertical,      // FUEL UPDOWN
    LStickHorizontal,    // FUEL LEFTRIGHT
    RStickAny,
    DpadAny,
    kIconCount
};

// FUEL's glyph index (0xa675c8: 1 CROSS .. 19 START) -> the pad input it stands for in FUEL's own Xbox font
// (CROSS = A, CIRCLE = B, SQUARE = X, TRIANGLE = Y, L1 = LB .. R3 = RSTICK, UP..RIGHT = the left stick's arrows,
// SELECT = BACK, START = START); -1 for EMPTY / unknown.
int iconForGlyph(int glyph);

// Text form of an icon (U+E000 + icon) and back (codes as ui::utf8Codes gives them: the UTF-8 bytes big-endian).
std::string iconText(int icon);
constexpr uint32_t kCodeBase = 0xEE8080;  // U+E000 = EE 80 80
inline int iconOfCode(uint32_t c) { return c >= kCodeBase && c < kCodeBase + uint32_t(kIconCount) ? int(c - kCodeBase) : -1; }

// The picture of an icon for a family: up to two SF Symbols drawn on top of each other in the same box (the second one
// is "" for most), the palette (primary, secondary) and, when the symbol is a text pill, its caption.
struct Look {
    std::string symbol, symbol2;  // SF Symbol names
    uint32_t primary = 0xffffffff, secondary = 0xffffffff;  // 0xRRGGBBAA, sRGB (palette rendering)
    std::string pill;             // non-empty: a rounded pill with this caption instead of a symbol
    float scale = 1;              // the symbol's size against a face button (FUEL's own icons: wide shoulder caps, small stick tops)
};
Look look(Family f, int icon);
// The short name of a pad input as the controller prints it (Cross, Circle, Square, Triangle, L1 .. R3, Options, Create;
// A, B, X, Y, LB, RB, LT, RT, LS, RS, Menu, View; the d-pad and stick directions).
std::string shortName(Family f, int icon);

// The process's prompt state (PaddleChecker_G's pad flag + the port's family). `serial` moves on every change that
// alters a picture (the atlas rebuilds); live values are what the connected pad's GCControllerElements report.
struct State {
    bool present = false;               // a pad is connected (PaddleChecker_G +0x24)
    Family family = Family::Xbox;
    bool forced = false;                // the family comes from pad_glyphs (or $FUEL_SHOT_PAD / $FUEL_PAD_FAMILY), not the pad
    std::string padName;                // GCController.vendorName ("DualSense Wireless Controller")
    std::map<int, std::string> liveSymbol, liveName;  // per icon: sfSymbolsName / localizedName of the pad's element
    int serial = 0;
};
State& state();
// Sets the state; bumps the serial when the family, the presence or a live value changed. True when it changed.
bool setState(const State& s);
// The name of an icon now: the pad's own localizedName (without " Button") when it is short, else shortName().
std::string buttonName(int icon);
// The look of an icon now: the family's table, with the pad's own sfSymbolsName where it reports one for a plain
// button (face buttons, shoulders, triggers, sticks): Switch-style pads get their own letters this way.
Look currentLook(int icon);

// --menu-check: the tables, the text form and the family rules. Returns the failure count.
int runPromptCheck();

}  // namespace fuel::ui::pad
