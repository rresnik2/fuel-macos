// [port] DEVIATION (user request 2026-11-06): real Xbox / PlayStation button prompts. See pad_prompts.hpp and
// ~/re-work/fuel/notes/pad-ux.md.
#include "ui/pad_prompts.hpp"

#include "core/input_map.hpp"

#include <cctype>
#include <cstdio>

namespace fuel::ui::pad {
namespace {

std::string lower(std::string s) {
    for (char& c : s) c = char(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

constexpr uint32_t kWhite = 0xffffffff, kDark = 0x141416ff, kGrey = 0x5a5a5eff;
// Xbox face colours (A green, B red, X blue, Y yellow), close to FUEL's own SMALL_FONT pad icons.
constexpr uint32_t kGreen = 0x5cb22eff, kRed = 0xd8261eff, kBlue = 0x2f6fdcff, kYellow = 0xf2b519ff;

uint32_t letterColour(const std::string& sym) {
    if (sym.rfind("a.", 0) == 0) return kGreen;
    if (sym.rfind("b.", 0) == 0) return kRed;
    if (sym.rfind("x.", 0) == 0) return kBlue;
    if (sym.rfind("y.", 0) == 0) return kYellow;
    return kWhite;
}

Look sym(const char* s, uint32_t p = kDark, uint32_t q = kWhite, const char* s2 = "", float scale = 1) {
    Look l;
    l.symbol = s, l.symbol2 = s2, l.primary = p, l.secondary = q, l.scale = scale;
    return l;
}
constexpr float kCap = 1.3f;    // shoulder / trigger caps: FUEL's LB / RB are nearly as tall as its A
constexpr float kTilt = 0.9f;   // the stick tops with their arrows: FUEL's LS arrows are smaller than its A

}  // namespace

Style styleFromSetting(const std::string& v0) {
    const std::string v = lower(v0);
    if (v == "playstation" || v == "ps" || v == "dualsense" || v == "dualshock") return Style::PlayStation;
    if (v == "xbox") return Style::Xbox;
    return Style::Auto;
}

const char* settingOf(Style s) { return s == Style::PlayStation ? "playstation" : s == Style::Xbox ? "xbox" : "auto"; }
const char* familyName(Family f) { return f == Family::PlayStation ? "PlayStation" : "Xbox"; }

Family resolve(Style s, bool playStationPad) {
    if (s == Style::PlayStation) return Family::PlayStation;
    if (s == Style::Xbox) return Family::Xbox;
    return playStationPad ? Family::PlayStation : Family::Xbox;
}

int iconForGlyph(int glyph) {
    using namespace fuel::input;
    switch (glyph) {
        case 1: return PadA;               // CROSS
        case 2: return PadB;               // CIRCLE
        case 3: return PadX;               // SQUARE
        case 4: return PadY;               // TRIANGLE
        case 5: return LStickVertical;     // UPDOWN
        case 6: return LStickHorizontal;   // LEFTRIGHT
        case 7: return LStickAny;          // DIRPAD
        case 8: return PadLB;              // L1
        case 9: return PadRB;              // R1
        case 10: return PadLT;             // L2
        case 11: return PadRT;             // R2
        case 12: return PadLStick;         // L3
        case 13: return PadRStick;         // R3
        case 14: return PadLYPos;          // UP    (FUEL's Xbox font: the left stick with an up arrow)
        case 15: return PadLYNeg;          // DOWN
        case 16: return PadLXNeg;          // LEFT
        case 17: return PadLXPos;          // RIGHT
        case 18: return PadBack;           // SELECT
        case 19: return PadStart;          // START
        default: return -1;
    }
}

std::string iconText(int icon) {
    if (icon < 0 || icon >= kIconCount) return std::string();
    const uint32_t c = kCodeBase + uint32_t(icon);
    return std::string{char(c >> 16), char((c >> 8) & 0xff), char(c & 0xff)};
}

Look look(Family f, int icon) {
    using namespace fuel::input;
    const bool ps = f == Family::PlayStation;
    switch (icon) {
        // Face buttons: PlayStation's monochrome shapes in a ring; Xbox's coloured discs with dark letters.
        case PadA: return ps ? sym("xmark.circle", kWhite, kWhite) : sym("a.circle.fill", kDark, kGreen);
        case PadB: return ps ? sym("circle.circle", kWhite, kWhite) : sym("b.circle.fill", kDark, kRed);
        case PadX: return ps ? sym("square.circle", kWhite, kWhite) : sym("x.circle.fill", kDark, kBlue);
        case PadY: return ps ? sym("triangle.circle", kWhite, kWhite) : sym("y.circle.fill", kDark, kYellow);
        // Shoulders and triggers: white caps with dark letters (FUEL's LB / RB look).
        case PadLB: return sym(ps ? "l1.rectangle.roundedbottom.fill" : "lb.rectangle.roundedbottom.fill", kDark, kWhite, "", kCap);
        case PadRB: return sym(ps ? "r1.rectangle.roundedbottom.fill" : "rb.rectangle.roundedbottom.fill", kDark, kWhite, "", kCap);
        case PadLT: return sym(ps ? "l2.rectangle.roundedtop.fill" : "lt.rectangle.roundedtop.fill", kDark, kWhite, "", kCap);
        case PadRT: return sym(ps ? "r2.rectangle.roundedtop.fill" : "rt.rectangle.roundedtop.fill", kDark, kWhite, "", kCap);
        case PadLStick: return sym("l.joystick.press.down.fill");
        case PadRStick: return sym("r.joystick.press.down.fill");
        // BACK / START: Xbox View / Menu discs; the DualSense's Create / Options buttons are both capsule.portrait in
        // GameController, so they are drawn as pills with their names (as PlayStation games print them).
        case PadBack: {
            if (!ps) return sym("rectangle.fill.on.rectangle.fill.circle.fill");
            Look l;
            l.pill = "CREATE";
            return l;
        }
        case PadStart: {
            if (!ps) return sym("line.horizontal.3.circle.fill");
            Look l;
            l.pill = "OPTIONS";
            return l;
        }
        // The d-pad: the pressed arm white, the others grey.
        case PadDLeft: return sym("dpad.left.fill", kWhite, kGrey);
        case PadDRight: return sym("dpad.right.fill", kWhite, kGrey);
        case PadDUp: return sym("dpad.up.fill", kWhite, kGrey);
        case PadDDown: return sym("dpad.down.fill", kWhite, kGrey);
        case DpadAny: return sym("dpad.fill", kWhite, kWhite);
        // Stick directions: the stick seen from above with its arrow (FUEL's own LS arrows); both arrows for UPDOWN /
        // LEFTRIGHT (two symbols on the same box: they share the ring and the letter).
        case PadLXNeg: return sym("l.joystick.tilt.left.fill", kWhite, kGrey, "", kTilt);
        case PadLXPos: return sym("l.joystick.tilt.right.fill", kWhite, kGrey, "", kTilt);
        case PadLYPos: return sym("l.joystick.tilt.up.fill", kWhite, kGrey, "", kTilt);
        case PadLYNeg: return sym("l.joystick.tilt.down.fill", kWhite, kGrey, "", kTilt);
        case PadRXNeg: return sym("r.joystick.tilt.left.fill", kWhite, kGrey, "", kTilt);
        case PadRXPos: return sym("r.joystick.tilt.right.fill", kWhite, kGrey, "", kTilt);
        case PadRYPos: return sym("r.joystick.tilt.up.fill", kWhite, kGrey, "", kTilt);
        case PadRYNeg: return sym("r.joystick.tilt.down.fill", kWhite, kGrey, "", kTilt);
        case LStickVertical: return sym("l.joystick.tilt.up.fill", kWhite, kGrey, "l.joystick.tilt.down.fill", kTilt);
        case LStickHorizontal: return sym("l.joystick.tilt.left.fill", kWhite, kGrey, "l.joystick.tilt.right.fill", kTilt);
        case LStickAny: return sym("l.joystick.fill");
        case RStickAny: return sym("r.joystick.fill");
        default: return Look{};
    }
}

std::string shortName(Family f, int icon) {
    using namespace fuel::input;
    const bool ps = f == Family::PlayStation;
    switch (icon) {
        case PadA: return ps ? "Cross" : "A";
        case PadB: return ps ? "Circle" : "B";
        case PadX: return ps ? "Square" : "X";
        case PadY: return ps ? "Triangle" : "Y";
        case PadLB: return ps ? "L1" : "LB";
        case PadRB: return ps ? "R1" : "RB";
        case PadLT: return ps ? "L2" : "LT";
        case PadRT: return ps ? "R2" : "RT";
        case PadLStick: return ps ? "L3" : "LS";
        case PadRStick: return ps ? "R3" : "RS";
        case PadBack: return ps ? "Create" : "View";
        case PadStart: return ps ? "Options" : "Menu";
        case PadDLeft: return "D-pad Left";
        case PadDRight: return "D-pad Right";
        case PadDUp: return "D-pad Up";
        case PadDDown: return "D-pad Down";
        case DpadAny: return "D-pad";
        case PadLXNeg: return ps ? "Left stick Left" : "LS Left";
        case PadLXPos: return ps ? "Left stick Right" : "LS Right";
        case PadLYPos: return ps ? "Left stick Up" : "LS Up";
        case PadLYNeg: return ps ? "Left stick Down" : "LS Down";
        case PadRXNeg: return ps ? "Right stick Left" : "RS Left";
        case PadRXPos: return ps ? "Right stick Right" : "RS Right";
        case PadRYPos: return ps ? "Right stick Up" : "RS Up";
        case PadRYNeg: return ps ? "Right stick Down" : "RS Down";
        case LStickVertical: return ps ? "Left stick Up/Down" : "LS Up/Down";
        case LStickHorizontal: return ps ? "Left stick Left/Right" : "LS Left/Right";
        case LStickAny: return ps ? "Left stick" : "LS";
        case RStickAny: return ps ? "Right stick" : "RS";
        default: return std::string();
    }
}

State& state() {
    static State s;
    return s;
}

bool setState(const State& n) {
    State& s = state();
    const bool changed = n.present != s.present || n.family != s.family || n.forced != s.forced || n.padName != s.padName ||
                         n.liveSymbol != s.liveSymbol || n.liveName != s.liveName;
    if (!changed) return false;
    const int serial = s.serial;
    s = n;
    s.serial = serial + 1;
    return true;
}

std::string buttonName(int icon) {
    const State& s = state();
    if (!s.forced)
        if (auto it = s.liveName.find(icon); it != s.liveName.end()) {
            std::string n = it->second;
            const std::string suffix = " Button";
            if (n.size() > suffix.size() && n.compare(n.size() - suffix.size(), suffix.size(), suffix) == 0) n.resize(n.size() - suffix.size());
            if (!n.empty() && n.size() <= 8) return n;
        }
    return shortName(s.family, icon);
}

Look currentLook(int icon) {
    const State& s = state();
    Look l = look(s.family, icon);
    if (s.forced || icon < 0 || icon > fuel::input::PadRT) return l;  // only plain buttons take the pad's own symbol
    auto it = s.liveSymbol.find(icon);
    if (it == s.liveSymbol.end() || it->second.empty()) return l;
    std::string live = it->second;
    if (icon <= fuel::input::PadY) {
        if (s.family == Family::PlayStation) {
            l.symbol = live;  // the ringed shape as the pad reports it
        } else {
            if (live.size() < 5 || live.compare(live.size() - 5, 5, ".fill") != 0) live += ".fill";
            l.symbol = live, l.primary = kDark, l.secondary = letterColour(live);
        }
    } else {
        if (live.size() < 5 || live.compare(live.size() - 5, 5, ".fill") != 0) live += ".fill";
        l.symbol = live;
    }
    return l;
}

int runPromptCheck() {
    int bad = 0;
    auto expect = [&](bool ok, const std::string& what) {
        if (!ok) ++bad, std::printf("pad prompts: FAIL %s\n", what.c_str());
    };
    using namespace fuel::input;
    // FUEL's glyph names -> the inputs FUEL's own Xbox font draws for them.
    const int want[20] = {-1, PadA, PadB, PadX, PadY, LStickVertical, LStickHorizontal, LStickAny, PadLB, PadRB,
                          PadLT, PadRT, PadLStick, PadRStick, PadLYPos, PadLYNeg, PadLXNeg, PadLXPos, PadBack, PadStart};
    int okGlyphs = 0;
    for (int g = 0; g < 20; ++g) okGlyphs += iconForGlyph(g) == want[g];
    expect(okGlyphs == 20 && iconForGlyph(20) == -1, "FUEL glyph -> pad input table (" + std::to_string(okGlyphs) + "/20)");
    int okText = 0;
    for (int i = 0; i < kIconCount; ++i) {
        const std::string t = iconText(i);
        const uint32_t code = t.size() == 3 ? (uint32_t(uint8_t(t[0])) << 16) | (uint32_t(uint8_t(t[1])) << 8) | uint8_t(t[2]) : 0;
        okText += t.size() == 3 && iconOfCode(code) == i && uint8_t(t[0]) == 0xEE;
    }
    expect(okText == kIconCount && iconText(-1).empty() && iconText(kIconCount).empty() && iconOfCode('A') < 0,
           "icon text round trip (U+E000 + id), " + std::to_string(okText) + "/" + std::to_string(int(kIconCount)));
    expect(styleFromSetting("") == Style::Auto && styleFromSetting("AUTO") == Style::Auto && styleFromSetting("PlayStation") == Style::PlayStation &&
               styleFromSetting("ps") == Style::PlayStation && styleFromSetting("xbox") == Style::Xbox && styleFromSetting("nonsense") == Style::Auto,
           "pad_glyphs spellings (auto default)");
    expect(resolve(Style::Auto, true) == Family::PlayStation && resolve(Style::Auto, false) == Family::Xbox &&
               resolve(Style::Xbox, true) == Family::Xbox && resolve(Style::PlayStation, false) == Family::PlayStation,
           "family: auto follows the pad, a forced style wins");
    int looks = 0;
    for (Family f : {Family::Xbox, Family::PlayStation})
        for (int i = 0; i < kIconCount; ++i) {
            const Look l = look(f, i);
            looks += (!l.symbol.empty() || !l.pill.empty()) && !shortName(f, i).empty();
        }
    expect(looks == 2 * kIconCount, "every icon has a picture and a name in both families (" + std::to_string(looks) + ")");
    expect(look(Family::PlayStation, PadA).symbol == "xmark.circle" && look(Family::PlayStation, PadB).symbol == "circle.circle" &&
               look(Family::PlayStation, PadX).symbol == "square.circle" && look(Family::PlayStation, PadY).symbol == "triangle.circle",
           "PlayStation faces: xmark / circle / square / triangle .circle");
    expect(look(Family::Xbox, PadA).secondary == kGreen && look(Family::Xbox, PadB).secondary == kRed && look(Family::Xbox, PadX).secondary == kBlue &&
               look(Family::Xbox, PadY).secondary == kYellow,
           "Xbox faces: A green, B red, X blue, Y yellow");
    expect(look(Family::PlayStation, PadLB).symbol.rfind("l1.", 0) == 0 && look(Family::Xbox, PadRT).symbol.rfind("rt.", 0) == 0 &&
               look(Family::PlayStation, PadStart).pill == "OPTIONS" && look(Family::PlayStation, PadBack).pill == "CREATE",
           "shoulders / triggers by family, PlayStation Options / Create pills");
    expect(shortName(Family::PlayStation, PadA) == "Cross" && shortName(Family::Xbox, PadA) == "A" && shortName(Family::PlayStation, PadStart) == "Options" &&
               shortName(Family::Xbox, PadBack) == "View" && shortName(Family::Xbox, PadLB) == "LB" && shortName(Family::PlayStation, PadRT) == "R2",
           "short names");
    // Live values: names without " Button" when short, else the table; the pad's own face symbols in the family style.
    const State saved = state();
    State s;
    s.present = true, s.family = Family::PlayStation;
    s.liveName = {{PadA, "Cross Button"}, {PadStart, "Options Button"}, {PadLB, "Left Bumper Long"}};
    s.liveSymbol = {{PadA, "xmark.circle"}};
    const int serial0 = state().serial;
    expect(setState(s) && state().serial == serial0 + 1 && !setState(s), "state: serial moves on change only");
    expect(buttonName(PadA) == "Cross" && buttonName(PadStart) == "Options" && buttonName(PadLB) == "L1", "live names (" + buttonName(PadLB) + ")");
    s.family = Family::Xbox, s.liveSymbol = {{PadA, "b.circle"}}, s.liveName.clear();  // a Switch-style pad: B on the south button
    setState(s);
    expect(currentLook(PadA).symbol == "b.circle.fill" && currentLook(PadA).secondary == kRed, "live face symbol in the Xbox style (Switch B -> red)");
    s.forced = true, setState(s);
    expect(currentLook(PadA).symbol == "a.circle.fill", "a forced family ignores the pad's own symbols");
    setState(saved);
    std::printf("pad prompts: %d icons, %s\n", int(kIconCount), bad ? "FAILED" : "ok");
    return bad;
}

}  // namespace fuel::ui::pad
