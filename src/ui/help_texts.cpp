// PK26: binding-aware help texts, see help_texts.hpp and ~/re-work/fuel/notes/pk26.md.
#include "ui/help_texts.hpp"

#include "core/input_map.hpp"
#include "ui/pad_prompts.hpp"
#include "ui/tsc.hpp"

#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fstream>
#include <memory>
#include <sstream>

namespace fuel::ui {
namespace {

// 0xa675c8, filled by the static initialiser 0x8e79a0 from the names at 0x9c7354, 0x9b2510, ... 0x9b2b80 [O].
const char* const kGlyphNames[20] = {"EMPTY", "CROSS", "CIRCLE", "SQUARE", "TRIANGLE", "UPDOWN", "LEFTRIGHT", "DIRPAD", "L1", "R1",
                                     "L2",    "R2",    "L3",     "R3",     "UP",       "DOWN",   "LEFT",      "RIGHT",  "SELECT", "START"};

// [port] the glyph byte as text: 10 (L2: SMALL_FONT_PC's C key cap / SMALL_FONT's LT) would be the port renderer's line
// break ('~' becomes '\n' there), so it travels as the 2-byte code C2 8A, which UiRenderer::glyphCode maps back to 10.
std::string glyphByte(int i) { return i == 10 ? std::string("\xc2\x8a") : std::string(1, char(i)); }

std::string upperOf(std::string s) {
    for (char& c : s) c = char(std::toupper(static_cast<unsigned char>(c)));
    return s;
}

}  // namespace

int HelpTexts::glyphIndex(const std::string& name) {
    // 0x677c80 compares AsoboCrc(name) with the table: the CRC is case-insensitive (0xa018f0 folds the bytes) [O].
    const std::string u = upperOf(name);
    for (int i = 0; i < 20; ++i)
        if (u == kGlyphNames[i]) return i;
    return 0xff;
}

const char* HelpTexts::glyphName(int index) { return index >= 0 && index < 20 ? kGlyphNames[index] : nullptr; }

const char* HelpTexts::dikGlyph(int dik) {
    // 0x4e2580 jump table (0x4e26ec byte index -> 0x4e26a0 targets) [O], keyboard bindings (dev -1) only.
    switch (dik) {
        case 0x01: return "CIRCLE";     // ESCAPE
        case 0x0e: return "DIRPAD";     // BACK
        case 0x11: return "LEFT";       // W
        case 0x12: return "R2";         // E
        case 0x13: return "R1";         // R
        case 0x14: return "UP";         // T
        case 0x15: return "SELECT";     // Y
        case 0x16: return "DOWN";       // U
        case 0x17: return "R3";         // I
        case 0x1c: return "CROSS";      // RETURN
        case 0x1d: return "L1";         // LCONTROL
        case 0x23: return "L3";         // H
        case 0x2d: return "RIGHT";      // X
        case 0x2e: return "L2";         // C
        case 0xc8: return "TRIANGLE";   // UP arrow
        case 0xcb: return "LEFTRIGHT";  // LEFT arrow
        case 0xcd: return "UPDOWN";     // RIGHT arrow
        case 0xd0: return "SQUARE";     // DOWN arrow
        default: return nullptr;
    }
}

std::string HelpTexts::keyLabel(int dik, const std::string& keyName) {
    // 0x4e27c0: out = ""; 0x4e2580 (glyph found and != 0xff) -> sprintf "^000%s^000 " of the one-byte glyph string;
    // else "%s%s%s" of "^951" (0x9b24d4), WinInput vf28's key name, "^999" (0x9b24cc) [O].
    if (const char* g = dikGlyph(dik)) {
        const int i = glyphIndex(g);
        if (i != 0xff) return std::string("^000") + (i ? glyphByte(i) : std::string()) + "^000 ";
    }
    return "^951" + keyName + "^999";
}

size_t HelpTexts::load(const std::string& dataDir) {
    std::lock_guard<std::mutex> lk(m_);
    if (loaded_) return records_.size();
    loaded_ = true;
    // Story/init.tsc LoadINPUT 0x4e2bf0 -> "BSource input\input.PC" -> RemapTextAdd lines (0x4e2d00) [O].
    const std::string path = findPathNoCase(dataDir + "/CommonComponent", "Input\\input.PC");
    std::ifstream f(path.empty() ? dataDir + "/CommonComponent/Input/input.PC" : path);
    for (std::string line; std::getline(f, line);) {
        if (const size_t c = line.find("//"); c != std::string::npos) line.resize(c);
        std::istringstream ls(line);
        std::vector<std::string> a;
        for (std::string t; ls >> t;) a.push_back(t);
        if (a.empty() || upperOf(a[0]) != "REMAPTEXTADD" || a.size() < 2) continue;
        Record r;
        r.name = a[1].substr(0, 23);  // char[24]
        // 0x4e2d00 [O]: glyph = 0x677c80(arg 3), +0x1c = atol(arg 3) (< 1 -> -1), action = arg 2 unless "NONE"
        // (Input_ActionIndexByName), group = crc(arg 4) when given and not "ALL" (stricmp), else [0xa67434] (ALL).
        if (a.size() > 3) r.glyph = glyphIndex(a[3]);
        const long tt = a.size() > 3 ? std::atol(a[3].c_str()) : 0;
        r.tt = tt < 1 ? -1 : int(tt);
        if (a.size() > 2 && a[2] != "NONE") r.action = fuel::input::actionByName(a[2]);
        if (a.size() > 4 && upperOf(a[4]) != "ALL") r.group = upperOf(a[4]);
        records_.push_back(r);
    }
    return records_.size();
}

bool HelpTexts::loaded() const {
    std::lock_guard<std::mutex> lk(m_);
    return loaded_;
}

std::vector<HelpTexts::Record> HelpTexts::records() const {
    std::lock_guard<std::mutex> lk(m_);
    return records_;
}

void HelpTexts::transText(std::string& s) {
    std::lock_guard<std::mutex> lk(m_);
    // 0x4e2820 [O]: for every record in order, strstr from the start; each hit at `count` bytes from the line start adds
    // 2 + count to the search offset (FUEL's own arithmetic: a second hit of the same name can be skipped); a hit not
    // preceded by '{' is replaced by "{NAME}" and its variable is made (record +0x1c < 1: string variable 0x57bd50, else a
    // TT variable 0x57ca00; an existing TT variable is reset to the record's TT).
    for (const Record& r : records_) {
        if (r.name.empty()) continue;
        size_t off = 0;
        size_t p = s.find(r.name);
        while (p != std::string::npos) {
            const size_t count = p;
            off += 2 + count;
            if (count < 1 || s[p - 1] != '{') {
                auto it = vars_.find(r.name);
                if (it == vars_.end()) vars_.emplace(r.name, std::string());
                else if (r.tt >= 1) it->second.clear();
                s = s.substr(0, count) + "{" + r.name + "}" + s.substr(p + r.name.size());
            }
            p = off <= s.size() ? s.find(r.name, off) : std::string::npos;
        }
    }
}

bool HelpTexts::rebuild(bool pad, const std::function<std::string(int)>& keyLabel, const PadLabelFn& padLabel) {
    std::lock_guard<std::mutex> lk(m_);
    ++rebuilds_;
    for (const Record& r : records_) {
        auto it = vars_.find(r.name);
        if (it == vars_.end()) continue;  // only variables the texts made (LevelAgent +0x90c lookup by crc)
        // 0x4e2250 [O]: label 1 = the pad glyph byte (pads present) or the key label of the action (none for NONE);
        // label 2 stays empty. Group ALL: label 1 when the call's argument or the binding set's byte +0x8d is clear,
        // else "%s / %s" (with the STR_MOVE exception). [I] +0x8d is clear for one local player (the PC build), so ALL
        // is label 1; P1 -> label 1, P2 -> label 2, other groups -> "".
        std::string l1;
        if (pad && padLabel) {
            l1 = padLabel(r.action, r.glyph < 0xff ? r.glyph : -1);  // [port] the bound pad input's icon (pad-ux)
        } else if (pad) {
            if (r.glyph > 0 && r.glyph < 0xff) l1 = glyphByte(r.glyph);
        } else if (r.action >= 0 && keyLabel) {
            l1 = keyLabel(r.action);
        }
        std::string v;
        if (r.group == "ALL" || r.group == "P1") v = l1;
        it->second = v;  // var +8 = string, +4 = -1
    }
    return true;
}

bool HelpTexts::padCheck(bool pad, const std::function<std::string(int)>& keyLabel, const PadLabelFn& padLabel, int variant) {
    int was, wasVariant;
    {
        std::lock_guard<std::mutex> lk(m_);
        was = padState_, wasVariant = padVariant_;
    }
    // 0x4e36e0 [O]: +0x25 set (nothing built yet): with a pad as below, without one rebuild and clear +0x25; then
    // pad present and +0x24 clear -> rebuild, +0x24 = 1; no pad and +0x24 set -> rebuild, +0x24 = 0.
    // [port] pad-ux: a pad of another family (or another pad_glyphs choice) while a pad is present rebuilds too.
    if (was == (pad ? 1 : 0) && (!pad || wasVariant == variant)) return false;
    rebuild(pad, keyLabel, padLabel);
    std::lock_guard<std::mutex> lk(m_);
    padState_ = pad ? 1 : 0;
    padVariant_ = variant;
    return true;
}

bool HelpTexts::padMode() const {
    std::lock_guard<std::mutex> lk(m_);
    return padState_ == 1;
}

std::string HelpTexts::expand(const std::string& s) const {
    if (s.find('{') == std::string::npos) return s;
    std::lock_guard<std::mutex> lk(m_);
    // 0x57bdf0 [O]: text up to "{", the variable (string +8 when +4 < 0, else its TT), then on after "}"; names longer
    // than 0x100 bytes stop the expansion.
    std::string out;
    size_t at = 0;
    for (;;) {
        const size_t a = s.find('{', at);
        if (a == std::string::npos) break;
        const size_t b = s.find('}', a);
        if (b == std::string::npos || b - a > 0x100) break;
        auto it = vars_.find(s.substr(a + 1, b - a - 1));
        if (it == vars_.end()) {  // [port] keep unknown "{...}" as written
            out += s.substr(at, b + 1 - at);
        } else {
            out += s.substr(at, a - at);
            out += it->second;
        }
        at = b + 1;
    }
    return out + s.substr(at);
}

std::string HelpTexts::value(const std::string& name) const {
    std::lock_guard<std::mutex> lk(m_);
    auto it = vars_.find(name);
    return it == vars_.end() ? std::string() : it->second;
}

size_t HelpTexts::variableCount() const {
    std::lock_guard<std::mutex> lk(m_);
    return vars_.size();
}

int HelpTexts::rebuilds() const {
    std::lock_guard<std::mutex> lk(m_);
    return rebuilds_;
}

HelpTexts& helpTexts() {
    static HelpTexts h;
    return h;
}

std::function<std::string(int)> keyboardLabels(const std::string& dataDir, const std::map<std::string, std::string>& settings) {
    auto user = std::make_shared<fuel::input::InputMap>();
    auto fuelMap = std::make_shared<fuel::input::InputMap>();
    std::string err;
    const bool okUser = user->load(dataDir, settings, &err);
    const bool okFuel = fuelMap->load(dataDir, {{"controls", "fuel"}}, &err);
    // [port] remap: the menus now answer to the bindings (ui/menu_input.*), so every label is the action's first key (the
    // menu actions' defaults are the keys the port's menus always used: Q / E for MENU_L1 / R1, G for MENU_CANCEL, = / -).
    return [user, fuelMap, okUser, okFuel](int action) -> std::string {
        if (action < 0 || action >= fuel::input::kActionCount) return std::string();
        const std::vector<int>* keys = okUser ? &user->keysOf(action) : nullptr;
        if ((!keys || keys->empty()) && okFuel) keys = &fuelMap->keysOf(action);
        if (!keys || keys->empty()) return HelpTexts::keyLabel(-1, std::string());  // 0x83cfa0 dev < -2 -> ""
        const int mac = keys->front();
        return HelpTexts::keyLabel(fuel::input::dikForMacKey(mac), fuel::input::keyDisplayName(mac));
    };
}

HelpTexts::PadLabelFn padLabels(std::shared_ptr<const fuel::input::InputMap> map) {
    return [map](int action, int glyph) -> std::string {
        if (map && action >= 0 && action < fuel::input::kActionCount)
            for (int p : map->padsOf(action))
                if (p >= 0) return pad::iconText(p);  // the first pad binding (0x66a9b0's two slots)
        return pad::iconText(pad::iconForGlyph(glyph));
    };
}

HelpTexts::PadLabelFn padLabels(const std::string& dataDir, const std::map<std::string, std::string>& settings) {
    auto map = std::make_shared<fuel::input::InputMap>();
    std::string err;
    return padLabels(map->load(dataDir, settings, &err) ? map : nullptr);
}

int runHelpTextCheck(const std::string& dataDir) {
    int bad = 0;
    auto expect = [&](bool ok, const std::string& what) {
        if (!ok) ++bad, std::printf("help texts: FAIL %s\n", what.c_str());
    };
    HelpTexts h;
    const size_t n = h.load(dataDir);
    expect(n == 79, "79 RemapTextAdd records in input.PC (got " + std::to_string(n) + ")");
    const auto recs = h.records();
    auto rec = [&](const std::string& name) -> const HelpTexts::Record* {
        for (const HelpTexts::Record& r : recs)
            if (r.name == name) return &r;
        return nullptr;
    };
    const HelpTexts::Record* valid = rec("STR_MENU_VALID");
    expect(valid && valid->glyph == 1 && valid->action == fuel::input::MenuValid && valid->tt == -1 && valid->group == "ALL",
           "STR_MENU_VALID: CROSS (1), MENU_VALID, tt -1, ALL");
    const HelpTexts::Record* start = rec("STR_START");
    expect(start && start->glyph == 19 && start->action == -1, "STR_START: START (19), NONE");
    expect(HelpTexts::glyphIndex("triangle") == 4 && HelpTexts::glyphIndex("FOO") == 0xff, "0x677c80 names (case-insensitive, 0xff)");
    // 0x4e2820 rewrite, with FUEL's quirks: STR_RESET also hits STR_RESETCAMERA; a rewritten name is not rewritten again.
    std::string t470 = "STR_MENU_L1/STR_MENU_R1 Switch menu\xc2\xb2\xc2\xb2\xc2\xb2STR_MENU_VALID Accept";
    h.transText(t470);
    expect(t470 == "{STR_MENU_L1}/{STR_MENU_R1} Switch menu\xc2\xb2\xc2\xb2\xc2\xb2{STR_MENU_VALID} Accept", "TT 470 rewrite: " + t470);
    std::string twice = t470;
    h.transText(twice);
    expect(twice == t470, "a rewritten line stays as it is");
    std::string cam = "STR_RESETCAMERA";
    h.transText(cam);
    expect(cam == "{STR_RESET}CAMERA", "STR_RESET hits STR_RESETCAMERA first (record order): " + cam);
    std::string start1218 = "Press STR_START to begin";
    h.transText(start1218);
    std::string t489 = "STR_MENU_VALID Yes\xc2\xb2\xc2\xb2\xc2\xb2STR_MENU_BACK No";
    h.transText(t489);
    expect(start1218 == "Press {STR_START} to begin", "TT 1218 rewrite");
    // 0x4e2250 / 0x4e27c0 / 0x4e2580: keyboard labels and pad glyphs.
    const auto labels = keyboardLabels(dataDir, {{"controls", "fuel"}});
    expect(h.padCheck(false, labels) && h.rebuilds() == 1, "PaddleChecker builds on its first frame");
    expect(!h.padCheck(false, labels) && h.rebuilds() == 1, "no rebuild without a pad change");
    expect(h.value("STR_MENU_VALID") == std::string("^000\x01^000 "), "MENU_VALID on RETURN -> the CROSS glyph \"^000\\x01^000 \"");
    expect(h.value("STR_MENU_BACK") == std::string("^000\x02^000 "), "MENU_BACK on ESCAPE -> the CIRCLE glyph");
    expect(h.value("STR_MENU_L1") == "^951Q^999", "MENU_L1 on Q -> \"^951Q^999\" (got \"" + h.value("STR_MENU_L1") + "\")");
    expect(h.value("STR_START").empty(), "STR_START (NONE) has no key label");
    expect(h.expand("Press {STR_START} to begin") == "Press  to begin", "expand without a pad");
    expect(h.padCheck(true, labels) && h.padMode(), "a pad arriving rebuilds (+0x24 = 1)");
    expect(h.value("STR_MENU_VALID") == std::string("\x01") && h.value("STR_START") == std::string("\x13"), "pad glyphs CROSS 1, START 0x13");
    expect(h.expand("{STR_MENU_L1}/{STR_MENU_R1} x") == std::string("\x08/\x09 x"), "expand with a pad");
    expect(h.expand("{padid} {STR_MENU_VALID}") == std::string("{padid} \x01"), "unknown variables stay [port]");
    expect(h.padCheck(false, labels) && !h.padMode(), "the pad leaving rebuilds (+0x24 = 0)");
    const auto portLabels = keyboardLabels(dataDir, {{"controls", "port"}});
    h.rebuild(false, portLabels);
    std::string accel = "STR_ACCEL Go";
    h.transText(accel);
    h.rebuild(false, portLabels);
    expect(h.expand(accel) == std::string("^000\x10^000  Go"), "controls = port: ACCEL on W -> the W key cap (0x4e2580 LEFT, 16)");
    const auto fuelLabels = keyboardLabels(dataDir, {{"controls", "fuel"}});
    h.rebuild(false, fuelLabels);
    expect(h.expand(accel) == "^951SPACE^999 Go", "controls = fuel: ACCEL on SPACE (DftMap) -> \"^951SPACE^999\"");
    expect(h.value("STR_MENU_VALID") == std::string("^000\x01^000 "), "controls = port: menu keys from DftMap");
    // ---- pad-ux [port]: the pad labels follow the bindings (InputMap::padsOf) as pad icons; NONE records use their glyph.
    {
        using namespace fuel::input;
        HelpTexts p;
        p.load(dataDir);
        std::string line = "STR_MENU_VALID a STR_START b STR_ACCEL c STR_MENU_UP d STR_LEFT_ACTION e STR_DIRPAD f STR_MENU_L1 g STR_UPDOWN";
        p.transText(line);
        const auto pl = padLabels(dataDir, {{"controls", "port"}});
        expect(p.padCheck(true, portLabels, pl, 1) && p.padMode(), "pad labels: a pad arriving builds the icons");
        const std::string want = pad::iconText(PadA) + " a " + pad::iconText(PadStart) + " b " + pad::iconText(PadRT) + " c " +
                                 pad::iconText(PadLYPos) + " d " + pad::iconText(PadLXNeg) + " e " + pad::iconText(pad::LStickAny) + " f " +
                                 pad::iconText(PadLB) + " g " + pad::iconText(pad::LStickVertical);
        expect(p.expand(line) == want, "MENU_VALID -> A, START (NONE) -> START, ACCEL -> RT, MENU_UP -> LANALOG_UP, LEFT_ACTION -> "
                                       "LANALOG_LEFT, DIRPAD / UPDOWN (NONE) -> the left stick, MENU_L1 -> LB");
        expect(!p.padCheck(true, portLabels, pl, 1) && p.padCheck(true, portLabels, pl, 2), "a family change rebuilds, the same one does not");
        expect(!p.padCheck(true, portLabels, pl, 2) && p.padCheck(false, portLabels, pl, 2) && p.expand("{STR_MENU_VALID}") == std::string("^000\x01^000 "),
               "the pad leaving: key caps again (controls = port)");
        // A rebound action shows its new button (the remap page's write path, InputMap::setPads).
        auto map = std::make_shared<InputMap>();
        std::string err;
        map->load(dataDir, {{"controls", "port"}}, &err);
        map->setPads(MenuValid, {PadX, -1});
        map->setPads(Accel, {-1, PadRB});
        p.rebuild(true, portLabels, padLabels(map));
        expect(p.expand("{STR_MENU_VALID}{STR_ACCEL}{STR_START}") == pad::iconText(PadX) + pad::iconText(PadRB) + pad::iconText(PadStart),
               "rebound MENU_VALID -> X, ACCEL (second slot) -> RB; NONE records keep their glyph");
    }
    std::printf("help texts: %zu records, %zu variables, %d rebuilds, %s\n", n, h.variableCount(), h.rebuilds(), bad ? "FAILED" : "ok");
    return bad;
}

}  // namespace fuel::ui
