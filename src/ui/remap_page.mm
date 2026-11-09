// [port] remap: Options > CONTROLS > LAYOUT, FUEL's remap page (state 0x2d), un-parked at the user's request (2026-11-06).
// Notes ~/re-work/fuel/notes/remap.md. Tags: [O] FUEL.exe / its data, [port] the port's own.
//
// FUEL [O]:
//  - entry 0x491910 case 0 -> 0x4cc340 (capture off, clocks 0, state 0x2d) when the player uses no pad, else state 0xad
//    (PageOptionsControllerConfigX360 / PS3: a picture of the pad, no remap). Title TT_TOP_TITLE_OPTION_CONTROLLER_CONFIG
//    (240 "LAYOUT"), help TT_BH_OPTIONS_CONTROLS_LAYOUT_KEYBOARD (7006) / _PAD (7056), legend TT 16028 "STR_RESET Reset
//    Default / STR_MENU_BACK Back".
//  - the list 0x4cc650: PageOptionsControllerConfigChange (ITEM_ID_OPT_CTRL_CHANGE_ITEM_FIRST at y 250, h 50 = the row
//    pitch; FLECHEUP 216 / FLECHEDOWN 503 = five rows), one PageOptionsControllerConfigChangeB per action (halo 772 x 59,
//    grey bar 764 x 34 + yellow part 379, ITEM_ID_OPT_CTRL_CH_COMMAND = the action's name at x 320, ITEM_ID_OPT_CTRL_CH_ACTION
//    = its key at x 697 w 324, ITEM_ID_OPT_CTRL_BOX = the pointer box), PageOptionsControllerConfigChangeT for a title.
//  - per frame 0x4cc870: capturing (+0x468) -> WinInput vt+0x50 (0x83dce0) until it reports an input, then 0x4cd080 binds
//    it (swap rule, core/remap.hpp), the label colour back, the clock +0x484 = 0. Otherwise Up / Down move the selection
//    (window 0x4ccc30), MENU_RESET (game +0x234) -> the default map (vt+0x80, vt+0x58), selection 0, the list again and the
//    help texts again (0x4e2250(1)); after 0.5 s on the page (+0x484 > 0.5) MENU_VALID or a click on the row's box starts a
//    capture (vt+100) and colours the row's key label (0.5, 0, 0, 1). The pointer selects the row it is over (0x4cd8f0).
//    Back (MENU_BACK or MENU_SELECT, 0x4c9190) -> 0x4ccfd0: apply, +0x80 = 1 (save), help texts again, state 0x29.
//  - the focused row pulses: alpha 0.25 + (sin(10 t) + 1) x 0.25, t wrapping at 10 s (0x4cd9d0).
// The port [port]: two binding columns (keyboard at x 697, pad at x 859: the ACTION label's 324 split in two; FUEL PC remaps
// the keyboard only), the column titles on the context title rows, Left / Right (and L1 / R1) pick the column, a "Reset to
// defaults" row with a cell per device and MENU_RESET resetting the focused column's device after FUEL's Yes / No popup
// (FUEL resets at once), Escape cancels a capture (FUEL's text TT 15545; a pad capture also gives up after 10 s or on a
// click), the swap / refusal named on a note line under the list, the mouse wheel scrolling the list (pad-ux).
#include "ui/menu.h"

#include "core/remap.hpp"
#include "ui/help_texts.hpp"
#include "ui/menu_input.h"
#include "ui/pad_prompts.hpp"
#include "ui/ui_renderer.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>
#include <unistd.h>

namespace fuel::ui {

namespace {

namespace in = fuel::input;
namespace rm = fuel::input::remap;

// Port texts (ids outside FUEL's table).
constexpr uint32_t kTtLegend = 0x7f520001;       // the page's legend: Change + FUEL's TT 16028
constexpr uint32_t kTtWait = 0x7f520002;         // capturing: TT 15544 + TT 15545
constexpr uint32_t kTtResetKeys = 0x7f520003;    // the reset popups
constexpr uint32_t kTtResetPad = 0x7f520004;
constexpr int kVisible = 5;                      // FLECHEUP 216 .. FLECHEDOWN 503 over 50-unit rows from 250
constexpr float kCellX[2] = {697.0f, 859.0f};    // [port] ITEM_ID_OPT_CTRL_CH_ACTION (697, w 324) split in two
constexpr float kCellW = 162.0f;
constexpr float kNoteY = 540.0f;                 // [port] under FLECHEDOWN (503 + 21), above BandeauHelp (597)

std::string upperOf(std::string s) {
    for (char& c : s) c = char(std::toupper(static_cast<unsigned char>(c)));
    return s;
}

}  // namespace

const in::InputMap& Menu::bindings() {
    if (!bindings_ || bindSerial_ != optSerial_) {
        auto m = std::make_shared<in::InputMap>();
        std::string err;
        if (!m->load(dataDir_, liveSettings().all, &err))
            for (int a = 0; a < in::kActionCount; ++a) m->setPads(a, {-1, -1});  // no data: no pad bindings (keys none)
        bindings_ = m, bindSerial_ = optSerial_, bindDefaults_.reset();
    }
    return *bindings_;
}

const in::InputMap& Menu::bindingDefaults() {
    bindings();
    if (!bindDefaults_) {
        auto d = std::make_shared<in::InputMap>();
        std::string err;
        if (!d->load(dataDir_, in::InputMap::withoutBindings(liveSettings().all), &err))
            for (int a = 0; a < in::kActionCount; ++a) d->setPads(a, {-1, -1});
        bindDefaults_ = d;
    }
    return *bindDefaults_;
}

std::vector<MenuKey> Menu::menuKeys(unsigned short macKey) { return menuKeysForKey(bindings(), macKey, remapPageUp()); }

// The page's texts: the legends (BandeauHelp ITEM_ID_BUTTONS entries) and the reset questions (PopUpHelp list boxes),
// through TransText like FUEL's own (0x4e2820: "STR_X" -> "{STR_X}").
static void addEntry(Page* p, const char* item, const std::string& key, uint32_t tt) {
    if (!p) return;
    for (Item& it : p->items) {
        if (item ? it.id != item : it.kind != ItemKind::ListBoxTT) continue;
        bool has = false;
        for (const ListEntry& e : it.entries) has = has || e.key == key;
        if (!has) it.entries.push_back(ListEntry{key, tt, false});
    }
}

void Menu::remapOpen() {
    if (!text_.get(kTtLegend)) {
        const std::string sep = "\xc2\xb2\xc2\xb2\xc2\xb2";
        auto put = [&](uint32_t id, std::string s) {
            helpTexts().transText(s);
            text_.strings[id] = s;
        };
        const std::string* reset = text_.get(16028);  // "STR_RESET Reset Default²²²STR_MENU_BACK Back"
        put(kTtLegend, "STR_MENU_VALID Change" + sep + "STR_MENU_L1/STR_MENU_R1 Keyboard / Pad" + sep +
                           (reset ? *reset : "STR_RESET Reset Default" + sep + "STR_MENU_BACK Back"));
        const std::string* w1 = text_.get(15544);  // "Waiting for a new key."
        const std::string* w2 = text_.get(15545);  // "Press ESCAPE to cancel."
        text_.strings[kTtWait] = (w1 ? *w1 : std::string("Waiting for a new key.")) + "   " + (w2 ? *w2 : std::string("Press ESCAPE to cancel."));
        text_.strings[kTtResetKeys] = "Reset every keyboard control to its default?";
        text_.strings[kTtResetPad] = "Reset every controller button to its default?";
        addEntry(scripts_.page("BandeauHelp"), "ITEM_ID_BUTTONS", "TT_MENU_OPTIONS_REMAP", kTtLegend);
        addEntry(scripts_.page("BandeauHelp"), "ITEM_ID_BUTTONS", "TT_MENU_OPTIONS_REMAP_WAIT", kTtWait);
        addEntry(scripts_.page("PopUpHelp"), nullptr, "TT_REMAP_RESET_KEYBOARD", kTtResetKeys);
        addEntry(scripts_.page("PopUpHelp"), nullptr, "TT_REMAP_RESET_PAD", kTtResetPad);
    }
    // 0x4cc340: capture off, the clocks 0, state 0x2d; 0x4cc650 builds the list.
    screen_ = Screen::OptRemap;
    ctrlMenu_ = false;
    remap_.rows = rm::rows();
    remap_.capture = false, remap_.clock = 0, remap_.pulse = 0, remap_.note.clear();
    remap_.column = 0;
    focus_ = 1;  // the first action (row 0 is the "General" title)
    scroll_ = ListScroll{};
    setTitle("TT_TOP_TITLE_OPTION_CONTROLLER_CONFIG");
}

// 0x4ccfd0: apply, +0x80 = 1 (saved), the help texts again (the hosts rebuild them on the serial), state 0x29.
void Menu::remapLeave() {
    remap_.capture = false;
    pauseDirty_ = true;
    saveOptions();
    goTab(TabOptions);
    ctrlMenu_ = true, focus_ = 0, setTitle("TT_TOP_TITLE_OPTION_CONTROLLER");
}

void Menu::remapMove(int d) {
    const int n = int(remap_.rows.size());
    int f = focus_;
    do f += d;
    while (f >= 0 && f < n && remap_.rows[size_t(f)].kind == rm::Row::Title);
    if (f < 0 || f >= n) return;  // no wrap (0x4cc870: Up while +0x458 > 0, Down while < count - 1)
    focus_ = f;
    syncScroll();
    if (focus_ == 1 || (focus_ > 0 && remap_.rows[size_t(focus_ - 1)].kind == rm::Row::Title && d < 0)) scroll_.follow(focus_ - 1);  // show its title
}

// After the wheel / an arrow moved the window: the selection inside it, never on a title.
void Menu::remapAfterScroll(int dir) {
    const int n = int(remap_.rows.size());
    if (focus_ >= 0 && focus_ < n && remap_.rows[size_t(focus_)].kind != rm::Row::Title) return;
    const int last = std::min(scroll_.first + scroll_.visible, n) - 1;
    for (int f = focus_; f >= scroll_.first && f <= last; f += dir >= 0 ? 1 : -1)
        if (remap_.rows[size_t(f)].kind != rm::Row::Title) return void(focus_ = f);
    for (int f = focus_; f >= scroll_.first && f <= last; f += dir >= 0 ? -1 : 1)
        if (remap_.rows[size_t(f)].kind != rm::Row::Title) return void(focus_ = f);
}

void Menu::remapResetAsk(int device) {
    openConfirm(device == rm::Keyboard ? "TT_REMAP_RESET_KEYBOARD" : "TT_REMAP_RESET_PAD", [this, device] {
        auto m = std::make_shared<in::InputMap>(bindings());
        rm::resetDevice(*m, bindingDefaults(), rm::Device(device));
        bindLines_ = m->bindingLines(bindingDefaults()), bindLinesSet_ = true;
        ++optSerial_;  // the menus, the session and the prompts follow (0x4e2250(1))
        remap_.note = device == rm::Keyboard ? "Keyboard controls reset to their defaults." : "Controller buttons reset to their defaults.";
        remap_.noteTime = 0;
        // 0x4cc870: the selection back to the first row, the window to the top.
        focus_ = 1, scroll_ = ListScroll{};
    });
}

void Menu::remapAccept() {
    if (focus_ < 0 || focus_ >= int(remap_.rows.size())) return;
    const rm::Row& row = remap_.rows[size_t(focus_)];
    if (row.kind == rm::Row::Reset) return remapResetAsk(remap_.column);
    if (row.kind != rm::Row::Action || remap_.clock <= 0.5f) return;  // 0x4cc870: +0x484 > 0.5
    remap_.capture = true, remap_.captureTime = 0, remap_.pad = {};
    remap_.note.clear();
}

void Menu::remapKey(MenuKey k) {
    if (remap_.capture) return;  // the raw key / pad path has the input while capturing
    switch (k) {
        case MenuKey::Up: remapMove(-1); break;
        case MenuKey::Down: remapMove(1); break;
        case MenuKey::Left:
        case MenuKey::TabLeft: remap_.column = 0; break;
        case MenuKey::Right:
        case MenuKey::TabRight: remap_.column = 1; break;
        case MenuKey::Accept: remapAccept(); break;
        case MenuKey::Reset: remapResetAsk(remap_.column); break;  // MENU_RESET (game +0x234) [port: asks first]
        case MenuKey::Back: remapLeave(); break;
        default: break;
    }
}

void Menu::remapApply(const rm::Result& r, int action, int device) {
    (void)device;
    remap_.capture = false, remap_.clock = 0;  // 0x4cc870 after 0x4cd080: +0x484 = 0, +0x468 = 0
    remap_.noteTime = 0;
    auto name = [&](int a) {
        const uint32_t tt = rm::actionText(a);
        return tt ? textOf(tt) : std::string(rm::portName(a) ? rm::portName(a) : in::actionName(a));
    };
    const std::string used = textOf(15560).empty() ? std::string("This key is already used for another action.") : textOf(15560);
    if (r.swapped >= 0) {
        std::string with;
        for (size_t i = 0; i < r.others.size(); ++i) with += (i ? (i + 1 == r.others.size() ? " and " : ", ") : "") + name(r.others[i]);
        remap_.note = used + " Swapped with " + with + ".";
    }
    else remap_.note.clear();
    if (r.refused) {
        remap_.note = used + " (" + name(action) + " not changed: it would leave the menus without a key.)";
        return;
    }
    if (!r.changed) return;
    bindLines_ = bindings_->bindingLines(bindingDefaults()), bindLinesSet_ = true;
    ++optSerial_;  // the bindings reload; the menus' prompts and the session follow (pad-ux's rebuild path)
}

bool Menu::rawKey(unsigned short macKey, bool repeat) {
    if (screen_ != Screen::OptRemap || !remap_.capture || confirm_.open || alertUp()) return false;
    if (repeat) return true;
    if (rm::captureCancels(macKey)) {  // [port] Escape: FUEL's TT 15545 "Press ESCAPE to cancel."
        remap_.capture = false, remap_.clock = 0;
        return true;
    }
    if (remap_.column != rm::Keyboard || !rm::capturable(macKey)) return true;  // the pad column waits for the pad
    const rm::Row& row = remap_.rows[size_t(focus_)];
    bindings();  // fresh map
    auto m = std::make_shared<in::InputMap>(*bindings_);
    const rm::Result r = rm::rebind(*m, row.action, rm::Keyboard, macKey);
    if (!r.refused) bindings_ = m, bindSerial_ = optSerial_;
    remapApply(r, row.action, rm::Keyboard);
    return true;
}

bool Menu::padFrame(const in::PadState& pad, float dt) {
    if (screen_ != Screen::OptRemap || !remap_.capture || confirm_.open || alertUp()) return false;
    if (remap_.column != rm::Pad) return true;  // a keyboard capture: the pad is ignored, not routed
    remap_.captureTime += dt;
    if (remap_.captureTime > 10.0f) {  // [port] nothing pressed: the capture gives up
        remap_.capture = false, remap_.clock = 0;
        return true;
    }
    const int code = remap_.pad.step(pad);
    if (code < 0) return true;
    const rm::Row& row = remap_.rows[size_t(focus_)];
    bindings();
    auto m = std::make_shared<in::InputMap>(*bindings_);
    const rm::Result r = rm::rebind(*m, row.action, rm::Pad, code);
    if (!r.refused) bindings_ = m, bindSerial_ = optSerial_;
    remapApply(r, row.action, rm::Pad);
    return true;
}

// A cell's text: the keys as the help bars show them (key caps / FUEL's key names), the pad inputs as the prompts (the
// connected pad's pictures, pad-ux) with their names.
std::string Menu::remapCell(int action, int device) {
    const in::InputMap& m = bindings();
    std::string out;
    int n = 0;
    if (device == rm::Keyboard) {
        for (int k : m.keysOf(action)) {
            if (n == 2) {
                out += " ...";
                break;
            }
            out += (n++ ? " / " : "") + in::keyDisplayName(k, [this](uint32_t tt) { return textOf(tt); });
        }
        return out.empty() ? "-" : out;
    }
    const auto& pads = m.padsOf(action);
    const bool two = pads[0] >= 0 && pads[1] >= 0;
    for (int p : pads) {
        if (p < 0) continue;
        // pad icons 0..23 = the Pad inputs (pad_prompts.hpp); two inputs: the pictures only (the names would not fit)
        out += (n++ ? "  " : "") + pad::iconText(p) + (two ? std::string() : " " + upperOf(pad::buttonName(p)));
    }
    return out.empty() ? "-" : out;
}

void Menu::drawRemap(UiRenderer& r) {
    Page* list = scripts_.page("PageOptionsControllerConfigChange");
    Page* rowPage = scripts_.page("PageOptionsControllerConfigChangeB");
    Page* titlePage = scripts_.page("PageOptionsControllerConfigChangeT");
    if (!list || !rowPage || !titlePage) return;
    syncScroll();
    const int n = int(remap_.rows.size());
    const Item* cellSrc = rowPage->find("ITEM_ID_OPT_CTRL_CH_ACTION");
    for (int s = 0; s < scroll_.visible && scroll_.first + s < n; ++s) {
        const int k = scroll_.first + s;
        const rm::Row& row = remap_.rows[size_t(k)];
        const float y = remapRowY(s);
        if (row.kind == rm::Row::Title) {
            for (const Item& src : titlePage->items) {
                Item it = src;
                it.visible = true;
                if (it.id == "ITEM_ID_OPT_CTRL_CH_TITLE") {
                    it.hasDynText = true, it.dynText = upperOf(textOf(row.tt));
                    // [port] the authored text colour is black (255:0:0:0) on a black bar (BARRE_GARAGE_PERFORMANCES_TITRE's
                    // pixels are black; its 255:156:18 tint cannot lighten them): the title in that tint instead, as the
                    // garage's vehicle name sits orange on the same bar.
                    it.tNormal = it.tEnabled = it.tFocus = Rgba{1.0f, 156.0f / 255.0f, 18.0f / 255.0f, 1};
                }
                drawItem(r, it, 0, y);
            }
            // [port] the column titles over the two binding columns, in the title label's style
            if (const Item* t = titlePage->find("ITEM_ID_OPT_CTRL_CH_TITLE"); t && cellSrc)
                for (int c = 0; c < 2; ++c) {
                    Item it = *cellSrc;
                    it.x = kCellX[c], it.w = kCellW, it.y = t->y, it.visible = true, it.focus = false;
                    it.tNormal = it.tEnabled = it.tFocus = Rgba{1, 1, 1, 1};
                    it.hasDynText = true;
                    it.dynText = c == 0 ? upperOf(textOf(15634).empty() ? std::string("Keyboard") : textOf(15634))  // TT 15634 "Keyboard"
                                        : (pad::state().present ? upperOf(pad::state().family == pad::Family::PlayStation ? "PlayStation" : "Xbox") + " PAD"
                                                                : std::string("CONTROLLER"));
                    drawItem(r, it, 0, y);
                }
            continue;
        }
        const bool focused = k == focus_;
        for (const Item& src : rowPage->items) {
            Item it = src;
            it.visible = true;
            if (it.id == "ITEM_ID_HALO") {
                if (!focused) continue;
                it.id = "ITEM_ID_HALO1";  // the pulse of a focused row (0x4cd9d0 = drawItem's 0x46c400 formula)
                it.focus = true;
            } else if (it.id == "ITEM_ID_OPT_CTRL_BOX") {
                continue;  // the pointer box (invisible)
            } else if (it.id == "ITEM_ID_OPT_CTRL_CH_COMMAND") {
                it.hasDynText = true;
                it.dynText = row.kind == rm::Row::Reset ? upperOf(row.name) : row.tt ? textOf(row.tt) : row.name;
                it.focus = focused;
            } else if (it.id == "ITEM_ID_OPT_CTRL_CH_ACTION") {
                for (int c = 0; c < 2; ++c) {
                    Item cell = it;
                    cell.x = kCellX[c], cell.w = kCellW;
                    cell.hasDynText = true;
                    cell.dynText = row.kind == rm::Row::Reset ? (c == 0 ? std::string("KEYBOARD") : std::string("CONTROLLER"))
                                                              : remapCell(row.action, c);
                    cell.focus = focused && remap_.column == c;
                    if (cell.focus)  // [port] the selected cell of the selected row in white (FUEL has one column)
                        cell.tNormal = cell.tEnabled = cell.tFocus = Rgba{1, 1, 1, 1};
                    if (cell.focus && remap_.capture)  // 0x4cc870: the row's key label (0.5, 0, 0, 1)
                        cell.tNormal = cell.tEnabled = cell.tFocus = Rgba{0.5f, 0, 0, 1};
                    drawItem(r, cell, 0, y);
                }
                continue;
            }
            drawItem(r, it, 0, y);
        }
    }
    drawScrollArrows(r);
    if (!remap_.note.empty())
        if (const Item* src = rowPage->find("ITEM_ID_OPT_CTRL_CH_COMMAND")) {
            Item it = *src;
            it.x = 290, it.w = 700, it.align = 1, it.visible = true, it.focus = false;
            it.tNormal = it.tEnabled = it.tFocus = Rgba{1, 1, 1, 1};
            it.hasDynText = true, it.dynText = remap_.note;
            drawItem(r, it, 0, kNoteY);
        }
}

// 0x4cd8f0 / 0x4cda90: the row under the pointer is selected; a click on the selected row's box starts a capture. [port]
// the cell under the pointer picks the column; the scroll arrows scroll.
bool Menu::remapPointer(float x, float y, bool click) {
    if (screen_ != Screen::OptRemap) return false;
    if (remap_.capture) {
        if (click) remap_.capture = false, remap_.clock = 0;  // [port] a click cancels the capture
        return true;
    }
    syncScroll();
    if (scrollArrowHit(x, y, click)) {
        remapAfterScroll(0);
        return true;
    }
    if (x < 300.0f || x > 1030.0f) return true;
    for (int s = 0; s < scroll_.visible && scroll_.first + s < int(remap_.rows.size()); ++s) {
        const float y0 = remapRowY(s);
        if (y < y0 - 4.0f || y >= y0 + 38.0f) continue;
        const int row = scroll_.first + s;
        if (remap_.rows[size_t(row)].kind == rm::Row::Title) return true;
        const int col = x >= kCellX[1] ? 1 : x >= kCellX[0] ? 0 : remap_.column;
        const bool was = focus_ == row && remap_.column == col;
        if (focus_ != row) soundBit(4);
        focus_ = row, remap_.column = col;
        if (click && was) remapAccept();
        return true;
    }
    return true;
}


// ---- --menu-check -------------------------------------------------------------------------------------------------

int runRemapCheck(const std::string& dataDir) {
    int bad = 0;
    auto expect = [&](bool ok, const std::string& what) {
        if (!ok) ++bad;
        std::printf("remap check: %s %s\n", ok ? "ok  " : "FAIL", what.c_str());
    };
    char tmpl[] = "/tmp/fuel-remap-check-XXXXXX";
    const char* dir = mkdtemp(tmpl);
    if (!dir) return 1;
    const std::string settingsFile = std::string(dir) + "/settings.txt", careerFile = std::string(dir) + "/career.sav";
    setenv("FUEL_SETTINGS", settingsFile.c_str(), 1);
    setenv("FUEL_CAREER_SAVE", careerFile.c_str(), 1);
    {
        std::ofstream f(settingsFile);  // the user's own layout: controls = port
        f << "# a comment\ndata = " << dataDir << "\ncontrols = port\nmy_key = 7\nkey.nitro = e\n";
    }
    auto fileText = [&] {
        std::ifstream f(settingsFile);
        std::stringstream t;
        t << f.rdbuf();
        return t.str();
    };
    Settings s0 = loadSettings();
    s0.dataDir = dataDir;
    Menu m(dataDir, s0, MenuMode::Main);
    m.setMode(MenuMode::Main);
    m.goTab(Menu::TabOptions);
    auto press = [&](MenuKey k, int n = 1) {
        for (int i = 0; i < n; ++i) m.applyItemStates(), m.key(k);
    };
    auto keyCode = [&](const char* name) { return static_cast<unsigned short>(in::macKeyByName(name)); };
    auto typeKey = [&](const char* name) {  // a host's keyDown: the capture first, then the menu keys
        const unsigned short c = keyCode(name);
        if (m.rawKey(c, false)) return;
        for (MenuKey k : m.menuKeys(c)) m.applyItemStates(), m.key(k);
    };
    // ---- the way in: Options > CONTROLS > LAYOUT (0x491910 case 0, no longer parked)
    m.focus_ = 0, press(MenuKey::Accept);
    m.applyItemStates();
    std::string left;
    for (Item* it : m.focusables()) left += (left.empty() ? "" : ",") + (it->hasDynText ? it->dynText : m.textOf(it->tt));
    expect(m.ctrlMenu_ && left == "LAYOUT,SETTINGS", "CONTROLS: LAYOUT enabled beside SETTINGS (" + left + ")");
    m.focus_ = 0, press(MenuKey::Accept);
    expect(m.screen_ == Menu::Screen::OptRemap && m.chrome().title == "TT_TOP_TITLE_OPTION_CONTROLLER_CONFIG", "LAYOUT opens the remap page (state 0x2d), title 240 LAYOUT");
    expect(m.chrome().buttons == "TT_MENU_OPTIONS_REMAP" && m.chrome().help == "TT_BH_OPTIONS_CONTROLS_LAYOUT_KEYBOARD", "legend and FUEL's help line 7006");
    // ---- the list (0x4cc650)
    const auto& rows = m.remap_.rows;
    int titles = 0, actions = 0;
    bool nitro = false, debug = false;
    for (const auto& r : rows) {
        titles += r.kind == rm::Row::Title;
        actions += r.kind == rm::Row::Action;
        nitro = nitro || (r.action == in::Boost && r.name == "Nitro");
        debug = debug || r.action == in::LFront;
    }
    expect(titles == 3 && actions == 17 + 4 + 5 + 17 && nitro && !debug && rows.back().kind == rm::Row::Reset,
           "rows: 3 titles, 17 + 4 + 5 + 17 actions, Nitro listed, no Map Debug, Reset last (" + std::to_string(actions) + ")");
    expect(m.textOf(rows[0].tt) == "General" && m.textOf(rows[1].tt) == "Steer left" && m.textOf(rm::actionText(in::MenuValid)) == "Valid",
           "FUEL's names: General / Steer left / Valid");
    expect(m.focus_ == 1, "the first action is selected");
    press(MenuKey::Up);
    expect(m.focus_ == 1, "Up on the first action stays (no wrap, never the title)");
    press(MenuKey::Down, 17);
    expect(rows[size_t(m.focus_)].action == in::PadDown, "Down steps over the Map title (" + std::to_string(m.focus_) + ")");
    press(MenuKey::Down, 200);
    expect(m.focus_ == int(rows.size()) - 1 && m.listScroll().first == int(rows.size()) - 5, "Down stops on Reset; the window follows (5 rows)");
    press(MenuKey::Up, 200);
    expect(m.focus_ == 1 && m.listScroll().first == 0, "back to the top");
    // ---- capture: FUEL's 0.5 s gate, Escape cancels
    auto find = [&](int action) {
        for (size_t i = 0; i < rows.size(); ++i)
            if (rows[i].action == action) return int(i);
        return -1;
    };
    m.focus_ = find(in::Accel), m.remap_.column = 0;
    m.remap_.clock = 0;
    press(MenuKey::Accept);
    expect(!m.remap_.capture, "Accept within 0.5 s of the page: no capture (0x4cc870 +0x484)");
    m.update(0.6f);
    press(MenuKey::Accept);
    expect(m.remap_.capture && m.chrome().buttons == "TT_MENU_OPTIONS_REMAP_WAIT", "Accept: waiting for a key (TT 15544 legend)");
    typeKey("escape");
    expect(!m.remap_.capture && m.bindings().keysOf(in::Accel)[0] == keyCode("w"), "Escape cancels; ACCEL keeps W");
    // ---- keyboard rebind with a swap (same context): ACCEL <- S (BRAKE's key in the port layout)
    m.update(0.6f), press(MenuKey::Accept);
    typeKey("s");
    {
        const auto& b = m.bindings();
        const bool swapped = b.keysOf(in::Accel) == std::vector<int>{keyCode("s")} && b.keysOf(in::Brake).size() == 2 &&
                             b.keysOf(in::Brake)[0] == keyCode("w") && b.keysOf(in::Brake)[1] == keyCode("up");
        expect(swapped && m.remap_.note.find("Swapped with Brake/Reverse") != std::string::npos,
               "ACCEL <- S swaps with BRAKE (W, Up): " + m.remap_.note);
    }
    // a key of another context: no swap (MENU_L1's Q stays)
    m.focus_ = find(in::Handbrake), m.update(0.6f), press(MenuKey::Accept);
    typeKey("q");
    expect(m.bindings().keysOf(in::Handbrake) == std::vector<int>{keyCode("q")} && m.bindings().keysOf(in::MenuL1)[0] == keyCode("q") &&
               m.remap_.note.empty(),
           "HANDBRAKE <- Q: MENU_L1 (another context) keeps Q, no swap");
    // the prompts follow: the keyboard labels from the live settings
    {
        const auto labels = keyboardLabels(dataDir, m.liveSettings().all);
        expect(labels(in::Accel) == HelpTexts::keyLabel(in::dikForMacKey(keyCode("s")), "S") &&
                   labels(in::Brake) == HelpTexts::keyLabel(in::dikForMacKey(keyCode("w")), "W"),
               "the help texts' key labels follow (ACCEL " + labels(in::Accel) + ", BRAKE " + labels(in::Brake) + ")");
    }
    // ---- menus through the bindings: MENU_VALID <- K (swaps with MAP_CURSOR_DOWN)
    m.focus_ = find(in::MenuValid), m.update(0.6f), press(MenuKey::Accept);
    typeKey("k");
    expect(m.menuKeys(keyCode("k")) == std::vector<MenuKey>{MenuKey::Accept} &&
               m.menuKeys(keyCode("return")) == std::vector<MenuKey>{MenuKey::MapDown},
           "MENU_VALID <- K: K accepts, Return moves the map cursor (swapped): " + m.remap_.note);
    // Accept by K starts the next capture; bind it back to Return.
    m.update(0.6f);
    typeKey("k");
    expect(m.remap_.capture, "K (the new MENU_VALID) starts a capture");
    typeKey("return");
    expect(m.menuKeys(keyCode("return")) == std::vector<MenuKey>{MenuKey::Accept}, "MENU_VALID back on Return");
    // ---- pad: column 2, capture waits for the release of the button that started it, LT swaps with BRAKE's
    m.focus_ = find(in::Accel), press(MenuKey::Right);
    expect(m.remap_.column == 1 && m.chrome().help == "TT_BH_OPTIONS_CONTROLS_LAYOUT_PAD", "Right: the pad column (help 7056)");
    m.update(0.6f), press(MenuKey::Accept);
    in::PadState held, rest, lt, a;
    held.connected = rest.connected = lt.connected = a.connected = true;
    held.buttons[in::PadA] = true, lt.lt = 1.0f, a.buttons[in::PadA] = true;
    m.padFrame(held, 0.016f), m.padFrame(held, 0.016f);
    expect(m.remap_.capture, "A still held from the Accept: not taken");
    m.padFrame(rest, 0.016f), m.padFrame(lt, 0.016f);
    expect(!m.remap_.capture && m.bindings().padsOf(in::Accel)[0] == in::PadLT && m.bindings().padsOf(in::Brake)[0] == in::PadRT,
           "pad ACCEL <- LT swaps with BRAKE (RT): " + m.remap_.note);
    // refusal: MAP_CURSOR_UP has no pad button; taking A from MENU_VALID would leave it none
    m.focus_ = find(in::MapCursorUp), m.update(0.6f), press(MenuKey::Accept);
    m.padFrame(rest, 0.016f), m.padFrame(a, 0.016f);
    expect(m.remap_.note.find("not changed") != std::string::npos && m.bindings().padsOf(in::MenuValid)[0] == in::PadA &&
               m.bindings().padsOf(in::MapCursorUp)[0] < 0,
           "MAP_CURSOR_UP <- A refused: MENU_VALID would have no button (" + m.remap_.note + ")");
    // a pad capture gives up after 10 s
    m.update(0.6f), press(MenuKey::Accept);
    for (int i = 0; i < 11; ++i) m.padFrame(rest, 1.0f);
    expect(!m.remap_.capture, "a pad capture gives up after 10 s");
    // ---- pointer: hover selects a row and its column, a click on the selected cell captures, a click cancels
    m.update(0.6f);
    m.syncScroll();
    {
        const int first = m.listScroll().first;
        const float y = m.remapRowY(2) + 10.0f;
        m.pointer(720.0f, y, false);
        const int row = first + 2;
        const bool title = rows[size_t(row)].kind == rm::Row::Title;
        expect(title || (m.focus_ == row && m.remap_.column == 0), "hover: row " + std::to_string(row) + ", keyboard column");
        m.pointer(900.0f, y, false);
        expect(title || m.remap_.column == 1, "hover over the pad cell: pad column");
        m.pointer(900.0f, y, true);
        expect(title || m.remap_.capture, "click on the selected cell: capture");
        m.pointer(500.0f, 100.0f, true);
        expect(!m.remap_.capture, "a click elsewhere cancels the capture");
    }
    // ---- wheel: the window moves, the selection stays in it and never on a title
    m.focus_ = 1;
    m.syncScroll();
    m.update(0.6f);
    int onTitle = 0, moves = 0;
    for (int i = 0; i < 60; ++i) {
        const int was = m.listScroll().first;
        m.wheel(1);
        moves += m.listScroll().first != was;
        onTitle += rows[size_t(m.focus_)].kind == rm::Row::Title;
        const auto& w = m.listScroll();
        onTitle += m.focus_ < w.first || m.focus_ >= w.first + w.visible;
    }
    expect(m.listScroll().first == int(rows.size()) - 5 && moves > 30 && onTitle == 0,
           "wheel: down to the end (" + std::to_string(moves) + " moves), selection always shown, never a title");
    m.wheel(-100);
    expect(m.listScroll().first == 0, "wheel up to the top");
    // ---- live serial and save (Back = keep + save, 0x4ccfd0)
    const int serial = m.optionsSerial();
    press(MenuKey::Back);
    expect(m.screen_ == Menu::Screen::Tab && m.ctrlMenu_, "Back: saved, the controller menu (state 0x29)");
    {
        const std::string t = fileText();
        expect(t.find("controls = port") != std::string::npos && t.find("# a comment") != std::string::npos && t.find("my_key = 7") != std::string::npos,
               "settings.txt keeps controls = port, comments and unknown keys");
        expect(t.find("key.ACCEL = s") != std::string::npos && t.find("key.BRAKE = w, up") != std::string::npos &&
                   t.find("key.HANDBRAKE = q") != std::string::npos && t.find("pad.ACCEL = LT") != std::string::npos &&
                   t.find("pad.BRAKE = RT") != std::string::npos,
               "saved as key.<ACTION> / pad.<ACTION>:\n" + t);
        // MENU_VALID went K -> Return: FUEL's capture replaces the whole binding, so Enter / Space are no longer on it.
        expect(t.find("key.nitro") == std::string::npos && t.find("key.NITRO") == std::string::npos &&
                   t.find("key.MENU_VALID = return\n") != std::string::npos && t.find("key.MAP_CURSOR_DOWN") == std::string::npos,
               "lines equal to the defaults are not written (key.nitro = e dropped, MAP_CURSOR_DOWN back on K), MENU_VALID = return:\n" + t);
        expect(serial > 0, "every rebind moved the options serial (sessions and prompts reload)");
    }
    {  // a new menu reads them back
        Settings s1 = loadSettings();
        s1.dataDir = dataDir;
        Menu m2(dataDir, s1, MenuMode::Main);
        const auto& b = m2.bindings();
        expect(b.layout == "port" && b.keysOf(in::Accel) == std::vector<int>{keyCode("s")} && b.padsOf(in::Accel)[0] == in::PadLT &&
                   b.padsOf(in::Brake)[0] == in::PadRT,
               "read back by a new menu (controls = port kept)");
    }
    // ---- reset per device (MENU_RESET -> FUEL's Yes / No popup [port], Yes -> the default map of that device)
    m.focus_ = 0, press(MenuKey::Accept);  // LAYOUT
    m.remap_.column = 0;
    typeKey("backspace");  // MENU_RESET on the remap page (before Back)
    expect(m.confirm_.open && m.confirm_.key == "TT_REMAP_RESET_KEYBOARD" && m.screen_ == Menu::Screen::OptRemap,
           "Backspace = MENU_RESET: the keyboard reset popup, the page stays");
    press(MenuKey::Left), press(MenuKey::Accept);  // Yes
    expect(m.bindings().keysOf(in::Accel)[0] == keyCode("w") && m.bindings().padsOf(in::Accel)[0] == in::PadLT,
           "keyboard reset: ACCEL on W again, its pad binding (LT) kept");
    m.remap_.column = 1;
    m.focus_ = int(rows.size()) - 1;
    press(MenuKey::Accept);  // the Reset row, pad cell
    expect(m.confirm_.open && m.confirm_.key == "TT_REMAP_RESET_PAD", "the Reset row's pad cell: the pad reset popup");
    press(MenuKey::Left), press(MenuKey::Accept);
    expect(m.bindings().padsOf(in::Accel)[0] == in::PadRT, "pad reset: ACCEL on RT again");
    press(MenuKey::Back);
    {
        const std::string t = fileText();
        expect(t.find("key.") == std::string::npos && t.find("pad.") == std::string::npos && t.find("controls = port") != std::string::npos,
               "everything at its default: no key.* / pad.* lines left, controls = port");
    }
    // ---- the help texts rebuild with the live bindings (pad-ux's path)
    {
        Settings ls = m.liveSettings();
        ls.all["pad.MENU_VALID"] = "X";
        HelpTexts h;
        h.load(dataDir);
        std::string t = "STR_MENU_VALID OK";
        h.transText(t);
        h.rebuild(true, keyboardLabels(dataDir, ls.all), padLabels(dataDir, ls.all));
        expect(h.expand(t).find(pad::iconText(in::PadX)) != std::string::npos, "pad prompt of MENU_VALID follows pad.MENU_VALID = X");
    }
    // ---- map: the wheel zooms (pad-ux's open item)
    {
        Menu mm(dataDir, s0, MenuMode::Pause);
        mm.showPage("MAP");
        if (mm.map_.open) {
            const float z0 = mm.map_.zoomTarget;
            mm.wheel(-2);
            const float zin = mm.map_.zoomTarget;
            mm.wheel(4);
            expect(zin < z0 && mm.map_.zoomTarget > z0, "map: wheel up zooms in, down out (" + std::to_string(int(z0)) + " -> " +
                                                            std::to_string(int(zin)) + " -> " + std::to_string(int(mm.map_.zoomTarget)) + ")");
            mm.wheel(-200);
            expect(mm.map_.zoomTarget == 1500.0f, "map wheel zoom clamped at PP340 (1500)");
        } else {
            expect(false, "map page open for the wheel zoom check");
        }
    }
    return bad;  // the scratch $FUEL_SETTINGS / $FUEL_CAREER_SAVE stay set: later checks never reach the player's files
}

bool Menu::remapShot(const std::string& args) {
    goTab(TabOptions);
    remapOpen();
    remap_.clock = 1.0f;  // past FUEL's 0.5 s gate
    std::vector<std::string> a;
    for (size_t p = 0; p <= args.size();) {
        size_t q = args.find(':', p);
        if (q == std::string::npos) q = args.size();
        if (q > p) a.push_back(args.substr(p, q - p));
        p = q + 1;
    }
    if (!a.empty()) {
        const int act = in::actionByName(a[0]);
        for (size_t i = 0; i < remap_.rows.size(); ++i)
            if ((act >= 0 && remap_.rows[i].action == act) || (act < 0 && upperOf(a[0]) == "RESET" && remap_.rows[i].kind == rm::Row::Reset))
                focus_ = int(i);
        syncScroll();
    }
    if (a.size() > 1) remap_.column = (a[1] == "p" || a[1] == "P") ? 1 : 0;
    for (size_t i = 2; i < a.size(); ++i) {
        const std::string& op = a[i];
        if (op == "capture") remapAccept();
        else if (op == "reset") remapResetAsk(remap_.column);
        else if (op.rfind("bind=", 0) == 0) {
            remapAccept();
            const std::string v = op.substr(5);
            if (remap_.column == 0) {
                rawKey(static_cast<unsigned short>(in::macKeyByName(v)), false);
            } else {
                in::PadState rest, press;
                rest.connected = press.connected = true;
                const int code = in::padByName(upperOf(v));
                if (code >= in::PadLXNeg) {
                    const float s = 1.0f;
                    switch (code) {
                        case in::PadLXNeg: press.lx = -s; break;
                        case in::PadLXPos: press.lx = s; break;
                        case in::PadLYPos: press.ly = s; break;
                        case in::PadLYNeg: press.ly = -s; break;
                        case in::PadRXNeg: press.rx = -s; break;
                        case in::PadRXPos: press.rx = s; break;
                        case in::PadRYPos: press.ry = s; break;
                        default: press.ry = -s; break;
                    }
                } else if (code == in::PadLT) press.lt = 1;
                else if (code == in::PadRT) press.rt = 1;
                else if (code >= 0) press.buttons[size_t(code)] = true;
                padFrame(rest, 0.016f), padFrame(press, 0.016f);
            }
        }
    }
    return true;
}

}  // namespace fuel::ui
