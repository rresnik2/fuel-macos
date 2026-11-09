// P101: the in-game settings pages (notes/p101-settings-menu.md).
//
// - Options > GRAPHICS [port]: a page built like FUEL's PageOptionsDisplay (its row 1 items cloned 49 px apart, the
//   list box's arrows on the focused row, the same no-wrap Up/Down, action key = keep + save, Back = FUEL's popup
//   TT_AUDIO_DISPLAY_CONFIRMATION). Rows: full screen, resolution, dynamic resolution, its lowest scale, upscaler,
//   shadow edges, post effects, grass, debris motion. The Display page is back to FUEL's six rows.
// - Options > AUDIO: a third row (the P100 limiter, `audio_limiter`) cloned from the music row.
// - Options > CONTROLS (state 0x29, PageOptionsController: LAYOUT = the remap page (remap_page.mm), SETTINGS) > SETTINGS (state 0x2c,
//   PageOptionsControllerSettings) [O, FUEL.exe]:
//     0x491910 (state 0x29 action): SETTINGS -> +0x450 = +0x454 = 0, 0x4946c0, state 0x2c; LAYOUT -> 0x4cc340 (remap).
//     0x4946c0: snapshot +0x42c = vibration (game +0x272), +0x42d = camera X (+0x271), +0x42e = camera Y (+0x2a0 != 0),
//       the list boxes set from them (0x495670(0) for the vibration row), row focus 0x4950f0.
//     0x492670 (state 0x2c keys): pointer over ITEM_ID_VALID_BOX<n> focuses row n (table 0xa66348); Back -> state 0xd5
//       (the popup, 0x495ca0: Yes 0x492f00 = 0x495c40 restore + state 0x29, No 0x496100 = state 0x2c); the action key
//       -> +0x80 = 1 (save), state 0x29; Up while row > +0x418 (the first enabled row, 0x4919e0), Down while row < 2
//       (no wrap); Left / Right = the list box prev / next (0x613bd0 / 0x613bf0: list boxes wrap, +0x124 = 1 from
//       0x49dda0) then 0x495260(-1 / +1).
//     0x495260: the focused item among VIBRATION_STATE / CAMERA_X_AXIS / CAMERA_Y_AXIS (table 0xa66480): 0 ->
//       0x495670(d); 1 -> +0x271 = entry != TT_NORMAL; 2 -> +0x2a0 = entry != TT_NORMAL.
//     0x495670(d): d != 0 -> +0x454 = +0x450 [U: a clock of the page, unused by the port]; +0x272 off -> on:
//       0x557d80 (manager on), cooldown +0x40 = 0, 0x557b40(player pad, IMPACT_MEDIUM, priority 2, scale 1.0) = the
//       test pulse; on -> off: 0x557c90 (manager off, stop). Then the list shows TT_ON / TT_OFF from +0x272.
//     0x495c40 (restore): +0x272 = +0x42c with 0x557c90 / 0x557d80 to match (no pulse), +0x271, +0x2a0.
//   Port rows below FUEL's three (row 2's items cloned 49 px apart): controls layout (port / FUEL), adaptive
//   triggers (off / light / full), trigger extras (none / all / the file's own list).
// The values apply as they change (FUEL applies its options live too); a session follows Menu::optionsSerial()
// (main.mm, game_menu.mm). settings.txt is written on the action key with the other options (saveOptions).
#include "ui/menu.h"

#include "core/career.hpp"
#include "core/vibration.hpp"
#include "ui/pad_prompts.hpp"  // pad-ux
#include "ui/ui_renderer.h"
#include "ui/hud_math.hpp"  // [nitro]

#import <Cocoa/Cocoa.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <fstream>
#include <set>
#include <sstream>
#include <unistd.h>

namespace fuel::ui {

namespace {

constexpr int kGraphicsRows = 12;  // PHYSIQUE: + Rider jiggle
// pad-ux: FUEL's Display rows are 49 apart; the page scrolls (ListScroll) instead of squeezing its rows to 42 (the
// eleven rows ran under the help bar). Seven rows fit between the title bar and BandeauHelp (597).
constexpr float kGraphicsStep = 49.0f;
constexpr int kGraphicsVisible = 7;
constexpr int kControllerRows = 10;  // FUEL's 3 + the port's 7 (pad-ux: button prompts; [nitro]: Nitrous; [upgrades U3]: Upgrades; [dev mode]: Dev mode, the last)
constexpr int kControllerVisible = 6;  // FUEL's three rows + three more (296 .. 538); the seventh scrolls in
// The controller page's window slots: FUEL's rows (296, 342, 391) then 49 apart.
constexpr float kControllerSlotY[kControllerVisible] = {296.0f, 342.0f, 391.0f, 440.0f, 489.0f, 538.0f};
constexpr float kControllerFuelY[3] = {296.0f, 342.0f, 391.0f};
const char* const kGraphicsLabels[kGraphicsRows] = {"Full screen", "Resolution",   "Dynamic resolution", "Lowest resolution", "Upscaler",
                                                    "Shadow edges", "Post effects", "Film grain",         "Grass",             "Debris motion",
                                                    "Soften",      "Rider jiggle"};
const char* const kControllerLabels[7] = {"Controls layout", "Adaptive triggers", "Trigger effects", "Button prompts", "Nitrous", "Upgrades", "Dev mode"};
// Rows whose change waits for the next start (the window is made once per process).
constexpr int kRowResolution = 1;

std::string resName(int w, int h) { return std::to_string(w) + " x " + std::to_string(h); }

// The trigger extras values: none, all, and the file's own list when it is neither.
std::vector<std::string> extrasChoices(const std::string& start) {
    std::vector<std::string> v = {"none", "all"};
    if (start != "none" && start != "all" && !start.empty()) v.push_back(start);
    return v;
}

int wrap(int i, int n) { return n <= 0 ? 0 : ((i % n) + n) % n; }

}  // namespace

// GRAPHICS in the Options left menu [port]: a copy of DISPLAY's button; FUEL's list is anchored at its last row
// (CREDITS 567), so CONTROLS and DISPLAY move up one row (37 px, x - 2 along the slant) and GRAPHICS takes DISPLAY's place.
void Menu::addGraphicsItem() {
    Page* p = scripts_.page("Options");
    if (!p || p->find("ITEM_ID_TOPMENU_OPTIONS_GRAPHICS")) return;
    const Item* disp = p->find("ITEM_ID_TOPMENU_OPTIONS_DISPLAY");
    if (!disp) return;
    Item g = *disp;
    g.id = "ITEM_ID_TOPMENU_OPTIONS_GRAPHICS";
    g.hasDynText = true, g.dynText = "GRAPHICS";
    for (Item& it : p->items)
        if (it.id == "ITEM_ID_TOPMENU_OPTIONS_CONTROLLER" || it.id == "ITEM_ID_TOPMENU_OPTIONS_DISPLAY") it.y -= 37.0f, it.x -= 2.0f;
    p->items.push_back(g);
}

Settings Menu::liveSettings() const {
    Settings s = settings_;
    port_.store(s.all);
    s.all["music_volume"] = std::to_string(options_.musicVolume);
    if (bindLinesSet_) {  // [port] remap: the page's key.* / pad.* lines replace the file's
        for (auto it = s.all.begin(); it != s.all.end();)
            it = fuel::input::InputMap::isBindingKey(it->first) ? s.all.erase(it) : std::next(it);
        for (const auto& [k, v] : bindLines_) s.all[k] = v;
    }
    return s;
}

int Menu::optionRows(Screen s) const {
    switch (s) {
        case Screen::OptDisplay: return 6;
        case Screen::OptAudio: return 3;
        case Screen::OptGraphics: return kGraphicsRows;
        case Screen::OptController: return kControllerRows;
        case Screen::OptRemap: return int(remap_.rows.size());  // [port] remap
        default: return 0;
    }
}

std::vector<std::pair<int, int>> Menu::resolutions() const {
    std::vector<std::pair<int, int>> v = {{1280, 720}, {1280, 800}, {1440, 900}, {1600, 900}, {1600, 1000}, {1920, 1080},
                                          {1920, 1200}, {2560, 1440}, {2560, 1600}};
    if (NSScreen* sc = NSScreen.mainScreen) {  // the screen's own pixel size (fullscreen at native)
        const NSSize px = [sc convertRectToBacking:sc.frame].size;
        v.emplace_back(int(px.width), int(px.height));
    }
    v.emplace_back(settings_.width, settings_.height);
    v.emplace_back(startSettings_.width, startSettings_.height);
    std::sort(v.begin(), v.end(), [](const auto& a, const auto& b) { return a.first * a.second != b.first * b.second ? a.first * a.second < b.first * b.second : a.first < b.first; });
    v.erase(std::unique(v.begin(), v.end()), v.end());
    return v;
}

bool Menu::pendingRestart(int row) const {
    return row == kRowResolution && (settings_.width != startSettings_.width || settings_.height != startSettings_.height);
}

std::string Menu::graphicsValue(int row) const {
    const std::string yes = textOf(302), no = textOf(303);
    switch (row) {
        case 0: return settings_.fullscreen ? yes : no;
        case 1: return resName(settings_.width, settings_.height) + (pendingRestart(1) ? " (next start)" : "");  // the font has no '*' 
        case 2: return port_.dynres ? yes : no;
        case 3: return std::to_string(int(std::lround(port_.dynresMin * 100.0f))) + " %";
        case 4: return port_.upscaler == 1 ? "MetalFX" : port_.upscaler == 2 ? "MetalFX temporal" : "Sharpen";  // TAA: the third
        case 5: return port_.shadow3x3 ? "Soft (FUEL)" : "Sharp (fast)";
        case 6: return settings_.postEffects ? yes : no;
        case 7: return port_.filmGrain == 0 ? no : port_.filmGrain == 2 ? "Strong" : "Light";
        case 8: return settings_.grass ? yes : no;
        case 9: return port_.debrisFuel ? "FUEL" : "Smooth";
        case 10: return port_.soften == 0 ? no : port_.soften == 2 ? "Strong" : "Light";  // P102 soften (port)
        case 11: return port_.jiggle == 0 ? no : port_.jiggle == 2 ? "Full" : "Subtle";  // PHYSIQUE (port)
        default: return "";
    }
}

// 0x495670 (d != 0): VIBRATION flips; switched on, the test pulse is asked for (the session's PadHaptics plays it).
void Menu::vibrationSwitch(int) {
    port_.vibration = !port_.vibration;
    if (port_.vibration) ++vibTests_;
}

void Menu::optionChange(int d) {
    ++optSerial_;
    if (screen_ == Screen::OptAudio) {  // the limiter row (a two-entry list: either key flips it)
        port_.limiter = !port_.limiter;
        return;
    }
    if (screen_ == Screen::OptController) {
        switch (focus_) {
            case 0: vibrationSwitch(d); break;                   // 0x495260 case 0
            case 1: port_.invertX = !port_.invertX; break;       // case 1: +0x271 (two entries, the list wraps)
            case 2: port_.invertY = !port_.invertY; break;       // case 2: +0x2a0
            case 3: port_.portControls = !port_.portControls; break;
            case 4: port_.triggers = wrap(port_.triggers + d, 3); break;
            case 5: {
                const auto c = extrasChoices(optSnap_.port.extras);
                int i = int(std::find(c.begin(), c.end(), port_.extras) - c.begin());
                port_.extras = c[size_t(wrap(i + d, int(c.size())))];
                break;
            }
            case 6: port_.padGlyphs = wrap(port_.padGlyphs + d, 3); break;  // pad-ux: auto, playstation, xbox
            case 7: port_.nitrous = !port_.nitrous; break;  // [nitro]
            case 8: port_.upgrades = !port_.upgrades; break;  // [upgrades U3] `upgrades` (U2's key; off = FUEL's vehicles and garage)
            case 9: port_.devMode = !port_.devMode, applyDevMode(); break;  // [dev mode] `dev_mode`: the career's overlay at once
        }
        return;
    }
    switch (focus_) {  // GRAPHICS
        case 0: settings_.fullscreen = !settings_.fullscreen; break;
        case 1: {
            const auto v = resolutions();
            int i = 0;
            for (size_t k = 0; k < v.size(); ++k)
                if (v[k].first == settings_.width && v[k].second == settings_.height) i = int(k);
            i = wrap(i + d, int(v.size()));
            settings_.width = v[size_t(i)].first, settings_.height = v[size_t(i)].second;
            break;
        }
        case 2: port_.dynres = !port_.dynres; break;
        case 3: port_.dynresMin = std::clamp(std::round((port_.dynresMin + 0.05f * float(d)) * 20.0f) / 20.0f, 0.5f, 1.0f); break;  // a slider: stops
        case 4: port_.upscaler = wrap(port_.upscaler + d, 3); break;  // sharpen, MetalFX spatial, MetalFX temporal (TAA)
        case 5: port_.shadow3x3 = !port_.shadow3x3; break;
        case 6: settings_.postEffects = !settings_.postEffects; break;
        case 7: port_.filmGrain = wrap(port_.filmGrain + d, 3); break;
        case 8: settings_.grass = !settings_.grass; break;
        case 9: port_.debrisFuel = !port_.debrisFuel; break;
        case 10: port_.soften = wrap(port_.soften + d, 3); break;
        case 11: port_.jiggle = wrap(port_.jiggle + d, 3); break;  // PHYSIQUE
    }
}

// One port row in a FUEL page's style: the template row's items (those within [rowY - 16, rowY + 12] of its label's
// y, without the VALID_BOX and the list arrows) drawn `dy` lower, the halo shown when focused, the label and the list
// box's text replaced (pad-ux: an empty label / value keeps the authored one: FUEL's own rows drawn in a scroll slot).
void Menu::drawPortRow(UiRenderer& r, Page& p, const char* rowPrefix, float rowY, float dy, int row, const std::string& label,
                       const std::string& value, bool focused) {
    (void)row;
    for (const Item& src : p.items) {
        if (src.y < rowY - 16.0f || src.y > rowY + 12.0f) continue;
        if (src.id.rfind("ITEM_ID_VALID_BOX", 0) == 0 || src.id.rfind("ITEM_ID_FLECHE", 0) == 0) continue;
        Item it = src;
        if (it.id.rfind("ITEM_ID_HALO", 0) == 0) it.visible = focused, it.focus = true;
        else if (it.kind == ItemKind::ListBoxTT) {
            it.focus = focused, it.visible = true;
            if (!value.empty()) it.hasDynText = true, it.dynText = value;
        } else if (it.isText() && it.id.rfind(rowPrefix, 0) == 0) {
            it.visible = true;
            if (!label.empty()) it.hasDynText = true, it.dynText = label;
        }
        drawItem(r, it, 0, dy);
    }
}

void Menu::drawGraphics(UiRenderer& r) {
    Page* p = scripts_.page("PageOptionsDisplay");
    if (!p) return;
    syncScroll();  // pad-ux: the rows of the window (7 of 11)
    for (int s = 0; s < scroll_.visible && scroll_.first + s < kGraphicsRows; ++s) {
        const int k = scroll_.first + s;
        drawPortRow(r, *p, "ITEM_ID_DISPLAY_NAME", 222.0f, kGraphicsStep * float(s), k, kGraphicsLabels[k], graphicsValue(k), focus_ == k);
    }
    // The list box's arrows (authored on row 5 at y 419) on the focused row.
    for (const char* a : {"ITEM_ID_FLECHE_GAUCHE", "ITEM_ID_FLECHE_DROITE"})
        if (const Item* src = p->find(a)) {
            Item it = *src;
            it.y = 223.0f + kGraphicsStep * float(focus_ - scroll_.first), it.visible = true;
            drawItem(r, it, 0, 0);
        }
    drawScrollArrows(r);
    // [port] FUEL has no "next time" text: the port's note under the rows, in the label style.
    bool pending = false;
    for (int k = 0; k < kGraphicsRows; ++k) pending |= pendingRestart(k);
    if (pending)
        if (const Item* src = p->find("ITEM_ID_DISPLAY_NAME1")) {
            Item it = *src;
            it.hasDynText = true, it.dynText = "Resolution: takes effect the next time the game starts", it.w = 740, it.h = 18;
            drawItem(r, it, 0, 578.0f - 222.0f);  // pad-ux: under the scroll arrow, above BandeauHelp (597)
        }
}

void Menu::drawController(UiRenderer& r) {
    Page* p = scripts_.page("PageOptionsControllerSettings");
    if (!p) return;
    syncScroll();  // pad-ux: six of the seven rows at once
    // FUEL's rows (0x4946c0 / 0x495670(0) / 0x4950f0): the list boxes from +0x272 / +0x271 / +0x2a0, ITEM_ID_HALO<row + 1>.
    if (Item* it = p->find("ITEM_ID_VIBRATION_STATE")) it->entryIndex = port_.vibration ? 1 : 0;
    if (Item* it = p->find("ITEM_ID_DISPLAY_OPTIONS_CAMERA_X_AXIS")) it->entryIndex = port_.invertX ? 1 : 0;
    if (Item* it = p->find("ITEM_ID_DISPLAY_OPTIONS_CAMERA_Y_AXIS")) it->entryIndex = port_.invertY ? 1 : 0;
    const std::string trig[3] = {textOf(279), "LIGHT", "FULL"};
    std::string extras = port_.extras == "none" ? "NONE" : port_.extras == "all" ? "ALL" : port_.extras;
    for (char& c : extras) c = char(std::toupper(static_cast<unsigned char>(c)));
    // pad-ux: pad_glyphs; auto names the family it follows while a pad is connected.
    std::string prompts = port_.padGlyphs == 1 ? "PLAYSTATION" : port_.padGlyphs == 2 ? "XBOX" : "AUTO";
    if (port_.padGlyphs == 0 && pad::state().present) prompts += pad::state().family == pad::Family::PlayStation ? " (PS)" : " (XBOX)";
    const std::string values[7] = {port_.portControls ? "PORT" : "FUEL", trig[std::clamp(port_.triggers, 0, 2)], extras, prompts,
                                   port_.nitrous ? textOf(937) : textOf(279),    // [nitro] ON / OFF (the limiter row's texts)
                                   port_.upgrades ? textOf(937) : textOf(279),   // [upgrades U3]
                                   port_.devMode ? textOf(937) : textOf(279)};   // [dev mode]
    // The window's rows: FUEL's own rows keep their authored items (moved to their slot), the port's rows clone row 2
    // (342) as before (440, 489, 538 when not scrolled).
    for (int s = 0; s < scroll_.visible && scroll_.first + s < kControllerRows; ++s) {
        const int row = scroll_.first + s;
        if (row < 3)
            drawPortRow(r, *p, "ITEM_ID_SETTINGS_NAME", kControllerFuelY[row], kControllerSlotY[s] - kControllerFuelY[row], row, "", "", focus_ == row);
        else
            drawPortRow(r, *p, "ITEM_ID_SETTINGS_NAME", 342.0f, kControllerSlotY[s] - 342.0f, row, kControllerLabels[row - 3], values[row - 3], focus_ == row);
    }
    // The list arrows (authored on row 1 at 294, label 296) on the focused row's slot.
    const int slot = std::clamp(focus_ - scroll_.first, 0, kControllerVisible - 1);
    for (const char* a : {"ITEM_ID_FLECHE_GAUCHE", "ITEM_ID_FLECHE_DROITE"})
        if (const Item* src = p->find(a)) {
            Item it = *src;
            it.y = kControllerSlotY[slot] - 2.0f, it.visible = true;
            drawItem(r, it, 0, 0);
        }
    drawScrollArrows(r);
}

// The audio page's third row (the limiter): row 2's items 60 px lower, the volume bar replaced by ON / OFF.
void Menu::drawAudioRows(UiRenderer& r) {
    Page* p = scripts_.page("PageOptionsAudio");
    if (!p) return;
    const bool f = focus_ == 2;
    for (const Item& src : p->items) {
        if (src.y < 352.0f || src.y > 372.0f) continue;
        if (src.id.rfind("ITEM_ID_VALID_BOX", 0) == 0 || src.kind == ItemKind::FillRect) continue;
        Item it = src;
        if (it.id == "ITEM_ID_HALO2") it.visible = f, it.focus = true;
        if (it.id == "ITEM_ID_SOUND_NAME2") {
            it.hasDynText = true, it.dynText = "Limiter";
            drawItem(r, it, 0, 60.0f);
            it.x = 660.0f, it.w = 317.0f, it.dynText = port_.limiter ? textOf(937) : textOf(279), it.focus = f;
        }
        drawItem(r, it, 0, 60.0f);
    }
    if (f)
        for (const char* a : {"ITEM_ID_FLECHE_GAUCHE", "ITEM_ID_FLECHE_DROITE"})
            if (const Item* src = p->find(a)) {
                Item it = *src;
                it.y = 307.0f + 120.0f, it.visible = true;
                drawItem(r, it, 0, 0);
            }
}

// Pointer on the port's rows (FUEL's own rows keep the VALID_BOX path in Menu::pointer): a row focuses, a click on the
// focused row changes it (as the Display page's port rows did), the arrows step. pad-ux: on the scrolled pages the rows
// are the window's slots (row = first + slot) and FUEL's scroll arrows scroll by one row.
bool Menu::optionsPointer(float x, float y, bool click) {
    if (screen_ == Screen::OptGraphics || screen_ == Screen::OptController) {
        syncScroll();
        if (scrollArrowHit(x, y, click)) return true;
        const float left = screen_ == Screen::OptGraphics ? 266.0f : 250.0f, right = screen_ == Screen::OptGraphics ? 1014.0f : 1020.0f;
        if (x < left || x > right) return false;
        for (int s = 0; s < scroll_.visible && scroll_.first + s < scroll_.count; ++s) {
            const float y0 = scrollRowY(s) - 4.0f;  // the VALID_BOX / row band starts 3-4 above the label
            if (y < y0 || y >= y0 + 30.0f) continue;
            const int row = scroll_.first + s;
            const bool was = focus_ == row;
            if (!was) soundBit(4);  // 0x46c510: the pointer moved the selection
            focus_ = row;
            if (click && was) optionChange(x < 650.0f ? -1 : 1);
            return true;
        }
        return false;
    }
    if (screen_ == Screen::OptAudio) {
        const float top = 366.0f + 60.0f;
        if (x < 242 || x > 1012 || y < top || y >= top + 30.0f) return false;
        const bool was = focus_ == 2;
        focus_ = 2;
        if (click && was) optionChange(x < 650.0f ? -1 : 1);
        return true;
    }
    return false;
}

// ---- pad-ux [port]: the scrolled option pages -----------------------------------------------------------------------

int Menu::scrollVisible(Screen s) const {
    switch (s) {
        case Screen::OptGraphics: return kGraphicsVisible;
        case Screen::OptController: return kControllerVisible;
        case Screen::OptRemap: return 5;  // [port] remap: FUEL's FLECHEUP 216 .. FLECHEDOWN 503 over 50-unit rows from 250
        default: return 0;
    }
}

void Menu::syncScroll() {
    const int vis = scrollVisible(screen_);
    if (!vis) {
        scroll_ = ListScroll{};
        scroll_.count = 0;
        return;
    }
    const int n = optionRows(screen_);
    if (scroll_.count != n || scroll_.visible != vis) scroll_.set(n, vis);
    scroll_.follow(std::clamp(focus_, 0, n - 1));
}

float Menu::scrollRowY(int slot) const {
    if (screen_ == Screen::OptController) return kControllerSlotY[std::clamp(slot, 0, kControllerVisible - 1)];
    if (screen_ == Screen::OptRemap) return remapRowY(slot);  // [port] remap
    return 222.0f + kGraphicsStep * float(slot);
}

// FUEL's own scroll arrows: FLECHEUP / FLECHEDOWN as PageOptionsControllerConfigChange places them for its controls list
// (x 624, 32 x 21; 34 above the first row, 3 below the last row's 50-high box): above the window's first label and
// below its last row, shown while rows lie that way.
void Menu::drawScrollArrows(UiRenderer& r) {
    if (scroll_.count <= scroll_.visible) return;
    Page* p = scripts_.page("PageOptionsControllerConfigChange");
    const int last = std::min(scroll_.visible, scroll_.count) - 1;
    for (int up = 0; up < 2; ++up) {
        const bool shown = up ? scroll_.moreUp() : scroll_.moreDown();
        const Item* src = p ? p->find(up ? "ITEM_ID_FLECHE_UP" : "ITEM_ID_FLECHE_DOWN") : nullptr;
        if (!shown || !src) continue;
        Item it = *src;
        it.visible = true;
        if (screen_ != Screen::OptRemap) it.y = up ? scrollRowY(0) - 34.0f : scrollRowY(last) + 34.0f;  // remap: FUEL's own places
        drawItem(r, it, 0, 0);
    }
}

bool Menu::scrollArrowHit(float x, float y, bool click) {
    if (scroll_.count <= scroll_.visible) return false;
    const int last = std::min(scroll_.visible, scroll_.count) - 1;
    for (int up = 0; up < 2; ++up) {
        const float ay = screen_ == Screen::OptRemap ? (up ? 216.0f : 503.0f) : up ? scrollRowY(0) - 34.0f : scrollRowY(last) + 34.0f;
        // the arrow (624, 32 x 21) with a few units of slack: it is small
        if (x < 616.0f || x > 664.0f || y < ay - 4.0f || y > ay + 25.0f) continue;
        if (!(up ? scroll_.moreUp() : scroll_.moreDown())) return false;
        if (click) {
            const int before = focus_;
            scroll_.scroll(up ? -1 : 1, focus_);
            if (focus_ != before) soundBit(4);
        }
        return true;
    }
    return false;
}

void Menu::wheel(int rows) {
    if (rows == 0 || alertUp() || tutorialOpen() || confirm_.open) return;
    if (screen_ == Screen::OptRemap && remap_.capture) return;  // [port] remap: 0x4cc870 reads nothing else while capturing
    // [port] remap (pad-ux's open item): the wheel zooms the Map page, a notch = 1.2 x (about what a zoom key held 0.2 s
    // does, 0x482eb0: target +-= displayed x dt); wheel up zooms in. Clamped like the keys (PP340 1500 .. PP320 140000).
    if (screen_ == Screen::Tab && tab_ == TabMap && map_.open && infoBox_.empty() && !mapData_) {
        map_.zoomTarget = std::clamp(map_.zoomTarget * std::pow(1.2f, float(rows)), 1500.0f, 140000.0f);
        return;
    }
    if (screen_ == Screen::GarageUpgrades && infoBox_.empty() && upg_.scroll.count > upg_.scroll.visible) {
        // [upgrades U3] the category rows, when they do not fit: the window moves, the selection is pulled inside it (else
        // the wheel = Up / Down below, as on FUEL's own lists)
        const auto cats = upgradeRows();
        int row = int(std::find(cats.begin(), cats.end(), upg_.cat) - cats.begin());
        upg_.scroll.scroll(rows, row);
        if (row >= 0 && row < int(cats.size()) && cats[size_t(row)] != upg_.cat) {
            upg_.cat = cats[size_t(row)], upg_.stage = career_ ? career_->upgradesEquipped(vehicle_)[upg_.cat] : 0;
            soundBit(4);
        }
        return;
    }
    syncScroll();
    if (scroll_.count > 0) {  // the scrolled pages: the window moves, the selection stays inside it
        const int before = focus_;
        scroll_.scroll(rows, focus_);
        if (screen_ == Screen::OptRemap) remapAfterScroll(rows);  // [port] remap: never on a title row
        if (focus_ != before) soundBit(4);
        return;
    }
    const MenuKey k = rows > 0 ? MenuKey::Down : MenuKey::Up;
    for (int i = 0; i < std::abs(rows); ++i) {
        // FUEL's paged lists keep Up / Down inside a page (0x47dfc0 / 0x47e9f0 / 0x47e610 / 0x4762b0); the wheel turns the
        // page at its ends [port]: down -> the next page's first row (FUEL's Right), up -> the previous page's last row.
        const bool paged = screen_ == Screen::CareerList || screen_ == Screen::ChallengeList || screen_ == Screen::CampList ||
                           (screen_ == Screen::Tab && tab_ == TabEditor && ed_.state >= 0xa1 && ed_.state <= 0xa4);
        const bool editor = screen_ == Screen::Tab && tab_ == TabEditor;
        const int before = editor ? ed_.row : focus_;
        key(k);
        const int after = editor ? ed_.row : focus_;
        if (!paged || after != before) continue;
        key(rows > 0 ? MenuKey::Right : MenuKey::Left);
        if (rows < 0) {  // FUEL's Left lands on the page's first row: the wheel wants its last
            if (editor) {
                const int per = ed_.perPage;
                if ((editor ? ed_.row : focus_) != before) ed_.row = per - 1;
            } else if (focus_ != before) {
                const int perPage = screen_ == Screen::CareerList ? 3 : screen_ == Screen::ChallengeList ? 5 : 10;
                focus_ = perPage - 1;
                if (screen_ == Screen::CampList)  // the last shown camp row (0x47e610 counts them from the top)
                    for (focus_ = 0; focus_ + 1 < perPage && campShown(focus_ + 1);) ++focus_;
                listFirst_ = 0;
            }
        }
    }
}
// ---- end pad-ux

// ---- --menu-check -------------------------------------------------------------------------------------------------

int runOptionsCheck(const std::string& dataDir) {
    int bad = 0;
    auto expect = [&](bool ok, const std::string& what) {
        if (!ok) ++bad;
        std::printf("options check: %s %s\n", ok ? "ok  " : "FAIL", what.c_str());
    };
    // A scratch folder: the player's own settings.txt and career.sav are never read or written here.
    char tmpl[] = "/tmp/fuel-options-check-XXXXXX";
    const char* dir = mkdtemp(tmpl);
    if (!dir) return 1;
    const std::string settingsFile = std::string(dir) + "/settings.txt", careerFile = std::string(dir) + "/career.sav";
    setenv("FUEL_SETTINGS", settingsFile.c_str(), 1);
    setenv("FUEL_CAREER_SAVE", careerFile.c_str(), 1);
    {
        std::ofstream f(settingsFile);
        f << "# the player's comment\ndata = " << dataDir << "\nmy_unknown_key = 42\nvibration = on\nadaptive_triggers_extras = gear,air\n"
          << "width = 1600\nheight = 1000\n";
    }
    expect(settingsPath() == settingsFile, "settings path follows $FUEL_SETTINGS");
    // PortOptions alone: every value through store -> from.
    {
        int okN = 0, n = 0;
        for (int m = 0; m < 4096; m += 37) {
            PortOptions o;
            o.dynres = m & 1, o.upscaler = (m >> 1) % 3, o.shadow3x3 = m & 4, o.debrisFuel = m & 8, o.limiter = m & 16, o.vibration = m & 32;
            o.invertX = m & 64, o.invertY = m & 128, o.portControls = m & 256, o.triggers = (m >> 9) % 3, o.extras = (m & 2048) ? "all" : "gear,air";
            o.dynresMin = 0.5f + 0.05f * float(m % 11);
            o.filmGrain = m % 3;
            o.padGlyphs = (m / 7) % 3;  // pad-ux
            o.nitrous = (m / 3) & 1;    // [nitro]
            o.upgrades = (m / 5) & 1;   // [upgrades U3]
            o.devMode = (m / 11) & 1;   // [dev mode]
            std::map<std::string, std::string> all;
            o.store(all);
            ++n, okN += PortOptions::from(all) == o;
        }
        expect(okN == n, "port options round-trip through their keys (" + std::to_string(okN) + "/" + std::to_string(n) + ")");
    }
    Settings s0 = loadSettings();
    s0.dataDir = dataDir;
    Menu m(dataDir, s0, MenuMode::Main);
    m.setMode(MenuMode::Main);
    m.goTab(Menu::TabOptions);
    auto press = [&](MenuKey k, int n = 1) {
        for (int i = 0; i < n; ++i) m.applyItemStates(), m.key(k);  // a frame's item states before each key, as drawn
    };
    auto names = [&] {
        m.applyItemStates();
        std::string out;
        for (Item* it : m.focusables()) out += (out.empty() ? "" : ",") + (it->hasDynText ? it->dynText : m.textOf(it->tt));
        return out;
    };
    expect(names() == "CONTROLS,DISPLAY,GRAPHICS,AUDIO,BONUS CODES,CREDITS", "Options left menu: " + names());
    // ---- GRAPHICS
    m.focus_ = 2, press(MenuKey::Accept);
    expect(m.screen_ == Menu::Screen::OptGraphics && m.focus_ == 0, "GRAPHICS opens on row 0");
    press(MenuKey::Up);
    expect(m.focus_ == 0, "Up on the first row stays (no wrap)");
    press(MenuKey::Down, 20);
    expect(m.focus_ == kGraphicsRows - 1, "Down stops on the last row (" + std::to_string(m.focus_) + ")");
    const int serial0 = m.optionsSerial();
    m.focus_ = 10, press(MenuKey::Right);  // soften off -> light
    m.focus_ = 9, press(MenuKey::Right);  // debris -> fuel
    m.focus_ = 7, press(MenuKey::Right);  // film grain light -> strong
    m.focus_ = 2, press(MenuKey::Right);  // dynres off
    m.focus_ = 3, press(MenuKey::Left);   // 0.8 -> 0.75
    m.focus_ = 4, press(MenuKey::Right);  // metalfx
    expect(m.graphicsValue(4) == "MetalFX", "Upscaler: Sharpen -> MetalFX (" + m.graphicsValue(4) + ")");
    press(MenuKey::Right);                 // TAA: metalfx-temporal
    expect(m.graphicsValue(4) == "MetalFX temporal" && PortOptions::from(m.liveSettings().all).upscaler == 2 &&
               m.liveSettings().all.at("dynamic_resolution_upscaler") == "metalfx-temporal",
           "Upscaler: MetalFX -> MetalFX temporal (live: dynamic_resolution_upscaler = metalfx-temporal)");
    press(MenuKey::Right), press(MenuKey::Left);  // wraps to Sharpen and back
    expect(m.graphicsValue(4) == "MetalFX temporal", "Upscaler: the list wraps (" + m.graphicsValue(4) + ")");
    press(MenuKey::Left);                  // back to metalfx (the checks below expect it)
    m.focus_ = 5, press(MenuKey::Left);   // 3x3
    m.focus_ = 6, press(MenuKey::Right);  // post effects off
    m.focus_ = 1, press(MenuKey::Right);  // a resolution above 1600 x 1000
    expect(m.optionsSerial() == serial0 + 13, "each change moves the live serial");
    {
        const PortOptions live = PortOptions::from(m.liveSettings().all);
        expect(live.soften == 1 && live.debrisFuel && live.filmGrain == 2 && !live.dynres && std::fabs(live.dynresMin - 0.75f) < 1e-4f && live.upscaler == 1 && live.shadow3x3,
               "live settings carry the page's values");
        expect(!m.liveSettings().postEffects && m.pendingRestart(1), "post effects live; the resolution waits for the next start");
    }
    m.focus_ = 11, press(MenuKey::Right);  // PHYSIQUE: rider jiggle subtle -> full (the last row)
    expect(m.graphicsValue(11) == "Full" && m.liveSettings().all.at("rider_jiggle") == "full", "Rider jiggle: Subtle -> Full (live: rider_jiggle = full)");
    m.focus_ = 3, press(MenuKey::Left, 20);
    expect(std::fabs(m.port_.dynresMin - 0.5f) < 1e-4f, "lowest resolution stops at 50 %");
    // Back -> the popup, No focused first: the page stays; Yes restores.
    press(MenuKey::Back);
    expect(m.confirm_.open && m.confirm_.focus == 1 && m.confirm_.key == "TT_AUDIO_DISPLAY_CONFIRMATION", "Back opens FUEL's popup on No");
    press(MenuKey::Accept);
    expect(!m.confirm_.open && m.screen_ == Menu::Screen::OptGraphics, "No keeps the page");
    press(MenuKey::Back), press(MenuKey::Left), press(MenuKey::Accept);
    expect(m.screen_ == Menu::Screen::Tab && m.tab_ == Menu::TabOptions, "Yes returns to the options menu");
    expect(m.port_ == PortOptions::from(s0.all) && m.settings_.postEffects && m.settings_.width == 1600, "Yes restores the values");
    // Change and keep.
    m.openList(Menu::Screen::OptGraphics);
    m.focus_ = 2, press(MenuKey::Right);
    m.focus_ = 5, press(MenuKey::Right);
    m.focus_ = 7, press(MenuKey::Left);   // film grain off
    m.focus_ = 8, press(MenuKey::Right);  // grass off
    m.focus_ = 9, press(MenuKey::Right);  // debris fuel
    press(MenuKey::Accept);
    {
        const Settings rs = loadSettings();
        const PortOptions ro = PortOptions::from(rs.all);
        expect(!ro.dynres && ro.shadow3x3 && ro.debrisFuel && !rs.grass && ro.filmGrain == 0, "GRAPHICS values saved and read back");
        std::ifstream f(settingsFile);
        std::stringstream t;
        t << f.rdbuf();
        const std::string txt = t.str();
        expect(txt.find("# the player's comment") != std::string::npos && txt.find("my_unknown_key = 42") != std::string::npos,
               "unknown keys and comments kept");
        expect(txt.find("vibration = on") != std::string::npos, "an unchanged key keeps the player's spelling");
        expect(txt.find("dynamic_resolution = off") != std::string::npos && txt.find("shadow_filter = 3x3") != std::string::npos &&
                   txt.find("dynamic_resolution_upscaler") == std::string::npos,
               "changed keys written, default ones not added");
    }
    // ---- DISPLAY: FUEL's six rows, no wrap.
    m.openList(Menu::Screen::OptDisplay);
    press(MenuKey::Down, 10);
    expect(m.focus_ == 5, "Display: six rows, Down stops on Brightness");
    m.focus_ = 0, press(MenuKey::Right), press(MenuKey::Accept);
    expect(!loadMenuOptions().raceInfo, "Display: race info saved");
    // ---- AUDIO: SFX, music, limiter.
    m.openList(Menu::Screen::OptAudio);
    press(MenuKey::Down, 5);
    expect(m.focus_ == 2, "Audio: three rows, Down stops on the limiter");
    press(MenuKey::Right);
    expect(!m.port_.limiter && !PortOptions::from(m.liveSettings().all).limiter, "Audio: limiter off (live)");
    m.focus_ = 0, press(MenuKey::Left, 3);
    m.focus_ = 1, press(MenuKey::Left, 2);
    press(MenuKey::Accept);
    {
        const Settings rs = loadSettings();
        const MenuOptions mo = loadMenuOptions();
        expect(rs.all.count("audio_limiter") && rs.all.at("audio_limiter") == "off", "Audio: audio_limiter = off saved");
        // From FUEL's factory 8/16 (kFuelDefaultVolume, P103): SFX three steps down, music two.
        const float d = std::round(kFuelDefaultVolume * 16.0f);
        expect(std::fabs(rs.volume - (d - 3.0f) / 16.0f) < 1e-4f && std::fabs(mo.musicVolume - (d - 2.0f) / 16.0f) < 1e-4f,
               "Audio: volumes saved in 1/16 steps");
    }
    // ---- CONTROLS (state 0x29) and SETTINGS (0x2c).
    m.goTab(Menu::TabOptions);
    m.focus_ = 0, press(MenuKey::Accept);
    expect(m.ctrlMenu_ && m.leftPage() && m.leftPage()->name == "PageOptionsController", "CONTROLS opens the controller menu (0x29)");
    expect(names() == "LAYOUT,SETTINGS", "controller menu: LAYOUT ([port] remap, no longer parked), SETTINGS (" + names() + ")");
    m.focus_ = 1, press(MenuKey::Accept);
    expect(m.screen_ == Menu::Screen::OptController && m.focus_ == 0, "SETTINGS opens the settings page (0x2c) on row 0");
    press(MenuKey::Up);
    expect(m.focus_ == 0, "Up on row 0 stays (row > +0x418)");
    press(MenuKey::Down, 10);
    expect(m.focus_ == kControllerRows - 1, "Down stops on the last row");
    m.focus_ = 0;
    const int tests0 = m.vibrationTests();
    press(MenuKey::Right);
    expect(!m.port_.vibration && m.vibrationTests() == tests0, "VIBRATION off: no pulse (0x557c90)");
    press(MenuKey::Left);
    expect(m.port_.vibration && m.vibrationTests() == tests0 + 1, "VIBRATION on: the test pulse (0x495670)");
    m.focus_ = 1, press(MenuKey::Right), m.focus_ = 2, press(MenuKey::Left);
    expect(m.port_.invertX && m.port_.invertY, "camera axes flip (0x495260 cases 1, 2)");
    m.focus_ = 3, press(MenuKey::Right);
    m.focus_ = 4, press(MenuKey::Left);
    expect(m.port_.portControls && m.port_.triggers == 1, "controls layout and trigger level");
    m.focus_ = 5;
    std::string seen = m.port_.extras;
    for (int i = 0; i < 3; ++i) press(MenuKey::Right), seen += ">" + m.port_.extras;
    expect(seen == "gear,air>none>all>gear,air", "trigger effects cycle keeps the file's own list (" + seen + ")");
    m.focus_ = 8, press(MenuKey::Right);  // [upgrades U3]
    expect(!m.port_.upgrades && m.liveSettings().all.at("upgrades") == "off", "Upgrades off (live: upgrades = off)");
    press(MenuKey::Left);
    expect(m.port_.upgrades, "Upgrades back on");
    m.focus_ = 9, press(MenuKey::Right);  // [dev mode] the last row
    expect(m.port_.devMode && m.liveSettings().all.at("dev_mode") == "on" && m.career() && m.career()->devMode(),
           "Dev mode on (live: dev_mode = on; the career's overlay on at once)");
    press(MenuKey::Left);
    expect(!m.port_.devMode && m.liveSettings().all.at("dev_mode") == "off" && !m.career()->devMode(), "Dev mode back off (the overlay off)");
    press(MenuKey::Right);  // on again: Yes below restores it
    m.focus_ = 7, press(MenuKey::Right);  // [nitro]
    expect(!m.port_.nitrous && m.liveSettings().all.at("nitrous") == "off", "Nitrous off (live: nitrous = off)");
    press(MenuKey::Left);
    expect(m.port_.nitrous, "Nitrous back on");
    press(MenuKey::Right);
    press(MenuKey::Back), press(MenuKey::Left), press(MenuKey::Accept);  // Yes: 0x492f00
    expect(m.screen_ == Menu::Screen::Tab && m.ctrlMenu_ && !m.port_.invertX && !m.port_.portControls && m.port_.triggers == 2 && m.port_.nitrous &&
               !m.port_.devMode && !m.career()->devMode(),
           "Yes restores (0x495c40; dev mode off again, the overlay too) and returns to the controller menu (0x29)");
    m.focus_ = 1, press(MenuKey::Accept);  // SETTINGS (LAYOUT is row 0)
    m.focus_ = 0, press(MenuKey::Right);  // vibration off
    m.focus_ = 2, press(MenuKey::Right);  // camera Y inverted
    m.focus_ = 4, press(MenuKey::Left, 2);  // triggers off
    m.focus_ = 6, press(MenuKey::Right);    // pad-ux: button prompts auto -> playstation
    m.focus_ = 7, press(MenuKey::Right);    // [nitro] nitrous off
    m.focus_ = 9, press(MenuKey::Right);    // [dev mode] on
    press(MenuKey::Accept);
    expect(m.screen_ == Menu::Screen::Tab && m.ctrlMenu_, "the action key returns to the controller menu (0x29)");
    press(MenuKey::Back);
    expect(!m.ctrlMenu_ && m.tab_ == Menu::TabOptions, "Back in the controller menu -> the options menu (state 6)");
    {
        const PortOptions ro = PortOptions::from(loadSettings().all);
        expect(!ro.vibration && ro.invertY && ro.triggers == 0 && !ro.nitrous, "controller values saved and read back (nitrous = off)");
        std::ifstream f(settingsFile);
        std::stringstream t;
        t << f.rdbuf();
        expect(ro.padGlyphs == 1 && t.str().find("pad_glyphs = playstation") != std::string::npos, "pad-ux: pad_glyphs = playstation saved and read back");
        expect(ro.devMode && t.str().find("dev_mode = on") != std::string::npos, "[dev mode] dev_mode = on saved and read back");
        Career c(loadCareerRules(dataDir));
        const bool loaded = c.load();
        expect(loaded && c.state().options[0x26] == 0 && c.state().options2a0 == 1 && c.state().options[0x25] == 0,
               "career.sav: +0x272 / +0x271 / +0x2a0 at their places (scratch save)");
    }
    // ---- The test pulse as PadHaptics::testPulse plays it in the pause menu (pause = 0x557c90: stop + cooldown 1 s;
    // 0x495670: manager on, cooldown 0, IMPACT_MEDIUM priority 2): FUEL's manager on the shipped shapes, no device.
    {
        vib::Shapes shapes = vib::Shapes::load(dataDir);
        vib::Manager vm(&shapes);
        vm.option = false, vm.setActive(false);  // paused, option off
        const bool refusedOff = !vm.add("IMPACT_MEDIUM", 2, 1.0f);
        vm.option = true, vm.setActive(true);
        const bool refusedCooldown = !vm.add("IMPACT_MEDIUM", 2, 1.0f);  // still in the stop's cooldown without 0x495670's reset
        vm.clearCooldown();
        const bool started = vm.add("IMPACT_MEDIUM", 2, 1.0f);
        int frames = 0, firstLow = -1;
        for (int i = 0; i < 60; ++i) {
            const vib::Motors mo = vm.update(1.0f / 60);
            if (i == 0) firstLow = mo.low;
            if (mo.low || mo.high) ++frames;
        }
        expect(shapes.error.empty() && refusedOff && refusedCooldown && started && firstLow > 200 && frames >= 20 && frames <= 25,
               "test pulse: IMPACT_MEDIUM after the cooldown reset, " + std::to_string(frames) + " frames, first left motor " + std::to_string(firstLow));
    }
    // ---- [nitro] The nitrous gauge right of the speedometer dial (hud::nitroBottle from the compteur dial's snapped rect)
    // clears every item of FUEL's in-race / free-roam HUD pages. UiRenderer letterboxes the 1280 x 720 canvas the same at
    // 16:9 and 16:10 (full width, centred), so these canvas rects hold for both.
    {
        const Page* cp = m.scripts_.page("compteur");
        const Item* dial = nullptr;
        if (cp)
            for (const Item& it : cp->items)
                if (it.id.rfind("ITEM_ID_COMPTEUR", 0) == 0 && it.id.size() == 18) dial = &it;
        expect(dial != nullptr, "nitro gauge: the compteur dial item");
        if (dial) {
            float dx = 0, dy = 0;
            Menu::itemRect(*dial, dx, dy);
            const hud::Rect g = hud::nitroBounds(hud::nitroBottle(dx, dy, dial->w, dial->h));
            static const char* kPages[] = {"compteur", "degat", "boussole", "classement", "MissionInfoAtoBRace", "MissionInfoCheckpointRace",
                                           "MissionInfoCircuitRace", "MissionInfoKnockOutRace", "MissionInfoTimeTrialRace",
                                           "MissionInfoTimeTrialCheckpointRace", "MissionInfoCatchIARace", "MissionInfoHelicoRace", "PressStart",
                                           "StatusPress", "NewChallengeDetected", "NewLiveryDetected", "NewVistaPointDetected", "NewLiveryDiscovered",
                                           "NewVistaPointDiscovered", "infofuel", "BANDEAU_DOPPLER_LIVERY", "BANDEAU_DOPPLER_VISTAPT",
                                           "BANDEAU_DOPPLER_CHALLENGE", "BANDEAU_DOPPLER_BONUS_CAR"};
            // (economy_hud is a menu page: drawn only with the menus open, when the session draws no HUD, game_menu.mm)
            int items = 0;
            std::string hits;
            for (const char* name : kPages) {
                const Page* pg = m.scripts_.page(name);
                if (!pg) continue;
                for (const Item& it : pg->items) {
                    if (it.w <= 0 || it.h <= 0) continue;
                    float x = 0, y = 0;
                    Menu::itemRect(it, x, y);
                    ++items;
                    if (x < g.x + g.w && g.x < x + it.w && y < g.y + g.h && g.y < y + it.h)
                        hits += std::string(hits.empty() ? "" : ", ") + name + ":" + it.id;
                }
            }
            char buf[200];
            std::snprintf(buf, sizeof buf, "nitro gauge (%.0f, %.0f, %.0f x %.0f) clear of %d HUD items at 16:9 and 16:10", g.x, g.y, g.w, g.h, items);
            expect(hits.empty() && items > 50, std::string(buf) + (hits.empty() ? "" : ": overlaps " + hits));
        }
    }
    // ---- In a session: the pause menu reaches Options with the tab keys.
    {
        Menu pm(dataDir, loadSettings(), MenuMode::Pause);
        pm.setSession("", "", "QUAD_ROAD_2");
        pm.setMode(MenuMode::Pause);
        int steps = 0;
        while (pm.tab_ != Menu::TabOptions && steps < 10) pm.key(MenuKey::TabRight), ++steps;
        expect(pm.tab_ == Menu::TabOptions, "pause menu: Options is " + std::to_string(steps) + " tab(s) to the right");
        pm.focus_ = 2, pm.key(MenuKey::Accept);
        expect(pm.screen_ == Menu::Screen::OptGraphics, "pause menu: GRAPHICS opens");
    }
    std::remove(settingsFile.c_str()), std::remove(careerFile.c_str()), rmdir(dir);
    std::printf("options check: %s (%d failures)\n", bad ? "FAILED" : "ok", bad);
    return bad;
}


// ---- pad-ux --menu-check: scrolling ----------------------------------------------------------------------------------

int runScrollCheck(const std::string& dataDir) {
    int bad = 0;
    auto expect = [&](bool ok, const std::string& what) {
        if (!ok) ++bad;
        std::printf("scroll check: %s %s\n", ok ? "ok  " : "FAIL", what.c_str());
    };
    char tmpl[] = "/tmp/fuel-scroll-check-XXXXXX";
    const char* dir = mkdtemp(tmpl);
    if (!dir) return 1;
    const std::string settingsFile = std::string(dir) + "/settings.txt", careerFile = std::string(dir) + "/career.sav";
    setenv("FUEL_SETTINGS", settingsFile.c_str(), 1);
    setenv("FUEL_CAREER_SAVE", careerFile.c_str(), 1);
    { std::ofstream(settingsFile) << "data = " << dataDir << "\ncontrols = port\n"; }
    Settings s0 = loadSettings();
    s0.dataDir = dataDir;
    Menu m(dataDir, s0, MenuMode::Main);
    m.setMode(MenuMode::Main);
    auto press = [&](MenuKey k, int n = 1) {
        for (int i = 0; i < n; ++i) m.applyItemStates(), m.key(k);
    };
    auto win = [&] {
        const ListScroll& w = m.listScroll();
        return std::to_string(w.first) + "+" + std::to_string(w.visible) + "/" + std::to_string(w.count) + " sel " + std::to_string(m.focus_);
    };
    auto shown = [&] {  // the selection inside the window
        const ListScroll& w = m.listScroll();
        return m.focus_ >= w.first && m.focus_ < w.first + w.visible && m.focus_ < w.count;
    };
    // ---- GRAPHICS: 12 rows (PHYSIQUE: + Rider jiggle), 7 at once, FUEL's 49 apart.
    m.goTab(Menu::TabOptions);
    m.openList(Menu::Screen::OptGraphics);
    {
        const ListScroll& w = m.listScroll();
        expect(w.first == 0 && w.visible == 7 && w.count == 12 && !w.moreUp() && w.moreDown(), "GRAPHICS opens: window 0+7/12, down arrow only (" + win() + ")");
    }
    bool always = true;
    std::string firsts;
    for (int i = 0; i < 13; ++i) {
        press(MenuKey::Down);
        always = always && shown();
        firsts += std::to_string(m.listScroll().first);
    }
    expect(always && firsts == "0000001234555" && m.focus_ == 11, "Down: the window moves only when the selection leaves it (firsts " + firsts + ")");
    expect(!m.listScroll().moreDown() && m.listScroll().moreUp(), "at the end: up arrow only");
    firsts.clear();
    for (int i = 0; i < 12; ++i) {
        press(MenuKey::Up);
        always = always && shown();
        firsts += std::to_string(m.listScroll().first);
    }
    expect(always && firsts == "555555432100" && m.focus_ == 0, "Up: the same rule back (firsts " + firsts + ")");
    // The wheel moves the window, the selection is pulled inside it.
    m.wheel(1);
    expect(m.listScroll().first == 1 && m.focus_ == 1 && shown(), "wheel down 1: window 1, selection pulled to row 1 (" + win() + ")");
    m.wheel(10);
    expect(m.listScroll().first == 5 && m.focus_ == 5, "wheel down 10: clamped at the last window (" + win() + ")");
    m.focus_ = 10;
    m.wheel(-3);
    expect(m.listScroll().first == 2 && m.focus_ == 8, "wheel up 3 from row 10: window 2, selection 8 (" + win() + ")");
    m.wheel(-5);
    expect(m.listScroll().first == 0 && m.focus_ == 6 && shown(), "wheel up to the top: selection the window's last row (" + win() + ")");
    // The pointer after a scroll: window slot k is row first + k.
    m.wheel(3);
    {
        const int first = m.listScroll().first;
        m.pointer(400, 222.0f + 2.0f * 49.0f + 8.0f, false);
        expect(first == 3 && m.focus_ == 5, "hover slot 2 with window 3 -> row 5 (" + win() + ")");
        const std::string before = m.graphicsValue(7);
        m.pointer(400, 222.0f + 4.0f * 49.0f + 8.0f, false);
        m.pointer(800, 222.0f + 4.0f * 49.0f + 8.0f, true);
        expect(m.focus_ == 7 && m.graphicsValue(7) != before, "click the focused slot 4 -> row 7 (film grain) changes: " + before + " -> " + m.graphicsValue(7));
        m.pointer(640, 222.0f - 34.0f + 10.0f, true);
        expect(m.listScroll().first == 2 && shown(), "click FUEL's up arrow: one row up (" + win() + ")");
        for (int k = 0; k < 3; ++k) m.pointer(640, 222.0f + 6.0f * 49.0f + 34.0f + 10.0f, true);
        expect(m.listScroll().first == 5 && shown(), "click the down arrow three times: window 5 (" + win() + ")");
        m.pointer(640, 222.0f + 6.0f * 49.0f + 34.0f + 10.0f, true);
        expect(m.listScroll().first == 5, "the down arrow at the end does nothing");
        m.pointer(400, 222.0f - 4.0f + 2.0f, false);
        expect(m.focus_ == 5, "hover the top slot -> row 5 (" + win() + ")");
    }
    press(MenuKey::Back), press(MenuKey::Left), press(MenuKey::Accept);  // Yes: restore
    // ---- CONTROLS > SETTINGS: FUEL's three rows + seven port rows ([dev mode]: Dev mode last), six at once.
    m.goTab(Menu::TabOptions);
    m.openList(Menu::Screen::OptController);
    expect(m.listScroll().count == 10 && m.listScroll().visible == 6 && m.listScroll().first == 0, "CONTROLLER: 10 rows, 6 at once (" + win() + ")");
    press(MenuKey::Down, 6);
    expect(m.focus_ == 6 && m.listScroll().first == 1 && shown(), "Down to button prompts: the window moves one row (" + win() + ")");
    m.pointer(400, 296.0f + 2.0f, false);
    expect(m.focus_ == 1, "hover slot 0 with window 1 -> FUEL's camera X row (" + win() + ")");
    const bool invX = m.port_.invertX;
    m.pointer(800, 296.0f + 2.0f, true);
    expect(m.port_.invertX != invX, "click on it flips camera X (0x495260 case 1)");
    m.pointer(400, 538.0f + 2.0f, false);
    std::string cycle = std::to_string(m.port_.padGlyphs);
    for (int i = 0; i < 3; ++i) press(MenuKey::Right), cycle += ">" + std::to_string(m.port_.padGlyphs);
    expect(m.focus_ == 6 && cycle == "0>1>2>0", "button prompts: auto > playstation > xbox > auto (" + cycle + ")");
    expect(PortOptions::from(m.liveSettings().all).padGlyphs == 0 && m.liveSettings().all.at("pad_glyphs") == "auto", "pad_glyphs live (auto)");
    press(MenuKey::Down);  // [nitro]
    expect(m.focus_ == 7 && m.listScroll().first == 2 && shown(), "Down to Nitrous: window 2 (" + win() + ")");
    press(MenuKey::Down);  // [upgrades U3]
    expect(m.focus_ == 8 && m.listScroll().first == 3 && shown(), "Down to Upgrades: window 3 (" + win() + ")");
    press(MenuKey::Down);  // [dev mode]
    expect(m.focus_ == 9 && m.listScroll().first == 4 && shown(), "Down to Dev mode (the last row): window 4 (" + win() + ")");
    press(MenuKey::Down);
    expect(m.focus_ == 9 && m.listScroll().first == 4, "Down on the last row stays (no wrap)");
    press(MenuKey::Up, 9);
    expect(m.focus_ == 0 && m.listScroll().first == 0, "Up to vibration: window 0");
    press(MenuKey::Back), press(MenuKey::Left), press(MenuKey::Accept);
    // ---- Pages that do not scroll keep FUEL's rules (no window).
    m.openList(Menu::Screen::OptDisplay);
    expect(m.listScroll().count == 0, "DISPLAY (FUEL's six rows) does not scroll");
    m.wheel(2);
    expect(m.focus_ == 2, "the wheel on DISPLAY = Down twice (row 2)");
    press(MenuKey::Back), press(MenuKey::Left), press(MenuKey::Accept);
    // ---- FUEL's own paging, every entry reachable by keys and by the wheel (all hubs unlocked, all challenges listed).
    Career* c = m.career();
    if (c) {
        for (auto& h : c->state().hubs) h.unlocked = 1, h.opened = 1;
        for (auto& hub : c->state().challenges)
            for (auto& st : hub) st.available = 1;
        // Camps: two pages of ten (0x481580 / 0x47e610).
        m.goTab(Menu::TabMap);
        m.openList(Menu::Screen::CampList);
        m.focus_ = 0, m.listFirst_ = 0;
        std::set<int> keysSeen{0}, wheelSeen{0};
        for (int i = 0; i < 12; ++i) press(MenuKey::Down), keysSeen.insert(m.focus_);
        press(MenuKey::Right), keysSeen.insert(m.focus_);
        for (int i = 0; i < 12; ++i) press(MenuKey::Down), keysSeen.insert(m.focus_);
        m.focus_ = 0, m.listFirst_ = 0;
        for (int i = 0; i < 25; ++i) m.wheel(1), wheelSeen.insert(m.focus_);
        const int lastByWheel = m.focus_;
        for (int i = 0; i < 25; ++i) m.wheel(-1), wheelSeen.insert(m.focus_);
        const int hubs = int(m.hubs_.size());
        expect(int(keysSeen.size()) == hubs && int(wheelSeen.size()) == hubs && lastByWheel == hubs - 1 && m.focus_ == 0,
               "camps: " + std::to_string(hubs) + " hubs, keys reach " + std::to_string(keysSeen.size()) + ", the wheel " + std::to_string(wheelSeen.size()) +
                   " (down to the last, up to the first)");
        // Races (3 a page) and challenges (5 a page): every hub's list, both ways.
        int hubsOk = 0, maxCareer = 0, maxChallenge = 0, hubsTried = 0;
        for (size_t h = 0; h < m.hubs_.size(); ++h) {
            m.hubIndex_ = int(h);
            bool ok = true;
            for (const bool career : {true, false}) {
                const auto list = m.hubEvents(career ? Event::Career : Event::Challenge);
                const int n = int(list.size()), per = career ? 3 : 5;
                (career ? maxCareer : maxChallenge) = std::max(career ? maxCareer : maxChallenge, n);
                if (n == 0) continue;
                m.openList(career ? Menu::Screen::CareerList : Menu::Screen::ChallengeList);
                std::set<int> byKeys{0}, byWheel{0};
                for (int pg = 0; pg < 2; ++pg) {
                    for (int i = 0; i < per + 1; ++i) press(MenuKey::Down), byKeys.insert(m.focus_);
                    press(MenuKey::Right), byKeys.insert(m.focus_);
                }
                m.focus_ = 0, m.listFirst_ = 0;
                for (int i = 0; i < n + 2; ++i) m.wheel(1), byWheel.insert(m.focus_);
                const bool lastOk = m.focus_ == std::min(n, 2 * per) - 1 && m.listFirst_ == (m.focus_ / per) * per;
                for (int i = 0; i < n + 2; ++i) m.wheel(-1), byWheel.insert(m.focus_);
                const int reach = std::min(n, 2 * per);  // FUEL's two pages
                ok = ok && int(byKeys.size()) == reach && int(byWheel.size()) == reach && lastOk && m.focus_ == 0;
                if (n > 2 * per) std::printf("scroll check: note: %s has %d %s, FUEL's two pages show %d\n", m.hubs_[h].id.c_str(), n,
                                             career ? "races" : "challenges", 2 * per);
            }
            ++hubsTried, hubsOk += ok;
        }
        expect(hubsOk == hubsTried, "race / challenge lists: every entry of " + std::to_string(hubsOk) + "/" + std::to_string(hubsTried) +
                                        " hubs reached by keys and by the wheel (most: " + std::to_string(maxCareer) + " races, " +
                                        std::to_string(maxChallenge) + " challenges)");
        // Profile completion lines: FUEL's 6-line scroll (0x496d00) by keys and by the wheel.
        m.goTab(Menu::TabProfile), m.profileEnter(0x24);
        const int lines = int(m.profileLines().size());
        press(MenuKey::Down, lines + 3);
        const int byKeys = m.profile_.scroll;
        m.profile_.scroll = 0;
        m.wheel(lines + 3);
        const int byWheel = m.profile_.scroll;
        m.wheel(-(lines + 3));
        expect(byKeys == std::max(0, lines - 6) && byWheel == byKeys && m.profile_.scroll == 0,
               "profile completion: " + std::to_string(lines) + " lines, scroll stops at count - 6 = " + std::to_string(byKeys) + " (wheel " +
                   std::to_string(byWheel) + ")");
    }
    // Edited races (0x4762b0): ten used slots, FUEL's pages of the list page's rows, by keys and by the wheel.
    if (c) {
        auto& b = c->state().editedRaces;
        for (size_t i = 0; i < fuel::EditedRace::kSlots && (i + 1) * fuel::EditedRace::kBytes <= b.size(); ++i) b[i * fuel::EditedRace::kBytes + 0x10] = 2;
        m.goTab(Menu::TabEditor);
        m.editorSetState(0xa2);  // EDIT RACE list
        m.ed_.row = 0;
        std::set<int> byKeys{0}, byWheel{0};
        // 0x4762b0: Right keeps the row on the next page; Up / Down stay inside the page.
        for (int i = 0; i < m.ed_.perPage + 1; ++i) press(MenuKey::Down), byKeys.insert(m.ed_.row);
        press(MenuKey::Right), byKeys.insert(m.ed_.row);
        for (int i = 0; i < m.ed_.perPage + 1; ++i) press(MenuKey::Up), byKeys.insert(m.ed_.row);
        m.ed_.row = 0;
        for (int i = 0; i < 12; ++i) m.wheel(1), byWheel.insert(m.ed_.row);
        const int last = m.ed_.row;
        for (int i = 0; i < 12; ++i) m.wheel(-1), byWheel.insert(m.ed_.row);
        expect(byKeys.size() == 10 && byWheel.size() == 10 && last == 9 && m.ed_.row == 0,
               "edited races: 10 slots, " + std::to_string(m.ed_.perPage) + " a page, keys reach " + std::to_string(byKeys.size()) + ", the wheel " +
                   std::to_string(byWheel.size()) + " (down to slot 10, up to slot 1)");
    }
    // Ranking (0x4b1aa0): seven racers, five rows.
    m.showPage("RANKINGSCROLL");
    m.wheel(6);
    const int sel = m.rankSelected(), row = m.rankHighlight();
    m.wheel(-6);
    expect(sel == 6 && row == 5 && m.rankSelected() == 0 && m.rankHighlight() == 1, "ranking: the wheel = FUEL's Up / Down (sel 6 on row 5, back to 0 on row 1)");
    std::remove(settingsFile.c_str()), std::remove(careerFile.c_str()), rmdir(dir);
    std::printf("scroll check: %s (%d failures)\n", bad ? "FAILED" : "ok", bad);
    return bad;
}

}  // namespace fuel::ui
