// PK25: FUEL's in-game alert dialogs and the MCAlert save/load-error flows (see alerts.h, notes/pk25.md).
#include "ui/alerts.h"

#include "ui/menu.h"
#include "core/save_import.hpp"

#include "ui/tsc.hpp"
#include "ui/ui_renderer.h"

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sys/stat.h>
#include <unistd.h>

namespace fuel::ui {
namespace {

constexpr float kW = 1280, kH = 720;  // the canvas (0x49bec0); dialog coordinates are fractions of it

std::string upperOf(std::string s) {
    for (char& c : s) c = char(std::toupper(static_cast<unsigned char>(c)));
    return s;
}

// "r255g255b255a255" (MENUSTyle colours) -> 0..1.
Rgba styleColour(const std::string& s) {
    Rgba c{1, 1, 1, 1};
    int r = 255, g = 255, b = 255, a = 255;
    if (std::sscanf(s.c_str(), "r%dg%db%da%d", &r, &g, &b, &a) >= 3) c = Rgba{r / 255.0f, g / 255.0f, b / 255.0f, a / 255.0f};
    return c;
}

}  // namespace

// ---- the script -----------------------------------------------------------------------------------------------------

const DialogDef* DialogScript::find(const std::string& name) const {
    for (const DialogDef& d : dialogs)
        if (upperOf(d.name) == upperOf(name)) return &d;
    return nullptr;
}

DialogScript loadDialogScript(const std::string& dataDir, const std::string& file) {
    DialogScript ds;
    TscReader reader((std::filesystem::path(dataDir) / "CommonComponent").string());
    std::vector<DialogWindow*> stack;  // open MENUWindow blocks
    DialogDef* dlg = nullptr;
    reader.run(file, {}, [&](const TscLine& l) {
        const std::string cmd = upperOf(l.args[0]);
        const auto& a = l.args;
        auto f = [&](size_t i) { return i < a.size() ? float(std::atof(a[i].c_str())) : 0.0f; };
        if (cmd == "MENUSTYLETEXTSTRUCT" && a.size() >= 5) {
            DialogTextStyle t;
            t.scale = f(2), t.big = upperOf(a[4]) == "BIG";
            if (a.size() > 5) t.fill = styleColour(a[5]);
            if (a.size() > 6) t.outline = styleColour(a[6]);
            ds.textStyles[upperOf(a[1])] = t;
        } else if (cmd == "MENUSTYLEBOX" && a.size() >= 3) {
            DialogBoxStyle b;
            b.type = a[2];
            if (a.size() > 3) b.colour = styleColour(a[3]);
            ds.boxStyles[upperOf(a[1])] = b;
        } else if (cmd == "MENUDIALOG" && a.size() >= 6) {
            ds.dialogs.push_back(DialogDef{a[1], f(2), f(3), f(4), f(5), {}});
            dlg = &ds.dialogs.back();
            stack.clear();
        } else if (cmd == "MENUENDDIALOG") {
            dlg = nullptr, stack.clear();
        } else if (cmd == "MENUWINDOW" && a.size() >= 7 && dlg) {
            DialogWindow w;
            w.type = a[1], w.name = a[2], w.x = f(3), w.y = f(4), w.w = f(5), w.h = f(6);
            auto& list = stack.empty() ? dlg->windows : stack.back()->children;
            list.push_back(w);
            stack.push_back(&list.back());
        } else if (cmd == "MENUENDWINDOW") {
            if (!stack.empty()) stack.pop_back();
        } else if (!stack.empty() && cmd.rfind("MENUPROPERTY", 0) == 0) {
            DialogWindow& w = *stack.back();
            const std::string v = a.size() > 1 ? a[1] : std::string();
            if (cmd == "MENUPROPERTYTEXTSTYLE") w.textStyle = upperOf(v);
            else if (cmd == "MENUPROPERTYBOXSTYLE") w.boxStyle = upperOf(v);
            else if (cmd == "MENUPROPERTYTEXT" && !v.empty()) w.tt = uint32_t(std::atoi(v.c_str()));
            else if (cmd == "MENUPROPERTYTEXTALIGN") w.align = upperOf(v);
            else if (cmd == "MENUPROPERTYOUTLINED") w.outlined = upperOf(v) == "TRUE";
            else if (cmd == "MENUPROPERTYIMAGE") w.image = v;
            else if (cmd == "MENUPROPERTYCOLOR" && a.size() >= 5) w.colour = Rgba{f(1), f(2), f(3), f(4)}, w.hasColour = true;  // PK26: hasColour
            else if (cmd == "MENUPROPERTYTEXTSCALE" && !v.empty()) w.textScale = f(1);         // PK26
            else if (cmd == "MENUPROPERTYTEXTSHADOWED") w.shadowed = upperOf(v) == "TRUE";  // PK26
        }
    });
    for (const std::string& p : reader.problems) ds.problems.push_back(p);
    if (ds.dialogs.empty()) ds.problems.push_back("no MENUDialog in " + file);
    return ds;
}

// ---- Dialog_MCAlert -------------------------------------------------------------------------------------------------

void McAlert::show() {
    // BaseAlert bind 0x43fe50: input lock on, timer 0 (keys are taken 0.4 s after the box appears) [O]; vt+0x6c state 1.
    lock_ = true, timer_ = 0;
    valid_ = back_ = false;
    shown_ = true;
}

bool McAlert::saveError(int code) {  // 0x441320
    switch (code) {
        case 9:
        case 10:
        case 0xc:
        case 0xd:
            setMessage(9024);  // 0x2340
            setButtons(489);   // 0x1e9: "STR_MENU_VALID Yes²²²STR_MENU_BACK No"
            break;
        case 0xb:
        case 0x1b:
            break;  // [O] shown with whatever texts the box holds
        case 0xe: {
            // 0x441760(TT 0x2355 = 9045, 0x118): "Not enough free space for saves. %dkB needed." -> 280 kB; the
            // message window's string (+0xe8) overrides its TT. Buttons TT 0x239e = 9118 (absent in PC tt01 -> empty).
            const std::string fmt = textOf ? textOf(9045) : std::string();
            std::string msg = fmt;
            if (const size_t p = fmt.find("%d"); p != std::string::npos) msg = fmt.substr(0, p) + std::to_string(0x118) + fmt.substr(p + 2);
            messageText_ = msg;
            setButtons(9118);
            break;
        }
        default: return false;
    }
    state_ = 1, answer_ = 1, mode_ = 1, code_ = code;
    if (host.pause) host.pause();  // 0x441670
    show();
    return true;
}

bool McAlert::loadError(int code) {  // 0x4411a0 (jump table 0x441308 indexed by code - 5)
    if (code == 0x17) setMessage(9113);       // 0x2399 "Load Failed. The saved game data is corrupt. ... overwrite ...?"
    else if (code == 0x1c) setMessage(9034);  // 0x234a "... created by another user ... overwrite this save?"
    else return false;
    setButtons(489);
    state_ = 1, answer_ = 2, mode_ = 0, code_ = code;  // no 0x441670 here: a load error does not pause [O]
    show();
    return true;
}

bool McAlert::noDevice(int arg) {  // 0x441540
    setMessage(9038);  // 0x234e (absent in PC tt01)
    setButtons(490);   // 0x1ea "STR_MENU_VALID OK"
    answer_ = arg, state_ = 1, mode_ = 2, code_ = 1;
    if (host.pause) host.pause();  // 0x441670
    show();
    return true;
}

void McAlert::key(bool valid, bool back) {
    valid_ = valid_ || valid;
    back_ = back_ || back;
}

bool McAlert::pressed(bool& which) {  // 0x4400c0: an action pressed while the lock is off -> lock again for 0.4 s
    if (lock_ || !which) return false;
    which = false;
    lock_ = true, timer_ = 0;
    return true;
}

void McAlert::close(bool keepAnswer) {  // 0x440b50
    if (!keepAnswer) answer_ = 0;
    mode_ = 6, state_ = 7;
    shown_ = false;  // vt+0x6c -> 2 (hide)
    // "UnPause" when the pause page set is closed and the HUD shows a race/end-race screen (0x4b8950 +0xd6c in 6..0x1c,
    // 0x26, 0x27): the host knows both [port].
    if (host.unpause) host.unpause();
}

bool McAlert::result(int* code, int* answer) {  // 0x440cd0
    *code = code_, *answer = answer_;
    if (state_ == 7) {
        answer_ = 0, state_ = 0;
        return true;
    }
    return false;
}

void McAlert::update(float dt) {  // Dialog_MCAlert vf16 0x4410d0
    if (lock_ && (timer_ += dt) > 0.4f) lock_ = false;
    // NetPlayManager vt+0x68(0) (GFWL sign-in changed) -> code 0x1d, close(0): Games for Windows - LIVE, dropped.
    switch (mode_) {
        case 0: mode_ = 3; break;
        case 1: mode_ = 4; break;
        case 2: mode_ = 5; break;
        case 3: loadErrorFlow(); break;
        case 4: saveErrorFlow(); break;
        case 5: noDeviceFlow(); break;
        default: break;
    }
    valid_ = back_ = false;  // the action keys are per frame
}

void McAlert::loadErrorFlow() {  // 0x440f90
    if (code_ == 0x17) {  // 0x440d10
        auto no = [&] {   // LAB_00440d98: Back -> autosave off, "AUTOSAVE DISABLED. No progress will be saved." + OK
            if (!pressed(back_)) return;
            if (host.autosaveOff) host.autosaveOff();
            setMessage(9023);  // 0x233f
            setButtons(490);   // 0x1ea
            state_ = 5;
        };
        if (state_ == 1) {
            if (pressed(valid_)) return answer_ = 1, close(true);  // overwrite
            return no();
        }
        if (state_ == 5) {
            if (!pressed(valid_)) return;
            return close(false);
        }
        if (state_ == 6) {  // [O] no setter of state 6 found
            if (pressed(valid_)) {
                answer_ = 1, close(true);
                if (host.restartFromBuffer) host.restartFromBuffer();
                return;
            }
            return no();
        }
        return close(false);
    }
    if (code_ == 0x1c) {  // 0x440df0 (another user's save; the PC reader never returns 0x1c)
        if (state_ == 1) {
            if (pressed(valid_)) {
                code_ = 0, close(false);
                if (host.restartFromBuffer) host.restartFromBuffer();
                return;
            }
            if (pressed(back_)) code_ = 0, close(true);
            return;
        }
        if (state_ == 3 && !pressed(valid_)) return;
        return close(false);
    }
}

void McAlert::saveErrorFlow() {  // 0x440fb0
    switch (code_) {
        case 9:
        case 10:
        case 0xc:
        case 0xd:
            if (pressed(valid_)) return close(true);  // Yes: save again (answer 1)
            if (pressed(back_)) close(false);         // No
            return;
        case 0xe: break;
        default: return;  // [O] 0xb / 0x1b have no handler here
    }
    if (state_ == 1) {
        // Yes -> g_SaveGame vt+0x50(0x118, 0) (the storage-device selector: a no-op on PC), state 2.
        if (pressed(valid_)) return void(state_ = 2);
        if (pressed(back_)) close(false);
        return;
    }
    if (state_ == 2) {
        // vt+0x54 selector busy: false on PC; vt+0x60 device state: 0 on PC -> close(1) = save again [O]. The port has
        // no device to choose either, so this is the PC path.
        return close(true);
    }
    if (state_ == 4) {
        if (pressed(valid_)) close(false);
        return;
    }
    close(false);
}

void McAlert::noDeviceFlow() {  // 0x440ff0
    if (state_ == 1) {
        // Yes -> the Xbox storage-device selector vt+0x50(0x118, 1) [port: none on a Mac: taken as "no device chosen"
        // (vt+0x60 != 0): autosave off, TT 9023 + OK, state 5].
        if (pressed(valid_)) {
            if (host.autosaveOff) host.autosaveOff();
            setMessage(9023), setButtons(490);
            state_ = 5;
        }
        return;
    }
    if (state_ == 5 && !pressed(valid_)) return;
    close(false);
}

// ---- the dialogs (FrontGlobalInGameData_G vf24 0x442990) -------------------------------------------------------------

InGameDialogs::InGameDialogs(const std::string& dataDir, const MenuAssets& assets, std::function<std::string(uint32_t)> textOf)
    : assets_(assets), textOf_(std::move(textOf)) {
    // The styles of the boot dialogs (resrce.tsc) first: resGIGD's IGMsgBox uses text_msgBox, which only resrce.tsc defines.
    DialogScript base = loadDialogScript(dataDir, "resrce.tsc");
    script_ = loadDialogScript(dataDir, "resGIGD.tsc");
    for (const auto& [k, v] : base.textStyles) script_.textStyles.emplace(k, v);
    for (const auto& [k, v] : base.boxStyles) script_.boxStyles.emplace(k, v);
    mc_.textOf = [this](uint32_t tt) { return text(tt); };
}

std::string InGameDialogs::text(uint32_t tt) const { return tt && textOf_ ? textOf_(tt) : std::string(); }

// CBox_G 9-slice 0x439330: corner = min(style corner x W/640 x scale x 32, half the box) [O]; texture by the box type
// (the style table 0xa05848 is filled from MENUStyleBox at load) [I: the type name is the picture name; corner factor
// 1, scale 1]; UV split 0.499 / 0.501.
void InGameDialogs::drawBox(UiRenderer& r, const DialogBoxStyle& s, float x0, float y0, float x1, float y1) const {
    const BitmapRef b = assets_.image(s.type);
    const float c = 1.0f * kW / 640.0f * 1.0f * 32.0f;
    const float cx = std::min(c, (x1 - x0) * 0.5f), cy = std::min(c, (y1 - y0) * 0.5f);
    if (!b) {
        r.rect(x0, y0, x1 - x0, y1 - y0, Rgba{0, 0, 0, 0.5f * s.colour.a});  // [port] no picture: a dim box
        return;
    }
    const float xs[4] = {x0, x0 + cx, x1 - cx, x1}, ys[4] = {y0, y0 + cy, y1 - cy, y1};
    const float us[4] = {0, 0.499f, 0.501f, 1}, vs[4] = {0, 0.499f, 0.501f, 1};
    for (int j = 0; j < 3; ++j)
        for (int i = 0; i < 3; ++i)
            r.image(b, xs[i], ys[j], xs[i + 1] - xs[i], ys[j + 1] - ys[j], s.colour, us[i], vs[j], us[i + 1], vs[j + 1]);
}

void InGameDialogs::drawWindow(UiRenderer& r, const DialogWindow& w, float px, float py, float pw, float ph,
                               const std::function<std::string(const DialogWindow&)>& textFor) const {
    const float x = px + w.x * pw, y = py + w.y * ph, ww = w.w * pw, hh = w.h * ph;
    const std::string type = upperOf(w.type);
    if (type == "BITMAP") {
        if (const BitmapRef b = assets_.image(w.image)) r.image(b, x, y, ww, hh, w.colour);
    } else if (type == "BOX") {
        auto it = script_.boxStyles.find(w.boxStyle);
        if (it != script_.boxStyles.end()) drawBox(r, it->second, x, y, x + ww, y + hh);
    }
    // Pictures under the texts [I: the MCAlert lists its Background last; drawn first so the texts stay readable].
    for (const DialogWindow& c : w.children)
        if (upperOf(c.type) != "STATICTEXT") drawWindow(r, c, x, y, ww, hh, textFor);
    for (const DialogWindow& c : w.children)
        if (upperOf(c.type) == "STATICTEXT") drawWindow(r, c, x, y, ww, hh, textFor);
    if (type == "STATICTEXT") {
        // CStaticText_G vf18 0x5a05b0: the string (+0xe8) or the TT (+0xe0), button tokens swapped (0x57bdf0), drawn by
        // the rich-text engine 0x43d3c0 in the window box with the style's colour, outline (MENUPropertyOutlined) and
        // scale; SMALL font [I: MENUPropertySmallFont]; centred for Centered / CenterAligned, top of the box [I].
        const std::string s = textFor(w);
        if (s.empty()) return;
        DialogTextStyle ts;
        if (auto it = script_.textStyles.find(w.textStyle); it != script_.textStyles.end()) ts = it->second;
        TextStyle st;
        st.font = 0;
        st.height = ts.scale * assets_.font(0).refHeight;
        st.fill = ts.fill;
        st.outline = w.outlined ? ts.outline : Rgba{0, 0, 0, 0};
        const int align = w.align.find("CENT") != std::string::npos ? 1 : w.align.find("RIGHT") != std::string::npos ? 2 : 0;
        r.richText(s, x, y, x + ww, y + hh, align, st, true, true);
    }
}

void InGameDialogs::drawDialog(UiRenderer& r, const DialogDef& d, const std::function<std::string(const DialogWindow&)>& textFor) const {
    const float x = d.x * kW, y = d.y * kH, w = d.w * kW, h = d.h * kH;
    for (const DialogWindow& c : d.windows)
        if (upperOf(c.type) != "STATICTEXT") drawWindow(r, c, x, y, w, h, textFor);
    for (const DialogWindow& c : d.windows)
        if (upperOf(c.type) == "STATICTEXT") drawWindow(r, c, x, y, w, h, textFor);
}

void InGameDialogs::draw(UiRenderer& r) const {
    if (padShown_)
        if (const DialogDef* d = script_.find("PADAlert"))
            drawDialog(r, *d, [&](const DialogWindow& w) {
                // 0x4424a0: Message (+0x90, vf28 child 1) = TT 9019 with the text variable "padid" = pad + 1; Buttons (+0x94) "".
                if (upperOf(w.name) != "MESSAGE") return std::string();
                std::string s = text(9019);
                for (size_t p; (p = s.find("{padid}")) != std::string::npos;) s.replace(p, 7, std::to_string(pad_ + 1));
                return s;
            });
    if (infosShown_)
        if (const DialogDef* d = script_.find("InfosText"))
            drawDialog(r, *d, [&](const DialogWindow& w) { return upperOf(w.name) == "STATICTEXT0" ? infos_ : text(w.tt); });
    if (msgBoxShown_)
        if (const DialogDef* d = script_.find("IGMsgBox")) drawDialog(r, *d, [&](const DialogWindow& w) { return text(w.tt); });
    if (mc_.shown())
        if (const DialogDef* d = script_.find("MCAlert"))
            drawDialog(r, *d, [&](const DialogWindow& w) {
                const std::string n = upperOf(w.name);
                if (n == "MESSAGE") return mc_.messageText().empty() ? text(mc_.messageTT()) : mc_.messageText();
                if (n == "BUTTONS") return text(mc_.buttonsTT());
                return text(w.tt);
            });
}

// ---- Menu: the storage (MemoryCardManager_C) around a Mac file save -------------------------------------------------

void Menu::setAutosaveOff(bool off) {
    storage_.autosaveOff = off;
    // [port] FUEL keeps +0x75 for the run of the game; the port's sessions are exec'd processes, which inherit it.
    if (off) ::setenv("FUEL_AUTOSAVE_OFF", "1", 1);
    else ::unsetenv("FUEL_AUTOSAVE_OFF");
    std::printf("menu: autosave %s\n", off ? "disabled" : "enabled");
}

// SaveManager_LoadSlot -> Load 0x4ec1b0 -> vf17 0x4eb9c0: read, the '=' marker and CRC, apply (Career::read); a corrupt
// save (0x17) asks TT 9113 (0x4411a0) [O: only with an in-game player; the port's menus count as in game].
int Menu::loadCareerFile() {
    if (!career_) return pcsave::NotFound;
    storage_.op = 2, storage_.loadAlert = true;  // +0x3c = 2, +0x76 = 1
    const int rc = career_->read();
    storage_.loadErr = rc, storage_.loadAlert = false;  // +0x5c = err
    if (rc == 0) {
        storage_.op = 0;
        return 0;
    }
    if (rc == pcsave::Corrupt) {
        // [port] once per run of the game: after "No" (autosave off, kept in $FUEL_AUTOSAVE_OFF) the next sessions of the
        // same run load quietly, as FUEL loads once.
        if (!storage_.autosaveOff && dialogs_) storage_.loadAlert = dialogs_->mcAlert().loadError(rc);  // +0x76 = 0x4411a0(err)
        if (!storage_.loadAlert) storage_.op = 0;  // the no-player path: +0x3c = 0
        std::fprintf(stderr, "menu: career save %s is corrupt (code 0x17)%s\n", Career::careerPath().c_str(),
                     storage_.loadAlert ? ": asking (TT 9113)" : "");
    }
    return rc;  // 7 / 8: no alert, +0x3c stays 2 [O]
}

// MemoryCardManager_C vf17 0x4eb9c0, the MCAlert part: once the alert closes (0x440cd0) the operation's error takes its
// code; answer 1 saves again (SaveSlot(+0x60), after clearing a corrupt-load 0x17), 2 loads again, 0 ends the operation.
void Menu::storageFrame() {
    int code = 0, answer = 0;
    if (dialogs_->mcAlert().result(&code, &answer)) {
        if (storage_.op == 1) storage_.saveErr = code, storage_.saveAlert = false;
        else if (storage_.op == 2) storage_.loadErr = code, storage_.loadAlert = false;
        if (answer == 1) {
            if (storage_.loadErr == pcsave::Corrupt) storage_.loadErr = 0, careerUnreadable_ = false;  // "overwrite": saves may replace it
            if (storage_.slot >= 0) saveCareer();
        } else if (answer == 2) {
            loadCareerFile();
        } else {
            storage_.op = answer;
        }
        // +0x78 (quit once saved: console "Exit") is not ported: the port quits at once.
    }
    if (afterAlert_ && !storage_.saveAlert && !storage_.loadAlert && !alertUp()) {
        auto f = std::move(afterAlert_);
        afterAlert_ = {};
        f();
    }
}

bool Menu::alertShot(const std::string& n) {
    if (!dialogs_) return false;
    goTab(TabMap);  // FUEL opens the pause page set under a save alert (0x441670)
    McAlert& mc = dialogs_->mcAlert();
    if (n == "MCALERT" || n == "MCALERT_9024") return mc.saveError(pcsave::WriteFailed);
    if (n == "MCALERT_9045") return mc.saveError(pcsave::NoSpace);
    if (n == "MCALERT_9113") return mc.loadError(pcsave::Corrupt);
    if (n == "MCALERT_9034") return mc.loadError(0x1c);
    if (n == "MCALERT_9038") return mc.noDevice(1);
    if (n == "MCALERT_9023") return mc.preview(9023, 490), true;  // the corrupt-save "No" answer (0x440d10 state 5)
    if (n == "PADALERT") return dialogs_->previewPad(0), true;
    if (n == "INFOSTEXT") return dialogs_->previewInfos(textOf(9023)), true;  // [port] sample text (the show path 0x440710 is dead)
    if (n == "IGMSGBOX") return dialogs_->previewMsgBox(), true;
    return false;
}

// ---- --menu-check: the flows on a scratch career.sav ------------------------------------------------------------------

int runAlertCheck(const std::string& dataDir) {
    int bad = 0;
    auto expect = [&](bool ok, const std::string& what) {
        if (!ok) ++bad;
        std::printf("alert check: %s %s\n", ok ? "ok  " : "FAIL", what.c_str());
    };
    // A scratch folder: the player's settings.txt and career.sav are never read or written here.
    char tmpl[] = "/tmp/fuel-alert-check-XXXXXX";
    const char* dir = mkdtemp(tmpl);
    if (!dir) return 1;
    const std::string settingsFile = std::string(dir) + "/settings.txt", save = std::string(dir) + "/career.sav";
    ::setenv("FUEL_SETTINGS", settingsFile.c_str(), 1);
    ::setenv("FUEL_CAREER_SAVE", save.c_str(), 1);
    ::unsetenv("FUEL_AUTOSAVE_OFF");
    { std::ofstream(settingsFile) << "data = " << dataDir << "\n"; }
    auto readAll = [](const std::string& p) {
        std::ifstream f(p, std::ios::binary);
        return std::vector<uint8_t>(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
    };
    auto writeAll = [](const std::string& p, const std::vector<uint8_t>& b) {
        std::ofstream(p, std::ios::binary | std::ios::trunc).write(reinterpret_cast<const char*>(b.data()), std::streamsize(b.size()));
    };
    Settings s0 = loadSettings();
    s0.dataDir = dataDir;
    // Frames: the box takes keys 0.4 s after it appears and after each accepted key (0x4400c0 / 0x4410d0).
    auto frames = [](Menu& m, float t) {
        for (float k = 0; k < t; k += 0.1f) m.update(0.1f);
    };
    auto press = [&](Menu& m, MenuKey k) {
        frames(m, 0.5f);
        m.key(k);
        m.update(0.016f);
        frames(m, 0.1f);
    };
    // A valid career.sav to start from.
    {
        Menu m(dataDir, s0, MenuMode::Main);
        m.career()->state().fuel = 4321;
        m.requestSave();
        expect(m.lastSaveCode() == 0 && !m.alertUp(), "a scratch career.sav is written without an alert");
    }
    const std::vector<uint8_t> good = readAll(save);
    std::vector<uint8_t> corrupt = good;
    if (corrupt.size() > 40) corrupt[corrupt.size() / 2] ^= 0x10;
    // 1. Corrupt save, "No": autosave off (TT 9023), the file kept, saves refused, the next session quiet.
    writeAll(save, corrupt);
    {
        Menu m(dataDir, s0, MenuMode::Main);
        McAlert& mc = m.dialogs()->mcAlert();
        expect(m.alertUp() && mc.messageTT() == 9113 && mc.buttonsTT() == 489 && m.lastLoadCode() == 0x17,
               "corrupt save: MCAlert TT 9113 (0x4411a0), Yes/No TT 489, load code 0x17");
        m.key(MenuKey::Back);
        m.update(0.016f);
        expect(mc.messageTT() == 9113, "keys wait 0.4 s after the box appears (0x43fe50 input lock)");
        press(m, MenuKey::Back);
        expect(mc.shown() && mc.messageTT() == 9023 && mc.buttonsTT() == 490 && m.autosaveOff(),
               "No: autosave off (+0x75), TT 9023 AUTOSAVE DISABLED with OK (0x440d10 state 5)");
        press(m, MenuKey::Accept);
        expect(!m.alertUp(), "OK closes the box");
        m.requestSave();
        m.sessionEnd();
        const char* env = std::getenv("FUEL_AUTOSAVE_OFF");
        expect(m.lastSaveCode() == 1 && readAll(save) == corrupt && env && std::string(env) == "1",
               "autosave off: SaveManager_Save refused (+0x64 = 1, 0x4ebea0), the corrupt file untouched, kept for the next session");
    }
    {
        Menu m(dataDir, s0, MenuMode::Main);  // the next session of the same run
        expect(!m.alertUp() && m.autosaveOff(), "the next session loads quietly with autosave still off");
        // Map > DATA > SAVE: autosave back on (0x4ceaf0: TT_AUTO_SAVE_ENABLED), then the save.
        m.mapData_ = true;
        m.dataSave();
        expect(m.confirmOpen() && m.confirmText() == "TT_AUTO_SAVE_ENABLED" && !m.autosaveOff(), "DATA > SAVE: TT_AUTO_SAVE_ENABLED, autosave on");
        m.key(MenuKey::Accept);
        Career back(m.career()->rules());
        expect(m.confirmOpen() && m.confirmText() == "TT_SAVE_COMPLETED_SUCCESSFULLY" && back.read(save) == 0 && !std::getenv("FUEL_AUTOSAVE_OFF"),
               "then the save overwrites the corrupt file (TT_SAVE_COMPLETED_SUCCESSFULLY)");
    }
    // 2. Corrupt save, "Yes": the next save overwrites it (0x4eb9c0 answer 1: +0x5c 0x17 -> 0).
    writeAll(save, corrupt);
    {
        Menu m(dataDir, s0, MenuMode::Main);
        press(m, MenuKey::Accept);
        const bool kept = readAll(save) == corrupt;  // SaveSlot(+0x60) only after a save of this run (+0x60 = -1 here)
        m.requestSave();
        Career back(m.career()->rules());
        expect(!m.alertUp() && m.lastLoadCode() == 0 && kept && m.lastSaveCode() == 0 && back.read(save) == 0,
               "Yes: the corrupt save is overwritten by the next save");
    }
    // 3. A read-only save: TT 9024, the game paused (0x441670), Yes retries, No closes; the file kept.
    writeAll(save, good);
    if (::geteuid() != 0) {
        Menu m(dataDir, s0, MenuMode::Main);
        ::chmod(save.c_str(), 0444);
        m.career()->state().fuel = 99;
        m.requestSave();
        McAlert& mc = m.dialogs()->mcAlert();
        expect(m.alertUp() && mc.messageTT() == 9024 && mc.buttonsTT() == 489 && m.takeAlertPause() && m.lastSaveCode() == 9,
               "read-only save: code 9, MCAlert TT 9024 Yes/No, pause asked (0x441670)");
        press(m, MenuKey::Accept);
        expect(m.alertUp() && mc.messageTT() == 9024, "Yes: saved again (SaveSlot), failed again, asked again");
        press(m, MenuKey::Back);
        expect(!m.alertUp() && readAll(save) == good, "No: closed, the file unchanged");
        ::chmod(save.c_str(), 0644);
        press(m, MenuKey::Accept);  // nothing up: the menus have the keys again
        m.requestSave();
        expect(m.lastSaveCode() == 0 && !m.alertUp(), "writable again: the next save goes through");
    }
    // 4. Demo / tutorial mode (game+0x32bc == 2): saves refused with no operation running.
    {
        Menu m(dataDir, s0, MenuMode::Main);
        const std::vector<uint8_t> before = readAll(save);
        m.setGameMode(2);
        m.career()->state().fuel = 7;
        m.requestSave();
        expect(m.lastSaveCode() == 1 && readAll(save) == before, "game mode 2: SaveManager_Save refuses (0x4ebea0)");
    }
    // 5. The no-space alert text (0x441320 case 0xe).
    {
        Menu m(dataDir, s0, MenuMode::Main);
        McAlert& mc = m.dialogs()->mcAlert();
        mc.saveError(0xe);
        expect(mc.messageText().find("280") != std::string::npos && mc.buttonsTT() == 9118, "no space: TT 9045 with 280 kB (0x118), buttons TT 9118");
        press(m, MenuKey::Accept);
        frames(m, 0.2f);
        expect(!mc.shown(), "no space, Yes: the PC device selector is a no-op -> saved again (close(1))");
    }
    ::unsetenv("FUEL_AUTOSAVE_OFF");
    std::error_code ec;
    ::chmod(save.c_str(), 0644);
    std::filesystem::remove_all(dir, ec);
    std::printf("alert check: %s (%d failures)\n", bad ? "FAILED" : "ok", bad);
    return bad;
}

}  // namespace fuel::ui
