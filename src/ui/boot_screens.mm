// PK26: FUEL's boot screens (MenuManager_G boot dialogs), see boot_screens.h and ~/re-work/fuel/notes/pk26.md.
#include "ui/boot_screens.h"

#include "ui/loading_screen.h"
#include "ui/help_texts.hpp"
#include "ui/menu_model.hpp"
#include "ui/ui_renderer.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <utility>

namespace fuel::ui {
namespace {

constexpr float kW = 1280, kH = 720;  // the canvas; dialog coordinates are fractions of it

std::string upperOf(std::string s) {
    for (char& c : s) c = char(std::toupper(static_cast<unsigned char>(c)));
    return s;
}

// Math_ExpSmooth 0x4100b0 [O]: |target - cur| < 1e-6 -> target; else cur + (target - cur) (1 - 1 / rate^(dt x 30.30303)).
float expSmooth(float cur, float target, float rate, float dt) {
    const float d = target - cur;
    if (std::fabs(d) < 1e-6f) return target;
    return float((1.0 - 1.0 / std::pow(double(rate), double(dt) * 30.30303)) * double(d) + double(cur));
}

}  // namespace

// ---- CGuiSmoothAnimation_G ------------------------------------------------------------------------------------------

void GuiAnimation::show() {  // 0x596690 [O] (+0xab clear: the live block restarts from the hidden one)
    showing = true, hiding = false;
    live = hidden, target = shown;
}

void GuiAnimation::hide() {  // 0x596630 [O]
    hiding = true, showing = false;
    live = shown, target = hidden;
}

bool GuiAnimation::atTarget(const Block& a, const Block& b) {
    // 0x596900: fields 0..4 equal, 0x5969a0: the colour within 1e-6 (<= for the first component), field 9 equal [O].
    for (int i = 0; i < 5; ++i)
        if (a[size_t(i)] != b[size_t(i)]) return false;
    if (!(std::fabs(b[5] - a[5]) <= 1e-6f)) return false;
    for (int i = 6; i < 9; ++i)
        if (!(std::fabs(b[size_t(i)] - a[size_t(i)]) < 1e-6f)) return false;
    return a[9] == b[9];
}

bool GuiAnimation::step(float dt) {
    if (!showing && !hiding) return false;
    // vf01 0x596ce0 [O]: ExpSmooth on fields 0..4 and 9; 0x596e00 lerps r, g, b by the same factor and takes alpha as is.
    for (size_t i : {0u, 1u, 2u, 3u, 4u, 9u}) live[i] = expSmooth(live[i], target[i], rate, dt);
    const double k = 1.0 - 1.0 / std::pow(double(rate), double(dt) * 30.30303);
    for (size_t i = 5; i < 8; ++i) live[i] = float(double(target[i] - live[i]) * k + double(live[i]));
    live[8] = target[8];
    if (!atTarget(live, target)) return false;
    showing = hiding = false;  // 0x596500: no auto-reverse (+0xaa) on the boot widgets [O: only WaitingStyle sets it]
    return true;
}

// ---- the dialogs ----------------------------------------------------------------------------------------------------

BootScreens::BootScreens(const std::string& dataDir, const MenuAssets& assets, std::function<std::string(uint32_t)> textOf, Dialog first)
    : assets_(assets), textOf_(std::move(textOf)) {
    script_ = loadDialogScript(dataDir, "resrce.tsc");
    // MenuManager_G vf24 0x589180 builds the 16 dialogs; the window ids (+0x3c) count every MENUWindow of the file from 0
    // (PressStart's AutoSaveWarning 2 .. Logo 5, TRCMessage's PressX 0x9c) [O: 0x591b90 / 0x5947b0 look them up].
    const std::map<std::string, int> index = {{"BACKGROUND", Background}, {"PRESSSTART", PressStart}, {"COMPILESHADER", CompileShader},
                                              {"PROFILE", Profile}, {"LEGALTEXT", LegalText}, {"TRCMESSAGE", TRCMessage}};
    int id = 0;
    for (const DialogDef& d : script_.dialogs) {
        auto it = index.find(upperOf(d.name));
        Dlg dl;
        dl.def = &d;
        for (const DialogWindow& w : d.windows) {
            dl.widgets.emplace_back();
            build(dl.widgets.back(), w, id);
        }
        if (it != index.end()) dialogs_[it->second] = std::move(dl);
    }
    // 0x589930 boot open (mode 0) [O]: idle timer +0x15fc = 17.0, fade colour 0xa064c8 = (0, 0, 0, 1), MusicManager
    // +0x20 = 1 (music held), dialog +0x15e4 = 10, SetState(2) -> LegalText opens.
    fade_ = Rgba{0, 0, 0, 1};
    idle_ = 17.0f;
    current_ = first;
    if (first == PressStart) {  // [port] a return to the title opens PressStart over the Background, as after Profile
        openDialog(Background);
        dialogs_[Background].state = 4;
    }
    setMenuState(2);
}

void BootScreens::build(Widget& w, const DialogWindow& d, int& id) {
    w.def = &d;
    w.id = id++;
    const std::string type = upperOf(d.type);
    w.text = type == "STATICTEXT";
    w.tt = d.tt;
    // CWindow_G vf15 0x5a1260: a smooth animation, rate 100.0 after its 1.1 ctor value [O]; the bitmap colour
    // (MENUPropertyColor 0x598d90 -> 0x5982f0) is the shown block's rgba [O]; CStaticText_G vf15 0x5a0320: hidden scale 0.
    w.anim.rate = 100.0f;
    if (d.hasColour) {
        for (GuiAnimation::Block* b : {&w.anim.shown, &w.anim.live})
            (*b)[5] = d.colour.r, (*b)[6] = d.colour.g, (*b)[7] = d.colour.b, (*b)[8] = d.colour.a;
        w.anim.hidden = w.anim.shown;
    }
    if (w.text) w.anim.hidden[0] = 0.0f;
    for (const DialogWindow& c : d.children) {
        w.children.emplace_back();
        build(w.children.back(), c, id);
    }
}

BootScreens::Widget* BootScreens::find(int dialog, const std::string& name) {
    auto it = dialogs_.find(dialog);
    if (it == dialogs_.end()) return nullptr;
    std::function<Widget*(std::vector<Widget>&)> walk = [&](std::vector<Widget>& ws) -> Widget* {
        for (Widget& w : ws) {
            if (upperOf(w.def->name) == upperOf(name)) return &w;
            if (Widget* c = walk(w.children)) return c;
        }
        return nullptr;
    };
    return walk(it->second.widgets);
}

void BootScreens::showWidget(Widget& w) {  // 0x5a1560 (+0x4d = 1) + PlayShow 0x5a21c0 (the zoom of a StaticText)
    w.visible = true;
    w.anim.show();
    // CStaticText_G vf23 0x5a0420 (OnShow): restart the typewriter (+0xf4 = 0, +0xfc = 0, +0x100 = 0) when in that mode.
    if (w.typewriter) w.typeTime = 0, w.typeDone = false;
}

void BootScreens::hideWidget(Widget& w) {  // +0x4d = 0, vt+0x60 (OnHide), listeners [O]
    w.visible = false;
    if (w.typewriter) w.typeTime = 0, w.typeDone = false;
}

void BootScreens::setMenuState(int s) {  // 0x589c40 [O]: +0x15b4 = s, timer +0x15ac = 0
    mmState_ = s, mmTimer_ = 0;
    if (s == 2) openDialog(current_);  // focus the current dialog (+0x74[+0x15e4] -> state 1)
}

void BootScreens::openDialog(int index) {
    auto it = dialogs_.find(index);
    if (it == dialogs_.end()) return;
    Dlg& d = it->second;
    // Dialog_G SetState (0x447e60 pattern) [O]: +0x40 = 0, +0x3c = 1, +0x44 = 0, vf19 enter, the show animation.
    d.state = 1, d.stateTime = 0;
    enter(index, helpPad_);
    d.anim.show();
}

void BootScreens::go(int index, bool keepBack) {  // 0x58a470 [O]
    if (mmState_ != 3) setMenuState(3);
    leaving_ = current_;
    current_ = index;
    auto it = dialogs_.find(leaving_);
    if (it != dialogs_.end() && it->second.state != 2 && it->second.state != 0) {
        it->second.state = 2, it->second.stateTime = 0;  // vt+0x6c(2): closing (no animation for state 2; vf20 when it ends)
    }
    if (!keepBack) dialogs_[index].back = leaving_;  // +0x6c = the leaving dialog's index, +0x80 = 4
}

void BootScreens::enter(int index, bool padMode) {
    Dlg& d = dialogs_[index];
    d.time = 0;  // Dialog_G vf19 0x4476b0: +0x84 = 0
    switch (index) {
        case LegalText:  // 0x58f920 [O]: cursor hidden (+0x15a7), +0xe0 = 0, up to 4 streamed boot scripts (none pending in the port)
            legalTimer_ = 0;
            break;
        case CompileShader:  // 0x58f590 [O]: Background -> state 1, gauge reset, +0x6c = -1
            if (dialogs_[Background].state == 0) openDialog(Background);
            d.back = -1;
            break;
        case Profile:  // 0x592f10 [O]: +0xe4 = 0.1 s, Background -> state 1
            profileTimer_ = 0.1f;
            if (dialogs_[Background].state == 0) openDialog(Background);
            break;
        case PressStart: {  // 0x591b90 [O]
            musicOn_ = true;  // MusicManager +0x20 = 0
            fade_ = Rgba{0, 0, 0, 1};  // +0x15c0..+0x15cc
            if (Widget* w = find(PressStart, "PressStartText"); w && !w->visible) showWidget(*w);  // child 3
            if (Widget* w = find(PressStart, "PressX")) hideWidget(*w);                              // child 4
            if (Widget* w = find(PressStart, "AutoSaveWarning")) hideWidget(*w);                     // child 2
            if (padMode) pressText_ = 1218;  // [game+0x3214]+0x24: TT 0x4c2 (child 3 +0xe0, +0xe8 = 0, vt+0x7c)
            if (Widget* w = find(PressStart, "PressStartText")) w->tt = pressText_;
            pressBlocked_ = false;           // +0x100 = 0, +0x104 = 0
            idleOn_ = true;                  // MenuManager +0x1600 = 1 (0x5882f0 resets the pad slots)
            break;
        }
        case TRCMessage:  // 0x5947b0 [O]: PressX (0x9c) -> +0xe4, hidden; up to 3 streamed boot scripts (none)
            if (Widget* w = find(TRCMessage, "PressX")) hideWidget(*w);
            break;
        default:
            break;
    }
}

void BootScreens::leave(int index) {
    switch (index) {
        case LegalText:
            // 0x58f9d0 [O]: MusicManager vt+0x4c(0), 0x4e61a0(0), +0x190 bit 3 off, then `ChoosePlayMovie INTRO` (Intro.bik,
            // 22.5 s). [port] Bink is not ported: the opening movie is skipped.
            std::printf("boot: LegalText closed (Intro.bik skipped: Bink is not ported)\n");
            break;
        case TRCMessage:  // 0x594850 [O]: LoadingDraw +0x35 = 0, custom centre +0x280/+0x284 = -1
            saveIcon_ = false;
            break;
        default:
            break;
    }
}

void BootScreens::input(int index, const Input& in, bool padMode) {
    switch (index) {
        case PressStart:
            // 0x591ae0 [O]: (GFWL invite: none) unless +0x100: confirm on boot-mode 0x14's auto flag (MenuManager +0x13ec,
            // only after a sign-in change), Validate, or Start in pad mode -> 0x592760: storage idle -> menu sound
            // [0xa02800] + 0x5929e0: not the tutorial mode (game+0x32bc != 2) -> 0x58a470(0xb, 0) TRCMessage.
            if (!pressBlocked_ && (in.validate || (padMode && in.start))) {
                sound_ = 3;  // CONFIRM
                go(TRCMessage, false);
            }
            break;
        case TRCMessage:
            // 0x5949e0 [O]: Back (+0x1429) -> vt+0x74 (DialogMenu back to +0x6c = PressStart); after the timer, nothing
            // pending (0x55adc0) and Validate -> menu sound + 0x594a50 (PlayLevel USA1 + the career vehicle).
            if (in.back && dialogs_[TRCMessage].back >= 0) {
                go(dialogs_[TRCMessage].back, true);
                break;
            }
            if (trcTimer_ <= 0 && in.validate) {
                sound_ = 3;
                // 0x594a50 [O]: queue "PlayLevel USA1 <vehicle>", fade out 0.2 s (0x438190 [0x9dbfa8]), MissionManager
                // +0x30 = 0, close the dialogs (0x58a210).
                launchFade_ = 0;
                std::printf("boot: TRCMessage Continue -> PlayLevel USA1 (career world)\n");
            }
            break;
        default:
            break;  // LegalText / CompileShader / Profile: DialogMenu vf15 Back -> +0x6c = -1: nothing [O]
    }
}

void BootScreens::stepWidgets(std::vector<Widget>& ws, float dt) {
    for (Widget& w : ws) {
        if (!w.visible && !w.anim.playing()) continue;  // CWindow_G vf17 0x5a14c0 [O]
        stepWidgets(w.children, dt);
        w.anim.step(dt);
        if (w.text && w.typewriter && !w.typeDone) w.typeTime += dt;  // CStaticText_G vf17 0x5a03c0 [O]
    }
}

void BootScreens::dialogFrame(int index, float dt, const Input& in, bool padMode) {
    Dlg& d = dialogs_[index];
    if (d.state == 0) return;
    // Dialog_G vf23 0x4476d0 [O]: the animation (m_596500), then by state: 1 -> 4 once it is idle (vf25: PlayShow on the
    // visible children); 2 -> 0; 4: vf16(dt), the children, vf15 input; +0x40 / +0x84 += dt.
    d.anim.step(dt);
    if (d.state == 1 && !d.anim.playing()) {
        d.state = 4, d.stateTime = 0;
        for (Widget& w : d.widgets)
            if (w.visible) showWidget(w);  // 0x447f30 -> 0x5a21c0 (children of a shown window too)
        std::function<void(std::vector<Widget>&)> deep = [&](std::vector<Widget>& ws) {
            for (Widget& w : ws) {
                for (Widget& c : w.children)
                    if (c.visible) c.anim.show();
                deep(w.children);
            }
        };
        deep(d.widgets);
    } else if (d.state == 2) {
        d.state = 0;
        leave(index);  // vf20
    }
    if (d.state == 4) {
        if (index == LegalText) {  // 0x58f980 [O]: +0xe0 += dt; >= 6.0 and no streamed script running -> CompileShader
            legalTimer_ += dt;
            if (legalTimer_ >= 6.0f) go(CompileShader, false);
        } else if (index == CompileShader) {
            // 0x58f710: -> Profile when the shader gauge reaches 1.0. [port] no shader compile: the gauge is full at once.
            go(Profile, false);
        }
    }
    if (d.state != 0) stepWidgets(d.widgets, dt);
    if (d.state == 4) input(index, in, padMode);
    d.stateTime += dt, d.time += dt;
    // The classes' own vf23 tails.
    if (index == Profile && d.state != 0 && d.state != 3) {  // 0x593020 [O]
        profileTimer_ -= dt;
        if (profileTimer_ <= 0 && current_ == Profile) go(PressStart, false);
    } else if (index == PressStart && d.state != 0) {  // 0x591d60 [O]
        // hover ramp 0x5927b0: the mouse inside (0.6, 0.76) (0.8, 0.76) (0.8, 0.82) (0.61, 0.87) -> +0x114 up to 0.5
        // at 1/s, else down to 0 (no reader of +0x114 in FUEL.exe).
        const float px[4] = {0.6f, 0.8f, 0.8f, 0.61f}, py[4] = {0.76f, 0.76f, 0.82f, 0.87f};
        bool inside = false;
        if (in.mouseX >= 0 && in.mouseY >= 0)
            for (int i = 0, j = 3; i < 4; j = i++)
                if ((py[i] > in.mouseY) != (py[j] > in.mouseY) && in.mouseX < (px[j] - px[i]) * (in.mouseY - py[i]) / (py[j] - py[i]) + px[i])
                    inside = !inside;
        hover_ = inside ? std::min(0.5f, hover_ + dt) : std::max(0.0f, hover_ - dt);
        // 0x592ce0: PressStartText colour (0, 0, 0, (cos 5t + 1) / 2), t = +0x84.
        if (Widget* w = find(PressStart, "PressStartText")) w->fill = Rgba{0, 0, 0, pulseAlpha()}, w->fillSet = true;
        // 0x58a9e0 / 0x58a950: idle 17 s -> `ChoosePlayMovie VIDGMPL` (unless the tutorial mode). [port] Bink is not
        // ported: the timer runs and restarts, no attract movie plays.
        if (idleOn_) {
            idle_ = in.any ? 17.0f : idle_ - dt;
            if (idle_ < 0) idle_ = 17.0f, std::printf("boot: PressStart idle 17 s (VIDGMPL attract movie skipped: Bink)\n");
        }
        pressText_ = padMode ? 1218 : 230;  // TT 0x4c2 / 0xe6 by [game+0x3218]+0x24 -> child 3 +0xe0 (+0xe8 = 0, vt+0x7c)
        if (Widget* w = find(PressStart, "PressStartText")) w->tt = pressText_;
    } else if (index == TRCMessage && d.state != 0) {  // 0x5948d0 [O]
        trcTimer_ -= dt;
        saveIcon_ = true;  // LoadingDraw +0x44 = 1.0, +0x48 = 0 (white), +0x35 = 1, centre (0.5, 0.55)
        // (X X RB LB -> EnablePopupMenu: a debug menu that is dead in retail, not ported.)
        if (Widget* w = find(TRCMessage, "PressX"); w && !w->visible && trcTimer_ <= 0) showWidget(*w);
    }
}

void BootScreens::update(float dt, const Input& in, bool padMode) {
    helpPad_ = padMode;
    if (launch_) return;
    if (launchFade_ >= 0) {  // 0x594a50's fade; the dialogs are closed (0x58a210)
        launchFade_ += dt;
        if (launchFade_ >= 0.2f) launch_ = true;
        return;
    }
    // MenuManager_G per frame 0x589ea0 [O]: 3 -> 4 when the leaving dialog is closed (state 0 or 7); 4 -> 2 after 0.366 s.
    mmTimer_ += dt;
    if (mmState_ == 3) {
        auto it = dialogs_.find(leaving_);
        if (it == dialogs_.end() || it->second.state == 0 || it->second.state == 7) setMenuState(4);
    } else if (mmState_ == 4 && mmTimer_ > 0.366f) {
        setMenuState(2);
    }
    // FrontGlobal / MenuManager vf17: every dialog's vf23 in index order.
    for (auto& [index, d] : dialogs_) dialogFrame(index, dt, in, padMode);
}

int BootScreens::dialogState(Dialog d) const {
    auto it = dialogs_.find(d);
    return it == dialogs_.end() ? 0 : it->second.state;
}

float BootScreens::dialogTime(Dialog d) const {
    auto it = dialogs_.find(d);
    return it == dialogs_.end() ? 0 : it->second.time;
}

bool BootScreens::continueShown() const {
    auto* self = const_cast<BootScreens*>(this);
    const Widget* w = self->find(TRCMessage, "PressX");
    return w && w->visible;
}

float BootScreens::pulseAlpha() const { return (std::cos(dialogTime(PressStart) * 5.0f) + 1.0f) * 0.5f; }

void BootScreens::preview(Dialog d, float t, bool padMode) {
    helpPad_ = padMode;
    for (auto& [i, dl] : dialogs_) dl.state = 0;
    current_ = d, mmState_ = 2, launchFade_ = -1, launch_ = false;
    if (d == PressStart || d == TRCMessage) openDialog(Background), dialogs_[Background].state = 4;
    if (d == TRCMessage) openDialog(PressStart), dialogs_[PressStart].state = 0;  // passed through PressStart first
    openDialog(d);
    Input none;
    const float step = 1.0f / 60.0f;
    for (float s = 0; s < t; s += step) {
        for (auto& [i, dl] : dialogs_) dialogFrame(i, step, none, padMode);
        if (current_ != d) break;  // LegalText moved on after 6 s
    }
}

// ---- drawing --------------------------------------------------------------------------------------------------------

void BootScreens::drawWidgets(UiRenderer& r, const std::vector<Widget>& ws, float px, float py, float pw, float ph) const {
    // The children draw after their parent (CBitmap_G vf18 0x598830 -> 0x5a1410); among siblings the pictures first
    // [I: as PK25's MCAlert; TRCMessage lists its Background last and its texts must stay readable].
    for (int pass = 0; pass < 2; ++pass)
        for (const Widget& w : ws) {
            if (w.text != (pass == 1)) continue;
            if (!w.visible && !w.anim.playing()) continue;  // +0x4d or the animation running (0x598830 / 0x5a05b0)
            const DialogWindow& d = *w.def;
            const GuiAnimation::Block& b = w.anim.live;
            // The live block [I]: offsets (1, 2) in canvas fractions, scale (0) about the window centre.
            float x = px + d.x * pw + b[1] * kW, y = py + d.y * ph + b[2] * kH, ww = d.w * pw, hh = d.h * ph;
            const float cx = x + ww * 0.5f, cy = y + hh * 0.5f;
            if (w.text) {
                std::string s = w.tt ? textOf_(w.tt) : std::string();
                // typewriter: chars = min(length, ftol(+0xfc / +0xf8)) (0x5a05b0) [O]; never on for PC's texts
                if (w.typewriter && !w.typeDone) s = s.substr(0, std::min(s.size(), size_t(std::max(0.0f, w.typeTime / w.typeRate))));
                if (!s.empty() && b[0] > 1e-4f) {
                    DialogTextStyle ts;
                    if (auto it = script_.textStyles.find(d.textStyle); it != script_.textStyles.end()) ts = it->second;
                    TextStyle st;
                    st.font = ts.big ? 1 : 0;
                    st.height = ts.scale * d.textScale * b[0] * assets_.font(st.font).refHeight;
                    st.fill = w.fillSet ? w.fill : ts.fill;
                    st.outline = d.outlined ? ts.outline : Rgba{0, 0, 0, 0};
                    st.outline.a *= st.fill.a;  // [I] the outline fades with the text (0x592ce0's alpha)
                    // MENUPropertyTextAlign 0x58c630 [O]: LeftAligned 0, RightAligned 2, anything else (Centered,
                    // CenterAligned, Justified) 1. [I] lines from the top of the box; the zoom grows the box about its centre.
                    const int align = upperOf(d.align) == "LEFTALIGNED" ? 0 : upperOf(d.align) == "RIGHTALIGNED" ? 2 : 1;
                    const float zx = cx - ww * 0.5f * b[0], zy = cy - hh * 0.5f * b[0], zw = ww * b[0], zh = hh * b[0];
                    // [I] the lines centred in the box height (0x43d3c0 has a vertical centring path; the port measures
                    // the block with an invisible pass first).
                    const float keep = r.alpha;
                    r.alpha = 0;
                    const float used = r.richText(s, zx, zy, zx + zw, zy + zh, align, st, true, true);
                    r.alpha = keep;
                    const float oy = std::max(0.0f, (zh - used) * 0.5f);
                    if (d.shadowed) {  // +0xd1 [I: a black copy 2 canvas px down-right]
                        TextStyle sh = st;
                        sh.fill = Rgba{0, 0, 0, st.fill.a * 0.8f}, sh.outline = Rgba{0, 0, 0, 0};
                        r.richText(s, zx + 2, zy + oy + 2, zx + zw + 2, zy + zh + 2, align, sh, true, true);
                    }
                    r.richText(s, zx, zy + oy, zx + zw, zy + zh, align, st, true, true);
                }
            } else if (upperOf(d.type) == "BITMAP" && !d.image.empty()) {
                // The colour block x 2 (0.5 = the picture as is: MENUPropertyColor 0.5 0.5 0.5 1 on FOND_LEGAL) [O 0x5982f0
                // writes the colour into the animation's rgba, whose neutral default is 0.5].
                const Rgba c{std::min(1.0f, b[5] * 2), std::min(1.0f, b[6] * 2), std::min(1.0f, b[7] * 2), b[8]};
                if (const BitmapRef img = assets_.image(d.image))
                    r.image(img, cx - ww * 0.5f * b[0], cy - hh * 0.5f * b[0], ww * b[0], hh * b[0], c);
            }
            drawWidgets(r, w.children, x, y, ww, hh);
        }
}

void BootScreens::draw(UiRenderer& r, id<MTLTexture> saveIcon, float spin) const {
    // MenuManager_G vf19 0x588b30 [O]: (letterbox: UiRenderer), the fade quad +0x15c0 when its alpha > 0, the dialogs
    // whose state != 0 in index order, (the software cursor: the port keeps the system's).
    if (fade_.a > 0) r.rect(r.viewLeft(), r.viewTop(), r.viewWidth(), r.viewHeight(), fade_);
    for (const auto& [index, d] : dialogs_) {
        if (d.state == 0) continue;
        const DialogDef& def = *d.def;
        drawWidgets(r, d.widgets, def.x * kW + d.anim.live[1] * kW, def.y * kH + d.anim.live[2] * kH, def.w * kW, def.h * kH);
    }
    // LoadingDraw 0x5035c0 with TRCMessage's custom centre (0.5, 0.55) [O].
    if (saveIcon_) drawSaveIcon(r, saveIcon, spin, false, 0.5f, 0.55f);
    // 0x594a50's screen fade (0x438190: to black in 0.2 s) [I: linear].
    if (launchFade_ >= 0) r.rect(r.viewLeft(), r.viewTop(), r.viewWidth(), r.viewHeight(), Rgba{0, 0, 0, std::min(1.0f, launchFade_ / 0.2f)});
}

// ---- --menu-check ------------------------------------------------------------------------------------------------------

int runBootCheck(const std::string& dataDir, const std::string& language) {
    int bad = 0;
    auto expect = [&](bool ok, const std::string& what) {
        if (!ok) ++bad, std::printf("boot check: FAIL %s\n", what.c_str());
    };
    // 0x596ce0 maths: rate 1.1 at 60 fps moves 1 - 1/1.1^0.505 = 4.70 % a frame; rate 100: 90.2 %, snapped (< 1e-6) in frame 7.
    {
        GuiAnimation a;
        a.hidden[0] = 0, a.show();
        a.step(1.0f / 60.0f);
        expect(std::fabs(a.live[0] - 0.04703f) < 1e-4f, "rate 1.1 step " + std::to_string(a.live[0]));
        GuiAnimation z;
        z.rate = 100, z.hidden[0] = 0, z.hidden[8] = 0.25f, z.show();
        int frames = 0;
        while (z.playing() && frames < 100) z.step(1.0f / 60.0f), ++frames;
        expect(frames == 7 && z.live[0] == 1.0f, "rate 100 zoom done in " + std::to_string(frames) + " frames");
        GuiAnimation c;
        c.hidden[8] = 0, c.show();
        c.step(1.0f / 60.0f);
        expect(c.live[8] == 1.0f, "0x596e00 takes alpha as is");
    }
    const MenuAssets assets(dataDir);
    const TextTable text = loadText(dataDir, language);
    auto textOf = [&](uint32_t tt) {
        const std::string* s = text.get(tt);
        return s ? helpTexts().expand(*s) : std::string();
    };
    for (uint32_t tt : {16032u, 230u, 1218u, 231u, 16033u}) expect(text.get(tt) != nullptr, "TT " + std::to_string(tt) + " present");
    for (const char* img : {"FOND_LEGAL", "PRESS_START"}) expect(bool(assets.image(img)), std::string("picture ") + img);
    expect(text.get(1218) && *text.get(1218) == "Press {STR_START} to begin", "TT 1218 rewritten by TransText");

    const float dt = 1.0f / 60.0f;
    BootScreens b(dataDir, assets, textOf);
    BootScreens::Input none, valid, back, start;
    valid.validate = true, back.back = true, start.start = true;
    expect(b.current() == BootScreens::LegalText && b.dialogState(BootScreens::LegalText) == 1, "boot opens LegalText (0x589930)");
    float t = 0, pressAt = -1, legalOff = -1;
    bool musicEarly = false;
    for (int i = 0; i < 60 * 10 && pressAt < 0; ++i) {
        b.update(dt, (i % 30) == 0 ? valid : none, false);  // Validate does not skip the legal screen
        t += dt;
        if (legalOff < 0 && b.dialogState(BootScreens::LegalText) == 0) legalOff = t;
        if (b.musicOn() && b.dialogState(BootScreens::PressStart) == 0) musicEarly = true;
        if (b.dialogState(BootScreens::PressStart) == 4) pressAt = t;
    }
    expect(legalOff >= 6.0f && legalOff < 6.1f, "LegalText closes after 6.0 s (got " + std::to_string(legalOff) + ")");
    // legal 6.0 + 3 x (closing frame + 0.366 s + open frame) + Profile 0.1 s
    expect(pressAt > 7.1f && pressAt < 7.4f, "PressStart active at " + std::to_string(pressAt) + " s");
    expect(!musicEarly && b.musicOn(), "music held until PressStart (MusicManager +0x20)");
    expect(b.dialogState(BootScreens::Background) == 4, "Background stays under PressStart");
    expect(b.pressStartText() == 230, "TT 230 without a pad");
    b.update(dt, none, true);
    expect(b.pressStartText() == 1218, "TT 1218 in pad mode");
    expect(std::fabs(b.pulseAlpha() - (std::cos(b.dialogTime(BootScreens::PressStart) * 5.0f) + 1.0f) * 0.5f) < 1e-6f, "pulse (cos 5t + 1) / 2");
    b.update(dt, start, false);
    expect(b.current() == BootScreens::PressStart, "Start without a pad does nothing (0x591ae0)");
    b.update(dt, start, true);
    expect(b.current() == BootScreens::TRCMessage && b.takeSound() == 3, "Start in pad mode confirms (+ CONFIRM sound)");
    float trcOpen = -1;
    t = 0;
    for (int i = 0; i < 120 && trcOpen < 0; ++i) {
        b.update(dt, none, false), t += dt;
        if (b.dialogState(BootScreens::TRCMessage) == 4) trcOpen = t;
    }
    expect(trcOpen > 0.36f && trcOpen < 0.42f, "TRCMessage opens 0.366 s later (" + std::to_string(trcOpen) + ")");
    expect(b.saveIconOn() && !b.continueShown(), "TRC: save icon on, Continue hidden");
    for (int i = 0; i < 60; ++i) b.update(dt, valid, false);
    expect(!b.launch() && b.current() == BootScreens::TRCMessage, "Validate before 3 s does nothing");
    for (int i = 0; i < 150 && !b.continueShown(); ++i) b.update(dt, none, false);
    expect(b.continueShown() && b.trcTimer() <= 0 && b.trcTimer() > -0.05f, "Continue shows at 3.0 s (0x5948d0)");
    b.update(dt, back, false);
    for (int i = 0; i < 60; ++i) b.update(dt, none, false);
    expect(b.current() == BootScreens::PressStart && b.dialogState(BootScreens::PressStart) == 4 && !b.saveIconOn(), "Back -> PressStart (+0x6c), icon off");
    b.update(dt, valid, false);
    for (int i = 0; i < 30; ++i) b.update(dt, none, false);
    expect(b.continueShown(), "TRC timer is not reset by its enter (ctor 3.0 s only): Continue at once");
    b.update(dt, valid, false);
    expect(!b.launch(), "0x594a50's 0.2 s fade first");
    for (int i = 0; i < 13; ++i) b.update(dt, none, false);
    expect(b.launch(), "launch after the fade");
    // A return to the title opens PressStart directly [port].
    BootScreens t2(dataDir, assets, textOf, BootScreens::PressStart);
    t2.update(dt, none, false);
    expect(t2.current() == BootScreens::PressStart && t2.dialogState(BootScreens::PressStart) == 4, "title return: PressStart at once");
    std::printf("boot check: legal %.2f s, PressStart at %.2f s, TRC +%.3f s, %s\n", legalOff, pressAt, trcOpen, bad ? "FAILED" : "ok");
    return bad;
}

}  // namespace fuel::ui
