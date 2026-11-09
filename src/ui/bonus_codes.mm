// P78: Options > Bonus codes (FUEL.exe states 0x33 / 0x34 / 0x35; notes/p75-work/C.md sec. 5, notes/p78-ui-gaps.md):
// the list PageOptionsBonusCodes (0x494a40 / 0x494d40), the keyboard PageOptionsBonusCodesKeyboard (0x493320 /
// 0x493580 / 0x495ca0) and the answer PageOptionsBonusCodesResult (0x491fb0, keys 0x4934c0).
#include "ui/menu.h"

#include "core/career.hpp"
#include "core/resources.hpp"
#include "ui/ui_renderer.h"

#include <algorithm>
#include <cctype>

namespace fuel::ui {
namespace {

std::string upperAscii(std::string s) {
    for (char& c : s) c = char(std::toupper(static_cast<unsigned char>(c)));
    return s;
}
bool hasCode(const std::string& c) { return !c.empty() && upperAscii(c) != "NONE"; }

}  // namespace

// 0x494a40 [O]: slots 1, 2, .. in this order: every vehicle record with a code (+8 != crc "NONE"; picture = the vehicle's
// (+0x4c) when owned (+1)); every hub's CHEAT livery (+0x20 == 3) with a code (+0xc; VIGNETTE_GENERIC_LIVERIE when owned
// (+1)); every pilot livery with a code (+8; VIGNETTE_GENERIC_LIVERIE when owned (+0), name argument +0x18).
std::vector<Menu::BonusSlot> Menu::bonusSlots() const {
    std::vector<BonusSlot> out;
    if (!career_) return out;
    const CareerRules& R = career_->rules();
    const CareerState& S = career_->state();
    for (size_t v = 0; v < R.vehicles.size(); ++v)
        if (hasCode(R.vehicles[v].code)) {
            BonusSlot b;
            b.kind = 0, b.a = int(v), b.code = R.vehicles[v].code;
            b.owned = v < S.vehicles.size() && S.vehicles[v].owned;
            if (const VehicleInfo* vi = vehicleInfo(R.vehicles[v].label)) b.picture = vi->picture;
            out.push_back(b);
        }
    for (size_t h = 0; h < R.hubs.size(); ++h)
        for (size_t k = 0; k < R.hubs[h].liveries.size(); ++k) {
            const auto& l = R.hubs[h].liveries[k];
            if (upperAscii(l.kind) != "CHEAT" || !hasCode(l.arg)) continue;
            BonusSlot b;
            b.kind = 1, b.a = int(h), b.b = int(k), b.code = l.arg;
            b.owned = h < S.liveries.size() && k < S.liveries[h].size() && S.liveries[h][k].b;
            b.picture = "VIGNETTE_GENERIC_LIVERIE";
            out.push_back(b);
        }
    for (size_t p = 0; p < R.pilotLiveries.size(); ++p)
        if (hasCode(R.pilotLiveries[p].code)) {
            BonusSlot b;
            b.kind = 2, b.a = int(p), b.code = R.pilotLiveries[p].code;
            b.owned = p < S.pilotOwned.size() && S.pilotOwned[p];
            b.picture = "VIGNETTE_GENERIC_LIVERIE";
            b.arg = int(R.pilotLiveries[p].text);
            out.push_back(b);
        }
    return out;
}

// 0x493730 [O]: the vehicles (0x56a380 -> 0x569f60: unlocked and owned, its ALWAYS (0x569b20(0)) and CHEAT (0x569b20(3))
// liveries owned), then the hubs' CHEAT liveries (0x56a3f0: owned = revealed = 1), then the pilot liveries (0x56b8d0:
// owned); codes compare by AsoboCrc. Returns the slot (0x491e20: its place in 0x494a40's order) or -1.
int Menu::bonusRedeem(const std::string& entered) {
    if (!career_ || entered.empty()) return -1;
    const uint32_t crc = asoboCrc(entered);
    const std::vector<BonusSlot> slots = bonusSlots();
    CareerRules const& R = career_->rules();
    CareerState& S = career_->state();
    for (size_t i = 0; i < slots.size(); ++i) {
        const BonusSlot& b = slots[i];
        if (asoboCrc(b.code) != crc) continue;
        if (b.kind == 0) {
            auto& v = S.vehicles[size_t(b.a)];
            v.unlocked = 1, v.owned = 1;
            const std::string mesh = upperAscii(R.vehicles[size_t(b.a)].mesh);
            for (size_t h = 0; h < R.hubs.size() && h < S.liveries.size(); ++h)
                for (size_t k = 0; k < R.hubs[h].liveries.size() && k < S.liveries[h].size(); ++k) {
                    const auto& l = R.hubs[h].liveries[k];
                    const std::string kind = upperAscii(l.kind);
                    if ((kind == "ALWAYS" || kind == "CHEAT") && upperAscii(l.mesh) == mesh) S.liveries[h][k].a = S.liveries[h][k].b = 1;
                }
        } else if (b.kind == 1) {
            S.liveries[size_t(b.a)][size_t(b.b)].a = S.liveries[size_t(b.a)][size_t(b.b)].b = 1;
        } else {
            S.pilotOwned[size_t(b.a)] = 1;
        }
        return int(i);
    }
    return -1;
}

void Menu::bonusOpen(Screen s) {
    screen_ = s;
    if (s == Screen::BonusKeyboard) bonus_.entered.clear(), bonus_.focus = 0;  // 0x495ca0: empty text, focus KEY_0
    setTitle("TT_TOP_TITLE_OPTION_BONUS_CODES");
}

void Menu::bonusKey(MenuKey k) {
    if (screen_ == Screen::BonusCodes) {  // 0x4931f0: Back -> the options (state 6); Accept / BOX_VALIDATION -> 0x34
        if (k == MenuKey::Back) goTab(TabOptions);
        else if (k == MenuKey::Accept) bonusOpen(Screen::BonusKeyboard);
        return;
    }
    if (screen_ == Screen::BonusKeyboard) {
        // 0x493320: Back -> 0x33; Left / Right move the focus over KEY_0 .. KEY_9, KEY_VALID [port: wrapping];
        // Accept 0x493580: a digit is added while the text is shorter than 8, KEY_VALID checks the code -> 0x35.
        if (k == MenuKey::Back) return bonusOpen(Screen::BonusCodes);
        if (k == MenuKey::Left || k == MenuKey::Up) bonus_.focus = (bonus_.focus + 10) % 11;
        if (k == MenuKey::Right || k == MenuKey::Down) bonus_.focus = (bonus_.focus + 1) % 11;
        if (k != MenuKey::Accept) return;
        if (bonus_.focus < 10) {
            if (bonus_.entered.size() < 8) bonus_.entered += char('0' + bonus_.focus);
            return;
        }
        bonus_.slot = bonusRedeem(bonus_.entered);
        bonus_.ok = bonus_.slot >= 0;
        if (bonus_.ok) markDirty();  // PK17: +0x80 = 1 (saved when the pause menu closes, 0x46e9b0)
        screen_ = Screen::BonusResult;
        return;
    }
    // 0x4934c0: success -> the OK popup (0x4ce400), failure -> yes / no (0x4cdfa0) [I: Yes tries again, No -> the list].
    if (bonus_.ok) {
        if (k == MenuKey::Accept || k == MenuKey::Back) bonusOpen(Screen::BonusCodes);
    } else {
        if (k == MenuKey::Accept) bonusOpen(Screen::BonusKeyboard);
        if (k == MenuKey::Back) bonusOpen(Screen::BonusCodes);
    }
}

void Menu::drawBonus(UiRenderer& r) {
    // 0x494d40(slot k, picture, arg) [O]: u = picture present; BONUS_CODE_BARRE k enabled = u; STATE k entry u (Locked /
    // Unlocked); PICT k = the picture (enabled) or the default material (disabled) [port: nothing]; NAME k entry u, and
    // with u and arg >= 0 the entry's text takes arg (0x613860) [port: the TT id arg].
    if (Page* p = scripts_.page("PageOptionsBonusCodes")) {
        const std::vector<BonusSlot> slots = bonusSlots();
        for (int k = 1; k <= 6; ++k) {
            const BonusSlot* b = size_t(k) <= slots.size() ? &slots[size_t(k - 1)] : nullptr;
            const bool u = b && b->owned;
            const std::string n = std::to_string(k);
            if (Item* it = p->find("ITEM_ID_BONUS_CODE_BARRE" + n)) it->enabled = u;
            if (Item* it = p->find("ITEM_ID_BONUS_CODE_STATE" + n)) it->entryIndex = u ? 1 : 0, it->visible = b != nullptr;
            if (Item* it = p->find("ITEM_ID_BONUS_CODE_PICT" + n)) {
                it->enabled = u;
                it->imgNormal = it->imgEnabled = it->imgFocus = u ? b->picture : std::string();
            }
            if (Item* it = p->find("ITEM_ID_BONUS_CODE_NAME" + n)) {
                it->entryIndex = u ? 1 : 0;
                it->hasDynText = false;
                if (u && b->arg > 0 && size_t(1) < it->entries.size() && it->entries[1].tt == 0) it->hasDynText = true, it->dynText = textOf(uint32_t(b->arg));
            }
        }
        drawPage(r, *p);
    }
    if (screen_ == Screen::BonusCodes) return;
    if (Page* p = scripts_.page("PageOptionsBonusCodesKeyboard")) {
        for (int k = 0; k <= 10; ++k) {
            Item* it = p->find(k < 10 ? "ITEM_ID_KEY_" + std::to_string(k) : std::string("ITEM_ID_KEY_VALID"));
            if (!it) continue;
            it->enabled = true, it->focus = k == bonus_.focus && screen_ == Screen::BonusKeyboard;
            if (k < 10) it->hasDynText = true, it->dynText = std::to_string(k);  // TT 1304 "%d" (0x495ca0)
        }
        if (Item* it = p->find("ITEM_ID_BONUS_CODE_ENTERED")) it->hasDynText = true, it->dynText = bonus_.entered;
        drawPage(r, *p);
    }
    if (screen_ != Screen::BonusResult) return;
    if (Page* p = scripts_.page("PageOptionsBonusCodesResult")) {
        // 0x491fb0: success -> "TT_BONUS_CODE%d" (slot + 1) and ITEM_ID_SUCESS; failure -> TT_BONUS_CODE_FAILED, ITEM_ID_FAILED.
        if (Item* it = p->find("ITEM_ID_BONUS_CODE_RESULT"))
            it->entryIndex = bonus_.ok ? std::clamp(bonus_.slot, 0, 5) : int(it->entries.size()) - 1;
        if (Item* it = p->find("ITEM_ID_SUCESS")) it->visible = bonus_.ok;
        if (Item* it = p->find("ITEM_ID_FAILED")) it->visible = !bonus_.ok;
        drawPage(r, *p);
    }
}

}  // namespace fuel::ui
