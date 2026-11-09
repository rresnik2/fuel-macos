// P66: the Profile tab (FUEL's profile page, state word +0x3e4), decoded from FUEL.exe; see
// ~/re-work/fuel/notes/p66-garage-profile.md. Tags: [O] observed, [I] inferred, [port] port choice.
//
// States (titles 0x4977c0, keys 0x4965c0, draws 0x4974a0): 3 the Profile left menu (0x496670: GENERAL -> 0x16 via
// 0x4963a0, COLLECTION -> 0x1e via 0x496410, COMPLETION -> 0x24 via 0x496450); Back from the pages -> 3.
// - General (0x496760): ITEM_ID_SELECTION_TYPE (GENERAL, PROGRESSION, VEHICLE, RECORDS, DISTANCE STATISTICS, TRIVIA,
//   ONLINE) moved by Left / Right (+0x128 -+ 1, 0x613b50); every sub page is filled when the page opens (0x497920):
//   0x49a130 general, 0x49a200 progression, 0x49a3d0 vehicles, 0x49a4e0 records, 0x49a690 distances, 0x49ac20 trivia,
//   0x49ac80 online. Values go through the label's own text (0x46c900 one int, 0x46ca50 two ints, 0x46ceb0 a string).
// - Collection (0x4968f0 / 0x497950): VEHICLE (0x497a50), VISTAPOINT (0x497ec0), PILOT (0x498280); Completion
//   (0x496d00 / 0x4979e0): CAREER (0x498700), CHALLENGES (0x498c50). Lines are copies of the line pages laid from
//   ITEM_ID_RECT_DYN_LINE_START downwards one rectangle height apart (0x499190.., 0x499b50..); six show (0x497320,
//   GROUP_GLOBAL<n> visible for scroll <= n - 1 < scroll + 6); Up / Down scroll by one line within count - 6.
#include "ui/menu.h"

#include "ui/ui_renderer.h"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>

namespace fuel::ui {
namespace {

std::string upper(std::string s) {
    for (char& c : s) c = char(std::toupper(static_cast<unsigned char>(c)));
    return s;
}

constexpr uint32_t kUnknownText = 0x1767;  // "??????" (lines not discovered yet)

// The labels' printf templates ("%02d/%02d", "%d h %d min", "%d.%03d km") with ints, and %s with a string.
std::string format(const std::string& t, std::initializer_list<long> v, const std::string& s = {}) {
    std::string out;
    auto it = v.begin();
    for (size_t i = 0; i < t.size(); ++i) {
        if (t[i] != '%' || i + 1 >= t.size()) {
            out += t[i];
            continue;
        }
        size_t j = i + 1;
        while (j < t.size() && (std::isdigit(static_cast<unsigned char>(t[j])) || t[j] == '.')) ++j;
        if (j >= t.size()) break;
        const std::string spec = t.substr(i, j - i + 1);
        char buf[64] = "";
        if (t[j] == 'd' || t[j] == 'i' || t[j] == 'u') {
            std::snprintf(buf, sizeof buf, (spec.substr(0, spec.size() - 1) + "ld").c_str(), it != v.end() ? *it++ : 0L);
            out += buf;
        } else if (t[j] == 's') {
            out += s;
        } else if (t[j] == '%') {
            out += '%';
        }
        i = j;
    }
    return out;
}

const char* const kGeneralPages[7] = {"PageProfileGeneral_General", "PageProfileGeneral_Progression", "PageProfileGeneral_Vehicles",
                                      "PageProfileGeneral_Records", "PageProfileGeneral_DStatistics", "PageProfileGeneral_Trivia",
                                      "PageProfileGeneral_Online"};

}  // namespace

void Menu::profileEnter(int state) {
    profile_.state = state;
    profile_.sel = 0, profile_.scroll = 0;
    if (state == 3) {
        screen_ = Screen::Tab, tab_ = TabProfile;
        setTitle("TT_TOP_TITLE_TOP_PROFILE");
        return;
    }
    screen_ = Screen::Profile;
    setTitle(state == 0x16 ? "TT_TOP_TITLE_PROFILE_GENERAL" : state == 0x1e ? "TT_TOP_TITLE_PROFILE_COLLECTION" : "TT_TOP_TITLE_PROFILE_COMPLETION");
}

bool Menu::profileAccept(const std::string& id) {
    if (id == "ITEM_ID_TOPMENU_PROFILE_GENERAL") return profileEnter(0x16), true;
    if (id == "ITEM_ID_TOPMENU_PROFILE_COLLECTION") return profileEnter(0x1e), true;
    if (id == "ITEM_ID_TOPMENU_PROFILE_COMPLETION") return profileEnter(0x24), true;
    return false;
}

void Menu::profileKey(MenuKey k) {
    auto& p = profile_;
    if (k == MenuKey::Back) return profileEnter(3);
    const int types = p.state == 0x16 ? 7 : p.state == 0x1e ? 3 : 2;
    if (k == MenuKey::Left || k == MenuKey::Right) {
        // List box previous / next (0x613bd0 / 0x613bf0) [I: they stop at the ends]; the line list starts at its top.
        const int n = std::clamp(p.sel + (k == MenuKey::Left ? -1 : 1), 0, types - 1);
        if (n != p.sel) p.sel = n, p.scroll = 0;
        return;
    }
    if (p.state == 0x16) return;
    const int lines = int(profileLines().size());
    if (k == MenuKey::Up && p.scroll > 0) --p.scroll;                    // 0x496d00: +0x3ec - 1 while > 0
    if (k == MenuKey::Down && p.scroll < lines - 6) ++p.scroll;          // + 1 while < count - 6
}

// The values of the General sub pages.
void Menu::profileFillGeneral() {
    if (!career_) return;
    const Career& c = *career_;
    const CareerState& s = c.state();
    const CareerRules& R = c.rules();
    auto set = [&](const char* page, const char* id, const std::string& v) {
        if (Page* pg = scripts_.page(page))
            if (Item* it = pg->find(id)) it->hasDynText = true, it->dynText = v;
    };
    auto tmpl = [&](const char* page, const char* id) {
        Page* pg = scripts_.page(page);
        const Item* it = pg ? pg->find(id) : nullptr;
        return it ? textOf(it->tt) : std::string("%d");
    };
    auto ints = [&](const char* page, const char* id, std::initializer_list<long> v) { set(page, id, format(tmpl(page, id), v)); };
    // 0x49a130: fuel (game +0x408); time played = the process clock split by 0x484690 into h, min [port: the menu's].
    const char* G = kGeneralPages[0];
    ints(G, "ITEM_ID_FUEL", {long(s.fuel)});
    ints(G, "ITEM_ID_TIMEPLAYED", {long(time_ / 3600.0f), long(std::fmod(time_ / 60.0f, 60.0f))});
    // 0x49a200 (each "%02d/%02d"): hubs open (0x577e90: 0x579720 test) / hubs; career races played (+0x10c, 0x577f10) /
    // career races (0x577ed0); with gold (+0x120 == 0, 0x577fa0); stars / 3 x career races; challenges won (+0x120 != 3,
    // 0x5780f0) / challenges; vistas visited (+1, 0x5781c0) / vistas; pilot liveries owned / all; special trucks caught
    // (kinds 0..2) / hub +0x24c sums (0x578220); wandering vehicles (kind 3) caught / +0x250 sums (0x5782d0) [O].
    const char* P = kGeneralPages[1];
    int hubsOpen = 0, careerN = 0, played = 0, gold = 0, chN = 0, chWon = 0, vistaN = 0, visited = 0, radarN = 0, radar = 0, wanderN = 0, wander = 0;
    for (size_t h = 0; h < R.hubs.size(); ++h) {
        hubsOpen += c.hubUnlockTest(int(h));
        for (const MissionStatus& m : s.career[h]) ++careerN, played += m.flag10c != 0, gold += m.bestRank == 0;
        for (const MissionStatus& m : s.challenges[h]) ++chN, chWon += m.bestRank != 3;
        for (const auto& v : s.vistas[h]) ++vistaN, visited += v.b != 0;
        for (size_t k = 0; k < R.hubs[h].traffic.size(); ++k) {
            const bool caught = c.trafficCaught(int(h), int(k));
            if (R.hubs[h].traffic[k].kind() < 3) ++radarN, radar += caught;
            else ++wanderN, wander += caught;
        }
    }
    int pilotOwned = 0;
    for (uint8_t o : s.pilotOwned) pilotOwned += o != 0;
    ints(P, "ITEM_ID_EVENTS_UNLOCKED", {hubsOpen, long(R.hubs.size())});
    ints(P, "ITEM_ID_CAREER_RACE_PLAYED", {played, careerN});
    ints(P, "ITEM_ID_CRACE_WITH_GOLD", {gold, careerN});
    ints(P, "ITEM_ID_CAREER_STARS", {c.stars(), c.starsMax()});
    ints(P, "ITEM_ID_CHALLENGE_CLEARED", {chWon, chN});
    ints(P, "ITEM_ID_VPOINT_VISITED", {visited, vistaN});
    ints(P, "ITEM_ID_PILOT_GEAR_OWNED", {pilotOwned, long(s.pilotOwned.size())});
    ints(P, "ITEM_ID_RADAR_FOUND", {radar, radarN});
    ints(P, "ITEM_ID_WANDERING_FOUND", {wander, wanderN});
    // 0x49a3d0: vehicles owned (0x56a180: owned, no code, a class) / such vehicles (0x56a100); vehicle liveries owned
    // (0x569cd0: owned, kind != CHEAT) / liveries (0x569d40, kind != CHEAT); favourite = the record with the largest
    // +0x10 (0x56a700; the seconds driven, game_menu.mm) [O]; empty while every record is 0.
    const char* V = kGeneralPages[2];
    int vehN = 0, vehOwned = 0, livN = 0, livOwned = 0;
    float best = 0;
    int fav = -1;
    for (size_t v = 0; v < R.vehicles.size(); ++v) {
        const auto& rv = R.vehicles[v];
        if (rv.code.empty() && rv.cls != -1) ++vehN, vehOwned += s.vehicles[v].owned != 0;
        if (s.vehicles[v].f10 > best) best = s.vehicles[v].f10, fav = int(v);
    }
    for (size_t h = 0; h < R.hubs.size(); ++h)
        for (size_t k = 0; k < R.hubs[h].liveries.size(); ++k)
            if (R.hubs[h].liveries[k].kind != "CHEAT") ++livN, livOwned += s.liveries[h][k].b != 0;
    ints(V, "ITEM_ID_VEHICLE_OWNED", {vehOwned, vehN});
    ints(V, "ITEM_ID_LIVERIES_OWNED", {livOwned, livN});
    set(V, "ITEM_ID_FAVORITE_VEHICLE", fav >= 0 ? vehicleName(R.vehicles[size_t(fav)].label) : std::string());
    // 0x49a4e0: top speed trunc(+0x345c m/s x 3.6) "%d km/h" (TT 1308), miles (+0x330c): trunc(x 0.621371) "%d %s" with
    // TT 1217 "mph"; longest jump trunc(+0x3428) TT 1306 "%d m"; time in the air trunc(+0x3458) with the label's TT [O].
    const char* Rp = kGeneralPages[3];
    const long kmh = long(s.get<float>(0xa4) * 3.6f);
    set(Rp, "ITEM_ID_PIK_SPEED", options_.miles ? format("%d %s", {long(float(kmh) * 0.6213712f)}, textOf(1217)) : format(textOf(1308), {kmh}));
    set(Rp, "ITEM_ID_LONGEST_JUMP", format(textOf(1306), {long(s.get<float>(0x70))}));
    ints(Rp, "ITEM_ID_TIME_SPENT_AIR", {long(s.get<float>(0xa0))});
    // 0x49a690: metres = u32 km x 1000 + f32 m; km: a = trunc(m x 0.001), b = trunc(m - 1000 a) with TT 1315
    // "%d.%03d km"; miles: x = m x 0.000621371, a = trunc(x), b = trunc((x - a) x 1000), TT 1319. Total = on road +
    // off road; free ride +0x3408; race / challenge +0x3410; on road +0x3418; off road +0x3420 [O].
    const char* D = kGeneralPages[4];
    auto dist = [&](double m) {
        if (options_.miles) {
            const double x = m * 0.000621371;
            const long a = long(x);
            return format(textOf(1319), {a, long((x - double(a)) * 1000.0)});
        }
        const long a = long(m * 0.001);
        return format(textOf(1315), {a, long(m - double(a) * 1000.0)});
    };
    auto pair = [&](size_t off) { return double(s.get<uint32_t>(off)) * 1000.0 + double(s.get<float>(off + 4)); };
    set(D, "ITEM_ID_TDIST_COVERED", dist(pair(0x60) + pair(0x68)));
    set(D, "ITEM_ID_DIST_COV_FREERIDE", dist(pair(0x50)));
    set(D, "ITEM_ID_DIST_COV_INRACE_CHALLENGE", dist(pair(0x58)));
    set(D, "ITEM_ID_DIST_COVERED_ONROAD", dist(pair(0x60)));
    set(D, "ITEM_ID_DIST_COVERED_OFF", dist(pair(0x68)));
    // 0x49ac20: heliports used (+0x3460), fuel drums found (+0x40c); 0x49ac80: online races played / won (+0x3444 /
    // +0x3448, not in the port: 0).
    ints(kGeneralPages[5], "ITEM_ID_NUMBER_OF_HELIPORTS", {long(s.get<int32_t>(0xa8))});
    ints(kGeneralPages[5], "ITEM_ID_FUEL_DRUMS_FOUND", {long(s.fuelFound)});
    ints(kGeneralPages[6], "ITEM_ID_RACE_ONLINE_PLAYED", {long(s.get<int32_t>(0x8c))});
    ints(kGeneralPages[6], "ITEM_ID_RACE_ONLINE_WON", {long(s.get<int32_t>(0x90))});
}

// The lines of the Collection / Completion page for the current type.
std::vector<Menu::ProfileLine> Menu::profileLines() const {
    std::vector<ProfileLine> out;
    if (!career_) return out;
    const Career& c = *career_;
    const CareerState& s = c.state();
    const CareerRules& R = c.rules();
    auto raceText = [&](int mission) -> uint32_t {
        const std::string& name = R.missions[size_t(mission)].name;
        for (const Event& e : events_)
            if (upper(e.id) == upper(name)) return e.nameText;
        return 0;
    };
    if (profile_.state == 0x24) {
        // 0x498700 / 0x498c50: per hub (0x579720 test -> enabled) its career (+0x214) or challenge (+0x21c) races.
        const bool career = profile_.sel == 0;
        for (size_t h = 0; h < R.hubs.size(); ++h) {
            const bool open = c.hubUnlockTest(int(h));
            ProfileLine hl;
            hl.hub = true, hl.text = R.hubs[h].nameText, hl.enabled = open;
            out.push_back(hl);
            const auto& list = career ? R.hubs[h].career : R.hubs[h].challenges;
            const auto& st = career ? s.career[h] : s.challenges[h];
            for (size_t k = 0; k < list.size(); ++k) {
                ProfileLine l;
                l.rank = k < st.size() ? st[k].bestRank : 3;
                l.text = open ? raceText(list[k]) : kUnknownText;
                l.enabled = l.rank != 3;
                l.career = career;
                out.push_back(l);
            }
        }
        return out;
    }
    if (profile_.sel == 0) {
        // 0x497a50 over the vehicles of 0x47a7d0(page, 0, 0, 0) (unlocked, the order list TT 3401..3475), those without a code:
        // a subtitle (name, enabled = owned), then per livery (0x569c00 / 0x569db0, CHEAT ones skipped): left the
        // livery's text, right its hub's name; not owned: revealed -> both, disabled; else "??????" [O].
        for (const VehicleInfo* v : vehicleOrder(true)) {
            const int vi = R.vehicleIndex(v->name);
            if (vi < 0 || !R.vehicles[size_t(vi)].code.empty()) continue;
            ProfileLine sl;
            sl.hub = true, sl.text = v->nameText, sl.enabled = s.vehicles[size_t(vi)].owned != 0;
            out.push_back(sl);
            const std::string mesh = upper(R.vehicles[size_t(vi)].mesh);
            int count = 0;
            for (const auto& hh : R.hubs)
                for (const auto& l : hh.liveries) count += upper(l.mesh) == mesh;
            for (int i = 1; i <= count; ++i)
                for (size_t h = 0; h < R.hubs.size(); ++h) {
                    bool found = false;
                    for (size_t k = 0; k < R.hubs[h].liveries.size() && !found; ++k) {
                        const auto& l = R.hubs[h].liveries[k];
                        if (upper(l.mesh) != mesh || l.index != i) continue;
                        found = true;
                        if (l.kind == "CHEAT") break;
                        const auto f = s.liveries[h][k];
                        ProfileLine ll;
                        ll.text2 = f.a || f.b ? l.text : kUnknownText;
                        ll.text = f.a || f.b ? R.hubs[h].nameText : kUnknownText;
                        ll.enabled = f.b != 0;
                        out.push_back(ll);
                    }
                    if (found) break;
                }
        }
    } else if (profile_.sel == 1) {
        // 0x497ec0: per hub (enabled = 0x579720) its vistas: text if revealed or visited, else "??????"; enabled =
        // visited; the right label hidden [O].
        for (size_t h = 0; h < R.hubs.size(); ++h) {
            ProfileLine hl;
            hl.hub = true, hl.text = R.hubs[h].nameText, hl.enabled = c.hubUnlockTest(int(h));
            out.push_back(hl);
            for (size_t k = 0; k < s.vistas[h].size(); ++k) {
                const auto f = s.vistas[h][k];
                ProfileLine l;
                const uint32_t t = k < R.hubs[h].vistaPoints.size() ? R.hubs[h].vistaPoints[k].text : 0;
                l.text2 = f.a || f.b ? t : kUnknownText;
                l.text = 0, l.enabled = f.b != 0;
                out.push_back(l);
            }
        }
    } else {
        // 0x498280: hubs with a career race that gives a pilot livery (0x579620 [I]); per such race (pilot livery entry
        // +4 == crc(race name)): left the livery's text, right the race's name when owned, else "??????" both [O].
        for (size_t h = 0; h < R.hubs.size(); ++h) {
            std::vector<ProfileLine> lines;
            for (int mi : R.hubs[h].career) {
                const std::string& race = R.missions[size_t(mi)].name;
                for (size_t p = 0; p < R.pilotLiveries.size(); ++p)
                    if (upper(R.pilotLiveries[p].unlock) == upper(race)) {
                        const bool owned = p < s.pilotOwned.size() && s.pilotOwned[p];
                        ProfileLine l;
                        l.text2 = owned ? R.pilotLiveries[p].text : kUnknownText;
                        l.text = owned ? raceText(mi) : kUnknownText;
                        l.enabled = owned;
                        lines.push_back(l);
                        break;
                    }
            }
            if (lines.empty()) continue;
            ProfileLine hl;
            hl.hub = true, hl.text = R.hubs[h].nameText, hl.enabled = c.hubUnlockTest(int(h));
            out.push_back(hl);
            out.insert(out.end(), lines.begin(), lines.end());
        }
    }
    return out;
}

void Menu::drawProfile(UiRenderer& r) {
    auto& p = profile_;
    const char* main = p.state == 0x16 ? "PageProfileGeneral" : p.state == 0x1e ? "PageProfileCollection" : "PageProfileCompletion";
    Page* pg = scripts_.page(main);
    if (!pg) return;
    if (Item* sel = pg->find("ITEM_ID_SELECTION_TYPE")) sel->entryIndex = p.sel, sel->enabled = false;  // authored: black, white outline
    if (Item* rect = pg->find("ITEM_ID_RECT_DYN_LINE_START")) rect->visible = false;  // a layout reference
    // 0x4973b0 / 0x497430: the type's vignette (collection: VIGNETTE_GENERIC_HELIPORT / _VISTA / _LIVERIE; completion:
    // VIGNETTE_GENERIC_CAREER / _CHALLENGE; image table +0xdd8 / +0xdcc / +0xdc8, +0xdd0 / +0xdd4) [O].
    if (Item* pic = pg->find("ITEM_ID_PICT_SELECTION_TYPE")) {
        static const char* const kCollection[3] = {"VIGNETTE_GENERIC_HELIPORT", "VIGNETTE_GENERIC_VISTA", "VIGNETTE_GENERIC_LIVERIE"};
        static const char* const kCompletion[2] = {"VIGNETTE_GENERIC_CAREER", "VIGNETTE_GENERIC_CHALLENGE"};
        const char* img = p.state == 0x1e ? kCollection[std::clamp(p.sel, 0, 2)] : kCompletion[std::clamp(p.sel, 0, 1)];
        pic->imgNormal = pic->imgEnabled = pic->imgFocus = img;
    }
    drawPage(r, *pg);
    if (p.state == 0x16) {
        profileFillGeneral();
        if (Page* sub = scripts_.page(kGeneralPages[std::clamp(p.sel, 0, 6)])) {
            for (Item& it : sub->items)  // the captions are LabelTT items with their own text
                if (it.kind == ItemKind::LabelDyn && !it.hasDynText) it.hasDynText = true, it.dynText.clear();
            drawPage(r, *sub);
        }
        return;
    }
    const Item* rect = pg->find("ITEM_ID_RECT_DYN_LINE_START");
    const float y0 = rect ? rect->y : 256, step = rect ? rect->h : 36;
    const auto lines = profileLines();
    const bool completion = p.state == 0x24;
    Page* hubPage = scripts_.page(completion ? "PageProfileCompletionHubNameLine" : "PageProfileCollectionSubtitleInfo");
    Page* linePage = scripts_.page(completion ? "PageProfileCompletionRaceNameLine" : "PageProfileCollectionLineInfo");
    for (int i = p.scroll; i < int(lines.size()) && i < p.scroll + 6; ++i) {
        const ProfileLine& l = lines[size_t(i)];
        const float y = y0 + step * float(i - p.scroll);
        Page* lp = l.hub ? hubPage : linePage;
        if (!lp) continue;
        for (Item& it : lp->items) {
            it.enabled = l.enabled;
            if (it.id == "ITEM_ID_TT_HUB_NAME" || it.id == "ITEM_ID_INFO_NAME" || it.id == "ITEM_ID_RACE_NAME")
                it.hasDynText = true, it.dynText = l.text ? textOf(l.text) : std::string();
            else if (it.id == "ITEM_ID_INFO2_NAME") it.hasDynText = true, it.dynText = l.text2 ? textOf(l.text2) : std::string();
            else if (it.id == "ITEM_ID_MEDAL_ICON") {
                // MAP_CAREER_* / MAP_CHALLENGE_* by the best rank (+0xce8 / +0xd04 + 4 rank), alpha 0 at rank 3 [O].
                static const char* const kC[4] = {"MAP_CAREER_GOLD_ICON", "MAP_CAREER_SILVER_ICON", "MAP_CAREER_BRONZE_ICON", "MAP_CAREER_NONE_ICON"};
                static const char* const kH[4] = {"MAP_CHALLENGE_GOLD_ICON", "MAP_CHALLENGE_SILVER_ICON", "MAP_CHALLENGE_BRONZE_ICON",
                                                  "MAP_CHALLENGE_NONE_ICON"};
                const int rk = std::clamp(l.rank, 0, 3);
                it.imgNormal = it.imgEnabled = it.imgFocus = l.career ? kC[rk] : kH[rk];
                it.visible = l.rank != 3;
            } else if (it.id == "ITEM_ID_PICT_RACE_TYPE") {
                it.imgNormal = l.career ? "VIGNETTE_GENERIC_CAREER_DISABLE" : "VIGNETTE_GENERIC_CHALLENGE_DISABLE";
                it.imgEnabled = it.imgFocus = l.career ? "VIGNETTE_GENERIC_CAREER" : "VIGNETTE_GENERIC_CHALLENGE";
            } else if (it.id.rfind("ITEM_ID_STAR", 0) == 0) {
                // STAR(4 - u) shown when rank < u (u = 1..3): 3 stars for gold .. 1 for bronze; challenges none [O].
                const int u = 4 - (it.id.back() - '0');
                it.visible = l.career && l.rank < u;
            }
        }
        drawPage(r, *lp, 0, y);
    }
    // PageProfileArrows: up / down while the list scrolls further [I].
    if (Page* ar = scripts_.page("PageProfileArrows"))
        for (Item& it : ar->items) {
            if (it.id == "ITEM_ID_FLECHE_UP") it.visible = p.scroll > 0;
            else if (it.id == "ITEM_ID_FLECHE_DOWN") it.visible = p.scroll < int(lines.size()) - 6;
            else it.visible = false;
            if (it.visible) drawItem(r, it, 0, 0);
        }
}

}  // namespace fuel::ui

namespace fuel::ui {

// P99: --career-audit views.
std::vector<Menu::ProfileRow> Menu::profileRows(int state, int sel) {
    const ProfileState keep = profile_;
    profile_.state = state, profile_.sel = sel;
    std::vector<ProfileRow> out;
    for (const ProfileLine& l : profileLines()) out.push_back({l.hub, l.enabled, l.text, l.text2, l.rank});
    profile_ = keep;
    return out;
}

std::string Menu::profileValue(const std::string& page, const std::string& id) {
    profileFillGeneral();
    const Page* p = scripts_.page(page);
    const Item* it = p ? p->find(id) : nullptr;
    return it && it->hasDynText ? it->dynText : std::string();
}

}  // namespace fuel::ui
