// FUEL's tutorials (P56 in-game, P62 menu side and gate; notes/p56-hud2.md, p62-menus.md): the tutorial manager
// (game+0x15c: +4 entries of 0x1c bytes {+0 seen, +1 off, +4 index, +8 icon, +0xc text list, +0x14 title, +0x18 immediate},
// +8 queue, +0x10 text page, +0x14 busy, +0x18 queue clock, +0x1c close clock) and the page `tutorial` (InGamePlayer
// +0x74f8, vtable PageTutorial; items ITEM_ID_TUTO_TITLE +0x98, ITEM_ID_TUTO_TEXT +0x9c, the button +0xa0,
// ITEM_ID_TUTO_ICONE +0xa4).
//  - content 0x56d900 (jump table 0x56dea0): per index the icon (+8), the text pages (+0xc list), the title (+0x14) and,
//    for 11..15, the immediate flag (+0x18 = 1: the menu-side entries);
//  - 0x56e420(i): an entry neither seen nor off is queued once; an entry without the immediate flag only when the gate
//    0x56e620 allows it now and no race runs (race +0x48 == 0);
//  - 0x56e340 per update: queue empty -> the close clock runs and 0.5 s after a close the manager is free again (+0x14 = 0);
//    FREE RIDE (0) is queued while the profile's free-ride distance (game+0x3408 km x 1000 + +0x340c m) is in (0.1, 0.25)
//    and the player's vehicle is in a normal state (0x54bc80(0x5f)); queue not empty -> +0x18 += dt, and an immediate
//    front entry, or any after 1 s, goes to 0x56e5b0: gate refused -> +0x18 = 0; front already seen/off -> dropped;
//    else 0x56e8a0 opens the page (busy, seen = 1);
//  - OK 0x56e4c0: the next text page; after the last: seen = off = 1, the entry leaves the queue, the next queued one
//    opens at once, else the page closes (close clock = 0); closing 15 (SMART GPS) resumes the garage start
//    (garage mode +0xe4: 1 -> 0x47eee0, 2 -> 0x47efb0);
//  - menu triggers: the tab switch 0x46ee10 (tab 0 Race -> 11), garage 0x4782f0 (-> 12), map 0x482410 (-> 13), the garage
//    vehicle choice 0x4786e0 (14 not seen -> 14 and 15, else 15 not seen -> 15);
//  - save: SECTION_TUTORIAL (career.sav, CareerState::tutorials). A P56 settings.txt mask (`tutorials_seen`) migrates in.
// Port choices ("port"): 0x56e620's engine objects map to what the port knows (below); Back also acts as OK.
#include "ui/menu.h"

#include "core/freeroam.hpp"
#include "ui/ui_renderer.h"

namespace fuel::ui {

const Menu::TutorialDef* Menu::tutorialDef(int i) {
    // 0x56d900, cases 0..15: icon (AsoboCrc string), text pages, title, immediate flag (+0x18 = 1 for 11..15).
    static const TutorialDef k[16] = {
        {"ICONE_EVENT_END_RACE", {963, 964, 965}, 962},           // 0 FREE RIDE
        {"icone_end_race", {967, 968}, 966},                      // 1 CHALLENGE
        {"ICONE_LIVERY_END_RACE", {970}, 969},                    // 2 VEHICLE LIVERY
        {"ICONE_VISTAPOINT_icone", {972}, 971},                   // 3 VISTA POINT
        {"ICONE_MAVERICK_END_RACE", {976, 977}, 975},             // 4 DOPPLER TRUCK
        {"ICONE_MAVERICK_END_RACE_VIOLET", {979}, 978},           // 5 MAVERICK VEHICLE
        {"ICONE_MAVERICK_END_RACE", {974}, 973},                  // 6 FUEL DRUM
        {"ICONE_MAVERICK_END_RACE_GREEN", {5951}, 981},           // 7 VISTA POINT DOPPLER TRUCK
        {"ICONE_MAVERICK_END_RACE", {5950}, 980},                 // 8 CHALLENGE DOPPLER TRUCK
        {"ICONE_MAVERICK_END_RACE_BLUE", {5952}, 982},            // 9 LIVERY DOPPLER TRUCK
        {"ICONE_MAVERICK_END_RACE_VIOLET", {984}, 978},           // 10 MAVERICK caught
        {"ICONE_EVENT_END_RACE", {2911, 2912, 2913, 2914}, 2910, true}, // 11 START YOUR ENGINES! (race menu)
        {"ICONE_VEHICLE_END_RACE", {2916}, 2915, true},                 // 12 WELCOME TO THE GARAGE
        {"ICONE_EVENT_END_RACE", {2919, 2920}, 2918, true},             // 13 USE YOUR MAP!
        {"ICONE_VEHICLE_END_RACE", {2922, 2923}, 2921, true},           // 14 GETTING BACK ON TRACK!
        {"ICONE_VEHICLE_END_RACE", {5976}, 5975, true},                 // 15 SMART GPS
    };
    return i >= 0 && i < 16 ? &k[i] : nullptr;
}

CareerState::Tutorial& Menu::tutorialFlags(int i) {
    static CareerState::Tutorial local[16];  // no career (scripts missing): kept for the session only
    if (career_ && size_t(i) < career_->state().tutorials.size()) return career_->state().tutorials[size_t(i)];
    return local[i & 15];
}

// 0x56e620, as far as the port has the objects it reads [O for the structure, port for the mapping]:
//  player + vehicle present, HUD flag [0xa653a5], world loaded -> always true in the port's windows;
//  +0x14 busy -> tutorial_.busy;
//  InGamePlayer pages (+0x908: +0x5c, +0x98), fades (0x694290(0)), 0x433940 +0x6c -> port: no menu open, for the in-game
//  (not immediate) entries only, since 11..15 open from the menus;
//  front entry not immediate and 0x57b550 (another notification pending) -> port: a MessagesHUD message is showing;
//  a race object exists -> its state must be 0, 5, 6 or 7 -> port: no race HUD (the ended states show the results menu);
//  front entry not immediate -> 0x54bc80(0x5f) (vehicle normal) and race state 0 -> port: not in a race.
bool Menu::tutorialGate(bool flaggedFront) const {
    if (tutorial_.busy) return false;
    if (rtcFreeze_) return false;  // PK17: 0x56e620 -> 0x534b40(2): no tutorial while a freeze-player RTC runs
    // port: the menu-side entries 11..13 belong to the page they explain (Race, Garage, Map tabs) and wait for the menus
    // (the in-game overlay builds its menu on the Race tab before it is shown).
    if (!tutorial_.queue.empty() && tutorial_.queue.front() >= 11 && tutorial_.queue.front() <= 13 && !tutorial_.menuOpen) return false;
    if (flaggedFront) return true;
    if (tutorial_.menuOpen || hudRace_) return false;
    if (freeRoam_ && freeRoam_->messages().showing()) return false;
    return true;
}

void Menu::queueTutorial(int i) {  // 0x56e420
    const TutorialDef* d = tutorialDef(i);
    if (!d) return;
    const CareerState::Tutorial& f = tutorialFlags(i);
    if (f.seen || f.off) return;
    if (!d->immediate && (!tutorialGate(false) || hudRace_)) return;
    if (tutorial_.shown == i) return;
    for (int q : tutorial_.queue)
        if (q == i) return;
    tutorial_.queue.push_back(i);  // 0x56edb0
}

void Menu::tutorialOpenFront() {  // 0x56e5b0
    const int front = tutorial_.queue.front();
    const TutorialDef* d = tutorialDef(front);
    if (!tutorialGate(d && d->immediate)) {
        tutorial_.clock = 0;
        return;
    }
    const CareerState::Tutorial& f = tutorialFlags(front);
    if (f.seen || f.off) {  // 0x56ed30: dropped
        tutorial_.queue.erase(tutorial_.queue.begin());
        return;
    }
    tutorial_.busy = true, tutorial_.page = 0, tutorial_.shown = front;  // 0x56e8a0
    tutorialFlags(front).seen = 1;
}

// ---- PK17: 0x56eb20 (PAUSE 0x4442f0): for i = count - 1 .. 0, entry i without the immediate flag -> 0x56ed30 (removed).
void Menu::pauseDropTutorials() {
    for (int i = int(tutorial_.queue.size()) - 1; i >= 0; --i) {
        const int q = tutorial_.queue[size_t(i)];
        const TutorialDef* d = tutorialDef(q);
        if (!d || !d->immediate) tutorial_.queue.erase(tutorial_.queue.begin() + i);
    }
}
// ---- end PK17

void Menu::updateTutorial(float dt, bool menuOpen) {  // 0x56e340
    tutorial_.menuOpen = menuOpen;
    if (tutorial_.shown >= 0) return;
    // MessagesHUD pushes start tutorials (0x4af9d0): kind 1 challenge spotted -> 1, 2 livery -> 2, 3 vista -> 3,
    // 12 fuel drum -> 6. The port watches the shown message instead of the push.
    if (freeRoam_) {
        const fuel::MessageQueue& q = freeRoam_->messages();
        const int kind = q.showing() ? q.current().kind : 0;
        if (kind != tutorial_.lastKind) {
            tutorial_.lastKind = kind;
            if (kind == 1) queueTutorial(1);
            if (kind == 2) queueTutorial(2);
            if (kind == 3) queueTutorial(3);
            if (kind == 12) queueTutorial(6);
        }
    }
    if (tutorial_.queue.empty()) {
        // FREE RIDE: a profile that has just started driving (free-ride distance 0.1 .. 0.25 m) in free roam.
        if (career_ && freeRoam_ && sessionEvent_.empty() && !menuOpen) {
            const double d = career_->freeRideDistance();
            if (d > 0.1 && d < 0.25) queueTutorial(0);
        }
        if (tutorial_.closeClock >= 0) {
            tutorial_.closeClock += dt;
            if (tutorial_.closeClock > 0.5f) tutorial_.closeClock = -1, tutorial_.busy = false;
        }
        if (tutorial_.queue.empty()) return;
    }
    tutorial_.clock += dt;
    const TutorialDef* d = tutorialDef(tutorial_.queue.front());
    if ((d && d->immediate) || tutorial_.clock > 1.0f) tutorialOpenFront();
}

void Menu::tutorialKey(MenuKey k) {  // OK button 0x4cf1a0 -> 0x56e4c0
    if (tutorial_.shown < 0 || (k != MenuKey::Accept && k != MenuKey::Back)) return;
    const TutorialDef* d = tutorialDef(tutorial_.shown);
    if (d && tutorial_.page + 1 < int(d->texts.size())) {
        ++tutorial_.page;
        return;
    }
    const int done = tutorial_.shown;
    CareerState::Tutorial& f = tutorialFlags(done);
    f.seen = f.off = 1;
    tutorial_.page = 0;
    if (!tutorial_.queue.empty() && tutorial_.queue.front() == done) tutorial_.queue.erase(tutorial_.queue.begin());
    tutorial_.shown = -1;
    saveCareer();  // port: FUEL writes SECTION_TUTORIAL with the next SaveGameData; the port's sessions end by exec
    if (!tutorial_.queue.empty()) {
        tutorial_.busy = false;
        tutorialOpenFront();  // 0x56e8a0 at once
        if (tutorial_.shown >= 0) return;
    }
    tutorial_.queue.clear();
    tutorial_.closeClock = 0;  // +0x1c
    if (done == 15 && pendingStart_.kind != MenuRequest::None) request_ = pendingStart_, pendingStart_ = {};
}

// 0x4786e0: the vehicle chosen in the garage. Not seen 14 -> 14 and 15; else not seen 15 -> 15; the start then waits for
// 15's OK (0x56e4c0). Without them the start goes on at once.
void Menu::requestStart(const MenuRequest& r) {
    if (!tutorialFlags(14).seen) queueTutorial(14), queueTutorial(15);
    else if (!tutorialFlags(15).seen) queueTutorial(15);
    bool waits = false;
    for (int q : tutorial_.queue) waits |= q == 15;
    if (waits) pendingStart_ = r;
    else request_ = r;
}

bool Menu::showTutorial(int i) {
    if (!tutorialDef(i)) return false;
    tutorial_.shown = i, tutorial_.page = 0;
    return true;
}

void Menu::drawTutorial(UiRenderer& r) {
    const TutorialDef* d = tutorialDef(tutorial_.shown);
    Page* p = scripts_.page("tutorial");  // page 145 (InGamePlayer page set +0xc0 [0x244 / 4], checked in 0x4cee50)
    if (!d || !p) return;
    for (Item& it : p->items) {
        if (it.id == "ITEM_ID_TUTO_TITLE") it.hasDynText = true, it.dynText = textOf(d->title);
        if (it.id == "ITEM_ID_TUTO_TEXT") {
            // The text item wraps inside its width (font engine 0x6993a0) [port: greedy word wrap].
            const std::string t = textOf(d->texts[size_t(std::min<int>(tutorial_.page, int(d->texts.size()) - 1))]);
            TextStyle st;
            st.font = it.font, st.height = it.h, st.upper = it.upper;
            std::string out, line, word;
            auto flush = [&]() {
                if (word.empty()) return;
                const std::string cand = line.empty() ? word : line + " " + word;
                if (!line.empty() && r.textWidth(cand, st) > it.w) out += line + "\n", line = word;
                else line = cand;
                word.clear();
            };
            for (char c : t) {
                if (c == ' ') flush();
                else word += c;
            }
            flush();
            it.hasDynText = true, it.dynText = out + line;
        }
        if (it.id == "ITEM_ID_TUTO_ICONE") it.imgNormal = it.imgEnabled = it.imgFocus = d->icon;  // the three image states (+0x78..+0x80)
        if (it.id == "ITEM_ID_TUTO_BTN_OK") it.focus = true;
    }
    drawPage(r, *p);
}

}  // namespace fuel::ui
