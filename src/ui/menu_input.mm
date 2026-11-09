// [port] remap: the menus' keys from the bindings (menu_input.h).
#include "ui/menu_input.h"

#import <GameController/GameController.h>

#include <algorithm>
#include <cstdio>

namespace fuel::ui {

namespace {

using namespace fuel::input;

struct Route {
    MenuKey key;
    int actions[3];
};
// FUEL's menu tests (menu_input.h) in the order a key's menu keys are given.
const Route kRoutes[] = {
    {MenuKey::Up, {MenuUp, PadUp, -1}},          {MenuKey::Down, {MenuDown, PadDown, -1}},
    {MenuKey::Left, {MenuLeft, PadLeft, -1}},    {MenuKey::Right, {MenuRight, PadRight, -1}},
    {MenuKey::Accept, {MenuValid, -1, -1}},      {MenuKey::Back, {MenuBack, MenuSelect, MenuStart}},
    {MenuKey::TabLeft, {MenuL1, -1, -1}},        {MenuKey::TabRight, {MenuR1, -1, -1}},
    {MenuKey::Vali2, {MenuVali2, -1, -1}},       {MenuKey::Reset, {MenuReset, -1, -1}},
    {MenuKey::MapAction, {MapActionKey, -1, -1}}, {MenuKey::MapTarget, {MenuCancel, -1, -1}},
    {MenuKey::MapUp, {MapCursorUp, -1, -1}},     {MenuKey::MapDown, {MapCursorDown, -1, -1}},
    {MenuKey::MapLeft, {MapCursorLeft, -1, -1}}, {MenuKey::MapRight, {MapCursorRight, -1, -1}},
    {MenuKey::MapZoomIn, {MenuR2, -1, -1}},      {MenuKey::MapZoomOut, {MenuL2, -1, -1}},
};
constexpr size_t kRouteCount = sizeof kRoutes / sizeof kRoutes[0];
static_assert(kRouteCount <= 32, "PadMenuRouter::held_");

// MENU_RESET only on the remap page, where it is the only key of its key (0x4cc870 resets before it tests Back).
std::vector<MenuKey> filter(std::vector<MenuKey> v, bool remapPage) {
    const bool reset = std::find(v.begin(), v.end(), MenuKey::Reset) != v.end();
    if (!reset) return v;
    if (remapPage) return {MenuKey::Reset};
    v.erase(std::remove(v.begin(), v.end(), MenuKey::Reset), v.end());
    return v;
}

void add(std::vector<MenuKey>& v, MenuKey k) {
    if (std::find(v.begin(), v.end(), k) == v.end()) v.push_back(k);
}

bool padRouted(const InputMap& m, int code) {
    for (const Route& r : kRoutes)
        for (int a : r.actions)
            if (a >= 0)
                for (int p : m.padsOf(a))
                    if (p == code) return true;
    return false;
}

}  // namespace

std::vector<MenuKey> menuKeysForKey(const InputMap& m, unsigned short key, bool remapPage) {
    std::vector<MenuKey> out;
    for (const Route& r : kRoutes)
        for (int a : r.actions)
            if (a >= 0)
                for (int k : m.keysOf(a))
                    if (k == key) add(out, r.key);
    out = filter(out, remapPage);
    if (out.empty()) {  // the fallback keys (bound to no menu action)
        bool routed = false;
        for (const Route& r : kRoutes)
            for (int a : r.actions)
                if (a >= 0)
                    for (int k : m.keysOf(a)) routed = routed || k == key;
        if (!routed && (key == 36 || key == 76)) out.push_back(MenuKey::Accept);
        if (!routed && key == 53) out.push_back(MenuKey::Back);
    }
    return out;
}

std::vector<MenuKey> PadMenuRouter::step(const InputMap& m, const PadState& p, bool remapPage, std::vector<MenuKey>* released) {
    std::vector<MenuKey> out, up;
    for (size_t i = 0; i < kRouteCount; ++i) {
        bool down = false;
        for (int a : kRoutes[i].actions) {
            if (a < 0) continue;
            for (int c : m.padsOf(a))
                if (c >= 0) down = down || padValue(p, c) > (c >= PadLXNeg ? 0.6f : 0.5f);
        }
        if (i == 4 && !down && !padRouted(m, PadA)) down = padValue(p, PadA) > 0.5f;  // the fallbacks (Accept, Back)
        if (i == 5 && !down && !padRouted(m, PadB)) down = padValue(p, PadB) > 0.5f;
        if (down && !held_[i]) add(out, kRoutes[i].key);
        if (!down && held_[i]) add(up, kRoutes[i].key);
        held_[i] = down;
    }
    if (released) *released = filter(up, remapPage);
    return filter(out, remapPage);
}

PadState readControllerPad() {
    PadState p;
    GCExtendedGamepad* g = GCController.controllers.firstObject.extendedGamepad;
    if (g == nil) return p;
    p.connected = true;
    p.buttons[PadA] = g.buttonA.pressed, p.buttons[PadB] = g.buttonB.pressed;
    p.buttons[PadX] = g.buttonX.pressed, p.buttons[PadY] = g.buttonY.pressed;
    p.buttons[PadLB] = g.leftShoulder.pressed, p.buttons[PadRB] = g.rightShoulder.pressed;
    p.buttons[PadLT] = g.leftTrigger.pressed, p.buttons[PadRT] = g.rightTrigger.pressed;
    p.buttons[PadLStick] = g.leftThumbstickButton.pressed, p.buttons[PadRStick] = g.rightThumbstickButton.pressed;
    p.buttons[PadBack] = g.buttonOptions.pressed, p.buttons[PadStart] = g.buttonMenu.pressed;
    p.buttons[PadDLeft] = g.dpad.left.pressed, p.buttons[PadDRight] = g.dpad.right.pressed;
    p.buttons[PadDUp] = g.dpad.up.pressed, p.buttons[PadDDown] = g.dpad.down.pressed;
    p.lt = g.leftTrigger.value, p.rt = g.rightTrigger.value;
    p.lx = g.leftThumbstick.xAxis.value, p.ly = g.leftThumbstick.yAxis.value;
    p.rx = g.rightThumbstick.xAxis.value, p.ry = g.rightThumbstick.yAxis.value;
    return p;
}

int runMenuInputCheck(const std::string& dataDir) {
    int bad = 0;
    auto expect = [&](bool ok, const std::string& what) {
        if (!ok) ++bad;
        std::printf("menu input: %s %s\n", ok ? "ok  " : "FAIL", what.c_str());
    };
    auto names = [](const std::vector<MenuKey>& v) {
        static const char* n[] = {"Up", "Down", "Left", "Right", "Accept", "Back", "TabLeft", "TabRight", "Vali2", "Reset", "MapAction",
                                  "MapTarget", "MapUp", "MapDown", "MapLeft", "MapRight", "MapZoomIn", "MapZoomOut"};
        std::string o;
        for (MenuKey k : v) o += (o.empty() ? "" : ",") + std::string(n[int(k)]);
        return o.empty() ? std::string("-") : o;
    };
    for (const char* layout : {"port", "fuel"}) {
        InputMap m;
        std::string err;
        if (!m.load(dataDir, {{"controls", layout}}, &err)) {
            expect(false, std::string("load (") + layout + "): " + err);
            continue;
        }
        // The keys the fixed menuKeyFor gave (before the remap) give the same menu keys from the default bindings.
        struct K {
            unsigned short code;
            MenuKey key;
        };
        const K fixed[] = {{126, MenuKey::Up},       {125, MenuKey::Down},     {123, MenuKey::Left},      {124, MenuKey::Right},
                           {36, MenuKey::Accept},    {76, MenuKey::Accept},    {49, MenuKey::Accept},     {53, MenuKey::Back},
                           {51, MenuKey::Back},      {12, MenuKey::TabLeft},   {116, MenuKey::TabLeft},   {14, MenuKey::TabRight},
                           {121, MenuKey::TabRight}, {15, MenuKey::Vali2},     {13, MenuKey::Up},         {1, MenuKey::Down},
                           {0, MenuKey::Left},       {2, MenuKey::Right},      {3, MenuKey::MapAction},   {5, MenuKey::MapTarget},
                           {34, MenuKey::MapUp},     {40, MenuKey::MapDown},   {38, MenuKey::MapLeft},    {37, MenuKey::MapRight},
                           {24, MenuKey::MapZoomIn}, {27, MenuKey::MapZoomOut}};
        int same = 0;
        std::string diff;
        for (const K& k : fixed) {
            const auto v = menuKeysForKey(m, k.code, false);
            if (v.size() == 1 && v[0] == k.key) ++same;
            else diff += " " + macKeyName(k.code) + "=" + names(v);
        }
        expect(same == int(sizeof fixed / sizeof fixed[0]), std::string("controls = ") + layout + ": the old fixed menu keys, same menu keys (" +
                                                                 std::to_string(same) + "/" + std::to_string(sizeof fixed / sizeof fixed[0]) + ")" + diff);
        expect(names(menuKeysForKey(m, 51, true)) == "Reset" && names(menuKeysForKey(m, 51, false)) == "Back",
               "Backspace: Reset on the remap page (0x4cc870 tests it before Back), Back elsewhere");
        expect(names(menuKeysForKey(m, 7, false)) == "Back", "X = MENU_SELECT = Back (0x4c9190)");
        // The pad: the old gamepadMenuKeys buttons.
        PadMenuRouter r;
        PadState rest;
        rest.connected = true;
        r.step(m, rest, false);
        auto press = [&](PadState p, bool remap = false) {
            p.connected = true;
            const auto v = r.step(m, p, remap);
            std::vector<MenuKey> up;
            r.step(m, rest, remap, &up);
            return names(v) + "/" + names(up);
        };
        PadState a, b, menu, lb, rb, x, du, sl, y, rt, view;
        a.buttons[PadA] = true, b.buttons[PadB] = true, menu.buttons[PadStart] = true, lb.buttons[PadLB] = true, rb.buttons[PadRB] = true;
        x.buttons[PadX] = true, du.buttons[PadDUp] = true, sl.lx = -0.8f, y.buttons[PadY] = true, rt.rt = 1.0f, view.buttons[PadBack] = true;
        const std::string got = press(a) + " " + press(b) + " " + press(menu) + " " + press(lb) + " " + press(rb) + " " + press(x) + " " + press(du) + " " +
                                press(sl) + " " + press(y) + " " + press(rt) + " " + press(view) + " " + press(x, true);
        expect(got == "Accept/Accept Back/Back Back/Back TabLeft/TabLeft TabRight/TabRight Vali2/Vali2 Up/Up Left/Left MapTarget/MapTarget "
                      "MapZoomIn/MapZoomIn Back/Back Reset/Reset",
               std::string("pad (") + layout + "): A B Menu LB RB X d-up stick-left Y RT View, X on the remap page: " + got);
        PadState half;
        half.connected = true, half.lx = -0.5f;
        expect(r.step(m, half, false).empty(), "a stick at 0.5 is no menu key (0.6, as before)");
        r.step(m, rest, false);
    }
    {  // Rebound: MENU_VALID on K, MENU_BACK on pad Y; the fallbacks.
        InputMap m;
        std::string err;
        m.load(dataDir, {{"controls", "port"}, {"key.MENU_VALID", "k"}, {"key.MAP_CURSOR_DOWN", "return"}, {"pad.MENU_VALID", "x"}}, &err);
        expect(names(menuKeysForKey(m, 40, false)) == "Accept" && names(menuKeysForKey(m, 36, false)) == "MapDown",
               "key.MENU_VALID = k: K accepts, Return is the map cursor key it was given");
        expect(names(menuKeysForKey(m, 76, false)) == "Accept", "Enter, bound to no menu action now: the fallback Accept");
        InputMap n;
        n.load(dataDir, {{"controls", "port"}, {"key.MENU_BACK", "none"}, {"key.MENU_START", "none"}, {"pad.MENU_VALID", "none"}}, &err);
        expect(n.fallbacks == 2 && names(menuKeysForKey(n, 53, false)) == "Back" && n.padsOf(MenuValid)[0] == PadA,
               "MENU_BACK / MENU_VALID left without a binding: the defaults again (" + std::to_string(n.fallbacks) + " fallbacks)");
        PadMenuRouter r;
        PadState rest, a;
        rest.connected = a.connected = true, a.buttons[PadA] = true;
        InputMap q;
        q.load(dataDir, {{"controls", "port"}, {"pad.MENU_VALID", "x"}, {"pad.MENU_RESET", "y"}, {"pad.MENU_CANCEL", "none"}}, &err);
        r.step(q, rest, false);
        expect(names(r.step(q, a, false)) == "Accept", "pad A bound to nothing: the fallback Accept");
    }
    return bad;
}

}  // namespace fuel::ui
