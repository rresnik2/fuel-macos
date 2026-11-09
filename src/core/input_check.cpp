// --inputcheck: FUEL's input layer (core/input_map.*) against vectors emulated on the player's FUEL.exe
// (Ghidra EmuRun, P47 notes §1.6) and the bindings read from the player's game data, compared by hash.
#include "input_check.hpp"

#include "input_map.hpp"
#include "remap.hpp"  // [port] remap

#include <cmath>
#include <map>
#include <cstdio>
#include <fstream>
#include <sstream>

namespace fuel::input {
namespace rm = fuel::input::remap;
namespace {

uint64_t fnv1a(const std::string& s) {
    uint64_t h = 0xcbf29ce484222325ull;
    for (unsigned char c : s) h = (h ^ c) * 0x100000001b3ull;
    return h;
}

std::string readFile(const std::string& p) {
    std::ifstream f(p, std::ios::binary);
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

struct Tally {
    int ok = 0, total = 0;
    void check(bool good, const char* what, double got, double want) {
        ++total;
        if (good) ++ok;
        std::printf("  %-44s %s (got %.8g, want %.8g)\n", what, good ? "ok" : "MISMATCH", got, want);
    }
    void near(const char* what, double got, double want, double tol = 2e-7) { check(std::fabs(got - want) <= tol, what, got, want); }
};

}  // namespace

int runInputCheck(const std::string& dataDir) {
    Tally t;
    std::printf("record update 0x669850 (emulated: thresholds 0.2/0.2/0.9, dt 1/60):\n");
    Record r = defaultRecord(Accel);
    const float dt = 1.0f / 60;
    updateRecord(r, 0.55f, dt);
    t.near("raw 0.55 -> value", r.value, 0.50000006);
    t.check(r.pressed && r.justPressed, "raw 0.55 -> pressed + just pressed", r.pressed + 2 * r.justPressed, 3);
    t.near("raw 0.55 -> held time", r.heldTime, 0.016666668);
    updateRecord(r, 0.55f, dt);
    t.check(r.pressed && !r.justPressed, "held -> pressed only", r.pressed + 2 * r.justPressed, 1);
    t.near("held -> held time", r.heldTime, 0.033333335);
    updateRecord(r, 0.1f, dt);
    t.check(!r.pressed && r.justReleased && r.value == 0, "raw 0.1 -> released, value 0", r.justReleased, 1);
    updateRecord(r, 0.95f, dt);
    t.near("raw 0.95 -> value", r.value, 1.0);
    Record sq = defaultRecord(Accel);
    sq.square = true;
    updateRecord(sq, 0.55f, dt);
    t.near("+0x2c == 0: raw 0.55 -> value", sq.value, 0.25000006);

    std::printf("pad inputs 0x83b7e0, sampler 0x42e160, camera 0x414dd0:\n");
    PadState p;
    p.connected = true;
    p.lx = -0.5f, p.ly = 1.0f, p.rt = 0.25f, p.buttons[PadY] = true;
    t.near("LANALOG_LEFT at lx -0.5", padValue(p, PadLXNeg), 0.5);
    t.near("LANALOG_RIGHT at lx -0.5", padValue(p, PadLXPos), 0.0);
    t.near("LANALOG_UP at ly 1", padValue(p, PadLYPos), 1.0);
    t.near("RT at 0.25", padValue(p, PadRT), 0.25);
    t.near("Y pressed", padValue(p, PadY), 1.0);
    t.near("camera axis 0.3", cameraAxis(0.3f), 0.0);
    t.near("camera axis 0.8", cameraAxis(0.8f), 0.3, 1e-6);
    t.near("camera axis -1", cameraAxis(-1.0f), -0.5);

    std::printf("bindings from the game data:\n");
    const std::string pc = readFile(dataDir + "/CommonComponent/Input/input.PC");
    const std::string map = readFile(dataDir + "/CommonComponent/DftMap.map");
    std::array<std::array<int, 2>, kActionCount> pads;
    int lines = 0;
    InputMap::parseInputPc(pc, pads, &lines);
    t.check(lines == 42, "input.PC InputDefAdd lines", lines, 42);
    t.check(fnv1a(pc) == 0x592948acd6ac8637ull, "input.PC hash", double(fnv1a(pc) & 0xffffff), double(0x592948acd6ac8637ull & 0xffffff));
    std::vector<std::vector<std::array<int, 4>>> ctx;
    const bool parsed = InputMap::parseDftMap(std::vector<uint8_t>(map.begin(), map.end()), ctx);
    t.check(parsed && ctx.size() == 3 && ctx[0].size() == 17 && ctx[1].size() == 8 && ctx[2].size() == 17, "DftMap.map contexts 17/8/17",
            parsed ? double(ctx.size()) : -1, 3);
    t.check(fnv1a(map) == 0xa2addfbd16e2d30bull, "DftMap.map hash", double(fnv1a(map) & 0xffffff), double(0xa2addfbd16e2d30bull & 0xffffff));
    t.check(pads[Left][0] == PadLXNeg && pads[Left][1] == PadDLeft, "LEFT_ACTION = LANALOG_LEFT + d-pad LEFT", pads[Left][0], PadLXNeg);
    t.check(pads[CameraReset][0] == PadRStick, "CAMERA_RESET = RSTICK", pads[CameraReset][0], PadRStick);

    InputMap m;
    std::string err;
    const bool loaded = m.load(dataDir, {}, &err);
    t.check(loaded, "InputMap::load (controls = fuel)", loaded, 1);
    if (loaded) {
        auto has = [&](int a, const char* key) {
            for (int k : m.keysOf(a))
                if (k == macKeyByName(key)) return true;
            return false;
        };
        t.check(has(Accel, "space") && has(Brake, "lctrl") && has(Left, "a") && has(Right, "d") && has(Up, "w") && has(Down, "s") &&
                    has(Handbrake, "num0") && has(Respawn, "f") && has(CamLeft, "left") && has(CameraReset, "num1"),
                "DftMap context 0 -> FUEL's PC keys", 1, 1);
        // A held W (UP_ACTION) and D: the stick lands on the unit circle (0x42e160).
        m.update(dt, {13, 2}, PadState{});
        const Block b = m.block();
        t.near("W + D -> stick x", b.stickX, std::sqrt(0.5), 1e-6);
        t.near("W + D -> stick z (lean)", b.stickZ, std::sqrt(0.5), 1e-6);
        // [nitro] the Nitro button = FUEL's BOOST slot: key E (DftMap.map), pad LB (input.PC's A is MENU_VALID too, the
        // free-roam race prompt), held -> Block::boost.
        t.check(Nitro == Boost && actionByName("nitro") == Boost && std::string(actionDisplayName(Boost)) == "Nitro", "Nitro = BOOST slot 16, display name",
                actionByName("nitro"), Boost);
        t.check(pads[Boost][0] == PadA && m.padsOf(Boost)[0] == PadLB && m.padsOf(Boost)[1] == -1 && has(Boost, "e"),
                "Nitro: input.PC's A moved to LB, key E (FUEL's BOOST key)", m.padsOf(Boost)[0], PadLB);
        {  // LB / E drive no other in-game action (context 0, LEFT .. BOOST; PAD_* / L_FRONT .. are not read while driving)
            int clash = 0;
            for (int a = Left; a < Boost; ++a) {
                for (int p : m.padsOf(a)) clash += p == PadLB;
                for (int k : m.keysOf(a)) clash += k == macKeyByName("e");
            }
            t.check(clash == 0, "Nitro: LB and E are free in-game", clash, 0);
        }
        PadState pa;
        pa.connected = true, pa.buttons[PadLB] = true;
        m.update(dt, {}, pa);
        const bool padHeld = m.block().boost;
        PadState pA;
        pA.connected = true, pA.buttons[PadA] = true;
        m.update(dt, {}, pA);
        const bool aFree = !m.block().boost && m.record(MenuValid).pressed;
        m.update(dt, {14}, PadState{});
        const bool keyHeld = m.block().boost;
        m.update(dt, {}, PadState{});
        t.check(padHeld && keyHeld && aFree && !m.block().boost, "Nitro held by LB and by E, released; A stays MENU_VALID only",
                padHeld + keyHeld + aFree, 3);
        InputMap pm;  // the port layout keeps E (its keys: W/S/A/D, arrows, Space, R, V)
        bool clash = false;
        if (pm.load(dataDir, {{"controls", "port"}}, nullptr))
            for (int a = Left; a <= Boost; ++a)
                if (a != Boost)
                    for (int k : pm.keysOf(a)) clash = clash || k == macKeyByName("e");
        bool portE = false;
        for (int k : pm.keysOf(Boost)) portE = portE || k == macKeyByName("e");
        t.check(portE && !clash && pm.padsOf(Boost)[0] == PadLB, "controls = port: Nitro keeps E and LB, no other in-game action on E", portE + !clash, 2);
        InputMap km;  // key.NITRO / pad.NITRO rebind it
        km.load(dataDir, {{"key.NITRO", "q"}, {"pad.NITRO", "a, rb"}}, nullptr);
        t.check(km.keysOf(Boost).size() == 1 && km.keysOf(Boost)[0] == macKeyByName("q"), "key.NITRO = q rebinds the Nitro key",
                km.keysOf(Boost).empty() ? -1 : km.keysOf(Boost)[0], macKeyByName("q"));
        t.check(km.padsOf(Boost)[0] == PadA && km.padsOf(Boost)[1] == PadRB, "pad.NITRO = a, rb rebinds the Nitro pad buttons", km.padsOf(Boost)[0] * 10 + km.padsOf(Boost)[1],
                PadA * 10 + PadRB);
        std::printf("%s", m.summary().c_str());
    }
    {  // ---- [port] remap: the bindings the remap page writes (core/remap.*, notes/remap.md)
        InputMap f, p;
        std::string err;
        const bool okF = f.load(dataDir, {{"controls", "fuel"}}, &err), okP = p.load(dataDir, {{"controls", "port"}}, &err);
        auto keys = [](const InputMap& m, int a) { return InputMap::keysSetting(m.keysOf(a)); };
        t.check(okF && okP && keys(p, MenuValid) == "return, numenter, space" && keys(p, MenuBack) == "escape, backspace" &&
                    keys(p, MenuL1) == "q, pageup" && keys(p, MenuR1) == "e, pagedown" && keys(p, MenuCancel) == "g" &&
                    keys(p, MenuR2) == "equals" && keys(p, MenuL2) == "minus" && keys(p, MenuSelect) == "x" && keys(p, MenuReset) == "backspace" &&
                    keys(p, MapCursorUp) == "i" && keys(p, MapActionKey) == "f" && keys(f, MenuL1) == "q, pageup",
                "remap: the menu keys' defaults = the old fixed keys", 1, 1);
        int named = 0, round = 0;
        for (int k = 0; k < 128; ++k)
            if (knownMacKey(k)) ++named, round += macKeyByName(macKeyName(k)) == k;
        t.check(named == round && named >= 90, "remap: every named key round-trips its name", round, named);
        // 0x4cd080: a key of the same context swaps (the whole entry), another context's does not.
        InputMap m = p;
        rm::Result r = rm::rebind(m, Accel, rm::Keyboard, macKeyByName("s"));
        t.check(r.changed && r.swapped == Brake && keys(m, Accel) == "s" && keys(m, Brake) == "w, up", "remap: ACCEL <- S swaps with BRAKE (W, Up)",
                r.swapped, Brake);
        r = rm::rebind(m, Handbrake, rm::Keyboard, macKeyByName("q"));
        t.check(r.changed && r.swapped < 0 && keys(m, MenuL1) == "q, pageup", "remap: HANDBRAKE <- Q: MENU_L1 (menus) keeps Q", r.swapped, -1);
        r = rm::rebind(m, Accel, rm::Pad, PadLT);
        t.check(r.swapped == Brake && m.padsOf(Accel)[0] == PadLT && m.padsOf(Brake)[0] == PadRT, "remap: pad ACCEL <- LT swaps with BRAKE (RT)",
                m.padsOf(Brake)[0], PadRT);
        r = rm::rebind(m, MenuValid, rm::Pad, PadX);  // input.PC shares X: MENU_RESET and MENU_VALI2 both take A
        t.check(r.others.size() == 2 && m.padsOf(MenuReset)[0] == PadA && m.padsOf(MenuVali2)[0] == PadA, "remap: MENU_VALID <- X: RESET and VALI2 take A",
                double(r.others.size()), 2);
        rm::rebind(m, MenuValid, rm::Pad, PadA);  // and back
        r = rm::rebind(m, MapCursorUp, rm::Pad, PadA);  // MAP_CURSOR_UP has no pad input: MENU_VALID would have none
        t.check(r.refused && m.padsOf(MenuValid)[0] == PadA, "remap: refused when MENU_VALID would be left without a button", r.refused, 1);
        // The settings lines, read back through load() (controls = port kept), and the reset per device.
        const auto lines = m.bindingLines(p);
        std::map<std::string, std::string> st = {{"controls", "port"}};
        for (const auto& [k, v] : lines) st[k] = v;
        InputMap back;
        back.load(dataDir, st, &err);
        bool same = back.layout == "port";
        for (int a = 0; a < kActionCount; ++a) same = same && back.keysOf(a) == m.keysOf(a) && back.padsOf(a) == m.padsOf(a);
        t.check(same && lines.count("key.ACCEL") && lines.at("key.ACCEL") == "s" && lines.count("pad.BRAKE") && lines.at("pad.BRAKE") == "RT" &&
                    !lines.count("key.MENU_VALID"),
                "remap: key.* / pad.* lines read back the same map", double(lines.size()), double(lines.size()));
        rm::resetDevice(m, p, rm::Keyboard);
        t.check(m.bindingLines(p).count("key.ACCEL") == 0 && m.bindingLines(p).count("pad.ACCEL") == 1, "remap: keyboard reset keeps the pad",
                double(m.bindingLines(p).size()), 2);
        // Fallbacks: the guarded actions never load without a key / a pad.
        InputMap g;
        g.load(dataDir, {{"key.MENU_VALID", "none"}, {"pad.PAUSE_MODE", "none"}, {"pad.MENU_CANCEL", "none"}, {"key.MENU_BACK", "nosuchkey"}}, &err);
        t.check(g.fallbacks == 3 && !g.keysOf(MenuValid).empty() && g.padsOf(Pause)[0] == PadStart && g.padsOf(MenuCancel)[0] < 0 &&
                    !g.keysOf(MenuBack).empty(),
                "remap: MENU_VALID / MENU_BACK / PAUSE fall back; MENU_CANCEL may be unbound", g.fallbacks, 3);
        // Pad capture: armed only once every input is back at rest.
        rm::PadCapture cap;
        PadState held, rest, stick;
        held.connected = rest.connected = stick.connected = true;
        held.buttons[PadA] = true, stick.ry = -0.9f;
        const int c1 = cap.step(held), c2 = cap.step(rest), c3 = cap.step(stick);
        t.check(c1 < 0 && c2 < 0 && c3 == PadRYNeg, "remap: pad capture waits for the release, then takes RS down", c3, PadRYNeg);
        t.check(rm::captureCancels(53) && !rm::capturable(53) && rm::capturable(macKeyByName("equals")) && !rm::capturable(200),
                "remap: Escape cancels, unnamed keys are not taken", 1, 1);
    }
    std::printf("inputcheck: %d of %d vectors match\n", t.ok, t.total);
    return t.ok == t.total ? 0 : 1;
}

}  // namespace fuel::input
