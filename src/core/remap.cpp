// [port] remap: see remap.hpp and notes/remap.md.
#include "core/remap.hpp"

#include <algorithm>

namespace fuel::input::remap {

namespace {

// FUEL's list texts (tt01: 15700 General, 15701.. the context 0 actions, 15718 END OF ACTION, 15719 Map, 15720.., 15728
// END OF ACTION, 15729 Menu, 15730.., 15747 END OF ACTION, 15748 END OF CONTEXT): the action at slot j of context c is
// the text 15701 + j, 15720 + j, 15730 + j (0x4cc650 walks the table from +0x46c and steps over the titles).
constexpr uint32_t kContextTitle[3] = {15700, 15719, 15729};
constexpr uint32_t kContextFirst[3] = {15701, 15720, 15730};
constexpr int kContextBase[3] = {Left, PadDown, MenuUp};
constexpr int kContextEnd[3] = {Boost, RBack, MenuVali2};

bool hiddenFuel(int a) { return a == LFront || a == RFront || a == LBack || a == RBack; }  // "Map Debug" x 4

}  // namespace

const char* portName(int a) {
    switch (a) {
        case Boost: return "Nitro";  // [nitro] FUEL skips TT 15717 "Boost" (0x4cc650: iVar5 != 0x3d65)
        case MapCursorUp: return "Map cursor up";
        case MapCursorDown: return "Map cursor down";
        case MapCursorLeft: return "Map cursor left";
        case MapCursorRight: return "Map cursor right";
        case MapActionKey: return "Map action";
        default: return nullptr;
    }
}

uint32_t actionText(int a) {
    if (portName(a)) return 0;
    for (int c = 0; c < 3; ++c)
        if (a >= kContextBase[c] && a <= kContextEnd[c]) {
            // [port] the port's tabs: MENU_L1 goes to the tab on the left, MENU_R1 to the right (FUEL's 15737 "Next menu" /
            // 15738 "Previous menu" sit the other way round); the map zooms in on MENU_R2 (FUEL's own help TT 486 "R2 Zoom in
            // on zone") and out on MENU_L2 (15739 / 15740 swapped likewise).
            if (a == MenuL1) return 15738;
            if (a == MenuR1) return 15737;
            if (a == MenuL2) return 15740;
            if (a == MenuR2) return 15739;
            return kContextFirst[c] + uint32_t(a - kContextBase[c]);
        }
    return 0;
}

bool listed(int a) {
    if (a < 0 || a >= kActionCount || a == NextContext1 || a == NextContext2 || a == EndContext) return false;
    return !hiddenFuel(a);
}

int group(int a) {
    if (!listed(a)) return -1;
    return a <= Boost ? 0 : 1;
}

std::vector<Row> rows() {
    std::vector<Row> out;
    for (int c = 0; c < 3; ++c) {
        Row t;
        t.kind = Row::Title, t.tt = kContextTitle[c];
        out.push_back(t);
        for (int a = kContextBase[c]; a <= kContextEnd[c]; ++a) {
            if (!listed(a)) continue;
            Row r;
            r.action = a, r.tt = actionText(a);
            if (const char* n = portName(a)) r.name = n;
            out.push_back(r);
        }
        if (c == 1)  // the port's map keys close the Map context
            for (int a = MapCursorUp; a <= MapActionKey; ++a) {
                Row r;
                r.action = a, r.name = portName(a);
                out.push_back(r);
            }
    }
    Row reset;  // [port] "Reset to defaults": one cell per device (FUEL: MENU_RESET on the page resets the whole map)
    reset.kind = Row::Reset, reset.name = "Reset to defaults";
    out.push_back(reset);
    return out;
}

Result rebind(InputMap& m, int a, Device d, int code) {
    Result res;
    if (!listed(a) || code < 0) return res;
    const int g = group(a);
    if (d == Keyboard) {
        const std::vector<int> old = m.keysOf(a);
        if (old.size() == 1 && old[0] == code) return res;
        std::vector<int> others;
        for (int b = 0; b < kActionCount; ++b)
            if (b != a && group(b) == g)
                for (int k : m.keysOf(b))
                    if (k == code) others.push_back(b);
        // 0x4cd080: the two entries swap whole (FUEL's 12-byte DftMap entry, primary and secondary): the other action
        // takes the selected one's old binding.
        std::vector<int> mine;
        for (int k : old)
            if (k != code) mine.push_back(k);
        for (int b : others)
            if (mine.empty() && InputMap::guarded(b)) return res.refused = true, res;
        for (int b : others) m.setKeys(b, mine);
        m.setKeys(a, {code});
        res.changed = true, res.others = others, res.swapped = others.empty() ? -1 : others[0];
        return res;
    }
    const std::array<int, 2> old = m.padsOf(a);
    if (old[0] == code && old[1] < 0) return res;
    std::vector<int> others;
    for (int b = 0; b < kActionCount; ++b)
        if (b != a && group(b) == g)
            for (int p : m.padsOf(b))
                if (p == code && (others.empty() || others.back() != b)) others.push_back(b);
    std::array<int, 2> np{-1, -1};  // the whole binding swaps (as the keyboard's)
    size_t n = 0;
    for (int p : old)
        if (p >= 0 && p != code && n < 2) np[n++] = p;
    for (int b : others)
        if (n == 0 && InputMap::guarded(b)) return res.refused = true, res;
    for (int b : others) m.setPads(b, np);
    m.setPads(a, {code, -1});
    res.changed = true, res.others = others, res.swapped = others.empty() ? -1 : others[0];
    return res;
}

void resetDevice(InputMap& m, const InputMap& defaults, Device d) {
    for (int a = 0; a < kActionCount; ++a) {
        if (d == Keyboard) m.setKeys(a, defaults.keysOf(a));
        else m.setPads(a, defaults.padsOf(a));
    }
}

bool captureCancels(int macKey) { return macKey == 53; }  // Escape
bool capturable(int macKey) { return !captureCancels(macKey) && knownMacKey(macKey); }

int PadCapture::step(const PadState& p) {
    if (!p.connected) return -1;
    auto down = [&](int code, float level) {
        const float v = padValue(p, code);
        const bool stick = code >= PadLXNeg;
        return v > (stick ? 0.6f : 0.5f) * level;
    };
    if (!armed) {  // every input back to rest (0.5 of the press levels) first
        for (int c = 0; c < kPadCount; ++c)
            if (down(c, 0.5f)) return -1;
        armed = true;
        return -1;
    }
    for (int c = 0; c < kPadCount; ++c)
        if (down(c, 1.0f)) return c;
    return -1;
}

}  // namespace fuel::input::remap
