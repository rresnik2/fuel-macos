// FUEL's control mapping; see input_map.hpp and notes/p47-input-anim.md §1.
#include "input_map.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstring>
#include <fstream>
#include <sstream>

namespace fuel::input {
namespace {

// The engine's action names in index order (0x8e5100: hashed into 0xa66520.., the binding slots of
// game +0x18c..). Names only: the bindings come from the game's files.
const char* const kActionNames[kActionCount] = {
    "LEFT_ACTION", "RIGHT_ACTION", "UP_ACTION", "DOWN_ACTION", "CAMERA_LEFT_ACTION", "CAMERA_RIGHT_ACTION",
    "CAMERA_UP_ACTION", "CAMERA_DOWN_ACTION", "CAMERA_RESET", "PAUSE_MODE", "INFOS", "CONTEXT", "ACCEL", "BRAKE",
    "HANDBRAKE", "RESPAWN", "BOOST", "NEXTCONTEXT1", "PAD_DOWN", "PAD_LEFT", "PAD_UP", "PAD_RIGHT", "L_FRONT",
    "R_FRONT", "L_BACK", "R_BACK", "NEXTCONTEXT2", "MENU_UP", "MENU_DOWN", "MENU_RIGHT", "MENU_LEFT", "MENU_VALID",
    "MENU_CANCEL", "MENU_BACK", "MENU_L1", "MENU_R1", "MENU_L2", "MENU_R2", "MENU_START", "MENU_SELECT", "MENU_R3",
    "MENU_L3", "MENU_RESET", "MENU_VALI2", "END_CONTEXT",
    // [port] remap: the port's map keys
    "MAP_CURSOR_UP", "MAP_CURSOR_DOWN", "MAP_CURSOR_LEFT", "MAP_CURSOR_RIGHT", "MAP_ACTION"};

// The pad-name table 0xa67438 (filled at 0x8e7730), index = WinInput_Z vt[40] case.
const char* const kPadNames[kPadCount] = {
    "A", "B", "X", "Y", "LB", "RB", "LT", "RT", "LSTICK", "RSTICK", "BACK", "START", "LEFT", "RIGHT", "UP", "DOWN",
    "LANALOG_LEFT", "LANALOG_RIGHT", "LANALOG_UP", "LANALOG_DOWN", "RANALOG_LEFT", "RANALOG_RIGHT", "RANALOG_UP", "RANALOG_DOWN"};

std::string upper(std::string s) {
    for (char& c : s) c = char(std::toupper(static_cast<unsigned char>(c)));
    return s;
}
std::string lower(std::string s) {
    for (char& c : s) c = char(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

// macOS virtual key codes by name (port), and the DirectInput scan code of the same key.
struct KeyName {
    const char* name;
    int mac;
    int dik;
};
const KeyName kKeys[] = {
    {"a", 0, 30}, {"s", 1, 31}, {"d", 2, 32}, {"f", 3, 33}, {"h", 4, 35}, {"g", 5, 34}, {"z", 6, 44}, {"x", 7, 45},
    {"c", 8, 46}, {"v", 9, 47}, {"b", 11, 48}, {"q", 12, 16}, {"w", 13, 17}, {"e", 14, 18}, {"r", 15, 19},
    {"y", 16, 21}, {"t", 17, 20}, {"1", 18, 2}, {"2", 19, 3}, {"3", 20, 4}, {"4", 21, 5}, {"6", 22, 7}, {"5", 23, 6},
    {"9", 25, 10}, {"7", 26, 8}, {"8", 28, 9}, {"0", 29, 11}, {"o", 31, 24}, {"u", 32, 22}, {"i", 34, 23},
    {"p", 35, 25}, {"return", 36, 28}, {"l", 37, 38}, {"j", 38, 36}, {"k", 40, 37}, {"n", 45, 49}, {"m", 46, 50},
    {"tab", 48, 15}, {"space", 49, 57}, {"backspace", 51, 14}, {"escape", 53, 1}, {"rcmd", 54, 220},
    {"lcmd", 55, 219}, {"lshift", 56, 42}, {"lalt", 58, 56}, {"lctrl", 59, 29}, {"rshift", 60, 54},
    {"ralt", 61, 184}, {"rctrl", 62, 157}, {"num.", 65, 83}, {"num*", 67, 55}, {"num+", 69, 78}, {"num/", 75, 181},
    {"numenter", 76, 156}, {"num-", 78, 74}, {"num0", 82, 82}, {"num1", 83, 79}, {"num2", 84, 80}, {"num3", 85, 81},
    {"num4", 86, 75}, {"num5", 87, 76}, {"num6", 88, 77}, {"num7", 89, 71}, {"num8", 91, 72}, {"num9", 92, 73},
    {"left", 123, 203}, {"right", 124, 205}, {"down", 125, 208}, {"up", 126, 200}, {"home", 115, 199},
    {"end", 119, 207}, {"pageup", 116, 201}, {"pagedown", 121, 209}, {"delete", 117, 211},
    // [port] remap: the rest of a Mac keyboard a player may bind (DirectInput scan codes of the same keys). Punctuation
    // has word names: "," separates the keys of a key.<ACTION> line and "=" the settings key from its value.
    {"minus", 27, 12}, {"equals", 24, 13}, {"lbracket", 33, 26}, {"rbracket", 30, 27}, {"semicolon", 41, 39},
    {"quote", 39, 40}, {"comma", 43, 51}, {"period", 47, 52}, {"slash", 44, 53}, {"grave", 50, 41}, {"backslash", 42, 43},
    {"capslock", 57, 58}, {"insert", 114, 210}, {"num=", 81, 141}, {"f1", 122, 59}, {"f2", 120, 60}, {"f3", 99, 61},
    {"f4", 118, 62}, {"f5", 96, 63}, {"f6", 97, 64}, {"f7", 98, 65}, {"f8", 100, 66}, {"f9", 101, 67}, {"f10", 109, 68},
    {"f11", 103, 87}, {"f12", 111, 88}};

void addKey(std::vector<int>& v, int k) {
    if (k >= 0 && std::find(v.begin(), v.end(), k) == v.end()) v.push_back(k);
}

std::string readText(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    std::ostringstream ss;
    ss << f.rdbuf();
    return ss.str();
}

}  // namespace

const char* actionName(int a) { return a >= 0 && a < kActionCount ? kActionNames[a] : "?"; }

const char* actionDisplayName(int a) { return a == Boost ? "Nitro" : actionName(a); }  // [nitro]

int actionByName(const std::string& name) {
    const std::string u = upper(name);
    if (u == "NITRO") return Boost;  // [nitro] settings.txt key.NITRO = ...
    for (int a = 0; a < kActionCount; ++a)
        if (u == kActionNames[a] || u + "_ACTION" == kActionNames[a]) return a;
    return -1;
}

int padByName(const std::string& name) {
    const std::string u = upper(name);
    for (int p = 0; p < kPadCount; ++p)
        if (u == kPadNames[p]) return p;
    return -1;
}

const char* padInputName(int code) { return code >= 0 && code < kPadCount ? kPadNames[code] : "NULL"; }

float padValue(const PadState& p, int code) {
    if (!p.connected) return 0;
    auto half = [](float v) { return std::clamp(v, 0.0f, 1.0f); };  // clamp(s * 255/32767, 0, 255) / 255
    switch (code) {
        case PadLT: return half(p.lt);
        case PadRT: return half(p.rt);
        case PadLXNeg: return half(-p.lx);
        case PadLXPos: return half(p.lx);
        case PadLYPos: return half(p.ly);
        case PadLYNeg: return half(-p.ly);
        case PadRXNeg: return half(-p.rx);
        case PadRXPos: return half(p.rx);
        case PadRYPos: return half(p.ry);
        case PadRYNeg: return half(-p.ry);
        default: return code >= 0 && code < 16 && p.buttons[size_t(code)] ? 1.0f : 0.0f;
    }
}

void updateRecord(Record& r, float raw, float dt) {
    r.justPressed = r.justReleased = false;
    const bool was = r.pressed;
    r.raw = raw;
    const bool now = r.threshold < raw;  // +0x34 = +0x18 < raw
    float v = raw;
    if (r.remap) v = raw >= r.hi ? 1.0f : raw < r.lo ? 0.0f : (raw - r.lo) / (r.hi - r.lo);
    if (r.square) v *= v;
    r.value = v;
    if (!was && now) r.justPressed = true;
    if (was && !now) r.justReleased = true;
    r.pressed = now;
    r.heldTime = now ? r.heldTime + dt : 0.0f;
}

Record defaultRecord(int a) {
    Record r;  // 0x6697d0: threshold 0.5, lo 0.2, hi 0.9, remap on, +0x2c = -1 (no square)
    const bool analog = (a >= Left && a <= CamDown) || a == Accel || a == Brake || (a >= MenuUp && a <= MenuLeft);
    r.threshold = r.lo = analog ? 0.2f : 0.0f;  // 0x4e1170
    r.hi = 0.9f;
    return r;
}

float cameraAxis(float v) {
    if (v > -0.5f && v < 0.5f) return 0.0f;
    return v > 0 ? v - 0.5f : v + 0.5f;
}

int macKeyForDik(int dik) {
    for (const KeyName& k : kKeys)
        if (k.dik == dik) return k.mac;
    return -1;
}

int macKeyByName(const std::string& n) {
    const std::string l = lower(n);
    for (const KeyName& k : kKeys)
        if (l == k.name) return k.mac;
    return -1;
}

std::string macKeyName(int key) {
    for (const KeyName& k : kKeys)
        if (k.mac == key) return k.name;
    return "key" + std::to_string(key);
}

bool knownMacKey(int key) {
    for (const KeyName& k : kKeys)
        if (k.mac == key) return true;
    return false;
}

// [port] remap: FUEL's PC key texts (tt01 15608.. / 15638..: the names its own remap list showed) by macOS key; the other keys
// show their character or their settings name.
std::string keyDisplayName(int key, const std::function<std::string(uint32_t)>& text) {
    static const std::pair<int, uint32_t> kFuelText[] = {
        {51, 15638}, {48, 15639}, {36, 15640}, {59, 15641}, {62, 15642}, {56, 15643}, {60, 15644}, {53, 15645}, {58, 15608},
        {61, 15609}, {49, 15610}, {76, 15612}, {115, 15613}, {126, 15614}, {125, 15615}, {123, 15616}, {124, 15617},
        {119, 15618}, {116, 15619}, {121, 15620}, {114, 15621}, {117, 15622}};
    if (text)
        for (const auto& [mac, tt] : kFuelText)
            if (mac == key) {
                std::string t = text(tt);
                if (!t.empty()) return upper(t);
            }
    static const std::pair<int, const char*> kChar[] = {{27, "-"}, {24, "="}, {33, "["}, {30, "]"}, {41, ";"}, {39, "'"},
                                                        {43, ","}, {47, "."}, {44, "/"}, {50, "`"}, {42, "\\"}};
    for (const auto& [mac, c] : kChar)
        if (mac == key) return c;
    return upper(macKeyName(key));
}

// ---- PK26: the reverse of macKeyForDik (help-text key glyphs 0x4e2580 are chosen by scan code).
int dikForMacKey(int key) {
    for (const KeyName& k : kKeys)
        if (k.mac == key) return k.dik;
    return -1;
}
// ---- end PK26

bool InputMap::parseInputPc(const std::string& text, std::array<std::array<int, 2>, kActionCount>& pads, int* lines) {
    for (auto& p : pads) p = {-1, -1};
    std::istringstream in(text);
    int n = 0;
    for (std::string line; std::getline(in, line);) {
        std::istringstream ls(line);
        std::string cmd, action, p1, p2;
        if (!(ls >> cmd) || cmd != "InputDefAdd") continue;
        ls >> action >> p1 >> p2;
        const int a = actionByName(action);
        if (a < 0) continue;
        pads[size_t(a)] = {padByName(p1), padByName(p2)};  // 0x66a9b0: unknown names (NULL) -> -1
        ++n;
    }
    if (lines) *lines = n;
    return n > 0;
}

bool InputMap::parseDftMap(const std::vector<uint8_t>& b, std::vector<std::vector<std::array<int, 4>>>& ctx) {
    ctx.clear();
    auto u32 = [&](size_t o) {
        uint32_t v;
        std::memcpy(&v, &b[o], 4);
        return v;
    };
    if (b.size() < 12 || u32(0) != b.size()) return false;
    const uint32_t n = u32(8);
    size_t o = 12;
    for (uint32_t c = 0; c < n; ++c) {
        if (o + 4 > b.size()) return false;
        const uint32_t m = u32(o);
        o += 4;
        if (o + size_t(m) * 12 > b.size()) return false;
        std::vector<std::array<int, 4>> entries;
        for (uint32_t i = 0; i < m; ++i, o += 12) {
            int16_t d0, d1;
            uint16_t c0, c1;
            std::memcpy(&d0, &b[o], 2), std::memcpy(&c0, &b[o + 2], 2), std::memcpy(&d1, &b[o + 4], 2), std::memcpy(&c1, &b[o + 6], 2);
            entries.push_back({d0, c0, d1, c1});
        }
        ctx.push_back(std::move(entries));
    }
    return o == b.size();
}

bool InputMap::load(const std::string& dataDir, const std::map<std::string, std::string>& settings, std::string* error) {
    for (int a = 0; a < kActionCount; ++a) records_[size_t(a)] = defaultRecord(a), keys_[size_t(a)].clear();
    const std::string pc = readText(dataDir + "/CommonComponent/Input/input.PC");
    if (!parseInputPc(pc, pads_, nullptr)) {
        if (error) *error = "CommonComponent/Input/input.PC: no InputDefAdd lines";
        return false;
    }
    // [nitro] (the user's request: the Nitro button on a free pad button). FUEL's BOOST slot is pad A in input.PC, but A
    // is also MENU_VALID, which the game reads while driving (the free-roam race-start prompt, 0x4f6fd0). LB is free
    // in-game: FUEL's L_FRONT only toggled its debug CheatCodeHUD (0x444290), which the port does not have, and the menus
    // read it as MENU_L1 only while a menu is open. So the Nitro button defaults to LB (L1); pad.NITRO rebinds it.
    if (pads_[Boost] == std::array<int, 2>{PadA, -1}) pads_[Boost] = {PadLB, -1};
    if (auto it = settings.find("controls"); it != settings.end()) layout = lower(it->second);
    if (layout != "port") layout = "fuel";
    {  // FUEL's key map (DftMap.map) for every action; the port layout then replaces the driving keys only.
        const std::string mp = readText(dataDir + "/CommonComponent/DftMap.map");
        std::vector<std::vector<std::array<int, 4>>> ctx;
        if (!parseDftMap(std::vector<uint8_t>(mp.begin(), mp.end()), ctx)) {
            if (error) *error = "CommonComponent/DftMap.map: unreadable";
            return false;
        }
        const int base[3] = {0, NextContext1 + 1, NextContext2 + 1};  // contexts follow the NEXTCONTEXT separators
        for (size_t c = 0; c < ctx.size() && c < 3; ++c)
            for (size_t j = 0; j < ctx[c].size() && base[c] + int(j) < kActionCount; ++j) {
                const auto& e = ctx[c][j];
                std::vector<int>& k = keys_[size_t(base[c] + int(j))];
                if (e[0] == -1) addKey(k, macKeyForDik(e[1]));  // device -1 = keyboard
                if (e[2] == -1) addKey(k, macKeyForDik(e[3]));
            }
        // port: Mac keyboards have no numeric keypad -> a second key for the two keypad bindings.
        addKey(keys_[Handbrake], macKeyByName("lshift"));
        addKey(keys_[CameraReset], macKeyByName("v"));
    }
    // [port] remap: the menu keys. The port's menus answered to fixed keys (game_menu.mm menuKeyFor, before the remap) that
    // are not all DftMap's context 2 (MENU_L1 / R1 = T / Y there, Q / E and Page Up / Down in the port; the map's GPS key
    // MENU_CANCEL = E there, G in the port; zoom on - / =). The menus now read the bindings (ui/menu_input.*), so these are
    // the menu actions' default keys, in both layouts: the same keys do the same as before. MENU_UP .. MENU_LEFT keep
    // DftMap's arrows and PAD_UP .. PAD_RIGHT its W / S / A / D (FUEL's menu Up = MENU_UP or PAD_UP, 0x4c9450), MENU_VALI2
    // its R, MENU_RESET its Backspace (FUEL's remap page reset, 0x4cc870), MENU_SELECT its X (FUEL's menus read it as Back,
    // 0x4c9190; TT 5939 "CHECK YOUR MAP BY PRESSING STR_MENU_SELECT" shows it). MENU_START = Escape, the port's pause key
    // (TT 964 "PRESS STR_MENU_START AND SELECT 'CAREER'"; DftMap's Z: FUEL's menus read START as Accept, the port's as Back).
    {
        struct MenuKeys {
            int action;
            const char* names[3];
        };
        static const MenuKeys menu[] = {{MenuValid, {"return", "numenter", "space"}}, {MenuBack, {"escape", "backspace", nullptr}},
                                        {MenuL1, {"q", "pageup", nullptr}},          {MenuR1, {"e", "pagedown", nullptr}},
                                        {MenuCancel, {"g", nullptr, nullptr}},       {MenuL2, {"minus", nullptr, nullptr}},
                                        {MenuR2, {"equals", nullptr, nullptr}},      {MenuStart, {"escape", nullptr, nullptr}},
                                        {MapCursorUp, {"i", nullptr, nullptr}},
                                        {MapCursorDown, {"k", nullptr, nullptr}},    {MapCursorLeft, {"j", nullptr, nullptr}},
                                        {MapCursorRight, {"l", nullptr, nullptr}},   {MapActionKey, {"f", nullptr, nullptr}}};
        for (const MenuKeys& mk : menu) {
            keys_[size_t(mk.action)].clear();
            for (const char* n : mk.names)
                if (n) addKey(keys_[size_t(mk.action)], macKeyByName(n));
        }
        for (int a = MapCursorUp; a < kActionCount; ++a) pads_[size_t(a)] = {-1, -1};  // the pad moves the map with its focus
    }
    if (layout == "port") {  // the port's earlier driving layout (before P47) over FUEL's map
        // Before, this layout bound only these driving keys, so the menu / context actions (MENU_VALID: Return at a
        // race-start prompt, CONTEXT, INFOS, ...) had no key at all. Now they keep FUEL's keys, minus any key the port's
        // driving layout takes.
        struct PortKey {
            int action;
            const char* names[2];
        };
        static const PortKey port[] = {{Accel, {"w", "up"}}, {Brake, {"s", "down"}},  {Left, {"a", "left"}},  {Right, {"d", "right"}},
                                       {Handbrake, {"space", nullptr}}, {Respawn, {"r", nullptr}}, {CameraReset, {"v", nullptr}}};
        for (const PortKey& pk : port) keys_[size_t(pk.action)].clear();
        std::set<int> taken;
        for (const PortKey& pk : port)
            for (const char* n : pk.names)
                if (n) addKey(keys_[size_t(pk.action)], macKeyByName(n)), taken.insert(macKeyByName(n));
        for (int a = 0; a < kActionCount; ++a) {
            bool driving = false;
            for (const PortKey& pk : port) driving = driving || pk.action == a;
            if (driving || a > Boost) continue;  // the menu context (after NEXTCONTEXT) keeps its keys: it is not read while driving
            auto& k = keys_[size_t(a)];
            k.erase(std::remove_if(k.begin(), k.end(), [&](int c) { return taken.count(c) > 0; }), k.end());
        }
    }
    const auto defaultKeys = keys_;  // [port] remap: the layout's defaults (the guarded actions' fallback)
    const auto defaultPads = pads_;
    for (const auto& [k, v] : settings) {
        if (k.rfind("key.", 0) != 0) continue;
        const int a = actionByName(k.substr(4));
        if (a < 0) continue;
        keys_[size_t(a)].clear();
        std::string tok;
        std::istringstream ls(v);
        while (std::getline(ls, tok, ',')) {
            tok.erase(0, tok.find_first_not_of(" \t"));
            tok.erase(tok.find_last_not_of(" \t") + 1);
            addKey(keys_[size_t(a)], macKeyByName(tok));
        }
    }
    // pad.<ACTION> = input[, input] (input.PC names: A, B, X, Y, LB, RB, LT, RT, LSTICK, RSTICK, BACK, START, LEFT, ...;
    // NULL = none) replaces an action's two pad bindings (the write path of a remap page, InputMap::setPads).
    for (const auto& [k, v] : settings) {
        if (k.rfind("pad.", 0) != 0) continue;
        const int a = actionByName(k.substr(4));
        if (a < 0) continue;
        std::array<int, 2> pads{-1, -1};
        std::string tok;
        std::istringstream ls(v);
        for (int n = 0; n < 2 && std::getline(ls, tok, ',');) {
            tok.erase(0, tok.find_first_not_of(" \t"));
            tok.erase(tok.find_last_not_of(" \t") + 1);
            if (const int code = padByName(upper(tok)); code >= 0) pads[size_t(n++)] = code;
        }
        const std::string lv = lower(v);
        if (pads[0] >= 0 || lv == "none" || lv == "null") pads_[size_t(a)] = pads;  // [port] remap: "none" unbinds
    }
    // [port] remap: the menus need MENU_VALID / MENU_BACK and the pause key on both devices; a file that leaves one of them
    // without a key or a pad button gets that device's default again.
    fallbacks = 0;
    for (int a = 0; a < kActionCount; ++a) {
        if (!guarded(a)) continue;
        if (keys_[size_t(a)].empty()) keys_[size_t(a)] = defaultKeys[size_t(a)], ++fallbacks;
        if (pads_[size_t(a)][0] < 0 && pads_[size_t(a)][1] < 0) pads_[size_t(a)] = defaultPads[size_t(a)], ++fallbacks;
    }
    auto flag = [&](const char* k) {
        auto it = settings.find(k);
        return it != settings.end() && (it->second == "1" || it->second == "true" || it->second == "yes" || it->second == "on");
    };
    invertX = flag("invert_camera_x"), invertY = flag("invert_camera_y");
    drivingKeys_.clear();
    for (int a = Left; a <= Boost; ++a)  // PAUSE_MODE (Escape) stays with the viewer: it opens the port's pause menu
        if (a != Pause)
            for (int k : keys_[size_t(a)]) drivingKeys_.insert(k);
    return true;
}

void InputMap::update(float dt, const std::set<unsigned short>& keys, const PadState& pad) {
    for (int a = 0; a < kActionCount; ++a) {
        float raw = 0;  // 0x66aa30: the two pad bindings summed, clamped to [-1, 1]
        const auto& p = pads_[size_t(a)];
        if (p[0] >= 0) raw = padValue(pad, p[0]);
        if (p[1] >= 0) raw += padValue(pad, p[1]);
        raw = std::clamp(raw, -1.0f, 1.0f);
        for (int k : keys_[size_t(a)])
            if (keys.count(static_cast<unsigned short>(k))) raw = 1.0f;  // a held key reads full scale [I]
        updateRecord(records_[size_t(a)], raw, dt);
    }
}

Block InputMap::block() const {
    Block b;
    auto v = [&](int a) { return records_[size_t(a)].value; };
    auto disc = [](float& x, float& z) {  // 0x42e160: longer than 1 -> scaled onto the unit circle
        const float l = std::sqrt(x * x + z * z);
        if (l > 1.0f) x /= l, z /= l;
    };
    b.stickX = v(Right) - v(Left), b.stickZ = v(Up) - v(Down);
    disc(b.stickX, b.stickZ);
    b.camX = v(CamRight) - v(CamLeft), b.camZ = v(CamUp) - v(CamDown);
    disc(b.camX, b.camZ);
    b.accel = v(Accel), b.brake = v(Brake);
    b.handbrake = records_[Handbrake].pressed;
    b.boost = records_[Boost].pressed;
    b.respawn = records_[Respawn].justPressed;
    b.respawnHeld = records_[Respawn].pressed;
    return b;
}

CameraInput InputMap::camera() const {
    CameraInput c;
    auto v = [&](int a) { return records_[size_t(a)].value; };
    c.viewChange = records_[CameraReset].justPressed;
    float x = cameraAxis(v(CamRight) - v(CamLeft));
    if (invertX) x = -x;
    float y = cameraAxis(v(CamUp) - v(CamDown));
    if (!invertY) y = -y;
    c.x = x, c.y = y;
    c.active = x != 0 || y != 0;
    return c;
}

bool InputMap::drivesAction(unsigned short key) const { return drivingKeys_.count(key) != 0; }

// ---- [port] remap: the write path and the settings lines of the remap page -----------------------------------------
void InputMap::setKeys(int a, const std::vector<int>& keys) {
    if (a < 0 || a >= kActionCount) return;
    keys_[size_t(a)].clear();
    for (int k : keys) addKey(keys_[size_t(a)], k);
    if (a >= Left && a <= Boost && a != Pause) {  // the viewer leaves the driving keys to the game
        drivingKeys_.clear();
        for (int d = Left; d <= Boost; ++d)
            if (d != Pause)
                for (int k : keys_[size_t(d)]) drivingKeys_.insert(k);
    }
}

std::string InputMap::settingName(int a) {
    if (a == Boost) return "NITRO";  // [nitro] key.NITRO / pad.NITRO
    std::string n = actionName(a);
    if (n.size() > 7 && n.compare(n.size() - 7, 7, "_ACTION") == 0) n.resize(n.size() - 7);
    return n;
}

std::string InputMap::keysSetting(const std::vector<int>& keys) {
    std::string out;
    for (int k : keys)
        if (knownMacKey(k)) out += (out.empty() ? "" : ", ") + macKeyName(k);
    return out.empty() ? "none" : out;
}

std::string InputMap::padsSetting(const std::array<int, 2>& pads) {
    std::string out;
    for (int p : pads)
        if (p >= 0 && p < kPadCount) out += (out.empty() ? "" : ", ") + std::string(kPadNames[p]);
    return out.empty() ? "none" : out;
}

bool InputMap::isBindingKey(const std::string& k) {
    return (k.rfind("key.", 0) == 0 || k.rfind("pad.", 0) == 0) && actionByName(k.substr(4)) >= 0;
}

std::map<std::string, std::string> InputMap::withoutBindings(const std::map<std::string, std::string>& settings) {
    std::map<std::string, std::string> out;
    for (const auto& [k, v] : settings)
        if (!isBindingKey(k)) out[k] = v;
    return out;
}

std::map<std::string, std::string> InputMap::bindingLines(const InputMap& d) const {
    std::map<std::string, std::string> out;
    for (int a = 0; a < kActionCount; ++a) {
        if (a == NextContext1 || a == NextContext2 || a == EndContext) continue;
        auto same = [](std::vector<int> x, std::vector<int> y) {
            std::sort(x.begin(), x.end()), std::sort(y.begin(), y.end());
            return x == y;
        };
        if (!same(keys_[size_t(a)], d.keys_[size_t(a)])) out["key." + settingName(a)] = keysSetting(keys_[size_t(a)]);
        if (pads_[size_t(a)] != d.pads_[size_t(a)]) out["pad." + settingName(a)] = padsSetting(pads_[size_t(a)]);
    }
    return out;
}
// ---- end remap

std::string InputMap::summary() const {
    std::ostringstream o;
    for (int a = 0; a < kActionCount; ++a) {
        if (a == NextContext1 || a == NextContext2 || a == EndContext) continue;
        o << kActionNames[a] << ": pad";
        for (int p : pads_[size_t(a)])
            if (p >= 0) o << ' ' << kPadNames[p];
        o << "; keys";
        for (int k : keys_[size_t(a)]) o << ' ' << macKeyName(k);
        o << '\n';
    }
    return o.str();
}

}  // namespace fuel::input
