// FUEL's control mapping (P47, notes ~/re-work/fuel/notes/p47-input-anim.md §1), ported 1:1 above the
// platform layer:
//  - 45 actions in the engine's order (name hashes stored 0x8e5100.. -> 0xa66520; lookup 0x4e1050), with
//    their pad bindings from CommonComponent/Input/input.PC (`InputDefAdd ACTION PAD1 PAD2`, handler
//    0x4e2e00 -> InputPlatForm_Z vt[11] 0x66a9b0) and their keyboard bindings from CommonComponent/DftMap.map
//    (loaded by 0x66af10; per context `n x {s16 dev, u16 DIK code, s16 dev2, u16 code2, u32}`, dev -1 =
//    keyboard, -3 = none; registry "Controls\Control.cc.ss.Primary/Secondary" overrides, 0x83d390). Both
//    files are read at run time from the player's game data.
//  - the 24 physical pad inputs of WinInput_Z vt[40] 0x83b7e0 (XInput layout: buttons 1.0, triggers b/255,
//    stick half-axes |s|/32767), summed over the two pad bindings and clamped to [-1, 1] (0x66aa30);
//  - the per-action record update 0x669850 (record defaults 0x6697d0, per-action thresholds 0x4e1170):
//    pressed = raw > threshold, value = (raw - lo) / (hi - lo) clamped (lo = threshold = 0.2 for the stick,
//    camera and pedal actions, 0 for buttons; hi = 0.9), squared only when record +0x2c == 0 (never set:
//    linear), just-pressed / just-released edges, held time;
//  - the player sampler 0x42e160: left stick = (RIGHT - LEFT, UP - DOWN), right stick = (CAM_RIGHT -
//    CAM_LEFT, CAM_UP - CAM_DOWN), each scaled back onto the unit disc when longer than 1;
//  - the chase camera's input 0x414dd0 (from CameraMove_G 0x412e80): look axes with a 0.5 dead zone and
//    the 0.5 removed (range -0.5..0.5), X negated when "camera X inverted" (game +0x271), Y negated unless
//    "camera Y inverted" (game +0x2a0); view change = CAMERA_RESET just pressed (block +0x18d).
// The vehicle's steer shaping (s|s| and the four rate-limited channels, 0x52e830) stays in the physics.
#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <map>
#include <set>
#include <string>
#include <vector>

namespace fuel::input {

enum Action : int {
    Left, Right, Up, Down, CamLeft, CamRight, CamUp, CamDown, CameraReset, Pause, Infos, Context, Accel, Brake,
    Handbrake, Respawn, Boost, NextContext1, PadDown, PadLeft, PadUp, PadRight, LFront, RFront, LBack, RBack,
    NextContext2, MenuUp, MenuDown, MenuRight, MenuLeft, MenuValid, MenuCancel, MenuBack, MenuL1, MenuR1, MenuL2,
    MenuR2, MenuStart, MenuSelect, MenuR3, MenuL3, MenuReset, MenuVali2, EndContext,
    // [port] remap: the port's map keys, after FUEL's 45 (outside DftMap's contexts). FUEL moves the map cursor with the
    // stick / d-pad once the map has the focus (the port does too); the port's keyboard also has I / K / J / L and F
    // (the map action) on the map page at any time. Remappable like FUEL's own actions.
    MapCursorUp, MapCursorDown, MapCursorLeft, MapCursorRight, MapActionKey,
    kActionCount
};
constexpr int kFuelActionCount = EndContext + 1;  // FUEL's own actions (engine order 0x8e5100)
const char* actionName(int a);  // "LEFT_ACTION", "ACCEL", ... (the engine's names)
int actionByName(const std::string& name);  // -1 when unknown (case-insensitive); "NITRO" = BOOST [nitro]
// [nitro] The port's Nitro button is FUEL's BOOST slot (16): dead in FUEL (P633 = 0). Keyboard: E, FUEL's own BOOST key
// (DftMap.map), free in-game. Pad: input.PC binds BOOST to A, which is also MENU_VALID (read while driving: the free-roam
// race-start prompt), so the port moves it to LB (L1), free in-game (InputMap::load). settings.txt key.NITRO /
// pad.NITRO rebind it. Its display name for prompts and the remap page: actionDisplayName(Boost) = "Nitro"; the other
// actions keep their engine names until they get their own.
constexpr int Nitro = 16;  // == Boost
const char* actionDisplayName(int a);

// WinInput_Z vt[40] 0x83b7e0 cases = the order of the pad-name table 0xa67438 (0x8e7730).
enum Pad : int {
    PadA, PadB, PadX, PadY, PadLB, PadRB, PadLT, PadRT, PadLStick, PadRStick, PadBack, PadStart,
    PadDLeft, PadDRight, PadDUp, PadDDown, PadLXNeg, PadLXPos, PadLYPos, PadLYNeg, PadRXNeg, PadRXPos, PadRYPos, PadRYNeg,
    kPadCount
};
int padByName(const std::string& name);  // input.PC names (A, LT, LANALOG_LEFT, LEFT, ...); -1 = NULL/unknown
const char* padInputName(int code);       // the reverse ("A", "LANALOG_LEFT"); "NULL" when out of range

// The state of a gamepad in XInput terms (what 0x83b7e0 reads): buttons, triggers 0..1, sticks -1..1 (y up).
struct PadState {
    bool connected = false;
    std::array<bool, 16> buttons{};  // indexed by Pad (PadA .. PadDDown)
    float lt = 0, rt = 0, lx = 0, ly = 0, rx = 0, ry = 0;
};
// 0x83b7e0: the 0..1 value of physical input `code` (stick half-axes: XInput s16 * 255/32767, clamped, /255).
float padValue(const PadState& p, int code);

// One action record (0x4c bytes in FUEL; the fields the game reads).
struct Record {
    float threshold = 0.5f, lo = 0.2f, hi = 0.9f;  // +0x18, +0x1c, +0x20 (0x6697d0 defaults; 0x4e1170 sets them)
    bool remap = true;                             // +2
    bool square = false;                           // +0x2c == 0
    float raw = 0, value = 0;                      // raw input, +0x30
    bool pressed = false, justPressed = false, justReleased = false;  // +0x34, +0x35, +0x36
    float heldTime = 0;                            // +0x38
};
// 0x669850 for an action without auto-repeat (+9 == 0; the menus use their own key handling in the port).
void updateRecord(Record& r, float raw, float dt);
// 0x4e1170's thresholds: 0.2 / 0.2 / 0.9 for the stick, camera and pedal actions, 0 / 0 / 0.9 for the others.
Record defaultRecord(int action);

// The player's input block (0x42e160 -> owner +0x7f4, copied to the vehicle at phys +0x1090).
struct Block {
    float stickX = 0, stickZ = 0;       // +0x190 / +0x198: steer (right +), lean / air pitch (up +)
    float camX = 0, camZ = 0;           // +0x19c / +0x1a4
    float accel = 0, brake = 0;         // e0.f3, e1.f3
    bool handbrake = false, boost = false, respawn = false, respawnHeld = false;
};

// The chase camera's look input (0x414dd0).
struct CameraInput {
    bool active = false;
    float x = 0, y = 0;
    bool viewChange = false;
};
float cameraAxis(float v);  // 0x414dd0: |v| < 0.5 -> 0, else v -/+ 0.5

// Keyboard keys use macOS virtual key codes; DirectInput scan codes (DftMap.map) are mapped onto them.
int macKeyForDik(int dik);                // -1 when the port has no equivalent
int macKeyByName(const std::string& n);   // settings names: "a".."z", "0".."9", "space", "lctrl", "num0", "up", ...
std::string macKeyName(int key);
// [port] remap: true when the port knows the key (a settings name exists, so a binding to it survives settings.txt).
bool knownMacKey(int key);
// [port] remap: the key's name as the menus show it: FUEL's key texts where it has one (TT 15608.. "Left Alt", "Space",
// "Up Arrow", ...: `text` looks them up), else the character ("=", "-", "[", ...) or the upper-case settings name.
std::string keyDisplayName(int key, const std::function<std::string(uint32_t)>& text = {});
int dikForMacKey(int key);                // PK26: the DirectInput scan code of a macOS key (-1 when none)

class InputMap {
public:
    // dataDir = the folder holding CommonComponent. settings: the "key = value" lines of settings.txt:
    //   controls = fuel (default: FUEL's keyboard map) | port (the port's earlier W/S/A/D + arrows layout)
    //   key.<ACTION> = name[, name]  (replaces the keys of an action, e.g. key.ACCEL = space, up)
    //   pad.<ACTION> = input[, input] (replaces the two pad bindings, input.PC names: pad.NITRO = A) [nitro]
    //   invert_camera_x / invert_camera_y = 0|1 (FUEL's controller page, game +0x271 / +0x2a0)
    bool load(const std::string& dataDir, const std::map<std::string, std::string>& settings, std::string* error);

    // One frame: keys = macOS key codes held, pad = the controller. Updates every action record.
    void update(float dt, const std::set<unsigned short>& keys, const PadState& pad);
    const Record& record(int a) const { return records_[size_t(a)]; }
    Block block() const;               // 0x42e160
    CameraInput camera() const;        // 0x414dd0
    // True if `key` drives an in-game action (context 0 of the keyboard map): the viewer then leaves it alone.
    bool drivesAction(unsigned short key) const;
    const std::vector<int>& keysOf(int a) const { return keys_[size_t(a)]; }
    const std::array<int, 2>& padsOf(int a) const { return pads_[size_t(a)]; }
    // [port] pad-ux: replaces an action's two pad bindings (the write path a remap page needs; the button prompts read
    // padsOf, ui/help_texts.cpp padLabels). Nothing in the port calls it outside the checks yet.
    void setPads(int a, const std::array<int, 2>& pads) {
        if (a >= 0 && a < kActionCount) pads_[size_t(a)] = pads;
    }
    // [port] remap: the write path for an action's keys (the remap page; key.<ACTION> in settings.txt).
    void setKeys(int a, const std::vector<int>& keys);
    // [port] remap: the settings lines of this map: key.<ACTION> / pad.<ACTION> for every action whose keys / pads differ
    // from `defaults` ("none" when it has none). Names as settingName.
    std::map<std::string, std::string> bindingLines(const InputMap& defaults) const;
    // The settings without the player's key.* / pad.* lines (load() of them = the layout's defaults).
    static std::map<std::string, std::string> withoutBindings(const std::map<std::string, std::string>& settings);
    static bool isBindingKey(const std::string& settingsKey);  // "key.<ACTION>" / "pad.<ACTION>" with a known action
    static std::string settingName(int a);           // ACCEL, LEFT (for LEFT_ACTION), NITRO (BOOST), MENU_VALID, MAP_ACTION ...
    static std::string keysSetting(const std::vector<int>& keys);       // "w, up" / "none"
    static std::string padsSetting(const std::array<int, 2>& pads);     // "A" / "LANALOG_LEFT, LEFT" / "none"
    // [port] remap: actions the menus cannot do without; load() gives them their default key / pad again when the
    // settings leave them none on a device (a hand-edited file, an unknown key name).
    static bool guarded(int a) { return a == MenuValid || a == MenuBack || a == Pause; }
    int fallbacks = 0;  // how many such defaults load() restored (checks)
    bool invertX = false, invertY = false;
    std::string layout = "fuel";
    std::string summary() const;  // the key and pad map, one action per line

    // Parsers (exposed for --inputcheck).
    static bool parseInputPc(const std::string& text, std::array<std::array<int, 2>, kActionCount>& pads, int* lines);
    // DftMap.map: contexts of (dev, code, dev2, code2) entries; actions = the contexts laid over the action
    // order between the NEXTCONTEXT separators (context k starts after the k-th separator).
    static bool parseDftMap(const std::vector<uint8_t>& bytes, std::vector<std::vector<std::array<int, 4>>>& ctx);

private:
    std::array<Record, kActionCount> records_;
    std::array<std::array<int, 2>, kActionCount> pads_{};
    std::array<std::vector<int>, kActionCount> keys_;
    std::set<int> drivingKeys_;
};

}  // namespace fuel::input
