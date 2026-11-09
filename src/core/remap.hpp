// [port] remap (the user's request 2026-11-06: custom controller and keyboard mapping). FUEL's own remap page and rules,
// decoded from FUEL.exe (notes ~/re-work/fuel/notes/remap.md), with the port's additions marked:
//  - the list (0x4cc650): the actions of DftMap's three contexts in engine order, named by the text table from TT 15701
//    ("Steer left" ...) with "END OF ACTION" (15718 / 15728 / 15747) closing a context and "END OF CONTEXT" (15748) the
//    list; the context titles are TT 15700 "General", 15719 "Map", 15729 "Menu". FUEL skips TT 0x3d65 (15717 "Boost",
//    dead in the shipped game); the port lists it as "Nitro" [nitro]. [port] FUEL's four "Map Debug" actions (L_FRONT ..
//    R_BACK, its debug cheat HUD) are not listed; the port's map keys (MAP_CURSOR_*, MAP_ACTION) are, in the Map context.
//  - a capture (0x4cd080) replaces the selected action's binding (FUEL writes the whole 12-byte DftMap entry: the
//    secondary key is cleared). When another action of the SAME context already has the captured input, the two entries
//    swap whole: that action gets the selected one's old binding (both its keys) [O]; [port] when several actions of the
//    group share the input (input.PC: MENU_RESET and MENU_VALI2 on X) all of them take it. [port] contexts 1 and 2 (Map, Menu) and the port's map keys
//    form one group (all read while a menu is open); the driving context is the other.
//  - [port] pads: FUEL PC remaps the keyboard only (with an XInput pad it shows the pad's picture page, state 0xad); the
//    port remaps the pad's two input.PC slots the same way.
//  - [port] MENU_VALID, MENU_BACK and PAUSE_MODE never end up without a binding on a device: a swap that would leave one
//    of them empty is refused (FUEL's TT 15560 "This key is already used for another action.").
#pragma once

#include "core/input_map.hpp"

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace fuel::input::remap {

enum Device { Keyboard = 0, Pad = 1 };

// The page's rows in FUEL's order: context titles, then the actions.
struct Row {
    enum Kind { Title, Action, Reset } kind = Action;
    int action = -1;  // Action rows
    uint32_t tt = 0;  // the name's text (Title / Action); 0 = port text in `name`
    std::string name; // port names (Nitro, the map keys, Reset to defaults)
};
std::vector<Row> rows();
// The text of FUEL's name of an action on the page (TT 15701 ..), 0 when it has none (the port's actions).
uint32_t actionText(int action);
const char* portName(int action);  // the port's own names ("Nitro", "Map cursor up", ...), nullptr otherwise

// Conflict group: 0 = driving (DftMap context 0), 1 = menus (contexts 1, 2 and the port's map keys); -1 = not listed.
int group(int action);
bool listed(int action);

struct Result {
    bool changed = false;
    int swapped = -1;     // the other action of the swap, -1 when none
    std::vector<int> others;  // every action that took the old binding (input.PC shares pad X by MENU_RESET and MENU_VALI2)
    bool refused = false; // the swap would leave a guarded action unbound on this device
};
// Binds `code` (a macOS key code or a Pad input) to `action` on `device` (FUEL's 0x4cd080 rule above).
Result rebind(InputMap& m, int action, Device device, int code);
// The device's bindings back to `defaults` (FUEL's MENU_RESET on its page, 0x4cc870: the default map again).
void resetDevice(InputMap& m, const InputMap& defaults, Device device);

// Keyboard capture: Escape cancels (FUEL TT 15545 "Press ESCAPE to cancel."); keys the port has no name for are not taken.
bool captureCancels(int macKey);
bool capturable(int macKey);

// Pad capture: the first pad input pressed (buttons, triggers > 0.5, stick half-axes > 0.6) once every input was released
// (the button that started the capture is still down). Returns the Pad code, or -1.
struct PadCapture {
    bool armed = false;
    int step(const PadState& p);
};

}  // namespace fuel::input::remap
