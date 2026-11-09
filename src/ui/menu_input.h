// [port] remap: the menus read the bindings (core/input_map.*), not fixed keys and buttons. FUEL's menu keys are actions
// [O]: Up = MENU_UP or PAD_UP (0x4c9450), Down = MENU_DOWN or PAD_DOWN (0x4c94e0), Left = MENU_LEFT or PAD_LEFT
// (0x4c9330), Right = MENU_RIGHT or PAD_RIGHT (0x4c93c0), Accept = MENU_VALID (0x4c9220; it also reads MENU_START),
// Back = MENU_BACK or MENU_SELECT (0x4c9190), MENU_VALI2 (0x4c92b0), the tabs MENU_L1 / MENU_R1, the map's GPS key
// MENU_CANCEL (game +0x20c, 0x484580) and zoom MENU_L2 / MENU_R2, the remap page's reset MENU_RESET (game +0x234,
// 0x4cc870). [port] MENU_START is Back (the port's pause key closes the pause menu, as it did before), not Accept; the
// port's map keys (MAP_CURSOR_*, MAP_ACTION) give the map page's cursor and action keys.
// Fallback [port]: Return / Enter and Escape on the keyboard, A and B on the pad, when bound to no menu action at all,
// keep their old meaning (Accept / Back), so a menu stays usable whatever the bindings.
#pragma once

#include "core/input_map.hpp"
#include "ui/menu.h"

#include <array>
#include <vector>

namespace fuel::ui {

// The menu keys a keyboard key gives (several when it is bound to several menu actions; deduplicated). `remapPage`: the
// remap page is up (MENU_RESET is read there only, and there it wins over the other keys of the same key: 0x4cc870
// tests the reset before Back).
std::vector<MenuKey> menuKeysForKey(const fuel::input::InputMap& m, unsigned short macKey, bool remapPage);

// The pad side: edges of the routed actions' pad inputs (stick half-axes past 0.6, buttons and triggers past 0.5, as the
// fixed reader did). `released`: the menu keys whose inputs went up this frame (the map's held keys).
class PadMenuRouter {
public:
    std::vector<MenuKey> step(const fuel::input::InputMap& m, const fuel::input::PadState& p, bool remapPage,
                              std::vector<MenuKey>* released = nullptr);
    void clear() { held_.fill(false); }

private:
    std::array<bool, 32> held_{};
};

// The first GameController pad as XInput would read it (main.mm's readPad: WinInput_Z 0x83b7e0's inputs).
fuel::input::PadState readControllerPad();

// The checks (--menu-check): routing with the default and remapped bindings, the fallbacks.
int runMenuInputCheck(const std::string& dataDir);

}  // namespace fuel::ui
