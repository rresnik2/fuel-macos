// The game's front end for `fuelview --game`, the in-game pause/results overlay and
// HUD, and the headless --menu-shot / --menu-check. A session (race or free ride) runs
// in a fresh fuelview process: the menu exec()s `fuelview --in-game ...`, and the
// in-game menu exec()s back to `fuelview --game` (or into the next session).
#pragma once

#import <Metal/Metal.h>

#include "core/settings.hpp"
#include "ui/menu.h"

#include <memory>
#include <string>
#include <vector>

namespace fuel::ui {

class UiRenderer;

// fuelview --menu-shot PAGE OUT.png [--size WxH]
int runMenuShot(const std::string& dataDir, const Settings& s, const std::string& page, const std::string& out, int w, int h);

// fuelview --game: title screen and menus in a window until the player starts a session
// (then this process is replaced) or quits.
int runGameMenu(const std::string& self, const std::string& dataDir, const Settings& s, const std::string& startPage);

// Replaces this process with the session `r` asks for (never returns unless exec fails).
void execSession(const std::string& self, const std::string& dataDir, const MenuRequest& r, const Menu& menu);
// Replaces this process with the menu (`--game`, optionally opening `page`).
void execMenu(const std::string& self, const std::string& dataDir, const std::string& page = "");

// The pause / results menu and the HUD inside a session (`--in-game`).
class GameOverlay {
public:
    GameOverlay(id<MTLDevice> device, MTLPixelFormat format, const std::string& self, const std::string& dataDir, const Settings& s,
                const std::string& event, const std::string& hub, const std::string& vehicle);
    ~GameOverlay();

    bool open() const { return open_; }
    void openPause();
    void openResults(const std::vector<ResultRow>& rows);
    // Keyboard (macOS virtual key codes); true when the overlay used the key.
    bool keyDown(unsigned short code);
    void pointer(float x, float y, bool click, float viewW, float viewH);
    void pollGamepad();
    // Requests the host must act on in-process; others exec() away.
    enum Action { None, Resume, Restart };
    Action update(float dt);
    void draw(id<MTLCommandBuffer> cb, id<MTLTexture> target, const HudState& hud);
    Menu& menu() { return *menu_; }

private:
    std::string self_, dataDir_;
    std::unique_ptr<Menu> menu_;
    std::unique_ptr<UiRenderer> ui_;
    bool open_ = false;
    unsigned padButtons_ = 0;
};

// macOS key code -> menu key (arrows, Return, Escape, Q/E, page up/down); false if none.
bool menuKeyFor(unsigned short code, MenuKey* k);
// Newly pressed gamepad buttons -> menu keys.
std::vector<MenuKey> gamepadMenuKeys(unsigned* lastButtons);

}  // namespace fuel::ui
