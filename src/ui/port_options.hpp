// P101: the settings.txt keys the port reads beside fuel::Settings (renderer, audio, controller), as the in-game
// options pages edit them. One place for each key's name, its accepted spellings, its default and the spelling the
// game writes back, so the menus, the live apply (viewer/game_settings.*) and --menu-check agree with the readers:
//   dynamic_resolution          on | off                 (default on;  viewer/game_settings.mm, P93)
//   dynamic_resolution_min      0.5 .. 1                 (default 0.8)
//   dynamic_resolution_upscaler sharpen | metalfx | metalfx-temporal  (default sharpen; TAA: metalfx-temporal)
//   shadow_filter               2x2 | 3x3                (default 2x2)
//   wind_debris                 smooth | fuel            (default smooth; P90)
//   audio_limiter               on | off                 (default on;  P100's mixer limiter)
//   vibration                   1 | 0                    (default 1;   FUEL's VIBRATION, game +0x272, P89)
//   invert_camera_x / _y        1 | 0                    (default 0;   FUEL's camera axes, game +0x271 / +0x2a0, P47)
//   controls                    fuel | port              (default fuel; core/input_map.*)
//   adaptive_triggers           off | light | full       (default full; P91)
//   adaptive_triggers_extras    none | all | a list      (default none; a hand-written list is kept as a third value)
//   film_grain                  off | light | strong     (default light; P102's grain, read by its renderer code)
//   soften                      off | light | strong     (default off; P102's pre-tonemap tent)
//   rider_jiggle                off | subtle | full      (default subtle; PHYSIQUE: the player's custom rider's glutes)
//   pad_glyphs                  auto | playstation | xbox (default auto; [port] pad-ux: the button prompts' family)
//   nitrous                     on | off                 (default on;  the port's nitrous, physics/nitro.hpp)
//   upgrades                    on | off                 (default on;  [upgrades U2] vehicle upgrades, core/upgrades.hpp:
//                                                         off = every session stock, the career's stages kept)
//   dev_mode                    on | off                 (default off; [DEVIATION dev mode] every vehicle and upgrade stage
//                                                         owned as an overlay: core/career.hpp Career::setDevMode)
// Saving rewrites the lines of these keys in place, appends a key only when its value is not the default, and keeps
// every other line (unknown keys, comments) as it was.
#pragma once

#include <map>
#include <string>

namespace fuel::ui {

struct PortOptions {
    bool dynres = true;
    float dynresMin = 0.8f;
    int upscaler = 0;              // 0 sharpen, 1 metalfx (spatial), 2 metalfx-temporal (TAA)
    bool shadow3x3 = false;
    bool debrisFuel = false;
    bool limiter = true;
    bool vibration = true;
    bool invertX = false, invertY = false;
    bool portControls = false;
    int triggers = 2;              // 0 off, 1 light, 2 full
    std::string extras = "none";   // none | all | the file's own list
    int filmGrain = 1;             // 0 off, 1 light, 2 strong
    int soften = 0;                // 0 off, 1 light, 2 strong (P102 `soften`, default off)
    int jiggle = 1;                // PHYSIQUE: 0 off, 1 subtle, 2 full (`rider_jiggle`, default subtle)
    int padGlyphs = 0;             // 0 auto, 1 playstation, 2 xbox (pad-ux `pad_glyphs`; = pad::Style)
    bool nitrous = true;           // [nitro] `nitrous` (the user's request 2026-11-06), CONTROLS > SETTINGS row 8
    bool upgrades = true;          // [upgrades U2] `upgrades` (the user's request 2026-11-06; U3: CONTROLS > SETTINGS "Upgrades")
    bool devMode = false;          // [dev mode] `dev_mode` (the user's request 2026-11-07; CONTROLS > SETTINGS "Dev mode", the last row)

    static PortOptions from(const std::map<std::string, std::string>& all);
    // The keys as the game writes them (into a Settings::all copy for the live apply).
    void store(std::map<std::string, std::string>& all) const;
    bool operator==(const PortOptions& o) const;
    bool operator!=(const PortOptions& o) const { return !(*this == o); }
};

// Rewrites `path` with these keys (see above). False when the file cannot be written.
bool savePortOptions(const PortOptions& o, const std::string& path);

// [port] remap: the bindings the remap page saves, `lines` = key.<ACTION> / pad.<ACTION> -> value (core/input_map.hpp
// InputMap::bindingLines: only the actions that differ from the layout's defaults). Every key.* / pad.* line of a known
// action is replaced (the first one, in place) or removed when the action is back to its default; new ones are appended;
// comments, unknown keys and every other line stay as they were.
bool saveBindingLines(const std::map<std::string, std::string>& lines, const std::string& path);

}  // namespace fuel::ui
