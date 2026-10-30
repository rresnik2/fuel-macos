// Player settings shared by the launcher (FUEL.app, P20) and the game (fuelview
// --game, P19). Stored as "key = value" lines in settingsPath(); unknown keys are
// kept on save, missing keys take the defaults below.
#pragma once

#include <string>

namespace fuel {

struct Settings {
    std::string dataDir;        // the player's extracted game folder (holds CommonComponent)
    int width = 1600, height = 1000;
    bool fullscreen = false;
    bool postEffects = true;    // bloom, grade, blur (the O key)
    bool grass = true;
    float volume = 1.0f;        // master, 0..1
    std::string language = "en";  // LocalizedComponent text tables: en, fr, de, es, it
};

// ~/Library/Application Support/FUEL Port/settings.txt
std::string settingsPath();
Settings loadSettings(const std::string& path = settingsPath());  // defaults when missing
bool saveSettings(const Settings& s, const std::string& path = settingsPath());

}  // namespace fuel
