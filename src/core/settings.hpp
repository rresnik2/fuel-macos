// Player settings shared by the launcher (FUEL.app, P20) and the game (fuelview
// --game, P19). Stored as "key = value" lines in settingsPath(); unknown keys are
// kept on save, missing keys take the defaults below.
#pragma once

#include <map>
#include <string>

namespace fuel {

// P103: FUEL's volume options are 17-step sliders n = 0..16 applied linearly as n / 16 (SFX 0x5540e0 / 0x554090 ->
// SoundManager vt+0x40 = bus +0x20ec; music 0x554040 -> vt+0x50 = bus +0x20e4; voice -> vt+0x48 = bus +0x20f4) and the
// factory value of all three is n = 8 (0x553c80(1), called by the game init 0x556230 and 0x55c760 before a save is
// read): half gain, -6.02 dB. The port's `volume` (SFX) and `music_volume` keys keep FUEL's meaning (slider n / 16).
constexpr float kFuelDefaultVolume = 8.0f / 16.0f;

struct Settings {
    std::string dataDir;        // the player's extracted game folder (holds CommonComponent)
    int width = 1600, height = 1000;
    bool fullscreen = false;
    bool postEffects = true;    // bloom, grade, blur (the O key)
    bool grass = true;
    float volume = kFuelDefaultVolume;  // the SFX volume slider n / 16 (FUEL's bus +0x20ec), 0..1; FUEL's default n = 8
    std::string language = "en";  // LocalizedComponent text tables: en, fr, de, es, it
    // Every "key = value" line as read (P47: controls, key.<ACTION>, invert_camera_x/y are read from here by
    // core/input_map.*). Not written back by saveSettings: unknown lines stay in the file as they are.
    std::map<std::string, std::string> all;
};

// ~/Library/Application Support/FUEL Port/settings.txt ($FUEL_SETTINGS overrides: checks use a scratch copy)
std::string settingsPath();
Settings loadSettings(const std::string& path = settingsPath());  // defaults when missing
bool saveSettings(const Settings& s, const std::string& path = settingsPath());

// ---- PK26: FUEL's language index (GetUserLanguageIndex 0x81e060 maps the Windows UserLanguageID to it: 0x409 -> 1,
// 0x809 -> 10, 0x40c -> 2, 0x40a -> 3, 0x407 -> 4, 0x410 -> 5, 0xc0c -> 0x13, ...; default 1). [port] from settings.txt
// `language` (en, en-uk, fr, es, de, it), the codes the port's text tables use; unknown -> 1.
int languageIndex(const std::string& language);
// SetLanguageAuto 0x54c900 / 0x553f40 / 0x55af00: miles (game+0x330c, +0x33f5) for the languages 1, 10, 0x13, 0x14.
bool languageMiles(int index);
// ---- end PK26

}  // namespace fuel
