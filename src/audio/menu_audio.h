// The menus' audio path (fuelview --game): FUEL's MusicManager_G in its MENU context (0x4e6610: "the menu stack
// is open, or not in game" -> \MUSIC\MENU) through the same software mixer and music stream as the driving audio,
// on a small AVAudioEngine of its own. Survives a missing or busy audio device (stays silent).
#pragma once

#include <memory>
#include <string>

namespace fuel {

class MenuAudio {
public:
    // `dataDir` = the game's data root (MusicComponent, LocalizedComponent/GameTsc/Story/Levels.tsc). `output` false:
    // no audio device, the checks pull the samples with render().
    MenuAudio(const std::string& dataDir, float volume, bool output = true);
    ~MenuAudio();
    MenuAudio(const MenuAudio&) = delete;
    MenuAudio& operator=(const MenuAudio&) = delete;

    void update(float dt);           // MusicManager_G vt[17], once per frame
    void setVolume(float v);         // the music bus (setting / 16 in FUEL), 0..1
    bool playing() const;            // a MENU stream is open and playing
    std::string track() const;       // the file the manager opened ("MENU")
    void render(float* L, float* R, int n);  // 44.1 kHz stereo, as the audio engine would pull it

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// PK18: the menu sound effects (Front vf00 0x4ecd40: 12 WAVs "DB:>SOUNDS>SFX>MENU>" BACK CANCEL CHANGELETTER CONFIRM INVALID
// MOVEBACKWARD MOVEFORWARD NAVIGATIONOPTIONS POPUPCLOSED POPUPOPEN START GPS = indices 0..11, the Sound_Z resources of
// CommonComponent/DATAS/SHARED.DPC) played one at a time per frame by the pause HUD (0x46d380 -> 0x55a1e0, SoundManager vt+0x9c:
// not positional, no distance gain, the SFX bus volume). Own small AVAudioEngine; survives a missing or busy audio device.
class MenuSfx {
public:
    static constexpr int kCount = 12;
    static const char* name(int index);  // "BACK", ...
    // `volume` = the SFX bus (the Sound Volume option / 16); `output` false: no audio device (the checks pull render()).
    MenuSfx(const std::string& dataDir, float volume, bool output = true);
    ~MenuSfx();
    MenuSfx(const MenuSfx&) = delete;
    MenuSfx& operator=(const MenuSfx&) = delete;
    bool play(int index);              // false: out of range, not loaded, no voice free
    bool loaded(int index) const;
    void setVolume(float v);
    void update();                     // once per frame (the mixer's gain pass)
    int voices() const;
    void render(float* L, float* R, int n);  // 44.1 kHz stereo, as the audio engine would pull it

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// Headless --menu-check part: the 12 menu SFX decode, play and render audibly. Returns failures.
int menuSfxCheck(const std::string& dataDir);

// Headless --menu-check part: the MENU track renders (non-silent, loops past its length). Returns failures.
int menuAudioCheck(const std::string& dataDir);

}  // namespace fuel
