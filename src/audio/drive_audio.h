// Driving sounds for fuelview --drive: FUEL's own vehicle sound model (audio/vehicle_sound.*) for the
// player and the opponents, mixed like FUEL's DirectSound layer (audio/sound_mixer.*) and played
// through one AVAudioEngine source node. Survives a missing or busy audio device (stays silent).
#pragma once

#include "audio/ambience.hpp"
#include "audio/music.hpp"
#include "core/dpc.hpp"
#include "physics/vehicle_physics.hpp"

#include <array>
#include <functional>
#include <map>
#include <memory>
#include <string>
#include <utility>
#include <vector>

namespace fuel {
namespace vib { class Manager; }

// What the music (MusicManager_G) and the ambience (Game_SoundMgr 0x4e81e0) read from the game each frame.
struct GameSound {
    snd::MusicGame music;
    snd::AmbientInput ambient;
    int countdown = -1;  // hud::countdown() of the start countdown (MissionHUD 0x4b3da0), -1 = none
};

class DriveAudio {
public:
    DriveAudio();
    ~DriveAudio();
    DriveAudio(const DriveAudio&) = delete;
    DriveAudio& operator=(const DriveAudio&) = delete;

    // The player's vehicle (decodes its bank; not for the frame loop). Prints a message and stays
    // silent if the sounds cannot be built.
    void setVehicle(const DpcFile& dpc, const std::string& variant, const std::vector<float>& params);
    // P89: the pad vibration manager the player's sounds ask (CLUTCH, IMPACT_*, ABSORBER_n); null = none.
    void setVibration(vib::Manager* v);
    // Opponent `k` (engine only, at 0.8, attenuated with distance as FUEL does).
    void setRival(size_t k, const DpcFile& dpc, const std::string& variant, const std::vector<float>& params);
    // The camera (world): FUEL attenuates and pans every sound from it. `view` = FUEL's camera view
    // (5, 4 or 1): the category mix follows it (0x402930).
    void setListener(const std::array<float, 3>& eye, const std::array<float, 3>& target, int view);
    // Once per frame, after the rivals: the player's sounds, the game mix and the voices.
    void update(float dt, const VehicleState& state, float throttle);
    void updateRival(size_t k, float dt, const VehicleState& state);
    // PK23: the player's backfire flash +0x2ebc (0x5d9d60 sets it on a pop, decays 2/s); -1 without the player's sounds.
    float playerFlash() const;
    // PK13: 0x539a50 (TR_DEATH's 0x10e hold): the vehicles' sound update 0x5d8ed0 is skipped (player, AI, traffic);
    // their voices keep playing with their last settings. update() then mixes only the rest (ambience, music, RTC).
    void setVehicleSoundsHeld(bool held) { vehicleSoundsHeld_ = held; }
    // ---- PK17: the pause menu. open (InGameMenuHUD vf07 0x46aca0): every vehicle's sound slots off (0x5d8e60 ->
    // 0x5de1c0: level and target 0, voice stopped; the player, the opponents) and WinSound's pause mask 7 (SFX, voice
    // bus, dialogs; the music plays on). close (vf08 0x46ade0): mask 0 (paused voices resume where they were); the
    // vehicles' slots come back through their normal update 0x5d8ed0 and fade in from 0 at their own rates.
    void pauseMenu(bool open);
    uint32_t pauseMask() const;
    // ---- end PK17
    // The world (USA1): the IT_SOUND emitter bank for the ambience and the music library next to FUEL.exe
    // (MusicComponent, Levels.tsc AddMUSIC). Without it the ambience and music stay silent.
    void setWorld(const DpcFile& world);
    // Once per frame before update(): the game state the music and ambience follow.
    void setGame(const GameSound& g);
    // The race started or ended (FUEL's inlined 0x4e6b50): the music fades out and is chosen again.
    void reselectMusic();
    // Teleport_Queue 0x57e650: the event one-shot 0xfa. Call when the session teleports the player (hub jump, map).
    void teleportQueued();
    // The effects port's lightning flash hook (Ambience::onLightning): called with the bolt's position when the
    // thunder timer creates a bolt (cloud >= 0.95).
    void setLightningHook(std::function<void(const std::array<float, 3>&)> f);
    // P96: headless: never opens an audio device; renderOffline(dt) mixes dt seconds into a buffer that the destructor
    // writes to `wavPath` (16-bit stereo 44.1 kHz WAV), with per-second levels on stdout (FUEL_AUDIO_DUMP in --film).
    void setOffline(const std::string& wavPath);
    void renderOffline(float dt);
    int activeVoices() const;
    void toggleMute();
    bool muted() const;
    void setMasterVolume(float v);  // the SFX volume (SoundManager +0x20ec), 0..1
    void setMusicVolume(float v);   // P101: the music volume (0x554040, game +0x254 / 16), 0..1; default 1
    // P67: the screen fader's sound slot (0x554200 slot 1 = 1 - fade value), multiplies the master volume.
    void setFadeGain(float g);
    // P67: a cut-scene sample (RTC 0x24 / 0x25): positional (flags 0xf at a node, distance gain 6 / 140 m) or 2D.
    // P96: `scale` = the node's world scale: the distances are 6 / 140 m x scale (0x733080; TELEPORT_HELICO's A1 is x3).
    int playSample(std::shared_ptr<const snd::Pcm> pcm, bool positional, const std::array<double, 3>& pos, float volume, float scale = 1);
    void moveSample(int handle, const std::array<double, 3>& pos, float scale = 1);
    void stopSample(int handle);
    // P61: an object's own bank sound (0x636ee0 with flags 0xf: positional, distance gain), one-shot: prop impacts
    // (symbols 0x83 / 0x84 / 0x86 of the prop's Lod_Z bank, by its name CRC) with FUEL's pitch and distances.
    void objectSound(uint32_t lodName, int symbol, const std::array<double, 3>& pos, float pitch, float volume, float minDist, float maxDist);
    // A looping object sound kept alive while `on` (the tornadoes: Tornade 0x7e / 0x80, SmallTornadoAgent_G 0x7f /
    // 0x81, MotherTornadoManipulator_G vf17 0x4d55d0 / SmallTornadoManipulator_G vf18 0x5bf5f0).
    void objectLoop(uint64_t key, uint32_t lodName, int symbol, const std::array<double, 3>& pos, bool on, float minDist, float maxDist);
    // ---- PK27: the F14 fly-by's emitter (core/f14_flyby): `pass` = the pass serial (a new one starts the loop once),
    // `on` = the pass flies, `pos` = the F14 node, `pitch` = 0x4ea560's doppler.
    void flyby(int pass, bool on, const std::array<double, 3>& pos, float pitch);
    // The saved voice level (game +0x25c, Career::voiceLevel) applied as 0x554130 does: voice bus = n / 16 (vt+0x48).
    void setVoiceVolume(float v);
    // ---- PK27 end

private:
    bool vehicleSoundsHeld_ = false;  // PK13
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

// Headless: FUEL's sound maths against decoded / emulated vectors, the listed vehicles' banks, and an
// offline render of a short drive (no audio device). Returns the number of failures.
int audioCheck(const std::vector<std::pair<const DpcFile*, std::string>>& variants, const std::map<std::string, std::vector<float>>& params);
// Headless: the player's sounds over recorded physics states (1/60 s apart), rendered offline; prints
// the backfires and levels per second. Returns the number of failures.
// Headless: music (cue reader, manager, intensity rules vs emulated vectors) and ambience (beds, animals,
// emitters) on synthetic inputs, rendered offline. Returns the number of failures.
int musicAmbienceCheck(const DpcFile* world);
int audioDriveCheck(const DpcFile& dpc, const std::string& variant, const std::vector<float>& params, const std::vector<VehicleState>& states);
// P89 --haptics-check: FUEL's vibration rules (emulated vectors), then the pad motors FUEL's rules produce over
// a recorded drive on the port's physics plus a synthetic impact series (impact vector + crash meter).
int hapticsDriveCheck(const DpcFile& dpc, const std::string& variant, const std::vector<float>& params, const std::vector<VehicleState>& states,
                      const std::string& dataDir);

}  // namespace fuel
