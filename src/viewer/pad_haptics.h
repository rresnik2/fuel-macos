// P89: FUEL's pad vibration on a Mac controller (DualSense, DualShock 4, Xbox) through GameController
// haptics. FUEL's rules (core/vibration.*: VibrationManager_C with the player's Vibrations.tsc) give two
// motor bytes per frame, LOW for XInput's left (low-frequency) motor and HIGH for the right
// (high-frequency) one; here they drive a continuous haptic event on the controller's left and right
// handles (GCHapticsLocalityLeftHandle / RightHandle), or one default-locality engine when the
// controller has no handles (intensity = the stronger motor). The controller is the one the game reads
// (GCController.controllers.firstObject); it may connect, disconnect or change at any time.
//
// settings.txt: `vibration = 0` turns it off (FUEL's Controller settings page, game +0x272; default on).
// The file is only read; P101: the in-game Controller page switches it live (setOption / testPulse).
#pragma once

#include "core/settings.hpp"
#include "core/vibration.hpp"

#include <memory>
#include <string>

namespace fuel {

class PadHaptics {
public:
    PadHaptics(const std::string& dataDir, const Settings& settings);
    ~PadHaptics();
    PadHaptics(const PadHaptics&) = delete;
    PadHaptics& operator=(const PadHaptics&) = delete;

    // What the player's vehicle sounds ask (DriveAudio::setVibration).
    vib::Manager* manager() { return &manager_; }
    // Once per driving frame, after the sounds: advances FUEL's vibrations and sends the motors.
    // `cutscene`: a cut-scene started this frame (0x53b860 stops every vibration).
    void frame(float dt, bool cutsceneStarted);
    // The pause menu opened / closed (0x553c80 -> 0x557c90 / 0x557d80).
    void setPaused(bool paused);
    // P101: each frame while the pause menu is up: the first call pauses (0x557c90); after that only what the options
    // page started (the test pulse) runs, as FUEL's manager is switched back on by 0x495670 inside the menu.
    void menuFrame(float dt);
    // P101: the VIBRATION option changed (0x495670 / 0x495c40): off -> 0x557c90 (manager off, stopped); on -> 0x557d80.
    void setOption(bool on);
    // P101: 0x495670 switching ON: manager on, cooldown +0x40 = 0, IMPACT_MEDIUM at priority 2, scale 1 on the pad.
    void testPulse();
    bool option() const { return manager_.option; }
    // [DEVIATION, the user's request 2026-11-05]: a continuous drive rumble FUEL does not have (its Vibrations.tsc
    // only has impacts): the ground's texture on the low motor (FUEL's per-ground BUMPS, pp001 PP 4000..), the tyres
    // sliding / spinning / locking on the high motor. Combined with FUEL's motors by max. settings.txt
    // `drive_rumble = off | light | full` (default full); `vibration = 0` turns it off too. Call before frame().
    struct DriveFeel {
        bool active = false;   // driving (not paused, no menu / cut-scene / loading)
        bool airborne = false;
        float speed = 0;       // m/s
        float rough = 0;       // mean of max(bumpsWheel, bumpsRot) under the loaded wheels (0 tarmac .. 0.135 rock)
        float slip = 0;        // 0..1: tyres past their grip (sliding, spinning or locked)
        float nitro = 0;       // [nitro] the boost amount 0..1 (physics/nitro.hpp)
    };
    void drive(float dt, const DriveFeel& f);
    // Motors off and the engines released (window closing).
    void shutdown();
    const vib::Motors& motors() const { return motors_; }

private:
    vib::Shapes shapes_;
    vib::Manager manager_;
    vib::Motors motors_;
    bool inMenu_ = false;  // P101
    float driveScale_ = 1.0f;          // drive_rumble: 0 off, 0.6 light, 1 full
    float texS_ = 0, slipS_ = 0;       // smoothed drive rumble levels 0..1
    float grainT_ = 0, grain_ = 1;     // the texture's grain (a new random level every ~0.6 m)
    uint32_t grainSeed_ = 0x9e3779b9u;
    int driveLow_ = 0, driveHigh_ = 0; // bytes added (max) to FUEL's motors
    // [nitro] the boost's rumble: a pulse as a boost kicks in, a light buzz while it lasts (max-combined like the above;
    // `vibration = 0` turns it off, `drive_rumble` does not: it is its own feedback).
    vib::NitroRumble nitro_;
    vib::Motors nitroMotors_;
    struct Device;
    std::unique_ptr<Device> dev_;
    void send(const vib::Motors& m);
};

}  // namespace fuel
