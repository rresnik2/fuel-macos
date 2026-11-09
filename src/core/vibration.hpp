// FUEL's pad vibration (P89): VibrationManager_C (vtable 0x9b..; vf17 0x4e4110 update, add 0x4e4530,
// stop 0x4e4060) driven by the shapes of CommonComponent/Vibrations.tsc (ADDVibration 0x4e56f0,
// AddLowVibrationKey 0x4e5770, AddHighVibrationKey 0x4e57b0). Decode notes: notes/p89-pad-vibration.md.
//
// A vibration is two key curves (time s, level 0..100 stored x 2.55): HIGH -> the right, high-frequency
// motor and LOW -> the left, low-frequency motor (WinInput_Z::SetVibration 0x83b2e0 sends arg 2 as
// wRightMotorSpeed and arg 3 as wLeftMotorSpeed, each byte x 0x101). Curves are linear (Curve_Eval8
// 0x72f9e0, flags 2), keys kept in file order; a time outside the walked span takes the last key's level.
// A playing instance lasts max(last HIGH key time, last LOW key time) (0x4e4a80) and ends when its
// time reaches that. Every frame the instances of a pad add level x scale per motor; each motor is
// clamped to [0, 255] and truncated (x87 RC = chop) to the byte sent.
//
// The game asks for a vibration through Game 0x557b40 (needs "in game" +0xdc and the vibration option
// +0x272): IMPACT_SMALL / IMPACT_MEDIUM / IMPACT_HIGH and CLUTCH (vehicle sounds, priority 1..3, scale
// 1), ABSORBER_n (priority 1, scale clamp((|rate| - P668) P669, 0, 1) / wheels), STARTRTCFX SHAKEPAD
// (unused in shipped data) and the options page test (IMPACT_MEDIUM, priority 2). The data is the
// player's own; nothing of it is compiled in.
#pragma once

#include <cstdint>
#include <functional>
#include <string>
#include <unordered_map>
#include <vector>

namespace fuel::vib {

struct Key {
    float t = 0, v = 0;  // v already x 2.55 (0..255)
};
struct Shape {
    std::string name;
    std::vector<Key> high, low;  // +0x8 (AddHighVibrationKey), +0x14 (AddLowVibrationKey)
    bool loop = false;           // +0x1c (never set by the shipped scripts)
    float duration() const;      // 0x4e4a80
};

// Curve_Eval8 0x72f9e0 from key 1 (linear).
float evalCurve(const std::vector<Key>& keys, float t);

struct Shapes {
    std::vector<Shape> list;
    std::unordered_map<uint32_t, int> byCrc;  // asoboCrc(name) -> index (0x4e4c10)
    std::string error;
    int find(const std::string& name) const;
    // CommonComponent/Vibrations.tsc of the player's data (#ifdef _PC; _PS3 blocks skipped).
    static Shapes load(const std::string& dataDir);
    static Shapes parse(const std::string& text);
};

// The two motor levels FUEL sends for a pad (0..255 each).
struct Motors {
    int low = 0, high = 0;          // left (low-frequency) and right (high-frequency) motor bytes
    float lowF = 0, highF = 0;      // the clamped levels before truncation
    bool operator==(const Motors& o) const { return low == o.low && high == o.high; }
};

class Manager {
public:
    explicit Manager(const Shapes* shapes = nullptr) : shapes_(shapes) {}
    void setShapes(const Shapes* s) { shapes_ = s; }
    // g_Game +0x272 (the Controller settings page's VIBRATION ON/OFF; 0x495c40 from the profile).
    bool option = true;
    bool gameStarted = true;  // PK26: GameManager +0xdc (GAMEStarted after the career boot's teleport), 0x557b40 needs it
    // 0x557b40 -> 0x4e4530: starts `name` on the player's pad unless the option is off, the manager is
    // off (+0x3c), the stop cooldown (+0x40) is above 0.1 s or the same vibration plays at the same or a
    // higher priority (a lower one is replaced and restarts). Returns whether it started.
    bool add(const std::string& name, int priority, float scale);
    bool add(uint32_t crc, int priority, float scale);
    // vf17 0x4e4110: advances the instances by dt (ends those past their length), returns the pad's motors.
    Motors update(float dt);
    // 0x4e4060: stops every instance and sets the cooldown +0x40 to 1 s (no new vibration until it falls
    // to 0.1 s, i.e. 0.9 s of updates).
    void stopAll();
    // 0x557d80 (resume, options on) / 0x557c90 (pause, options off: stopAll twice, no updates until resumed).
    void setActive(bool on);
    bool active() const { return active_; }
    float cooldown() const { return cooldown_; }
    void clearCooldown() { cooldown_ = 0; }  // P101: 0x495670 writes +0x40 = 0 before its test pulse
    size_t playing() const { return inst_.size(); }
    // Checks: every request (name CRC, priority, scale, started).
    std::function<void(uint32_t, int, float, bool)> trace;
    // Checks: an instance at a given time (as 0x4e4990 builds it, then time set).
    void inject(int shape, int priority, float scale, float time, int loop = -1);  // loop -1: the shape's

private:
    struct Instance {
        int shape = -1;
        float time = 0, duration = 0, scale = 1;
        int priority = 0;
        bool loop = false;
    };
    const Shapes* shapes_;
    std::vector<Instance> inst_;
    bool start(uint32_t crc, int priority, float scale);
    float cooldown_ = 0;  // +0x40
    bool active_ = true;  // +0x3c
};

// Headless: Vibrations.tsc, the emulated vf17 vectors (FUEL.exe under EmuRun, notes/p89-work) and the
// manager rules. Returns the number of failures.
int vibrationRulesCheck(const std::string& dataDir);

// [DEVIATION nitro, the user's request 2026-11-06; physics/nitro.hpp] The boost's rumble, added to FUEL's motors by max
// (PadHaptics): a 0.25 s pulse as a boost kicks in (left 0.55 / right 0.35 of full, fading out), then a light buzz while
// it lasts (left 0.12 / right 0.22 x the amount). `on` false (vibration off, paused, a menu): nothing.
struct NitroRumble {
    float prev = 0, pulse = 0;
    Motors step(float dt, float amount, bool on);
    void reset() { prev = pulse = 0; }
};
// Per motor max of FUEL's motors and the port's additions (drive rumble, nitro).
Motors maxCombine(const Motors& a, const Motors& b);

}  // namespace fuel::vib
