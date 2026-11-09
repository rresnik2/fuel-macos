// P91: DualSense adaptive triggers. An addition beyond FUEL (FUEL.exe, 2009, has no trigger effects): it only reads
// the game (the player's FUEL vehicle model and the session state) and never changes the physics or the input FUEL
// sees. Notes: ~/re-work/fuel/notes/p91-adaptive-triggers.md.
//
// Two parts:
//  - trig::Model, the feel: a deterministic function of the telemetry stream (with light smoothing and hysteresis
//    state) that picks a GCDualSenseAdaptiveTrigger mode and its parameters for each trigger:
//      progressive resistance: slope feedback, the throttle gentle -> firmer, the brake soft top -> stiff bottom,
//        heavier for trucks, lighter for bikes;
//      grip feel: "grip left" 0..1 per trigger from FUEL's own grip utilisation of the wheel records (sumFn/sumDen,
//        0x523c50: >= 1 is FUEL's "sliding", 0x524240) on the driven wheels (throttle) and the braked wheels
//        (brake), with the rim against the ground speed for wheelspin and lock-up; the surface's static friction
//        (the ground block of the 18 ground classes, pp001 PP 4000.., 0x5c6890) sets the overall ceiling.
//    It is off whenever the game is paused, in a menu, a cut-scene or loading.
//  - PadTriggers, the device: GCController.controllers.firstObject (the pad the game reads) when it is a
//    GCDualSenseGamepad; nothing for any other pad. Sends at most 60 times a second over USB (only when a value
//    changed), about 10 times a second in 5 grip bands with hysteresis over Bluetooth. The link comes from the
//    IORegistry (the DualSense's IOHIDDevice "Transport": USB / Bluetooth); unknown -> the Bluetooth rate.
//    Always off on pause, menus, cut-scenes, loading, disconnect, controller change and quit.
//
// settings.txt (read only):
//   adaptive_triggers        = off | light | full          (default full)
//   adaptive_triggers_extras = none | all | gear,limiter,air,impact,damage   (default none: secondary effects)
//   adaptive_triggers_link   = auto | usb | bluetooth      (default auto)
#pragma once

#include "core/settings.hpp"

#include <array>
#include <cstdint>
#include <functional>
#include <memory>
#include <string>
#include <vector>

class DriveSession;

namespace fuel {
namespace fv {
class FuelVehicle;
}
struct VehicleState;

namespace trig {

enum class Level { Off, Light, Full };
enum class Link { Usb, Bluetooth };
enum class Role { None, Throttle, Brake };  // what a physical trigger drives (from the input map)

// One trigger's command (GCDualSenseAdaptiveTrigger).
struct Command {
    enum Mode { Off, Slope, Vibration, Weapon } mode = Off;
    float start = 0, end = 1;                  // Slope / Weapon / Vibration (start only): trigger travel 0..1
    float startStrength = 0, endStrength = 0;  // Slope; Weapon uses endStrength
    float amplitude = 0, frequency = 0;        // Vibration (normalized 0..1)
    // Positional strengths (10 zones) for the fallback when slope feedback is missing (macOS < 12.3).
    std::array<float, 10> positional() const;
    bool same(const Command& o, float eps) const;
    std::string describe() const;
};

const char* className(int vehicleClass);  // BUGGY, BIKE, MUSCLE, QUAD, SUV, TRUCK
float lightScale();                       // `light` = full x this

// Optional extras (secondary effects; off unless adaptive_triggers_extras names them).
struct Extras {
    bool gear = false, limiter = false, air = false, impact = false, damage = false;
    bool any() const { return gear || limiter || air || impact || damage; }
};

struct Config {
    Level level = Level::Full;
    Link link = Link::Usb;
    Extras extras;
    static Config fromSettings(const Settings& s);
};

// What the model reads, per frame.
struct Telemetry {
    // Game state: any of these -> both triggers off.
    bool paused = false, menu = false, cutscene = false, loading = false;
    bool haveVehicle = false;
    int vehicleClass = 0;  // VehicleInfo+0x38: BUGGY 0, BIKE 1, MUSCLE 2, QUAD 3, SUV 4, TRUCK 5
    int gear = 1;          // -1 reverse (the brake drives, the accelerator brakes: 0x523dd0 / 0x523e10)
    float speed = 0;       // forward speed, m/s
    float brakePedal = 0;  // the brake action FUEL reads (0..1), before the reverse swap
    bool wrecked = false;
    struct Wheel {
        bool loaded = false, driven = false;
        float brakeTorque = 0;  // P395 front / P396 rear
        float util = 0;         // grip utilisation sumFn/sumDen (0x523c50); >= 1 sliding
        float rim = 0;          // rim speed, m/s (aux dof 1+w)
        float surface = 1;      // the ground block's static friction C[w].stat (tarmac dry 1)
    };
    std::vector<Wheel> wheels;
    // Extras only.
    bool revLimiter = false;
    float damage = 0;       // 0..1, the HUD needle (P46)
    float impactSpeed = 0;  // chassis approach speed of a new impact this step (m/s)
    float nitro = 0;        // [nitro] the boost amount 0..1 (VehicleState::nitro): a light buzz in the throttle
};
// Reads the player's FUEL vehicle model and state (no writes).
Telemetry telemetryFrom(const fv::FuelVehicle& m, const VehicleState& st, int vehicleClass, float brakeInput);

// What the model worked out (for the check and the trace).
struct Feel {
    float throttleGrip = 1, brakeGrip = 1;  // smoothed grip left, 0..1
    float spin = 0, lock = 0;               // wheelspin / lock-up depth 0..1 (smoothed)
    float surface = 1;                      // ceiling multiplier
    bool spinning = false, locked = false, airborne = false;
    const char* why = "";
};

class Model {
public:
    explicit Model(const Config& c) : cfg_(c) {}
    // dt: seconds since the last call. Roles: what the left (L2) and right (R2) triggers drive.
    void update(const Telemetry& t, float dt, Role left, Role right);
    const Command& left() const { return left_; }
    const Command& right() const { return right_; }
    const Feel& feel() const { return feel_; }
    const Config& config() const { return cfg_; }
    void reset();  // forget smoothing (after a pause, a respawn, a controller change)

private:
    Command throttleCommand(const Telemetry& t, float grip, float surface) const;
    Command brakeCommand(const Telemetry& t, float grip, float surface) const;
    static float band(float v, float& held);  // Bluetooth: 5 bands with hysteresis

    Config cfg_;
    Command left_, right_;
    Feel feel_;
    bool warm_ = false;
    float tGrip_ = 1, bGrip_ = 1, spin_ = 0, lock_ = 0, surf_ = 1;
    float tBand_ = 1, bBand_ = 1, surfBand_ = 1;  // Bluetooth bands
    bool spinOn_ = false, lockOn_ = false;
    float airTime_ = 0;
    int lastGear_ = 0;
    float gearClick_ = 0, impactKick_ = 0, impactAmp_ = 0;
};

// Rate limit: what to send and when. USB: up to 60 Hz, only on a change; Bluetooth: up to 10 Hz. Off is always
// sent at once.
class Sender {
public:
    explicit Sender(Link l) : link_(l) {}
    // True when `c` should be sent now on trigger `side` (0 left, 1 right); then it counts as sent.
    bool due(int side, const Command& c, float dt);
    void forget() { have_[0] = have_[1] = false; }
    float interval() const { return link_ == Link::Usb ? 1.0f / 60 : 0.1f; }

private:
    Link link_;
    Command last_[2];
    bool have_[2] = {false, false};
    float since_[2] = {1e9f, 1e9f};
};

// The device side, swappable for the check.
struct Sink {
    virtual ~Sink() = default;
    virtual bool present() = 0;  // a DualSense is the game's controller
    virtual bool changed() = 0;  // a different controller (or none) since the last call: start from off
    virtual void send(int side, const Command& c) = 0;
    virtual Link link() = 0;     // the detected transport
    virtual std::string name() = 0;
};
std::unique_ptr<Sink> deviceSink(Link fallback, bool forced);
const char* detectedLink();  // what the IORegistry says now (for the check)

}  // namespace trig

class PadTriggers {
public:
    PadTriggers(const Settings& settings, std::unique_ptr<trig::Sink> sink = nullptr);
    ~PadTriggers();
    PadTriggers(const PadTriggers&) = delete;
    PadTriggers& operator=(const PadTriggers&) = delete;

    // Once per driving frame. Roles: what L2 / R2 drive (input map).
    void frame(float dt, const trig::Telemetry& t, trig::Role left = trig::Role::Brake, trig::Role right = trig::Role::Throttle);
    // Pause / menus / cut-scenes / loading outside frame(): both triggers off now.
    void off();
    // Off and released (quit, window closing).
    void shutdown();
    // P101: the Controller page changed adaptive_triggers / _extras: the new level from the next driving frame (off now).
    void reconfigure(const Settings& settings);
    const trig::Model& model() const { return model_; }
    const trig::Command& sent(int side) const { return sent_[side]; }
    bool active() const { return cfg_.level != trig::Level::Off; }

private:
    trig::Config cfg_;
    trig::Model model_;
    std::unique_ptr<trig::Sink> sink_;
    std::unique_ptr<trig::Sender> sender_;
    trig::Command sent_[2];
    bool off_ = true;
    void send(int side, const trig::Command& c);
    void bind();
    void* terminateObserver_ = nullptr;
};

// --trigger-check: a scripted drive of the player's vehicle (wheelspin start, gravel, a jump, a lock-up stop, an
// impact into a parked vehicle that `addObstacle` places, then pause, menu, cut-scene, loading, a disconnect and a
// resume), its telemetry through the model and a recording device at every level and link; prints the modes per
// phase. Returns the number of failures (any trigger on while paused, in a menu, a cut-scene or loading; not off
// after quit or a disconnect; Bluetooth above 10 updates/s; and the drive's expected feel).
struct TriggerCheckFrame {
    std::string phase;
    trig::Telemetry t;
    float dt = 1.0f / 60;
};
int triggerCheck(const std::vector<TriggerCheckFrame>& frames, const std::string& vehicle);
// `steer` (optional): the stick for the vehicle at (x, z) facing `heading` (the race line in main.mm).
int triggerDriveCheck(DriveSession& d, const std::string& vehicle, const std::function<bool(double x, double z, float yaw)>& addObstacle,
                      const std::function<float(double x, double z, float heading)>& steer = nullptr);

}  // namespace fuel
