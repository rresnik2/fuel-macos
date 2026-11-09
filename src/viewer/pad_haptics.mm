// P89: FUEL's motor bytes -> CoreHaptics on the game's controller. See pad_haptics.h.
#include "pad_haptics.h"

#include <algorithm>
#include <cmath>

#import <QuartzCore/QuartzCore.h>

#import <CoreHaptics/CoreHaptics.h>
#import <GameController/GameController.h>

#include <algorithm>
#include <atomic>
#include <cstdio>

namespace fuel {

namespace {

bool settingOn(const Settings& s, const char* key, bool def) {
    const auto it = s.all.find(key);
    if (it == s.all.end()) return def;
    const std::string& v = it->second;
    return !(v == "0" || v == "false" || v == "no" || v == "off");
}

// One handle (or the whole pad): an engine with a looping continuous event whose intensity follows a motor.
struct Channel {
    CHHapticEngine* engine = nil;
    id<CHHapticPatternPlayer> player = nil;  // a plain player: the controller's haptics service (gamecontrollerd.haptics)
                                             // drops the connection on createAdvancedPlayer (NSCocoaErrorDomain 4097,
                                             // DualSense over USB, macOS 26), so no loop: a 30 s event restarted instead
    double startedAt = 0;  // CACurrentMediaTime of the last start
    float sharpness = 0.5f;
    int sent = -1;       // the last level sent (0..255), -1 = none
    bool playing = false;
    std::atomic<bool> broken{false};  // the engine stopped or reset (set on CoreHaptics' queue): rebuilt by sync()

    bool open(GCController* c, GCHapticsLocality locality, float sharp) API_AVAILABLE(macos(11.0)) {
        sharpness = sharp;
        broken = false;
        engine = [c.haptics createEngineWithLocality:locality];
        if (!engine) {
            std::printf("pad haptics: %s: createEngineWithLocality returned nil\n", locality.UTF8String);
            return false;
        }
        engine.playsHapticsOnly = YES;
        std::atomic<bool>* flag = &broken;  // the handlers are cleared in close(), before the Channel goes
        engine.stoppedHandler = ^(CHHapticEngineStoppedReason) { *flag = true; };
        engine.resetHandler = ^{ *flag = true; };
        NSError* err = nil;
        if (![engine startAndReturnError:&err]) {
            std::printf("pad haptics: %s: engine start failed: %s\n", locality.UTF8String, err ? err.description.UTF8String : "?");
            engine = nil;
            return false;
        }
        // A 30 s continuous event (CoreHaptics' longest) at full intensity, restarted before it ends; the intensity
        // control scales it per frame.
        CHHapticEventParameter* in = [[CHHapticEventParameter alloc] initWithParameterID:CHHapticEventParameterIDHapticIntensity value:1.0f];
        CHHapticEventParameter* sh = [[CHHapticEventParameter alloc] initWithParameterID:CHHapticEventParameterIDHapticSharpness value:sharp];
        CHHapticEvent* ev = [[CHHapticEvent alloc] initWithEventType:CHHapticEventTypeHapticContinuous parameters:@[ in, sh ] relativeTime:0 duration:kEventSeconds];
        CHHapticPattern* pat = [[CHHapticPattern alloc] initWithEvents:@[ ev ] parameters:@[] error:&err];
        if (pat) player = [engine createPlayerWithPattern:pat error:&err];
        if (!player) {
            std::printf("pad haptics: %s: pattern/player failed: %s\n", locality.UTF8String, err ? err.description.UTF8String : "?");
            [engine stopWithCompletionHandler:nil];
            engine = nil;
            return false;
        }
        return true;
    }
    static constexpr double kEventSeconds = 30.0;
    void set(int level) {
        if (player && playing && level > 0 && CACurrentMediaTime() - startedAt > kEventSeconds - 1.0) {
            NSError* err = nil;  // the event is about to end: start it again (the intensity control carries over below)
            if ([player startAtTime:CHHapticTimeImmediate error:&err]) startedAt = CACurrentMediaTime(), sent = -1;
            else broken = true;
        }
        if (!player || level == sent) return;
        sent = level;
        NSError* err = nil;
        if (level <= 0) {
            if (playing) [player stopAtTime:CHHapticTimeImmediate error:&err];
            playing = false;
            return;
        }
        CHHapticDynamicParameter* p = [[CHHapticDynamicParameter alloc] initWithParameterID:CHHapticDynamicParameterIDHapticIntensityControl
                                                                                      value:float(level) / 255.0f
                                                                               relativeTime:0];
        [player sendParameters:@[ p ] atTime:CHHapticTimeImmediate error:&err];
        if (!playing) {
            if ([player startAtTime:CHHapticTimeImmediate error:&err]) {
                playing = true, startedAt = CACurrentMediaTime();
                [player sendParameters:@[ p ] atTime:CHHapticTimeImmediate error:&err];  // the level again, on the running event
            } else {
                broken = true;
            }
        }
    }
    void close() {
        NSError* err = nil;
        if (player && playing) [player stopAtTime:CHHapticTimeImmediate error:&err];
        player = nil;
        if (engine) {
            engine.stoppedHandler = ^(CHHapticEngineStoppedReason) {};
            engine.resetHandler = ^{};
            [engine stopWithCompletionHandler:nil];
        }
        engine = nil;
        playing = false;
        sent = -1;
        broken = false;
    }
};

}  // namespace

struct PadHaptics::Device {
    GCController* controller = nil;  // the one the channels were built for
    bool tried = false;              // built (or found without haptics) for `controller`
    bool split = false;              // left and right handles; else one default channel
    Channel left, right, whole;
    void close() {
        left.close(), right.close(), whole.close();
        controller = nil;
        tried = false;
    }
    // Follows GCController.controllers.firstObject (the game's pad): rebuilds on a change or a broken engine.
    void sync() {
        GCController* c = GCController.controllers.firstObject;
        const bool broken = left.broken || right.broken || whole.broken;
        if (c == controller && tried && !broken) return;
        close();
        controller = c;
        tried = true;
        if (!c) return;
        if (@available(macOS 11.0, *)) {
            if (!c.haptics) {
                std::printf("pad haptics: %s has no haptics (no vibration)\n", c.vendorName ? c.vendorName.UTF8String : "controller");
                return;
            }
            NSSet<GCHapticsLocality>* loc = c.haptics.supportedLocalities;
            split = [loc containsObject:GCHapticsLocalityLeftHandle] && [loc containsObject:GCHapticsLocalityRightHandle] &&
                    left.open(c, GCHapticsLocalityLeftHandle, 0.15f) && right.open(c, GCHapticsLocalityRightHandle, 0.85f);
            if (!split) {
                left.close(), right.close();
                if (!whole.open(c, GCHapticsLocalityDefault, 0.5f)) {
                    std::printf("pad haptics: %s: no haptic engine\n", c.vendorName ? c.vendorName.UTF8String : "controller");
                    return;
                }
            }
            std::printf("pad haptics: %s, %s\n", c.vendorName ? c.vendorName.UTF8String : "controller",
                        split ? "left handle = FUEL's low-frequency motor, right handle = high-frequency" : "one engine (the stronger motor)");
        }
    }
    void send(const vib::Motors& m) {
        sync();
        if (split) left.set(m.low), right.set(m.high);
        else if (whole.engine) whole.set(std::max(m.low, m.high));
    }
};

PadHaptics::PadHaptics(const std::string& dataDir, const Settings& settings)
    : shapes_(vib::Shapes::load(dataDir)), manager_(&shapes_), dev_(std::make_unique<Device>()) {
    manager_.option = settingOn(settings, "vibration", true);
    if (!shapes_.error.empty()) std::printf("pad haptics: %s (no vibration)\n", shapes_.error.c_str());
    else if (!manager_.option) std::printf("pad haptics: off (settings.txt vibration = 0)\n");
    if (const auto it = settings.all.find("drive_rumble"); it != settings.all.end())  // port deviation (user request)
        driveScale_ = it->second == "off" || it->second == "0" ? 0.0f : it->second == "light" ? 0.6f : 1.0f;
}

PadHaptics::~PadHaptics() { shutdown(); }

void PadHaptics::send(const vib::Motors& m) {
    motors_ = m;  // FUEL's own motors (what the checks read)
    vib::Motors out = m;  // + the port's drive rumble (max per motor)
    out.low = std::max(out.low, driveLow_), out.high = std::max(out.high, driveHigh_);
    out = vib::maxCombine(out, nitroMotors_);  // [nitro]
    if (dev_) dev_->send(out);
}

void PadHaptics::drive(float dt, const DriveFeel& f) {
    auto clamp01 = [](float x) { return std::clamp(x, 0.0f, 1.0f); };
    auto follow = [dt](float cur, float target, float up, float down) {
        const float tau = target > cur ? up : down;
        return cur + (target - cur) * (1 - std::exp(-dt / tau));
    };
    float tex = 0, slip = 0;
    if (f.active && driveScale_ > 0 && manager_.option && dt > 0) {
        const float v = std::fabs(f.speed);
        // A new grain level every ~0.6 m of ground (at least 1 m/s), so the rumble reads as a surface, not a hum.
        grainT_ += dt * std::max(v, 1.0f) / 0.6f;
        if (grainT_ >= 1) {
            grainT_ -= std::floor(grainT_);
            grainSeed_ = grainSeed_ * 1664525u + 1013904223u;
            grain_ = 0.55f + 0.45f * float(grainSeed_ >> 8) / float(1u << 24);
        }
        // tarmac a faint hum, FUEL's roughest ground (BUMPS 0.135) the full texture; it grows with speed up to 20 m/s
        if (!f.airborne) tex = (0.12f + 0.88f * clamp01(f.rough / 0.135f)) * clamp01((v - 1.0f) / 19.0f) * grain_;
        slip = clamp01(f.slip) * clamp01(v / 4.0f);
    }
    texS_ = follow(texS_, tex, 0.03f, 0.08f);
    slipS_ = follow(slipS_, slip, 0.04f, 0.45f);  // the slide fades out slowly
    driveLow_ = int(std::lround(255.0f * driveScale_ * 0.40f * texS_));
    driveHigh_ = int(std::lround(255.0f * driveScale_ * 0.60f * slipS_));
    nitroMotors_ = nitro_.step(dt, f.nitro, f.active && manager_.option);  // [nitro] core/vibration.hpp NitroRumble
}

void PadHaptics::frame(float dt, bool cutsceneStarted) {
    if (cutsceneStarted) manager_.stopAll();
    send(manager_.update(dt));
}

void PadHaptics::setPaused(bool paused) {
    if (!paused) inMenu_ = false;
    if (paused == !manager_.active()) return;
    manager_.setActive(!paused);
    if (paused) driveLow_ = driveHigh_ = 0, texS_ = slipS_ = 0, nitroMotors_ = {}, nitro_.reset(), send({});
}

void PadHaptics::menuFrame(float dt) {
    if (!inMenu_) inMenu_ = true, setPaused(true);
    if (manager_.active()) send(manager_.update(dt));
}

void PadHaptics::setOption(bool on) {
    if (on == manager_.option) return;
    manager_.option = on;
    manager_.setActive(on);  // 0x557d80 / 0x557c90
    if (!on) send({});
    std::printf("pad haptics: vibration %s (options page)\n", on ? "on" : "off");
}

void PadHaptics::testPulse() {
    manager_.option = true;
    manager_.setActive(true);
    manager_.clearCooldown();
    manager_.add("IMPACT_MEDIUM", 2, 1.0f);
}

void PadHaptics::shutdown() {
    if (!dev_) return;
    dev_->send({});
    dev_->close();
}

}  // namespace fuel
