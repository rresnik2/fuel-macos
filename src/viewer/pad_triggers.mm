// P91: DualSense adaptive triggers (an addition beyond FUEL). See pad_triggers.h.
#include "pad_triggers.h"

#include "drive.h"
#include "physics/fuel_vehicle.hpp"
#include "physics/vehicle_physics.hpp"

#import <AppKit/AppKit.h>
#import <GameController/GameController.h>
#include <IOKit/IOKitLib.h>
#include <IOKit/hid/IOHIDKeys.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <map>
#include <sstream>
#include <unistd.h>

namespace fuel {
namespace trig {

namespace {

float clamp01(float v) { return std::clamp(v, 0.0f, 1.0f); }
float smoothstep(float a, float b, float x) {
    const float t = clamp01((x - a) / (b - a));
    return t * t * (3 - 2 * t);
}
// Exponential smoothing toward `target` with time constants for rising / falling.
float follow(float cur, float target, float dt, float tauUp, float tauDown) {
    const float tau = target > cur ? tauUp : tauDown;
    if (tau <= 0 || dt <= 0) return dt <= 0 ? cur : target;
    return cur + (target - cur) * (1 - std::exp(-dt / tau));
}
std::string lower(std::string s) {
    for (char& c : s) c = char(std::tolower(static_cast<unsigned char>(c)));
    return s;
}

// ---- the feel's numbers (tuning; see the notes) ----
// Progressive resistance at `full`, before the class weight and the surface: slope from `start` to `end` of the
// travel, strength s0 -> s1.
constexpr float kThrStart = 0.05f, kThrEnd = 0.95f, kThrS0 = 0.10f, kThrS1 = 0.45f;  // gentle -> firmer
constexpr float kBrkStart = 0.05f, kBrkEnd = 0.85f, kBrkS0 = 0.08f, kBrkS1 = 0.90f;  // soft top -> stiff bottom
// Class weight (VehicleInfo+0x38): BUGGY, BIKE, MUSCLE, QUAD, SUV, TRUCK.
constexpr float kClassWeight[6] = {1.0f, 0.7f, 1.0f, 0.85f, 1.1f, 1.3f};
constexpr const char* kClassName[6] = {"BUGGY", "BIKE", "MUSCLE", "QUAD", "SUV", "TRUCK"};
constexpr float kLightScale = 0.55f;  // `light`: every strength and amplitude
// Grip left of one wheel from FUEL's utilisation (sumFn/sumDen; 1 = FUEL's sliding threshold 0x524240).
float wheelGrip(float util) { return 1 - smoothstep(0.7f, 1.2f, util); }
// Surface ceiling from the ground block's static friction (tarmac dry 1, dirt road 0.7, mud 0.5, ice/snow 0.3).
float surfaceCeiling(float stat) { return 0.45f + 0.55f * clamp01(stat); }

}  // namespace

const char* className(int c) { return kClassName[std::clamp(c, 0, 5)]; }
float lightScale() { return kLightScale; }

std::array<float, 10> Command::positional() const {
    std::array<float, 10> z{};
    if (mode != Slope) return z;
    for (int i = 0; i < 10; ++i) {
        const float p = (float(i) + 0.5f) / 10.0f;
        if (p < start || p > end) continue;
        z[size_t(i)] = startStrength + (endStrength - startStrength) * (p - start) / std::max(end - start, 1e-3f);
    }
    return z;
}

bool Command::same(const Command& o, float eps) const {
    if (mode != o.mode) return false;
    auto near = [&](float a, float b) { return std::fabs(a - b) <= eps; };
    switch (mode) {
    case Off: return true;
    case Slope: return near(start, o.start) && near(end, o.end) && near(startStrength, o.startStrength) && near(endStrength, o.endStrength);
    case Vibration: return near(start, o.start) && near(amplitude, o.amplitude) && near(frequency, o.frequency);
    case Weapon: return near(start, o.start) && near(end, o.end) && near(endStrength, o.endStrength);
    }
    return false;
}

std::string Command::describe() const {
    char b[128];
    switch (mode) {
    case Off: return "off";
    case Slope: std::snprintf(b, sizeof b, "slope %.2f-%.2f %.2f->%.2f", start, end, startStrength, endStrength); break;
    case Vibration: std::snprintf(b, sizeof b, "vibration from %.2f amp %.2f freq %.2f", start, amplitude, frequency); break;
    case Weapon: std::snprintf(b, sizeof b, "weapon %.2f-%.2f %.2f", start, end, endStrength); break;
    }
    return b;
}

Config Config::fromSettings(const Settings& s) {
    Config c;
    auto get = [&](const char* k) {
        const auto it = s.all.find(k);
        return it == s.all.end() ? std::string() : lower(it->second);
    };
    const std::string lv = get("adaptive_triggers");
    if (lv == "off" || lv == "0" || lv == "false" || lv == "no") c.level = Level::Off;
    else if (lv == "light" || lv == "low") c.level = Level::Light;
    else c.level = Level::Full;  // default (user decision 2026-11-05): on, full
    const std::string ex = get("adaptive_triggers_extras");
    const bool all = ex == "all" || ex == "on" || ex == "1";
    c.extras.gear = all || ex.find("gear") != std::string::npos;
    c.extras.limiter = all || ex.find("limiter") != std::string::npos;
    c.extras.air = all || ex.find("air") != std::string::npos;
    c.extras.impact = all || ex.find("impact") != std::string::npos;
    c.extras.damage = all || ex.find("damage") != std::string::npos;
    return c;  // the link (rate) comes from the device: PadTriggers::bind

}

Telemetry telemetryFrom(const fv::FuelVehicle& m, const VehicleState& st, int vehicleClass, float brakeInput) {
    Telemetry t;
    t.haveVehicle = true;
    t.vehicleClass = vehicleClass;
    t.gear = m.gear();
    t.speed = m.forwardSpeed();
    t.brakePedal = brakeInput;
    t.wrecked = m.wrecked();
    t.nitro = st.nitro.amount;  // [nitro]
    const bool frontDriven = m.param(46) > 0.5f, rearDriven = m.param(47) > 0.5f;  // driven(w) (0x52def0 flags)
    for (int w = 0; w < m.wheelCount(); ++w) {
        const fv::WheelRec& r = m.wheel(w);
        Telemetry::Wheel x;
        x.loaded = r.load > 1e-6f;
        x.driven = m.front(w) ? frontDriven : rearDriven;
        x.brakeTorque = m.param(m.front(w) ? 395 : 396);  // 0x520fd0 Pb
        x.util = x.loaded && r.sumDen >= 1e-6f ? r.sumFn / r.sumDen : 0.0f;  // 0x523c50
        x.rim = m.rimSpeed(w);
        x.surface = m.ground(w).stat;
        t.wheels.push_back(x);
    }
    t.revLimiter = st.revLimiter;
    t.damage = st.damage;
    t.impactSpeed = st.impactSpeed;
    return t;
}

void Model::reset() {
    warm_ = false;
    spinOn_ = lockOn_ = false;
    airTime_ = 0;
    gearClick_ = impactKick_ = 0;
    lastGear_ = 0;
}

float Model::band(float v, float& held) {
    // 5 bands (0, .25, .5, .75, 1); move only past the band edge plus a margin.
    const float nearest = std::round(v * 4) / 4;
    if (std::fabs(v - held) > 0.125f + 0.06f) held = nearest;
    return held;
}

void Model::update(const Telemetry& t, float dt, Role left, Role right) {
    feel_ = Feel{};
    const bool stop = cfg_.level == Level::Off || t.paused || t.menu || t.cutscene || t.loading || !t.haveVehicle;
    if (stop) {
        left_ = right_ = Command{};
        feel_.why = cfg_.level == Level::Off ? "setting off" : t.paused ? "paused" : t.menu ? "menu" : t.cutscene ? "cut-scene" : t.loading ? "loading" : "no vehicle";
        reset();
        return;
    }
    if (t.wrecked) {
        left_ = right_ = Command{};
        feel_.why = "wrecked";
        reset();
        return;
    }
    // In reverse FUEL swaps the pedals (0x523dd0 / 0x523e10): the brake drives, the accelerator brakes.
    const bool reverse = t.gear == -1;
    const float v = std::fabs(t.speed);

    // ---- grip from the wheel records ----
    float gT = 0, spin = 0, nT = 0;          // driven wheels
    float gB = 0, lock = 0, wB = 0;          // braked wheels, weighted by brake torque
    float surf = 0, nS = 0;
    bool drivenLoaded = false;
    const float brakeOn = reverse ? 0.0f : t.brakePedal;  // the pedal that brakes (reverse: the accelerator; not read)
    for (const Telemetry::Wheel& w : t.wheels) {
        if (!w.loaded) continue;
        surf += w.surface, nS += 1;
        const float g = wheelGrip(w.util);
        if (w.driven) {
            drivenLoaded = true;
            // FUEL: a driven wheel past its grip turns free (tyre reaction 0x520d40), else it rolls with the ground
            // (kinematic rolling): spin = the rim running ahead of the ground, by more than 1.5 m/s and 20 %.
            const float ahead = std::fabs(w.rim) - v;
            const float s = ahead > 1.5f ? clamp01((ahead / std::max(v, 3.0f) - 0.2f) / 0.8f) : 0.0f;
            gT += std::min(g, 1 - s), spin += s, nT += 1;
        }
        if (w.brakeTorque > 0) {
            float l = 0;
            if (v > 2.5f && brakeOn > 0.05f) {
                // Braking past the grip: FUEL's sliding (utilisation >= 1, 0x524240), or the rim far slower than the
                // ground. (FUEL's brake torque fades with the rim speed, 0x520fd0, so its wheels rarely stop dead.)
                const float roll = w.rim * (t.speed >= 0 ? 1.0f : -1.0f) / v;
                l = std::max(clamp01((1 - roll - 0.4f) / 0.5f), smoothstep(1.0f, 1.6f, w.util));
            }
            gB += w.brakeTorque * std::min(g, 1 - l), lock += w.brakeTorque * l, wB += w.brakeTorque;
        }
    }
    const float targetT = nT > 0 ? gT / nT : 1.0f;    // nothing driven on the ground: the base slope
    const float targetSpin = nT > 0 ? spin / nT : 0.0f;
    const float targetB = wB > 0 ? gB / wB : 1.0f;
    const float targetLock = wB > 0 ? lock / wB : 0.0f;
    const float targetSurf = nS > 0 ? surfaceCeiling(surf / nS) : surf_;
    if (!warm_) {
        tGrip_ = targetT, bGrip_ = targetB, spin_ = targetSpin, lock_ = targetLock, surf_ = targetSurf;
        tBand_ = bBand_ = surfBand_ = 1;
        warm_ = true;
    } else {  // grip drops fast, comes back slowly
        // [user, 2026-11-05: the grip feel faded too quickly] grip comes back over ~0.6 s (was 0.15 s), wheelspin and
        // lock-up fade over ~0.5 / 0.4 s (were 0.2 / 0.15 s); the drop stays immediate.
        tGrip_ = follow(tGrip_, targetT, dt, 0.6f, 0.06f);
        bGrip_ = follow(bGrip_, targetB, dt, 0.6f, 0.06f);
        spin_ = follow(spin_, targetSpin, dt, 0.05f, 0.5f);
        lock_ = follow(lock_, targetLock, dt, 0.04f, 0.4f);
        surf_ = follow(surf_, targetSurf, dt, 0.25f, 0.25f);
    }
    if (spin_ > 0.55f) spinOn_ = true;
    else if (spin_ < 0.35f) spinOn_ = false;
    if (lock_ > 0.5f) lockOn_ = true;
    else if (lock_ < 0.25f) lockOn_ = false;
    airTime_ = drivenLoaded ? 0.0f : airTime_ + dt;

    feel_.throttleGrip = tGrip_, feel_.brakeGrip = bGrip_, feel_.spin = spin_, feel_.lock = lock_, feel_.surface = surf_;
    feel_.spinning = spinOn_, feel_.locked = lockOn_, feel_.airborne = airTime_ > 0.15f;
    feel_.why = "driving";

    // ---- extras (timers) ----
    if (cfg_.extras.gear && lastGear_ != 0 && t.gear != lastGear_) gearClick_ = 0.07f;
    lastGear_ = t.gear;
    gearClick_ = std::max(0.0f, gearClick_ - dt);
    impactKick_ = std::max(0.0f, impactKick_ - dt);
    if (cfg_.extras.impact && t.impactSpeed > 3.0f) {
        const float a = 0.2f + 0.6f * clamp01((t.impactSpeed - 3.0f) / 8.0f);
        impactAmp_ = impactKick_ > 0 ? std::max(impactAmp_, a) : a;
        impactKick_ = 0.12f;
    }

    // Bluetooth: fewer, coarser updates (5 grip bands with hysteresis).
    const bool bt = cfg_.link == Link::Bluetooth;
    const float gThr = bt ? band(tGrip_, tBand_) : tGrip_, gBrk = bt ? band(bGrip_, bBand_) : bGrip_;
    const float surface = bt ? band(surf_, surfBand_) : surf_;
    Command thr = throttleCommand(t, gThr, surface), brk = brakeCommand(t, gBrk, surface);
    const float lv = cfg_.level == Level::Light ? kLightScale : 1.0f;
    if (impactKick_ > 0) {  // a short kick on both
        Command k;
        k.mode = Command::Vibration, k.start = 0, k.amplitude = impactAmp_ * lv, k.frequency = 0.3f;
        thr = brk = k;
    }
    auto roleCmd = [&](Role r) {
        if (r == Role::None) return Command{};
        const bool drives = (r == Role::Throttle) != reverse;
        return drives ? thr : brk;
    };
    left_ = roleCmd(left), right_ = roleCmd(right);
}

Command Model::throttleCommand(const Telemetry& t, float g, float s) const {
    const float lv = cfg_.level == Level::Light ? kLightScale : 1.0f;
    const float k = kClassWeight[std::clamp(t.vehicleClass, 0, 5)] * lv;
    Command c;
    if (cfg_.extras.gear && gearClick_ > 0) {  // a short click on a shift
        c.mode = Command::Vibration, c.start = 0, c.amplitude = 0.5f * lv, c.frequency = 0.6f;
        return c;
    }
    if (cfg_.extras.air && airTime_ > 0.15f) return c;  // airborne: slack until the wheels land
    const bool bt = cfg_.link == Link::Bluetooth;
    if (spinOn_) {  // deep wheelspin: the trigger gives way and buzzes, low and light
        c.mode = Command::Vibration, c.start = 0.15f;
        c.amplitude = std::min(1.0f, (0.15f + 0.25f * clamp01((spin_ - 0.35f) / 0.65f)) * lv);
        if (bt) c.amplitude = std::round(c.amplitude * 20) / 20;  // coarse steps: fewer Bluetooth updates
        c.frequency = 0.12f;
        return c;
    }
    if (t.nitro > 0.05f) {  // [nitro] boosting: a light, quick buzz past the first tenth of the travel
        c.mode = Command::Vibration, c.start = 0.1f;
        c.amplitude = std::min(1.0f, 0.2f * clamp01(t.nitro) * lv);
        if (bt) c.amplitude = std::round(c.amplitude * 20) / 20;
        c.frequency = 0.55f;
        return c;
    }
    if (cfg_.extras.limiter && t.revLimiter) {
        c.mode = Command::Vibration, c.start = 0.5f, c.amplitude = 0.15f * lv, c.frequency = 0.8f;
        return c;
    }
    if (cfg_.extras.damage && t.damage > 0.6f) {
        c.mode = Command::Vibration, c.start = 0.05f, c.amplitude = (t.damage > 0.85f ? 0.18f : 0.1f) * lv, c.frequency = 0.08f;
        return c;
    }
    c.mode = Command::Slope, c.start = kThrStart, c.end = kThrEnd;
    c.startStrength = std::min(1.0f, kThrS0 * k * s * (0.4f + 0.6f * g));
    c.endStrength = std::min(1.0f, kThrS1 * k * s * (0.25f + 0.75f * g));
    return c;
}

Command Model::brakeCommand(const Telemetry& t, float g, float s) const {
    const float lv = cfg_.level == Level::Light ? kLightScale : 1.0f;
    const float k = kClassWeight[std::clamp(t.vehicleClass, 0, 5)] * lv;
    Command c;
    if (lockOn_) {  // locked: the resistance falls away and a slow pulse starts (FUEL has no ABS: feel only)
        c.mode = Command::Vibration, c.start = 0.25f, c.amplitude = 0.35f * lv, c.frequency = 0.04f;
        return c;
    }
    c.mode = Command::Slope, c.start = kBrkStart, c.end = kBrkEnd;
    c.startStrength = std::min(1.0f, kBrkS0 * k * s);
    c.endStrength = std::min(1.0f, kBrkS1 * k * s * (0.2f + 0.8f * g));  // the bottom softens toward lock-up
    return c;
}

bool Sender::due(int side, const Command& c, float dt) {
    since_[side] += dt;
    if (have_[side] && last_[side].same(c, link_ == Link::Usb ? 0.015f : 0.01f)) return false;
    const bool urgent = c.mode == Command::Off || !have_[side];
    if (!urgent && since_[side] + 1e-4f < interval()) return false;
    last_[side] = c, have_[side] = true, since_[side] = 0;
    return true;
}

// ---- the device: the game's controller when it is a DualSense ----

namespace {

// The DualSense's transport from the IORegistry (no device is opened): Sony 0x054c, DualSense 0x0ce6 /
// DualSense Edge 0x0df2. Any Bluetooth one -> Bluetooth; USB only -> USB; none found -> unknown (false).
bool detectLink(Link& out) {
    CFMutableDictionaryRef match = IOServiceMatching(kIOHIDDeviceKey);
    if (!match) return false;
    io_iterator_t it = 0;
    if (IOServiceGetMatchingServices(MACH_PORT_NULL, match, &it) != KERN_SUCCESS) return false;
    bool usb = false, bt = false;
    auto num = [](io_service_t s, CFStringRef key) {
        long v = -1;
        if (CFTypeRef r = IORegistryEntryCreateCFProperty(s, key, kCFAllocatorDefault, 0)) {
            if (CFGetTypeID(r) == CFNumberGetTypeID()) CFNumberGetValue(CFNumberRef(r), kCFNumberLongType, &v);
            CFRelease(r);
        }
        return v;
    };
    while (io_service_t s = IOIteratorNext(it)) {
        const long vid = num(s, CFSTR(kIOHIDVendorIDKey)), pid = num(s, CFSTR(kIOHIDProductIDKey));
        if (vid == 0x054c && (pid == 0x0ce6 || pid == 0x0df2)) {
            if (CFTypeRef r = IORegistryEntryCreateCFProperty(s, CFSTR(kIOHIDTransportKey), kCFAllocatorDefault, 0)) {
                if (CFGetTypeID(r) == CFStringGetTypeID()) {
                    char b[64] = {};
                    CFStringGetCString(CFStringRef(r), b, sizeof b, kCFStringEncodingUTF8);
                    const std::string tr = lower(b);
                    if (tr.find("bluetooth") != std::string::npos) bt = true;
                    else if (tr == "usb") usb = true;
                }
                CFRelease(r);
            }
        }
        IOObjectRelease(s);
    }
    IOObjectRelease(it);
    if (bt) return out = Link::Bluetooth, true;
    if (usb) return out = Link::Usb, true;
    return false;
}

struct DeviceSink final : Sink {
    GCController* controller = nil;
    bool dualSense = false;
    bool seen = false;
    bool dirty = false;
    Link fallback, current;
    bool forced;
    DeviceSink(Link f, bool forced_) : fallback(f), current(f), forced(forced_) {}

    GCDualSenseAdaptiveTrigger* trigger(GCController* c, int side) API_AVAILABLE(macos(11.3)) {
        GCDualSenseGamepad* ds = (GCDualSenseGamepad*)c.extendedGamepad;
        return side ? ds.rightTrigger : ds.leftTrigger;
    }
    static bool isDualSense(GCController* c) {
        if (!c) return false;
        if (@available(macOS 11.3, *)) return [c.extendedGamepad isKindOfClass:[GCDualSenseGamepad class]];
        return false;
    }
    void poll() {
        GCController* c = GCController.controllers.firstObject;
        if (seen && c == controller) return;
        // The old pad, if still connected and a DualSense: leave it off.
        if (controller && dualSense && [GCController.controllers containsObject:controller]) {
            if (@available(macOS 11.3, *)) [trigger(controller, 0) setModeOff], [trigger(controller, 1) setModeOff];
        }
        controller = c;
        dualSense = isDualSense(c);
        seen = true;
        dirty = true;
        if (!dualSense) {
            if (c) std::printf("adaptive triggers: %s is not a DualSense (none)\n", c.vendorName ? c.vendorName.UTF8String : "controller");
            return;
        }
        Link l = fallback;
        const bool known = !forced && detectLink(l);
        current = forced ? fallback : l;
        std::printf("adaptive triggers: %s over %s%s\n", c.vendorName ? c.vendorName.UTF8String : "DualSense",
                    current == Link::Usb ? "USB (up to 60 updates/s)" : "Bluetooth (10 updates/s, 5 grip bands)",
                    forced ? " (settings.txt)" : known ? "" : " (link unknown: the Bluetooth rate)");
    }
    bool present() override {
        poll();
        return dualSense;
    }
    bool changed() override {
        poll();
        const bool d = dirty;
        dirty = false;
        return d;
    }
    Link link() override { return current; }
    std::string name() override { return controller && controller.vendorName ? controller.vendorName.UTF8String : "none"; }
    void send(int side, const Command& c) override {
        if (!dualSense || !controller) return;
        if (@available(macOS 11.3, *)) {
            GCDualSenseAdaptiveTrigger* tr = trigger(controller, side);
            switch (c.mode) {
            case Command::Off: [tr setModeOff]; break;
            case Command::Slope:
                if (@available(macOS 12.3, *)) {
                    if ([tr respondsToSelector:@selector(setModeSlopeFeedbackWithStartPosition:endPosition:startStrength:endStrength:)]) {
                        [tr setModeSlopeFeedbackWithStartPosition:c.start endPosition:c.end startStrength:c.startStrength endStrength:c.endStrength];
                        break;
                    }
                    if ([tr respondsToSelector:@selector(setModeFeedbackWithResistiveStrengths:)]) {
                        GCDualSenseAdaptiveTriggerPositionalResistiveStrengths z;
                        const auto p = c.positional();
                        for (int i = 0; i < 10; ++i) z.values[i] = p[size_t(i)];
                        [tr setModeFeedbackWithResistiveStrengths:z];
                        break;
                    }
                }
                // Before 12.3: one constant strength from the slope's middle.
                [tr setModeFeedbackWithStartPosition:c.start resistiveStrength:0.5f * (c.startStrength + c.endStrength)];
                break;
            case Command::Vibration: [tr setModeVibrationWithStartPosition:c.start amplitude:c.amplitude frequency:c.frequency]; break;
            case Command::Weapon: [tr setModeWeaponWithStartPosition:c.start endPosition:c.end resistiveStrength:c.endStrength]; break;
            }
        }
    }
};

}  // namespace

std::unique_ptr<Sink> deviceSink(Link fallback, bool forced) { return std::make_unique<DeviceSink>(fallback, forced); }

const char* detectedLink() {
    Link l;
    if (!detectLink(l)) return "no DualSense in the IORegistry (the Bluetooth rate if one appears without a known link)";
    return l == Link::Usb ? "a DualSense on USB" : "a DualSense on Bluetooth";
}

}  // namespace trig

// ---- PadTriggers ----

namespace {
bool linkForced(const Settings& s, trig::Link& l) {
    const auto it = s.all.find("adaptive_triggers_link");
    if (it == s.all.end()) return false;
    const std::string v = it->second;
    if (v == "usb" || v == "USB") return l = trig::Link::Usb, true;
    if (v == "bluetooth" || v == "bt" || v == "Bluetooth") return l = trig::Link::Bluetooth, true;
    return false;
}
}  // namespace

PadTriggers::PadTriggers(const Settings& settings, std::unique_ptr<trig::Sink> sink)
    : cfg_(trig::Config::fromSettings(settings)), model_(cfg_) {
    trig::Link forcedLink = trig::Link::Bluetooth;
    const bool forced = linkForced(settings, forcedLink);
    if (!sink) sink = trig::deviceSink(forced ? forcedLink : trig::Link::Bluetooth, forced);
    sink_ = std::move(sink);
    bind();
    if (cfg_.level == trig::Level::Off) std::printf("adaptive triggers: off (settings.txt adaptive_triggers = off)\n");
    // Leave the triggers off whatever an earlier run did; then the window's quit turns them off too.
    if (sink_->present()) send(0, {}), send(1, {});
    PadTriggers* self = this;
    id obs = [[NSNotificationCenter defaultCenter] addObserverForName:NSApplicationWillTerminateNotification
                                                               object:nil
                                                                queue:nil
                                                           usingBlock:^(NSNotification*) { self->shutdown(); }];
    terminateObserver_ = (__bridge_retained void*)obs;
}

PadTriggers::~PadTriggers() {
    shutdown();
    if (terminateObserver_) {
        id obs = (__bridge_transfer id)terminateObserver_;
        [[NSNotificationCenter defaultCenter] removeObserver:obs];
        terminateObserver_ = nullptr;
    }
}

void PadTriggers::bind() {
    cfg_.link = sink_->link();
    model_ = trig::Model(cfg_);
    sender_ = std::make_unique<trig::Sender>(cfg_.link);
}

void PadTriggers::send(int side, const trig::Command& c) {
    sent_[side] = c;
    sink_->send(side, c);
}

void PadTriggers::frame(float dt, const trig::Telemetry& t, trig::Role left, trig::Role right) {
    if (sink_->changed()) {  // new controller / disconnect: start from off at its own link rate
        bind();
        sent_[0] = sent_[1] = {};
        if (sink_->present()) send(0, {}), send(1, {}), sender_->due(0, {}, 0), sender_->due(1, {}, 0);
    }
    if (!sink_->present()) return;
    model_.update(t, dt, left, right);
    const trig::Command* want[2] = {&model_.left(), &model_.right()};
    off_ = want[0]->mode == trig::Command::Off && want[1]->mode == trig::Command::Off;
    for (int side = 0; side < 2; ++side)
        if (sender_->due(side, *want[side], dt)) send(side, *want[side]);
}

void PadTriggers::reconfigure(const Settings& settings) {
    const trig::Config c = trig::Config::fromSettings(settings);
    if (c.level == cfg_.level && c.extras.gear == cfg_.extras.gear && c.extras.limiter == cfg_.extras.limiter && c.extras.air == cfg_.extras.air &&
        c.extras.impact == cfg_.extras.impact && c.extras.damage == cfg_.extras.damage)
        return;
    cfg_.level = c.level, cfg_.extras = c.extras;
    bind();  // the model on the new config, at the pad's link
    off();
    std::printf("adaptive triggers: %s (options page)\n", cfg_.level == trig::Level::Off ? "off" : cfg_.level == trig::Level::Light ? "light" : "full");
}

void PadTriggers::off() {
    model_.reset();
    if (!sink_->present()) return;
    for (int side = 0; side < 2; ++side)
        if (sender_->due(side, {}, 0)) send(side, {});
    off_ = true;
}

void PadTriggers::shutdown() {
    if (!sink_) return;
    if (sink_->present() && (sent_[0].mode != trig::Command::Off || sent_[1].mode != trig::Command::Off)) {
        send(0, {}), send(1, {});
        sender_->forget();
        usleep(30000);  // let the output report leave before the process goes
    }
    off_ = true;
}

}  // namespace fuel

// ---- --trigger-check ----

namespace fuel {

namespace {

// A controller that records what it is told.
struct RecordSink final : trig::Sink {
    bool here = true, dirty = true;
    trig::Link l = trig::Link::Usb;
    trig::Command last[2];
    std::vector<double> times[2];  // send times
    std::vector<trig::Command> log[2];
    double now = 0;
    bool present() override { return here; }
    bool changed() override {
        const bool d = dirty;
        dirty = false;
        return d;
    }
    trig::Link link() override { return l; }
    std::string name() override { return "check"; }
    void send(int side, const trig::Command& c) override {
        if (!here) return;
        last[side] = c;
        times[side].push_back(now);
        log[side].push_back(c);
    }
};

bool gameOff(const trig::Telemetry& t) { return t.paused || t.menu || t.cutscene || t.loading; }

struct PhaseStats {
    int frames = 0;
    float vMin = 1e9f, vMax = -1e9f;
    float tGripMin = 1, bGripMin = 1, spinMax = 0, lockMax = 0, surfMin = 9, surfMax = 0, impactMax = 0;
    int spinning = 0, locked = 0, airborne = 0, sends = 0;
    std::map<std::string, int> modes[2];
    trig::Command first[2], last[2], weakest[2];
    float weakestEnd[2] = {9, 9};
    bool haveFirst = false;
};

struct Run {
    std::vector<std::string> order;
    std::map<std::string, PhaseStats> phases;
    int violations = 0;   // on while paused / menu / cut-scene / loading
    int maxPerSecond[2] = {0, 0};
    int totalSends = 0;
    bool offAfterQuit = false, offAfterReconnect = true;
    std::vector<trig::Command> state[2];  // the device state per frame
    int gearClicks = 0, kicks = 0;
};

Run runFrames(const std::vector<TriggerCheckFrame>& frames, const char* level, trig::Link link, const char* extras, bool trace) {
    Settings s;
    s.all["adaptive_triggers"] = level;
    if (extras) s.all["adaptive_triggers_extras"] = extras;
    auto owned = std::make_unique<RecordSink>();
    RecordSink* sink = owned.get();
    sink->l = link;
    Run r;
    {
        PadTriggers pt(s, std::move(owned));
        double clock = 0;
        std::string prevPhase;
        bool reconnecting = false;
        for (const TriggerCheckFrame& f : frames) {
            clock += f.dt;
            sink->now = clock;
            if (f.phase != prevPhase) {
                if (prevPhase == "disconnect") sink->here = true, sink->dirty = true, reconnecting = true;
                if (f.phase == "disconnect") sink->here = false, sink->dirty = true;
                prevPhase = f.phase;
                if (!r.phases.count(f.phase)) r.order.push_back(f.phase);
            }
            const size_t before[2] = {sink->log[0].size(), sink->log[1].size()};
            if (f.t.paused || f.t.menu) pt.off();  // main.mm: the pause branch returns before the frame
            else pt.frame(f.dt, f.t);
            if (reconnecting) {  // the first thing a reconnected pad hears is off
                for (int side = 0; side < 2; ++side)
                    if (sink->log[side].size() > before[side] && sink->log[side][before[side]].mode != trig::Command::Off) r.offAfterReconnect = false;
                reconnecting = false;
            }
            PhaseStats& p = r.phases[f.phase];
            const trig::Feel& fe = pt.model().feel();
            ++p.frames;
            p.vMin = std::min(p.vMin, f.t.speed * 3.6f), p.vMax = std::max(p.vMax, f.t.speed * 3.6f);
            if (!gameOff(f.t) && f.phase != "disconnect") {
                p.tGripMin = std::min(p.tGripMin, fe.throttleGrip), p.bGripMin = std::min(p.bGripMin, fe.brakeGrip);
                p.spinMax = std::max(p.spinMax, fe.spin), p.lockMax = std::max(p.lockMax, fe.lock);
                p.surfMin = std::min(p.surfMin, fe.surface), p.surfMax = std::max(p.surfMax, fe.surface);
                p.spinning += fe.spinning, p.locked += fe.locked, p.airborne += fe.airborne;
            }
            p.impactMax = std::max(p.impactMax, f.t.impactSpeed);
            p.sends += int(sink->log[0].size() - before[0] + sink->log[1].size() - before[1]);
            for (int side = 0; side < 2; ++side) {
                const trig::Command& c = sink->last[side];
                r.state[side].push_back(c);
                ++p.modes[side][c.describe().substr(0, c.describe().find(' '))];
                if (!p.haveFirst) p.first[side] = c;
                p.last[side] = c;
                const float e = c.mode == trig::Command::Slope ? c.endStrength : c.mode == trig::Command::Off ? 9.0f : 0.0f;
                if (e < p.weakestEnd[side]) p.weakestEnd[side] = e, p.weakest[side] = c;
                if (gameOff(f.t) && sink->here && c.mode != trig::Command::Off) {
                    if (r.violations < 5) std::printf("  FAIL [%s] %s: %s is on during %s: %s\n", level, f.phase.c_str(), side ? "R2" : "L2", f.phase.c_str(), c.describe().c_str());
                    ++r.violations;
                }
                for (size_t k = before[side]; k < sink->log[side].size(); ++k) {
                    const trig::Command& sc = sink->log[side][k];
                    if (sc.mode == trig::Command::Vibration && std::fabs(sc.frequency - 0.6f) < 1e-3f && side == 1) ++r.gearClicks;
                    if (sc.mode == trig::Command::Vibration && std::fabs(sc.frequency - 0.3f) < 1e-3f && side == 1) ++r.kicks;
                }
            }
            p.haveFirst = true;
            if (trace) std::printf("    %-16s v %6.1f  L2 %-44s R2 %-44s grip T %.2f B %.2f spin %.2f lock %.2f surf %.2f\n", f.phase.c_str(), f.t.speed * 3.6f,
                                   sink->last[0].describe().c_str(), sink->last[1].describe().c_str(), fe.throttleGrip, fe.brakeGrip, fe.spin, fe.lock, fe.surface);
        }
        pt.shutdown();  // quit
        r.offAfterQuit = sink->last[0].mode == trig::Command::Off && sink->last[1].mode == trig::Command::Off;
        for (int side = 0; side < 2; ++side) {  // (the sink goes with `pt`)
            r.totalSends += int(sink->times[side].size());
            const auto& tv = sink->times[side];
            for (size_t a = 0, b = 0; b < tv.size(); ++b) {  // most sends in any 1 s window
                while (tv[b] - tv[a] >= 1.0) ++a;
                r.maxPerSecond[side] = std::max(r.maxPerSecond[side], int(b - a + 1));
            }
        }
    }
    return r;
}

}  // namespace

int triggerCheck(const std::vector<TriggerCheckFrame>& frames, const std::string& vehicle) {
    int fails = 0;
    auto fail = [&](const std::string& m) {
        std::printf("  FAIL %s\n", m.c_str());
        ++fails;
    };
    const bool trace = std::getenv("FUEL_TRIGGER_TRACE") != nullptr;
    std::printf("trigger check: %s, %zu frames (%.1f s)\n", vehicle.c_str(), frames.size(), frames.size() / 60.0);
    std::printf("link detection (IORegistry, nothing sent): %s\n", trig::detectedLink());

    // 1. full over USB (the default): per phase.
    const Run full = runFrames(frames, "full", trig::Link::Usb, nullptr, trace);
    std::printf("\nadaptive_triggers = full, USB: modes per phase (L2 = brake, R2 = throttle)\n");
    for (const std::string& name : full.order) {
        const PhaseStats& p = full.phases.at(name);
        std::printf("  %-16s %4d frames  %5.0f..%-5.0f km/h  grip left T %.2f B %.2f  spin %.2f lock %.2f  surface %.2f..%.2f  air %d  sends %d\n",
                    name.c_str(), p.frames, p.vMin, p.vMax, p.tGripMin, p.bGripMin, p.spinMax, p.lockMax, p.surfMin > 8 ? 0.0f : p.surfMin, p.surfMax,
                    p.airborne, p.sends);
        for (int side = 1; side >= 0; --side) {
            std::string modes;
            for (const auto& [m, n] : p.modes[side]) modes += (modes.empty() ? "" : ", ") + m + " " + std::to_string(n);
            std::printf("      %s  %-12s first %-44s weakest %-44s last %s\n", side ? "R2" : "L2", modes.c_str(), p.first[side].describe().c_str(),
                        p.weakestEnd[side] > 8 ? "-" : p.weakest[side].describe().c_str(), p.last[side].describe().c_str());
        }
    }
    auto phase = [&](const char* n) -> const PhaseStats* {
        const auto it = full.phases.find(n);
        return it == full.phases.end() ? nullptr : &it->second;
    };
    // Hard rules.
    if (full.violations) fail(std::to_string(full.violations) + " frames with a trigger on during pause / menu / cut-scene / loading");
    if (!full.offAfterQuit) fail("triggers not off after quit");
    if (!full.offAfterReconnect) fail("a reconnected controller did not start from off");
    if (full.maxPerSecond[0] > 61 || full.maxPerSecond[1] > 61) fail("USB above 60 updates/s");
    for (const char* n : {"pause", "menu", "cutscene", "loading"})
        if (!phase(n)) fail(std::string("no ") + n + " phase");
    // The feel the drive should show.
    if (const PhaseStats* p = phase("idle"))
        if (p->last[0].mode != trig::Command::Slope || p->last[1].mode != trig::Command::Slope) fail("idle: both triggers should hold their progressive slope");
    if (const PhaseStats* p = phase("wheelspin start"))
        if (!(p->spinning > 0 || p->tGripMin < 0.6f)) fail("wheelspin start: the throttle never gave way");
    if (const PhaseStats* p = phase("gravel"); p && phase("idle"))
        if (!(p->surfMin < phase("idle")->surfMax - 0.08f)) fail("gravel: the surface ceiling did not drop below tarmac's");
    if (const PhaseStats* p = phase("tarmac"))
        if (!(p->surfMax > 0.95f)) fail("tarmac: the surface ceiling did not come back");
    if (const PhaseStats* p = phase("jump"))
        if (p->airborne == 0) fail("jump: never airborne");
    if (const PhaseStats* p = phase("lock-up stop"))
        if (p->locked == 0) fail("lock-up stop: the brake never showed lock-up");
    if (const PhaseStats* p = phase("resume"))
        if (p->last[1].mode == trig::Command::Off) fail("resume: the throttle stayed off after the pause");
    std::printf("  USB: %d commands, at most %d / %d per second (L2 / R2)\n", full.totalSends, full.maxPerSecond[0], full.maxPerSecond[1]);

    // 2. Bluetooth: 10 updates a second, 5 bands.
    const Run bt = runFrames(frames, "full", trig::Link::Bluetooth, nullptr, false);
    std::printf("\nBluetooth: %d commands, at most %d / %d per second; lock-up frames %d, spin frames %d\n", bt.totalSends, bt.maxPerSecond[0], bt.maxPerSecond[1],
                bt.phases.count("lock-up stop") ? bt.phases.at("lock-up stop").locked : 0, bt.phases.count("wheelspin start") ? bt.phases.at("wheelspin start").spinning : 0);
    if (bt.violations) fail("Bluetooth: triggers on during pause / menu / cut-scene / loading");
    if (bt.maxPerSecond[0] > 11 || bt.maxPerSecond[1] > 11) fail("Bluetooth above 10 updates/s");
    if (!bt.offAfterQuit) fail("Bluetooth: not off after quit");

    // 3. light: never stronger than full.
    const Run light = runFrames(frames, "light", trig::Link::Usb, nullptr, false);
    int stronger = 0;
    for (int side = 0; side < 2; ++side)
        for (size_t i = 0; i < light.state[side].size() && i < full.state[side].size(); ++i) {
            const trig::Command &a = light.state[side][i], &b = full.state[side][i];
            if (a.mode == trig::Command::Slope && b.mode == trig::Command::Slope && a.endStrength > b.endStrength + 1e-4f) ++stronger;
            if (a.mode == trig::Command::Vibration && b.mode == trig::Command::Vibration && a.amplitude > b.amplitude + 1e-4f) ++stronger;
        }
    std::printf("light: %d commands; %d frames stronger than full\n", light.totalSends, stronger);
    if (stronger) fail("light stronger than full");
    if (light.violations) fail("light: triggers on during pause / menu / cut-scene / loading");

    // 4. off: nothing but off.
    const Run off = runFrames(frames, "off", trig::Link::Usb, nullptr, false);
    int on = 0;
    for (int side = 0; side < 2; ++side)
        for (const auto& c : off.state[side]) on += c.mode != trig::Command::Off;
    std::printf("off: %d commands, %d frames on\n", off.totalSends, on);
    if (on) fail("adaptive_triggers = off still sends effects");

    // 5. extras (all): what the secondary effects add (they are off by default).
    const Run ex = runFrames(frames, "full", trig::Link::Usb, "all", false);
    std::printf("extras = all: %d commands, gear clicks %d, impact kicks %d, airborne slack frames %d\n", ex.totalSends, ex.gearClicks, ex.kicks,
                ex.phases.count("jump") ? ex.phases.at("jump").modes[1].count("off") ? ex.phases.at("jump").modes[1].at("off") : 0 : 0);
    if (ex.violations) fail("extras: triggers on during pause / menu / cut-scene / loading");

    // 6. The progressive slopes per vehicle class (tarmac, full grip, standing).
    std::printf("\nbase slopes by class (full; light = x %.2f):\n", trig::lightScale());
    for (int k = 0; k < 6; ++k) {
        trig::Config c;
        trig::Model m(c);
        trig::Telemetry t;
        t.haveVehicle = true, t.vehicleClass = k;
        for (int w = 0; w < 4; ++w) {
            trig::Telemetry::Wheel x;
            x.loaded = true, x.driven = w == 1 || w == 3, x.brakeTorque = 1, x.util = 0.2f, x.surface = 1;
            t.wheels.push_back(x);
        }
        m.update(t, 1.0f / 60, trig::Role::Brake, trig::Role::Throttle);
        std::printf("  %-6s  R2 %-40s L2 %s\n", trig::className(k), m.right().describe().c_str(), m.left().describe().c_str());
    }
    // 7. [nitro] boosting (Telemetry::nitro): R2 buzzes lightly past a tenth of its travel (full 0.20 x the amount, light
    // x lightScale, Bluetooth in 0.05 steps), L2 keeps its slope, wheelspin still wins, amount 0 is the plain slope.
    {
        auto run = [&](float nitro, trig::Level level, trig::Link link, bool spin) {
            trig::Config c;
            c.level = level, c.link = link;
            trig::Model m(c);
            trig::Telemetry t;
            t.haveVehicle = true, t.vehicleClass = 0, t.nitro = nitro, t.speed = 25;
            for (int w = 0; w < 4; ++w) {
                trig::Telemetry::Wheel x;
                x.loaded = true, x.driven = w == 1 || w == 3, x.brakeTorque = 1, x.util = spin && x.driven ? 3.0f : 0.2f, x.surface = 1;
                x.rim = spin && x.driven ? 60.0f : 25.0f;
                t.wheels.push_back(x);
            }
            for (int i = 0; i < 30; ++i) m.update(t, 1.0f / 60, trig::Role::Brake, trig::Role::Throttle);
            return std::make_pair(m.right(), m.left());
        };
        const auto [r1, l1] = run(1, trig::Level::Full, trig::Link::Usb, false);
        const auto [rh, lh] = run(0.5f, trig::Level::Full, trig::Link::Bluetooth, false);
        const auto [rl, ll] = run(1, trig::Level::Light, trig::Link::Usb, false);
        const auto [r0, l0] = run(0, trig::Level::Full, trig::Link::Usb, false);
        const auto [rs, ls] = run(1, trig::Level::Full, trig::Link::Usb, true);
        const auto [ro, lo] = run(1, trig::Level::Off, trig::Link::Usb, false);
        std::printf("\n[nitro] boosting: R2 %s; half, Bluetooth %s; light %s; no boost %s; spinning %s; off %s; L2 %s\n", r1.describe().c_str(),
                    rh.describe().c_str(), rl.describe().c_str(), r0.describe().c_str(), rs.describe().c_str(), ro.describe().c_str(), l1.describe().c_str());
        const bool ok = r1.mode == trig::Command::Vibration && std::fabs(r1.amplitude - 0.2f) < 1e-4f && std::fabs(r1.start - 0.1f) < 1e-4f &&
                        rh.mode == trig::Command::Vibration && std::fabs(rh.amplitude - 0.1f) < 1e-4f && rl.mode == trig::Command::Vibration &&
                        std::fabs(rl.amplitude - 0.2f * trig::lightScale()) < 1e-4f && r0.mode == trig::Command::Slope &&
                        (rs.mode != trig::Command::Vibration || std::fabs(rs.frequency - 0.55f) > 1e-4f) && ro.mode == trig::Command::Off &&
                        l1.mode == l0.mode;
        if (!ok) fail("[nitro] the boost's throttle buzz");
    }
    std::printf("\ntrigger check: %d failures\n", fails);
    return fails;
}

int triggerDriveCheck(DriveSession& d, const std::string& vehicle, const std::function<bool(double x, double z, float yaw)>& addObstacle,
                      const std::function<float(double x, double z, float heading)>& steer) {
    // The ground under the wheels, scripted: tarmac (class 0) or loose stones (class 4: static 0.6, gravel sounds 0.5).
    int groundClass = 0;
    d.physics().setClassQuery([&groundClass](double, double, double, float* frac, float& depth) {
        for (int k = 0; k < fv::kGroundClasses; ++k) frac[k] = 0;
        frac[groundClass] = 1;
        depth = 0;
    });
    const fv::FuelVehicle* m = d.physics().model(0);
    if (!m) {
        std::printf("trigger check: no vehicle\n");
        return 1;
    }
    const int cls = d.setup(0).vehicleClass;
    std::vector<TriggerCheckFrame> frames;
    const float dt = 1.0f / 60;
    trig::Telemetry last;
    auto step = [&](const char* phase, DriveInput in) {
        if (steer) {
            const simd_float3 p = d.position();
            in.steer = steer(p.x, p.z, d.heading());
        }
        d.update(dt, in);
        TriggerCheckFrame f;
        f.phase = phase;
        f.t = trig::telemetryFrom(*d.physics().model(0), d.state(), cls, in.brake);
        last = f.t;
        if (const char* tr = std::getenv("FUEL_TRIGGER_TRACE"); tr && *tr == '2') {
            std::printf("  %-16s v %6.2f gear %d ground %d |", phase, f.t.speed, f.t.gear, d.state().wheelsOnGround);
            for (const auto& w : f.t.wheels) std::printf(" %c%c u %5.2f rim %6.2f s %.2f |", w.loaded ? 'L' : '-', w.driven ? 'D' : '-', w.util, w.rim, w.surface);
            std::printf("\n");
        }
        frames.push_back(f);
    };
    auto hold = [&](const char* phase, float seconds, bool trig::Telemetry::*flag) {
        for (int k = 0; k < int(seconds * 60); ++k) {
            TriggerCheckFrame f;
            f.phase = phase;
            f.t = last;
            if (flag) f.t.*flag = true;
            frames.push_back(f);
        }
    };
    DriveInput gas;
    gas.throttle = 1;
    DriveInput brake;
    brake.brake = 1;
    auto envInt = [](const char* k, int def) { const char* e = std::getenv(k); return e ? std::atoi(e) : def; };
    // Ground classes (FUEL's 18; names unknown, p9): 0 tarmac (static 1.0), 8 mud (0.5), 4 loose stones (0.6, gravel
    // sounds 0.5) for the "gravel".
    const int launchClass = envInt("FUEL_TRIGGER_LAUNCH_CLASS", 8), looseClass = envInt("FUEL_TRIGGER_LOOSE_CLASS", 4);
    auto where = [&](const char* phase) {
        const simd_float3 p = d.position();
        std::printf("  drive: %-16s ground class %2d, ends at (%.0f, %.0f) %5.1f km/h\n", phase, groundClass, p.x, p.z, d.state().speedKmh);
    };
    // Held on the handbrake at the start (it is on a slope; the brake pedal would reverse), then a full-throttle launch
    // on mud, a tarmac stretch, and loose stones.
    DriveInput park;
    park.handbrake = true;
    for (int k = 0; k < 90; ++k) step("idle", park);
    where("idle");
    groundClass = launchClass;
    for (int k = 0; k < 150; ++k) step("wheelspin start", gas);
    where("wheelspin start");
    groundClass = 0;
    for (int k = 0; k < 90; ++k) step("tarmac", gas);
    where("tarmac");
    groundClass = looseClass;
    for (int k = 0; k < 180; ++k) step("gravel", gas);
    where("gravel");
    groundClass = 0;
    {  // a jump: lifted 2.5 m, thrown forward and up
        const auto v = d.state().velocity;
        d.physics().nudge(0, 2.5, {v[0], 5.0f, v[2]});
        int landed = -1;
        for (int k = 0; k < 240 && (landed < 0 || k < landed + 30); ++k) {
            step("jump", gas);
            if (k > 10 && landed < 0 && d.state().wheelsOnGround >= 2) landed = k;
        }
    }
    where("jump");
    for (int k = 0; k < 60; ++k) step("lock-up stop", gas);  // back to speed on tarmac, then brake hard on loose ground
    groundClass = looseClass;
    for (int k = 0; k < 360 && std::fabs(d.state().speedKmh) > 2; ++k) step("lock-up stop", brake);
    groundClass = 0;
    where("lock-up stop");
    {  // an impact: a parked vehicle 15 m ahead
        const simd_float3 p = d.position();
        const float h = d.heading();
        const bool placed = addObstacle && addObstacle(p.x + std::sin(h) * 15.0, p.z + std::cos(h) * 15.0, h + 1.5708f);
        if (!placed) std::printf("trigger check: no obstacle placed (the impact phase drives on)\n");
        float hit = 0;
        for (int k = 0; k < 360; ++k) {
            DriveInput straight = gas;  // straight at the obstacle (no steering)
            straight.steer = 0;
            d.update(dt, straight);
            TriggerCheckFrame f;
            f.phase = "impact";
            f.t = trig::telemetryFrom(*d.physics().model(0), d.state(), cls, 0);
            last = f.t;
            frames.push_back(f);
            hit = std::max(hit, d.state().impactSpeed);
            if (hit > 0 && k > 30) {
                for (int j = 0; j < 30; ++j) step("impact", {});
                break;
            }
        }
        std::printf("  drive: impact %.1f m/s\n", hit);
    }
    hold("pause", 1.0f, &trig::Telemetry::paused);
    hold("menu", 1.0f, &trig::Telemetry::menu);
    hold("cutscene", 1.0f, &trig::Telemetry::cutscene);
    hold("loading", 0.5f, &trig::Telemetry::loading);
    hold("disconnect", 0.5f, nullptr);
    for (int k = 0; k < 90; ++k) step("resume", gas);
    return triggerCheck(frames, vehicle);
}

}  // namespace fuel
