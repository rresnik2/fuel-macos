#include "game_settings.h"

#include "ui/menu.h"
#include "audio/sound_mixer.hpp"
#include "physics/fuel_vehicle.hpp"
#include "physics/nitro.hpp"  // [nitro]
#include "core/upgrades.hpp"  // [upgrades U1]

#include <algorithm>
#include <cstdio>
#include <cstdlib>

float gGameVolume = fuel::kFuelDefaultVolume;  // P103: FUEL's SFX volume default (8 / 16) until applySettings runs

void applySettings(const fuel::Settings& s, fuel::MeshRenderer& renderer) {
    renderer.postEffects = s.postEffects;
    renderer.grass = s.grass;
    gGameVolume = std::clamp(s.volume, 0.0f, 1.0f);
    // P62: the Brightness option (slider p in settings.txt `brightness_slider`; the older `brightness` key was
    // saved but never applied, so it is ignored rather than read as a slider position): World_SetBrightness(0.5 + 1.5 p) (0x495540).
    renderer.brightness = 0.5f + 1.5f * fuel::ui::loadMenuOptions().brightness;
    // P90 [DEVIATION]: `wind_debris = smooth | fuel` (read only; default smooth, core/wind_tank.hpp DebrisMotion).
    if (const auto it = s.all.find("wind_debris"); it != s.all.end()) {
        renderer.windDebris = fuel::storm::debrisMotionFrom(it->second, renderer.windDebris);
        std::fprintf(stderr, "fuelview: settings: wind_debris %s\n", fuel::storm::debrisMotionName(renderer.windDebris));
    }
    // P93: `dynamic_resolution = on|off` (default on), `dynamic_resolution_min` (default 0.8, 0.5 .. 1) and
    // `dynamic_resolution_upscaler = sharpen|metalfx|metalfx-temporal` (default sharpen; TAA: metalfx-temporal),
    // `shadow_filter = 2x2|3x3` (default 2x2), read only (the launcher keeps unknown lines when it saves).
    auto flag = [&](const char* key, bool def) {
        const auto it = s.all.find(key);
        if (it == s.all.end()) return def;
        const std::string& v = it->second;
        if (v == "on" || v == "1" || v == "true" || v == "yes") return true;
        if (v == "off" || v == "0" || v == "false" || v == "no") return false;
        return def;
    };
    // P100 [DEVIATION]: `audio_limiter = on|off` (default on; read only): the final mix's smooth limiter instead of the hard clip.
    fuel::snd::Mixer::setLimiterDefault(flag("audio_limiter", true));
    renderer.dynamicResolution = flag("dynamic_resolution", true);
    if (const auto it = s.all.find("dynamic_resolution_min"); it != s.all.end()) {
        const float m = std::strtof(it->second.c_str(), nullptr);
        if (m > 0) renderer.dynamicResolutionParams.minScale = std::clamp(m, 0.5f, 1.0f);
    }
    if (const auto it = s.all.find("dynamic_resolution_upscaler"); it != s.all.end())
        renderer.upscaler = fuel::MeshRenderer::upscalerFromName(it->second, fuel::MeshRenderer::Upscaler::Sharpen);
    // P102 [DEVIATION, the user's request]: `film_grain = off|light|strong` (default light), read only.
    renderer.filmGrain = fuel::MeshRenderer::kFilmGrainLight;
    if (const auto it = s.all.find("film_grain"); it != s.all.end())
        renderer.filmGrain = fuel::MeshRenderer::filmGrainAmplitude(it->second, renderer.filmGrain);
    // `soften = off|light|strong` (default off), read only.
    renderer.soften = 0;
    if (const auto it = s.all.find("soften"); it != s.all.end()) renderer.soften = fuel::MeshRenderer::softenAmount(it->second, 0);
    std::fprintf(stderr, "fuelview: settings: film grain %.3f, soften %.2f\n", renderer.filmGrain, renderer.soften);
    // [DEVIATION, the user's request]: `engine_power = 1.25` (default 1 = FUEL's torque curves): all vehicles' engine
    // torque x this (FuelVehicle::powerScale), clamped to 0.5..2.
    fuel::fv::FuelVehicle::powerScale = 1.0f;
    if (const auto it = s.all.find("engine_power"); it != s.all.end()) {
        const float p = std::strtof(it->second.c_str(), nullptr);
        if (p > 0) fuel::fv::FuelVehicle::powerScale = std::clamp(p, 0.5f, 2.0f);
    }
    std::fprintf(stderr, "fuelview: settings: engine power x%.2f\n", fuel::fv::FuelVehicle::powerScale);
    // [DEVIATION, the user's request 2026-11-06]: `nitrous = on|off` (default on; FUEL_NITRO overrides): the port's nitrous
    // (physics/nitro.hpp). Off: no tanks, no AI use, no gauge, no effects (the game as before).
    {
        const auto it = s.all.find("nitrous");
        fuel::nitro::setEnabled(fuel::nitro::enabledFrom(it == s.all.end() ? nullptr : it->second.c_str(), true));
        std::fprintf(stderr, "fuelview: settings: nitrous %s\n", fuel::nitro::enabled() ? "on" : "off");
    }
    // [DEVIATION, the user's request 2026-11-06]: `upgrades = on|off` (default on; FUEL_UPGRADES=off overrides): the vehicle
    // upgrade stages (core/upgrades.hpp). Off: every vehicle stock (FUEL's), the stages kept.
    {
        const auto it = s.all.find("upgrades");
        fuel::upg::setEnabled(fuel::upg::enabledFrom(it == s.all.end() ? nullptr : it->second.c_str(), true));
        std::fprintf(stderr, "fuelview: settings: upgrades %s\n", fuel::upg::enabled() ? "on" : "off");
    }
    renderer.shadow2x2 = true;
    if (const auto it = s.all.find("shadow_filter"); it != s.all.end()) renderer.shadow2x2 = it->second != "3x3";
    std::fprintf(stderr, "fuelview: settings: dynamic resolution %s (min %.2f, upscaler %s), shadow filter %s\n", renderer.dynamicResolution ? "on" : "off",
                 renderer.dynamicResolutionParams.minScale, fuel::MeshRenderer::upscalerName(renderer.upscaler), renderer.shadow2x2 ? "2x2" : "3x3");
    std::fprintf(stderr, "fuelview: settings: %dx%d%s, post effects %s, grass %s, volume %.2f, language %s\n", s.width, s.height,
                 s.fullscreen ? " fullscreen" : "", s.postEffects ? "on" : "off", s.grass ? "on" : "off", gGameVolume, s.language.c_str());
}

NSSize gameWindowSize(const fuel::Settings& s) {
    NSSize size = NSMakeSize(std::clamp(s.width, 640, 16384), std::clamp(s.height, 400, 16384));
    // A window never opens bigger than the usable screen (the native size is for fullscreen).
    const NSRect usable = NSScreen.mainScreen.visibleFrame;
    size.width = std::min(size.width, usable.size.width);
    size.height = std::min(size.height, usable.size.height - 30);  // title bar
    return size;
}

void applyWindowSettings(const fuel::Settings& s, NSWindow* window, MTKView* view) {
    if (!s.fullscreen) return;
    [window toggleFullScreen:nil];
    // A resolution below the screen's renders at that size and is scaled up.
    const NSSize screenPx = [window.screen convertRectToBacking:window.screen.frame].size;
    if (s.width > 0 && s.height > 0 && (s.width < screenPx.width || s.height < screenPx.height)) {
        view.autoResizeDrawable = NO;
        view.drawableSize = CGSizeMake(s.width, s.height);
    }
}

void applyLiveSettings(const fuel::Settings& s, float brightnessSlider, fuel::MeshRenderer& renderer) {
    applySettings(s, renderer);
    renderer.brightness = 0.5f + 1.5f * brightnessSlider;  // the slider as shown (applySettings reads the saved one)
}

void applyFullscreenLive(bool on, NSWindow* window, MTKView* view) {
    if (!window) return;
    const bool now = (window.styleMask & NSWindowStyleMaskFullScreen) != 0;
    if (now == on) return;
    view.autoResizeDrawable = YES;
    [window toggleFullScreen:nil];
    std::fprintf(stderr, "fuelview: settings: full screen %s (options page)\n", on ? "on" : "off");
}
