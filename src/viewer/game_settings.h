// Applies the player's settings file (core/settings.hpp, written by the FUEL.app
// launcher) to a fuelview window run. Kept out of main.mm so the game's menus
// (--game) can call it with one line: applySettings(...) before the window is
// built, applyWindowSettings(...) once it is on screen.
//
// Honoured: width/height (window size; render size in fullscreen), fullscreen,
// post_effects, grass, volume. data is read by main.mm, language by the menus.
#pragma once

#import <Cocoa/Cocoa.h>
#import <MetalKit/MetalKit.h>

#include "core/settings.hpp"
#include "renderer.h"

// Master volume (0..1) for every DriveAudio the window creates; 1 until applySettings runs.
extern float gGameVolume;

// Renderer-side settings: post effects and grass. Also sets gGameVolume.
void applySettings(const fuel::Settings& s, fuel::MeshRenderer& renderer);

// The window size the settings ask for, in points, clamped to something sane.
NSSize gameWindowSize(const fuel::Settings& s);

// Window-side settings, after makeKeyAndOrderFront: fullscreen (render at width x height
// when that is smaller than the screen).
void applyWindowSettings(const fuel::Settings& s, NSWindow* window, MTKView* view);
