#include "game_settings.h"

#include <algorithm>
#include <cstdio>

float gGameVolume = 1.0f;

void applySettings(const fuel::Settings& s, fuel::MeshRenderer& renderer) {
    renderer.postEffects = s.postEffects;
    renderer.grass = s.grass;
    gGameVolume = std::clamp(s.volume, 0.0f, 1.0f);
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
