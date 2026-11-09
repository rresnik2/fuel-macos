// [port] DEVIATION from FUEL (user request 2026-11-06, notes ~/re-work/fuel/notes/pad-ux.md): the connected pad's own
// button prompts. The GameController side and the picture side of ui/pad_prompts.hpp:
//  - detect(): the pad the game reads (GCController.controllers, the first with an extended gamepad) -> its family
//    (GCDualSenseGamepad / GCDualShockGamepad = PlayStation, GCXboxGamepad and anything else = Xbox style) and the
//    elements' sfSymbolsName / localizedName; settings.txt pad_glyphs forces a family, $FUEL_PAD_FAMILY too (testing);
//  - atlas(): Apple's SF Symbols rasterized with NSImage into one Metal texture for the current family (rebuilt when the
//    family or the pad changes; nothing is read from or written to disk);
//  - paddleCheck(): the port's PaddleChecker_G (0x4e36e0) for every host (menus, session overlay): presence + family ->
//    the {STR_*} labels rebuilt (ui/help_texts.*), UiRenderer::padIcons.
#pragma once

#import <Metal/Metal.h>

#include "ui/help_texts.hpp"
#include "ui/pad_prompts.hpp"

#include <functional>
#include <map>
#include <string>

namespace fuel::ui::pad {

// The prompt state from GameController (`style` = pad_glyphs).
State detect(Style style);
// The state a --menu-shot uses: $FUEL_SHOT_PAD unset or "0" -> no pad; "1" -> a pad of `style` (auto: Xbox);
// "playstation" / "ps" / "dualsense" or "xbox" -> a pad of that family.
State shotState(Style style);

// One picture per icon in a texture (straight alpha, sRGB, mipmapped). Cell rectangles in texels; `centre` = the
// vertical centre of the drawn shape in the cell (texels from the cell top), `ref` = the texel height of a face button
// (the size every icon is scaled by).
struct Atlas {
    id<MTLTexture> texture = nil;
    float width = 1, height = 1, ref = 1;
    struct Cell {
        bool ok = false;
        float x0 = 0, y0 = 0, x1 = 0, y1 = 0, centre = 0;
    } cells[kIconCount];
    int serial = -1;  // State::serial it was built for
    int symbols = 0, pills = 0, missing = 0;  // how the cells were drawn (checks)
};
// The atlas for the current state on `device` (built on first use, rebuilt when State::serial moves). Thread-safe.
const Atlas* atlas(id<MTLDevice> device);
// --menu-shot GLYPHS / --menu-check: writes the current atlas as a PNG.
bool writeAtlasPng(id<MTLDevice> device, const std::string& path);

// The port's PaddleChecker: `s` -> State, then the help texts rebuilt when the pad comes, goes or changes family;
// UiRenderer::padIcons follows. Returns true when the labels were rebuilt.
bool paddleCheck(const State& s, const std::function<std::string(int)>& keyLabels, const HelpTexts::PadLabelFn& padLabels);

}  // namespace fuel::ui::pad

namespace fuel::ui {
// pad-ux --menu-check: both families' atlases (every icon drawn, symbols / pills), and the prompts as glyphs of FUEL's
// fonts (a pad-prompt character and FUEL's glyph bytes in pad mode give the same picture; the key caps stay without a pad;
// a face button is as tall as FUEL's old A icon x 0.92 and shares its vertical centre). Returns the failure count.
int runPadAtlasCheck(const std::string& dataDir);
}  // namespace fuel::ui
