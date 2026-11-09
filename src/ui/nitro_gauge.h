// [DEVIATION, the user's request 2026-11-06] The nitrous gauge beside FUEL's speedometer (notes nitro.md): a small
// upright bottle right of the compteur dial (hud::nitroBottle), filled with blue nitrous liquid to the player's tank
// level. Smooth fill (hud::nitroShown), a gentle shimmer and a rippling surface (stronger and with bubbles while
// boosting or just after a bonus), a soft blue glow when full, a dim flash when empty. In FUEL's HUD style: the
// dial's orange outline and tick marks (one per second of boost), the dark glass behind.
#pragma once

#include "ui/menu.h"

namespace fuel::ui {

class UiRenderer;

// Draws the gauge for the dial picture at (dialX, dialY) of dialW x dialH canvas units (Menu::drawMeter); nothing when
// the HUD's nitro is off.
void drawNitroGauge(UiRenderer& r, const HudState& hud, float dialX, float dialY, float dialW, float dialH);

}  // namespace fuel::ui
