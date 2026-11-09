// CUSTOM VEHICLES (port deviation, user-requested): a synthesized electric-motor whine for the electric custom vehicles
// (pack line `PortVehicleSynth EV`). FUEL has no electric vehicle; the pack borrows a FUEL variant's bank for tyres,
// wind, impacts and suspension, and this replaces that bank's engine samples (idle 2, off-load 3..7, on-load 9..13)
// with generated tones, so FUEL's own layer mixer (0x5d98c0) pitches and blends them with the motor speed exactly
// as it does for an engine. Nothing is recorded or stored: the samples are computed when the bank loads.
#pragma once

#include "audio/vehicle_sound.hpp"

namespace fuel::snd {

// Each layer L's loop holds the motor orders at the layer's native rpm (EngineModel::nativeRpm, P636..P642): a
// fundamental at 4 x the shaft frequency (inverter / pole pass), the gear mesh at 11 x (softer) and a low hum at 2 x;
// off-load layers lose most of the inverter tone. Loops are one second long with whole-hertz components (seamless).
// The idle sample is a faint hum. Backfire samples are removed (no exhaust).
void synthesizeEvBank(VehicleBank& bank);

}  // namespace fuel::snd
