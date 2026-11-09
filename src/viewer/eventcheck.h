// `fuelview --event-check`: every event of the catalogue, briefly. The player spawns on the
// start, the rules count down, then the player hops through the gates (as FUEL_RACE_WARP does;
// onto the helicopter's track or the hunted racers where the mode asks for it) with the vehicle
// physics running between hops. One line per event and a total; exit code 0 when every playable
// event is won through its finish trigger.
#pragma once

#include <functional>

#include "core/events.hpp"
#include "core/terrain.hpp"
#include "drive.h"

int runEventCheck(const std::vector<fuel::Event>& events, DriveSession& drive, WorldStreamer& streamer, fuel::MeshRenderer& renderer,
                  const fuel::Heightfield& terrain, const std::string& only,
                  const std::function<float(float, float)>& gateWidth = {});  // 0x4fdea0 (p38); empty: 25 m gates
