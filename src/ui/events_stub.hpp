// STUB until P17's fuel::loadEvents (core/events.hpp) is merged: builds the event
// list from hubinfos.tsc (AddHubCareerRaces / AddHubCHallengeRaces with their name
// texts), AddMissionNeededVehicle (vehicle family) and the EMD race geometry
// (core/races.hpp). Delete this file and events_stub.cpp, and call fuel::loadEvents
// in menu.mm (the one line marked "P17"), once P17 lands.
#pragma once

#include "core/events.hpp"

#include <string>
#include <vector>

namespace fuel::ui {

std::vector<Event> loadEventsStub(const std::string& dataDir);

}  // namespace fuel::ui
