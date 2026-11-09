// fuelview --career-audit (P99): a whole FUEL career played headless through the port's real code paths: the menus
// driven by synthetic keys (Map tab, hub, CAREER / CHALLENGES lists, vehicle carousel, garage, camps, DATA, Profile),
// every race resolved by the race rules (fuel::RaceRules) with a scripted autopilot and handed to the results chain
// (raceResultRows -> Menu::setResults -> the end-race pages -> WhatNext), every session a new Menu that reloads
// career.sav (the port restarts its process per session). Each step is checked against FUEL's rules decoded from
// FUEL.exe (notes/p99-career-audit.md). Runs on a scratch save only (HOME and FUEL_CAREER_SAVE are redirected).
#pragma once

#include <string>

namespace fuel::ui {

// Returns the number of failed checks (0 = the career works).
int runCareerAudit(const std::string& dataDir);

}  // namespace fuel::ui
