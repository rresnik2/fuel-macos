// --animcheck (P47): FUEL's animation samplers, RTC files and playback timing.
#pragma once
#include <string>
namespace fuel::anim {
int runAnimCheck(const std::string& dataDir);  // 0 when every vector matches
}
