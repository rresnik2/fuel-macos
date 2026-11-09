// --inputcheck (P47): FUEL's input layer against emulated vectors and the player's binding files.
#pragma once
#include <string>
namespace fuel::input {
int runInputCheck(const std::string& dataDir);  // 0 when every vector matches
}
