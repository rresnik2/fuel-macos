// --menu-check: loads every menu page and resolves every text id, font and image
// it names; prints the problems and a total. Returns the process exit code.
#pragma once

#include <string>

namespace fuel::ui {

int runMenuCheck(const std::string& dataDir, const std::string& language);

}  // namespace fuel::ui
