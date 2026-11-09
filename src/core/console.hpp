// FUEL's console command lookup (ConsoleInterp_Z, notes/p20-console.md C1 and p38-scripting.md S1):
// Console_Register 0x69a400 stores two case-insensitive hashes (0x669160) per command, one of the
// full name and one of its abbreviation (the name's A-Z and 0-9 characters, at most 15), and
// Console_FindCommand 0x69a4f0 accepts either. So "TeLePort", "teleport" and "TLP" are one command,
// and a script word matches a command whatever its case.
#pragma once

#include <string>

namespace fuel {

std::string consoleAbbreviation(const std::string& name);
// The script word `word` runs the command registered as `name`.
bool consoleCommandIs(const std::string& word, const std::string& name);

}  // namespace fuel
