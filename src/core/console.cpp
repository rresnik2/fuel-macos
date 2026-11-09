#include "console.hpp"

#include <strings.h>

namespace fuel {

std::string consoleAbbreviation(const std::string& name) {
    std::string a;
    for (char c : name) {  // 0x69a400: '@' < c < '[' or '/' < c < ':', at most 15 (0xe < n stops)
        if (a.size() > 14) break;
        if ((c > '@' && c < '[') || (c > '/' && c < ':')) a += c;
    }
    return a;
}

bool consoleCommandIs(const std::string& word, const std::string& name) {
    if (strcasecmp(word.c_str(), name.c_str()) == 0) return true;
    const std::string a = consoleAbbreviation(name);
    return !a.empty() && strcasecmp(word.c_str(), a.c_str()) == 0;
}

}  // namespace fuel
