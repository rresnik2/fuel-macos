#include "settings.hpp"

#include <cctype>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>
#include <vector>

namespace fs = std::filesystem;

namespace fuel {

namespace {

std::string trim(const std::string& s) {
    const size_t a = s.find_first_not_of(" \t\r");
    if (a == std::string::npos) return "";
    return s.substr(a, s.find_last_not_of(" \t\r") - a + 1);
}

// The file's lines as (key, value), in order; comments and blank lines are kept with an empty key.
std::vector<std::pair<std::string, std::string>> readLines(const std::string& path) {
    std::vector<std::pair<std::string, std::string>> out;
    std::ifstream f(path);
    for (std::string line; std::getline(f, line);) {
        const size_t eq = line.find('=');
        const std::string key = eq == std::string::npos ? "" : trim(line.substr(0, eq));
        if (key.empty() || key[0] == '#')
            out.emplace_back("", line);
        else
            out.emplace_back(key, trim(line.substr(eq + 1)));
    }
    return out;
}

bool toBool(const std::string& v) { return v == "1" || v == "true" || v == "yes" || v == "on"; }

}  // namespace

std::string settingsPath() {
    // P101: tests and checks point the game at a scratch copy (never the player's own file).
    if (const char* p = std::getenv("FUEL_SETTINGS"); p && *p) return p;
    const char* home = std::getenv("HOME");
    return (fs::path(home ? home : ".") / "Library/Application Support/FUEL Port/settings.txt").string();
}

Settings loadSettings(const std::string& path) {
    Settings s;
    for (const auto& [k, v] : readLines(path)) {
        if (!k.empty()) s.all[k] = v;
        try {
            if (k == "data") s.dataDir = v;
            else if (k == "width") s.width = std::stoi(v);
            else if (k == "height") s.height = std::stoi(v);
            else if (k == "fullscreen") s.fullscreen = toBool(v);
            else if (k == "post_effects") s.postEffects = toBool(v);
            else if (k == "grass") s.grass = toBool(v);
            else if (k == "volume") s.volume = std::stof(v);
            else if (k == "language") s.language = v;
        } catch (const std::exception&) {
            // a malformed value keeps its default
        }
    }
    return s;
}

bool saveSettings(const Settings& s, const std::string& path) {
    const std::map<std::string, std::string> values = {
        {"data", s.dataDir},
        {"width", std::to_string(s.width)},
        {"height", std::to_string(s.height)},
        {"fullscreen", s.fullscreen ? "1" : "0"},
        {"post_effects", s.postEffects ? "1" : "0"},
        {"grass", s.grass ? "1" : "0"},
        {"volume", (std::ostringstream() << s.volume).str()},
        {"language", s.language},
    };
    std::vector<std::pair<std::string, std::string>> lines = readLines(path);
    std::map<std::string, bool> written;
    for (auto& [k, v] : lines)
        if (auto it = values.find(k); it != values.end()) v = it->second, written[k] = true;
    for (const auto& [k, v] : values)
        if (!written[k]) lines.emplace_back(k, v);
    std::error_code ec;
    fs::create_directories(fs::path(path).parent_path(), ec);
    std::ofstream f(path);
    if (!f) return false;
    for (const auto& [k, v] : lines) f << (k.empty() ? v : k + " = " + v) << "\n";
    return bool(f);
}

// ---- PK26 (see settings.hpp)
int languageIndex(const std::string& language) {
    std::string l;
    for (char c : language) l += char(std::tolower(static_cast<unsigned char>(c)));
    if (l == "en-uk" || l == "english_uk") return 10;  // 0x809
    if (l == "fr" || l == "french") return 2;          // 0x40c
    if (l == "es" || l == "spanish") return 3;         // 0x40a
    if (l == "de" || l == "german") return 4;          // 0x407
    if (l == "it" || l == "italian") return 5;         // 0x410
    if (l == "fr-ca") return 0x13;                     // 0xc0c
    return 1;                                          // 0x81e060's default (English US)
}

bool languageMiles(int index) { return index == 1 || index == 10 || index == 0x13 || index == 0x14; }
// ---- end PK26

}  // namespace fuel
