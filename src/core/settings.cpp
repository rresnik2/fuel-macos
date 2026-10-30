#include "settings.hpp"

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
    const char* home = std::getenv("HOME");
    return (fs::path(home ? home : ".") / "Library/Application Support/FUEL Port/settings.txt").string();
}

Settings loadSettings(const std::string& path) {
    Settings s;
    for (const auto& [k, v] : readLines(path)) {
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

}  // namespace fuel
