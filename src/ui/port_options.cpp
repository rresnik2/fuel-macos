#include "ui/port_options.hpp"

#include "ui/pad_prompts.hpp"
#include "core/input_map.hpp"  // [port] remap

#include <algorithm>
#include <cctype>
#include <cstdlib>
#include <fstream>
#include <sstream>
#include <vector>

namespace fuel::ui {

namespace {

std::string lower(std::string s) {
    for (char& c : s) c = char(std::tolower(static_cast<unsigned char>(c)));
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back()))) s.pop_back();
    size_t a = 0;
    while (a < s.size() && std::isspace(static_cast<unsigned char>(s[a]))) ++a;
    return s.substr(a);
}

const std::string* find(const std::map<std::string, std::string>& all, const char* k) {
    const auto it = all.find(k);
    return it == all.end() ? nullptr : &it->second;
}

// game_settings.mm flag(): on/1/true/yes, off/0/false/no, anything else the default.
bool onOff(const std::map<std::string, std::string>& all, const char* k, bool def) {
    const std::string* v = find(all, k);
    if (!v) return def;
    const std::string s = lower(*v);
    if (s == "on" || s == "1" || s == "true" || s == "yes") return true;
    if (s == "off" || s == "0" || s == "false" || s == "no") return false;
    return def;
}

std::string num(float f) {
    std::ostringstream s;
    s << f;
    return s.str();
}

}  // namespace

PortOptions PortOptions::from(const std::map<std::string, std::string>& all) {
    PortOptions o;
    o.dynres = onOff(all, "dynamic_resolution", true);
    if (const std::string* v = find(all, "dynamic_resolution_min")) {
        const float m = std::strtof(v->c_str(), nullptr);
        if (m > 0) o.dynresMin = std::clamp(m, 0.5f, 1.0f);
    }
    if (const std::string* v = find(all, "dynamic_resolution_upscaler")) {  // renderer.h upscalerFromName
        const std::string s = lower(*v);
        o.upscaler = s == "metalfx" || s == "metalfx-spatial" ? 1 : s == "metalfx-temporal" || s == "temporal" ? 2 : 0;
    }
    if (const std::string* v = find(all, "shadow_filter")) o.shadow3x3 = *v == "3x3";
    if (const std::string* v = find(all, "wind_debris")) {
        const std::string s = lower(*v);
        if (s == "fuel") o.debrisFuel = true;
        else if (s == "smooth") o.debrisFuel = false;
    }
    o.limiter = onOff(all, "audio_limiter", true);
    if (const std::string* v = find(all, "vibration")) {  // pad_haptics.mm settingOn: only 0/false/no/off turn it off
        const std::string s = lower(*v);
        o.vibration = !(s == "0" || s == "false" || s == "no" || s == "off");
    }
    auto invert = [&](const char* k) {  // input_map.cpp flag(): 1/true/yes/on
        const std::string* v = find(all, k);
        return v && (*v == "1" || *v == "true" || *v == "yes" || *v == "on");
    };
    o.invertX = invert("invert_camera_x"), o.invertY = invert("invert_camera_y");
    if (const std::string* v = find(all, "controls")) o.portControls = lower(*v) == "port";
    if (const std::string* v = find(all, "adaptive_triggers")) {  // pad_triggers.mm Config::fromSettings
        const std::string s = lower(*v);
        o.triggers = (s == "off" || s == "0" || s == "false" || s == "no") ? 0 : (s == "light" || s == "low") ? 1 : 2;
    }
    if (const std::string* v = find(all, "adaptive_triggers_extras")) {
        const std::string s = lower(*v);
        o.extras = s.empty() ? "none" : (s == "on" || s == "1") ? "all" : s;
    }
    if (const std::string* v = find(all, "film_grain")) {
        const std::string s = lower(*v);
        o.filmGrain = (s == "off" || s == "0" || s == "no" || s == "false") ? 0 : s == "strong" ? 2 : 1;
    }
    if (const std::string* v = find(all, "soften")) {
        const std::string s = lower(*v);
        o.soften = (s == "light" || s == "1") ? 1 : s == "strong" ? 2 : 0;
    }
    if (const std::string* v = find(all, "pad_glyphs")) o.padGlyphs = int(pad::styleFromSetting(*v));  // pad-ux [port]
    if (const std::string* v = find(all, "rider_jiggle")) {  // PHYSIQUE (core/rider_jiggle.cpp riderJiggleSetting)
        const std::string s = lower(*v);
        o.jiggle = (s == "off" || s == "0" || s == "no" || s == "false") ? 0 : (s == "full" || s == "2" || s == "strong") ? 2 : 1;
    }
    o.nitrous = onOff(all, "nitrous", true);  // [nitro] game_settings.mm: nitro::enabledFrom
    o.upgrades = onOff(all, "upgrades", true);  // [upgrades U2] game_settings.mm: upg::enabledFrom
    o.devMode = onOff(all, "dev_mode", false);  // [dev mode] the menus' Career::setDevMode
    return o;
}

void PortOptions::store(std::map<std::string, std::string>& all) const {
    all["dynamic_resolution"] = dynres ? "on" : "off";
    all["dynamic_resolution_min"] = num(dynresMin);
    all["dynamic_resolution_upscaler"] = upscaler == 1 ? "metalfx" : upscaler == 2 ? "metalfx-temporal" : "sharpen";
    all["shadow_filter"] = shadow3x3 ? "3x3" : "2x2";
    all["wind_debris"] = debrisFuel ? "fuel" : "smooth";
    all["audio_limiter"] = limiter ? "on" : "off";
    all["vibration"] = vibration ? "1" : "0";
    all["invert_camera_x"] = invertX ? "1" : "0";
    all["invert_camera_y"] = invertY ? "1" : "0";
    all["controls"] = portControls ? "port" : "fuel";
    all["adaptive_triggers"] = triggers == 0 ? "off" : triggers == 1 ? "light" : "full";
    all["adaptive_triggers_extras"] = extras;
    all["film_grain"] = filmGrain == 0 ? "off" : filmGrain == 2 ? "strong" : "light";
    all["soften"] = soften == 0 ? "off" : soften == 2 ? "strong" : "light";
    all["pad_glyphs"] = pad::settingOf(pad::Style(std::clamp(padGlyphs, 0, 2)));
    all["rider_jiggle"] = jiggle == 0 ? "off" : jiggle == 2 ? "full" : "subtle";  // PHYSIQUE
    all["nitrous"] = nitrous ? "on" : "off";
    all["upgrades"] = upgrades ? "on" : "off";  // [upgrades U2]
    all["dev_mode"] = devMode ? "on" : "off";   // [dev mode]
}

bool PortOptions::operator==(const PortOptions& o) const {
    return dynres == o.dynres && dynresMin == o.dynresMin && upscaler == o.upscaler && shadow3x3 == o.shadow3x3 && debrisFuel == o.debrisFuel &&
           limiter == o.limiter && vibration == o.vibration && invertX == o.invertX && invertY == o.invertY && portControls == o.portControls &&
           triggers == o.triggers && extras == o.extras && filmGrain == o.filmGrain && soften == o.soften && padGlyphs == o.padGlyphs &&
           jiggle == o.jiggle && nitrous == o.nitrous && upgrades == o.upgrades && devMode == o.devMode;
}

bool savePortOptions(const PortOptions& o, const std::string& path) {
    // The file's lines as they are (comments, blanks and unknown keys stay).
    std::vector<std::pair<std::string, std::string>> lines;
    {
        std::ifstream f(path);
        for (std::string line; std::getline(f, line);) {
            const size_t eq = line.find('=');
            std::string key = eq == std::string::npos ? "" : line.substr(0, eq);
            while (!key.empty() && std::isspace(static_cast<unsigned char>(key.back()))) key.pop_back();
            size_t a = 0;
            while (a < key.size() && std::isspace(static_cast<unsigned char>(key[a]))) ++a;
            key = key.substr(a);
            if (key.empty() || key[0] == '#') lines.emplace_back("", line);
            else lines.emplace_back(key, line);  // the original line, replaced below when its key is ours
        }
    }
    std::map<std::string, std::string> mine, defaults;
    o.store(mine);
    PortOptions().store(defaults);
    for (const auto& [k, v] : mine) {
        bool done = false;
        for (auto& [lk, lv] : lines)
            if (lk == k) {
                // Keep the player's own spelling when it already means the same value (e.g. vibration = on).
                std::map<std::string, std::string> one{{lk, lv.substr(lv.find('=') + 1)}};
                PortOptions was = PortOptions::from(one);
                std::map<std::string, std::string> back;
                was.store(back);
                if (back[k] != v) lv = k + " = " + v;
                done = true;
            }
        if (!done && v != defaults[k]) lines.emplace_back(k, k + " = " + v);
    }
    std::ofstream f(path);
    if (!f) return false;
    for (const auto& [k, l] : lines) f << l << "\n";
    return bool(f);
}

// ---- [port] remap
bool saveBindingLines(const std::map<std::string, std::string>& lines, const std::string& path) {
    using fuel::input::InputMap;
    auto id = [](const std::string& k) {  // "key." / "pad." + the action, whatever its spelling (key.nitro = key.BOOST)
        return k.substr(0, 4) + std::to_string(fuel::input::actionByName(k.substr(4)));
    };
    std::map<std::string, std::pair<std::string, std::string>> want;  // id -> (key, value)
    for (const auto& [k, v] : lines)
        if (InputMap::isBindingKey(k)) want[id(k)] = {k, v};
    std::vector<std::string> out;
    std::map<std::string, bool> done;
    {
        std::ifstream f(path);
        for (std::string line; std::getline(f, line);) {
            const size_t eq = line.find('=');
            std::string key = eq == std::string::npos ? "" : line.substr(0, eq);
            while (!key.empty() && std::isspace(static_cast<unsigned char>(key.back()))) key.pop_back();
            size_t a = 0;
            while (a < key.size() && std::isspace(static_cast<unsigned char>(key[a]))) ++a;
            key = key.substr(a);
            if (key.empty() || key[0] == '#' || !InputMap::isBindingKey(key)) {
                out.push_back(line);
                continue;
            }
            const std::string i = id(key);
            auto w = want.find(i);
            if (w == want.end() || done[i]) continue;  // back to its default (or a second line of the same action): gone
            out.push_back(key + " = " + w->second.second);
            done[i] = true;
        }
    }
    for (const auto& [i, kv] : want)
        if (!done[i]) out.push_back(kv.first + " = " + kv.second);
    std::ofstream f(path);
    if (!f) return false;
    for (const std::string& l : out) f << l << "\n";
    return bool(f);
}
// ---- end remap

}  // namespace fuel::ui
