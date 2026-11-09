#include "object_script.hpp"

#include <cstring>
#include <strings.h>

namespace fuel {

namespace {

bool startsWith(const std::string& s, const char* p) { return s.compare(0, std::strlen(p), p) == 0; }

// 0x53f640: a UserDefine line's keyword and its argument (the rest of the line, trimmed).
void splitLine(const std::string& line, std::string& key, std::string& arg) {
    const size_t a = line.find_first_not_of(" \t");
    if (a == std::string::npos) return key.clear(), arg.clear();
    const size_t b = line.find_first_of(" \t", a);
    key = line.substr(a, b == std::string::npos ? std::string::npos : b - a);
    arg.clear();
    if (b == std::string::npos) return;
    const size_t c = line.find_first_not_of(" \t", b);
    if (c == std::string::npos) return;
    const size_t d = line.find_last_not_of(" \t");
    arg = line.substr(c, d - c + 1);
}

bool sameName(const std::string& a, const std::string& b) { return strcasecmp(a.c_str(), b.c_str()) == 0; }

}  // namespace

std::optional<ObjectCondition> parseObjectCondition(const std::vector<std::string>& lines) {
    ObjectCondition c;
    bool found = false;
    for (const std::string& line : lines) {
        std::string key, arg;
        splitLine(line, key, arg);
        if (strcasecmp(key.c_str(), "CONDITION_OBJECT") == 0) {
            found = true;
            c.sons = sameName(arg, "SONS");
        } else if (strcasecmp(key.c_str(), "IA_BLOCK_ROAD") == 0) {
            c.blocksRoad = true;  // 0x55edd0: node +0xd8 |= 0x80
        }
    }
    if (!found) return std::nullopt;
    // 0x513010: every token after the keyword (0x6861e0 walks the UserDefine).
    for (const std::string& line : lines) {
        std::string key, arg;
        splitLine(line, key, arg);
        if (key.empty()) continue;
        if (strcasecmp(key.c_str(), "HIDEBYRTC") == 0) {
            c.hideByRtc = true;
        } else if (strcasecmp(key.c_str(), "DEFAULTHIDE") == 0) {
            c.defaultHidden = true;
        } else if (strcasecmp(key.c_str(), "DONTHIDEUNHIDE") == 0) {
            c.dontHideUnhide = true;
        } else {
            // The condition: strstr in this order (so M_WONCLOSED is tested before M_WON).
            static const struct {
                const char* s;
                int kind;
            } kConditions[] = {{"M_CLOSED", ObjectCondition::Closed},     {"M_TELEPORT", ObjectCondition::Teleport},
                               {"M_START", ObjectCondition::Start},       {"M_END", ObjectCondition::End},
                               {"M_OPEN", ObjectCondition::Open},         {"M_NOTOPEN", ObjectCondition::NotOpen},
                               {"M_RUNNING", ObjectCondition::Running},   {"M_WONCLOSED", ObjectCondition::WonClosed},
                               {"M_WON", ObjectCondition::Won},           {"R_RUNNING", ObjectCondition::RtcRunning},
                               {"L_OPEN", ObjectCondition::LevelOpen},    {"P_PLAYER", ObjectCondition::Player},
                               {"P_DIED", ObjectCondition::Died},         {"PLATEFORM_WII", ObjectCondition::Wii},
                               {"G_DEMO", ObjectCondition::Demo},         {"LAUNCHED_FROM_STORY", ObjectCondition::FromStory},
                               {"LAUNCHED_FROM_MENU", ObjectCondition::FromMenu}};
            ObjectCondition::Rule r;
            for (const auto& k : kConditions)
                if (key.find(k.s) != std::string::npos) {
                    r.kind = k.kind;
                    break;
                }
            // The action: the token's prefix (case-sensitive strstr == start), in 0x513010's order.
            static const struct {
                const char* p;
                int action;
            } kActions[] = {{"TELEP_", ObjectCondition::Telep},         {"HIDE_", ObjectCondition::Hide},
                            {"RESET_", ObjectCondition::Reset},         {"STARTANIM_", ObjectCondition::StartAnim},
                            {"STOP_", ObjectCondition::Stop},           {"UNHIDE_", ObjectCondition::Unhide},
                            {"STREAM_", ObjectCondition::Stream},       {"UNSTREAM_", ObjectCondition::Unstream}};
            for (const auto& a : kActions)
                if (startsWith(key, a.p)) {
                    r.action = a.action;
                    break;
                }
            r.arg = arg;
            // A rule counts only with both a condition and an action, at most 4 of them (0xa4-byte record).
            if (r.kind != ObjectCondition::None && r.action != ObjectCondition::NoAction && c.rules.size() < 4) c.rules.push_back(std::move(r));
        }
    }
    return c;
}

std::optional<bool> evaluateObjectCondition(const ObjectCondition& c, const ObjectGameState& state, const ObjectEvent& event,
                                            std::vector<const ObjectCondition::Rule*>* actions) {
    // P67, 0x513630 head (asm 0x513640..0x513654): HIDEBYRTC and no R_RUNNING rule (+0xe): hidden while an RTC plays
    // (0x539990, then 0x5399c0 and 0x513ed0 with BL = 1), otherwise untouched; the rules are not looked at.
    bool rtcRule = false;
    for (const ObjectCondition::Rule& r : c.rules) rtcRule = rtcRule || r.kind == ObjectCondition::RtcRunning;
    if (c.hideByRtc && !rtcRule) return state.rtcRunning ? std::optional<bool>(true) : std::nullopt;
    bool hidden = c.defaultHidden;
    for (const ObjectCondition::Rule& r : c.rules) {
        bool holds = false;
        switch (r.kind) {
        case ObjectCondition::Closed:
        case ObjectCondition::NotOpen: holds = state.missionOpen && !state.missionOpen(r.arg); break;
        case ObjectCondition::Open: holds = !state.missionOpen || state.missionOpen(r.arg); break;
        case ObjectCondition::Running: holds = !state.runningMission.empty() && sameName(state.runningMission, r.arg); break;
        case ObjectCondition::Won: holds = state.missionWon && state.missionWon(r.arg); break;
        case ObjectCondition::WonClosed: holds = state.missionWon && state.missionWon(r.arg); break;  // port: + the hub state check (0x57f540) not decoded
        case ObjectCondition::FromStory: holds = !state.launchedFromMenu; break;
        case ObjectCondition::FromMenu: holds = state.launchedFromMenu; break;
        case ObjectCondition::RtcRunning: holds = state.rtcRunning; break;  // P67: case 9, agent 0x539920 +0x12d
        default: break;  // level, player, Wii and demo conditions: false in the port's single-player PC game
        }
        // Event rules (0x513c6c table): TELEPORT / START / END fire for their mission, DIED always.
        bool fires = holds;
        if (event.kind != ObjectCondition::None && r.kind == event.kind) {
            if (r.kind == ObjectCondition::Died) fires = true;
            else if (r.kind == ObjectCondition::Teleport || r.kind == ObjectCondition::Start || r.kind == ObjectCondition::End)
                fires = fires || sameName(r.arg, event.mission);
        }
        if (!fires) continue;
        if (r.action == ObjectCondition::Hide) hidden = true;
        else if (r.action == ObjectCondition::Unhide) hidden = false;
        if (actions) actions->push_back(&r);
    }
    // 0x513bb5: Apply(hidden) unless DONTHIDEUNHIDE; then 0x513bc0..0x513c06: an RTC plays and HIDEBYRTC -> Apply(1).
    if (c.hideByRtc && state.rtcRunning) return true;
    if (c.dontHideUnhide) return std::nullopt;
    return hidden;
}

}  // namespace fuel
