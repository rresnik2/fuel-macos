#include "races.hpp"

#include "console.hpp"

#include <cmath>
#include <cstdlib>
#include <sstream>

namespace fuel {

namespace {

// Tokens of one script line: quoted strings or bare words; "//" starts a comment.
std::vector<std::string> tokens(const std::string& line) {
    std::vector<std::string> out;
    size_t i = 0;
    while (i < line.size()) {
        if (line[i] == ' ' || line[i] == '\t' || line[i] == '\r') {
            ++i;
        } else if (line.compare(i, 2, "//") == 0) {
            break;
        } else if (line[i] == '"') {
            const size_t end = line.find('"', i + 1);
            out.push_back(line.substr(i + 1, end == std::string::npos ? std::string::npos : end - i - 1));
            i = end == std::string::npos ? line.size() : end + 1;
        } else {
            size_t end = i;
            while (end < line.size() && line[end] != ' ' && line[end] != '\t' && line[end] != '\r') ++end;
            out.push_back(line.substr(i, end - i));
            i = end;
        }
    }
    return out;
}

// 0x5510a0 + Quat_ToRotationVector 0x41d280: the one-argument form of EMD_SetStartRot / EMD_SetEndRot is a yaw in degrees,
// q = (0, sin(d/2), 0, cos(d/2)); the rotation vector is axis * angle with the angle folded into [0, pi].
std::array<float, 3> yawDegreesToRotationVector(float deg) {
    const float h = deg * 3.1415927f * 0.0055555557f * 0.5f;
    const float qy = std::sin(h);
    float w = std::cos(h);
    w = w < -1.0f ? -1.0f : w > 1.0f ? 1.0f : w;
    const double s = std::sqrt(1.0 - double(w) * w);
    if (s <= 1e-6) return {0, 0, 0};
    float axis = float(qy / s);
    double angle = 2.0 * std::acos(double(w));
    if (angle > 3.1415927) axis = -axis, angle = 6.2831855 - angle;
    return {0.0f, float(axis * angle), 0.0f};
}

float num(const std::vector<std::string>& t, size_t i) { return i < t.size() ? std::strtof(t[i].c_str(), nullptr) : 0.0f; }

}  // namespace

int Race::gameMode() const {
    auto timeTrial = [&] {
        if (checkpoints.empty()) return GmTimeTrial;
        return checkBonus == 0 && checkpointBonus.empty() ? GmTimeLimitCheckpoints : GmTimeBonus;
    };
    switch (scenario) {
    case 1: case 7: return GmAtoB;
    case 2: return GmCheckpoint;
    case 3: return GmKnockOut;
    case 4: case 5: return timeTrial();
    case 6: case 8: return GmCircuit;
    case 9:
        switch (mode) {
        case AtoB: return GmAtoB;
        case Checkpoint: return GmCheckpoint;
        case KnockOut: return GmKnockOut;
        case TimeTrial: return timeTrial();
        case Circuit: return GmCircuit;
        case Baja: return GmBaja;
        case Raid: return GmRaid;
        default: return GmInvalid;
        }
    case 11: return GmHelicopter;
    case 13: return GmCatchAI;
    default: return GmInvalid;
    }
}

std::vector<Race> parseMissions(const std::string& text) {
    std::vector<Race> out;
    std::istringstream in(text);
    std::string line;
    bool inside = false;
    Race r;
    while (std::getline(in, line)) {
        const std::vector<std::string> t = tokens(line);
        if (t.empty()) continue;
        const std::string& k = t[0];
        auto is = [&k](const char* name) { return consoleCommandIs(k, name); };  // 0x69a4f0: any case or abbreviation
        if (is("EMD_Begin")) {
            r = Race{};
            inside = true;
        } else if (!inside) {
            continue;
        } else if (is("EMD_End")) {
            // As FUEL's EMD_End (0x5520c0): a looped race whose finish is not on the start gets the finish
            // as its last checkpoint and finishes on the start; a solo race without timer or bonus gets 1200 s.
            const auto& s = r.start;
            const auto& f = r.finish;
            if (r.laps > 0 && !r.checkpoints.empty() &&
                (std::fabs(s[0] - f[0]) > 1e-6f || std::fabs(s[1] - f[1]) > 1e-6f || std::fabs(s[2] - f[2]) > 1e-6f)) {
                r.checkpoints.push_back(f);
                r.checkpointFlags.push_back(r.checkpointFlags.empty() ? 1 : r.checkpointFlags.back());
                r.checkpointRot.push_back(r.finishRot);
                r.finish = s;
                r.finishYaw = r.startYaw;
                r.finishRot = r.startRot;
            }
            if (r.timeLimit == 0 && r.checkBonus == 0 && r.aiCount == 0) r.timeLimit = 1200.0f;
            out.push_back(r);
            inside = false;
        } else if (is("EMD_SetName") && t.size() > 1) {
            r.name = t[1];
        } else if (is("EMD_SetNbLoop")) {
            r.laps = int(num(t, 1));
        } else if (is("EMD_SetTimer")) {
            r.timeLimit = num(t, 1);
        } else if (is("EMD_SetRaceMode")) {
            r.mode = int(num(t, 1));
        } else if (is("EMD_SetStartPos")) {
            r.start = {num(t, 1), num(t, 2), num(t, 3)};
        } else if (is("EMD_SetStartRot") && (t.size() == 2 || t.size() == 5)) {  // 0x5511a0: one or four arguments
            r.startRot = t.size() == 2 ? yawDegreesToRotationVector(num(t, 1)) : std::array<float, 3>{num(t, 1), num(t, 2), num(t, 3)};
            r.startYaw = r.startRot[1];  // rotation vector (0x644980: axis-angle); y = heading
        } else if (is("EMD_SetEndPos")) {
            r.finish = {num(t, 1), num(t, 2), num(t, 3)};
        } else if (is("EMD_SetEndRot") && (t.size() == 2 || t.size() == 5)) {  // 0x551230
            r.finishRot = t.size() == 2 ? yawDegreesToRotationVector(num(t, 1)) : std::array<float, 3>{num(t, 1), num(t, 2), num(t, 3)};
            r.finishYaw = r.finishRot[1];
        } else if (is("EMD_SetCheckpoint")) {
            r.checkpoints.push_back({num(t, 1), num(t, 2), num(t, 3)});
            r.checkpointRot.push_back({num(t, 4), num(t, 5), num(t, 6)});
            // 0x5512c0: the 8th value when present, else 2 in baja scenarios (7, 8) and 1 elsewhere.
            r.checkpointFlags.push_back(t.size() > 8 ? int(num(t, 8)) : (r.scenario == 7 || r.scenario == 8) ? 2 : 1);
        } else if (is("EMD_SetRaceScenario")) {
            r.scenario = int(num(t, 1));
        } else if (is("EMD_SetNbIA")) {
            r.aiCount = int(num(t, 1));
        } else if (is("EMD_SetTimerCheckBonus")) {
            r.checkBonus = num(t, 1);
        } else if (is("EMD_SetCheckpointTimerBonus")) {
            r.checkpointBonus.push_back(num(t, 1));
        } else if (is("EMD_ForceStartPosition")) {
            r.aiGrid.push_back({num(t, 1), num(t, 2)});
        } else if (is("EMD_HideStartEnd")) {
            r.showStartEnd = false;
        } else if (is("EMD_SetPlayerForbiddenVehiculeFilter")) {  // PK02: 0x5517d0 (clear, then append each argument)
            r.forbidden.clear(), r.forbiddenNames.clear();
            for (size_t i = 1; i < t.size(); ++i) {
                char* end = nullptr;
                const long v = std::strtol(t[i].c_str(), &end, 10);
                if (!t[i].empty() && *end == '\0') r.forbidden.push_back(int(v));
                else r.forbiddenNames.push_back(t[i]);
            }
        } else if (is("EMD_SetFilterIA")) {  // PK10: 0x551530 (needs an argument; appends, never clears)
            for (size_t i = 1; i < t.size(); ++i) r.aiFilterTokens.push_back(t[i]);
        } else if (is("EMD_SetAIElasticParam")) {
            for (int i = 0; i < 3; ++i) r.elastic[i] = num(t, size_t(i + 1));
        }
    }
    return out;
}

}  // namespace fuel
