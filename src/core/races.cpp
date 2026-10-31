#include "races.hpp"

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
        if (k == "EMD_Begin") {
            r = Race{};
            inside = true;
        } else if (!inside) {
            continue;
        } else if (k == "EMD_End") {
            // As FUEL's EMD_End (0x5520c0): a looped race whose finish is not on the start gets the finish
            // as its last checkpoint and finishes on the start; a solo race without timer or bonus gets 1200 s.
            const auto& s = r.start;
            const auto& f = r.finish;
            if (r.laps > 0 && !r.checkpoints.empty() &&
                (std::fabs(s[0] - f[0]) > 1e-6f || std::fabs(s[1] - f[1]) > 1e-6f || std::fabs(s[2] - f[2]) > 1e-6f)) {
                r.checkpoints.push_back(f);
                r.checkpointFlags.push_back(r.checkpointFlags.empty() ? 1 : r.checkpointFlags.back());
                r.finish = s;
                r.finishYaw = r.startYaw;
            }
            if (r.timeLimit == 0 && r.checkBonus == 0 && r.aiCount == 0) r.timeLimit = 1200.0f;
            out.push_back(r);
            inside = false;
        } else if (k == "EMD_SetName" && t.size() > 1) {
            r.name = t[1];
        } else if (k == "EMD_SetNbLoop") {
            r.laps = int(num(t, 1));
        } else if (k == "EMD_SetTimer") {
            r.timeLimit = num(t, 1);
        } else if (k == "EMD_SetRaceMode") {
            r.mode = int(num(t, 1));
        } else if (k == "EMD_SetStartPos") {
            r.start = {num(t, 1), num(t, 2), num(t, 3)};
        } else if (k == "EMD_SetStartRot") {
            r.startYaw = num(t, 2);  // Euler angles in radians: x, y (heading), z
        } else if (k == "EMD_SetEndPos") {
            r.finish = {num(t, 1), num(t, 2), num(t, 3)};
        } else if (k == "EMD_SetEndRot") {
            r.finishYaw = num(t, 2);
        } else if (k == "EMD_SetCheckpoint") {
            r.checkpoints.push_back({num(t, 1), num(t, 2), num(t, 3)});
            // 0x5512c0: the 8th value when present, else 2 in baja scenarios (7, 8) and 1 elsewhere.
            r.checkpointFlags.push_back(t.size() > 8 ? int(num(t, 8)) : (r.scenario == 7 || r.scenario == 8) ? 2 : 1);
        } else if (k == "EMD_SetRaceScenario") {
            r.scenario = int(num(t, 1));
        } else if (k == "EMD_SetNbIA") {
            r.aiCount = int(num(t, 1));
        } else if (k == "EMD_SetTimerCheckBonus") {
            r.checkBonus = num(t, 1);
        } else if (k == "EMD_SetCheckpointTimerBonus") {
            r.checkpointBonus.push_back(num(t, 1));
        } else if (k == "EMD_ForceStartPosition") {
            r.aiGrid.push_back({num(t, 1), num(t, 2)});
        } else if (k == "EMD_HideStartEnd") {
            r.showStartEnd = false;
        } else if (k == "EMD_SetAIElasticParam") {
            for (int i = 0; i < 3; ++i) r.elastic[i] = num(t, size_t(i + 1));
        }
    }
    return out;
}

}  // namespace fuel
