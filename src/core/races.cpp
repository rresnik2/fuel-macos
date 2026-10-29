#include "races.hpp"

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
        }
    }
    return out;
}

}  // namespace fuel
