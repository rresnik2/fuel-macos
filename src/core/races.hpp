// Races from the mission scripts (LocalizedComponent/GameTsc/Story/
// miss_official.tsc for the career, missChallenge.tsc for hub challenges):
// EMD_Begin .. EMD_End blocks with the start, ordered checkpoints and finish.
#pragma once

#include <array>
#include <string>
#include <vector>

namespace fuel {

struct Race {
    enum Mode { AtoB = 0, Checkpoint = 1, KnockOut = 2, TimeTrial = 3, Circuit = 4, Baja = 5, Raid = 6 };
    std::string name;  // e.g. "CR2-Yell-CP"
    int mode = -1;
    int laps = 0;      // circuits and baja; 0 = one run
    float timeLimit = 0;  // seconds (time trials), 0 = none
    std::array<float, 3> start{}, finish{};
    float startYaw = 0, finishYaw = 0;  // heading in radians; forward = (sin, cos) in x, z
    std::vector<std::array<float, 3>> checkpoints;

    bool loops() const { return laps > 0 && (mode == Circuit || mode == Baja); }
};

std::vector<Race> parseMissions(const std::string& tscText);

}  // namespace fuel
