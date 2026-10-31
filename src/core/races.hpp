// Races from the mission scripts (LocalizedComponent/GameTsc/Story/
// miss_official.tsc for the career, missChallenge.tsc for hub challenges):
// EMD_Begin .. EMD_End blocks with the start, ordered checkpoints and finish.
// Field meanings follow FUEL.exe's console handlers (notes/p17-events.md E5-E8).
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

    // FUEL's scenario enum (EMD_SetRaceScenario, mission +0x58; validator 0x738590):
    // 1 A to B, 2 checkpoint, 3 knock out, 4 time trial, 5 time trial with checkpoints,
    // 6 circuit, 7 baja, 8 baja circuit, 9 automatic (by `mode`), 11 helicopter, 13 catch AI.
    int scenario = 0;
    int aiCount = 0;          // EMD_SetNbIA (for helicopter races: the helicopter)
    float checkBonus = 0;     // EMD_SetTimerCheckBonus: seconds added at every checkpoint
    std::vector<float> checkpointBonus;  // EMD_SetCheckpointTimerBonus, one per checkpoint
    std::vector<int> checkpointFlags;    // 8th EMD_SetCheckpoint value (default 2 for baja, else 1);
                                         // 4 marks an AI path point (helicopter / hunted racers), not a gate
    std::vector<std::array<float, 2>> aiGrid;  // EMD_ForceStartPosition x z
    bool showStartEnd = true;  // EMD_HideStartEnd clears it
    float elastic[3] = {0, 0, 0};  // EMD_SetAIElasticParam

    // The game's effective race kind, decoded from 0x4fe2d0 (scenario first; scenario 9 goes by `mode`).
    enum GameMode {
        GmInvalid = -1, GmTimeTrial = 1, GmHelicopter = 2, GmCatchAI = 3, GmRaid = 4, GmTimeBonus = 5,
        GmTimeLimitCheckpoints = 6, GmKnockOut = 7, GmCircuit = 8, GmCheckpoint = 9, GmBaja = 11, GmAtoB = 12
    };
    int gameMode() const;
};

std::vector<Race> parseMissions(const std::string& tscText);

}  // namespace fuel
