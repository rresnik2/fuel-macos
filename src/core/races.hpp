// Races from the mission scripts (LocalizedComponent/GameTsc/Story/
// miss_official.tsc for the career, missChallenge.tsc for hub challenges):
// EMD_Begin .. EMD_End blocks with the start, ordered checkpoints and finish.
// Field meanings follow FUEL.exe's console handlers (notes/p17-events.md E5-E8).
#pragma once

#include "core/spline_graph.hpp"
#include <functional>
#include <memory>

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
    // EMD_SetStartRot / EMD_SetEndRot / EMD_SetCheckpoint values 4..6: a rotation vector (axis times
    // angle in radians), which FUEL turns into a quaternion with 0x644980 (angle = |v|, axis = v / |v|).
    // The scripts only use the y component (heading), so it equals the yaw above.
    std::array<float, 3> startRot{}, finishRot{};
    std::vector<std::array<float, 3>> checkpointRot;

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
    // PK02: EMD_SetPlayerForbiddenVehiculeFilter (0x5517d0): the vehicles the player may not use in this race, as indices
    // into the game's vehicle table (AddVehicleInfo order of VehiclesDef.tsc; mission +0xb8/+0xbc). The command clears the
    // list first and appends one entry per argument: an integer (strtol of the whole token), else a vehicle name
    // (0x5d6180, resolved by loadEvents into `forbidden`; `forbiddenNames` holds the names until then).
    std::vector<int> forbidden;
    std::vector<std::string> forbiddenNames;
    // PK10: EMD_SetFilterIA (0x551530): the AI vehicle list of the race (mission +0xa8/+0xac), appended argument by
    // argument (not cleared first): an integer (strtol of the whole token) or a vehicle name (0x5d6180). `aiFilter`
    // holds the table indices; `aiFilterNames` the names until loadEvents resolves them (in argument order they
    // are kept in `aiFilterTokens`, which is what the parser fills). IA_RaceSetup 0x620e30 gives AI racer k the
    // entry k mod n; AddMissionParam's list (0x575b00) replaces it.
    std::vector<std::string> aiFilterTokens;
    std::vector<int> aiFilter;

    // Missions built from a world node instead of an EMD block (0x4f4db0, notes/p42-missions.md):
    // mission +8 = 0x1d. Their gates carry their own half-widths (0x5c16c0 instead of the road
    // widths of 0x5c1360): checkpointHalf[k] = mission +0x188[k], finishHalf = +0x180, startHalf =
    // +0x17c (only 0x4fdea0 reads it). popping: "EditRacePoppingCheckpoint" (the checkpoint nodes
    // are hidden at load and shown one by one); checkpointNodes = those Node_Z ids (+0x178), 0 = none.
    bool worldCourse = false;
    std::vector<float> checkpointHalf;
    float startHalf = 0, finishHalf = 0;
    bool popping = false;
    std::vector<uint32_t> checkpointNodes;
    uint32_t courseNode = 0;  // the MISSION_COURSE Node_Z (mission +0x1fc = node +8)
    // The course's racing lanes: the first child node of the course node whose object is a SplineGraph_Z
    // (0x623ac0: object +0x64 type 0x19 -> the AI path request +0x3c; IAPathBlockSpline, P43).
    std::shared_ptr<const SplineGraph> splineGraph;
    // ---- PK21: an edited race (race_editor.hpp, commit 0x465be0): mission +0x54, the editor's weather / time scenario bits
    // (0x45ff30; 0x631aa0 reads bits 2-5 when bit 1 is set). 0 for the scripts' races.
    uint16_t editorFlags = 0;

    // The game's effective race kind, decoded from 0x4fe2d0 (scenario first; scenario 9 goes by `mode`).
    enum GameMode {
        GmInvalid = -1, GmTimeTrial = 1, GmHelicopter = 2, GmCatchAI = 3, GmRaid = 4, GmTimeBonus = 5,
        GmTimeLimitCheckpoints = 6, GmKnockOut = 7, GmCircuit = 8, GmCheckpoint = 9, GmBaja = 11, GmAtoB = 12
    };
    int gameMode() const;
};

std::vector<Race> parseMissions(const std::string& tscText);

class DpcFile;
// The missions the world itself defines (0x4f4db0, called by the level's mission build 0x4f40f0
// after the EMD blocks): every Node_Z whose UserDefine holds MISSION_COURSE, in the order of a walk
// of the World_Z root's tree, with its start, checkpoints and finish from its child nodes.
// In the shipped USA1.DPC: CR1-YELL-CP and the 14 -CT circuits.
std::vector<Race> loadWorldMissions(const DpcFile& usa1);

// ---- PK14: GameManager_G::SetGameStarted 0x557e50 (console GAMEStarted, the front end's "AddCommandAfterTeleport
// GAMEStarted" once the boot teleport landed) -> 0x4f5b90 on the level's mission list (game+0x3348 object +0x38: the EMD
// missions, then the world's, 0x210-B records): each record's start y (+0x10), finish y (+0x1c) and every point's y
// (list +0x64, count +0x60 >> 14, stride 0xc, +4; the gates and the AI path points) = the coarse world height 0x445570
// (the height map's B-spline; 0 while the world's map is not loaded, W+0x4b8 != 1). The port runs it on the missions a
// session loads: every in-world session comes after FUEL's GAMEStarted.
void snapMissionHeights(Race& r, const std::function<float(float x, float z)>& coarseHeight);
// ---- end PK14

}  // namespace fuel
