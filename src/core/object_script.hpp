// FUEL's object scripts: the UserDefine_Z keywords that make a placed object react to the game
// state ("CONDITION_OBJECT"). Decoded in ~/re-work/fuel/notes/p38-scripting.md (section S3):
//   0x512da0  register: walks a node tree (child +0xa4, sibling +0xac); a node whose UserDefine has
//             CONDITION_OBJECT gets a 0xa4-byte record (at most 4 rules of 0x24 bytes)
//   0x513010  parse the UserDefine tokens into flags and rules (below)
//   0x513630  evaluate: hidden := DEFAULTHIDE, then each rule whose condition holds (or whose event
//             fires) applies its action in order; unless DONTHIDEUNHIDE, the result is applied
//   0x513ed0  apply: hide (0x513fd0: node flag 0x40000) or show (0x514170) the node, and with SONS
//             every node below it; IA_BLOCK_ROAD objects (node +0xd8 bit 0x80) block the nearest
//             road record within 10 m (bit 29) while shown
// A rule token is "<ACTION>_<CONDITION> [argument]", e.g. "UNHIDE_M_RUNNING CR2-Yell-CP": the action
// is the token's prefix (TELEP_, HIDE_, RESET_, STARTANIM_, STOP_, UNHIDE_, STREAM_, UNSTREAM_), the
// condition the first of M_CLOSED, M_TELEPORT, M_START, M_END, M_OPEN, M_NOTOPEN, M_RUNNING,
// M_WONCLOSED, M_WON, R_RUNNING, L_OPEN, P_PLAYER, P_DIED, PLATEFORM_WII, G_DEMO, LAUNCHED_FROM_STORY,
// LAUNCHED_FROM_MENU found in it (strstr, in that order); the argument is compared by its
// case-insensitive hash (0x669160), so names match without case.
#pragma once

#include <cstdint>
#include <functional>
#include <optional>
#include <string>
#include <vector>

namespace fuel {

struct ObjectCondition {
    // Condition kinds (rule +0): 0x513010's numbering.
    enum Kind {
        None = 0, Teleport = 1, Closed = 2, NotOpen = 3, Open = 4, Running = 5, Start = 6, End = 7, Won = 8,
        RtcRunning = 9, WonClosed = 10, LevelOpen = 11, Player = 12, Died = 13, FromStory = 14, FromMenu = 15,
        Wii = 16, Demo = 19
    };
    // Actions (rule +4).
    enum Action { NoAction = 0, Telep = 1, Hide = 2, Unhide = 3, Reset = 4, StartAnim = 5, Stop = 6, Stream = 7, Unstream = 8 };
    struct Rule {
        int kind = None;
        int action = NoAction;
        std::string arg;  // mission / RTC / level name (compared without case)
    };
    bool sons = false;            // "CONDITION_OBJECT SONS": the rules also hide/show every node below
    bool hideByRtc = false;       // HIDEBYRTC (+0xa)
    bool defaultHidden = false;   // DEFAULTHIDE (+9 cleared): hidden at load and before the rules
    bool dontHideUnhide = false;  // DONTHIDEUNHIDE (+8): the rules never change the visibility
    bool blocksRoad = false;      // IA_BLOCK_ROAD
    std::vector<Rule> rules;      // in UserDefine order (the engine keeps at most 4)
};

// 0x513010 on a node's UserDefine lines; nothing when there is no CONDITION_OBJECT line
// (0x53f710 looks for that keyword; its argument "SONS" sets `sons`).
std::optional<ObjectCondition> parseObjectCondition(const std::vector<std::string>& userDefine);

// What 0x513630 asks the game. Mission questions take the mission name (EMD name, +0xda).
struct ObjectGameState {
    std::string runningMission;             // M_RUNNING: the race state is past 0/1 and this is its mission
    bool launchedFromMenu = false;          // LAUNCHED_FROM_MENU / _STORY (game +0x328c -> +0x30)
    std::function<bool(const std::string&)> missionOpen;  // M_OPEN / M_NOTOPEN / M_CLOSED (0x5628d0); unset: open
    std::function<bool(const std::string&)> missionWon;   // M_WON / M_WONCLOSED (0x562890); unset: not won
    // P67: a cut-scene plays (any RtcScriptAgent_G with +0x12d set, 0x539990 / 0x539920; 0x5399c0 also wants the RTC
    // to have no ENDOFRTC event 0x96, which none of the shipped ones has): R_RUNNING and HIDEBYRTC.
    bool rtcRunning = false;
};

// Events 0x513630 receives (its second argument: kind, -, name).
struct ObjectEvent {
    int kind = ObjectCondition::None;  // Teleport, Start, End or Died
    std::string mission;
};

// 0x513630: whether the object ends up hidden, or nothing when DONTHIDEUNHIDE leaves it alone (also HIDEBYRTC without
// an R_RUNNING rule while no RTC plays: FUEL then returns before the rules and leaves the node as it is).
// `actions` (optional) receives every rule action that fired, in order (TELEP_, RESET_, STREAM_ ...).
std::optional<bool> evaluateObjectCondition(const ObjectCondition& c, const ObjectGameState& state, const ObjectEvent& event = {},
                                            std::vector<const ObjectCondition::Rule*>* actions = nullptr);

}  // namespace fuel
