// FUEL's career: stars and medals, the fuel economy, unlocks (hubs, events, vehicles,
// liveries), local achievements and the save file. Decoded from FUEL.exe by P39; see
// ~/re-work/fuel/notes/p39-career.md. Every rule value (fuel rewards, prices, stars a
// hub needs, unlock conditions, trophies) is read at run time from the game's scripts
// (Story/hubinfos.tsc, VehiclesDef.tsc, VehiclesUnlock.tsc, Trophy.tsc, poi.tsc), the
// same way FUEL.exe's console commands read them.
//
// Where it comes from (FUEL.exe addresses; game = [0xa7c094]):
//   race end            0x6300c0 (mission state 6 = MISSION_ENDING_WON) -> 0x630dd0 career,
//                       0x630ca0 challenge, then the console command "SaveGameData"
//   stars               0x578b50: career race best rank 0/1/2 = 3/2/1 stars
//   medals and fuel     mission +0x11b+i (i = 0 gold, 1 silver, 2 bronze), AddMissionFuel
//                       0x576420 -> mission +0x94+4i; game +0x408 = fuel, capped 99,999,999
//   hubs                SetHubTargetInfos 0x575290/0x578a20 (stars needed, mission needed),
//                       unlock test 0x579720, new unlocks after a race 0x578aa0,
//                       entering a hub 0x481cd0 (career races + 3 challenges available)
//   vehicles            AddVehicleInfo 0x56aa90, AddVehicleUnlock 0x56b110 (price, race,
//                       code), unlock 0x569e60, buy 0x4ce590 (fuel -= price)
//   unlock rules        UnlockNeed 0x54cdf0, check loop 0x5641c0, condition 0x564560,
//                       apply 0x564a70
//   trophies            AddNewTROPHY 0x56cf50, AddConditionTROPHY 0x56cf90/0x56c1f0,
//                       evaluation 0x56c340
//   save                writer 0x566050 (sections 0x566110..0x567640), reader 0x567ae0
#pragma once

#include "upgrades.hpp"  // [upgrades U2] upg::Stages, kCategories (header only: fuelcore needs no physics)

#include <algorithm>
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

namespace fuel {

// ---------------------------------------------------------------- rules (scripts)

struct CareerRules {
    struct Mission {               // FUEL's 0x210-byte mission record, the parts the career uses
        std::string name;          // as the scripts spell it (matched without case, _stricmp)
        float fuel[3] = {0, 0, 0}; // AddMissionFuel: reward per medal (gold, silver, bronze) -> +0x94
        float times[3] = {0, 0, 0};// AddMissionTime a b c -> +0x80..+0x88
        bool world = false;        // P99: a MISSION_COURSE node of the world (0x4f4db0), not an EMD block: absent from
                                   // the hub lists of saves written before P42 (57 career races; see deserialize)
    };
    struct Livery {                // AddVehicleLivery MESH n "hub" text KIND [arg x z] (0x5756e0)
        std::string mesh, kind, arg;  // kind ALWAYS / POI / CHEAT
        int index = 0;
        uint32_t text = 0;
        float x = 0, z = 0;
    };
    struct Traffic {               // AddHubTrafficVehicle hub Radar_x MESH x0 x1 z0 z1 mission (0x574fb0)
        std::string radar, mesh, mission;
        float x0 = 0, x1 = 0, z0 = 0, z1 = 0;  // P61: the box its road loop is searched in (0x5c4370; entry shorts x 2)
        // Entry +4: 0 Radar_VistaPoint, 1 Radar_VehicleLivery, 2 Radar_Challenge, 3 a vehicle (any other word).
        int kind() const;
    };
    struct Hub {
        std::string name;          // "YellowStone"
        int tier = 0;
        uint32_t nameText = 0;
        int starsNeeded = 0;       // SetHubTargetInfos 5th arg -> hub +0x23c
        std::string missionNeeded; // 6th arg ("NONE") -> hub +0x244
        std::vector<int> tutorial, career, challenges;  // mission indices (+0x20c, +0x214, +0x21c)
        int vistas = 0;            // poi.tsc "VP" points of this hub (+0x234)
        // P44: the hub's points of interest (AddTypePointOfInterest 0x54f760, poi.tsc order): "VP" = POI type 3
        // (IT_DISCOVERY) with a vista record whose +4 = entry +0x48 = the vista's text id; "Teleport" = type 4
        // (CAMIONHUB_RESPAWN), the heliports of the map page.
        struct Point {
            uint32_t text = 0;     // vista text id (arg 2); 0 for teleports
            float x = 0, z = 0;
            std::string name;      // "YellowStone_VP_1"
        };
        std::vector<Point> vistaPoints, teleports;
        std::vector<Traffic> traffic;   // +0x22c
        std::vector<Livery> liveries;   // +0x224
    };
    struct Vehicle {               // FUEL's 0x10c-byte vehicle record (game +0x3258)
        std::string mesh;          // "MESH_ID_QUAD_ROAD_2"
        std::string label;         // "QUAD_ROAD_2" (the port's variant name)
        std::string family;        // AddVehicleInfo class
        int cls = -1;              // 0x56a580: BUGGY 0 BIKE 1 MUSCLE 2 QUAD 3 SUV 4 TRUCK 5 HYDRO 6 DRAGSTER 7, UNKNOWN -1 (+0x38)
        int terrain = 2;           // 0x56a6a0: AddVehicleInfo 3rd argument -> +0x3c: OFF 0, ROAD 1, anything else 2 (readers not traced; data only)
        uint32_t nameText = 0;
        int price = 0;             // AddVehicleUnlock 2nd arg -> +0x34 (-1: never sold)
        std::string raceUnlock;    // 3rd arg -> +0x04 (crc): finishing this race gives the vehicle
        std::string code;          // 4th arg -> +0x08 (crc): bonus code
        bool custom = false;       // CUSTOM VEHICLES [port]: a port pack's vehicle (core/custom_vehicles.hpp): saved in
                                   // PORT_VEHICLES, not in FUEL's VEHICLES_INFOS; not counted by FUEL's trophies
        int level = -1;            // [upgrades U2] AddVehicleInfo 7th argument (the mesh = 1st): 1..5, gifts 6, -1 traffic /
                                   // tests [I: FUEL's reader not traced; the upgrade price tier, plan 3.1 / 3.2]
    };
    struct PilotLivery {           // AddPilotLivery PART n text "ALWAYS"|"NONE"|race [code] (0x56b920)
        std::string part;          // HEAD / CHEST / LEGS / BODY
        int index = 0;
        uint32_t text = 0;
        std::string unlock;        // "ALWAYS", "NONE" (code only) or a race name
        std::string code;
    };
    struct Unlock {                // UnlockNeed TARGETKIND target CONDKIND [arg] (0x54cdf0)
        std::string kind, target;  // e.g. VEHICLE MESH_ID_SUV_ALL_1
        std::string cond, arg;     // HUB "Valley 1" / ALWAYS / MISSION name
    };
    struct Trophy {                // AddNewTROPHY id grade credits Show|Hide b b + AddConditionTROPHY
        int id = 0;
        int grade = 0;             // BRONZE 1, SILVER 2, GOLD 3
        int credits = 0;
        bool hidden = false;
        int verb = 0, object = 0;  // tables at 0xa02a48 / 0xa02a78 (names below)
        std::string arg1, arg2;
        double value = 0;          // arg1 as a number (+0x3c float / +0x40 int)
    };

    std::vector<Mission> missions;   // every mission the scripts know (miss_official + missChallenge)
    std::vector<Hub> hubs;           // SetHubTargetInfos order (= SortHubListByUID)
    std::vector<Vehicle> vehicles;   // VehiclesDef order
    std::vector<PilotLivery> pilotLiveries;
    // PK09: the world POI list in the order the scripts append it (0x4c-byte records of the POI manager +0x94/+0x98):
    // hubinfos.tsc's AddVehicleLivery ... POI lines (0x5756e0, type 2; run first by 0x559895) then poi.tsc's
    // AddTypePointOfInterest lines (0x54f760, "Teleport" 4 / "VP" 3; 0x5598c7). `index` = the hub's livery / vista /
    // teleport index. FreeRoam sorts it (0x735ac0) as FUEL's POI grid rebuild 0x4ecf00 does.
    struct PoiRef {
        int type = 0, hub = -1, index = -1;
    };
    std::vector<PoiRef> poiScript;
    std::vector<Unlock> unlocks;
    std::vector<Trophy> trophies;

    int missionIndex(const std::string& name) const;  // -1 when unknown (case-insensitive)
    int hubIndex(const std::string& name) const;      // name compared without case
    int vehicleIndex(const std::string& meshOrLabel) const;
};

// Reads the rules from the extracted game folder (the one holding LocalizedComponent).
CareerRules loadCareerRules(const std::string& dataDir);

extern const char* const kTrophyVerbs[12];    // NONE FIND OWN WON RIDE JUMP FALL TRICK CREATE OPEN GOTO USE
extern const char* const kTrophyObjects[19];  // NONE VEHICLE STARS CHALLENGE ... HELIPORT

// ---------------------------------------------------------------- state (save)

// The 40-byte status block of a mission (mission +0x10c..+0x133), saved as is.
struct MissionStatus {
    uint8_t flag10c = 0;           // +0x10c [U]
    uint8_t available = 0;         // +0x10d: listed / startable
    uint8_t pad10e[2] = {0, 0};
    uint32_t bestScore = 0;        // +0x110: max of 0x7fffffff - centiseconds
    uint32_t lastScore = 0;        // +0x114
    uint8_t flag118 = 0;           // +0x118 [U]
    uint8_t bestPlace = 0xff;      // +0x119: lowest 1-based place (min) [default I]
    uint8_t lastPlace = 0;         // +0x11a
    uint8_t medal[3] = {0, 0, 0};  // +0x11b..+0x11d: gold, silver, bronze won
    uint8_t pad11e[2] = {0, 0};
    int32_t bestRank = 3;          // +0x120: 0 gold .. 2 bronze, 3 = none (challenges: 0 = won)
    int32_t lastRank = 3;          // +0x124
    uint8_t tail[12] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0, 0, 0, 0};  // +0x128/+0x12c = -1, +0x130 = 0 (0x4fe450)
};
static_assert(sizeof(MissionStatus) == 40, "mission status block is 0x28 bytes");

struct CareerState {
    char profile[2][33] = {};      // game +0x3c4, +0x3e5 (0x21 bytes each) [U: names]
    uint32_t fuel = 0;             // game +0x408
    uint32_t fuelFound = 0;        // game +0x40c: fuel barrels found (trophies "FIND fuel")
    uint8_t stats[0xb8] = {};      // game +0x33b8..+0x346f (see the accessors)
    uint8_t options[0x38] = {};    // game +0x24c [U]
    uint32_t options2a0 = 0;       // game +0x2a0 [U]

    // P66: +0xc = the chosen texture livery (0-based: MESH_ID_x_LIV<+0xc + 1>, garage livery page 0x4796e0), +0xd = the
    // palette colour (paint page 0x479a60); AddVehicleInfo 0x56aa90 zeroes both. (+0xe / +0xf are the online garage's.)
    struct Vehicle { uint32_t crc = 0; uint8_t unlocked = 0, owned = 0, livery = 0, color = 0; float f10 = 0; };
    std::vector<Vehicle> vehicles;
    struct HubState { uint8_t opened = 0, unlocked = 0; };  // hub +0x248 (entered), +0x249 (unlocked)
    std::vector<HubState> hubs;
    struct Flags2 { uint8_t a = 0, b = 0; };  // vista: revealed, visited; traffic/livery: revealed, owned / caught
    std::vector<std::vector<Flags2>> vistas, traffic, liveries;
    std::vector<uint32_t> trafficCrc, liveryCrc;  // per entry (flattened by hub) as saved
    std::vector<std::vector<MissionStatus>> tutorial, career, challenges;  // per hub, list order
    std::vector<uint8_t> pilotOwned;   // per pilot livery (entry +0)
    // P66: game +0x3264: per pilot part type (crc("HEAD1"), script order, EndOfParsePilotLivery 0x56bc00) the texture
    // livery last chosen for it (-1 = none yet: 3 x the first owned livery's ordinal - 2, 0x471e80).
    std::vector<std::pair<uint32_t, uint32_t>> pilotPairs;
    std::vector<uint8_t> editedRaces = std::vector<uint8_t>(0x2dc8, 0);  // EDITEDRACE_INFOS (editor not ported)
    std::vector<int> trophiesWon;      // port: achievements awarded locally (ids)
    // P62: the tutorial manager's entries (game+0x15c, 0x1c bytes each, content 0x56d900: 16 entries): +0 seen,
    // +1 off. Saved in SECTION_TUTORIAL (0x56e140: u32 n, n x {u32 index (+4), u8 seen, u8 off}), read by 0x56e0d0.
    struct Tutorial { uint8_t seen = 0, off = 0; };
    std::vector<Tutorial> tutorials = std::vector<Tutorial>(16);
    // P44, not saved (FUEL keeps them outside the save buffer):
    // hub travel history of the trophies object (game+0xe4: 8 x {from uid, to uid, valid}, count +0x60; 0x56ca20).
    struct Trip { int from = -1, to = -1; bool valid = true; };
    std::vector<Trip> trips;
    // ---- PHYSIQUE (port deviation, notes physique.md): the player's outfit (custom_rider.hpp RiderOutfit: 0 FUEL's rider,
    // 1 riding gear, 2 tank top + cutoffs, 3 shirtless + shorts) and whether the new head wears the garage's helmet. Saved in
    // the port's own section PORT_WARDROBE (FUEL's readers find sections by name, so the PC layout and PK25's import are
    // untouched; a save without it gets these defaults).
    uint8_t riderOutfit = 1, riderHelmet = 1;
    // ---- [DEVIATION upgrades U2] vehicle upgrades (core/upgrades.hpp; notes upgrades-plan.md 3 / 6, upg2.md): per vehicle
    // (the `vehicles` order, FUEL's and the packs'), the highest stage bought and the stage equipped of each category
    // (upg::Category order e, s, t, w, np, nt), and the fuel spent on them. Saved in the port's own section PORT_UPGRADES
    // keyed by crc(mesh); FUEL's sections never hold it, so a save without it (older port builds, PC imports) is all stock.
    struct VehicleUpgrades {
        uint8_t owned[upg::kCategories] = {}, equipped[upg::kCategories] = {};
        uint32_t spent = 0;
        bool any() const {
            for (int c = 0; c < upg::kCategories; ++c)
                if (owned[c] || equipped[c]) return true;
            return spent != 0;
        }
    };
    std::vector<VehicleUpgrades> upgrades;  // sized with `vehicles` (newGame); missing entries read as stock
    // ---- [DEVIATION dev mode] (the user's request 2026-11-07; notes devmode.md) the overlay's own store: what the player chose
    // in dev mode that the real career could not hold (Career::setDevMode). Per vehicle (the `vehicles` order): the stages
    // equipped in dev mode (flag kDevStages; any stage, owned or not) and the livery / colour chosen in dev mode for a vehicle
    // not really owned (kDevLook). Saved in the port's own section PORT_DEVMODE (only when a record exists), never in FUEL's
    // sections or PORT_UPGRADES; kept (and ignored) while dev mode is off, so turning it on again gives the same choices.
    enum : uint8_t { kDevStages = 1, kDevLook = 2 };
    struct DevVehicle {
        uint8_t flags = 0;
        uint8_t equipped[upg::kCategories] = {};
        uint8_t livery = 0, color = 0;
    };
    std::vector<DevVehicle> dev;  // sized with `vehicles` (newGame); missing entries = no record
    // the running race's flags (race +0x64 crashed, +0x65 no GPS, +0x3b8 [U]), set at race start 0x62d74c.
    uint8_t raceCrashed = 0, raceNoGps = 1, race3b8 = 0;

    // Named fields of the 0xb8-byte block (offsets from game +0x33b8).
    int32_t& currentHub() { return at<int32_t>(0x08); }      // +0x33c0 hub uid (TeleportToHub on load)
    int32_t& currentVehicle() { return at<int32_t>(0x10); }  // +0x33c8 vehicle index
    // P66: the rider (garage Driver pages commit 0x4715c0; applied on load by 0x42b420): +0x33d0 head type (HEAD<n>),
    // +0x33d4 chest type, +0x33d8 legs type, +0x33dc / +0x33e0 / +0x33e4 their texture liveries (LIVERY_<k>_HEAD<n>),
    // +0x33e8 body livery, +0x33ec skin (body type BODY<n>). Values < 1 mean "default" (Career::pilotLook).
    int32_t& pilot(int k) { return at<int32_t>(0x18 + 4 * size_t(k)); }  // k: 0 head .. 7 skin (PilotLook order)
    // P44 distances (0x569460): u32 km + f32 m pairs; free ride +0x3408, race +0x3410, on road +0x3418, off road +0x3420.
    uint32_t& freeRideKm() { return at<uint32_t>(0x50); }
    float& freeRideM() { return at<float>(0x54); }
    uint32_t& raceKm() { return at<uint32_t>(0x58); }
    float& raceM() { return at<float>(0x5c); }
    uint32_t& onRoadKm() { return at<uint32_t>(0x60); }
    float& onRoadM() { return at<float>(0x64); }
    uint32_t& offRoadKm() { return at<uint32_t>(0x68); }
    float& offRoadM() { return at<float>(0x6c); }
    float& longestAirTime() { return at<float>(0xa0); }       // +0x3458 (0x4c8450)
    float& topSpeed() { return at<float>(0xa4); }             // +0x345c (0x42d730)
    float& longestJump() { return at<float>(0x70); }          // +0x3428 (trophy JUMP)
    float& longestFall() { return at<float>(0x74); }          // +0x342c (trophy FALL)
    uint8_t& noCrashWin() { return at<uint8_t>(0x79); }       // +0x3431 (CAREER_NO_CRASH)
    int32_t& tricks() { return at<int32_t>(0x7c); }           // +0x3434 (TRICK)
    int32_t& racesCreated() { return at<int32_t>(0x80); }     // +0x3438 (CREATE Race)
    uint8_t& noGpsWin() { return at<uint8_t>(0x84); }         // +0x343c (CAREER_NO_GPS)
    int32_t& heliportUses() { return at<int32_t>(0xa8); }     // +0x3460 (USE HELIPORT)
    int32_t& hubsUnlockedByRaces() { return at<int32_t>(0xb4); }  // +0x346c (++ in 0x630dd0)
    template <class T> T& at(size_t off) { return *reinterpret_cast<T*>(stats + off); }
    template <class T> T get(size_t off) const { T v; std::memcpy(&v, stats + off, sizeof v); return v; }
};

// P66: the rider's look in game-field order (+0x33d0..+0x33ec).
struct PilotLook {
    int head = 1, chest = 1, legs = 1;                   // part types: meshes Head<n> / Chest<n> / Legs<n>
    int headLivery = 1, chestLivery = 1, legsLivery = 1;  // texture liveries LIVERY_<k>_HEAD<n> ...
    int bodyLivery = 1, skin = 1;                         // LIVERY_<k>_BODY<skin>
    int& at(int k) { return (&head)[k]; }
    int at(int k) const { return (&head)[k]; }
};

// What one finished event changed (the results pages and --career-check use it).
struct CareerAward {
    bool counted = false;          // the event is in FUEL's hub lists and was won
    int newMedals = 0;             // stars won now (CareerRaceReward ITEM_ID_NBSTARS)
    bool medalNew[3] = {}, medalHad[3] = {};  // this race's medals: won now / had before
    uint32_t fuelWon = 0;          // ITEM_ID_FUEL_REWARD4
    std::vector<std::string> hubsUnlocked, vehiclesUnlocked, pilotLiveries, trafficRevealed;
    std::string giftVehicle;       // P99: the vehicle the race itself gives (0x56a320 -> MissionHUD +0xe90, BonusCar page);
                                   // vehiclesUnlocked also holds what UnlockNeed opened after the race
    std::vector<int> trophies;     // achievements newly awarded
};

class Career {
public:
    explicit Career(CareerRules rules);

    const CareerRules& rules() const { return rules_; }
    CareerState& state() { return state_; }
    const CareerState& state() const { return state_; }

    // InitEmptySave 0x54ec80: the state the scripts leave (hubs with 0 stars needed
    // unlocked, ALWAYS rules applied, free vehicles owned).
    void newGame();

    int stars() const;                    // 0x578b50
    int challengesWon() const;            // 0x5797e0
    bool hubUnlockTest(int hub) const;    // 0x579720
    bool hubUnlocked(int hub) const { return state_.hubs[size_t(hub)].unlocked != 0; }
    // Entering a hub from the map (0x481cd0): opened, its career races and its first
    // three challenges become available. Returns false when the hub is locked.
    bool enterHub(int hub);

    // Where an event sits in FUEL's lists: hub, list (1 career, 2 challenge) and index;
    // false for names the hub lists do not hold (FUEL drops them, 0x575370).
    bool locate(const std::string& event, int* hub, int* list, int* index) const;
    MissionStatus* status(const std::string& event);
    const MissionStatus* status(const std::string& event) const;
    bool eventAvailable(const std::string& event) const;

    // Race end (0x6300c0). `won` = mission state 6 (MISSION_ENDING_WON: finished 1st in a race); `rank` = race +1000
    // (+0x3e8), the race's level 0 gold / 1 silver / 2 bronze (medals: the career page's medal row, not the place; a
    // career win gives the medals rank..2 not owned yet, only when rank beats the best); `place` = 1-based place (race +0x270);
    // `seconds` = race time. Applies medals, fuel, unlocks and trophies.
    CareerAward finishEvent(const std::string& event, bool won, int rank, int place, double seconds);

    // Garage (0x4ce590 buy). Owned vehicles can be driven.
    bool vehicleOwned(const std::string& meshOrLabel) const;
    bool vehicleUnlocked(const std::string& meshOrLabel) const;
    int vehiclePrice(const std::string& meshOrLabel) const;
    enum BuyResult { Bought, AlreadyOwned, Locked, TooExpensive, NotForSale };
    BuyResult buyVehicle(const std::string& meshOrLabel, std::vector<int>* trophies = nullptr);

    // ---- [DEVIATION upgrades U2] vehicle upgrades (notes upgrades-plan.md 3.2-3.4, 7; upg2.md). Per vehicle; no refund.
    // Buying stage n of a category buys the stages below it, charges their sum and equips it; any owned stage (stock
    // included) can be equipped for free. Gates: the vehicle owned (as LIVERY / PAINTWORK, 0x47c050), stage 2 = 20 stars,
    // stage 3 = 45 (0x578b50). The fuel goes through FUEL's spend path (0x4ce590: the too-expensive test, fuel -= price on
    // game +0x408). The caller saves (the menus: Menu::saveCareer, as after a vehicle purchase).
    enum class UpgradeResult { Bought, AlreadyOwned, VehicleNotOwned, NeedsStars, TooExpensive, NotUpgradable };
    static const char* upgradeResultName(UpgradeResult r);
    bool upgradable(const std::string& meshOrLabel) const;    // a classed vehicle (+0x38 != -1: traffic and tests never)
    int vehicleLevel(const std::string& meshOrLabel) const;   // AddVehicleInfo level (-1 unknown vehicle)
    upg::Stages upgradesOwned(const std::string& meshOrLabel) const;
    upg::Stages upgradesEquipped(const std::string& meshOrLabel) const;
    uint32_t upgradesSpent(const std::string& meshOrLabel) const;
    // Fuel to own `stage` (1..3) of `category`: the prices of the stages above the owned one up to it (upg::price at the
    // vehicle's level); 0 when it is owned; -1 for a vehicle that is never upgraded or a bad category / stage.
    int upgradePrice(const std::string& meshOrLabel, int category, int stage) const;
    // What buying would answer, without buying (Bought = it can be bought); the order of the tests is buyUpgrade's.
    UpgradeResult upgradeLock(const std::string& meshOrLabel, int category, int stage) const;
    UpgradeResult buyUpgrade(const std::string& meshOrLabel, int category, int stage);
    bool equipUpgrade(const std::string& meshOrLabel, int category, int stage);  // false: stage not owned / bad args
    // The stages a session drives `vehicle` with (execSession's `--upgrades`, plan 7): its equipped stages when upgrades are
    // on, the vehicle is owned and not handed over by a challenge (AddMissionNeededVehicle); else stock. The stages stay.
    upg::Stages sessionUpgrades(const std::string& vehicle, bool upgradesOn, bool handedOver = false) const;

    // ---- [DEVIATION dev mode] (the user's request 2026-11-07: "Unlock all vehicles and upgrades for me in dev mode"; notes
    // devmode.md). An overlay, never progress: while on, every classed vehicle (FUEL's 75 with the gifts, the packs' vehicles;
    // not traffic / tests) counts as unlocked and owned (vehicleUnlocked / vehicleOwned, its ALWAYS liveries offered), and
    // every upgrade stage of it as owned (upgradesOwned = 3 each: no star gate, no fuel; buyVehicle / buyUpgrade answer
    // AlreadyOwned and spend nothing). The real records are never written while it is on: equipping goes to the dev store
    // (CareerState::dev, PORT_DEVMODE) and is free; a livery / colour of a vehicle not really owned likewise; the time
    // driven in such a vehicle is not added (its record +0x10). Hubs, events, fuel, stars, medals, trophies and the
    // unlock rules read the real records. Not saved in career.sav: the menus set it from settings.txt `dev_mode`.
    void setDevMode(bool on) { devMode_ = on; }
    bool devMode() const { return devMode_; }
    bool devOverlay(const std::string& meshOrLabel) const;  // dev mode on and a classed vehicle
    bool devStages(const std::string& meshOrLabel) const;   // the dev store holds stages for it (equipped in dev mode)

    // ---- garage (P66, notes/p66-garage-profile.md) ----
    // Vehicle livery page (0x47aa40): the vehicle's liveries are the hub entries of AddVehicleLivery for its mesh
    // (0x569c00), entry n owned (+1) giving texture liveries 3(n-1) .. 3(n-1)+2 (0-based, MESH_ID_x_LIV<k+1>).
    std::vector<int> vehicleLiveryChoices(const std::string& meshOrLabel) const;
    int vehicleLivery(const std::string& meshOrLabel) const;   // record +0xc (0-based)
    int vehicleColor(const std::string& meshOrLabel) const;    // record +0xd
    void setVehicleLivery(const std::string& meshOrLabel, int k);
    void setVehicleColor(const std::string& meshOrLabel, int c);
    void addVehicleTime(const std::string& meshOrLabel, float seconds);  // record +0x10 (0x5cb8c0)
    // Pilot liveries (AddPilotLivery "HEAD1" n ...): type "HEAD3" offered when one of its liveries is owned (0x4740f0);
    // texture livery k (1-based) of a type belongs to its entry n = (k - 1) / 3 + 1 (0x474320).
    bool pilotTypeOwned(const std::string& partType) const;
    const CareerRules::PilotLivery* pilotLivery(const std::string& partType, int n, bool* owned = nullptr) const;
    int pilotTypeLivery(const std::string& partType) const;    // pair list, default 3 x first owned - 2 (0x471e80)
    void setPilotTypeLivery(const std::string& partType, int k);   // 0x4721f0
    // The rider as 0x42b420 applies it: game fields, < 1 -> defaults (head: first HEAD<n> with an owned livery
    // 0x56b610 and its livery 0x56b6f0; chest 1 (4 in tutorial mode), legs 1 (3), liveries and skin 1).
    PilotLook pilotLook(bool tutorialMode = false) const;
    void setPilotLook(const PilotLook& p);                      // 0x4715c0
    // PHYSIQUE: the outfit (CareerState::riderOutfit / riderHelmet).
    int riderOutfit() const { return state_.riderOutfit; }
    bool riderHelmet() const { return state_.riderHelmet != 0; }
    void setRiderOutfit(int outfit, bool helmet) { state_.riderOutfit = uint8_t(std::clamp(outfit, 0, 3)), state_.riderHelmet = helmet ? 1 : 0; }

    // ---- free roam (P44, notes/p44-freeroam.md; the per-frame rules are in freeroam.hpp) ----
    // Current hub (0x5693b0: game+0x33c0 = uid) and the trophies' travel history (0x56ca20).
    void setCurrentHub(int hub);
    int currentHub() const { return state_.get<int32_t>(0x08); }
    // Map heliport flight (0x482b60 type 21): the open trip no longer counts (0x56ca00), heliport uses + 1.
    void useHeliport();
    // Trophy GOTO HUB a b (0x56cb20): a driven trip from hub a to hub b in the history.
    bool travelled(int from, int to) const;
    // Distance driven this frame (0x569460): free ride or race (mission running), on or off road.
    void addDistance(float metres, bool mission, bool offRoad);
    double freeRideDistance() const;  // 0x49ad40: km x 1000 + m (trophy RIDE FREERIDE)
    double raceDistance() const;      // 0x49ad20
    // Race flags (race +0x64 / +0x65): reset at race start (0x62d74c), updated per frame (0x632b50).
    void raceStart();
    void raceFrame(bool wrecked, bool gpsShown);
    // Collectibles. Each returns true when the state changed (FUEL then pushes a message and saves).
    uint32_t collectFuel(float amount);     // IT_DYNCOLLECT pickup 0x51d600: fuel += trunc, barrels + 1 / + 4
    bool visitVista(int hub, int k);        // 0x4ed0a0 type 3: visited = revealed = 1
    bool revealVista(int hub, int k);       // 0x4c01d0
    bool revealLivery(int hub, int k);      // 0x4c00c0
    bool pickLivery(int hub, int k);        // 0x4fcb60: owned = revealed = 1
    bool discoverChallenge(int hub, int k); // 0x4c0150: +0x10d = 1
    // P61: catching special traffic vehicle k of `hub` (0x5c53b0): caught = revealed = 1 and the reward by kind on the
    // hub (0x577380): 0 every vista revealed (0x57a060), 1 every vehicle livery revealed (0x57a390), 2 every challenge
    // +0x10d = 1 (0x57a1d0), 3 the vehicle owned (0x477f40 / 0x569eb0). Returns FUEL's message (10, 9, 11, 8), 0 when
    // it was already caught (nothing happens). The caller saves ("SaveGameData") and shows the message for 5 s.
    int catchTraffic(int hub, int k);
    bool trafficCaught(int hub, int k) const;
    int vistasVisited() const { return vistasFound(); }
    int vehicleLiveriesOwned() const { return ownedVehicleLiveries(); }
    int starsMax() const;                   // 0x578070: 3 x every hub's career count (economy_hud)

    // UnlockNeed check loop (CheckUnlock 0x5641c0); returns the targets it unlocked.
    std::vector<std::string> checkUnlocks();
    // Trophy evaluation (0x56c340) over every trophy not yet awarded; returns the new ones.
    std::vector<int> checkTrophies();
    bool trophyMet(const CareerRules::Trophy& t, std::string* progress = nullptr) const;

    // Save file: FUEL's section stream (0x566050) behind a small port header.
    std::vector<uint8_t> serialize() const;
    bool deserialize(const std::vector<uint8_t>& bytes, std::string* error = nullptr);
    bool save(const std::string& path = careerPath()) const { return write(path) == 0; }
    bool load(const std::string& path = careerPath()) { return read(path) == 0; }  // false (state untouched) when missing/bad
    // ---- PK25 (notes/pk25.md): save results as FUEL's SaveGamePc_Z codes (vf33 0x8403c0 *err) ----
    // write: 0 ok, 9 write failed (0x840000: CreateFileA GENERIC_WRITE fails on a read-only file or folder),
    // 0xe no space [port: ENOSPC / EDQUOT; the PC writer never returns it, MCAlert 0x441320 shows TT 9045 for it].
    int write(const std::string& path = careerPath()) const;
    // read: 0 ok, 7 cannot open (0x840190 CreateFileA), 8 empty, 0x17 corrupt (bad header, the '=' marker and CRC of
    // 0x4eb9c0, a section stream that does not parse). The state is untouched unless 0.
    int read(const std::string& path = careerPath());
    // FUEL's section stream alone (the payload of the FUELPSAV file; what 0x566050 packs on PC) and its reverse (the
    // PC career.sav import, core/save_import.hpp).
    std::vector<uint8_t> sections() const;
    bool deserializeSections(const std::vector<uint8_t>& stream, std::string* error = nullptr);

    static std::string careerPath();  // ~/Library/Application Support/FUEL Port/career.sav ($FUEL_CAREER_SAVE overrides)

    // ---- PK27: the options block (game +0x24c = options-struct +0xc0, saved in career.sav). newGame() gives it FUEL's
    // factory values (0x553c80(1): brightness 1.0, music / SFX / voice / online voice / +0xd8 levels 8, +0xe4 = 1,
    // vibration +0xe6 = 1, +0xe8 = 2, +0xed..+0xf0 = 1). voiceLevel() = +0x25c (options +0x10) clamped 0..16 as 0x554130.
    int voiceLevel() const;
    // ---- PK27 end

private:
    bool spendFuel(uint32_t price);             // 0x4ce590: false (nothing spent) when price > fuel; else fuel -= price
    CareerState::VehicleUpgrades* upgradesAt(const std::string& meshOrLabel);  // [upgrades U2] nullptr: unknown vehicle
    bool devOn(int vehicle) const;              // [dev mode] the overlay covers record `vehicle` (on, classed)
    bool devLook(int vehicle) const;            // [dev mode] the overlay's livery / colour apply (on, not really owned)
    CareerState::DevVehicle* devAt(int vehicle);  // [dev mode] the store's record (sized on demand)
    const CareerState::DevVehicle* devAt(int vehicle) const;
    bool conditionMet(const CareerRules::Unlock& u) const;
    std::string applyUnlock(const CareerRules::Unlock& u);
    void unlockVehicle(size_t v);
    void ownAlwaysLiveries(size_t v);           // 0x569b20(0)
    std::vector<std::string> newHubUnlocks();  // 0x578aa0
    int ownedVehicles() const;                  // 0x56a180
    int ownedVehicleLiveries() const;           // 0x57a330
    int ownedPilotLiveries() const;             // 0x56b7b0
    int vistasFound() const;                    // sum of 0x579b90

    CareerRules rules_;
    CareerState state_;
    std::vector<uint8_t> unlockDone_;           // UnlockNeed entry +0 (not saved: CheckUnLock runs after load)
    bool devMode_ = false;                      // [dev mode] settings.txt `dev_mode` (Menu), not the save's
};

// fuelview --career-check: plays a scripted career (events won and lost, fuel, hub
// unlocks, a purchase, save and reload) on a temporary file and checks every step
// against the decoded rules. Returns 0 when everything holds.
int runCareerCheck(const std::string& dataDir);

}  // namespace fuel
