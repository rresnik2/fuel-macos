// PK21: FUEL's race editor, the parts that are not drawing (notes/pk21.md; decode p98 b030 / b032 / b040 / b041).
//
// An edited race is FUEL's 0x494-byte record; the career keeps ten of them at game +0x410 (SECTION_EDITEDRACE_INFOS,
// CareerState::editedRaces, saved as is). The retail Editor tab (InGamePauseHUD +0x934 states 5 / 0xa1..0xac, its
// EditorHUD +0x49c with the retail flag +0x1b99) edits one slot in place on the map:
//   record        0x45fab0 ctor, point ctor 0x460b40, point i 0x45f860 / 0x45fb80, closed loop 0x45f940
//   slots         free count 0x569060 (point count +0x10 < 2), first free 0x569190, k-th used 0x569200
//   map editing   0x4651f0 (keys: MENU_VALID game+0x208 add, MENU_CANCEL +0x20c delete, MENU_VALI2 +0x238 pick / drop),
//                 snap 0x460220 / 0x4602a0, drop rule 0x464ac0 (RoadGraph_DistToJunction 0x6d1de0 >= 30 m), append
//                 0x464b50, remove 0x4646b0 / 0x464680, nearest 0x465830, move 0x465660, too long 0x465150, cursor
//                 0x464be0, open 0x4630a0
//   length        0x4608a0 (road route between road points, else the straight x z distance); km = trunc((m + 500) / 1000)
//   race types    0x463950 .. 0x463c00 (the debug panels' availability tests; the retail commit takes 0x4664d0's type)
//   new race      0x4664d0 (AI counts and options cleared, option 3 = 1, timer 0; loop -> type 4 + loop flag, 2 points ->
//                 type 0, else type 1); the caller adds 1 to game +0x3438 (races created) and saves
//   commit        0x465be0 (the mission record of TRY RACE): start = point 0, finish = the last point, checkpoints in
//                 between (flag 1, 2 for gate points), mission +0x58 scenario (type 0 -> 1, 4 -> 6, else 2), +0x5c laps
//                 = the loop flag, +0x50 AI = sum of the class counts (vehicles 0x5d65f0), +0x48 timer (no AI only),
//                 +0x54 options 0x45ff30, gate orientation 0x468740 (0x468290 road route 1 m ahead / 0x468580 gates)
#pragma once

#include "core/events.hpp"
#include "core/races.hpp"

#include <array>
#include <cstdint>
#include <functional>
#include <string>
#include <vector>

namespace fuel {

class RoadGraph;

struct EditedRace {
    static constexpr size_t kBytes = 0x494, kSlots = 10, kMaxPoints = 32;
    struct Point {                       // 0x24 bytes (ctor 0x460b40)
        float x = 0, y = 0, z = 0;       // +0
        float rot[3] = {0, 0, 0};        // +0xc rotation vector (identity)
        float weight = 1;                // +0x18
        uint32_t u1c = 0;                // +0x1c
        uint8_t gate = 0;                // +0x20 gate flag (the point was not snapped to a road)
        uint8_t valid = 1;               // +0x21
    };
    uint8_t ai[7] = {};                  // +0..+6 AI count per vehicle class
    uint16_t timer = 0;                  // +8
    uint8_t loop = 0;                    // +0xa
    uint8_t opt[4] = {};                 // +0xb..+0xe race options 0..3
    int8_t type = -1;                    // +0xf race type (-1 none)
    uint8_t count = 0;                   // +0x10 points (<= 32)
    std::array<Point, kMaxPoints> pts{}; // +0x14

    static EditedRace read(const uint8_t* p);  // a 0x494-byte record
    void write(uint8_t* p) const;
    bool closedLoop() const;             // 0x45f940: count > 1 and |first - last| (x z) < 1
    int aiTotal() const;                 // 0x45fc70
    bool noGate() const;                 // 0x464850
    bool gatesButLast() const;           // 0x4648f0
    bool typeAvailable(int t) const;     // 0x463950 .. 0x463c00 (race type panel)
    uint16_t optionFlags() const;        // 0x45ff30 -> mission +0x54
};

// What the editor needs of the world. `graph` null: no road -> no point can be dropped (0x4a0670 == 0).
struct EditorWorld {
    const RoadGraph* graph = nullptr;
    std::function<float(int type)> flatHalf;         // road type T+0xcc (0x6d1d70)
    std::function<float(float x, float z)> ground;   // 0x445500 (unset: 0)
    std::function<bool(float x, float z)> hubOpen;   // 0x4a7f40 + 0x577500 + 0x579720: the point's hub is unlocked
};

namespace editor {

// Slot helpers over the career's 0x2dc8-byte block.
int freeSlots(const std::vector<uint8_t>& block);         // 0x569060
int firstFreeSlot(const std::vector<uint8_t>& block);     // 0x569190, -1 none
int usedSlot(const std::vector<uint8_t>& block, int k);   // 0x569200, -1 none
EditedRace slot(const std::vector<uint8_t>& block, int i);
void setSlot(std::vector<uint8_t>& block, int i, const EditedRace& r);
std::string slotName(int i);                               // "Edited Race %02d" (i + 1)
int slotOfName(const std::string& name);                   // inverse, -1 when not an edited race's name
void initBlock(std::vector<uint8_t>& block);               // 0x568ad0: every slot type 0xff, count 0

float routeLength(const EditedRace& r, const EditorWorld& w);  // 0x4608a0, metres
int lengthKm(float metres);                                    // trunc((m + 500) x 0.001)
bool tooLong(float metres);                                    // km > 150 (0x465150 / 0x475d80)
float distToJunction(const RoadGraph& g, float x, float z, float r);  // RoadGraph_DistToJunction 0x6d1de0 (-1: no road)
bool newRaceDefaults(EditedRace& r);                           // 0x4664d0 (false: fewer than 2 points)
// 0x465be0: the race TRY RACE starts (and the port's session runs) for slot `i`.
Race buildRace(const EditedRace& r, int i, const EditorWorld& w);
// The used slots of the career's block as playable events ("Edited Race NN", kind Edited, no hub list).
std::vector<Event> editedEvents(const std::vector<uint8_t>& block, const EditorWorld& w);

// The retail EditorHUD's map state (+0x1b38 cursor, +0x19e8 / +0x19f0 zoom, +0x1d7c picked point) over one record.
class Session {
public:
    enum Key { Add, Remove, Pick };      // game +0x208 / +0x20c / +0x238
    // 0x4630a0: cursor on the first point, or the player (< 2 points); nothing picked.
    void open(EditedRace* r, float playerX, float playerZ, const EditorWorld& w);
    // 0x4651f0 for one key press: 0 nothing, 1 changed, 2 the route got too long (popup 0xab; the add is undone),
    // 4 the picked point moved (not for keys), 3 the cursor moved.
    int key(Key k);
    // 0x464be0 + 0x4ac3d0 per frame: stick (sx right, sz up, |s| <= 1) and zoom keys (+1 in = 0x21c, -1 out = 0x220).
    // Returns 4 when a picked point follows the cursor, 3 when the cursor moved, else 0 (the rest of 0x4651f0).
    int update(float dt, float sx, float sz, int zoomDir, float pp47, float pp48);
    float pickRadius() const { return zoom * 30.0f * 0.00066666666f; }  // zoom x 30 / 1500
    bool cursorPlaceable() const;        // !0x464a50: the snapped cursor can take a point
    // 0x460220: the cursor snapped to the nearest road within the pick radius (gate = 0) or as is (gate = 1).
    std::array<float, 3> snapped(uint8_t* gate) const;
    int nearest(int exclude, float limitSq) const;  // 0x465830 (x z), -1

    EditedRace* race = nullptr;
    EditorWorld world;
    float cx = 0, cy = 0, cz = 0;        // +0x1b38 (+0x1b28 the 3-D copy)
    float zoom = 2000, zoomTarget = 2000;  // +0x19e8 / +0x19f0 (MapHUD ctor 2000)
    float hx = -1, hz = 0;               // +0x19f4 / +0x19f8
    int picked = -1;                     // +0x1d7c[0]

private:
    bool dropAllowed(uint8_t gate, const std::array<float, 3>& p) const;  // 0x464ac0 (retail instance)
    void append(const std::array<float, 3>& p, uint8_t gate);            // 0x464b50
    void remove(int k);                                                   // 0x4646b0
    void move();                                                          // 0x465660
    bool overLimit(bool drop);                                            // 0x465150
};

}  // namespace editor
}  // namespace fuel
