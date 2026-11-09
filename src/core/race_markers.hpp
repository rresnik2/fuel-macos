// FUEL's race markers: the start / finish banners and the checkpoint flags it spawns as game objects (P69, decoded in
// ~/re-work/fuel/notes/p69-objects-scripting.md section M).
//  - VisibleObjectMgr_G (vt[0] 0x4fc6f0 builds the visible-object list; vt[17] 0x4fc9d0 runs it every frame): each
//    frame every marker record is marked unused (0x4fbe00), the visible objects re-place theirs (lookup-or-insert
//    0x4fbe80 clears the mark), 0x4fbe30 releases the records still unused (their agents go away), 0x4fbf10 puts each
//    record's agent on the ground.
//  - GeneratedOfflineVisibleObject vt[4] 0x4f83d0: in free roam (race state 0 / 1) the start banners of every mission
//    whose grid cell is within 10000 m ([0xa2a4e0]) of the player, with banners on (+0) and not being launched
//    (+0x20b); during a race 0x4f73b0 (Mission_PlaceMarkers) for the running mission (race +0x294).
//  - 0x4f8680 fills one record: type, position (offset sideways by the half-width), rotation (types < 9 turned 180
//    degrees about y), text; 0x4fb980 spawns the record's agent from the type's class (one clone.tsc / code class per
//    type, the Lod_Z of that name in USA1.DPC).
//  - 0x4fbf10 per frame: y = ground + record +0x38 (-0.3) (0x6407e0 within 1000 m of the player, else 0x445500); all
//    these markers have +0x6f set: upright, heading = the rotation's z axis flattened (0x410510, 0x40fc80, 0x6443a0).
// The checkpoint flag kind comes from the level's mission build 0x4f40f0 (by the road under the checkpoint).
// Nothing of the game is compiled in: the class names are looked up in the player's USA1.DPC, PP360 is read from pp001.tsc.
#pragma once

#include "races.hpp"

#include <array>
#include <functional>
#include <string>
#include <vector>

namespace fuel {

struct RaceMarker {
    // Record +0x64. 3 start middle (with the mission name), 5 / 6 start or finish left / right, 7 finish middle
    // ("Finish"), 9 a checkpoint centre (no object), 10 / 11 flag left / right, 12 / 13 flag left02 / right02,
    // 14 / 15 the light at the current checkpoint (left / right).
    int type = 0;
    int index = 0;                 // +0x40: the mission (banners) or the checkpoint (flags)
    std::array<float, 3> pos{};    // +0x10
    std::array<float, 4> rot{0, 0, 0, 1};  // +0x00, quaternion x y z w
    std::string text;              // +0x44 (drawn on the middle banners)
    float lift = -0.3f;            // +0x38 (0xbe99999a)
    // P79: records of other visible objects (type 16, DiscoveryPointVisibleObject) name their class themselves; null =
    // raceMarkerClass(type). upright = +0x6f (heading only); else 0x4fbf10 tilts the object by the ground normal.
    const char* cls = nullptr;
    bool upright = true;
};

// The class 0x4fb980 spawns for a record type (0x4f8680's switch); null for types without an object (8, 9).
const char* raceMarkerClass(int type);

struct RaceMarkerContext {
    // Full gate width at (x, z) (0x4fdea0, race_rules.hpp missionGateWidth); unset: 20.
    std::function<float(float x, float z)> gateWidth;
    // The checkpoint flag the level's mission build gives a checkpoint (0x4f40f0, checkpointMarkerFlag); unset: the
    // mission's own flag.
    std::function<int(int flag, float x, float z)> flagAt;
    float minHalf = 8.0f;  // PP360: the smallest half-width a marker stands off the centre line
};

// 0x4f40f0: a checkpoint whose flag is not 0 or 4 gets flag 1 (CHECKPOINT_FLAG_LEFT / RIGHT) or 2 (..._LEFT02 /
// _RIGHT02) by the road under it: nearest road record within 1000 m of type t: t < 0 or (t > 16 and t != 20) -> 1,
// else 2; no road within 1000 m: 2 when 0x6d1d70 finds a road at the point, else 1. Scenario 11 (helicopter): all 0.
int checkpointMarkerFlag(int flag, int scenario, int nearestRoadType, bool roadAtPoint);

// 0x4f73b0: the markers of the running mission. `target` = the player's current gate in the lap (racer passed %
// (n + 1); the light flags go on checkpoint `target`).
void missionMarkers(const Race& r, int missionIndex, int target, const RaceMarkerContext& c, std::vector<RaceMarker>& out);
// 0x4f83d0 (free roam): the start banners of one mission (when its banners are on).
void startBanners(const Race& r, int missionIndex, const RaceMarkerContext& c, std::vector<RaceMarker>& out);

// 0x4fbf10: the object's world matrix (row vectors, translation in 12..14) for a marker on ground height `ground`.
// Records without +0x6f (upright false) are turned so their up axis follows `normal` (0x640690, the detailed ground's
// normal; [I] the shortest arc from (0, 1, 0)).
std::array<float, 16> raceMarkerWorld(const RaceMarker& m, float ground, const std::array<float, 3>& normal = {0, 1, 0});

// P79: DiscoveryPointVisibleObject vt[4] 0x4ed2d0, the points of interest VisibleObjectMgr_G keeps as game objects (the
// POI list of AddTypePointOfInterest 0x54f760 and AddVehicleLivery 0x5756e0, notes p44 F1). Every frame, for each POI
// whose (x, z) is within 1000 m of the player (d^2 <= 1e6), a record of type 0x10 (+0x64) at (x, 0, z) with the POI's
// name as text; the agent by POI type: 1 IT_PILOTLIVERY_DYN, 2 a vehicle livery not taken (the livery's vehicle, painted,
// 0x4ed6e0), 3 IT_DISCOVERY (vista, always; upright +0x6f; alpha clamp(d x 0.05, 0.1, 1) within 20 m; anim slot 335
// once visited), 4 CAMIONHUB_RESPAWN (teleport / heliport; on the ground normal), 5 IT_FUEL01. Retail has 190 type 4,
// 95 type 3 and 148 type 2 entries (p44).
struct PoiPoint {
    int type = 0;
    float x = 0, z = 0;
    bool taken = false;  // +4
    std::string name;    // +0x14
};
const char* poiMarkerClass(int poiType);
// poi.tsc's AddTypePointOfInterest lines (0x54f760: "Teleport" -> 4, "VP" -> 3; x = arg 3, z = arg 4, name arg 5).
std::vector<PoiPoint> parsePoiScript(const std::string& text);
// PK09: the world POI list as FUEL indexes it: hubinfos.tsc's AddVehicleLivery ... POI x z lines (0x5756e0 at 0x575855:
// type 2, name = the mesh) appended first (the script runs before poi.tsc, 0x559895 / 0x5598c7), then poi.tsc's
// points, then the grid rebuild's sort (0x4ecf00 -> 0x735ac0). Index i of the result = FUEL's POI index.
std::vector<PoiPoint> worldPoiList(const std::string& hubinfosText, const std::string& poiText);

// PK09: 0x735ac0, the sort of the POI list (+0x94 count, +0x98 0x4c-byte records) at every grid rebuild 0x4ecf00
// (DiscoveryPointVisibleObject reset 0x4ecec0): an exchange sort by the type (+0, int) ascending, for i, for j > i,
// key[j] < key[i] -> records i and j swap through a temp copy (0x735c20). Unstable: equal types do not keep their order.
template <class T>
void poiExchangeSort(std::vector<T>& v) {
    for (size_t i = 0; i < v.size(); ++i)
        for (size_t j = i + 1; j < v.size(); ++j)
            if (v[j].type < v[i].type) std::swap(v[i], v[j]);
}
// 0x4ecf00 after the sort, per record in index order: type 0 -> 5 at an odd index, rand() % 5 + 1 at an even one
// (no type-0 record exists in retail: every script entry is typed).
template <class T>
void poiAssignUntyped(std::vector<T>& v, const std::function<int()>& rnd) {
    for (size_t i = 0; i < v.size(); ++i)
        if (v[i].type == 0) v[i].type = (i & 1) ? 5 : rnd() % 5 + 1;
}

// PK09: the POI grid of 0x4ecf00: 0x4edb00 builds (-70000, -70000)..(70000, 70000) in cells of 5000 m (29 x 29, width =
// round(size / cell) + 1), 0x4edc10 inserts each index into the cell ftol((p - origin) / cell) (in index order). Queries
// 0x4edc70 / 0x4edd60 / 0x4ede90: the cells ftol((p -+ r - origin) / cell) clamped to the grid, row by row (x inner, z
// outer), each cell's entries newest (highest index) first.
class PoiGrid {
public:
    static constexpr float kMin = -70000.0f, kMax = 70000.0f, kCell = 5000.0f;  // 0x9dc814 / 0x9dc810 / 0x9dc3f8
    void build(const std::vector<std::array<float, 2>>& xz);
    // Calls fn(index) for each entry of the cells covering (x +- r, z +- r); fn returns false to stop.
    void query(float x, float z, float r, const std::function<bool(int)>& fn) const;

private:
    int w_ = 0, h_ = 0;
    std::vector<std::vector<int>> cells_;
};
// The records 0x4ed2d0 keeps for the player at (px, pz). Liveries (type 2) are left out [port: the livery vehicle is not
// drawn]; the vista's distance fade and found animation are not modelled.
void poiMarkers(const std::vector<PoiPoint>& pois, float px, float pz, std::vector<RaceMarker>& out);

// PP360 from the player's pp001.tsc (8 in the shipped file); `fallback` when absent.
float loadMarkerMinHalf(const std::string& pp001Path, float fallback = 8.0f);

}  // namespace fuel
