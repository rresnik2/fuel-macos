// FUEL's off-road path finder (PATH_ATV_MANAGER vt 0x9b1a8c, ctor 0x4dc6b0; PATH_ATV_FINDER vt 0x9b1a68),
// decoded by P43 (notes/p43-water-ai.md "P") and P58 (notes/p58-traffic.md), ported 1:1 by P58.
//
// A weighted A* run backwards from the goal over a 128 x 128 window of 10 m cells (cell (i, j) at world
// (10 i, 10 j), keyed by i & 127, j & 127), 8 neighbours (orthogonal 10 m, diagonal 14.142 m), inside a box of
// +-63 cells around the midpoint of the start and goal cells (0x4dcf40). It fills a cost field (time to the
// goal); the path is a descent of that field from any start cell (0x61f470), so one search serves every point
// of the block. f = g + 2.0 x |start - cell| (0x4dcb70, weight 0x4dd000).
//
// Clients in FUEL: IAPathBlockOffRoad (0x624cf0 / 0x624e20; stretches < 700 m, only for path requests whose
// flag +4 is 0: the AI racers' requests (0x617fc0) set it to 1, so racers never use this finder),
// IAOffroadFollower_G (0x621e60). The port uses it from --atv-path and the vectors.
#pragma once

#include <array>
#include <cstdint>
#include <functional>
#include <vector>

namespace fuel {

// The world as the finder samples it.
struct AtvWorld {
    // PATH_ATV_ALT_Case 0x61eaf0 -> 0x61e980: the higher of the ground and a downward ray (mask 0x202/0x10fc1)
    // from 100 m above it against the static objects.
    std::function<float(float x, float z)> height;
    // 0x61ef30 -> 0x6fb2a0 (0x6fac40 + closest point): a road within r: writes the centreline distance and the
    // road type's AI blend (AGW_TerraRoad +0xb4, table 0xa677e0); false when none.
    std::function<bool(float x, float z, float r, float& dist, float& blend)> road;
    // 0x45dd90: GenWorld_ForestDensity 0x6bd200 at the cell (0..1).
    std::function<float(float x, float z)> forest;
    // 0x40fd10 -> 0x40fd70: water depth at the cell (<= 0 when dry; 0 when the world has no water).
    std::function<float(float x, float z)> water;
};

// PATH_ATV_Case (0x1c bytes; init 0x61e960, compute PathATV_CellInit 0x61eb30).
struct AtvCell {
    float c = 1;         // +0x0c: 1 off-road, -1 in water (blocked), on a road: d / 14.142 x 0.1 + type blend
    float slope = 1;     // +0x10: 1 / max(0.01, (n.y - 0.85) x 6.666668); >= 99 blocks
    float forest = 0;    // +0x14: forest density
    bool road = false;   // +0x18
};

class AtvPathFinder {
public:
    // Grid descriptor (manager +0x2c, ctor 0x4dc6b0): width 128, mask 127, 16384 cells, 10 m, 14.142 m.
    static constexpr int kWidth = 128, kMask = 127, kCells = 16384, kHalfBox = 63;
    static constexpr float kCell = 10.0f, kInvCell = 0.1f;
    static const float kDiag, kInvDiag;   // 0x4162462a, 0x3d90d0c7
    static constexpr float kUnreached = 1e13f;  // field init 0x551184e7

    explicit AtvPathFinder(AtvWorld world);

    // 0x6238a0: speeds of the request record (+8, +0xc; the vehicle's P613 / P614 for racers, 0.2 / 0.8 in
    // mode 4): +0x40 = 1 / offRoad, +0x48 = 1 / road. A change forgets the goal (the next request searches again).
    void setWeights(float offRoad, float road);
    // 0x623820: start point and cell (floor(p / 10 + 0.5), FUEL rounds down), box = the start cell +- 63.
    void setStart(float x, float y, float z);
    // 0x623580: goal point. A new goal cell: box around the start and goal cells, fails (false) when no
    // neighbour of the goal can be stepped to; otherwise the field is reset (goal 0) and the goal is pushed
    // (0x623730). The same goal cell keeps the search.
    bool request(float x, float y, float z);
    // 0x61f6a0: up to `budget` expansions; returns what is left of the budget when the open list empties
    // (0 when it ran out, or when the per-frame cell budget ran out).
    int search(int budget);
    bool found() const { return found_; }          // +0x38: the start cell was expanded
    bool idle() const { return open_.empty(); }    // +0x10 == 0
    int expansions() const { return expanded_; }   // +0x34
    // Per-frame cell computations (cache +8; the manager sets 100 per frame, 0x61f970 -> vt 0x74).
    // Negative: no limit (the port runs searches to the end at once).
    void setCellBudget(int n) { cellBudget_ = n; }
    // 0x61f470: descent of the field from the start cell: the start point, cell centres (y = cell height), the
    // goal point; at most maxPoints. Empty when the start cell is unreached.
    std::vector<std::array<float, 3>> extract(int maxPoints) const;
    // 0x61f230 (PATH_FINDER vf06): p_i = (2 p_i + p_(i-1) + p_(i+1)) / 4 on the original points, each move
    // clamped to 15 m; only for more than 3 points.
    static void smooth(std::vector<std::array<float, 3>>& pts);

    // 0x61e710 (vt [2]): cost of stepping from cell b to cell a (a = the cell nearer the goal; dh = (h(a) -
    // h(b)) x invLen is the rise per metre in the travel direction b -> a). <= 0: blocked (-1.0, 0xa05224).
    float stepCost(int ax, int az, int bx, int bz, float invLen, float len) const;
    const AtvCell& cell(int x, int z) const;       // cached (0x61f090 -> 0x61eb30)
    float cellHeight(int x, int z) const;          // cached (vt [1] 0x61e6b0)
    float field(int x, int z) const { return field_[index(x, z)]; }
    bool inBox(int x, int z) const { return x <= box_[2] && box_[0] <= x && z <= box_[3] && box_[1] <= z; }  // 0x4dcf90
    std::array<int, 2> startCell() const { return {sx_, sz_}; }
    std::array<int, 2> goalCell() const { return {gx_, gz_}; }
    static int toCell(float v);
    void setBox(int ax, int az, int bx, int bz);  // 0x4dcf40: the midpoint cell +- 63
    void setBoxBounds(int x0, int z0, int x1, int z1) { box_[0] = x0, box_[1] = z0, box_[2] = x1, box_[3] = z1; }  // test vectors
    // Test vectors: put a cell's height and record into the caches (as if computed).
    void prime(int x, int z, float h, const AtvCell& c);

    // The 8 moves of 0x61ffe0: 0 (0,-1), 1 (-1,-1), 2 (-1,0), 3 (-1,+1), 4 (0,+1), 5 (+1,+1), 6 (+1,0), 7 (+1,-1).
    static constexpr int kDx[8] = {0, -1, -1, -1, 0, 1, 1, 1}, kDz[8] = {-1, -1, 0, 1, 1, 1, 0, -1};

private:
    static int index(int x, int z) { return (z & kMask) * kWidth + (x & kMask); }
    float heuristic(int x, int z) const;          // 0x4dcb70
    void push(float f, int x, int z, float g);    // 0x61fc40 + 0x61fa30
    void computeCell(int x, int z, AtvCell& c) const;  // 0x61eb30

    AtvWorld world_;
    float w40_ = 1, w48_ = 1, w3c_ = 0, w44_ = 0;
    float start_[3] = {0, 0, 0}, goal_[3] = {0, 0, 0};
    int sx_ = 0, sz_ = 0;
    int gx_ = INT32_MIN, gz_ = INT32_MIN;          // +0x70/+0x74 (0x80000000 = none)
    int box_[4] = {0, 0, 0, 0};                    // +0x88 min x, +0x8c min z, +0x90 max x, +0x94 max z
    std::vector<float> field_;                     // +4
    // Open list (0x61fa30 / 0x61fbe0): a size-balanced tree keyed by (f, record id), smallest first; records
    // (x, z, g) come from a pool whose freed ids are reused last-in first-out (0x61fdb0 / 0x61fc90).
    struct Rec { int x = 0, z = 0; float g = 0; };
    std::vector<Rec> recs_;                        // id 1.. -> recs_[id - 1]
    std::vector<uint16_t> freeIds_;
    std::vector<std::pair<float, int>> open_;      // binary heap on (f, id)
    bool found_ = false;
    int expanded_ = 0;
    mutable int cellBudget_ = -1;
    struct AltSlot { int x = INT32_MIN, z = INT32_MIN; float h = 0; };
    struct CellSlot { int x = INT32_MIN, z = INT32_MIN; AtvCell c; };
    mutable std::vector<AltSlot> alt_;
    mutable std::vector<CellSlot> cells_;
};

// PATH_ATV_MANAGER update 0x61f970: 600 expansions per frame shared by the open requests (600 / n each, the
// leftovers carried to the next one and into a second pass).
void atvManagerUpdate(const std::vector<AtvPathFinder*>& open);

// --aicheck-vectors: A1-A10 (0x61e710), S1-S3 (0x61f230) against emulated FUEL.exe, P1 (search invariants).
void atvCheckVectors(int& ok, int& bad);

}  // namespace fuel
