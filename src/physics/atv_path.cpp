// FUEL's off-road path finder (see atv_path.hpp). Decoded from FUEL.exe: request 0x623580, start 0x623820,
// weights 0x6238a0, search 0x61f6a0, step cost 0x61e710, cell 0x61eb30, height 0x61eaf0 / 0x61e980, path
// 0x61f470, smoothing 0x61f230, open list 0x61fa30 / 0x61fbe0 / 0x61fc40 / 0x61fc90 / 0x61fdb0, box 0x4dcf40 /
// 0x4dcf90, heuristic 0x4dcb70, manager 0x61f970. Step-cost vectors: --aicheck-vectors A1-A9 (emulated).
#include "physics/atv_path.hpp"

#include <algorithm>
#include <cstring>
#include <cmath>

namespace fuel {

static float bitsToFloat(uint32_t u) {
    float f;
    std::memcpy(&f, &u, 4);
    return f;
}
const float AtvPathFinder::kDiag = bitsToFloat(0x4162462au);     // 14.142136
const float AtvPathFinder::kInvDiag = bitsToFloat(0x3d90d0c7u);  // 1 / 14.142136

AtvPathFinder::AtvPathFinder(AtvWorld world)
    : world_(std::move(world)), field_(size_t(kCells), kUnreached), alt_(size_t(kCells)), cells_(size_t(kCells)) {}

// FUEL stores ROUND(v x 0.1 + 0.5) with the x87 rounding down (the game sets RC_DOWN): floor.
int AtvPathFinder::toCell(float v) { return int(std::floor(double(v * kInvCell + 0.5f))); }

void AtvPathFinder::setWeights(float offRoad, float road) {
    if (offRoad != w3c_ || road != w44_) gx_ = gz_ = INT32_MIN;
    w3c_ = offRoad, w44_ = road;
    w40_ = 1.0f / offRoad, w48_ = 1.0f / road;
}

void AtvPathFinder::setBox(int ax, int az, int bx, int bz) {
    const int cx = (ax + bx) >> 1, cz = (az + bz) >> 1;
    box_[0] = cx - kHalfBox, box_[1] = cz - kHalfBox, box_[2] = cx + kHalfBox, box_[3] = cz + kHalfBox;
}

void AtvPathFinder::setStart(float x, float y, float z) {
    start_[0] = x, start_[1] = y, start_[2] = z;
    sx_ = toCell(x), sz_ = toCell(z);
    setBox(sx_, sz_, sx_, sz_);
}

float AtvPathFinder::heuristic(int x, int z) const {
    const float dx = start_[0] - float(x) * kCell, dz = start_[2] - float(z) * kCell;
    return std::sqrt(dz * dz + dx * dx);
}

void AtvPathFinder::push(float f, int x, int z, float g) {
    int id;
    if (!freeIds_.empty()) {
        id = freeIds_.back();
        freeIds_.pop_back();
    } else {
        recs_.push_back({});
        id = int(recs_.size());
    }
    recs_[size_t(id - 1)] = {x, z, g};
    open_.push_back({f, id});
    std::push_heap(open_.begin(), open_.end(), std::greater<>());
}

bool AtvPathFinder::request(float x, float y, float z) {
    const int gx = toCell(x), gz = toCell(z);
    if (gx == gx_ && gz == gz_) return true;
    setBox(sx_, sz_, gx, gz);
    bool any = false;
    for (int d = 0; d < 8; ++d) {
        const bool diag = d & 1;
        if (stepCost(gx, gz, gx + kDx[d], gz + kDz[d], diag ? kInvDiag : kInvCell, diag ? kDiag : kCell) > 0) any = true;
    }
    if (!any) return false;
    gx_ = gx, gz_ = gz;
    goal_[0] = x, goal_[1] = y, goal_[2] = z;
    std::fill(field_.begin(), field_.end(), kUnreached);
    field_[size_t(index(gx, gz))] = 0;
    // 0x623730: empty open list and pool, push the goal (f 0, g 0).
    recs_.clear(), freeIds_.clear(), open_.clear();
    push(0, gx, gz, 0);
    expanded_ = 0;
    found_ = false;
    return true;
}

int AtvPathFinder::search(int budget) {
    const float w = 2.0f;  // vt [7] 0x4dd000
    while (budget != 0) {
        if (cellBudget_ >= 0 && cellBudget_ < 1) return 0;
        if (open_.empty()) return budget;
        std::pop_heap(open_.begin(), open_.end(), std::greater<>());
        const int id = open_.back().second;
        open_.pop_back();
        const Rec r = recs_[size_t(id - 1)];
        freeIds_.push_back(uint16_t(id));
        if (r.x == sx_ && r.z == sz_ && !found_) found_ = true;
        ++expanded_;
        for (int d = 0; d < 8; ++d) {
            const int nx = r.x + kDx[d], nz = r.z + kDz[d];
            const bool diag = d & 1;
            float& fn = field_[size_t(index(nx, nz))];
            if (!(r.g < fn)) continue;
            const double c = stepCost(r.x, r.z, nx, nz, diag ? kInvDiag : kInvCell, diag ? kDiag : kCell);
            if (!(c > 0)) continue;
            const double g = c + double(r.g);
            if (!(g < double(fn))) continue;
            fn = float(g);
            if (inBox(nx, nz)) push(float(double(heuristic(nx, nz)) * w + double(float(g))), nx, nz, float(g));
        }
        --budget;
    }
    return 0;
}

void AtvPathFinder::prime(int x, int z, float h, const AtvCell& c) {
    alt_[size_t(index(x, z))] = {x, z, h};
    cells_[size_t(index(x, z))] = {x, z, c};
}

float AtvPathFinder::cellHeight(int x, int z) const {
    AltSlot& s = alt_[size_t(index(x, z))];
    if (s.x != x || s.z != z) {
        s.x = x, s.z = z;
        s.h = world_.height ? world_.height(float(x) * kCell, float(z) * kCell) : 0.0f;
        if (cellBudget_ > 0) --cellBudget_;
    }
    return s.h;
}

const AtvCell& AtvPathFinder::cell(int x, int z) const {
    CellSlot& s = cells_[size_t(index(x, z))];
    if (s.x != x || s.z != z) {
        s.x = x, s.z = z;
        computeCell(x, z, s.c);
        if (cellBudget_ > 0) --cellBudget_;
    }
    return s.c;
}

// 0x61eb30. On a road (centreline within 14.142 m, 0x61ef30 -> 0x6fb2a0): road flag, c = d / 14.142 x 0.1 +
// the type's blend (+0xb4), slope 1, forest 0. Off roads: the normal from the four diagonal neighbours' heights
// (0x61eeb0 moves 5, 1, 3, 7 = (+1,+1), (-1,-1), (-1,+1), (+1,-1); n = (a, 4 x cell, b) normalised), slope =
// 1 / max(0.01, (n.y - 0.85) x 6.666668), forest density (0x45dd90), c = 1 when the water depth (0x40fd10) is
// <= 0.2, else -1.
void AtvPathFinder::computeCell(int x, int z, AtvCell& c) const {
    const float px = float(x) * kCell, pz = float(z) * kCell;
    float d = kDiag, blend = 0;
    const bool near = world_.road && world_.road(px, pz, kDiag, d, blend) && d < kDiag;
    if (near) {
        c.road = true;
        c.slope = 1.0f;
        c.forest = 0.0f;
        c.c = (d / kDiag) * 0.1f + blend;
        return;
    }
    const float h5 = cellHeight(x + 1, z + 1), h1 = cellHeight(x - 1, z - 1), h3 = cellHeight(x - 1, z + 1), h7 = cellHeight(x + 1, z - 1);
    const float k = kCell * 4.0f;
    const float b0 = h3 - h7, a = b0 + (h1 - h5), b = (h1 - h5) - b0;
    double f = ((1.0 / std::sqrt(double(k) * k + double(a) * a + double(b) * b)) * k - 0.85) * 6.666668;
    if (f <= 0.01) f = 0.01;
    c.road = false;
    c.slope = float(1.0 / f);
    c.forest = world_.forest ? world_.forest(px, pz) : 0.0f;
    c.c = (world_.water ? world_.water(px, pz) : 0.0f) <= 0.2f ? 1.0f : -1.0f;
}

// 0x61e710. dh = (h(a) - h(b)) x invLen: the rise per metre walking b -> a. Blocked (-1): b outside the box;
// dh > 0.5 or dh < -4 unless both cells are road cells; min(c) < -0.5 (water); max(slope) >= 99. Else
// k = dh <= 0 ? dh^2 + 1 : dh^2 (10 c + 1) + 1, times the speed term (1 - c) / road + c / offRoad, times
// max(slope) uphill, times (forest_a + forest_b) / 2 + 1, times the step length.
float AtvPathFinder::stepCost(int ax, int az, int bx, int bz, float invLen, float len) const {
    if (!inBox(bx, bz)) return -1.0f;
    const float ha = cellHeight(ax, az), hb = cellHeight(bx, bz);
    const float dh = (ha - hb) * invLen;
    const AtvCell& ca = cell(ax, az);
    const AtvCell& cb = cell(bx, bz);
    const bool roads = ca.road && cb.road;
    if (!((dh <= 0.5f || roads) && (-4.0f <= dh || roads))) return -1.0f;
    const double c = std::min(cb.c, ca.c);
    if (!(-0.5 <= c)) return -1.0f;
    const double h = dh;
    double k = h <= 0 ? h * h + 1 : h * h * (c * 10.0 + 1) + 1;
    k *= (1 - c) * double(w48_) + double(w40_) * c;
    const double s = std::max(ca.slope, cb.slope);
    if (!(s < 99.0)) return -1.0f;
    if (h > 0) k *= s;
    return float(((double(cb.forest) + double(ca.forest)) * 0.5 + 1) * k * double(len));
}

std::vector<std::array<float, 3>> AtvPathFinder::extract(int maxPoints) const {
    std::vector<std::array<float, 3>> out;
    if (field_[size_t(index(sx_, sz_))] == kUnreached) return out;
    int cx = sx_, cz = sz_;
    out.push_back({start_[0], start_[1], start_[2]});
    float travelled = 0;
    int n = 1;
    do {
        if (cx == gx_ && cz == gz_) break;
        double best = kUnreached, bestStep = 0;
        int bx = 0, bz = 0;
        for (int d = 0; d < 8; ++d) {
            const int nx = cx + kDx[d], nz = cz + kDz[d];
            const bool diag = d & 1;
            const double c = stepCost(nx, nz, cx, cz, diag ? kInvDiag : kInvCell, diag ? kDiag : kCell);
            if (!(c > 0)) continue;
            const double v = double(field_[size_t(index(nx, nz))]) + c + double(travelled);
            if (v < best) best = v, bestStep = c, bx = nx, bz = nz;
        }
        if (best >= double(kUnreached)) break;
        travelled = float(bestStep + double(travelled));
        out.push_back({float(bx) * kCell, cellHeight(bx, bz), float(bz) * kCell});
        ++n;
        cx = bx, cz = bz;
    } while (n < maxPoints - 1);
    out.push_back({goal_[0], goal_[1], goal_[2]});
    return out;
}

void AtvPathFinder::smooth(std::vector<std::array<float, 3>>& p) {
    const int n = int(p.size());
    if (n <= 3) return;
    std::array<float, 3> prev = p[0];
    for (int i = 1; i < n - 1; ++i) {
        const std::array<float, 3> cur = p[size_t(i)];
        const std::array<float, 3>& next = p[size_t(i + 1)];
        float q[3];
        for (int k = 0; k < 3; ++k) q[k] = cur[size_t(k)] * 0.0f + (cur[size_t(k)] * 2.0f + prev[size_t(k)] + next[size_t(k)]) * 0.25f;
        const float dx = q[0] - cur[0], dy = q[1] - cur[1], dz = q[2] - cur[2];
        const float l2 = dx * dx + dz * dz + dy * dy;
        if (225.0f < l2) {
            const double s = 15.0 / std::sqrt(double(l2));
            q[0] = float(dx * s + cur[0]), q[1] = float(dy * s + cur[1]), q[2] = float(dz * s + cur[2]);
        }
        p[size_t(i)] = {q[0], q[1], q[2]};
        prev = cur;
    }
}

void atvManagerUpdate(const std::vector<AtvPathFinder*>& open) {
    if (open.empty()) return;
    for (AtvPathFinder* f : open) f->setCellBudget(100);
    const int share = 600 / int(open.size());
    int left = 0;
    for (AtvPathFinder* f : open) left = f->search(left + share);
    for (AtvPathFinder* f : open) left = f->search(left);
}

}  // namespace fuel
