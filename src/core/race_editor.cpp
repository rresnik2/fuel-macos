// PK21: FUEL's race editor (see race_editor.hpp; notes/pk21.md).
#include "core/race_editor.hpp"

#include "core/road_graph.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <strings.h>

namespace fuel {

namespace {
template <class T> T get(const uint8_t* p, size_t off) {
    T v;
    std::memcpy(&v, p + off, sizeof v);
    return v;
}
template <class T> void put(uint8_t* p, size_t off, T v) { std::memcpy(p + off, &v, sizeof v); }
float dist2xz(float ax, float az, float bx, float bz) { return (ax - bx) * (ax - bx) + (az - bz) * (az - bz); }
}  // namespace

EditedRace EditedRace::read(const uint8_t* p) {
    EditedRace r;
    std::memcpy(r.ai, p, 7);
    r.timer = get<uint16_t>(p, 8);
    r.loop = p[0xa];
    std::memcpy(r.opt, p + 0xb, 4);
    r.type = int8_t(p[0xf]);
    r.count = p[0x10];
    for (size_t i = 0; i < kMaxPoints; ++i) {
        const uint8_t* q = p + 0x14 + i * 0x24;
        Point& pt = r.pts[i];
        pt.x = get<float>(q, 0), pt.y = get<float>(q, 4), pt.z = get<float>(q, 8);
        for (int k = 0; k < 3; ++k) pt.rot[k] = get<float>(q, 0xc + 4 * size_t(k));
        pt.weight = get<float>(q, 0x18), pt.u1c = get<uint32_t>(q, 0x1c), pt.gate = q[0x20], pt.valid = q[0x21];
    }
    return r;
}

void EditedRace::write(uint8_t* p) const {
    std::memcpy(p, ai, 7);
    put<uint16_t>(p, 8, timer);
    p[0xa] = loop;
    std::memcpy(p + 0xb, opt, 4);
    p[0xf] = uint8_t(type);
    p[0x10] = count;
    for (size_t i = 0; i < kMaxPoints; ++i) {
        uint8_t* q = p + 0x14 + i * 0x24;
        const Point& pt = pts[i];
        put(q, 0, pt.x), put(q, 4, pt.y), put(q, 8, pt.z);
        for (int k = 0; k < 3; ++k) put(q, 0xc + 4 * size_t(k), pt.rot[k]);
        put(q, 0x18, pt.weight), put(q, 0x1c, pt.u1c), q[0x20] = pt.gate, q[0x21] = pt.valid;
    }
}

bool EditedRace::closedLoop() const {
    if (count <= 1) return false;
    const Point& a = pts[0];
    const Point& b = pts[size_t(count - 1)];
    return std::sqrt(dist2xz(a.x, a.z, b.x, b.z)) < 1.0f;
}

int EditedRace::aiTotal() const {
    int n = 0;
    for (uint8_t a : ai) n += a;
    return n;
}

bool EditedRace::noGate() const {
    for (size_t i = 0; i < count; ++i)
        if (pts[i].gate) return false;
    return true;
}

bool EditedRace::gatesButLast() const {
    for (size_t i = 0; i + 1 < count; ++i)
        if (!pts[i].gate) return false;
    return true;
}

bool EditedRace::typeAvailable(int t) const {
    const bool loopy = closedLoop();
    const int ai = aiTotal();
    switch (t) {
        case 0: return count == 2 && !loopy && noGate() && ai > 0;                // 0x463950
        case 1: case 2: return count > 2 && !loopy && ai > 0;                    // 0x4639c0 / 0x463a30
        case 3: return count >= 2 && !loopy && noGate() && ai == 0;              // 0x463aa0
        case 4: return count > 3 && loopy && noGate() && ai > 0;                 // 0x463b10
        case 5: return count > 5 && pts[0].gate && ai > 0;                       // 0x463ba0
        case 6: return count >= 2 && gatesButLast() && ai > 0;                   // 0x463c00
        default: return false;
    }
}

uint16_t EditedRace::optionFlags() const {
    // 0x45ff30 + the jump table 0x45fed0 of 0x45fea0 (AL = 1 for option 0, 0 for option 1) [O asm].
    auto T = [](uint8_t v, bool alt) -> uint32_t {
        if (v == 0) return 0x800;
        if (v <= 2) return alt ? 0xa00 : 0xc00;
        if (v <= 5) return alt ? 0xa80 : 0xd00;
        return 0;
    };
    const uint32_t o3 = opt[3] & 3u;
    uint32_t f = 2u | (opt[2] == 1 ? 1u : 0u) | ((o3 * 4 | o3) * 4);
    if (opt[0] < 10) f |= T(opt[0], true);
    if (opt[1] < 10) f |= T(opt[1], false);
    return uint16_t(f);
}

namespace editor {

int freeSlots(const std::vector<uint8_t>& b) {
    int n = 0;
    for (size_t i = 0; i < EditedRace::kSlots; ++i) n += (i + 1) * EditedRace::kBytes <= b.size() && b[i * EditedRace::kBytes + 0x10] < 2;
    return n;
}

int firstFreeSlot(const std::vector<uint8_t>& b) {
    for (size_t i = 0; i < EditedRace::kSlots && (i + 1) * EditedRace::kBytes <= b.size(); ++i)
        if (b[i * EditedRace::kBytes + 0x10] < 2) return int(i);
    return -1;
}

int usedSlot(const std::vector<uint8_t>& b, int k) {
    for (size_t i = 0; i < EditedRace::kSlots && (i + 1) * EditedRace::kBytes <= b.size(); ++i)
        if (b[i * EditedRace::kBytes + 0x10] >= 2 && k-- == 0) return int(i);
    return -1;
}

EditedRace slot(const std::vector<uint8_t>& b, int i) {
    if (i < 0 || size_t(i + 1) * EditedRace::kBytes > b.size()) return {};
    return EditedRace::read(b.data() + size_t(i) * EditedRace::kBytes);
}

void setSlot(std::vector<uint8_t>& b, int i, const EditedRace& r) {
    if (i < 0 || size_t(i + 1) * EditedRace::kBytes > b.size()) return;
    r.write(b.data() + size_t(i) * EditedRace::kBytes);
}

std::string slotName(int i) {
    char buf[32];
    std::snprintf(buf, sizeof buf, "Edited Race %02d", i + 1);
    return buf;
}

int slotOfName(const std::string& name) {
    for (int i = 0; i < int(EditedRace::kSlots); ++i)
        if (strcasecmp(name.c_str(), slotName(i).c_str()) == 0) return i;
    return -1;
}

void initBlock(std::vector<uint8_t>& b) {
    b.assign(EditedRace::kSlots * EditedRace::kBytes, 0);
    for (size_t i = 0; i < EditedRace::kSlots; ++i) b[i * EditedRace::kBytes + 0xf] = 0xff;
}

namespace {
// The road-route length between two points (0x6d8e20 + RoadRoute_Cost with the router's cost 0 = length [I]): the
// route's polyline (snapped start, records, snapped end).
bool roadLength(const RoadGraph& g, float ax, float az, float bx, float bz, float& out) {
    RoadGraph::Route route;
    if (!g.request(route, ax, az, bx, bz, [&g](uint32_t r, uint32_t off, uint32_t n, int) { return g.lengthCost(r, off, n); }) || !route.valid)
        return false;
    const auto pl = g.polyline(route);
    double s = 0;
    for (size_t k = 1; k < pl.size(); ++k) s += std::hypot(double(pl[k][0] - pl[k - 1][0]), double(pl[k][1] - pl[k - 1][1]));
    out = float(s);
    return true;
}
int roadAt(const EditorWorld& w, float x, float z) {
    if (!w.graph) return -1;
    return w.graph->roadAt(x, z, w.flatHalf ? w.flatHalf : [](int) { return 0.0f; });
}
}  // namespace

float routeLength(const EditedRace& r, const EditorWorld& w) {
    // 0x4608a0: 0 without the router; per consecutive pair: the road route when both points are on a road (0x6d1d70) and
    // the route request succeeds, else the straight x z distance.
    if (!w.graph || r.count < 2) return 0;
    double total = 0;
    int prevRoad = roadAt(w, r.pts[0].x, r.pts[0].z);
    for (size_t i = 1; i < r.count; ++i) {
        const auto& a = r.pts[i - 1];
        const auto& b = r.pts[i];
        const int road = roadAt(w, b.x, b.z);
        float d = 0;
        if (prevRoad < 0 || road < 0 || !roadLength(*w.graph, a.x, a.z, b.x, b.z, d)) d = std::sqrt(dist2xz(a.x, a.z, b.x, b.z));
        total += d;
        prevRoad = road;
    }
    return float(total);
}

int lengthKm(float m) { return int(double(float((m + 500.0f) * 0.001f))); }
bool tooLong(float m) { return lengthKm(m) > 150; }

float distToJunction(const RoadGraph& g, float x, float z, float rad) {
    // 0x6d1de0: the nearest record within rad (else -1); at each end the ring of records meeting there is walked (at most
    // 4); an end with 3 or more is a junction: distance = |q - point| + the record's length from there to that end.
    int rec = -1, k = -1;
    float qx = 0, qz = 0;
    if (g.nearestRoad(x, z, rad, rec, k, qx, qz) < 0) return -1.0f;
    auto ring = [&](bool end1) {
        uint32_t r = uint32_t(rec);
        bool e = end1;
        int c = 0;
        do {
            if (++c > 3) break;
            const RoadGraph::Rec& R = g.rec(r);
            const uint32_t next = e ? R.link1 : R.link0;
            e = e ? R.side1 != 0 : R.side0 != 0;
            r = next;
        } while (r != uint32_t(rec));
        return c;
    };
    const RoadGraph::Rec& R = g.rec(size_t(rec));
    const int n = int(R.n);
    float d1 = 1e6f, d0 = 1e6f;
    if (ring(true) > 2) {
        const auto p = g.point(uint32_t(rec), uint32_t(k + 1));
        d1 = std::hypot(p[0] - qx, p[1] - qz);
        if (k + 1 < n - 1) d1 += g.lengthCost(uint32_t(rec), uint32_t(k + 1), uint32_t(n - (k + 1)));
    }
    if (ring(false) > 2) {
        const auto p = g.point(uint32_t(rec), uint32_t(k));
        d0 = std::hypot(p[0] - qx, p[1] - qz);
        if (k > 0) d0 += g.lengthCost(uint32_t(rec), 0, uint32_t(k + 1));
    }
    return std::min(d0, d1);
}

bool newRaceDefaults(EditedRace& r) {
    if (r.count < 2) return false;
    std::fill(std::begin(r.ai), std::end(r.ai), uint8_t(0));
    r.opt[0] = r.opt[1] = r.opt[2] = 0, r.opt[3] = 1;
    r.timer = 0;
    if (r.closedLoop()) {
        r.type = 4, r.loop = 1;
    } else {
        r.loop = 0;
        r.type = r.count == 2 ? 0 : 1;
    }
    return true;
}

namespace {
// 0x5d65f0: the vehicle of an AI class (table index of the mesh; 9 = random MUSCLE_ROAD_6 .. BIKE_OFF_5 [port: the first]).
const char* classVehicle(int c) {
    static const char* const k[6] = {"BUGGY_OFF_2", "BIKE_OFF_5", "MUSCLE_ROAD_6", "QUAD_OFF_1", "SUV_ALL_1", "TRUCK_ALL_1"};
    return c >= 0 && c < 6 ? k[c] : c == 9 ? "MUSCLE_ROAD_6" : nullptr;
}
// 0x468290: the heading at point i towards point i+1 (i-1 reversed, then turned round) along the road route 1 m from its
// start (RoadRoute_WalkFromStart), the straight direction when no route; 0x468580 for gate points: the sum of the
// normalised in / out directions. [port] rotation vector (0, yaw, 0): the ground arc (Quat_FromArc) is left out.
float gateYaw(const EditedRace& r, size_t i, bool reversed, const EditorWorld& w) {
    const auto& p = r.pts[i];
    float dx = 0, dz = 1;
    if (p.gate) {
        float ix = 0, iz = 0, ox = 0, oz = 0;
        if (i > 0) ix = p.x - r.pts[i - 1].x, iz = p.z - r.pts[i - 1].z;
        if (i + 1 < r.count) ox = r.pts[i + 1].x - p.x, oz = r.pts[i + 1].z - p.z;
        const float li = std::hypot(ix, iz), lo = std::hypot(ox, oz);
        if (li > 0) ix /= li, iz /= li;
        if (lo > 0) ox /= lo, oz /= lo;
        dx = ix + ox, dz = iz + oz;
    } else {
        const size_t j = reversed ? i - 1 : i + 1;
        if (j >= r.count) return 0;
        float tx = r.pts[j].x, tz = r.pts[j].z;
        if (w.graph) {
            RoadGraph::Route route;
            const RoadGraph& g = *w.graph;
            if (g.request(route, p.x, p.z, tx, tz, [&g](uint32_t rr, uint32_t off, uint32_t n, int) { return g.lengthCost(rr, off, n); }) && route.valid) {
                const auto pl = g.polyline(route);
                float left = 1.0f;
                for (size_t k = 1; k < pl.size(); ++k) {
                    const float seg = std::hypot(pl[k][0] - pl[k - 1][0], pl[k][1] - pl[k - 1][1]);
                    if (seg >= left && seg > 0) {
                        tx = pl[k - 1][0] + (pl[k][0] - pl[k - 1][0]) * left / seg, tz = pl[k - 1][1] + (pl[k][1] - pl[k - 1][1]) * left / seg;
                        break;
                    }
                    left -= seg;
                }
            }
        }
        dx = tx - p.x, dz = tz - p.z;
        if (reversed) dx = -dx, dz = -dz;
    }
    if (std::fabs(dx) + std::fabs(dz) < 1e-9f) return 0;
    return std::atan2(dx, dz);
}
}  // namespace

Race buildRace(const EditedRace& r, int i, const EditorWorld& w) {
    Race race;
    race.name = slotName(i);
    if (r.count < 2) return race;
    const size_t n = r.count;
    auto pos = [&](size_t k) { return std::array<float, 3>{r.pts[k].x, r.pts[k].y, r.pts[k].z}; };
    race.start = pos(0);
    race.startYaw = gateYaw(r, 0, false, w);
    race.startRot = {0, race.startYaw, 0};
    for (size_t k = 1; k + 1 < n; ++k) {
        race.checkpoints.push_back(pos(k));
        const float y = gateYaw(r, k, false, w);
        race.checkpointRot.push_back({0, y, 0});
        race.checkpointFlags.push_back(r.pts[k].valid ? (r.pts[k].gate ? 2 : 1) : 0);
    }
    race.finish = pos(n - 1);
    race.finishYaw = gateYaw(r, n - 1, true, w);
    race.finishRot = {0, race.finishYaw, 0};
    race.aiCount = r.aiTotal();                                   // +0x50
    race.timeLimit = race.aiCount == 0 ? float(r.timer) : 0.0f;   // +0x48
    race.laps = r.loop;                                           // +0x5c (+0x209 = laps != 0)
    race.scenario = r.type == 0 ? 1 : r.type == 4 ? 6 : 2;        // +0x58
    race.mode = race.scenario == 1 ? Race::AtoB : race.scenario == 6 ? Race::Circuit : Race::Checkpoint;
    race.editorFlags = r.optionFlags();                           // +0x54
    for (int c = 0; c < 7; ++c)                                   // the AI list (+0xa8), class by class
        for (int k = 0; k < r.ai[c]; ++k)
            if (const char* v = classVehicle(c)) race.aiFilterTokens.push_back(v);
    return race;
}

std::vector<Event> editedEvents(const std::vector<uint8_t>& block, const EditorWorld& w) {
    std::vector<Event> out;
    for (int i = 0; i < int(EditedRace::kSlots); ++i) {
        const EditedRace r = slot(block, i);
        if (r.count < 2) continue;
        Event e;
        e.id = slotName(i);
        e.kind = Event::Edited;
        e.race = buildRace(r, i, w);
        e.playable = true;
        e.note = "edited race (career slot " + std::to_string(i + 1) + ")";
        out.push_back(std::move(e));
    }
    return out;
}

// ---- the EditorHUD's map state ----

void Session::open(EditedRace* r, float px, float pz, const EditorWorld& w) {
    race = r, world = w;
    picked = -1;
    if (!r) return;
    if (r->count < 2) cx = px, cz = pz;
    else cx = r->pts[0].x, cz = r->pts[0].z;
    cy = world.ground ? world.ground(cx, cz) : 0.0f;
}

std::array<float, 3> Session::snapped(uint8_t* gate) const {
    const float rad = pickRadius();
    if (world.graph) {
        int rec, k;
        float qx, qz;
        const float d2 = world.graph->nearestRoad(cx, cz, rad, rec, k, qx, qz);
        if (d2 >= 0 && d2 < rad * rad) {
            *gate = 0;
            return {qx, world.ground ? world.ground(qx, qz) : 0.0f, qz};
        }
    }
    *gate = 1;
    return {cx, cy, cz};
}

int Session::nearest(int exclude, float limitSq) const {
    if (!race) return -1;
    float best = 1.96e10f;
    int out = -1;
    for (int k = int(race->count) - 1; k >= 0; --k) {
        if (k == exclude) continue;
        const float d = dist2xz(race->pts[size_t(k)].x, race->pts[size_t(k)].z, cx, cz);
        if (d < best) out = k, best = d;
    }
    return limitSq < best ? -1 : out;
}

bool Session::dropAllowed(uint8_t gate, const std::array<float, 3>& p) const {
    if (gate != 0 || !world.graph) return false;
    if (world.hubOpen && !world.hubOpen(p[0], p[2])) return false;
    return distToJunction(*world.graph, p[0], p[2], pickRadius()) >= 30.0f;
}

bool Session::cursorPlaceable() const {
    if (!race) return false;
    uint8_t gate = 0;
    const auto p = snapped(&gate);
    return dropAllowed(gate, p);
}

void Session::append(const std::array<float, 3>& p, uint8_t gate) {
    if (!race || race->count >= EditedRace::kMaxPoints) return;
    const int i = nearest(race->count, 900.0f);  // 0x44610000: no second point within 30 m of the start
    if (i == 0 && race->count == 1) return;
    EditedRace::Point& q = race->pts[race->count];
    q.x = p[0], q.y = p[1], q.z = p[2];
    q.gate = gate, q.valid = 1;
    race->count = uint8_t(std::min<int>(race->count + 1, EditedRace::kMaxPoints));
}

void Session::remove(int k) {
    if (!race || k < 0 || k >= int(race->count)) return;
    for (int i = k; i + 1 < int(race->count); ++i) race->pts[size_t(i)] = race->pts[size_t(i + 1)], race->pts[size_t(i)].valid = 1;
    race->count = uint8_t(std::min<int>(race->count - 1, EditedRace::kMaxPoints));
}

bool Session::overLimit(bool drop) {
    if (!race || !tooLong(routeLength(*race, world))) return false;
    if (drop) race->count = uint8_t(std::min<int>(race->count - 1, EditedRace::kMaxPoints));
    return true;
}

void Session::move() {
    if (!race || picked < 0 || picked >= int(race->count)) return;
    uint8_t gate = 1;
    auto p = snapped(&gate);
    if (!dropAllowed(gate, p)) return;
    const int j = picked, n = race->count;
    const int i = nearest(j, 900.0f);
    if (i >= 1 && i != n - 1) return;
    if (i == 0) {  // the last point dropped on the start closes the loop
        if (j != n - 1 || n == 2) return;
        p = {race->pts[0].x, race->pts[0].y, race->pts[0].z}, gate = race->pts[0].gate;
    } else if (i == n - 1) {  // the start dropped on the last point
        if (j != 0 || n == 2) return;
        p = {race->pts[size_t(n - 1)].x, race->pts[size_t(n - 1)].y, race->pts[size_t(n - 1)].z}, gate = race->pts[size_t(n - 1)].gate;
    }
    EditedRace::Point& q = race->pts[size_t(j)];
    q.x = p[0], q.y = p[1], q.z = p[2], q.gate = gate;
    picked = -1;
}

int Session::key(Key k) {
    if (!race) return 0;
    if (k == Add) {
        if (race->closedLoop() || picked >= 0) return 0;
        uint8_t gate = 1;
        auto p = snapped(&gate);
        const int i = nearest(-1, 900.0f);
        if (i > 0) return 1;
        if (i == 0) p = {race->pts[0].x, race->pts[0].y, race->pts[0].z}, gate = race->pts[0].gate;  // closing on the start
        if (dropAllowed(gate, p)) append(p, gate);
        if (overLimit(true)) return 2;
        return 1;
    }
    if (k == Remove) {
        if (picked < 0) {
            const float rad = pickRadius();
            const int i = nearest(-1, rad * rad);
            if (i < 0) remove(int(race->count) - 1);  // 0x464680
            else remove(i);
        } else {
            remove(picked);
            picked = -1;
        }
        return 1;
    }
    // Pick: take the point under the cursor, or drop the picked one (0x465660).
    if (picked < 0) {
        const float rad = pickRadius();
        picked = nearest(-1, rad * rad);
        return 1;
    }
    const int was = picked;
    move();
    if (!overLimit(false)) return 1;
    picked = was;
    return 2;
}

int Session::update(float dt, float sx, float sz, int zoomDir, float pp47, float pp48) {
    // 0x464be0 (retail): zoom target +-= trigger x PP48 x zoom x dt in [1500, 30000]; cursor += rotated stick x PP47 x
    // zoom x dt (screen right = (hx, hz), up = (hz, -hx)), clamped to +-65530 (0x4a0e60), height from the ground.
    const float f = zoom * dt;
    if (zoomDir != 0) {
        zoomTarget -= float(zoomDir) * pp48 * f;
        zoomTarget = std::clamp(zoomTarget, 1500.0f, 30000.0f);
    }
    zoom = (zoom + zoomTarget) * 0.5f;  // 0x4ac3d0
    const float ox = cx, oy = cy, oz = cz;
    const float l = std::hypot(sx, sz);
    if (l > 1) sx /= l, sz /= l;
    if (l > 0) {
        const float step = f * pp47;
        cx = std::clamp(cx + step * (sx * hx + sz * hz), -65530.0f, 65530.0f);
        cz = std::clamp(cz + step * (sx * hz - sz * hx), -65530.0f, 65530.0f);
        cy = world.ground ? world.ground(cx, cz) : cy;
    }
    if (race && picked >= 0 && picked < int(race->count)) {  // the picked point follows the cursor
        EditedRace::Point& q = race->pts[size_t(picked)];
        if (std::fabs(cx - q.x) < 1e-6f && std::fabs(cy - q.y) < 1e-6f && std::fabs(cz - q.z) < 1e-6f) return 0;
        q.x = cx, q.y = cy, q.z = cz;
        return 4;
    }
    return std::fabs(cx - ox) < 1e-6f && std::fabs(cy - oy) < 1e-6f && std::fabs(cz - oz) < 1e-6f ? 0 : 3;
}

}  // namespace editor
}  // namespace fuel
