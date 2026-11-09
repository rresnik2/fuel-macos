// Missions defined by the world itself: FUEL's 0x4f4db0, which the level's mission build 0x4f40f0
// runs after it has turned every EMD block into a mission (notes/p42-missions.md). The shipped
// USA1.DPC defines 15 of the 72 career races this way (CR1-YELL-CP and the 14 -CT circuits); no
// script has an EMD block for them, which is why the hub lists dropped them before.
//
// Decoded from FUEL.exe (Observed unless marked):
//  - The walk: from the World_Z root node, each node then its children (Node +0xa4 = body +4) and
//    the next sibling (Node +0xac = body +0xc, the port's Node::prev; walking it from +0xa4 reaches
//    every child in all 1305 USA1 parents, walking body +8 in 644).
//  - A node is a mission when it has a UserDefine (+0x9c) whose keyword search 0x53f710 finds
//    MISSION_COURSE. 0x53f710: each line's first token compared with _stricmp, up to three
//    arguments; when the node's own lines miss it, a Lod_Z object's UserDefine is searched.
//  - Mission record (0x210 B): +8 = 0x1d (type), +0 = 0 (no start / finish banners), +0x1fc = node,
//    editRaceName (31 chars), editRaceNbLoop / editRaceNbLap (+0x5c, laps), editRaceNbIA (+0x50),
//    editRaceTimer (+0x48), editRaceScenario (+0x58), editRaceMode (+4, kept when 0..6),
//    editRaceIAElastic (+0x100..), EditRacePoppingCheckpoint (flag). EditRaceWeatherScenario, Fog,
//    Clouds, Rain and Time set weather fields the port does not use. Unset fields keep the
//    constructor's values (0x4fd430: 0, so no AI until AddMissionPARAM sets +0x50).
//  - Points: the direct children with editRaceStart / editRaceCheck / editRaceEnd. Position = the
//    node's world translation, or, when the node has a collision volume (Inferred: the handle chain
//    node +0xc -> +0x58 -> +0x98 ends at its CollisionVol_Z), the centre of its first box
//    (0x7375d0: box matrix x node matrix, 0x743210) with width 0x4f8ea0 = |(X, Z)| of the box's
//    world-aligned half extents (0x4f96b0). Rotation = 0x644c10 (matrix -> quaternion) then
//    0x41d280 (quaternion -> rotation vector).
//  - editRaceCheck: when the node or a node below it (or after it, 0x4f5970's search order) is
//    CP_LEFT and another CP_RIGHT, the gate is their midpoint, half-width = half their distance and
//    heading atan2(dz, -dx) of right - left (0x4f5a10); else the node's own centre and width.
//    Flag 0 (+0x6c).
//  - After the children: a finish off the start on any axis by >= 1e-6 but within 1 m (x, z) of
//    it is put on the start (with the start's rotation). EMD_End's circuit rule is not applied.
#include "races.hpp"
#include "events.hpp"
#include "race_markers.hpp"
#include "road_graph.hpp"
#include "road_types.hpp"
#include "vegetation.hpp"
#include <filesystem>
#include <fstream>
#include <set>

#include "assembly.hpp"
#include "dpc.hpp"

#include <strings.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstring>
#include <functional>
#include <optional>
#include <sstream>
#include <unordered_map>
#include <unordered_set>

namespace fuel {

namespace {

constexpr uint32_t kNodeClass = 0x85D02528;
constexpr uint32_t kUserDefineClass = 0x52F79F96;
constexpr uint32_t kLodClass = 0x73DC6A13;
constexpr uint32_t kWorldClass = 0x39B67ECB;
constexpr uint32_t kCollisionVolClass = 0x8EF49632;

using M4 = std::array<float, 16>;

// One "keyword arg arg arg" line match of 0x53f710.
struct Keyword {
    bool found = false;
    std::string args[3];
};

class World {
public:
    explicit World(const DpcFile& d) : d_(d) {}

    const Node* node(uint32_t id) {
        auto it = nodes_.find(id);
        if (it != nodes_.end()) return it->second ? &*it->second : nullptr;
        std::optional<Node> n;
        if (const DpcFile::Entry* e = d_.find(id); e != nullptr && e->classCrc == kNodeClass) try {
                if (const auto b = d_.body(*e)) n = parseNodeBody(*b);
            } catch (const std::exception&) {
            }
        return (nodes_[id] = n) ? &*nodes_[id] : nullptr;
    }
    uint32_t child(uint32_t id) { const Node* n = node(id); return n ? n->child : 0; }    // Node +0xa4
    uint32_t sibling(uint32_t id) { const Node* n = node(id); return n ? n->prev : 0; }   // Node +0xac

    const std::vector<std::string>& lines(uint32_t userDefine) {
        static const std::vector<std::string> none;
        if (userDefine == 0) return none;
        auto it = ud_.find(userDefine);
        if (it != ud_.end()) return it->second;
        std::vector<std::string> l;
        if (const DpcFile::Entry* e = d_.find(userDefine); e != nullptr && e->classCrc == kUserDefineClass) try {
                if (const auto b = d_.body(*e)) l = parseUserDefineBody(*b);
            } catch (const std::exception&) {
            }
        return ud_[userDefine] = std::move(l);
    }
    bool hasUserDefine(uint32_t id) { const Node* n = node(id); return n && n->userDefine != 0; }

    // 0x53f710: the node's UserDefine, then (when it misses) a Lod_Z object's own UserDefine (+0x88).
    Keyword keyword(uint32_t id, const char* key) {
        Keyword k;
        const Node* n = node(id);
        if (n == nullptr) return k;
        if (search(lines(n->userDefine), key, k)) return k;
        if (const DpcFile::Entry* e = d_.find(n->object); e != nullptr && e->classCrc == kLodClass) try {
                if (const auto b = d_.body(*e)) search(lines(parseLodBody(*b).userDefine), key, k);
            } catch (const std::exception&) {
            }
        return k;
    }

    std::shared_ptr<const SplineGraph> splineGraph(uint32_t object) {
        const DpcFile::Entry* e = d_.find(object);
        if (e == nullptr || e->classCrc != kSplineGraphClass) return nullptr;
        try {
            if (const auto b = d_.body(*e)) return parseSplineGraph(*b);
        } catch (const std::exception&) {
        }
        return nullptr;
    }
    // The first box (0x80 B: matrix, inverse) of a CollisionVol_Z object (loader 0x7fedd0: u32 n, n boxes).
    std::optional<M4> box(uint32_t object) {
        const DpcFile::Entry* e = d_.find(object);
        if (e == nullptr || e->classCrc != kCollisionVolClass) return std::nullopt;
        const auto b = d_.body(*e);
        if (!b || b->size() < 4 + 64) return std::nullopt;
        uint32_t n;
        std::memcpy(&n, b->data(), 4);
        if (n == 0) return std::nullopt;
        M4 m;
        std::memcpy(m.data(), b->data() + 4, 64);
        return m;
    }

private:
    static bool search(const std::vector<std::string>& ls, const char* key, Keyword& k) {
        for (const std::string& line : ls) {
            std::istringstream in(line);
            std::string first;
            if (!(in >> first) || strcasecmp(first.c_str(), key) != 0) continue;
            k.found = true;
            for (std::string& a : k.args)
                if (!(in >> a)) break;
            return true;
        }
        return false;
    }
    const DpcFile& d_;
    std::unordered_map<uint32_t, std::optional<Node>> nodes_;
    std::unordered_map<uint32_t, std::vector<std::string>> ud_;
};

// 0x644c10 (matrix -> quaternion, x y z w, rows of a row-vector matrix) and 0x41d4b0 (normalise).
std::array<float, 4> quatOf(const M4& m) {
    std::array<float, 4> q{};
    float t = m[5] + m[0] + m[10];
    if (t > 0) {
        t = std::sqrt(t + 1.0f);
        q[3] = t * 0.5f;
        t = 0.5f / t;
        q[0] = (m[6] - m[9]) * t;
        q[1] = (m[8] - m[2]) * t;
        q[2] = (m[1] - m[4]) * t;
    } else {
        static const int next[3] = {1, 2, 0};  // 0xa2b044
        int i = m[0] < m[5] ? 1 : 0;
        if (m[size_t(i) * 5] < m[10]) i = 2;
        const int j = next[i], k = next[j];
        float s = std::sqrt(m[size_t(i) * 5] - m[size_t(j) * 5] - m[size_t(k) * 5] + 1.0f);
        q[size_t(i)] = s * 0.5f;
        if (s != 0) s = 0.5f / s;
        q[3] = (m[size_t(k + j * 4)] - m[size_t(j + k * 4)]) * s;
        q[size_t(j)] = (m[size_t(i + j * 4)] + m[size_t(j + i * 4)]) * s;
        q[size_t(k)] = (m[size_t(i + k * 4)] + m[size_t(k + i * 4)]) * s;
    }
    const float l = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (l < 1e-6f) return {0, 0, 0, 1};
    for (float& c : q) c /= l;
    return q;
}

// 0x41d280: quaternion -> rotation vector (axis x angle, angle in [0, pi]).
std::array<float, 3> rotationVector(const std::array<float, 4>& q) {
    const float w = std::clamp(q[3], -1.0f, 1.0f);  // [0xa05224] = -1
    const float s = std::sqrt(1.0f - w * w);
    if (s <= 1e-6f) return {0, 0, 0};  // [0xa66208..]
    std::array<float, 3> v = {q[0] / s, q[1] / s, q[2] / s};
    float a = 2.0f * std::acos(w);
    if (a < 0) v = {-v[0], -v[1], -v[2]}, a = -a;
    if (a > 3.1415927f) v = {-v[0], -v[1], -v[2]}, a = 6.2831855f - a;
    return {v[0] * a, v[1] * a, v[2] * a};
}

struct Point {
    std::array<float, 3> pos{};
    std::array<float, 4> quat{0, 0, 0, 1};
    float width = 0;  // 0x4f8ea0
};

// The point a start / checkpoint / finish node stands for (0x4f4db0 at 0x4f52ed..0x4f540b, 0x4f8ea0).
Point pointOf(World& w, uint32_t id) {
    Point p;
    const Node* n = w.node(id);
    if (n == nullptr) return p;
    const M4& N = n->world;
    p.pos = {N[12], N[13], N[14]};
    p.quat = quatOf(N);
    if (const auto B = w.box(n->object)) {
        // Combined matrix C = B x N (0x5857b0, row vectors); centre = its translation, the world-aligned
        // half extents (0x743210 + 0x4f96b0) = sum over rows i of |C[i][axis]|.
        M4 C{};
        for (int r = 0; r < 4; ++r)
            for (int c = 0; c < 4; ++c) {
                float s = 0;
                for (int k = 0; k < 4; ++k) s += (*B)[size_t(r * 4 + k)] * N[size_t(k * 4 + c)];
                C[size_t(r * 4 + c)] = s;
            }
        p.pos = {C[12], C[13], C[14]};
        float ex = 0, ez = 0;
        for (int i = 0; i < 3; ++i) ex += std::fabs(C[size_t(i * 4 + 0)]), ez += std::fabs(C[size_t(i * 4 + 2)]);
        p.width = std::sqrt(ex * ex + ez * ez);
    }
    return p;
}

// 0x4f5970: the node itself, then the nodes below it, then the nodes after it.
uint32_t findKeyword(World& w, uint32_t id, const char* key, int depth = 0) {
    if (id == 0 || depth > 256) return 0;
    if (w.hasUserDefine(id) && w.keyword(id, key).found) return id;
    for (uint32_t c = w.child(id); c != 0; c = w.sibling(c))
        if (uint32_t r = findKeyword(w, c, key, depth + 1)) return r;
    if (uint32_t s = w.sibling(id)) return findKeyword(w, s, key, depth + 1);
    return 0;
}

int toInt(const std::string& s) { return int(std::strtol(s.c_str(), nullptr, 10)); }
float toFloat(const std::string& s) { return std::strtof(s.c_str(), nullptr); }

Race buildMission(World& w, uint32_t id) {
    Race r;
    r.worldCourse = true;
    r.showStartEnd = false;  // +0 = 0
    r.courseNode = id;
    if (auto k = w.keyword(id, "editRaceName"); k.found) r.name = k.args[0].substr(0, 31);
    for (const char* key : {"editRaceNbLoop", "editRaceNbLap"})
        if (auto k = w.keyword(id, key); k.found) r.laps = toInt(k.args[0]);
    if (auto k = w.keyword(id, "editRaceNbIA"); k.found) r.aiCount = toInt(k.args[0]);
    if (auto k = w.keyword(id, "editRaceTimer"); k.found) r.timeLimit = toFloat(k.args[0]);
    if (auto k = w.keyword(id, "editRaceScenario"); k.found) r.scenario = toInt(k.args[0]);
    if (auto k = w.keyword(id, "editRaceMode"); k.found) {
        const unsigned m = unsigned(toInt(k.args[0]));
        if (m < 7) r.mode = int(m);
    }
    if (auto k = w.keyword(id, "editRaceIAElastic"); k.found)
        for (int i = 0; i < 3; ++i) r.elastic[i] = toFloat(k.args[i]);
    r.popping = w.keyword(id, "EditRacePoppingCheckpoint").found;
    // 0x623ac0: the first child (+0xa4, then +0xac) whose object (+0x98) is a SplineGraph_Z (type 0x19).
    for (uint32_t c = w.child(id); c != 0 && !r.splineGraph; c = w.sibling(c))
        if (const Node* n = w.node(c); n != nullptr && n->object != 0) r.splineGraph = w.splineGraph(n->object);

    for (uint32_t c = w.child(id); c != 0; c = w.sibling(c)) {
        if (!w.hasUserDefine(c)) continue;
        if (w.keyword(c, "editRaceStart").found) {
            const Point p = pointOf(w, c);
            r.start = p.pos;
            r.startRot = rotationVector(p.quat);
            r.startYaw = r.startRot[1];
            r.startHalf = p.width;
        }
        if (w.keyword(c, "editRaceCheck").found) {
            Point p = pointOf(w, c);
            // 0x4f5a10: CP_LEFT / CP_RIGHT give the gate's centre, half-width and heading.
            const uint32_t left = findKeyword(w, c, "CP_LEFT"), right = findKeyword(w, c, "CP_RIGHT");
            const Node* L = left ? w.node(left) : nullptr;
            const Node* R = right ? w.node(right) : nullptr;
            if (L != nullptr && R != nullptr) {
                const float dx = R->world[12] - L->world[12], dz = R->world[14] - L->world[14];
                p.width = std::sqrt(dz * dz + dx * dx) * 0.5f;
                const float h = std::atan2(dz, -dx) * 0.5f;
                p.quat = {0, std::sin(h), 0, std::cos(h)};
                p.pos = {(L->world[12] + R->world[12]) * 0.5f, (L->world[13] + R->world[13]) * 0.5f,
                         (L->world[14] + R->world[14]) * 0.5f};
            }
            r.checkpoints.push_back(p.pos);
            r.checkpointRot.push_back(rotationVector(p.quat));
            r.checkpointFlags.push_back(0);
            r.checkpointHalf.push_back(p.width);
            r.checkpointNodes.push_back(r.popping ? c : 0);
        }
        if (w.keyword(c, "editRaceEnd").found) {
            const Point p = pointOf(w, c);
            r.finish = p.pos;
            r.finishRot = rotationVector(p.quat);
            r.finishYaw = r.finishRot[1];
            r.finishHalf = p.width;
        }
    }
    const float dx = r.finish[0] - r.start[0], dy = r.finish[1] - r.start[1], dz = r.finish[2] - r.start[2];
    if ((std::fabs(dx) >= 1e-6f || std::fabs(dy) >= 1e-6f || std::fabs(dz) >= 1e-6f) && dz * dz + dx * dx < 1.0f) {
        r.finish = r.start;  // [0x9963b8] = 1.0
        r.finishRot = r.startRot;
        r.finishYaw = r.startYaw;
    }
    return r;
}

}  // namespace

std::vector<Race> loadWorldMissions(const DpcFile& usa1) {
    std::vector<Race> out;
    uint32_t root = 0;
    for (uint32_t id : usa1.ids(kWorldClass))
        if (const auto b = usa1.body(*usa1.find(id)); b && b->size() >= 4) std::memcpy(&root, b->data(), 4);
    if (root == 0) return out;
    World w(usa1);
    std::unordered_set<uint32_t> seen;
    // 0x4f4db0: for each node of the sibling list: a mission when MISSION_COURSE, then its children.
    std::function<void(uint32_t, int)> walk = [&](uint32_t id, int depth) {
        for (; id != 0 && depth < 512; id = w.sibling(id)) {
            if (!seen.insert(id).second) return;  // port: guard against a malformed cycle
            if (w.hasUserDefine(id) && w.keyword(id, "MISSION_COURSE").found) out.push_back(buildMission(w, id));
            walk(w.child(id), depth + 1);
        }
    };
    walk(root, 0);
    return out;
}

// PK05: Missions_SphereClear's grid items (0x4f9080) and the POI list (0x4ed9e0) for Decor_FreeOfObjects.
size_t loadPlantClearance(Vegetation& v, const DpcFile& usa1, const std::string& dataDir, const RoadGraph* graph, const RoadTypeParams* types,
                          size_t typeCount) {
    namespace fs = std::filesystem;
    std::vector<Race> missions;
    std::set<std::string> seen;
    auto add = [&](const Race& r) {
        if (seen.insert(r.name).second) missions.push_back(r);
    };
    try {
        for (const Event& e : loadEvents(dataDir)) add(e.race);  // aiCount = AddMissionParam's list length (0x575b00)
    } catch (const std::exception&) {
    }
    try {
        for (const char* file : {"miss_official.tsc", "missChallenge.tsc"}) {
            std::ifstream in(fs::path(dataDir) / "LocalizedComponent/GameTsc/Story" / file, std::ios::binary);
            std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
            for (const Race& r : parseMissions(text)) add(r);
        }
    } catch (const std::exception&) {
    }
    for (const Race& r : loadWorldMissions(usa1)) add(r);

    auto roadHalf = [&](float x, float z) {  // 0x4f8e00: [0xa678ac + 0x200 * type] of the road under the gate, 20 m without
        if (graph == nullptr || types == nullptr) return 20.0f;
        const int rec = graph->roadAt(x, z, [&](int t) { return t >= 0 && size_t(t) < typeCount ? types[t].flat : 0.0f; });
        if (rec < 0) return 20.0f;
        const int t = graph->rec(size_t(rec)).type;
        return t >= 0 && size_t(t) < typeCount ? types[t].flat : 20.0f;
    };
    v.clearance.gates.clear();
    for (const Race& r : missions) {
        const float box = float((r.aiCount + 1) / 2 + 1) * 10.0f;
        auto push = [&](float x, float z, float half) { v.clearance.gates.push_back({x, z, half, roadHalf(x, z)}); };
        push(r.start[0], r.start[2], box);
        for (const auto& c : r.checkpoints) push(c[0], c[2], 25.0f);
        push(r.finish[0], r.finish[2], box);
    }
    v.clearance.pois.clear();
    try {
        std::ifstream in(fs::path(dataDir) / "LocalizedComponent/GameTsc/Story/poi.tsc", std::ios::binary);
        std::string text((std::istreambuf_iterator<char>(in)), std::istreambuf_iterator<char>());
        for (const PoiPoint& p : parsePoiScript(text)) v.clearance.pois.push_back({p.x, p.z});
    } catch (const std::exception&) {
    }
    v.clearance.index();
    return v.clearance.gates.size();
}

// ---- PK14: 0x4f5b90 (races.hpp).
void snapMissionHeights(Race& r, const std::function<float(float x, float z)>& coarseHeight) {
    auto h = [&](float x, float z) { return coarseHeight ? coarseHeight(x, z) : 0.0f; };
    r.start[1] = h(r.start[0], r.start[2]);    // +0x10
    for (auto& p : r.checkpoints) p[1] = h(p[0], p[2]);  // +0x64[k] +4
    r.finish[1] = h(r.finish[0], r.finish[2]);  // +0x1c
}
// ---- end PK14

}  // namespace fuel
