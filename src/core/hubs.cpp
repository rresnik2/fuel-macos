#include "hubs.hpp"

#include "assembly.hpp"
#include "dpc.hpp"
#include "resources.hpp"
#include "world.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <sstream>

namespace fuel {

namespace {

namespace fs = std::filesystem;

// Tokens of one script line: quoted strings or bare words; "//" starts a comment.
std::vector<std::string> tokens(const std::string& line) {
    std::vector<std::string> out;
    size_t i = 0;
    while (i < line.size()) {
        const char c = line[i];
        if (c == ' ' || c == '\t' || c == '\r') {
            ++i;
        } else if (line.compare(i, 2, "//") == 0) {
            break;
        } else if (c == '"') {
            const size_t end = line.find('"', i + 1);
            out.push_back(line.substr(i + 1, end == std::string::npos ? std::string::npos : end - i - 1));
            i = end == std::string::npos ? line.size() : end + 1;
        } else {
            size_t end = i;
            while (end < line.size() && line[end] != ' ' && line[end] != '\t' && line[end] != '\r') ++end;
            out.push_back(line.substr(i, end - i));
            i = end;
        }
    }
    return out;
}

std::string readText(const fs::path& p) {
    std::ifstream f(p, std::ios::binary);
    std::stringstream s;
    s << f.rdbuf();
    return s.str();
}

std::string upper(std::string s) {
    for (char& c : s) c = char(std::toupper(static_cast<unsigned char>(c)));
    return s;
}

std::string squash(const std::string& s) {
    std::string out;
    for (char c : s)
        if (std::isalnum(static_cast<unsigned char>(c))) out += char(std::toupper(static_cast<unsigned char>(c)));
    return out;
}

struct Poi {
    std::string type, name, hub;
    float x = 0, z = 0;
};

struct Xz {
    float x = 0, z = 0;
};

// What a hub's sub-world (USA1_REGION_HUB_*.DPC) tells about its camp.
struct Region {
    std::string file;
    std::vector<Xz> points;      // the main cluster's object positions (2 km single linkage)
    float x0 = 0, z0 = 0, x1 = 0, z1 = 0;  // their box
    Xz centre() const { return {(x0 + x1) / 2, (z0 + z1) / 2}; }
};

// Largest group of `pts` in which every point has a neighbour within `link` metres (x and z).
std::vector<Xz> mainCluster(const std::vector<Xz>& pts, float link = 2000.0f) {
    std::vector<int> label(pts.size(), -1);
    int best = -1;
    size_t bestSize = 0;
    int next = 0;
    for (size_t i = 0; i < pts.size(); ++i) {
        if (label[i] >= 0) continue;
        std::vector<size_t> stack{i};
        label[i] = next;
        size_t size = 0;
        while (!stack.empty()) {
            const size_t j = stack.back();
            stack.pop_back();
            ++size;
            for (size_t m = 0; m < pts.size(); ++m)
                if (label[m] < 0 && std::fabs(pts[m].x - pts[j].x) < link && std::fabs(pts[m].z - pts[j].z) < link) label[m] = next, stack.push_back(m);
        }
        if (size > bestSize) bestSize = size, best = next;
        ++next;
    }
    std::vector<Xz> out;
    for (size_t i = 0; i < pts.size(); ++i)
        if (label[i] == best) out.push_back(pts[i]);
    return out;
}

// A hub camp's road graph: SplineGraph_Z in USA1.DPC (u32 n, n x xyz).
std::vector<std::vector<std::array<float, 3>>> loadSplineGraphs(const DpcFile& usa1) {
    std::vector<std::vector<std::array<float, 3>>> out;
    for (uint32_t id : usa1.ids(asoboCrc("SplineGraph_Z"))) {
        const DpcFile::Entry* e = usa1.find(id);
        if (e == nullptr) continue;
        const auto body = usa1.body(*e);
        if (!body || body->size() < 4) continue;
        uint32_t n = 0;
        std::memcpy(&n, body->data(), 4);
        if (n == 0 || size_t(n) * 12 + 4 > body->size()) continue;
        std::vector<std::array<float, 3>> pts(n);
        std::memcpy(pts.data(), body->data() + 4, size_t(n) * 12);
        out.push_back(std::move(pts));
    }
    return out;
}

float yawToward(float fx, float fz, float tx, float tz) { return std::atan2(tx - fx, tz - fz); }

// 0x577630 (HubVolumeAgent_G init via 0x5771d0): the hub volume is the USA1.DPC node whose object is a
// CollisionVol_Z and whose UserDefine's first line is the hub id ("MOUNTAIN 2"); its DIRECT children are matched by
// name CRC: CAMERA_CAREER -> H+0x1f0 with its first child TARGET_CAREER -> H+0x1fc (also START1, END1, CAMERA_HUB /
// TARGET_HUB, HUB_BOUNDARIES; the shipped volumes have no SHOP / CHALLENGE cameras). TeleportToHub 0x5495a0 puts the
// player at CAMERA_CAREER x, z (y from the ground), facing atan2(T.x - C.x, T.z - C.z) (0x40f550).
// PK03: also the volume's boxes, END1 (H+0x18c..+0x194: the node's world translation) and HUB_BOUNDARIES: its object
// (node +0x98, a Spline_Z: u32 n, n x 12-B points, u32 m, m x 0xf0-B segment records, 5 floats; loader 0x777d00) gives
// 8 points per record (record +0x1c + 0x1c k, 3 floats) through the node's world matrix (0x4126b0), (x, z) pushed to
// H+0x40 (0x482170) in record order. Returns hub id (upper case) -> record.
struct VolumeRec {
    HubPoint camp;
    bool hasCamp = false;
    std::vector<HubBox> boxes;
    std::vector<std::array<float, 2>> boundary;
    HubPoint end1;
    bool hasEnd1 = false;
};
std::map<std::string, VolumeRec> hubVolumes(const DpcFile& usa1) {
    constexpr uint32_t kNodeClass = 0x85D02528, kCollisionVolClass = 0x8EF49632, kUserDefineClass = 0x52F79F96;
    const uint32_t kCameraCareer = asoboCrc("CAMERA_CAREER"), kTargetCareer = asoboCrc("TARGET_CAREER");
    const uint32_t kEnd1 = asoboCrc("END1"), kBoundaries = asoboCrc("HUB_BOUNDARIES"), kSpline = 0x43A9B06Fu;  // Spline_Z (p4-dpc-classes)
    std::map<uint32_t, Node> nodes;
    std::map<uint32_t, uint32_t> names;
    for (uint32_t id : usa1.ids(kNodeClass)) try {
            const DpcFile::Entry* e = usa1.find(id);
            if (e == nullptr) continue;
            const auto b = usa1.body(*e);
            if (!b) continue;
            nodes[id] = parseNodeBody(*b);
            const std::vector<uint8_t> link = usa1.link(*e);  // link +0: AsoboCrc of the node name (short node link)
            uint32_t nm = 0;
            if (link.size() >= 4) std::memcpy(&nm, link.data(), 4);
            names[id] = nm;
        } catch (const std::exception&) {
        }
    auto xform = [](const std::array<float, 16>& m, const float* p) {  // 0x4126b0, row vectors
        return std::array<float, 3>{p[0] * m[0] + p[1] * m[4] + p[2] * m[8] + m[12], p[0] * m[1] + p[1] * m[5] + p[2] * m[9] + m[13],
                                    p[0] * m[2] + p[1] * m[6] + p[2] * m[10] + m[14]};
    };
    std::map<std::string, VolumeRec> out;
    for (const auto& [id, n] : nodes) {
        if (n.userDefine == 0 || n.object == 0 || n.child == 0) continue;
        const DpcFile::Entry* oe = usa1.find(n.object);
        const DpcFile::Entry* ue = usa1.find(n.userDefine);
        if (oe == nullptr || oe->classCrc != kCollisionVolClass || ue == nullptr || ue->classCrc != kUserDefineClass) continue;
        std::vector<std::string> lines;
        try {
            if (const auto b = usa1.body(*ue)) lines = parseUserDefineBody(*b);
        } catch (const std::exception&) {
        }
        if (lines.empty()) continue;
        std::string hubId = upper(lines[0]);
        while (!hubId.empty() && (hubId.back() == ' ' || hubId.back() == '\r' || hubId.back() == '\t')) hubId.pop_back();
        VolumeRec rec;
        // The boxes: CollisionVol_Z body u32 n, n x 0x80 (matrix, inverse), each x the node matrix.
        if (const auto vb = usa1.body(*oe); vb && vb->size() >= 4) {
            uint32_t nb = 0;
            std::memcpy(&nb, vb->data(), 4);
            for (uint32_t k = 0; k < nb && 4 + size_t(k + 1) * 0x80 <= vb->size(); ++k) {
                float b[16];
                std::memcpy(b, vb->data() + 4 + size_t(k) * 0x80, 64);
                float m[16];
                for (int r = 0; r < 4; ++r)
                    for (int c = 0; c < 4; ++c) {
                        float s = 0;
                        for (int i = 0; i < 4; ++i) s += b[r * 4 + i] * n.world[size_t(i * 4 + c)];
                        m[r * 4 + c] = s;
                    }
                HubBox box;
                for (int r = 0; r < 3; ++r) {
                    const float len = std::sqrt(m[r * 4] * m[r * 4] + m[r * 4 + 1] * m[r * 4 + 1] + m[r * 4 + 2] * m[r * 4 + 2]);
                    box.half[size_t(r)] = len;
                    for (int c = 0; c < 3; ++c) box.axis[size_t(r)][size_t(c)] = len > 0 ? m[r * 4 + c] / len : float(r == c);
                }
                box.centre = {m[12], m[13], m[14]};
                rec.boxes.push_back(box);
            }
        }
        // Direct children: first child (+0xa4), then the sibling chain (+0xac, Node::prev as in world_missions.cpp).
        for (uint32_t c = n.child, guard = 0; c != 0 && guard < 4096; ++guard) {
            const auto it = nodes.find(c);
            if (it == nodes.end()) break;
            const Node& cn = it->second;
            if (!rec.hasCamp && names[c] == kCameraCareer && cn.child != 0 && names[cn.child] == kTargetCareer) {
                const Node& t = nodes[cn.child];
                rec.camp.x = cn.world[12], rec.camp.z = cn.world[14];
                rec.camp.yaw = yawToward(rec.camp.x, rec.camp.z, t.world[12], t.world[14]);
                rec.hasCamp = true;
            } else if (names[c] == kEnd1) {
                rec.end1.name = hubId + "_END1";
                rec.end1.x = cn.world[12], rec.end1.z = cn.world[14];
                rec.hasEnd1 = true;
            } else if (names[c] == kBoundaries && cn.object != 0) {
                const DpcFile::Entry* se = usa1.find(cn.object);
                const auto sb = se != nullptr && se->classCrc == kSpline ? usa1.body(*se) : std::nullopt;
                if (sb && sb->size() >= 8) {
                    uint32_t np = 0, ns = 0;
                    std::memcpy(&np, sb->data(), 4);
                    const size_t off = 4 + size_t(np) * 12;
                    if (off + 4 <= sb->size()) std::memcpy(&ns, sb->data() + off, 4);
                    for (uint32_t k = 0; k < ns && off + 4 + size_t(k + 1) * 0xf0 <= sb->size(); ++k)
                        for (int j = 0; j < 8; ++j) {
                            float p[3];
                            std::memcpy(p, sb->data() + off + 4 + size_t(k) * 0xf0 + 0x1c + size_t(j) * 0x1c, 12);
                            const auto w = xform(cn.world, p);
                            rec.boundary.push_back({w[0], w[2]});
                        }
                }
            }
            c = cn.prev;
        }
        out.emplace(hubId, std::move(rec));
    }
    return out;
}

}  // namespace

std::vector<Hub> loadHubs(const std::string& dataDir) {
    const fs::path story = fs::path(dataDir) / "LocalizedComponent/GameTsc/Story";

    // 1. hubinfos.tsc: SetHubTargetInfos "<hub>" tier textId ... in career order.
    std::vector<Hub> hubs;
    std::vector<std::string> spelled;  // the hub's name as the scripts spell it (poi.tsc uses the same)
    {
        std::istringstream in(readText(story / "hubinfos.tsc"));
        std::string line;
        while (std::getline(in, line)) {
            const auto t = tokens(line);
            if (t.size() < 4 || t[0] != "SetHubTargetInfos") continue;
            Hub h;
            h.id = upper(t[1]);
            h.tier = std::atoi(t[2].c_str());
            h.nameText = uint32_t(std::strtoul(t[3].c_str(), nullptr, 10));
            hubs.push_back(h);
            spelled.push_back(t[1]);
        }
    }
    std::vector<size_t> order(hubs.size());
    for (size_t i = 0; i < order.size(); ++i) order[i] = i;
    std::stable_sort(order.begin(), order.end(), [&](size_t a, size_t b) { return hubs[a].tier < hubs[b].tier; });

    // 2. poi.tsc: AddTypePointOfInterest "<Teleport|VP>" id x z "<name>" "<hub>" (height comes from the terrain).
    std::vector<Poi> pois;
    {
        std::istringstream in(readText(story / "poi.tsc"));
        std::string line;
        while (std::getline(in, line)) {
            const auto t = tokens(line);
            if (t.size() < 7 || t[0] != "AddTypePointOfInterest") continue;
            pois.push_back({t[1], t[5], t[6], std::strtof(t[3].c_str(), nullptr), std::strtof(t[4].c_str(), nullptr)});
        }
    }

    // 3. Camps. Neither script gives a camp position, so it is derived from the world files:
    //    each hub's sub-world USA1_REGION_HUB_*.DPC (the camp and outpost dressing) is matched to the
    //    hub whose teleports centre on it, and the camp is the middle of the main cluster of
    //    objects; where USA1.DPC has a SplineGraph_Z (the camp's road graph) inside that cluster,
    //    the camp is the graph's point nearest its centre (a spot on a road). The heading runs along
    //    the road there, or toward the objects round the camp when there is no graph, and points
    //    into the camp (toward the objects, or the hub's teleports when those are too close to tell).
    //    P76: FUEL's own camp is the hub volume's CAMERA_CAREER node (0x577630 / 0x5495a0, hubVolumeCamps); the
    //    heuristic below is only the fallback for a hub without a volume (none in the shipped USA1.DPC).
    std::vector<Region> regions;
    std::vector<std::vector<std::array<float, 3>>> graphs;
    std::map<std::string, VolumeRec> volumeCamps;
    try {
        const fs::path world = fs::path(dataDir) / "CommonComponent/WORLD";
        DpcFile usa1((world / "USA1.DPC").string());
        volumeCamps = hubVolumes(usa1);
        graphs = loadSplineGraphs(usa1);
        std::vector<fs::path> files;
        for (const auto& e : fs::directory_iterator(world))
            if (e.path().filename().string().rfind("USA1_REGION_HUB_", 0) == 0 && e.path().extension() == ".DPC") files.push_back(e.path());
        std::sort(files.begin(), files.end());
        for (const fs::path& f : files) {
            try {
                DpcFile sub(f.string());
                if (!sub.isCurrentVersion()) continue;
                std::vector<Xz> pts;
                for (const PlacedMesh& pm : collectPlacedMeshes(sub, {&sub, &usa1}))
                    if (std::fabs(pm.world[12]) > 50 || std::fabs(pm.world[14]) > 50) pts.push_back({pm.world[12], pm.world[14]});
                if (pts.empty()) continue;
                Region r;
                r.file = f.filename().string();
                r.points = mainCluster(pts);
                r.x0 = r.x1 = r.points[0].x, r.z0 = r.z1 = r.points[0].z;
                for (const Xz& p : r.points)
                    r.x0 = std::min(r.x0, p.x), r.x1 = std::max(r.x1, p.x), r.z0 = std::min(r.z0, p.z), r.z1 = std::max(r.z1, p.z);
                regions.push_back(std::move(r));
            } catch (const std::exception&) {
            }
        }
    } catch (const std::exception&) {
        regions.clear();
    }

    // Teleport centroid and spread per hub, then pair hubs with regions, nearest pairs first.
    struct Centroid {
        Xz c;
        float spread = 0;
        bool any = false;
    };
    std::vector<Centroid> cent(hubs.size());
    for (size_t i = 0; i < hubs.size(); ++i) {
        int n = 0;
        for (const Poi& p : pois)
            if (p.type == "Teleport" && p.hub == spelled[i]) cent[i].c.x += p.x, cent[i].c.z += p.z, ++n;
        if (n == 0) continue;
        cent[i].c.x /= float(n), cent[i].c.z /= float(n), cent[i].any = true;
        for (const Poi& p : pois)
            if (p.type == "Teleport" && p.hub == spelled[i]) cent[i].spread = std::max(cent[i].spread, std::hypot(p.x - cent[i].c.x, p.z - cent[i].c.z));
    }
    struct Pair {
        float d;
        size_t hub, region;
    };
    std::vector<Pair> pairs;
    for (size_t i = 0; i < hubs.size(); ++i)
        for (size_t r = 0; r < regions.size(); ++r) {
            if (!cent[i].any) continue;
            const Xz c = regions[r].centre();
            const float d = std::hypot(c.x - cent[i].c.x, c.z - cent[i].c.z);
            if (d < cent[i].spread + 3000.0f) pairs.push_back({d, i, r});
        }
    std::sort(pairs.begin(), pairs.end(), [](const Pair& a, const Pair& b) { return a.d < b.d; });
    std::vector<int> regionOf(hubs.size(), -1);
    std::vector<bool> taken(regions.size(), false);
    for (const Pair& p : pairs)
        if (regionOf[p.hub] < 0 && !taken[p.region]) regionOf[p.hub] = int(p.region), taken[p.region] = true;

    for (size_t i = 0; i < hubs.size(); ++i) {
        Hub& h = hubs[i];
        for (const Poi& p : pois) {
            if (p.hub != spelled[i]) continue;
            HubPoint hp;
            hp.name = p.name;
            hp.x = p.x, hp.z = p.z;
            if (p.type == "Teleport") h.teleports.push_back(hp);
            else if (p.type == "VP") h.vistas.push_back(hp);
        }
        HubPoint& camp = h.camp;
        camp.name = spelled[i] + "_Camp";
        const Xz tele = cent[i].c;
        const auto vc = volumeCamps.find(upper(spelled[i]));
        if (vc != volumeCamps.end()) {  // PK03: the hub record's volume, boundary and END1
            h.volume = vc->second.boxes, h.boundary = vc->second.boundary, h.end1 = vc->second.end1, h.hasEnd1 = vc->second.hasEnd1;
        }
        if (vc != volumeCamps.end() && vc->second.hasCamp) {
            camp.x = vc->second.camp.x, camp.z = vc->second.camp.z, camp.yaw = vc->second.camp.yaw;
        } else if (regionOf[i] < 0) {  // no sub-world: the middle of the hub's teleports, facing north
            camp.x = tele.x, camp.z = tele.z;
        } else {
            const Region& r = regions[size_t(regionOf[i])];
            const Xz centre = r.centre();
            camp.x = centre.x, camp.z = centre.z;
            // The road graph inside the cluster whose middle is nearest the cluster's.
            const std::vector<std::array<float, 3>>* graph = nullptr;
            float graphD = 1e30f;
            for (const auto& g : graphs) {
                double gx = 0, gz = 0;
                for (const auto& p : g) gx += p[0], gz += p[2];
                gx /= double(g.size()), gz /= double(g.size());
                const float pad = 300.0f;
                if (gx < r.x0 - pad || gx > r.x1 + pad || gz < r.z0 - pad || gz > r.z1 + pad) continue;
                const float d = float(std::hypot(gx - centre.x, gz - centre.z));
                if (d < graphD) graphD = d, graph = &g;
            }
            std::vector<Xz> road;  // graph points near the camp
            if (graph != nullptr) {
                double gx = 0, gz = 0;
                for (const auto& p : *graph) gx += p[0], gz += p[2];
                gx /= double(graph->size()), gz /= double(graph->size());
                float best = 1e30f;
                for (const auto& p : *graph) {
                    const float d = float(std::hypot(p[0] - gx, p[2] - gz));
                    if (d < best) best = d, camp.x = p[0], camp.z = p[2];
                }
                for (const auto& p : *graph)
                    if (std::hypot(p[0] - camp.x, p[2] - camp.z) < 80.0f) road.push_back({p[0], p[2]});
            }
            // Objects round the camp: where "into the camp" is.
            double ox = 0, oz = 0;
            int on = 0;
            for (const Xz& p : r.points)
                if (std::hypot(p.x - camp.x, p.z - camp.z) < 250.0f) ox += p.x, oz += p.z, ++on;
            Xz toward = tele;
            if (on > 0 && std::hypot(ox / on - camp.x, oz / on - camp.z) > 15.0f) toward = {float(ox / on), float(oz / on)};
            if (road.size() >= 3) {  // along the road, the way that leads toward `toward`
                double mx = 0, mz = 0;
                for (const Xz& p : road) mx += p.x, mz += p.z;
                mx /= double(road.size()), mz /= double(road.size());
                double sxx = 0, szz = 0, sxz = 0;
                for (const Xz& p : road) sxx += (p.x - mx) * (p.x - mx), szz += (p.z - mz) * (p.z - mz), sxz += (p.x - mx) * (p.z - mz);
                const double a = 0.5 * std::atan2(2 * sxz, sxx - szz);  // principal axis in the (x, z) plane
                float ax = float(std::cos(a)), az = float(std::sin(a));
                if (ax * (toward.x - camp.x) + az * (toward.z - camp.z) < 0) ax = -ax, az = -az;
                camp.yaw = std::atan2(ax, az);
            } else {
                camp.yaw = yawToward(camp.x, camp.z, toward.x, toward.z);
            }
        }
        // Teleports and vistas face the camp (the scripts give no heading).
        for (auto* list : {&h.teleports, &h.vistas})
            for (HubPoint& p : *list) p.yaw = yawToward(p.x, p.z, camp.x, camp.z);
    }

    std::vector<Hub> out;
    for (size_t i : order) out.push_back(std::move(hubs[i]));
    return out;
}

const Hub* findHub(const std::vector<Hub>& hubs, const std::string& name) {
    const std::string want = squash(name);
    if (want.empty()) return nullptr;
    for (const Hub& h : hubs)
        if (squash(h.id) == want) return &h;
    return nullptr;
}

const Hub* nearestHub(const std::vector<Hub>& hubs, float x, float z) {
    const Hub* best = nullptr;
    float bestD = 1e30f;
    auto test = [&](const Hub& h, const HubPoint& p) {
        const float d = std::hypot(p.x - x, p.z - z);
        if (d < bestD) bestD = d, best = &h;
    };
    for (const Hub& h : hubs) {
        test(h, h.camp);
        for (const HubPoint& p : h.teleports) test(h, p);
        for (const HubPoint& p : h.vistas) test(h, p);
    }
    return best;
}

bool nudgeToClearSpot(float& x, float& z, const std::function<bool(float, float)>& isClear, float maxRadius) {
    if (isClear(x, z)) return true;
    constexpr float kStep = 4.0f;
    for (float r = kStep; r <= maxRadius; r += kStep) {
        const int n = std::max(8, int(2 * 3.14159265f * r / kStep));
        for (int k = 0; k < n; ++k) {
            const float a = 2 * 3.14159265f * float(k) / float(n);
            const float px = x + r * std::cos(a), pz = z + r * std::sin(a);
            if (isClear(px, pz)) {
                x = px, z = pz;
                return true;
            }
        }
    }
    return false;
}

// ---- PK03: hub presence and arrival ---------------------------------------------------------------------------------

bool HubBox::contains(float x, float y, float z) const {
    const float d[3] = {x - centre[0], y - centre[1], z - centre[2]};
    for (size_t i = 0; i < 3; ++i)
        if (std::fabs(d[0] * axis[i][0] + d[1] * axis[i][1] + d[2] * axis[i][2]) > half[i]) return false;
    return true;
}

// 0x7652b0 (ESI = the point, args = points, count): for each edge (P[i], P[i+1], the last back to P[0]) whose z range
// holds pz half-open ((a.z < pz <= b.z) or (b.z < pz <= a.z)): right of both ends counts, left of (or on) both does
// not, otherwise the cross product of the edge taken upward (low z end first) with the point counts when < 0.
// Returns the parity.
bool insideBoundary(const Hub& hub, float px, float pz) {
    const auto& P = hub.boundary;
    const size_t n = P.size();
    unsigned c = 0;
    for (size_t i = 0; i < n; ++i) {
        const auto& a = P[i];
        const auto& b = P[i + 1 == n ? 0 : i + 1];
        if (!((a[1] < pz && pz <= b[1]) || (b[1] < pz && pz <= a[1]))) continue;
        if (px > b[0] && px > a[0]) {
            ++c;
            continue;
        }
        if (px <= b[0] && px <= a[0]) continue;
        const auto& lo = a[1] <= b[1] ? a : b;
        const auto& hi = a[1] <= b[1] ? b : a;
        const float f = (pz - lo[1]) * (hi[0] - lo[0]) - (px - lo[0]) * (hi[1] - lo[1]);
        if (f < 0) ++c;
    }
    return (c & 1) != 0;
}

void HubPresence::volumes(float x, float y, float z, bool race, int zone) {
    zone_ = zone;  // 0x62f680: +0x374 = Zone_FindContaining(player position) while a player exists
    inside_.resize(hubs_.size(), false);
    std::vector<int> left, entered;
    for (size_t h = 0; h < hubs_.size(); ++h) {
        bool in = false;
        for (const HubBox& b : hubs_[h].volume) in = in || b.contains(x, y, z);
        if (in != inside_[h]) (in ? entered : left).push_back(int(h));
        inside_[h] = in;
    }
    // Both handlers run only while no race runs (MissionStatus game+0x334c +0x48 is 0 or 1), for the player's vehicle
    // (agent +0x60 & 8; the port tests the player's position only).
    if (race) return;
    for (int h : left)  // vf44 0x577c90: this volume's hub is the pending one -> 0x577dc0 clears +0x954 and +0x958
        if (h == pending_) pending_ = -1, request_ = false;
    for (int h : entered)  // vf46 0x577bf0: +0x954 = this hub, +0x958 = 1, unless the pause mode is 2 (0x46e650)
        if (pauseMode(x, z, race) != 2) pending_ = h, request_ = true;
}

bool HubPresence::inPendingBoundary(float x, float z) const {
    // 0x481a40: a pending hub, game+0x2b8 clear (set by a loaded save's OK 0x4ce8e0 [U: who clears it]; the port never
    // sets it), a player, and its position (x, z) inside the hub's boundary (0x7652b0 on H+0x40).
    return pending_ >= 0 && size_t(pending_) < hubs_.size() && insideBoundary(hubs_[size_t(pending_)], x, z);
}

int HubPresence::pauseMode(float x, float z, bool race) const {
    if (race) return 3;  // 0x46e650: race +0x3b8 online -> 4 (no online play in the port)
    return inPendingBoundary(x, z) ? 2 : 1;
}

void HubPresence::startHubHUD(int hub) {
    if (hub < 0 || size_t(hub) >= hubs_.size()) return;
    pending_ = hub;   // StartHubHUD 0x5492d0: +0x954 = the hub record of uid argv[1]
    menuMode_ = 2;    // +0x94 = 2
    state_ = Wait;    // 0x47d970: "SetHudsVisibleState OFF", state 0xc4, title / items / tutorial refresh
}

HubPresence::Action HubPresence::update(float x, float z, bool race, bool movie, bool key) {
    Action a;
    switch (state_) {
        case Idle:  // 0x481b60
            // [U] first: IGP (+0x31fc -> +0x908) object +0x5c set and the 0x443260 object's +0x5c < 3 -> wait; the port has
            // neither object. Then no race running (+0x48 0 / 1) and 0x481a40 -> "TeleportToHub <uid>", +0x958 = 0.
            if (request_ && !race && inPendingBoundary(x, z)) {
                a.kind = Action::TeleportToHub, a.hub = pending_;
                request_ = false;
            }
            break;
        case Wait:  // 0x481c90: once the storage is idle (save object +0x76; the port saves synchronously) -> 0x481d70
            if (pending_ < 0) break;
            if (career_.unlocked && !career_.unlocked(pending_)) {  // 0x579720 fails -> 0x47da10
                state_ = Alert;                                      // vf22(vf09()) then +0x944 = 0xc7 (TT_PAGE_HUB_ALERT)
                a.kind = Action::OpenAlert;
                break;
            }
            if (career_.arrive) career_.arrive(pending_);  // game+0x33c0 = H+0x168, 0x56ca20(game+0xe4)
            if (career_.opened && !career_.opened(pending_)) {
                // 0x47d9c0 -> 0x481e40: LevelManipulator byte +0x3d = 0 [U], SetDayClock(43200.0), `ChoosePlayMovie <Hub_Name>`
                // (spaces -> '_', VideoComponent/<Hub>.bik) [port: no movie, Bink stays replaced]; then state 0xc5.
                if (career_.setDayClock) career_.setDayClock(43200.0);
                state_ = Open;
                break;
            }
            a.kind = Action::OpenMenu;  // vf22(vf09(), 0) and MusicManager_G +0x190 |= 8 (state stays 0xc4 until the close)
            break;
        case Open:  // 0x481cd0: a player, and MENU_BACK / MENU_VALID or no movie playing
            if (pending_ >= 0 && (key || !movie)) {
                if (career_.open) career_.open(pending_);  // +0x248 = 1, 0x57a100, 0x57a240, +0x80 = 1 (save at the close)
                a.kind = Action::OpenMenu;                  // vf22(1, 0)
            }
            break;
        default: break;  // 0xc7: the popup's OK (0x4ce850 -> 0x47f060)
    }
    return a;
}

void HubPresence::drop() {
    pending_ = -1;  // 0x47ee80 / 0x47f060: row +0x948 = 0, +0x88 = 2, +0x94c = 0, +0x954 = 0, state 0xc3
    state_ = Idle;
}

bool HubPresence::end1(float& tx, float& tz) const {
    if (pending_ < 0 || size_t(pending_) >= hubs_.size() || !hubs_[size_t(pending_)].hasEnd1) return false;
    tx = hubs_[size_t(pending_)].end1.x, tz = hubs_[size_t(pending_)].end1.z;
    return true;
}

bool HubPresence::menuClosed(bool back, float& tx, float& tz) {
    // 0x46fd60 / 0x46ea50: back or START with +0x94 == 2 -> 0x47f060 (only with a pending hub: TLP 0x4caa90 to its END1);
    // every other close: vt+0x20 + 0x47ee80.
    const bool tlp = back && menuMode_ == 2 && end1(tx, tz);
    drop();
    return tlp;
}

}  // namespace fuel
