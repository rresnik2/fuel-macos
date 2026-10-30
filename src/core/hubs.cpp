#include "hubs.hpp"

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
    std::vector<Region> regions;
    std::vector<std::vector<std::array<float, 3>>> graphs;
    try {
        const fs::path world = fs::path(dataDir) / "CommonComponent/WORLD";
        DpcFile usa1((world / "USA1.DPC").string());
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
        if (regionOf[i] < 0) {  // no sub-world: the middle of the hub's teleports, facing north
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

}  // namespace fuel
