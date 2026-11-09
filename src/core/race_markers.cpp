#include "race_markers.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <fstream>
#include <sstream>

namespace fuel {

namespace {

using Quat = std::array<float, 4>;
using Vec3 = std::array<float, 3>;

// 0x644980: angle |v|, axis v / |v|.
Quat quatFromRotationVector(const Vec3& v) {
    const double a = std::sqrt(double(v[0]) * v[0] + double(v[1]) * v[1] + double(v[2]) * v[2]);
    if (a < 1e-6) return {0, 0, 0, 1};
    const double s = std::sin(a * 0.5) / a;
    return {float(v[0] * s), float(v[1] * s), float(v[2] * s), float(std::cos(a * 0.5))};
}

// 0x410510 (Quat_RotateVec).
Vec3 rotate(const Quat& q, const Vec3& v) {
    const float x = q[0], y = q[1], z = q[2], w = q[3];
    const float m = -x * v[0] - y * v[1] - z * v[2];
    const float a = y * v[2] + w * v[0] - z * v[1];
    const float b = v[0] * z + w * v[1] - x * v[2];
    const float c = v[1] * x + w * v[2] - v[0] * y;
    return {c * y + ((w * a - m * x) - z * b), a * z + ((w * b - m * y) - c * x), x * b + ((c * w - m * z) - y * a)};
}

// Hamilton product a * b.
Quat mul(const Quat& a, const Quat& b) {
    return {a[3] * b[0] + a[0] * b[3] + a[1] * b[2] - a[2] * b[1], a[3] * b[1] - a[0] * b[2] + a[1] * b[3] + a[2] * b[0],
            a[3] * b[2] + a[0] * b[1] - a[1] * b[0] + a[2] * b[3], a[3] * b[3] - a[0] * b[0] - a[1] * b[1] - a[2] * b[2]};
}

float gateWidthAt(const RaceMarkerContext& c, const Vec3& p) { return c.gateWidth ? c.gateWidth(p[0], p[2]) : 20.0f; }

// 0x4f8680(position, text, mission, index, side, type, width): side 0 = on the centre, 1 = -half, 2 = +half along the
// rotation's x axis (0xa06338 = (1, 0, 0)), half = max(width / 2, PP360); types < 9 are turned by pi about y (0xa06320,
// 0x40f8b0 / 0x410260) after the offset; +0x6e (on the ground) and +0x6f (upright) set, +0x38 = -0.3.
void fill(std::vector<RaceMarker>& out, const Vec3& at, const std::string& text, int index, int side, int type, float width,
          const Quat& q, float minHalf) {
    RaceMarker m;
    m.type = type;
    m.index = index;
    m.pos = at;
    m.rot = q;
    if (side != 0) {
        float h = std::max(width * 0.5f, minHalf);
        if (side == 1) h = -h;
        const Vec3 o = rotate(q, {h, 0, 0});
        m.pos = {at[0] + o[0], at[1] + o[1], at[2] + o[2]};
    }
    if (type < 9) {
        const float s = std::sin(float(M_PI) * 0.5f), c = std::cos(float(M_PI) * 0.5f);  // Quat_FromAxisAngle(y, pi)
        m.rot = mul(Quat{0, s, 0, c}, q);
    }
    m.text = text;
    out.push_back(std::move(m));
}

}  // namespace

const char* raceMarkerClass(int type) {
    switch (type) {  // 0x4f8680 switch (table 0x4f891c, type - 3), names 0x9b30a8..0x9b3158
    case 3:
    case 4: return "MISSIONSTARTMIDDLE";
    case 5: return "MISSIONSTARTLEFT";
    case 6: return "MISSIONSTARTRIGHT";
    case 7: return "MISSIONFINISHMIDDLE";
    case 10: return "CHECKPOINT_FLAG_LEFT";
    case 11: return "CHECKPOINT_FLAG_RIGHT";
    case 12: return "CHECKPOINT_FLAG_LEFT02";
    case 13: return "CHECKPOINT_FLAG_RIGHT02";
    case 14:
    case 15: return "CHECKPOINT_FLAG_LIGHT";
    default: return nullptr;
    }
}

int checkpointMarkerFlag(int flag, int scenario, int nearestRoadType, bool roadAtPoint) {
    if (scenario == 11) return 0;           // 0x4f40f0: helicopter missions, all flags 0
    if (flag == 0 || flag == 4) return flag;
    if (nearestRoadType == -2) return roadAtPoint ? 2 : 1;  // none within 1000 m: 0x6d1d70
    const int t = nearestRoadType;          // record +0xb, a signed char
    return (t < 0 || (t > 16 && t != 20)) ? 1 : 2;
}

void startBanners(const Race& r, int missionIndex, const RaceMarkerContext& c, std::vector<RaceMarker>& out) {
    if (!r.showStartEnd) return;  // +0
    // 0x4f83d0 / end of 0x4f73b0: width = max(0x4fdea0, 20), rotation +0x24, at the start +0xc.
    const float w = std::max(gateWidthAt(c, r.start), 20.0f);
    const Quat q = quatFromRotationVector(r.startRot);
    fill(out, r.start, "", missionIndex, 2, 5, w, q, c.minHalf);
    fill(out, r.start, r.name, missionIndex, 0, 3, w, q, c.minHalf);  // +0xda, the mission name
    fill(out, r.start, "", missionIndex, 1, 6, w, q, c.minHalf);
}

void missionMarkers(const Race& r, int missionIndex, int target, const RaceMarkerContext& c, std::vector<RaceMarker>& out) {
    // Finish banners: no laps (+0x5c), banners on, finish more than 0.1 m from the start (x, z).
    if (r.laps == 0 && r.showStartEnd && std::hypot(r.start[2] - r.finish[2], r.start[0] - r.finish[0]) > 0.1f) {
        const float w = std::max(gateWidthAt(c, r.finish), 20.0f);
        const Quat q = quatFromRotationVector(r.finishRot);  // +0x34
        fill(out, r.finish, "", missionIndex, 1, 6, w, q, c.minHalf);
        fill(out, r.finish, "Finish", missionIndex, 0, 7, w, q, c.minHalf);
        fill(out, r.finish, "", missionIndex, 2, 5, w, q, c.minHalf);
    }
    // Checkpoints: +0x60 positions (+0x64), +0x6c flags, +0x7c rotations.
    for (size_t i = 0; i < r.checkpoints.size(); ++i) {
        const Vec3& p = r.checkpoints[i];
        int flag = i < r.checkpointFlags.size() ? r.checkpointFlags[i] : 1;
        if (c.flagAt) flag = c.flagAt(flag, p[0], p[2]);
        if (flag == 4) continue;
        const Quat q = quatFromRotationVector(i < r.checkpointRot.size() ? r.checkpointRot[i] : Vec3{0, r.finishYaw, 0});
        const float w = gateWidthAt(c, p);  // 0x4fdea0 at the checkpoint (EDI = +0x64 + 12 i)
        const int k = int(i);
        if (flag == 1) {
            fill(out, p, "", k, 1, 11, w, q, c.minHalf);
            fill(out, p, "", k, 2, 10, w, q, c.minHalf);
        } else if (flag == 2) {
            fill(out, p, "", k, 1, 13, w, q, c.minHalf);
            fill(out, p, "", k, 2, 12, w, q, c.minHalf);
        }
        if (flag != 0) {
            if (k == target) {  // the light on the player's current checkpoint
                fill(out, p, "", k, 1, 15, w, q, c.minHalf);
                fill(out, p, "", k, 2, 14, w, q, c.minHalf);
            }
            fill(out, p, "", k, 0, 9, w, q, c.minHalf);  // the checkpoint record itself (no object)
        }
    }
    startBanners(r, missionIndex, c, out);
}

std::array<float, 16> raceMarkerWorld(const RaceMarker& m, float ground, const std::array<float, 3>& normal) {
    if (!m.upright) {
        // 0x4fbf10 without +0x6f: the record's rotation turned onto the ground normal (0x640690) [I: shortest arc].
        Vec3 n = normal;
        const float l = std::sqrt(n[0] * n[0] + n[1] * n[1] + n[2] * n[2]);
        Quat tilt{0, 0, 0, 1};
        if (l > 1e-6f) {
            n = {n[0] / l, n[1] / l, n[2] / l};
            // arc (0, 1, 0) -> n: axis = up x n = (n.z, 0, -n.x), cos = n.y
            const float c = n[1], w = std::sqrt(std::max(0.0f, (1.0f + c) * 0.5f));
            if (w > 1e-4f) tilt = {n[2] / (2 * w), 0, -n[0] / (2 * w), w};
        }
        const Quat q = mul(tilt, m.rot);
        const Vec3 x = rotate(q, {1, 0, 0}), y = rotate(q, {0, 1, 0}), z = rotate(q, {0, 0, 1});
        return {x[0], x[1], x[2], 0, y[0], y[1], y[2], 0, z[0], z[1], z[2], 0, m.pos[0], ground + m.lift, m.pos[2], 1};
    }
    // v = rot (0, 0, 1) (0xa06350), flattened and normalised (0x40fc80), heading = the arc from (0, 0, 1) to v (0x6443a0).
    Vec3 v = rotate(m.rot, {0, 0, 1});
    const float l2 = v[0] * v[0] + v[2] * v[2];
    float s = 0, c = 1;
    if (l2 > 1e-6f) {
        const float l = std::sqrt(l2);
        s = v[0] / l, c = v[2] / l;
    }
    return {c, 0, -s, 0, 0, 1, 0, 0, s, 0, c, 0, m.pos[0], ground + m.lift, m.pos[2], 1};
}

const char* poiMarkerClass(int poiType) {
    switch (poiType) {  // 0x4ed2d0's switch, names 0x9b2c34..0x9b2c78
        case 1: return "IT_PILOTLIVERY_DYN";
        case 2: return "IT_LIVERY";  // only without the vehicle table (game +0x3258), which retail always has
        case 3: return "IT_DISCOVERY";
        case 4: return "CAMIONHUB_RESPAWN";
        case 5: return "IT_FUEL01";
        default: return nullptr;
    }
}

std::vector<PoiPoint> parsePoiScript(const std::string& text) {
    std::vector<PoiPoint> out;
    std::istringstream in(text);
    std::string line;
    while (std::getline(in, line)) {
        // tokens: words, or "quoted strings"; "//" starts a comment
        std::vector<std::string> t;
        for (size_t i = 0; i < line.size();) {
            const char c = line[i];
            if (c == '/' && i + 1 < line.size() && line[i + 1] == '/') break;
            if (std::isspace(static_cast<unsigned char>(c))) { ++i; continue; }
            if (c == '"') {
                const size_t e = line.find('"', i + 1);
                t.push_back(line.substr(i + 1, (e == std::string::npos ? line.size() : e) - i - 1));
                i = e == std::string::npos ? line.size() : e + 1;
            } else {
                size_t e = i;
                while (e < line.size() && !std::isspace(static_cast<unsigned char>(line[e]))) ++e;
                t.push_back(line.substr(i, e - i));
                i = e;
            }
        }
        if (t.size() < 6 || t[0] != "AddTypePointOfInterest") continue;
        std::string kind = t[1];
        for (char& ch : kind) ch = char(std::toupper(static_cast<unsigned char>(ch)));
        PoiPoint p;
        p.type = kind == "TELEPORT" ? 4 : kind == "VP" ? 3 : 0;
        if (!p.type) continue;
        p.x = float(std::atof(t[3].c_str())), p.z = float(std::atof(t[4].c_str()));
        p.name = t[5];
        out.push_back(std::move(p));
    }
    return out;
}

std::vector<PoiPoint> worldPoiList(const std::string& hubinfosText, const std::string& poiText) {
    std::vector<PoiPoint> out;
    std::istringstream in(hubinfosText);
    std::string line;
    while (std::getline(in, line)) {
        // AddVehicleLivery MESH n "hub" text KIND arg x z (words or "quoted strings"; "//" starts a comment)
        std::vector<std::string> t;
        for (size_t i = 0; i < line.size();) {
            const char c = line[i];
            if (c == '/' && i + 1 < line.size() && line[i + 1] == '/') break;
            if (std::isspace(static_cast<unsigned char>(c))) { ++i; continue; }
            size_t e = i;
            if (c == '"') {
                e = line.find('"', i + 1);
                t.push_back(line.substr(i + 1, (e == std::string::npos ? line.size() : e) - i - 1));
                i = e == std::string::npos ? line.size() : e + 1;
                continue;
            }
            while (e < line.size() && !std::isspace(static_cast<unsigned char>(line[e]))) ++e;
            t.push_back(line.substr(i, e - i));
            i = e;
        }
        if (t.empty()) continue;
        if (t[0] == "RemoveOldVehicleLiveries") {  // [port] the hubs' lists empty; their POI records go as well
            out.erase(std::remove_if(out.begin(), out.end(), [](const PoiPoint& p) { return p.type == 2; }), out.end());
            continue;
        }
        if (t[0] != "AddVehicleLivery" || t.size() < 9) continue;
        std::string kind = t[5];
        for (char& ch : kind) ch = char(std::toupper(static_cast<unsigned char>(ch)));
        if (kind != "POI") continue;
        PoiPoint p;
        p.type = 2;
        p.x = float(std::atof(t[7].c_str())), p.z = float(std::atof(t[8].c_str()));
        p.name = t[1];
        out.push_back(std::move(p));
    }
    for (PoiPoint& p : parsePoiScript(poiText)) out.push_back(std::move(p));
    poiExchangeSort(out);
    return out;
}

void PoiGrid::build(const std::vector<std::array<float, 2>>& xz) {
    const float inv = 1.0f / kCell;  // +0x14 / +0x18
    w_ = int(std::nearbyint((kMax - kMin) * inv)) + 1, h_ = w_;  // fistp (round to nearest) + 1
    cells_.assign(size_t(w_ * h_), {});
    for (size_t i = 0; i < xz.size(); ++i) {
        const int cx = int((xz[i][0] - kMin) * inv), cz = int((xz[i][1] - kMin) * inv);  // ftol 0x8e01d0
        if (cx >= 0 && cx < w_ && cz >= 0 && cz < h_) cells_[size_t(cz * w_ + cx)].push_back(int(i));
    }
}

void PoiGrid::query(float x, float z, float r, const std::function<bool(int)>& fn) const {
    if (cells_.empty()) return;
    const float inv = 1.0f / kCell;
    auto clampI = [](int v, int n) { return v < 0 ? 0 : v > n - 1 ? n - 1 : v; };
    const int x0 = clampI(int((x - r - kMin) * inv), w_), z0 = clampI(int((z - r - kMin) * inv), h_);
    const int x1 = clampI(int((x + r - kMin) * inv), w_), z1 = clampI(int((z + r - kMin) * inv), h_);
    for (int cz = z0; cz <= z1; ++cz)
        for (int cx = x0; cx <= x1; ++cx) {
            const auto& c = cells_[size_t(cz * w_ + cx)];
            for (size_t k = c.size(); k-- > 0;)
                if (!fn(c[k])) return;
        }
}

void poiMarkers(const std::vector<PoiPoint>& pois, float px, float pz, std::vector<RaceMarker>& out) {
    for (size_t i = 0; i < pois.size(); ++i) {
        const PoiPoint& p = pois[i];
        const float d2 = (p.z - pz) * (p.z - pz) + (p.x - px) * (p.x - px);
        if (d2 > 1e6f) continue;  // 0x4ed41f: only within 1000 m (the equality counts as inside)
        if (p.type == 2 || p.type < 1 || p.type > 5) continue;  // [port] liveries: the painted vehicle is not drawn
        RaceMarker m;
        m.type = 16;  // +0x64 = 0x10
        m.index = int(i);
        m.pos = {p.x, 0, p.z};
        m.text = p.name.substr(0, 31);  // strncpy(+0x44, name, 0x1f)
        m.cls = poiMarkerClass(p.type);
        m.upright = p.type == 3;  // +0x6f = 1 only for the vista (0x4ed617); the rest clear it (0x4ed68e)
        m.lift = 0;               // [I] 0x4ed2d0 never sets +0x38
        out.push_back(std::move(m));
    }
}

float loadMarkerMinHalf(const std::string& pp001Path, float fallback) {
    std::ifstream f(pp001Path, std::ios::binary);
    std::string line;
    while (std::getline(f, line)) {
        std::istringstream ls(line);
        std::string tag;
        int n = -1;
        float v = 0;
        if ((ls >> tag >> n >> v) && tag == "PP" && n == 360) return v;  // g_pp001_params[0x168]
    }
    return fallback;
}

}  // namespace fuel
