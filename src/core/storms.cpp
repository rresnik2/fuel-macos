// P61: FUEL's tornadoes (see storms.hpp; notes ~/re-work/fuel/notes/p61-storms.md).
#include "storms.hpp"

#include "assembly.hpp"
#include "resources.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <sstream>
#include <strings.h>

namespace fuel::storm {

namespace {
constexpr uint32_t kNodeClass = 0x85D02528;
constexpr double kTwoPi = 6.283185307179586;

const DpcFile::Entry* findIn(const std::vector<const DpcFile*>& lookup, uint32_t id, const DpcFile** file) {
    for (const DpcFile* f : lookup)
        if (const DpcFile::Entry* e = f->find(id); e != nullptr && e->header != nullptr) {
            *file = f;
            return e;
        }
    return nullptr;
}

uint32_t linkName(const DpcFile& f, const DpcFile::Entry& e) {
    const auto l = f.link(e);
    uint32_t n = 0;
    if (l.size() >= 4) std::memcpy(&n, l.data(), 4);
    return n;
}

std::vector<std::string> userDefineLines(const std::vector<const DpcFile*>& lookup, uint32_t id) {
    if (id == 0) return {};
    const DpcFile* f = nullptr;
    const DpcFile::Entry* e = findIn(lookup, id, &f);
    if (e == nullptr) return {};
    const auto b = f->body(*e);
    if (!b) return {};
    try {
        return parseUserDefineBody(*b);
    } catch (const std::exception&) {
        return {};
    }
}

// Node_UserDefineKeyword (0x53f710): the token after `key` on a line starting with it.
bool keyword(const std::vector<std::string>& lines, const char* key, std::string& value) {
    for (const std::string& l : lines) {
        std::istringstream in(l);
        std::string w;
        while (in >> w)
            if (strcasecmp(w.c_str(), key) == 0) {
                value.clear();
                in >> value;
                return true;
            }
    }
    return false;
}

double hdist2(const V3& a, const V3& b) { return (a[0] - b[0]) * (a[0] - b[0]) + (a[2] - b[2]) * (a[2] - b[2]); }
}  // namespace

bool EventVolume::touches(const V3& p, double r) const {
    const double d[3] = {p[0] - centre[0], p[1] - centre[1], p[2] - centre[2]};
    double q = 0;
    for (size_t i = 0; i < 3; ++i) {
        const double l = d[0] * axis[i][0] + d[1] * axis[i][1] + d[2] * axis[i][2];
        const double c = std::clamp(l, -half[i], half[i]);
        q += (l - c) * (l - c);
    }
    return q <= r * r;
}

bool Storms::inEventVolume(const V3& p, double r) const {
    for (const EventVolume& v : data_.eventVolumes)
        if (v.touches(p, r)) return true;
    return false;
}

bool weatherHasTornado(const std::string& weather) {
    std::string u = weather;
    for (char& c : u) c = char(std::toupper(static_cast<unsigned char>(c)));
    return u.find("TOR") != std::string::npos;  // 0x5759c0: strstr(_strupr(s), "TOR") -> +0 = 1
}

StormData loadStormData(const DpcFile& usa1, const std::vector<const DpcFile*>& lookup) {
    StormData d;
    constexpr uint32_t kCollisionVolClass = 0x8EF49632;
    // The two volumes' node (and Lod) name in USA1.DPC: the only nodes called this, both CollisionVol_Z boxes beside the CR21
    // valley tornado (p61); the string is not in FUEL.exe, so the CRC is the data's.
    constexpr uint32_t kEventVol = 0xAFD84CC3;
    const uint32_t kTornade = asoboCrc("Tornade"), kPath = asoboCrc("IT_TornadoPath"), kSmall = asoboCrc("SmallTornadoAgent_G");
    auto nodeOf = [&](uint32_t id, Node& out) -> bool {
        const DpcFile::Entry* e = usa1.find(id);
        if (e == nullptr || e->classCrc != kNodeClass) return false;
        const auto b = usa1.body(*e);
        if (!b) return false;
        try {
            out = parseNodeBody(*b);
        } catch (const std::exception&) {
            return false;
        }
        return true;
    };
    for (uint32_t id : usa1.ids(kNodeClass)) {
        Node n;
        if (!nodeOf(id, n)) continue;
        const uint32_t name = linkName(usa1, *usa1.find(id));
        // The agent class comes from the node name (clone classes, "Tornade") or the object's name (Lod_Z link +0).
        uint32_t objName = 0;
        if (n.object) {
            const DpcFile* of = nullptr;
            if (const DpcFile::Entry* oe = findIn(lookup, n.object, &of)) objName = linkName(*of, *oe);
        }
        if (objName == kSmall) d.hasSmall = true;
        if (name == kEventVol || objName == kEventVol) {  // PK06: 0x57ab40 (Obb_PlaceByNode on box 0 of the node's CollisionVol_Z)
            const DpcFile* bf = nullptr;
            const DpcFile::Entry* be = n.object ? findIn(lookup, n.object, &bf) : nullptr;
            uint32_t cnt = 0;
            auto bb = be != nullptr && be->classCrc == kCollisionVolClass ? bf->body(*be) : decltype(bf->body(*be)){};
            if (bb && bb->size() >= 4 + 64) std::memcpy(&cnt, bb->data(), 4);
            if (cnt != 0) {
                float b[16];
                std::memcpy(b, bb->data() + 4, 64);
                float m[16];  // b x world (row vectors)
                for (int r = 0; r < 4; ++r)
                    for (int c = 0; c < 4; ++c) {
                        float s = 0;
                        for (int k = 0; k < 4; ++k) s += b[r * 4 + k] * n.world[size_t(k * 4 + c)];
                        m[r * 4 + c] = s;
                    }
                EventVolume v;
                v.node = id;
                for (int r = 0; r < 3; ++r) {
                    const float len = std::sqrt(m[r * 4] * m[r * 4] + m[r * 4 + 1] * m[r * 4 + 1] + m[r * 4 + 2] * m[r * 4 + 2]);
                    v.half[size_t(r)] = len;
                    for (int c = 0; c < 3; ++c) v.axis[size_t(r)][size_t(c)] = len > 0 ? m[r * 4 + c] / len : float(r == c);
                }
                v.centre = {m[12], m[13], m[14]};
                d.eventVolumes.push_back(v);
            }
        }
        if (name == kTornade) {
            d.mothers.push_back(id);
            d.motherPos.push_back({n.world[12], n.world[13], n.world[14]});
        }
        if (objName != kPath) continue;
        if (n.parent == 0) continue;  // the library copy under no parent (dc348ea5)
        TornadoPath p;
        p.node = id, p.parent = n.parent;
        p.pos = {n.world[12], n.world[13], n.world[14]};
        const auto lines = userDefineLines(lookup, n.userDefine);
        std::string v;
        if (keyword(lines, "PLAYERDIST", v)) p.playerDist = float(std::atof(v.c_str()));
        if (keyword(lines, "SPEED", v)) p.speed = float(std::atof(v.c_str()));
        if (keyword(lines, "RESET_M_START", v)) p.resetMission = v;
        for (uint32_t c = n.child; c != 0 && p.points.size() < 256;) {  // 0x518c30: +0xa4, then +0xac
            Node cn;
            if (!nodeOf(c, cn)) break;
            PathPoint pt;
            pt.p = {cn.world[12], cn.world[13], cn.world[14]};
            if (keyword(userDefineLines(lookup, cn.userDefine), "SPEED", v)) pt.speed = float(std::atof(v.c_str()));
            p.points.push_back(pt);
            c = cn.prev;
        }
        d.paths.push_back(std::move(p));
    }
    return d;
}

bool vortexForce(const ForceVolume& v, const V3& p, float bodyRadius, std::array<float, 3>& force) {
    const double dx = p[0] - v.centre[0], dz = p[2] - v.centre[2];
    const double reach = double(v.radius) + bodyRadius;
    if (!(dx * dx + dz * dz < reach * reach)) return false;
    const float dy = float(p[1] - v.centre[1]);
    float k = 1.0f - std::fabs(dy * 0.25f) / v.radius;
    k = std::clamp(k, 0.0f, 1.0f);
    const double h = std::sqrt(dx * dx + dz * dz);  // Vec_FlattenNormalize
    const float nx = h > 0 ? float(dx / h) : 0.0f, nz = h > 0 ? float(dz / h) : 0.0f;
    const float kk = 1.0f - k * 1.2f;
    // ((up x n) * 0.25 + kk n + up * 0.15) * strength, up = (0, 1, 0) (0xa06320)
    force = {(-nz * 0.25f + kk * nx) * v.strength, 0.15f * v.strength, (nx * 0.25f + kk * nz) * v.strength};
    return true;
}

Storms::Storms(StormData data, World world) : data_(std::move(data)), world_(std::move(world)) {
    for (size_t i = 0; i < data_.mothers.size(); ++i) {  // MotherTornadoAgent_G vf24 0x5bfcc0 registers, shown
        Agent a;
        a.radius = 50.0f;  // vt[105] 0x4d5670
        a.pos = data_.motherPos[i];
        a.shown = true;
        agents_.push_back(a);
    }
    if (data_.hasSmall) {  // SmallTornadoAgent_G vf24 0x5bf3b0 registers; vf14 0x5bf530 -> 0x5bf560 hides it
        Agent a;
        a.small = true;
        a.radius = 20.0f;  // vt[105] 0x4d54d0
        a.shown = false;
        small_ = int(agents_.size());
        agents_.push_back(a);
    }
    paths_ = data_.paths;
    pathState_.assign(paths_.size(), {});
}

void Storms::startRace(bool tornado, const std::array<float, 3>& start, const std::array<float, 3>& startRot,
                       const std::array<float, 3>& finish) {
    if (!tornado) return;
    // 0x631bd0 (mission +0x18c): d = normalize(finish - start); p = (d.z, 0, -d.x) normalised; f1 / f2 = the start
    // rotation (0x644980: rotation vector -> quaternion) applied to x / z (0x410510), normalised; p flips when
    // f1 . d < 0. A = start + 300 f2 + 100 p, B = finish + 300 d + 100 p (0x6313b0 appends {A, B} to race +0x110).
    double d[3] = {double(finish[0]) - start[0], double(finish[1]) - start[1], double(finish[2]) - start[2]};
    double l = std::sqrt(d[0] * d[0] + d[1] * d[1] + d[2] * d[2]);
    if (l < 1e-6) return;
    for (double& c : d) c /= l;
    double p[3] = {d[2], 0, -d[0]};
    l = std::sqrt(p[0] * p[0] + p[2] * p[2]);
    if (l > 1e-9) p[0] /= l, p[2] /= l;
    auto rotate = [&](const double v[3], double out[3]) {
        const double a = std::sqrt(double(startRot[0]) * startRot[0] + double(startRot[1]) * startRot[1] + double(startRot[2]) * startRot[2]);
        if (a < 1e-9) {
            out[0] = v[0], out[1] = v[1], out[2] = v[2];
            return;
        }
        const double s = std::sin(a * 0.5) / a, qx = startRot[0] * s, qy = startRot[1] * s, qz = startRot[2] * s, qw = std::cos(a * 0.5);
        // v + 2 q.xyz x (q.xyz x v + w v)
        const double tx = qy * v[2] - qz * v[1] + qw * v[0], ty = qz * v[0] - qx * v[2] + qw * v[1], tz = qx * v[1] - qy * v[0] + qw * v[2];
        out[0] = v[0] + 2 * (qy * tz - qz * ty), out[1] = v[1] + 2 * (qz * tx - qx * tz), out[2] = v[2] + 2 * (qx * ty - qy * tx);
        const double n = std::sqrt(out[0] * out[0] + out[1] * out[1] + out[2] * out[2]);
        if (n > 1e-9) out[0] /= n, out[1] /= n, out[2] /= n;
    };
    const double X[3] = {1, 0, 0}, Z[3] = {0, 0, 1};
    double f1[3], f2[3];
    rotate(X, f1);
    rotate(Z, f2);
    if (f1[0] * d[0] + f1[1] * d[1] + f1[2] * d[2] < 0) p[0] = -p[0], p[1] = -p[1], p[2] = -p[2];
    const V3 A = {start[0] + 300 * f2[0] + 100 * p[0], start[1] + 300 * f2[1] + 100 * p[1], start[2] + 300 * f2[2] + 100 * p[2]};
    const V3 B = {finish[0] + 300 * d[0] + 100 * p[0], finish[1] + 300 * d[1] + 100 * p[1], finish[2] + 300 * d[2] + 100 * p[2]};
    // [I] The race script hands the route to a mother tornado (race property getters 0x4fde50.. expose +0x110 /
    // +0x119 to the scripts; the script is not decoded): the mother nearest the start, placed at A, travelling to B
    // and back (TornadoManipulator_G +0x40c / +0x418 targets, +0x425 ping-pong, 25 m/s in 0x5be880).
    int best = -1;
    double bd = 1e300;
    for (size_t i = 0; i < agents_.size(); ++i) {
        if (agents_[i].small) continue;
        double m = 1e300;  // the mother's group: distance from the start to its nearest path node
        for (const TornadoPath& tp : paths_)
            if (i < data_.mothers.size() && tp.parent == data_.mothers[i])
                m = std::min(m, hdist2(tp.pos, {start[0], start[1], start[2]}));
        if (m < bd) bd = m, best = int(i);
    }
    if (best < 0) return;
    Agent& a = agents_[size_t(best)];
    a.targetA = A, a.targetB = B;
    a.pos = A;
    a.moving = true, a.pingPong = true, a.toFirst = false;
    a.fading = false;
    a.age = 0;
}

void Storms::endRace() {
    for (size_t i = 0; i < agents_.size(); ++i) {
        Agent& a = agents_[i];
        if (a.small) {
            hide(a);
            continue;
        }
        a.moving = false;
        a.fading = false;
        if (i < data_.motherPos.size()) a.pos = data_.motherPos[i];
    }
}

// ---- PK14: 0x5b96a0 (see storms.hpp).
void Storms::killMission() {
    for (size_t i = 0; i < agents_.size(); ++i) {
        Agent& a = agents_[i];
        a.moving = a.pingPong = a.toFirst = false;  // the route went with the agent (+0x424..+0x426 of a new one: 0)
        a.fading = a.frozen = false;                // 0x4f1b60 hides the node directly: no state-1 fade
        a.path.clear();
        a.index = 0;
        a.debrisTimer = 0;                          // vf00 of the next spawn
        a.age = 0;
        if (a.small) a.shown = false;               // node 0x40000 (leaves the 0x5bc860 list)
        else if (i < data_.motherPos.size()) a.pos = data_.motherPos[i];
    }
    volumes_.clear(), breaks_.clear(), animTriggers_.clear(), throws_.clear();  // nothing left to make them this frame
}
// ---- end PK14

void Storms::missionStart(const std::string& mission) {
    for (size_t i = 0; i < paths_.size(); ++i)
        if (!paths_[i].resetMission.empty() && strcasecmp(paths_[i].resetMission.c_str(), mission.c_str()) == 0)
            pathState_[i].linked = false;  // 0x513d70 -> mover vt+0x38 (vf14 0x518bb0 releases the +0x380 handle)
}

bool Storms::fadeOut(Agent& a) {  // 0x5be290: look state 1; alpha > 0 -> not done (0), else the emitters stop and vf103 stops the sounds
    a.fading = true;
    const int id = int(&a - agents_.data());
    return !(lookAlpha_ && lookAlpha_(id) > 0.0f);
}

void Storms::hide(Agent& a) {  // 0x5bf560: the fade-out first; once it is over node +0x2c |= 0x40000 (leaves the list), +0x42c = 1
    if (!fadeOut(a)) return;
    a.shown = false;
    a.fading = false;
    a.path.clear();
    a.index = 0;
}

void Storms::fadeOutRace() {  // the weather's generator tornado is gone: 0x5b89e0 calls 0x5be290 on the mother
    for (Agent& a : agents_)
        if (!a.small && a.moving) a.fading = true;
}

void Storms::wander(Agent& a, float dt, uint64_t clockMs) {  // 0x5be880
    // Rotation about y (0xa06320) by -(clock % 250000) x 4e-6 x 2 pi.
    a.spin = float(-double(clockMs % 250000ull) * 4e-6 * kTwoPi);
    if (a.moving) {  // +0x424
        const V3& t = a.toFirst ? a.targetA : a.targetB;  // +0x426 ? +0x40c : +0x418
        double dx = t[0] - a.pos[0], dz = t[2] - a.pos[2];
        const double h = std::sqrt(dx * dx + dz * dz);
        if (a.pingPong && h < 10.0) a.toFirst = !a.toFirst;  // +0x425: within 10 m (0x996494) turn round
        if (h > 0) dx /= h, dz /= h;                         // Vec_FlattenNormalize
        a.pos[0] += dx * dt * 25.0, a.pos[2] += dz * dt * 25.0;  // 0x996184 = 25
    }
    // 0x5be320 (every frame in FUEL); [port] only while the mother travels: the idle ones sit at their group node
    // (the world origin, outside the terrain).
    if (a.moving && world_.ground) a.pos[1] = world_.ground(a.pos[0], a.pos[2]);
}

void Storms::smallStep(Agent& a, float dt) {  // 0x5bf5f0, path part
    if (a.path.empty()) return;
    if (a.index >= int(a.path.size())) {
        hide(a);
        return;
    }
    const PathPoint& pt = a.path[size_t(a.index)];
    const float sp = pt.speed < 0 ? a.pathSpeed : pt.speed;  // point +0xc < 0 -> +0x408
    double dx = pt.p[0] - a.pos[0], dz = pt.p[2] - a.pos[2];
    const double h = std::sqrt(dx * dx + dz * dz);
    if (h < 10.0) ++a.index;
    if (h > 0) dx /= h, dz /= h;
    a.pos[0] += dx * sp * dt, a.pos[2] += dz * sp * dt;
}

float Storms::rand01() {
    rng_ = uint32_t((uint64_t(rng_) * 48271u) % 2147483647u);
    return float(rng_ - 1) / 2147483646.0f;
}

namespace {
struct DebrisItem {
    const char* name;
    int mode;
};
// 0x5bd150 (forest density > 0.5) and 0x5bd2d0 (else): k = floor(rand x 10.9) (RC_DOWN), the AsoboCrc of the name
// and the return value; 0x5bd0c0 (in a TornadoEventVolume): IT_Arbre01..06 by floor(rand x 5.9), all 1.
constexpr DebrisItem kForestDebris[11] = {{"IT_DYN_PierreG", 1}, {"IT_DYN_PierreM", 0}, {"IT_DYN_PierreP", 0}, {"IT_Arbre01", 1},
                                          {"IT_Arbre02", 1},     {"IT_Arbre03", 1},     {"IT_Arbre04", 1},     {"IT_Arbre05", 1},
                                          {"IT_Arbre06", 1},     {"IT_DYN_VoitureThrow", 2}, {"RemorqueCheval02", 3}};
constexpr DebrisItem kOpenDebris[11] = {{"IT_DYN_PierreG", 1},     {"IT_DYN_PierreM", 0},     {"IT_DYN_PierreP", 0},
                                        {"IT_DYN_PlaqueBoisG", 1}, {"IT_DYN_PlaqueBoisM", 0}, {"IT_DYN_PlaqueBoisP", 0},
                                        {"IT_DYN_BarreBoisG", 1},  {"IT_DYN_BarreBoisM", 0},  {"IT_DYN_BarreBoisP", 0},
                                        {"IT_DYN_VoitureThrow", 2}, {"RemorqueCheval02", 3}};
constexpr const char* kTrees[6] = {"IT_Arbre01", "IT_Arbre02", "IT_Arbre03", "IT_Arbre04", "IT_Arbre05", "IT_Arbre06"};
using F3 = std::array<float, 3>;
F3 fnorm(F3 v) {
    const float l = std::sqrt(v[0] * v[0] + v[1] * v[1] + v[2] * v[2]);
    return l > 0 ? F3{v[0] / l, v[1] / l, v[2] / l} : v;
}
F3 flatNorm(F3 v) {  // Vec_FlattenNormalize 0x40fc80
    v[1] = 0;
    return fnorm(v);
}
F3 fcross(const F3& a, const F3& b) { return {a[1] * b[2] - a[2] * b[1], a[2] * b[0] - a[0] * b[2], a[0] * b[1] - a[1] * b[0]}; }
}  // namespace

void Storms::debris(Agent& a, float dt, double camDist2d, const V3& camera) {
    if (!(camDist2d < 1000.0)) return;  // vf18: the timer runs within 1000 m of the camera
    a.debrisTimer -= dt;
    if (a.debrisTimer > 0.0f) return;
    // 0x5bd590 (T = the tornado, r = its radius vt+0x1a4)
    a.debrisTimer = rand01() + 1.0f;
    const V3& T = a.pos;
    const double cx = camera[0] - T[0], cy = camera[1] - T[1], cz = camera[2] - T[2];
    const double dist = std::sqrt(cx * cx + cy * cy + cz * cz);
    if (!(dist <= 500.0)) return;  // 0x9dbfc4
    DebrisThrow t;
    t.near = dist < 250.0;  // 0x9dc3a4
    // 0x5bd430: the item.
    if (inEventVolume(T) || (world_.inEventVolume && world_.inEventVolume(T))) {  // 0x57acf0 (sphere 20 m)
        t.item = kTrees[std::min(5, int(std::floor(rand01() * 5.9f)))], t.mode = 1;
    } else {
        const DebrisItem* table = world_.forest && world_.forest(T[0], T[2]) > 0.5f ? kForestDebris : kOpenDebris;
        const DebrisItem& it = table[std::min(10, int(std::floor(rand01() * 10.9f)))];
        t.item = it.name, t.mode = it.mode;
    }
    const F3 up = {0, 1, 0}, down = {0, -1, 0};  // 0xa06320 / 0xa0632c
    const F3 P = {float(player_[0] - T[0]), float(player_[1] - T[1]), float(player_[2] - T[2])};  // player, tornado-relative
    F3 pos{}, vel{};
    if (t.mode == 1) {  // at the player: from 10 m out, 20 m/s, up a tenth of the horizontal distance, +-2.5 m aside
        const float h = std::sqrt(P[0] * P[0] + P[2] * P[2]);
        const F3 n = fnorm({P[0], h * 0.1f, P[2]});
        vel = {n[0] * 20.0f, n[1] * 20.0f, n[2] * 20.0f};
        pos = {n[0] * 10.0f, n[1] * 10.0f, n[2] * 10.0f};
        const F3 L = flatNorm(fcross(up, vel));
        const float r = (rand01() - 0.5f) * 5.0f;
        pos = {pos[0] + L[0] * r, pos[1] + L[1] * r, pos[2] + L[2] * r};
    } else if (t.mode == 2) {  // ahead of the player: 2 s of its velocity + 150 m along its forward, 50 m up; down at 50 and back at 75 m/s
        const F3& V = playerVel_;
        const F3& Fw = playerFwd_;
        pos = {P[0] + V[0] * 2.0f + Fw[0] * 150.0f + up[0] * 50.0f, P[1] + V[1] * 2.0f + Fw[1] * 150.0f + up[1] * 50.0f,
               P[2] + V[2] * 2.0f + Fw[2] * 150.0f + up[2] * 50.0f};
        vel = {down[0] * 50.0f - Fw[0] * 75.0f, down[1] * 50.0f - Fw[1] * 75.0f, down[2] * 50.0f - Fw[2] * 75.0f};
    } else if (t.mode == 3) {  // from the side: 2 s ahead + 50 m forward + 50 m aside, 20 m up; across at 50 and down at 15 m/s
        const F3& V = playerVel_;
        const F3& Fw = playerFwd_;
        const F3 L = flatNorm(fcross(Fw, up));
        for (int k = 0; k < 3; ++k) {
            pos[size_t(k)] = P[size_t(k)] + V[size_t(k)] * 2.0f + Fw[size_t(k)] * 50.0f + L[size_t(k)] * 50.0f + up[size_t(k)] * 20.0f;
            vel[size_t(k)] = -L[size_t(k)] * 50.0f + down[size_t(k)] * 15.0f;
        }
    } else {  // 0: lobbed up beside the tornado towards the player, 2 m up and r out
        const F3 d = flatNorm(P);
        const F3 L = fcross(d, up);
        const float r1 = rand01() - 0.5f, r2 = rand01();
        const F3 dir = {d[0] * r2 + L[0] * r1, d[1] * r2 + L[1] * r1, d[2] * r2 + L[2] * r1};
        const float r3 = rand01() * 0.75f + 0.25f;
        vel = {(up[0] * r3 * 4.0f + dir[0]) * 5.0f, (up[1] * r3 * 4.0f + dir[1]) * 5.0f, (up[2] * r3 * 4.0f + dir[2]) * 5.0f};
        const F3 nd = fnorm(dir);
        pos = {up[0] * 2.0f + nd[0] * a.radius, up[1] * 2.0f + nd[1] * a.radius, up[2] * 2.0f + nd[2] * a.radius};
    }
    t.pos = {T[0] + pos[0], T[1] + pos[1], T[2] + pos[2]};
    t.vel = vel;
    throws_.push_back(t);
}

void Storms::update(float dt, const V3& camera, uint64_t clockMs) {
    volumes_.clear();
    animTriggers_.clear();
    throws_.clear();
    breaks_.clear();
    // IT_TornadoPathMove vt[17] (0x518dc0), per path.
    if (small_ >= 0)
        for (size_t i = 0; i < paths_.size(); ++i) {
            const TornadoPath& tp = paths_[i];
            const double dx = camera[0] - tp.pos[0], dy = camera[1] - tp.pos[1], dz = camera[2] - tp.pos[2];
            if (!(std::sqrt(dx * dx + dy * dy + dz * dz) <= double(tp.playerDist) + tp.playerDist)) continue;
            float r = 0;
            if (!(nearestDistance(tp.pos[0], tp.pos[2], &r) <= 5000.0f)) continue;
            Agent& s = agents_[size_t(small_)];
            bool go;
            if (!pathState_[i].linked) {  // build the path once (0x518c30), find SmallTornadoAgent_G by name
                pathState_[i].linked = true;
                go = true;
            } else {
                go = !s.shown;  // only while the small tornado is hidden
            }
            if (!go) continue;
            s.shown = true;  // agent +0x3c |= 0x20000, &= ~0x40000; node shown (0x65ede0)
            s.fading = false;
            s.age = 0;
            s.pos = tp.points.empty() ? tp.pos : tp.points.front().p;  // mover vt+0xd4
            s.path = tp.points;                                       // 0x5190c0
            s.index = 0;
            s.pathSpeed = tp.speed;                                   // mover +0x408 = path +0x88
            if (world_.ground) s.pos[1] = world_.ground(s.pos[0], s.pos[2]);
        }
    for (Agent& a : agents_) {
        if (a.small) {
            a.frozen = false;
            if (!a.shown) continue;  // +0x42c
            a.age += dt;
            if (nearestOther(a.pos[0], a.pos[2]) > 5000.0f) {  // 0x5bcc20
                hide(a);        // 0x5bf560, then the vf18 returns: no move, no look update (the fade does not advance)
                a.frozen = a.shown;
                continue;
            }
            a.spin = float(-double(clockMs % 250000ull) * 4e-6 * kTwoPi);
            smallStep(a, dt);
            if (!a.shown) continue;
            if (world_.ground) a.pos[1] = world_.ground(a.pos[0], a.pos[2]);
            const double d = std::sqrt(hdist2(a.pos, camera));
            if (d < 2000.0) {
                volumes_.push_back({a.pos, 100000.0f, a.radius, 0x209});  // 0x6810a0(world, 100000, r)
                animTriggers_.push_back(a.pos);                           // 0x5181f0
                if (d < 200.0) breaks_.push_back({a.pos, a.radius});      // PK06: 0x516570 (the camera < 200 m)
                debris(a, dt, d, camera);                                 // P65: +0x38c timer, 0x5bd590
            }
        } else {
            a.age += dt;
            wander(a, dt, clockMs);
            const double d = std::sqrt(hdist2(a.pos, camera));
            if (d < 2000.0) {
                volumes_.push_back({a.pos, 30000.0f, a.radius, 0x209});  // 0x6810a0(world, 30000, r)
                if (d < 200.0) breaks_.push_back({a.pos, a.radius});     // PK06: 0x516570 (the camera < 200 m)
                debris(a, dt, d, camera);                                // P65: +0x388 timer, 0x5bd590
            }
            if (a.fading && a.moving && !(lookAlpha_ && lookAlpha_(int(&a - agents_.data())) > 0.0f)) {
                a.moving = false;  // the fade is over: the emitters stop, vf103 stops 0x7e / 0x80, the node is hidden
                a.fading = false;
                const size_t i = size_t(&a - agents_.data());
                if (i < data_.motherPos.size()) a.pos = data_.motherPos[i];
            }
        }
    }
}

int Storms::list(V3* pos, float* radius, int max) const {
    int n = 0;
    for (const Agent& a : agents_) {
        if (!a.shown) continue;
        if (pos) pos[n] = a.pos;
        if (radius) radius[n] = a.radius;
        if (++n == max) break;
    }
    return n;
}

float Storms::nearestDistance(double x, double z, float* radius) const {
    double best = 1e15;
    if (radius) *radius = 0;
    for (const Agent& a : agents_) {
        if (!a.shown) continue;
        const double d = (z - a.pos[2]) * (z - a.pos[2]) + (x - a.pos[0]) * (x - a.pos[0]);
        if (d < best) {
            best = d;
            if (radius) *radius = a.radius;
        }
    }
    return float(std::sqrt(best));
}

bool Storms::nearestPosition(double x, double z, V3& out) const {
    double best = 1e15;
    bool any = false;
    for (const Agent& a : agents_) {
        if (!a.shown) continue;
        const double d = (z - a.pos[2]) * (z - a.pos[2]) + (x - a.pos[0]) * (x - a.pos[0]);
        if (d < best) best = d, out = a.pos, any = true;
    }
    return any;
}

float Storms::nearestOther(double x, double z) const {
    double best = 1e15;
    for (const Agent& a : agents_) {
        if (!a.shown) continue;
        const double d = (z - a.pos[2]) * (z - a.pos[2]) + (x - a.pos[0]) * (x - a.pos[0]);
        if (1.0 < d && d <= best) best = d;
    }
    return float(std::sqrt(best));
}

std::array<float, 3> Storms::forceAt(const V3& p, float bodyRadius, const std::array<float, 3>& bodyVel, float scale) const {
    std::array<float, 3> f{0, 0, 0}, g;
    bool any = false;
    float best = 0;  // param_7: |F| of the kept volume
    for (const ForceVolume& v : volumes_) {
        if (v.flags & 0x200) {  // the vortex branch writes the force of each volume it is in
            if (vortexForce(v, p, bodyRadius, g)) f = g, best = std::sqrt(g[0] * g[0] + g[1] * g[1] + g[2] * g[2]), any = true;
            continue;
        }
        float m = 0;
        if (!radialForce(v, p, bodyRadius, bodyVel, scale, g, m)) continue;
        if (any && m <= best) continue;  // local_e5: a weaker radial volume does not replace the kept force
        f = g, best = m, any = true;
    }
    return f;
}

bool radialForce(const ForceVolume& v, const V3& p, float bodyRadius, const std::array<float, 3>& bodyVel, float scale,
                 std::array<float, 3>& force, float& magnitude) {
    const double dx = p[0] - v.centre[0], dy = p[1] - v.centre[1], dz = p[2] - v.centre[2];
    const double reach = double(v.radius) + bodyRadius;
    const double d2 = dx * dx + dy * dy + dz * dz;
    if (!(d2 < reach * reach)) return false;
    const float d = float(std::sqrt(d2));
    std::array<float, 3> n = {0, 0, 0};  // 0xa67728: (body - centre) / |..| (left at the raw difference when 0)
    if (d != 0.0f) n = {float(dx) / d, float(dy) / d, float(dz) / d};
    const float inv = 1.0f / v.radius;  // +0x24
    float k;
    if (0.0f <= v.strength) {
        float x = (d - bodyRadius) - 1.0f;
        if (x < 0.0f) x = 0.001f;
        k = 1.0f - x * inv * x * inv;
    } else {  // the magnet
        float q = d < 0.0f ? 0.001f : d;
        q *= inv;
        k = 0.0f;
        if (q < 1.0f) k = std::sqrt(q) * std::sqrt(1.0f - q) * 2.0f;
        const float sp = std::sqrt(bodyVel[0] * bodyVel[0] + bodyVel[1] * bodyVel[1] + bodyVel[2] * bodyVel[2]);
        if (0.01f < sp) {  // body +0x820 velocity; 0x530b40 [I: true]
            const float c = bodyVel[2] / sp * n[2] + 0.0f + bodyVel[0] / sp * n[0];
            if (c > 0.0f) k = c * 30.0f + k;
        }
    }
    if (k < 0.01f) k = 0.01f;
    float F = k * v.strength;
    if (v.flags & 4) F *= scale;
    if (v.flags & 1) {
        force = {v.dir[0] * F, v.dir[1] * F, v.dir[2] * F};
    } else {
        if (v.flags & 0x40) {
            const float ex = float(dx), ey = float(dy) + float(dy) + 1.0f, ez = float(dz);
            const float l = std::sqrt(ex * ex + ey * ey + ez * ez);
            if (l != 0.0f) n = {ex / l, ey / l, ez / l};
        }
        force = {n[0] * F, n[1] * F, n[2] * F};
        if (v.flags & 4) force[1] = 0.0f;
    }
    magnitude = std::fabs(F);
    return true;
}

std::vector<Storms::Visible> Storms::visibles() const {
    std::vector<Visible> out;
    for (size_t i = 0; i < agents_.size(); ++i) {
        const Agent& a = agents_[i];
        if (!a.shown) continue;
        if (!a.small && !a.moving) continue;  // the idle mothers sit at their group node (the world origin)
        out.push_back({a.pos, a.spin, a.small, a.age, int(i), a.fading, a.frozen});
    }
    return out;
}

bool Storms::anyActive() const {
    for (const Agent& a : agents_)
        if (a.shown && (a.small || a.moving)) return true;
    return false;
}

}  // namespace fuel::storm
