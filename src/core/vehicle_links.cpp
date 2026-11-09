// PK22: FUEL's vehicle visual links; see vehicle_links.hpp and notes pk22.md.
#include "core/vehicle_links.hpp"

#include "core/resources.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstring>

namespace fuel::vlinks {

namespace {

bool iequal(const std::string& a, const char* b) {
    const size_t n = std::strlen(b);
    if (a.size() != n) return false;
    for (size_t i = 0; i < n; ++i)
        if (std::toupper(static_cast<unsigned char>(a[i])) != std::toupper(static_cast<unsigned char>(b[i]))) return false;
    return true;
}
bool istarts(const std::string& a, const char* b) {
    const size_t n = std::strlen(b);
    if (a.size() < n) return false;
    for (size_t i = 0; i < n; ++i)
        if (std::toupper(static_cast<unsigned char>(a[i])) != std::toupper(static_cast<unsigned char>(b[i]))) return false;
    return true;
}
// CRT strtod as 0x8b5bc0 is used: the value and whether the whole field was read (and it was not empty).
bool fullFloat(const std::string& s, float& out) {
    if (s.empty()) return false;
    char* end = nullptr;
    const double v = std::strtod(s.c_str(), &end);
    if (end == nullptr || *end != '\0') return false;
    out = float(v);
    return true;
}
// 0x696e00 with ';' / '+': the fields of a token.
std::vector<std::string> split(const std::string& s, char sep) {
    std::vector<std::string> out;
    std::string cur;
    for (char c : s) {
        if (c == sep) {
            out.push_back(cur);
            cur.clear();
        } else {
            cur += c;
        }
    }
    out.push_back(cur);
    return out;
}
// The value fields of "RPM=a;x;y;z" / "TPM=a;x;y;z" (after the '=').
void rateFields(const std::string& tok, float& rate, bool& rateOk, V3& axis) {
    const std::vector<std::string> f = split(tok.substr(4), ';');
    rateOk = fullFloat(f[0], rate);
    for (size_t k = 1; k < f.size() && k <= 3; ++k) axis[k - 1] = float(std::strtod(f[k].c_str(), nullptr));
}

Q4 conj(const Q4& q) { return {-q[0], -q[1], -q[2], q[3]}; }  // 0x40f130
// 0x6a3fc0 (EAX = a, ESI = b): b = Quat_Mul(conj(a), b) = b conj(a).
Q4 relative(const Q4& a, const Q4& b) { return quatMul(conj(a), b); }
float atan2f_(float y, float x) { return float(std::atan2(double(y), double(x))); }  // 0x40f550 fpatan

}  // namespace

// ---- lamp groups -----------------------------------------------------------------------------------------------------

int lampOfMaterial(uint32_t nameCrc) {
    static const uint32_t k[kLampCount] = {asoboCrc("LIGHT_BACK"), asoboCrc("LIGHT_STOP"), asoboCrc("LIGHT_POSITION"), asoboCrc("LIGHT_HEAD")};
    for (int i = 0; i < kLampCount; ++i)
        if (k[i] == nameCrc) return i;
    return -1;
}

std::array<float, kLampCount> lampAlphas(const LampInputs& in) {
    // 0x5cc780: I = 0x3f666666 (0.9) with the headlights on, else 0x3f800000.
    const float I = in.headlights ? 0.9f : 1.0f;
    std::array<float, kLampCount> a{};
    a[kBack] = in.physics && in.gear == -1 ? I : 0.0f;  // 0x4297d0 == 0 or +0x1270 != -1: 0
    a[kStop] = in.brakeValue > 0.1f ? I : 0.0f;  // 0x5cd8f0 > 0.1 (fcomp dword [0x99648c] = 0.1f)
    a[kPosition] = in.headlights ? I : 0.0f;
    a[kHead] = in.headlights ? I : 0.0f;
    return a;
}

// ---- V:DYN records -------------------------------------------------------------------------------------------------

std::vector<std::string> commandTokens(const std::string& line) {
    std::vector<std::string> out;
    size_t i = 0;
    while (i < line.size()) {
        while (i < line.size() && std::isspace(static_cast<unsigned char>(line[i]))) ++i;
        if (i >= line.size()) break;
        std::string t;
        if (line[i] == '"') {
            ++i;
            while (i < line.size() && line[i] != '"') t += line[i++];
            if (i < line.size()) ++i;
        } else {
            while (i < line.size() && !std::isspace(static_cast<unsigned char>(line[i]))) t += line[i++];
        }
        out.push_back(t);
    }
    return out;
}

void reset(DynRecord& r) {  // 0x66f490
    r.damage = 0, r.loose = false, r.damageTotal = 0, r.detached = false;
    r.quat = {0, 0, 0, 1};  // [0xa065b8]
    for (SpringPoint& p : r.points) {
        p.pos = p.rest;
        p.vel = {0, 0, 0};    // [0xa66208] (bss)
        p.adopt = p.vel;
    }
}

std::optional<DynRecord> parseDyn(const std::vector<std::string>& tok, const std::vector<NamedPoint>& spheres, const std::vector<NamedObb>& boxes) {
    if (tok.size() < 3 || !istarts(tok[0], "V:DYN:")) return std::nullopt;  // 0x69e020: more than 2 tokens
    DynRecord r;  // 0x6a2930's defaults
    r.key = asoboCrc(tok[0].substr(2));
    r.name = asoboCrc(tok[1]);
    auto findBox = [&](const std::string& n) -> const NamedObb* {
        const uint32_t c = asoboCrc(n);
        for (const NamedObb& b : boxes)
            if (b.name == c) return &b;
        return nullptr;
    };
    auto findSphere = [&](const std::string& n) -> const NamedPoint* {
        const uint32_t c = asoboCrc(n);
        for (const NamedPoint& s : spheres)
            if (s.name == c) return &s;
        return nullptr;
    };
    for (size_t k = 2; k < tok.size(); ++k) {
        const std::string& t = tok[k];
        float v = 0;
        bool ok = false;
        if (iequal(t, "AUTOSTART")) {
            r.autostart = true;
        } else if (istarts(t, "RPM=")) {
            r.autostart = true;
            rateFields(t, v, ok, r.rpmAxis);
            r.rpm = ok ? float(double(v) * 0.017453292) : 0.020943953f;
        } else if (iequal(t, "RPM")) {
            r.autostart = true;
            r.rpm = 0.020943953f;
        } else if (istarts(t, "TPM=")) {
            r.autostart = true;
            rateFields(t, v, ok, r.tpmAxis);
            r.tpm = ok ? v : 1.0f;
        } else if (iequal(t, "DYNADOPT")) {
            r.adopt = true;
        } else if (istarts(t, "AXE")) {
            const std::string s = t.substr(3);  // strchr from the 4th character, upper-case letters
            if (s.find('X') != std::string::npos) r.axe[0] = 1.0f;
            if (s.find('Y') != std::string::npos) r.axe[1] = 1.0f;
            if (s.find('Z') != std::string::npos) r.axe[2] = 1.0f;
        } else if (istarts(t, "LDAMAGE=") && fullFloat(t.substr(8), v)) {
            r.ldamage = v;
        } else if (istarts(t, "LDYN=") && fullFloat(t.substr(5), v)) {
            r.ldyn = v;
        } else if (istarts(t, "LIMB=")) {
            if (const NamedObb* b = findBox(t.substr(5))) r.limb = true, r.limbBox = b->box;
        } else {
            SpringPoint p;  // 0x6a4100
            std::string pointName, boxName;
            for (const std::string& f : split(t, '+')) {
                if (istarts(f, "POINT=")) pointName = f.substr(6);
                else if (istarts(f, "BOX=")) boxName = f.substr(4);
                else if (istarts(f, "GRAVITY=") && fullFloat(f.substr(8), v)) p.gravity = v;
                else if (istarts(f, "DAMPING=") && fullFloat(f.substr(8), v)) p.damping = v;
                else if (istarts(f, "SPRING=") && fullFloat(f.substr(7), v)) p.spring = v;
            }
            const NamedObb* b = findBox(boxName);
            const NamedPoint* s = findSphere(pointName);
            if (b != nullptr && s != nullptr) {
                p.point = s->name, p.boxName = b->name;
                p.box = b->box;
                p.rest = s->c, p.radius = s->r;
                r.points.push_back(p);
            }
        }
    }
    if (r.points.empty() && r.rpm == 0.0f && r.tpm == 0.0f) return std::nullopt;
    if (r.limb && r.points.size() != 2) return std::nullopt;
    reset(r);
    return r;
}

// ---- per frame -----------------------------------------------------------------------------------------------------

Q4 axisAngle(const V3& axis, float a) {  // Quat_FromAxisAngle 0x40f8b0 (the half angle stored as a float)
    const float h = a * 0.5f;
    const float s = std::sin(h), c = std::cos(h);
    return {axis[0] * s, axis[1] * s, s * axis[2], c};
}

V3 pointVelocity(const Motion& m, const V3& W) {
    const float rx = W[0] - m.pos[0], ry = W[1] - m.pos[1], rz = W[2] - m.pos[2];
    return {m.vel[0] + (m.angVel[2] * ry - m.angVel[1] * rz), m.vel[1] + (m.angVel[0] * rz - m.angVel[2] * rx),
            (m.angVel[1] * rx - m.angVel[0] * ry) + m.vel[2]};
}

V3 rotate(const Q4& q, const V3& v) {  // Quat_RotateVec 0x410510
    const float x = q[0], y = q[1], z = q[2], w = q[3];
    const float t8 = (-(x * v[0]) - y * v[1]) - z * v[2];
    const float t10 = (y * v[2] + w * v[0]) - z * v[1];
    const float t9 = (v[0] * z + w * v[1]) - x * v[2];
    const float t1 = (v[1] * x + w * v[2]) - v[0] * y;
    return {t1 * y + ((w * t10 - t8 * x) - z * t9), t10 * z + ((w * t9 - t8 * y) - t1 * x), x * t9 + ((t1 * w - t8 * z) - y * t10)};
}

Q4 quatMul(const Q4& a, const Q4& b) {  // Quat_Mul 0x410260 (ECX = a, EDX = b)
    Q4 o;
    o[3] = b[3] * a[3] - (a[0] * b[0] + b[1] * a[1] + a[2] * b[2]);
    o[0] = a[0] * b[3] + b[0] * a[3] + (a[2] * b[1] - a[1] * b[2]);
    o[1] = a[1] * b[3] + b[1] * a[3] + (a[0] * b[2] - a[2] * b[0]);
    o[2] = (a[1] * b[0] - b[1] * a[0]) + b[3] * a[2] + a[3] * b[2];
    return o;
}

float advancePhase(float phase, float rpm, float dt) {
    float p = rpm * dt * 0.01f + phase;  // 0x5cb8c0: rpm_sound x dt x 0.01 + +0x1084
    if (6.2831855f < p) p -= 6.2831855f;
    return p;
}

bool obbClamp(const std::array<float, 16>& b, const V3& p, V3& out) {  // 0x75fbc0
    const float d0 = p[0] - b[3], d1 = p[1] - b[7], d2 = p[2] - b[11];
    float l0 = d1 * b[1] + d2 * b[2] + b[0] * d0;
    float l1 = b[5] * d1 + b[6] * d2 + b[4] * d0;
    float l2 = d0 * b[8] + b[10] * d2 + b[9] * d1;
    bool c = false;
    if (l0 < -b[12]) l0 = -b[12], c = true;
    else if (b[12] < l0) l0 = b[12], c = true;
    if (l1 < -b[13]) l1 = -b[13], c = true;
    else if (b[13] < l1) l1 = b[13], c = true;
    if (l2 < -b[14]) l2 = -b[14], c = true;
    else if (b[14] < l2) l2 = b[14], c = true;
    if (!c) return false;
    out[0] = l2 * b[8] + b[0] * l0 + b[4] * l1 + b[3];
    out[1] = l0 * b[1] + b[5] * l1 + b[9] * l2 + b[7];
    out[2] = l0 * b[2] + l1 * b[6] + b[10] * l2 + b[11];
    return true;
}

bool step(DynRecord& r, const Motion& m, const Engine& e, float dt) {
    if (r.rpm != 0.0f) {  // engine shake: 2 x cos(+0x1084) (0.25 + 0.75 +0x1088) x +0x10 about +0x14
        const double a = double(std::cos(e.phase)) * (double(e.throttle) * 0.75 + 0.25) * double(r.rpm);
        r.quat = axisAngle(r.rpmAxis, float(a + a));
        return false;
    }
    if (r.tpm != 0.0f) {  // spin: +0x24 += / -= rpm x tpm x dt x 0.002 (0x5cd940 == -1: reverse), wrapped
        const float inc = e.rpm * r.tpm * dt * 0.002f;
        r.tpmAngle = e.gear == -1 ? r.tpmAngle + inc : r.tpmAngle - inc;
        if (6.2831855f < r.tpmAngle) r.tpmAngle -= 6.2831855f;
        else if (r.tpmAngle < -6.2831855f) r.tpmAngle += 6.2831855f;
        r.quat = axisAngle(r.tpmAxis, r.tpmAngle);
        return false;
    }
    bool debris = false;
    if (r.loose && !r.detached && r.ldyn < r.damageTotal) {  // the part leaves as an IT_DEBRIS (0x6a28d0, 0x533d70)
        r.detached = true;
        debris = true;
    }
    const Q4 qi{-m.q[0], -m.q[1], -m.q[2], m.q[3]};
    for (SpringPoint& p : r.points) {
        // The body moved under the point: its world position W, the body's velocity there (0x5cc370: v + r x w),
        // back to body space by -q and taken off the position.
        const V3 a = rotate(m.q, p.pos);
        const V3 W{a[0] + m.pos[0], a[1] + m.pos[1], a[2] + m.pos[2]};
        const V3 pv = pointVelocity(m, W);
        const V3 d{pv[0] * dt, pv[1] * dt, pv[2] * dt};
        const V3 l = rotate(qi, d);
        for (int k = 0; k < 3; ++k) p.pos[size_t(k)] -= l[size_t(k)];
        // Gravity (0, -1, 0) [0xa0632c] x dt x +0x18, damping +0x1c (x87: the sums stay extended).
        const double g[3] = {0.0, -1.0, 0.0};
        double v[3];
        for (int k = 0; k < 3; ++k) v[k] = g[k] * double(dt) * double(p.gravity) + double(p.vel[size_t(k)]);
        float vd[3];
        for (int k = 0; k < 3; ++k) vd[k] = float(v[k] - v[k] * double(dt) * double(p.damping));
        // The spring +0x20 towards the rest point (body space, turned to world).
        const V3 s{p.rest[0] - p.pos[0], p.rest[1] - p.pos[1], p.rest[2] - p.pos[2]};
        const V3 prev = p.pos;
        const V3 S = rotate(m.q, s);
        for (int k = 0; k < 3; ++k) p.vel[size_t(k)] = float(double(S[size_t(k)]) * double(dt) * double(p.spring) + double(vd[k]));
        const V3 V = rotate(qi, p.vel);
        for (int k = 0; k < 3; ++k) p.pos[size_t(k)] = float(double(V[size_t(k)]) * double(dt) + double(p.pos[size_t(k)]));
        // Kept in the BOX (0x75fbc0): the correction from the step's start gives the velocity (x 1 / dt).
        V3 c;
        if (obbClamp(p.box, p.pos, c)) {
            const V3 diff{c[0] - prev[0], c[1] - prev[1], c[2] - prev[2]};
            const V3 dv = rotate(m.q, diff);  // 0x4db890
            const double inv = 1.0 / double(dt);
            for (int k = 0; k < 3; ++k) p.vel[size_t(k)] = float(inv * double(dv[size_t(k)]));
            p.pos = c;
        }
    }
    return debris;
}

// ---- the part pose -------------------------------------------------------------------------------------------------

DynPose pose(const DynRecord& r, const V3& b, const Q4& boneQ) {
    DynPose out;
    out.pos = b;
    if (r.detached) {
        out.state = 2;
        return out;
    }
    if (!r.autostart && !r.loose) return out;  // LAB_006a159e: the static placement
    out.state = 1;
    Q4 acc{0, 0, 0, 1};  // [0xa065b8]
    if (r.rpm != 0.0f || r.tpm != 0.0f) {
        acc = r.quat;
    } else {
        V3 p0 = r.points.empty() ? V3{} : r.points[0].pos;
        V3 p1{};  // [0xa66208]
        if (r.points.size() > 1) p1 = r.points[1].pos;
        if (r.adopt) {
            if (!r.points.empty()) p0 = r.points[0].adopt;
            if (r.points.size() > 1) p1 = r.points[1].adopt;
        }
        if (r.limb) {
            V3 mid{(p1[0] + p0[0]) * 0.5f, (p1[1] + p0[1]) * 0.5f, (p1[2] + p0[2]) * 0.5f};
            V3 c;
            if (obbClamp(r.limbBox, mid, c)) mid = c;
            const Q4 qy = axisAngle({0, 1, 0}, atan2f_(p0[0] - mid[0], p0[2] - mid[2]));  // [0xa06320]
            const Q4 qx = axisAngle({1, 0, 0}, atan2f_(p0[1] - p1[1], p0[2] - p1[2]));    // [0xa06338]
            acc = quatMul(conj(qx), conj(qy));
            out.pos = mid;
        } else {
            const V3 rr{r.points.empty() ? 0.0f : r.points[0].rest[0] - b[0], r.points.empty() ? 0.0f : r.points[0].rest[1] - b[1],
                        r.points.empty() ? 0.0f : r.points[0].rest[2] - b[2]};
            const V3 a{p0[0] - b[0], p0[1] - b[1], p0[2] - b[2]};
            const Q4 qx = relative(axisAngle({1, 0, 0}, atan2f_(rr[2], rr[1])), axisAngle({1, 0, 0}, atan2f_(a[2], a[1])));
            const Q4 qy = relative(axisAngle({0, 1, 0}, atan2f_(rr[0], rr[2])), axisAngle({0, 1, 0}, atan2f_(a[0], a[2])));
            const Q4 qz = relative(axisAngle({0, 0, 1}, atan2f_(rr[1], rr[0])), axisAngle({0, 0, 1}, atan2f_(a[1], a[0])));
            if (r.axe[0] != 0.0f) acc = quatMul(qx, acc);
            if (r.axe[1] != 0.0f) acc = quatMul(qy, acc);
            if (r.axe[2] != 0.0f) acc = quatMul(qz, acc);
        }
    }
    // 0x6a0a2c: Quat_Mul(boneQ, acc) = acc boneQ (then the actor's rotation, then 0x41d4b0 normalises).
    Q4 q = quatMul(boneQ, acc);
    const float n = std::sqrt(q[0] * q[0] + q[1] * q[1] + q[2] * q[2] + q[3] * q[3]);
    if (n > 0) q = {q[0] / n, q[1] / n, q[2] / n, q[3] / n};
    out.rot = q;
    return out;
}

// ---- damage --------------------------------------------------------------------------------------------------------

void addDamage(DynRecord& r, float f, float v, float s) {
    // 0x6a2480: v, s clamped at 0 by the caller's entry; f x (s + v) into +0x34 and +0x38; above 1.6 [0x9dc470] the
    // part is loose (+0 = 1, +0x34 = 0) and, with DYNADOPT, each point's position is copied to +0x7c.
    const double add = double(f) * double(s + v);
    const double d = double(r.damage) + add;
    r.damage = float(d);
    r.damageTotal = float(double(r.damageTotal) + add);
    if (1.600000023841858 < d) {
        if (r.adopt)
            for (SpringPoint& p : r.points) p.adopt = p.pos;
        r.loose = true;
        r.damage = 0;
    }
}

bool sphereBox(const std::array<float, 16>& b, const V3& c, float r, float& dist) {  // 0x75fd20 (the distance only)
    const float d0 = c[0] - b[3], d1 = c[1] - b[7], d2 = c[2] - b[11];
    const float l[3] = {d1 * b[1] + d2 * b[2] + b[0] * d0, b[5] * d1 + b[6] * d2 + b[4] * d0, d0 * b[8] + b[10] * d2 + b[9] * d1};
    const float r2 = r * r;
    float sq = 0;
    for (int k = 0; k < 3; ++k) {
        const float h = b[size_t(12 + k)];
        if (l[k] < -h) sq = (l[k] + h) * (l[k] + h) + sq;
        else if (h < l[k]) sq = (l[k] - h) * (l[k] - h) + sq;
        if (r2 < sq) return false;
    }
    if (sq <= 1e-05f) {  // inside: the smallest penetration + r
        float best = 0;
        for (int k = 0; k < 3; ++k) {
            const float h = b[size_t(12 + k)];
            const float pen = l[k] <= 0 ? h + l[k] : h - l[k];
            if (k == 0) best = std::min(pen, 1e6f);
            else if (pen < best) best = pen;
        }
        dist = best + r;
        return dist <= 1e10f;
    }
    dist = float(std::sqrt(double(sq)));
    return dist <= 1e10f;
}

// ---- dirt ----------------------------------------------------------------------------------------------------------

namespace {
void addAll(float s[10], float& total, float v) {  // Damage_AddAll 0x420020
    for (int i = 0; i < 10; ++i) {
        s[i] += v;
        if (1.0f < s[i]) s[i] = 1.0f;
        if (s[i] < 0.0f) s[i] = 0.0f;
    }
    total = v + total;
}
void lower(float s[10], float v, float floor) {  // Damage_Lower 0x4200c0
    for (int i = 0; i < 10; ++i) {
        if (floor < s[i]) {
            s[i] -= v;
            if (1.0f < s[i]) s[i] = 1.0f;
            if (s[i] < floor) s[i] = floor;
        }
    }
}
}  // namespace

void stepDirt(float slots[10], float& total, float dt, bool loaded, float meanDirty, float speed, float waterDepth, float rain, const DirtParams& P) {
    const float floor = 0.05f;  // [0x996158] 0x3d4ccccd
    if (0.0f < waterDepth) {    // +0x1380 > 0: washed by the water (P597)
        lower(slots, P.p597 * dt, floor);
        return;
    }
    if (0.25f <= rain) lower(slots, float(double(rain) * P.p598 * dt), floor);  // the rain (P598) from 0.25 on
    if (!loaded) return;
    const float v = speed - P.p594;
    if (v <= 0.0f) return;
    addAll(slots, total, float(double(meanDirty) * P.p595 * v * dt));  // (sum / n) x P595 x (speed - P594) x dt
    lower(slots, P.p596 * dt, floor);                                  // and the dry wash P596
}

float dirtMean(const float s[10]) {
    return (s[0] + s[1] + s[2] + s[3] + s[4] + s[5] + s[6] + s[7] + s[8] + s[9]) * 0.1f;
}

}  // namespace fuel::vlinks
