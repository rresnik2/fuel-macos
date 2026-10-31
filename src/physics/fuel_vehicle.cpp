// FUEL's vehicle model, engine side and per-substep vehicle hooks. See fuel_vehicle.hpp and
// ~/re-work/fuel/notes/p23-vehicle-physics.md (sections 2-8, Part E) for the decode.
#include "fuel_vehicle.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <map>
#include <sstream>
#include <stdexcept>

namespace fuel::fv {

namespace {
constexpr Vec3 kUp{0, 1, 0};
constexpr float kTwoPi = 6.2831853f;
// Proxy sphere names of the wheels (CRC by position, p7 N11), in the engine's wheel order: front wheels are
// 0 and 2, rear 1 and 3 (the anti-roll pairs (0,2), (1,3) and the Ackermann axles say so).
constexpr uint32_t kWheelName[4] = {0xD8B7A397, 0xFC33776C, 0xA828D8ED, 0x8CAC0C16};
}  // namespace

// ---- ground tables -------------------------------------------------------------------------

GroundTables parseGroundTables(const std::string& text) {
    std::map<int, float> pp;
    std::istringstream in(text);
    std::string line;
    while (std::getline(in, line)) {
        std::istringstream ls(line);
        std::string tag;
        int n = 0;
        if (!(ls >> tag >> n) || tag != "PP" || n < 4000 || n > 4400) continue;
        float v;
        if (ls >> v) pp[n] = v;
    }
    GroundTables t;
    auto row = [&](int base, std::array<float, kGroundClasses>& out) {
        bool ok = true;
        for (int k = 0; k < kGroundClasses; ++k) {
            auto it = pp.find(base + 1 + k);
            ok = ok && it != pp.end();
            out[size_t(k)] = it != pp.end() ? it->second : 0.0f;
        }
        return ok;
    };
    bool ok = true;
    for (int wet = 0; wet < 2; ++wet) {
        const int o = wet * 114;
        ok = row(4000 + o, t.stat[wet]) && ok;
        ok = row(4019 + o, t.kin[wet]) && ok;
        ok = row(4038 + o, t.dyn[wet]) && ok;
        ok = row(4057 + o, t.rolling[wet]) && ok;
        ok = row(4076 + o, t.soft[wet]) && ok;
        ok = row(4095 + o, t.sideSlow[wet]) && ok;
    }
    row(4342, t.dirty[0]);
    row(4361, t.dirty[1]);
    row(4228, t.bumpsRot);
    row(4247, t.bumpsWheel);
    row(4266, t.depth);
    row(4285, t.boost);
    row(4304, t.tremRoll);
    row(4323, t.tremPitch);
    row(4380, t.rollDamage);
    t.loaded = ok;
    return t;
}

GroundBlock buildGroundBlock(const GroundTables& t, const float frac[kGroundClasses], float wet) {
    GroundBlock c;
    float sum = 0;
    for (int k = 0; k < kGroundClasses; ++k) sum += frac[k];
    if (!(sum > 0) || !t.loaded) return c;
    auto mix = [&](const std::array<float, kGroundClasses>* rows, bool wetRow) {
        float dry = 0, w = 0;
        for (int k = 0; k < kGroundClasses; ++k) {
            dry += frac[k] * rows[0][size_t(k)];
            w += frac[k] * rows[wetRow ? 1 : 0][size_t(k)];
        }
        return (dry * (1 - wet) + w * wet) / sum;
    };
    auto one = [&](const std::array<float, kGroundClasses>& r) {
        float v = 0;
        for (int k = 0; k < kGroundClasses; ++k) v += frac[k] * r[size_t(k)];
        return v / sum;
    };
    c.stat = mix(t.stat, true);
    c.kin = mix(t.kin, true);
    c.dyn = mix(t.dyn, true);
    c.rolling = mix(t.rolling, true);
    c.soft = mix(t.soft, true);
    c.hard = 1 - c.soft;
    c.sideSlow = mix(t.sideSlow, true);
    c.dirty = mix(t.dirty, true);
    c.bumpsRot = one(t.bumpsRot);
    c.bumpsWheel = one(t.bumpsWheel);
    c.depth = one(t.depth);
    c.boost = one(t.boost);
    c.tremPitch = one(t.tremPitch);
    c.tremRoll = one(t.tremRoll);
    c.rollDamage = one(t.rollDamage);
    return c;
}

// ---- construction --------------------------------------------------------------------------

FuelVehicle::FuelVehicle(std::vector<float> params, const ProxyShapes& proxy, const GroundTables* ground, bool player)
    : P_(std::move(params)), proxy_(proxy), tables_(ground), player_(player) {
    if (P_.size() < 680) P_.resize(680, 0.0f);
    if (!(P(2) > 10 && P(2) < 100000)) throw std::runtime_error("implausible vehicle mass");
    nWheels_ = int(std::floor(P(45) + 0.5f));
    // Wheels by proxy sphere name.
    const int want = std::clamp(nWheels_, 2, 4);
    wheelSphere_.clear();
    for (int w = 0; w < want; ++w) {
        int found = -1;
        for (size_t i = 0; i < proxy_.spheres.size(); ++i)
            if (proxy_.spheres[i].name == kWheelName[w]) found = int(i);
        if (found < 0) break;
        wheelSphere_.push_back(found);
    }
    nWheels_ = int(wheelSphere_.size());
    if (nWheels_ != 2 && nWheels_ != 4) throw std::runtime_error("vehicle proxy needs two or four wheel spheres");
    for (Vec3& d : wheelDirPost_) d = {0, 0, 1};
    comBase_ = {0, P(11), P(12)};
    com_ = comPrev_ = comBase_;
    body_.mass = P(2);
    body_.invMass = 1.0f / body_.mass;
    computeMassProperties();
    buildWheels();
    BodyState& S = body_.S;
    S.nAux = 1 + nWheels_;
    S.auxVel[0] = 1000;  // 0x520400
    for (int w = 0; w < nWheels_; ++w) S.auxDead[1 + w] = 0.05f;
    C_.assign(size_t(nWheels_), GroundBlock{});
    frac_.assign(size_t(nWheels_), {});
    for (auto& f : frac_) f[3] = 1;
    waterDepth_.assign(size_t(nWheels_), 0.0f);
    if (tables_ && tables_->loaded)
        for (int w = 0; w < nWheels_; ++w) C_[size_t(w)] = buildGroundBlock(*tables_, frac_[size_t(w)].data(), 0);
    tyreCoefficients();
}

void FuelVehicle::computeMassProperties() {
    // 0x712de0: volume pass (sphere 4.1762004 r^3 as coded, box 8 hx hy hz), density = mass / volume;
    // inertia pass (sphere m = d 4.1887903 r^3, I = 0.4 m r^2; box I = m/12 (b^2 + c^2)), parallel axis to
    // the centre of mass, rescaled so the masses sum to the vehicle's; then x diag(P4..P6) / diag(P7..P9).
    float volume = 0;
    for (const auto& s : proxy_.spheres) volume += 4.1762004f * s.r * s.r * s.r;
    for (const auto& b : proxy_.boxes) volume += 8 * b.half.x * b.half.y * b.half.z;
    Mat3 I = Mat3::zero();
    float msum = 0;
    float boundR = 0.2f;
    if (volume > 0) {
        const float density = body_.mass / volume;
        auto add = [&](float m, const Mat3& Ilocal, const Vec3& at) {
            const Vec3 r = at - com_;
            Mat3 shift = Mat3::zero();
            const float r2 = len2(r);
            const float rv[3] = {r.x, r.y, r.z};
            for (int i = 0; i < 3; ++i)
                for (int j = 0; j < 3; ++j) shift.m[i][j] = m * ((i == j ? r2 : 0.0f) - rv[i] * rv[j]);
            I = I + Ilocal + shift;
            msum += m;
        };
        for (const auto& s : proxy_.spheres) {
            const float m = density * 4.1887903f * s.r * s.r * s.r;
            const float i = 0.4f * m * s.r * s.r;
            add(m, Mat3::diag(i, i, i), s.c);
            boundR = std::max(boundR, len(s.c - com_) + s.r);
        }
        for (const auto& b : proxy_.boxes) {
            const float m = 8 * b.half.x * b.half.y * b.half.z * density;
            const float ex = 2 * b.half.x, ey = 2 * b.half.y, ez = 2 * b.half.z;
            add(m, Mat3::diag(m / 12 * (ey * ey + ez * ez), m / 12 * (ex * ex + ez * ez), m / 12 * (ex * ex + ey * ey)), b.c);
            boundR = std::max(boundR, len(b.c - com_) + len(b.half));
        }
    }
    if (msum > 0) {
        I = I * (body_.mass / msum);
    } else {
        // Reset default (0x70bf50): diag 0.66 r^2 (per unit mass), here times the mass.
        const float i = 0.66f * boundR * boundR * body_.mass;
        I = Mat3::diag(i, i, i);
    }
    body_.boundR = boundR;
    body_.I1 = I * Mat3::diag(P(4), P(5), P(6));
    body_.I2 = I * Mat3::diag(P(7), P(8), P(9));
    body_.I1inv = body_.I1.inverse();
    body_.I2inv = body_.I2.inverse();
    body_.updateWorldInertia();
}

void FuelVehicle::buildWheels() {
    // 0x52def0: hub = proxy sphere - CoM (offset helper 0x5d7a90: Unknown U4, 0 in the port); sphere radius =
    // proxy radius + P55 (front) | P56 (rear); the tyre material (front odd parameter, rear even).
    wheels_.assign(size_t(nWheels_), WheelRec{});
    for (int w = 0; w < nWheels_; ++w) {
        WheelRec& r = wheels_[size_t(w)];
        const auto& s = proxy_.spheres[size_t(wheelSphere_[size_t(w)])];
        const bool f = front(w);
        auto pp = [&](int frontParam) { return P(f ? frontParam : frontParam + 1); };
        r.hubL = s.c - com_;
        r.radius = s.r + pp(55);
        ContactMat& m = r.mat;
        m.c = pp(75);
        m.bias = 0.5f;
        m.maxDepth = std::min(pp(71), r.radius - 0.2f);
        m.preload = pp(73);
        m.dampC = pp(79), m.dampR = pp(81), m.quadC = pp(83), m.quadR = pp(85);
        m.maxN = pp(89);
        m.loadBlend = pp(275);
        m.loadExp = pp(77);
        m.set1[0] = pp(108), m.set1[2] = pp(108) * 0.5f, m.set1[3] = pp(156), m.set1[4] = pp(160), m.set1[5] = 0,
        m.set1[6] = P(162), m.set1[7] = pp(120);
        for (int i = 0; i < 8; ++i) m.set2[i] = m.set1[i];
        r.invR = 1.0f / std::max(wheelR(w), 0.01f);
        r.tyreAxis = true;
        r.useTyre = true;
    }
}

// ---- helpers -------------------------------------------------------------------------------

float FuelVehicle::ratio(int g) const {  // 0x524d00
    if (g > 0) return P(350) * P(351 + g);
    if (g == -1) return P(350) * P(351);
    return 1;
}
float FuelVehicle::gearMul(int g) const {
    if (g > 0) return P(359 + g);
    if (g == -1) return P(359);
    return 1;
}
float FuelVehicle::engineTorque(float rpm) const {  // 0x524c70 (cell index rounds down)
    const float f = clampf(rpm / P(324), 0, 1) * 10.0f;
    const int i = int(std::floor(f));
    const int j = std::min(i + 1, 10);
    const float t = f - float(i);
    return ((1 - t) * P(338 + i) + P(338 + j) * t) * P(327);
}
float FuelVehicle::meanDrivenRadius() const {  // 0x525680
    float sum = 0;
    int n = 0;
    for (int w = 0; w < nWheels_; ++w)
        if (driven(w)) sum += wheelR(w), ++n;
    return n ? sum / float(n) : wheelR(0);
}
float FuelVehicle::meanDrivenWheelSpeed() const {  // 0x525710
    float sum = 0;
    int n = 0;
    for (int w = 0; w < nWheels_; ++w) {
        if (!driven(w)) continue;
        float x = body_.S.auxVel[1 + w];
        if (std::fabs(x) < body_.S.auxDead[1 + w]) x = 0;
        sum += x, ++n;
    }
    return n ? sum / float(n) : 0.0f;
}
float FuelVehicle::engineSpeedAtWheel() const {  // 0x525980
    if (gear_ == 0) return 0;
    return body_.S.auxVel[0] / ratio(gear_) * meanDrivenRadius() * 0.10471976f;
}
float FuelVehicle::wheelTorqueAtSpeed(float v, int g) const {  // 0x524d70
    const float r = ratio(g);
    return engineTorque(r * (v * 60.0f / (meanDrivenRadius() * kTwoPi))) * gearMul(g) * r;
}
float FuelVehicle::slipRatio(int w) const {  // 0x523c50
    const WheelRec& r = wheels_[size_t(w)];
    if (r.load <= 1e-6f) return 100000.0f;
    if (r.sumDen >= 1e-6f) return r.sumFn / r.sumDen;
    return 0;
}
float FuelVehicle::wheelUtil(int w) const { return slipRatio(w); }
bool FuelVehicle::anyWheelLoaded() const {
    for (int w = 0; w < nWheels_; ++w)
        if (wheels_[size_t(w)].load > 1e-6f) return true;
    return false;
}
bool FuelVehicle::allWheelsLoaded() const {
    for (int w = 0; w < nWheels_; ++w)
        if (!(wheels_[size_t(w)].load > 1e-6f)) return false;
    return true;
}
bool FuelVehicle::wheelSlidingOrUnloaded(int w) const {  // 0x524290
    const WheelRec& r = wheels_[size_t(w)];
    if (r.load <= 0) return true;
    return r.sumDen >= 1e-6f && r.sumFn / r.sumDen >= 1;
}
bool FuelVehicle::wheelSliding(int w) const {  // 0x524240
    const WheelRec& r = wheels_[size_t(w)];
    return r.load > 0 && r.sumDen >= 1e-6f && r.sumFn / r.sumDen >= 1;
}
bool FuelVehicle::enoughWheelsSliding() const {  // 0x5243a0
    int n = 0;
    for (int w = 0; w < nWheels_; ++w) n += wheelSliding(w) ? 1 : 0;
    return n >= (nWheels_ <= 2 ? 1 : 2);
}
bool FuelVehicle::allFrontSliding() const {  // 0x524560
    for (int w = 0; w < nWheels_; ++w)
        if (front(w) && !wheelSliding(w)) return false;
    return true;
}
bool FuelVehicle::allFrontSlidingOrAir() const {  // 0x5246c0
    for (int w = 0; w < nWheels_; ++w)
        if (front(w) && !wheelSlidingOrUnloaded(w)) return false;
    return true;
}
bool FuelVehicle::drivenWheelsSliding() const {  // 0x524440
    for (int w = 0; w < nWheels_; ++w)
        if (driven(w) && wheelSliding(w)) return true;
    return false;
}
float FuelVehicle::minDrivenSlipRatio() const {  // 0x523be0
    float m = 1e30f;
    for (int w = 0; w < std::min(nWheels_, 4); ++w)
        if (driven(w)) m = std::min(m, slipRatio(w));
    return m;
}
void FuelVehicle::setWheelSliding(int w, bool sliding, float load) {
    WheelRec& r = wheels_[size_t(w)];
    r.load = load;
    r.sumDen = 1;
    r.sumFn = sliding ? 2.0f : 0.5f;
}
void FuelVehicle::setGroundBlock(int w, const GroundBlock& c, const float frac[kGroundClasses]) {
    C_[size_t(w)] = c;
    for (int k = 0; k < kGroundClasses; ++k) frac_[size_t(w)][size_t(k)] = frac[k];
}
float FuelVehicle::surfaceClassFraction(int k) const {  // 0x51ffe0
    float s = 0;
    for (int w = 0; w < nWheels_; ++w) s += frac_[size_t(w)][size_t(k)];
    return nWheels_ ? s / float(nWheels_) : 0.0f;
}
float FuelVehicle::wheelSpin(int w) const { return wheelAngle_[w]; }
float FuelVehicle::wheelSteer(int w) const {
    const Vec3 d = w == 0 ? st_.dir0 : (w == 2 ? st_.dir2 : Vec3{0, 0, 1});
    return std::atan2(d.x, d.z);
}

// ---- per substep ----------------------------------------------------------------------------

void FuelVehicle::wheelUpdate() {
    body_.P = body_.S;  // 0x421540 (the averaged accelerations of the last substep stay in P)
    const BodyState& S = body_.S;
    const Vec3 upW = rotate(S.q, kUp);
    for (int w = 0; w < nWheels_; ++w) {  // 0x5213c0
        WheelRec& r = wheels_[size_t(w)];
        const Vec3 dir = w == 0 ? st_.dir0 : (w == 2 ? st_.dir2 : Vec3{0, 0, 1});
        const float rim = S.auxVel[1 + w];
        const float rr = std::max(wheelR(w), 0.01f);
        r.rimVelL = -dir * rim;
        r.rimAccL = -dir * (dOmega_[w] * P(69));
        r.dirL = dir;
        r.angL = cross(kUp, r.rimVelL) / rr;
        r.invR = 1.0f / rr;
        r.slidingDriven = slipRatio(w) > 1 && driven(w);
        r.rimVelW = rotate(S.q, r.rimVelL);
        r.rimAccW = rotate(S.q, r.rimAccL);
        r.dirW = rotate(S.q, r.dirL);
        r.angW = rotate(S.q, r.angL);
        r.hubW = S.x + rotate(S.q, r.hubL);
        r.up = upW;
    }
}

namespace {

// Contact_Add 0x70ce10 with the vehicle hooks: the contact frame 0x51f1e0 (static chassis contacts: the
// wall normal bent towards the chassis at speed) and the filter 0x51f6c0 (restitution for chassis
// contacts). Returns false when rejected.
bool addContact(Contact& c, RigidBody& body, const Quat& q, bool wrecked, float restitutionP284, float restitutionP285,
                std::vector<Contact>& out) {
    if (c.depth < 1e-6f) return false;
    const Vec3 up = rotate(q, {0, 1, 0});
    if ((c.wheel == nullptr || c.depth > c.m.maxDepth) && !wrecked) {
        if (c.b == nullptr) {
            const float cc = std::max(dot(up, c.n), c.n.y);
            Vec3 d = unrotate(q, body.S.x - c.p);
            d = rotate(q, {d.x, 0.25f * d.y, 0.1f * d.z});
            const Vec3 dh = normalize(d);
            const Vec3 n1 = normalize(c.n + dh * (1 - std::fabs(cc)));
            const float t = clampf((len(body.S.v) - 5) * 0.1f, 0, 1);
            const Vec3 n2 = normalize({t * n1.x + (1 - t) * c.n.x, n1.y, t * n1.z + (1 - t) * c.n.z});
            c.depth = std::max(c.depth * dot(c.n, n2), 0.0f);
            c.n = n2;
        } else {
            for (float* f : {&c.m.set1[0], &c.m.set1[2], &c.m.set2[0], &c.m.set2[2]}) *f = 0;  // vehicle-vehicle: frictionless
        }
    }
    Vec3 rel = body.pointVel(c.p) + (c.wheel ? c.surfVelWheel : c.surfVel);
    if (c.b) rel -= c.b->pointVel(c.p);
    c.approach = -dot(rel, c.n);
    c.tangential = len(rel - c.n * dot(rel, c.n));
    if (c.wheel == nullptr) {
        const float e = c.b == nullptr ? restitutionP284 : restitutionP285;
        c.m.dampC = (1 - std::fabs(dot(c.n, up))) * e;
        body.scrapeSpeed = std::max(body.scrapeSpeed, c.tangential);
    }
    body.impactSpeed = std::max(body.impactSpeed, c.approach);
    out.push_back(c);
    return true;
}

ContactMat chassisMaterial(float P282, float P283, uint8_t wrecked, int wheels) {  // 0x52ff30 + 0x521ec0
    ContactMat m;
    m.c = 0;
    m.dampC = P283;
    float mu = std::min(P282, 0.341f);
    if (wrecked) mu = wheels >= 3 ? 0.75f : 1.5f;
    m.set1[0] = m.set1[2] = mu;
    m.set1[3] = 5;
    m.maxDepth = 1e11f;
    m.bias = 0.5f;
    m.maxN = 50;
    m.loadExp = 1;
    m.dampR = m.quadC = m.quadR = 1;
    for (int i = 0; i < 8; ++i) m.set2[i] = m.set1[i];
    return m;
}

}  // namespace

void FuelVehicle::generateContacts(Environment& env, const std::vector<FuelVehicle*>& others) {
    contacts_.clear();
    body_.updateWorldInertia();
    body_.slidingContacts = 0;
    const BodyState& S = body_.S;
    const Vec3 upW = rotate(S.q, kUp);
    std::vector<const Tri*> tris;
    std::vector<StaticHit> hits;
    // Wheels (0x723400).
    for (int w = 0; w < nWheels_; ++w) {
        WheelRec& r = wheels_[size_t(w)];
        r.maxDepthSeen = r.load = r.compRatio = r.sumFn = r.sumDen = 0;
        r.fric = r.slip = r.nDepth = {};
        r.surface = -1;
        if (!r.active) continue;
        const Vec3 c = r.hubW;
        const float rad = r.radius;
        // Water floor (World_SphereVsTerrain mode 4): 0.2 m below the water surface.
        if (env.waterDepthAt && env.terrain) {
            const float wd = env.waterDepthAt(double(c.x) + env.originX, double(c.z) + env.originZ);
            const float off = std::max(wd - 0.2f, 0.0f);
            if (off > 1e-6f) {
                const float h = env.terrain->surfaceHeight(c.x, c.z);
                const float depth = std::min(rad - (c.y - (h + off)), 1.5f);
                if (depth > 0.001f) {
                    Contact ct;
                    ct.n = kUp;
                    ct.depth = depth;
                    ct.p = c - ct.n * rad;
                    ct.a = &body_;
                    ct.surface = 1;
                    attachWheel(ct, r, upW);
                    addContact(ct, body_, S.q, wrecked_, P(284), P(285), contacts_);
                }
            }
        }
        // Terrain triangles (GenWorld_Z 0x77f510) and static objects.
        if (env.terrain) {
            tris.clear();
            env.terrain->query(c.x, c.z, rad, tris);
            for (const Tri* t : tris) {
                float depth;
                Vec3 n, p;
                if (!sphereTriangle(*t, c, rad, depth, n, p)) continue;
                if (depth > r.mat.maxDepth || dot(t->n, n) > 0.3f) n = t->n;
                Contact ct;
                ct.p = p, ct.n = n, ct.depth = depth;
                ct.a = &body_;
                ct.surface = t->surface;
                ct.fromTriangle = true;
                attachWheel(ct, r, upW);
                if (addContact(ct, body_, S.q, wrecked_, P(284), P(285), contacts_)) r.surface = t->surface;
            }
        }
        if (env.sphereStatics) {
            hits.clear();
            env.sphereStatics(c, rad, hits);
            for (const StaticHit& h : hits) {
                Contact ct;
                ct.p = h.p, ct.n = h.n, ct.depth = h.depth;
                ct.a = &body_;
                ct.surface = 3;
                attachWheel(ct, r, upW);
                addContact(ct, body_, S.q, wrecked_, P(284), P(285), contacts_);
            }
        }
    }
    // Chassis: proxy spheres that are not wheels (0x723b80), lowered by P10 (+0x450), and boxes (0x724120).
    const ContactMat chassis = chassisMaterial(P(282), P(283), wrecked_, nWheels_);
    const Vec3 lower{0, -P(10), 0};
    for (size_t i = 0; i < proxy_.spheres.size(); ++i) {
        if (std::find(wheelSphere_.begin(), wheelSphere_.end(), int(i)) != wheelSphere_.end()) continue;
        const auto& s = proxy_.spheres[i];
        const Vec3 c = S.x + rotate(S.q, s.c - com_ + lower);
        if (env.terrain) {
            tris.clear();
            env.terrain->query(c.x, c.z, s.r, tris);
            for (const Tri* t : tris) {
                Contact ct;
                if (!sphereTriangle(*t, c, s.r, ct.depth, ct.n, ct.p)) continue;
                ct.a = &body_;
                ct.m = chassis;
                ct.surface = t->surface;
                addContact(ct, body_, S.q, wrecked_, P(284), P(285), contacts_);
            }
        }
        if (env.sphereStatics) {
            hits.clear();
            env.sphereStatics(c, s.r, hits);
            for (const StaticHit& h : hits) {
                Contact ct;
                ct.p = h.p, ct.n = h.n, ct.depth = h.depth, ct.a = &body_, ct.m = chassis;
                addContact(ct, body_, S.q, wrecked_, P(284), P(285), contacts_);
            }
        }
    }
    for (const auto& b : proxy_.boxes) {
        const Vec3 c = S.x + rotate(S.q, b.c - com_);
        // Box vs terrain (0x74ead0, not transcribed): the port tests the eight corners against the
        // triangulated ground.
        if (env.terrain) {
            for (int k = 0; k < 8; ++k) {
                const Vec3 corner = c + rotate(S.q, {(k & 1) ? b.half.x : -b.half.x, (k & 2) ? b.half.y : -b.half.y,
                                                     (k & 4) ? b.half.z : -b.half.z});
                Vec3 n;
                const float h = env.terrain->surfaceHeight(corner.x, corner.z, &n);
                if (corner.y >= h) continue;
                Contact ct;
                ct.n = n;
                ct.depth = (h - corner.y) * n.y;
                ct.p = corner;
                ct.a = &body_;
                ct.m = chassis;
                ct.surface = 2;
                addContact(ct, body_, S.q, wrecked_, P(284), P(285), contacts_);
            }
        }
        if (env.boxStatics) {
            hits.clear();
            env.boxStatics(c, S.q, b.half, hits);
            for (const StaticHit& h : hits) {
                Contact ct;
                ct.p = h.p, ct.n = h.n, ct.depth = h.depth, ct.a = &body_, ct.m = chassis;
                addContact(ct, body_, S.q, wrecked_, P(284), P(285), contacts_);
            }
        }
    }
    // Vehicle vs vehicle (0x725490): the port pairs the chassis boxes and the wheel spheres as spheres of
    // the box's inner radius; rigid, frictionless (0x51f1e0), restitution from P285.
    for (FuelVehicle* o : others) {
        if (o == this || o < this) continue;  // each pair once (the contact carries both bodies)
        if (len(o->body_.S.x - S.x) > body_.boundR + o->body_.boundR) continue;
        auto spheres = [](const FuelVehicle& v, std::vector<std::pair<Vec3, float>>& out) {
            const BodyState& s = v.body_.S;
            for (const auto& b : v.proxy_.boxes) {
                const float r = std::min({b.half.x, b.half.y, b.half.z});
                // Spheres along the box's longest axis.
                const Vec3 h = b.half;
                const int axis = h.x >= h.y && h.x >= h.z ? 0 : (h.y >= h.z ? 1 : 2);
                const float half = axis == 0 ? h.x : axis == 1 ? h.y : h.z;
                const int n = std::max(1, int(std::ceil((half - r) / r)) + 1);
                for (int i = 0; i < n; ++i) {
                    const float t = n == 1 ? 0.0f : -half + r + (2 * (half - r)) * float(i) / float(n - 1);
                    const Vec3 off = axis == 0 ? Vec3{t, 0, 0} : axis == 1 ? Vec3{0, t, 0} : Vec3{0, 0, t};
                    out.push_back({s.x + rotate(s.q, b.c - v.com_ + off), r});
                }
            }
            for (int w = 0; w < v.nWheels_; ++w) out.push_back({v.wheels_[size_t(w)].hubW, v.wheels_[size_t(w)].radius * 0.8f});
        };
        std::vector<std::pair<Vec3, float>> A, B;
        spheres(*this, A);
        spheres(*o, B);
        for (const auto& [ca, ra] : A)
            for (const auto& [cb, rb] : B) {
                const Vec3 d = ca - cb;
                const float dist = len(d);
                if (dist >= ra + rb || dist < 1e-6f) continue;
                Contact ct;
                ct.n = d / dist;
                ct.depth = ra + rb - dist;
                ct.p = cb + ct.n * rb;
                ct.a = &body_;
                ct.b = &o->body_;
                ct.m = chassis;
                addContact(ct, body_, S.q, wrecked_, P(284), P(286), contacts_);
            }
    }
    // Reduction (0x71b7e0): with more than two contacts, for two penalty contacts of the same wheel with
    // n1.n2 > 0 the later (shallower: the port sorts by depth) gets depth 0.
    if (contacts_.size() > 2) {
        std::stable_sort(contacts_.begin(), contacts_.end(), [](const Contact& a, const Contact& b) {
            if (a.wheel != b.wheel) return a.wheel < b.wheel;
            return a.depth > b.depth;
        });
        for (size_t i = 0; i < contacts_.size(); ++i) {
            if (contacts_[i].wheel == nullptr || contacts_[i].m.c == 0 || contacts_[i].depth <= 0) continue;
            for (size_t j = i + 1; j < contacts_.size() && contacts_[j].wheel == contacts_[i].wheel; ++j)
                if (dot(contacts_[i].n, contacts_[j].n) > 0) contacts_[j].depth = 0;
        }
    }
    if (contacts_.size() > 256) contacts_.resize(256);
}

void FuelVehicle::forces() {
    body_.F = body_.T = {};
    Vec3 nsum;
    for (Contact& c : contacts_) {
        penaltyForce(c);
        nsum += c.n * c.depth;
    }
    for (auto& r : wheels_) r.util = r.sumDen != 0 ? r.sumFn / r.sumDen : 1.0f;  // 0x70f490
    if (len2(nsum) > 0) body_.meanNormal = normalize(nsum);
    bodyForces();
    // 0x70dc20: F -> a, T -> alpha (no gyroscopic term for vehicles).
    body_.S.a = body_.F * body_.invMass;
    body_.S.alpha = body_.I1invW * body_.T;
}

// ---- body forces 0x522830 --------------------------------------------------------------------

void FuelVehicle::aeroDrag() {  // 0x530d20
    BodyState& S = body_.S;
    const float k = 200.0f * body_.boundR * body_.boundR;
    float linF = allWheelsLoaded() ? P(93) : P(94);
    float angF = lerpf(P(100), P(98), clampf(speed_ / std::max(P(99), 1e-6f), 0, 1));
    if (wrecked_) linF *= 0.01f, angF *= 0.01f;
    // Coefficients applied per body axis (U2: Inferred frame).
    const Vec3 lc{P(95), P(96), P(97)};
    const Vec3 ac{P(101), lerpf(P(102), P(103), std::fabs(st_.chB)), P(104)};
    const Vec3 vw = rotate(S.q, mulc(lc, unrotate(S.q, S.v)));
    const float lv = len(vw);
    if (lv > 1e-6f) body_.addForceAt(vw * (-k * linF * std::min(lv, 100.0f) * std::min(lv, 100.0f) / lv), S.x);
    const Vec3 ww = rotate(S.q, mulc(ac, unrotate(S.q, S.w)));
    const float lw = len(ww);
    if (lw > 1e-6f) body_.T += ww * (-k * angF * std::min(lw, 10.0f) * std::min(lw, 10.0f) / lw);
}

void FuelVehicle::antiRollLift() {  // 0x52c900
    if (nWheels_ < 4) return;
    const int pairs[3][2] = {{0, 2}, {1, 3}, {4, 5}};
    const float gain[3] = {P(87), P(88), P(87)};
    const Vec3 upW = up();
    for (int i = 0; i < 3; ++i) {
        const int a = pairs[i][0], b = pairs[i][1];
        if (b >= nWheels_) continue;
        const float La = wheels_[size_t(a)].load, Lb = wheels_[size_t(b)].load;
        const float d = La - Lb;
        const int w = d >= 0 ? a : b;
        if (!(wheels_[size_t(w)].load > 0)) continue;
        const float f = body_.mass * 0.1f * std::min(gain[i] * std::fabs(d), 9.81f);
        body_.addForceAt(upW * f, wheels_[size_t(w)].hubW);
    }
}

void FuelVehicle::bodyForces() {
    BodyState& S = body_.S;
    const float m = body_.mass;
    // 1. Drowned: strong lift (P288 < 0).
    if (wrecked_ == 2) body_.addForceAt({0, P(288) * m * -4.0f, 0}, S.x);
    // 2. Aero drag.
    aeroDrag();
    // 4. Gravity (0x530cb0): per-vehicle gravity, P288 within P287 s of the last ground contact, else P289.
    const bool nearGround = airtime_ < P(287);
    body_.addForceAt({0, m * (nearGround ? P(288) : P(289)), 0}, S.x);
    // 5. Downforce.
    body_.addForceAt(up() * (-std::max(0.0f, dot(S.v, fwd())) * m * (nearGround ? P(291) : P(292))), S.x);
    // 6. Anti-roll lift.
    antiRollLift();
    // 7. Water / mud drag (water depth from the world's water, see Environment::waterDepthAt).
    if (!wrecked_) {
        inWater_ = true;
        const int n = player_ ? nWheels_ : 1;
        for (int w = 0; w < n; ++w) {
            const float t = waterDepth_[size_t(w)];
            if (t < P(581)) inWater_ = false;
            const Vec3 hub = player_ ? wheels_[size_t(w)].hubL : Vec3{};
            const Vec3 vp = S.v + rotate(S.q, cross(unrotate(S.q, S.w), hub));
            const float s = len(vp);
            if (s <= 0.001f) continue;
            const float N = player_ ? 1.0f : float(nWheels_);
            const Vec3 at = player_ ? wheels_[size_t(w)].hubW : S.x;
            if (t != 0) {
                maxWaterDepth_ = std::max(maxWaterDepth_, t);
                const float k = t > P(581) ? 0.2f * P(579) : (t > P(580) ? 0.2f * P(577) : 0.0f);
                body_.addForceAt(vp / s * (-k * m * N * std::sqrt(s)), at);
            }
            const GroundBlock& C = C_[size_t(w)];
            const Vec3 vh = vp / s;
            const float Y = (P(154) * C.soft + P(130) * C.hard) * std::fabs(dot(vh, lat())) * C.sideSlow +
                            (P(155) * C.soft + P(131) * C.hard) * std::fabs(st_.chB) * C.rolling;
            if (Y != 0)
                body_.addForceAt(vp * (-0.03f * Y * clampf(0.33f * wheels_[size_t(w)].load, 0, 1) * m * N), at);
        }
    }
    // 8. Slide-steer couple (Part V2d): a lateral force at each hub, opposite front and rear.
    if (!airborneLong_ && std::fabs(st_.accel) > 1e-4f) {
        const Vec3 n = refUp_;
        const Vec3 l = lat();
        const Vec3 dir = normalize(l - n * dot(n, l));
        const float sg = gear_ >= 0 ? 1.0f : -1.0f;
        for (int w = 0; w < nWheels_; ++w) {
            const bool f = front(w);
            const float k = lerpf(f ? P(266) : P(267), f ? P(268) : P(269), clampf(0.03f * speed_, 0, 1));
            if (k < 0.001f) continue;
            const WheelRec& r = wheels_[size_t(w)];
            const float F = -r.hubL.z * m * st_.accel * sg * k;
            Vec3 at = r.hubW;
            at -= n * dot(at - S.x, n);  // onto the plane through the CoM, normal to the reference up
            body_.addForceAt(dir * F, at);
        }
    }
    // 9. Guided target forces (AI/script, U5): not used; the port's AI drives through the inputs.
}

// ---- driveline 0x5211e0 ----------------------------------------------------------------------

void FuelVehicle::driveline() {
    BodyState& S = body_.S;
    S.auxAcc[0] = 0;
    for (int w = 0; w < nWheels_; ++w) S.auxAcc[1 + w] = 0, dOmega_[w] = 0;
    const float rpm = S.auxVel[0];
    const float r = ratio(gear_);
    const float clutch = std::min(clutchFraction(), notBraking());  // 0x525650
    // Engine 0x520560.
    {
        float thr = engThrottle_;
        if (allWheelsLoaded()) thr *= 1 + std::max(0.0f, fwd().y) * P(328);
        if (!player_) thr *= 1.25f;
        const float engAcc = thr * boostMul_ * engineTorque(rpm) * gearMul(gear_) / P(323) - (1 - engThrottle_) * rpm * P(329);
        S.auxAcc[0] += engAcc;
        if (gear_ != 0 && P(377) >= 0.5f)
            for (int w = 0; w < nWheels_; ++w)
                if (driven(w)) dOmega_[w] += clutch * P(378) * engAcc * r * 0.001f;
    }
    // Clutch coupling 0x520b00.
    if (gear_ != 0) {
        const Vec3 vl = normalize(unrotate(S.q, S.v));
        const float x = std::fabs(vl.x);
        for (int w = 0; w < nWheels_; ++w) {
            if (!driven(w)) continue;
            const float rr = wheelR(w);
            const float s = (S.auxVel[1 + w] * 60.0f / (kTwoPi * rr) - S.auxVel[0] / r) * 0.04f * clutch;
            S.auxAcc[0] += P(379) * s / (P(323) * std::fabs(r));
            const float aw = -(std::fabs(r) * ((1 - x) * P(380) + x * P(381)) / wheelInertia(w) * s) * rr * 0.10471976f;
            S.auxAcc[1 + w] += aw;
            if (P(377) < 0.5f) dOmega_[w] += aw;
        }
    }
    // Brakes and rolling resistance 0x520fd0.
    const bool hbm = handbrakeMode();
    const float brake = brakeEff();
    for (int w = 0; w < nWheels_; ++w) {
        const bool f = front(w);
        const float Pb = f ? P(395) : P(396), I = f ? P(53) : P(54), hb = f ? 0.0f : P(397);
        float B = (gear_ != 0 ? brake : 1.0f) * Pb;
        const GroundBlock& C = C_[size_t(w)];
        const float om = S.auxVel[1 + w];
        const float sg = clampf(0.1f * om, -1, 1);
        const float load = std::min(100 * wheels_[size_t(w)].load, 1.0f);
        const float boost = 0;  // boost is dead data (P633 = 0)
        const float a = -((hbm ? hb : 0.0f) / I + B / I) * sg - (P(129) * C.hard + P(153) * C.soft) * C.rolling * load * om -
                        ((1 - boost) * P(91) + boost * P(92)) * om * std::fabs(om) * load;
        S.auxAcc[1 + w] += a;
        dOmega_[w] += a;
    }
    // Tyre reaction or kinematic rolling 0x520d40.
    for (int w = 0; w < nWheels_; ++w) {
        const WheelRec& rec = wheels_[size_t(w)];
        const float load = std::min(100 * rec.load, 1.0f);
        // Direction: the Ackermann vectors for wheels 0 and 2 (+0x1000 / +0x1018, passed in EDX by 0x5211e0), else Z.
        const Vec3 dw = rotate(S.q, w == 0 ? st_.dir0 : (w == 2 ? st_.dir2 : Vec3{0, 0, 1}));
        if (slipRatio(w) > 1 && driven(w)) {
            const float a = P(68) / wheelInertia(w) * -(dot(rec.fric, dw) * body_.mass) * load * 0.03f;
            const float bf = wheelBrakeFactor(w) - 0.5f;
            S.auxAcc[1 + w] += (1 - 2 * clampf(bf, 0, 1)) * a;
        } else {
            S.auxAcc[1 + w] = 0;
            S.auxVel[1 + w] = dot(body_.pointVel(rec.hubW), dw);
        }
    }
}

// ---- integration (mode 3, 0x684ed6-0x6850b1) ---------------------------------------------------

void FuelVehicle::integrate(float h) {
    BodyState& S = body_.S;
    const BodyState& P0 = body_.P;
    S.a = (S.a + P0.a) * 0.5f;
    S.alpha = (S.alpha + P0.alpha) * 0.5f;
    for (int i = 0; i < S.nAux; ++i) S.auxAcc[i] = 0.5f * (S.auxAcc[i] + P0.auxAcc[i]);
    integrateVelocity(S, h);
    // 0x680db0
    const float lv = len(S.v), lw = len(S.w);
    if (lv > body_.maxSpeed) S.v *= body_.maxSpeed / lv;
    if (lw > body_.maxSpeed) S.w *= body_.maxSpeed / lw;
    body_.N = S;
    S.v = (S.v + P0.v) * 0.5f;
    S.w = (S.w + P0.w) * 0.5f;
    for (int i = 0; i < S.nAux; ++i) S.auxVel[i] = 0.5f * (S.auxVel[i] + P0.auxVel[i]);
    integratePosition(S, h * body_.timeScale, body_.dv2, body_.dw2);  // 0x714d20
    body_.avgV = S.v;
    body_.avgW = S.w;
    S.v = body_.N.v;
    S.w = body_.N.w;
    for (int i = 0; i < S.nAux; ++i) S.auxVel[i] = body_.N.auxVel[i];
}

void FuelVehicle::postSubstep() {  // 0x5212d0 + 0x520790
    BodyState& S = body_.S;
    S.auxVel[0] = clampf(std::max(S.auxVel[0], 500.0f), P(325), P(324));
    for (int w = 0; w < nWheels_; ++w) S.auxVel[1 + w] = clampf(S.auxVel[1 + w], -83.34f, 83.34f);
    const float clutch = std::min(clutchFraction(), notBraking());
    if (gear_ != 0 && P(377) >= 0.5f && clutch >= 0.99f) {
        const float r = ratio(gear_);
        const float x = std::fabs(normalize(unrotate(S.q, S.v)).x);
        float Ww = 0, mean = 0;
        int n = 0;
        for (int w = 0; w < nWheels_; ++w)
            if (driven(w)) Ww += wheelInertia(w), mean += S.auxVel[1 + w] * 60.0f / (kTwoPi * wheelR(w)), ++n;
        if (n > 0) {
            Ww *= P(379);
            mean /= float(n);
            const float We = std::fabs(r) * P(323) * lerpf(P(380), P(381), x) * 100.0f;
            const float wc = (Ww * mean + We * S.auxVel[0] / r) / (We + Ww);
            S.auxVel[0] = wc * r;
            for (int w = 0; w < nWheels_; ++w)
                if (driven(w)) S.auxVel[1 + w] = kTwoPi * wheelR(w) * wc / 60.0f;
        }
    }
}

// ---- island step ----------------------------------------------------------------------------

void stepVehicles(const std::vector<FuelVehicle*>& vehicles, Environment& env, float dt) {
    dt = std::max(dt, 0.001f);
    for (FuelVehicle* v : vehicles) {
        v->body().impactSpeed = 0;
        v->body().scrapeSpeed = 0;
        v->preStep(dt, env);
    }
    float remaining = dt;
    std::vector<Contact*> imp;
    while (remaining > 0.0001f) {
        const float h = std::min(remaining, 0.01f);
        for (FuelVehicle* v : vehicles) v->wheelUpdate();
        for (FuelVehicle* v : vehicles) v->generateContacts(env, vehicles);
        for (FuelVehicle* v : vehicles) v->forces();
        for (FuelVehicle* v : vehicles) v->driveline();
        for (FuelVehicle* v : vehicles) v->integrate(h);
        // Impulse pass 0x683ee0 (<= 5 iterations, tolerance 0.001).
        imp.clear();
        for (FuelVehicle* v : vehicles) {
            v->body().updateWorldInertia();
            for (Contact& c : v->contacts())
                if (prepareImpulse(c)) imp.push_back(&c);
        }
        if (!imp.empty()) {
            for (int it = 0; it < 5; ++it) {
                std::vector<std::pair<Vec3, Vec3>> before;
                for (FuelVehicle* v : vehicles) before.push_back({v->body().S.v, v->body().S.w});
                for (Contact* c : imp) applyImpulses(*c);
                bool done = true;
                for (size_t i = 0; i < vehicles.size(); ++i) {
                    RigidBody& b = vehicles[i]->body();
                    const float lv = len(b.S.v), lw = len(b.S.w);
                    if (lv > b.maxSpeed) b.S.v *= b.maxSpeed / lv;
                    if (lw > b.maxSpeed) b.S.w *= b.maxSpeed / lw;
                    const float change = len(b.S.v - before[i].first) + len(b.S.w - before[i].second) * b.boundR;
                    if (change > 0.001f) done = false;
                }
                if (done) break;
            }
        }
        for (FuelVehicle* v : vehicles) v->postSubstep();
        remaining -= h;
    }
    for (FuelVehicle* v : vehicles) v->postStep(dt, env);
}

}  // namespace fuel::fv
