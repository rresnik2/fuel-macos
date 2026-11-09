// FUEL's vehicle model, engine side and per-substep vehicle hooks. See fuel_vehicle.hpp and
// ~/re-work/fuel/notes/p23-vehicle-physics.md (sections 2-8, Part E) for the decode.
#include "fuel_vehicle.hpp"
#include "nitro.hpp"  // [nitro]

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
    body_.massP3 = P(3);
    if (P_.size() > 593) body_.crashP = &P_[585];  // the crash meter 0x51ee90 (P585..P593)
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
    // +0x404 (0x712b60, called once from 0x70e210): starts at 0.01 [0x99614c] and takes the largest distance from
    // the centre of mass of every sphere's far side (wheel spheres included: they are proxy spheres here) and of
    // every box corner (0x7426b0: the 8 corners). The breadcrumbs' blocking sphere is this + 2 (0x527bf0).
    extentR_ = 0.01f;
    for (const auto& s : proxy_.spheres) extentR_ = std::max(extentR_, len(s.c - com_) + s.r);
    for (const auto& b : proxy_.boxes)
        for (int k = 0; k < 8; ++k) {
            const Vec3 c{b.c.x + ((k & 1) ? b.half.x : -b.half.x), b.c.y + ((k & 2) ? b.half.y : -b.half.y), b.c.z + ((k & 4) ? b.half.z : -b.half.z)};
            extentR_ = std::max(extentR_, len(c - com_));
        }
    body_.I1 = I * Mat3::diag(P(4), P(5), P(6));
    body_.I2 = I * Mat3::diag(P(7), P(8), P(9));
    body_.I1inv = body_.I1.inverse();
    body_.I2inv = body_.I2.inverse();
    body_.updateWorldInertia();
}

void FuelVehicle::buildWheels() {
    // 0x52def0: hub = proxy sphere (+ hub offset 0x5d7a90, not applied: hubRest) - CoM; sphere radius =
    // proxy radius + P55 (front) | P56 (rear); the tyre material (front odd parameter, rear even).
    wheels_.assign(size_t(nWheels_), WheelRec{});
    for (int w = 0; w < nWheels_; ++w) {
        WheelRec& r = wheels_[size_t(w)];
        const auto& s = proxy_.spheres[size_t(wheelSphere_[size_t(w)])];
        const bool f = front(w);
        auto pp = [&](int frontParam) { return P(f ? frontParam : frontParam + 1); };
        r.hubL = hubRest(w) - com_;
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

Vec3 FuelVehicle::hubOffset(int w) const {
    // 0x5d7a90 switch (EDX = wheel): 0 (P57, P58, P59), 1 (P60, P61, P62), 2 (-P57, P58, P59), 3 / 4 / 5 (-P60, P61, P62);
    // the decompiled axes are (0,1,0) [0xa06320], (-1,0,0) [0xa06344], (0,0,1) [0xa06350].
    if (w < 0 || w > 5) return {};
    const bool rear = w == 1 || w >= 3;
    const float x = P(rear ? 60 : 57), y = P(rear ? 61 : 58), z = P(rear ? 62 : 59);
    return {(w == 0 || w == 1) ? x : -x, y, z};
}

float FuelVehicle::visualCamber(int w, float c) const {
    if (P(66) == 0) return 0;  // asm 0x5cb320: fucomp with 0
    float h = c + P(67);
    if (w > 1) h = -h;        // asm 0x5cb331: cmp wheel, 1; jle
    return std::atan2(h, P(66));  // 0x40f550 = fpatan(h, P66)
}

// Port choice (P81, notes p81-physics-big.md open W2): the hub stays at the proxy sphere. 0x52def0 / 0x5c8620 add
// hubOffset(w) (VehiclePP 58 = 0.1 .. 0.4 for 42 vehicles), but with this physics the body then rides that much
// lower and the full-size proxy box drags (BUGGY_ALL_1: 4 chassis contacts per frame, top speed 164 -> 136 km/h),
// and FUEL's own drawn-wheel height (0x5caf40, W1) disagrees with this contact geometry by about the same amount.
Vec3 FuelVehicle::hubRest(int w) const {
    const Vec3 c = proxy_.spheres[size_t(wheelSphere_[size_t(w)])].c;
    if (!hubShifted_) return c;
    // Port deviation (upgrade plan, suspension lift): the wheel centre lower by the lift (the body rides higher) and
    // out by the axle's track widening, away from the model's centre line.
    const float wide = front(w) ? widenF_ : widenR_;
    return {c.x + (c.x >= 0 ? wide : -wide), c.y - lift_, c.z};
}
void FuelVehicle::setHubShift(float lift, float widenFront, float widenRear) {
    lift_ = lift, widenF_ = widenFront, widenR_ = widenRear;
    hubShifted_ = lift != 0 || widenFront != 0 || widenRear != 0;
    for (int w = 0; w < nWheels_; ++w) wheels_[size_t(w)].hubL = hubRest(w) - com_;
}

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
    return governedTorque(rpm, powerScale * torqueScale_);
}
// 0x524c70's curve x P327, and the port's extra torque on top of it: scale s (1 = FUEL, returned bit for bit), the part
// above FUEL's governed by gov_ (truck-power; notes truck-power.md).
float FuelVehicle::governedTorque(float rpm, float s) const {
    const float f = clampf(rpm / (P(324) * rpmStretch_), 0, 1) * 10.0f;  // [nitro] x 1 without a boost
    const int i = int(std::floor(f));
    const int j = std::min(i + 1, 10);
    const float t = f - float(i);
    const float T = ((1 - t) * P(338 + i) + P(338 + j) * t) * P(327);
    if (s == 1.0f && !upgraded_) return T;
    if (upgraded_ && governorOn)  // [upgrades U1] FUEL's own part x the traction hold (1 unless the driven wheels slide)
        return T * (hold_ + (s - 1) * gov_);
    return T * (1 + (s - 1) * (governorOn ? gov_ : 1.0f));
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
float FuelVehicle::meanWheelSpeed() const {  // 0x5257b0: every wheel, rim speed with the dead zone, / the wheel count
    float sum = 0;
    for (int w = 0; w < nWheels_; ++w) {
        float x = body_.S.auxVel[1 + w];
        if (std::fabs(x) < body_.S.auxDead[1 + w]) x = 0;
        sum += x;
    }
    return sum / float(nWheels_);
}
bool FuelVehicle::drivenWheelsSlidingEnough() const {  // 0x5244d0: sliding (loaded) driven wheels >= min(driven wheels, 2)
    int driven_ = 0, sliding = 0;
    for (int w = 0; w < nWheels_; ++w) {
        if (!driven(w)) continue;
        ++driven_;
        if (wheelSliding(w)) ++sliding;
    }
    return sliding >= std::min(driven_, 2);
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
bool FuelVehicle::slidingMajority() const {  // 0x52f100: wheel 1; with > 2 wheels also 3; with > 5 also 4 and 5
    int n = wheelSliding(1 % std::max(nWheels_, 1)) ? 1 : 0, need = 1;
    if (nWheels_ > 2) {
        n += wheelSliding(3) ? 1 : 0, need = 2;
        if (nWheels_ > 5) n += (wheelSliding(4) ? 1 : 0) + (wheelSliding(5) ? 1 : 0), need = 4;
    }
    return n >= need;
}
float FuelVehicle::maxWheelBumps() const {
    float m = 0;
    for (int w = 0; w < nWheels_; ++w) m = std::max(m, C_[size_t(w)].bumpsWheel);
    return m;
}
float FuelVehicle::meanWheelBumpsRot() const {
    float s = 0;
    for (int w = 0; w < nWheels_; ++w) s += C_[size_t(w)].bumpsRot;
    return nWheels_ ? s / float(nWheels_) : 0.0f;
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
// 0x51ee90 crash meter (the last of 0x51f6c0's accept hooks): wheel contacts only when bottomed out (depth past
// R[0x3e]); with an approach speed >= 0. The impact direction is (p - x) normalised, or -n when (p - x).y > -n.y;
// a, b, c = |dir . fwd|, |dir . lat|, |dir . up|; e = (max(rel . dir, 0) - (P586 a + P588 b + P590 c)) x f (0x51ee20);
// a hard hit (|n.y| < 0.707 or approach > 15 m/s) with e > 0 raises +0x138c to (P587 a + P589 b + P591 c) e 100/P585 x 20
// and records the direction in the body frame (+0x1060); the scrape +0x1390 = max(, (tangential - 100/P585 x P592) P593 x 20).
}  // namespace

void crashMeter(const Contact& c, RigidBody& body, const Vec3& rel) {
    const float* P = body.crashP;  // P[0] = P585 ... P[8] = P593
    if (P == nullptr) return;
    if (c.wheel != nullptr && !(c.m.maxDepth < c.depth)) return;
    if (!(c.approach >= 0)) return;
    float f = 1;
    if (c.b) {
        const float mo = c.b->massP3 > 0 ? c.b->massP3 : c.b->mass;
        f = 1 - body.massP3 / (mo + body.massP3);
    }
    const bool hard = std::fabs(c.n.y) < 0.707f || std::fabs(c.approach) > 15.0f;
    Vec3 dir = c.p - body.S.x;
    const float dl = len(dir);
    dir = dl > 0 ? dir / dl : dir;  // 0x410050(1.0)
    if (-c.n.y < dir.y) dir = -c.n;
    const Vec3 fw = rotate(body.S.q, {0, 0, 1}), lt = rotate(body.S.q, {1, 0, 0}), up = rotate(body.S.q, {0, 1, 0});
    const float a = std::fabs(dot(dir, fw)), b = std::fabs(dot(dir, lt)), cc = std::fabs(dot(dir, up));
    const float rv = std::max(dot(rel, dir), 0.0f);
    const float s = 100.0f / P[0];
    const float e = (rv - (P[1] * a + P[3] * b + P[5] * cc)) * f;
    if (hard && e > 0) {
        const float v = (P[6] * cc + P[4] * b + P[2] * a) * e * s * 20.0f;
        if (body.impactMeter < v) {
            body.impactMeter = v;
            body.impactDirL = unrotate(body.S.q, dir);
        }
    }
    const float t = c.tangential - s * P[7];
    if (t > 0) body.scrapeMeter = std::max(body.scrapeMeter, t * P[8] * 20.0f);
}

// P87: the debris branch of Vehicle_ImpactFx 0x51e9e0 (asm 0x51e9e0..0x51ed70) for `self` (ESI) and contact R
// (EDI). The FX stats (+0x1408..+0x1410, +0xf1c, +0xf2c) and the scrape sound 0x5d07f0 are elsewhere (soundImpact).
static thread_local const char* tContactTag = "";  // P87 trace: which gather made the contact
void impactDebris(const Contact& c, RigidBody& self, const Vec3& rel, const RigidBody* bodyA) {
    if (self.crashP == nullptr) return;  // not a vehicle (no 0x234 hook)
    const float r = len(rel);            // |R[0x10..0x12]| (0x70ce10: vA(p) + R[8..] surface velocity - vB(p))
    const bool counts = 5.0f < r;        // (statistics only)
    if (!self.fxDriver) {                // 0x51e9e3: +0x142b
        if (counts) ++self.fxRefused[0];
        return;
    }
    if (self.fxRaceState == 6 || self.fxRaceState == 7) {  // 0x51ea20: race won / lost
        if (counts) ++self.fxRefused[1];
        return;
    }
    if (c.wheel != nullptr) {  // R[0x39] = the wheel record (0 for sphere / box contacts)
        if (c.depth < c.m.maxDepth) {  // 0x51ea79: R[0xd8] < R[0xf8], the tyre still springs
            if (counts) ++self.fxRefused[2];
            return;
        }
        // 0x51ea90: the hub (rec+0xf0) seen from the contact point in the body frame (conj(+0x7f0), 0x40f130 /
        // 0x4db890): a bottomed wheel touching below (or above) its hub throws nothing unless rec+0xd4 (the y of
        // the slip velocities 0x70f490 summed in the last substep) is below -P643 (PP+0xa08).
        const Vec3 d = unrotate(self.S.q, c.wheel->hubW - c.p);
        if (std::sqrt(d.x * d.x + d.z * d.z) < std::fabs(d.y) && -self.fxWheelDrop < c.wheel->slipGather.y) {
            if (counts) ++self.fxRefused[2];
            return;
        }
    }
    if (self.fxClass == 1 || self.fxClass == 3) {
        // 0x51ebba (BIKE / QUAD): the vehicle node's up (vt+0xb4(0) -> node +0x10 quaternion x 0xa06320) against
        // the probe normal +0x1030 > 0.3 [0x9dbfa4] and the contact normal along it > 0.707 [0x9dc0c8]: skip.
        const Vec3 up = rotate(self.S.q, {0, 1, 0});
        if (0.3f < dot(self.fxProbeNormal, up) && 0.707f < dot(self.fxProbeNormal, c.n)) {
            if (counts) ++self.fxRefused[3];
            return;
        }
    }
    // ---- PK23: the scrape sparks (asm 0x51ed72: 10 [0x996494] <= R[0x34], the tangential speed) run after the debris
    // block whatever it did; R[0x14..0x16] = rel - n (n . rel) (0x70ce10).
    struct SparkAfter {
        const Contact& c;
        RigidBody& self;
        const Vec3& rel;
        const RigidBody* a;
        ~SparkAfter() {
            if (!(10.0f <= c.tangential) || self.sparkCalls.size() >= RigidBody::kMaxSparkCalls) return;
            RigidBody::SparkCall s;
            s.a = uint64_t(reinterpret_cast<uintptr_t>(a)), s.b = uint64_t(reinterpret_cast<uintptr_t>(c.b));
            s.point = c.p;
            s.tangent = rel - c.n * dot(rel, c.n);
            s.q = self.S.q;
            self.sparkCalls.push_back(s);
        }
    } sparkAfter{c, self, rel, bodyA};
    // ---- PK23 end
    if (!(self.fxTimer < 0)) {  // 0x51ed0a: fldz; fcomp +0x1340
        if (counts) ++self.fxRefused[4];
        return;
    }
    if (!counts) return;  // 0x51ed19: 5 [0x9dbf8c] < |rel|
    const int n = std::min(int(r * 0.5f), 20);  // 0x51ed28: x 0.5 [0x9960c0], _ftol 0x8e01d0, at most 0x14
    static const bool trace = std::getenv("FUEL_P87_TRACE") != nullptr;
    if (trace) {
        const Vec3 dl = c.wheel ? unrotate(self.S.q, c.wheel->hubW - c.p) : Vec3{};
        std::printf("p87 impact [%s]: %s%s depth %.3f (bottom %.3f) n (%.2f, %.2f, %.2f) |rel| %.1f approach %.1f tangential %.1f, hub-p local "
                    "(%.2f, %.2f, %.2f) slip.y %.2f, at (%.1f, %.1f, %.1f) -> %d pieces\n",
                    tContactTag, c.wheel ? "wheel" : "body", c.b ? " vs vehicle" : "", c.depth, c.m.maxDepth, c.n.x, c.n.y, c.n.z, r, c.approach, c.tangential,
                    dl.x, dl.y, dl.z, c.wheel ? c.wheel->slipGather.y : 0.0f, c.p.x, c.p.y, c.p.z, n);
    }
    if (0 < n) {
        self.debrisPieces = n;
        self.debrisAt = c.p;  // R[0..2]: every piece starts at the contact point
        ++self.debrisSerial;
    }
    self.fxTimer = 1.0f;  // 0x51ed66
}

namespace {

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
    if (c.wheel == nullptr || c.depth > c.m.maxDepth) {  // 0x51e9e0: wheel contacts only when bottomed out
        const float mf = c.b ? 1 - body.mass / std::max(body.mass + c.b->mass, 1e-6f) : 1.0f;  // 0x51ee20
        if (c.tangential * mf > body.soundImpactKey) {
            body.soundImpactKey = c.tangential * mf;
            body.soundImpact = c.n * (mf * c.approach);
        }
    }
    // P87: 0x51e9e0's debris request (0x52ee60). Contact_Add 0x70ce10 runs the filter hook 0x234 of A, then of B
    // (+0xebf, both vehicles), so a vehicle-vehicle contact reaches 0x51e9e0 of both bodies with the same record.
    impactDebris(c, body, rel, &body);
    if (c.b != nullptr && c.b != &body) impactDebris(c, *c.b, rel, &body);
    body.impactSpeed = std::max(body.impactSpeed, c.approach);
    crashMeter(c, body, rel);
    out.push_back(c);
    return true;
}

// 0x725db0: two bodies' contact materials combined for a body-body contact (R[0x3c..]): the larger compliance,
// preload, friction peaks and load terms, the smaller bias, bottom-out depth, damping and exponent; the friction
// set 1 coefficients 0, 2-4 and 7 by the larger, or by the smaller when both carry the min-friction flag (+0x21).
ContactMat combineMaterials(const ContactMat& a, const ContactMat& b) {
    ContactMat m = a;
    m.c = std::max(a.c, b.c);
    m.bias = std::min(a.bias, b.bias);
    m.maxDepth = std::min(a.maxDepth, b.maxDepth);
    m.preload = std::max(a.preload, b.preload);
    m.dampC = std::min(a.dampC, b.dampC);
    m.dampR = std::min(a.dampR, b.dampR);
    m.quadC = std::min(a.quadC, b.quadC);
    m.quadR = std::min(a.quadR, b.quadR);
    m.set1[5] = std::max(a.set1[5], b.set1[5]);
    m.set1[6] = std::max(a.set1[6], b.set1[6]);
    m.maxN = std::max(a.maxN, b.maxN);
    m.loadFloor = std::max(a.loadFloor, b.loadFloor);
    m.loadBlend = std::max(a.loadBlend, b.loadBlend);
    m.extraDepth = std::max(a.extraDepth, b.extraDepth);
    m.loadExp = std::min(a.loadExp, b.loadExp);
    const bool both = a.minFriction && b.minFriction;
    for (int i : {0, 2, 3, 4, 7}) m.set1[i] = both ? std::min(a.set1[i], b.set1[i]) : std::max(a.set1[i], b.set1[i]);
    return m;
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

// The chassis spheres (proxy spheres that are not wheels, lowered by P10: 0x71c5f0 -> +0xdbc) and boxes in the world.
void FuelVehicle::chassisSpheres(std::vector<std::pair<Vec3, float>>& out) const {
    const BodyState& S = body_.S;
    const Vec3 lower{0, -P(10), 0};
    for (size_t i = 0; i < proxy_.spheres.size(); ++i) {
        if (std::find(wheelSphere_.begin(), wheelSphere_.end(), int(i)) != wheelSphere_.end()) continue;
        out.push_back({S.x + rotate(S.q, proxy_.spheres[i].c - com_ + lower), proxy_.spheres[i].r});
    }
}
void FuelVehicle::modelShapes(std::vector<std::pair<Vec3, float>>& spheres, std::vector<ProxyShapes::Box>& boxes) const {
    const Vec3 lower{0, -P(10), 0};
    for (size_t i = 0; i < proxy_.spheres.size(); ++i) {
        if (std::find(wheelSphere_.begin(), wheelSphere_.end(), int(i)) != wheelSphere_.end()) continue;
        spheres.push_back({proxy_.spheres[i].c + lower, proxy_.spheres[i].r});
    }
    for (const auto& b : proxy_.boxes) boxes.push_back(b);
}
void FuelVehicle::chassisBoxes(std::vector<Obb>& out) const {
    const BodyState& S = body_.S;
    for (const auto& b : proxy_.boxes) {
        Obb ob;
        ob.ax[0] = rotate(S.q, {1, 0, 0}), ob.ax[1] = rotate(S.q, {0, 1, 0}), ob.ax[2] = rotate(S.q, {0, 0, 1});
        ob.c = S.x + rotate(S.q, b.c - com_);
        ob.h = b.half;
        ob.r = b.r > 0 ? b.r : len(b.half);
        out.push_back(ob);
    }
}

void FuelVehicle::generateContacts(Environment& env, const std::vector<FuelVehicle*>& others) {
    contacts_.clear();
    body_.updateWorldInertia();
    body_.slidingContacts = 0;
    const BodyState& S = body_.S;
    const Vec3 upW = rotate(S.q, kUp);
    std::vector<const Tri*> tris;
    std::vector<StaticHit> hits;
    // The island's static scene (0x71dd80): gathered once around the body's broadphase sphere (+0x410 = +0x408
    // + 0.5).
    if (P_.size() > 593) body_.crashP = &P_[585];  // (re-pointed: the vehicle may have been copied)
    body_.fxDriver = player_;  // P87: 0x51e9e0's inputs
    body_.fxProbeNormal = probeNormal_;
    body_.fxWheelDrop = P(643);
    StaticScene& scene = scene_;
    scene.clear();
    if (env.gatherStatics) env.gatherStatics(S.x, body_.boundR + 0.5f, scene);
    // Wheels (0x723400).
    tContactTag = "wheel";
    for (int w = 0; w < nWheels_; ++w) {
        WheelRec& r = wheels_[size_t(w)];
        r.maxDepthSeen = r.load = r.compRatio = r.sumFn = r.sumDen = 0;
        r.slipGather = r.slip;  // P87: FUEL clears +0xd0 after the gather (0x70f490), the port before it
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
        // Static boxes 0x71a9e0, sphere objects 0x71b1b0, mesh triangles 0x769560 (as the terrain's).
        for (const Obb& ob : scene.boxes) {
            Contact ct;
            if (!sphereObb(ob, c, rad, ct.p, ct.n, ct.depth)) continue;
            ct.a = &body_;
            ct.surface = 0;
            attachWheel(ct, r, upW);
            addContact(ct, body_, S.q, wrecked_, P(284), P(285), contacts_);
        }
        for (const auto& [sc, sr] : scene.spheres) {
            Contact ct;
            if (!sphereSphere(sc, sr, c, rad, ct.p, ct.n, ct.depth)) continue;
            ct.a = &body_;
            ct.surface = 0;
            attachWheel(ct, r, upW);
            addContact(ct, body_, S.q, wrecked_, P(284), P(285), contacts_);
        }
        for (const Tri* t : scene.tris) {
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
        // P50: no trunks for the wheels. 0x723400 calls 0x74ee80 with mode 4 (water floor only); trees 0x785200 and
        // bushes 0x788ab0 need mode bit 2, which only the chassis spheres (0x723b80) and boxes (0x74ead0) pass.
        if (env.sphereStatics && false) {  // tree and bush trunks
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
        tContactTag = "chassis sphere";
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
        // 0x723b80: static boxes, sphere objects, mesh triangles (0x7260a0(0)).
        for (const Obb& ob : scene.boxes) {
            Contact ct;
            if (!sphereObb(ob, c, s.r, ct.p, ct.n, ct.depth)) continue;
            ct.a = &body_, ct.m = chassis;
            addContact(ct, body_, S.q, wrecked_, P(284), P(285), contacts_);
        }
        for (const auto& [sc, sr] : scene.spheres) {
            Contact ct;
            if (!sphereSphere(sc, sr, c, s.r, ct.p, ct.n, ct.depth)) continue;
            ct.a = &body_, ct.m = chassis;
            addContact(ct, body_, S.q, wrecked_, P(284), P(285), contacts_);
        }
        for (const Tri* t : scene.tris) {
            Contact ct;
            if (!sphereTriangle(*t, c, s.r, ct.depth, ct.n, ct.p)) continue;
            ct.a = &body_;
            ct.m = chassis;
            ct.surface = t->surface;
            addContact(ct, body_, S.q, wrecked_, P(284), P(285), contacts_);
        }
        if (env.sphereStatics) {  // trunks
            hits.clear();
            env.sphereStatics(c, s.r, hits);
            for (const StaticHit& h : hits) {
                Contact ct;
                ct.p = h.p, ct.n = h.n, ct.depth = h.depth, ct.a = &body_, ct.m = chassis;
                addContact(ct, body_, S.q, wrecked_, P(284), P(285), contacts_);
            }
        }
    }
    // Boxes (0x724120): the body's box records (axes = the body's, centre, half extents, radius +0xa0).
    std::vector<Obb>& boxes = boxes_;
    tContactTag = "chassis box";
    boxes.clear();
    chassisBoxes(boxes);
    if (!boxes.empty()) {
        const Vec3 bc = S.x;  // the body's bound sphere (+0xc10 = position, +0xc1c = +0x408)
        const float br = body_.boundR;
        // Trunks (0x74ead0; port: Jolt).
        if (env.boxStatics)
            for (size_t i = 0; i < boxes.size(); ++i) {
                hits.clear();
                env.boxStatics(boxes[i].c, S.q, boxes[i].h, hits);
                for (const StaticHit& h : hits) {
                    Contact ct;
                    ct.p = h.p, ct.n = h.n, ct.depth = h.depth, ct.a = &body_, ct.m = chassis;
                    addContact(ct, body_, S.q, wrecked_, P(284), P(285), contacts_);
                }
            }
        // Static boxes: 0x7266a0 (the bound sphere), 0x726160 (the pair), 0x763890.
        Vec3 n, pts[20];
        float dep[20];
        for (const Obb& sb : scene.boxes) {
            if (!obbSphereCull(sb, bc, br)) continue;
            for (const Obb& ob : boxes) {
                if (!obbObbCull(ob, sb)) continue;
                const int k = boxBox(sb, ob, n, pts, dep);
                for (int j = 0; j < k; ++j) {
                    Contact ct;
                    ct.p = pts[j], ct.n = n, ct.depth = dep[j], ct.a = &body_, ct.m = chassis;
                    addContact(ct, body_, S.q, wrecked_, P(284), P(285), contacts_);
                }
            }
        }
        // Triangles (terrain and meshes): 0x7260a0(1) against the bound sphere, then 0x765bc0 + 0x7667e0 per box;
        // the contact normal is the triangle's.
        auto boxTris = [&](const Tri& t) {
            float d;
            Vec3 nn, pp;
            if (!sphereTriangle(t, bc, br, d, nn, pp)) return;
            for (const Obb& ob : boxes) {
                const int k = boxTriangle(ob, t, pts, dep);
                for (int j = 0; j < k; ++j) {
                    Contact ct;
                    ct.p = pts[j], ct.n = t.n, ct.depth = dep[j], ct.a = &body_, ct.m = chassis;
                    ct.surface = t.surface;
                    ct.fromTriangle = true;
                    addContact(ct, body_, S.q, wrecked_, P(284), P(285), contacts_);
                }
            }
        };
        if (env.terrain) {
            tris.clear();
            env.terrain->query(bc.x, bc.z, br, tris);
            for (const Tri* t : tris) boxTris(*t);
        }
        for (const Tri* t : scene.tris) boxTris(*t);
        // Sphere objects vs boxes: 0x71a9e0 with the normal reversed (towards the body).
        for (const auto& [sc, sr] : scene.spheres)
            for (const Obb& ob : boxes) {
                Contact ct;
                if (!sphereObb(ob, sc, sr, ct.p, ct.n, ct.depth)) continue;
                ct.n = -ct.n;
                ct.a = &body_, ct.m = chassis;
                addContact(ct, body_, S.q, wrecked_, P(284), P(285), contacts_);
            }
    }
    // Vehicle vs vehicle (0x725490, each pair once from the body-pair list 0x71e1d0): A's chassis spheres 0x725130
    // (vs B's spheres and wheels 0x71b1b0, B's boxes 0x71a9e0), A's boxes 0x724950 (vs B's spheres and wheels with
    // the normal reversed, B's boxes 0x7266a0 / 0x726160 / 0x763890), A's wheels 0x724eb0 (vs B's spheres 0x71b1b0
    // and boxes 0x71a9e0, tyre material). Non-wheel contacts take the two chassis materials combined (0x725db0).
    for (FuelVehicle* o : others) {
        if (o == this || o < this) continue;  // each pair once (the contact carries both bodies)
        if (len(o->body_.S.x - S.x) > body_.boundR + o->body_.boundR) continue;
        const ContactMat mat = combineMaterials(chassis, chassisMaterial(o->P(282), o->P(283), o->wrecked_, o->nWheels_));
        std::vector<std::pair<Vec3, float>> mySph, oSph, oWheel;
        chassisSpheres(mySph);
        o->chassisSpheres(oSph);
        for (int w = 0; w < o->nWheels_; ++w)
            if (o->wheels_[size_t(w)].active) oWheel.push_back({o->wheels_[size_t(w)].hubW, o->wheels_[size_t(w)].radius});
        std::vector<Obb> oBoxes;
        o->chassisBoxes(oBoxes);
        auto add = [&](Contact& ct, float e) {
            ct.b = ct.a == &body_ ? &o->body_ : &body_;
            addContact(ct, body_, S.q, wrecked_, P(284), e, contacts_);
        };
        // 0x725130: A's spheres.
        for (const auto& [c, r] : mySph) {
            for (const auto& [oc, orr] : oSph) {
                Contact ct;
                if (!sphereSphere(oc, orr, c, r, ct.p, ct.n, ct.depth)) continue;
                ct.a = &body_, ct.m = mat;
                add(ct, P(286));
            }
            for (const auto& [oc, orr] : oWheel) {
                Contact ct;
                if (!sphereSphere(oc, orr, c, r, ct.p, ct.n, ct.depth)) continue;
                ct.a = &body_, ct.m = mat;
                add(ct, P(286));
            }
            for (const Obb& ob : oBoxes) {
                Contact ct;
                if (!sphereObb(ob, c, r, ct.p, ct.n, ct.depth)) continue;
                ct.a = &body_, ct.m = mat;
                add(ct, P(286));
            }
        }
        // 0x724950: A's boxes.
        for (const Obb& mb : boxes) {
            for (const auto& [oc, orr] : oSph) {
                Contact ct;
                if (!sphereObb(mb, oc, orr, ct.p, ct.n, ct.depth)) continue;
                ct.n = -ct.n;
                ct.a = &body_, ct.m = mat;
                add(ct, P(286));
            }
            for (int w = 0; w < o->nWheels_; ++w) {  // B's wheel against A's box: a contact of B's wheel
                WheelRec& wr = o->wheels_[size_t(w)];
                if (!wr.active) continue;
                Contact ct;
                if (!sphereObb(mb, wr.hubW, wr.radius, ct.p, ct.n, ct.depth)) continue;
                ct.a = &o->body_;
                attachWheel(ct, wr, rotate(o->body_.S.q, kUp));
                ct.b = &body_;
                addContact(ct, o->body_, o->body_.S.q, o->wrecked_, o->P(284), o->P(286), contacts_);
            }
        }
        {
            Vec3 n, pts[20];
            float dep[20];
            for (const Obb& ob : oBoxes) {
                if (!obbSphereCull(ob, S.x, body_.boundR)) continue;
                for (const Obb& mb : boxes) {
                    if (!obbObbCull(mb, ob)) continue;
                    const int k = boxBox(ob, mb, n, pts, dep);
                    for (int j = 0; j < k; ++j) {
                        Contact ct;
                        ct.p = pts[j], ct.n = n, ct.depth = dep[j], ct.a = &body_, ct.m = mat;
                        add(ct, P(286));
                    }
                }
            }
        }
        // 0x724eb0: A's wheels.
        for (int w = 0; w < nWheels_; ++w) {
            WheelRec& r = wheels_[size_t(w)];
            if (!r.active) continue;
            for (const auto& [oc, orr] : oSph) {
                Contact ct;
                if (!sphereSphere(oc, orr, r.hubW, r.radius, ct.p, ct.n, ct.depth)) continue;
                ct.a = &body_;
                attachWheel(ct, r, upW);
                add(ct, P(286));
            }
            for (const Obb& ob : oBoxes) {
                Contact ct;
                if (!sphereObb(ob, r.hubW, r.radius, ct.p, ct.n, ct.depth)) continue;
                ct.a = &body_;
                attachWheel(ct, r, upW);
                add(ct, P(286));
            }
        }
    }
    // Reduction 0x71b7e0 (more than two contacts; vehicles have +0xedd = 0, so never 0x71bad0).
    if (contacts_.size() > 2) reducePerWheel(contacts_);
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
    if (nitro_ > 0) nitroPush();  // [nitro]
    // 0x70dc20: F -> a, T -> alpha (no gyroscopic term for vehicles).
    body_.S.a = body_.F * body_.invMass;
    body_.S.alpha = body_.I1invW * body_.T;
}

void FuelVehicle::setNitro(float amount, float topSpeed) {  // [nitro]
    nitro_ = amount, nitroTop_ = topSpeed;
    rpmStretch_ = amount > 0 ? nitro::rpmStretch(amount, nitroParams()) : 1.0f;
}

// [DEVIATION nitro, physics/nitro.hpp] The boost's push: along the body's forward axis through the centre of mass (no
// pitching torque: a bike does not wheelie), mass x thrustAccel (the amount, the strength, the fade below the lifted top
// speed) while any wheel is on the ground (0x52e620: a bike on its back wheel pushes as hard); forward gears only, never
// wrecked.
void FuelVehicle::nitroPush() {
    if (wrecked_ || gear_ < 1 || !anyWheelLoaded()) return;
    const float a = nitro::thrustAccel(nitro_, vfwd_, nitroTop_, 1.0f, nitroParams());
    if (a > 0) body_.addForceAt(fwd() * (a * body_.mass), body_.S.x);
}

// ---- body forces 0x522830 --------------------------------------------------------------------

void FuelVehicle::aeroDrag() {  // 0x530d20
    BodyState& S = body_.S;
    const float k = 200.0f * body_.boundR * body_.boundR;
    float linF = allWheelsLoaded() ? P(93) : P(94);
    if (nitro_ > 0) linF /= rpmStretch_ * rpmStretch_;  // [nitro] the boost's lifted top speed holds against the air too
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
    // 1. Caught by a tornado (+0x1420 == 2, P43/P61; p23's "drowned"): strong lift (P288 < 0).
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
    // 9. Guided target forces (+0x1421, any wheel loaded; AI branch, owner +0x1a49 set; P29, U5 decoded
    // from the asm 0x522fb1..0x52370f): the position pull acts across the body's up axis (+0x4e4)
    // beyond 0.02 v of the target, F = e sqrt(|e| - 0.02 v) / |e| * m * 20; the velocity match
    // F = (v_target - v) across up * m * 5 (both at the CoM); the heading torque about up,
    // T = u (u . (fwd x fwd_target)) m * 10 / sqrt(|..|), when the sine exceeds 0.2.
    if ((guide_.pos || guide_.rot) && anyWheelLoaded()) {
        const Vec3 u = up();
        if (guide_.pos) {
            const float thr = speed_ * 0.02f;  // +0x13cc * 0x9dc018
            Vec3 e = guide_.p - S.x;
            e = e - u * dot(e, u);
            const float L = len(e);
            if (thr < L) body_.addForceAt(e * (std::sqrt(L - thr) / L * m * 20.0f), S.x);  // 20.0 @0x996154
            Vec3 dv = guide_.v - S.v;
            dv = dv - u * dot(dv, u);
            body_.addForceAt(dv * (m * 5.0f), S.x);  // 5.0 @0x9dbf8c
        }
        if (guide_.rot) {
            const Vec3 ft = rotate(guide_.q, {0, 0, 1});  // (0,0,1) @0xa06478
            const Vec3 t = u * dot(u, cross(fwd(), ft));
            const float tl = len(t);
            if (tl > 0.2f) body_.T += t * (std::sqrt(tl) * m / tl * 10.0f);  // 0.2 @0x9dbfa8, 10.0 @0x996494
        }
    }
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
        // [merge nitro + truck-power] the boost's torque gain (+0x13c0) is part of the governed extra torque: s =
        // engine_power x torqueScale x boost (notes truck-power.md, "Merge into main"). Without a boost (+0x13c0 = 1)
        // this is FUEL's thr x 1 x 0x524c70 bit for bit; nitroGoverned = false (bench) is main's ungoverned boost.
        const float drive = nitroGoverned && boostMul_ != 1.0f ? thr * governedTorque(rpm, powerScale * torqueScale_ * boostMul_)
                                                               : thr * boostMul_ * engineTorque(rpm);
        const float engAcc = drive * gearMul(gear_) / P(323) - (1 - engThrottle_) * rpm * P(329);
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
        // boost is dead data (P633 = 0); [nitro] the port's boost blends FUEL's boost drag P92 in and scales the quadratic
        // drag by 1 / stretch^2, so a vehicle this drag holds back (P92 = P91: the bikes) gains the same lifted top speed
        // (both 0 / 1 without nitro: FUEL's drag).
        const float boost = nitro_ > 0 ? nitro::wheelDragBlend(nitro_, nitroParams()) : 0.0f;
        const float dragK = nitro_ > 0 ? 1.0f / (rpmStretch_ * rpmStretch_) : 1.0f;
        const float a = -((hbm ? hb : 0.0f) / I + B / I) * sg - (P(129) * C.hard + P(153) * C.soft) * C.rolling * load * om -
                        ((1 - boost) * P(91) + boost * P(92)) * dragK * om * std::fabs(om) * load;
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
    S.auxVel[0] = clampf(std::max(S.auxVel[0], 500.0f), P(325), P(324) * rpmStretch_);  // [nitro] x 1 without a boost
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
        if (FuelVehicle::substepHook) for (FuelVehicle* v : vehicles) FuelVehicle::substepHook(*v, 0);
        for (FuelVehicle* v : vehicles) v->driveline();
        if (FuelVehicle::substepHook) for (FuelVehicle* v : vehicles) FuelVehicle::substepHook(*v, 1);
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
        if (FuelVehicle::substepHook) for (FuelVehicle* v : vehicles) FuelVehicle::substepHook(*v, 2);
        remaining -= h;
    }
    for (FuelVehicle* v : vehicles) v->postStep(dt, env);
}

}  // namespace fuel::fv
