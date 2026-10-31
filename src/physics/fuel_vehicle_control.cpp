// FUEL's vehicle control layer: the per-frame pre-step 0x526bd0 (inputs, steer channels, gearbox, steer
// angle, Ackermann, engine throttle, surface rumble) and post-step 0x529ce0 (timers, reference normals,
// overturn and wreck, assists, rider CoM shift, tyre coefficients, visual lean, ground blocks).
// Decoded in ~/re-work/fuel/notes/p23-vehicle-physics.md Part V and sections 7.3, 7.4, 9.
#include "fuel_vehicle.hpp"

#include <algorithm>
#include <cmath>

namespace fuel::fv {

namespace {
constexpr Vec3 kUp{0, 1, 0};
uint32_t gRandom = 1;
float random01() {  // Park-Miller (0x6460f0, Inferred)
    gRandom = uint32_t((uint64_t(gRandom) * 16807u) % 2147483647u);
    return float(gRandom) / 2147483647.0f;
}
}  // namespace

// ---- derived inputs (Part V1) -----------------------------------------------------------------

bool FuelVehicle::handbrakeMode() const { return P(397) >= 0.1f && !wrecked_ && !airborneLong_ && in_.handbrake; }
float FuelVehicle::brakePedal() const {  // 0x523dd0
    if (wrecked_) return 1;
    return gear_ == -1 ? in_.accel : in_.brake;
}
float FuelVehicle::drivePedal() const {  // 0x523e10
    if (wrecked_ || airborneLong_) return 0;
    return std::max(0.0f, gear_ == -1 ? in_.brake : in_.accel);
}
float FuelVehicle::brakeCombined() const {  // 0x523fa0
    return 1 - (1 - std::max(brakePedal(), 0.0f)) * (1 - (handbrakeMode() ? 1.0f : 0.0f));
}
float FuelVehicle::notBraking() const {  // 0x523d50
    if (wrecked_) return 0;
    return clampf(1 - 2 * clampf(brakeCombined() - 0.5f, 0, 1), 0, 1);
}
float FuelVehicle::throttleEff() const {  // 0x524160
    if (bumpTimer_ > 0 || airborneLong_ || wrecked_) return 0;
    const float pedal = gear_ == -1 ? in_.brake : in_.accel;
    const float g = brakeCombined();
    return std::max(clampf(pedal - 2 * clampf(g - 0.5f, 0, 1), 0, 1) - 0.0f, 0.0f);  // - phys+0x141c (0)
}
float FuelVehicle::brakeEff() const {  // 0x523e90
    if (!(bumpTimer_ <= 0 && !wrecked_ && !airborneLong_)) return 1;
    return std::max(autoBrake_, brakePedal());
}
float FuelVehicle::wheelBrakeFactor(int w) const {  // 0x523f00
    const float b = brakePedal();
    if (front(w)) return b;
    return 1 - (1 - b) * (1 - (handbrakeMode() ? 1.0f : 0.0f));
}
float FuelVehicle::steerStick() const { return (wrecked_ || airborneLong_) ? 0.0f : in_.steer; }  // 0x524120
float FuelVehicle::clutchFraction() const {  // 0x525580
    const float T0 = shiftTimes_[0], T1 = shiftTimes_[1], T2 = shiftTimes_[2], t = shiftT_;
    float f;
    if (t < T0) {
        f = T0 > 0 ? 1 - t / T0 : 1.0f;
    } else if (t < T0 + T1) {
        f = 0;  // (the decompile returns early here; the throttle is cut anyway)
    } else {
        f = T2 > 0 ? (t - T0 - T1) / T2 : 1.0f;
    }
    return clampf(f, 0, 1);
}

// ---- steering (Part V2) -----------------------------------------------------------------------

void FuelVehicle::steerChannels(float dt) {  // 0x52e830
    const float s = steerStick();
    const float T = s * std::fabs(s);
    const float k = P(321) + 1;
    struct Ch {
        float* c;
        int pa, pd, pr;
    } ch[4] = {{&st_.chA, 309, 310, 311}, {&st_.chB, 306, 307, 308}, {&st_.chC, 316, 318, 320}, {&st_.chD, 315, 317, 319}};
    for (const Ch& c : ch) {
        const float cur = *c.c;
        const float R = cur * T < 0 ? P(c.pr) : (std::fabs(cur) <= std::fabs(T) ? P(c.pa) : P(c.pd));
        const float n = cur + clampf(T - cur, -R * dt, R * dt);
        *c.c = expSmooth(cur, n, k, dt);
    }
    const float y = dot(refUp_, body_.S.w) * P(105) * (1 - std::min(20 * std::fabs(s), 1.0f));
    st_.feedback = clampf(y, -P(106), P(106));
    st_.sinceSteer = std::fabs(s) > 0.25f ? 0.0f : st_.sinceSteer + dt;
}

void FuelVehicle::steerAngle(float dt) {  // 0x524710
    const float v0 = airtime_ > 0.5f ? 0.0f : speed_;
    const float vr = std::max(v0, 0.01f);
    const float inv = 1.0f / (std::sqrt(vr) * vr);
    float D = inv * P(314);
    int nFront = 0, nSat = 0;
    for (int w = 0; w < nWheels_; ++w) {
        if (!front(w)) continue;
        ++nFront;
        nSat += wheelSlidingOrUnloaded(w) ? 1 : 0;
    }
    float base, lim;
    if (nSat < nFront) {
        const float bp = std::max(brakePedal(), 0.0f);
        const float hb = handbrakeMode() ? 1.0f : 0.0f;
        const float a = throttleEff();
        const float A = (1 - hb) * ((1 - bp) * (a * P(304) + (1 - a) * P(302)) + bp * P(303)) + hb * P(303);
        const float B = (1 - hb) * ((1 - bp) * (a * P(300) + (1 - a) * P(298)) + bp * P(299)) + hb * P(299);
        base = std::min(inv * B, A);
        lim = A - base;
    } else {
        base = std::min(P(301) * inv, P(305));
        lim = P(305) - base;
    }
    st_.base = expSmooth(st_.base, base, 1.1f, dt);
    D = std::min(D, lim);
    const float tanB = D * st_.chD + st_.base * st_.chB;
    const float tanF = D * st_.chD + st_.base * clampf(st_.chB + st_.feedback, -1, 1);
    st_.tanF = tanF;
    const float fr = clampf(v0 / P(297), 0, 1);
    const float kB = (1 - fr) * P(295) + fr * P(296);  // vehicle type in {0, 4}
    st_.dirB = normalize({-tanB * kB, 0, 1});
    st_.dirF = normalize({-tanF, 0, 1});
}

void FuelVehicle::ackermann() {  // 0x526480
    if (nWheels_ < 3) {
        st_.dir0 = st_.dirF;
        return;
    }
    const Vec3 F = (wheels_[0].hubL + wheels_[2].hubL) * 0.5f;
    const Vec3 R = (wheels_[1].hubL + wheels_[3].hubL) * 0.5f;
    const Vec3 m = cross(kUp, st_.dirF);
    const float den = -m.z;
    if (std::fabs(den) < 0.001f) {
        st_.dir0 = st_.dir2 = st_.dirF;
        return;
    }
    const float t = (F - R).z / den;
    const Vec3 Cc = F + m * t;
    for (int w : {0, 2}) {
        Vec3 d = normalize(cross(kUp, wheels_[size_t(w)].hubL - Cc));
        if (den < 0) d = -d;  // (the decode reads "negated if den > 0" with the operands of 0x4adfe0 swapped)
        (w == 0 ? st_.dir0 : st_.dir2) = d;
    }
}

void FuelVehicle::steerAccel(float dt) {  // 0x521f80
    const float v = speed_;
    const float s = steerStick();
    const float wy = dot(body_.S.w, kUp);
    const float dT = (v > 0.001f && s * wy > 0.01f) ? 1.0f : 0.0f;
    st_.drift = expSmooth(st_.drift, dT, st_.drift <= dT ? 1.5f : 2.0f, dt);
    const float d = st_.drift;
    float f = 0;
    for (int w = 0; w < nWheels_; ++w) f += C_[size_t(w)].soft;
    f /= float(std::max(nWheels_, 1));
    const float a = throttleEff(), b = brakeEff(), m = handbrakeMode() ? 1.0f : 0.0f;
    auto table = [&](int f0, int f1, int df0, int df1) {
        float X[4];
        for (int i = 0; i < 4; ++i)
            X[i] = (P(f1 + i) * f + P(f0 + i) * (1 - f)) * (1 - d) + (P(df1 + i) * f + P(df0 + i) * (1 - f)) * d;
        return X[3] * m + (X[2] * b + (X[1] * a + X[0] * (1 - a)) * (1 - b)) * (1 - m);
    };
    float High = table(228, 232, 237, 241), Low = table(204, 208, 213, 217);
    if (notAllLoaded_ > 0) {
        const float t = clampf(notAllLoaded_ / std::max(P(265), 0.5f), 0, 1);
        High = lerpf(High, P(236) * (1 - d) + P(245) * d, t);
        Low = lerpf(Low, P(212) * (1 - d) + P(221) * d, t);
    }
    const float vref = P(224) * (1 - f) + P(227) * f;
    const float r = vref > 1e-6f ? clampf(v / vref, 0, 1) : 1.0f;
    float S = r * r * High + (1 - r * r) * Low;
    const float vt = P(322) > 0.5f ? std::max(v, vDriven_) : v;
    const float ramp = clampf(vt * ((P(223) * (1 - a) + P(222) * a) * (1 - f) + (P(226) * (1 - a) + P(225) * a) * f), 0, 1);
    bool frontSlide = notAllLoaded_ > 0;
    for (int w = 0; w < nWheels_; ++w)
        if (front(w) && wheelSliding(w)) frontSlide = true;
    const float gate = frontSlide ? 1.0f : 0.0f;
    const float many = (enoughWheelsSliding() || notAllLoaded_ > 0) ? 1.0f : 0.0f;
    const float kk = clampf(b + m, 0, 1);
    S = (kk * many + (1 - kk) * gate) * clampf(up().y, 0, 1) * ramp * S * slopeGrip_;
    if (S > 0.001f) {
        float sum = 0;
        for (int k = 0; k < kGroundClasses; ++k) sum += surfaceClassFraction(k) * P(246 + k);
        S = S * sum / std::max(P(264) * v, 1.0f);
    }
    const float rate = P(273) * m + (P(272) * b + (P(274) * a + P(271) * (1 - a)) * (1 - b)) * (1 - m);
    st_.accelMag = expSmooth(st_.accelMag, S, rate, dt);
    const float c = P(313) * st_.chC + P(312) * clampf(st_.chA + st_.feedback, -1, 1);
    float gy = 1;
    if (P(270) > 0.001f) {
        const float wu = dot(body_.S.w, up());
        gy = clampf(c <= 0 ? P(270) - wu : P(270) + wu, 0, 1);
    }
    st_.accel = c * gy * st_.accelMag;
}

// ---- gearbox and engine throttle (Part V6) ------------------------------------------------------

int FuelVehicle::autoGearbox() {  // 0x525c50
    const int g = gear_;
    bool hold = (g == 1 && st_.sinceSteer < 5 && drivenWheelsSliding() && len2(body_.S.v) < 225.0f) || !anyWheelLoaded();
    const float v = (!allFrontSlidingOrAir() || (g < 2 && st_.sinceSteer >= 5)) ? engineSpeedAtWheel() : P(393) * speed_;
    const int nG = int(std::floor(P(349)));
    const float a = throttleEff(), b = brakeEff();
    auto toNeutral = [&]() {
        standstill_ = 0;
        return 0;
    };
    if (g >= 2) {
        if (P(374) < sinceDown_ && P(375) < sinceUp_) {
            float best = wheelTorqueAtSpeed(v, g);
            int cand = g;
            for (int gi = 1; gi <= g - 1; ++gi) {
                const float t = wheelTorqueAtSpeed(v, gi) * hysteresis_;
                if (best < t) best = t, cand = gi;
            }
            if (cand != g) return cand;
        }
        if (!(standstill_ <= P(376) || b <= 0.01f || a >= 0.01f)) return toNeutral();
    }
    if (g > 0) {
        if (g < nG && !hold && notBraking() > 0.5f && P(374) < sinceUp_ && P(375) < sinceDown_) {
            float best = wheelTorqueAtSpeed(v, g);
            int cand = g;
            for (int gi = g + 1; gi <= nG; ++gi) {
                const float t = wheelTorqueAtSpeed(v, gi) * hysteresis_;
                if (best < t) best = t, cand = gi;
            }
            if (cand != g) return cand;
        }
        if (g == 1 && P(376) < standstill_ && b > 0.01f && a < 0.01f) return toNeutral();
    }
    if (g == 0) {
        if (P(376) < standstill_ && brakePedal() > 0.01f && drivePedal() < 0.01f) {
            standstill_ = 0;
            return -1;
        }
        if (a > 0.01f && notBraking() > 0.5f) return 1;
    }
    if (g != -1 || b <= 0.01f || a >= 0.01f) return g;
    return 0;
}

void FuelVehicle::shiftSequencer(float dt) {  // 0x526030
    const float m = std::max(throttleEff(), brakeEff());
    peakPedal_ = m <= peakPedal_ ? expSmooth(peakPedal_, m, 1.01f, dt) : m;
    sinceUp_ += dt;
    sinceDown_ += dt;
    hysteresis_ = std::min(hysteresis_ + P(394) * dt, 0.99f);
    standstill_ = len2(body_.S.v) >= 9.0f ? 0.0f : standstill_ + dt;
    if (shiftT_ == 0) {
        gearPrev_ = gear_;
        gearTarget_ = autoGearbox();
        if (gearTarget_ != gear_) {
            shiftT_ += dt;
            hysteresis_ = 0.25f;
            st_.sinceSteer = 0;
        }
        return;
    }
    const int tgt = gearTarget_;
    if (tgt < gearPrev_ || tgt < 1 || throttleEff() > 0.5f)
        shiftTimes_[0] = P(382), shiftTimes_[1] = P(383), shiftTimes_[2] = P(384);
    else
        shiftTimes_[0] = P(385), shiftTimes_[1] = P(386), shiftTimes_[2] = P(387);
    if (gear_ == 0) {
        shiftT_ = std::max(shiftT_, shiftTimes_[0] + shiftTimes_[1]);
        gear_ = tgt;
    }
    shiftT_ += dt;
    if (shiftT_ > shiftTimes_[0]) {
        if (tgt != gear_) {
            if (gear_ < tgt) sinceUp_ = 0;
            if (tgt < gear_) sinceDown_ = 0;
            gear_ = tgt;
        }
        if (shiftT_ > shiftTimes_[0] + shiftTimes_[1] + shiftTimes_[2]) shiftT_ = 0;
    }
}

void FuelVehicle::engineThrottle(float dt) {  // 0x525310
    const float a = throttleEff();
    engThrottle_ = a;
    if (a < 0.5f) st_.sinceSteer = 0;
    if (gear_ != 0) {
        const float frac = clutchFraction();
        if (frac < 0.99f) {
            if (shiftTimes_[0] <= shiftT_) {
                if (shiftT_ <= shiftTimes_[1] + shiftTimes_[0]) engThrottle_ = 0;
            } else {
                engThrottle_ = a * frac * frac;
            }
            if (gearTarget_ < gearPrev_) engThrottle_ = std::max(engThrottle_, P(388));
        }
    }
    if (P(401) > 0.5f) {
        if (minDrivenSlipRatio() <= 1)
            launchRamp_ = std::min(launchRamp_ + P(403) * dt, 1.0f);
        else
            launchRamp_ = std::max(launchRamp_ - P(402) * dt, 0.0f);
        engThrottle_ *= launchRamp_;
    }
    if (airtime_ > 0.5f) engThrottle_ *= clampf(1 - (airtime_ - 0.5f), 0, 1);
    const float idle = clampf((P(325) - body_.S.auxVel[0]) * P(326) * 0.01f, 0, 1);
    engThrottle_ = clampf((1 - autoBrake_) * idle + engThrottle_, 0, 1);
}

// ---- pre-step 0x526bd0 -----------------------------------------------------------------------

void FuelVehicle::rumble(float dt) {  // Part V3.15
    BodyState& S = body_.S;
    const Vec3 upW = up();
    const float k1 = clampf(0.1f * speed_, 0, 1);
    for (int w = 0; w < nWheels_; ++w) {
        const GroundBlock& C = C_[size_t(w)];
        const float l = clampf(20 * wheels_[size_t(w)].load, 0, 1);
        if (rumblePeriod_[w] <= 0) rumblePeriod_[w] = 0.1f;
        rumbleT_[w] += dt;
        while (rumbleT_[w] > rumblePeriod_[w]) {
            rumbleT_[w] -= rumblePeriod_[w];
            rumblePeriod_[w] = (random01() + 1) * (1 / (speed_ + 1) + 0.1f);
        }
        const float sn = std::sin(6.28f * rumbleT_[w] / rumblePeriod_[w]);
        rumbleA_[w] = P(665) * sn * 0.5f * l * C.bumpsWheel * k1;
        const float h = (sn + 1) * 0.5f;
        const float amp = C.bumpsRot * l * k1;
        S.w += normalize(cross(rotate(S.q, wheels_[size_t(w)].hubL), upW)) * (h * amp * P(664));
        rumbleB_[w] = -(1 - h) * P(666) * amp;
    }
}

void FuelVehicle::preStep(float dt, Environment&) {
    BodyState& S = body_.S;
    speed_ = len(S.v);
    vfwd_ = dot(S.v, fwd());
    vDriven_ = meanDrivenWheelSpeed();
    slopeGrip_ = 0.2f + 0.8f * clampf(10 * (1 - clampf(std::acos(clampf(refUp_.y, 0, 1)) / (P(107) * 0.017453292f), 0, 1)), 0, 1);
    autoBrake_ = speed_ < P(663) ? (1 - clampf(speed_ - (P(663) - 1), 0, 1)) * (1 - clampf(4 * drivePedal(), 0, 1)) : 0.0f;
    steerChannels(dt);
    shiftSequencer(dt);
    steerAccel(dt);
    body_.timeScale = 1;
    steerAngle(dt);
    ackermann();
    engineThrottle(dt);
    boostMul_ = 1;  // boost is dead data (P633 = 0 in every vehicle)
    for (auto& r : wheels_) r.onRoad &= uint8_t(~1u);
    rumble(dt);
    bool allDeep = nWheels_ > 0;
    for (int w = 0; w < nWheels_; ++w) allDeep = allDeep && waterDepth_[size_t(w)] > P(581);
    airborneLong_ = allDeep;
}

// ---- post-step 0x529ce0 ---------------------------------------------------------------------

void FuelVehicle::groundProbe(Environment& env) {  // 0x5298d0 (terrain only)
    if (!env.terrain) return;
    Vec3 n;
    groundHeight_ = env.terrain->surfaceHeight(body_.S.x.x, body_.S.x.z, &n);
    heightAbove_ = body_.S.x.y - groundHeight_;
    probeNormal_ = n;
}

void FuelVehicle::assists(float dt) {  // 0x52bb60
    if (wrecked_) return;
    BodyState& S = body_.S;
    const Vec3 u = up(), f = fwd(), l = lat();
    bool grounded = false;
    for (int w = 0; w < nWheels_; ++w) grounded = grounded || wheels_[size_t(w)].load > 1e-6f;
    if (grounded && P(15) > 0) {
        // Bike roll balance; with the shipped P21-P23 = 0 the target is world up.
        const Vec3 t = kUp;
        const Vec3 n = refUp_;
        const Vec3 a = normalize(f - n * dot(n, f));
        float G = P(15), M = P(16);
        if (u.y > t.y) G = P(19), M = P(20);
        const float target = clampf(G * dot(a, cross(u, t)), -M, M);
        S.w += a * (target - dot(S.w, a));
    }
    if (grounded) {
        airAssist_ = 0;
        return;
    }
    airAssist_ += dt;
    if (airAssist_ > P(33)) {
        const float s = (wrecked_ || airborneLong_) ? 0.0f : in_.steer;
        const float sY = (wrecked_ || airborneLong_) ? 0.0f : in_.stickZ;
        S.w += cross(u, refUp_) * (P(36) * dt);
        S.w += f * (s * dt * 10 * P(41));
        S.w += l * (sY * dt * 10 * P(43));
        S.w -= u * (s * dt * 10 * P(42));
        S.w *= 1 - std::min(2 * P(44) * dt, 0.9f);
    }
}

void FuelVehicle::comShift() {  // 0x52b630
    float k = 1;
    if (P(40) > 1) k = 1 - clampf(2 * (speed_ / P(40) - 0.5f), 0, 1);
    const float sY = in_.stickZ;
    const float e = sY >= 0.75f ? (sY - 0.75f) * 4 : (sY <= -0.75f ? (sY + 0.75f) * 4 : 0.0f);
    const float pedal = sY > 0 ? brakePedal() : drivePedal();
    const float ks = clampf(0.2f * speed_, 0, 1);
    Vec3 offset = Vec3{0, 0, 1} * (k * (sY * P(38) * (1 - ks) + pedal * e * P(37) * ks)) + Vec3{1, 0, 0} * (-P(39) * steerStick() * k);
    if (wrecked_ && nWheels_ == 2) offset = {0, 0.1f, 0};
    const Vec3 com = comBase_ + offset;
    if (len(com - comPrev_) > 0.01f) {
        const Vec3 delta = com - comPrev_;
        comPrev_ = com;
        body_.S.x += rotate(body_.S.q, delta);
        com_ = com;
        for (int w = 0; w < nWheels_; ++w) wheels_[size_t(w)].hubL = proxy_.spheres[size_t(wheelSphere_[size_t(w)])].c - com_;
        computeMassProperties();
    }
}

void FuelVehicle::tyreCoefficients() {  // 0x52cce0
    const float v = speed_;
    const float lc0 = clampf(1 - v * 0.05f, 0, 1);
    const float fB = clampf(1 - (v - 0.75f * P(152)) / (P(152) - 0.75f * P(152)), 0, 1);  // soft: low-speed weight
    const float fA = clampf(1 - (v - 0.75f * P(128)) / (P(128) - 0.75f * P(128)), 0, 1);  // hard
    const float hB = 1 - fB, hA = 1 - fA;
    float f9 = 1;
    if (P(279) > 1e-6f) f9 = 1 - (3.6f / P(279)) * v;
    f9 = clampf(f9, 0, 1);
    f9 = f9 * f9;
    float kk = (handbrakeMode() ? 1.0f : 0.0f) + surfaceClassFraction(0) + brakeEff();  // 0x51ffe0: class argument Unknown (0 assumed)
    kk = clampf(kk, 0, 1);
    // Speed multipliers: linear tables at 0,10,20,30,40,50,75,100,150,200 km/h.
    const float kmh = v * 3.6f;
    const float knots[10] = {0, 10, 20, 30, 40, 50, 75, 100, 150, 200};
    int seg = 0;
    while (seg < 8 && kmh >= knots[seg + 1]) ++seg;
    const float t = clampf((kmh - knots[seg]) / (knots[seg + 1] - knots[seg]), 0, 1);
    auto table = [&](int first) { return P(first + seg) * (1 - t) + P(first + seg + 1) * t; };
    const float Mf = table(184) * kk + table(164) * (1 - kk);
    const float Mr = table(194) * kk + table(174) * (1 - kk);
    const bool frontSlide = allFrontSliding();
    for (int w = 0; w < nWheels_; ++w) {
        WheelRec& r = wheels_[size_t(w)];
        const GroundBlock& C = C_[size_t(w)];
        const bool f = front(w);
        // Extra depth: rumble and soft-ground sink (non-player form of the P667 term).
        const float k10 = clampf(v * 0.05f, 0, 1);
        const float f6 = C.bumpsRot * P(667) * k10 * k10 * groundTime_[w];
        float e8;
        if (0.001f <= f6)
            e8 = std::max(-(C.depth * clampf(v * 0.5f, 0, 1) * f6), -0.25f);
        else
            e8 = -(C.depth * clampf(v * 0.5f, 0, 1) * groundTime_[w]);
        r.mat.extraDepth = rumbleB_[w] * groundTime_[w] + e8;
        float* s1 = r.mat.set1;
        float* s2 = r.mat.set2;
        s1[0] = C.stat * slopeGrip_;
        if (!f && frontSlide) s1[0] *= 0.5f;
        s1[2] = C.kin * slopeGrip_;
        s1[7] = C.dyn * slopeGrip_;
        const float wB = C.soft, wA = C.hard;
        s1[6] = (P(163) * wB + P(162) * wA) * slopeGrip_;
        for (int i = 0; i < 8; ++i) s2[i] = s1[i];
        const int o = f ? 0 : 1;  // rear reads the next parameter
        const float M = f ? Mf : Mr;
        const float c3 = f ? P(158) * (1 - lc0) + P(156) * lc0 : P(157) * lc0 + P(159) * (1 - lc0);
        s1[3] = s2[3] = c3;
        s1[0] = (P(108 + o) * wA + P(132 + o) * wB) * M * s1[0];
        s2[0] = (P(110 + o) * wA + P(134 + o) * wB) * M * s2[0];
        s1[1] = (P(112 + o) * wA + P(136 + o) * wB) * M * s1[0];
        s2[1] = (P(114 + o) * wA + P(138 + o) * wB) * M * s2[0];
        s1[2] = (P(116 + o) * wA + P(140 + o) * wB) * M * s1[2];
        s2[2] = (P(118 + o) * wA + P(142 + o) * wB) * M * s2[2];
        s1[7] = (P(124 + o) * wA * hA + P(148 + o) * wB * hB + P(144 + o) * fB * wB + P(120 + o) * fA * wA) * M * s1[7];
        s2[7] = M * (P(146 + o) * fB * wB + P(122 + o) * fA * wA + P(126 + o) * wA * hA + hB * P(150 + o) * wB) * s2[7];
        r.mat.loadFloor = P(277 + o) * f9 + P(280 + o) * (1 - f9);
        r.mat.loadBlend = P(275 + o);
        r.mat.maxDepth = P(71 + o);
        r.mat.c = P(75 + o);
    }
}

void FuelVehicle::updateGround(Environment& env) {  // 0x5c9e80 (per wheel in the port)
    for (int w = 0; w < nWheels_; ++w) {
        const WheelRec& r = wheels_[size_t(w)];
        const double x = double(r.hubW.x) + env.originX, z = double(r.hubW.z) + env.originZ;
        float frac[kGroundClasses] = {};
        float sink = 0;
        if (env.groundClasses)
            env.groundClasses(x, z, frac, sink);
        else
            frac[3] = 1;
        const float water = env.waterDepthAt ? env.waterDepthAt(x, z) : 0.0f;
        waterDepth_[size_t(w)] = std::max(water, sink);
        const float wet = clampf(100 * waterDepth_[size_t(w)] + 0.0f, 0, 1);  // + weather wetness (none)
        for (int k = 0; k < kGroundClasses; ++k) frac_[size_t(w)][size_t(k)] = frac[k];
        if (tables_ && tables_->loaded) C_[size_t(w)] = buildGroundBlock(*tables_, frac, wet);
    }
}

void FuelVehicle::wreckDetect(float dt) {  // 0x528e40 (the orientation tests; damage meter and P599 not ported)
    if (wrecked_) {
        wreckTime_ += dt;
        if (wreckTime_ > 2.0f) respawnWanted_ = true;
        return;
    }
    const Vec3 u = up(), f = fwd(), l = lat();
    bool grounded = anyWheelLoaded();
    bool wreck = dot(u, refUp_) < -0.3f;
    if (grounded && dot(u, refUp_) < P(627)) wreck = true;
    if (grounded && notAllLoaded_ > P(628) && (std::fabs(dot(f, refUp_)) > P(629) || std::fabs(dot(l, refUp_)) > P(630))) wreck = true;
    if (wreck) {
        wrecked_ = 1;
        wreckTime_ = 0.001f;
        if (!player_) respawnWanted_ = true;  // AI on a path: reset at once
    }
}

void FuelVehicle::postStep(float dt, Environment& env) {
    BodyState& S = body_.S;
    speed_ = len(S.v);
    vfwd_ = dot(S.v, fwd());
    groundProbe(env);
    if (bumpTimer_ > 0) bumpTimer_ -= dt;
    if (bumpTimer2_ > 0) bumpTimer2_ -= dt;
    airtime_ = anyWheelLoaded() ? 0.0f : airtime_ + dt;
    notAllLoaded_ = allWheelsLoaded() ? 0.0f : notAllLoaded_ + dt;
    for (int w = 0; w < nWheels_; ++w)
        groundTime_[w] = wheels_[size_t(w)].load <= 1e-6f ? 0.0f : std::min(groundTime_[w] + dt, 1.0f);
    // Respawn state machine 0x529410: wreck, RESPAWN button, fell out of the world.
    wreckDetect(dt);
    if (in_.respawn) {
        respawnHold_ += dt;
    } else if (respawnHold_ > 0) {
        respawnHold_ = 0;
        respawnWanted_ = true;
    }
    if (S.x.y < groundHeight_ - 50) respawnWanted_ = true;
    // Reference normals (smoothed towards the ground probe normal).
    auto smoothVec = [&](Vec3& cur, float k) {
        const Vec3 d = probeNormal_ - cur;
        if (len2(d) < 1e-6f) {
            cur = probeNormal_;
        } else {
            cur += d * (1 - 1 / std::pow(k, dt * 30.30303f));
        }
        if (len2(cur) > 1e-6f) cur = normalize(cur);
    };
    smoothVec(refUp_, 1.1f);
    smoothVec(refUp2_, 1.05f);
    // Overturn timer.
    if (wrecked_ || dot(up(), refUp_) >= 0.25f)
        overturn_ = 0;
    else
        overturn_ += dt;
    if ((overturn_ > 1 && speed_ < 2) || overturn_ > 2) {
        overturn_ = 0;
        respawnWanted_ = true;
    }
    for (int w = 0; w < nWheels_; ++w) wheelDirPost_[w] = w == 0 ? st_.dir0 : (w == 2 ? st_.dir2 : Vec3{0, 0, 1});  // 0x526760
    assists(dt);
    comShift();
    tyreCoefficients();
    // Visual body lean (bikes, P29 > 1).
    if (P(29) > 1) {
        float lean = clampf(dot(S.v, lat()) * P(658), -P(659), P(659)) + clampf(speed_ * P(657), 0, 1) * st_.chB * P(656);
        const float slope = 1.5707964f - std::acos(clampf(refUp_.y, -1, 1)) - P(660);
        if (slope < std::fabs(lean)) lean = std::copysign(std::max(slope, 0.0f), lean);
        if (lean_ < -100) lean_ = lean;
        lean_ = expSmooth(lean_, lean, std::fabs(lean) > std::fabs(lean_) ? P(29) : P(30), dt);
        leanRender_ = clampf(lean_ * (P(31) * (1 - P(661)) + P(661)), -1.04f, 1.04f);
    } else {
        leanRender_ = 0;
    }
    // Wheel spin for the visuals; display rpm toward the engine (P389/P390 rates).
    for (int w = 0; w < nWheels_; ++w) wheelAngle_[w] += S.auxVel[1 + w] / std::max(wheelR(w), 0.01f) * dt;
    {
        const float target = S.auxVel[0];
        const float rate = (gear_ == 0 || clutchFraction() < 0.99f) ? P(389) : P(390);
        const float step = (rate > 0 ? rate : 1e5f) * dt;
        dispRpm_ += clampf(target - dispRpm_, -step, step);
    }
    updateGround(env);
}

void FuelVehicle::resetTo(const Vec3& modelOrigin, float yaw, Environment& env) {  // 0x5273b0 (snap)
    BodyState& S = body_.S;
    // Orientation from the ground normal and the heading.
    Vec3 n{0, 1, 0};
    float h = modelOrigin.y;
    if (env.terrain) h = env.terrain->surfaceHeight(modelOrigin.x, modelOrigin.z, &n);
    const Quat qy = axisAngle(kUp, yaw);
    const Vec3 axis = cross(kUp, n);
    const float ang = std::acos(clampf(n.y, -1, 1));
    const Quat qt = len(axis) > 1e-6f ? axisAngle(axis, ang) : Quat{};
    S.q = qnormalize(mul(qt, qy));
    // Lowest point of the wheel spheres on the ground.
    float lift = 0;
    for (int w = 0; w < nWheels_; ++w) lift = std::max(lift, wheels_[size_t(w)].radius - hubModel(w).y);
    const Vec3 origin{modelOrigin.x, h + lift, modelOrigin.z};
    com_ = comPrev_ = comBase_;
    for (int w = 0; w < nWheels_; ++w) wheels_[size_t(w)].hubL = proxy_.spheres[size_t(wheelSphere_[size_t(w)])].c - com_;
    computeMassProperties();
    S.x = origin + rotate(S.q, com_);
    S.xComp = {};
    S.v = S.w = S.a = S.alpha = {};
    for (int i = 0; i < 8; ++i) S.auxPos[i] = S.auxAcc[i] = 0, S.auxVel[i] = 0;
    S.auxVel[0] = 1000;
    body_.P = body_.N = S;
    refUp_ = refUp2_ = probeNormal_ = n;
    lean_ = -1000;
    leanRender_ = 0;
    gear_ = gearTarget_ = gearPrev_ = 1;
    shiftT_ = 0;
    sinceUp_ = sinceDown_ = 10;
    st_ = Steer{};
    airtime_ = notAllLoaded_ = overturn_ = airAssist_ = 0;
    wrecked_ = 0;
    wreckTime_ = 0;
    respawnWanted_ = false;
    for (int w = 0; w < nWheels_; ++w) {
        dOmega_[w] = 0;
        groundTime_[w] = 0;
        wheelDirPost_[w] = {0, 0, 1};
    }
    wheelUpdate();
    updateGround(env);
    tyreCoefficients();
}

}  // namespace fuel::fv
