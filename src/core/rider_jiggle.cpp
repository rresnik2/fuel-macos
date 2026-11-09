// PHYSIQUE (port deviation): glute secondary motion of the player's custom rider (rider_jiggle.hpp).
#include "rider_jiggle.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <vector>

namespace fuel {

void RiderJiggle::reset() {
    for (auto& a : x_) a = {};
    for (auto& a : v_) a = {};
    history_ = 0;
    acc_ = 0;
}

void RiderJiggle::integrate(const std::array<std::array<double, 3>, kBones>& a, double dt, bool seated) {
    const double w = 2.0 * M_PI * tune_.freqHz;
    const double k = w * w, c = 2.0 * tune_.zeta * w;
    const std::array<double, 3> g{tune_.gainX, tune_.gainY, tune_.gainZ};
    // the springs live in "amplitude 1" units; the clamps act there (the output is scaled afterwards)
    const std::array<double, 3> hi{tune_.maxSide, tune_.maxUp, tune_.maxFwd};
    const std::array<double, 3> lo{-tune_.maxSide, -(seated ? tune_.seatDown : tune_.maxDown), -tune_.maxFwd};
    acc_ += dt;
    int guard = 0;
    while (acc_ >= kStep - 1e-9 && guard++ < 240) {  // at most a second of steps per call (a stall)
        acc_ -= kStep;
        for (int b = 0; b < kBones; ++b)
            for (int i = 0; i < 3; ++i) {
                double& x = x_[size_t(b)][size_t(i)];
                double& v = v_[size_t(b)][size_t(i)];
                // semi-implicit Euler (stable for these rates at 240 Hz)
                v += (-k * x - c * v - g[size_t(i)] * a[size_t(b)][size_t(i)]) * kStep;
                x += v * kStep;
                if (x > hi[size_t(i)]) x = hi[size_t(i)], v = std::min(v, 0.0);
                if (x < lo[size_t(i)]) x = lo[size_t(i)], v = std::max(v, 0.0);  // inelastic stop (the seat)
            }
    }
    if (guard >= 240) acc_ = 0;
}

void RiderJiggle::stepLocal(const std::array<double, 3>& accel, double dt, bool seated) {
    if (dt <= 0) return;
    std::array<std::array<double, 3>, kBones> a;
    a.fill(accel);
    integrate(a, dt, seated);
}

void RiderJiggle::step(const anim::Mat4& M, double dt, bool seated) {
    if (dt <= 0 || dt > 1.0) {  // a stall or a pause: start over
        reset();
        return;
    }
    std::array<std::array<double, 3>, kBones> p{}, accel{};
    for (int b = 0; b < kBones; ++b) {
        const auto& c = bones_[size_t(b)].centre;
        for (int i = 0; i < 3; ++i) p[size_t(b)][size_t(i)] = c[0] * M[size_t(i)] + c[1] * M[size_t(4 + i)] + c[2] * M[size_t(8 + i)] + M[size_t(12 + i)];
    }
    if (history_ > 0) {
        double jump = 0;
        for (int i = 0; i < 3; ++i) jump = std::max(jump, std::fabs(p[0][size_t(i)] - pPrev_[0][size_t(i)]));
        if (jump > 1.5) reset();  // a teleport / respawn the caller did not report
    }
    if (history_ == 0) {
        pPrev_ = p;
        history_ = 1;
        return;
    }
    std::array<std::array<double, 3>, kBones> vel{};
    for (int b = 0; b < kBones; ++b)
        for (int i = 0; i < 3; ++i) vel[size_t(b)][size_t(i)] = (p[size_t(b)][size_t(i)] - pPrev_[size_t(b)][size_t(i)]) / dt;
    pPrev_ = p;
    if (history_ == 1) {
        velPrev_ = vel;
        history_ = 2;
        return;
    }
    for (int b = 0; b < kBones; ++b) {
        std::array<double, 3> aw{};
        for (int i = 0; i < 3; ++i) aw[size_t(i)] = (vel[size_t(b)][size_t(i)] - velPrev_[size_t(b)][size_t(i)]) / dt;
        // world -> bind frame: the transpose of the (orthonormal) rotation block (row-vector: d_world = d_bind * R)
        for (int i = 0; i < 3; ++i) {
            const double n = std::sqrt(M[size_t(4 * i)] * M[size_t(4 * i)] + M[size_t(4 * i + 1)] * M[size_t(4 * i + 1)] + M[size_t(4 * i + 2)] * M[size_t(4 * i + 2)]);
            accel[size_t(b)][size_t(i)] = n > 0 ? (aw[0] * M[size_t(4 * i)] + aw[1] * M[size_t(4 * i + 1)] + aw[2] * M[size_t(4 * i + 2)]) / n : 0.0;
        }
    }
    velPrev_ = vel;
    integrate(accel, dt, seated);
}

RiderJiggle::Offset RiderJiggle::offset(int k) const {
    Offset o;
    const auto& x = x_[size_t(k)];
    for (int i = 0; i < 3; ++i) o.d[size_t(i)] = amp_ * x[size_t(i)];
    // squash when the glute is pushed down (x[1] < 0), stretch when it rises; volume kept to first order
    const double r = std::max(bones_[size_t(k)].radius, 0.03);
    const double e = std::clamp(amp_ * x[1] / r, -0.12, 0.12);
    o.s = {1.0 - 0.5 * e, 1.0 + e, 1.0 - 0.5 * e};
    return o;
}

std::array<double, 3> RiderJiggle::displace(int k, const float p[3]) const {
    const Offset o = offset(k);
    const auto& c = bones_[size_t(k)].centre;
    std::array<double, 3> d{};
    for (int i = 0; i < 3; ++i) d[size_t(i)] = (o.s[size_t(i)] - 1.0) * (double(p[i]) - c[size_t(i)]) + o.d[size_t(i)];
    return d;
}

double riderJiggleSetting(const std::string& value) {
    std::string s;
    for (char ch : value)
        if (!std::isspace(static_cast<unsigned char>(ch))) s += char(std::tolower(static_cast<unsigned char>(ch)));
    if (s == "off" || s == "0" || s == "no" || s == "false") return 0.0;
    if (s == "full" || s == "2" || s == "strong") return 1.0;
    return 0.5;  // subtle (the default)
}

double riderJiggleOutfitScale(int outfit) {
    switch (outfit) {
        case 1: return 0.35;  // riding gear: heavy trousers
        case 2: return 0.60;  // tank top + denim cutoffs: the denim is tight
        case 3: return 1.00;  // shirtless + shorts
        default: return 0.0;  // FUEL's rider: no helper bones
    }
}

int riderJiggleCheck(int* total) {
    int bad = 0, n = 0;
    auto expect = [&](bool ok, const char* what, double v) {
        ++n;
        if (!ok) ++bad;
        std::printf("jiggle check: %s %s (%.6g)\n", ok ? "ok  " : "FAIL", what, v);
    };
    JiggleTuning t;
    // 1. a step acceleration (a landing: 2 g up for 0.1 s) -> the glutes drop, overshoot, ring at ~freqHz, decay
    {
        RiderJiggle j;
        j.setAmplitude(1.0);
        std::vector<double> y;
        for (int f = 0; f < 240 * 3; ++f) {
            const double a = f < 24 ? 2 * 9.81 : 0.0;
            j.stepLocal({0, a, 0}, RiderJiggle::kStep, false);
            y.push_back(j.state(0)[1]);
        }
        double mn = 0;
        for (double v : y) mn = std::min(mn, v);
        expect(mn < -0.004 && mn >= -t.maxDown - 1e-12, "a landing pushes the glutes down within the clamp (m)", mn);
        // zero crossings after the pulse -> frequency (damped: w_d = w sqrt(1 - z^2))
        int first = -1, last = -1, n = 0;
        for (size_t i = 25; i + 1 < y.size(); ++i)
            if ((y[i] < 0) != (y[i + 1] < 0)) {
                if (first < 0) first = int(i);
                last = int(i), ++n;
            }
        const double f = n > 1 ? 0.5 * (n - 1) / ((last - first) * RiderJiggle::kStep) : 0.0;
        const double fd = t.freqHz * std::sqrt(1 - t.zeta * t.zeta);
        expect(std::fabs(f - fd) < 0.35, "rings at the damped frequency (Hz)", f);
        expect(std::fabs(y.back()) < 1e-4, "and settles within 3 s (m)", y.back());
    }
    // 2. frame-rate independence: the same 1 s of input at 30, 60 and 144 fps ends in the same state
    {
        double ref = 0;
        for (double fps : {30.0, 60.0, 120.0, 144.0}) {
            RiderJiggle j;
            j.setAmplitude(1.0);
            const int n = int(std::lround(fps * 0.5));
            for (int f = 0; f < n; ++f) j.stepLocal({0, -9.81, 0}, 1.0 / fps, false);  // 0.5 s of free fall
            const double v = j.state(0)[1];
            if (fps == 30.0) ref = v;
            expect(std::fabs(v - ref) < 2e-4, "free fall: the same state at any frame rate (m)", v);
        }
    }
    // 3. clamps: a huge sustained push stays inside the limits; seated, the downward travel stops at the seat
    {
        RiderJiggle j;
        j.setAmplitude(1.0);
        for (int f = 0; f < 480; ++f) j.stepLocal({200, 200, 200}, RiderJiggle::kStep, false);
        expect(j.state(0)[1] >= -t.maxDown - 1e-12 && j.state(0)[0] >= -t.maxSide - 1e-12 && j.state(0)[2] >= -t.maxFwd - 1e-12,
               "clamped (down m)", j.state(0)[1]);
        j.reset();
        for (int f = 0; f < 480; ++f) j.stepLocal({0, 200, 0}, RiderJiggle::kStep, true);
        expect(std::fabs(j.state(0)[1] + t.seatDown) < 1e-12, "seated: one-sided limit at the seat (m)", j.state(0)[1]);
        for (int f = 0; f < 480; ++f) j.stepLocal({0, -200, 0}, RiderJiggle::kStep, true);
        expect(std::fabs(j.state(0)[1] - t.maxUp) < 1e-12, "seated: free to rise up to maxUp (m)", j.state(0)[1]);
    }
    // 4. reset, and off = identity (no displacement at all)
    {
        RiderJiggle j;
        j.setAmplitude(1.0);
        for (int f = 0; f < 50; ++f) j.stepLocal({0, 30, 0}, RiderJiggle::kStep, false);
        j.reset();
        expect(j.state(0)[1] == 0 && j.state(1)[1] == 0, "reset clears the springs", j.state(0)[1]);
        j.setAmplitude(0.0);
        for (int f = 0; f < 50; ++f) j.stepLocal({0, 30, 0}, RiderJiggle::kStep, false);
        const float p[3] = {0.07f, 0.9f, -0.12f};
        const auto d = j.displace(0, p);
        expect(!j.active() && d[0] == 0 && d[1] == 0 && d[2] == 0, "off: inactive, zero displacement", d[1]);
    }
    // 5. world-space driving: a pelvis moving at constant velocity gives no motion; a vertical bounce does
    {
        RiderJiggle j;
        j.setAmplitude(1.0);
        anim::Mat4 M = anim::kIdentity;
        for (int f = 0; f < 120; ++f) {
            M[12] = 3.0 * f / 60.0;
            j.step(M, 1.0 / 60.0, true);
        }
        expect(std::fabs(j.state(0)[0]) < 1e-9 && std::fabs(j.state(0)[1]) < 1e-9, "constant velocity: still (m)", j.state(0)[0]);
        for (int f = 0; f < 120; ++f) {
            M[13] = 0.05 * std::sin(2 * M_PI * 4.0 * f / 60.0);
            j.step(M, 1.0 / 60.0, true);
        }
        expect(std::fabs(j.state(0)[1]) > 1e-4, "a 4 Hz bounce shakes them (m)", j.state(0)[1]);
        M[12] += 50.0;
        j.step(M, 1.0 / 60.0, true);
        expect(j.state(0)[1] == 0, "a teleport resets", j.state(0)[1]);
    }
    std::printf("jiggle check: %s\n", bad ? "FAILED" : "passed");
    if (total) *total = n;
    return bad;
}

}  // namespace fuel
