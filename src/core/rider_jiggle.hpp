// PHYSIQUE (port deviation, user request 2026-11-06; ~/re-work/fuel/notes/physique.md): secondary motion of the player's
// custom rider's glutes. Two helper bones (left / right glute) hang from the pelvis (Hips); they are appended after FUEL's
// skeleton in the asset's bone table and never enter FUEL's bone arrays: Rider::skin adds their offset to the vertices
// they weight, on top of the normal skinning, so "off" is exactly the plain skinning (no extra bones, no extra terms).
//
// Each bone is a damped spring (about 4 Hz, damping ratio about 0.3) in the pelvis's bind frame, driven by the world
// acceleration of its rest centre (animation + vehicle: suspension, landings, ground rumble, turns, standing on the
// pegs). Inertia: x'' = -w^2 x - 2 z w x' - g * a (a = the centre's acceleration in the pelvis frame, gravity excluded:
// the bind pose already hangs under 1 g; in the air a = -g, so the glutes rise, and they drop on the landing). Mostly
// vertical (lateral and fore-aft gains smaller). Clamped, and while seated the downward travel (into the seat, along the
// seat normal = the pelvis's -y) is limited one-sidedly. Stepped at a fixed 240 Hz with the frame's mean acceleration
// (frame-rate independent), reset on respawn / teleport / camera cuts and on a jump of the pelvis (> 1.5 m in a frame).
#pragma once

#include "anim_clip.hpp"

#include <array>
#include <cstdint>
#include <string>

namespace fuel {

struct JiggleBone {
    int base = -1;                     // the skeleton bone it hangs from (Hips)
    std::array<double, 3> centre{};    // rest centre, bind (rider model) space
    double radius = 0.09;              // for the squash / stretch
};

struct JiggleTuning {
    double freqHz = 4.2, zeta = 0.30;
    double gainY = 0.55, gainX = 0.22, gainZ = 0.16;  // response per m/s^2, x'' units (dimensionless)
    double maxUp = 0.014, maxDown = 0.010, maxSide = 0.006, maxFwd = 0.006;  // metres, at amplitude 1
    double seatDown = 0.0025;  // metres: the most the glute may sink into the seat while seated
};

class RiderJiggle {
public:
    static constexpr double kStep = 1.0 / 240.0;
    static constexpr int kBones = 2;
    struct Offset {
        std::array<double, 3> d{};          // translation, bind space
        std::array<double, 3> s{1, 1, 1};   // scale about the centre, bind axes
    };

    // amplitude: setting (off 0, subtle 0.5, full 1) x outfit (riding gear 0.35, tank top + cutoffs 0.6, shirtless 1).
    void setAmplitude(double a) { amp_ = a < 0 ? 0 : a; }
    double amplitude() const { return amp_; }
    bool active() const { return amp_ > 0; }
    void setBones(const std::array<JiggleBone, kBones>& b) { bones_ = b; }
    const std::array<JiggleBone, kBones>& bones() const { return bones_; }
    const JiggleTuning& tuning() const { return tune_; }
    void setTuning(const JiggleTuning& t) { tune_ = t; }

    void reset();
    // `baseSkin` = the base bone's skin matrix (bind -> world, row-vector: p_world = p_bind * M) this frame; dt seconds.
    void step(const anim::Mat4& baseSkin, double dt, bool seated);
    // Drives the springs with a bind-frame acceleration directly (the check vectors), dt seconds.
    void stepLocal(const std::array<double, 3>& accel, double dt, bool seated);
    // The current offset of bone k, already scaled by the amplitude and clamped.
    Offset offset(int k) const;
    // The displacement (bind space) that bone k's offset gives the bind-space point p.
    std::array<double, 3> displace(int k, const float p[3]) const;
    // Raw spring state (check vectors).
    const std::array<double, 3>& state(int k) const { return x_[size_t(k)]; }

private:
    void integrate(const std::array<std::array<double, 3>, kBones>& a, double dt, bool seated);
    std::array<JiggleBone, kBones> bones_{};
    JiggleTuning tune_;
    double amp_ = 0, acc_ = 0;
    std::array<std::array<double, 3>, kBones> x_{}, v_{};
    std::array<std::array<double, 3>, kBones> pPrev_{}, velPrev_{};
    int history_ = 0;  // 0: no previous position, 1: a position, 2: a position and a velocity
};

// `rider_jiggle = off | subtle | full` (settings.txt, default subtle) -> 0 / 0.5 / 1.
double riderJiggleSetting(const std::string& value);
// The outfit's share (RiderOutfit::outfit): riding gear 0.35, tank top + cutoffs 0.6, shirtless + shorts 1.0, FUEL 0.
double riderJiggleOutfitScale(int outfit);
// The check vectors (fuelview --animcheck): spring step and decay, frequency, clamps, seat limit, reset, off = identity.
// Returns the failures; *total = the number of vectors.
int riderJiggleCheck(int* total = nullptr);

}  // namespace fuel
