// Driver AI for race opponents: follows the checkpoint line, brakes for the
// corners ahead, backs out when stuck and asks to be put back on the line
// when that does not help.
//
// The line is the polyline start -> checkpoints -> finish (FUEL's courses give
// nothing finer). The driver steers at a point a speed-dependent distance
// ahead along it, slows for each upcoming corner to a speed set by the corner's
// angle (braking distance from an assumed deceleration), and caps its speed at
// a per-driver share of the vehicle's top speed, scaled by rubber-banding.
// Static obstacles on the way (buildings, rocks, log piles) and terrain too steep
// to climb are seen with a fan of rays: it steers for the free heading nearest
// the line and slows if boxed in. Each reset that does not lead to the next
// checkpoint puts it further along the line.
#pragma once

#include "physics/vehicle_physics.hpp"

#include <array>
#include <cstddef>
#include <functional>
#include <vector>

namespace fuel {

// The course as the driver sees it: line[0] is the start, line[1 + k] the k-th
// target (checkpoints per lap, then the finish).
struct RaceLine {
    std::vector<std::array<float, 3>> points;
    std::vector<float> along;  // path length from the start to each point
    float length() const { return along.empty() ? 0.0f : along.back(); }
    void build(const std::array<float, 3>& start, const std::vector<std::array<float, 3>>& targets);
    // Distance along the line of the point (x, z) heading for target `next` (0-based target index).
    float progress(size_t next, double x, double z) const;
    // A point on segment `next` (towards target `next`): the foot of (x, z), moved `ahead` metres
    // further along the line (crossing into later segments). Writes the line's heading there.
    std::array<float, 3> pointAhead(size_t next, double x, double z, float ahead, float* heading = nullptr) const;
};

class RaceDriver {
public:
    // `skill` 0.85..1: share of the top speed this driver uses (and how late it brakes).
    explicit RaceDriver(float skill = 1.0f) : skill_(skill) {}

    struct Sense {
        double x = 0, z = 0;
        float heading = 0;     // radians, 0 = +z
        float speedKmh = 0;    // signed, along the vehicle's forward axis
        float upY = 1;         // vehicle up vector's y (upside down < 0)
        float topSpeedKmh = 150;
        // Free distance ahead along a heading, up to a length (DriveWorld::clearance), counting
        // steep terrain as blocked when `slopes`; none = open ground.
        std::function<float(float yaw, float length, bool slopes)> clearance;
    };
    // One control step (call at a steady rate with `dt` seconds). `next` is the target
    // the racer heads for; `rubber` scales the speed cap (1 = none).
    DriveInput drive(const RaceLine& line, size_t next, const Sense& s, float dt, float rubber);
    // Set by drive when the racer should be put back on the line: where and facing which way.
    bool wantsReset() const { return resetWanted_; }
    std::array<float, 3> resetPoint() const { return resetAt_; }
    float resetHeading() const { return resetHeading_; }
    void didReset();  // the owner moved the vehicle
    void restart();   // a new race: forget the stuck state
    float skill() const { return skill_; }
    int resets() const { return resets_; }

private:
    float skill_;
    float stuckTime_ = 0, reverseTime_ = 0, upsideDown_ = 0, stuckTotal_ = 0;  // seconds
    float bestRemaining_ = 1e30f, sinceProgress_ = 0;  // distance to the target, seconds since it last shrank by 5 m
    size_t lastNext_ = size_t(-1);
    float dodge_ = 0;  // heading offset (radians) chosen last step to get round an obstacle
    bool resetWanted_ = false;
    std::array<float, 3> resetAt_{};
    float resetHeading_ = 0;
    int resets_ = 0;
    int failedResets_ = 0;  // resets since the last checkpoint
};

}  // namespace fuel
