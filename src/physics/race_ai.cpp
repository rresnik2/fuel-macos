#include "race_ai.hpp"

#include <algorithm>
#include <cmath>

namespace fuel {

namespace {

constexpr float kPi = 3.14159265f;
constexpr float kDecel = 5.5f;         // m/s^2 the driver counts on when braking (off-road tyres)
constexpr float kMinCornerKmh = 28.0f;  // the slowest it takes any corner

float wrap(float a) { return std::remainder(a, 2.0f * kPi); }

// The speed (km/h) to take a corner that turns by `angle` radians, for a vehicle
// whose own limit is `vmax`: full speed when straight, ~half for a right angle.
float cornerSpeed(float angle, float vmax) {
    const float c = std::cos(std::min(std::fabs(angle), kPi) * 0.5f);
    return kMinCornerKmh + (vmax - kMinCornerKmh) * c * c * c * c;
}

}  // namespace

void RaceLine::build(const std::array<float, 3>& start, const std::vector<std::array<float, 3>>& targets) {
    points.clear();
    along.clear();
    points.push_back(start);
    points.insert(points.end(), targets.begin(), targets.end());
    float total = 0;
    for (size_t i = 0; i < points.size(); ++i) {
        if (i > 0) total += std::hypot(points[i][0] - points[i - 1][0], points[i][2] - points[i - 1][2]);
        along.push_back(total);
    }
}

float RaceLine::progress(size_t next, double x, double z) const {
    if (next + 1 >= points.size()) return length();
    const auto& b = points[next + 1];
    const float seg = along[next + 1] - along[next];
    const float d = float(std::hypot(x - b[0], z - b[2]));
    return along[next + 1] - std::min(d, seg + 50.0f);  // a little behind the segment start still counts
}

std::array<float, 3> RaceLine::pointAhead(size_t next, double x, double z, float ahead, float* heading) const {
    if (points.size() < 2) return points.empty() ? std::array<float, 3>{float(x), 0, float(z)} : points[0];
    size_t i = std::min(next, points.size() - 2);  // segment points[i] -> points[i + 1]
    const auto& a = points[i];
    const auto& b = points[i + 1];
    const float dx = b[0] - a[0], dz = b[2] - a[2];
    const float len2 = std::max(dx * dx + dz * dz, 1e-6f);
    float t = std::clamp(float(((x - a[0]) * dx + (z - a[2]) * dz) / len2), 0.0f, 1.0f);
    float left = ahead;
    for (;;) {
        const auto& p = points[i];
        const auto& q = points[i + 1];
        const float len = std::hypot(q[0] - p[0], q[2] - p[2]);
        const float rest = (1.0f - t) * len;
        if (left <= rest || i + 2 >= points.size()) {
            const float u = len > 1e-3f ? std::min(1.0f, t + left / len) : 1.0f;
            if (heading) *heading = std::atan2(q[0] - p[0], q[2] - p[2]);
            return {p[0] + (q[0] - p[0]) * u, p[1] + (q[1] - p[1]) * u, p[2] + (q[2] - p[2]) * u};
        }
        left -= rest;
        ++i;
        t = 0;
    }
}

void RaceDriver::didReset() {
    resetWanted_ = false;
    stuckTime_ = reverseTime_ = upsideDown_ = sinceProgress_ = stuckTotal_ = 0;
    bestRemaining_ = 1e30f;
    ++resets_;
    ++failedResets_;
}

void RaceDriver::restart() {
    resetWanted_ = false;
    stuckTime_ = reverseTime_ = upsideDown_ = sinceProgress_ = stuckTotal_ = 0;
    bestRemaining_ = 1e30f;
    lastNext_ = size_t(-1);
    resets_ = failedResets_ = 0;
}

DriveInput RaceDriver::drive(const RaceLine& line, size_t next, const Sense& s, float dt, float rubber) {
    if (line.points.size() < 2 || next + 1 >= line.points.size()) return {0, 1, 0, false};
    const float speed = s.speedKmh / 3.6f;  // m/s
    const float vmax = s.topSpeedKmh * skill_ * rubber;

    // Steer at a point ahead on the line (further at speed): pure pursuit.
    const float look = std::clamp(9.0f + std::fabs(speed) * 0.8f, 10.0f, 34.0f);
    float lineHeading = 0;
    auto target = line.pointAhead(next, s.x, s.z, look, &lineHeading);
    {
        // Keep to its lane, closing in on the line within 50 m of the gate.
        const auto& gate = line.points[next + 1];
        const float lane = lane_ * std::clamp((float(std::hypot(gate[0] - s.x, gate[2] - s.z)) - 10.0f) / 40.0f, 0.0f, 1.0f);
        target[0] += std::cos(lineHeading) * lane;
        target[2] -= std::sin(lineHeading) * lane;
    }
    float want = std::atan2(target[0] - float(s.x), target[2] - float(s.z));

    // Obstacles: try headings fanning out from the line (the side picked last time first)
    // and take the first one free for the distance needed to react.
    float blockedSpeed = 1e9f;
    if (s.clearance) {
        const float range = std::clamp(12.0f + std::fabs(speed) * 1.4f, 18.0f, 50.0f);
        // Not past the target point when it is close (the gate itself may sit between obstacles).
        const float need = std::min(range, std::max(8.0f, float(std::hypot(target[0] - s.x, target[2] - s.z)) + 4.0f));
        float best = -1, bestOffset = 0;
        bool found = false;
        const float side = dodge_ >= 0 ? 1.0f : -1.0f;
        // Slopes first, fanning wider (up to 78 degrees: a diagonal makes a slope gentler); if no
        // heading is free, the one that gets furthest when that is a fair way, else any heading
        // free of objects (a slope everywhere ahead: take it anyway).
        for (bool slopes : {true, false}) {
            best = -1;
            for (int k = 0; k <= 8 && !found; ++k)
                for (float sign : {side, -side}) {
                    const float offset = sign * float(k) * (slopes ? 0.17f : 0.13f);
                    const float free = s.clearance(want + offset, need, slopes);
                    if (free > best) best = free, bestOffset = offset;
                    if (free >= need) {
                        found = true;
                        bestOffset = offset;
                        break;
                    }
                    if (k == 0) break;
                }
            if (found || (slopes && best >= 16.0f)) break;
        }
        found = found || best >= need;
        dodge_ = found && bestOffset == 0 ? dodge_ * 0.9f : bestOffset;
        want += bestOffset;
        if (!found) blockedSpeed = std::sqrt(2.0f * kDecel * std::max(0.0f, best - 4.0f)) * 3.6f;
    }
    const float err = wrap(want - s.heading);
    float steer = std::clamp(-err * 2.2f, -1.0f, 1.0f);  // positive steer turns right (decreasing heading)

    // The speed to hold: the cap, the corners ahead (each reachable with braking at kDecel),
    // and the turn needed right now.
    float allowed = vmax;
    {
        const auto& p = line.points;
        float dist = float(std::hypot(s.x - p[next + 1][0], s.z - p[next + 1][2]));
        for (size_t k = next + 1; k + 1 < p.size() && dist < 400.0f; ++k) {
            const float in = std::atan2(p[k][0] - p[k - 1][0], p[k][2] - p[k - 1][2]);
            const float out = std::atan2(p[k + 1][0] - p[k][0], p[k + 1][2] - p[k][2]);
            // The gate is ~25 m wide: start the turn a little before it.
            const float vc = cornerSpeed(wrap(out - in), vmax) / 3.6f;
            const float reach = std::max(0.0f, dist - 12.0f);
            allowed = std::min(allowed, std::sqrt(vc * vc + 2.0f * kDecel * (0.8f + 0.2f * skill_) * reach) * 3.6f);
            dist += line.along[k + 1] - line.along[k];
        }
        allowed = std::min({allowed, cornerSpeed(err * 1.6f, vmax), std::max(blockedSpeed, 12.0f)});
    }
    const float kmh = s.speedKmh;
    DriveInput in;
    if (kmh > allowed + 4.0f) {
        in.brake = std::clamp((kmh - allowed) / 20.0f, 0.3f, 1.0f);
    } else {
        in.throttle = std::clamp((allowed - kmh) / 8.0f, 0.25f, 1.0f);
        if (std::fabs(err) > 1.3f && kmh > 20.0f) in.throttle = 0.3f;  // pointing the wrong way: turn first
    }
    in.handbrake = std::fabs(err) > 1.2f && kmh > 25.0f && kmh < 70.0f;  // swing round tight hairpins
    in.steer = steer;

    // Progress watch: the distance to the target should shrink by 5 m every 12 s.
    const float remaining = float(std::hypot(s.x - line.points[next + 1][0], s.z - line.points[next + 1][2]));
    if (next != lastNext_) {
        lastNext_ = next;
        failedResets_ = 0;
        bestRemaining_ = remaining;
        sinceProgress_ = 0;
    } else if (remaining < bestRemaining_ - 5.0f) {
        bestRemaining_ = remaining;
        sinceProgress_ = 0;
    } else {
        sinceProgress_ += dt;
    }
    upsideDown_ = s.upY < 0.3f ? upsideDown_ + dt : 0.0f;
    stuckTotal_ = std::fabs(kmh) < 4.0f ? stuckTotal_ + dt : std::max(0.0f, stuckTotal_ - dt * 0.5f);

    // Stuck against something: back off with the opposite lock, then try again.
    stuckTime_ = (std::fabs(kmh) < 4.0f && reverseTime_ <= 0) ? stuckTime_ + dt : 0.0f;
    if (stuckTime_ > 1.5f) {
        reverseTime_ = 1.3f;
        stuckTime_ = 0;
    }
    if (reverseTime_ > 0) {
        reverseTime_ -= dt;
        in = {0, 1, -steer, false};
    }

    // Still nowhere after that (or lying on its roof, or circling): back on the line, a little
    // ahead of where it is.
    if (!resetWanted_ && (stuckTotal_ > 4.0f || upsideDown_ > 4.0f || sinceProgress_ > 12.0f)) {
        resetWanted_ = true;
        // 6 m ahead, then 30 m more for each reset that did not help; never past the target.
        const float ahead = std::min(6.0f + 30.0f * float(failedResets_), std::max(0.0f, remaining - 30.0f));
        resetAt_ = line.pointAhead(next, s.x, s.z, ahead, &resetHeading_);
    }
    return in;
}

}  // namespace fuel
