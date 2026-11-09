// PK29: the PlayAnim_Z channel clock of FUEL's animated world objects (IT_ANIMATED / BaseSharingPlayAnimAgent_G),
// decoded from FUEL.exe (notes ~/re-work/fuel/notes/pk29.md, p98 b027 / b053):
//   - 0x7bc150 channel update: t += dt (speed 1); past the end, unless a loop section runs (flag 2): a clip without
//     the loop flag (+0x14 bit 0) stops at its end (one last pass to the end, then vt+0x5c(0) = no anim: the pose
//     holds), a looping clip wraps (t -= dur while t > dur, key cursors reset);
//   - 0x7bc3d0 in-animation loop markers: HIPS-track (bone id 0x19) events 0xc / 0xd in the window (0x74a0b0):
//     0xc sets the loop start (flag 2); the first 0xd sets the count a (flag 4), every later one counts down
//     (count 0 = forever) and jumps back to the loop start with the excess past the marker (cursors reset);
//   - each step ends in 0x7bc5c0 passes (pose at `time`, events of the window [from, time) / [from, end], the
//     material and visibility tracks), returned here as AnimPass records for the caller to apply.
#pragma once

#include "anim_clip.hpp"

#include <cstdint>
#include <vector>

namespace fuel::anim {

constexpr uint32_t kHipsCrc = 0xfeeaf0f3;  // AsoboCrc("HIPS"), registry id 0x19 (25)

struct AnimPass {  // one 0x7bc5c0 call
    double from = 0, time = 0, end = 0;
};

class AnimChannel {
public:
    // PlayAnim_Z::Play 0x7bcd40: the clip from t = 0, channel flags &= ~(2 | 4 | 0x10).
    void play(const Clip* clip);
    const Clip* clip() const { return clip_; }
    double time() const { return t_; }
    bool running() const { return clip_ != nullptr && !stopped_; }
    // 0x7bc150 + 0x7bc3d0: appends this step's passes; true when a non-looping clip reached its end this step (the
    // channel then stops: vt+0x5c with no anim).
    bool advance(double dt, std::vector<AnimPass>& passes);
    // For the checks.
    double loopStart() const { return loopStart_; }
    uint16_t flags() const { return flags_; }

private:
    void markers(double from, double to, std::vector<AnimPass>& passes);  // 0x7bc3d0
    const Clip* clip_ = nullptr;
    int hips_ = -1;
    double t_ = 0, loopStart_ = 0;  // +4, +0xc
    uint16_t count_ = 0, flags_ = 0;  // +0x1c, +0x1e (2 loop section, 4 counting)
    bool stopped_ = false;
};

}  // namespace fuel::anim
