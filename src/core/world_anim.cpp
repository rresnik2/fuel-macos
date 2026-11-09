// PK29: PlayAnim_Z channel clock (0x7bc150 / 0x7bc3d0); see world_anim.hpp.
#include "world_anim.hpp"

namespace fuel::anim {

void AnimChannel::play(const Clip* clip) {
    clip_ = clip;
    hips_ = clip ? clip->track(kHipsCrc) : -1;
    t_ = 0;
    flags_ = uint16_t(flags_ & ~(2 | 4 | 0x10));  // 0x7bcd40: & 0xffe9
    count_ = 0;
    stopped_ = clip == nullptr;
}

bool AnimChannel::advance(double dt, std::vector<AnimPass>& passes) {
    if (clip_ == nullptr || stopped_) return false;
    const double from = t_;  // 0x7bc150
    t_ += dt;                // speed 1.0
    const double dur = clip_->duration;
    if (dur <= 0) return false;
    if (dur < t_ && !(flags_ & 2)) {
        if (!clip_->loops()) {  // channel flag 0x40 is never set for the world objects
            t_ = dur;
            markers(from, dur, passes);
            stopped_ = true;  // vf16 0x40 finished, no queued anim -> vt+0x5c(0): Play without anim, the pose holds
            return true;
        }
        while (dur < t_) t_ -= dur;  // the key cursors reset (0x7bd400): the samplers here search from the start
    }
    markers(from, dur, passes);
    return false;
}

void AnimChannel::markers(double from, double to, std::vector<AnimPass>& passes) {  // 0x7bc3d0
    if (!(flags_ & 0x10)) {
        std::vector<std::pair<float, Clip::Msg>> ev;
        for (int guard = 0; guard < 64; ++guard) {  // [port] guard: FUEL rescans until no marker acts
            if (to < t_) t_ = to;
            ev.clear();
            if (hips_ >= 0) clip_->eventWindow(hips_, from, t_, to, ev);  // 0x74a0b0
            if (ev.empty()) break;
            bool again = false;
            for (const auto& [kt, m] : ev) {
                if (m.kind == 0xc && !(flags_ & 2)) {
                    loopStart_ = kt;
                    flags_ |= 2;
                    if (!(from <= loopStart_)) from = 0;
                    again = true;
                } else if (m.kind == 0xd && (flags_ & 2)) {
                    if (flags_ & 4) {
                        const uint16_t c = count_;
                        count_ = uint16_t(c - 1);
                        if (c != 0 && count_ == 0) {  // the last run: play on past the marker
                            flags_ &= uint16_t(~6);
                            passes.push_back({from, kt, to});
                            from = kt;
                        }
                    } else {
                        count_ = uint16_t(m.a);
                        flags_ |= 4;
                    }
                    again = true;
                    if (flags_ & 4) {  // back to the loop start with the time past the marker
                        const double x = (double(kt) > t_) ? to : t_;
                        const double excess = x - kt;
                        t_ = kt;
                        passes.push_back({from, kt, to});
                        from = loopStart_;
                        t_ = loopStart_ + excess;
                    }
                }
            }
            if (!again) break;
        }
    }
    passes.push_back({from, t_, to});
}

}  // namespace fuel::anim
