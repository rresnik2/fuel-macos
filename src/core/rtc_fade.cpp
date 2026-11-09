// FUEL's screen fader, RTC fade messages, scripted camera shake and race-end grade (see rtc_fade.hpp).
#include "rtc_fade.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <fstream>
#include <sstream>

namespace fuel::rtc {

// 0x438190: fade to black (white: mode 6). Arguments 4/5 of FUEL's call = white, strip (+0xe02).
void Fader::fadeOut(float duration, bool white, bool strip) {
    delay_ = 0;  // +0xe01 clear in every caller seen
    duration_ = duration;
    strip_ = strip;
    mode_ = white ? 6 : 2;
    if (value_ == 0.0f) {
        elapsed_ = 0;
        value_ = duration == 0.0f ? 1.0f : 0.0f;
        skipFrame_ = true;
    } else {
        skipFrame_ = false;
        elapsed_ = value_ == 1.0f ? duration : duration * value_;
    }
}

// 0x438030: fade from black (white: mode 5).
void Fader::fadeIn(float duration, bool white, bool strip) {
    delay_ = 0;
    duration_ = duration;
    strip_ = strip;
    mode_ = white ? 5 : 1;
    if (value_ == 1.0f) {
        elapsed_ = 0;
        skipFrame_ = true;
        value_ = 1.0f;
    } else {
        skipFrame_ = false;
        elapsed_ = value_ == 0.0f ? duration : (1.0f - value_) * duration;
    }
}

void Fader::holdBlack() {  // 0x4382e0
    skipFrame_ = false, elapsed_ = 0, mode_ = 3, delay_ = 0, duration_ = 0, value_ = 1;
}

void Fader::clear() {  // 0x438480 (and 0x53e1a0 when skipping)
    mode_ = 0, elapsed_ = 0, skipFrame_ = false, delay_ = 0, duration_ = 0, value_ = 0;
}

// vf17 0x438560 (the fade part).
bool Fader::update(float dt) {
    if (mode_ == 0) return false;
    if (skipFrame_) {
        skipFrame_ = false;
        return false;
    }
    if (delay_ <= 0) elapsed_ += dt;
    else delay_ -= dt;
    if (elapsed_ <= duration_) {
        if (mode_ == 1 || mode_ == 5) {
            const float hold = duration_ * 0.1f;
            value_ = hold < elapsed_ ? 1.0f - (elapsed_ - hold) / (duration_ - hold) : 1.0f;
        } else if (mode_ == 2 || mode_ == 6) {
            const float ramp = duration_ - duration_ * 0.1f;
            value_ = elapsed_ < ramp ? elapsed_ / ramp : 1.0f;
        }
        return false;
    }
    elapsed_ = duration_;
    const bool done = mode_ != 3 && mode_ != 7;  // the callback (0x638040) is posted for the fades only
    if (mode_ == 1 || mode_ == 5) mode_ = 0, value_ = 0;
    else if (mode_ == 6) mode_ = 7, value_ = 1;
    else if (mode_ == 2) mode_ = 3, value_ = 1;
    return done;
}

// RtcScriptAgent_G vf43 0x536d40 case 0x98 and the commands 0x1a..0x1d it runs (table 0xa02bb0). The strip flag of
// the timed paths is "record value == 1.0", the value being 1.0 for kinds >= 5 (so TOBLACKSTRIP 9 / FROMBLACKSTRIP 8).
void fadeMessage(Fader& f, uint32_t kind, bool skipping) {
    const int k = int(kind);
    const bool strip = k >= 5;
    if (k == 6) {  // TOWHITE: 0x53e280 -> 0x438190(0.1, white, strip); nothing when skipping
        if (!skipping) f.fadeOut(0.1f, true, true);
        return;
    }
    if (k == 5) {  // FROMWHITE: 0x53e300 -> 0x438030(0.3, white, strip)
        if (!skipping) f.fadeIn(0.3f, true, true);
        return;
    }
    if (k == 4 || k == 2 || k == 9) {  // 0x53e0d0: to black 0.1 s (instant when skipping)
        if (skipping) f.fadeOut(0.0f, false, false);
        else f.fadeOut(0.1f, false, strip);
    }
    if (k == 4 || k == 1 || k == 8) {  // 0x53e1a0: from black 0.3 s (skipping: the fader is cleared)
        if (skipping) f.clear();
        else f.fadeIn(0.3f, false, strip);
    }
}

std::array<float, 3> CameraShake::update(float dt) {
    if (t <= 0) return {0, 0, 0};
    const float a = (1.0f - t) * amplitude * t * 0.05f;
    const std::array<float, 3> o = {a * std::cos(30.0f * t), a * std::cos(40.0f * t), a * std::cos(50.0f * t)};
    t -= dt;
    if (t <= 0) t = -1;
    return o;
}

void EndGrade::update(float dt) {
    if (dir > 0) {
        ramp += dt;
        if (ramp > 1.0f) ramp = 1.0f, dir = 0;
    } else if (dir < 0) {
        ramp -= dt;
        if (ramp < 0.0f) ramp = 0.0f, dir = 0;
    }
}

std::vector<float> readPpRows(const std::string& path, int count) {
    std::vector<float> pp(size_t(std::max(count, 0)), 0.0f);
    std::ifstream in(path, std::ios::binary);
    std::string line;
    while (std::getline(in, line)) {
        std::istringstream s(line);
        std::string tag;
        int n = -1;
        float v = 0;
        if (s >> tag >> n >> v && tag == "PP" && n >= 0 && n < count) pp[size_t(n)] = v;
    }
    return pp;
}

}  // namespace fuel::rtc
