// FUEL's screen fader and the cut-scene presentation around it (P67; notes/p67-cutscenes.md).
//
// Fader = BaseInGameDatas_G's fade (the in-game data object at game+0x3310), decoded from FUEL.exe:
//  - fadeOut 0x438190 (mode 2, white 6): from the current value; value = elapsed / (0.9 dur), 1 for the last 10 %;
//    at the end mode 3 (7) holds the colour.
//  - fadeIn 0x438030 (mode 1, white 5): value = 1 for the first 10 %, then 1 - (elapsed - 0.1 dur) / (0.9 dur);
//    at the end mode 0 (off).
//  - holdBlack 0x4382e0 (mode 3, value 1), clear 0x438480 (mode 0, value 0).
//  - update = vf17 0x438560: a fade started from a settled value skips its first frame (+0xe00); a fade from
//    a fade in progress resumes at the same value (elapsed = dur x value, or (1 - value) x dur).
//  - draw = vf19 0x438be0: a full-screen quad, black (modes 1..3) or white (5..7), alpha min(1.5 value, 1);
//    the sound volume slot 1 (0x554200 -> SoundManager vt+0x40) follows 1 - value, 1 while off.
// RTC message 0x98 (RtcScriptAgent_G vf43 0x536d40; keywords from the RTC script parser 0x55f0f0):
//   FROMBLACK 1, TOBLACK 2, FADE_B 4 (both), FROMWHITE 5, TOWHITE 6, FROMBLACKSTRIP 8, TOBLACKSTRIP 9.
//   Command table 0xa02bb0: 0x1a 0x53e0d0 to black 0.1 s, 0x1b 0x53e1a0 from black 0.3 s, 0x1c 0x53e280 to white
//   0.1 s, 0x1d 0x53e300 from white 0.3 s. Skipping: to black is instant (0x438190(0)), from black clears the fader.
// STARTRTCFX (message 0xa3 -> command 0x15, 0x53e740): SHAKECAM 1 = the game camera's scripted shake (+0x344 = 1,
//   +0x348 = 0.3, radial blur flag 0x8000 off), SHAKEPAD 2 = pad rumble, GLOW 3 = "ebe on|off", BLUR 4 = renderer
//   flag 0x1000 (B&W). No shipped RTC carries it.
#pragma once

#include <array>
#include <cstdint>
#include <string>
#include <vector>

namespace fuel::rtc {

class Fader {
public:
    void fadeOut(float duration, bool white = false, bool strip = false);  // 0x438190
    void fadeIn(float duration, bool white = false, bool strip = false);   // 0x438030
    void holdBlack();                                                      // 0x4382e0
    void clear();                                                          // 0x438480
    // vf17 0x438560. True on the frame a fade completes (FUEL posts its callback message, 0x74 for RTC fades).
    bool update(float dt);

    int mode() const { return mode_; }
    float value() const { return value_; }
    bool active() const { return mode_ != 0; }
    bool white() const { return mode_ >= 5 && mode_ <= 7; }
    bool strip() const { return strip_; }
    // vf19 0x438be0: the quad's alpha and the sound volume slot 1.
    float alpha() const { return mode_ == 0 ? 0.0f : std::min(1.0f, value_ * 1.5f); }
    float soundGain() const { return mode_ == 0 ? 1.0f : 1.0f - value_; }
    // RGBA of the quad (rgb black or white).
    std::array<float, 4> rgba() const {
        const float c = white() ? 1.0f : 0.0f;
        return {c, c, c, alpha()};
    }

private:
    int mode_ = 0;           // +0xdec
    float delay_ = 0;        // +0xdf0
    float elapsed_ = 0;      // +0xdf4
    float duration_ = 0;     // +0xdf8
    float value_ = 0;        // +0xdfc
    bool skipFrame_ = false; // +0xe00
    bool strip_ = false;     // +0xe02
};

// Message 0x98 of kind `kind`; `skipping` = the RTC is being seeked or shut (0x72aeb0 / 0x536230).
void fadeMessage(Fader& f, uint32_t kind, bool skipping);

// The scripted camera shake (0x4156e0, timer counted down in 0x412e80): t = +0x344 (from 1), A = +0x348;
// a = (1 - t) A t 0.05; the camera position moves by a (cos 30t, cos 40t, cos 50t).
struct CameraShake {
    float t = -1, amplitude = 0;
    void start(float a = 0.3f) { t = 1.0f, amplitude = a; }  // STARTRTCFX SHAKECAM / camera message 0x13
    // Advances the timer and returns the offset to add to the camera position.
    std::array<float, 3> update(float dt);
};

// MissionHUD 0x4b6050 (+0xd38 / +0xd3c): the race-end grade. SetState 6/7 starts the ramp up (+1 / s, to 1),
// SetState 0 the ramp down; viewport tint = (PP39, PP40, PP41) at PP43 x ramp, radial blur = PP42 x ramp.
struct EndGrade {
    int dir = 0;      // +0xd38: 1 up, -1 down, 0 settled
    float ramp = 0;   // +0xd3c
    void update(float dt);
};

// pp001.tsc rows ("PP n value"); missing rows stay 0.
std::vector<float> readPpRows(const std::string& pp001Path, int count);

}  // namespace fuel::rtc
