// Session recorder for `fuelview --record DIR`: a per-frame timing log and
// timestamped frame grabs, so a play session can be reviewed afterwards.
//
//   DIR/frames.csv   one row per frame: time, frame interval, CPU time of the
//                    physics, streaming and render encoding, GPU time, and the
//                    player's speed and position. Long frames are flagged.
//   DIR/marks.txt    times the player pressed K ("glitch here").
//   DIR/f_<ms>.jpg   a frame grab every `interval` seconds, on long frames
//                    and on K.
#pragma once

#import <Metal/Metal.h>

#include <cstdio>
#include <memory>
#include <string>

namespace fuel {

class Recorder {
public:
    // Returns false (and records nothing) when DIR cannot be created.
    bool open(const std::string& dir, float interval = 0.5f);
    bool active() const { return log_ != nullptr; }
    const std::string& dir() const { return dir_; }
    ~Recorder();

    // Seconds since open().
    double now() const;

    struct Frame {
        double dt = 0;                                 // since the last frame, s
        double physicsMs = 0, streamMs = 0, encodeMs = 0;  // CPU
        float speedKmh = 0;
        int gear = 0, wheelsOnGround = 0, vehicles = 0;
        float x = 0, y = 0, z = 0;
    };
    // Logs the frame and decides whether to grab it: call before presenting,
    // then pass the drawable texture to grab() when it returns true.
    bool frame(const Frame& f, const char* note = nullptr);
    void mark(const char* what);  // K pressed: grab the next frame
    // Copies `tex` (BGRA8, not framebuffer-only) into `cb` and writes a JPEG
    // when the command buffer completes. Also records the GPU time.
    void grab(id<MTLCommandBuffer> cb, id<MTLTexture> tex);
    void watchGpu(id<MTLCommandBuffer> cb);
    // Every frame: a small copy of `tex` into a ring of the last ~1.5 s. K writes the whole ring
    // (DIR/k<mark ms>/r_<ms>.jpg), so a one-frame glitch seen just before K is kept. A frame whose
    // mean brightness jumps away from both neighbours is logged as FLICKER and the half second up
    // to it written to DIR/flicker_<ms>/.
    void capture(id<MTLCommandBuffer> cb, id<MTLTexture> tex);

private:
    std::string dir_;
    std::FILE* log_ = nullptr;
    std::FILE* marks_ = nullptr;
    double start_ = 0, nextGrab_ = 0;
    float interval_ = 0.5f;
    bool grabNext_ = false;
    std::string grabNote_;
    int written_ = 0;

    struct Ring;  // the rolling small-frame buffer (recorder.mm)
    std::shared_ptr<Ring> ring_;
};

}  // namespace fuel
