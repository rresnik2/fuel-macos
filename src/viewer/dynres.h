// P93: the dynamic-resolution controller (settings.txt `dynamic_resolution`, `dynamic_resolution_min`).
//
// The renderer draws the 3-D scene and its post chain at `scale` x the output size per axis and upscales the result
// (a sharpened bilinear, or MetalFX spatial); the HUD and menus are drawn afterwards at the output size. This class
// picks the scale from the GPU time of completed frames (the time each frame added on the GPU: GPUEnd minus the later
// of its GPUStart and the previous frame's GPUEnd, so frames in flight are not double-counted).
//
// Model: a frame rendered at scale s costs N (a + (1 - a) s^2) + U, where N is the frame's cost at the output size,
// a the share that does not scale with the pixel count (shadow slices, cube face, vertex work, the 128 x 128 post
// targets) and U the upscale's own cost (0 at native). Each report gives an estimate of N.
//   - Down: the fast estimate of N (EMA) asks for a smaller scale than the current one -> step down towards it, at
//     most kMaxDrop per frame (a fraction of a percent of the width per frame is not visible).
//   - Up: only when the third-worst N of the last kWindow frames would still fit under raiseMs (< targetMs: hysteresis) at a
//     higher scale, for kHold frames in a row; then step up at most kMaxRaise per frame. Native (no upscale) as soon
//     as the worst recent frame would fit there.
// Pure arithmetic, no Metal: unit-testable and deterministic for a given sequence of reports.
#pragma once

#include <algorithm>
#include <array>
#include <cmath>
#include <functional>

namespace fuel {

class DynamicResolution {
public:
    struct Params {
        float minScale = 0.8f;      // settings.txt dynamic_resolution_min (clamped to 0.5 .. 1)
        float targetMs = 11.5f;     // keep the GPU frame under this (60 fps with headroom for heat; perf budget 13 ms)
        float raiseMs = 10.5f;      // a higher scale only when the worst recent frame would stay under this
        float fixedShare = 0.30f;   // a: the part of a frame that does not scale with the pixel count (P93: 0.29 measured)
        float upscaleMs = 0.6f;     // U: the upscale pass at the output size (sharpened bilinear, 3420x2224)
        // TAA: the temporal upscaler runs on native frames too (its history must not break), so U is paid at every
        // scale and native saves nothing. false: U only below native (the sharpened bilinear, MetalFX spatial).
        bool alwaysUpscale = false;
    };
    static constexpr float kMaxDrop = 0.02f;    // scale per frame, down
    static constexpr float kMaxRaise = 0.004f;  // scale per frame, up (0.8 -> 1 in 50 frames)
    static constexpr int kHold = 20;            // frames of headroom before the scale rises
    static constexpr int kWindow = 30;          // frames the raise test looks back over

    Params params;

    void reset() {
        scale_ = 1.0f, fast_ = -1.0f, hold_ = 0, n_ = 0, next_ = 0, cut_ = false;
        recent_.fill(0.0f);
    }

    // A camera cut: what was measured belongs to the old view. The scale drops to the lowest until kCutFrames frames of
    // the new view have been reported, then goes straight to the scale they ask for (no step limit or hold: the change
    // lands a few frames after the cut, where it is not seen), and the normal rules take over.
    static constexpr int kCutFrames = 2;
    void cut() {
        reset();
        scale_ = std::clamp(params.minScale, 0.5f, 1.0f);
        cut_ = true;
    }

    // A completed frame: its GPU milliseconds and the scale it was drawn at.
    void report(double gpuMs, float scale) {
        if (!(gpuMs > 0.0) || gpuMs > 1000.0) return;
        const float s = std::clamp(scale, 0.25f, 1.0f);
        const float a = params.fixedShare;
        const float u = s < 1.0f || params.alwaysUpscale ? params.upscaleMs : 0.0f;
        const float native = std::max(0.1f, (float(gpuMs) - u) / (a + (1.0f - a) * s * s));
        fast_ = fast_ < 0.0f ? native : fast_ + 0.3f * (native - fast_);
        recent_[size_t(next_)] = native;
        next_ = (next_ + 1) % kWindow;
        n_ = std::min(n_ + 1, kWindow);
    }

    // The scale for the next frame (1 = native).
    float next() {
        const float lo = std::clamp(params.minScale, 0.5f, 1.0f);
        if (fast_ < 0.0f) return scale_;
        if (cut_) {
            if (n_ < kCutFrames) return scale_;
            cut_ = false;
            float worst = 0.0f;
            for (int k = 0; k < n_; ++k) worst = std::max(worst, recent_[size_t(k)]);
            scale_ = scaleFor(worst, params.targetMs, lo);
            return scale_;
        }
        // Down: the scale whose predicted time meets the target.
        const float want = scaleFor(fast_, params.targetMs, lo);
        if (want < scale_ - 1e-4f) {
            scale_ = std::max(want, scale_ - kMaxDrop);
            hold_ = 0;
            return scale_;
        }
        // Up: judged on a high percentile of the recent frames (the third worst of kWindow: one stray frame does not
        // hold the scale down), against the lower raise threshold.
        std::array<float, kWindow> sorted = recent_;
        std::sort(sorted.begin(), sorted.begin() + n_, std::greater<float>());
        const float worst = sorted[size_t(std::min(n_ - 1, 2))];
        if (scale_ < 1.0f && worst + nativeUpscale() <= params.raiseMs) {  // native fits (and saves the upscale): go back at once
            if (++hold_ >= kHold) scale_ = 1.0f, hold_ = 0;
            return scale_;
        }
        const float up = scaleFor(worst, params.raiseMs, lo);
        if (up > scale_ + 1e-3f) {
            if (++hold_ >= kHold) scale_ = std::min(up, scale_ + kMaxRaise);
        } else {
            hold_ = 0;
        }
        return scale_;
    }

    float scale() const { return scale_; }

private:
    // The largest scale in [lo, 1] at which a frame of native cost n is predicted to take at most t ms.
    float scaleFor(float n, float t, float lo) const {
        const float a = params.fixedShare;
        if (n + nativeUpscale() <= t) return 1.0f;  // native fits
        const float s2 = ((t - params.upscaleMs) / n - a) / (1.0f - a);
        if (!(s2 > 0.0f)) return lo;
        return std::clamp(std::sqrt(s2), lo, 0.999f);
    }

    float nativeUpscale() const { return params.alwaysUpscale ? params.upscaleMs : 0.0f; }  // TAA: U at native

    float scale_ = 1.0f;
    float fast_ = -1.0f;
    int hold_ = 0, n_ = 0, next_ = 0;
    bool cut_ = false;
    std::array<float, kWindow> recent_{};
};

}  // namespace fuel
