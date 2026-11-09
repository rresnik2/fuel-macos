// P92: fuelview --perf-check: a scripted race film rendered offscreen at full size (default 3420x2224, the
// player's fullscreen size) that reports what each frame costs and FAILS when the 95th-percentile frame is over
// budget or a frame stalls.
//
//   fuelview --data DIR --perf-check [--race NAME] [--time HH.MM] [--size WxH] [--drive-time S]
//            [--perf-budget MS] [--perf-breakdown] [--perf-csv FILE] [--assemble VARIANT DPC]
//
// The race runs like --film every frame (chase camera, cut-scenes, dust, storms, opponents, streaming), but each
// frame is rendered into an offscreen target instead of being written out: one frame at a time by default, so each
// pass's GPU time is its own (--perf-inflight 3 lets frames overlap as in the window: throughput). Per frame it records:
//   - the GPU time of every pass (gpu_profile.h, stage-boundary timestamps), charged exclusively so they add up;
//   - the CPU time of the simulation (physics, streaming, the rest) and of the renderer's encoding phases;
//   - the one-off work in the frame (pipeline compiles, texture uploads, mesh builds, terrain chunk buffers).
// A frame is over budget when its GPU time exceeds --perf-budget (default 13 ms: 60 fps with 20% headroom) or its
// CPU time exceeds 16.7 ms; it is a hitch when either is over 25 ms. The cause printed for a slow frame is the CPU
// phase or GPU pass that grew the most against its median.
//
// --perf-breakdown cycles the scene pass's parts off frame by frame (MeshRenderer::perfSkip) and prints what each
// part costs (the scene pass with it minus without); the frames with a part left out do not count for the verdict.
// It is a profiling view only: those frames are not the game's image.
#pragma once

#import <Metal/Metal.h>

#include <array>
#include <memory>
#include <string>
#include <vector>

#include "viewer/renderer.h"

namespace fuel {

class PerfCheck {
public:
    struct Options {
        int width = 3420, height = 2224;
        double budgetMs = 13.0;   // GPU p95 limit
        double cpuBudgetMs = 16.7;  // CPU p95 limit (a 60 Hz frame)
        double hitchMs = 25.0;
        int warmupFrames = 60;    // not judged (the first second: loading, every shadow slice and cube face)
        int inFlight = 1;         // frames the GPU may have queued: 1 = serial (clean per-pass times, the default),
                                  // 3 = as MTKView (throughput; early passes overlap the previous frame's last ones)
        bool breakdown = false;
        std::string csv;          // per-frame log
        std::string label;        // printed in the report (race, time)
        // P93: A/B within one run (the GPU clock drifts with heat and other load between runs): even frames A, odd B.
        // "scale=S": native / S; "upscaler=S": MetalFX / sharpened bilinear at S; "temporal=S": sharpened bilinear / MetalFX
        // temporal at S (TAA); "shadow": 3x3 / 2x2. The report adds
        // each pass's mean for A and B.
        std::string alternate;
        double paceHz = 0;        // P93: > 0: a frame starts no sooner than 1 / paceHz after the last (the window's vsync;
                                  // the GPU then idles between frames and its clock governor acts as in the game)
    };
    // The simulation's CPU time of one frame (milliseconds) by phase, measured by the caller (kSimPartNames).
    static constexpr int kSimParts = 12;
    static const char* const kSimPartNames[kSimParts];
    struct Sim {
        std::array<double, kSimParts> part{};
        double t = 0;  // seconds into the run
        double total() const {
            double s = 0;
            for (double x : part) s += x;
            return s;
        }
    };

    PerfCheck(id<MTLDevice> device, const Options& o);
    ~PerfCheck();
    // Renders one frame of the film: the scene, the post chain, into the offscreen target. Waits while three
    // frames are in flight.
    void frame(MeshRenderer& renderer, const OrbitCamera& cam, const Sim& sim);
    // Waits for the GPU, prints the report and returns true when the run passes.
    bool finish(MeshRenderer& renderer);

private:
    struct Impl;
    std::unique_ptr<Impl> impl_;
};

}  // namespace fuel
