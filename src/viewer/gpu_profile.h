// P92: per-pass GPU timing for the renderer's performance work (fuelview --perf-check, FUEL_GPU_PASSES=1).
//
// Every render / blit pass the renderer encodes is tagged with a name (gpuprof::tag on its pass descriptor). When the
// profiler is on, the tag attaches stage-boundary timestamp samples (MTLCounterSampleBuffer, Apple GPUs support
// MTLCounterSamplingPointAtStageBoundary) to the pass: start / end of its vertex and fragment stages. When the frame's
// command buffer completes the samples are resolved into milliseconds and queued as a FrameTimes record that
// drain() hands out. Off (the default) every call is a cheap no-op and no pass descriptor is touched, so the
// renderer's output and cost are unchanged.
//
// A GPU without stage-boundary counters still gets the command buffer's GPUStartTime / GPUEndTime per frame (no
// per-pass split).
#pragma once

#import <Metal/Metal.h>

#include <cstdint>
#include <string>
#include <vector>

namespace fuel::gpuprof {

// Turns the profiler on (idempotent). Returns false when the device has no stage-boundary timestamps (frames are
// still timed as a whole).
bool enable(id<MTLDevice> device);
bool enabled();
bool passTimingAvailable();

// Brackets one frame's command buffer: begin before the first pass is encoded, end after the last (before commit).
// end() adds the completion handler that resolves the samples. begin() inside an open frame does nothing.
void beginFrame();
void endFrame(id<MTLCommandBuffer> cb);
// The number beginFrame() gave the frame being encoded (FrameTimes::frame).
uint64_t currentFrame();

// Names the pass the descriptor is about to start (call right before renderCommandEncoderWithDescriptor /
// blitCommandEncoderWithDescriptor). Passes of the same name in one frame add up in the reports.
// P93: chargeGap = true charges the pass also the idle time since the pass before it ended (the work of an encoder that
// cannot be tagged, e.g. MetalFX's, which ran in between).
void tag(MTLRenderPassDescriptor* rp, const char* name, bool chargeGap = false);
void tag(MTLBlitPassDescriptor* bp, const char* name);
// A blit / compute encoder for `cb` with the name attached (the plain encoder when the profiler is off).
id<MTLBlitCommandEncoder> blit(id<MTLCommandBuffer> cb, const char* name);
id<MTLComputeCommandEncoder> compute(id<MTLCommandBuffer> cb, const char* name);

struct PassTime {
    std::string name;
    double start = 0, end = 0;  // ms from the frame's first sample (the earliest valid stage boundary)
    double vertexMs = 0, fragmentMs = 0;  // the stages' own spans (0 when not sampled); they overlap on a tiler
    bool chargeGap = false;  // P93 (tag)
};
struct FrameTimes {
    uint64_t frame = 0;           // beginFrame() count
    double cbMs = 0;              // the command buffer's GPUEndTime - GPUStartTime
    double cbEnd = 0;             // GPUEndTime (s, CPU timebase)
    double spanMs = 0;            // first stage boundary to last (the passes' own span)
    double startAbsMs = 0, endAbsMs = 0;  // first / last stage boundary on the GPU clock (ms), to compare frames
    std::vector<PassTime> passes; // in encoding order
};
// The frames completed since the last call, in completion order.
std::vector<FrameTimes> drain();

// Exclusive time per pass: each pass is charged only the time not already covered by the passes encoded before it
// (the GPU overlaps one pass's vertex stage with the previous pass's fragment stage) nor by the previous frame, which
// ended at prevEndAbsMs (frames in flight overlap too: a command buffer's own GPUStart..End then counts the wait for
// the frame before it). The parts add up to the GPU time the frame added: its end minus the later of its start and
// the previous frame's end.
std::vector<std::pair<std::string, double>> exclusive(const FrameTimes& f, double prevEndAbsMs = -1e300);

}  // namespace fuel::gpuprof
