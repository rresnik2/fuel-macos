// P92: per-pass GPU timing (gpu_profile.h).
#include "viewer/gpu_profile.h"

#include <QuartzCore/QuartzCore.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <memory>
#include <cstdio>
#include <mutex>
#include <thread>

namespace fuel::gpuprof {
namespace {

constexpr NSUInteger kSamplesPerFrame = 4 * 96;  // 4 stage boundaries x up to 96 passes
constexpr size_t kRing = 8;                      // frames that may be in flight at once (MTKView keeps 3)

struct Pending {
    std::vector<std::string> names;  // pass k uses samples 4k .. 4k+3
    std::vector<bool> render;        // render pass (stage samples) or blit / compute (encoder start / end only)
    std::vector<bool> gap;           // P93: charged from the previous pass's end (tag's chargeGap)
    uint64_t frame = 0;
};

struct State {
    bool on = false, counters = false;
    id<MTLDevice> device = nil;
    std::array<id<MTLCounterSampleBuffer>, kRing> buffers{};
    double ticksPerMs = 1e6;  // GPU timestamp ticks per millisecond (calibrated)
    uint64_t frame = 0;
    Pending cur;
    bool inFrame = false;
    std::mutex mu;
    std::vector<FrameTimes> done;
} g;

id<MTLCounterSet> timestampSet(id<MTLDevice> d) {
    for (id<MTLCounterSet> s in d.counterSets)
        if ([s.name isEqualToString:MTLCommonCounterSetTimestamp]) return s;
    return nil;
}

// GPU timestamps are in device ticks: compare two CPU / GPU timestamp pairs taken 50 ms apart.
void calibrate() {
    MTLTimestamp c0 = 0, g0 = 0, c1 = 0, g1 = 0;
    [g.device sampleTimestamps:&c0 gpuTimestamp:&g0];
    const double w0 = CACurrentMediaTime();
    std::this_thread::sleep_for(std::chrono::milliseconds(50));
    [g.device sampleTimestamps:&c1 gpuTimestamp:&g1];
    const double w1 = CACurrentMediaTime();
    if (g1 > g0 && w1 > w0) g.ticksPerMs = double(g1 - g0) / ((w1 - w0) * 1000.0);
}

}  // namespace

bool enable(id<MTLDevice> device) {
    if (g.on) return g.counters;
    g.on = true;
    g.device = device;
    id<MTLCounterSet> ts = timestampSet(device);
    if (ts != nil && [device supportsCounterSampling:MTLCounterSamplingPointAtStageBoundary]) {
        MTLCounterSampleBufferDescriptor* d = [MTLCounterSampleBufferDescriptor new];
        d.counterSet = ts;
        d.storageMode = MTLStorageModeShared;
        d.sampleCount = kSamplesPerFrame;
        g.counters = true;
        for (auto& b : g.buffers) {
            NSError* err = nil;
            b = [device newCounterSampleBufferWithDescriptor:d error:&err];
            if (b == nil) {
                std::fprintf(stderr, "gpu profile: no counter sample buffer (%s); frame times only\n", err.localizedDescription.UTF8String);
                g.counters = false;
                break;
            }
        }
        if (g.counters) calibrate();
    }
    return g.counters;
}

bool enabled() { return g.on; }
uint64_t currentFrame() { return g.cur.frame; }
bool passTimingAvailable() { return g.counters; }

void beginFrame() {
    if (!g.on || g.inFrame) return;  // a frame the caller opened (PerfCheck) stays open through render()
    g.cur = Pending{};
    g.cur.frame = g.frame++;
    g.inFrame = true;
}

namespace {
// The slot of the next pass of the current frame, or -1 (profiler off, outside a frame, or the frame is full).
int nextSlot(const char* name) {
    if (!g.on || !g.counters || !g.inFrame) return -1;
    if (g.cur.names.size() * 4 + 4 > kSamplesPerFrame) return -1;
    g.cur.names.emplace_back(name);
    g.cur.render.push_back(true);
    g.cur.gap.push_back(false);
    return int(g.cur.names.size() - 1);
}
}  // namespace

void tag(MTLRenderPassDescriptor* rp, const char* name, bool chargeGap) {
    const int k = nextSlot(name);
    if (k < 0) return;
    g.cur.gap[size_t(k)] = chargeGap;
    MTLRenderPassSampleBufferAttachmentDescriptor* a = rp.sampleBufferAttachments[0];
    a.sampleBuffer = g.buffers[g.cur.frame % kRing];
    a.startOfVertexSampleIndex = NSUInteger(4 * k);
    a.endOfVertexSampleIndex = NSUInteger(4 * k + 1);
    a.startOfFragmentSampleIndex = NSUInteger(4 * k + 2);
    a.endOfFragmentSampleIndex = NSUInteger(4 * k + 3);
}

void tag(MTLBlitPassDescriptor* bp, const char* name) {
    const int k = nextSlot(name);
    if (k < 0) return;
    g.cur.render[size_t(k)] = false;
    MTLBlitPassSampleBufferAttachmentDescriptor* a = bp.sampleBufferAttachments[0];
    a.sampleBuffer = g.buffers[g.cur.frame % kRing];
    a.startOfEncoderSampleIndex = NSUInteger(4 * k);
    a.endOfEncoderSampleIndex = NSUInteger(4 * k + 3);
}

id<MTLBlitCommandEncoder> blit(id<MTLCommandBuffer> cb, const char* name) {
    if (!g.on || !g.counters || !g.inFrame) return [cb blitCommandEncoder];
    MTLBlitPassDescriptor* bp = [MTLBlitPassDescriptor blitPassDescriptor];
    tag(bp, name);
    return [cb blitCommandEncoderWithDescriptor:bp];
}

id<MTLComputeCommandEncoder> compute(id<MTLCommandBuffer> cb, const char* name) {
    if (!g.on || !g.counters || !g.inFrame) return [cb computeCommandEncoder];
    const int k = nextSlot(name);
    if (k < 0) return [cb computeCommandEncoder];
    g.cur.render[size_t(k)] = false;
    MTLComputePassDescriptor* cp = [MTLComputePassDescriptor computePassDescriptor];
    MTLComputePassSampleBufferAttachmentDescriptor* a = cp.sampleBufferAttachments[0];
    a.sampleBuffer = g.buffers[g.cur.frame % kRing];
    a.startOfEncoderSampleIndex = NSUInteger(4 * k);
    a.endOfEncoderSampleIndex = NSUInteger(4 * k + 3);
    return [cb computeCommandEncoderWithDescriptor:cp];
}

void endFrame(id<MTLCommandBuffer> cb) {
    if (!g.on || !g.inFrame) return;
    g.inFrame = false;
    auto pending = std::make_shared<Pending>(std::move(g.cur));
    id<MTLCounterSampleBuffer> buf = g.counters ? g.buffers[pending->frame % kRing] : nil;
    const double tpm = g.ticksPerMs;
    [cb addCompletedHandler:^(id<MTLCommandBuffer> b) {
        FrameTimes ft;
        ft.frame = pending->frame;
        if (b.GPUEndTime > b.GPUStartTime) ft.cbMs = (b.GPUEndTime - b.GPUStartTime) * 1000.0;
        ft.cbEnd = b.GPUEndTime;
        const NSUInteger n = pending->names.size() * 4;
        if (buf != nil && n > 0) {
            NSData* data = [buf resolveCounterRange:NSMakeRange(0, n)];
            if (data != nil && data.length >= n * sizeof(MTLCounterResultTimestamp)) {
                const auto* ts = static_cast<const MTLCounterResultTimestamp*>(data.bytes);
                uint64_t t0 = UINT64_MAX, t1 = 0;
                for (NSUInteger i = 0; i < n; ++i)
                    if (ts[i].timestamp != MTLCounterErrorValue && ts[i].timestamp != 0) t0 = std::min(t0, ts[i].timestamp);
                for (size_t k = 0; k < pending->names.size(); ++k) {
                    uint64_t s = UINT64_MAX, e = 0;
                    for (int j = 0; j < 4; ++j) {
                        const uint64_t v = ts[4 * k + size_t(j)].timestamp;
                        if (v == MTLCounterErrorValue || v == 0 || v < t0) continue;
                        s = std::min(s, v), e = std::max(e, v);
                    }
                    if (e < s) continue;  // the pass did no GPU work (or the samples are missing)
                    t1 = std::max(t1, e);
                    const bool render = pending->render[k];
                    auto stage = [&](int a) {  // samples a, a + 1: start / end of one stage (render passes only)
                        if (!render) return 0.0;
                        const uint64_t x = ts[4 * k + size_t(a)].timestamp, y = ts[4 * k + size_t(a) + 1].timestamp;
                        if (x == MTLCounterErrorValue || y == MTLCounterErrorValue || x == 0 || y < x) return 0.0;
                        return double(y - x) / tpm;
                    };
                    ft.passes.push_back({pending->names[k], double(s - t0) / tpm, double(e - t0) / tpm, stage(0), stage(2), pending->gap[k]});
                }
                if (t1 > t0 && t0 != UINT64_MAX) {
                    ft.spanMs = double(t1 - t0) / tpm;
                    ft.startAbsMs = double(t0) / tpm, ft.endAbsMs = double(t1) / tpm;
                }
            }
        }
        std::lock_guard<std::mutex> lk(g.mu);
        g.done.push_back(std::move(ft));
    }];
}

std::vector<FrameTimes> drain() {
    std::lock_guard<std::mutex> lk(g.mu);
    std::vector<FrameTimes> out;
    out.swap(g.done);
    std::sort(out.begin(), out.end(), [](const FrameTimes& a, const FrameTimes& b) { return a.frame < b.frame; });
    return out;
}

std::vector<std::pair<std::string, double>> exclusive(const FrameTimes& f, double prevEndAbsMs) {
    std::vector<std::pair<std::string, double>> out;
    // The GPU time already charged, as the end of the union so far (passes run in order), from the frame's start or
    // the previous frame's end, whichever is later.
    double covered = std::max(0.0, prevEndAbsMs - f.startAbsMs);
    for (const PassTime& p : f.passes) {
        const double s = p.chargeGap ? covered : std::max(p.start, covered), e = std::max(p.end, covered);
        out.emplace_back(p.name, e - s);
        covered = std::max(covered, p.end);
    }
    return out;
}

}  // namespace fuel::gpuprof
