// P92: fuelview --perf-check (perf_check.h).
#include "viewer/perf_check.h"

#include "viewer/gpu_profile.h"

#include <QuartzCore/QuartzCore.h>
#include <dispatch/dispatch.h>

#include <algorithm>
#include <cstdio>
#include <map>
#include <set>
#include <thread>
#include <unordered_map>

namespace fuel {
namespace {

double pct(std::vector<double> v, double p) {
    if (v.empty()) return 0;
    std::sort(v.begin(), v.end());
    const size_t i = std::min(v.size() - 1, size_t(p * double(v.size() - 1) + 0.5));
    return v[i];
}
double mean(const std::vector<double>& v) {
    double s = 0;
    for (double x : v) s += x;
    return v.empty() ? 0 : s / double(v.size());
}

// The scene-pass parts --perf-breakdown leaves out, one per frame in turn (0 = the whole scene).
const std::vector<std::pair<uint32_t, const char*>>& breakdownMasks() {
    static const std::vector<std::pair<uint32_t, const char*>> m = {
        {0, "(all)"},
        {MeshRenderer::SkipSky, "sky"},
        {MeshRenderer::SkipTerrain, "terrain"},
        {MeshRenderer::SkipOpaque, "opaque meshes"},
        {MeshRenderer::SkipCutout, "cut-out meshes (foliage)"},
        {MeshRenderer::SkipGrass, "grass"},
        {MeshRenderer::SkipWater, "water"},
        {MeshRenderer::SkipBlend, "blended + fading"},
        {MeshRenderer::SkipShadowLookup, "shadow lookups"},
        {MeshRenderer::SkipOmnis, "headlight omnis"},
        {MeshRenderer::SkipShadowTerrain, "shadow: terrain casters"},  // P95
        {MeshRenderer::SkipShadowObjects, "shadow: object casters"},
        {MeshRenderer::SkipCube, "cube face"},
    };
    return m;
}

// The GPU clock probe: a fixed amount of arithmetic (no memory traffic) timed every frame. The M2 in a MacBook Air
// lowers its GPU clock after minutes of full load (no fan); the probe's time rises with it, so runs made at different
// temperatures can be told apart (and compared fairly only at similar probe times).
const char* kProbeSource = R"METAL(
#include <metal_stdlib>
using namespace metal;
kernel void kClockProbe(device float* out [[buffer(0)]], uint id [[thread_position_in_grid]]) {
    float a = float(id) * 1e-7, b = 1.0001, c = 0.9999;
    for (int k = 0; k < 512; ++k) a = fma(a, b, c), b = fma(b, c, a * 1e-9);
    if (a == 12345.678) out[0] = b;  // never true: keeps the loop
}
)METAL";

}  // namespace

const char* const PerfCheck::kSimPartNames[PerfCheck::kSimParts] = {
    "sim: cut-scene, world anims", "sim: race rules, AI", "sim: physics", "sim: camera", "sim: cut fx, storms, dust",
    "sim: damage, debris", "sim: HUD state", "sim: streaming", "set-up: world anims", "set-up: riders (skinning)",
    "set-up: vehicles, markers", "set-up: effect lists",
};

struct PerfCheck::Impl {
    Options o;
    id<MTLDevice> device;
    id<MTLCommandQueue> queue;
    id<MTLTexture> target;
    dispatch_semaphore_t inFlight;
    id<MTLComputePipelineState> probe = nil;
    id<MTLBuffer> probeOut = nil;
    struct Rec {
        int index = 0;
        Sim sim;
        MeshRenderer::CpuTimes rc;
        double encodeMs = 0, waitMs = 0;
        uint32_t skip = 0;
        bool haveGpu = false;
        double gpuMs = 0, cbMs = 0, spanMs = 0, probeMs = 0;
        float scale = 1;  // P93: the render scale (dynamic resolution)
        std::map<std::string, double> pass;  // exclusive ms by pass name
        std::map<std::string, std::pair<double, double>> stages;  // vertex / fragment stage ms by pass name
        double cpuMs() const { return sim.total() + encodeMs; }
    };
    std::vector<Rec> recs;
    std::unordered_map<uint64_t, size_t> byGpuFrame;  // gpuprof frame -> recs index
    std::set<std::string> passNames;
    std::vector<std::string> passOrder;
    double prevEndAbs = -1e300, prevCbEnd = -1;  // the last collected frame's GPU end (frames arrive in order)
    double lastStart = 0;  // P93: --perf-pace

    void collect() {
        for (const gpuprof::FrameTimes& f : gpuprof::drain()) {
            auto it = byGpuFrame.find(f.frame);
            if (it == byGpuFrame.end()) continue;
            Rec& r = recs[it->second];
            r.haveGpu = true;
            // The GPU time this frame added: with frames in flight its command buffer starts while the one before
            // still runs, so GPUStart..End double-counts; the time since the previous frame ended does not.
            r.cbMs = prevCbEnd > 0 && f.cbEnd > prevCbEnd ? std::min(f.cbMs, (f.cbEnd - prevCbEnd) * 1000.0) : f.cbMs;
            prevCbEnd = f.cbEnd;
            r.spanMs = f.spanMs;
            double sum = 0;
            const double prev = prevEndAbs;
            if (f.endAbsMs > 0) prevEndAbs = std::max(prevEndAbs, f.endAbsMs);
            for (const auto& [name, ms] : gpuprof::exclusive(f, prev)) {
                if (name == std::string("clock probe")) {
                    r.probeMs += ms;
                    continue;
                }
                r.pass[name] += ms, sum += ms;
                if (passNames.insert(name).second) passOrder.push_back(name);
            }
            for (const gpuprof::PassTime& p : f.passes) r.stages[p.name].first += p.vertexMs, r.stages[p.name].second += p.fragmentMs;
            r.gpuMs = gpuprof::passTimingAvailable() && !f.passes.empty() ? sum : f.cbMs - r.probeMs;
            byGpuFrame.erase(it);
        }
    }
};

PerfCheck::PerfCheck(id<MTLDevice> device, const Options& o) : impl_(std::make_unique<Impl>()) {
    Impl& m = *impl_;
    m.o = o;
    m.device = device;
    m.queue = [device newCommandQueue];
    MTLTextureDescriptor* td = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:kOutputColorFormat
                                                                                  width:NSUInteger(o.width)
                                                                                 height:NSUInteger(o.height)
                                                                              mipmapped:NO];
    td.usage = MTLTextureUsageRenderTarget;
    td.storageMode = MTLStorageModePrivate;
    m.target = [device newTextureWithDescriptor:td];
    m.inFlight = dispatch_semaphore_create(std::clamp(o.inFlight, 1, 3));
    {
        NSError* err = nil;
        id<MTLLibrary> lib = [device newLibraryWithSource:@(kProbeSource) options:nil error:&err];
        if (lib != nil) m.probe = [device newComputePipelineStateWithFunction:[lib newFunctionWithName:@"kClockProbe"] error:&err];
        m.probeOut = [device newBufferWithLength:16 options:MTLResourceStorageModePrivate];
    }
    if (!gpuprof::enable(device)) std::printf("perf check: this GPU has no stage-boundary timestamps; whole-frame GPU times only\n");
}

PerfCheck::~PerfCheck() = default;

void PerfCheck::frame(MeshRenderer& renderer, const OrbitCamera& cam, const Sim& sim) {
    Impl& m = *impl_;
    Impl::Rec r;
    r.index = int(m.recs.size());
    r.sim = sim;
    const double w0 = CACurrentMediaTime();
    dispatch_semaphore_wait(m.inFlight, DISPATCH_TIME_FOREVER);
    if (m.o.paceHz > 0) {  // P93: the window's vsync cadence
        const double due = m.lastStart + 1.0 / m.o.paceHz;
        for (double now = CACurrentMediaTime(); now < due; now = CACurrentMediaTime())
            std::this_thread::sleep_for(std::chrono::microseconds(int64_t(std::max(50.0, (due - now) * 1e6 - 300))));
        m.lastStart = std::max(CACurrentMediaTime(), due);
    }
    r.waitMs = (CACurrentMediaTime() - w0) * 1000.0;
    if (m.o.breakdown) {
        const auto& masks = breakdownMasks();
        r.skip = masks[size_t(r.index) % masks.size()].first;
    }
    renderer.perfSkip = r.skip;
    if (!m.o.alternate.empty()) {  // P93: A/B frames
        const bool b = (r.index & 1) != 0;
        const std::string& a = m.o.alternate;
        const float s = a.find('=') != std::string::npos ? std::stof(a.substr(a.find('=') + 1)) : 0.8f;
        if (a.rfind("scale", 0) == 0) renderer.fixedRenderScale = b ? s : 1.0f;
        else if (a.rfind("upscaler", 0) == 0) renderer.fixedRenderScale = s, renderer.upscaler = b ? MeshRenderer::Upscaler::Sharpen : MeshRenderer::Upscaler::MetalFX;
        else if (a.rfind("temporal", 0) == 0)  // TAA: A the sharpened bilinear at S, B MetalFX temporal at S (its history restarts
            // on every B frame: the cost is the same pass, the picture is not judged)
            renderer.fixedRenderScale = s, renderer.upscaler = b ? MeshRenderer::Upscaler::MetalFXTemporal : MeshRenderer::Upscaler::Sharpen;
        else if (a == "shadow") renderer.shadow2x2 = b;
        else if (a.rfind("grain", 0) == 0)  // P102: A no grain, B film grain (grain=AMPLITUDE, default light)
            renderer.filmGrain = b ? (a.find('=') != std::string::npos ? s : MeshRenderer::kFilmGrainLight) : 0.0f;
        else if (a.rfind("soften", 0) == 0)  // P102: A no soften, B soften (soften=AMOUNT, default light)
            renderer.soften = b ? (a.find('=') != std::string::npos ? s : MeshRenderer::kSoftenLight) : 0.0f;
        else if (a == "vlod") {  // P104: A every vehicle at LOD1 (pre-P104), B FUEL's vehicle LOD
            renderer.vehicleLodOff = !b;
            renderer.applyVehicleLodSwitch();
        } else if (a == "links") {  // PK22: A the lamp groups as stored, B switched (core/vehicle_links)
            renderer.linksOff = !b;
        } else if (a.rfind("omni", 0) == 0) {  // PK28: A the first N gathered headlights (omni=N, default 16), B FUEL's 64
            renderer.omniCap = b ? 64 : (a.find('=') != std::string::npos ? int(s) : 16);
        } else if (a == "rotshapes") {  // PK28: A no RotShape_Z, B drawn
            renderer.rotShapesOn = b;
        } else if (a.rfind("opt", 0) == 0) renderer.p95Off = b ? 0u : uint32_t(std::stoul(a.substr(a.find('=') + 1), nullptr, 0));  // P95: A = off, B = on
    }
    id<MTLCommandBuffer> cb = [m.queue commandBuffer];
    const double t0 = CACurrentMediaTime();
    gpuprof::beginFrame();
    if (m.probe != nil) {  // the clock probe, before the frame's own passes
        id<MTLComputeCommandEncoder> ce = gpuprof::compute(cb, "clock probe");
        [ce setComputePipelineState:m.probe];
        [ce setBuffer:m.probeOut offset:0 atIndex:0];
        [ce dispatchThreads:MTLSizeMake(32768, 1, 1) threadsPerThreadgroup:MTLSizeMake(256, 1, 1)];
        [ce endEncoding];
    }
    renderer.render(cb, m.target, cam);
    r.encodeMs = (CACurrentMediaTime() - t0) * 1000.0;
    renderer.perfSkip = 0;
    r.rc = renderer.cpuTimes();
    r.scale = renderer.lastRenderScale();
    m.byGpuFrame[gpuprof::currentFrame()] = m.recs.size();
    gpuprof::endFrame(cb);
    dispatch_semaphore_t sem = m.inFlight;
    [cb addCompletedHandler:^(id<MTLCommandBuffer>) { dispatch_semaphore_signal(sem); }];
    [cb commit];
    m.recs.push_back(std::move(r));
    if (m.recs.size() % 30 == 0) m.collect();
}

bool PerfCheck::finish(MeshRenderer&) {
    Impl& m = *impl_;
    const int slots = std::clamp(m.o.inFlight, 1, 3);
    for (int k = 0; k < slots; ++k) dispatch_semaphore_wait(m.inFlight, DISPATCH_TIME_FOREVER);
    for (int k = 0; k < slots; ++k) dispatch_semaphore_signal(m.inFlight);
    m.collect();
    const Options& o = m.o;

    // Judged frames: past the warm-up, the whole scene drawn, GPU times known.
    std::vector<const Impl::Rec*> judged;
    for (const Impl::Rec& r : m.recs)
        if (r.index >= o.warmupFrames && r.skip == 0 && r.haveGpu) judged.push_back(&r);
    std::printf("\nperf check: %s, %dx%d, %zu frames (%zu judged), %d in flight%s\n", o.label.c_str(), o.width, o.height, m.recs.size(),
                judged.size(), std::clamp(o.inFlight, 1, 3),
                gpuprof::passTimingAvailable() ? "" : " [no per-pass timing]");
    if (o.paceHz > 0) std::printf("perf check: paced at %.0f Hz (frames start no sooner than the window's vsync)\n", o.paceHz);
    if (judged.empty()) {
        std::printf("perf check: no frames to judge: FAILED\n");
        return false;
    }

    // ---- GPU per pass.
    std::map<std::string, std::vector<double>> passMs;
    std::vector<double> gpu, cb, span, cpu;
    for (const Impl::Rec* r : judged) {
        for (const std::string& n : m.passOrder) {
            auto it = r->pass.find(n);
            passMs[n].push_back(it == r->pass.end() ? 0.0 : it->second);
        }
        gpu.push_back(r->gpuMs), cb.push_back(r->cbMs), span.push_back(r->spanMs), cpu.push_back(r->cpuMs());
    }
    std::map<std::string, double> passMedian;
    std::printf("GPU per pass (ms, exclusive)       mean     p95     max   (vertex / fragment stage, mean)\n");
    for (const std::string& n : m.passOrder) {
        const auto& v = passMs[n];
        passMedian[n] = pct(v, 0.5);
        double vs = 0, fs = 0;
        for (const Impl::Rec* r : judged)
            if (auto it = r->stages.find(n); it != r->stages.end()) vs += it->second.first, fs += it->second.second;
        std::printf("  %-30s %7.2f %7.2f %7.2f   (%.2f / %.2f)\n", n.c_str(), mean(v), pct(v, 0.95), pct(v, 1.0), vs / double(judged.size()),
                    fs / double(judged.size()));
    }
    std::printf("  %-30s %7.2f %7.2f %7.2f\n", "frame (sum of passes)", mean(gpu), pct(gpu, 0.95), pct(gpu, 1.0));
    std::printf("  %-30s %7.2f %7.2f %7.2f\n", "frame (command buffers, net)", mean(cb), pct(cb, 0.95), pct(cb, 1.0));
    {
        std::vector<double> pr;
        for (const Impl::Rec* r : judged) pr.push_back(r->probeMs);
        std::printf("  %-30s %7.3f %7.3f %7.3f   (fixed arithmetic: higher = lower GPU clock, e.g. a hot MacBook Air)\n", "clock probe (not in the frame)",
                    mean(pr), pct(pr, 0.95), pct(pr, 1.0));
    }

    // ---- P93: --perf-alternate: each pass's mean over the A (even) and B (odd) frames.
    if (!o.alternate.empty()) {
        std::printf("alternate %s: pass means (ms)         A        B      B-A\n", o.alternate.c_str());
        std::vector<std::string> names = m.passOrder;
        names.push_back("(frame)");
        for (const std::string& n : names) {
            double s[2] = {0, 0};
            int c[2] = {0, 0};
            for (const Impl::Rec* r : judged) {
                const int g = r->index & 1;
                double v = r->gpuMs;
                if (n != "(frame)") {
                    auto it = r->pass.find(n);
                    v = it == r->pass.end() ? 0.0 : it->second;
                }
                s[g] += v, ++c[g];
            }
            const double a = s[0] / std::max(1, c[0]), b = s[1] / std::max(1, c[1]);
            std::printf("  %-34s %7.2f  %7.2f  %+7.2f\n", n.c_str(), a, b, b - a);
        }
    }

    // ---- P93: the render scale (dynamic resolution): share of the judged frames per band.
    {
        const double edges[] = {0.999, 0.95, 0.90, 0.85, 0.80};
        const char* names[] = {"1.00 (native)", "0.95 .. 1", "0.90 .. 0.95", "0.85 .. 0.90", "0.80 .. 0.85", "below 0.80"};
        int count[6] = {0, 0, 0, 0, 0, 0};
        double sum = 0;
        for (const Impl::Rec* r : judged) {
            int k = 0;
            while (k < 5 && r->scale < edges[k] - 1e-6) ++k;
            ++count[k], sum += r->scale;
        }
        std::printf("render scale (per axis): mean %.3f;", sum / double(judged.size()));
        for (int k = 0; k < 6; ++k)
            if (count[k]) std::printf(" %s %.1f%%;", names[k], 100.0 * count[k] / double(judged.size()));
        std::printf("\n");
    }

    // ---- CPU.
    struct Col {
        const char* name;
        double (*get)(const Impl::Rec&);
    };
#define P92_SIM_COL(k) {kSimPartNames[k], [](const Impl::Rec& r) { return r.sim.part[k]; }}
    static const Col cols[] = {
        P92_SIM_COL(0), P92_SIM_COL(1), P92_SIM_COL(2), P92_SIM_COL(3), P92_SIM_COL(4), P92_SIM_COL(5), P92_SIM_COL(6), P92_SIM_COL(7),
        P92_SIM_COL(8), P92_SIM_COL(9), P92_SIM_COL(10), P92_SIM_COL(11),
        {"render: terrain chunks", [](const Impl::Rec& r) { return r.rc.terrain; }},
        {"render: object LOD", [](const Impl::Rec& r) { return r.rc.lod; }},
        {"render: shadow casters", [](const Impl::Rec& r) { return r.rc.shadows; }},
        {"render: cube map", [](const Impl::Rec& r) { return r.rc.cube; }},
        {"render: scene pass", [](const Impl::Rec& r) { return r.rc.scene; }},
        {"render: effects", [](const Impl::Rec& r) { return r.rc.effects; }},
        {"render: post", [](const Impl::Rec& r) { return r.rc.post; }},
        {"render (encode, total)", [](const Impl::Rec& r) { return r.encodeMs; }},
        {"waiting for the GPU", [](const Impl::Rec& r) { return r.waitMs; }},
    };
#undef P92_SIM_COL
    std::printf("CPU (ms)                           mean     p95     max\n");
    std::map<std::string, double> cpuMedian;
    for (const Col& c : cols) {
        std::vector<double> v;
        for (const Impl::Rec* r : judged) v.push_back(c.get(*r));
        cpuMedian[c.name] = pct(v, 0.5);
        std::printf("  %-30s %7.2f %7.2f %7.2f\n", c.name, mean(v), pct(v, 0.95), pct(v, 1.0));
    }
    std::printf("  %-30s %7.2f %7.2f %7.2f\n", "frame (simulation + encode)", mean(cpu), pct(cpu, 0.95), pct(cpu, 1.0));
    {
        double sc = 0, st = 0, sk = 0, dd = 0, dt = 0, ck = 0, gt = 0;
        for (const Impl::Rec* r : judged)
            sc += r->rc.shadowCasters, st += r->rc.shadowTriangles, sk += r->rc.shadowChunks, dd += r->rc.sceneDraws, dt += r->rc.sceneTriangles,
                ck += r->rc.sceneChunks, gt += r->rc.grassTufts;
        const double n = double(judged.size());
        std::printf("work per frame (mean): shadow casters %.0f, shadow triangles %.2f M (%.0f terrain chunks), scene opaque/cut-out draws %.0f, "
                    "%.2f M triangles, %.0f terrain chunks, %.0f grass tufts\n", sc / n, st / n * 1e-6, sk / n, dd / n, dt / n * 1e-6, ck / n, gt / n);
    }

    // ---- Slow frames and their causes (every frame, warm-up included, so loading stalls show).
    auto cause = [&](const Impl::Rec& r) {
        char buf[256];
        std::string s;
        if (r.haveGpu && r.gpuMs >= r.cpuMs()) {
            std::string worst;
            double grow = -1e9;
            for (const auto& [n, ms] : r.pass)
                if (ms - passMedian[n] > grow) grow = ms - passMedian[n], worst = n;
            std::snprintf(buf, sizeof buf, "GPU %.1f ms: %s %+.1f ms over its median", r.gpuMs, worst.c_str(), grow);
            s = buf;
        } else {
            std::string worst;
            double grow = -1e9;
            for (const Col& c : cols) {
                if (std::string(c.name) == "waiting for the GPU" || std::string(c.name) == "render (encode, total)") continue;
                const double g = c.get(r) - cpuMedian[c.name];
                if (g > grow) grow = g, worst = c.name;
            }
            std::snprintf(buf, sizeof buf, "CPU %.1f ms: %s %+.1f ms over its median", r.cpuMs(), worst.c_str(), grow);
            s = buf;
        }
        if (r.rc.pipelines || r.rc.textures || r.rc.meshes || r.rc.chunks) {
            std::snprintf(buf, sizeof buf, " [%d pipeline compiles, %d texture uploads, %d mesh builds, %d terrain chunks]", r.rc.pipelines,
                          r.rc.textures, r.rc.meshes, r.rc.chunks);
            s += buf;
        }
        return s;
    };
    int hitches = 0, hitchesJudged = 0, overGpu = 0, overCpu = 0, printed = 0;
    for (const Impl::Rec& r : m.recs) {
        if (r.skip != 0) continue;
        const double worst = std::max(r.cpuMs(), r.gpuMs);
        const bool hitch = worst > o.hitchMs;
        const bool inJudged = r.index >= o.warmupFrames;
        if (inJudged && r.gpuMs > o.budgetMs) ++overGpu;
        if (inJudged && r.cpuMs() > o.cpuBudgetMs) ++overCpu;
        if (!hitch) continue;
        ++hitches;
        if (inJudged) ++hitchesJudged;
        if (printed++ < 25)
            std::printf("  hitch at %.2f s (frame %d%s): %s\n", r.sim.t, r.index, inJudged ? "" : ", warm-up", cause(r).c_str());
    }
    if (printed > 25) std::printf("  ... %d more\n", printed - 25);

    // ---- The scene pass's parts (--perf-breakdown). P95: also the scene's vertex stage, the shadow / cube passes and
    // the whole frame, each "with the part minus without".
    if (o.breakdown) {
        struct Acc { std::vector<double> scene, sceneV, sceneF, shadow, cube, frame; };
        std::map<uint32_t, Acc> acc;
        auto get = [](const Impl::Rec& r, const char* n) { auto it = r.pass.find(n); return it == r.pass.end() ? 0.0 : it->second; };
        for (const Impl::Rec& r : m.recs)
            if (r.index >= o.warmupFrames && r.haveGpu) {
                Acc& a = acc[r.skip];
                a.scene.push_back(get(r, "scene")), a.shadow.push_back(get(r, "shadow")), a.cube.push_back(get(r, "cube")), a.frame.push_back(r.gpuMs);
                auto st = r.stages.find("scene");
                a.sceneV.push_back(st == r.stages.end() ? 0.0 : st->second.first), a.sceneF.push_back(st == r.stages.end() ? 0.0 : st->second.second);
            }
        const Acc& all = acc[0];
        std::printf("breakdown (ms: with the part minus without; %zu frames each)   scene  (vertex / fragment)   shadow    cube   frame\n", all.scene.size());
        for (const auto& [mask, name] : breakdownMasks()) {
            if (mask == 0) continue;
            const Acc& a = acc[mask];
            std::printf("  %-30s %7.2f   (%5.2f / %5.2f)   %7.2f %7.2f %7.2f\n", name, mean(all.scene) - mean(a.scene), mean(all.sceneV) - mean(a.sceneV),
                        mean(all.sceneF) - mean(a.sceneF), mean(all.shadow) - mean(a.shadow), mean(all.cube) - mean(a.cube), mean(all.frame) - mean(a.frame));
        }
        std::printf("  %-30s %7.2f   (%5.2f / %5.2f)   %7.2f %7.2f %7.2f\n", "whole (all parts drawn)", mean(all.scene), mean(all.sceneV), mean(all.sceneF),
                    mean(all.shadow), mean(all.cube), mean(all.frame));
    }

    // ---- Per-frame log.
    if (!o.csv.empty()) {
        if (std::FILE* f = std::fopen(o.csv.c_str(), "w")) {
            std::fprintf(f, "frame,t,skip,gpu,cb,span,cpu");
            for (int k = 0; k < kSimParts; ++k) std::fprintf(f, ",sim%d", k);
            std::fprintf(f, ",probe,encode,wait,cpu_terrain,cpu_lod,cpu_shadows,cpu_cube,cpu_scene,cpu_effects,cpu_post,pipelines,textures,meshes,chunks");
            for (const std::string& n : m.passOrder) std::fprintf(f, ",%s", n.c_str());
            std::fprintf(f, ",scene_vertex,scene_fragment,scale\n");
            for (const Impl::Rec& r : m.recs) {
                std::fprintf(f, "%d,%.3f,%u,%.3f,%.3f,%.3f,%.3f", r.index, r.sim.t, r.skip, r.gpuMs, r.cbMs, r.spanMs, r.cpuMs());
                for (double x : r.sim.part) std::fprintf(f, ",%.3f", x);
                std::fprintf(f, ",%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f,%d,%d,%d,%d", r.probeMs, r.encodeMs, r.waitMs, r.rc.terrain, r.rc.lod, r.rc.shadows,
                             r.rc.cube, r.rc.scene, r.rc.effects, r.rc.post, r.rc.pipelines, r.rc.textures, r.rc.meshes, r.rc.chunks);
                for (const std::string& n : m.passOrder) {
                    auto it = r.pass.find(n);
                    std::fprintf(f, ",%.3f", it == r.pass.end() ? 0.0 : it->second);
                }
                const auto st = r.stages.find("scene");
                std::fprintf(f, ",%.3f,%.3f,%.4f\n", st == r.stages.end() ? 0.0 : st->second.first, st == r.stages.end() ? 0.0 : st->second.second, r.scale);
            }
            std::fclose(f);
            std::printf("perf check: per-frame log in %s\n", o.csv.c_str());
        }
    }

    const double gpu95 = pct(gpu, 0.95), cpu95 = pct(cpu, 0.95);
    const bool ok = gpu95 <= o.budgetMs && cpu95 <= o.cpuBudgetMs && hitchesJudged == 0;
    std::printf("perf check: GPU p95 %.2f ms (budget %.1f, %d frames over), CPU p95 %.2f ms (budget %.1f, %d over), %d hitches over %.0f ms "
                "(%d after the warm-up): %s\n",
                gpu95, o.budgetMs, overGpu, cpu95, o.cpuBudgetMs, overCpu, hitches, o.hitchMs, hitchesJudged, ok ? "ok" : "FAILED");
    return ok;
}

}  // namespace fuel
