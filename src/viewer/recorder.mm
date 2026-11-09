#import "recorder.h"

#import <CoreGraphics/CoreGraphics.h>
#import <ImageIO/ImageIO.h>
#import <QuartzCore/QuartzCore.h>
#import <UniformTypeIdentifiers/UniformTypeIdentifiers.h>

#include <atomic>
#include <cmath>
#include <filesystem>
#include <mutex>
#include <vector>

namespace fuel {

namespace {

constexpr double kLongFrame = 1.0 / 40.0;  // flag frames slower than 40 fps
constexpr size_t kMaxGrabWidth = 1280;     // grabs are scaled down to this width

std::atomic<double> gGpuMs{0};

// Writes BGRA8 pixels as a JPEG, scaled down to kMaxGrabWidth.
void writeJpeg(const std::string& path, const uint8_t* bgra, size_t w, size_t h, size_t rowBytes) {
    CGColorSpaceRef cs = CGColorSpaceCreateWithName(kCGColorSpaceSRGB);
    CGContextRef src = CGBitmapContextCreate(const_cast<uint8_t*>(bgra), w, h, 8, rowBytes, cs,
                                             kCGImageAlphaNoneSkipFirst | kCGBitmapByteOrder32Little);
    CGImageRef full = src ? CGBitmapContextCreateImage(src) : nullptr;
    CGImageRef out = full;
    if (full && w > kMaxGrabWidth) {
        const size_t dw = kMaxGrabWidth, dh = h * kMaxGrabWidth / w;
        CGContextRef dst = CGBitmapContextCreate(nullptr, dw, dh, 8, 0, cs, kCGImageAlphaNoneSkipFirst | kCGBitmapByteOrder32Little);
        if (dst) {
            CGContextSetInterpolationQuality(dst, kCGInterpolationMedium);
            CGContextDrawImage(dst, CGRectMake(0, 0, dw, dh), full);
            out = CGBitmapContextCreateImage(dst);
            CGContextRelease(dst);
        }
    }
    if (out) {
        NSURL* url = [NSURL fileURLWithPath:[NSString stringWithUTF8String:path.c_str()]];
        CGImageDestinationRef d = CGImageDestinationCreateWithURL((__bridge CFURLRef)url, (__bridge CFStringRef)UTTypeJPEG.identifier, 1, nullptr);
        if (d) {
            NSDictionary* props = @{(__bridge NSString*)kCGImageDestinationLossyCompressionQuality : @0.75};
            CGImageDestinationAddImage(d, out, (__bridge CFDictionaryRef)props);
            CGImageDestinationFinalize(d);
            CFRelease(d);
        }
    }
    if (out && out != full) CGImageRelease(out);
    if (full) CGImageRelease(full);
    if (src) CGContextRelease(src);
    CGColorSpaceRelease(cs);
}

constexpr size_t kRingW = 640;          // small frames for the rolling buffer
constexpr size_t kRingFrames = 90;      // ~1.5 s at 60 fps
constexpr float kFlickerJump = 0.10f;   // mean-brightness jump (0..1) against both neighbours

const char* kScaleShader = R"(
#include <metal_stdlib>
using namespace metal;
struct V { float4 p [[position]]; float2 uv; };
vertex V vsScale(uint id [[vertex_id]]) {
    float2 uv = float2((id << 1) & 2, id & 2);
    V o; o.p = float4(uv * float2(2, -2) + float2(-1, 1), 0, 1); o.uv = uv; return o;
}
fragment float4 fsScale(V in [[stage_in]], texture2d<float> t [[texture(0)]]) {
    constexpr sampler s(filter::linear, address::clamp_to_edge);
    return t.sample(s, in.uv);
}
)";

}  // namespace

// The last kRingFrames frames, scaled to kRingW wide, in shared buffers the GPU fills each frame.
struct Recorder::Ring {
    id<MTLRenderPipelineState> pipe = nil;
    id<MTLTexture> small = nil;
    size_t w = 0, h = 0;
    std::vector<id<MTLBuffer>> bufs;
    std::vector<double> times;   // capture time of each slot (-1 = empty)
    std::vector<float> mean;     // mean brightness once the slot's frame completed (-1 = pending)
    size_t next = 0;
    uint64_t frames = 0;
    std::mutex m;
    std::string dir;
    double lastDump = -10;
};

namespace {

float meanBrightness(const uint8_t* p, size_t w, size_t h) {
    double s = 0;
    size_t n = 0;
    for (size_t y = 0; y < h; y += 8)
        for (size_t x = 0; x < w; x += 8, ++n) {
            const uint8_t* q = p + (y * w + x) * 4;
            s += (q[0] + q[1] + q[2]) / 765.0;
        }
    return n ? float(s / double(n)) : 0.0f;
}

// Writes ring slots (copied under the lock) as JPEGs into `sub`, oldest first.
void dumpSlots(std::string sub, std::vector<std::pair<double, std::vector<uint8_t>>> frames, size_t w, size_t h) {  // by value: the block outlives the caller
    std::error_code ec;
    std::filesystem::create_directories(sub, ec);
    dispatch_async(dispatch_get_global_queue(QOS_CLASS_UTILITY, 0), ^{
        for (const auto& [t, px] : frames) {
            char name[48];
            std::snprintf(name, sizeof name, "/r_%07.0f.jpg", t * 1000.0);
            writeJpeg(sub + name, px.data(), w, h, w * 4);
        }
    });
}

}  // namespace

bool Recorder::open(const std::string& dir, float interval) {
    std::error_code ec;
    std::filesystem::create_directories(dir, ec);
    log_ = std::fopen((dir + "/frames.csv").c_str(), "w");
    if (log_ == nullptr) return false;
    marks_ = std::fopen((dir + "/marks.txt").c_str(), "w");
    dir_ = dir;
    interval_ = interval;
    start_ = CACurrentMediaTime();
    std::fprintf(log_, "t,dt_ms,physics_ms,stream_ms,encode_ms,gpu_ms,kmh,gear,wheels,vehicles,x,y,z,note\n");
    return true;
}

Recorder::~Recorder() {
    if (log_) std::fclose(log_);
    if (marks_) std::fclose(marks_);
}

double Recorder::now() const { return CACurrentMediaTime() - start_; }

bool Recorder::frame(const Frame& f, const char* note) {
    if (!log_) return false;
    const double t = now();
    std::string n = note ? note : "";
    if (f.dt > kLongFrame) n += n.empty() ? "LONG" : " LONG";
    if (grabNext_ && !grabNote_.empty()) n += (n.empty() ? "" : " ") + grabNote_;
    std::fprintf(log_, "%.3f,%.1f,%.2f,%.2f,%.2f,%.2f,%.1f,%d,%d,%d,%.1f,%.1f,%.1f,%s\n", t, f.dt * 1000.0, f.physicsMs, f.streamMs,
                 f.encodeMs, gGpuMs.load(), f.speedKmh, f.gear, f.wheelsOnGround, f.vehicles, f.x, f.y, f.z, n.c_str());
    // Long frames grab at most twice a second so a stutter does not flood the folder.
    const bool longGrab = f.dt > kLongFrame * 2 && t > nextGrab_ - interval_ * 0.5;
    if (grabNext_ || t >= nextGrab_ || longGrab) {
        grabNext_ = false;
        grabNote_.clear();
        nextGrab_ = t + interval_;
        return true;
    }
    return false;
}

void Recorder::mark(const char* what) {
    if (!log_) return;
    const double t = now();
    if (marks_) {
        std::fprintf(marks_, "%.3f %s\n", t, what);
        std::fflush(marks_);
    }
    std::printf("record: %s at %.1f s\n", what, t);
    grabNext_ = true;
    grabNote_ = "MARK";
    // The rolling buffer: every completed slot of the last ~1.5 s goes to DIR/k<ms>/.
    if (auto r = ring_) {
        std::vector<std::pair<double, std::vector<uint8_t>>> frames;
        {
            std::lock_guard<std::mutex> lock(r->m);
            for (size_t k = 0; k < r->bufs.size(); ++k) {
                const size_t i = (r->next + k) % r->bufs.size();
                if (r->times[i] < 0 || r->mean[i] < 0) continue;  // empty, or its frame is still on the GPU
                const auto* p = static_cast<const uint8_t*>(r->bufs[i].contents);
                frames.push_back({r->times[i], std::vector<uint8_t>(p, p + r->w * r->h * 4)});
            }
        }
        char sub[48];
        std::snprintf(sub, sizeof sub, "/k%07.0f", t * 1000.0);
        dumpSlots(dir_ + sub, std::move(frames), r->w, r->h);
    }
}

void Recorder::capture(id<MTLCommandBuffer> cb, id<MTLTexture> tex) {
    if (!log_ || tex == nil || tex.framebufferOnly) return;
    if (!ring_) {
        auto r = std::make_shared<Ring>();
        id<MTLDevice> dev = cb.device;
        NSError* err = nil;
        id<MTLLibrary> lib = [dev newLibraryWithSource:@(kScaleShader) options:nil error:&err];
        MTLRenderPipelineDescriptor* pd = [MTLRenderPipelineDescriptor new];
        pd.vertexFunction = [lib newFunctionWithName:@"vsScale"];
        pd.fragmentFunction = [lib newFunctionWithName:@"fsScale"];
        pd.colorAttachments[0].pixelFormat = tex.pixelFormat;
        r->pipe = lib ? [dev newRenderPipelineStateWithDescriptor:pd error:&err] : nil;
        if (r->pipe == nil) {
            std::fprintf(stderr, "record: no rolling buffer (%s)\n", err ? err.localizedDescription.UTF8String : "shader");
            ring_ = r;    // an empty ring: capture() does nothing more
            return;
        }
        r->w = kRingW;
        r->h = std::max<size_t>(1, tex.height * kRingW / std::max<size_t>(1, tex.width));
        MTLTextureDescriptor* td = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:tex.pixelFormat width:r->w height:r->h mipmapped:NO];
        td.usage = MTLTextureUsageRenderTarget;
        td.storageMode = MTLStorageModePrivate;
        r->small = [dev newTextureWithDescriptor:td];
        for (size_t k = 0; k < kRingFrames; ++k) r->bufs.push_back([dev newBufferWithLength:r->w * r->h * 4 options:MTLResourceStorageModeShared]);
        r->times.assign(kRingFrames, -1.0);
        r->mean.assign(kRingFrames, -1.0f);
        r->dir = dir_;
        ring_ = r;
    }
    auto r = ring_;
    if (r->pipe == nil) return;
    size_t slot;
    {
        std::lock_guard<std::mutex> lock(r->m);
        slot = r->next;
        r->next = (r->next + 1) % r->bufs.size();
        r->times[slot] = now();
        r->mean[slot] = -1.0f;
        ++r->frames;
    }
    MTLRenderPassDescriptor* rp = [MTLRenderPassDescriptor renderPassDescriptor];
    rp.colorAttachments[0].texture = r->small;
    rp.colorAttachments[0].loadAction = MTLLoadActionDontCare;
    rp.colorAttachments[0].storeAction = MTLStoreActionStore;
    id<MTLRenderCommandEncoder> enc = [cb renderCommandEncoderWithDescriptor:rp];
    [enc setRenderPipelineState:r->pipe];
    [enc setFragmentTexture:tex atIndex:0];
    [enc drawPrimitives:MTLPrimitiveTypeTriangle vertexStart:0 vertexCount:3];
    [enc endEncoding];
    id<MTLBlitCommandEncoder> blit = [cb blitCommandEncoder];
    [blit copyFromTexture:r->small sourceSlice:0 sourceLevel:0 sourceOrigin:MTLOriginMake(0, 0, 0) sourceSize:MTLSizeMake(r->w, r->h, 1)
                 toBuffer:r->bufs[slot] destinationOffset:0 destinationBytesPerRow:r->w * 4 destinationBytesPerImage:r->w * r->h * 4];
    [blit endEncoding];
    std::FILE* marks = marks_;
    [cb addCompletedHandler:^(id<MTLCommandBuffer>) {
        std::vector<std::pair<double, std::vector<uint8_t>>> frames;
        double flickerT = -1;
        {
            std::lock_guard<std::mutex> lock(r->m);
            r->mean[slot] = meanBrightness(static_cast<const uint8_t*>(r->bufs[slot].contents), r->w, r->h);
            // A spike at the previous slot: brighter or darker than both its neighbours by kFlickerJump.
            const size_t n = r->bufs.size(), b = (slot + n - 1) % n, a = (slot + n - 2) % n;
            const float ma = r->mean[a], mb = r->mean[b], mc = r->mean[slot];
            if (ma >= 0 && mb >= 0 && r->times[b] > r->lastDump + 1.0) {
                const float d1 = mb - ma, d2 = mb - mc;
                if (std::fabs(d1) > kFlickerJump && std::fabs(d2) > kFlickerJump && (d1 > 0) == (d2 > 0)) {
                    flickerT = r->times[b];
                    r->lastDump = flickerT;
                    for (size_t k = 0; k < n; ++k) {  // the last ~0.5 s up to this frame
                        const size_t i = (slot + n - 30 + k) % n;
                        if (k > 30 || r->times[i] < 0 || r->mean[i] < 0) continue;
                        const auto* p = static_cast<const uint8_t*>(r->bufs[i].contents);
                        frames.push_back({r->times[i], std::vector<uint8_t>(p, p + r->w * r->h * 4)});
                    }
                }
            }
        }
        if (flickerT >= 0) {
            if (marks) {
                std::fprintf(marks, "%.3f FLICKER\n", flickerT);
                std::fflush(marks);
            }
            std::printf("record: FLICKER at %.1f s\n", flickerT);
            char sub[48];
            std::snprintf(sub, sizeof sub, "/flicker_%07.0f", flickerT * 1000.0);
            dumpSlots(r->dir + sub, std::move(frames), r->w, r->h);
        }
    }];
}

void Recorder::watchGpu(id<MTLCommandBuffer> cb) {
    [cb addCompletedHandler:^(id<MTLCommandBuffer> b) {
        if (b.GPUEndTime > b.GPUStartTime) gGpuMs = (b.GPUEndTime - b.GPUStartTime) * 1000.0;
    }];
}

void Recorder::grab(id<MTLCommandBuffer> cb, id<MTLTexture> tex) {
    if (!log_ || tex == nil || tex.framebufferOnly || tex.pixelFormat != MTLPixelFormatBGRA8Unorm_sRGB) return;
    const size_t w = tex.width, h = tex.height, row = w * 4;
    id<MTLBuffer> buf = [cb.device newBufferWithLength:row * h options:MTLResourceStorageModeShared];
    if (buf == nil) return;
    id<MTLBlitCommandEncoder> blit = [cb blitCommandEncoder];
    [blit copyFromTexture:tex sourceSlice:0 sourceLevel:0 sourceOrigin:MTLOriginMake(0, 0, 0) sourceSize:MTLSizeMake(w, h, 1)
                 toBuffer:buf destinationOffset:0 destinationBytesPerRow:row destinationBytesPerImage:row * h];
    [blit endEncoding];
    char name[64];
    std::snprintf(name, sizeof name, "/f_%07.0f.jpg", now() * 1000.0);
    const std::string path = dir_ + name;
    ++written_;
    [cb addCompletedHandler:^(id<MTLCommandBuffer>) {
        dispatch_async(dispatch_get_global_queue(QOS_CLASS_UTILITY, 0), ^{
            writeJpeg(path, static_cast<const uint8_t*>(buf.contents), w, h, row);
        });
    }];
}

}  // namespace fuel
