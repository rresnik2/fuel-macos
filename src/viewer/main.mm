// fuelview: browse the meshes in a FUEL .DPC file with Metal.
//
//   fuelview [--data DIR] [--mesh ID | --scene FILE.json] [--vehicle NAME [--livery N] [--color N | --paint R,G,B]]
//            [--shot FILE.png [--size WxH] [--yaw DEG] [--pitch DEG] [--no-cull]] [DPC ...]
//
// DPC paths are relative to the game data directory (the folder that holds
// CommonComponent/); the first one is browsed, the others only supply
// textures and materials. Without --shot it opens a window:
//   drag = orbit, scroll/pinch = zoom, left/right = previous/next mesh,
//   C = back-face culling, W = wireframe, R = reset view, Q/Esc = quit,
//   L = next livery, P = next palette colour (with --vehicle).
// --vehicle NAME (for example QUAD_ALL_1) picks the vehicle whose palette and
// liveries paint the bodywork; --livery 0 shows plain paint.
// --scene loads an assembled vehicle exported by the reverse-engineering
// tools (a JSON list of parts with mesh IDs and Direct3D row-major world
// matrices); its variant name also selects the paint.

#import <Cocoa/Cocoa.h>
#import <ImageIO/ImageIO.h>
#import <Metal/Metal.h>
#import <MetalKit/MetalKit.h>
#import <UniformTypeIdentifiers/UniformTypeIdentifiers.h>

#import "renderer.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace {

constexpr MTLPixelFormat kColorFormat = MTLPixelFormatBGRA8Unorm;
constexpr MTLPixelFormat kDepthFormat = MTLPixelFormatDepth32Float;
constexpr NSUInteger kSamples = 4;

struct Options {
    std::string dataDir;
    std::vector<std::string> dpcs;
    uint32_t meshId = 0;
    bool haveMeshId = false;
    std::string shot;
    std::string scene;
    int width = 1600, height = 1000;
    float yawDeg = 37.0f, pitchDeg = 17.0f;
    bool cull = true;
    std::string vehicle;
    int livery = 1;
    int color = 0;
    bool havePaint = false;
    int paint[3] = {60, 60, 60};
};

// Paint and liveries available for the chosen vehicle.
struct VehicleLook {
    std::string name;
    std::vector<fuel::Rgb8> palette;
    std::array<std::array<uint32_t, 3>, 10> liveries{};  // [1..9]
    int livery = 1;
    int color = 0;
    bool custom = false;
    simd_float3 customColor = {0, 0, 0};

    fuel::Paint paint() const {
        fuel::Paint p;
        if (custom)
            p.color = customColor;
        else if (!palette.empty()) {
            const fuel::Rgb8& c = palette[size_t(color) % palette.size()];
            p.color = simd_make_float3(c[0], c[1], c[2]) / 255.0f;
        }
        if (livery >= 1 && livery <= 9) p.livery = liveries[size_t(livery)];
        return p;
    }
    std::string describe() const {
        if (name.empty()) return "";
        std::string d = " — " + name + " livery " + (livery ? std::to_string(livery) : std::string("none"));
        return d + (custom ? " custom paint" : " colour " + std::to_string(color));
    }
};

struct MeshEntry {
    uint32_t id;
    uint32_t triangles;
    std::string label;
};

[[noreturn]] void die(const std::string& msg) {
    std::fprintf(stderr, "fuelview: %s\n", msg.c_str());
    std::exit(1);
}

std::string defaultDataDir() {
    if (const char* env = std::getenv("FUEL_DATA"); env && *env) return env;
    const char* home = std::getenv("HOME");
    return std::string(home ? home : "") + "/Library/Application Support/FUEL/game";
}

Options parseArgs(int argc, const char** argv) {
    Options o;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&]() -> std::string {
            if (i + 1 >= argc) die("missing value after " + a);
            return argv[++i];
        };
        if (a == "--data") o.dataDir = next();
        else if (a == "--mesh") { o.meshId = uint32_t(std::stoul(next(), nullptr, 16)); o.haveMeshId = true; }
        else if (a == "--shot") o.shot = next();
        else if (a == "--scene") o.scene = next();
        else if (a == "--size") { const std::string s = next(); if (std::sscanf(s.c_str(), "%dx%d", &o.width, &o.height) != 2) die("bad --size"); }
        else if (a == "--yaw") o.yawDeg = std::stof(next());
        else if (a == "--pitch") o.pitchDeg = std::stof(next());
        else if (a == "--no-cull") o.cull = false;
        else if (a == "--vehicle") o.vehicle = next();
        else if (a == "--livery") o.livery = std::stoi(next());
        else if (a == "--color") o.color = std::stoi(next());
        else if (a == "--paint") { const std::string v = next(); if (std::sscanf(v.c_str(), "%d,%d,%d", &o.paint[0], &o.paint[1], &o.paint[2]) != 3) die("bad --paint"); o.havePaint = true; }
        else if (a == "-h" || a == "--help") { std::puts("usage: fuelview [--data DIR] [--mesh ID | --scene FILE.json] [--vehicle NAME [--livery N] [--color N | --paint R,G,B]]\n"
                                                         "                [--shot FILE.png [--size WxH] [--yaw DEG] [--pitch DEG] [--no-cull]] [DPC ...]"); std::exit(0); }
        else if (!a.empty() && a[0] == '-') die("unknown option " + a);
        else o.dpcs.push_back(a);
    }
    if (o.dataDir.empty()) o.dataDir = defaultDataDir();
    if (o.dpcs.empty()) o.dpcs.push_back("CommonComponent/DATAS/QUAD.DPC");
    return o;
}

// Every mesh in the browsed file that decodes, largest first.
std::vector<MeshEntry> listMeshes(const fuel::DpcFile& dpc) {
    std::vector<MeshEntry> out;
    for (uint32_t id : dpc.ids(fuel::cls::Mesh)) {
        try {
            const auto* e = dpc.find(id);
            const fuel::MeshLink link = fuel::parseMeshLink(dpc.link(*e));
            const auto body = dpc.body(*e);
            if (!body) continue;
            const fuel::Mesh m = fuel::parseMeshBody(*body, link.flags);
            fuel::MeshStats st;
            if (!fuel::checkMesh(m, &st).empty() || st.triangles == 0) continue;
            out.push_back({id, st.triangles, m.morphTargets.empty() ? std::string() : m.morphTargets.front()});
        } catch (const std::exception&) {
        }
    }
    std::stable_sort(out.begin(), out.end(), [](const MeshEntry& a, const MeshEntry& b) { return a.triangles > b.triangles; });
    return out;
}

struct SceneFile {
    std::string variant;
    std::vector<fuel::MeshInstance> parts;
};

SceneFile loadScene(const std::string& path) {
    NSData* data = [NSData dataWithContentsOfFile:@(path.c_str())];
    if (data == nil) die("cannot read " + path);
    NSError* err = nil;
    NSDictionary* root = [NSJSONSerialization JSONObjectWithData:data options:0 error:&err];
    if (![root isKindOfClass:[NSDictionary class]]) die("bad scene file " + path);
    SceneFile sf;
    if (NSString* v = root[@"variant"]; [v isKindOfClass:[NSString class]]) sf.variant = v.UTF8String;
    for (NSDictionary* part in root[@"parts"]) {
        NSString* kind = part[@"kind"];
        NSString* meshId = part[@"mesh_id"];
        NSArray* m = part[@"world_matrix"];
        if (![meshId isKindOfClass:[NSString class]] || ![m isKindOfClass:[NSArray class]] || m.count != 16) continue;
        if ([kind hasSuffix:@"_unplaced"] || [kind hasPrefix:@"light"]) continue;
        // Row-major row-vector (p * M) is the transpose of simd's column-vector
        // form, so the file's rows become simd columns.
        simd_float4 cols[4];
        for (int r = 0; r < 4; ++r)
            cols[r] = simd_make_float4([m[r * 4] floatValue], [m[r * 4 + 1] floatValue], [m[r * 4 + 2] floatValue], [m[r * 4 + 3] floatValue]);
        sf.parts.push_back({uint32_t(std::stoul(meshId.UTF8String, nullptr, 16)), simd_matrix(cols[0], cols[1], cols[2], cols[3])});
    }
    if (sf.parts.empty()) die("no drawable parts in " + path);
    return sf;
}

MTLClearColor clearColor() { return MTLClearColorMake(0.13, 0.15, 0.18, 1.0); }

void writePng(id<MTLDevice> device, id<MTLCommandQueue> queue, id<MTLTexture> tex, const std::string& path) {
    const NSUInteger w = tex.width, h = tex.height, rowBytes = w * 4;
    id<MTLBuffer> buf = [device newBufferWithLength:rowBytes * h options:MTLResourceStorageModeShared];
    id<MTLCommandBuffer> cb = [queue commandBuffer];
    id<MTLBlitCommandEncoder> blit = [cb blitCommandEncoder];
    [blit copyFromTexture:tex sourceSlice:0 sourceLevel:0 sourceOrigin:MTLOriginMake(0, 0, 0) sourceSize:MTLSizeMake(w, h, 1)
                 toBuffer:buf destinationOffset:0 destinationBytesPerRow:rowBytes destinationBytesPerImage:rowBytes * h];
    [blit endEncoding];
    [cb commit];
    [cb waitUntilCompleted];

    CGColorSpaceRef cs = CGColorSpaceCreateWithName(kCGColorSpaceSRGB);
    CGDataProviderRef dp = CGDataProviderCreateWithData(nullptr, buf.contents, rowBytes * h, nullptr);
    CGImageRef img = CGImageCreate(w, h, 8, 32, rowBytes, cs, kCGBitmapByteOrder32Little | kCGImageAlphaNoneSkipFirst, dp,
                                   nullptr, false, kCGRenderingIntentDefault);
    NSURL* url = [NSURL fileURLWithPath:@(path.c_str())];
    CGImageDestinationRef dst = CGImageDestinationCreateWithURL((__bridge CFURLRef)url, (__bridge CFStringRef)UTTypePNG.identifier, 1, nullptr);
    if (dst == nullptr) die("cannot write " + path);
    CGImageDestinationAddImage(dst, img, nullptr);
    const bool ok = CGImageDestinationFinalize(dst);
    CFRelease(dst);
    CGImageRelease(img);
    CGDataProviderRelease(dp);
    CGColorSpaceRelease(cs);
    if (!ok) die("cannot write " + path);
}

void renderShot(id<MTLDevice> device, fuel::MeshRenderer& renderer, const Options& o, const fuel::OrbitCamera& cam) {
    id<MTLCommandQueue> queue = [device newCommandQueue];
    auto makeTarget = [&](MTLPixelFormat pf, NSUInteger samples) {
        MTLTextureDescriptor* td = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:pf width:NSUInteger(o.width) height:NSUInteger(o.height) mipmapped:NO];
        td.textureType = samples > 1 ? MTLTextureType2DMultisample : MTLTextureType2D;
        td.sampleCount = samples;
        td.usage = MTLTextureUsageRenderTarget;
        td.storageMode = MTLStorageModePrivate;
        return [device newTextureWithDescriptor:td];
    };
    id<MTLTexture> msaa = makeTarget(kColorFormat, kSamples);
    id<MTLTexture> resolved = makeTarget(kColorFormat, 1);
    id<MTLTexture> depth = makeTarget(kDepthFormat, kSamples);

    MTLRenderPassDescriptor* rp = [MTLRenderPassDescriptor renderPassDescriptor];
    rp.colorAttachments[0].texture = msaa;
    rp.colorAttachments[0].resolveTexture = resolved;
    rp.colorAttachments[0].loadAction = MTLLoadActionClear;
    rp.colorAttachments[0].clearColor = clearColor();
    rp.colorAttachments[0].storeAction = MTLStoreActionMultisampleResolve;
    rp.depthAttachment.texture = depth;
    rp.depthAttachment.loadAction = MTLLoadActionClear;
    rp.depthAttachment.clearDepth = 1.0;
    rp.depthAttachment.storeAction = MTLStoreActionDontCare;

    id<MTLCommandBuffer> cb = [queue commandBuffer];
    id<MTLRenderCommandEncoder> enc = [cb renderCommandEncoderWithDescriptor:rp];
    renderer.encode(enc, float(o.width) / float(o.height), cam);
    [enc endEncoding];
    [cb commit];
    [cb waitUntilCompleted];
    writePng(device, queue, resolved, o.shot);
}

}  // namespace

// ---- interactive window -----------------------------------------------------

@interface FuelView : MTKView <MTKViewDelegate>
@property(nonatomic, copy) void (^onChange)(void);
@end

@implementation FuelView {
    fuel::MeshRenderer* _renderer;
    std::vector<MeshEntry>* _meshes;
    std::string _fileName;
    id<MTLCommandQueue> _queue;
    fuel::OrbitCamera _cam;
    size_t _index;
    VehicleLook _look;
    SceneFile _scene;
}

- (instancetype)initWithFrame:(NSRect)frame
                       device:(id<MTLDevice>)device
                     renderer:(fuel::MeshRenderer*)renderer
                       meshes:(std::vector<MeshEntry>*)meshes
                     fileName:(const std::string&)fileName
                        start:(size_t)start
                       camera:(fuel::OrbitCamera)cam
                         look:(const VehicleLook&)look
                        scene:(const SceneFile&)scene {
    if ((self = [super initWithFrame:frame device:device])) {
        _renderer = renderer;
        _meshes = meshes;
        _fileName = fileName;
        _queue = [device newCommandQueue];
        _cam = cam;
        _index = start;
        _look = look;
        _scene = scene;
        self.colorPixelFormat = kColorFormat;
        self.depthStencilPixelFormat = kDepthFormat;
        self.sampleCount = kSamples;
        self.clearColor = clearColor();
        self.delegate = self;
    }
    return self;
}

- (BOOL)acceptsFirstResponder { return YES; }

- (void)updateTitle {
    if (!_scene.parts.empty()) {
        self.window.title = [NSString stringWithFormat:@"FUEL viewer — %s — %zu parts — %u triangles%s",
                                                       _scene.variant.c_str(), _scene.parts.size(), _renderer->triangleCount(),
                                                       _look.describe().c_str()];
        return;
    }
    const MeshEntry& m = (*_meshes)[_index];
    NSString* title = [NSString stringWithFormat:@"FUEL viewer — %s — mesh %zu/%zu — %08x — %u triangles%s%s%s%s%s",
                                                 _fileName.c_str(), _index + 1, _meshes->size(), m.id, _renderer->triangleCount(),
                                                 m.label.empty() ? "" : " — ", m.label.c_str(), _look.describe().c_str(),
                                                 _renderer->cullBackFaces ? " — cull on" : "", _renderer->wireframe ? " — wireframe" : ""];
    self.window.title = title;
}

- (void)showMesh:(size_t)index {
    _index = index;
    std::string err;
    if (!_scene.parts.empty()) {
        if (!_renderer->setScene(_scene.parts, &err) || !err.empty()) std::fprintf(stderr, "fuelview: %s\n", err.c_str());
        [self updateTitle];
        return;
    }
    if (!_renderer->setMesh((*_meshes)[_index].id, &err)) std::fprintf(stderr, "fuelview: mesh %08x: %s\n", (*_meshes)[_index].id, err.c_str());
    [self updateTitle];
}

- (void)keyDown:(NSEvent*)e {
    const size_t n = _scene.parts.empty() ? _meshes->size() : 1;
    switch (e.keyCode) {
        case 123: [self showMesh:(_index + n - 1) % n]; return;  // left
        case 124: [self showMesh:(_index + 1) % n]; return;      // right
        case 126: _cam.distance = std::max(0.4f, _cam.distance / 1.15f); return;  // up
        case 125: _cam.distance = std::min(20.0f, _cam.distance * 1.15f); return; // down
        case 53: [NSApp terminate:nil]; return;                   // esc
        default: break;
    }
    NSString* ch = e.charactersIgnoringModifiers.lowercaseString;
    if ([ch isEqualToString:@"c"]) _renderer->cullBackFaces = !_renderer->cullBackFaces;
    else if ([ch isEqualToString:@"w"]) _renderer->wireframe = !_renderer->wireframe;
    else if ([ch isEqualToString:@"r"]) _cam = fuel::OrbitCamera{};
    else if ([ch isEqualToString:@"q"]) [NSApp terminate:nil];
    else if (!_look.name.empty() && ([ch isEqualToString:@"l"] || [ch isEqualToString:@"p"])) {
        if ([ch isEqualToString:@"l"]) _look.livery = (_look.livery + 1) % 10;
        else { _look.custom = false; _look.color = (_look.color + 1) % int(std::max<size_t>(1, _look.palette.size())); }
        _renderer->setPaint(_look.paint());
        [self showMesh:_index];
        return;
    }
    [self updateTitle];
}

- (void)mouseDragged:(NSEvent*)e {
    _cam.yaw -= float(e.deltaX) * 0.008f;
    _cam.pitch = std::clamp(_cam.pitch + float(e.deltaY) * 0.008f, -1.45f, 1.45f);
}

- (void)scrollWheel:(NSEvent*)e {
    _cam.distance = std::clamp(_cam.distance * std::exp(-float(e.scrollingDeltaY) * (e.hasPreciseScrollingDeltas ? 0.01f : 0.1f)), 0.4f, 20.0f);
}

- (void)magnifyWithEvent:(NSEvent*)e {
    _cam.distance = std::clamp(_cam.distance / (1.0f + float(e.magnification)), 0.4f, 20.0f);
}

- (void)mtkView:(MTKView*)view drawableSizeWillChange:(CGSize)size {}

- (void)drawInMTKView:(MTKView*)view {
    MTLRenderPassDescriptor* rp = view.currentRenderPassDescriptor;
    if (rp == nil) return;
    id<MTLCommandBuffer> cb = [_queue commandBuffer];
    id<MTLRenderCommandEncoder> enc = [cb renderCommandEncoderWithDescriptor:rp];
    const CGSize s = view.drawableSize;
    _renderer->encode(enc, s.height > 0 ? float(s.width / s.height) : 1.0f, _cam);
    [enc endEncoding];
    [cb presentDrawable:view.currentDrawable];
    [cb commit];
}
@end

@interface AppDelegate : NSObject <NSApplicationDelegate>
@end
@implementation AppDelegate
- (BOOL)applicationShouldTerminateAfterLastWindowClosed:(NSApplication*)app { return YES; }
@end

int main(int argc, const char** argv) {
    @autoreleasepool {
        const Options opt = parseArgs(argc, argv);

        fuel::ResourceLibrary library;
        for (const std::string& rel : opt.dpcs) {
            const fs::path p = fs::path(rel).is_absolute() ? fs::path(rel) : fs::path(opt.dataDir) / rel;
            if (!fs::exists(p))
                die("cannot find " + p.string() + "\nPoint --data (or FUEL_DATA) at the folder that holds CommonComponent/ from your FUEL install.");
            try {
                auto f = std::make_unique<fuel::DpcFile>(p.string());
                if (!f->isCurrentVersion())
                    std::fprintf(stderr, "fuelview: warning: %s is version '%s'; the game never loads it\n", p.filename().c_str(), f->version().c_str());
                library.add(std::move(f));
            } catch (const std::exception& e) {
                die(e.what());
            }
        }
        const fuel::DpcFile& subject = *library.files().front();
        std::vector<MeshEntry> meshes = listMeshes(subject);
        if (meshes.empty()) die("no drawable meshes in " + subject.path());

        size_t start = 0;
        if (opt.haveMeshId) {
            auto it = std::find_if(meshes.begin(), meshes.end(), [&](const MeshEntry& m) { return m.id == opt.meshId; });
            if (it == meshes.end()) die("mesh not found or not drawable");
            start = size_t(it - meshes.begin());
        }

        id<MTLDevice> device = MTLCreateSystemDefaultDevice();
        if (device == nil) die("no Metal device");
        if (!device.supportsBCTextureCompression) die("this GPU cannot sample DXT (BC) textures");
        fuel::MeshRenderer renderer(device, kColorFormat, kDepthFormat, kSamples, library);
        renderer.cullBackFaces = opt.cull;

        SceneFile scene;
        if (!opt.scene.empty()) scene = loadScene(opt.scene);
        const std::string vehicle = !opt.vehicle.empty() ? opt.vehicle : scene.variant;

        VehicleLook look;
        if (!vehicle.empty()) {
            look.name = vehicle;
            const fs::path tsc = fs::path(opt.dataDir) / "LocalizedComponent/GameTsc/Story/VehiclesDef.tsc";
            if (FILE* f = std::fopen(tsc.c_str(), "rb")) {
                std::string text;
                char buf[65536];
                for (size_t n; (n = std::fread(buf, 1, sizeof buf, f)) > 0;) text.append(buf, n);
                std::fclose(f);
                const auto colors = fuel::parseVehicleColors(text);
                if (auto it = colors.find(vehicle); it != colors.end()) look.palette = it->second;
            }
            if (look.palette.empty()) std::fprintf(stderr, "fuelview: warning: no palette for %s in VehiclesDef.tsc\n", vehicle.c_str());
            for (int i = 1; i <= 9; ++i) look.liveries[size_t(i)] = fuel::findLiveryTextures(subject, vehicle, i);
            if (look.liveries[1][0] == 0) std::fprintf(stderr, "fuelview: warning: no liveries for %s in %s\n", vehicle.c_str(), subject.path().c_str());
            look.livery = std::clamp(opt.livery, 0, 9);
            look.color = std::max(0, opt.color);
            if (opt.havePaint) {
                look.custom = true;
                look.customColor = simd_make_float3(opt.paint[0], opt.paint[1], opt.paint[2]) / 255.0f;
            }
            renderer.setPaint(look.paint());
        }

        fuel::OrbitCamera cam;
        cam.yaw = opt.yawDeg * float(M_PI) / 180.0f;
        cam.pitch = opt.pitchDeg * float(M_PI) / 180.0f;

        if (!opt.shot.empty()) {
            std::string err;
            if (!scene.parts.empty()) {
                if (!renderer.setScene(scene.parts, &err)) die(err);
                if (!err.empty()) std::fprintf(stderr, "fuelview: %s\n", err.c_str());
                renderShot(device, renderer, opt, cam);
                std::printf("wrote %s: %s, %zu parts, %u triangles\n", opt.shot.c_str(), scene.variant.c_str(), scene.parts.size(), renderer.triangleCount());
                return 0;
            }
            if (!renderer.setMesh(meshes[start].id, &err)) die(err);
            renderShot(device, renderer, opt, cam);
            std::printf("wrote %s: mesh %08x, %u triangles\n", opt.shot.c_str(), meshes[start].id, renderer.triangleCount());
            return 0;
        }

        [NSApplication sharedApplication];
        [NSApp setActivationPolicy:NSApplicationActivationPolicyRegular];
        AppDelegate* appDelegate = [AppDelegate new];
        NSApp.delegate = appDelegate;

        const NSRect frame = NSMakeRect(0, 0, 1280, 800);
        NSWindow* window = [[NSWindow alloc] initWithContentRect:frame
                                                       styleMask:NSWindowStyleMaskTitled | NSWindowStyleMaskClosable |
                                                                 NSWindowStyleMaskResizable | NSWindowStyleMaskMiniaturizable
                                                         backing:NSBackingStoreBuffered
                                                           defer:NO];
        FuelView* view = [[FuelView alloc] initWithFrame:frame
                                                  device:device
                                                renderer:&renderer
                                                  meshes:&meshes
                                                fileName:fs::path(subject.path()).filename().string()
                                                   start:start
                                                  camera:cam
                                                    look:look
                                                   scene:scene];
        window.contentView = view;
        [window center];
        [window makeKeyAndOrderFront:nil];
        [window makeFirstResponder:view];
        [view showMesh:start];
        [NSApp activateIgnoringOtherApps:YES];
        [NSApp run];
    }
    return 0;
}
