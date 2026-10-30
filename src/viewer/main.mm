// fuelview: browse the meshes in a FUEL .DPC file with Metal.
//
//   fuelview [--data DIR] [--mesh ID | --assemble VARIANT [--lod N] | --scene FILE.json | --world]
//            [--vehicle NAME] [--livery N] [--color N | --paint R,G,B]
//            [--shot FILE.png [--size WxH] [--yaw DEG] [--pitch DEG] [--distance RADII] [--no-cull] [--dirt 0..1]] [DPC ...]
//
// DPC paths are relative to the game data directory (the folder that holds
// CommonComponent/); the first one is browsed, the others only supply
// textures and materials. Without --shot it opens a window:
//   drag = orbit, scroll/pinch = zoom, left/right = previous/next mesh
//   (with --assemble: previous/next variant), C = back-face culling,
//   W = wireframe, R = reset view, Q/Esc = quit,
//   L = next livery, P = next palette colour (when a vehicle is painted),
//   D = dirt level, G = ground or terrain.
// --terrain draws the world heightfield; --at X,Z (metres) stands the vehicle on it there.
// --assemble VARIANT (for example QUAD_ROAD_2, or a GameObj_Z ID in hex)
// builds that vehicle variant from the DPC (core/assembly.hpp) at LOD N
// (default 1) and paints it with the variant's palette and liveries.
// --vehicle NAME (for example QUAD_ALL_1) picks the vehicle whose palette and
// liveries paint the bodywork when browsing meshes; --livery 0 shows plain paint.
// --scene loads an assembled vehicle exported by the reverse-engineering
// tools (a JSON list of parts with mesh IDs and Direct3D row-major world
// matrices); its variant name also selects the paint.

#import <Cocoa/Cocoa.h>
#import <ImageIO/ImageIO.h>
#import <Metal/Metal.h>
#import <MetalKit/MetalKit.h>
#import <UniformTypeIdentifiers/UniformTypeIdentifiers.h>

#import "renderer.h"
#import "drive.h"
#include "rider_view.h"
#import <GameController/GameController.h>
#include "audio/drive_audio.h"
#include "core/races.hpp"
#include "core/vehicle_params.hpp"
#include <fstream>
#include <set>
#include <sstream>
#include "core/terrain.hpp"
#include "core/world.hpp"

#include "core/assembly.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace {

simd_float4x4 matrix_from_translation(float x, float y, float z) {
    simd_float4x4 m = matrix_identity_float4x4;
    m.columns[3] = simd_make_float4(x, y, z, 1);
    return m;
}

constexpr MTLPixelFormat kColorFormat = fuel::kRenderColorFormat;
constexpr MTLPixelFormat kDepthFormat = fuel::kRenderDepthFormat;
constexpr NSUInteger kSamples = 4;

struct Options {
    std::string dataDir;
    std::vector<std::string> dpcs;
    uint32_t meshId = 0;
    bool haveMeshId = false;
    std::string shot;
    std::string scene;
    bool world = false;
    bool terrain = false;
    bool drive = false;
    bool audioCheck = false;
    bool propCheck = false;
    std::string race;
    float driveTime = 4.0f;
    bool haveAt = false;
    float atX = 0, atZ = 0;
    std::string assemble;
    int lod = 1;
    int width = 1600, height = 1000;
    float yawDeg = 37.0f, pitchDeg = 17.0f;
    float distance = 0;
    bool cull = true;
    float dirt = 0.0f;
    std::string vehicle;
    int livery = 1;
    int color = 0;
    bool havePaint = false;
    int paint[3] = {60, 60, 60};
    bool noRider = false;
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
        else if (a == "--world") o.world = true;
        else if (a == "--terrain") o.terrain = true;
        else if (a == "--drive") { o.drive = true; o.terrain = true; }
        else if (a == "--audio-check") o.audioCheck = true;
        else if (a == "--prop-check") { o.propCheck = true; o.drive = true; o.terrain = true; }
        else if (a == "--drive-time") o.driveTime = std::stof(next());
        else if (a == "--race") { o.race = next(); o.drive = true; o.terrain = true; }
        else if (a == "--at") { const std::string v = next(); if (std::sscanf(v.c_str(), "%f,%f", &o.atX, &o.atZ) != 2) die("bad --at (use X,Z in metres)"); o.haveAt = true; o.terrain = true; }
        else if (a == "--assemble") o.assemble = next();
        else if (a == "--lod") { o.lod = std::stoi(next()); if (o.lod < 1) die("--lod starts at 1"); }
        else if (a == "--size") { const std::string s = next(); if (std::sscanf(s.c_str(), "%dx%d", &o.width, &o.height) != 2) die("bad --size"); }
        else if (a == "--yaw") o.yawDeg = std::stof(next());
        else if (a == "--pitch") o.pitchDeg = std::stof(next());
        else if (a == "--distance") o.distance = std::stof(next());
        else if (a == "--no-cull") o.cull = false;
        else if (a == "--dirt") o.dirt = std::stof(next());
        else if (a == "--no-rider") o.noRider = true;
        else if (a == "--vehicle") o.vehicle = next();
        else if (a == "--livery") o.livery = std::stoi(next());
        else if (a == "--color") o.color = std::stoi(next());
        else if (a == "--paint") { const std::string v = next(); if (std::sscanf(v.c_str(), "%d,%d,%d", &o.paint[0], &o.paint[1], &o.paint[2]) != 3) die("bad --paint"); o.havePaint = true; }
        else if (a == "-h" || a == "--help") { std::puts("usage: fuelview [--data DIR] [--mesh ID | --assemble VARIANT [--lod N] | --scene FILE.json]\n"
                                                         "                [--vehicle NAME] [--livery N] [--color N | --paint R,G,B]\n"
                                                         "                [--shot FILE.png [--size WxH] [--yaw DEG] [--pitch DEG] [--distance RADII] [--no-cull] [--dirt 0..1]] [DPC ...]"); std::exit(0); }
        else if (!a.empty() && a[0] == '-') die("unknown option " + a);
        else o.dpcs.push_back(a);
    }
    if (!o.assemble.empty() && (!o.scene.empty() || o.haveMeshId)) die("--assemble cannot be combined with --scene or --mesh");
    if (!o.assemble.empty() && !o.vehicle.empty()) die("with --assemble the paint follows the variant; drop --vehicle");
    if (o.propCheck && o.shot.empty()) o.shot = "-";  // headless: drive and report, no picture
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

// Row-major row-vector (p * M) is the transpose of simd's column-vector form,
// so the rows become simd columns.
simd_float4x4 matrixFromRows(const float m[16]) {
    return simd_matrix(simd_make_float4(m[0], m[1], m[2], m[3]), simd_make_float4(m[4], m[5], m[6], m[7]),
                       simd_make_float4(m[8], m[9], m[10], m[11]), simd_make_float4(m[12], m[13], m[14], m[15]));
}

// The parts the game draws (placed meshes; not lights or unplaced group nodes).
SceneFile sceneFromAssembly(const fuel::AssembledVehicle& v, const std::string& label) {
    SceneFile sf;
    sf.variant = label;
    for (const fuel::PlacedPart& p : v.parts) {
        if (!p.drawn()) continue;
        float m[16];
        for (int i = 0; i < 16; ++i) m[i] = float(p.world[size_t(i)]);
        sf.parts.push_back({p.meshId, matrixFromRows(m)});
    }
    return sf;
}

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
        float rows[16];
        for (int i = 0; i < 16; ++i) rows[i] = [m[NSUInteger(i)] floatValue];
        sf.parts.push_back({uint32_t(std::stoul(meshId.UTF8String, nullptr, 16)), matrixFromRows(rows)});
    }
    if (sf.parts.empty()) die("no drawable parts in " + path);
    return sf;
}

using Palettes = std::map<std::string, std::vector<fuel::Rgb8>>;

Palettes loadPalettes(const std::string& dataDir) {
    const fs::path tsc = fs::path(dataDir) / "LocalizedComponent/GameTsc/Story/VehiclesDef.tsc";
    FILE* f = std::fopen(tsc.c_str(), "rb");
    if (f == nullptr) return {};
    std::string text;
    char buf[65536];
    for (size_t n; (n = std::fread(buf, 1, sizeof buf, f)) > 0;) text.append(buf, n);
    std::fclose(f);
    return fuel::parseVehicleColors(text);
}

// Palette and liveries of the vehicle whose name has CRC `nameCrc` (palette
// names and livery tables are matched by CRC, so variants whose name the DPC
// never spells out still find theirs). Livery and colour choices come from
// `prefs`.
VehicleLook makeLook(const std::string& label, uint32_t nameCrc, const Palettes& palettes, const fuel::DpcFile& subject,
                     const VehicleLook& prefs) {
    VehicleLook look = prefs;
    look.name = label;
    look.palette.clear();
    for (const auto& [name, colors] : palettes)
        if (fuel::asoboCrc(name) == nameCrc) {
            look.palette = colors;
            if (!label.empty() && label[0] == '?') look.name = name;
            break;
        }
    if (look.palette.empty()) std::fprintf(stderr, "fuelview: warning: no palette for %s in VehiclesDef.tsc\n", label.c_str());
    for (int i = 1; i <= 9; ++i) look.liveries[size_t(i)] = fuel::findLiveryTextures(subject, nameCrc, i);
    if (look.liveries[1][0] == 0) std::fprintf(stderr, "fuelview: warning: no liveries for %s in %s\n", label.c_str(), subject.path().c_str());
    return look;
}

// --assemble: the variants of the browsed DPC and the one shown.
struct AssemblyMode {
    std::unique_ptr<fuel::VehicleAssembler> assembler;
    size_t index = 0;
    int lod = 1;
    Palettes palettes;
    RiderView* riders = nullptr;  // null: no rider

    const fuel::VehicleVariant& variant() const { return assembler->variants()[index]; }
    // Assembles the current variant; on failure the scene is empty and the reason is printed.
    SceneFile scene() const {
        try {
            SceneFile sf = sceneFromAssembly(assembler->assemble(variant(), lod), variant().label);
            if (riders)
                for (uint32_t id : riders->parts(variant().label)) sf.parts.push_back({id, matrix_identity_float4x4});
            return sf;
        } catch (const std::exception& e) {
            std::fprintf(stderr, "fuelview: %s: %s\n", variant().label.c_str(), e.what());
            return SceneFile{variant().label, {}};
        }
    }
    VehicleLook look(const VehicleLook& prefs) const {
        return makeLook(variant().label, variant().name, palettes, assembler->dpc(), prefs);
    }
};

// --drive: every drivable vehicle in the vehicle archives, for Tab switching.
struct Garage {
    std::vector<std::unique_ptr<AssemblyMode>> archives;  // one per vehicle DPC
    std::vector<std::pair<size_t, size_t>> entries;       // (archive, variant)
    size_t current = 0;
    std::map<std::string, std::vector<float>> params;     // Param/VehiclesParam.tsc
    const fuel::Heightfield* terrain = nullptr;
    RiderView* riders = nullptr;

    // Builds entry `i` at (x, z) facing `yaw`; nullptr (with a message) if it cannot drive.
    std::unique_ptr<DriveSession> build(size_t i, double x, double z, float yaw) const {
        AssemblyMode& am = *archives[entries[i].first];
        am.index = entries[i].second;
        const std::string name = am.variant().label;
        try {
            const auto it = params.find(name);
            if (it == params.end()) throw std::runtime_error("no driving parameters");
            const fuel::AssembledVehicle av = am.assembler->assemble(am.variant(), am.lod);
            const fuel::VehicleSetup setup = fuel::makeVehicleSetup(am.assembler->dpc(), name, it->second, assembledHubs(av));
            return std::make_unique<DriveSession>(av, setup, *terrain, x, z, yaw, riders ? riders->parts(name) : std::vector<uint32_t>{});
        } catch (const std::exception& e) {
            std::fprintf(stderr, "fuelview: %s: %s\n", name.c_str(), e.what());
            return nullptr;
        }
    }
};

// --race: FUEL's races (start line, ordered checkpoints, finish, laps).
struct RaceSession {
    enum State { Countdown, Running, Finished };
    static constexpr float kRadius = 25.0f;  // the scripts give no gate width
    std::vector<fuel::Race> races;           // those with a usable route
    size_t index = 0;
    State state = Countdown;
    std::vector<std::array<float, 3>> route;  // targets in order: checkpoints (per lap), then the finish
    size_t next = 0;
    double clock = 0;  // seconds since the start (negative during the countdown)

    const fuel::Race& race() const { return races[index]; }
    void begin(DriveSession& d, WorldStreamer* streamer, fuel::MeshRenderer& renderer) {
        const fuel::Race& r = race();
        route.clear();
        for (int lap = 0; lap < (r.loops() ? r.laps : 1); ++lap) route.insert(route.end(), r.checkpoints.begin(), r.checkpoints.end());
        if (r.finish[0] != 0 || r.finish[2] != 0) route.push_back(r.finish);
        next = 0;
        clock = -3.0;
        state = Countdown;
        // Face the first target: the scripts' start rotation is only roughly a heading.
        float yaw = r.startYaw;
        if (!route.empty()) {
            const float dx = route[0][0] - r.start[0], dz = route[0][2] - r.start[2];
            if (std::hypot(dx, dz) > 5 && std::hypot(dx, dz) < 2000) yaw = std::atan2(dx, dz);
        }
        d.teleport(r.start[0], r.start[2], yaw);
        if (streamer) streamer->update(d.position(), renderer, d.physics(), true);
    }
    // Advances the race; holds the vehicle on the line during the countdown.
    void update(float dt, simd_float3 at, fuel::DriveInput& in) {
        if (state == Finished) return;
        clock += dt;
        if (state == Countdown) {
            in = {0, 0, 0, true};
            if (clock >= 0) state = Running;
            return;
        }
        const auto& t = route[next];
        if (std::hypot(at.x - t[0], at.z - t[2]) < kRadius && std::fabs(at.y - t[1]) < 60 && ++next >= route.size()) state = Finished;
        if (race().timeLimit > 0 && clock > race().timeLimit) state = Finished;
    }
    // Test driver: full throttle, steering at the next target.
    // When stuck against something it backs off with the opposite lock.
    float stuckTime = 0, reverseTime = 0;
    fuel::DriveInput autopilot(simd_float3 at, float heading, float speedKmh) {
        if (state != Running) return {};
        const auto& t = route[next];
        const float want = std::atan2(t[0] - at.x, t[2] - at.z);
        const float err = std::remainder(want - heading, 2.0f * float(M_PI));
        const float steer = std::clamp(-err * 2.0f, -1.0f, 1.0f);  // positive steer turns right (decreasing heading)
        stuckTime = std::fabs(speedKmh) < 3 ? stuckTime + 1.0f / 60 : 0;
        if (stuckTime > 2) reverseTime = 1.5f, stuckTime = 0;
        if (reverseTime > 0) {
            reverseTime -= 1.0f / 60;
            return {0, 1, -steer, false};
        }
        return {std::fabs(err) > 1.2f ? 0.4f : 1.0f, 0, steer, false};
    }
    std::vector<simd_float4> markers() const {
        std::vector<simd_float4> m;
        for (size_t i = next; i < route.size() && i < next + 2; ++i)
            m.push_back(simd_make_float4(route[i][0], route[i][1], route[i][2], i + 1 == route.size() ? 2.0f : float(i - next)));
        return m;
    }
    std::string hud(simd_float3 at) const {
        char buf[256];
        const double t = std::max(0.0, clock);
        const int min = int(t / 60);
        const double sec = t - min * 60;
        const fuel::Race& r = race();
        if (state == Countdown) {
            std::snprintf(buf, sizeof buf, "%s   starts in %d", r.name.c_str(), int(std::ceil(-clock)));
        } else if (state == Finished) {
            const bool timedOut = next < route.size();
            std::snprintf(buf, sizeof buf, "%s   %s %d:%04.1f   Enter: retry   N: next race", r.name.c_str(), timedOut ? "OUT OF TIME" : "FINISHED",
                          min, sec);
        } else {
            const auto& tgt = route[next];
            const size_t perLap = r.loops() && !r.checkpoints.empty() ? r.checkpoints.size() : route.size();
            std::string lap = r.loops() ? "   lap " + std::to_string(std::min<size_t>(next / perLap + 1, size_t(r.laps))) + "/" + std::to_string(r.laps) : "";
            std::snprintf(buf, sizeof buf, "%s   checkpoint %zu/%zu%s   %d:%04.1f   next %.0f m", r.name.c_str(), next + 1, route.size(), lap.c_str(),
                          min, sec, std::hypot(at.x - tgt[0], at.z - tgt[2]));
        }
        return buf;
    }
};

std::vector<fuel::Race> loadRaces(const std::string& dataDir) {
    std::vector<fuel::Race> out;
    for (const char* file : {"miss_official.tsc", "missChallenge.tsc"}) {
        std::ifstream f(fs::path(dataDir) / "LocalizedComponent/GameTsc/Story" / file);
        std::stringstream text;
        text << f.rdbuf();
        for (fuel::Race& r : fuel::parseMissions(text.str()))
            if ((r.start[0] != 0 || r.start[2] != 0) && (!r.checkpoints.empty() || r.finish[0] != 0 || r.finish[2] != 0)) out.push_back(r);
    }
    return out;
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
    renderer.encodeShadows(cb);
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
    bool _sceneMode;          // showing a scene file or an assembled variant
    AssemblyMode* _assembly;  // --assemble, else nullptr
    DriveSession* _drive;     // --drive, else nullptr
    WorldStreamer* _streamer; // --drive: the static world
    Garage* _garage;          // --drive: vehicles to switch between
    RaceSession* _race;       // --drive: the current race
    std::unique_ptr<DriveSession> _switched;  // a vehicle chosen with Tab
    std::unique_ptr<fuel::DriveAudio> _audio; // --drive in a window: engine, tyre, wind, impact sounds
    NSTextField* _hud;        // --drive: speed, gear, race
    std::set<unsigned short> _keys;
    double _lastTime;
    float _steer;             // smoothed keyboard steering
    float _camYawOffset;      // mouse look around the chase camera
}

- (instancetype)initWithFrame:(NSRect)frame
                       device:(id<MTLDevice>)device
                     renderer:(fuel::MeshRenderer*)renderer
                       meshes:(std::vector<MeshEntry>*)meshes
                     fileName:(const std::string&)fileName
                        start:(size_t)start
                       camera:(fuel::OrbitCamera)cam
                         look:(const VehicleLook&)look
                        scene:(const SceneFile&)scene
                     assembly:(AssemblyMode*)assembly
                        drive:(DriveSession*)drive
                     streamer:(WorldStreamer*)streamer
                       garage:(Garage*)garage
                         race:(RaceSession*)race {
    if ((self = [super initWithFrame:frame device:device])) {
        _renderer = renderer;
        _meshes = meshes;
        _fileName = fileName;
        _queue = [device newCommandQueue];
        _cam = cam;
        _index = start;
        _look = look;
        _scene = scene;
        _assembly = assembly;
        _drive = drive;
        _streamer = streamer;
        _garage = garage;
        _race = race;
        if (drive && garage && !garage->entries.empty()) {
            _audio = std::make_unique<fuel::DriveAudio>();
            [self loadAudio];
        }
        if (drive) {
            _hud = [NSTextField labelWithString:@""];
            _hud.font = [NSFont monospacedDigitSystemFontOfSize:22 weight:NSFontWeightSemibold];
            _hud.textColor = NSColor.whiteColor;
            _hud.shadow = [NSShadow new];
            _hud.shadow.shadowBlurRadius = 3;
            _hud.shadow.shadowColor = [NSColor colorWithWhite:0 alpha:0.8];
            _hud.frame = NSMakeRect(18, 14, 900, 64);
            _hud.maximumNumberOfLines = 2;
            [self addSubview:_hud];
        }
        _lastTime = 0;
        _steer = 0;
        _camYawOffset = 0;
        _sceneMode = assembly != nullptr || !scene.parts.empty();
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
    if (_sceneMode) {
        std::string where;
        if (_assembly)
            where = _fileName + " — variant " + std::to_string(_assembly->index + 1) + "/" +
                    std::to_string(_assembly->assembler->variants().size()) + " LOD" + std::to_string(_assembly->lod) + " — ";
        self.window.title = [NSString stringWithFormat:@"FUEL viewer — %s%s — %zu parts — %u triangles%s",
                                                       where.c_str(), _scene.variant.c_str(), _scene.parts.size(),
                                                       _renderer->triangleCount(), _look.describe().c_str()];
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
    if (_sceneMode) {
        if (!_renderer->setScene(_scene.parts, &err) || !err.empty()) std::fprintf(stderr, "fuelview: %s: %s\n", _scene.variant.c_str(), err.c_str());
        [self updateTitle];
        return;
    }
    if (!_renderer->setMesh((*_meshes)[_index].id, &err)) std::fprintf(stderr, "fuelview: mesh %08x: %s\n", (*_meshes)[_index].id, err.c_str());
    [self updateTitle];
}

// Assembles another variant of the DPC and paints it with its own palette
// and liveries (keeping the chosen livery and colour numbers).
- (void)showVariant:(size_t)index {
    _assembly->index = index;
    _scene = _assembly->scene();
    _look = _assembly->look(_look);
    _renderer->setPaint(_look.paint());
    [self showMesh:0];
}

- (void)step:(bool)forward {
    if (_assembly) {
        const size_t n = _assembly->assembler->variants().size();
        [self showVariant:forward ? (_assembly->index + 1) % n : (_assembly->index + n - 1) % n];
    } else if (!_sceneMode) {
        const size_t n = _meshes->size();
        [self showMesh:forward ? (_index + 1) % n : (_index + n - 1) % n];
    }
}

// Drive controls: W/S or Up/Down throttle and brake (reverse), A/D or
// Left/Right steer, Space handbrake, R back on the wheels. A gamepad works too.
- (bool)driveKey:(unsigned short)code {
    switch (code) {
        case 13: case 1: case 0: case 2: case 49: case 123: case 124: case 125: case 126: return true;
        default: return false;
    }
}

- (void)keyUp:(NSEvent*)e { _keys.erase(e.keyCode); }

// Builds the sound set of the garage's current vehicle.
- (void)loadAudio {
    AssemblyMode& am = *_garage->archives[_garage->entries[_garage->current].first];
    am.index = _garage->entries[_garage->current].second;
    const std::string name = am.variant().label;
    const auto it = _garage->params.find(name);
    if (it != _garage->params.end()) _audio->setVehicle(am.assembler->dpc(), name, it->second);
}

// Swaps in the next drivable vehicle where the current one stands.
- (void)switchVehicle:(int)dir {
    const simd_float3 at = _drive->position();
    const float heading = _drive->heading();
    const size_t n = _garage->entries.size();
    for (size_t tries = 0; tries < n; ++tries) {
        _garage->current = (_garage->current + n + size_t(dir + int(n))) % n;
        std::unique_ptr<DriveSession> next = _garage->build(_garage->current, at.x, at.z, heading);
        if (!next) continue;
        AssemblyMode& am = *_garage->archives[_garage->entries[_garage->current].first];
        _switched = std::move(next);
        _drive = _switched.get();
        _scene.parts = _drive->parts();
        _scene.variant = am.variant().label;
        _look = am.look(_look);
        _renderer->setPaint(_look.paint());
        std::string err;
        if (!_renderer->setScene(_scene.parts, &err) && !err.empty()) std::fprintf(stderr, "fuelview: %s\n", err.c_str());
        if (_streamer) _streamer->update(_drive->position(), *_renderer, _drive->physics(), true);
        if (_audio) [self loadAudio];
        return;
    }
}

- (void)keyDown:(NSEvent*)e {
    if (_drive) {
        if ([self driveKey:e.keyCode]) {
            _keys.insert(e.keyCode);
            return;
        }
        if (e.keyCode == 46 && _audio) {  // M: mute
            _audio->toggleMute();
            return;
        }
        if (e.keyCode == 15) {  // R
            _drive->reset();
            return;
        }
        if (_race && (e.keyCode == 45 || e.keyCode == 36)) {  // N (Shift-N back): another race; Return: restart
            if (e.keyCode == 45) {
                const size_t n = _race->races.size();
                _race->index = (_race->index + ((e.modifierFlags & NSEventModifierFlagShift) ? n - 1 : 1)) % n;
            }
            _race->begin(*_drive, _streamer, *_renderer);
            return;
        }
        if (e.keyCode == 48 && _garage && !_garage->entries.empty()) {  // Tab / Shift-Tab: another vehicle
            [self switchVehicle:(e.modifierFlags & NSEventModifierFlagShift) ? -1 : 1];
            return;
        }
    }
    switch (e.keyCode) {
        case 123: [self step:false]; return;  // left
        case 124: [self step:true]; return;   // right
        case 126: _cam.distance = std::max(0.4f, _cam.distance / 1.15f); return;  // up
        case 125: _cam.distance = std::min(3000.0f, _cam.distance * 1.15f); return; // down
        case 53: [NSApp terminate:nil]; return;                   // esc
        default: break;
    }
    NSString* ch = e.charactersIgnoringModifiers.lowercaseString;
    if ([ch isEqualToString:@"c"]) _renderer->cullBackFaces = !_renderer->cullBackFaces;
    else if ([ch isEqualToString:@"w"]) _renderer->wireframe = !_renderer->wireframe;
    else if ([ch isEqualToString:@"r"]) _cam = fuel::OrbitCamera{};
    else if ([ch isEqualToString:@"d"]) _renderer->dirt = _renderer->dirt >= 1.0f ? 0.0f : _renderer->dirt + 0.5f;
    else if ([ch isEqualToString:@"g"]) { _renderer->showGround = !_renderer->showGround; _renderer->showTerrain = !_renderer->showTerrain; }
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
    if (_drive) _camYawOffset -= float(e.deltaX) * 0.008f;
    else _cam.yaw -= float(e.deltaX) * 0.008f;
    _cam.pitch = std::clamp(_cam.pitch + float(e.deltaY) * 0.008f, -1.45f, 1.45f);
}

- (void)scrollWheel:(NSEvent*)e {
    _cam.distance = std::clamp(_cam.distance * std::exp(-float(e.scrollingDeltaY) * (e.hasPreciseScrollingDeltas ? 0.01f : 0.1f)), 0.4f, 3000.0f);
}

- (void)magnifyWithEvent:(NSEvent*)e {
    _cam.distance = std::clamp(_cam.distance / (1.0f + float(e.magnification)), 0.4f, 3000.0f);
}

- (void)mtkView:(MTKView*)view drawableSizeWillChange:(CGSize)size {}

- (void)driveStep {
    const double now = CACurrentMediaTime();
    const float dt = _lastTime > 0 ? float(std::min(now - _lastTime, 0.1)) : 0.0f;
    _lastTime = now;
    auto down = [&](unsigned short a, unsigned short b) { return _keys.count(a) || _keys.count(b); };
    fuel::DriveInput in;
    in.throttle = down(13, 126) ? 1.0f : 0.0f;
    in.brake = down(1, 125) ? 1.0f : 0.0f;
    const float target = (down(2, 124) ? 1.0f : 0.0f) - (down(0, 123) ? 1.0f : 0.0f);
    _steer += std::clamp(target - _steer, -4.0f * dt, 4.0f * dt);
    in.steer = _steer;
    in.handbrake = _keys.count(49) > 0;
    if (GCExtendedGamepad* pad = GCController.controllers.firstObject.extendedGamepad) {
        const float stick = pad.leftThumbstick.xAxis.value;
        if (std::fabs(stick) > 0.1f) in.steer = stick;
        in.throttle = std::max(in.throttle, pad.rightTrigger.value);
        in.brake = std::max(in.brake, pad.leftTrigger.value);
        in.handbrake = in.handbrake || pad.buttonX.pressed;
        if (pad.buttonY.pressed) _drive->reset();
    }
    if (_race) _race->update(dt, _drive->position(), in);
    _drive->update(dt, in);
    if (_race) _renderer->setMarkers(_race->markers());
    if (_streamer) _streamer->update(_drive->position(), *_renderer, _drive->physics());
    _renderer->setInstanceWorlds(_drive->partWorlds());
    _renderer->setFocus(_drive->position());
    // Chase camera: swing behind the vehicle, mouse look decays back.
    float targetYaw = _drive->heading() + float(M_PI) + _camYawOffset;
    float diff = std::remainder(targetYaw - _cam.yaw, 2.0f * float(M_PI));
    _cam.yaw += diff * std::min(1.0f, dt * 3.0f);
    _camYawOffset *= std::max(0.0f, 1.0f - dt * 0.5f);
    const fuel::VehicleState st = _drive->state();
    if (_audio) _audio->update(dt, st, in.throttle);
    self.window.title = [NSString stringWithFormat:@"FUEL — %s", _scene.variant.c_str()];
    const std::string raceLine = _race ? _race->hud(_drive->position()) : _scene.variant;
    _hud.stringValue = [NSString stringWithFormat:@"%s\n%3.0f km/h   gear %d   %5.0f rpm   %s", raceLine.c_str(), std::fabs(st.speedKmh),
                                                  st.gear, st.rpm, _race ? _scene.variant.c_str() : ""];
}

- (void)drawInMTKView:(MTKView*)view {
    if (_drive) [self driveStep];
    id<MTLCommandBuffer> cb = [_queue commandBuffer];
    _renderer->encodeShadows(cb);
    MTLRenderPassDescriptor* rp = view.currentRenderPassDescriptor;
    if (rp == nil) {
        [cb commit];
        return;
    }
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
        // Browsing meshes needs the list of drawable meshes; an assembled vehicle does not.
        std::vector<MeshEntry> meshes;
        size_t start = 0;
        if (opt.assemble.empty() && opt.scene.empty() && !opt.world) {
            meshes = listMeshes(subject);
            if (meshes.empty()) die("no drawable meshes in " + subject.path());
            if (opt.haveMeshId) {
                auto it = std::find_if(meshes.begin(), meshes.end(), [&](const MeshEntry& m) { return m.id == opt.meshId; });
                if (it == meshes.end()) die("mesh not found or not drawable");
                start = size_t(it - meshes.begin());
            }
        }

        id<MTLDevice> device = MTLCreateSystemDefaultDevice();
        if (device == nil) die("no Metal device");
        if (!device.supportsBCTextureCompression) die("this GPU cannot sample DXT (BC) textures");
        fuel::MeshRenderer renderer(device, kSamples, library);
        renderer.cullBackFaces = opt.cull;
        renderer.dirt = std::clamp(opt.dirt, 0.0f, 1.0f);
        std::unique_ptr<fuel::Heightfield> terrain;
        if (opt.terrain) {
            const fs::path world = fs::path(opt.dataDir) / "CommonComponent/WORLD/USA1.DPC";
            try {
                fuel::DpcFile usa1(world.string());
                terrain = std::make_unique<fuel::Heightfield>(fuel::loadWorldHeightfield(usa1));
                renderer.setTerrain(*terrain);
                renderer.setGround(fuel::buildGroundMaps(usa1, *terrain, 2334), usa1);
                if (opt.haveAt && !opt.world && !opt.drive)
                    renderer.placement = matrix_from_translation(opt.atX, terrain->heightAt(opt.atX, opt.atZ), opt.atZ);
            } catch (const std::exception& e) {
                die(std::string("terrain: ") + e.what());
            }
        }

        // The rider (PILOT.DPC) lives in the library so its materials and textures resolve.
        std::unique_ptr<RiderView> riders;
        if (!opt.assemble.empty() && !opt.noRider) {
            try {
                library.add(std::make_unique<fuel::DpcFile>((fs::path(opt.dataDir) / "CommonComponent/DATAS/PILOT.DPC").string()));
                std::ifstream f(fs::path(opt.dataDir) / "LocalizedComponent/GameTsc/Story/VehiclesDef.tsc");
                std::stringstream text;
                text << f.rdbuf();
                riders = std::make_unique<RiderView>(*library.files().back(), text.str(), renderer);
            } catch (const std::exception& e) {
                std::fprintf(stderr, "fuelview: no rider: %s\n", e.what());
            }
        }

        VehicleLook prefs;
        prefs.livery = std::clamp(opt.livery, 0, 9);
        prefs.color = std::max(0, opt.color);
        if (opt.havePaint) {
            prefs.custom = true;
            prefs.customColor = simd_make_float3(opt.paint[0], opt.paint[1], opt.paint[2]) / 255.0f;
        }

        SceneFile scene;
        VehicleLook look;
        std::unique_ptr<AssemblyMode> assembly;
        std::unique_ptr<DriveSession> drive;
        std::unique_ptr<WorldStreamer> streamer;
        std::unique_ptr<fuel::Vegetation> vegetation;
        std::unique_ptr<fuel::RoadNetwork> roads;
        std::unique_ptr<Garage> garage;
        std::unique_ptr<RaceSession> race;
        if (opt.drive && opt.assemble.empty()) die("--drive needs --assemble VARIANT");
        if (!opt.assemble.empty()) {
            assembly = std::make_unique<AssemblyMode>();
            try {
                assembly->assembler = std::make_unique<fuel::VehicleAssembler>(subject);
            } catch (const std::exception& e) {
                die(e.what());
            }
            const auto& variants = assembly->assembler->variants();
            const fuel::VehicleVariant* v = assembly->assembler->find(opt.assemble);
            if (v == nullptr) {
                std::string list;
                for (const fuel::VehicleVariant& x : variants) list += " " + x.label;
                die("no variant '" + opt.assemble + "' in " + subject.path() + (variants.empty() ? std::string(" (it has none)") : "; it has:" + list));
            }
            assembly->index = size_t(v - variants.data());
            assembly->lod = opt.lod;
            assembly->palettes = loadPalettes(opt.dataDir);
            assembly->riders = riders.get();
            scene = assembly->scene();
            look = assembly->look(prefs);
            renderer.setPaint(look.paint());
            if (opt.drive) {
                std::ifstream f(fs::path(opt.dataDir) / "CommonComponent/Param/VehiclesParam.tsc");
                std::stringstream text;
                text << f.rdbuf();
                const auto params = fuel::parseVehiclesParam(text.str());
                const std::string name = assembly->variant().label;
                const auto it = params.find(name);
                if (it == params.end()) die("no driving parameters for " + name + " in Param/VehiclesParam.tsc");
                try {
                    const fuel::AssembledVehicle av = assembly->assembler->assemble(assembly->variant(), assembly->lod);
                    const fuel::VehicleSetup setup = fuel::makeVehicleSetup(subject, name, it->second, assembledHubs(av));
                    drive = std::make_unique<DriveSession>(av, setup, *terrain, opt.haveAt ? opt.atX : -51358.0, opt.haveAt ? opt.atZ : -30043.0,
                                                           0.0f, riders ? riders->parts(name) : std::vector<uint32_t>{});
                } catch (const std::exception& e) {
                    die(name + ": " + e.what());
                }
                scene.parts = drive->parts();
                // The static world: every sub-world's placements (geometry lives in USA1.DPC).
                std::vector<const fuel::DpcFile*> subs, lookup;
                const fs::path worldDir = fs::path(opt.dataDir) / "CommonComponent/WORLD";
                std::vector<fs::path> files;
                for (const auto& e : fs::directory_iterator(worldDir))
                    if (e.path().filename().string().rfind("USA1_", 0) == 0 && e.path().extension() == ".DPC") files.push_back(e.path());
                std::sort(files.begin(), files.end());
                files.push_back(worldDir / "USA1.DPC");
                for (const fs::path& f : files) {
                    try {
                        auto dpc = std::make_unique<fuel::DpcFile>(f.string());
                        if (!dpc->isCurrentVersion()) continue;  // the game never loads these
                        if (f.filename() != "USA1.DPC") subs.push_back(dpc.get());
                        library.add(std::move(dpc));
                    } catch (const std::exception& e) {
                        std::fprintf(stderr, "fuelview: %s\n", e.what());
                    }
                }
                for (const auto& f : library.files()) lookup.push_back(f.get());
                streamer = std::make_unique<WorldStreamer>(subs, lookup);
                for (const auto& f : library.files())
                    if (fs::path(f->path()).filename() == "USA1.DPC") {
                        try {
                            vegetation = std::make_unique<fuel::Vegetation>(fuel::loadVegetation(*f));
                            streamer->setVegetation(vegetation.get(), terrain.get());
                            roads = std::make_unique<fuel::RoadNetwork>(fuel::loadRoads(*f));
                            streamer->setRoads(roads.get());
                            std::printf("roads: %zu drawable lines\n", roads->lines.size());
                            std::printf("vegetation: %zu tree and %zu bush species\n", vegetation->trees.size(), vegetation->bushes.size());
                        } catch (const std::exception& e) {
                            std::fprintf(stderr, "fuelview: vegetation: %s\n", e.what());
                        }
                    }
                // Every vehicle archive's variants that have driving parameters.
                garage = std::make_unique<Garage>();
                garage->params = params;
                garage->terrain = terrain.get();
                garage->riders = riders.get();
                for (const char* name : {"QUAD", "BIKE", "BUGGY", "MUSCLE", "SUV", "TRUCK", "VEH", "BONUS1"}) {
                    const fs::path path = fs::path(opt.dataDir) / "CommonComponent/DATAS" / (std::string(name) + ".DPC");
                    const fuel::DpcFile* dpc = nullptr;
                    for (const auto& f : library.files())
                        if (fs::equivalent(f->path(), path)) dpc = f.get();
                    if (dpc == nullptr) {
                        try {
                            auto f = std::make_unique<fuel::DpcFile>(path.string());
                            dpc = f.get();
                            library.add(std::move(f));
                        } catch (const std::exception&) {
                            continue;
                        }
                    }
                    auto am = std::make_unique<AssemblyMode>();
                    am->assembler = std::make_unique<fuel::VehicleAssembler>(*dpc);
                    am->palettes = assembly->palettes;
                    am->riders = riders.get();
                    am->lod = 1;
                    for (size_t v = 0; v < am->assembler->variants().size(); ++v) {
                        const std::string& label = am->assembler->variants()[v].label;
                        if (params.count(label)) {
                            if (dpc == &subject && label == assembly->variant().label) garage->current = garage->entries.size();
                            garage->entries.push_back({garage->archives.size(), v});
                        }
                    }
                    garage->archives.push_back(std::move(am));
                }
                std::printf("garage: %zu drivable variants (Tab to switch)\n", garage->entries.size());
                if (!opt.race.empty()) {
                    race = std::make_unique<RaceSession>();
                    race->races = loadRaces(opt.dataDir);
                    if (race->races.empty()) die("no races found in the mission scripts");
                    auto it = std::find_if(race->races.begin(), race->races.end(), [&](const fuel::Race& r) { return r.name == opt.race; });
                    if (it == race->races.end()) die("no race named " + opt.race);
                    race->index = size_t(it - race->races.begin());
                    race->begin(*drive, streamer.get(), renderer);
                    std::printf("race %s: %zu targets; %zu races (N for the next)\n", race->race().name.c_str(), race->route.size(), race->races.size());
                }
                if (opt.audioCheck) {  // build sound sets for a few vehicles and print their layers
                    std::vector<std::pair<const fuel::DpcFile*, std::string>> list;
                    for (const char* want : {"QUAD_ROAD_2", "BUGGY_ROAD_2", "TRUCK_OFF_1", "BIKE_ROAD_3"})
                        for (const auto& [ai, vi] : garage->entries) {
                            AssemblyMode& am = *garage->archives[ai];
                            if (am.assembler->variants()[vi].label == want) list.push_back({&am.assembler->dpc(), want});
                        }
                    const int bad = fuel::audioCheck(list, params);
                    std::printf("audio check: %zu vehicles, %d problems\n", list.size(), bad);
                    return bad ? 1 : 0;
                }
                if (std::getenv("FUEL_GARAGE_CHECK")) {  // build and drive every vehicle briefly
                    int ok = 0;
                    for (size_t i = 0; i < garage->entries.size(); ++i) {
                        auto d = garage->build(i, -1000, -30000, 0);
                        if (!d) continue;
                        for (int k = 0; k < 180; ++k) d->update(1.0f / 60, {1.0f, 0, 0, false});
                        const fuel::VehicleState st = d->state();
                        const bool good = st.wheelsOnGround >= 2 && st.speedKmh > 15;
                        ok += good;
                        std::printf("  %-18s %5.1f km/h after 3 s, %d wheels down%s\n", garage->archives[garage->entries[i].first]->variant().label.c_str(),
                                    st.speedKmh, st.wheelsOnGround, good ? "" : "  <-- check");
                    }
                    std::printf("garage check: %d of %zu drive\n", ok, garage->entries.size());
                    return 0;
                }
                std::printf("world: %zu objects in %zu sub-worlds\n", streamer->placementCount(), subs.size());
            }
        } else {
            if (!opt.scene.empty()) scene = loadScene(opt.scene);
            if (opt.world) {
                // Every mesh the first DPC places; the others help resolve references.
                std::vector<const fuel::DpcFile*> lookup;
                for (const auto& f : library.files()) lookup.push_back(f.get());
                for (const fuel::PlacedMesh& pm : fuel::collectPlacedMeshes(subject, lookup)) {
                    simd_float4 cols[4];
                    for (size_t r = 0; r < 4; ++r)
                        cols[r] = simd_make_float4(pm.world[r * 4], pm.world[r * 4 + 1], pm.world[r * 4 + 2], pm.world[r * 4 + 3]);
                    scene.parts.push_back({pm.meshId, simd_matrix(cols[0], cols[1], cols[2], cols[3])});
                }
                if (scene.parts.empty()) die("no placed meshes in " + subject.path());
                scene.variant = fs::path(subject.path()).filename().string();
            }
            const std::string vehicle = !opt.vehicle.empty() ? opt.vehicle : (opt.world ? std::string() : scene.variant);
            if (!vehicle.empty()) {
                // Unknown names appear as "?<crc>" in scene files.
                uint32_t crc = fuel::asoboCrc(vehicle);
                if (vehicle.size() == 9 && vehicle[0] == '?' && vehicle.find_first_not_of("0123456789abcdef", 1) == std::string::npos)
                    crc = uint32_t(std::stoul(vehicle.substr(1), nullptr, 16));
                look = makeLook(vehicle, crc, loadPalettes(opt.dataDir), subject, prefs);
                renderer.setPaint(look.paint());
            }
        }
        const bool sceneMode = assembly != nullptr || !scene.parts.empty();

        fuel::OrbitCamera cam;
        cam.yaw = opt.yawDeg * float(M_PI) / 180.0f;
        cam.pitch = opt.pitchDeg * float(M_PI) / 180.0f;
        if (opt.distance > 0) cam.distance = opt.distance;

        if (!opt.shot.empty()) {
            std::string err;
            if (sceneMode) {
                if (!renderer.setScene(scene.parts, &err)) die(scene.variant + ": " + err);
                if (!err.empty()) std::fprintf(stderr, "fuelview: %s\n", err.c_str());
                if (drive) {  // drive straight ahead for a while, then look from behind
                    if (streamer) streamer->update(drive->position(), renderer, drive->physics(), true);
                    std::vector<simd_float3> tried;
                    simd_float3 checkFocus = {0, 0, 0};
                    bool haveCheckFocus = false;
                    const simd_float3 checkFrom = drive->position();
                    const float runTime = opt.propCheck ? std::max(opt.driveTime, 9.0f) : opt.driveTime;
                    for (int trial = 0; trial < (opt.propCheck ? 8 : 1); ++trial) {  // --prop-check: try props until one is hit cleanly
                    if (streamer && opt.propCheck) {  // aim the vehicle at a prop from 45 m away and keep the throttle down
                        simd_float3 target;
                        if (!streamer->nearestProp(checkFrom, tried, target)) die("--prop-check: no movable props near the start (use --race or --at)");
                        tried.push_back(target);
                        const simd_float3 from = checkFrom;
                        const float yaw = std::atan2(target.x - from.x, target.z - from.z);
                        drive->teleport(target.x - 45 * std::sin(yaw), target.z - 45 * std::cos(yaw), yaw);
                        streamer->update(drive->position(), renderer, drive->physics(), true);
                        std::printf("prop check: aiming at a prop at (%.0f, %.0f) from 45 m\n", target.x, target.z);
                    }
                    float speedBefore = 0, speedMin = 1e9f, firstContactS = -1, speedLater = -1;
                    const size_t movedBase = streamer ? streamer->propStats().moved : 0;
                    for (int i = 0; i < int(runTime * 60); ++i) {
                        fuel::DriveInput in{1.0f, 0, 0, false};
                        if (race && !opt.propCheck) {
                            if (race->state == RaceSession::Finished) break;
                            in = race->autopilot(drive->position(), drive->heading(), drive->state().speedKmh);
                            if (std::getenv("FUEL_RACE_WARP") && race->state == RaceSession::Running && i % 60 == 0) {
                                // Logic test: hop to the next target every second.
                                const auto& t = race->route[race->next];
                                drive->teleport(t[0], t[2], drive->heading());
                            }
                            race->update(1.0f / 60, drive->position(), in);
                            if (std::getenv("FUEL_RACE_TRACE") && i % 600 == 0) {
                                const simd_float3 p = drive->position();
                                std::printf("  t=%5.0fs at (%.0f, %.0f, %.0f) %5.1f km/h  heading %4.0f  %s\n", i / 60.0, p.x, p.y, p.z,
                                            drive->state().speedKmh, drive->heading() * 180 / M_PI, race->hud(p).c_str());
                            }
                        }
                        drive->update(1.0f / 60, in);
                        if (streamer && (opt.propCheck || i % 30 == 0)) streamer->update(drive->position(), renderer, drive->physics());
                        if (streamer && opt.propCheck) {
                            const float v = drive->state().speedKmh;
                            if (i % 20 == 0 && std::getenv("FUEL_PROP_TRACE")) {
                                const simd_float3 q = drive->position();
                                std::printf("  t=%.2f (%.1f, %.1f, %.1f) %.1f km/h moved %zu\n", i / 60.0, q.x, q.y, q.z, v, streamer->propStats().moved);
                            }
                            if (firstContactS < 0) {
                                if (streamer->propStats().moved > movedBase) firstContactS = float(i) / 60.0f;
                                else speedBefore = v;
                            } else {
                                speedMin = std::min(speedMin, v);
                                if (speedLater < 0 && float(i) / 60.0f >= firstContactS + 1.0f) speedLater = v;
                                if (speedLater >= 0 && float(i) / 60.0f >= firstContactS + 1.5f) break;  // enough: show the aftermath
                            }
                        }
                    }
                    if (streamer && opt.propCheck) {
                        const auto ps = streamer->propStats();
                        const float speedAfter = drive->state().speedKmh;
                        std::printf("prop check: %zu props tracked, %zu simulated, %zu moved (%zu in this run); first contact at %.1f s\n", ps.tracked, ps.bodies, ps.moved, ps.moved - movedBase, firstContactS);
                        std::printf("prop check: speed before contact %.1f km/h, lowest after %.1f km/h, 1 s after %.1f km/h\n", speedBefore,
                                    firstContactS < 0 ? 0.0f : speedMin, speedLater);
                        const bool ok = ps.moved > movedBase && speedBefore > 15.0f && speedLater > 0.6f * speedBefore;
                        { const simd_float3 car = drive->position(); std::printf("prop check: furthest prop displaced %.1f m, now at (%.1f, %.1f, %.1f); vehicle at (%.1f, %.1f, %.1f); terrain under it %.1f\n", ps.maxDisplacement, ps.where.x, ps.where.y, ps.where.z, car.x, car.y, car.z, terrain ? terrain->smoothHeightAt(ps.where.x, ps.where.z) : 0.0f); }
                        if (ok && opt.shot != "-") checkFocus = ps.where + simd_make_float3(0, 1, 0), haveCheckFocus = true;
                        std::printf("prop check: %s (at end %.1f km/h)\n", ok ? "ok" : "FAILED (no props moved, or the vehicle lost its speed)", speedAfter);
                        if (ok || trial == 7) {
                            if (opt.shot == "-" && !ok) return 1;
                            break;
                        }
                    }
                    }
                    renderer.setInstanceWorlds(drive->partWorlds());
                    if (race) {
                        renderer.setMarkers(race->markers());
                        std::printf("%s\n", race->hud(drive->position()).c_str());
                    }
                    renderer.setFocus(haveCheckFocus ? checkFocus : drive->position());
                    cam.yaw += drive->heading() + float(M_PI);
                    const fuel::VehicleState st = drive->state();
                    std::printf("after %.1f s: %.1f km/h, gear %d, %d wheels on the ground\n", opt.driveTime, st.speedKmh, st.gear, st.wheelsOnGround);
                }
                if (opt.shot == "-") return 0;
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
                                                   scene:scene
                                                assembly:assembly.get()
                                                   drive:drive.get()
                                                streamer:streamer.get()
                                                  garage:garage.get()
                                                    race:race.get()];
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
