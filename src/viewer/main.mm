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
#import "recorder.h"
#import "game_settings.h"
#include "ui/game_menu.h"
#include "ui/menu_check.hpp"
#include "rider_view.h"
#include "eventcheck.h"
#include "hubcheck.h"
#include "capture.h"
#include "core/terrain_height.hpp"
#include "core/hubs.hpp"
#import <GameController/GameController.h>
#include "audio/drive_audio.h"
#include "core/events.hpp"
#include "core/race_rules.hpp"
#include "core/races.hpp"
#include "physics/race_ai.hpp"
#include "physics/camcheck.hpp"
#include "core/rider_anim.hpp"
#include "core/vehicle_params.hpp"
#include <fstream>
#include <set>
#include <strings.h>
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
#include <random>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace {

simd_float4x4 matrix_from_translation(float x, float y, float z) {
    simd_float4x4 m = matrix_identity_float4x4;
    m.columns[3] = simd_make_float4(x, y, z, 1);
    return m;
}

constexpr MTLPixelFormat kColorFormat = fuel::kOutputColorFormat;
constexpr NSUInteger kSamples = 4;  // the scene's MSAA

fuel::Recorder gRecorder;  // --record
std::unique_ptr<fuel::ui::GameOverlay> gOverlay;
fuel::ui::HudState gShotHud;  // --in-game --film: the HUD drawn on the film frames  // --in-game: pause/results menu and HUD
std::vector<fuel::Hub> gHubs;  // the 19 hubs (--hub, --hub-check, the T key)
const fuel::Heightfield* gTerrain = nullptr;  // for the T key

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
    int opponents = -1;    // --race: AI opponents on the grid (-1: the event's EMD_SetNbIA)
    bool aiCheck = false;  // --ai-check: race headless and report the opponents' progress
    float driveTime = 4.0f;
    bool haveAt = false;
    float atX = 0, atZ = 0, atYaw = 0;
    std::string hub;       // --hub NAME: start at that hub's camp (free roam)
    bool eventCheck = false;  // --event-check [--event-only ID|HUB]: run every event headless and report
    std::string eventOnly;
    bool hubCheck = false;  // --hub-check: spawn at every hub point and report
    std::string assemble;
    int lod = 1;
    int width = 1600, height = 1000;
    float yawDeg = 37.0f, pitchDeg = 17.0f;
    bool haveYaw = false;  // --yaw given: with --film, the chase camera looks from that angle (0 = behind)
    float distance = 0;
    bool cull = true;
    float dirt = 0.0f;
    std::string vehicle;
    int livery = 1;
    int color = 0;
    bool havePaint = false;
    int paint[3] = {60, 60, 60};
    bool noRider = false;
    float film = 0;      // --film S: with --shot, a chase-camera frame every S seconds of the drive
    std::string record;  // --record DIR: frame log and grabs of a window session
    bool game = false;      // --game: FUEL's menus (title, top menu, lists, options); sessions run in a new process
    bool inGame = false;    // --in-game: a session started from the menus (pause/results menu, HUD)
    std::string menuShot, menuShotOut, menuPage;  // --menu-shot PAGE OUT.png, --menu-page PAGE
    bool menuCheck = false; // --menu-check
    bool camCheck = false;  // --camcheck: FUEL's camera and rider functions against emulated test vectors
    int camView = 0;        // --cam-view N: the player's camera view for --film (5 near chase, 1 first person, 4 far chase)
    float dropAt = -1, dropHeight = 0;  // --drop T,H: lift the player H metres at T seconds (a jump landing for --film)
    bool settings = false;  // --settings: size, fullscreen, post effects, grass, volume (and data) from the settings file; --game implies it
    // Reference captures (capture.h): --capture-spot NAME renders one spot of the spot list to
    // NAME.png (or --shot FILE), --capture-all DIR every spot to DIR/NAME.png, --spots FILE picks
    // the list, --capture-find X,Z[,R] prints roads and floating objects near a point.
    std::string captureSpot, captureAll, spotsFile;
    std::vector<CaptureSpot> spots;
    bool captureFind = false;
    double findX = 0, findZ = 0, findRadius = 300;
    bool haveSize = false;
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
        else if (a == "--opponents") o.opponents = std::clamp(std::stoi(next()), 0, 11);
        else if (a == "--ai-check") o.aiCheck = true;
        else if (a == "--at") { const std::string v = next(); if (std::sscanf(v.c_str(), "%f,%f", &o.atX, &o.atZ) != 2) die("bad --at (use X,Z in metres)"); o.haveAt = true; o.terrain = true; }
        else if (a == "--hub") { o.hub = next(); o.terrain = true; }
        else if (a == "--event-check") { o.eventCheck = true; o.drive = true; o.terrain = true; }
        else if (a == "--event-only") o.eventOnly = next();
        else if (a == "--hub-check") { o.hubCheck = true; o.drive = true; o.terrain = true; }
        else if (a == "--assemble") o.assemble = next();
        else if (a == "--lod") { o.lod = std::stoi(next()); if (o.lod < 1) die("--lod starts at 1"); }
        else if (a == "--size") { const std::string s = next(); if (std::sscanf(s.c_str(), "%dx%d", &o.width, &o.height) != 2) die("bad --size"); o.haveSize = true; }
        else if (a == "--capture-spot") { o.captureSpot = next(); o.drive = true; o.terrain = true; }
        else if (a == "--capture-all") { o.captureAll = next(); o.drive = true; o.terrain = true; }
        else if (a == "--spots") o.spotsFile = next();
        else if (a == "--capture-find") {
            const std::string v = next();
            if (std::sscanf(v.c_str(), "%lf,%lf,%lf", &o.findX, &o.findZ, &o.findRadius) < 2) die("bad --capture-find (use X,Z or X,Z,RADIUS in metres)");
            o.captureFind = true, o.drive = true, o.terrain = true;
        }
        else if (a == "--yaw") o.yawDeg = std::stof(next()), o.haveYaw = true;
        else if (a == "--pitch") o.pitchDeg = std::stof(next());
        else if (a == "--distance") o.distance = std::stof(next());
        else if (a == "--no-cull") o.cull = false;
        else if (a == "--dirt") o.dirt = std::stof(next());
        else if (a == "--no-rider") o.noRider = true;
        else if (a == "--record") o.record = next();
        else if (a == "--settings") o.settings = true;
        else if (a == "--game") { o.game = true; o.settings = true; }
        else if (a == "--in-game") { o.inGame = true; o.settings = true; }
        else if (a == "--menu-shot") { o.menuShot = next(); o.menuShotOut = next(); }
        else if (a == "--menu-page") o.menuPage = next();
        else if (a == "--menu-check") o.menuCheck = true;
        else if (a == "--camcheck") o.camCheck = true;
        else if (a == "--cam-view") o.camView = std::stoi(next());
        else if (a == "--drop") { const std::string v = next(); if (std::sscanf(v.c_str(), "%f,%f", &o.dropAt, &o.dropHeight) != 2) die("bad --drop (use T,H)"); }
        else if (a == "--film") o.film = std::stof(next());
        else if (a == "--vehicle") o.vehicle = next();
        else if (a == "--livery") o.livery = std::stoi(next());
        else if (a == "--color") o.color = std::stoi(next());
        else if (a == "--paint") { const std::string v = next(); if (std::sscanf(v.c_str(), "%d,%d,%d", &o.paint[0], &o.paint[1], &o.paint[2]) != 3) die("bad --paint"); o.havePaint = true; }
        else if (a == "-h" || a == "--help") { std::puts("usage: fuelview [--data DIR] [--mesh ID | --assemble VARIANT [--lod N] | --scene FILE.json]\n"
                                                         "                [--vehicle NAME] [--livery N] [--color N | --paint R,G,B]\n"
                                                         "                [--shot FILE.png [--size WxH] [--yaw DEG] [--pitch DEG] [--distance RADII] [--no-cull] [--dirt 0..1]] [DPC ...]\n"
                                                         "       fuelview [--data DIR] [--spots FILE.tsv] (--capture-spot NAME [--shot FILE.png] | --capture-all DIR) [--size WxH]\n"
                                                         "       fuelview [--data DIR] --capture-find X,Z[,RADIUS]"); std::exit(0); }
        else if (!a.empty() && a[0] == '-') die("unknown option " + a);
        else o.dpcs.push_back(a);
    }
    if (!o.assemble.empty() && (!o.scene.empty() || o.haveMeshId)) die("--assemble cannot be combined with --scene or --mesh");
    if (!o.assemble.empty() && !o.vehicle.empty()) die("with --assemble the paint follows the variant; drop --vehicle");
    if (o.eventCheck) {  // headless: every event, warped through
        if (o.shot.empty()) o.shot = "-";
        if (o.assemble.empty()) o.assemble = "QUAD_ROAD_2";
        o.opponents = 0;
    }
    if (o.hubCheck) {  // headless: spawn everywhere and report
        if (o.shot.empty()) o.shot = "-";
        if (o.assemble.empty()) o.assemble = "QUAD_ROAD_2";
        o.opponents = 0;
    }
    if (!o.captureSpot.empty() || !o.captureAll.empty() || o.captureFind) {  // headless reference captures
        if (o.assemble.empty()) o.assemble = "QUAD_ROAD_2";
        o.opponents = 0;
        if (!o.haveSize) o.width = 1920, o.height = 1080;  // the size the checklist asks for from the real game
        if (o.captureFind) {
            o.atX = float(o.findX), o.atZ = float(o.findZ), o.haveAt = true;
        } else {
            if (o.spotsFile.empty()) o.spotsFile = defaultCaptureSpotsPath();
            try {
                o.spots = loadCaptureSpots(o.spotsFile);
            } catch (const std::exception& e) {
                die(e.what());
            }
            if (o.spots.empty()) die("no spots in " + o.spotsFile);
            const CaptureSpot* first = &o.spots.front();
            if (!o.captureSpot.empty()) {
                first = findCaptureSpot(o.spots, o.captureSpot);
                if (first == nullptr) {
                    std::string names;
                    for (const CaptureSpot& sp : o.spots) names += " " + sp.name;
                    die("no spot '" + o.captureSpot + "' in " + o.spotsFile + "; it has:" + names);
                }
                if (o.shot.empty()) o.shot = first->name + ".png";
            }
            o.atX = float(first->x), o.atZ = float(first->z), o.atYaw = first->headingDeg * float(M_PI) / 180.0f, o.haveAt = true;
        }
        if (o.shot.empty()) o.shot = "-";
    }
    if (o.propCheck && o.shot.empty()) o.shot = "-";  // headless: drive and report, no picture
    if (o.propCheck) o.opponents = 0;                  // the player alone among the props
    if (o.aiCheck) {
        if (o.race.empty()) die("--ai-check needs --race NAME");
        if (o.shot.empty()) o.shot = "-";
        if (o.driveTime <= 4.0f) o.driveTime = 120.0f;
    }
    if (o.dataDir.empty() && o.settings) o.dataDir = fuel::loadSettings().dataDir;
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
// The ground maps and roads behind the physics' per-wheel surface query (drive mode).
static fuel::GroundMaps gGround;
static fuel::SurfaceMap gSurfaces;

// Per-frame physics cost of the headless runs (--ai-check prints it).
static struct PhysStats {
    std::vector<double> frame, vehicle, cpu;
    int vehicles = 0;
    void add(double ms, double vehicleMs, double cpuMs, int n) {
        frame.push_back(ms);
        vehicle.push_back(vehicleMs);
        cpu.push_back(cpuMs);
        vehicles = n;
    }
    void print() const {
        if (frame.empty()) return;
        auto stats = [](std::vector<double> v, double& mean, double& p99, double& worst) {
            mean = 0;
            for (double x : v) mean += x;
            mean /= double(v.size());
            std::sort(v.begin(), v.end());
            p99 = v[std::min(v.size() - 1, size_t(double(v.size()) * 0.99))];
            worst = v.back();
        };
        double m, p, w, vm, vp, vw, cm, cp, cw;
        stats(frame, m, p, w);
        stats(vehicle, vm, vp, vw);
        stats(cpu, cm, cp, cw);
        std::printf("physics cost, %d vehicles, %zu frames: mean %.2f ms, p99 %.2f ms, worst %.2f ms per frame "
                    "(FUEL vehicle model: mean %.2f, p99 %.2f, worst %.2f ms wall; CPU mean %.2f, p99 %.2f, worst %.2f ms)\n",
                    vehicles, frame.size(), m, p, w, vm, vp, vw, cm, cp, cw);
    }
} gPhysStats;

struct Garage {
    std::vector<std::unique_ptr<AssemblyMode>> archives;  // one per vehicle DPC
    std::vector<std::pair<size_t, size_t>> entries;       // (archive, variant)
    size_t current = 0;
    std::map<std::string, std::vector<float>> params;     // Param/VehiclesParam.tsc
    const fuel::Heightfield* terrain = nullptr;
    RiderView* riders = nullptr;

    const std::string& label(size_t i) const { return archives[entries[i].first]->assembler->variants()[entries[i].second].label; }
    // Assembles entry `i` with its physics setup and rider; false (with a message) if it cannot drive.
    bool load(size_t i, fuel::AssembledVehicle& av, fuel::VehicleSetup& setup, std::vector<uint32_t>& rider) const {
        AssemblyMode& am = *archives[entries[i].first];
        am.index = entries[i].second;
        const std::string name = am.variant().label;
        try {
            const auto it = params.find(name);
            if (it == params.end()) throw std::runtime_error("no driving parameters");
            av = am.assembler->assemble(am.variant(), am.lod);
            setup = fuel::makeVehicleSetup(am.assembler->dpc(), name, it->second, assembledHubs(av));
            rider = riders ? riders->animated(name, av, am.assembler->dpc(), setup.params) : std::vector<uint32_t>{};
            return true;
        } catch (const std::exception& e) {
            std::fprintf(stderr, "fuelview: %s: %s\n", name.c_str(), e.what());
            return false;
        }
    }
    // Builds entry `i` at (x, z) facing `yaw`; nullptr (with a message) if it cannot drive.
    std::unique_ptr<DriveSession> build(size_t i, double x, double z, float yaw) const {
        fuel::AssembledVehicle av;
        fuel::VehicleSetup setup;
        std::vector<uint32_t> rider;
        if (!load(i, av, setup, rider)) return nullptr;
        return std::make_unique<DriveSession>(av, setup, *terrain, x, z, yaw, rider);
    }
    // Race opponents for entry `player`: entries of its class (the label up to the first '_',
    // e.g. BUGGY), those of its category (OFF, ROAD, ALL) first, shuffled by `rng`. Repeats
    // when the class has fewer than `count`.
    std::vector<size_t> rivals(size_t player, int count, std::mt19937& rng) const {
        const std::string& me = label(player);
        auto field = [](const std::string& s, int k) {
            size_t a = 0;
            for (int j = 0; j < k && a != std::string::npos; ++j) a = s.find('_', a) == std::string::npos ? std::string::npos : s.find('_', a) + 1;
            return a == std::string::npos ? std::string() : s.substr(a, s.find('_', a) - a);
        };
        std::vector<size_t> same, kin;
        for (size_t i = 0; i < entries.size(); ++i) {
            if (i == player || field(label(i), 0) != field(me, 0)) continue;
            (field(label(i), 1) == field(me, 1) ? same : kin).push_back(i);
        }
        std::shuffle(same.begin(), same.end(), rng);
        std::shuffle(kin.begin(), kin.end(), rng);
        std::vector<size_t> pool = same;
        pool.insert(pool.end(), kin.begin(), kin.end());
        if (pool.empty()) pool.push_back(player);
        std::vector<size_t> out;
        for (int k = 0; k < count; ++k) out.push_back(pool[size_t(k) % pool.size()]);
        return out;
    }
};

// --race: FUEL's events (start line, ordered checkpoints, finish, laps; the rules of each game
// mode come from fuel::RaceRules), with AI opponents (vehicles 1.. of the drive session) on the
// grid behind the player. Hunted racers of "catch AI" challenges are opponents that follow the
// event's path; the helicopter of chopper races has no vehicle (a beacon marks it).
struct RaceSession {
    using State = fuel::RaceRules::State;
    static constexpr State Countdown = fuel::RaceRules::Countdown, Running = fuel::RaceRules::Running, Finished = fuel::RaceRules::Finished;
    std::vector<fuel::Event> races;           // playable events (and uncatalogued EMD races)
    size_t index = 0;
    fuel::RaceRules rules;
    State state = Countdown;
    std::vector<std::array<float, 3>> route;  // the player's gates (per lap), then the finish
    fuel::RaceLine line;                      // start, then the route (what the AI follows)
    fuel::RaceLine aiLine;                    // hunted racers: their grid, then the event's path
    const fuel::Heightfield* terrain = nullptr;  // puts gates far off the ground onto it
    double clock = 0;  // seconds since the start (negative during the countdown)

    // Everyone in the race: [0] is the player, the others match the session's vehicles 1...
    // Gates passed and finish times live in rules.racers.
    struct Racer {
        std::string name;        // variant label
        size_t entry = 0;        // in the garage
        fuel::RaceDriver ai;     // opponents (and the player's autopilot)
    };
    std::vector<Racer> racers = std::vector<Racer>(1);
    bool reported = false;          // finishing order printed

    RaceSession() = default;
    RaceSession(const RaceSession&) = delete;

    const fuel::Event& event() const { return races[index]; }
    const fuel::Race& race() const { return races[index].race; }
    const fuel::RaceLine& lineFor(size_t i) const { return rules.usesRoute(i) ? line : aiLine; }
    std::vector<std::array<float, 3>> where(const DriveSession& d) const {
        std::vector<std::array<float, 3>> p;
        for (size_t i = 0; i < racers.size() && int(i) < d.vehicleCount(); ++i) {
            const simd_float3 v = d.position(int(i));
            p.push_back({v.x, v.y, v.z});
        }
        return p;
    }
    void begin(DriveSession& d, WorldStreamer* streamer, fuel::MeshRenderer& renderer) {
        const fuel::Race& r = race();
        if (terrain) rules.ground = [t = terrain](float x, float z) { return t->smoothHeightAt(x, z); };
        rules.begin(r, racers.size(), event().times[0]);
        route = rules.route;
        line.build(r.start, route);
        aiLine.build(rules.aiPath.empty() ? r.start : rules.aiPath[0], rules.aiTargets);
        clock = rules.clock;
        state = rules.state;
        reported = false;
        for (Racer& racer : racers) racer.ai.restart();
        // Face the first target: the scripts' start rotation is only roughly a heading.
        float yaw = r.startYaw;
        const std::vector<std::array<float, 3>>& first = route.empty() ? rules.aiTargets : route;
        if (!first.empty()) {
            const float dx = first[0][0] - r.start[0], dz = first[0][2] - r.start[2];
            if (std::hypot(dx, dz) > 5 && std::hypot(dx, dz) < 2000) yaw = std::atan2(dx, dz);
        }
        // The grid: the player on the start, opponents beside and in rows behind (three abreast),
        // spaced for the largest vehicle; EMD_ForceStartPosition spots when the event has them.
        float width = 1, length = 2;
        for (int i = 0; i < d.vehicleCount(); ++i)
            for (const auto& w : d.setup(i).wheels) {
                width = std::max(width, 2 * std::fabs(w.hub[0]) + w.width);
                length = std::max(length, 2 * (std::fabs(w.hub[2]) + w.radius));
            }
        const float across = width + 2.5f, back = length + 4.0f;
        const float fx = std::sin(yaw), fz = std::cos(yaw);
        for (int i = 0; i < d.vehicleCount(); ++i) {
            if (i > 0 && size_t(i - 1) < r.aiGrid.size()) {
                d.teleport(r.aiGrid[size_t(i - 1)][0], r.aiGrid[size_t(i - 1)][1], yaw, i);
                continue;
            }
            // 0 = player; 1, 2 beside it; then rows of three (centre, right, left) behind.
            const float lanes[3] = {0, 1, -1};
            const float side = (i == 0 ? 0.0f : i <= 2 ? lanes[i] : lanes[(i - 3) % 3]) * across;
            const float behind = i <= 2 ? 0.0f : float(1 + (i - 3) / 3) * back;
            d.teleport(r.start[0] - fx * behind + fz * side, r.start[2] - fz * behind - fx * side, yaw, i);
        }
        if (streamer) streamer->update(d.position(), renderer, d.physics(), true);
    }
    std::vector<int> positions(const DriveSession& d) const { return rules.positions(where(d)); }
    // Advances the race for every racer; drives the opponents (and holds everyone on the
    // line during the countdown). `in` is the player's input.
    void update(float dt, DriveSession& d, fuel::DriveInput& in) {
        rules.update(dt, where(d));
        clock = rules.clock;
        state = rules.state;
        if (state == Countdown) {
            in = {0, 0, 0, true};
            for (int i = 1; i < d.vehicleCount(); ++i) d.setInput(i, {0, 0, 0, true});
            return;
        }
        // Opponents: the AI driver, rubber-banded on the player (up to 12% slower when far
        // ahead, 8% faster when far behind; FUEL's EMD_SetAIElasticParam gives 200 m).
        const float player = line.progress(std::min(rules.racers[0].next, route.size()), d.position(0).x, d.position(0).z);
        for (int i = 1; i < d.vehicleCount() && size_t(i) < racers.size(); ++i) {
            Racer& racer = racers[size_t(i)];
            const fuel::RaceRules::Racer& rr = rules.racers[size_t(i)];
            if (rr.finished >= 0 || rr.out || rr.next >= rules.lineFor(size_t(i)).size()) {
                d.setInput(i, {0, 0, 0, true});
                continue;
            }
            const simd_float3 at = d.position(i);
            const float ahead = rules.usesRoute(size_t(i)) ? line.progress(rr.next, at.x, at.z) - player : 0.0f;
            const float rubber = ahead > 200 ? 1.0f - 0.12f * std::min(1.0f, (ahead - 200) / 400) :
                                 ahead < -200 ? 1.0f + 0.08f * std::min(1.0f, (-ahead - 200) / 400) : 1.0f;
            d.setInput(i, drive(racer, d, i, dt, rubber));
            // FUEL_AI_TRACE=N: follow opponent N twice a second (and print every reset).
            static const char* trace = std::getenv("FUEL_AI_TRACE");
            if (trace && std::atoi(trace) == i && int(clock * 60) % 30 == 0) {
                const fuel::VehicleState st = d.state(i);
                const auto& t = rules.lineFor(size_t(i))[rr.next];
                std::printf("    t=%5.1f #%d (%.0f, %.0f, %.0f) next %zu dist %.0f  %5.1f km/h hdg %4.0f  up %.2f wheels %d resets %d\n", clock, i, at.x, at.y, at.z, rr.next,
                            std::hypot(at.x - t[0], at.z - t[2]), st.speedKmh, d.heading(i) * 57.3, 1 - 2 * (st.chassis.rotation[0] * st.chassis.rotation[0] + st.chassis.rotation[2] * st.chassis.rotation[2]), st.wheelsOnGround, racer.ai.resets());
            }
        }
    }
    // Test driver for the player: the opponents' AI at full skill, no rubber-banding.
    fuel::DriveInput autopilot(DriveSession& d) {
        if (state != Running || rules.racers[0].finished >= 0 || rules.racers[0].next >= route.size()) return {};
        return drive(racers[0], d, 0, 1.0f / 60, 1.0f);
    }
    fuel::DriveInput drive(Racer& racer, DriveSession& d, int i, float dt, float rubber) {
        const fuel::VehicleState st = d.state(i);
        const auto& q = st.chassis.rotation;
        fuel::RaceDriver::Sense s;
        s.x = st.chassis.position[0];
        s.z = st.chassis.position[2];
        s.heading = d.heading(i);
        s.speedKmh = st.speedKmh;
        s.upY = 1 - 2 * (q[0] * q[0] + q[2] * q[2]);  // y of the rotated up axis
        s.topSpeedKmh = d.setup(i).topSpeedKmh;
        fuel::DriveWorld& world = d.physics();
        s.clearance = [&world, x = s.x, z = s.z](float yaw, float length, bool slopes) { return world.clearance(x, z, yaw, length, slopes ? 0.55f : 0.0f, 0.5f); };
        const size_t next = rules.racers[size_t(i)].next;
        const fuel::DriveInput in = racer.ai.drive(lineFor(size_t(i)), next, s, dt, rubber);
        if (racer.ai.wantsReset()) {
            // Back on the line, a few metres to the side for each racer so they do not land on each other.
            const auto p = racer.ai.resetPoint();
            if (std::getenv("FUEL_AI_TRACE"))
                std::printf("    reset #%d at t=%.1f from (%.0f, %.0f) next %zu speed %.1f up %.2f\n", i, clock, s.x, s.z, next, s.speedKmh, s.upY);
            const float yaw = racer.ai.resetHeading(), side = float(i % 3 - 1) * 4.0f;
            d.teleport(p[0] + std::cos(yaw) * side, p[2] - std::sin(yaw) * side, yaw, i);
            racer.ai.didReset();
            return {};
        }
        return in;
    }
    size_t next() const { return rules.racers[0].next; }  // the player's target (the HUD and markers follow it)
    bool finished(size_t i) const { return rules.racers[i].finished >= 0; }
    // FUEL_RACE_WARP: where the player hops next (a gate, or the nearest hunted racer).
    bool warpTarget(const DriveSession& d, std::array<float, 3>& t) const {
        if (next() < route.size()) {
            t = route[next()];
            return true;
        }
        for (size_t i = 1; i < racers.size() && int(i) < d.vehicleCount(); ++i)
            if (rules.mode == fuel::Race::GmCatchAI && !rules.racers[i].out) {
                const simd_float3 p = d.position(int(i));
                t = {p.x, p.y, p.z};
                return true;
            }
        return false;
    }
    // Every racer is done (finished, knocked out or caught) or the event is over for the player.
    bool allDone() const {
        for (size_t i = 0; i < racers.size(); ++i)
            if (rules.racers[i].finished < 0 && !rules.racers[i].out && rules.racers[i].next < rules.lineFor(i).size()) return false;
        return true;
    }
    std::vector<simd_float4> markers() const {
        const size_t next = this->next();
        std::vector<simd_float4> m;
        for (size_t i = next; i < route.size() && i < next + 2; ++i)
            m.push_back(simd_make_float4(route[i][0], route[i][1], route[i][2], i + 1 == route.size() ? 2.0f : float(i - next)));
        if (rules.helicopter) m.push_back(simd_make_float4(rules.heli[0], rules.heli[1], rules.heli[2], 1.0f));
        return m;
    }
    std::string hud(const DriveSession& d) const {
        char buf[320];
        const simd_float3 at = d.position();
        const double t = std::max(0.0, clock);
        const int min = int(t / 60);
        const double sec = t - min * 60;
        const fuel::Race& r = race();
        const size_t next = this->next();
        std::string place;
        if (racers.size() > 1) place = "   P" + std::to_string(positions(d)[0]) + "/" + std::to_string(racers.size());
        std::string extra;
        if (rules.timed) extra += "   time left " + std::to_string(int(std::ceil(rules.timeLeft))) + " s";
        if (rules.helicopter) extra += "   helicopter " + std::to_string(int(std::hypot(at.x - rules.heli[0], at.z - rules.heli[2]))) + " m";
        if (rules.mode == fuel::Race::GmCatchAI) {
            int left = 0;
            for (size_t i = 1; i < rules.racers.size(); ++i) left += !rules.racers[i].out;
            extra += "   to catch " + std::to_string(left);
        }
        const std::string title = r.name + " (" + fuel::gameModeName(rules.mode) + ")";
        if (state == Countdown) {
            std::snprintf(buf, sizeof buf, "%s   starts in %d", title.c_str(), int(std::ceil(-clock)));
        } else if (state == Finished) {
            std::snprintf(buf, sizeof buf, "%s   %s   Enter: retry   N: next race", title.c_str(), rules.result(where(d)).c_str());
        } else if (next < route.size()) {
            const auto& tgt = route[next];
            const size_t perLap = r.loops() ? std::max<size_t>(1, (route.size() - 1) / size_t(std::max(1, r.laps))) : route.size();
            std::string lap = r.loops() ? "   lap " + std::to_string(std::min<size_t>(next / perLap + 1, size_t(r.laps))) + "/" + std::to_string(r.laps) : "";
            std::snprintf(buf, sizeof buf, "%s%s   checkpoint %zu/%zu%s   %d:%04.1f   next %.0f m%s", title.c_str(), place.c_str(), next + 1, route.size(),
                          lap.c_str(), min, sec, std::hypot(at.x - tgt[0], at.z - tgt[2]), extra.c_str());
        } else {
            std::snprintf(buf, sizeof buf, "%s   %d:%04.1f%s", title.c_str(), min, sec, extra.c_str());
        }
        return buf;
    }
    // The order of the field: finishers by time, then the rest by how far they got.
    std::string standings(const DriveSession& d) const {
        const std::vector<int> pos = positions(d);
        std::string out;
        for (int p = 1; p <= int(racers.size()); ++p)
            for (size_t i = 0; i < racers.size(); ++i) {
                if (pos[i] != p) continue;
                const fuel::RaceRules::Racer& r = rules.racers[i];
                char buf[160];
                if (r.finished >= 0)
                    std::snprintf(buf, sizeof buf, "  %d. %-16s %s %d:%04.1f\n", p, racers[i].name.c_str(), i == 0 ? "(you)" : "     ", int(r.finished / 60),
                                  r.finished - int(r.finished / 60) * 60);
                else if (r.out)
                    std::snprintf(buf, sizeof buf, "  %d. %-16s %s %s at %.0f s\n", p, racers[i].name.c_str(), i == 0 ? "(you)" : "     ",
                                  rules.mode == fuel::Race::GmCatchAI ? "caught" : "knocked out", r.outAt);
                else
                    std::snprintf(buf, sizeof buf, "  %d. %-16s %s racing, checkpoint %zu/%zu\n", p, racers[i].name.c_str(), i == 0 ? "(you)" : "     ",
                                  r.next + 1, rules.lineFor(i).size());
                out += buf;
            }
        if (rules.outcome != fuel::RaceRules::Pending) out += "  result: " + rules.result(where(d)) + "\n";
        return out;
    }
};

// The playable events (loadEvents), then EMD races no hub lists (the tutorial tests, unlisted
// "Career Race n" blocks), so --race NAME still finds them.
std::vector<fuel::Event> loadRaces(const std::string& dataDir) {
    std::vector<fuel::Event> out;
    std::set<std::string> have;
    auto key = [](std::string s) {
        for (char& c : s) c = char(std::toupper(static_cast<unsigned char>(c)));
        return s;
    };
    for (fuel::Event& e : fuel::loadEvents(dataDir))
        if (e.playable) {
            have.insert(key(e.race.name));
            out.push_back(std::move(e));
        }
    for (const char* file : {"miss_official.tsc", "missChallenge.tsc"}) {
        std::ifstream f(fs::path(dataDir) / "LocalizedComponent/GameTsc/Story" / file);
        std::stringstream text;
        text << f.rdbuf();
        for (fuel::Race& r : fuel::parseMissions(text.str()))
            if ((r.start[0] != 0 || r.start[2] != 0) && (!r.checkpoints.empty() || r.finish[0] != 0 || r.finish[2] != 0) &&
                r.gameMode() != fuel::Race::GmInvalid && have.insert(key(r.name)).second) {
                fuel::Event e;
                e.id = r.name;
                e.race = r;
                e.playable = true;
                e.note = "not in any hub list";
                out.push_back(std::move(e));
            }
    }
    return out;
}

// How many opponents an event puts on the grid: EMD_SetNbIA (no vehicle for the helicopter),
// or `requested` (--opponents) when given (>= 0), at most 11.
int eventOpponents(const fuel::Event& e, int requested) {
    if (e.race.gameMode() == fuel::Race::GmHelicopter) return 0;
    return std::clamp(requested >= 0 ? requested : e.race.aiCount, 0, 11);
}

// Puts `count` opponents of the player's class into the drive session and the race,
// each with a random livery and colour of its own, and hands their parts to the renderer.
void addOpponents(Garage& garage, DriveSession& drive, RaceSession& race, int count, fuel::MeshRenderer& renderer, const VehicleLook& prefs) {
    std::mt19937 rng(fuel::asoboCrc(race.race().name));
    race.racers.resize(1);
    race.racers[0].name = garage.label(garage.current);
    race.racers[0].entry = garage.current;
    std::vector<std::vector<fuel::MeshInstance>> parts;
    std::vector<fuel::Paint> paints;
    const simd_float3 at = drive.position();
    for (size_t entry : garage.rivals(garage.current, count, rng)) {
        fuel::AssembledVehicle av;
        fuel::VehicleSetup setup;
        std::vector<uint32_t> rider;
        if (!garage.load(entry, av, setup, rider)) continue;
        const int i = drive.addVehicle(av, setup, at.x, at.z + 30.0 * drive.vehicleCount(), 0, rider);  // the grid comes with RaceSession::begin
        RaceSession::Racer racer;
        racer.name = garage.label(entry);
        racer.entry = entry;
        const float lanes[3] = {3.5f, -3.5f, 0.0f};  // matching the grid: right, left, centre
        racer.ai = fuel::RaceDriver(std::uniform_real_distribution<float>(0.85f, 1.0f)(rng), lanes[race.racers.size() % 3]);
        race.racers.push_back(racer);
        // Paint: a random livery the vehicle has (or plain paint) and a random palette colour.
        const AssemblyMode& am = *garage.archives[garage.entries[entry].first];
        VehicleLook look = makeLook(racer.name, am.assembler->variants()[garage.entries[entry].second].name, am.palettes, am.assembler->dpc(), prefs);
        std::vector<int> liveries;
        for (int l = 1; l <= 9; ++l)
            if (look.liveries[size_t(l)][0] != 0) liveries.push_back(l);
        look.livery = liveries.empty() ? 0 : liveries[rng() % liveries.size()];
        look.custom = false;
        look.color = look.palette.empty() ? 0 : int(rng() % look.palette.size());
        paints.push_back(look.paint());
        parts.push_back(drive.parts(i));
    }
    renderer.setVehicles(parts, paints);
    renderer.shadowRadius = parts.empty() ? 0.0f : 22.0f;  // shadows for the vehicles around the player
}

// Moves the opponents' parts to their physics poses.
void showOpponents(DriveSession& drive, fuel::MeshRenderer& renderer) {
    for (int i = 1; i < drive.vehicleCount(); ++i) renderer.setVehicleWorlds(size_t(i - 1), drive.partWorlds(i));
}

// --ai-check: each racer's checkpoints, speed, position and resets.
void printAiCheck(const RaceSession& race, const DriveSession& drive) {
    const std::vector<int> pos = race.positions(drive);
    std::printf("ai check t=%5.1f s\n", race.clock);
    for (size_t i = 0; i < race.racers.size(); ++i) {
        const RaceSession::Racer& r = race.racers[i];
        const fuel::RaceRules::Racer& rr = race.rules.racers[i];
        const fuel::VehicleState st = drive.state(int(i));
        std::printf("  P%d %-16s %s skill %.2f  checkpoints %2zu/%zu  %5.1f km/h  resets %d%s\n", pos[i], r.name.c_str(), i == 0 ? "(auto)" : "      ",
                    r.ai.skill(), rr.next, race.rules.lineFor(i).size(), st.speedKmh, r.ai.resets(),
                    rr.finished >= 0 ? (" finished " + std::to_string(int(rr.finished)) + " s").c_str() : rr.out ? " out" : "");
    }
}

// Passes when every opponent got through checkpoints and none needed more than a few resets.
bool aiCheckVerdict(const RaceSession& race) {
    int moving = 0, resets = 0;
    for (size_t i = 1; i < race.racers.size(); ++i) {
        moving += race.rules.racers[i].next >= std::min<size_t>(3, race.rules.lineFor(i).size());
        resets += race.racers[i].ai.resets();
    }
    const int n = int(race.racers.size()) - 1;
    const bool ok = n > 0 && moving == n && resets <= 2 * n;
    std::printf("ai check: %d of %d opponents past 3 checkpoints, %d resets in all: %s\n", moving, n, resets, ok ? "ok" : "FAILED");
    return ok;
}

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

// Renders one frame to o.shot; returns its GPU time in milliseconds.
// The camera override the race asks for (0x402930): the intro during the countdown (view 8), the finish (9).
fuel::cam::Override cameraOverride(const RaceSession* race) {
    if (race == nullptr) return fuel::cam::Override::None;
    if (race->state == RaceSession::Countdown) return fuel::cam::Override::Intro;
    if (race->state == RaceSession::Finished || (!race->rules.racers.empty() && race->rules.racers[0].finished >= 0)) return fuel::cam::Override::Finish;
    return fuel::cam::Override::None;
}

// Rider vectors for --camcheck: the slot table and action properties replayed from the player's FUEL.exe
// (emulated 0x5d6950 / 0x5d3660, p25 R2/R3) and the sway target 0x5d7d40.
std::vector<fuel::CamCheckItem> riderCheckItems(const std::string& dataDir) {
    std::vector<fuel::CamCheckItem> out;
    std::string err;
    const auto t = fuel::loadRiderTables((fs::path(dataDir) / "XFinalComponent/FUEL.exe").string(), &err);
    if (!t) {
        out.push_back({"Rider tables 0x5d6950/0x5d3660", false, err});
        return out;
    }
    static const int kMbike[47] = {/* engine table removed from the published history; read from FUEL.exe at run time */};
    bool ok = true;
    std::string row;
    for (int k = 0; k < 47; ++k) ok = ok && t->slots[2][size_t(k)] == kMbike[k], row += " " + std::to_string(t->slots[2][size_t(k)]);
    out.push_back({"Slot table 0x5d6950 MBIKE actions 0..46", ok, "got" + row});
    static const int kBase[11] = {/* engine table removed from the published history; read from FUEL.exe at run time */};
    ok = true, row.clear();
    for (int a = 0; a < 11; ++a) ok = ok && t->slots[size_t(a)][0] == kBase[a], row += " " + std::to_string(t->slots[size_t(a)][0]);
    out.push_back({"Slot table 0x5d6950 base slots (11 archetypes)", ok, "got" + row});
    // R3 action properties {group, priority, mode}
    auto want = [](int k) -> std::array<int, 3> {
        return {0, 0, 0};  // engine table removed from the published history; read from FUEL.exe at run time
    };
    ok = true;
    int bad = -1;
    for (int k = 0; k < 47; ++k) {
        const auto w = want(k);
        const auto& p = t->props[size_t(k)];
        if (p.group != w[0] || p.priority != w[1] || p.mode != w[2]) ok = false, bad = bad < 0 ? k : bad;
    }
    out.push_back({"Action props 0x5d3660 (47 actions)", ok, ok ? "all 47 records" : "first difference at action " + std::to_string(bad)});
    const auto s1 = fuel::RiderAnimator::swayTarget({0, 0, 0, 1}, {0, 0, 5}), s2 = fuel::RiderAnimator::swayTarget({0, 0, 0, 1}, {30, 0, 0});
    const bool sOk = std::fabs(s1[1] - 0.025f) < 1e-7f && std::fabs(s2[1] - 0.05f) < 1e-7f && s1[0] == 0 && s1[2] == 0 && s2[0] == 0 && s2[2] == 0;
    char b[96];
    std::snprintf(b, sizeof b, "got (0,%.7g,0) (0,%.7g,0)", s1[1], s2[1]);
    out.push_back({"Rider sway target 0x5d7d40", sOk, b});
    return out;
}

// --camcheck: FUEL's camera (and rider) functions against the emulated test vectors (p25 §P4, p27).
int runCamCheck(const std::string& dataDir) {
    std::vector<std::function<std::vector<fuel::CamCheckItem>()>> extra;
    extra.push_back([&] { return riderCheckItems(dataDir); });
    const fuel::CamCheckResult r = fuel::runCameraVectors(extra);
    std::printf("%s", r.report.c_str());
    std::printf("camcheck: %d of %d vectors match\n", r.matched, r.total);
    return r.matched == r.total ? 0 : 1;
}

double renderShot(id<MTLDevice> device, fuel::MeshRenderer& renderer, const Options& o, const fuel::OrbitCamera& cam) {
    id<MTLCommandQueue> queue = [device newCommandQueue];
    MTLTextureDescriptor* td = [MTLTextureDescriptor texture2DDescriptorWithPixelFormat:kColorFormat
                                                                                  width:NSUInteger(o.width)
                                                                                 height:NSUInteger(o.height)
                                                                              mipmapped:NO];
    td.usage = MTLTextureUsageRenderTarget;
    td.storageMode = MTLStorageModePrivate;
    id<MTLTexture> image = [device newTextureWithDescriptor:td];
    // FUEL_GPU_BENCH=N: render the frame N more times first and report the GPU time.
    const int bench = std::getenv("FUEL_GPU_BENCH") ? std::max(0, std::atoi(std::getenv("FUEL_GPU_BENCH"))) : 0;
    double total = 0, best = 1e9;
    for (int k = 0; k <= bench; ++k) {
        id<MTLCommandBuffer> cb = [queue commandBuffer];
        renderer.render(cb, image, cam);
        if (gOverlay) gOverlay->draw(cb, image, gShotHud);
        [cb commit];
        [cb waitUntilCompleted];
        const double ms = (cb.GPUEndTime - cb.GPUStartTime) * 1000.0;
        if (k == bench) {
            if (bench > 0) std::printf("gpu bench %s: %d frames at %dx%d, mean %.2f ms, best %.2f ms\n", o.shot.c_str(), bench, o.width, o.height, total / bench, best);
            writePng(device, queue, image, o.shot);
            return ms;
        }
        total += ms, best = std::min(best, ms);
    }
    return 0;
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
    std::unique_ptr<fuel::DriveAudio> _audio; // --drive in a window: engine, tyre, wind, impact sounds
    std::vector<std::unique_ptr<fuel::DriveAudio>> _rivalAudio;  // the opponents' sounds, louder as they come near
    NSTextField* _hud;        // --drive: speed, gear, race
    std::set<unsigned short> _keys;
    double _lastTime;
    float _steer;             // smoothed keyboard steering
    float _camYawOffset;      // mouse look around the chase camera
    fuel::OrbitCamera _startCam;  // --drive: mouse pitch and zoom act relative to this
    ChaseCamera _chase;       // --drive: the camera actually used (_cam holds the mouse's changes)
    bool _viewChange;         // V pressed: the CAMERA button
    fuel::OrbitCamera _chaseView;
    DustSystem _dust;
    fuel::Recorder::Frame _frame;  // --record: this frame's timings
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
            _audio->setMasterVolume(gGameVolume);
            [self loadAudio];
            for (size_t i = 1; race && i < race->racers.size(); ++i) {
                const size_t entry = race->racers[i].entry;
                const AssemblyMode& am = *garage->archives[garage->entries[entry].first];
                const auto it = garage->params.find(race->racers[i].name);
                auto audio = std::make_unique<fuel::DriveAudio>();
                audio->setMasterVolume(0);
                if (it != garage->params.end()) audio->setVehicle(am.assembler->dpc(), race->racers[i].name, it->second);
                _rivalAudio.push_back(std::move(audio));
            }
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
        _viewChange = false;
        _startCam = cam;
        _sceneMode = assembly != nullptr || !scene.parts.empty();
        self.colorPixelFormat = kColorFormat;  // the renderer draws offscreen and finishes into the drawable
        self.depthStencilPixelFormat = MTLPixelFormatInvalid;
        self.sampleCount = 1;
        self.delegate = self;
        if (gRecorder.active()) self.framebufferOnly = NO;  // frame grabs read the drawable
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
    const size_t n = _garage->entries.size();
    for (size_t tries = 0; tries < n; ++tries) {
        _garage->current = (_garage->current + n + size_t(dir + int(n))) % n;
        fuel::AssembledVehicle av;
        fuel::VehicleSetup setup;
        std::vector<uint32_t> rider;
        if (!_garage->load(_garage->current, av, setup, rider)) continue;
        AssemblyMode& am = *_garage->archives[_garage->entries[_garage->current].first];
        _drive->replacePlayer(av, setup, rider);  // the world and the opponents stay
        if (_race) _race->racers[0].name = am.variant().label;
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

// T / Shift-T: the nearest hub's next teleport / vista point (placed on a clear spot nearby).
- (void)hubJump:(bool)vista {
    static std::map<std::string, size_t> next;  // per hub and kind: the point to visit next
    const simd_float3 at = _drive->position();
    const fuel::Hub* h = fuel::nearestHub(gHubs, at.x, at.z);
    if (h == nullptr) return;
    const auto& list = vista ? h->vistas : h->teleports;
    if (list.empty() || gTerrain == nullptr) return;
    size_t& k = next[h->id + (vista ? "/v" : "/t")];
    const fuel::HubPoint& p = list[k % list.size()];
    k = (k + 1) % list.size();
    const HubSpawn s = placeAtHubPoint(*_drive, *_streamer, *_renderer, *gTerrain, p);
    std::printf("%s: %s at (%.0f, %.0f)%s\n", vista ? "vista" : "teleport", p.name.c_str(), s.x, s.z, s.nudged ? " (nudged)" : "");
}

- (void)keyDown:(NSEvent*)e {
    if (gOverlay && gOverlay->keyDown(e.keyCode)) return;
    if (_drive) {
        if ([self driveKey:e.keyCode]) {
            _keys.insert(e.keyCode);
            return;
        }
        if (e.keyCode == 40 && gRecorder.active()) {  // K: mark a glitch in the recording
            gRecorder.mark("MARK");
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
        if (e.keyCode == 17 && !gHubs.empty() && _streamer && !_race) {  // T: next teleport of the nearest hub (Shift-T: next vista point)
            [self hubJump:(e.modifierFlags & NSEventModifierFlagShift) != 0];
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
    else if ([ch isEqualToString:@"v"]) _viewChange = true;
    else if ([ch isEqualToString:@"r"]) _cam = fuel::OrbitCamera{};
    else if ([ch isEqualToString:@"o"]) _renderer->postEffects = !_renderer->postEffects;
    else if ([ch isEqualToString:@"d"]) _renderer->dirt = _renderer->dirt >= 1.0f ? 0.0f : _renderer->dirt + 0.5f;
    else if ([ch isEqualToString:@"g"]) { _renderer->showGround = !_renderer->showGround; _renderer->showTerrain = !_renderer->showTerrain; }
    else if ([ch isEqualToString:@"q"] && !gOverlay) [NSApp terminate:nil];  // --in-game: quit through the menu
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

// --in-game: the race order for the results page and the HUD.
- (std::vector<fuel::ui::ResultRow>)resultRows {
    std::vector<fuel::ui::ResultRow> rows;
    if (!_race) return rows;
    const std::vector<int> pos = _race->positions(*_drive);
    for (int p = 1; p <= int(_race->racers.size()); ++p)
        for (size_t i = 0; i < _race->racers.size(); ++i)
            if (pos[i] == p) rows.push_back({_race->racers[i].name, float(std::max(0.0, _race->rules.racers[i].finished)), i == 0});
    return rows;
}

- (fuel::ui::HudState)hudState {
    static float odometer = 0;
    static double last = 0;
    const double now = CACurrentMediaTime();
    fuel::ui::HudState h;
    const fuel::VehicleState st = _drive->state();
    h.speedKmh = std::fabs(st.speedKmh);
    if (last > 0 && !gOverlay->open()) odometer += h.speedKmh / 3600.0f * float(std::min(0.1, now - last));
    last = now;
    h.odometerKm = odometer;
    const simd_float3 at = _drive->position();
    const auto& hubs = gOverlay->menu().hubs();
    if (const fuel::Hub* hub = fuel::nearestHub(hubs, at.x, at.z))
        h.hubName = hub->nameText ? gOverlay->menu().textOf(hub->nameText) : hub->id;
    if (_race && _race->racers.size() > 1) {
        h.race = true;
        h.standings = [self resultRows];
        for (auto& r : h.standings) r.time = 0;
    }
    return h;
}

- (void)mouseDown:(NSEvent*)e {
    if (!gOverlay || !gOverlay->open()) return;
    const NSPoint p = [self convertPoint:e.locationInWindow fromView:nil];
    const CGSize ds = self.drawableSize;
    gOverlay->pointer(float(p.x * ds.width / self.bounds.size.width), float((self.bounds.size.height - p.y) * ds.height / self.bounds.size.height), true,
                      float(ds.width), float(ds.height));
}

- (void)driveStep {
    const double now = CACurrentMediaTime();
    const float dt = _lastTime > 0 ? float(std::min(now - _lastTime, 0.1)) : 0.0f;
    _lastTime = now;
    if (gOverlay) {  // --in-game: the menu pauses the session
        gOverlay->pollGamepad();
        const auto action = gOverlay->update(dt);
        if (action == fuel::ui::GameOverlay::Restart) {
            if (_race) _race->begin(*_drive, _streamer, *_renderer);
            else _drive->reset();
        }
        if (gOverlay->open()) return;
    }
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
    _frame.dt = dt;
    double t0 = CACurrentMediaTime();
    if (_race) _race->update(dt, *_drive, in);
    if (_race) _drive->setRaceState(_race->state == RaceSession::Countdown ? 4 : _race->state == RaceSession::Finished ? 6 : 0);
    _drive->update(dt, in);
    _frame.physicsMs = (CACurrentMediaTime() - t0) * 1000.0;
    if (_race) {
        _renderer->setMarkers(_race->markers());
        if (_race->state == RaceSession::Finished && !_race->reported) {
            _race->reported = true;
            std::printf("%s finished: order\n%s", _race->race().name.c_str(), _race->standings(*_drive).c_str());
            if (gOverlay) gOverlay->openResults([self resultRows]);
        }
    }
    showOpponents(*_drive, *_renderer);
    t0 = CACurrentMediaTime();
    if (_streamer) _streamer->update(_drive->position(), *_renderer, _drive->physics());
    _frame.streamMs = (CACurrentMediaTime() - t0) * 1000.0;
    _renderer->setInstanceWorlds(_drive->partWorlds());
    // Chase camera: swings behind the vehicle; mouse look decays back, drag and
    // scroll raise and zoom it.
    // FUEL's camera: mouse drag = the look stick (CAM X/Y), V = the CAMERA button (views 5 -> 1 -> 4).
    fuel::cam::CameraControls cc;
    cc.stickX = std::clamp(_camYawOffset * -2.0f, -1.0f, 1.0f);
    cc.stickY = std::clamp((_cam.pitch - _startCam.pitch) * 2.0f, -1.0f, 1.0f);
    cc.viewChange = _viewChange;
    _viewChange = false;
    if (GCExtendedGamepad* pad = GCController.controllers.firstObject.extendedGamepad) {
        if (std::fabs(pad.rightThumbstick.xAxis.value) > 0.1f) cc.stickX = pad.rightThumbstick.xAxis.value;
        if (std::fabs(pad.rightThumbstick.yAxis.value) > 0.1f) cc.stickY = -pad.rightThumbstick.yAxis.value;
    }
    _chaseView = _chase.update(dt, *_drive, cc, cameraOverride(_race));
    _camYawOffset *= std::max(0.0f, 1.0f - dt * 4.0f);
    _cam.pitch = _startCam.pitch + (_cam.pitch - _startCam.pitch) * std::max(0.0f, 1.0f - dt * 4.0f);
    _drive->uploadRiders();
    _renderer->setFocus(_chase.focus());
    _renderer->speedBlur = _chase.speedEffect();
    _dust.update(dt, *_drive, _drive->position());
    _renderer->setDust(_dust.particles());
    const fuel::VehicleState st = _drive->state();
    _frame.speedKmh = std::fabs(st.speedKmh);
    _frame.gear = st.gear;
    _frame.wheelsOnGround = st.wheelsOnGround;
    _frame.vehicles = _drive->vehicleCount();
    const simd_float3 at = _drive->position();
    _frame.x = at.x, _frame.y = at.y, _frame.z = at.z;
    if (_audio) _audio->update(dt, st, in.throttle);
    for (size_t k = 0; k < _rivalAudio.size() && int(k) + 1 < _drive->vehicleCount(); ++k) {
        // Quiet, and only within ~150 m (no panning).
        const int i = int(k) + 1;
        const float near = std::clamp(1.0f - simd_distance(_drive->position(), _drive->position(i)) / 150.0f, 0.0f, 1.0f);
        _rivalAudio[k]->setMasterVolume(_audio && _audio->muted() ? 0.0f : 0.45f * near * near * gGameVolume);
        _rivalAudio[k]->update(dt, _drive->state(i), _drive->input(i).throttle);
    }
    self.window.title = [NSString stringWithFormat:@"FUEL — %s", _scene.variant.c_str()];
    const std::string raceLine = _race ? _race->hud(*_drive) : _scene.variant;
    _hud.stringValue = [NSString stringWithFormat:@"%s\n%3.0f km/h   gear %d   %5.0f rpm   %s", raceLine.c_str(), std::fabs(st.speedKmh),
                                                  st.gear, st.rpm, _race ? _scene.variant.c_str() : ""];
}

- (void)drawInMTKView:(MTKView*)view {
    if (_drive) [self driveStep];
    id<CAMetalDrawable> drawable = view.currentDrawable;
    if (drawable == nil) return;
    id<MTLCommandBuffer> cb = [_queue commandBuffer];
    const double t0 = CACurrentMediaTime();
    _renderer->render(cb, drawable.texture, _drive ? _chaseView : _cam);
    if (gOverlay && _drive) gOverlay->draw(cb, drawable.texture, [self hudState]);
    if (gRecorder.active()) {
        _frame.encodeMs = (CACurrentMediaTime() - t0) * 1000.0;
        gRecorder.watchGpu(cb);
        if (gRecorder.frame(_frame)) gRecorder.grab(cb, drawable.texture);
        _frame = {};
    }
    [cb presentDrawable:drawable];
    [cb commit];
}
@end

@interface AppDelegate : NSObject <NSApplicationDelegate>
@end
@implementation AppDelegate
- (BOOL)applicationShouldTerminateAfterLastWindowClosed:(NSApplication*)app { return YES; }
@end

// A plain dark window with "Loading…", drawn once before the main thread starts loading.
NSWindow* showLoadingWindow() {
    [NSApplication sharedApplication];
    [NSApp setActivationPolicy:NSApplicationActivationPolicyRegular];
    const NSRect frame = NSMakeRect(0, 0, 640, 360);
    NSWindow* w = [[NSWindow alloc] initWithContentRect:frame styleMask:NSWindowStyleMaskTitled
                                                backing:NSBackingStoreBuffered defer:NO];
    w.title = @"FUEL";
    w.releasedWhenClosed = NO;
    w.backgroundColor = [NSColor colorWithCalibratedRed:0.05 green:0.045 blue:0.04 alpha:1];
    NSTextField* label = [NSTextField labelWithString:@"Loading…"];
    label.font = [NSFont boldSystemFontOfSize:28];
    label.textColor = [NSColor colorWithCalibratedWhite:0.85 alpha:1];
    [label sizeToFit];
    [label setFrameOrigin:NSMakePoint((frame.size.width - label.frame.size.width) / 2, (frame.size.height - label.frame.size.height) / 2)];
    [w.contentView addSubview:label];
    [w center];
    [w makeKeyAndOrderFront:nil];
    [NSApp activateIgnoringOtherApps:YES];
    [w displayIfNeeded];
    // Let the window server show it before the loading blocks the main thread.
    for (int i = 0; i < 5; ++i) {
        NSEvent* e = [NSApp nextEventMatchingMask:NSEventMaskAny untilDate:[NSDate dateWithTimeIntervalSinceNow:0.02]
                                           inMode:NSDefaultRunLoopMode dequeue:YES];
        if (e) [NSApp sendEvent:e];
    }
    return w;
}

int main(int argc, const char** argv) {
    @autoreleasepool {
        Options opt = parseArgs(argc, argv);
        if (opt.menuCheck) return fuel::ui::runMenuCheck(opt.dataDir, fuel::loadSettings().language);
        if (opt.camCheck) return runCamCheck(opt.dataDir);
        if (!opt.menuShot.empty()) {
            fuel::Settings ms = fuel::loadSettings();
            ms.dataDir = opt.dataDir;
            return fuel::ui::runMenuShot(opt.dataDir, ms, opt.menuShot, opt.menuShotOut, opt.width > 0 ? opt.width : 1280, opt.height > 0 ? opt.height : 720);
        }
        if (opt.game) {
            fuel::Settings gs = fuel::loadSettings();
            gs.dataDir = opt.dataDir;
            return fuel::ui::runGameMenu(argv[0], opt.dataDir, gs, opt.menuPage);
        }
        // --in-game: the menu's window is gone after the exec, and loading the world takes a while,
        // so a loading window stands in until the game's own window opens.
        NSWindow* loadingWindow = nil;
        if (opt.inGame && opt.shot.empty() && !opt.aiCheck && !opt.eventCheck && !opt.hubCheck && !opt.propCheck && !opt.audioCheck)
            loadingWindow = showLoadingWindow();
        if (!opt.hub.empty() || opt.hubCheck || (opt.drive && opt.shot.empty())) {  // hubs: --hub, --hub-check and the T key
            try {
                gHubs = fuel::loadHubs(opt.dataDir);
            } catch (const std::exception& e) {
                std::fprintf(stderr, "fuelview: hubs: %s\n", e.what());
            }
        }
        if (!opt.hub.empty()) {
            const fuel::Hub* h = fuel::findHub(gHubs, opt.hub);
            if (h == nullptr) {
                std::string list;
                for (const fuel::Hub& x : gHubs) list += " \"" + x.id + "\"";
                die("no hub '" + opt.hub + "'; hubs:" + list);
            }
            opt.atX = h->camp.x, opt.atZ = h->camp.z, opt.atYaw = h->camp.yaw, opt.haveAt = true;
        }

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
        renderer.postEffects = std::getenv("FUEL_NO_POST") == nullptr;  // FUEL_NO_POST=1: plain tone mapping (O toggles in the window)
        fuel::Settings playerSettings;
        if (opt.settings) {
            playerSettings = fuel::loadSettings();
            applySettings(playerSettings, renderer);
        }
        renderer.cullBackFaces = opt.cull;
        renderer.dirt = std::clamp(opt.dirt, 0.0f, 1.0f);
        std::unique_ptr<fuel::Heightfield> terrain;
        if (opt.terrain) {
            const fs::path world = fs::path(opt.dataDir) / "CommonComponent/WORLD/USA1.DPC";
            try {
                fuel::DpcFile usa1(world.string());
                terrain = std::make_unique<fuel::Heightfield>(fuel::loadLevelledHeightfield(usa1));
                renderer.setTerrain(*terrain);
                gTerrain = terrain.get();
                gGround = fuel::buildGroundMaps(usa1, *terrain, 2334);
                renderer.setGround(gGround, usa1);
                gSurfaces.ground = &gGround;
                fuel::DriveWorld::setDefaultSurfaceQuery([](double x, double z) { return gSurfaces.at(x, z); });
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
                riders = std::make_unique<RiderView>(*library.files().back(), text.str(), renderer,
                                                     (fs::path(opt.dataDir) / "XFinalComponent/FUEL.exe").string());
                gRiderView = riders.get();
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
        std::unique_ptr<fuel::Vegetation> vegetation;
        std::unique_ptr<fuel::RoadNetwork> roads;
        std::unique_ptr<fuel::Lakes> lakes;
        std::unique_ptr<fuel::GroundGenerator> ground;  // p14: grass and the trees' ground tests
        std::unique_ptr<WorldStreamer> streamer;  // after what its worker reads, so it is destroyed (and waits) first
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
                                                           opt.haveAt ? opt.atYaw : 0.0f,
                                                           riders ? riders->animated(name, av, subject, setup.params) : std::vector<uint32_t>{});
                } catch (const std::exception& e) {
                    die(name + ": " + e.what());
                }
                scene.parts = drive->parts();
                renderer.cameraClearance = [d = drive.get()](simd_float3 from, simd_float3 to) {
                    // A 0.6 m sphere, so the camera stays clear of trunks and walls, not just in sight of the target.
                    float t = d->physics().rayFraction({from.x, from.y, from.z}, {to.x, to.y, to.z}, 0.6f);
                    // Opponents (vehicle and rider) as 1.8 m spheres, so the camera never ends up inside one.
                    const simd_float3 dir = to - from;
                    const float len2 = simd_length_squared(dir);
                    for (int i = 1; i < d->vehicleCount() && len2 > 1e-6f; ++i) {
                        const simd_float3 c = d->position(i) - from;
                        const float along = simd_dot(c, dir) / len2;
                        if (along <= 0 || along >= t) continue;
                        const float miss2 = simd_length_squared(c - dir * along), r = 1.8f;
                        // Never closer than 3 m to the target: a rival alongside must not push
                        // the camera into the player's own rider.
                        const float minT = std::min(1.0f, 3.0f / std::sqrt(len2));
                        if (miss2 < r * r) t = std::max(minT, along - std::sqrt((r * r - miss2) / len2));
                    }
                    return t;
                };
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
                            {
                                std::unordered_set<uint32_t> foliage;
                                for (const auto* list : {&vegetation->trees, &vegetation->bushes})
                                    for (const fuel::Species& sp : *list) foliage.insert(sp.full);
                                renderer.setFoliage(std::move(foliage));
                            }
                            streamer->setVegetation(vegetation.get(), terrain.get());
                            streamer->setSurfaces([](double x, double z) { return gSurfaces.at(x, z); });
                            roads = std::make_unique<fuel::RoadNetwork>(fuel::loadRoads(*f));
                            streamer->setRoads(roads.get());
                            gSurfaces.roads = roads.get();
                            std::printf("roads: %zu drawable lines\n", roads->lines.size());
                            std::printf("vegetation: %zu tree and %zu bush species\n", vegetation->trees.size(), vegetation->bushes.size());
                            ground = std::make_unique<fuel::GroundGenerator>(*f, *terrain, roads.get());  // p14
                            // p22: FUEL's road segments (roads painted into the ground, decal path), shared
                            // with the ground, which carves the roads with them (loadLevelledHeightfield).
                            if (terrain->roads)
                                ground->setRoadSampler(terrain->roads.get());
                            else
                                std::fprintf(stderr, "fuelview: road table not rebuilt from FUEL.exe; roads use the port's widths\n");
                            // p22: the lakes the engine adds from code (FUEL.exe table, flood-filled on
                            // the bare height map) and their water meshes.
                            try {
                                // Flood fill on the map's B-spline (0x445570 = the base height h0).
                                std::unique_ptr<fuel::Heightfield> bare;
                                if (!terrain->detail) bare = std::make_unique<fuel::Heightfield>(fuel::loadWorldHeightfield(*f));
                                const fuel::Heightfield* hf = terrain.get();
                                lakes = std::make_unique<fuel::Lakes>(*f, [hf, &bare](double x, double z) {
                                    return hf->detail ? hf->detail->baseHeight(x, z) : double(bare->smoothHeightAt(x, z));
                                });
                                if (lakes->valid()) {
                                    ground->setLakes(lakes.get());
                                    streamer->setLakes(lakes.get());
                                    if (drive) {  // p24: water for FUEL's vehicle physics, 0x4df5a0's first term (depth above the 2 m floor)
                                        const fuel::Lakes* lk = lakes.get();
                                        const fuel::Heightfield* th = terrain.get();
                                        drive->physics().setWaterDepthQuery([lk, th](double x, double z) {
                                            bool inside = false;
                                            for (const fuel::Lake& l : lk->lakes()) inside = inside || l.contains(x, z);
                                            if (!inside) return 0.0f;
                                            return float(std::max(0.0, lk->depthAbove(x, z, th->smoothHeightAt(x, z)) - 2.0));
                                        });
                                    }
                                    const std::vector<float> wc = fuel::readEngineFloats(*f, 0xa26880, 32);  // WaterConstant_Z defaults
                                    if (wc.size() == 32) {
                                        std::array<simd_float4, 8> c;
                                        for (int k = 0; k < 8; ++k) c[size_t(k)] = simd_make_float4(wc[size_t(4 * k)], wc[size_t(4 * k + 1)], wc[size_t(4 * k + 2)], wc[size_t(4 * k + 3)]);
                                        renderer.setWaterLook(c, *f);
                                    }
                                }
                            } catch (const std::exception& e) {
                                std::fprintf(stderr, "fuelview: lakes: %s\n", e.what());
                            }
                            streamer->setGround(ground.get());
                            renderer.setGrassAtlas(ground->grassAtlas());
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
                if (!opt.hub.empty()) {  // free roam: on the camp, or the nearest clear spot
                    const fuel::HubPoint camp{"", opt.atX, opt.atZ, opt.atYaw};
                    const HubSpawn s = placeAtHubPoint(*drive, *streamer, renderer, *terrain, camp);
                    std::printf("hub %s: camp at (%.0f, %.0f) heading %.0f%s\n", opt.hub.c_str(), s.x, s.z, s.yaw * 57.2958f, s.nudged ? " (nudged to a clear spot)" : "");
                }
                if (opt.hubCheck) return runHubCheck(gHubs, *drive, *streamer, renderer, *terrain);
                if (opt.eventCheck) return runEventCheck(fuel::loadEvents(opt.dataDir), *drive, *streamer, renderer, *terrain, opt.eventOnly);
                if (opt.captureFind) {
                    printCaptureCandidates(opt.findX, opt.findZ, opt.findRadius, *streamer, roads.get(), *terrain);
                    return 0;
                }
                if (!opt.captureSpot.empty() || !opt.captureAll.empty()) {  // reference captures: settled world, chase camera at rest
                    std::string err;
                    if (!renderer.setScene(scene.parts, &err)) die(scene.variant + ": " + err);
                    std::vector<const CaptureSpot*> todo;
                    if (!opt.captureSpot.empty()) todo.push_back(findCaptureSpot(opt.spots, opt.captureSpot));
                    else
                        for (const CaptureSpot& sp : opt.spots) todo.push_back(&sp);
                    if (!opt.captureAll.empty()) fs::create_directories(opt.captureAll);
                    for (const CaptureSpot* sp : todo) {
                        const CaptureView v = prepareCapture(*sp, *drive, *streamer, renderer, *terrain);
                        renderer.setFocus(v.focus);
                        renderer.speedBlur = 0;
                        Options frame = opt;
                        frame.shot = opt.captureAll.empty() ? opt.shot : (fs::path(opt.captureAll) / (sp->name + ".png")).string();
                        std::printf("%s\n", v.report.c_str());
                        const double gpuMs = renderShot(device, renderer, frame, v.cam);
                        std::printf("wrote %s (%dx%d, gpu %.1f ms)\n", frame.shot.c_str(), frame.width, frame.height, gpuMs);
                    }
                    return 0;
                }
                if (!opt.race.empty()) {
                    race = std::make_unique<RaceSession>();
                    race->terrain = terrain.get();
                    race->races = loadRaces(opt.dataDir);
                    if (race->races.empty()) die("no races found in the mission scripts");
                    auto it = std::find_if(race->races.begin(), race->races.end(), [&](const fuel::Event& e) {
                        return strcasecmp(e.id.c_str(), opt.race.c_str()) == 0 || strcasecmp(e.race.name.c_str(), opt.race.c_str()) == 0;
                    });
                    if (it == race->races.end()) die("no race named " + opt.race);
                    race->index = size_t(it - race->races.begin());
                    addOpponents(*garage, *drive, *race, eventOpponents(*it, opt.opponents), renderer, prefs);
                    race->begin(*drive, streamer.get(), renderer);
                    std::printf("race %s: %zu targets, %zu opponents; %zu races (N for the next)\n", race->race().name.c_str(), race->route.size(),
                                race->racers.size() - 1, race->races.size());
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
                    ChaseCamera chase;  // --film: the window's chase camera and dust, stepped with the physics
                    DustSystem dust;
                    fuel::OrbitCamera chaseView;
                    float speedBefore = 0, speedMin = 1e9f, firstContactS = -1, speedLater = -1;
                    const size_t movedBase = streamer ? streamer->propStats().moved : 0;
                    for (int i = 0; i < int(runTime * 60); ++i) {
                        fuel::DriveInput in{1.0f, 0, 0, false};
                        if (race && !opt.propCheck) {
                            if (race->state == RaceSession::Finished && !opt.aiCheck) break;
                            in = race->autopilot(*drive);
                            if (std::getenv("FUEL_RACE_WARP") && race->state == RaceSession::Running && i % 60 == 0) {
                                // Logic test: hop to the next target every second.
                                std::array<float, 3> t;
                                if (race->warpTarget(*drive, t)) drive->teleport(t[0], t[2], drive->heading());
                            }
                            race->update(1.0f / 60, *drive, in);
                            if (std::getenv("FUEL_RACE_TRACE") && i % 600 == 0) {
                                const simd_float3 p = drive->position();
                                std::printf("  t=%5.0fs at (%.0f, %.0f, %.0f) %5.1f km/h  heading %4.0f  %s\n", i / 60.0, p.x, p.y, p.z,
                                            drive->state().speedKmh, drive->heading() * 180 / M_PI, race->hud(*drive).c_str());
                            }
                            if (opt.aiCheck && i % 600 == 0) printAiCheck(*race, *drive);
                            if (opt.aiCheck && race->allDone()) break;
                        }
                        if (opt.dropAt >= 0 && i == int(opt.dropAt * 60)) {  // --drop: a jump landing
                            const auto st0 = drive->state();
                            const float yaw = drive->heading();
                            const float v = std::fabs(st0.speedKmh) / 3.6f;
                            drive->physics().nudge(0, opt.dropHeight, {v * std::sin(yaw), 0, v * std::cos(yaw)});
                            std::printf("drop: player lifted %.1f m at t=%.1f\n", opt.dropHeight, opt.dropAt);
                        }
                        if (race) drive->setRaceState(race->state == RaceSession::Countdown ? 4 : race->state == RaceSession::Finished ? 6 : 0);
                        const double tp0 = CACurrentMediaTime();
                        drive->update(1.0f / 60, in);
                        const double tp1 = CACurrentMediaTime();
                        {  // physics cost per frame (all vehicles, props, queries), for --ai-check
                            gPhysStats.add((tp1 - tp0) * 1000.0, drive->physics().lastVehicleMs(), drive->physics().lastVehicleCpuMs(), drive->vehicleCount());
                        }
                        if (opt.film > 0) {
                            if (i == 0 && opt.camView > 0) chase.setView(opt.camView);
                            chaseView = chase.update(1.0f / 60, *drive, {}, cameraOverride(race.get()));
                            dust.update(1.0f / 60, *drive, drive->position());
                        }
                        if (streamer && (opt.propCheck || opt.film > 0 || i % 30 == 0)) streamer->update(drive->position(), renderer, drive->physics());
                        if (opt.film > 0) {  // worst step times since the last frame
                            static double worstPhys = 0, worstStream = 0;
                            worstPhys = std::max(worstPhys, (tp1 - tp0) * 1000.0);
                            worstStream = std::max(worstStream, (CACurrentMediaTime() - tp1) * 1000.0);
                            if (i % std::max(1, int(opt.film * 60)) == 0) {
                                std::printf("film timing: worst physics step %.1f ms, worst streaming %.1f ms\n", worstPhys, worstStream);
                                worstPhys = worstStream = 0;
                            }
                        }
                        if (opt.film > 0 && i % std::max(1, int(opt.film * 60)) == 0) {  // --film: OUT_0001.png ... from the chase camera
                            drive->uploadRiders();
                            renderer.setInstanceWorlds(drive->partWorlds());
                            showOpponents(*drive, renderer);
                            if (race) renderer.setMarkers(race->markers());
                            renderer.setFocus(chase.focus());
                            renderer.speedBlur = chase.speedEffect();
                            renderer.setDust(dust.particles());
                            Options frame = opt;
                            const fs::path out(opt.shot);
                            char suffix[16];
                            std::snprintf(suffix, sizeof suffix, "_%04d", i / std::max(1, int(opt.film * 60)));
                            frame.shot = (out.parent_path() / (out.stem().string() + suffix + out.extension().string())).string();
                            if (opt.inGame) {  // --in-game --film: the HUD (and, with FUEL_MENU_FILM=1, the pause menu) on the frames
                                if (!gOverlay)
                                    gOverlay = std::make_unique<fuel::ui::GameOverlay>(device, kColorFormat, argv[0], opt.dataDir, fuel::loadSettings(), opt.race,
                                                                                       opt.hub, opt.assemble);
                                if (std::getenv("FUEL_MENU_FILM") && !gOverlay->open()) gOverlay->openPause();
                                gShotHud = {};
                                gShotHud.speedKmh = std::fabs(drive->state().speedKmh);
                                gShotHud.odometerKm = float(i) / 60.0f * gShotHud.speedKmh / 3600.0f;
                                if (const fuel::Hub* h = fuel::nearestHub(gOverlay->menu().hubs(), drive->position().x, drive->position().z))
                                    gShotHud.hubName = gOverlay->menu().textOf(h->nameText);
                                if (race && race->racers.size() > 1) {
                                    gShotHud.race = true;
                                    const std::vector<int> pos = race->positions(*drive);
                                    for (int q = 1; q <= int(race->racers.size()); ++q)
                                        for (size_t k = 0; k < race->racers.size(); ++k)
                                            if (pos[k] == q) gShotHud.standings.push_back({race->racers[k].name, 0, k == 0});
                                }
                            }
                            const double gpuMs = renderShot(device, renderer, frame, chaseView);
                            const fuel::VehicleState fs = drive->state();
                            const simd_float3 fp = drive->position();
                            std::printf("film %s t=%.1f (%.0f, %.0f, %.0f) %.0f km/h %d wheels down, %zu dust, gpu %.1f ms%s\n", frame.shot.c_str(), i / 60.0, fp.x, fp.y, fp.z,
                                        fs.speedKmh, fs.wheelsOnGround, dust.particles().size(), gpuMs, race ? (" " + race->hud(*drive)).c_str() : "");
                            const auto& cp = chase.pose();
                            std::printf("  camera view %d eye (%.1f, %.1f, %.1f) target (%.1f, %.1f, %.1f) %.1f m, fov %.1f, roll %.3f, near %.2f\n", cp.view, cp.ex, cp.ey,
                                        cp.ez, cp.tx, cp.ty, cp.tz, std::hypot(std::hypot(cp.ex - fp.x, cp.ey - fp.y), cp.ez - fp.z), cp.fovDeg, cp.roll, cp.zNear);
                        }
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
                    showOpponents(*drive, renderer);
                    if (race) {
                        renderer.setMarkers(race->markers());
                        std::printf("%s\n", race->hud(*drive).c_str());
                        if (race->racers.size() > 1) std::printf("order after %.1f s of racing:\n%s", std::max(0.0, race->clock), race->standings(*drive).c_str());
                        if (opt.aiCheck) {
                            printAiCheck(*race, *drive);
                            gPhysStats.print();
                            return aiCheckVerdict(*race) ? 0 : 1;
                        }
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

        const NSSize windowSize = opt.settings ? gameWindowSize(playerSettings) : NSMakeSize(1280, 800);
        const NSRect frame = NSMakeRect(0, 0, windowSize.width, windowSize.height);
        NSWindow* window = [[NSWindow alloc] initWithContentRect:frame
                                                       styleMask:NSWindowStyleMaskTitled | NSWindowStyleMaskClosable |
                                                                 NSWindowStyleMaskResizable | NSWindowStyleMaskMiniaturizable
                                                         backing:NSBackingStoreBuffered
                                                           defer:NO];
        if (!opt.record.empty()) {
            if (!gRecorder.open(opt.record)) die("cannot write to " + opt.record);
            if (std::FILE* f = std::fopen((opt.record + "/command.txt").c_str(), "w")) {
                for (int i = 0; i < argc; ++i) std::fprintf(f, "%s%s", i ? " " : "", argv[i]);
                std::fprintf(f, "\n");
                std::fclose(f);
            }
            std::printf("recording to %s (press K to mark a glitch)\n", opt.record.c_str());
        }
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
        if (opt.inGame && drive) {
            gOverlay = std::make_unique<fuel::ui::GameOverlay>(device, kColorFormat, argv[0], opt.dataDir, playerSettings, opt.race, opt.hub, opt.assemble);
            for (NSView* sub in view.subviews)
                if ([sub isKindOfClass:[NSTextField class]]) sub.hidden = YES;  // the game's HUD pages replace the text line
        }
        [window center];
        [window makeKeyAndOrderFront:nil];
        [loadingWindow close];
        [window makeFirstResponder:view];
        [view showMesh:start];
        [NSApp activateIgnoringOtherApps:YES];
        if (opt.settings) applyWindowSettings(playerSettings, window, view);
        [NSApp run];
    }
    return 0;
}
