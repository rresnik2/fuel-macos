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
// --drive: FUEL's controls (P47, core/input_map.*; keys from the game's DftMap.map, pads from input.PC):
//   A/D steer, W/S lean fore/aft, Space accelerate, left Ctrl brake, keypad 0 or left Shift handbrake,
//   F respawn, arrows look around, keypad 1 or V camera view, Return GPS on/off, R pause menu, Esc pause;
//   pad: left stick steer + lean, RT/LT, X handbrake, Y respawn, right stick look, R3 view, B GPS, Back/Start.
//   settings.txt: controls = port (the earlier W/S/A/D + arrows layout), key.<ACTION> = keys, invert_camera_x/y.
//   Cut-scenes: TELEPORT_HELICO at a race start (Return / A skips), TR_DEATH on a respawn, VI_BIKE at the end
//   (FUEL_NO_RTC=1 turns them off; FUEL_RTC=NAME plays one from the first --film frame).

#import <Cocoa/Cocoa.h>
#import <ImageIO/ImageIO.h>
#import <Metal/Metal.h>
#import <MetalKit/MetalKit.h>
#import <UniformTypeIdentifiers/UniformTypeIdentifiers.h>

#import "renderer.h"
#import "drive.h"
#import "recorder.h"
#import "perf_check.h"  // P92
#import "pop_check.h"   // P94
#import "game_settings.h"
#import "pad_haptics.h"
#import "pad_triggers.h"  // P91
#include "core/custom_vehicles.hpp"  // CUSTOM VEHICLES
#include "core/upgrades.hpp"  // [upgrades U1]
#include "viewer/pause_backdrop.hpp"  // PK17
#include "ui/game_menu.h"
#include "ui/menu_input.h"  // [port] remap
#include "ui/boot_screens.h"  // PK26
#include "ui/help_texts.hpp"   // PK26
#include "ui/pad_glyphs.h"     // pad-ux
#include "ui/menu_check.hpp"
#include "ui/career_audit.h"
#include "rider_view.h"
#include "eventcheck.h"
#include "hubcheck.h"
#include "freeroam_view.h"
#include "traffic_view.h"
#include "storm_view.h"
#include "damage_view.h"  // P85
#include <deque>          // P87
#include <unordered_map>  // P87
#include "marker_view.h"
#include "capture.h"
#include "rendercheck.h"
#include "core/terrain_height.hpp"
#include "core/hubs.hpp"
#import <GameController/GameController.h>
#include "audio/drive_audio.h"
#include "audio/menu_audio.h"
#include "core/sound.hpp"
#include "core/career.hpp"
#include "core/events.hpp"
#include "core/race_editor.hpp"  // PK21
#include "core/race_rules.hpp"
#include "core/races.hpp"
#include "physics/race_ai.hpp"
#include "physics/ai_nitro.hpp"  // [nitro]
#include "audio/nitro_sound.hpp"  // [nitro]
#include "physics/atv_path.hpp"
#include "core/road_types.hpp"
#include "core/roads.hpp"
#include "core/road_graph.hpp"
#include "core/hub_zones.hpp"
#include "physics/camcheck.hpp"
#include "physics/breadcrumbs.hpp"  // PK17: World_IsOutside 0x5280a0
#include "core/input_map.hpp"
#include "core/input_check.hpp"
#include "core/anim_check.hpp"
#include "anim_view.h"
#include "heli_view.h"
#include "f14_view.h"  // PK27
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
#include <atomic>
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

#include "core/particles.hpp"

// p48: FUEL's placed engine particles (Particles_Z), stepped near the camera and handed to the renderer.
static fuel::particles::PlacedSystem gPlacedFx;
// cr5: the race's AI level (race manager +0x3e8): `--ai-level N` from the menu (the career page's medal row, a challenge 0);
// -1 = not given (see raceAiLevel).
static int gAiLevelArg = -1;
// PK13: the running RTC's node FX (TELEPORT_HELICO rotor dust, 0x539140); defined after gRtc.
static bool rtcHasFx();
static void appendRtcFx(std::vector<fuel::particles::Quad>& quads, std::vector<fuel::particles::Batch>& batches, simd_float3 eye);
// PK29: the world animation's event FX (IT_ANIMATED message 0x20 -> GameParticleMgr 0x4ee870); defined after gWorldAnim.
static bool worldAnimHasFx();
static void appendWorldAnimFx(std::vector<fuel::particles::Quad>& quads, std::vector<fuel::particles::Batch>& batches);
static void updatePlacedFx(float dt, simd_float3 eye, fuel::MeshRenderer& r) {
    if (gPlacedFx.placements() == 0 && !gDamageView.smoking() && !rtcHasFx() && !worldAnimHasFx()) return;  // P85: or the damage smoke; PK13: or RTC FX; PK29
    const fuel::LodView view;  // K 10, view scale tan 20 / tan 42.5 (p36)
    gPlacedFx.update(dt, {eye.x, eye.y, eye.z}, view.viewScale, view.Kfade);
    static std::vector<fuel::particles::Quad> quads;
    static std::vector<fuel::particles::Batch> batches;
    gPlacedFx.collect(quads, batches);
    gDamageView.appendSmoke(quads, batches);  // P85: the damage smoke (FX_CAR_Damage01 / 03)
    appendRtcFx(quads, batches, eye);         // PK13
    appendWorldAnimFx(quads, batches);        // PK29
    std::vector<fuel::MeshRenderer::EngineParticle> q;
    q.reserve(quads.size());
    for (const auto& p : quads)
        q.push_back({simd_make_float4(p.pos[0], p.pos[1], p.pos[2], p.angle), simd_make_float4(p.sx, p.sy, float(p.orient), 0),
                     simd_make_float4(p.rgba[0], p.rgba[1], p.rgba[2], p.rgba[3])});
    std::vector<fuel::MeshRenderer::EngineBatch> b;
    for (const auto& x : batches) b.push_back({x.material, x.first, x.count});
    if (std::getenv("FUEL_SFX_DEBUG") && !quads.empty()) {
        static int n = 0;
        if (++n % 60 == 0) std::fprintf(stderr, "p48 placed particles: %zu active emitters, %zu quads, %zu batches\n", gPlacedFx.active(), q.size(), b.size());
    }
    r.setEngineParticles(std::move(q), std::move(b));
}

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
std::unique_ptr<TrafficView> gTraffic;  // P58: free-roam road traffic (TrafficManipulator_G 0x5c3010)
std::unique_ptr<FreeRoamView> gFreeRoam;  // P44: free-roam rules (vistas, liveries, barrels, stats) with --in-game
// ---- PK03 (PK15): the teleport-queue landing is tlpLand below (0x57e0f0 state 3 + 0x42bd30).
// ---- PK03: the session was started by TeleportToHub (--hub without --at): its queued StartHubHUD runs (0x5492d0).
static bool gHubTravel = false;
// ---- end PK03
// ---- PK26: the career boot's TeleportToHub (Dialog_Loading 0x58fc20 -> 0x590300): the hub it queued (-1 none) and whether
// its AddCommandAfterTeleport GAMEStarted is still pending.
static int gBootHub = -1;
static bool gBootStartedPending = false;
// ---- end PK26
// ---- PK15: the teleport settle. LevelManipulator_G's teleport record and machine around the logo stencil's timers
// (fuel::ui::LogoTransition = +0x50 frames, +0x54 p, +0x58 q, +0x5c state, +0x7c the 1 s idle timer):
//   Teleport_Queue 0x57e650: +0x4c "hop" = the target is less than 20 m from the player and GenWorld +0x330 (a
//     regeneration under way) is clear; otherwise the player and the camera go to the target at once (0x57ede0 ->
//     0x57eb50) and the world is regenerated around it (0x640e20, +0x330 = 1).
//   state 1 -> 2 (60 frames): 0x57eeb0: unless a hop, the full regeneration 0x640fe0 (0x640cf0 + GenWorld restart
//     0x74f770: ground, plants, grass around the target, a 3-frame settle) [port: WorldStreamer::settle at the target
//     and the renderer's terrain chunks built before its next frame]. A hop is placed in state 2 (0x57eb50 every frame).
//   state 2: >= 156 frames, GenWorld settled (0x57f050 clears +0x330), +0x5b4b4 == 0 and the pool streamer idle
//     (!0x748ef0), then 1.0 s of that -> 3 [port: tlpReady = WorldStreamer::idle and MeshRenderer::terrainIdle].
//   entering 3: the player on the ground + 0.1 m with the record's heading, its reset vt+0x50 (tlpLand: the vehicle
//     reset and PK01's placeOnNode), then the queued console lines (0x57e5f0: StartHubHUD <uid> of TeleportToHub).
// 0x57b9a0 (the one-shot placement at game+0x360 + 0.1 m when the player's vehicle is entered) is not ported: no
// writer of game+0x360 exists in the exe (decompile and code scan), so the vector stays zero and FUEL never runs it.
struct TeleportSettle {
    bool pending = false;  // a record waits for state 3 (+0x9c..)
    bool hop = false;      // +0x4c
    bool regenerating = false;  // GenWorld +0x330
    double x = 0, z = 0;
    float yaw = 0;
    int afterHub = -1;     // StartHubHUD <hub> queued after the teleport (0x57e5f0)
    int state = 0;         // the machine's state seen last frame
};
static TeleportSettle gTlp;
// The landing (was PK03's hubPlace): 0x41b2f0 + the player's reset vt+0x50, whose vehicle re-placement (Player_G
// vf137 -> body vf117 0x713e80) re-inits the vehicle (vf14: wheels, engine, contacts) before posing it at rest: the
// vehicle reset (DriveWorld::placeVehicle, VehMove_ResetTo), then the node pose (PK01 placeOnNode: 0.3 m above it).
static void tlpLand(DriveSession& d, double x, double z, float yaw) {
    d.physics().placeVehicle(0, x, z, yaw);  // vf14 re-init (VehMove_ResetTo)
    // 0x6407e0 + 0.1, then the mover's ground re-place (+0x5fd from 0x41b2f0: VehMove_GroundHeightProbe 0x529600, a
    // scene ray from y + 0.25 down 100 m, the higher of its hit and the ground).
    double y = d.terrain().smoothHeightAt(x, z) + 0.1;
    const float floor = d.physics().staticFloorBelow(x, y + 0.25, z, 100.0f);
    if (!std::isnan(floor) && floor > y) y = floor;
    d.physics().placeOnNode(0, {x, y, z}, {0, float(std::sin(yaw * 0.5)), 0, float(std::cos(yaw * 0.5))});
}
static bool tlpReady(const WorldStreamer* s, const fuel::MeshRenderer& r) { return (s == nullptr || s->idle()) && r.terrainIdle(); }
// Teleport_Queue 0x57e650 (the caller has started the logo: LogoTransition::start).
static void tlpQueue(TeleportSettle& t, DriveSession& d, WorldStreamer* s, fuel::MeshRenderer& r, double x, double z, float yaw, int afterHub = -1) {
    const simd_float3 at = d.position();
    t = {true, std::hypot(x - at.x, z - at.z) < 20.0 && !t.regenerating, t.regenerating, x, z, yaw, afterHub, 1};
    if (t.hop) return;
    tlpLand(d, x, z, yaw);  // 0x57ede0 -> 0x57eb50
    r.resetTemporalHistory();  // TAA: a teleport is a cut
    if (s) s->update(d.position(), r, d.physics(), true);  // 0x640e20: regenerated around the target now
    t.regenerating = true;
}
// 0x57e0f0's actions on the logo machine's state changes (call every frame after LogoTransition::update).
static void tlpStep(TeleportSettle& t, int state, DriveSession& d, WorldStreamer* s, fuel::MeshRenderer& r, const std::function<void(int)>& startHubHud) {
    const int was = std::exchange(t.state, state);
    if (state == was) return;
    if (state == 2 && was == 1) {  // 0x57eeb0 (a race start's teleport too: the grid is its target)
        if (t.pending && t.hop) {
            tlpLand(d, t.x, t.z, t.yaw);  // 0x57eb50
            r.resetTemporalHistory();     // TAA
        } else {
            if (s) s->settle(d.position(), r, d.physics());  // 0x640fe0
            r.settleTerrain();
            t.regenerating = true;
        }
    } else if (state == 3) {
        t.regenerating = false;  // 0x57f050
        if (!t.pending) return;
        t.pending = false;
        tlpLand(d, t.x, t.z, t.yaw);  // ground + 0.1 (0x6407e0), 0x41b2f0, vt+0x50
        r.resetTemporalHistory();     // TAA
        if (t.afterHub >= 0 && startHubHud) startHubHud(t.afterHub);
    } else if (state == 0) {
        t.pending = false, t.regenerating = false;
    }
}
// ---- end PK15
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
    bool hapticsCheck = false;  // --haptics-check: FUEL's pad vibration rules and motors over a drive (P89)
    bool triggerCheck = false;  // --trigger-check: P91's DualSense adaptive triggers over a scripted drive
    bool propCheck = false;
    bool retryCheck = false;  // PK14 --retry-check: knock props, retry the race (KillMission + TTMN), props back in place
    std::string race;
    int opponents = -1;    // --race: AI opponents on the grid (-1: the event's EMD_SetNbIA)
    bool aiCheck = false;  // --ai-check: race headless and report the opponents' progress
    std::string aiPlayer;  // --ai-player stop|launch (P97): the player for --ai-check (default the autopilot)
    float driveTime = 4.0f;
    double timeOfDay = -1;     // --time HH.MM: FUEL's ForceDayTime (seconds of the day); < 0 the running clock
    float cloud = 0, fog = 0, wet = 0;  // --weather C,F,W: inputs of World_SetWeatherAtmosphere (0..1)
    bool weatherGiven = false;          // else the world's weather fields at the camera (core/weather.hpp)
    bool renderCheck = false;  // --rendercheck: the renderer's test vectors (no game data needed)
    bool haveAt = false;
    bool noHubArrival = false;  // --no-hub-arrival: a free ride (0x49b190), not TeleportToHub: no StartHubHUD
    bool careerBoot = false;    // PK26 --career-boot: the boot screens' PlayLevel USA1 (Dialog_Loading 0x58fc20 runs the boot order)
    float atX = 0, atZ = 0, atYaw = 0;
    std::string hub;       // --hub NAME: start at that hub's camp (free roam)
    bool eventCheck = false;  // --event-check [--event-only ID|HUB]: run every event headless and report
    std::string eventOnly;
    bool hubCheck = false;  // --hub-check: spawn at every hub point and report
    bool collisionCheck = false;  // --collision-check (P50)
    std::string assemble;
    int lod = 1;
    int width = 1600, height = 1000;
    float yawDeg = 37.0f, pitchDeg = 17.0f;
    bool haveYaw = false;  // --yaw given: with --film, the chase camera looks from that angle (0 = behind)
    float distance = 0;
    bool haveFocus = false;  // CARPOLISH: --focus X,Y,Z (vehicle space): the shot camera orbits this point
    simd_float3 focus = {0, 0, 0};
    bool cull = true;
    float dirt = 0.0f;
    std::string vehicle;
    int livery = 1;
    int color = 0;
    std::string pilot;  // P66: --pilot head,chest,legs,headLiv,chestLiv,legsLiv,bodyLiv,skin (the garage's rider)
    std::string outfit;  // PHYSIQUE: --outfit N[,H] (0 FUEL's rider, 1 riding gear, 2 tank top + cutoffs, 3 shirtless + shorts; H helmet 0/1)
    std::string jiggle;  // PHYSIQUE: --jiggle off|subtle|full (the glute springs; else settings.txt rider_jiggle)
    bool havePaint = false;
    int paint[3] = {60, 60, 60};
    bool noRider = false;
    float film = 0;      // --film S: with --shot, a chase-camera frame every S seconds of the drive
    std::string record;  // --record DIR: frame log and grabs of a window session
    bool game = false;      // --game: FUEL's menus (title, top menu, lists, options); sessions run in a new process
    bool inGame = false;    // --in-game: a session started from the menus (pause/results menu, HUD)
    std::string menuShot, menuShotOut, menuPage;  // --menu-shot PAGE OUT.png, --menu-page PAGE
    bool menuCheck = false; // --menu-check
    bool careerCheck = false;  // --career-check: a scripted career against the decoded rules (P39)
    bool careerAudit = false;  // --career-audit: a whole career through the menus and the race rules (P99)
    std::string upgradeCmd;    // [upgrades U2] --upgrade-cmd "buy V C S; equip ...; list; session V [EVENT]" (game_menu.h)
    bool hudCheck = false;  // --hudcheck: FUEL's HUD maths against emulated / code vectors (p40)
    bool animCheck = false;   // --animcheck: FUEL's animation samplers, RTC files and playback (P47)
    bool inputCheck = false;  // --inputcheck: FUEL's input records, sampler and bindings (P47)
    bool camCheck = false;  // --camcheck: FUEL's camera and rider functions against emulated test vectors
    bool aiVectors = false;  // --aicheck-vectors: the AI functions against emulated FUEL.exe vectors (P29)
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
    // P92: --perf-check: a race film rendered offscreen every frame, timed per pass (perf_check.h).
    bool perfCheck = false, perfBreakdown = false;
    double perfBudget = 13.0;
    int perfInFlight = 1;
    std::string perfCsv;
    double perfPace = 0;        // P93: > 0: frames start no faster than this rate (Hz), like the window's vsync
    std::string perfAlternate;  // P93 (perf_check.h Options::alternate)
    // P93: dynamic resolution and the shadow filter (the game takes them from settings.txt; --perf-check from the
    // game's defaults, on and 2x2): -1 = not given.
    int dynres = -1, shadow2x2 = -1;
    float dynresMin = -1, renderScale = 0;
    std::string upscaler;
    // P94: --pop-check: the same race film, unrendered, with every frame's draw decisions compared (pop_check.h).
    bool popCheck = false;
    float popPixels = 3.0f;
    std::string popCsv;
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
        else if (a == "--time") {  // ForceDayTime HH.MM (0x4d64d0): H 0..23, M 0..59
            const std::string v = next();
            int h = 0, m = 0;
            if (std::sscanf(v.c_str(), "%d.%d", &h, &m) < 1) die("bad --time (use HH.MM)");
            o.timeOfDay = (std::clamp(h, 0, 23) * 60 + std::clamp(m, 0, 59)) * 60.0;
        }
        else if (a == "--weather") {
            if (std::sscanf(next().c_str(), "%f,%f,%f", &o.cloud, &o.fog, &o.wet) < 1) die("bad --weather (use CLOUD,FOG,WET in 0..1)");
            o.weatherGiven = true;
        }
        else if (a == "--rendercheck") o.renderCheck = true;
        else if (a == "--mesh") { o.meshId = uint32_t(std::stoul(next(), nullptr, 16)); o.haveMeshId = true; }
        else if (a == "--shot") o.shot = next();
        else if (a == "--scene") o.scene = next();
        else if (a == "--world") o.world = true;
        else if (a == "--terrain") o.terrain = true;
        else if (a == "--drive") { o.drive = true; o.terrain = true; }
        else if (a == "--audio-check") o.audioCheck = true;
        else if (a == "--haptics-check") o.hapticsCheck = true;
        else if (a == "--trigger-check") o.triggerCheck = true;
        else if (a == "--prop-check") { o.propCheck = true; o.drive = true; o.terrain = true; }
        else if (a == "--retry-check") { o.retryCheck = true; o.drive = true; o.terrain = true; }  // PK14
        else if (a == "--drive-time") o.driveTime = std::stof(next());
        else if (a == "--race") { o.race = next(); o.drive = true; o.terrain = true; }
        else if (a == "--opponents") o.opponents = std::clamp(std::stoi(next()), 0, 11);
        else if (a == "--ai-check") o.aiCheck = true;
        else if (a == "--ai-player") {
            o.aiPlayer = next();
            if (o.aiPlayer != "stop" && o.aiPlayer != "launch" && o.aiPlayer != "bot") die("--ai-player takes stop, launch or bot");
        }
        else if (a == "--at") {  // X,Z[,YAW degrees] (P84: optional heading, atan2(dx, dz))
            const std::string v = next();
            float yawDeg = 0;
            const int n = std::sscanf(v.c_str(), "%f,%f,%f", &o.atX, &o.atZ, &yawDeg);
            if (n < 2) die("bad --at (use X,Z[,YAW] in metres / degrees)");
            if (n == 3) o.atYaw = yawDeg * float(M_PI) / 180.0f;
            o.haveAt = true, o.terrain = true;
        }
        else if (a == "--hub") { o.hub = next(); o.terrain = true; }
        else if (a == "--no-hub-arrival") o.noHubArrival = true;
        else if (a == "--career-boot") o.careerBoot = true;  // PK26
        else if (a == "--event-check") { o.eventCheck = true; o.drive = true; o.terrain = true; }
        else if (a == "--event-only") o.eventOnly = next();
        else if (a == "--hub-check") { o.hubCheck = true; o.drive = true; o.terrain = true; }
        else if (a == "--collision-check") { o.hubCheck = o.collisionCheck = true; o.drive = true; o.terrain = true; }
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
        else if (a == "--focus") {  // CARPOLISH: framed close-ups of a vehicle (lamps, glass): vehicle-space point
            const std::string v = next();
            float fx = 0, fy = 0, fz = 0;
            if (std::sscanf(v.c_str(), "%f,%f,%f", &fx, &fy, &fz) != 3) die("bad --focus (use X,Y,Z in metres)");
            o.focus = simd_make_float3(fx, fy, fz), o.haveFocus = true;
        }
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
        else if (a == "--career-check") o.careerCheck = true;
        else if (a == "--career-audit") o.careerAudit = true;
        else if (a == "--upgrade-cmd") o.upgradeCmd = next();  // [upgrades U2]
        else if (a == "--hudcheck") o.hudCheck = true;
        else if (a == "--camcheck") o.camCheck = true;
        else if (a == "--inputcheck") o.inputCheck = true;
        else if (a == "--animcheck") o.animCheck = true;
        else if (a == "--aicheck-vectors") o.aiVectors = true;
        else if (a == "--cam-view") o.camView = std::stoi(next());
        else if (a == "--drop") { const std::string v = next(); if (std::sscanf(v.c_str(), "%f,%f", &o.dropAt, &o.dropHeight) != 2) die("bad --drop (use T,H)"); }
        else if (a == "--film") o.film = std::stof(next());
        else if (a == "--perf-check") { o.perfCheck = true; o.drive = true; o.terrain = true; }  // P92
        else if (a == "--perf-budget") o.perfBudget = std::stod(next());
        else if (a == "--perf-breakdown") o.perfBreakdown = true;
        else if (a == "--perf-csv") o.perfCsv = next();
        else if (a == "--perf-inflight") o.perfInFlight = std::stoi(next());
        else if (a == "--perf-pace") o.perfPace = std::stod(next());  // P93
        else if (a == "--perf-alternate") o.perfAlternate = next();   // P93
        else if (a == "--dynres") { const std::string v = next(); o.dynres = v == "on" || v == "1" ? 1 : v == "off" || v == "0" ? 0 : (die("bad --dynres (on|off)"), 0); }
        else if (a == "--dynres-min") o.dynresMin = std::stof(next());
        else if (a == "--render-scale") o.renderScale = std::stof(next());  // P93: a fixed scale (quality comparisons)
        else if (a == "--upscaler") o.upscaler = next();                    // P93: metalfx | sharpen (TAA: | metalfx-temporal)
        else if (a == "--shadow-filter") { const std::string v = next(); o.shadow2x2 = v == "2x2" ? 1 : v == "3x3" ? 0 : (die("bad --shadow-filter (2x2|3x3)"), 0); }
        else if (a == "--pop-check") { o.popCheck = true; o.drive = true; o.terrain = true; }  // P94
        else if (a == "--pop-pixels") o.popPixels = std::stof(next());
        else if (a == "--pop-csv") o.popCsv = next();
        else if (a == "--vehicle") o.vehicle = next();
        else if (a == "--livery") o.livery = std::stoi(next());
        else if (a == "--pilot") o.pilot = next();
        else if (a == "--outfit") o.outfit = next();  // PHYSIQUE
        else if (a == "--upgrades") {  // [upgrades U1] the driven vehicle's stages e,s,t,w,np,nt (FUEL_UPGRADES overrides)
            fuel::upg::Stages st;
            std::string err;
            if (!fuel::upg::parseStages(next(), st, &err)) die("bad --upgrades (e,s,t,w,np,nt, each 0..3): " + err);
            fuel::upg::setPlayerStages(st);
        }
        else if (a == "--ai-level") {  // cr5: the race's AI level 0..2 (the medal raced for; FUEL_AI_LEVEL overrides)
            const std::string v = next();
            if (v.size() != 1 || v[0] < '0' || v[0] > '2') die("bad --ai-level (0 gold, 1 silver, 2 bronze)");
            gAiLevelArg = v[0] - '0';
        }
        else if (a == "--ai-upgrade-tier") {  // [upgrades U4] the race's hub tier for the opponents' stages (FUEL_AI_UPGRADE_TIER overrides)
            int t = 0;
            if (!fuel::upg::parseAiTier(next(), t)) die("bad --ai-upgrade-tier (a hub tier 0..9, -1 / none for no hub, auto)");
            fuel::upg::setAiHubTier(t);
        }
        else if (a == "--jiggle") o.jiggle = next();  // PHYSIQUE: off | subtle | full (else settings.txt rider_jiggle)
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
    if (o.retryCheck) {                                // PK14: headless, the player alone, a race to retry
        if (o.race.empty()) o.race = "CR3-Yell-CT";    // props 60 m from its start (CR2-Yell-CP's start has none)
        if (o.shot.empty()) o.shot = "-";
        o.opponents = 0;
    }
    if (o.triggerCheck) {  // P91: the scripted drive follows CR2-Yell-CP's line, alone
        if (o.race.empty()) o.race = "CR2-Yell-CP", o.drive = o.terrain = true;
        if (o.opponents < 0) o.opponents = 0;
    }
    if (o.popCheck) {  // P94: the perf-check race, every frame's draw decisions (nothing rendered)
        if (o.perfCheck || o.inGame || o.game) die("--pop-check runs alone (no --perf-check / --in-game / --game)");
        if (o.race.empty() && !o.haveAt) o.race = "CR2-Yell-CP";
        if (o.assemble.empty()) {
            o.assemble = "BUGGY_OFF_3";
            if (o.dpcs.empty()) o.dpcs.push_back("CommonComponent/DATAS/BUGGY.DPC");
        }
        if (!o.haveSize) o.width = 3420, o.height = 2224;
        if (o.driveTime <= 4.0f) o.driveTime = 100.0f;
        o.film = 1.0f / 60.0f;
        o.shot = "-";
    }
    if (o.perfCheck) {  // P92: the player's race at the player's fullscreen size, every frame rendered offscreen
        if (o.inGame || o.game) die("--perf-check runs without the menus (no --in-game / --game)");
        if (o.race.empty() && !o.haveAt) o.race = "CR2-Yell-CP";
        if (o.assemble.empty()) {
            o.assemble = "BUGGY_OFF_3";  // the user's recorded race (BUGGY.DPC)
            if (o.dpcs.empty()) o.dpcs.push_back("CommonComponent/DATAS/BUGGY.DPC");
        }
        if (!o.haveSize) o.width = 3420, o.height = 2224;
        if (o.driveTime <= 4.0f) o.driveTime = 100.0f;
        o.film = 1.0f / 60.0f;  // the film's per-frame path (camera, cut-scenes, effects), every frame
        o.shot = "-";
    }
    if (o.aiCheck) {
        if (o.race.empty()) die("--ai-check needs --race NAME");
        if (o.shot.empty()) o.shot = "-";
        if (o.driveTime <= 4.0f) o.driveTime = 120.0f;
    }
    if (o.dataDir.empty() && o.settings) o.dataDir = fuel::loadSettings().dataDir;
    if (o.dataDir.empty()) o.dataDir = defaultDataDir();
    fuel::custom::setDataDir(o.dataDir);  // CUSTOM VEHICLES
    if (o.dpcs.empty() && !o.assemble.empty())  // CUSTOM VEHICLES: a pack's vehicle is in the pack's own archive
        if (const fuel::custom::Vehicle* cv = fuel::custom::find(o.assemble)) o.dpcs.push_back(cv->dpc);
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
    if (!fs::exists(tsc)) return {};
    return fuel::parseVehicleColors(fuel::custom::readScript(tsc.string()));  // CUSTOM VEHICLES: packs appended
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
            const fuel::AssembledVehicle av = assembler->assemble(variant(), lod);
            SceneFile sf = sceneFromAssembly(av, variant().label);
            if (riders)
                for (uint32_t id : riders->parts(variant().label, RiderView::seat(av, assembler->dpc())))
                    sf.parts.push_back({id, matrix_identity_float4x4});
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

// [upgrades U5] --assemble without --drive (shots): the stages' visible upgrades at rest (core/upgrade_visuals restScene: the
// physics' rest stance on flat ground, the body `lift` higher, the wheels out by the widening), the rider lifted with the body.
void applyStaticUpgrades(SceneFile& sf, const AssemblyMode& am, const std::string& dataDir) {
    if (fuel::upg::playerStages().stock() || !fuel::upg::enabled() || !fuel::upgvis::enabled()) return;
    const auto params = fuel::parseVehiclesParam(fuel::custom::readScript((fs::path(dataDir) / "CommonComponent/Param/VehiclesParam.tsc").string()));
    const std::string name = am.variant().label;
    const auto it = params.find(name);
    if (it == params.end()) return;
    try {
        const fuel::AssembledVehicle av = am.assembler->assemble(am.variant(), am.lod);
        fuel::VehicleSetup setup = fuel::makeVehicleSetup(am.assembler->dpc(), name, it->second, assembledHubs(av));
        fuel::upg::applyPlayer(setup, fuel::fv::FuelVehicle::powerScale);
        const fuel::upgvis::Plan plan = fuel::upgvis::makePlan(av, setup);
        std::printf("upgrade visuals: %s: %s\n", name.c_str(), fuel::upgvis::describe(plan).c_str());
        if (std::getenv("FUEL_U5_TRACE"))
            for (const std::string& n : plan.notes) std::printf("u5:   %s\n", n.c_str());
        if (!plan.active) return;
        size_t drawn = 0;
        for (const fuel::PlacedPart& p : av.parts) drawn += p.drawn() ? 1 : 0;
        std::vector<fuel::MeshInstance> parts;
        for (const fuel::upgvis::ScenePart& sp : fuel::upgvis::restScene(av, setup, plan)) {
            float m[16];
            for (int i = 0; i < 16; ++i) m[i] = float(sp.world[size_t(i)]);
            parts.push_back({sp.mesh, matrixFromRows(m)});
        }
        for (size_t k = drawn; k < sf.parts.size(); ++k) {  // the rider rides on the lifted body
            fuel::MeshInstance r = sf.parts[k];
            r.world.columns[3].y += plan.lift;
            parts.push_back(r);
        }
        sf.parts = std::move(parts);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "fuelview: upgrade visuals: %s: %s\n", name.c_str(), e.what());
    }
}

// --drive: every drivable vehicle in the vehicle archives, for Tab switching.
// The ground maps and roads behind the physics' per-wheel surface query (drive mode).
static fuel::GroundMaps gGround;
// FUEL's point colour (the decoded ground map, gamma bytes as stored) at (x, z), for the wheel effects.
static std::array<float, 4> groundPointColour(const fuel::GroundMaps& g, double x, double z) {
    if (g.size == 0 || g.colour.empty()) return {0.5f, 0.5f, 0.5f, 1.0f};
    const double step = 140000.0 / double(g.size);
    auto idx = [&](double v) { return size_t(std::min(double(g.size - 1), std::max(0.0, std::floor((v + 70000.0) / step)))); };
    const uint8_t* c = &g.colour[(idx(z) * g.size + idx(x)) * 4];
    return {c[0] / 255.0f, c[1] / 255.0f, c[2] / 255.0f, 1.0f};
}
static fuel::SurfaceMap gSurfaces;
static fuel::HubZones gHubZones;  // P75: GenWorld_Z zone tail (Map page)
static std::atomic<const fuel::GroundGenerator*> gGroundGen{nullptr};
static const fuel::DpcFile* gWorldDpc = nullptr;  // USA1.DPC: the ambience's IT_SOUND bank, the music library (p45)  // for the tyres' class fractions (p32)

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
    // `lods` (P104): also the lower levels of a non-player vehicle and its rider (VehicleLods).
    bool load(size_t i, fuel::AssembledVehicle& av, fuel::VehicleSetup& setup, std::vector<uint32_t>& rider, VehicleLods* lods = nullptr) const {
        AssemblyMode& am = *archives[entries[i].first];
        am.index = entries[i].second;
        const std::string name = am.variant().label;
        try {
            const auto it = params.find(name);
            if (it == params.end()) throw std::runtime_error("no driving parameters");
            av = am.assembler->assemble(am.variant(), am.lod);
            setup = fuel::makeVehicleSetup(am.assembler->dpc(), name, it->second, assembledHubs(av));
            // ---- P104: the variant's LOD2..n; LodData flag 0x400000 (never LOD1) on every non-player vehicle but the
            // FLOTTE fleet (0x5c7fe0 asm 0x5c857d..0x5c85a9: MESH_ID_FLOTTE_BIGTRUCK_1 / MESH_ID_FLOTTE_JEEP_1).
            if (lods != nullptr && vehicleLodEnabled()) {
                *lods = VehicleLods{};
                lods->minLevel = (name == "FLOTTE_BIGTRUCK_1" || name == "FLOTTE_JEEP_1") ? 0 : 1;
                for (int k = 2; k <= av.lodLevels; ++k) lods->lower.push_back(am.assembler->assemble(am.variant(), k));
            }
            // P104: the rider's IK_* points come from the parts of the vehicle's view-0 LOD (0x6a23b0: LOD2 with 0x400000).
            const fuel::AssembledVehicle& ikFrom = lods != nullptr && lods->minLevel == 1 && !lods->lower.empty() ? lods->lower[0] : av;
            if (std::getenv("FUEL_P104_TRACE")) {
                const auto a = fuel::riderIkPoints(am.assembler->dpc(), av), b = fuel::riderIkPoints(am.assembler->dpc(), ikFrom);
                for (int k = 0; k < 4; ++k)
                    std::printf("p104: %s IK %d LOD1 (%.3f %.3f %.3f) used (%.3f %.3f %.3f)\n", name.c_str(), k, a[k][0], a[k][1], a[k][2], b[k][0], b[k][1], b[k][2]);
            }
            rider = riders ? riders->animated(name, ikFrom, am.assembler->dpc(), setup.params, lods == nullptr) : std::vector<uint32_t>{};  // PHYSIQUE: the player
            if (lods != nullptr && vehicleLodEnabled() && riders != nullptr) {
                if (rider.size() == 1) lods->riderLower = riders->lowerLevels(rider[0]);
                lods->riderLevels = riders->lodLevels();
                lods->riderD = riders->lodBaseDistance(), lods->riderView = riders->lodViewDist();
                lods->seat = RiderView::seat(av, am.assembler->dpc());
            }
            // ---- P104 end
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
        auto split = [](const std::string& s, int k) {
            size_t a = 0;
            for (int j = 0; j < k && a != std::string::npos; ++j) a = s.find('_', a) == std::string::npos ? std::string::npos : s.find('_', a) + 1;
            return a == std::string::npos ? std::string() : s.substr(a, s.find('_', a) - a);
        };
        // CUSTOM VEHICLES: a pack's vehicle counts as its AddVehicleInfo class / surface ("MUSCLE", "ROAD"); opponents
        // never get one unless its pack allows it (PortVehicleAI 1).
        auto field = [&](const std::string& s, int k) {
            if (const fuel::custom::Vehicle* cv = fuel::custom::find(s)) return k == 0 ? cv->family : k == 1 ? cv->surface : std::string();
            return split(s, k);
        };
        std::vector<size_t> same, kin;
        for (size_t i = 0; i < entries.size(); ++i) {
            if (const fuel::custom::Vehicle* cv = fuel::custom::find(label(i)); cv && !cv->ai) continue;
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

// P97: 0x821330 + 0x821c20 (from 0x5d3300): a sphere against the game camera's view frustum (near plane, the four
// side planes; the far plane is FUEL's draw distance, never nearer than the 150 m that matter here). `aspect` =
// the frame's width / height (Hor+ from the 4:3 FOV, as the renderer draws). Cameras without an explicit pose
// (none yet) see everything.
bool sphereInView(const fuel::OrbitCamera& cam, float aspect, simd_float3 c, float r) {
    if (!cam.explicitPose) return true;
    const simd_float3 d = cam.target - cam.eye;
    if (simd_length_squared(d) < 1e-8f) return true;
    const simd_float3 f = simd_normalize(d);
    simd_float3 rt = simd_cross(f, simd_make_float3(0, 1, 0));
    if (simd_length_squared(rt) < 1e-8f) return true;
    rt = simd_normalize(rt);
    const simd_float3 up0 = simd_cross(rt, f);
    const simd_float3 up = up0 * std::cos(cam.roll) + rt * std::sin(-cam.roll), right = simd_cross(f, up);
    const float fovY = cam.fovH43 > 0 ? 2.0f * std::atan(std::tan(cam.fovH43 * 0.5f) * 0.75f) : cam.fovY;
    const float ty = std::tan(fovY * 0.5f), tx = ty * aspect;
    const simd_float3 v = c - cam.eye;
    const float z = simd_dot(v, f), x = simd_dot(v, right), y = simd_dot(v, up);
    if (z + r < std::max(cam.zNear, 0.06f)) return false;
    // Side planes through the eye: n = (cos a, -sin a) in (x, z) with tan a = tx; inside when n . p >= -r.
    const float cx = 1.0f / std::sqrt(1 + tx * tx), cy = 1.0f / std::sqrt(1 + ty * ty);
    if ((x - z * tx) * cx > r || (-x - z * tx) * cx > r) return false;
    if ((y - z * ty) * cy > r || (-y - z * ty) * cy > r) return false;
    return true;
}

// --race: FUEL's events (start line, ordered checkpoints, finish, laps; the rules of each game
// mode come from fuel::RaceRules), with AI opponents (vehicles 1.. of the drive session) on the
// grid behind the player. Hunted racers of "catch AI" challenges are opponents that follow the
// event's path; the helicopter of chopper races has no vehicle (a beacon marks it).
// cr5: the race's AI level, race manager +0x3e8 (notes cr5-ai.md) [O]. Race_Start 0x62e650 stores its 4th argument there
// (asm 0x62e7a1); its callers are RestartMission 0x549130 (console, 2nd argument; without one it keeps the last level) and
// the edited-race arch start 0x4f6fd0 (0). The menus start races with TTMN "<mission>" <index> <level> (0x54a910 ->
// "RestartMission %d %d"): the career page 0x47eee0 passes its medal row +0x88 (0x46b970 preselects best rank - 1, at
// least 0: bronze = 2 for a race not won yet; Up / Down 0..2), the challenge page 0x47efb0 passes 0. 0x617aa0 then reads
// AddMissionTime value `level` (mission +0x80 + 4 level) as the AI's target time, and 0x616d70 the level's near-player
// rubber band (S+0x2c, S+0x30). The port used level 1 for every race: a challenge (AddMissionTime "T 0 0") raced to
// max(0, 1) = 1 s, and a career race always to the silver time.
// Order: FUEL_AI_LEVEL (dev), `--ai-level` (the menu), else 0 for challenges and edited races and 1 for a career race started
// without the menu (--race dev runs and checks keep the port's old target). [port] A level whose time is 0 falls back to 0.
// medals: the same value is the medal a won career race gives (0x630dd0 reads race +0x3e8 as the rank), so raceLevel is
// what the end-race award gets and raceAiLevel only adds the port's zero-time fallback for the AI's target.
static int raceLevel(const fuel::Event& e) {
    int level = e.kind == fuel::Event::Career ? 1 : 0;
    if (gAiLevelArg >= 0) level = gAiLevelArg;
    if (const char* lv = std::getenv("FUEL_AI_LEVEL")) level = std::atoi(lv);
    return std::clamp(level, 0, 2);
}
static int raceAiLevel(const fuel::Event& e) {
    const int level = raceLevel(e);
    return e.times[size_t(level)] <= 0 ? 0 : level;
}

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
    const fuel::RoadNetwork* roads = nullptr;      // the AI's road / off-road speed blend
    const fuel::RoadGraph* graph = nullptr;        // FUEL's road graph (snap, road at point, route search)
    std::optional<std::array<fuel::RoadTypeParams, 21>> roadTypes;
    std::optional<fuel::AiTables> aiTables;        // from the player's FUEL.exe
    std::shared_ptr<bool> crumbAlive = std::make_shared<bool>(true);  // the breadcrumb target ends with the session
    ~RaceSession() { *crumbAlive = false; }
    double clock = 0;  // seconds since the start (negative during the countdown)

    // Everyone in the race: [0] is the player, the others match the session's vehicles 1...
    // Gates passed and finish times live in rules.racers.
    struct Racer {
        std::string name;        // variant label
        size_t entry = 0;        // in the garage
        fuel::RaceDriver ai;     // opponents (and the player's autopilot): FUEL's ghost racer
        fuel::nitro::AiNitro nitro;  // [nitro] when the racer presses its Nitro button (physics/ai_nitro.hpp)
        int nameTT = 0;          // PK16 0x620c20: the opponent's name text id (1500..1573 shuffled per race); 0 = none (the player)
        // [upgrades U4] an opponent's draw (hub tier, seeded per race and grid slot), what the transform did to its vehicle and
        // its ghost's factors (core/upgrades aiFactors; identity for the player and stock racers).
        fuel::upg::AiDraw upgDraw;
        fuel::upg::Result upgResult;
        fuel::upg::AiFactors upg;
    };
    std::vector<Racer> racers = std::vector<Racer>(1);
    fuel::RankReactions reactions;                   // PK31: 0x5c2820's +0x178 / +0x17c
    fuel::nitro::Overtakes nitroPlaces;              // [nitro] race places gained fill the tanks
    int upgTier = -1;                                // [upgrades U4] the hub tier the opponents' stages were drawn for
    std::vector<std::array<float, 3>> reactPrev;     // PK31: the records' previous positions (+0xe4..)
    bool reported = false;          // finishing order printed

    RaceSession() = default;
    RaceSession(const RaceSession&) = delete;

    const fuel::Event& event() const { return races[index]; }
    const fuel::Race& race() const { return races[index].race; }
    const fuel::RaceLine& lineFor(size_t i) const { return rules.usesRoute(i) ? line : aiLine; }
    // PK16 0x5c2070: the straight length from the racer's position through the gates it has left (race +0x138 list from
    // +0x164), the clamp of 0x5c21a0.
    float straightLeft(size_t i, double x, double z) const {
        const std::vector<std::array<float, 3>>& gates = rules.lineFor(i);
        float sum = 0;
        double px = x, pz = z;
        for (size_t k = rules.racers[i].next; k < gates.size(); ++k) {
            sum += float(std::hypot(gates[k][0] - px, gates[k][2] - pz));
            px = gates[k][0], pz = gates[k][2];
        }
        return sum;
    }
    // PK16 0x5c21a0: racer record +0x170 (distance left): scenarios 11 / 13 (helicopter, catch AI) the straight length,
    // else 0x628c40's blocks length clamped to [straight, 20 x straight]. `s` = the racer's distance along its path.
    float leftOf(size_t i, double x, double z, float s) const {
        const float straight = straightLeft(i, x, z);
        if (rules.mode == fuel::Race::GmHelicopter || rules.mode == fuel::Race::GmCatchAI) return straight;
        return lineFor(i).distanceLeft(x, z, s, straight);
    }
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
        if (streamer) streamer->setMission(event().id);  // the course sub-world streams in with its mission (p35)
        gStorms.raceBegin(event());  // P61: RESET_M_START on the tornado paths, the TOR route (0x631bd0)
        // ---- PK04: the race's weather overrides + time of day (0x631bd0 / 0x631aa0), lerped by 0x632250 per frame
        renderer.beginRaceWeather(event().id, r.start, r.finish, [pd = &d] {
            const simd_float3 p = pd->position();
            return std::array<double, 3>{p.x, p.y, p.z};
        }, r.editorFlags);  // PK21: mission +0x54 of an edited race (0x631aa0's time of day)
        // ---- end PK04
        if (terrain) rules.ground = [t = terrain](float x, float z) { return t->smoothHeightAt(x, z); };
        if (graph && roadTypes)  // gate widths from the road under each gate (0x4fdea0, p38)
            rules.gateWidth = [g = graph, &types = *roadTypes](float x, float z) { return fuel::missionGateWidth(*g, types.data(), types.size(), x, z); };
        rules.begin(r, racers.size(), event().times[0]);
        route = rules.route;
        line.build(r.start, route);
        aiLine.build(rules.aiPath.empty() ? r.start : rules.aiPath[0], rules.aiTargets);
        // PK16 0x627590: the finish block runs along the finish gate's direction (the checkpoint record's +8 / +0xc);
        // forward = (sin yaw, cos yaw) of the finish rotation, turned towards the course like the spline gates.
        {
            auto finishDirOf = [&](fuel::RaceLine& ln, float yaw) {
                if (ln.points.size() < 2) return;
                const auto& a = ln.points[ln.points.size() - 2];
                const auto& b = ln.points.back();
                float tx = b[0] - a[0], tz = b[2] - a[2];
                float dx = std::sin(yaw), dz = std::cos(yaw);
                if (dx * tx + dz * tz < 0) dx = -dx, dz = -dz;
                ln.setFinishDir(dx, dz);
            };
            finishDirOf(line, r.finishRot[1]);
            finishDirOf(aiLine, r.finishRot[1]);
        }
        if (roads && roadTypes) {
            // FUEL's road blocks (0x627590): the AI follows the roads where a stretch's ends lie near them.
            const auto flat = [&types = *roadTypes](int t) { return t == 14 || t < 0 || t >= 21 ? -1.0f : types[size_t(t)].flat; };
            // FUEL's road graph is the default since P46 (FUEL's static contacts: the trail jumps no longer flip the
            // racers); FUEL_AI_ROADGRAPH=0 restores the port's road blocks.
            const char* rg = std::getenv("FUEL_AI_ROADGRAPH");
            if (graph && !(rg && std::atoi(rg) == 0)) {
                std::function<float(float, float)> height = [](float, float) { return 0.0f; };
                if (terrain) height = [t = terrain](float x, float z) { return t->smoothHeightAt(x, z); };
                const fuel::AiTables* tab = aiTables ? &*aiTables : nullptr;
                line.routeOnRoadGraph(*graph, flat, tab, height);
                aiLine.routeOnRoadGraph(*graph, flat, tab, height);
            } else {
                line.routeOnRoads(*roads, flat);
                aiLine.routeOnRoads(*roads, flat);
            }
            // World courses: the authored lanes (IAPathBlockSpline, P43). The default since P58 (FUEL's obstacle boxes,
            // 0x6189b0: CR3-Yell-CT's pier no longer overturns the quads); FUEL_AI_SPLINE=0 turns them off.
            if (r.splineGraph && !(std::getenv("FUEL_AI_SPLINE") && std::atoi(std::getenv("FUEL_AI_SPLINE")) == 0)) splineBlocks(r);
            int onRoad = 0;
            for (size_t k = 0; k + 1 < line.path.size(); ++k) onRoad += line.pathHalf[k] < 30.0f;
            if (std::getenv("FUEL_AI_PATHDUMP"))
                for (size_t k = 0; k < line.path.size(); ++k)
                    std::printf("  path %zu %.1f %.1f half %.1f along %.0f\n", k, line.path[k][0], line.path[k][1], line.pathHalf[k], line.pathAlong[k]);
            std::printf("AI path: %zu points, %.0f m (gates %.0f m), %d road segments\n", line.path.size(), line.pathLength(), line.length(), onRoad);
        }
        playerS = 0;
        clock = rules.clock;
        state = rules.state;
        reported = false;
        // FUEL's AI races against the event's target time for the level (race manager +0x3e8, set by Race_Start 0x62e7a1
        // from RestartMission's 2nd argument; cr5, notes cr5-ai.md): the career page's medal row, a challenge 0.
        const int level = raceAiLevel(event());
        rules.level = raceLevel(event());  // medals: the result line's stars
        std::printf("race AI level %d: target time %.0f s (AddMissionTime %.0f / %.0f / %.0f)\n", level, event().times[size_t(level)],
                    event().times[0], event().times[1], event().times[2]);
        for (Racer& racer : racers) {
            racer.ai.setTarget(event().times[size_t(level)] * racer.upg.time, level);  // [upgrades U4] x the racer's stages' lap gain (1 = FUEL)
            racer.ai.setCornerScale(racer.upg.corner);                                  // [upgrades U4] its corner caps (1 = FUEL)
            racer.ai.restart();
            racer.nitro.reset();  // [nitro]
        }
        d.physics().resetNitro(), nitroPlaces.reset();  // [nitro] the tanks start full at every race start
        // PK01: FUEL's start heading. The race object's start quaternion (0x62d480: EMD_SetStartRot through 0x644980)
        // is what the pre-start hold (0x5279c0 -> 0x62ce40 -> 0x529480) keeps the player on until the start, and the
        // AI grid is built from it (0x621270 -> IA_BuildGrid). (The port used to face the first gate instead.)
        const float yaw = fuel::RaceRules::headingOf(rules.startPose().quat);
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
        // PK01: the pre-start hold (_CheckpointRespawn 0x5279c0 -> 0x62ce40): in race states 2-4 the player's vehicle
        // goes back on its start spot and heading after every physics step (the TELEPORT_HELICO slot branch is the
        // cut-scene hold, which steps no physics).
        d.physics().setStartHold([x = double(r.start[0]), z = double(r.start[2]), yaw, alive = crumbAlive](double& sx, double& sz, float& h) {
            if (!*alive) return false;
            sx = x, sz = z, h = yaw;
            return true;
        });
        // P70b: breadcrumbs face the race path 30 m ahead of the player (0x528690 -> 0x628bc0 IAPath_Sample(30, 30):
        // the second sample, the first when the path ends sooner); the port's cursor is the nearest point on the path.
        d.physics().setCrumbTargets(
            [ln = std::make_shared<const fuel::RaceLine>(line), alive = crumbAlive](double px, double pz, double& x, double& z) {
                if (!*alive || ln->path.size() < 2) return false;
                const float s = ln->project(px, pz, 0.0f, ln->pathLength());
                const auto p = ln->at(s + 30.0f <= ln->pathLength() + ln->finishRun ? s + 30.0f : s);
                x = p[0], z = p[2];
                return true;
            },
            {});
        if (streamer) streamer->update(d.position(), renderer, d.physics(), true);
    }
    // Spline blocks (P43): each line point's gate direction (forward = (sin yaw, cos yaw) of its start /
    // checkpoint / finish rotation, towards the course) and half-width (the world course's own, 0x4fdea0).
    void splineBlocks(const fuel::Race& r) {
        std::vector<std::array<float, 2>> dir;
        std::vector<float> half;
        for (size_t i = 0; i < line.points.size(); ++i) {
            const auto& p = line.points[i];
            float yaw = std::nanf(""), h = 12.5f;
            auto match = [&](const std::array<float, 3>& q, float y, float hw) {
                if (std::hypot(p[0] - q[0], p[2] - q[2]) < 1.0f) yaw = y, h = hw;
            };
            match(r.start, r.startRot[1], r.startHalf);
            for (size_t k = 0; k < r.checkpoints.size(); ++k)
                match(r.checkpoints[k], k < r.checkpointRot.size() ? r.checkpointRot[k][1] : 0.0f, k < r.checkpointHalf.size() ? r.checkpointHalf[k] : 12.5f);
            if (i + 1 == line.points.size()) match(r.finish, r.finishRot[1], r.finishHalf);
            // Travel direction through the point (also orients a gate whose rotation faces the other way).
            const auto& a = line.points[i == 0 ? 0 : i - 1];
            const auto& b = line.points[std::min(i + 1, line.points.size() - 1)];
            float tx = b[0] - a[0], tz = b[2] - a[2];
            const float tl = std::hypot(tx, tz);
            if (tl > 1e-3f) tx /= tl, tz /= tl;
            float dx = tx, dz = tz;
            if (!std::isnan(yaw)) {
                dx = std::sin(yaw), dz = std::cos(yaw);
                if (dx * tx + dz * tz < 0) dx = -dx, dz = -dz;
            }
            dir.push_back({dx, dz});
            half.push_back(h);
        }
        const int n = line.routeOnSplineGraph(*r.splineGraph, dir, half);
        std::printf("AI path: %d of %zu stretches on the course's lanes (SplineGraph_Z)\n", n, line.points.size() - 1);
    }
    std::vector<int> positions(const DriveSession& d) const { return rules.positions(where(d)); }
    // Advances the race for every racer; drives the opponents (and holds everyone on the
    // line during the countdown). `in` is the player's input.
    void update(float dt, DriveSession& d, fuel::DriveInput& in) {
        rules.update(dt, where(d));
        clock = rules.clock;
        state = rules.state;
        // ---- PK31: the ranking pass's overtake gestures (0x5c2820 -> 0x5c2370) on the racers' riders
        if (racers.size() > 1) {
            const auto at = where(d);
            const std::vector<int> pos = rules.positions(at);
            std::vector<int> order(at.size());
            for (size_t i = 0; i < order.size(); ++i) order[i] = int(i);
            std::stable_sort(order.begin(), order.end(), [&](int a, int b) { return pos[size_t(a)] < pos[size_t(b)]; });
            std::vector<fuel::RankReactions::Rec> recs(at.size());
            if (reactPrev.size() != at.size()) reactPrev = at;
            for (size_t i = 0; i < at.size(); ++i) {
                const auto& rr = rules.racers[i];
                recs[i] = {rr.finished < 0 && !rr.out, at[i][0], at[i][2], at[i][0] - reactPrev[i][0], at[i][2] - reactPrev[i][2]};
            }
            reactPrev = at;
            for (const auto& g : reactions.update(dt, order, recs)) d.riderGesture(g.racer, g.base);
        }
        // ---- end PK31
        if (fuel::nitro::enabled() && racers.size() > 1) {  // [nitro] a race place gained (and held 1 s) fills the tank
            std::vector<bool> racing(racers.size());
            for (size_t i = 0; i < racers.size(); ++i) racing[i] = state == Running && rules.racers[i].finished < 0 && !rules.racers[i].out;
            std::vector<float> got;
            nitroPlaces.step(dt, positions(d), racing, got, fuel::nitro::params());
            for (size_t i = 0; i < got.size(); ++i)
                if (got[i] > 0) d.physics().addNitroBonus(int(i), got[i], fuel::nitro::KindOvertake);
        }
        if (state == Countdown) {
            in = {0, 0, 0, true};
            for (int i = 1; i < d.vehicleCount(); ++i) d.setInput(i, {0, 0, 0, true});
            return;
        }
        // Opponents: FUEL's ghost racers (race_ai, P29). The ranks of 0x5cb6f0 (pm+0x2e8c, 0x5d3300): the AI vehicles
        // sorted by pm+0x2e88 = their distance to the player (3D), x 5 when a 5 m sphere round them is outside the
        // game camera's frustum (0x821330 / 0x821c20). So the racers in view are the simulated ones (rank < 4 within
        // 150 m); the ones behind the camera are placed on their ghosts. (Before P97 the port ranked by the plain
        // distance: at the start the leaders, in full view, were the kinematic ones.)
        const simd_float3 p0 = d.position(0);
        std::vector<std::pair<float, int>> near;
        for (int i = 1; i < d.vehicleCount() && size_t(i) < racers.size(); ++i) {
            const simd_float3 p = d.position(i);
            float dist = simd_distance(p, p0);
            if (view && !sphereInView(*view, viewAspect, p, 5.0f)) dist *= 5.0f;
            near.push_back({dist, i});
        }
        std::sort(near.begin(), near.end());
        for (size_t r = 0; r < near.size(); ++r) {
            const int i = near[r].second;
            Racer& racer = racers[size_t(i)];
            const fuel::RaceRules::Racer& rr = rules.racers[size_t(i)];
            // PK16: a finished racer keeps being driven (0x619650 state 4 / 5): it runs out to its parking spot (0x624310).
            if ((rr.out || (rr.finished < 0 && rr.next >= rules.lineFor(size_t(i)).size()))) {
                d.physics().setGuide(i, false);
                d.setInput(i, {0, 0, 0, true});
                continue;
            }
            drive(racer, d, i, dt, int(r), near[r].first);
            // FUEL_AI_TRACE=N: follow opponent N twice a second (and print every reset).
            static const char* trace = std::getenv("FUEL_AI_TRACE");
            if (trace && std::atoi(trace) == i && int(clock * 60) % 30 == 0) {
                const fuel::VehicleState st = d.state(i);
                const simd_float3 at = d.position(i);
                const auto& t = rules.lineFor(size_t(i))[rr.next];
                std::printf("    t=%5.1f #%d (%.0f, %.0f, %.0f) next %zu dist %.0f  %5.1f km/h ghost %5.1f km/h s %.0f %s  resets %d/%d\n", clock, i, at.x, at.y, at.z, rr.next,
                            std::hypot(at.x - t[0], at.z - t[2]), st.speedKmh, racer.ai.ghostSpeed() * 3.6f, racer.ai.progress(),
                            racer.ai.physical() ? "sim" : "kin", racer.ai.resets(), d.physics().respawns(i));
            }
        }
    }
    // Test driver for the player: the opponents' AI (always simulated), no rubber band.
    fuel::DriveInput autopilot(DriveSession& d) {
        if (state != Running || rules.racers[0].finished >= 0 || rules.racers[0].next >= route.size()) {
            d.physics().setGuide(0, false);
            return {};
        }
        drive(racers[0], d, 0, 1.0f / 60, -1);
        return autoInput;
    }
    fuel::DriveInput autoInput;  // the autopilot's last input
    fuel::RaceDriver::Command autoCmd;  // [upgrades U4] its last command (the lap bot rides its line)
    // P97: the game camera of the last frame (0x5d3300's frustum test) and the frame's aspect; none: all in view.
    std::optional<fuel::OrbitCamera> view;
    float viewAspect = 16.0f / 9.0f;
    // P97: test players for --ai-check (FUEL_AI_PLAYER): "stop" holds the handbrake; "launch" is full throttle,
    // steering for the AI path 20 m ahead of the player (no guide forces).
    fuel::DriveInput testPlayer(DriveSession& d, const std::string& mode) {
        d.physics().setGuide(0, false);
        if (mode == "stop") return {0, 0, 0, true};
        if (mode == "bot") return lapBot(d);
        const simd_float3 p = d.position(0);
        playerS = line.project(p.x, p.z, playerS - 30.0f, playerS + 60.0f);
        const auto t = line.at(playerS + 25.0f);
        const float err = std::remainder(std::atan2(t[0] - p.x, t[2] - p.z) - d.heading(), 2 * float(M_PI));
        return {1.0f, 0, std::clamp(-2.0f * err, -1.0f, 1.0f), false};  // steer +1 = right (the trigger check's sign)
    }
    // [upgrades U4] --ai-player bot: the lap bot of the stage lap-time comparison (notes upg4.md). Its line is the autopilot's
    // (the opponents' ghost, FUEL's racing line, obstacles and corridor), held by the guide forces SIDEWAYS only: the guide's
    // velocity target is the vehicle's own speed along the ghost's direction, so the guide never pushes or brakes along the
    // track. Throttle and brake are the bot's: the speed the bends ahead allow (lateral and braking accelerations
    // FUEL_BOT_ALAT / FUEL_BOT_ABRAKE m/s^2, default 8 / 8; FUEL_BOT_GRIP=1 scales the lateral one by the tyre stage's
    // lateral grip gain, as the ghost's corner caps are), full throttle below it; the Nitro button as an opponent's
    // (AiNitro::decide on the vehicle's own Params). So the time measures the drive, weight, traction, suspension and nitro
    // stages on FUEL's physics. Not FUEL: a test driver.
    fuel::nitro::AiNitro botNitro;
    float botFull = 0, botBrake = 0, botAir = 0;  // seconds at full throttle, braking, airborne
    fuel::DriveInput lapBot(DriveSession& d) {
        if (rules.racers[0].finished >= 0 || rules.racers[0].next >= route.size()) {
            d.physics().setGuide(0, false);
            return {0, 1, 0, true};
        }
        static const float aBrake = std::getenv("FUEL_BOT_ABRAKE") ? float(std::atof(std::getenv("FUEL_BOT_ABRAKE"))) : 8.0f;
        static const float aLat = std::getenv("FUEL_BOT_ALAT") ? float(std::atof(std::getenv("FUEL_BOT_ALAT"))) : 8.0f;
        static float grip = -1;
        if (grip < 0) {  // the tyre stage's corner factor for this vehicle (aiFactors' corner = sqrt(1 + lateral gain))
            grip = 1;
            if (std::getenv("FUEL_BOT_GRIP") && std::atoi(std::getenv("FUEL_BOT_GRIP")) != 0) {
                const float c = fuel::upg::aiFactors(fuel::upg::playerStages(), fuel::upg::classOf(d.setup(0).name), 1.0f).corner;
                grip = c * c;
            }
        }
        const fuel::RaceDriver::Command& c = autoCmd;  // the autopilot's ghost this frame (autopilot() ran first): line, aim
        const fuel::VehicleState st = d.state(0);
        const simd_float3 p = d.position(0);
        const fuel::fv::FuelVehicle* m = d.physics().model(0);
        if (c.physical && m) {  // the guide sideways only: the vehicle's speed along the ghost's direction
            const fuel::fv::Vec3 vv = m->body().S.v;
            const float gl = std::hypot(c.vel[0], c.vel[2]), sp = std::hypot(vv.x, vv.z);
            std::array<float, 3> vt{vv.x, vv.y, vv.z};
            if (gl > 0.1f) vt[0] = c.vel[0] / gl * sp, vt[2] = c.vel[2] / gl * sp;
            d.physics().setGuide(0, true, c.pos, vt, c.rot);
        }
        const float v = std::fabs(st.speedKmh) / 3.6f;
        playerS = line.project(p.x, p.z, playerS - 30.0f, playerS + 60.0f);
        auto heading = [&](float s) {
            float dx = 0, dz = 1;
            line.at(s, &dx, &dz);
            return std::atan2(dx, dz);
        };
        // The speed the bends ahead allow: min over d of sqrt(v_bend(d)^2 + 2 aBrake d), v_bend = sqrt(aLat / curvature).
        float target = 200.0f;
        for (float a = 0; a <= 160.0f; a += 5.0f) {
            const float k = std::fabs(std::remainder(heading(playerS + a + 6.0f) - heading(playerS + a - 6.0f), 2 * float(M_PI))) / 12.0f;
            const float vb = std::sqrt(aLat * grip / std::max(k, 1e-4f));
            target = std::min(target, std::sqrt(vb * vb + 2.0f * aBrake * a));
        }
        fuel::DriveInput in = autoInput;  // the autopilot's steering (FUEL's AI has none: 0)
        in.throttle = in.brake = 0, in.handbrake = false;
        if (v < target - 0.5f) in.throttle = 1.0f;
        else if (v > target + 1.5f) in.brake = std::clamp((v - target) / 4.0f, 0.2f, 1.0f);
        else in.throttle = 0.3f;
        // FUEL_BOT_TRACE=1: time at full throttle / braking / in the air, once a second the speed and the bend target.
        static const bool trace = std::getenv("FUEL_BOT_TRACE") != nullptr;
        botFull += in.throttle >= 1.0f ? 1.0f / 60 : 0.0f, botBrake += in.brake > 0 ? 1.0f / 60 : 0.0f, botAir += st.wheelsOnGround == 0 ? 1.0f / 60 : 0.0f;
        if (trace && int(clock * 60) % 60 == 0)
            std::printf("  bot t=%5.1f s=%6.0f v=%5.1f target=%5.1f km/h gear %d ground %d slide %.2f full %.0f s brake %.0f s air %.0f s\n", clock, playerS, v * 3.6f,
                        target * 3.6f, st.gear, st.wheelsOnGround, st.skid, botFull, botBrake, botAir);
        in.nitro = false;
        if (fuel::nitro::enabled() && st.nitro.on) {  // the opponents' button rule (ai_nitro), the vehicle's own tank
            fuel::nitro::AiView nv;
            nv.level = st.nitro.level, nv.wrecked = st.wrecked, nv.airborne = st.wheelsOnGround == 0, nv.speed = v;
            nv.turnNear = fuel::nitro::pathTurn(line, playerS, 0, v * 1.2f + 15.0f);
            nv.turnAhead = fuel::nitro::pathTurn(line, playerS, 0, std::clamp(v * 2.5f + 30.0f, 40.0f, 150.0f));
            nv.turnBehind = fuel::nitro::pathTurn(line, playerS, -40.0f, 0.0f);
            in.nitro = botNitro.decide(1.0f / 60, nv, d.physics().nitroParams(0)) && in.throttle >= 1.0f;
        }
        return in;
    }
    float playerS = 0;           // the player's distance along the AI path
    float maxGhostKmh = 0, maxGhostClock = 0, topKmh = 0;  // P82: the fastest opponent ghost, the fastest vehicle (P619/P620)
    int maxGhostRacer = -1;
    static fuel::AiVehicle aiVehicle(const fuel::VehicleSetup& setup) {
        fuel::AiVehicle v;
        auto P = [&](int n, float dflt) { return size_t(n) < setup.params.size() && setup.params[size_t(n)] > 0 ? setup.params[size_t(n)] : dflt; };
        v.roadKmh = P(619, setup.topSpeedKmh), v.offKmh = P(620, setup.topSpeedKmh);
        v.road0to100 = P(623, 3.0f), v.off0to100 = P(624, 3.2f);
        v.maxSlopeDeg = std::min(P(626, 90.0f), 90.0f);
        float r = 1.0f;
        float hw = 0;
        for (const auto& w : setup.wheels) {
            r = std::max(r, std::hypot(w.hub[0], w.hub[2]) + w.radius * 0.5f);
            hw = std::max(hw, std::fabs(w.hub[0]) + w.width * 0.5f);
        }
        v.radius = r;
        if (hw > 0.3f) v.halfWidth = hw;  // racer +0x220 = pm+0x2e98 / 2 (P58; pm+0x2e98 taken as the width over the tyres)
        v.vehicleClass = setup.vehicleClass;  // PK31: the AI lean springs' bike limits (0x61c1b0)
        return v;
    }
    // [upgrades U4] racer i's AiVehicle with its stages: 0-100 times / accel, top speeds x top (core/upgrades aiFactors).
    // The player (i = 0) and stock racers: FUEL's numbers.
    fuel::AiVehicle ownAiVehicle(const DriveSession& d, int i) const {
        fuel::AiVehicle v = aiVehicle(d.setup(i));
        if (i > 0 && size_t(i) < racers.size()) {
            const fuel::upg::AiFactors& f = racers[size_t(i)].upg;
            v.road0to100 /= f.accel, v.off0to100 /= f.accel;
            v.roadKmh *= f.top, v.offKmh *= f.top;
        }
        return v;
    }
    // One step of racer i's ghost and what it does to the vehicle. rank < 0: the player's autopilot.
    void drive(Racer& racer, DriveSession& d, int i, float dt, int rank, float rankDist = 0) {
        fuel::DriveWorld& world = d.physics();
        const fuel::VehicleState st = d.state(i);
        const fuel::fv::FuelVehicle* m = world.model(i);
        fuel::RaceDriver::Sense s;
        s.pos = st.chassis.position;
        if (m) {
            const fuel::fv::Vec3 v = m->body().S.v, f = fuel::fv::rotate(m->body().S.q, {0, 0, 1}), u = fuel::fv::rotate(m->body().S.q, {0, 1, 0});
            s.vel = {v.x, v.y, v.z}, s.fwd = {f.x, f.y, f.z}, s.up = {u.x, u.y, u.z};
            s.fwdSpeed = m->forwardSpeed();
            s.cruiseSpeed = m->meanWheelSpeed();  // PK16 0x5257b0
        }
        s.frozen = state == Countdown;
        s.clock = float(clock);
        s.forcePhysical = rank < 0;
        s.rank = std::max(rank, 0);
        const simd_float3 p0 = d.position(0);
        s.playerDist = rank < 0 ? 0.0f : float(std::hypot(s.pos[0] - p0.x, s.pos[2] - p0.z));
        s.switchDist = rank < 0 ? 0.0f : rankDist;
        const fuel::RaceLine& ln = lineFor(size_t(i));
        playerS = line.project(p0.x, p0.z, playerS - 30.0f, playerS + 60.0f);
        const float playerLeft = line.pathLength() - playerS;
        // PK16: pd = the player's record +0x170 minus the racer's (0x5c21a0 for both).
        s.playerAhead = rank < 0 ? 0.0f : leftOf(0, p0.x, p0.z, playerS) - leftOf(size_t(i), s.pos[0], s.pos[2], racer.ai.progress());
        (void)playerLeft;
        s.finishRank = rank < 0 ? -1 : rules.finishRank(size_t(i));
        s.aiCount = race().aiCount;
        if (const fuel::fv::FuelVehicle* pm = world.model(0)) s.playerVel = {pm->body().S.v.x, pm->body().S.v.y, pm->body().S.v.z};
        s.own = ownAiVehicle(d, i);  // [upgrades U4] an opponent's own 0-100 / top speeds with its stages
        s.player = aiVehicle(d.setup(0));  // the player's P619 / P620 / P623 / P624 as FUEL reads them (never staged)
        for (int j = 0; j < d.vehicleCount() && size_t(j) < racers.size(); ++j) {
            if (j == i) continue;
            const fuel::VehicleState o = d.state(j);
            fuel::RaceDriver::Other other;
            other.x = o.chassis.position[0], other.z = o.chassis.position[2];
            if (const fuel::fv::FuelVehicle* om = world.model(j)) other.vx = om->body().S.v.x, other.vz = om->body().S.v.z;
            other.radius = aiVehicle(d.setup(j)).radius;
            s.others.push_back(other);
        }
        if (terrain)
            s.ground = [t = terrain](double x, double z, float n[3]) {
                const float h = t->smoothHeightAt(float(x), float(z));
                const float hx = t->smoothHeightAt(float(x) + 1, float(z)) - t->smoothHeightAt(float(x) - 1, float(z));
                const float hz = t->smoothHeightAt(float(x), float(z) + 1) - t->smoothHeightAt(float(x), float(z) - 1);
                const float l = std::sqrt(hx * hx * 0.25f + 1 + hz * hz * 0.25f);
                n[0] = -hx * 0.5f / l, n[1] = 1 / l, n[2] = -hz * 0.5f / l;
                return h;
            };
        static const bool ghostGround = !(std::getenv("FUEL_AI_GHOSTGROUND") && std::atoi(std::getenv("FUEL_AI_GHOSTGROUND")) == 0);
        if (terrain && ghostGround) {
            // 0x61b810 (P43): statics first (a ray from 2 m above the ghost, 100 m down: piers, bridges), then the
            // ground; the water level counts for vehicles with P631 > 0.5 (pm+0xfac; 74 of the 83 vehicles).
            const auto& P = d.setup(i).params;
            const bool floats = P.size() > 631 && P[631] > 0.5f;
            s.ghostGround = [t = terrain, &world, floats](double x, double y, double z, float n[3]) {
                float h = t->smoothHeightAt(float(x), float(z));
                if (floats) h += std::max(0.0f, world.waterDepthAt(x, z));  // 0x4df550 (the soft term not ported)
                const float hs = world.staticFloorBelow(x, y + 2.0, z, 100.0f);
                if (hs > h) {
                    n[0] = 0, n[1] = 1, n[2] = 0;  // port: the object's normal is not returned (flat)
                    return hs;
                }
                const float hx = t->smoothHeightAt(float(x) + 1, float(z)) - t->smoothHeightAt(float(x) - 1, float(z));
                const float hz = t->smoothHeightAt(float(x), float(z) + 1) - t->smoothHeightAt(float(x), float(z) - 1);
                const float l = std::sqrt(hx * hx * 0.25f + 1 + hz * hz * 0.25f);
                n[0] = -hx * 0.5f / l, n[1] = 1 / l, n[2] = -hz * 0.5f / l;
                return h;
            };
        }
        if (graph && roadTypes)
            // 0x617000: T[type][+0xb4] of the record 0x6d1d70 finds (1 off roads).
            s.offroad = [g = graph, &types = *roadTypes](double x, double z) {
                const int r = g->roadAt(float(x), float(z), [&types](int t) { return t >= 0 && t < 21 ? types[size_t(t)].flat : 0.0f; });
                const int t = r >= 0 ? g->rec(size_t(r)).type : -1;
                return t >= 0 && t < 21 ? types[size_t(t)].aiSpeedBlend : 1.0f;
            };
        s.statics = [&world](double x0, double z0, double x1, double z1, std::vector<std::array<double, 6>>& out) { world.staticBounds(x0, z0, x1, z1, out); };
        // P58: FUEL's obstacle boxes (0x619d70 / 0x6189b0); FUEL_AI_BOXES=0 restores the port's AABBs with their height filter.
        static const bool aiBoxes = !(std::getenv("FUEL_AI_BOXES") && std::atoi(std::getenv("FUEL_AI_BOXES")) == 0);
        if (aiBoxes) s.boxes = [&world](double x0, double z0, double x1, double z1, std::vector<fuel::DriveWorld::WorldBox>& out) { world.staticBoxes(x0, z0, x1, z1, out); };
        if (graph && roadTypes)
            s.roadType = [g = graph, &types = *roadTypes](double x, double z) {
                const int r = g->roadAt(float(x), float(z), [&types](int t) { return t >= 0 && t < 21 ? types[size_t(t)].flat : 0.0f; });
                return r >= 0 ? int(g->rec(size_t(r)).type) : -1;
            };
        s.tables = aiTables ? &*aiTables : nullptr;
        const bool nitroWant = fuel::nitro::aiNitroStep(racer.nitro, racer.ai, ln, s, st, dt, i, rank >= 0 || fuel::nitro::autopilotNitro(),
                                                        world.nitroParams(i));  // [nitro]; [upgrades U4] the racer's own Params
        const fuel::RaceDriver::Command c = racer.ai.step(ln, s, dt);
        // rain-ai (0x6182b0): an opponent whose path cursor passed the checkpoint block of its record's target passes that
        // gate at the next update even when its move missed the gate segment (FUEL's player has no AI path: not rank < 0).
        if (rank >= 0 && c.checkpointBlock >= 0 && size_t(c.checkpointBlock) < ln.points.size())
            rules.aiCheckpointBlock(size_t(i), ln.points[size_t(c.checkpointBlock)]);
        if (rank >= 0 && !s.frozen) {  // P82: the fastest opponent ghost against the fastest vehicle's top speed (--ai-check)
            const float kmh = racer.ai.ghostSpeed() * 3.6f;
            if (kmh > maxGhostKmh) maxGhostKmh = kmh, maxGhostRacer = i, maxGhostClock = float(clock);
            topKmh = std::max({topKmh, s.own.roadKmh, s.own.offKmh, s.player.roadKmh, s.player.offKmh});
        }
        if (c.reset && std::getenv("FUEL_AI_TRACE"))
            std::printf("    reset #%d at t=%.1f from (%.0f, %.0f) speed %.1f\n", i, clock, s.pos[0], s.pos[2], st.speedKmh);
        if (c.place) world.holdPose(i, c.pos, c.rot, c.reset ? std::array<float, 3>{} : c.vel);
        if (c.physical && !c.reset && !s.frozen) world.setGuide(i, true, c.pos, c.vel, c.rot);
        else if (!c.place) world.setGuide(i, false);
        fuel::DriveInput in = c.physical ? c.input : fuel::DriveInput{};
        in.nitro = nitroWant;  // [nitro] a kinematic racer's tank drains too: its ghost boosts
        if (i == 0) autoCmd = c;  // [upgrades U4] the lap bot's line
        if (i == 0) autoInput = in;
        else d.setInput(i, in);
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
        // P69: FUEL marks the gates with its flag objects (gRaceMarkers); the port's gate columns only on request (the
        // helicopter's column stays: the port draws no helicopter).
        static const bool beacons = std::getenv("FUEL_RACE_BEACONS") != nullptr;
        if (!gRaceMarkers.ready() || beacons)
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

// p40: the HUD values FUEL's HUD classes read (ui/hud.mm, notes/p40-hud.md): speedometer inputs, the minimap's
// player and opponents, and the race info of MissionHUD 0x4b4530.
// What FUEL's music (MusicManager_G) and ambience (Game_SoundMgr 0x4e81e0) read from the game (p45).
static fuel::GameSound gameSound(const DriveSession& d, const RaceSession* race, const std::array<float, 3>& eye, const fuel::MeshRenderer& renderer,
                                 const WorldStreamer* streamer, bool streaming = false) {
    fuel::GameSound g;
    const fuel::VehicleState st = d.state();
    const simd_float3 at = d.position();
    fuel::snd::AmbientInput& a = g.ambient;
    a.camera = {eye[0], eye[1], eye[2]};
    a.player = true, a.playerPos = {at.x, at.y, at.z};
    a.playerSpeed = std::sqrt(st.velocity[0] * st.velocity[0] + st.velocity[1] * st.velocity[1] + st.velocity[2] * st.velocity[2]);
    const fuel::WeatherAmounts& w = renderer.weatherAmounts();
    a.cloud = w.cloud, a.rain = w.rain, a.snow = w.snow, a.wind = w.wind;
    a.timeOfDay = renderer.timeOfDay();
    if (const fuel::GroundGenerator* gg = gGroundGen.load()) a.forest = gg->forestAt(eye[0], eye[2]), a.patch = gg->grassPatch(eye[0], eye[2]);
    if (streamer)  // P49: decor fires (bank symbol 0 of IT_DECORMESH_G objects): kind = the template's name CRC
        for (const auto& s : streamer->decorSounds()) a.decor.push_back({{float(s.x), float(s.y), float(s.z)}, s.crc});
    fuel::snd::MusicGame& m = g.music;
    m.context = race ? fuel::snd::MusicGame::Race : fuel::snd::MusicGame::FreeRoam;
    if (!race) return g;
    const fuel::RaceRules& R = race->rules;
    m.missionState = R.state == fuel::RaceRules::Countdown ? 4 : R.state == fuel::RaceRules::Running ? 5 : R.outcome == fuel::RaceRules::Won ? 6 : 7;
    // P96: while TELEPORT_HELICO holds the race the mission is in state 3 STREAMING (0x632c10); PREPARING (4) and its
    // countdown (game+0x328c +0x2c = 1, MissionHUD 0x4b3da0's beeps 0xf7 / 0xf8) only start when the RTC ends.
    if (streaming && R.state == fuel::RaceRules::Countdown) m.missionState = 3;
    m.mode = std::max(0, R.mode);
    a.mission = true;
    if (R.state == fuel::RaceRules::Countdown && !streaming) g.countdown = fuel::hud::countdown(float(-R.clock));
    const auto pos = race->where(d);
    if (pos.empty()) return g;
    const fuel::RaceLine& ln = race->lineFor(0);
    const auto& r0 = R.racers[0];
    m.total = ln.gateAlong.size() >= 2 && ln.path.size() >= 2 ? ln.pathLength() : ln.length();
    if (r0.finished < 0 && r0.next < ln.points.size()) {  // port: as the HUD's distance (FUEL: route +0x170, 0x5c21a0)
        if (ln.gateAlong.size() >= 2 && ln.path.size() >= 2) {
            const size_t k = std::min(r0.next, ln.gateAlong.size() - 2);
            m.remaining = std::max(0.0f, ln.pathLength() - ln.project(pos[0][0], pos[0][2], ln.gateAlong[k], ln.gateAlong[k + 1]));
        } else {
            m.remaining = std::max(0.0f, ln.length() - ln.progress(r0.next, pos[0][0], pos[0][2]));
        }
    }
    m.timeLeft = R.timed ? R.timeLeft : 1e9f;
    m.position = R.playerPosition(pos);
    m.lastPlace = m.position == int(R.racers.size()) && R.racers.size() > 1;
    m.lastGate = !R.route.empty() && r0.next + 1 == R.route.size();
    if (race->graph) {  // 0x4e6f80: no road record within 50 m, or the nearest farther than 10 m
        int rec = -1, k = 0;
        float qx = 0, qz = 0;
        const float d2 = race->graph->nearestRoad(pos[0][0], pos[0][2], 50.0f, rec, k, qx, qz);
        m.offroad = d2 < 0 || d2 > 100.0f;
    }
    for (size_t i = 1; i < R.racers.size() && i < pos.size(); ++i) {
        if (R.racers[i].finished >= 0 || R.racers[i].out) continue;
        const float dx = pos[i][0] - pos[0][0], dy = pos[i][1] - pos[0][1], dz = pos[i][2] - pos[0][2];
        const float dist = std::sqrt(dx * dx + dy * dy + dz * dz);
        m.nearestRacer = std::min(m.nearestRacer, dist);
        if (R.mode == fuel::Race::GmCatchAI && m.nearestHunted > 1e8f) m.nearestHunted = dist;  // class 3: the first hunted racer
    }
    return g;
}

static fuel::hud::GpsGuide gGps;  // Navigator3dHUD's trail and ribbons (p40)
// P47: FUEL's control mapping (core/input_map.*), loaded for --drive; CONTEXT toggles the 3D GPS (0x4445c0:
// "SetGPS3DVisibleState 0|1", game +0x33f3).
static fuel::input::InputMap gInput;
static bool gInputLoaded = false;
static bool gGps3dVisible = true;
// P86: the GPS arrows for the renderer's scene pass (filled by fillHud, handed over before each render).
static std::vector<fuel::MeshRenderer::GpsRibbonDraw> gGpsDraw;
static std::vector<fuel::MeshRenderer::NavBeamDraw> gNavBeams;  // PK19: Navigator3dHUD beams (0x4bf820) for the renderer
// P47: the world's animated skins (flags, IT_ANIMATED idles) and FUEL's cut-scenes (TELEPORT_HELICO at a race start,
// TR_DEATH on a respawn, VI_BIKE at a race end; game RTC table from Story/Levels.tsc, callers 0x632c10, 0x52f370, 0x62f770).
static WorldAnimation gWorldAnim;
static HeliView gHeliView;  // PK07: the race helicopter in the scene
// ---- PK27: the F14 fly-by (core/f14_flyby, viewer/f14_view): one per process, as FUEL's resident USA1 agent (its mover
// is never reset: F14Move_G vf14 0x51a040 is empty, so retries, KillMission and teleports leave the timer alone).
static fuel::F14Flyby gF14;
static F14View gF14View;
static bool gF14Ready = false;  // the F14 Lod was found and the world map is loaded (0x445500's W+0x4b8)
// ---- PK27 end
// P55: the streamer's IT_ANIMATED events (STARTANIMATED volumes, RESET_ rules) to the world animation.
static void pumpWorldAnimEvents(WorldStreamer* s) {
    if (s == nullptr) return;
    if (static WorldStreamer* placed = nullptr; placed != s) {  // PK29: where the class's event sounds and FX play
        placed = s;
        gWorldAnim.setPlacements([s](uint32_t lod, std::vector<std::array<float, 16>>& out) { s->placementsOf(lod, out); });
    }
    for (uint32_t lod : s->takeAnimResets()) gWorldAnim.reset(lod);
    for (uint32_t lod : s->takeAnimStarts())
        if (gWorldAnim.start(lod) && std::getenv("FUEL_STREAM_TRACE")) std::printf("world animation: %08x plays its event (slot 334)\n", lod);
}
static RtcView gRtc;
static fuel::ResourceLibrary* gLibrary = nullptr;
static std::string gDataDir;
static bool gRtcStartPending = true;  // a race began: its TELEPORT_HELICO is due
static bool gRtcEnabled = std::getenv("FUEL_NO_RTC") == nullptr;
static bool gHudHidden = false;
static CutsceneFx gCutFx;  // P67: FUEL's fader, scripted shake and race-end grade around the cut-scenes
// ---- PK17: the pause backdrop (ui/game_menu.h, viewer/pause_backdrop.hpp). The overlay reports InGameMenuHUD's
// show / close / fade end; the host applies them to the renderer and the sounds.
static float gPauseSavedTint = 0;  // InGameMenuHUD +0x6c (renderer +0x1b8 when the menu opened)
static void pauseBackdropEvents(fuel::MeshRenderer& r, fuel::DriveAudio* audio, simd_float3 player) {
    if (!gOverlay) return;
    gOverlay->setBackdropHosted(true);
    const int ev = gOverlay->takeBackdropEvents();
    if (ev & fuel::ui::GameOverlay::Opened) {  // vf07 0x46aca0
        gPauseSavedTint = r.tintLerpA();       // renderer +0x1ac..+0x1b8 -> +0x60..+0x6c
        r.setPauseBackdrop(true);              // "Pause", 0x46abd0 (20:00, cloud 0), sway factor 0
        if (audio) audio->pauseMenu(true);     // 0x5d8e60 on each vehicle, WinSound mask 7
        std::printf("pause: open (Pause, sky 20:00 cloud 0, sound mask 7, HUDs off)\n");
    }
    if (ev & fuel::ui::GameOverlay::Closed) {  // vf08 0x46ade0 + "UnPause"
        r.setPauseBackdrop(false);
        if (audio) audio->pauseMenu(false);    // mask 0
        std::printf("pause: close (UnPause, clock restored, sound mask 0)\n");
    }
    if (ev & fuel::ui::GameOverlay::FadedOut) {  // vf10 0x46aed0 (< 0): the tint, "SetHudsVisibleState ON"
        const float a = fuel::pause::closedTintA(fuel::fv::Breadcrumbs::worldOutside(player.x, player.z, gOverlay->menu().gameMode() == 2), gPauseSavedTint);
        r.tintOverride = a > 0 ? simd_make_float4(r.tintOverride.x, r.tintOverride.y, r.tintOverride.z, a) : simd_make_float4(0, 0, 0, -1);
        std::printf("pause: faded out (tint %.2f, HUDs on)\n", a);
    }
}
// 0x46af90 every frame while +0x4c != 0 and alpha > 0 (after the camera's and the end grade's writes: the pause HUD's
// manipulator group 0x14 runs after them).
static void pauseBackdropTint(fuel::MeshRenderer& r) {
    if (!gOverlay || !gOverlay->backdropShown() || !(gOverlay->backdropAlpha() > 0)) return;
    const fuel::pause::Tint t = fuel::pause::menuTint([](int n) { return gCutFx.pp(n); }, gOverlay->backdropAlpha(), gPauseSavedTint);
    r.tintOverride = simd_make_float4(t.r, t.g, t.b, t.a);
    r.speedBlur = 0, r.speedBlurAbsolute = t.blurFlag ? t.blur : 0.0f;  // viewport +0x1dc, flag 0x8000
}
// ---- end PK17
}  // namespace (PK13: the RTC node FX hooks declared before it, at file scope)
static bool rtcHasFx() { return gRtc.active() && gRtc.hasFx(); }
static bool worldAnimHasFx() { return gWorldAnim.hasFx(); }  // PK29
static void appendWorldAnimFx(std::vector<fuel::particles::Quad>& quads, std::vector<fuel::particles::Batch>& batches) {
    gWorldAnim.appendFx(quads, batches);
}
static void appendRtcFx(std::vector<fuel::particles::Quad>& quads, std::vector<fuel::particles::Batch>& batches, simd_float3 eye) {
    if (gRtc.active()) gRtc.appendFx(quads, batches, {eye.x, eye.y, eye.z});
}
namespace {

// The vehicle's body transform as a row-vector matrix (RTC father for VI_BIKE: event 0xe9).
static fuel::anim::Mat4 vehicleFather(const DriveSession& d) {
    const fuel::VehicleState st = d.state();
    const auto& q = st.chassis.rotation;
    return fuel::anim::localMatrix({1, 1, 1}, {q[0], q[1], q[2], q[3]}, {st.chassis.position[0], st.chassis.position[1], st.chassis.position[2]});
}
// A frame on the game camera (RTC father for TR_DEATH: event 0xea): x right, y up, z forward. PK13: FUEL's 0xea father
// sits at (0, 0, 1) in a camera-space 3-D item seen with FOV 90 (port_play 0x535260): rtc::overlayFather.
static fuel::anim::Mat4 cameraFather(const fuel::OrbitCamera& c) {
    simd_float3 f = simd_normalize(c.target - c.eye);
    simd_float3 r = simd_normalize(simd_cross(simd_make_float3(0, 1, 0), f));
    simd_float3 u = simd_cross(f, r);
    return fuel::rtc::overlayFather({c.eye.x, c.eye.y, c.eye.z}, {r.x, r.y, r.z}, {u.x, u.y, u.z}, {f.x, f.y, f.z}, c.fovH43);
}
// TELEPORT_HELICO's father: RTCMgr_Father at the mission start (0x53ea30), on the ground (ON_GROUND 0x100).
static fuel::anim::Mat4 startFather(const RaceSession& race) {
    const fuel::Race& r = race.race();
    const double y = race.terrain ? race.terrain->smoothHeightAt(r.start[0], r.start[2]) : r.start[1];
    const double a = r.startYaw;  // forward = (sin, cos)
    return fuel::anim::localMatrix({1, 1, 1}, {0, std::sin(a / 2), 0, std::cos(a / 2)}, {r.start[0], y, r.start[2]});
}

// The first controller as XInput sees a pad (what WinInput_Z 0x83b7e0 reads).
static fuel::input::PadState readPad() {
    fuel::input::PadState p;
    GCExtendedGamepad* g = GCController.controllers.firstObject.extendedGamepad;
    if (g == nil) return p;
    using namespace fuel::input;
    p.connected = true;
    p.buttons[PadA] = g.buttonA.pressed, p.buttons[PadB] = g.buttonB.pressed;
    p.buttons[PadX] = g.buttonX.pressed, p.buttons[PadY] = g.buttonY.pressed;
    p.buttons[PadLB] = g.leftShoulder.pressed, p.buttons[PadRB] = g.rightShoulder.pressed;
    p.buttons[PadLT] = g.leftTrigger.pressed, p.buttons[PadRT] = g.rightTrigger.pressed;
    p.buttons[PadLStick] = g.leftThumbstickButton.pressed, p.buttons[PadRStick] = g.rightThumbstickButton.pressed;
    p.buttons[PadBack] = g.buttonOptions.pressed, p.buttons[PadStart] = g.buttonMenu.pressed;
    p.buttons[PadDLeft] = g.dpad.left.pressed, p.buttons[PadDRight] = g.dpad.right.pressed;
    p.buttons[PadDUp] = g.dpad.up.pressed, p.buttons[PadDDown] = g.dpad.down.pressed;
    p.lt = g.leftTrigger.value, p.rt = g.rightTrigger.value;
    p.lx = g.leftThumbstick.xAxis.value, p.ly = g.leftThumbstick.yAxis.value;
    p.rx = g.rightThumbstick.xAxis.value, p.ry = g.rightThumbstick.yAxis.value;
    return p;
}
static void fillHudState(fuel::ui::HudState& h, const DriveSession& d, const RaceSession* race, float dt, const fuel::OrbitCamera& cam) {
    using P = fuel::hud::InfoPage;
    const fuel::VehicleState st = d.state();
    h.dt = dt;
    h.speedMs = std::sqrt(st.velocity[0] * st.velocity[0] + st.velocity[1] * st.velocity[1] + st.velocity[2] * st.velocity[2]);
    h.rpm = st.rpm;                    // port: +0x1280 (FUEL's needle reads the +0x1284 mix, 0x529ce0)
    h.damage = st.damage;              // 0x4235b0: the largest slot of damage record 1 (P46)
    h.maxRpm = d.setup(0).maxRpm;      // VehiclePP 324
    h.nitro = {st.nitro.on, st.nitro.level, st.nitro.amount, st.nitro.active, st.nitro.emptyFlash, st.nitro.bonusGlow,
               st.nitro.tankSeconds, st.nitro.minStartLevel};  // [nitro]; [upgrades U1] the vehicle's tank
    // ---- minimap-fix: the radar banner's inputs (0x4a53e0 gate, 0x4a54b0 status): the mission state +0x48 as gameSound
    // has it (3 while TELEPORT_HELICO holds the countdown), its hub list (0x577d70), the teleport machine and GAMEStarted.
    if (race) {
        const fuel::RaceRules& R = race->rules;
        h.missionState = R.state == fuel::RaceRules::Countdown ? 4 : R.state == fuel::RaceRules::Running ? 5 : R.outcome == fuel::RaceRules::Won ? 6 : 7;
        if (h.missionState == 4 && gRtc.active() && gRtc.name() == "TELEPORT_HELICO") h.missionState = 3;
        if (gOverlay) h.missionList = gOverlay->menu().missionList(race->event().id);
    }
    h.teleporting = gOverlay && gOverlay->logoState() != 0;
    h.gameStarted = !gOverlay || gOverlay->gameStarted();
    // The banner's hub [O asm 0x4a554a..0x4a561a] (PK03's rule, now for the live HUD and the film frames alike): the hub of
    // the zone holding the player (0x421f60 -> 0x49e980 -> 0x4a7f40 Zone_FindContaining -> 0x577500), its name TT (0x579140
    // -> 0x677d20). In no zone the uid is -1: the name is TT 998 "OUT OF AREA" (0x677cb0(0x3e6)) and every frame restarts
    // the fade (bVar1), so the banner stays up. (Also out of area: game +0x32bc == 2 with the vehicle outside the world,
    // 0x528160 [not ported]; the world-edge fade 0x52b4b0 called there is not ported, pk17.) Without the zones (no GenWorld
    // tail) the nearest hub [port].
    if (gOverlay) {
        const simd_float3 p = d.position();
        const auto& hubs = gOverlay->menu().hubs();
        const fuel::Hub* hub = nullptr;
        if (!gHubZones.zones.empty()) {
            const int z = gHubZones.find(p.x, p.z);
            if (z >= 0) hub = fuel::findHub(hubs, gHubZones.zones[size_t(z)].name);
        } else {
            hub = fuel::nearestHub(hubs, p.x, p.z);
        }
        h.outOfArea = hub == nullptr;
        h.hubName = hub ? (hub->nameText ? gOverlay->menu().textOf(hub->nameText) : hub->id) : gOverlay->menu().textOf(998);
    }
    // ---- minimap-fix end
    const simd_float3 at = d.position();
    h.px = at.x, h.pz = at.z;
    h.quat = st.chassis.rotation;
    h.opponents.clear();
    // 0x4ab920: MAP_CONCURRENT_CAR_ICON only for the current race's racers (0x62abd0 = the race; none in free roam).
    // The session's other vehicles (free-roam traffic, its idle pool parked above the camera) get no icon.
    const int racers = race ? std::min(d.vehicleCount(), int(race->racers.size())) : 0;
    for (int i = 1; i < racers; ++i) {
        const simd_float3 o = d.position(i);
        const float yaw = d.heading(i);
        h.opponents.push_back({o.x, o.z, std::sin(yaw), std::cos(yaw)});
    }
    h.py = at.y;
    h.gps3d = gGps3dVisible;
    {  // P56: the camera for the Navigator3dHUD band (compass, screen icons) in races and free roam
        const simd_float3 f = simd_normalize(cam.target - cam.eye);
        const simd_float3 rt = simd_normalize(simd_cross(f, simd_make_float3(0, 1, 0)));
        const simd_float3 up = simd_cross(rt, f);
        const float fovY = cam.fovH43 > 0 ? 2.0f * std::atan(std::tan(cam.fovH43 * 0.5f) * 0.75f) : cam.fovY;
        h.eye = {cam.eye.x, cam.eye.y, cam.eye.z}, h.camF = {f.x, f.y, f.z}, h.camR = {rt.x, rt.y, rt.z}, h.camU = {up.x, up.y, up.z};
        h.camFy = cam.explicitPose ? 360.0f / std::tan(fovY * 0.5f) : 0.0f;
    }
    if (!race && gFreeRoam) gFreeRoam->fillSpecials(h);  // P78: minimap doppler / bonus-car icons
    if (!race) return freeRoamGps(h, d, dt, cam, gOverlay ? &gOverlay->menu() : nullptr);  // P44: the map target's GPS
    if (race->racers.size() < 1) return;
    const fuel::RaceRules& R = race->rules;
    const fuel::Race& r = race->race();
    // port: FUEL picks the page by scenario (race +0x27c); the port maps the effective game mode back.
    switch (R.mode) {
        case fuel::Race::GmAtoB: h.page = P::AtoB; break;
        case fuel::Race::GmCheckpoint: h.page = P::Checkpoint; break;
        case fuel::Race::GmKnockOut: h.page = P::KnockOut; break;
        case fuel::Race::GmTimeTrial: h.page = P::TimeTrial; break;
        case fuel::Race::GmTimeBonus: case fuel::Race::GmTimeLimitCheckpoints: h.page = P::TimeTrialCheckpoint; break;
        case fuel::Race::GmCircuit: h.page = P::Circuit; break;
        case fuel::Race::GmHelicopter: h.page = P::Helico; break;
        case fuel::Race::GmCatchAI: h.page = P::CatchAI; break;
        default: h.page = P::None; break;  // raid, baja (scenario 9 modes 5/6 fill nothing)
    }
    const size_t next = std::min(race->next(), race->route.size());
    const int laps = std::max(1, r.laps);
    const size_t perLap = r.loops() ? std::max<size_t>(1, (race->route.size() - 1) / size_t(laps)) : race->route.size();
    h.topA = int(next), h.topB = int(race->route.size());                // 0x4b4c10: gates passed / gates x laps
    if (h.page == P::Circuit) h.topA = int(std::min<size_t>(next / perLap + 1, size_t(laps))), h.topB = laps;  // 0x4b4cb0
    int left = 0;
    for (size_t i = 1; i < R.racers.size(); ++i) left += !R.racers[i].out;
    if (h.page == P::KnockOut) h.bottomA = left, h.bottomB = int(R.racers.size()) - 1;
    if (h.page == P::CatchAI) h.topA = int(R.racers.size()) - 1 - left, h.topB = int(R.racers.size()) - 1;
    const auto pos = race->where(d);
    // port: remaining distance = course length - progress (FUEL: racer +0x170 = IA path left, 0x5c21a0).
    auto remaining = [&](size_t i) {
        const fuel::RaceLine& ln = race->lineFor(i);
        if (R.racers[i].finished >= 0 || R.racers[i].next >= ln.points.size()) return 0.0f;
        const size_t g = R.racers[i].next;
        if (ln.gateAlong.size() >= 2 && ln.path.size() >= 2) {  // PK16: FUEL's record +0x170 (0x5c21a0 / 0x628c40)
            const size_t k = std::min(g, ln.gateAlong.size() - 2);
            return std::max(0.0f, race->leftOf(i, pos[i][0], pos[i][2], ln.project(pos[i][0], pos[i][2], ln.gateAlong[k], ln.gateAlong[k + 1])));
        }
        return std::max(0.0f, ln.length() - ln.progress(g, pos[i][0], pos[i][2]));
    };
    h.distance = pos.empty() ? 0 : remaining(0);
    if (R.timed) h.time = R.timeLeft, h.timeTotal = r.timeLimit;
    else h.time = float(std::max(0.0, R.clock));
    h.countdown = R.state == fuel::RaceRules::Countdown ? float(-R.clock) : R.state == fuel::RaceRules::Running ? float(-R.clock) : -10.0f;
    // Ranking lines: positions with the remaining metres (ResultRow::time).
    h.standings.clear();
    if (race->racers.size() > 1) {
        const std::vector<int> order = R.positions(pos);
        for (int p = 1; p <= int(race->racers.size()); ++p)
            for (size_t i = 0; i < race->racers.size() && i < pos.size(); ++i)
                if (order[i] == p) h.standings.push_back({race->racers[i].name, remaining(i), i == 0, race->racers[i].nameTT});
    }
    // Camera for the in-world ribbons (eye/target pose; roll ignored).
    {
        const simd_float3 f = simd_normalize(cam.target - cam.eye);
        const simd_float3 rt = simd_normalize(simd_cross(f, simd_make_float3(0, 1, 0)));
        const simd_float3 up = simd_cross(rt, f);
        const float fovY = cam.fovH43 > 0 ? 2.0f * std::atan(std::tan(cam.fovH43 * 0.5f) * 0.75f) : cam.fovY;
        h.eye = {cam.eye.x, cam.eye.y, cam.eye.z}, h.camF = {f.x, f.y, f.z}, h.camR = {rt.x, rt.y, rt.z}, h.camU = {up.x, up.y, up.z};
        h.camFy = cam.explicitPose ? 360.0f / std::tan(fovY * 0.5f) : 0.0f;
        const auto& ln = race->line;
        if (R.state == fuel::RaceRules::Running && ln.path.size() >= 2 && race->terrain) {
            const simd_float3 me = d.position();
            gGps.update(dt, {me.x, me.y, me.z}, h.eye, h.camF, ln.path, ln.pathAlong,
                        [t = race->terrain](float x, float z) { return t->smoothHeightAt(x, z); });
            h.ribbons = gGps.ribbons();
            if (!gGps3dVisible) h.ribbons.clear();  // 0x4c0fe0: DAT_00a6536e skips the draw
        } else {
            gGps.reset();
        }

    }
    h.raceLine = race->line.path;
    // ---- PK19 0x4a1bd0 -> 0x629990: on a world course with racing lanes (SplineGraph_Z) the minimap's race line is the
    // circuit built from the spline blocks' middle-lane edges, a point every 10 m. [port] Other races keep the AI path
    // as the line: there FUEL draws no race line (0x629990 returns 0 points) but the GPS samples of 0x4a2250 (30 x 10 m
    // ahead, GPS style +0x19e4), which the port's minimap does not draw (outside PK19).
    if (race->race().splineGraph && !race->line.splineEdges.empty()) {
        auto circuit = fuel::hud::circuitLine(*race->race().splineGraph, race->line.splineEdges);
        if (circuit.size() >= 2) h.raceLine = std::move(circuit);
    }
    // ---- PK19 end
    if (h.raceLine.size() < 2) {
        h.raceLine.clear();
        for (const auto& q : race->line.points) h.raceLine.push_back({q[0], q[2]});
    }
    h.gates.clear();
    for (const auto& g : race->route) h.gates.push_back({g[0], g[2]});
    h.gateY.clear();
    for (const auto& g : race->route) h.gateY.push_back(g[1]);
    h.helico = R.mode == fuel::Race::GmHelicopter;
    h.nextGate = next;
}
// P86: the HUD state, and the GPS arrows it carries for the renderer's scene pass (0x4c0fe0 draws them only while
// the 3-D GPS is on).
static void fillHud(fuel::ui::HudState& h, const DriveSession& d, const RaceSession* race, float dt, const fuel::OrbitCamera& cam) {
    fillHudState(h, d, race, dt, cam);
    gGpsDraw.clear();
    if (h.gps3d)
        for (const auto& rb : h.ribbons) gGpsDraw.push_back({{rb.points.begin(), rb.points.end()}, rb.up, rb.alpha});
    // ---- PK19: the band icons' FX_NAVIGATOR3D_LINE beams (0x4bfc60 -> 0x4bf820), drawn by the renderer like the GPS arrows.
    gNavBeams.clear();
    if (gOverlay && !gHudHidden)
        for (const auto& b : gOverlay->menu().navBeams(h)) gNavBeams.push_back({{b.points.begin(), b.points.end()}, b.colour, b.halfWidth});
    // ---- PK19 end
}

// P69: FUEL's race markers for this frame (marker_view.h): the running mission's banners and flags (0x4f73b0), else the
// free-roam start banners (0x4f83d0). The player's target in the lap = gates passed % (n + 1) (0x4f73b0, race +0x220).
static void stepRaceMarkers(const RaceSession* race, simd_float3 player, fuel::MeshRenderer& r) {
    if (!gRaceMarkers.ready()) return;
    if (!race) return gRaceMarkers.update(nullptr, -1, player, r);
    const size_t perLap = race->race().checkpoints.size() + 1;
    gRaceMarkers.update(&race->race(), int(race->next() % perLap), player, r);
}

// ---- PK14: KillMission (console 0x5490a0 -> 0x62c7d0, then 0x62baa0(mission, argc == 2) while a mission runs).
// In FUEL it runs from TTMN 0x54a910 (a retry: the end menu's MENU_VALI2 and the race page's restart 0x49b0a0, before
// the "TLP x z yaw" to the start), from `KillMission 1` (WhatNext's free ride / the race tab 0x49b190), the online
// leave and the save load 0x5539f0. The port restarts its session for the free ride, TeleportToHub and the load (P78 /
// P99: a new process, the world fresh by construction), so only the retry runs it in a live world.
// 0x62baa0, offline, in order (notes/pk14.md):
//   MapHUD +0x1ae4 = 2 / +0x1ae8 = 5.0 s       Menu::hudMissionKilled (the radar status line, 0x4a54b0: minimap-fix)
//   0x54bee0 CloseFogOfWar                       no fog nodes in retail (b042): nothing
//   0x5b96a0 tornadoes released                  StormView::killMission
//   0x515be0 / 0x51df10 breakables, props        WorldStreamer::killMission (+ the course sub-world and 0x62c850 below)
//   console "ton" (TrafficON)                    Traffic::enabled (the port runs no traffic during a race anyway)
//   +0x67 = 1, rich presence (GFWL)              [online / GFWL: out of scope]
//   record +0x178 nodes hidden, +0x20b = 0       the popping checkpoint nodes (not shown by the port: nothing to hide)
//   WeatherManager +0x114..+0x11b = 0            MeshRenderer::endRaceWeather (+ the storm view's race wind +0x118)
//   Mission::SetState(0)                         the end grade ramps down (CutsceneFx::missionLeft), race state 0, M_END
//   0x620cd0 / 0x5c0ed0 AI racers released       RaceSession::begin restarts every racer [by construction]
//   0x62c850 started IT_ANIMATED reset           WorldStreamer::killMission
//   MissionHUD +0xd74.. timers, key repeat       Menu::raceRestarted (the end pages' state) [by construction]
//   road one-way edges + router cache cleared    the port sets no one-way edges (0x62d710's 0x6d2030 not ported): nothing
//   0x512920(0) every object condition           WorldStreamer::killMission (setMission(""))
//   player vehicle +0x240 / +0x244 = 47          RiderAnimator::killMission
//   IGP +0x4c7c = the VIGNETTAGE material         the port's vignette always draws VIGNETTAGE (P102): nothing
// 0x4f5b90 on one event's mission record (core/races.hpp): y = the coarse height 0x445570 (Heightfield::baseHeightAt).
static void snapMissionHeights(fuel::Event& e, const fuel::Heightfield& terrain) {
    fuel::snapMissionHeights(e.race, [&terrain](float x, float z) { return terrain.raw.empty() ? 0.0f : terrain.baseHeightAt(x, z); });
}

static void killMission(RaceSession* race, DriveSession& drive, WorldStreamer* streamer, fuel::MeshRenderer& renderer) {
    // 0x62c7d0: a race-end VI_BIKE still queued / playing is ended (0x536230(agent, 1, 0)) and "SetHudsVisibleState ON"
    // (the port's HUD follows the RTC each frame: gHudHidden).
    if (gRtc.active() && gRtc.name() == "VI_BIKE") gRtc.stop(renderer);
    if (!race) return;  // 0x5490a0: 0x62baa0 only with a mission past state 0 / 1
    if (gOverlay) gOverlay->menu().hudMissionKilled();  // minimap-fix: 0x62baa0 offline: MapHUD +0x1ae4 = 2, +0x1ae8 = 5.0
    gStorms.killMission();
    WorldStreamer::KillReport rep;
    if (streamer) rep = streamer->killMission(renderer, drive.physics());
    if (gTraffic) gTraffic->traffic().enabled = true;
    renderer.endRaceWeather();
    gCutFx.missionLeft();
    drive.setRaceState(0);
    drive.riderKillMission(0);
    std::printf("KillMission %s: %zu pieces released, %zu props + %zu AutoMesh props reset, %zu course decor whole, %zu animations reset\n",
                race->event().id.c_str(), rep.pieces, rep.props, rep.autoProps, rep.courseFresh, rep.anims);
}
// --retry-check: "retry after knocking props shows props back in place". The player knocks a prop near the spot (--at,
// else the race start) at full throttle, then the race is retried as the pause menu's RESTART does (killMission, then
// RaceSession::begin: TTMN's KillMission and TLP to the start), and the player comes back near the prop: every prop
// knocked must stand on its placement again, nothing displaced, no break pieces left.
static int runRetryCheck(RaceSession& race, DriveSession& drive, WorldStreamer& streamer, fuel::MeshRenderer& renderer, bool haveAt, float atX,
                         float atZ) {
    const float dt = 1.0f / 60.0f;
    auto frames = [&](int n, const fuel::DriveInput& in) {
        for (int i = 0; i < n; ++i) @autoreleasepool {
                drive.setRaceState(5);  // RUNNING: no pre-start hold (PK01)
                streamer.breakThrough(drive, dt, renderer, drive.physics());
                drive.update(dt, in);
                streamer.update(drive.position(), renderer, drive.physics());
            }
    };
    const simd_float3 spot = haveAt ? simd_make_float3(atX, 0, atZ) : simd_make_float3(race.race().start[0], 0, race.race().start[2]);
    drive.teleport(spot.x, spot.z, 0);
    streamer.update(drive.position(), renderer, drive.physics(), true);
    const simd_float3 from = drive.position();
    std::vector<simd_float3> tried;
    simd_float3 target{};
    std::vector<WorldStreamer::PropPlace> knocked;
    for (int trial = 0; trial < 16 && knocked.empty(); ++trial) {
        if (trial % 4 == 0) {  // the next prop, from the spot's side first, then from three other sides
            if (!streamer.nearestProp(from, tried, target)) break;
            tried.push_back(target);
        }
        const float yaw = std::atan2(target.x - from.x, target.z - from.z) + float(trial % 4) * float(M_PI) * 0.5f;
        drive.teleport(target.x - 45 * std::sin(yaw), target.z - 45 * std::cos(yaw), yaw);
        streamer.update(drive.position(), renderer, drive.physics(), true);
        std::printf("retry check: aiming at the prop at (%.1f, %.1f) from 45 m\n", target.x, target.z);
        for (int t = 0; t < 9 * 60 && streamer.propStats().moved == 0; ++t) frames(1, fuel::DriveInput{1.0f, 0, 0, false});
        frames(90, fuel::DriveInput{1.0f, 0, 0, false});  // the aftermath
        for (const auto& p : streamer.propsNear(target.x, target.z, 60.0f))
            if (p.displacement > 0.3f) knocked.push_back(p);
        const simd_float3 q = drive.position();
        std::printf("retry check:   %zu props moved, vehicle at (%.1f, %.1f) %.0f km/h, %.1f m from the prop\n", streamer.propStats().moved, q.x, q.z,
                    drive.state().speedKmh, simd_distance(simd_make_float3(q.x, 0, q.z), simd_make_float3(target.x, 0, target.z)));
    }
    if (knocked.empty()) {
        std::printf("retry check: FAILED (no prop knocked near (%.0f, %.0f))\n", spot.x, spot.z);
        return 1;
    }
    float far = 0;
    for (const auto& p : knocked) far = std::max(far, p.displacement);
    std::printf("retry check: %zu props knocked (furthest %.1f m off its placement), %zu moved in all, %zu break pieces\n", knocked.size(), far,
                streamer.propStats().moved, streamer.piecesNow());
    // The retry: RESTART -> TTMN "<mission>" -> KillMission, then the TLP to the start and the mission's start.
    if (!std::getenv("FUEL_RETRY_NO_KILL")) killMission(&race, drive, &streamer, renderer);  // the variable: the control run (must fail)
    size_t stillOff = 0;  // right after KillMission (0x51df10): no knocked prop is off its placement any more
    for (const auto& k : knocked)
        for (const auto& p : streamer.propsNear(k.origin.x, k.origin.z, 1.0f))
            stillOff += simd_distance(p.origin, k.origin) < 0.01f && p.displacement > 0.05f;
    std::printf("retry check: after KillMission: %zu knocked props still off their placement, %zu break pieces\n", stillOff, streamer.piecesNow());
    race.begin(drive, &streamer, renderer);
    frames(30, fuel::DriveInput{});
    const auto atStart = streamer.propStats();
    std::printf("retry check: after the retry at the start: %zu props moved, %zu break pieces\n", atStart.moved, streamer.piecesNow());
    // Back to the props (40 m off the target, facing away, at rest).
    const float yaw = std::atan2(target.x - from.x, target.z - from.z);
    drive.teleport(target.x - 40 * std::sin(yaw), target.z - 40 * std::cos(yaw), yaw + float(M_PI));
    streamer.update(drive.position(), renderer, drive.physics(), true);
    frames(120, fuel::DriveInput{});
    size_t back = 0;
    for (const auto& k : knocked) {
        bool found = false;
        for (const auto& p : streamer.propsNear(k.origin.x, k.origin.z, 1.0f))
            if (simd_distance(p.origin, k.origin) < 0.01f && p.displacement < 0.05f) found = true;
        back += found;
        std::printf("retry check: prop placed at (%.1f, %.1f, %.1f): %s\n", k.origin.x, k.origin.y, k.origin.z, found ? "back in place" : "NOT back");
    }
    const auto after = streamer.propStats();
    const bool ok = stillOff == 0 && back == knocked.size() && atStart.moved == 0 && after.moved == 0 && streamer.piecesNow() == 0;
    std::printf("retry check: %zu / %zu knocked props back in place, %zu props moved after the retry: %s\n", back, knocked.size(), after.moved,
                ok ? "ok" : "FAILED");
    return ok ? 0 : 1;
}
// ---- end PK14

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


// FUEL's AI classes (IA_RaceSetup 0x620e30): 1..3 racers of class 0 and 1..3 of class 2 (random), the
// rest class 1, handed out from the last racer back (class 2 first), each with its share k / count of
// its class; knock-out races use class 4, catch-AI challenges class 3. Grid row: two abreast (0x6212e0).
void assignAiClasses(RaceSession& race, std::mt19937& rng) {
    const int n = int(race.racers.size()) - 1;
    const int gm = int(race.race().gameMode());
    const int c0 = 1 + int(rng() % 3), c2 = 1 + int(rng() % 3);
    int left2 = c2, left1 = n - c2 - c0, left0 = c0;
    const int c1 = std::max(left1, 1);
    for (int i = n; i >= 1; --i) {
        int mode = 0;
        float frac = 1;
        if (gm == fuel::Race::GmKnockOut) mode = 4;
        else if (gm == fuel::Race::GmCatchAI) mode = 3;
        else if (left2 > 0) mode = 2, frac = float(left2--) / float(c2);
        else if (left1 > 0) mode = 1, frac = float(left1--) / float(c1);
        else mode = 0, frac = float(left0--) / float(c0);
        race.racers[size_t(i)].ai = fuel::RaceDriver(mode, frac, i / 2);
    }
    race.racers[0].ai = fuel::RaceDriver(1, 1, 0);
    // PK16 0x620e30 / 0x620c20: racer k gets the k-th entry of the per-race shuffle of the text ids 1500..1573.
    const std::array<int, 74> names = fuel::aiNameShuffle();
    for (int i = 1; i <= n; ++i) race.racers[size_t(i)].nameTT = names[size_t(i - 1) % 74];
}

// [upgrades U4] The hub tier the opponents' stages come from (core/upgrades aiDraw): `--ai-upgrade-tier N` (the menu passes
// the event's hub tier) or FUEL_AI_UPGRADE_TIER; else the event's own hub (career and challenge events: SetHubTargetInfos'
// tier; an edited race: the hub zone under its start, P75's GenWorld_Z zones, -1 outside every hub = stock). A challenge that
// hands over its vehicle (AddMissionNeededVehicle, the player driving it) runs the player stock (U2): its opponents too.
int aiUpgradeTier(const RaceSession& race, const std::string& playerVehicle) {
    if (const int t = fuel::upg::aiHubTier(); t != fuel::upg::kAiTierAuto) return t;
    const fuel::Event& e = race.event();
    if (e.kind == fuel::Event::Challenge && !e.vehicle.empty() && strcasecmp(e.vehicle.c_str(), playerVehicle.c_str()) == 0) return fuel::upg::kAiTierNone;
    if (e.kind != fuel::Event::Edited) return e.hub.empty() ? fuel::upg::kAiTierNone : e.tier;
    const int z = gHubZones.find(e.race.start[0], e.race.start[2]);
    if (z < 0) return fuel::upg::kAiTierNone;
    for (const fuel::Event& o : race.races)
        if (o.kind != fuel::Event::Edited && strcasecmp(o.hub.c_str(), gHubZones.zones[size_t(z)].name.c_str()) == 0) return o.tier;
    return fuel::upg::kAiTierNone;
}

// Puts `count` opponents of the player's class into the drive session and the race,
// each with a random livery and colour of its own, and hands their parts to the renderer.
// [upgrades U4] Each opponent's vehicle gets its stages (aiDraw for grid slot k, the race's hub tier `hubTier`) through
// U1's transform before the physics builds it; its ghost's factors are kept on the racer (RaceSession::begin, drive).
void addOpponents(Garage& garage, DriveSession& drive, RaceSession& race, int count, fuel::MeshRenderer& renderer, const VehicleLook& prefs,
                  int hubTier = fuel::upg::kAiTierNone) {
    std::mt19937 rng(fuel::asoboCrc(race.race().name));
    race.racers.resize(1);
    race.upgTier = hubTier;  // [upgrades U4]
    race.racers[0].name = garage.label(garage.current);
    race.racers[0].entry = garage.current;
    std::vector<std::vector<fuel::MeshInstance>> parts;
    std::vector<fuel::Paint> paints;
    const simd_float3 at = drive.position();
    // FUEL's opponent vehicles (P43): AI racer k drives the event's AddMissionParam vehicle k mod n (0x620e30);
    // a vehicle whose DPC the garage did not load (or an event without a list) takes the port's pick.
    std::vector<size_t> picks = garage.rivals(garage.current, count, rng);
    if (const auto& want = race.event().aiVehicleLabels; !want.empty()) {
        int fromList = 0;
        for (int k = 0; k < count && size_t(k) < picks.size(); ++k) {
            const std::string& w = want[size_t(k) % want.size()];
            for (size_t e = 0; e < garage.entries.size(); ++e)
                if (garage.label(e) == w) {
                    picks[size_t(k)] = e, ++fromList;
                    break;
                }
        }
        std::printf("AI vehicles: %d of %d from the event's list (%s)\n", fromList, count, race.event().aiVehicles.c_str());
    }
    for (size_t entry : picks) {
        fuel::AssembledVehicle av;
        fuel::VehicleSetup setup;
        std::vector<uint32_t> rider;
        VehicleLods lods;  // P104
        if (!garage.load(entry, av, setup, rider, &lods)) continue;
        // [upgrades U4] the opponent's stages: drawn per race (the event id) and grid slot, applied as the player's are.
        RaceSession::Racer racer;
        const int slot = int(race.racers.size()) - 1;
        racer.upgDraw = fuel::upg::aiDraw(race.event().id, slot, hubTier, fuel::upg::aiSeedOverride());
        racer.upgResult = fuel::upg::apply(setup, racer.upgDraw.stages, fuel::fv::FuelVehicle::powerScale);
        racer.upg = fuel::upg::aiFactors(racer.upgDraw.stages, racer.upgResult.cls, fuel::fv::FuelVehicle::powerScale);
        std::printf("ai upgrades: #%d %s: %s\n", slot + 1, setup.name.c_str(), fuel::upg::describeAi(racer.upgDraw, racer.upgResult, racer.upg).c_str());
        const int i = drive.addVehicle(av, setup, at.x, at.z + 30.0 * drive.vehicleCount(), 0, rider, &lods);  // the grid comes with RaceSession::begin
        racer.name = garage.label(entry);
        racer.entry = entry;
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
    assignAiClasses(race, rng);
}

// ---- P104: FUEL's vehicle LOD for this frame's camera eye (Lod_Z::Draw 0x884170 with the LodData flags of 0x5c7fe0):
// each rig's levels (before uploadRiders: only the drawn rider level is skinned) and the renderer's per-part alphas.
// pushVehicleLods: the rigs' current levels as they are (showOpponents: paths without a camera eye yet).
void pushVehicleLods(const DriveSession& drive, fuel::MeshRenderer& renderer) {
    for (int i = 1; i < drive.vehicleCount(); ++i) {
        const VehicleRig& r = drive.rig(i);
        fuel::MeshRenderer::VehicleLodState st;
        if (r.lodSwitched()) {
            st.anchor = r.lodAnchor(), st.riderAnchor = r.riderAnchor();
            st.level = r.level(), st.alpha = r.alpha();
            st.riderLevel = r.hasRider() ? r.riderLevel() : -1, st.riderAlpha = r.riderAlpha();
        }
        renderer.setVehicleLod(size_t(i - 1), r.partAlphas(), st, r.partAlphas(true));
    }
}
void applyVehicleLods(DriveSession& drive, fuel::MeshRenderer& renderer, simd_float3 eye) {
    drive.updateLods(eye);
    pushVehicleLods(drive, renderer);
    static const bool trace = std::getenv("FUEL_P104_TRACE") != nullptr;
    if (trace)
        for (int i = 1; i < drive.vehicleCount(); ++i) {
            const VehicleRig& r = drive.rig(i);
            const auto a = r.partAlphas();
            int on = 0;
            for (float x : a) on += x > 0;
            std::printf("p104: vehicle %d d %.1f level %d alpha %.2f rider %d level %d alpha %.2f d %.1f, %d of %zu parts drawn\n", i,
                        simd_distance(r.lodAnchor(), eye), r.level(), r.alpha(), int(r.hasRider()), r.riderLevel(), r.riderAlpha(),
                        simd_distance(r.riderAnchor(), eye), on, a.size());
        }
}
// ---- P104 end

// Moves the opponents' parts to their physics poses.
void showOpponents(DriveSession& drive, fuel::MeshRenderer& renderer) {
    for (int i = 1; i < drive.vehicleCount(); ++i) renderer.setVehicleWorlds(size_t(i - 1), drive.partWorlds(i));
    pushVehicleLods(drive, renderer);  // P104
    if (gTraffic) gTraffic->hideIdle(drive, renderer);
    setCarOcclusion(drive, renderer);  // P74
    gDamageView.update(drive, renderer);  // P85: the morph damage of 0x5c7760
}

// P58: free-roam traffic. A pool of 5 fleet jeeps and 5 big trucks (BONUS1.DPC FLOTTE_JEEP_1 / FLOTTE_BIGTRUCK_1,
// FUEL's MESH_ID_FLOTTE_*) joins the session as vehicles 1..10 (free roam only: the race code owns vehicles 1..n);
// FUEL_NO_TRAFFIC=1 turns it off (FUEL's console TrafficOFF).
std::unique_ptr<TrafficView> addTraffic(Garage& garage, DriveSession& drive, fuel::MeshRenderer& renderer, const fuel::RoadGraph& graph,
                                        const std::array<fuel::RoadTypeParams, 21>& types, const fuel::Heightfield& terrain, const VehicleLook& prefs) {
    if (const char* e = std::getenv("FUEL_NO_TRAFFIC"); e && std::atoi(e) != 0) return nullptr;
    std::array<std::vector<int>, 3> pool;
    std::vector<std::vector<fuel::MeshInstance>> parts;
    std::vector<fuel::Paint> paints;
    // P61: model 2 = the special catchable truck (AddHubTrafficVehicle MESH_ID_TRAFFIC_TRUCK -> TRAFFIC_TM), 2 of them.
    const char* labels[3] = {"FLOTTE_JEEP_1", "FLOTTE_BIGTRUCK_1", "TRAFFIC_TM"};
    const simd_float3 at = drive.position();
    for (int m = 0; m < 3; ++m) {
        size_t entry = garage.entries.size();
        for (size_t e = 0; e < garage.entries.size(); ++e)
            if (garage.label(e) == labels[m]) entry = e;
        if (entry == garage.entries.size()) {
            std::printf("traffic: no %s in the garage (BONUS1.DPC); %s\n", labels[m], m < 2 ? "no traffic" : "no special vehicles");
            if (m < 2) return nullptr;
            continue;
        }
        for (int k = 0; k < (m < 2 ? fuel::Traffic::kSlots : 2); ++k) {
            fuel::AssembledVehicle av;
            fuel::VehicleSetup setup;
            std::vector<uint32_t> rider;
            VehicleLods lods;  // P104
            if (!garage.load(entry, av, setup, rider, &lods)) return nullptr;
            const int i = drive.addVehicle(av, setup, at.x + 40.0 * (m * 5 + k + 1), at.z, 0, rider, &lods);
            pool[size_t(m)].push_back(i);
            const AssemblyMode& am = *garage.archives[garage.entries[entry].first];
            VehicleLook look = makeLook(labels[m], am.assembler->variants()[garage.entries[entry].second].name, am.palettes, am.assembler->dpc(), prefs);
            look.custom = false;
            look.color = k;  // port: a fixed colour per pool vehicle (FUEL: vehicle +0x514 = the spawn's colour 0..9)
            paints.push_back(look.paint());
            parts.push_back(drive.parts(i));
        }
    }
    renderer.setVehicles(parts, paints);
    renderer.shadowRadius = 22.0f;
    fuel::Traffic::World w;
    w.flatHalf = [&types](int t) { return t >= 0 && t < 21 ? types[size_t(t)].flat : 0.0f; };
    w.ground = [&terrain](float x, float z, float n[3]) {
        const float h = terrain.smoothHeightAt(x, z);
        const float hx = terrain.smoothHeightAt(x + 1, z) - terrain.smoothHeightAt(x - 1, z);
        const float hz = terrain.smoothHeightAt(x, z + 1) - terrain.smoothHeightAt(x, z - 1);
        const float l = std::sqrt(hx * hx * 0.25f + 1 + hz * hz * 0.25f);
        n[0] = -hx * 0.5f / l, n[1] = 1 / l, n[2] = -hz * 0.5f / l;
        return h;
    };
    std::printf("traffic: %zu jeeps, %zu big trucks and %zu special trucks in the pool\n", pool[0].size(), pool[1].size(), pool[2].size());
    return std::make_unique<TrafficView>(graph, std::move(w), std::move(pool));
}

// PK07: the race helicopter's rotor (IT_HelicoMove vf17 0x515490: the agent's bank symbol 0 of IT_HELICO, flags 0x10f,
// on the helicopter object; node sound distances 200 / 600 m from the mover init 0x5153a0). A loop for as long as the
// race object exists (it ends with the race).
void heliSounds(fuel::DriveAudio& audio, const RaceSession* race) {
    static const uint32_t kHelico = fuel::asoboCrc("IT_HELICO");
    const bool on = race && race->rules.helicopter && race->state != RaceSession::Finished;
    const auto& h = race ? race->rules.hel.disp : std::array<float, 3>{};
    audio.objectLoop(0x7b0001, kHelico, 0, {h[0], h[1], h[2]}, on, 200.0f, 600.0f);
}

// ---- PK27: the F14 fly-by, once per frame the world runs (F14Move_G vf17 0x51a050, then the node draw and, with the
// sounds, the emitter 0x4ea560). The player = the vehicle (0x420330 position, 0x421ab0 forward); camera 0 = `eye`.
// FUEL_F14=S [debug]: the first pass S s after the world loads (FUEL: PP80 - 45 -> 45 s); FUEL_F14_RAND=r [debug]: every
// heading draw returns r; FUEL_F14_TRACE=1 prints the passes.
void f14Init(const fuel::DpcFile& world, const std::string& dataDir, const fuel::Heightfield* terrain) {
    gF14Ready = false;
    if (!terrain || !gF14View.load(world)) {
        std::printf("F14: no F14 object in the world (or no world map): no fly-by\n");
        return;
    }
    const auto pp = fuel::hud::loadPp(dataDir + "/CommonComponent/Param/pp001.tsc");
    auto row = [&pp](size_t n, float d) { return n < pp.size() ? pp[n] : d; };
    gF14.init(row(80, 900.0f), row(81, 250.0f), row(82, 80.0f));
    if (const char* e = std::getenv("FUEL_F14")) gF14.timer = gF14.pp80 - float(std::atof(e));
    gF14.ground = [terrain](float x, float, float z) { return terrain->baseHeightAt(x, z); };  // 0x445570: the cubic map height
    static uint32_t seed = 0x2545f491u;  // [port] the game's 0x6460f0 sequence is shared with everything; a Park-Miller of our own
    gF14.rand01 = [] {
        if (const char* e = std::getenv("FUEL_F14_RAND")) return float(std::atof(e));
        seed = uint32_t((uint64_t(seed) * 48271u) % 0x7fffffffu);
        return float(seed) / float(0x7fffffff);
    };
    gF14Ready = true;
    std::printf("F14: fly-by every %.0f s (first in %.0f s), %.0f m/s, %.0f m over the ground\n", gF14.pp80, gF14.pp80 - gF14.timer, gF14.pp81, gF14.pp82);
}

void f14Step(float dt, DriveSession& drive, fuel::MeshRenderer& r) {
    if (!gF14Ready || !(dt > 0)) return;
    const simd_float3 p = drive.position();
    const float h = drive.heading();
    const bool wasShown = gF14.shown;
    static const bool trace = std::getenv("FUEL_F14_TRACE") != nullptr;
    static double worstMs = 0;
    const double t0 = trace ? CACurrentMediaTime() : 0;
    gF14.step(dt, true, true, {p.x, p.y, p.z}, {std::sin(h), 0.0f, std::cos(h)});
    gF14View.frame(&gF14, r);
    if (trace) {
        worstMs = std::max(worstMs, (CACurrentMediaTime() - t0) * 1000.0);
        if (gF14.shown != wasShown) {
            std::printf("F14: pass %d %s at (%.0f, %.0f, %.0f), velocity (%.0f, %.0f, %.0f), player (%.0f, %.0f, %.0f); worst step + draw list %.3f ms\n",
                        gF14.passes, gF14.shown ? "starts" : "ends", gF14.nodePos[0], gF14.nodePos[1], gF14.nodePos[2], gF14.vel[0], gF14.vel[1],
                        gF14.vel[2], p.x, p.y, p.z, worstMs);
            worstMs = 0;
        }
    }
}

void f14Sounds(fuel::DriveAudio& audio, float dt, simd_float3 eye) {
    if (!gF14Ready) return;
    const float pitch = gF14.doppler(dt, {eye.x, eye.y, eye.z});
    audio.flyby(gF14.passes, gF14.soundOn, {gF14.nodePos[0], gF14.nodePos[1], gF14.nodePos[2]}, pitch);
    if (std::getenv("FUEL_F14_TRACE") && gF14.soundOn) {
        static double acc = 0;
        acc += dt;
        if (acc >= 0.5) {
            acc = 0;
            const float d = std::sqrt(gF14.relPrev[0] * gF14.relPrev[0] + gF14.relPrev[1] * gF14.relPrev[1] + gF14.relPrev[2] * gF14.relPrev[2]);
            std::printf("F14: at (%.0f, %.0f, %.0f) %.0f m from the camera, pitch %.3f\n", gF14.nodePos[0], gF14.nodePos[1], gF14.nodePos[2], d, pitch);
        }
    }
}
// ---- PK27 end

// PK07: the helicopter's height over the ground after 10 s, sampled by printAiCheck (the cruise is 28 m above the
// higher of the ground here and 2 v0 ahead, wobble +-2 m, and it climbs at most 10 m/s).
float gHeliUpMin = 1e9f, gHeliUpMax = -1e9f;
int gHeliSamples = 0;

// --ai-check: each racer's checkpoints, speed, position and resets.
void printAiCheck(const RaceSession& race, const DriveSession& drive) {
    const std::vector<int> pos = race.positions(drive);
    std::printf("ai check t=%5.1f s\n", race.clock);
    for (size_t i = 0; i < race.racers.size(); ++i) {
        const RaceSession::Racer& r = race.racers[i];
        const fuel::RaceRules::Racer& rr = race.rules.racers[i];
        const fuel::VehicleState st = drive.state(int(i));
        std::printf("  P%d %-16s %s class %d/%.2f %s  checkpoints %2zu/%zu  %5.1f km/h  resets %d (vehicle %d)%s\n", pos[i], r.name.c_str(),
                    i == 0 ? "(auto)" : "      ", r.ai.mode(), r.ai.frac(), r.ai.physical() ? "sim" : "kin", rr.next, race.rules.lineFor(i).size(),
                    st.speedKmh, r.ai.resets(), drive.physics().respawns(int(i)),
                    rr.finished >= 0 ? (" finished " + std::to_string(int(rr.finished)) + " s").c_str() : rr.out ? " out" : "");
        if (std::getenv("FUEL_AI_PATHDUMP")) {
            const simd_float3 p = drive.position(int(i));
            std::printf("     at %.1f %.1f %.1f\n", p.x, p.y, p.z);
        }
    }
    // PK07: the race helicopter (0x6202c0): display position, speed, height over the ground, gates passed, time left.
    if (race.rules.helicopter) {
        const fuel::RaceHelicopter& h = race.rules.hel;
        if (race.rules.ground && race.clock > 10.0) {  // --ai-check verdict: it must keep to the cruise height above the ground
            const float up = h.disp[1] - race.rules.ground(h.disp[0], h.disp[2]);
            gHeliUpMin = std::min(gHeliUpMin, up), gHeliUpMax = std::max(gHeliUpMax, up), ++gHeliSamples;
        }
        const simd_float3 me = drive.position(0);
        std::printf("  helicopter at %.1f %.1f %.1f  %5.1f m/s (required %.1f)  %.1f m over the ground  gates %zu/%zu  time left %.1f s  %.0f m from the player%s\n",
                    h.disp[0], h.disp[1], h.disp[2], h.speed, h.requiredSpeed, race.rules.ground ? h.disp[1] - race.rules.ground(h.disp[0], h.disp[2]) : 0.0f,
                    race.rules.heliNext, race.rules.gates.size(), h.timeLeft, std::hypot(me.x - h.disp[0], me.z - h.disp[2]),
                    race.rules.heliLanded ? "  LANDED" : "");
    }
}

// --ai-check gap report (P82): at 5, 10, 20 and 30 s of race time, each opponent's speed and its gap to the
// player along the AI path (> 0: ahead), with the ghost's speed model (cap S+0x1c, accel S+4, target S+0x18).
// Returns the largest lead an opponent has over the player at that time.
float aiGapLead5 = -1e9f;  // the largest lead over the player at 5 s (the gap report)
float aiGapLead10 = -1e9f;  // ... and at 10 s
// P97: the ground under (x, z) as the ghost sees it (0x61b810: terrain, or a static floor from 2 m above y).
float aiGroundAt(const RaceSession& race, const DriveSession& drive, double x, double y, double z) {
    float h = race.terrain ? race.terrain->smoothHeightAt(float(x), float(z)) : float(y);
    const float hs = drive.physics().staticFloorBelow(x, y + 2.0, z, 100.0f);
    if (hs > h) h = hs;
    return h;
}
float printAiGaps(const RaceSession& race, const DriveSession& drive) {
    const float playerLeft = race.line.pathLength() - race.playerS;
    std::printf("ai gaps t=%4.1f s: player %5.1f km/h, %.0f m along of %.0f m\n", race.clock, drive.state(0).speedKmh, race.playerS, race.line.pathLength());
    float lead = -1e9f;
    for (size_t i = 1; i < race.racers.size(); ++i) {
        const RaceSession::Racer& r = race.racers[i];
        const fuel::RaceLine& ln = race.lineFor(i);
        const float gap = playerLeft - (ln.pathLength() - r.ai.progress());
        const simd_float3 p = drive.position(int(i)), p0 = drive.position(0);
        lead = std::max(lead, gap);
        // P97: height above the ground (terrain or a static floor) of the vehicle and of its ghost, vertical speed.
        const fuel::VehicleState st = drive.state(int(i));
        const float hv = p.y - aiGroundAt(race, drive, p.x, p.y, p.z), hg = float(r.ai.ghost()[1]) - aiGroundAt(race, drive, r.ai.ghost()[0], r.ai.ghost()[1], r.ai.ghost()[2]);
        std::printf("  #%zu %-16s class %d %s %5.1f km/h (ghost %5.1f, cap %5.1f, accel %5.2f, target %5.0f s)  gap %+7.1f m (%.0f m away)  "
                    "air %5.2f m (ghost %5.2f) vy %+5.1f m/s\n", i,
                    r.name.c_str(), r.ai.mode(), r.ai.physical() ? "sim" : "kin", st.speedKmh, r.ai.ghostSpeed() * 3.6f,
                    r.ai.speedCap() * 3.6f, r.ai.accel(), r.ai.targetTime(), gap, simd_distance(p, p0), hv, hg, st.velocity[1]);
    }
    return lead;
}

// P97: the opponents' height above the ground in the first 10 s (--ai-check): the ghost (what a kinematic racer is
// placed on; a simulated racer's ghost rides at the vehicle's height) against the ground 0x61b810 sees. In the
// launch window (the first 5 s: 2 s of S+0xc push, 3 s of blend, 0x617000) a flight (ghost more than 1 m up) may
// not last over 1 s, nor stay over 0.25 s while more than 2.5 m up: P97's bug floated racers switched to kinematic
// (0x5cb6f0) 2.6-6.4 m over the downhill start of CR2 for 1.25-2 s. The crest hops of the launch speeds (simulated
// racers too, and racers leaving a bump as they switch: CR3-Yell-CT, CR2 on quads: 0.3-0.8 s, 1.1-1.9 m) pass.
struct AiAirCheck {
    std::vector<float> maxAir5, maxAir10, airTime5, run, high;
    int worst = -1;
    float worstAir = 0, worstRun = 0, worstAt = 0;
    bool bad = false;
    void step(const RaceSession& race, const DriveSession& drive, float dt) {
        const size_t n = race.racers.size();
        if (maxAir5.size() != n)
            maxAir5.assign(n, 0), maxAir10.assign(n, 0), airTime5.assign(n, 0), run.assign(n, 0), high.assign(n, 0);
        for (size_t i = 1; i < n; ++i) {
            const auto& g = race.racers[i].ai.ghost();
            const float air = float(g[1]) - aiGroundAt(race, drive, g[0], g[1], g[2]);
            maxAir10[i] = std::max(maxAir10[i], air);
            if (race.clock >= 5) continue;
            maxAir5[i] = std::max(maxAir5[i], air);
            run[i] = air > 1.0f ? run[i] + dt : 0.0f;
            high[i] = air > 2.5f ? high[i] + dt : 0.0f;
            if (air > 1.0f) airTime5[i] += dt;
            const bool flagged = run[i] > 1.0f || high[i] > 0.25f;
            if ((flagged && !bad) || (flagged == bad && run[i] > worstRun)) worstRun = run[i], worst = int(i), worstAir = maxAir5[i], worstAt = float(race.clock);
            bad = bad || flagged;
        }
    }
    bool report() const {
        std::printf("ai air check: opponents' ghost height above the ground, max over 0-5 s / 0-10 s (time > 1 m in 0-5 s):\n ");
        for (size_t i = 1; i < maxAir5.size(); ++i) std::printf(" #%zu %.2f/%.2f m (%.2f s)", i, maxAir5[i], maxAir10[i], airTime5[i]);
        std::printf("\nai air check: longest flight in the launch window %.2f s (racer %d, %.1f m up, to %.1f s): %s\n", worstRun, worst, worstAir,
                    worstAt, bad ? "FLYING" : "ok");
        return !bad;
    }
};
AiAirCheck aiAir;

// P97: the largest lead FUEL's speed rules allow an opponent over a player who stays on the grid, at time T: the
// ghost's speed model in one dimension (0x61de60: v += (S+4 - 0.1 v) dt up to the target, above it x 0.9985 / ms):
// S+4 = the launch S+0xc = 20 m/s^2 (row 0) for 2 s, blended over 3 s into 27.78 / the player's 0-100 time
// (0x617000 near the player) or its own; the target = the player's top speed (P619 / P620, k = 1 at the start)
// while within 100 m ahead (0x617000), else its own pace, the distance left over its time budget (0x616ed0, the
// fastest class's S+0x18; x 1.1 when late, 0x617610). Slopes, corners and the others are left out (a bound).
float aiStoppedLeadBound(const RaceSession& race, const DriveSession& drive, float T) {
    float playerTop = 0, player0to100 = 1e9f, own0to100 = 1e9f, base = 1e9f;
    {
        const fuel::AiVehicle pv = RaceSession::aiVehicle(drive.setup(0));
        playerTop = std::max(pv.roadKmh, pv.offKmh) / 3.6f, player0to100 = std::min(pv.road0to100, pv.off0to100);
    }
    for (size_t i = 1; i < race.racers.size() && int(i) < drive.vehicleCount(); ++i) {
        const fuel::AiVehicle ov = race.ownAiVehicle(drive, int(i));  // [upgrades U4] with its stages
        own0to100 = std::min({own0to100, ov.road0to100, ov.off0to100});
        base = std::min(base, race.racers[i].ai.targetTime());
    }
    const float L = race.line.pathLength();
    float v = 0, x = 0;
    const float dt = 1.0f / 60;
    for (float t = 0; t < T; t += dt) {
        const bool near = x < 100.0f;
        const float pace = (L - x) / std::max(base - t, 1.0f) * 1.1f;
        const float target = near ? std::max(playerTop, pace) : pace;
        float a = 27.777779f / (near ? player0to100 : own0to100);
        if (t < 5.0f) a = t < 2.0f ? 20.0f : (5.0f - t) / 3.0f * (20.0f - a) + a;
        v = v < target ? std::min(v + (a - 0.1f * v) * dt, target) : std::max(v * std::pow(0.9985f, dt * 1000.0f), target);
        x += v * dt;
    }
    return x;
}

// P87: the crash debris of 0x51e9e0 -> 0x52ee60 during a headless check (--ai-check, --prop-check): what was
// thrown in the first 10 s of the race (the helicopter drop, the countdown, the launch: FUEL throws nothing there
// without a real crash), the most thrown in any second (a scrape: one request per second, |rel| / 2 pieces) and
// any piece that rides along with a vehicle (within 4 m of it, its velocity within 5 m/s of the vehicle's at
// more than 5 m/s: trapped in the body, which FUEL's +0xeb2 rules out).
struct DebrisCheck {
    size_t requestsEarly = 0, piecesEarly = 0, requests = 0, pieces = 0, lastRequests = 0, lastPieces = 0;
    std::deque<std::pair<double, size_t>> window;  // (time, pieces) of the last second
    size_t perSecondMax = 0;
    std::unordered_map<uint32_t, float> riding;  // piece -> seconds alongside a vehicle
    double firstAt = -1;
    void step(double t, bool early, const DriveSession& drive, WorldStreamer* streamer, float dt) {
        const size_t r = gDamageView.debrisRequests(), p = gDamageView.debrisSpawned();
        if (r > lastRequests) {
            if (firstAt < 0) firstAt = t;
            if (early) requestsEarly += r - lastRequests, piecesEarly += p - lastPieces;
            if (std::getenv("FUEL_P87_TRACE"))
                std::printf("debris t=%.2f: %zu request(s), %zu piece(s)%s\n", t, r - lastRequests, p - lastPieces, early ? " (first 10 s)" : "");
            window.push_back({t, p - lastPieces});
        }
        requests = r, pieces = p, lastRequests = r, lastPieces = p;
        while (!window.empty() && window.front().first <= t - 1.0) window.pop_front();
        size_t inWindow = 0;
        for (const auto& w : window) inWindow += w.second;
        perSecondMax = std::max(perSecondMax, inWindow);
        if (streamer == nullptr) return;
        for (uint32_t id : gDamageView.pieceIds()) {
            std::array<double, 3> pos{};
            std::array<float, 3> v{};
            if (!streamer->thrownPosition(id, pos) || !drive.physics().propVelocity(id, v)) continue;
            for (int i = 0; i < drive.vehicleCount(); ++i) {
                const fuel::VehicleState st = drive.state(i);
                const auto& c = st.chassis.position;
                const double d = std::hypot(std::hypot(pos[0] - c[0], pos[1] - c[1]), pos[2] - c[2]);
                const float dv = std::hypot(std::hypot(v[0] - st.velocity[0], v[1] - st.velocity[1]), v[2] - st.velocity[2]);
                const float sv = std::hypot(std::hypot(st.velocity[0], st.velocity[1]), st.velocity[2]);
                if (d < 4.0 && dv < 5.0f && sv > 5.0f) {
                    riding[id] += dt;
                    break;
                }
            }
        }
    }
    bool report(const DriveSession& drive) const {
        size_t ridingPieces = 0;
        float ridingMax = 0;
        for (const auto& [id, s] : riding) ridingMax = std::max(ridingMax, s), ridingPieces += s > 0.5f;
        std::printf("debris check: %zu requests / %zu pieces in all (first at %.1f s), %zu / %zu in the first 10 s (expected 0 without a crash), "
                    "at most %zu pieces in one second\n", requests, pieces, firstAt, requestsEarly, piecesEarly, perSecondMax);
        if (const fuel::fv::FuelVehicle* m = drive.physics().model(0)) {
            const auto& b = m->body();
            std::printf("debris check: player contacts over 5 m/s refused by 0x51e9e0: driver %d, race state %d, wheel %d, class %d, timer %d\n",
                        b.fxRefused[0], b.fxRefused[1], b.fxRefused[2], b.fxRefused[3], b.fxRefused[4]);
        }
        int aiRefused = 0;
        for (int i = 1; i < drive.vehicleCount(); ++i)
            if (const fuel::fv::FuelVehicle* m = drive.physics().model(i)) aiRefused += m->body().fxRefused[0];
        std::printf("debris check: opponents' contacts over 5 m/s (no driver +0x142b: no debris) %d\n", aiRefused);
        const bool ok = ridingPieces == 0;
        std::printf("debris check: %zu pieces rode with a vehicle for more than 0.5 s (longest %.2f s): %s\n", ridingPieces, ridingMax, ok ? "ok" : "FAILED");
        return ok;
    }
};
DebrisCheck gDebrisCheck;

// Passes when every opponent got through checkpoints and none needed more than a few resets (the AI's own
// resets and the vehicles' wreck/overturn respawns together).
bool aiCheckVerdict(const RaceSession& race, const DriveSession& drive, bool stoppedPlayer) {
    int moving = 0, resets = 0, respawns = 0;
    for (size_t i = 1; i < race.racers.size(); ++i) {
        moving += race.rules.racers[i].next >= std::min<size_t>(3, race.rules.lineFor(i).size());
        resets += race.racers[i].ai.resets();
        respawns += drive.physics().respawns(int(i));
    }
    const int n = int(race.racers.size()) - 1;
    // P82: no ghost may run far past the fastest vehicle's top speed (FUEL's caps: the time target, the player's
    // top speed near the player; 0x61e080's slope push is bounded by 2 S+4 sin / sin(P626)). The raw-vy slope
    // push sent kinematic ghosts to 700 km/h; the 5 s lead (launch S+0xc 20 m/s^2 at most 2 s) was 250 m.
    const bool fast = race.maxGhostKmh > race.topKmh * 1.25f;
    // The lead at 5 s: 120 m with the autopilot (P82); P97: with a player on the grid (--ai-player stop) FUEL's own
    // launch and near-player cap take the leaders well past that: the bound of FUEL's rules (aiStoppedLeadBound,
    // + 15 %) at 5 and 10 s. A launching player (--ai-player launch) keeps the 120 m.
    float limit5 = 120.0f, limit10 = 1e9f;
    if (stoppedPlayer) limit5 = aiStoppedLeadBound(race, drive, 5.0f) * 1.15f, limit10 = aiStoppedLeadBound(race, drive, 10.0f) * 1.15f;
    const bool far = aiGapLead5 > limit5 || aiGapLead10 > limit10;
    std::printf("ai check: fastest ghost %.0f km/h (racer %d at %.1f s), top speed %.0f km/h: %s; lead at 5 s %.0f m (limit %.0f), at 10 s %.0f m",
                race.maxGhostKmh, race.maxGhostRacer, race.maxGhostClock, race.topKmh, fast ? "TOO FAST" : "ok", aiGapLead5, limit5, aiGapLead10);
    if (limit10 < 1e8f) std::printf(" (limit %.0f)", limit10);
    std::printf(": %s\n", far ? "TOO FAR" : "ok");
    const bool airOk = aiAir.report();
    bool heliOk = true;  // PK07: a helicopter race has no opponent vehicle: the helicopter must fly at its cruise height
    if (race.rules.helicopter) {
        heliOk = gHeliSamples > 0 && gHeliUpMin > 20.0f && gHeliUpMax < 200.0f;
        std::printf("ai check: helicopter %d samples, %.1f..%.1f m over the ground (cruise 28 m + wobble): %s\n", gHeliSamples, gHeliUpMin, gHeliUpMax,
                    heliOk ? "ok" : "FAILED");
    }
    // PK16: finished AI racers park in the staggered row (0x624310): state 5 within a few metres of A + (6 (N - rank + 1) + 5) dir + 6 m aside.
    int finishers = 0, parkedOk = 0;
    for (size_t i = 1; i < race.racers.size() && int(i) < drive.vehicleCount(); ++i) {
        if (race.rules.racers[i].finished < 0 || !race.rules.usesRoute(i)) continue;
        ++finishers;
        const int rank = race.rules.finishRank(i);
        const fuel::RaceLine park = race.lineFor(i).parkingLine(rank, float(race.race().aiCount - rank + 1) * 6.0f + 5.0f);
        const auto& T = park.path.back();
        const simd_float3 p = drive.position(int(i));
        const float d = std::hypot(p.x - T[0], p.z - T[1]), g = std::hypot(float(race.racers[i].ai.ghost()[0]) - T[0], float(race.racers[i].ai.ghost()[2]) - T[1]);
        // A racer that finished less than 30 s before the run ended is still rolling out (state 4): not judged.
        const bool late = race.clock - race.rules.racers[i].finished < 30.0;
        const bool ok1 = late || (race.racers[i].ai.finishState() == 5 && d < 10.0f);
        parkedOk += ok1;
        std::printf("ai check: #%zu finished %d. (rank %d) state %d: vehicle %.1f m / ghost %.1f m from its parking spot, %.1f km/h: %s\n", i, rank + 1, rank,
                    race.racers[i].ai.finishState(), d, g, drive.state(int(i)).speedKmh, ok1 ? (late && race.racers[i].ai.finishState() != 5 ? "rolling out" : "parked") : "NOT PARKED");
    }
    const bool parkOk = parkedOk == finishers;
    // [upgrades U4] the opponents' stages are a function of the race, the grid slot, the hub tier and the seed only: drawn
    // again (twice) they are the same, and each vehicle carries what U1's transform made of them (torque scale, lift, widening,
    // nitro). A retry (RaceSession::begin) keeps the vehicles; a new session draws the same field (fuelphysvec AID3 holds
    // CR2-Yell-CP's field as a known answer).
    bool upgOk = true;
    {
        uint64_t field = 0xcbf29ce484222325ull;
        int differ = 0, mismatch = 0;
        for (size_t i = 1; i < race.racers.size() && int(i) < drive.vehicleCount(); ++i) {
            const RaceSession::Racer& r = race.racers[i];
            const fuel::upg::AiDraw a = fuel::upg::aiDraw(race.event().id, int(i) - 1, race.upgTier, fuel::upg::aiSeedOverride());
            const fuel::upg::AiDraw b = fuel::upg::aiDraw(race.event().id, int(i) - 1, race.upgTier, fuel::upg::aiSeedOverride());
            differ += !(a.stages == r.upgDraw.stages && b.stages == a.stages && a.seed == r.upgDraw.seed && a.rival == r.upgDraw.rival);
            const fuel::VehicleSetup& su = drive.setup(int(i));
            const fuel::upg::Result& u = r.upgResult;
            mismatch += su.torqueScale != u.torqueScale || su.lift != u.lift || su.widenFront != u.widenFront || su.nitroPower != u.nitroPower ||
                        su.nitroTank != u.nitroTank || su.upgraded != u.applied;
            for (int c = 0; c < fuel::upg::kCategories; ++c) field = (field ^ uint64_t(r.upgDraw.stages[c] + 1)) * 0x100000001b3ull;
        }
        upgOk = differ == 0 && mismatch == 0;
        std::printf("ai check: opponents' stages for %s, hub tier %d, seed %llu: field %016llx, drawn again %d differ, %d vehicles not as drawn: %s\n",
                    race.event().id.c_str(), race.upgTier, (unsigned long long)fuel::upg::aiSeedOverride(), (unsigned long long)field, differ, mismatch,
                    upgOk ? "deterministic" : "FAILED");
    }
    if (fuel::nitro::enabled()) {  // [nitro] each racer's use of its tank and what filled it
        // Bonus fills in seconds of boost, and their rate against the idle recharge (rechargeRate x 60 s of boost a minute).
        const fuel::nitro::Params& np = fuel::nitro::params();
        const double minutes = std::max(race.clock, 1.0) / 60.0;
        std::printf("ai check: nitro over %.1f s (boosts, seconds held; bonus seconds air / slide / overtake / near miss; bonus per minute "
                    "against the idle recharge's %.1f):\n", race.clock, np.rechargeRate * 60.0f);
        for (size_t i = 0; i < race.racers.size() && int(i) < drive.vehicleCount(); ++i)
            if (const fuel::nitro::Tank* t = drive.physics().nitroTank(int(i))) {
                const float sum = t->earned[fuel::nitro::KindAir] + t->earned[fuel::nitro::KindSlide] + t->earned[fuel::nitro::KindOvertake] +
                                  t->earned[fuel::nitro::KindNearMiss];
                std::printf("  #%zu %-16s %2d boosts %5.1f s  tank %.2f  bonus %5.2f / %5.2f / %5.2f / %5.2f s = %5.1f s/min (x%.2f idle)"
                            "  (%d flights, %.1f s sliding, %d places, %d near misses)\n",
                            i, race.racers[i].name.c_str(), t->starts, t->boosted, t->level, t->earned[fuel::nitro::KindAir], t->earned[fuel::nitro::KindSlide],
                            t->earned[fuel::nitro::KindOvertake], t->earned[fuel::nitro::KindNearMiss], sum / minutes,
                            sum / minutes / std::max(1e-3, double(np.rechargeRate) * 60.0), t->events[fuel::nitro::KindAir],
                            t->events[fuel::nitro::KindSlide] / 60.0, t->events[fuel::nitro::KindOvertake], t->events[fuel::nitro::KindNearMiss]);
            }
    }
    const bool ok = (n > 0 || race.rules.helicopter) && moving == n && resets + respawns <= 2 * n && !fast && !far && airOk && heliOk && parkOk && upgOk;
    std::printf("ai check: %d of %d opponents past 3 checkpoints, %d resets in all (%d AI, %d vehicle respawns): %s\n", moving, n, resets + respawns,
                resets, respawns, ok ? "ok" : "FAILED");
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
    // The emulated tables are compared by an FNV-1a hash of their values (as little-endian int32)
    // so the engine tables themselves stay out of the repo (hard rule).
    auto fnv = [](const std::vector<int32_t>& v) {
        uint64_t h = 0xcbf29ce484222325ull;
        for (int32_t x : v) {
            unsigned char b[4];
            std::memcpy(b, &x, 4);
            for (unsigned char c : b) h = (h ^ c) * 0x100000001b3ull;
        }
        return h;
    };
    auto hashItem = [&](const char* name, const std::vector<int32_t>& v, uint64_t want) {
        char b[64];
        std::snprintf(b, sizeof b, "hash %016llx", (unsigned long long)fnv(v));
        out.push_back({name, fnv(v) == want, b});
    };
    std::vector<int32_t> v;
    for (int k = 0; k < 47; ++k) v.push_back(t->slots[2][size_t(k)]);
    hashItem("Slot table 0x5d6950 MBIKE actions 0..46", v, 0x88f062ef35186e64ull);
    v.clear();
    for (int a = 0; a < 11; ++a) v.push_back(t->slots[size_t(a)][0]);
    hashItem("Slot table 0x5d6950 base slots (11 archetypes)", v, 0x577c694593b576f7ull);
    // R3 action properties {group, priority, mode}
    v.clear();
    for (int k = 0; k < 47; ++k) {
        const auto& p = t->props[size_t(k)];
        v.insert(v.end(), {int32_t(p.group), int32_t(p.priority), int32_t(p.mode)});
    }
    hashItem("Action props 0x5d3660 (47 actions)", v, 0x59ec26e8b8fd045eull);
    const auto s1 = fuel::RiderAnimator::swayTarget({0, 0, 0, 1}, {0, 0, 5}), s2 = fuel::RiderAnimator::swayTarget({0, 0, 0, 1}, {30, 0, 0});
    const bool sOk = std::fabs(s1[1] - 0.025f) < 1e-7f && std::fabs(s2[1] - 0.05f) < 1e-7f && s1[0] == 0 && s1[2] == 0 && s2[0] == 0 && s2[2] == 0;
    char b[96];
    std::snprintf(b, sizeof b, "got (0,%.7g,0) (0,%.7g,0)", s1[1], s2[1]);
    out.push_back({"Rider sway target 0x5d7d40", sOk, b});
    return out;
}

// --aicheck-vectors: the AI's line, aim, corners and corner speed (race_ai, P29) against FUEL.exe run in
// Ghidra's emulator on synthetic racer records (notes/p29-work/ai_gen.py: 0x61acb0, 0x61e540 + 0x61dcf0).
// The guided target forces are in build/fuelphysvec (GF1-GF3).
int runAiVectors(const std::string& dataDir) {
    int ok = 0, bad = 0;
    auto check = [&](const char* id, const char* what, const std::vector<float>& got, const std::vector<float>& want, float tol) {
        bool m = got.size() == want.size();
        for (size_t k = 0; m && k < got.size(); ++k) m = std::fabs(got[k] - want[k]) <= tol * std::max(1.0f, std::fabs(want[k]));
        std::printf("%-4s %-40s %s  got", id, what, m ? "match   " : "MISMATCH");
        for (float g : got) std::printf(" %.7g", g);
        if (!m) {
            std::printf("  want");
            for (float w : want) std::printf(" %.7g", w);
        }
        std::printf("\n");
        (m ? ok : bad)++;
    };
    std::string err;
    const auto tables = fuel::loadAiTables((fs::path(dataDir) / "XFinalComponent/FUEL.exe").string(), &err);
    if (!tables) {
        std::printf("aicheck: %s\n", err.c_str());
        return 1;
    }
    // The table FUEL's 0x61e540 writes (emulated), against the port's reading of the same code. Compared by
    // an FNV-1a hash of its 20 floats so the engine table itself stays out of the repo (hard rule).
    {
        uint64_t h = 0xcbf29ce484222325ull;
        for (float f : tables->cornerSpeed) {
            unsigned char b[4];
            std::memcpy(b, &f, 4);
            for (unsigned char c : b) h = (h ^ c) * 0x100000001b3ull;
        }
        const bool m = tables->cornerSpeed.size() == 20 && h == 0x60446c3cd0a57b7eull;
        std::printf("%-4s %-40s %s  hash %016llx\n", "T1", "corner speed table 0x61e540", m ? "match   " : "MISMATCH", (unsigned long long)h);
        (m ? ok : bad)++;
    }
    // Samples every 10 m along a path turning by `turns` (degrees at sample k) as ai_gen.py builds them.
    auto arc = [](int n, double perSample, const std::map<int, double>& turns, const std::function<std::pair<float, float>(int)>& lat) {
        std::vector<std::array<double, 2>> pts;
        double x = 0, z = 0, h = 0;
        for (int k = 0; k <= n; ++k) {
            pts.push_back({x, z});
            x += std::sin(h) * 10.0, z += std::cos(h) * 10.0;
            h += turns.empty() ? perSample : (turns.count(k) ? turns.at(k) * M_PI / 180.0 : 0.0);
        }
        std::vector<fuel::AiSample> S;
        double cum = 0;
        for (size_t k = 0; k < pts.size(); ++k) {
            fuel::AiSample a;
            double dx, dz, l;
            if (k + 1 < pts.size()) {
                dx = pts[k + 1][0] - pts[k][0], dz = pts[k + 1][1] - pts[k][1], l = std::hypot(dx, dz), dx /= l, dz /= l;
            } else {
                dx = S.back().dx, dz = S.back().dz, l = double(S.back().seg) + 1.0;
            }
            const auto [lo, hi] = lat(int(k));
            a.x = float(pts[k][0]), a.z = float(pts[k][1]), a.latMin = lo, a.latMax = hi, a.dx = float(dx), a.dz = float(dz), a.seg = float(l), a.cum = float(cum);
            if (k + 1 < pts.size()) cum += l;
            S.push_back(a);
        }
        return S;
    };
    struct LCase {
        const char* id;
        std::vector<fuel::AiSample> S;
        int idx;
        float p0[2], s0, hist;
        std::vector<float> aim, corners;
    };
    const LCase lc[] = {
        {"L1", arc(20, 60.0 * M_PI / 180.0 / 20.0, {}, [](int) { return std::make_pair(-30.0f, 30.0f); }), 0, {0, 2}, 3, 25, {1.0689021f, 14.454214f}, {}},
        {"L3", arc(14, 0, {{4, 35.0}, {5, 35.0}, {6, 35.0}, {9, -25.0}}, [](int) { return std::make_pair(-12.0f, 12.0f); }), 0, {0.5f, 1}, 1, 18,
         {0.42544538f, 9.999691f},
         {9.302983f, 47.06678f, 17.499998f, 12.150337f, 12.081834f, 49.92116f, 35.0f, 4.098469f, 15.547596f, 52.10884f, 34.999992f, 8.629087f, 23.698313f,
          54.94203f, 17.5f, 10.942888f}},
        {"L2", arc(10, -50.0 * M_PI / 180.0 / 10.0, {}, [](int k) { return k != 6 ? std::make_pair(-8.0f, 8.0f) : std::make_pair(-2.0f, 8.0f); }), 1,
         {1, 12}, 12, std::hypot(3.0f, 14.0f), {-0.9217987f, 18.896135f}, {}},
    };
    for (const LCase& c : lc) {
        float aim[2] = {0, 0};
        std::vector<fuel::AiCorner> corners;
        fuel::aiSmoothLineAndAim(c.S, c.idx, c.s0, c.p0, c.hist, aim, corners);
        check(c.id, "0x61acb0 aim point (x, z)", {aim[0], aim[1]}, c.aim, 2e-4f);
        std::vector<float> got;
        for (const auto& k : corners) got.insert(got.end(), {k.x, k.z, k.angle, k.len});
        check(c.id, "0x61acb0 corners (x z angle len)", got, c.corners, 5e-4f);
    }
    struct CCase {
        const char* id;
        std::vector<fuel::AiCorner> recs;
        double gx, gz;
        float want;
    };
    const CCase cc[] = {
        {"C1", {{10, 50, 30, 22}}, 0, 32, 38.8888893f},
        {"C2", {{0, 60, 25, 18}, {5, 75, 20, 16}}, 0, 36, 38.8888893f},
        {"C3", {{0, 30, 11, 40}}, 0, 0, 61.1111145f},
    };
    for (const CCase& c : cc) check(c.id, "0x61dcf0 corner speed", {fuel::aiCornerSpeed(c.recs, *tables, c.gx, c.gz)}, {c.want}, 1e-6f);
    fuel::atvCheckVectors(ok, bad);  // off-road path finder (P58)
    std::printf("aicheck: %d of %d vectors match\n", ok, ok + bad);
    return bad == 0 ? 0 : 1;
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
    const int bench = std::getenv("FUEL_GPU_BENCH") && !o.shot.empty() ? std::max(0, std::atoi(std::getenv("FUEL_GPU_BENCH"))) : 0;
    double total = 0, best = 1e9;
    for (int k = 0; k <= bench; ++k) {
        id<MTLCommandBuffer> cb = [queue commandBuffer];
        renderer.gpsRibbons = gGpsDraw;  // P86
        renderer.navBeams = gNavBeams;   // PK19
        renderer.render(cb, image, cam);
        if (gOverlay) gOverlay->draw(cb, image, gShotHud);
        [cb commit];
        [cb waitUntilCompleted];
        const double ms = (cb.GPUEndTime - cb.GPUStartTime) * 1000.0;
        if (k == bench) {
            if (o.shot.empty()) return ms;  // TAA: a film's history frame (not written)
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
    std::unique_ptr<fuel::PadHaptics> _haptics;  // P89: FUEL's pad vibration (asked by the player's sounds)
    std::unique_ptr<fuel::PadTriggers> _triggers;  // P91: DualSense adaptive triggers (reads the game only)
    int _optSerial, _vibTests;  // P101: the in-game options pages' change counters already applied
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
            DustSystem::playerFlash = [p = &_audio] { return *p ? (*p)->playerFlash() : -1.0f; };  // PK23: +0x2ebc for the exhaust flames
            _audio->setMusicVolume(fuel::ui::loadMenuOptions().musicVolume);  // P101: Music Volume (0x554040), its own bus
            if (gOverlay && gOverlay->menu().career())  // PK27: the saved voice level (+0x25c) -> the voice bus (0x554130)
                _audio->setVoiceVolume(float(gOverlay->menu().career()->voiceLevel()) / 16.0f);
            {  // P89: the controller rumbles as FUEL's pad (settings.txt vibration = 0 turns it off)
                const fuel::Settings ps = fuel::loadSettings();
                _haptics = std::make_unique<fuel::PadHaptics>(gDataDir.empty() ? ps.dataDir : gDataDir, ps);
                _audio->setVibration(_haptics->manager());
                _triggers = std::make_unique<fuel::PadTriggers>(ps);  // P91: settings.txt adaptive_triggers
            }
            [self loadAudio];
            if (gWorldDpc) _audio->setWorld(*gWorldDpc);  // ambience and music (p45)
            {  // p48: P49's thunder timer drives the lightning visuals (one bolt per sound strike)
                fuel::MeshRenderer* r = _renderer;
                _audio->setLightningHook([r](const std::array<float, 3>& p) { r->lightningStrike(p); });
            }
            for (size_t i = 1; race && i < race->racers.size(); ++i) {  // opponents: engine only, FUEL's distance attenuation
                const size_t entry = race->racers[i].entry;
                const AssemblyMode& am = *garage->archives[garage->entries[entry].first];
                const auto it = garage->params.find(race->racers[i].name);
                if (it != garage->params.end()) _audio->setRival(i - 1, am.assembler->dpc(), race->racers[i].name, it->second);
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
    if (gInputLoaded) return gInput.drivesAction(code);  // P47: the keys bound to FUEL's in-game actions
    switch (code) {
        case 13: case 1: case 0: case 2: case 49: case 123: case 124: case 125: case 126: return true;
        default: return false;
    }
}

- (void)keyUp:(NSEvent*)e {
    _keys.erase(e.keyCode);
    if (gOverlay) gOverlay->keyUp(e.keyCode);  // P75: held map keys
}

// Modifier keys (FUEL binds BRAKE to the left Control key) arrive as flag changes.
- (void)flagsChanged:(NSEvent*)e {
    NSEventModifierFlags f = 0;
    switch (e.keyCode) {
        case 56: case 60: f = NSEventModifierFlagShift; break;
        case 58: case 61: f = NSEventModifierFlagOption; break;
        case 59: case 62: f = NSEventModifierFlagControl; break;
        case 54: case 55: f = NSEventModifierFlagCommand; break;
        default: return;
    }
    if ((e.modifierFlags & f) && !_keys.count(e.keyCode)) {
        if (_drive && [self driveKey:e.keyCode]) _keys.insert(e.keyCode);
    } else {
        _keys.erase(e.keyCode);
    }
}

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
        fuel::upg::applyPlayer(setup, fuel::fv::FuelVehicle::powerScale);  // [upgrades U1] the session's stages on the new vehicle
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
    if (_audio) _audio->teleportQueued();  // Teleport_Queue 0x57e650 plays event symbol 0xfa
    std::printf("%s: %s at (%.0f, %.0f)%s\n", vista ? "vista" : "teleport", p.name.c_str(), s.x, s.z, s.nudged ? " (nudged)" : "");
}

- (void)keyDown:(NSEvent*)e {
    if (gOverlay && gOverlay->keyDown(e.keyCode, e.isARepeat)) return;  // [port] remap: repeats
    if (gOverlay && e.keyCode == 53) {  // PK17: Escape refused by the pause gate (an RTC runs): FUEL's PAUSE action (its skip), never quit
        if (_drive) _keys.insert(e.keyCode);
        return;
    }
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
            _renderer->resetTemporalHistory();  // TAA: a cut
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
            killMission(_race, *_drive, _streamer, *_renderer);  // PK14: TTMN 0x54a910 runs KillMission first
            _race->begin(*_drive, _streamer, *_renderer);
            gRtcStartPending = true;
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
    if (gOverlay && gOverlay->open()) {  // pad-ux [port]: the wheel / trackpad scrolls the pause menu (whole rows)
        static float acc = 0;
        // a menu row (49 canvas units) in view points: the canvas is the view's width / 1280 (16:9 letterbox, 0x49bec0)
        const float canvasPerPoint = 1280.0f / float(std::max(1.0, std::min(self.bounds.size.width, self.bounds.size.height / 0.5625)));
        acc += e.hasPreciseScrollingDeltas ? -float(e.scrollingDeltaY) * canvasPerPoint / 49.0f : -float(e.scrollingDeltaY);
        const int rows = int(acc);
        if (rows) acc -= float(rows), gOverlay->wheel(float(rows));
        return;
    }
    _cam.distance = std::clamp(_cam.distance * std::exp(-float(e.scrollingDeltaY) * (e.hasPreciseScrollingDeltas ? 0.01f : 0.1f)), 0.4f, 3000.0f);
}

// pad-ux [port]: the pointer hovers the pause menu's items as in the menu window (FuelMenuView mouseMoved).
- (void)mouseMoved:(NSEvent*)e {
    if (!gOverlay || !gOverlay->open()) return;
    const NSPoint p = [self convertPoint:e.locationInWindow fromView:nil];
    const CGSize ds = self.drawableSize;
    gOverlay->pointer(float(p.x * ds.width / self.bounds.size.width), float((self.bounds.size.height - p.y) * ds.height / self.bounds.size.height), false,
                      float(ds.width), float(ds.height));
}

- (void)magnifyWithEvent:(NSEvent*)e {
    _cam.distance = std::clamp(_cam.distance / (1.0f + float(e.magnification)), 0.4f, 3000.0f);
}

- (void)mtkView:(MTKView*)view drawableSizeWillChange:(CGSize)size {}

// --in-game: the race order for the results page and the HUD.
- (std::vector<fuel::ui::ResultRow>)resultRows {
    if (!_race) return {};
    std::vector<std::string> names;
    std::vector<double> finished;
    std::vector<int> nameTT;  // PK16: the opponents' names (0x620c20)
    for (size_t i = 0; i < _race->racers.size(); ++i)
        names.push_back(_race->racers[i].name), finished.push_back(_race->rules.racers[i].finished), nameTT.push_back(_race->racers[i].nameTT);
    return fuel::ui::raceResultRows(_race->positions(*_drive), names, finished, &nameTT);
}

- (fuel::ui::HudState)hudState {
    static float odometer = 0;
    static double last = 0;
    const double now = CACurrentMediaTime();
    fuel::ui::HudState h;
    const fuel::VehicleState st = _drive->state();
    h.speedKmh = std::fabs(st.speedKmh);
    const float dt = last > 0 && (!gOverlay->open() || gOverlay->worldRuns()) ? float(std::min(0.1, now - last)) : 0.0f;
    fillHud(h, *_drive, _race, dt, _chaseView);
    odometer += h.speedMs * dt * 0.001f;  // port: the session's distance (FUEL: the profile total, game+0x3408)
    last = now;
    h.odometerKm = odometer;
    // PK03 0x4a54b0's hub name: fillHudState (minimap-fix: in no zone TT 998 "OUT OF AREA").
    h.race = _race != nullptr;
    return h;
}

- (void)mouseDown:(NSEvent*)e {
    if (!gOverlay || !gOverlay->open()) return;
    const NSPoint p = [self convertPoint:e.locationInWindow fromView:nil];
    const CGSize ds = self.drawableSize;
    gOverlay->pointer(float(p.x * ds.width / self.bounds.size.width), float((self.bounds.size.height - p.y) * ds.height / self.bounds.size.height), true,
                      float(ds.width), float(ds.height));
}

// P101: the in-game options pages (ui/options_pages.mm) apply as they change, in a race or free roam too: the renderer
// settings, the volumes, the vibration option and its test pulse, the trigger level, the controls layout and camera
// axes, full screen. Only the resolution waits for the next start.
- (void)applyLiveOptions {
    fuel::ui::Menu& m = gOverlay->menu();
    if (m.optionsSerial() != _optSerial) {
        _optSerial = m.optionsSerial();
        const fuel::Settings s = m.liveSettings();
        applyLiveSettings(s, m.options().brightness, *_renderer);
        if (gRiderView) {  // PHYSIQUE: rider_jiggle applies at once
            const auto it = s.all.find("rider_jiggle");
            gRiderView->setJiggle(fuel::riderJiggleSetting(it == s.all.end() ? "subtle" : it->second));
        }
        if (_audio) _audio->setMasterVolume(gGameVolume), _audio->setMusicVolume(m.options().musicVolume);
        if (_audio && m.career()) _audio->setVoiceVolume(float(m.career()->voiceLevel()) / 16.0f);  // PK27: 0x553c80(0) -> 0x554130
        if (_haptics) _haptics->setOption(m.portOptions().vibration);
        if (_triggers) _triggers->reconfigure(s);
        if (gInputLoaded) {  // P47 map: controls = fuel | port, invert_camera_x / _y (0x495260 -> +0x271 / +0x2a0)
            std::string err;
            fuel::input::InputMap im;
            if (im.load(gDataDir, s.all, &err)) gInput = im;
        }
        applyFullscreenLive(s.fullscreen, self.window, self);
    }
    if (m.vibrationTests() != _vibTests) {
        _vibTests = m.vibrationTests();
        if (_haptics) _haptics->testPulse();  // 0x495670
    }
}

- (void)driveStep {
    const double now = CACurrentMediaTime();
    const float dt = _lastTime > 0 ? float(std::min(now - _lastTime, 0.1)) : 0.0f;
    _lastTime = now;
    if (gOverlay) {  // --in-game: the menu pauses the session
        // ---- PK25: Game_IsPlayerFree 0x57db50 for the pad-disconnect pause (0x57dd10): the race (game+0x334c) state +0x48
        // in {0, 5, 6, 7} (none / ended) and no freezing cut-scene.
        gOverlay->setPlayerFree(!(_race && _race->rules.outcome == fuel::RaceRules::Pending) && !gRtc.active());
        // ---- end PK25
        // ---- PK17: 0x539ac0 = an RTC runs (0x534b40 mask 0x40: every record) or its queue entry is issued (state 3; the
        // port: the race's TELEPORT_HELICO is due); 0x534b40(2) = a freeze-player RTC runs.
        gOverlay->setRtcGate(gRtc.active() || (gRtcEnabled && gLibrary && _race && gRtcStartPending), gRtc.active() && (gRtc.flags() & 2));
        // ---- end PK17
        gOverlay->pollGamepad();
        gOverlay->setLogoReady(tlpReady(_streamer, *_renderer));  // PK15: 0x57e0f0 state 2 waits for idle streaming
        const auto action = gOverlay->update(dt);
        pauseBackdropEvents(*_renderer, _audio.get(), _drive->position());  // PK17
        _renderer->brightness = 0.5f + 1.5f * gOverlay->menu().options().brightness;  // P62: Brightness option (0x495540 -> 0x554260)
        [self applyLiveOptions];  // P101: what the options pages changed, now
        if (action == fuel::ui::GameOverlay::Teleport) {  // PK15: through the teleport record (lands in state 3)
            tlpQueue(gTlp, *_drive, _streamer, *_renderer, gOverlay->teleportTarget()[0], gOverlay->teleportTarget()[1], _drive->heading());
            if (_audio) _audio->teleportQueued();  // 0x57e650
        }
        // ---- PK26: Dialog_Loading 0x58fc20 state 0xf after +0xe8 (0.5 s) -> 0x590300 [O]: `CheckUnLock` (the career re-ran
        // its unlock rules at the load), `TeleportToHub <game+0x33c0 or 0>` (0x5495a0: CAMERA_CAREER of the hub through the
        // teleport queue 0x57e650, StartHubHUD <uid> queued after it), `UnPause` (the session is not paused), and
        // `AddCommandAfterTeleport GAMEStarted` (GameManager +0xdc = 1 once the teleport landed).
        if (gOverlay->careerBootDue() && gBootHub >= 0 && size_t(gBootHub) < gOverlay->menu().hubs().size()) {
            const fuel::HubPoint& c = gOverlay->menu().hubs()[size_t(gBootHub)].camp;
            if (fuel::Career* car = gOverlay->menu().career()) car->checkUnlocks();
            gOverlay->startLogo();
            tlpQueue(gTlp, *_drive, _streamer, *_renderer, c.x, c.z, c.yaw, gBootHub);
            if (_audio) _audio->teleportQueued();
            gBootStartedPending = true;
            std::printf("boot: CheckUnLock, TeleportToHub %s, UnPause, AddCommandAfterTeleport GAMEStarted\n", gOverlay->menu().hubs()[size_t(gBootHub)].id.c_str());
        }
        // ---- end PK26
        // PK15: 0x57e0f0's regeneration (state 1 -> 2) and landing (-> 3, then the queued StartHubHUD 0x5492d0).
        tlpStep(gTlp, gOverlay->logoState(), *_drive, _streamer, *_renderer, [](int hub) {
            if (gFreeRoam) gFreeRoam->hubTeleported(hub);
            if (gBootStartedPending && gOverlay) {  // PK26: GAMEStarted 0x549510 -> 0x557e50 after the teleport
                gBootStartedPending = false;
                gOverlay->setGameStarted(true);
                std::printf("boot: GAMEStarted\n");
            }
        });
        if (_haptics) _haptics->manager()->gameStarted = gOverlay->gameStarted();  // PK26: 0x557b40 needs GameManager +0xdc
        if (gFreeRoam) {  // PK03: the pause menu opens (vf22: +0x94 = 0x46e650) / closes (0x47f060 or 0x47ee80)
            static bool wasOpen = false;
            const bool open = gOverlay->open();
            const simd_float3 at = _drive->position();
            if (open && !wasOpen) gFreeRoam->hubMenuOpened(at.x, at.z, _race != nullptr);
            float tx = 0, tz = 0;
            if (!open && wasOpen && gFreeRoam->hubMenuClosed(action == fuel::ui::GameOverlay::Resume, tx, tz)) {
                gOverlay->startLogo();  // TLP 0x4caa90 to the hub's END1 (the map teleport's path)
                tlpQueue(gTlp, *_drive, _streamer, *_renderer, tx, tz, _drive->heading());  // PK15: lands in state 3
                if (_audio) _audio->teleportQueued();
                std::printf("hub: left the camp menu: TLP to END1 (%.0f, %.0f)\n", tx, tz);
            }
            wasOpen = open;
        }
        if (action == fuel::ui::GameOverlay::Restart) {
            if (_race) killMission(_race, *_drive, _streamer, *_renderer);  // PK14: TTMN 0x54a910 -> KillMission, then the TLP
            if (_race) _race->begin(*_drive, _streamer, *_renderer), gRtcStartPending = true;
            else _drive->reset();
            _renderer->resetTemporalHistory();  // TAA: a restart is a cut
        }
        if (gOverlay->open() && !gOverlay->worldRuns()) {  // P67: the fader and the race-end grade keep running under the menus
            gCutFx.frame(dt, _chaseView, gRtc.active(), *_renderer, _audio.get());
            pauseBackdropTint(*_renderer);  // PK17: 0x46af90
            if (_haptics) _haptics->menuFrame(dt);  // P89: 0x553c80 -> 0x557c90; P101: then only the options page's test pulse
            if (_triggers) _triggers->off();          // P91: never left stiff in a menu
            return;
        }
    }
    if (_haptics) _haptics->setPaused(false);  // 0x557d80
    // P47: FUEL's cut-scenes. A race start plays TELEPORT_HELICO (mission state 3 STREAMING, 0x632c10) with the
    // player and the race held; PAUSE / MENU_VALID skip it (SKIPABLE, SKIP_BEFORE_END: the last second plays).
    if (gRtcEnabled && _race && gRtcStartPending && gLibrary && !(gOverlay && gOverlay->holdRaceStart())) {  // P78: logo stencil first
        gRtcStartPending = false;
        gCutFx.missionLeft();       // P67: SetState 0 then 3: the race-end grade ramps down (0x4b6050)
        gCutFx.missionStreaming();  // P67: STREAMING holds the screen black (0x4382e0) until the RTC's FROMBLACK
        if (_garage && !_garage->entries.empty()) gRtc.setVehicleMesh("MESH_ID_" + _garage->label(_garage->current));  // P96: 0x103
        if (!gRtc.play(gDataDir, "TELEPORT_HELICO", startFather(*_race), 0x1fb, *gLibrary, *_renderer)) gCutFx.fader.fadeIn(0.3f);  // [port]
    }
    RtcView::Frame rtcFrame;
    bool rtcHold = false, rtcPause = false;  // P76: rtcPause = 0x10e "pause all animation" (no race clock, input, traffic, effects)
    if (_streamer) _streamer->setRtcRunning(gRtc.active());  // P67: R_RUNNING / HIDEBYRTC (0x512920 at Play / End)
    if (gRtc.active()) {
        if (gInputLoaded && (gInput.record(fuel::input::Pause).justPressed || gInput.record(fuel::input::MenuValid).justPressed)) gRtc.skip(*_renderer);
        if (gRtc.name() == "VI_BIKE") gRtc.setFather(vehicleFather(*_drive));          // 0xe9: on the player vehicle
        if (gRtc.name() == "TR_DEATH") gRtc.setFather(cameraFather(_chaseView));       // 0xea: on the game camera
        gRtc.setAttachedPose(vehicleFather(*_drive));  // PK13: 0x5148a0 latches the vehicle's pose
        rtcFrame = gRtc.update(dt, *_renderer, &gCutFx.fader, _audio.get());
        if (rtcFrame.shake) gCutFx.shake.start();  // STARTRTCFX SHAKECAM: camera +0x344 = 1, +0x348 = 0.3 (0x53e740)
        if (rtcFrame.respawns > 0) _drive->reset(), _renderer->resetTemporalHistory();  // 0x10b: the vehicle code respawns (0x52f420); P76: the loop is let go by 0x10e (TAA: a cut)
        rtcPause = rtcFrame.pauseGame;
        rtcHold = gRtc.name() == "TELEPORT_HELICO";
        if (rtcFrame.ended) gRtc.stop(*_renderer);
    }
    if (gOverlay && gOverlay->logoHolds()) rtcHold = true;  // P78: the teleport holds the player while the logo covers
    auto down = [&](unsigned short a, unsigned short b) { return _keys.count(a) || _keys.count(b); };
    fuel::DriveInput in;
    if (gInputLoaded) {  // P47: FUEL's actions (records 0x669850, block 0x42e160); steer shaping is the vehicle's
        gInput.update(dt, _keys, readPad());
        const fuel::input::Block b = gInput.block();
        in.throttle = b.accel, in.brake = b.brake, in.steer = b.stickX, in.stickZ = b.stickZ, in.handbrake = b.handbrake;
        in.nitro = b.boost;  // [nitro] FUEL's BOOST slot = the Nitro button (pad LB / key E by default, core/input_map.cpp)
        if (b.respawn) {  // RESPAWN (Y / F): TR_DEATH, then the respawn on its event 0x10b (0x528c10 -> 0x52f370)
            if (!(gRtcEnabled && gLibrary && !gRtc.active() && gRtc.play(gDataDir, "TR_DEATH", cameraFather(_chaseView), 0x43, *gLibrary, *_renderer)))
                if (!gRtc.active()) _drive->reset(), _renderer->resetTemporalHistory();  // TAA: a cut
        }
        if (gInput.record(fuel::input::Context).justPressed && !(gOverlay && gOverlay->menu().archEvent() >= 0)) gGps3dVisible = !gGps3dVisible;  // B / Return
        // PK17: 0x4442f0 / 0x444400 -> 0x539ac0: no pause page (Map tab, arch page) while an RTC runs or is requested.
        if (gInput.record(fuel::input::Infos).justPressed && gOverlay && !gOverlay->open() && gOverlay->pauseAllowed()) gOverlay->openInfos();
        if (gInput.record(fuel::input::MenuValid).justPressed && gOverlay && !gOverlay->open() && !_race && gOverlay->pauseAllowed()) gOverlay->openArch();  // P75 0x4f6fd0  // Back / R: Map tab (0x46e750)
    } else {
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
        if (pad.buttonY.pressed) _drive->reset(), _renderer->resetTemporalHistory();  // TAA: a cut
    }
    }
    _frame.dt = dt;
    double t0 = CACurrentMediaTime();
    if (rtcPause) in = fuel::DriveInput{};  // PlayerMove 0x42dc40 skipped
    if (_race) {  // P97: the last frame's game camera for the AI ranks (0x5d3300)
        _race->view = _chaseView;
        const CGSize ds = self.drawableSize;
        if (ds.height > 0) _race->viewAspect = float(ds.width / ds.height);
    }
    if (_race && !rtcHold && !rtcPause) _race->update(dt, *_drive, in);
    if (_race) _drive->setRaceState(_race->state == RaceSession::Countdown ? 4 : _race->state == RaceSession::Finished ? 6 : 0);
    if (_race && _race->state == RaceSession::Finished) _drive->endingRestore();  // PK14: 0x632570 (states 6 / 7, every frame)
    if (_streamer && !rtcHold && !rtcPause) _streamer->breakThrough(*_drive, dt, *_renderer, _drive->physics());  // P50: 0x51edb0
    if (gTraffic && !rtcHold && !rtcPause && !_race) {  // P58: the camera of the last frame
        const float h = _drive->heading();
        const bool cam = simd_length_squared(_chaseView.eye) > 0;
        gTraffic->step(dt, *_drive, cam ? _chaseView.eye : _drive->position(), cam ? _chaseView.target - _chaseView.eye : simd_make_float3(std::sin(h), 0, std::cos(h)));
    }
    if (gOverlay && gOverlay->worldRuns()) in = {};  // P75: end-race pages: Input_NeutralBlock (0x4ba590)
    if (!rtcHold && !rtcPause) _drive->update(dt, in);  // PK13: 0xa65878: PhysicWorld_Update 0x680f90 calls vf106 (0x52f200 -> 0x5d4640: rider anim stopped) instead of stepping
    _frame.physicsMs = (CACurrentMediaTime() - t0) * 1000.0;
    stepRaceMarkers(_race, _drive->position(), *_renderer);  // P69
    if (_race) {
        _renderer->setMarkers(_race->markers());
        if (_race->state == RaceSession::Finished && !_race->reported) {
            _race->reported = true;
            std::printf("%s finished: order\n%s", _race->race().name.c_str(), _race->standings(*_drive).c_str());
            if (gRtcEnabled && gLibrary)  // states 6/7 ENDING_WON/LOST: VI_BIKE (0x632570 -> 0x62f770), DONTSHUT PLAYER_MOVE
                gRtc.play(gDataDir, "VI_BIKE", vehicleFather(*_drive), 0x40, *gLibrary, *_renderer);
            gCutFx.missionEnded();  // P67: SetState 6/7 -> MissionHUD +0xd38 = 1 (the end grade ramps in over 1 s)
            if (gOverlay) gOverlay->openResults([self resultRows], _race->rules.outcome == fuel::RaceRules::Won, raceLevel(_race->event()));  // medals
        }
    }
    showOpponents(*_drive, *_renderer);
    t0 = CACurrentMediaTime();
    if (_streamer) _streamer->update(_drive->position(), *_renderer, _drive->physics());
    if (gFreeRoam && gFreeRoam->step(dt, *_drive, _streamer, _race != nullptr, _chaseView) && _streamer)  // P44
        _streamer->update(_drive->position(), *_renderer, _drive->physics(), true);
    if (gFreeRoam && gOverlay && !gOverlay->open()) {  // PK03: the Map tab's hub arrival (0x481b30)
        const fuel::HubPresence::Action a = gFreeRoam->takeHubAction();
        const auto& hubs = gOverlay->menu().hubs();
        if (a.kind == fuel::HubPresence::Action::TeleportToHub && a.hub >= 0 && size_t(a.hub) < hubs.size()) {
            // TeleportToHub 0x5495a0: the player on CAMERA_CAREER facing TARGET_CAREER through the teleport queue, then
            // the queued StartHubHUD <uid> (no KillMission: the request only fires while no race runs).
            const fuel::HubPoint& c = hubs[size_t(a.hub)].camp;
            gOverlay->startLogo();
            tlpQueue(gTlp, *_drive, _streamer, *_renderer, c.x, c.z, c.yaw, a.hub);  // PK15: StartHubHUD runs at state 3
            if (_audio) _audio->teleportQueued();
            std::printf("hub: TeleportToHub %s\n", hubs[size_t(a.hub)].id.c_str());
        } else if (a.kind == fuel::HubPresence::Action::OpenMenu) {
            gOverlay->openPause();  // vf22(vf09() = 1, tab 0)
        } else if (a.kind == fuel::HubPresence::Action::OpenAlert) {
            gOverlay->openPause();  // 0x47da10: vf22, then TT_PAGE_HUB_ALERT (0xc7)
            float x = 0, z = 0;
            if (gFreeRoam->hubEnd1(x, z)) gOverlay->menu().openHubAlert(x, z);
            std::printf("hub: TT_PAGE_HUB_ALERT (locked hub)\n");
        }
    }
    _frame.streamMs = (CACurrentMediaTime() - t0) * 1000.0;
    if (rtcFrame.attached) {  // P67: 0xf1: the player vehicle hangs on the RTC node (TELEPORT_HELICO)
        auto parts = _drive->partWorlds();
        attachParts(parts, vehicleFather(*_drive), rtcFrame.attach);
        _renderer->setInstanceWorlds(parts);
    } else {
        _renderer->setInstanceWorlds(_drive->partWorlds());
    }
    // Chase camera: swings behind the vehicle; mouse look decays back, drag and
    // scroll raise and zoom it.
    // FUEL's camera: mouse drag = the look stick (CAM X/Y), V = the CAMERA button (views 5 -> 1 -> 4).
    fuel::cam::CameraControls cc;
    cc.stickX = std::clamp(_camYawOffset * -2.0f, -1.0f, 1.0f);
    cc.stickY = std::clamp((_cam.pitch - _startCam.pitch) * 2.0f, -1.0f, 1.0f);
    cc.viewChange = _viewChange;
    _viewChange = false;
    if (gInputLoaded) {  // P47: 0x414dd0 (look axes with FUEL's 0.5 dead zone, invert options; R3 / keypad 1 / V)
        const fuel::input::CameraInput ci = gInput.camera();
        if (ci.active) cc.stickX = ci.x, cc.stickY = ci.y;
        cc.viewChange = cc.viewChange || ci.viewChange;
    } else if (GCExtendedGamepad* pad = GCController.controllers.firstObject.extendedGamepad) {
        if (std::fabs(pad.rightThumbstick.xAxis.value) > 0.1f) cc.stickX = pad.rightThumbstick.xAxis.value;
        if (std::fabs(pad.rightThumbstick.yAxis.value) > 0.1f) cc.stickY = -pad.rightThumbstick.yAxis.value;
    }
    _chaseView = _chase.update(dt, *_drive, cc, cameraOverride(_race));
    if (rtcFrame.camera) _chaseView = rtcFrame.cam;  // the RTC camera after its first cut (viewport +0x24)
    gHudHidden = rtcHold || rtcFrame.hudHidden;      // HUD off during the mission-start RTC (0x632c10) and 0x10d
    if (!std::getenv("FUEL_NO_WORLD_ANIM")) {
        pumpWorldAnimEvents(_streamer);
        gWorldAnim.setAudio(_audio.get());                                                   // PK29: event sounds
        gWorldAnim.setEye({_chaseView.eye.x, _chaseView.eye.y, _chaseView.eye.z});         // PK29: event FX range
        gWorldAnim.advance(dt);
        gWorldAnim.upload(*_renderer, _drive ? &_chaseView : nullptr);  // P92: only what the next frame can show
    }
    gHeliView.frame(dt, _race && _race->rules.helicopter ? &_race->rules.hel : nullptr, *_renderer);  // PK07
    f14Step(dt, *_drive, *_renderer);  // PK27
    _camYawOffset *= std::max(0.0f, 1.0f - dt * 4.0f);
    _cam.pitch = _startCam.pitch + (_cam.pitch - _startCam.pitch) * std::max(0.0f, 1.0f - dt * 4.0f);
    applyVehicleLods(*_drive, *_renderer, _chaseView.eye);  // P104
    _drive->uploadRiders();
    _renderer->setFocus(_chase.focus());
    _renderer->speedBlur = _chase.speedEffect();
    _renderer->speedBlurAbsolute = _chase.pose().blur;  // FUEL's viewport +0x1dc (P574 x the speed fraction)
    if (rtcFrame.camera) _renderer->speedBlur = _renderer->speedBlurAbsolute = 0;  // P47: the RTC camera has no speed blur
    if (rtcFrame.clearRadialBlur) _renderer->speedBlur = _renderer->speedBlurAbsolute = 0;  // PK13: Play flag 2 (0x53b6b0)
    if (gOverlay) gOverlay->setRtcLoading(rtcFrame.loading);                                 // PK13: 0x539a50 "LOADING..."
    if (_audio) _audio->setVehicleSoundsHeld(rtcFrame.loading);                              // PK13: 0x5d8ed0 skipped
    gCutFx.frame(dt, _chaseView, rtcFrame.camera, *_renderer, _audio.get());     // P67: fade, shake, end grade, sound fade
    pauseBackdropTint(*_renderer);  // PK17: 0x46af90 while the pause page fades out over the running world
    _dust.update(dt, *_drive, _chaseView.eye, _chaseView.target);
    gStorms.update(dt, _chaseView.eye, _streamer);
    gDamageView.step(dt, *_drive, _streamer, *_renderer);  // P85: crash debris, damage smoke
    _renderer->setDust(gStorms.withSprites(_dust.particles(), _chaseView.eye));
    _renderer->setLensVehicle(_dust.lens());  // PK24
    _renderer->setTornadoes(gStorms.draws(_chaseView.eye));  // P65
    gStorms.hookWeather(*_renderer);
    updatePlacedFx(dt, _chaseView.eye, *_renderer);
    const fuel::VehicleState st = _drive->state();
    _frame.speedKmh = std::fabs(st.speedKmh);
    _frame.gear = st.gear;
    _frame.wheelsOnGround = st.wheelsOnGround;
    _frame.vehicles = _drive->vehicleCount();
    const simd_float3 at = _drive->position();
    _frame.x = at.x, _frame.y = at.y, _frame.z = at.z;
    if (_audio) {  // FUEL's vehicle sounds, heard from the camera
        _audio->setListener({_chaseView.eye.x, _chaseView.eye.y, _chaseView.eye.z}, {_chaseView.target.x, _chaseView.target.y, _chaseView.target.z},
                            _chase.view());
        for (int i = 1; i < _drive->vehicleCount(); ++i) _audio->updateRival(size_t(i - 1), dt, _drive->state(i));
        _audio->setGame(gameSound(*_drive, _race, {_chaseView.eye.x, _chaseView.eye.y, _chaseView.eye.z}, *_renderer, _streamer, rtcHold));
        if (_streamer)  // P61: prop impact sounds (vt[139]) and the tornadoes' loops
            for (const auto& o : _streamer->updateImpacts(dt, _drive->physics()))
                _audio->objectSound(o.lodName, o.symbol, o.pos, o.pitch, o.volume, o.minDist, o.maxDist);
        gStorms.sounds(*_audio);
        heliSounds(*_audio, _race);  // PK07
        f14Sounds(*_audio, dt, _chaseView.eye);  // PK27
        fuel::VehicleState sst = st;  // P96: hanging on the RTC node (0xf1 -> 0x514b70) the vehicle's sounds follow the node
        if (rtcFrame.attached) sst.chassis.position = {rtcFrame.attach[12], rtcFrame.attach[13], rtcFrame.attach[14]};
        _audio->update(dt, sst, in.throttle);
    }
    if (_haptics) {  // P89: after the sounds asked; a cut-scene start stops every vibration (0x53b860)
        // [deviation, user request] the drive rumble: FUEL's ground BUMPS under the loaded wheels, and the tyres' slip.
        fuel::PadHaptics::DriveFeel df;
        if (const fuel::fv::FuelVehicle* m = _drive->physics().model(0)) {
            df.active = !(gOverlay && (gOverlay->open() || gOverlay->logoActive())) && !gRtc.active() && !rtcPause && !rtcHold && !m->wrecked();
            df.speed = m->forwardSpeed();
            float rough = 0, slip = 0;
            int n = 0;
            for (int w = 0; w < m->wheelCount(); ++w) {
                const fuel::fv::WheelRec& r = m->wheel(w);
                if (r.load <= 1e-6f) continue;
                const fuel::fv::GroundBlock& g = m->ground(w);
                rough += std::max(g.bumpsWheel, g.bumpsRot);
                const float util = r.sumDen >= 1e-6f ? r.sumFn / r.sumDen : 0.0f;  // FUEL's grip use (0x523c50), >= 1 sliding
                const float ahead = std::fabs(std::fabs(m->rimSpeed(w)) - std::fabs(df.speed));  // spin or lock
                slip += std::max(std::clamp((util - 0.85f) / 0.45f, 0.0f, 1.0f),
                                 ahead > 1.5f ? std::clamp((ahead / std::max(std::fabs(df.speed), 3.0f) - 0.2f) / 0.8f, 0.0f, 1.0f) : 0.0f);
                ++n;
            }
            df.airborne = n == 0;
            if (n) df.rough = rough / float(n), df.slip = slip / float(n);
            df.nitro = st.nitro.amount;  // [nitro]
        }
        _haptics->drive(dt, df);
        static bool rtcWas = false;
        _haptics->frame(dt, gRtc.active() && !rtcWas);
        rtcWas = gRtc.active();
    }
    if (_triggers && _drive->physics().model(0)) {  // P91: off in menus, cut-scenes and loading; else the grip feel
        fuel::trig::Telemetry tt = fuel::trig::telemetryFrom(*_drive->physics().model(0), st, _drive->setup().vehicleClass, in.brake);
        tt.menu = gOverlay && gOverlay->open();
        tt.cutscene = gRtc.active() || rtcPause || rtcHold;
        tt.loading = gOverlay && gOverlay->logoActive();
        auto role = [](int pad) {
            return pad == fuel::input::PadRT ? 1 : pad == fuel::input::PadLT ? 2 : 0;
        };
        fuel::trig::Role l = fuel::trig::Role::Brake, r = fuel::trig::Role::Throttle;  // the port's pads: RT accel, LT brake
        if (gInputLoaded) {
            l = r = fuel::trig::Role::None;
            for (int a : {fuel::input::Accel, fuel::input::Brake})
                for (int p : gInput.padsOf(a)) {
                    const fuel::trig::Role ro = a == fuel::input::Accel ? fuel::trig::Role::Throttle : fuel::trig::Role::Brake;
                    if (role(p) == 1) r = ro;
                    if (role(p) == 2) l = ro;
                }
        }
        _triggers->frame(dt, tt, l, r);
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
    _renderer->gpsRibbons = _drive ? gGpsDraw : std::vector<fuel::MeshRenderer::GpsRibbonDraw>{};  // P86
    _renderer->navBeams = _drive ? gNavBeams : std::vector<fuel::MeshRenderer::NavBeamDraw>{};       // PK19
    _renderer->render(cb, drawable.texture, _drive ? _chaseView : _cam);
    // P102: every frame of a session (the VignettageHUD picture is drawn while the HUDs are hidden too, 0x443290).
    if (gOverlay && _drive) gOverlay->draw(cb, drawable.texture, [self hudState], !gHudHidden);
    if (gRecorder.active()) {
        _frame.encodeMs = (CACurrentMediaTime() - t0) * 1000.0;
        gRecorder.watchGpu(cb);
        if (gRecorder.frame(_frame)) gRecorder.grab(cb, drawable.texture);
        gRecorder.capture(cb, drawable.texture);  // rolling buffer for K and the flicker detector
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

int main(int argc, const char** argv) {
    @autoreleasepool {
        Options opt = parseArgs(argc, argv);
        if (opt.menuCheck) {
            const int bad = fuel::ui::runMenuCheck(opt.dataDir, fuel::loadSettings().language);
            // P101: the options pages over a scratch settings.txt / career.sav (last: it points $FUEL_SETTINGS there)
            // PK18: the menu click sounds and the pause chrome (tab rules, halos, BandeauHelp keys, sound pick)
            const int pk18 = fuel::menuSfxCheck(opt.dataDir);
            const int pk25 = fuel::ui::runAlertCheck(opt.dataDir);  // PK25: save / load alerts on a scratch career.sav
            // PK26: the {STR_*} help-text variables (0x4e2d00 / 0x4e2820 / 0x4e2250) and the boot screens' flow.
            const int pk26 = fuel::ui::runHelpTextCheck(opt.dataDir) + fuel::ui::runBootCheck(opt.dataDir, fuel::loadSettings().language);
            const int pk17 = fuel::ui::runPauseCheck(opt.dataDir);  // PK17: pause open / close, save-on-close wait
            const int padUx = fuel::ui::pad::runPromptCheck() + fuel::ui::runPadAtlasCheck(opt.dataDir) + fuel::ui::runScrollCheck(opt.dataDir);  // pad-ux
            const int remap = fuel::ui::runMenuInputCheck(opt.dataDir) + fuel::ui::runRemapCheck(opt.dataDir);  // [port] remap
            return bad + pk17 + pk18 + pk25 + pk26 + padUx + remap + fuel::menuAudioCheck(opt.dataDir) + fuel::ui::runChromeCheck(opt.dataDir) + fuel::ui::runOptionsCheck(opt.dataDir);  // P49: the MENU music path
        }
        if (opt.careerCheck) return fuel::runCareerCheck(opt.dataDir);
        if (opt.careerAudit) return fuel::ui::runCareerAudit(opt.dataDir);
        if (!opt.upgradeCmd.empty()) return fuel::ui::runUpgradeCommand(opt.dataDir, fuel::loadSettings(), opt.upgradeCmd);  // [upgrades U2]
        if (opt.hudCheck) return fuel::hud::runChecks(fuel::hud::loadPp(opt.dataDir + "/CommonComponent/Param/pp001.tsc")) ? 1 : 0;
        if (opt.renderCheck) return fuel::runRenderCheck(opt.dataDir);
        if (opt.camCheck) return runCamCheck(opt.dataDir);
        if (opt.inputCheck) return fuel::input::runInputCheck(opt.dataDir);
        if (opt.animCheck) return fuel::anim::runAnimCheck(opt.dataDir);
        if (opt.aiVectors) return runAiVectors(opt.dataDir);
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
        // P78: FUEL's loading screen (ui/loading_screen.mm: the world level's LOADING_BITMAP, tips, LOADING...), drawn by
        // its own thread; it falls back to a plain "Loading…" label.
        std::unique_ptr<fuel::ui::LoadingWindow> loadingWindow;
        if (opt.inGame && opt.shot.empty() && !opt.aiCheck && !opt.eventCheck && !opt.hubCheck && !opt.propCheck && !opt.audioCheck && !opt.hapticsCheck && !opt.triggerCheck) {
            const fuel::Settings ls = fuel::loadSettings();
            loadingWindow = std::make_unique<fuel::ui::LoadingWindow>(opt.dataDir, ls.language, opt.settings ? gameWindowSize(ls) : NSMakeSize(1280, 800),
                                                                       fuel::ui::LoadingWindow::World);
        }
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
            gHubTravel = !opt.haveAt && opt.race.empty() && !opt.noHubArrival;  // PK03: TeleportToHub, not a race end's free ride
            if (!opt.haveAt) opt.atX = h->camp.x, opt.atZ = h->camp.z, opt.atYaw = h->camp.yaw, opt.haveAt = true;  // P78: --at wins (free ride from the race end)
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
        if (const char* e = std::getenv("FUEL_P95_OFF")) renderer.p95Off = uint32_t(std::strtoul(e, nullptr, 0));  // P95: image A/B
        renderer.postEffects = std::getenv("FUEL_NO_POST") == nullptr;  // FUEL_NO_POST=1: plain tone mapping (O toggles in the window)
        renderer.timeFromCommandLine = opt.timeOfDay >= 0;
        renderer.forcedTimeOfDay = opt.timeOfDay;  // --time: ForceDayTime; else 08:00 + the game clock x PP371
        renderer.weather.cloud = opt.cloud, renderer.weather.fog = opt.fog, renderer.weather.wet = opt.wet;
        renderer.weatherFromWorld = !opt.weatherGiven;
        fuel::Settings playerSettings;
        if (opt.settings) {
            playerSettings = fuel::loadSettings();
            applySettings(playerSettings, renderer);
        }
        if (opt.perfCheck) renderer.dynamicResolution = true, renderer.shadow2x2 = true;  // P93: the game's defaults
        if (opt.dynres >= 0) renderer.dynamicResolution = opt.dynres == 1;
        if (opt.dynresMin > 0) renderer.dynamicResolutionParams.minScale = std::clamp(opt.dynresMin, 0.5f, 1.0f);
        if (opt.shadow2x2 >= 0) renderer.shadow2x2 = opt.shadow2x2 == 1;
        if (opt.renderScale > 0) renderer.fixedRenderScale = std::clamp(opt.renderScale, 0.25f, 1.0f);
        if (!opt.upscaler.empty()) renderer.upscaler = fuel::MeshRenderer::upscalerFromName(opt.upscaler, renderer.upscaler);  // TAA: + metalfx-temporal
        // P93 tuning (not settings): FUEL_DYNRES_TARGET / _RAISE (ms), _SHARE (the fixed share), _UPSCALE (ms).
        if (const char* e = std::getenv("FUEL_DYNRES_TARGET")) renderer.dynamicResolutionParams.targetMs = std::stof(e);
        if (const char* e = std::getenv("FUEL_DYNRES_RAISE")) renderer.dynamicResolutionParams.raiseMs = std::stof(e);
        if (const char* e = std::getenv("FUEL_DYNRES_SHARE")) renderer.dynamicResolutionParams.fixedShare = std::stof(e);
        if (const char* e = std::getenv("FUEL_DYNRES_UPSCALE")) renderer.dynamicResolutionParams.upscaleMs = std::stof(e);
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
                DustSystem::setDataDir(opt.dataDir);  // FUEL's wheel effects: pp001.tsc + FUEL.exe tables, ground colour
                DustSystem::groundColour = [](double x, double z) { return groundPointColour(gGround, x, z); };
                fuel::DriveWorld::setDefaultSurfaceQuery([](double x, double z) { return gSurfaces.at(x, z); });
                // P70b: breadcrumbs face the player's GPS target when there is no race path (0x421dc0: MapHUD +0x60).
                fuel::DriveWorld::setDefaultCrumbGpsTarget([](double, double, double& x, double& z) {
                    float fx = 0, fz = 0;
                    if (!gOverlay || !gOverlay->menu().gpsTarget(fx, fz)) return false;
                    x = fx, z = fz;
                    return true;
                });
                if (!std::getenv("FUEL_ONEHOT_CLASSES"))  // p32: FUEL's blended class fractions (0x6b76a0)
                    fuel::DriveWorld::setDefaultClassQuery([](double x, double y, double z, float* frac, float& depth) {
                        const fuel::GroundGenerator* g = gGroundGen.load();
                        if (!g) {  // no ground generator (no world loaded): bare earth
                            frac[3] = 1;
                            depth = 0;
                            return;
                        }
                        g->classFractions(x, y, z, frac, depth);
                    });
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
                std::stringstream text(fuel::custom::readScript((fs::path(opt.dataDir) / "LocalizedComponent/GameTsc/Story/VehiclesDef.tsc").string()));
                riders = std::make_unique<RiderView>(*library.files().back(), text.str(), renderer,
                                                     (fs::path(opt.dataDir) / "XFinalComponent/FUEL.exe").string());
                gRiderView = riders.get();
                if (!opt.pilot.empty()) {  // P66: the garage's rider
                    fuel::Rider::Parts p;
                    int* f = &p.head;
                    std::stringstream ps(opt.pilot);
                    for (std::string v; std::getline(ps, v, ',') && f <= &p.skin; ++f) *f = std::max(1, std::atoi(v.c_str()));
                    riders->setParts(p);
                }
                if (!opt.outfit.empty()) {  // PHYSIQUE: the garage's outfit (the player's rider only)
                    fuel::RiderOutfit o;
                    o.outfit = std::clamp(std::atoi(opt.outfit.c_str()), 0, fuel::kRiderOutfits - 1);
                    if (const size_t c = opt.outfit.find(','); c != std::string::npos) o.helmet = std::atoi(opt.outfit.c_str() + c + 1) != 0;
                    riders->setOutfit(o);
                }
                {  // PHYSIQUE: the glute springs (settings.txt rider_jiggle, default subtle; --jiggle overrides)
                    std::string j = "subtle";
                    if (const auto it = playerSettings.all.find("rider_jiggle"); opt.settings && it != playerSettings.all.end()) j = it->second;
                    if (!opt.jiggle.empty()) j = opt.jiggle;
                    riders->setJiggle(fuel::riderJiggleSetting(j));
                }
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
        std::unique_ptr<fuel::RoadGraph> roadGraph;
        std::optional<std::array<fuel::RoadTypeParams, 21>> roadTypes;  // the AI's road blend (+0xb4)
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
            if (!opt.drive) applyStaticUpgrades(scene, *assembly, opt.dataDir);  // [upgrades U5]
            look = assembly->look(prefs);
            renderer.setPaint(look.paint());
            if (opt.drive) {
                const auto params = fuel::parseVehiclesParam(fuel::custom::readScript((fs::path(opt.dataDir) / "CommonComponent/Param/VehiclesParam.tsc").string()));
                const std::string name = assembly->variant().label;
                const auto it = params.find(name);
                if (it == params.end()) die("no driving parameters for " + name + " in Param/VehiclesParam.tsc");
                try {
                    const fuel::AssembledVehicle av = assembly->assembler->assemble(assembly->variant(), assembly->lod);
                    fuel::VehicleSetup setup = fuel::makeVehicleSetup(subject, name, it->second, assembledHubs(av));
                    fuel::upg::applyPlayer(setup, fuel::fv::FuelVehicle::powerScale);  // [upgrades U1] the player's stages
                    drive = std::make_unique<DriveSession>(av, setup, *terrain, opt.haveAt ? opt.atX : -51358.0, opt.haveAt ? opt.atZ : -30043.0,
                                                           opt.haveAt ? opt.atYaw : 0.0f,
                                                           riders ? riders->animated(name, av, subject, setup.params, true) : std::vector<uint32_t>{});
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
                try {  // P85: the crash debris Lods (IT_DYN_GrosBoulon ... of 0x52ee60) live in DATAS/FONTES.DPC
                    library.add(std::make_unique<fuel::DpcFile>((fs::path(opt.dataDir) / "CommonComponent/DATAS/FONTES.DPC").string()));
                } catch (const std::exception& e) {
                    std::fprintf(stderr, "fuelview: %s\n", e.what());
                }
                for (const auto& f : library.files()) lookup.push_back(f.get());
                streamer = std::make_unique<WorldStreamer>(subs, lookup);
                if (opt.popCheck) {  // P107: the pop check sees streaming at a fixed latency, not the machine's load (drive.h)
                    const char* jf = std::getenv("FUEL_POP_JOB_FRAMES");  // 0: never wait (as the window)
                    streamer->setMaxJobFrames(jf ? std::atoi(jf) : 30);
                }
                if (!std::getenv("FUEL_NO_PARTICLES")) {  // p48: placed Particles_Z emitters (USA1 root + sub-worlds)
                    for (const fuel::DpcFile* sub : subs) gPlacedFx.add(fuel::particles::collectPlacedParticles(*sub, lookup, false));
                    for (const auto& f : library.files())
                        if (fs::path(f->path()).filename() == "USA1.DPC")
                            gPlacedFx.add(fuel::particles::collectPlacedParticles(*f, lookup, true));
                    std::printf("placed particle emitters: %zu\n", gPlacedFx.placements());
                    if (const char* d = std::getenv("FUEL_SFX_DEBUG"); d && std::atoi(d) == 4)
                        for (size_t i = 0; i < gPlacedFx.placements(); ++i) {
                            const auto& p = gPlacedFx.placement(i);
                            std::printf("  emitter %zu at %.0f %.0f %.0f: %zu elements, material %08x, flags %x, view %.0f\n", i, p.world[12], p.world[13],
                                        p.world[14], p.emitter->elements.size(), p.emitter->elements[0].material, p.emitter->elements[0].flags,
                                        p.emitter->viewDist);
                        }
                }
                {  // ---- PK28: placed RotShape_Z (USA1 root + sub-worlds; a course's only while its sub-world is attached)
                    std::vector<fuel::rotshape::Placement> rs;
                    for (const fuel::DpcFile* sub : subs) {
                        auto v = fuel::rotshape::collect(*sub, lookup, false, fs::path(sub->path()).filename().string());
                        rs.insert(rs.end(), v.begin(), v.end());
                    }
                    for (const auto& f : library.files())
                        if (fs::path(f->path()).filename() == "USA1.DPC") {
                            auto v = fuel::rotshape::collect(*f, lookup, true, "");
                            rs.insert(rs.end(), v.begin(), v.end());
                        }
                    std::printf("placed rot shapes: %zu\n", rs.size());
                    if (std::getenv("FUEL_ROTSHAPE_TRACE"))
                        for (const auto& p : rs)
                            std::printf("  rotshape %08x node %08x at %.0f %.0f %.0f scale %.2f points %zu front %d view %.0f near %.0f sub %s\n", p.shape->id,
                                        p.nodeId, p.world[12], p.world[13], p.world[14], p.lodScale, p.shape->points.size(), p.shape->front ? 1 : 0,
                                        p.shape->viewDist, p.shape->nearFade, p.sub.c_str());
                    renderer.setRotShapes(std::move(rs), [s = streamer.get()](const std::string& n) { return s->subWorldActive(n); });
                }  // ---- PK28 end
                // P50: node names that clone.tsc makes game objects (IT_DYNOBJ_G movable, IT_DECORDYNMESH_G breakable, ...).
                streamer->setGameClasses(fuel::loadCloneClasses((fs::path(opt.dataDir) / "CommonComponent/clone.tsc").string()));
                // P39: object conditions ask the career (career.sav, read only): open = listed and available
                // (+0x10d), won = placed (rank != 3). FUEL's 0x5628d0 / 0x562890 read AddMISSION records [I].
                try {
                    auto career = std::make_shared<fuel::Career>(fuel::loadCareerRules(opt.dataDir));
                    career->load();
                    streamer->setMissionState(
                        [career](const std::string& m) { return !career->status(m) || career->eventAvailable(m); },
                        [career](const std::string& m) { const fuel::MissionStatus* s = career->status(m); return s && s->bestRank != 3; });
                } catch (const std::exception& e) {
                    std::fprintf(stderr, "fuelview: career: %s\n", e.what());
                }
                for (const auto& f : library.files())
                    if (fs::path(f->path()).filename() == "USA1.DPC") {
                        gWorldDpc = f.get();
                        gLibrary = &library;
                        gDataDir = opt.dataDir;
                        gCutFx.load(opt.dataDir);
                        if (!std::getenv("FUEL_NO_WORLD_ANIM"))  // P47: animated skins before the first placements load
                            std::printf("world animation: %zu animated skin classes\n", gWorldAnim.init(lookup, *f, renderer));
                        if (!gHeliView.load(lookup, *f, renderer)) std::printf("race helicopter: no IT_HELICO object in the world\n");  // PK07
                        f14Init(*f, opt.dataDir, terrain.get());  // PK27
                        if (terrain && !std::getenv("FUEL_NO_STORMS")) {  // P61: tornadoes (storm_view.h)
                            gStorms.init(*f, lookup, [t = terrain.get()](double x, double z) { return t->smoothHeightAt(x, z); });
                            gStorms.setForest([](double x, double z) {  // P65: 0x5bd430's forest test
                                const fuel::GroundGenerator* g = gGroundGen.load();
                                return g ? g->forestAt(x, z) : 0.0f;
                            });
                            if (drive) gStorms.attach(*drive);
                        }
                        try {
                            vegetation = std::make_unique<fuel::Vegetation>(fuel::loadVegetation(*f));
                            {
                                std::unordered_set<uint32_t> foliage;
                                for (const auto* list : {&vegetation->trees, &vegetation->bushes})
                                    for (const fuel::Species& sp : *list) foliage.insert(sp.full);
                                renderer.setFoliage(std::move(foliage));
                            }
                            streamer->setVegetation(vegetation.get(), terrain.get());
                            streamer->addResident(*f);  // USA1's own scene placements (World_Z root, p36)
                            streamer->setSubWorlds(fuel::loadSubWorldRefs(*f));  // FUEL's region / course streaming (p35)
                            streamer->setSurfaces([](double x, double z) { return gSurfaces.at(x, z); });
                            roads = std::make_unique<fuel::RoadNetwork>(fuel::loadRoads(*f));
                            roadTypes = fuel::loadRoadTypes(*f);
                            try {
                                roadGraph = std::make_unique<fuel::RoadGraph>(*f);
                                gHubZones = fuel::loadHubZones(*f);  // P75: the Map page's hub zones
                            } catch (const std::exception& e) {
                                std::fprintf(stderr, "fuelview: road graph: %s\n", e.what());
                            }
                            streamer->setRoads(roads.get());
                            // PK05: Decor_FreeOfObjects 0x557650 (mission gates, POIs; the GenWorld boxes load with the vegetation).
                            if (!std::getenv("FUEL_NO_PLANT_CLEARANCE"))
                                fuel::loadPlantClearance(*vegetation, *f, opt.dataDir, roadGraph.get(), roadTypes ? roadTypes->data() : nullptr,
                                                         roadTypes ? roadTypes->size() : 0);
                            if (terrain)  // P69: FUEL's race banners and checkpoint flags (VisibleObjectMgr_G)
                                gRaceMarkers.init(lookup, opt.dataDir, roadGraph.get(), roadTypes ? &*roadTypes : nullptr,
                                                  [t = terrain.get()](float x, float z) { return t->smoothHeightAt(x, z); });
                            gSurfaces.roads = roads.get();
                            std::printf("roads: %zu drawable lines\n", roads->lines.size());
                            std::printf("vegetation: %zu tree and %zu bush species\n", vegetation->trees.size(), vegetation->bushes.size());
                            if (std::getenv("FUEL_STREAM_TRACE"))  // P50: the decor collision keys
                                for (const auto* list : {&vegetation->trees, &vegetation->bushes})
                                    for (const fuel::Species& sp : *list)
                                        std::printf("  species %s %08x: trunk %.3f, flags %u, size %.2f..%.2f\n", list == &vegetation->trees ? "tree" : "bush", sp.full, sp.trunk,
                                                    sp.flags, sp.size.lo, sp.size.hi);
                            ground = std::make_unique<fuel::GroundGenerator>(*f, *terrain, roads.get());  // p14
                            gGroundGen.store(ground.get());
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
                                        drive->physics().setLakeDepthQuery([lk, th](double x, double z) {  // P70b: 0x40fd10
                                            bool inside = false;
                                            for (const fuel::Lake& l : lk->lakes()) inside = inside || l.contains(x, z);
                                            return inside ? float(std::max(0.0, lk->depthAbove(x, z, th->smoothHeightAt(x, z)))) : 0.0f;
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
                            try {  // FUEL's AutoMesh / RoadMesh objects (p36)
                                auto am = std::make_unique<fuel::AutoMesh>((fs::path(opt.dataDir) / "CommonComponent").string(), *f, lookup);
                                std::printf("automesh: %zu + %zu + %zu cell rules, %zu road rules, %zu templates\n", am->ruleCount(0), am->ruleCount(1),
                                            am->ruleCount(2), am->ruleCount(3), am->templateCount());
                                // P49: the spawns that are IT_DECORMESH_G objects with a sound bank symbol 0 (decor fires)
                                am->setDecorSoundRules(fuel::decorSoundNames(*f, (fs::path(opt.dataDir) / "CommonComponent/clone.tsc").string()));
                                if (am->valid()) streamer->setAutoMesh(std::move(am));
                            } catch (const std::exception& e) {
                                std::fprintf(stderr, "fuelview: automesh: %s\n", e.what());
                            }
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
                // CUSTOM VEHICLES: the packs' vehicles (their own archives) are garage entries too: the player's racer
                // entry, Tab and the session's vehicle swaps see them.
                for (const fuel::custom::Vehicle& cv : fuel::custom::vehicles()) {
                    if (!params.count(cv.name)) continue;
                    const fuel::DpcFile* dpc = nullptr;
                    for (const auto& f : library.files())
                        if (f->path() == cv.dpc) dpc = f.get();
                    if (dpc == nullptr) {
                        try {
                            auto f = std::make_unique<fuel::DpcFile>(cv.dpc);
                            dpc = f.get();
                            library.add(std::move(f));
                        } catch (const std::exception& e) {
                            std::fprintf(stderr, "fuelview: %s: %s\n", cv.dpc.c_str(), e.what());
                            continue;
                        }
                    }
                    auto am = std::make_unique<AssemblyMode>();
                    am->assembler = std::make_unique<fuel::VehicleAssembler>(*dpc);
                    am->palettes = assembly->palettes;
                    am->riders = riders.get();
                    am->lod = 1;
                    for (size_t v = 0; v < am->assembler->variants().size(); ++v)
                        if (am->assembler->variants()[v].label == cv.name) {
                            if (dpc == &subject && cv.name == assembly->variant().label) garage->current = garage->entries.size();
                            garage->entries.push_back({garage->archives.size(), v});
                        }
                    garage->archives.push_back(std::move(am));
                }
                std::printf("garage: %zu drivable variants (Tab to switch)\n", garage->entries.size());
                if (!opt.hub.empty()) {  // free roam: on the camp, or the nearest clear spot
                    const fuel::HubPoint camp{"", opt.atX, opt.atZ, opt.atYaw};
                    const HubSpawn s = placeAtHubPoint(*drive, *streamer, renderer, *terrain, camp);
                    std::printf("hub %s: camp at (%.0f, %.0f) heading %.0f%s\n", opt.hub.c_str(), s.x, s.z, s.yaw * 57.2958f, s.nudged ? " (nudged to a clear spot)" : "");
                }
                if (opt.collisionCheck) {
                    // CR2-Yell-CP start, the prop-check spot, the CR1 jump, the CR26 cluster, Yellowstone's camp area
                    std::vector<std::array<float, 2>> spots = {{-39570, -28068}, {45850, -6250}, {-52638, -29119}, {-39053, 37523}};
                    if (!gHubs.empty()) spots.push_back({gHubs[0].camp.x, gHubs[0].camp.z});
                    return runCollisionCheck(gHubs, *drive, *streamer, renderer, *terrain, spots);
                }
                if (opt.hubCheck) return runHubCheck(gHubs, *drive, *streamer, renderer, *terrain, &gHubZones);  // PK03: + zones
                if (opt.eventCheck) {
                    std::function<float(float, float)> gw;
                    if (roadGraph && roadTypes)
                        gw = [g = roadGraph.get(), &types = *roadTypes](float x, float z) { return fuel::missionGateWidth(*g, types.data(), types.size(), x, z); };
                    std::vector<fuel::Event> events = fuel::loadEvents(opt.dataDir);
                    for (fuel::Event& e : events) snapMissionHeights(e, *terrain);  // PK14: 0x557e50 -> 0x4f5b90
                    return runEventCheck(std::move(events), *drive, *streamer, renderer, *terrain, opt.eventOnly, gw);
                }
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
                    if (const char* m = std::getenv("FUEL_SWEEP_MISSION")) streamer->setMission(m);  // course dressing of that race (P60)
                    for (const CaptureSpot* sp : todo) {
                        const CaptureView v = prepareCapture(*sp, *drive, *streamer, renderer, *terrain);
                        renderer.setFocus(v.focus);
                        renderer.speedBlur = 0;
                        Options frame = opt;
                        frame.shot = opt.captureAll.empty() ? opt.shot : (fs::path(opt.captureAll) / (sp->name + ".png")).string();
                        std::printf("%s\n", v.report.c_str());
                        if (const char* sw = std::getenv("FUEL_SWEEP")) {
                            // FUEL_SWEEP=YAW0,YAW1,N[,PITCH[,HEIGHT]]: N consecutive frames from the spot (eye HEIGHT m above
                            // the vehicle, default 1.6), the heading turning from YAW0 to YAW1 degrees (0 = +z, 90 = +x),
                            // pitch in degrees; NAME_0000.png ... (P60: frame-to-frame visibility at the screen edge).
                            float y0 = 0, y1 = 0, pitch = 0, h = 1.6f;
                            int n = 0;
                            if (std::sscanf(sw, "%f,%f,%d,%f,%f", &y0, &y1, &n, &pitch, &h) < 3 || n < 1) die("bad FUEL_SWEEP");
                            fuel::OrbitCamera c = v.cam;
                            c.explicitPose = true;
                            c.roll = 0;
                            c.eye = v.vehicle + simd_make_float3(0, h, 0);
                            for (int k = 0; k < n; ++k) {
                                const float yaw = (y0 + (y1 - y0) * float(k) / float(std::max(1, n - 1))) * float(M_PI) / 180.0f;
                                const float p = pitch * float(M_PI) / 180.0f;
                                c.target = c.eye + simd_make_float3(std::sin(yaw) * std::cos(p), std::sin(p), std::cos(yaw) * std::cos(p)) * 10.0f;
                                char suffix[16];
                                std::snprintf(suffix, sizeof suffix, "_%04d", k);
                                frame.shot = (fs::path(opt.captureAll.empty() ? fs::path(opt.shot).parent_path() : fs::path(opt.captureAll)) /
                                              (sp->name + suffix + ".png")).string();
                                renderShot(device, renderer, frame, c);
                            }
                            std::printf("sweep %s: %d frames\n", sp->name.c_str(), n);
                            continue;
                        }
                        const double gpuMs = renderShot(device, renderer, frame, v.cam);
                        std::printf("wrote %s (%dx%d, gpu %.1f ms)\n", frame.shot.c_str(), frame.width, frame.height, gpuMs);
                    }
                    return 0;
                }
                if (opt.race.empty() && (!opt.hub.empty() || std::getenv("FUEL_TRAFFIC")) && roadGraph && roadTypes && terrain && !opt.hubCheck && !opt.eventCheck &&
                    !opt.propCheck && !opt.collisionCheck)  // free roam (FUEL_TRAFFIC=1: any --drive without a race)
                    gTraffic = addTraffic(*garage, *drive, renderer, *roadGraph, *roadTypes, *terrain, prefs);  // P58
                if (!opt.race.empty()) {
                    race = std::make_unique<RaceSession>();
                    race->terrain = terrain.get();
                    race->roads = roads.get();
                    race->roadTypes = roadTypes;
                    race->graph = roadGraph.get();
                    {
                        std::string err;
                        race->aiTables = fuel::loadAiTables((fs::path(opt.dataDir) / "XFinalComponent/FUEL.exe").string(), &err);
                        if (!race->aiTables) std::printf("AI: %s (no corner speeds)\n", err.c_str());
                    }
                    race->races = loadRaces(opt.dataDir);
                    // ---- PK21: "Edited Race NN" = the career's edited race of that slot (TRY RACE 0x476670 -> commit 0x465be0),
                    // built from career.sav with the road graph (gate headings, start road) and the terrain.
                    if (fuel::editor::slotOfName(opt.race) >= 0) {
                        fuel::Career career(fuel::loadCareerRules(opt.dataDir));
                        if (career.load()) {
                            fuel::EditorWorld w;
                            w.graph = roadGraph.get();
                            if (roadTypes) w.flatHalf = [t = &*roadTypes](int k) { return k >= 0 && k < 21 ? (*t)[size_t(k)].flat : 0.0f; };
                            if (terrain) w.ground = [t = terrain.get()](float x, float z) { return t->smoothHeightAt(x, z); };
                            for (fuel::Event& e : fuel::editor::editedEvents(career.state().editedRaces, w)) race->races.push_back(std::move(e));
                        }
                    }
                    // ---- end PK21
                    for (fuel::Event& e : race->races) snapMissionHeights(e, *terrain);  // PK14: GAMEStarted 0x557e50 -> 0x4f5b90
                    if (race->races.empty()) die("no races found in the mission scripts");
                    auto it = std::find_if(race->races.begin(), race->races.end(), [&](const fuel::Event& e) {
                        return strcasecmp(e.id.c_str(), opt.race.c_str()) == 0 || strcasecmp(e.race.name.c_str(), opt.race.c_str()) == 0;
                    });
                    if (it == race->races.end()) die("no race named " + opt.race);
                    race->index = size_t(it - race->races.begin());
                    {  // [upgrades U4] the opponents' stages from the race's hub tier
                        const int tier = aiUpgradeTier(*race, garage->label(garage->current));
                        std::printf("ai upgrades: %s, hub tier %d (base stage %d)%s, seed override %llu\n", race->event().id.c_str(), tier,
                                    fuel::upg::aiBaseStage(tier), fuel::upg::enabled() ? "" : ": upgrades off, every opponent stock",
                                    (unsigned long long)fuel::upg::aiSeedOverride());
                        addOpponents(*garage, *drive, *race, fuel::eventOpponents(*it, opt.opponents), renderer, prefs, tier);
                    }
                    race->begin(*drive, streamer.get(), renderer);
                    std::printf("race %s: %zu targets, %zu opponents; %zu races (N for the next)\n", race->race().name.c_str(), race->route.size(),
                                race->racers.size() - 1, race->races.size());
                }
                if (opt.retryCheck) {  // PK14
                    if (!race || !streamer) die("--retry-check needs a race and the world");
                    return runRetryCheck(*race, *drive, *streamer, renderer, opt.haveAt, opt.atX, opt.atZ);
                }
                gDamageView.prewarm(*drive, renderer, streamer.get());  // P92: the first crash's loads now, not mid-race
                if (opt.audioCheck) {  // build sound sets for a few vehicles and print their layers
                    std::vector<std::pair<const fuel::DpcFile*, std::string>> list;
                    for (const char* want : {"QUAD_ROAD_2", "BUGGY_ROAD_2", "TRUCK_OFF_1", "BIKE_ROAD_3"})
                        for (const auto& [ai, vi] : garage->entries) {
                            AssemblyMode& am = *garage->archives[ai];
                            if (am.assembler->variants()[vi].label == want) list.push_back({&am.assembler->dpc(), want});
                        }
                    int bad = fuel::audioCheck(list, params);
                    bad += fuel::musicAmbienceCheck(gWorldDpc);
                    bad += fuel::snd::nitroSoundCheck();  // [nitro]
                    for (size_t i = 0; i < garage->entries.size(); ++i) {  // FUEL's sounds on FUEL's physics: a scripted drive
                        AssemblyMode& am = *garage->archives[garage->entries[i].first];
                        const std::string name = am.assembler->variants()[garage->entries[i].second].label;
                        if (name != "QUAD_ROAD_2") continue;
                        auto d = garage->build(i, -1000, -30000, 0);
                        if (!d) continue;
                        std::vector<fuel::VehicleState> states;
                        for (int k = 0; k < 60 * 14; ++k) {  // 6 s full throttle, 3 s lift, 3 s full, 2 s brake
                            const float t = float(k) / 60.0f;
                            const bool gas = t < 6 || (t >= 9 && t < 12);
                            d->update(1.0f / 60, {gas ? 1.0f : 0.0f, t >= 12 ? 1.0f : 0.0f, 0, false});
                            states.push_back(d->state());
                        }
                        bad += fuel::audioDriveCheck(am.assembler->dpc(), name, params.at(name), states);
                    }
                    std::printf("audio check: %zu vehicles, %d problems\n", list.size(), bad);
                    return bad ? 1 : 0;
                }
                if (opt.hapticsCheck) {  // P89: FUEL's vibration rules, then its motors over a scripted drive of the player's vehicle
                    for (size_t i = 0; i < garage->entries.size(); ++i) {
                        AssemblyMode& am = *garage->archives[garage->entries[i].first];
                        const std::string name = am.assembler->variants()[garage->entries[i].second].label;
                        auto d = name == "QUAD_ROAD_2" ? garage->build(i, -1000, -30000, 0) : nullptr;
                        if (!d) continue;
                        std::vector<fuel::VehicleState> states;
                        for (int k = 0; k < 60 * 14; ++k) {  // as --audio-check: 6 s full throttle, 3 s lift, 3 s full, 2 s brake
                            const float t = float(k) / 60.0f;
                            const bool gas = t < 6 || (t >= 9 && t < 12);
                            d->update(1.0f / 60, {gas ? 1.0f : 0.0f, t >= 12 ? 1.0f : 0.0f, 0, false});
                            states.push_back(d->state());
                        }
                        return fuel::hapticsDriveCheck(am.assembler->dpc(), name, params.at(name), states, opt.dataDir) ? 1 : 0;
                    }
                    die("--haptics-check needs --assemble QUAD_ROAD_2 --drive");
                }
                if (opt.triggerCheck) {  // P91: the adaptive triggers over a scripted drive (from the race start with --race)
                    const size_t me = garage->current;
                    auto obstacle = [&](double x, double z, float yaw) {
                        fuel::AssembledVehicle av;
                        fuel::VehicleSetup setup;
                        std::vector<uint32_t> rider;
                        if (!garage->load(me, av, setup, rider)) return false;
                        drive->addVehicle(av, setup, x, z, yaw, rider);
                        return true;
                    };
                    float along = 0;  // follow the race line (CR2-Yell-CP with --race), 25 m ahead
                    auto steer = [&](double x, double z, float heading) {
                        if (!race || race->line.pathLength() <= 0) return 0.0f;
                        along = race->line.project(x, z, std::max(0.0f, along - 20), along + 60);
                        const auto p = race->line.at(along + 25);
                        float err = std::atan2(float(p[0] - x), float(p[2] - z)) - heading;
                        err = std::remainder(err, 2 * float(M_PI));
                        return std::clamp(-2.0f * err, -1.0f, 1.0f);
                    };
                    return fuel::triggerDriveCheck(*drive, garage->label(me), obstacle, steer) ? 1 : 0;
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
                    simd_float3 propAim = {0, 0, 0};  // P55: the prop-check's target (steered at with FUEL_P55_AIM_BREAKABLE)
                    simd_float3 checkFocus = {0, 0, 0};
                    bool haveCheckFocus = false;
                    const simd_float3 checkFrom = drive->position();
                    const float runTime = opt.propCheck ? std::max(opt.driveTime, 9.0f) : opt.driveTime;
                    std::unique_ptr<fuel::PerfCheck> perf;  // P92: --perf-check
                    if (opt.perfCheck) {
                        fuel::PerfCheck::Options po;
                        po.width = opt.width, po.height = opt.height;
                        po.budgetMs = opt.perfBudget;
                        po.breakdown = opt.perfBreakdown;
                        po.inFlight = opt.perfInFlight;
                        po.csv = opt.perfCsv;
                        po.paceHz = opt.perfPace;  // P93
                        po.alternate = opt.perfAlternate;
                        char lbl[128];
                        const double tod = renderer.timeOfDay();
                        std::snprintf(lbl, sizeof lbl, "%s %s, %02d:%02d", opt.race.empty() ? "free drive" : opt.race.c_str(), opt.assemble.c_str(),
                                      int(tod / 3600) % 24, int(tod / 60) % 60);
                        po.label = lbl;
                        perf = std::make_unique<fuel::PerfCheck>(device, po);
                    }
                    std::unique_ptr<fuel::PopCheck> pop;  // P94: --pop-check
                    if (opt.popCheck) {
                        fuel::PopCheck::Options po;
                        po.width = opt.width, po.height = opt.height;
                        po.minPixels = opt.popPixels;
                        po.csv = opt.popCsv;
                        po.label = (opt.race.empty() ? std::string("free drive") : opt.race) + " " + opt.assemble;
                        pop = std::make_unique<fuel::PopCheck>(po);
                    }
                    // PK15: --pop-check's teleport case: 15 s before the end the player is sent back to where it drove 30 s
                    // earlier (its heading then) through the teleport machine (logo, regeneration, idle wait, landing); the
                    // frames the logo covers are not compared and the pops after the reveal are reported apart.
                    // FUEL_POP_TELEPORT=<s>: at that time instead (>= 31 s), 0: no teleport.
                    fuel::ui::LogoTransition popLogo;
                    TeleportSettle popTlp;
                    int popTlpFrame = -1;
                    long popTlpBefore = -1;
                    double popTlpReveal = -1;
                    simd_float3 popTlpTo{};
                    float popTlpYaw = 0;
                    if (pop && streamer) {
                        const char* e = std::getenv("FUEL_POP_TELEPORT");
                        const double at = e ? std::atof(e) : opt.driveTime - 15.0;
                        if (at >= 31.0) popTlpFrame = int(at * 60);
                    }
                    // P92: the simulation's CPU time per phase of the frame (--perf-check), lap by lap.
                    struct PerfLaps {
                        double last = 0;
                        std::array<double, fuel::PerfCheck::kSimParts> ms{};
                        void start() { last = CACurrentMediaTime(), ms.fill(0); }
                        void lap(int k) {
                            const double now = CACurrentMediaTime();
                            ms[size_t(k)] += (now - last) * 1000.0, last = now;
                        }
                    } laps;
                    for (int trial = 0; trial < (opt.propCheck ? 8 : 1); ++trial) {  // --prop-check: try props until one is hit cleanly
                    if (streamer && opt.propCheck) {  // aim the vehicle at a prop from 45 m away and keep the throttle down
                        simd_float3 target;
                        // P55: FUEL_P55_AIM_BREAKABLE=1 aims at the nearest breakable decor object instead (break and pieces).
                        if (std::getenv("FUEL_P55_AIM_BREAKABLE") ? !streamer->nearestBreakable(checkFrom.x, checkFrom.z, target, tried)
                                                                  : !streamer->nearestProp(checkFrom, tried, target))
                            die("--prop-check: no movable props near the start (use --race or --at)");
                        tried.push_back(target);
                        propAim = target;
                        const simd_float3 from = checkFrom;
                        const float yaw = std::atan2(target.x - from.x, target.z - from.z);
                        drive->teleport(target.x - 45 * std::sin(yaw), target.z - 45 * std::cos(yaw), yaw);
                        streamer->update(drive->position(), renderer, drive->physics(), true);
                        std::printf("prop check: aiming at a prop at (%.0f, %.0f) from 45 m\n", target.x, target.z);
                    }
                    ChaseCamera chase;  // --film: the window's chase camera and dust, stepped with the physics
                    DustSystem dust;
                    fuel::OrbitCamera chaseView;
                    // P96: FUEL_AUDIO_DUMP=<wav>: the film's sound (player, opponents, ambience, music, cut-scene sounds)
                    // mixed offline into a WAV, never through an audio device (FUEL_AUDIO_DUMP_PARTS=player,rivals,world,rtc: the
                    // sources mixed, all by default).
                    std::unique_ptr<fuel::DriveAudio> filmAudio;
                    bool filmAudioRtc = true;
                    if (const char* wav = std::getenv("FUEL_AUDIO_DUMP"); wav && opt.film > 0 && drive && garage && !garage->entries.empty()) {
                        filmAudio = std::make_unique<fuel::DriveAudio>();
                        filmAudio->setOffline(wav);
                        filmAudio->setMasterVolume(gGameVolume);
                        // P103: FUEL_AUDIO_SFX=<n 0..16>: the dump at FUEL's SFX slider n (n / 16; default: the settings, 8).
                        if (const char* n = std::getenv("FUEL_AUDIO_SFX")) filmAudio->setMasterVolume(std::clamp(float(std::atof(n)), 0.0f, 16.0f) / 16.0f);
                        const std::string parts = std::getenv("FUEL_AUDIO_DUMP_PARTS") ? std::getenv("FUEL_AUDIO_DUMP_PARTS") : "player,rivals,world,rtc";
                        auto has = [&](const char* p) { return parts.find(p) != std::string::npos; };
                        if (has("player")) {
                            const AssemblyMode& am = *garage->archives[garage->entries[garage->current].first];
                            const auto it = garage->params.find(garage->label(garage->current));
                            if (it != garage->params.end()) filmAudio->setVehicle(am.assembler->dpc(), garage->label(garage->current), it->second);
                        }
                        if (gWorldDpc && has("world")) filmAudio->setWorld(*gWorldDpc);
                        filmAudioRtc = has("rtc");
                        for (size_t k = 1; race && has("rivals") && k < race->racers.size(); ++k) {
                            const AssemblyMode& am = *garage->archives[garage->entries[race->racers[k].entry].first];
                            const auto it = garage->params.find(race->racers[k].name);
                            if (it != garage->params.end()) filmAudio->setRival(k - 1, am.assembler->dpc(), race->racers[k].name, it->second);
                        }
                    }
                    float speedBefore = 0, speedMin = 1e9f, firstContactS = -1, speedLater = -1;
                    const size_t movedBase = streamer ? streamer->propStats().moved : 0;
                    for (int i = 0; i < int(runTime * 60); ++i) @autoreleasepool {  // P92: drained every frame, as the window's run loop does
                        laps.start();  // P92: the frame's simulation time (--perf-check)
                        // P47: --film plays FUEL's cut-scenes like the window: TELEPORT_HELICO at a race start, or
                        // FUEL_RTC=<name> (TR_DEATH, VI_BIKE, TELEPORT_HELICO) from the first frame.
                        bool filmHold = false;
                        RtcView::Frame filmRtc;
                        if (opt.film > 0 && gRtcEnabled && gLibrary) {
                            if (i == 0) {
                                const char* env = std::getenv("FUEL_RTC");
                                const std::string n = env ? env : (race ? "TELEPORT_HELICO" : "");
                                if (garage && !garage->entries.empty()) gRtc.setVehicleMesh("MESH_ID_" + garage->label(garage->current));  // P96: 0x103
                                if (n == "TELEPORT_HELICO" && race) gCutFx.missionStreaming(), gRtc.play(gDataDir, n, startFather(*race), 0x1fb, *gLibrary, renderer);
                                else if (n == "TR_DEATH") gRtc.play(gDataDir, n, cameraFather(chaseView), 0x43, *gLibrary, renderer);
                                else if (!n.empty()) gRtc.play(gDataDir, n, vehicleFather(*drive), 0x40, *gLibrary, renderer);
                                if (n == "VI_BIKE") gCutFx.missionEnded();  // P67: VI_BIKE plays at SetState 6/7 (end grade)
                            }
                            if (streamer) streamer->setRtcRunning(gRtc.active());  // P67: R_RUNNING / HIDEBYRTC
                            if (gRtc.active()) {
                                if (gRtc.name() == "VI_BIKE") gRtc.setFather(vehicleFather(*drive));
                                if (gRtc.name() == "TR_DEATH") gRtc.setFather(cameraFather(chaseView));
                                if (const char* sk = std::getenv("FUEL_RTC_SKIP_AT"); sk && i == int(std::atof(sk) * 60)) gRtc.skip(renderer);  // P67
                                gRtc.setAttachedPose(vehicleFather(*drive));  // PK13
                                filmRtc = gRtc.update(1.0f / 60, renderer, &gCutFx.fader, filmAudio && filmAudioRtc ? filmAudio.get() : nullptr);
                                if (filmRtc.shake) gCutFx.shake.start();
                                if (filmRtc.respawns > 0) drive->reset(), renderer.resetTemporalHistory();  // P76: the loop is let go by 0x10e (TAA: a cut)
                                filmHold = gRtc.name() == "TELEPORT_HELICO";
                                if (filmRtc.ended) gRtc.stop(renderer);
                            }
                        }
                        if (popTlpFrame >= 0) {  // PK15: the pop check's teleport case
                            if (i == popTlpFrame - 30 * 60) popTlpTo = drive->position(), popTlpYaw = drive->heading();
                            if (i == popTlpFrame) {
                                const simd_float3 from = drive->position();
                                popLogo.start();
                                tlpQueue(popTlp, *drive, streamer.get(), renderer, popTlpTo.x, popTlpTo.z, popTlpYaw);
                                // The autopilot's guided ghost stays where the player was: from here the plain test
                                // driver (full throttle along the AI path, no guide forces) drives instead.
                                if (race) race->playerS = race->line.project(popTlpTo.x, popTlpTo.z, 0.0f, race->line.length());
                                std::printf("pop check: teleport at t=%.1f s from (%.0f, %.0f) to (%.0f, %.0f), %.0f m\n", i / 60.0, from.x, from.z, popTlpTo.x,
                                            popTlpTo.z, std::hypot(from.x - popTlpTo.x, from.z - popTlpTo.z));
                            }
                            if (i >= popTlpFrame) {
                                popLogo.update(1.0f / 60, tlpReady(streamer.get(), renderer));
                                tlpStep(popTlp, popLogo.state(), *drive, streamer.get(), renderer, {});
                                if (popLogo.holding()) filmHold = true;
                            }
                        }
                        if (opt.film > 0 && !std::getenv("FUEL_NO_WORLD_ANIM")) {
                            pumpWorldAnimEvents(streamer.get());
                            gWorldAnim.setEye({chaseView.eye.x, chaseView.eye.y, chaseView.eye.z});  // PK29
                            gWorldAnim.advance(1.0f / 60);
                        }
                        laps.lap(0);  // cut-scene, world anims
                        fuel::DriveInput in{1.0f, 0, 0, false};
                        if (opt.propCheck && std::getenv("FUEL_P55_AIM_BREAKABLE")) {  // keep heading at the target
                            const simd_float3 q = drive->position();
                            const auto& r = drive->state().chassis.rotation;  // body forward (as hubcheck's break test)
                            const float fx = 2 * (r[0] * r[2] + r[3] * r[1]), fz = 1 - 2 * (r[0] * r[0] + r[1] * r[1]);
                            float err = std::atan2(propAim.x - q.x, propAim.z - q.z) - std::atan2(fx, fz);
                            while (err > float(M_PI)) err -= 2 * float(M_PI);
                            while (err < -float(M_PI)) err += 2 * float(M_PI);
                            if (simd_distance(q, propAim) > 6) in.steer = std::clamp(err * 2.0f, -1.0f, 1.0f);
                        }
                        if (race && !opt.propCheck) {
                            if (race->state == RaceSession::Finished && !opt.aiCheck) break;
                            in = race->autopilot(*drive);
                            // P97: --ai-player stop (the player stays on the grid) or launch (full throttle from GO, steering
                            // for the AI path 25 m ahead, no guide): what a player sees of the opponents at the start.
                            if (!opt.aiPlayer.empty() && race->state == RaceSession::Running) in = race->testPlayer(*drive, opt.aiPlayer);
                            if (popTlpFrame >= 0 && i >= popTlpFrame && race->state == RaceSession::Running) in = race->testPlayer(*drive, "launch");  // PK15
                            if (std::getenv("FUEL_RACE_WARP") && race->state == RaceSession::Running && i % 60 == 0) {
                                // Logic test: hop to the next target every second.
                                std::array<float, 3> t;
                                if (race->warpTarget(*drive, t)) drive->teleport(t[0], t[2], drive->heading());
                            }
                            if (!std::getenv("FUEL_AI_NOVIEW")) {
                                // P97: the game camera (headless: FUEL's chase camera, 16:9) for the AI ranks (0x5d3300).
                                static ChaseCamera rankCam;
                                race->view = rankCam.update(1.0f / 60, *drive, {}, cameraOverride(race.get()));
                            }
                            if (!filmHold && !filmRtc.pauseGame) race->update(1.0f / 60, *drive, in);  // PK13: 0x633190 skipped while 0xa65878
                            if (std::getenv("FUEL_RACE_TRACE") && i % 600 == 0) {
                                const simd_float3 p = drive->position();
                                std::printf("  t=%5.0fs at (%.0f, %.0f, %.0f) %5.1f km/h  heading %4.0f  %s\n", i / 60.0, p.x, p.y, p.z,
                                            drive->state().speedKmh, drive->heading() * 180 / M_PI, race->hud(*drive).c_str());
                            }
                            if (opt.aiCheck && i % 600 == 0) printAiCheck(*race, *drive);
                            if (opt.aiCheck && race->state == RaceSession::Running) {  // P82: the gaps at 5/10/20/30 s
                                static const double marks[] = {5, 10, 20, 30};
                                for (double m : marks)
                                    if (race->clock >= m && race->clock - 1.0 / 60 < m) {
                                        const float l = printAiGaps(*race, *drive);
                                        if (m == 5) aiGapLead5 = l;
                                        if (m == 10) aiGapLead10 = l;
                                    }
                                static const double every = std::getenv("FUEL_AI_GAPS_EVERY") ? std::atof(std::getenv("FUEL_AI_GAPS_EVERY")) : 0;
                                static const double until = std::getenv("FUEL_AI_GAPS_UNTIL") ? std::atof(std::getenv("FUEL_AI_GAPS_UNTIL")) : 10;
                                if (every > 0 && race->clock < until && std::fmod(race->clock, every) < 1.0 / 60) printAiGaps(*race, *drive);
                                if (race->clock < 10) aiAir.step(*race, *drive, 1.0f / 60);
                            }
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
                        if (race && race->state == RaceSession::Finished) drive->endingRestore();  // PK14: 0x632570
                        laps.lap(1);  // race rules, AI
                        const double tp0 = CACurrentMediaTime();
                        if (streamer && !filmHold) streamer->breakThrough(*drive, 1.0f / 60, renderer, drive->physics());  // P50: 0x51edb0
                        if (gTraffic && !filmHold) {  // P58: the camera of the last frame (the vehicle's heading without --film)
                            const float h = drive->heading();
                            const bool cam = opt.film > 0 && simd_length_squared(chaseView.eye) > 0;  // the chase camera exists from the first update on
                            const simd_float3 eye = cam ? chaseView.eye : drive->position();
                            const simd_float3 fwd = cam ? chaseView.target - chaseView.eye : simd_make_float3(std::sin(h), 0, std::cos(h));
                            gTraffic->step(1.0f / 60, *drive, eye, fwd);
                            if (std::getenv("FUEL_TRAFFIC_TRACE") && i % 300 == 0)
                            {
                                std::printf("traffic t=%.0f s: %d loops, %d vehicles near (%.0f, %.0f)", i / 60.0, gTraffic->traffic().loopsFound(), gTraffic->active(), eye.x, eye.z);
                                for (const auto& v : gTraffic->traffic().vehicles())
                                    if (v.active) std::printf("  %s (%.0f, %.0f, %.0f)", v.model == 2 ? "SPECIAL" : v.model ? "truck" : "jeep", v.pos[0], v.pos[1], v.pos[2]);
                                std::printf("\n");
                            }
                        }
                        if (const char* nh = std::getenv("FUEL_NITRO_HOLD")) in.nitro = in.nitro || fuel::nitro::scriptedHold(nh, i / 60.0);  // [nitro] films, checks
                        if (!filmHold && !filmRtc.pauseGame) drive->update(1.0f / 60, in);  // PK13: the 0x10e freeze (0x680f90 -> vf106)
                        if ((opt.aiCheck || opt.propCheck) && opt.film <= 0) gDamageView.step(1.0f / 60, *drive, streamer.get(), renderer);  // P87
                        if (opt.aiCheck || opt.propCheck)
                            gDebrisCheck.step(i / 60.0, race ? race->clock < 10.0 : i < 600, *drive, streamer.get(), 1.0f / 60);
                        const double tp1 = CACurrentMediaTime();
                        laps.lap(2);  // physics (props' break-through, traffic, vehicles)
                        {  // physics cost per frame (all vehicles, props, queries), for --ai-check
                            gPhysStats.add((tp1 - tp0) * 1000.0, drive->physics().lastVehicleMs(), drive->physics().lastVehicleCpuMs(), drive->vehicleCount());
                        }
                        if (opt.film > 0) {
                            if (i == 0 && opt.camView > 0) chase.setView(opt.camView);
                            chaseView = chase.update(1.0f / 60, *drive, {}, cameraOverride(race.get()));
                            if (filmRtc.camera) chaseView = filmRtc.cam;
                            // [suspanim] FUEL_FILM_ORBIT=YAW,PITCH,DIST (degrees, metres) with --focus: the film frames look at that
                            // vehicle-space point from a fixed angle in the vehicle's frame (close-ups of the wheels while driving)
                            if (static const char* orb = std::getenv("FUEL_FILM_ORBIT"); orb && opt.haveFocus) {
                                float oy = 0, op = 10, od = 1.5f;
                                std::sscanf(orb, "%f,%f,%f", &oy, &op, &od);
                                const simd_float3 tgt = drive->vehiclePoint(opt.focus);
                                const float yr = oy * float(M_PI) / 180.0f, pr = op * float(M_PI) / 180.0f;
                                const simd_float3 local = simd_make_float3(std::sin(yr) * std::cos(pr), std::sin(pr), std::cos(yr) * std::cos(pr)) * od;
                                chaseView.explicitPose = true;
                                chaseView.eye = drive->vehiclePoint(opt.focus + local) ;
                                chaseView.eye.y = tgt.y + std::sin(pr) * od;
                                chaseView.target = tgt;
                                chaseView.roll = 0, chaseView.zNear = 0.05f;
                            }
                            if (race && race->state == RaceSession::Finished && gCutFx.endGrade.dir == 0 && gCutFx.endGrade.ramp == 0) gCutFx.missionEnded();
                        }
                        laps.lap(3);  // camera
                        if (opt.film > 0) {  // P67: the fader, shake and end grade advance every frame
                            f14Step(1.0f / 60, *drive, renderer);  // PK27
                            F14View::debugLook(chaseView, gF14, drive->position());  // PK27: FUEL_F14_LOOK
                            gCutFx.frame(1.0f / 60, chaseView, filmRtc.camera, renderer, filmAudio.get());
                            if (filmAudio) {  // P96: as the window's frame: listener = the camera, then the vehicles' sounds, then mix
                                filmAudio->setListener({chaseView.eye.x, chaseView.eye.y, chaseView.eye.z}, {chaseView.target.x, chaseView.target.y, chaseView.target.z},
                                                       chase.view());
                                filmAudio->setVehicleSoundsHeld(filmRtc.loading);  // PK13: 0x539a50
                                for (int k = 1; k < drive->vehicleCount(); ++k) filmAudio->updateRival(size_t(k - 1), 1.0f / 60, drive->state(k));
                                filmAudio->setGame(gameSound(*drive, race.get(), {chaseView.eye.x, chaseView.eye.y, chaseView.eye.z}, renderer, streamer.get(), filmHold));
                                fuel::VehicleState sst = drive->state();  // P96: the hanging vehicle's sounds follow the RTC node
                                if (filmRtc.attached) sst.chassis.position = {filmRtc.attach[12], filmRtc.attach[13], filmRtc.attach[14]};
                                filmAudio->update(1.0f / 60, sst, in.throttle);
                                heliSounds(*filmAudio, race.get());  // PK07
                                f14Sounds(*filmAudio, 1.0f / 60, chaseView.eye);  // PK27
                                filmAudio->renderOffline(1.0f / 60);
                            }
                            gHeliView.frame(1.0f / 60, race && race->rules.helicopter ? &race->rules.hel : nullptr, renderer);  // PK07
                            gStorms.debugLook(chaseView, drive->position());
                            if (race && race->rules.helicopter && race->state == RaceSession::Running) HeliView::debugLook(chaseView, race->rules.hel);  // PK07
                            dust.update(1.0f / 60, *drive, chaseView.eye, chaseView.target);
                            gStorms.update(1.0f / 60, chaseView.eye, streamer.get());
                            laps.lap(4);  // cut-scene effects, storms, dust
                            gDamageView.step(1.0f / 60, *drive, streamer.get(), renderer);  // P85
                            laps.lap(5);  // damage, debris
                            if (opt.inGame || perf || pop) {  // the HUD's smoothed state (GPS trail and ribbons) advances every frame
                                fuel::ui::HudState step;
                                fillHud(step, *drive, race.get(), 1.0f / 60, chaseView);
                            }
                            laps.lap(6);  // HUD state
                        }
                        if (streamer && (opt.propCheck || opt.film > 0 || i % 30 == 0)) streamer->update(drive->position(), renderer, drive->physics());
                        laps.lap(7);  // streaming
                        if (streamer && std::getenv("FUEL_IMPACT_TRACE"))  // P61: the props' impact sounds (vt[139]), headless
                            for (const auto& o : streamer->updateImpacts(1.0f / 60, drive->physics()))
                                std::printf("impact sound %02x of %08x at (%.1f, %.1f, %.1f) pitch %.2f, %.0f..%.0f m\n", o.symbol, o.lodName, o.pos[0], o.pos[1],
                                            o.pos[2], o.pitch, o.minDist, o.maxDist);
                        if (opt.inGame && opt.film > 0) {  // P44: the free-roam rules run every frame of an --in-game film
                            if (!gOverlay)
                                gOverlay = std::make_unique<fuel::ui::GameOverlay>(device, kColorFormat, argv[0], opt.dataDir, fuel::loadSettings(), opt.race,
                                                                                   opt.hub, opt.assemble);
                            if (!gFreeRoam) {
                                gFreeRoam = std::make_unique<FreeRoamView>(gOverlay->menu(), opt.race, opt.hub, opt.assemble);
            if (gTraffic) gFreeRoam->attachTraffic(*gTraffic);  // P61: special vehicles and catches
                                if (gTraffic) gFreeRoam->attachTraffic(*gTraffic);  // P61: special vehicles and catches
                            }
                            if (gFreeRoam->step(1.0f / 60, *drive, streamer.get(), race != nullptr, chaseView) && streamer)
                                streamer->update(drive->position(), renderer, drive->physics(), true);
                        }
                        if (opt.film > 0 && !perf && !pop) {  // worst step times since the last frame
                            static double worstPhys = 0, worstStream = 0;
                            worstPhys = std::max(worstPhys, (tp1 - tp0) * 1000.0);
                            worstStream = std::max(worstStream, (CACurrentMediaTime() - tp1) * 1000.0);
                            if (i % std::max(1, int(opt.film * 60)) == 0) {
                                std::printf("film timing: worst physics step %.1f ms, worst streaming %.1f ms\n", worstPhys, worstStream);
                                worstPhys = worstStream = 0;
                            }
                        }
                        // TAA: with the temporal upscaler a film also renders the frames just before each film frame (not written),
                        // so the frame has the history it would have in the game (default 24). FUEL_FILM_TAA_WARM=N sets how many and
                        // renders them with any upscaler (rendered frames advance the render-side clocks: same frames, same film).
                        const int filmPeriod = std::max(1, int(opt.film * 60));
                        const bool filmShot = opt.film > 0 && i % filmPeriod == 0;
                        static const char* warmEnv = std::getenv("FUEL_FILM_TAA_WARM");
                        static const int taaWarm = warmEnv ? std::max(0, std::atoi(warmEnv)) : 24;
                        const bool filmWarm = opt.film > 0 && !filmShot && !perf && !pop &&
                                              (warmEnv != nullptr || renderer.upscaler == fuel::MeshRenderer::Upscaler::MetalFXTemporal) &&
                                              filmPeriod - i % filmPeriod <= taaWarm;
                        if (filmShot || filmWarm) {  // --film: OUT_0001.png ... from the chase camera
                            if (!std::getenv("FUEL_NO_WORLD_ANIM")) gWorldAnim.upload(renderer, &chaseView);  // P92: only what can show
                            if (gRtc.active() && !perf && !pop && filmShot)
                                std::printf("film frame %d: RTC %s t %.2f%s eye (%.1f, %.1f, %.1f) target (%.1f, %.1f, %.1f) fov %.2f\n",
                                            i / std::max(1, int(opt.film * 60)), gRtc.name().c_str(), gRtc.time(), filmRtc.camera ? " (RTC camera)" : "",
                                            filmRtc.cam.eye.x, filmRtc.cam.eye.y, filmRtc.cam.eye.z, filmRtc.cam.target.x, filmRtc.cam.target.y,
                                            filmRtc.cam.target.z, filmRtc.cam.fovH43);
                            if (perf) laps.lap(8);  // world anims upload
                            applyVehicleLods(*drive, renderer, chaseView.eye);  // P104
                            drive->uploadRiders();
                            if (perf) laps.lap(9);  // riders (CPU skinning)
                            if (filmRtc.attached) {  // P67: 0xf1
                                auto parts = drive->partWorlds();
                                attachParts(parts, vehicleFather(*drive), filmRtc.attach);
                                renderer.setInstanceWorlds(parts);
                            } else {
                                renderer.setInstanceWorlds(drive->partWorlds());
                            }
                            showOpponents(*drive, renderer);
                            if (race) renderer.setMarkers(race->markers());
                            stepRaceMarkers(race.get(), drive->position(), renderer);  // P69
                            if (perf) laps.lap(10);  // vehicles, opponents, markers
                            renderer.setFocus(chase.focus());
                            renderer.speedBlur = chase.speedEffect();
                            renderer.speedBlurAbsolute = chase.pose().blur;
                            if (filmRtc.camera) renderer.speedBlur = 0, renderer.speedBlurAbsolute = gCutFx.rtcBlur();
                            if (filmRtc.clearRadialBlur) renderer.speedBlur = renderer.speedBlurAbsolute = 0;  // PK13: 0x53b6b0
                            if (gOverlay) gOverlay->setRtcLoading(filmRtc.loading);                            // PK13: 0x539a50
                            if (std::getenv("FUEL_NO_SPEED_BLUR")) renderer.speedBlur = renderer.speedBlurAbsolute = 0;  // stills for comparisons
                            renderer.setDust(gStorms.withSprites(dust.particles(), chaseView.eye));
                            renderer.setLensVehicle(dust.lens());  // PK24
                            renderer.setTornadoes(gStorms.draws(chaseView.eye));  // P65
                            gStorms.hookWeather(renderer);
                            updatePlacedFx(1.0f / 60, chaseView.eye, renderer);
                            renderer.gameClock = i / 60.0;            // the time of day runs with the race clock
                            renderer.exposureDt = std::max(opt.film, 0.0f);  // the eye adapts over the time between frames
                            if (pop) {  // P94: this frame's draw decisions, compared with the last frame's
                                // PK15: the teleport's logo covers the world from the queue until its picture opens (q < 0.7).
                                if (popTlpFrame >= 0 && i >= popTlpFrame && popLogo.active() && (popLogo.holding() || popLogo.q() >= 0.7f)) {
                                    pop->covered();
                                } else {
                                    if (popTlpFrame >= 0 && i >= popTlpFrame && popTlpReveal < 0) {
                                        popTlpReveal = i / 60.0, popTlpBefore = pop->portPops();
                                        const simd_float3 p = drive->position();
                                        std::printf("pop check: teleport revealed at t=%.2f s, player at (%.0f, %.1f, %.0f)\n", popTlpReveal, p.x, p.y, p.z);
                                    }
                                    pop->frame(renderer, chaseView, i / 60.0);
                                }
                                continue;
                            }
                            if (perf) {  // P92: this frame offscreen, timed (no picture written)
                                laps.lap(11);  // effect lists (dust, storms, placed effects)
                                fuel::PerfCheck::Sim sim;
                                sim.part = laps.ms;
                                sim.t = i / 60.0;
                                renderer.gpsRibbons = gGpsDraw;  // P86
                                renderer.navBeams = gNavBeams;   // PK19
                                perf->frame(renderer, chaseView, sim);
                                continue;
                            }
                            Options frame = opt;
                            const fs::path out(opt.shot);
                            char suffix[16];
                            std::snprintf(suffix, sizeof suffix, "_%04d", i / std::max(1, int(opt.film * 60)));
                            frame.shot = (out.parent_path() / (out.stem().string() + suffix + out.extension().string())).string();
                            if (filmWarm) frame.shot.clear();  // TAA: a history frame, not written
                            if (opt.inGame) {  // --in-game --film: the HUD (and, with FUEL_MENU_FILM=1, the pause menu) on the frames
                                if (!gOverlay)
                                    gOverlay = std::make_unique<fuel::ui::GameOverlay>(device, kColorFormat, argv[0], opt.dataDir, fuel::loadSettings(), opt.race,
                                                                                       opt.hub, opt.assemble);
                                gOverlay->menu().setRoads(roadGraph.get(), roadTypes ? &*roadTypes : nullptr), gOverlay->menu().setHubZones(&gHubZones);  // minimap roads (p40), zones (P75)
                                if (terrain) gOverlay->menu().setGround([t = terrain.get()](float x, float z) { return t->smoothHeightAt(x, z); });  // PK19: the beams' ground (0x445500)
                                if (const char* mf = std::getenv("FUEL_MENU_FILM"); mf && !gOverlay->open()) std::string(mf) == "map" ? gOverlay->openInfos() : gOverlay->openPause();  // P75: =map: the Map tab
                                if (std::getenv("FUEL_MENU_FILM")) {  // PK17: the pause backdrop on the film frames (the overlay is not update()d here)
                                    gOverlay->stepBackdrop(std::max(opt.film, 1.0f / 60));  // the time between shots
                                    pauseBackdropEvents(renderer, nullptr, drive->position());
                                    pauseBackdropTint(renderer);
                                }
                                gShotHud = {};
                                gShotHud.speedKmh = std::fabs(drive->state().speedKmh);
                                gShotHud.odometerKm = float(i) / 60.0f * gShotHud.speedKmh / 3600.0f;
                                fillHud(gShotHud, *drive, race.get(), 0.0f, chaseView);  // minimap-fix: the banner's zone hub (was the nearest hub)
                                gShotHud.race = race != nullptr;
                                gShotHud.settle = true;  // film: the minimap zoom at its target
                            }
                            const double gpuMs = renderShot(device, renderer, frame, chaseView);
                            if (!filmWarm) {  // TAA: history frames print nothing
                            const fuel::VehicleState fs = drive->state();
                            const simd_float3 fp = drive->position();
                            std::printf("film %s t=%.1f (%.0f, %.0f, %.0f) %.0f km/h %d wheels down, %zu dust, gpu %.1f ms%s\n", frame.shot.c_str(), i / 60.0, fp.x, fp.y, fp.z,
                                        fs.speedKmh, fs.wheelsOnGround, dust.particles().sprites.size(), gpuMs, race ? (" " + race->hud(*drive)).c_str() : "");
                            const auto& cp = chase.pose();
                            std::printf("  camera view %d eye (%.1f, %.1f, %.1f) target (%.1f, %.1f, %.1f) %.1f m, fov %.1f, roll %.3f, near %.2f\n", cp.view, cp.ex, cp.ey,
                                        cp.ez, cp.tx, cp.ty, cp.tz, std::hypot(std::hypot(cp.ex - fp.x, cp.ey - fp.y), cp.ez - fp.z), cp.fovDeg, cp.roll, cp.zNear);
                            }
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
                        gDebrisCheck.report(*drive);  // P87
                        if (ok || trial == 7) {
                            if (opt.shot == "-" && !ok) return 1;
                            break;
                        }
                    }
                    }
                    if (perf) {  // P92
                        if (race) std::printf("%s\n", race->hud(*drive).c_str());
                        return perf->finish(renderer) ? 0 : 1;
                    }
                    if (pop) {  // P94
                        if (race) std::printf("%s\n", race->hud(*drive).c_str());
                        const bool popOk = pop->finish();
                        bool tlpOk = true;  // PK15: the teleport case revealed a settled world
                        if (popTlpFrame >= 0) {
                            tlpOk = popTlpReveal >= 0 && pop->portPops() == popTlpBefore;
                            if (popTlpReveal < 0) std::printf("pop check: the teleport case never revealed the world: FAILED\n");
                            else std::printf("pop check: teleport case: %ld pops FUEL would not have after the reveal (%.1f s after the queue): %s\n",
                                             pop->portPops() - popTlpBefore, popTlpReveal - popTlpFrame / 60.0, tlpOk ? "ok" : "FAILED");
                        }
                        return popOk && tlpOk ? 0 : 1;
                    }
                    renderer.setInstanceWorlds(drive->partWorlds());
                    showOpponents(*drive, renderer);
                    stepRaceMarkers(race.get(), drive->position(), renderer);  // P69
                    if (race) {
                        renderer.setMarkers(race->markers());
                        std::printf("%s\n", race->hud(*drive).c_str());
                        if (race->racers.size() > 1) std::printf("order after %.1f s of racing:\n%s", std::max(0.0, race->clock), race->standings(*drive).c_str());
                        if (opt.aiCheck) {
                            printAiCheck(*race, *drive);
                            if (opt.aiPlayer == "bot") {  // [upgrades U4] the lap bot's time (the stage lap-time comparison)
                                const fuel::RaceRules::Racer& r0 = race->rules.racers[0];
                                std::printf("lap bot: %s stages %s: %s %.2f s, %zu/%zu gates, %d respawns (full throttle %.0f s, braking %.0f s, airborne %.0f s)\n",
                                            race->racers[0].name.c_str(), fuel::upg::formatStages(fuel::upg::playerStages()).c_str(),
                                            r0.finished >= 0 ? "finished in" : "NOT FINISHED after", r0.finished >= 0 ? r0.finished : race->clock, r0.next,
                                            race->rules.lineFor(0).size(), drive->physics().respawns(0), race->botFull, race->botBrake, race->botAir);
                            }
                            gPhysStats.print();
                            const bool debrisOk = gDebrisCheck.report(*drive);
                            return aiCheckVerdict(*race, *drive, opt.aiPlayer == "stop") && debrisOk ? 0 : 1;
                        }
                    }
                    // ---- PK31: FUEL_SHOT_RACER=N frames opponent N instead (rider lean / gestures), and prints its lean
                    int shotRacer = 0;
                    if (const char* sr = std::getenv("FUEL_SHOT_RACER"); sr && race && std::atoi(sr) > 0 && std::atoi(sr) < drive->vehicleCount()) {
                        shotRacer = std::atoi(sr);
                        const auto& L = race->racers[size_t(shotRacer)].ai.lean();
                        std::printf("racer %d: %s, lean pitch %.2f roll %.2f yaw %.2f deg\n", shotRacer, race->racers[size_t(shotRacer)].ai.physical() ? "simulated" : "kinematic",
                                    L.x[0], L.x[1], L.x[2]);
                    }
                    renderer.setFocus(haveCheckFocus ? checkFocus : drive->position(shotRacer));
                    if (opt.haveFocus && !haveCheckFocus) {  // CARPOLISH: --focus in the vehicle's frame (x left, z forward)
                        renderer.setFocus(drive->vehiclePoint(opt.focus, shotRacer));
                    }
                    cam.yaw += drive->heading(shotRacer) + float(M_PI);
                    // ---- end PK31
                    const fuel::VehicleState st = drive->state();
                    std::printf("after %.1f s: %.1f km/h, gear %d, %d wheels on the ground\n", opt.driveTime, st.speedKmh, st.gear, st.wheelsOnGround);
                }
                if (opt.shot == "-") return 0;
                if (opt.haveFocus && !drive) renderer.setFocus(opt.focus);  // CARPOLISH
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
        // FUEL_RECORD=DIR: record every in-game session the menus start (the menu execs into a new
        // process, so --record would not survive); each session gets DIR/<time>_<race or hub>.
        if (opt.record.empty() && opt.inGame)
            if (const char* base = std::getenv("FUEL_RECORD"); base && *base) {
                char stamp[32];
                const std::time_t t = std::time(nullptr);
                std::strftime(stamp, sizeof stamp, "%Y%m%d_%H%M%S", std::localtime(&t));
                std::string what = !opt.race.empty() ? opt.race : !opt.hub.empty() ? opt.hub : "session";
                for (char& c : what)
                    if (!std::isalnum(static_cast<unsigned char>(c)) && c != '-') c = '_';
                fs::create_directories(base);
                opt.record = std::string(base) + "/" + stamp + "_" + what;
            }
        if (!opt.record.empty()) {
            if (!gRecorder.open(opt.record)) die("cannot write to " + opt.record);
            if (std::FILE* f = std::fopen((opt.record + "/command.txt").c_str(), "w")) {
                for (int i = 0; i < argc; ++i) std::fprintf(f, "%s%s", i ? " " : "", argv[i]);
                std::fprintf(f, "\n");
                std::fclose(f);
            }
            std::printf("recording to %s (press K to mark a glitch)\n", opt.record.c_str());
        }
        if (drive) {  // P47: FUEL's control mapping from the player's input.PC and DftMap.map, settings.txt overrides
            std::string ierr;
            gInputLoaded = gInput.load(opt.dataDir, fuel::loadSettings().all, &ierr);
            if (!gInputLoaded) std::fprintf(stderr, "fuelview: controls: %s (the port's keys stay)\n", ierr.c_str());
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
        if (opt.inGame) window.acceptsMouseMovedEvents = YES;  // pad-ux: the pause menu's hover (FuelView mouseMoved)
        if (opt.inGame && drive) {
            gOverlay = std::make_unique<fuel::ui::GameOverlay>(device, kColorFormat, argv[0], opt.dataDir, playerSettings, opt.race, opt.hub, opt.assemble);
            gOverlay->menu().setRoads(roadGraph.get(), roadTypes ? &*roadTypes : nullptr), gOverlay->menu().setHubZones(&gHubZones);  // minimap roads (p40), zones (P75)
            if (terrain) gOverlay->menu().setGround([t = terrain.get()](float x, float z) { return t->smoothHeightAt(x, z); });  // PK21: edited points
            gFreeRoam = std::make_unique<FreeRoamView>(gOverlay->menu(), opt.race, opt.hub, opt.assemble);
            gFreeRoam->setZones(&gHubZones);  // PK03
            // PK03: SetDayClock 0x46ab10 on a first hub arrival (0x481e40, 43200 s = 08:00): PK04's game clock.
            gFreeRoam->setDayClock = [r = &renderer](double t) { r->setDayClock(t), std::printf("hub: SetDayClock %.0f s\n", t); };
            // ---- PK26: the career boot runs Dialog_Loading's order instead: after its 0.5 s, TeleportToHub through the
            // teleport queue (PK15), StartHubHUD at the landing, then GAMEStarted (driveStep).
            if (gHubTravel && opt.careerBoot) {
                if (const fuel::Hub* h = fuel::findHub(gOverlay->menu().hubs(), opt.hub)) gBootHub = int(h - gOverlay->menu().hubs().data());
                gOverlay->startCareerBoot();
                std::printf("boot: _hub%02d (start-base suffix; no Start_Base node in the world, no effect)\n", gBootHub + 1);
            } else
            // ---- end PK26
            if (gHubTravel) gFreeRoam->startSessionHub(opt.hub);                        // PK03: StartHubHUD after TeleportToHub
            for (NSView* sub in view.subviews)
                if ([sub isKindOfClass:[NSTextField class]]) sub.hidden = YES;  // the game's HUD pages replace the text line
        }
        [window center];
        [window makeKeyAndOrderFront:nil];
        if (loadingWindow) loadingWindow->close();
        [window makeFirstResponder:view];
        [view showMesh:start];
        [NSApp activateIgnoringOtherApps:YES];
        if (opt.settings) applyWindowSettings(playerSettings, window, view);
        [NSApp run];
    }
    return 0;
}
