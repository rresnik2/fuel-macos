// P61: the tornadoes in the viewer (core/storms.hpp; notes ~/re-work/fuel/notes/p61-storms.md): the agents run each
// frame, the vehicles feel the list (state 2, view 11), props the vortex volumes (0x681540), IT_ANIMATED objects near
// the small tornado start (0x5181f0). P65: each tornado draws FUEL's SpecialEffectsTornado_G (funnel mesh, base smoke,
// cloud tops; core/tornado_fx.hpp, tornado_render.mm) and throws debris props (0x5bd590, WorldStreamer::throwDebris).
#pragma once

#include "core/dpc.hpp"
#include "core/events.hpp"
#include "core/sfx.hpp"
#include "core/storms.hpp"
#include "renderer.h"

#include <simd/simd.h>

#include <functional>
#include <map>
#include <string>
#include <memory>
#include <vector>

class DriveSession;
namespace fuel { class DriveAudio; }
class WorldStreamer;

class StormView {
public:
    // USA1 and the lookup the world resolves ids in; ground = the terrain height (0x5be320 stand-in).
    void init(const fuel::DpcFile& usa1, const std::vector<const fuel::DpcFile*>& lookup, std::function<float(double, double)> ground);
    bool ready() const { return storms_ != nullptr; }
    // The vehicles' tornado query and the props' force field.
    void attach(DriveSession& drive);
    // P65: the forest density for the debris spawner's item pick (0x45dd90).
    void setForest(std::function<float(double, double)> forest);
    // A race begins (mission start: RESET_M_START rewinds the paths; "TOR" weather places the route, 0x631bd0).
    void raceBegin(const fuel::Event& e);
    // PK14: KillMission (0x62baa0): 0x5b96a0 releases every tornado agent (their looks and base emitters go with them;
    // a new spawn fades in, 0x5bcfa0), and the weather override bytes +0x114..+0x11b are cleared (the race wind, +0x118).
    void killMission();
    void update(float dt, simd_float3 eye, WorldStreamer* streamer);
    // The frame's effects (P65: the tornadoes are no longer sprites, see draws()).
    fuel::SfxFrame withSprites(const fuel::SfxFrame& in, simd_float3 eye) const;
    // P65: the race weather's wind (AddMissionParam W#, lerped by 0x632250) on the renderer's weather; call per frame.
    void hookWeather(fuel::MeshRenderer& r);
    // P65: each shown tornado's SpecialEffectsTornado_G (core/tornado_fx.hpp) for the renderer, far to near.
    std::vector<fuel::TornadoDraw> draws(simd_float3 eye) const;
    fuel::storm::Storms* storms() { return storms_.get(); }
    // The nearest tornado position for the chase camera (0x5bcaa0); false when there is none.
    static bool nearestTornado(double x, double z, double out[3]);
    // FUEL_STORM_LOOK=1 (films): turn the camera to the nearest visible tornado from 12 m behind the vehicle.
    void debugLook(fuel::OrbitCamera& cam, simd_float3 player) const;
    // The tornadoes' own sounds while shown: Tornade 0x7e / 0x80 (MotherTornadoManipulator_G vf17 0x4d55d0), small
    // 0x7f / 0x81 (SmallTornadoManipulator_G vf18 0x5bf5f0), node distances 500 / 1000 (vf24 0x5bfcc0 / 0x5bf3b0).
    void sounds(fuel::DriveAudio& audio) const;

private:
    // PK06: the FX emitters of the tornado looks (base smoke 0x5e6a70) and the wind tank (gusts 0x5e5220 / 0x5e4d00): a
    // private effects system (its clock and WELL512), stepped here and merged into the dust frame by withSprites.
    void updateEmitters(float dt, simd_float3 eye);
    void updateGusts(float dt, simd_float3 eye);
    std::unique_ptr<fuel::sfx::System> fx_;
    fuel::sfx::Emitter gust_;
    bool gustMade_ = false;
    float gustTimer_ = 0;  // tank +0x84
    std::map<int, fuel::sfx::Emitter> baseEmitters_;  // look +0x54, by agent id
    std::vector<fuel::SfxSprite> sprites_;
    fuel::MeshRenderer* renderer_ = nullptr;  // for breakNear (set by hookWeather, which runs every frame)
    std::function<float(double, double)> ground_;
    int lastRaceState_ = 0;
    std::unique_ptr<fuel::storm::Storms> storms_;
    std::map<int, std::unique_ptr<fuel::storm::TornadoEffect>> effects_;
    DriveSession* drive_ = nullptr;  // P65: the player for the debris paths and the props' physics
    struct RaceWeather {
        bool active = false;
        std::string start, end;
        std::array<float, 3> from{}, to{};
    } raceWeather_;
    uint32_t yawRng_ = 0x1b873593u;  // by agent id (0x5bcfa0 / 0x5bf4a0 vf00)
    void updateEffects(float dt, simd_float3 eye);
    double clock_ = 0;
};

extern StormView gStorms;
