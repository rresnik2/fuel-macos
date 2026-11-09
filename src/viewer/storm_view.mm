// P61: tornadoes in the viewer (see storm_view.h).
#include "storm_view.h"

#include "drive.h"
#include "core/sfx.hpp"
#include "core/resources.hpp"
#include "audio/drive_audio.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>

StormView gStorms;

namespace fuel { extern simd_float3 gStormWind; }  // tornado_render.mm: the tank wind vector of the last frame (PK06)

namespace {
// 0x5e2a80: the CPU alpha function sin(pi t), t = age / life (1 when life <= 1e-6).
float sinPiFade(const fuel::sfx::Particle& p) { return p.life > 1e-6f ? std::sin(p.age / p.life * 3.14159265f) : 1.0f; }

// The sprite of a CPU particle (drive.mm's conversion of DustSystem::update): position, atlas rect, colour with the
// drawn alpha, size.
fuel::SfxSprite spriteOf(const fuel::sfx::Particle& p, float alpha) {
    fuel::SfxSprite s;
    s.posAngle = simd_make_float4(p.pos[0], p.pos[1], p.pos[2], p.angle);
    s.uv = simd_make_float4(p.uv[0], p.uv[1], p.uv[2], p.uv[3]);
    s.colour = simd_make_float4(p.colour[0], p.colour[1], p.colour[2], alpha);
    s.vel = simd_make_float4(p.vel[0], p.vel[1], p.vel[2], p.drawSize);
    s.acc = simd_make_float4(p.accel[0], p.accel[1], p.accel[2], p.rotSpeed);
    s.life = simd_make_float4(p.spawnTime, p.life, 0.0f, 0.0f);
    s.alphaP = s.scaleP = simd_make_float4(0, 0, 0, 0);
    return s;
}

// 0x5e6a70: SpecialEffectsTornado_G +0x54, the tornado base's CPU emitter. Spawn flags 0x21 (timer, 3 per emission,
// interval 0.01 .. 0.1), life 2 .. 3.4 s, initial angle 0 .. 3, rotation speed 0.5 .. 1.5, +0x6c = 8, motion 3, alpha
// sin(pi t), atlas 0x14, SubCode 0x16, started at the origin. [O/I] It never draws: no size is set (the range stays 0, 0)
// and the end of 0x5e6e20 stops the emitter every frame (vf04 0x4c5680) while nothing restarts it.
fuel::sfx::Emitter makeBaseEmitter() {
    fuel::sfx::Emitter e;
    e.spawnMode = 1, e.perEmission = 3;
    e.interval = {0.055f, 0.01f, 0.1f};
    e.life = {2.7f, 2.0f, 3.4f};
    e.angle = {1.5f, 0.0f, 3.0f};
    e.rotSpeed = {1.0f, 0.5f, 1.5f};
    e.uvFlags = 8, e.killFlags = 1, e.motion = 3, e.atlas = 0x14, e.kind = 0x16;
    e.stopped = true;
    e.start();  // vf+0xc at the end of 0x5e6a70
    return e;
}

// 0x5e5220: the wind tank's CPU emitter (tank +0x80): SubCode 0xc, atlas 0x2f, one particle per emission, motion 0, life
// PP5400..5401, size PP5402..5403, alpha (1,1,1,PP5409) -> (1,1,1,PP5410), rotation speed PP5411, initial angle
// +-PP5415 pi. The spawn timer is the tank's (+0x84); every particle is emitted by hand (0x5e4d00 vf+0x20).
fuel::sfx::Emitter makeGustEmitter(const fuel::sfx::Tables& T) {
    fuel::sfx::Emitter e;
    e.spawnMode = 3, e.perEmission = 1, e.kind = 0xc, e.motion = 0, e.killFlags = 1, e.atlas = 0x2f;
    e.size = {T.P(5402), T.P(5402), T.P(5403)};
    e.life = {T.P(5400), T.P(5400), T.P(5401)};
    e.rotSpeed = {T.P(5411), T.P(5411), T.P(5411)};
    const float a = T.P(5415) * 3.14159265f;
    e.angle = {0.0f, -a, a};
    e.colourA = {1, 1, 1, T.P(5409)}, e.colourB = {1, 1, 1, T.P(5410)};
    e.stopped = true;
    return e;
}
}  // namespace

void StormView::init(const fuel::DpcFile& usa1, const std::vector<const fuel::DpcFile*>& lookup, std::function<float(double, double)> ground) {
    fuel::storm::StormData d = fuel::storm::loadStormData(usa1, lookup);
    if (d.mothers.empty() && d.paths.empty()) return;
    const size_t nm = d.mothers.size(), np = d.paths.size(), nv = d.eventVolumes.size();
    fuel::storm::Storms::World w;
    w.ground = std::move(ground);
    ground_ = w.ground;
    storms_ = std::make_unique<fuel::storm::Storms>(std::move(d), std::move(w));
    // 0x5be290 asks the look's alpha: the fade-out ends when it reaches 0.
    storms_->setLookAlpha([this](int id) {
        auto it = effects_.find(id);
        return it == effects_.end() ? 0.0f : it->second->alpha();
    });
    if (std::getenv("FUEL_STORM_TRACE")) {
        std::printf("storms: %zu mother tornadoes, %zu tornado paths, %zu event volumes\n", nm, np, nv);
        for (size_t i = 0; i < nv; ++i) {
            const auto& v = storms_->eventVolume(i);
            std::printf("storms: event volume %08x centre (%.0f, %.0f, %.0f) half (%.0f, %.0f, %.0f)\n", v.node, v.centre[0], v.centre[1], v.centre[2], v.half[0],
                        v.half[1], v.half[2]);
        }
    }
}

bool StormView::nearestTornado(double x, double z, double out[3]) {
    if (!gStorms.storms_) return false;
    fuel::storm::V3 p;
    if (!gStorms.storms_->nearestPosition(x, z, p)) return false;
    out[0] = p[0], out[1] = p[1], out[2] = p[2];
    return true;
}

void StormView::setForest(std::function<float(double, double)> forest) {
    if (storms_) storms_->world().forest = std::move(forest);
}

void StormView::attach(DriveSession& drive) {
    if (!storms_) return;
    drive_ = &drive;
    fuel::storm::Storms* s = storms_.get();
    drive.physics().setTornadoQuery([s](double x, double z, float& r) { return s->nearestDistance(x, z, &r); });
    // PhysObj_ForceVolumes (0x710e30 -> 0x681540): the body radius +0x408 is taken as 0 [port: the hook has none].
    drive.physics().setPropForceField([s](const std::array<double, 3>& p, float) { return s->forceAt(p, 0.0f); });
}

void StormView::raceBegin(const fuel::Event& e) {
    raceWeather_ = {true, e.weatherStart, e.weatherEnd, e.race.start, e.race.finish};  // P65 (0x631bd0 copies, 0x632250 lerps)
    if (!storms_) return;
    lastRaceState_ = 0;
    storms_->missionStart(e.id);
    storms_->endRace();
    storms_->startRace(e.tornado, e.race.start, e.race.startRot, e.race.finish);
    if (std::getenv("FUEL_STORM_TRACE"))
        std::printf("storms: race %s weather \"%s\"%s\n", e.id.c_str(), e.weatherStart.c_str(), e.tornado ? " -> tornado route" : "");
}

// ---- PK14
void StormView::killMission() {
    raceWeather_.active = false;  // +0x118 (the race's W#) cleared with the other override bytes
    lastRaceState_ = 0;           // Mission::SetState(0)
    if (!storms_) return;
    storms_->killMission();
    effects_.clear();             // the looks (SpecialEffectsTornado_G) leave with their agents
    baseEmitters_.clear();
    if (std::getenv("FUEL_STORM_TRACE")) std::printf("storms: KillMission (0x5b96a0): every tornado agent released\n");
}
// ---- end PK14

void StormView::update(float dt, simd_float3 eye, WorldStreamer* streamer) {
    if (!storms_ || dt <= 0) return;
    clock_ += dt;
    if (drive_) {  // P65: the player for 0x5bd590 (0x421e70 / 0x422630 / 0x421ab0)
        const simd_float3 p = drive_->position();
        const fuel::VehicleState st = drive_->state();
        const float h = drive_->heading();
        storms_->setPlayer({p.x, p.y, p.z}, st.velocity, {std::sin(h), 0.0f, std::cos(h)});
    }
    storms_->update(dt, {eye.x, eye.y, eye.z}, uint64_t(clock_ * 1000.0));
    if (drive_) {  // the race is over: the weather's generator tornado goes (0x5b89e0 -> 0x5be290 fade-out) [I: at the finish]
        const int rs = drive_->raceState();
        if (lastRaceState_ < 6 && rs >= 6) storms_->fadeOutRace();
        lastRaceState_ = rs;
    }
    if (streamer && drive_ && renderer_ && !std::getenv("FUEL_NO_TORNADO_BREAK"))  // 0x516570: the camera < 200 m
        for (const auto& b : storms_->breakSpheres()) streamer->breakNear(b.pos[0], b.pos[1], b.pos[2], b.radius, *renderer_, drive_->physics());
    if (streamer && drive_ && !std::getenv("FUEL_NO_TORNADO_DEBRIS"))
        for (const auto& t : storms_->throws()) {
            yawRng_ = yawRng_ * 1664525u + 1013904223u;  // [I] 0x533d70's yaw (atan2 of a stack vector), random here
            const float yaw = float(yawRng_ >> 8) * (6.2831853f / 16777216.0f);
            const bool ok = streamer->throwDebris(t.item, t.pos, t.vel, yaw, drive_->physics());
            if (std::getenv("FUEL_STORM_TRACE"))
                std::printf("storms: debris %s mode %d at (%.0f, %.0f, %.0f) v (%.1f, %.1f, %.1f)%s\n", t.item, t.mode, t.pos[0], t.pos[1], t.pos[2],
                            t.vel[0], t.vel[1], t.vel[2], ok ? "" : " (not spawned)");
        }
    updateEffects(dt, eye);
    updateEmitters(dt, eye);
    if (streamer)
        for (const auto& p : storms_->animTriggers()) streamer->startAnimatedNear(p[0], p[1], p[2], 50.0f);  // 0x5181f0
    if (std::getenv("FUEL_STORM_TRACE") && int(clock_ * 60) % 120 == 0)
        for (const auto& v : storms_->visibles())
            std::printf("storms: %s tornado at (%.0f, %.0f, %.0f), %.0f m from the camera\n", v.small ? "small" : "mother", v.pos[0], v.pos[1],
                        v.pos[2], std::hypot(v.pos[0] - eye.x, v.pos[2] - eye.z));
}

// P65: the tornado's look is FUEL's SpecialEffectsTornado_G (core/tornado_fx.hpp, drawn by tornado_render.mm); the
// P61 billboard stand-in is gone and the effects frame passes through.
// PK06: the wind-gust and tornado-base emitters' sprites join the frame (the sfx billboard pass draws them with the dust).
fuel::SfxFrame StormView::withSprites(const fuel::SfxFrame& in, simd_float3) const {
    if (sprites_.empty()) return in;
    fuel::SfxFrame out = in;
    out.sprites.insert(out.sprites.end(), sprites_.begin(), sprites_.end());
    return out;
}

void StormView::hookWeather(fuel::MeshRenderer& r) {
    renderer_ = &r;
    if (r.weatherOverride) return;
    r.weatherOverride = [this](fuel::WeatherAmounts& w) {
        if (!raceWeather_.active || !drive_) return;
        const simd_float3 p = drive_->position();
        const float t = fuel::storm::raceWeatherProgress(raceWeather_.from, raceWeather_.to, {p.x, p.y, p.z});
        const std::optional<float> wind = fuel::storm::raceWind(raceWeather_.start, raceWeather_.end, t);
        if (!wind) return;
        // 0x5b89e0 with +0x118: speed = PP5460 x w; the direction stays the field's (0x5b8330 via 0x5b8680).
        const fuel::sfx::Tables* T = DustSystem::tables();
        const float speed = (T ? T->P(5460) : 27.0f) * *wind;
        const float len = std::sqrt(w.wind[0] * w.wind[0] + w.wind[1] * w.wind[1] + w.wind[2] * w.wind[2]);
        std::array<float, 3> dir = len > 1e-4f ? std::array<float, 3>{w.wind[0] / len, w.wind[1] / len, w.wind[2] / len}
                                                : fuel::windDirection(p.x, p.z);
        w.windSpeed = speed;
        w.wind = {dir[0] * speed, dir[1] * speed, dir[2] * speed};
    };
}

void StormView::updateEffects(float dt, simd_float3 eye) {
    const fuel::sfx::Tables* T = DustSystem::tables();
    std::map<int, std::unique_ptr<fuel::storm::TornadoEffect>> kept;
    for (const auto& v : storms_->visibles()) {
        auto it = effects_.find(v.id);
        std::unique_ptr<fuel::storm::TornadoEffect> fx;
        if (it != effects_.end()) {
            fx = std::move(it->second);
        } else {
            // vf00 0x5bcfa0 / 0x5bf4a0: a new SpecialEffectsTornado_G (state 2, fading in) per agent; the look is
            // m_5e6860's pp001 rows (FUEL's defaults when the tables are missing).
            const fuel::storm::TornadoLook look =
                T ? fuel::storm::TornadoLook::load([T](int n) { return T->P(n); }, !v.small) : fuel::storm::TornadoLook{};
            fx = std::make_unique<fuel::storm::TornadoEffect>(look, 0x9e3779b9u ^ uint32_t(v.id * 7919 + 1));
        }
        if (!v.frozen) {  // 0x5bf5f0 returned after the fade call: 0x5e6e20 does not run
            fx->update(dt, v.pos, {eye.x, eye.y, eye.z}, v.fading);  // 0x5e6e20 from the manipulators' vf18; fading = state 1 (0x5be290)
            // 0x5e91d0 (from the mesh build): four ground heights round the first listed tornado within 10 km.
            if (ground_) {
                fuel::storm::V3 pos[32];
                float rad[32];
                const int n = storms_->list(pos, rad);
                for (int k = 0; k < n; ++k) {
                    const double dx = eye.x - pos[k][0], dy = eye.y - pos[k][1], dz = eye.z - pos[k][2];
                    if (!(std::sqrt(dx * dx + dy * dy + dz * dz) < 10000.0)) continue;
                    const double d = double(fx->baseRadius()) * 3.0;  // 0x9dbf40
                    fx->setClipHeights({ground_(pos[k][0], pos[k][2] + d), ground_(pos[k][0] - d, pos[k][2]), ground_(pos[k][0], pos[k][2] - d),
                                        ground_(pos[k][0] + d, pos[k][2])});
                    break;
                }
            }
            // The end of 0x5e6e20: the base emitter follows the tornado (vf+0x14) and is stopped (vf+0x10).
            if (auto e = baseEmitters_.find(v.id); e != baseEmitters_.end()) {
                e->second.setPosition({float(v.pos[0]), float(v.pos[1]), float(v.pos[2])});
                e->second.stop();
            }
        }
        kept.emplace(v.id, std::move(fx));
        if (!baseEmitters_.count(v.id)) baseEmitters_.emplace(v.id, makeBaseEmitter());  // 0x5e6a70 in vf00
    }
    for (auto it = baseEmitters_.begin(); it != baseEmitters_.end();) it = kept.count(it->first) ? std::next(it) : baseEmitters_.erase(it);
    effects_ = std::move(kept);  // [port] hidden agents drop their effect (FUEL hides the node; a new show fades in)
}

// The FX manager's step of the look emitters and the wind tank's gusts (PK06), then the sprites for withSprites.
void StormView::updateEmitters(float dt, simd_float3 eye) {
    const fuel::sfx::Tables* T = DustSystem::tables();
    sprites_.clear();
    if (!T || !T->ok || std::getenv("FUEL_NO_WIND_GUSTS")) return;
    if (!fx_) fx_ = std::make_unique<fuel::sfx::System>(*T);
    const fuel::sfx::V3 e3 = {eye.x, eye.y, eye.z};
    fx_->update(dt, {}, e3);  // the clock
    for (auto& [id, e] : baseEmitters_) fx_->runEmitter(e, dt, e3);
    updateGusts(dt, eye);
    for (const auto& [id, e] : baseEmitters_)
        for (const fuel::sfx::Particle& p : e.live)
            if (p.age > 0) sprites_.push_back(spriteOf(p, sinPiFade(p) * p.colour[3]));
    for (const fuel::sfx::Particle& p : gust_.live)
        if (p.age > 0) sprites_.push_back(spriteOf(p, sinPiFade(p) * p.colour[3]));
}

// 0x5e4d00 (SpecialEffectsWindTank_G, once a frame from the tank's vf05): while the tank's wind is stronger than PP5404
// a timer (tank +0x84) lets off a burst of big soft puffs ahead of the camera that cross the view with the wind.
void StormView::updateGusts(float dt, simd_float3 eye) {
    const fuel::sfx::Tables& T = *DustSystem::tables();
    if (!gustMade_) gust_ = makeGustEmitter(T), gustMade_ = true;
    auto P = [&](int n) { return T.P(n); };
    fuel::sfx::Emitter& g = gust_;
    const fuel::sfx::V3 e3 = {eye.x, eye.y, eye.z};
    const fuel::sfx::V3 wind = {fuel::gStormWind.x, fuel::gStormWind.y, fuel::gStormWind.z};
    const float len = std::sqrt(wind[0] * wind[0] + wind[1] * wind[1] + wind[2] * wind[2]);
    auto finish = [&] {
        g.stopped = true;  // +0x50 = 0: nothing is emitted but by hand
        fx_->runEmitter(g, dt, e3);  // the manager's step: the particles move, die
    };
    if (len <= P(5404)) return finish();
    gustTimer_ -= dt;
    if (gustTimer_ > 0.0f) return finish();
    fuel::sfx::Well512& r = fx_->rng();
    const float w = std::clamp((len - P(5404)) / (P(5460) - P(5404)), 0.0f, 1.0f);  // 0x9dd000 = 27 = PP5460
    const float c = std::clamp(P(5418), 0.0f, 1.0f);
    const float cnt = float(int(P(5417) + (P(5416) - P(5417)) * w));          // ftol: how many at most
    const float size = float(int(P(5403) + (P(5402) - P(5403)) * w));        // ftol
    const float alpha = P(5410) + (P(5409) - P(5410)) * w;
    const float intLo = P(5413) + (P(5412) - P(5413)) * w;
    float speed = P(5419) + (P(5405) - P(5419)) * w;
    gustTimer_ = intLo + (r.uniform()) * ((2.0f - c) * intLo - intLo);
    // The player's velocity (0x421700 -> +0x838) and the reference position (0x421980).
    fuel::sfx::V3 vel{0, 0, 0};
    if (drive_) {
        const auto v = drive_->state().velocity;
        vel = {float(v[0]), float(v[1]), float(v[2])};
    }
    if (P(5406) < speed * len) speed = P(5406) / len;
    const fuel::sfx::V3 push = {wind[0] * speed, wind[1] * speed, wind[2] * speed};
    g.vel.cur = g.vel.lo = g.vel.hi = push;
    g.colourA = {1, 1, 1, (1.0f - c) * alpha}, g.colourB = {1, 1, 1, alpha};
    g.life.lo = P(5400), g.life.hi = P(5401);
    g.size.lo = size * (1.0f - c), g.size.hi = size;
    g.angle.lo = -3.14159265f, g.angle.hi = 3.14159265f;
    g.rotSpeed.lo = g.rotSpeed.hi = P(5411);
    const float lo = cnt * (1.0f - c);
    const int n = int(lo + r.uniform() * (cnt - lo));
    const float half = P(5401) * 0.5f, spread = P(5414);
    if (std::getenv("FUEL_STORM_TRACE"))
        std::printf("storms: gust %d puffs, wind %.1f m/s (w %.2f), size %.0f, alpha %.2f, next in %.2f s\n", n, len, w, size, alpha, gustTimer_);
    const float px = wind[2] / len, pz = -wind[0] / len;  // the horizontal perpendicular
    for (int k = 0; k < n; ++k) {
        const float u = (r.uniform()) * (spread + spread) - spread;
        g.setPosition({e3[0] + vel[0] * P(5407) - push[0] * half + px * u, e3[1] + vel[1] * P(5407) - push[1] * half,
                       e3[2] + vel[2] * P(5407) - push[2] * half + pz * u});
        g.stopped = false, g.burstDone = false;
        fx_->runEmitter(g, 0.0f, e3);  // vf+0x20: one emission (burst mode), the particles stay put
    }
    finish();
}

std::vector<fuel::TornadoDraw> StormView::draws(simd_float3 eye) const {
    std::vector<std::pair<double, fuel::TornadoDraw>> v;
    for (const auto& [id, fx] : effects_) {
        if (!fx->built()) continue;
        fuel::TornadoDraw d;
        d.base = fx->base();
        d.scroll = fx->uvScroll();
        d.funnel = fx->funnel();
        d.smoke = fx->smoke();
        d.clouds = fx->cloudTops();
        const double dist = std::hypot(d.base[0] - eye.x, d.base[2] - eye.z);
        if (dist > 10000.0) continue;  // 0xa2a4e0: the mesh and cloud tops draw within 10 km of the camera
        v.emplace_back(dist, std::move(d));
    }
    std::sort(v.begin(), v.end(), [](const auto& a, const auto& b) { return a.first > b.first; });
    std::vector<fuel::TornadoDraw> out;
    for (auto& [d, t] : v) out.push_back(std::move(t));
    return out;
}

void StormView::debugLook(fuel::OrbitCamera& cam, simd_float3 player) const {
    if (!storms_ || !std::getenv("FUEL_STORM_LOOK")) return;
    double best = 1e30;
    simd_float3 tp{};
    for (const auto& v : storms_->visibles()) {
        const double d = std::hypot(v.pos[0] - player.x, v.pos[2] - player.z);
        // FUEL_STORM_LOOK=H (> 1): look H m above the base instead (P65: the whole funnel).
        const float lookUp = std::atof(std::getenv("FUEL_STORM_LOOK")) > 1 ? float(std::atof(std::getenv("FUEL_STORM_LOOK"))) : (v.small ? 60.0f : 120.0f);
        if (d < best) best = d, tp = simd_make_float3(float(v.pos[0]), float(v.pos[1]) + lookUp, float(v.pos[2]));
    }
    if (best > 1e29) return;
    simd_float3 dir = tp - player;
    dir.y = 0;
    dir = simd_normalize(dir);
    cam.eye = player - dir * 12.0f + simd_make_float3(0, 4, 0);
    if (const char* d = std::getenv("FUEL_STORM_LOOK_DIST")) {  // P65 shots: the eye D m from the tornado, towards the player
        const simd_float3 base = simd_make_float3(tp.x, player.y, tp.z);
        cam.eye = base - dir * float(std::atof(d)) + simd_make_float3(0, 30, 0);
    }
    cam.target = tp;
}

void StormView::sounds(fuel::DriveAudio& audio) const {
    if (!storms_) return;
    static const uint32_t kTornade = fuel::asoboCrc("Tornade"), kSmall = fuel::asoboCrc("SmallTornadoAgent_G");
    bool mother = false, small = false;
    std::array<double, 3> mp{}, sp{};
    for (const auto& v : storms_->visibles()) {
        if (v.small) small = true, sp = v.pos;
        else if (!mother) mother = true, mp = v.pos;
    }
    audio.objectLoop(0x7a0001, kTornade, 0x7e, mp, mother, 500.0f, 1000.0f);
    audio.objectLoop(0x7a0002, kTornade, 0x80, mp, mother, 500.0f, 1000.0f);
    audio.objectLoop(0x7a0003, kSmall, 0x7f, sp, small, 501.0f, 1001.0f);
    audio.objectLoop(0x7a0004, kSmall, 0x81, sp, small, 501.0f, 1001.0f);
}
