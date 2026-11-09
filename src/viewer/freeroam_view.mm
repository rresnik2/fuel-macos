// P44 glue: see freeroam_view.h and ~/re-work/fuel/notes/p44-freeroam.md.
#include "freeroam_view.h"
#include "traffic_view.h"

#include "physics/fuel_vehicle.hpp"
#include "core/hub_zones.hpp"
#include "core/road_graph.hpp"
#include "ui/gps_guide.hpp"
#include "ui/hud_math.hpp"

#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <strings.h>

FreeRoamView::FreeRoamView(fuel::ui::Menu& menu, const std::string& event, const std::string& hub, const std::string& vehicle) : menu_(menu) {
    fuel::Career* c = menu_.career();
    if (!c) return;
    const fuel::CareerRules& R = c->rules();
    std::vector<std::pair<std::string, std::array<float, 2>>> starts;
    for (const fuel::Event& e : menu_.events()) starts.push_back({e.id, {e.race.start[0], e.race.start[2]}});
    freeRoam_ = std::make_unique<fuel::FreeRoam>(*c, [starts](const std::string& n, float& x, float& z) {
        // P99: mission names compare without case (0x4f8f90 _stricmp): 7 hub challenges ("Career Race 2" ...) differ in
        // case from their EMD block and were never discovered.
        for (const auto& [id, p] : starts)
            if (strcasecmp(id.c_str(), n.c_str()) == 0) return x = p[0], z = p[1], true;
        return false;
    });
    menu_.setFreeRoam(freeRoam_.get());
    float gx, gz;  // FUEL_GPS_TARGET=X,Z (films): a map GPS target as if set on the map page
    if (const char* t = std::getenv("FUEL_GPS_TARGET"); t && std::sscanf(t, "%f,%f", &gx, &gz) == 2) menu_.setGpsTarget(gx, gz);
    if (const int v = R.vehicleIndex(vehicle); v >= 0) vehicleClass_ = R.vehicles[size_t(v)].cls;
    // The current hub: a mission's hub at its start (0x6310d0), else the hub the session was started in (TeleportToHub).
    int h = -1, l = 0, k = 0;
    if (!event.empty() && c->locate(event, &h, &l, &k)) c->setCurrentHub(h);
    else if (!hub.empty() && R.hubIndex(hub) >= 0) c->setCurrentHub(R.hubIndex(hub));
    // PK03: the hub presence on the menu's hubs (index -> the career's hub uid by name).
    const std::vector<fuel::Hub>& hubs = menu_.hubs();
    auto uid = [c, &hubs](int i) { return i >= 0 && size_t(i) < hubs.size() ? c->rules().hubIndex(hubs[size_t(i)].id) : -1; };
    fuel::HubPresence::Hooks hk;
    hk.unlocked = [c, uid](int i) { return uid(i) >= 0 && c->hubUnlockTest(uid(i)); };
    hk.opened = [c, uid](int i) { return uid(i) >= 0 && c->state().hubs[size_t(uid(i))].opened != 0; };
    hk.arrive = [this, c, uid, &hubs](int i) {
        if (uid(i) < 0) return;
        c->setCurrentHub(uid(i));  // 0x481d70: game+0x33c0 = uid, travel history 0x56ca20
        menu_.requestSave();       // [port] saved at once (FUEL: at the next SaveGameData); the hub change survives a restart (PK25: through SaveManager_Save)
        std::printf("hub: arrived in %s\n", hubs[size_t(i)].id.c_str());
    };
    hk.open = [this, c, uid, &hubs](int i) {
        if (uid(i) < 0) return;
        if (!c->enterHub(uid(i))) std::printf("hub: %s is not unlocked in the career state\n", hubs[size_t(i)].id.c_str());
        menu_.requestSave();  // +0x80: SaveGameData at the menu's close (vf08 0x46e9b0) [port: at once] (PK25: SaveManager_Save)
        std::printf("hub: %s opened (first arrival; the hub's intro movie stays replaced)\n", hubs[size_t(i)].id.c_str());
    };
    hk.setDayClock = [this](double t) {
        if (setDayClock) setDayClock(t);
    };
    presence_ = std::make_unique<fuel::HubPresence>(hubs, std::move(hk));
}

void FreeRoamView::hubTeleported(int hub) {
    if (presence_) presence_->startHubHUD(hub);
}

void FreeRoamView::startSessionHub(const std::string& hub) {
    const fuel::Hub* h = fuel::findHub(menu_.hubs(), hub);
    if (presence_ && h) presence_->startHubHUD(int(h - menu_.hubs().data()));
}

FreeRoamView::~FreeRoamView() { menu_.setFreeRoam(nullptr); }

void FreeRoamView::attachTraffic(TrafficView& traffic) {
    if (!freeRoam_ || !menu_.career()) return;
    fuel::Career& c = *menu_.career();
    const fuel::CareerRules& R = c.rules();
    std::vector<fuel::Traffic::SpecialDef> defs;
    for (size_t h = 0; h < R.hubs.size(); ++h)
        for (size_t k = 0; k < R.hubs[h].traffic.size(); ++k) {
            if (c.trafficCaught(int(h), int(k))) continue;
            const auto& t = R.hubs[h].traffic[k];
            fuel::Traffic::SpecialDef d;
            d.hub = int(h), d.index = int(k);
            d.x0 = std::min(t.x0, t.x1), d.x1 = std::max(t.x0, t.x1), d.z0 = std::min(t.z0, t.z1), d.z1 = std::max(t.z0, t.z1);
            uint32_t hsh = uint32_t(h * 131 + k) * 2654435761u;  // +0x30 = 0x640700 (a random 0..1) [port: a hash]
            hsh ^= hsh >> 15;
            d.frac = float(hsh & 0xffff) / 65536.0f;
            defs.push_back(d);
        }
    traffic.traffic().setSpecials(std::move(defs));
    traffic_ = &traffic;
    fuel::FreeRoam* fr = freeRoam_.get();
    traffic.onCatch = [fr](int hub, int k) { return fr->catchTraffic(hub, k); };
}

void FreeRoamView::fillSpecials(fuel::ui::HudState& h) const {
    h.specials.clear();
    if (!traffic_ || !menu_.career()) return;
    const fuel::CareerRules& R = menu_.career()->rules();
    const auto& defs = traffic_->traffic().specials();
    const auto& cars = traffic_->traffic().vehicles();
    for (size_t i = 0; i < defs.size(); ++i) {
        const auto& d = defs[i];
        if (d.hub < 0 || size_t(d.hub) >= R.hubs.size() || d.index < 0 || size_t(d.index) >= R.hubs[size_t(d.hub)].traffic.size()) continue;
        fuel::ui::HudState::Special s{(d.x0 + d.x1) * 0.5f, (d.z0 + d.z1) * 0.5f, R.hubs[size_t(d.hub)].traffic[size_t(d.index)].kind(), d.hub, d.index};
        for (const auto& v : cars)
            if (v.active && v.special == int(i)) s.x = float(v.pos[0]), s.z = float(v.pos[2]);  // the live vehicle (+0x4a8 / +0x4b0)
        // ---- PK19: the minimap lists only the specials a traffic slot drives (0x5c5690, HudState::Special::live).
        for (const auto& v : cars)
            if (v.active && v.special == int(i)) s.live = true;
        // ---- PK19 end
        h.specials.push_back(s);
    }
}

bool FreeRoamView::step(float dt, DriveSession& d, WorldStreamer* streamer, bool mission, const fuel::OrbitCamera& cam) {
    if (!freeRoam_) return false;
    if (streamer && streamer->barrelGeneration() != barrelGen_) {
        barrelGen_ = streamer->barrelGeneration();
        std::vector<fuel::FuelBarrel> b;
        for (const auto& s : streamer->barrels()) b.push_back({s.x, s.y, s.z, s.amount, s.radius});
        if (std::getenv("FUEL_BARREL_TRACE"))
            for (const auto& s : b) std::printf("barrel %.0f at (%.1f, %.1f, %.1f) r %.2f\n", s.amount, s.x, s.y, s.z, s.radius);
        freeRoam_->setBarrels(std::move(b));
    }
    const fuel::VehicleState st = d.state();
    const simd_float3 at = d.position();
    fuel::FreeRoamFrame f;
    f.dt = dt;
    f.pos = {at.x, at.y, at.z};
    f.vel = st.velocity;
    f.onGround = st.wheelsOnGround > 0;
    if (const fuel::fv::FuelVehicle* m = d.physics().model(0)) {
        f.airtime = m->airtime(), f.heightAbove = m->heightAboveGround(), f.wrecked = m->wrecked();
    }
    f.mission = mission;
    f.gpsShown = menu_.options().gps;
    f.vehicleClass = vehicleClass_;
    // On screen: 0x4c1030 -> 0x4ca890 -> 0x414b00, the band's LEVEL projection (minimap-fix [O asm 0x414b49..0x414c77]: the
    // camera's look vector with y zeroed, depth > 0.1, x = 640 + focal (lateral) / depth on the 1280 canvas), then 0x4c10c0
    // (within 10 S of the sides): the pitch does not matter (the port tested the pitched view frustum).
    const simd_float3 fw = simd_normalize(cam.target - cam.eye);
    const float fl = std::hypot(fw.x, fw.z);
    const float fx = fl > 1e-6f ? fw.x / fl : 0.0f, fz = fl > 1e-6f ? fw.z / fl : 1.0f;
    const float fovY = cam.fovH43 > 0 ? 2.0f * std::atan(std::tan(cam.fovH43 * 0.5f) * 0.75f) : cam.fovY;
    const float focal = 360.0f / std::tan(fovY * 0.5f);
    const simd_float3 eye = cam.eye;
    f.onScreen = [=](float x, float, float z) {
        const float dx = x - eye.x, dz = z - eye.z, depth = dx * fx + dz * fz;
        return depth > 0.1f && fuel::hud::offScreen(640.0f + focal * (dz * fx - dx * fz) / depth) == 0;
    };
    // minimap-fix: 0x4c0260 measures from the camera and takes the hub of the zone holding it (0x4a7f40 -> 0x577500).
    f.haveEye = true;
    f.eye = {eye.x, eye.y, eye.z};
    if (zones_ && !zones_->zones.empty() && menu_.career()) {
        const fuel::CareerRules& R = menu_.career()->rules();
        f.zoneHub = -1;
        if (const int zi = zones_->find(eye.x, eye.z); zi >= 0)
            for (size_t i = 0; i < R.hubs.size(); ++i)
                if (strcasecmp(R.hubs[i].name.c_str(), zones_->zones[size_t(zi)].name.c_str()) == 0) f.zoneHub = int(i);
    }
    // PK09: the vehicles' bodies for the barrel contacts (0x531310): the player's (node +0xd8 0x4) and the AI racers'
    // (0x8), their chassis shapes placed with the model origin pose [port: the drawn pose (bike lean included)].
    const fuel::DriveWorld& W = d.physics();
    for (int i = 0; i < W.vehicleCount(); ++i) {
        const fuel::fv::FuelVehicle* m = W.model(i);
        if (!m) continue;
        const fuel::Pose P = W.state(i).chassis;
        const simd_quatf q = simd_quaternion(P.rotation[0], P.rotation[1], P.rotation[2], P.rotation[3]);
        const simd_float3 o = simd_make_float3(float(P.position[0]), float(P.position[1]), float(P.position[2]));
        std::vector<std::pair<fuel::fv::Vec3, float>> sp;
        std::vector<fuel::fv::ProxyShapes::Box> bx;
        m->modelShapes(sp, bx);
        fuel::ContactBody b;
        b.flags = i == 0 ? 0x4 : 0x8;
        for (const auto& [c, r] : sp) {
            const simd_float3 w = o + simd_act(q, simd_make_float3(c.x, c.y, c.z));
            b.spheres.push_back({w.x, w.y, w.z, r});
        }
        for (const auto& x : bx) {
            fuel::ContactBody::Box ob;
            const simd_float3 w = o + simd_act(q, simd_make_float3(x.c.x, x.c.y, x.c.z));
            ob.c = {w.x, w.y, w.z}, ob.h = {x.half.x, x.half.y, x.half.z};
            for (int a = 0; a < 3; ++a) {
                const simd_float3 ax = simd_act(q, simd_make_float3(a == 0, a == 1, a == 2));
                ob.ax[size_t(a)] = {ax.x, ax.y, ax.z};
            }
            b.boxes.push_back(ob);
        }
        f.bodies.push_back(std::move(b));
    }
    freeRoam_->update(f);
    if (presence_) {  // PK03: hub volumes, MissionStatus +0x374 (0x62f680), then the Map tab's step 0x481b30
        presence_->volumes(at.x, at.y, at.z, mission, zones_ ? zones_->find(at.x, at.z) : -1);
        const fuel::HubPresence::Action a = presence_->update(at.x, at.z, mission);
        if (a.kind != fuel::HubPresence::Action::None) hubAction_ = a;
    }
    float hx = -1, hz = 0;
    fuel::hud::mapHeading(st.chassis.rotation, hx, hz);
    menu_.setPlayer(at.x, at.z, hx, hz);
    const auto picked = freeRoam_->takePicked();
    if (!picked.empty() && streamer) {
        std::vector<std::array<float, 2>> taken;
        for (const auto& b : freeRoam_->barrels())
            if (b.taken) taken.push_back({b.x, b.z});
        streamer->setTakenBarrels(std::move(taken));
    }
    for (int id : freeRoam_->takeTrophies()) std::printf("achievement %d unlocked\n", id);
    if (freeRoam_->takeSaveRequest()) menu_.requestSave();  // console "SaveGameData" (PK25: SaveManager_Save 0x4ebea0, refusal + alerts)
    return !picked.empty();
}

void freeRoamGps(fuel::ui::HudState& h, const DriveSession& d, float dt, const fuel::OrbitCamera& cam, fuel::ui::Menu* menu) {
    static fuel::hud::GpsGuide gps;
    float tx = 0, tz = 0;
    const simd_float3 me = d.position();
    if (!menu || !menu->gpsTarget(tx, tz) || !menu->options().gps) {
        gps.reset();
        return;
    }
    const float dx = tx - me.x, dz = tz - me.z, L = std::hypot(dx, dz);
    if (L < 20.0f) {  // arrival (0x4a2250): MapHUD +0x60 off
        menu->clearGpsTarget();
        gps.reset();
        return;
    }
    const simd_float3 f = simd_normalize(cam.target - cam.eye);
    const simd_float3 rt = simd_normalize(simd_cross(f, simd_make_float3(0, 1, 0)));
    const simd_float3 up = simd_cross(rt, f);
    const float fovY = cam.fovH43 > 0 ? 2.0f * std::atan(std::tan(cam.fovH43 * 0.5f) * 0.75f) : cam.fovY;
    h.eye = {cam.eye.x, cam.eye.y, cam.eye.z}, h.camF = {f.x, f.y, f.z}, h.camR = {rt.x, rt.y, rt.z}, h.camU = {up.x, up.y, up.z};
    h.camFy = cam.explicitPose ? 360.0f / std::tan(fovY * 0.5f) : 0.0f;
    // P75 0x4a4080: the free-roam GPS follows the road router's route (route slot M+0x1a88, filled by 0x4f0080 when
    // the map target M+0x60 is set; drawn record by record: the start record's part from the snapped start point, every
    // record of the list, the end record's part up to the snapped end point) = RoadGraph::request + polyline [O].
    // [port] cost = record length (0x4ae340); the route is requested again when the target changes or the player has
    // moved 50 m from the last request; no route (no road within 1000 m) -> the straight line as before.
    static fuel::RoadGraph::Route route;
    static std::vector<std::array<float, 2>> routeLine;
    static float rqx = 1e30f, rqz = 1e30f, rtx = 1e30f, rtz = 1e30f;
    if (const fuel::RoadGraph* g = menu->roads(); g && (tx != rtx || tz != rtz || std::hypot(me.x - rqx, me.z - rqz) > 50.0f)) {
        rqx = me.x, rqz = me.z, rtx = tx, rtz = tz;
        route = {};
        routeLine.clear();
        if (g->request(route, me.x, me.z, tx, tz, [g](uint32_t r, uint32_t off, uint32_t count, int) { return g->lengthCost(r, off, count); }) && route.valid)
            routeLine = g->polyline(route);
    }
    std::vector<std::array<float, 2>> line;
    if (routeLine.size() >= 2) {
        line.push_back({me.x, me.z});
        line.insert(line.end(), routeLine.begin(), routeLine.end());
        line.push_back({tx, tz});
    } else {
        line = {{me.x, me.z}, {tx, tz}};
    }
    // The 3-D guide's samples every 10 m along that line.
    std::vector<std::array<float, 2>> path;
    std::vector<float> along;
    float acc = 0;
    path.push_back(line[0]), along.push_back(0);
    for (size_t k = 1; k < line.size(); ++k) {
        const float sx = line[k][0] - line[k - 1][0], sz = line[k][1] - line[k - 1][1], sl = std::hypot(sx, sz);
        if (sl <= 0) continue;
        for (float u = 10.0f - std::fmod(acc, 10.0f); u < sl; u += 10.0f) path.push_back({line[k - 1][0] + sx * u / sl, line[k - 1][1] + sz * u / sl}), along.push_back(acc + u);
        acc += sl;
        path.push_back(line[k]), along.push_back(acc);
    }
    (void)L;
    const fuel::Heightfield& terrain = d.terrain();
    gps.update(dt, {me.x, me.y, me.z}, h.eye, h.camF, path, along, [&terrain](float x, float z) { return terrain.smoothHeightAt(x, z); });
    h.ribbons = gps.ribbons();
    h.raceLine = line;  // the GPS route on the minimap and map page (0x4a4080)
}
