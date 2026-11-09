#include "hubcheck.h"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>

namespace {

// Slope of the terrain at (x, z) in degrees, over a 6 m baseline.
float slopeDeg(const fuel::Heightfield& t, double x, double z) {
    const float dx = (t.smoothHeightAt(x + 3, z) - t.smoothHeightAt(x - 3, z)) / 6.0f;
    const float dz = (t.smoothHeightAt(x, z + 3) - t.smoothHeightAt(x, z - 3)) / 6.0f;
    return std::atan(std::hypot(dx, dz)) * 57.29578f;
}

constexpr float kMaxSlope = 35.0f;  // degrees: steeper is a cliff, not a place to start (vista points sit on edges)

}  // namespace

std::string spawnProblem(DriveSession& drive, WorldStreamer& streamer, fuel::MeshRenderer& renderer, const fuel::Heightfield& terrain, float x,
                         float z, float yaw) {
    if (slopeDeg(terrain, x, z) > kMaxSlope) return "cliff";
    drive.teleport(x, z, yaw);
    streamer.update(drive.position(), renderer, drive.physics(), true);
    for (int i = 0; i < 90; ++i) drive.update(1.0f / 60, fuel::DriveInput{});
    const fuel::VehicleState st = drive.state();
    const double ground = terrain.smoothHeightAt(st.chassis.position[0], st.chassis.position[2]);
    const double above = st.chassis.position[1] - ground;
    const auto& q = st.chassis.rotation;
    const float upY = 1 - 2 * (q[0] * q[0] + q[2] * q[2]);
    const int wheels = int(drive.setup().wheels.size());
    if (above < -0.3) return "under the terrain";
    if (above > 3.0) return "in the air";
    if (st.wheelsOnGround < std::min(wheels, 3)) return "wheels off the ground";
    if (upY < 0.8f) return "tipped over";
    if (st.speedKmh > 15.0f || std::hypot(st.chassis.position[0] - x, st.chassis.position[2] - z) > 8.0) return "slides away";
    return "";
}

HubSpawn placeAtHubPoint(DriveSession& drive, WorldStreamer& streamer, fuel::MeshRenderer& renderer, const fuel::Heightfield& terrain,
                         const fuel::HubPoint& p) {
    HubSpawn s;
    s.x = p.x, s.z = p.z, s.yaw = p.yaw;
    s.problem = spawnProblem(drive, streamer, renderer, terrain, p.x, p.z, p.yaw);
    if (s.problem.empty()) return s;
    float x = p.x, z = p.z;
    // spawnProblem leaves the player at the candidate, so a hit ends the search with the player placed.
    s.found = fuel::nudgeToClearSpot(x, z, [&](float cx, float cz) { return spawnProblem(drive, streamer, renderer, terrain, cx, cz, p.yaw).empty(); });
    if (s.found) {
        s.x = x, s.z = z, s.nudged = true, s.moved = std::hypot(x - p.x, z - p.z);
    } else {
        spawnProblem(drive, streamer, renderer, terrain, p.x, p.z, p.yaw);  // use the point anyway
    }
    return s;
}

// ---- PK03: hub presence ------------------------------------------------------------------------------------------
int runHubPresenceCheck(const std::vector<fuel::Hub>& hubs, const fuel::HubZones* zones) {
    int fails = 0, cases = 0;
    auto check = [&](bool ok, const std::string& what) {
        ++cases;
        if (!ok) ++fails, std::printf("  FAIL hub presence: %s\n", what.c_str());
    };
    using A = fuel::HubPresence::Action;
    for (size_t hi = 0; hi < hubs.size(); ++hi) {
        const fuel::Hub& h = hubs[hi];
        const int H = int(hi);
        check(!h.volume.empty() && h.boundary.size() >= 3 && h.hasEnd1, h.id + ": hub record (volume, HUB_BOUNDARIES, END1)");
        if (h.volume.empty() || h.boundary.size() < 3 || !h.hasEnd1) continue;
        const fuel::HubBox& box = h.volume[0];
        const float y = box.centre[1];
        auto inVol = [&](float x, float z) {
            for (const auto& b : h.volume)
                if (b.contains(x, y, z)) return true;
            return false;
        };
        check(fuel::insideBoundary(h, h.camp.x, h.camp.z) && inVol(h.camp.x, h.camp.z), h.id + ": the camp (CAMERA_CAREER) lies in the boundary and the volume");
        check(!fuel::insideBoundary(h, h.end1.x, h.end1.z), h.id + ": END1 lies outside the boundary");
        // Three careers: opened, first arrival, locked.
        for (int kind = 0; kind < 3; ++kind) {
            bool opened = kind == 0, arrived = false, openedNow = false;
            double clock = -1;
            fuel::HubPresence::Hooks hk;
            hk.unlocked = [&](int i) { return i != H || kind != 2; };
            hk.opened = [&](int i) { return i != H || opened; };
            hk.arrive = [&](int i) { arrived = i == H; };
            hk.open = [&](int i) { openedNow = i == H, opened = true; };
            hk.setDayClock = [&](double t) { clock = t; };
            fuel::HubPresence p(hubs, hk);
            // From outside the volume, along END1 -> camp extended, into the camp.
            float dx = h.end1.x - h.camp.x, dz = h.end1.z - h.camp.z;
            const float L = std::hypot(dx, dz);
            dx /= L, dz /= L;
            const float far = 2.5f * std::max({box.half[0], box.half[1], box.half[2]}) + 50.0f;
            const std::string tag = h.id + (kind == 0 ? " (opened)" : kind == 1 ? " (first arrival)" : " (locked)");
            A act;
            int teleports = 0;
            float px = 0, pz = 0;
            for (float d = far; d >= 0; d -= 1.0f) {
                px = h.camp.x + dx * d, pz = h.camp.z + dz * d;
                p.volumes(px, y, pz, false);
                const A a = p.update(px, pz, false);
                if (a.kind == A::TeleportToHub) act = a, ++teleports;
                if (teleports) break;
            }
            check(p.pending() == H || teleports, tag + ": entering the volume makes the hub pending (+0x954)");
            check(teleports == 1 && act.hub == H, tag + ": TeleportToHub once inside the boundary (0x481b60 / 0x481a40)");
            check(!p.request(), tag + ": the request +0x958 is cleared");
            // TeleportToHub: the player on the camp, StartHubHUD.
            p.volumes(h.camp.x, y, h.camp.z, false);
            p.startHubHUD(H);
            check(p.state() == fuel::HubPresence::Wait && p.pending() == H, tag + ": StartHubHUD: state 0xc4, hub pending");
            A a = p.update(h.camp.x, h.camp.z, false);
            if (kind == 2) {
                check(a.kind == A::OpenAlert && p.state() == fuel::HubPresence::Alert && !arrived, tag + ": TT_PAGE_HUB_ALERT (0xc7), the current hub unchanged");
                float tx = 0, tz = 0;
                check(p.end1(tx, tz) && tx == h.end1.x && tz == h.end1.z, tag + ": the alert's OK goes to END1");
                p.menuClosed(false, tx, tz);  // the OK's teleport closes the menu
                p.volumes(h.end1.x, y, h.end1.z, false);
                check(p.pending() < 0 && p.state() == fuel::HubPresence::Idle && p.update(h.end1.x, h.end1.z, false).kind == A::None,
                      tag + ": at END1 nothing is pending (no second TeleportToHub)");
                continue;
            }
            check(arrived, tag + ": 0x481d70 makes it the current hub (game+0x33c0)");
            if (kind == 1) {
                check(a.kind == A::None && p.state() == fuel::HubPresence::Open && clock == 43200.0, tag + ": 0x47d9c0: SetDayClock 43200, state 0xc5, no movie");
                a = p.update(h.camp.x, h.camp.z, false);
                check(a.kind == A::OpenMenu && openedNow, tag + ": 0x481cd0 opens the hub (+0x248) and the menu");
            } else {
                check(a.kind == A::OpenMenu && clock < 0, tag + ": an opened hub opens the menu");
            }
            // The camp menu: opened in pause mode 2; Back -> 0x47f060: TLP to END1.
            p.menuOpened(h.camp.x, h.camp.z, false);
            check(p.pauseMode(h.camp.x, h.camp.z, false) == 2, tag + ": pause mode 2 in the camp (0x46e650)");
            float tx = 0, tz = 0;
            const bool tlp = p.menuClosed(true, tx, tz);
            check(tlp && tx == h.end1.x && tz == h.end1.z && p.pending() < 0 && p.state() == fuel::HubPresence::Idle, tag + ": Back -> END1, the tab reset (0xc3)");
            // At END1 (still in the volume): nothing more; out of the volume and back in: the hub is pending again.
            p.volumes(h.end1.x, y, h.end1.z, false);
            check(p.update(h.end1.x, h.end1.z, false).kind == A::None, tag + ": no TeleportToHub at END1");
            p.volumes(h.camp.x + dx * far, y, h.camp.z + dz * far, false);
            p.volumes(h.end1.x, y, h.end1.z, false);
            check(p.pending() == H && p.request(), tag + ": leave and re-enter: pending again");
            // During a race neither handler runs.
            fuel::HubPresence q(hubs, hk);
            q.volumes(h.camp.x, y, h.camp.z, true);
            check(q.pending() < 0 && q.update(h.camp.x, h.camp.z, true).kind == A::None, tag + ": no hub presence during a race");
        }
    }
    size_t named = 0;
    if (zones)
        for (const auto& z : zones->zones) {
            const bool ok = fuel::findHub(hubs, z.name) != nullptr;
            named += ok;
            check(ok, "zone '" + z.name + "' names a hub (banner 0x4a54b0 / 0x577500)");
        }
    std::printf("hub presence: %zu hubs, %d cases, %zu/%zu zones name a hub, %d failed: %s\n", hubs.size(), cases, named, zones ? zones->zones.size() : size_t(0),
                fails, fails ? "FAILED" : "ok");
    std::fflush(stdout);
    return fails;
}

int runHubCheck(const std::vector<fuel::Hub>& hubs, DriveSession& drive, WorldStreamer& streamer, fuel::MeshRenderer& renderer,
                const fuel::Heightfield& terrain, const fuel::HubZones* zones) {
    const int presenceFails = runHubPresenceCheck(hubs, zones);  // PK03
    if (std::getenv("FUEL_HUB_PRESENCE_ONLY")) return presenceFails ? 1 : 0;  // the presence case alone (no spawns)
    size_t spots = 0, nudged = 0, failed = 0, inCircle = 0;
    for (const fuel::Hub& h : hubs) {
        size_t n[3] = {0, 0, 0}, bad[3] = {0, 0, 0};
        std::string notes;
        auto test = [&](int kind, const fuel::HubPoint& p) {
            const HubSpawn s = placeAtHubPoint(drive, streamer, renderer, terrain, p);
            ++n[kind], ++spots;
            if (streamer.nearObject(p.x, p.z, 0.0f)) ++inCircle;  // information only: bounding circles are generous
            if (s.problem.empty()) return;
            ++bad[kind];
            char line[256];
            if (s.found) {
                ++nudged;
                std::snprintf(line, sizeof line, "    nudged %s (%.0f, %.0f) -> (%.0f, %.0f), %.0f m: %s\n", p.name.c_str(), p.x, p.z, s.x, s.z, s.moved, s.problem.c_str());
            } else {
                ++failed;
                std::snprintf(line, sizeof line, "    NO CLEAR SPOT near %s (%.0f, %.0f): %s\n", p.name.c_str(), p.x, p.z, s.problem.c_str());
            }
            notes += line;
        };
        test(0, h.camp);
        for (const fuel::HubPoint& p : h.teleports) test(1, p);
        for (const fuel::HubPoint& p : h.vistas) test(2, p);
        std::printf("hub %-20s camp %s, teleports %zu/%zu ok, vistas %zu/%zu ok\n", h.id.c_str(), bad[0] ? "NUDGED" : "ok", n[1] - bad[1], n[1], n[2] - bad[2], n[2]);
        std::fputs(notes.c_str(), stdout);
        std::fflush(stdout);
    }
    std::printf("hub check: %zu hubs, %zu spawns (%zu of the points lie inside a placed object's bounding circle), %zu nudged to a clear spot, %zu without one: %s\n", hubs.size(), spots, inCircle, nudged, failed, failed ? "FAILED" : "ok");
    return failed || presenceFails ? 1 : 0;
}

// P50 `fuelview --collision-check`: FUEL's collision world (notes p50-collision-world.md) at a few spots, by class
// (this build against main 1efc828's rule), solids that do not show, and every hub point inside a solid body.
int runCollisionCheck(const std::vector<fuel::Hub>& hubs, DriveSession& drive, WorldStreamer& streamer, fuel::MeshRenderer& renderer,
                      const fuel::Heightfield& terrain, const std::vector<std::array<float, 2>>& spots) {
    std::map<std::string, WorldStreamer::CollisionClassCount> total;
    size_t outside = 0, invisible = 0;
    std::vector<std::array<float, 2>> at = spots;
    if (const char* e = std::getenv("FUEL_P50_SPOTS")) {  // "x,z;x,z": these spots instead, no hub points
        at.clear();
        for (const char* q = e; *q;) {
            float x = 0, z = 0;
            if (std::sscanf(q, "%f,%f", &x, &z) == 2) at.push_back({x, z});
            const char* n = std::strchr(q, ';');
            if (!n) break;
            q = n + 1;
        }
    }
    for (const auto& s : at) {
        drive.teleport(s[0], s[1], 0);
        streamer.update(drive.position(), renderer, drive.physics(), true);
        const WorldStreamer::CollisionStats st = streamer.collisionStats();
        std::printf("collision at (%.0f, %.0f): class now / main\n", s[0], s[1]);
        for (const auto& [k, c] : st.classes) {
            if (!c.now && !c.main) continue;
            std::printf("  %-40s %6zu %6zu\n", k.c_str(), c.now, c.main);
            total[k].now += c.now, total[k].main += c.main;
        }
        std::printf("  solids reaching > 1 m outside their mesh: %zu; not drawn at 10 m: %zu\n", st.outsideMesh, st.invisibleNear);
        for (const std::string& e : st.examples) std::printf("    %s\n", e.c_str());
        outside += st.outsideMesh, invisible += st.invisibleNear;
    }
    std::printf("collision totals over %zu spots: class now / main\n", at.size());
    for (const auto& [k, c] : total) std::printf("  %-40s %6zu %6zu\n", k.c_str(), c.now, c.main);
    std::printf("  outside the mesh %zu, not drawn at 10 m %zu\n", outside, invisible);
    if (std::getenv("FUEL_P50_DYNOBJ")) {  // where the IT_DYNOBJ_G placements are (a --prop-check spot)
        const auto list = streamer.placementsOfClass(fuel::GameClass::DynObj);
        std::printf("IT_DYNOBJ_G placements: %zu\n", list.size());
        for (size_t k = 0; k < list.size(); k += 25) std::printf("  (%.0f, %.0f, %.0f)\n", list[k].x, list[k].y, list[k].z);
        return 0;
    }
    if (std::getenv("FUEL_P50_BREAKTEST")) {  // drive at the nearest breakable from 40 m at full throttle
        std::vector<simd_float3> tried;
        for (size_t trial = 0; trial < at.size() * 15; ++trial) {
            const auto& s = at[trial / 15];
            drive.teleport(s[0], s[1], 0);
            streamer.update(drive.position(), renderer, drive.physics(), true);
            simd_float3 b;
            if (!streamer.nearestBreakable(s[0], s[1], b, tried)) continue;
            tried.push_back(b);
            // A clear run-up: 8 headings, the first whose 150 m line has no other solid within 2.5 m and
            // stays within 1.5 m of the terrain slope (the breakable itself is skipped).
            float yaw = 0, sx = 0, sz = 0;
            bool clear = false;
            for (int h = 0; h < 8 && !clear; ++h) {
                yaw = float(h) * 0.785398f;
                sx = b.x - 150 * std::sin(yaw), sz = b.z - 150 * std::cos(yaw);
                clear = true;
                for (float t = 0; t < 146 && clear; t += 2) {
                    const float px = sx + t * std::sin(yaw), pz = sz + t * std::cos(yaw);
                    if (std::hypot(px - b.x, pz - b.z) < 4) continue;
                    clear = streamer.solidsAt(px, terrain.smoothHeightAt(px, pz) + 0.9, pz, 1.5).empty() && slopeDeg(terrain, px, pz) < 25;
                }
            }
            if (!clear) continue;
            // P55: FUEL_P50_BREAKTEST=fast starts 25 m short of it at 25 m/s (a launch, not a run-up).
            static const bool fast = std::getenv("FUEL_P50_BREAKTEST") && std::string(std::getenv("FUEL_P50_BREAKTEST")) == "fast";
            if (fast) sx = b.x - 25 * std::sin(yaw), sz = b.z - 25 * std::cos(yaw);
            drive.teleport(sx, sz, yaw);
            streamer.update(drive.position(), renderer, drive.physics(), true);
            if (fast) drive.physics().nudge(0, 0.3, {25 * std::sin(yaw), 0, 25 * std::cos(yaw)});
            const size_t before = streamer.brokenCount(), piecesBefore = streamer.piecesSpawned();
            float top = 0, minAfter = 1e9f;
            for (int i = 0; i < 720; ++i) {
                const simd_float3 q = drive.position();
                fuel::DriveInput in;
                in.throttle = 1;
                const float want = std::atan2(b.x - q.x, b.z - q.z);
                const fuel::VehicleState st0 = drive.state();
                const auto& r = st0.chassis.rotation;  // yaw of the body
                const float fx = 2 * (r[0] * r[2] + r[3] * r[1]), fz = 1 - 2 * (r[0] * r[0] + r[1] * r[1]);
                float err = want - std::atan2(fx, fz);
                while (err > 3.14159f) err -= 6.28318f;
                while (err < -3.14159f) err += 6.28318f;
                in.steer = std::clamp(err * 2.0f, -1.0f, 1.0f);
                streamer.breakThrough(drive, 1.0f / 60, renderer, drive.physics());
                drive.update(1.0f / 60, in);
                const fuel::VehicleState st = drive.state();
                if (i > 0 && st0.speedKmh - st.speedKmh > 5.0f) {  // what it hit
                    const auto hits = streamer.solidsAt(st.chassis.position[0], st.chassis.position[1], st.chassis.position[2], 3.0);
                    std::printf("  hit at %.0f km/h -> %.0f km/h:", st0.speedKmh, st.speedKmh);
                    for (size_t k = 0; k < hits.size() && k < 3; ++k) std::printf(" [%s]", hits[k].c_str());
                    std::printf("\n");
                }
                top = std::max(top, st.speedKmh);
                if (streamer.brokenCount() > before) minAfter = std::min(minAfter, st.speedKmh);
                if (i % 30 == 0) streamer.update(drive.position(), renderer, drive.physics());
            }
            const auto ps = streamer.propStats();
            std::printf("break test at (%.0f, %.0f): breakable at (%.0f, %.0f, %.0f), top %.0f km/h, broken %zu, pieces %zu, lowest speed after the break %.0f km/h, end %.0f km/h; props %zu, %zu moved\n", s[0],
                        s[1], b.x, b.y, b.z, top, streamer.brokenCount() - before, streamer.piecesSpawned() - piecesBefore, minAfter < 1e8f ? minAfter : -1.0f,
                        drive.state().speedKmh, ps.tracked, ps.moved);
            if (fast) break;
        }
        return 0;
    }
    if (std::getenv("FUEL_P50_SPOTS")) return 0;
    size_t points = 0, inside = 0, inCircle = 0;
    auto test = [&](const fuel::HubPoint& p) {
        ++points;
        if (streamer.nearObject(p.x, p.z, 0.0f)) ++inCircle;
        drive.teleport(p.x, p.z, p.yaw);
        streamer.update(drive.position(), renderer, drive.physics(), true);
        drive.update(1.0f / 60, fuel::DriveInput{});  // a physics step lets Jolt rebuild its broad phase (aborts after many teleports without)
        const double y = terrain.smoothHeightAt(p.x, p.z) + 0.9;
        std::vector<std::string> hits = streamer.solidsAt(p.x, y, p.z, 1.2);  // a quad-sized sphere at the point
        // P55: dynamic objects (props, pieces) are pushed aside by the vehicle (light: FUEL's masses), not walls.
        hits.erase(std::remove_if(hits.begin(), hits.end(), [](const std::string& h) { return h.rfind("movable prop", 0) == 0 || h.rfind("piece", 0) == 0; }),
                   hits.end());
        if (hits.empty()) return;
        ++inside;
        std::printf("  inside: %s (%.0f, %.0f): %s%s\n", p.name.c_str(), p.x, p.z, hits[0].c_str(), hits.size() > 1 ? " (+ more)" : "");
    };
    for (const fuel::Hub& h : hubs) {
        test(h.camp);
        for (const auto& p : h.teleports) test(p);
        for (const auto& p : h.vistas) test(p);
    }
    std::printf("collision check: %zu spawn points, %zu inside a solid body (%zu inside a placed object's bounding circle)\n", points, inside, inCircle);
    return 0;
}
