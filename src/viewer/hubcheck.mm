#include "hubcheck.h"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace {

// Slope of the terrain at (x, z) in degrees, over a 6 m baseline.
float slopeDeg(const fuel::Heightfield& t, double x, double z) {
    const float dx = (t.smoothHeightAt(x + 3, z) - t.smoothHeightAt(x - 3, z)) / 6.0f;
    const float dz = (t.smoothHeightAt(x, z + 3) - t.smoothHeightAt(x, z - 3)) / 6.0f;
    return std::atan(std::hypot(dx, dz)) * 57.29578f;
}

constexpr float kMaxSlope = 25.0f;      // degrees: steeper is a cliff, not a place to start
constexpr float kObjectClearance = 3.0f;  // metres from a placed object's bounding circle

}  // namespace

std::string spawnProblem(DriveSession& drive, WorldStreamer& streamer, fuel::MeshRenderer& renderer, const fuel::Heightfield& terrain, float x,
                         float z, float yaw) {
    if (slopeDeg(terrain, x, z) > kMaxSlope) return "cliff";
    if (streamer.nearObject(x, z, kObjectClearance)) return "inside an object";
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

int runHubCheck(const std::vector<fuel::Hub>& hubs, DriveSession& drive, WorldStreamer& streamer, fuel::MeshRenderer& renderer,
                const fuel::Heightfield& terrain) {
    size_t spots = 0, nudged = 0, failed = 0;
    for (const fuel::Hub& h : hubs) {
        size_t n[3] = {0, 0, 0}, bad[3] = {0, 0, 0};
        std::string notes;
        auto test = [&](int kind, const fuel::HubPoint& p) {
            const HubSpawn s = placeAtHubPoint(drive, streamer, renderer, terrain, p);
            ++n[kind], ++spots;
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
    std::printf("hub check: %zu hubs, %zu spawns, %zu nudged to a clear spot, %zu without one: %s\n", hubs.size(), spots, nudged, failed, failed ? "FAILED" : "ok");
    return failed ? 1 : 0;
}
