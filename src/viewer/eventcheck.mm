#include "eventcheck.h"

#include "core/race_rules.hpp"
#include "hubcheck.h"

#include <strings.h>

#include <algorithm>
#include <array>
#include <map>
#include <cmath>
#include <cstdio>

namespace {

using Vec3 = fuel::RaceRules::Vec3;

struct Outcome {
    bool pass = false;
    std::string line;
};

Outcome checkEvent(const fuel::Event& e, DriveSession& drive, WorldStreamer& streamer, fuel::MeshRenderer& renderer, const fuel::Heightfield& terrain) {
    streamer.setMission(e.id);  // its course sub-world, as the mission's teleport streams it in (p35)
    const fuel::Race& r = e.race;
    const int mode = r.gameMode();
    // Opponents without vehicles: they wait on their grid spots (hunted racers along their path,
    // so the hops can reach them); the helicopter is simulated by the rules.
    const size_t others = mode == fuel::Race::GmHelicopter ? 0 : size_t(std::clamp(r.aiCount, 0, 11));
    fuel::RaceRules rules;
    rules.ground = [&terrain](float x, float z) { return terrain.smoothHeightAt(x, z); };
    rules.begin(r, 1 + others, e.times[0]);
    std::vector<Vec3> pos(rules.racers.size(), r.start);
    for (size_t i = 1; i < pos.size(); ++i) {
        if (mode == fuel::Race::GmCatchAI && !rules.aiTargets.empty()) {
            pos[i] = rules.aiTargets[std::min(rules.aiTargets.size() - 1, (i - 1) * rules.aiTargets.size() / others)];
        } else if (i - 1 < r.aiGrid.size()) {
            pos[i] = {r.aiGrid[i - 1][0], r.start[1], r.aiGrid[i - 1][1]};
        } else {
            pos[i] = {r.start[0] - 8.0f * float(i), r.start[1], r.start[2]};
        }
        pos[i][1] = terrain.smoothHeightAt(pos[i][0], pos[i][2]);  // they would stand on the ground
    }
    // Spawn on the start, facing the first target.
    float yaw = r.startYaw;
    const std::vector<Vec3>& first = rules.route.empty() ? rules.aiTargets : rules.route;
    if (!first.empty()) yaw = std::atan2(first[0][0] - r.start[0], first[0][2] - r.start[2]);
    const std::string spawn = spawnProblem(drive, streamer, renderer, terrain, r.start[0], r.start[2], yaw);
    const bool spawnBroken = spawn == "under the terrain" || spawn == "in the air";
    auto player = [&] {
        const simd_float3 p = drive.position();
        return Vec3{p.x, p.y, p.z};
    };
    // Countdown (vehicle held), then hops every 6 steps.
    int steps = 0;
    const int maxSteps = 60 * 60 * 25;  // 25 minutes of race time
    while (rules.state != fuel::RaceRules::Finished && steps < maxSteps) {
        if (rules.state == fuel::RaceRules::Running && steps % 6 == 0) {
            Vec3 to{};
            bool hop = false;
            if (mode == fuel::Race::GmCatchAI) {
                for (size_t i = 1; i < rules.racers.size() && !hop; ++i)
                    if (!rules.racers[i].out) to = pos[i], hop = true;
            } else if (rules.helicopter && rules.racers[0].next < rules.route.size()) {
                // Stay on the helicopter until the landing point is near, then go for it.
                const Vec3& goal = rules.route[rules.racers[0].next];
                const bool near = std::hypot(rules.heli[0] - goal[0], rules.heli[2] - goal[2]) < 300;
                to = near ? goal : rules.heli;
                hop = true;
            } else if (rules.racers[0].next < rules.route.size()) {
                to = rules.route[rules.racers[0].next];
                hop = true;
            }
            if (hop) drive.teleport(to[0], to[2], drive.heading());
        }
        drive.update(1.0f / 60, fuel::DriveInput{0, 0, 0, rules.state == fuel::RaceRules::Countdown});
        pos[0] = player();
        rules.update(1.0 / 60, pos);
        ++steps;
    }
    const bool finishTrigger = mode == fuel::Race::GmCatchAI || rules.racers[0].finished >= 0;
    Outcome o;
    o.pass = !spawnBroken && rules.outcome == fuel::RaceRules::Won && finishTrigger;
    char buf[512];
    std::snprintf(buf, sizeof buf, "%-4s %-8s %-14s %-30s %-28s spawn %-21s %s", o.pass ? "ok" : "FAIL", e.kind == fuel::Event::Career ? "career" : "challnge",
                  e.hub.c_str(), e.id.c_str(), fuel::gameModeName(mode), spawn.empty() ? "ok" : spawn.c_str(), rules.result(pos).c_str());
    o.line = buf;
    return o;
}

}  // namespace

int runEventCheck(const std::vector<fuel::Event>& events, DriveSession& drive, WorldStreamer& streamer, fuel::MeshRenderer& renderer,
                  const fuel::Heightfield& terrain, const std::string& only) {
    int pass = 0, fail = 0, off = 0, warned = 0;
    std::map<std::string, std::array<int, 3>> byMode;  // game mode -> ok, failed, unplayable
    for (const fuel::Event& e : events) {
        if (!only.empty() && strcasecmp(only.c_str(), e.id.c_str()) != 0 && e.hub != only) continue;
        const std::string mode = e.playable ? fuel::gameModeName(e.race.gameMode()) : "(no data)";
        if (!e.playable) {
            ++off;
            ++byMode[mode][2];
            std::printf("off  %-8s %-14s %-30s unplayable: %s\n", e.kind == fuel::Event::Career ? "career" : "challnge", e.hub.c_str(), e.id.c_str(),
                        e.note.c_str());
            continue;
        }
        const Outcome o = checkEvent(e, drive, streamer, renderer, terrain);
        (o.pass ? pass : fail)++;
        ++byMode[mode][o.pass ? 0 : 1];
        warned += o.line.find("spawn ok ") == std::string::npos;
        std::printf("%s\n", o.line.c_str());
        std::fflush(stdout);
    }
    std::printf("by mode:\n");
    for (const auto& [m, n] : byMode) std::printf("  %-28s %3d ok %3d failed %3d unplayable\n", m.c_str(), n[0], n[1], n[2]);
    std::printf("event check: %d events, %d ok, %d failed, %d unplayable (with a reason); %d starts needed a settle warning: %s\n", pass + fail + off,
                pass, fail, off, warned, fail ? "FAILED" : "ok");
    return fail ? 1 : 0;
}
