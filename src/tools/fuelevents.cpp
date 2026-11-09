// Lists FUEL's event catalogue (72 career races, 190 hub challenges) and runs each event's
// rules with the player warped through the gates (no physics; fuelview --event-check does the
// same with the vehicle). Usage: fuelevents DATA_DIR [--list]
#include "core/events.hpp"
#include "core/race_rules.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: fuelevents DATA_DIR [--list]\n");
        return 2;
    }
    const bool list = argc > 2 && std::strcmp(argv[2], "--list") == 0;
    const std::vector<fuel::Event> events = fuel::loadEvents(argv[1]);
    int ok = 0, bad = 0, off = 0;
    for (const fuel::Event& e : events) {
        if (list) {
            std::printf("%-8s %-14s t%d %-30s text %5u desc %4u %-6s %-12s %-12s mode %-28s sc %2d ai %2d laps %d gates %2zu limit %4.0f%s%s\n",
                        e.kind == fuel::Event::Career ? "career" : "challnge", e.hub.c_str(), e.tier, e.id.c_str(), e.nameText, e.descText,
                        e.surface.c_str(), e.vehicleFamily.c_str(), e.vehicle.c_str(), fuel::gameModeName(e.race.gameMode()), e.race.scenario,
                        e.race.aiCount, e.race.laps, e.race.checkpoints.size(), e.race.timeLimit, e.playable ? "" : "  UNPLAYABLE: ",
                        e.playable ? "" : e.note.c_str());
            // PK10: start / finish heading (rotation vector y) and the AI vehicle list (AddMissionParam or EMD_SetFilterIA).
            std::printf("   yaw %.4f %.4f ai [", e.race.startYaw, e.race.finishYaw);
            for (size_t i = 0; i < e.aiVehicleLabels.size(); ++i) std::printf("%s%s", i ? " " : "", e.aiVehicleLabels[i].c_str());
            std::printf("]\n");
            continue;
        }
        if (!e.playable) {
            ++off;
            std::printf("%-30s unplayable: %s\n", e.id.c_str(), e.note.c_str());
            continue;
        }
        // Logic warp: the player hops onto each gate; opponents stay on the start; hunted racers
        // are visited in turn.
        fuel::RaceRules rules;
        const bool heli = e.race.gameMode() == fuel::Race::GmHelicopter;
        const size_t others = heli ? 0 : size_t(std::max(0, e.race.aiCount));
        rules.begin(e.race, 1 + others, e.times[0]);
        std::vector<fuel::RaceRules::Vec3> pos(rules.racers.size(), e.race.start);
        for (size_t i = 1; i < pos.size(); ++i) pos[i] = {e.race.start[0] + 40.0f * float(i), e.race.start[1], e.race.start[2]};
        size_t hunted = 1;
        for (int step = 0; step < 60 * 60 * 30 && rules.state != fuel::RaceRules::Finished; ++step) {
            if (rules.state == fuel::RaceRules::Running && step % 30 == 0) {
                if (rules.mode == fuel::Race::GmCatchAI) {
                    while (hunted < rules.racers.size() && rules.racers[hunted].out) ++hunted;
                    if (hunted < pos.size()) pos[0] = pos[hunted];
                } else if (rules.racers[0].next < rules.route.size()) {
                    pos[0] = rules.route[rules.racers[0].next];
                }
            }
            rules.update(1.0 / 60, pos);
        }
        // The losing side: the player dawdles (one hop per 3 s; not at all when alone against the clock) while
        // opponents hop every 0.25 s; hunted racers wait away from the start.
        fuel::RaceRules lose;
        lose.begin(e.race, 1 + others, e.times[0]);
        std::vector<fuel::RaceRules::Vec3> lp(lose.racers.size(), e.race.start);
        for (size_t i = 1; i < lp.size() && lose.mode == fuel::Race::GmCatchAI && !lose.aiTargets.empty(); ++i) lp[i] = lose.aiTargets.back();
        const int slow = lose.soloTimed() ? 1 << 30 : 180;  // alone against the clock: stand still
        for (int step = 0; step < 60 * 60 * 30 && lose.state != fuel::RaceRules::Finished; ++step) {
            if (lose.state == fuel::RaceRules::Running) {
                for (size_t i = 1; i < lp.size() && step % 15 == 0; ++i)
                    if (lose.usesRoute(i) && lose.racers[i].next < lose.route.size()) lp[i] = lose.route[lose.racers[i].next];
                if (step % slow == slow / 2 && lose.mode != fuel::Race::GmCatchAI && lose.mode != fuel::Race::GmHelicopter && lose.racers[0].next < lose.route.size())
                    lp[0] = lose.route[lose.racers[0].next];
            }
            lose.update(1.0 / 60, lp);
        }
        const bool pass = rules.outcome == fuel::RaceRules::Won && lose.outcome == fuel::RaceRules::Lost;
        (pass ? ok : bad)++;
        std::printf("%-30s %-28s %s | slow player: %s\n", e.id.c_str(), fuel::gameModeName(rules.mode), rules.result(pos).c_str(), lose.result(lp).c_str());
    }
    if (!list) std::printf("event logic check: %zu events, %d won and lost as expected, %d failed, %d unplayable\n", events.size(), ok, bad, off);
    return bad ? 1 : 0;
}
