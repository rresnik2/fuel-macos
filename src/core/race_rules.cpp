#include "race_rules.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>

namespace fuel {

namespace {

float dist2d(const RaceRules::Vec3& a, const RaceRules::Vec3& b) { return std::hypot(a[0] - b[0], a[2] - b[2]); }

bool set(const RaceRules::Vec3& p) { return p[0] != 0 || p[2] != 0; }

}  // namespace

void RaceRules::begin(const Race& r, size_t racerCount, float targetTime) {
    race = r;
    mode = r.gameMode();
    route.clear();
    aiPath.clear();
    aiTargets.clear();
    knockedAt.clear();
    std::vector<Vec3> gates, path;
    for (size_t i = 0; i < r.checkpoints.size(); ++i) {
        const int flag = i < r.checkpointFlags.size() ? r.checkpointFlags[i] : 1;
        // Flag 4 only occurs in helicopter and hunt challenges: the AI's path, not the player's gates (Inferred).
        if (mode == Race::GmCatchAI || flag == 4) path.push_back(r.checkpoints[i]);
        else gates.push_back(r.checkpoints[i]);
    }
    if (mode != Race::GmCatchAI) {
        for (int lap = 0; lap < (r.loops() ? r.laps : 1); ++lap) route.insert(route.end(), gates.begin(), gates.end());
        if (set(r.finish)) route.push_back(r.finish);
    }
    if (mode == Race::GmHelicopter || mode == Race::GmCatchAI) {
        Vec3 from = r.start;
        if (mode == Race::GmHelicopter && !r.aiGrid.empty()) from = {r.aiGrid[0][0], r.start[1], r.aiGrid[0][1]};
        aiPath.push_back(from);
        aiPath.insert(aiPath.end(), path.begin(), path.end());
        if (set(r.finish)) aiPath.push_back(r.finish);
    }
    aiTargets.assign(aiPath.size() > 1 ? aiPath.begin() + 1 : aiPath.end(), aiPath.end());
    racers.assign(std::max<size_t>(1, racerCount), Racer{});
    state = Countdown;
    outcome = Pending;
    reason.clear();
    clock = -3.0;
    timed = r.timeLimit > 0;
    timeLeft = timed ? r.timeLimit : 0;
    helicopter = mode == Race::GmHelicopter;
    heliTravelled = heliFar = 0;
    heliLanded = false;
    if (helicopter) {
        heli = aiPath.front();
        float length = 0;
        for (size_t i = 1; i < aiPath.size(); ++i) length += dist2d(aiPath[i - 1], aiPath[i]);
        // port: the helicopter lands after the event's target time (AddMissionTime 1st value), else at 100 km/h.
        heliSpeed = targetTime > 0 ? length / targetTime : 27.8f;
    }
}

bool RaceRules::usesRoute(size_t i) const { return !(mode == Race::GmCatchAI && i > 0); }

void RaceRules::update(double dt, const std::vector<Vec3>& pos) {
    clock += dt;
    if (state == Countdown) {
        if (clock >= 0) state = Running;
        return;
    }
    if (state == Finished || pos.empty()) return;
    auto end = [&](Outcome o, const std::string& why) {
        state = Finished;
        outcome = o;
        reason = why;
    };
    // Gates (or path points) for every racer.
    const size_t perLap = route.empty() ? 0 : race.loops() ? (route.size() - (set(race.finish) ? 1 : 0)) / size_t(std::max(1, race.laps)) : 0;
    for (size_t i = 0; i < racers.size() && i < pos.size(); ++i) {
        Racer& rc = racers[i];
        const std::vector<Vec3>& line = lineFor(i);
        if (rc.finished >= 0 || rc.out || rc.next >= line.size()) continue;
        const Vec3& g = line[rc.next];
        const bool heightOk = g[1] == 0 || std::fabs(pos[i][1] - g[1]) < 60;  // some challenge gates have y = 0
        if (dist2d(pos[i], g) >= kGateRadius || !heightOk) continue;
        const size_t passed = rc.next++;
        if (i == 0 && timed && passed + 1 < line.size()) {
            const size_t k = perLap ? passed % perLap : passed;
            timeLeft += k < race.checkpointBonus.size() ? race.checkpointBonus[k] : race.checkBonus;
        }
        if (rc.next >= line.size()) rc.finished = clock;
    }
    if (timed) timeLeft = std::max(0.0f, timeLeft - float(dt));

    // Knock-out (port rule from the description "the last racer going through each checkpoint is out"):
    // when every racer still in but one has passed gate k, that one is out.
    if (mode == Race::GmKnockOut && route.size() > 1) {
        for (size_t k = 0; k + 1 < route.size(); ++k) {
            if (std::find(knockedAt.begin(), knockedAt.end(), k) != knockedAt.end()) continue;
            size_t alive = 0, passed = 0, last = 0;
            for (size_t i = 0; i < racers.size(); ++i) {
                if (racers[i].out) continue;
                ++alive;
                if (racers[i].next > k) ++passed;
                else last = i;
            }
            if (alive < 2 || passed + 1 != alive) continue;
            racers[last].out = true;
            racers[last].outAt = clock;
            knockedAt.push_back(k);
            if (last == 0) return end(Lost, "knocked out at checkpoint " + std::to_string(k + 1));
        }
        bool alone = racers.size() > 1;
        for (size_t i = 1; i < racers.size(); ++i) alone = alone && racers[i].out;
        if (alone) return end(Won, "every opponent knocked out");
    }
    // Hunt: touch every hunted racer before the time runs out.
    if (mode == Race::GmCatchAI) {
        size_t caught = 0;
        for (size_t i = 1; i < racers.size() && i < pos.size(); ++i) {
            if (!racers[i].out && dist2d(pos[0], pos[i]) < kCatchDistance && std::fabs(pos[0][1] - pos[i][1]) < 5) {
                racers[i].out = true;
                racers[i].outAt = clock;
            }
            caught += racers[i].out;
        }
        if (racers.size() > 1 && caught + 1 == racers.size()) return end(Won, "every contender caught");
    }
    // Helicopter: flies its path; lands at the end.
    if (helicopter && !heliLanded) {
        heliTravelled += heliSpeed * float(dt);
        float left = heliTravelled;
        heli = aiPath.back();
        heliLanded = true;
        for (size_t i = 1; i < aiPath.size(); ++i) {
            const float seg = dist2d(aiPath[i - 1], aiPath[i]);
            if (left < seg) {
                const float t = seg > 0 ? left / seg : 0;
                for (int c = 0; c < 3; ++c) heli[size_t(c)] = aiPath[i - 1][size_t(c)] + t * (aiPath[i][size_t(c)] - aiPath[i - 1][size_t(c)]);
                heliLanded = false;
                break;
            }
            left -= seg;
        }
        heliFar = dist2d(pos[0], heli) > kHeliKeepClose ? heliFar + float(dt) : 0.0f;
    }
    // The player's finish.
    if (racers[0].finished >= 0) {
        if (timed && racers.size() == 1) return end(Won, "finished in time");
        if (helicopter) return end(Won, "landed before the helicopter");
        const int p = playerPosition(pos);
        return end(p == 1 ? Won : Lost, p == 1 ? "finished first" : "finished " + std::to_string(p) + (p == 2 ? "nd" : p == 3 ? "rd" : "th"));
    }
    if (helicopter && heliLanded) return end(Lost, "the helicopter landed first");
    if (helicopter && heliFar > kHeliGrace) return end(Lost, "lost the helicopter");
    if (timed && timeLeft <= 0) return end(Lost, "out of time");
}

double RaceRules::progress(size_t i, const Vec3& at) const {
    const Racer& r = racers[i];
    if (r.finished >= 0) return 1e12 - r.finished;
    if (r.out) return -1e12 + r.outAt;
    const std::vector<Vec3>& line = lineFor(i);
    if (r.next >= line.size()) return double(r.next) * 1e6;
    return double(r.next) * 1e6 - dist2d(at, line[r.next]);
}

std::vector<int> RaceRules::positions(const std::vector<Vec3>& at) const {
    std::vector<double> score;
    for (size_t i = 0; i < racers.size(); ++i) score.push_back(i < at.size() ? progress(i, at[i]) : -2e12);
    std::vector<int> pos(racers.size(), 1);
    for (size_t i = 0; i < racers.size(); ++i)
        for (size_t j = 0; j < racers.size(); ++j) pos[i] += j != i && (score[j] > score[i] || (score[j] == score[i] && j < i));
    return pos;
}

int RaceRules::stars(const std::vector<Vec3>& at) const {
    if (outcome == Pending) return 0;
    if (racers.size() == 1 || mode == Race::GmCatchAI || helicopter) return outcome == Won ? 3 : 0;
    if (racers[0].finished < 0) return 0;
    return std::max(0, 4 - playerPosition(at));
}

std::string RaceRules::result(const std::vector<Vec3>& at) const {
    char buf[256];
    const double t = std::max(0.0, clock);
    const char* o = outcome == Won ? "WON" : outcome == Lost ? "LOST" : "RUNNING";
    std::snprintf(buf, sizeof buf, "%s %d:%04.1f gate %zu/%zu P%d/%zu stars %d%s%s", o, int(t / 60), t - int(t / 60) * 60,
                  std::min(racers[0].next, route.size()), route.size(), playerPosition(at), racers.size(), stars(at), reason.empty() ? "" : ": ",
                  reason.c_str());
    return buf;
}

}  // namespace fuel
