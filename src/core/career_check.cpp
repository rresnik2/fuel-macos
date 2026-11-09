// fuelview --career-check: a scripted career run checked against the decoded rules
// (career.hpp; notes/p39-career.md). The expected values are worked out here from the
// script data with the formulas read from FUEL.exe, independently of Career's code paths
// where possible. Uses a temporary save file, never the player's career.sav.
#include "career.hpp"
#include "race_editor.hpp"  // PK21
#include "save_import.hpp"  // PK25
#include "freeroam.hpp"
#include "hubs.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <set>
#include <sys/stat.h>
#include <unistd.h>

namespace fuel {

int runCareerCheck(const std::string& dataDir) {
    int fails = 0, checks = 0;
    auto check = [&](bool ok, const std::string& what) {
        ++checks;
        if (!ok) ++fails;
        std::printf("  %s %s\n", ok ? "ok  " : "FAIL", what.c_str());
    };
    const CareerRules rules = loadCareerRules(dataDir);
    size_t careerN = 0, challengeN = 0, liveryN = 0, vistaN = 0, trafficN = 0;
    for (const auto& h : rules.hubs) {
        careerN += h.career.size(), challengeN += h.challenges.size(), liveryN += h.liveries.size();
        vistaN += size_t(h.vistas), trafficN += h.traffic.size();
    }
    std::printf("career check: %zu hubs, %zu career races, %zu challenges, %zu vehicles, %zu vehicle liveries, %zu pilot liveries, "
                "%zu vistas, %zu traffic vehicles, %zu unlock rules, %zu trophies\n",
                rules.hubs.size(), careerN, challengeN, rules.vehicles.size(), liveryN, rules.pilotLiveries.size(), vistaN, trafficN,
                rules.unlocks.size(), rules.trophies.size());
    check(rules.hubs.size() == 19 && careerN == 72 && challengeN == 190,
          "scripts: 19 hubs, 72 career races (57 EMD blocks + 15 world MISSION_COURSE nodes, 0x4f4db0), 190 challenges");
    check(rules.trophies.size() == 45 && vistaN == 95, "scripts: 45 trophies, 95 vista points");
    if (rules.hubs.empty() || rules.hubs[0].career.empty() || rules.hubs[0].challenges.size() < 4) {
        std::printf("career check: FAILED (no scripts at %s)\n", dataDir.c_str());
        return 1;
    }

    Career c(rules);
    CareerState& s = c.state();
    auto mission = [&](const std::string& n) -> const CareerRules::Mission& { return rules.missions[size_t(rules.missionIndex(n))]; };
    auto fuelOf = [&](const std::string& n, int i) { return uint32_t(std::lround(mission(n).fuel[i])); };

    // 1. New game (InitEmptySave): hubs needing no stars are open, ALWAYS / HUB rules applied.
    int open = 0;
    for (size_t h = 0; h < rules.hubs.size(); ++h) open += c.hubUnlocked(int(h)) == (rules.hubs[h].starsNeeded < 1) && c.hubUnlocked(int(h));
    check(open == 1 && c.hubUnlocked(0), "new game: only " + rules.hubs[0].name + " is unlocked (stars needed < 1, 0x578a20)");
    check(s.fuel == 0 && c.stars() == 0 && c.challengesWon() == 0, "new game: no fuel, stars or challenges");
    {  // ---- PK27: 0x553c80(1) factory options; 0x554130 reads the voice level +0x25c (options +0x10), clamped 0..16
        float bright = 0;
        int32_t music = 0, sfx = 0, e8 = 0;
        std::memcpy(&bright, s.options, 4), std::memcpy(&music, s.options + 8, 4), std::memcpy(&sfx, s.options + 0xc, 4), std::memcpy(&e8, s.options + 0x28, 4);
        check(bright == 1.0f && music == 8 && sfx == 8 && c.voiceLevel() == 8 && s.options[0x26] == 1 && e8 == 2 && s.options[0x30] == 1,
              "new game: FUEL's factory options (0x553c80(1): brightness 1, music / SFX / voice 8 of 16, vibration on)");
    }  // ---- PK27 end
    {
        bool same = true;
        std::string owned;
        for (size_t v = 0; v < rules.vehicles.size(); ++v) {
            bool unlocked = false;
            for (const auto& u : rules.unlocks)
                if (u.kind == "VEHICLE" && rules.vehicleIndex(u.target) == int(v)) {
                    const int h = rules.hubIndex(u.arg);
                    unlocked |= u.cond == "ALWAYS" || (u.cond == "HUB" && h >= 0 && rules.hubs[size_t(h)].starsNeeded <= 0);
                }
            const bool own = unlocked && rules.vehicles[v].price < 1;
            same &= bool(s.vehicles[v].unlocked) == unlocked && bool(s.vehicles[v].owned) == own;
            if (own && rules.vehicles[v].cls >= 0) owned += " " + rules.vehicles[v].label;
        }
        check(same, "new game: vehicles unlocked by ALWAYS/HUB rules, free ones owned (0x5641c0, 0x569e60); owned:" + owned);
    }

    // 2. Entering the first hub (0x481cd0).
    const std::string cr = rules.missions[size_t(rules.hubs[0].career[0])].name;
    const int islands = 1;
    const std::string race = cr;  // YellowStone's first race (CR1-Yell-CP, a world MISSION_COURSE)
    const std::string ch1 = rules.missions[size_t(rules.hubs[0].challenges[0])].name;
    const std::string ch4 = rules.missions[size_t(rules.hubs[0].challenges[3])].name;
    check(!c.eventAvailable(race), "before entering: " + race + " not available");
    check(!c.enterHub(1), "a locked hub cannot be entered");
    check(c.enterHub(0) && c.eventAvailable(race) && c.eventAvailable(ch1) && !c.eventAvailable(ch4),
          "entering " + rules.hubs[0].name + ": career races and the first 3 challenges available (0x57a100, 0x57a240)");

    // 3. A silver race won (medals: rank = the race's level, race +0x3e8; won = finished 1st): silver and bronze (0x630dd0,
    //    medals from rank to 2).
    uint32_t fuel = 0;
    CareerAward a = c.finishEvent(race, true, 1, 2, 201.5);
    fuel += fuelOf(race, 1) + fuelOf(race, 2);
    check(a.counted && a.newMedals == 2 && a.medalNew[1] && a.medalNew[2] && !a.medalNew[0], race + " silver race won: silver + bronze");
    check(s.fuel == fuel && a.fuelWon == fuelOf(race, 1) + fuelOf(race, 2), "fuel " + std::to_string(s.fuel) + " = silver + bronze rewards (AddMissionFuel)");
    check(c.stars() == 2, "stars 2 (0x578b50: rank 1 = 2 stars)");
    check(std::find(a.trophies.begin(), a.trophies.end(), 2) != a.trophies.end(), "achievement 2 (WON Stars 1) awarded locally");
    const MissionStatus* st = c.status(race);
    check(st && st->bestRank == 1 && st->lastPlace == 2 && st->bestScore == 0x7fffffffu - 20150u, "status block: rank 1 (the level), place 2, score 0x7fffffff - 20150");

    // 4. A lost race changes nothing (state 7): medals: a gold race finished 2nd is lost (bit 0x800 is in no end list);
    //    0x6300c0 writes no last rank, place or score then.
    a = c.finishEvent(race, false, 0, 2, 150.0);
    check(!a.counted && s.fuel == fuel && c.stars() == 2 && c.status(race)->lastRank == 1 && c.status(race)->lastPlace == 2 &&
              c.status(race)->lastScore == 0x7fffffffu - 20150u,
          "gold race lost (2nd): no medal, no fuel, last rank / place / score kept");

    // 5. A gold race won: only gold is new.
    a = c.finishEvent(race, true, 0, 1, 180.0);
    fuel += fuelOf(race, 0);
    check(a.newMedals == 1 && a.medalNew[0] && a.medalHad[1] && a.medalHad[2] && s.fuel == fuel && c.stars() == 3,
          race + " gold race won: gold added, silver/bronze kept, fuel " + std::to_string(s.fuel) + ", stars 3");
    // 6. A bronze race won later gives nothing (rank 2 does not beat best 0); last rank = 2.
    a = c.finishEvent(race, true, 2, 3, 240.0);
    check(a.newMedals == 0 && a.fuelWon == 0 && c.status(race)->bestRank == 0 && c.status(race)->lastRank == 2 &&
              c.status(race)->bestScore == 0x7fffffffu - 18000u, "bronze race won afterwards: no medal, best rank and score kept, last rank 2");

    {  // medals: the ladder on a fresh career: bronze race won (bronze), then a gold race won (gold + silver new, their fuel),
       // a gold race finished 2nd (lost, state 7), then a won silver race (nothing: rank 1 does not beat 0) [O 0x630dd0].
        Career l(rules);
        l.enterHub(0);
        const uint32_t f0 = l.state().fuel;
        CareerAward b = l.finishEvent(race, true, 2, 1, 260.0);
        const bool bronze = b.counted && b.newMedals == 1 && b.medalNew[2] && l.stars() == 1 && b.fuelWon == fuelOf(race, 2);
        CareerAward g = l.finishEvent(race, true, 0, 1, 185.0);
        const bool gold = g.newMedals == 2 && g.medalNew[0] && g.medalNew[1] && g.medalHad[2] && l.stars() == 3 &&
                          g.fuelWon == fuelOf(race, 0) + fuelOf(race, 1) &&
                          l.state().fuel == f0 + fuelOf(race, 0) + fuelOf(race, 1) + fuelOf(race, 2);
        CareerAward x = l.finishEvent(race, false, 0, 2, 182.0);
        CareerAward y = l.finishEvent(race, true, 1, 1, 200.0);
        check(bronze && gold && !x.counted && y.counted && y.newMedals == 0 && y.fuelWon == 0 && l.status(race)->bestRank == 0 &&
                  l.status(race)->lastRank == 1,
              race + " ladder: bronze race won = bronze; gold race won = gold + silver (+their fuel); 2nd = lost; silver race won = nothing");
    }

    // 7. Challenge (0x630ca0): fuel[0] once.
    a = c.finishEvent(ch1, true, 0, 1, 100.0);
    fuel += fuelOf(ch1, 0);
    check(a.fuelWon == fuelOf(ch1, 0) && s.fuel == fuel && c.challengesWon() == 1 && c.stars() == 3,
          "challenge " + ch1 + " won: +" + std::to_string(fuelOf(ch1, 0)) + " fuel, no stars");
    check(std::find(a.trophies.begin(), a.trophies.end(), 4) != a.trophies.end(), "achievement 4 (WON Challenge 1)");
    a = c.finishEvent(ch1, true, 0, 1, 90.0);
    check(a.fuelWon == 0 && s.fuel == fuel, "challenge won again: no fuel");

    // 8. Hub unlock by stars (0x579720 / 0x578aa0) and the vehicles it brings (UnlockNeed HUB).
    const int need = rules.hubs[islands].starsNeeded;
    check(!c.hubUnlocked(islands) == (c.stars() < need), rules.hubs[islands].name + " needs " + std::to_string(need) + " stars");
    // The next stars come from YellowStone's second race (the award code does not check that a
    // race's hub is open; the menus do).
    std::string other;
    for (int hh : {0, islands})
        for (int idx : rules.hubs[size_t(hh)].career)
            if (other.empty() && rules.missions[size_t(idx)].name != race) other = rules.missions[size_t(idx)].name;
    if (!other.empty()) {
        a = c.finishEvent(other, true, 0, 1, 300.0);
        fuel += fuelOf(other, 0) + fuelOf(other, 1) + fuelOf(other, 2);
        const bool opened = std::find(a.hubsUnlocked.begin(), a.hubsUnlocked.end(), rules.hubs[islands].name) != a.hubsUnlocked.end();
        check(c.stars() == 6 && opened == (6 >= need) && c.hubUnlocked(islands) == (6 >= need) && s.fuel == fuel,
              other + " gold: stars 6, " + rules.hubs[islands].name + (opened ? " unlocked" : " still locked"));
        bool vehiclesOk = true;
        for (const auto& u : rules.unlocks)
            if (u.kind == "VEHICLE" && u.cond == "HUB" && rules.hubIndex(u.arg) == islands) vehiclesOk &= c.vehicleUnlocked(u.target) == opened;
        check(vehiclesOk, "vehicles of UnlockNeed ... HUB \"" + rules.hubs[islands].name + "\" follow the hub");
        check(s.hubsUnlockedByRaces() == (opened ? 1 : 0), "game +0x346c counts the race that unlocked a hub");
    }

    // 9. Garage (0x4ce590).
    std::string buy, dear, locked;
    for (size_t v = 0; v < rules.vehicles.size(); ++v) {
        const auto& rv = rules.vehicles[v];
        if (rv.cls < 0) continue;
        if (s.vehicles[v].unlocked && !s.vehicles[v].owned && rv.price > 0 && uint32_t(rv.price) <= s.fuel && buy.empty()) buy = rv.label;
        if (!s.vehicles[v].unlocked && locked.empty()) locked = rv.label;
    }
    if (!buy.empty()) {
        const uint32_t before = s.fuel;
        const int price = c.vehiclePrice(buy);
        check(c.buyVehicle(buy) == Career::Bought && c.vehicleOwned(buy) && s.fuel == before - uint32_t(price),
              "bought " + buy + " for " + std::to_string(price) + " fuel");
        check(c.buyVehicle(buy) == Career::AlreadyOwned, "buying it again: already owned");
        fuel = s.fuel;
    } else {
        check(false, "no affordable unlocked vehicle to buy");
    }
    for (size_t v = 0; v < rules.vehicles.size(); ++v)
        if (s.vehicles[v].unlocked && !s.vehicles[v].owned && rules.vehicles[v].price > int(s.fuel)) { dear = rules.vehicles[v].label; break; }
    if (dear.empty()) {  // everything unlocked is affordable: try one with one fuel point too few
        for (size_t v = 0; v < rules.vehicles.size() && dear.empty(); ++v)
            if (s.vehicles[v].unlocked && !s.vehicles[v].owned && rules.vehicles[v].price > 0) dear = rules.vehicles[v].label;
        if (!dear.empty()) s.fuel = uint32_t(c.vehiclePrice(dear) - 1);
    }
    if (!dear.empty()) {
        const uint32_t before = s.fuel;
        check(c.buyVehicle(dear) == Career::TooExpensive && s.fuel == before && !c.vehicleOwned(dear),
              dear + " too expensive (InfoBoxGarageVehicleIsTooExpensive), fuel untouched");
    }
    s.fuel = fuel;
    if (!locked.empty()) check(c.buyVehicle(locked) == Career::Locked, locked + " locked");

    // 10. Save, reload, compare; a damaged file is refused.
    const std::string path = (std::filesystem::temp_directory_path() / "fuel-career-check.sav").string();
    s.tutorials[12].seen = 1, s.tutorials[3].off = 1;  // P62: SECTION_TUTORIAL (0x56e140 / 0x56e0d0)
    check(c.save(path), "saved " + path);
    if (const char* keep = std::getenv("FUEL_CAREER_CHECK_OUT"); keep && *keep) c.save(keep);  // a mid-career save for menu shots
    Career d(rules);
    check(d.load(path) && d.serialize() == c.serialize() && d.state().fuel == s.fuel && d.stars() == c.stars() &&
              d.hubUnlocked(islands) == c.hubUnlocked(islands) && (buy.empty() || d.vehicleOwned(buy)) && d.eventAvailable(race) &&
              d.state().tutorials[12].seen == 1 && d.state().tutorials[3].off == 1 && d.state().tutorials[0].seen == 0,
          "reloaded: same bytes, tutorials (SECTION_TUTORIAL), fuel, stars, hubs, vehicles, availability");
    {
        const std::vector<uint8_t> b = c.serialize();
        const char want[] = "SECTION_SAVESTRUCT_INFOS";
        uint32_t size = 0;
        std::memcpy(&size, &b[16 + 28], 4);  // name 25 bytes -> (25 | 3) + 1 = 28
        check(b.size() > 48 && std::memcmp(&b[16], want, sizeof want) == 0 && size == 33 + 33 + 4 + 4 + 0xb8 + 0x38 + 4,
              "layout: SECTION_SAVESTRUCT_INFOS, padded to 28, body 318 bytes (0x566110)");
        std::vector<uint8_t> bad = b;
        bad[3] ^= 0xff;
        Career e(rules);
        const uint32_t f0 = e.state().fuel;
        check(!e.deserialize(bad) && e.state().fuel == f0, "damaged header refused");
    }
    {  // PHYSIQUE (port deviation): the outfit round-trips in PORT_WARDROBE; FUEL's sections (what PK25's import and the
       // PC layout know) come out byte-identical whatever the outfit; a stream without the section loads the default.
        Career w(rules);
        w.deserialize(c.serialize());
        w.setRiderOutfit(3, false);
        Career x(rules);
        const bool same = x.deserialize(w.serialize()) && x.riderOutfit() == 3 && !x.riderHelmet();
        std::vector<uint8_t> a0 = c.sections(), a1 = w.sections();
        auto cut = [](std::vector<uint8_t> v) {  // the bytes before SECTION_PORT_WARDROBE
            const std::string tag = "SECTION_PORT_WARDROBE";
            for (size_t i = 0; i + tag.size() < v.size(); ++i)
                if (std::memcmp(&v[i], tag.data(), tag.size()) == 0) return std::vector<uint8_t>(v.begin(), v.begin() + long(i));
            return v;
        };
        Career y(rules);
        const bool dflt = y.deserializeSections(cut(a1)) && y.riderOutfit() == 1 && y.riderHelmet();
        check(same && cut(a0) == cut(a1) && dflt, "wardrobe: outfit 3 / no helmet round-trips (PORT_WARDROBE), FUEL's sections unchanged, "
                                                  "a save without it gets riding gear + helmet");
    }
    std::filesystem::remove(path);

    // 11. Fuel cap (99,999,999) and the whole career: 216 stars, every hub, achievements.
    s.fuel = 99999990;
    a = c.finishEvent(other.empty() ? race : other, true, 0, 1, 1.0);  // nothing new: rank already 0
    for (const auto& h : rules.hubs)
        for (int idx : h.career) c.finishEvent(rules.missions[size_t(idx)].name, true, 0, 1, 100.0);
    check(s.fuel == 99999999u, "fuel capped at 99,999,999");
    bool allHubs = true;
    for (size_t h = 0; h < rules.hubs.size(); ++h) allHubs &= c.hubUnlocked(int(h));
    check(c.stars() == int(3 * careerN) && allHubs, "all career gold: " + std::to_string(c.stars()) + " stars, every hub unlocked");
    const std::set<int> won(s.trophiesWon.begin(), s.trophiesWon.end());
    check(won.count(6) && won.count(17) && won.count(18) && won.count(19),
          "achievements 6 (OPEN Mount Rainier, 170), 17 (40), 18 (120), 19 (216 stars)");

    // 12. A new career played hub by hub as the menus allow (enter unlocked hubs, race what is
    // available): every hub opens; YellowStone alone opens Islands.
    {
        auto play = [&](int rank, std::string* order) {
            Career g(rules);
            for (bool changed = true; changed;) {
                changed = false;
                for (size_t h = 0; h < rules.hubs.size(); ++h) {
                    if (!g.hubUnlocked(int(h))) continue;
                    g.enterHub(int(h));
                    for (int idx : rules.hubs[h].career) {
                        const std::string& n = rules.missions[size_t(idx)].name;
                        const MissionStatus* ms = g.status(n);
                        if (!g.eventAvailable(n) || (ms && ms->bestRank <= rank)) continue;
                        const CareerAward aw = g.finishEvent(n, true, rank, rank + 1, 200.0);
                        changed = true;
                        if (order)
                            for (const std::string& hu : aw.hubsUnlocked) *order += " " + hu + "@" + std::to_string(g.stars());
                    }
                }
            }
            int open = 0;
            for (size_t h = 0; h < rules.hubs.size(); ++h) open += g.hubUnlocked(int(h));
            return std::make_pair(open, g.stars());
        };
        Career y(rules);
        y.enterHub(0);
        for (int idx : rules.hubs[0].career) y.finishEvent(rules.missions[size_t(idx)].name, true, 0, 1, 200.0);
        check(rules.hubs[0].career.size() == 3 && y.stars() == 9 && y.hubUnlocked(islands),
              rules.hubs[0].name + " alone: " + std::to_string(rules.hubs[0].career.size()) + " races, " + std::to_string(y.stars()) +
                  " stars, " + rules.hubs[islands].name + (y.hubUnlocked(islands) ? " unlocked" : " locked"));
        std::string order;
        const auto gold = play(0, &order);
        std::printf("       gold career, hubs in the order they open:%s\n", order.c_str());
        check(gold.first == int(rules.hubs.size()) && gold.second == 216,
              "gold career hub by hub: " + std::to_string(gold.first) + "/" + std::to_string(rules.hubs.size()) + " hubs, " +
                  std::to_string(gold.second) + " stars");
        const auto silver = play(1, nullptr), bronze = play(2, nullptr);
        std::printf("       silver everywhere: %d hubs, %d stars; bronze everywhere: %d hubs, %d stars\n", silver.first, silver.second,
                    bronze.first, bronze.second);
    }
    // 13. Free roam (P44, notes/p44-freeroam.md): travel, vistas, liveries, barrels, discovery, statistics.
    {
        Career g(rules);
        CareerState& gs = g.state();
        const int ys = 0, isl = rules.hubIndex("Islands"), gc = rules.hubIndex("Grand Canyon"), sl = rules.hubIndex("Salt Lake"),
                  m1 = rules.hubIndex("Mountain 1");
        auto won = [&](int id) { return std::find(gs.trophiesWon.begin(), gs.trophiesWon.end(), id) != gs.trophiesWon.end(); };
        // Travel history (0x56ca20 / 0x56ca00 / 0x56cb20).
        g.setCurrentHub(ys), g.setCurrentHub(ys), g.setCurrentHub(isl);
        g.checkTrophies();
        check(g.travelled(ys, isl) && gs.trips.size() == 2 && won(1), "travel " + rules.hubs[ys].name + " -> " + rules.hubs[isl].name +
                                                                         ": GOTO trophy 1, the repeated hub kept one trip");
        g.useHeliport(), g.setCurrentHub(gc);
        g.setCurrentHub(sl), g.setCurrentHub(m1);
        g.checkTrophies();
        check(!g.travelled(isl, gc) && g.travelled(sl, m1) && gs.heliportUses() == 1 && won(36) && !won(40),
              "a heliport flight does not count as travel (0x56ca00): trophy 36 yes, 40 no");
        for (int i = 0; i < 12; ++i) g.setCurrentHub(i % 2 ? ys : isl);
        check(gs.trips.size() == 8, "travel history keeps 8 trips");

        // Mission starts for discovery: challenge k of YellowStone at 1000 m x (k + 1) east of its first vista.
        const auto& Y = rules.hubs[ys];
        const float vx = Y.vistaPoints[0].x, vz = Y.vistaPoints[0].z;
        auto startOf = [&](const std::string& n, float& x, float& z) {
            for (size_t k = 0; k < Y.challenges.size(); ++k)
                if (rules.missions[size_t(Y.challenges[k])].name == n) return x = vx + 1000.0f * float(k + 1), z = vz, true;
            return false;
        };
        Career f(rules);
        FreeRoam fr(f, startOf);
        FreeRoamFrame fm;
        fm.dt = 1.0f / 60, fm.pos = {vx + 3000.0f, 0, vz};
        fm.onScreen = [](float, float, float) { return true; };
        f.setCurrentHub(ys);
        fr.update(fm);
        check(f.state().vistas[ys][0].a == 0 && !f.state().challenges[ys][3].available, "hub not opened: nothing revealed or discovered");
        f.enterHub(ys);
        fr.update(fm);
        int disc = 0;
        for (size_t k = 0; k < Y.challenges.size(); ++k) disc += f.state().challenges[ys][k].available;
        // within 5000 m of x + 3000: challenges at +1000..+7000 -> k = 0..6 (|dx| < 5000) plus the first three anyway
        const int expect = int(std::min<size_t>(Y.challenges.size(), 7));
        check(disc == expect && f.state().vistas[ys][0].a == 1 && f.state().vistas[ys][0].b == 0,
              "challenge discovery within 5000 m (0x4c0150): " + std::to_string(disc) + " available; vista 1 revealed on screen (0x4c01d0)");
        std::vector<int> kinds;
        for (int i = 0; i < 400; ++i) {
            fm.pos = {vx + 3000.0f, 0, vz}, fm.dt = 1.0f / 60;
            fr.update(fm);
            if (fr.messages().showing() && (kinds.empty() || kinds.back() != fr.messages().current().kind)) kinds.push_back(fr.messages().current().kind);
        }
        check(!kinds.empty() && (kinds[0] == 1 || kinds[0] == 3), "MessagesHUD shows NewChallengeDetected / NewVistaPointDetected in turn");
        // Visit: within 4 m.
        fr.messages().clear();
        fm.pos = {vx + 3.0f, 0, vz};
        fr.update(fm);
        fr.update(fm);
        const bool visited = f.state().vistas[ys][0].b == 1;
        fr.messages().update(0);
        const auto msg = fr.messages().current();
        check(visited && msg.kind == 7 && msg.param == int(Y.vistaPoints[0].text) && fr.takeSaveRequest(),
              "vista visited within 4 m (0x4ed0a0): message 7 with the vista's text " + std::to_string(Y.vistaPoints[0].text) + ", save");
        bool t3 = false;
        for (int id : fr.takeTrophies()) t3 |= id == 3;
        check(t3, "achievement 3 (FIND Vistapoint 1) earned");
        // Message fade (0x4afe00).
        MessageQueue q;
        q.push(3, -1), q.push(12, 100);
        q.update(0);
        q.update(0.25f);
        const float a1 = q.alpha();
        q.update(2.25f);
        const float a2 = q.alpha();
        q.update(2.25f);
        const float a3 = q.alpha();
        q.update(0.3f), q.update(0);
        check(std::fabs(a1 - 0.5f) < 1e-4f && a2 == 1.0f && std::fabs(a3 - 0.5f) < 1e-4f && q.current().kind == 12,
              "message fade 0.5 / 1 / 0.5 at 0.25 / 2.5 / 4.75 s, then the next message");

        // Livery POI: the first POI livery of any hub.
        int lh = -1, lk = -1;
        for (size_t h = 0; h < rules.hubs.size() && lh < 0; ++h)
            for (size_t k = 0; k < rules.hubs[h].liveries.size(); ++k)
                if (rules.hubs[h].liveries[k].kind == "POI") { lh = int(h), lk = int(k); break; }
        const int ownedBefore = f.vehicleLiveriesOwned();
        fr.messages().clear();
        fm.pos = {rules.hubs[size_t(lh)].liveries[size_t(lk)].x + 1.0f, 0, rules.hubs[size_t(lh)].liveries[size_t(lk)].z};
        fr.update(fm);
        fr.messages().update(0);
        const int vi = rules.vehicleIndex(rules.hubs[size_t(lh)].liveries[size_t(lk)].mesh);
        check(f.vehicleLiveriesOwned() == ownedBefore + 1 && fr.messages().current().kind == 5 && vi >= 0 &&
                  fr.messages().current().param == int(rules.vehicles[size_t(vi)].nameText),
              "livery pickup (0x4fcb60): owned, message 5 with the vehicle's name (" + rules.hubs[size_t(lh)].liveries[size_t(lk)].mesh + ")");

        // Fuel barrels (0x51d600): 100 = one barrel; a 400 pack of four counts 4 once.
        const uint32_t fuel0 = f.state().fuel;
        fm.pos = {0, 0, 0};
        fm.onScreen = nullptr;  // (POIs around the origin would be revealed first)
        fr.setBarrels({{0.5f, 0, 0, 100, 0.5f}, {40, 0, 0, 400, 0.5f}, {41, 0, 1, 400, 0.5f}, {42, 0, 0, 400, 0.5f}, {40, 0, 2, 400, 0.5f}});
        fr.messages().clear();
        // PK09: barrels are taken by contact (0x531310): the player's body, a 1.8 x 1 x 3.6 m box on the ground.
        auto car = [](float x, float z) {
            ContactBody b;
            b.boxes.push_back({{x, 0.5f, z}, {0.9f, 0.5f, 1.8f}});
            return b;
        };
        fm.bodies = {car(0, 0)};
        fr.update(fm);
        fr.messages().update(0);
        bool t29 = false;
        for (int id : fr.takeTrophies()) t29 |= id == 29;
        check(f.state().fuel == fuel0 + 100 && f.state().fuelFound == 1 && fr.messages().current().kind == 12 &&
                  fr.messages().current().param == 100 && t29,
              "barrel IT_Fuel01: +100 fuel, 1 barrel, infofuel message, achievement 29 (FIND fuel 1)");
        fm.pos = {40.5f, 0, 0}, fm.bodies = {car(40.5f, 0)};
        fr.update(fm);
        check(f.state().fuel == fuel0 + 500 && f.state().fuelFound == 5 && fr.takePicked().size() == 5,
              "IT_Fuel02 pack: +400 fuel and 4 barrels once, its other agents within 5 m taken");
        fr.setBarrels({{0.5f, 0, 0, 100, 0.5f}});
        const bool kept = fr.barrels()[0].taken;
        fr.setBarrels({}), fr.setBarrels({{0.5f, 0, 0, 100, 0.5f}});
        check(kept && !fr.barrels()[0].taken, "a barrel keeps its state while spawned and respawns with its cell");
        for (int i = 0; i < 124; ++i) f.collectFuel(400);
        f.checkTrophies();
        auto fwon = [&](int id) { return std::find(f.state().trophiesWon.begin(), f.state().trophiesWon.end(), id) != f.state().trophiesWon.end(); };
        check(f.state().fuelFound == 501 && fwon(30) && fwon(31), "achievements 30 / 31 (100 / 500 barrels)");

        // Statistics: distance, jump, fall, tricks, heliports.
        Career st(rules);
        st.addDistance(1500.0f, false, false);
        check(st.state().freeRideKm() == 1 && std::fabs(st.state().freeRideM() - 500.0f) < 1e-3f && st.state().onRoadKm() == 1,
              "distance 0x569460: 1500 m -> 1 km + 500 m (free ride, on road)");
        st.addDistance(800.0f, true, true);
        check(st.state().raceKm() == 0 && std::fabs(st.raceDistance() - 800.0) < 1e-3 && std::fabs(st.freeRideDistance() - 1500.0) < 1e-3,
              "race distance counts apart from free ride (race object state not 0/1)");
        FreeRoam sr(st, nullptr);
        FreeRoamFrame a;
        a.dt = 0.05f, a.vehicleClass = 3, a.onGround = true;
        a.dt = 1.0f;
        for (int i = 0; i < 2000; ++i) {
            a.vel = {50, 0, 0};
            sr.update(a);  // 100000 m of free ride
        }
        a.dt = 0.05f;
        a.onGround = false;
        for (int i = 0; i < 60; ++i) {  // 3 s airborne: 30 m/s along, falling 60 m/s
            a.vel = {30, -60, 0};
            sr.update(a);
        }
        a.onGround = true, a.vel = {0, 0, 0};
        sr.update(a);
        check(std::fabs(st.state().longestJump() - 88.5f) < 0.01f && std::fabs(st.state().longestFall() - 177.0f) < 0.01f &&
                  std::fabs(st.state().longestAirTime() - 2.95f) < 1e-3f,
              "jump 0x4c8450: 60 frames of 0.05 s in the air = 88.5 m long, 177 m fall, 2.95 s (the take-off frame starts it)");
        for (int j = 0; j < 200; ++j) {
            a.airtime = 0.5f, a.heightAbove = 3, a.vel = {0, 9, 0}, a.onGround = false;
            sr.update(a);
            sr.update(a);
            a.airtime = 0, a.onGround = true;
            sr.update(a);
        }
        for (int i = 0; i < 100; ++i) st.useHeliport();
        sr.update(a);
        sr.update(a);
        st.checkTrophies();
        auto swon = [&](int id) { return std::find(st.state().trophiesWon.begin(), st.state().trophiesWon.end(), id) != st.state().trophiesWon.end(); };
        check(st.state().tricks() == 200 && swon(32) && !swon(33) && swon(35) && swon(39) && swon(41) && swon(45),
              "achievements 32 (100 km), 35 (jump 80), 39 (fall 150), 41 (100 heliports), 45 (200 tricks); 33 not yet");

        // No crash / no GPS (race +0x64 / +0x65 -> 0x630dd0).
        auto race = [&](bool wrecked, bool gps) {
            Career r(rules);
            FreeRoam rr(r, nullptr);
            FreeRoamFrame b;
            b.dt = 1.0f / 60, b.mission = true, b.wrecked = wrecked, b.gpsShown = gps;
            r.enterHub(0);
            rr.update(b), rr.update(b);
            r.finishEvent(rules.missions[size_t(rules.hubs[0].career[0])].name, true, 0, 1, 100.0);
            return std::make_pair(r.state().noCrashWin(), r.state().noGpsWin());
        };
        const auto clean = race(false, false), dirty = race(true, true);
        check(clean.first == 1 && clean.second == 1 && dirty.first == 0 && dirty.second == 0,
              "career win without a wreck / with the GPS off sets CAREER_NO_CRASH / NO_GPS (achievements 37 / 38)");

        // P61: special traffic catches (0x5c53b0): the reward by kind, once; message 10 / 9 / 11; the save keeps them.
        {
            Career t(rules);
            FreeRoam tr(t, nullptr);
            int vi = -1, li = -1, ci = -1;
            for (size_t k = 0; k < rules.hubs[0].traffic.size(); ++k) {
                const int kd = rules.hubs[0].traffic[k].kind();
                (kd == 0 ? vi : kd == 1 ? li : ci) = int(k);
            }
            const auto& T = rules.hubs[0].traffic;
            check(T.size() == 3 && vi >= 0 && li >= 0 && ci >= 0 && T[size_t(vi)].x0 == -43725 && T[size_t(vi)].z1 == -27595,
                  "AddHubTrafficVehicle: Yellowstone has a vista, a livery and a challenge truck with their boxes");
            const int mv = t.catchTraffic(0, vi), again = t.catchTraffic(0, vi);
            bool allVistas = true, allLiveries = true, allChallenges = true;
            for (const auto& v : t.state().vistas[0]) allVistas = allVistas && v.a;
            check(mv == 10 && again == 0 && allVistas && t.trafficCaught(0, vi), "catch the vista truck: every vista of the hub revealed, message 10, once");
            check(t.catchTraffic(0, li) == 9, "catch the livery truck: message 9");
            for (const auto& v : t.state().liveries[0]) allLiveries = allLiveries && v.a;
            check(tr.catchTraffic(0, ci), "catch the challenge truck through FreeRoam (save request, message)");
            for (const auto& m : t.state().challenges[0]) allChallenges = allChallenges && m.available;
            check(allLiveries && allChallenges && tr.takeSaveRequest(), "liveries revealed, challenges discovered, SaveGameData requested");
            Career tb(rules);
            check(tb.deserialize(t.serialize()) && tb.trafficCaught(0, vi) && tb.trafficCaught(0, li) && tb.trafficCaught(0, ci),
                  "the save keeps the caught trucks (TRAFFIC_INFOS)");
        }

        // The save keeps the free-roam progress.
        const auto bytes = f.serialize();
        Career back(rules);
        std::string err;
        check(back.deserialize(bytes, &err) && back.state().fuelFound == f.state().fuelFound && back.vistasVisited() == f.vistasVisited() &&
                  back.vehicleLiveriesOwned() == f.vehicleLiveriesOwned() && back.state().challenges[ys][6].available == 1 &&
                  back.state().trips.size() == f.state().trips.size() && !back.state().trips.empty(),
              "save and reload keep barrels, vistas, liveries, discovered challenges and (port section) the travel history");
    }
    // P66: garage choices (vehicle livery +0xc / colour +0xd, the rider +0x33d0.., the pilot pair list).
    {
        Career g(rules);
        g.newGame();
        std::string ownedVeh;
        for (const auto& v : rules.vehicles)
            if (ownedVeh.empty() && g.vehicleOwned(v.label) && !g.vehicleLiveryChoices(v.label).empty()) ownedVeh = v.label;
        const auto ch = ownedVeh.empty() ? std::vector<int>{} : g.vehicleLiveryChoices(ownedVeh);
        check(!ownedVeh.empty() && ch.size() >= 3 && ch[0] == 0 && ch[1] == 1 && ch[2] == 2,
              "an owned vehicle has its ALWAYS livery (0x569b20(0)): texture liveries 0..2 (" + ownedVeh + ")");
        std::string unowned;
        for (const auto& v : rules.vehicles)
            if (unowned.empty() && !g.vehicleOwned(v.label) && v.cls >= 0) unowned = v.label;
        check(!unowned.empty() && g.vehicleLiveryChoices(unowned).empty(), "a vehicle not owned has no livery yet (" + unowned + ")");
        const PilotLook d = g.pilotLook();
        check(d.head == 1 && d.headLivery == 1 && d.chest == 1 && d.legs == 1 && d.skin == 1 && d.bodyLivery == 1,
              "rider defaults (0x42b420): HEAD1 (first with an owned livery) livery 1, chest 1, legs 1, skin 1");
        check(g.state().pilotPairs.size() == 21 && g.pilotTypeLivery("HEAD1") == 1 && g.pilotTypeLivery("HEAD2") == -5,
              "pilot pair list: 21 part types (0x56bc00); HEAD1 defaults to livery 1, HEAD2 (none owned) to none");
        g.setVehicleLivery(ownedVeh, 2), g.setVehicleColor(ownedVeh, 4);
        PilotLook p = d;
        p.head = 1, p.headLivery = 3, p.skin = 2;
        g.setPilotLook(p), g.setPilotTypeLivery("HEAD1", 3);
        Career gb(rules);
        const PilotLook q = gb.deserialize(g.serialize()) ? gb.pilotLook() : PilotLook{};
        check(gb.vehicleLivery(ownedVeh) == 2 && gb.vehicleColor(ownedVeh) == 4 && q.headLivery == 3 && q.skin == 2 && gb.pilotTypeLivery("HEAD1") == 3,
              "save and reload keep the livery, the colour, the rider and the HEAD1 pair");
    }
    {
        // P76: hub camps = the hub volume's CAMERA_CAREER node facing TARGET_CAREER (0x577630, TeleportToHub 0x5495a0);
        // expected values read from USA1.DPC with tools/scene_dump.py (p76-gameflow-anim.md).
        const std::vector<Hub> hubs = loadHubs(dataDir);
        struct Want {
            const char* id;
            float x, z, yawDeg;
        };
        const Want want[] = {{"YELLOWSTONE", -51259.0f, -30274.6f, -116.8f}, {"MOUNTAIN 2", -28193.6f, 46627.7f, -130.8f},
                             {"SALT LAKE", 50607.2f, -55460.9f, 74.6f}, {"CHELLY CANYON", 48329.4f, 29706.7f, 4.0f}};
        int good = 0;
        for (const Want& w : want)
            if (const Hub* h = findHub(hubs, w.id))
                good += std::fabs(h->camp.x - w.x) < 0.1f && std::fabs(h->camp.z - w.z) < 0.1f &&
                        std::fabs(h->camp.yaw * 57.29578f - w.yawDeg) < 0.1f;
        check(hubs.size() == 19 && good == 4, "hub camps at CAMERA_CAREER facing TARGET_CAREER (0x577630 / 0x5495a0): 4 of 4 sampled hubs");
    }
    // ---- PK21: the race editor's record rules and the CREATE trophy (0x4664d0 -> game +0x3438, Trophy.tsc "CREATE Race 1").
    {
        Career e(rules);
        e.newGame();
        int createId = -1;
        for (const auto& t : rules.trophies)
            if (t.verb == 8 && t.object == 9) createId = t.id;
        auto has = [&](int id) { return std::find(e.state().trophiesWon.begin(), e.state().trophiesWon.end(), id) != e.state().trophiesWon.end(); };
        e.checkTrophies();
        const bool before = createId >= 0 && !has(createId);
        auto& block = e.state().editedRaces;
        const bool empty = editor::freeSlots(block) == 10 && editor::firstFreeSlot(block) == 0 && editor::usedSlot(block, 0) == -1;
        EditedRace r;
        r.count = 3;
        r.pts[0].x = 0, r.pts[1].x = 400, r.pts[2].x = 800;
        const bool defaults = editor::newRaceDefaults(r) && r.type == 1 && r.loop == 0 && r.opt[3] == 1 && r.optionFlags() == 0x816;
        editor::setSlot(block, 3, r);
        e.state().racesCreated() += 1;
        e.checkTrophies();
        const bool slots = editor::freeSlots(block) == 9 && editor::firstFreeSlot(block) == 0 && editor::usedSlot(block, 0) == 3 &&
                           editor::slotOfName("Edited Race 04") == 3 && editor::slot(block, 3).pts[2].x == 800;
        Career back(rules);
        const bool trip = back.deserialize(e.serialize()) && back.state().editedRaces == block && back.state().racesCreated() == 1;
        check(before && empty && defaults && slots && has(createId) && trip,
              "race editor: new-race defaults (type 1, options 0x816), slots 0x569060 / 0x569200, races created + 1 -> achievement " +
                  std::to_string(createId) + " (CREATE Race 1), EDITEDRACE_INFOS round-trips");
        EditedRace loop = r;
        loop.count = 4, loop.pts[3] = loop.pts[0];
        EditedRace two = r;
        two.count = 2;
        check(editor::newRaceDefaults(loop) && loop.type == 4 && loop.loop == 1 && editor::newRaceDefaults(two) && two.type == 0 &&
                  editor::lengthKm(150499.0f) == 150 && !editor::tooLong(150499.0f) && editor::tooLong(150500.0f),
              "race editor: a closed loop -> type 4 + loop flag, 2 points -> type 0; km = trunc((m + 500) / 1000), invalid above 150");
    }
    // ---- PK25: save robustness (notes/pk25.md). Every file lives in a scratch folder under the temp directory.
    {
        namespace fs = std::filesystem;
        const fs::path dir = fs::temp_directory_path() / ("fuel-career-check-pk25-" + std::to_string(::getpid()));
        std::error_code ec;
        fs::create_directories(dir, ec);
        // The ModelOrder1C coder (pack 0x7597e0 / unpack 0x759a50): every byte value, runs, literal 0xfe, then the career.
        std::vector<uint8_t> raw;
        for (int k = 0; k < 256; ++k) raw.push_back(uint8_t(k));
        for (int k = 0; k < 300; ++k) raw.push_back(0xfe), raw.push_back(uint8_t(k * 7)), raw.push_back(0);
        raw.insert(raw.end(), 5000, 0x41);
        std::vector<uint8_t> back;
        const std::vector<uint8_t> blob = pcsave::pack(raw);
        uint32_t total = 0, n = 0;
        std::memcpy(&total, blob.data(), 4), std::memcpy(&n, blob.data() + 4, 4);
        check(pcsave::unpack(blob.data(), blob.size(), &back) && back == raw && n == raw.size() && total == blob.size() &&
                  total < raw.size() / 2,
              "ModelOrder1C: pack / unpack round trip (" + std::to_string(raw.size()) + " -> " + std::to_string(total) +
                  " bytes; header {coded + 9, raw size})");
        {
            // FUEL.exe's own encoder (0x7fe3a0 + 0x7fea50 under Ghidra's EmulatorHelper, read-only; notes/pk25.md) on this
            // synthetic 390-byte vector writes 414 coded bytes with save CRC 0xca4a14ee; its decoder (0x7feaf0 + 0x7fe540)
            // gives the 390 bytes back.
            std::vector<uint8_t> v;
            const char* a = "SECTION_SAVESTRUCT_INFOS";
            v.insert(v.end(), a, a + std::strlen(a) + 1);
            v.insert(v.end(), 40, 0);
            for (int k = 0; k < 60; ++k) v.push_back(0xfe), v.push_back(uint8_t(k * 37 + 11));
            for (int k = 0; k < 256; k += 3) v.push_back(uint8_t(k));
            const char* b = "SECTION_VEHICLES_INFOS MESH_ID_QUAD_ROAD_2 Yellowstone";
            v.insert(v.end(), b, b + std::strlen(b) + 1);
            for (int i = 0; i < 64; ++i) v.push_back(uint8_t((i * i) & 0xff));
            const std::vector<uint8_t> pb = pcsave::pack(v);
            uint32_t tot = 0;
            std::memcpy(&tot, pb.data(), 4);
            std::vector<uint8_t> vb;
            check(v.size() == 390 && tot == 414 + 9 && pcsave::saveCrc(pb.data() + 8, 414) == 0xca4a14eeu && pcsave::unpack(pb.data(), pb.size(), &vb) && vb == v,
                  "ModelOrder1C coder = FUEL.exe's (emulator vector: 390 bytes -> 414 coded, CRC 0xca4a14ee)");
        }
        const std::vector<uint8_t> stream = c.sections();
        std::vector<uint8_t> body;
        const bool made = pcsave::makeBody(stream, &body);
        check(made && body.size() == 200000 && body[0x30d3b] == 0x3d && pcsave::checkBody(body) == pcsave::Ok,
              "PC body (0x4ebea0): packed career at 0, '=' at 0x30d3b, CRC at 0x30d3c accepted by 0x4eb9c0");
        // The CRC itself: MSB-first CRC-32 of the lower-cased bytes (table 0xa01dc8, map 0xa018f0).
        const uint8_t abc[] = {'A', 'b', 'C'}, lower[] = {'a', 'b', 'c'};
        uint32_t ref = 0;  // bitwise reference of the same polynomial
        for (uint8_t ch : lower) {
            ref ^= uint32_t(ch) << 24;
            for (int k = 0; k < 8; ++k) ref = (ref & 0x80000000u) ? (ref << 1) ^ 0x04C11DB7u : ref << 1;
        }
        check(pcsave::saveCrc(abc, 3) == ref && pcsave::saveCrc(lower, 3) == ref, "save CRC: poly 0x04C11DB7 MSB first, case-folded");
        // A PC career.sav built here (0x840000 layout), imported read-only into a FUELPSAV file at an explicit path.
        const std::string pc = (dir / "pc-career.sav").string(), out = (dir / "imported.sav").string();
        bool imported = false;
        if (made && pcsave::writeFile(pc, body) == pcsave::Ok) {
            ::chmod(pc.c_str(), 0444);  // the import only reads it
            std::vector<uint8_t> got;
            Career im(rules);
            Career same(rules);  // the same career through the port's own file format
            imported = pcsave::importFile(pc, &got, nullptr) == pcsave::Ok && got == stream && im.deserializeSections(got) &&
                       same.deserialize(c.serialize()) && im.write(out) == 0 && im.serialize() == same.serialize() &&
                       im.state().fuel == c.state().fuel && im.stars() == c.stars();
        }
        check(imported, "PC career.sav import: read (0x840190 layout), '=' + CRC, unpack, sections applied; same career");
        // Corrupt PC saves: a flipped byte in the packed data, the marker gone, a short file.
        int flipped = -1, unmarked = -1, shortFile = -1;
        if (made) {
            std::vector<uint8_t> bad = body;
            bad[100] ^= 0x20;
            const std::string p1 = (dir / "pc-flip.sav").string(), p2 = (dir / "pc-nomark.sav").string(), p3 = (dir / "pc-short.sav").string();
            pcsave::writeFile(p1, bad);
            std::vector<uint8_t> got;
            flipped = pcsave::importFile(p1, &got);
            bad = body, bad[0x30d3b] = 0;
            {
                const uint32_t crc = pcsave::saveCrc(bad.data(), 0x30d3c);
                std::memcpy(&bad[0x30d3c], &crc, 4);
            }
            pcsave::writeFile(p2, bad);
            unmarked = pcsave::importFile(p2, &got);
            if (FILE* f = std::fopen(p3.c_str(), "wb")) {
                const uint32_t four = 4;
                std::fwrite(&four, 4, 1, f), std::fwrite(body.data(), 1, 1000, f), std::fclose(f);
            }
            shortFile = pcsave::importFile(p3, &got);
        }
        check(flipped == 0x17 && unmarked == 0x17 && shortFile == 8,
              "corrupt PC saves: CRC mismatch -> 0x17 (TT 9113), no '=' -> 0x17, short file -> 8 (0x840190)");
        // The port's own save: '=' + CRC after the payload; a damaged byte is corrupt (0x17), state untouched.
        const std::string own = (dir / "career.sav").string();
        const int w = c.write(own);
        std::vector<uint8_t> file;
        {
            std::ifstream f(own, std::ios::binary);
            file.assign(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
        }
        Career r(rules);
        const uint32_t f0 = r.state().fuel;
        int damaged = -1, cut = -1, empty = -1, missing = -1, legacy = -1;
        if (file.size() > 64) {
            std::vector<uint8_t> bad = file;
            bad[file.size() / 2] ^= 0x01;
            const std::string pb = (dir / "career-bad.sav").string(), pt = (dir / "career-cut.sav").string(),
                              pe = (dir / "career-empty.sav").string(), pl = (dir / "career-legacy.sav").string();
            std::ofstream(pb, std::ios::binary).write(reinterpret_cast<const char*>(bad.data()), std::streamsize(bad.size()));
            damaged = r.read(pb);
            std::ofstream(pt, std::ios::binary).write(reinterpret_cast<const char*>(file.data()), std::streamsize(file.size() - 9));
            cut = r.read(pt);
            std::ofstream(pe, std::ios::binary).flush();
            empty = r.read(pe);
            missing = r.read((dir / "none.sav").string());
            // A save written before PK25 (no trailer) still loads.
            std::ofstream(pl, std::ios::binary).write(reinterpret_cast<const char*>(file.data()), std::streamsize(file.size() - 5));
            Career l(rules);
            legacy = l.read(pl) == 0 && l.state().fuel == c.state().fuel ? 0 : 1;
        }
        check(w == 0 && file.size() > 5 && file[file.size() - 5] == 0x3d && damaged == 0x17 && cut == 0x17 && empty == 8 && missing == 7 &&
                  legacy == 0 && r.state().fuel == f0,
              "port save: '=' + CRC trailer; damaged -> 0x17, truncated -> 0x17, empty -> 8, missing -> 7, pre-PK25 save loads");
        // A read-only save: 0x840000 cannot open it for writing -> 9 (MCAlert TT 9024); the file is left as it was.
        int ro = -1;
        bool kept = false;
        if (::geteuid() != 0 && w == 0) {
            ::chmod(own.c_str(), 0444);
            ro = c.write(own);
            std::ifstream f(own, std::ios::binary);
            kept = std::vector<uint8_t>(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>()) == file;
            ::chmod(own.c_str(), 0644);
        }
        check(ro == 9 && kept, "read-only save: write refused with code 9 (TT 9024), file unchanged");
        for (const auto& e : fs::directory_iterator(dir, ec)) ::chmod(e.path().c_str(), 0644);
        fs::remove_all(dir, ec);
    }
    // ---- CUSTOM VEHICLES [port]: the packs' vehicles stay out of FUEL's sections (PORT_VEHICLES), so a save keeps FUEL's
    // VEHICLES_INFOS / SAVESTRUCT_INFOS bytes, loads without the section (PC import, pack removed) and FUEL's trophies.
    {
        std::vector<size_t> packs;
        for (size_t v = 0; v < rules.vehicles.size(); ++v)
            if (rules.vehicles[v].custom) packs.push_back(v);
        if (packs.empty()) {
            std::printf("  (no custom vehicle packs: PORT_VEHICLES checks skipped)\n");
        } else {
            bool tail = true;
            for (size_t v = 0; v < rules.vehicles.size(); ++v) tail = tail && (rules.vehicles[v].custom == (v >= rules.vehicles.size() - packs.size()));
            check(tail, "custom vehicles follow FUEL's " + std::to_string(rules.vehicles.size() - packs.size()) + " (table indices unchanged)");
            const auto upper = [](std::string t) {
                for (char& c : t) c = char(std::toupper(static_cast<unsigned char>(c)));
                return t;
            };
            for (const size_t pv : packs) {  // vehicles4: every pack, not only the first
            const std::string label = rules.vehicles[pv].label;
            {  // the listing and the unlock rule: a class, a price, a name text; one UnlockNeed VEHICLE ... HUB rule on a
               // known hub, locked in a new career unless that hub needs no stars
                const auto& rv = rules.vehicles[pv];
                int nrules = 0, hub = -1;
                for (const auto& u : rules.unlocks)
                    if (u.kind == "VEHICLE" && upper(u.target) == upper(rv.mesh)) ++nrules, hub = u.cond == "HUB" ? rules.hubIndex(u.arg) : -2;
                Career n(rules);
                n.newGame();
                n.checkUnlocks();
                const bool free0 = hub >= 0 && rules.hubs[size_t(hub)].starsNeeded == 0;
                std::printf("  pack %s: class %s, terrain %d, price %d, text %u, unlock hub %s\n", label.c_str(), rv.family.c_str(),
                            rv.terrain, rv.price, rv.nameText, hub >= 0 ? rules.hubs[size_t(hub)].name.c_str() : "?");
                check(rv.cls >= 0 && rv.price > 0 && rv.nameText >= 990000 && nrules == 1 && hub >= 0 &&
                          n.vehicleUnlocked(label) == free0,
                      label + ": listed (class, price, name text), one hub unlock rule, " + (free0 ? "unlocked" : "locked") + " in a new career");
            }
            auto section = [](const std::vector<uint8_t>& stream, const std::string& name) {
                const std::string tag = "SECTION_" + name;
                const auto it = std::search(stream.begin(), stream.end(), tag.begin(), tag.end());
                if (it == stream.end()) return std::vector<uint8_t>{};
                const std::string next = "SECTION_";
                const auto end = std::search(it + 1, stream.end(), next.begin(), next.end());
                return std::vector<uint8_t>(it, end);
            };
            Career a(rules);
            a.newGame();
            a.state().fuel = 5000000;
            const std::vector<uint8_t> before = a.sections();
            // FUEL's OWN Vehicle trophy progress (verb 2, object 1) before and after
            auto ownProgress = [&](const Career& k) {
                std::string out;
                for (const auto& t : rules.trophies)
                    if (t.verb == 2 && t.object == 1) {
                        std::string p;
                        k.trophyMet(t, &p);
                        out += p + ";";
                    }
                return out;
            };
            const std::string owned0 = ownProgress(a);
            // own it (unlock rule or not: the save handling is what is checked) and make it the current vehicle
            a.state().vehicles[pv].unlocked = 1;
            const auto bought = a.buyVehicle(label);
            const std::vector<uint8_t> after = a.sections();
            check(bought == Career::Bought && a.state().currentVehicle() == int(pv) && a.vehicleOwned(label), label + ": bought, current vehicle");
            check(section(before, "VEHICLES_INFOS") == section(after, "VEHICLES_INFOS") && !section(after, "VEHICLES_INFOS").empty(),
                  "VEHICLES_INFOS unchanged by owning " + label + " (FUEL's vehicles only)");
            {
                Career b(rules);
                std::string err;
                check(b.deserializeSections(after, &err) && b.vehicleOwned(label) && b.state().currentVehicle() == int(pv) &&
                          b.sections() == after,
                      "PORT_VEHICLES round trip: owned, current vehicle restored, identical stream");
            }
            {  // FUEL's SAVESTRUCT current vehicle never names a pack vehicle: drop PORT_VEHICLES and load
                std::vector<uint8_t> noPort = after;
                const std::string tag = "SECTION_PORT_VEHICLES";
                auto it = std::search(noPort.begin(), noPort.end(), tag.begin(), tag.end());
                bool cut = false;
                if (it != noPort.end()) {
                    const std::string next = "SECTION_";
                    const auto end = std::search(it + 1, noPort.end(), next.begin(), next.end());
                    noPort.erase(it, end);
                    cut = true;
                }
                Career b(rules);
                std::string err;
                const bool ok = b.deserializeSections(noPort, &err);
                check(cut && ok && !b.vehicleOwned(label) && b.state().currentVehicle() != int(pv) && b.state().fuel == a.state().fuel,
                      "without PORT_VEHICLES (PC import, pack removed): FUEL's state loads, " + label + " not owned, not current");
            }
            check(ownProgress(a) == owned0 && !owned0.empty(), "FUEL's OWN Vehicle trophies do not count " + label + " (" + owned0 + ")");
            }
        }
    }
    // ---- [DEVIATION upgrades U2] vehicle upgrades (notes upgrades-plan.md 3 / 6 / 7, upg2.md): prices by the vehicle's level,
    // the gates (owned, 20 / 45 stars), FUEL's spend path, buy-below and free equip, per vehicle, the session's stages, and
    // the save section PORT_UPGRADES (FUEL's sections untouched, the CRC over it, old / PC / odd saves all stock).
    {
        namespace fs = std::filesystem;
        auto stagesOf = [](const upg::Stages& st) { return upg::formatStages(st); };
        // Where a section starts in a stream ("SECTION_<name>\0") and its body size; npos when absent.
        auto sectionAt = [](const std::vector<uint8_t>& v, const std::string& name) {
            const std::string tag = "SECTION_" + name;
            for (size_t i = 0; i + tag.size() + 1 <= v.size(); ++i)
                if (std::memcmp(&v[i], tag.c_str(), tag.size() + 1) == 0) return i;
            return std::string::npos;
        };
        auto bodyAt = [](size_t sec, const std::string& name) { return ((sec + 8 + name.size() + 1) | 3) + 1; };  // the u32 size
        // The plan's numbers (3.1 / 3.2): levels from AddVehicleInfo, QUAD_ROAD_2 (level 3) engine 1,700 / 5,600 / 14,000,
        // tires and suspension 1,200 / 4,000 / 10,000; a whole level-5 vehicle 237,500.
        int classed = 0, levelled = 0, level5 = -1;
        for (size_t v = 0; v < rules.vehicles.size(); ++v)
            if (rules.vehicles[v].cls >= 0) {
                ++classed;
                levelled += rules.vehicles[v].level >= 1 && rules.vehicles[v].level <= 6;
                if (level5 < 0 && rules.vehicles[v].level == 5 && !rules.vehicles[v].custom) level5 = int(v);
            }
        Career u(rules);
        u.newGame();
        const std::string quad = "QUAD_ROAD_2";
        auto price3 = [&](const std::string& v, int cat) {
            return std::to_string(u.upgradePrice(v, cat, 1)) + "/" + std::to_string(u.upgradePrice(v, cat, 2) - u.upgradePrice(v, cat, 1)) + "/" +
                   std::to_string(u.upgradePrice(v, cat, 3) - u.upgradePrice(v, cat, 2));
        };
        int whole = 0;
        if (level5 >= 0)
            for (int k = 0; k < upg::kCategories; ++k) whole += u.upgradePrice(rules.vehicles[size_t(level5)].label, k, 3);
        check(classed == levelled && u.vehicleLevel("MUSCLE_ROAD_6") == 1 && u.vehicleLevel(quad) == 3 && u.vehicleLevel("BONUS_ROAD_3") == 6 &&
                  price3(quad, upg::Engine) == "1700/5600/14000" && price3(quad, upg::Tires) == "1200/4000/10000" &&
                  price3(quad, upg::Suspension) == "1200/4000/10000" && whole == 237500,
              "upgrades: levels (AddVehicleInfo 7th arg) on all " + std::to_string(classed) + " classed vehicles; " + quad + " engine " +
                  price3(quad, upg::Engine) + ", tires " + price3(quad, upg::Tires) + "; a level-5 vehicle in full " + std::to_string(whole));
        // Two owned starters (new career) and a vehicle not owned.
        std::string v1, v2, notOwned;
        for (const auto& rv : rules.vehicles) {
            if (rv.cls < 0 || rv.custom) continue;
            if (!u.vehicleOwned(rv.label)) {
                if (notOwned.empty()) notOwned = rv.label;
            } else if (v1.empty()) {
                v1 = rv.label;
            } else if (v2.empty()) {
                v2 = rv.label;
            }
        }
        bool allStock = true;
        for (const auto& rv : rules.vehicles) allStock = allStock && u.upgradesOwned(rv.label).stock() && u.upgradesEquipped(rv.label).stock();
        check(allStock && !v1.empty() && !v2.empty() && u.sessionUpgrades(v1, true).stock(),
              "upgrades: a new career is all stock (owned starters " + v1 + ", " + v2 + ")");
        const int e1 = u.upgradePrice(v1, upg::Engine, 1);
        u.state().fuel = 1000000;
        check(u.buyUpgrade(notOwned, upg::Engine, 1) == Career::UpgradeResult::VehicleNotOwned &&
                  u.buyUpgrade("TRAFFIC_TRUCK", upg::Engine, 1) == Career::UpgradeResult::NotUpgradable &&
                  u.buyUpgrade(v1, upg::Engine, 4) == Career::UpgradeResult::NotUpgradable && u.state().fuel == 1000000,
              "upgrades: stage 1 needs the vehicle owned (" + notOwned + "); traffic is never upgraded; nothing spent");
        u.state().fuel = uint32_t(e1 - 1);
        check(u.buyUpgrade(v1, upg::Engine, 1) == Career::UpgradeResult::TooExpensive && u.state().fuel == uint32_t(e1 - 1) &&
                  u.upgradesOwned(v1).stock(),
              "upgrades: engine stage 1 (" + std::to_string(e1) + ") with one fuel too few: too expensive, nothing spent");
        u.state().fuel = uint32_t(e1);
        check(u.buyUpgrade(v1, upg::Engine, 1) == Career::UpgradeResult::Bought && u.state().fuel == 0 && u.upgradesOwned(v1)[upg::Engine] == 1 &&
                  u.upgradesEquipped(v1)[upg::Engine] == 1 && u.upgradesSpent(v1) == uint32_t(e1) &&
                  u.buyUpgrade(v1, upg::Engine, 1) == Career::UpgradeResult::AlreadyOwned,
              "upgrades: engine stage 1 bought with the exact fuel (game +0x408 -= price, 0x4ce590), equipped; again: already owned");
        // Stars: none yet -> stage 2 locked; 20 stars (6 golds + a silver) open it, 45 (15 golds) stage 3.
        std::vector<std::string> careerRaces;
        for (const auto& h : rules.hubs)
            for (int idx : h.career) careerRaces.push_back(rules.missions[size_t(idx)].name);
        u.state().fuel = 1000000;
        const bool locked2 = u.buyUpgrade(v1, upg::Engine, 2) == Career::UpgradeResult::NeedsStars && u.state().fuel == 1000000;
        for (size_t k = 0; k < 6; ++k) u.finishEvent(careerRaces[k], true, 0, 1, 100.0);
        u.finishEvent(careerRaces[6], true, 1, 2, 100.0);
        u.state().fuel = 1000000;
        const int e2 = upg::price(u.vehicleLevel(v1), upg::Engine, 2);
        const bool bought2 = u.stars() == 20 && u.upgradePrice(v1, upg::Engine, 2) == e2 &&
                             u.buyUpgrade(v1, upg::Engine, 2) == Career::UpgradeResult::Bought && u.state().fuel == 1000000u - uint32_t(e2);
        check(locked2 && bought2 && u.buyUpgrade(v1, upg::Engine, 3) == Career::UpgradeResult::NeedsStars,
              "upgrades: stage 2 locked at 0 stars, bought at 20 for its own price " + std::to_string(e2) + "; stage 3 locked under 45");
        for (size_t k = 6; k < 15; ++k) u.finishEvent(careerRaces[k], true, 0, 1, 100.0);
        u.state().fuel = 1000000;
        const int t123 = upg::price(u.vehicleLevel(v1), upg::Tires, 1) + upg::price(u.vehicleLevel(v1), upg::Tires, 2) +
                         upg::price(u.vehicleLevel(v1), upg::Tires, 3);
        check(u.stars() == 45 && u.buyUpgrade(v1, upg::Tires, 3) == Career::UpgradeResult::Bought && u.state().fuel == 1000000u - uint32_t(t123) &&
                  u.upgradesOwned(v1)[upg::Tires] == 3 && u.upgradesEquipped(v1)[upg::Tires] == 3 && u.upgradesSpent(v1) == uint32_t(e1 + e2 + t123),
              "upgrades: tires stage 3 from stock at 45 stars buys stages 1-3 for their sum " + std::to_string(t123) + " and equips it");
        const uint32_t f1 = u.state().fuel;
        check(u.equipUpgrade(v1, upg::Tires, 1) && u.equipUpgrade(v1, upg::Engine, 0) && !u.equipUpgrade(v1, upg::Suspension, 1) &&
                  !u.equipUpgrade(v1, upg::Engine, 3) && u.state().fuel == f1 && stagesOf(u.upgradesOwned(v1)) == "2,0,3,0,0,0" &&
                  stagesOf(u.upgradesEquipped(v1)) == "0,0,1,0,0,0",
              "upgrades: equipping an owned stage (stock included) is free; a stage not owned cannot be equipped; no refund");
        check(u.upgradesOwned(v2).stock() && u.buyUpgrade(v2, upg::NitroTank, 1) == Career::UpgradeResult::Bought &&
                  stagesOf(u.upgradesOwned(v1)) == "2,0,3,0,0,0" && stagesOf(u.upgradesOwned(v2)) == "0,0,0,0,0,1",
              "upgrades: per vehicle (" + v2 + " stock, then its own nitro tank stage; " + v1 + " unchanged)");
        u.equipUpgrade(v1, upg::Engine, 2), u.equipUpgrade(v1, upg::Tires, 3);
        // Fuel cap: a purchase at 99,999,999 spends from the cap; rewards cap again (0x630dd0).
        u.state().fuel = 99999999;
        const int w1 = u.upgradePrice(v1, upg::Weight, 1);
        u.buyUpgrade(v1, upg::Weight, 1);
        const bool spentAtCap = u.state().fuel == 99999999u - uint32_t(w1);
        u.finishEvent(careerRaces[15], true, 0, 1, 100.0);
        check(spentAtCap && u.state().fuel == 99999999u, "upgrades: bought at the 99,999,999 cap (fuel - price), the next reward caps again");
        // The session's stages (execSession --upgrades): equipped when on; stock off, handed over or not owned; stages kept.
        const std::string eq = stagesOf(u.upgradesEquipped(v1));
        check(stagesOf(u.sessionUpgrades(v1, true)) == eq && eq == "2,0,3,1,0,0" && u.sessionUpgrades(v1, false).stock() &&
                  u.sessionUpgrades(v1, true, true).stock() && u.sessionUpgrades(notOwned, true).stock() &&
                  stagesOf(u.upgradesEquipped(v1)) == eq && stagesOf(u.sessionUpgrades(v1, true)) == eq,
              "upgrades: session --upgrades " + eq + "; upgrades = off, a challenge's own vehicle and a vehicle not owned: stock; stages kept");
        // Save: round trip, the section's layout, FUEL's sections unchanged.
        const std::vector<uint8_t> bytes = u.serialize(), stream = u.sections();
        Career b(rules);
        std::string err;
        check(b.deserialize(bytes, &err) && err.empty() && b.serialize() == bytes && stagesOf(b.upgradesOwned(v1)) == "2,0,3,1,0,0" &&
                  stagesOf(b.upgradesEquipped(v1)) == eq && b.upgradesSpent(v1) == u.upgradesSpent(v1) &&
                  stagesOf(b.upgradesOwned(v2)) == "0,0,0,0,0,1" && b.state().fuel == u.state().fuel,
              "upgrades: save round trip (owned, equipped, spent per vehicle; same bytes)");
        const size_t sec = sectionAt(stream, "PORT_UPGRADES");
        uint32_t body = 0, fmt = 0, flags = 0, n = 0;
        if (sec != std::string::npos && bodyAt(sec, "PORT_UPGRADES") + 16 <= stream.size()) {
            const size_t at = bodyAt(sec, "PORT_UPGRADES");
            std::memcpy(&body, &stream[at], 4), std::memcpy(&fmt, &stream[at + 4], 4), std::memcpy(&flags, &stream[at + 8], 4), std::memcpy(&n, &stream[at + 12], 4);
        }
        check(sec != std::string::npos && n == 2 && fmt == 1 && flags == 0 && body == 12 + 20 * n && bodyAt(sec, "PORT_UPGRADES") + 4 + body == stream.size(),
              "upgrades: SECTION_PORT_UPGRADES last, padded to 4, body {format 1, flags 0, n 2, 2 x 20 bytes}");
        {
            Career z(rules);
            z.deserialize(bytes);
            z.state().upgrades.assign(z.state().vehicles.size(), {});
            const std::vector<uint8_t> zs = z.sections();
            const size_t zsec = sectionAt(zs, "PORT_UPGRADES");
            check(zsec == sec && std::equal(stream.begin(), stream.begin() + long(sec), zs.begin()) && zs.size() == bodyAt(sec, "PORT_UPGRADES") + 4 + 12,
                  "upgrades: FUEL's sections (and the other port sections) byte-identical with and without stages; no stages = n 0");
        }
        {   // The PK25 trailer's CRC covers the section: one flipped byte in its body is a corrupt save (0x17), state untouched.
            const fs::path dir = fs::temp_directory_path() / ("fuel-career-check-upg-" + std::to_string(::getpid()));
            std::error_code ec;
            fs::create_directories(dir, ec);
            const std::string own = (dir / "career.sav").string(), bad = (dir / "career-bad.sav").string();
            const int w = u.write(own);
            std::vector<uint8_t> file;
            {
                std::ifstream f(own, std::ios::binary);
                file.assign(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
            }
            int flipped = -1;
            if (w == 0 && file.size() == bytes.size()) {
                std::vector<uint8_t> x = file;
                x[16 + bodyAt(sec, "PORT_UPGRADES") + 4 + 12 + 4] ^= 0x02;  // the first record's owned[0]
                std::ofstream(bad, std::ios::binary).write(reinterpret_cast<const char*>(x.data()), std::streamsize(x.size()));
                Career r(rules);
                flipped = r.read(bad);
                flipped = flipped == 0x17 && r.upgradesOwned(v1).stock() ? flipped : -flipped;
            }
            check(w == 0 && file == bytes && flipped == 0x17, "upgrades: the save CRC covers PORT_UPGRADES (a flipped stage byte -> 0x17, TT 9113)");
            // PC career.sav import: FUEL's sections only (a PC save has no port sections) -> all stock, the same fuel.
            std::vector<uint8_t> fuelOnly(stream.begin(), stream.begin() + long(sectionAt(stream, "PORT_TROPHIES")));
            std::vector<uint8_t> pcBody, got;
            const std::string pc = (dir / "pc-career.sav").string();
            bool pcOk = pcsave::makeBody(fuelOnly, &pcBody) && pcsave::writeFile(pc, pcBody) == pcsave::Ok && pcsave::importFile(pc, &got) == pcsave::Ok;
            Career im(rules);
            pcOk = pcOk && im.deserializeSections(got) && im.state().fuel == u.state().fuel;
            bool imStock = true;
            for (const auto& rv : rules.vehicles) imStock = imStock && im.upgradesOwned(rv.label).stock() && im.upgradesEquipped(rv.label).stock();
            check(pcOk && imStock && im.serialize().size() > 0, "upgrades: a PC career.sav import (no port sections) loads all stock, same fuel");
            fs::remove_all(dir, ec);
        }
        {   // An older port build: it reads the file without PORT_UPGRADES and re-saves without it; the fuel stays spent.
            std::vector<uint8_t> old(stream.begin(), stream.begin() + long(sec));
            Career o(rules);
            std::string e;
            const bool ok = o.deserializeSections(old, &e);
            bool st = true;
            for (const auto& rv : rules.vehicles) st = st && o.upgradesOwned(rv.label).stock();
            check(ok && e.empty() && st && o.state().fuel == u.state().fuel && o.stars() == u.stars(),
                  "upgrades: a save without the section (old saves, an older build's re-save) loads all stock; the fuel stays spent");
        }
        {   // A body of the wrong size or format: all stock and a note, not the corrupt alert. Clamps and unknown crcs.
            auto withBody = [&](const std::vector<uint8_t>& payload) {
                std::vector<uint8_t> v(stream.begin(), stream.begin() + long(bodyAt(sec, "PORT_UPGRADES")));
                const uint32_t sz = uint32_t(payload.size());
                v.insert(v.end(), reinterpret_cast<const uint8_t*>(&sz), reinterpret_cast<const uint8_t*>(&sz) + 4);
                v.insert(v.end(), payload.begin(), payload.end());
                return v;
            };
            auto u32 = [](std::vector<uint8_t>& p, uint32_t x) { p.insert(p.end(), reinterpret_cast<const uint8_t*>(&x), reinterpret_cast<const uint8_t*>(&x) + 4); };
            const uint32_t crc1 = u.state().vehicles[size_t(rules.vehicleIndex(v1))].crc;
            std::vector<uint8_t> odd;  // n = 2 but one record
            u32(odd, 1), u32(odd, 0), u32(odd, 2), u32(odd, crc1);
            odd.insert(odd.end(), {1, 1, 1, 0, 0, 0, 1, 1, 1, 0, 0, 0});
            u32(odd, 100);
            std::vector<uint8_t> fmt2 = odd;
            fmt2[0] = 2, fmt2[8] = 1;  // format 2, n 1: a format this build does not know
            std::vector<uint8_t> clamp;  // owned 7 -> 3, equipped 9 -> owned, equipped 2 > owned 1 -> 1, an unknown crc dropped
            u32(clamp, 1), u32(clamp, 0), u32(clamp, 2), u32(clamp, crc1);
            clamp.insert(clamp.end(), {7, 1, 0, 0, 0, 0, 9, 2, 0, 0, 0, 0});
            u32(clamp, 200000000), u32(clamp, 0x12345678u);
            clamp.insert(clamp.end(), {3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3, 3});
            u32(clamp, 5);
            Career a1(rules), a2(rules), a3(rules);
            std::string n1, n2, n3;
            const bool ok1 = a1.deserializeSections(withBody(odd), &n1), ok2 = a2.deserializeSections(withBody(fmt2), &n2),
                       ok3 = a3.deserializeSections(withBody(clamp), &n3);
            check(ok1 && ok2 && a1.upgradesOwned(v1).stock() && a2.upgradesOwned(v1).stock() && !n1.empty() && !n2.empty() &&
                      a1.state().fuel == u.state().fuel,
                  "upgrades: a short or odd PORT_UPGRADES body loads all stock with a note (" + n1 + "), not the corrupt alert");
            check(ok3 && stagesOf(a3.upgradesOwned(v1)) == "3,1,0,0,0,0" && stagesOf(a3.upgradesEquipped(v1)) == "3,1,0,0,0,0" &&
                      a3.upgradesSpent(v1) == 99999999u && !n3.empty(),
                  "upgrades: stages clamp to 0..3, equipped <= owned, spent <= 99,999,999, an unknown vehicle crc is dropped (" + n3 + ")");
        }
        // CUSTOM VEHICLES: a pack's vehicle keeps its stages through PORT_UPGRADES (by crc(mesh), like PORT_VEHICLES).
        std::string pack;
        for (const auto& rv : rules.vehicles)
            if (rv.custom && rv.cls >= 0 && pack.empty()) pack = rv.label;
        if (pack.empty()) {
            std::printf("  (no custom vehicle packs: the upgrades pack check skipped)\n");
        } else {
            Career pc(rules);
            pc.newGame();
            pc.state().vehicles[size_t(rules.vehicleIndex(pack))].unlocked = 1;
            pc.state().fuel = 5000000;
            pc.buyVehicle(pack);
            const auto r1 = pc.buyUpgrade(pack, upg::Suspension, 1);
            Career pb(rules);
            check(r1 == Career::UpgradeResult::Bought && pb.deserialize(pc.serialize()) && stagesOf(pb.upgradesOwned(pack)) == "0,1,0,0,0,0" &&
                      pb.vehicleOwned(pack) && stagesOf(pb.sessionUpgrades(pack, true)) == "0,1,0,0,0,0",
                  "upgrades: custom vehicle " + pack + " (level " + std::to_string(pc.vehicleLevel(pack)) + "): suspension stage 1 bought, saved, reloaded");
        }
    }
    // ---- [DEVIATION dev mode] (notes devmode.md): the overlay (every classed vehicle unlocked and owned, every stage owned:
    // no gate, no fuel), its store (equips, a livery / colour of a vehicle not really owned) and PORT_DEVMODE; off = the
    // real career exactly; a dev-mode save identical to the real one in FUEL's sections and PORT_UPGRADES, loading cleanly
    // with dev mode off; the PC import, the CRC, odd bodies.
    {
        namespace fs = std::filesystem;
        auto stagesOf = [](const upg::Stages& st) { return upg::formatStages(st); };
        auto sectionAt = [](const std::vector<uint8_t>& v, const std::string& name) {
            const std::string tag = "SECTION_" + name;
            for (size_t i = 0; i + tag.size() + 1 <= v.size(); ++i)
                if (std::memcmp(&v[i], tag.c_str(), tag.size() + 1) == 0) return i;
            return std::string::npos;
        };
        auto bodyAt = [](size_t sec, const std::string& name) { return ((sec + 8 + name.size() + 1) | 3) + 1; };
        // A real career: a new game with fuel, its first owned vehicle with stages bought, some medals.
        Career d(rules);
        d.newGame();
        std::vector<std::string> careerRaces;
        for (const auto& h : rules.hubs)
            for (int idx : h.career) careerRaces.push_back(rules.missions[size_t(idx)].name);
        for (size_t k = 0; k < 3; ++k) d.finishEvent(careerRaces[k], true, 0, 1, 100.0);  // 9 stars: under every stage-2 gate
        d.state().fuel = 30000;
        std::string own, notOwned, pack, traffic;
        for (const auto& rv : rules.vehicles) {
            if (rv.cls < 0) {
                if (traffic.empty()) traffic = rv.label;
                continue;
            }
            if (rv.custom) {
                if (pack.empty() && !d.vehicleOwned(rv.label)) pack = rv.label;
            } else if (d.vehicleOwned(rv.label)) {
                if (own.empty()) own = rv.label;
            } else if (notOwned.empty() && rv.price > 0) {
                notOwned = rv.label;
            }
        }
        d.buyUpgrade(own, upg::Engine, 1), d.buyUpgrade(own, upg::Suspension, 1), d.equipUpgrade(own, upg::Engine, 0);
        const CareerState real = d.state();
        const std::vector<uint8_t> realBytes = d.serialize(), realStream = d.sections();
        const std::string ownOwned = stagesOf(d.upgradesOwned(own)), ownEq = stagesOf(d.upgradesEquipped(own));
        int classed = 0, ownedReal = 0, trophies0 = int(d.state().trophiesWon.size());
        for (const auto& rv : rules.vehicles) classed += rv.cls >= 0, ownedReal += rv.cls >= 0 && d.vehicleOwned(rv.label);
        check(!d.devMode() && !own.empty() && !notOwned.empty() && !traffic.empty() && ownOwned == "1,1,0,0,0,0" && ownEq == "0,1,0,0,0,0" && d.stars() == 9,
              "dev mode: off in a new Career; the real career owns " + std::to_string(ownedReal) + " of " + std::to_string(classed) + " classed vehicles (" + own +
                  " owned " + ownOwned + " equipped " + ownEq + "), " + notOwned + " not, 9 stars");
        d.setDevMode(true);
        int ownedDev = 0, unlockedDev = 0;
        bool stagesAll = true, locks = true;
        for (const auto& rv : rules.vehicles) {
            if (rv.cls < 0) continue;
            ownedDev += d.vehicleOwned(rv.label), unlockedDev += d.vehicleUnlocked(rv.label);
            stagesAll = stagesAll && stagesOf(d.upgradesOwned(rv.label)) == "3,3,3,3,3,3";
            for (int cat = 0; cat < upg::kCategories; ++cat)
                for (int stg = 1; stg <= upg::kMaxStage; ++stg)
                    locks = locks && d.upgradeLock(rv.label, cat, stg) == Career::UpgradeResult::AlreadyOwned && d.upgradePrice(rv.label, cat, stg) == 0;
        }
        const int ti = rules.vehicleIndex(traffic);
        check(ownedDev == classed && unlockedDev == classed && (pack.empty() || d.vehicleOwned(pack)) &&
                  d.vehicleOwned(traffic) == (real.vehicles[size_t(ti)].owned != 0) && !d.upgradable(traffic),
              "dev mode on: all " + std::to_string(classed) + " classed vehicles unlocked and owned (FUEL's, the gifts" + (pack.empty() ? "" : ", " + pack) +
                  "); " + traffic + " (no class) as the real career has it");
        check(stagesAll && locks, "dev mode on: every stage of every classed vehicle owned (upgradeLock: already owned; price 0) at 9 stars");
        const uint32_t fuel0 = d.state().fuel;
        check(d.buyVehicle(notOwned) == Career::AlreadyOwned && d.buyUpgrade(own, upg::Engine, 3) == Career::UpgradeResult::AlreadyOwned &&
                  d.buyUpgrade(notOwned, upg::Tires, 2) == Career::UpgradeResult::AlreadyOwned && d.state().fuel == fuel0,
              "dev mode on: buyVehicle / buyUpgrade answer already owned; no fuel spent (" + std::to_string(fuel0) + ")");
        check(stagesOf(d.upgradesEquipped(own)) == ownEq && stagesOf(d.upgradesEquipped(notOwned)) == "0,0,0,0,0,0" && !d.devStages(own),
              "dev mode on, nothing equipped yet: the real equipped stages shown (" + ownEq + ")");
        check(d.equipUpgrade(own, upg::Engine, 3) && d.equipUpgrade(notOwned, upg::Tires, 2) && d.equipUpgrade(notOwned, upg::NitroTank, 3) &&
                  d.equipUpgrade(notOwned, upg::Tires, 1) && stagesOf(d.upgradesEquipped(own)) == "3,1,0,0,0,0" &&
                  stagesOf(d.upgradesEquipped(notOwned)) == "0,0,1,0,0,3" && d.state().fuel == fuel0,
              "dev mode on: any stage equipped for free (" + own + " engine 3 over its real suspension 1; " + notOwned + " tires 2 then 1, tank 3)");
        const int oi = rules.vehicleIndex(own), ni = rules.vehicleIndex(notOwned);
        check(std::memcmp(d.state().upgrades[size_t(oi)].owned, real.upgrades[size_t(oi)].owned, upg::kCategories) == 0 &&
                  std::memcmp(d.state().upgrades[size_t(oi)].equipped, real.upgrades[size_t(oi)].equipped, upg::kCategories) == 0 &&
                  !d.state().upgrades[size_t(ni)].any() && d.state().vehicles[size_t(ni)].owned == 0 && d.state().vehicles[size_t(ni)].unlocked == real.vehicles[size_t(ni)].unlocked,
              "dev mode on: the real records untouched (" + own + " stages, " + notOwned + " not owned, stock)");
        check(stagesOf(d.sessionUpgrades(notOwned, true)) == "0,0,1,0,0,3" && d.sessionUpgrades(notOwned, false).stock() && d.sessionUpgrades(notOwned, true, true).stock(),
              "dev mode on: the session's --upgrades = the dev stages; upgrades = off or a handed-over vehicle: stock");
        // Livery / colour: a vehicle only the overlay owns keeps them in the dev store; a really owned one in its record.
        const uint8_t col0 = real.vehicles[size_t(ni)].color, liv0 = real.vehicles[size_t(ni)].livery;
        const auto livs = d.vehicleLiveryChoices(notOwned);
        d.setVehicleColor(notOwned, 7), d.setVehicleLivery(notOwned, livs.empty() ? 0 : livs.back());
        d.setVehicleColor(own, 4);
        check(!livs.empty() && d.vehicleColor(notOwned) == 7 && d.vehicleLivery(notOwned) == (livs.empty() ? 0 : livs.back()) &&
                  d.state().vehicles[size_t(ni)].color == col0 && d.state().vehicles[size_t(ni)].livery == liv0 && d.vehicleColor(own) == 4 &&
                  d.state().vehicles[size_t(oi)].color == 4,
              "dev mode on: " + notOwned + "'s ALWAYS liveries offered (" + std::to_string(livs.size()) + "); its colour / livery in the dev store (record +0xc / +0xd "
              "unchanged); " + own + "'s colour in its record (really owned)");
        d.setVehicleColor(own, real.vehicles[size_t(oi)].color);  // back: the save comparison below is against `real`
        const float t0 = d.state().vehicles[size_t(ni)].f10, to0 = d.state().vehicles[size_t(oi)].f10;
        d.addVehicleTime(notOwned, 60.0f), d.addVehicleTime(own, 30.0f);
        check(d.state().vehicles[size_t(ni)].f10 == t0 && d.state().vehicles[size_t(oi)].f10 == to0 + 30.0f,
              "dev mode on: time driven in " + notOwned + " not added to its record (+0x10); " + own + " (really owned) counts");
        d.state().vehicles[size_t(oi)].f10 = to0;
        // Real progress still counts in dev mode: a race won, its gift (if any) really given, trophies on the real records.
        d.checkTrophies();
        check(int(d.state().trophiesWon.size()) == trophies0 && d.stars() == 9 && d.hubUnlocked(0) == (real.hubs[0].unlocked != 0),
              "dev mode on: no trophy from the overlay (OWN VEHICLE reads the real records), stars and hubs the real ones");
        // ---- the save in dev mode: FUEL's sections, PORT_VEHICLES and PORT_UPGRADES as the real career's; PORT_DEVMODE last
        const std::vector<uint8_t> devBytes = d.serialize(), devStream = d.sections();
        const size_t dsec = sectionAt(devStream, "PORT_DEVMODE");
        uint32_t body = 0, fmt = 0, flg = 0, n = 0;
        if (dsec != std::string::npos && bodyAt(dsec, "PORT_DEVMODE") + 16 <= devStream.size()) {
            const size_t at = bodyAt(dsec, "PORT_DEVMODE");
            std::memcpy(&body, &devStream[at], 4), std::memcpy(&fmt, &devStream[at + 4], 4), std::memcpy(&flg, &devStream[at + 8], 4), std::memcpy(&n, &devStream[at + 12], 4);
        }
        check(dsec == realStream.size() && std::equal(realStream.begin(), realStream.end(), devStream.begin()) && sectionAt(realStream, "PORT_DEVMODE") == std::string::npos,
              "dev-mode save: every section before PORT_DEVMODE byte for byte the real career's (FUEL's, PORT_VEHICLES, PORT_UPGRADES); PORT_DEVMODE appended last");
        check(n == 2 && fmt == 1 && flg == 0 && body == 12 + 16 * n && bodyAt(dsec, "PORT_DEVMODE") + 4 + body == devStream.size(),
              "PORT_DEVMODE: padded to 4, body {format 1, flags 0, n 2, 2 x 16 bytes}");
        {
            Career e(rules);  // dev mode off (the default)
            std::string err;
            const bool ok = e.deserialize(devBytes, &err);
            bool same = ok && err.empty();
            for (const auto& rv : rules.vehicles)
                same = same && e.vehicleOwned(rv.label) == bool(real.vehicles[size_t(rules.vehicleIndex(rv.label))].owned) &&
                       e.vehicleUnlocked(rv.label) == bool(real.vehicles[size_t(rules.vehicleIndex(rv.label))].unlocked);
            check(same && stagesOf(e.upgradesOwned(own)) == ownOwned && stagesOf(e.upgradesEquipped(own)) == ownEq && e.vehicleColor(notOwned) == col0 &&
                      e.state().fuel == real.fuel && e.stars() == 9 && e.sections() == devStream && e.serialize() == devBytes,
                  "the dev-mode save loads cleanly with dev mode off: the real career (ownership, stages, colours, fuel, stars); same bytes again");
            e.setDevMode(true);
            check(stagesOf(e.upgradesEquipped(notOwned)) == "0,0,1,0,0,3" && stagesOf(e.upgradesEquipped(own)) == "3,1,0,0,0,0" && e.vehicleColor(notOwned) == 7,
                  "loaded, dev mode on again: the dev choices are back (PORT_DEVMODE round trip)");
        }
        d.setDevMode(false);
        bool offSame = true;
        for (const auto& rv : rules.vehicles) {
            const int i = rules.vehicleIndex(rv.label);
            offSame = offSame && d.vehicleOwned(rv.label) == bool(real.vehicles[size_t(i)].owned) && d.vehicleUnlocked(rv.label) == bool(real.vehicles[size_t(i)].unlocked);
        }
        check(offSame && stagesOf(d.upgradesOwned(own)) == ownOwned && stagesOf(d.upgradesEquipped(own)) == ownEq && d.upgradesOwned(notOwned).stock() &&
                  d.upgradeLock(notOwned, upg::Engine, 1) == Career::UpgradeResult::VehicleNotOwned &&
                  d.upgradeLock(own, upg::Engine, 2) == Career::UpgradeResult::NeedsStars && d.vehicleColor(notOwned) == col0 &&
                  !d.equipUpgrade(own, upg::Engine, 3) && d.state().fuel == fuel0,
              "dev mode off: the real career exactly (" + std::to_string(ownedReal) + " owned, " + own + " " + ownOwned + " / " + ownEq + ", gates back), nothing lost or gained");
        {   // A career that never used dev mode: no PORT_DEVMODE (the save as before this package).
            Career z(rules);
            z.deserialize(devBytes);
            z.state().dev.assign(z.state().vehicles.size(), {});
            check(z.sections() == realStream && z.serialize() == realBytes, "no dev choices: no PORT_DEVMODE, the save byte for byte as without dev mode");
        }
        {   // The PK25 trailer's CRC covers PORT_DEVMODE; a PC import (FUEL's sections only) has no dev choices.
            const fs::path dir = fs::temp_directory_path() / ("fuel-career-check-dev-" + std::to_string(::getpid()));
            std::error_code ec;
            fs::create_directories(dir, ec);
            const std::string file = (dir / "career.sav").string(), bad = (dir / "career-bad.sav").string();
            d.setDevMode(true);
            const int w = d.write(file);
            d.setDevMode(false);
            std::vector<uint8_t> bytes;
            {
                std::ifstream f(file, std::ios::binary);
                bytes.assign(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
            }
            int flipped = -1;
            if (w == 0 && bytes.size() == devBytes.size()) {
                std::vector<uint8_t> x = bytes;
                x[16 + bodyAt(dsec, "PORT_DEVMODE") + 4 + 12 + 4 + 4] ^= 0x01;  // the first record's equipped[0]
                std::ofstream(bad, std::ios::binary).write(reinterpret_cast<const char*>(x.data()), std::streamsize(x.size()));
                Career r(rules);
                flipped = r.read(bad);
            }
            check(w == 0 && bytes == devBytes && flipped == 0x17, "the save CRC covers PORT_DEVMODE (a flipped byte -> 0x17, TT 9113); written in dev mode = the same bytes");
            std::vector<uint8_t> fuelOnly(devStream.begin(), devStream.begin() + long(sectionAt(devStream, "PORT_TROPHIES")));
            std::vector<uint8_t> pcBody, got;
            const std::string pc = (dir / "pc-career.sav").string();
            bool pcOk = pcsave::makeBody(fuelOnly, &pcBody) && pcsave::writeFile(pc, pcBody) == pcsave::Ok && pcsave::importFile(pc, &got) == pcsave::Ok;
            Career im(rules);
            pcOk = pcOk && got == fuelOnly && im.deserializeSections(got) && !im.devStages(notOwned) && sectionAt(im.sections(), "PORT_DEVMODE") == std::string::npos;
            im.setDevMode(true);
            check(pcOk && im.upgradesEquipped(notOwned).stock() && im.vehicleOwned(notOwned),
                  "a PC career.sav import (FUEL's sections only): no dev choices, no PORT_DEVMODE; the overlay still applies when on");
            fs::remove_all(dir, ec);
        }
        {   // Odd bodies: no dev choices and a note (not the corrupt alert); unknown crcs and unclassed vehicles dropped; clamps.
            auto withBody = [&](const std::vector<uint8_t>& payload) {
                std::vector<uint8_t> v(devStream.begin(), devStream.begin() + long(bodyAt(dsec, "PORT_DEVMODE")));
                const uint32_t sz = uint32_t(payload.size());
                v.insert(v.end(), reinterpret_cast<const uint8_t*>(&sz), reinterpret_cast<const uint8_t*>(&sz) + 4);
                v.insert(v.end(), payload.begin(), payload.end());
                return v;
            };
            auto u32 = [](std::vector<uint8_t>& p, uint32_t x) { p.insert(p.end(), reinterpret_cast<const uint8_t*>(&x), reinterpret_cast<const uint8_t*>(&x) + 4); };
            const uint32_t crcN = real.vehicles[size_t(ni)].crc, crcT = real.vehicles[size_t(ti)].crc;
            std::vector<uint8_t> odd;  // n 2, one record
            u32(odd, 1), u32(odd, 0), u32(odd, 2), u32(odd, crcN);
            odd.insert(odd.end(), {1, 0, 0, 0, 3, 3, 3, 3, 3, 3, 0, 0});
            std::vector<uint8_t> clamp;  // stage 9 -> 3; an unknown crc and a traffic vehicle dropped
            u32(clamp, 1), u32(clamp, 0), u32(clamp, 3), u32(clamp, crcN);
            clamp.insert(clamp.end(), {3, 2, 5, 0, 9, 1, 0, 0, 0, 0, 0, 0});
            u32(clamp, 0x12345678u);
            clamp.insert(clamp.end(), {1, 0, 0, 0, 3, 3, 3, 3, 3, 3, 0, 0});
            u32(clamp, crcT);
            clamp.insert(clamp.end(), {1, 0, 0, 0, 3, 3, 3, 3, 3, 3, 0, 0});
            Career a1(rules), a3(rules);
            std::string n1, n3;
            const bool ok1 = a1.deserializeSections(withBody(odd), &n1), ok3 = a3.deserializeSections(withBody(clamp), &n3);
            a1.setDevMode(true), a3.setDevMode(true);
            check(ok1 && !n1.empty() && !a1.devStages(notOwned) && a1.state().fuel == real.fuel && stagesOf(a1.upgradesOwned(own)) == "3,3,3,3,3,3",
                  "an odd PORT_DEVMODE body: no dev choices and a note (" + n1 + "), not the corrupt alert; the overlay still on");
            check(ok3 && !n3.empty() && stagesOf(a3.upgradesEquipped(notOwned)) == "3,1,0,0,0,0" && a3.vehicleColor(notOwned) == 5 && a3.vehicleLivery(notOwned) == 2,
                  "PORT_DEVMODE: stages clamp to 0..3, an unknown crc and a vehicle without a class are dropped (" + n3 + ")");
        }
        {   // A pack's vehicle: owned in the overlay, its dev stages saved by crc(mesh).
            if (pack.empty()) {
                std::printf("  (no custom vehicle packs: the dev mode pack check skipped)\n");
            } else {
                Career pc(rules);
                pc.newGame();
                pc.setDevMode(true);
                const bool eq = pc.vehicleOwned(pack) && pc.equipUpgrade(pack, upg::Suspension, 3);
                Career pb(rules);
                const bool ld = pb.deserialize(pc.serialize());
                const bool offOk = !pb.vehicleOwned(pack) && pb.upgradesEquipped(pack).stock();
                pb.setDevMode(true);
                check(eq && ld && offOk && stagesOf(pb.upgradesEquipped(pack)) == "0,3,0,0,0,0",
                      "dev mode: custom vehicle " + pack + " owned in the overlay, suspension stage 3 equipped, saved, reloaded (off: not owned, stock)");
            }
        }
    }
    std::printf("career check: %d/%d ok -> %s\n", checks - fails, checks, fails ? "FAILED" : "ok");
    return fails ? 1 : 0;
}

}  // namespace fuel
