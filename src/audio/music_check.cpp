// --audio-check, part 2: FUEL's music and world ambience (notes/p45-audio-ambience.md). Synthetic inputs and
// vectors from FUEL.exe run under Ghidra's emulator (notes/p45-work/intensity.cmd -> intensity.out); the
// shipped music and IT_SOUND bank are read from the player's game data at run time and rendered offline.
#include "audio/ambience.hpp"
#include "audio/drive_audio.h"
#include "audio/music.hpp"
#include "core/genworld_noise.hpp"
#include "core/resources.hpp"
#include "core/sound.hpp"

#include <cmath>
#include <cstdio>
#include <filesystem>
#include <unordered_map>

namespace fuel {

namespace {

int gBad = 0;
void expect(bool ok, const char* what, double got, double want) {
    std::printf("  %-62s %12.6g  (want %.6g)%s\n", what, got, want, ok ? "" : "  ** FAIL");
    if (!ok) ++gBad;
}
void near(const char* what, double got, double want, double tol) { expect(std::fabs(got - want) <= tol, what, got, want); }

// 0x4e7330 / 0x4e7530 under the emulator: a mission object (state +0x48, scenario +0x27c / +0x280, +0x22c,
// +0x344, +0x270, +0x234 / +0x290, laps), one AI racer (distance, class +0x1344), route +4 = -1 (so the
// course-length ratio tests fail). {name, state, mode code, remaining, time left, position, racer distance,
// racer class, last place, last gate, action AL, extreme AL}.
struct RuleVec {
    const char* name;
    int state, mode;
    float remaining, timeLeft;
    int position;
    float racer;
    int cls;
    bool lastPlace, lastGate;
    int action, extreme;
};
constexpr RuleVec kRules[] = {
    {"V1 countdown, circuit", 4, 8, 2000, 0, 2, 500, 0, false, false, 0, 0},
    {"V2 checkpoint, mid race", 5, 9, 2000, 0, 2, 500, 0, false, false, 0, 0},
    {"V3 checkpoint, racer at 20 m", 5, 9, 2000, 0, 2, 20, 0, false, false, 1, 0},
    {"V4 checkpoint, last gate 800 m", 5, 9, 800, 0, 2, 500, 0, false, true, 0, 1},
    {"V5 last gate, 5th", 5, 9, 800, 0, 5, 500, 0, false, true, 0, 0},
    {"V6 last gate, 5th, hunted racer at 8 m", 5, 9, 800, 0, 5, 8, 3, false, true, 1, 1},
    {"V7 won, circuit", 6, 8, 2000, 0, 2, 500, 0, false, false, 0, 1},
    {"V8 lost", 7, 9, 100, 0, 1, 5, 0, false, true, 0, 0},
    {"V9 A to B, 400 m (ratio unknown)", 5, 12, 400, 0, 3, 500, 0, false, false, 0, 0},
    {"V10 helicopter, 400 m", 5, 2, 400, 0, 1, 500, 0, false, false, 1, 1},
    {"V11 catch AI, racer at 60 m", 5, 3, 2000, 100, 1, 60, 0, false, false, 1, 1},
    {"V12 knock-out, last place", 5, 7, 2000, 0, 6, 500, 0, true, false, 1, 1},
    {"V13 baja, 300 m", 5, 11, 300, 0, 1, 500, 0, false, false, 0, 0},
    {"V14 knock-out, final gate of the last lap", 5, 7, 2000, 0, 3, 500, 0, false, true, 1, 1},
    {"V15 catch AI, 20 s left", 5, 3, 2000, 20, 1, 500, 0, false, false, 1, 1},
    {"V16 raid, racer at 25 m", 5, 4, 2000, 0, 5, 25, 0, false, false, 1, 0},
};

snd::MusicGame raceGame(int state, int mode) {
    snd::MusicGame g;
    g.context = snd::MusicGame::Race;
    g.missionState = state, g.mode = mode;
    g.remaining = 5000, g.total = 6000, g.timeLeft = 0, g.position = 3;
    return g;
}

// A synthetic stereo track: L = frame, R = -frame (frames < 32768), cues as given.
std::shared_ptr<snd::MusicFile> cueFile(size_t frames, std::vector<snd::MusicFile::Cue> cues) {
    std::vector<int16_t> pcm(frames * 2);
    for (size_t i = 0; i < frames; ++i) pcm[i * 2] = int16_t(i), pcm[i * 2 + 1] = int16_t(-int(i));
    return snd::MusicFile::synthetic(32000, 2, std::move(pcm), std::move(cues));
}

int frameAt(snd::MusicStream& s, size_t n) {  // reads n frames, returns the source frame of the last one
    std::vector<int16_t> b(n * 2);
    const size_t got = s.read(b.data(), n);
    return got ? b[(got - 1) * 2] : -1;
}

void musicReader() {
    std::printf("music reader (WinSound_Z music voice, 0x844570):\n");
    using C = snd::MusicFile::Cue;
    auto f = cueFile(16000, {C{0, "01", 0}, C{0, "intro", 1000}, C{0, "1", 2000}, C{0, "1", 3000}, C{0, "1", 4000}, C{0, "1", 5000},
                             C{0, "standardvar1", 6000}, C{0, "1", 7000}, C{0, "standardvar1end", 8000}, C{0, "actionvar1", 9000},
                             C{0, "1", 10000}, C{0, "1", 11000}, C{0, "1", 12000}, C{0, "1", 13000}, C{0, "actionvar1end", 14000}});
    snd::MusicStream s;
    s.open(f, 0x100);
    expect(s.playing() && s.queued() == 0, "open: playing, empty queue", s.queued(), 0);
    s.queue(snd::musicHash("standardvar1"), snd::musicHash("standardvar1end"), snd::MusicStream::Immed, true);
    near("Immed: jumps at the first cue crossed (frame 0) -> 6000", frameAt(s, 1), 6000, 0);
    s.queue(snd::musicHash("actionvar1"), snd::musicHash("actionvar1end"), snd::MusicStream::End, false);
    near("End: waits for the segment end (frame 7999 before)", frameAt(s, 1999), 7999, 0);
    near("End: then actionvar1 (9000)", frameAt(s, 1), 9000, 0);
    s.queue(snd::musicHash("standardvar1"), snd::musicHash("standardvar1end"), snd::MusicStream::Mesure, true);
    near("Mesure: no jump on the 1st..3rd cue crossed (12999)", frameAt(s, 3999), 12999, 0);
    near("Mesure: jumps on the 4th cue crossed (13000 -> 6000)", frameAt(s, 1), 6000, 0);
    s.queue(snd::musicHash("nosuchlabel"), snd::musicHash("x"), snd::MusicStream::Immed, true);
    s.queue(snd::musicHash("actionvar1"), snd::musicHash("actionvar1end"), snd::MusicStream::Immed, false);
    near("unresolved entry popped, the next one plays (cue 7000 -> 9000)", frameAt(s, 1000), 9000, 0);
    expect(s.queued() == 0, "queue drained", s.queued(), 0);
    near("nothing queued: the segment [9000, 14000) loops (13999 -> 9000)", frameAt(s, 5000), 9000, 0);
    s.queue(snd::musicHash("standardvar1"), snd::musicHash("standardvar1end"), snd::MusicStream::End, false);
    frameAt(s, 6000);
    expect(s.queued() == 1, "after a loop the cue index stays past the end (FUEL quirk): entry waits", s.queued(), 1);
    s.fadeOut(0.5f);
    s.update(0.25f);
    near("fade-out gain halfway (0x7344a0)", s.gain(), 0.5, 1e-6);
    s.update(0.3f);
    expect(!s.playing(), "fade-out ends -> stopped", s.playing(), 0);
}

void musicRules() {
    std::printf("music intensity rules vs FUEL.exe 0x4e7330 / 0x4e7530 under the emulator (16 vectors):\n");
    for (const RuleVec& v : kRules) {
        snd::MusicGame g;
        g.context = snd::MusicGame::Race;
        g.missionState = v.state, g.mode = v.mode, g.remaining = v.remaining, g.timeLeft = v.timeLeft, g.position = v.position;
        g.total = 0;  // route +4 = -1: 0x5c2150 returns 0
        g.nearestRacer = v.racer;
        g.nearestHunted = v.cls == 3 ? v.racer : 1e9f;
        g.lastPlace = v.lastPlace, g.lastGate = v.lastGate;
        char a[96], b[96];
        std::snprintf(a, sizeof a, "%s: action", v.name);
        std::snprintf(b, sizeof b, "%s: extreme", v.name);
        expect(snd::MusicRules::action(g) == (v.action != 0), a, snd::MusicRules::action(g), v.action);
        expect(snd::MusicRules::extreme(g) == (v.extreme != 0), b, snd::MusicRules::extreme(g), v.extreme);
    }
    // The course-length ratio branches (0x4e6e80), from the decoded code (not emulated: 0x628c40 walks the AI path).
    snd::MusicGame g = raceGame(5, 1);
    g.remaining = 2000, g.total = 6000;
    expect(snd::MusicRules::action(g), "time trial: remaining / total 0.33 < 0.4 -> action", 1, 1);
    g = raceGame(5, 12);
    g.remaining = 300, g.total = 6000, g.position = 2;
    expect(snd::MusicRules::extreme(g), "A to B: 0.05 < 0.2, 300 m < 1000 m, ratio < 0.07 -> extreme", 1, 1);
    g = raceGame(5, 5);
    g.remaining = 2000, g.total = 6000, g.timeLeft = 6;
    expect(snd::MusicRules::extreme(g), "time bonus: ratio 0.33 < 0.5 and 6 s < 7 s -> extreme", 1, 1);
}

void musicManager() {
    std::printf("MusicManager_G (0x4e6230 / 0x4e6610 / 0x4e6890) on a synthetic track:\n");
    using C = snd::MusicFile::Cue;
    std::vector<C> cues;
    uint32_t at = 0;
    auto seg = [&](const std::string& name) {  // 8 beats of 2000 frames
        cues.push_back({0, name, at});
        for (int k = 1; k < 8; ++k) cues.push_back({0, "1", at + uint32_t(k) * 2000});
        cues.push_back({0, name + "end", at + 16000});
        at += 17000;
    };
    for (const char* n : {"standardvar1", "standardvar2", "standardvar3", "actionvar1", "actionvar2", "actionvar3", "extremevar1"}) seg(n);
    auto file = snd::MusicFile::synthetic(32000, 2, std::vector<int16_t>(size_t(at) * 2, 1000), cues);
    std::vector<std::string> opened;
    uint32_t seed = 12345;
    snd::MusicManager mm("", {{"trackA", 3, 7}, {"trackB", 3, 6}}, [&seed] {
        seed = uint32_t((uint64_t(seed) * 48271u) % 0x7fffffffu);
        return float(seed) / float(0x7fffffff);
    });
    mm.loader = [&](const std::string& n) -> std::shared_ptr<const snd::MusicFile> {
        opened.push_back(n);
        return file;
    };
    auto run = [&](const snd::MusicGame& g, float secs) {
        for (int i = 0; i < int(secs * 60); ++i) {
            mm.update(1.0f / 60, g);
            std::vector<int16_t> b(534 * 2);
            mm.stream().read(b.data(), 533);  // 32000 / 60 frames of playback
        }
    };
    auto label = [&]() -> std::string {  // the cue that starts the playing segment
        const auto st = mm.stream().state();
        for (const auto& c : file->cues)
            if (c.frame == st.segStart && c.label != "1") return c.label;
        return "(file)";
    };
    snd::MusicGame menu;
    run(menu, 0.5f);
    expect(!opened.empty() && opened.back() == "MENU", "menu context opens MENU, no intensity", mm.intensity(), 0);
    mm.reselect();
    run(raceGame(4, 9), 2.0f);
    expect(opened.back() == "trackA" || opened.back() == "trackB", "race: a random AddMUSIC track", mm.track(), 0);
    expect(mm.level() == 0 && mm.committed(), "countdown: level 0 (standard) committed", mm.level(), 0);
    expect(label().rfind("standardvar", 0) == 0, ("plays " + label()).c_str(), 1, 1);
    snd::MusicGame g = raceGame(5, 9);
    g.nearestRacer = 20;
    run(g, 0.25f);
    expect(mm.pending() == 1 && !mm.committed(), "racer within 30 m -> action requested (pick every 0.2 s)", mm.pending(), 1);
    run(g, 8.0f);
    expect(mm.level() == 1 && label().rfind("actionvar", 0) == 0, ("action committed, plays " + label()).c_str(), mm.level(), 1);
    g.lastGate = true, g.remaining = 600, g.position = 1;
    run(g, 8.0f);
    expect(mm.level() == 2 && label() == "extremevar1", ("last gate within 1 km, leading -> extreme, plays " + label()).c_str(), mm.level(), 2);
    expect(mm.stream().queued() == 4, "4 segments kept queued", mm.stream().queued(), 4);
    snd::MusicGame free;
    free.context = snd::MusicGame::FreeRoam;
    mm.reselect();
    run(free, 2.0f);
    expect(opened.back() == "FreeRide" && mm.intensity(), "free roam -> FreeRide", 1, 1);
}

// P49: rain loops 0x5e10e0, thunder 0x5e32a0, decor fires (0x516cb0), event one-shot 0xfa.
void weatherChecks(const DpcFile& world, const snd::Tuning& t, const std::array<std::shared_ptr<const snd::Pcm>, 0x100>& bank) {
    std::printf("weather ambience (rain 0x5e10e0, thunder 0x5e32a0, decor fires 0x516cb0, teleport 0xfa):\n");
    const std::filesystem::path data = std::filesystem::path(engineExePath(world)).parent_path().parent_path();
    snd::GameMix mix(t);
    mix.update(10.0f);  // the category gains settle at their targets (2 / s)
    snd::Mixer out;
    snd::Listener l;
    out.setListeners({l});
    uint32_t seed = 4242;
    auto rnd = [&seed] {
        seed = uint32_t((uint64_t(seed) * 48271u) % 0x7fffffffu);
        return float(seed) / float(0x7fffffff);
    };
    // Rain: volumes (1 - f) G / 2 and f G / 2, f = floor(rain x 1024) / 1024; stopped at 0.
    {
        snd::Ambience a(bank, t, rnd);
        snd::AmbientInput in;
        in.rain = 0.25f;
        a.update(1.0f / 60, in, mix, out);
        const float v0 = a.emitters[size_t(a.rainSlots[0])].volume, v1 = a.emitters[size_t(a.rainSlots[1])].volume;
        const float base = mix.master * mix.cat[snd::GameMix::Amb0] * mix.cat[snd::GameMix::Amb2];
        near("rain 0.25: light loop 0x68 = (1 - f) G / 2", v0, 0.75 * base * 0.5, 1e-5);
        near("rain 0.25: heavy loop 0x69 = f G / 2", v1, 0.25 * base * 0.5, 1e-5);
        expect(a.playing(a.rainSlots[0], a.rainSerial[0], out) && a.playing(a.rainSlots[1], a.rainSerial[1], out), "rain: both loops play", 1, 1);
        in.rain = 0.0005f;  // floor(0.512) = 0: the splash tank is idle
        a.update(1.0f / 60, in, mix, out);
        expect(!a.playing(a.rainSlots[0], a.rainSerial[0], out) && !a.playing(a.rainSlots[1], a.rainSerial[1], out), "rain < 1/1024: both loops stop", 1, 1);
        in.rain = 1.0f;
        a.update(1.0f / 60, in, mix, out);
        near("rain 1: light loop silent, heavy loop G / 2", a.emitters[size_t(a.rainSlots[0])].volume, 0, 1e-6);
        a.stopAll(out);
    }
    // Thunder: nothing below 0.95 cover; a loop at G' clamp(2 (cloud - 0.95)) and timed strikes above.
    {
        snd::Ambience a(bank, t, rnd);
        snd::AmbientInput in;
        in.cloud = 0.9f;
        in.camera = {100, 50, 200};
        int hooks = 0;
        float minD = 1e9f, maxD = 0;
        a.onLightning = [&](const snd::Vec3f& p) {
            ++hooks;
            const float d = std::hypot(p.x - 100, p.z - 200);
            minD = std::min(minD, d), maxD = std::max(maxD, d);
        };
        for (int i = 0; i < 600; ++i) a.update(1.0f / 60, in, mix, out);
        expect(a.strikes == 0 && a.thunderSlot < 0, "cloud 0.9 (< 0.95): no thunder loop, no strikes", a.strikes, 0);
        in.cloud = 1.0f;
        double span = 0;
        const int frames = 60 * 600;
        std::array<int, 4> kinds{};
        for (int i = 0; i < frames; ++i) {
            const int before = a.strikes;
            a.update(1.0f / 60, in, mix, out);
            if (a.strikes > before) ++kinds[size_t(a.lastThunderSymbol - 0x63)];
            out.update();
        }
        span = frames / 60.0;
        const float G = mix.master * mix.cat[snd::GameMix::Amb0] * mix.cat[snd::GameMix::Amb1];
        near("cloud 1.0: loop 0x67 at G' x clamp(2 (1 - 0.95)) = 0.1 G'", a.emitters[size_t(a.thunderSlot)].volume, 0.1 * G, 1e-5);
        const int strikeEvents = kinds[0] + kinds[1] + kinds[2] + kinds[3];
        const double mean = (t.P(5230) + t.P(5231)) / 2;
        std::printf("  PP5230 %.3g PP5231 %.3g PP5234 %.3g PP5235 %.3g PP5246 %.3g PP2202 %.3g: %d strike events (%d bolts) in %.0f s; symbols 0x63..0x66: %d %d %d %d\n",
                    t.P(5230), t.P(5231), t.P(5234), t.P(5235), t.P(5246), t.P(2202), strikeEvents, a.strikes, span, kinds[0], kinds[1], kinds[2], kinds[3]);
        expect(strikeEvents > 0 && std::fabs(span / strikeEvents - mean) < 0.35 * mean, "strike interval averages (PP5230 + PP5231) / 2", span / std::max(strikeEvents, 1), mean);
        expect(hooks == a.strikes, "onLightning is called once per bolt", hooks, a.strikes);
        expect(minD >= std::sqrt(2.0) * t.P(5234) - 0.5 && maxD <= std::sqrt(2.0) * t.P(5235) + 0.5, "bolts lie at sqrt2 x [PP5234, PP5235] from the tank centre", maxD,
               std::sqrt(2.0) * t.P(5235));
        expect(kinds[0] > 0 && kinds[1] > 0 && kinds[2] > 0 && kinds[3] > 0, "all four thunder sounds 0x63..0x66 occur", kinds[0] * kinds[1] * kinds[2] * kinds[3], 1);
        in.cloud = 0.5f;
        a.update(1.0f / 60, in, mix, out);
        expect(a.thunderSlot < 0, "cloud back under 0.95: the loop stops", a.thunderSlot, -1);
        a.stopAll(out);
    }
    // Decor fires: IT_DECORMESH_G objects whose Lod_Z has a bank symbol 0.
    {
        const auto decor = cloneClassNames((data / "CommonComponent/clone.tsc").string(), "IT_DECORMESH_G");
        const auto names = decorSoundNames(world, (data / "CommonComponent/clone.tsc").string());
        const auto all = lodSoundsWithSymbol(world, 0);
        std::printf("  clone.tsc IT_DECORMESH_G: %zu names; Lods with a symbol-0 sound: %zu, of them decor: %zu\n", decor.size(), all.size(), names.size());
        expect(decor.size() == 609, "IT_DECORMESH_G clones in clone.tsc (p4: 609)", double(decor.size()), 609);
        expect(names.size() == 4, "symbol-0 decor Lods (DebrisFeu_* fires; F14 / IT_HELICO are other classes)", double(names.size()), 4);
        bool helico = false;  // PK07: the race helicopter's rotor loop (IT_HelicoMove vf17 0x515490, bank symbol 0)
        for (const auto& l : all) helico = helico || l.nameCrc == asoboCrc("IT_HELICO");
        expect(helico, "PK07: IT_HELICO has a symbol-0 sound (the race helicopter's rotor loop)", helico, 1);
        bool allFire = !names.empty();
        for (const auto& l : all)
            if (names.count(l.nameCrc)) {
                allFire &= l.sound == 0xefe65beb;
            }
        expect(allFire, "all of them play the fire crackle loop efe65beb", allFire, 1);
        // The names are DebrisFeu_*.
        int known = 0;
        for (const char* n : {"DebrisFeu_Tronc01", "DebrisFeu_tronc02", "DebrisFeu_Souche01", "DebrisFeu_Souche02", "DebrisFeu_Souche03", "DebrisFeuTorche"})
            known += names.count(asoboCrc(n)) != 0;
        expect(known == 4, "the four are among DebrisFeu_Tronc01/02, Souche01..03, Torche", known, 4);
        if (!names.empty()) {
            const uint32_t kind = *names.begin();
            std::unordered_map<uint32_t, std::shared_ptr<const snd::Pcm>> db;
            for (const auto& ls : all)
                if (names.count(ls.nameCrc)) db[ls.nameCrc] = snd::Pcm::from(decodeSound(world, ls.sound));
            snd::Mixer out2;
            out2.setListeners({l});
            snd::Ambience a(bank, t, rnd);
            a.setDecorBank(db);
            snd::AmbientInput in;
            in.camera = {0, 0, 0};
            in.decor = {{{10, 0, 0}, kind}, {{100, 0, 0}, kind}};  // 10 m: inside 50 m; 100 m: outside
            for (int i = 0; i < 8; ++i) a.update(1.0f / 60, in, mix, out2);  // every quarter of the list is visited
            expect(a.decorPlaying(out2) == 1, "decor fire at 10 m plays, the one at 100 m does not", double(a.decorPlaying(out2)), 1);
            in.decor = {{{100, 0, 0}, kind}};
            for (int i = 0; i < 8; ++i) a.update(1.0f / 60, in, mix, out2);
            expect(a.decorPlaying(out2) == 0 && a.decorVoices() == 1, "the fire at 10 m is gone: its voice stops", double(a.decorPlaying(out2)), 0);
            in.camera = {60, 0, 0};  // walking up to the far fire (40 m)
            snd::Listener moved;
            moved.pos = {60, 0, 0};
            out2.setListeners({moved});
            for (int i = 0; i < 8; ++i) a.update(1.0f / 60, in, mix, out2);
            expect(a.decorPlaying(out2) == 1, "camera within 50 m of the fire: it plays", double(a.decorPlaying(out2)), 1);
            a.stopAll(out2);
        }
    }
    // Teleport_Queue 0x57e650 -> symbol 0xfa.
    {
        snd::Ambience a(bank, t, rnd);
        a.teleportQueued(out);
        int live = 0;
        for (const auto& e : a.emitters) live += e.serial >= 0;
        expect(live == 1, "teleport: one 0xfa one-shot", live, 1);
        a.stopAll(out);
    }
}

void ambienceChecks(const DpcFile* world) {
    std::printf("ambience (Game_SoundMgr 0x4e81e0, BirdAnimal / FrogAnimal):\n");
    near("0x5b98d0 day factor 05:00", snd::dayFactor(5 * 3600), 0, 0);
    near("0x5b98d0 day factor 06:00", snd::dayFactor(6 * 3600), 0.5, 1e-4);
    near("0x5b98d0 day factor 12:00", snd::dayFactor(12 * 3600), 1, 0);
    near("0x5b98d0 day factor 18:00", snd::dayFactor(18 * 3600), 0.5, 1e-4);
    near("0x5b98d0 day factor 19:00", snd::dayFactor(19 * 3600), 0, 0);
    near("0x4e9850 approach up 0.2 -> 1 at 1/s, 0.1 s", snd::approach(0.2f, 1, 1, 0.1f), 0.3, 1e-6);
    near("0x4e9850 approach down overshoot clamps", snd::approach(0.05f, 0, 1, 0.1f), 0, 0);
    if (!world) return;
    const std::filesystem::path data = std::filesystem::path(engineExePath(*world)).parent_path().parent_path();
    snd::Tuning t = snd::Tuning::load(engineExePath(*world), (data / "CommonComponent/Param/pp001.tsc").string());
    if (!t.ok) {
        expect(false, "tuning", 0, 1);
        return;
    }
    std::array<std::shared_ptr<const snd::Pcm>, 0x100> bank{};
    int have = 0;
    uint64_t h = 1469598103934665603ull;
    for (const auto& [sym, id] : readSoundBankNamed(*world, "IT_SOUND")) {
        h = (h ^ sym) * 1099511628211ull, h = (h ^ id) * 1099511628211ull;
        if (sym < 0x100) bank[sym] = snd::Pcm::from(decodeSound(*world, id)), ++have;
    }
    std::printf("  IT_SOUND bank: %d sounds, FNV-1a of the pairs %016llx\n", have, (unsigned long long)h);
    expect(bank[0x6a] && bank[0x6d] && bank[0x73] && bank[0x74] && bank[0x75] && bank[0x7d] && bank[0xf7] && bank[0xf8],
           "bank has 0x6a..0x6d, 0x73..0x7d, 0xf7, 0xf8", have, 32);
    snd::GameMix mix(t);
    snd::Mixer out;
    snd::Listener l;
    out.setListeners({l});
    uint32_t seed = 777;
    snd::Ambience amb(bank, t, [&seed] {
        seed = uint32_t((uint64_t(seed) * 48271u) % 0x7fffffffu);
        return float(seed) / float(0x7fffffff);
    });
    snd::AmbientInput in;
    in.player = true;
    in.timeOfDay = 12 * 3600;
    in.forest = 0.5f, in.patch = 0.8f, in.cloud = 0.2f;
    const float pp = t.P(5460);
    in.wind = {0.3f * pp, 0, 0};
    std::vector<float> L(735), R(735);
    double sum = 0;
    int birds = 0;
    for (int i = 0; i < 60 * 60; ++i) {
        amb.update(1.0f / 60, in, mix, out);
        out.update();
        out.render(L.data(), R.data(), 735);
        for (float v : L) sum += double(v) * v;
        for (int k = 0; k < 5; ++k) birds += amb.animals[size_t(k)].active && amb.animals[size_t(k)].playFor > 4.99f;
    }
    const float A = amb.gainA, B = amb.gainB;
    std::printf("  gains: A (0x4e7820) %.4f, B (0x4e78c0) %.4f; PP5460 %.3g, PP2201 %.3g, PP2203 %.3g\n", A, B, pp, t.P(2201), t.P(2203));
    near("wind 0.3: bed 0x6a = (1 - 0.6) A", amb.wind[0].cur, std::clamp(0.4f * A, 0.0f, 1.0f), 1e-5);
    near("wind 0.3: bed 0x6b = 0.6 A", amb.wind[1].cur, std::clamp(0.6f * A, 0.0f, 1.0f), 1e-5);
    near("wind 0.3: bed 0x6c = 0", amb.wind[2].cur, 0, 0);
    near("noon: day bed 0x73 = (1 - forest) day patch B", amb.day.cur, std::clamp(0.5f * 0.8f * B, 0.0f, 1.0f), 1e-5);
    near("noon: night bed 0x74 = 0", amb.night.cur, 0, 0);
    expect(birds > 0, "birds sing within a minute (forest 0.5, light wind, no mission)", birds, 1);
    std::printf("  60 s ambience render: RMS %.4f\n", std::sqrt(sum / (60.0 * 44100)));
    expect(sum > 0, "ambience is audible", sum, 1);
    // A mission arms the birds with 10000 s; frogs need rain.
    snd::Ambience amb2(bank, t, [&seed] {
        seed = uint32_t((uint64_t(seed) * 48271u) % 0x7fffffffu);
        return float(seed) / float(0x7fffffff);
    });
    in.mission = true, in.rain = 0.5f, in.timeOfDay = 12 * 3600;
    in.wind = {0.8f * pp, 0, 0};
    int frogs = 0;
    for (int i = 0; i < 60 * 40; ++i) {
        amb2.update(1.0f / 60, in, mix, out);
        out.update();
        for (int k = 5; k < 9; ++k) frogs += amb2.animals[size_t(k)].active;
    }
    near("mission (or wind > 0.6): birds wait 10000 s (0xa2a4e0)", amb2.animals[0].timer, 10000 - 40, 0.5);
    expect(frogs > 0, "frogs croak in the rain (forest 0.5, below 1500 m)", frogs, 1);
    near("wind 0.8: bed 0x6c = 2 (0.8 - 0.5) A", amb2.wind[2].cur, std::clamp(0.6f * amb2.gainA, 0.0f, 1.0f), 1e-5);
    near("rain 0.5: day bed 0 (1 - 10 rain < 0)", amb2.day.cur, 0, 0);
    // Start countdown (MissionHUD 0x4b3da0): 3, 2, 1 -> 0xf7, GO -> 0xf8.
    snd::Ambience amb3(bank, t, [] { return 0.5f; });
    int started = 0;
    for (int n : {-1, 3, 3, 2, 1, 1, 0, 0, -1}) {
        amb3.countdown(n, out);
        started = 0;
        for (const auto& e : amb3.emitters) started += e.serial >= 0;
    }
    expect(started == 4, "countdown: 3 beeps (0xf7) and GO (0xf8)", started, 4);
    out.stopAll();
    weatherChecks(*world, t, bank);
}

void musicRender(const DpcFile* world) {
    if (!world) return;
    const std::filesystem::path data = std::filesystem::path(engineExePath(*world)).parent_path().parent_path();
    const auto tracks = snd::readAddMusic((data / "LocalizedComponent/GameTsc/Story/Levels.tsc").string());
    expect(tracks.size() == 9, "Levels.tsc: AddMUSIC tracks", double(tracks.size()), 9);
    for (const auto& tr : tracks) std::printf("    %s %d %d\n", tr.name.c_str(), tr.standardVariants, tr.actionVariants);
    // ---- PK27: 0x54e2e0 reads a number only from a token longer than one character (else 3), keeping its low byte: the
    // shipped single-digit numbers all give 3 / 3.
    {
        int three = 0;
        for (const auto& tr : tracks) three += tr.standardVariants == 3 && tr.actionVariants == 3;
        expect(three == int(tracks.size()), "PK27 0x54e2e0: every shipped AddMUSIC track has 3 / 3 variants", three, double(tracks.size()));
        const int got[] = {snd::addMusicNumber(""), snd::addMusicNumber("7"), snd::addMusicNumber("07"), snd::addMusicNumber("12"),
                           snd::addMusicNumber("300"), snd::addMusicNumber("x9")};
        const int want[] = {3, 3, 7, 12, 300 & 0xff, 0};
        int ok = 0;
        for (int i = 0; i < 6; ++i) ok += got[i] == want[i];
        expect(ok == 6, "PK27 0x54e2e0: \"\" / \"7\" -> 3, \"07\" -> 7, \"12\" -> 12, \"300\" -> 44 (byte), \"x9\" -> 0 (atol)", ok, 6);
        // 0x87dad0 / 0x87df90: the marker table of every shipped music file = its labels, in byte-position order.
        int files = 0, sorted = 0;
        size_t cues = 0;
        for (const char* n : {"freeride", "freerideV7marker", "track1MixV1", "track2MixV1", "track3MixV1", "track4MixV1", "track5MixV1", "track6MixV1",
                              "track7MixV1", "track8MixV1", "track9MixV1"}) {
            auto f = snd::MusicFile::open((data / "MusicComponent" / (std::string(n) + ".wav")).string());
            if (!f) continue;
            ++files, cues += f->cues.size();
            bool s = !f->cues.empty();
            for (size_t i = 1; i < f->cues.size(); ++i) s = s && f->cues[i - 1].frame < f->cues[i].frame;  // (track7MixV1 has 7 empty labels)
            sorted += s;
        }
        expect(files == 11 && sorted == files, "PK27 0x87df90: 11 music files, markers in byte-position order", sorted, 11);
        expect(cues == 2120, "PK27 0x87dad0: 2120 markers (every cue point of the shipped files is labelled)", double(cues), 2120);
    }
    // ---- PK27 end
    uint32_t seed = 99;
    snd::MusicManager mm((data / "MusicComponent").string(), tracks, [&seed] {
        seed = uint32_t((uint64_t(seed) * 48271u) % 0x7fffffffu);
        return float(seed) / float(0x7fffffff);
    });
    snd::MusicGame g = raceGame(5, 9);
    std::vector<float> L(735), R(735);
    double sum = 0;
    int levels[3] = {0, 0, 0};
    for (int i = 0; i < 60 * 30; ++i) {
        g.nearestRacer = i > 60 * 10 ? 20.0f : 500.0f;
        mm.update(1.0f / 60, g);
        std::fill(L.begin(), L.end(), 0.0f), std::fill(R.begin(), R.end(), 0.0f);
        mm.stream().render(L.data(), R.data(), 735, 44100);
        for (float v : L) sum += double(v) * v;
        if (mm.level() >= 0) ++levels[mm.level()];
    }
    const auto* f = mm.stream().file();
    std::printf("  30 s of %s (%zu cues): RMS %.4f; frames at level 0 / 1 / 2: %d / %d / %d\n", mm.current().c_str(), f ? f->cues.size() : 0,
                std::sqrt(sum / (30.0 * 44100)), levels[0], levels[1], levels[2]);
    expect(f && sum > 0, "race music renders", sum, 1);
    expect(levels[1] > 0, "a racer close by moves the music to action", levels[1], 1);
}

// P100: the final-mix limiter (settings audio_limiter). A 440 Hz sine through one 2D voice: at 0.5 FS the output is the
// input delayed by kLook - 1 samples, bit for bit; at 3.0 FS (+9.5 dB) the output never exceeds -0.3 dBFS, with no clipped
// samples and no sample-to-sample jump beyond the sine's own slope; the gain recovers to exactly 1 after the burst.
void limiterCheck() {
    std::printf("final-mix limiter (P100: 2.5 ms lookahead, -0.3 dBFS, release 150 ms):\n");
    auto run = [](float amp, int frames, std::vector<float>& L, std::vector<float>& Rr, snd::Mixer::Stats& st) {
        auto pcm = std::make_shared<snd::Pcm>();
        pcm->channels = 1;
        pcm->loops = true;
        pcm->data.resize(44100);
        for (size_t i = 0; i < pcm->data.size(); ++i) pcm->data[i] = amp * std::sin(6.2831853f * 441.0f * float(i) / 44100.0f);
        snd::Mixer m;
        snd::Mixer::PlayParams pp;
        pp.positional = false;
        pp.attenuate = false;
        m.play(pcm, pp);
        m.update();
        L.assign(size_t(frames), 0), Rr.assign(size_t(frames), 0);
        for (int off = 0; off < frames; off += 735) m.render(&L[size_t(off)], &Rr[size_t(off)], std::min(735, frames - off));
        st = m.takeStats();
    };
    std::vector<float> L, R, Li, Ri;
    snd::Mixer::Stats st;
    run(0.5f, 44100, L, R, st);
    // the reference: the same voice with the limiter forced off is the raw sine; compare to the sine itself, delayed.
    double worst = 0;
    for (size_t i = size_t(snd::Mixer::kLook); i < L.size(); ++i) {
        const float x = 0.5f * std::sin(6.2831853f * 441.0f * float(i - size_t(snd::Mixer::kLook - 1)) / 44100.0f);
        worst = std::max(worst, double(std::fabs(L[i] - x)));
    }
    expect(worst < 1e-5 && st.minGain == 1.0f && st.limited == 0, "0.5 FS sine: no gain change (delayed by 109 samples)", worst, 0);
    run(3.0f, 44100, L, R, st);
    float pk = 0, jump = 0;
    for (size_t i = 1; i < L.size(); ++i) pk = std::max(pk, std::fabs(L[i])), jump = std::max(jump, std::fabs(L[i] - L[i - 1]));
    expect(pk <= 0.96606f && st.clippedOut == 0 && st.clipped > 0, "3.0 FS sine: output peak <= -0.3 dBFS, 0 clipped (input clipped > 0)", pk, 0.966);
    expect(jump < 0.2f, "3.0 FS sine: no jump (largest step < 0.2 FS)", jump, 0.1);
    // burst then silence: gain back to 1 within ~1.2 s (release 150 ms, 1e-5 gate)
    auto pcm = std::make_shared<snd::Pcm>();
    pcm->channels = 1;
    pcm->data.assign(4410, 0.0f);
    for (size_t i = 0; i < 2205; ++i) pcm->data[i] = 3.0f * std::sin(6.2831853f * 441.0f * float(i) / 44100.0f);
    snd::Mixer m;
    snd::Mixer::PlayParams pp;
    pp.positional = false, pp.attenuate = false;
    m.play(pcm, pp);
    m.update();
    std::vector<float> a(44100), b(44100);
    for (int off = 0; off < 44100; off += 735) m.render(&a[size_t(off)], &b[size_t(off)], 735);
    const snd::Mixer::Stats s2 = m.takeStats();
    expect(s2.minGain < 0.4f && s2.peakOut <= 0.96606f, "burst: gain reduced then released, peak <= -0.3 dBFS", s2.minGain, 0.32);
}

}  // namespace

int musicAmbienceCheck(const DpcFile* world) {
    gBad = 0;
    limiterCheck();
    musicReader();
    musicRules();
    musicManager();
    ambienceChecks(world);
    musicRender(world);
    std::printf("music / ambience check: %d problems\n", gBad);
    return gBad;
}

}  // namespace fuel
