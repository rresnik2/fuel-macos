#include "core/custom_vehicles.hpp"  // CUSTOM VEHICLES
#include "drive_audio.h"
#include "core/upgrade_visuals.hpp"  // [upgrades U5]
#include "core/upgrades.hpp"         // [upgrades U5]
#include <complex>

#include "audio/sound_mixer.hpp"
#include "audio/vehicle_sound.hpp"
#include "audio/ev_whine.hpp"
#include "audio/nitro_sound.hpp"  // [nitro]
#include "core/f14_flyby.hpp"  // PK27
#include "core/genworld_noise.hpp"
#include "core/resources.hpp"
#include "core/settings.hpp"
#include "core/vibration.hpp"

#import <AVFoundation/AVFoundation.h>

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <mutex>

namespace fuel {

namespace {

// One tuning per FUEL.exe (pp001.tsc next to it), loaded on first use.
const snd::Tuning* tuningFor(const DpcFile& dpc) {
    static std::mutex m;
    static std::map<std::string, std::unique_ptr<snd::Tuning>> cache;
    // CUSTOM VEHICLES: a pack's archive sits outside the data folder; FUEL.exe is found from its anchor there.
    const std::string exe = custom::anchor(dpc.path()) == dpc.path()
                                ? engineExePath(dpc)
                                : (std::filesystem::path(custom::dataDir()) / "XFinalComponent/FUEL.exe").string();
    std::lock_guard<std::mutex> lock(m);
    auto& t = cache[exe];
    if (!t) {
        const std::filesystem::path data = std::filesystem::path(exe).parent_path().parent_path();
        t = std::make_unique<snd::Tuning>(snd::Tuning::load(exe, (data / "CommonComponent/Param/pp001.tsc").string()));
        if (!t->ok) std::fprintf(stderr, "fuelview: audio: %s\n", t->error.c_str());
    }
    return t->ok ? t.get() : nullptr;
}

std::unique_ptr<snd::VehicleSound> makeSound(const DpcFile& dpc, const std::string& variant, const std::vector<float>& params, bool player,
                                             const snd::Tuning** tuning, uint32_t seed) {
    const snd::Tuning* t = tuningFor(dpc);
    if (!t) return nullptr;
    *tuning = t;
    try {
        // CUSTOM VEHICLES: a pack names a FUEL variant whose sound bank it uses (PortVehicleSound), driven by its own
        // VehiclePP values (rpm range, layer rpms, pops); archives opened once and kept.
        if (const custom::Vehicle* cv = custom::find(variant); cv && !cv->soundDonor.empty()) {
            static std::mutex dm;
            static std::map<std::string, std::unique_ptr<DpcFile>> donors;
            const DpcFile* donor = nullptr;
            {
                std::lock_guard<std::mutex> lock(dm);
                auto& d = donors[cv->soundDpc];
                if (!d) d = std::make_unique<DpcFile>((std::filesystem::path(custom::dataDir()) / "CommonComponent/DATAS" / cv->soundDpc).string());
                donor = d.get();
            }
            snd::VehicleBank b = snd::loadVehicleBank(*donor, cv->soundDonor, params, *t);
            if (cv->synth == "EV") snd::synthesizeEvBank(b);  // vehicles4: electric motor whine instead of the engine
            auto bank = std::make_shared<const snd::VehicleBank>(std::move(b));
            return std::make_unique<snd::VehicleSound>(std::move(bank), *t, player, seed);
        }
        auto bank = std::make_shared<const snd::VehicleBank>(snd::loadVehicleBank(dpc, variant, params, *t));
        return std::make_unique<snd::VehicleSound>(std::move(bank), *t, player, seed);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "fuelview: audio: %s: %s\n", variant.c_str(), e.what());
        return nullptr;
    }
}

}  // namespace

struct DriveAudio::Impl {
    AVAudioEngine* engine = nil;
    AVAudioSourceNode* source = nil;
    std::shared_ptr<snd::Mixer> mixer = std::make_shared<snd::Mixer>();
    const snd::Tuning* tuning = nullptr;
    std::unique_ptr<snd::GameMix> mix;
    std::unique_ptr<snd::VehicleSound> player;
    std::vector<std::unique_ptr<snd::VehicleSound>> rivals;
    std::unique_ptr<snd::Ambience> ambience;
    std::unique_ptr<snd::MusicManager> music;
    GameSound game;
    std::function<void(const std::array<float, 3>&)> lightningHook;
    bool haveGame = false;
    vib::Manager* vibration = nullptr;  // P89
    // [nitro] the boost's synthesized sound per vehicle (audio/nitro_sound.hpp): 0 = the player, k + 1 = rival k.
    std::vector<snd::NitroVoice> nitro;
    void nitroStep(size_t i, const VehicleState& st, float dt, float gain) {
        if (nitro.size() <= i) nitro.resize(i + 1);
        const float amount = st.nitro.on ? st.nitro.amount : 0.0f;
        if (amount <= 0 && !nitro[i].playing() && nitro[i].starts() == 0) return;  // never boosted: nothing to do
        const float v = std::sqrt(st.velocity[0] * st.velocity[0] + st.velocity[1] * st.velocity[1] + st.velocity[2] * st.velocity[2]);
        nitro[i].update(dt, amount, st.chassis.position, v, gain, *mixer);
    }
    uint32_t rng = 0x2545f491;  // port generator (FUEL's 0x6460f0 is shared with the whole game)
    float rand01() {
        rng = uint32_t((uint64_t(rng) * 48271u) % 0x7fffffffu);
        return float(rng) / float(0x7fffffff);
    }
    bool started = false, tried = false, muted = false;
    float master = kFuelDefaultVolume;  // P103: FUEL's SFX volume default 8 / 16 (0x553c80)
    float fade = 1.0f;  // P67: sound volume slot 1 (0x554200), 1 - the screen fader's value
    float bus() const { return master * fade; }
    float musicVol = kFuelDefaultVolume;  // P101: the Music Volume option (0x554040: game +0x254 / 16 -> the music voice,
                                          // vt +0x50); FUEL's factory value is 8 / 16 like SFX (0x553c80, P103)
    float musicBus() const { return musicVol; }  // P103: the screen fader (0x554200 slot 1) dims the SFX bus only
    // P61: object bank sounds, (name CRC << 8 | symbol) -> sound; voices of the loops by key.
    std::unordered_map<uint64_t, std::shared_ptr<const snd::Pcm>> objectBank;
    std::unordered_map<uint64_t, int> loops;
    int f14Voice = -1, f14Pass = 0;  // PK27: the F14 fly-by's emitter voice and the pass it belongs to
    int view = 5;
    // P96: headless dump (setOffline): the mixer is pulled by renderOffline, never by an audio device.
    bool offline = false;
    std::string dumpPath;
    std::vector<int16_t> dump;
    double dumpAcc = 0, dumpSecond = 0;
    snd::Mixer::Stats sec;
    int secVoices = 0;
    void writeDump();

    void start() {
        if (tried) return;
        tried = true;
        if (offline) {
            started = true;
            return;
        }
        engine = [AVAudioEngine new];
        AVAudioFormat* fmt = [[AVAudioFormat alloc] initStandardFormatWithSampleRate:snd::Mixer::kRate channels:2];
        std::shared_ptr<snd::Mixer> m = mixer;
        source = [[AVAudioSourceNode alloc] initWithFormat:fmt
                                               renderBlock:^OSStatus(BOOL*, const AudioTimeStamp*, AVAudioFrameCount n, AudioBufferList* abl) {
                                                   float* L = static_cast<float*>(abl->mBuffers[0].mData);
                                                   if (abl->mNumberBuffers > 1) {
                                                       m->render(L, static_cast<float*>(abl->mBuffers[1].mData), int(n));
                                                   } else {
                                                       std::vector<float> r(n);
                                                       m->render(L, r.data(), int(n));
                                                   }
                                                   return noErr;
                                               }];
        // A missing or busy output device (another copy of the game) must not crash the game: stay silent.
        @try {
            [engine attachNode:source];
            [engine connect:source to:engine.mainMixerNode format:fmt];
            engine.mainMixerNode.outputVolume = muted ? 0.0f : 1.0f;
            NSError* err = nil;
            [engine prepare];
            if (![engine startAndReturnError:&err]) {
                std::fprintf(stderr, "fuelview: audio: cannot start the engine: %s\n", err.localizedDescription.UTF8String);
                return;
            }
            started = true;
        } @catch (NSException* e) {
            std::fprintf(stderr, "fuelview: audio: %s; continuing without sound\n", e.reason.UTF8String);
            started = false;
        }
    }
};

DriveAudio::DriveAudio() : impl_(std::make_unique<Impl>()) {}

void DriveAudio::Impl::writeDump() {
    if (dumpPath.empty()) return;
    std::ofstream f(dumpPath, std::ios::binary);
    auto u32 = [&](uint32_t v) { f.write(reinterpret_cast<const char*>(&v), 4); };
    auto u16 = [&](uint16_t v) { f.write(reinterpret_cast<const char*>(&v), 2); };
    const uint32_t bytes = uint32_t(dump.size() * 2);
    f.write("RIFF", 4), u32(36 + bytes), f.write("WAVEfmt ", 8), u32(16), u16(1), u16(2), u32(snd::Mixer::kRate), u32(snd::Mixer::kRate * 4), u16(4),
        u16(16), f.write("data", 4), u32(bytes);
    f.write(reinterpret_cast<const char*>(dump.data()), std::streamsize(bytes));
    std::printf("audio dump: %s (%.2f s)\n", dumpPath.c_str(), double(dump.size() / 2) / snd::Mixer::kRate);
    dumpPath.clear();
}

void DriveAudio::setOffline(const std::string& wavPath) {
    impl_->offline = true;
    impl_->dumpPath = wavPath;
}

int DriveAudio::activeVoices() const { return impl_->mixer->activeVoices(); }

void DriveAudio::renderOffline(float dt) {
    Impl& m = *impl_;
    if (!m.offline) return;
    m.dumpAcc += double(dt) * snd::Mixer::kRate;
    const int n = int(m.dumpAcc);
    m.dumpAcc -= n;
    if (n <= 0) return;
    std::vector<float> L(static_cast<size_t>(n)), R(static_cast<size_t>(n));
    m.mixer->render(L.data(), R.data(), n);
    for (int i = 0; i < n; ++i) {
        m.dump.push_back(int16_t(std::lround(std::clamp(L[size_t(i)], -1.0f, 1.0f) * 32767.0f)));
        m.dump.push_back(int16_t(std::lround(std::clamp(R[size_t(i)], -1.0f, 1.0f) * 32767.0f)));
    }
    const snd::Mixer::Stats s = m.mixer->takeStats();
    m.sec.peak = std::max(m.sec.peak, s.peak), m.sec.clipped += s.clipped, m.sec.samples += s.samples;
    m.sec.peakOut = std::max(m.sec.peakOut, s.peakOut), m.sec.minGain = std::min(m.sec.minGain, s.minGain), m.sec.limited += s.limited;
    m.sec.clippedOut += s.clippedOut, m.sec.sumSq += s.sumSq, m.sec.sumSqOut += s.sumSqOut;
    m.secVoices = std::max(m.secVoices, m.mixer->activeVoices());
    m.dumpSecond += dt;
    if (m.dumpSecond >= 1.0 - 1e-6) {
        std::printf("audio second %.0f: peak %.3f (%.1f dBFS) clipped %llu of %llu, rms %.1f dBFS, voices <= %d | out peak %.1f dBFS, min gain %.2f dB, limited %llu\n",
                    double(m.dump.size() / 2) / snd::Mixer::kRate, m.sec.peak, 20 * std::log10(std::max(m.sec.peak, 1e-9f)),
                    (unsigned long long)m.sec.clipped, (unsigned long long)m.sec.samples,
                    10 * std::log10(std::max(m.sec.sumSq / double(std::max<uint64_t>(m.sec.samples, 1)), 1e-18)), m.secVoices,
                    20 * std::log10(std::max(m.sec.peakOut, 1e-9f)), 20 * std::log10(std::max(m.sec.minGain, 1e-9f)), (unsigned long long)m.sec.limited);
        if (std::getenv("FUEL_AUDIO_VOICES")) std::printf("%s", m.mixer->describe().c_str());
        m.dumpSecond = 0, m.sec = {}, m.secVoices = 0;
    }
}

DriveAudio::~DriveAudio() {
    impl_->writeDump();
    impl_->mixer->setAux(nullptr);
    if (impl_->engine != nil) {
        @try {
            [impl_->engine stop];
        } @catch (NSException*) {
        }
    }
    impl_->mixer->stopAll();
}

void DriveAudio::setVehicle(const DpcFile& dpc, const std::string& variant, const std::vector<float>& params) {
    Impl& m = *impl_;
    if (m.player) m.player->stop(*m.mixer);
    m.player = makeSound(dpc, variant, params, true, &m.tuning, 1);
    if (m.player) m.player->setVibration(m.vibration);
    // [upgrades U5, DEVIATION] the session's engine stage: a deeper, louder note (+1.5 dB and a +2 dB shelf under 200 Hz per
    // stage) on FUEL's own engine samples; stock / upgrades off / a never-upgraded class: FUEL's.
    if (m.player && upg::enabled() && upgvis::enabled() && upg::classOf(variant) >= 0)
        if (const int e = upg::playerStages()[upg::Engine]; e > 0) {
            m.player->setTone(1.5f * float(e), 2.0f * float(e));
            std::printf("audio: engine stage %d: +%.1f dB, low shelf +%.1f dB (200 Hz)\n", e, 1.5f * float(e), 2.0f * float(e));
        }
    if (m.tuning && !m.mix) {
        m.mix = std::make_unique<snd::GameMix>(*m.tuning);
        m.mix->setView(m.view);
    }
    m.mixer->setBusVolume(m.bus());
    if (m.player) m.start();
}

void DriveAudio::setVibration(vib::Manager* v) {
    impl_->vibration = v;
    if (impl_->player) impl_->player->setVibration(v);
}

void DriveAudio::setRival(size_t k, const DpcFile& dpc, const std::string& variant, const std::vector<float>& params) {
    Impl& m = *impl_;
    if (m.rivals.size() <= k) m.rivals.resize(k + 1);
    if (m.rivals[k]) m.rivals[k]->stop(*m.mixer);
    m.rivals[k] = makeSound(dpc, variant, params, false, &m.tuning, uint32_t(k + 2));
    if (m.tuning && !m.mix) m.mix = std::make_unique<snd::GameMix>(*m.tuning);
}

void DriveAudio::setListener(const std::array<float, 3>& eye, const std::array<float, 3>& target, int view) {
    Impl& m = *impl_;
    snd::Listener l;
    l.pos = {eye[0], eye[1], eye[2]};
    snd::Vec3f f{target[0] - eye[0], target[1] - eye[1], target[2] - eye[2]};
    const float fl = std::sqrt(f.x * f.x + f.y * f.y + f.z * f.z);
    if (fl > 1e-6f) {
        f = {f.x / fl, f.y / fl, f.z / fl};
        // Screen right = forward x up (the port's renderer, lookAtRH); up = right x forward.
        snd::Vec3f r{-f.z, 0, f.x};
        const float rl = std::sqrt(r.x * r.x + r.z * r.z);
        if (rl > 1e-6f) r = {r.x / rl, 0, r.z / rl};
        else r = {1, 0, 0};
        l.forward = f;
        l.right = r;
        l.up = {r.y * f.z - r.z * f.y, r.z * f.x - r.x * f.z, r.x * f.y - r.y * f.x};
    }
    m.mixer->setListeners({l});
    if (view != m.view) {
        m.view = view;
        if (m.mix) m.mix->setView(view);
    }
}

void DriveAudio::updateRival(size_t k, float dt, const VehicleState& st) {
    Impl& m = *impl_;
    if (k >= m.rivals.size() || !m.rivals[k] || !m.mix || !m.started) return;
    if (vehicleSoundsHeld_) return;  // PK13: 0x5d8ed0 skipped while 0x539a50
    m.rivals[k]->update(snd::VehicleInput::from(st, dt), *m.mix, *m.mixer);
    m.nitroStep(k + 1, st, dt, 0.7f);  // [nitro]
}

// ---- PK23
float DriveAudio::playerFlash() const {
    const Impl& m = *impl_;
    return m.player && m.started ? m.player->flash() : -1.0f;
}
// ---- PK23 end

void DriveAudio::update(float dt, const VehicleState& st, float) {
    Impl& m = *impl_;
    if (!m.started || dt < 1e-4f) return;
    if (!vehicleSoundsHeld_) m.nitroStep(0, st, dt, 1.0f);  // [nitro]
    if (!m.mix) {  // P96: no vehicle bank (or none set): the cut-scene and object voices still mix
        m.mixer->update();
        return;
    }
    if (m.player && !vehicleSoundsHeld_) m.player->update(snd::VehicleInput::from(st, dt), *m.mix, *m.mixer);  // PK13: 0x539a50
    m.mix->update(dt);
    if (m.haveGame) {  // Game_SoundMgr 0x4e9b70 (ambience, countdown) and MusicManager_G vt[17]
        if (m.ambience) {
            m.ambience->update(dt, m.game.ambient, *m.mix, *m.mixer);
            m.ambience->countdown(m.game.countdown, *m.mixer);
        }
        if (m.music) m.music->update(dt, m.game.music);
    }
    m.mixer->update();
}

// ---- PK17 (drive_audio.h)
void DriveAudio::pauseMenu(bool open) {
    Impl& m = *impl_;
    if (open) {
        if (m.player) m.player->stop(*m.mixer);  // 0x5d8e60 for each vehicle of 0xa66c10
        for (auto& r : m.rivals)
            if (r) r->stop(*m.mixer);
        for (auto& n : m.nitro) n.stop(*m.mixer);  // [nitro]
    }
    m.mixer->setPauseMask(open ? 7u : 0u);  // WinSound vt+0x3c(7) / (0)
}

uint32_t DriveAudio::pauseMask() const { return impl_->mixer->pauseMask(); }
// ---- end PK17

void DriveAudio::setWorld(const DpcFile& world) {
    Impl& m = *impl_;
    const snd::Tuning* t = tuningFor(world);
    if (!t) return;
    m.tuning = t;
    if (!m.mix) {
        m.mix = std::make_unique<snd::GameMix>(*t);
        m.mix->setView(m.view);
    }
    try {  // Game_SoundMgr's emitters: the "IT_SOUND" Lod_Z bank (0x4e9980)
        std::array<std::shared_ptr<const snd::Pcm>, 0x100> bank{};
        for (const auto& [symbol, id] : readSoundBankNamed(world, "IT_SOUND"))
            if (symbol < 0x100) bank[symbol] = snd::Pcm::from(decodeSound(world, id));
        m.ambience = std::make_unique<snd::Ambience>(bank, *t, [&m] { return m.rand01(); });
        // IT_DECORMESH_G objects with a sound of their own (bank symbol 0): the template name CRC -> its loop.
        const std::filesystem::path common = std::filesystem::path(engineExePath(world)).parent_path().parent_path() / "CommonComponent";
        const auto decor = cloneClassNames((common / "clone.tsc").string(), "IT_DECORMESH_G");
        std::unordered_map<uint32_t, std::shared_ptr<const snd::Pcm>> decorBank;
        for (const LodSound& l : lodSoundsWithSymbol(world, 0))
            if (decor.count(l.nameCrc)) decorBank[l.nameCrc] = snd::Pcm::from(decodeSound(world, l.sound));
        m.ambience->setDecorBank(std::move(decorBank));
        for (uint32_t sym : {0x83u, 0x84u, 0x86u, 0x7eu, 0x7fu, 0x80u, 0x81u})  // P61: impacts, tornadoes
            for (const LodSound& l : lodSoundsWithSymbol(world, sym))
                if (!m.objectBank.count((uint64_t(l.nameCrc) << 8) | sym))
                    m.objectBank[(uint64_t(l.nameCrc) << 8) | sym] = snd::Pcm::from(decodeSound(world, l.sound));
        // PK07: the race helicopter's rotor loop: IT_HELICO's bank symbol 0 (IT_HelicoMove vf17 0x515490).
        for (const LodSound& l : lodSoundsWithSymbol(world, 0))
            if ((l.nameCrc == fuel::asoboCrc("IT_HELICO") || l.nameCrc == fuel::asoboCrc("F14") /* PK27 */) && !m.objectBank.count(uint64_t(l.nameCrc) << 8))
                m.objectBank[uint64_t(l.nameCrc) << 8] = snd::Pcm::from(decodeSound(world, l.sound));
        if (m.lightningHook) setLightningHook(m.lightningHook);
    } catch (const std::exception& e) {
        std::fprintf(stderr, "fuelview: audio: ambience: %s\n", e.what());
    }
    const std::filesystem::path data = std::filesystem::path(engineExePath(world)).parent_path().parent_path();
    m.mixer->setAux(nullptr);
    auto tracks = snd::readAddMusic((data / "LocalizedComponent/GameTsc/Story/Levels.tsc").string());
    m.music = std::make_unique<snd::MusicManager>((data / "MusicComponent").string(), std::move(tracks), [&m] { return m.rand01(); });
    m.music->stream().setBus(m.musicBus());
    snd::MusicManager* mm = m.music.get();
    m.mixer->setAux([mm](float* L, float* R, int n) { mm->stream().render(L, R, n, snd::Mixer::kRate); });
    m.start();
}

void DriveAudio::objectSound(uint32_t lodName, int symbol, const std::array<double, 3>& pos, float pitch, float volume, float minDist,
                             float maxDist) {
    Impl& m = *impl_;
    const auto it = m.objectBank.find((uint64_t(lodName) << 8) | uint32_t(symbol));
    if (it == m.objectBank.end() || !it->second) return;
    snd::Mixer::PlayParams p;  // flags 0xf
    p.pos = {float(pos[0]), float(pos[1]), float(pos[2])}, p.minDist = minDist, p.maxDist = maxDist;
    const int h = m.mixer->play(it->second, p);
    if (h < 0) return;
    m.mixer->setPitch(h, pitch);
    m.mixer->setVolume(h, volume);
}

void DriveAudio::objectLoop(uint64_t key, uint32_t lodName, int symbol, const std::array<double, 3>& pos, bool on, float minDist, float maxDist) {
    Impl& m = *impl_;
    auto v = m.loops.find(key);
    const bool live = v != m.loops.end() && m.mixer->alive(v->second);
    if (!on) {
        if (live) m.mixer->stop(v->second);
        if (v != m.loops.end()) m.loops.erase(v);
        return;
    }
    const snd::Vec3f p3{float(pos[0]), float(pos[1]), float(pos[2])};
    if (live) {
        m.mixer->setPosition(v->second, p3);
        return;
    }
    const auto it = m.objectBank.find((uint64_t(lodName) << 8) | uint32_t(symbol));
    if (it == m.objectBank.end() || !it->second) return;
    snd::Mixer::PlayParams p;
    p.pos = p3, p.minDist = minDist, p.maxDist = maxDist;
    const int h = m.mixer->play(it->second, p);  // [I] the bank sound carries its own loop points, restarted when it ends
    if (h >= 0) m.loops[key] = h;
}

// ---- PK27: the F14 fly-by's Game_SoundMgr emitter. 0x51a380 starts it once per pass (0x4e9da0 -> 0x4ea6c0: the F14
// bank's symbol 0 (0x4ea930), flags 0x4f = positional, attenuated; the emitter node's distances +0x88 / +0x8c = 300 /
// 1200 m; voice volume = descriptor +0x34 = 0). Each frame while the voice lives, 0x4ea560: position from the node,
// the doppler pitch (0x732e30), descriptor volume +0x38 = 1 -> voice +0x6c. A voice that dies (culled beyond 1.5 x
// 1200 m, 0x732f40) is not started again during the pass. 0x51a050 stops it at the end of the pass (0x4e9e80).
void DriveAudio::flyby(int pass, bool on, const std::array<double, 3>& pos, float pitch) {
    Impl& m = *impl_;
    const snd::Vec3f p3{float(pos[0]), float(pos[1]), float(pos[2])};
    if (!on) {
        if (m.f14Voice >= 0 && m.mixer->alive(m.f14Voice)) m.mixer->stop(m.f14Voice);
        m.f14Voice = -1;
        return;
    }
    if (pass != m.f14Pass) {  // a new pass: the emitter starts
        if (m.f14Voice >= 0 && m.mixer->alive(m.f14Voice)) m.mixer->stop(m.f14Voice);
        m.f14Pass = pass;
        m.f14Voice = -1;
        const auto it = m.objectBank.find(uint64_t(fuel::asoboCrc("F14")) << 8);
        if (it == m.objectBank.end() || !it->second) return;
        snd::Mixer::PlayParams pp;  // flags 0x4f: positional, distance gain
        pp.pos = p3, pp.minDist = F14Flyby::kSoundMin, pp.maxDist = F14Flyby::kSoundMax;
        m.f14Voice = m.mixer->play(it->second, pp);  // refused beyond 1.5 x 1200 m (0x732960)
        if (m.f14Voice >= 0) m.mixer->setVolume(m.f14Voice, 0.0f);  // descriptor +0x34
    }
    if (m.f14Voice < 0 || !m.mixer->alive(m.f14Voice)) return;
    m.mixer->setPosition(m.f14Voice, p3);
    m.mixer->setPitch(m.f14Voice, pitch);
    m.mixer->setVolume(m.f14Voice, 1.0f);  // +0x38 -> +0x34 -> voice +0x6c
}

void DriveAudio::setVoiceVolume(float v) { impl_->mixer->setVoiceBus(v); }
// ---- PK27 end

void DriveAudio::setGame(const GameSound& g) {
    // Port: a context change (a race starts or the session returns to free roam) reselects the music, as
    // FUEL's inlined 0x4e6b50 does on its mission events.
    if (impl_->haveGame && impl_->music && g.music.context != impl_->game.music.context) impl_->music->reselect();
    impl_->game = g;
    impl_->haveGame = true;
}

void DriveAudio::teleportQueued() {
    if (impl_->ambience && impl_->started) impl_->ambience->teleportQueued(*impl_->mixer);
}

void DriveAudio::setLightningHook(std::function<void(const std::array<float, 3>&)> f) {
    impl_->lightningHook = std::move(f);
    if (impl_->ambience) {
        auto h = impl_->lightningHook;
        impl_->ambience->onLightning = h ? std::function<void(const snd::Vec3f&)>([h](const snd::Vec3f& p) { h({p.x, p.y, p.z}); }) : nullptr;
    }
}

void DriveAudio::reselectMusic() {
    if (impl_->music) impl_->music->reselect();
}

void DriveAudio::toggleMute() {
    impl_->muted = !impl_->muted;
    if (impl_->engine != nil) impl_->engine.mainMixerNode.outputVolume = impl_->muted ? 0.0f : 1.0f;
}

bool DriveAudio::muted() const { return impl_->muted; }

void DriveAudio::setMasterVolume(float v) {
    impl_->master = std::clamp(v, 0.0f, 1.0f);
    impl_->mixer->setBusVolume(impl_->bus());  // P101: the music has its own volume (setMusicVolume)
}

void DriveAudio::setMusicVolume(float v) {
    impl_->musicVol = std::clamp(v, 0.0f, 1.0f);
    if (impl_->music) impl_->music->stream().setBus(impl_->musicBus());
}

// P67: SoundManager vt+0x40 from 0x554200: volume = slot 0 x slot 1 x the setting; slot 1 = 1 - the fader's value
// (BaseInGameDatas_G vf19 0x438be0), so the screen fades take the sound with them.
void DriveAudio::setFadeGain(float g) {
    g = std::clamp(g, 0.0f, 1.0f);
    if (g == impl_->fade) return;
    impl_->fade = g;
    impl_->mixer->setBusVolume(impl_->bus());
    if (impl_->music) impl_->music->stream().setBus(impl_->musicBus());
}

// P67: cut-scene sounds (RTC events 0x24 / 0x25: SoundManager vt+0x9c play at a node or 2D, vt+0xa4 stop).
int DriveAudio::playSample(std::shared_ptr<const snd::Pcm> pcm, bool positional, const std::array<double, 3>& pos, float volume, float scale) {
    if (!pcm) return -1;
    Impl& m = *impl_;
    m.start();
    snd::Mixer::PlayParams p;
    p.positional = positional, p.attenuate = positional;
    p.pos = {float(pos[0]), float(pos[1]), float(pos[2])};
    p.minDist = 6 * scale, p.maxDist = 140 * scale;  // P96: 0x733080 (the node's +0x88 / +0x8c are 0: 6 / 140 m) x scale
    const int h = m.mixer->play(pcm, p);
    if (h >= 0) m.mixer->setVolume(h, volume);
    return h;
}
void DriveAudio::moveSample(int h, const std::array<double, 3>& pos, float scale) {
    if (h >= 0 && impl_->mixer->alive(h)) {
        impl_->mixer->setPosition(h, {float(pos[0]), float(pos[1]), float(pos[2])});
        impl_->mixer->setDistances(h, 6 * scale, 140 * scale);
    }
}
void DriveAudio::stopSample(int h) {
    if (h >= 0 && impl_->mixer->alive(h)) impl_->mixer->stop(h);
}

// ---- --audio-check --------------------------------------------------------------------------------

namespace {

int gFail = 0;
void expect(bool ok, const char* what, double got, double want) {
    std::printf("  %-58s %12.6g  (want %.6g)%s\n", what, got, want, ok ? "" : "  ** FAIL");
    if (!ok) ++gFail;
}
void near(const char* what, double got, double want, double tol) { expect(std::fabs(got - want) <= tol, what, got, want); }

// A synthetic engine (not a shipped vehicle): parameters 0..677 by number.
std::vector<float> syntheticParams(const float rpm[7], float spread, float gain) {
    std::vector<float> p(678, 0.0f);
    p[45] = 4;
    p[324] = 8000, p[325] = 1000;
    for (int i = 0; i < 7; ++i) p[size_t(636 + i)] = rpm[i];
    p[644] = spread, p[649] = gain;
    p[635] = 3.5f, p[645] = 0.15f, p[646] = -1500, p[650] = 0.15f, p[651] = 2, p[652] = 10, p[653] = 0.5f, p[654] = 0.5f;
    return p;
}

// FUEL's 0x5dbed0 run under Ghidra's emulator on the synthetic sets (notes/p41-work/engine_*.cmd):
// {layer, index, pitch, weight}. Whole tables: max |diff| in notes/p41-audio.md.
struct EngineVec {
    int layer, index;
    float pitch, weight;
};
constexpr EngineVec kEngineA[] = {{0, 0, 1.0f, 1.3f},           {0, 7, 1.3422222f, 0.3454545f}, {0, 14, 1.6844444f, 0.04193545f},
                                  {1, 0, 0.5769231f, 0.0f},     {1, 9, 0.77732795f, 1.0733944f}, {1, 19, 1.0f, 1.3f},
                                  {1, 25, 1.145749f, 1.0055162f}, {1, 37, 1.4372469f, 0.008865163f}};
constexpr EngineVec kEngineB[] = {{0, 7, 1.2333333f, 0.27419353f}, {1, 2, 0.6862745f, 0.21352313f}, {1, 34, 1.4705882f, 0.00846054f},
                                  {2, 34, 0.989899f, 0.9915395f},  {2, 50, 1.2087542f, 0.009138123f}, {3, 51, 0.9944904f, 1.0f},
                                  {3, 67, 1.2561983f, 0.009030897f}, {4, 84, 1.1344538f, 0.008513413f}, {5, 83, 0.9852941f, 0.96623605f},
                                  {5, 99, 1.1544118f, 1.0f}};

std::shared_ptr<snd::Pcm> tone(float seconds, float hz, bool loop) {
    auto p = std::make_shared<snd::Pcm>();
    p->rate = 44100;
    p->loops = loop;
    p->data.resize(size_t(seconds * 44100));
    for (size_t i = 0; i < p->data.size(); ++i) p->data[i] = 0.5f * std::sin(6.2831853f * hz * float(i) / 44100.0f);
    return p;
}

// Backfire rule: drive the synthetic engine through an rpm ramp for `secs`; returns pops and the last volume.
std::pair<int, float> popRun(const snd::Tuning& t, const std::vector<float>& params, float rpm0, float rate, float throttle, float secs) {
    auto bank = std::make_shared<snd::VehicleBank>();
    bank->engine = EngineModel::fromParams(params, t.P(2009));
    bank->wheels = 4;
    bank->bySymbol[2] = tone(1, 100, true);
    for (int s = 0x5e; s <= 0x61; ++s) bank->bySymbol[size_t(s)] = tone(0.05f, 300, false);
    snd::VehicleSound vs(bank, t, true, 7);
    snd::GameMix mix(t);
    snd::Mixer out;
    std::vector<float> L(735), R(735);
    snd::VehicleInput in;
    in.dt = 1.0f / 60;
    in.wheels.resize(4);
    for (auto& w : in.wheels) w.load = 0.05f;
    for (int f = 0; f < int(secs * 60); ++f) {
        in.soundRpm = std::max(rpm0 + rate * float(f) / 60.0f, 1000.0f);
        in.engineThrottle = throttle;
        vs.update(in, mix, out);
        mix.update(in.dt);
        out.update();
        out.render(L.data(), R.data(), 735);
    }
    return {vs.pops(), vs.lastPopVolume() / std::max(mix.gain(snd::GameMix::Bang), 1e-6f)};
}

}  // namespace

int audioCheck(const std::vector<std::pair<const DpcFile*, std::string>>& variants, const std::map<std::string, std::vector<float>>& params) {
    gFail = 0;
    if (variants.empty()) {
        std::printf("audio check: no vehicles\n");
        return 1;
    }
    const snd::Tuning* tp = tuningFor(*variants.front().first);
    if (!tp) return 1;
    const snd::Tuning& t = *tp;

    std::printf("tuning (pp001.tsc PC rows, FUEL.exe 0xa06160)\n");
    // Whole-table check by hash (the table itself stays in the player's FUEL.exe).
    expect(t.surfaceHash == 0x2686b33759f15c7eull, "surface table 0xa06160: FNV-1a matches the decoded version", double(t.surfaceHash == 0x2686b33759f15c7eull), 1);
    near("PP 2009 (first engine layer rise)", t.P(2009), t.P(2009), 0);

    std::printf("engine tables 0x5dbed0 vs the emulator (synthetic sets A and B)\n");
    {
        const float rpmB[7] = {2000, 3000, 4500, 5500, 7000, 8000, 9500};
        const EngineModel eb = EngineModel::fromParams(syntheticParams(rpmB, 0.94f, 1.0f), 2.0f);
        for (const EngineVec& v : kEngineB) {
            char what[64];
            std::snprintf(what, sizeof what, "B layer %d index %d pitch / weight", v.layer, v.index);
            near(what, eb.tablePitch(v.layer, v.index), v.pitch, 2e-6);
            near(what, eb.tableVol(v.layer, v.index), v.weight, 2e-6);
        }
        const float rpmA[7] = {1500, 2600, 3800, 5000, 6200, 7400, 8600};
        const EngineModel e = EngineModel::fromParams(syntheticParams(rpmA, 1.06f, 1.3f), 2.0f);
        for (const EngineVec& v : kEngineA) {
            char what[64];
            std::snprintf(what, sizeof what, "layer %d index %d pitch", v.layer, v.index);
            near(what, e.tablePitch(v.layer, v.index), v.pitch, 2e-6);
            std::snprintf(what, sizeof what, "layer %d index %d weight", v.layer, v.index);
            near(what, e.tableVol(v.layer, v.index), v.weight, 2e-6);
        }
        if (const char* dump = std::getenv("FUEL_AUDIO_DUMP")) {  // whole tables, for the offline comparison
            std::ofstream f(dump);
            for (int L = 0; L < EngineModel::kLayers; ++L) {
                for (int i = 0; i < 100; ++i) f << e.tablePitch(L, i) << (i < 99 ? ' ' : '\n');
                for (int i = 0; i < 100; ++i) f << e.tableVol(L, i) << (i < 99 ? ' ' : '\n');
            }
        }
        // 0x5d9740 pitch lerp and the volume index (not interpolated).
        const EngineModel::Mix m = e.mixAt(0.095f);
        near("mixAt(0.095) layer 1 pitch = lerp(idx 9, 10, 0.5)", m.pitch[1], 0.5f * (e.tablePitch(1, 9) + e.tablePitch(1, 10)), 1e-5);
        near("mixAt(0.095) layer 1 weight = table[9]", m.weight[1], e.tableVol(1, 9), 0);
    }

    std::printf("backfire 0x5d9d60 (P645 0.15 s, P646 -1500 rpm/s, P653 0.5, P654 0.5)\n");
    {
        const float rpmA[7] = {1500, 2600, 3800, 5000, 6200, 7400, 8600};
        const std::vector<float> p = syntheticParams(rpmA, 1.06f, 1.3f);
        auto [n1, v1] = popRun(t, p, 7000, 0, 1, 2);
        near("full throttle, steady 7000 rpm: pops", n1, 0, 0);
        auto [n2, v2] = popRun(t, p, 7500, -3000, 0, 1);
        expect(n2 >= 4 && n2 <= 7, "lift, -3000 rpm/s for 1 s: pops (every 0.15..0.225 s)", n2, 5);
        near("  volume = clamp(rate / P646 - 1) = 1", v2, 1, 1e-3);
        auto [n3, v3] = popRun(t, p, 7500, -2000, 0, 1);
        expect(n3 >= 4 && n3 <= 7, "lift, -2000 rpm/s: pops", n3, 5);
        near("  volume = 2000 / 1500 - 1", v3, 1.0f / 3.0f, 2e-3);
        auto [n4, v4] = popRun(t, p, 7500, -1400, 0, 1);
        near("lift, -1400 rpm/s (slower than P646): pops", n4, 0, 0);
        auto [n5, v5] = popRun(t, p, 7500, -3000, 0.6f, 1);
        near("throttle 0.6 > P653, -3000 rpm/s: pops", n5, 0, 0);
        auto [n6, v6] = popRun(t, p, 4400, -3000, 0, 1);
        near("below half revs (x < P654), -3000 rpm/s: pops", n6, 0, 0);
        (void)v1, (void)v4, (void)v5, (void)v6;
    }

    std::printf("3D gain 0x733150, DirectSound volume / pan (0x8346c0, 0x834630)\n");
    near("vehicle (50 / 350 m) at 30 m", snd::distanceGain(30, 50, 350), 1, 0);
    near("vehicle at 100 m: min(1 - 50/300, 50/100)", snd::distanceGain(100, 50, 350), 0.5, 1e-6);
    near("vehicle at 300 m", snd::distanceGain(300, 50, 350), 1.0 / 6.0, 1e-6);
    near("vehicle at 349 m", snd::distanceGain(349, 50, 350), 1.0 / 300.0, 1e-5);
    near("vehicle at 350 m", snd::distanceGain(350, 50, 350), 0, 0);
    near("default (6 / 140 m) at 20 m: 7 / 20", snd::distanceGain(20, 6, 140), 0.35, 1e-6);
    near("volume 0.5 -> mB", snd::volumeMb(0.5f), -601, 0);
    near("volume 1e-7 -> mB", snd::volumeMb(1e-7f), -10000, 0);
    near("pan +0.9 -> mB", snd::panMb(0.9f), 1998, 0);
    near("pan -0.5 -> mB", snd::panMb(-0.5f), -601, 0);
    {
        snd::Mixer mx;
        snd::Listener l;
        mx.setListeners({l});
        snd::Mixer::PlayParams pp;
        pp.minDist = 50, pp.maxDist = 350;
        pp.pos = {0, 0, 530};
        near("start refused beyond 1.5 x 350 m", mx.play(tone(1, 100, true), pp) < 0, 1, 0);
        pp.pos = {0, 0, 520};
        near("start allowed within 1.5 x 350 m", mx.play(tone(1, 100, true), pp) >= 0, 1, 0);
        // P96: 0x733080 scales the owner's distances by its node scale: TELEPORT_HELICO's A1 (x3) -> 18 / 420 m.
        near("RTC rotor, node scale 3 (18 / 420 m) at 20 m", snd::distanceGain(20, 18, 420), std::min(1.0 - 2.0 / 402.0, 21.0 / 20.0), 1e-6);
    }

    // ---- [upgrades U5, DEVIATION] the engine stage's note: gain + RBJ low shelf (200 Hz) on the engine layers' voices
    std::printf("upgrades U5 engine note (Mixer::toneCoefficients, PlayParams toneDb / shelfDb)\n");
    {
        auto gainAt = [](const std::array<float, 5>& c, double hz) {  // |H(e^jw)| in dB
            const double w = 2.0 * M_PI * hz / 44100.0;
            const std::complex<double> z1 = std::polar(1.0, -w), z2 = z1 * z1;
            const std::complex<double> h = (double(c[0]) + double(c[1]) * z1 + double(c[2]) * z2) / (1.0 + double(c[3]) * z1 + double(c[4]) * z2);
            return 20.0 * std::log10(std::abs(h));
        };
        const auto c3 = snd::Mixer::toneCoefficients(4.5f, 6.0f), c0 = snd::Mixer::toneCoefficients(0, 0);
        near("stage 3: 30 Hz = +4.5 dB gain + 6 dB shelf", gainAt(c3, 30), 10.5, 0.15);
        near("stage 3: 200 Hz (shelf midpoint) = +4.5 + 3 dB", gainAt(c3, 200), 7.5, 0.1);
        near("stage 3: 5 kHz = the gain alone", gainAt(c3, 5000), 4.5, 0.05);
        near("stock coefficients are the identity (b0, b1, b2, a1, a2)", std::fabs(c0[0] - 1) + std::fabs(c0[1] - c0[3]) + std::fabs(c0[2] - c0[4]), 0, 1e-6);
        // A voice without tone renders the same samples as before; with stage 3 the low tone is ~+10.5 dB louder.
        auto render = [&](float toneDb, float shelfDb, float hz) {
            snd::Mixer mx;
            snd::Listener l;
            mx.setListeners({l});
            snd::Mixer::PlayParams pp;
            pp.positional = false, pp.attenuate = false, pp.toneDb = toneDb, pp.shelfDb = shelfDb;
            const int h = mx.play(tone(1, hz, true), pp);
            mx.setVolume(h, 0.1f);
            mx.update();
            std::vector<float> L(8820), R(8820);
            mx.render(L.data(), R.data(), 8820);
            double e = 0;
            for (size_t i = 4410; i < L.size(); ++i) e += double(L[i]) * L[i];
            return std::make_pair(e, L);
        };
        const auto a = render(0, 0, 60), b = render(4.5f, 6.0f, 60);
        near("a 60 Hz engine voice at stage 3: +dB (rendered)", 10.0 * std::log10(b.first / a.first), gainAt(c3, 60), 0.3);
        const auto s0 = render(0, 0, 220);
        snd::Mixer ref;  // the same voice through FUEL's path (no tone fields): bit for bit
        {
            snd::Listener l;
            ref.setListeners({l});
            snd::Mixer::PlayParams pp;
            pp.positional = false, pp.attenuate = false;
            const int h = ref.play(tone(1, 220, true), pp);
            ref.setVolume(h, 0.1f);
            ref.update();
            std::vector<float> L(8820), R(8820);
            ref.render(L.data(), R.data(), 8820);
            near("no tone: the samples are FUEL's mixer's", std::memcmp(L.data(), s0.second.data(), L.size() * 4) == 0 ? 1 : 0, 1, 0);
        }
    }

    std::printf("game mix (Game_SoundMgr faders and categories)\n");
    {
        snd::GameMix mix(t);
        near("view 5 motor category = PP 2021", mix.cat[snd::GameMix::Motor], t.P(2021), 0);
        mix.rec[2].tgt = t.P(2223);
        for (int i = 0; i < 21; ++i) mix.update(1.0f / 60);
        near("airborne duck: fader 2 after 0.35 s (down 0.5/s)", mix.rec[2].cur, std::max(t.P(2223), 1.0f - 0.5f * 0.35f), 1e-4);
        mix.rec[0].tgt = t.P(2220);
        mix.update(1.0f / 60);
        near("impact: fader 0 jumps to PP 2220", mix.rec[0].cur, t.P(2220), 1e-6);
        mix.rec[0].tgt = 1;
        for (int i = 0; i < 21; ++i) mix.update(1.0f / 60);
        near("  then back up at 1/s (0.35 s)", mix.rec[0].cur, std::min(1.0f, t.P(2220) + 0.35f), 1e-4);
        mix.setView(1);
        for (int i = 0; i < 6; ++i) mix.update(1.0f / 60);
        near("view 1: motor category toward PP 2051 at 2/s (0.1 s)", mix.cat[snd::GameMix::Motor],
             t.P(2021) + std::copysign(std::min(0.2f, std::fabs(t.P(2051) - t.P(2021))), t.P(2051) - t.P(2021)), 1e-4);
        // P103: 0x402930 (asm 0x402bbb..0x402c77): view 1 -> PP 0x802 (2050), view 4 -> 0x820 (2080), any other -> 0x7e4 (2020).
        const int views[] = {1, 4, 5, 8, 9, 10, 11};
        const int bases[] = {2050, 2080, 2020, 2020, 2020, 2020, 2020};
        for (int i = 0; i < 7; ++i) {
            char what[96];
            std::snprintf(what, sizeof what, "view %d: categories from PP %d (global, motor)", views[i], bases[i]);
            snd::GameMix vm(t);
            vm.setView(views[i]);
            for (int k = 0; k < 120; ++k) vm.update(1.0f / 60);
            near(what, vm.cat[snd::GameMix::Global] * 10 + vm.cat[snd::GameMix::Motor], t.P(bases[i]) * 10 + t.P(bases[i] + 1), 1e-5);
        }
    }

    std::printf("skid slip value 0x5da710 (P103: P634 dead zone, brake pedal term)\n");
    {
        snd::VehicleInput::Wheel w;
        w.load = 0.5f, w.slip = 0.8f;
        near("ratio 0.8, no pedal, P634 4: no skid", snd::soundSlip(w, 20, 0, 4), 0, 0);
        w.slip = 4.5f;
        near("ratio 4.5 (sliding), P634 4: 4.5 - 4", snd::soundSlip(w, 20, 0, 4), 0.5, 1e-6);
        w.slip = 0.3f;
        near("braking (pedal 1) at 20 m/s, ratio 0.3: 0.3 + 2 x 4 - 4", snd::soundSlip(w, 20, 1, 4), 4.3, 1e-5);
        near("braking at 1.5 m/s (below 2): pedal ignored", snd::soundSlip(w, 1.5f, 1, 4), 0, 0);
        near("half brake, P634 1.5: 0.3 + 1.5 - 1.5", snd::soundSlip(w, 20, 0.5f, 1.5f), 0.3, 1e-6);
        w.load = 0.005f;
        near("unloaded wheel (load <= 0.01): 0", snd::soundSlip(w, 20, 1, 4), 0, 0);
    }

    std::printf("volume options (P103: FUEL's sliders n / 16, factory n = 8)\n");
    {
        near("default SFX / music volume = 8 / 16 (0x553c80(1))", fuel::Settings{}.volume, 0.5, 0);
        near("  as DirectSound volume: trunc(868 ln 0.5) mB (0x8346c0)", snd::volumeMb(fuel::kFuelDefaultVolume), -601, 0);
        near("  slider 16: 0 mB", snd::volumeMb(16.0f / 16.0f), 0, 0);
        near("  slider 1: trunc(868 ln (1/16)) mB", snd::volumeMb(1.0f / 16.0f), -2406, 0);
        near("  slider 0: -10000 mB (silent)", snd::volumeMb(0.0f), -10000, 0);
        // The player's motor at FUEL's default (view 5, standstill, P649 = 2 at the idle index): min(2 x 1.15 x 0.9, 1) x 0.5.
        snd::GameMix vm(t);
        near("idle slot x SFX bus at the default (P649 2, view 5)", std::min(2.0f * vm.motorGain(true), 1.0f) * fuel::kFuelDefaultVolume, 0.5, 1e-6);
        near("an AI's idle slot (x 0.8) at the default", std::min(2.0f * vm.motorGain(false) * 0.8f, 1.0f) * fuel::kFuelDefaultVolume, 0.5, 1e-6);
    }

    // The listed vehicles: banks and an offline render of a short drive.
    for (const auto& [dpc, name] : variants) {
        std::printf("%s\n", name.c_str());
        const auto it = params.find(name);
        if (it == params.end()) {
            std::printf("  ** no driving parameters\n");
            ++gFail;
            continue;
        }
        try {
            auto bank = std::make_shared<const snd::VehicleBank>(snd::loadVehicleBank(*dpc, name, it->second, t));
            int have = 0;
            for (const auto& s : bank->bySymbol) have += s != nullptr;
            const EngineModel& e = bank->engine;
            std::printf("  %d bank samples, %d wheels; idle %.0f max %.0f rpm; P645 %.2f s P646 %.0f rpm/s P653 %.2f P654 %.2f\n", have, bank->wheels,
                        e.idleRpm, e.maxRpm, e.popInterval, e.popRateThreshold, e.popThrottleMax, e.popRpmMin);
            for (int i = 1; i < 100; ++i) {
                float sum = 0;
                for (int L = 0; L < EngineModel::kLayers; ++L) sum += e.tableVol(L, i);
                if (sum > 0 && std::fabs(sum - e.gain) > 1e-4f * e.gain) {
                    expect(false, "engine weights sum to P649", sum, e.gain);
                    break;
                }
            }
            // Drive: 1 s idle, 2 s full throttle (revs to 95 %), 1.5 s lift (engine braking -4000 rpm/s), 1 s skid on class 2.
            snd::VehicleSound vs(bank, t, true, 3);
            snd::GameMix mix(t);
            snd::Mixer out;
            snd::Listener l;
            l.pos = {0, 2, -6};
            out.setListeners({l});
            snd::VehicleInput in;
            in.dt = 1.0f / 60;
            in.wheels.resize(size_t(bank->wheels));
            for (auto& w : in.wheels) w.load = 0.05f, w.classes[2] = 1;
            const char* phase[4] = {"idle", "full throttle", "lift", "skid"};
            const float len[4] = {1, 2, 1.5f, 1};
            float rpm = e.idleRpm;
            std::vector<float> L(735), R(735);
            for (int ph = 0; ph < 4; ++ph) {
                double sq = 0;
                int frames = 0, pops0 = vs.pops();
                for (int f = 0; f < int(len[ph] * 60); ++f) {
                    if (ph == 0) rpm = e.idleRpm, in.engineThrottle = 0;
                    if (ph == 1) rpm = std::min(rpm + (e.maxRpm * 0.95f - e.idleRpm) / 120.0f, e.maxRpm * 0.95f), in.engineThrottle = 1;
                    if (ph == 2) rpm = std::max(rpm - 4000.0f / 60.0f, e.idleRpm), in.engineThrottle = 0;
                    const float v = ph == 0 ? 0.0f : ph == 1 ? 10.0f * float(f) / 120.0f : 10.0f;
                    in.velocity = {0, 0, v};
                    for (auto& w : in.wheels) w.slip = ph == 3 ? 0.6f : 0.0f, w.slipSpeed = ph == 3 ? 8.0f : 0.0f;
                    in.soundRpm = rpm;
                    vs.update(in, mix, out);
                    mix.update(in.dt);
                    out.update();
                    out.render(L.data(), R.data(), 735);
                    for (int i = 0; i < 735; ++i) sq += double(L[size_t(i)]) * L[size_t(i)] + double(R[size_t(i)]) * R[size_t(i)];
                    frames += 735;
                }
                const double rms = std::sqrt(sq / (2.0 * frames));
                std::printf("  %-14s rms %.4f (%5.1f dBFS)  voices %2d  pops %d\n", phase[ph], rms, 20 * std::log10(std::max(rms, 1e-9)), out.activeVoices(),
                            vs.pops() - pops0);
                if (ph < 2 && !(rms > 1e-4)) expect(false, "engine audible", rms, 0.01);
            }
        } catch (const std::exception& ex) {
            std::printf("  ** FAILED: %s\n", ex.what());
            ++gFail;
        }
    }

    // The live graph (AVAudioSourceNode -> main mixer) rendered offline: no audio device involved.
    std::printf("AVAudioEngine offline render of the source node\n");
    @autoreleasepool {
        auto mixer = std::make_shared<snd::Mixer>();
        snd::Mixer::PlayParams pp;
        pp.positional = false;
        mixer->play(tone(1, 220, true), pp);
        mixer->update();
        AVAudioEngine* engine = [AVAudioEngine new];
        AVAudioFormat* fmt = [[AVAudioFormat alloc] initStandardFormatWithSampleRate:snd::Mixer::kRate channels:2];
        std::shared_ptr<snd::Mixer> m = mixer;
        AVAudioSourceNode* src = [[AVAudioSourceNode alloc] initWithFormat:fmt
                                                              renderBlock:^OSStatus(BOOL*, const AudioTimeStamp*, AVAudioFrameCount n, AudioBufferList* abl) {
                                                                  m->render(static_cast<float*>(abl->mBuffers[0].mData),
                                                                            static_cast<float*>(abl->mBuffers[1].mData), int(n));
                                                                  return noErr;
                                                              }];
        double rms = 0;
        @try {
            [engine attachNode:src];
            [engine connect:src to:engine.mainMixerNode format:fmt];
            NSError* err = nil;
            const bool ok = [engine enableManualRenderingMode:AVAudioEngineManualRenderingModeOffline format:fmt maximumFrameCount:4096 error:&err] &&
                            [engine startAndReturnError:&err];
            if (!ok) std::printf("  manual rendering refused: %s\n", err.localizedDescription.UTF8String);
            AVAudioPCMBuffer* buf = [[AVAudioPCMBuffer alloc] initWithPCMFormat:engine.manualRenderingFormat frameCapacity:4096];
            double sq = 0;
            size_t n = 0;
            for (int k = 0; ok && k < 6; ++k) {
                if ([engine renderOffline:4096 toBuffer:buf error:&err] != AVAudioEngineManualRenderingStatusSuccess) break;
                for (AVAudioFrameCount i = 0; i < buf.frameLength; ++i) sq += double(buf.floatChannelData[0][i]) * buf.floatChannelData[0][i];
                n += buf.frameLength;
            }
            [engine stop];
            rms = n ? std::sqrt(sq / double(n)) : 0;
        } @catch (NSException* e) {
            std::printf("  %s\n", e.reason.UTF8String);
        }
        near("rms of a 0.5 sine through the graph (0.5 / sqrt 2)", rms, 0.5 / std::sqrt(2.0), 0.02);
    }
    std::printf("audio check: %d failures\n", gFail);
    return gFail;
}

int audioDriveCheck(const DpcFile& dpc, const std::string& variant, const std::vector<float>& params, const std::vector<VehicleState>& states) {
    const snd::Tuning* tp = tuningFor(dpc);
    if (!tp) return 1;
    std::printf("%s: FUEL's sounds over a scripted drive on the port's physics (offline)\n", variant.c_str());
    try {
        auto bank = std::make_shared<const snd::VehicleBank>(snd::loadVehicleBank(dpc, variant, params, *tp));
        snd::VehicleSound vs(bank, *tp, true, 5);
        snd::GameMix mix(*tp);
        snd::Mixer out;
        std::vector<float> L(735), R(735);
        double sq = 0, peak = 0;
        int pops0 = 0;
        float minRate = 0, maxRpm = 0, rollMax = 0;
        for (size_t k = 0; k < states.size(); ++k) {
            const VehicleState& st = states[k];
            snd::Listener l;  // a chase camera 6 m behind, 2 m up
            l.pos = {float(st.chassis.position[0]), float(st.chassis.position[1]) + 2, float(st.chassis.position[2]) - 6};
            out.setListeners({l});
            vs.update(snd::VehicleInput::from(st, 1.0f / 60), mix, out);
            mix.update(1.0f / 60);
            out.update();
            out.render(L.data(), R.data(), 735);
            for (int i = 0; i < 735; ++i) {
                sq += double(L[size_t(i)]) * L[size_t(i)] + double(R[size_t(i)]) * R[size_t(i)];
                peak = std::max(peak, double(std::fabs(L[size_t(i)])));
            }
            minRate = std::min(minRate, vs.rpmRate());
            for (int i = 29; i < 61; ++i) rollMax = std::max(rollMax, vs.slot(i).current);  // tyres (not wind / whine)
            maxRpm = std::max(maxRpm, st.soundRpm);
            if ((k + 1) % 60 == 0) {
                std::printf("  t %2zu s  %5.0f rpm  gear %d  thr %.2f  %5.1f km/h  rms %5.1f dBFS  peak %.2f  pops %d (min rate %6.0f rpm/s)\n", (k + 1) / 60,
                            st.soundRpm, st.gear, st.engineThrottle, st.speedKmh, 20 * std::log10(std::max(std::sqrt(sq / (2.0 * 60 * 735)), 1e-9)), peak,
                            vs.pops() - pops0, minRate);
                sq = 0, peak = 0, pops0 = vs.pops(), minRate = 0;
            }
        }
        std::printf("  loudest tyre rolling slot %.2f\n", rollMax);
        if (!(rollMax > 0)) {
            std::printf("  ** no tyre rolling sound (no ground class from the physics)\n");
            return 1;
        }
        if (!(maxRpm > 0)) {
            std::printf("  ** no sound rpm from the physics\n");
            return 1;
        }
    } catch (const std::exception& e) {
        std::printf("  ** %s\n", e.what());
        return 1;
    }
    return 0;
}

}  // namespace fuel

namespace fuel {

int hapticsDriveCheck(const DpcFile& dpc, const std::string& variant, const std::vector<float>& params, const std::vector<VehicleState>& states,
                      const std::string& dataDir) {
    int bad = vib::vibrationRulesCheck(dataDir);
    const vib::Shapes shapes = vib::Shapes::load(dataDir);
    const snd::Tuning* tp = tuningFor(dpc);
    if (!tp || !shapes.error.empty()) return bad + 1;
    std::printf("%s: the pad motors FUEL's rules produce (left = low-frequency motor, right = high-frequency; 0..255)\n", variant.c_str());
    try {
        auto bank = std::make_shared<const snd::VehicleBank>(snd::loadVehicleBank(dpc, variant, params, *tp));
        snd::VehicleSound vs(bank, *tp, true, 5);
        snd::GameMix mix(*tp);
        snd::Mixer out;
        vib::Manager man(&shapes);
        vs.setVibration(&man);
        std::map<std::string, int> started, asked;
        man.trace = [&](uint32_t crc, int, float, bool ok) {
            const auto it = shapes.byCrc.find(crc);
            const std::string n = it == shapes.byCrc.end() ? "?" : shapes.list[size_t(it->second)].name;
            ++asked[n];
            if (ok) ++started[n];
        };
        const float dt = 1.0f / 60;
        auto step = [&](const VehicleState& st) {
            vs.update(snd::VehicleInput::from(st, dt), mix, out);
            mix.update(dt);
            out.update();
            return man.update(dt);
        };
        // 1. The scripted drive (full throttle, lift, full, brake): suspension (ABSORBER_n) and backfires (CLUTCH).
        int maxL = 0, maxR = 0, busy = 0;
        for (size_t k = 0; k < states.size(); ++k) {
            const vib::Motors m = step(states[k]);
            maxL = std::max(maxL, m.low), maxR = std::max(maxR, m.high);
            busy += m.low > 0 || m.high > 0;
            if ((k + 1) % 60 == 0) {
                std::printf("  t %2zu s  %5.1f km/h  motors max left %3d right %3d  (%2d of 60 frames rumble)  started:", (k + 1) / 60, states[k].speedKmh, maxL,
                            maxR, busy);
                for (const auto& [n, c] : started) std::printf(" %s %d", n.c_str(), c);
                std::printf("\n");
                maxL = maxR = busy = 0;
                started.clear();
            }
        }
        {  // what the ABSORBER_n rule sees on this drive
            float top = 0;
            for (size_t k = 1; k < states.size(); ++k)
                for (size_t w = 0; w < states[k].wheelFx.size() && w < states[k - 1].wheelFx.size(); ++w)
                    top = std::max(top, std::fabs(states[k].wheelFx[w].load - states[k - 1].wheelFx[w].load) * 60.0f);
            std::printf("  largest suspension rate |d load / dt| %.2f /s (ABSORBER_n above P668 = %.2f)\n", top, bank->engine.absorberRumbleThreshold);
        }
        // 2. Impacts on the last state, held still (no suspension change): FUEL's impact vector phys+0xf2c
        // peaks then falls; the frame it falls with the crash meter +0x138c > 0 asks for the tier's vibration
        // (|c| of the peak vs PP 2150 / 2151), a landing (mostly vertical c) asks for none; a scrape (|c| >
        // PP 2003 along the velocity) asks for IMPACT_SMALL every frame.
        VehicleState still = states.empty() ? VehicleState{} : states.back();
        still.velocity = {0, 0, 0};
        still.impactMeter = 0;
        still.soundImpact = {0, 0, 0};
        for (int k = 0; k < 120; ++k) step(still);  // let everything end
        auto bodyForward = [&](float len) {  // c along the body's -z (a frontal hit), world space
            const auto& q = still.chassis.rotation;  // x, y, z, w
            const float x = q[0], y = q[1], z = q[2], w = q[3];
            const float fx = 2 * (x * z + w * y), fy = 2 * (y * z - w * x), fz = 1 - 2 * (x * x + y * y);
            return std::array<float, 3>{-fx * len, -fy * len, -fz * len};
        };
        struct Hit {
            const char* what;
            float peak;
            bool vertical;
            const char* want;
        };
        const Hit hits[] = {{"light hit", 0.5f * (tp->P(2001) + tp->P(2150)), false, "IMPACT_SMALL"},
                            {"medium hit", 0.5f * (tp->P(2150) + tp->P(2151)), false, "IMPACT_MEDIUM"},
                            {"hard crash", 1.5f * tp->P(2151), false, "IMPACT_HIGH"},
                            {"landing", 1.5f * tp->P(2151), true, nullptr}};
        std::printf("  impact tiers: PP 2001 %.3g (an impact), PP 2150 %.3g (medium), PP 2151 %.3g (hard)\n", tp->P(2001), tp->P(2150), tp->P(2151));
        for (const Hit& h : hits) {
            started.clear();
            VehicleState a = still;
            a.soundImpact = h.vertical ? std::array<float, 3>{0, -h.peak, 0} : bodyForward(h.peak);
            a.impactMeter = 1;
            step(a);  // the peak frame
            VehicleState b = still;
            b.impactMeter = 1;  // still touching on the frame c falls
            std::vector<vib::Motors> seq{step(b)};
            for (int k = 0; k < 70 && man.playing(); ++k) seq.push_back(step(still));
            std::string got;
            for (const auto& [n, c] : started) got += (got.empty() ? "" : ",") + n;
            const bool ok = h.want ? got == h.want : got.empty();
            bad += !ok;
            std::printf("  %-10s |c| %7.2f -> %-13s %s  left/right:", h.what, h.peak, got.empty() ? "(no rumble)" : got.c_str(), ok ? "ok" : "MISMATCH");
            for (size_t k = 0; k < seq.size(); k += 3) std::printf(" %d/%d", seq[k].low, seq[k].high);
            std::printf("\n");
            if (h.want) {  // the first frame against the emulated vf17 vector of that shape at t = 0
                vib::Manager ref(&shapes);
                ref.inject(shapes.find(h.want), 1, 1, 0);
                const vib::Motors r = ref.update(dt);
                const bool same = r == seq[0];
                bad += !same;
                std::printf("    first frame %d/%d = the shape played alone %d/%d: %s\n", seq[0].low, seq[0].high, r.low, r.high, same ? "ok" : "MISMATCH");
            }
            for (int k = 0; k < 30; ++k) step(still);
        }
        {  // a hard landing: the wheels go from unloaded to loaded in one frame (suspension rate past P668)
            VehicleState air = still, down = still;
            for (auto& w : air.wheelFx) w.load = 0;
            for (int k = 0; k < 30; ++k) step(air);  // (the take-off unloads the springs just as fast: ABSORBER_n too)
            started.clear();
            for (auto& w : down.wheelFx) w.load = std::max(w.load, 0.5f);
            std::vector<vib::Motors> seq;
            for (int k = 0; k < 30; ++k) seq.push_back(step(down));
            std::string got;
            int n = 0;
            for (const auto& [name, c] : started) got += (got.empty() ? "" : ",") + name, n += c;
            const bool ok = n == int(down.wheelFx.size()) && got.rfind("ABSORBER_1", 0) == 0;
            bad += !ok;
            std::printf("  landing on %zu wheels -> %s %s  left/right:", down.wheelFx.size(), got.c_str(), ok ? "ok" : "MISMATCH");
            for (size_t k = 0; k < seq.size(); k += 3) std::printf(" %d/%d", seq[k].low, seq[k].high);
            std::printf("\n");
            for (int k = 0; k < 60; ++k) step(still);
        }
        {  // scrape: 1 s along the velocity
            started.clear();
            asked.clear();
            VehicleState s = still;
            s.velocity = {0, 0, 8};
            s.soundImpact = {0, 0, -(tp->P(2003) + 1)};
            s.impactMeter = 1;
            int busyS = 0;
            for (int k = 0; k < 60; ++k) busyS += step(s).low > 0;
            const int n = started.count("IMPACT_SMALL") ? started["IMPACT_SMALL"] : 0;
            const bool ok = n >= 3 && asked["IMPACT_SMALL"] == 60;
            bad += !ok;
            std::printf("  scrape 1 s: IMPACT_SMALL asked %d times, started %d (one 0.3 s pulse at a time), left motor on %d of 60 frames: %s\n",
                        asked["IMPACT_SMALL"], n, busyS, ok ? "ok" : "MISMATCH");
        }
    } catch (const std::exception& e) {
        std::printf("  ** %s\n", e.what());
        return bad + 1;
    }
    {  // [nitro] the boost's rumble (vib::NitroRumble) and its max with FUEL's motors (PadHaptics::send)
        vib::NitroRumble nr;
        const float dt = 1.0f / 60;
        auto amountAt = [](int k) {  // the tank's ramps: 0 for 0.5 s, up over 0.12 s, held to 3.5 s, down over 0.25 s
            const float t = float(k) / 60.0f;
            return t < 0.5f ? 0.0f : t < 3.5f ? std::min(1.0f, (t - 0.5f) / 0.12f) : std::max(0.0f, 1.0f - (t - 3.5f) / 0.25f);
        };
        int first = -1, peakL = 0, peakH = 0, endL = -1, endH = -1, after = 0;
        vib::Motors buzz;
        for (int k = 0; k < 60 * 5; ++k) {
            const vib::Motors m = nr.step(dt, amountAt(k), true);
            if (first < 0 && (m.low > 0 || m.high > 0)) first = k;
            peakL = std::max(peakL, m.low), peakH = std::max(peakH, m.high);
            if (k == 60 * 3) buzz = m;
            if (k >= 60 * 4) after += m.low + m.high;
        }
        vib::NitroRumble off;
        for (int k = 0; k < 60 * 5; ++k) endL = std::max(endL, off.step(dt, amountAt(k), false).low), endH = std::max(endH, off.step(dt, amountAt(k), false).high);
        vib::Motors fuelM;
        fuelM.low = 200, fuelM.high = 10;
        const vib::Motors c = vib::maxCombine(fuelM, buzz);
        // the pulse starts the frame the amount passes 0.05 (0.5 s + 0.006 s): 1 - 1/60/0.25 of 0.55 x 255
        const bool ok = first == 31 && peakL == int(std::lround(255 * 0.55f * (1 - dt / 0.25f))) && buzz.low == 31 && buzz.high == 56 && after == 0 &&
                        endL == 0 && endH == 0 && c.low == 200 && c.high == 56;
        bad += !ok;
        std::printf("  [nitro] boost rumble: pulse from frame %d, peak left/right %d/%d, buzz %d/%d, after the boost %d, vibration off %d/%d, "
                    "max with FUEL's 200/10 -> %d/%d: %s\n",
                    first, peakL, peakH, buzz.low, buzz.high, after, endL, endH, c.low, c.high, ok ? "ok" : "MISMATCH");
    }
    std::printf("haptics check: %d failures\n", bad);
    return bad;
}

}  // namespace fuel
