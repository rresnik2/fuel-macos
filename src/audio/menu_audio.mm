#include "audio/menu_audio.h"

#include "audio/music.hpp"
#include "core/dpc.hpp"
#include "core/resources.hpp"
#include "core/sound.hpp"
#include "audio/sound_mixer.hpp"

#import <AVFoundation/AVFoundation.h>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <vector>

namespace fuel {

struct MenuAudio::Impl {
    std::shared_ptr<snd::Mixer> mixer = std::make_shared<snd::Mixer>();
    std::unique_ptr<snd::MusicManager> music;
    snd::MusicGame game;  // context Menu
    uint32_t rng = 0x2545f491;
    AVAudioEngine* engine = nil;
    AVAudioSourceNode* source = nil;
    bool started = false;

    float rand01() {
        rng = uint32_t((uint64_t(rng) * 48271u) % 0x7fffffffu);
        return float(rng) / float(0x7fffffff);
    }
    void startEngine() {
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
        @try {
            [engine attachNode:source];
            [engine connect:source to:engine.mainMixerNode format:fmt];
            NSError* err = nil;
            [engine prepare];
            if (![engine startAndReturnError:&err]) {
                std::fprintf(stderr, "fuelview: menu audio: cannot start the engine: %s\n", err.localizedDescription.UTF8String);
                return;
            }
            started = true;
        } @catch (NSException* e) {
            std::fprintf(stderr, "fuelview: menu audio: %s; continuing without sound\n", e.reason.UTF8String);
        }
    }
};

MenuAudio::MenuAudio(const std::string& dataDir, float volume, bool output) : impl_(std::make_unique<Impl>()) {
    Impl& m = *impl_;
    const std::filesystem::path data(dataDir);
    auto tracks = snd::readAddMusic((data / "LocalizedComponent/GameTsc/Story/Levels.tsc").string());
    m.music = std::make_unique<snd::MusicManager>((data / "MusicComponent").string(), std::move(tracks), [&m] { return m.rand01(); });
    m.music->stream().setBus(std::clamp(volume, 0.0f, 1.0f));
    snd::MusicManager* mm = m.music.get();
    m.mixer->setAux([mm](float* L, float* R, int n) { mm->stream().render(L, R, n, snd::Mixer::kRate); });
    m.game.context = snd::MusicGame::Menu;
    if (output) m.startEngine();
}

MenuAudio::~MenuAudio() {
    impl_->mixer->setAux(nullptr);
    if (impl_->engine != nil) {
        @try {
            [impl_->engine stop];
        } @catch (NSException*) {
        }
    }
}

void MenuAudio::update(float dt) { impl_->music->update(dt, impl_->game); }

void MenuAudio::setVolume(float v) { impl_->music->stream().setBus(std::clamp(v, 0.0f, 1.0f)); }

bool MenuAudio::playing() const { return impl_->music->stream().playing(); }

std::string MenuAudio::track() const { return impl_->music->current(); }

void MenuAudio::render(float* L, float* R, int n) { impl_->mixer->render(L, R, n); }

// ---- PK18: the menu SFX -----------------------------------------------------------------------------------------------

namespace {
const char* const kSfxNames[MenuSfx::kCount] = {"BACK",        "CANCEL",         "CHANGELETTER", "CONFIRM",   "INVALID", "MOVEBACKWARD",
                                                "MOVEFORWARD", "NAVIGATIONOPTIONS", "POPUPCLOSED",  "POPUPOPEN", "START",   "GPS"};
}

struct MenuSfx::Impl {
    std::shared_ptr<snd::Mixer> mixer = std::make_shared<snd::Mixer>();
    std::array<std::shared_ptr<const snd::Pcm>, kCount> pcm;
    AVAudioEngine* engine = nil;
    AVAudioSourceNode* source = nil;
    void startEngine() {
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
        @try {
            [engine attachNode:source];
            [engine connect:source to:engine.mainMixerNode format:fmt];
            NSError* err = nil;
            [engine prepare];
            if (![engine startAndReturnError:&err])
                std::fprintf(stderr, "fuelview: menu sfx: cannot start the engine: %s\n", err.localizedDescription.UTF8String);
        } @catch (NSException* e) {
            std::fprintf(stderr, "fuelview: menu sfx: %s; continuing without sound\n", e.reason.UTF8String);
        }
    }
};

const char* MenuSfx::name(int index) { return index >= 0 && index < kCount ? kSfxNames[index] : ""; }

MenuSfx::MenuSfx(const std::string& dataDir, float volume, bool output) : impl_(std::make_unique<Impl>()) {
    Impl& m = *impl_;
    try {
        const DpcFile dpc((std::filesystem::path(dataDir) / "CommonComponent/DATAS/SHARED.DPC").string());
        for (int i = 0; i < kCount; ++i) {
            // 0x4ecd40: "DB:>SOUNDS>SFX>MENU>" + name + ".WAV" (the resource id is the CRC of the path); a missing one falls back to
            // ERROR_SOUND.WAV, which the shipped data does not hold.
            const uint32_t id = asoboCrc(std::string("DB:>SOUNDS>SFX>MENU>") + kSfxNames[i] + ".WAV");
            try {
                m.pcm[size_t(i)] = snd::Pcm::from(decodeSound(dpc, id));
            } catch (const std::exception&) {
            }
        }
    } catch (const std::exception&) {
    }
    m.mixer->setBusVolume(volume);
    if (output) m.startEngine();
}

MenuSfx::~MenuSfx() {
    if (impl_->engine != nil) {
        @try {
            [impl_->engine stop];
        } @catch (NSException*) {
        }
    }
}

bool MenuSfx::loaded(int index) const { return index >= 0 && index < kCount && impl_->pcm[size_t(index)] != nullptr; }

bool MenuSfx::play(int index) {
    if (!loaded(index)) return false;
    snd::Mixer::PlayParams p;
    p.positional = false, p.attenuate = false;  // no owner object: no 3D, no distance gain
    return impl_->mixer->play(impl_->pcm[size_t(index)], p) >= 0;
}

void MenuSfx::setVolume(float v) { impl_->mixer->setBusVolume(v); }
void MenuSfx::update() { impl_->mixer->update(); }
int MenuSfx::voices() const { return impl_->mixer->activeVoices(); }
void MenuSfx::render(float* L, float* R, int n) { impl_->mixer->render(L, R, n); }

int menuSfxCheck(const std::string& dataDir) {
    int bad = 0;
    auto expect = [&](bool ok, const std::string& what, double got) {
        std::printf("  %-58s %12.6g%s\n", what.c_str(), got, ok ? "" : "  ** FAIL");
        bad += !ok;
    };
    if (!std::filesystem::exists(std::filesystem::path(dataDir) / "CommonComponent/DATAS/SHARED.DPC")) {
        std::printf("menu sfx check: no SHARED.DPC under %s, skipped\n", dataDir.c_str());
        return 0;
    }
    MenuSfx s(dataDir, 1.0f, false);
    std::vector<float> L(4410), R(4410);
    for (int i = 0; i < MenuSfx::kCount; ++i) {
        const bool ok = s.loaded(i) && s.play(i);
        double sum = 0;
        for (int blk = 0; ok && blk < 10; ++blk) {
            s.update();
            s.render(L.data(), R.data(), 4410);
            for (int k = 0; k < 4410; ++k) sum += double(L[size_t(k)]) * L[size_t(k)] + double(R[size_t(k)]) * R[size_t(k)];
        }
        expect(ok && sum > 0, std::string("menu SFX ") + std::to_string(i) + " " + MenuSfx::name(i) + " decodes, plays and is audible", std::sqrt(sum / (44100.0 * 2)));
    }
    std::printf("menu sfx check: %d problems\n", bad);
    return bad;
}

int menuAudioCheck(const std::string& dataDir) {
    int bad = 0;
    auto expect = [&](bool ok, const char* what, double got) {
        std::printf("  %-58s %12.6g%s\n", what, got, ok ? "" : "  ** FAIL");
        bad += !ok;
    };
    if (!std::filesystem::exists(std::filesystem::path(dataDir) / "MusicComponent/MENU.wav")) {
        std::printf("menu audio check: no MusicComponent/MENU.wav under %s, skipped\n", dataDir.c_str());
        return 0;
    }
    MenuAudio a(dataDir, 1.0f, false);
    std::vector<float> L(735), R(735);
    double sum = 0, tail = 0;
    const int frames = 60 * 20;  // 20 s of menu time, rendered offline
    for (int i = 0; i < frames; ++i) {
        a.update(1.0f / 60);
        a.render(L.data(), R.data(), 735);
        for (int k = 0; k < 735; ++k) {
            const double e = double(L[size_t(k)]) * L[size_t(k)] + double(R[size_t(k)]) * R[size_t(k)];
            sum += e;
            if (i >= frames - 60) tail += e;
        }
    }
    expect(a.playing(), "MENU stream is playing after 20 s", a.playing());
    expect(a.track() == "MENU", "the manager opened \"MENU\" (0x4e6610, menu context)", a.track() == "MENU");
    expect(sum > 0, "the MENU track renders audible samples", std::sqrt(sum / (frames * 735.0 * 2)));
    expect(tail > 0, "still audible in the last second", std::sqrt(tail / (60 * 735.0 * 2)));
    std::printf("menu audio check: %d problems\n", bad);
    return bad;
}

}  // namespace fuel
