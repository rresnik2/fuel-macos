// FUEL's music (MusicManager_G + the WinSound_Z music voice). See music.hpp and notes/p45-audio-ambience.md.
#include "music.hpp"

#include "audio/sound_mixer.hpp"
#include "core/console.hpp"  // PK27
#include "core/resources.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <sstream>

#include <fcntl.h>
#include <sys/mman.h>
#include <sys/stat.h>
#include <unistd.h>

namespace fuel::snd {

uint32_t musicHash(const std::string& label) { return label.empty() ? 0 : asoboCrc(label); }

// ---- MusicFile ------------------------------------------------------------------------------------

MusicFile::~MusicFile() {
    if (map_) munmap(map_, mapSize_);
}

std::shared_ptr<MusicFile> MusicFile::synthetic(int rate, int channels, std::vector<int16_t> pcm, std::vector<Cue> cues) {
    auto f = std::make_shared<MusicFile>();
    f->rate = rate, f->channels = channels;
    f->own_ = std::move(pcm);
    f->data = f->own_.data();
    f->frames = f->own_.size() / size_t(channels);
    for (Cue& c : cues) c.hash = musicHash(c.label);
    std::stable_sort(cues.begin(), cues.end(), [](const Cue& a, const Cue& b) { return a.frame < b.frame; });
    f->cues = std::move(cues);
    return f;
}

std::shared_ptr<MusicFile> MusicFile::open(const std::string& path, std::string* error) {
    auto fail = [&](const std::string& why) -> std::shared_ptr<MusicFile> {
        if (error) *error = path + ": " + why;
        return nullptr;
    };
    const int fd = ::open(path.c_str(), O_RDONLY);
    if (fd < 0) return fail("cannot open");
    struct stat st {};
    if (fstat(fd, &st) != 0 || st.st_size < 44) {
        ::close(fd);
        return fail("too small");
    }
    void* map = mmap(nullptr, size_t(st.st_size), PROT_READ, MAP_PRIVATE, fd, 0);
    ::close(fd);
    if (map == MAP_FAILED) return fail("cannot map");
    auto f = std::make_shared<MusicFile>();
    f->map_ = map, f->mapSize_ = size_t(st.st_size), f->path = path;
    const auto* b = static_cast<const uint8_t*>(map);
    const size_t n = size_t(st.st_size);
    auto u32 = [&](size_t o) { uint32_t v; std::memcpy(&v, b + o, 4); return v; };
    auto u16 = [&](size_t o) { uint16_t v; std::memcpy(&v, b + o, 2); return v; };
    if (std::memcmp(b, "RIFF", 4) != 0 || std::memcmp(b + 8, "WAVE", 4) != 0) return fail("not a RIFF WAVE");
    // ---- PK27: FUEL's marker table (WAV parser 0x87dad0 tail + 0x87df90): one entry per LIST/adtl sub-chunk in file
    // order (any sub-chunk id is read as {cue id, text}: hash = AsoboCrc(text), position -1); then one walk over the "cue "
    // points in file order: label i matches point j when their ids are equal (position = bits x channels x
    // dwSampleOffset / 8 + the data chunk's file offset, i + 1, the same point is tried against the next label), else
    // j + 1; finally qsort by unsigned byte position (comparator 0x87d9a0). Unlabelled points are not in the table.
    std::vector<std::pair<uint32_t, uint32_t>> cuePoints;  // (id, dwSampleOffset) in file order
    std::vector<std::pair<uint32_t, std::string>> labels;  // (cue id, text) in adtl order
    // ---- PK27 end
    int bits = 0, format = 0;
    for (size_t o = 12; o + 8 <= n;) {
        const uint32_t size = u32(o + 4);
        const size_t body = o + 8;
        if (body + size > n) break;
        if (std::memcmp(b + o, "fmt ", 4) == 0 && size >= 16) {
            format = u16(body), f->channels = u16(body + 2), f->rate = int(u32(body + 4)), bits = u16(body + 14);
        } else if (std::memcmp(b + o, "data", 4) == 0) {
            f->data = reinterpret_cast<const int16_t*>(b + body);
            f->frames = size / (2 * size_t(std::max(1, f->channels)));
        } else if (std::memcmp(b + o, "cue ", 4) == 0 && size >= 4) {
            const uint32_t k = u32(body);
            for (uint32_t i = 0; i < k && body + 4 + 24 * (i + 1) <= body + size; ++i) {
                const size_t c = body + 4 + 24 * i;
                cuePoints.push_back({u32(c), u32(c + 20)});  // dwName, dwSampleOffset
            }
        } else if (std::memcmp(b + o, "LIST", 4) == 0 && size >= 4 && std::memcmp(b + body, "adtl", 4) == 0) {
            std::string text;  // PK27: 0x87dad0's 260-byte text buffer: a sub-chunk of size 4 (no text) keeps the last text
            for (size_t s = body + 4; s + 8 <= body + size;) {  // PK27: every sub-chunk is a label entry (0x87dad0)
                const uint32_t ss = u32(s + 4);
                if (ss >= 4 && s + 8 + ss <= body + size) {
                    const char* txt = reinterpret_cast<const char*>(b + s + 12);
                    if (ss != 4) text = std::string(txt, strnlen(txt, std::min<size_t>(ss - 4, 259)));
                    labels.push_back({u32(s + 8), text});
                }
                s += 8 + ss + (ss & 1);
            }
        }
        o = body + size + (size & 1);
    }
    if (!f->data || format != 1 || bits != 16 || f->channels < 1 || f->channels > 2) return fail("not PCM16 mono / stereo");
    // ---- PK27: the merge walk and the position sort (see above). The table keeps FUEL's byte positions as frames
    // (sample offsets: bits x channels / 8 per frame, the same order). [port] A label the walk leaves unmatched (position
    // 0xffffffff, sorted last in FUEL) is left out: none in the shipped music, and FUEL would seek past the data with it.
    std::vector<int64_t> pos(labels.size(), -1);
    for (size_t i = 0, j = 0; i < labels.size() && j < cuePoints.size();) {
        if (labels[i].first == cuePoints[j].first) pos[i] = cuePoints[j].second, ++i;
        else ++j;
    }
    for (size_t i = 0; i < labels.size(); ++i) {
        if (pos[i] < 0) continue;
        Cue c;
        c.frame = uint32_t(pos[i]);
        c.label = labels[i].second;
        c.hash = musicHash(c.label);
        f->cues.push_back(c);
    }
    // 0x87df90: qsort by position (MSVC qsort is not stable; equal positions do not occur in the shipped files).
    std::stable_sort(f->cues.begin(), f->cues.end(), [](const Cue& a, const Cue& c) { return a.frame < c.frame; });
    // ---- PK27 end
    return f;
}

// ---- MusicStream ----------------------------------------------------------------------------------

void MusicStream::open(std::shared_ptr<const MusicFile> file, uint32_t flags) {
    std::lock_guard<std::mutex> lock(m_);
    file_ = std::move(file);
    live_ = file_ != nullptr;
    loop_ = (flags & 0x100) != 0;
    s_ = State{};
    if (file_) s_.segLen = uint32_t(file_->frames), s_.left = int64_t(file_->frames);
    q_.clear();
    fadeTotal_ = fadeLeft_ = 0, fadeGain_ = 1;
    phase_ = 0, buf_.clear();
}

void MusicStream::fadeOut(float t) {  // 0x832450
    std::lock_guard<std::mutex> lock(m_);
    if (t > 0 && live_) {
        fadeTotal_ = fadeLeft_ = t;
        return;
    }
    live_ = false, file_.reset(), q_.clear();
}

bool MusicStream::playing() const {
    std::lock_guard<std::mutex> lock(m_);
    return live_;
}

int MusicStream::queued() const {
    std::lock_guard<std::mutex> lock(m_);
    return live_ ? int(q_.size()) : -1;
}

void MusicStream::queue(uint32_t a, uint32_t b, int mode, bool flush) {  // 0x832510
    std::lock_guard<std::mutex> lock(m_);
    if (!live_) return;
    if (flush) q_.clear();
    q_.push_back({a, b, mode});
}

void MusicStream::update(float dt) {  // 0x7344a0 (fade-out part) and 0x733550 (stop when it ends)
    std::lock_guard<std::mutex> lock(m_);
    fadeGain_ = 1;
    if (fadeLeft_ != 0) {
        fadeLeft_ -= dt;
        if (fadeLeft_ < 0) {
            fadeLeft_ = 0, fadeGain_ = 0;
            live_ = false, file_.reset(), q_.clear();  // vt 0x90(0)
            return;
        }
        fadeGain_ = 1 - (fadeTotal_ - fadeLeft_) / fadeTotal_;
    }
}

void MusicStream::setBus(float v) {
    std::lock_guard<std::mutex> lock(m_);
    bus_ = std::clamp(v, 0.0f, 1.0f);
}

float MusicStream::gain() const {
    std::lock_guard<std::mutex> lock(m_);
    return fadeGain_ * bus_;
}

MusicStream::State MusicStream::state() const {
    std::lock_guard<std::mutex> lock(m_);
    return s_;
}

const MusicFile* MusicStream::file() const {
    std::lock_guard<std::mutex> lock(m_);
    return file_.get();
}

bool MusicStream::lookup(const Entry& e, size_t& si, size_t& ei) const {  // 0x844540 -> 0x844500 twice
    const auto& c = file_->cues;
    si = 0;
    while (si < c.size() && c[si].hash != e.start) ++si;
    if (si >= c.size()) return false;
    ei = si + 1;
    while (ei < c.size() && c[ei].hash != e.end) ++ei;
    return ei < c.size();
}

size_t MusicStream::read(int16_t* out, size_t n) {
    std::lock_guard<std::mutex> lock(m_);
    return readLocked(out, n);
}

// The stream reader 0x844570 at frame granularity. Writes up to `n` frames; returns how many.
size_t MusicStream::readLocked(int16_t* out, size_t n) {
    if (!live_ || !file_) return 0;
    const MusicFile& f = *file_;
    const int ch = f.channels;
    static const uint32_t kNod = musicHash("NOD");
    size_t done = 0;
    auto write = [&](size_t k) {  // 0x844a90: k frames from the read position
        for (size_t i = 0; i < k; ++i) {
            const size_t src = size_t(s_.pos) + i;
            for (int c = 0; c < ch; ++c) out[(done + i) * size_t(ch) + size_t(c)] = src < f.frames ? f.data[src * size_t(ch) + size_t(c)] : 0;
        }
        s_.pos += uint32_t(k), s_.left -= int64_t(k), done += k;
    };
    while (done < n) {
        const size_t want = n - done;
        size_t chunk = s_.left >= 1 ? size_t(s_.left) : 0;
        if (want <= chunk) chunk = want;
        bool jumped = false;
        if (!f.cues.empty()) {
            const uint64_t end = uint64_t(s_.pos) + chunk;
            while (s_.cueIdx < f.cues.size() && f.cues[s_.cueIdx].frame <= end) {
                const MusicFile::Cue& cue = f.cues[s_.cueIdx];
                ++s_.count;
                if (!q_.empty()) {
                    const Entry& e = q_.front();
                    bool go = true;
                    if (e.mode == Mesure) {
                        if (s_.count > 3) {
                            if (cue.hash == kNod) s_.count = 0, go = false;
                        } else {
                            go = false;
                        }
                    } else if (e.mode == End) {
                        go = int64_t(cue.frame) - int64_t(s_.pos) == s_.left;
                    }
                    if (s_.count > 3) s_.count = 0;
                    if (go) {
                        while (!q_.empty()) {
                            size_t si = 0, ei = 0;
                            if (lookup(q_.front(), si, ei)) {
                                const size_t upto = cue.frame > s_.pos ? size_t(cue.frame - s_.pos) : 0;
                                write(std::min(upto, n - done));
                                s_.cueIdx = si;
                                s_.segStart = f.cues[si].frame;
                                s_.segLen = f.cues[ei].frame - s_.segStart;
                                s_.left = s_.segLen;
                                s_.pos = s_.segStart;
                                s_.count = 0;
                                q_.pop_front();
                                jumped = true;
                                break;
                            }
                            q_.pop_front();
                        }
                    }
                }
                ++s_.cueIdx;
                if (jumped) break;
            }
        }
        if (jumped) continue;
        write(chunk);
        if (s_.left > 0) continue;
        if (!loop_) break;
        if (s_.segLen == 0) break;
        s_.pos = s_.segStart, s_.left = s_.segLen;  // the segment loops (cue index is not rewound)
    }
    return done;
}

void MusicStream::render(float* L, float* R, int n, int outRate) {
    std::lock_guard<std::mutex> lock(m_);
    if (!live_ || !file_) return;
    const float g = mbToAmplitude(volumeMb(fadeGain_ * bus_));  // 0x8345bb -> 0x8346c0
    const int ch = file_->channels;
    const double step = double(file_->rate) / double(outRate);
    // buf_ holds source frames (L, R floats); phase_ indexes it. Refill from the reader as needed.
    const size_t need = size_t(phase_ + step * n) + 2;
    if (buf_.size() / 2 < need) {
        const size_t more = need - buf_.size() / 2;
        tmp_.assign(more * size_t(ch), 0);
        const size_t got = readLocked(tmp_.data(), more);
        for (size_t k = 0; k < more; ++k) {
            const float l = k < got ? tmp_[k * size_t(ch)] / 32768.0f : 0.0f;
            const float r = k < got ? (ch == 2 ? tmp_[k * size_t(ch) + 1] / 32768.0f : l) : 0.0f;
            buf_.push_back(l), buf_.push_back(r);
        }
    }
    double p = phase_;
    for (int i = 0; i < n; ++i) {
        const size_t i0 = size_t(p);
        const float t = float(p - double(i0));
        const float l0 = buf_[i0 * 2], r0 = buf_[i0 * 2 + 1], l1 = buf_[i0 * 2 + 2], r1 = buf_[i0 * 2 + 3];
        if (g > 0) {
            L[i] += (l0 + (l1 - l0) * t) * g;
            R[i] += (r0 + (r1 - r0) * t) * g;
        }
        p += step;
    }
    const size_t used = size_t(p);
    buf_.erase(buf_.begin(), buf_.begin() + std::ptrdiff_t(std::min(used * 2, buf_.size())));
    phase_ = p - double(used);
}

// ---- MusicRules -----------------------------------------------------------------------------------

namespace {
bool within(const MusicGame& g, float m) { return g.remaining < m; }                           // 0x4e6e30
bool fraction(const MusicGame& g, float x) { return g.total >= 1e-6f && g.remaining / g.total < x; }  // 0x4e6e80
bool timeBelow(const MusicGame& g, float s) { return g.timeLeft < s; }                        // 0x4e6f30
bool racerWithin(const MusicGame& g, float m) { return g.nearestRacer < m; }                  // 0x4e7060
bool huntedClose(const MusicGame& g) { return g.nearestHunted < 10.0f; }                      // 0x4e71a0 (d^2 < 100)
}  // namespace

bool MusicRules::action(const MusicGame& g) {  // 0x4e7330
    if (g.forceAction) return true;
    if (g.forceExtreme || g.suppress) return false;
    if (g.fading || !g.player || g.context != MusicGame::Race) return false;
    if (g.missionState < 5 || g.missionState == 7) return false;
    switch (g.mode) {
    case 0: case 2: case 3: case 7:
        return true;
    case 1: case 6:
        return g.offroad || fraction(g, 0.4f) || timeBelow(g, 90.0f);
    case 4: case 9: case 10: case 11: case 12:
        return (g.position < 4 && fraction(g, 0.3f)) || racerWithin(g, 30.0f) || g.offroad || fraction(g, 0.07f);
    case 5:
        return g.offroad || timeBelow(g, 30.0f);
    case 8:
        return (g.position < 4 && fraction(g, 0.3f)) || racerWithin(g, 30.0f) || g.offroad;
    default:
        return false;
    }
}

bool MusicRules::extreme(const MusicGame& g) {  // 0x4e7530
    if (g.forceExtreme) return true;
    if (g.forceAction || g.suppress) return false;
    if (g.fading || !g.player || g.context != MusicGame::Race) return false;
    if (g.mode == 0 && g.missionState == 0) return false;  // ("DeBugChangeGameMusic" path)
    if (g.missionState < 5 || g.missionState == 7) return false;
    if (g.missionState == 6) return true;
    switch (g.mode) {
    case 1: case 6:
        return within(g, 500) || timeBelow(g, 30);
    case 2:
        return within(g, 500);
    case 3:
        return racerWithin(g, 75) || timeBelow(g, 30);
    case 4: case 10: case 11: case 12:
        return fraction(g, 0.2f) && within(g, 1000) && (fraction(g, 0.07f) || g.position < 4 || huntedClose(g));
    case 5:
        return within(g, 500) || (fraction(g, 0.5f) && timeBelow(g, 7.0f));  // [0x9dc804] = 7
    case 7:
        return g.lastPlace || g.lastGate;
    case 8:
        return within(g, 500) && (g.position < 4 || huntedClose(g));
    case 9:
        return g.lastGate && within(g, 1000) && (g.position < 4 || fraction(g, 0.07f) || huntedClose(g));
    default:
        return false;
    }
}

// ---- Levels.tsc -----------------------------------------------------------------------------------

// ---- PK27: AddMUSIC 0x54e2e0: each number defaults to 3 and is read with atol (low byte kept) only when its token is
// longer than one character, so the shipped "AddMUSIC track1MixV1 3 7" stores 3 / 3 (every race track plays variants
// 1..3). The command word is found like every console command (case-insensitive, or its abbreviation; core/console).
int addMusicNumber(const std::string& token) {
    if (token.size() <= 1) return 3;
    return int(std::strtol(token.c_str(), nullptr, 10) & 0xff);  // _atol, stored as a byte
}

std::vector<MusicTrackInfo> readAddMusic(const std::string& path) {
    std::vector<MusicTrackInfo> out;
    std::ifstream in(path, std::ios::binary);
    std::string line;
    while (std::getline(in, line)) {
        const size_t c = line.find("//");
        if (c != std::string::npos) line.erase(c);
        std::istringstream ls(line);
        std::string verb;
        if (!(ls >> verb) || !consoleCommandIs(verb, "AddMUSIC")) continue;
        MusicTrackInfo t;
        if (!(ls >> t.name)) continue;
        std::string a, b;
        ls >> a >> b;
        t.standardVariants = addMusicNumber(a), t.actionVariants = addMusicNumber(b);
        out.push_back(t);
    }
    return out;
}
// ---- PK27 end

// ---- MusicManager ---------------------------------------------------------------------------------

MusicManager::MusicManager(std::string dir, std::vector<MusicTrackInfo> tracks, std::function<float()> rand01)
    : dir_(std::move(dir)), tracks_(std::move(tracks)), rnd_(std::move(rand01)) {
    loader = [this](const std::string& name) -> std::shared_ptr<const MusicFile> {
        std::error_code ec;
        for (const auto& e : std::filesystem::directory_iterator(dir_, ec)) {
            std::string stem = e.path().stem().string(), want = name;
            std::transform(stem.begin(), stem.end(), stem.begin(), ::tolower);
            std::transform(want.begin(), want.end(), want.begin(), ::tolower);
            if (stem == want && e.path().extension().string().size() == 4) {
                std::string err;
                auto f = MusicFile::open(e.path().string(), &err);
                if (!f) std::fprintf(stderr, "fuelview: music: %s\n", err.c_str());
                return f;
            }
        }
        std::fprintf(stderr, "fuelview: music: no %s in %s\n", name.c_str(), dir_.c_str());  // "LoadMusic GetInfos Error"
        return nullptr;
    };
}

int MusicManager::randInt(int lo, int hi) {  // 0x6462e0: floor(rand x (hi - lo + 0.9999)) + lo
    return int(std::floor(rnd_() * (float(hi - lo) + 0.9999f))) + lo;
}

void MusicManager::stop() {  // 0x4e61a0(0)
    stream_.fadeOut(0);
    flags_ &= ~4u;
    cur_ = pend_ = -1;
}

void MusicManager::reselect() {  // 0x4e6b50
    stream_.fadeOut(0.5f);
    flags_ &= ~4u;
    retry_ = 1.0f;
    cur_ = pend_ = -1;
}

void MusicManager::open() {  // 0x4e6610
    if (flags_ & 4) return;
    stream_.fadeOut(0);
    freeRide_ = false;
    std::string name;
    if (game_.context == MusicGame::Menu) {  // the menu stack is open, or not in game
        name = "MENU";
        intensity_ = false;
    } else if (game_.context == MusicGame::Race && game_.mode != 0 && game_.mode != -1 && !tracks_.empty()) {
        track_ = randInt(0, int(tracks_.size()) - 1);
        name = tracks_[size_t(track_)].name;
        intensity_ = true;
    } else if (game_.context != MusicGame::Loading) {  // free roam (0x443230 +0x5c == 0)
        name = "FreeRide";
        auto f = loader ? loader(name) : nullptr;
        if (f) stream_.open(f, 0x100);
        if (randInt(1, 6) != 1) {  // a jump to a random marker "N" ending at label hash [0xa67434] (0: never found)
            char b[8];
            std::snprintf(b, sizeof b, "%d", randInt(1, 6));
            stream_.queue(musicHash(b), 0, MusicStream::Immed, true);
        }
        intensity_ = freeRide_ = true;
        fileName_ = name;
        if (stream_.playing()) flags_ |= 4;
        return;
    } else {
        name = "TrackLoading";
        intensity_ = false;
    }
    auto f = loader ? loader(name) : nullptr;
    if (f) stream_.open(f, 0x100);
    fileName_ = name;
    if (stream_.playing()) flags_ |= 4;
}

void MusicManager::queueSegment(int mode) {  // 0x4e6890 (the 4th argument is the caller's flush flag)
    const bool flush = mode == MusicStream::Mesure || mode == MusicStream::Immed;  // bl = 1 only for the first call
    const int lvl = committed_ ? cur_ : pend_;
    std::string a, b;
    if (lvl == -1) {
        a = "intro", b = "introend";
    } else if (!freeRide_) {
        if (lvl != 2) {
            const int n = track_ >= 0 && size_t(track_) < tracks_.size() ? tracks_[size_t(track_)].standardVariants : 1;
            const float span = float(n - 1) + 0.9999f;
            int v;
            do {
                v = int(std::floor(rnd_() * span)) + 1;
            } while (v == variant_);
            prevVariant_ = variant_;
            variant_ = v;
        }
        char buf[32];
        if (lvl == 0) std::snprintf(buf, sizeof buf, "standardvar%d", variant_);
        else if (lvl == 1) std::snprintf(buf, sizeof buf, "actionvar%d", variant_);
        else std::snprintf(buf, sizeof buf, "extremevar1");
        a = buf, b = a + "end";
    } else {
        if (variant_ == -1) variant_ = randInt(1, 6);
        a = std::to_string(variant_);
        ++variant_;
        b = std::to_string(variant_);
        if (variant_ == 7) variant_ = 1;
    }
    stream_.queue(musicHash(a), musicHash(b), mode, flush);
}

void MusicManager::commit() {  // 0x4e6ac0
    t184_ = 0;
    committed_ = true;
    if (pend_ == -1) t184_ = 15, hold_ = 5;
    else hold_ = 0;
    cur_ = pend_;
    hold2_ = 0;
}

int MusicManager::pickLevel() {
    if (freeRide_) return 0;
    const int levels[3] = {0, 1, 2};
    for (int i = 2; i >= 0; --i) {
        if (hold2_ > 0 && levels[i] <= cur_) return -1;
        const bool ok = i == 0 ? true : i == 1 ? MusicRules::action(game_) : MusicRules::extreme(game_);
        if (ok) return levels[i];
    }
    return -1;
}

void MusicManager::update(float dt, const MusicGame& g) {
    game_ = g;
    stream_.update(dt);
    if (!(flags_ & 8)) {
        if (flags_ & 4) stop();
        return;
    }
    if ((flags_ & 4) && !stream_.playing()) reselect();  // the stream ended: 0x4e6b50
    bool open4 = (flags_ & 4) != 0;
    if (!open4) {
        retry_ -= dt;
        if (retry_ < 0) {
            open();
            open4 = (flags_ & 4) != 0;
            if (!open4 && retry_ < 0) {
                retry_ = 15;
                return;
            }
        }
    }
    if (!intensity_) return;
    hold_ -= dt, hold2_ -= dt, pick_ -= dt, t184_ -= dt;
    if (!open4) return;
    if (hold_ < 0 && pick_ < 0) {
        pick_ = 0.2f;
        const int lvl = pickLevel();
        bool change = true;
        if (!committed_) {
            if (lvl == pend_) change = false;
        } else if (lvl == cur_) {
            change = false;  // (+0x19c == the music context id: always the same in the port)
        }
        if (change) {
            const int was = cur_;
            hold_ = hold2_ = 1.0f;
            pend_ = lvl;
            variant_ = prevVariant_ = -1;
            committed_ = false;
            queueSegment(was != -1 ? MusicStream::Mesure : MusicStream::Immed);
            if (stream_.playing()) {
                int added = 0;
                for (int k = 4 - stream_.queued(); k != 0; k = 4 - stream_.queued()) {
                    if (k > 0) {
                        added += k;
                        for (; k; --k) queueSegment(MusicStream::End);
                    }
                    if (added > 3) break;
                }
            }
            if (lvl == cur_) commit();
        }
    }
    if (stream_.playing()) {  // LAB_004e652e
        if (!committed_ && stream_.queued() < 4) commit();
        refill_ -= dt;
        if (refill_ < 0) {
            refill_ = 0.25f;
            if (stream_.playing() && stream_.queued() < 4) queueSegment(MusicStream::End);
        }
    }
}

}  // namespace fuel::snd
