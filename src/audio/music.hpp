// FUEL's music, ported from MusicManager_G (vtable 0x9adfb0) and the music voice of WinSound_Z.
//
// Decoded (notes/p45-audio-ambience.md section 1):
//  - MusicManager_G vt[17] 0x4e5e90: a state machine at +0x40; state 0 = the game music (0x4e6230), the
//    other states (1..13) serve scripted streams (vt[18..22]: dialogue, "##SKIP##", cut-scenes) and are not
//    used by the port.
//  - 0x4e6610 picks the file by context: the menu stack / not in game -> "MENU"; a race (0x62ac50 not 0 / -1)
//    -> one of the Levels.tsc AddMUSIC tracks at random (index rand[0, n-1] at +0x3c), intensity on;
//    free roam -> "FreeRide", intensity on, free-ride cue mode (+0x3b1); otherwise "TrackLoading".
//    The stream opens with flag 0x100 (loop the current segment). No crossfades: every open passes fade 0.
//  - 0x4e6230 (state 0): every 0.2 s (+0x180), once the 1 s hold (+0x178) is over, the highest level of the
//    list (0 standard = always, 1 action = 0x4e7330, 2 extreme = 0x4e7530) whose rule holds is picked. A new
//    level queues its first cue segment (mode 1 "Mesure" after another level, 0 "Immed" for the first one,
//    with flush) and tops the queue up to 4 segments (mode 2 "End"); it is committed (0x4e6ac0) once the
//    queue falls below 4. Every 0.25 s (+0x18c) one more "End" segment keeps 4 queued.
//  - 0x4e6890: race tracks play "standardvarN" / "actionvarN" -> "...end", N = 1 + floor(rand x (a - 1 +
//    0.9999)) different from the last one, a = AddMUSIC's first number (byte +0x40 of the 0x42-byte record;
//    used for both levels: the second number is never read); "extremevar1"; level -1 = "intro" -> "introend".
//    Free ride plays the cue pairs "N" -> "N+1", N from rand[1, 6] then +1, 7 wraps to 1.
//  - The intensity rules 0x4e7330 (action) and 0x4e7530 (extreme): MusicRules below.
//  - The music voice (WinSound_Z vt 0x80 open 0x832310 -> 0x831ee0, 0x84 queue 0x832510, 0x88 count
//    0x8324f0, 0x90 fade 0x832450, 0x94 playing 0x8324b0) and its stream reader 0x844570: the WAV's cue
//    points (cue + LIST/adtl/labl, label hash 0x669160 = the engine CRC, case-insensitive) are crossed in
//    byte-position order (PK27: 0x87df90 sorts the marker table, built from the adtl labels by 0x87dad0); at each
//    crossing (counter +0x38) the front queue entry may jump: mode 0 at once, mode 1 at
//    a crossing after more than 3 counted (a cue named "NOD" resets the count instead), mode 2 only at the
//    current segment's end; the jump pops entries until both labels resolve (start = first cue with the
//    start label, end = the next cue with the end label after it) and plays [start, end). A segment that
//    runs out with nothing queued restarts itself (flag 0x100).
//  - Volume: fade gain (0x7344a0, linear, fade-out then stop) x music bus (+0x20e4 = setting / 16) ->
//    DirectSound volume (0x8345bb). The music is a stereo 32 kHz PCM16 WAV streamed from MusicComponent.
// Port: the reader runs on the audio thread one block at a time, so the queue state it sees is the current
// one (FUEL reads 0x20000-byte chunks, about 1 s ahead of playback). Linear resampling 32 -> 44.1 kHz.
#pragma once

#include <array>
#include <cstdint>
#include <deque>
#include <functional>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

namespace fuel::snd {

// A RIFF WAV (PCM16) mapped read-only, with its cue points in position order.
class MusicFile {
public:
    struct Cue {
        uint32_t hash = 0;  // 0x669160 of the label (0 = no label)
        std::string label;
        uint32_t frame = 0;
    };
    static std::shared_ptr<MusicFile> open(const std::string& path, std::string* error = nullptr);
    // A file built in memory (for the checks): interleaved PCM16.
    static std::shared_ptr<MusicFile> synthetic(int rate, int channels, std::vector<int16_t> pcm, std::vector<Cue> cues);
    ~MusicFile();
    int rate = 32000, channels = 2;
    size_t frames = 0;
    const int16_t* data = nullptr;
    std::vector<Cue> cues;
    std::string path;

private:
    void* map_ = nullptr;
    size_t mapSize_ = 0;
    std::vector<int16_t> own_;
};

uint32_t musicHash(const std::string& label);  // 0x669160

// The WinSound_Z music voice: one stream, its segment queue and its fade.
class MusicStream {
public:
    enum Mode { Immed = 0, Mesure = 1, End = 2 };
    // vt 0x80 with fade 0: stops what plays and starts `file` from its first frame (flags 0x100: loop the segment).
    void open(std::shared_ptr<const MusicFile> file, uint32_t flags);
    void fadeOut(float seconds);  // vt 0x90: 0 = stop now
    bool playing() const;         // vt 0x94
    int queued() const;           // vt 0x88
    // vt 0x84: queue {start, end, mode}; `flush` empties the queue first (0x831e90).
    void queue(uint32_t startHash, uint32_t endHash, int mode, bool flush);
    void update(float dt);  // 0x733550 / 0x7344a0: the fade
    void setBus(float v);   // +0x20e4 (the music volume, 0..1)
    float gain() const;     // fade x bus (before the DirectSound volume step)

    // Audio thread: adds `n` frames at `outRate` into L / R.
    void render(float* L, float* R, int n, int outRate);
    // Checks: the reader 0x844570 alone, `n` source frames into `out` (interleaved; silence when stopped).
    // Returns the frames written before the stream ran dry.
    size_t read(int16_t* out, size_t n);

    struct State {  // the reader fields (stream request +0x1c..+0x38)
        uint32_t segStart = 0, segLen = 0;  // +0x20, +0x1c
        int64_t left = 0;                   // +0x24
        uint32_t pos = 0;                   // +0x28
        size_t cueIdx = 0;                  // +0x2c
        int count = 0;                      // +0x38
    };
    State state() const;
    const MusicFile* file() const;

private:
    struct Entry {
        uint32_t start, end;
        int mode;
    };
    mutable std::mutex m_;
    std::shared_ptr<const MusicFile> file_;
    bool loop_ = false, live_ = false;
    State s_;
    std::deque<Entry> q_;
    float fadeTotal_ = 0, fadeLeft_ = 0, fadeGain_ = 1, bus_ = 1;
    // resampler
    double phase_ = 0;
    std::vector<float> buf_;  // source frames waiting for the resampler (L, R)
    std::vector<int16_t> tmp_;
    size_t readLocked(int16_t* out, size_t n);
    bool lookup(const Entry& e, size_t& si, size_t& ei) const;  // 0x844540
};

// What the music rules read from the game (FUEL's mission object 0x62abd0 and friends), per frame.
struct MusicGame {
    enum Context { Menu, FreeRoam, Race, Loading };
    Context context = Menu;
    int missionState = 0;     // mission +0x48: 3 streaming, 4 preparing (countdown), 5 running, 6 won, 7 lost
    int mode = 0;             // 0x62ac50 (= Race::gameMode(): 1 time trial .. 12 A to B); 0 when state is 0 / 1
    bool player = true;       // 0x421f60: there is a player vehicle
    bool fading = false;      // 0x46dd80 +0x28 > 1e-6 (screen fade): no action / extreme
    bool forceAction = false, forceExtreme = false, suppress = false;  // debug bytes 0xa65440 / 0xa65441 / 0xa6543f
    float remaining = 0;      // mission +0x22c = route +0x170: metres to the finish (0x5c21a0)
    float total = 0;          // 0x5c2150 -> 0x628c40(0): the course length (0: no route, ratios fail)
    float timeLeft = 0;       // mission +0x344 (0x4e6f30 always reads it)
    int position = 1;         // mission +0x270 (1 = leading)
    bool lastPlace = false;   // 0x4e6ef0: mission +0x234 (the player's rank index) == +0x290 (racer count) [I]
    bool lastGate = false;    // 0x4e7240: only the finish line is left (last lap)
    bool offroad = false;     // 0x4e6f80: no road record within 50 m, or the nearest is farther than 10 m
    float nearestRacer = 1e9f;   // 0x4e7060: closest opponent still racing (state < 4), metres
    float nearestHunted = 1e9f;  // 0x4e71a0: the first class-3 racer (hunted), if still racing, metres
};

namespace MusicRules {
bool action(const MusicGame& g);   // 0x4e7330
bool extreme(const MusicGame& g);  // 0x4e7530
}  // namespace MusicRules

// Levels.tsc "AddMUSIC <name> <a> <b>" (MusicManager_G +0x170 / +0x174, 0x42-byte records).
struct MusicTrackInfo {
    std::string name;
    int standardVariants = 1, actionVariants = 1;
};
std::vector<MusicTrackInfo> readAddMusic(const std::string& levelsTsc);
int addMusicNumber(const std::string& token);  // PK27: 0x54e2e0's number rule (3 unless the token is longer than 1 char)

class MusicManager {
public:
    // `dir` = MusicComponent; `rand01` = the game's uniform [0, 1) (0x6460f0).
    MusicManager(std::string dir, std::vector<MusicTrackInfo> tracks, std::function<float()> rand01);
    // vt[17] 0x4e5e90 in state 0 (0x4e6230), once per frame.
    void update(float dt, const MusicGame& g);
    // The inlined 0x4e6b50 (race start / end, mission changes): fade out over 0.5 s, reselect after 1 s.
    void reselect();
    MusicStream& stream() { return stream_; }
    int level() const { return cur_; }        // +0x194
    int pending() const { return pend_; }     // +0x198
    bool committed() const { return committed_; }  // +0x1a6
    bool intensity() const { return intensity_; }  // +0x3b0
    std::string current() const { return fileName_; }
    int track() const { return track_; }      // +0x3c
    // Opens files by name ("\MUSIC\<name>"); the checks replace it.
    std::function<std::shared_ptr<const MusicFile>(const std::string&)> loader;

private:
    std::string dir_;
    std::vector<MusicTrackInfo> tracks_;
    std::function<float()> rnd_;
    MusicStream stream_;
    MusicGame game_;
    uint32_t flags_ = 8;  // +0x190: bit 2 a file is open, bit 3 enabled
    bool intensity_ = false, freeRide_ = false, committed_ = true;  // +0x3b0, +0x3b1, +0x1a6
    int cur_ = -1, pend_ = 3, track_ = -1;                          // +0x194, +0x198, +0x3c
    int variant_ = -1, prevVariant_ = -1;                           // +0x1a4, +0x1a5 (signed chars)
    float hold_ = 0, hold2_ = 0, pick_ = 0, t184_ = 0, retry_ = 0, refill_ = -1;  // +0x178 .. +0x18c
    std::string fileName_;
    int randInt(int lo, int hi);  // 0x6462e0
    void open();                  // 0x4e6610
    void queueSegment(int mode);  // 0x4e6890
    void commit();                // 0x4e6ac0
    void stop();                  // 0x4e61a0
    int pickLevel();
};

}  // namespace fuel::snd
