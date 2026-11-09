// FUEL's pad vibration rules (P89); see vibration.hpp and notes/p89-pad-vibration.md.
#include "vibration.hpp"

#include "resources.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <filesystem>
#include <fstream>
#include <sstream>

namespace fuel::vib {

float Shape::duration() const {  // 0x4e4a80: the larger of the two LAST keys' times (file order, not the max)
    const float h = high.empty() ? 0.0f : high.back().t, l = low.empty() ? 0.0f : low.back().t;
    return h > l ? h : l;
}

// Curve_Eval8 0x72f9e0 (ECX = 1): walks from the pair (k0, k1) while t is outside [t_i, t_i+1); past the
// last key, or a single key, gives the last walked key's level. Linear interpolation in x87 precision.
float evalCurve(const std::vector<Key>& k, float t) {
    const size_t n = k.size();
    if (n == 0) return 0.0f;
    size_t i = 1;
    if (n == i) return k[0].v;
    float a = k[0].t, b = k[1].t;
    while (i < n && (t < a || b <= t)) {
        ++i;
        if (i == n) return k[n - 1].v;
        a = b;
        b = k[i].t;
    }
    const double v0 = k[i - 1].v, v1 = k[i].v;
    return float(v0 + (v1 - v0) * ((double(t) - a) / (double(b) - a)));
}

int Shapes::find(const std::string& name) const {
    const auto it = byCrc.find(asoboCrc(name));
    return it == byCrc.end() ? -1 : it->second;
}

Shapes Shapes::parse(const std::string& text) {
    Shapes s;
    std::istringstream in(text);
    std::string raw;
    std::vector<bool> on{true};
    int cur = -1;
    const float k255 = 2.55f;  // 0x9dc7f8
    auto num = [](const std::string& x) { return std::strtof(x.c_str(), nullptr); };
    while (std::getline(in, raw)) {
        const size_t c = raw.find("//");
        if (c != std::string::npos) raw.resize(c);
        std::istringstream ls(raw);
        std::vector<std::string> a;
        for (std::string w; ls >> w;) a.push_back(w);
        if (a.empty()) continue;
        std::string cmd = a[0];
        for (char& ch : cmd) ch = char(std::tolower(static_cast<unsigned char>(ch)));
        if (cmd == "#ifdef" || cmd == "#ifndef") {  // the PC build defines _PC and CD_VERSION
            bool v = a.size() > 1 && (a[1] == "_PC" || a[1] == "CD_VERSION");
            if (cmd == "#ifndef") v = !v;
            on.push_back(on.back() && v);
            continue;
        }
        if (cmd == "#else" && on.size() > 1) {
            on.back() = on[on.size() - 2] && !on.back();
            continue;
        }
        if (cmd == "#endif") {
            if (on.size() > 1) on.pop_back();
            continue;
        }
        if (!on.back()) continue;
        if (cmd == "addvibration" && a.size() > 1) {  // 0x4e56f0 -> 0x4e4c40: a known name is emptied and reused
            const uint32_t crc = asoboCrc(a[1]);
            const auto it = s.byCrc.find(crc);
            if (it != s.byCrc.end()) {
                cur = it->second;
                s.list[size_t(cur)].high.clear(), s.list[size_t(cur)].low.clear(), s.list[size_t(cur)].loop = false;
            } else {
                cur = int(s.list.size());
                s.list.push_back({a[1], {}, {}, false});
                s.byCrc[crc] = cur;
            }
        } else if ((cmd == "addlowvibrationkey" || cmd == "addhighvibrationkey") && a.size() > 2 && cur >= 0) {
            const Key k{num(a[1]), float(double(num(a[2])) * double(k255))};
            (cmd[3] == 'l' ? s.list[size_t(cur)].low : s.list[size_t(cur)].high).push_back(k);
        }
    }
    return s;
}

Shapes Shapes::load(const std::string& dataDir) {
    namespace fs = std::filesystem;
    fs::path p = fs::path(dataDir) / "CommonComponent" / "Vibrations.tsc";
    if (!fs::exists(p)) {  // any case
        std::error_code ec;
        for (const auto& e : fs::directory_iterator(fs::path(dataDir) / "CommonComponent", ec)) {
            std::string n = e.path().filename().string();
            for (char& ch : n) ch = char(std::tolower(static_cast<unsigned char>(ch)));
            if (n == "vibrations.tsc") p = e.path();
        }
    }
    std::ifstream f(p, std::ios::binary);
    if (!f) {
        Shapes s;
        s.error = "cannot read " + p.string();
        return s;
    }
    std::ostringstream ss;
    ss << f.rdbuf();
    Shapes s = parse(ss.str());
    if (s.list.empty()) s.error = p.string() + ": no AddVibration";
    return s;
}

bool Manager::add(const std::string& name, int priority, float scale) { return add(asoboCrc(name), priority, scale); }

bool Manager::add(uint32_t crc, int priority, float scale) {
    const bool ok = start(crc, priority, scale);
    if (trace) trace(crc, priority, scale, ok);
    return ok;
}

bool Manager::start(uint32_t crc, int priority, float scale) {
    // 0x557b40: in game (+0xdc) and the option (+0x272); 0x4e4530: the manager on (+0x3c), cooldown <= 0.1,
    // game speed +0x3bc x +0x3c0 >= 0.95 (always 1 in the port), the shape known, a pad (not the keyboard).
    if (!gameStarted || !option || !active_ || !shapes_ || !(cooldown_ <= 0.1f)) return false;  // PK26: gameStarted
    const auto it = shapes_->byCrc.find(crc);
    if (it == shapes_->byCrc.end()) return false;
    const int shape = it->second;
    for (size_t i = 0; i < inst_.size(); ++i) {
        if (inst_[i].shape != shape) continue;
        if (uint8_t(priority) <= uint8_t(inst_[i].priority)) return false;  // byte compare (jbe)
        inst_.erase(inst_.begin() + long(i));                               // 0x4da3e0: a lower one restarts
        break;
    }
    inject(shape, priority, scale, 0.0f);
    return true;
}

void Manager::inject(int shape, int priority, float scale, float time, int loop) {  // 0x4e51c0 + 0x4e4990
    if (!shapes_ || shape < 0 || size_t(shape) >= shapes_->list.size()) return;
    const Shape& s = shapes_->list[size_t(shape)];
    Instance in;
    in.shape = shape, in.priority = priority & 0xff, in.scale = scale, in.time = time, in.duration = s.duration(), in.loop = loop < 0 ? s.loop : loop > 0;
    inst_.push_back(in);
}

Motors Manager::update(float dt) {
    Motors m;
    if (!active_) return m;  // removed from the manipulators by 0x557c90 (its stop sent 0, 0)
    if (0 < cooldown_) {
        const float c = cooldown_ - dt;
        cooldown_ = c < 0 ? 0.0f : c;
    }
    float hi = 0, lo = 0;  // local_40[pad], local_40[pad + 8]
    for (size_t i = 0; i < inst_.size();) {
        Instance& in = inst_[i];
        float t = in.time + dt;
        if (in.loop && in.duration <= t) t -= in.duration;
        in.time = t;
        if (!(t < in.duration)) {  // ended (also exactly at its length)
            inst_.erase(inst_.begin() + long(i));
            continue;
        }
        const Shape& s = shapes_->list[size_t(in.shape)];
        const float h = s.high.empty() ? 0.0f : evalCurve(s.high, t);
        hi = float(double(h) * in.scale + hi);
        const float l = s.low.empty() ? 0.0f : evalCurve(s.low, t);
        lo = float(double(l) * in.scale + lo);
        ++i;
    }
    m.highF = std::clamp(hi, 0.0f, 255.0f);
    m.lowF = std::clamp(lo, 0.0f, 255.0f);
    m.high = int(m.highF) & 0xff;  // fistp with RC = chop (control word | 0xc00)
    m.low = int(m.lowF) & 0xff;
    return m;
}

void Manager::stopAll() {  // 0x4e4060: SetVibration(pad, 0, 0) per instance, cooldown 1 s
    inst_.clear();
    cooldown_ = 1.0f;
}

void Manager::setActive(bool on) {
    if (!on && active_) stopAll();
    active_ = on;
}

// ---- check -----------------------------------------------------------------------------------------

namespace {

struct Inst {
    const char* name;
    float t, scale;
    bool loop;  // +0x11 (no shipped shape loops; set here to test the wrap)
};
struct Vec {
    std::vector<Inst> inst;
    int high, low;  // SetVibration args 2 and 3 as the emulator produced them
};
// VibrationManager_C vf17 0x4e4110 under Ghidra EmuRun on the player's FUEL.exe (notes/p89-work/vib_gen.py,
// vib.out): the instances on pad 0 at `t` (before the update), dt = 1/60, shapes from Vibrations.tsc.
// The emulator's fistp rounds to nearest, half up (pcode ignores the x87 RC field), so these bytes are
// compared with the port's levels rounded that way; the game itself truncates (control word | 0xc00).
const Vec kVecs[] = {
    {{{"IMPACT_SMALL", 0.0f, 1.0f, false}}, 4, 241},
    {{{"IMPACT_SMALL", 0.02f, 1.0f, false}}, 9, 224},
    {{{"IMPACT_SMALL", 0.05f, 1.0f, false}}, 17, 198},
    {{{"IMPACT_SMALL", 0.083f, 1.0f, false}}, 25, 170},
    {{{"IMPACT_SMALL", 0.1f, 1.0f, false}}, 30, 156},
    {{{"IMPACT_SMALL", 0.12f, 1.0f, false}}, 35, 139},
    {{{"IMPACT_SMALL", 0.14f, 1.0f, false}}, 40, 122},
    {{{"IMPACT_SMALL", 0.2f, 1.0f, false}}, 55, 71},
    {{{"IMPACT_SMALL", 0.24f, 1.0f, false}}, 65, 37},
    {{{"IMPACT_SMALL", 0.26f, 1.0f, false}}, 71, 20},
    {{{"IMPACT_SMALL", 0.3f, 1.0f, false}}, 0, 0},
    {{{"IMPACT_SMALL", 0.35f, 1.0f, false}}, 0, 0},
    {{{"IMPACT_SMALL", 0.39f, 1.0f, false}}, 0, 0},
    {{{"IMPACT_SMALL", 0.45f, 1.0f, false}}, 0, 0},
    {{{"IMPACT_SMALL", 0.55f, 1.0f, false}}, 0, 0},
    {{{"IMPACT_SMALL", 0.65f, 1.0f, false}}, 0, 0},
    {{{"IMPACT_SMALL", 0.7f, 1.0f, false}}, 0, 0},
    {{{"IMPACT_SMALL", 0.78f, 1.0f, false}}, 0, 0},
    {{{"IMPACT_SMALL", 0.79f, 1.0f, false}}, 0, 0},
    {{{"IMPACT_SMALL", 1.2f, 1.0f, false}}, 0, 0},
    {{{"IMPACT_SMALL", 2.0f, 1.0f, false}}, 0, 0},
    {{{"IMPACT_SMALL", 2.9f, 1.0f, false}}, 0, 0},
    {{{"IMPACT_SMALL", 3.1f, 1.0f, false}}, 0, 0},
    {{{"IMPACT_MEDIUM", 0.0f, 1.0f, false}}, 204, 244},
    {{{"IMPACT_MEDIUM", 0.02f, 1.0f, false}}, 204, 232},
    {{{"IMPACT_MEDIUM", 0.05f, 1.0f, false}}, 204, 213},
    {{{"IMPACT_MEDIUM", 0.083f, 1.0f, false}}, 204, 191},
    {{{"IMPACT_MEDIUM", 0.1f, 1.0f, false}}, 204, 181},
    {{{"IMPACT_MEDIUM", 0.12f, 1.0f, false}}, 204, 168},
    {{{"IMPACT_MEDIUM", 0.14f, 1.0f, false}}, 204, 155},
    {{{"IMPACT_MEDIUM", 0.2f, 1.0f, false}}, 204, 117},
    {{{"IMPACT_MEDIUM", 0.24f, 1.0f, false}}, 0, 91},
    {{{"IMPACT_MEDIUM", 0.26f, 1.0f, false}}, 0, 79},
    {{{"IMPACT_MEDIUM", 0.3f, 1.0f, false}}, 0, 53},
    {{{"IMPACT_MEDIUM", 0.35f, 1.0f, false}}, 0, 21},
    {{{"IMPACT_MEDIUM", 0.39f, 1.0f, false}}, 0, 0},
    {{{"IMPACT_MEDIUM", 0.45f, 1.0f, false}}, 0, 0},
    {{{"IMPACT_MEDIUM", 0.55f, 1.0f, false}}, 0, 0},
    {{{"IMPACT_MEDIUM", 0.65f, 1.0f, false}}, 0, 0},
    {{{"IMPACT_MEDIUM", 0.7f, 1.0f, false}}, 0, 0},
    {{{"IMPACT_MEDIUM", 0.78f, 1.0f, false}}, 0, 0},
    {{{"IMPACT_MEDIUM", 0.79f, 1.0f, false}}, 0, 0},
    {{{"IMPACT_MEDIUM", 1.2f, 1.0f, false}}, 0, 0},
    {{{"IMPACT_MEDIUM", 2.0f, 1.0f, false}}, 0, 0},
    {{{"IMPACT_MEDIUM", 2.9f, 1.0f, false}}, 0, 0},
    {{{"IMPACT_MEDIUM", 3.1f, 1.0f, false}}, 0, 0},
    {{{"IMPACT_HIGH", 0.0f, 1.0f, false}}, 255, 234},
    {{{"IMPACT_HIGH", 0.02f, 1.0f, false}}, 255, 208},
    {{{"IMPACT_HIGH", 0.05f, 1.0f, false}}, 255, 170},
    {{{"IMPACT_HIGH", 0.083f, 1.0f, false}}, 255, 128},
    {{{"IMPACT_HIGH", 0.1f, 1.0f, false}}, 255, 106},
    {{{"IMPACT_HIGH", 0.12f, 1.0f, false}}, 255, 81},
    {{{"IMPACT_HIGH", 0.14f, 1.0f, false}}, 255, 55},
    {{{"IMPACT_HIGH", 0.2f, 1.0f, false}}, 255, 21},
    {{{"IMPACT_HIGH", 0.24f, 1.0f, false}}, 255, 72},
    {{{"IMPACT_HIGH", 0.26f, 1.0f, false}}, 255, 98},
    {{{"IMPACT_HIGH", 0.3f, 1.0f, false}}, 255, 149},
    {{{"IMPACT_HIGH", 0.35f, 1.0f, false}}, 255, 213},
    {{{"IMPACT_HIGH", 0.39f, 1.0f, false}}, 247, 247},
    {{{"IMPACT_HIGH", 0.45f, 1.0f, false}}, 179, 170},
    {{{"IMPACT_HIGH", 0.55f, 1.0f, false}}, 64, 43},
    {{{"IMPACT_HIGH", 0.65f, 1.0f, false}}, 102, 85},
    {{{"IMPACT_HIGH", 0.7f, 1.0f, false}}, 159, 149},
    {{{"IMPACT_HIGH", 0.78f, 1.0f, false}}, 251, 251},
    {{{"IMPACT_HIGH", 0.79f, 1.0f, false}}, 0, 0},
    {{{"IMPACT_HIGH", 1.2f, 1.0f, false}}, 0, 0},
    {{{"IMPACT_HIGH", 2.0f, 1.0f, false}}, 0, 0},
    {{{"IMPACT_HIGH", 2.9f, 1.0f, false}}, 0, 0},
    {{{"IMPACT_HIGH", 3.1f, 1.0f, false}}, 0, 0},
    {{{"ABSORBER_1", 0.0f, 1.0f, false}}, 0, 122},
    {{{"ABSORBER_1", 0.02f, 1.0f, false}}, 0, 116},
    {{{"ABSORBER_1", 0.05f, 1.0f, false}}, 0, 106},
    {{{"ABSORBER_1", 0.083f, 1.0f, false}}, 0, 96},
    {{{"ABSORBER_1", 0.1f, 1.0f, false}}, 0, 90},
    {{{"ABSORBER_1", 0.12f, 1.0f, false}}, 0, 84},
    {{{"ABSORBER_1", 0.14f, 1.0f, false}}, 0, 78},
    {{{"ABSORBER_1", 0.2f, 1.0f, false}}, 0, 58},
    {{{"ABSORBER_1", 0.24f, 1.0f, false}}, 0, 46},
    {{{"ABSORBER_1", 0.26f, 1.0f, false}}, 0, 39},
    {{{"ABSORBER_1", 0.3f, 1.0f, false}}, 0, 27},
    {{{"ABSORBER_1", 0.35f, 1.0f, false}}, 0, 11},
    {{{"ABSORBER_1", 0.39f, 1.0f, false}}, 0, 0},
    {{{"ABSORBER_1", 0.45f, 1.0f, false}}, 0, 0},
    {{{"ABSORBER_1", 0.55f, 1.0f, false}}, 0, 0},
    {{{"ABSORBER_1", 0.65f, 1.0f, false}}, 0, 0},
    {{{"ABSORBER_1", 0.7f, 1.0f, false}}, 0, 0},
    {{{"ABSORBER_1", 0.78f, 1.0f, false}}, 0, 0},
    {{{"ABSORBER_1", 0.79f, 1.0f, false}}, 0, 0},
    {{{"ABSORBER_1", 1.2f, 1.0f, false}}, 0, 0},
    {{{"ABSORBER_1", 2.0f, 1.0f, false}}, 0, 0},
    {{{"ABSORBER_1", 2.9f, 1.0f, false}}, 0, 0},
    {{{"ABSORBER_1", 3.1f, 1.0f, false}}, 0, 0},
    {{{"CLUTCH", 0.0f, 1.0f, false}}, 0, 227},
    {{{"CLUTCH", 0.02f, 1.0f, false}}, 0, 193},
    {{{"CLUTCH", 0.05f, 1.0f, false}}, 0, 142},
    {{{"CLUTCH", 0.083f, 1.0f, false}}, 0, 86},
    {{{"CLUTCH", 0.1f, 1.0f, false}}, 255, 57},
    {{{"CLUTCH", 0.12f, 1.0f, false}}, 255, 23},
    {{{"CLUTCH", 0.14f, 1.0f, false}}, 238, 0},
    {{{"CLUTCH", 0.2f, 1.0f, false}}, 85, 0},
    {{{"CLUTCH", 0.24f, 1.0f, false}}, 0, 0},
    {{{"CLUTCH", 0.26f, 1.0f, false}}, 0, 0},
    {{{"CLUTCH", 0.3f, 1.0f, false}}, 0, 0},
    {{{"CLUTCH", 0.35f, 1.0f, false}}, 0, 0},
    {{{"CLUTCH", 0.39f, 1.0f, false}}, 0, 0},
    {{{"CLUTCH", 0.45f, 1.0f, false}}, 0, 0},
    {{{"CLUTCH", 0.55f, 1.0f, false}}, 0, 0},
    {{{"CLUTCH", 0.65f, 1.0f, false}}, 0, 0},
    {{{"CLUTCH", 0.7f, 1.0f, false}}, 0, 0},
    {{{"CLUTCH", 0.78f, 1.0f, false}}, 0, 0},
    {{{"CLUTCH", 0.79f, 1.0f, false}}, 0, 0},
    {{{"CLUTCH", 1.2f, 1.0f, false}}, 0, 0},
    {{{"CLUTCH", 2.0f, 1.0f, false}}, 0, 0},
    {{{"CLUTCH", 2.9f, 1.0f, false}}, 0, 0},
    {{{"CLUTCH", 3.1f, 1.0f, false}}, 0, 0},
    {{{"DAMAGED_CAR", 0.0f, 1.0f, false}}, 0, 74},
    {{{"DAMAGED_CAR", 0.02f, 1.0f, false}}, 0, 72},
    {{{"DAMAGED_CAR", 0.05f, 1.0f, false}}, 0, 68},
    {{{"DAMAGED_CAR", 0.083f, 1.0f, false}}, 0, 64},
    {{{"DAMAGED_CAR", 0.1f, 1.0f, false}}, 0, 62},
    {{{"DAMAGED_CAR", 0.12f, 1.0f, false}}, 0, 59},
    {{{"DAMAGED_CAR", 0.14f, 1.0f, false}}, 0, 57},
    {{{"DAMAGED_CAR", 0.2f, 1.0f, false}}, 0, 49},
    {{{"DAMAGED_CAR", 0.24f, 1.0f, false}}, 0, 44},
    {{{"DAMAGED_CAR", 0.26f, 1.0f, false}}, 0, 41},
    {{{"DAMAGED_CAR", 0.3f, 1.0f, false}}, 0, 36},
    {{{"DAMAGED_CAR", 0.35f, 1.0f, false}}, 0, 30},
    {{{"DAMAGED_CAR", 0.39f, 1.0f, false}}, 0, 25},
    {{{"DAMAGED_CAR", 0.45f, 1.0f, false}}, 0, 17},
    {{{"DAMAGED_CAR", 0.55f, 1.0f, false}}, 0, 14},
    {{{"DAMAGED_CAR", 0.65f, 1.0f, false}}, 0, 17},
    {{{"DAMAGED_CAR", 0.7f, 1.0f, false}}, 0, 18},
    {{{"DAMAGED_CAR", 0.78f, 1.0f, false}}, 0, 20},
    {{{"DAMAGED_CAR", 0.79f, 1.0f, false}}, 0, 21},
    {{{"DAMAGED_CAR", 1.2f, 1.0f, false}}, 0, 37},
    {{{"DAMAGED_CAR", 2.0f, 1.0f, false}}, 0, 27},
    {{{"DAMAGED_CAR", 2.9f, 1.0f, false}}, 0, 33},
    {{{"DAMAGED_CAR", 3.1f, 1.0f, false}}, 0, 0},
    {{{"ABSORBER_1", 0.05f, 0.37f, false}}, 0, 39},
    {{{"ABSORBER_2", 0.1f, 0.25f, false}, {"ABSORBER_3", 0.0f, 0.5f, false}}, 0, 84},
    {{{"IMPACT_HIGH", 0.05f, 1.0f, false}, {"IMPACT_MEDIUM", 0.02f, 1.0f, false}}, 255, 255},
    {{{"CLUTCH", 0.11f, 0.6f, false}, {"IMPACT_SMALL", 0.2f, 0.9f, false}}, 203, 88},
    {{{"CLUTCH", 0.24f, 1.0f, true}}, 0, 244},
    {{{"ABSORBER_1", 0.55f, 1.0f, true}}, 0, 74},
    {{{"DAMAGED_CAR", 3.1f, 1.0f, true}}, 0, 62},
    {{{"IMPACT_HIGH", 0.79f, 0.8f, true}}, 204, 197},
};

}  // namespace

int vibrationRulesCheck(const std::string& dataDir) {
    int fails = 0, n = 0;
    const Shapes s = Shapes::load(dataDir);
    if (!s.error.empty()) {
        std::printf("vibration: %s\n", s.error.c_str());
        return 1;
    }
    std::printf("Vibrations.tsc: %zu shapes:", s.list.size());
    for (const Shape& sh : s.list) std::printf(" %s (%.2f s)", sh.name.c_str(), sh.duration());
    std::printf("\n");
    // 1. vf17 against the emulated vectors.
    const float dt = 0.016666668f;
    int vf = 0;
    for (const Vec& v : kVecs) {
        Manager m(&s);
        for (const Inst& i : v.inst) m.inject(s.find(i.name), 1, i.scale, i.t, i.loop);
        const Motors o = m.update(dt);
        const int hr = int(std::floor(o.highF + 0.5f)), lr = int(std::floor(o.lowF + 0.5f));  // the emulator rounds half up
        ++n;
        if (hr != v.high || lr != v.low) {
            ++fails, ++vf;
            std::printf("  MISMATCH");
            for (const Inst& i : v.inst) std::printf(" %s@%g*%g", i.name, i.t, i.scale);
            std::printf(": high %d low %d (%.4f %.4f), emulated %d %d\n", hr, lr, o.highF, o.lowF, v.high, v.low);
        }
    }
    const int nv = int(sizeof(kVecs) / sizeof(kVecs[0]));
    std::printf("vf17 0x4e4110 vs the emulator: %d of %d vectors match\n", nv - vf, nv);
    // 2. the add rules of 0x4e4530 / 0x4e4060 / 0x557c90.
    auto expect = [&](bool good, const char* what) {
        ++n;
        if (!good) ++fails;
        std::printf("  %-72s %s\n", what, good ? "ok" : "FAIL");
    };
    Manager m(&s);
    expect(m.add("IMPACT_SMALL", 1, 1), "IMPACT_SMALL starts");
    m.update(dt);
    expect(!m.add("IMPACT_SMALL", 1, 1), "the same shape at the same priority keeps playing");
    expect(m.add("IMPACT_SMALL", 2, 1) && m.playing() == 1, "a higher priority restarts it");
    expect(m.add("CLUTCH", 1, 1) && m.playing() == 2, "another shape plays alongside");
    expect(!m.add("NO_SUCH_VIBRATION", 3, 1), "an unknown name is ignored");
    m.stopAll();
    expect(m.playing() == 0 && !m.add("IMPACT_HIGH", 3, 1), "stop: nothing plays, the cooldown blocks a new one");
    int frames = 0;
    while (!m.add("IMPACT_HIGH", 3, 1) && frames < 200) m.update(dt), ++frames;
    std::printf("  cooldown after a stop: %d updates (%.3f s)\n", frames, frames * dt);
    expect(frames == 55, "vibrations start again ~0.9 s after a stop (55 float updates of 1/60)");
    m.setActive(false);
    expect(!m.add("IMPACT_HIGH", 3, 1) && m.update(dt) == Motors{}, "paused (0x557c90): silent, nothing starts");
    m.setActive(true);
    m.option = false;
    for (int k = 0; k < 120; ++k) m.update(dt);
    expect(!m.add("IMPACT_HIGH", 3, 1), "vibration option off (game +0x272): nothing starts");
    m.option = true;
    expect(m.add("IMPACT_HIGH", 3, 1), "option on again: it starts");
    // 3. each shipped shape as FUEL plays it (60 Hz, bytes sent: left = LOW, right = HIGH).
    for (const Shape& sh : s.list) {
        Manager p(&s);
        p.add(sh.name, 1, 1);
        std::printf("  %-13s left/right every 3rd frame:", sh.name.c_str());
        for (int k = 0; k < 400 && p.playing(); ++k) {
            const Motors o = p.update(dt);
            if (k % 3 == 0) std::printf(" %d/%d", o.low, o.high);
        }
        std::printf("\n");
    }
    std::printf("vibration rules: %d checks, %d failures\n", n, fails);
    return fails;
}

// ---- [nitro] (vibration.hpp)
Motors NitroRumble::step(float dt, float amount, bool on) {
    const float n = on && dt > 0 ? std::clamp(amount, 0.0f, 1.0f) : 0.0f;
    if (n > 0.05f && prev <= 0.05f) pulse = 0.25f;
    prev = n;
    pulse = on ? std::max(0.0f, pulse - dt) : 0.0f;
    const float p = pulse / 0.25f;
    Motors m;
    m.lowF = std::max(0.55f * p, 0.12f * n), m.highF = std::max(0.35f * p, 0.22f * n);
    m.low = int(std::lround(255.0f * m.lowF)), m.high = int(std::lround(255.0f * m.highF));
    return m;
}

Motors maxCombine(const Motors& a, const Motors& b) {
    Motors m = a;
    m.low = std::max(a.low, b.low), m.high = std::max(a.high, b.high);
    m.lowF = std::max(a.lowF, b.lowF), m.highF = std::max(a.highF, b.highF);
    return m;
}

}  // namespace fuel::vib
