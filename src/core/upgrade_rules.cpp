// [DEVIATION, the user's request 2026-11-06] Vehicle upgrades: the parts of core/upgrades.hpp the career needs (category
// names, the stage list, prices and star gates), in fuelcore so core/career.cpp (and the tools that link only fuelcore:
// fuelsaveimport) can use them; the physics side (apply, the stage tables) stays in upgrades.cpp (fuelphysics).
// Moved here unchanged from upgrades.cpp by U2 (notes upg2.md); categoryFromKey is U2's.
#include "core/upgrades.hpp"

#include <algorithm>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <sstream>

namespace fuel::upg {

namespace {

std::string lower(std::string s) {
    for (char& c : s) c = char(std::tolower(static_cast<unsigned char>(c)));
    while (!s.empty() && std::isspace(static_cast<unsigned char>(s.back()))) s.pop_back();
    size_t a = 0;
    while (a < s.size() && std::isspace(static_cast<unsigned char>(s[a]))) ++a;
    return s.substr(a);
}

}  // namespace

const char* categoryName(int c) {
    static const char* const k[] = {"engine", "suspension", "tires", "weight", "nitro power", "nitro tank"};
    return c >= 0 && c < kCategories ? k[c] : "?";
}
const char* categoryKey(int c) {
    static const char* const k[] = {"e", "s", "t", "w", "np", "nt"};
    return c >= 0 && c < kCategories ? k[c] : "?";
}
int categoryFromKey(const std::string& name) {
    std::string n = lower(name);
    for (char& ch : n)
        if (ch == '_' || ch == '-') ch = ' ';
    for (int c = 0; c < kCategories; ++c)
        if (n == categoryKey(c) || n == categoryName(c)) return c;
    if (n == "tyres" || n == "tyre" || n == "tire") return Tires;
    if (n == "nitrous power" || n == "nitro") return NitroPower;
    if (n == "nitrous tank" || n == "tank") return NitroTank;
    return -1;
}

bool Stages::stock() const {
    for (int x : s)
        if (x != 0) return false;
    return true;
}

bool parseStages(const std::string& spec, Stages& out, std::string* err) {
    const std::string v = lower(spec);
    Stages r;
    if (v == "max" || v == "all") {
        r.s.fill(kMaxStage);
        out = r;
        return true;
    }
    if (v == "stock" || v.empty()) {
        out = r;
        return true;
    }
    std::stringstream ss(v);
    int k = 0;
    for (std::string x; std::getline(ss, x, ',');) {
        if (k >= kCategories) {
            if (err) *err = "more than 6 values";
            return false;
        }
        x = lower(x);
        char* end = nullptr;
        const long n = std::strtol(x.c_str(), &end, 10);
        if (x.empty() || *end != 0 || n < 0 || n > kMaxStage) {
            if (err) *err = "stage '" + x + "' is not 0..3";
            return false;
        }
        r[k++] = int(n);
    }
    out = r;
    return true;
}

std::string formatStages(const Stages& s) {
    std::string r;
    for (int c = 0; c < kCategories; ++c) r += (c ? "," : "") + std::to_string(s[c]);
    return r;
}

// ---- prices and gates

int price(int level, int category, int stage) {
    if (stage < 1 || stage > kMaxStage || category < 0 || category >= kCategories) return 0;
    static const int kBase[5][3] = {{500, 1500, 4000}, {800, 2500, 6500}, {1200, 4000, 10000}, {2000, 6500, 16000}, {3000, 10000, 25000}};
    static const float kFactor[kCategories] = {1.40f, 1.00f, 1.00f, 1.10f, 0.90f, 0.85f};  // engine, suspension, tires, weight, np, nt
    const int l = std::clamp(level, 1, 5) - 1;
    const float p = float(kBase[l][stage - 1]) * kFactor[category];
    return int(std::lround(p / 50.0f)) * 50;
}

int starsNeeded(int stage) { return stage >= 3 ? 45 : stage == 2 ? 20 : 0; }

}  // namespace fuel::upg
