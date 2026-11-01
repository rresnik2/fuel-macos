#include "water.hpp"
#include "terrain_height.hpp"

#include "genworld_noise.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>

namespace fuel {

namespace {

// StockLakes_GenLakes (FUEL.exe 0x6ffdd0 .. 0x70034b) calls StockLakes_AddFloodLake (0x6ffc90)
// or StockLakes_AddRectLake (0x6ffd20) once per lake, each call preceded by four
// `fld dword [abs]` loads of x, level, z and radius from .rdata.
constexpr uint32_t kGenLakes = 0x6ffdd0, kGenLakesEnd = 0x70034c, kAddFlood = 0x6ffc90, kAddRect = 0x6ffd20;

std::vector<Lake> readLakeTable(const DpcFile& world) {
    std::vector<Lake> out;
    const size_t n = (kGenLakesEnd - kGenLakes + 3) / 4;
    const std::vector<float> words = readEngineFloats(world, kGenLakes, n);
    if (words.size() != n) return out;
    std::vector<uint8_t> code(n * 4);
    std::memcpy(code.data(), words.data(), code.size());
    auto u32 = [&](size_t o) { return uint32_t(code[o]) | uint32_t(code[o + 1]) << 8 | uint32_t(code[o + 2]) << 16 | uint32_t(code[o + 3]) << 24; };
    std::vector<uint32_t> loads;
    struct Call {
        uint32_t addr[4];
        bool flood;
    };
    std::vector<Call> calls;
    bool keptFirst = false;
    for (size_t i = 0; i + 6 <= kGenLakesEnd - kGenLakes;) {
        if (code[i] == 0xD9 && code[i + 1] == 0x05) {
            loads.push_back(u32(i + 2));
            // `fst` (not fstp) right after the first load keeps it for a second argument.
            if (loads.size() == 1) keptFirst = code[i + 6] == 0xD9 && code[i + 7] == 0x54 && code[i + 8] == 0x24;
            i += 6;
        } else if (code[i] == 0xE8) {
            const uint32_t target = uint32_t(kGenLakes + i + 5 + u32(i + 1));
            if (target == kAddFlood || target == kAddRect) {
                if (loads.size() == 4) calls.push_back({{loads[0], loads[1], loads[2], loads[3]}, target == kAddFlood});
                else if (loads.size() == 3 && keptFirst) calls.push_back({{loads[0], loads[1], loads[0], loads[2]}, target == kAddFlood});  // x == z
                else return {};
                loads.clear();
            }
            i += 5;
        } else {
            ++i;
        }
    }
    if (calls.empty()) return out;
    uint32_t lo = 0xFFFFFFFFu, hi = 0;
    for (const Call& c : calls)
        for (uint32_t a : c.addr) lo = std::min(lo, a), hi = std::max(hi, a);
    if (hi - lo > 4096) return out;
    const std::vector<float> data = readEngineFloats(world, lo, (hi - lo) / 4 + 1);
    if (data.empty()) return out;
    for (const Call& c : calls) {
        float v[4];
        for (int k = 0; k < 4; ++k) {
            if ((c.addr[k] - lo) % 4 != 0) return {};
            v[k] = data[(c.addr[k] - lo) / 4];
        }
        Lake L;
        L.x = v[0], L.level = v[1], L.z = v[2], L.radius = v[3];
        L.flood = c.flood;
        if (!(L.radius > 0)) return {};
        out.push_back(L);
    }
    return out;
}

}  // namespace

bool Lake::contains(double px, double pz) const {
    // ValidityMap_WorldToCell 0x659ab0 (float32, floor) + bit test.
    const int32_t i = int32_t(std::floor(float(float(px) - minX) * inv)), j = int32_t(std::floor(float(float(pz) - minZ) * inv));
    if (i < 0 || j < 0 || i >= w || j >= h) return false;
    const uint32_t b = uint32_t(i) + uint32_t(w) * uint32_t(j);
    return (mask[b >> 5] >> (b & 31)) & 1u;
}

Lakes::Lakes(const DpcFile& world, std::function<double(double, double)> baseHeight) {
    lakes_ = readLakeTable(world);
    if (lakes_.empty()) {
        std::fprintf(stderr, "fuel: lake table not found in FUEL.exe; no lakes\n");
        return;
    }
    for (Lake& L : lakes_) {
        // DataLake_InitGrid 0x6ff450: the square x +- R, z +- R in cells of 100 m (basins) or R/16 (ponds).
        L.cell = L.flood ? 100.0f : L.radius * 0.0625f;
        L.inv = 1.0f / L.cell;
        L.minX = L.x - L.radius, L.maxX = L.x + L.radius, L.minZ = L.z - L.radius, L.maxZ = L.z + L.radius;
        L.w = int32_t(std::floor((L.maxX - L.minX) / L.cell));
        L.h = int32_t(std::floor((L.maxZ - L.minZ) / L.cell));
        L.mask.assign((size_t(L.w) * size_t(L.h) + 31) >> 5, 0u);
        if (!L.flood) {  // 0x6ffd20: every cell
            std::fill(L.mask.begin(), L.mask.end(), 0xFFFFFFFFu);
            continue;
        }
        // DataLake_FloodFill 0x6ff600 with the predicate 0x6ffbc0 (a 10 x 10 sample of the cell
        // below the level). Cells are marked before the test, so the dry rim is in the mask too.
        auto bit = [&](int32_t i, int32_t j) { return (L.mask[(uint32_t(i) + uint32_t(L.w) * uint32_t(j)) >> 5] >> ((uint32_t(i) + uint32_t(L.w) * uint32_t(j)) & 31)) & 1u; };
        auto set = [&](int32_t i, int32_t j) { const uint32_t b = uint32_t(i) + uint32_t(L.w) * uint32_t(j); L.mask[b >> 5] |= 1u << (b & 31); };
        auto wet = [&](int32_t i, int32_t j) {
            for (int jj = 0; jj < 10; ++jj)
                for (int ii = 0; ii < 10; ++ii) {
                    const double X = L.minX + i * L.cell + 5 + 10 * ii, Z = L.minZ + j * L.cell + 5 + 10 * jj;
                    if (baseHeight(X, Z) < L.level) return true;
                }
            return false;
        };
        const int32_t i0 = int32_t(std::floor((L.x - L.minX) * L.inv)), j0 = int32_t(std::floor((L.z - L.minZ) * L.inv));
        if (i0 < 0 || j0 < 0 || i0 >= L.w || j0 >= L.h) continue;
        set(i0, j0);
        if (!wet(i0, j0)) continue;
        std::vector<std::pair<int32_t, int32_t>> stack{{i0, j0}};
        while (!stack.empty()) {
            const auto [i, j] = stack.back();
            stack.pop_back();
            const std::pair<int32_t, int32_t> nb[4] = {{i + 1, j}, {i - 1, j}, {i, j + 1}, {i, j - 1}};
            for (const auto& [a, b] : nb) {
                if (a < 0 || b < 0 || a >= L.w || b >= L.h || bit(a, b)) continue;
                set(a, b);
                if (wet(a, b)) stack.push_back({a, b});
            }
        }
    }
}

double Lakes::depthAbove(double x, double z, double h) const {
    double best = 0;
    for (const Lake& L : lakes_)
        if (x >= L.minX && x <= L.maxX && z >= L.minZ && z <= L.maxZ && L.contains(x, z)) best = std::max(best, double(L.level) - h);
    return best;
}

double Lakes::surfaceAt(double x, double z, double h) const {
    double best = 0, s = -1e9;
    for (const Lake& L : lakes_)
        if (x >= L.minX && x <= L.maxX && z >= L.minZ && z <= L.maxZ && L.contains(x, z) && double(L.level) - h > best)
            best = double(L.level) - h, s = double(L.level) - 2.0;
    return s;
}

WaterPoint waterAt(const Lakes& lakes, double x, double z, double h) {
    // GenWorld_WaterQuery 0x6acf80 without rivers: ctx+0x1ac = max(D = 0, lake depth),
    // ctx+0x1b0 = max(A = 0, 2.0); ctx+0x30c = 0 (the lake flag is always 0).
    WaterPoint w;
    w.depth = float(lakes.depthAbove(x, z, h));
    w.base = 2.0f;
    w.fade = waterFade(w.depth, w.base);
    return w;
}

float waterFade(float depth, float base) {
    // 0x6b9560 tail: wet-shore fade.
    if (depth <= 0 || depth <= base - 1) return 1;
    const float u = (depth - (base - 1)) * 2;
    return u > 1 ? 0.66999996f : 1 - 0.33f * u;
}

WaterPoint waterAt(const Lakes* lakes, const TerrainDetail* detail, double x, double z, double h) {
    // 0x6acf80: (h1, A, flagR) = river 0x7a07d0; (h2, B = 2, flagL = 0) = lakes 0x65a480.
    WaterPoint w;
    TerrainDetail::River r;
    if (detail) r = detail->river(x, z, h);
    const double h2 = lakes ? lakes->depthAbove(x, z, h) : 0.0;
    const double A = r.flag ? r.bed : 0.0;
    w.base = float(std::max(A, 2.0));
    w.depth = float(std::max(r.flag ? r.depth : 0.0, h2));
    if (!r.flag) w.riverShare = 0;
    else {
        const double dA = std::max(r.depth - A, 0.0), dB = std::max(h2 - 2.0, 0.0);
        w.riverShare = dA + dB > double(1e-4f) ? float(dA / (dA + dB)) : 1.0f;
    }
    w.fade = waterFade(w.depth, w.base);
    return w;
}

}  // namespace fuel
