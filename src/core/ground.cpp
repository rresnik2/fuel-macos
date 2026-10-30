#include "ground.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <map>
#include <stdexcept>

namespace fuel {

namespace {

constexpr uint32_t kGenWorldClass = 0x31FA94AE;

const char* const kSurfaceNames[kSurfaceTypes] = {
    "EARTH", "EARTH2", "DIRT_FAIRLY_ROUGH", "STONE", "STONE2", "PEBBLE",
    "SAND", "SAND2", "SNOW", "SNOW2", "PLOUGH", "ASPHALT",
    "ASPHALT2", "DIRT_FLAT", "DIRT_ROUGH", "ROCK", "ROCK2", "STEPPENL",
    "DESERTNL", "HILLSNL", "ROCKNL", "PLAINNL", "GRASS", "GRASS2",
    "CRACKED", "DIRT_FAIRLY_FLAT", "BIG_STONE", "BIG_STONE2", "STEEP_EARTH", "STEEP_STONE",
    "HUMIDEARTH", "HUMIDEARTH2", "ROUGHEARTH", "ROUGHEARTH2", "ROUGHROCK", "ROUGHROCK2",
    "WETSAND", "WETSAND2", "ROUGHSAND", "ROUGHSAND2"};

// Palette slot i is the "GenWorld Colors" entry whose date key is kPaletteDates[i]
// (the order the game's loader asks for them).
const char* const kPaletteDates[] = {
    "04/12/2007 17:39", "04/12/2007 17:40", "04/12/2007 17:41", "04/12/2007 17:42",
    "04/12/2007 17:43", "04/12/2007 17:44", "04/12/2007 17:45", "04/12/2007 17:46",
    "04/12/2007 17:47", "04/12/2007 17:48", "04/12/2007 17:49", "04/12/2007 17:50",
    "04/12/2007 17:51", "04/12/2007 17:52", "04/12/2007 17:53", "04/12/2007 17:54",
    "04/12/2007 17:55", "04/12/2007 17:56", "28/07/2008 16:59", "28/07/2008 17:00",
    "04/12/2007 17:57", "04/12/2007 18:26", "28/07/2008 17:01", "28/07/2008 17:02",
    "04/12/2007 17:58", "04/12/2007 17:59", "28/07/2008 17:03", "28/07/2008 17:04",
    "04/12/2007 18:00", "04/12/2007 18:01", "28/07/2008 17:05", "28/07/2008 17:06",
    "04/12/2007 18:02", "04/12/2007 18:03", "28/07/2008 17:07", "28/07/2008 17:08",
    "04/12/2007 18:04", "04/12/2007 18:05", "04/12/2007 18:06", "04/12/2007 18:07",
    "04/12/2007 18:08", "04/12/2007 18:09", "04/12/2007 18:10", "04/12/2007 18:11",
    "04/12/2007 18:12", "04/12/2007 18:13", "04/12/2007 18:14", "04/12/2007 18:15",
    "04/12/2007 18:16", "04/12/2007 18:17", "04/12/2007 18:18", "04/12/2007 18:19",
    "04/12/2007 18:20", "04/12/2007 18:21", "04/12/2007 18:22", "04/12/2007 18:23",
    "04/12/2007 18:24", "04/12/2007 18:25", "29/08/2008 15:34", "28/07/2008 17:13",
    "28/07/2008 17:14", "28/07/2008 17:15", "28/07/2008 17:16", "28/07/2008 17:17",
    "28/07/2008 17:18", "28/07/2008 17:19", "28/07/2008 17:20", "28/07/2008 17:21",
    "28/07/2008 17:22", "28/07/2008 17:23", "28/07/2008 17:24", "28/07/2008 17:25",
    "28/07/2008 17:26", "28/07/2008 17:27", "28/07/2008 17:28", "28/07/2008 17:29",
    "28/07/2008 17:30", "28/07/2008 17:31", "28/07/2008 17:32", "28/07/2008 17:33",
    "28/07/2008 17:34", "28/07/2008 17:35", "28/07/2008 17:36", "28/07/2008 17:37",
    "28/07/2008 17:38", "28/07/2008 17:39", "28/07/2008 17:40", "28/07/2008 17:41",
    "28/07/2008 17:42", "28/07/2008 17:43", "28/07/2008 17:44", "28/07/2008 17:45",
    "28/07/2008 17:46", "28/07/2008 17:47", "28/07/2008 17:48", "28/07/2008 17:49",
    "28/07/2008 17:50", "28/07/2008 17:51", "28/07/2008 17:52", "28/07/2008 17:53",
    "28/07/2008 17:54", "28/07/2008 17:55", "28/07/2008 17:56", "28/07/2008 17:57",
    "28/07/2008 17:58", "28/07/2008 17:59", "28/07/2008 18:00", "28/07/2008 18:01",
    "28/07/2008 18:02", "28/07/2008 18:03", "28/07/2008 18:04", "04/12/2007 18:34",
    "04/12/2007 18:27", "04/12/2007 18:28", "04/12/2007 18:29", "04/12/2007 18:30",
    "04/12/2007 18:31", "04/12/2007 18:32", "04/12/2007 18:33", "15/10/2008 11:46",
    "15/10/2008 11:47", "15/10/2008 11:48", "15/10/2008 11:49"};

float smoothstep(float e0, float e1, float x) {
    if (e0 == e1) return x >= e1 ? 1.0f : 0.0f;
    const float t = std::clamp((x - e0) / (e1 - e0), 0.0f, 1.0f);
    return t * t * (3 - 2 * t);
}

uint32_t rd32(const std::vector<uint8_t>& b, size_t o) {
    if (o + 4 > b.size()) throw std::runtime_error("GenWorld_Z body too short");
    uint32_t v;
    std::memcpy(&v, &b[o], 4);
    return v;
}

// Ground classes: (surface type, palette slot). Palette picks are inferred from
// the colours (see the reverse-engineering notes).
enum Class { Snow, Rock, SteepStone, Sand, Dune, ClayDry, ClayWet, Chalk, Volcanic, Burnt, Wet, Forest, Grass, DryGrass, Earth, Dirt, kClasses };
const int kClassSurface[kClasses] = {8, 15, 29, 6, 38, 24, 30, 3, 26, 1, 36, 23, 22, 17, 0, 13};
const int kClassPalette[kClasses] = {106, 14, 16, 8, 8, 27, 27, 30, 16, 67, 0, 5, 1, 6, 2, 17};

struct Layers {
    float humidity, clay, volcano, chalk, forest, soil, incinerated, flooded, sanded, south;
};

// Port heuristic: class weights from height (m), slope (rise/run) and the layers.
void classify(float h, float s, const Layers& L, float w[kClasses]) {
    const float hum = L.humidity, ny2 = 1.0f / (1.0f + s * s);
    const float snowline = 2700.0f - 600.0f * hum;
    const float flat = smoothstep(0.6f, 0.25f, s);
    const float treecut = 1.0f - smoothstep(2600.0f - 500.0f * hum - 200.0f, 2800.0f - 500.0f * hum, h);
    w[Snow] = smoothstep(snowline - 150, snowline + 150, h) * smoothstep(1.0f, 0.6f, s) * 3.0f;
    w[Rock] = smoothstep(0.55f, 1.0f, s) * 2.0f + (1.0f - L.soil) * smoothstep(0.3f, 0.6f, s);
    w[SteepStone] = smoothstep(1.0f, 1.6f, s) * 2.0f;
    w[Sand] = L.sanded * flat * 2.0f;
    w[Dune] = L.south * (1.0f - hum) * flat;
    w[ClayDry] = L.clay * (1.0f - hum) * 1.5f;
    w[ClayWet] = L.clay * hum * 1.5f;
    w[Chalk] = L.chalk * 1.5f;
    w[Volcanic] = L.volcano * 1.5f;
    w[Burnt] = L.incinerated * 1.2f;
    w[Wet] = L.flooded * flat * 1.5f;
    w[Forest] = L.forest * hum * ny2 * treecut * 1.2f;
    w[Grass] = smoothstep(0.35f, 0.7f, hum) * treecut * ny2;
    w[DryGrass] = smoothstep(0.15f, 0.4f, hum) * smoothstep(0.75f, 0.45f, hum) * treecut * 0.8f;
    w[Earth] = smoothstep(0.5f, 0.1f, hum) * 0.7f;
    w[Dirt] = 0.15f;
}

// A layer map resampled (bilinear) at world sample (i, j) of an n x n grid.
float sampleLayer(const Raster* r, uint32_t i, uint32_t j, uint32_t n) {
    if (r == nullptr) return 0.0f;
    const float fx = std::clamp((i + 0.5f) * r->width / n - 0.5f, 0.0f, float(r->width - 1));
    const float fz = std::clamp((j + 0.5f) * r->height / n - 0.5f, 0.0f, float(r->height - 1));
    const uint32_t x0 = uint32_t(fx), z0 = uint32_t(fz);
    const uint32_t x1 = std::min(x0 + 1, r->width - 1), z1 = std::min(z0 + 1, r->height - 1);
    const float tx = fx - x0, tz = fz - z0;
    const float a = r->at(x0, z0) + (r->at(x1, z0) - r->at(x0, z0)) * tx;
    const float b = r->at(x0, z1) + (r->at(x1, z1) - r->at(x0, z1)) * tx;
    return a + (b - a) * tz;
}

}  // namespace

std::vector<std::array<float, 4>> loadGroundPalette(const DpcFile& world) {
    std::vector<std::array<float, 4>> out(std::size(kPaletteDates), std::array<float, 4>{0, 0, 0, 0});
    const std::vector<uint32_t> gens = world.ids(kGenWorldClass);
    if (gens.empty()) return out;
    const auto gen = world.body(*world.find(gens.front()));
    if (!gen) return out;
    // Palette: a text script of ItemColor "<date>" R G B entries (UserDefine_Z id at GenWorld_Z body +4).
    std::map<std::string, std::array<uint8_t, 3>> byDate;
    if (const DpcFile::Entry* pe = world.find(rd32(*gen, 4))) {
        if (const auto body = world.body(*pe)) {
            const std::string text(body->begin() + std::min<size_t>(4, body->size()), body->end());
            size_t p = 0;
            while ((p = text.find('"', p)) != std::string::npos) {
                const size_t q = text.find('"', p + 1);
                if (q == std::string::npos) break;
                unsigned r, g, b;
                if (std::sscanf(text.c_str() + q + 1, " %u %u %u", &r, &g, &b) == 3)
                    byDate[text.substr(p + 1, q - p - 1)] = {uint8_t(r), uint8_t(g), uint8_t(b)};
                p = q + 1;
            }
        }
    }
    // The loader (0x6afed0 -> 0x6afe40) scales by 1/255 ([0x9dbfcc]); a slot without an entry keeps its
    // previous value (Unknown; alpha 0 marks it here).
    for (size_t i = 0; i < out.size(); ++i)
        if (auto it = byDate.find(kPaletteDates[i]); it != byDate.end())
            out[i] = {it->second[0] / 255.0f, it->second[1] / 255.0f, it->second[2] / 255.0f, 1.0f};
    return out;
}

const char* surfaceName(int index) { return index >= 0 && index < kSurfaceTypes ? kSurfaceNames[index] : "?"; }

GroundMaps buildGroundMaps(const DpcFile& world, const Heightfield& heights, uint32_t size) {
    const std::vector<uint32_t> gens = world.ids(kGenWorldClass);
    if (gens.empty()) throw std::runtime_error("no GenWorld_Z");
    const auto gen = world.body(*world.find(gens.front()));
    if (!gen) throw std::runtime_error("GenWorld_Z has no body");
    // GenWorld_Z body: u32 node, u32 palette (UserDefine_Z), u32 roads, u32 n, maps[n], u32 m, ground textures[m].
    const uint32_t nMaps = rd32(*gen, 12);
    std::vector<uint32_t> maps;
    for (uint32_t i = 0; i < nMaps; ++i) maps.push_back(rd32(*gen, 16 + size_t(i) * 4));
    const size_t texOff = 16 + size_t(nMaps) * 4;
    GroundMaps out;
    const uint32_t nTex = rd32(*gen, texOff);
    for (uint32_t i = 0; i < nTex && i < uint32_t(kSurfaceTypes); ++i) out.textures[i] = rd32(*gen, texOff + 4 + size_t(i) * 4);

    const std::vector<std::array<float, 4>> pal = loadGroundPalette(world);
    float palette[kClasses][3];
    for (int c = 0; c < kClasses; ++c) {
        const int slot = kClassPalette[c];
        for (int k = 0; k < 3; ++k) palette[c][k] = pal[size_t(slot)][3] > 0 ? pal[size_t(slot)][size_t(k)] : 0.5f;
    }

    // Layers by map slot (names from the game's debug HUD; see the notes).
    std::vector<Raster> rasters(maps.size());
    auto layer = [&](size_t slot) -> const Raster* {
        if (slot >= maps.size() || maps[slot] == 0) return nullptr;
        if (rasters[slot].raw.empty()) {
            const DpcFile::Entry* e = world.find(maps[slot]);
            const auto body = e ? world.body(*e) : std::nullopt;
            if (!body) return nullptr;
            rasters[slot] = decodeRaster(*body);
        }
        return &rasters[slot];
    };
    const Raster* humidity = layer(2);
    const Raster* clay = layer(3);
    const Raster* volcano = layer(5);
    const Raster* chalk = layer(6);
    const Raster* forest = layer(7);
    const Raster* soil = layer(9);
    const Raster* incinerated = layer(13);
    const Raster* flooded = layer(15);
    const Raster* sanded = layer(16);
    const Raster* south = layer(20);

    out.size = size;
    out.colour.resize(size_t(size) * size * 4);
    out.surface.resize(size_t(size) * size);
    const float step = 140000.0f / float(size);
    for (uint32_t j = 0; j < size; ++j) {
        for (uint32_t i = 0; i < size; ++i) {
            const float x = -70000.0f + (i + 0.5f) * step, z = -70000.0f + (j + 0.5f) * step;
            const float h = heights.heightAt(x, z);
            const float dx = (heights.heightAt(x + step, z) - heights.heightAt(x - step, z)) / (2 * step);
            const float dz = (heights.heightAt(x, z + step) - heights.heightAt(x, z - step)) / (2 * step);
            const Layers L{sampleLayer(humidity, i, j, size), sampleLayer(clay, i, j, size),       sampleLayer(volcano, i, j, size),
                           sampleLayer(chalk, i, j, size),    sampleLayer(forest, i, j, size),     sampleLayer(soil, i, j, size),
                           sampleLayer(incinerated, i, j, size), sampleLayer(flooded, i, j, size), sampleLayer(sanded, i, j, size),
                           sampleLayer(south, i, j, size)};
            float w[kClasses];
            classify(h, std::hypot(dx, dz), L, w);
            float total = 0, rgb[3] = {0, 0, 0};
            int best = 0;
            for (int c = 0; c < kClasses; ++c) {
                total += w[c];
                for (int k = 0; k < 3; ++k) rgb[k] += w[c] * palette[c][k];
                if (w[c] > w[best]) best = c;
            }
            uint8_t* px = &out.colour[(size_t(j) * size + i) * 4];
            for (int k = 0; k < 3; ++k) px[k] = uint8_t(std::clamp(rgb[k] / std::max(total, 1e-4f), 0.0f, 1.0f) * 255.0f + 0.5f);
            px[3] = 255;
            out.surface[size_t(j) * size + i] = uint8_t(kClassSurface[best]);
        }
    }
    return out;
}

}  // namespace fuel
