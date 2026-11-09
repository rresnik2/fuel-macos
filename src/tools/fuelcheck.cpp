// fuelcheck: parses every Bitmap_Z, Mesh_Z and Material_Z in a set of DPC
// files and reports success counts, split by whether the file carries the
// version the game accepts.
//
//   fuelcheck <game data dir or .DPC files...>
//   fuelcheck --materials <...>   also surveys Material_Z / MaterialAnim_Z fields (p57)

#include "core/dpc.hpp"
#include "core/material_anim.hpp"
#include "core/resources.hpp"

#include <algorithm>
#include <cctype>
#include <chrono>
#include <cstdio>
#include <filesystem>
#include <map>
#include <memory>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace {

struct Count {
    unsigned ok = 0, total = 0;
};

bool isDpc(const fs::path& p) {
    std::string ext = p.extension().string();
    for (char& c : ext) c = char(std::toupper(static_cast<unsigned char>(c)));
    return ext == ".DPC";
}

}  // namespace

int main(int argc, char** argv) {
    if (argc < 2) {
        std::fprintf(stderr, "usage: fuelcheck <game data dir or .DPC files...>\n");
        return 2;
    }
    std::vector<fs::path> files;
    bool survey = false;
    for (int i = 1; i < argc; ++i) {
        if (std::string(argv[i]) == "--materials") {
            survey = true;
            continue;
        }
        const fs::path p = argv[i];
        if (fs::is_directory(p)) {
            for (const auto& e : fs::recursive_directory_iterator(p))
                if (e.is_regular_file() && isDpc(e.path())) files.push_back(e.path());
        } else {
            files.push_back(p);
        }
    }
    std::sort(files.begin(), files.end());

    const auto t0 = std::chrono::steady_clock::now();
    std::map<std::string, Count> counts;  // "<version tag> <class>"
    std::map<std::string, unsigned> formats, failures;
    unsigned filesOk = 0, filesBad = 0;
    uint64_t triangles = 0;

    for (const fs::path& path : files) {
        std::unique_ptr<fuel::DpcFile> dpc;
        try {
            dpc = std::make_unique<fuel::DpcFile>(path.string());
            ++filesOk;
        } catch (const std::exception& e) {
            ++filesBad;
            std::printf("SKIP %s\n", e.what());
            continue;
        }
        const std::string tag = dpc->isCurrentVersion() ? "current    " : "off-version";
        auto fail = [&](const char* cls, const std::string& why) { ++failures[tag + " " + cls + ": " + why.substr(0, 50)]; };

        for (uint32_t id : dpc->ids(fuel::cls::Bitmap)) {
            Count& c = counts[tag + " Bitmap_Z  "];
            ++c.total;
            try {
                const auto* e = dpc->find(id);
                const fuel::BitmapInfo bm = fuel::parseBitmapLink(dpc->link(*e));
                const auto size = dpc->bodySize(*e);
                const auto problems = fuel::checkBitmap(bm, size.value_or(0));
                if (!size || !problems.empty()) {
                    fail("Bitmap_Z", size ? problems.front() : "no body");
                    continue;
                }
                ++c.ok;
                ++formats[fuel::pixelFormatName(bm.format)];
            } catch (const std::exception& ex) {
                fail("Bitmap_Z", ex.what());
            }
        }
        for (uint32_t id : dpc->ids(fuel::cls::Mesh)) {
            Count& c = counts[tag + " Mesh_Z    "];
            ++c.total;
            try {
                const auto* e = dpc->find(id);
                const fuel::MeshLink link = fuel::parseMeshLink(dpc->link(*e));
                const auto body = dpc->body(*e);
                if (!body) {
                    fail("Mesh_Z", "no body");
                    continue;
                }
                const fuel::Mesh mesh = fuel::parseMeshBody(*body, link.flags);
                fuel::MeshStats st;
                const auto problems = fuel::checkMesh(mesh, &st);
                if (!problems.empty()) {
                    fail("Mesh_Z", problems.front());
                    continue;
                }
                ++c.ok;
                triangles += st.triangles;
            } catch (const std::exception& ex) {
                const std::string what = ex.what();
                fail("Mesh_Z", what.find("overrun") != std::string::npos || what.find("implausible") != std::string::npos
                                   ? "overrun/implausible count"
                                   : what);
            }
        }
        for (uint32_t id : dpc->ids(fuel::cls::Material)) {
            Count& c = counts[tag + " Material_Z"];
            ++c.total;
            try {
                const auto body = dpc->body(*dpc->find(id));
                if (!body) {
                    fail("Material_Z", "no body");
                    continue;
                }
                const fuel::Material m = fuel::parseMaterialBody(*body);
                ++c.ok;
                if (survey) {
                    const std::string f = path.filename().string();
                    ++counts["survey materials " + f].total;
                    if (!m.identityUv()) ++counts["survey materials non-identity uv " + f].total;
                    fuel::Material r = m;
                    fuel::updateUvMatrix(r);
                    float dmax = 0;
                    for (size_t k = 0; k < 6; ++k) dmax = std::max(dmax, std::abs(r.uvMatrix[k] - m.uvMatrix[k]));
                    if (dmax > 1e-4f) ++counts["survey materials uv matrix != UpdateUVMatrix(inputs)"].total;
                    for (uint32_t bit : {0x20u, 0x40u, 0x8000u, 0x10000u, 0x40000u, 0x80000u, 0x100000u, 0x200000u, 0x400000u, 0x1000u, 0x2000u})
                        if (m.flags & bit) {
                            char key[64];
                            std::snprintf(key, sizeof key, "survey flag %07x", bit);
                            ++counts[key].total;
                        }
                    char key[64];
                    std::snprintf(key, sizeof key, "survey code %02u", m.flags & 0x1f);
                    ++counts[key].total;
                    if (m.alpha < 1) ++counts["survey alpha < 1"].total;
                    if (m.fresnel[0] + m.fresnel[1] != 0) ++counts["survey fresnel != 0"].total;
                    if (m.textureMask & 0x100) ++counts["survey mask bit 8"].total;
                    if (((m.fresnel[0] + m.fresnel[1] != 0) || m.textures[8]) != bool(m.textureMask & 0x100))
                        ++counts["survey mask bit 8 != fallOff/slot 8"].total;
                    std::snprintf(key, sizeof key, "survey field28 %d field2c %d", m.field28, m.field2c);
                    ++counts[key].total;
                }
            } catch (const std::exception& ex) {
                fail("Material_Z", ex.what());
            }
        }
        if (survey)
            for (uint32_t id : dpc->ids(fuel::cls::MaterialAnimZ)) {
                const auto body = dpc->body(*dpc->find(id));
                if (!body) continue;
                const std::string f = path.filename().string();
                try {
                    const fuel::MaterialAnim a = fuel::parseMaterialAnimBody(*body);
                    ++counts["survey anim " + f].ok;
                    char key[96];
                    std::snprintf(key, sizeof key, "survey anim state %02x", a.state);
                    ++counts[key].total;
                    if (a.empty()) ++counts["survey anim static " + f].total;
                    if (!a.texture.empty()) ++counts["survey anim track flipbook " + f].total;
                    if (!a.uvOffset.keys.empty()) ++counts["survey anim track uvOffset " + f].total;
                    if (!a.uvScale.keys.empty()) ++counts["survey anim track uvScale " + f].total;
                    if (!a.uvAngle.keys.empty()) ++counts["survey anim track uvAngle " + f].total;
                    if (!a.diffuse.keys.empty()) ++counts["survey anim track diffuse " + f].total;
                    if (!a.emissive.keys.empty()) ++counts["survey anim track emissive " + f].total;
                    if (!a.alpha.keys.empty()) ++counts["survey anim track alpha " + f].total;
                    if (!a.specular.keys.empty()) ++counts["survey anim track specular " + f].total;
                    if (!a.params.keys.empty()) ++counts["survey anim track params " + f].total;
                    if (!a.field2c.empty()) ++counts["survey anim track field2c " + f].total;
                    if (!a.flags.empty()) ++counts["survey anim track flags " + f].total;
                    if (body->size() != 69 && a.empty()) ++counts["survey anim odd size"].total;
                    if (!a.empty())
                        std::printf("anim %08x %s material %08x state %02x period %.3f: tex %zu off %zu scale %zu angle %zu diff %zu emis %zu "
                                    "alpha %zu spec %zu params %zu f2c %zu flags %zu\n",
                                    id, f.c_str(), a.material, a.state, a.period, a.texture.size(), a.uvOffset.keys.size(),
                                    a.uvScale.keys.size(), a.uvAngle.keys.size(), a.diffuse.keys.size(), a.emissive.keys.size(),
                                    a.alpha.keys.size(), a.specular.keys.size(), a.params.keys.size(), a.field2c.size(), a.flags.size());
                } catch (const std::exception& ex) {
                    ++counts["survey anim FAIL " + f + " " + ex.what()].total;
                }
            }
    }

    const double secs = std::chrono::duration<double>(std::chrono::steady_clock::now() - t0).count();
    std::printf("files: %u parsed, %u failed (%.1f s)\n", filesOk, filesBad, secs);
    for (const auto& [k, c] : counts) std::printf("  %s %6u / %-6u OK\n", k.c_str(), c.ok, c.total);
    std::printf("triangles in good meshes: %llu\n", static_cast<unsigned long long>(triangles));
    std::printf("bitmap formats:");
    for (const auto& [k, n] : formats) std::printf(" %s %u", k.c_str(), n);
    std::printf("\n");
    if (!failures.empty()) {
        std::printf("failures by reason:\n");
        for (const auto& [k, n] : failures) std::printf("  %6u  %s\n", n, k.c_str());
    }
    unsigned currentBad = 0;
    for (const auto& [k, c] : counts)
        if (k.rfind("current", 0) == 0) currentBad += c.total - c.ok;
    return currentBad == 0 && filesBad == 0 ? 0 : 1;
}
