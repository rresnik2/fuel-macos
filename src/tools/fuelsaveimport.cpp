// PK25: fuelsaveimport --data DIR IN.sav OUT.sav [--force]
// Imports an original Windows FUEL career save (SaveGamePc_Z file, Documents\My Games\FUEL\<gamer tag>\<name>.sav) into
// the port's FUELPSAV career file. IN is only ever opened for reading; OUT must be given explicitly, must not be IN, and
// an existing OUT is kept unless --force. The career rules (scripts) come from the game data folder, so the imported
// state is applied exactly as a load from the port's own save (Career::deserialize: 0x567af0's section readers).
#include "core/career.hpp"
#include "core/save_import.hpp"

#include <cstdio>
#include <cstring>
#include <filesystem>
#include <string>

namespace fs = std::filesystem;

int main(int argc, char** argv) {
    std::string data, in, out;
    bool force = false;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        if (a == "--data" && i + 1 < argc) data = argv[++i];
        else if (a == "--force") force = true;
        else if (in.empty()) in = a;
        else if (out.empty()) out = a;
        else {
            std::fprintf(stderr, "fuelsaveimport: unexpected argument %s\n", a.c_str());
            return 2;
        }
    }
    if (data.empty() || in.empty() || out.empty()) {
        std::fprintf(stderr, "usage: fuelsaveimport --data GAMEDIR IN.sav OUT.sav [--force]\n");
        return 2;
    }
    std::error_code ec;
    if (fs::exists(out, ec) && fs::equivalent(in, out, ec)) {
        std::fprintf(stderr, "fuelsaveimport: OUT is IN; the input is never written\n");
        return 2;
    }
    if (fs::exists(out, ec) && !force) {
        std::fprintf(stderr, "fuelsaveimport: %s exists (use --force to replace it)\n", out.c_str());
        return 2;
    }
    std::vector<uint8_t> stream;
    std::string err;
    const int rc = fuel::pcsave::importFile(in, &stream, &err);
    if (rc != fuel::pcsave::Ok) {
        std::fprintf(stderr, "fuelsaveimport: %s: code 0x%x (%s)\n", in.c_str(), rc, err.c_str());
        return 1;
    }
    std::printf("fuelsaveimport: %s: body ok, %zu bytes of sections\n", in.c_str(), stream.size());
    try {
        fuel::Career c(fuel::loadCareerRules(data));
        if (!c.deserializeSections(stream, &err)) {
            std::fprintf(stderr, "fuelsaveimport: the sections do not parse: %s\n", err.c_str());
            return 1;
        }
        if (!err.empty()) std::printf("fuelsaveimport: note: %s\n", err.c_str());
        const int w = c.write(out);
        if (w != 0) {
            std::fprintf(stderr, "fuelsaveimport: cannot write %s (code 0x%x)\n", out.c_str(), w);
            return 1;
        }
        std::printf("fuelsaveimport: wrote %s: fuel %u, %d stars, %d challenges won, %zu achievements\n", out.c_str(), c.state().fuel,
                    c.stars(), c.challengesWon(), c.state().trophiesWon.size());
    } catch (const std::exception& e) {
        std::fprintf(stderr, "fuelsaveimport: %s\n", e.what());
        return 1;
    }
    return 0;
}
