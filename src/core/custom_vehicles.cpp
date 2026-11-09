// CUSTOM VEHICLES (port deviation): see custom_vehicles.hpp.
#include "custom_vehicles.hpp"

#include <mach-o/dyld.h>

#include <algorithm>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <mutex>
#include <sstream>

namespace fs = std::filesystem;

namespace fuel::custom {

namespace {

std::string upperOf(std::string s) {
    for (char& c : s) c = char(std::toupper(static_cast<unsigned char>(c)));
    return s;
}

// Splits a script line as FUEL's console does for these commands: blanks separate, "..." quotes, // starts a comment.
std::vector<std::string> words(const std::string& line) {
    std::vector<std::string> out;
    size_t i = 0;
    while (i < line.size()) {
        while (i < line.size() && std::isspace(static_cast<unsigned char>(line[i]))) ++i;
        if (i >= line.size()) break;
        if (line.compare(i, 2, "//") == 0) break;
        std::string w;
        if (line[i] == '"') {
            ++i;
            while (i < line.size() && line[i] != '"') w += line[i++];
            ++i;
        } else {
            while (i < line.size() && !std::isspace(static_cast<unsigned char>(line[i]))) w += line[i++];
        }
        out.push_back(w);
    }
    return out;
}

std::string gDataDir;

std::string exeDir() {
    char path[4096];
    uint32_t size = sizeof path;
    if (_NSGetExecutablePath(path, &size) != 0) return {};
    std::error_code ec;
    return fs::weakly_canonical(fs::path(path), ec).parent_path().string();
}

bool loadPack(const fs::path& dir, Vehicle& v) {
    std::ifstream f(dir / "vehicle.tsc", std::ios::binary);
    if (!f) return false;
    v.dir = dir.string();
    std::string dpcFile;
    bool inParams = false;
    for (std::string line; std::getline(f, line);) {
        if (!line.empty() && line.back() == '\r') line.pop_back();
        const std::vector<std::string> w = words(line);
        if (w.empty()) continue;
        const std::string cmd = upperOf(w[0]);
        if (cmd == "PORTVEHICLE" && w.size() > 2) {
            v.name = w[1], dpcFile = w[2];
        } else if (cmd == "PORTVEHICLESOUND" && w.size() > 2) {
            v.soundDonor = w[1], v.soundDpc = w[2];
        } else if (cmd == "PORTVEHICLESYNTH" && w.size() > 1) {
            v.synth = upperOf(w[1]);
        } else if (cmd == "PORTVEHICLEAI" && w.size() > 1) {
            v.ai = std::atoi(w[1].c_str()) != 0;
        } else if (cmd == "PORTTEXT" && w.size() > 2) {
            v.texts.emplace_back(unsigned(std::strtoul(w[1].c_str(), nullptr, 10)), w[2]);
        } else if (cmd == "PORTCREDIT") {  // the rest of the line as written (quotes kept: attribution texts quote titles)
            const size_t cmdAt = line.find_first_not_of(" \t");
            const size_t gap = line.find_first_of(" \t", cmdAt);
            const size_t at = gap == std::string::npos ? std::string::npos : line.find_first_not_of(" \t", gap);
            std::string text = at == std::string::npos ? std::string() : line.substr(at);
            while (!text.empty() && std::isspace(static_cast<unsigned char>(text.back()))) text.pop_back();
            v.credits.push_back(text);
        } else if (cmd == "VEHICLEPP" || cmd == "ENDOFVEHICLEPP") {
            v.params += line + "\n";
            inParams = cmd == "VEHICLEPP";
        } else if (cmd == "UNLOCKNEED") {
            v.unlocks += line + "\n";
        } else if (cmd.rfind("ADDVEHICLE", 0) == 0) {
            v.defs += line + "\n";
            if (cmd == "ADDVEHICLEINFO" && w.size() > 3) v.family = upperOf(w[2]), v.surface = upperOf(w[3]);
        }
    }
    if (inParams) v.params += "EndOfVehiclePP\n";
    std::error_code ec;
    if (v.name.empty() || dpcFile.empty() || !fs::exists(dir / dpcFile, ec)) {
        std::fprintf(stderr, "custom vehicles: %s incomplete (no PortVehicle line or no %s): skipped\n", dir.string().c_str(),
                     dpcFile.empty() ? "archive" : dpcFile.c_str());
        return false;
    }
    v.dpc = (dir / dpcFile).string();
    return true;
}

}  // namespace

std::string root() {
    if (const char* e = std::getenv("FUEL_CUSTOM_VEHICLES"); e && *e) return std::string(e) == "off" ? std::string() : std::string(e);
    const std::string exe = exeDir();
    if (exe.empty()) return {};
    std::error_code ec;
    for (const fs::path& p : {fs::path(exe).parent_path() / "Resources" / "vehicles", fs::path(exe) / "FUEL.app/Contents/Resources/vehicles"})
        if (fs::is_directory(p, ec)) return p.string();
    return {};
}

const std::vector<Vehicle>& vehicles() {
    static std::once_flag once;
    static std::vector<Vehicle> list;
    std::call_once(once, [] {
        const std::string r = root();
        std::error_code ec;
        if (r.empty() || !fs::is_directory(r, ec)) return;
        std::vector<fs::path> dirs;
        for (const auto& e : fs::directory_iterator(r, ec))
            if (e.is_directory()) dirs.push_back(e.path());
        std::sort(dirs.begin(), dirs.end());
        for (const fs::path& d : dirs) {
            Vehicle v;
            if (!loadPack(d, v)) continue;
            const bool dup = std::any_of(list.begin(), list.end(), [&](const Vehicle& o) { return upperOf(o.name) == upperOf(v.name); });
            if (!dup) list.push_back(std::move(v));
        }
        for (const Vehicle& v : list) std::fprintf(stderr, "custom vehicle: %s (%s)\n", v.name.c_str(), v.dpc.c_str());
    });
    return list;
}

const Vehicle* find(const std::string& nameOrMesh) {
    std::string n = upperOf(nameOrMesh);
    if (n.rfind("MESH_ID_", 0) == 0) n = n.substr(8);
    for (const Vehicle& v : vehicles())
        if (upperOf(v.name) == n) return &v;
    return nullptr;
}

std::string extraScript(const std::string& fileName) {
    const std::string f = upperOf(fs::path(fileName).filename().string());
    std::string out;
    for (const Vehicle& v : vehicles()) {
        if (f == "VEHICLESDEF.TSC") out += v.defs;
        else if (f == "VEHICLESUNLOCK.TSC") out += v.unlocks;
        else if (f == "VEHICLESPARAM.TSC") out += v.params;
    }
    return out;
}

std::string readScript(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    std::stringstream s;
    s << f.rdbuf();
    std::string text = s.str();
    const std::string extra = extraScript(path);
    if (!extra.empty()) {
        if (!text.empty() && text.back() != '\n') text += "\r\n";
        text += extra;
    }
    return text;
}

void setDataDir(const std::string& dir) { gDataDir = dir; }
const std::string& dataDir() { return gDataDir; }

std::string anchor(const std::string& dpcPath) {
    if (gDataDir.empty()) return dpcPath;
    for (const Vehicle& v : vehicles())
        if (v.dpc == dpcPath) return (fs::path(gDataDir) / "CommonComponent/DATAS" / fs::path(dpcPath).filename()).string();
    return dpcPath;
}

std::vector<std::string> creditLines() {
    std::vector<std::string> out;
    if (vehicles().empty()) return out;
    out.push_back("$");
    out.push_back("MACOS PORT");
    out.push_back("ADDITIONAL VEHICLES");
    for (const Vehicle& v : vehicles()) {
        out.push_back("$");
        for (const std::string& c : v.credits) out.push_back(c);
    }
    out.push_back("$");
    return out;
}

}  // namespace fuel::custom
