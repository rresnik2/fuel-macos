// fuelscene: assembles a vehicle variant from a vehicle DPC and prints its
// placed parts.
//
//   fuelscene [--data DIR] DPC                      list the variants
//   fuelscene [--data DIR] DPC VARIANT [--lod N] [--json]
//   fuelscene [--data DIR] DPC --all [--lod N]      one summary line per variant
//
// VARIANT is a variant name (QUAD_ROAD_2), a GameObj_Z ID in hex or the start
// of a group name. --json prints the part list in the layout of the
// reverse-engineering tool scene_dump.py (same keys, order and number
// formatting), so the two can be compared directly.

#include "core/assembly.hpp"
#include "core/dpc.hpp"
#include "core/resources.hpp"

#include <charconv>
#include <cmath>
#include <cstdio>
#include <cstdlib>
#include <filesystem>
#include <string>
#include <vector>

namespace fs = std::filesystem;

namespace {

[[noreturn]] void die(const std::string& msg) {
    std::fprintf(stderr, "fuelscene: %s\n", msg.c_str());
    std::exit(1);
}

std::string hex8(uint32_t v) {
    char b[12];
    std::snprintf(b, sizeof b, "%08x", v);
    return b;
}

std::string defaultDataDir() {
    if (const char* env = std::getenv("FUEL_DATA"); env && *env) return env;
    const char* home = std::getenv("HOME");
    return std::string(home ? home : "") + "/Library/Application Support/FUEL/game";
}

const char* className(uint32_t c) {
    if (c == fuel::cls::Mesh) return "Mesh_Z";
    if (c == fuel::cls::Lod) return "Lod_Z";
    if (c == fuel::cls::Omni) return "Omni_Z";
    return nullptr;
}

// ---- JSON in Python's json.dump(indent=1) layout ------------------------------

// repr() of a Python float: the shortest digits that read back to the same
// double, in fixed notation unless the exponent is below -4 or above 15.
std::string pyFloat(double v) {
    if (std::isnan(v)) return "NaN";
    if (std::isinf(v)) return v > 0 ? "Infinity" : "-Infinity";
    char buf[64];
    const auto res = std::to_chars(buf, buf + sizeof buf, v, std::chars_format::scientific);
    std::string s(buf, res.ptr);
    std::string sign;
    if (s[0] == '-') {
        sign = "-";
        s.erase(0, 1);
    }
    const size_t e = s.find('e');
    const int exp = std::stoi(s.substr(e + 1));
    std::string digits;
    for (char c : s.substr(0, e))
        if (c != '.') digits += c;
    const int point = exp + 1;  // digits before the decimal point
    std::string out;
    if (point <= -4 || point > 16) {
        out = digits.substr(0, 1) + (digits.size() > 1 ? "." + digits.substr(1) : "");
        char eb[16];
        std::snprintf(eb, sizeof eb, "e%c%02d", exp < 0 ? '-' : '+', std::abs(exp));
        out += eb;
    } else if (point <= 0) {
        out = "0." + std::string(size_t(-point), '0') + digits;
    } else if (size_t(point) < digits.size()) {
        out = digits.substr(0, size_t(point)) + "." + digits.substr(size_t(point));
    } else {
        out = digits + std::string(size_t(point) - digits.size(), '0') + ".0";
    }
    return sign + out;
}

// JSON string with every byte outside printable ASCII escaped (Latin-1 text).
std::string jsonString(const std::string& s) {
    std::string out = "\"";
    for (unsigned char c : s) {
        switch (c) {
            case '"': out += "\\\""; break;
            case '\\': out += "\\\\"; break;
            case '\n': out += "\\n"; break;
            case '\r': out += "\\r"; break;
            case '\t': out += "\\t"; break;
            case '\b': out += "\\b"; break;
            case '\f': out += "\\f"; break;
            default:
                if (c < 0x20 || c > 0x7E) {
                    char b[8];
                    std::snprintf(b, sizeof b, "\\u%04x", c);
                    out += b;
                } else {
                    out += char(c);
                }
        }
    }
    return out + "\"";
}

// Writes {"key": value, ...} with one-space indentation per level.
class JsonObject {
public:
    JsonObject(std::string& out, int level) : out_(out), level_(level) { out_ += "{"; }
    void raw(const std::string& key, const std::string& value) {
        out_ += first_ ? "\n" : ",\n";
        first_ = false;
        out_ += std::string(size_t(level_ + 1), ' ') + jsonString(key) + ": " + value;
    }
    void str(const std::string& key, const std::string& value) { raw(key, jsonString(value)); }
    void close() { out_ += first_ ? "}" : "\n" + std::string(size_t(level_), ' ') + "}"; }

private:
    std::string& out_;
    int level_;
    bool first_ = true;
};

std::string jsonList(const std::vector<std::string>& items, int level) {
    if (items.empty()) return "[]";
    std::string out = "[";
    for (size_t i = 0; i < items.size(); ++i) out += (i ? ",\n" : "\n") + std::string(size_t(level + 1), ' ') + items[i];
    return out + "\n" + std::string(size_t(level), ' ') + "]";
}

std::string sceneJson(const fuel::VehicleAssembler& va, const fuel::AssembledVehicle& v, const std::string& source) {
    const fuel::NameTable& names = va.names();
    std::string out;
    JsonObject doc(out, 0);
    doc.str("source", source);
    doc.str("variant", names(v.name));
    doc.str("gameobj_id", hex8(v.gameObjId));
    doc.raw("lod_level", std::to_string(v.lod));
    doc.raw("skeleton_id", v.skeleton ? jsonString(hex8(*v.skeleton)) : "null");
    doc.str("matrix_convention",
            "16 floats row-major, Direct3D row-vector: p_world = [x y z 1] * M; translation in elements 12..14; metres, right-handed, y up");
    doc.str("notes",
            "draw each mesh with its world_matrix (same mesh may appear on several bones = instances); draw all batches with "
            "their own VB; do not apply the stride-12 morph stream (weight 0 = undamaged); kind *_unplaced = group node no V: "
            "command places (not drawn by the vehicle code; node matrix); light_* = Omni_Z position only");
    std::vector<std::string> parts;
    for (const fuel::PlacedPart& p : v.parts) {
        std::string s;
        JsonObject o(s, 2);
        o.str("node_id", hex8(p.nodeId));
        o.str("group", p.group);
        o.str("object_id", hex8(p.objectId));
        const char* cn = className(p.objectClass);
        o.str("object_class", cn ? cn : "?" + hex8(p.objectClass));
        o.str("object_name", names(p.objectName));
        o.raw("lod_level", std::to_string(v.lod));
        o.raw("bone", p.bone >= 0 ? jsonString(names(p.boneName)) : "null");
        o.raw("command", p.bone >= 0 ? jsonString(p.command) : "null");
        o.raw("mesh_id", p.meshId != 0 ? jsonString(hex8(p.meshId)) : "null");
        std::vector<std::string> m;
        for (double x : p.world) m.push_back(pyFloat(x));
        o.raw("world_matrix", jsonList(m, 3));
        o.str("kind", fuel::partKind(p));
        if (p.shape == fuel::PartShape::SkinBindPose) {
            o.str("skin_id", hex8(p.skinId));
            o.str("skel_id", hex8(p.skelId));
        }
        if (p.placement == fuel::Placement::Wheel || p.placement == fuel::Placement::WheelFlipped) o.str("hub", p.hub);
        o.close();
        parts.push_back(std::move(s));
    }
    doc.raw("parts", jsonList(parts, 1));
    doc.close();
    return out;
}

// ---- text output -------------------------------------------------------------

void printVariants(const fuel::VehicleAssembler& va) {
    for (const fuel::VehicleVariant& v : va.variants()) {
        std::printf("%08x %-18s", v.gameObjId, v.label.c_str());
        for (const std::string& g : v.groups) std::printf(" %s", g.c_str());
        std::printf("\n");
    }
}

void printAssembly(const fuel::VehicleAssembler& va, const fuel::AssembledVehicle& v) {
    const fuel::NameTable& names = va.names();
    std::printf("# variant %s GameObj_Z %08x LOD%d skeleton %s groups", names(v.name).c_str(), v.gameObjId, v.lod,
                v.skeleton ? hex8(*v.skeleton).c_str() : "-");
    for (const std::string& g : v.groups) std::printf(" %s", g.c_str());
    std::printf("\n# matrices: row-major, Direct3D row vectors (p_world = [x y z 1] * M), translation in [12..14]\n");
    for (const fuel::PlacedPart& p : v.parts) {
        const char* cn = className(p.objectClass);
        std::printf("%-22s node %08x %-7s %08x mesh %-8s %-34s T(%.3f %.3f %.3f) bone %-10s %s\n", fuel::partKind(p).c_str(), p.nodeId,
                    cn ? cn : "?", p.objectId, p.meshId ? hex8(p.meshId).c_str() : "-", names(p.objectName).c_str(), p.world[12],
                    p.world[13], p.world[14], p.bone >= 0 ? names(p.boneName).c_str() : "-", p.command.c_str());
    }
    for (const std::string& s : v.problems) std::printf("# problem: %s\n", s.c_str());
    const auto& bones = v.otherCommands;
    for (const auto& [bone, text] : bones) std::printf("# other command on bone %d: %s\n", bone, text.c_str());
    // P74: the CAR_OCCLUSION omni box (0x5cc410)
    std::printf("# car occlusion: Lod_Z %s box %s half extents %.3f %.3f %.3f\n", v.occlusionLod ? hex8(v.occlusionLod).c_str() : "-",
                v.occlusionBoxName == fuel::asoboCrc("CAR_OCCLUSION") ? "CAR_OCCLUSION"
                : v.occlusionBoxName == fuel::asoboCrc("DURL")        ? "DURL+0.75"
                                                                       : "default",
                v.occlusionExtents[0], v.occlusionExtents[1], v.occlusionExtents[2]);
    // P77: the merged Lod_Z boxes (0x5ca7a0: the body blob's size without wheels)
    for (const fuel::VehicleLight& l : v.lights)  // P77
        std::printf("# light %s: Omni_Z %08x on bone %d at (%.3f %.3f %.3f) z (%.2f %.2f %.2f) colour %.2f %.2f %.2f r %.1f R %.1f cut %.4f %.4f\n",
                    l.key.c_str(), l.omniId, l.bone, l.world[12], l.world[13], l.world[14], l.world[8], l.world[9], l.world[10], l.params.colour[0],
                    l.params.colour[1], l.params.colour[2], l.params.inner, l.params.outer, l.params.cutZ, l.params.cutW);
    std::printf("# merged boxes: half %.3f %.3f %.3f centre %.3f %.3f %.3f\n", v.boxHalf[0], v.boxHalf[1], v.boxHalf[2], v.boxCentre[0],
                v.boxCentre[1], v.boxCentre[2]);
    // ---- PK22: the V:DYN records (0x69e020) and the parts that follow them
    for (size_t d = 0; d < v.dyn.size(); ++d) {
        const fuel::vlinks::DynRecord& r = v.dyn[d];
        std::printf("# dyn %zu: key %08x autostart %d adopt %d limb %d rpm %.5f axis %.2f %.2f %.2f tpm %.2f axe %.0f%.0f%.0f points %zu", d, r.key,
                    r.autostart, r.adopt, r.limb, r.rpm, r.rpmAxis[0], r.rpmAxis[1], r.rpmAxis[2], r.tpm, r.axe[0], r.axe[1], r.axe[2], r.points.size());
        for (const auto& p : r.points)
            std::printf(" [rest %.3f %.3f %.3f g %.2f k %.1f d %.1f box half %.3f %.3f %.3f]", p.rest[0], p.rest[1], p.rest[2], p.gravity, p.spring,
                        p.damping, p.box[12], p.box[13], p.box[14]);
        std::printf("\n");
        for (const fuel::PlacedPart& p : v.parts)
            if (p.dyn == int(d))
                std::printf("#   part mesh %08x pivot %.3f %.3f %.3f boneQ %.3f %.3f %.3f %.3f\n", p.meshId, p.dynPivot[0], p.dynPivot[1], p.dynPivot[2],
                            p.dynBoneQ[0], p.dynBoneQ[1], p.dynBoneQ[2], p.dynBoneQ[3]);
    }
    std::printf("# headlight records %d\n", v.headlightRecords ? 1 : 0);
    // ---- PK22 end
}

void printSummary(const fuel::VehicleAssembler& va, const fuel::VehicleVariant& variant, int lod) {
    try {
        const fuel::AssembledVehicle v = va.assemble(variant, lod);
        unsigned drawn = 0, wheels = 0, unplaced = 0, lights = 0;
        for (const fuel::PlacedPart& p : v.parts) {
            drawn += p.drawn();
            wheels += p.drawn() && (p.placement == fuel::Placement::Wheel || p.placement == fuel::Placement::WheelFlipped);
            unplaced += p.placement == fuel::Placement::Unplaced && p.meshId != 0;
            lights += p.shape == fuel::PartShape::Light;
        }
        std::printf("%08x %-18s LOD%d skeleton %s: %u parts drawn (%u wheels), %u unplaced meshes, %u lights, %zu problems\n",
                    variant.gameObjId, variant.label.c_str(), lod, v.skeleton ? hex8(*v.skeleton).c_str() : "-", drawn, wheels,
                    unplaced, lights, v.problems.size());
    } catch (const std::exception& e) {
        std::printf("%08x %-18s LOD%d FAILED: %s\n", variant.gameObjId, variant.label.c_str(), lod, e.what());
    }
}

}  // namespace

int main(int argc, char** argv) {
    std::string dataDir, dpcArg, variantArg;
    int lod = 1;
    bool json = false, all = false;
    for (int i = 1; i < argc; ++i) {
        const std::string a = argv[i];
        auto next = [&]() -> std::string {
            if (i + 1 >= argc) die("missing value after " + a);
            return argv[++i];
        };
        if (a == "--data") dataDir = next();
        else if (a == "--lod") lod = std::atoi(next().c_str());
        else if (a == "--json") json = true;
        else if (a == "--all") all = true;
        else if (a == "-h" || a == "--help") {
            std::puts("usage: fuelscene [--data DIR] DPC [VARIANT [--lod N] [--json] | --all [--lod N]]");
            return 0;
        } else if (!a.empty() && a[0] == '-') die("unknown option " + a);
        else if (dpcArg.empty()) dpcArg = a;
        else if (variantArg.empty()) variantArg = a;
        else die("too many arguments");
    }
    if (dpcArg.empty()) die("usage: fuelscene [--data DIR] DPC [VARIANT [--lod N] [--json] | --all [--lod N]]");
    if (lod < 1) die("--lod starts at 1");
    if (dataDir.empty()) dataDir = defaultDataDir();
    fs::path path = dpcArg;
    if (!fs::exists(path) && path.is_relative()) path = fs::path(dataDir) / dpcArg;

    try {
        const fuel::DpcFile dpc(path.string());
        const fuel::VehicleAssembler va(dpc);
        if (all) {
            for (const fuel::VehicleVariant& v : va.variants()) printSummary(va, v, lod);
            return 0;
        }
        if (variantArg.empty()) {
            printVariants(va);
            return 0;
        }
        const fuel::VehicleVariant* variant = va.find(variantArg);
        if (variant == nullptr) {
            std::fprintf(stderr, "fuelscene: no variant matches '%s'; the file has:\n", variantArg.c_str());
            for (const fuel::VehicleVariant& v : va.variants()) std::fprintf(stderr, "  %08x %s\n", v.gameObjId, v.label.c_str());
            return 1;
        }
        const fuel::AssembledVehicle v = va.assemble(*variant, lod);
        if (json)
            std::fputs(sceneJson(va, v, path.filename().string()).c_str(), stdout);
        else
            printAssembly(va, v);
    } catch (const std::exception& e) {
        die(e.what());
    }
    return 0;
}
