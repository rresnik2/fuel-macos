#include "assembly.hpp"

#include "reader.hpp"
#include "resources.hpp"

#include <algorithm>
#include <cstdio>
#include <cstring>
#include <stdexcept>
#include <unordered_set>

// Placement maths runs in double, in the same order of operations as the
// reverse-engineering tool (tools/scene_dump.py), so both give identical
// matrices. Fused multiply-adds would change the last bits.
#pragma clang fp contract(off)

namespace fuel {

namespace {

constexpr uint32_t kListLimit = 1'000'000;

std::string hex8(uint32_t v) {
    char b[12];
    std::snprintf(b, sizeof b, "%08x", v);
    return b;
}

// Whitespace as the original tools see it in Latin-1 text.
bool isSpace(unsigned char c) { return (c >= 0x09 && c <= 0x0D) || (c >= 0x1C && c <= 0x20) || c == 0x85 || c == 0xA0; }

std::string trim(const std::string& s) {
    size_t b = 0, e = s.size();
    while (b < e && isSpace(static_cast<unsigned char>(s[b]))) ++b;
    while (e > b && isSpace(static_cast<unsigned char>(s[e - 1]))) --e;
    return s.substr(b, e - b);
}

std::string upper(std::string s) {
    for (char& c : s)
        if (c >= 'a' && c <= 'z') c = char(c - 32);
    return s;
}

bool startsWith(const std::string& s, const char* prefix) { return s.rfind(prefix, 0) == 0; }

// Trailing "LOD<digits>" of a group name: the digits' value and where "LOD"
// starts; nullopt without one. Values past 10^9 saturate.
struct LodSuffix {
    long long value;
    size_t lodPos;
};
std::optional<LodSuffix> lodSuffix(std::string s) {
    if (!s.empty() && s.back() == '\n') s.pop_back();
    size_t e = s.size();
    while (e > 0 && s[e - 1] >= '0' && s[e - 1] <= '9') --e;
    if (e == s.size() || e < 3 || upper(s.substr(e - 3, 3)) != "LOD") return std::nullopt;
    long long v = 0;
    for (size_t i = e; i < s.size(); ++i) v = std::min(1'000'000'000LL, v * 10 + (s[i] - '0'));
    return LodSuffix{v, e - 3};
}

// "COMMON" has no LOD; "QUAD_ROAD_2_LOD3" is LOD 3.
std::optional<long long> groupLod(const std::string& name) {
    const auto s = lodSuffix(name);
    return s ? std::optional<long long>(s->value) : std::nullopt;
}

// "QUAD_ROAD_2_LOD3" -> "QUAD_ROAD_2".
std::optional<std::string> withoutLodSuffix(const std::string& name) {
    const auto s = lodSuffix(name);
    if (!s || s->lodPos < 1 || name[s->lodPos - 1] != '_') return std::nullopt;
    std::string prefix = name.substr(0, s->lodPos - 1);
    if (prefix.find('\n') != std::string::npos) return std::nullopt;
    return prefix;
}

// Named points: m spheres {centre, radius, type, name}, then k boxes {3x4
// frame, half extents + radius, type, name}. "+POINT" references only use
// the spheres (spec S4, S18).
std::vector<NamedSphere> readNamedPoints(ByteReader& r) {
    std::vector<NamedSphere> spheres(r.count(100000));
    for (NamedSphere& s : spheres) {
        for (float& c : s.centre) c = r.f32();
        s.radius = r.f32();
        s.type = r.u32();
        s.name = r.u32();
    }
    r.skipArray(72, 100000);
    return spheres;
}

// ---- matrices (row-vector convention) ---------------------------------------

using Vec3 = std::array<double, 3>;

Mat4 toMat4(const std::array<float, 16>& m) {
    Mat4 out;
    for (size_t i = 0; i < 16; ++i) out[i] = m[i];
    return out;
}

Mat4 translation(const Vec3& t) { return {1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0, t[0], t[1], t[2], 1}; }

Vec3 position(const std::array<float, 16>& m) { return {m[12], m[13], m[14]}; }

Vec3 transformPoint(const Mat4& m, const std::array<float, 3>& p) {
    const double x = p[0], y = p[1], z = p[2];
    return {x * m[0] + y * m[4] + z * m[8] + m[12], x * m[1] + y * m[5] + z * m[9] + m[13],
            x * m[2] + y * m[6] + z * m[10] + m[14]};
}

// D3DXMatrixRotationQuaternion, q = (x, y, z, w).
Mat4 rotation(const std::array<float, 4>& q) {
    const double x = q[0], y = q[1], z = q[2], w = q[3];
    return {1 - 2 * (y * y + z * z), 2 * (x * y + z * w),     2 * (x * z - y * w),     0,
            2 * (x * y - z * w),     1 - 2 * (x * x + z * z), 2 * (y * z + x * w),     0,
            2 * (x * z + y * w),     2 * (y * z - x * w),     1 - 2 * (x * x + y * y), 0,
            0,                       0,                       0,                       1};
}

// a * b: apply a first, then b.
Mat4 multiply(const Mat4& a, const Mat4& b) {
    Mat4 out;
    for (int i = 0; i < 4; ++i)
        for (int j = 0; j < 4; ++j) {
            double s = 0.0;
            for (int k = 0; k < 4; ++k) s += a[size_t(i * 4 + k)] * b[size_t(k * 4 + j)];
            out[size_t(i * 4 + j)] = s;
        }
    return out;
}

const Mat4 kRotY180 = {-1, 0, 0, 0, 0, 1, 0, 0, 0, 0, -1, 0, 0, 0, 0, 1};

// The biped bone names of the vehicle rigs, and other names the data uses
// without spelling them out (spec S9).
const char* const kKnownNames[] = {
    "Hips", "Spine", "Spine1", "Spine2", "Neck", "Head", "Root", "LeftShoulder", "LeftArm",
    "LeftForeArm", "LeftHand", "RightShoulder", "RightArm", "RightForeArm", "RightHand",
    "LeftUpLeg", "LeftLeg", "LeftFoot", "LeftToeBase", "RightUpLeg", "RightLeg", "RightFoot",
    "RightToeBase", "LIGHT", "LIGHTF", "LightF1", "COMMON", "GAMEOBJECTS",
};

// Quoted strings of a command line, in order.
std::vector<std::string> quotedStrings(const std::string& line) {
    std::vector<std::string> out;
    for (size_t open = line.find('"'); open != std::string::npos;) {
        const size_t close = line.find('"', open + 1);
        if (close == std::string::npos) break;
        out.push_back(line.substr(open + 1, close - open - 1));
        open = line.find('"', close + 1);
    }
    return out;
}

// "PART+POINT": the part, and the point (LOCAL_LINK when omitted).
struct PointRef {
    uint32_t part = 0, point = 0;
    std::string partText, pointText;
};
PointRef splitPoint(const std::string& arg) {
    PointRef p;
    const size_t plus = arg.find('+');
    p.partText = arg.substr(0, plus);
    p.pointText = plus == std::string::npos ? std::string() : arg.substr(plus + 1);
    if (p.pointText.empty()) p.pointText = "LOCAL_LINK";
    p.part = asoboCrc(p.partText);
    p.point = asoboCrc(p.pointText);
    return p;
}

}  // namespace

// ---- parsers ------------------------------------------------------------------

Node parseNodeBody(const std::vector<uint8_t>& body) {
    // Ten links, a 0x82-byte transform block, the 0x40 world matrix (spec S1, S12).
    if (body.size() != 234) throw std::runtime_error("Node_Z body " + std::to_string(body.size()) + " bytes, expected 234");
    ByteReader r(body);
    Node n;
    n.parent = r.u32();
    n.child = r.u32();
    n.next = r.u32();
    n.prev = r.u32();
    n.object = r.u32();
    n.objectData = r.u32();
    n.userDefine = r.u32();
    n.light = r.u32();
    r.raw(8 + 0x82);  // two unused links, the transform block
    for (float& f : n.world) f = r.f32();
    return n;
}

ObjectLink parseObjectLink(const std::vector<uint8_t>& link) {
    if (link.size() < 98) throw std::runtime_error("short object link (" + std::to_string(link.size()) + " bytes)");
    ByteReader r(link);
    ObjectLink l;
    l.name = r.u32();
    l.dataId = r.u32();
    return l;
}

std::vector<NamedSphere> parseMeshLinkSpheres(const std::vector<uint8_t>& link) {
    // After the 98-byte object part: dependency IDs, 12 bytes, then the named points.
    ByteReader r(link, std::min<size_t>(98, link.size()));
    r.skipArray(4, kListLimit);
    r.raw(12);
    return readNamedPoints(r);
}

std::vector<NodeGroup> parseGameObjBody(const std::vector<uint8_t>& body) {
    // Groups: u32 name length; name; u32 value; u32 n; Node_Z IDs (spec S3).
    ByteReader r(body);
    std::vector<NodeGroup> groups(r.count(10000));
    for (NodeGroup& g : groups) {
        const uint32_t len = r.count(0x100);
        const char* s = reinterpret_cast<const char*>(r.raw(len));
        g.name.assign(s, strnlen(s, len));
        g.value = r.u32();
        g.nodes = r.u32Array(kListLimit);
    }
    r.expectEnd();
    return groups;
}

Lod parseLodBody(const std::vector<uint8_t>& body) {
    // Named points, three u32, objects per LOD, a handle, two optional
    // registry pair lists, a pointer (spec S4, S11).
    ByteReader r(body);
    Lod l;
    l.spheres = readNamedPoints(r);
    r.raw(12);
    l.objects = r.u32Array(kListLimit);
    r.u32();
    for (int i = 0; i < 2; ++i)
        if (r.u32() != 0) {
            const uint32_t n = r.count(kListLimit);
            for (uint32_t k = 0; k < n; ++k) {
                const uint32_t slot = r.u32(), anim = r.u32();
                if (i == 0) l.animations.push_back({slot, anim});
            }
        }
    r.u32();
    r.expectEnd();
    return l;
}

Skin parseSkinBody(const std::vector<uint8_t>& body) {
    // Meshes, then per-bone tables and animation sets that placement does not need (spec S5).
    ByteReader r(body);
    Skin s;
    s.meshes = r.u32Array(kListLimit);
    r.skipArray(8, kListLimit);
    for (uint32_t i = 0, n = r.count(kListLimit); i < n; ++i) {
        r.u32();
        for (uint32_t j = 0, m = r.count(kListLimit); j < m; ++j) {
            r.u16();
            r.skipArray(8, kListLimit);
            r.skipArray(8, kListLimit);
        }
    }
    for (int i = 0; i < 2; ++i)
        if (r.u32() != 0) r.skipArray(8, kListLimit);
    r.u32();
    r.u32();
    s.sets.resize(r.count(kListLimit));
    for (auto& set : s.sets) {
        set.resize(r.count(kListLimit));
        for (SkinRecord& rec : set) {
            std::memcpy(rec.data(), r.raw(0x60), 0x60);
            for (uint32_t k = 0, subs = r.count(kListLimit); k < subs; ++k) {
                r.u32();
                r.raw(r.count(4096));
            }
        }
    }
    r.expectEnd();
    return s;
}

Skel parseSkelBody(const std::vector<uint8_t>& body) {
    // Header, 248-byte bones, then material / mesh-data / point tables (spec S7).
    ByteReader r(body);
    r.raw(20);
    Skel s;
    s.bones.resize(r.count(4096));
    for (Bone& b : s.bones) {
        b.userDefine = r.u32();
        const uint8_t* block = r.raw(0xA0);  // local TRS; +0x60 holds a second copy of the rotation
        std::memcpy(b.restQuat.data(), block + 0x60, 16);
        std::memcpy(b.localQuat.data(), block, 16);
        std::memcpy(b.localScale.data(), block + 0x10, 12);
        std::memcpy(b.localT.data(), block + 0x20, 12);
        for (float& f : b.world) f = r.f32();
        b.firstChild = r.i32();
        b.parent = r.i32();
        b.nextSibling = r.i32();
        b.prevSibling = r.i32();
        b.name = r.u32();
    }
    r.skipArray(4, kListLimit);  // Material_Z
    r.skipArray(4, kListLimit);  // MeshData_Z
    for (uint32_t i = 0, n = r.count(kListLimit); i < n; ++i) r.skipArray(4, kListLimit);
    r.skipArray(4, kListLimit);
    r.skipArray(28, kListLimit);  // spheres + u32
    r.skipArray(28, kListLimit);
    r.skipArray(76, kListLimit);  // boxes + u32
    r.expectEnd();
    return s;
}

std::vector<std::string> parseUserDefineBody(const std::vector<uint8_t>& body) {
    // u32 length, text. The engine drops '[' and ']' and turns ';' into line breaks (spec S6).
    ByteReader r(body);
    const uint32_t len = r.u32();
    if (len > r.remaining()) throw std::runtime_error("UserDefine_Z text runs past the body");
    const char* p = reinterpret_cast<const char*>(r.raw(len));
    const std::string text(p, strnlen(p, len));
    std::vector<std::string> lines;
    std::string cur;
    auto flush = [&] {
        std::string t = trim(cur);
        if (!t.empty()) lines.push_back(std::move(t));
        cur.clear();
    };
    for (char c : text) {
        if (c == '[' || c == ']') continue;
        if (c == ';' || c == '\n' || c == '\r')
            flush();
        else
            cur += c;
    }
    flush();
    return lines;
}

bool BoneCommand::appliesToLod(int lod) const {
    const std::string digit = std::to_string(lod);
    bool tagged = false;
    for (const std::string& t : tags) {
        if (!startsWith(upper(t), "LOD")) continue;
        tagged = true;
        if (t.find(digit) != std::string::npos) return true;
    }
    return !tagged && lod == 1;
}

std::optional<BoneCommand> parseBoneCommand(const std::string& line) {
    // Tokens: "quoted" (may be empty, may hold spaces) or bare words. A quote
    // without a partner is part of a bare word.
    std::vector<std::pair<std::string, bool>> tokens;  // text, quoted
    const size_t n = line.size();
    for (size_t i = 0; i < n;) {
        if (line[i] == '"') {
            const size_t close = line.find('"', i + 1);
            if (close != std::string::npos) {
                tokens.emplace_back(line.substr(i + 1, close - i - 1), true);
                i = close + 1;
                continue;
            }
        }
        if (isSpace(static_cast<unsigned char>(line[i]))) {
            ++i;
            continue;
        }
        size_t j = i;
        while (j < n && !isSpace(static_cast<unsigned char>(line[j]))) ++j;
        tokens.emplace_back(line.substr(i, j - i), false);
        i = j;
    }
    if (tokens.empty()) return std::nullopt;
    BoneCommand c;
    c.keyword = upper(tokens[0].first);
    for (size_t i = 1; i < tokens.size(); ++i) (tokens[i].second ? c.args : c.tags).push_back(tokens[i].first);
    c.text = line;
    return c;
}

// ---- names ---------------------------------------------------------------------

NameTable::NameTable(const DpcFile& dpc) {
    for (const char* n : kKnownNames) names_[asoboCrc(n)] = n;
    auto add = [&](const std::string& s) {
        if (!s.empty()) names_.emplace(asoboCrc(s), s);
    };
    auto body = [&](uint32_t id) {
        auto b = dpc.body(*dpc.find(id));
        if (!b) throw std::runtime_error("no body");
        return std::move(*b);
    };
    // Part, point and bone names spelled out in the bone commands.
    for (uint32_t id : dpc.ids(cls::UserDefine)) {
        std::vector<std::string> lines;
        try {
            lines = parseUserDefineBody(body(id));
        } catch (const std::exception&) {
            continue;
        }
        for (const std::string& line : lines)
            for (const std::string& q : quotedStrings(line)) {
                add(q);
                for (size_t b = 0;;) {
                    const size_t e = q.find('+', b);
                    add(q.substr(b, e == std::string::npos ? std::string::npos : e - b));
                    if (e == std::string::npos) break;
                    b = e + 1;
                }
            }
    }
    // Group names, and the variant names they start with.
    for (uint32_t id : dpc.ids(cls::GameObj)) {
        std::vector<NodeGroup> groups;
        try {
            groups = parseGameObjBody(body(id));
        } catch (const std::exception&) {
            continue;
        }
        for (const NodeGroup& g : groups) {
            names_.emplace(asoboCrc(g.name), g.name);
            if (auto prefix = withoutLodSuffix(g.name)) names_.emplace(asoboCrc(*prefix), *prefix);
        }
    }
}

const std::string* NameTable::find(uint32_t crc) const {
    auto it = names_.find(crc);
    return it == names_.end() ? nullptr : &it->second;
}

std::string NameTable::operator()(uint32_t crc) const {
    const std::string* s = find(crc);
    return s ? *s : "?" + hex8(crc);
}

// ---- assembly -------------------------------------------------------------------

std::string partKind(const PlacedPart& p) {
    static const char* const kHow[] = {"link", "linkb", "linkr", "linkm", "linkmm", "wheel", "wheel_flipped", "unplaced"};
    const char* shape = p.shape == PartShape::Mesh ? "mesh_" : p.shape == PartShape::SkinBindPose ? "skin_bindpose_" : "light_";
    return std::string(shape) + kHow[int(p.placement)] + (p.keptNodeMatrix ? "_nolookat" : "");
}

// Parsed resources of one DPC, cached by ID.
struct VehicleAssembler::Resources {
    const DpcFile& dpc;
    std::unordered_map<uint32_t, Node> nodes;
    std::unordered_map<uint32_t, std::vector<NodeGroup>> gameObjs;
    std::unordered_map<uint32_t, Lod> lods;
    std::unordered_map<uint32_t, Skin> skins;
    std::unordered_map<uint32_t, Skel> skels;
    std::unordered_map<uint32_t, std::vector<std::string>> userDefines;
    std::unordered_map<uint32_t, ObjectLink> objectLinks;
    std::unordered_map<uint32_t, std::vector<NamedSphere>> meshSpheres;

    explicit Resources(const DpcFile& f) : dpc(f) {}

    // A resource with a block copy (and of class `classCrc`, when given).
    bool has(uint32_t id, uint32_t classCrc = 0) const {
        const DpcFile::Entry* e = dpc.find(id);
        return e != nullptr && e->header != nullptr && (classCrc == 0 || e->classCrc == classCrc);
    }
    uint32_t classOf(uint32_t id) const {
        const DpcFile::Entry* e = dpc.find(id);
        if (e == nullptr) throw std::runtime_error(hex8(id) + ": no such resource");
        return e->classCrc;
    }
    std::vector<uint8_t> link(uint32_t id) const {
        const DpcFile::Entry* e = dpc.find(id);
        if (e == nullptr) throw std::runtime_error(hex8(id) + ": no such resource");
        return dpc.link(*e);
    }
    std::vector<uint8_t> body(uint32_t id) const {
        const DpcFile::Entry* e = dpc.find(id);
        if (e == nullptr) throw std::runtime_error(hex8(id) + ": no such resource");
        auto b = dpc.body(*e);
        if (!b) throw std::runtime_error(hex8(id) + ": no body");
        return std::move(*b);
    }
    // Name CRC in the first four link bytes (every object class has one).
    uint32_t linkName(uint32_t id) const {
        const std::vector<uint8_t> l = link(id);
        if (l.size() < 4) throw std::runtime_error(hex8(id) + ": link shorter than its name");
        uint32_t v;
        std::memcpy(&v, l.data(), 4);
        return v;
    }

    template <typename T, typename Parse>
    const T& cached(std::unordered_map<uint32_t, T>& cache, uint32_t id, const char* what, Parse parse) {
        auto it = cache.find(id);
        if (it != cache.end()) return it->second;
        try {
            return cache.emplace(id, parse(id)).first->second;
        } catch (const std::exception& e) {
            throw std::runtime_error(std::string(what) + " " + hex8(id) + ": " + e.what());
        }
    }
    const Node& node(uint32_t id) { return cached(nodes, id, "Node_Z", [&](uint32_t i) { return parseNodeBody(body(i)); }); }
    const std::vector<NodeGroup>& gameObj(uint32_t id) {
        return cached(gameObjs, id, "GameObj_Z", [&](uint32_t i) { return parseGameObjBody(body(i)); });
    }
    const Lod& lod(uint32_t id) { return cached(lods, id, "Lod_Z", [&](uint32_t i) { return parseLodBody(body(i)); }); }
    const Skin& skin(uint32_t id) { return cached(skins, id, "Skin_Z", [&](uint32_t i) { return parseSkinBody(body(i)); }); }
    const Skel& skel(uint32_t id) { return cached(skels, id, "Skel_Z", [&](uint32_t i) { return parseSkelBody(body(i)); }); }
    const std::vector<std::string>& userDefine(uint32_t id) {
        return cached(userDefines, id, "UserDefine_Z", [&](uint32_t i) { return parseUserDefineBody(body(i)); });
    }
    const ObjectLink& objectLink(uint32_t id) {
        return cached(objectLinks, id, "object link", [&](uint32_t i) { return parseObjectLink(link(i)); });
    }
    // Named points of a part: from the Mesh_Z link or the Lod_Z body.
    const NamedSphere* findSphere(uint32_t object, uint32_t name) {
        const std::vector<NamedSphere>* spheres = nullptr;
        const uint32_t c = classOf(object);
        if (c == cls::Mesh)
            spheres = &cached(meshSpheres, object, "Mesh_Z link", [&](uint32_t i) { return parseMeshLinkSpheres(link(i)); });
        else if (c == cls::Lod)
            spheres = &lod(object).spheres;
        if (spheres)
            for (const NamedSphere& s : *spheres)
                if (s.name == name) return &s;
        return nullptr;
    }
    // A Lod_Z's object for LOD `lod` (the last one when it has fewer); 0 if none.
    uint32_t lodObject(uint32_t lodId, int level) {
        const std::vector<uint32_t>& objs = lod(lodId).objects;
        return objs.empty() ? 0 : objs[std::min(size_t(level), objs.size()) - 1];
    }
};

VehicleAssembler::VehicleAssembler(const DpcFile& dpc) : dpc_(dpc), names_(dpc), res_(std::make_unique<Resources>(dpc)) {
    for (uint32_t id : dpc.ids(cls::GameObj)) {
        const std::vector<NodeGroup>* groups = nullptr;
        try {
            groups = &res_->gameObj(id);
        } catch (const std::exception&) {
            continue;  // not a variant this code can read
        }
        VehicleVariant v;
        v.gameObjId = id;
        v.name = res_->linkName(id);
        v.label = names_(v.name);
        for (const NodeGroup& g : *groups) v.groups.push_back(g.name);
        variants_.push_back(std::move(v));
    }
}

VehicleAssembler::~VehicleAssembler() = default;

const VehicleVariant* VehicleAssembler::find(const std::string& selector) const {
    // GameObj_Z ID in hex ("3be8c9b1", "0x3be8c9b1").
    std::string s = trim(selector);
    if (startsWith(s, "0x") || startsWith(s, "0X")) s = s.substr(2);
    if (!s.empty() && s.find_first_not_of("0123456789abcdefABCDEF") == std::string::npos &&
        s.size() - std::min(s.size() - 1, s.find_first_not_of('0')) <= 8) {
        const uint32_t id = uint32_t(std::stoull(s.substr(s.size() > 8 ? s.size() - 8 : 0), nullptr, 16));
        for (const VehicleVariant& v : variants_)
            if (v.gameObjId == id) return &v;
    }
    const uint32_t crc = asoboCrc(selector);
    for (const VehicleVariant& v : variants_)
        if (v.name == crc) return &v;
    const std::string prefix = upper(selector) + "_";
    for (const VehicleVariant& v : variants_)
        for (const std::string& g : v.groups)
            if (startsWith(upper(g), prefix.c_str())) return &v;
    return nullptr;
}

AssembledVehicle VehicleAssembler::assemble(const VehicleVariant& variant, int lod) const {
    if (lod < 1) throw std::invalid_argument("LOD levels start at 1");
    Resources& R = *res_;
    const std::vector<NodeGroup>& groups = R.gameObj(variant.gameObjId);

    AssembledVehicle out;
    out.gameObjId = variant.gameObjId;
    out.name = variant.name;
    out.lod = lod;

    // The groups drawn at this LOD: the ones without a LOD suffix (COMMON) and <V>_LOD<lod>.
    std::vector<bool> chosen(groups.size());
    for (size_t i = 0; i < groups.size(); ++i) {
        const auto gl = groupLod(groups[i].name);
        chosen[i] = !gl || *gl == lod;
        if (chosen[i]) out.groups.push_back(groups[i].name);
    }

    // Commands find parts by name over all groups of the variant; the first node wins (spec S18).
    struct Part {
        uint32_t node = 0, object = 0;
        const std::string* group = nullptr;
    };
    std::unordered_map<uint32_t, Part> byName;
    std::vector<std::pair<const std::string*, uint32_t>> chosenNodes;  // (group, node)
    for (size_t i = 0; i < groups.size(); ++i)
        for (uint32_t n : groups[i].nodes) {
            if (!R.has(n, cls::Node)) continue;
            const uint32_t obj = R.node(n).object;
            if (obj != 0 && R.has(obj)) byName.try_emplace(R.linkName(obj), Part{n, obj, &groups[i].name});
            if (chosen[i]) chosenNodes.emplace_back(&groups[i].name, n);
        }

    // The vehicle skeleton: a chosen Lod_Z -> Skin_Z (this LOD) -> Skel_Z whose
    // bones carry commands; the one with the most bones (spec, port step 2).
    for (const auto& [group, n] : chosenNodes) {
        const uint32_t obj = R.node(n).object;
        if (!R.has(obj, cls::Lod)) continue;
        const uint32_t skin = R.lodObject(obj, lod);
        if (skin == 0 || !R.has(skin, cls::Skin)) continue;
        const uint32_t sk = R.objectLink(skin).dataId;
        if (!R.has(sk, cls::Skel)) continue;
        const Skel& s = R.skel(sk);
        if (std::none_of(s.bones.begin(), s.bones.end(), [](const Bone& b) { return b.userDefine != 0; })) continue;
        if (!out.skeleton || s.bones.size() > R.skel(*out.skeleton).bones.size()) out.skeleton = sk;
    }

    // Adds the meshes of one placed object: a Mesh_Z, or a Lod_Z's object for
    // this LOD (a Skin_Z drawn in bind pose, or a Mesh_Z); an Omni_Z is a light.
    auto addParts = [&](uint32_t node, const std::string& group, uint32_t obj, const Mat4& world, PlacedPart tmpl) {
        tmpl.world = world;
        tmpl.nodeId = node;
        tmpl.objectId = obj;
        tmpl.objectClass = R.classOf(obj);
        tmpl.objectName = R.linkName(obj);
        tmpl.group = group;
        if (tmpl.objectClass == cls::Mesh) {
            tmpl.meshId = obj;
            out.parts.push_back(tmpl);
        } else if (tmpl.objectClass == cls::Lod) {
            const uint32_t o = R.lodObject(obj, lod);
            if (o != 0 && R.has(o, cls::Skin)) {
                tmpl.shape = PartShape::SkinBindPose;
                tmpl.skinId = o;
                tmpl.skelId = R.objectLink(o).dataId;
                for (uint32_t mesh : R.skin(o).meshes)
                    if (R.has(mesh, cls::Mesh)) {
                        tmpl.meshId = mesh;
                        out.parts.push_back(tmpl);
                    }
            } else if (o != 0 && R.has(o, cls::Mesh)) {
                tmpl.meshId = o;
                out.parts.push_back(tmpl);
            }
        } else if (tmpl.objectClass == cls::Omni) {
            tmpl.shape = PartShape::Light;
            out.parts.push_back(tmpl);
        }
    };

    std::unordered_set<uint32_t> drawnObjects;
    if (out.skeleton) {
        const std::vector<Bone>& bones = R.skel(*out.skeleton).bones;

        // Link records, wheels and hub points from the bones' commands (spec S18).
        struct Record {
            Placement how = Placement::Link;
            int bone = 0;
            std::string drawn, text;
            bool atPoint = false;  // V:LINKM / V:LINKMM: at a named point of another part
            PointRef point;
            bool snap = false;  // V:LINK naming its own bone: take the bone's rotation
        };
        struct Wheel {
            int bone;
            std::string drawn, text;
        };
        std::vector<Record> records;
        std::vector<Wheel> wheels;
        std::unordered_map<int, PointRef> hubs;
        for (size_t k = 0; k < bones.size(); ++k) {
            const Bone& b = bones[k];
            if (b.userDefine == 0 || !R.has(b.userDefine, cls::UserDefine)) continue;
            for (const std::string& line : R.userDefine(b.userDefine)) {
                auto cmd = parseBoneCommand(line);
                if (!cmd || !startsWith(cmd->keyword, "V:") || !cmd->appliesToLod(lod)) continue;
                const std::string& kw = cmd->keyword;
                const std::vector<std::string>& a = cmd->args;
                const bool light = std::any_of(cmd->tags.begin(), cmd->tags.end(), [](const std::string& t) { return startsWith(upper(t), "LIGHT:"); });
                Record r;
                r.bone = int(k);
                r.text = cmd->text;
                if (kw == "V:LINK" && a.size() >= 2 && !light) {
                    r.drawn = a[1];
                    r.snap = asoboCrc(a[0]) == b.name;
                } else if ((kw == "V:LINKB" || kw == "V:LINKR") && a.size() >= 2) {
                    r.how = kw == "V:LINKB" ? Placement::LinkB : Placement::LinkR;
                    r.drawn = a[1];
                } else if (kw == "V:LINKM" && a.size() >= 2) {
                    r.how = Placement::LinkM;
                    r.drawn = a[1];
                    r.atPoint = true;
                    r.point = splitPoint(a[0]);
                } else if (kw == "V:LINKMM" && a.size() >= 3 && !light && upper(a[1]) != "LIGHT") {
                    r.how = Placement::LinkMM;
                    r.drawn = a[1];
                    r.atPoint = true;
                    r.point = splitPoint(a[0]);
                } else if (kw == "V:WHEEL" && !a.empty()) {
                    wheels.push_back({int(k), a[0], cmd->text});
                    continue;
                } else if (kw == "V:LINKWHEEL" && !a.empty()) {
                    hubs[int(k)] = splitPoint(a[0]);
                    continue;
                } else {
                    out.otherCommands.emplace_back(int(k), cmd->text);
                    continue;
                }
                records.push_back(std::move(r));
            }
        }

        // Place the records. A part at a point of part A waits until A is
        // placed (the engine sorts them topologically).
        std::unordered_map<uint32_t, Mat4> worldOf;  // placed object -> world matrix
        const uint32_t lookAt = asoboCrc("LOCAL_LOOKAT");
        std::vector<const Record*> pending;
        for (const Record& r : records) pending.push_back(&r);
        for (bool progress = true; !pending.empty() && progress;) {
            progress = false;
            const std::vector<const Record*> pass = pending;
            for (const Record* r : pass) {
                auto done = [&] {
                    pending.erase(std::find(pending.begin(), pending.end(), r));
                    progress = true;
                };
                auto hit = byName.find(asoboCrc(r->drawn));
                if (hit == byName.end()) {
                    out.problems.push_back("no part named \"" + r->drawn + "\" (" + r->text + ")");
                    done();
                    continue;
                }
                const Part& part = hit->second;
                const Bone& K = bones[size_t(r->bone)];
                // Position: the commanding bone, or the named point of part A.
                Vec3 pos = position(K.world);
                if (r->atPoint) {
                    auto a = byName.find(r->point.part);
                    if (a == byName.end() || worldOf.count(a->second.object) == 0) continue;
                    if (const NamedSphere* s = R.findSphere(a->second.object, r->point.point))
                        pos = transformPoint(worldOf[a->second.object], s->centre);
                    else
                        out.problems.push_back("no point " + r->point.partText + "+" + r->point.pointText + "; using the bone (" + r->text + ")");
                }
                // Rotation: the vehicle's axes (hinge angles are 0 at rest),
                // or the bone's own rotation for a same-bone V:LINK.
                Mat4 w;
                if (r->snap) {
                    w = rotation(K.restQuat);
                    w[12] = pos[0];
                    w[13] = pos[1];
                    w[14] = pos[2];
                } else {
                    w = translation(pos);
                }
                PlacedPart t;
                t.placement = r->how;
                t.bone = r->bone;
                t.boneName = K.name;
                t.command = r->text;
                // The game skips records whose part has no LOCAL_LOOKAT point
                // (lights excepted); such a part stays at its node matrix.
                if (R.findSphere(part.object, lookAt) == nullptr && R.classOf(part.object) != cls::Omni) {
                    t.keptNodeMatrix = true;
                    w = toMat4(R.node(part.node).world);
                }
                worldOf[part.object] = w;
                drawnObjects.insert(part.object);
                addParts(part.node, *part.group, part.object, w, std::move(t));
                done();
            }
        }
        for (const Record* r : pending) out.problems.push_back("part it hangs from is not placed (" + r->text + ")");

        // Wheels: at the hub point of the bone's V:LINKWHEEL, else at the
        // bone; identity rotation.
        struct WheelPos {
            const Wheel* wheel;
            Part part;
            Vec3 pos;
            std::string hub;
        };
        std::vector<WheelPos> placed;
        for (const Wheel& wh : wheels) {
            auto hit = byName.find(asoboCrc(wh.drawn));
            if (hit == byName.end()) {
                out.problems.push_back("no wheel named \"" + wh.drawn + "\"");
                continue;
            }
            WheelPos p{&wh, hit->second, position(bones[size_t(wh.bone)].world), "bone"};
            if (auto h = hubs.find(wh.bone); h != hubs.end()) {
                auto a = byName.find(h->second.part);
                if (a != byName.end() && worldOf.count(a->second.object) != 0)
                    if (const NamedSphere* s = R.findSphere(a->second.object, h->second.point)) {
                        p.pos = transformPoint(worldOf[a->second.object], s->centre);
                        p.hub = h->second.partText + "+" + h->second.pointText;
                    }
            }
            placed.push_back(std::move(p));
        }
        // When one mesh serves both sides, the x > 0 copy is turned 180 degrees
        // about y (spec S14, S18: wheel slots w < 3 vs w + 3).
        std::unordered_map<uint32_t, unsigned> sides;  // object -> bit 0: x <= 0 seen, bit 1: x > 0 seen
        for (const WheelPos& p : placed) sides[p.part.object] |= p.pos[0] > 0 ? 2u : 1u;
        for (const WheelPos& p : placed) {
            PlacedPart t;
            t.placement = Placement::Wheel;
            t.bone = p.wheel->bone;
            t.boneName = bones[size_t(p.wheel->bone)].name;
            t.command = p.wheel->text;
            t.hub = p.hub;
            Mat4 w = translation(p.pos);
            if (sides[p.part.object] == 3u && p.pos[0] > 0) {
                w = multiply(kRotY180, w);
                t.placement = Placement::WheelFlipped;
            }
            drawnObjects.insert(p.part.object);
            addParts(p.part.node, *p.part.group, p.part.object, w, std::move(t));
        }
    }

    // Nodes of the chosen groups that no command places. The vehicle code
    // does not draw them; they are listed at their node matrix.
    for (const auto& [group, n] : chosenNodes) {
        const uint32_t obj = R.node(n).object;
        if (obj == 0 || !R.has(obj) || drawnObjects.count(obj) != 0) continue;
        if (R.has(obj, cls::Lod) && out.skeleton) {
            const uint32_t o = R.lodObject(obj, lod);
            if (R.has(o, cls::Skin) && R.objectLink(o).dataId == *out.skeleton) continue;  // the skeleton's holder
        }
        addParts(n, *group, obj, toMat4(R.node(n).world), PlacedPart{});
    }
    return out;
}

std::vector<VehicleVariant> listVehicleVariants(const DpcFile& dpc) { return VehicleAssembler(dpc).variants(); }

}  // namespace fuel
