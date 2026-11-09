// fuelvehpack: CUSTOM VEHICLES (port deviation) tool. Builds a custom vehicle's archive (a FUEL-format .DPC, see
// core/custom_vehicles.hpp) from a Blender export, and inspects FUEL's own vehicles to model it on them.
//
//   fuelvehpack inspect DATA_DIR DPC VARIANT [--dump DIR]
//     every resource the variant reaches (GameObj_Z groups, Node_Z, Lod_Z, Skin_Z, Skel_Z bones and their V: commands,
//     Mesh_Z links / bodies, Material_Z, Bitmap_Z, Omni_Z, the _DYN proxy); --dump DIR also writes the assembled LOD1
//     parts as DIR/<variant>.obj (vehicle space, FUEL axes: x left, y up, z forward) and raw link / body bytes of the
//     first resource of each class (DIR/<class>.link / .body) as layout references.
//   fuelvehpack sounds DATA_DIR DPC VARIANT OUTDIR
//     the variant's engine bank (<V>_SKEL pair list 2) as raw 16-bit PCM per symbol, to choose a PortVehicleSound bank.
//   fuelvehpack build DATA_DIR SPEC_DIR OUT.DPC
//     SPEC_DIR/pack.json (written by the Blender export script volvo-work/scripts/export_pack.py and friends) + its
//     mesh / texture files -> OUT.DPC. See writePack below for what is generated.
//
// Assets never enter the repo: SPEC_DIR and OUT.DPC live in ~/re-work/fuel/custom-assets/vehicles/<pack>/.
#include "core/assembly.hpp"
#include "core/dpc.hpp"
#include "core/reader.hpp"
#include "core/resources.hpp"
#include "core/vehicle_params.hpp"

#include <algorithm>
#include <array>
#include <cctype>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <filesystem>
#include <fstream>
#include <map>
#include <set>
#include <sstream>
#include <string>
#include <vector>

namespace fs = std::filesystem;
using namespace fuel;

namespace {

const char* const kClassNames[] = {"Node_Z",        "Lod_Z",     "Skin_Z",     "Skel_Z",        "GameObj_Z",     "UserDefine_Z", "Omni_Z",
                                   "Mesh_Z",        "MeshData_Z", "Material_Z", "Bitmap_Z",      "MaterialObj_Z", "MaterialAnim_Z",
                                   "Sound_Z",       "Animation_Z", "UserConstant_Z", "LodData_Z", "SkelData_Z", "Light_Z", "Spline_Z",
                                   "CollisionVol_Z", "Particles_Z", "RotShape_Z", "Surface_Z", "SoundData_Z", "Binary_Z"};

std::string className(uint32_t c) {
    for (const char* n : kClassNames)
        if (asoboCrc(n) == c) return n;
    char b[16];
    std::snprintf(b, sizeof b, "?%08x", c);
    return b;
}

std::string hex8(uint32_t v) {
    char b[12];
    std::snprintf(b, sizeof b, "%08x", v);
    return b;
}

uint32_t rd32(const std::vector<uint8_t>& b, size_t o) {
    uint32_t v = 0;
    if (o + 4 <= b.size()) std::memcpy(&v, &b[o], 4);
    return v;
}
float rdf(const std::vector<uint8_t>& b, size_t o) {
    float v = 0;
    if (o + 4 <= b.size()) std::memcpy(&v, &b[o], 4);
    return v;
}

int inspect(const std::string& data, const std::string& dpcRel, const std::string& variant, const std::string& dumpDir) {
    const std::string path = !dpcRel.empty() && dpcRel[0] == '/' ? dpcRel : data + "/" + dpcRel;
    DpcFile dpc(path);
    VehicleAssembler va(dpc);
    const VehicleVariant* v = va.find(variant);
    if (!v) throw std::runtime_error("no variant " + variant);
    const NameTable& names = va.names();
    auto link = [&](uint32_t id) { return dpc.link(*dpc.find(id)); };
    auto body = [&](uint32_t id) -> std::vector<uint8_t> {
        const auto* e = dpc.find(id);
        auto b = e ? dpc.body(*e) : std::nullopt;
        return b ? *b : std::vector<uint8_t>{};
    };
    std::set<uint32_t> seen;
    std::vector<uint32_t> order;
    std::vector<uint32_t> todo{v->gameObjId};
    // The variant's _DYN / _SKEL Lods by name, too.
    for (const char* suffix : {"_DYN", "_SKEL"})
        for (uint32_t id : dpc.ids(asoboCrc("Lod_Z")))
            if (rd32(link(id), 0) == asoboCrc(variant + suffix)) todo.push_back(id);
    std::map<uint32_t, int> perClass;
    std::map<uint32_t, uint32_t> firstOfClass;
    while (!todo.empty()) {
        const uint32_t id = todo.back();
        todo.pop_back();
        if (id == 0 || seen.count(id) || !dpc.find(id)) continue;
        seen.insert(id), order.push_back(id);
        const auto* e = dpc.find(id);
        const uint32_t c = e->classCrc;
        ++perClass[c];
        firstOfClass.emplace(c, id);
        const std::vector<uint8_t> l = link(id), b = body(id);
        const std::string cn = className(c);
        std::printf("%s %-14s link %4zu body %7zu name %s\n", hex8(id).c_str(), cn.c_str(), l.size(), b.size(), names(rd32(l, 0)).c_str());
        auto push = [&](uint32_t x) {
            if (x && dpc.find(x) && !seen.count(x)) todo.push_back(x);
        };
        try {
            if (cn == "GameObj_Z") {
                for (const NodeGroup& g : parseGameObjBody(b)) {
                    std::printf("    group %s value %u nodes %zu:", g.name.c_str(), g.value, g.nodes.size());
                    for (uint32_t n : g.nodes) {
                        push(n);
                        uint32_t obj = 0;
                        try {
                            obj = parseNodeBody(body(n)).object;
                        } catch (const std::exception&) {
                        }
                        std::printf(" %s=%s(%s)", hex8(n).c_str(), obj && dpc.find(obj) ? className(dpc.find(obj)->classCrc).c_str() : "-",
                                    obj && dpc.find(obj) ? names(rd32(link(obj), 0)).c_str() : "");
                    }
                    std::printf("\n");
                }
            } else if (cn == "Node_Z") {
                const Node n = parseNodeBody(b);
                std::printf("    parent %s object %s data %s ud %s light %s flags %08x lodScale %.3f T(%.3f %.3f %.3f)\n", hex8(n.parent).c_str(),
                            hex8(n.object).c_str(), hex8(n.objectData).c_str(), hex8(n.userDefine).c_str(), hex8(n.light).c_str(), n.flags, n.lodScale,
                            n.world[12], n.world[13], n.world[14]);
                push(n.object), push(n.objectData), push(n.userDefine), push(n.light);
            } else if (cn == "Lod_Z") {
                const Lod L = parseLodBody(b);
                std::printf("    near %.2f cull %.2f D %.2f viewDist %.2f objects", L.nearFade, L.hardCull, L.baseDistance, rdf(l, 88));
                for (uint32_t o : L.objects) std::printf(" %s", hex8(o).c_str()), push(o);
                std::printf(" anims %zu ud %s\n", L.animations.size(), hex8(L.userDefine).c_str());
                for (const NamedSphere& s : L.spheres)
                    std::printf("      sphere %-16s (%.3f %.3f %.3f) r %.3f type %u\n", names(s.name).c_str(), s.centre[0], s.centre[1], s.centre[2], s.radius,
                                s.type);
                for (const NamedBox& x : L.boxes)
                    std::printf("      box %-16s c(%.3f %.3f %.3f) half(%.3f %.3f %.3f) type %u\n", names(x.name).c_str(), x.rows[3], x.rows[7], x.rows[11],
                                x.half[0], x.half[1], x.half[2], x.type);
                push(L.userDefine);
                if (cn == "Lod_Z") {  // sound pairs etc. are not followed (Sound_Z)
                }
            } else if (cn == "Skin_Z") {
                const Skin s = parseSkinBody(b);
                const ObjectLink ol = parseObjectLink(l);
                std::printf("    skel %s meshes", hex8(ol.dataId).c_str());
                for (uint32_t m : s.meshes) std::printf(" %s", hex8(m).c_str()), push(m);
                std::printf(" sets %zu\n", s.sets.size());
                push(ol.dataId);
            } else if (cn == "Skel_Z") {
                const Skel s = parseSkelBody(b);
                for (size_t k = 0; k < s.bones.size(); ++k) {
                    const Bone& bn = s.bones[k];
                    std::printf("    bone %2zu %-14s parent %2d T(%.3f %.3f %.3f) local(%.3f %.3f %.3f) q(%.2f %.2f %.2f %.2f)\n", k, names(bn.name).c_str(),
                                bn.parent, bn.world[12], bn.world[13], bn.world[14], bn.localT[0], bn.localT[1], bn.localT[2], bn.restQuat[0], bn.restQuat[1],
                                bn.restQuat[2], bn.restQuat[3]);
                    if (bn.userDefine && dpc.find(bn.userDefine))
                        for (const std::string& line : parseUserDefineBody(body(bn.userDefine))) std::printf("        | %s\n", line.c_str());
                    push(bn.userDefine);
                }
                for (const auto& p : s.points) std::printf("    point %s (%.3f %.3f %.3f) r %.3f\n", names(p.name).c_str(), p.c[0], p.c[1], p.c[2], p.r);
                for (const auto& x : s.boxes) std::printf("    obb %s\n", names(x.name).c_str());
                std::printf("    palettes %zu\n", s.palettes.size());
            } else if (cn == "UserDefine_Z") {
                for (const std::string& line : parseUserDefineBody(b)) std::printf("    | %s\n", line.c_str());
            } else if (cn == "Mesh_Z") {
                const MeshLink ml = parseMeshLink(l);
                const Mesh m = parseMeshBody(b, ml.flags);
                MeshStats st;
                checkMesh(m, &st);
                std::printf("    data %s sphere (%.2f %.2f %.2f) r %.2f flags %08x tris %u bounds (%.2f %.2f %.2f)..(%.2f %.2f %.2f) vbs", hex8(ml.dataId).c_str(),
                            ml.sphere[0], ml.sphere[1], ml.sphere[2], ml.sphere[3], ml.flags, st.triangles, st.boundsMin[0], st.boundsMin[1], st.boundsMin[2],
                            st.boundsMax[0], st.boundsMax[1], st.boundsMax[2]);
                for (const auto& x : m.vbs) std::printf(" %u/%u/%x", x.count, x.stride, x.flags);
                std::printf(" batches %zu morphs %zu coll %zu\n", m.batches.size(), m.morphs.size(), m.collisionTris.size() / 4);
                for (const Batch& bt : m.batches) {
                    // Winding: triangles whose (b - a) x (c - a) points along the stored vertex normal (D3DCOLOR at 16).
                    int along = 0, against = 0;
                    if (bt.vb < m.vbs.size() && m.vbs[bt.vb].stride >= 20) {
                        const VertexBuffer& vb = m.vbs[bt.vb];
                        const auto tris = batchTriangles(m, bt);
                        auto P = [&](uint32_t i, int k) { float f; std::memcpy(&f, &vb.data[size_t(i) * vb.stride + size_t(k) * 4], 4); return f; };
                        auto N = [&](uint32_t i, int k) { return vb.data[size_t(i) * vb.stride + 16 + size_t(2 - k)] / 127.5f - 1.0f; };  // BGRA
                        for (size_t t = 0; t + 2 < tris.size(); t += 3) {
                            const uint32_t a = tris[t], b = tris[t + 1], c = tris[t + 2];
                            if (std::max({a, b, c}) >= vb.count) continue;
                            const float ux = P(b, 0) - P(a, 0), uy = P(b, 1) - P(a, 1), uz = P(b, 2) - P(a, 2);
                            const float vx = P(c, 0) - P(a, 0), vy = P(c, 1) - P(a, 1), vz = P(c, 2) - P(a, 2);
                            const float cx = uy * vz - uz * vy, cy = uz * vx - ux * vz, cz = ux * vy - uy * vx;
                            (cx * N(a, 0) + cy * N(a, 1) + cz * N(a, 2) > 0 ? along : against)++;
                        }
                    }
                    std::printf("      batch vb %u type %u prims %u mat %u winding along %d against %d\n", bt.vb, bt.vertexType, bt.primitiveCount, bt.material,
                                along, against);
                }
                for (const NamedSphere& s : parseMeshLinkSpheres(l))
                    std::printf("      sphere %-16s (%.3f %.3f %.3f) r %.3f\n", names(s.name).c_str(), s.centre[0], s.centre[1], s.centre[2], s.radius);
                for (const NamedBox& x : parseMeshLinkBoxes(l))
                    std::printf("      box %-16s c(%.3f %.3f %.3f) half(%.3f %.3f %.3f)\n", names(x.name).c_str(), x.rows[3], x.rows[7], x.rows[11], x.half[0],
                                x.half[1], x.half[2]);
                for (uint32_t mat : m.materials) push(mat);
                push(ml.dataId);
            } else if (cn == "Material_Z") {
                const Material m = parseMaterialBody(b);
                std::printf("    diffuse %.2f %.2f %.2f a %.2f emis %.2f %.2f %.2f spec %.2f %.2f %.2f p %.1f fresnel %.2f %.2f mask %04x flags %08x code %u tex",
                            m.diffuse[0], m.diffuse[1], m.diffuse[2], m.alpha, m.emissive[0], m.emissive[1], m.emissive[2], m.specular[0], m.specular[1],
                            m.specular[2], m.specular[3], m.fresnel[0], m.fresnel[1], m.textures.size() ? m.textureMask : 0, m.flags, m.flags & 31);
                for (uint32_t t : m.textures) std::printf(" %s", t ? hex8(t).c_str() : "-"), push(t);
                std::printf("\n");
            } else if (cn == "Bitmap_Z") {
                const BitmapInfo bi = parseBitmapLink(l);
                std::printf("    %ux%u %s mips %u alpha %u\n", bi.width, bi.height, pixelFormatName(bi.format), bi.mips, bi.alphaKind);
            } else if (cn == "Omni_Z") {
                std::printf("    (omni)\n");
            }
        } catch (const std::exception& ex) {
            std::printf("    ** %s\n", ex.what());
        }
    }
    std::printf("# classes:");
    for (const auto& [c, n] : perClass) std::printf(" %s %d", className(c).c_str(), n);
    std::printf("\n");
    if (!dumpDir.empty()) {
        fs::create_directories(dumpDir);
        for (const auto& [c, id] : firstOfClass) {
            const auto l = link(id), b = body(id);
            std::ofstream(dumpDir + "/" + className(c) + ".link", std::ios::binary).write(reinterpret_cast<const char*>(l.data()), std::streamsize(l.size()));
            std::ofstream(dumpDir + "/" + className(c) + ".body", std::ios::binary).write(reinterpret_cast<const char*>(b.data()), std::streamsize(b.size()));
        }
        // The assembled LOD1 parts as an OBJ (one object per part, vehicle space).
        const AssembledVehicle av = va.assemble(*v, 1);
        std::ofstream obj(dumpDir + "/" + variant + ".obj");
        size_t base = 1;
        for (const PlacedPart& p : av.parts) {
            if (!p.drawn()) continue;
            const auto l = link(p.meshId);
            const Mesh m = parseMeshBody(body(p.meshId), parseMeshLink(l).flags);
            obj << "o " << names(p.objectName) << "_" << hex8(p.meshId) << "\n";
            for (const Batch& bt : m.batches) {
                if (bt.vb >= m.vbs.size()) continue;
                const VertexBuffer& vb = m.vbs[bt.vb];
                const auto tris = batchTriangles(m, bt);
                std::map<uint32_t, size_t> remap;
                for (uint32_t i : tris) {
                    if (remap.count(i) || i >= vb.count) continue;
                    float x, y, z;
                    std::memcpy(&x, &vb.data[size_t(i) * vb.stride], 4);
                    std::memcpy(&y, &vb.data[size_t(i) * vb.stride + 4], 4);
                    std::memcpy(&z, &vb.data[size_t(i) * vb.stride + 8], 4);
                    const auto& w = p.world;
                    obj << "v " << x * w[0] + y * w[4] + z * w[8] + w[12] << " " << x * w[1] + y * w[5] + z * w[9] + w[13] << " "
                        << x * w[2] + y * w[6] + z * w[10] + w[14] << "\n";
                    remap[i] = base++;
                }
                for (size_t t = 0; t + 2 < tris.size(); t += 3)
                    if (remap.count(tris[t]) && remap.count(tris[t + 1]) && remap.count(tris[t + 2]))
                        obj << "f " << remap[tris[t]] << " " << remap[tris[t + 1]] << " " << remap[tris[t + 2]] << "\n";
            }
        }
        std::printf("# wrote %s/%s.obj\n", dumpDir.c_str(), variant.c_str());
    }
    return 0;
}

// ---------------------------------------------------------------- build: a minimal JSON reader (pack.json only)
struct Json {
    enum Kind { Null, Bool, Num, Str, Arr, Obj } kind = Null;
    double num = 0;
    bool b = false;
    std::string str;
    std::vector<Json> arr;
    std::vector<std::pair<std::string, Json>> obj;
    const Json* get(const std::string& k) const {
        for (const auto& [n, v] : obj)
            if (n == k) return &v;
        return nullptr;
    }
    const Json& operator[](const std::string& k) const {
        const Json* j = get(k);
        if (!j) throw std::runtime_error("pack.json: missing \"" + k + "\"");
        return *j;
    }
    const Json& operator[](size_t i) const { return arr.at(i); }
    float f(size_t i) const { return float(arr.at(i).num); }
    std::array<float, 3> v3() const { return {f(0), f(1), f(2)}; }
};

class JsonParser {
public:
    explicit JsonParser(const std::string& s) : s_(s) {}
    Json parse() {
        Json j = value();
        ws();
        if (p_ != s_.size()) fail("trailing text");
        return j;
    }

private:
    [[noreturn]] void fail(const char* w) { throw std::runtime_error(std::string("pack.json: ") + w + " at " + std::to_string(p_)); }
    void ws() {
        while (p_ < s_.size() && std::isspace(static_cast<unsigned char>(s_[p_]))) ++p_;
    }
    std::string string() {
        if (s_[p_] != '"') fail("string expected");
        ++p_;
        std::string out;
        while (p_ < s_.size() && s_[p_] != '"') {
            char c = s_[p_++];
            if (c == '\\' && p_ < s_.size()) {
                const char e = s_[p_++];
                c = e == 'n' ? '\n' : e == 't' ? '\t' : e;
                if (e == 'u') {  // \uXXXX: ASCII only
                    c = char(std::stoi(s_.substr(p_, 4), nullptr, 16));
                    p_ += 4;
                }
            }
            out += c;
        }
        ++p_;
        return out;
    }
    Json value() {
        ws();
        if (p_ >= s_.size()) fail("unexpected end");
        Json j;
        const char c = s_[p_];
        if (c == '{') {
            j.kind = Json::Obj;
            ++p_;
            ws();
            if (s_[p_] == '}') return ++p_, j;
            for (;;) {
                ws();
                std::string k = string();
                ws();
                if (s_[p_++] != ':') fail("':' expected");
                j.obj.emplace_back(k, value());
                ws();
                if (s_[p_] == ',') {
                    ++p_;
                    continue;
                }
                if (s_[p_++] != '}') fail("'}' expected");
                return j;
            }
        }
        if (c == '[') {
            j.kind = Json::Arr;
            ++p_;
            ws();
            if (s_[p_] == ']') return ++p_, j;
            for (;;) {
                j.arr.push_back(value());
                ws();
                if (s_[p_] == ',') {
                    ++p_;
                    continue;
                }
                if (s_[p_++] != ']') fail("']' expected");
                return j;
            }
        }
        if (c == '"') {
            j.kind = Json::Str;
            j.str = string();
            return j;
        }
        if (s_.compare(p_, 4, "true") == 0) return p_ += 4, j.kind = Json::Bool, j.b = true, j;
        if (s_.compare(p_, 5, "false") == 0) return p_ += 5, j.kind = Json::Bool, j;
        if (s_.compare(p_, 4, "null") == 0) return p_ += 4, j;
        size_t used = 0;
        j.num = std::stod(s_.substr(p_, 32), &used);
        p_ += used;
        j.kind = Json::Num;
        return j;
    }
    const std::string& s_;
    size_t p_ = 0;
};

// ---------------------------------------------------------------- build: byte helpers
struct Bytes {
    std::vector<uint8_t> b;
    void u8(uint8_t v) { b.push_back(v); }
    void u16(uint16_t v) { raw(&v, 2); }
    void u32(uint32_t v) { raw(&v, 4); }
    void f32(float v) { raw(&v, 4); }
    void raw(const void* p, size_t n) { b.insert(b.end(), static_cast<const uint8_t*>(p), static_cast<const uint8_t*>(p) + n); }
    void raw(const std::vector<uint8_t>& v) { b.insert(b.end(), v.begin(), v.end()); }
};
void put32(std::vector<uint8_t>& b, size_t o, uint32_t v) {
    if (o + 4 > b.size()) throw std::runtime_error("template too short");
    std::memcpy(&b[o], &v, 4);
}
void putf(std::vector<uint8_t>& b, size_t o, float v) {
    if (o + 4 > b.size()) throw std::runtime_error("template too short");
    std::memcpy(&b[o], &v, 4);
}

uint32_t nameCrc(const Json& j) {  // {"name": "..."} or {"crc": "hex"}
    if (const Json* c = j.get("crc")) return uint32_t(std::stoul(c->str, nullptr, 16));
    return asoboCrc(j["name"].str);
}

// The resources of the archive being written.
struct Archive {
    struct Obj {
        uint32_t cls = 0, id = 0;
        std::vector<uint8_t> link, body;
    };
    std::vector<Obj> objs;
    std::set<uint32_t> ids;
    std::string prefix;
    uint32_t id(const std::string& what) const { return asoboCrc("DB:>PORT>VEHICLES>" + prefix + ">" + what); }
    void add(const char* cls, uint32_t id, std::vector<uint8_t> link, std::vector<uint8_t> body) {
        if (!ids.insert(id).second) throw std::runtime_error("duplicate resource id " + hex8(id));
        objs.push_back({asoboCrc(cls), id, std::move(link), std::move(body)});
    }
    // FUEL's container (dpc.cpp): a 0x800-byte header with the version and the block table, then one block of objects
    // packed back to back (0x18-byte header: data size, link size, body size, 0 = stored raw, class, id), sector padded.
    void write(const std::string& path) const {
        std::vector<uint8_t> block;
        for (const Obj& o : objs) {
            Bytes h;
            h.u32(uint32_t(o.link.size() + o.body.size())), h.u32(uint32_t(o.link.size())), h.u32(uint32_t(o.body.size())), h.u32(0);
            h.u32(o.cls), h.u32(o.id);
            block.insert(block.end(), h.b.begin(), h.b.end());
            block.insert(block.end(), o.link.begin(), o.link.end());
            block.insert(block.end(), o.body.begin(), o.body.end());
        }
        const uint32_t dataSize = uint32_t(block.size());
        const uint32_t padded = (dataSize + 0x7ff) & ~0x7ffu;
        block.resize(padded, 0);
        std::vector<uint8_t> header(0x800, 0);
        std::memcpy(header.data(), kDpcVersion, std::strlen(kDpcVersion));
        put32(header, 0x104, 1);
        put32(header, 0x120, uint32_t(objs.size()));
        put32(header, 0x124, padded);
        put32(header, 0x128, dataSize);
        std::ofstream f(path, std::ios::binary);
        if (!f) throw std::runtime_error("cannot write " + path);
        f.write(reinterpret_cast<const char*>(header.data()), std::streamsize(header.size()));
        f.write(reinterpret_cast<const char*>(block.data()), std::streamsize(block.size()));
    }
};

// The donor variant's resources used as layout templates (bytes the port does not interpret are kept as FUEL has them).
struct Donor {
    const DpcFile& dpc;
    explicit Donor(const DpcFile& d) : dpc(d) {}
    std::vector<uint8_t> link(uint32_t id) const { return dpc.link(*dpc.find(id)); }
    std::vector<uint8_t> body(uint32_t id) const {
        auto b = dpc.body(*dpc.find(id));
        if (!b) throw std::runtime_error("donor " + hex8(id) + " has no body");
        return *b;
    }
    uint32_t classOf(uint32_t id) const { return dpc.find(id) ? dpc.find(id)->classCrc : 0; }
};

std::vector<uint8_t> readFileBytes(const std::string& path) {
    std::ifstream f(path, std::ios::binary);
    if (!f) throw std::runtime_error("cannot read " + path);
    return std::vector<uint8_t>((std::istreambuf_iterator<char>(f)), std::istreambuf_iterator<char>());
}

// Bitmap_Z: A8R8G8B8 (code 0x0c, B G R A bytes) with a full box-filtered mip chain.
std::pair<std::vector<uint8_t>, std::vector<uint8_t>> makeBitmap(const std::vector<uint8_t>& tmplLink, uint32_t id, const std::string& file, int alphaKind) {
    const std::vector<uint8_t> raw = readFileBytes(file);
    if (raw.size() < 8) throw std::runtime_error(file + ": too short");
    uint32_t w, h;
    std::memcpy(&w, raw.data(), 4), std::memcpy(&h, raw.data() + 4, 4);
    if (raw.size() != 8 + size_t(w) * h * 4 || (w & (w - 1)) || (h & (h - 1))) throw std::runtime_error(file + ": bad size");
    std::vector<uint8_t> level(raw.begin() + 8, raw.end()), body;
    uint32_t mips = 0;
    for (uint32_t lw = w, lh = h;; ++mips) {
        for (size_t i = 0; i < size_t(lw) * lh; ++i) {  // RGBA -> BGRA
            const uint8_t* p = &level[i * 4];
            body.insert(body.end(), {p[2], p[1], p[0], p[3]});
        }
        if (lw == 1 && lh == 1) {
            ++mips;
            break;
        }
        const uint32_t nw = std::max(1u, lw / 2), nh = std::max(1u, lh / 2);
        std::vector<uint8_t> next(size_t(nw) * nh * 4);
        for (uint32_t y = 0; y < nh; ++y)
            for (uint32_t x = 0; x < nw; ++x)
                for (int c = 0; c < 4; ++c) {
                    int s = 0, n = 0;
                    for (uint32_t dy = 0; dy < 2; ++dy)
                        for (uint32_t dx = 0; dx < 2; ++dx) {
                            const uint32_t sx = std::min(lw - 1, x * 2 + dx), sy = std::min(lh - 1, y * 2 + dy);
                            s += level[(size_t(sy) * lw + sx) * 4 + size_t(c)], ++n;
                        }
                    next[(size_t(y) * nw + x) * 4 + size_t(c)] = uint8_t((s + n / 2) / n);
                }
        level.swap(next), lw = nw, lh = nh;
    }
    std::vector<uint8_t> link = tmplLink;
    link.resize(32, 0);
    put32(link, 0, id);
    link[4] = link[5] = 0;  // 2D
    std::memcpy(&link[6], &w, 4);
    std::memcpy(&link[10], &h, 4);
    const uint32_t size = uint32_t(body.size());
    std::memcpy(&link[14], &size, 4);
    const float bias = 0;
    std::memcpy(&link[22], &bias, 4);
    link[0x1B] = uint8_t(mips - 1);
    link[0x1E] = 0x0C;
    link[0x1F] = uint8_t(alphaKind);
    return {link, body};
}

// FVM1 mesh -> Mesh_Z body (static vertices, stride 36: position, tangent and normal as D3DCOLOR, uv0, uv1) with one
// batch per material run; vertex buffers split at 65,535 vertices (16-bit indices).
struct BuiltMesh {
    std::vector<uint8_t> body;
    std::array<float, 3> lo{1e30f, 1e30f, 1e30f}, hi{-1e30f, -1e30f, -1e30f};
    std::vector<uint32_t> materials;
};
uint32_t packColour(float x, float y, float z, uint8_t a) {  // D3DCOLOR: x in R (byte 2), y in G (1), z in B (0)
    auto q = [](float v) { return uint32_t(std::lround(std::clamp((v * 0.5f + 0.5f) * 255.0f, 0.0f, 255.0f))); };
    return q(z) | (q(y) << 8) | (q(x) << 16) | (uint32_t(a) << 24);
}
BuiltMesh buildMesh(const std::string& file, const std::vector<uint32_t>& materialIds) {
    const std::vector<uint8_t> f = readFileBytes(file);
    if (f.size() < 16 || std::memcmp(f.data(), "FVM1", 4) != 0) throw std::runtime_error(file + ": not an FVM1 mesh");
    uint32_t nv, nt, nb;
    std::memcpy(&nv, &f[4], 4), std::memcpy(&nt, &f[8], 4), std::memcpy(&nb, &f[12], 4);
    const size_t need = 16 + size_t(nv) * 32 + size_t(nt) * 12 + size_t(nb) * 12;
    if (f.size() != need) throw std::runtime_error(file + ": size mismatch");
    const float* V = reinterpret_cast<const float*>(&f[16]);
    const uint32_t* T = reinterpret_cast<const uint32_t*>(&f[16 + size_t(nv) * 32]);
    const uint32_t* B = reinterpret_cast<const uint32_t*>(&f[16 + size_t(nv) * 32 + size_t(nt) * 12]);
    BuiltMesh out;
    // Materials used, in order; batches refer to them by index.
    std::map<uint32_t, uint16_t> matIndex;
    for (uint32_t k = 0; k < nb; ++k) {
        const uint32_t mi = B[k * 3];
        if (mi >= materialIds.size()) throw std::runtime_error(file + ": material index out of range");
        if (!matIndex.count(mi)) matIndex[mi] = uint16_t(out.materials.size()), out.materials.push_back(materialIds[mi]);
    }
    // Vertex buffers: each batch's triangles, vertices remapped into the current VB until it is full.
    struct VB {
        std::vector<uint8_t> data;
        uint32_t count = 0;
    };
    std::vector<VB> vbs;
    std::vector<uint16_t> indices;
    struct Bat {
        uint32_t vb, start, prims, minV, numV;
        uint16_t mat;
    };
    std::vector<Bat> bats;
    for (uint32_t k = 0; k < nb; ++k) {
        const uint32_t mi = B[k * 3], first = B[k * 3 + 1], count = B[k * 3 + 2];
        uint32_t t = first;
        while (t < first + count) {
            if (vbs.empty() || vbs.back().count > 60000) vbs.emplace_back();
            VB& vb = vbs.back();
            std::map<uint32_t, uint16_t> remap;
            Bat bt{uint32_t(vbs.size() - 1), uint32_t(indices.size()), 0, 0xffffffffu, 0, matIndex[mi]};
            uint32_t lo = 0xffffffffu, hi = 0;
            for (; t < first + count && vb.count + 3 <= 65535; ++t) {
                for (int c = 0; c < 3; ++c) {
                    const uint32_t vi = T[size_t(t) * 3 + size_t(c)];
                    if (vi >= nv) throw std::runtime_error(file + ": vertex index out of range");
                    auto it = remap.find(vi);
                    if (it == remap.end()) {
                        const float* v = V + size_t(vi) * 8;
                        Bytes vx;
                        vx.f32(v[0]), vx.f32(v[1]), vx.f32(v[2]);
                        // tangent: any unit vector across the normal (no normal maps on these materials)
                        const float nx = v[3], ny = v[4], nz = v[5];
                        float tx = -nz, ty = 0, tz = nx;
                        const float tl = std::sqrt(tx * tx + tz * tz);
                        if (tl < 1e-4f) tx = 1, tz = 0;
                        else tx /= tl, tz /= tl;
                        vx.u32(packColour(tx, ty, tz, 1));
                        vx.u32(packColour(nx, ny, nz, 255));
                        vx.f32(v[6]), vx.f32(v[7]), vx.f32(0), vx.f32(0);
                        vb.data.insert(vb.data.end(), vx.b.begin(), vx.b.end());
                        it = remap.emplace(vi, uint16_t(vb.count++)).first;
                        for (int a = 0; a < 3; ++a) out.lo[size_t(a)] = std::min(out.lo[size_t(a)], v[a]), out.hi[size_t(a)] = std::max(out.hi[size_t(a)], v[a]);
                    }
                    indices.push_back(it->second);
                    lo = std::min<uint32_t>(lo, it->second), hi = std::max<uint32_t>(hi, it->second);
                }
                ++bt.prims;
                if (vb.count > 65000) {
                    ++t;
                    break;
                }
            }
            bt.minV = lo, bt.numV = hi - lo + 1;
            bats.push_back(bt);
            if (t < first + count) vbs.emplace_back();
        }
    }
    Bytes b;
    for (int k = 0; k < 4; ++k) b.u32(0);  // point arrays
    b.u32(0), b.u32(0);                    // groups, groups 2
    b.u32(uint32_t(out.materials.size()));
    for (uint32_t m : out.materials) b.u32(m);
    b.u32(0), b.u32(0), b.u32(0);  // collision nodes, collision triangles, 0x20 records
    b.u32(uint32_t(vbs.size()));
    for (const VB& vb : vbs) b.u32(vb.count), b.u32(36), b.u32(0x22), b.raw(vb.data);
    b.u32(1);  // one index buffer (all batches)
    b.u32(uint32_t(indices.size())), b.u32(0);
    b.raw(indices.data(), indices.size() * 2);
    b.u32(0);  // 0x3c records
    b.u32(uint32_t(bats.size()));
    for (const Bat& bt : bats) {
        std::vector<uint8_t> h(0x30, 0);
        put32(h, 0x00, bt.vb), put32(h, 0x04, 0);
        const uint16_t prim = 4, vtype = 0, minV = uint16_t(bt.minV), stride = 36;
        std::memcpy(&h[0x10], &prim, 2), std::memcpy(&h[0x12], &vtype, 2), std::memcpy(&h[0x14], &minV, 2);
        put32(h, 0x18, bt.numV), put32(h, 0x1C, bt.start), put32(h, 0x20, bt.prims), put32(h, 0x24, 0), put32(h, 0x28, 0);
        std::memcpy(&h[0x2C], &stride, 2), std::memcpy(&h[0x2E], &bt.mat, 2);
        b.raw(h);
        b.u32(0);  // 0x1c records
    }
    b.u32(0), b.u32(0), b.u32(0);  // morph tree, pairs, groups
    b.u32(0);                      // morph targets
    b.u32(0);                      // collision vertices
    out.body = std::move(b.b);
    return out;
}

// Named spheres / boxes of a Mesh_Z link or a Lod_Z body.
void writePoints(Bytes& b, const Json* spheres, const Json* boxes) {
    b.u32(spheres ? uint32_t(spheres->arr.size()) : 0u);
    if (spheres)
        for (const Json& s : spheres->arr) {
            const auto c = s["c"].v3();
            b.f32(c[0]), b.f32(c[1]), b.f32(c[2]), b.f32(float(s["r"].num));
            b.u32(s.get("type") ? uint32_t(s["type"].num) : 0u), b.u32(nameCrc(s));
        }
    b.u32(boxes ? uint32_t(boxes->arr.size()) : 0u);
    if (boxes)
        for (const Json& x : boxes->arr) {
            std::array<float, 12> rows{1, 0, 0, 0, 0, 1, 0, 0, 0, 0, 1, 0};
            if (const Json* r = x.get("rows"))
                for (size_t k = 0; k < 12; ++k) rows[k] = r->f(k);
            else {
                const auto c = x["c"].v3();
                rows[3] = c[0], rows[7] = c[1], rows[11] = c[2];
            }
            for (float v : rows) b.f32(v);
            const auto h = x["half"].v3();
            b.f32(h[0]), b.f32(h[1]), b.f32(h[2]);
            b.f32(x.get("r") ? float(x["r"].num) : std::sqrt(h[0] * h[0] + h[1] * h[1] + h[2] * h[2]));
            b.u32(x.get("type") ? uint32_t(x["type"].num) : 0u), b.u32(nameCrc(x));
        }
}

// The 98-byte object part of a link (name, data, bounding sphere, the rest kept from the template).
std::vector<uint8_t> objectLink(std::vector<uint8_t> tmpl, uint32_t name, uint32_t data, const std::array<float, 4>& sphere) {
    tmpl.resize(98);
    put32(tmpl, 0, name), put32(tmpl, 4, data);
    for (int k = 0; k < 4; ++k) putf(tmpl, 8 + size_t(k) * 4, sphere[size_t(k)]);
    return tmpl;
}

int build(const std::string& data, const std::string& specDir, const std::string& outPath) {
    std::ifstream jf(specDir + "/pack.json");
    if (!jf) throw std::runtime_error("no " + specDir + "/pack.json");
    std::stringstream ss;
    ss << jf.rdbuf();
    const std::string text = ss.str();
    const Json P = JsonParser(text).parse();
    const std::string V = P["name"].str;
    Archive A;
    A.prefix = V;

    // ---- donor templates
    const std::string donorVariant = P["donor"]["variant"].str;
    DpcFile ddpc(data + "/" + P["donor"]["dpc"].str);
    Donor D(ddpc);
    VehicleAssembler dva(ddpc);
    const VehicleVariant* dv = dva.find(donorVariant);
    if (!dv) throw std::runtime_error("donor has no " + donorVariant);
    const std::vector<NodeGroup> dgroups = parseGameObjBody(D.body(dv->gameObjId));
    uint32_t dSkelLod = 0, dDynLod = 0, dSkelLodNode = 0, dDynLodNode = 0, dOmni = 0, dOmniNode = 0, dMeshNode = 0, dMesh = 0;
    for (const NodeGroup& g : dgroups)
        for (uint32_t n : g.nodes) {
            const Node nd = parseNodeBody(D.body(n));
            const uint32_t c = D.classOf(nd.object);
            const uint32_t nm = rd32(D.link(nd.object), 0);
            if (c == cls::Lod && nm == asoboCrc(donorVariant + "_SKEL")) dSkelLod = nd.object, dSkelLodNode = n;
            if (c == cls::Lod && nm == asoboCrc(donorVariant + "_DYN")) dDynLod = nd.object, dDynLodNode = n;
            if (c == cls::Omni && !dOmni) dOmni = nd.object, dOmniNode = n;
            if (c == cls::Mesh && !dMesh && (parseMeshLink(D.link(nd.object)).flags & 0x04000000) == 0) dMesh = nd.object, dMeshNode = n;
        }
    if (!dSkelLod || !dDynLod || !dOmni || !dMesh) throw std::runtime_error("donor variant lacks a _SKEL / _DYN Lod, an Omni_Z or a mesh");
    const Node dSkelNode = parseNodeBody(D.body(dSkelLodNode));
    const Lod dSkelLodBody = parseLodBody(D.body(dSkelLod));
    const uint32_t dSkin = dSkelLodBody.objects.at(0);
    const uint32_t dSkel = parseObjectLink(D.link(dSkin)).dataId;
    const Lod dDynLodBody = parseLodBody(D.body(dDynLod));
    const uint32_t dDynSkin = dDynLodBody.objects.at(0);
    const uint32_t dDynSkel = parseObjectLink(D.link(dDynSkin)).dataId;
    const uint32_t dLight = dSkelNode.light;
    const uint32_t dMaterial = D.dpc.ids(cls::Material).at(0), dBitmap = D.dpc.ids(cls::Bitmap).at(0);
    const uint32_t dMeshData = parseMeshLink(D.link(dMesh)).dataId;

    const uint32_t vName = asoboCrc(V);
    // Light context object (every node's +0x1c link): FUEL's, under a new id.
    const uint32_t lightId = A.id("LIGHTCTX");
    {
        std::vector<uint8_t> l = D.link(dLight);
        if (l.size() >= 4) put32(l, 0, lightId);
        A.add(className(D.classOf(dLight)).c_str(), lightId, l, D.body(dLight));
    }
    auto makeNode = [&](uint32_t tmplNode, const std::string& tag, uint32_t object, uint32_t objectData) {
        std::vector<uint8_t> b = D.body(tmplNode);
        for (size_t o : {0x0u, 0x4u, 0x8u, 0xcu}) put32(b, o, 0);
        put32(b, 0x10, object), put32(b, 0x14, objectData), put32(b, 0x18, 0), put32(b, 0x1c, lightId);
        const uint32_t id = A.id("NODE>" + tag);
        std::vector<uint8_t> l(4);
        put32(l, 0, vName);
        A.add("Node_Z", id, l, b);
        return id;
    };

    // ---- textures and materials
    std::map<std::string, uint32_t> texIds;
    const Json& texs = P["textures"];
    for (const auto& [key, t] : texs.obj) {
        const uint32_t id = A.id("TEX>" + key);
        auto [l, b] = makeBitmap(D.link(dBitmap), id, specDir + "/" + t["file"].str, int(t["alpha"].num));
        A.add("Bitmap_Z", id, l, b);
        texIds[key] = id;
    }
    std::vector<uint32_t> matIds;
    const std::vector<uint8_t> matTmpl = D.body(dMaterial);
    for (const Json& m : P["materials"].arr) {
        std::vector<uint8_t> b = matTmpl;
        const auto d = m["diffuse"].v3();
        putf(b, 0x00, d[0]), putf(b, 0x04, d[1]), putf(b, 0x08, d[2]), putf(b, 0x0c, float(m["alpha"].num));
        const auto e = m["emissive"].v3();
        putf(b, 0x10, e[0]), putf(b, 0x14, e[1]), putf(b, 0x18, e[2]);
        // identity UV transform
        putf(b, 0x20, 1), putf(b, 0x24, 0), putf(b, 0x30, 0), putf(b, 0x34, 1), putf(b, 0x40, 0), putf(b, 0x44, 0), putf(b, 0x4c, 0);
        putf(b, 0x70, 0), putf(b, 0x74, 0), putf(b, 0x78, 1), putf(b, 0x7c, 1);
        put32(b, 0x3c, uint32_t(m["flags"].num));
        for (int k = 0; k < 4; ++k) putf(b, 0x50 + size_t(k) * 4, m["specular"].f(size_t(k)));
        putf(b, 0x80, m["fresnel"].f(0)), putf(b, 0x84, m["fresnel"].f(1));
        uint16_t mask = 0;
        for (int k = 0; k < 9; ++k) {
            uint32_t tid = 0;
            if (const Json* t = m["textures"].get(std::to_string(k))) {
                auto it = texIds.find(t->str);
                if (it == texIds.end()) throw std::runtime_error("material texture " + t->str + " not in textures");
                tid = it->second;
                mask |= uint16_t(1u << k);
            }
            put32(b, 0x88 + size_t(k) * 4, tid);
        }
        if (m["fresnel"].f(0) != 0 || m["fresnel"].f(1) != 0) mask |= 0x100;
        std::memcpy(&b[0x38], &mask, 2);
        const uint32_t id = A.id("MAT>" + m["name"].str + ">" + std::to_string(matIds.size()));
        std::vector<uint8_t> l(4);
        put32(l, 0, asoboCrc(m["name"].str));  // the link name: lamp groups find LIGHT_* by it (0x69da00)
        A.add("Material_Z", id, l, b);
        matIds.push_back(id);
    }

    // ---- meshes: Mesh_Z + MeshData_Z + Node_Z per part, grouped by LOD
    std::map<int, std::vector<uint32_t>> groupNodes;  // LOD -> nodes
    const std::vector<uint8_t> meshLinkTmpl = D.link(dMesh);
    const std::vector<uint8_t> meshDataTmpl = D.body(dMeshData);
    for (const Json& part : P["parts"].arr) {
        const std::string pn = part["name"].str;
        const BuiltMesh bm = buildMesh(specDir + "/" + part["mesh"].str, matIds);
        const uint32_t meshId = A.id("MESH>" + pn), dataId = A.id("MESHDATA>" + pn);
        std::array<float, 4> sph{};
        for (int k = 0; k < 3; ++k) sph[size_t(k)] = (bm.lo[size_t(k)] + bm.hi[size_t(k)]) / 2;
        sph[3] = std::sqrt(std::pow(bm.hi[0] - sph[0], 2.0f) + std::pow(bm.hi[1] - sph[1], 2.0f) + std::pow(bm.hi[2] - sph[2], 2.0f));
        Bytes l;
        l.raw(objectLink(meshLinkTmpl, asoboCrc(pn), dataId, sph));
        put32(l.b, 92, 0x00dffc30);  // a static mesh's link flags (FUEL's, no morph)
        std::vector<uint32_t> deps = bm.materials;
        deps.push_back(dataId);
        l.u32(uint32_t(deps.size()));
        for (uint32_t x : deps) l.u32(x);
        // the 12 bytes after the dependency list, as the template has them
        {
            ByteReader r(meshLinkTmpl, 98);
            r.skipArray(4);
            const uint8_t* p = r.raw(12);
            l.raw(p, 12);
        }
        writePoints(l, part.get("spheres"), part.get("boxes"));
        A.add("Mesh_Z", meshId, l.b, bm.body);
        std::vector<uint8_t> dl(4);
        put32(dl, 0, asoboCrc(pn));
        A.add("MeshData_Z", dataId, dl, meshDataTmpl);
        groupNodes[int(part["lod"].num)].push_back(makeNode(dMeshNode, pn, meshId, dataId));
    }

    // ---- headlight Omni_Z (FUEL's headlight, renamed) and one node per LOD group, as FUEL's vehicles have it
    const uint32_t omniId = A.id("OMNI");
    {
        std::vector<uint8_t> l = D.link(dOmni);
        put32(l, 0, asoboCrc(P["omni"]["name"].str));
        A.add("Omni_Z", omniId, l, D.body(dOmni));
        for (const Json& k : P["omni"]["lods"].arr)
            groupNodes[int(k.num)].push_back(makeNode(dOmniNode, "OMNI" + std::to_string(int(k.num)), omniId, 0));
    }

    // ---- skeleton: FUEL's donor skeleton with this vehicle's positions and commands
    auto skelBody = D.body(dSkel);
    const Skel donorSkel = parseSkelBody(skelBody);
    // Map donor positions to this vehicle: per axis, the affine fit through the donor's and this vehicle's wheel hubs.
    std::array<float, 3> dLo{1e9f, 1e9f, 1e9f}, dHi{-1e9f, -1e9f, -1e9f}, nLo = dLo, nHi = dHi;
    const NameTable dnames(ddpc);
    for (const Bone& b : donorSkel.bones) {
        const std::string n = dnames(b.name);
        if (n == "LeftHand" || n == "RightHand" || n == "LeftFoot" || n == "RightFoot") {
            for (int a = 0; a < 3; ++a) dLo[size_t(a)] = std::min(dLo[size_t(a)], b.world[size_t(12 + a)]), dHi[size_t(a)] = std::max(dHi[size_t(a)], b.world[size_t(12 + a)]);
            const auto p = P["bones"][n]["pos"].v3();
            for (int a = 0; a < 3; ++a) nLo[size_t(a)] = std::min(nLo[size_t(a)], p[size_t(a)]), nHi[size_t(a)] = std::max(nHi[size_t(a)], p[size_t(a)]);
        }
    }
    auto remapPos = [&](std::array<float, 3> p) {
        for (int a = 0; a < 3; ++a) {
            const float span = dHi[size_t(a)] - dLo[size_t(a)];
            const float s = span > 1e-3f ? (nHi[size_t(a)] - nLo[size_t(a)]) / span : nHi[size_t(a)] / std::max(1e-3f, dHi[size_t(a)]);
            p[size_t(a)] = nLo[size_t(a)] + (p[size_t(a)] - dLo[size_t(a)]) * s;
        }
        return p;
    };
    // Bone records: 4 (userDefine) + 0xa0 (local TRS block) + 64 (world) + 4 x i32 + name = 248 bytes, after a 20-byte head.
    const Json& cmds = P["commands"];
    size_t o = 24;
    const std::array<float, 3> bmin = P["bounds"][0].v3(), bmax = P["bounds"][1].v3();
    {  // head: u16, u16, bounding sphere (centre, radius) of the vehicle
        putf(skelBody, 4, (bmin[0] + bmax[0]) / 2), putf(skelBody, 8, (bmin[1] + bmax[1]) / 2), putf(skelBody, 12, (bmin[2] + bmax[2]) / 2);
        putf(skelBody, 16, 0.5f * std::sqrt(std::pow(bmax[0] - bmin[0], 2.0f) + std::pow(bmax[1] - bmin[1], 2.0f) + std::pow(bmax[2] - bmin[2], 2.0f)));
    }
    const uint32_t skelName = asoboCrc(V + "_SKEL");
    for (size_t k = 0; k < donorSkel.bones.size(); ++k, o += 248) {
        const Bone& b = donorSkel.bones[k];
        const std::string n = k == 0 ? std::string("<root>") : dnames(b.name);
        std::array<float, 3> pos{b.world[12], b.world[13], b.world[14]};
        if (const Json* jb = P["bones"].get(n)) pos = (*jb)["pos"].v3();
        else if (k > 1) pos = remapPos(pos);
        putf(skelBody, o + 4 + 0x20, pos[0]), putf(skelBody, o + 4 + 0x24, pos[1]), putf(skelBody, o + 4 + 0x28, pos[2]);  // local T (parent at 0)
        putf(skelBody, o + 4 + 0xa0 + 48, pos[0]), putf(skelBody, o + 4 + 0xa0 + 52, pos[1]), putf(skelBody, o + 4 + 0xa0 + 56, pos[2]);  // world
        uint32_t ud = 0;
        if (const Json* c = cmds.get(n)) {
            std::string t;
            for (const Json& line : c->arr) t += line.str + "\n\r";
            Bytes u;
            u.u32(uint32_t(t.size()));
            u.raw(t.data(), t.size());
            ud = A.id("UD>" + std::to_string(k));
            std::vector<uint8_t> ul(4);
            put32(ul, 0, skelName);
            A.add("UserDefine_Z", ud, ul, u.b);
        }
        put32(skelBody, o, ud);
    }
    {  // tail: Material_Z list (FUEL lists one: the first material), MeshData_Z list, palettes, u32 list, 2 sphere lists, obbs
        ByteReader r(skelBody, o);
        const uint32_t nm = r.u32();
        for (uint32_t k = 0; k < nm; ++k) put32(skelBody, r.offset() + k * 4, matIds.at(0));
        r.raw(size_t(nm) * 4);
        r.skipArray(4);
        for (uint32_t k = 0, np = r.count(); k < np; ++k) r.skipArray(4);
        r.skipArray(4);
        const float scale = 0.5f * std::sqrt(std::pow(bmax[0] - bmin[0], 2.0f) + std::pow(bmax[1] - bmin[1], 2.0f) + std::pow(bmax[2] - bmin[2], 2.0f)) /
                            std::max(1e-3f, rd32(D.body(dSkel), 16) ? rdf(D.body(dSkel), 16) : 1.0f);
        for (int set = 0; set < 2; ++set)
            for (uint32_t k = 0, ns = r.count(); k < ns; ++k) {  // named spheres: centre, radius scaled to this vehicle
                const size_t e = r.offset();
                r.raw(28);
                putf(skelBody, e, (bmin[0] + bmax[0]) / 2 * (rdf(skelBody, e) != 0 ? 1.0f : 0.0f));
                putf(skelBody, e + 4, rdf(skelBody, e + 4) != 0 ? (bmin[1] + bmax[1]) / 2 : 0.0f);
                putf(skelBody, e + 12, rdf(skelBody, e + 12) * scale);
            }
        for (uint32_t k = 0, nb = r.count(); k < nb; ++k) {  // obbs (FUEL's DURL): this vehicle's DURL box
            const size_t e = r.offset();
            r.raw(76);
            for (const Json& bx : P["skel_lod"]["boxes"].arr)
                if (nameCrc(bx) == rd32(skelBody, e + 68)) {
                    const auto c = bx["c"].v3(), h = bx["half"].v3();
                    const float rows[12] = {1, 0, 0, c[0], 0, 1, 0, c[1], 0, 0, 1, c[2]};
                    for (int q = 0; q < 12; ++q) putf(skelBody, e + size_t(q) * 4, rows[q]);
                    putf(skelBody, e + 48, h[0]), putf(skelBody, e + 52, h[1]), putf(skelBody, e + 56, h[2]);
                    putf(skelBody, e + 60, std::sqrt(h[0] * h[0] + h[1] * h[1] + h[2] * h[2]));
                }
        }
        r.expectEnd();
    }
    const uint32_t skelId = A.id("SKEL");
    {
        std::vector<uint8_t> l(4);
        put32(l, 0, skelName);
        A.add("Skel_Z", skelId, l, skelBody);
    }
    // ---- the <V>_SKEL Lod_Z: the skeleton holder (one Skin_Z per LOD), its sphere, the DURL box, the LOD distances
    std::array<float, 4> vehSphere{(bmin[0] + bmax[0]) / 2, (bmin[1] + bmax[1]) / 2, (bmin[2] + bmax[2]) / 2,
                                   0.5f * std::sqrt(std::pow(bmax[0] - bmin[0], 2.0f) + std::pow(bmax[1] - bmin[1], 2.0f) + std::pow(bmax[2] - bmin[2], 2.0f))};
    auto makeSkin = [&](const std::string& tag, uint32_t skel, uint32_t name) {
        const uint32_t id = A.id("SKIN>" + tag);
        A.add("Skin_Z", id, objectLink(D.link(dSkin), name, skel, vehSphere), D.body(dSkin));
        return id;
    };
    const int lods = int(P["lods"].num);
    std::vector<uint32_t> skins;
    for (int k = 1; k <= lods; ++k) skins.push_back(makeSkin("LOD" + std::to_string(k), skelId, asoboCrc(V + "_SKIN_LOD" + std::to_string(k))));
    auto makeLod = [&](const std::string& tag, uint32_t donorLod, uint32_t donorNode, const std::string& name, const Json* spheres, const Json* boxes,
                       float D_, const std::vector<uint32_t>& objects) {
        Bytes b;
        writePoints(b, spheres, boxes);
        b.f32(0), b.f32(0), b.f32(D_);
        b.u32(uint32_t(objects.size()));
        for (uint32_t x : objects) b.u32(x);
        b.u32(0);
        b.u32(0), b.u32(0);  // no pair lists (the engine bank comes from PortVehicleSound)
        b.u32(0);            // no UserDefine_Z
        const uint32_t id = A.id("LOD>" + tag);
        const uint32_t dataId = A.id("LODDATA>" + tag);
        A.add("Lod_Z", id, objectLink(D.link(donorLod), asoboCrc(name), dataId, vehSphere), b.b);
        const Node dn = parseNodeBody(D.body(donorNode));
        std::vector<uint8_t> dl(4);
        put32(dl, 0, asoboCrc(name));
        A.add("LodData_Z", dataId, dl, D.body(dn.objectData));
        return makeNode(donorNode, "LOD>" + tag, id, dataId);
    };
    Json skelSpheres;
    skelSpheres.kind = Json::Arr;
    {  // FUEL's _SKEL sphere (name kept from the donor) around this vehicle
        Json s;
        s.kind = Json::Obj;
        Json c;
        c.kind = Json::Arr;
        for (int a = 0; a < 3; ++a) {
            Json v;
            v.kind = Json::Num, v.num = vehSphere[size_t(a)];
            c.arr.push_back(v);
        }
        Json r, t, n;
        r.kind = t.kind = Json::Num, r.num = vehSphere[3], t.num = dSkelLodBody.spheres.empty() ? 0 : dSkelLodBody.spheres[0].type;
        n.kind = Json::Str, n.str = hex8(dSkelLodBody.spheres.empty() ? 0 : dSkelLodBody.spheres[0].name);
        s.obj = {{"c", c}, {"r", r}, {"type", t}, {"crc", n}};
        skelSpheres.arr.push_back(s);
    }
    const uint32_t skelLodNode =
        makeLod("SKEL", dSkelLod, dSkelLodNode, V + "_SKEL", &skelSpheres, &P["skel_lod"]["boxes"], dSkelLodBody.baseDistance, skins);

    // ---- the <V>_DYN Lod_Z: the physics proxy (wheel spheres, chassis boxes) and its own one-bone skeleton
    std::vector<uint8_t> dynSkelBody = D.body(dDynSkel);
    {
        // head sphere, then the bones as they are, then the tail lists rewritten from the proxy
        putf(dynSkelBody, 4, vehSphere[0]), putf(dynSkelBody, 8, vehSphere[1]), putf(dynSkelBody, 12, vehSphere[2]), putf(dynSkelBody, 16, vehSphere[3]);
        ByteReader r(dynSkelBody, 20);
        const uint32_t nb = r.u32();
        r.raw(size_t(nb) * 248);
        const size_t tailAt = r.offset();
        const uint32_t rootName = rd32(dynSkelBody, 24 + 4 + 0xa0 + 64 + 16);
        Bytes t;
        t.u32(0), t.u32(0), t.u32(0), t.u32(0);  // materials, meshdata, palettes, u32 list
        const Json& sp = P["proxy"]["spheres"];
        t.u32(uint32_t(sp.arr.size()));
        for (const Json& s : sp.arr) {
            const auto c = s["c"].v3();
            t.f32(c[0]), t.f32(c[1]), t.f32(c[2]), t.f32(float(s["r"].num)), t.u32(0), t.u32(nameCrc(s)), t.u32(rootName);
        }
        t.u32(0);
        const Json& bx = P["proxy"]["boxes"];
        t.u32(uint32_t(bx.arr.size()));
        for (const Json& x : bx.arr) {
            const auto c = x["c"].v3(), h = x["half"].v3();
            const float rows[12] = {1, 0, 0, c[0], 0, 1, 0, c[1], 0, 0, 1, c[2]};
            for (float v : rows) t.f32(v);
            t.f32(h[0]), t.f32(h[1]), t.f32(h[2]), t.f32(std::sqrt(h[0] * h[0] + h[1] * h[1] + h[2] * h[2])), t.u32(0), t.u32(nameCrc(x)), t.u32(rootName);
        }
        dynSkelBody.resize(tailAt);
        dynSkelBody.insert(dynSkelBody.end(), t.b.begin(), t.b.end());
        parseSkelBody(dynSkelBody);  // must still read
    }
    const uint32_t dynSkelId = A.id("DYNSKEL");
    {
        std::vector<uint8_t> l(4);
        put32(l, 0, asoboCrc(V + "_DYN"));
        A.add("Skel_Z", dynSkelId, l, dynSkelBody);
    }
    const uint32_t dynSkin = makeSkin("DYN", dynSkelId, asoboCrc(V + "_DYN_SKIN"));
    const uint32_t dynLodNode = makeLod("DYN", dDynLod, dDynLodNode, V + "_DYN", &P["proxy"]["spheres"], &P["proxy"]["boxes"], 1.0f, {dynSkin});

    // ---- the garage carousel picture (optional): Bitmap_Z + Material_Z + MaterialAnim_Z named "VEH_<V>" as FUEL's
    // SHARED.DPC holds them (the menu finds MaterialAnim_Z by link name; its material sits 9 bytes before the body end)
    if (const Json* pic = P.get("picture")) {
        DpcFile shared(data + "/CommonComponent/DATAS/SHARED.DPC");
        uint32_t tmplAnim = 0;
        for (uint32_t id : shared.ids(asoboCrc("MaterialAnim_Z")))
            if (rd32(shared.link(*shared.find(id)), 0) == asoboCrc("VEH_" + donorVariant)) tmplAnim = id;
        if (!tmplAnim) throw std::runtime_error("no VEH_" + donorVariant + " picture in SHARED.DPC to model the picture on");
        std::vector<uint8_t> animBody = *shared.body(*shared.find(tmplAnim));
        const uint32_t tmplMat = rd32(animBody, animBody.size() - 9);
        const uint32_t bmpId = A.id("PICTURE>BITMAP"), matId = A.id("PICTURE>MATERIAL"), animId = A.id("PICTURE>ANIM");
        auto [bl, bb] = makeBitmap(D.link(dBitmap), bmpId, specDir + "/" + (*pic)["file"].str, 2);
        A.add("Bitmap_Z", bmpId, bl, bb);
        std::vector<uint8_t> mb = *shared.body(*shared.find(tmplMat));
        put32(mb, 0x88, bmpId);
        for (int k = 1; k < 9; ++k) put32(mb, 0x88 + size_t(k) * 4, 0);
        std::vector<uint8_t> ml = shared.link(*shared.find(tmplMat));
        if (ml.size() >= 4) put32(ml, 0, asoboCrc("VEH_" + V));
        A.add("Material_Z", matId, ml, mb);
        put32(animBody, animBody.size() - 9, matId);
        std::vector<uint8_t> al(4);
        put32(al, 0, asoboCrc("VEH_" + V));
        A.add("MaterialAnim_Z", animId, al, animBody);
    }

    // ---- GameObj_Z: <V>_COMMON (the two Lods), <V>_LOD1 .. <V>_LODn
    Bytes g;
    g.u32(uint32_t(1 + groupNodes.size()));
    auto group = [&](const std::string& name, const std::vector<uint32_t>& nodes) {
        g.u32(uint32_t(name.size() + 1));
        g.raw(name.c_str(), name.size() + 1);
        g.u32(1);
        g.u32(uint32_t(nodes.size()));
        for (uint32_t n : nodes) g.u32(n);
    };
    group(V + "_COMMON", {dynLodNode, skelLodNode});
    for (const auto& [lod, nodes] : groupNodes) group(V + "_LOD" + std::to_string(lod), nodes);
    {
        std::vector<uint8_t> l(4);
        put32(l, 0, vName);
        A.add("GameObj_Z", A.id("GAMEOBJ"), l, g.b);
    }
    A.write(outPath);
    std::printf("fuelvehpack: wrote %s: %zu resources\n", outPath.c_str(), A.objs.size());
    return 0;
}


// sounds DATA DPC VARIANT OUTDIR: the <V>_SKEL Lod's sound bank (second pair list: symbol, Sound_Z) written as raw
// signed 16-bit PCM (OUTDIR/<symbol hex>.s16, 44.1 kHz) for listening / analysis when picking a PortVehicleSound bank.
int dumpSounds(const std::string& data, const std::string& dpcRel, const std::string& variant, const std::string& out) {
    DpcFile dpc(!dpcRel.empty() && dpcRel[0] == '/' ? dpcRel : data + "/" + dpcRel);
    fs::create_directories(out);
    for (uint32_t id : dpc.ids(asoboCrc("Lod_Z"))) {
        if (rd32(dpc.link(*dpc.find(id)), 0) != asoboCrc(variant + "_SKEL")) continue;
        const std::vector<uint8_t> b = *dpc.body(*dpc.find(id));
        ByteReader r(b);
        r.skipArray(24), r.skipArray(72);
        r.raw(12);
        r.skipArray(4);
        r.u32();
        for (int i = 0; i < 2; ++i) {
            if (r.u32() == 0) continue;
            const uint32_t n = r.count();
            for (uint32_t k = 0; k < n; ++k) {
                const uint32_t sym = r.u32(), snd = r.u32();
                if (i != 1 || !dpc.find(snd)) continue;
                const auto pcm = dpc.body(*dpc.find(snd));
                const auto l = dpc.link(*dpc.find(snd));
                if (!pcm) continue;
                char name[32];
                std::snprintf(name, sizeof name, "/%02x.s16", sym);
                std::ofstream(out + name, std::ios::binary).write(reinterpret_cast<const char*>(pcm->data()), std::streamsize(pcm->size()));
                std::printf("symbol %02x sound %s %zu bytes rate %u flags %u\n", sym, hex8(snd).c_str(), pcm->size(), rd32(l, 4), l.size() > 12 ? l[12] : 0);
            }
        }
        return 0;
    }
    throw std::runtime_error("no " + variant + "_SKEL");
}

}  // namespace

int main(int argc, char** argv) {
    try {
        const std::string mode = argc > 1 ? argv[1] : "";
        if (mode == "inspect" && argc >= 5) {
            std::string dump;
            for (int a = 5; a + 1 < argc; ++a)
                if (std::string(argv[a]) == "--dump") dump = argv[a + 1];
            return inspect(argv[2], argv[3], argv[4], dump);
        }
        if (mode == "sounds" && argc == 6) return dumpSounds(argv[2], argv[3], argv[4], argv[5]);
        if (mode == "build" && argc == 5) return build(argv[2], argv[3], argv[4]);
        std::fprintf(stderr, "usage: fuelvehpack inspect DATA_DIR DPC VARIANT [--dump DIR]\n       fuelvehpack build DATA_DIR SPEC_DIR OUT.DPC\n");
        return 2;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "fuelvehpack: %s\n", e.what());
        return 1;
    }
}
