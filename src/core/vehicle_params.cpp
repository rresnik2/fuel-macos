#include "vehicle_params.hpp"

#include "resources.hpp"

#include <cstdio>
#include <cstring>
#include <sstream>
#include <stdexcept>

namespace fuel {

namespace {

constexpr uint32_t kLodClass = 0x73DC6A13;

class Cursor {
public:
    explicit Cursor(const std::vector<uint8_t>& b) : b_(b) {}
    const uint8_t* take(size_t n) {
        if (n > b_.size() - o_) throw std::runtime_error("Lod_Z body too short");
        const uint8_t* p = b_.data() + o_;
        o_ += n;
        return p;
    }
    uint32_t u32() {
        uint32_t v;
        std::memcpy(&v, take(4), 4);
        return v;
    }
    float f32(const uint8_t* p, int i) const {
        float v;
        std::memcpy(&v, p + i * 4, 4);
        return v;
    }

private:
    const std::vector<uint8_t>& b_;
    size_t o_ = 0;
};

}  // namespace

std::map<std::string, std::vector<float>> parseVehiclesParam(const std::string& text) {
    std::map<std::string, std::vector<float>> out;
    std::istringstream in(text);
    std::string line, current;
    while (std::getline(in, line)) {
        char name[128];
        int index;
        float value;
        if (std::sscanf(line.c_str(), " VehiclePP 0 \"MESH_ID_%127[^\"]\"", name) == 1) {
            current = name;
            out[current].assign(1, 0.0f);
        } else if (!current.empty() && std::sscanf(line.c_str(), " VehiclePP %d %f", &index, &value) == 2 && index > 0 &&
                   index < 4096) {
            std::vector<float>& v = out[current];
            if (v.size() <= size_t(index)) v.resize(size_t(index) + 1, 0.0f);
            v[size_t(index)] = value;
        } else if (line.find("EndOfVehiclePP") != std::string::npos) {
            current.clear();
        }
    }
    return out;
}

VehicleProxy findVehicleProxy(const DpcFile& dpc, const std::string& variant) {
    const uint32_t want = asoboCrc(variant + "_DYN");
    for (uint32_t id : dpc.ids(kLodClass)) {
        const DpcFile::Entry* e = dpc.find(id);
        const std::vector<uint8_t> link = dpc.link(*e);
        uint32_t name = 0;
        if (link.size() >= 4) std::memcpy(&name, link.data(), 4);
        if (name != want) continue;
        const auto body = dpc.body(*e);
        if (!body) break;
        // Lod_Z body: u32 m; m x {f32 centre[3], f32 radius, u32 type, u32 name};
        //             u32 k; k x {f32 m[3][4] (translation in column 3), f32 half[3], f32 r, u32 type, u32 name}.
        Cursor c(*body);
        VehicleProxy p;
        const uint32_t m = c.u32();
        for (uint32_t i = 0; i < m; ++i) {
            const uint8_t* s = c.take(24);
            ProxySphere sp;
            sp.centre = {c.f32(s, 0), c.f32(s, 1), c.f32(s, 2)};
            sp.radius = c.f32(s, 3);
            std::memcpy(&sp.name, s + 20, 4);
            p.spheres.push_back(sp);
        }
        const uint32_t k = c.u32();
        for (uint32_t i = 0; i < k; ++i) {
            const uint8_t* b = c.take(72);
            ProxyBox bx;
            bx.centre = {c.f32(b, 3), c.f32(b, 7), c.f32(b, 11)};
            bx.halfExtents = {c.f32(b, 12), c.f32(b, 13), c.f32(b, 14)};
            p.boxes.push_back(bx);
        }
        return p;
    }
    throw std::runtime_error("no physics proxy " + variant + "_DYN in " + dpc.path());
}

}  // namespace fuel
