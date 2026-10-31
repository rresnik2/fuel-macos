#include "rider_tables.hpp"

#include <cstdio>
#include <fstream>
#include <iterator>
#include <map>
#include <mutex>
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace fuel {

namespace {

// Code addresses in FUEL.exe (not data).
constexpr uint32_t kSlotTable = 0x5d6950, kActionProps = 0x5d3660;
constexpr uint32_t kCodeLo = 0x5d3660, kCodeHi = 0x5d79a0;
constexpr uint32_t kVeh = 0x10000000;  // fake vehicle object
constexpr uint32_t kSlotBase = 0x518, kPropBase = 0x270;

struct Image {
    std::vector<uint8_t> file;
    struct Section {
        uint32_t va, size, raw;
    };
    std::vector<Section> sections;
    bool load(const std::string& path) {
        std::ifstream f(path, std::ios::binary);
        if (!f) return false;
        file.assign(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
        auto u16 = [&](size_t o) { return o + 2 <= file.size() ? uint32_t(file[o] | file[o + 1] << 8) : 0u; };
        auto u32 = [&](size_t o) {
            return o + 4 <= file.size() ? uint32_t(file[o] | file[o + 1] << 8 | file[o + 2] << 16 | uint32_t(file[o + 3]) << 24) : 0u;
        };
        const size_t pe = u32(0x3c);
        if (u32(pe) != 0x00004550) return false;
        const uint32_t n = u16(pe + 6), opt = u16(pe + 20), base = u32(pe + 24 + 28);
        for (uint32_t k = 0; k < n; ++k) {
            const size_t sh = pe + 24 + opt + size_t(k) * 40;
            sections.push_back({base + u32(sh + 12), u32(sh + 16), u32(sh + 20)});
        }
        return true;
    }
    uint8_t byte(uint32_t va) const {
        for (const Section& s : sections)
            if (va >= s.va && va < s.va + s.size && size_t(s.raw) + (va - s.va) < file.size()) return file[s.raw + (va - s.va)];
        throw std::runtime_error("address outside the image");
    }
    uint32_t u32(uint32_t va) const { return byte(va) | byte(va + 1) << 8 | byte(va + 2) << 16 | uint32_t(byte(va + 3)) << 24; }
};

// Straight-line integer stores with a jump table: just enough x86 for 0x5d6950 / 0x5d3660.
class Cpu {
public:
    explicit Cpu(const Image& img) : img_(img) {}
    uint32_t r[8] = {};  // eax ecx edx ebx esp ebp esi edi
    std::map<uint32_t, uint32_t> mem;  // dword stores (aligned to the vehicle fields)
    bool zf = false, cf = false;

    uint32_t rd(uint32_t a) const {
        auto it = mem.find(a);
        return it != mem.end() ? it->second : 0;
    }
    void run(uint32_t entry) {
        constexpr uint32_t kRet = 0x0c000000;
        r[4] = 0x0ff00000;
        push(kRet);
        ip_ = entry;
        for (long steps = 0; ip_ != kRet; ++steps) {
            if (steps > 100000) throw std::runtime_error("replay does not end");
            if (ip_ < kCodeLo || ip_ >= kCodeHi) fail("jump outside the rider code");
            step();
        }
    }
    void push(uint32_t v) { r[4] -= 4, mem[r[4]] = v; }
    uint32_t pop() {
        const uint32_t v = rd(r[4]);
        r[4] += 4;
        return v;
    }

private:
    const Image& img_;
    uint32_t ip_ = 0;
    [[noreturn]] void fail(const char* what) const {
        char b[128];
        std::snprintf(b, sizeof b, "unsupported code at 0x%x (%s)", ip_, what);
        throw std::runtime_error(b);
    }
    uint8_t fetch() { return img_.byte(ip_++); }
    uint32_t fetch32() {
        const uint32_t v = img_.u32(ip_);
        ip_ += 4;
        return v;
    }
    // ModRM memory operand: returns the address (register forms are handled by the callers).
    uint32_t address(uint8_t modrm) {
        const int mod = modrm >> 6, rm = modrm & 7;
        uint32_t base;
        if (rm == 4) {
            const uint8_t sib = fetch();
            const int scale = 1 << (sib >> 6), index = (sib >> 3) & 7, b = sib & 7;
            if (b == 5 && mod == 0) base = fetch32();
            else base = r[b];
            if (index != 4) base += r[index] * uint32_t(scale);
        } else if (rm == 5 && mod == 0) {
            return fetch32();
        } else {
            base = r[rm];
        }
        if (mod == 1) base += uint32_t(int32_t(int8_t(fetch())));
        else if (mod == 2) base += fetch32();
        return base;
    }
    void arith(int op, uint32_t& dst, uint32_t src) {
        switch (op) {
            case 0: dst += src, zf = dst == 0, cf = dst < src; break;           // add
            case 5: cf = dst < src, dst -= src, zf = dst == 0; break;           // sub
            case 6: dst ^= src, zf = dst == 0, cf = false; break;               // xor
            case 7: cf = dst < src, zf = dst == src; break;                     // cmp
            default: fail("arith op");
        }
    }
    void jump(bool take, int32_t rel) {
        if (take) ip_ += uint32_t(rel);
    }
    void step() {
        const uint8_t op = fetch();
        if (op >= 0xb8 && op <= 0xbf) { r[op - 0xb8] = fetch32(); return; }
        if (op >= 0x50 && op <= 0x57) { push(r[op - 0x50]); return; }
        if (op >= 0x58 && op <= 0x5f) { r[op - 0x58] = pop(); return; }
        switch (op) {
            case 0xc7: {  // mov r/m32, imm32
                const uint8_t m = fetch();
                if ((m >> 6) == 3) { r[m & 7] = fetch32(); return; }
                const uint32_t a = address(m);
                mem[a] = fetch32();
                return;
            }
            case 0x89: case 0x8b: {  // mov r/m32, r32 / mov r32, r/m32
                const uint8_t m = fetch();
                const int reg = (m >> 3) & 7;
                if ((m >> 6) == 3) {
                    if (op == 0x89) r[m & 7] = r[reg];
                    else r[reg] = r[m & 7];
                    return;
                }
                const uint32_t a = address(m);
                if (op == 0x89) mem[a] = r[reg];
                else r[reg] = rd(a);
                return;
            }
            case 0x8d: {  // lea
                const uint8_t m = fetch();
                if ((m >> 6) == 3) fail("lea reg");
                r[(m >> 3) & 7] = address(m);
                return;
            }
            case 0x31: case 0x33: case 0x29: case 0x2b: case 0x39: case 0x3b: case 0x01: case 0x03: {
                const uint8_t m = fetch();
                if ((m >> 6) != 3) fail("alu mem");
                const int reg = (m >> 3) & 7, rm = m & 7;
                const int kind = (op & 0xf8) == 0x30 ? 6 : (op & 0xf8) == 0x28 ? 5 : (op & 0xf8) == 0x38 ? 7 : 0;
                const bool toReg = op & 2;
                uint32_t& dst = toReg ? r[reg] : r[rm];
                arith(kind, dst, toReg ? r[rm] : r[reg]);
                return;
            }
            case 0x83: case 0x81: {  // group 1 with imm8 / imm32
                const uint8_t m = fetch();
                const int kind = (m >> 3) & 7;
                if ((m >> 6) == 3) {
                    const uint32_t imm = op == 0x83 ? uint32_t(int32_t(int8_t(fetch()))) : fetch32();
                    arith(kind, r[m & 7], imm);
                } else {
                    const uint32_t a = address(m);
                    const uint32_t imm = op == 0x83 ? uint32_t(int32_t(int8_t(fetch()))) : fetch32();
                    uint32_t v = rd(a);
                    arith(kind, v, imm);
                    if (kind != 7) mem[a] = v;
                }
                return;
            }
            case 0x74: jump(zf, int8_t(fetch())); return;
            case 0x75: jump(!zf, int8_t(fetch())); return;
            case 0x77: jump(!cf && !zf, int8_t(fetch())); return;
            case 0x76: jump(cf || zf, int8_t(fetch())); return;
            case 0x72: jump(cf, int8_t(fetch())); return;
            case 0x73: jump(!cf, int8_t(fetch())); return;
            case 0xeb: jump(true, int8_t(fetch())); return;
            case 0xe9: jump(true, int32_t(fetch32())); return;
            case 0x0f: {
                const uint8_t o2 = fetch();
                const int32_t rel = int32_t(fetch32());
                switch (o2) {
                    case 0x84: jump(zf, rel); return;
                    case 0x85: jump(!zf, rel); return;
                    case 0x87: jump(!cf && !zf, rel); return;
                    case 0x86: jump(cf || zf, rel); return;
                    default: fail("0f jcc");
                }
            }
            case 0xe8: {  // call rel32 (inside the rider code only)
                const int32_t rel = int32_t(fetch32());
                push(ip_);
                ip_ += uint32_t(rel);
                return;
            }
            case 0xff: {  // jmp [mem] (the archetype jump table)
                const uint8_t m = fetch();
                if (((m >> 3) & 7) != 4 || (m >> 6) == 3) fail("ff");
                ip_ = img_.u32(address(m));
                return;
            }
            case 0xc3: ip_ = pop(); return;
            case 0xc2: {
                const uint16_t n = uint16_t(fetch() | fetch() << 8);
                ip_ = pop();
                r[4] += n;
                return;
            }
            default: fail("opcode");
        }
    }
};

}  // namespace

int riderArchetypeIndex(const std::string& n) {
    static const char* const kNames[11] = {"BUGGY", "BUGGY_ROAD", "MBIKE", "MBIKE_SPORT", "MBIKE_CHOPPER", "MBIKE_TRAIL",
                                           "MUSCLE", "QUAD", "QUAD_TRAIL", "SUV", "TRUCK"};
    for (int i = 0; i < 11; ++i)
        if (n == kNames[i]) return i;
    return -1;
}

std::optional<RiderTables> loadRiderTables(const std::string& exePath, std::string* error) {
    static std::mutex mu;
    static std::unordered_map<std::string, std::pair<std::optional<RiderTables>, std::string>> cache;
    std::lock_guard<std::mutex> lk(mu);
    auto it = cache.find(exePath);
    if (it == cache.end()) {
        std::pair<std::optional<RiderTables>, std::string> res;
        try {
            Image img;
            if (!img.load(exePath)) throw std::runtime_error("cannot read " + exePath);
            RiderTables t;
            for (int a = 0; a < 11; ++a) {
                Cpu cpu(img);
                cpu.r[1] = uint32_t(a);  // ECX = archetype
                cpu.r[0] = kVeh;         // EAX = vehicle
                cpu.run(kSlotTable);
                for (int k = 0; k < kRiderActions; ++k) {
                    const uint32_t addr = kVeh + kSlotBase + 4 * uint32_t(k);
                    if (!cpu.mem.count(addr)) throw std::runtime_error("archetype " + std::to_string(a) + ": action " + std::to_string(k) + " has no slot");
                    const uint32_t s = cpu.rd(addr);
                    if (s > 400) throw std::runtime_error("implausible slot");
                    t.slots[size_t(a)][size_t(k)] = int(s);
                }
            }
            Cpu cpu(img);
            cpu.r[0] = kVeh;
            cpu.run(kActionProps);
            for (int k = 0; k < kRiderActions; ++k) {
                const uint32_t b = kVeh + kPropBase + 12 * uint32_t(k);
                t.props[size_t(k)] = {int(cpu.rd(b)), int(cpu.rd(b + 4)), int(cpu.rd(b + 8))};
                if (t.props[size_t(k)].priority > 20 || t.props[size_t(k)].mode > 3) throw std::runtime_error("implausible action properties");
            }
            res.first = t;
        } catch (const std::exception& e) {
            res.second = std::string("rider tables from FUEL.exe: ") + e.what();
        }
        it = cache.emplace(exePath, std::move(res)).first;
    }
    if (error) *error = it->second.second;
    return it->second.first;
}

}  // namespace fuel
