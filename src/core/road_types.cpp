#include "road_types.hpp"
#include "genworld_noise.hpp"

#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iterator>
#include <limits>
#include <mutex>
#include <stdexcept>
#include <unordered_map>
#include <vector>

namespace fuel {

namespace {

// Addresses in FUEL.exe (code locations, not data).
constexpr uint32_t kInit = 0x6f24a0;        // AGW_TerraRoad::Init
constexpr uint32_t kTable = 0xa677e0;       // per-type records, stride 0x200
constexpr uint32_t kStride = 0x200;
constexpr uint32_t kCodeLo = 0x6f1240, kCodeHi = 0x6f5940;  // Init and the helpers it calls (0x6f1240, 0x6f5690, 0x6f5770)
// External calls the replay skips, with the bytes of arguments each callee pops.
struct Stub {
    uint32_t va, pops;
};
constexpr Stub kStubs[] = {{0x8b6e86, 0}, {0x6fee30, 0}, {0x6597b0, 8}, {0x7a9c80, 12}};

// The PE image of FUEL.exe, readable by virtual address.
struct Image {
    std::vector<uint8_t> file;
    struct Section {
        uint32_t va, size, raw;
    };
    std::vector<Section> sections;
    uint32_t base = 0;
    bool load(const std::string& path) {
        std::ifstream f(path, std::ios::binary);
        if (!f) return false;
        file.assign(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
        auto u16 = [&](size_t o) { return o + 2 <= file.size() ? uint32_t(file[o] | file[o + 1] << 8) : 0u; };
        auto u32 = [&](size_t o) { return o + 4 <= file.size() ? uint32_t(file[o] | file[o + 1] << 8 | file[o + 2] << 16 | uint32_t(file[o + 3]) << 24) : 0u; };
        const size_t pe = u32(0x3c);
        if (u32(pe) != 0x00004550) return false;
        const uint32_t n = u16(pe + 6), opt = u16(pe + 20);
        base = u32(pe + 24 + 28);
        for (uint32_t k = 0; k < n; ++k) {
            const size_t sh = pe + 24 + opt + size_t(k) * 40;
            sections.push_back({base + u32(sh + 12), u32(sh + 16), u32(sh + 20)});
        }
        return true;
    }
    bool byte(uint32_t va, uint8_t& out) const {
        for (const Section& s : sections)
            if (va >= s.va && va < s.va + s.size && size_t(s.raw) + (va - s.va) < file.size()) {
                out = file[s.raw + (va - s.va)];
                return true;
            }
        return false;
    }
};

double floatDown(double v) {  // float32 store in the engine's round-down FPU mode
    float f = float(v);
    if (double(f) > v) f = std::nextafter(f, -std::numeric_limits<float>::infinity());
    return double(f);
}

// A minimal x86-32 / x87 interpreter for the straight-line table set-up code.
class Cpu {
public:
    explicit Cpu(const Image& img) : img_(img) {}
    std::unordered_map<uint32_t, uint8_t> mem;  // everything written (overrides the image)
    uint32_t r[8] = {};                          // eax ecx edx ebx esp ebp esi edi
    double st[8] = {};
    int top = 0;
    uint16_t sw = 0;
    bool cf = false, zf = false, sf = false, of = false, pf = false;
    uint32_t ip = 0;
    long steps = 0;

    uint8_t rd8(uint32_t a) const {
        if (auto it = mem.find(a); it != mem.end()) return it->second;
        uint8_t b = 0;
        img_.byte(a, b);
        return b;
    }
    uint32_t rd32(uint32_t a) const { return uint32_t(rd8(a)) | uint32_t(rd8(a + 1)) << 8 | uint32_t(rd8(a + 2)) << 16 | uint32_t(rd8(a + 3)) << 24; }
    uint16_t rd16(uint32_t a) const { return uint16_t(rd8(a) | rd8(a + 1) << 8); }
    void wr8(uint32_t a, uint8_t v) { mem[a] = v; }
    void wr16(uint32_t a, uint16_t v) { wr8(a, uint8_t(v)), wr8(a + 1, uint8_t(v >> 8)); }
    void wr32(uint32_t a, uint32_t v) {
        for (int k = 0; k < 4; ++k) wr8(a + uint32_t(k), uint8_t(v >> (8 * k)));
    }
    float rdf(uint32_t a) const {
        const uint32_t u = rd32(a);
        float f;
        std::memcpy(&f, &u, 4);
        return f;
    }
    void wrf(uint32_t a, double v) {
        const float f = float(floatDown(v));
        uint32_t u;
        std::memcpy(&u, &f, 4);
        wr32(a, u);
    }
    void push(uint32_t v) { r[4] -= 4, wr32(r[4], v); }
    uint32_t pop() {
        const uint32_t v = rd32(r[4]);
        r[4] += 4;
        return v;
    }
    double& ST(int i) { return st[(top + i) & 7]; }
    void fpush(double v) { top = (top - 1) & 7, st[top] = v; }
    void fpop() { top = (top + 1) & 7; }

    [[noreturn]] void fail(const char* what) const {
        char b[160];
        std::snprintf(b, sizeof b, "unsupported instruction at 0x%x (%s, byte %02x)", ip, what, rd8(ip));
        throw std::runtime_error(b);
    }

    // Runs from `entry` until it returns to `sentinel`.
    void run(uint32_t entry, uint32_t sentinel) {
        ip = entry;
        while (ip != sentinel) {
            if (++steps > 2000000) throw std::runtime_error("road table replay does not end");
            if (ip < kCodeLo || ip >= kCodeHi) {
                const Stub* s = nullptr;
                for (const Stub& k : kStubs)
                    if (k.va == ip) s = &k;
                if (!s && ip != 0) fail("call outside the table code");
                ip = pop();
                r[4] += s ? s->pops : 0;
                r[0] = 0;
                continue;
            }
            step();
        }
    }

private:
    const Image& img_;

    struct ModRM {
        int mod, reg, rm;
        uint32_t addr;
    };
    ModRM modrm(uint32_t& p) {
        const uint8_t m = rd8(p++);
        ModRM o{m >> 6, (m >> 3) & 7, m & 7, 0};
        if (o.mod == 3) return o;
        uint32_t a = 0;
        if (o.rm == 4) {
            const uint8_t sib = rd8(p++);
            const int ss = sib >> 6, idx = (sib >> 3) & 7, b = sib & 7;
            if (b == 5 && o.mod == 0) a = rd32(p), p += 4;
            else a = r[b];
            if (idx != 4) a += r[idx] << ss;
        } else if (o.rm == 5 && o.mod == 0) {
            a = rd32(p), p += 4;
        } else {
            a = r[o.rm];
        }
        if (o.mod == 1) a += uint32_t(int32_t(int8_t(rd8(p++))));
        if (o.mod == 2) a += rd32(p), p += 4;
        o.addr = a;
        return o;
    }
    uint32_t get32(const ModRM& m) const { return m.mod == 3 ? r[m.rm] : rd32(m.addr); }
    void set32(const ModRM& m, uint32_t v) {
        if (m.mod == 3) r[m.rm] = v;
        else wr32(m.addr, v);
    }
    uint8_t reg8(int i) const { return i < 4 ? uint8_t(r[i]) : uint8_t(r[i - 4] >> 8); }
    void setReg8(int i, uint8_t v) {
        if (i < 4) r[i] = (r[i] & ~0xFFu) | v;
        else r[i - 4] = (r[i - 4] & ~0xFF00u) | uint32_t(v) << 8;
    }
    uint8_t get8(const ModRM& m) const { return m.mod == 3 ? reg8(m.rm) : rd8(m.addr); }
    void set8(const ModRM& m, uint8_t v) {
        if (m.mod == 3) setReg8(m.rm, v);
        else wr8(m.addr, v);
    }
    void logic(uint32_t v, int bits) {
        const uint32_t mask = bits == 32 ? 0xFFFFFFFFu : 0xFFu;
        v &= mask;
        cf = of = false;
        zf = v == 0;
        sf = (v >> (bits - 1)) & 1;
        pf = !(__builtin_popcount(v & 0xFF) & 1);
    }
    uint32_t alu(int op, uint32_t a, uint32_t b, int bits) {
        const uint64_t mask = bits == 32 ? 0xFFFFFFFFull : 0xFFull;
        uint64_t res = 0;
        switch (op) {
            case 0: res = (uint64_t(a) + b); break;           // add
            case 1: logic(a | b, bits); return (a | b) & uint32_t(mask);
            case 4: logic(a & b, bits); return (a & b) & uint32_t(mask);
            case 6: logic(a ^ b, bits); return (a ^ b) & uint32_t(mask);
            case 5:
            case 7: res = (uint64_t(a) - b); break;           // sub, cmp
            default: fail("alu op");
        }
        const uint32_t v = uint32_t(res & mask);
        cf = op == 0 ? (res >> bits) & 1 : (uint64_t(a) & mask) < (uint64_t(b) & mask);
        zf = v == 0;
        sf = (v >> (bits - 1)) & 1;
        const uint32_t sa = (a >> (bits - 1)) & 1, sb = (b >> (bits - 1)) & 1, sr = (v >> (bits - 1)) & 1;
        of = op == 0 ? (sa == sb && sr != sa) : (sa != sb && sr != sa);
        pf = !(__builtin_popcount(v & 0xFF) & 1);
        return v;
    }
    bool cond(int c) const {
        bool v = false;
        switch (c >> 1) {
            case 0: v = of; break;
            case 1: v = cf; break;
            case 2: v = zf; break;
            case 3: v = cf || zf; break;
            case 4: v = sf; break;
            case 5: v = pf; break;
            case 6: v = sf != of; break;
            case 7: v = zf || sf != of; break;
        }
        return (c & 1) ? !v : v;
    }
    void fcompare(double a, double b) {
        sw &= ~0x4500;
        if (std::isnan(a) || std::isnan(b)) sw |= 0x4500;
        else if (a < b) sw |= 0x0100;
        else if (a == b) sw |= 0x4000;
    }
    // st(dst) = st(dst) op src for x87 /reg codes: 0 add 1 mul 4 sub 5 subr 6 div 7 divr.
    static double farith(int op, double d, double s) {
        switch (op) {
            case 0: return d + s;
            case 1: return d * s;
            case 4: return d - s;
            case 5: return s - d;
            case 6: return d / s;
            case 7: return s / d;
        }
        return d;
    }

    void step() {
        uint32_t p = ip;
        uint8_t op = rd8(p++);
        bool o16 = false, rep = false;
        if (op == 0x66) o16 = true, op = rd8(p++);
        if (op == 0xF3) rep = true, op = rd8(p++);
        auto imm8s = [&] { return uint32_t(int32_t(int8_t(rd8(p++)))); };
        auto imm32 = [&] {
            const uint32_t v = rd32(p);
            p += 4;
            return v;
        };
        if (op <= 0x3B && (op & 7) <= 3) {  // ALU r/m, r and r, r/m
            const int kind = op >> 3, dir = op & 2, wide = op & 1;
            ModRM m = modrm(p);
            if (wide) {
                const uint32_t a = dir ? r[m.reg] : get32(m), b = dir ? get32(m) : r[m.reg];
                const uint32_t v = alu(kind, a, b, 32);
                if (kind != 7) {
                    if (dir) r[m.reg] = v;
                    else set32(m, v);
                }
            } else {
                const uint8_t a = dir ? reg8(m.reg) : get8(m), b = dir ? get8(m) : reg8(m.reg);
                const uint32_t v = alu(kind, a, b, 8);
                if (kind != 7) {
                    if (dir) setReg8(m.reg, uint8_t(v));
                    else set8(m, uint8_t(v));
                }
            }
        } else if (op <= 0x3D && (op & 7) == 5) {  // ALU eax, imm32
            const int kind = op >> 3;
            const uint32_t v = alu(kind, r[0], imm32(), 32);
            if (kind != 7) r[0] = v;
        } else if (op >= 0x40 && op <= 0x47) {
            r[op - 0x40] = alu(0, r[op - 0x40], 1, 32);
        } else if (op >= 0x48 && op <= 0x4F) {
            r[op - 0x48] = alu(5, r[op - 0x48], 1, 32);
        } else if (op >= 0x50 && op <= 0x57) {
            push(r[op - 0x50]);
        } else if (op >= 0x58 && op <= 0x5F) {
            r[op - 0x58] = pop();
        } else if (op == 0x68) {
            push(imm32());
        } else if (op == 0x6A) {
            push(imm8s());
        } else if (op >= 0x70 && op <= 0x7F) {
            const uint32_t t = imm8s();
            if (cond(op - 0x70)) p += t;
        } else if (op == 0x0F) {
            const uint8_t o2 = rd8(p++);
            if (o2 >= 0x80 && o2 <= 0x8F) {
                const uint32_t t = imm32();
                if (cond(o2 - 0x80)) p += t;
            } else if (o2 == 0xB6) {
                ModRM m = modrm(p);
                r[m.reg] = get8(m);
            } else if (o2 == 0xB7) {
                ModRM m = modrm(p);
                r[m.reg] = m.mod == 3 ? (r[m.rm] & 0xFFFF) : rd16(m.addr);
            } else {
                fail("0f");
            }
        } else if (op == 0x80 || op == 0x81 || op == 0x83) {
            ModRM m = modrm(p);
            if (op == 0x80) {
                const uint8_t v = uint8_t(alu(m.reg, get8(m), rd8(p++), 8));
                if (m.reg != 7) set8(m, v);
            } else {
                const uint32_t b = op == 0x81 ? imm32() : imm8s();
                const uint32_t v = alu(m.reg, get32(m), b, 32);
                if (m.reg != 7) set32(m, v);
            }
        } else if (op == 0x84) {
            ModRM m = modrm(p);
            logic(uint32_t(get8(m) & reg8(m.reg)), 8);
        } else if (op == 0x85) {
            ModRM m = modrm(p);
            logic(get32(m) & r[m.reg], 32);
        } else if (op == 0xF6 && (rd8(p) & 0x38) == 0) {
            ModRM m = modrm(p);
            logic(uint32_t(get8(m) & rd8(p++)), 8);
        } else if (op == 0x88) {
            ModRM m = modrm(p);
            set8(m, reg8(m.reg));
        } else if (op == 0x89) {
            ModRM m = modrm(p);
            if (o16) {
                if (m.mod == 3) r[m.rm] = (r[m.rm] & ~0xFFFFu) | (r[m.reg] & 0xFFFF);
                else wr16(m.addr, uint16_t(r[m.reg]));
            } else {
                set32(m, r[m.reg]);
            }
        } else if (op == 0x8A) {
            ModRM m = modrm(p);
            setReg8(m.reg, get8(m));
        } else if (op == 0x8B) {
            ModRM m = modrm(p);
            r[m.reg] = get32(m);
        } else if (op == 0x8D) {
            ModRM m = modrm(p);
            if (m.mod == 3) fail("lea reg");
            r[m.reg] = m.addr;
        } else if (op == 0x90) {
        } else if (op == 0xA1) {
            r[0] = rd32(imm32());
        } else if (op == 0xA3) {
            wr32(imm32(), r[0]);
        } else if (op == 0xA5 && rep) {  // rep movsd
            for (; r[1]; --r[1]) wr32(r[7], rd32(r[6])), r[6] += 4, r[7] += 4;
        } else if (op >= 0xB0 && op <= 0xB7) {
            setReg8(op - 0xB0, rd8(p++));
        } else if (op >= 0xB8 && op <= 0xBF) {
            r[op - 0xB8] = imm32();
        } else if (op == 0xC2) {
            const uint16_t n = rd16(p);
            p = pop();
            r[4] += n;
        } else if (op == 0xC3) {
            p = pop();
        } else if (op == 0xC6) {
            ModRM m = modrm(p);
            set8(m, rd8(p++));
        } else if (op == 0xC7) {
            ModRM m = modrm(p);
            if (o16) {
                const uint16_t v = rd16(p);
                p += 2;
                if (m.mod == 3) r[m.rm] = (r[m.rm] & ~0xFFFFu) | v;
                else wr16(m.addr, v);
            } else {
                set32(m, imm32());
            }
        } else if (op == 0xE8) {
            const uint32_t t = imm32();
            push(p);
            p += t;
        } else if (op == 0xE9) {
            const uint32_t t = imm32();
            p += t;
        } else if (op == 0xEB) {
            const uint32_t t = imm8s();
            p += t;
        } else if (op == 0xFF) {
            ModRM m = modrm(p);
            if (m.reg == 2) {
                const uint32_t t = get32(m);
                push(p);
                p = t;
            } else if (m.reg == 6) {
                push(get32(m));
            } else {
                fail("ff");
            }
        } else if (op >= 0xD8 && op <= 0xDF) {
            p = fpu(op, p);
        } else {
            fail("opcode");
        }
        ip = p;
    }

    uint32_t fpu(uint8_t op, uint32_t p) {
        const uint8_t m0 = rd8(p);
        if (m0 < 0xC0) {
            ModRM m = modrm(p);
            const int g = m.reg;
            switch (op) {
                case 0xD8:  // arith st0, m32
                    if (g == 2 || g == 3) {
                        fcompare(ST(0), rdf(m.addr));
                        if (g == 3) fpop();
                    } else {
                        ST(0) = farith(g, ST(0), rdf(m.addr));
                    }
                    return p;
                case 0xD9:
                    if (g == 0) fpush(rdf(m.addr));
                    else if (g == 2) wrf(m.addr, ST(0));
                    else if (g == 3) wrf(m.addr, ST(0)), fpop();
                    else fail("d9 mem");
                    return p;
                case 0xDB:
                    if (g == 0) fpush(double(int32_t(rd32(m.addr))));
                    else if (g == 3) wr32(m.addr, uint32_t(int32_t(std::floor(ST(0))))), fpop();  // fistp, round-down mode
                    else fail("db mem");
                    return p;
                case 0xDC:  // arith st0, m64
                    fail("dc mem");
                case 0xDD:
                    fail("dd mem");
                default:
                    fail("fpu mem");
            }
        }
        ++p;
        const int i = m0 & 7, g = (m0 >> 3) & 7;
        switch (op) {
            case 0xD8:
                if (g == 2 || g == 3) {
                    fcompare(ST(0), ST(i));
                    if (g == 3) fpop();
                } else {
                    ST(0) = farith(g, ST(0), ST(i));
                }
                return p;
            case 0xD9:
                if (g == 0) {
                    fpush(ST(i));
                } else if (g == 1) {
                    std::swap(ST(0), ST(i));
                } else if (m0 == 0xE0) {
                    ST(0) = -ST(0);
                } else if (m0 == 0xE1) {
                    ST(0) = std::fabs(ST(0));
                } else if (m0 == 0xE8) {
                    fpush(1.0);
                } else if (m0 == 0xEE) {
                    fpush(0.0);
                } else {
                    fail("d9");
                }
                return p;
            case 0xDC:  // st(i) = st(i) op st0 (Intel: DC E0+i = fsubr, E8+i = fsub, F0+i = fdivr, F8+i = fdiv)
            case 0xDE: {
                int opc = g;
                if (g == 4) opc = 5;
                else if (g == 5) opc = 4;
                else if (g == 6) opc = 7;
                else if (g == 7) opc = 6;
                if (op == 0xDE && m0 == 0xD9) {  // fcompp
                    fcompare(ST(0), ST(1));
                    fpop(), fpop();
                    return p;
                }
                if (g == 2 || g == 3) fail("dc/de compare");
                ST(i) = farith(opc, ST(i), ST(0));
                if (op == 0xDE) fpop();
                return p;
            }
            case 0xDD:
                if (g == 2) ST(i) = ST(0);
                else if (g == 3) ST(i) = ST(0), fpop();
                else fail("dd");
                return p;
            case 0xDF:
                if (m0 == 0xE0) {
                    r[0] = (r[0] & ~0xFFFFu) | sw;
                    return p;
                }
                fail("df");
            default:
                fail("fpu reg");
        }
    }
};

}  // namespace

std::optional<std::array<RoadTypeParams, 21>> loadRoadTypes(const DpcFile& world, std::string* error) {
    static std::mutex mu;
    static std::unordered_map<std::string, std::pair<std::optional<std::array<RoadTypeParams, 21>>, std::string>> cache;
    std::lock_guard<std::mutex> lk(mu);
    const std::string exe = engineExePath(world);
    auto it = cache.find(exe);
    if (it == cache.end()) {
        std::pair<std::optional<std::array<RoadTypeParams, 21>>, std::string> res;
        try {
            Image img;
            if (!img.load(exe)) throw std::runtime_error("cannot read " + exe);
            Cpu cpu(img);
            // Fake objects: `this` (AGW_TerraRoad, vtable pointer 0 -> its virtual call is skipped)
            // and the GenWorld it reads runtime sizes from (zeros: those fields are not used here).
            constexpr uint32_t kStack = 0x0ff00000, kThis = 0x0e000000, kWorld = 0x0d000000, kRet = 0x0c000000;
            for (int run = 0; run < 2; ++run) {  // the engine initialises two GenWorlds (0x77c690)
                std::memset(cpu.r, 0, sizeof cpu.r);
                cpu.top = 0;
                cpu.r[4] = kStack;
                cpu.r[1] = kThis;
                cpu.push(kWorld);  // argument: the GenWorld
                cpu.push(kRet);
                cpu.run(kInit, kRet);
            }
            std::array<RoadTypeParams, 21> t{};
            for (uint32_t k = 0; k < 21; ++k) {
                const uint32_t b = kTable + k * kStride;
                RoadTypeParams& q = t[k];
                q.reach = cpu.rdf(b + 0x00), q.crown = cpu.rdf(b + 0x24), q.verge = cpu.rdf(b + 0x28), q.noise = cpu.rdf(b + 0x34);
                q.fade = cpu.rdf(b + 0x4c), q.ditch = cpu.rdf(b + 0x50), q.ditchDepth = cpu.rdf(b + 0x54), q.stretch = cpu.rdf(b + 0x5c);
                q.bumpAmp = cpu.rdf(b + 0x74), q.bumpLen = cpu.rdf(b + 0x78), q.waveAmp = cpu.rdf(b + 0x7c), q.waveLen = cpu.rdf(b + 0x80);
                q.band = cpu.rdf(b + 0xbc), q.core = cpu.rdf(b + 0xc4), q.flat = cpu.rdf(b + 0xcc);
                q.aiSpeedBlend = cpu.rdf(b + 0xb4);
                q.smooth = cpu.rdf(b + 0x1f0) * cpu.rdf(b + 0x1e4), q.smoothMin = cpu.rdf(b + 0x1e8);
                // Sanity: plausible metres for every road type (a different FUEL.exe fails here).
                auto in = [](float v, float lo, float hi) { return std::isfinite(v) && v >= lo && v <= hi; };
                if (!(in(q.reach, 1, 200) && in(q.band, 1, 200) && q.band <= q.reach && in(q.fade, 0.5f, 100) && in(q.flat, -50, 100) &&
                      in(q.crown, -5, 5) && in(q.verge, -5, 5) && in(q.noise, -5, 5) && in(q.ditch, 0, 50) && in(q.ditchDepth, 0, 10) &&
                      in(q.stretch, 0, 100) && in(q.waveLen, 0, 1000) && in(q.smooth, 0, 1000) && in(q.smoothMin, 0, 1000))) {
                    char m[200];
                    std::snprintf(m, sizeof m, "road type %u: implausible values (reach %g band %g fade %g flat %g)", k, q.reach, q.band, q.fade,
                                  q.flat);
                    throw std::runtime_error(m);
                }
            }
            res.first = t;
        } catch (const std::exception& e) {
            res.second = std::string("road table from FUEL.exe: ") + e.what();
        }
        it = cache.emplace(exe, std::move(res)).first;
    }
    if (error) *error = it->second.second;
    return it->second.first;
}

}  // namespace fuel
