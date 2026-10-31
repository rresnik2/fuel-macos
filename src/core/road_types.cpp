// Road-type table rebuilt from the player's FUEL.exe by running the engine's own
// initialisation code (FUN_006f24a0 and its three helpers) in a small emulator.
//
// Nothing of the table is stored here: only addresses, opcode handling and the
// arithmetic the code performs. The emulator knows the ~45 instruction forms that occur in
// the routine and stops with an error on anything else. x87 results are kept as float32
// after every operation (that is what the engine stores into the table).
#include "road_types.hpp"

#include <algorithm>
#include <cmath>
#include <cstdio>
#include <cstring>
#include <fstream>
#include <iterator>
#include <optional>
#include <stdexcept>
#include <vector>

namespace fuel {
namespace {

// ---- where things live in FUEL.exe (virtual addresses) -------------------------------------
constexpr uint32_t kTable = 0xa677e0, kTypes = 21, kBlock = 0x200;  // g_RoadTypes
constexpr uint32_t kEntry = 0x6f2560;  // first useful instruction; the frame set-up before it is skipped
constexpr uint32_t kExit = 0x6f567a;   // epilogue of the init function: everything is done by here
constexpr uint32_t kGwColours = 0x47b74;  // GenWorld palette inside the GenWorld object (16 bytes per slot)
constexpr uint32_t kHelpers[3] = {0x6f1240, 0x6f5690, 0x6f5770};  // the only functions the init calls

struct Range { uint32_t from, to; };
constexpr Range kMain{0x6f24a0, 0x6f5682};  // whole init function
constexpr Range kHelperCode[3] = {{0x6f1240, 0x6f1296}, {0x6f5690, 0x6f576a}, {0x6f5770, 0x6f5937}};

// SHA-256 of the code bytes of the Steam/retail build this was written against (hashes, not game data).
const char* const kMainHash = "eb5f54ea7abe5b7b1cad50afc01e21fc5f26141e6a10cd55919422bf09a8ebf5";
const char* const kHelperHash = "972ab0788d7bc13a28f5f3cf0880e94018bbb34fa79ebc68175749d6637049a0";  // the three helpers back to back

struct Fail : std::runtime_error { using std::runtime_error::runtime_error; };
[[noreturn]] void fail(const char* what, uint32_t at) {
    char b[128];
    std::snprintf(b, sizeof b, "%s (at 0x%x)", what, at);
    throw Fail(b);
}

// ---- SHA-256 (to recognise the exe build) ----------------------------------------------------
std::string sha256(const std::vector<uint8_t>& msg) {
    static const uint32_t K[64] = {
        0x428a2f98, 0x71374491, 0xb5c0fbcf, 0xe9b5dba5, 0x3956c25b, 0x59f111f1, 0x923f82a4, 0xab1c5ed5, 0xd807aa98, 0x12835b01, 0x243185be,
        0x550c7dc3, 0x72be5d74, 0x80deb1fe, 0x9bdc06a7, 0xc19bf174, 0xe49b69c1, 0xefbe4786, 0x0fc19dc6, 0x240ca1cc, 0x2de92c6f, 0x4a7484aa,
        0x5cb0a9dc, 0x76f988da, 0x983e5152, 0xa831c66d, 0xb00327c8, 0xbf597fc7, 0xc6e00bf3, 0xd5a79147, 0x06ca6351, 0x14292967, 0x27b70a85,
        0x2e1b2138, 0x4d2c6dfc, 0x53380d13, 0x650a7354, 0x766a0abb, 0x81c2c92e, 0x92722c85, 0xa2bfe8a1, 0xa81a664b, 0xc24b8b70, 0xc76c51a3,
        0xd192e819, 0xd6990624, 0xf40e3585, 0x106aa070, 0x19a4c116, 0x1e376c08, 0x2748774c, 0x34b0bcb5, 0x391c0cb3, 0x4ed8aa4a, 0x5b9cca4f,
        0x682e6ff3, 0x748f82ee, 0x78a5636f, 0x84c87814, 0x8cc70208, 0x90befffa, 0xa4506ceb, 0xbef9a3f7, 0xc67178f2};
    uint32_t h[8] = {0x6a09e667, 0xbb67ae85, 0x3c6ef372, 0xa54ff53a, 0x510e527f, 0x9b05688c, 0x1f83d9ab, 0x5be0cd19};
    std::vector<uint8_t> m = msg;
    const uint64_t bits = uint64_t(msg.size()) * 8;
    m.push_back(0x80);
    while (m.size() % 64 != 56) m.push_back(0);
    for (int i = 7; i >= 0; --i) m.push_back(uint8_t(bits >> (8 * i)));
    auto ror = [](uint32_t x, int n) { return (x >> n) | (x << (32 - n)); };
    for (size_t o = 0; o < m.size(); o += 64) {
        uint32_t w[64];
        for (int i = 0; i < 16; ++i) w[i] = uint32_t(m[o + 4 * i]) << 24 | uint32_t(m[o + 4 * i + 1]) << 16 | uint32_t(m[o + 4 * i + 2]) << 8 | m[o + 4 * i + 3];
        for (int i = 16; i < 64; ++i) {
            const uint32_t s0 = ror(w[i - 15], 7) ^ ror(w[i - 15], 18) ^ (w[i - 15] >> 3), s1 = ror(w[i - 2], 17) ^ ror(w[i - 2], 19) ^ (w[i - 2] >> 10);
            w[i] = w[i - 16] + s0 + w[i - 7] + s1;
        }
        uint32_t a = h[0], b = h[1], c = h[2], d = h[3], e = h[4], f = h[5], g = h[6], hh = h[7];
        for (int i = 0; i < 64; ++i) {
            const uint32_t t1 = hh + (ror(e, 6) ^ ror(e, 11) ^ ror(e, 25)) + ((e & f) ^ (~e & g)) + K[i] + w[i];
            const uint32_t t2 = (ror(a, 2) ^ ror(a, 13) ^ ror(a, 22)) + ((a & b) ^ (a & c) ^ (b & c));
            hh = g; g = f; f = e; e = d + t1; d = c; c = b; b = a; a = t1 + t2;
        }
        h[0] += a; h[1] += b; h[2] += c; h[3] += d; h[4] += e; h[5] += f; h[6] += g; h[7] += hh;
    }
    char out[65];
    for (int i = 0; i < 8; ++i) std::snprintf(out + 8 * i, 9, "%08x", h[i]);
    return out;
}

// ---- the PE image: virtual address -> byte --------------------------------------------------
class Image {
public:
    bool load(const std::string& path) {
        std::ifstream f(path, std::ios::binary);
        if (!f) return false;
        b_.assign(std::istreambuf_iterator<char>(f), std::istreambuf_iterator<char>());
        const size_t pe = u32(0x3c);
        if (u32(pe) != 0x00004550) return false;  // "PE\0\0"
        const uint32_t sections = u16(pe + 6), optSize = u16(pe + 20);
        base_ = u32(pe + 24 + 28);
        for (uint32_t k = 0; k < sections; ++k) {
            const size_t sh = pe + 24 + optSize + size_t(k) * 40;
            secs_.push_back({u32(sh + 12), u32(sh + 8), u32(sh + 20), u32(sh + 16)});
        }
        return !secs_.empty();
    }
    // Section bytes past the raw size read as zero (uninitialised data).
    bool byte(uint32_t va, uint8_t& out) const {
        const uint32_t rva = va - base_;
        for (const Sec& s : secs_) {
            if (rva < s.rva || rva - s.rva >= std::max(s.vsize, s.raw_size)) continue;
            const uint32_t off = rva - s.rva;
            if (off >= s.raw_size) { out = 0; return true; }
            if (size_t(s.raw) + off >= b_.size()) return false;
            out = b_[s.raw + off];
            return true;
        }
        return false;
    }
    bool bytes(Range r, std::vector<uint8_t>& out) const {
        for (uint32_t a = r.from; a < r.to; ++a) {
            uint8_t v;
            if (!byte(a, v)) return false;
            out.push_back(v);
        }
        return true;
    }

private:
    struct Sec { uint32_t rva, vsize, raw, raw_size; };
    uint32_t u16(size_t o) const { return o + 2 <= b_.size() ? uint32_t(b_[o] | b_[o + 1] << 8) : 0u; }
    uint32_t u32(size_t o) const { return o + 4 <= b_.size() ? uint32_t(b_[o] | b_[o + 1] << 8 | b_[o + 2] << 16 | uint32_t(b_[o + 3]) << 24) : 0u; }
    std::vector<uint8_t> b_;
    std::vector<Sec> secs_;
    uint32_t base_ = 0;
};

uint32_t floatBits(float f) { uint32_t u; std::memcpy(&u, &f, 4); return u; }
float bitsFloat(uint32_t u) { float f; std::memcpy(&f, &u, 4); return f; }

using Palette = std::function<std::array<float, 4>(int)>;

// ---- the emulator ----------------------------------------------------------------------------
// Memory map seen by the code:
//   - the road-type table (persistent between runs, starts as zeroed BSS),
//   - a stack, and a fake "TerraRoad" object (ebx at entry) whose +0xa8 points to
//   - a fake GenWorld object whose palette reads go to the palette callback,
//   - FUEL.exe itself, read only (the constants and the type-name strings).
// Reading stack/object bytes nobody wrote is an error, so a code path that depends on state
// this emulator does not model is caught instead of silently producing zeros.
class Emu {
public:
    Emu(const Image& img, const Palette& pal, std::vector<uint8_t>& table) : img_(img), pal_(pal), tbl_(table) {
        stack_.base = kStackBase; stack_.d.assign(kStackSize, 0); stack_.w.assign(kStackSize, 0);
        tr_.base = kTr; tr_.d.assign(kTrSize, 0); tr_.w.assign(kTrSize, 0);
        st(kTr + 0xa8, 4, kGw);                       // TR+0xa8 = GenWorld
        r_[ESP] = kStackBase + kStackSize - 0x100;
        for (uint32_t o = 0; o < 12; o += 4) st(r_[ESP] + o, 4, 0);  // the registers the prologue saved (popped at the end)
        st(r_[ESP] + 0xc, 4, kTr);                    // [esp+0xc] = TR, as the prologue left it
        r_[EBX] = kTr;
        eip_ = kEntry;
    }

    void run() {
        for (uint64_t steps = 0; eip_ != kExit; ++steps) {
            if (steps > 2000000) fail("runaway", eip_);
            step();
        }
    }

private:
    enum { EAX, ECX, EDX, EBX, ESP, EBP, ESI, EDI };
    static constexpr uint32_t kStackBase = 0x7f0000, kStackSize = 0x10000, kTr = 0x30000000, kTrSize = 0x5000, kGw = 0x40000000;
    static constexpr int kPaletteSlots = 64;

    struct Region { uint32_t base = 0; std::vector<uint8_t> d, w; };  // w = byte was written
    struct Modrm { int mod, reg, rm; uint32_t addr; };

    // -- memory --
    static bool inside(const Region& r, uint32_t a) { return a - r.base < r.d.size(); }
    uint8_t ld8(uint32_t a) {
        for (const Region* r : {&stack_, &tr_})
            if (inside(*r, a)) {
                if (!r->w[a - r->base]) fail("read of a byte nobody wrote", eip_);
                return r->d[a - r->base];
            }
        if (a - kTable < tbl_.size()) return tbl_[a - kTable];
        if (a - (kGw + kGwColours) < 16u * kPaletteSlots) {  // a palette colour: float k of slot s
            const uint32_t o = a - (kGw + kGwColours), slot = o / 16, comp = o % 16 / 4;
            auto& c = pal_cache_[slot];
            if (!c) c = pal_(int(slot));
            return uint8_t(floatBits((*c)[comp]) >> (8 * (o & 3)));
        }
        uint8_t v;
        if (img_.byte(a, v)) return v;
        fail("read of unmapped memory", eip_);
    }
    void st8(uint32_t a, uint8_t v) {
        for (Region* r : {&stack_, &tr_})
            if (inside(*r, a)) { r->d[a - r->base] = v; r->w[a - r->base] = 1; return; }
        if (a - kTable < tbl_.size()) { tbl_[a - kTable] = v; return; }
        fail("write outside the table, stack and TR object", eip_);
    }
    uint32_t ld(uint32_t a, int n) { uint32_t v = 0; for (int i = 0; i < n; ++i) v |= uint32_t(ld8(a + i)) << (8 * i); return v; }
    void st(uint32_t a, int n, uint32_t v) { for (int i = 0; i < n; ++i) st8(a + i, uint8_t(v >> (8 * i))); }

    // -- instruction stream --
    uint8_t f8() {
        uint8_t v;
        const bool code = (eip_ >= kMain.from && eip_ < kMain.to) || [&] {
            for (const Range& r : kHelperCode) if (eip_ >= r.from && eip_ < r.to) return true;
            return false;
        }();
        if (!code || !img_.byte(eip_, v)) fail("instruction fetch outside the known code", eip_);
        ++eip_;
        return v;
    }
    uint32_t f32() { uint32_t v = f8(); v |= uint32_t(f8()) << 8; v |= uint32_t(f8()) << 16; v |= uint32_t(f8()) << 24; return v; }
    uint32_t f16() { uint32_t v = f8(); v |= uint32_t(f8()) << 8; return v; }

    // -- registers --
    uint32_t reg(int i, int size) {
        if (size == 4) return r_[i];
        if (size == 2) return r_[i] & 0xffff;
        if (i > 3) fail("high byte register", eip_);
        return r_[i] & 0xff;
    }
    void setReg(int i, int size, uint32_t v) {
        if (size == 4) r_[i] = v;
        else if (size == 2) r_[i] = (r_[i] & 0xffff0000u) | (v & 0xffff);
        else if (i > 3) fail("high byte register", eip_);
        else r_[i] = (r_[i] & ~0xffu) | (v & 0xff);
    }

    // -- ModRM (32-bit addressing, with SIB and 8/32-bit displacements) --
    Modrm modrm(uint8_t b) {
        Modrm m{b >> 6, (b >> 3) & 7, b & 7, 0};
        if (m.mod == 3) return m;
        uint32_t a = 0;
        if (m.rm == 4) {
            const uint8_t sib = f8();
            const int idx = (sib >> 3) & 7, base = sib & 7;
            if (idx != 4) a += r_[idx] << (sib >> 6);
            if (base == 5 && m.mod == 0) a += f32(); else a += r_[base];
        } else if (m.rm == 5 && m.mod == 0) {
            a = f32();
        } else {
            a = r_[m.rm];
        }
        if (m.mod == 1) a += uint32_t(int32_t(int8_t(f8())));
        else if (m.mod == 2) a += f32();
        m.addr = a;
        return m;
    }
    Modrm modrm() { return modrm(f8()); }
    uint32_t getRm(const Modrm& m, int size) { return m.mod == 3 ? reg(m.rm, size) : ld(m.addr, size); }
    void setRm(const Modrm& m, int size, uint32_t v) { if (m.mod == 3) setReg(m.rm, size, v); else st(m.addr, size, v); }

    // -- integer ALU: 0 add, 4 and, 5 sub, 6 xor, 7 cmp (result dropped); only ZF is tracked --
    uint32_t alu(int kind, uint32_t a, uint32_t b) {
        uint32_t r;
        switch (kind) {
            case 0: r = a + b; break;
            case 4: r = a & b; break;
            case 5: case 7: r = a - b; break;
            case 6: r = a ^ b; break;
            default: fail("ALU operation", eip_);
        }
        zf_ = r == 0;
        return kind == 7 ? a : r;
    }
    void push(uint32_t v) { r_[ESP] -= 4; st(r_[ESP], 4, v); }
    uint32_t pop() { const uint32_t v = ld(r_[ESP], 4); r_[ESP] += 4; return v; }

    // -- x87 (float32 after every operation) --
    float& stI(int i) {
        if (i >= nst_) fail("x87 stack underflow", eip_);
        return fst_[nst_ - 1 - i];
    }
    void fpush(float v) { if (nst_ == 8) fail("x87 stack overflow", eip_); fst_[nst_++] = v; }
    void fpop() { if (nst_ == 0) fail("x87 stack underflow", eip_); --nst_; }
    // k: 0 x+y, 1 x*y, 4 x-y, 5 y-x, 6 x/y, 7 y/x
    static float fop(int k, float x, float y) {
        const double a = x, b = y;
        switch (k) {
            case 0: return float(a + b);
            case 1: return float(a * b);
            case 4: return float(a - b);
            case 5: return float(b - a);
            case 6: return float(a / b);
            default: return float(b / a);
        }
    }
    void x87(uint8_t op) {
        const uint8_t mb = f8();
        const int k = (mb >> 3) & 7, i = mb & 7;
        if (mb < 0xc0) {  // memory operand (32-bit float)
            const Modrm m = modrm(mb);
            if (op == 0xd8 && k != 2 && k != 3) { float& s = stI(0); s = fop(k, s, bitsFloat(ld(m.addr, 4))); return; }
            if (op == 0xd9 && k == 0) { fpush(bitsFloat(ld(m.addr, 4))); return; }
            if (op == 0xd9 && (k == 2 || k == 3)) { st(m.addr, 4, floatBits(stI(0))); if (k == 3) fpop(); return; }
            fail("x87 memory form", eip_);
        }
        if (op == 0xd9) {
            if (k == 0) { fpush(stI(i)); return; }                      // fld st(i)
            if (k == 1) { std::swap(stI(0), stI(i)); return; }          // fxch st(i)
            if (mb == 0xe0) { stI(0) = -stI(0); return; }               // fchs
            if (mb == 0xe8) { fpush(1.0f); return; }                    // fld1
            if (mb == 0xee) { fpush(0.0f); return; }                    // fldz
            fail("x87 D9 form", eip_);
        }
        if (k == 2 || k == 3) fail("x87 compare form", eip_);
        if (op == 0xd8) { stI(0) = fop(k, stI(0), stI(i)); return; }     // st0 = st0 op st(i)
        // DC / DE: st(i) = st(i) op st0; the sub/div encodings are crossed over (E0<->E8, F0<->F8)
        const int kk = (k == 4 || k == 6) ? k + 1 : (k == 5 || k == 7) ? k - 1 : k;
        stI(i) = fop(kk, stI(i), stI(0));
        if (op == 0xde) fpop();
    }

    void step() {
        bool op16 = false, rep = false;
        uint8_t op = f8();
        for (;; op = f8()) {
            if (op == 0x66) op16 = true;
            else if (op == 0xf3) rep = true;
            else break;
        }
        const int w = op16 ? 2 : 4;
        if (rep && op != 0xa5) fail("rep prefix", eip_);
        if (op16 && op != 0x89) fail("16-bit operand", eip_);

        if (op >= 0xd8 && op <= 0xdf) {
            if (op != 0xd8 && op != 0xd9 && op != 0xdc && op != 0xde) fail("x87 opcode", eip_);
            x87(op);
        } else if (op >= 0x50 && op <= 0x57) { push(r_[op & 7]);                          // push r32
        } else if (op >= 0x58 && op <= 0x5f) { r_[op & 7] = pop();                         // pop r32
        } else if (op >= 0xb8 && op <= 0xbf) { r_[op & 7] = f32();                         // mov r32, imm32
        } else {
            switch (op) {
                case 0x68: push(f32()); break;                                             // push imm32
                case 0x6a: push(uint32_t(int32_t(int8_t(f8())))); break;                   // push imm8
                case 0x05: r_[EAX] = alu(0, r_[EAX], f32()); break;                        // add eax, imm32
                case 0x03: { const Modrm m = modrm(); r_[m.reg] = alu(0, r_[m.reg], getRm(m, 4)); break; }  // add r32, r/m32
                case 0x31: { const Modrm m = modrm(); setRm(m, 4, alu(6, getRm(m, 4), r_[m.reg])); break; } // xor r/m32, r32
                case 0x33: { const Modrm m = modrm(); r_[m.reg] = alu(6, r_[m.reg], getRm(m, 4)); break; }  // xor r32, r/m32
                case 0x81: case 0x83: {                                                    // add/and/sub/cmp r/m32, imm
                    const Modrm m = modrm();
                    const uint32_t imm = op == 0x81 ? f32() : uint32_t(int32_t(int8_t(f8())));
                    if (m.reg != 0 && m.reg != 4 && m.reg != 5 && m.reg != 7) fail("group-1 operation", eip_);
                    const uint32_t v = alu(m.reg, getRm(m, 4), imm);
                    if (m.reg != 7) setRm(m, 4, v);
                    break;
                }
                case 0x84: { const Modrm m = modrm(); zf_ = (getRm(m, 1) & reg(m.reg, 1)) == 0; break; }   // test r/m8, r8
                case 0x88: { const Modrm m = modrm(); setRm(m, 1, reg(m.reg, 1)); break; }                 // mov r/m8, r8
                case 0x89: { const Modrm m = modrm(); setRm(m, w, reg(m.reg, w)); break; }                 // mov r/m, r
                case 0x8a: { const Modrm m = modrm(); setReg(m.reg, 1, getRm(m, 1)); break; }              // mov r8, r/m8
                case 0x8b: { const Modrm m = modrm(); r_[m.reg] = getRm(m, 4); break; }                    // mov r32, r/m32
                case 0xa3: st(f32(), 4, r_[EAX]); break;                                                   // mov [abs32], eax
                case 0x8d: { const Modrm m = modrm(); if (m.mod == 3) fail("lea of a register", eip_); r_[m.reg] = m.addr; break; }
                case 0xc6: { const Modrm m = modrm(); if (m.reg) fail("C6 /r", eip_); setRm(m, 1, f8()); break; }   // mov r/m8, imm8
                case 0xc7: { const Modrm m = modrm(); if (m.reg) fail("C7 /r", eip_); setRm(m, 4, f32()); break; }  // mov r/m32, imm32
                case 0x90: break;                                                          // nop
                case 0x74: { const int32_t d = int8_t(f8()); if (zf_) eip_ += uint32_t(d); break; }          // je
                case 0x75: { const int32_t d = int8_t(f8()); if (!zf_) eip_ += uint32_t(d); break; }         // jne
                case 0xeb: { const int32_t d = int8_t(f8()); eip_ += uint32_t(d); break; }                   // jmp short
                case 0xe9: { const uint32_t d = f32(); eip_ += d; break; }                                   // jmp near
                case 0xe8: {                                                               // call a helper
                    const uint32_t d = f32(), target = eip_ + d;
                    if (std::find(std::begin(kHelpers), std::end(kHelpers), target) == std::end(kHelpers)) fail("call to an unknown function", eip_);
                    push(eip_);
                    eip_ = target;
                    break;
                }
                case 0xc3: eip_ = pop(); break;                                            // ret
                case 0xc2: { const uint32_t n = f16(); eip_ = pop(); r_[ESP] += n; break; }                  // ret imm16
                case 0xa5: {                                                               // rep movsd
                    if (!rep) fail("movsd without rep", eip_);
                    for (; r_[ECX]; --r_[ECX], r_[ESI] += 4, r_[EDI] += 4) st(r_[EDI], 4, ld(r_[ESI], 4));
                    break;
                }
                case 0x0f: {
                    const uint8_t op2 = f8();
                    if (op2 == 0xb6) { const Modrm m = modrm(); r_[m.reg] = getRm(m, 1); }        // movzx r32, r/m8
                    else if (op2 == 0x84 || op2 == 0x85) {                                          // je / jne near
                        const uint32_t d = f32();
                        if (zf_ == (op2 == 0x84)) eip_ += d;
                    } else fail("0F opcode", eip_);
                    break;
                }
                default: fail("opcode", eip_ - 1);
            }
        }
    }

    const Image& img_;
    const Palette& pal_;
    std::vector<uint8_t>& tbl_;
    Region stack_, tr_;
    std::array<std::optional<std::array<float, 4>>, kPaletteSlots> pal_cache_;
    uint32_t r_[8] = {0, 0, 0, 0, 0, 0, 0, 0};
    uint32_t eip_ = 0;
    bool zf_ = false;
    float fst_[8] = {};
    int nst_ = 0;
};

bool codeMatches(const Image& img) {
    std::vector<uint8_t> main, helpers;
    if (!img.bytes(kMain, main)) return false;
    for (const Range& r : kHelperCode)
        if (!img.bytes(r, helpers)) return false;
    return sha256(main) == kMainHash && sha256(helpers) == kHelperHash;
}

}  // namespace

uint32_t RoadType::u32(uint32_t off) const {
    uint32_t v;
    std::memcpy(&v, &raw[off], 4);
    return v;
}

RoadTypeTable buildRoadTypes(const std::string& exePath, const std::function<std::array<float, 4>(int slot)>& palette, bool laterInit) {
    RoadTypeTable out;
    try {
        Image img;
        if (!palette || !img.load(exePath)) return out;
        if (!codeMatches(img)) {
            std::fprintf(stderr, "fuel: %s is not the FUEL.exe build the road-type table was written for\n", exePath.c_str());
            return out;
        }
        std::vector<uint8_t> table(size_t(kTypes) * kBlock, 0);
        // A second run sees the +0xb8 values the first run left behind, as a repeated init does in the engine.
        for (int pass = 0; pass < (laterInit ? 2 : 1); ++pass) Emu(img, palette, table).run();
        for (uint32_t t = 0; t < kTypes; ++t) {
            RoadType& r = out.types[t];
            std::memcpy(r.raw.data(), &table[size_t(t) * kBlock], kBlock);
            std::memcpy(r.f.data(), r.raw.data(), kBlock);  // little endian host assumed (as the rest of the port)
            const char* n = reinterpret_cast<const char*>(&r.raw[0xd8]);
            r.name.assign(n, strnlen(n, 0x100));
        }
        out.valid = true;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "fuel: road-type emulation failed: %s\n", e.what());
        out.valid = false;
    }
    return out;
}

}  // namespace fuel
