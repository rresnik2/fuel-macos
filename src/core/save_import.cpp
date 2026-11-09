// PK25: FUEL's PC career save container and ModelOrder1C packing (see save_import.hpp, notes/pk25.md).
// Tags: [O] observed in FUEL.exe, [I] inferred, [port] port choice.
#include "save_import.hpp"

#include <array>
#include <cstring>
#include <fstream>

namespace fuel::pcsave {
namespace {

// 0xa01dc8: t[i] = i << 24 run through 8 MSB-first steps of poly 0x04C11DB7 [O: the table in FUEL.exe is exactly this].
const std::array<uint32_t, 256>& crcTable() {
    static const std::array<uint32_t, 256> t = [] {
        std::array<uint32_t, 256> a{};
        for (uint32_t i = 0; i < 256; ++i) {
            uint32_t c = i << 24;
            for (int k = 0; k < 8; ++k) c = (c & 0x80000000u) ? (c << 1) ^ 0x04C11DB7u : c << 1;
            a[i] = c;
        }
        return a;
    }();
    return t;
}

// ---- ModelOrder1C (0x83c bytes, vtable 0x9d3e68) -------------------------------------------------------------------
// Context table = counts [0..0x100] (+0..+0x400), total [0x101] (+0x404), flag byte [0x102] (+0x408). 256 context
// tables (+0x438 pointers), the escape table at +0x2c (counts all 1, total 0x101 +0x430, flag 1 +0x434), the previous
// symbol +0x28, the current table +0x838.
struct Table {
    uint32_t n[0x102] = {};  // counts 0..0x100, total at [0x101]
    uint8_t flag = 0;
};

struct Model {
    std::vector<Table> ctx = std::vector<Table>(256);
    Table esc;
    int prev = -1;
    Table* cur = nullptr;

    void reset() {  // vf01 0x7fe140
        cur = &ctx[0];
        for (Table& t : ctx) {
            std::memset(t.n, 0, sizeof t.n);
            t.n[0x101] = 2;
            t.n[0xfe] = 1;   // escape
            t.n[0x100] = 1;  // end
            t.flag = 0;
        }
        esc.n[0x101] = 0x101;
        for (int i = 0; i < 0x101; ++i) esc.n[i] = 1;
        esc.flag = 1;
        prev = -1;
    }
    void update(int s) {  // vf02 0x7fe270
        if (s == 0x100) {
            ++cur->n[0x100], ++cur->n[0x101];
            return;
        }
        if (prev != -1) {
            cur = &ctx[size_t(prev)];
            ++cur->n[s], ++cur->n[0x101];
        }
        prev = s;
        cur = &ctx[size_t(s)];
    }
    uint32_t total() const { return cur->n[0x101]; }  // vf04 0x7fe120
    bool flag() const { return cur->flag != 0; }      // vf06 0x7fe130
    // vf03 0x7fe2d0 -> 0x7fe6b0: the range of `s` in the current table; a count of 0 (or a literal 0xfe outside the
    // escape table) codes the escape 0xfe instead and returns 0xfe, switching to the escape table.
    int encodeRange(int s, uint32_t r[3]) {
        int sym = s, ret = 0;
        if (cur->n[s] == 0) sym = ret = 0xfe;
        else if (s == 0xfe && cur->flag == 0) ret = 0xfe;
        uint32_t lo = 0;
        for (int i = 0; i < sym; ++i) lo += cur->n[i];
        r[0] = lo, r[1] = lo + cur->n[sym], r[2] = cur->n[0x101];
        if (ret == 0xfe) cur = &esc;
        return ret;
    }
    // vf05 0x7fe300 -> 0x7fe710: the symbol whose range holds `target`; 0xfe outside the escape table switches to it.
    int decode(uint32_t target, uint32_t r[3]) {
        const uint32_t* t = cur->n;
        uint32_t cum = 0;
        int i = 0;
        if (t[0] <= target) {
            uint32_t next;
            do {
                cum += t[i];
                ++i;
                next = t[i] + cum;
            } while (next <= target && i < 0x100);  // [port] the bound: FUEL's loop has none
        }
        r[0] = cum, r[1] = cum + t[i], r[2] = t[0x101];
        if (i == 0xfe && !flag()) cur = &esc;
        return i;
    }
};

// ---- the coder state (model +4) ------------------------------------------------------------------------------------
struct Coder {
    uint8_t byte = 0, bits = 0;   // +0 / +1
    uint32_t low = 0, high = 0x7fffffff, step = 0, pending = 0, code = 0;  // +8 +0xc +0x10 +0x14 +0x18
    std::vector<uint8_t>* out = nullptr;
    const uint8_t* in = nullptr;
    size_t inSize = 0, inPos = 0;

    // encoder bit (0x7fe790; the same inline sequence in 0x7fe870 / 0x7fea50)
    void emit(uint32_t b) {
        ++bits;
        byte = uint8_t(byte * 2 | b);
        if (bits == 8) out->push_back(byte), bits = 0;
    }
    // 0x7fe870: narrow to [lo, hi) of total, then E1 / E2 / E3 scaling
    void encode(const uint32_t r[3]) {
        step = (high - low + 1u) / r[2];
        high = r[1] * step - 1u + low;
        low = r[0] * step + low;
        for (;;) {
            while (high < 0x40000000u) {
                emit(0);
                low <<= 1;
                high = high * 2u + 1u;
                for (; pending; --pending) emit(1);
            }
            if (low < 0x40000000u) break;
            emit(1);
            low = low * 2u - 0x80000000u;
            high = high * 2u - 0x7fffffffu;
            for (; pending; --pending) emit(0);
        }
        while (low >= 0x20000000u && high < 0x60000000u) {
            ++pending;
            low = low * 2u - 0x40000000u;
            high = high * 2u - 0x3fffffffu;
        }
    }
    void finish() {  // 0x7fea50 + 0x7fe7e0
        if (low < 0x20000000u) {
            emit(0);
            for (uint32_t k = 0; k < pending + 1u; ++k) emit(1);
        } else {
            emit(1);  // [O] no pending 0s here
        }
        while (bits != 0) {  // 0x7fe7e0: pad the last byte with 0s
            byte = uint8_t(byte << 1);
            if (++bits == 8) out->push_back(byte), bits = 0;
        }
    }

    // decoder input bit (0x7feaf0 / 0x7feb70 inline; 0x735fa0 reads one byte, a failed read gives 0)
    uint32_t inBit() {
        if (bits == 0) {
            byte = inPos < inSize ? in[inPos] : 0;
            if (inPos < inSize) ++inPos;
            bits = 8;
        }
        const uint32_t b = byte >> 7;
        byte = uint8_t(byte << 1);
        --bits;
        return b;
    }
    void start() {  // 0x7feaf0: 31 bits MSB-first into the code value
        for (int k = 0; k < 31; ++k) code = code * 2u | inBit();
    }
    void narrow(uint32_t lo, uint32_t hi) {  // 0x7feb70
        high = step * hi - 1u + low;
        low = step * lo + low;
        uint32_t l;
        for (;;) {
            while (high < 0x40000000u) {
                low <<= 1;
                high = high * 2u + 1u;
                code = code * 2u + inBit();
                pending = 0;
            }
            l = low;
            if (l < 0x40000000u) break;
            low = l * 2u - 0x80000000u;
            high = high * 2u - 0x7fffffffu;
            code = code * 2u + inBit() + 0x80000000u;
            pending = 0;
        }
        while (l >= 0x20000000u && high < 0x60000000u) {
            ++pending;
            low = low * 2u - 0x40000000u;
            high = high * 2u - 0x3fffffffu;
            code = code * 2u + inBit() + 0xc0000000u;
            l = low;
        }
    }
};

}  // namespace

uint32_t saveCrc(const uint8_t* p, size_t n) {
    const auto& t = crcTable();
    uint32_t c = 0;
    for (size_t i = 0; i < n; ++i) {
        uint8_t b = p[i];
        if (b >= 'A' && b <= 'Z') b = uint8_t(b + 0x20);  // 0xa018f0: ASCII upper case -> lower, every other byte as is [O]
        c = (c << 8) ^ t[(b ^ (c >> 24)) & 0xff];
    }
    return c;
}

std::vector<uint8_t> pack(const std::vector<uint8_t>& raw) {
    Model m;
    Coder c;
    std::vector<uint8_t> coded;
    c.out = &coded;
    m.reset();  // 0x7fe3a0
    uint32_t r[3];
    for (uint8_t b : raw) {
        int e;
        do {
            e = m.encodeRange(b, r);
            c.encode(r);
        } while (e == 0xfe);
        m.update(b);
    }
    // Pad with symbol 0 until the output position moves (the decoder stops at the unpacked size) [O].
    const size_t pos0 = coded.size();
    while (coded.size() == pos0) {
        int e;
        do {
            e = m.encodeRange(0, r);
            c.encode(r);
        } while (e == 0xfe);
        m.update(0);
    }
    c.finish();
    // 0x7597e0: {u32 coded + 9, u32 raw size, coded, 1 byte} (it copies total - 8 bytes from the target stream; the
    // byte after the coded output is whatever that buffer held [port: 0]).
    std::vector<uint8_t> blob(8 + coded.size() + 1, 0);
    const uint32_t total = uint32_t(coded.size() + 9), n = uint32_t(raw.size());
    std::memcpy(&blob[0], &total, 4);
    std::memcpy(&blob[4], &n, 4);
    if (!coded.empty()) std::memcpy(&blob[8], coded.data(), coded.size());
    return blob;
}

bool unpack(const uint8_t* blob, size_t avail, std::vector<uint8_t>* raw, std::string* error) {
    auto fail = [&](const char* e) {
        if (error) *error = e;
        return false;
    };
    if (avail < 8) return fail("packed blob shorter than its header");
    uint32_t total = 0, n = 0;
    std::memcpy(&total, blob, 4);
    std::memcpy(&n, blob + 4, 4);
    if (total < 9 || total > avail) return fail("packed blob size out of range");
    if (n == 0 || n > (64u << 20)) return fail("unpacked size out of range");
    Model m;
    Coder c;
    c.in = blob + 8, c.inSize = total - 8;  // 0x759a50: "Source" = blob + 8, size total - 8 [O]
    raw->clear();
    raw->reserve(n);
    c.start();
    m.reset();  // 0x7fe540
    // [port] FUEL loops until the output reaches the size; a damaged blob is bounded here.
    const uint64_t maxSteps = uint64_t(n) * 4 + (uint64_t(total) << 4) + 4096;
    for (uint64_t steps = 0; steps < maxSteps; ++steps) {
        const uint32_t total2 = m.total();
        if (total2 == 0) return fail("model total 0");
        c.step = uint32_t((uint64_t(c.high - c.low) + 1u) / total2);
        if (c.step == 0) return fail("coder step 0");
        const bool flag = m.flag();
        uint32_t r[3];
        const int sym = m.decode((c.code - c.low) / c.step, r);
        if (sym < 0x100 && (sym != 0xfe || flag)) {
            raw->push_back(uint8_t(sym));
            if (raw->size() == n) return true;
        }
        c.narrow(r[0], r[1]);
        if (sym != 0xfe || flag) {
            m.update(sym);
            if (sym == 0x100) return fail("end symbol before the unpacked size");
        }
    }
    return fail("coded data ends before the unpacked size");
}

bool makeBody(const std::vector<uint8_t>& sections, std::vector<uint8_t>* body) {
    const std::vector<uint8_t> blob = pack(sections);
    // 0x4ebea0 copies game+0x2ec bytes = the blob's size field (coded + 9) [O]; only when < 0x30d3b.
    uint32_t size = 0;
    std::memcpy(&size, blob.data(), 4);
    if (size >= kMarkerAt) return false;
    body->assign(kBodySize, 0);
    std::memcpy(body->data(), blob.data(), std::min<size_t>(size, blob.size()));
    (*body)[kMarkerAt] = 0x3d;
    const uint32_t crc = saveCrc(body->data(), kCrcAt);
    std::memcpy(&(*body)[kCrcAt], &crc, 4);
    return true;
}

int checkBody(const std::vector<uint8_t>& body) {
    if (body.size() < kBodySize) return Corrupt;
    if (body[kMarkerAt] != 0x3d) return Corrupt;  // 0x4eb9c0: byte 0x30d3b == '='
    uint32_t stored = 0;
    std::memcpy(&stored, &body[kCrcAt], 4);
    return stored == saveCrc(body.data(), kCrcAt) ? Ok : Corrupt;
}

int readFile(const std::string& path, std::vector<uint8_t>* body) {
    std::ifstream f(path, std::ios::binary);  // read only (0x840190: GENERIC_READ, OPEN_EXISTING)
    if (!f) return NotFound;
    f.seekg(0, std::ios::end);
    const std::streamoff size = f.tellg();
    f.seekg(0);
    if (size <= 0) return Empty;
    uint32_t n = 0;
    if (!f.read(reinterpret_cast<char*>(&n), 4)) return Empty;
    if (std::streamoff(n) > size) return Empty;
    std::vector<uint8_t> blob(n);
    if (n && !f.read(reinterpret_cast<char*>(blob.data()), n)) return Empty;  // read count != n -> 8 [O]
    // XLiveUnprotectData(blob) -> the CRC copy [port: not possible without GFWL; the body's CRC is checked instead].
    body->assign(kBodySize, 0);
    if (!f.read(reinterpret_cast<char*>(body->data()), std::streamsize(kBodySize))) return Empty;  // read != 200000 -> 8 [O]
    if (n == 4) {  // a fixture written by writeFile (or any blob holding the plain CRC): compare as 0x840190 does
        uint32_t a = 0, b = 0;
        std::memcpy(&a, blob.data(), 4);
        std::memcpy(&b, &(*body)[kBodySize - 4], 4);
        if (a != b) return Corrupt;
    }
    return Ok;
}

int writeFile(const std::string& path, const std::vector<uint8_t>& body) {
    if (body.size() != kBodySize) return WriteFailed;
    std::ofstream f(path, std::ios::binary | std::ios::trunc);
    if (!f) return WriteFailed;
    const uint32_t n = 4;
    f.write(reinterpret_cast<const char*>(&n), 4);
    f.write(reinterpret_cast<const char*>(&body[kBodySize - 4]), 4);
    f.write(reinterpret_cast<const char*>(body.data()), std::streamsize(body.size()));
    return f ? Ok : WriteFailed;
}

int importFile(const std::string& path, std::vector<uint8_t>* sections, std::string* error) {
    std::vector<uint8_t> body;
    const int rc = readFile(path, &body);
    if (rc != Ok) {
        if (error) *error = rc == NotFound ? "cannot open the file" : rc == Empty ? "empty or short file" : "protected CRC mismatch";
        return rc;
    }
    if (checkBody(body) != Ok) {
        if (error) *error = "save marker or CRC mismatch (0x4eb9c0)";
        return Corrupt;
    }
    std::string e;
    if (!unpack(body.data(), kMarkerAt, sections, &e)) {
        if (error) *error = "unpack: " + e;
        return Corrupt;
    }
    return Ok;
}

}  // namespace fuel::pcsave
