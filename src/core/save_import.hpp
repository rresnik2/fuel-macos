// PK25: FUEL's PC career save container and its packing, so an original Windows career.sav can be
// imported into the port's FUELPSAV file. Decoded from FUEL.exe (notes/pk25.md, p98 b024/b035/b070/b073):
//
//   file (SaveGamePc_Z write 0x840000 / read 0x840190)
//     u32 n | n bytes: XLiveProtectData of the body's last 4 bytes (the CRC) | 200000-byte body
//   body (SaveManager_Save 0x4ebea0 / MemoryCardManager_C vf17 0x4eb9c0, ScanSaves 0x4ec3f0)
//     packed blob at 0 (size < 0x30d3b), zeros, body[0x30d3b] = '=', u32 CRC at 0x30d3c over bytes [0, 0x30d3c)
//   CRC: MSB-first CRC-32 (poly 0x04C11DB7, table 0xa01dc8), each byte lower-cased first (map 0xa018f0), init 0, no xor
//   packed blob (pack 0x565990 -> 0x7597e0, unpack 0x565ac0 -> 0x759a50; mode always 1 = ModelOrder1C)
//     u32 total = coded bytes + 9 | u32 unpacked size | coded bytes (+ 1 byte the encoder copies past them)
//   coder: 31-bit arithmetic coder (encode 0x7fe3a0 / 0x7fe870 / 0x7fea50 / 0x7fe7e0 / 0x7fe790, decode 0x7feaf0 /
//   0x7fe540 / 0x7feb70) over the adaptive order-1 model ModelOrder1C (0x7fe140 reset, 0x7fe270 update, 0x7fe2d0 /
//   0x7fe6b0 encode range, 0x7fe300 / 0x7fe710 decode, 0x7fe120 total, 0x7fe130 escape flag).
//   unpacked = the section stream of the save writer 0x566050 (the payload of the port's FUELPSAV file).
//
// The XLive-protected copy of the CRC cannot be checked without Games for Windows - LIVE: the import skips it and
// checks the body's own '=' marker and CRC, as 0x4eb9c0 does after the read [port].
#pragma once

#include <cstddef>
#include <cstdint>
#include <string>
#include <vector>

namespace fuel::pcsave {

// SaveGamePc_Z / MemoryCardManager result codes the port uses (SaveGamePc_Z vf33 0x8403c0 *err).
enum Code : int {
    Ok = 0,
    NotFound = 7,      // 0x840190: CreateFileA failed
    Empty = 8,         // 0x840190: size 0 or a short read
    WriteFailed = 9,   // 0x840000: CreateFileA / WriteFile failed
    NoSpace = 0xe,     // 0x441320 TT 9045 [port: a write that failed with ENOSPC / EDQUOT; the PC writer never returns it]
    Corrupt = 0x17,    // 0x840190 protected-CRC mismatch, 0x4eb9c0 '=' marker or CRC mismatch
};

constexpr size_t kBodySize = 200000;     // 0x4ebea0 malloc(200000), g_SaveGame vt+0x3c(name, buf, 200000)
constexpr size_t kMarkerAt = 0x30d3b;    // '=' (0x3d)
constexpr size_t kCrcAt = 0x30d3c;       // CRC of bytes [0, 0x30d3c)

// 0x4ebea0 / 0x4eb9c0 CRC (lower-cased bytes, MSB-first table of poly 0x04C11DB7, init 0).
uint32_t saveCrc(const uint8_t* p, size_t n);

// ModelOrder1C pack / unpack of a raw buffer (0x7597e0 / 0x759a50). unpack returns false on a blob that does not hold
// what its header says (the port bounds the decoder; FUEL trusts the CRC checked before).
std::vector<uint8_t> pack(const std::vector<uint8_t>& raw);
bool unpack(const uint8_t* blob, size_t avail, std::vector<uint8_t>* raw, std::string* error = nullptr);

// The 200000-byte body around a section stream (0x4ebea0): false when the packed blob does not fit (FUEL skips the save).
bool makeBody(const std::vector<uint8_t>& sections, std::vector<uint8_t>* body);
// 0x4eb9c0: Ok when body[0x30d3b] == '=' and the stored CRC matches, else Corrupt.
int checkBody(const std::vector<uint8_t>& body);

// 0x840190 without the XLive unprotect: Ok, NotFound, Empty or Corrupt; `body` gets the 200000 bytes after the blob.
// Opens `path` read-only.
int readFile(const std::string& path, std::vector<uint8_t>* body);
// 0x840000 for test fixtures [port]: the protected blob is stood in for by the 4 CRC bytes themselves (n = 4); the
// reader skips the blob either way.
int writeFile(const std::string& path, const std::vector<uint8_t>& body);

// The whole import: read (read-only), check, unpack -> FUEL's section stream. Returns a Code; `error` says why.
int importFile(const std::string& path, std::vector<uint8_t>* sections, std::string* error = nullptr);

}  // namespace fuel::pcsave
