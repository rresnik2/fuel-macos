#include "dpc.hpp"

#include "lz.hpp"

#include <fcntl.h>
#include <sys/stat.h>
#include <unistd.h>

#include <cstring>
#include <stdexcept>

namespace fuel {

namespace {

constexpr uint32_t kSector = 0x800;
constexpr uint32_t kObjectHeaderSize = 0x18;
constexpr uint32_t kMaxBlocks = 64;

uint32_t rd32(const uint8_t* p) {
    uint32_t v;
    std::memcpy(&v, p, 4);
    return v;  // little-endian host
}

std::runtime_error dpcError(const std::string& path, const std::string& what) {
    return std::runtime_error(path + ": " + what);
}

}  // namespace

DpcFile::DpcFile(const std::string& path) : path_(path) {
    fd_ = ::open(path.c_str(), O_RDONLY);
    if (fd_ < 0) throw dpcError(path, "cannot open");
    struct stat st;
    if (::fstat(fd_, &st) != 0) {
        ::close(fd_);
        throw dpcError(path, "cannot stat");
    }
    fileSize_ = uint64_t(st.st_size);
    try {
        parse();
    } catch (...) {
        ::close(fd_);
        fd_ = -1;
        throw;
    }
}

DpcFile::~DpcFile() {
    if (fd_ >= 0) ::close(fd_);
}

void DpcFile::readAt(uint64_t offset, void* dst, size_t size) const {
    auto* out = static_cast<uint8_t*>(dst);
    while (size > 0) {
        const ssize_t n = ::pread(fd_, out, size, off_t(offset));
        if (n <= 0) throw dpcError(path_, "read past end of file");
        out += n;
        offset += uint64_t(n);
        size -= size_t(n);
    }
}

void DpcFile::parse() {
    if (fileSize_ < kSector) throw dpcError(path_, "file shorter than header");
    std::vector<uint8_t> h(kSector);
    readAt(0, h.data(), kSector);

    version_.assign(reinterpret_cast<const char*>(h.data()), strnlen(reinterpret_cast<const char*>(h.data()), 0x100));
    if (version_.rfind("v1.", 0) != 0 || version_.find("Asobo") == std::string::npos)
        throw dpcError(path_, "not a DPC file (bad version string)");

    const uint32_t blockCount = rd32(&h[0x104]);
    if (blockCount > kMaxBlocks) throw dpcError(path_, "too many blocks");
    const uint32_t manifestSectors = rd32(&h[0x720]);
    const uint32_t manifestSector = rd32(&h[0x724]);

    // Blocks: objects packed back to back, starting at sector 1.
    uint64_t pos = kSector;
    std::vector<uint8_t> block;
    for (uint32_t bi = 0; bi < blockCount; ++bi) {
        const uint8_t* d = &h[0x120 + bi * 0x18];
        const uint32_t objectCount = rd32(d + 0x00);
        const uint32_t paddedSize = rd32(d + 0x04);
        const uint32_t dataSize = rd32(d + 0x08);
        if (dataSize > paddedSize) throw dpcError(path_, "block data larger than block");
        block.resize(dataSize);
        readAt(pos, block.data(), dataSize);
        uint32_t o = 0;
        for (uint32_t oi = 0; oi < objectCount; ++oi) {
            if (o + kObjectHeaderSize > dataSize) throw dpcError(path_, "object header past block end");
            const uint8_t* oh = &block[o];
            DpcObject obj;
            obj.offset = pos + o;
            obj.dataSize = rd32(oh + 0x00);
            obj.linkSize = rd32(oh + 0x04);
            obj.decSize = rd32(oh + 0x08);
            obj.compSize = rd32(oh + 0x0C);
            obj.classCrc = rd32(oh + 0x10);
            obj.id = rd32(oh + 0x14);
            if (uint64_t(o) + kObjectHeaderSize + obj.dataSize > dataSize)
                throw dpcError(path_, "object body past block end");
            objects_.push_back(obj);
            o += kObjectHeaderSize + obj.dataSize;
        }
        if (o != dataSize) throw dpcError(path_, "block object walk does not end at the block size");
        pos += paddedSize;
    }

    // Pool: one sector-aligned object per manifest entry, up to EOF.
    if (manifestSectors != 0) {
        uint64_t ppos = uint64_t(manifestSector) * kSector + uint64_t(manifestSectors) * kSector;
        uint8_t oh[kObjectHeaderSize];
        while (ppos < fileSize_) {
            readAt(ppos, oh, kObjectHeaderSize);
            DpcObject obj;
            obj.offset = ppos;
            obj.dataSize = rd32(oh + 0x00);
            obj.linkSize = rd32(oh + 0x04);
            obj.decSize = rd32(oh + 0x08);
            obj.compSize = rd32(oh + 0x0C);
            obj.classCrc = rd32(oh + 0x10);
            obj.id = rd32(oh + 0x14);
            obj.inPool = true;
            objects_.push_back(obj);
            ppos = (ppos + kObjectHeaderSize + obj.dataSize + kSector - 1) & ~uint64_t(kSector - 1);
        }
        if (ppos != fileSize_) throw dpcError(path_, "pool does not end at EOF");
    }

    // objects_ is final, so pointers into it stay valid.
    for (const DpcObject& obj : objects_) {
        auto [it, inserted] = entries_.try_emplace(obj.id);
        Entry& e = it->second;
        if (inserted) {
            e.id = obj.id;
            e.classCrc = obj.classCrc;
            order_.push_back(obj.id);
        }
        if (!obj.inPool) e.header = &obj;
        if (obj.hasBody() && e.body == nullptr) e.body = &obj;
    }
}

const DpcFile::Entry* DpcFile::find(uint32_t id) const {
    auto it = entries_.find(id);
    return it == entries_.end() ? nullptr : &it->second;
}

std::vector<uint32_t> DpcFile::ids(uint32_t classCrc) const {
    std::vector<uint32_t> out;
    for (uint32_t id : order_) {
        const Entry& e = entries_.at(id);
        if (e.classCrc == classCrc && e.header != nullptr) out.push_back(id);
    }
    return out;
}

std::vector<uint8_t> DpcFile::link(const Entry& e) const {
    if (e.header == nullptr) return {};
    std::vector<uint8_t> out(e.header->linkSize);
    readAt(e.header->offset + kObjectHeaderSize, out.data(), out.size());
    return out;
}

std::optional<uint32_t> DpcFile::bodySize(const Entry& e) const {
    if (e.body == nullptr) return std::nullopt;
    return e.body->decSize;
}

std::optional<std::vector<uint8_t>> DpcFile::body(const Entry& e) const {
    const DpcObject* b = e.body;
    if (b == nullptr) return std::nullopt;
    if (b->linkSize > b->dataSize) throw dpcError(path_, "link larger than object");
    std::vector<uint8_t> stored(b->dataSize - b->linkSize);
    readAt(b->offset + kObjectHeaderSize + b->linkSize, stored.data(), stored.size());
    if (b->compSize != 0) return lzDecompress(stored.data(), stored.size(), b->decSize);
    if (stored.size() < b->decSize) throw dpcError(path_, "raw body shorter than its size");
    stored.resize(b->decSize);
    return stored;
}

}  // namespace fuel
