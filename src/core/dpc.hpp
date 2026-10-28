// Reader for FUEL's .DPC "BigFile" containers.
//
// Layout (see docs/formats.md): a 0x800-byte header, sector-padded blocks of
// packed objects, then optionally a streaming manifest and a pool of
// sector-aligned object bodies. Every object has a 0x18-byte header, a
// class-specific link header and a body that may be LZ-compressed.
#pragma once

#include <cstdint>
#include <optional>
#include <string>
#include <unordered_map>
#include <vector>

namespace fuel {

// The only version string FUEL.exe accepts; files with any other version
// never finish loading in the original game.
inline constexpr const char* kDpcVersion = "v1.381.67.09 - Asobo Studio - Internal Cross Technology";

struct DpcObject {
    uint64_t offset = 0;  // file offset of the 0x18-byte object header
    uint32_t dataSize = 0;  // link + stored body
    uint32_t linkSize = 0;
    uint32_t decSize = 0;   // body size after decompression
    uint32_t compSize = 0;  // 0 = stored raw
    uint32_t classCrc = 0;
    uint32_t id = 0;        // resource ID = Asobo CRC of the source asset path
    bool inPool = false;

    bool hasBody() const { return decSize != 0 || compSize != 0; }
};

class DpcFile {
public:
    // One resource: the link header comes from the block copy, the body from
    // the block copy when it has one, otherwise from the first pool copy.
    struct Entry {
        uint32_t id = 0;
        uint32_t classCrc = 0;
        const DpcObject* header = nullptr;
        const DpcObject* body = nullptr;
    };

    explicit DpcFile(const std::string& path);  // throws std::runtime_error
    ~DpcFile();
    DpcFile(const DpcFile&) = delete;
    DpcFile& operator=(const DpcFile&) = delete;

    const std::string& path() const { return path_; }
    const std::string& version() const { return version_; }
    bool isCurrentVersion() const { return version_ == kDpcVersion; }

    const std::vector<DpcObject>& objects() const { return objects_; }
    const Entry* find(uint32_t id) const;
    // IDs of resources of one class that have a block header, in file order.
    std::vector<uint32_t> ids(uint32_t classCrc) const;

    std::vector<uint8_t> link(const Entry& e) const;
    // Decompressed body, or nullopt when the resource has none.
    std::optional<std::vector<uint8_t>> body(const Entry& e) const;
    std::optional<uint32_t> bodySize(const Entry& e) const;

private:
    void readAt(uint64_t offset, void* dst, size_t size) const;
    void parse();

    std::string path_;
    int fd_ = -1;
    uint64_t fileSize_ = 0;
    std::string version_;
    std::vector<DpcObject> objects_;
    std::vector<uint32_t> order_;
    std::unordered_map<uint32_t, Entry> entries_;
};

}  // namespace fuel
