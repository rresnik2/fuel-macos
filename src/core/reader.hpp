// Bounds-checked little-endian reader over a resource body or link header.
// Every read past the end throws std::runtime_error.
#pragma once

#include <cstdint>
#include <cstring>
#include <stdexcept>
#include <string>
#include <vector>

namespace fuel {

class ByteReader {
public:
    explicit ByteReader(const std::vector<uint8_t>& b, size_t offset = 0) : b_(b), o_(offset) {
        if (offset > b.size()) throw std::runtime_error("start offset " + std::to_string(offset) + " past the end");
    }

    size_t offset() const { return o_; }
    size_t size() const { return b_.size(); }
    size_t remaining() const { return b_.size() - o_; }

    const uint8_t* raw(size_t n) {
        if (n > b_.size() - o_) throw std::runtime_error("overrun reading " + std::to_string(n) + " bytes at " + std::to_string(o_));
        const uint8_t* p = b_.data() + o_;
        o_ += n;
        return p;
    }
    template <typename T>
    T get() {
        T v;
        std::memcpy(&v, raw(sizeof(T)), sizeof(T));
        return v;
    }
    uint32_t u32() { return get<uint32_t>(); }
    int32_t i32() { return get<int32_t>(); }
    uint16_t u16() { return get<uint16_t>(); }
    float f32() { return get<float>(); }

    uint32_t count(uint32_t limit = 5'000'000) {
        const uint32_t n = u32();
        if (n > limit) throw std::runtime_error("implausible count " + std::to_string(n) + " at " + std::to_string(o_ - 4));
        return n;
    }
    // u32 count followed by count * elemSize bytes; returns the count.
    uint32_t skipArray(size_t elemSize, uint32_t limit = 5'000'000) {
        const uint32_t n = count(limit);
        raw(size_t(n) * elemSize);
        return n;
    }
    // u32 count followed by that many u32 values.
    std::vector<uint32_t> u32Array(uint32_t limit = 5'000'000) {
        std::vector<uint32_t> v(count(limit));
        if (!v.empty()) std::memcpy(v.data(), raw(v.size() * 4), v.size() * 4);
        return v;
    }
    void expectEnd() const {
        if (o_ != b_.size()) throw std::runtime_error("parse ended at " + std::to_string(o_) + " of " + std::to_string(b_.size()));
    }

private:
    const std::vector<uint8_t>& b_;
    size_t o_ = 0;
};

}  // namespace fuel
