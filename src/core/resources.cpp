#include "resources.hpp"

#include "reader.hpp"

#include <algorithm>
#include <cmath>
#include <cstring>
#include <stdexcept>

namespace fuel {

namespace {

template <typename T>
T rd(const uint8_t* p) {
    T v;
    std::memcpy(&v, p, sizeof(T));
    return v;
}

struct FormatDesc {
    PixelFormat format;
    uint32_t bitsPerPixel;
};

FormatDesc formatForCode(uint8_t code) {
    switch (code) {
        case 0x07: return {PixelFormat::A1R5G5B5, 16};
        case 0x08: return {PixelFormat::R5G6B5, 16};
        case 0x0A: return {PixelFormat::A4R4G4B4, 16};
        case 0x0C: return {PixelFormat::A8R8G8B8, 32};
        case 0x0D: return {PixelFormat::R8G8B8, 24};
        case 0x0E: return {PixelFormat::DXT1, 4};
        case 0x0F: return {PixelFormat::DXT3, 8};
        case 0x10:
        case 0x11: return {PixelFormat::DXT5, 8};
        case 0x14: return {PixelFormat::R32F, 32};
        case 0x17: return {PixelFormat::RGBA16F, 64};
        case 0x18: return {PixelFormat::RGBA32F, 128};
        case 0x1A: return {PixelFormat::L8, 8};
        default: return {PixelFormat::Unknown, 0};
    }
}

bool isPow2(uint32_t v) { return v != 0 && (v & (v - 1)) == 0; }

}  // namespace

uint32_t asoboCrc(const std::string& s) {
    static const auto table = [] {
        std::array<uint32_t, 256> t{};
        for (uint32_t i = 0; i < 256; ++i) {
            uint32_t c = i << 24;
            for (int k = 0; k < 8; ++k) c = (c & 0x80000000u) ? (c << 1) ^ 0x04C11DB7u : c << 1;
            t[i] = c;
        }
        return t;
    }();
    uint32_t crc = 0;
    for (unsigned char ch : s) {
        const unsigned char lower = (ch >= 'A' && ch <= 'Z') ? ch + 32 : ch;
        crc = (crc >> 8) ^ table[(lower ^ crc) & 0xFF];
    }
    return crc;
}

// ---- Bitmap_Z ---------------------------------------------------------------

const char* pixelFormatName(PixelFormat f) {
    switch (f) {
        case PixelFormat::A1R5G5B5: return "A1R5G5B5";
        case PixelFormat::R5G6B5: return "R5G6B5";
        case PixelFormat::A4R4G4B4: return "A4R4G4B4";
        case PixelFormat::A8R8G8B8: return "A8R8G8B8";
        case PixelFormat::R8G8B8: return "R8G8B8";
        case PixelFormat::DXT1: return "DXT1";
        case PixelFormat::DXT3: return "DXT3";
        case PixelFormat::DXT5: return "DXT5";
        case PixelFormat::R32F: return "R32F";
        case PixelFormat::RGBA16F: return "A16B16G16R16F";
        case PixelFormat::RGBA32F: return "A32B32G32R32F";
        case PixelFormat::L8: return "L8";
        case PixelFormat::Unknown: break;
    }
    return "unknown";
}

bool BitmapInfo::isBlockCompressed() const {
    return format == PixelFormat::DXT1 || format == PixelFormat::DXT3 || format == PixelFormat::DXT5;
}

std::vector<uint32_t> BitmapInfo::mipSizes() const {
    const uint32_t bpp = formatForCode(formatCode).bitsPerPixel;
    std::vector<uint32_t> sizes;
    uint32_t w = width, h = height;
    for (uint32_t i = 0; i < mips; ++i) {
        if (isBlockCompressed())
            sizes.push_back(std::max(1u, (w + 3) / 4) * std::max(1u, (h + 3) / 4) * (format == PixelFormat::DXT1 ? 8 : 16));
        else
            sizes.push_back(std::max(1u, w) * std::max(1u, h) * bpp / 8);
        w = std::max(1u, w >> 1);
        h = std::max(1u, h >> 1);
    }
    return sizes;
}

uint32_t BitmapInfo::expectedBodySize() const {
    uint32_t total = 0;
    for (uint32_t s : mipSizes()) total += s;
    return total * faces();
}

BitmapInfo parseBitmapLink(const std::vector<uint8_t>& link) {
    if (link.size() < 32) throw std::runtime_error("old/short Bitmap_Z link (" + std::to_string(link.size()) + " bytes)");
    BitmapInfo bm;
    bm.type = rd<uint16_t>(&link[4]);
    bm.width = rd<uint32_t>(&link[6]);
    bm.height = rd<uint32_t>(&link[10]);
    bm.size = rd<uint32_t>(&link[14]);
    bm.mips = uint32_t(link[0x1B]) + 1;
    bm.formatCode = link[0x1E];
    bm.format = formatForCode(bm.formatCode).format;
    bm.alphaKind = link[0x1F];
    return bm;
}

std::vector<std::string> checkBitmap(const BitmapInfo& bm, uint32_t bodySize) {
    std::vector<std::string> p;
    if (bm.format == PixelFormat::Unknown) return {"unknown format code " + std::to_string(bm.formatCode)};
    if (bm.type != BitmapInfo::Tex2D && bm.type != BitmapInfo::Cube) p.push_back("texture type " + std::to_string(bm.type));
    if (!isPow2(bm.width) || !isPow2(bm.height)) p.push_back("dimension not a power of two");
    if (bodySize != bm.expectedBodySize()) p.push_back("body size does not match the mip chain");
    if (bm.size != 0 && bm.size != bodySize) p.push_back("link size does not match the body");
    return p;
}

// ---- Material_Z -------------------------------------------------------------

Material parseMaterialBody(const std::vector<uint8_t>& b) {
    if (b.size() != 0xAC) throw std::runtime_error("Material_Z body " + std::to_string(b.size()) + " bytes, expected 172");
    Material m;
    for (int i = 0; i < 3; ++i) m.diffuse[size_t(i)] = rd<float>(&b[size_t(i) * 4]);
    m.alpha = rd<float>(&b[0x0C]);
    for (size_t i = 0; i < 3; ++i) m.emissive[i] = rd<float>(&b[0x10 + i * 4]);
    for (size_t i = 0; i < 4; ++i) m.specular[i] = rd<float>(&b[0x50 + i * 4]);
    m.fresnel = {rd<float>(&b[0x80]), rd<float>(&b[0x84])};
    m.textureMask = rd<uint16_t>(&b[0x38]);
    m.flags = rd<uint32_t>(&b[0x3C]);
    for (size_t i = 0; i < 9; ++i) m.textures[i] = rd<uint32_t>(&b[0x88 + i * 4]);
    return m;
}

// ---- Mesh_Z -----------------------------------------------------------------

MeshLink parseMeshLink(const std::vector<uint8_t>& link) {
    if (link.size() < 98) throw std::runtime_error("short Mesh_Z link (" + std::to_string(link.size()) + " bytes)");
    MeshLink l;
    l.dataId = rd<uint32_t>(&link[4]);
    for (size_t i = 0; i < 4; ++i) l.sphere[i] = rd<float>(&link[8 + i * 4]);
    l.flags = rd<uint32_t>(&link[92]);
    return l;
}

Mesh parseMeshBody(const std::vector<uint8_t>& body, uint32_t linkFlags) {
    ByteReader r(body);
    Mesh m;
    // Point arrays: positions, plus three per-point attribute arrays.
    r.skipArray(12);
    r.skipArray(16);
    r.skipArray(8);
    r.skipArray(12);
    // Groups: u32 n; u16[n]; u32; u32. An extra u32 each when link flag 2 is set.
    const uint32_t groups = r.count();
    for (uint32_t i = 0; i < groups; ++i) {
        r.skipArray(2);
        r.u32();
        r.u32();
    }
    if (linkFlags & 2) r.raw(size_t(groups) * 4);
    const uint32_t groups2 = r.count();
    for (uint32_t i = 0; i < groups2; ++i) r.skipArray(8);

    const uint32_t nm = r.count();
    m.materials.resize(nm);
    for (uint32_t i = 0; i < nm; ++i) m.materials[i] = r.u32();
    r.skipArray(0x20);
    r.skipArray(8);
    r.skipArray(0x20);

    const uint32_t nvb = r.count();
    for (uint32_t i = 0; i < nvb; ++i) {
        VertexBuffer vb;
        vb.count = r.count();
        vb.stride = r.u32();
        vb.flags = r.u32();
        if (vb.stride == 0 || vb.stride > 256) throw std::runtime_error("implausible vertex stride " + std::to_string(vb.stride));
        const uint8_t* p = r.raw(size_t(vb.count) * vb.stride);
        vb.data.assign(p, p + size_t(vb.count) * vb.stride);
        m.vbs.push_back(std::move(vb));
    }
    const uint32_t nib = r.count();
    for (uint32_t i = 0; i < nib; ++i) {
        IndexBuffer ib;
        ib.count = r.count();
        ib.flags = r.u32();
        const uint8_t* p = r.raw(size_t(ib.count) * 2);
        ib.data.resize(ib.count);
        std::memcpy(ib.data.data(), p, size_t(ib.count) * 2);
        m.ibs.push_back(std::move(ib));
    }
    r.skipArray(0x3C);

    const uint32_t nb = r.count();
    for (uint32_t i = 0; i < nb; ++i) {
        const uint8_t* h = r.raw(0x30);
        r.skipArray(0x1C);
        Batch b;
        b.vb = rd<uint32_t>(h + 0x00);
        b.ib = rd<uint32_t>(h + 0x04);
        b.primitiveType = rd<uint16_t>(h + 0x10);
        b.vertexType = rd<uint16_t>(h + 0x12);
        b.minVertex = rd<uint16_t>(h + 0x14);
        b.numVertices = rd<uint32_t>(h + 0x18);
        b.startIndex = rd<uint32_t>(h + 0x1C);
        b.primitiveCount = rd<uint32_t>(h + 0x20);
        b.baseVertex = rd<uint32_t>(h + 0x24);
        b.startVertex = rd<uint32_t>(h + 0x28);
        b.stride = rd<uint16_t>(h + 0x2C);
        b.material = rd<uint16_t>(h + 0x2E);
        m.batches.push_back(b);
    }
    r.skipArray(0x20);
    r.skipArray(4);
    r.skipArray(2);
    // Morph targets: name, then per-target tables.
    const uint32_t nnamed = r.count();
    for (uint32_t i = 0; i < nnamed; ++i) {
        const uint32_t len = r.count(4096);
        const char* s = reinterpret_cast<const char*>(r.raw(len));
        m.morphTargets.emplace_back(s, strnlen(s, len));
        r.u32();
        r.u16();
        r.skipArray(2);
        r.skipArray(8);
    }
    r.skipArray(6);
    if (r.offset() != r.size())
        throw std::runtime_error("parse ended at " + std::to_string(r.offset()) + " of " + std::to_string(r.size()));
    return m;
}

std::vector<uint32_t> batchTriangles(const Mesh& m, const Batch& b) {
    const IndexBuffer* ib = b.ib < m.ibs.size() ? &m.ibs[b.ib] : nullptr;
    uint32_t cnt;
    if (b.primitiveType == 4)
        cnt = b.primitiveCount * 3;
    else if (b.primitiveType == 5)
        cnt = b.primitiveCount + 2;
    else
        throw std::runtime_error("primitive type " + std::to_string(b.primitiveType) + " not handled");
    std::vector<uint32_t> idx(cnt);
    if (ib == nullptr) {
        for (uint32_t i = 0; i < cnt; ++i) idx[i] = b.startVertex + i;
    } else {
        if (uint64_t(b.startIndex) + cnt > ib->count) throw std::runtime_error("index range past the index buffer");
        for (uint32_t i = 0; i < cnt; ++i) idx[i] = ib->data[b.startIndex + i] + b.baseVertex;
    }
    if (b.primitiveType == 4) return idx;
    std::vector<uint32_t> tris;
    for (uint32_t k = 0; k < b.primitiveCount; ++k) {
        const uint32_t a = idx[k], c = idx[k + 1], d = idx[k + 2];
        if (a == c || c == d || a == d) continue;
        if (k % 2 == 0)
            tris.insert(tris.end(), {a, c, d});
        else
            tris.insert(tris.end(), {c, a, d});
    }
    return tris;
}

std::vector<std::string> checkMesh(const Mesh& m, MeshStats* stats) {
    std::vector<std::string> p;
    MeshStats st;
    st.boundsMin = {1e30f, 1e30f, 1e30f};
    st.boundsMax = {-1e30f, -1e30f, -1e30f};
    for (const VertexBuffer& vb : m.vbs) {
        if (vb.stride < 12) continue;
        for (uint32_t i = 0; i < vb.count; ++i) {
            float x[3];
            std::memcpy(x, &vb.data[size_t(i) * vb.stride], 12);
            for (size_t k = 0; k < 3; ++k) {
                st.boundsMin[k] = std::min(st.boundsMin[k], x[k]);
                st.boundsMax[k] = std::max(st.boundsMax[k], x[k]);
            }
        }
    }
    if (st.boundsMin[0] <= st.boundsMax[0]) {
        for (size_t k = 0; k < 3; ++k)
            if (!std::isfinite(st.boundsMin[k]) || !std::isfinite(st.boundsMax[k]) || std::fabs(st.boundsMin[k]) > 1e6f ||
                std::fabs(st.boundsMax[k]) > 1e6f) {
                p.push_back("implausible bounding box");
                break;
            }
    }
    for (size_t bi = 0; bi < m.batches.size(); ++bi) {
        const Batch& b = m.batches[bi];
        const std::string tag = "batch " + std::to_string(bi) + ": ";
        if (b.vb >= m.vbs.size()) {
            p.push_back(tag + "VB index out of range");
            continue;
        }
        const VertexBuffer& vb = m.vbs[b.vb];
        if (b.stride != vb.stride) p.push_back(tag + "stride does not match its VB");
        if (b.material != 0xFFFF && b.material >= std::max<size_t>(1, m.materials.size())) p.push_back(tag + "material out of range");
        try {
            const std::vector<uint32_t> tris = batchTriangles(m, b);
            const uint64_t lo = uint64_t(b.baseVertex) + b.minVertex;
            for (uint32_t v : tris) {
                if (v >= vb.count) throw std::runtime_error("vertex past the VB");
                if (v < lo || v >= lo + b.numVertices) throw std::runtime_error("vertex outside the batch range");
            }
            st.triangles += uint32_t(tris.size() / 3);
        } catch (const std::runtime_error& e) {
            p.push_back(tag + e.what());
        }
    }
    if (stats) *stats = st;
    return p;
}

}  // namespace fuel
