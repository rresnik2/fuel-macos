// The u16 cell boxes FUEL's terraform items carry (roads, gulches, flat surfaces, work bumps):
// cells of 2.4420025 m over x, z in [-80000, 80000). Decoded from FUEL.exe:
//   q(v)            0x659180 / 0x798f70: floor(f32((v + 80000) * 0.4095)) (round-down FPU, p21 H0)
//   deq(u)          u * 2.4420025 - 80000
//   init / extend   0x659670 / 0x659370 / 0x659400: min = q(p) when deq(min) > p, max = q(p) + 1
//                   when deq(max) < p
//   segment box     0x6f0ef0: A, B each +-w(d + n), +-w(d - n), n = (dz, -dx)
//   box overlap     0x6f0c40: x + r >= deq(minX), x - r <= deq(maxX) (same for z)
//   grid query      0x7a9e60: item box overlaps [q(p - r), q(p + r)] (inclusive)
// See ~/re-work/fuel/notes/p34-terrain-leftovers.md.
#pragma once

#include <cmath>
#include <cstdint>
#include <limits>

namespace fuel {

struct CellBox {
    int32_t minX = 0, minZ = 0, maxX = 0, maxZ = 0;
};

namespace terra {

// float32 store under the engine's round-down FPU mode.
inline double f32down(double v) {
    float f = float(v);
    if (double(f) > v) f = std::nextafter(f, -std::numeric_limits<float>::infinity());
    return double(f);
}
inline int32_t q(double v) { return int32_t(std::floor(f32down((v + 80000.0) * double(0.4095f)))); }
inline double deq(int32_t u) { return double(u) * double(2.4420025f) - 80000.0; }

inline CellBox boxAt(double x, double z) { return {q(x), q(z), q(x) + 1, q(z) + 1}; }
inline void extend(CellBox& b, double x, double z) {
    if (deq(b.minX) > x) b.minX = q(x);
    if (deq(b.minZ) > z) b.minZ = q(z);
    if (deq(b.maxX) < x) b.maxX = q(x) + 1;
    if (deq(b.maxZ) < z) b.maxZ = q(z) + 1;
}
// 0x6f0ef0 for a capsule A -> B of half width w and unit direction (dx, dz).
inline CellBox segmentBox(double ax, double az, double bx, double bz, double dx, double dz, double w) {
    CellBox b = boxAt(ax, az);
    const double px = dx + dz, pz = dz - dx;  // d + n
    const double mx = dx - dz, mz = dz + dx;  // d - n
    auto four = [&](double ox, double oz) {
        extend(b, ox + px * w, oz + pz * w);
        extend(b, ox + mx * w, oz + mz * w);
        extend(b, ox - mx * w, oz - mz * w);
        extend(b, ox - px * w, oz - pz * w);
    };
    four(ax, az);
    four(bx, bz);
    return b;
}
// 0x6f0c40.
inline bool overlaps(const CellBox& b, double x, double z, double r) {
    return x + r >= deq(b.minX) && x - r <= deq(b.maxX) && z + r >= deq(b.minZ) && z - r <= deq(b.maxZ);
}
// 0x7a9e60 box test against the query box p +- r.
inline bool overlapsQuery(const CellBox& b, double x, double z, double r) {
    const int32_t x0 = q(f32down(x - r)), z0 = q(f32down(z - r)), x1 = q(f32down(x + r)), z1 = q(f32down(z + r));
    return b.minX <= x1 && b.minZ <= z1 && x0 <= b.maxX && z0 <= b.maxZ;
}
// 0x798ef0: the cell of (x, z) inside the box.
inline bool holds(const CellBox& b, double x, double z) {
    const int32_t cx = q(x), cz = q(z);
    return b.minX <= cx && cx <= b.maxX && b.minZ <= cz && cz <= b.maxZ;
}

}  // namespace terra
}  // namespace fuel
