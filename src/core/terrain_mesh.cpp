#include "terrain_mesh.hpp"
#include "terrain_height.hpp"

#include <algorithm>
#include <array>
#include <cmath>

namespace fuel {

namespace {

constexpr double kRoot = 12288.0;       // root chunk size (p15 D2)
constexpr double kSplit = 2.0625;       // W+0xa88: split while the camera is within kSplit * C
constexpr double kLeaf = 24.01;         // 0x9dc4f4: chunks this small never split
constexpr double kFarChunk = 240.0;     // 0x7f92c0: chunks >= 240 m use the far height (0x64be70)
constexpr double kMapNormalChunk = 120.0;  // chunks >= 120 m take the map normal (0x64be70)

uint64_t chunkKey(int level, double cx, double cz) {
    const auto ix = uint64_t(int64_t(std::llround(cx)) + (int64_t(1) << 27)) & ((uint64_t(1) << 28) - 1);
    const auto iz = uint64_t(int64_t(std::llround(cz)) + (int64_t(1) << 27)) & ((uint64_t(1) << 28) - 1);
    return (uint64_t(level) << 56) | (ix << 28) | iz;
}

}  // namespace

std::vector<TerrainVertex> buildTerrainChunk(const Heightfield& hf, double cx, double cz, double size) {
    constexpr int N = kChunkSide;
    const double s = size / 24.0, x0 = cx - size * 0.5, z0 = cz - size * 0.5;
    const TerrainDetail* d = hf.detail.get();
    // Heights with a border (0x64dfe0 evaluates the neighbours outside the patch): one point for
    // the fine normals, two for the coarse normals of chunks under 60 m.
    const bool coarse2 = size < 60.0;
    const int B = coarse2 ? 2 : 1, G = N + 2 * B;
    std::vector<double> h(size_t(G) * G);
    std::vector<std::array<float, 3>> mapN(size_t(N) * N, {0.f, 1.f, 0.f});
    const bool far = size >= kFarChunk, mapNormal = size >= kMapNormalChunk;
    for (int j = -B; j < N + B; ++j)
        for (int i = -B; i < N + B; ++i) {
            const bool inside = i >= 0 && j >= 0 && i < N && j < N;
            if ((far || mapNormal) && !inside) continue;
            const double x = x0 + i * s, z = z0 + j * s;
            double y;
            if (!d) {
                y = hf.baseHeightAt(x, z);
            } else if (far) {
                float n[3];
                y = d->farHeight(x, z, n);
                mapN[size_t(j) * N + size_t(i)] = {n[0], n[1], n[2]};
            } else {
                y = d->height(x, z);
                if (mapNormal && inside) {
                    float n[3];
                    d->baseNormal(x, z, n);
                    mapN[size_t(j) * N + size_t(i)] = {n[0], n[1], n[2]};
                }
            }
            h[size_t(j + B) * G + size_t(i + B)] = y;
        }
    auto H = [&](int i, int j) { return h[size_t(j + B) * G + size_t(i + B)]; };
    auto setN = [](float* o, double nx, double ny, double nz) {
        const double l = std::sqrt(nx * nx + ny * ny + nz * nz);
        o[0] = float(nx / l), o[1] = float(ny / l), o[2] = float(nz / l);
    };
    const float tweenStart = float((size * kSplit * 2 - size) - size * 0.5 * 1.5), tweenInv = float(1.0 / (size * 0.5));
    std::vector<TerrainVertex> v(size_t(N) * N + 4 * N);
    for (int j = 0; j < N; ++j)
        for (int i = 0; i < N; ++i) {
            TerrainVertex& t = v[size_t(j) * N + size_t(i)];
            t.p0[0] = float(x0 + i * s), t.p0[1] = float(H(i, j)), t.p0[2] = float(z0 + j * s);
            t.tweenStart = tweenStart, t.tweenInv = tweenInv, t.size = float(size);
            t.pad = 0;
            if (mapNormal || !d) {
                const auto& n = mapN[size_t(j) * N + size_t(i)];
                if (d) {
                    t.n[0] = n[0], t.n[1] = n[1], t.n[2] = n[2];
                } else {
                    t.n[0] = 0, t.n[1] = 1, t.n[2] = 0;
                }
            } else {  // central differences over the neighbour points (0x7f92c0)
                setN(t.n, H(i - 1, j) - H(i + 1, j), 2 * s, H(i, j - 1) - H(i, j + 1));
            }
        }
    // Morph targets (0x7f9b50, asm 0x7f9e30-0x7fa2a8, notes p31 E6): vertex (i, j) morphs to the
    // even point (i & ~1, j & ~1) as the parent chunk (2C) builds it, so odd vertices collapse onto
    // the lower-left even vertex and at full tween every 2 x 2 block is the parent's quad (with the
    // (i+1, j)-(i, j+1) diagonal of terrainChunkIndices). The target height is that point's +0x30:
    // its own height for C < 120 and C >= 240, the far height 0x64be70 for 120 <= C < 240 (what the
    // parent, C >= 240, draws). The target normal is its coarse normal +0x20: central differences
    // over two steps for C < 60, the map normal from 60 m up. Root chunks (chunk +0xa8 = 1) never morph.
    const bool root = size >= kRoot;
    for (int j = 0; j < N; j += 2)
        for (int i = 0; i < N; i += 2) {
            TerrainVertex& e = v[size_t(j) * N + size_t(i)];
            float p1[3] = {e.p0[0], e.p0[1], e.p0[2]}, n1[3] = {0, 1, 0};
            if (coarse2) {
                setN(n1, H(i - 2, j) - H(i + 2, j), 4 * s, H(i, j - 2) - H(i, j + 2));
            } else if (d && mapNormal && !far) {
                p1[1] = float(d->farHeight(e.p0[0], e.p0[2], n1));
            } else if (d && !mapNormal) {
                d->baseNormal(e.p0[0], e.p0[2], n1);
            } else {
                n1[0] = e.n[0], n1[1] = e.n[1], n1[2] = e.n[2];
            }
            if (root) {
                p1[1] = e.p0[1];
                n1[0] = e.n[0], n1[1] = e.n[1], n1[2] = e.n[2];
            }
            for (int dj = 0; dj < 2 && j + dj < N; ++dj)
                for (int di = 0; di < 2 && i + di < N; ++di) {
                    TerrainVertex& t = v[size_t(j + dj) * N + size_t(i + di)];
                    if (root) {
                        std::copy(t.p0, t.p0 + 3, t.p1);
                        std::copy(t.n, t.n + 3, t.n1);
                    } else {
                        std::copy(p1, p1 + 3, t.p1);
                        std::copy(n1, n1 + 3, t.n1);
                    }
                }
        }
    // Skirts (port addition against cracks where neighbouring chunks differ in size).
    const float drop = float(1.0 + 0.05 * size);
    size_t o = size_t(N) * N;
    auto skirt = [&](int i, int j) {
        TerrainVertex t = v[size_t(j) * N + size_t(i)];
        t.p0[1] -= drop, t.p1[1] -= drop;
        v[o++] = t;
    };
    for (int i = 0; i < N; ++i) skirt(i, 0);
    for (int i = 0; i < N; ++i) skirt(i, N - 1);
    for (int j = 0; j < N; ++j) skirt(0, j);
    for (int j = 0; j < N; ++j) skirt(N - 1, j);
    return v;
}

const std::vector<uint32_t>& terrainChunkIndices() {
    static const std::vector<uint32_t> idx = [] {
        constexpr uint32_t N = kChunkSide;
        std::vector<uint32_t> r;
        for (uint32_t j = 0; j + 1 < N; ++j)
            for (uint32_t i = 0; i + 1 < N; ++i) {
                const uint32_t a = j * N + i, b = a + 1, c = a + N, d = c + 1;
                r.insert(r.end(), {a, b, c, b, d, c});  // FUEL's (k, k+1, k+25), (k+1, k+26, k+25): diagonal (i+1, j)-(i, j+1)
            }
        const uint32_t base = N * N;
        auto side = [&](uint32_t sk, auto edge) {
            for (uint32_t k = 0; k + 1 < N; ++k) {
                const uint32_t e0 = edge(k), e1 = edge(k + 1), s0 = base + sk * N + k, s1 = s0 + 1;
                r.insert(r.end(), {e0, e1, s1, e0, s1, s0});
            }
        };
        side(0, [](uint32_t k) { return k; });
        side(1, [](uint32_t k) { return (N - 1) * N + k; });
        side(2, [](uint32_t k) { return k * N; });
        side(3, [](uint32_t k) { return k * N + N - 1; });
        return r;
    }();
    return idx;
}

TerrainLod::TerrainLod(const Heightfield& hf, int threads) : hf_(hf) {
    for (int t = 0; t < threads; ++t) threads_.emplace_back([this] { worker(); });
}

TerrainLod::~TerrainLod() {
    {
        std::lock_guard<std::mutex> lk(mu_);
        stop_ = true;
        queue_.clear();
    }
    cv_.notify_all();
    for (auto& t : threads_) t.join();
}

void TerrainLod::worker() {
    for (;;) {
        Job job;
        {
            std::unique_lock<std::mutex> lk(mu_);
            cv_.wait(lk, [&] { return stop_ || !queue_.empty(); });
            if (stop_) return;
            job = queue_.front();
            queue_.pop_front();
            ++busy_;
        }
        auto c = std::make_shared<TerrainChunk>();
        c->key = job.key, c->cx = job.cx, c->cz = job.cz, c->size = job.size;
        c->verts = buildTerrainChunk(hf_, job.cx, job.cz, job.size);
        {
            std::lock_guard<std::mutex> lk(mu_);
            built_[job.key] = std::move(c);
            lastUsed_[job.key] = frame_;
            pending_.erase(job.key);
            --busy_;
        }
        idle_.notify_all();
    }
}

bool TerrainLod::cover(double cx, double cz, double size, int level, double ex, double ez,
                       std::vector<std::shared_ptr<const TerrainChunk>>& out, std::vector<Job>& want) {
    const uint64_t key = chunkKey(level, cx, cz);
    const double dist = std::hypot(ex - cx, ez - cz);
    const bool split = size > kLeaf && dist < size * kSplit;
    if (split) {
        std::vector<std::shared_ptr<const TerrainChunk>> kids;
        bool all = true;
        const double q = size * 0.25;
        for (int k = 0; k < 4; ++k)
            all = cover(cx + ((k & 1) ? q : -q), cz + ((k & 2) ? q : -q), size * 0.5, level + 1, ex, ez, kids, want) && all;
        if (all) {
            out.insert(out.end(), kids.begin(), kids.end());
            return true;
        }
    }
    const auto it = built_.find(key);
    if (it != built_.end()) {
        lastUsed_[key] = frame_;
        out.push_back(it->second);
        return true;
    }
    if (!pending_.count(key)) want.push_back({key, cx, cz, size, dist / size});
    return false;
}

std::vector<std::shared_ptr<const TerrainChunk>> TerrainLod::update(double ex, double ez) {
    std::vector<std::shared_ptr<const TerrainChunk>> out;
    std::vector<Job> want;
    {
        std::lock_guard<std::mutex> lk(mu_);
        ++frame_;
        // 9 x 9 roots of 12288 m around the camera, inside the ring i^2 + j^2 < 17 (0x7df0e0).
        const double rx = std::round(ex / kRoot), rz = std::round(ez / kRoot);
        for (int j = -4; j <= 4; ++j)
            for (int i = -4; i <= 4; ++i)
                if (i * i + j * j < 17) cover((rx + i) * kRoot, (rz + j) * kRoot, kRoot, 0, ex, ez, out, want);
        // Requeue: nearest first, relative to chunk size.
        std::sort(want.begin(), want.end(), [](const Job& a, const Job& b) { return a.priority < b.priority; });
        std::deque<Job> keep;
        for (const Job& j : queue_) pending_.erase(j.key);  // not started: re-decide
        queue_.clear();
        for (const Job& j : want) {
            if (pending_.count(j.key)) continue;
            pending_.insert(j.key);
            queue_.push_back(j);
        }
        // Forget chunks unused for a while.
        for (auto it = built_.begin(); it != built_.end();) {
            if (frame_ - lastUsed_[it->first] > 600) {
                lastUsed_.erase(it->first);
                it = built_.erase(it);
            } else {
                ++it;
            }
        }
    }
    cv_.notify_all();
    return out;
}

void TerrainLod::settle() {
    std::unique_lock<std::mutex> lk(mu_);
    idle_.wait(lk, [&] { return stop_ || (queue_.empty() && busy_ == 0); });
}

}  // namespace fuel
