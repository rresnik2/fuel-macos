#include "fuel_contacts.hpp"

#include <cmath>

namespace fuel::fv {

namespace {

// Direction of `v` as the engine normalises (only when longer than 0.001; else the raw vector).
Vec3 dir001(const Vec3& v) {
    const float l = len(v);
    return l > 0.001f ? v / l : v;
}

// One friction curve evaluation (shared by 0x720cc0's two parts and 0x7216c0). Returns F (>= 0) and
// writes Fn, den (c1 + lim) and whether the denominator was usable.
struct Curve {
    float F = 0, Fn = 0, den = 0;
    bool ok = true;
};
Curve curve(const float c[8], float v, float Nf, bool wheel) {
    Curve r;
    const float k = clampf(1.0f - 0.1f * v, 0, 1);
    const float b = k * c[5];
    float F = c[3] * v + c[4] * std::sqrt(v);
    const float Fn = F / Nf;
    const float lim = c[0] + b;
    if (Fn > lim) {
        const float w = c[1] <= 0.001f ? 1.0f : sstep((Fn - lim) / c[1]);
        const float Fs = std::min(F, (std::sqrt(std::fabs(F - lim * Nf) * c[7]) + b + c[2]) * Nf);
        F = (1 - w) * F + w * Fs;
    }
    if (c[6] > 1e-4f && wheel) F += c[6] * v;
    r.F = F;
    r.Fn = Fn;
    r.den = c[1] + lim;
    r.ok = r.den >= 1e-6f;
    return r;
}

}  // namespace

void integrateVelocity(BodyState& S, float h) {
    S.v += S.a * h;
    S.w += S.alpha * h;
    for (int i = 0; i < S.nAux; ++i) S.auxVel[i] += S.auxAcc[i] * h;
}

void integratePosition(BodyState& S, float h, float dv2, float dw2) {
    if (len2(S.v) > dv2) {
        const Vec3 y = S.v * h + S.xComp;  // Kahan summation (the engine stores the residual y - (t - x))
        const Vec3 t = S.x + y;
        S.xComp = y - (t - S.x);
        S.x = t;
    }
    const float w2 = len2(S.w);
    if (w2 > dw2) {
        const float wl = std::sqrt(w2);
        const float half = wl * h * 0.5f;
        const Vec3 ax = S.w * (std::sin(half) / wl);
        S.q = qnormalize(mul(Quat{ax.x, ax.y, ax.z, std::cos(half)}, S.q));
    }
    for (int i = 0; i < S.nAux; ++i)
        if (std::fabs(S.auxVel[i]) > S.auxDead[i]) S.auxPos[i] += S.auxVel[i] * h;
}

float tyreFriction(const Vec3& axis, float Nf, const float set1[8], const float set2[8], bool rigid, WheelRec* rec, float depth,
                   const Vec3& uIn, Vec3& friction) {
    if (!(Nf > 1e-5f)) return 0;
    const Vec3 a = dir001(axis);
    Vec3 u = uIn;
    if (rec != nullptr && !rec->slidingDriven) u = u - a * dot(a, u) + rec->rimAccW;  // gripping: slip := rim accel term
    // Part 1: longitudinal, set 1.
    const Vec3 part1 = a * dot(a, u);
    const Vec3 d1 = dir001(part1);
    const float v1 = dot(d1, part1);
    float F1 = 0, Fn1 = 0, den1 = 0, ratio1 = 0;
    if (v1 > 1e-6f) {
        if (rigid) {
            Fn1 = v1;
            ratio1 = v1 / set1[0];
            if (ratio1 > 1) F1 = set1[2] * 4.0f;
        } else {
            const Curve c = curve(set1, v1, Nf, rec != nullptr);
            if (!c.ok) return 0;
            F1 = c.F, Fn1 = c.Fn, den1 = c.den;
            ratio1 = Fn1 / den1;
        }
        friction -= d1 * F1;
    }
    if (rec != nullptr) rec->fric -= d1 * F1;
    if (set1[0] == 0) return 0;
    // Part 2: lateral, set 2.
    const Vec3 part2 = u - part1;
    const Vec3 d2 = dir001(part2);
    const float v2 = dot(d2, part2);
    float F2 = F1, Fn2 = 0, den2 = 0, ratio2 = 0;
    if (v2 > 0) {
        if (rigid) {
            ratio2 = v2 / set2[0];
            F2 = ratio2 <= 1 ? F1 : set2[2] * 4.0f + F1;
        } else {
            const Curve c = curve(set2, v2, Nf, rec != nullptr);
            if (!c.ok) return 0;
            F2 = c.F, Fn2 = c.Fn, den2 = c.den;
            ratio2 = Fn2 / den2;
        }
        friction -= d2 * F2;
    }
    if (rec == nullptr) return ratio1 > ratio2 ? ratio1 : ratio2;  // (0x720cc0 returns fVar11 = 0 here)
    rec->fric -= d2 * F2;
    float ratio;
    if (ratio1 <= ratio2) {
        rec->sumFn += Fn2;
        rec->sumDen += den2;
        ratio = ratio2;
    } else {
        rec->sumFn += Fn1;
        rec->sumDen += den1;
        ratio = ratio1;
    }
    rec->compRatio = std::max(rec->compRatio, depth / rec->mat.maxDepth);
    rec->slip += uIn;
    return ratio;
}

float genericFriction(const Vec3& axisIn, float Nf, const float set1[8], const float set2[8], bool rigid, WheelRec* rec, float depth,
                      const Vec3& u, Vec3& friction) {
    Vec3 a = (rec != nullptr && rec->tyreAxis) ? rec->dirW : axisIn;
    a = dir001(a);
    const Vec3 uh = dir001(u);
    float cs = dot(uh, a);
    if (cs < 0) cs = -cs, a = -a;
    float c[8];
    for (int i = 0; i < 8; ++i) c[i] = set1[i] * cs + set2[i] * (1 - cs);
    Vec3 up = u;
    if (rec != nullptr && !rec->slidingDriven) up = u - a * dot(u, a) + rec->rimAccW;
    const float v = len(up);
    if (c[0] == 0 || Nf <= 1e-5f) return 0;
    float ratio = 0, F = 0, Fn = 0, den = 0;
    if (v > 0) {
        const Vec3 d = up / v;
        if (rigid) {
            ratio = v / c[0];
            F = ratio > 1 ? 4 * c[2] : 0;
        } else {
            const Curve cv = curve(c, v, Nf, rec != nullptr);
            F = cv.F, Fn = cv.Fn, den = cv.den;
            ratio = den > 0 ? Fn / den : 0;
        }
        friction -= d * F;
        if (rec != nullptr) rec->fric -= d * F;
    }
    if (rec != nullptr) {
        rec->compRatio = std::max(rec->compRatio, depth / rec->mat.maxDepth);
        rec->sumFn += Fn;
        rec->sumDen += den;
        rec->slip += u;
    }
    return ratio;
}

void penaltyForce(Contact& c) {
    const ContactMat& m = c.m;
    if (m.c == 0 || !(c.depth > 0) || c.a == nullptr) return;
    RigidBody& A = *c.a;
    float depth = c.depth;
    Vec3 n = c.n;
    WheelRec* rec = c.wheel;
    // 1. Camber load factor.
    float L = 1;
    if (rec != nullptr) {
        const float cy = std::fabs(n.y);  // normal rotated by the material quaternion (identity in the port)
        float k = (1 - cy) * dot(n, rec->up) + cy;
        if (k < 0) {
            n = rec->up;
            depth = std::max(m.maxDepth, depth);
            k = -k;
        }
        L = k * k;
    }
    // 2. Normal speed with the entry normal; slip base.
    const Vec3 vA = A.pointVel(c.p);
    const Vec3 vB = c.b ? c.b->pointVel(c.p) : Vec3{};
    float vn = dot(vA - vB, n);
    const Vec3 base = (rec != nullptr ? c.surfVelWheel : c.surfVel) + vA - vB;
    // 3. Low-speed normal snap towards world up.
    Vec3 p = c.p;
    const float s = len(A.S.v) + (c.b ? len(c.b->S.v) : 0.0f);
    if (s < 1 && std::fabs(n.y) > 0.8f) {
        const float t = std::max(2 * s - 1, 0.0f);
        const Vec3 up = n.y >= 0 ? Vec3{0, 1, 0} : Vec3{0, -1, 0};
        n = normalize(up + (n - up) * t);
        if (rec != nullptr) p = rec->hubW - n * (rec->radius - depth);
    }
    // 4. Spring and damper.
    float S = 0, D = 0;
    if (m.c != 1) {
        S = (depth + m.preload) * (1.0f / m.c - 1.0f);
        if (depth > m.maxDepth) vn = std::max(vn, 0.0f);
        if (depth > 1e-4f) {
            float cA = 1.0f / std::max(m.dampC, 0.01f) - 1.0f;
            if (m.dampR < 1 && vn > 0) cA = 1.0f / std::max(m.dampR, 0.01f) - 1.0f;
            float cB = 1.0f / std::max(m.quadC, 0.01f) - 1.0f;
            if (m.quadR < 1 && vn > 0) cB = 1.0f / std::max(m.quadR, 0.01f) - 1.0f;
            D = vn <= 0 ? std::min(vn * vn, -vn) * cB - cA * vn : -cA * vn - std::min(vn * vn, vn) * cB;
        }
    }
    const float N = clampf(S + D, 0, m.maxN);
    // 5. Friction load.
    float Nf = S;
    if (S > 0.001f && m.loadFloor != 0) Nf = m.loadBlend * S + (1 - m.loadBlend) * m.loadFloor;
    if (Nf > 0.001f && m.loadExp != 1) Nf *= std::min(std::pow(S, m.loadExp) / S, 1.0f);
    // 6. Forces.
    if (!(S > 0.001f)) return;
    const Vec3 u = base - n * dot(n, base);
    Vec3 fr;
    float ratio;
    if (rec != nullptr && rec->useTyre)
        ratio = tyreFriction(rec->dirW, Nf * L, m.set1, m.set2, false, rec, depth, u, fr);
    else
        ratio = genericFriction(u, Nf * L, m.set1, m.set2, false, rec, depth, u, fr);
    fr = fr - n * dot(n, fr);
    if (ratio > 1) A.slidingContacts += 1;
    Vec3 F = fr + n * N;
    const float mass = c.b ? std::min(A.mass, c.b->mass) : A.mass;
    F *= mass;
    A.addForceAt(F, p);
    if (c.b) c.b->addForceAt(-F, p);
}

void attachWheel(Contact& c, WheelRec& rec, const Vec3& bodyUp) {
    const Vec3 d = rec.hubW - c.p;
    const float proj = dot(d, bodyUp);
    const float dist = len(d);
    float comp = c.depth;
    if (dist > 1e-6f) comp = c.depth / dist * proj;
    c.m = rec.mat;
    if (dist > 1e-6f) {
        const Vec3 a = cross(rec.rimVelW, bodyUp) / dist;
        rec.contactRimVel = cross(d, a);
        c.surfVelWheel = c.surfVel + rec.contactRimVel;
        c.p = rec.hubW - c.n * (rec.radius - c.depth);
    } else {
        c.surfVelWheel = c.surfVel;
    }
    c.depth = std::max(c.depth + c.m.extraDepth, 0.0f);  // (the normal would be rotated by rec+0x174: identity)
    const float load = std::min(std::max(c.m.extraDepth + comp, 0.0f), rec.mat.maxDepth);
    const float dd = std::min(c.depth, rec.mat.maxDepth);
    c.wheel = &rec;
    rec.maxDepthSeen = std::max(rec.maxDepthSeen, dd);
    rec.load = std::max(rec.load, load);
    rec.nDepth += c.n * c.depth;
}

bool prepareImpulse(Contact& c) {
    c.impN = c.impF = false;
    if (c.m.c == 0) {
        c.impN = true;
        c.impF = c.m.set1[0] > 0;
    } else if (c.depth > c.m.maxDepth) {
        c.impN = true;
        c.depth -= c.m.maxDepth;
        c.m.dampC = 0;
        if (c.wheel != nullptr && dot(c.n, c.wheel->up) < 0) c.n = c.wheel->up;
    }
    if (!c.impN) return false;
    RigidBody& A = *c.a;
    c.ra = c.p - A.S.x;
    c.raXn = cross(c.ra, c.n);
    c.IraXn = A.I2invW * c.raXn;
    float k = A.invMass + dot(c.raXn, c.IraXn);
    c.thr = A.impulseThr;
    if (c.b) {
        c.rb = c.p - c.b->S.x;
        c.rbXn = cross(c.rb, c.n);
        c.IrbXn = c.b->I2invW * c.rbXn;
        k += c.b->invMass + dot(c.rbXn, c.IrbXn);
        c.thr = 0.5f * (A.impulseThr + c.b->impulseThr);
    }
    c.effMass = k > 0 ? 1.0f / k : 0.0f;
    c.biasVel = c.m.bias * c.depth;
    return true;
}

namespace {

float normalVel(const Contact& c) {  // 0x720410
    const RigidBody& A = *c.a;
    float vn = dot(c.n, A.S.v) + dot(c.raXn, A.S.w) + dot(c.n, c.surfVel);
    if (c.b) vn -= dot(c.n, c.b->S.v) + dot(c.rbXn, c.b->S.w);
    return vn;
}

void applyAt(Contact& c, const Vec3& dir, float j) {
    RigidBody& A = *c.a;
    const Vec3 rx = cross(c.ra, dir);
    A.S.v += dir * (A.invMass * j);
    A.S.w += A.I2invW * rx * j;
    if (c.b) {
        const Vec3 ry = cross(c.rb, dir);
        c.b->S.v -= dir * (c.b->invMass * j);
        c.b->S.w -= c.b->I2invW * ry * j;
    }
}

}  // namespace

void applyImpulses(Contact& c) {
    RigidBody& A = *c.a;
    const float e = c.m.dampC;
    if (c.impF) {  // 0x7204d0
        const float vn = normalVel(c);
        if (vn <= 0) {
            float jn = (-e * vn - vn) * c.effMass;
            if (jn >= c.thr) {
                Vec3 vrel = A.pointVel(c.p) + c.surfVel;
                if (c.b) vrel -= c.b->pointVel(c.p);
                const Vec3 vt = vrel - c.n * dot(vrel, c.n);
                const float s = len(vt);
                if (s > 1e-6f) {
                    const Vec3 t = vt / s;
                    const Vec3 rax = cross(c.ra, t);
                    float k = A.invMass + dot(rax, A.I2invW * rax);
                    if (c.b) {
                        const Vec3 rbx = cross(c.rb, t);
                        k += c.b->invMass + dot(rbx, c.b->I2invW * rbx);
                    }
                    float jt = s / k;
                    if (jt > c.m.set1[0] * jn) jt = c.m.set1[2] * jn;
                    applyAt(c, -t, jt);
                }
            }
        }
    }
    if (c.impN) {  // 0x720b10
        const float vn = normalVel(c);
        if (vn < c.biasVel) {
            const float target = std::max(-e * vn, c.biasVel);
            const float dv = target - vn;
            if (dv >= c.thr) applyAt(c, c.n, dv * c.effMass);
        }
    }
}

// ---- geometry ------------------------------------------------------------------------------

Tri makeTri(const Vec3& v0, const Vec3& v1, const Vec3& v2, int surface) {
    Tri t;
    t.v0 = v0, t.v1 = v1, t.v2 = v2;
    t.e0 = v1 - v0;
    t.e1 = v2 - v0;
    const Vec3 nn = cross(t.e0, t.e1);
    const float l = len(nn);
    t.degenerate = !(l > 0);
    t.n = l > 0 ? nn / l : nn;
    t.d = dot(t.n, v0);
    t.a00 = dot(t.e0, t.e0);
    t.a01 = dot(t.e0, t.e1);
    t.a11 = dot(t.e1, t.e1);
    t.centroid = (v0 + v1 + v2) * 0.33333334f;
    t.boundR = std::max({len(t.centroid - v0), len(t.centroid - v1), len(t.centroid - v2)});
    t.surface = surface;
    return t;
}

bool sphereTriangle(const Tri& t, const Vec3& c, float r, float& depth, Vec3& n, Vec3& p) {
    // 0x7260a0: plane distance and bounding sphere.
    if (!(std::fabs(dot(t.n, c) - t.d) < r)) return false;
    if (len2(t.centroid - c) > (t.boundR + r) * (t.boundR + r) || t.degenerate) return false;
    // 0x769560: the closest point of the triangle to the centre (the engine uses Eberly's region tests; the
    // port uses Ericson's equivalent formulation), then depth and normal.
    const Vec3 ap = c - t.v0;
    const float d1 = dot(t.e0, ap), d2 = dot(t.e1, ap);
    Vec3 closest;
    if (d1 <= 0 && d2 <= 0) {
        closest = t.v0;
    } else {
        const Vec3 bp = c - t.v1;
        const float d3 = dot(t.e0, bp), d4 = dot(t.e1, bp);
        const Vec3 cp = c - t.v2;
        const float d5 = dot(t.e0, cp), d6 = dot(t.e1, cp);
        const float vc = d1 * d4 - d3 * d2, vb = d5 * d2 - d1 * d6, va = d3 * d6 - d5 * d4;
        if (d3 >= 0 && d4 <= d3) {
            closest = t.v1;
        } else if (vc <= 0 && d1 >= 0 && d3 <= 0) {
            closest = t.v0 + t.e0 * (d1 / (d1 - d3));
        } else if (d6 >= 0 && d5 <= d6) {
            closest = t.v2;
        } else if (vb <= 0 && d2 >= 0 && d6 <= 0) {
            closest = t.v0 + t.e1 * (d2 / (d2 - d6));
        } else if (va <= 0 && (d4 - d3) >= 0 && (d5 - d6) >= 0) {
            closest = t.v1 + (t.v2 - t.v1) * ((d4 - d3) / ((d4 - d3) + (d5 - d6)));
        } else {
            const float den = 1.0f / (va + vb + vc);
            closest = t.v0 + t.e0 * (vb * den) + t.e1 * (vc * den);
        }
    }
    const Vec3 q = closest - c;
    const float dist2 = len2(q);
    if (dist2 > r * r) return false;
    const float dist = std::sqrt(dist2);
    if (!(dist > 0)) return false;  // centre on the triangle: the engine keeps no contact either (no direction)
    depth = r - dist;
    n = q * (-1.0f / dist);         // from the closest point towards the centre
    p = c - n * r;
    return true;
}

float TerrainTriangles::sample(int64_t x, int64_t z) {
    const uint64_t key = (uint64_t(uint32_t(int32_t(x))) << 32) | uint32_t(int32_t(z));
    auto it = samples_.find(key);
    if (it != samples_.end()) return it->second;
    const float h = height_(double(x) + ox_, double(z) + oz_);
    samples_.emplace(key, h);
    return h;
}

const TerrainTriangles::Cell& TerrainTriangles::cell(int64_t ix, int64_t iz) {
    const uint64_t key = (uint64_t(uint32_t(int32_t(ix))) << 32) | uint32_t(int32_t(iz));
    auto it = cells_.find(key);
    if (it != cells_.end()) {
        it->second.used = ++tick_;
        return it->second;
    }
    Cell c;
    c.used = ++tick_;
    const int64_t x0 = ix * 4, z0 = iz * 4;
    Vec3 pt[9];
    for (int i = 0; i < 9; ++i) {
        const int64_t x = x0 + (i % 3) * 2, z = z0 + (i / 3) * 2;
        pt[i] = Vec3(float(x), sample(x, z), float(z));
    }
    // Border straightness (corner, midpoint, corner): cos of the angle at the first corner > 0.9997.
    const int border[4][3] = {{0, 1, 2}, {0, 3, 6}, {2, 5, 8}, {6, 7, 8}};
    bool straight[4];
    int count = 8;
    for (int k = 0; k < 4; ++k) {
        const Vec3 far = pt[border[k][2]] - pt[border[k][0]], mid = pt[border[k][1]] - pt[border[k][0]];
        straight[k] = dot(normalize(mid), normalize(far)) > 0.9997f;
        count -= straight[k] ? 1 : 0;
    }
    if (count == 4) {
        c.tris.push_back(makeTri(pt[0], pt[6], pt[8]));
        c.tris.push_back(makeTri(pt[0], pt[8], pt[2]));
    } else {
        // Each border (a, centre, midpoint, b) in the engine's order.
        const int fan[4][4] = {{0, 4, 1, 2}, {6, 4, 3, 0}, {2, 4, 5, 8}, {8, 4, 7, 6}};
        for (int k = 0; k < 4; ++k) {
            const Vec3 &a = pt[fan[k][0]], &ce = pt[fan[k][1]], &m = pt[fan[k][2]], &b = pt[fan[k][3]];
            if (straight[k]) {
                c.tris.push_back(makeTri(a, ce, b));
            } else {
                c.tris.push_back(makeTri(a, ce, m));
                c.tris.push_back(makeTri(m, ce, b));
            }
        }
    }
    return cells_.emplace(key, std::move(c)).first->second;
}

void TerrainTriangles::query(float cx, float cz, float r, std::vector<const Tri*>& out) {
    const int64_t ix0 = int64_t(std::floor((cx - r) / 4)), ix1 = int64_t(std::floor((cx + r) / 4));
    const int64_t iz0 = int64_t(std::floor((cz - r) / 4)), iz1 = int64_t(std::floor((cz + r) / 4));
    for (int64_t iz = iz0; iz <= iz1; ++iz)
        for (int64_t ix = ix0; ix <= ix1; ++ix)
            for (const Tri& t : cell(ix, iz).tris) out.push_back(&t);
}

float TerrainTriangles::surfaceHeight(float x, float z, Vec3* normal) {
    const Cell& c = cell(int64_t(std::floor(x / 4)), int64_t(std::floor(z / 4)));
    for (const Tri& t : c.tris) {
        // Barycentric test in the xz plane.
        const float d00x = t.v1.x - t.v0.x, d00z = t.v1.z - t.v0.z, d01x = t.v2.x - t.v0.x, d01z = t.v2.z - t.v0.z;
        const float px = x - t.v0.x, pz = z - t.v0.z;
        const float den = d00x * d01z - d01x * d00z;
        if (den == 0) continue;
        const float s = (px * d01z - d01x * pz) / den, u = (d00x * pz - px * d00z) / den;
        if (s < -1e-5f || u < -1e-5f || s + u > 1 + 1e-5f) continue;
        if (normal) *normal = t.n.y >= 0 ? t.n : -t.n;
        return t.v0.y + s * (t.v1.y - t.v0.y) + u * (t.v2.y - t.v0.y);
    }
    if (normal) *normal = {0, 1, 0};
    return sample(int64_t(std::floor(x)), int64_t(std::floor(z)));
}

void TerrainTriangles::trim(size_t maxCells) {
    if (cells_.size() <= maxCells) return;
    const uint64_t keep = tick_ > maxCells ? tick_ - maxCells : 0;
    for (auto it = cells_.begin(); it != cells_.end();) it = it->second.used < keep ? cells_.erase(it) : std::next(it);
    if (samples_.size() > maxCells * 12) samples_.clear();
}

}  // namespace fuel::fv
