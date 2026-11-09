#include "fuel_collide.hpp"

#include <algorithm>
#include <cmath>

namespace fuel::fv {

Obb placeObb(const float E[12], const float half[3], float radius, const float M[16]) {  // 0x743ce0 + 0x742b80
    const float scale = std::sqrt(M[0] * M[0] + M[1] * M[1] + M[2] * M[2]);
    const float s = 1.0f / scale;
    Obb b;
    for (int i = 0; i < 3; ++i) {
        const float* e = E + 4 * i;
        b.ax[i] = Vec3{e[0] * M[0] + e[1] * M[4] + e[2] * M[8], e[0] * M[1] + e[1] * M[5] + e[2] * M[9],
                       e[0] * M[2] + e[1] * M[6] + e[2] * M[10]} *
                  s;
    }
    b.c = {E[3] * M[0] + E[7] * M[4] + E[11] * M[8] + M[12], E[3] * M[1] + E[7] * M[5] + E[11] * M[9] + M[13],
           E[3] * M[2] + E[7] * M[6] + E[11] * M[10] + M[14]};
    b.h = Vec3{half[0], half[1], half[2]} * scale;
    b.r = radius * scale;
    return b;
}

bool sphereObb(const Obb& b, const Vec3& c, float r, Vec3& p, Vec3& n, float& depth) {  // 0x71a9e0
    const Vec3 d = c - b.c;
    const float l[3] = {dot(d, b.ax[0]), dot(d, b.ax[1]), dot(d, b.ax[2])};
    // Excess outside the box per axis: min(-h, l) + max(h, l).
    float e[3];
    for (int i = 0; i < 3; ++i) e[i] = std::min(-b.h[i], l[i]) + std::max(b.h[i], l[i]);
    const float d2 = e[0] * e[0] + e[1] * e[1] + e[2] * e[2];
    if (!(r * r >= d2)) return false;
    Vec3 nl;
    if (d2 <= 0.0001f) {
        // Centre inside: the face with the least penetration h - |l|.
        float best = 1e6f;
        int axis = -1;
        float sign = 0;
        for (int i = 0; i < 3; ++i) {
            const float f = l[i] <= 0 ? b.h[i] + l[i] : b.h[i] - l[i];
            if (f < best) {
                best = f;
                axis = i;
                sign = l[i] == 0 ? 0.0f : (l[i] >= 0 ? 1.0f : -1.0f);
            }
        }
        float v[3] = {0, 0, 0};
        if (axis >= 0) v[axis] = sign;
        nl = {v[0], v[1], v[2]};
        depth = best + r;
    } else {
        const float dist = std::sqrt(d2);
        depth = r - dist;
        nl = Vec3{e[0], e[1], e[2]} * (1.0f / dist);
    }
    n = b.ax[0] * nl.x + b.ax[1] * nl.y + b.ax[2] * nl.z;
    p = c + n * -(r - depth * 0.5f);
    return true;
}

bool sphereSphere(const Vec3& cs, float rs, const Vec3& c, float r, Vec3& p, Vec3& n, float& depth) {  // 0x71b1b0
    const Vec3 d = c - cs;
    const float d2 = len2(d);
    const float sum = rs + r;
    if (!(sum * sum > d2) || !(d2 > 1e-6f)) return false;
    const float dist = std::sqrt(d2);
    n = d * (1.0f / dist);
    depth = sum - dist;
    p = c + n * (r - depth * 0.5f);
    return true;
}

namespace {
// Distance from a point (relative to the box centre) to the box, squared, against r^2 (0x7266a0's inner test).
bool pointNearBox(const Obb& b, const Vec3& d, float r) {
    float q = 0;
    for (int i = 0; i < 3; ++i) {
        const float x = std::max(std::fabs(dot(d, b.ax[i])) - b.h[i], 0.0f);
        q += x * x;
    }
    return q <= r * r;
}
}  // namespace

bool obbSphereCull(const Obb& b, const Vec3& c, float r) {  // 0x7266a0
    const Vec3 d = c - b.c;
    const float s = b.r + r;
    if (!(len2(d) <= s * s)) return false;
    return pointNearBox(b, d, r);
}

bool obbObbCull(const Obb& a, const Obb& b) {  // 0x726160
    const Vec3 d = b.c - a.c;
    const float s = a.r + b.r;
    if (!(len2(d) <= s * s)) return false;
    if (!pointNearBox(a, d, b.r)) return false;
    return pointNearBox(b, d, a.r);
}

// ---- box vs box (0x763890) -------------------------------------------------------------------

namespace {

// 0x760a90 (ODE intersectRectQuad): clips the quad p[8] against the rectangle |x| < h[0], |y| < h[1].
int intersectRectQuad(const float h[2], float p[8], float ret[16]) {
    int nq = 4, nr = 0;
    float buffer[16];
    float* q = p;
    float* r = ret;
    for (int dir = 0; dir <= 1; ++dir) {
        for (int sign = -1; sign <= 1; sign += 2) {
            float* pq = q;
            float* pr = r;
            nr = 0;
            for (int i = nq; i > 0; --i) {
                if (float(sign) * pq[dir] < h[dir]) {
                    pr[0] = pq[0], pr[1] = pq[1];
                    pr += 2;
                    ++nr;
                    if (nr & 8) {
                        q = r;
                        goto done;
                    }
                }
                {
                    float* nextq = i > 1 ? pq + 2 : q;
                    if ((float(sign) * pq[dir] < h[dir]) != (float(sign) * nextq[dir] < h[dir])) {
                        pr[1 - dir] = pq[1 - dir] + (nextq[1 - dir] - pq[1 - dir]) / (nextq[dir] - pq[dir]) * (float(sign) * h[dir] - pq[dir]);
                        pr[dir] = float(sign) * h[dir];
                        pr += 2;
                        ++nr;
                        if (nr & 8) {
                            q = r;
                            goto done;
                        }
                    }
                }
                pq += 2;
            }
            q = r;
            r = q == ret ? buffer : ret;
            nq = nr;
        }
    }
done:
    if (q != ret) std::copy(q, q + nr * 2, ret);
    return nr;
}

// 0x761060 (ODE dLineClosestApproach).
void lineClosestApproach(const Vec3& pa, const Vec3& ua, const Vec3& pb, const Vec3& ub, float& alpha, float& beta) {
    const Vec3 p = pb - pa;
    const float uaub = dot(ua, ub), q1 = dot(ua, p), q2 = -dot(ub, p);
    float d = 1 - uaub * uaub;
    if (d <= 0) {
        alpha = beta = 0;
        return;
    }
    d = 1.0f / d;
    alpha = (q1 + uaub * q2) * d;
    beta = (uaub * q1 + q2) * d;
}

}  // namespace

int boxBox(const Obb& a, const Obb& b, Vec3& normal, Vec3 pts[20], float depthOut[20]) {
    constexpr float kTol = 0.001f;    // FUEL: separated when the gap exceeds 1 mm (ODE: 0)
    constexpr float kFudge = 1.05f;   // edge axes must beat the faces by 5 %
    const Vec3 p = b.c - a.c;
    const float pp[3] = {dot(a.ax[0], p), dot(a.ax[1], p), dot(a.ax[2], p)};
    const float A[3] = {a.h.x, a.h.y, a.h.z}, B[3] = {b.h.x, b.h.y, b.h.z};
    float R[3][3], Q[3][3];
    for (int i = 0; i < 3; ++i)
        for (int j = 0; j < 3; ++j) R[i][j] = dot(a.ax[i], b.ax[j]), Q[i][j] = std::fabs(R[i][j]);
    float s = -1e14f;
    bool invert = false;
    int code = 0;
    Vec3 normalR;
    bool haveR = false;
    Vec3 normalC;
    auto tstFace = [&](float e1, float e2, const Vec3& ax, int cc) {
        const float s2 = std::fabs(e1) - e2;
        if (s2 > kTol) return false;
        if (s2 > s) {
            s = s2, normalR = ax, haveR = true, invert = e1 < 0, code = cc;
        }
        return true;
    };
    if (!tstFace(pp[0], A[0] + B[0] * Q[0][0] + B[1] * Q[0][1] + B[2] * Q[0][2], a.ax[0], 1)) return 0;
    if (!tstFace(pp[1], A[1] + B[0] * Q[1][0] + B[1] * Q[1][1] + B[2] * Q[1][2], a.ax[1], 2)) return 0;
    if (!tstFace(pp[2], A[2] + B[0] * Q[2][0] + B[1] * Q[2][1] + B[2] * Q[2][2], a.ax[2], 3)) return 0;
    if (!tstFace(dot(b.ax[0], p), A[0] * Q[0][0] + A[1] * Q[1][0] + A[2] * Q[2][0] + B[0], b.ax[0], 4)) return 0;
    if (!tstFace(dot(b.ax[1], p), A[0] * Q[0][1] + A[1] * Q[1][1] + A[2] * Q[2][1] + B[1], b.ax[1], 5)) return 0;
    if (!tstFace(dot(b.ax[2], p), A[0] * Q[0][2] + A[1] * Q[1][2] + A[2] * Q[2][2] + B[2], b.ax[2], 6)) return 0;
    auto tstEdge = [&](float e1, float e2, float n1, float n2, float n3, int cc) {
        float s2 = std::fabs(e1) - e2;
        if (s2 > kTol) return false;
        const float l = std::sqrt(n1 * n1 + n2 * n2 + n3 * n3);
        if (l > 0) {
            s2 /= l;
            if (s2 * kFudge > s) {
                s = s2, haveR = false, normalC = Vec3{n1, n2, n3} / l, invert = e1 < 0, code = cc;
            }
        }
        return true;
    };
    // R[i][j] = a_i . b_j; ODE's Rij is R[i-1][j-1].
    const float R11 = R[0][0], R12 = R[0][1], R13 = R[0][2], R21 = R[1][0], R22 = R[1][1], R23 = R[1][2], R31 = R[2][0], R32 = R[2][1],
                R33 = R[2][2];
    const float Q11 = Q[0][0], Q12 = Q[0][1], Q13 = Q[0][2], Q21 = Q[1][0], Q22 = Q[1][1], Q23 = Q[1][2], Q31 = Q[2][0], Q32 = Q[2][1],
                Q33 = Q[2][2];
    if (!tstEdge(pp[2] * R21 - pp[1] * R31, A[1] * Q31 + A[2] * Q21 + B[1] * Q13 + B[2] * Q12, 0, -R31, R21, 7)) return 0;
    if (!tstEdge(pp[2] * R22 - pp[1] * R32, A[1] * Q32 + A[2] * Q22 + B[0] * Q13 + B[2] * Q11, 0, -R32, R22, 8)) return 0;
    if (!tstEdge(pp[2] * R23 - pp[1] * R33, A[1] * Q33 + A[2] * Q23 + B[0] * Q12 + B[1] * Q11, 0, -R33, R23, 9)) return 0;
    if (!tstEdge(pp[0] * R31 - pp[2] * R11, A[0] * Q31 + A[2] * Q11 + B[1] * Q23 + B[2] * Q22, R31, 0, -R11, 10)) return 0;
    if (!tstEdge(pp[0] * R32 - pp[2] * R12, A[0] * Q32 + A[2] * Q12 + B[0] * Q23 + B[2] * Q21, R32, 0, -R12, 11)) return 0;
    if (!tstEdge(pp[0] * R33 - pp[2] * R13, A[0] * Q33 + A[2] * Q13 + B[0] * Q22 + B[1] * Q21, R33, 0, -R13, 12)) return 0;
    if (!tstEdge(pp[1] * R11 - pp[0] * R21, A[0] * Q21 + A[1] * Q11 + B[1] * Q33 + B[2] * Q32, -R21, R11, 0, 13)) return 0;
    if (!tstEdge(pp[1] * R12 - pp[0] * R22, A[0] * Q22 + A[1] * Q12 + B[0] * Q33 + B[2] * Q31, -R22, R12, 0, 14)) return 0;
    if (!tstEdge(pp[1] * R13 - pp[0] * R23, A[0] * Q23 + A[1] * Q13 + B[0] * Q32 + B[1] * Q31, -R23, R13, 0, 15)) return 0;
    if (code == 0) return 0;
    // 0x70af10: the edge normal in world space = box a's axes weighted by normalC.
    normal = haveR ? normalR : a.ax[0] * normalC.x + a.ax[1] * normalC.y + a.ax[2] * normalC.z;
    if (invert) normal = -normal;
    const float depth = -s;

    if (code > 6) {  // edge-edge: one contact between the two edges, moved by n * depth / 2
        // Signs of the normal on the axes in double (the engine's x87, 53-bit; one axis of each box is
        // perpendicular to the normal, its sign is rounding).
        auto sgn = [&](const Vec3& ax) { return double(ax.y) * normal.y + double(normal.z) * ax.z + double(ax.x) * normal.x; };
        Vec3 pa = a.c;
        for (int j = 0; j < 3; ++j) pa += a.ax[j] * ((sgn(a.ax[j]) > 0 ? 1.0f : -1.0f) * A[j]);
        Vec3 pb = b.c;
        for (int j = 0; j < 3; ++j) pb += b.ax[j] * ((sgn(b.ax[j]) > 0 ? -1.0f : 1.0f) * B[j]);
        const Vec3 ua = a.ax[(code - 7) / 3], ub = b.ax[(code - 7) % 3];
        float alpha, beta;
        lineClosestApproach(pa, ua, pb, ub, alpha, beta);
        pa += ua * alpha;
        pb += ub * beta;
        pts[0] = (pa + pb) * 0.5f + normal * (depth * 0.5f);
        depthOut[0] = depth;
        return 1;
    }

    // Face-something: reference face on box a (codes 1-3) or b (4-6).
    const Obb& Ra = code <= 3 ? a : b;
    const Obb& Rb = code <= 3 ? b : a;
    const Vec3 pa = Ra.c, pb = Rb.c;
    const float* Sa = code <= 3 ? A : B;
    const float* Sb = code <= 3 ? B : A;
    const Vec3 normal2 = code <= 3 ? normal : -normal;
    const float nr[3] = {dot(Rb.ax[0], normal2), dot(Rb.ax[1], normal2), dot(Rb.ax[2], normal2)};
    const float anr[3] = {std::fabs(nr[0]), std::fabs(nr[1]), std::fabs(nr[2])};
    int lanr, a1, a2;
    if (anr[1] > anr[0]) {
        if (anr[1] > anr[2]) {
            a1 = 0, lanr = 1, a2 = 2;
        } else {
            a1 = 0, a2 = 1, lanr = 2;
        }
    } else {
        if (anr[0] > anr[2]) {
            lanr = 0, a1 = 1, a2 = 2;
        } else {
            a1 = 0, a2 = 1, lanr = 2;
        }
    }
    const Vec3 center = nr[lanr] < 0 ? pb - pa + Rb.ax[lanr] * Sb[lanr] : pb - pa - Rb.ax[lanr] * Sb[lanr];
    const int codeN = code <= 3 ? code - 1 : code - 4;
    const int code1 = codeN == 0 ? 1 : 0, code2 = codeN == 2 ? 1 : 2;
    float quad[8];
    const float c1 = dot(center, Ra.ax[code1]), c2 = dot(center, Ra.ax[code2]);
    float m11 = dot(Ra.ax[code1], Rb.ax[a1]), m12 = dot(Ra.ax[code1], Rb.ax[a2]);
    float m21 = dot(Ra.ax[code2], Rb.ax[a1]), m22 = dot(Ra.ax[code2], Rb.ax[a2]);
    {
        const float k1 = m11 * Sb[a1], k2 = m21 * Sb[a1], k3 = m12 * Sb[a2], k4 = m22 * Sb[a2];
        quad[0] = c1 - k1 - k3, quad[1] = c2 - k2 - k4;
        quad[2] = c1 - k1 + k3, quad[3] = c2 - k2 + k4;
        quad[4] = c1 + k1 + k3, quad[5] = c2 + k2 + k4;
        quad[6] = c1 + k1 - k3, quad[7] = c2 + k2 - k4;
    }
    const float rect[2] = {Sa[code1], Sa[code2]};
    float ret[16];
    const int nPts = intersectRectQuad(rect, quad, ret);
    if (nPts < 1) return 0;
    const float det1 = 1.0f / (m11 * m22 - m12 * m21);
    m11 *= det1, m12 *= det1, m21 *= det1, m22 *= det1;
    Vec3 point[8];
    float dep[8];
    int cnum = 0;
    for (int j = 0; j < nPts; ++j) {
        const float k1 = m22 * (ret[j * 2] - c1) - m12 * (ret[j * 2 + 1] - c2);
        const float k2 = -m21 * (ret[j * 2] - c1) + m11 * (ret[j * 2 + 1] - c2);
        point[cnum] = center + Rb.ax[a1] * k1 + Rb.ax[a2] * k2;
        dep[cnum] = Sa[codeN] - dot(normal2, point[cnum]);
        if (dep[cnum] >= 0) {
            ret[cnum * 2] = ret[j * 2], ret[cnum * 2 + 1] = ret[j * 2 + 1];
            ++cnum;
        }
    }
    if (cnum < 1) return 0;
    // At most 8 points here, under FUEL's limit of 20: the cull (0x760cc0) never runs.
    for (int j = 0; j < cnum; ++j) {
        pts[j] = point[j] + pa + normal * (dep[j] * 0.5f);
        depthOut[j] = dep[j];
    }
    return cnum;
}

// ---- box vs triangle (0x765bc0 + 0x7667e0) ---------------------------------------------------

namespace {

struct Sat {
    float best = 0;  // 0xa3ebb4
    Vec3 axis;       // 0xa6ac88
    int code = 0;    // 0xa65924
};

// 0x765800: a box face axis (unit) with the triangle's projections p0..p2 against the half extent h.
bool satFace(Sat& s, const Vec3& axis, int code, float p0, float p1, float p2, float h) {
    float mn = std::min(p1, p0);
    if (p2 <= mn) mn = p2;
    float mx = std::max(p1, p0);
    mx = std::max(p2, mx);
    const float lo = h - mn, hi = h + mx;
    if (!(hi * lo >= -9.9999994e-11f)) return false;
    if (lo <= hi) {
        if (s.best > lo) s.best = lo, s.axis = axis, s.code = code;
    } else {
        if (s.best > hi) s.best = hi, s.axis = -axis, s.code = code;
    }
    return true;
}

// 0x765920: an edge axis X (not unit) with the triangle's two projections pa, pb against the box radius R;
// an edge wins only if 1.5 x its depth is below the best.
bool satEdge(Sat& s, const Vec3& X, int code, float pa, float pb, float R) {
    float mx = pb, mn = pa;
    if (pb <= pa) mx = pa, mn = pb;
    if ((mx + R) * (R - mn) < -9.9999994e-11f) return false;
    const float l = len(X);
    if (!(l > 1e-5f)) return true;
    const float inv = 1.0f / l;
    if (R - mn <= mx + R) {
        const float d = (R - mn) * inv;
        if (1.5f * d < s.best) s.best = d, s.axis = X * inv, s.code = code;
    } else {
        const float d = (mx + R) * inv;
        if (1.5f * d < s.best) s.best = d, s.axis = X * -inv, s.code = code;
    }
    return true;
}

bool boxTriSat(const Obb& b, const Tri& t, Sat& s) {  // 0x765bc0
    const Vec3 d = t.v0 - b.c;
    const float nA[3] = {dot(b.ax[0], t.N), dot(b.ax[1], t.N), dot(b.ax[2], t.N)};
    const float f = std::fabs(nA[2]) * b.h.z + b.h.y * std::fabs(nA[1]) + b.h.x * std::fabs(nA[0]) + dot(t.N, d);
    if (!(f >= 0)) return false;
    if (t.Nlen > 0) {
        s.best = f * t.invNlen;
        s.code = 1;
        s.axis = t.N * -t.invNlen;
    }
    const Vec3 E[3] = {t.e0, t.e1, t.e2};
    float pe[3][3];  // pe[i][k] = A_i . e_k
    float pd[3];
    for (int i = 0; i < 3; ++i) {
        for (int k = 0; k < 3; ++k) pe[i][k] = dot(b.ax[i], E[k]);
        pd[i] = dot(b.ax[i], d);
    }
    for (int i = 0; i < 3; ++i)
        if (!satFace(s, b.ax[i], 2 + i, pd[i], pe[i][0] + pd[i], pe[i][1] + pd[i], b.h[i])) return false;
    // Edges: A_i x e_k, codes 5 + 3 i + k; the second projection differs by +-(A_i . N).
    for (int i = 0; i < 3; ++i) {
        const int j = (i + 1) % 3, k2 = (i + 2) % 3;
        const int lo = std::min(j, k2), hi = std::max(j, k2);
        for (int k = 0; k < 3; ++k) {
            const Vec3 X = cross(b.ax[i], E[k]);
            const float pa = dot(X, d);
            const float pb = k == 0 ? pa + nA[i] : pa - nA[i];
            const float R = std::fabs(pe[hi][k]) * b.h[lo] + std::fabs(pe[lo][k]) * b.h[hi];
            if (!satEdge(s, X, 5 + 3 * i + k, pa, pb, R)) return false;
        }
    }
    return true;
}

// 0x765a50: Sutherland-Hodgman against the plane (n, w) (inside: n . v + w v.w >= 0); vertices are xyzw in
// `pool`, new ones appended at *poolN (w = 1).
int clip(Vec3 pool[], float poolW[], int& poolN, const int* in, int nIn, int* out, const Vec3& n, float w) {
    int nOut = 0;
    int prev = in[nIn - 1];
    float dPrev = dot(pool[prev], n) + poolW[prev] * w;
    for (int i = 0; i < nIn; ++i) {
        const int cur = in[i];
        const float dCur = dot(pool[cur], n) + poolW[cur] * w;
        if (dPrev >= 0) out[nOut++] = prev;
        if (dCur * dPrev < 0) {
            const float k = dPrev / (dPrev - dCur);
            pool[poolN] = pool[prev] + (pool[cur] - pool[prev]) * k;
            poolW[poolN] = 1;
            out[nOut++] = poolN++;
        }
        prev = cur;
        dPrev = dCur;
    }
    return nOut;
}

}  // namespace

int boxTriangle(const Obb& b, const Tri& t, Vec3 pts[8], float depth[8], Vec3* axisOut) {
    Sat s;
    s.best = 3.4e38f;
    if (t.Nlen <= 0 || !boxTriSat(b, t, s) || s.code == 0) return 0;
    if (axisOut) *axisOut = s.axis;
    const Vec3 nb = s.axis;
    if (s.code > 4) {  // edge-edge: one contact between the box edge and the triangle edge
        // The corner towards the triangle. The axis is perpendicular to the edge's box axis, so that sign is
        // rounding: the engine's x87 (53-bit) dot product decides; done in double, in its order.
        Vec3 corner = b.c;
        for (int i = 0; i < 3; ++i) {
            const double d = double(nb.x) * b.ax[i].x + double(nb.y) * b.ax[i].y + double(nb.z) * b.ax[i].z;
            corner += b.ax[i] * ((d > 0 ? 1.0f : -1.0f) * b.h[i]);
        }
        const int bi = (s.code - 5) / 3, te = (s.code - 5) % 3;
        // The triangle edge: (v0, e0), (v0, e1) or (v2, e2) (sic: e2 = v2 - v1 from v2).
        const Vec3 q0 = te == 2 ? t.v2 : t.v0;
        Vec3 dir = te == 0 ? t.e0 : te == 1 ? t.e1 : t.e2;
        const float elen = len(dir);
        dir = len2(dir) > 0 ? dir * (1.0f / elen) : Vec3{1, 0, 0};  // 0x7655f0
        const Vec3 ax = b.ax[bi];
        const float hb = b.h[bi];
        float sb, st;
        lineClosestApproach(corner, ax, q0, dir, sb, st);  // 0x766740
        if (!(-hb <= sb && sb <= hb)) return 0;
        if (!(0 <= st && st <= elen)) return 0;
        pts[0] = (q0 + dir * st + corner + ax * sb) * 0.5f;
        depth[0] = s.best;
        return 1;
    }
    Vec3 pool[24];
    float poolW[24];
    int idxA[16], idxB[16];
    if (s.code != 1) {  // a box face: the triangle (relative to the box centre) clipped to that face's sides
        const int f = s.code - 2;
        const int o1 = f == 0 ? 1 : 0, o2 = f == 2 ? 1 : 2;
        pool[0] = t.v0 - b.c, pool[1] = t.v1 - b.c, pool[2] = t.v2 - b.c;
        poolW[0] = poolW[1] = poolW[2] = 1;
        int poolN = 3;
        idxA[0] = 0, idxA[1] = 1, idxA[2] = 2;
        int n = clip(pool, poolW, poolN, idxA, 3, idxB, -nb, b.h[f]);
        if (n == 0) return 0;
        n = clip(pool, poolW, poolN, idxB, n, idxA, b.ax[o1], b.h[o1]);
        if (n == 0) return 0;
        n = clip(pool, poolW, poolN, idxA, n, idxB, -b.ax[o1], b.h[o1]);
        if (n == 0) return 0;
        n = clip(pool, poolW, poolN, idxB, n, idxA, b.ax[o2], b.h[o2]);
        if (n == 0) return 0;
        n = clip(pool, poolW, poolN, idxA, n, idxB, -b.ax[o2], b.h[o2]);
        n = std::min(n, 8);
        for (int k = 0; k < n; ++k) {
            const Vec3& v = pool[idxB[k]];
            const float dd = std::min(dot(nb, v) - b.h[f], 0.0f);
            pts[k] = b.c + v;
            depth[k] = -dd;
        }
        return n;
    }
    // The triangle face: the box face most anti-parallel to it, relative to v0, clipped by the triangle's back
    // plane and its three edge planes; depth = max(0, p . axis).
    float an[3];
    for (int i = 0; i < 3; ++i) an[i] = std::fabs(-dot(b.ax[i], nb));
    int k, j, l;
    if (an[1] <= an[0]) {
        if (an[2] < an[0])
            k = 0, j = 1, l = 2;
        else
            k = 2, j = 0, l = 1;
    } else if (an[2] < an[1]) {
        k = 1, j = 0, l = 2;
    } else {
        k = 2, j = 0, l = 1;
    }
    Vec3 rel = b.c - t.v0;
    const Vec3 hk = b.ax[k] * b.h[k];
    if (-dot(b.ax[k], nb) <= 0)
        rel += hk;
    else
        rel -= hk;
    const Vec3 ha = b.ax[j] * b.h[j], hb = b.ax[l] * b.h[l];
    // The corners in the engine's order (+a-b, -a+b, -a-b, +a+b: a crossed quad).
    pool[0] = rel + (ha - hb), pool[1] = rel - (ha - hb), pool[2] = rel - (ha + hb), pool[3] = rel + (ha + hb);
    for (int q = 0; q < 4; ++q) poolW[q] = 1;
    int poolN = 4;
    idxA[0] = 0, idxA[1] = 1, idxA[2] = 2, idxA[3] = 3;
    int n = clip(pool, poolW, poolN, idxA, 4, idxB, t.planeN[0], t.planeW[0]);
    if (n == 0) return 0;
    n = clip(pool, poolW, poolN, idxB, n, idxA, t.planeN[1], t.planeW[1]);
    if (n == 0) return 0;
    n = clip(pool, poolW, poolN, idxA, n, idxB, t.planeN[2], t.planeW[2]);
    if (n == 0) return 0;
    n = clip(pool, poolW, poolN, idxB, n, idxA, t.planeN[3], t.planeW[3]);
    if (n == 0) return 0;
    n = std::min(n, 8);
    for (int q = 0; q < n; ++q) {
        const Vec3& v = pool[idxA[q]];
        pts[q] = t.v0 + v;
        depth[q] = std::max(dot(v, nb), 0.0f);
    }
    return n;
}

}  // namespace fuel::fv
