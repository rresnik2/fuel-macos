// Small float vector/quaternion/matrix helpers for FUEL's vehicle model (fuel_vehicle.*,
// fuel_contacts.*). FUEL stores floats and computes on the x87 at 53-bit precision; the
// port computes in float, which agrees with the emulated test vectors to ~1e-6 relative.
#pragma once

#include <algorithm>
#include <cmath>

namespace fuel::fv {

struct Vec3 {
    float x = 0, y = 0, z = 0;
    constexpr Vec3() = default;
    constexpr Vec3(float a, float b, float c) : x(a), y(b), z(c) {}
    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator-() const { return {-x, -y, -z}; }
    Vec3 operator*(float s) const { return {x * s, y * s, z * s}; }
    Vec3 operator/(float s) const { return {x / s, y / s, z / s}; }
    Vec3& operator+=(const Vec3& o) { x += o.x, y += o.y, z += o.z; return *this; }
    Vec3& operator-=(const Vec3& o) { x -= o.x, y -= o.y, z -= o.z; return *this; }
    Vec3& operator*=(float s) { x *= s, y *= s, z *= s; return *this; }
    float operator[](int i) const { return i == 0 ? x : i == 1 ? y : z; }
};
inline Vec3 operator*(float s, const Vec3& v) { return v * s; }
inline float dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 cross(const Vec3& a, const Vec3& b) { return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x}; }
inline float len2(const Vec3& a) { return dot(a, a); }
inline float len(const Vec3& a) { return std::sqrt(dot(a, a)); }
inline Vec3 mulc(const Vec3& a, const Vec3& b) { return {a.x * b.x, a.y * b.y, a.z * b.z}; }
// Normalised copy; a zero vector stays zero.
inline Vec3 normalize(const Vec3& a) {
    const float l = len(a);
    return l > 0 ? a / l : a;
}
inline float clampf(float x, float lo, float hi) { return x < lo ? lo : (x > hi ? hi : x); }
inline float lerpf(float a, float b, float t) { return a + (b - a) * t; }

// Rotation quaternion (x, y, z, w), Hamilton product; rotate(q, v) = q v q*.
struct Quat {
    float x = 0, y = 0, z = 0, w = 1;
};
inline Quat mul(const Quat& a, const Quat& b) {
    return {a.w * b.x + a.x * b.w + a.y * b.z - a.z * b.y, a.w * b.y - a.x * b.z + a.y * b.w + a.z * b.x,
            a.w * b.z + a.x * b.y - a.y * b.x + a.z * b.w, a.w * b.w - a.x * b.x - a.y * b.y - a.z * b.z};
}
inline Quat conj(const Quat& q) { return {-q.x, -q.y, -q.z, q.w}; }
inline Vec3 rotate(const Quat& q, const Vec3& v) {
    const Vec3 u{q.x, q.y, q.z};
    const Vec3 t = cross(u, v) * 2.0f;
    return v + t * q.w + cross(u, t);
}
inline Vec3 unrotate(const Quat& q, const Vec3& v) { return rotate(conj(q), v); }
inline Quat qnormalize(const Quat& q) {
    const float l = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z + q.w * q.w);
    return l > 0 ? Quat{q.x / l, q.y / l, q.z / l, q.w / l} : Quat{};
}
inline Quat axisAngle(const Vec3& axis, float angle) {
    const Vec3 a = normalize(axis) * std::sin(angle * 0.5f);
    return {a.x, a.y, a.z, std::cos(angle * 0.5f)};
}

struct Mat3 {
    float m[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    static Mat3 zero() {
        Mat3 r;
        for (auto& row : r.m)
            for (float& e : row) e = 0;
        return r;
    }
    static Mat3 diag(float a, float b, float c) {
        Mat3 r = zero();
        r.m[0][0] = a, r.m[1][1] = b, r.m[2][2] = c;
        return r;
    }
    static Mat3 fromQuat(const Quat& q) {  // columns = rotated basis vectors
        Mat3 r;
        const Vec3 c[3] = {rotate(q, {1, 0, 0}), rotate(q, {0, 1, 0}), rotate(q, {0, 0, 1})};
        for (int j = 0; j < 3; ++j) r.m[0][j] = c[j].x, r.m[1][j] = c[j].y, r.m[2][j] = c[j].z;
        return r;
    }
    Vec3 operator*(const Vec3& v) const {
        return {m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z, m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
                m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z};
    }
    Mat3 operator*(const Mat3& o) const {
        Mat3 r = zero();
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j)
                for (int k = 0; k < 3; ++k) r.m[i][j] += m[i][k] * o.m[k][j];
        return r;
    }
    Mat3 operator+(const Mat3& o) const {
        Mat3 r;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) r.m[i][j] = m[i][j] + o.m[i][j];
        return r;
    }
    Mat3 operator*(float s) const {
        Mat3 r;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) r.m[i][j] = m[i][j] * s;
        return r;
    }
    Mat3 transposed() const {
        Mat3 r;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) r.m[i][j] = m[j][i];
        return r;
    }
    Mat3 inverse() const {
        const float a = m[0][0], b = m[0][1], c = m[0][2], d = m[1][0], e = m[1][1], f = m[1][2], g = m[2][0], h = m[2][1],
                    i = m[2][2];
        const float A = e * i - f * h, B = -(d * i - f * g), C = d * h - e * g;
        const float det = a * A + b * B + c * C;
        if (std::fabs(det) < 1e-20f) return diag(0, 0, 0);
        const float s = 1.0f / det;
        Mat3 r;
        r.m[0][0] = A * s, r.m[0][1] = -(b * i - c * h) * s, r.m[0][2] = (b * f - c * e) * s;
        r.m[1][0] = B * s, r.m[1][1] = (a * i - c * g) * s, r.m[1][2] = -(a * f - c * d) * s;
        r.m[2][0] = C * s, r.m[2][1] = -(a * h - b * g) * s, r.m[2][2] = (a * e - b * d) * s;
        return r;
    }
};

// FUEL's exponential smoother (Math_ExpSmooth 0x4100b0): out = cur + (tgt - cur)(1 - k^(-dt*30.30303)),
// or tgt when they are within 1e-6.
inline float expSmooth(float cur, float tgt, float k, float dt) {
    if (std::fabs(tgt - cur) < 1e-6f) return tgt;
    return cur + (tgt - cur) * (1.0f - std::pow(k, -dt * 30.30303f));
}
// smoothstep used by the friction curve and the ground generator: s = 2t-1; 0.5 + (2-|s|)s/2.
inline float sstep(float t) {
    const float s = 2.0f * clampf(t, 0, 1) - 1.0f;
    return 0.5f + (2.0f - std::fabs(s)) * s * 0.5f;
}

}  // namespace fuel::fv
