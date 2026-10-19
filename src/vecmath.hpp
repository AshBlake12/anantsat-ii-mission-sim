// Small, dependency-free 3D math: vectors, 3x3 matrices, unit quaternions.
#pragma once
#include <algorithm>
#include <cmath>

namespace cs {

constexpr double PI = 3.14159265358979323846;
constexpr double TWO_PI = 2.0 * PI;
constexpr double DEG = PI / 180.0;
constexpr double RAD = 180.0 / PI;

struct Vec3 {
    double x = 0, y = 0, z = 0;
    constexpr Vec3() = default;
    constexpr Vec3(double x_, double y_, double z_) : x(x_), y(y_), z(z_) {}
    Vec3 operator+(const Vec3& o) const { return {x + o.x, y + o.y, z + o.z}; }
    Vec3 operator-(const Vec3& o) const { return {x - o.x, y - o.y, z - o.z}; }
    Vec3 operator-() const { return {-x, -y, -z}; }
    Vec3 operator*(double s) const { return {x * s, y * s, z * s}; }
    Vec3 operator/(double s) const { return {x / s, y / s, z / s}; }
    Vec3& operator+=(const Vec3& o) { x += o.x; y += o.y; z += o.z; return *this; }
    Vec3& operator-=(const Vec3& o) { x -= o.x; y -= o.y; z -= o.z; return *this; }
    Vec3& operator*=(double s) { x *= s; y *= s; z *= s; return *this; }
};
inline Vec3 operator*(double s, const Vec3& v) { return v * s; }
inline double dot(const Vec3& a, const Vec3& b) { return a.x * b.x + a.y * b.y + a.z * b.z; }
inline Vec3 cross(const Vec3& a, const Vec3& b) {
    return {a.y * b.z - a.z * b.y, a.z * b.x - a.x * b.z, a.x * b.y - a.y * b.x};
}
inline double norm(const Vec3& a) { return std::sqrt(dot(a, a)); }
inline Vec3 unit(const Vec3& a) {
    double n = norm(a);
    return n > 0 ? a / n : Vec3{0, 0, 0};
}
// Angle between two vectors, robust near 0 and pi.
inline double angle_between(const Vec3& a, const Vec3& b) {
    return std::atan2(norm(cross(a, b)), dot(a, b));
}
// Any unit vector perpendicular to a.
inline Vec3 any_perp(const Vec3& a) {
    Vec3 t = std::fabs(a.x) < 0.9 ? Vec3{1, 0, 0} : Vec3{0, 1, 0};
    return unit(cross(a, t));
}

struct Mat3 {
    double m[3][3] = {{1, 0, 0}, {0, 1, 0}, {0, 0, 1}};
    static Mat3 from_cols(const Vec3& c0, const Vec3& c1, const Vec3& c2) {
        Mat3 r;
        r.m[0][0] = c0.x; r.m[1][0] = c0.y; r.m[2][0] = c0.z;
        r.m[0][1] = c1.x; r.m[1][1] = c1.y; r.m[2][1] = c1.z;
        r.m[0][2] = c2.x; r.m[1][2] = c2.y; r.m[2][2] = c2.z;
        return r;
    }
    static Mat3 rot_z(double a) {
        Mat3 r;
        double c = std::cos(a), s = std::sin(a);
        r.m[0][0] = c; r.m[0][1] = -s;
        r.m[1][0] = s; r.m[1][1] = c;
        return r;
    }
    Vec3 operator*(const Vec3& v) const {
        return {m[0][0] * v.x + m[0][1] * v.y + m[0][2] * v.z,
                m[1][0] * v.x + m[1][1] * v.y + m[1][2] * v.z,
                m[2][0] * v.x + m[2][1] * v.y + m[2][2] * v.z};
    }
    Mat3 operator*(const Mat3& o) const {
        Mat3 r;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) {
                r.m[i][j] = 0;
                for (int k = 0; k < 3; ++k) r.m[i][j] += m[i][k] * o.m[k][j];
            }
        return r;
    }
    Mat3 T() const {
        Mat3 r;
        for (int i = 0; i < 3; ++i)
            for (int j = 0; j < 3; ++j) r.m[i][j] = m[j][i];
        return r;
    }
};

// Unit quaternion, scalar-first. q represents the rotation that maps
// body-frame vectors into the inertial frame: v_I = q * v_B * q^-1.
struct Quat {
    double w = 1, x = 0, y = 0, z = 0;
    constexpr Quat() = default;
    constexpr Quat(double w_, double x_, double y_, double z_) : w(w_), x(x_), y(y_), z(z_) {}

    Quat operator*(const Quat& b) const {
        return {w * b.w - x * b.x - y * b.y - z * b.z,
                w * b.x + x * b.w + y * b.z - z * b.y,
                w * b.y - x * b.z + y * b.w + z * b.x,
                w * b.z + x * b.y - y * b.x + z * b.w};
    }
    Quat conj() const { return {w, -x, -y, -z}; }
    Quat normalized() const {
        double n = std::sqrt(w * w + x * x + y * y + z * z);
        return {w / n, x / n, y / n, z / n};
    }
    Vec3 rotate(const Vec3& v) const {
        // v' = v + 2w(u x v) + 2 u x (u x v)
        Vec3 u{x, y, z};
        Vec3 t = 2.0 * cross(u, v);
        return v + w * t + cross(u, t);
    }
    static Quat from_axis_angle(const Vec3& axis, double ang) {
        Vec3 a = unit(axis);
        double s = std::sin(ang / 2);
        return {std::cos(ang / 2), a.x * s, a.y * s, a.z * s};
    }
    // Rotation vector (axis * angle), shortest path.
    Vec3 to_rotvec() const {
        Quat q = w < 0 ? Quat{-w, -x, -y, -z} : *this;
        double s = std::sqrt(q.x * q.x + q.y * q.y + q.z * q.z);
        if (s < 1e-12) return {2 * q.x, 2 * q.y, 2 * q.z};
        double ang = 2.0 * std::atan2(s, q.w);
        return Vec3{q.x, q.y, q.z} * (ang / s);
    }
    static Quat from_rotvec(const Vec3& rv) {
        double ang = norm(rv);
        if (ang < 1e-12) return Quat{1, rv.x / 2, rv.y / 2, rv.z / 2}.normalized();
        return from_axis_angle(rv, ang);
    }
    static Quat from_matrix(const Mat3& R) {
        const auto& m = R.m;
        double tr = m[0][0] + m[1][1] + m[2][2];
        Quat q;
        if (tr > 0) {
            double s = std::sqrt(tr + 1.0) * 2;
            q = {0.25 * s, (m[2][1] - m[1][2]) / s, (m[0][2] - m[2][0]) / s, (m[1][0] - m[0][1]) / s};
        } else if (m[0][0] > m[1][1] && m[0][0] > m[2][2]) {
            double s = std::sqrt(1.0 + m[0][0] - m[1][1] - m[2][2]) * 2;
            q = {(m[2][1] - m[1][2]) / s, 0.25 * s, (m[0][1] + m[1][0]) / s, (m[0][2] + m[2][0]) / s};
        } else if (m[1][1] > m[2][2]) {
            double s = std::sqrt(1.0 + m[1][1] - m[0][0] - m[2][2]) * 2;
            q = {(m[0][2] - m[2][0]) / s, (m[0][1] + m[1][0]) / s, 0.25 * s, (m[1][2] + m[2][1]) / s};
        } else {
            double s = std::sqrt(1.0 + m[2][2] - m[0][0] - m[1][1]) * 2;
            q = {(m[1][0] - m[0][1]) / s, (m[0][2] + m[2][0]) / s, (m[1][2] + m[2][1]) / s, 0.25 * s};
        }
        return q.normalized();
    }
};

// Rotation angle between two attitudes, in [0, pi].
inline double quat_angle(const Quat& a, const Quat& b) {
    Quat d = b * a.conj();
    return norm(d.to_rotvec());
}

inline double wrap_2pi(double a) {
    a = std::fmod(a, TWO_PI);
    return a < 0 ? a + TWO_PI : a;
}
inline double wrap_pi(double a) {
    a = wrap_2pi(a);
    return a > PI ? a - TWO_PI : a;
}

}  // namespace cs
