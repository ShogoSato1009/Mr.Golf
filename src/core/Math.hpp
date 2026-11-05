#pragma once

#include <cmath>

namespace golf {

struct Vec3 {
    double x = 0.0;
    double y = 0.0;
    double z = 0.0;

    Vec3& operator+=(Vec3 b) { x += b.x; y += b.y; z += b.z; return *this; }
    Vec3& operator-=(Vec3 b) { x -= b.x; y -= b.y; z -= b.z; return *this; }
    Vec3& operator*=(double s) { x *= s; y *= s; z *= s; return *this; }
};

inline Vec3 operator+(Vec3 a, Vec3 b) { return a += b; }
inline Vec3 operator-(Vec3 a, Vec3 b) { return a -= b; }
inline Vec3 operator-(Vec3 a) { return {-a.x, -a.y, -a.z}; }
inline Vec3 operator*(Vec3 a, double s) { return a *= s; }
inline Vec3 operator*(double s, Vec3 a) { return a *= s; }
inline Vec3 operator/(Vec3 a, double s) { return a * (1.0 / s); }
inline double dot(Vec3 a, Vec3 b) { return a.x*b.x + a.y*b.y + a.z*b.z; }
inline Vec3 cross(Vec3 a, Vec3 b) {
    return {a.y*b.z - a.z*b.y, a.z*b.x - a.x*b.z, a.x*b.y - a.y*b.x};
}
inline double length(Vec3 a) { return std::hypot(a.x, a.y, a.z); }
inline Vec3 tangent(Vec3 a, Vec3 unitNormal) { return a - dot(a, unitNormal)*unitNormal; }
inline bool isFinite(Vec3 a) {
    return std::isfinite(a.x) && std::isfinite(a.y) && std::isfinite(a.z);
}

inline constexpr double pi = 3.14159265358979323846;

} // namespace golf
