#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <algorithm>
#include <cmath>

struct Vec2 {
    double x = 0.0;
    double y = 0.0;
};

struct Pose2D {
    Vec2 position;
    double headingRadians = 0.0;
    double curvature = 0.0; // inverse metres
};

inline Vec2 operator+(Vec2 a, Vec2 b) { return {a.x + b.x, a.y + b.y}; }
inline Vec2 operator-(Vec2 a, Vec2 b) { return {a.x - b.x, a.y - b.y}; }
inline Vec2 operator*(Vec2 value, double scalar) {
    return {value.x * scalar, value.y * scalar};
}
inline Vec2 operator*(double scalar, Vec2 value) { return value * scalar; }
inline Vec2 operator/(Vec2 value, double scalar) {
    return std::fabs(scalar) > 1e-12 ? value * (1.0 / scalar) : Vec2{};
}

inline double dot(Vec2 a, Vec2 b) { return a.x * b.x + a.y * b.y; }
inline double cross(Vec2 a, Vec2 b) { return a.x * b.y - a.y * b.x; }
inline double lengthSquared(Vec2 value) { return dot(value, value); }
inline double length(Vec2 value) { return std::sqrt(lengthSquared(value)); }
inline double distance(Vec2 a, Vec2 b) { return length(b - a); }
inline Vec2 normalized(Vec2 value, Vec2 fallback = {1.0, 0.0}) {
    const double magnitude = length(value);
    return magnitude > 1e-12 ? value / magnitude : fallback;
}
inline Vec2 lerp(Vec2 a, Vec2 b, double t) {
    return a + (b - a) * std::clamp(t, 0.0, 1.0);
}
inline Vec2 rightNormal(Vec2 direction) {
    const Vec2 unit = normalized(direction);
    return {unit.y, -unit.x};
}

#endif
