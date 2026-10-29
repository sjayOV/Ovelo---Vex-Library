#pragma once

#include <algorithm>
#include <cmath>
#include <cstdint>

namespace ovelo {
constexpr double pi = 3.14159265358979323846;
constexpr double radians(double degrees) { return degrees * pi / 180.0; }
constexpr double degrees(double angle) { return angle * 180.0 / pi; }
struct Point { double x = 0; double y = 0; };
// Inches, radians counterclockwise; heading zero points along field +X.
struct Pose { double x = 0; double y = 0; double theta = 0; };
inline bool finite(Pose p) {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.theta);
}
inline double wrapAngle(double angle) {
    return std::isfinite(angle) ? std::remainder(angle, 2.0 * pi) : angle;
}
inline double clamp(double x, double low, double high) {
    return std::max(low, std::min(x, high));
}
inline double distance(Point a, Point b) { return std::hypot(a.x - b.x, a.y - b.y); }
inline double sinc(double angle) {
    if (std::abs(angle) < 1e-6) return 1.0 - angle * angle / 6.0;
    return std::sin(angle) / angle;
}
// Unsigned subtraction remains valid across a millis() rollover.
inline std::uint32_t elapsed(std::uint32_t now, std::uint32_t then) { return now - then; }
inline double wheelInches(double centidegrees, double diameter, double wheelPerSensor = 1) {
    return centidegrees / 36000.0 * pi * diameter * wheelPerSensor;
}
struct DriveCommand { double left = 0; double right = 0; };
inline DriveCommand arcade(double forward, double ccwTurn) {
    if (!std::isfinite(forward) || !std::isfinite(ccwTurn)) return {};
    const double inputScale = std::max(1.0, std::max(std::abs(forward), std::abs(ccwTurn)));
    forward /= inputScale;
    ccwTurn /= inputScale;
    double left = forward - ccwTurn;
    double right = forward + ccwTurn;
    const double scale = std::max(1.0, std::max(std::abs(left), std::abs(right)));
    return {left / scale, right / scale};
}
} // namespace ovelo
