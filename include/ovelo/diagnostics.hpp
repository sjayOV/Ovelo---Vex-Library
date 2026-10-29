#pragma once
#include "odometry.hpp"

namespace ovelo {
enum class HandTest { Forward, Left, Counterclockwise };
enum class DiagnosticStatus { Pass, WrongDirection, NoMotion, Invalid };
struct DiagnosticReport {
    DiagnosticStatus status = DiagnosticStatus::Invalid;
    const char* message = "Invalid sensor reading or geometry";
};
// The user supplies a known hand motion. This cannot discover wiring automatically.
inline DiagnosticReport diagnose(HandTest test, SensorSample before, SensorSample after,
                                 OdometryConfig config = {}) {
    if (!Odometry(config).validConfig() || !std::isfinite(before.left) ||
        !std::isfinite(after.left) || !std::isfinite(before.side) || !std::isfinite(after.side) ||
        (config.useRight && (!std::isfinite(before.right) || !std::isfinite(after.right))) ||
        (config.useImu && (!std::isfinite(before.heading) || !std::isfinite(after.heading)))) return {};
    const double l = after.left - before.left;
    const double r = config.useRight ? after.right - before.right : 0;
    const double s = after.side - before.side;
    if (test == HandTest::Forward) {
        if (std::abs(l) < 0.1 || (config.useRight && std::abs(r) < 0.1))
            return {DiagnosticStatus::NoMotion, "Push farther; check forward wheel contact and ports"};
        if (l < 0 || (config.useRight && r < 0))
            return {DiagnosticStatus::WrongDirection, "Forward wheel sign reversed"};
    } else if (test == HandTest::Left) {
        if (std::abs(s) < 0.1) return {DiagnosticStatus::NoMotion, "Push left farther; check sideways wheel"};
        if (s < 0) return {DiagnosticStatus::WrongDirection, "Sideways wheel must increase when pushed left"};
    } else {
        const double a = config.useImu ? wrapAngle(after.heading - before.heading) :
            (r - l) / (config.leftOffset - config.rightOffset);
        if (std::abs(a) < radians(3)) return {DiagnosticStatus::NoMotion, "Rotate CCW farther; check heading source"};
        if (a < 0 || (std::abs(config.leftOffset) > 0.1 && l * config.leftOffset > 0.1) ||
            (config.useRight && std::abs(config.rightOffset) > 0.1 && r * config.rightOffset > 0.1) ||
            (std::abs(config.sideOffset) > 0.1 && s * config.sideOffset < -0.1))
            return {DiagnosticStatus::WrongDirection, "Check heading sign, wheel signs, and signed offsets"};
    }
    return {DiagnosticStatus::Pass, "Direction check passed; measure scale and rotation drift next"};
}
} // namespace ovelo
