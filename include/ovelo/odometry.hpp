#pragma once

#include "math.hpp"

namespace ovelo {
struct OdometryConfig {
    double leftOffset = 5;   // Robot-frame y: left is positive.
    double rightOffset = -5;
    double sideOffset = -3;  // Robot-frame x: forward is positive.
    bool useImu = true;
    bool useRight = true;    // false: left + sideways + IMU (2 tracking wheels).
    std::uint32_t maxGapMs = 250;
    double maxWheelSpeed = 150;   // Inches/second; reject disconnections/jumps.
    double maxAngularSpeed = 20;  // Radians/second.
};
struct SensorSample {
    double left = 0, right = 0, side = 0; // Cumulative wheel travel, inches.
    double heading = 0;                   // IMU radians CCW, any zero reference.
    std::uint32_t timeMs = 0;
};
enum class OdometryStatus { Waiting, Baseline, Updated, InvalidConfig, InvalidSample, Stale, Jump };
struct OdometryTrace {
    OdometryStatus status = OdometryStatus::Waiting;
    double rawLeft = 0, rawRight = 0, rawSide = 0;
    double dtheta = 0, forward = 0, lateral = 0, dx = 0, dy = 0;
};
class Odometry {
 public:
    explicit Odometry(OdometryConfig config = {}) : config_(config) {}
    Pose pose() const { return pose_; }
    bool healthy() const { return healthy_; }
    bool initialized() const { return initialized_; }
    // Only while motors are stopped: discard readings collected before a mode gap.
    void forgetBaseline() { initialized_ = healthy_ = false; trace_ = {}; }
    const OdometryTrace& trace() const { return trace_; }
    const OdometryConfig& config() const { return config_; }
    bool validConfig() const {
        return std::isfinite(config_.leftOffset) && std::isfinite(config_.rightOffset) &&
            std::isfinite(config_.sideOffset) && config_.maxGapMs > 0 &&
            config_.maxGapMs < 0x80000000u && std::isfinite(config_.maxWheelSpeed) &&
            config_.maxWheelSpeed > 0 && std::isfinite(config_.maxAngularSpeed) &&
            config_.maxAngularSpeed > 0 && (config_.useImu || (config_.useRight &&
            std::abs(config_.leftOffset - config_.rightOffset) > 1e-6));
    }
    // Set the field pose without resetting sensors or changing the sensor baseline.
    bool reset(Pose pose = {}) {
        if (!finite(pose)) return false;
        pose.theta = wrapAngle(pose.theta);
        pose_ = pose;
        return true;
    }
    bool update(SensorSample sample) {
        trace_ = {};
        if (!validConfig()) return reject(OdometryStatus::InvalidConfig);
        if (!std::isfinite(sample.left) || !std::isfinite(sample.side) ||
            (config_.useRight && !std::isfinite(sample.right)) ||
            (config_.useImu && !std::isfinite(sample.heading)))
            return reject(OdometryStatus::InvalidSample);
        if (!initialized_) {
            previous_ = sample;
            initialized_ = healthy_ = true;
            trace_.status = OdometryStatus::Baseline;
            return true;
        }
        const auto dtMs = elapsed(sample.timeMs, previous_.timeMs);
        if (!dtMs || dtMs > config_.maxGapMs) return reject(OdometryStatus::Stale);
        const double dt = dtMs / 1000.0;
        const double dl = sample.left - previous_.left;
        const double dr = config_.useRight ? sample.right - previous_.right : 0;
        const double ds = sample.side - previous_.side;
        const double da = config_.useImu ? wrapAngle(sample.heading - previous_.heading) :
            (dr - dl) / (config_.leftOffset - config_.rightOffset);
        trace_.rawLeft = dl; trace_.rawRight = dr; trace_.rawSide = ds; trace_.dtheta = da;
        if (std::max({std::abs(dl), std::abs(dr), std::abs(ds)}) > config_.maxWheelSpeed * dt ||
            std::abs(da) > config_.maxAngularSpeed * dt) return reject(OdometryStatus::Jump);
        // Wheel displacement = forward - y*dtheta, sideways + x*dtheta.
        double forward = dl + config_.leftOffset * da;
        if (config_.useRight) forward = (forward + dr + config_.rightOffset * da) / 2;
        const double lateral = ds - config_.sideOffset * da;
        const double scale = sinc(da / 2);
        const double mid = pose_.theta + da / 2;
        const double dx = scale * (forward * std::cos(mid) - lateral * std::sin(mid));
        const double dy = scale * (forward * std::sin(mid) + lateral * std::cos(mid));
        Pose next{pose_.x + dx, pose_.y + dy, wrapAngle(pose_.theta + da)};
        if (!finite(next)) return reject(OdometryStatus::Jump);
        pose_ = next;
        previous_ = sample;
        healthy_ = true;
        trace_.forward = forward; trace_.lateral = lateral; trace_.dx = dx; trace_.dy = dy;
        trace_.status = OdometryStatus::Updated;
        return true;
    }
 private:
    bool reject(OdometryStatus status) {
        trace_.status = status;
        initialized_ = healthy_ = false; // Next valid reading establishes a fresh baseline.
        return false;
    }
    OdometryConfig config_;
    Pose pose_{};
    SensorSample previous_{};
    OdometryTrace trace_{};
    bool initialized_ = false, healthy_ = false;
};
} // namespace ovelo
