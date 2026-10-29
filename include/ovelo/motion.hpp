#pragma once

#include "math.hpp"

namespace ovelo {
enum class MotionStatus { Idle, Running, Settled, TimedOut, Stalled, Invalid, Cancelled };
inline const char* statusName(MotionStatus status) {
    switch (status) {
        case MotionStatus::Idle: return "idle";
        case MotionStatus::Running: return "running";
        case MotionStatus::Settled: return "settled";
        case MotionStatus::TimedOut: return "timed out";
        case MotionStatus::Stalled: return "stalled";
        case MotionStatus::Invalid: return "invalid input or timing";
        case MotionStatus::Cancelled: return "cancelled";
    }
    return "unknown";
}
struct MotionConfig {
    double driveKp = 0.075;     // Normalized output / inch.
    double turnKp = 1.5;        // Normalized output / radian.
    double maxDrive = 0.65;
    double maxTurn = 0.6;
    double positionTolerance = 0.75; // Inches.
    double headingTolerance = radians(2);
    std::uint32_t settleMs = 150;
    std::uint32_t stallMs = 1000;
    std::uint32_t maxStepGapMs = 100;
    double distanceProgress = 0.1;
    double angleProgress = radians(0.5);
};
// A deliberately small point/turn controller. No background task and no implicit motor writes.
class MotionController {
 public:
    explicit MotionController(MotionConfig config = {}) : config_(config) {}
    MotionStatus status() const { return status_; }
    bool running() const { return status_ == MotionStatus::Running; }
    const MotionConfig& config() const { return config_; }
    void cancel() { if (running()) status_ = MotionStatus::Cancelled; }
    void moveToPoint(Point target, Pose pose, std::uint32_t nowMs, std::uint32_t timeoutMs) {
        target_ = target; turnOnly_ = false;
        begin(pose, nowMs, timeoutMs);
    }
    void turnTo(double heading, Pose pose, std::uint32_t nowMs, std::uint32_t timeoutMs) {
        heading_ = wrapAngle(heading); turnOnly_ = true;
        begin(pose, nowMs, timeoutMs);
    }
    DriveCommand step(Pose pose, std::uint32_t nowMs) {
        if (!running()) return {};
        if (!finite(pose) || elapsed(nowMs, previousMs_) == 0 ||
            elapsed(nowMs, previousMs_) > config_.maxStepGapMs) return finish(MotionStatus::Invalid);
        previousMs_ = nowMs;
        if (elapsed(nowMs, startMs_) >= timeoutMs_) return finish(MotionStatus::TimedOut);
        const double dist = turnOnly_ ? 0 : distance({pose.x, pose.y}, target_);
        const double error = wrapAngle((turnOnly_ ? heading_ :
            std::atan2(target_.y - pose.y, target_.x - pose.x)) - pose.theta);
        if (!std::isfinite(dist) || !std::isfinite(error)) return finish(MotionStatus::Invalid);
        const bool inside = turnOnly_ ? std::abs(error) <= config_.headingTolerance :
            dist <= config_.positionTolerance;
        if (inside) {
            if (!settling_) { settling_ = true; settledSince_ = nowMs; }
            if (elapsed(nowMs, settledSince_) >= config_.settleMs) return finish(MotionStatus::Settled);
            return {}; // Do not creep while checking the settling window.
        }
        settling_ = false;
        // Heading progress counts while aiming; distance progress counts while driving.
        const bool aiming = turnOnly_ || std::abs(error) > radians(45);
        const double progress = aiming ? std::abs(error) : dist;
        const double minimum = aiming ? config_.angleProgress : config_.distanceProgress;
        if (aiming != aiming_ || progress <= best_ - minimum) {
            aiming_ = aiming; best_ = progress; progressMs_ = nowMs;
        }
        if (elapsed(nowMs, progressMs_) >= config_.stallMs) return finish(MotionStatus::Stalled);
        const double turn = clamp(config_.turnKp * error, -config_.maxTurn, config_.maxTurn);
        const double forward = turnOnly_ ? 0 : std::min(config_.maxDrive, config_.driveKp * dist) *
            std::max(0.0, std::cos(error));
        return arcade(forward, turn);
    }
 private:
    bool validConfig() const {
        return std::isfinite(config_.driveKp) && config_.driveKp > 0 &&
            std::isfinite(config_.turnKp) && config_.turnKp > 0 &&
            std::isfinite(config_.maxDrive) && config_.maxDrive > 0 && config_.maxDrive <= 1 &&
            std::isfinite(config_.maxTurn) && config_.maxTurn > 0 && config_.maxTurn <= 1 &&
            std::isfinite(config_.positionTolerance) && config_.positionTolerance > 0 &&
            std::isfinite(config_.headingTolerance) && config_.headingTolerance > 0 &&
            config_.headingTolerance < pi && std::isfinite(config_.distanceProgress) &&
            config_.distanceProgress > 0 && std::isfinite(config_.angleProgress) &&
            config_.angleProgress > 0 && config_.stallMs > 0 && config_.stallMs < 0x80000000u &&
            config_.settleMs < 0x80000000u && config_.maxStepGapMs > 0 &&
            config_.maxStepGapMs < 0x80000000u;
    }
    void begin(Pose pose, std::uint32_t now, std::uint32_t timeout) {
        startMs_ = previousMs_ = progressMs_ = now; timeoutMs_ = timeout;
        best_ = 1e100; settling_ = false; aiming_ = false;
        status_ = (!validConfig() || !finite(pose) || !timeout || timeout >= 0x80000000u ||
            (turnOnly_ ? !std::isfinite(heading_) :
             (!std::isfinite(target_.x) || !std::isfinite(target_.y)))) ?
            MotionStatus::Invalid : MotionStatus::Running;
    }
    DriveCommand finish(MotionStatus status) { status_ = status; return {}; }
    MotionConfig config_;
    MotionStatus status_ = MotionStatus::Idle;
    Point target_{};
    double heading_ = 0, best_ = 0;
    bool turnOnly_ = false, settling_ = false, aiming_ = false;
    std::uint32_t startMs_ = 0, previousMs_ = 0, timeoutMs_ = 0,
                  settledSince_ = 0, progressMs_ = 0;
};
} // namespace ovelo
