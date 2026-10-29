#include "main.h"
#include "ovelo/ovelo.hpp"
#include "ovelo/localization.hpp" // Compile-check the optional portable module too.

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdint>
#include <cstdio>
#include <limits>

namespace {
// Edit ALL geometry, ports, motor reversals, and sensor signs for your robot.
// Leave false until the sensor checklist in docs/02-sensor-checks.md passes.
constexpr bool hardwareConfirmed = false;
constexpr std::int8_t leftPorts[] = {1, 2};
constexpr std::int8_t rightPorts[] = {3, 4};
constexpr std::uint8_t leftSensor = 7;
constexpr std::uint8_t rightSensor = 8;
constexpr std::uint8_t sideSensor = 9;
constexpr std::uint8_t imuPort = 10;
constexpr double trackingDiameter = 2.75; // inches, measure under load
constexpr double leftSign = 1.0;
constexpr double rightSign = 1.0;
constexpr double sideSign = 1.0;
constexpr double pi = 3.14159265358979323846;

ovelo::OdometryConfig geometry() {
    ovelo::OdometryConfig result;
    result.leftOffset = 5.0;   // signed Y: left wheel is left of robot center
    result.rightOffset = -5.0;
    result.sideOffset = -3.0;  // signed X: sideways wheel is behind center
    result.useImu = true;
    result.useRight = true;   // false uses left + sideways + IMU only.
    return result;
}

ovelo::Odometry odom(geometry());
ovelo::MotionController motion;
bool sensorsReady = false;
bool motorsReady = false;
ovelo::SensorSample lastSample;

void stopDrive() {
    for (const auto port : leftPorts) pros::c::motor_move_voltage(port, 0);
    for (const auto port : rightPorts) pros::c::motor_move_voltage(port, 0);
}

bool configureMotor(std::int8_t port, bool reversed) {
    errno = 0;
    const auto gearing = pros::c::motor_set_gearing(port, pros::E_MOTOR_GEARSET_18);
    const bool gearOk = gearing == 1 && errno == 0;
    errno = 0;
    const auto reversal = pros::c::motor_set_reversed(port, reversed);
    return gearOk && reversal == 1 && errno == 0;
}

// Absolute Rotation Sensor positions are centidegrees, not motor encoder ticks.
// No resets are performed: Odometry takes the first reading as its baseline.
double wheelInches(std::uint8_t port, double sign) {
    errno = 0;
    const auto position = pros::c::rotation_get_position(port);
    if (errno != 0 || position == std::numeric_limits<std::int32_t>::max())
        return std::numeric_limits<double>::quiet_NaN();
    return sign * static_cast<double>(position) / 36000.0 * pi * trackingDiameter;
}

ovelo::SensorSample readSensors() {
    ovelo::SensorSample sample;
    sample.left = wheelInches(leftSensor, leftSign);
    sample.right = odom.config().useRight ? wheelInches(rightSensor, rightSign) : 0;
    sample.side = wheelInches(sideSensor, sideSign);
    if (!odom.config().useImu) {
        sample.timeMs = pros::millis();
        return sample; // Three-wheel heading; no IMU connected or required.
    }
    errno = 0;
    const auto status = pros::c::imu_get_status(imuPort);
    const bool imuReady = errno == 0 && status != pros::E_IMU_STATUS_ERROR &&
        !(status & pros::E_IMU_STATUS_CALIBRATING);
    errno = 0;
    const double degrees = pros::c::imu_get_rotation(imuPort);
    sample.heading = imuReady && errno == 0 && std::isfinite(degrees)
        ? -degrees * pi / 180.0 // PROS clockwise -> Ovelo counterclockwise
        : std::numeric_limits<double>::quiet_NaN();
    sample.timeMs = pros::millis();
    return sample;
}

bool updatePose() {
    if (sensorsReady) lastSample = readSensors();
    if (!sensorsReady || !odom.update(lastSample)) {
        // A reconnect requires deliberate reinitialization, not automatic motion.
        sensorsReady = false;
        motion.cancel();
        stopDrive();
        return false;
    }
    return true;
}

bool beginControl() {
    // No updater runs while disabled. Discard that old time/distance baseline
    // deliberately at mode entry; do not clear a fault latched during operation.
    if (!sensorsReady) return false;
    odom.forgetBaseline();
    lastSample = readSensors();
    sensorsReady = odom.update(lastSample);
    if (!sensorsReady) stopDrive();
    return sensorsReady;
}

void drive(ovelo::DriveCommand command) {
    if (!hardwareConfirmed || !sensorsReady || !motorsReady ||
        pros::competition::is_disabled() ||
        !std::isfinite(command.left) || !std::isfinite(command.right)) {
        stopDrive();
        return;
    }
    const auto voltage = [](double value) {
        return static_cast<std::int32_t>(std::clamp(value, -1.0, 1.0) * 12000.0);
    };
    bool ok = true;
    for (const auto port : leftPorts) {
        errno = 0;
        if (pros::c::motor_move_voltage(port, voltage(command.left)) != 1 || errno != 0)
            ok = false;
    }
    for (const auto port : rightPorts) {
        errno = 0;
        if (pros::c::motor_move_voltage(port, voltage(command.right)) != 1 || errno != 0)
            ok = false;
    }
    if (!ok) {
        motorsReady = false;
        motion.cancel();
        stopDrive();
    }
}

double joystick(pros::controller_analog_e_t channel) {
    errno = 0;
    const auto raw = pros::c::controller_get_analog(pros::E_CONTROLLER_MASTER, channel);
    if (errno != 0 || std::abs(raw) > 127) return 0.0;
    return std::abs(raw) < 8 ? 0.0 : static_cast<double>(raw) / 127.0;
}
} // namespace

void initialize() {
    stopDrive();
    sensorsReady = false;
    motorsReady = true;
    for (const auto port : leftPorts) {
        if (!configureMotor(port, false)) motorsReady = false;
    }
    for (const auto port : rightPorts) {
        if (!configureMotor(port, true)) motorsReady = false;
    }
    if (odom.config().useImu) {
        errno = 0;
        if (pros::c::imu_reset(imuPort) != 1 || errno != 0) {
            std::printf("Ovelo: IMU reset failed; drive disabled.\n");
            return;
        }
        // Keep the robot still. Calibration runs in firmware, with an 8-second cap.
        const auto start = pros::millis();
        pros::delay(100);
        while (true) {
            errno = 0;
            const auto status = pros::c::imu_get_status(imuPort);
            if (errno != 0 || status == pros::E_IMU_STATUS_ERROR) {
                std::printf("Ovelo: IMU unavailable; drive disabled.\n");
                return;
            }
            if (!(status & pros::E_IMU_STATUS_CALIBRATING)) break;
            if (static_cast<std::uint32_t>(pros::millis() - start) >= 8000) {
                std::printf("Ovelo: calibration timed out; drive disabled.\n");
                return;
            }
            pros::delay(10);
        }
    }
    lastSample = readSensors();
    sensorsReady = odom.update(lastSample);
    odom.reset({0.0, 0.0, 0.0});
    std::printf("Ovelo: sensors=%s motors=%s hardwareConfirmed=%s\n",
        sensorsReady ? "ready" : "failed", motorsReady ? "ready" : "failed",
        hardwareConfirmed ? "true" : "false (drive inhibited)");
}

void disabled() {
    motion.cancel();
    stopDrive();
}

void competition_initialize() {
    // Do not calibrate here: a calibration belongs in initialize() while still.
    stopDrive();
}

void autonomous() {
    stopDrive();
    if (!hardwareConfirmed || !motorsReady || !beginControl()) return;
    // Place the robot at this known pose BEFORE running autonomous.
    odom.reset({0.0, 0.0, 0.0});
    motion.moveToPoint({24.0, 0.0}, odom.pose(), pros::millis(), 4000);
    while (motion.status() == ovelo::MotionStatus::Running &&
           !pros::competition::is_disabled() && pros::competition::is_autonomous()) {
        pros::delay(10);
        if (pros::competition::is_disabled() || !pros::competition::is_autonomous()) break;
        if (!updatePose()) break;
        drive(motion.step(odom.pose(), pros::millis()));
    }
    stopDrive();
    if (motion.running()) motion.cancel();
    std::printf("Ovelo: autonomous result=%s\n", ovelo::statusName(motion.status()));
}

void opcontrol() {
    motion.cancel();
    stopDrive();
    beginControl();
    auto telemetryTime = pros::millis();
    while (!pros::competition::is_autonomous() && !pros::competition::is_disabled()) {
        pros::delay(10);
        if (updatePose() && !pros::competition::is_disabled()) {
            const double forward = joystick(pros::E_CONTROLLER_ANALOG_LEFT_Y);
            // Right stick right is clockwise, hence negative core turn direction.
            const double clockwise = joystick(pros::E_CONTROLLER_ANALOG_RIGHT_X);
            const double left = forward + clockwise;
            const double right = forward - clockwise;
            const double scale = std::max(1.0, std::max(std::abs(left), std::abs(right)));
            drive({left / scale, right / scale});
        } else {
            stopDrive();
        }
        if (static_cast<std::uint32_t>(pros::millis() - telemetryTime) >= 250) {
            telemetryTime = pros::millis();
            const auto pose = odom.pose();
            std::printf("Ovelo wheels[in] L=%.3f R=%.3f S=%.3f imu[rad]=%.3f "
                        "pose[in,rad]=(%.3f,%.3f,%.3f) sensors=%s drive=%s\n",
                lastSample.left, lastSample.right, lastSample.side, lastSample.heading,
                pose.x, pose.y, pose.theta, sensorsReady ? "ready" : "FAULT",
                hardwareConfirmed && sensorsReady && motorsReady ? "enabled" : "inhibited");
        }
    }
    motion.cancel();
    stopDrive();
}
