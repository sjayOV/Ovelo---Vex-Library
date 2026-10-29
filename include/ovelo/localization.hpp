#pragma once

#include "math.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <cstdint>
#include <limits>
#include <random>

namespace ovelo {

// Inches, with the same +X / +Y / counterclockwise convention as odometry.
struct FieldBounds {
    double minX = 0;
    double minY = 0;
    double maxX = 144;
    double maxY = 144;
};

// The beam starts at (x,y) in the robot frame: +x forward, +y left.
// angle is counterclockwise from robot forward. sigma is range noise in inches.
struct RangeSensor {
    double x = 0;
    double y = 0;
    double angle = 0;
    double minRange = 2;
    double maxRange = 78;
    double sigma = 2;
};

struct RangeObservation {
    RangeSensor sensor{};
    double distance = 0;
};

namespace localization_detail {
inline bool finitePose(const Pose& p) {
    return std::isfinite(p.x) && std::isfinite(p.y) && std::isfinite(p.theta);
}
inline bool validField(const FieldBounds& f) {
    return std::isfinite(f.minX) && std::isfinite(f.minY) &&
           std::isfinite(f.maxX) && std::isfinite(f.maxY) &&
           f.minX < f.maxX && f.minY < f.maxY &&
           std::isfinite(f.maxX - f.minX) && std::isfinite(f.maxY - f.minY);
}
inline bool contains(const FieldBounds& f, double x, double y) {
    return x >= f.minX && x <= f.maxX && y >= f.minY && y <= f.maxY;
}
inline bool validSensor(const RangeSensor& s) {
    return std::isfinite(s.x) && std::isfinite(s.y) && std::isfinite(s.angle) &&
           std::isfinite(s.minRange) && std::isfinite(s.maxRange) &&
           std::isfinite(s.sigma) && s.minRange >= 0 &&
           s.maxRange > s.minRange && s.sigma > 0;
}
} // namespace localization_detail

// Rectangular walls only: game objects, other robots and angled/occluded walls
// are deliberately outside this model. NaN means no valid prediction.
inline double raycastWall(const Pose& p, const RangeSensor& sensor,
                          const FieldBounds& field = {}) {
    using namespace localization_detail;
    const double invalid = std::numeric_limits<double>::quiet_NaN();
    if (!finitePose(p) || !validField(field) || !validSensor(sensor)) return invalid;
    const double c = std::cos(p.theta);
    const double s = std::sin(p.theta);
    const double x = p.x + c * sensor.x - s * sensor.y;
    const double y = p.y + s * sensor.x + c * sensor.y;
    if (!contains(field, x, y)) return invalid;
    const double dx = std::cos(p.theta + sensor.angle);
    const double dy = std::sin(p.theta + sensor.angle);
    double distance = std::numeric_limits<double>::infinity();
    // Use the wall in the direction of travel. At a boundary this correctly
    // distinguishes a beam pointing into the field from one pointing outward.
    if (dx > 1e-12) distance = std::min(distance, (field.maxX - x) / dx);
    if (dx < -1e-12) distance = std::min(distance, (field.minX - x) / dx);
    if (dy > 1e-12) distance = std::min(distance, (field.maxY - y) / dy);
    if (dy < -1e-12) distance = std::min(distance, (field.minY - y) / dy);
    return std::isfinite(distance) && distance >= 0 ? distance : invalid;
}

// Optional bounded particle filter. It owns no heap memory and never changes
// Odometry: the caller must explicitly decide whether to use its estimate.
// N=128 uses about 13 KiB including scratch storage and the random generator.
// Place globally or static; do not consume a small V5 task's stack with it.
template <std::size_t N = 128>
class ParticleLocalizer {
    static_assert(N >= 2, "ParticleLocalizer needs at least two particles");

public:
    struct Particle {
        Pose pose{};
        double weight = 1.0 / N;
    };

    explicit ParticleLocalizer(FieldBounds field = {}, std::uint32_t seed = 1)
        : field_(field), random_(seed), valid_(localization_detail::validField(field)) {
        if (valid_) reset({field.minX + (field.maxX - field.minX) / 2,
                           field.minY + (field.maxY - field.minY) / 2, 0}, 0, 0);
    }

    bool valid() const { return valid_; }
    // false after the model loses all possible hypotheses; reset is required.
    bool hasEstimate() const { return valid_ && estimateAvailable_; }
    bool lastObservationAccepted() const { return accepted_; }
    // ESS before the most recent resampling; N means uniform weights.
    double effectiveParticleCount() const { return effectiveCount_; }
    const std::array<Particle, N>& particles() const { return particles_; }

    // Gaussian spread around a known starting pose, clipped to the rectangle.
    // A narrow prior is recommended; one range sensor cannot determine pose.
    bool reset(Pose center, double xySpread = 1, double headingSpread = 0.05) {
        if (!valid_ || !localization_detail::finitePose(center) ||
            !localization_detail::contains(field_, center.x, center.y) ||
            !nonnegative(xySpread) || !nonnegative(headingSpread)) return false;
        for (auto& particle : scratch_) {
            particle.pose = {std::clamp(center.x + normal() * xySpread, field_.minX, field_.maxX),
                             std::clamp(center.y + normal() * xySpread, field_.minY, field_.maxY),
                             wrapAngle(center.theta + normal() * headingSpread)};
            particle.weight = 1.0 / N;
            if (!localization_detail::finitePose(particle.pose)) return false;
        }
        particles_ = scratch_;
        estimateAvailable_ = true;
        accepted_ = false;
        effectiveCount_ = N;
        return true;
    }

    // Apply odometry's body-frame translation (forward/left) and CCW angle delta.
    // Translation is expressed at the midpoint heading of this update.
    bool predict(double forward, double left, double dtheta,
                 double linearNoise = 0, double angularNoise = 0) {
        if (!hasEstimate() || !std::isfinite(forward) || !std::isfinite(left) ||
            !std::isfinite(dtheta) || !nonnegative(linearNoise) ||
            !nonnegative(angularNoise)) return false;
        scratch_ = particles_;
        double totalWeight = 0;
        for (auto& particle : scratch_) {
            const double rotation = dtheta + normal() * angularNoise;
            const double heading = particle.pose.theta + rotation / 2;
            const double f = forward + normal() * linearNoise;
            const double l = left + normal() * linearNoise;
            particle.pose.x += std::cos(heading) * f - std::sin(heading) * l;
            particle.pose.y += std::sin(heading) * f + std::cos(heading) * l;
            particle.pose.theta = wrapAngle(particle.pose.theta + rotation);
            if (!localization_detail::finitePose(particle.pose)) return false;
            if (!localization_detail::contains(field_, particle.pose.x, particle.pose.y))
                particle.weight = 0;
            totalWeight += particle.weight;
        }
        if (!std::isfinite(totalWeight) || totalWeight <= 0) {
            estimateAvailable_ = false;
            accepted_ = false;
            return false;
        }
        for (auto& particle : scratch_) particle.weight /= totalWeight;
        particles_ = scratch_;
        accepted_ = false;
        return true;
    }

    bool observe(const RangeSensor& sensor, double distance) {
        return observe(std::array<RangeObservation, 1>{{{sensor, distance}}});
    }

    // Batch sensors from one instant so resampling cannot bias later sensors.
    // Reject the whole batch if any supplied reading is invalid/out of range.
    // Missing/occluded readings should be omitted, never replaced with zero.
    template <std::size_t M>
    bool observe(const std::array<RangeObservation, M>& observations) {
        accepted_ = false;
        if (!hasEstimate() || M == 0) return false;
        for (const auto& observation : observations) {
            if (!localization_detail::validSensor(observation.sensor) ||
                !std::isfinite(observation.distance) ||
                observation.distance < observation.sensor.minRange ||
                observation.distance > observation.sensor.maxRange) return false;
        }
        std::array<double, N> logs{};
        double maximum = -std::numeric_limits<double>::infinity();
        for (std::size_t i = 0; i < N; ++i) {
            const auto& particle = particles_[i];
            double logWeight = particle.weight > 0 ? std::log(particle.weight)
                : -std::numeric_limits<double>::infinity();
            if (!localization_detail::contains(field_, particle.pose.x, particle.pose.y))
                logWeight = -std::numeric_limits<double>::infinity();
            for (const auto& observation : observations) {
                const double expected = raycastWall(particle.pose, observation.sensor, field_);
                if (!std::isfinite(expected)) {
                    logWeight = -std::numeric_limits<double>::infinity();
                    break;
                }
                const double residual = (observation.distance - expected) / observation.sensor.sigma;
                // Small uniform outlier term prevents one unexpected object
                // from destroying all weights. It cannot detect occlusion.
                const double probability = 0.98 * std::exp(-0.5 * residual * residual) + 0.02;
                logWeight += std::log(probability);
            }
            logs[i] = logWeight;
            maximum = std::max(maximum, logWeight);
        }
        // No hypothesis predicts a wall. Signal lost localization rather than
        // returning a plausible-looking pose from an impossible prior.
        if (!std::isfinite(maximum)) {
            estimateAvailable_ = false;
            return false;
        }
        double sum = 0;
        for (std::size_t i = 0; i < N; ++i) {
            scratch_[i] = particles_[i];
            scratch_[i].weight = std::exp(logs[i] - maximum);
            sum += scratch_[i].weight;
        }
        if (!std::isfinite(sum) || sum <= 0) {
            estimateAvailable_ = false;
            return false;
        }
        double squaredSum = 0;
        for (auto& particle : scratch_) {
            particle.weight /= sum;
            squaredSum += particle.weight * particle.weight;
        }
        effectiveCount_ = 1 / squaredSum;
        particles_ = scratch_;
        // Retain informative weights until degeneracy warrants resampling.
        if (effectiveCount_ < N * 0.5) systematicResample();
        accepted_ = true;
        return true;
    }

    Pose estimate() const {
        if (!hasEstimate()) {
            const double nan = std::numeric_limits<double>::quiet_NaN();
            return {nan, nan, nan};
        }
        Pose pose{};
        double sine = 0;
        double cosine = 0;
        for (const auto& particle : particles_) {
            pose.x += particle.pose.x * particle.weight;
            pose.y += particle.pose.y * particle.weight;
            sine += std::sin(particle.pose.theta) * particle.weight;
            cosine += std::cos(particle.pose.theta) * particle.weight;
        }
        pose.theta = std::atan2(sine, cosine);
        return pose;
    }

private:
    static bool nonnegative(double value) { return std::isfinite(value) && value >= 0; }
    double uniform() { return (static_cast<double>(random_()) + 0.5) / 4294967296.0; }
    double normal() { return std::sqrt(-2 * std::log(uniform())) * std::cos(2 * pi * uniform()); }
    void systematicResample() {
        scratch_ = particles_;
        std::size_t source = 0;
        double cumulative = scratch_[0].weight;
        const double offset = uniform() / N;
        for (std::size_t i = 0; i < N; ++i) {
            const double target = offset + static_cast<double>(i) / N;
            while (target > cumulative && source + 1 < N)
                cumulative += scratch_[++source].weight;
            particles_[i] = {scratch_[source].pose, 1.0 / N};
        }
    }

    FieldBounds field_;
    std::mt19937 random_;
    std::array<Particle, N> particles_{};
    std::array<Particle, N> scratch_{};
    bool valid_ = false;
    bool estimateAvailable_ = false;
    bool accepted_ = false;
    double effectiveCount_ = N;
};

} // namespace ovelo
