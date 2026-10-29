#include "ovelo/localization.hpp"

#include <cassert>
#include <cmath>
#include <iostream>
#include <limits>

namespace {
bool near(double a, double b, double tolerance = 1e-8) {
    return std::abs(a - b) <= tolerance;
}
}

int main() {
    using namespace ovelo;
    RangeSensor front{0, 0, 0, 0, 200, 1};
    assert(near(raycastWall({24, 48, 0}, front), 120));
    assert(near(raycastWall({24, 48, pi / 2}, front), 96));
    assert(near(raycastWall({24, 48, pi}, front), 24));
    assert(near(raycastWall({24, 48, -pi / 2}, front), 48));
    RangeSensor offset{5, 2, 0, 0, 200, 1};
    assert(near(raycastWall({24, 48, 0}, offset), 115));
    assert(near(raycastWall({24, 48, pi / 2}, offset), 91));
    assert(near(raycastWall({0, 50, 0}, front), 144));
    assert(near(raycastWall({0, 50, pi}, front), 0));
    assert(std::isnan(raycastWall({-1, 48, 0}, front)));
    assert(std::isnan(raycastWall({24, 48, 0}, RangeSensor{0, 0, 0, 0, 100, 0})));

    ParticleLocalizer<64> deterministic({}, 42);
    assert(deterministic.reset({24, 48, pi / 2}, 0, 0));
    assert(deterministic.predict(10, 2, 0));
    auto pose = deterministic.estimate();
    assert(near(pose.x, 22));
    assert(near(pose.y, 58));
    assert(near(pose.theta, pi / 2));
    assert(deterministic.predict(0, 0, pi / 2));
    assert(near(deterministic.estimate().theta, pi));
    const auto before = deterministic.estimate();
    assert(!deterministic.predict(std::numeric_limits<double>::quiet_NaN(), 0, 0));
    assert(!deterministic.predict(1, 0, 0, -1));
    assert(near(deterministic.estimate().x, before.x));
    assert(!deterministic.observe(front, -1));
    assert(!deterministic.observe(front, std::numeric_limits<double>::infinity()));
    assert(!deterministic.observe(std::array<RangeObservation, 0>{}));
    assert(!deterministic.lastObservationAccepted());

    ParticleLocalizer<512> filter({}, 7);
    assert(filter.reset({68, 64, 0}, 12, 0.01));
    const Pose truth{60, 72, 0};
    RangeSensor left{0, 0, pi / 2, 0, 200, 1.5};
    std::array<RangeObservation, 2> observations{{
        {front, raycastWall(truth, front)}, {left, raycastWall(truth, left)}
    }};
    const Pose initial = filter.estimate();
    assert(filter.observe(observations));
    assert(filter.lastObservationAccepted());
    assert(filter.effectiveParticleCount() >= 1);
    assert(filter.effectiveParticleCount() <= 512 + 1e-7);
    const Pose corrected = filter.estimate();
    const double initialError = std::hypot(initial.x - truth.x, initial.y - truth.y);
    const double correctedError = std::hypot(corrected.x - truth.x, corrected.y - truth.y);
    assert(correctedError < initialError);
    assert(correctedError < 5);
    double total = 0;
    for (const auto& particle : filter.particles()) {
        assert(std::isfinite(particle.weight));
        assert(particle.weight >= 0);
        total += particle.weight;
    }
    assert(near(total, 1));

    // Very unlikely readings stay finite; repeated likelihoods do not underflow.
    for (int i = 0; i < 100; ++i) assert(filter.observe(front, 199));
    assert(std::isfinite(filter.estimate().x));
    assert(std::isfinite(filter.estimate().theta));
    assert(!filter.predict(500, 0, 0));
    assert(!filter.hasEstimate());
    assert(std::isnan(filter.estimate().x));
    assert(!filter.observe(front, 50));
    assert(!filter.lastObservationAccepted());
    assert(filter.reset({60, 72, 0}, 1, 0));
    assert(filter.hasEstimate());

    // Large same-sign bounds must not overflow their midpoint.
    ParticleLocalizer<16> enormous({1e308, 1e308, 1.5e308, 1.5e308});
    assert(enormous.valid());
    assert(enormous.hasEstimate());
    assert(std::isfinite(enormous.estimate().x));
    assert(!filter.reset({60, 72, 0}, -1, 0));
    assert(filter.hasEstimate());
    RangeSensor invalidRange{0, 0, 0, 20, 10, 1};
    assert(!filter.observe(invalidRange, 15));
    RangeSensor outsideOrigin{1000, 1000, 0, 0, 200, 1};
    assert(!filter.observe(outsideOrigin, 10));
    assert(!filter.hasEstimate());

    ParticleLocalizer<64> invalid({10, 0, 5, 144});
    assert(!invalid.valid());
    assert(!invalid.reset({7, 7, 0}));
    assert(std::isnan(invalid.estimate().x));
    std::cout << "localization: raycast, offsets, prediction, rejection, correction, normalization passed\n";
}
