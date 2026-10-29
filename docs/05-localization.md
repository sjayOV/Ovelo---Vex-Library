# Optional wall-based localization

This chapter is for readers who already have working odometry. The default robot example does **not** enable this filter. Keep it disabled until sensor directions, wheel geometry, timestamps and motion shutdown behavior pass the earlier checks.

The implementation is an educational rectangular-wall particle filter. Host tests verify its mathematics and input handling. No range-sensor or physical V5 validation has been performed. It is not a replacement for properly configured odometry, and one distance sensor cannot uniquely determine a robot's location and heading.

## What it does

Odometry estimates movement from relative sensors. A particle filter keeps a set of possible poses, predicts their movement, then weights them according to how closely their expected wall distances match actual readings. The weighted mean is a candidate pose correction.

Ovelo exposes every particle through `particles()` and reports the effective particle count before the most recent measurement resampling. You can inspect whether the filter is concentrating on a few hypotheses. A high effective count does not prove the pose is accurate: uninformative readings also leave weights nearly uniform.

The filter uses fixed `std::array` storage. The default `ParticleLocalizer<128>` needs approximately 13 KiB for its object, depending on the platform. Declare it globally or `static`, as below: an automatic local instance can exceed a PROS task's stack. Prediction reuses the object's scratch storage; observation still uses a bounded temporary array of `N` doubles (about 1 KiB for 128 particles), plus normal call overhead. Check your task's stack margin and processing time on the Brain before using it. Larger counts consume more CPU and memory. Randomness is seeded; the same seed and command sequence on the same implementation reproduce a simulation.

## Units and field model

All lengths are **inches**. Angles are **radians counterclockwise**. At heading zero, robot forward points along field +X and robot left points along field +Y.

`FieldBounds{minX, minY, maxX, maxY}` describes the interior wall surfaces. Its default is a 144 × 144 inch square starting at (0,0), which is only a convenient simulation example. Measure the actual field and wall surfaces; do not assume that nominal competition dimensions match the geometry seen by a sensor.

`RangeSensor{x, y, angle, minRange, maxRange, sigma}` describes one beam:

| Field | Meaning |
| --- | --- |
| `x` | Beam origin forward of robot center; negative behind center |
| `y` | Beam origin left of robot center; negative to its right |
| `angle` | Beam direction relative to robot forward, counterclockwise radians |
| `minRange`, `maxRange` | Allowed measured distances in inches; configure for your sensor and environment |
| `sigma` | Positive distance uncertainty in inches; larger values make a reading less influential |

PROS distance sensors report millimeters. Convert a valid reading with `inches = millimeters / 25.4`. Check sensor errors and its documented useful range before passing the result to Ovelo. Zero, disconnected, maximum sentinel and no-object readings must never be treated as measured wall distances. The portable filter cannot infer whether a finite, in-range return came from a wall or a game object.

## Minimal example

```cpp
#include "ovelo/localization.hpp"

// Keep this object alive between updates.
static ovelo::ParticleLocalizer<128> localizer({0, 0, 144, 144}, 7);
ovelo::RangeSensor front{5, 0, 0, 2, 78, 2};
ovelo::RangeSensor left{0, 5, ovelo::pi / 2, 2, 78, 2};

// Known starting location, with a 2-inch position and 0.03-radian heading spread.
bool started = localizer.reset({72, 72, 0}, 2, 0.03);

// For each new odometry increment, predict exactly once. Translation here is
// forward/left displacement expressed at the midpoint heading of the update.
// This example is a straight one-inch movement with small process noise.
bool predicted = localizer.predict(1, 0, 0, 0.05, 0.002);

// Example readings are INCHES and must be replaced by validated sensor values.
std::array<ovelo::RangeObservation, 2> readings{{
    {front, 66}, {left, 67}
}};
bool accepted = localizer.observe(readings);
if (started && predicted && accepted && localizer.hasEstimate()) {
    ovelo::Pose candidate = localizer.estimate();
    // Log and compare candidate to odometry. Decide explicitly whether to use it.
}
```

`predict(forward, left, dtheta, linearNoise, angularNoise)` accepts one body-frame translation and angle increment. Noise values are Gaussian standard deviations **per call**, not per second. Replaying the same motion increment twice moves the filter twice. If deriving increments from `OdometryTrace`, its `forward` and `lateral` are arc distances: convert each to the corresponding chord displacement with `sinc(trace.dtheta / 2)` before calling `predict`. Its `dx`/`dy` are already field-frame deltas and must not be passed as body-frame forward/left.

Batch simultaneous sensors in `observe(std::array<RangeObservation, M>)`. The filter evaluates all readings before deciding whether to resample. For a single reading, use `observe(sensor, distance)`. Invalid or missing readings should be omitted; do not insert a placeholder zero. Any invalid member rejects an entire batch, so validate the batch first or pass only the sensors with useful readings.

## Failure and correction rules

1. Check `valid()` once after construction. Invalid, reversed or non-finite field bounds disable the instance.
2. Check `reset(...)`. The starting center must be finite and inside the field. Position and heading spreads must be finite and nonnegative. Particles are clipped to the field at reset. A rejected reset preserves the previous usable state.
3. Check every `predict(...)` and `observe(...)` result. Invalid readings reject a correction and leave the previous estimate available. A rejected reading does not mean a new correction exists.
4. If all hypotheses become physically impossible, `hasEstimate()` becomes false. `estimate()` then returns NaN components, further prediction/correction is rejected, and a deliberate `reset` at a known pose is required. Do not send a missing estimate to the motion controller.
5. Apply corrections only under an explicit policy. Ovelo never changes `Odometry` automatically. A practical initial experiment logs candidate-minus-odometry error while leaving motor control on odometry.

A call returning true means the filter accepted the supplied data under its model. It does not establish that a return was a wall or that the estimate matches the real robot. A small uniform outlier likelihood limits the influence of unexpected ranges, but does not identify occlusion. Grossly inconsistent finite readings may remain uninformative rather than being rejected.

## Model limits

The raycast includes sensor mounting translation and angle, then finds the nearest rectangle boundary along a single ideal beam. It does not model beam width, VEX distance sensor confidence, wall reflectivity, motion blur, tilted mounting, angled walls, game objects, other robots, or wheel slip correlations. These require independent sensing and modeling work.

Repeated observations cannot recover hypotheses that were absent from the initial particle population. Symmetric fields and repeated range patterns can produce multiple plausible poses; a simple mean can lie between them. Use a known starting pose, independent heading information, multiple useful wall views, modest process noise, and logged experiments before considering a correction policy.

The module is intentionally optional. First verify `make test`, then compare its candidate estimates against surveyed positions in a controlled hardware experiment. Record failures as carefully as successful estimates; no simulation result here supports a real-world accuracy claim.
