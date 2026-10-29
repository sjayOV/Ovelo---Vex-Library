# Odometry and units

Odometry estimates where the robot moved since its previous sensor reading. It does not know where the robot started until you provide an initial pose, and it does not prevent drift from wheel slip or measurement error.

## Units and data

| Value | Unit/convention |
| --- | --- |
| `Pose.x`, `Pose.y`, `Point.x`, `Point.y` | Field inches |
| `Pose.theta` | Radians, positive counterclockwise, zero along field `+X` |
| `SensorSample.left`, `.right`, `.side` | Cumulative wheel travel in inches |
| `SensorSample.heading` | IMU heading converted to counterclockwise radians |
| `SensorSample.timeMs` | Monotonic unsigned 32-bit milliseconds |
| Wheel offsets | Signed inches from the same tracking origin |
| `DriveCommand.left`, `.right` | Normalized motor commands, bounded to `[-1, 1]` |

Do not pass motor encoder counts, degrees, millimeters, or deltas into a field that expects cumulative inches. The hardware adapter performs conversion before calling `update`.

## Supported layouts

- **Three tracking wheels with an IMU:** left and right forward tracking plus one sideways tracking wheel; `useRight = true`, `useImu = true`.
- **Two tracking wheels with an IMU:** one forward tracking wheel and one sideways tracking wheel; `useRight = false`, `useImu = true`. The unused right reading is ignored.
- **Three tracking wheels without an IMU:** `useRight = true`, `useImu = false`; heading comes from the forward tracking wheels and their separation.

Two tracking wheels without an IMU cannot observe all three planar degrees of freedom. That configuration is invalid.

The PROS example defaults to three wheels plus an IMU. Change `useRight` and `useImu` in its `geometry()` function to select these layouts. It skips the unused right sensor or IMU reads and calibration accordingly. The printed IMU field is zero in wheel-only mode; heading then comes from `pose.theta`.

```cpp
ovelo::OdometryConfig config;
config.leftOffset = 5.0;    // wheel left of the tracking origin
config.rightOffset = -5.0; // wheel right of the tracking origin
config.sideOffset = -3.0;  // wheel behind the tracking origin
config.useRight = true;
config.useImu = true;
ovelo::Odometry odom(config);
```

## Initialization and periodic updates

The first valid sample establishes a baseline. It must not create movement from the sensors' existing absolute positions. Call `update` on a consistent schedule, such as every 10 ms, and check its result before using the pose for motor control.

```cpp
if (!odom.update(sample)) {
    motion.cancel();
    // Send zero voltage to both sides; report the sensor/configuration fault.
    return;
}
const auto pose = odom.pose();
```

Call `reset({x, y, theta})` to assign a known field pose. Resetting the pose preserves the sensor baseline; it does not reset the physical sensors. Stop the robot while assigning its initial pose. A tracker that has lost valid samples needs a clean baseline before control resumes; follow the implementation's healthy-state result rather than treating a stale pose as a valid estimate.

If updates were deliberately paused while disabled, `forgetBaseline()` explicitly discards the old sensor baseline without changing the field pose. The next valid `update` establishes a new baseline. The supplied adapter does this at control-mode entry and refuses to use it to clear a latched operating fault. Displacement during the paused interval is deliberately not reconstructed; set a known pose when absolute field position matters.

## The rotation compensation

Let `dθ` be the counterclockwise heading change. A forward wheel at sideways position `y` moves `-y × dθ` from rotation alone. A sideways wheel at forward position `x` moves `x × dθ` from rotation alone. Subtract those rotational contributions before interpreting translation:

```text
forward travel at the origin = forward wheel travel + y × dθ
sideways travel at the origin = sideways wheel travel - x × dθ
```

The implementation integrates the local arc and rotates it into field coordinates. For a tiny angle it uses the straight-line limit to avoid division by zero. This matters during curved motion; treating all wheel travel as straight forward movement creates a repeatable position error.

Inspect `odom.trace()` when a result is surprising. Compare raw wheel deltas, heading change, compensation, and field movement. A trace explains the calculation; it does not prove that the input sensors measured reality correctly.

The trace exposes `status`, `rawLeft`, `rawRight`, `rawSide`, `dtheta`, compensated `forward`/`lateral`, and field `dx`/`dy`. Status values are `Waiting`, `Baseline`, `Updated`, `InvalidConfig`, `InvalidSample`, `Stale`, and `Jump`. In `Baseline`, no travel was integrated. An error makes `healthy()` false and invalidates the old baseline; the next valid sample establishes a new one while preserving the last pose. Movement during the missing interval is not recoverable from that operation.

`OdometryConfig.maxGapMs` defaults to 250 ms; sample gaps must be positive and no larger than this. Plausibility limits default to 150 inches/second for wheel travel and 20 radians/second for heading. Choose limits from your actual robot's physical maximum with margin; do not increase them merely to hide a discontinuous reading.

## Practical limitations

Slipping or lifted tracking wheels, wheel compression, loose sensor couplings, IMU movement during calibration, and skipped readings can all corrupt the estimate. Keep the update loop responsive. If sensor readings are invalid, stop the motion command and diagnose the input rather than continuing on the previous pose. Particle localization is optional and has its own environmental assumptions; see [localization](05-localization.md).
