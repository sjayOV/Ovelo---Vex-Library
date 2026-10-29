# Sensor and geometry checks

Complete this checklist before tuning or enabling autonomous output. Keep the robot's motors disabled while pushing or rotating it by hand. Support the robot safely whenever checking motor direction with wheels lifted.

## Know the coordinate convention

Looking down on the robot:

- Robot `+x` points forward; robot `+y` points left.
- Counterclockwise rotation is positive.
- At field heading zero, forward points along field `+X`.
- Distances are inches and angles in the core are radians.

PROS IMU rotation is clockwise degrees. The adapter negates it and converts degrees to radians. Do not negate it again in the core.

## Measure the geometry

Choose one fixed tracking origin on the chassis. Measure each wheel center relative to it; offsets are perpendicular to each wheel's measurement direction.

| Setting | Measurement | Typical sign |
| --- | --- | --- |
| `leftOffset` | Left forward-tracking wheel's sideways `y` position | Positive if left of origin |
| `rightOffset` | Right forward-tracking wheel's sideways `y` position | Negative if right of origin |
| `sideOffset` | Sideways-tracking wheel's forward `x` position | Negative if behind origin |

Use the tracking wheels' effective diameters, not the drive wheels' diameters. If a rotation sensor shaft turns multiple times per wheel revolution, account for that ratio when converting shaft angle to travel. A 36000-centidegree shaft revolution corresponds to `π × diameter` inches only for a 1:1 ratio.

The default offsets `+5`, `-5`, and `-3` inches are demonstration values. Enter your measurements. In wheel-only heading mode, left and right offsets must be distinct.

## A. Connectivity and calibration

1. Confirm each Smart Port number, device type, and cable.
2. Power the Brain and leave the robot still until IMU calibration completes.
3. Check for sensor read errors and a healthy odometry baseline.
4. Do not reset a tracking sensor's hardware position while odometry is running. If you must, stop and reinitialize its baseline.

## B. Forward push

Push the robot forward approximately 12 inches without turning.

- Both forward tracking wheel readings must increase.
- The sideways reading should stay close to its starting value.
- Heading should remain close to its starting value.
- At heading zero, pose `x` should increase and pose `y` should remain nearly constant.

A wrong wheel sign is a configuration error. Fix the sensor reversal in the hardware adapter; do not hide it with a negative wheel diameter.

## C. Leftward slide

Slide the robot left without rotating it.

- The sideways reading must increase.
- Forward tracking readings should stay close to their starting values.
- At heading zero, pose `y` should increase.

Drive-wheel odometry cannot directly measure sideways slide. This template expects separate tracking measurements for the documented layouts.

## D. Counterclockwise turn

Rotate the robot counterclockwise about the chosen origin, about 90 degrees.

- Core heading must increase by approximately `π/2` radians.
- The left forward wheel usually reports negative travel and the right forward wheel positive travel.
- A sideways wheel behind the origin reports negative travel.
- After rotation compensation, pose should stay close to the same position.

For a pure counterclockwise turn `dθ`, expected raw tracking distances are `dL = -leftOffset × dθ`, `dR = -rightOffset × dθ`, and `dS = sideOffset × dθ`. Small translation during hand rotation is normal; a large repeatable translation points to signs or offsets.

## E. Motor direction

With the drive wheels safely lifted, command low positive voltage on each side separately. Both sides must move the robot forward. Correct motor reversal in the adapter. Verify that a positive left-turn command turns counterclockwise: right-side forward output must exceed left-side output.

## Record the result

For a known hand motion, the portable helper can provide an initial direction report:

```cpp
const auto report = ovelo::diagnose(
    ovelo::HandTest::Forward, beforeSample, afterSample, config);
// Inspect report.status and print report.message through your adapter.
```

`beforeSample` and `afterSample` use the same cumulative-inch and CCW-radian conversions as odometry. Available tests are `Forward`, `Left`, and `Counterclockwise`. Reports distinguish `Pass`, `WrongDirection`, `NoMotion`, and `Invalid`. A direction pass is not a wheel-scale or position-accuracy test.

Record the date, robot, measured geometry, firmware/kernel version, and each check's outcome. The diagnostic helpers can flag inconsistent measurements, but they cannot discover every wiring error or distinguish all real movement from sensor problems. Recheck after changing wheels, gearing, sensor orientation, or chassis geometry.
