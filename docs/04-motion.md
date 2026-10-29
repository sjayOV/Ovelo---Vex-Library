# Motion and tuning

Ovelo supplies a small proportional point-and-turn controller. It owns no motors or background task. Your program supplies a fresh pose and time, then applies its returned normalized voltage request. It does not include path following, obstacle avoidance, automatic reverse driving, or a promise of a particular positioning accuracy.

## Basic sequence

```cpp
ovelo::MotionController motion;
const std::uint32_t startMs = /* monotonic clock */;
motion.moveToPoint({24.0, 0.0}, odom.pose(), startMs, 4000);

// On each subsequent 10 ms iteration, after a successful odometry update:
const auto output = motion.step(odom.pose(), nowMs);
// Apply output.left * 12000 and output.right * 12000 millivolts through the adapter.
```

The first `step` must occur at a later timestamp than the command's start. Call `step` once per update; duplicate timestamps and gaps longer than `maxStepGapMs` are invalid. Use the same unsigned 32-bit millisecond clock for command creation, sensor samples, and steps. The elapsed-time arithmetic supports clock rollover for intervals shorter than half the counter range.

`moveToPoint` aims toward the point and drives forward. It reduces forward output when the heading error is large, turns in place when facing away, and checks position tolerance before declaring arrival. It does not require a final heading at the point. If a final orientation matters, run a separate turn after a settled move:

```cpp
motion.turnTo(ovelo::radians(90), odom.pose(), nowMs, 2500);
```

`turnTo` chooses the wrapped shortest heading error. Angles are absolute field headings in radians; `radians(90)` means field `+Y`, not “turn 90 degrees relative to wherever you are.” For a relative turn, add the desired angle to the current `odom.pose().theta`.

## Handle every result

| `MotionStatus` | Meaning and required response |
| --- | --- |
| `Idle` | No command started; send zero output |
| `Running` | Continue fresh sensor updates and periodic steps |
| `Settled` | Target stayed within tolerance for `settleMs`; zero output and optionally start the next command |
| `TimedOut` | Overall time limit reached; stop and inspect the attempted move |
| `Stalled` | Insufficient measured progress for `stallMs`; stop and check obstruction, traction, signs, or gains |
| `Invalid` | Invalid configuration/pose/target/timing; stop and correct the cause |
| `Cancelled` | A running command was cancelled; send zero output |

`step` returns zero output after a terminal state. `cancel()` changes a running command's state; it cannot undo voltage already sent to a physical motor. The hardware layer must also send zero voltage when cancelling or disabling.

Never continue an autonomous sequence after a failed move merely because it finished waiting. Advance only on the result your sequence expects. On sensor error, cancel, zero the motors, and require deliberate recovery. Print `statusName(motion.status())` for a readable result.

## Configuration

Construct `MotionController` with an `ovelo::MotionConfig` value:

| Setting | Default | Meaning |
| --- | --- | --- |
| `driveKp` | `0.075` | Normalized forward output per inch of distance error |
| `turnKp` | `1.5` | Normalized turn output per radian of heading error |
| `maxDrive` | `0.65` | Forward component cap |
| `maxTurn` | `0.6` | Turn component cap |
| `positionTolerance` | `0.75` in | Point arrival tolerance |
| `headingTolerance` | `radians(2)` | Turn arrival tolerance |
| `settleMs` | `150` ms | Continuous time inside tolerance before completion |
| `stallMs` | `1000` ms | Time without sufficient error reduction before stall |
| `maxStepGapMs` | `100` ms | Maximum permitted controller update gap |
| `distanceProgress` | `0.1` in | Meaningful distance-error reduction |
| `angleProgress` | `radians(0.5)` | Meaningful heading-error reduction |

Forward and turn components mix into left/right outputs and normalize to `[-1, 1]`. A component cap is not a cap on an individual side after mixing: `maxDrive = 0.3` and `maxTurn = 0.3` can request `0.6` on one side. If you need a strict physical voltage cap during initial testing, additionally scale or clamp each side in the hardware adapter.

Defaults are starting values for a simulator, not tuned gains for your robot. Positive finite gains, positive tolerances, valid time intervals, and component caps in `(0, 1]` are required. Zero or negative timeouts are invalid; choose a timeout longer than the expected movement and its settling window.

## Tune in this order

1. Verify wheel scaling, signed offsets, IMU direction, motor reversals, and disabled-state stopping using the [sensor checks](02-sensor-checks.md).
2. Set conservative physical voltage limits and test a small turn. Increase `turnKp` gradually until it turns promptly without oscillation. Keep the same load and surface between trials.
3. Test a short straight move. Increase `driveKp` gradually until it approaches reliably without repeated overshoot.
4. Check diagonal targets and targets behind the initial heading. Confirm it turns before driving and produces the intended turn direction.
5. Tune tolerances and settling time to measured behavior. Very tight tolerances may be unattainable with a simple proportional controller and real drivetrain friction.
6. Verify that an obstructed or disconnected system stops and reports a failure. Choose progress thresholds and timeouts based on measured operation, not merely to suppress failures.

The stall detector measures progress in the pose estimate. A slipping robot with incorrect odometry may appear to make progress. It is a bounded software check, not collision detection or an independent motor watchdog.
