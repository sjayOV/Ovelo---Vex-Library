# Troubleshooting

Start with the first error, keep the full command and its working directory, and change one diagnosed cause at a time. Do not raise gains to compensate for sensor errors.

| Symptom | Check and action |
| --- | --- |
| `make` or a C++ compiler is missing | Install the host development tools; desktop tests need C++17 and GNU Make |
| `pros` is missing | Install the official PROS development tools; run the command in their configured terminal |
| `arm-none-eabi-g++` is missing | Install or activate PROS's ARM toolchain; a desktop `g++` cannot produce V5 firmware |
| LeakSanitizer reports an unsupported process-inspection/ptrace environment | For that diagnosed container limitation, run `ASAN_OPTIONS=detect_leaks=0 make sanitize`; address/undefined-behavior checks still run, but leak checking remains unrun |
| Project generator refuses the destination | Choose a new directory outside this source checkout; do not overwrite a robot project |
| Kernel or dependency download fails | Check the destination, proxy, and network policy; keep TLS and package checks enabled |
| `Failed to connect to proxy port 8080` in the cloud | The environment proxy cannot be reached. Restart/rebuild the managed environment or have its networking repaired; a GitHub token does not fix a proxy connection failure |
| `pros make` fails | Confirm you are inside the generated SDK project, then inspect the first compiler/linker error; report the actual kernel and compiler versions |
| `pros upload` cannot find a Brain | Connect USB locally, check drivers and cable, and close conflicting serial tools. Cloud machines usually cannot access your USB device |
| Robot never moves | `hardwareConfirmed` defaults false. Complete the sensor checks before enabling it; also inspect initialization and sensor/motor fault output |
| Robot turns when told to move forward | Verify motor reversals with wheels lifted and verify tracking wheel signs separately |
| Counterclockwise motion decreases heading | Convert PROS clockwise degrees to negative radians exactly once |
| Position jumps during a pure turn | Verify signed offsets and sideways wheel orientation, then inspect `odom.trace()` |
| Distance is wrong by a fixed ratio | Check tracking wheel diameter, centidegrees conversion, external gearing, and inch/mm conversion |
| `OdometryStatus::Stale` | Duplicate timestamps or an update gap exceeded `maxGapMs`; do not run from stale pose data |
| `OdometryStatus::Jump` | A travel/angle change exceeded configured physical limits. Check sensor errors, resets, timing, or incorrect conversion before changing those limits |
| `MotionStatus::Invalid` | Check finite target/pose, configuration, timeout, duplicate timestamps, and `maxStepGapMs` |
| `MotionStatus::Stalled` | Check obstructions, traction, motor output, wheel contact, pose feedback, and measured progress |
| `MotionStatus::TimedOut` | Inspect the trace and tuning. Confirm the target is feasible and allow a realistic timeout; do not silently advance to the next move |
| Point reached but orientation differs | A point move has no final-heading requirement; follow it with `turnTo` |
| Particle estimate jumps or follows the wrong wall | Check the rectangular-field assumption, sensor offset, beam direction, measurement units, and ambiguous symmetric locations |

## Recover from a sensor fault deliberately

The core rejects bad samples and takes a new baseline on the next valid sample. It cannot recover travel that occurred during the rejected interval. The adapter latches a sensor fault to prevent automatic resumption; stop the robot, repair the fault, then restart the program to run initialization again. Assign a known pose again before trusting autonomous coordinates. Entering a new control mode discards the disabled interval's old baseline, but does not clear a fault latched during operation.

Print the error status and relevant trace fields. Do not dump credentials, the entire environment, or private configuration when reporting build/network problems.

## Information to include in an issue

- Ovelo revision, PROS kernel version, compiler version, and operating system.
- Exact failing command and working directory.
- First meaningful error plus surrounding output.
- For robot behavior: port map, wheel geometry, sensor sign results, start pose, target, and motion result.
- Which checks passed: desktop tests, real SDK build, and physical checklist must remain separate.
