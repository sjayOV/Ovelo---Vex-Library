# Quickstart

Choose the desktop route to learn and test the math. Choose the V5 route to prepare a robot project. You can do both.

## 1. Get the files

Clone or download this repository. Open a terminal in its root, where `Makefile`, `include`, and `tools` are located. Avoid placing it inside another project's `include` directory.

If the repository is already open in a cloud task, use that checkout. A separate Git worktree is unnecessary.

## 2. Run the desktop checks

Required tools: a C++17 compiler, GNU Make, and Python 3 for the helper scripts. Linux and macOS provide these through their normal development tools. On Windows, use WSL for these shell commands, or use the PROS VS Code terminal for the V5 workflow.

```sh
make test
make example
make sanitize
```

`test` must execute tests, not merely compile. `example` prints a simulated trace. `sanitize` requires a compiler with AddressSanitizer and UndefinedBehaviorSanitizer support. A simulation passing does not establish motor or sensor compatibility.

In a restricted container, LeakSanitizer may fail because its process-inspection capability is unavailable. When that is the diagnosed error, `ASAN_OPTIONS=detect_leaks=0 make sanitize` still runs address and undefined-behavior checks, but **does not check for leaks**. Do not use this setting to dismiss unrelated sanitizer reports.

## 3. Install PROS for V5

1. Follow the [official PROS installation guide](https://pros.cs.purdue.edu/v5/getting-started/installation.html). Install the PROS CLI and its ARM compiler, linker, and runtime libraries. The VS Code PROS extension is the convenient desktop route.
2. In the same terminal you will use for the build, run `pros --version` and `arm-none-eabi-g++ --version`. Both must succeed.
3. Keep a working Internet connection for the first kernel download. Do not disable TLS or package verification to work around a download failure.

The adapter targets PROS kernel **4.1.1**. The generator creates a normal SDK project instead of imitating a PROS project with desktop-only headers.

```sh
python3 tools/create_pros_project.py /tmp/ovelo-v5
cd /tmp/ovelo-v5
pros make
```

On your own machine, replace `/tmp/ovelo-v5` with a new permanent directory, for example `../ovelo-v5`. The destination must not already contain a project. Use a new destination for a fresh generated copy; the helper avoids silently overwriting your robot changes.

**Edit the generated project's `src/main.cpp` for robot work.** The generator copies the repository's example and library. Later edits in this template do not automatically update a generated project, and rerunning the generator must not erase your robot configuration.

If you already have a working PROS V5 C++ project, copy `include/ovelo/` into its `include/` directory. Integrate the lifecycle functions from `examples/pros/main.cpp`; do not define duplicate `initialize`, `autonomous`, or `opcontrol` functions. Preserve your SDK-generated Makefile and project metadata.

## 4. Enter the physical configuration

The supplied ports and dimensions are examples. Match all of these to your robot:

- Left/right motor ports, reversals, cartridge gearing, and drive layout.
- Tracking rotation sensor ports, direction, wheel diameters, and external gearing.
- Left/right tracking wheels' signed sideways offsets and the sideways wheel's signed forward offset.
- IMU port and conversion from clockwise degrees into counterclockwise radians.

Run `python3 tools/setup_wizard.py --interactive` if you want guided geometry entry. Without `--interactive`, it prints a starter configuration using defaults or the explicit command-line options shown by `--help`. The result is a configuration aid, not a sensor calibration or wiring detector.

Leave `hardwareConfirmed` false while checking wiring and telemetry. It prevents the example from applying drive voltage; it cannot make an incorrectly wired robot correct. Read the [sensor checklist](02-sensor-checks.md) before changing it.

## 5. Build and upload

From the generated PROS project directory:

```sh
pros make
pros upload --slot 1
```

Connect the V5 Brain by USB and select a slot intentionally. Uploading replaces the program in that slot. If the CLI's options differ, run `pros upload --help` for your installed CLI. A cloud container normally has no USB access; upload from your local computer.

## 6. First motion

1. Secure the robot with drive wheels clear of the floor and an accessible stop control.
2. Test driver directions at low output. Stop if either side is reversed.
3. Put the robot on a clear test surface and confirm the [unpowered sensor checks](02-sensor-checks.md).
4. Use short autonomous distances and conservative output limits. Confirm timeout, cancellation, and disabled-state stopping.
5. Record results in the [acceptance checklist](verification.md). Tune only after measurement signs and geometry are correct.

An IMU calibration needs the robot to remain still. A calibration error, disconnected sensor, or invalid odometry update must keep the drive stopped; investigate it rather than increasing gains.
