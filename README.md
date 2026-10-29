# Ovelo

**Autonomy made understandable.** A header-only C++17 robotics learning template with a portable mathematical core and a VEX V5 adapter for PROS.

Ovelo exposes the measurements and decisions behind autonomous movement: sensor checks, rotation compensation, pose traces, bounded motion commands, and an optional rectangular-field particle filter. The implementation is independent; LemLib, EZ-Template, and PROS are acknowledged reference projects.

**Validation boundary:** desktop tests can verify the equations and simulated behavior. They cannot verify a V5 Brain, sensor wiring, motor direction, or traction. The adapter requires an actual PROS SDK build and a robot acceptance test before use. See [verification](docs/verification.md) for the precise status.

## Start here

1. Follow the [quickstart](docs/01-quickstart.md) for desktop installation or a PROS project.
2. Enter your ports, wheel diameters, and signed offsets in `examples/pros/main.cpp`.
3. Complete the [sensor checks](docs/02-sensor-checks.md) with motor output disabled.
4. Read [motion and tuning](docs/04-motion.md), then enable the example's `hardwareConfirmed` setting only when its checklist passes.
5. Follow [GitHub publishing](docs/07-github.md) to put your reviewed version in your repository.

```sh
make test       # portable behavior tests
make example    # desktop telemetry simulation
make sanitize  # portable tests with memory/undefined-behavior checks
```

Current run: portable tests and the desktop simulation passed. AddressSanitizer/UndefinedBehaviorSanitizer checks passed with leak detection disabled because this container cannot run LeakSanitizer. The real SDK build and robot checks remain unrun; see [verification](docs/verification.md).

For V5, install [PROS](https://pros.cs.purdue.edu/v5/getting-started/installation.html) and its ARM toolchain, then generate a separate SDK project:

```sh
python3 tools/create_pros_project.py /tmp/ovelo-v5
cd /tmp/ovelo-v5
pros make
```

The repository's root Makefile builds desktop programs. Upload the generated **PROS project**, not the desktop test binary. `make pros-smoke` requires the actual PROS SDK/toolchain; it is not a substitute API stub check.

## What is included

| Component | Purpose |
| --- | --- |
| `include/ovelo/` | Portable odometry, motion control, diagnostics, and optional localization |
| `examples/pros/` | PROS lifecycle, V5 sensors and voltage output, disabled-state handling |
| `examples/desktop/` | Repeatable simulation without a robot |
| `tests/` | Behavior tests for the portable implementation |
| `tools/` | Configuration wizard and real PROS project generator |
| `.github/workflows/ci.yml` | Desktop checks and an SDK build job |

The data path is `sensor readings → wheel travel → pose → motion command → motor voltage`. The hardware adapter owns unit conversion and sensor error detection; the core owns the mathematics. Commands report completion, timeout, stall, cancellation, or invalid input rather than waiting forever.

## Documentation

- [Quickstart and installation](docs/01-quickstart.md)
- [Sensor direction and geometry checklist](docs/02-sensor-checks.md)
- [Coordinates, units, and odometry](docs/03-odometry.md)
- [Motion API, tuning, and failure handling](docs/04-motion.md)
- [Optional particle localization](docs/05-localization.md)
- [Design decisions and reference projects](docs/06-design-and-comparison.md)
- [Upload your project to GitHub](docs/07-github.md)
- [Troubleshooting](docs/08-troubleshooting.md)
- [Verification and robot acceptance](docs/verification.md)

## Attribution and use

This version was developed with AI assistance from the supplied prototype documentation. The uploaded files did not include the former implementation, so this is a fresh implementation rather than a patch to unseen source. Review and understand it before use, and consult the current rules and student-centered requirements for your event before using generated code in competition.

Useful related work: [LemLib](https://github.com/LemLib/LemLib), [LemLib documentation](https://lemlib.readthedocs.io/), [EZ-Template](https://github.com/EZ-Robotics/EZ-Template), and [PROS](https://github.com/purduesigbots/pros). Network access to those sources was unavailable during this implementation; compatibility must be established by the real SDK build and hardware checklist, not by an assumed equivalence to another template.

MIT license; see [LICENSE](LICENSE). No affiliation with VEX, PROS, LemLib, EZ-Robotics, or RECF. No claim of globally unique algorithms, superior performance, or hardware validation is made.
