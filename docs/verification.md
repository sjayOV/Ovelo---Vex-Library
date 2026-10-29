# Verification and robot acceptance

## Current evidence and limits

The project contains portable behavior tests, a desktop simulation, and a CI path for a real PROS V5 project build. The current development environment has a failed network proxy and no installed PROS SDK or ARM compiler. An actual PROS build, V5 firmware upload, and robot operation could not be verified here.

The portable checks exercise mathematics and simulated behavior. Their results must be taken from the command output for the current revision. The presence of a test, workflow, or command in this repository is not evidence that it ran successfully.

Results from this development run:

- `make test`: passed 62 portable core checks, the localization behavior executable, and four Python helper tests.
- `make example`: the idealized closed-loop simulation reached the `(24, 24)` target and settled after 2000 ms, with approximately 0.744 inches of simulated final position error. This is a simulation result, not a measured robot accuracy claim.
- `ASAN_OPTIONS=detect_leaks=0 make sanitize`: address and undefined-behavior checks passed. LeakSanitizer could not run under this container's process-inspection restrictions; leak checks remain unrun.
- `make pros-smoke`: blocked because PROS CLI is missing. The failed network proxy prevented obtaining the SDK/toolchain. Compilation and linking against PROS were not reached.
- GitHub Actions SDK job: configured, unrun here.
- Brain upload and physical robot checks: unrun; no V5 device was attached.

| Check | What it establishes | What it does not establish |
| --- | --- | --- |
| `make test` | Executed portable behavior assertions | SDK API or hardware compatibility |
| `make example` | The desktop simulation ran with its chosen model | Real friction, inertia, motor response, or sensor noise |
| `make sanitize` | Executed portable checks under supported runtime sanitizers | All possible inputs or real-time correctness |
| `make pros-smoke` / real SDK CI job | When successful, compilation and linking against the selected PROS kernel | Brain upload, wiring, sensor directions, or robot performance |
| Local `pros upload` | When successful, transfer to the selected Brain program slot | Correct physical behavior |
| Physical acceptance below | Observed behavior on the recorded robot/configuration | All robots, surfaces, loads, or conditions |

Do not replace the real SDK check with local PROS-shaped stubs and call that V5 compatibility. A passing desktop compiler cannot validate the SDK's declarations, kernel libraries, linker script, or ARM ABI.

## Required before a V5 compatibility claim

Record the commit or archive version, kernel version, ARM compiler version, Brain firmware version, robot configuration, and date. Mark each item as passed, failed, skipped, or unrun.

- [ ] `make test` completed and actually executed assertions.
- [ ] `make example` completed and produced a plausible trace.
- [ ] A fresh project was created through `tools/create_pros_project.py`.
- [ ] `pros make` compiled and linked the generated project against kernel 4.1.1.
- [ ] The built program uploaded to the intended V5 Brain and slot.
- [ ] Stationary IMU calibration completed within the configured timeout.
- [ ] Disconnected/wrong-port sensors produce a fault and inhibit drive output.
- [ ] Positive motor commands move both drive sides forward when safely lifted.
- [ ] Forward, left-slide, and counterclockwise hand tests passed.
- [ ] A known straight push produced the expected measured distance.
- [ ] An in-place turn produced little translation after offset compensation.
- [ ] A small turn completed and reported `Settled` at conservative output.
- [ ] A short point move completed and reported `Settled`.
- [ ] A diagonal target and a target behind the initial heading behaved as documented.
- [ ] Reaching timeout, stalling, cancellation, and disabling all stopped motor output.
- [ ] A sensor fault during movement stopped output and required deliberate recovery.
- [ ] Driver-control/autonomous/disabled transitions did not reuse stale measurements or resume an old command.

## Optional experiments

- [ ] Particle localization tested in a known empty rectangular area, with measured distance sensor geometry.
- [ ] Repeated trials recorded error under different loads and surfaces.
- [ ] A student new to the project completed the setup guide without undocumented steps.

Particle localization is disabled unless explicitly integrated. Its simulated tests do not imply that a V5 distance sensor sees the intended field wall. Performance or usability claims require recorded measurements and a described procedure.

## Release notes

When publishing a release, summarize actual results rather than changing this checklist to imply universal support. If the SDK build passes but a physical check fails, report both. Keep a specific failure visible until it has been diagnosed and retested.
