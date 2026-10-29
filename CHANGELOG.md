# Changelog

## 0.2.0 — development

- Fresh implementation from the supplied Ovelo prototype requirements; the former source was not supplied.
- Header-only C++17 mathematical core with explicit inches/radians conventions.
- Two-wheel-plus-IMU and three-tracking-wheel odometry, rotation compensation, traces, and rejected-input states.
- Hand-motion direction diagnostics and bounded point/turn control with settling, timeout, stall, cancellation, and invalid-input outcomes.
- Optional fixed-size rectangular-field particle localization.
- PROS V5 hardware adapter, real SDK project generator, desktop simulation, portable tests, and CI configuration.
- Step-by-step installation, geometry, tuning, troubleshooting, validation, and GitHub upload guides.

**Validation limitation:** this environment could not access the real PROS SDK or attached V5 hardware. Do not treat the portable tests or CI configuration as an executed V5 compatibility test.

## 0.1.0-prototype — 2026-10-26

The supplied prototype documents described an educational portable core, odometry traces, motion states, optional localization, and a stub-only adapter check. Its source files were not included in the uploads. No prior implementation or hardware results could be inspected.
