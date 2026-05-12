# Task Brief

## Task Metadata

- Date: 2026-05-12
- Owner: Codex
- Task type: firmware
- Related phase: Phase 3B / Phase 4A prerequisite
- Related issue or request: odometry scale error around 10-20% affecting SLAM and navigation

## Goal

Reduce systematic odometry distance error by fixing basic motion-control / encoder integration issues before higher-level SLAM tuning.

## Scope

- In scope:
  - encoder pulse-to-distance handling
  - odometry integration logic
  - relative-move distance completion logic
  - relative-move progress diagnostics
  - calibration and reset support for bench testing
  - serial-visible diagnostics needed for manual verification
- Out of scope:
  - ICP tuning
  - mapping changes
  - navigation behavior changes
  - full motor-control retuning

## Constraints

- Architecture constraints: keep control, localization, and mapping boundaries intact
- Memory or timing constraints: no new dynamic allocation, no heavy logging in control loop
- Safe edit boundaries: `Core/Src/encoder.c`, `Core/Src/encoder.h`, `Core/Src/hc04.c`, `Core/Src/pid.c`

## Expected Files

- Files to update:
  - `Core/Src/encoder.c`
  - `Core/Src/encoder.h`
  - `Core/Src/hc04.c`
  - `Core/Src/pid.c`
  - `Core/Src/system.h`
  - `docs/work_items/2026-05-12_odometry-scale-debug/*`
- Files to avoid:
  - generated peripheral setup
  - mapping/localization/navigation modules unless blocked

## Verification Plan

- Desktop verification: firmware build
- Hardware verification: manual straight-line and reverse-line distance test, plus relative-move overshoot check
- Required logs or outputs: `O`, `K`, `MOVE target/progress/remain`, new reset/calibration command outputs

## Completion Criteria

- [x] Implementation complete
- [x] Documents updated
- [x] Desktop verification done or explicitly blocked
- [x] Hardware test plan prepared if needed
- [x] Residual risks recorded
