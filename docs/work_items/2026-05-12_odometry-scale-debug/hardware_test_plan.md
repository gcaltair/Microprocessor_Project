# Hardware Test Plan

## Test Metadata

- Date: 2026-05-12
- Author: Codex
- Feature: odometry scale correction and reset/calibration workflow
- Firmware build: `cmake --build cmake-build-debug` successful
- Hardware version: current STM32F446 robot car

## Objective

Confirm that straight-line and reverse odometry distance error is materially reduced, that reset/calibration workflow supports repeatable measurement, and that `P1.0,0.0` no longer overshoots because of relative-move progress accounting.

## Preconditions

- Robot power state: normal boot, wheels free to move on test floor
- Required sensors/modules: encoders, IMU, Bluetooth command link
- Map or environment setup: straight 1.0 m test lane with tape marks at 0 m and 1.0 m
- Safety conditions: keep stop command available; test at conservative speed

## Command Sequence

1. Power on robot and connect Bluetooth serial
2. Send `R0`
3. Send `K` and record current calibration factors
4. Send `V0.20`
5. Send `P1.0,0.0`
6. Wait for motion complete, then send `O`
7. Measure physical stop position against 1.0 m mark
8. Note whether the robot made heading trims during the run and whether physical stop position still overshot despite `ODOM` being near 1.0 m
9. Send `R0`
10. Send `P-1.0,0.0`
11. Wait for motion complete, then send `O`
12. Measure physical stop position against -1.0 m mark

If forward or reverse scale is still off, adjust with:

13. Send `Klf,lr,rf,rr` using revised factors
14. Repeat steps 2-12

Optional calibration helper:

13.5 After measuring actual travel, send `D0.95` or `D-0.92`
13.6 Record the suggested `K...` line printed by firmware
13.7 Apply that suggestion with `Klf,lr,rf,rr`

## Expected Behavior

- Serial output:
  - reset acknowledgement and a clean localization/navigation reset
  - calibration printout
  - `O` shows odometry close to commanded distance
  - `O` now includes the just-finished run's `MOVE cmd/target/progress/remain` for relative-move completion diagnosis
  - `O` now includes `ENC dl/dr/cntL/cntR/cal=...` for left-right diagnosis
  - `D...` prints a suggested `K...` calibration command
- Physical robot behavior:
  - straight movement without large heading drift
  - forward and reverse stop positions closer to commanded distance than before
  - when heading trims happen mid-run, `P1.0,0.0` should not show the old "ODOM near target but body still physically overshoots" pattern
- Timing or tolerance expectations:
  - target odometry error after calibration: preferably within 5%

## Failure Signs

- Unexpected serial output
- odometry resets but physical pose does not start from stable zero reference
- `R0` is accepted but `O` still shows stale `EST / CTRL / MOVE` state from a previous run
- forward or reverse distance still biased by about 10-20%
- `ENC dl` and `ENC dr` differ significantly during straight motion
- obvious heading drift dominating the distance error
- `P1.0,0.0` still overshoots materially whenever the robot drives a shallow arc
- `MOVE progress` remains clearly below physical travel even when `ENC dl/dr` and stop position imply the car already went far enough

## Result Interpretation

Use the forward and reverse runs to separate root causes:

- If physical stop distance is wrong, and `ENC dl/dr` are both wrong by a similar ratio:
  - likely dominant issue: wheel-scale calibration
  - next action: use `D...`, then apply suggested `K...`

- If physical stop distance is wrong, and `ENC dl` and `ENC dr` differ a lot during straight motion:
  - likely dominant issue: left/right asymmetry, traction, or wheel-specific calibration
  - next action: tune `Klf/lr/rf/rr` separately and inspect floor / wheel condition

- If physical stop distance is near target, but `MOVE progress` is still much smaller than actual travel:
  - likely dominant issue: relative-move progress accounting still mismatches real motion
  - next action: inspect `Core/Src/pid.c` distance-progress logic

- If `MOVE progress` is near target, but physical stop still overshoots while `ENC dl/dr` also overshoot:
  - likely dominant issue: odometry scale is still biased
  - next action: prioritize encoder calibration workflow

- If forward is acceptable but reverse is still poor:
  - likely dominant issue: reverse-direction wheel scale or traction asymmetry
  - next action: focus on reverse coefficients `lr` / `rr`

- If `R0` is issued and the next `O` still shows stale `MOVE` or nonzero pose state before motion:
  - likely dominant issue: reset path not fully clean
  - next action: inspect `R0`, `LocalizationTask_Reset()`, and control-state reset sequence

## Safe Stop / Recovery

- Stop command: `S`
- Reset command: `R0`
- Required operator action: lift wheels or power-cycle if motion becomes unsafe

## Result

- Status: not run
- Tester:
- Notes:

## Quick Reply Template

Copy this back after testing:

```text
[Step 1]
R0 / O:
ODOM zero?:
MOVE zero?:
EST/CTRL zero?:

[Step 2]
L behavior:
O after L:
R behavior:
O after R:
Relative turn ok?:

[Step 3]
K output:
O after P1.0,0.0:
Physical forward distance:
Forward drift:
MOVE forward:
ENC forward:

[Step 4]
O after P-1.0,0.0:
Physical reverse distance:
Reverse drift:
MOVE reverse:
ENC reverse:

[Step 5]
D output, if any:
Applied K, if any:
```
