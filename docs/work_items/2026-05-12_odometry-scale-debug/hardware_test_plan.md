# Hardware Test Plan

## Test Metadata

- Date: 2026-05-12
- Author: Codex
- Feature: odometry scale correction and reset/calibration workflow
- Firmware build: `cmake --build cmake-build-debug` successful
- Hardware version: current STM32F446 robot car

## Objective

Confirm that straight-line and reverse odometry distance error is materially reduced and that reset/calibration workflow supports repeatable measurement.

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
8. Send `R0`
9. Send `P-1.0,0.0`
10. Wait for motion complete, then send `O`
11. Measure physical stop position against -1.0 m mark

If forward or reverse scale is still off, adjust with:

12. Send `Klf,lr,rf,rr` using revised factors
13. Repeat steps 2-11

## Expected Behavior

- Serial output:
  - reset acknowledgement
  - calibration printout
  - `O` shows odometry close to commanded distance
  - `O` now includes `ENC dl/dr/cntL/cntR/cal=...` for left-right diagnosis
- Physical robot behavior:
  - straight movement without large heading drift
  - forward and reverse stop positions closer to commanded distance than before
- Timing or tolerance expectations:
  - target odometry error after calibration: preferably within 5%

## Failure Signs

- Unexpected serial output
- odometry resets but physical pose does not start from stable zero reference
- forward or reverse distance still biased by about 10-20%
- `ENC dl` and `ENC dr` differ significantly during straight motion
- obvious heading drift dominating the distance error

## Safe Stop / Recovery

- Stop command: `S`
- Reset command: `R0`
- Required operator action: lift wheels or power-cycle if motion becomes unsafe

## Result

- Status: not run
- Tester:
- Notes:
