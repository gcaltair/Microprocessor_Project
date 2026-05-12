# Verification Report

## Verification Summary

- Date: 2026-05-12
- Feature: odometry scale correction
- Owner: Codex

## Verification Levels Reached

- Level 0 static review: yes
- Level 1 desktop verification: yes
- Level 2 prepared operator verification: yes
- Level 3 hardware verification: no

## Desktop Evidence

- Commands run: `cmake --build cmake-build-debug`
- Result: success
- Latest build note: passed after `Core/Src/pid.c` relative-move progress fix; RAM `98.83%`, FLASH `21.96%`
- Limits of coverage: cannot prove physical wheel-scale accuracy on desktop

## Hardware Evidence

- Test plan: `hardware_test_plan.md`
- Manual test record: `manual_test_record.md`
- Result: not run

## Remaining Gaps

- Need real forward/reverse distance measurements
- Need compare `ENC dl/dr` symmetry during straight motion
- Need confirm `P1.0,0.0` no longer overshoots when heading corrections occur during the drive phase
- Need confirm whether residual error is still scale bias or mainly heading drift / traction

## Release / Merge Recommendation

- Ready with hardware follow-up
