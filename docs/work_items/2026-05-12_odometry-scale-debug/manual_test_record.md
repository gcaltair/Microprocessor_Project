# Manual Test Record

## Test Reference

- Date:
- Tester:
- Related test plan: `hardware_test_plan.md`
- Firmware build:
- Environment:

## Execution

- Was the exact plan followed?
- If not, what changed?

## Observations

- Serial/log observations:
- `MOVE target/progress/remain` during forward run:
- `ENC dl/dr/cntL/cntR` during forward run:
- `ENC dl/dr/cntL/cntR` during reverse run:
- Relative-move overshoot observation during `P1.0,0.0`:
- `D...` suggestion output:
- Robot behavior observations:
- Forward 1.0 m physical result:
- Reverse 1.0 m physical result:

## Result

- Status: blocked
- Primary reason: hardware test not run by agent; manual tester required

## Deviations

- Expected:
- Actual:

## Follow-Up

- Required fix:
- Retest needed: yes
- Related files:
  - `Core/Src/encoder.c`
  - `Core/Src/encoder.h`
  - `Core/Src/hc04.c`
  - `Core/Src/pid.c`
