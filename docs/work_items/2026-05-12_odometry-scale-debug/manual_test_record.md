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

## Step-by-Step Return

### Step 1: Baseline after `R0`

- Commands sent:
- Raw `O` output:
- `ODOM` near zero?:
- `MOVE` near zero?:
- `EST / CTRL` near zero?:

### Step 2: Manual turn check

- Commands sent:
- `L` behavior observation:
- Raw `O` after `L`:
- `R` behavior observation:
- Raw `O` after `R`:
- Did `L / R` behave as relative turns?:

### Step 3: Forward 1.0 m run

- Commands sent:
- Raw `K` output:
- Raw `O` output:
- Physical stop distance:
- Heading drift observation:
- `MOVE target/progress/remain`:
- `ENC dl/dr/cntL/cntR`:

### Step 4: Reverse 1.0 m run

- Commands sent:
- Raw `O` output:
- Physical stop distance:
- Heading drift observation:
- `MOVE target/progress/remain`:
- `ENC dl/dr/cntL/cntR`:

### Step 5: Calibration helper, if used

- Commands sent:
- Raw `D...` output:
- Suggested `K...`:
- Was suggested `K...` applied?:

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
- Likely root cause based on interpretation matrix:
- Suggested next command or coefficient change:
- Related files:
  - `Core/Src/encoder.c`
  - `Core/Src/encoder.h`
  - `Core/Src/hc04.c`
  - `Core/Src/pid.c`
