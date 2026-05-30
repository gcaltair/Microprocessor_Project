# Performance Validation Log

Use this file together with `benchmark_log_template.csv`.

## Firmware Under Test

- Branch:
- Commit:
- Build configuration:
- Robot hardware revision:
- Battery chemistry/cell count:
- Battery voltage before runs:
- Battery voltage after runs:

## Timing Procedure

1. Place the robot at the entrance start mark.
2. Start one continuous video with a visible timer.
3. Start exploration using B1 or UART command `START`.
4. Record `Exit Time` when the robot reaches the exit criterion.
5. Command return using B1 long press or UART command `HOME`.
6. Record `Return Time` when the robot reaches the entrance/start criterion.
7. Repeat for at least three complete runs.

## Score Calculation

Copy the official course formula here before final submission:

```text
Performance Score =
```

For each run, record:

- `Exit Time`
- `Return Time`
- `Total Time = Exit Time + Return Time`
- Whether the run completed without reset, emergency stop, or manual correction
- Observed map quality and navigation behavior

## Safety Evidence

Record evidence for each item:

- PB2 emergency stop immediately cuts TIM3 PWM.
- Low battery warning appears on OLED/telemetry.
- Critical battery voltage triggers stop.
- Watchdog is enabled and refreshed by `safetyTask`.
- LiDAR timeout is detected and recovery is attempted.
- Motor stall detection stops motion and enters fault mode.

## Repeatability Summary

| Run | Exit Time (s) | Return Time (s) | Score | Result | Notes |
| --- | ---: | ---: | ---: | --- | --- |
| 1 |  |  |  |  |  |
| 2 |  |  |  |  |  |
| 3 |  |  |  |  |  |

## Submission Consistency

- ZIP file commit/hash:
- Product video commit/hash:
- Benchmark video commit/hash:
- Confirm the flashed binary was built from the submitted code:
