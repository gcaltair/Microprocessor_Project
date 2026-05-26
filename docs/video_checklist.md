# Video Checklist

## Product Video

- Team introduction is included.
- Full system demo shows start, exploration, map update, target/return-home, and stop.
- OLED clearly shows mode, pose, battery voltage, map progress, and target status.
- Bluetooth/host telemetry is visible or mentioned with live status data.
- Evaluation section explains timing results and safety behavior.
- Code structure section matches the submitted repository.

## Benchmark Video

- One continuous uncut shot for each timed run.
- Timer is visible and readable.
- Start, exit arrival, return command, and entrance arrival are all visible.
- The robot and course remain in frame.
- No manual correction occurs during a scored run.
- Firmware commit/hash is recorded in the benchmark log.

## Safety Demo Shots

- PB2 emergency stop cuts motor PWM immediately.
- Low-battery threshold behavior is demonstrated if a bench supply is available.
- OLED display page switching is shown.
- UART `STAT`, `HOME`, `STOP`, and `CLEAR` commands are shown if Bluetooth is used.
