# Performance and Safety Validation

## Baseline Data to Record

Use Bluetooth status telemetry (`P`) during each run and save the serial log.

Required fields already emitted by firmware:

```text
STAT,<MODE>,<SAFE>,<BAT_MV>,<X_MM>,<Y_MM>,<TH_DEG>,<PWM_L>,<PWM_R>,<LIDAR_POINTS>
```

For the final report/video, record these additional values manually:

| Run | Entry->Exit Time | Exit->Entry Time | Battery mV | Max PWM L/R | Notes |
| --- | --- | --- | --- | --- | --- |
| 1 | TBD | TBD | TBD | TBD | Baseline |
| 2 | TBD | TBD | TBD | TBD | Repeatability |
| 3 | TBD | TBD | TBD | TBD | Repeatability |

## Functional Checklist

- OLED shows mode, safety state, battery voltage, speed limit, pose, LiDAR points, and PWM.
- OLED update is scheduled in the main loop and flushes incrementally; it is not done inside the TIM4 interrupt.
- Bluetooth `P` enables periodic status using DMA when UART5 is idle.
- Bluetooth `Q` disables status telemetry.
- Button PC13 long press latches emergency stop and PWM is cut.
- Button PC13 short press toggles stop latch during bench test.
- Button PB2 short press switches OLED pages.
- Button PB2 long press requests return-home.
- Bluetooth `E` latches emergency stop.
- Bluetooth `C` clears emergency stop.
- Bluetooth `G` requests return-home.
- Battery warning requests return-home.
- Battery cutoff latches emergency stop.
- IWDG is refreshed only by the 10 ms application task.

## Video Evidence Checklist

The product video should include:

- Team introduction.
- System demo with OLED visible.
- Safety demo: emergency stop cutting PWM.
- Bluetooth status log visible while the robot runs.
- Evaluation: entry-to-exit and exit-to-entry timing.
- Code structure overview showing `main.c`, `app_ui.c`, `hc04.c`, `pid.c`, `lidar.c`, and this README.
