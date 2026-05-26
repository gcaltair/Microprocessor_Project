# Performance and Safety Validation

This document is the execution record template for the UI, safety, and benchmark
acceptance work. Timing values must be collected from a physical run; they are not
manufactured during a firmware build.

## Required Setup

- Flash the current `Debug` or `Release` firmware to the STM32F446 board.
- Connect the OLED, LiDAR, encoders/motor driver, PC13 emergency-stop button, PB2
  function button, PA4 battery divider, and PC0 speed potentiometer.
- Pair/connect the UART5 Bluetooth link at `921600` baud.
- Start `host_app/main.py`, connect the serial port, and verify binary frames arrive.

The PA4 divider must be calibrated before driving. Firmware treats a zero ADC battery
reading as low voltage and disables motion.

## Benchmark Workflow

1. Place the robot at the map entry/home origin with a safe battery voltage.
2. In the host UI select **Start Recording**.
3. Send `NAV x,y` for the exit location. This initializes the benchmark timer.
4. The robot records `exit_time_ms` at the exit and automatically assigns home as the
   return target.
5. The robot records `return_time_ms` once it reaches home.
6. Select **Stop Recording**, then **Save CSV**.
7. Repeat for at least three runs and copy values from the completed telemetry frames
   into the results table.

The reported definitions are:

| Field | Definition |
| --- | --- |
| `exit_time_ms` | `NAV` accepted to exit target reached |
| `return_time_ms` | Exit target reached to home reached |
| `mission_elapsed_ms` | Total elapsed time since `NAV`; fixed at completion |
| `benchmark_state` | `outbound`, `returning`, or `complete` |

## Baseline Results

| Run | Exit Time (ms) | Return Time (ms) | Total Time (ms) | Minimum Battery (mV) | LiDAR Recoveries | Safety Events | CSV / Notes |
| --- | ---: | ---: | ---: | ---: | ---: | --- | --- |
| 1 | TBD | TBD | TBD | TBD | TBD | TBD | Physical run required |
| 2 | TBD | TBD | TBD | TBD | TBD | TBD | Repeatability |
| 3 | TBD | TBD | TBD | TBD | TBD | TBD | Repeatability |

## Functional Verification

| Check | Evidence to capture | Result |
| --- | --- | --- |
| OLED status | Main page shows mode, safety, battery, speed limit, map count, benchmark state | TBD |
| OLED pose/debug | PB2 cycles goal/pose and timing/PWM/LiDAR pages | TBD |
| Button start action | After a stopped state, PC13 short press starts a new mapping/exploration run when battery is safe | TBD |
| Runtime adjustment | Turn PC0 and observe `speed_limit_cmps` change while driving | TBD |
| Bluetooth reporting | Host receives map/status frames continuously during control | TBD |
| Navigation timing | Completed run has nonzero `exit_time_ms` and `return_time_ms` in CSV | TBD |

## Safety Verification

Perform motor tests with the chassis safely lifted or restrained.

| Check | Expected behavior | Evidence / Result |
| --- | --- | --- |
| Hardware emergency stop | Press PC13/B1 while moving; PWM goes to zero immediately and status is stopped | TBD |
| Low-voltage warning | Lower simulated PA4 voltage below `6.4 V`; `LOWBAT` appears and return-home is requested | TBD |
| Low-voltage cutoff | Lower simulated PA4 voltage below `6.0 V`; stop latch prevents motion | TBD |
| Watchdog | Prevent control heartbeat in a dedicated bench build; IWDG resets the MCU after refresh ceases | TBD |
| Motor stall | Command motion while wheels cannot respond; `MOTOR` stop occurs after about `1 s` | TBD |
| LiDAR recovery | Interrupt LiDAR scan stream for more than `2 s`; recovery count increments after restart | TBD |

## Non-Blocking Design Evidence

- `StartControlTask()` contains only sensor/odometry/control work and
  `App_RecordControlTick()`; it does not call OLED, ADC, or UART transmit routines.
- `App_ServiceTask()` runs OLED updates, ADC monitoring, fault checks, and IWDG
  refresh in `safetyTask`.
- `StartTelemetryTask()` submits UART5 map/status frames with
  `HAL_UART_Transmit_DMA()` only while UART5 is ready.
- The telemetry field `control_age_ms` provides run-time evidence that UI and
  Bluetooth servicing are not starving the control task.

## Video Evidence Checklist

- Team introduction and hardware wiring view.
- OLED main/pose/debug page demonstration.
- Live host application showing status telemetry and map target.
- Potentiometer speed-limit adjustment demonstration.
- Hardware emergency-stop demonstration with visible stopped wheels/PWM.
- One complete outbound and return run with the final CSV timing fields shown.
- Brief code structure walkthrough of `app_ui.c`, `freertos.c`, `hc04.c`,
  `navigation_task.c`, `pid.c`, and this document.
