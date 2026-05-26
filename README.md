# Microprocessor Project

STM32F446 robot firmware for LiDAR mapping/navigation, motor control, OLED status UI,
Bluetooth telemetry/commands, safety supervision, and benchmark recording.

## Code Structure

- `Core/Src/freertos.c`: RTOS tasks and the 10 ms real-time control scheduling path.
- `Core/Src/app_ui.c`: background OLED service, buttons, ADC monitoring, watchdog
  service, emergency-stop policy, peripheral health monitoring, and benchmark timers.
- `Core/Src/adc.c`: ADC1 PA4 battery-divider and PC0 speed-potentiometer sampling.
- `Core/Src/gpio.c`, `Core/Src/stm32f4xx_it.c`: PC13 emergency-stop EXTI path.
- `Core/Src/iwdg.c`: independent watchdog setup.
- `Core/Src/hc04.c`: UART5 binary telemetry v8 using DMA transmission.
- `Core/Src/navigation_task.c`: UART5 command parsing, planning, automatic return,
  and entry-to-exit/return benchmark transitions.
- `Core/Src/lidar.c`, `Core/Src/mapping_task.c`: LiDAR DMA ingestion and occupancy grid.
- `Core/Src/pid.c`, `Core/Src/motor.c`: closed-loop movement and PWM actuation.
- `host_app/`: live map/status desktop UI and CSV benchmark logging.

## Build

Recommended toolchain:

- STM32Cube FW_F4 V1.28.2
- GNU Arm Embedded / GNU Tools for STM32
- CMake 3.22+ and Ninja

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

Flash the resulting ELF with STM32CubeProgrammer:

```powershell
STM32_Programmer_CLI -c port=SWD -w build/Debug/FInal_fina.elf -v -rst
```

## Hardware Mapping

| Peripheral | Connection | Use |
| --- | --- | --- |
| UART5 | PC12 TX / PD2 RX, 921600 baud | Bluetooth binary telemetry and commands |
| USART6 | PC6 TX / PC7 RX, 460800 baud | RPLIDAR |
| I2C1 | PB6/PB7, SSD1306 `0x3C` | OLED status display |
| ADC1 CH4 | PA4 | Battery divider voltage input |
| ADC1 CH10 | PC0 | Speed-limit potentiometer, 10 to 40 cm/s |
| TIM3 CH1-CH4 | Motor driver inputs | Motor PWM |
| PC13 / B1 | Falling-edge EXTI | Hardware emergency stop |
| PB2 | Polled, debounced input | OLED page / return-home button |

`PA4` must be driven by the battery divider. A reading below `6.0 V`, including a
disconnected/zero reading, latches stop for fail-safe behavior. Adjust
`BATTERY_DIVIDER_X100`, `BATTERY_WARN_MV`, and `BATTERY_CUTOFF_MV` in
`Core/Src/app_ui.c` to match the actual pack and divider.

The checked-in `.ioc` file predates the manually integrated ADC/IWDG additions. Before
regenerating CubeMX code, mirror ADC1 CH4/CH10, IWDG, and EXTI15_10 settings in CubeMX
so those source-level integrations are retained.

## Runtime Architecture

- `controlTask` runs from the 10 ms timer semaphore at real-time priority. It updates
  odometry and closed-loop motor control, records a heartbeat, and never performs
  OLED, ADC, or Bluetooth formatting/transmission work.
- `safetyTask` runs every 20 ms at above-normal priority. It polls debounced UI
  actions, services OLED pages incrementally, samples ADC values, checks motor/LiDAR
  health, and refreshes IWDG only while the control heartbeat remains current.
- `telemetryTask` runs at low priority and sends version 8 map/status frames by UART5
  DMA, so serial transfer does not stall motor control.

## User Interface

OLED pages:

- Main: control mode, safety reason, battery voltage, speed limit, mapping scan count,
  and benchmark state.
- Pose: `x`, `y`, heading, navigation goal, and remaining goal distance.
- Debug: PWM, measured wheel speed, LiDAR recovery count, exit/return times, and
  Bluetooth telemetry state.

Button behavior:

- PC13/B1 falling edge immediately cuts both PWM outputs in interrupt context.
- A held PC13/B1 press also latches stop through the debounced service path, covering
  a button already active during startup.
- After a stop, a subsequent short PC13/B1 press clears the latch and starts a fresh
  mapping/exploration run unless battery safety blocks motion.
- PB2 short press selects the next OLED page; PB2 long press cancels current motion
  and requests return-home.

## Bluetooth Commands

Commands are ASCII lines terminated by `\n`:

| Command | Function |
| --- | --- |
| `NAV x,y` | Start an outbound navigation benchmark; arrival automatically starts return-home |
| `PLAN x,y` | Plan/show a target without driving |
| `NAVC` | Clear the current goal |
| `Pdx,dy` | Direct relative move through the position controller |
| `P` / `Q` | Enable / disable periodic binary telemetry |
| `E` or `STOP` | Latch emergency stop and cut PWM |
| `C` | Clear stop latch when battery safety permits |
| `G` or `HOME` | Request return-home |
| `START` | Clear permitted stop state, reset the grid, and restart LiDAR streaming |

## Safety Behavior

- Emergency stop: PC13 EXTI executes `Motor_StopAll()` immediately; deferred task
  cleanup cancels navigation and motion state.
- Battery supervision: below `6.4 V` displays `LOWBAT`, cancels current motion, and
  requests return-home;
  below `6.0 V` latches stop.
- Watchdog: the safety task deliberately stops refreshing IWDG if no control heartbeat
  is seen for `250 ms`.
- Motor monitoring: commanded motion with high PWM and no wheel-speed response for
  `1 s` latches a `MOTOR` stop.
- LiDAR monitoring: an active stream without completed scans for `2 s` is restarted;
  the OLED/telemetry reports `LIDAR` and a recovery counter.
- Runtime speed limit: the PC0 potentiometer constrains closed-loop motion to
  `0.10` through `0.40 m/s`.

## Telemetry And Validation

UART5 binary protocol v8 extends the map/control frame with:

- battery voltage and speed limit
- safety code, emergency-stop latch, and LiDAR stream state
- LiDAR recovery count and control heartbeat age
- benchmark state, mission elapsed time, `exit_time_ms`, and `return_time_ms`

Run the host UI and use **Start Recording** / **Save CSV** to retain acceptance
evidence:

```powershell
python .\host_app\main.py --port COM5
```

CSV files are written to `host_app/logs/` and intentionally excluded from source
control because they are run artifacts. See
`docs/performance_validation.md` for the execution checklist and report table.
