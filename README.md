# Microprocessor Project

STM32F446 robot firmware for motor control, LiDAR telemetry, Bluetooth commands, OLED status UI, and safety supervision.

## Code Structure

- `Core/Src/main.c`: system startup and main loop scheduling.
- `Core/Src/app_ui.c`: OLED pages, button handling, battery/speed limit sampling, telemetry switch, emergency stop, return-home request, and watchdog refresh.
- `Core/Src/adc.c`: ADC1 PA4 sampling for battery divider or knob input.
- `Core/Src/iwdg.c`: independent watchdog setup.
- `Core/Src/hc04.c`: UART5 Bluetooth command parser.
- `Core/Src/lidar.c`: RPLIDAR parsing and DMA packet transmission.
- `Core/Src/pid.c`: speed, angle, and relative-position PID control.
- `Core/Src/encoder.c`: wheel speed and odometry update.
- `Core/Src/motor.c`: TIM3 PWM motor output.

## Build

Recommended toolchain:

- STM32Cube FW_F4 V1.28.2
- GNU Arm Embedded / GNU Tools for STM32
- CMake 3.22+ and Ninja, or CLion STM32 CMake profile

Command-line build:

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

If CMake is not in `PATH`, build from CLion or add the CMake binary directory to `PATH`.

## Hardware Mapping

- UART5: Bluetooth status/commands, 9600 baud in generated `usart.c`.
- USART6: RPLIDAR, 460800 baud.
- I2C1 PB6/PB7: SSD1306 OLED, address `0x3C`.
- ADC1 PA4: battery divider or speed-limit knob input.
- TIM3 CH1-CH4: motor PWM.
- PC13: user button, short press toggles stop latch, long press latches emergency stop.
- PB2: auxiliary button, short press switches OLED page, long press requests return-home.

## Bluetooth Commands

- `F`, `B`, `L`, `R`, `S`: manual forward/back/left/right/stop.
- `V<number>`: set requested manual speed. Runtime safety clamps it to the ADC-derived speed limit.
- `A<number>`: relative angle adjustment.
- `P<x>,<y>`: start relative move.
- `M`, `N`: start/stop LiDAR.
- `P`: enable periodic non-blocking status telemetry.
- `Q`: disable periodic status telemetry.
- `E`: latch emergency stop and cut PWM.
- `C`: clear emergency stop latch.
- `G`: request return-home from current odometry.
- `H`: show command help.

## OLED Pages

The UI is updated from the main loop, not from the timer interrupt. It flushes one SSD1306 page per 10 ms tick to avoid blocking the motor-control ISR/timing path.

- Main page: mode, safety state, battery voltage, speed limit, LiDAR points, PWM output.
- Pose page: `x`, `y`, heading, relative-move state, return-home hint.
- Debug page: wheel speeds, angle setpoint, base speed, Bluetooth telemetry state, LiDAR overflow count.

## Safety Behavior

- Hardware emergency stop: PC13 long press or Bluetooth `E` latches stop and calls `Motor_StopAll()`.
- Stop latch: PC13 short press toggles stop/clear for bench testing.
- Low battery: PA4 voltage below warning threshold requests return-home; below cutoff latches stop.
- Watchdog: IWDG is refreshed only from the 10 ms application task. If the control tick stops progressing, the watchdog is not refreshed.
- Speed limiting: ADC input constrains `base_car_speed` before PID output is generated.

Battery constants are in `Core/Src/app_ui.c`:

- `BATTERY_DIVIDER_X100`
- `BATTERY_WARN_MV`
- `BATTERY_CUTOFF_MV`
- `SPEED_LIMIT_MIN_CMPS`
- `SPEED_LIMIT_MAX_CMPS`

## Validation

See `docs/performance_validation.md` for the baseline test checklist, timing fields, and expected video evidence.
