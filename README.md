# Microprocessor_Project

STM32F446 robot firmware with FreeRTOS tasks for motor control, LiDAR parsing,
localization, occupancy-grid mapping, navigation, telemetry, UI, and safety
supervision.

## Code Structure

- `Core/Src/main.c` initializes CubeMX peripherals, motors, sensors, PID, app
  state, and watchdog.
- `Core/Src/freertos.c` creates RTOS objects and tasks:
  - `controlTask`: 10 ms odometry, IMU/encoder update, cascaded PID motor control.
  - `lidarParseTask`: USART6 DMA block parser for RPLIDAR scan frames.
  - `localizeTask`: publishes pose snapshots and gates mapping during turns.
  - `mappingTask`: updates the 96x96 occupancy grid.
  - `navigationTask`: UART5 command parser plus A* path planning.
  - `safetyTask`: ADC battery/potentiometer sampling, watchdog refresh, emergency stop, LiDAR and motor health checks.
  - `telemetryTask`: non-blocking UART5 DMA telemetry frames for Bluetooth/host app.
  - `uiTask`: debounced B1 button handling and SSD1306 OLED status pages.
- `Core/Src/robot_app.c` centralizes robot mode, safety status, debug commands,
  return-home behavior, and online speed limit updates.
- `Core/Src/oled_ssd1306.c` is a small SSD1306 I2C text driver.
- `Core/Src/adc.c` reads battery voltage and potentiometer inputs.
- `host_app/` contains the Python telemetry viewer and protocol tests.
- `docs/` contains benchmark and demo preparation templates.

## Hardware Mapping

Default mappings are in `Core/Inc/robot_config.h` and `Core/Inc/main.h`.

- OLED: SSD1306 on `I2C1` (`PB6=SCL`, `PB7=SDA`), address `0x3C`.
- User button: `B1` on `PC13`; short press starts or switches display page,
  long press requests return home.
- Emergency stop: `PB2`, active low with pull-up. EXTI immediately forces all
  TIM3 PWM compare values to zero.
- Battery ADC: `PA4 / ADC1_IN4`, default divider ratio `3.0`.
- Potentiometer ADC: `PC0 / ADC1_IN10`, maps max speed from `0.06` to
  `0.22 m/s`.
- Motor PWM: `TIM3 CH1..CH4`.
- LiDAR: `USART6` with DMA RX.
- Bluetooth/host telemetry: `UART5` at `921600`.

Adjust voltage divider, low-voltage thresholds, and pin assumptions in
`Core/Inc/robot_config.h` before hardware testing.

## UART5 Debug Commands

Send commands terminated by newline:

- `START` or `RUN`: enter exploration mode.
- `STOP` or `HALT`: stop motion and clear navigation target.
- `ESTOP`: software emergency stop.
- `CLEAR`: clear emergency stop after PB2 is released.
- `HOME` or `RETURN`: navigate back to `(0, 0)`.
- `DISP`: switch OLED page.
- `SPD <mps>`: set runtime max base speed.
- `PID <A|L|R|P> <kp> <ki> <kd>`: tune angle, left speed, right speed, or position PID.
- `WT <left_mps> <right_mps>`: wheel-speed test mode.
- `STAT`: print concise robot status.
- Existing navigation commands remain available: `NAV x y`, `PLAN x y`,
  `NAVC`, and `Pdx,dy`.

## Build

From the repository root:

```powershell
cmake --preset Debug
cmake --build --preset Debug
```

Flash with:

```powershell
STM32_Programmer_CLI -c port=SWD -w build/Debug/FInal_fina.elf -v -rst
```

## Host Telemetry Viewer

```powershell
cd host_app
python -m pip install -r requirements.txt
python main.py
```

If `requirements.txt` is not present, install `pyserial`, `numpy`, and `PyQt6`.

## Verification Artifacts

- Use `docs/benchmark_log_template.csv` to record entrance-to-exit and
  exit-to-entrance runs.
- Use `docs/performance_validation.md` for repeatability notes, score
  calculation, and fault/safety evidence.
- Use `docs/video_checklist.md` before recording product and benchmark videos.
