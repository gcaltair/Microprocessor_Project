# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Current Planning Entry Point

For current development status and agent workflow, read these first:

1. `AGENTS.md`
2. `README.md`
3. `docs/README.md`
4. `docs/agent_workflow/README.md`
5. `docs/development_plan/README.md`
6. `docs/development_plan/slam_icp_progress_status.md`

The detailed architecture notes below are useful background, but the current phase is tracked in `docs/development_plan/README.md`.

## Status Warning

Some sections in this file preserve historical architecture notes from earlier migration stages.

Current source of truth:

- FreeRTOS task architecture is live in `Core/Src/freertos.c`
- Current phase and priorities live in `docs/development_plan/*`
- If a section below conflicts with those files, prefer the newer files

## Project Overview

This is an STM32F446-based robot car project for a microprocessor course competition. The robot navigates a 5x5 meter maze autonomously using SLAM (Simultaneous Localization and Mapping), with capabilities for mapping, path planning, and autonomous exploration.

**Target MCU**: STM32F446xx (Cortex-M4, 180MHz, FPU)
**Build System**: CMake with arm-none-eabi-gcc toolchain
**Project Name**: FInal_fina
**RTOS**: FreeRTOS V10.3.1 (CMSIS-RTOS V2 wrapper)

## Build Commands

**Prerequisites**: arm-none-eabi-gcc toolchain must be in PATH.

```bash
# Configure (Debug build)
cmake -DCMAKE_BUILD_TYPE=Debug -B cmake-build-debug -G "MinGW Makefiles"

# Configure (Release build)
cmake -DCMAKE_BUILD_TYPE=Release -B cmake-build-release -G "MinGW Makefiles"

# Build
cmake --build cmake-build-debug

# Build with verbose output
cmake --build cmake-build-debug --verbose

# Clean
cmake --build cmake-build-debug --target clean
```

**Output Files**:
- `cmake-build-debug/FInal_fina.elf` - ELF executable
- `cmake-build-debug/FInal_fina.map` - Linker map file

## Flashing/Debugging

The project is typically flashed using STM32CubeProgrammer or OpenOCD. In CLion/STM32CubeIDE:

1. Connect ST-Link to the board
2. Use "Download" or "Debug" configuration
3. SWD interface at default pins (SWDIO: PA13, SWCLK: PA14)

## Architecture Overview

### Historical Control Loop Architecture (Pre-FreeRTOS Migration)

This section is historical background only.

The current code no longer runs the main control loop in bare-metal `while(1)` after `osKernelStart()`. The active task wiring now lives in `Core/Src/freertos.c`.

```
main():
  ├─ HAL_Init(), SystemClock_Config()
  ├─ MX_GPIO/DMA/TIM/UART/SPI_Init()
  ├─ system_init(): Motor, Bluetooth, Encoder, IMU, LiDAR
  ├─ PID_system_init()
  ├─ osKernelInitialize()
  ├─ MX_FREERTOS_Init(): Creates defaultTask only (empty loop)
  └─ osKernelStart()  <-- SHOULD NOT RETURN

  while(1)  <-- historical state before task migration
    ├─ LIDAR_ParseTask()          // Parse LiDAR data from ring buffer
    ├─ if (g_system_update_flag): // Set by TIM4 @ 100Hz
    │   ├─ MPU_update()           // Read IMU (gyro for heading)
    │   ├─ encoder_update_speed() // Calculate wheel speeds
    │   ├─ Odometry_Update(dt)    // Update pose (x, y, theta)
    │   └─ Control Mode:
    │       ├─ CONTROL_MODE_MANUAL: Angle_Speed_Cascade_Control()
    │       └─ CONTROL_MODE_POSITION: Update_Relative_Move_PID()
    └─ if (scan_data_ready_flag): // Send LiDAR data via Bluetooth
        └─ send_binary_packaged_data()
```

### Historical Planned FreeRTOS Task Architecture

Per earlier planning documents, the migration target structure was:

| Task | Priority | Period | Responsibility |
|------|----------|--------|----------------|
| SensorTask | High | 10ms | MPU6500 read, encoder updates |
| ControlTask | High | 10ms | PID control, motor output |
| LiDAR_ParseTask | Normal | Loop | Parse ring buffer data |
| MappingTask | Normal | 50ms | Occupancy grid map updates |
| PlanningTask | Low | Event | A*, frontier detection |
| UITask | Low | 100ms | OLED, Bluetooth telemetry |
| SafetyTask | Highest | Event | Watchdog, emergency stop |

### Key Modules

**Motor Control** (`motor.c/motor.h`):
- TIM3 PWM channels: CH1/CH2 (left motor), CH3/CH4 (right motor)
- Dead zone compensation: 686 PWM units (`MOTOR_DEAD_ZONE`)
- Direction control via complementary PWM

**Encoder/Odometry** (`encoder.c/encoder.h`):
- TIM1 (right wheel), TIM2 (left wheel) in encoder mode
- 380 PPR encoders, 65mm wheel diameter, 165mm wheelbase
- Speed filtering with alpha=0.8, max speed clamp at 1.2 m/s
- Odometry uses gyro integration for heading (not encoder differential)

**PID Control** (`pid.c/pid.h`):
- Three controllers: speed left/right, angle (heading), position
- Cascaded structure: Angle PID outputs speed differential
- Integral anti-windup with dynamic integral_max
- Deadband for angle control: ±1.0°
- State machine for relative position moves (TURNING → DRIVING)

**MPU6500 IMU** (`MPU6500.c/MPU6500.h`):
- SPI2 interface (PB12-15: CS/SCK/MISO/MOSI), ±1000dps gyro range
- Calibration on init (100 samples, 5ms interval)
- Low-pass filter (alpha=0.8) on gyro outputs
- Gyro Z used for heading integration (`g_th_continuous`)

**RPLIDAR A1** (`lidar.c/lidar.h`):
- USART6 @ 115200 baud, interrupt-driven RX
- 5-byte protocol parsing with ring buffer (4096 bytes)
- Binary data packaging for Bluetooth transmission
- Ping-pong DMA TX buffers (8KB each)
- Packet header: 0xA5 0x5A + payload length

**Bluetooth Command Interface** (`hc04.c/hc04.h`):
- UART5 with DMA RX (idle line detection)
- Commands:
  - F/B/L/R/S: Movement (sets base speed and angle setpoint)
  - V{speed}: Set speed magnitude (0-10.0)
  - A{angle}: Turn relative angle (e.g., A-90.5)
  - P{x},{y}: Position mode - relative move (e.g., P0.5,1.2)
  - M/N: LiDAR start/stop
  - H: Help
- Binary protocol for telemetry: odometry + LiDAR points

### Memory Layout

- Linker script: `STM32F446XX_FLASH.ld`
- FreeRTOS heap: 15KB (heap_4 allocator)
- Stack/heap configured for embedded target
- No dynamic memory allocation in application code
- Static buffers for LiDAR (1000 points max), DMA TX (8KB x2)

### Peripheral Mapping

| Peripheral | Instance | Pins | Purpose |
|------------|----------|------|---------|
| PWM | TIM3 | PA6, PA7, PB0, PB1 | Motor control |
| Encoder | TIM1/TIM2 | PA8-9, PA0-1 | Wheel encoders |
| System Tick | TIM4 | Internal | 100Hz control loop (to be replaced by FreeRTOS tick) |
| IMU | SPI2 | PB12-15 (CS/SCK/MISO/MOSI) | MPU6500 |
| LiDAR | USART6 | PC6 (TX), PC7 (RX) | RPLIDAR A1 |
| Bluetooth | UART5 | PC12 (TX), PD2 (RX) | HC-04 module |
| I2C | I2C1 | PB6, PB7 | Reserved |

### Coordinate Systems

**Robot Frame**: X forward, Y left, theta CCW positive (standard right-hand rule)
**World Frame**: Same orientation, origin at startup position
**Angle Convention**: Degrees, continuous accumulation in `g_th_continuous`, normalized in `g_th`

### Control Modes

1. **MANUAL** (`CONTROL_MODE_MANUAL`): Direct F/B/L/R commands set base speed and angle setpoint
2. **POSITION** (`CONTROL_MODE_POSITION`): Relative move command (P{dx},{dy}) executes turn-then-drive sequence via state machine

## Important Constants

```c
// PID Limits
#define MOTOR_DEAD_ZONE 686          // Minimum PWM to overcome friction
#define MAX_BASE_SPEED 0.4f          // m/s
#define MIN_BASE_SPEED 0.05f         // m/s
#define POSITION_REACHED_THRESHOLD 0.05f  // 5cm position accuracy

// Mechanical
#define ENCODER_PULSES_PER_REV 380
#define DIAMETER 0.065f              // 65mm wheels
#define WHEEL_BASE 0.165f            // 165mm track width

// Timing
#define dt 0.01f                     // 10ms control period
```

## FreeRTOS Configuration

Key settings from `FreeRTOSConfig.h`:
- `configTICK_RATE_HZ`: 1000 (1ms tick)
- `configTOTAL_HEAP_SIZE`: 15360 bytes
- `configMAX_PRIORITIES`: 56
- `configUSE_MUTEXES`: 1
- `configUSE_COUNTING_SEMAPHORES`: 1
- `configUSE_TIMERS`: 1
- Memory allocation: heap_4

## Development Notes

- **C Standard**: C11 with GNU extensions
- **Optimization**: Debug (-O0 -g3), Release (-Os -g0)
- **Float printf**: Enabled via `-u _printf_float` linker flag
- **Critical Sections**: Global variables shared between main loop and ISRs use `volatile`
- **DMA Safety**: `is_dma_tx_busy` flag prevents buffer collisions in LiDAR TX
- **Current Branch**: `dev/Feature/FreeRTOS` - migration in progress

## Common Issues

1. **Motor not moving**: Check `MOTOR_DEAD_ZONE` - PWM must exceed this value
2. **LiDAR data corruption**: Verify ring buffer overflow flag `lidar_raw_overflow`
3. **Gyro drift**: Ensure calibration is performed on stationary robot
4. **Bluetooth disconnect**: DMA TX busy flag may stall if callback not firing
5. **FreeRTOS task not running**: Current implementation still uses bare-metal main loop

## Migration TODO (FreeRTOS Integration)

Per team schedule in `docs/team_worksplit_analysis.md`:

- [ ] Move control logic from `while(1)` to dedicated FreeRTOS tasks
- [ ] Implement inter-task communication (queues for sensor data, commands)
- [ ] Add mutex protection for shared resources (g_x, g_y, g_th_continuous)
- [ ] Replace `g_system_update_flag` polling with `osDelay()` or notification
- [ ] Create proper task priorities and verify timing
- [ ] Add stack usage monitoring (`uxTaskGetStackHighWaterMark`)
- [ ] Implement SafetyTask for watchdog and fault handling
- [ ] Test continuous operation >10 minutes for stability

## Team Documentation

See `docs/team_worksplit_analysis.md` for the 6-person team分工 (task distribution) and 6-week development schedule covering:
- SLAM core (mapping + localization) - Week 1-6
- Planning & navigation (A*, frontier detection) - Week 1-6
- **FreeRTOS integration - Week 1-6 (current focus)**
- UI/safety/verification - Week 4-6
- Hardware bring-up - Week 1-4
- Documentation & assembly - Week 5-6
