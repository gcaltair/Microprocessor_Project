# Microprocessor Project

STM32F446 robot car project for autonomous maze navigation, mapping, and host-side visualization.

The repository has two active development surfaces:

- Firmware: `Core/`, `cmake/`, `CMakeLists.txt`, `FInal_fina.ioc`
- Host app: `host_app/`

Start here when using an agent:

1. Read `AGENTS.md` for working rules, safe edit boundaries, and verification commands.
2. Read `docs/README.md` for the documentation map.
3. Read `docs/agent_workflow/README.md` for the document hierarchy and update protocol.
4. Read `docs/agent_workflow/standard_process.md` for the standard AI-agent execution and verification process.
5. Read `docs/development_plan/README.md` for the current phase and next tasks.
6. Read `docs/development_plan/slam_icp_progress_status.md` for the latest project state.
7. Use `CLAUDE.md` as detailed background, not as the single source of truth for current phase status.

## Current Status

As of 2026-04-25, the project is beyond the initial FreeRTOS migration. The firmware has a running task architecture, LiDAR parsing, occupancy-grid mapping, lightweight localization correction, ASCII map output, and an initial known-target grid navigation path.

The active work is:

- Phase 3B: conservative control-usable fusion, without feeding LiDAR correction directly into PID.
- Phase 4A: stabilize known-target grid navigation before full exploration.
- Resource cleanup: RAM is close to the limit, so new buffers and path caches must be justified.

## Directory Map

| Path | Purpose | Agent guidance |
| --- | --- | --- |
| `Core/Inc/` | Firmware public headers and STM32CubeMX generated headers | Add module headers here when they are shared across firmware modules. |
| `Core/Src/` | Firmware application, drivers, tasks, and generated sources | Keep algorithm modules separate from `freertos.c`; do not pile logic into the scheduler file. |
| `Drivers/` | STM32 HAL and CMSIS vendor code | Treat as vendor code. Avoid edits unless the task explicitly requires HAL-level changes. |
| `Middlewares/` | FreeRTOS middleware | Treat as vendor code. Avoid edits except RTOS configuration or explicit porting tasks. |
| `cmake/` | Toolchain and STM32CubeMX CMake integration | Change only for build-system tasks. |
| `host_app/` | Python/PySide6 host app for serial control, telemetry, maps, PID diagnostics, and logs | Use normal Python tests before changing host protocol/UI behavior. |
| `docs/` | Course brief, reports, and development plans | Keep current status and agent plans indexed here. |
| `cmake-build-debug/` | Local firmware build output | Generated artifact; do not edit manually. |
| `host_app/build/`, `host_app/dist/`, `host_app/release/` | Host app packaging output | Generated or release artifacts; do not touch unless the task is packaging. |

## Firmware Build

Prerequisite: `arm-none-eabi-gcc` and CMake generator support available in `PATH`.

```powershell
cmake -DCMAKE_BUILD_TYPE=Debug -B cmake-build-debug -G "MinGW Makefiles"
cmake --build cmake-build-debug
```

Expected outputs:

- `cmake-build-debug/FInal_fina.elf`
- `cmake-build-debug/FInal_fina.map`

## Host App

```powershell
cd host_app
python -m venv .venv
. .venv\Scripts\Activate.ps1
pip install -r requirements.txt
python main.py
```

Run host-app tests:

```powershell
cd host_app
pytest
```

## Development Rule

Every agent change should be small, phase-aligned, and verifiable. If a change affects firmware behavior, run or explain the firmware build result. If it affects the host app, run or explain the relevant Python tests.

Documentation changes follow the workflow in `docs/agent_workflow/README.md`. Current status belongs in the development-plan files, while historical notes and references stay under `docs/` as supporting material.

Archived and superseded documents are indexed in `docs/archive/README.md` and should not override the active development-plan files.

Standard process and reusable templates for AI-assisted development live under `docs/agent_workflow/` and `docs/templates/`.
