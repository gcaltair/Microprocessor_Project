# Agent Instructions

This file is the standard entrypoint for Codex, Claude Code, and other coding agents working in this repository.

## Read Order

Before editing code, read these files in order:

1. `README.md`
2. `docs/README.md`
3. `docs/agent_workflow/README.md`
4. `docs/agent_workflow/standard_process.md`
5. `docs/development_plan/README.md`
6. `docs/development_plan/slam_icp_progress_status.md`
7. `docs/freertos_further_development_plan.md`
8. `CLAUDE.md` for deeper architectural background

For host-app-only work, also read:

9. `host_app/README.md`
10. `host_app/pyproject.toml`

## Documentation Organization

Use the repository's unified documentation workflow:

- `AGENTS.md`: mandatory agent operating rules
- `README.md`: top-level project overview
- `docs/README.md`: documentation index
- `docs/agent_workflow/README.md`: source-of-truth hierarchy and documentation update protocol
- `docs/agent_workflow/standard_process.md`: standard execution flow and verification ladder
- `docs/development_plan/README.md`: active phase and priority index
- `docs/development_plan/slam_icp_progress_status.md`: latest dated status snapshot
- `docs/freertos_further_development_plan.md`: current RTOS integration and command surface
- `CLAUDE.md`: deeper background and lower-priority historical context

Historical or debugging documents should remain available, but must not silently override the current phase files above.

## Current Engineering Direction

The current phase is not basic FreeRTOS migration. The firmware already has the main task chain, LiDAR scan handling, occupancy-grid mapping, localization correction, ASCII map export, and initial known-target navigation.

Priorities:

1. Stabilize Phase 4A known-target grid navigation.
2. Continue Phase 3B control-usable fusion conservatively.
3. Reduce RAM pressure before adding frontier exploration, return-to-start, or larger planning caches.

Do not implement full frontier exploration, exit detection, or return-to-start until Phase 4A navigation behavior is stable and resource usage has been reviewed.

## Safe Edit Boundaries

Preferred firmware edit targets:

- `Core/Inc/slam_types.h`
- `Core/Inc/occupancy_grid.h`
- `Core/Src/occupancy_grid.c`
- `Core/Inc/scan_preprocess.h`
- `Core/Src/scan_preprocess.c`
- `Core/Inc/localization_task.h`
- `Core/Src/localization_task.c`
- `Core/Inc/mapping_task.h`
- `Core/Src/mapping_task.c`
- `Core/Inc/navigation_task.h`
- `Core/Src/navigation_task.c`
- `Core/Inc/control_logic.h`
- `Core/Src/control_logic.c`
- `Core/Src/hc04.c`
- `Core/Src/lidar.c`

Use care with generated or shared files:

- `Core/Src/freertos.c`: scheduling and RTOS wiring only; avoid algorithm logic here.
- `Core/Src/main.c`, `Core/Inc/main.h`: STM32CubeMX-owned surface; keep edits minimal.
- `Core/Src/usart.c`, `tim.c`, `gpio.c`, `dma.c`, `spi.c`, `i2c.c`: generated peripheral setup; do not refactor casually.
- `Drivers/` and `Middlewares/`: vendor code; avoid edits unless explicitly required.

Generated or packaged outputs should not be edited manually:

- `cmake-build-debug/`
- `host_app/build/`
- `host_app/dist/`
- `host_app/release/`
- `host_app/.pyinstaller/`
- `host_app/logs/`
- `__pycache__/`
- `*.pyc`

Some generated host-app artifacts are currently tracked in git history. Do not remove or rewrite them unless the user explicitly asks for repository cleanup.

## Firmware Architecture Rules

- Keep control stability ahead of mapping or navigation accuracy.
- Do not feed LiDAR correction directly into PID control.
- Keep `odometry`, `control_pose`, and `corrected_pose / EST` conceptually separate.
- Keep SLAM and navigation modules bounded in memory; RAM usage is already high.
- Prefer fixed-size buffers and explicit bounds checks.
- Do not introduce uncontrolled dynamic allocation in firmware application code.
- One task should solve one closed-loop problem; avoid broad multi-module rewrites.

## Verification Commands

Firmware:

```powershell
cmake -DCMAKE_BUILD_TYPE=Debug -B cmake-build-debug -G "MinGW Makefiles"
cmake --build cmake-build-debug
```

Host app:

```powershell
cd host_app
pytest
```

If a command cannot be run because the local toolchain is unavailable, report that explicitly and provide the narrower verification that was performed.

## Embedded Verification Rule

For firmware behavior changes, desktop verification alone is usually insufficient.

If the agent cannot perform the real hardware test, it must provide:

- a hardware test plan
- the exact command sequence to run
- expected outputs or behavior
- explicit manual verification status

Use the templates under `docs/templates/`.

## Documentation Update Rule

If a task changes current behavior, workflow, priorities, or agent guidance, update the relevant documentation in the same task.

At minimum, review whether the task should update:

- `docs/development_plan/slam_icp_progress_status.md`
- `docs/development_plan/README.md`
- `docs/freertos_further_development_plan.md`
- `host_app/README.md`
- `README.md`

## Required Handoff Format

End every coding task with:

- Task goal
- Files changed
- Key behavior changes
- Build/test result
- Verification level reached
- Known risks
- Suggested next step

For development-plan-only tasks, include the updated plan files and the next implementation target.
