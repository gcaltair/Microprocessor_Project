# Development Plan Index

This is the current planning entrypoint for agents.

## Current Phase

Status date: 2026-05-14.

The project is in a runnable integration phase:

- Phase 0: complete, RTOS skeleton and interface cleanup.
- Phase 1: complete, LiDAR scan and pose data chain.
- Phase 2: complete, occupancy-grid mapping.
- Phase 3A: complete, localization correction usable for mapping.
- Phase 3B: in progress, control-usable fusion remains conservative.
- Phase 4A: in progress, known-target grid navigation has a first implementation.

Current system goal:

> Keep the current control baseline stable, make SLAM map orientation consistent after turns, then resume known-target navigation stabilization before full exploration and return-to-start work.

## Authoritative Files

Read in this order:

1. `slam_icp_progress_status.md` for actual current status and risks.
2. `../freertos_further_development_plan.md` for current FreeRTOS task architecture and command list.
3. `slam_mapping_debug_note.md` before touching mapping mutexes, ASCII map output, or ray tracing.
4. `../archive/development_history/slam_icp_ai_agent_plan.md` and `../archive/development_history/slam_icp_development_plan.md` for historical phase intent.

## Document Update Contract

Use this directory for active phase tracking, not for long-term historical narration.

- `README.md`: active phase, priorities, and where to look next
- `slam_icp_progress_status.md`: latest dated project snapshot and risks
- `../archive/development_history/slam_icp_ai_agent_plan.md`: historical task decomposition
- `../archive/development_history/slam_icp_development_plan.md`: historical phase breakdown
- `slam_mapping_debug_note.md`: focused debug history

If a task changes current scope, priorities, or status, update `README.md` and `slam_icp_progress_status.md` together.

## Active Priorities

### Priority 1: SLAM turn-to-map orientation consistency

Tasks that fit this priority:

- Diagnose the current issue where the map appears to rotate or duplicate after robot turns.
- Capture `P / G / O / X4` samples before and after `A90`, `A-90`, and smaller turns.
- Decide whether the root cause is ICP reject/odom-only writes, weak yaw correction, premature mapping gate recovery, or LiDAR angle convention.
- Add conservative diagnostics before changing the matching algorithm.
- Keep mapping updates paused after turns until pose quality is trustworthy.

Suggested files:

- `Core/Inc/localization_task.h`
- `Core/Src/localization_task.c`
- `Core/Inc/mapping_task.h`
- `Core/Src/mapping_task.c`
- `Core/Src/hc04.c`
- `docs/work_items/2026-05-14_slam-turn-rotation-debug/`

### Priority 2: Phase 4A navigation stabilization

Tasks that fit this priority:

- Improve `Jx,y`, `J`, and `C` navigation status visibility.
- Add or refine path visualization in ASCII map output.
- Improve arrival tolerance and failure states.
- Add conservative replan behavior when a segment fails.
- Keep path buffers bounded and memory usage visible.

Suggested files:

- `Core/Inc/navigation_task.h`
- `Core/Src/navigation_task.c`
- `Core/Inc/mapping_task.h`
- `Core/Src/mapping_task.c`
- `Core/Src/hc04.c`

### Priority 3: Phase 3B conservative fusion

Tasks that fit this priority:

- Preserve `odometry`, `control_pose`, and `corrected_pose` separation.
- Tighten odometry scale diagnostics and calibration workflow before relying on longer navigation runs.
- Improve diagnostics for drift, correction magnitude, and slow correction.
- Keep PID input stable; do not directly replace control state with LiDAR-corrected pose.

Suggested files:

- `Core/Inc/localization_task.h`
- `Core/Src/localization_task.c`
- `Core/Inc/control_logic.h`
- `Core/Src/control_logic.c`
- `Core/Src/hc04.c`

### Priority 4: resource cleanup

Tasks that fit this priority:

- Review RAM-heavy arrays and path caches.
- Remove duplicate temporary buffers.
- Prefer static bounded memory over dynamic allocation.
- Record build memory usage after firmware builds.

Suggested files:

- `Core/Inc/FreeRTOSConfig.h`
- `Core/Inc/occupancy_grid.h`
- `Core/Src/occupancy_grid.c`
- `Core/Inc/navigation_task.h`
- `Core/Src/navigation_task.c`

## Not Yet

Do not start these unless the user explicitly asks or Phase 4A is verified stable:

- Full frontier exploration.
- Exit detection.
- Return-to-start state machine.
- Large map expansion.
- Heavy ICP, EKF, graph optimization, or global loop closure.

## Verification Gates

For firmware changes:

```powershell
cmake -DCMAKE_BUILD_TYPE=Debug -B cmake-build-debug -G "MinGW Makefiles"
cmake --build cmake-build-debug
```

For host-app changes:

```powershell
cd host_app
pytest
```

For behavior changes that cannot be fully verified on the desktop, include the exact serial command sequence to test on the robot.

For odometry or motion-scale changes, also record the measured forward/reverse travel, resulting `K...` coefficients, and whether `R0 -> motion -> O -> D...` was exercised in the task work item.
