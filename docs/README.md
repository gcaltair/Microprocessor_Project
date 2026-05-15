# Documentation Index

This directory contains the current engineering plan, AI-agent workflow documents, historical reports, and external references.

## Read This First

- `agent_workflow/README.md`: unified AI-agent documentation workflow and source-of-truth hierarchy.
- `agent_workflow/standard_process.md`: standard task execution and embedded verification process.
- `development_plan/README.md`: current phase, active priorities, and plan navigation.
- `development_plan/slam_icp_progress_status.md`: latest dated status snapshot.
- `development_plan/slam_icp_current_analysis.md`: current SLAM/ICP issue analysis and next improvement sequence.
- `freertos_further_development_plan.md`: current RTOS-based integration plan and command surface.

## Document Categories

### Current source-of-truth documents

- `agent_workflow/README.md`
- `agent_workflow/standard_process.md`
- `development_plan/README.md`
- `development_plan/slam_icp_progress_status.md`
- `development_plan/slam_icp_current_analysis.md`
- `freertos_further_development_plan.md`

### Standard templates

- `templates/README.md`
- `templates/task_brief.md`
- `templates/status_update.md`
- `templates/hardware_test_plan.md`
- `templates/manual_test_record.md`
- `templates/verification_report.md`

### Task records

- `work_items/README.md`: where filled templates and task-specific verification artifacts should live
- `work_items/2026-05-14_slam-turn-rotation-debug/`: current SLAM turn-after-map-rotation diagnosis and hardware test plan
- `work_items/2026-05-15_slam-icp-current-analysis/`: current SLAM/ICP analysis and documentation handoff

### Background and lower-priority context

- `../CLAUDE.md`: deeper architectural background
- `../host_app/README.md`: host-app usage and workflow

### Historical planning and debugging notes

- `archive/development_history/slam_icp_ai_agent_plan.md`: historical agent task plan
- `archive/development_history/slam_icp_development_plan.md`: original SLAM/ICP breakdown
- `development_plan/slam_mapping_debug_note.md`: mapping debug history and root-cause notes
- `archive/reports/week4_coding_day_report.md`: historical weekly report
- `archive/README.md`: archive index and usage rules

### External references

- `archive/references/problem_of_slam.png`: current reference screenshot for the SLAM map rotation/ghosting issue after turns
- `EBU6475_EoM_Brief_2026_v1.pdf`: course brief and benchmark context
- `RPLIDAR 360度激光扫描测距雷达-通讯接口协议与应用手册.pdf`: LiDAR protocol reference

## Conflict Rule

When files disagree, prefer documents in this order:

1. `AGENTS.md`
2. `agent_workflow/README.md`
3. `development_plan/README.md`
4. `development_plan/slam_icp_progress_status.md`
5. `development_plan/slam_icp_current_analysis.md`
6. `freertos_further_development_plan.md`
7. historical notes and background files
