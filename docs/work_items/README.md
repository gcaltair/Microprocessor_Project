# Work Item Records

This directory stores task-specific working records produced during AI-assisted development.

Use it when a task needs persistent planning, verification, or human-test artifacts beyond a short final summary.

## When To Create A Work Item Folder

Create a folder here when the task involves any of the following:

- firmware behavior changes
- mixed firmware/host protocol changes
- manual or hardware verification
- multi-step debugging
- a task large enough to need a persistent brief and verification log

## Naming Convention

Create folders using:

`YYYY-MM-DD_short-task-name`

Examples:

- `2026-05-12_navigation-arrival-tolerance`
- `2026-05-12_localization-slow-correction-debug`
- `2026-05-12_host-telemetry-parser-update`

## Recommended Contents

For firmware or mixed tasks, prefer this set:

- `task_brief.md`
- `hardware_test_plan.md`
- `manual_test_record.md`
- `verification_report.md`

For documentation-only or small host-app tasks, this may be enough:

- `task_brief.md`
- `verification_report.md`

## Workflow

1. Copy the needed files from `docs/templates/`
2. Fill them in during the task
3. Reference them in the final handoff if they are part of the task evidence
4. Leave them in place for traceability unless the user explicitly wants cleanup

## Rule

Files in this directory are task records, not source-of-truth project status files.

Current phase and repository-wide rules must still be updated in:

- `docs/development_plan/*`
- `docs/agent_workflow/*`
- `AGENTS.md`
- `README.md`
