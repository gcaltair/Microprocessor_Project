# AI Agent Documentation Workflow

This directory defines the repository's unified document organization for AI-assisted development.

## Goals

- Give every agent one stable entry path.
- Separate current source-of-truth documents from historical notes.
- Make documentation updates part of normal engineering work.
- Reduce drift between code, plans, and agent instructions.
- Standardize embedded human-in-the-loop verification.

## Canonical Read Order

Agents should read documents in this order before changing code:

1. `AGENTS.md`
2. `README.md`
3. `docs/README.md`
4. `docs/agent_workflow/README.md`
5. `docs/development_plan/README.md`
6. `docs/development_plan/slam_icp_progress_status.md`
7. `docs/freertos_further_development_plan.md`
8. `docs/agent_workflow/standard_process.md`
9. `CLAUDE.md`

For host-app-only work, also read:

10. `host_app/README.md`
11. `host_app/pyproject.toml`

## Document Roles

### Level 0: Agent operating rules

- `AGENTS.md`
- Mandatory workflow, edit boundaries, verification, and handoff format.

### Level 1: Project entrypoints

- `README.md`
- `docs/README.md`
- High-level overview and navigation only.
- These files should point to deeper authoritative documents instead of duplicating detail.

### Level 2: Current engineering truth

- `docs/development_plan/README.md`
- `docs/development_plan/slam_icp_progress_status.md`
- `docs/freertos_further_development_plan.md`
- These files define the active phase, priorities, current architecture direction, and approved near-term scope.

### Level 3: Background and implementation context

- `docs/agent_workflow/standard_process.md`
- `CLAUDE.md`
- `host_app/README.md`
- Module-specific usage and architecture notes.
- Useful context, but not the final authority on current phase status.

### Level 4: Historical and reference material

- `docs/archive/development_history/slam_icp_ai_agent_plan.md`
- `docs/archive/development_history/slam_icp_development_plan.md`
- `docs/development_plan/slam_mapping_debug_note.md`
- `docs/archive/reports/week4_coding_day_report.md`
- `docs/archive/legacy_root/freertos开发计划.md`
- `docs/archive/references/tmp_brief.txt`
- course brief and protocol PDFs

## Update Protocol

When code changes, update documentation in the same task if the behavior, workflow, or project status changed.

### If firmware behavior changes

Update as needed:

- `docs/development_plan/slam_icp_progress_status.md`
- `docs/freertos_further_development_plan.md`
- `README.md` if user-visible capability or project status summary changed

### If host app behavior or workflow changes

Update as needed:

- `host_app/README.md`
- `README.md` if top-level run/test instructions changed
- `docs/README.md` if document navigation changed

### If priorities, scope boundaries, or current phase change

Update together:

- `docs/development_plan/README.md`
- `docs/development_plan/slam_icp_progress_status.md`
- `AGENTS.md` if agent constraints or preferred edit targets changed

### If verification workflow changes

Update together:

- `docs/agent_workflow/README.md`
- `docs/agent_workflow/standard_process.md`
- `docs/templates/*` as needed
- `AGENTS.md` if the minimum reporting contract changed

## Writing Rules

- Keep one document responsible for one class of truth.
- Prefer linking over repeating the same status in many files.
- Put concrete dates on status snapshots.
- Mark historical documents as historical in index files.
- When two files overlap, the index files must state which one wins.

## Required End-of-Task Documentation Check

Before finishing an implementation task, the agent should check:

1. Did current status change?
2. Did commands, workflows, or interfaces change?
3. Did safe edit boundaries or priorities change?
4. Does any index now point to stale or misleading information?
5. Does the task need a hardware test plan or manual verification record?

If the answer is yes, update the relevant documents in the same task.
