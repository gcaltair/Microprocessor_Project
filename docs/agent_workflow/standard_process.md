# Standard AI Agent Development Process

This document defines the standard execution process for AI-assisted development in this repository.

It is optimized for embedded work where many behavior changes cannot be fully verified on the desktop and require human-in-the-loop hardware testing.

## 1. Task Intake

Before changing code, the agent should classify the task:

### Type A: documentation-only

- plans
- status updates
- process rules
- archive cleanup

### Type B: host-app-only

- Python application
- telemetry parsing
- UI and logging
- replay tooling

### Type C: firmware logic

- control
- localization
- mapping
- navigation
- command handling

### Type D: mixed workflow

- firmware + host protocol
- firmware + host visualization
- task/documentation/verification changes together

## 2. Required Pre-Work

For every coding task:

1. Read the canonical entrypoint documents from `AGENTS.md`.
2. Confirm the current phase and active priorities.
3. Identify the smallest safe edit surface.
4. Decide what can be verified locally and what requires hardware.

## 3. Change Strategy

Default strategy:

1. make the smallest phase-aligned change
2. preserve current architecture boundaries
3. keep verification close to the change
4. update documents in the same task when behavior or workflow changes

For embedded tasks, prefer:

- bounded buffers
- explicit failure states
- serial-visible diagnostics
- conservative feature rollout

## 4. Verification Ladder

The project uses a four-level verification model.

### Level 0: static review

- code inspection
- bounds checks
- API compatibility review
- document consistency check

### Level 1: desktop verification

- firmware build
- host-app tests
- parser/unit tests
- replay-log validation

### Level 2: bench or simulated operator verification

- command sequence prepared for a human tester
- expected serial output documented
- expected task states and transitions documented
- failure conditions documented

### Level 3: hardware verification

- real robot execution
- operator records actual observations
- deviations from expectation written down

Agents must clearly report the highest level actually reached.

## 5. Embedded Human-Test Mechanism

Because many firmware tasks cannot be fully closed on the desktop, every behavior-changing firmware task should produce a human-test package when hardware verification is not performed by the agent.

The package must include:

1. test objective
2. prerequisites
3. exact command sequence
4. expected outputs or visible behavior
5. failure signs
6. rollback or safe-stop command
7. result status:
   - not run
   - pass
   - fail
   - pass with deviations

Use `docs/templates/hardware_test_plan.md` and `docs/templates/manual_test_record.md`.
Store filled records under `docs/work_items/`.

## 6. Required Artifacts By Task Type

### Documentation-only tasks

- updated target documents
- short summary of changed rules or structure

### Host-app-only tasks

- code changes
- `pytest` result or explicit reason not run
- affected protocol or UI notes
- README update if workflow changed

### Firmware logic tasks

- code changes
- firmware build result or explicit reason not run
- hardware test plan
- manual verification status
- known unverified risk
- persistent task record under `docs/work_items/` when the task is non-trivial

### Mixed workflow tasks

- both firmware and host verification evidence
- interface compatibility notes
- updated docs for both surfaces

## 7. Required End-of-Task Output

Every coding task should end with:

- task goal
- files changed
- key behavior changes
- build/test result
- verification level reached
- known risks
- suggested next step

For firmware behavior changes, also include:

- hardware test plan file
- manual verification status

## 8. Status Labels

Use these labels consistently in plans and test records:

- `planned`
- `in_progress`
- `blocked`
- `desktop_verified`
- `needs_hardware_test`
- `hardware_verified`
- `closed`

## 9. When The Agent Must Stop And Ask

The agent should stop and ask only when:

- hardware access is required to claim completion and no human result is available
- the change touches generated/vendor files without a clear need
- the requested change conflicts with current phase boundaries
- there is no safe way to infer expected behavior from existing code or docs

Otherwise, proceed and leave a precise human-test package.
