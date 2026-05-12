# Templates Index

Use these templates to standardize AI-agent development records, plans, and embedded verification.

## Templates

- `task_brief.md`: define scope, constraints, and verification expectations before implementation
- `status_update.md`: record dated progress snapshots
- `hardware_test_plan.md`: define a reproducible human hardware test
- `manual_test_record.md`: record the result of a hardware or bench test
- `verification_report.md`: summarize what was verified locally vs manually

## Usage Rules

- Copy the template into the appropriate working document before filling it in.
- Prefer storing filled task artifacts under `docs/work_items/YYYY-MM-DD_short-task-name/`.
- Keep templates unchanged unless you are improving the repository-wide process.
- For firmware behavior changes, prefer pairing:
  - `hardware_test_plan.md`
  - `manual_test_record.md`
  - `verification_report.md`
