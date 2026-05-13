# Host App

`host_app` is a Windows-first Python + PySide6 desktop application for the STM32 robot car.

It is used to:

- connect to the robot over serial
- switch between text-compatible and structured telemetry workflows
- send manual control, LiDAR, and navigation commands
- visualize runtime state, occupancy map, path, scan points, PID diagnostics, and logs
- apply RAM-only PID tuning values and record PID diagnostics for later AI-agent analysis
- record raw serial sessions and replay saved logs

## Install

```powershell
cd host_app
python -m venv .venv
. .venv\Scripts\Activate.ps1
pip install -r requirements.txt
```

## Run

```powershell
cd host_app
python main.py
```

## PID Tuning

The desktop app includes a PID panel. It parses firmware text lines:

- `PID ...`
- `CTRLDBG ...`
- `LCTRL ...`
- `TCTRL ...`

The panel can read all loop gains, read one loop, apply RAM-only gains, and plot recent control diagnostics.

CLI usage:

```powershell
cd host_app
python -m host_app.tools.pid_tuner_cli list-ports
python -m host_app.tools.pid_tuner_cli --port COM7 show --json
python -m host_app.tools.pid_tuner_cli --port COM7 set angle 0.05 0 0.001 --json
python -m host_app.tools.pid_tuner_cli --port COM7 send R0 A90 O --duration 3 --jsonl logs/pid_angle.jsonl
```

PID loop names:

- `angle` / `A`
- `left_speed` / `L`
- `right_speed` / `R`
- `position` / `P`

Firmware tuning commands are RAM-only. Rebooting the controller restores compiled defaults.

## Tests

```powershell
cd host_app
pytest
```

## Package

```powershell
cd host_app
.\build.ps1
```

## Source Layout

- `host_app/app.py`: application entrypoint
- `host_app/protocol/`: command building and telemetry parsing
- `host_app/services/`: controller and session state management
- `host_app/transport/`: serial transport
- `host_app/tools/pid_tuner_cli.py`: PID tuning CLI for logs and later AI-agent integration
- `host_app/ui/`: Qt UI widgets and views
- `tests/`: parser and state tests

## Agent Notes

For host-app-only work, read this file together with:

- `../AGENTS.md`
- `../docs/agent_workflow/README.md`
- `pyproject.toml`

If a task changes host-app workflow, protocol usage, or test commands, update this file in the same task.
