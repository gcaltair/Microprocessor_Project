# Host App

`host_app` is a Windows-first Python + PySide6 desktop application for the STM32 robot car.

It is used to:

- connect to the robot over serial
- switch between text-compatible and structured telemetry workflows
- send manual control, LiDAR, and navigation commands
- visualize runtime state, occupancy map, path, scan points, and logs
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
- `host_app/ui/`: Qt UI widgets and views
- `tests/`: parser and state tests

## Agent Notes

For host-app-only work, read this file together with:

- `../AGENTS.md`
- `../docs/agent_workflow/README.md`
- `pyproject.toml`

If a task changes host-app workflow, protocol usage, or test commands, update this file in the same task.
