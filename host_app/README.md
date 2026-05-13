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

## PID 调参

桌面上位机右侧有 `PID` 面板，调参时建议按这个流程使用：

1. 连接串口后点击 `Read All`，读取当前四个 PID 环参数。
2. 选择 `angle / left_speed / right_speed / position`，点击 `Load` 把当前值载入输入框。
3. 修改 `Kp / Ki / Kd` 后点击 `Apply RAM`，参数只写入控制器 RAM，重启后恢复固件默认值。
4. 勾选 `Live O`，上位机会按间隔自动发送 `O` 并刷新曲线。
5. 点击 `A Test` 可执行一次指定角度的转向测试；必要时点击 `Stop`。

曲线页含义：

- `Angle`：角度误差、实际角度、目标角度、转向输出。
- `Speed`：左右轮目标速度与编码器实际速度。
- `PWM`：左右轮 PWM 输出。

面板解析的固件文本行：

- `PID ...`：当前 PID 参数
- `ODOM ...`：角度、左右轮实际速度等响应数据
- `CTRLDBG ...`：当前控制误差、目标速度和 PWM
- `LCTRL ...`：最近一次相对位移结束前的控制快照
- `TCTRL ...`：最近一次原地转向有输出时的控制快照

CLI 用法：

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
