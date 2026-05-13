# Host App

`host_app` 是面向 Windows 的 Python + PySide6 上位机，用于 STM32 小车调试。

它主要用于：

- 通过串口连接小车。
- 在文本兼容命令和结构化 telemetry 工作流之间切换。
- 发送手动控制、LiDAR、导航和 PID 调参命令。
- 可视化运行状态、占据栅格地图、路径、扫描点、PID 诊断和日志。
- 写入 RAM-only PID 参数，并记录 PID 诊断数据供后续 AI agent 分析。
- 记录原始串口会话并回放保存的日志。

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
- `Position`：位置误差、相对移动进度、剩余距离、基础速度。

面板解析的固件文本行：

- `PID ...`：当前 PID 参数。
- `ODOM ...`：角度、左右轮实际速度等响应数据。
- `MOVE ...`：相对移动目标、进度和剩余距离。
- `CTRLDBG ...`：当前控制误差、目标速度和 PWM。
- `LCTRL ...`：最近一次相对位移结束前的控制快照。
- `TCTRL ...`：最近一次原地转向有输出时的控制快照。

速度环隔离测试使用固件命令 `WSleft,right`，单位是 `m/s`，会绕过位置环和角度环，直接给左右轮速度 PID 设置目标值。倒车低速测试示例：

```text
S
R0
WS-0.03,-0.03
O
S
```

CLI 用法：

```powershell
cd host_app
python -m host_app.tools.pid_tuner_cli list-ports
python -m host_app.tools.pid_tuner_cli --port COM7 show --json
python -m host_app.tools.pid_tuner_cli --port COM7 set angle 0.05 0 0.001 --json
python -m host_app.tools.pid_tuner_cli --port COM7 send R0 A90 O --duration 3 --jsonl logs/pid_angle.jsonl
python -m host_app.tools.pid_tuner_cli --port COM15 --baud 961200 send R0 WS-0.03,-0.03 O S --duration 5 --jsonl logs/speed_loop_isolation.jsonl
```

PID loop names:

- `angle` / `A`
- `left_speed` / `L`
- `right_speed` / `R`
- `position` / `P`

固件调参命令只写 RAM，控制器重启后恢复编译进固件的默认值。

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

- `host_app/app.py`：应用入口。
- `host_app/protocol/`：命令构造和 telemetry 解析。
- `host_app/services/`：控制器和会话状态管理。
- `host_app/transport/`：串口传输。
- `host_app/tools/pid_tuner_cli.py`：PID 调参 CLI，用于日志采集和后续 AI agent 集成。
- `host_app/ui/`：Qt UI 控件和视图。
- `tests/`：解析器和状态测试。

## Agent Notes

只改上位机时，也要阅读：

- `../AGENTS.md`
- `../docs/agent_workflow/README.md`
- `pyproject.toml`

如果任务改变上位机工作流、协议用法或测试命令，需要在同一任务中更新本文档。
