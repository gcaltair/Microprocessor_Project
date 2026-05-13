# PID 调参上位机硬件测试计划

## Test Metadata

- Date: 2026-05-13
- Author: AI agent
- Feature: PID RAM 调参、上位机可视化和 CLI
- Firmware build: 待测试前记录 commit/hash
- Hardware version: 待填写

## Objective

确认 `PK...` 命令能读取和临时设置 PID 参数，上位机 PID 面板能显示当前参数并绘制 `CTRLDBG / LCTRL / TCTRL` 诊断曲线，CLI 能输出后续 AI agent 可消费的 JSON/JSONL 数据。

## Preconditions

- Robot power state: 车体架空或放在开阔地面，旁边有人值守。
- Required sensors/modules: 串口/蓝牙通信正常，电机、编码器、电源和地线连接稳定。
- Map or environment setup: 不需要地图；转向测试需要周围留出安全空间。
- Safety conditions: 急停或断电手段可用，第一次设置 PID 后只做短动作。

## Command Sequence

1. 烧录当前固件并连接上位机。
2. 串口发送 `PK`，确认四个环均返回参数。
3. 串口发送 `PKA`，确认只返回角度环参数。
4. 串口发送 `PKA,0.05,0,0.001`，再发送 `PKA`，确认 RAM 参数更新。
5. 串口发送 `R0`。
6. 串口发送 `A90`，动作结束后发送 `O`。
7. 观察 `CTRLDBG / TCTRL` 输出中的 `ang_err / turn / l_sp / r_sp / pwm`。
8. 打开 host app，连接同一端口，在 PID 面板点击 `Read All`。
9. 在 PID 面板修改角度环参数并点击 `Apply RAM`，再点击 `Read Loop` 验证回读。
10. 运行 CLI：`python -m host_app.tools.pid_tuner_cli --port COMx show --json`。
11. 运行 CLI：`python -m host_app.tools.pid_tuner_cli --port COMx send R0 A90 O --duration 3 --jsonl logs/pid_angle.jsonl`。
12. 重启控制器，再发送 `PKA`，确认参数恢复编译默认值。

## Expected Behavior

- Serial output:
  - `PK` 输出 `PID loop=A/L/R/P ... kp=... ki=... kd=...`
  - `PK<loop>,kp,ki,kd` 输出 `PID set loop=...`
  - `O` 包含 `CTRLDBG`，原地转向后包含 `TCTRL`
- Physical robot behavior:
  - 机器人动作不应突然全速冲出或持续震荡。
  - 角度环参数变化应能反映在转向响应和 `TCTRL turn/l_sp/r_sp` 上。
- Timing or tolerance expectations:
  - CLI `show` 应在超时时间内返回 PID 数据。
  - JSONL 文件应包含结构化的 `pid_tuning` 或 `control_debug` 记录。

## Failure Signs

- Unexpected serial output: `Invalid PID format`、无 `PID loop=...`、ACK 失败。
- Unsafe movement: 设置后电机突然全速、无法停止、持续原地剧烈抖动。
- Incorrect state transition: 重启后 PID 参数仍保持 RAM 设置值，说明可能发生了非预期持久化。
- Timeout or no response: host app 或 CLI 能打开端口但收不到串口输出。

## Safe Stop / Recovery

- Stop command: `S`
- Reset command: `R0`
- Required operator action: 必要时立即断电，恢复默认 PID 后再继续。

## Result

- Status: not run
- Tester:
- Notes:
