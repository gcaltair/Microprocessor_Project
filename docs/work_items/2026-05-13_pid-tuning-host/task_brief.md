# PID 调参上位机任务简报

## Task Metadata

- Date: 2026-05-13
- Owner: AI agent
- Task type: mixed
- Related phase: Phase 3B 控制稳定化
- Related issue or request: 制作可拓展的 PID 调参上位机，并为后续 AI agent 调参保留接口

## Goal

在不重写主控制逻辑的前提下，补充 RAM-only PID 参数读写命令、上位机可视化面板和 CLI/JSONL 数据入口，用于人工调参、复盘 `CTRLDBG / LCTRL / TCTRL`，并为后续 AI agent 调参准备稳定接口。

## Scope

- In scope:
  - 固件新增 PID 参数读取和临时设置命令
  - 上位机新增 PID 面板、串口 CLI 和可解析的诊断数据模型
  - 测试命令生成、文本解析和状态仓库更新
  - 文档记录命令、安全边界和硬件验证步骤
- Out of scope:
  - AI 自动调参闭环
  - Flash 持久化 PID 参数
  - 改写 PID 主控制算法
  - 自动替代急停或人工安全观察

## Constraints

- Architecture constraints: PID 仍由固件原控制链执行，上位机只发送已有串口命令。
- Memory or timing constraints: 固件只增加小型命令解析和读写函数，不新增动态分配或后台任务。
- Safe edit boundaries: 避免改动 CubeMX 生成外设初始化和导航/建图主链路。

## Expected Files

- Files to update:
  - `Core/Src/pid.c`
  - `Core/Src/pid.h`
  - `Core/Src/hc04.c`
  - `host_app/host_app/protocol/pid_tuning.py`
  - `host_app/host_app/tools/pid_tuner_cli.py`
  - `host_app/host_app/ui/pid_tuning_panel.py`
  - `host_app/README.md`
  - `docs/freertos_further_development_plan.md`
  - `docs/development_plan/slam_icp_progress_status.md`
- Files to avoid:
  - `Drivers/`
  - `Middlewares/`
  - generated host app build outputs

## Verification Plan

- Desktop verification:
  - `cmake --build cmake-build-debug`
  - `python -m pytest`
- Hardware verification:
  - 见 `hardware_test_plan.md`
- Required logs or outputs:
  - `PK`/`PKA` 输出
  - `CTRLDBG / LCTRL / TCTRL` 输出
  - CLI JSON/JSONL 样例

## Completion Criteria

- [x] Implementation complete
- [x] Documents updated
- [x] Desktop verification done or explicitly blocked
- [x] Hardware test plan prepared if needed
- [x] Residual risks recorded
