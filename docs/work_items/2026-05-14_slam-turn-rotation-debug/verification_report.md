# SLAM 转向后地图旋转问题验证记录

## 本轮验证范围

本轮完成第一版固件保护和诊断增强。

2026-05-14 硬件调试补充：

- 通过 `COM15 961200` 连接，`SL` 命令可响应。
- 静止建图、`A90`、`A-90` 前半段日志已保存到 `hardware_runs/20260514_202957_slam_turn_debug_serial.log`。
- 用户观察到心跳灯长亮，说明 safety task 没有继续按 100 ms 翻转，需按调度/阻塞问题处理。
- 首轮日志确认 `A90 / A-90` 后 `MAP gate=active` 且 `skip turn=0 settle=0 recovery=0`，说明旧 gate 没有覆盖 `Axx` 原地转向。

## 已检查内容

- `Core/Src/localization_task.c`
- `Core/Src/mapping_task.c`
- `Core/Src/hc04.c`
- `Core/Inc/localization_task.h`
- `Core/Inc/mapping_task.h`
- `Core/Inc/freertos_app.h`
- `docs/development_plan/slam_icp_progress_status.md`
- `docs/development_plan/README.md`
- `docs/freertos_further_development_plan.md`

## 静态诊断结论

当前实现已经有基础保护：

- 转向期间暂停建图。
- 转向结束后 `200 ms` settle 窗口暂停建图。
- 单帧 odom 位移/角度过大时跳过建图。
- scan usable point 太少时跳过建图。

本轮修复/增强：

- 新增 `SL` 诊断命令。
- `G` 输出补充 ICP delta 和 recovery skip 计数。
- 转向后进入 turn recovery。
- recovery 期间，必须等待可靠 ICP accepted 才恢复写图。
- recovery/turning/settle/quality gate 阻止写图时，不更新 ICP reference scan。
- `Axx` 原地转向现在使用 `ControlLogic_ShouldPauseMappingForTurn()` 识别，不再只依赖 `g_relative_move_state == RELATIVE_MOVE_TURNING`。
- 新增 `g_lastMappingTurnDetected`，确保 `Axx` 转向结束后也会进入 settle/recovery，而不是只对 `P{x,y}` 内部 turning 生效。
- `safetyTask` 优先级从 `BelowNormal` 提高到 `AboveNormal`，心跳更能反映系统是否仍被调度。
- 串口文本输出每次发送后在 RTOS 运行状态下 `osDelay(1)`，避免长串调试输出连续占用 defaultTask。

仍未完全证明的问题：

- ICP yaw 是否能在真实迷宫转向后可靠 accepted。
- `delta_theta` 方向是否与真实姿态变化一致。
- LiDAR 角度方向/安装零位是否正确。
- 心跳长亮的直接触发点尚未完全复现到源码级根因；当前修复先覆盖已确认的转向 gate 缺口和调试输出占用问题。

## 构建/测试结果

- 固件构建：通过。
  - 命令：`cmake --build cmake-build-debug`
  - 结果：`[100%] Built target FInal_fina`
  - RAM：`129936 B / 128 KB = 99.13%`
  - FLASH：`128392 B / 512 KB = 24.49%`
- Host app 测试：未运行。本轮没有改 host app 代码。

## 最高验证等级

`Level 1: desktop verification`

说明：当前 RAM 占用已经非常接近上限，但本阶段以 SLAM 调试和数据采样优先；RAM 收口放到 SLAM 行为稳定后处理。

## 待硬件验证

执行同目录下 `hardware_test_plan.md`，确认：

- 转向后第一批 active scan 是否在 ICP accepted 前写入地图。
- `LOC mode / inliers / fit_mm` 与地图旋转的对应关系。
- `ODOM th / EST th / POSE pred/corr` 是否出现 yaw 不一致。
