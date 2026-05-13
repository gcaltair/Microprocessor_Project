# 终点与转向抖动验证报告

## 验证摘要

- 日期：2026-05-13
- 功能：相对位移终点与转向抖动控制稳定化
- 负责人：Codex

## 验证层级

- Level 0 静态审查：yes
- Level 1 桌面验证：yes
- Level 2 操作者验证准备：yes
- Level 3 硬件验证：no

## 桌面证据

- 已完成静态检查：
  - 终点附近只在 `distance_error > 0.15 m` 时保留 `MIN_BASE_SPEED`。
  - 最后 `0.15 m` 增加终端速度上限，从 `0.08 m/s` 线性收缩到约 `0.015 m/s`，避免当前位置环在到达阈值附近仍给出偏硬速度。
  - 倒车相对移动补齐 `-MAX_BASE_SPEED` 对称限幅，避免 `P-1,0` 在负向位置环输出过大时放大终点抖动。
  - 移动中的 `turn_output` 额外受 `abs(base_speed) * 0.80` 限制，降低一侧轮子反转风险。
  - 角度控制从单一 `1 deg` 死区改为 `1 deg / 2 deg` 迟滞，减少边界反复启停。
  - `O` 输出增加 `CTRLDBG`，显示 `pos_err / ang_err / turn / l_sp / r_sp / pwm`。
  - `O` 输出增加 `LCTRL`，保留最近一次相对位移动作结束前最后一轮驱动控制输出，避免动作结束后只看到停止态。
  - `O` 输出增加 `TCTRL`，保留最近一次原地转向有输出时的控制状态，避免转角结束后只看到静止态。
- 构建命令：
  - `cmake --build cmake-build-debug`
- 结果：
  - 通过
  - RAM：`129680 B / 128 KB = 98.94%`
  - FLASH：`119064 B / 512 KB = 22.71%`
- 覆盖限制：
  - 桌面构建只能验证编译，不代表实车抖动已解决。

## 硬件证据

- 测试计划：`docs/work_items/2026-05-13_control-jitter-debug/hardware_test_plan.md`
- 手动测试记录：`docs/work_items/2026-05-13_control-jitter-debug/manual_test_record.md`
- 结果：未运行，等待实车复测。

## 剩余缺口

- 需要确认新版在 `P1,0` 终点附近是否确实减少抖动。
- 需要确认 `A90 / A-90` 转角目标附近是否仍反复摆动。
- 如仍抖动，需要继续区分软件输出震荡、IMU/编码器噪声、机械摩擦和电机死区问题。

## 发布 / 合并建议

- Ready with hardware follow-up
