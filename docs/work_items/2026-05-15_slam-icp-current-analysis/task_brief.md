# Task Brief: SLAM / ICP 当前状态分析

日期：`2026-05-15`

## 目标

整理当前 SLAM / ICP 的真实状态，明确剩余问题、导航影响和下一阶段改进顺序，避免后续 agent 继续围绕已收口的角度符号问题重复试错。

## 背景

- 已通过 A/B 测试确认 LiDAR scan 投影应使用 `pose - scan_angle`。
- 用户在上位机复测后反馈地图转向问题已经基本可用。
- 当前仍存在轻微地图重影，需要判断它是否影响迷宫导航。
- 课程最终要求是基于 OGM 的自主探索、全局路径规划、局部避障，以及 Start -> Exit / Exit -> Start。

## 产出

- 新增 `docs/development_plan/slam_icp_current_analysis.md`
- 更新开发计划索引，明确下一阶段优先做 Phase 4A 导航稳定化
- 保留历史 SLAM turn debug 文档作为证据链，不让旧结论覆盖当前状态

## 非目标

- 不修改固件算法。
- 不重新调 PID。
- 不实现 frontier exploration。
- 不引入 EKF / Kalman 融合。
