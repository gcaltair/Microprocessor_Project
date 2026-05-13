# 终点与转向抖动调试任务简报

## 任务元数据

- 日期：2026-05-13
- 负责人：Codex
- 任务类型：firmware + documentation
- 相关阶段：Phase 3B 控制稳定性 / Phase 4A 基础导航稳定性
- 相关请求：`P1,0` 等相对位移快到终点时抖动严重，转角时也存在抖动

## 目标

降低相对位移终点附近和原地/相对转向时的闭环抖动，并补齐能解释抖动来源的控制调试输出。

## 范围

- 范围内：
  - 相对位移控制终点附近的速度处理
  - 移动中的角度修正限幅
  - 角度控制死区迟滞
  - `O` 调试输出中的控制诊断信息
  - 硬件复测步骤和验证记录
- 范围外：
  - 不改 LiDAR 定位融合策略
  - 不改导航路径规划算法
  - 不做大规模 PID 重构
  - 不引入动态内存

## 约束

- 控制稳定优先于最终厘米级精度。
- 不把 LiDAR 修正直接送入 PID 主闭环。
- 继续保持 `odometry / control_pose / corrected_pose` 分层。
- 固件 RAM 已接近上限，本任务不新增大数组。

## 预期文件

- 更新：
  - `Core/Src/pid.c`
  - `Core/Src/system.h`
  - `Core/Src/hc04.c`
  - `docs/work_items/2026-05-13_control-jitter-debug/*`
  - `docs/development_plan/slam_icp_progress_status.md`
  - `docs/freertos_further_development_plan.md`
- 避免：
  - `Drivers/`
  - `Middlewares/`
  - 生成目录和打包产物

## 验证计划

- 桌面验证：
  - 构建固件，确认无编译错误。
- 硬件验证：
  - `R0 -> O -> P1,0 -> 接近终点前/后 O -> D...`
  - `R0 -> A90 -> O`
  - `R0 -> A-90 -> O`
  - 观察终点附近是否仍有明显左右轮反向拉扯。

## 完成标准

- [x] 实现完成
- [x] 文档更新
- [x] 桌面构建完成
- [x] 硬件测试计划已准备
- [x] 残余风险已记录
