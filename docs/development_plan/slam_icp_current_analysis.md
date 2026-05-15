# SLAM / ICP 当前问题分析

更新日期：`2026-05-15`

## 1. 当前判断

当前 SLAM 的主要阻塞已经从“转向后整张图跟着车体旋转”下降为“轻微重影、墙体变厚和局部虚假障碍风险”。

已确认的关键修复：

- LiDAR scan 投影角度应使用 `pose - scan_angle`，当前固件默认已经切换为该方向。
- 转向期间和转向恢复期会暂停写图，避免把未重新匹配的 scan 写入 OGM。
- 转向恢复期暂停更新 ICP reference scan，避免错误姿态污染下一次匹配参考。
- `SL` / `P` / `G` / `X...` 已能观察 ICP delta、inliers、fitness、mapping gate 和写图状态。

用户在上位机复测后反馈“已经没问题了”，因此当前不应继续围绕角度符号做大改。下一阶段重点应切到：确认地图质量是否足够支撑 Phase 4A 已知目标导航，并在进入 frontier exploration 前降低轻微重影对路径规划的影响。

## 2. 当前相关代码位置

- LiDAR scan 投影与角度符号：`Core/Src/scan_preprocess.c`
- ICP 采样、匹配、验收、turn recovery：`Core/Src/localization_task.c`
- OGM 写图、ray tracing、ASCII map、路径规划：`Core/Src/mapping_task.c`
- 已知目标格点导航与路径执行：`Core/Src/navigation_task.c`
- 串口调试命令入口：`Core/Src/hc04.c`

## 3. 已解决的问题

### 3.1 转向后地图大幅旋转

历史问题表现为：`A90` / `A-90` 后，地图坐标系看起来随车体方向旋转，或新 scan 与旧墙面严重错位。

当前判断：

- 根因之一是 LiDAR scan 角度约定方向错误。
- A/B 测试显示 `pose - scan_angle` 明显优于 `pose + scan_angle`。
- 该问题已经通过 `2fcf4e3 firmware: correct slam scan angle sign` 收口。

### 3.2 转向恢复期写图污染

历史风险是：转向刚结束时 ICP 还没有重新锁定，MappingTask 已经继续写图。

当前处理：

- 转向与 settle/recovery 期间 gate 会阻止写图。
- recovery 需要看到可接受的 ICP 质量后才恢复 active。
- 暂停写图期间不更新 reference scan，降低错误参考链式污染。

## 4. 剩余问题

### 4.1 轻微地图重影

轻微重影本身不一定阻止导航。判断标准不是“图像是否好看”，而是：

- 是否把通道堵成不可通行。
- 是否把墙体膨胀到影响 `path inflation radius = 1` 后的可行路径。
- 是否出现孤立虚假障碍，导致路径频繁绕行或规划失败。
- 是否在同一路径多次扫描后持续变厚，而不是被后续观测稳定下来。

如果重影只在 `1 ~ 2` 个 cell 内，且不封闭通道，对迷宫任务通常可以接受。若虚假墙体进入通道中心或形成闭合障碍，会直接影响 Start -> Exit 与 Exit -> Start 导航。

### 4.2 ICP 参考仍偏局部

当前 ICP 更接近“scan-to-last-reference”的轻量局部匹配：

- 采样点上限为 `64`。
- reference scan 是小规模最近参考，不是全局地图或多关键帧集合。
- 没有 loop closure。
- 没有全局重定位。

这适合 STM32 上的实时轻量修正，但在长走廊、平行墙、转角后几何信息不足时，容易出现局部最优或低可观测性。

### 4.3 验收阈值较固定

当前 ICP 使用固定的 match distance、fitness、inlier 数和 correction 限幅。固定阈值的问题是：

- 空旷区、单墙、走廊和角落的几何约束强度不同。
- 同样的 fitness 在低约束场景里不一定可信。
- 缺少对“退化场景”的显式判断，例如大部分点只来自一面墙。

因此后续应增加几何分布质量指标，而不是简单放宽阈值。

### 4.4 OGM 对路径规划还不够友好

当前 OGM 写图已经可用，但导航更关心拓扑连通性：

- endpoint 噪声可能让墙体变厚。
- 少量虚假 occupied cell 可能配合 inflation 关闭狭窄通道。
- 地图没有专门的导航层清理，例如孤立障碍抑制、短期 hit 确认、局部开闭运算。

后续不一定要把 SLAM 做得更“漂亮”，但需要让规划层看到稳定、连通、保守的通行空间。

### 4.5 Odometry 和 yaw 仍会影响建图

当前控制层已经能支持建图/导航联调，但 odometry 仍不是高精度真值。SLAM 在以下情况下仍会受影响：

- 转向后 yaw 残差过大。
- 轮胎打滑或地面摩擦变化。
- LiDAR scan 与 pose snapshot 时间不同步。
- 低速运动时编码器量化导致速度估计抖动。

这些问题应通过诊断和轻量融合逐步处理，不建议现在直接上重型 EKF 或图优化。

## 5. 是否需要 Kalman / IMU 融合

当前不建议立刻做完整 EKF / Kalman 位姿融合。

原因：

- 目前最关键的 SLAM 问题已经被角度符号和写图 gate 明显改善，继续做 EKF 不一定解决剩余轻微重影。
- 现有架构明确要求 `odometry`、`control_pose`、`corrected_pose / EST` 分离，不能把低频 LiDAR 修正直接喂进 PID。
- RAM 已经紧张，完整 EKF、地图匹配和导航缓存同时增长会增加集成风险。
- IMU yaw 如果未完成偏置、零漂和磁干扰处理，直接融合可能把错误姿态稳定地注入系统。

推荐路线：

1. 先继续使用当前 odometry + ICP correction 作为建图与导航输入。
2. 如果转向后 yaw 收敛仍是瓶颈，增加轻量 yaw 互补滤波或 bias 估计，只用于改善预测姿态。
3. 保持控制 PID 使用稳定的 odometry 主链路，不让 LiDAR / ICP correction 直接进入底层闭环。
4. 只有当导航实测证明 yaw 漂移是主要失败原因时，再设计有状态的 EKF。

## 6. 下一步改进顺序

### P0：建立导航可用性的判据

先用地图是否支持导航来评价 SLAM，而不是只看截图：

- 同一场景多次 `A90` / `A-90` 后，墙体不应持续扩散。
- `X4` 中通道不应被虚假 occupied cell 封闭。
- `Jx,y` 规划结果稳定，不应因轻微重影反复失败。
- `SL` 中 recovery 不应长期停留，`accept / reject` 比例应可解释。

### P1：做导航友好的地图后处理

优先级高于重写 ICP：

- 增加路径规划层使用的障碍置信度策略。
- 抑制孤立 occupied cell 对规划的影响。
- 对新 occupied endpoint 增加 hit 确认或局部一致性判断。
- 在 ICP quality 弱、turn recovery 未完成时继续禁止写图。

### P2：增强轻量 ICP 鲁棒性

在 RAM 可控前提下逐步增强：

- 保留少量关键帧 reference，而不是只依赖单个最近 reference。
- 转向后允许小范围 yaw 多假设搜索，选择 fitness 和 inlier 更稳定的结果。
- 增加点云几何分布检查，识别单墙或长走廊退化场景。
- 将 ICP 验收阈值与场景质量关联，而不是固定放宽。

### P3：再考虑 IMU yaw 辅助

如果 P0/P1/P2 后仍主要失败在转向后 yaw 初值：

- 增加 IMU yaw bias 估计。
- 做轻量互补滤波，限制单次 yaw correction。
- 只把结果用于 `predicted_pose`，继续保护底层控制闭环。

## 7. 建议测试序列

每次测试建议记录 `P / G / SL / X4`，必要时截图上位机地图。

```text
R0
M
等待 3-5 s
P
G
SL
X4
A90
等待 3-5 s
P
G
SL
X4
A-90
等待 3-5 s
P
G
SL
X4
```

导航验证在地图稳定后执行：

```text
Jx,y
J
P
G
X4
```

重点看：

- `LOC accept/reject`
- `fit_mm`
- `inliers`
- `SLAM gate reason`
- `map written`
- 规划是否能得到稳定路径
- 通道是否被 ghost wall 或 inflated obstacle 关闭

## 8. 当前最推荐的实现目标

下一阶段不应直接做完整 frontier exploration。最推荐先做 Phase 4A 导航稳定化：

- `Jx,y` 路径可视化。
- 导航失败原因输出。
- 已知目标路径重复测试。
- 规划层对轻微地图噪声的容错。
- 再决定是否进入 frontier、Exit 检测和 Return-to-Start。

这条路线更符合课程要求：最终要完成 Start -> Exit 和 Exit -> Start，SLAM 的目标是服务导航，而不是单独追求无重影地图。
