# SLAM 转向后地图旋转问题任务记录

## 状态

- 日期：2026-05-14
- 状态：`desktop_verified`
- 任务类型：SLAM 建图调优准备 / 文档状态整理
- 当前分支：`rollback/before-deadzone-debug`

## 背景

控制三环已经进入可用于迷宫建图和导航联调的基线状态。当前不再继续追求小于 `0.025 m` 的单段位置误差，优先转入 SLAM 建图质量调优。

用户提供的当前地图截图见：

- `docs/archive/references/problem_of_slam.png`

主要现象：

- 建图能形成可读轮廓。
- 机器人转向后，地图结构也出现明显旋转/重影。
- 用户判断“像是没有匹配上”，这与当前代码链路中的风险一致。

## 当前代码状态

当前链路：

`LiDAR scan -> LocalizationTask ICP -> corrected_pose -> MappingTask occupancy update`

关键实现事实：

- `MappingTask` 使用 `scan_msg.corrected_pose` 投影 LiDAR 点。
- `LocalizationTask` 使用上一帧参考扫描做轻量 ICP，输出 `delta_x / delta_y / delta_theta`。
- ICP 接受门限当前包括：
  - 最少 inliers：`10`
  - 最大 fitness：`0.08 m`
  - 最大单次 XY 修正：`0.12 m`
  - 最大单次角度修正：`12 deg`
- 建图 gate 已经会跳过：
  - 相对移动正在 turning 的扫描
  - turning 结束后约 `200 ms` 的 settle 窗口
  - 单帧 odom 位移超过 `0.035 m`
  - 单帧 odom 角度超过 `4 deg`
  - usable points 少于 `24`

这些保护能降低转向中污染地图的概率，但不能保证“转向后第一批可写入扫描已经和旧地图姿态匹配”。

## 初步诊断

从截图和代码链路看，最可疑的问题不是 PID，而是 SLAM 姿态一致性：

1. 转向后 ICP 没有可靠锁住 yaw，`corrected_pose.theta_deg` 仍主要跟随 odometry。
2. `MappingTask` 在 ICP rejected 或 odometry-only 时仍可能继续写图，只要 mapping gate 认为 scan 质量可接受。
3. 参考扫描是连续更新的轻量局部参考，不是全局 map matching；一旦转向后参考关系弱，可能会把新的局部扫描以错误姿态写入地图。
4. 只有 `G/P/O/X` 的现有输出可以观察最终状态，但缺少“转向前后 yaw 修正是否成功”的连续记录。
5. 如果 LiDAR 角度约定、安装方向或 `pose->theta + scan_angle` 的符号存在系统偏差，也会在转向后表现为整片结构旋转。

## 当前判断

**在进入已知目标导航或完整探索前，应先把 SLAM 转向后的姿态一致性作为 Priority 1。**

现在不建议立刻扩大导航状态机。因为导航依赖地图和 `EST`，如果转向后地图会旋转/重影，路径规划会基于不稳定地图做决策。

## 本轮固件改动

本轮已把最可疑的污染路径收紧：

- 新增 `SL` 串口命令，输出 SLAM 专用诊断：
  - ICP mode / accepted / rejected / odom-only 计数
  - reference/current points
  - inliers / `fit_mm`
  - `pred/corr` 位姿
  - ICP `delta_x / delta_y / delta_theta`
  - mapping gate reason
  - turning / recovery 状态
  - map written endpoints 和 skip 计数
- `G` 的 `MAP loc` 输出补充 ICP `delta_theta`。
- `G` 的 `MAP gate` 输出补充 recovery 状态和 recovery skip 计数。
- 新增 `MAPPING_SKIP_REASON_RECOVERY`。
- 相对转向结束后进入 turn recovery：
  - 转向结束后先等待约 `800 ms`。
  - recovery 期间，如果没有可靠 ICP accepted，不允许写图。
  - recovery 期间，不更新下一帧 ICP 的 reference scan。
  - 只有满足 `ICP accepted + inliers >= 18 + fit_mm <= 60 mm + |delta_theta| <= 10 deg` 后，才退出 recovery 并恢复写图/参考更新。

这个改动的目的不是证明 SLAM 已经修好，而是先阻断“转向后坏扫描污染地图和后续参考帧”的路径，同时让后续 AI agent 能根据日志判断下一步。

## 推荐调试方向

第一步先拿样本，不急于改算法：

- 记录转向前、转向中、转向后的 `P/G/O/SL/X4`。
- 重点看：
  - `LOC mode`
  - `accept/reject/odom`
  - `inliers`
  - `fit_mm`
  - `MAP gate`
  - `SLAM gate`
  - ICP `delta=(dx,dy,dth)`
  - `dth`
  - `pose=(x,y,theta)`
  - `ODOM th`
  - `EST th`
  - `POSE pred/corr`

第二步再按数据决定代码改动：

- 如果转向后 `LOC reject` 或 `odom-only` 仍在写图：说明 recovery gate 仍有缺口，应继续收紧恢复条件。
- 如果 `LOC accepted` 但 `corr.theta` 仍明显错误：加强 yaw 匹配诊断，记录 ICP `delta_theta`，必要时加入小角度搜索或更严格的角度接受门限。
- 如果 `dth` 很小但图仍旋转：检查 LiDAR 角度方向、安装零位、世界坐标投影公式。
- 如果 `MAP gate` 很少跳过转向后扫描：延长 settle 窗口或增加“turn recovery”状态，而不是只依赖 200 ms 固定延迟。

## 近期不做

- 不把 LiDAR 修正直接接入 PID。
- 不继续调 PID 追求更小单段误差。
- 不做 frontier exploration、出口检测、return-to-start。
- 不上重型 ICP / graph SLAM，除非轻量方案被实测证明不够。
