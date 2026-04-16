# SLAM / ICP 开发进度状态

## 1. 当前结论

截至目前，项目已经从“只有 RTOS 骨架和传感器管线”推进到了：

- 具备稳定的 `LiDAR + odometry + OGM` 最小建图闭环
- 具备基本可用的运行时调试接口
- 已修复建图核心链路中的关键死锁样表现问题

更准确地说，当前状态已经完成了原计划中的：

- `Phase 0`：基本完成
- `Phase 1`：基本完成
- `Phase 2`：已完成一个可运行版本，并做过一轮稳定性修复

尚未进入真正的：

- `Phase 3`：ICP 位姿修正
- `Phase 4`：自主探索、路径规划、出口搜索、回程
- `Phase 5`：benchmark 时间与鲁棒性优化


## 2. 现在进度到哪里了

如果按“开发阶段”来判断：

- 已经完成从工程骨架到最小建图闭环的阶段
- 当前进度大致位于“Phase 2 完成，准备进入 Phase 3”的位置

如果按“课程最终目标”来判断：

- 已经具备最终系统的一部分基础能力
- 但距离最终 benchmark 目标仍然有明显差距

当前最准确的状态描述是：

**系统已经能稳定接收雷达、做基础预处理、使用 odometry 进行 occupancy grid 建图，并能通过调试命令观察地图状态；但尚未具备 scan matching、路径规划、自主探索和回程能力。**


## 3. 已完成内容

### 3.1 Phase 0：基线冻结与接口清理

已完成内容：

- 引入统一的 SLAM 公共类型定义
- 补齐 `pose / grid / waypoint` 等跨模块共享结构
- 给 LiDAR scan 增加 `pose_snapshot`
- 引入并统一关键 mutex 和任务间队列
- 让工程维持可编译、可回归的状态

对应实现：

- [slam_types.h](<D:\Project\Microprocessor_Project\Core\Inc\slam_types.h>)
- [freertos_app.h](<D:\Project\Microprocessor_Project\Core\Inc\freertos_app.h>)
- [encoder.c](<D:\Project\Microprocessor_Project\Core\Src\encoder.c>)
- [freertos.c](<D:\Project\Microprocessor_Project\Core\Src\freertos.c>)

结论：

- 这一阶段的目标已经基本达成


### 3.2 Phase 1：里程计与 Scan 对齐

已完成内容：

- 每帧 LiDAR scan 关联 `pose_snapshot`
- 增加 scan sequence / point count / quality 统计
- 实现基础 scan 预处理：
  - 距离过滤
  - 质量过滤
  - usable point 统计
- 通过 `P` 命令可以输出 runtime 和 scan 质量信息

对应实现：

- [scan_preprocess.h](<D:\Project\Microprocessor_Project\Core\Inc\scan_preprocess.h>)
- [scan_preprocess.c](<D:\Project\Microprocessor_Project\Core\Src\scan_preprocess.c>)
- [encoder.c](<D:\Project\Microprocessor_Project\Core\Src\encoder.c>)
- [hc04.c](<D:\Project\Microprocessor_Project\Core\Src\hc04.c>)

当前评价：

- `pose + scan` 已经能形成稳定输入
- 但仍属于“建图可用”，不是“高精定位可用”


### 3.3 Phase 2：最小 OGM 建图闭环

已完成内容：

- 实现固定尺寸 OGM
- 实现 `world -> cell`
- 实现射线更新：
  - free cell 更新
  - occupied endpoint 更新
- 接入独立 `MappingTask`
- 支持地图状态摘要和 ASCII 导图
- 修复 `OccupancyGrid_TraceRay()` 中的 Bresenham 无限循环问题

对应实现：

- [occupancy_grid.h](<D:\Project\Microprocessor_Project\Core\Inc\occupancy_grid.h>)
- [occupancy_grid.c](<D:\Project\Microprocessor_Project\Core\Src\occupancy_grid.c>)
- [mapping_task.h](<D:\Project\Microprocessor_Project\Core\Inc\mapping_task.h>)
- [mapping_task.c](<D:\Project\Microprocessor_Project\Core\Src\mapping_task.c>)
- [hc04.c](<D:\Project\Microprocessor_Project\Core\Src\hc04.c>)

当前评价：

- 已经完成“能运行的最小建图闭环”
- 已经不是纯数据采集阶段
- 但地图仍然是 `odometry-only`，精度和鲁棒性还不足以支持最终回程任务


## 4. 当前能做到什么

当前系统已经能做到：

- 启动 LiDAR 建图模式
- 持续接收 LiDAR 数据并做基础 scan 解析
- 为每帧 scan 记录对应 odometry pose 快照
- 用 odometry + LiDAR 更新 occupancy grid
- 输出运行时统计：
  - `ctrl`
  - `cmd`
  - `dma`
  - `dma_drop`
  - `scan`
  - `raw / usable`
- 输出地图状态摘要
- 在停止 LiDAR 后导出 ASCII 地图

换句话说：

- **已经能“建图”**
- **还不能“定位修正 + 自主导航 + 回程”**


## 5. 当前还不能做到什么

当前系统还不具备以下关键能力：

### 5.1 ICP / Scan Matching

目前还没有：

- `ICP_Match()` 之类的 scan matching 模块
- 局部地图匹配
- 上一帧点集匹配
- 基于 fitness 的位姿修正

因此当前位姿仍主要依赖：

- 编码器里程计

这意味着：

- 地图仍会受到累计漂移影响
- 回到已知区域时不能自动收敛位姿误差


### 5.2 Localization 闭环

目前还没有独立的：

- `LocalizationTask`
- “odometry 预测 + ICP 修正”融合链路

所以当前系统只有：

- 建图

而没有真正意义上的：

- 稳定定位


### 5.3 Path Planning

当前还没有：

- A*
- 路径点输出
- 基于 OGM 的导航目标求解

也就是说当前没有任何真正的全局规划器。


### 5.4 Exploration / Exit 搜索

当前还没有：

- frontier detection
- 未知区域扩展策略
- 出口判定逻辑
- 探索目标生成逻辑

因此当前系统还不会“主动搜索出口”。


### 5.5 Return-to-Start

当前还没有：

- 起点记忆到规划层的闭环路径
- 回程任务调度
- Exit -> Start 模式切换

因此 benchmark 的第二部分尚未启动开发。


## 6. 距离最终目标还差什么

距离课程最终目标，当前还差四大块：

### 6.1 差一个可用的轻量 ICP

这是从“能建图”走向“地图不漂、可回程”的关键一步。

至少需要补上：

- scan 降采样接口
- 匹配目标集构建
- 轻量 ICP 迭代
- fitness / inlier / 修正幅度阈值
- mismatch 回退到 odometry


### 6.2 差一个独立定位输出链路

需要建立：

- `predicted_pose` from odometry
- `corrected_pose` from ICP
- 建图统一使用修正后位姿

这是地图精度和后续规划稳定性的前提。


### 6.3 差一个“能走起来”的规划与探索闭环

至少需要：

- A* 路径规划
- 已知目标点导航
- frontier exploration
- 出口发现逻辑

只有把这一层补上，系统才从“会建图”变成“会找路”。


### 6.4 差 benchmark 级状态机

最终还需要：

- Start -> Explore -> Exit
- Exit detected -> Return-to-Start
- 超时 / 失败保护
- 速度与实时性调参

这部分目前几乎还没开始。


## 7. 当前阶段的主要风险

虽然现在基础建图已经能跑，但仍有以下风险：

- 当前是 `odometry-only` 建图，累计漂移仍明显存在
- LiDAR 数据链路做过减载和保护，但还没围绕最终 benchmark 做完整性能收敛
- ASCII 导图与运行时命令更多是调试用途，不是最终展示接口
- 地图质量尚未通过系统性的场地实测验证


## 8. 建议如何定义当前里程碑

建议把当前里程碑定义为：

**Milestone A：稳定的最小建图闭环完成**

里程碑含义：

- 传感器链路打通
- 里程计与 scan 对齐完成
- OGM 更新完成
- 关键卡死问题已修复
- 系统从“骨架阶段”进入“SLAM 核心开发阶段”

这不是最终目标，但已经是一个明确、有效的阶段性交付。


## 9. 下一步建议

最合理的下一步不是继续堆调试命令，而是进入：

### Priority 1

- 开始 `Phase 3`
- 设计并实现轻量 ICP

### Priority 2

- 引入独立 `LocalizationTask`
- 让建图使用修正后位姿

### Priority 3

- 做最小 A*
- 先实现“已知目标点导航”

### Priority 4

- 再接 frontier exploration 和回程状态机


## 10. 一句话总结

当前项目进度已经从“RTOS + LiDAR 骨架”推进到“稳定的 odometry-only 最小建图闭环”，下一阶段的真正分水岭是：**把 ICP 和定位闭环接起来。**
