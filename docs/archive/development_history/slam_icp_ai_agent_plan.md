# SLAM / ICP AI Agent 开发计划

## 1. 文档用途

这不是给人看的概念性计划，而是给后续 AI Agent 直接执行的任务说明书。

使用方式：

- 你给 AI Agent 下达任务时，要求它遵守本文件
- 一次只分配一个明确阶段或一个明确模块
- 每次任务结束必须按本文件的“交付格式”汇报
- 未完成当前阶段验收标准前，不进入下一阶段

## 2. 项目目标

根据课程 brief，系统最终必须实现：

- 基于轮编码器的里程计
- 2D Occupancy Grid Map 增量建图
- 轻量级 scan matching，推荐 `ICP`
- 基于 OGM 的自主探索
- `Start -> Exit`
- `Exit -> Start`
- 全部运行于 `STM32F446RE + FreeRTOS`

量化 benchmark 的核心是：

- `Start -> Exit <= 120s`
- `Exit -> Start <= 120s`

所以开发优先级必须固定为：

1. 先做出稳定闭环
2. 再提升定位和地图质量
3. 最后优化 benchmark 时间

## 3. 当前代码基线

AI Agent 必须先接受以下现状，不要假设项目从零开始。

当前已有内容：

- `FreeRTOS` 主干任务已存在
- `LiDAR DMA -> scan buffer -> queue` 已存在
- 蓝牙命令链路已存在
- 编码器测速和基础里程计已存在
- 工程当前可成功编译

当前缺口：

- 尚未接入真正的 `MappingTask`
- 尚未接入真正的 `LocalizationTask`
- 尚未接入真正的 `PlanningTask`
- 尚未接入真正的 `ExplorationTask`
- `LiDAR` 数据当前主要仍用于传输，不是用于地图闭环
- 仓库存在“历史构建产物与当前源码状态不完全一致”的风险

## 4. AI Agent 总规则

所有 AI Agent 都必须遵守以下规则。

### 4.1 通用执行规则

- 不要一次改太多模块
- 一次任务只解决一个闭环问题
- 优先做最小可用实现，不要先追求复杂算法
- 禁止在没有验收证据的前提下声称“已完成”
- 每次修改后必须重新编译
- 如果涉及行为变化，必须说明如何验证

### 4.2 架构规则

- 不要把新算法继续堆进 `freertos.c`
- 优先新增独立模块：
  - `slam_types.h`
  - `occupancy_grid.c/.h`
  - `scan_preprocess.c/.h`
  - `icp.c/.h`
  - `mapping_task.c/.h`
  - `localization_task.c/.h`
  - `planning.c/.h`
  - `exploration.c/.h`
- `FreeRTOS` 只负责调度和通信，不负责承载所有算法细节

### 4.3 实时性规则

- `ControlTask` 优先级不能被 SLAM 任务压制
- `ICP` 只能是轻量实现
- 不允许引入明显阻塞控制环的长计算
- 若算法无法在当前 MCU 预算内稳定运行，应先降采样、限迭代、降低触发频率

### 4.4 安全规则

- 不得破坏当前已有可编译状态
- 不得删除用户现有功能，除非本任务明确要求
- 不得引入未受控的动态内存依赖
- 不得把“失败匹配结果”直接写回全局 pose

## 5. AI Agent 交付格式

每次任务结束时，AI Agent 必须按以下格式汇报：

### 5.1 必填内容

- 本次任务目标
- 实际修改的文件列表
- 关键改动摘要
- 编译结果
- 验证方式
- 已知风险
- 下一步建议

### 5.2 输出模板

```text
任务目标:
- ...

修改文件:
- path/file1
- path/file2

关键改动:
- ...

编译结果:
- 成功 / 失败
- 使用的命令: ...

验证:
- 做了什么验证
- 结果是什么

风险:
- ...

建议下一步:
- ...
```

## 6. 阶段拆分

AI Agent 必须按阶段推进，不允许跳阶段。

## Phase 0: 基线冻结与接口清理

### 目标

让项目具备后续安全迭代的基础，不再出现“算法还没开始，工程先乱掉”的情况。

### 输入

- 当前仓库源码
- 课程 brief
- 现有 `FreeRTOS` 主干结构

### 任务

- 检查并整理 `pose`、`scan`、`map`、`path` 的公共结构定义
- 确认 `icp_map` 历史残留是否需要恢复、重建或彻底替换
- 统一共享状态访问方式
- 固定锁顺序
- 补齐必要的头文件边界和模块职责

### 建议输出文件

- `Core/Src/slam_types.h`
- 必要时更新：
  - `Core/Inc/freertos_app.h`
  - `Core/Src/system.h`
  - `Core/Src/freertos.c`

### 验收标准

- 工程可编译
- 公共数据结构稳定
- 没有明显职责混乱的跨模块全局访问
- 后续模块可以在不改动大面积旧代码的前提下接入

### 禁止事项

- 不要在这个阶段实现 ICP
- 不要在这个阶段实现 A*
- 不要在这个阶段重写整个控制框架

## Phase 1: 里程计与 Scan 对齐

### 目标

让 `pose + scan` 成为可信的 SLAM 输入。

### 任务

- 审核并修正 `Odometry_Update()` 的积分逻辑
- 明确姿态单位是角度还是弧度，并统一
- 每帧 LiDAR scan 必须带：
  - scan sequence
  - pose snapshot
  - point count
- 对 LiDAR scan 做基础预处理：
  - 去掉无效距离
  - 去掉极近极远点
  - 可选角度排序或降采样

### 建议新增文件

- `Core/Src/scan_preprocess.c`
- `Core/Src/scan_preprocess.h`

### 验收标准

- 小车直行时 `x/y/theta` 变化方向正确
- 小车原地转向时姿态变化方向正确
- 连续多帧 scan 点数稳定
- 每帧 scan 都能关联到对应 pose 快照

### 必做验证

- 编译通过
- 打印或导出至少一组 `pose + point_count + sequence`

## Phase 2: OGM 最小建图闭环

### 目标

在不依赖 ICP 的前提下，先用 odometry 建出可读地图。

### 任务

- 实现固定大小 OGM
- 实现世界坐标到网格坐标映射
- 实现射线更新：
  - 终点为 occupied
  - 沿线为 free
- 做好边界保护
- 提供可调试输出

### 推荐参数

- 地图尺寸：`96 x 96`
- 分辨率：`0.05m`
- 地图范围：约 `4.8m x 4.8m`

### 建议新增文件

- `Core/Src/occupancy_grid.c`
- `Core/Src/occupancy_grid.h`
- `Core/Src/mapping_task.c`
- `Core/Src/mapping_task.h`

### 验收标准

- 地图更新逻辑能运行
- 在简单环境下墙体轮廓可辨认
- 没有数组越界和地图污染

### 必做验证

- 编译通过
- 展示一份地图导出结果或调试输出

## Phase 3: 轻量 ICP 位姿修正

### 目标

让地图从“能出图”提升到“能稳定回程”。

### 任务

- 实现轻量 ICP
- 使用 odometry pose 作为初值
- 对 scan 做降采样
- 迭代次数受限
- 加入 fitness 或误差门限
- ICP 失败时回退到 odometry

### 建议算法边界

- 每帧点数控制在 `60-120`
- 迭代 `5-8` 次
- 单帧修正量超阈值则拒绝更新

### 建议新增文件

- `Core/Src/icp.c`
- `Core/Src/icp.h`
- `Core/Src/localization_task.c`
- `Core/Src/localization_task.h`

### 验收标准

- 地图重影减少
- 重复经过相同区域时漂移降低
- ICP 失败时不会污染 pose

### 必做验证

- 编译通过
- 至少给出一组“修正前 vs 修正后”的日志或数值

### 禁止事项

- 不要上 EKF
- 不要做全局回环
- 不要实现复杂图优化

## Phase 4: 全局规划与自主探索

### 目标

让小车能够根据地图行动，而不是只会建图。

### 任务

- 实现 A*
- 实现 frontier detection
- 从 frontier 选择探索目标
- 规划结果输出给控制层
- 支持出口发现后的目标切换

### 建议新增文件

- `Core/Src/planning.c`
- `Core/Src/planning.h`
- `Core/Src/exploration.c`
- `Core/Src/exploration.h`

### 验收标准

- 已知目标下可以规划出路径
- frontier 可以持续给出探索目标
- 能切换到回程目标

### 必做验证

- 编译通过
- 至少展示一次路径规划结果

## Phase 5: Benchmark 收敛

### 目标

从“功能可用”推进到“满足课程 benchmark”。

### 任务

- 统计任务周期、执行时间、队列长度、堆栈余量
- 优化 scan 点数
- 优化 ICP 频率
- 优化规划触发频率
- 补齐 fault handling
- 收敛 `Start -> Exit`
- 收敛 `Exit -> Start`

### 验收标准

- 双程可以稳定完成
- 系统长时间运行不死锁
- 关键指标可重复

### 必做验证

- 编译通过
- 输出 benchmark 结果记录

## 7. 推荐任务颗粒度

你给 AI Agent 下任务时，建议控制在以下粒度。

好的任务粒度示例：

- “补齐 `slam_types.h` 和 scan snapshot 结构，不要做 ICP”
- “实现固定大小 OGM 和坐标映射，不接入 A*”
- “实现轻量 ICP，失败时回退，不要碰规划模块”
- “实现 A*，输入是现有 OGM，先不做 frontier”

不好的任务粒度示例：

- “把整个 SLAM 全做完”
- “把 mapping、ICP、planning、exploration 一次性都接上”
- “顺便把控制也一起重构了”

## 8. AI Agent 常见错误清单

后续你要重点防这些问题。

- 把 `freertos.c` 变成 2000 行大杂烩
- 没有 scan snapshot，直接读取“当前 pose”去解释旧 scan
- ICP 没有失败保护
- 地图分辨率开太大
- 动态内存乱用
- 编译没过也宣称完成
- 没做验证就说“逻辑正确”
- 一次提交同时改控制、建图、定位、规划，导致无法定位回归来源

## 9. 你给 AI Agent 的推荐命令模板

下面这些模板可以直接复用。

### 模板 1: Phase 0

```text
请按 docs/development_plan/slam_icp_ai_agent_plan.md 执行 Phase 0。
目标是冻结 SLAM 开发基线，整理 pose/scan/map/path 公共结构和共享访问边界。
不要实现 ICP，不要实现 A*，不要重写控制框架。
完成后按文档规定的交付格式汇报，并确保工程可编译。
```

### 模板 2: Phase 1

```text
请按 docs/development_plan/slam_icp_ai_agent_plan.md 执行 Phase 1。
目标是修正 odometry 与 scan 对齐问题，并给每帧 scan 保存 pose snapshot。
只做 pose + scan 输入质量加固，不要实现 OGM、ICP、A*。
完成后给出修改文件、编译结果和一组调试输出示例。
```

### 模板 3: Phase 2

```text
请按 docs/development_plan/slam_icp_ai_agent_plan.md 执行 Phase 2。
目标是实现最小 OGM 建图闭环，只允许使用 odometry pose，不要引入 ICP。
请新增独立模块，不要把地图逻辑堆进 freertos.c。
完成后提供地图更新验证方式，并保证工程可编译。
```

### 模板 4: Phase 3

```text
请按 docs/development_plan/slam_icp_ai_agent_plan.md 执行 Phase 3。
目标是实现轻量 ICP 位姿修正，必须使用 odometry 作为初值，并加入失败回退。
不要实现 EKF，不要做回环，不要碰 planning。
完成后提供修正前后对比证据，并确保工程可编译。
```

### 模板 5: Phase 4

```text
请按 docs/development_plan/slam_icp_ai_agent_plan.md 执行 Phase 4。
目标是实现 A* 和 frontier exploration。
先让已知目标规划可用，再接 frontier，不要顺手重构定位模块。
完成后给出路径规划验证结果，并确保工程可编译。
```

### 模板 6: Phase 5

```text
请按 docs/development_plan/slam_icp_ai_agent_plan.md 执行 Phase 5。
目标是围绕 benchmark 做稳定性和时间优化。
优先保证控制环稳定，再优化 ICP 和规划频率。
完成后输出 benchmark 指标记录和剩余风险。
```

## 10. 最终要求

AI Agent 的开发顺序必须固定为：

1. 基线冻结
2. `pose + scan` 对齐
3. odometry-only OGM
4. 轻量 ICP
5. A* + frontier
6. benchmark 调优

如果 AI Agent 想跳过这个顺序，默认判定为高风险方案，应拒绝执行，除非你明确批准。
