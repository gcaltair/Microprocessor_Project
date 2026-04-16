# FreeRTOS 进一步开发计划

## 文档信息
- **版本**: v1.0
- **日期**: 2026-04-16
- **分支**: dev/Feature/FreeRTOS
- **目标**: 在现有 FreeRTOS 架构基础上，完成稳定性优化和功能扩展

---

## 1. 当前状态评估

### 1.1 已完成的工作

| 模块 | 状态 | 关键实现 |
|------|------|----------|
| 基础架构 | 完成 | 5个任务、3个互斥锁、5个队列/信号量 |
| ControlTask | 完成 | 100Hz TIM4 硬件节拍驱动 |
| LiDAR DMA 双缓冲 | 完成 | 半传输/全传输中断 + 队列 |
| 命令队列化 | 完成 | UART5 RX → cmdQueue → DefaultTask |
| 运行时统计 | 完成 | 堆、栈、各类计数器 |
| LocalizationTask | 完成（地图层） | 轻量 scan matching，输出 `predicted_pose / corrected_pose` |
| 建图位姿修正 | 完成 | MappingTask 统一使用 `corrected_pose` |

### 1.2 已知问题

| 问题 | 严重程度 | 位置 | 影响 |
|------|----------|------|------|
| 互斥锁嵌套顺序需验证 | 中 | 多处 | 潜在死锁风险 |
| `g_left/right_speed` 非 volatile | 低 | encoder.c:17-18 | 编译器优化可能导致问题 |
| ControlTask 无超时保护 | 中 | freertos.c:326 | TIM4 故障时永久阻塞 |
| CommTask DMA 忙等待无上限 | 低 | freertos.c:430-445 | DMA 故障时死循环 |
| DefaultTask 职责重叠 | 低 | freertos.c:191-206 | 与 CommTask 分工不明 |
| LiDAR 修正直接替换控制位姿会破坏 PID 稳定性 | 高 | 控制/定位接口层 | 小车静止时可能被姿态抖动驱动 |

### 1.3 当前工程结论

现阶段已经验证：

- 建图层使用 `corrected_pose` 是合理的
- 控制层直接使用 `EST` 或 `corrected_pose` 作为 PID 输入是不合理的
- 下一阶段必须引入“分层融合”而不是“直接替换”

建议明确三类位姿：

- `odom_pose`: 高频、平滑，直接来自编码器/IMU
- `control_pose`: 用于控制层的融合位姿，只允许慢修正
- `corrected_pose / nav_est_pose`: 用于建图和导航层的高精位姿

---

## 2. 开发阶段规划

### 阶段一：稳定性优化（预计 1-2 周）

#### 2.1.1 互斥锁顺序规范化
**优先级**: 高
**目标**: 消除潜在死锁风险

```c
// 定义全局锁获取顺序（在 freertos_app.h 中）
#define LOCK_ORDER_ODOM     1
#define LOCK_ORDER_CONTROL  2
#define LOCK_ORDER_PID      3

// 封装统一的锁管理函数
void Locks_AcquireOdomControlPid(void);
void Locks_ReleasePidControlOdom(void);
```

**验证方法**:
- 代码审查：检查所有 `osMutexAcquire` 调用顺序
- 运行时：启用 `configASSERT` 检测嵌套违规

#### 2.1.2 添加超时保护
**优先级**: 高
**文件**: `Core/Src/freertos.c`

```c
// ControlTask 改进
void StartControlTask(void *argument)
{
    const float dt = 0.01f;
    const uint32_t tick_timeout_ms = 20;  // 2个周期的容忍

    (void)argument;

    for (;;) {
        if (osSemaphoreAcquire(g_controlTickSem, pdMS_TO_TICKS(tick_timeout_ms)) != osOK) {
            g_runtimeStats.control_tick_timeouts++;
            // 可选择：尝试恢复或进入安全模式
            continue;
        }
        // ... 原有控制逻辑
    }
}

// CommTask DMA 发送改进
#define LIDAR_TX_MAX_RETRIES 100  // 100ms 上限
```

#### 2.1.3 修复 volatile 声明
**优先级**: 中
**文件**: `Core/Src/encoder.c`

```c
// 修改前
float g_left_speed = 0.0f;
float g_right_speed = 0.0f;

// 修改后
volatile float g_left_speed = 0.0f;
volatile float g_right_speed = 0.0f;
```

#### 2.1.4 统一任务职责
**优先级**: 中
**方案**: 将 DefaultTask 改为 SystemMonitorTask

```c
void StartSystemMonitorTask(void *argument)
{
    (void)argument;

    for (;;) {
        // 1. 检查各任务健康状态
        // 2. 输出综合状态报告（降低频率，如 1Hz）
        // 3. 检测死锁迹象（如某任务长时间无响应）
        osDelay(1000);
    }
}
```

**交付标准**:
- [ ] 所有互斥锁按固定顺序获取
- [ ] ControlTask 有超时保护并记录
- [ ] 全局速度变量声明为 volatile
- [ ] 各任务职责清晰无重叠

---

### 阶段二：功能增强（预计 2-3 周）

#### 2.2.1 看门狗集成（IWDG）
**优先级**: 高
**依赖**: 需 CubeMX 重新生成代码添加 IWDG 外设

```c
// freertos.c - SafetyTask 扩展
void StartSafetyTask(void *argument)
{
    (void)argument;

    for (;;) {
        // 现有统计代码...

#if defined(USE_IWDG)
        // 检查关键任务是否还在运行
        uint32_t now = HAL_GetTick();
        if ((now - g_controlTask_last_wake) < 100 &&
            (now - g_lidarTask_last_wake) < 500) {
            HAL_IWDG_Refresh(&hiwdg);
        }
#endif

        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
        osDelay(100);
    }
}
```

#### 2.2.2 添加事件标志组（EventFlags）
**优先级**: 中
**用途**: 替代部分轮询，提高响应性

```c
// freertos_app.h 新增
extern osEventFlagsId_t g_systemEvents;

#define EVENT_CONTROL_MODE_CHANGED  0x0001
#define EVENT_LIDAR_SCAN_STARTED    0x0002
#define EVENT_LIDAR_SCAN_STOPPED    0x0004
#define EVENT_EMERGENCY_STOP        0x0008

// SafetyTask 中检测紧急停止
void StartSafetyTask(void *argument)
{
    for (;;) {
        uint32_t flags = osEventFlagsWait(g_systemEvents,
                                           EVENT_EMERGENCY_STOP,
                                           osFlagsWaitAny, 0);
        if (flags & EVENT_EMERGENCY_STOP) {
            Control_EmergencyStop();
        }
        // ...
    }
}
```

#### 2.2.3 软件定时器替代部分轮询
**优先级**: 低
**用途**: 降低任务复杂度

```c
// 定时器回调：定期发送遥测（替代部分 CommTask 逻辑）
void TelemetryTimer_Callback(void *argument)
{
    (void)argument;
    osMessageQueuePut(g_telemetryQueue, &telemetry_data, 0, 0);
}
```

#### 2.2.4 堆栈溢出检测
**优先级**: 中
**配置**: `configCHECK_FOR_STACK_OVERFLOW = 2`

```c
// 实现钩子函数
void vApplicationStackOverflowHook(TaskHandle_t xTask, char *pcTaskName)
{
    (void)xTask;
    // 记录到错误日志
    snprintf(g_errorLog.lastTaskName, sizeof(g_errorLog.lastTaskName),
             "%s", pcTaskName);
    g_errorLog.stackOverflowCount++;

    // 进入安全模式
    Control_EmergencyStop();

    // 可选：通过蓝牙发送错误报告
    Error_Report(ERROR_STACK_OVERFLOW, pcTaskName);
}
```

**交付标准**:
- [ ] IWDG 正常喂狗，故障时复位
- [ ] 事件标志组实现紧急停止
- [ ] 软件定时器实现遥测发送
- [ ] 堆栈溢出时进入安全模式

---

### 阶段三：SLAM 任务集成（预计 3-4 周）

#### 2.3.1 MappingTask - 占用栅格地图
**优先级**: 高
**周期**: 50ms (20Hz)
**优先级**: osPriorityNormal (24)
**堆栈**: 2048 字节（需要更多空间进行地图计算）

```c
void StartMappingTask(void *argument)
{
    LidarScanMsg_t scan_msg;
    uint8_t buffer_idx;

    (void)argument;

    OccupancyGrid_Init(&g_grid, GRID_SIZE_M, GRID_RESOLUTION_M);

    for (;;) {
        // 等待 LiDAR 扫描完成
        if (osMessageQueueGet(g_lidarResultQueue, &scan_msg, NULL, 50) == osOK) {
            buffer_idx = scan_msg.scan_index;

            osMutexAcquire(g_odomMutex, osWaitForever);
            Pose2D_t pose = {
                .x = g_x,
                .y = g_y,
                .theta = g_th_continuous
            };
            osMutexRelease(g_odomMutex);

            // 更新栅格地图
            OccupancyGrid_Update(&g_grid, &pose, &g_lidarScanBuf[buffer_idx]);

            // 归还缓冲区
            osMessageQueuePut(g_lidarFreeQueue, &buffer_idx, 0, 0);
        }

        osDelay(50);
    }
}
```

**注意**: MappingTask 需要从 CommTask 中分离 LiDAR 数据消费，或采用多消费者模式。

#### 2.3.2 LocalizationTask - 分层融合定位
**优先级**: 高
**目标**: 先实现“地图可用定位”，再实现“控制可用定位”

本阶段需要拆成两个子阶段，而不是直接把 LiDAR 修正覆盖控制位姿：

- **Phase 3A 已完成**
  - `LocalizationTask` 输出 `predicted_pose / corrected_pose`
  - `MappingTask` 使用 `corrected_pose`
  - 控制层继续使用 `odom_pose`

- **Phase 3B 待完成**
  - 引入 `control_pose`
  - LiDAR 修正只以“限幅、低通、慢修正”的方式注入控制层
  - 导航层继续使用 `corrected_pose / nav_est_pose`

建议目标结构：

```c
typedef struct {
    SlamPose2D_t odom_pose;         // high-rate
    SlamPose2D_t control_pose;      // bounded correction
    SlamPose2D_t corrected_pose;    // mapping/navigation
} PoseFusionState_t;
```

控制融合约束建议：

- 位置修正每周期限幅到毫米级
- 角度修正每周期限幅到小角度
- 仅在 ICP `fitness / inliers` 满足条件时允许注入
- 静止状态下不得因 LiDAR 抖动触发电机输出

#### 2.3.3 PlanningTask - A* 路径规划
**优先级**: 中
**触发**: 事件驱动（收到目标点后执行）
**优先级**: osPriorityBelowNormal (16)
**堆栈**: 2560 字节（路径规划需要较大栈空间）

```c
void StartPlanningTask(void *argument)
{
    NavigationGoal_t goal;
    Path_t path;

    (void)argument;

    for (;;) {
        // 等待导航目标
        if (osMessageQueueGet(g_navGoalQueue, &goal, NULL, osWaitForever) == osOK) {

            osMutexAcquire(g_gridMutex, osWaitForever);
            // A* 搜索
            PathPlanResult_t result = AStar_Plan(&g_grid,
                                                  &g_currentPose,
                                                  &goal.targetPose,
                                                  &path);
            osMutexRelease(g_gridMutex);

            if (result == PATH_SUCCESS) {
                // 发送路径到执行器
                osMessageQueuePut(g_pathQueue, &path, 0, osWaitForever);
            } else {
                // 报告规划失败
                Error_Report(ERROR_PATH_PLANNING_FAILED, NULL);
            }
        }
    }
}
```

#### 2.3.4 ExplorationTask - 自主探索
**优先级**: 低
**周期**: 500ms
**优先级**: osPriorityLow (8)
**堆栈**: 1536 字节

```c
void StartExplorationTask(void *argument)
{
    (void)argument;

    for (;;) {
        // 检测前沿点（边界）
        FrontierList_t frontiers;
        FrontierDetector_Find(&g_grid, &g_currentPose, &frontiers);

        if (frontiers.count > 0) {
            // 选择最近的前沿点作为探索目标
            Pose2D_t target = Frontier_SelectBest(&frontiers, &g_currentPose);

            NavigationGoal_t goal = {
                .targetPose = target,
                .type = NAV_GOAL_EXPLORATION
            };

            osMessageQueuePut(g_navGoalQueue, &goal, 0, 0);
        }

        osDelay(500);
    }
}
```

#### 2.3.5 任务优先级与通信关系图

```
Priority    Task                    Communication
--------    ----                    -------------
48          ControlTask             <-- TIM4 ISR (Semaphore)
                                    --> LocalizationTask (Semaphore)
                                    <-- g_odomMutex, g_pidMutex, g_controlMutex

32          LocalizationTask        <-- ControlTask (Semaphore)
                                    --> g_odomMutex

24          MappingTask             <-- g_lidarResultQueue
                                    --> g_lidarFreeQueue
                                    <-- g_gridMutex

16          PlanningTask            <-- g_navGoalQueue (Event-driven)
                                    --> g_pathQueue
                                    <-- g_gridMutex

12          SafetyTask              <-- Watchdog
                                    --> g_systemEvents

8           ExplorationTask         --> g_navGoalQueue
                                    <-- g_gridMutex (Read-only)

8           CommTask                <-- g_lidarResultQueue (需要调整)
                                    <-- g_cmdQueue
                                    <-- g_odomMutex

8           DefaultTask/SystemMonitorTask  <-- (监控各任务状态)
```

**交付标准**:
- [ ] MappingTask 正常更新栅格地图
- [ ] LocalizationTask 稳定输出 `predicted_pose / corrected_pose`
- [ ] 控制层不直接消费 `corrected_pose`
- [ ] `control_pose` 融合策略设计完成
- [ ] PlanningTask A* 规划路径
- [ ] ExplorationTask 自主探索边界
- [ ] 各任务间通信无数据丢失

#### 2.3.6 控制侧准入验证

在进入 A*、frontier 或回程之前，必须先通过以下控制侧验证：

1. 静止 30s 不自转、不蠕动
2. 原地转角误差可重复
3. 直线前进 1m 的误差稳定
4. 前进后退回零误差可接受
5. 开启 LiDAR 建图后，以上控制指标不明显恶化

---

### 阶段四：性能优化与调优（预计 1-2 周）

#### 2.4.1 控制抖动优化
**目标**: ControlTask 实际执行周期抖动 < 1ms

**方法**:
1. 使用 `osDelayUntil()` 替代信号量（评估是否可行）
2. 记录每个周期的实际 dt，计算统计值

```c
void StartControlTask(void *argument)
{
    TickType_t xLastWakeTime = xTaskGetTickCount();
    const TickType_t xFrequency = pdMS_TO_TICKS(10);

    for (;;) {
        vTaskDelayUntil(&xLastWakeTime, xFrequency);

        uint32_t start_us = DWT->CYCCNT;  // 使用 DWT 周期计数器

        // ... 控制逻辑

        uint32_t elapsed_us = (DWT->CYCCNT - start_us) / (SystemCoreClock / 1000000);
        g_runtimeStats.control_exec_time_us = elapsed_us;

        if (elapsed_us > 5000) {  // >5ms 警告
            g_runtimeStats.control_overrun_count++;
        }
    }
}
```

#### 2.4.2 内存优化
**目标**: 减少堆使用，评估静态分配

```c
// 评估将部分队列改为静态分配
static StaticQueue_t g_lidarResultQueueStatic;
static uint8_t g_lidarResultQueueStorage[2 * sizeof(LidarScanMsg_t)];

g_lidarResultQueue = xQueueCreateStatic(
    2, sizeof(LidarScanMsg_t),
    g_lidarResultQueueStorage, &g_lidarResultQueueStatic);
```

#### 2.4.3 LiDAR 数据流优化
**问题**: CommTask 和 MappingTask 竞争 LiDAR 数据
**方案**: 实现多播或添加路由逻辑

```c
// 添加 LiDAR 数据路由器
void StartLiDARRouterTask(void *argument)
{
    LidarScanMsg_t scan_msg;

    for (;;) {
        if (osMessageQueueGet(g_lidarRawQueue, &scan_msg, NULL, osWaitForever) == osOK) {
            // 发送到 MappingTask
            osMessageQueuePut(g_lidarResultQueue, &scan_msg, 0, 0);

            // 发送到 CommTask（如果蓝牙连接且空闲）
            if (!g_bluetoothBusy) {
                osMessageQueuePut(g_lidarTxQueue, &scan_msg, 0, 0);
            } else {
                // 直接归还缓冲区，CommTask 不发送此帧
                osMessageQueuePut(g_lidarFreeQueue, &scan_msg.scan_index, 0, 0);
            }
        }
    }
}
```

**交付标准**:
- [ ] ControlTask 抖动 < 1ms
- [ ] 堆内存使用峰值 < 80%
- [ ] LiDAR 数据无丢失（或丢帧可接受）

---

## 3. 测试验证计划

### 3.1 单元测试

| 测试项 | 方法 | 通过标准 |
|--------|------|----------|
| 互斥锁顺序 | 代码审查 + 运行时检测 | 无死锁发生 |
| 任务堆栈 | `uxTaskGetStackHighWaterMark()` | 剩余 >20% |
| 控制周期 | GPIO 翻转 + 逻辑分析仪 | 100Hz ± 1% |
| 队列深度 | 运行时统计 | 无溢出记录 |

### 3.2 集成测试

| 测试场景 | 步骤 | 通过标准 |
|----------|------|----------|
| 基础运动 | F/B/L/R 命令 | 小车响应正确，无异常停止 |
| 相对移动 | P0.5,0.5 命令 | 准确到达目标点（±5cm） |
| LiDAR 扫描 | M 命令启动 | 数据连续，无丢帧 |
| 长时稳定性 | 连续运行 30 分钟 | 无死锁，堆内存稳定 |
| 并发压力 | LiDAR + 运动同时运行 | 控制周期稳定 |

### 3.3 SLAM 测试

| 测试项 | 环境 | 通过标准 |
|--------|------|----------|
| 地图构建 | 5x5m 迷宫 | 地图边界清晰，无漂移 |
| 重定位 | 移动到已知位置 | 位姿误差 < 10cm |
| 路径规划 | 设置目标点 | 生成可行路径，无碰撞 |
| 自主探索 | 空地图启动 | 覆盖所有可达区域 |

---

## 4. 风险与缓解

| 风险 | 阶段 | 可能性 | 影响 | 缓解措施 |
|------|------|--------|------|----------|
| SLAM 任务过多导致控制抖动 | 阶段三 | 中 | 高 | 严格优先级分配，控制任务永不阻塞 |
| 堆栈溢出 | 阶段三 | 中 | 高 | 增加初始栈大小，启用溢出检测 |
| EKF 收敛问题 | 阶段三 | 中 | 中 | 添加退化检测，降级到纯编码器 |
| 栅格地图内存不足 | 阶段三 | 中 | 高 | 使用稀疏表示或分层地图 |
| IWDG 配置错误导致复位 | 阶段二 | 低 | 高 | 先在调试环境验证喂狗逻辑 |

---

## 5. 附录

### 5.1 新增任务资源预算

| 任务 | 堆栈(字节) | 周期 | 每次执行时间(估计) |
|------|-----------|------|-------------------|
| MappingTask | 2048 | 50ms | ~10ms |
| LocalizationTask | 1536 | 10ms | ~2ms |
| PlanningTask | 2560 | 事件驱动 | ~50-200ms |
| ExplorationTask | 1536 | 500ms | ~20ms |
| LiDARRouterTask | 1024 | 连续 | ~1ms |
| **总计新增** | **~8700** | - | - |

**当前堆**: 32KB
**当前已用**: ~15KB（5个任务 + 队列）
**剩余**: ~17KB
**新增后剩余**: ~8KB（余量 25%，可接受）

### 5.2 关键配置文件变更

**FreeRTOSConfig.h**:
```c
// 可能需要增加最大任务数
#define configMAX_PRIORITIES  56  // 保持不变，足够

// 启用更多调试功能（开发阶段）
#define configUSE_TRACE_FACILITY    1
#define configGENERATE_RUN_TIME_STATS 1
```

### 5.3 调试技巧

1. **任务状态打印**: 通过蓝牙命令触发 `vTaskList()` 输出
2. **运行时统计**: 使用 `vTaskGetRunTimeStats()` 分析 CPU 占用
3. **断点策略**: 避免在 ISR 中打断，优先在任务上下文调试
4. **日志分级**:
   - ERROR: 严重错误，可能影响安全
   - WARN: 警告，功能可能受限
   - INFO: 关键状态变化
   - DEBUG: 详细调试信息（编译时开关）

---

## 6. 任务分工建议

| 开发者 | 负责模块 | 预计工时 |
|--------|----------|----------|
| A | 阶段一稳定性优化 | 16h |
| B | 阶段二看门狗/事件标志 | 12h |
| C | MappingTask + 栅格地图 | 20h |
| D | LocalizationTask (EKF) | 20h |
| E | PlanningTask (A*) | 16h |
| F | ExplorationTask + 集成测试 | 16h |

**总计**: 约 100 工时（2-3 周并行开发）

---

## 7. 版本历史

| 版本 | 日期 | 作者 | 变更说明 |
|------|------|------|----------|
| v1.0 | 2026-04-16 | - | 初始版本 |

