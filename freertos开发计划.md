# FreeRTOS 重构计划 - STM32F446 机器人小车

## 上下文 (Context)

### 问题背景
当前项目虽然集成了 FreeRTOS V10.3.1，但控制逻辑仍在 `main()` 的 `while(1)` 循环中运行，这在 `osKernelStart()` 之后理论上不应该执行。这是一个过渡状态，需要将裸机轮询架构迁移到真正的多任务 FreeRTOS 架构。

### 当前架构问题
1. **设计缺陷**: `osKernelStart()` 后主循环不应执行，存在不可预期行为
2. **轮询效率低**: `g_system_update_flag` 轮询浪费 CPU 周期
3. **无任务隔离**: 所有功能混杂在一个循环，难以维护和扩展
4. **资源竞争风险**: 全局变量无保护，多任务环境可能产生竞态条件

### 目标
将现有裸机控制循环迁移到 FreeRTOS 多任务架构，实现：
- 高实时性的 100Hz 控制任务
- 独立的 LiDAR 解析和通信任务
- 安全的共享资源访问
- 可扩展的任务结构（为后续 SLAM 任务预留）

---

## 重构方案

### 阶段一: 修复 FreeRTOS 基础配置 (优先级: 高)

#### 1.1 启用 FPU 支持
**文件**: `Core/Inc/FreeRTOSConfig.h`
**修改**: 第 59 行
```c
// 修改前
#define configENABLE_FPU 0

// 修改后
#define configENABLE_FPU 1
```
**原因**: STM32F446 有 Cortex-M4F 内核，启用 FPU 可加速 PID 浮点计算

#### 1.2 增加堆内存
**文件**: `Core/Inc/FreeRTOSConfig.h`
**修改**: 第 71 行
```c
// 修改前
#define configTOTAL_HEAP_SIZE ((size_t)15360)  // 15KB

// 修改后
#define configTOTAL_HEAP_SIZE ((size_t)32768)  // 32KB
```
**原因**: 后续会增加多个任务、队列和互斥锁，当前 15KB 余量偏小。
**补充约束**: 修改后必须结合 `.map` 文件、`xPortGetFreeHeapSize()` 和 `uxTaskGetStackHighWaterMark()` 验证，不能只凭经验放大堆。

#### 1.3 保留 TIM4 作为 100Hz 硬件节拍
**文件**: `Core/Src/main.c`, `Core/Src/encoder.c`, `Core/Src/freertos.c`
**决策**: 第一阶段不移除 TIM4，不用 RTOS tick 直接替代控制节拍。
- TIM4 IRQ 只负责唤醒 `ControlTask`，不在中断里做控制计算
- `ControlTask` 每次被唤醒后执行一次完整的 10ms 控制周期
- 等 RTOS 版本稳定后，再评估是否可以改成纯 `osDelayUntil()` 周期任务
**推荐**: 保留 TIM4。这样能最大程度保持现有 PID、里程计和控制时序不变，降低迁移风险。

**补充说明（2026-04 实测后更新）**:
- 当前控制闭环已经验证：直接将 LiDAR 修正位姿替换为 PID 输入会破坏稳定性
- 因此现阶段策略调整为：
  - 控制层继续使用平滑的 `odometry / IMU` 状态
  - 建图层使用 `corrected_pose`
  - 后续再引入“限幅、低通后的慢修正”作为控制融合层

---

### 阶段二: 设计任务架构 (优先级: 高)

#### 2.1 任务划分

| 任务名 | 优先级 | 周期 | 堆栈大小 | 职责 |
|--------|--------|------|----------|------|
| `ControlTask` | `osPriorityRealtime` (48) | TIM4 10ms 硬件触发 | 1536 字节 | 执行 `MPU_update()`、`encoder_update_speed()`、`Odometry_Update()`、PID 控制和电机输出 |
| `LiDARParseTask` | `osPriorityNormal` (24) | 连续 + `osDelay(1)` | 1536 字节 | 解析 raw ring buffer，写入当前 LiDAR 扫描缓冲 |
| `CommTask` | `osPriorityLow` (8) | 事件驱动 | 1024 字节 | 蓝牙命令处理、遥测发送、归还 LiDAR 缓冲区所有权 |
| `SafetyTask` | `osPriorityBelowNormal` (16) | 100ms | 512 字节 | 心跳、堆栈/堆监控、故障计数；IWDG 启用后再加入喂狗 |
| `defaultTask` | `osPriorityLow` (8) | 空闲 | 512 字节 | 可保留为系统监控占位，也可在稳定后删除 |

#### 2.2 任务间通信机制

| 通信对象 | 类型 | 用途 | 位置定义 |
|----------|------|------|----------|
| `g_controlTickSem` | 二进制信号量 | TIM4 ISR → ControlTask，传递 10ms 控制节拍 | freertos.c |
| `g_lidarReadyQueue` | 消息队列 | LiDARParseTask → CommTask，传递“已完成扫描缓冲区索引” | freertos.c |
| `g_lidarFreeQueue` | 消息队列 | CommTask → LiDARParseTask，归还“可写扫描缓冲区索引” | freertos.c |
| `g_cmdQueue` | 消息队列 | UART5 RX 回调 → CommTask，传递蓝牙命令 | freertos.c |
| `g_odomMutex` | 互斥锁 | 保护里程计快照和遥测打包时的位姿读取 | freertos.c |
| `g_pidMutex` | 互斥锁 | 保护 PID 目标值、`base_car_speed` 等控制参数修改 | freertos.c |
| `g_controlMutex` | 互斥锁 | 保护 `g_control_mode` 和相对运动状态切换 | freertos.c |

---

### 阶段三: 实现任务函数 (优先级: 高)

#### 3.1 TIM4 → ControlTask 节拍通知
**文件**: `Core/Src/freertos.c`
**代码模板**:
```c
void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
    if (htim->Instance == TIM4)
    {
        osSemaphoreRelease(g_controlTickSem);
    }
}
```
**注意**:
- TIM4 中断里只做任务唤醒，不做控制运算
- 若 ISR 中调用 RTOS API，相关 IRQ 优先级必须保持数值上 **不小于** `configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY`，当前工程配置为 5
- `g_system_update_flag` 在 RTOS 版本中应删除，不再保留轮询路径
- `HAL_TIM_PeriodElapsedCallback()` 只能保留一个最终定义，不能同时散落在多个 `.c` 文件中

#### 3.2 ControlTask - 100Hz 控制主循环
**文件**: `Core/Src/freertos.c`
**代码模板**:
```c
void StartControlTask(void *argument)
{
    const float dt = 0.01f;

    for(;;)
    {
        // 等待 TIM4 的 10ms 节拍
        osSemaphoreAcquire(g_controlTickSem, osWaitForever);

        // 保持现有执行顺序，最大化行为一致性
        MPU_update();
        encoder_update_speed();

        // 获取互斥锁保护里程计
        osMutexAcquire(g_odomMutex, osWaitForever);

        // 更新里程计和控制逻辑
        Odometry_Update(dt);

        osMutexAcquire(g_controlMutex, osWaitForever);
        ControlMode mode = g_control_mode;
        osMutexRelease(g_controlMutex);

        if (mode == CONTROL_MODE_POSITION)
        {
            Update_Relative_Move_PID(dt);
        }
        else
        {
            Angle_Speed_Cascade_Control(g_th_continuous, base_car_speed, dt);
        }

        osMutexRelease(g_odomMutex);
    }
}
```

**补充说明（当前实际策略）**:
- `ControlTask` 仍是系统中最高实时性的闭环
- 在 `Phase 3B` 完成前，不应直接把 `corrected_pose / EST` 硬接入该任务的 PID 输入
- 更合理的后续做法是新增：
  - `odom_pose`
  - `control_pose`
  - `corrected_pose`
  三层状态，并只允许 LiDAR 修正以慢速、限幅方式影响 `control_pose`

#### 3.3 LiDARParseTask - LiDAR 双缓冲解析
**文件**: `Core/Src/freertos.c`
**前置重构**:
- 先把 `lidar.c` 中依赖全局 `lidar_points/point_count/scan_data_ready_flag` 的逻辑改成“写入调用方指定缓冲区”
- 将当前 `send_binary_packaged_data()` 提取为 `send_binary_packaged_data_from_buffer(...)`

**代码模板**:
```c
void StartLiDARParseTask(void *argument)
{
    uint8_t write_idx;
    osMessageQueueGet(g_lidarFreeQueue, &write_idx, NULL, osWaitForever);

    for(;;)
    {
        // 将解析结果写入“当前可写缓冲区”
        if (LIDAR_ParseStep(&g_lidarScanBuf[write_idx]) == LIDAR_SCAN_COMPLETE)
        {
            // 只传递缓冲区索引，不再共享单一全局点云缓冲
            osMessageQueuePut(g_lidarReadyQueue, &write_idx, 0, osWaitForever);

            // 等待 CommTask 归还一个空闲缓冲区
            osMessageQueueGet(g_lidarFreeQueue, &write_idx, NULL, osWaitForever);
        }

        osDelay(1);
    }
}
```
**说明**:
- `g_lidarScanBuf` 至少准备 2 份缓冲区，避免解析与发送争用同一份点云
- 队列传递的是“缓冲区所有权”，不是仅传 `point_count`
- 如果后续发现两份缓冲仍不足，再增加缓冲数量或引入丢帧计数器

#### 3.4 CommTask - 通信任务
**文件**: `Core/Src/freertos.c`
**代码模板**:
```c
void StartCommTask(void *argument)
{
    uint8_t ready_idx;
    CmdMsg_t cmdMsg;

    for(;;)
    {
        // 优先发送已经完成的一圈 LiDAR 数据
        if (osMessageQueueGet(g_lidarReadyQueue, &ready_idx, NULL, 0) == osOK)
        {
            osMutexAcquire(g_odomMutex, osWaitForever);
            send_binary_packaged_data_from_buffer(&g_lidarScanBuf[ready_idx]);
            osMutexRelease(g_odomMutex);

            // 发送完成后归还缓冲区所有权
            osMessageQueuePut(g_lidarFreeQueue, &ready_idx, 0, osWaitForever);
        }

        // 处理蓝牙命令
        if (osMessageQueueGet(g_cmdQueue, &cmdMsg, NULL, 0) == osOK)
        {
            process_command(cmdMsg.data, cmdMsg.len);
        }

        osDelay(1);
    }
}
```

#### 3.5 SafetyTask - 系统健康监控（第二阶段启用）
**文件**: `Core/Src/freertos.c`
**前置条件**:
- 第一版 RTOS 重构不把 IWDG 作为必选项
- 如果要启用硬件看门狗，必须先通过 CubeMX 增加 `IWDG` 外设、生成 `iwdg.c/iwdg.h` 并确保存在 `hiwdg`

**代码模板**:
```c
void StartSafetyTask(void *argument)
{
    for(;;)
    {
        // 检查任务堆栈使用情况（调试用）
        // uxTaskGetStackHighWaterMark(...);

        // LED 心跳指示
        HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);

#if USE_IWDG
        HAL_IWDG_Refresh(&hiwdg);
#endif

        osDelay(100);  // 10Hz
    }
}
```

---

### 阶段四: 共享资源保护 (优先级: 高)

#### 4.1 需要互斥锁保护的全局变量

| 变量 | 所在文件 | 读取者 | 写入者 | 互斥锁 |
|------|----------|--------|--------|--------|
| `g_x`, `g_y`, `g_th`, `g_th_continuous` | encoder.c | lidar.c, pid.c, hc04.c, CommTask | ControlTask (`Odometry_Update`) | `g_odomMutex` |
| `g_pid_angle.setpoint` | pid.c | pid.c, hc04.c, ControlTask | hc04.c (命令), pid.c | `g_pidMutex` |
| `g_pid_speed_left/right.setpoint` | pid.c | pid.c, ControlTask | pid.c | `g_pidMutex` |
| `base_car_speed` | pid.c | pid.c, hc04.c, ControlTask | hc04.c, pid.c | `g_pidMutex` |
| `g_control_mode` | pid.c | ControlTask, hc04.c | hc04.c, pid.c | `g_controlMutex` |

#### 4.2 RTOS 对象定义
**文件**: `Core/Src/freertos.c`
```c
// 在 Variables 区域添加
osSemaphoreId_t g_controlTickSem;
osMessageQueueId_t g_lidarReadyQueue;
osMessageQueueId_t g_lidarFreeQueue;
osMessageQueueId_t g_cmdQueue;

osMutexId_t g_odomMutex;
osMutexId_t g_pidMutex;
osMutexId_t g_controlMutex;

const osMutexAttr_t odomMutex_attr = {
    .name = "odomMutex"
};

// 在 MX_FREERTOS_Init 中添加
void MX_FREERTOS_Init(void)
{
    // ... 原有代码 ...

    /* USER CODE BEGIN RTOS_SEMAPHORES */
    g_controlTickSem = osSemaphoreNew(1, 0, NULL);
    /* USER CODE END RTOS_SEMAPHORES */

    /* USER CODE BEGIN RTOS_QUEUES */
    g_lidarReadyQueue = osMessageQueueNew(2, sizeof(uint8_t), NULL);
    g_lidarFreeQueue = osMessageQueueNew(2, sizeof(uint8_t), NULL);
    g_cmdQueue = osMessageQueueNew(8, sizeof(CmdMsg_t), NULL);
    /* USER CODE END RTOS_QUEUES */

    /* USER CODE BEGIN RTOS_MUTEX */
    g_odomMutex = osMutexNew(&odomMutex_attr);
    g_pidMutex = osMutexNew(NULL);
    g_controlMutex = osMutexNew(NULL);
    /* USER CODE END RTOS_MUTEX */

    uint8_t buf_idx0 = 0;
    uint8_t buf_idx1 = 1;
    osMessageQueuePut(g_lidarFreeQueue, &buf_idx0, 0, 0);
    osMessageQueuePut(g_lidarFreeQueue, &buf_idx1, 0, 0);

    // ... 创建任务 ...
}
```

#### 4.3 中断服务程序修改

**USART6 RX 回调** (`lidar.c:278`):
```c
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6)
    {
        uint8_t b = rplidar_rx_byte;
        if(lidar_raw_stream_active) {
            raw_push(b);
        }
        HAL_UART_Receive_IT(&huart6, (uint8_t *)&rplidar_rx_byte, 1);
    }
}
```
**注意**: 保持 ISR 精简，只操作 ring buffer，不做任务切换

**UART5 RX 回调** (`hc04.c:114`):
```c
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t size)
{
    if(huart->Instance==UART5)
    {
        // 将命令放入队列，而不是直接处理
        CmdMsg_t msg = {0};
        if (size > sizeof(msg.data)) {
            size = sizeof(msg.data);
        }
        memcpy(msg.data, buffer, size);
        msg.len = size;

        // 本工程 CMSIS-RTOS2 封装支持在 ISR 中调用 osMessageQueuePut，
        // 条件是 timeout 必须为 0
        osMessageQueuePut(g_cmdQueue, &msg, 0, 0);

        HAL_UARTEx_ReceiveToIdle_DMA(&huart5, buffer, sizeof(buffer));
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }
}
```

---

### 阶段五: 修改主函数 (优先级: 高)

**文件**: `Core/Src/main.c`

#### 5.1 移除或注释掉 while(1) 循环
```c
int main(void)
{
    HAL_Init();
    SystemClock_Config();

    MX_GPIO_Init();
    MX_DMA_Init();
    MX_TIM3_Init();
    MX_UART5_Init();
    MX_I2C1_Init();
    MX_TIM1_Init();
    MX_TIM2_Init();
    MX_SPI2_Init();
    MX_USART6_UART_Init();
    MX_TIM4_Init();

    system_init();
    PID_system_init();

    osKernelInitialize();
    MX_FREERTOS_Init();
    osKernelStart();

    /* 以下代码理论上不会执行 */
    /* 但如果执行到了，说明 FreeRTOS 启动失败 */
    Error_Handler();
}
```

#### 5.2 删除/注释的内容
- 删除 `main.c` 第 155-192 行的 `while(1)` 循环
- 删除 `main.c` 中基于 `g_system_update_flag` 的轮询控制逻辑
- 删除 `main.c` 中基于 `scan_data_ready_flag` 的 LiDAR 发送逻辑
- 可删除 `main.c` 第 54-61 行里 RTOS 版本不再使用的局部全局变量
- 第一版 **不要删除** `system_init()` 和 `PID_system_init()` 调用，它们仍然负责启动前初始化

#### 5.3 保留的内容
- 保留外设初始化 `MX_GPIO_Init()` 等
- 保留 `system_init()` 和 `PID_system_init()` 调用，并放在 `osKernelInitialize()` 之前
- 保留 `osKernelStart()` 后的 `Error_Handler()` 兜底
- 后续如果要继续清理，再把 `system_init()` 拆成 `platform_init()` 与 `control_init()`

---

### 阶段六: 逐步验证方案 (推荐执行顺序)

考虑到风险，建议分步迁移：

#### 步骤 1: 最小 RTOS 控制闭环
1. 保留 `system_init()` 与 `PID_system_init()`，删除业务 `while(1)` 循环
2. 创建 `g_controlTickSem` 和 `ControlTask`
3. TIM4 ISR 改为仅唤醒 `ControlTask`
4. 验证 100Hz 控制环、F/B/L/R 命令和电机输出正常

#### 步骤 2: 迁移蓝牙命令处理
1. 创建 `g_cmdQueue` 和 `CommTask`
2. UART5 RX 回调只入队，不直接调用 `process_command()`
3. 验证命令处理不再阻塞中断上下文

#### 步骤 3: 迁移 LiDAR 解析与发送
1. 先把 `lidar.c` 重构为“双缓冲 + 缓冲区所有权交接”模型
2. 创建 `LiDARParseTask`、`g_lidarReadyQueue`、`g_lidarFreeQueue`
3. 将遥测发送函数改为从指定缓冲区打包发送
4. 验证 LiDAR 扫描与蓝牙发送并发时无 ring buffer 溢出

#### 步骤 4: 清理和增强
1. 清理 `g_system_update_flag`、`scan_data_ready_flag` 等旧路径
2. 优化任务优先级和堆栈大小
3. 按需引入 `SafetyTask`、任务统计和硬件看门狗

---

## 关键文件修改清单

| 文件路径 | 修改类型 | 修改内容 |
|----------|----------|----------|
| `Core/Inc/FreeRTOSConfig.h` | 配置修改 | 启用 FPU (configENABLE_FPU=1)，增加堆大小 |
| `Core/Src/freertos.c` | 新增代码 | 定义任务、互斥锁、信号量、队列，实现任务函数 |
| `Core/Inc/freertos_app.h` (新建) | 新增文件 | 声明 RTOS 对象、缓冲区结构和任务函数原型 |
| `Core/Src/main.c` | 删除代码 | 移除 while(1) 循环，简化 main 函数 |
| `Core/Src/encoder.c` | 修改 | 移除 `g_system_update_flag` 路径，仅保留里程计与编码器逻辑 |
| `Core/Src/lidar.c` | 修改 | 改为双缓冲解析，并支持从指定缓冲区打包发送 |
| `Core/Src/hc04.c` | 修改 | UART 回调改用队列发送命令，命令处理移至 `CommTask` |
| `Core/Src/pid.c` | 包装函数 | 添加带互斥锁保护的 PID 参数设置函数 |
| `Core/Src/system.h` | 新增声明 | 添加 RTOS 对象和共享结构外部声明 |

---

## 验证计划

### 编译验证
```bash
cmake -DCMAKE_BUILD_TYPE=Debug -B cmake-build-debug -G "MinGW Makefiles"
cmake --build cmake-build-debug
```

### 运行时验证
1. **任务创建验证**: 检查所有任务是否正常创建，无内存分配失败
2. **时序验证**: 使用示波器或逻辑分析仪检查 ControlTask 是否稳定 100Hz
3. **功能验证**: 测试 F/B/L/R 命令，验证小车运动正常
4. **并发验证**: 同时运行 LiDAR 扫描和蓝牙通信，检查无数据丢失
5. **长时间稳定性**: 运行 >10 分钟，检查无死锁、无内存泄漏

### 调试技巧
- 使用 `uxTaskGetStackHighWaterMark()` 监控堆栈使用
- 使用 `xPortGetFreeHeapSize()` 监控堆内存
- 使用 `vTaskList()` 输出任务状态（需启用 configUSE_TRACE_FACILITY）

---

## 风险与缓解

| 风险 | 可能性 | 影响 | 缓解措施 |
|------|--------|------|----------|
| 任务优先级配置错误导致控制不稳定 | 中 | 高 | 先使用保守优先级，逐步调优 |
| 互斥锁使用不当导致死锁 | 中 | 高 | 使用超时参数，避免无限等待；代码审查 |
| 堆栈溢出 | 中 | 高 | 使用较大初始堆栈，监控高水位标记 |
| 任务唤醒抖动影响 PID 性能 | 中 | 中 | 第一阶段保持 TIM4 硬件节拍，只让 ISR 唤醒 `ControlTask`；稳定后再评估纯 RTOS 定时 |
| LiDAR 双缓冲被发送任务占满 | 中 | 中 | 先用 2 份缓冲区实现所有权交接；若仍不足，增加缓冲数量并记录丢帧计数 |
| FreeRTOS 启动失败 | 低 | 高 | 保留 Error_Handler 检测；逐步迁移策略 |

---

## 后续扩展

本计划完成后，可方便地添加：
1. **MappingTask**: 占用栅格地图更新 (50ms 周期)
2. **PlanningTask**: A* 路径规划 (事件触发)
3. **ExplorationTask**: 前沿检测和自主探索
4. **UITask**: OLED 显示更新 (100ms 周期)
