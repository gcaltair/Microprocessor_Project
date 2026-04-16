#ifndef FREERTOS_APP_H
#define FREERTOS_APP_H

#include <stdint.h>

#include "cmsis_os.h"
#include "slam_types.h"
#include "../Src/lidar.h"

#define CMD_MSG_BUFFER_SIZE      100U
#define LIDAR_SCAN_BUFFER_COUNT  2U
#define LIDAR_DMA_BLOCK_QUEUE_LENGTH  2U

typedef struct {
    uint16_t len;
    uint8_t data[CMD_MSG_BUFFER_SIZE];
} CmdMsg_t;

typedef struct {
    uint16_t offset;
    uint16_t length;
    uint32_t sequence;
} LidarDmaBlockMsg_t;

typedef struct {
    uint8_t scan_index;
    uint16_t point_count;
    uint32_t scan_sequence;
    SlamPose2D_t pose_snapshot;
} LidarScanMsg_t;

typedef struct {
    uint32_t control_cycles;
    uint32_t control_tick_overruns;
    uint32_t cmd_rx_count;
    uint32_t cmd_drop_count;
    uint32_t lidar_dma_block_count;
    uint32_t lidar_dma_drop_count;
    uint32_t lidar_scan_complete_count;
    uint32_t lidar_tx_count;
    uint32_t lidar_tx_busy_count;
    uint32_t lidar_tx_error_count;
    uint32_t bt_tx_wait_count;
    uint32_t free_heap_bytes;
    uint32_t min_ever_free_heap_bytes;
    uint32_t default_task_stack_free_bytes;
    uint32_t control_task_stack_free_bytes;
    uint32_t lidar_task_stack_free_bytes;
    uint32_t comm_task_stack_free_bytes;
    uint32_t safety_task_stack_free_bytes;
} FreertosRuntimeStats_t;

typedef struct LidarScanBuffer {
    LidarPoint_t points[MAX_LIDAR_POINTS_PER_SCAN];
    uint16_t point_count;
} LidarScanBuffer_t;

extern osSemaphoreId_t g_controlTickSem;
extern osMessageQueueId_t g_lidarBlockQueue;
extern osMessageQueueId_t g_lidarResultQueue;
extern osMessageQueueId_t g_lidarFreeQueue;
extern osMessageQueueId_t g_cmdQueue;

extern osMutexId_t g_odomMutex;
extern osMutexId_t g_pidMutex;
extern osMutexId_t g_controlMutex;

extern LidarScanBuffer_t g_lidarScanBuf[LIDAR_SCAN_BUFFER_COUNT];
extern FreertosRuntimeStats_t g_runtimeStats;

void StartControlTask(void *argument);
void StartLiDARParseTask(void *argument);
void StartCommTask(void *argument);
void StartSafetyTask(void *argument);

osStatus_t Freertos_NotifyControlTickFromISR(void);
osStatus_t Freertos_SubmitCommandFromISR(const uint8_t *data, uint16_t len);
osStatus_t Freertos_SubmitLidarBlockFromISR(uint16_t offset, uint16_t length);
void Freertos_ResetLidarPipeline(void);
void Freertos_GetRuntimeStatsSnapshot(FreertosRuntimeStats_t *stats);
void Freertos_RecordBluetoothTxWait(void);

#endif /* FREERTOS_APP_H */
