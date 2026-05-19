#ifndef FREERTOS_APP_H
#define FREERTOS_APP_H

#include <stdint.h>

#include "cmsis_os.h"
#include "scan_preprocess.h"
#include "slam_types.h"

#define LIDAR_SCAN_BUFFER_COUNT  4U
#define LIDAR_DMA_BLOCK_QUEUE_LENGTH  16U

typedef enum {
    LOCALIZATION_MODE_ODOMETRY_ONLY = 0
} LocalizationMode_t;

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
    SlamPose2D_t corrected_pose;
    LidarScanQuality_t quality;
    float localization_fitness_m;
    uint16_t localization_inliers;
    uint8_t localization_mode;
    uint8_t map_update_allowed;
    uint8_t turning_detected;
    uint8_t map_skip_reason;
    float odom_delta_theta_deg;
    float odom_delta_translation_m;
} LidarScanMsg_t;

typedef struct {
    uint32_t control_cycles;
    uint32_t control_tick_overruns;
    uint32_t lidar_dma_block_count;
    uint32_t lidar_dma_drop_count;
    uint32_t lidar_dma_stale_block_count;
    uint32_t lidar_dma_max_block_lag;
    uint32_t lidar_scan_complete_count;
    uint32_t lidar_parser_resync_count;
    uint16_t last_scan_raw_point_count;
    uint16_t last_scan_usable_point_count;
    uint16_t last_scan_rejected_range_count;
    uint16_t last_scan_rejected_quality_count;
    uint16_t last_scan_min_distance_mm;
    uint16_t last_scan_max_distance_mm;
    uint32_t free_heap_bytes;
    uint32_t min_ever_free_heap_bytes;
    uint32_t default_task_stack_free_bytes;
    uint32_t control_task_stack_free_bytes;
    uint32_t lidar_task_stack_free_bytes;
    uint32_t localization_task_stack_free_bytes;
    uint32_t mapping_task_stack_free_bytes;
    uint32_t safety_task_stack_free_bytes;
} FreertosRuntimeStats_t;

typedef struct LidarScanBuffer {
    LidarPoint_t points[MAX_LIDAR_POINTS_PER_SCAN];
    uint16_t point_count;
} LidarScanBuffer_t;

extern osSemaphoreId_t g_controlTickSem;
extern osMessageQueueId_t g_lidarBlockQueue;
extern osMessageQueueId_t g_lidarResultQueue;
extern osMessageQueueId_t g_localizedScanQueue;
extern osMessageQueueId_t g_lidarFreeQueue;

extern osMutexId_t g_odomMutex;
extern osMutexId_t g_localizationMutex;
extern osMutexId_t g_gridMutex;
extern osMutexId_t g_pidMutex;
extern osMutexId_t g_controlMutex;

extern LidarScanBuffer_t g_lidarScanBuf[LIDAR_SCAN_BUFFER_COUNT];
extern FreertosRuntimeStats_t g_runtimeStats;

void StartControlTask(void *argument);
void StartLiDARParseTask(void *argument);
void StartLocalizationTask(void *argument);
void StartMappingTask(void *argument);
void StartSafetyTask(void *argument);
void StartTelemetryTask(void *argument);

osStatus_t Freertos_NotifyControlTickFromISR(void);
osStatus_t Freertos_SubmitLidarBlockFromISR(uint16_t offset, uint16_t length);
void Freertos_ResetLidarPipeline(void);
void Freertos_GetRuntimeStatsSnapshot(FreertosRuntimeStats_t *stats);

#endif /* FREERTOS_APP_H */
