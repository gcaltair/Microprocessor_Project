#ifndef FREERTOS_APP_H
#define FREERTOS_APP_H

#include <stdint.h>

#include "cmsis_os.h"
#include "../Src/lidar.h"

#define CMD_MSG_BUFFER_SIZE      100U
#define LIDAR_SCAN_BUFFER_COUNT  2U

typedef struct {
    uint16_t len;
    uint8_t data[CMD_MSG_BUFFER_SIZE];
} CmdMsg_t;

typedef struct LidarScanBuffer {
    LidarPoint_t points[MAX_LIDAR_POINTS_PER_SCAN];
    uint16_t point_count;
} LidarScanBuffer_t;

extern osSemaphoreId_t g_controlTickSem;
extern osMessageQueueId_t g_lidarReadyQueue;
extern osMessageQueueId_t g_lidarFreeQueue;
extern osMessageQueueId_t g_cmdQueue;

extern osMutexId_t g_odomMutex;
extern osMutexId_t g_pidMutex;
extern osMutexId_t g_controlMutex;

extern LidarScanBuffer_t g_lidarScanBuf[LIDAR_SCAN_BUFFER_COUNT];

void StartControlTask(void *argument);
void StartLiDARParseTask(void *argument);
void StartCommTask(void *argument);
void StartSafetyTask(void *argument);

osStatus_t Freertos_NotifyControlTickFromISR(void);
osStatus_t Freertos_SubmitCommandFromISR(const uint8_t *data, uint16_t len);

#endif /* FREERTOS_APP_H */
