/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "navigation_task.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>

#include "freertos_app.h"
#include "localization_task.h"
#include "mapping_task.h"
#include "scan_preprocess.h"
#include "system.h"
#include "telemetry.h"
#include "tim.h"
/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */
/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */
/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */
osSemaphoreId_t g_controlTickSem = NULL;
osMessageQueueId_t g_lidarBlockQueue = NULL;
osMessageQueueId_t g_lidarResultQueue = NULL;
osMessageQueueId_t g_localizedScanQueue = NULL;
osMessageQueueId_t g_lidarFreeQueue = NULL;

osMutexId_t g_odomMutex = NULL;
osMutexId_t g_localizationMutex = NULL;
osMutexId_t g_gridMutex = NULL;
osMutexId_t g_pidMutex = NULL;
osMutexId_t g_controlMutex = NULL;

LidarScanBuffer_t g_lidarScanBuf[LIDAR_SCAN_BUFFER_COUNT];
FreertosRuntimeStats_t g_runtimeStats;
static volatile uint32_t g_lidarBlockSequence = 0U;
static uint32_t g_lidarScanSequence = 0U;
/* USER CODE END Variables */

/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 512,
  .priority = (osPriority_t) osPriorityLow,
};

/* USER CODE BEGIN ThreadDefs */
osThreadId_t controlTaskHandle;
osThreadId_t lidarParseTaskHandle;
osThreadId_t localizationTaskHandle;
osThreadId_t mappingTaskHandle;
osThreadId_t safetyTaskHandle;
osThreadId_t telemetryTaskHandle;

const osThreadAttr_t controlTask_attributes = {
  .name = "controlTask",
  .stack_size = 1536,
  .priority = (osPriority_t) osPriorityRealtime,
};

const osThreadAttr_t lidarParseTask_attributes = {
  .name = "lidarParseTask",
  .stack_size = 1536,
  .priority = (osPriority_t) osPriorityNormal,
};

const osThreadAttr_t localizationTask_attributes = {
  .name = "localizeTask",
  .stack_size = 2048,
  .priority = (osPriority_t) osPriorityLow,
};

const osThreadAttr_t mappingTask_attributes = {
  .name = "mappingTask",
  .stack_size = 2048,
  .priority = (osPriority_t) osPriorityLow,
};

const osThreadAttr_t safetyTask_attributes = {
  .name = "safetyTask",
  .stack_size = 512,
  .priority = (osPriority_t) osPriorityAboveNormal,
};

const osThreadAttr_t telemetryTask_attributes = {
  .name = "telemetryTask",
  .stack_size = 768,
  .priority = (osPriority_t) osPriorityLow,
};

const osMutexAttr_t odomMutex_attributes = {
  .name = "odomMutex",
};

const osMutexAttr_t localizationMutex_attributes = {
  .name = "localizationMutex",
};

const osMutexAttr_t gridMutex_attributes = {
  .name = "gridMutex",
};

const osMutexAttr_t pidMutex_attributes = {
  .name = "pidMutex",
};

const osMutexAttr_t controlMutex_attributes = {
  .name = "controlMutex",
};

osThreadId_t navigationTaskHandle;

const osThreadAttr_t navigationTask_attributes = {
    .name = "navigationTask",
    .stack_size = 1024,
    .priority = (osPriority_t) osPriorityBelowNormal,
};

/* USER CODE END ThreadDefs */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */
/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
  memset(g_lidarScanBuf, 0, sizeof(g_lidarScanBuf));
  memset(&g_runtimeStats, 0, sizeof(g_runtimeStats));
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  g_odomMutex = osMutexNew(&odomMutex_attributes);
  g_localizationMutex = osMutexNew(&localizationMutex_attributes);
  g_gridMutex = osMutexNew(&gridMutex_attributes);
  g_pidMutex = osMutexNew(&pidMutex_attributes);
  g_controlMutex = osMutexNew(&controlMutex_attributes);
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  g_controlTickSem = osSemaphoreNew(1U, 0U, NULL);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  g_lidarBlockQueue = osMessageQueueNew(LIDAR_DMA_BLOCK_QUEUE_LENGTH, sizeof(LidarDmaBlockMsg_t), NULL);
  g_lidarResultQueue = osMessageQueueNew(LIDAR_SCAN_BUFFER_COUNT, sizeof(LidarScanMsg_t), NULL);
  g_localizedScanQueue = osMessageQueueNew(LIDAR_SCAN_BUFFER_COUNT, sizeof(LidarScanMsg_t), NULL);
  g_lidarFreeQueue = osMessageQueueNew(LIDAR_SCAN_BUFFER_COUNT, sizeof(uint8_t), NULL);

  if (g_lidarFreeQueue != NULL) {
    for (uint8_t idx = 0U; idx < LIDAR_SCAN_BUFFER_COUNT; ++idx) {
      (void)osMessageQueuePut(g_lidarFreeQueue, &idx, 0U, 0U);
    }
  }
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  controlTaskHandle = osThreadNew(StartControlTask, NULL, &controlTask_attributes);
  lidarParseTaskHandle = osThreadNew(StartLiDARParseTask, NULL, &lidarParseTask_attributes);
  localizationTaskHandle = osThreadNew(StartLocalizationTask, NULL, &localizationTask_attributes);
  mappingTaskHandle = osThreadNew(StartMappingTask, NULL, &mappingTask_attributes);
  safetyTaskHandle = osThreadNew(StartSafetyTask, NULL, &safetyTask_attributes);
  telemetryTaskHandle = osThreadNew(StartTelemetryTask, NULL, &telemetryTask_attributes);
  navigationTaskHandle = osThreadNew(StartNavigationTask, NULL, &navigationTask_attributes);
  if (HAL_TIM_Base_Start_IT(&htim4) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* USER CODE END RTOS_EVENTS */
}

void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  (void)argument;

  /*
   * Start the lidar stream after the RTOS objects and worker threads exist.
   * RPLIDAR_StartRaw() resets the lidar pipeline and arms USART6 DMA, so
   * calling it here avoids starting the stream before the FreeRTOS queues
   * used by the parser/localization/mapping pipeline are ready.
   */
  osDelay(50U);
  RPLIDAR_StartRaw();

  for (;;) {
    osDelay(1000U);
  }
  /* USER CODE END StartDefaultTask */
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

osStatus_t Freertos_NotifyControlTickFromISR(void)
{
  osStatus_t status;

  if (g_controlTickSem == NULL) {
    return osErrorResource;
  }

  status = osSemaphoreRelease(g_controlTickSem);
  if (status == osErrorResource) {
    g_runtimeStats.control_tick_overruns++;
  }

  return status;
}

osStatus_t Freertos_SubmitLidarBlockFromISR(uint16_t offset, uint16_t length)
{
  LidarDmaBlockMsg_t msg;
  osStatus_t status;

  if ((g_lidarBlockQueue == NULL) || (length == 0U)) {
    return osErrorResource;
  }

  msg.offset = offset;
  msg.length = length;
  msg.sequence = ++g_lidarBlockSequence;

  status = osMessageQueuePut(g_lidarBlockQueue, &msg, 0U, 0U);
  if (status == osOK) {
    g_runtimeStats.lidar_dma_block_count++;
  } else {
    lidar_raw_overflow = 1U;
    overflow_count++;
    g_runtimeStats.lidar_dma_drop_count++;
  }

  return status;
}

void Freertos_ResetLidarPipeline(void)
{
  uint8_t idx;

  memset(g_lidarScanBuf, 0, sizeof(g_lidarScanBuf));
  g_lidarBlockSequence = 0U;
  g_lidarScanSequence = 0U;

  if (osKernelGetState() != osKernelRunning) {
    return;
  }

  if (g_lidarBlockQueue != NULL) {
    (void)osMessageQueueReset(g_lidarBlockQueue);
  }

  if (g_lidarResultQueue != NULL) {
    (void)osMessageQueueReset(g_lidarResultQueue);
  }

  if (g_localizedScanQueue != NULL) {
    (void)osMessageQueueReset(g_localizedScanQueue);
  }

  if (g_lidarFreeQueue != NULL) {
    (void)osMessageQueueReset(g_lidarFreeQueue);
    for (idx = 0U; idx < LIDAR_SCAN_BUFFER_COUNT; ++idx) {
      (void)osMessageQueuePut(g_lidarFreeQueue, &idx, 0U, 0U);
    }
  }

  LocalizationTask_Reset();
  MappingTask_ResetGrid();
}

void Freertos_GetRuntimeStatsSnapshot(FreertosRuntimeStats_t *stats)
{
  if (stats != NULL) {
    *stats = g_runtimeStats;
  }
}

void StartControlTask(void *argument)
{
  const float dt = 0.01f;
  SlamPose2D_t odom_pose;

  (void)argument;
  (void)memset(&odom_pose, 0, sizeof(odom_pose));

  for (;;) {
    (void)osSemaphoreAcquire(g_controlTickSem, osWaitForever);

    MPU_update();
    encoder_update_speed();

    if (g_odomMutex != NULL) {
      (void)osMutexAcquire(g_odomMutex, osWaitForever);
    }

    Odometry_Update(dt);
    Odometry_GetPoseSnapshot(&odom_pose);
    LocalizationTask_UpdatePredictedPose(&odom_pose);

    if (g_controlMutex != NULL) {
      (void)osMutexAcquire(g_controlMutex, osWaitForever);
    }

    if (g_pidMutex != NULL) {
      (void)osMutexAcquire(g_pidMutex, osWaitForever);
    }

    if (g_control_mode == CONTROL_MODE_SPEED_TEST) {
      Control_UpdateWheelSpeedTest(dt);
    } else if (g_control_mode == CONTROL_MODE_POSITION) {
      Update_Relative_Move_PID(dt, &odom_pose);
    } else {
      Angle_Speed_Cascade_Control(odom_pose.theta_deg, base_car_speed, dt);
    }

    if (g_pidMutex != NULL) {
      (void)osMutexRelease(g_pidMutex);
    }

    if (g_controlMutex != NULL) {
      (void)osMutexRelease(g_controlMutex);
    }

    if (g_odomMutex != NULL) {
      (void)osMutexRelease(g_odomMutex);
    }

    g_runtimeStats.control_cycles++;
  }
}

void StartLiDARParseTask(void *argument)
{
  const uint8_t *dma_rx_buffer = LIDAR_GetDmaRxBuffer();
  LidarDmaBlockMsg_t block_msg;
  uint32_t latest_block_sequence;
  uint32_t block_lag;
  uint8_t write_idx = 0U;
  uint16_t byte_idx;

  (void)argument;
  memset(&block_msg, 0, sizeof(block_msg));

  (void)osMessageQueueGet(g_lidarFreeQueue, &write_idx, NULL, osWaitForever);
  g_lidarScanBuf[write_idx].point_count = 0U;

  for (;;) {
    if (osMessageQueueGet(g_lidarBlockQueue, &block_msg, NULL, osWaitForever) != osOK) {
      continue;
    }

    if ((lidar_raw_stream_active == 0U) ||
        (block_msg.length == 0U) ||
        ((uint32_t)block_msg.offset + block_msg.length > LIDAR_DMA_RX_BUFFER_SIZE)) {
      continue;
    }

    latest_block_sequence = g_lidarBlockSequence;
    block_lag = LIDAR_GetDmaBlockLag(latest_block_sequence, block_msg.sequence);
    if (block_lag > g_runtimeStats.lidar_dma_max_block_lag) {
      g_runtimeStats.lidar_dma_max_block_lag = block_lag;
    }

    if (LIDAR_IsDmaBlockStale(latest_block_sequence, block_msg.sequence) != 0U) {
      g_runtimeStats.lidar_dma_stale_block_count++;
      g_runtimeStats.lidar_parser_resync_count++;
      g_lidarScanBuf[write_idx].point_count = 0U;
      LIDAR_ResetParserState();
      continue;
    }

    LIDAR_ConsumePendingPacket(&g_lidarScanBuf[write_idx]);

    for (byte_idx = 0U; byte_idx < block_msg.length; ++byte_idx) {
      if (LIDAR_ConsumeByte(dma_rx_buffer[block_msg.offset + byte_idx],
                            &g_lidarScanBuf[write_idx]) == LIDAR_SCAN_COMPLETE) {
        LidarScanMsg_t scan_msg;

        g_runtimeStats.lidar_scan_complete_count++;
        scan_msg.scan_index = write_idx;
        scan_msg.point_count = g_lidarScanBuf[write_idx].point_count;
        scan_msg.scan_sequence = ++g_lidarScanSequence;
        ScanPreprocess_Analyze(g_lidarScanBuf[write_idx].points,
                               g_lidarScanBuf[write_idx].point_count,
                               &scan_msg.quality);
        if (g_odomMutex != NULL) {
          (void)osMutexAcquire(g_odomMutex, osWaitForever);
        }
        Odometry_GetPoseSnapshot(&scan_msg.pose_snapshot);
        if (g_odomMutex != NULL) {
          (void)osMutexRelease(g_odomMutex);
        }
        g_runtimeStats.last_scan_raw_point_count = scan_msg.quality.raw_point_count;
        g_runtimeStats.last_scan_usable_point_count = scan_msg.quality.usable_point_count;
        g_runtimeStats.last_scan_rejected_range_count = scan_msg.quality.rejected_range_count;
        g_runtimeStats.last_scan_rejected_quality_count = scan_msg.quality.rejected_quality_count;
        g_runtimeStats.last_scan_min_distance_mm = scan_msg.quality.min_distance_mm;
        g_runtimeStats.last_scan_max_distance_mm = scan_msg.quality.max_distance_mm;

        (void)osMessageQueuePut(g_lidarResultQueue, &scan_msg, 0U, osWaitForever);
        (void)osMessageQueueGet(g_lidarFreeQueue, &write_idx, NULL, osWaitForever);
        g_lidarScanBuf[write_idx].point_count = 0U;
        LIDAR_ConsumePendingPacket(&g_lidarScanBuf[write_idx]);
      }
    }

    osThreadYield();
  }
}

void StartSafetyTask(void *argument)
{
  (void)argument;

  for (;;) {
    g_runtimeStats.free_heap_bytes = (uint32_t)xPortGetFreeHeapSize();
    g_runtimeStats.min_ever_free_heap_bytes = (uint32_t)xPortGetMinimumEverFreeHeapSize();
    g_runtimeStats.lidar_dma_drop_count = overflow_count;
    g_runtimeStats.default_task_stack_free_bytes =
        (uint32_t)uxTaskGetStackHighWaterMark(defaultTaskHandle) * sizeof(StackType_t);
    g_runtimeStats.control_task_stack_free_bytes =
        (uint32_t)uxTaskGetStackHighWaterMark(controlTaskHandle) * sizeof(StackType_t);
    g_runtimeStats.lidar_task_stack_free_bytes =
        (uint32_t)uxTaskGetStackHighWaterMark(lidarParseTaskHandle) * sizeof(StackType_t);
    g_runtimeStats.localization_task_stack_free_bytes =
        (uint32_t)uxTaskGetStackHighWaterMark(localizationTaskHandle) * sizeof(StackType_t);
    g_runtimeStats.mapping_task_stack_free_bytes =
        (uint32_t)uxTaskGetStackHighWaterMark(mappingTaskHandle) * sizeof(StackType_t);
    g_runtimeStats.safety_task_stack_free_bytes =
        (uint32_t)uxTaskGetStackHighWaterMark(safetyTaskHandle) * sizeof(StackType_t);

    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    osDelay(100U);
  }
}

/* USER CODE END Application */
