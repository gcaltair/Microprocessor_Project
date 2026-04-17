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

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include <string.h>

#include "freertos_app.h"
#include "hc04.h"
#include "localization_task.h"
#include "mapping_task.h"
#include "navigation_task.h"
#include "scan_preprocess.h"
#include "system.h"
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
osMessageQueueId_t g_lidarTxQueue = NULL;
osMessageQueueId_t g_lidarFreeQueue = NULL;
osMessageQueueId_t g_cmdQueue = NULL;

osMutexId_t g_odomMutex = NULL;
osMutexId_t g_localizationMutex = NULL;
osMutexId_t g_gridMutex = NULL;
osMutexId_t g_pidMutex = NULL;
osMutexId_t g_controlMutex = NULL;

LidarScanBuffer_t g_lidarScanBuf[LIDAR_SCAN_BUFFER_COUNT];
FreertosRuntimeStats_t g_runtimeStats;
static volatile uint32_t g_lidarBlockSequence = 0U;
static uint32_t g_lidarScanSequence = 0U;
static volatile uint8_t g_lidarBinaryTxEnabled = 0U;
static volatile uint8_t g_telemetryStreamingEnabled = 0U;
static volatile uint8_t g_telemetryScanStreamingEnabled = 0U;
/* USER CODE END Variables */
/* Definitions for defaultTask */
osThreadId_t defaultTaskHandle;
const osThreadAttr_t defaultTask_attributes = {
  .name = "defaultTask",
  .stack_size = 4096,
  .priority = (osPriority_t) osPriorityNormal,
};

/* USER CODE BEGIN ThreadDefs */
osThreadId_t controlTaskHandle;
osThreadId_t lidarParseTaskHandle;
osThreadId_t localizationTaskHandle;
osThreadId_t mappingTaskHandle;
osThreadId_t commTaskHandle;
osThreadId_t safetyTaskHandle;

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

const osThreadAttr_t commTask_attributes = {
  .name = "commTask",
  .stack_size = 1024,
  .priority = (osPriority_t) osPriorityLow,
};

const osThreadAttr_t safetyTask_attributes = {
  .name = "safetyTask",
  .stack_size = 512,
  .priority = (osPriority_t) osPriorityBelowNormal,
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
/* USER CODE END ThreadDefs */

/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

#pragma pack(push, 1)
typedef struct {
  float x_m;
  float y_m;
  float theta_deg;
} TelemetryPosePayload_t;

typedef struct {
  uint32_t timestamp_ms;
  TelemetryPosePayload_t odom_pose;
  TelemetryPosePayload_t estimated_pose;
  TelemetryPosePayload_t control_pose;
  uint8_t control_mode;
  uint8_t move_state;
  uint8_t localization_mode;
  uint8_t navigation_state;
  float base_speed_mps;
  uint16_t nav_path_length;
  uint16_t nav_current_waypoint_index;
  int16_t nav_goal_x;
  int16_t nav_goal_y;
  float nav_last_waypoint_distance_m;
  uint32_t control_cycles;
  uint32_t cmd_rx_count;
  uint32_t cmd_drop_count;
  uint32_t lidar_scan_complete_count;
  uint32_t lidar_dma_drop_count;
  uint32_t free_heap_bytes;
  uint32_t min_ever_free_heap_bytes;
  uint16_t localization_inliers;
  float localization_fitness_m;
  uint8_t map_update_active;
  uint8_t map_last_skip_reason;
  uint8_t reserved0;
  uint8_t reserved1;
  uint32_t mapping_skipped_turning_count;
  uint32_t mapping_skipped_settle_count;
  uint32_t mapping_skipped_quality_count;
  uint8_t lidar_active;
  uint8_t lidar_binary_enabled;
  uint8_t telemetry_enabled;
  uint8_t scan_stream_enabled;
} TelemetryStatusPayload_t;

typedef struct {
  uint16_t width_cells;
  uint16_t height_cells;
  float resolution_m_per_cell;
  float origin_x_m;
  float origin_y_m;
} TelemetryMapMetaPayload_t;

typedef struct {
  uint16_t width_cells;
  uint16_t height_cells;
  uint16_t row_offset;
  uint16_t row_count;
} TelemetryMapDataHeader_t;

typedef struct {
  uint8_t navigation_state;
  uint8_t reserved;
  uint16_t current_waypoint_index;
  uint16_t path_length;
  int16_t goal_x;
  int16_t goal_y;
  int16_t current_waypoint_x;
  int16_t current_waypoint_y;
  float last_waypoint_distance_m;
} TelemetryPathHeader_t;

typedef struct {
  uint16_t angle_x100;
  uint16_t distance_mm;
} TelemetryScanPointPayload_t;

typedef struct {
  uint32_t scan_sequence;
  TelemetryPosePayload_t pose_snapshot;
  TelemetryPosePayload_t corrected_pose;
  uint16_t point_count;
} TelemetryScanHeader_t;
#pragma pack(pop)

#define TELEMETRY_SCAN_MAX_POINTS         128U

static void telemetry_fill_pose(TelemetryPosePayload_t *payload, const SlamPose2D_t *pose);
static void telemetry_send_status_frame(void);
static void telemetry_send_map_frames(void);
static void telemetry_send_path_frame(void);
static void telemetry_send_scan_frame(const LidarScanMsg_t *scan_msg);
static uint32_t telemetry_compose_nav_signature(const NavigationTaskStats_t *stats);

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
  g_lidarTxQueue = osMessageQueueNew(LIDAR_SCAN_BUFFER_COUNT, sizeof(LidarScanMsg_t), NULL);
  g_lidarFreeQueue = osMessageQueueNew(LIDAR_SCAN_BUFFER_COUNT, sizeof(uint8_t), NULL);
  g_cmdQueue = osMessageQueueNew(8U, sizeof(CmdMsg_t), NULL);

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
  commTaskHandle = osThreadNew(StartCommTask, NULL, &commTask_attributes);
  safetyTaskHandle = osThreadNew(StartSafetyTask, NULL, &safetyTask_attributes);

  if (HAL_TIM_Base_Start_IT(&htim4) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
void StartDefaultTask(void *argument)
{
  /* USER CODE BEGIN StartDefaultTask */
  CmdMsg_t cmd_msg;

  (void)argument;
  memset(&cmd_msg, 0, sizeof(cmd_msg));

  for(;;)
  {
    if (osMessageQueueGet(g_cmdQueue, &cmd_msg, NULL, osWaitForever) == osOK) {
      process_command(cmd_msg.data, cmd_msg.len);
    }
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

osStatus_t Freertos_SubmitCommandFromISR(const uint8_t *data, uint16_t len)
{
  CmdMsg_t msg;
  osStatus_t status;

  if ((data == NULL) || (g_cmdQueue == NULL)) {
    return osErrorParameter;
  }

  memset(&msg, 0, sizeof(msg));
  if (len > CMD_MSG_BUFFER_SIZE) {
    len = CMD_MSG_BUFFER_SIZE;
  }

  memcpy(msg.data, data, len);
  msg.len = len;

  status = osMessageQueuePut(g_cmdQueue, &msg, 0U, 0U);
  if (status == osOK) {
    g_runtimeStats.cmd_rx_count++;
  } else {
    g_runtimeStats.cmd_drop_count++;
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

  if (g_lidarTxQueue != NULL) {
    (void)osMessageQueueReset(g_lidarTxQueue);
  }

  if (g_lidarFreeQueue != NULL) {
    (void)osMessageQueueReset(g_lidarFreeQueue);
    for (idx = 0U; idx < LIDAR_SCAN_BUFFER_COUNT; ++idx) {
      (void)osMessageQueuePut(g_lidarFreeQueue, &idx, 0U, 0U);
    }
  }

  LocalizationTask_Reset();
  MappingTask_ResetGrid();
  NavigationTask_Reset();
}

void Freertos_GetRuntimeStatsSnapshot(FreertosRuntimeStats_t *stats)
{
  if (stats != NULL) {
    *stats = g_runtimeStats;
  }
}

void Freertos_RecordBluetoothTxWait(void)
{
  g_runtimeStats.bt_tx_wait_count++;
}

void Freertos_SetLidarBinaryTxEnabled(uint8_t enabled)
{
  g_lidarBinaryTxEnabled = (enabled != 0U) ? 1U : 0U;
}

uint8_t Freertos_GetLidarBinaryTxEnabled(void)
{
  return g_lidarBinaryTxEnabled;
}

void Freertos_SetTelemetryStreamingEnabled(uint8_t enabled)
{
  g_telemetryStreamingEnabled = (enabled != 0U) ? 1U : 0U;
  if (g_telemetryStreamingEnabled == 0U) {
    g_telemetryScanStreamingEnabled = 0U;
  }
}

uint8_t Freertos_GetTelemetryStreamingEnabled(void)
{
  return g_telemetryStreamingEnabled;
}

void Freertos_SetTelemetryScanStreamingEnabled(uint8_t enabled)
{
  g_telemetryScanStreamingEnabled = (enabled != 0U) ? 1U : 0U;
  if (g_telemetryScanStreamingEnabled != 0U) {
    g_telemetryStreamingEnabled = 1U;
  }
}

uint8_t Freertos_GetTelemetryScanStreamingEnabled(void)
{
  return g_telemetryScanStreamingEnabled;
}

static void telemetry_fill_pose(TelemetryPosePayload_t *payload, const SlamPose2D_t *pose)
{
  if ((payload == NULL) || (pose == NULL)) {
    return;
  }

  payload->x_m = pose->x_m;
  payload->y_m = pose->y_m;
  payload->theta_deg = pose->theta_deg;
}

static void telemetry_send_status_frame(void)
{
  TelemetryStatusPayload_t payload;
  SlamPose2D_t odom_pose;
  SlamPose2D_t estimated_pose;
  SlamPose2D_t control_pose;
  FreertosRuntimeStats_t runtime_stats;
  LocalizationTaskStats_t localization_stats;
  NavigationTaskStats_t navigation_stats;
  MappingTaskStats_t mapping_stats;

  (void)memset(&payload, 0, sizeof(payload));
  (void)memset(&odom_pose, 0, sizeof(odom_pose));
  (void)memset(&estimated_pose, 0, sizeof(estimated_pose));
  (void)memset(&control_pose, 0, sizeof(control_pose));

  if (g_odomMutex != NULL) {
    (void)osMutexAcquire(g_odomMutex, osWaitForever);
  }
  Odometry_GetPoseSnapshot(&odom_pose);
  if (g_odomMutex != NULL) {
    (void)osMutexRelease(g_odomMutex);
  }

  LocalizationTask_GetEstimatedPoseSnapshot(&estimated_pose);
  LocalizationTask_GetControlPoseSnapshot(&control_pose);
  Freertos_GetRuntimeStatsSnapshot(&runtime_stats);
  LocalizationTask_GetStatsSnapshot(&localization_stats);
  NavigationTask_GetStatsSnapshot(&navigation_stats);
  MappingTask_GetStatsSnapshot(&mapping_stats);

  payload.timestamp_ms = HAL_GetTick();
  telemetry_fill_pose(&payload.odom_pose, &odom_pose);
  telemetry_fill_pose(&payload.estimated_pose, &estimated_pose);
  telemetry_fill_pose(&payload.control_pose, &control_pose);
  payload.control_mode = (uint8_t)g_control_mode;
  payload.move_state = (uint8_t)g_relative_move_state;
  payload.localization_mode = (uint8_t)localization_stats.last_mode;
  payload.navigation_state = (uint8_t)navigation_stats.state;
  payload.base_speed_mps = base_car_speed;
  payload.nav_path_length = navigation_stats.path_length;
  payload.nav_current_waypoint_index = navigation_stats.current_waypoint_index;
  payload.nav_goal_x = navigation_stats.goal_cell.x;
  payload.nav_goal_y = navigation_stats.goal_cell.y;
  payload.nav_last_waypoint_distance_m = navigation_stats.last_waypoint_distance_m;
  payload.control_cycles = runtime_stats.control_cycles;
  payload.cmd_rx_count = runtime_stats.cmd_rx_count;
  payload.cmd_drop_count = runtime_stats.cmd_drop_count;
  payload.lidar_scan_complete_count = runtime_stats.lidar_scan_complete_count;
  payload.lidar_dma_drop_count = runtime_stats.lidar_dma_drop_count;
  payload.free_heap_bytes = runtime_stats.free_heap_bytes;
  payload.min_ever_free_heap_bytes = runtime_stats.min_ever_free_heap_bytes;
  payload.localization_inliers = localization_stats.last_inliers;
  payload.localization_fitness_m = localization_stats.last_fitness_m;
  payload.map_update_active = mapping_stats.map_update_active;
  payload.map_last_skip_reason = mapping_stats.last_skip_reason;
  payload.reserved0 = 0U;
  payload.reserved1 = 0U;
  payload.mapping_skipped_turning_count = mapping_stats.skipped_turning_count;
  payload.mapping_skipped_settle_count = mapping_stats.skipped_settle_count;
  payload.mapping_skipped_quality_count = mapping_stats.skipped_quality_count;
  payload.lidar_active = lidar_raw_stream_active;
  payload.lidar_binary_enabled = g_lidarBinaryTxEnabled;
  payload.telemetry_enabled = g_telemetryStreamingEnabled;
  payload.scan_stream_enabled = g_telemetryScanStreamingEnabled;

  HC04_SendTelemetryFrame(TELEMETRY_FRAME_STATUS_V2, (const uint8_t *)&payload, sizeof(payload));
}

static void telemetry_send_map_frames(void)
{
  MappingGridMeta_t meta;
  uint16_t row_offset;

  if (MappingTask_GetGridMeta(&meta) == 0U) {
    return;
  }

  {
    TelemetryMapMetaPayload_t meta_payload;

    meta_payload.width_cells = meta.width_cells;
    meta_payload.height_cells = meta.height_cells;
    meta_payload.resolution_m_per_cell = meta.resolution_m_per_cell;
    meta_payload.origin_x_m = meta.origin_x_m;
    meta_payload.origin_y_m = meta.origin_y_m;
    HC04_SendTelemetryFrame(TELEMETRY_FRAME_MAP_META_V2,
                            (const uint8_t *)&meta_payload,
                            sizeof(meta_payload));
  }

  for (row_offset = 0U; row_offset < meta.height_cells; row_offset = (uint16_t)(row_offset + 8U)) {
    static uint8_t payload[sizeof(TelemetryMapDataHeader_t) + (8U * OGM_MAX_WIDTH_CELLS)];
    TelemetryMapDataHeader_t header;
    uint16_t row_count = 8U;
    uint16_t cell_count;

    if ((uint16_t)(row_offset + row_count) > meta.height_cells) {
      row_count = (uint16_t)(meta.height_cells - row_offset);
    }

    header.width_cells = meta.width_cells;
    header.height_cells = meta.height_cells;
    header.row_offset = row_offset;
    header.row_count = row_count;
    cell_count = (uint16_t)(row_count * meta.width_cells);

    if (MappingTask_CopyGridRows(row_offset,
                                 row_count,
                                 (int8_t *)&payload[sizeof(header)],
                                 cell_count) == 0U) {
      continue;
    }

    (void)memcpy(payload, &header, sizeof(header));
    HC04_SendTelemetryFrame(TELEMETRY_FRAME_MAP_DATA_V2,
                            payload,
                            (uint16_t)(sizeof(header) + cell_count));
  }
}

static void telemetry_send_path_frame(void)
{
  NavigationTaskStats_t navigation_stats;
  static SlamGridCoord_t path_copy[64];
  TelemetryPathHeader_t header;
  uint16_t path_length = 0U;
  static uint8_t payload[sizeof(TelemetryPathHeader_t) + sizeof(path_copy)];

  NavigationTask_GetStatsSnapshot(&navigation_stats);
  if (NavigationTask_CopyPath(path_copy, 64U, &path_length) == 0U) {
    return;
  }

  header.navigation_state = (uint8_t)navigation_stats.state;
  header.reserved = 0U;
  header.current_waypoint_index = navigation_stats.current_waypoint_index;
  header.path_length = path_length;
  header.goal_x = navigation_stats.goal_cell.x;
  header.goal_y = navigation_stats.goal_cell.y;
  header.current_waypoint_x = navigation_stats.current_waypoint_cell.x;
  header.current_waypoint_y = navigation_stats.current_waypoint_cell.y;
  header.last_waypoint_distance_m = navigation_stats.last_waypoint_distance_m;

  (void)memcpy(payload, &header, sizeof(header));
  if (path_length > 0U) {
    (void)memcpy(&payload[sizeof(header)],
                 path_copy,
                 path_length * sizeof(path_copy[0]));
  }

  HC04_SendTelemetryFrame(TELEMETRY_FRAME_PATH_V2,
                          payload,
                          (uint16_t)(sizeof(header) + path_length * sizeof(path_copy[0])));
}

static void telemetry_send_scan_frame(const LidarScanMsg_t *scan_msg)
{
  const LidarScanBuffer_t *scan_buffer;
  TelemetryScanHeader_t header;
  static uint8_t payload[sizeof(TelemetryScanHeader_t) +
                         (TELEMETRY_SCAN_MAX_POINTS * sizeof(TelemetryScanPointPayload_t))];
  uint16_t idx;
  uint16_t step;
  uint16_t out_count = 0U;

  if ((scan_msg == NULL) || (scan_msg->scan_index >= LIDAR_SCAN_BUFFER_COUNT)) {
    return;
  }

  scan_buffer = &g_lidarScanBuf[scan_msg->scan_index];
  if (scan_buffer->point_count == 0U) {
    return;
  }

  header.scan_sequence = scan_msg->scan_sequence;
  telemetry_fill_pose(&header.pose_snapshot, &scan_msg->pose_snapshot);
  telemetry_fill_pose(&header.corrected_pose, &scan_msg->corrected_pose);
  step = (scan_buffer->point_count > TELEMETRY_SCAN_MAX_POINTS)
         ? (uint16_t)((scan_buffer->point_count + TELEMETRY_SCAN_MAX_POINTS - 1U) / TELEMETRY_SCAN_MAX_POINTS)
         : 1U;

  (void)memcpy(payload, &header, sizeof(header));
  for (idx = 0U; idx < scan_buffer->point_count; idx = (uint16_t)(idx + step)) {
    TelemetryScanPointPayload_t point_payload;

    point_payload.angle_x100 = (uint16_t)(scan_buffer->points[idx].angle_deg * 100.0f);
    point_payload.distance_mm = (uint16_t)scan_buffer->points[idx].distance_mm;
    (void)memcpy(&payload[sizeof(header) + out_count * sizeof(point_payload)],
                 &point_payload,
                 sizeof(point_payload));
    out_count++;
  }

  header.point_count = out_count;
  (void)memcpy(payload, &header, sizeof(header));

  HC04_SendTelemetryFrame(TELEMETRY_FRAME_SCAN_V2,
                          payload,
                          (uint16_t)(sizeof(header) +
                                     out_count * sizeof(TelemetryScanPointPayload_t)));
}

static uint32_t telemetry_compose_nav_signature(const NavigationTaskStats_t *stats)
{
  uint32_t signature;

  if (stats == NULL) {
    return 0U;
  }

  signature = ((uint32_t)stats->state & 0xFFU);
  signature ^= ((uint32_t)stats->plan_count << 8);
  signature ^= ((uint32_t)stats->completion_count << 12);
  signature ^= ((uint32_t)stats->failure_count << 16);
  signature ^= ((uint32_t)stats->path_length << 20);
  signature ^= ((uint32_t)stats->current_waypoint_index << 24);
  signature ^= (((uint32_t)(uint16_t)stats->goal_cell.x) << 3);
  signature ^= (((uint32_t)(uint16_t)stats->goal_cell.y) << 11);
  return signature;
}

void StartControlTask(void *argument)
{
  const float dt = 0.01f;
  SlamPose2D_t control_pose;
  SlamPose2D_t odom_pose;

  (void)argument;
  (void)memset(&control_pose, 0, sizeof(control_pose));
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
    LocalizationTask_GetControlPoseSnapshot(&control_pose);

    if (g_controlMutex != NULL) {
      (void)osMutexAcquire(g_controlMutex, osWaitForever);
    }

    if (g_pidMutex != NULL) {
      (void)osMutexAcquire(g_pidMutex, osWaitForever);
    }

    if (g_control_mode == CONTROL_MODE_POSITION) {
      Update_Relative_Move_PID(dt, &control_pose);
    } else {
      Angle_Speed_Cascade_Control(control_pose.theta_deg, base_car_speed, dt);
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

void StartCommTask(void *argument)
{
  LidarScanMsg_t scan_msg;
  uint32_t last_status_tick = 0U;
  uint32_t last_map_tick = 0U;
  uint32_t last_path_tick = 0U;
  uint32_t last_nav_signature = 0U;

  (void)argument;
  memset(&scan_msg, 0, sizeof(scan_msg));

  for (;;) {
    HAL_StatusTypeDef tx_status = HAL_OK;
    uint8_t have_scan_msg = 0U;
    uint32_t now_tick;
    NavigationTaskStats_t navigation_stats;
    uint32_t nav_signature;

    if (osMessageQueueGet(g_lidarTxQueue, &scan_msg, NULL, 20U) == osOK) {
      have_scan_msg = 1U;
    }

    if (have_scan_msg != 0U) {
      uint8_t free_idx = scan_msg.scan_index;

      if (Freertos_GetLidarBinaryTxEnabled() != 0U) {
        tx_status = HAL_BUSY;

        while (tx_status == HAL_BUSY) {
          if (g_odomMutex != NULL) {
            (void)osMutexAcquire(g_odomMutex, osWaitForever);
          }

          tx_status = send_binary_packaged_data_from_buffer(&g_lidarScanBuf[free_idx]);

          if (g_odomMutex != NULL) {
            (void)osMutexRelease(g_odomMutex);
          }

          if (tx_status == HAL_BUSY) {
            g_runtimeStats.lidar_tx_busy_count++;
            osDelay(1U);
          }
        }

        if (tx_status == HAL_OK) {
          g_runtimeStats.lidar_tx_count++;
        } else {
          g_runtimeStats.lidar_tx_error_count++;
        }
      }

      if (Freertos_GetTelemetryStreamingEnabled() != 0U &&
          Freertos_GetTelemetryScanStreamingEnabled() != 0U) {
        telemetry_send_scan_frame(&scan_msg);
      }

      g_lidarScanBuf[free_idx].point_count = 0U;
      (void)osMessageQueuePut(g_lidarFreeQueue, &free_idx, 0U, osWaitForever);
    }

    if (Freertos_GetTelemetryStreamingEnabled() == 0U) {
      continue;
    }

    now_tick = HAL_GetTick();
    if ((now_tick - last_status_tick) >= 200U) {
      telemetry_send_status_frame();
      last_status_tick = now_tick;
    }

    if ((now_tick - last_map_tick) >= 1000U) {
      telemetry_send_map_frames();
      last_map_tick = now_tick;
    }

    NavigationTask_GetStatsSnapshot(&navigation_stats);
    nav_signature = telemetry_compose_nav_signature(&navigation_stats);
    if ((nav_signature != last_nav_signature) ||
        ((now_tick - last_path_tick) >= 1000U)) {
      telemetry_send_path_frame();
      last_nav_signature = nav_signature;
      last_path_tick = now_tick;
    }
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
    g_runtimeStats.comm_task_stack_free_bytes =
        (uint32_t)uxTaskGetStackHighWaterMark(commTaskHandle) * sizeof(StackType_t);
    g_runtimeStats.safety_task_stack_free_bytes =
        (uint32_t)uxTaskGetStackHighWaterMark(safetyTaskHandle) * sizeof(StackType_t);

    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    NavigationTask_Service();
    HC04_ServiceStatusStream();
    osDelay(100U);
  }
}

/* USER CODE END Application */

