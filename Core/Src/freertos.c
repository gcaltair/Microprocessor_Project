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
osMessageQueueId_t g_lidarReadyQueue = NULL;
osMessageQueueId_t g_lidarFreeQueue = NULL;
osMessageQueueId_t g_cmdQueue = NULL;

osMutexId_t g_odomMutex = NULL;
osMutexId_t g_pidMutex = NULL;
osMutexId_t g_controlMutex = NULL;

LidarScanBuffer_t g_lidarScanBuf[LIDAR_SCAN_BUFFER_COUNT];
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

const osMutexAttr_t pidMutex_attributes = {
  .name = "pidMutex",
};

const osMutexAttr_t controlMutex_attributes = {
  .name = "controlMutex",
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
  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  g_odomMutex = osMutexNew(&odomMutex_attributes);
  g_pidMutex = osMutexNew(&pidMutex_attributes);
  g_controlMutex = osMutexNew(&controlMutex_attributes);
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  g_controlTickSem = osSemaphoreNew(1U, 0U, NULL);
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  g_lidarReadyQueue = osMessageQueueNew(LIDAR_SCAN_BUFFER_COUNT, sizeof(uint8_t), NULL);
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
  commTaskHandle = osThreadNew(StartCommTask, NULL, &commTask_attributes);
  safetyTaskHandle = osThreadNew(StartSafetyTask, NULL, &safetyTask_attributes);

  if (HAL_TIM_Base_Start_IT(&htim4) != HAL_OK) {
    Error_Handler();
  }
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* USER CODE END RTOS_EVENTS */

}

osStatus_t Freertos_NotifyControlTickFromISR(void)
{
  if (g_controlTickSem == NULL) {
    return osErrorResource;
  }

  return osSemaphoreRelease(g_controlTickSem);
}

osStatus_t Freertos_SubmitCommandFromISR(const uint8_t *data, uint16_t len)
{
  CmdMsg_t msg;

  if ((data == NULL) || (g_cmdQueue == NULL)) {
    return osErrorParameter;
  }

  memset(&msg, 0, sizeof(msg));
  if (len > CMD_MSG_BUFFER_SIZE) {
    len = CMD_MSG_BUFFER_SIZE;
  }

  memcpy(msg.data, data, len);
  msg.len = len;

  return osMessageQueuePut(g_cmdQueue, &msg, 0U, 0U);
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
  (void)argument;

  for(;;)
  {
    osDelay(1000U);
  }
  /* USER CODE END StartDefaultTask */
}

void StartControlTask(void *argument)
{
  const float dt = 0.01f;

  (void)argument;

  for (;;) {
    (void)osSemaphoreAcquire(g_controlTickSem, osWaitForever);

    MPU_update();
    encoder_update_speed();

    if (g_odomMutex != NULL) {
      (void)osMutexAcquire(g_odomMutex, osWaitForever);
    }

    Odometry_Update(dt);

    if (g_controlMutex != NULL) {
      (void)osMutexAcquire(g_controlMutex, osWaitForever);
    }

    if (g_pidMutex != NULL) {
      (void)osMutexAcquire(g_pidMutex, osWaitForever);
    }

    if (g_control_mode == CONTROL_MODE_POSITION) {
      Update_Relative_Move_PID(dt);
    } else {
      Angle_Speed_Cascade_Control(g_th_continuous, base_car_speed, dt);
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
  }
}

void StartLiDARParseTask(void *argument)
{
  uint8_t write_idx = 0U;

  (void)argument;

  (void)osMessageQueueGet(g_lidarFreeQueue, &write_idx, NULL, osWaitForever);
  g_lidarScanBuf[write_idx].point_count = 0U;

  for (;;) {
    if (LIDAR_ParseStep(&g_lidarScanBuf[write_idx]) == LIDAR_SCAN_COMPLETE) {
      (void)osMessageQueuePut(g_lidarReadyQueue, &write_idx, 0U, osWaitForever);
      (void)osMessageQueueGet(g_lidarFreeQueue, &write_idx, NULL, osWaitForever);
      g_lidarScanBuf[write_idx].point_count = 0U;
    }

    osDelay(1U);
  }
}

void StartCommTask(void *argument)
{
  uint8_t ready_idx = 0U;
  uint8_t ready_scan_owned = 0U;
  CmdMsg_t cmd_msg;

  (void)argument;
  memset(&cmd_msg, 0, sizeof(cmd_msg));

  for (;;) {
    if ((ready_scan_owned == 0U) &&
        (osMessageQueueGet(g_lidarReadyQueue, &ready_idx, NULL, 0U) == osOK)) {
      ready_scan_owned = 1U;
    }

    if (ready_scan_owned != 0U) {
      HAL_StatusTypeDef tx_status;

      if (g_odomMutex != NULL) {
        (void)osMutexAcquire(g_odomMutex, osWaitForever);
      }

      tx_status = send_binary_packaged_data_from_buffer(&g_lidarScanBuf[ready_idx]);

      if (g_odomMutex != NULL) {
        (void)osMutexRelease(g_odomMutex);
      }

      if (tx_status != HAL_BUSY) {
        g_lidarScanBuf[ready_idx].point_count = 0U;
        (void)osMessageQueuePut(g_lidarFreeQueue, &ready_idx, 0U, osWaitForever);
        ready_scan_owned = 0U;
      }
    }

    if (osMessageQueueGet(g_cmdQueue, &cmd_msg, NULL, 0U) == osOK) {
      process_command(cmd_msg.data, cmd_msg.len);
    }

    osDelay(1U);
  }
}

void StartSafetyTask(void *argument)
{
  (void)argument;

  for (;;) {
    (void)uxTaskGetStackHighWaterMark(NULL);
    (void)xPortGetFreeHeapSize();

    HAL_GPIO_TogglePin(LD2_GPIO_Port, LD2_Pin);
    osDelay(100U);
  }
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if (htim->Instance == TIM5) {
    HAL_IncTick();
  }

  if (htim->Instance == TIM4) {
    (void)Freertos_NotifyControlTickFromISR();
  }
}

/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */

/* USER CODE END Application */
