/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    iwdg.c
  * @brief   Independent watchdog configuration.
  ******************************************************************************
  */
/* USER CODE END Header */

#include "iwdg.h"

IWDG_HandleTypeDef hiwdg;

void MX_IWDG_Init(void)
{
  hiwdg.Instance = IWDG;
  hiwdg.Init.Prescaler = IWDG_PRESCALER_64;
  hiwdg.Init.Reload = 2000;
  if (HAL_IWDG_Init(&hiwdg) != HAL_OK)
  {
    Error_Handler();
  }
}
