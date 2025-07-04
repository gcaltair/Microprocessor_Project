/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
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
#include "main.h"
#include "dma.h"
#include "i2c.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "motor.h"
#include "hc04.h"
#include "encoder.h"
#include "MPU6500.h"
#include "lidar.h"
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

/* USER CODE BEGIN PV */
//uint8_t speed = 50; // 默认速度为50%
uint8_t buffer[100]; // 单字节接收缓冲区
uint8_t rxData[100]; // 接收数据缓冲
uint16_t rxIndex = 0; // 接收数据索引


/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
//void HAL_UART_RxCpltCallback(UART_HandleTypeDef * huart)
//{
//    if(huart->Instance == UART5)
//    {
//        HAL_UART_Transmit_DMA(&huart5,buffer,2);
//        HAL_UART_Receive_DMA(&huart5, buffer, 2);
//    }
//}
//void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t size)
//{
//    if(huart==&huart5)
//    {
//        HAL_UART_Transmit_DMA(&huart5,buffer,size);
//        HAL_UARTEx_ReceiveToIdle_DMA(&huart5,buffer,sizeof(buffer)); //一次能接收的最大
//        __HAL_DMA_DISABLE_IT(&hdma_uart5_rx,DMA_IT_HT); //关闭传输过半中断
//    }
//}
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */
  AccelData accelData;
  GyroData gyroData;
  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */
  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_DMA_Init();
  MX_TIM3_Init();
  MX_UART5_Init();
  MX_I2C1_Init();
  MX_TIM1_Init();
  MX_TIM2_Init();
  MX_SPI2_Init();
  MX_USART6_UART_Init();
  /* USER CODE BEGIN 2 */

  Motor_Init();// 初始化电机
  hc04_init();// 蓝牙初始化
  encoder_init(); //编码器初始化
  MPU6500_Init();
  SystemClock_Config();
    //Lidar_Init(&huart6);
  RPLIDAR_Init();
  
  // 开始工作
  HAL_GPIO_WritePin(GPIOA, LD2_Pin, GPIO_PIN_RESET);

    //Lidar_StartScan_Standard();
    //Car_Forward(100);
   // Lidar_StartScan_Express();
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */


      uint32_t currentTick = HAL_GetTick();
//
      if(encoder_UpdateSpeed_SysTick())
      {
          MPU6500_Read_Accel(&accelData);
          MPU6500_PrintAccelData(&accelData);
          MPU6500_Read_Gyro(&gyroData);
          MPU6500_PrintGyroData(&gyroData);

          char msg[150]; // 增加缓冲区大小，确保有足够空间
          int32_t speedA = encoderA_GetSpeed();
          int32_t speedB = encoderB_GetSpeed();
          // 输出详细信息，包括系统tick计数
          sprintf(msg, "Tick:%lu SpeedA:%d SpeedB:%d\r\n",
                 currentTick, speedA, speedB);
//          // 发送数据
            transmit(msg);
      }
//      // 检查雷达是否完成了一圈扫描
//      if (g_lidar_scan_ready == 1)
//      {
//          // 立即清除标志位，为下一圈数据做准备
//          g_lidar_scan_ready = 0;
//
//          // 创建一个用于发送的数据包缓冲区
//          uint8_t tx_buffer[8];
//
//          // 遍历所有有效的雷达数据点
//          for (int i = 0; i < 300; i++)
//          {
//              // 只发送距离大于0的有效数据点
//              if (g_lidar_points[i].distance > 0)
//              {
//                  // 1. 按照我们定义的协议打包数据
//                  tx_buffer[0] = 0x5A; // 帧头1
//                  tx_buffer[1] = 0xA5; // 帧头2
//
//                  // 角度 (高字节在前)
//                  tx_buffer[2] = (g_lidar_points[i].angle >> 8) & 0xFF;
//                  tx_buffer[3] = g_lidar_points[i].angle & 0xFF;
//
//                  // 距离 (高字节在前)
//                  tx_buffer[4] = (g_lidar_points[i].distance >> 8) & 0xFF;
//                  tx_buffer[5] = g_lidar_points[i].distance & 0xFF;
//
//                  // 计算校验和
//                  tx_buffer[6] = tx_buffer[0] ^ tx_buffer[1] ^ tx_buffer[2] ^ tx_buffer[3] ^ tx_buffer[4] ^ tx_buffer[5];
//
//                  tx_buffer[7] = 0xFF; // 帧尾
//
//                  // 2. 通过 UART5 将数据包发送出去
//                  HAL_UART_Transmit(&huart5, tx_buffer, sizeof(tx_buffer), 10); // 10ms超时
//              }
//          }
//      }


      
      // 检查是否需要自动停车
      check_auto_stop();
      
      // 加入短暂延时，避免CPU过载，但不要影响测速定时精度
      HAL_Delay(10);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE3);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 16;
  RCC_OscInitStruct.PLL.PLLN = 336;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV4;
  RCC_OscInitStruct.PLL.PLLQ = 2;
  RCC_OscInitStruct.PLL.PLLR = 2;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}

#ifdef  USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
