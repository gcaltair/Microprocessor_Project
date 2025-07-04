//
// Created by G on 25-7-4.
//

#ifndef FINAL_FINA_LIDAR_H
#define FINAL_FINA_LIDAR_H
// 请根据您的STM32型号更改此处的头文件
#include "stm32f446xx.h"
#include "usart.h"
#include <stdint.h>

// --- 协议常量定义 ---

// 命令
#define RPLIDAR_CMD_STOP                 0x25       //
#define RPLIDAR_CMD_SCAN                 0x20       //
#define RPLIDAR_CMD_EXPRESS_SCAN         0x82       //

// 帧格式
#define RPLIDAR_FRAME_HEADER_1           0x0A       //
#define RPLIDAR_FRAME_HEADER_2           0x05       //
#define RPLIDAR_POINTS_PER_FRAME         40         //
#define RPLIDAR_STANDARD_FRAME_SIZE      5          //
#define RPLIDAR_EXPRESS_FRAME_SIZE       84         //


// 雷达工作模式枚举
typedef enum {
    LIDAR_MODE_NONE,
    LIDAR_MODE_STANDARD,
    LIDAR_MODE_EXPRESS
} LidarMode_t;

// 雷达点结构体定义
typedef struct
{
    uint16_t angle;     // 角度，单位：度 * 100
    uint16_t distance;  // 距离，单位：毫米
} LaserPointTypeDef; //

// --- 外部变量与函数声明 ---

// 外部可访问的雷达数据点数组
// 雷达一圈大概有250-300个点，可以根据实际情况调整大小
extern LaserPointTypeDef g_lidar_points[300];
// 一圈扫描完成标志，主循环可以查询此标志
extern volatile uint8_t g_lidar_scan_ready;

/**
 * @brief  初始化雷达驱动
 * @param  huart: 指向雷达所用UART句柄的指针 (例如 &huart6)
 * @retval 无
 */
void Lidar_Init(UART_HandleTypeDef *huart);

/**
 * @brief  启动扫描 (密实模式 Express)
 * @retval 无
 */
void Lidar_StartScan_Express(void);

/**
 * @brief  启动扫描 (标准模式 Standard)
 * @retval 无
 */
void Lidar_StartScan_Standard(void);

/**
 * @brief  发送停止扫描命令
 * @retval 无
 */
void Lidar_StopScan(void);


#endif //FINAL_FINA_LIDAR_H
