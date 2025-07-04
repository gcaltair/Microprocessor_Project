#ifndef __RPLIDAR_H
#define __RPLIDAR_H

#include "usart.h"
#include "stdint.h"

// --- 协议常量定义 ---

// 命令
#define RPLIDAR_CMD_STOP                 0x25
#define RPLIDAR_CMD_SCAN                 0x20
#define RPLIDAR_CMD_EXPRESS_SCAN         0x82
// 帧格式
#define RPLIDAR_FRAME_HEADER_1           0x0A
#define RPLIDAR_FRAME_HEADER_2           0x05
#define RPLIDAR_POINTS_PER_FRAME         40
#define RPLIDAR_STANDARD_FRAME_SIZE      5
#define RPLIDAR_EXPRESS_FRAME_SIZE       84
extern uint8_t rplidar_rx_byte;
void RPLIDAR_Init(void);
void RPLIDAR_ProcessByte(uint8_t byte);
void RPLIDAR_RequestScan(void);
void Lidar_StopScan(void);
#endif

