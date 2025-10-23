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

// 原始数据透传状态
extern volatile uint8_t lidar_raw_stream_active;
extern volatile uint8_t lidar_raw_overflow;

// 1. 定义一个结构体来存储解析后的单个雷达数据点
typedef struct {
    float angle_deg;    // 角度 (单位: 度)
    float distance_mm;  // 距离 (单位: 毫米)
    uint8_t quality;      // 信号质量
} LidarPoint_t;

// 2. 定义解析器状态机
typedef enum {
    STATE_WAIT_START,      // 等待数据包的起始字节
    STATE_RECEIVE_DATA,    // 接收数据包的后续字节
} ParserState_t;
// 接口
void RPLIDAR_Init(void);
void RPLIDAR_RequestScan(void);
void Lidar_StopScan(void);

// 原始透传控制
void RPLIDAR_StartRaw(void);
void RPLIDAR_StopRaw(void);

// 任务：发送环形缓冲内容
void RPLIDAR_RawTask(void);

void LIDAR_ParseTask(void);
void send_odom_and_lidar_data(void);

#endif
