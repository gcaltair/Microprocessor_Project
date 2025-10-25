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


typedef struct {
    float   angle_deg;    // 角度 (单位: 度)
    float   distance_mm;  // 距离 (单位: 毫米)
    uint8_t quality;      // 信号质量
} LidarPoint_t;

typedef enum {
    STATE_WAIT_START_BIT,  // 等待数据包的起始位 (S=1)
    STATE_RECEIVE_DATA     // 接收数据包的后续4个字节
} ParserState_t;

// 1. 定义数据包的包头，用于帧同步
#define PROTOCOL_HEADER_1 0xA5
#define PROTOCOL_HEADER_2 0x5A

// 2. 为了避免内存对齐问题，强制编译器按1字节对齐
#pragma pack(push, 1)

// 3. 定义数据包头部结构体
typedef struct {
    uint8_t header1;        // 固定为 PROTOCOL_HEADER_1
    uint8_t header2;        // 固定为 PROTOCOL_HEADER_2
    uint16_t payload_len;   // 负载长度 (从 odom_data 到 lidar_points_data 的总字节数)
} BinaryPacketHeader_t;

// 4. 定义里程计数据结构体
typedef struct {
    float x;
    float y;
    float theta_continuous;
} OdometryData_t;

// 5. 定义单个雷达点的数据结构体 (优化版)
//    我们不传输浮点数，而是传输放大后的整数，以节省空间和解析时间
typedef struct {
    uint16_t angle_x100;    // 角度 * 100, 例如 123.45 度 -> 12345
    uint16_t distance_mm;   // 距离(毫米)
} LidarPointPacked_t;

// 恢复默认的内存对齐设置
#pragma pack(pop)

// 接口
void RPLIDAR_Init(void);
void RPLIDAR_RequestScan(void);
void Lidar_StopScan(void);

// 原始透传控制
void RPLIDAR_StartRaw(void);
void RPLIDAR_StopRaw(void);

// 任务：发送环形缓冲内容
void RPLIDAR_RawTask(void);

//DMA传输相关
void LIDAR_ParseTask(void);
void send_packaged_data(void);
void send_binary_packaged_data(void);
#endif
