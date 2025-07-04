#include "lidar.h"
#include <string.h>
#include "usart.h"
// --- 内部变量定义 ---

// 雷达所用的UART句柄，必须与CubeMX中配置的句柄变量名一致

#define LIDAR_UART_HANDLE (&huart6)

// 串口接收相关
static volatile uint8_t g_rx_byte;                              // 单字节接收缓冲
static uint8_t g_rx_buffer[RPLIDAR_EXPRESS_FRAME_SIZE];         // 完整数据帧缓冲 (使用最大尺寸)
static volatile uint16_t g_rx_index = 0;                        // 接收计数器
static volatile LidarMode_t g_current_mode = LIDAR_MODE_NONE;   // 当前扫描模式

// 雷达数据点全局数组
LaserPointTypeDef g_lidar_points[300];
volatile uint8_t g_lidar_scan_ready = 0;

// --- 内部函数声明 ---
static void Lidar_ProcessFrame_Express(uint8_t* frame);
static void Lidar_ProcessFrame_Standard(uint8_t* frame);


// --- 公共函数实现 ---

void Lidar_Init(UART_HandleTypeDef *huart)
{
    // 启动HAL的UART接收中断，每次只接收1个字节
    // 当接收完成时，会调用 HAL_UART_RxCpltCallback
    HAL_UART_Receive_IT(LIDAR_UART_HANDLE, (uint8_t *)&g_rx_byte, 1);

}

void Lidar_StartScan_Express(void)
{
    // 命令格式: A5 82 05 00 00 00 00 00 27
    uint8_t start_cmd[] = {0xA5, RPLIDAR_CMD_EXPRESS_SCAN, 0x05, 0x00, 0x00, 0x00, 0x00, 0x00, 0x27}; // [cite: 326]
    g_rx_index = 0;
    g_current_mode = LIDAR_MODE_EXPRESS;
    HAL_UART_Transmit(LIDAR_UART_HANDLE, start_cmd, sizeof(start_cmd), 100);
}

void Lidar_StartScan_Standard(void)
{
    // 标准扫描请求报文: A5 20
    uint8_t start_cmd[] = {0xA5, RPLIDAR_CMD_SCAN};
    g_rx_index = 0;
    g_current_mode = LIDAR_MODE_STANDARD;
    HAL_UART_Transmit(LIDAR_UART_HANDLE, start_cmd, sizeof(start_cmd), 100);
}

void Lidar_StopScan(void)
{
    uint8_t stop_cmd[] = {0xA5, RPLIDAR_CMD_STOP};// [cite: 196]
    g_current_mode = LIDAR_MODE_NONE;
    HAL_UART_Transmit(LIDAR_UART_HANDLE, stop_cmd, sizeof(stop_cmd), 100);
    HAL_Delay(10); // 根据协议建议，等待一段时间 [cite: 199]
}

// --- 内部数据处理函数 ---

/**
 * @brief  数据处理函数 - 密实模式 (84字节)
 */
static void Lidar_ProcessFrame_Express(uint8_t* frame)
{
    static float angle_last = 0.0f;
    static uint16_t point_count = 0;

    // 提取起始角度和S标志 [cite: 425]
    uint8_t s_flag = frame[3] >> 7;
    float angle_new = ((uint16_t)((frame[3] & 0x7F) << 8) | frame[2]) / 64.0f; // [cite: 424]

    if (s_flag) { // [cite: 425]
        if (point_count > 0) {
            g_lidar_scan_ready = 1;
        }
        point_count = 0;
    }

    float angle_diff;
    if (angle_new < angle_last) {
        angle_diff = (angle_new + 360.0f - angle_last); // [cite: 474]
    } else {
        angle_diff = (angle_new - angle_last); // [cite: 474]
    }

    for (int i = 0; i < RPLIDAR_POINTS_PER_FRAME; i++) { // [cite: 433]
        if (point_count >= 300) break;

        uint16_t distance = (uint16_t)frame[4 + i * 2 + 1] << 8 | frame[4 + i * 2]; // [cite: 436]
        g_lidar_points[point_count].distance = distance;

        float current_angle = angle_last + (angle_diff / RPLIDAR_POINTS_PER_FRAME) * i; // [cite: 467]
        if (current_angle >= 360.0f) {
            current_angle -= 360.0f;
        }
        g_lidar_points[point_count].angle = (uint16_t)(current_angle * 100.0f);

        point_count++;
    }

    angle_last = angle_new;
}

/**
 * @brief  数据处理函数 - 标准模式 (5字节)
 */
static void Lidar_ProcessFrame_Standard(uint8_t* frame)
{
    static uint16_t point_count = 0;

    uint8_t s_flag = frame[0] & 0x01;

    if (s_flag) {
        if (point_count > 0) {
            g_lidar_scan_ready = 1;
        }
        point_count = 0;
    }

    if (point_count >= 300) return;

    uint16_t angle_q6 = ((uint16_t)frame[2] << 7) | (frame[1] >> 1); // [cite: 267]
    g_lidar_points[point_count].angle = (uint16_t)((angle_q6 / 64.0f) * 100.0f); // [cite: 267]

    uint16_t distance_q2 = ((uint16_t)frame[4] << 8) | frame[3]; // [cite: 267]
    g_lidar_points[point_count].distance = (uint16_t)(distance_q2 / 4.0f); // [cite: 267]

    point_count++;
}


// --- HAL库回调函数实现 ---

/**
  * @brief  HAL库的UART接收完成回调函数
  * @note   此函数在每次通过IT方式成功接收一个字节后被HAL_UART_IRQHandler调用
  * @param  huart: 发生中断的UART句柄
  */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // 确认是雷达所在的串口(USART6)触发的中断
    if (huart->Instance == USART6)
    {
        uint8_t current_byte = g_rx_byte;

        switch (g_current_mode)
        {
            case LIDAR_MODE_EXPRESS:
                // Express 模式接收状态机 (84字节)
                if (g_rx_index == 0 && current_byte != (RPLIDAR_FRAME_HEADER_1 | (RPLIDAR_FRAME_HEADER_2 << 4))) break;
                if (g_rx_index == 1 && current_byte != (RPLIDAR_FRAME_HEADER_2 | (RPLIDAR_FRAME_HEADER_1 << 4))) { g_rx_index = 0; break; }

                g_rx_buffer[g_rx_index++] = current_byte;
                if (g_rx_index >= RPLIDAR_EXPRESS_FRAME_SIZE) {
                    uint8_t received_checksum = ((g_rx_buffer[1] >> 4) << 4) | (g_rx_buffer[0] & 0x0F);
                    uint8_t calculated_checksum = 0;
                    for (int i = 2; i < RPLIDAR_EXPRESS_FRAME_SIZE; i++) {
                        calculated_checksum ^= g_rx_buffer[i];
                    }
                    if (received_checksum == calculated_checksum) { // [cite: 429]
                        Lidar_ProcessFrame_Express(g_rx_buffer);
                    }
                    g_rx_index = 0;
                }
                break;

            case LIDAR_MODE_STANDARD:
                // Standard 模式接收状态机 (5字节)
                if (g_rx_index == 0) {
                    if ((current_byte & 0x01) != ((current_byte & 0x02) >> 1)) break; // 起始位校验 [cite: 267]
                } else if (g_rx_index == 1) {
                    if (!(current_byte & 0x01)) { g_rx_index = 0; break; } // 校验位C [cite: 267]
                }

                g_rx_buffer[g_rx_index++] = current_byte;
                if (g_rx_index >= RPLIDAR_STANDARD_FRAME_SIZE) {
                    Lidar_ProcessFrame_Standard(g_rx_buffer);
                    g_rx_index = 0;
                }
                break;

            case LIDAR_MODE_NONE:
            default:
                g_rx_index = 0;
                break;
        }

        // 接收完一个字节后，必须立即重新开启下一次接收
        HAL_UART_Receive_IT(LIDAR_UART_HANDLE, (uint8_t *)&g_rx_byte, 1);
    }
}