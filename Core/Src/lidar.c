#include "lidar.h"
#include "usart.h"
#include "string.h"
#include <stdio.h>
uint8_t rplidar_rx_byte;
static uint8_t rplidar_response_header[7];
static uint8_t scan_started = 0;

void RPLIDAR_Init(void)
{
    HAL_Delay(100);
    HAL_UART_Receive_IT(&huart6, &rplidar_rx_byte, 1);
    RPLIDAR_RequestScan();
}

void RPLIDAR_RequestScan(void)
{
    uint8_t cmd[] = {0xA5, 0x20};
    HAL_UART_Transmit(&huart6, cmd, 2, HAL_MAX_DELAY);
}

void RPLIDAR_ProcessByte(uint8_t byte) {
    static uint8_t buffer[5];
    static int index = 0;
    static int header_index = 0;
    static int print_count = 0;

    if (!scan_started) {
        rplidar_response_header[header_index++] = byte;
        if (header_index == 7) {
            header_index = 0;
            if (rplidar_response_header[0] == 0xA5 && rplidar_response_header[1] == 0x5A) {
                scan_started = 1;
            }
        }
    } else {
        buffer[index++] = byte;
        if (index == 5) {
            index = 0;

            if ((buffer[0] & 0x01) != 0x01 || (buffer[0] & 0x02) != 0x00) return;

            uint16_t angle_q6 = ((buffer[1] >> 1) | ((uint16_t)(buffer[2] & 0x7F) << 7));
            uint16_t distance_q2 = (buffer[3] | ((uint16_t)buffer[4] << 8));

            float angle = angle_q6 / 64.0f;
            float distance = distance_q2 / 4.0f;

            print_count++;
            if (print_count >= 10) {
                char msg[64];
                snprintf(msg, sizeof(msg), "Angle: %.1f deg  Dist: %.1f mm\r\n", angle, distance);

                HAL_UART_Transmit(&huart5, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);
                print_count = 0;
            }
        }
    }

//    HAL_UART_Receive_IT(&huart6, &rplidar_rx_byte, 1);  // ????
}
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    // 确认是雷达所在的串口(USART6)触发的中断
    if (huart->Instance == USART6)
    {
        RPLIDAR_ProcessByte(rplidar_rx_byte);

        // 接收完一个字节后，必须立即重新开启下一次接收
        HAL_UART_Receive_IT(&huart6, (uint8_t *)&rplidar_rx_byte, 1);
    }
}
void Lidar_StopScan(void)
{
    uint8_t stop_cmd[] = {0xA5, RPLIDAR_CMD_STOP}; //
    HAL_UART_Transmit(&huart6, stop_cmd, sizeof(stop_cmd), 100);
}