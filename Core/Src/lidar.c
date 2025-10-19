#include "lidar.h"
#include "usart.h"
#include "string.h"
#include <stdio.h>

#include "hc04.h"
uint8_t rplidar_rx_byte;
volatile uint8_t lidar_raw_stream_active = 0;
volatile uint8_t lidar_raw_overflow = 0;

// 环形缓冲区
#define LIDAR_RAW_BUF_SIZE 1024
static uint8_t raw_buf[LIDAR_RAW_BUF_SIZE];
static volatile uint16_t raw_head = 0; // 写入位置
static volatile uint16_t raw_tail = 0; // 读取位置

// 分片参数（典型 BLE MTU=20）
#define BLE_MTU_PAYLOAD 20
static uint32_t last_flush_tick = 0;
volatile uint32_t overflow_count = 0;
#define RAW_FLUSH_INTERVAL_MS 5  // 最长等待时间
#define RAW_MIN_BATCH 20         // 满足 MTU 时立即发送

static inline uint16_t raw_count(void) {
    if(raw_head >= raw_tail) return raw_head - raw_tail;
    return (uint16_t)(LIDAR_RAW_BUF_SIZE - raw_tail + raw_head);
}

static void raw_push(uint8_t b) {
    uint16_t next = (uint16_t)((raw_head + 1) % LIDAR_RAW_BUF_SIZE);
    if(next == raw_tail) {
        // 溢出：丢弃最旧一个字节
        raw_tail = (uint16_t)((raw_tail + 1) % LIDAR_RAW_BUF_SIZE);
        lidar_raw_overflow = 1;
        overflow_count++; // 增加计数器
    }
    raw_buf[raw_head] = b;
    raw_head = next;
}

static void raw_flush_chunk(uint16_t len) {
    // len 不超过 raw_count()
    // 分片发送（最多 BLE_MTU_PAYLOAD）
    while(len) {
        uint16_t one = len > BLE_MTU_PAYLOAD ? BLE_MTU_PAYLOAD : len;
        // 处理环形 wrap
        uint16_t contiguous = (raw_head >= raw_tail) ? (raw_head - raw_tail) : (LIDAR_RAW_BUF_SIZE - raw_tail);
        if(one > contiguous) one = contiguous;
        HAL_UART_Transmit(&huart5, &raw_buf[raw_tail], one, 20);
        raw_tail = (uint16_t)((raw_tail + one) % LIDAR_RAW_BUF_SIZE);
        len -= one;
    }
}

static void raw_try_flush(void) {
    uint16_t cnt = raw_count();
    if(!cnt) return;
    // 满 MTU 或超时都 flush
    uint32_t now = HAL_GetTick();
    if(cnt >= RAW_MIN_BATCH || (now - last_flush_tick) >= RAW_FLUSH_INTERVAL_MS) {
        raw_flush_chunk(cnt);
        last_flush_tick = now;
    }
}

void RPLIDAR_Init(void)
{
    HAL_Delay(100);
    HAL_UART_Receive_IT(&huart6, &rplidar_rx_byte, 1);
}

void RPLIDAR_RequestScan(void)
{
    uint8_t cmd[] = {0xA5, 0x20};
    HAL_UART_Transmit(&huart6, cmd, 2, HAL_MAX_DELAY);
}

void RPLIDAR_StartRaw(void)
{
    lidar_raw_stream_active = 1;
    lidar_raw_overflow = 0;
    raw_head = raw_tail = 0;
    RPLIDAR_RequestScan(); // 发送启动命令，等待输出头 a5 5a ...
}

void RPLIDAR_StopRaw(void)
{
    Lidar_StopScan();
    lidar_raw_stream_active = 0;
    // 结束前 flush 剩余
    raw_try_flush();
}

void Lidar_StopScan(void)
{
    uint8_t stop_cmd[] = {0xA5, RPLIDAR_CMD_STOP};
    HAL_UART_Transmit(&huart6, stop_cmd, sizeof(stop_cmd), 50);
}

void RPLIDAR_RawTask(void)
{
    if(lidar_raw_stream_active) {
        raw_try_flush();
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6)
    {
        uint8_t b = rplidar_rx_byte;
        if(lidar_raw_stream_active) {
            raw_push(b); // 仅入缓冲，不阻塞
        }
        // 若未开启透传可忽略或实现其它功能（此处忽略）
        HAL_UART_Receive_IT(&huart6, (uint8_t *)&rplidar_rx_byte, 1);
    }
}
