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
#define RAW_FLUSH_INTERVAL_MS 5
#define RAW_MIN_BATCH 20

static ParserState_t parser_state = STATE_WAIT_START;
static uint8_t packet_buffer[5]; // 临时存储一个5字节的数据包
static uint8_t packet_idx = 0;

#define MAX_LIDAR_POINTS (500 * 2)
static LidarPoint_t lidar_points[MAX_LIDAR_POINTS];
static volatile uint16_t point_count = 0; // 当前已存储点的数量

static volatile uint8_t scan_count = 0; // 记录扫描了多少圈
#define TARGET_SCAN_COUNT 2 // 目标扫描圈数
volatile uint8_t scan_data_ready_flag = 0; // 数据准备就绪标志

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

void LIDAR_ParseTask(void)
{
    // 如果数据已准备好，或正在处理，则暂时不解析新数据
    if (scan_data_ready_flag) {
        return;
    }

    uint8_t current_byte;

    // 循环处理环形缓冲区中的所有字节
    while (raw_head != raw_tail) {
        // 从缓冲区取出一个字节
        current_byte = raw_buf[raw_tail];
        raw_tail = (raw_tail + 1) % LIDAR_RAW_BUF_SIZE;

        switch (parser_state) {
            case STATE_WAIT_START:
            {
                // S位(bit0)为1, ~S位(bit1)为0
                if ((current_byte & 0x01) && !(current_byte & 0x02)) {
                    // 这是新一圈扫描的第一个点
                    if (scan_count == 0) {
                        // 如果是第一次启动，或处理完后复位，开始第一圈计数
                        point_count = 0; // 清空点计数器
                    }
                    scan_count++; // 扫描圈数加1

                    packet_buffer[0] = current_byte;
                    packet_idx = 1;
                    parser_state = STATE_RECEIVE_DATA;
                }
                break;
            }

            case STATE_RECEIVE_DATA:
            {
                packet_buffer[packet_idx++] = current_byte;

                if (packet_idx >= 5) {
                    // --- 收到一个完整的5字节数据包，开始解析 ---

                    // 校验位C (Byte1的bit0) 必须为1
                    uint8_t check_bit = packet_buffer[1] & 0x01;
                    if (check_bit) {
                        // 提取数据
                        uint8_t quality = packet_buffer[0] >> 2;
                        uint16_t angle_q6 = ((uint16_t)packet_buffer[2] << 7) | (packet_buffer[1] >> 1);
                        uint16_t distance_q2 = ((uint16_t)packet_buffer[4] << 8) | packet_buffer[3];

                        // 转换成浮点数 (根据你提供的图片公式)
                        float angle = (float)angle_q6 / 64.0f;
                        float distance = (float)distance_q2 / 4.0f;

                        // 存储有效数据点 (距离不为0)
                        if (distance > 0 && point_count < MAX_LIDAR_POINTS) {
                            lidar_points[point_count].angle_deg = angle;
                            lidar_points[point_count].distance_mm = distance;
                            lidar_points[point_count].quality = quality;
                            point_count++;
                        }
                    }

                    // --- 解析完成，重置状态机，等待下一个包的起始位 ---
                    packet_idx = 0;
                    parser_state = STATE_WAIT_START;

                    // --- 检查是否达到目标扫描圈数 ---
                    if (scan_count >= TARGET_SCAN_COUNT) {
                         // S=1标志位表示一圈的开始，所以当下一圈开始时，说明前两圈已经完整
                         // 更准确的触发时机是：当检测到第 (TARGET_SCAN_COUNT+1) 圈的S=1标志时
                         // 这里我们简化为，只要计数值达到目标，且检测到下一个包的S=1就触发
                        if ((packet_buffer[0] & 0x01)) { // 确认这个包确实是下一圈的起始包
                           scan_data_ready_flag = 1; // 设置数据就绪标志
                           scan_count = 0; // 复位计数器，为下一次做准备
                           // 此时，可以停止解析，直到数据被发送
                           return; // 直接退出，主循环会处理数据
                        }
                    }
                }
                break;
            }
        }
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
/**
 * @brief 发送里程计和雷达数据
 */
void send_odom_and_lidar_data(void)
{
    // --- 1. 获取并发送里程计数据 ---
    Pose_t current_pose;
    Odometry_GetPose(&current_pose); // 安全地获取当前位姿

    // *** 强烈建议使用二进制格式发送 ***
    // 示例：帧头(0xA5, 0x5A) + 类型(0x01) + 3个float (12字节) + 校验和
    uint8_t odom_buffer[16];
    odom_buffer[0] = 0xA5;
    odom_buffer[1] = 0x5A;
    odom_buffer[2] = 0x01; // 类型: 里程计
    // 将 float 复制到字节数组中
    memcpy(&odom_buffer[3], &current_pose.x, 4);
    memcpy(&odom_buffer[7], &current_pose.y, 4);
    memcpy(&odom_buffer[11], &current_pose.theta, 4);
    // odom_buffer[15] = calculate_checksum(...); // 计算校验和
    HAL_UART_Transmit(&huart5, odom_buffer, 16, 100);

    // --- 2. 发送雷达数据 (同样建议二进制) ---
    // 示例：帧头(0xA5, 0x5A) + 类型(0x02) + 点数量(2字节) + 数据 + 校验和
    // ... 发送雷达数据的代码 ...

    // 为了调试，你也可以先用字符串格式
    /*
    char odom_str_buffer[60];
    sprintf(odom_str_buffer, "ODOM:%.3f,%.3f,%.3f\n", current_pose.x, current_pose.y, current_pose.theta);
    HAL_UART_Transmit(&huart5, (uint8_t*)odom_str_buffer, strlen(odom_str_buffer), HAL_MAX_DELAY);
    */

    // 发送雷达数据部分 (与上次回答一致)
    HAL_UART_Transmit(&huart5, (uint8_t*)"SCAN_START\n", 11, 100);
    char point_buffer[30];
    for (int i = 0; i < point_count; i++) {
        sprintf(point_buffer, "%.2f,%.2f\n", lidar_points[i].angle_deg, lidar_points[i].distance_mm);
        HAL_UART_Transmit(&huart5, (uint8_t*)point_buffer, strlen(point_buffer), 100);
    }
    HAL_UART_Transmit(&huart5, (uint8_t*)"SCAN_END\n", 9, 100);
}