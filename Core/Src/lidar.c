#include "lidar.h"
#include "usart.h"
#include "system.h"
uint8_t rplidar_rx_byte;

/*环形缓冲区相关*/
volatile uint8_t lidar_raw_stream_active = 0;
volatile uint8_t lidar_raw_overflow = 0;
#define LIDAR_RAW_BUF_SIZE 1024
static uint8_t raw_buf[LIDAR_RAW_BUF_SIZE];
static volatile uint16_t raw_head = 0;
static volatile uint16_t raw_tail = 0;
#define BLE_MTU_PAYLOAD 20
static uint32_t last_flush_tick = 0;
volatile uint32_t overflow_count = 0;
#define RAW_FLUSH_INTERVAL_MS 5
#define RAW_MIN_BATCH 20

// 解析器状态变量
static ParserState_t parser_state = STATE_WAIT_START_BIT;
static uint8_t packet_buffer[5]; // 临时存储一个5字节的数据包
static uint8_t packet_idx = 0;

// --- 核心数据缓冲区 ---
// 假设雷达一圈最多产生800个点，为安全起见，我们定义得大一些
#define MAX_LIDAR_POINTS_PER_SCAN 1000
static LidarPoint_t lidar_points[MAX_LIDAR_POINTS_PER_SCAN];
volatile uint16_t point_count = 0; // 当前已存储点的数量

// --- 扫描完成标志 ---
// volatile 关键字至关重要，因为它会在主循环和中断中被访问
volatile uint8_t scan_data_ready_flag = 0;

// --- DMA 发送相关定义 ---
#define TX_BUFFER_SIZE 8192 // 发送缓冲区大小 (8KB)
uint8_t tx_buffer_A[TX_BUFFER_SIZE];
uint8_t tx_buffer_B[TX_BUFFER_SIZE];
uint8_t* current_cpu_buffer = tx_buffer_A; // CPU当前可以写入的缓冲区
volatile uint8_t is_dma_tx_busy = 0;      // DMA忙标志, volatile是必须的
// ----------------------
/**
 * @brief  打包里程计和雷达数据为【文本格式】，并通过DMA以非阻塞方式发送
 * @note   这个函数专为调试设计，它会立即返回，不会阻塞主循环。
 */
void send_packaged_data(void)
{
    // 1. 安全检查：如果DMA硬件还在忙于发送上一包数据，则立即返回。
    if (is_dma_tx_busy) {
        return;
    }

    // 2. 准备打包数据，定义一个变量来追踪当前缓冲区的长度
    uint16_t buffer_len = 0;

    // // --- A. 打包里程计数据为字符串 ---
    // Pose_t current_pose;
    // Odometry_GetPose(&current_pose); // 安全地获取当前位姿

    // 使用 sprintf 将格式化的字符串写入发送缓冲区
    // sprintf 会返回写入的字符数，我们用它来累加 buffer_len
    buffer_len += sprintf((char*)current_cpu_buffer + buffer_len,
                          "ODOM:%.3f,%.3f,%.3f\n",
                          g_x, g_y,g_th_continuous);

    // --- B. 打包雷达点云数据为字符串 ---
    buffer_len += sprintf((char*)current_cpu_buffer + buffer_len, "SCAN_START\n");

    // 遍历所有雷达点
    for (int i = 0; i < point_count; i++) {
        // 安全检查：预估一个点需要的最大空间（约30字节），防止缓冲区溢出
        if (buffer_len > TX_BUFFER_SIZE - 30) {
            break;
        }
        buffer_len += sprintf((char*)current_cpu_buffer + buffer_len,
                              "%.2f,%.2f\n",
                              lidar_points[i].angle_deg, lidar_points[i].distance_mm);
    }

    buffer_len += sprintf((char*)current_cpu_buffer + buffer_len, "SCAN_END\n");


    // 3. 启动DMA传输 (这部分逻辑和二进制版本完全一样)
    if (buffer_len > 0)
    {
        // 设置忙碌标志
        is_dma_tx_busy = 1;

        // 调用DMA发送函数
        if (HAL_UART_Transmit_DMA(&huart5, current_cpu_buffer, buffer_len) != HAL_OK)
        {
            is_dma_tx_busy = 0;
        }

        // 切换缓冲区 (Ping-Pong)
        current_cpu_buffer = (current_cpu_buffer == tx_buffer_A) ? tx_buffer_B : tx_buffer_A;
    }
}
/**
  * @brief  UART发送完成回调函数 (由DMA完成中断触发)
  * @param  huart: 触发回调的UART句柄
  * @retval None
  */
void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    // 检查是否是我们的蓝牙串口(USART5)完成了发送
    if (huart->Instance == UART5)
    {
        // 清除忙碌标志，表示DMA已空闲，可以进行下一次发送了
        is_dma_tx_busy = 0;
    }
}
//环形缓冲区大小
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

    // 关键修改：用一个变量来接收HAL函数的返回值
    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart6, cmd, 2, 100);

    // 检查返回值
    if (status != HAL_OK)
    {
        // 如果失败了，通过蓝牙串口(huart5)把错误码打印出来
        char error_msg[50];
        sprintf(error_msg, "LIDAR Start FAILED! HAL Status: %d\r\n", status);

        // 使用阻塞发送，确保调试信息能发出去
        HAL_UART_Transmit(&huart5, (uint8_t*)error_msg, strlen(error_msg), 200);
    }
    else
    {
        // 如果成功了，也可以发一个成功信息
        char success_msg[] = "LIDAR Start command sent successfully.\r\n";
        HAL_UART_Transmit(&huart5, (uint8_t*)success_msg, strlen(success_msg), 200);
    }
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
/**
 * @brief 从环形缓冲区取出字节并解析雷达数据 (新版逻辑)
 */
void LIDAR_ParseTask(void)
{
    if (scan_data_ready_flag) return; // 等待主循环发送数据

    uint8_t current_byte;

    while (raw_head != raw_tail) { // 假设 raw_head/tail 是你的环形缓冲区指针
        current_byte = raw_buf[raw_tail];
        raw_tail = (raw_tail + 1) % LIDAR_RAW_BUF_SIZE;

        // --- 简单的5字节数据包拼装 ---
        if (packet_idx == 0) {
            // 检查是不是一个合法的数据包起始字节
            // S和~S不能相同，否则是无效数据 (Quality全为1或全为0)
            uint8_t s_bit = current_byte & 0x01;
            uint8_t s_inv_bit = (current_byte & 0x02) >> 1;
            if (s_bit != s_inv_bit) {
                packet_buffer[0] = current_byte;
                packet_idx = 1;
            }
        } else {
            packet_buffer[packet_idx++] = current_byte;
        }

        // --- 当一个完整的5字节包拼装好后，进入应用逻辑 ---
        if (packet_idx >= 5) {
            // 检查S位，判断是否是新一圈扫描的开始
            uint8_t is_start_point = packet_buffer[0] & 0x01;

            if (is_start_point) {
                // 观察到了 S=1 的数据包！

                // 1. 如果 point_count > 0，说明我们已经有上一圈的完整数据了
                if (point_count > 0) {
                    scan_data_ready_flag = 1; // 通知主循环发送
                    return;
                }

                // 2. 为新的一圈扫描做准备
                point_count = 0;
            }

            // 无论是不是起始点，只要包有效，就解析并存储
            // 校验位C (Byte1的bit0) 必须为1
            if (packet_buffer[1] & 0x01) {
                if (point_count < MAX_LIDAR_POINTS_PER_SCAN) {
                    // 提取并计算数据
                    lidar_points[point_count].quality     = packet_buffer[0] >> 2;
                    uint16_t angle_q6   = ((uint16_t)packet_buffer[2] << 7) | (packet_buffer[1] >> 1);
                    uint16_t distance_q2= ((uint16_t)packet_buffer[4] << 8) | packet_buffer[3];

                    lidar_points[point_count].angle_deg   = (float)angle_q6 / 64.0f;
                    lidar_points[point_count].distance_mm = (float)distance_q2 / 4.0f;

                    // 只有有效距离的点才增加计数
                    if (distance_q2 != 0) {
                        point_count++;
                    }
                }
            }

            // 重置包解析器，准备接收下一个包
            packet_idx = 0;

            // 如果数据已就绪，立刻退出，让主循环优先处理发送任务
            if (scan_data_ready_flag) return;
        }
    }
}