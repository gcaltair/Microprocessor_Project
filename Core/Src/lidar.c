#include <string.h>

#include "freertos_app.h"
#include "lidar.h"
#include "system.h"
#include "usart.h"

uint8_t rplidar_rx_byte;

volatile uint8_t lidar_raw_stream_active = 0U;
volatile uint8_t lidar_raw_overflow = 0U;
volatile uint32_t overflow_count = 0U;

#define LIDAR_RAW_BUF_SIZE  4096U
#define TX_BUFFER_SIZE      8192U

static uint8_t raw_buf[LIDAR_RAW_BUF_SIZE];
static volatile uint16_t raw_head = 0U;
static volatile uint16_t raw_tail = 0U;

static uint8_t tx_buffer_A[TX_BUFFER_SIZE];
static uint8_t tx_buffer_B[TX_BUFFER_SIZE];
static uint8_t *current_cpu_buffer = tx_buffer_A;
static volatile uint8_t is_dma_tx_busy = 0U;

static uint8_t packet_buffer[RPLIDAR_STANDARD_FRAME_SIZE];
static uint8_t packet_idx = 0U;
static uint8_t pending_packet[RPLIDAR_STANDARD_FRAME_SIZE];
static uint8_t pending_packet_valid = 0U;

static void raw_push(uint8_t byte)
{
    uint16_t next = (uint16_t)((raw_head + 1U) % LIDAR_RAW_BUF_SIZE);
    if (next == raw_tail) {
        raw_tail = (uint16_t)((raw_tail + 1U) % LIDAR_RAW_BUF_SIZE);
        lidar_raw_overflow = 1U;
        overflow_count++;
    }

    raw_buf[raw_head] = byte;
    raw_head = next;
}

static uint8_t raw_pop(uint8_t *byte)
{
    if ((byte == NULL) || (raw_head == raw_tail)) {
        return 0U;
    }

    *byte = raw_buf[raw_tail];
    raw_tail = (uint16_t)((raw_tail + 1U) % LIDAR_RAW_BUF_SIZE);
    return 1U;
}

static uint8_t parse_packet_into_scan(const uint8_t *packet, LidarScanBuffer_t *scan_buffer)
{
    uint16_t angle_q6;
    uint16_t distance_q2;
    uint16_t index;

    if ((packet == NULL) || (scan_buffer == NULL)) {
        return 0U;
    }

    if ((packet[1] & 0x01U) == 0U) {
        return 0U;
    }

    if (scan_buffer->point_count >= MAX_LIDAR_POINTS_PER_SCAN) {
        return 0U;
    }

    angle_q6 = ((uint16_t)packet[2] << 7) | (packet[1] >> 1);
    distance_q2 = ((uint16_t)packet[4] << 8) | packet[3];
    if (distance_q2 == 0U) {
        return 0U;
    }

    index = scan_buffer->point_count;
    scan_buffer->points[index].quality = packet[0] >> 2;
    scan_buffer->points[index].angle_deg = (float)angle_q6 / 64.0f;
    scan_buffer->points[index].distance_mm = (float)distance_q2 / 4.0f;
    scan_buffer->point_count++;

    return 1U;
}

void LIDAR_ResetScanState(void)
{
    raw_head = 0U;
    raw_tail = 0U;
    packet_idx = 0U;
    pending_packet_valid = 0U;
    lidar_raw_overflow = 0U;
}

void RPLIDAR_Init(void)
{
    HAL_Delay(100);
    (void)HAL_UART_Receive_IT(&huart6, &rplidar_rx_byte, 1U);
}

void RPLIDAR_RequestScan(void)
{
    uint8_t cmd[] = {0xA5, 0x20};
    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart6, cmd, sizeof(cmd), 100);

    if (status != HAL_OK) {
        char error_msg[50];
        (void)snprintf(error_msg, sizeof(error_msg), "LIDAR Start FAILED! HAL Status: %d\r\n", status);
        (void)HAL_UART_Transmit(&huart5, (uint8_t *)error_msg, (uint16_t)strlen(error_msg), 200);
    }
}

void RPLIDAR_StartRaw(void)
{
    lidar_raw_stream_active = 1U;
    LIDAR_ResetScanState();
    RPLIDAR_RequestScan();
}

void RPLIDAR_StopRaw(void)
{
    Lidar_StopScan();
    lidar_raw_stream_active = 0U;
    LIDAR_ResetScanState();
}

void Lidar_StopScan(void)
{
    uint8_t stop_cmd[] = {0xA5, RPLIDAR_CMD_STOP};
    (void)HAL_UART_Transmit(&huart6, stop_cmd, sizeof(stop_cmd), 50);
}

void RPLIDAR_RawTask(void)
{
}

LidarParseResult_t LIDAR_ParseStep(LidarScanBuffer_t *scan_buffer)
{
    uint8_t current_packet[RPLIDAR_STANDARD_FRAME_SIZE];

    if ((scan_buffer == NULL) || (lidar_raw_stream_active == 0U)) {
        return LIDAR_PARSE_IN_PROGRESS;
    }

    for (;;) {
        if (pending_packet_valid != 0U) {
            (void)memcpy(current_packet, pending_packet, sizeof(current_packet));
            pending_packet_valid = 0U;
        } else {
            while (packet_idx < RPLIDAR_STANDARD_FRAME_SIZE) {
                uint8_t current_byte = 0U;

                if (raw_pop(&current_byte) == 0U) {
                    return LIDAR_PARSE_IN_PROGRESS;
                }

                if (packet_idx == 0U) {
                    uint8_t s_bit = current_byte & 0x01U;
                    uint8_t s_inv_bit = (current_byte & 0x02U) >> 1;
                    if (s_bit == s_inv_bit) {
                        continue;
                    }
                }

                packet_buffer[packet_idx++] = current_byte;
            }

            (void)memcpy(current_packet, packet_buffer, sizeof(current_packet));
            packet_idx = 0U;
        }

        if (((current_packet[0] & 0x01U) != 0U) && (scan_buffer->point_count > 0U)) {
            (void)memcpy(pending_packet, current_packet, sizeof(pending_packet));
            pending_packet_valid = 1U;
            return LIDAR_SCAN_COMPLETE;
        }

        (void)parse_packet_into_scan(current_packet, scan_buffer);
    }
}

HAL_StatusTypeDef send_binary_packaged_data_from_buffer(const LidarScanBuffer_t *scan_buffer)
{
    BinaryPacketHeader_t header;
    OdometryData_t odom_data;
    uint16_t odom_size = sizeof(OdometryData_t);
    uint16_t lidar_points_size;
    uint16_t total_payload_len;
    uint16_t buffer_offset = 0U;

    if ((scan_buffer == NULL) || (scan_buffer->point_count == 0U)) {
        return HAL_OK;
    }

    if (is_dma_tx_busy != 0U) {
        return HAL_BUSY;
    }

    lidar_points_size = (uint16_t)(scan_buffer->point_count * sizeof(LidarPointPacked_t));
    total_payload_len = (uint16_t)(odom_size + lidar_points_size);
    if ((sizeof(BinaryPacketHeader_t) + total_payload_len) > TX_BUFFER_SIZE) {
        return HAL_ERROR;
    }

    header.header1 = PROTOCOL_HEADER_1;
    header.header2 = PROTOCOL_HEADER_2;
    header.payload_len = total_payload_len;
    (void)memcpy(current_cpu_buffer + buffer_offset, &header, sizeof(header));
    buffer_offset += sizeof(header);

    odom_data.x = g_x;
    odom_data.y = g_y;
    odom_data.theta_continuous = g_th_continuous;
    (void)memcpy(current_cpu_buffer + buffer_offset, &odom_data, sizeof(odom_data));
    buffer_offset += sizeof(odom_data);

    for (uint16_t i = 0U; i < scan_buffer->point_count; ++i) {
        LidarPointPacked_t packed_point;

        packed_point.angle_x100 = (uint16_t)(scan_buffer->points[i].angle_deg * 100.0f);
        packed_point.distance_mm = (uint16_t)scan_buffer->points[i].distance_mm;
        (void)memcpy(current_cpu_buffer + buffer_offset, &packed_point, sizeof(packed_point));
        buffer_offset += sizeof(packed_point);
    }

    is_dma_tx_busy = 1U;
    if (HAL_UART_Transmit_DMA(&huart5, current_cpu_buffer, buffer_offset) != HAL_OK) {
        is_dma_tx_busy = 0U;
        return HAL_ERROR;
    }

    current_cpu_buffer = (current_cpu_buffer == tx_buffer_A) ? tx_buffer_B : tx_buffer_A;
    return HAL_OK;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == UART5) {
        is_dma_tx_busy = 0U;
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6) {
        if (lidar_raw_stream_active != 0U) {
            raw_push(rplidar_rx_byte);
        }

        (void)HAL_UART_Receive_IT(&huart6, &rplidar_rx_byte, 1U);
    }
}
