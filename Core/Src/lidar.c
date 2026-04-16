#include <string.h>

#include "freertos_app.h"
#include "lidar.h"
#include "system.h"
#include "usart.h"

volatile uint8_t lidar_raw_stream_active = 0U;
volatile uint8_t lidar_raw_overflow = 0U;
volatile uint32_t overflow_count = 0U;

#define TX_BUFFER_SIZE  8192U

static uint8_t lidar_dma_rx_buffer[LIDAR_DMA_RX_BUFFER_SIZE];
static uint8_t tx_buffer_A[TX_BUFFER_SIZE];
static uint8_t tx_buffer_B[TX_BUFFER_SIZE];
static uint8_t *current_cpu_buffer = tx_buffer_A;
static volatile uint8_t is_dma_tx_busy = 0U;

static uint8_t packet_buffer[RPLIDAR_STANDARD_FRAME_SIZE];
static uint8_t packet_idx = 0U;
static uint8_t pending_packet[RPLIDAR_STANDARD_FRAME_SIZE];
static uint8_t pending_packet_valid = 0U;

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

static void report_lidar_status(const char *prefix, HAL_StatusTypeDef status)
{
    char error_msg[64];
    int len;

    if (prefix == NULL) {
        return;
    }

    len = snprintf(error_msg, sizeof(error_msg), "%s %d\r\n", prefix, (int)status);
    if (len > 0) {
        (void)HAL_UART_Transmit(&huart5, (uint8_t *)error_msg, (uint16_t)len, 200U);
    }
}

static void queue_dma_half_from_isr(uint16_t offset)
{
    if (lidar_raw_stream_active == 0U) {
        return;
    }

    (void)Freertos_SubmitLidarBlockFromISR(offset, LIDAR_DMA_HALF_BUFFER_SIZE);
}

static HAL_StatusTypeDef start_lidar_dma_rx(void)
{
    (void)HAL_UART_DMAStop(&huart6);
    return HAL_UART_Receive_DMA(&huart6, lidar_dma_rx_buffer, LIDAR_DMA_RX_BUFFER_SIZE);
}

void LIDAR_ResetScanState(void)
{
    packet_idx = 0U;
    pending_packet_valid = 0U;
    lidar_raw_overflow = 0U;
    memset(packet_buffer, 0, sizeof(packet_buffer));
    memset(pending_packet, 0, sizeof(pending_packet));
    memset(lidar_dma_rx_buffer, 0, sizeof(lidar_dma_rx_buffer));
}

void RPLIDAR_Init(void)
{
    HAL_Delay(100U);
    LIDAR_ResetScanState();
}

void RPLIDAR_RequestScan(void)
{
    uint8_t cmd[] = {0xA5, RPLIDAR_CMD_SCAN};
    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart6, cmd, sizeof(cmd), 100U);

    if (status != HAL_OK) {
        report_lidar_status("LIDAR Start FAILED! HAL Status:", status);
    }
}

void RPLIDAR_StartRaw(void)
{
    HAL_StatusTypeDef status;

    lidar_raw_stream_active = 0U;
    LIDAR_ResetScanState();
    Freertos_ResetLidarPipeline();

    status = start_lidar_dma_rx();
    if (status != HAL_OK) {
        report_lidar_status("LIDAR DMA start failed:", status);
        return;
    }

    lidar_raw_stream_active = 1U;
    RPLIDAR_RequestScan();
}

void RPLIDAR_StopRaw(void)
{
    lidar_raw_stream_active = 0U;
    Lidar_StopScan();
    (void)HAL_UART_DMAStop(&huart6);
    LIDAR_ResetScanState();
    Freertos_ResetLidarPipeline();
}

void Lidar_StopScan(void)
{
    uint8_t stop_cmd[] = {0xA5, RPLIDAR_CMD_STOP};
    (void)HAL_UART_Transmit(&huart6, stop_cmd, sizeof(stop_cmd), 50U);
}

LidarParseResult_t LIDAR_ConsumeByte(uint8_t byte, LidarScanBuffer_t *scan_buffer)
{
    uint8_t current_packet[RPLIDAR_STANDARD_FRAME_SIZE];

    if (scan_buffer == NULL) {
        return LIDAR_PARSE_IN_PROGRESS;
    }

    if (packet_idx == 0U) {
        uint8_t s_bit = byte & 0x01U;
        uint8_t s_inv_bit = (byte & 0x02U) >> 1;
        if (s_bit == s_inv_bit) {
            return LIDAR_PARSE_IN_PROGRESS;
        }
    }

    packet_buffer[packet_idx++] = byte;
    if (packet_idx < RPLIDAR_STANDARD_FRAME_SIZE) {
        return LIDAR_PARSE_IN_PROGRESS;
    }

    (void)memcpy(current_packet, packet_buffer, sizeof(current_packet));
    packet_idx = 0U;

    if (((current_packet[0] & 0x01U) != 0U) && (scan_buffer->point_count > 0U)) {
        (void)memcpy(pending_packet, current_packet, sizeof(pending_packet));
        pending_packet_valid = 1U;
        return LIDAR_SCAN_COMPLETE;
    }

    (void)parse_packet_into_scan(current_packet, scan_buffer);
    return LIDAR_PARSE_IN_PROGRESS;
}

void LIDAR_ConsumePendingPacket(LidarScanBuffer_t *scan_buffer)
{
    if ((scan_buffer == NULL) || (pending_packet_valid == 0U)) {
        return;
    }

    pending_packet_valid = 0U;
    (void)parse_packet_into_scan(pending_packet, scan_buffer);
}

const uint8_t *LIDAR_GetDmaRxBuffer(void)
{
    return lidar_dma_rx_buffer;
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

void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6) {
        queue_dma_half_from_isr(0U);
    }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6) {
        queue_dma_half_from_isr(LIDAR_DMA_HALF_BUFFER_SIZE);
    }
}
