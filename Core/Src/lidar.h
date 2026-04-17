#ifndef __RPLIDAR_H
#define __RPLIDAR_H

#include <stdint.h>

#include "usart.h"

#define RPLIDAR_CMD_STOP                 0x25
#define RPLIDAR_CMD_SCAN                 0x20
#define RPLIDAR_CMD_EXPRESS_SCAN         0x82
#define RPLIDAR_FRAME_HEADER_1           0x0A
#define RPLIDAR_FRAME_HEADER_2           0x05
#define RPLIDAR_POINTS_PER_FRAME         40
#define RPLIDAR_STANDARD_FRAME_SIZE      5
#define RPLIDAR_EXPRESS_FRAME_SIZE       84
#define MAX_LIDAR_POINTS_PER_SCAN        512U
#define LIDAR_DMA_RX_BUFFER_SIZE         4096U
#define LIDAR_DMA_HALF_BUFFER_SIZE       (LIDAR_DMA_RX_BUFFER_SIZE / 2U)

#define PROTOCOL_HEADER_1                0xA5
#define PROTOCOL_HEADER_2                0x5A

extern volatile uint8_t lidar_raw_stream_active;
extern volatile uint8_t lidar_raw_overflow;

typedef struct {
    float angle_deg;
    float distance_mm;
    uint8_t quality;
} LidarPoint_t;

typedef struct LidarScanBuffer LidarScanBuffer_t;

typedef enum {
    LIDAR_PARSE_IN_PROGRESS = 0,
    LIDAR_SCAN_COMPLETE = 1
} LidarParseResult_t;

#pragma pack(push, 1)

typedef struct {
    uint8_t header1;
    uint8_t header2;
    uint16_t payload_len;
} BinaryPacketHeader_t;

typedef struct {
    float x;
    float y;
    float theta_continuous;
} OdometryData_t;

typedef struct {
    uint16_t angle_x100;
    uint16_t distance_mm;
} LidarPointPacked_t;

#pragma pack(pop)

void RPLIDAR_Init(void);
void RPLIDAR_RequestScan(void);
void Lidar_StopScan(void);

void RPLIDAR_StartRaw(void);
void RPLIDAR_StopRaw(void);

void LIDAR_ResetScanState(void);
void LIDAR_ResetParserState(void);
uint32_t LIDAR_GetDmaBlockLag(uint32_t latest_sequence, uint32_t block_sequence);
uint8_t LIDAR_IsDmaBlockStale(uint32_t latest_sequence, uint32_t block_sequence);
LidarParseResult_t LIDAR_ConsumeByte(uint8_t byte, LidarScanBuffer_t *scan_buffer);
void LIDAR_ConsumePendingPacket(LidarScanBuffer_t *scan_buffer);
const uint8_t *LIDAR_GetDmaRxBuffer(void);
HAL_StatusTypeDef send_binary_packaged_data_from_buffer(const LidarScanBuffer_t *scan_buffer);

#endif
