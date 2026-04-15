#ifndef __RPLIDAR_H
#define __RPLIDAR_H

#include "stdint.h"

/* RPLidar commands */
#define RPLIDAR_CMD_STOP                 0x25U
#define RPLIDAR_CMD_SCAN                 0x20U
#define RPLIDAR_CMD_EXPRESS_SCAN         0x82U

/* RPLidar standard node frame format */
#define RPLIDAR_STANDARD_FRAME_SIZE      5U

/* DMA ping-pong configuration: 2 fixed-size halves */
#define LIDAR_DMA_HALF_SIZE              256U
#define LIDAR_DMA_BUFFER_SIZE            (2U * LIDAR_DMA_HALF_SIZE)

typedef struct {
    uint16_t offset;          /* Start offset in DMA buffer */
    uint16_t length;          /* Completed block length */
    uint32_t block_id;        /* Monotonic block counter */
    uint32_t event_counter;   /* Monotonic ISR event counter */
} LidarDmaBlockDesc_t;

typedef struct {
    uint32_t block_id;
    uint32_t event_counter;
    uint16_t offset;
    uint16_t length;
    uint16_t valid_nodes;
    uint16_t invalid_nodes;
    uint16_t start_nodes;
    uint16_t min_distance_mm;
    uint16_t max_distance_mm;
    uint16_t resync_events;
} LidarParseResult_t;

/* Legacy compatibility globals */
extern volatile uint8_t lidar_raw_stream_active;
extern volatile uint8_t lidar_raw_overflow;
extern volatile uint8_t scan_data_ready_flag;
extern volatile uint16_t point_count;
extern volatile uint32_t overflow_count;

void RPLIDAR_Init(void);
void RPLIDAR_RequestScan(void);
void Lidar_StopScan(void);
void RPLIDAR_StartRaw(void);
void RPLIDAR_StopRaw(void);

/* RTOS pipeline setup */
void LIDAR_RTOS_Init(void);

/* Legacy interfaces kept for compatibility with existing modules */
void RPLIDAR_RawTask(void);
void LIDAR_ParseTask(void);
void send_packaged_data(void);
void send_binary_packaged_data(void);

#endif
