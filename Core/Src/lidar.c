#include "lidar.h"

#include <stdio.h>
#include <string.h>

#include "cmsis_os2.h"
#include "usart.h"

/* -------------------------------------------------------------------------- */
/* Global state (kept for compatibility with existing modules)                */
/* -------------------------------------------------------------------------- */
volatile uint8_t lidar_raw_stream_active = 0U;
volatile uint8_t lidar_raw_overflow = 0U;
volatile uint8_t scan_data_ready_flag = 0U;
volatile uint16_t point_count = 0U;
volatile uint32_t overflow_count = 0U;

/* -------------------------------------------------------------------------- */
/* RTOS + DMA pipeline objects                                                */
/* UART + DMA -> ISR -> Queue1 -> Parser Task -> Queue2 -> Output Task       */
/* -------------------------------------------------------------------------- */
static uint8_t s_lidar_dma_rx_buffer[LIDAR_DMA_BUFFER_SIZE];

static osMessageQueueId_t s_dma_desc_queue = NULL;
static osMessageQueueId_t s_parse_result_queue = NULL;

static volatile uint32_t s_dma_event_counter = 0U;
static volatile uint32_t s_dma_block_counter = 0U;
static volatile uint32_t s_queue1_drop_counter = 0U;
static volatile uint32_t s_queue2_drop_counter = 0U;

#define LIDAR_QUEUE1_LEN  16U
#define LIDAR_QUEUE2_LEN  16U

typedef struct {
    uint8_t packet[RPLIDAR_STANDARD_FRAME_SIZE];
    uint8_t packet_idx;
} LidarStreamParserState_t;

static LidarStreamParserState_t s_parser_state;

static void LIDAR_ParserTask(void *argument);
static void LIDAR_OutputTask(void *argument);

static void LIDAR_DebugPrint(const char *message)
{
    size_t len = strlen(message);
    if (len > 0U) {
        (void)HAL_UART_Transmit(&huart5, (uint8_t *)message, (uint16_t)len, 100U);
    }
}

static void LIDAR_ResetParserState(void)
{
    memset(&s_parser_state, 0, sizeof(s_parser_state));
}

static void LIDAR_PublishDmaDescriptorFromISR(uint16_t offset, uint16_t length)
{
    if ((lidar_raw_stream_active == 0U) || (s_dma_desc_queue == NULL)) {
        return;
    }

    if (osKernelGetState() != osKernelRunning) {
        return;
    }

    LidarDmaBlockDesc_t desc;
    desc.offset = offset;
    desc.length = length;
    desc.block_id = ++s_dma_block_counter;
    desc.event_counter = ++s_dma_event_counter;

    if (osMessageQueuePut(s_dma_desc_queue, &desc, 0U, 0U) != osOK) {
        s_queue1_drop_counter++;
        overflow_count = s_queue1_drop_counter;
        lidar_raw_overflow = 1U;
    }
}

static void LIDAR_ParseStreamBytes(const uint8_t *data, uint16_t len, LidarParseResult_t *result)
{
    uint16_t i;

    for (i = 0U; i < len; i++) {
        uint8_t byte = data[i];

        if (s_parser_state.packet_idx == 0U) {
            uint8_t s_bit = byte & 0x01U;
            uint8_t s_inv_bit = (byte >> 1) & 0x01U;

            if (s_bit == s_inv_bit) {
                result->resync_events++;
                continue;
            }

            s_parser_state.packet[0] = byte;
            s_parser_state.packet_idx = 1U;
            continue;
        }

        s_parser_state.packet[s_parser_state.packet_idx++] = byte;
        if (s_parser_state.packet_idx < RPLIDAR_STANDARD_FRAME_SIZE) {
            continue;
        }

        {
            const uint8_t *pkt = s_parser_state.packet;
            uint16_t distance_q2;
            uint16_t distance_mm;

            s_parser_state.packet_idx = 0U;

            if ((pkt[1] & 0x01U) == 0U) {
                result->invalid_nodes++;
                continue;
            }

            if ((pkt[0] & 0x01U) != 0U) {
                result->start_nodes++;
            }

            distance_q2 = ((uint16_t)pkt[4] << 8) | pkt[3];
            if (distance_q2 == 0U) {
                continue;
            }

            distance_mm = (uint16_t)(distance_q2 >> 2);
            result->valid_nodes++;

            if (distance_mm < result->min_distance_mm) {
                result->min_distance_mm = distance_mm;
            }
            if (distance_mm > result->max_distance_mm) {
                result->max_distance_mm = distance_mm;
            }
        }
    }

    if (result->valid_nodes == 0U) {
        result->min_distance_mm = 0U;
    }
}

void LIDAR_RTOS_Init(void)
{
    static const osThreadAttr_t parser_task_attr = {
        .name = "LidarParser",
        .priority = osPriorityAboveNormal,
        .stack_size = 1024U
    };
    static const osThreadAttr_t output_task_attr = {
        .name = "LidarOutput",
        .priority = osPriorityNormal,
        .stack_size = 1024U
    };

    s_dma_desc_queue = osMessageQueueNew(LIDAR_QUEUE1_LEN, sizeof(LidarDmaBlockDesc_t), NULL);
    s_parse_result_queue = osMessageQueueNew(LIDAR_QUEUE2_LEN, sizeof(LidarParseResult_t), NULL);

    if ((s_dma_desc_queue == NULL) || (s_parse_result_queue == NULL)) {
        Error_Handler();
    }

    if (osThreadNew(LIDAR_ParserTask, NULL, &parser_task_attr) == NULL) {
        Error_Handler();
    }
    if (osThreadNew(LIDAR_OutputTask, NULL, &output_task_attr) == NULL) {
        Error_Handler();
    }
}

void RPLIDAR_Init(void)
{
    LIDAR_ResetParserState();
    memset(s_lidar_dma_rx_buffer, 0, sizeof(s_lidar_dma_rx_buffer));
}

void RPLIDAR_RequestScan(void)
{
    const uint8_t cmd[] = {0xA5U, RPLIDAR_CMD_SCAN};
    HAL_StatusTypeDef status = HAL_UART_Transmit(&huart6, (uint8_t *)cmd, sizeof(cmd), 100U);
    if (status != HAL_OK) {
        LIDAR_DebugPrint("LIDAR scan cmd failed\r\n");
    }
}

void Lidar_StopScan(void)
{
    const uint8_t stop_cmd[] = {0xA5U, RPLIDAR_CMD_STOP};
    (void)HAL_UART_Transmit(&huart6, (uint8_t *)stop_cmd, sizeof(stop_cmd), 50U);
}

void RPLIDAR_StartRaw(void)
{
    if (lidar_raw_stream_active != 0U) {
        return;
    }

    LIDAR_ResetParserState();
    lidar_raw_overflow = 0U;
    scan_data_ready_flag = 0U;
    point_count = 0U;
    overflow_count = 0U;
    s_queue1_drop_counter = 0U;
    s_queue2_drop_counter = 0U;

    if (HAL_UART_Receive_DMA(&huart6, s_lidar_dma_rx_buffer, LIDAR_DMA_BUFFER_SIZE) != HAL_OK) {
        LIDAR_DebugPrint("LIDAR DMA start failed\r\n");
        return;
    }

    lidar_raw_stream_active = 1U;
    RPLIDAR_RequestScan();
    LIDAR_DebugPrint("LIDAR DMA stream started\r\n");
}

void RPLIDAR_StopRaw(void)
{
    if (lidar_raw_stream_active == 0U) {
        return;
    }

    lidar_raw_stream_active = 0U;
    Lidar_StopScan();
    (void)HAL_UART_DMAStop(&huart6);
    LIDAR_DebugPrint("LIDAR DMA stream stopped\r\n");
}

/* DMA half-buffer complete callback */
void HAL_UART_RxHalfCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6) {
        LIDAR_PublishDmaDescriptorFromISR(0U, LIDAR_DMA_HALF_SIZE);
    }
}

/* DMA full-buffer complete callback */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART6) {
        LIDAR_PublishDmaDescriptorFromISR(LIDAR_DMA_HALF_SIZE, LIDAR_DMA_HALF_SIZE);
    }
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
    (void)huart;
}

static void LIDAR_ParserTask(void *argument)
{
    (void)argument;

    for (;;) {
        LidarDmaBlockDesc_t desc;

        if (osMessageQueueGet(s_dma_desc_queue, &desc, NULL, osWaitForever) != osOK) {
            continue;
        }

        if (desc.offset + desc.length > LIDAR_DMA_BUFFER_SIZE) {
            continue;
        }

        {
            LidarParseResult_t result;
            memset(&result, 0, sizeof(result));

            result.block_id = desc.block_id;
            result.event_counter = desc.event_counter;
            result.offset = desc.offset;
            result.length = desc.length;
            result.min_distance_mm = 0xFFFFU;

            LIDAR_ParseStreamBytes(&s_lidar_dma_rx_buffer[desc.offset], desc.length, &result);

            if (osMessageQueuePut(s_parse_result_queue, &result, 0U, 0U) != osOK) {
                s_queue2_drop_counter++;
            }
        }
    }
}

static void LIDAR_OutputTask(void *argument)
{
    (void)argument;

    uint32_t summary_blocks = 0U;
    uint32_t summary_valid_nodes = 0U;
    uint16_t summary_min_distance = 0xFFFFU;
    uint16_t summary_max_distance = 0U;

    for (;;) {
        LidarParseResult_t result;
        char line[160];
        int len;

        if (osMessageQueueGet(s_parse_result_queue, &result, NULL, osWaitForever) != osOK) {
            continue;
        }

        summary_blocks++;
        summary_valid_nodes += result.valid_nodes;
        if ((result.valid_nodes > 0U) && (result.min_distance_mm < summary_min_distance)) {
            summary_min_distance = result.min_distance_mm;
        }
        if (result.max_distance_mm > summary_max_distance) {
            summary_max_distance = result.max_distance_mm;
        }

        /* Throttle debug output to match UART5 bandwidth. */
        if (summary_blocks < 25U) {
            continue;
        }

        if (summary_min_distance == 0xFFFFU) {
            summary_min_distance = 0U;
        }

        len = snprintf(line, sizeof(line),
                       "[LDR] evt=%lu blk=%lu blocks=%lu valid=%lu min=%umm max=%umm q1drop=%lu q2drop=%lu\r\n",
                       (unsigned long)result.event_counter,
                       (unsigned long)result.block_id,
                       (unsigned long)summary_blocks,
                       (unsigned long)summary_valid_nodes,
                       summary_min_distance,
                       summary_max_distance,
                       (unsigned long)s_queue1_drop_counter,
                       (unsigned long)s_queue2_drop_counter);
        if (len > 0) {
            (void)HAL_UART_Transmit(&huart5, (uint8_t *)line, (uint16_t)len, 100U);
        }

        summary_blocks = 0U;
        summary_valid_nodes = 0U;
        summary_min_distance = 0xFFFFU;
        summary_max_distance = 0U;
    }
}

/* -------------------------------------------------------------------------- */
/* Legacy interfaces kept for existing code paths                             */
/* -------------------------------------------------------------------------- */
void RPLIDAR_RawTask(void)
{
    /* No-op: stream handling is fully event-driven in RTOS tasks now. */
}

void LIDAR_ParseTask(void)
{
    /* No-op: parsing is handled in LIDAR_ParserTask. */
}

void send_packaged_data(void)
{
    /* No-op compatibility shim. */
}

void send_binary_packaged_data(void)
{
    /* No-op compatibility shim. */
}
