#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "cmsis_os.h"

#include "../Inc/mapping_task.h"
#include "../Inc/navigation_task.h"
#include "../Inc/usart.h"
#include "hc04.h"
#include "pid.h"

#define UART_PRINTF_BUFFER_SIZE  256U
#define UART5_TX_TIMEOUT_MS      1000U
#define TELEMETRY_FRAME_MAGIC_1          0xC3U
#define TELEMETRY_FRAME_MAGIC_2          0x3CU
#define TELEMETRY_PROTOCOL_VERSION       7U
#define TELEMETRY_FRAME_TYPE_MAP_GRID    1U
#define TELEMETRY_FRAME_HEADER_SIZE      8U
#define TELEMETRY_FRAME_CRC_SIZE         2U
#define TELEMETRY_TASK_PERIOD_MS         250U
#define TELEMETRY_KEEPALIVE_MS           1000U
#define TELEMETRY_UART_TIMEOUT_MS        250U
#define TELEMETRY_FIXED_PAYLOAD_SIZE     159U
#define TELEMETRY_PATH_POINT_SIZE        8U
#define TELEMETRY_MAX_FRAME_SIZE         (TELEMETRY_FRAME_HEADER_SIZE + \
                                          TELEMETRY_FIXED_PAYLOAD_SIZE + \
                                          (NAVIGATION_PATH_TELEMETRY_MAX_POINTS * TELEMETRY_PATH_POINT_SIZE) + \
                                          OGM_MAX_CELL_COUNT + \
                                          TELEMETRY_FRAME_CRC_SIZE)

static char g_uart_printf_buffer[UART_PRINTF_BUFFER_SIZE];
static int8_t g_telemetryCellBuffer[OGM_MAX_CELL_COUNT];
static NavigationPathPoint_t g_telemetryPathBuffer[NAVIGATION_PATH_TELEMETRY_MAX_POINTS];
static uint8_t g_telemetryFrameBuffer[TELEMETRY_MAX_FRAME_SIZE];
static uint16_t g_telemetrySequence = 0U;

static void transmit_buffer(const uint8_t *message, uint16_t size)
{
    if ((message == NULL) || (size == 0U)) {
        return;
    }

    (void)HAL_UART_Transmit(&huart5, (uint8_t *)message, size, UART5_TX_TIMEOUT_MS);
    if (osKernelGetState() == osKernelRunning) {
        osDelay(1U);
    }
}

void transmit(const char *message)
{
    if (message != NULL) {
        transmit_buffer((const uint8_t *)message, (uint16_t)strlen(message));
    }
}

void transmit_uint8(const uint8_t *message, uint16_t size)
{
    transmit_buffer(message, size);
}

void uart_printf(const char *format, ...)
{
    va_list args;
    int len;

    va_start(args, format);
    len = vsnprintf(g_uart_printf_buffer, sizeof(g_uart_printf_buffer), format, args);
    va_end(args);

    if (len <= 0) {
        return;
    }

    if ((size_t)len >= sizeof(g_uart_printf_buffer)) {
        len = (int)(sizeof(g_uart_printf_buffer) - 1U);
    }

    transmit_buffer((const uint8_t *)g_uart_printf_buffer, (uint16_t)len);
}

static uint16_t telemetry_crc16_ccitt(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;
    uint16_t idx;

    if (data == NULL) {
        return crc;
    }

    for (idx = 0U; idx < length; ++idx) {
        uint8_t bit;

        crc ^= (uint16_t)((uint16_t)data[idx] << 8);
        for (bit = 0U; bit < 8U; ++bit) {
            if ((crc & 0x8000U) != 0U) {
                crc = (uint16_t)((crc << 1) ^ 0x1021U);
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}

static void telemetry_write_u8(uint16_t *offset, uint8_t value)
{
    g_telemetryFrameBuffer[*offset] = value;
    *offset = (uint16_t)(*offset + 1U);
}

static void telemetry_write_u16(uint16_t *offset, uint16_t value)
{
    g_telemetryFrameBuffer[*offset] = (uint8_t)(value & 0xFFU);
    g_telemetryFrameBuffer[*offset + 1U] = (uint8_t)((value >> 8) & 0xFFU);
    *offset = (uint16_t)(*offset + 2U);
}

static void telemetry_write_u32(uint16_t *offset, uint32_t value)
{
    g_telemetryFrameBuffer[*offset] = (uint8_t)(value & 0xFFU);
    g_telemetryFrameBuffer[*offset + 1U] = (uint8_t)((value >> 8) & 0xFFU);
    g_telemetryFrameBuffer[*offset + 2U] = (uint8_t)((value >> 16) & 0xFFU);
    g_telemetryFrameBuffer[*offset + 3U] = (uint8_t)((value >> 24) & 0xFFU);
    *offset = (uint16_t)(*offset + 4U);
}

static void telemetry_write_i16(uint16_t *offset, int16_t value)
{
    telemetry_write_u16(offset, (uint16_t)value);
}

static void telemetry_write_f32(uint16_t *offset, float value)
{
    (void)memcpy(&g_telemetryFrameBuffer[*offset], &value, sizeof(value));
    *offset = (uint16_t)(*offset + (uint16_t)sizeof(value));
}

static uint8_t telemetry_uart_ready(void)
{
    return (huart5.gState == HAL_UART_STATE_READY) ? 1U : 0U;
}

static uint16_t telemetry_build_map_frame(uint32_t tick_ms)
{
    MappingGridMeta_t meta;
    MappingTaskStats_t stats;
    NavigationTaskStats_t nav_stats;
    ControlDebugSnapshot_t control_stats;
    uint16_t offset = 0U;
    uint16_t payload_offset;
    uint16_t payload_len;
    uint16_t cell_count;
    uint16_t path_point_count;
    uint16_t path_idx;
    uint16_t frame_len;
    uint16_t crc;

    if (MappingTask_GetGridMeta(&meta) == 0U) {
        return 0U;
    }

    cell_count = (uint16_t)(meta.width_cells * meta.height_cells);
    if ((cell_count == 0U) || (cell_count > OGM_MAX_CELL_COUNT)) {
        return 0U;
    }

    if (MappingTask_CopyGridCells(g_telemetryCellBuffer, OGM_MAX_CELL_COUNT) == 0U) {
        return 0U;
    }

    MappingTask_GetStatsSnapshot(&stats);
    NavigationTask_GetStatsSnapshot(&nav_stats);
    path_point_count = NavigationTask_CopySmoothPathPoints(g_telemetryPathBuffer,
                                                           NAVIGATION_PATH_TELEMETRY_MAX_POINTS);
    Control_GetDebugSnapshot(&control_stats);

    if ((uint32_t)TELEMETRY_FRAME_HEADER_SIZE +
        TELEMETRY_FIXED_PAYLOAD_SIZE +
        ((uint32_t)path_point_count * TELEMETRY_PATH_POINT_SIZE) +
        cell_count +
        TELEMETRY_FRAME_CRC_SIZE > TELEMETRY_MAX_FRAME_SIZE) {
        return 0U;
    }

    telemetry_write_u8(&offset, TELEMETRY_FRAME_MAGIC_1);
    telemetry_write_u8(&offset, TELEMETRY_FRAME_MAGIC_2);
    telemetry_write_u8(&offset, TELEMETRY_PROTOCOL_VERSION);
    telemetry_write_u8(&offset, TELEMETRY_FRAME_TYPE_MAP_GRID);
    telemetry_write_u16(&offset, g_telemetrySequence);
    telemetry_write_u16(&offset, 0U);

    payload_offset = offset;
    telemetry_write_u16(&offset, meta.width_cells);
    telemetry_write_u16(&offset, meta.height_cells);
    telemetry_write_f32(&offset, meta.resolution_m_per_cell);
    telemetry_write_f32(&offset, meta.origin_x_m);
    telemetry_write_f32(&offset, meta.origin_y_m);
    telemetry_write_u32(&offset, stats.update_count);
    telemetry_write_u32(&offset, stats.last_scan_sequence);
    telemetry_write_u32(&offset, tick_ms);
    telemetry_write_u16(&offset, stats.last_usable_points);
    telemetry_write_u16(&offset, stats.last_endpoints_written);
    telemetry_write_u8(&offset, stats.grid_initialized);
    telemetry_write_u8(&offset, stats.robot_inside_grid);
    telemetry_write_u8(&offset, stats.map_update_active);
    telemetry_write_u8(&offset, stats.last_skip_reason);
    telemetry_write_u8(&offset, stats.last_localization_mode);
    telemetry_write_i16(&offset, stats.last_robot_cell.x);
    telemetry_write_i16(&offset, stats.last_robot_cell.y);
    telemetry_write_f32(&offset, stats.last_odom_delta_theta_deg);
    telemetry_write_f32(&offset, stats.last_odom_delta_translation_m);
    telemetry_write_f32(&offset, stats.last_pose.x_m);
    telemetry_write_f32(&offset, stats.last_pose.y_m);
    telemetry_write_f32(&offset, stats.last_pose.theta_deg);
    telemetry_write_u32(&offset, stats.skipped_turning_count);
    telemetry_write_u32(&offset, stats.skipped_settle_count);
    telemetry_write_u32(&offset, stats.skipped_quality_count);
    telemetry_write_u8(&offset, nav_stats.goal_valid);
    telemetry_write_u8(&offset, nav_stats.target_valid);
    telemetry_write_u8(&offset, (uint8_t)nav_stats.last_status);
    telemetry_write_u8(&offset, (uint8_t)nav_stats.phase);
    telemetry_write_u32(&offset, nav_stats.update_count);
    telemetry_write_u32(&offset, nav_stats.fail_count);
    telemetry_write_u16(&offset, nav_stats.raw_path_len);
    telemetry_write_u16(&offset, nav_stats.smooth_path_len);
    telemetry_write_f32(&offset, nav_stats.distance_to_goal_m);
    telemetry_write_f32(&offset, nav_stats.goal_pose.x_m);
    telemetry_write_f32(&offset, nav_stats.goal_pose.y_m);
    telemetry_write_f32(&offset, nav_stats.target_pose.x_m);
    telemetry_write_f32(&offset, nav_stats.target_pose.y_m);
    telemetry_write_u16(&offset, path_point_count);
    for (path_idx = 0U; path_idx < path_point_count; ++path_idx) {
        telemetry_write_f32(&offset, g_telemetryPathBuffer[path_idx].x_m);
        telemetry_write_f32(&offset, g_telemetryPathBuffer[path_idx].y_m);
    }
    telemetry_write_u8(&offset, control_stats.control_mode);
    telemetry_write_u8(&offset, control_stats.relative_move_state);
    telemetry_write_u8(&offset, control_stats.left_direction);
    telemetry_write_u8(&offset, control_stats.right_direction);
    telemetry_write_u16(&offset, control_stats.left_pwm);
    telemetry_write_u16(&offset, control_stats.right_pwm);
    telemetry_write_i16(&offset, control_stats.left_speed_pid_output);
    telemetry_write_i16(&offset, control_stats.right_speed_pid_output);
    telemetry_write_f32(&offset, control_stats.position_pid_output_mps);
    telemetry_write_f32(&offset, control_stats.angle_pid_output_mps);
    telemetry_write_f32(&offset, control_stats.angle_error_deg);
    telemetry_write_f32(&offset, control_stats.position_error_m);
    telemetry_write_f32(&offset, control_stats.base_speed_mps);
    telemetry_write_f32(&offset, control_stats.left_speed_setpoint_mps);
    telemetry_write_f32(&offset, control_stats.right_speed_setpoint_mps);
    telemetry_write_f32(&offset, control_stats.left_speed_feedback_mps);
    telemetry_write_f32(&offset, control_stats.right_speed_feedback_mps);
    (void)memcpy(&g_telemetryFrameBuffer[offset], g_telemetryCellBuffer, cell_count);
    offset = (uint16_t)(offset + cell_count);

    payload_len = (uint16_t)(offset - payload_offset);
    g_telemetryFrameBuffer[6] = (uint8_t)(payload_len & 0xFFU);
    g_telemetryFrameBuffer[7] = (uint8_t)((payload_len >> 8) & 0xFFU);

    crc = telemetry_crc16_ccitt(g_telemetryFrameBuffer, offset);
    telemetry_write_u16(&offset, crc);
    frame_len = offset;
    g_telemetrySequence++;
    return frame_len;
}

void StartTelemetryTask(void *argument)
{
    MappingTaskStats_t stats;
    uint32_t last_sent_update_count = 0U;
    uint32_t last_sent_tick_ms = 0U;
    uint8_t has_sent_frame = 0U;

    (void)argument;
    (void)memset(&stats, 0, sizeof(stats));

    for (;;) {
        uint32_t now_ms = HAL_GetTick();
        uint8_t should_send = 0U;

        MappingTask_GetStatsSnapshot(&stats);

        if (stats.grid_initialized != 0U) {
            if (has_sent_frame == 0U) {
                should_send = 1U;
            } else if ((stats.update_count != last_sent_update_count) &&
                       ((now_ms - last_sent_tick_ms) >= TELEMETRY_TASK_PERIOD_MS)) {
                should_send = 1U;
            } else if ((now_ms - last_sent_tick_ms) >= TELEMETRY_KEEPALIVE_MS) {
                should_send = 1U;
            }
        }

        if ((should_send != 0U) && (telemetry_uart_ready() != 0U)) {
            uint16_t frame_len = telemetry_build_map_frame(now_ms);

            if ((frame_len > 0U) &&
                (HAL_UART_Transmit(&huart5,
                                   g_telemetryFrameBuffer,
                                   frame_len,
                                   TELEMETRY_UART_TIMEOUT_MS) == HAL_OK)) {
                has_sent_frame = 1U;
                last_sent_update_count = stats.update_count;
                last_sent_tick_ms = now_ms;
            }
        }

        osDelay(TELEMETRY_TASK_PERIOD_MS);
    }
}
