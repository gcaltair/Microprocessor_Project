#include <stdarg.h>
#include <math.h>
#include <stdio.h>

#include "freertos_app.h"
#include "localization_task.h"
#include "mapping_task.h"
#include "navigation_task.h"
#include "system.h"

#define CMD_FORWARD                 'F'
#define CMD_BACKWARD                'B'
#define CMD_LEFT                    'L'
#define CMD_RIGHT                   'R'
#define CMD_STOP                    'S'
#define CMD_SPEED                   'V'
#define PID_DEFAULT_TURN_SETPOINT   10.0f
#define UART_PRINTF_BUFFER_SIZE     256
#define UART5_TX_TIMEOUT_MS         1000U
#define STATUS_STREAM_PERIOD_MS     500U
#define STATUS_STREAM_MAP_DOWNSAMPLE 4U
#define TELEMETRY_MAGIC_1           0xC3U
#define TELEMETRY_MAGIC_2           0x3CU
#define TELEMETRY_PROTOCOL_VERSION  1U
#define TELEMETRY_MAX_PAYLOAD_SIZE  2102U
#define TELEMETRY_MAX_FRAME_SIZE    2112U

uint8_t buffer[100];
uint8_t status_enable = 0U;

volatile float speed_magnitude = 0.25f;
static volatile uint8_t g_bt_rx_rearm_pending = 0U;
static char g_uart_printf_buffer[UART_PRINTF_BUFFER_SIZE];
static char g_map_ascii_line_buffer[OGM_MAX_WIDTH_CELLS + 1U];
static uint8_t g_telemetry_frame_buffer[TELEMETRY_MAX_FRAME_SIZE];
static uint16_t g_telemetry_frame_sequence = 0U;

static uint16_t telemetry_crc16_ccitt(const uint8_t *data, uint16_t length)
{
    uint16_t crc = 0xFFFFU;

    if (data == NULL) {
        return crc;
    }

    for (uint16_t idx = 0U; idx < length; ++idx) {
        crc ^= (uint16_t)((uint16_t)data[idx] << 8);
        for (uint8_t bit = 0U; bit < 8U; ++bit) {
            if ((crc & 0x8000U) != 0U) {
                crc = (uint16_t)((crc << 1) ^ 0x1021U);
            } else {
                crc <<= 1;
            }
        }
    }

    return crc;
}

static uint16_t normalize_command(uint8_t *cmd, uint16_t size)
{
    uint16_t start = 0U;

    if (cmd == NULL) {
        return 0U;
    }

    while ((start < size) &&
           ((cmd[start] == '\r') || (cmd[start] == '\n') || (cmd[start] == ' ') || (cmd[start] == '\t'))) {
        start++;
    }

    while (size > start) {
        uint8_t tail = cmd[size - 1U];
        if ((tail != '\r') && (tail != '\n') && (tail != ' ') && (tail != '\t') && (tail != '\0')) {
            break;
        }
        size--;
    }

    if (start > 0U) {
        uint16_t dst = 0U;
        while ((start + dst) < size) {
            cmd[dst] = cmd[start + dst];
            dst++;
        }
        size = dst;
    } else {
        size = (uint16_t)(size - start);
    }

    return size;
}

static HAL_StatusTypeDef wait_for_uart5_tx_ready(uint32_t timeout_ms)
{
    uint32_t start_tick = HAL_GetTick();
    uint8_t waited = 0U;

    while (huart5.gState != HAL_UART_STATE_READY) {
        if (waited == 0U) {
            Freertos_RecordBluetoothTxWait();
            waited = 1U;
        }

        if ((HAL_GetTick() - start_tick) >= timeout_ms) {
            return HAL_TIMEOUT;
        }

        if (osKernelGetState() == osKernelRunning) {
            osDelay(1U);
        } else {
            HAL_Delay(1U);
        }
    }

    return HAL_OK;
}

static HAL_StatusTypeDef start_uart5_rx_to_idle_dma(void)
{
    HAL_StatusTypeDef status;

    status = HAL_UARTEx_ReceiveToIdle_DMA(&huart5, buffer, sizeof(buffer));
    if (status == HAL_OK) {
        __HAL_DMA_DISABLE_IT(huart5.hdmarx, DMA_IT_HT);
        g_bt_rx_rearm_pending = 0U;
    }

    return status;
}

static void try_rearm_uart5_rx_dma(void)
{
    if (g_bt_rx_rearm_pending == 0U) {
        return;
    }

    if (start_uart5_rx_to_idle_dma() != HAL_OK) {
        g_bt_rx_rearm_pending = 1U;
    }
}

static void transmit_buffer(const uint8_t *message, uint16_t size)
{
    if ((message == NULL) || (size == 0U)) {
        return;
    }

    if (wait_for_uart5_tx_ready(200U) == HAL_OK) {
        (void)HAL_UART_Transmit(&huart5, (uint8_t *)message, size, UART5_TX_TIMEOUT_MS);
        try_rearm_uart5_rx_dma();
    }
}

void HC04_SendTelemetryFrame(uint8_t frame_type, const uint8_t *payload, uint16_t payload_len)
{
    uint16_t crc;
    uint16_t offset = 0U;

    if (payload_len > TELEMETRY_MAX_PAYLOAD_SIZE) {
        return;
    }

    if ((uint16_t)(payload_len + 10U) > TELEMETRY_MAX_FRAME_SIZE) {
        return;
    }

    g_telemetry_frame_buffer[offset++] = TELEMETRY_MAGIC_1;
    g_telemetry_frame_buffer[offset++] = TELEMETRY_MAGIC_2;
    g_telemetry_frame_buffer[offset++] = TELEMETRY_PROTOCOL_VERSION;
    g_telemetry_frame_buffer[offset++] = (uint8_t)frame_type;
    g_telemetry_frame_buffer[offset++] = (uint8_t)(g_telemetry_frame_sequence & 0xFFU);
    g_telemetry_frame_buffer[offset++] = (uint8_t)((g_telemetry_frame_sequence >> 8) & 0xFFU);
    g_telemetry_frame_buffer[offset++] = (uint8_t)(payload_len & 0xFFU);
    g_telemetry_frame_buffer[offset++] = (uint8_t)((payload_len >> 8) & 0xFFU);

    if ((payload != NULL) && (payload_len > 0U)) {
        (void)memcpy(&g_telemetry_frame_buffer[offset], payload, payload_len);
        offset = (uint16_t)(offset + payload_len);
    }

    crc = telemetry_crc16_ccitt(g_telemetry_frame_buffer, offset);
    g_telemetry_frame_buffer[offset++] = (uint8_t)(crc & 0xFFU);
    g_telemetry_frame_buffer[offset++] = (uint8_t)((crc >> 8) & 0xFFU);
    g_telemetry_frame_sequence++;

    transmit_buffer(g_telemetry_frame_buffer, offset);
}

void HC04_RecordCommandAck(const uint8_t *cmd, uint16_t size, uint8_t ok, const char *detail)
{
    uint8_t payload[192];
    uint16_t cmd_len = 0U;
    uint16_t detail_len = 0U;
    TelemetryFrameType_t frame_type = (ok != 0U) ? TELEMETRY_FRAME_ACK_V2 : TELEMETRY_FRAME_ERR_V2;

    if (Freertos_GetTelemetryStreamingEnabled() == 0U) {
        return;
    }

    if ((cmd != NULL) && (size > 0U)) {
        cmd_len = (size > 48U) ? 48U : size;
    }

    if (detail != NULL) {
        detail_len = (uint16_t)strlen(detail);
        if (detail_len > 120U) {
            detail_len = 120U;
        }
    }

    payload[0] = ok;
    payload[1] = (uint8_t)cmd_len;
    payload[2] = (uint8_t)(detail_len & 0xFFU);
    payload[3] = (uint8_t)((detail_len >> 8) & 0xFFU);
    if (cmd_len > 0U) {
        (void)memcpy(&payload[4], cmd, cmd_len);
    }
    if (detail_len > 0U) {
        (void)memcpy(&payload[4 + cmd_len], detail, detail_len);
    }

    HC04_SendTelemetryFrame(frame_type, payload, (uint16_t)(4U + cmd_len + detail_len));
}

static const char *control_mode_to_string(ControlMode mode)
{
    return (mode == CONTROL_MODE_POSITION) ? "POSITION" : "MANUAL";
}

static const char *move_state_to_string(RelativeMoveState state)
{
    switch (state) {
        case RELATIVE_MOVE_TURNING:
            return "TURNING";
        case RELATIVE_MOVE_DRIVING:
            return "DRIVING";
        case RELATIVE_MOVE_IDLE:
        default:
            return "IDLE";
    }
}

static const char *navigation_state_to_string(NavigationState_t state)
{
    switch (state) {
        case NAVIGATION_STATE_ACTIVE:
            return "ACTIVE";
        case NAVIGATION_STATE_GOAL_REACHED:
            return "REACHED";
        case NAVIGATION_STATE_FAILED:
            return "FAILED";
        case NAVIGATION_STATE_CANCELLED:
            return "CANCELLED";
        case NAVIGATION_STATE_IDLE:
        default:
            return "IDLE";
    }
}

static void transmit_odometry_snapshot(void)
{
    float x;
    float y;
    float th;
    float left_distance_m;
    float right_distance_m;
    LocalizationTaskStats_t loc_stats;
    float left_speed;
    float right_speed;
    float current_base_speed;
    float angle_setpoint;
    ControlMode mode;
    RelativeMoveState move_state;
    int16_t left_counter_raw;
    int16_t right_counter_raw;

    if (g_odomMutex != NULL) {
        (void)osMutexAcquire(g_odomMutex, osWaitForever);
    }
    if (g_controlMutex != NULL) {
        (void)osMutexAcquire(g_controlMutex, osWaitForever);
    }
    if (g_pidMutex != NULL) {
        (void)osMutexAcquire(g_pidMutex, osWaitForever);
    }

    x = g_x;
    y = g_y;
    th = g_th_continuous;
    left_speed = g_left_speed;
    right_speed = g_right_speed;
    Encoder_GetTravelSnapshot(&left_distance_m,
                              &right_distance_m,
                              &left_counter_raw,
                              &right_counter_raw);
    current_base_speed = base_car_speed;
    angle_setpoint = g_pid_angle.setpoint;
    mode = g_control_mode;
    move_state = g_relative_move_state;

    if (g_pidMutex != NULL) {
        (void)osMutexRelease(g_pidMutex);
    }
    if (g_controlMutex != NULL) {
        (void)osMutexRelease(g_controlMutex);
    }
    if (g_odomMutex != NULL) {
        (void)osMutexRelease(g_odomMutex);
    }

    LocalizationTask_GetStatsSnapshot(&loc_stats);
    uart_printf("ODOM x=%.3f y=%.3f th=%.2f ls=%.3f rs=%.3f base=%.3f ang_sp=%.2f mode=%s move=%s\r\n",
                x,
                y,
                th,
                left_speed,
                right_speed,
                current_base_speed,
                angle_setpoint,
                control_mode_to_string(mode),
                move_state_to_string(move_state));
    uart_printf("ENC  dl=%.3f dr=%.3f cntL=%d cntR=%d cal=(%.3f,%.3f,%.3f,%.3f)\r\n",
                left_distance_m,
                right_distance_m,
                left_counter_raw,
                right_counter_raw,
                g_encoder_left_forward_scale,
                g_encoder_left_reverse_scale,
                g_encoder_right_forward_scale,
                g_encoder_right_reverse_scale);
    uart_printf("EST  x=%.3f y=%.3f th=%.2f CTRL=(%.3f,%.3f,%.2f)\r\n",
                loc_stats.current_estimated_pose.x_m,
                loc_stats.current_estimated_pose.y_m,
                loc_stats.current_estimated_pose.theta_deg,
                loc_stats.current_control_pose.x_m,
                loc_stats.current_control_pose.y_m,
                loc_stats.current_control_pose.theta_deg);
    uart_printf("POSE pred=(%.3f,%.3f,%.2f) corr=(%.3f,%.3f,%.2f) slow=(%.3f,%.3f,%.2f)\r\n",
                loc_stats.last_predicted_pose.x_m,
                loc_stats.last_predicted_pose.y_m,
                loc_stats.last_predicted_pose.theta_deg,
                loc_stats.last_corrected_pose.x_m,
                loc_stats.last_corrected_pose.y_m,
                loc_stats.last_corrected_pose.theta_deg,
                loc_stats.last_control_correction_delta.x_m,
                loc_stats.last_control_correction_delta.y_m,
                loc_stats.last_control_correction_delta.theta_deg);
}

static void transmit_encoder_calibration_suggestion(float actual_distance_m)
{
    float left_distance_m = 0.0f;
    float right_distance_m = 0.0f;
    float left_forward = 0.0f;
    float left_reverse = 0.0f;
    float right_forward = 0.0f;
    float right_reverse = 0.0f;
    float suggested_left_scale;
    float suggested_right_scale;

    if (fabsf(actual_distance_m) < 0.001f) {
        transmit("Calibration suggestion requires non-zero actual distance\r\n");
        return;
    }

    Encoder_GetTravelSnapshot(&left_distance_m, &right_distance_m, NULL, NULL);
    Encoder_GetCalibration(&left_forward, &left_reverse, &right_forward, &right_reverse);

    if ((fabsf(left_distance_m) < 0.001f) || (fabsf(right_distance_m) < 0.001f)) {
        transmit("Calibration suggestion unavailable: wheel travel too small\r\n");
        return;
    }

    suggested_left_scale = (actual_distance_m / left_distance_m) *
                           ((actual_distance_m >= 0.0f) ? left_forward : left_reverse);
    suggested_right_scale = (actual_distance_m / right_distance_m) *
                            ((actual_distance_m >= 0.0f) ? right_forward : right_reverse);

    if (actual_distance_m >= 0.0f) {
        uart_printf("CAL suggest forward: left=%.3f right=%.3f from actual=%.3f dl=%.3f dr=%.3f\r\n",
                    suggested_left_scale,
                    suggested_right_scale,
                    actual_distance_m,
                    left_distance_m,
                    right_distance_m);
        uart_printf("Use: K%.3f,%.3f,%.3f,%.3f\r\n",
                    suggested_left_scale,
                    left_reverse,
                    suggested_right_scale,
                    right_reverse);
    } else {
        uart_printf("CAL suggest reverse: left=%.3f right=%.3f from actual=%.3f dl=%.3f dr=%.3f\r\n",
                    suggested_left_scale,
                    suggested_right_scale,
                    actual_distance_m,
                    left_distance_m,
                    right_distance_m);
        uart_printf("Use: K%.3f,%.3f,%.3f,%.3f\r\n",
                    left_forward,
                    suggested_left_scale,
                    right_forward,
                    suggested_right_scale);
    }
}

static void transmit_runtime_snapshot(void)
{
    FreertosRuntimeStats_t stats;
    LocalizationTaskStats_t loc_stats;
    NavigationTaskStats_t nav_stats;
    uint8_t lidar_binary_tx_enabled;
    uint8_t lidar_active;

    Freertos_GetRuntimeStatsSnapshot(&stats);
    LocalizationTask_GetStatsSnapshot(&loc_stats);
    NavigationTask_GetStatsSnapshot(&nav_stats);
    lidar_binary_tx_enabled = Freertos_GetLidarBinaryTxEnabled();
    lidar_active = lidar_raw_stream_active;
    uart_printf("RTOS ctrl=%lu overrun=%lu cmd=%lu drop=%lu dma=%lu dma_drop=%lu scan=%lu tx=%lu busy=%lu err=%lu wait=%lu\r\n",
                (unsigned long)stats.control_cycles,
                (unsigned long)stats.control_tick_overruns,
                (unsigned long)stats.cmd_rx_count,
                (unsigned long)stats.cmd_drop_count,
                (unsigned long)stats.lidar_dma_block_count,
                (unsigned long)stats.lidar_dma_drop_count,
                (unsigned long)stats.lidar_scan_complete_count,
                (unsigned long)stats.lidar_tx_count,
                (unsigned long)stats.lidar_tx_busy_count,
                (unsigned long)stats.lidar_tx_error_count,
                (unsigned long)stats.bt_tx_wait_count);
    uart_printf("LIDAR active=%u binary_tx=%u\r\n", lidar_active, lidar_binary_tx_enabled);
    uart_printf("HEAP free=%lu min=%lu\r\n",
                (unsigned long)stats.free_heap_bytes,
                (unsigned long)stats.min_ever_free_heap_bytes);
    uart_printf("STACK freeB dft=%lu ctrl=%lu lidar=%lu loc=%lu map=%lu comm=%lu safe=%lu\r\n",
                (unsigned long)stats.default_task_stack_free_bytes,
                (unsigned long)stats.control_task_stack_free_bytes,
                (unsigned long)stats.lidar_task_stack_free_bytes,
                (unsigned long)stats.localization_task_stack_free_bytes,
                (unsigned long)stats.mapping_task_stack_free_bytes,
                (unsigned long)stats.comm_task_stack_free_bytes,
                (unsigned long)stats.safety_task_stack_free_bytes);
    uart_printf("SCAN raw=%u usable=%u rej_range=%u rej_quality=%u min_mm=%u max_mm=%u\r\n",
                stats.last_scan_raw_point_count,
                stats.last_scan_usable_point_count,
                stats.last_scan_rejected_range_count,
                stats.last_scan_rejected_quality_count,
                stats.last_scan_min_distance_mm,
                stats.last_scan_max_distance_mm);
    uart_printf("LOC init=%u updates=%lu accept=%lu reject=%lu odom=%lu mode=%u pts=%u/%u inliers=%u fit_mm=%.1f slow_mm=(%.1f,%.1f) slow_deg=%.2f\r\n",
                loc_stats.initialized,
                (unsigned long)loc_stats.update_count,
                (unsigned long)loc_stats.icp_accept_count,
                (unsigned long)loc_stats.icp_reject_count,
                (unsigned long)loc_stats.odom_only_count,
                (unsigned int)loc_stats.last_mode,
                (unsigned int)loc_stats.last_reference_points,
                (unsigned int)loc_stats.last_current_points,
                (unsigned int)loc_stats.last_inliers,
                (double)(loc_stats.last_fitness_m * 1000.0f),
                (double)(loc_stats.last_control_correction_delta.x_m * 1000.0f),
                (double)(loc_stats.last_control_correction_delta.y_m * 1000.0f),
                (double)loc_stats.last_control_correction_delta.theta_deg);
    uart_printf("NAV state=%s goal=(%d,%d) step=%u/%u dist=%.2f plans=%lu done=%lu fail=%lu\r\n",
                navigation_state_to_string(nav_stats.state),
                nav_stats.goal_cell.x,
                nav_stats.goal_cell.y,
                (unsigned int)nav_stats.current_waypoint_index,
                (unsigned int)nav_stats.path_length,
                nav_stats.last_waypoint_distance_m,
                (unsigned long)nav_stats.plan_count,
                (unsigned long)nav_stats.completion_count,
                (unsigned long)nav_stats.failure_count);
}

static const char *mapping_skip_reason_to_string(uint8_t reason)
{
    switch ((MappingSkipReason_t)reason) {
        case MAPPING_SKIP_REASON_TURNING:
            return "paused(turning)";
        case MAPPING_SKIP_REASON_SETTLE:
            return "paused(settle)";
        case MAPPING_SKIP_REASON_QUALITY:
            return "paused(quality)";
        case MAPPING_SKIP_REASON_NONE:
        default:
            return "active";
    }
}

static void transmit_mapping_snapshot(void)
{
    MappingTaskStats_t stats;
    NavigationTaskStats_t nav_stats;

    MappingTask_GetStatsSnapshot(&stats);
    NavigationTask_GetStatsSnapshot(&nav_stats);
    uart_printf("MAP init=%u inside=%u updates=%lu seq=%lu usable=%u written=%u cell=%d,%d pose=(%.3f,%.3f,%.2f)\r\n",
                stats.grid_initialized,
                stats.robot_inside_grid,
                (unsigned long)stats.update_count,
                (unsigned long)stats.last_scan_sequence,
                stats.last_usable_points,
                stats.last_endpoints_written,
                stats.last_robot_cell.x,
                stats.last_robot_cell.y,
                stats.last_pose.x_m,
                stats.last_pose.y_m,
                stats.last_pose.theta_deg);
    uart_printf("MAP loc mode=%u inliers=%u fit_mm=%.1f\r\n",
                (unsigned int)stats.last_localization_mode,
                (unsigned int)stats.last_localization_inliers,
                (double)(stats.last_localization_fitness_m * 1000.0f));
    uart_printf("MAP gate=%s dxy_mm=%.1f dth=%.2f skip turn=%lu settle=%lu quality=%lu\r\n",
                mapping_skip_reason_to_string(stats.last_skip_reason),
                (double)(stats.last_odom_delta_translation_m * 1000.0f),
                (double)stats.last_odom_delta_theta_deg,
                (unsigned long)stats.skipped_turning_count,
                (unsigned long)stats.skipped_settle_count,
                (unsigned long)stats.skipped_quality_count);
    uart_printf("MAP nav state=%s goal=(%d,%d) step=%u/%u\r\n",
                navigation_state_to_string(nav_stats.state),
                nav_stats.goal_cell.x,
                nav_stats.goal_cell.y,
                (unsigned int)nav_stats.current_waypoint_index,
                (unsigned int)nav_stats.path_length);
}

static void transmit_live_map_snapshot(void)
{
    float x;
    float y;
    float th;
    ControlMode mode;
    RelativeMoveState move_state;
    LocalizationTaskStats_t loc_stats;
    MappingTaskStats_t map_stats;
    uint16_t width = 0U;
    uint16_t height = 0U;
    uint16_t row;

    if (g_odomMutex != NULL) {
        (void)osMutexAcquire(g_odomMutex, osWaitForever);
    }
    if (g_controlMutex != NULL) {
        (void)osMutexAcquire(g_controlMutex, osWaitForever);
    }
    if (g_pidMutex != NULL) {
        (void)osMutexAcquire(g_pidMutex, osWaitForever);
    }

    x = g_x;
    y = g_y;
    th = g_th_continuous;
    mode = g_control_mode;
    move_state = g_relative_move_state;

    if (g_pidMutex != NULL) {
        (void)osMutexRelease(g_pidMutex);
    }
    if (g_controlMutex != NULL) {
        (void)osMutexRelease(g_controlMutex);
    }
    if (g_odomMutex != NULL) {
        (void)osMutexRelease(g_odomMutex);
    }

    LocalizationTask_GetStatsSnapshot(&loc_stats);
    MappingTask_GetStatsSnapshot(&map_stats);
    MappingTask_GetRenderDimensions(STATUS_STREAM_MAP_DOWNSAMPLE, &width, &height);

    uart_printf("MAP LIVE ds=%u size=%ux%u cell=(%d,%d) odom=(%.3f,%.3f,%.2f) ctrl=(%.3f,%.3f,%.2f) mode=%s/%s loc=%u inliers=%u fit=%.1f\r\n",
                STATUS_STREAM_MAP_DOWNSAMPLE,
                width,
                height,
                map_stats.last_robot_cell.x,
                map_stats.last_robot_cell.y,
                x,
                y,
                th,
                loc_stats.current_control_pose.x_m,
                loc_stats.current_control_pose.y_m,
                loc_stats.current_control_pose.theta_deg,
                control_mode_to_string(mode),
                move_state_to_string(move_state),
                (unsigned int)loc_stats.last_mode,
                (unsigned int)loc_stats.last_inliers,
                (double)(loc_stats.last_fitness_m * 1000.0f));

    for (row = 0U; row < height; ++row) {
        if (MappingTask_RenderAsciiRow(row,
                                       STATUS_STREAM_MAP_DOWNSAMPLE,
                                       g_map_ascii_line_buffer,
                                       sizeof(g_map_ascii_line_buffer)) == 0U) {
            transmit("MAP LIVE render failed\r\n");
            return;
        }

        uart_printf("|%s|\r\n", g_map_ascii_line_buffer);
    }
}

static void transmit_mapping_ascii(uint8_t downsample)
{
    uint16_t width = 0U;
    uint16_t height = 0U;
    uint16_t row;

    if (downsample == 0U) {
        downsample = 2U;
    }

    MappingTask_GetRenderDimensions(downsample, &width, &height);
    uart_printf("MAP ASCII ds=%u size=%ux%u legend: # occupied, . free, space unknown, R robot\r\n",
                downsample,
                width,
                height);

    for (row = 0U; row < height; ++row) {
        if (MappingTask_RenderAsciiRow(row,
                                       downsample,
                                       g_map_ascii_line_buffer,
                                       sizeof(g_map_ascii_line_buffer)) == 0U) {
            transmit("MAP render failed\r\n");
            return;
        }

        uart_printf("|%s|\r\n", g_map_ascii_line_buffer);
        if (osKernelGetState() == osKernelRunning) {
            osDelay(1U);
        }
    }

    transmit("MAP dump done\r\n");
}

void process_command(uint8_t *cmd, uint16_t size)
{
    if (cmd == NULL) {
        return;
    }

    size = normalize_command(cmd, size);
    if (size == 0U) {
        return;
    }

    transmit_uint8(cmd, (uint8_t)size);

    if (size > 1U) {
        process_complex_command(cmd, size);
        return;
    }

    switch (cmd[0]) {
        case CMD_FORWARD:
            Control_SetManualDrive(speed_magnitude);
            transmit("Moving forward (PID)\r\n");
            HC04_RecordCommandAck(cmd, size, 1U, "manual-forward");
            break;

        case CMD_BACKWARD:
            Control_SetManualDrive(-speed_magnitude);
            transmit("Moving backward (PID)\r\n");
            HC04_RecordCommandAck(cmd, size, 1U, "manual-backward");
            break;

        case CMD_LEFT:
            Control_SetManualCommand(0.0f, -PID_DEFAULT_TURN_SETPOINT);
            transmit("Turning left (PID)\r\n");
            HC04_RecordCommandAck(cmd, size, 1U, "manual-left");
            break;

        case CMD_RIGHT:
            Control_SetManualCommand(0.0f, PID_DEFAULT_TURN_SETPOINT);
            transmit("Turning right (PID)\r\n");
            HC04_RecordCommandAck(cmd, size, 1U, "manual-right");
            break;

        case CMD_STOP:
            Control_StopCommand();
            transmit("Stopped (PID)\r\n");
            HC04_RecordCommandAck(cmd, size, 1U, "manual-stop");
            break;

        case 'N':
            transmit("Stopping lidar...\r\n");
            Freertos_SetLidarBinaryTxEnabled(0U);
            Freertos_SetTelemetryScanStreamingEnabled(0U);
            RPLIDAR_StopRaw();
            transmit("Lidar Stopped\r\n");
            HC04_RecordCommandAck(cmd, size, 1U, "lidar-stop");
            break;

        case 'M':
            Freertos_SetLidarBinaryTxEnabled(0U);
            RPLIDAR_StartRaw();
            transmit("Lidar Started (mapping mode)\r\n");
            HC04_RecordCommandAck(cmd, size, 1U, "lidar-start");
            break;

        case 'H':
            transmit("\r\n--- Bluetooth PID Control ---\r\n");
            transmit("F/B/L/R/S: Manual Control\r\n");
            transmit("G: Show mapping/grid snapshot\r\n");
            transmit("X: Dump ASCII map (downsample 4)\r\n");
            transmit("Z: Clear occupancy grid\r\n");
            transmit("A+angle: Set relative angle turn\r\n");
            transmit("V+speed: Set speed for manual mode\r\n");
            transmit("P{x},{y}: Set relative target point (e.g., P0.5,1.2)\r\n");
            transmit("J: Show navigation status\r\n");
            transmit("Jx,y: Navigate to map cell x,y\r\n");
            transmit("C: Cancel active navigation\r\n");
            transmit("O: Show odometry snapshot\r\n");
            transmit("P: Show RTOS/runtime stats and scan quality\r\n");
            transmit("K: Show encoder calibration\r\n");
            transmit("Klf,lr,rf,rr: Set encoder calibration\r\n");
            transmit("Dvalue: Suggest new wheel calibration from actual travel distance in meters\r\n");
            transmit("R0: Reset encoder counts and odometry pose\r\n");
            transmit("M: Start LIDAR (mapping mode, no binary stream)\r\n");
            transmit("N: Stop LIDAR\r\n");
            transmit("T0/T1: Disable/enable binary LiDAR telemetry on Bluetooth\r\n");
            transmit("Y0/Y1: Disable/enable structured telemetry frames\r\n");
            transmit("Q0/Q1: Disable/enable structured scan telemetry frames\r\n");
            transmit("B9600/B115200/B921600: Switch UART5 baud rate\r\n");
            transmit("U0/U1: Disable/enable 0.5 s live thumbnail map stream\r\n");
            transmit("H: Show This Help\r\n");
            HC04_RecordCommandAck(cmd, size, 1U, "help");
            break;

        case 'O':
            transmit_odometry_snapshot();
            HC04_RecordCommandAck(cmd, size, 1U, "odometry");
            break;

        case 'P':
            transmit_runtime_snapshot();
            HC04_RecordCommandAck(cmd, size, 1U, "runtime");
            break;

        case 'G':
            transmit_mapping_snapshot();
            HC04_RecordCommandAck(cmd, size, 1U, "mapping");
            break;

        case 'K':
        {
            float left_forward;
            float left_reverse;
            float right_forward;
            float right_reverse;

            Encoder_GetCalibration(&left_forward, &left_reverse, &right_forward, &right_reverse);
            uart_printf("ENC cal lf=%.3f lr=%.3f rf=%.3f rr=%.3f\r\n",
                        left_forward,
                        left_reverse,
                        right_forward,
                        right_reverse);
            HC04_RecordCommandAck(cmd, size, 1U, "encoder-calibration");
            break;
        }

        case 'J':
        {
            NavigationTaskStats_t nav_stats;

            NavigationTask_GetStatsSnapshot(&nav_stats);
            uart_printf("NAV state=%s goal=(%d,%d) step=%u/%u dist=%.2f plans=%lu done=%lu fail=%lu\r\n",
                        navigation_state_to_string(nav_stats.state),
                        nav_stats.goal_cell.x,
                        nav_stats.goal_cell.y,
                        (unsigned int)nav_stats.current_waypoint_index,
                        (unsigned int)nav_stats.path_length,
                        nav_stats.last_waypoint_distance_m,
                        (unsigned long)nav_stats.plan_count,
                        (unsigned long)nav_stats.completion_count,
                        (unsigned long)nav_stats.failure_count);
            HC04_RecordCommandAck(cmd, size, 1U, "navigation-status");
            break;
        }

        case 'X':
            transmit_mapping_ascii(4U);
            HC04_RecordCommandAck(cmd, size, 1U, "map-ascii");
            break;

        case 'Z':
            MappingTask_ResetGrid();
            NavigationTask_Reset();
            transmit("Occupancy grid reset\r\n");
            HC04_RecordCommandAck(cmd, size, 1U, "grid-reset");
            break;

        case 'C':
            NavigationTask_Cancel();
            transmit("Navigation cancelled\r\n");
            HC04_RecordCommandAck(cmd, size, 1U, "navigation-cancel");
            break;

        default:
            transmit("Unknown command. Send 'H' for help\r\n");
            HC04_RecordCommandAck(cmd, size, 0U, "unknown-command");
            break;
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef *huart, uint16_t size)
{
    if (huart->Instance == UART5) {
        if (size > 0U) {
            (void)Freertos_SubmitCommandFromISR(buffer, size);
        }

        if (HAL_UARTEx_ReceiveToIdle_DMA(&huart5, buffer, sizeof(buffer)) == HAL_OK) {
            __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
            g_bt_rx_rearm_pending = 0U;
        } else {
            g_bt_rx_rearm_pending = 1U;
        }
    }
}

void hc04_init(void)
{
    if (start_uart5_rx_to_idle_dma() != HAL_OK) {
        g_bt_rx_rearm_pending = 1U;
    }
    HAL_Delay(1000);
    transmit("Bluetooth Car Control Ready\r\n");
}

void transmit(char *message)
{
    if (message != NULL) {
        transmit_buffer((const uint8_t *)message, (uint16_t)strlen(message));
    }
}

void transmit_uint8(uint8_t *message, uint8_t size)
{
    if ((message != NULL) && (size > 0U)) {
        transmit_buffer(message, size);
    }
}

void process_complex_command(uint8_t *cmd, uint16_t size)
{
    char reply[100];

    if ((size >= 2U) && (cmd[0] == CMD_SPEED)) {
        float int_part = 0.0f;
        float frac_part = 0.0f;
        float frac_divisor = 1.0f;
        int parsing_frac = 0;

        for (uint16_t i = 1U; i < size; ++i) {
            if ((cmd[i] >= '0') && (cmd[i] <= '9')) {
                if (!parsing_frac) {
                    int_part = int_part * 10.0f + (float)(cmd[i] - '0');
                } else {
                    frac_part = frac_part * 10.0f + (float)(cmd[i] - '0');
                    frac_divisor *= 10.0f;
                }
            } else if ((cmd[i] == '.') && !parsing_frac) {
                parsing_frac = 1;
            } else {
                transmit("Invalid speed format. Use V+number (e.g., V5.25)\r\n");
                HC04_RecordCommandAck(cmd, size, 0U, "invalid-speed-format");
                return;
            }
        }

        speed_magnitude = int_part + (frac_part / frac_divisor);

        if ((speed_magnitude >= 0.0f) && (speed_magnitude <= 10.0f)) {
            Control_SetBaseSpeed(speed_magnitude);
            (void)snprintf(reply, sizeof(reply), "base_car_speed is %.2f\r\n", speed_magnitude);
            transmit(reply);
            HC04_RecordCommandAck(cmd, size, 1U, "speed-updated");
        } else {
            transmit("Invalid speed value. Use 0.00-10.00\r\n");
            HC04_RecordCommandAck(cmd, size, 0U, "invalid-speed-value");
        }
    } else if ((size >= 2U) && (cmd[0] == 'A')) {
        float sign = 1.0f;
        float int_part = 0.0f;
        float frac_part = 0.0f;
        float frac_divisor = 1.0f;
        int parsing_frac = 0;
        uint16_t start_index = 1U;

        if (cmd[1] == '-') {
            sign = -1.0f;
            start_index = 2U;
        }

        for (uint16_t i = start_index; i < size; ++i) {
            if ((cmd[i] >= '0') && (cmd[i] <= '9')) {
                if (!parsing_frac) {
                    int_part = int_part * 10.0f + (float)(cmd[i] - '0');
                } else {
                    frac_part = frac_part * 10.0f + (float)(cmd[i] - '0');
                    frac_divisor *= 10.0f;
                }
            } else if ((cmd[i] == '.') && !parsing_frac) {
                parsing_frac = 1;
            } else {
                transmit("Invalid angle format. Use A+number (e.g., A-90.5)\r\n");
                HC04_RecordCommandAck(cmd, size, 0U, "invalid-angle-format");
                return;
            }
        }

        float delta_angle = (int_part + (frac_part / frac_divisor)) * sign;
        if ((delta_angle >= -360.0f) && (delta_angle <= 360.0f)) {
            Control_SetRelativeTurn(delta_angle);
            (void)snprintf(reply, sizeof(reply), "Relative turn %.2f degrees\r\n", delta_angle);
            transmit(reply);
            HC04_RecordCommandAck(cmd, size, 1U, "relative-turn");
        } else {
            transmit("Invalid angle value. Use -360.0 to 360.0\r\n");
            HC04_RecordCommandAck(cmd, size, 0U, "invalid-angle-value");
        }
    } else if ((size > 2U) && (cmd[0] == 'K')) {
        float left_forward = 0.0f;
        float left_reverse = 0.0f;
        float right_forward = 0.0f;
        float right_reverse = 0.0f;

        if (sscanf((char *)cmd, "K%f,%f,%f,%f",
                   &left_forward,
                   &left_reverse,
                   &right_forward,
                   &right_reverse) == 4) {
            if (Encoder_SetCalibration(left_forward, left_reverse, right_forward, right_reverse) != 0U) {
                uart_printf("ENC cal set lf=%.3f lr=%.3f rf=%.3f rr=%.3f\r\n",
                            left_forward,
                            left_reverse,
                            right_forward,
                            right_reverse);
                HC04_RecordCommandAck(cmd, size, 1U, "encoder-calibration-updated");
            } else {
                transmit("Invalid calibration values. Use positive numbers\r\n");
                HC04_RecordCommandAck(cmd, size, 0U, "invalid-calibration-values");
            }
        } else {
            transmit("Invalid K format. Use Klf,lr,rf,rr\r\n");
            HC04_RecordCommandAck(cmd, size, 0U, "invalid-k-format");
        }
    } else if ((size > 2U) && (cmd[0] == 'D')) {
        float actual_distance_m = 0.0f;

        if (sscanf((char *)cmd, "D%f", &actual_distance_m) == 1) {
            transmit_encoder_calibration_suggestion(actual_distance_m);
            HC04_RecordCommandAck(cmd, size, 1U, "encoder-calibration-suggestion");
        } else {
            transmit("Invalid D format. Use D0.95 or D-0.92\r\n");
            HC04_RecordCommandAck(cmd, size, 0U, "invalid-d-format");
        }
    } else if ((size > 2U) && (cmd[0] == 'P')) {
        float dx = 0.0f;
        float dy = 0.0f;

        if (sscanf((char *)cmd, "P%f,%f", &dx, &dy) == 2) {
            Start_Relative_Move(dx, dy);
            uart_printf("Relative move started to %f,%f\r\n", dx, dy);
            HC04_RecordCommandAck(cmd, size, 1U, "relative-move");
        } else {
            transmit("Invalid format. Use P{dx},{dy}\r\n");
            HC04_RecordCommandAck(cmd, size, 0U, "invalid-p-format");
        }
    } else if ((size > 2U) && (cmd[0] == 'J')) {
        int goal_x = 0;
        int goal_y = 0;

        if (sscanf((char *)cmd, "J%d,%d", &goal_x, &goal_y) == 2) {
            if (NavigationTask_StartGoalCell((int16_t)goal_x, (int16_t)goal_y) != 0U) {
                uart_printf("Navigation started to cell %d,%d\r\n", goal_x, goal_y);
                HC04_RecordCommandAck(cmd, size, 1U, "navigation-started");
            } else {
                transmit("Navigation planning failed\r\n");
                HC04_RecordCommandAck(cmd, size, 0U, "navigation-planning-failed");
            }
        } else {
            transmit("Invalid J format. Use Jx,y\r\n");
            HC04_RecordCommandAck(cmd, size, 0U, "invalid-j-format");
        }
    } else if ((size == 2U) && (cmd[0] == 'R') && (cmd[1] == '0')) {
        if (g_odomMutex != NULL) {
            (void)osMutexAcquire(g_odomMutex, osWaitForever);
        }
        encoder_Reset();
        Odometry_ResetPose();
        if (g_odomMutex != NULL) {
            (void)osMutexRelease(g_odomMutex);
        }
        transmit("Odometry and encoder accumulators reset\r\n");
        HC04_RecordCommandAck(cmd, size, 1U, "odometry-reset");
    } else if ((size >= 2U) && (cmd[0] == 'X')) {
        int downsample = 0;

        if (sscanf((char *)cmd, "X%d", &downsample) == 1) {
            if ((downsample >= 1) && (downsample <= 8)) {
                transmit_mapping_ascii((uint8_t)downsample);
                HC04_RecordCommandAck(cmd, size, 1U, "map-ascii");
            } else {
                transmit("Invalid X value. Use X1..X8\r\n");
                HC04_RecordCommandAck(cmd, size, 0U, "invalid-x-value");
            }
        } else {
            transmit("Invalid map format. Use X or X2\r\n");
            HC04_RecordCommandAck(cmd, size, 0U, "invalid-x-format");
        }
    } else if ((size >= 2U) && (cmd[0] == 'T')) {
        if ((cmd[1] == '0') && (size == 2U)) {
            Freertos_SetLidarBinaryTxEnabled(0U);
            transmit("Lidar binary telemetry disabled\r\n");
            HC04_RecordCommandAck(cmd, size, 1U, "lidar-binary-off");
        } else if ((cmd[1] == '1') && (size == 2U)) {
            Freertos_SetLidarBinaryTxEnabled(1U);
            transmit("Lidar binary telemetry enabled\r\n");
            HC04_RecordCommandAck(cmd, size, 1U, "lidar-binary-on");
        } else {
            transmit("Invalid T format. Use T0 or T1\r\n");
            HC04_RecordCommandAck(cmd, size, 0U, "invalid-t-format");
        }
    } else if ((size >= 2U) && (cmd[0] == 'Y')) {
        if ((cmd[1] == '0') && (size == 2U)) {
            HC04_RecordCommandAck(cmd, size, 1U, "telemetry-off");
            Freertos_SetTelemetryStreamingEnabled(0U);
            Freertos_SetTelemetryScanStreamingEnabled(0U);
            transmit("Structured telemetry disabled\r\n");
        } else if ((cmd[1] == '1') && (size == 2U)) {
            Freertos_SetTelemetryStreamingEnabled(1U);
            transmit("Structured telemetry enabled\r\n");
            HC04_RecordCommandAck(cmd, size, 1U, "telemetry-on");
        } else {
            transmit("Invalid Y format. Use Y0 or Y1\r\n");
            HC04_RecordCommandAck(cmd, size, 0U, "invalid-y-format");
        }
    } else if ((size >= 2U) && (cmd[0] == 'Q')) {
        if ((cmd[1] == '0') && (size == 2U)) {
            HC04_RecordCommandAck(cmd, size, 1U, "scan-telemetry-off");
            Freertos_SetTelemetryScanStreamingEnabled(0U);
            transmit("Structured scan telemetry disabled\r\n");
        } else if ((cmd[1] == '1') && (size == 2U)) {
            Freertos_SetTelemetryStreamingEnabled(1U);
            Freertos_SetTelemetryScanStreamingEnabled(1U);
            transmit("Structured scan telemetry enabled\r\n");
            HC04_RecordCommandAck(cmd, size, 1U, "scan-telemetry-on");
        } else {
            transmit("Invalid Q format. Use Q0 or Q1\r\n");
            HC04_RecordCommandAck(cmd, size, 0U, "invalid-q-format");
        }
    } else if ((size >= 2U) && (cmd[0] == 'B')) {
        unsigned long baud_rate = 0UL;

        if (sscanf((char *)cmd, "B%lu", &baud_rate) == 1) {
            if ((baud_rate == 9600UL) || (baud_rate == 115200UL) || (baud_rate == 921600UL)) {
                uart_printf("Switching UART5 baud to %lu\r\n", baud_rate);
                HC04_RecordCommandAck(cmd, size, 1U, "baud-switch");
                (void)UART5_ReconfigureBaudRate((uint32_t)baud_rate);
                g_bt_rx_rearm_pending = 1U;
            } else {
                transmit("Invalid baud. Use B9600, B115200 or B921600\r\n");
                HC04_RecordCommandAck(cmd, size, 0U, "invalid-baud-value");
            }
        } else {
            transmit("Invalid baud format. Use B9600, B115200 or B921600\r\n");
            HC04_RecordCommandAck(cmd, size, 0U, "invalid-baud-format");
        }
    } else if ((size >= 2U) && (cmd[0] == 'U')) {
        if ((cmd[1] == '0') && (size == 2U)) {
            status_enable = 0U;
            transmit("Live map stream disabled\r\n");
            HC04_RecordCommandAck(cmd, size, 1U, "live-map-off");
        } else if ((cmd[1] == '1') && (size == 2U)) {
            status_enable = 1U;
            transmit("Live thumbnail map stream enabled (500 ms)\r\n");
            HC04_RecordCommandAck(cmd, size, 1U, "live-map-on");
        } else {
            transmit("Invalid U format. Use U0 or U1\r\n");
            HC04_RecordCommandAck(cmd, size, 0U, "invalid-u-format");
        }
    } else {
        transmit("Unknown complex command. Send 'H' for help\r\n");
        HC04_RecordCommandAck(cmd, size, 0U, "unknown-complex-command");
    }
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

void HC04_ServiceStatusStream(void)
{
    static uint32_t s_last_status_tick = 0U;
    uint32_t now_tick;

    if (status_enable == 0U) {
        s_last_status_tick = HAL_GetTick();
        return;
    }

    now_tick = HAL_GetTick();
    if ((now_tick - s_last_status_tick) < STATUS_STREAM_PERIOD_MS) {
        return;
    }

    s_last_status_tick = now_tick;
    transmit_live_map_snapshot();
}
