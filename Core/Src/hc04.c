#include <stdarg.h>
#include <stdio.h>

#include "freertos_app.h"
#include "localization_task.h"
#include "mapping_task.h"
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

uint8_t buffer[100];
uint8_t status_enable = 0U;

volatile float speed_magnitude = 1.0f;
static volatile uint8_t g_bt_rx_rearm_pending = 0U;
static char g_uart_printf_buffer[UART_PRINTF_BUFFER_SIZE];
static char g_map_ascii_line_buffer[OGM_MAX_WIDTH_CELLS + 1U];

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

static void transmit_odometry_snapshot(void)
{
    float x;
    float y;
    float th;
    float left_speed;
    float right_speed;
    float current_base_speed;
    float angle_setpoint;
    ControlMode mode;
    RelativeMoveState move_state;

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
}

static void transmit_runtime_snapshot(void)
{
    FreertosRuntimeStats_t stats;
    LocalizationTaskStats_t loc_stats;
    uint8_t lidar_binary_tx_enabled;
    uint8_t lidar_active;

    Freertos_GetRuntimeStatsSnapshot(&stats);
    LocalizationTask_GetStatsSnapshot(&loc_stats);
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
    uart_printf("LOC init=%u updates=%lu accept=%lu reject=%lu odom=%lu mode=%u pts=%u/%u inliers=%u fit_mm=%.1f\r\n",
                loc_stats.initialized,
                (unsigned long)loc_stats.update_count,
                (unsigned long)loc_stats.icp_accept_count,
                (unsigned long)loc_stats.icp_reject_count,
                (unsigned long)loc_stats.odom_only_count,
                (unsigned int)loc_stats.last_mode,
                (unsigned int)loc_stats.last_reference_points,
                (unsigned int)loc_stats.last_current_points,
                (unsigned int)loc_stats.last_inliers,
                (double)(loc_stats.last_fitness_m * 1000.0f));
}

static void transmit_mapping_snapshot(void)
{
    MappingTaskStats_t stats;

    MappingTask_GetStatsSnapshot(&stats);
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
}

static void transmit_mapping_ascii(uint8_t downsample)
{
    uint16_t width = 0U;
    uint16_t height = 0U;
    uint16_t row;

    if (lidar_raw_stream_active != 0U) {
        transmit("Map dump unavailable while LIDAR is active. Send N first, then X.\r\n");
        return;
    }

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
            Control_SetManualCommand(speed_magnitude, 0.0f);
            transmit("Moving forward (PID)\r\n");
            break;

        case CMD_BACKWARD:
            Control_SetManualCommand(-speed_magnitude, 0.0f);
            transmit("Moving backward (PID)\r\n");
            break;

        case CMD_LEFT:
            Control_SetManualCommand(0.0f, -PID_DEFAULT_TURN_SETPOINT);
            transmit("Turning left (PID)\r\n");
            break;

        case CMD_RIGHT:
            Control_SetManualCommand(0.0f, PID_DEFAULT_TURN_SETPOINT);
            transmit("Turning right (PID)\r\n");
            break;

        case CMD_STOP:
            Control_StopCommand();
            transmit("Stopped (PID)\r\n");
            break;

        case 'N':
            transmit("Stopping lidar...\r\n");
            Freertos_SetLidarBinaryTxEnabled(0U);
            RPLIDAR_StopRaw();
            transmit("Lidar Stopped\r\n");
            break;

        case 'M':
            Freertos_SetLidarBinaryTxEnabled(0U);
            RPLIDAR_StartRaw();
            transmit("Lidar Started (mapping mode)\r\n");
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
            transmit("O: Show odometry snapshot\r\n");
            transmit("P: Show RTOS/runtime stats and scan quality\r\n");
            transmit("M: Start LIDAR (mapping mode, no binary stream)\r\n");
            transmit("N: Stop LIDAR\r\n");
            transmit("T0/T1: Disable/enable binary LiDAR telemetry on Bluetooth\r\n");
            transmit("H: Show This Help\r\n");
            break;

        case 'O':
            transmit_odometry_snapshot();
            break;

        case 'P':
            transmit_runtime_snapshot();
            break;

        case 'G':
            transmit_mapping_snapshot();
            break;

        case 'X':
            transmit_mapping_ascii(4U);
            break;

        case 'Z':
            MappingTask_ResetGrid();
            transmit("Occupancy grid reset\r\n");
            break;

        default:
            transmit("Unknown command. Send 'H' for help\r\n");
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
                return;
            }
        }

        speed_magnitude = int_part + (frac_part / frac_divisor);

        if ((speed_magnitude >= 0.0f) && (speed_magnitude <= 10.0f)) {
            Control_SetBaseSpeed(speed_magnitude);
            (void)snprintf(reply, sizeof(reply), "base_car_speed is %.2f\r\n", speed_magnitude);
            transmit(reply);
        } else {
            transmit("Invalid speed value. Use 0.00-10.00\r\n");
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
                return;
            }
        }

        float delta_angle = (int_part + (frac_part / frac_divisor)) * sign;
        if ((delta_angle >= -360.0f) && (delta_angle <= 360.0f)) {
            Control_SetRelativeTurn(delta_angle);
            (void)snprintf(reply, sizeof(reply), "Relative turn %.2f degrees\r\n", delta_angle);
            transmit(reply);
        } else {
            transmit("Invalid angle value. Use -360.0 to 360.0\r\n");
        }
    } else if ((size > 2U) && (cmd[0] == 'P')) {
        float dx = 0.0f;
        float dy = 0.0f;

        if (sscanf((char *)cmd, "P%f,%f", &dx, &dy) == 2) {
            Start_Relative_Move(dx, dy);
            uart_printf("Relative move started to %f,%f\r\n", dx, dy);
        } else {
            transmit("Invalid format. Use P{dx},{dy}\r\n");
        }
    } else if ((size >= 2U) && (cmd[0] == 'X')) {
        int downsample = 0;

        if (sscanf((char *)cmd, "X%d", &downsample) == 1) {
            if ((downsample >= 1) && (downsample <= 8)) {
                transmit_mapping_ascii((uint8_t)downsample);
            } else {
                transmit("Invalid X value. Use X1..X8\r\n");
            }
        } else {
            transmit("Invalid map format. Use X or X2\r\n");
        }
    } else if ((size >= 2U) && (cmd[0] == 'T')) {
        if ((cmd[1] == '0') && (size == 2U)) {
            Freertos_SetLidarBinaryTxEnabled(0U);
            transmit("Lidar binary telemetry disabled\r\n");
        } else if ((cmd[1] == '1') && (size == 2U)) {
            Freertos_SetLidarBinaryTxEnabled(1U);
            transmit("Lidar binary telemetry enabled\r\n");
        } else {
            transmit("Invalid T format. Use T0 or T1\r\n");
        }
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
