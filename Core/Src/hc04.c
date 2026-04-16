#include <stdarg.h>
#include <stdio.h>

#include "freertos_app.h"
#include "system.h"

#define CMD_FORWARD                 'F'
#define CMD_BACKWARD                'B'
#define CMD_LEFT                    'L'
#define CMD_RIGHT                   'R'
#define CMD_STOP                    'S'
#define CMD_SPEED                   'V'
#define PID_DEFAULT_TURN_SETPOINT   10.0f
#define UART_PRINTF_BUFFER_SIZE     256

uint8_t buffer[100];
uint8_t status_enable = 0U;

volatile float speed_magnitude = 1.0f;

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

    Freertos_GetRuntimeStatsSnapshot(&stats);
    uart_printf("RTOS ctrl=%lu overrun=%lu cmd=%lu drop=%lu scan=%lu tx=%lu busy=%lu err=%lu wait=%lu\r\n",
                (unsigned long)stats.control_cycles,
                (unsigned long)stats.control_tick_overruns,
                (unsigned long)stats.cmd_rx_count,
                (unsigned long)stats.cmd_drop_count,
                (unsigned long)stats.lidar_scan_complete_count,
                (unsigned long)stats.lidar_tx_count,
                (unsigned long)stats.lidar_tx_busy_count,
                (unsigned long)stats.lidar_tx_error_count,
                (unsigned long)stats.bt_tx_wait_count);
    uart_printf("HEAP free=%lu min=%lu lidar_overflow=%lu\r\n",
                (unsigned long)stats.free_heap_bytes,
                (unsigned long)stats.min_ever_free_heap_bytes,
                (unsigned long)stats.lidar_raw_overflow_count);
    uart_printf("STACK freeB dft=%lu ctrl=%lu lidar=%lu comm=%lu safe=%lu\r\n",
                (unsigned long)stats.default_task_stack_free_bytes,
                (unsigned long)stats.control_task_stack_free_bytes,
                (unsigned long)stats.lidar_task_stack_free_bytes,
                (unsigned long)stats.comm_task_stack_free_bytes,
                (unsigned long)stats.safety_task_stack_free_bytes);
}

void process_command(uint8_t *cmd, uint16_t size)
{
    if ((cmd == NULL) || (size == 0U)) {
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
            RPLIDAR_StopRaw();
            transmit("Lidar Stopped\r\n");
            break;

        case 'M':
            RPLIDAR_StartRaw();
            break;

        case 'H':
            transmit("\r\n--- Bluetooth PID Control ---\r\n");
            transmit("F/B/L/R/S: Manual Control\r\n");
            transmit("A+angle: Set relative angle turn\r\n");
            transmit("V+speed: Set speed for manual mode\r\n");
            transmit("P{x},{y}: Set relative target point (e.g., P0.5,1.2)\r\n");
            transmit("O: Show odometry snapshot\r\n");
            transmit("P: Show RTOS/runtime stats\r\n");
            transmit("M: Start LIDAR\r\n");
            transmit("N: Stop LIDAR\r\n");
            transmit("H: Show This Help\r\n");
            break;

        case 'O':
            transmit_odometry_snapshot();
            break;

        case 'P':
            transmit_runtime_snapshot();
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

        (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart5, buffer, sizeof(buffer));
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }
}

void hc04_init(void)
{
    (void)HAL_UARTEx_ReceiveToIdle_DMA(&huart5, buffer, sizeof(buffer));
    HAL_Delay(1000);
    transmit("Bluetooth Car Control Ready\r\n");
}

void transmit(char *message)
{
    if (message != NULL) {
        if (wait_for_uart5_tx_ready(100U) == HAL_OK) {
            (void)HAL_UART_Transmit(&huart5, (uint8_t *)message, (uint16_t)strlen(message), 1000);
        }
    }
}

void transmit_uint8(uint8_t *message, uint8_t size)
{
    if ((message != NULL) && (size > 0U)) {
        if (wait_for_uart5_tx_ready(100U) == HAL_OK) {
            (void)HAL_UART_Transmit(&huart5, message, size, 1000);
        }
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
    } else if ((size >= 2U) && (cmd[0] == 'T')) {
        transmit("Auto-stop 'T' command is disabled.\r\n");
    }
}

void uart_printf(const char *format, ...)
{
    char local_buffer[UART_PRINTF_BUFFER_SIZE];
    va_list args;
    int len;

    va_start(args, format);
    len = vsnprintf(local_buffer, sizeof(local_buffer), format, args);
    va_end(args);

    if (len > 0) {
        if (wait_for_uart5_tx_ready(100U) == HAL_OK) {
            (void)HAL_UART_Transmit(&huart5, (uint8_t *)local_buffer, (uint16_t)len, HAL_MAX_DELAY);
        }
    }
}
