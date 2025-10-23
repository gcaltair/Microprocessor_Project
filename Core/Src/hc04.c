//
// Created by G on 25-7-1.
//

#include <stdio.h>
#include <stdarg.h>     // 用于 va_list, va_start, va_arg, va_end
#include "system.h"
extern UART_HandleTypeDef huart5;
//extern uint8_t speed;
// Car control command flags
#define CMD_FORWARD   'F'
#define CMD_BACKWARD  'B'
#define CMD_LEFT      'L'
#define CMD_RIGHT     'R'
#define CMD_STOP      'S'
#define CMD_SPEED     'V'
volatile float speed_magnitude = 1.0f; // 默认值 (例如 50.0)
#define PID_DEFAULT_TURN_SETPOINT 10.0f


void process_command(uint8_t *cmd, uint16_t size)
{
    // // 原始透传期间... (保持不变)
    // if(lidar_raw_stream_active) {
    //     if(size==1) {
    //         if(cmd[0]=='N') {
    //             RPLIDAR_StopRaw();
    //         } else if(cmd[0]=='M') {
    //             RPLIDAR_StopRaw();
    //             HAL_Delay(5);
    //             RPLIDAR_StartRaw();
    //         }
    //     }
    //     return;
    // }

    transmit_uint8(cmd,size);

    // 处理复杂命令 (V)
    if(size > 1) {
        process_complex_command(cmd, size);
        return;
    }

    // 处理单字符命令
    if (size == 1) {
        switch (cmd[0]) {
            case 'F': // 【PID 简化】
                base_car_speed = speed_magnitude;  // 设置PID速度 (正向)
                g_pid_angle.setpoint = 0.0f;         // 设置PID角度 (直线)

                transmit("Moving forward (PID)\r\n");
                g_control_mode = CONTROL_MODE_MANUAL;
                break;

            case 'B': // 【PID 简化】
                base_car_speed = -speed_magnitude; // 设置PID速度 (反向)
                g_pid_angle.setpoint = 0.0f;         // 设置PID角度 (直线)
                transmit("Moving backward (PID)\r\n");
                g_control_mode = CONTROL_MODE_MANUAL;
                break;

            case 'L': // 【PID 简化】
                base_car_speed = 0.0f;               // 设置PID速度 (原地)
                g_pid_angle.setpoint = -PID_DEFAULT_TURN_SETPOINT; // 设置PID角度 (左转)
                transmit("Turning left (PID)\r\n");
                g_control_mode = CONTROL_MODE_MANUAL;
                break;

            case 'R': // 【PID 简化】
                base_car_speed = 0.0f;               // 设置PID速度 (原地)
                g_pid_angle.setpoint = PID_DEFAULT_TURN_SETPOINT;  // 设置PID角度 (右转)
                transmit("Turning right (PID)\r\n");
                g_control_mode = CONTROL_MODE_MANUAL;
                break;

            case 'S': // 【PID 简化】
                base_car_speed = 0.0f;       // 设置PID速度 (停止)
                g_pid_angle.setpoint = g_th_continuous;
                transmit("Stopped (PID)\r\n");
                break;

            // --- 其他 case 保持不变 ---
            case 'N':
                RPLIDAR_StopRaw();
                transmit("Lidar Stopped\r\n");
                break;
            case 'M':
                RPLIDAR_StartRaw();
                break;

            case 'H': // 【更新帮助信息】
                transmit("\r\n--- Bluetooth PID Control ---\r\n");
                transmit("F/B/L/R/S: Manual Control\r\n");
                transmit("A+angle: Set relative angle turn\r\n");
                transmit("V+speed: Set speed for manual mode\r\n");
                transmit("P{x},{y}: Set absolute target point (e.g., P0.5,1.2)\r\n"); // <-- 新命令
                // 'T' 命令已移除
                transmit("M: Start LIDAR\r\n");
                transmit("N: Stop LIDAR\r\n");
                transmit("O: Output odometry (dl, dr, dθ)\r\n");
                transmit("P: Show status\r\n");
                transmit("Q: Disable status\r\n");
                transmit("H: Show This Help\r\n");
                break;

            default:
                transmit("Unknown command. Send 'H' for help\r\n");
                break;
        }
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t size)
{
    if(huart->Instance==UART5)
    {
        // 仅处理命令
        process_command(buffer, size);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart5, buffer, sizeof(buffer));
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT);
    }
}

void hc04_init(void)
{
    // Initialize the motor control
    Motor_Init();
    
    // Setup UART reception for Bluetooth
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5, buffer, sizeof(buffer)); 
    HAL_Delay(1000); // Wait for system to stabilize
    transmit("Bluetooth Car Control Ready\r\n");
}

void transmit(char* message)
{
    HAL_UART_Transmit(&huart5, (uint8_t*)message, strlen(message),1000);
}
void transmit_uint8(uint8_t * message,uint8_t size)
{
    HAL_UART_Transmit(&huart5, message,size,1000);
}

// Process complex commands
void process_complex_command(uint8_t *cmd, uint16_t size)
{
    char reply[100];
    if(size >= 2 && cmd[0] == 'V') {
        float int_part = 0.0f;
        float frac_part = 0.0f;
        float frac_divisor = 1.0f;
        int parsing_frac = 0;

        // 手动解析浮点数 (避免引入 atof)
        for(int i = 1; i < size; i++) {
            if(cmd[i] >= '0' && cmd[i] <= '9') {
                if (!parsing_frac) {
                    // 整数部分
                    int_part = int_part * 10.0f + (cmd[i] - '0');
                } else {
                    // 小数部分
                    frac_part = frac_part * 10.0f + (cmd[i] - '0');
                    frac_divisor *= 10.0f;
                }
            } else if (cmd[i] == '.' && !parsing_frac) {
                // 遇到小数点
                parsing_frac = 1;
            } else {
                // 遇到无效字符
                transmit("Invalid speed format. Use V+number (e.g., V50.75)\r\n");
                return;
            }
        }

        float new_speed_val = int_part + (frac_part / frac_divisor);

        // 更新速度大小
        if(new_speed_val >= 0 && new_speed_val <= 10.0) { // 允许 V0 停止
            speed_magnitude = new_speed_val; // 存储新的速度大小
            base_car_speed=speed_magnitude;

            sprintf(reply, "base_car_spped is%.2f\r\n", base_car_speed);
            transmit(reply);
        } else {
            transmit("Invalid speed value. Use 0.00-100.00\r\n");
        }
    }
    else if(size >= 2 && cmd[0] == 'A') {
        float sign = 1.0f;
        int start_index = 1;

        // 检查负号
        if (cmd[1] == '-') {
            sign = -1.0f;
            start_index = 2; // 从负号之后开始解析
        }

        float int_part = 0.0f;
        float frac_part = 0.0f;
        float frac_divisor = 1.0f;
        int parsing_frac = 0;

        for(int i = start_index; i < size; i++) {
            if(cmd[i] >= '0' && cmd[i] <= '9') {
                if (!parsing_frac) {
                    int_part = int_part * 10.0f + (cmd[i] - '0');
                } else {
                    frac_part = frac_part * 10.0f + (cmd[i] - '0');
                    frac_divisor *= 10.0f;
                }
            } else if (cmd[i] == '.' && !parsing_frac) {
                parsing_frac = 1;
            } else {
                transmit("Invalid angle format. Use A+number (e.g., A-90.5)\r\n");
                return;
            }
        }

        float new_angle_setpoint = (int_part + (frac_part / frac_divisor)) * sign;

        // 假设角度限制在 -360 到 360 度之间
        if (new_angle_setpoint >= -360.0f && new_angle_setpoint <= 360.0f) {
            // 设置PID角度目标
            g_pid_angle.setpoint += new_angle_setpoint;
            // 设置为原地转向
            base_car_speed = 0.0f;

            sprintf(reply, "Angle setpoint set to %.2f degrees\r\n", g_pid_angle.setpoint);
            transmit(reply);
        } else {
            transmit("Invalid angle value. Use -360.0 to 360.0\r\n");
        }
    }
    else if (size > 2 && cmd[0] == 'P') { // M for Move Relative
        float dx, dy;
        if (sscanf((char*)cmd, "P%f,%f", &dx, &dy) == 2) {
            Start_Relative_Move(dx, dy); // 启动相对移动任务
            g_control_mode = CONTROL_MODE_POSITION;
            transmit("Relative move started.\r\n");
        } else {
            transmit("Invalid format. Use M{dx},{dy}\r\n");
        }
    }
    else if(size >= 2 && cmd[0] == 'T') {
        transmit("Auto-stop 'T' command is disabled.\r\n");
    }
}
#define UART_PRINTF_BUFFER_SIZE 256

/**
 * @brief 自定义的串口 printf 函数
 * @param format 格式化字符串，用法同 printf
 * @param ...    可变参数
 * @retval None
 */
void uart_printf(const char *format, ...)
{
    char buffer[UART_PRINTF_BUFFER_SIZE];
    va_list args;
    int len;

    // 1. 从 format 参数之后开始解析可变参数
    va_start(args, format);

    // 2. 使用 vsnprintf 将格式化字符串和可变参数合成一个完整的字符串
    //    vsnprintf 是 printf 的一个安全版本，可以防止缓冲区溢出
    //    它会将结果存入 buffer，并返回最终字符串的长度
    len = vsnprintf(buffer, sizeof(buffer), format, args);

    // 3. 结束解析可变参数
    va_end(args);

    // 4. 检查长度是否有效
    if (len > 0)
    {
        // 5. 调用 HAL 库函数，通过 UART 将 buffer 中的数据发送出去
        //    我们只发送实际生成的长度 (len)
        HAL_UART_Transmit(&huart5, (uint8_t *)buffer, len, HAL_MAX_DELAY);
    }
}