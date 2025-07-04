//
// Created by G on 25-7-1.
//

#include <stdio.h>
#include "hc04.h"
#include "main.h"
#include "string.h"
#include "motor.h" // Include the motor control functions
#include "lidar.h"
extern UART_HandleTypeDef huart5;
//extern uint8_t speed;
// Car control command flags
#define CMD_FORWARD   'F'
#define CMD_BACKWARD  'B'
#define CMD_LEFT      'L'
#define CMD_RIGHT     'R'
#define CMD_STOP      'S'
#define CMD_SPEED     'V'
uint8_t speed = 65;
uint32_t period = 500;
uint8_t status_enable=0;
// 自动停车控制变量
uint32_t auto_stop_time = 0;      // 自动停车的时间点
uint8_t auto_stop_pending = 0;    // 是否有待执行的自动停车

void process_command(uint8_t *cmd, uint16_t size)
{
    transmit_uint8(cmd,size);

    // Process complex commands (commands with parameters)
    if(size > 1) {
        process_complex_command(cmd, size);
        return;
    }

    // Process single character commands
    if (size == 1) {
        switch (cmd[0]) {
            case 'F':
                Car_Forward(speed);
                transmit("Moving forward\r\n");
                // Set auto-stop
                auto_stop_time = HAL_GetTick() + period;
                auto_stop_pending = 1;
                break;

            case 'B':
                Car_Backward(speed);
                transmit("Moving backward\r\n");
                // Set auto-stop
                auto_stop_time = HAL_GetTick() + period;
                auto_stop_pending = 1;
                break;

            case 'L':
                Car_TurnLeft(65);
                transmit("Turning left\r\n");
                // Set auto-stop
                auto_stop_time = HAL_GetTick() + period/3;
                auto_stop_pending = 1;
                break;

            case 'R':
                Car_TurnRight(65 );
                transmit("Turning right\r\n");
                // Set auto-stop
                auto_stop_time = HAL_GetTick() + period/3;
                auto_stop_pending = 1;
                break;

            case 'S':
                Car_Stop();
                transmit("Stopped\r\n");
                // Cancel auto-stop
                auto_stop_pending = 0;
                break;
            case 'N':
                Lidar_StopScan();
                transmit("Lidar Stopped\r\n");
                break;
            case 'M':
                RPLIDAR_RequestScan();
                break;
            case 'P':
                status_enable=1;
                break;
            case 'Q':
                status_enable=0;
                break;
//            case 'D': // Check LIDAR connection
//                if (AX_LASER_CheckConnection()) {
//                    transmit("LIDAR Status: Connected\r\n");
//
//                    // Get health status
//                    LaserHealthTypeDef health;
//                    if(AX_LASER_GetHealth(&health)) {
//                        char health_msg[50];
//                        sprintf(health_msg, "Health: %d (0=Good,1=Warning,2=Error), Error Code: %d\r\n",
//                                health.status, health.error_code);
//                        transmit(health_msg);
//                    }
//
//                    // Send some LIDAR data samples
//                    char data_msg[100];
//                    uint8_t samples_sent = 0;
//                    transmit("LIDAR Data Samples:\r\n");
//
//                    for (int i = 0; i < 250 && samples_sent < 5; i++) {
//                        if (ax_ls_point[i].distance > 0) {
//                            sprintf(data_msg, "Angle: %d deg, Distance: %dmm, Quality: %d\r\n",
//                                    ax_ls_point[i].angle/100, ax_ls_point[i].distance, ax_ls_point[i].quality);
//                            transmit(data_msg);
//                            samples_sent++;
//                        }
//                    }
//
//                    if (samples_sent == 0) {
//                        transmit("No valid LIDAR data found\r\n");
//                    }
//                } else {
//                    transmit("LIDAR Status: Disconnected\r\n");
//                    transmit("Attempting to reinitialize LIDAR...\r\n");
//
//                    // Try to reinitialize
//                    AX_LASER_Init();
//                    AX_LASER_Start();
//                }
//                break;

//            case 'M': // Start LIDAR
//                transmit("Starting LIDAR...\r\n");
//                AX_LASER_Start();
//                break;

//            case 'N': // Stop LIDAR
//                transmit("Stopping LIDAR...\r\n");
//                AX_LASER_Stop();
//                break;

            case 'H': // Show help
                transmit("\r\n--- Bluetooth Control Commands ---\r\n");
                transmit("F: Move Forward\r\n");
                transmit("B: Move Backward\r\n");
                transmit("L: Turn Left\r\n");
                transmit("R: Turn Right\r\n");
                transmit("S: Stop\r\n");
                transmit("V+number: Set Speed (1-100)\r\n");
                transmit("T+number: Set Auto-stop Time (ms)\r\n");
                transmit("M: Start LIDAR\r\n");
                transmit("N: Stop LIDAR\r\n");
                //transmit("P+number: Get LIDAR Data in Front Angle\r\n");
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

// 检查是否需要自动停车
void check_auto_stop(void)
{
    if (auto_stop_pending && HAL_GetTick() >= auto_stop_time) {
        Car_Stop();
        auto_stop_pending = 0;
        transmit("Auto stopped\r\n");
    }
}

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t size)
{
    if(huart->Instance==UART5)
    {
        transmit("Have Received:");
        HAL_UART_Transmit(&huart5,buffer,size,1000);
        transmit("\n");

        process_command(buffer, size);


        HAL_UARTEx_ReceiveToIdle_DMA(&huart5, buffer, sizeof(buffer)); 
        __HAL_DMA_DISABLE_IT(huart->hdmarx, DMA_IT_HT); // Disable half-transfer interrupt
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

    // Process possible command format: V80 (set speed to 80%)
    if(size >= 2 && cmd[0] == 'V') {
        // Extract the numeric part
        uint8_t new_speed = 0;
        for(int i = 1; i < size; i++) {
            if(cmd[i] >= '0' && cmd[i] <= '9') {
                new_speed = new_speed * 10 + (cmd[i] - '0');
            }
        }

        // Update speed
        if(new_speed > 0 && new_speed <= 100) {
            speed = new_speed;
            sprintf(reply, "Speed set to %d%%\r\n", speed);
            transmit(reply);
        } else {
            transmit("Invalid speed value. Use 1-100\r\n");
        }
    }
        // Process delay setting command: T1000 (set delay to 1000ms)
    else if(size >= 2 && cmd[0] == 'T') {
        // Extract the numeric part
        uint32_t new_period = 0;
        for(int i = 1; i < size; i++) {
            if(cmd[i] >= '0' && cmd[i] <= '9') {
                new_period = new_period * 10 + (cmd[i] - '0');
            }
        }

        // Update delay
        if(new_period > 0 && new_period <= 10000) {
            period = new_period;
            sprintf(reply, "Auto-stop period set to %ldms\r\n", period);
            transmit(reply);
        } else {
            transmit("Invalid period value. Use 1-10000\r\n");
        }
    }
//        // Process get LIDAR scan data command: P30 (get LIDAR data within 30 degrees in front)
//    else if(size >= 2 && cmd[0] == 'P') {
//        // Extract the numeric part (angle range)
//        uint16_t angle_range = 30; // Default 30 degrees
//        for(int i = 1; i < size; i++) {
//            if(cmd[i] >= '0' && cmd[i] <= '9') {
//                angle_range = angle_range * 10 + (cmd[i] - '0');
//            }
//        }
//
//        // Limit range to 1-180 degrees
//        if(angle_range < 1) angle_range = 1;
//        if(angle_range > 180) angle_range = 180;
//
//        // Send scan data
//        transmit("LIDAR Scan Data:\r\n");
//
//        uint8_t data_count = 0;
//        for(int i = 0; i < 250 && data_count < 10; i++) {
//            // Only get points within the specified angle range in front
//            uint16_t angle_deg = ax_ls_point[i].angle / 100;
//            if((angle_deg < angle_range || angle_deg > (360-angle_range)) &&
//               ax_ls_point[i].distance > 0) {
//                sprintf(reply, "Angle:%d deg Distance:%dmm Quality:%d\r\n",
//                        angle_deg, ax_ls_point[i].distance, ax_ls_point[i].quality);
//                transmit(reply);
//                data_count++;
//            }
//        }
//
//        if(data_count == 0) {
//            transmit("No valid data detected\r\n");
//        }
//    }
}