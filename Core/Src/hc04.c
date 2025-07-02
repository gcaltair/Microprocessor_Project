//
// Created by G on 25-7-1.
//

#include <stdio.h>
#include "hc04.h"
#include "main.h"
#include "string.h"
#include "motor.h" // Include the motor control functions

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

// 自动停车控制变量
uint32_t auto_stop_time = 0;      // 自动停车的时间点
uint8_t auto_stop_pending = 0;    // 是否有待执行的自动停车

// Process received command
void process_command(uint8_t *cmd, uint16_t size)
{
    transmit_uint8(cmd,size);
    if (size == 1) {
        switch (cmd[0]) {
            case 'F':
                Car_Forward(speed);
                transmit("Moving forward\r\n");
                // 设置1秒后自动停车
                auto_stop_time = HAL_GetTick() + period;
                auto_stop_pending = 1;
                break;
                
            case 'B':
                Car_Backward(speed);
                transmit("Moving backward\r\n");
                // 设置1秒后自动停车
                auto_stop_time = HAL_GetTick() + period;
                auto_stop_pending = 1;
                break;
                
            case 'L':
                Car_TurnLeft(speed);
                transmit("Turning left\r\n");
                // 设置1秒后自动停车
                auto_stop_time = HAL_GetTick() + period;
                auto_stop_pending = 1;
                break;
                
            case 'R':
                Car_TurnRight(speed);
                transmit("Turning right\r\n");
                // 设置1秒后自动停车
                auto_stop_time = HAL_GetTick() + period;
                auto_stop_pending = 1;
                break;
                
            case 'S':
                Car_Stop();
                transmit("Stopped\r\n");
                // 取消自动停车
                auto_stop_pending = 0;
                break;
                
            default:
                transmit("Unknown command\r\n");
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

