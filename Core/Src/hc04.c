//
// Created by G on 25-7-1.
//

#include "hc04.h"
#include "main.h"

extern UART_HandleTypeDef huart5;

void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t size)
{
    if(huart->Instance==UART5)
    {
        HAL_UART_Transmit_DMA(&huart5,buffer,size);
        HAL_UARTEx_ReceiveToIdle_DMA(&huart5,buffer,sizeof(buffer)); //一次能接收的最大
        __HAL_DMA_DISABLE_IT(huart->hdmarx,DMA_IT_HT); //关闭传输过半中断
    }
}

void hc04_init(void)
{
    HAL_UARTEx_ReceiveToIdle_DMA(&huart5,buffer,sizeof(buffer)); //一次能接收的最大
    HAL_Delay(3000); // 等待系统稳定
    HAL_UART_Transmit_DMA(&huart5, (uint8_t*)"UART Ready\r\n", 12);
}