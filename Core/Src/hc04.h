//
// Created by G on 25-7-1.
//

#ifndef FINAL_FINA_HC04_H
#define FINAL_FINA_HC04_H

#include "stm32f4xx_hal.h"

extern uint8_t buffer[100]; // Declare buffer as extern to avoid redefinition
void hc04_init(void);
void HAL_UARTEx_RxEventCallback(UART_HandleTypeDef* huart, uint16_t size); // Add function prototype

#endif //FINAL_FINA_HC04_H
