//
// Created by G on 25-7-1.
//

#ifndef HC04_H
#define HC04_H

#include <stdint.h>
#include "encoder.h"  // 修正: 只包含头文件, 不包含 .c

// Buffer for Bluetooth data reception
extern uint8_t buffer[100];

// Initialize HC-04/HC-05 Bluetooth module
void hc04_init(void);
extern uint8_t status_enable;
// Transmit message over UART
void transmit(char* message);

void transmit_uint8(uint8_t * message,uint8_t size);

// Process received command
void process_command(uint8_t *cmd, uint16_t size);

// 检查是否需要自动停车
void check_auto_stop(void);

void process_complex_command(uint8_t *cmd, uint16_t size);

void uart_printf(const char *format, ...);
void HC04_ServiceStatusStream(void);
#endif // HC04_H
