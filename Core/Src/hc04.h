#ifndef HC04_H
#define HC04_H

#include <stdint.h>

void transmit(const char *message);
void transmit_uint8(const uint8_t *message, uint16_t size);
void uart_printf(const char *format, ...);

#endif /* HC04_H */
