#include <stdarg.h>
#include <stdio.h>
#include <string.h>

#include "cmsis_os.h"

#include "../Inc/usart.h"
#include "hc04.h"

#define UART_PRINTF_BUFFER_SIZE  256U
#define UART5_TX_TIMEOUT_MS      1000U

static char g_uart_printf_buffer[UART_PRINTF_BUFFER_SIZE];

static void transmit_buffer(const uint8_t *message, uint16_t size)
{
    if ((message == NULL) || (size == 0U)) {
        return;
    }

    (void)HAL_UART_Transmit(&huart5, (uint8_t *)message, size, UART5_TX_TIMEOUT_MS);
    if (osKernelGetState() == osKernelRunning) {
        osDelay(1U);
    }
}

void transmit(const char *message)
{
    if (message != NULL) {
        transmit_buffer((const uint8_t *)message, (uint16_t)strlen(message));
    }
}

void transmit_uint8(const uint8_t *message, uint16_t size)
{
    transmit_buffer(message, size);
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
