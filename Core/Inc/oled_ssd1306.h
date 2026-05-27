#ifndef OLED_SSD1306_H
#define OLED_SSD1306_H

#include <stdint.h>

#include "main.h"

#define OLED_WIDTH   128U
#define OLED_HEIGHT  64U

uint8_t OLED_Init(void);
uint8_t OLED_IsReady(void);
void OLED_Clear(void);
void OLED_DrawString(uint8_t x, uint8_t y, const char *text);
void OLED_Update(void);

#endif /* OLED_SSD1306_H */
