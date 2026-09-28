#ifndef __SSD1306_H__
#define __SSD1306_H__

#include "main.h"

/* SSD1306 I2C µÿ÷∑ */
#define SSD1306_I2C_ADDR    (0x3C << 1)

/* OLED ∑÷±Ê¬  */
#define SSD1306_WIDTH       128
#define SSD1306_HEIGHT      64

void SSD1306_Init(void);
void SSD1306_UpdateScreen(void);
void SSD1306_Clear(void);

void SSD1306_DrawPixel(uint8_t x, uint8_t y, uint8_t color);

void SSD1306_GotoXY(uint8_t x, uint8_t y);

void SSD1306_Puts(const char *str);

#endif
