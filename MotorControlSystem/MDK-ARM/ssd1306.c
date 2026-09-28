#include "ssd1306.h"
#include "i2c.h"
#include <string.h>

/* =========================
 * SSD1306 参数
 * ========================= */

#define SSD1306_CONTROL_CMD      0x00
#define SSD1306_CONTROL_DATA     0x40

#define SSD1306_TIMEOUT          100

/* OLED 显存 */
static uint8_t SSD1306_Buffer[
    SSD1306_WIDTH * SSD1306_HEIGHT / 8
];

/* 当前光标 */
static uint8_t current_x = 0;
static uint8_t current_y = 0;


/* =========================
 * 基础 I2C 写函数
 * ========================= */

static void SSD1306_WriteCommand(uint8_t command)
{
    uint8_t data[2];

    data[0] = SSD1306_CONTROL_CMD;
    data[1] = command;

    HAL_I2C_Master_Transmit(
        &hi2c1,
        SSD1306_I2C_ADDR,
        data,
        2,
        SSD1306_TIMEOUT
    );
}


static void SSD1306_WriteData(uint8_t *data, uint16_t size)
{
    uint8_t buffer[17];

    buffer[0] = SSD1306_CONTROL_DATA;

    while (size > 0)
    {
        uint16_t chunk;

        chunk = (size > 16) ? 16 : size;

        memcpy(&buffer[1], data, chunk);

        HAL_I2C_Master_Transmit(
            &hi2c1,
            SSD1306_I2C_ADDR,
            buffer,
            chunk + 1,
            SSD1306_TIMEOUT
        );

        data += chunk;
        size -= chunk;
    }
}


/* =========================
 * OLED 初始化
 * ========================= */

void SSD1306_Init(void)
{
    HAL_Delay(100);

    SSD1306_WriteCommand(0xAE);

    SSD1306_WriteCommand(0x20);
    SSD1306_WriteCommand(0x02);

    SSD1306_WriteCommand(0xB0);

    SSD1306_WriteCommand(0xC8);

    SSD1306_WriteCommand(0x00);
    SSD1306_WriteCommand(0x10);

    SSD1306_WriteCommand(0x40);

    SSD1306_WriteCommand(0x81);
    SSD1306_WriteCommand(0x7F);

    SSD1306_WriteCommand(0xA1);

    SSD1306_WriteCommand(0xA6);

    SSD1306_WriteCommand(0xA8);
    SSD1306_WriteCommand(0x3F);

    SSD1306_WriteCommand(0xD3);
    SSD1306_WriteCommand(0x00);

    SSD1306_WriteCommand(0xD5);
    SSD1306_WriteCommand(0x80);

    SSD1306_WriteCommand(0xD9);
    SSD1306_WriteCommand(0xF1);

    SSD1306_WriteCommand(0xDA);
    SSD1306_WriteCommand(0x12);

    SSD1306_WriteCommand(0xDB);
    SSD1306_WriteCommand(0x40);

    SSD1306_WriteCommand(0x8D);
    SSD1306_WriteCommand(0x14);

    SSD1306_WriteCommand(0xAF);

    SSD1306_Clear();
    SSD1306_UpdateScreen();
}


/* =========================
 * 清屏
 * ========================= */

void SSD1306_Clear(void)
{
    memset(
        SSD1306_Buffer,
        0,
        sizeof(SSD1306_Buffer)
    );

    current_x = 0;
    current_y = 0;
}


/* =========================
 * 更新 OLED 显示
 * ========================= */

void SSD1306_UpdateScreen(void)
{
    uint8_t page;

    for (page = 0; page < 8; page++)
    {
        SSD1306_WriteCommand(0xB0 + page);

        SSD1306_WriteCommand(0x00);
        SSD1306_WriteCommand(0x10);

        SSD1306_WriteData(
            &SSD1306_Buffer[
                SSD1306_WIDTH * page
            ],
            SSD1306_WIDTH
        );
    }
}


/* =========================
 * 绘制像素
 * ========================= */

void SSD1306_DrawPixel(
    uint8_t x,
    uint8_t y,
    uint8_t color)
{
    if (x >= SSD1306_WIDTH ||
        y >= SSD1306_HEIGHT)
    {
        return;
    }

    if (color)
    {
        SSD1306_Buffer[
            x + (y / 8) * SSD1306_WIDTH
        ] |= (1 << (y % 8));
    }
    else
    {
        SSD1306_Buffer[
            x + (y / 8) * SSD1306_WIDTH
        ] &= ~(1 << (y % 8));
    }
}


/* =========================
 * 设置光标
 * ========================= */

void SSD1306_GotoXY(
    uint8_t x,
    uint8_t y)
{
    current_x = x;
    current_y = y;
}


/* =========================
 * 最小文字显示
 *
 * 暂时使用 5x7 ASCII 字体
 * ========================= */

static const uint8_t font5x7[][5] =
{
    /* 0-9 */
    {0x3E,0x51,0x49,0x45,0x3E}, /* 0 */
    {0x00,0x42,0x7F,0x40,0x00}, /* 1 */
    {0x42,0x61,0x51,0x49,0x46}, /* 2 */
    {0x21,0x41,0x45,0x4B,0x31}, /* 3 */
    {0x18,0x14,0x12,0x7F,0x10}, /* 4 */
    {0x27,0x45,0x45,0x45,0x39}, /* 5 */
    {0x3C,0x4A,0x49,0x49,0x30}, /* 6 */
    {0x01,0x71,0x09,0x05,0x03}, /* 7 */
    {0x36,0x49,0x49,0x49,0x36}, /* 8 */
    {0x06,0x49,0x49,0x29,0x1E}, /* 9 */

    /* A-Z */
    {0x7E,0x11,0x11,0x11,0x7E}, /* A */
    {0x7F,0x49,0x49,0x49,0x36}, /* B */
    {0x3E,0x41,0x41,0x41,0x22}, /* C */
    {0x7F,0x41,0x41,0x22,0x1C}, /* D */
    {0x7F,0x49,0x49,0x49,0x41}, /* E */
    {0x7F,0x09,0x09,0x09,0x01}, /* F */
    {0x3E,0x41,0x49,0x49,0x7A}, /* G */
    {0x7F,0x08,0x08,0x08,0x7F}, /* H */
    {0x00,0x41,0x7F,0x41,0x00}, /* I */
    {0x20,0x40,0x41,0x3F,0x01}, /* J */
    {0x7F,0x08,0x14,0x22,0x41}, /* K */
    {0x7F,0x40,0x40,0x40,0x40}, /* L */
    {0x7F,0x02,0x0C,0x02,0x7F}, /* M */
    {0x7F,0x04,0x08,0x10,0x7F}, /* N */
    {0x3E,0x41,0x41,0x41,0x3E}, /* O */
    {0x7F,0x09,0x09,0x09,0x06}, /* P */
    {0x3E,0x41,0x51,0x21,0x5E}, /* Q */
    {0x7F,0x09,0x19,0x29,0x46}, /* R */
    {0x46,0x49,0x49,0x49,0x31}, /* S */
    {0x01,0x01,0x7F,0x01,0x01}, /* T */
    {0x3F,0x40,0x40,0x40,0x3F}, /* U */
    {0x1F,0x20,0x40,0x20,0x1F}, /* V */
    {0x3F,0x40,0x38,0x40,0x3F}, /* W */
    {0x63,0x14,0x08,0x14,0x63}, /* X */
    {0x07,0x08,0x70,0x08,0x07}, /* Y */
    {0x61,0x51,0x49,0x45,0x43}  /* Z */
};


/* =========================
 * 显示字符串
 *
 * 当前仅支持：
 * 空格 + 数字+字母
 * ========================= */

void SSD1306_Puts(const char *str)
{
    while (*str)
    {
        char c = *str;
        uint8_t index;
        uint8_t i;
        uint8_t bit;

        /*
         * 数字
         */
        if (c >= '0' && c <= '9')
        {
            index = c - '0';

            for (i = 0; i < 5; i++)
            {
                uint8_t column = font5x7[index][i];

                for (bit = 0; bit < 7; bit++)
                {
                    if (column & (1 << bit))
                    {
                        SSD1306_DrawPixel(
                            current_x + i,
                            current_y + bit,
                            1
                        );
                    }
                }
            }

            current_x += 6;
        }

        /*
         * 大写字母
         */
        else if (c >= 'A' && c <= 'Z')
        {
            index = 10 + (c - 'A');

            for (i = 0; i < 5; i++)
            {
                uint8_t column = font5x7[index][i];

                for (bit = 0; bit < 7; bit++)
                {
                    if (column & (1 << bit))
                    {
                        SSD1306_DrawPixel(
                            current_x + i,
                            current_y + bit,
                            1
                        );
                    }
                }
            }

            current_x += 6;
        }

        /*
         * 空格
         */
        else if (c == ' ')
        {
            current_x += 6;
        }

        /*
         * 冒号
         */
        else if (c == ':')
        {
            SSD1306_DrawPixel(current_x + 1, current_y + 2, 1);
            SSD1306_DrawPixel(current_x + 1, current_y + 5, 1);

            current_x += 4;
        }

        /*
         * 小数点
         */
        else if (c == '.')
        {
            SSD1306_DrawPixel(current_x + 1, current_y + 6, 1);

            current_x += 3;
        }

        /*
         * 负号
         */
        else if (c == '-')
        {
            for (i = 0; i < 4; i++)
            {
                SSD1306_DrawPixel(
                    current_x + i,
                    current_y + 3,
                    1
                );
            }

            current_x += 5;
        }

        /*
         * 其他暂时忽略
         */
        else
        {
            current_x += 6;
        }

        str++;
    }
}
