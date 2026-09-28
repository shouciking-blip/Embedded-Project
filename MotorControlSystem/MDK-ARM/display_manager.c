#include "display_manager.h"

#include "ssd1306.h"
#include "motor_status.h"
#include "fault_manager.h"

#include <stdio.h>
#include <string.h>


/* =========================
 * DisplayManager 初始化
 * ========================= */

void DisplayManager_Init(void)
{
    SSD1306_Init();

    SSD1306_Clear();

    SSD1306_GotoXY(0, 0);
    SSD1306_Puts("MOTOR CONTROL");

    SSD1306_UpdateScreen();
}


/* =========================
 * OLED 状态显示
 * ========================= */

void DisplayManager_Update(void)
{
    char line[24];

    SSD1306_Clear();


    /* =========================
     * 标题
     * ========================= */

    SSD1306_GotoXY(0, 0);
    SSD1306_Puts("MOTOR CONTROL");


    /* =========================
     * 实际转速
     * ========================= */

    snprintf(
        line,
        sizeof(line),
        "RPM: %.1f",
        motor_status.actual_rpm
    );

    SSD1306_GotoXY(0, 8);
    SSD1306_Puts(line);


    /* =========================
     * 目标转速
     * ========================= */

    snprintf(
        line,
        sizeof(line),
        "TGT: %.1f",
        motor_status.target_rpm
    );

    SSD1306_GotoXY(0, 16);
    SSD1306_Puts(line);


    /* =========================
     * 电流
     * ========================= */

    snprintf(
        line,
        sizeof(line),
        "CUR: %.3f",
        motor_status.current
    );

    SSD1306_GotoXY(0, 24);
    SSD1306_Puts(line);


    /* =========================
     * 电压
     * ========================= */

    snprintf(
        line,
        sizeof(line),
        "V: %.2f",
        motor_status.voltage
    );

    SSD1306_GotoXY(0, 32);
    SSD1306_Puts(line);


    /* =========================
     * 温度
     * ========================= */

    snprintf(
        line,
        sizeof(line),
        "TEMP: %.1f",
        motor_status.temperature
    );

    SSD1306_GotoXY(0, 40);
    SSD1306_Puts(line);


    /* =========================
     * PWM
     * ========================= */

    snprintf(
        line,
        sizeof(line),
        "PWM: %d",
        motor_status.pwm
    );

    SSD1306_GotoXY(0, 48);
    SSD1306_Puts(line);


    /* =========================
     * 故障状态
     * ========================= */

    snprintf(
        line,
        sizeof(line),
        "FLT: %d",
        motor_status.fault
    );

    SSD1306_GotoXY(80, 48);
    SSD1306_Puts(line);


    SSD1306_UpdateScreen();
}
