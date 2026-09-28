#include "buzzer_manager.h"

void BuzzerManager_Init(void)
{
    /* 低电平触发，所以 HIGH = 关闭 */
    HAL_GPIO_WritePin(BUZZER_GPIO_Port,
                      BUZZER_Pin,
                      GPIO_PIN_SET);
}

void BuzzerManager_On(void)
{
    HAL_GPIO_WritePin(BUZZER_GPIO_Port,
                      BUZZER_Pin,
                      GPIO_PIN_RESET);
}

void BuzzerManager_Off(void)
{
    HAL_GPIO_WritePin(BUZZER_GPIO_Port,
                      BUZZER_Pin,
                      GPIO_PIN_SET);
}

void BuzzerManager_Beep(uint32_t duration_ms)
{
    BuzzerManager_On();

    HAL_Delay(duration_ms);

    BuzzerManager_Off();
}

void BuzzerManager_StartBeep(void)
{
    BuzzerManager_Beep(100);
}

void BuzzerManager_StopBeep(void)
{
    BuzzerManager_Beep(200);
}

void BuzzerManager_ClearBeep(void)
{
    BuzzerManager_Beep(100);
}

void BuzzerManager_ModeBeep(void)
{
    BuzzerManager_Beep(50);
}

void BuzzerManager_FaultBeep(void)
{
    BuzzerManager_On();

    HAL_Delay(100);

    BuzzerManager_Off();

    HAL_Delay(100);

    BuzzerManager_On();

    HAL_Delay(100);

    BuzzerManager_Off();
}

