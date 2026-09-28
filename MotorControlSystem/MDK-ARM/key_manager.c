#include "key_manager.h"

#define KEY_DEBOUNCE_TIME_MS    20

static uint8_t key_start_state = 1;
static uint8_t key_stop_state  = 1;
static uint8_t key_mode_state  = 1;
static uint8_t key_clear_state = 1;

static uint32_t key_start_tick = 0;
static uint32_t key_stop_tick  = 0;
static uint32_t key_mode_tick  = 0;
static uint32_t key_clear_tick = 0;


void KeyManager_Init(void)
{
    key_start_state = 1;
    key_stop_state  = 1;
    key_mode_state  = 1;
    key_clear_state = 1;

    key_start_tick = HAL_GetTick();
    key_stop_tick  = HAL_GetTick();
    key_mode_tick  = HAL_GetTick();
    key_clear_tick = HAL_GetTick();
}


KeyEvent_t KeyManager_Scan(void)
{
    GPIO_PinState start_now;
    GPIO_PinState stop_now;
    GPIO_PinState mode_now;
    GPIO_PinState clear_now;

    uint32_t now = HAL_GetTick();


    start_now =
        HAL_GPIO_ReadPin(
            KEY_START_GPIO_Port,
            KEY_START_Pin
        );

    stop_now =
        HAL_GPIO_ReadPin(
            KEY_STOP_GPIO_Port,
            KEY_STOP_Pin
        );

    mode_now =
        HAL_GPIO_ReadPin(
            KEY_MODE_GPIO_Port,
            KEY_MODE_Pin
        );

    clear_now =
        HAL_GPIO_ReadPin(
            KEY_CLEAR_GPIO_Port,
            KEY_CLEAR_Pin
        );


    /*
     * START
     */
    if ((key_start_state == 1) &&
        (start_now == GPIO_PIN_RESET))
    {
        if ((now - key_start_tick) >=
            KEY_DEBOUNCE_TIME_MS)
        {
            key_start_state = 0;

            return KEY_EVENT_START;
        }
    }

    if (start_now == GPIO_PIN_SET)
    {
        key_start_state = 1;
        key_start_tick = now;
    }


    /*
     * STOP
     */
    if ((key_stop_state == 1) &&
        (stop_now == GPIO_PIN_RESET))
    {
        if ((now - key_stop_tick) >=
            KEY_DEBOUNCE_TIME_MS)
        {
            key_stop_state = 0;

            return KEY_EVENT_STOP;
        }
    }

    if (stop_now == GPIO_PIN_SET)
    {
        key_stop_state = 1;
        key_stop_tick = now;
    }


    /*
     * MODE
     */
    if ((key_mode_state == 1) &&
        (mode_now == GPIO_PIN_RESET))
    {
        if ((now - key_mode_tick) >=
            KEY_DEBOUNCE_TIME_MS)
        {
            key_mode_state = 0;

            return KEY_EVENT_MODE;
        }
    }

    if (mode_now == GPIO_PIN_SET)
    {
        key_mode_state = 1;
        key_mode_tick = now;
    }


    /*
     * CLEAR
     */
    if ((key_clear_state == 1) &&
        (clear_now == GPIO_PIN_RESET))
    {
        if ((now - key_clear_tick) >=
            KEY_DEBOUNCE_TIME_MS)
        {
            key_clear_state = 0;

            return KEY_EVENT_CLEAR;
        }
    }

    if (clear_now == GPIO_PIN_SET)
    {
        key_clear_state = 1;
        key_clear_tick = now;
    }


    return KEY_EVENT_NONE;
}
