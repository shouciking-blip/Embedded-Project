#include "dwt_timer.h"

void DWT_Timer_Init(void)
{
    /* Enable DWT and trace */
    CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk;

    /* Reset cycle counter */
    DWT->CYCCNT = 0;

    /* Enable cycle counter */
    DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;
}

uint32_t DWT_Timer_GetCounter(void)
{
    return DWT->CYCCNT;
}
