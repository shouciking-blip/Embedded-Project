#ifndef __DWT_TIMER_H
#define __DWT_TIMER_H

#include "stm32f4xx.h"

void DWT_Timer_Init(void);
uint32_t DWT_Timer_GetCounter(void);

#endif
