#ifndef __BUZZER_MANAGER_H
#define __BUZZER_MANAGER_H

#include "main.h"

void BuzzerManager_Init(void);

void BuzzerManager_On(void);
void BuzzerManager_Off(void);

void BuzzerManager_Beep(uint32_t duration_ms);

void BuzzerManager_StartBeep(void);
void BuzzerManager_StopBeep(void);
void BuzzerManager_ClearBeep(void);
void BuzzerManager_ModeBeep(void);
void BuzzerManager_FaultBeep(void);

#endif
