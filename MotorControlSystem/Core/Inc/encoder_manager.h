#ifndef __ENCODER_MANAGER_H__
#define __ENCODER_MANAGER_H__

#include "main.h"

void EncoderManager_Init(void);
void EncoderManager_Update(void);

float EncoderManager_GetRPM(void);
float EncoderManager_GetRawRPM(void);
int16_t EncoderManager_GetDeltaCount(void);
uint16_t EncoderManager_GetCount(void);

#endif
