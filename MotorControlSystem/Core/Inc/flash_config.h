#ifndef __FLASH_CONFIG_H__
#define __FLASH_CONFIG_H__

#include "main.h"

typedef struct
{
    float kp;
    float ki;
    float target_rpm;
    uint32_t crc;
} MotorConfig_t;

#define FLASH_CONFIG_ADDRESS    0x080E0000U

uint32_t Flash_CalculateCRC(MotorConfig_t *config);

void Flash_ConfigLoad(MotorConfig_t *config);

HAL_StatusTypeDef Flash_ConfigSave(MotorConfig_t *config);

#endif
