#ifndef __FAULT_MANAGER_H__
#define __FAULT_MANAGER_H__

#include "main.h"

typedef enum
{
    FAULT_NONE = 0,
    FAULT_SPEED,
    FAULT_TEMP,
    FAULT_OVERCURRENT,
    FAULT_UNDERVOLTAGE,
    FAULT_OVERVOLTAGE,
    FAULT_STALL,
    FAULT_LOCKED

} FaultState_t;


/* 初始化 */
void FaultManager_Init(void);


/* 周期性故障检测 */
void FaultManager_Update(void);


/* 清除锁定故障 */
uint8_t FaultManager_Clear(void);


/* 获取当前故障 */
FaultState_t FaultManager_GetState(void);


/* 故障转字符串 */
const char* FaultManager_GetStateString(FaultState_t state);

#endif
