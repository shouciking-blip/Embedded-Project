#ifndef __SENSOR_MANAGER_H__
#define __SENSOR_MANAGER_H__

#include "main.h"

typedef enum
{
    TEMP_NORMAL = 0,
    TEMP_WARNING,
    TEMP_OVER

} TemperatureState_t;


/* 初始化 ADC + DMA */
void SensorManager_Init(void);


/* 读取 ADC 并完成传感器数据处理 */
void SensorManager_Update(void);


/* 获取当前温度状态 */
TemperatureState_t SensorManager_GetTemperatureState(void);


/* 获取当前 ADC 原始值 */
uint16_t SensorManager_GetADC0(void);
uint16_t SensorManager_GetADC1(void);
uint16_t SensorManager_GetADC2(void);


/* 获取当前计算后的传感器数据 */
float SensorManager_GetCurrentRaw(void);
float SensorManager_GetBusVoltage(void);
float SensorManager_GetTemperatureRaw(void);
float SensorManager_GetNTCResistance(void);


/* 获取当前 ADC 电压 */
float SensorManager_GetADC0Voltage(void);
float SensorManager_GetADC1Voltage(void);
float SensorManager_GetADC2Voltage(void);

#endif
