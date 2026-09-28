#ifndef __MOTOR_STATUS_H__
#define __MOTOR_STATUS_H__

#include "main.h"

/* 电机状态 */
typedef struct
{
    /* 转速 */
    float actual_rpm;
    float target_rpm;

    /* 运行状态 */
    uint8_t motor_running;

    /* PWM */
    uint16_t pwm;

    /* 传感器 */
    float current;
    float voltage;
    float temperature;

    /* 故障 */
    uint8_t fault;

} MotorStatus_t;


/* 全局电机状态 */
extern MotorStatus_t motor_status;


#endif
