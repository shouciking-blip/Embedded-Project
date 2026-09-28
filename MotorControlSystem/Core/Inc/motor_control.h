#ifndef __MOTOR_CONTROL_H__
#define __MOTOR_CONTROL_H__

#include "main.h"


/* ================================
 * Motor Control API
 * ================================ */

/* 初始化电机控制硬件 */
void MotorControl_Init(void);

/* 设置PWM
 *
 * pwm = 0 ~ 4200
 *
 * 当前项目方向约定：
 * CH1 = 0
 * CH2 = PWM
 *
 * 即正转
 */
void MotorControl_SetPWM(uint16_t pwm);

/* 停止电机 */
void MotorControl_Stop(void);


#endif
