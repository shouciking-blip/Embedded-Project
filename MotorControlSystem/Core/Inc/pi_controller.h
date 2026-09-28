#ifndef __PI_CONTROLLER_H__
#define __PI_CONTROLLER_H__

#include "main.h"


typedef struct
{
    float kp;
    float ki;

    float integral;

    float base_pwm;
    float pwm_max;

    float integral_max;
    float integral_min;

} PIController_t;


/* 初始化PI控制器 */
void PI_Controller_Init(
    PIController_t *pi,
    float kp,
    float ki,
    float base_pwm,
    float pwm_max,
    float integral_min,
    float integral_max
);


/* PI计算 */
float PI_Controller_Update(
    PIController_t *pi,
    float target,
    float actual,
    float dt
);


/* 清除积分 */
void PI_Controller_Reset(
    PIController_t *pi
);


#endif
