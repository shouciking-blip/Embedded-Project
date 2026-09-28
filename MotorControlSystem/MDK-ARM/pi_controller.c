#include "pi_controller.h"


/* ==========================================
 * PI控制器初始化
 * ========================================== */
void PI_Controller_Init(
    PIController_t *pi,
    float kp,
    float ki,
    float base_pwm,
    float pwm_max,
    float integral_min,
    float integral_max
)
{
    pi->kp = kp;
    pi->ki = ki;

    pi->integral = 0.0f;

    pi->base_pwm = base_pwm;
    pi->pwm_max = pwm_max;

    pi->integral_min = integral_min;
    pi->integral_max = integral_max;
}


/* ==========================================
 * PI计算
 * ========================================== */
float PI_Controller_Update(
    PIController_t *pi,
    float target,
    float actual,
    float dt
)
{
    float error;
    float output;


    /* 计算误差 */
    error = target - actual;


    /* 积分 */
    pi->integral += error * dt;


    /* 积分限幅 */
    if (pi->integral > pi->integral_max)
    {
        pi->integral = pi->integral_max;
    }

    if (pi->integral < pi->integral_min)
    {
        pi->integral = pi->integral_min;
    }


    /* PI输出 */
    output =
        pi->base_pwm +
        pi->kp * error +
        pi->ki * pi->integral;


    /* 输出限幅 */
    if (output > pi->pwm_max)
    {
        output = pi->pwm_max;
    }

    if (output < 0.0f)
    {
        output = 0.0f;
    }


    return output;
}


/* ==========================================
 * 清除积分
 * ========================================== */
void PI_Controller_Reset(
    PIController_t *pi
)
{
    pi->integral = 0.0f;
}
