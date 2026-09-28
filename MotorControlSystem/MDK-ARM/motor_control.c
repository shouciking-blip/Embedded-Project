#include "motor_control.h"
#include "tim.h"


/* PWM最大值 */
#define MOTOR_PWM_MAX    4200


/* ==========================================
 * 电机控制初始化
 * ========================================== */
void MotorControl_Init(void)
{
    /* BTS7960使能 */
    HAL_GPIO_WritePin(
        GPIOE,
        GPIO_PIN_4,
        GPIO_PIN_SET
    );

    HAL_GPIO_WritePin(
        GPIOE,
        GPIO_PIN_5,
        GPIO_PIN_SET
    );


    /* 启动TIM1 PWM */
    HAL_TIM_PWM_Start(
        &htim1,
        TIM_CHANNEL_1
    );

    HAL_TIM_PWM_Start(
        &htim1,
        TIM_CHANNEL_2
    );


    /* 初始停止 */
    MotorControl_Stop();
}


/* ==========================================
 * 设置PWM
 * ========================================== */
void MotorControl_SetPWM(uint16_t pwm)
{
    if (pwm > MOTOR_PWM_MAX)
    {
        pwm = MOTOR_PWM_MAX;
    }


    /*
     * 当前项目方向：
     *
     * CH1 = 0
     * CH2 = PWM
     *
     * 即正转
     */

    __HAL_TIM_SET_COMPARE(
        &htim1,
        TIM_CHANNEL_1,
        0
    );

    __HAL_TIM_SET_COMPARE(
        &htim1,
        TIM_CHANNEL_2,
        pwm
    );
}


/* ==========================================
 * 停止电机
 * ========================================== */
void MotorControl_Stop(void)
{
    __HAL_TIM_SET_COMPARE(
        &htim1,
        TIM_CHANNEL_1,
        0
    );

    __HAL_TIM_SET_COMPARE(
        &htim1,
        TIM_CHANNEL_2,
        0
    );
}
