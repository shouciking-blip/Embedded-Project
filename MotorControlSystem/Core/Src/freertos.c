/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "usart.h"
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */


/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

typedef enum
{
    FAULT_NONE = 0,
    FAULT_SPEED,
    FAULT_LOCKED
} FaultState_t;

/* USER CODE END PTD */


/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */


/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */


/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

extern UART_HandleTypeDef huart2;
extern TIM_HandleTypeDef htim3;
extern TIM_HandleTypeDef htim1;


/* =========================
   电机控制参数
   ========================= */

volatile float actual_rpm = 0.0f;
volatile float target_rpm = 80.0f;

float kp = 20.0f;
float ki = 0.5f;


/* =========================
   UART CLEAR相关
   ========================= */

volatile uint8_t uart_rx_byte = 0;
volatile uint8_t uart_clear_request = 0;


/* =========================
   故障状态
   ========================= */

volatile FaultState_t fault_state = FAULT_NONE;


/* 自动恢复次数 */
volatile uint8_t recovery_count = 0;


/*
 * 故障检测抑制计数
 *
 * FaultTask每100ms执行一次
 *
 * 20 × 100ms = 2秒
 */
volatile uint16_t fault_inhibit_count = 0;


/* 要求MotorTask重新启动 */
volatile uint8_t motor_restart_request = 0;


/* USER CODE END Variables */


/* Definitions for MotorTask */
osThreadId_t MotorTaskHandle;

const osThreadAttr_t MotorTask_attributes = {
  .name = "MotorTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};


/* Definitions for SensorTask */
osThreadId_t SensorTaskHandle;

const osThreadAttr_t SensorTask_attributes = {
  .name = "SensorTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};


/* Definitions for CommTask */
osThreadId_t CommTaskHandle;

const osThreadAttr_t CommTask_attributes = {
  .name = "CommTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};


/* Definitions for FaultTask */
osThreadId_t FaultTaskHandle;

const osThreadAttr_t FaultTask_attributes = {
  .name = "FaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};


/* Definitions for DisplayTask */
osThreadId_t DisplayTaskHandle;

const osThreadAttr_t DisplayTask_attributes = {
  .name = "DisplayTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};


/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */


void StartMotorTask(void *argument);
void StartSensorTas(void *argument);
void StartCommTask(void *argument);
void StartFaultTask(void *argument);
void StartDisplayTask(void *argument);


void MX_FREERTOS_Init(void);


/**
  * @brief  FreeRTOS initialization
  */
void MX_FREERTOS_Init(void)
{
    /* USER CODE BEGIN Init */

    /* USER CODE END Init */


    /* Create the thread(s) */

    MotorTaskHandle =
        osThreadNew(StartMotorTask,
                    NULL,
                    &MotorTask_attributes);


    SensorTaskHandle =
        osThreadNew(StartSensorTas,
                    NULL,
                    &SensorTask_attributes);


    CommTaskHandle =
        osThreadNew(StartCommTask,
                    NULL,
                    &CommTask_attributes);


    FaultTaskHandle =
        osThreadNew(StartFaultTask,
                    NULL,
                    &FaultTask_attributes);


    DisplayTaskHandle =
        osThreadNew(StartDisplayTask,
                    NULL,
                    &DisplayTask_attributes);
}


/* USER CODE BEGIN Header_StartMotorTask */
/**
  * @brief Function implementing the MotorTask thread.
  */
/* USER CODE END Header_StartMotorTask */

void StartMotorTask(void *argument)
{
	  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
    HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
    float integral = 0.0f;

    float error = 0.0f;
    float output = 0.0f;

    const float pwm_max = 4200.0f;
    const float control_dt = 0.1f;

    const float base_pwm = 2000.0f;

    /* USER CODE BEGIN StartMotorTask */

    for(;;)
    {

        /* =========================
           检查是否要求重新启动
           ========================= */

        if (motor_restart_request)
        {
            motor_restart_request = 0;

            integral = 0.0f;

            __HAL_TIM_SET_COMPARE(
                &htim1,
                TIM_CHANNEL_1,
                0);

            __HAL_TIM_SET_COMPARE(
                &htim1,
                TIM_CHANNEL_2,
                0);
        }


        /* ==================================================
           第一阶段：80 RPM
           ================================================== */

        target_rpm = 80.0f;

        for(int i = 0; i < 50; i++)
        {

            if (fault_state != FAULT_NONE)
            {
                __HAL_TIM_SET_COMPARE(
                    &htim1,
                    TIM_CHANNEL_1,
                    0);

                __HAL_TIM_SET_COMPARE(
                    &htim1,
                    TIM_CHANNEL_2,
                    0);

                osDelay(100);

                continue;
            }


            error = target_rpm - actual_rpm;


            integral += error * control_dt;


            /* 积分限幅 */
            if (integral > 3000.0f)
                integral = 3000.0f;

            if (integral < -3000.0f)
                integral = -3000.0f;


            output =
                base_pwm
                + kp * error
                + ki * integral;


            if (output > pwm_max)
                output = pwm_max;

            if (output < 0)
                output = 0;


            /*
             * 当前测试方向：
             *
             * CH1 PWM
             * CH2 = 0
             *
             * 编码器计数下降
             * RPM为负
             *
             * 如果你当前电机方向已经调整为正方向，
             * 后续可以再修改这里。
             */

						__HAL_TIM_SET_COMPARE(
								&htim1,
								TIM_CHANNEL_1,
								0);

						__HAL_TIM_SET_COMPARE(
								&htim1,
								TIM_CHANNEL_2,
								(uint32_t)output);

            osDelay(100);
        }


        /* ==================================================
           第二阶段：100 RPM
           ================================================== */

        target_rpm = 100.0f;

        for(int i = 0; i < 50; i++)
        {

            if (fault_state != FAULT_NONE)
            {
                __HAL_TIM_SET_COMPARE(
                    &htim1,
                    TIM_CHANNEL_1,
                    0);

                __HAL_TIM_SET_COMPARE(
                    &htim1,
                    TIM_CHANNEL_2,
                    0);

                osDelay(100);

                continue;
            }


            error = target_rpm - actual_rpm;


            integral += error * control_dt;


            if (integral > 3000.0f)
                integral = 3000.0f;

            if (integral < -3000.0f)
                integral = -3000.0f;


            output =
                base_pwm
                + kp * error
                + ki * integral;


            if (output > pwm_max)
                output = pwm_max;

            if (output < 0)
                output = 0;


						__HAL_TIM_SET_COMPARE(
								&htim1,
								TIM_CHANNEL_1,
								0);

						__HAL_TIM_SET_COMPARE(
								&htim1,
								TIM_CHANNEL_2,
								(uint32_t)output);


            osDelay(100);
        }


        /* ==================================================
           第三阶段：120 RPM
           ================================================== */

        target_rpm = 120.0f;

        for(int i = 0; i < 50; i++)
        {

            if (fault_state != FAULT_NONE)
            {
                __HAL_TIM_SET_COMPARE(
                    &htim1,
                    TIM_CHANNEL_1,
                    0);

                __HAL_TIM_SET_COMPARE(
                    &htim1,
                    TIM_CHANNEL_2,
                    0);

                osDelay(100);

                continue;
            }


            error = target_rpm - actual_rpm;


            integral += error * control_dt;


            if (integral > 3000.0f)
                integral = 3000.0f;

            if (integral < -3000.0f)
                integral = -3000.0f;


            output =
                base_pwm
                + kp * error
                + ki * integral;


            if (output > pwm_max)
                output = pwm_max;

            if (output < 0)
                output = 0;


						__HAL_TIM_SET_COMPARE(
								&htim1,
								TIM_CHANNEL_1,
								0);

						__HAL_TIM_SET_COMPARE(
								&htim1,
								TIM_CHANNEL_2,
								(uint32_t)output);	
            osDelay(100);
        }


        /* ==================================================
           最终保持120 RPM
           ================================================== */

        for(;;)
        {

            if (fault_state != FAULT_NONE)
            {
                __HAL_TIM_SET_COMPARE(
                    &htim1,
                    TIM_CHANNEL_1,
                    0);

                __HAL_TIM_SET_COMPARE(
                    &htim1,
                    TIM_CHANNEL_2,
                    0);

                osDelay(100);

                continue;
            }


            target_rpm = 120.0f;


            error = target_rpm - actual_rpm;


            integral += error * control_dt;


            if (integral > 3000.0f)
                integral = 3000.0f;

            if (integral < -3000.0f)
                integral = -3000.0f;


            output =
                base_pwm
                + kp * error
                + ki * integral;


            if (output > pwm_max)
                output = pwm_max;

            if (output < 0)
                output = 0;


						__HAL_TIM_SET_COMPARE(
								&htim1,
								TIM_CHANNEL_1,
								0);

						__HAL_TIM_SET_COMPARE(
								&htim1,
								TIM_CHANNEL_2,
								(uint32_t)output);

            osDelay(100);
        }
    }

    /* USER CODE END StartMotorTask */
}


/* USER CODE BEGIN Header_StartSensorTas */
/**
* @brief Function implementing the SensorTask thread.
*/
/* USER CODE END Header_StartSensorTas */

void StartSensorTas(void *argument)
{
    /* USER CODE BEGIN StartSensorTas */

    for(;;)
    {
        osDelay(100);
    }

    /* USER CODE END StartSensorTas */
}


/* USER CODE BEGIN Header_StartCommTask */
/**
* @brief Function implementing the CommTask thread.
*/
/* USER CODE END Header_StartCommTask */

void StartCommTask(void *argument)
{
    /* USER CODE BEGIN StartCommTask */

    uint16_t last_count = 0;
    uint16_t current_count = 0;

    int16_t delta_count = 0;

    float rpm = 0.0f;
    float rpm_filtered = 0.0f;

    uint8_t filter_initialized = 0;

    char msg[80];


    /*
     * 编码器每转计数
     *
     * 13 PPR
     * × 20 减速比
     * × 4 倍频
     *
     * = 1040
     */
    const float encoder_cpr = 1040.0f;


    /*
     * 100ms采样
     */
    const float sample_time = 0.1f;


    /*
     * 启动UART2中断接收
     */
    HAL_UART_Receive_IT(
        &huart2,
        (uint8_t *)&uart_rx_byte,
        1);


    /*
     * 记录初始编码器计数
     */
    last_count =
        __HAL_TIM_GET_COUNTER(&htim3);


    for(;;)
    {

        /* =========================
           读取编码器
           ========================= */

        current_count =
            __HAL_TIM_GET_COUNTER(&htim3);


        /*
         * int16_t可以自动处理TIM3溢出
         */
        delta_count =
            (int16_t)(current_count - last_count);


        last_count = current_count;


        /* =========================
           RPM计算
           ========================= */

        rpm =
            (float)delta_count
            * 60.0f
            / encoder_cpr
            / sample_time;


        /* =========================
           一阶低通滤波
           ========================= */

        if (filter_initialized == 0)
        {
            rpm_filtered = rpm;

            filter_initialized = 1;
        }
        else
        {
            rpm_filtered =
                0.7f * rpm_filtered
                + 0.3f * rpm;
        }


        /* =========================
           更新实际RPM
           ========================= */

        actual_rpm = rpm_filtered;


        /* =========================
           串口打印
           ========================= */

        snprintf(
            msg,
            sizeof(msg),
            "CNT=%u  RPM=%.2f  Filtered=%.2f\r\n",
            current_count,
            rpm,
            rpm_filtered);


        HAL_UART_Transmit(
            &huart2,
            (uint8_t *)msg,
            strlen(msg),
            100);


        osDelay(100);
    }

    /* USER CODE END StartCommTask */
}


/* USER CODE BEGIN Header_StartFaultTask */
/**
* @brief Function implementing the FaultTask thread.
*/
/* USER CODE END Header_StartFaultTask */

void StartFaultTask(void *argument)
{
    /* USER CODE BEGIN StartFaultTask */

    float error = 0.0f;

    uint16_t fault_count = 0;

    uint16_t recovery_timer = 0;

    float last_target_rpm = 0.0f;

    char msg[100];


    for(;;)
    {

        /* ==================================================
           UART CLEAR处理
           ================================================== */

        if (uart_clear_request)
        {
            uart_clear_request = 0;


            /*
             * 只有LOCKED状态才允许CLEAR
             */
            if (fault_state == FAULT_LOCKED)
            {

                fault_state = FAULT_NONE;


                /*
                 * 自动恢复次数清零
                 */
                recovery_count = 0;


                /*
                 * 清除故障计数
                 */
                fault_count = 0;


                /*
                 * 2秒故障检测抑制
                 *
                 * 20 × 100ms = 2s
                 */
                fault_inhibit_count = 20;


                /*
                 * 通知MotorTask重新启动
                 */
                motor_restart_request = 1;


                /*
                 * 先确保PWM为0
                 */
                __HAL_TIM_SET_COMPARE(
                    &htim1,
                    TIM_CHANNEL_1,
                    0);

                __HAL_TIM_SET_COMPARE(
                    &htim1,
                    TIM_CHANNEL_2,
                    0);


                snprintf(
                    msg,
                    sizeof(msg),
                    "FAULT CLEARED, MOTOR READY\r\n");


                HAL_UART_Transmit(
                    &huart2,
                    (uint8_t *)msg,
                    strlen(msg),
                    100);
            }
            else
            {

                snprintf(
                    msg,
                    sizeof(msg),
                    "CLEAR IGNORED, NO LOCKED FAULT\r\n");


                HAL_UART_Transmit(
                    &huart2,
                    (uint8_t *)msg,
                    strlen(msg),
                    100);
            }
        }


        /* ==================================================
           检测目标转速是否发生变化
           ================================================== */

        if (target_rpm != last_target_rpm)
        {
            last_target_rpm = target_rpm;

            /*
             * 目标转速改变后，
             * 2秒内暂时不进行故障判断
             */
            fault_inhibit_count = 20;

            fault_count = 0;
        }


        /* ==================================================
           正常状态
           ================================================== */

        if (fault_state == FAULT_NONE)
        {

            if (fault_inhibit_count > 0)
            {
                fault_inhibit_count--;
            }
            else
            {

                error =
                    target_rpm - actual_rpm;


                /*
                 * 目标转速>=50 RPM才进行检测
                 */
                if (target_rpm >= 50.0f)
                {

                    /*
                     * 速度误差超过30 RPM
                     */
                    if (error > 30.0f)
                    {

                        fault_count++;


                        /*
                         * 连续10次
                         *
                         * 10 × 100ms = 1秒
                         */
                        if (fault_count >= 10)
                        {

                            fault_state =
                                FAULT_SPEED;


                            fault_count = 0;


                            /*
                             * 故障后等待3秒
                             *
                             * 30 × 100ms = 3秒
                             */
                            recovery_timer = 30;


                            /*
                             * 立即关闭PWM
                             */
                            __HAL_TIM_SET_COMPARE(
                                &htim1,
                                TIM_CHANNEL_1,
                                0);

                            __HAL_TIM_SET_COMPARE(
                                &htim1,
                                TIM_CHANNEL_2,
                                0);


                            snprintf(
                                msg,
                                sizeof(msg),
                                "FAULT: SPEED ERROR, STOP\r\n");


                            HAL_UART_Transmit(
                                &huart2,
                                (uint8_t *)msg,
                                strlen(msg),
                                100);
                        }
                    }
                    else
                    {
                        /*
                         * 误差恢复正常
                         * 清除连续故障计数
                         */
                        fault_count = 0;
                    }
                }
                else
                {
                    fault_count = 0;
                }
            }
        }


        /* ==================================================
           SPEED故障状态
           ================================================== */

        else if (fault_state == FAULT_SPEED)
        {

            /*
             * 持续关闭PWM
             */
            __HAL_TIM_SET_COMPARE(
                &htim1,
                TIM_CHANNEL_1,
                0);

            __HAL_TIM_SET_COMPARE(
                &htim1,
                TIM_CHANNEL_2,
                0);


            /*
             * 等待恢复时间
             */
            if (recovery_timer > 0)
            {
                recovery_timer--;
            }
            else
            {

                /*
                 * 最多自动恢复3次
                 */
                if (recovery_count < 3)
                {

                    recovery_count++;


                    /*
                     * 请求MotorTask重新启动
                     */
                    motor_restart_request = 1;


                    /*
                     * 恢复正常状态
                     */
                    fault_state = FAULT_NONE;


                    /*
                     * 恢复后2秒内不判断故障
                     */
                    fault_inhibit_count = 20;


                    snprintf(
                        msg,
                        sizeof(msg),
                        "RECOVERY: TRY %d\r\n",
                        recovery_count);


                    HAL_UART_Transmit(
                        &huart2,
                        (uint8_t *)msg,
                        strlen(msg),
                        100);
                }
                else
                {

                    /*
                     * 自动恢复次数用完
                     */
                    fault_state =
                        FAULT_LOCKED;


                    /*
                     * 确保PWM关闭
                     */
                    __HAL_TIM_SET_COMPARE(
                        &htim1,
                        TIM_CHANNEL_1,
                        0);

                    __HAL_TIM_SET_COMPARE(
                        &htim1,
                        TIM_CHANNEL_2,
                        0);


                    snprintf(
                        msg,
                        sizeof(msg),
                        "FAULT: LOCKED, CLEAR REQUIRED\r\n");


                    HAL_UART_Transmit(
                        &huart2,
                        (uint8_t *)msg,
                        strlen(msg),
                        100);
                }
            }
        }


        /* ==================================================
           LOCKED状态
           ================================================== */

        else if (fault_state == FAULT_LOCKED)
        {

            /*
             * 锁定以后永久关闭PWM
             *
             * 必须通过CLEAR解除
             */
            __HAL_TIM_SET_COMPARE(
                &htim1,
                TIM_CHANNEL_1,
                0);

            __HAL_TIM_SET_COMPARE(
                &htim1,
                TIM_CHANNEL_2,
                0);
        }


        osDelay(100);
    }

    /* USER CODE END StartFaultTask */
}


/* USER CODE BEGIN Header_StartDisplayTask */
/**
* @brief Function implementing the DisplayTask thread.
*/
/* USER CODE END Header_StartDisplayTask */

void StartDisplayTask(void *argument)
{
    /* USER CODE BEGIN StartDisplayTask */

    for(;;)
    {
        osDelay(100);
    }

    /* USER CODE END StartDisplayTask */
}


/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */


/* ==========================================================
   UART2接收完成回调
   ========================================================== */

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{

    static char rx_buffer[10];

    static uint8_t rx_index = 0;


    /*
     * 确认是USART2
     */
    if (huart->Instance == USART2)
    {

        /* =========================
           收到回车或者换行
           ========================= */

        if (uart_rx_byte == '\r'
            || uart_rx_byte == '\n')
        {

            rx_buffer[rx_index] = '\0';


            /*
             * 判断是否输入CLEAR
             */
            if (strcmp(rx_buffer, "CLEAR") == 0)
            {
                uart_clear_request = 1;
            }


            /*
             * 准备接收下一条命令
             */
            rx_index = 0;


            memset(
                rx_buffer,
                0,
                sizeof(rx_buffer));
        }


        /* =========================
           普通字符
           ========================= */

        else
        {

            /*
             * 防止数组越界
             */
            if (rx_index < sizeof(rx_buffer) - 1)
            {

                rx_buffer[rx_index++] =
                    uart_rx_byte;
            }
            else
            {

                /*
                 * 缓冲区满了
                 * 放弃当前命令
                 */
                rx_index = 0;


                memset(
                    rx_buffer,
                    0,
                    sizeof(rx_buffer));
            }
        }


        /*
         * 继续接收下一个字符
         */
        HAL_UART_Receive_IT(
            &huart2,
            (uint8_t *)&uart_rx_byte,
            1);
    }
}


/* USER CODE END Application */