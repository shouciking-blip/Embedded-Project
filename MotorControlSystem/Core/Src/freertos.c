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
#include "adc.h"
#include "math.h"

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
    FAULT_TEMP,
    FAULT_OVERCURRENT,
    FAULT_UNDERVOLTAGE,
    FAULT_OVERVOLTAGE,
    FAULT_STALL,
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
extern ADC_HandleTypeDef hadc1;

uint16_t adc_buffer[3 * 32] = {0};

/* =========================
   电机控制参数
   ========================= */

volatile float actual_rpm = 0.0f;
volatile float target_rpm = 80.0f;
volatile uint8_t motor_running = 0;

volatile uint8_t motor_start_request = 0;
volatile uint8_t motor_stop_request = 0;
volatile uint8_t uart_clear_request = 0;
volatile uint8_t uart_status_request = 0;
volatile uint16_t current_pwm = 0;
float kp = 20.0f;
float ki = 0.3f;

uint32_t motor_start_time = 0;
uint8_t motor_starting = 0;

/* =========================
   UART CLEAR相关
   ========================= */

volatile uint8_t uart_rx_byte = 0;


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
/* ADC */
volatile float current_a = 0.0f;
volatile float adc0_voltage = 0.0f;
volatile float adc1_voltage = 0.0f;
volatile float adc2_voltage = 0.0f;

volatile float bus_voltage = 0.0f;
volatile float bus_voltage_filtered = 0.0f;

float temperature_c = 0.0f;
float ntc_resistance = 0.0f;
float temperature_filtered = 0.0f;
float current_filtered = 0.0f;
typedef enum
{
    TEMP_NORMAL = 0,
    TEMP_WARNING,
    TEMP_OVER
} TemperatureState_t;


TemperatureState_t temperature_state = TEMP_NORMAL;

#define TEMP_WARNING_THRESHOLD   60.0f
#define TEMP_OVER_THRESHOLD      70.0f

#define TEMP_FILTER_ALPHA        0.2f
#define CURRENT_OVER_THRESHOLD    0.15f
#define CURRENT_FILTER_ALPHA      0.2f
#define CURRENT_TEST_MODE         0

#define STALL_PWM_THRESHOLD    50.0f
#define STALL_RPM_THRESHOLD    20.0f
#define STALL_TIME_COUNT       5
#define STALL_TEST_MODE 0

#define VOLTAGE_UNDER_THRESHOLD  6.5f
#define VOLTAGE_OVER_THRESHOLD   8.0f

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

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */

/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of MotorTask */
  MotorTaskHandle = osThreadNew(StartMotorTask, NULL, &MotorTask_attributes);

  /* creation of SensorTask */
  SensorTaskHandle = osThreadNew(StartSensorTas, NULL, &SensorTask_attributes);

  /* creation of CommTask */
  CommTaskHandle = osThreadNew(StartCommTask, NULL, &CommTask_attributes);

  /* creation of FaultTask */
  FaultTaskHandle = osThreadNew(StartFaultTask, NULL, &FaultTask_attributes);

  /* creation of DisplayTask */
  DisplayTaskHandle = osThreadNew(StartDisplayTask, NULL, &DisplayTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartMotorTask */
/**
  * @brief Function implementing the MotorTask thread.
  */
/* USER CODE END Header_StartMotorTask */
//	电机控制转动
void StartMotorTask(void *argument)
{
    float error = 0.0f;
    float integral = 0.0f;
    float output = 0.0f;

    uint16_t pwm = 0;

    const float pwm_max = 4200.0f;
    const float control_dt = 0.1f;

    /* 给电机一个初始PWM，避免低于死区 */
    const float base_pwm = 2000.0f;


    /* =====================================
     * 1. 使能 IBT-2
     * ===================================== */

    HAL_GPIO_WritePin(
        GPIOE,
        GPIO_PIN_4,
        GPIO_PIN_SET);

    HAL_GPIO_WritePin(
        GPIOE,
        GPIO_PIN_5,
        GPIO_PIN_SET);


    /* =====================================
     * 2. 启动 PWM
     * ===================================== */

    HAL_TIM_PWM_Start(
        &htim1,
        TIM_CHANNEL_1);

    HAL_TIM_PWM_Start(
        &htim1,
        TIM_CHANNEL_2);


    /* =====================================
     * 3. 初始停止
     * ===================================== */

    __HAL_TIM_SET_COMPARE(
        &htim1,
        TIM_CHANNEL_1,
        0);

    __HAL_TIM_SET_COMPARE(
        &htim1,
        TIM_CHANNEL_2,
        0);

    current_pwm = 0;


    /* =====================================
     * 4. 主循环
     * ===================================== */

    for(;;)
    {

        /* =====================================
         * 4.1 故障状态
         * ===================================== */

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

            current_pwm = 0;

            /* 清除积分 */
            integral = 0.0f;

            osDelay(100);

            continue;
        }


        /* =====================================
         * 4.2 STOP 请求
         * ===================================== */

        if (motor_stop_request)
        {
            motor_stop_request = 0;

            /* 真正进入停止状态 */
            motor_running = 0;

            /* 清除积分 */
            integral = 0.0f;

            /* PWM关闭 */
            __HAL_TIM_SET_COMPARE(
                &htim1,
                TIM_CHANNEL_1,
                0);

            __HAL_TIM_SET_COMPARE(
                &htim1,
                TIM_CHANNEL_2,
                0);

            current_pwm = 0;

            osDelay(100);

            continue;
        }


        /* =====================================
         * 4.3 START 请求
         * ===================================== */

        if (motor_start_request)
        {
						motor_start_request = 0;
					
						motor_running = 1;

						motor_start_time = HAL_GetTick();
						motor_starting = 1;


            /* 进入运行状态 */
            motor_running = 1;

            /*
             * 如果没有设定目标速度，
             * 默认使用80 RPM
             */
            if (target_rpm <= 0.0f)
            {
                target_rpm = 80.0f;
            }

            /* 清除积分 */
            integral = 0.0f;
        }


        /* =====================================
         * 4.4 电机未运行
         * ===================================== */

        if (!motor_running)
        {
            __HAL_TIM_SET_COMPARE(
                &htim1,
                TIM_CHANNEL_1,
                0);

            __HAL_TIM_SET_COMPARE(
                &htim1,
                TIM_CHANNEL_2,
                0);

            current_pwm = 0;

            integral = 0.0f;

            osDelay(100);

            continue;
        }


        /* =====================================
         * 4.5 PI速度控制
         * ===================================== */

        error = target_rpm - actual_rpm;


        /* 积分 */
        integral +=
            error * control_dt;


        /* =====================================
         * 4.6 积分限幅
         * ===================================== */

        if (integral > 1000.0f)
        {
            integral = 1000.0f;
        }

        if (integral < -1000.0f)
        {
            integral = -1000.0f;
        }


        /* =====================================
         * 4.7 PI输出
         * ===================================== */

        output =
            base_pwm
            + kp * error
            + ki * integral;


        /* =====================================
         * 4.8 PWM限幅
         * ===================================== */

        if (output > pwm_max)
        {
            output = pwm_max;
        }

        if (output < 0.0f)
        {
            output = 0.0f;
        }


        pwm = (uint16_t)output;

        /* 保存当前PWM，供GET_STATUS使用 */
        current_pwm = pwm;


        /* =====================================
         * 4.9 正转
         * ===================================== */

        __HAL_TIM_SET_COMPARE(
            &htim1,
            TIM_CHANNEL_1,
            0);

        __HAL_TIM_SET_COMPARE(
            &htim1,
            TIM_CHANNEL_2,
            pwm);


        /* =====================================
         * 4.10 控制周期100ms
         * ===================================== */

        osDelay(100);
    }
}

/* USER CODE BEGIN Header_StartSensorTas */
/**
* @brief Function implementing the SensorTask thread.
*/
/* USER CODE END Header_StartSensorTas */
//获取电流/电压/温度信息并转换
void StartSensorTas(void *argument)
{
    char msg[200];

    uint16_t adc0;
    uint16_t adc1;
    uint16_t adc2;

    uint32_t sum_adc0;
    uint32_t sum_adc1;
    uint32_t sum_adc2;

    uint8_t i;

    const char *temp_state_str;

    /*
     * =========================================================
     * ADC1 + DMA启动
     *
     * ADC扫描顺序：
     *
     * Rank 1 -> PA0 -> ADC0 -> INA180电流
     * Rank 2 -> PA1 -> ADC1 -> 电压
     * Rank 3 -> PA4 -> ADC2 -> NTC温度
     *
     * 每组：
     * [PA0, PC3, PA4]
     *
     * 一共32组：
     * 3 × 32 = 96个数据
     * =========================================================
     */

    if (HAL_ADC_Start_DMA(&hadc1,
                          (uint32_t *)adc_buffer,
                          3 * 32) != HAL_OK)
    {
        Error_Handler();
    }


    /*
     * 等待ADC和DMA开始稳定工作
     */
    osDelay(100);


    /*
     * =========================================================
     * 主循环
     * =========================================================
     */

    for (;;)
    {
        /*
         * -----------------------------------------------------
         * 1. 清零累加变量
         * -----------------------------------------------------
         */

        sum_adc0 = 0;
        sum_adc1 = 0;
        sum_adc2 = 0;


        /*
         * -----------------------------------------------------
         * 2. 读取32组ADC数据
         *
         * DMA数据排列：
         *
         * adc_buffer[0]  = PA0
         * adc_buffer[1]  = PA1
         * adc_buffer[2]  = PA4
         *
         * adc_buffer[3]  = PA0
         * adc_buffer[4]  = PA1
         * adc_buffer[5]  = PA4
         *
         * ...
         * -----------------------------------------------------
         */

        for (i = 0; i < 32; i++)
        {
            sum_adc0 += adc_buffer[i * 3];
            sum_adc1 += adc_buffer[i * 3 + 1];
            sum_adc2 += adc_buffer[i * 3 + 2];
        }


        /*
         * -----------------------------------------------------
         * 3. 计算32次平均值
         * -----------------------------------------------------
         */

        adc0 = (uint16_t)(sum_adc0 / 32);

        adc1 = (uint16_t)(sum_adc1 / 32);

        adc2 = (uint16_t)(sum_adc2 / 32);


        /*
         * =====================================================
         * 4. ADC值转换为电压
         *
         * ADC满量程：
         *
         * 0 ~ 4095
         *
         * Vref = 3.3V
         * =====================================================
         */

        adc0_voltage =
            (float)adc0 * 3.3f / 4095.0f;

        adc1_voltage =
            (float)adc1 * 3.21f / 4095.0f;

        adc2_voltage =
            (float)adc2 * 3.3f / 4095.0f;
				/*
				 * =====================================================
				 * 电机电源电压计算
				 *
				 * 分压电阻：
				 * R_TOP    = 47KΩ
				 * R_BOTTOM = 10KΩ
				 *
				 * PA1 = Vmotor × 10 / (47 + 10)
				 *
				 * 所以：
				 * Vmotor = VPA1 × 5.7
				 * =====================================================
				 */
				#define VOLTAGE_R_RATIO       5.7f
				#define VOLTAGE_CALIBRATION 0.887f
				bus_voltage =
						adc1_voltage * VOLTAGE_R_RATIO * VOLTAGE_CALIBRATION;
				/*
			 * =====================================================
			 * 电压一阶低通滤波
			 * =====================================================
			 */

				{
						static uint8_t voltage_filter_initialized = 0;

						if (!voltage_filter_initialized)
						{
								bus_voltage_filtered = bus_voltage;
								voltage_filter_initialized = 1;
						}
						else
						{
								bus_voltage_filtered =
										0.2f * bus_voltage
										+
										0.8f * bus_voltage_filtered;
						}
				}		

        /*
         * =====================================================
         * 5. INA180电流计算
         *
         * INA180：
         *
         * Shunt = 0.01Ω
         * Gain  = 50V/V
         *
         * Vout = I × 0.01 × 50
         *
         * 所以：
         *
         * I = Vout / 0.5
         * =====================================================
         */

        current_a = adc0_voltage / (0.01f * 50.0f);
				

				#if CURRENT_TEST_MODE

						/* 软件模拟过流，用于验证 FaultTask */
						current_filtered = 0.20f;

				#else

						{
								static uint8_t current_filter_initialized = 0;

								if (!current_filter_initialized)
								{
										current_filtered = current_a;
										current_filter_initialized = 1;
								}
								else
								{
										current_filtered =
												CURRENT_FILTER_ALPHA * current_a
												+ (1.0f - CURRENT_FILTER_ALPHA) * current_filtered;
								}
						}

				#endif

        /*
         * =====================================================
         * 6. NTC电阻计算
         *
         * 你的实际模块：
         *
         *             3.3V
         *              │
         *             10K
         *              │
         *              ├──── AO → PA4
         *              │
         *             NTC
         *              │
         *             GND
         *
         * 根据分压公式：
         *
         * Rntc = 10K × ADC / (4095 - ADC)
         * =====================================================
         */

        if (adc2 < 4090)
        {
            ntc_resistance =
                10000.0f *
                (float)adc2 /
                (4095.0f - (float)adc2);
        }
        else
        {
            ntc_resistance = 0.0f;
        }


        /*
         * =====================================================
         * 7. NTC温度计算
         *
         * 当前采用经过实际测试后的初始参数：
         *
         * R25 = 10KΩ
         * B   = 3200K
         *
         * T25 = 25°C = 298.15K
         *
         * Beta公式：
         *
         * 1/T =
         * 1/T25 +
         * (1/B) × ln(R/R25)
         *
         * 最终：
         *
         * Temperature = T - 273.15
         * =====================================================
         */

        if (ntc_resistance > 0.0f)
        {
            float R25 = 10000.0f;
            float B = 3200.0f;
            float T25 = 298.15f;

            temperature_c =
                1.0f /
                (
                    1.0f / T25 +
                    (1.0f / B) *
                    logf(ntc_resistance / R25)
                );

            temperature_c =
                temperature_c - 273.15f;
        }
        else
        {
            temperature_c = 0.0f;
        }


        /*
         * =====================================================
         * 8. 温度一阶低通滤波
         *
         * 第一次采样：
         *
         * filtered = temperature
         *
         * 后续：
         *
         * filtered =
         *     0.2 × 当前温度
         *     +
         *     0.8 × 上一次滤波温度
         * =====================================================
         */

        {
            static uint8_t temp_filter_initialized = 0;

            if (!temp_filter_initialized)
            {
                temperature_filtered = temperature_c;

                temp_filter_initialized = 1;
            }
            else
            {
                temperature_filtered =
                    TEMP_FILTER_ALPHA * temperature_c
                    +
                    (1.0f - TEMP_FILTER_ALPHA)
                    * temperature_filtered;
            }
        }


        /*
         * =====================================================
         * 9. 温度状态判断
         *
         * < 60°C
         *     NORMAL
         *
         * 60~70°C
         *     WARNING
         *
         * >= 70°C
         *     OVER_TEMP
         * =====================================================
         */

        if (temperature_filtered >= TEMP_OVER_THRESHOLD)
        {
            temperature_state = TEMP_OVER;
        }
        else if (temperature_filtered >= TEMP_WARNING_THRESHOLD)
        {
            temperature_state = TEMP_WARNING;
        }
        else
        {
            temperature_state = TEMP_NORMAL;
        }


        /*
         * =====================================================
         * 10. 将状态转换成字符串
         * =====================================================
         */

        if (temperature_state == TEMP_NORMAL)
        {
            temp_state_str = "NORMAL";
        }
        else if (temperature_state == TEMP_WARNING)
        {
            temp_state_str = "WARNING";
        }
        else
        {
            temp_state_str = "OVER_TEMP";
        }


        /*
         * =====================================================
         * 11. UART输出
         * =====================================================
         */

				snprintf(
						msg,
						sizeof(msg),

						"ADC0=%u  %.3fV  Current=%.3fA\r\n"
						"ADC1=%u  %.3fV  Bus=%.2fV\r\n"
						"ADC2=%u  %.3fV  NTC=%.1fkOhm\r\n"
						"Temp=%.1fC  Filtered=%.1fC  State=%s\r\n"
						"------------------------------\r\n",

						adc0,
						adc0_voltage,
						current_a,

						adc1,
						adc1_voltage,
						bus_voltage_filtered,

						adc2,
						adc2_voltage,
						ntc_resistance / 1000.0f,

						temperature_c,
						temperature_filtered,
						temp_state_str
				);

        /*
         * UART2发送
         */
        HAL_UART_Transmit(
            &huart2,
            (uint8_t *)msg,
            strlen(msg),
            100
        );


        /*
         * 每500ms输出一次
         */
        osDelay(500);
    }
}

/* USER CODE BEGIN Header_StartCommTask */
/**
* @brief Function implementing the CommTask thread.
*/

const char* FaultStateToString(FaultState_t state)
{
    switch (state)
    {
        case FAULT_NONE:
            return "NONE";

        case FAULT_SPEED:
            return "SPEED";

        case FAULT_TEMP:
            return "TEMP";

        case FAULT_OVERCURRENT:
            return "OVERCURRENT";

        case FAULT_STALL:
            return "STALL";

        case FAULT_LOCKED:
            return "LOCKED";

        default:
            return "UNKNOWN";
    }
}
/* USER CODE END Header_StartCommTask */
//获取当前转速信息
void StartCommTask(void *argument)
{
    /* USER CODE BEGIN StartCommTask */

    uint16_t last_count = 0;
    uint16_t current_count = 0;

    int16_t delta_count = 0;

    float rpm = 0.0f;
    float rpm_filtered = 0.0f;

    uint8_t filter_initialized = 0;

    char msg[256];

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


    /* =====================================
     * 主循环
     * ===================================== */

    for(;;)
    {

        /* =====================================
         * 1. GET_STATUS
         * ===================================== */

        if (uart_status_request)
        {
            uart_status_request = 0;


            snprintf(
                msg,
                sizeof(msg),

                "\r\n"
                "===== MOTOR STATUS =====\r\n"

                "State   : %s\r\n"

                "Target  : %.1f RPM\r\n"

                "Actual  : %.1f RPM\r\n"

                "Current : %.2f A\r\n"

                "Temp    : %.1f C\r\n"

                "PWM     : %u\r\n"

                "Fault   : %s\r\n"

                "========================\r\n",

                motor_running ?
                "RUNNING" :
                "STOPPED",

                target_rpm,

                actual_rpm,

                current_filtered,

                temperature_filtered,

                current_pwm,

                FaultStateToString(fault_state)
            );


            HAL_UART_Transmit(
                &huart2,
                (uint8_t *)msg,
                strlen(msg),
                100);
        }


        /* =====================================
         * 2. 读取编码器
         * ===================================== */

        current_count =
            __HAL_TIM_GET_COUNTER(&htim3);


        /*
         * int16_t自动处理TIM3溢出
         */
        delta_count =
            (int16_t)(
                current_count -
                last_count
            );


        last_count =
            current_count;


        /* =====================================
         * 3. RPM计算
         * ===================================== */

        rpm =
            (float)delta_count
            * 60.0f
            / encoder_cpr
            / sample_time;


        /* =====================================
         * 4. 一阶低通滤波
         * ===================================== */

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


        /* =====================================
         * 5. 更新实际RPM
         * ===================================== */

        actual_rpm =
            rpm_filtered;


        /* =====================================
         * 6. 串口打印测速数据
         * ===================================== */

        snprintf(
            msg,
            sizeof(msg),

            "CNT=%u  RPM=%.2f  Filtered=%.2f\r\n",

            current_count,

            rpm,

            rpm_filtered
        );


        HAL_UART_Transmit(
            &huart2,
            (uint8_t *)msg,
            strlen(msg),
            100);


        /* =====================================
         * 7. 100ms周期
         * ===================================== */

        osDelay(100);
    }

    /* USER CODE END StartCommTask */
}

/* USER CODE BEGIN Header_StartFaultTask */
/**
* @brief Function implementing the FaultTask thread.
*/
/* USER CODE END Header_StartFaultTask */
//故障检测
void StartFaultTask(void *argument)
{
  /* USER CODE BEGIN StartFaultTask */

    float error = 0.0f;

    uint16_t fault_count = 0;
    uint16_t temp_fault_count = 0;
    uint16_t overcurrent_fault_count = 0;
    uint16_t stall_fault_count = 0;
		uint16_t undervoltage_fault_count = 0;
		uint16_t overvoltage_fault_count = 0;
    uint16_t recovery_timer = 0;

    float last_target_rpm = 0.0f;

    char msg[120];

    uint8_t temp_recovered_reported = 0;
    uint8_t current_recovered_reported = 0;

    for(;;)
    {
        /* =========================================================
         * 1. 处理 UART CLEAR 请求
         * ========================================================= */
        if (uart_clear_request)
        {
            uart_clear_request = 0;

				if (fault_state == FAULT_LOCKED ||
						fault_state == FAULT_TEMP ||
						fault_state == FAULT_OVERCURRENT ||
						fault_state == FAULT_UNDERVOLTAGE ||
						fault_state == FAULT_OVERVOLTAGE ||
						fault_state == FAULT_STALL)
            {
                /* -------------------------
                 * 温度故障
                 * ------------------------- */
                if (fault_state == FAULT_TEMP &&
                    temperature_filtered >= TEMP_OVER_THRESHOLD)
                {
                    snprintf(msg,
                             sizeof(msg),
                             "CLEAR REJECTED, TEMP TOO HIGH\r\n");

                    HAL_UART_Transmit(&huart2,
                                      (uint8_t *)msg,
                                      strlen(msg),
                                      100);
                }

                /* -------------------------
                 * 过流故障
                 * ------------------------- */
                else if (fault_state == FAULT_OVERCURRENT &&
                         current_filtered >= CURRENT_OVER_THRESHOLD)
                {
                    snprintf(msg,
                             sizeof(msg),
                             "CLEAR REJECTED, CURRENT TOO HIGH\r\n");

                    HAL_UART_Transmit(&huart2,
                                      (uint8_t *)msg,
                                      strlen(msg),
                                      100);
                }

                /* -------------------------
                 * 堵转故障
                 *
                 * 堵转需要人工 CLEAR
                 * 不自动恢复
                 * ------------------------- */
                else if (fault_state == FAULT_STALL)
                {
                    fault_state = FAULT_NONE;

                    recovery_count = 0;

                    fault_count = 0;
                    temp_fault_count = 0;
                    overcurrent_fault_count = 0;
                    stall_fault_count = 0;

                    temp_recovered_reported = 0;
                    current_recovered_reported = 0;

                    fault_inhibit_count = 20;

                    motor_restart_request = 1;

                    __HAL_TIM_SET_COMPARE(&htim1,
                                          TIM_CHANNEL_1,
                                          0);

                    __HAL_TIM_SET_COMPARE(&htim1,
                                          TIM_CHANNEL_2,
                                          0);

                    snprintf(msg,
                             sizeof(msg),
                             "FAULT CLEARED, MOTOR READY\r\n");

                    HAL_UART_Transmit(&huart2,
                                      (uint8_t *)msg,
                                      strlen(msg),
                                      100);
                }
								/* -------------------------
								 * 欠压故障
								 * ------------------------- */
								else if (fault_state == FAULT_UNDERVOLTAGE &&
												 bus_voltage < VOLTAGE_UNDER_THRESHOLD)
								{
										snprintf(msg,
														 sizeof(msg),
														 "CLEAR REJECTED, VOLTAGE TOO LOW\r\n");

										HAL_UART_Transmit(&huart2,
																			(uint8_t *)msg,
																			strlen(msg),
																			100);
								}

								/* -------------------------
								 * 过压故障
								 * ------------------------- */
								else if (fault_state == FAULT_OVERVOLTAGE &&
												 bus_voltage > VOLTAGE_OVER_THRESHOLD)
								{
										snprintf(msg,
														 sizeof(msg),
														 "CLEAR REJECTED, VOLTAGE TOO HIGH\r\n");

										HAL_UART_Transmit(&huart2,
																			(uint8_t *)msg,
																			strlen(msg),
																			100);
								}

                /* -------------------------
                 * LOCKED 或其他可 CLEAR 故障
                 * ------------------------- */
                else
                {
                    fault_state = FAULT_NONE;

                    recovery_count = 0;

                    fault_count = 0;
                    temp_fault_count = 0;
                    overcurrent_fault_count = 0;
                    stall_fault_count = 0;
										undervoltage_fault_count = 0;
										overvoltage_fault_count = 0;
                    temp_recovered_reported = 0;
                    current_recovered_reported = 0;

                    fault_inhibit_count = 20;

                    motor_restart_request = 1;

                    __HAL_TIM_SET_COMPARE(&htim1,
                                          TIM_CHANNEL_1,
                                          0);

                    __HAL_TIM_SET_COMPARE(&htim1,
                                          TIM_CHANNEL_2,
                                          0);

                    snprintf(msg,
                             sizeof(msg),
                             "FAULT CLEARED, MOTOR READY\r\n");

                    HAL_UART_Transmit(&huart2,
                                      (uint8_t *)msg,
                                      strlen(msg),
                                      100);
                }
            }
            else
            {
                snprintf(msg,
                         sizeof(msg),
                         "CLEAR IGNORED, NO LOCKED FAULT\r\n");

                HAL_UART_Transmit(&huart2,
                                  (uint8_t *)msg,
                                  strlen(msg),
                                  100);
            }
        }


        /* =========================================================
         * 2. 检测目标转速是否发生变化
         * ========================================================= */
					 if (target_rpm != last_target_rpm)
				{
						last_target_rpm = target_rpm;

						fault_inhibit_count = 20;

						fault_count = 0;
						temp_fault_count = 0;
						overcurrent_fault_count = 0;
						undervoltage_fault_count = 0;
						overvoltage_fault_count = 0;
						stall_fault_count = 0;
				}


        /* =========================================================
         * 3. 正常状态 FAULT_NONE
         *
         * 故障优先级：
         *
         * 温度
         *   ↓
         * 过流
         *   ↓
         * 堵转
         *   ↓
         * 速度异常
         * ========================================================= */
        if (fault_state == FAULT_NONE)
        {
            /* =====================================================
             * 3.1 温度保护
             * ===================================================== */
            if (temperature_filtered >= TEMP_OVER_THRESHOLD)
            {
                temp_fault_count++;

                if (temp_fault_count >= 3)
                {
                    fault_state = FAULT_TEMP;

                    temp_fault_count = 0;
                    fault_count = 0;
                    stall_fault_count = 0;

                    /* 立即关闭 PWM */
                    __HAL_TIM_SET_COMPARE(&htim1,
                                          TIM_CHANNEL_1,
                                          0);

                    __HAL_TIM_SET_COMPARE(&htim1,
                                          TIM_CHANNEL_2,
                                          0);

                    temp_recovered_reported = 0;

                    snprintf(msg,
                             sizeof(msg),
                             "FAULT: OVER TEMP, STOP\r\n");

                    HAL_UART_Transmit(&huart2,
                                      (uint8_t *)msg,
                                      strlen(msg),
                                      100);
                }
            }
            else
            {
                temp_fault_count = 0;
            }


            /* 如果已经进入温度故障，
             * 不继续检测其他故障 */
            if (fault_state != FAULT_NONE)
            {
                osDelay(100);
                continue;
            }


            /* =====================================================
             * 3.2 过流保护
             * ===================================================== */
            if (current_filtered >= CURRENT_OVER_THRESHOLD)
            {
                overcurrent_fault_count++;

                if (overcurrent_fault_count >= 3)
                {
                    fault_state = FAULT_OVERCURRENT;

                    overcurrent_fault_count = 0;
                    fault_count = 0;
                    stall_fault_count = 0;

                    /* 立即关闭 PWM */
                    __HAL_TIM_SET_COMPARE(&htim1,
                                          TIM_CHANNEL_1,
                                          0);

                    __HAL_TIM_SET_COMPARE(&htim1,
                                          TIM_CHANNEL_2,
                                          0);

                    current_recovered_reported = 0;

                    snprintf(msg,
                             sizeof(msg),
                             "FAULT: OVERCURRENT, STOP\r\n");

                    HAL_UART_Transmit(&huart2,
                                      (uint8_t *)msg,
                                      strlen(msg),
                                      100);
                }
            }
            else
            {
                overcurrent_fault_count = 0;
            }


            /* 如果已经进入过流故障，
             * 不继续检测 */
            if (fault_state != FAULT_NONE)
            {
                osDelay(100);
                continue;
            }
				/* =====================================================
				 * 3.3 电压保护
				 *
				 * 欠压：Bus < 6.5V
				 * 过压：Bus > 8.0V
				 *
				 * 连续 3 次异常才触发
				 * 100ms × 3 = 300ms
				 * ===================================================== */

				if (bus_voltage < VOLTAGE_UNDER_THRESHOLD)
				{
						undervoltage_fault_count++;

						/* 电压恢复正常，清除过压计数 */
						overvoltage_fault_count = 0;

						if (undervoltage_fault_count >= 3)
						{
								fault_state = FAULT_UNDERVOLTAGE;

								undervoltage_fault_count = 0;
								fault_count = 0;
								stall_fault_count = 0;

								/* 立即关闭 PWM */
								__HAL_TIM_SET_COMPARE(
										&htim1,
										TIM_CHANNEL_1,
										0);

								__HAL_TIM_SET_COMPARE(
										&htim1,
										TIM_CHANNEL_2,
										0);

								snprintf(msg,
												 sizeof(msg),
												 "FAULT: UNDER VOLTAGE, STOP\r\n");

								HAL_UART_Transmit(
										&huart2,
										(uint8_t *)msg,
										strlen(msg),
										100);
						}
				}
				else if (bus_voltage > VOLTAGE_OVER_THRESHOLD)
				{
						overvoltage_fault_count++;

						/* 电压恢复正常，清除欠压计数 */
						undervoltage_fault_count = 0;

						if (overvoltage_fault_count >= 3)
						{
								fault_state = FAULT_OVERVOLTAGE;

								overvoltage_fault_count = 0;
								fault_count = 0;
								stall_fault_count = 0;

								/* 立即关闭 PWM */
								__HAL_TIM_SET_COMPARE(
										&htim1,
										TIM_CHANNEL_1,
										0);

								__HAL_TIM_SET_COMPARE(
										&htim1,
										TIM_CHANNEL_2,
										0);

								snprintf(msg,
												 sizeof(msg),
												 "FAULT: OVER VOLTAGE, STOP\r\n");

								HAL_UART_Transmit(
										&huart2,
										(uint8_t *)msg,
										strlen(msg),
										100);
						}
				}
				else
				{
						/* 电压正常 */
						undervoltage_fault_count = 0;
						overvoltage_fault_count = 0;
				}


				/* 如果已经进入电压故障，
				 * 不继续检测其他故障 */
				if (fault_state != FAULT_NONE)
				{
						osDelay(100);
						continue;
				}						

/* =====================================================
 * 3.4 堵转保护
 *
 * 正常条件：
 * 目标速度 >= 50 RPM
 * PWM >= 50%
 * 实际 RPM < 20
 * 持续 500ms
 *
 * 启动保护：
 * START 后 800ms 内暂不进行堵转判断
 * ===================================================== */
{
    float pwm_percent = 0.0f;

    uint32_t pwm_ch1;
    uint32_t pwm_ch2;
    uint32_t pwm_max;

    float stall_rpm;

    /* -------------------------------------------------
     * 启动保护计时
     *
     * START 后 800ms 内：
     * motor_starting = 1
     *
     * 800ms 后：
     * motor_starting = 0
     * ------------------------------------------------- */
    if (motor_starting)
    {
        if (HAL_GetTick() - motor_start_time >= 800)
        {
            motor_starting = 0;
        }
    }

    /* -------------------------------------------------
     * 软件模拟堵转
     *
     * 仅用于验证 FaultTask 堵转保护逻辑。
     * 不修改 Encoder 实际测速值 actual_rpm。
     * ------------------------------------------------- */
#if STALL_TEST_MODE

    stall_rpm = 0.0f;

#else

    stall_rpm = actual_rpm;

#endif

    if (target_rpm >= 50.0f)
    {
        pwm_ch1 =
            __HAL_TIM_GET_COMPARE(&htim1,
                                  TIM_CHANNEL_1);

        pwm_ch2 =
            __HAL_TIM_GET_COMPARE(&htim1,
                                  TIM_CHANNEL_2);

        /* 正反转取较大的 PWM */
        pwm_max =
            (pwm_ch1 > pwm_ch2) ?
            pwm_ch1 :
            pwm_ch2;

        pwm_percent =
            ((float)pwm_max / 8399.0f) * 100.0f;


#if STALL_TEST_MODE

        /*
         * 测试模式：
         * 不考虑 PWM，只模拟 RPM=0
         */
        if (!motor_starting &&
            stall_rpm < STALL_RPM_THRESHOLD)

#else

        /*
         * 正常模式：
         * 启动保护结束后才允许判断堵转
         */
        if (!motor_starting &&
            pwm_percent >= STALL_PWM_THRESHOLD &&
            stall_rpm < STALL_RPM_THRESHOLD)

#endif
        {
            stall_fault_count++;

            if (stall_fault_count >= STALL_TIME_COUNT)
            {
                fault_state = FAULT_STALL;

                stall_fault_count = 0;
                fault_count = 0;

                /* 立即关闭 PWM */
                __HAL_TIM_SET_COMPARE(
                    &htim1,
                    TIM_CHANNEL_1,
                    0);

                __HAL_TIM_SET_COMPARE(
                    &htim1,
                    TIM_CHANNEL_2,
                    0);

                snprintf(msg,
                         sizeof(msg),
                         "FAULT: MOTOR STALL, STOP\r\n");

                HAL_UART_Transmit(
                    &huart2,
                    (uint8_t *)msg,
                    strlen(msg),
                    100);
            }
        }
        else
        {
            stall_fault_count = 0;
        }
    }
    else
    {
        stall_fault_count = 0;
    }
}


            /* 如果已经进入堵转故障，
             * 不继续检测速度故障 */
            if (fault_state != FAULT_NONE)
            {
                osDelay(100);
                continue;
            }


            /* =====================================================
             * 3.4 速度异常保护
             * ===================================================== */

            if (fault_inhibit_count > 0)
            {
                fault_inhibit_count--;
            }
            else
            {
                error = target_rpm - actual_rpm;

                if (motor_running && target_rpm >= 50.0f)
                {
										error  = target_rpm - actual_rpm;
                    if (error > 30.0f)
                    {
                        fault_count++;

                        if (fault_count >= 10)
                        {
                            fault_state = FAULT_SPEED;

                            fault_count = 0;

                            recovery_timer = 30;

                            /* 立即关闭 PWM */
                            __HAL_TIM_SET_COMPARE(
                                &htim1,
                                TIM_CHANNEL_1,
                                0);

                            __HAL_TIM_SET_COMPARE(
                                &htim1,
                                TIM_CHANNEL_2,
                                0);

                            snprintf(msg,
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
                        fault_count = 0;
                    }
                }
                else
                {
                    fault_count = 0;
                }
            }
        }


        /* =========================================================
         * 4. 速度故障
         *
         * 原有自动恢复机制
         * ========================================================= */
        else if (fault_state == FAULT_SPEED)
        {
            /* PWM保持关闭 */
            __HAL_TIM_SET_COMPARE(&htim1,
                                  TIM_CHANNEL_1,
                                  0);

            __HAL_TIM_SET_COMPARE(&htim1,
                                  TIM_CHANNEL_2,
                                  0);


            if (recovery_timer > 0)
            {
                recovery_timer--;
            }
            else
            {
                if (recovery_count < 3)
                {
                    recovery_count++;

                    motor_restart_request = 1;

                    fault_state = FAULT_NONE;

                    fault_inhibit_count = 20;

                    snprintf(msg,
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
                    fault_state = FAULT_LOCKED;

                    __HAL_TIM_SET_COMPARE(
                        &htim1,
                        TIM_CHANNEL_1,
                        0);

                    __HAL_TIM_SET_COMPARE(
                        &htim1,
                        TIM_CHANNEL_2,
                        0);

                    snprintf(msg,
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


        /* =========================================================
         * 5. 温度故障
         *
         * 不自动重新启动
         * 温度恢复后等待 CLEAR
         * ========================================================= */
        else if (fault_state == FAULT_TEMP)
        {
            /* PWM保持关闭 */
            __HAL_TIM_SET_COMPARE(&htim1,
                                  TIM_CHANNEL_1,
                                  0);

            __HAL_TIM_SET_COMPARE(&htim1,
                                  TIM_CHANNEL_2,
                                  0);


            if (temperature_filtered < TEMP_OVER_THRESHOLD)
            {
                if (temp_recovered_reported == 0)
                {
                    temp_recovered_reported = 1;

                    snprintf(msg,
                             sizeof(msg),
                             "TEMP RECOVERED, SEND CLEAR\r\n");

                    HAL_UART_Transmit(
                        &huart2,
                        (uint8_t *)msg,
                        strlen(msg),
                        100);
                }
            }
            else
            {
                temp_recovered_reported = 0;
            }
        }


        /* =========================================================
         * 6. 过流故障
         *
         * 不自动重新启动
         * 电流恢复后等待 CLEAR
         * ========================================================= */
        else if (fault_state == FAULT_OVERCURRENT)
        {
            /* PWM保持关闭 */
            __HAL_TIM_SET_COMPARE(&htim1,
                                  TIM_CHANNEL_1,
                                  0);

            __HAL_TIM_SET_COMPARE(&htim1,
                                  TIM_CHANNEL_2,
                                  0);


            if (current_filtered < CURRENT_OVER_THRESHOLD)
            {
                if (current_recovered_reported == 0)
                {
                    current_recovered_reported = 1;

                    snprintf(msg,
                             sizeof(msg),
                             "CURRENT RECOVERED, SEND CLEAR\r\n");

                    HAL_UART_Transmit(
                        &huart2,
                        (uint8_t *)msg,
                        strlen(msg),
                        100);
                }
            }
            else
            {
                current_recovered_reported = 0;
            }
        }


        /* =========================================================
         * 7. 堵转故障
         *
         * 不自动重新启动
         * 必须人工 CLEAR
         * ========================================================= */
        else if (fault_state == FAULT_STALL)
        {
            /* PWM保持关闭 */
            __HAL_TIM_SET_COMPARE(&htim1,
                                  TIM_CHANNEL_1,
                                  0);

            __HAL_TIM_SET_COMPARE(&htim1,
                                  TIM_CHANNEL_2,
                                  0);

            /* 堵转状态下不自动恢复 */
        }


        /* =========================================================
         * 8. LOCKED
         *
         * 原有最终锁定状态
         * 必须人工 CLEAR
         * ========================================================= */
        else if (fault_state == FAULT_LOCKED)
        {
            /* PWM保持关闭 */
            __HAL_TIM_SET_COMPARE(&htim1,
                                  TIM_CHANNEL_1,
                                  0);

            __HAL_TIM_SET_COMPARE(&htim1,
                                  TIM_CHANNEL_2,
                                  0);
        }


        /* =========================================================
         * FaultTask周期
         * 100ms
         * ========================================================= */
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
//串口打印
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    static char rx_buffer[32];
    static uint8_t rx_index = 0;

    if (huart->Instance == USART2)
    {
        /* =========================
           收到回车或者换行
           ========================= */
        if (uart_rx_byte == '\r' || uart_rx_byte == '\n')
        {
            rx_buffer[rx_index] = '\0';

            /* =========================
               CLEAR
               ========================= */
            if (strcmp(rx_buffer, "CLEAR") == 0)
            {
                uart_clear_request = 1;
            }

            /* =========================
               START
               ========================= */
            else if (strcmp(rx_buffer, "START") == 0)
            {
                motor_start_request = 1;
            }

            /* =========================
               STOP
               ========================= */
            else if (strcmp(rx_buffer, "STOP") == 0)
            {
                motor_stop_request = 1;
            }
						else if (strcmp(rx_buffer, "GET_STATUS") == 0)
						{
								char status_msg[256];

								snprintf(status_msg, sizeof(status_msg),
												 "\r\n===== MOTOR STATUS =====\r\n"
												 "State   : %s\r\n"
												 "Target  : %.1f RPM\r\n"
												 "Actual  : %.1f RPM\r\n"
												 "Current : %.3f A\r\n"
												 "Temp    : %.1f C\r\n"
												 "PWM     : %d\r\n"
												 "Fault   : %d\r\n"
												 "========================\r\n",
												 motor_running ? "RUNNING" : "STOPPED",
												 target_rpm,
												 actual_rpm,
												 current_filtered,
												 temperature_filtered,
												 current_pwm,
												 fault_state);

								HAL_UART_Transmit(&huart2,
																	(uint8_t *)status_msg,
																	strlen(status_msg),
																	100);
						}

            /* =========================
               SET_SPEED xxx
               ========================= */
            else if (strncmp(rx_buffer, "SET_SPEED ", 10) == 0)
            {
                float speed = 0.0f;

                if (sscanf(rx_buffer + 10, "%f", &speed) == 1)
                {
                    if (speed >= 0.0f && speed <= 300.0f)
                    {
                        target_rpm = speed;

                        char msg[64];

                        snprintf(
                            msg,
                            sizeof(msg),
                            "SET SPEED: %.1f RPM\r\n",
                            target_rpm);

                        HAL_UART_Transmit(
                            &huart2,
                            (uint8_t *)msg,
                            strlen(msg),
                            100);
                    }
                    else
                    {
                        char msg[] =
                            "ERROR: SPEED RANGE 0-300 RPM\r\n";

                        HAL_UART_Transmit(
                            &huart2,
                            (uint8_t *)msg,
                            strlen(msg),
                            100);
                    }
                }
                else
                {
                    char msg[] =
                        "ERROR: INVALID SPEED\r\n";

                    HAL_UART_Transmit(
                        &huart2,
                        (uint8_t *)msg,
                        strlen(msg),
                        100);
                }
            }

            /* =========================
               未知命令
               ========================= */
            else if (rx_index > 0)
            {
                char msg[] =
                    "ERROR: UNKNOWN COMMAND\r\n";

                HAL_UART_Transmit(
                    &huart2,
                    (uint8_t *)msg,
                    strlen(msg),
                    100);
            }

            /* =========================
               清空接收缓冲区
               ========================= */
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
            if (rx_index < sizeof(rx_buffer) - 1)
            {
                rx_buffer[rx_index++] = uart_rx_byte;
            }
            else
            {
                /* 缓冲区溢出，丢弃当前命令 */

                rx_index = 0;

                memset(
                    rx_buffer,
                    0,
                    sizeof(rx_buffer));

                char msg[] =
                    "ERROR: COMMAND TOO LONG\r\n";

                HAL_UART_Transmit(
                    &huart2,
                    (uint8_t *)msg,
                    strlen(msg),
                    100);
            }
        }

        /* =========================
           继续接收下一个字符
           ========================= */
        HAL_UART_Receive_IT(
            &huart2,
            (uint8_t *)&uart_rx_byte,
            1);
    }
}

/* USER CODE END Application */

