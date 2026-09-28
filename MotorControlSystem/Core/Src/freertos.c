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
#include "flash_config.h"
#include "motor_status.h"
#include "motor_control.h"
#include "pi_controller.h"
#include "fault_manager.h"
#include "sensor_manager.h"
#include "can_manager.h"
#include "display_manager.h"
#include "encoder_manager.h"
#include "uart_manager.h"
#include "freertos_monitor.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "usart.h"
#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */



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

/* =========================
   电机控制参数
   ========================= */
PIController_t motor_pi;
MotorConfig_t motor_config;


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
  .stack_size = 768 * 4,
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
void StartSensorTask(void *argument);
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
  SensorTaskHandle = osThreadNew(StartSensorTask, NULL, &SensorTask_attributes);

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
void StartMotorTask(void *argument)
{
    /* =========================================
     * 1. PI控制器初始化
     * ========================================= */
    PI_Controller_Init(
        &motor_pi,
        20.0f,      /* Kp */
        0.3f,       /* Ki */
        2000.0f,   /* base PWM */
        4200.0f,   /* max PWM */
        -1000.0f,  /* integral min */
        1000.0f    /* integral max */
    );

    /* =========================================
     * 2. 电机控制模块初始化
     * ========================================= */
    MotorControl_Init();

    motor_status.pwm = 0;

    /* =========================================
     * 3. MotorTask 主循环
     * ========================================= */
    for (;;)
    {
        /* =====================================
         * 3.1 故障状态
         * ===================================== */
        if (motor_status.fault != FAULT_NONE)
        {
            MotorControl_Stop();

            motor_status.pwm = 0;

            PI_Controller_Reset(&motor_pi);

            osDelay(100);

            continue;
        }

        /* =====================================
         * 3.2 STOP 请求
         * ===================================== */
        if (motor_stop_request)
        {
            motor_stop_request = 0;

            /* 取消尚未执行的 START 请求 */
            motor_start_request = 0;

            /* 电机进入停止状态 */
            motor_status.motor_running = 0;

            /* 清除 PI 积分 */
            PI_Controller_Reset(&motor_pi);

            /* 停止电机 */
            MotorControl_Stop();

            motor_status.pwm = 0;

            osDelay(100);

            continue;
        }

        /* =====================================
         * 3.3 START 请求
         * ===================================== */
        if (motor_start_request)
        {
            motor_start_request = 0;

            /* 取消尚未执行的 STOP 请求 */
            motor_stop_request = 0;

            motor_status.motor_running = 1;

            /* 如果没有设定目标速度，默认 80 RPM */
            if (motor_status.target_rpm <= 0.0f)
            {
                motor_status.target_rpm = 80.0f;
            }

            /* 清除 PI 积分 */
            PI_Controller_Reset(&motor_pi);
        }

        /* =====================================
         * 3.4 电机未运行
         * ===================================== */
        if (!motor_status.motor_running)
        {
            MotorControl_Stop();

            motor_status.pwm = 0;

            PI_Controller_Reset(&motor_pi);

            osDelay(100);

            continue;
        }
				
					/* =====================================
					 * 3.5 编码器测速
					 * ===================================== */

					EncoderManager_Update();
        /* =====================================
         * 3.6 PI 速度控制
         * ===================================== */
        {
            float output;

            output = PI_Controller_Update(
                &motor_pi,
                motor_status.target_rpm,
                motor_status.actual_rpm,
                0.1f
            );

            motor_status.pwm = (uint16_t)output;

            MotorControl_SetPWM(motor_status.pwm);
        }

        /* =====================================
         * 3.6 控制周期：100ms
         * ===================================== */
        osDelay(100);
    }
}
  /* USER CODE END StartMotorTask */


/* USER CODE BEGIN Header_StartSensorTask */
/**
* @brief Function implementing the SensorTask thread.
* @param argument: Not used
* @retval None
*/
/* USER CODE END Header_StartSensorTask */
void StartSensorTask(void *argument)
{
    SensorManager_Init();

    osDelay(100);

    for (;;)
    {
        SensorManager_Update();

        UART_Manager_SendSensorDebug();

        osDelay(500);
    }
}

    /* USER CODE END StartSensorTask */


/* USER CODE BEGIN Header_StartCommTask */
/**
* @brief Function implementing the CommTask thread.
*/


/* USER CODE END Header_StartCommTask */
void StartCommTask(void *argument)
{
	

    /* =========================================
     * CommTask 局部变量
     * ========================================= */
			char msg[256];

			uint8_t can_status_timer = 0;
			uint8_t monitor_timer = 0;
    /* =========================================
     * 1. UART接收初始化
     * ========================================= */

			UART_Manager_Init();


    /* =========================================
     * 2. 编码器初始值
     * ========================================= */
			EncoderManager_Init();

    /* =========================================
     * 3. Flash配置加载
     * ========================================= */

    Flash_ConfigLoad(&motor_config);

    motor_status.target_rpm =
        motor_config.target_rpm;


    {
        char flash_msg[128];

        snprintf(
            flash_msg,
            sizeof(flash_msg),

            "\r\n===== FLASH CONFIG =====\r\n"
            "Kp       : %.2f\r\n"
            "Ki       : %.2f\r\n"
            "Target   : %.2f RPM\r\n"
            "CRC      : 0x%08lX\r\n"
            "========================\r\n",

            motor_config.kp,
            motor_config.ki,
            motor_config.target_rpm,
            (unsigned long)motor_config.crc
        );

        HAL_UART_Transmit(
            &huart2,
            (uint8_t *)flash_msg,
            strlen(flash_msg),
            100
        );
    }


    /* =========================================
     * 4. CAN初始化
     * ========================================= */

    CAN1_Start();


    /* =========================================
     * 5. CommTask主循环
     * ========================================= */

    for (;;)
    {


				if (UART_Manager_GetStatusRequest())
				{
						UART_Manager_SendStatus();
				}
				UART_Manager_ProcessPendingReplies();
        /* =====================================
         * 5.6 UART测速调试输出
         * ===================================== */

				/*	snprintf(
							msg,
							sizeof(msg),

							"CNT=%u  RPM=%.2f  Filtered=%.2f\r\n",

							EncoderManager_GetCount(),
							EncoderManager_GetRawRPM(),
							EncoderManager_GetRPM()
					);

        HAL_UART_Transmit(
            &huart2,
            (uint8_t *)msg,
            strlen(msg),
            100
        );
*/

        /* =====================================
         * 5.7 CAN接收
         * ===================================== */

        CAN1_ProcessRx();


        /* =====================================
         * 5.8 周期
         * ===================================== */

        osDelay(100);

        can_status_timer++;
				monitor_timer++;
        /* =====================================
         * 5.9 每500ms发送一次CAN状态
         * ===================================== */

				 if (can_status_timer >= 5)
						{
								can_status_timer = 0;

								CAN1_SendMotorStatus();
						}

					if (monitor_timer >= 50)
					{
							monitor_timer = 0;

							FreeRTOS_Monitor_PrintTasks();
							FreeRTOS_Monitor_PrintCPU();
					}
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
    FaultManager_Init();

    for (;;)
    {
        /*
         * CLEAR 请求
         *
         * UART / CAN 收到 CLEAR 后，
         * 最终都会通过这个请求标志进入 FaultManager。
         */
					if (uart_clear_request)
					{
							uart_clear_request = 0;

							if (FaultManager_Clear())
							{
									UART_Manager_SendFaultCleared();
							}
					}

        /*
         * 故障管理器周期运行
         *
         * 包括：
         * 1. 温度保护
         * 2. 过流保护
         * 3. 欠压保护
         * 4. 过压保护
         * 5. 堵转保护
         * 6. 转速异常
         * 7. 自动恢复
         * 8. LOCKED
         */
        FaultManager_Update();

        osDelay(100);
    }
		    /* USER CODE END StartFaultTask */
}



/* USER CODE BEGIN Header_StartDisplayTask */
/**
* @brief Function implementing the DisplayTask thread.
*/
/* USER CODE END Header_StartDisplayTask */

  /* USER CODE BEGIN StartDisplayTask */
void StartDisplayTask(void * argument)
{
    DisplayManager_Init();

    for (;;)
    {
        DisplayManager_Update();

        osDelay(500);
    }
}
  /* USER CODE END StartDisplayTask */


/* Private application code --------------------------------------------------*/
/* USER CODE BEGIN Application */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    UART_Manager_RxCpltCallback(huart);
}


/* USER CODE END Application */

