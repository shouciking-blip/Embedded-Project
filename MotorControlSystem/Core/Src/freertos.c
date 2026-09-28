/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * 文件名             : freertos.c
  * 描述               : FreeRTOS 应用程序代码
  ******************************************************************************
  */
/* USER CODE END Header */

/* 包含头文件 ----------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

/* 私有包含头文件 ------------------------------------------------------------*/
/* USER CODE BEGIN Includes */

#include "usart.h"

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

#include "key_manager.h"
#include "buzzer_manager.h"

#include <stdio.h>
#include <string.h>

/* USER CODE END Includes */

/* 私有类型定义 --------------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* 私有宏定义 ----------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* 私有宏 --------------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* 私有变量 ------------------------------------------------------------------*/
/* USER CODE BEGIN Variables */

extern UART_HandleTypeDef huart2;

PIController_t motor_pi;
MotorConfig_t motor_config;

/* USER CODE END Variables */


/* MotorTask 定义 */
osThreadId_t MotorTaskHandle;

const osThreadAttr_t MotorTask_attributes = {
  .name = "MotorTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};


/* SensorTask 定义 */
osThreadId_t SensorTaskHandle;

const osThreadAttr_t SensorTask_attributes = {
  .name = "SensorTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};


/* CommTask 定义 */
osThreadId_t CommTaskHandle;

const osThreadAttr_t CommTask_attributes = {
  .name = "CommTask",
  .stack_size = 768 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};


/* FaultTask 定义 */
osThreadId_t FaultTaskHandle;

const osThreadAttr_t FaultTask_attributes = {
  .name = "FaultTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityHigh,
};


/* DisplayTask 定义 */
osThreadId_t DisplayTaskHandle;

const osThreadAttr_t DisplayTask_attributes = {
  .name = "DisplayTask",
  .stack_size = 512 * 4,
  .priority = (osPriority_t) osPriorityLow,
};


/* KeyTask 定义 */
osThreadId_t KeyTaskHandle;

const osThreadAttr_t KeyTask_attributes = {
  .name = "KeyTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};


/* 私有函数原型 --------------------------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */


void StartMotorTask(void *argument);
void StartSensorTask(void *argument);
void StartCommTask(void *argument);
void StartFaultTask(void *argument);
void StartDisplayTask(void *argument);
void StartKeyTask(void *argument);

void MX_FREERTOS_Init(void);


/**
  * @brief  FreeRTOS 初始化
  * @param  无
  * @retval 无
  */
void MX_FREERTOS_Init(void)
{
  /* USER CODE BEGIN Init */

  /* USER CODE END Init */


  /* USER CODE BEGIN RTOS_MUTEX */
  /* 添加互斥量等 */
  /* USER CODE END RTOS_MUTEX */


  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* 添加信号量等 */
  /* USER CODE END RTOS_SEMAPHORES */


  /* USER CODE BEGIN RTOS_TIMERS */
  /* 启动定时器、添加定时器等 */
  /* USER CODE END RTOS_TIMERS */


  /* USER CODE BEGIN RTOS_QUEUES */
  /* 添加队列等 */
  /* USER CODE END RTOS_QUEUES */


  /* 创建线程 */


  /* MotorTask */
  MotorTaskHandle =
      osThreadNew(StartMotorTask,
                  NULL,
                  &MotorTask_attributes);


  /* SensorTask */
  SensorTaskHandle =
      osThreadNew(StartSensorTask,
                  NULL,
                  &SensorTask_attributes);


  /* CommTask */
  CommTaskHandle =
      osThreadNew(StartCommTask,
                  NULL,
                  &CommTask_attributes);


  /* FaultTask */
  FaultTaskHandle =
      osThreadNew(StartFaultTask,
                  NULL,
                  &FaultTask_attributes);


  /* DisplayTask */
  DisplayTaskHandle =
      osThreadNew(StartDisplayTask,
                  NULL,
                  &DisplayTask_attributes);


  /* KeyTask */
  KeyTaskHandle =
      osThreadNew(StartKeyTask,
                  NULL,
                  &KeyTask_attributes);


  /* USER CODE BEGIN RTOS_THREADS */
  /* 添加线程等 */
  /* USER CODE END RTOS_THREADS */


  /* USER CODE BEGIN RTOS_EVENTS */
  /* 添加事件等 */
  /* USER CODE END RTOS_EVENTS */
}


/* USER CODE BEGIN Header_StartMotorTask */
/**
  * @brief 实现 MotorTask 线程的函数
  */
/* USER CODE END Header */

void StartMotorTask(void *argument)
{
  /* USER CODE BEGIN StartMotorTask */

  PI_Controller_Init(
      &motor_pi,
      20.0f,
      0.3f,
      2000.0f,
      4200.0f,
      -1000.0f,
      1000.0f
  );

  MotorControl_Init();

  motor_status.pwm = 0;


  for (;;)
  {
    /*
     * 有故障：
     * 立即停止电机
     */
    if (motor_status.fault != FAULT_NONE)
    {
      MotorControl_Stop();

      motor_status.pwm = 0;

      PI_Controller_Reset(&motor_pi);

      osDelay(100);

      continue;
    }


    /*
     * STOP 请求
     */
    if (motor_stop_request)
    {
      motor_stop_request = 0;
      motor_start_request = 0;

      motor_status.motor_running = 0;

      PI_Controller_Reset(&motor_pi);

      MotorControl_Stop();

      motor_status.pwm = 0;

      osDelay(100);

      continue;
    }


    /*
     * START 请求
     */
    if (motor_start_request)
    {
      motor_start_request = 0;
      motor_stop_request = 0;

      motor_status.motor_running = 1;

      if (motor_status.target_rpm <= 0.0f)
      {
        motor_status.target_rpm = 80.0f;
      }

      PI_Controller_Reset(&motor_pi);
    }


    /*
     * 电机没有运行
     */
    if (!motor_status.motor_running)
    {
      MotorControl_Stop();

      motor_status.pwm = 0;

      PI_Controller_Reset(&motor_pi);

      osDelay(100);

      continue;
    }


    /*
     * 编码器更新
     */
    EncoderManager_Update();


    /*
     * PI 控制
     */
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


    osDelay(100);
  }

  /* USER CODE END StartMotorTask */
}


/* USER CODE BEGIN Header_StartSensorTask */
/**
* @brief 实现 SensorTask 线程的函数
*/
/* USER CODE END Header */

void StartSensorTask(void *argument)
{
  /* USER CODE BEGIN StartSensorTask */

  SensorManager_Init();

  osDelay(100);


  for (;;)
  {
    SensorManager_Update();

    UART_Manager_SendSensorDebug();

    osDelay(500);
  }

  /* USER CODE END StartSensorTask */
}


/* USER CODE BEGIN Header_StartCommTask */
/**
* @brief 实现 CommTask 线程的函数
*/
/* USER CODE END Header */

void StartCommTask(void *argument)
{
  /* USER CODE BEGIN StartCommTask */

  uint8_t can_status_timer = 0;
  uint8_t monitor_timer = 0;


  UART_Manager_Init();

  EncoderManager_Init();


  /*
   * 从 Flash 读取配置
   */
  Flash_ConfigLoad(&motor_config);

  motor_status.target_rpm =
      motor_config.target_rpm;


  /*
   * 输出 Flash 配置
   */
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


    UART_Manager_SendString(flash_msg);
  }


  /*
   * 启动 CAN
   */
  CAN1_Start();


  for (;;)
  {
    /*
     * UART GET_STATUS
     */
    if (UART_Manager_GetStatusRequest())
    {
      UART_Manager_SendStatus();
    }


    /*
     * UART 延迟回复
     */
    UART_Manager_ProcessPendingReplies();


    /*
     * CAN 接收处理
     */
    CAN1_ProcessRx();


    osDelay(100);


    can_status_timer++;
    monitor_timer++;


    /*
     * CAN 状态发送
     * 500 ms
     */
    if (can_status_timer >= 5)
    {
      can_status_timer = 0;

      CAN1_SendMotorStatus();
    }


    /*
     * FreeRTOS 监控
     * 5 s
     */
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
* @brief 实现 FaultTask 线程的函数
*/
/* USER CODE END Header */

void StartFaultTask(void *argument)
{
  /* USER CODE BEGIN StartFaultTask */

  FaultManager_Init();


  for (;;)
  {
    /*
     * UART CLEAR
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
     * 故障检测
     */
    FaultManager_Update();


    osDelay(100);
  }

  /* USER CODE END StartFaultTask */
}


/* USER CODE BEGIN Header_StartDisplayTask */
/**
* @brief 实现 DisplayTask 线程的函数
*/
/* USER CODE END Header */

void StartDisplayTask(void *argument)
{
  /* USER CODE BEGIN StartDisplayTask */

  DisplayManager_Init();


  for (;;)
  {
    DisplayManager_Update();

    osDelay(500);
  }

  /* USER CODE END StartDisplayTask */
}


/* USER CODE BEGIN Header_StartKeyTask */
/**
* @brief 实现 KeyTask 线程的函数
*/
/* USER CODE END Header */

void StartKeyTask(void *argument)
{
  /* USER CODE BEGIN StartKeyTask */

  KeyEvent_t event;


  KeyManager_Init();

  BuzzerManager_Init();


  for (;;)
  {
    event = KeyManager_Scan();


    switch (event)
    {
				case KEY_EVENT_START:

						motor_start_request = 1;

						BuzzerManager_StartBeep();

						break;


				case KEY_EVENT_STOP:

						motor_stop_request = 1;

						BuzzerManager_StopBeep();

						break;


				case KEY_EVENT_MODE:

						BuzzerManager_ModeBeep();

						break;


				case KEY_EVENT_CLEAR:

						uart_clear_request = 1;

						BuzzerManager_ClearBeep();

						break;

      default:

        break;
    }


    osDelay(10);
  }

  /* USER CODE END StartKeyTask */
}


/* 私有应用程序代码 ----------------------------------------------------------*/
/* USER CODE BEGIN Application */


/*
 * UART 接收完成回调
 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  UART_Manager_RxCpltCallback(huart);
}


/* USER CODE END Application */

