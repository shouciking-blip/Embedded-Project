#include "uart_manager.h"
#include "FreeRTOS.h"
#include "semphr.h"
#include "usart.h"
#include "motor_status.h"
#include "fault_manager.h"
#include "sensor_manager.h"
#include "encoder_manager.h"
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "uart_protocol.h"

/* =========================================================
 * UART Manager 内部变量
 * ========================================================= */

static char rx_buffer[UART_RX_BUFFER_SIZE];
static uint8_t rx_index = 0;
static SemaphoreHandle_t uart_mutex = NULL;
static volatile uint8_t uart_status_request = 0;
static volatile uint8_t uart_set_speed_reply = 0;
static volatile uint8_t uart_command_error_reply = 0;
static float uart_pending_speed = 0.0f;
/* =========================================================
 * UART请求标志
 * ========================================================= */

volatile uint8_t motor_start_request = 0;
volatile uint8_t motor_stop_request = 0;
volatile uint8_t uart_clear_request = 0;
uint8_t uart_rx_byte = 0;
static volatile uint8_t uart_binary_mode = 0;

/* =========================================================
 * Fault State → String
 * ========================================================= */

static const char *UART_FaultStateToString(FaultState_t state)
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

        case FAULT_UNDERVOLTAGE:
            return "UNDERVOLTAGE";

        case FAULT_OVERVOLTAGE:
            return "OVERVOLTAGE";

        default:
            return "UNKNOWN";
    }
}


/* =========================================================
 * UART Manager初始化
 * ========================================================= */

void UART_Manager_Init(void)
{
    rx_index = 0;

    memset(
        rx_buffer,
        0,
        sizeof(rx_buffer)
    );

    uart_rx_byte = 0;
		uart_binary_mode = 0;
    motor_start_request = 0;
    motor_stop_request = 0;
    uart_clear_request = 0;

    /* 创建 UART 互斥锁 */
    uart_mutex = xSemaphoreCreateMutex();
		
				
		UART_Protocol_Init();	
				
    HAL_UART_Receive_IT(
        &huart2,
        (uint8_t *)&uart_rx_byte,
        1
    );
}


void UART_Manager_Lock(void)
{
    if (uart_mutex != NULL)
    {
        xSemaphoreTake(
            uart_mutex,
            portMAX_DELAY
        );
    }
}

void UART_Manager_Unlock(void)
{
    if (uart_mutex != NULL)
    {
        xSemaphoreGive(uart_mutex);
    }
}

/* =========================================================
 * UART发送字符串
 * ========================================================= */

void UART_Manager_SendString(const char *str)
{
    if (str == NULL)
    {
        return;
    }

    if (uart_mutex != NULL)
    {
        xSemaphoreTake(
            uart_mutex,
            portMAX_DELAY
        );
    }

    HAL_UART_Transmit(
        &huart2,
        (uint8_t *)str,
        strlen(str),
        100
    );

    if (uart_mutex != NULL)
    {
        xSemaphoreGive(uart_mutex);
    }
}


/* =========================================================
 * GET_STATUS
 * ========================================================= */

void UART_Manager_SendStatus(void)
{
    char status_msg[256];

    snprintf(
        status_msg,
        sizeof(status_msg),

        "\r\n"
        "===== MOTOR STATUS =====\r\n"
        "State   : %s\r\n"
        "Target  : %.1f RPM\r\n"
        "Actual  : %.1f RPM\r\n"
        "Current : %.3f A\r\n"
        "Temp    : %.1f C\r\n"
        "PWM     : %u\r\n"
        "Fault   : %s\r\n"
        "========================\r\n",

        motor_status.motor_running ?
            "RUNNING" :
            "STOPPED",

        motor_status.target_rpm,
        motor_status.actual_rpm,
        motor_status.current,
        motor_status.temperature,
        motor_status.pwm,

        UART_FaultStateToString(
            (FaultState_t)motor_status.fault
        )
    );

    UART_Manager_SendString(status_msg);
}

void UART_Manager_SendSensorDebug(void)
{
    char msg[256];

    snprintf(
        msg,
        sizeof(msg),

        "ADC0=%u  %.3fV  Current=%.3fA\r\n"
        "ADC1=%u  %.3fV  Bus=%.2fV\r\n"
        "ADC2=%u  %.3fV  NTC=%.1fkOhm\r\n"
        "Temp=%.1fC  Filtered=%.1fC  State=%s\r\n"
        "------------------------------\r\n",

        SensorManager_GetADC0(),
        SensorManager_GetADC0Voltage(),
        SensorManager_GetCurrentRaw(),

        SensorManager_GetADC1(),
        SensorManager_GetADC1Voltage(),
        motor_status.voltage,

        SensorManager_GetADC2(),
        SensorManager_GetADC2Voltage(),
        SensorManager_GetNTCResistance() / 1000.0f,

        SensorManager_GetTemperatureRaw(),
        motor_status.temperature,

        SensorManager_GetTemperatureState() == TEMP_NORMAL
            ? "NORMAL"
            :
            (
                SensorManager_GetTemperatureState() == TEMP_WARNING
                    ? "WARNING"
                    : "OVER_TEMP"
            )
    );

    UART_Manager_SendString(msg);
}

void UART_Manager_SendEncoderDebug(void)
{
    char msg[128];

    snprintf(
        msg,
        sizeof(msg),

        "CNT=%u  RPM=%.2f  Filtered=%.2f\r\n",

        EncoderManager_GetCount(),
        EncoderManager_GetRawRPM(),
        EncoderManager_GetRPM()
    );

    UART_Manager_SendString(msg);
}

void UART_Manager_SendFaultCleared(void)
{
    UART_Manager_SendString(
        "FAULT CLEARED\r\n"
    );
}

/* =========================================================
 * UART命令处理
 * ========================================================= */

static void UART_Manager_ProcessCommand(char *command)
{
    /* =====================================================
     * CLEAR
     * ===================================================== */

    if (strcmp(command, "CLEAR") == 0)
    {
        uart_clear_request = 1;
    }


    /* =====================================================
     * START
     * ===================================================== */

    else if (strcmp(command, "START") == 0)
    {
        motor_start_request = 1;
    }


    /* =====================================================
     * STOP
     * ===================================================== */

    else if (strcmp(command, "STOP") == 0)
    {
        motor_stop_request = 1;
    }


    /* =====================================================
     * GET_STATUS
     * ===================================================== */

			 else if (strcmp(command, "GET_STATUS") == 0)
			{
					uart_status_request = 1;
			}


    /* =====================================================
     * SET_SPEED xxx
     * ===================================================== */

				else if (strncmp(command, "SET_SPEED ", 10) == 0)
				{
						float speed = 0.0f;

						if (sscanf(command + 10, "%f", &speed) == 1)
						{
								if (speed >= 0.0f && speed <= 300.0f)
								{
										motor_status.target_rpm = speed;

										uart_pending_speed = speed;
										uart_set_speed_reply = 1;
								}
								else
								{
										uart_command_error_reply = 1;
								}
						}
						else
						{
								uart_command_error_reply = 2;
						}
				}


    /* =====================================================
     * 未知命令
     * ===================================================== */

				else if (rx_index > 0)
				{
						uart_command_error_reply = 4;
				}
}


/* =========================================================
 * UART接收完成回调
 * ========================================================= */

void UART_Manager_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart != &huart2)
    {
        return;
    }

    /*
     * AA 表示二进制帧开始
     */
    if (uart_rx_byte == UART_PROTOCOL_HEADER)
    {
        uart_binary_mode = 1;
        rx_index = 0;
    }

    /*
     * 二进制模式
     */
    if (uart_binary_mode)
    {
        UART_Protocol_InputByte(uart_rx_byte);

        /*
         * 这里暂时不能简单判断一帧是否结束，
         * 因为第一阶段我们暂时不提供 Parser 状态 API。
         *
         * 下一阶段处理帧时再统一解决。
         */
    }
    else
    {
        /*
         * 原来的文本协议
         */

        if (uart_rx_byte == '\r' ||
            uart_rx_byte == '\n')
        {
            if (rx_index > 0)
            {
                rx_buffer[rx_index] = '\0';

                UART_Manager_ProcessCommand(rx_buffer);

                rx_index = 0;
            }
        }
        else
        {
            if (rx_index < UART_RX_BUFFER_SIZE - 1)
            {
                rx_buffer[rx_index++] =
                    (char)uart_rx_byte;
            }
            else
            {
                rx_index = 0;
            }
        }
    }

    HAL_UART_Receive_IT(
        &huart2,
        &uart_rx_byte,
        1
    );
}

uint8_t UART_Manager_GetStatusRequest(void)
{
    if (uart_status_request)
    {
        uart_status_request = 0;
        return 1;
    }

    return 0;
}

void UART_Manager_ProcessPendingReplies(void)
{
    char msg[64];

    if (uart_set_speed_reply)
    {
        uart_set_speed_reply = 0;

        snprintf(
            msg,
            sizeof(msg),
            "SET SPEED: %.1f RPM\r\n",
            uart_pending_speed
        );

        UART_Manager_SendString(msg);
    }

    if (uart_command_error_reply == 1)
    {
        uart_command_error_reply = 0;

        UART_Manager_SendString(
            "ERROR: SPEED RANGE 0-300 RPM\r\n"
        );
    }
    else if (uart_command_error_reply == 2)
    {
        uart_command_error_reply = 0;

        UART_Manager_SendString(
            "ERROR: INVALID SPEED\r\n"
        );
    }


		else if (uart_command_error_reply == 3)
		{
				uart_command_error_reply = 0;

				UART_Manager_SendString(
						"ERROR: COMMAND TOO LONG\r\n"
				);
		}
		
		else if (uart_command_error_reply == 4)
		{
				uart_command_error_reply = 0;

				UART_Manager_SendString(
						"ERROR: UNKNOWN COMMAND\r\n"
				);
		}

}
