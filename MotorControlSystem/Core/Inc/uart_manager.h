#ifndef __UART_MANAGER_H__
#define __UART_MANAGER_H__

#include "main.h"
#include <stdint.h>

/* =========================================================
 * UART Manager
 * ========================================================= */

/* UART接收缓冲区大小 */
#define UART_RX_BUFFER_SIZE    64

/* UART命令请求标志 */
extern volatile uint8_t motor_start_request;
extern volatile uint8_t motor_stop_request;
extern volatile uint8_t uart_clear_request;

/* UART接收字节 */
extern uint8_t uart_rx_byte;

/* =========================================================
 * 初始化
 * ========================================================= */

/**
 * @brief UART Manager初始化
 */
void UART_Manager_Init(void);


/* =========================================================
 * UART发送
 * ========================================================= */

/**
 * @brief 发送字符串
 */
void UART_Manager_SendString(const char *str);


/* =========================================================
 * UART状态
 * ========================================================= */

/**
 * @brief 发送当前电机状态
 */
void UART_Manager_SendStatus(void);
void UART_Manager_SendSensorDebug(void);
void UART_Manager_SendEncoderDebug(void);
void UART_Manager_SendFaultCleared(void);
void UART_Manager_Lock(void);
void UART_Manager_Unlock(void);
uint8_t UART_Manager_GetStatusRequest(void);
void UART_Manager_ProcessPendingReplies(void);
void UART_Manager_SendString(const char *str);
/* =========================================================
 * UART接收回调
 * ========================================================= */

/**
 * @brief UART接收完成处理
 *
 * 在 HAL_UART_RxCpltCallback() 中调用
 */
void UART_Manager_RxCpltCallback(UART_HandleTypeDef *huart);


#endif /* __UART_MANAGER_H__ */
