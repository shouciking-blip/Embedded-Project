#ifndef __UART_PROTOCOL_H
#define __UART_PROTOCOL_H

#include "main.h"
#include <stdint.h>

#define UART_PROTOCOL_HEADER    0xAA
#define UART_PROTOCOL_TAIL      0x55

#define UART_PROTOCOL_MAX_DATA  32
#define UART_PROTOCOL_QUEUE_SIZE 8

typedef struct
{
    uint8_t cmd;
    uint8_t len;
    uint8_t data[UART_PROTOCOL_MAX_DATA];
} UART_ProtocolFrame_t;


/* 初始化 */
void UART_Protocol_Init(void);

/* 输入一个字节 */

void UART_Protocol_InputByte(uint8_t byte);

/* CRC-8 */
uint8_t UART_Protocol_CRC8(const uint8_t *data, uint16_t len);

/* 获取已经解析完成的帧
   返回 1：成功获取
   返回 0：当前没有完整帧
*/
uint8_t UART_Protocol_GetFrame(UART_ProtocolFrame_t *frame);

/* 获取接收统计 */
uint32_t UART_Protocol_GetRxFrameCount(void);
uint32_t UART_Protocol_GetRxErrorCount(void);
uint32_t UART_Protocol_GetRxOverflowCount(void);

uint8_t UART_Protocol_IsBusy(void);
uint8_t UART_Protocol_IsFrameReady(void);

#endif
