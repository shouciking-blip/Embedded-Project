#ifndef __CAN_MANAGER_H__
#define __CAN_MANAGER_H__

#include "main.h"

/* =========================
 * CAN ID
 * ========================= */

#define CAN_ID_MOTOR_CMD      0x101
#define CAN_ID_MOTOR_STATUS   0x201


/* =========================
 * CAN RX Software Queue
 * ========================= */

#define CAN_RX_QUEUE_SIZE     8


typedef struct
{
    CAN_RxHeaderTypeDef header;
    uint8_t data[8];

} CAN_RxMessage_t;


/* =========================
 * CAN Manager API
 * ========================= */

void CAN1_Start(void);

void CAN1_SendCommand(
    uint8_t command,
    uint16_t parameter
);

void CAN1_SendStart(void);

void CAN1_SendStop(void);

void CAN1_SendSetSpeed(
    float speed
);

void CAN1_SendClear(void);

void CAN1_SendSave(void);

void CAN1_SendMotorStatus(void);

void CAN1_ProcessRx(void);


/* =========================
 * RX Queue API
 * ========================= */

uint8_t CAN1_RX_QueuePush(
    CAN_RxHeaderTypeDef *header,
    uint8_t *data
);

uint8_t CAN1_RX_QueuePop(
    CAN_RxHeaderTypeDef *header,
    uint8_t *data
);


/* =========================
 * CAN Statistics
 * ========================= */

extern volatile uint32_t can_rx_count;

extern volatile uint32_t can_rx_overflow_count;

#endif
