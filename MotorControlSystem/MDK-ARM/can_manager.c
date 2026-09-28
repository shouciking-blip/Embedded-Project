#include "can_manager.h"

#include "can.h"
#include "usart.h"

#include "motor_status.h"
#include "flash_config.h"

#include <stdio.h>
#include <string.h>


/* =========================================================
 * 外部变量
 * ========================================================= */

/* FreeRTOS / Application 层请求 */
extern volatile uint8_t motor_start_request;
extern volatile uint8_t motor_stop_request;
extern volatile uint8_t uart_clear_request;


/* Flash 配置 */
extern MotorConfig_t motor_config;


/* =========================================================
 * CAN RX Software Queue
 * ========================================================= */

static CAN_RxMessage_t can_rx_queue[CAN_RX_QUEUE_SIZE];

static volatile uint8_t can_rx_write_index = 0;

static volatile uint8_t can_rx_read_index = 0;


/* =========================================================
 * CAN Statistics
 * ========================================================= */

volatile uint32_t can_rx_count = 0;

volatile uint32_t can_rx_overflow_count = 0;


/* =========================================================
 * CAN Filter
 * ========================================================= */

static void CAN1_FilterConfig(void)
{
    CAN_FilterTypeDef FilterConfig;

    FilterConfig.FilterBank = 0;

    FilterConfig.FilterMode =
        CAN_FILTERMODE_IDMASK;

    FilterConfig.FilterScale =
        CAN_FILTERSCALE_32BIT;

    FilterConfig.FilterIdHigh = 0x0000;

    FilterConfig.FilterIdLow = 0x0000;

    FilterConfig.FilterMaskIdHigh = 0x0000;

    FilterConfig.FilterMaskIdLow = 0x0000;

    FilterConfig.FilterFIFOAssignment =
        CAN_RX_FIFO0;

    FilterConfig.FilterActivation =
        ENABLE;

    FilterConfig.SlaveStartFilterBank = 14;

    if (HAL_CAN_ConfigFilter(
            &hcan1,
            &FilterConfig
        ) != HAL_OK)
    {
        Error_Handler();
    }
}


/* =========================================================
 * CAN Start
 * ========================================================= */

void CAN1_Start(void)
{
    CAN1_FilterConfig();

    if (HAL_CAN_Start(&hcan1) != HAL_OK)
    {
        Error_Handler();
    }

    if (HAL_CAN_ActivateNotification(
            &hcan1,
            CAN_IT_RX_FIFO0_MSG_PENDING
        ) != HAL_OK)
    {
        Error_Handler();
    }
}


/* =========================================================
 * RX Queue Push
 * ========================================================= */

uint8_t CAN1_RX_QueuePush(
    CAN_RxHeaderTypeDef *header,
    uint8_t *data
)
{
    uint8_t next_write_index;

    next_write_index =
        (can_rx_write_index + 1)
        % CAN_RX_QUEUE_SIZE;


    /* 队列满 */
    if (next_write_index == can_rx_read_index)
    {
        can_rx_overflow_count++;

        return 0;
    }


    /* 保存 Header */
    can_rx_queue[
        can_rx_write_index
    ].header = *header;


    /* 保存 Data */
    for (uint8_t i = 0; i < 8; i++)
    {
        can_rx_queue[
            can_rx_write_index
        ].data[i] = data[i];
    }


    /* 更新写指针 */
    can_rx_write_index =
        next_write_index;


    return 1;
}


/* =========================================================
 * RX Queue Pop
 * ========================================================= */

uint8_t CAN1_RX_QueuePop(
    CAN_RxHeaderTypeDef *header,
    uint8_t *data
)
{
    /* 队列为空 */
    if (can_rx_read_index ==
        can_rx_write_index)
    {
        return 0;
    }


    /* 读取 Header */
    *header =
        can_rx_queue[
            can_rx_read_index
        ].header;


    /* 读取 Data */
    for (uint8_t i = 0; i < 8; i++)
    {
        data[i] =
            can_rx_queue[
                can_rx_read_index
            ].data[i];
    }


    /* 更新读指针 */
    can_rx_read_index =
        (can_rx_read_index + 1)
        % CAN_RX_QUEUE_SIZE;


    return 1;
}


/* =========================================================
 * CAN Send Command
 * ========================================================= */

void CAN1_SendCommand(
    uint8_t command,
    uint16_t parameter
)
{
    CAN_TxHeaderTypeDef tx_header;

    uint8_t tx_data[8];

    uint32_t tx_mailbox;

    HAL_StatusTypeDef status;

    uint32_t free_mailboxes;

    char msg[128];


    /* =====================================
     * 1. 检查发送邮箱
     * ===================================== */

    free_mailboxes =
        HAL_CAN_GetTxMailboxesFreeLevel(
            &hcan1
        );


    snprintf(
        msg,
        sizeof(msg),
        "CAN TX BEFORE: CMD=0x%02X PARAM=%u FREE=%lu\r\n",
        command,
        parameter,
        (unsigned long)free_mailboxes
    );


    HAL_UART_Transmit(
        &huart2,
        (uint8_t *)msg,
        strlen(msg),
        100
    );


    /* =====================================
     * 2. 配置 CAN TX Header
     * ===================================== */

    tx_header.StdId =
        CAN_ID_MOTOR_CMD;

    tx_header.ExtId = 0x00;

    tx_header.IDE =
        CAN_ID_STD;

    tx_header.RTR =
        CAN_RTR_DATA;

    tx_header.DLC = 8;

    tx_header.TransmitGlobalTime =
        DISABLE;


    /* =====================================
     * 3. 填充 CAN Data
     * ===================================== */

    tx_data[0] = command;

    tx_data[1] =
        (parameter >> 8) & 0xFF;

    tx_data[2] =
        parameter & 0xFF;

    tx_data[3] = 0x00;

    tx_data[4] = 0x00;

    tx_data[5] = 0x00;

    tx_data[6] = 0x00;

    tx_data[7] = 0x00;


    /* =====================================
     * 4. 发送
     * ===================================== */

    status =
        HAL_CAN_AddTxMessage(
            &hcan1,
            &tx_header,
            tx_data,
            &tx_mailbox
        );


    /* =====================================
     * 5. 判断发送结果
     * ===================================== */

    if (status != HAL_OK)
    {
        snprintf(
            msg,
            sizeof(msg),
            "CAN TX FAILED: CMD=0x%02X ERR=0x%08lX\r\n",
            command,
            (unsigned long)
            HAL_CAN_GetError(&hcan1)
        );


        HAL_UART_Transmit(
            &huart2,
            (uint8_t *)msg,
            strlen(msg),
            100
        );
    }
    else
    {
        snprintf(
            msg,
            sizeof(msg),
            "CAN TX OK: CMD=0x%02X PARAM=%u MAILBOX=%lu\r\n",
            command,
            parameter,
            (unsigned long)tx_mailbox
        );


        HAL_UART_Transmit(
            &huart2,
            (uint8_t *)msg,
            strlen(msg),
            100
        );
    }
}


/* =========================================================
 * CAN START
 * ========================================================= */

void CAN1_SendStart(void)
{
    CAN1_SendCommand(
        0x01,
        0
    );
}


/* =========================================================
 * CAN STOP
 * ========================================================= */

void CAN1_SendStop(void)
{
    CAN1_SendCommand(
        0x02,
        0
    );
}


/* =========================================================
 * CAN SET SPEED
 * ========================================================= */

void CAN1_SendSetSpeed(
    float speed
)
{
    uint16_t speed_value;


    if (speed < 0.0f)
        speed = 0.0f;


    if (speed > 300.0f)
        speed = 300.0f;


    speed_value =
        (uint16_t)(speed * 10.0f);


    CAN1_SendCommand(
        0x03,
        speed_value
    );
}


/* =========================================================
 * CAN CLEAR
 * ========================================================= */

void CAN1_SendClear(void)
{
    CAN1_SendCommand(
        0x04,
        0
    );
}


/* =========================================================
 * CAN SAVE
 * ========================================================= */

void CAN1_SendSave(void)
{
    CAN1_SendCommand(
        0x05,
        0
    );
}


/* =========================================================
 * CAN Send Motor Status
 * ========================================================= */

void CAN1_SendMotorStatus(void)
{
    CAN_TxHeaderTypeDef tx_header;

    uint8_t tx_data[8];

    uint32_t tx_mailbox;

    uint16_t actual_rpm_can;

    uint16_t target_rpm_can;

    uint16_t current_can;

    uint8_t temperature_can;


    /* =====================================
     * 数据转换
     * ===================================== */

    actual_rpm_can =
        (uint16_t)
        (motor_status.actual_rpm * 10.0f);


    target_rpm_can =
        (uint16_t)
        (motor_status.target_rpm * 10.0f);


    current_can =
        (uint16_t)
        (motor_status.current * 100.0f);


    temperature_can =
        (uint8_t)
        motor_status.temperature;


    /* =====================================
     * Header
     * ===================================== */

    tx_header.StdId =
        CAN_ID_MOTOR_STATUS;

    tx_header.ExtId = 0x00;

    tx_header.IDE =
        CAN_ID_STD;

    tx_header.RTR =
        CAN_RTR_DATA;

    tx_header.DLC = 8;

    tx_header.TransmitGlobalTime =
        DISABLE;


    /* =====================================
     * Data
     * ===================================== */

    tx_data[0] =
        (actual_rpm_can >> 8) & 0xFF;

    tx_data[1] =
        actual_rpm_can & 0xFF;


    tx_data[2] =
        (target_rpm_can >> 8) & 0xFF;

    tx_data[3] =
        target_rpm_can & 0xFF;


    tx_data[4] =
        (current_can >> 8) & 0xFF;

    tx_data[5] =
        current_can & 0xFF;


    tx_data[6] =
        temperature_can;


    tx_data[7] =
        (uint8_t)
        motor_status.fault;


    /* =====================================
     * Send
     * ===================================== */

    if (HAL_CAN_AddTxMessage(
            &hcan1,
            &tx_header,
            tx_data,
            &tx_mailbox
        ) != HAL_OK)
    {
        Error_Handler();
    }
}


/* =========================================================
 * CAN Process Command
 * ========================================================= */

static void CAN1_ProcessCommand(
    uint8_t *rx_data
)
{
    uint16_t speed_value;

    char msg[128];

    HAL_StatusTypeDef flash_status;

    uint32_t flash_error;

    char flash_msg[64];


    switch (rx_data[0])
    {
        /* =====================================
         * START
         * ===================================== */

        case 0x01:

            motor_start_request = 1;


            HAL_UART_Transmit(
                &huart2,
                (uint8_t *)
                "CAN CMD: START\r\n",
                strlen(
                    "CAN CMD: START\r\n"
                ),
                100
            );

            break;


        /* =====================================
         * STOP
         * ===================================== */

        case 0x02:

            motor_stop_request = 1;


            HAL_UART_Transmit(
                &huart2,
                (uint8_t *)
                "CAN CMD: STOP\r\n",
                strlen(
                    "CAN CMD: STOP\r\n"
                ),
                100
            );

            break;


        /* =====================================
         * SET SPEED
         * ===================================== */

        case 0x03:

            speed_value =
                ((uint16_t)rx_data[1] << 8)
                | rx_data[2];


            motor_status.target_rpm =
                speed_value / 10.0f;


            if (motor_status.target_rpm >
                300.0f)
            {
                motor_status.target_rpm =
                    300.0f;
            }


            /*
             * 保留原逻辑：
             * SET_SPEED 后同步更新 Flash 配置
             */
            motor_config.target_rpm =
                motor_status.target_rpm;


            snprintf(
                msg,
                sizeof(msg),
                "CAN CMD: SET_SPEED %.1f RPM\r\n",
                motor_status.target_rpm
            );


            HAL_UART_Transmit(
                &huart2,
                (uint8_t *)msg,
                strlen(msg),
                100
            );

            break;


        /* =====================================
         * CLEAR
         * ===================================== */

        case 0x04:

            uart_clear_request = 1;


            HAL_UART_Transmit(
                &huart2,
                (uint8_t *)
                "CAN CMD: CLEAR\r\n",
                strlen(
                    "CAN CMD: CLEAR\r\n"
                ),
                100
            );

            break;


        /* =====================================
         * SAVE
         * ===================================== */

        case 0x05:

            motor_config.kp =
                20.0f;

            motor_config.ki =
                0.3f;

            motor_config.target_rpm =
                motor_status.target_rpm;


            snprintf(
                msg,
                sizeof(msg),
                "CAN CMD: SAVE TARGET=%.1f RPM\r\n",
                motor_config.target_rpm
            );


            HAL_UART_Transmit(
                &huart2,
                (uint8_t *)msg,
                strlen(msg),
                100
            );


            flash_status =
                Flash_ConfigSave(
                    &motor_config
                );


            if (flash_status == HAL_OK)
            {
                HAL_UART_Transmit(
                    &huart2,
                    (uint8_t *)
                    "CAN CMD: SAVE OK\r\n",
                    strlen(
                        "CAN CMD: SAVE OK\r\n"
                    ),
                    100
                );
            }
            else
            {
                flash_error =
                    HAL_FLASH_GetError();


                snprintf(
                    flash_msg,
                    sizeof(flash_msg),
                    "CAN CMD: SAVE FAILED, ERROR=0x%08lX\r\n",
                    (unsigned long)
                    flash_error
                );


                HAL_UART_Transmit(
                    &huart2,
                    (uint8_t *)flash_msg,
                    strlen(flash_msg),
                    100
                );
            }

            break;


        /* =====================================
         * UNKNOWN
         * ===================================== */

        default:

            snprintf(
                msg,
                sizeof(msg),
                "CAN CMD: UNKNOWN 0x%02X\r\n",
                rx_data[0]
            );


            HAL_UART_Transmit(
                &huart2,
                (uint8_t *)msg,
                strlen(msg),
                100
            );

            break;
    }
}


/* =========================================================
 * CAN RX Callback
 * ========================================================= */

void HAL_CAN_RxFifo0MsgPendingCallback(
    CAN_HandleTypeDef *hcan
)
{
    CAN_RxHeaderTypeDef rx_header;

    uint8_t rx_data[8];


    if (hcan->Instance == CAN1)
    {
        if (HAL_CAN_GetRxMessage(
                hcan,
                CAN_RX_FIFO0,
                &rx_header,
                rx_data
            ) == HAL_OK)
        {
            can_rx_count++;


            CAN1_RX_QueuePush(
                &rx_header,
                rx_data
            );
        }
    }
}


/* =========================================================
 * CAN RX Process
 * ========================================================= */

void CAN1_ProcessRx(void)
{
    CAN_RxHeaderTypeDef rx_header;

    uint8_t rx_data[8];


    /*
     * 一次处理软件 FIFO 中的所有 CAN 帧
     */

    while (CAN1_RX_QueuePop(
        &rx_header,
        rx_data
    ))
    {
        /*
         * 只处理电机命令
         */

        if (rx_header.StdId ==
            CAN_ID_MOTOR_CMD)
        {
            CAN1_ProcessCommand(
                rx_data
            );
        }
    }
}
