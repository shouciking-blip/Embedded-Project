#include "uart_protocol.h"
#include <string.h>

/* ================================
 * 内部状态
 * ================================ */

typedef enum
{
    UART_PROTO_WAIT_HEADER = 0,
    UART_PROTO_WAIT_CMD,
    UART_PROTO_WAIT_LEN,
    UART_PROTO_WAIT_DATA,
    UART_PROTO_WAIT_CRC,
    UART_PROTO_WAIT_TAIL
} UART_ProtocolState_t;

static volatile UART_ProtocolState_t parser_state;

static volatile uint8_t parser_cmd;
static volatile uint8_t parser_len;
static volatile uint8_t parser_data[UART_PROTOCOL_MAX_DATA];
static volatile uint8_t parser_data_index;
static volatile uint8_t parser_crc;
static volatile uint8_t parser_busy = 0;
static volatile uint8_t frame_ready = 0;

/* 软件 FIFO */
static UART_ProtocolFrame_t rx_queue[UART_PROTOCOL_QUEUE_SIZE];

static volatile uint8_t rx_write_index;
static volatile uint8_t rx_read_index;

static volatile uint32_t rx_frame_count;
static volatile uint32_t rx_error_count;
static volatile uint32_t rx_overflow_count;


/* ================================
 * CRC-8
 *
 * Polynomial = 0x07
 * Initial    = 0x00
 * ================================ */

uint8_t UART_Protocol_CRC8(const uint8_t *data, uint16_t len)
{
    uint8_t crc = 0;
    uint16_t i;
    uint8_t bit;

    for (i = 0; i < len; i++)
    {
        crc ^= data[i];

        for (bit = 0; bit < 8; bit++)
        {
            if (crc & 0x80)
            {
                crc = (uint8_t)((crc << 1) ^ 0x07);
            }
            else
            {
                crc <<= 1;
            }
        }
    }

    return crc;
}


/* ================================
 * 初始化
 * ================================ */

void UART_Protocol_Init(void)
{
    parser_state = UART_PROTO_WAIT_HEADER;

    parser_cmd = 0;
    parser_len = 0;
    parser_data_index = 0;
    parser_crc = 0;

    rx_write_index = 0;
    rx_read_index = 0;

    rx_frame_count = 0;
    rx_error_count = 0;
    rx_overflow_count = 0;

    memset((void *)parser_data, 0, sizeof(parser_data));
    memset(rx_queue, 0, sizeof(rx_queue));
}


/* ================================
 * 将完整帧放入 FIFO
 * ================================ */

static void UART_Protocol_PushFrame(void)
{
    uint8_t next_write;

    next_write =
        (uint8_t)((rx_write_index + 1) % UART_PROTOCOL_QUEUE_SIZE);

    /* FIFO 满 */
    if (next_write == rx_read_index)
    {
        rx_overflow_count++;

        /* 丢弃当前帧 */
        parser_state = UART_PROTO_WAIT_HEADER;
        return;
    }

    rx_queue[rx_write_index].cmd = parser_cmd;
    rx_queue[rx_write_index].len = parser_len;

    if (parser_len > 0)
    {
        memcpy(
            rx_queue[rx_write_index].data,
            (const void *)parser_data,
            parser_len
        );
    }

    rx_write_index = next_write;

    rx_frame_count++;

    parser_state = UART_PROTO_WAIT_HEADER;
}


/* ================================
 * 输入一个字节
 * ================================ */

void UART_Protocol_InputByte(uint8_t byte)
{
    uint8_t crc_data[2 + UART_PROTOCOL_MAX_DATA];
    uint16_t crc_len;
    uint8_t calculated_crc;

    switch (parser_state)
    {
        /* 等待 AA */
        case UART_PROTO_WAIT_HEADER:

            if (byte == UART_PROTOCOL_HEADER)
            {
                parser_state = UART_PROTO_WAIT_CMD;
                parser_data_index = 0;
            }

            break;


        /* CMD */
        case UART_PROTO_WAIT_CMD:

            parser_cmd = byte;
            parser_state = UART_PROTO_WAIT_LEN;

            break;


        /* LEN */
        case UART_PROTO_WAIT_LEN:

            parser_len = byte;

            if (parser_len > UART_PROTOCOL_MAX_DATA)
            {
                rx_error_count++;

                parser_state = UART_PROTO_WAIT_HEADER;
                parser_len = 0;
            }
            else if (parser_len == 0)
            {
                parser_state = UART_PROTO_WAIT_CRC;
            }
            else
            {
                parser_data_index = 0;
                parser_state = UART_PROTO_WAIT_DATA;
            }

            break;


        /* DATA */
        case UART_PROTO_WAIT_DATA:

            parser_data[parser_data_index] = byte;
            parser_data_index++;

            if (parser_data_index >= parser_len)
            {
                parser_state = UART_PROTO_WAIT_CRC;
            }

            break;


        /* CRC */
        case UART_PROTO_WAIT_CRC:

            parser_crc = byte;

            /*
             * CRC 计算：
             * CMD + LEN + DATA
             */

            crc_data[0] = parser_cmd;
            crc_data[1] = parser_len;

            if (parser_len > 0)
            {
                memcpy(
                    &crc_data[2],
                    (const void *)parser_data,
                    parser_len
                );
            }

            crc_len = (uint16_t)(2 + parser_len);

            calculated_crc =
                UART_Protocol_CRC8(crc_data, crc_len);

            if (calculated_crc == parser_crc)
            {
                parser_state = UART_PROTO_WAIT_TAIL;
            }
            else
            {
                rx_error_count++;
                parser_state = UART_PROTO_WAIT_HEADER;
            }

            break;


        /* 55 */
        case UART_PROTO_WAIT_TAIL:

            if (byte == UART_PROTOCOL_TAIL)
            {
                UART_Protocol_PushFrame();
            }
            else
            {
                rx_error_count++;

                /*
                 * 当前字节如果刚好是 AA，
                 * 可以直接认为它是下一帧开始
                 */
                if (byte == UART_PROTOCOL_HEADER)
                {
                    parser_state = UART_PROTO_WAIT_CMD;
                }
                else
                {
                    parser_state = UART_PROTO_WAIT_HEADER;
                }
            }

            break;


        default:

            parser_state = UART_PROTO_WAIT_HEADER;

            break;
    }
}


/* ================================
 * 获取完整帧
 * ================================ */

uint8_t UART_Protocol_GetFrame(UART_ProtocolFrame_t *frame)
{
    uint8_t read_index;

    if (frame == NULL)
    {
        return 0;
    }

    if (rx_read_index == rx_write_index)
    {
        return 0;
    }

    read_index = rx_read_index;

    frame->cmd = rx_queue[read_index].cmd;
    frame->len = rx_queue[read_index].len;

    if (frame->len > 0)
    {
        memcpy(
            frame->data,
            rx_queue[read_index].data,
            frame->len
        );
    }

    rx_read_index =
        (uint8_t)((rx_read_index + 1) % UART_PROTOCOL_QUEUE_SIZE);

    return 1;
}


/* ================================
 * 统计信息
 * ================================ */

uint32_t UART_Protocol_GetRxFrameCount(void)
{
    return rx_frame_count;
}


uint32_t UART_Protocol_GetRxErrorCount(void)
{
    return rx_error_count;
}


uint32_t UART_Protocol_GetRxOverflowCount(void)
{
    return rx_overflow_count;
}
