#include "flash_config.h"
#include <string.h>
#include <stdio.h>
#include "usart.h"

/* =========================================================
 * 默认参数
 * ========================================================= */
#define DEFAULT_KP          20.0f
#define DEFAULT_KI          0.3f
#define DEFAULT_TARGET_RPM  80.0f


/* =========================================================
 * Flash 参数地址
 *
 * STM32F407VGT6
 * Flash = 1 MB
 *
 * Sector 11:
 * 0x080E0000 ~ 0x080FFFFF
 *
 * 用最后一个 Sector 保存参数
 * ========================================================= */
#define FLASH_CONFIG_ADDRESS    0x080E0000U


/* =========================================================
 * CRC32
 * ========================================================= */
uint32_t Flash_CalculateCRC(MotorConfig_t *config)
{
    uint32_t crc = 0xFFFFFFFFU;

    uint8_t *data = (uint8_t *)config;

    /*
     * MotorConfig_t:
     *
     * kp          4 bytes
     * ki          4 bytes
     * target_rpm  4 bytes
     * crc         4 bytes
     *
     * CRC计算时不计算最后4字节的crc本身
     */
    for (uint32_t i = 0;
         i < sizeof(MotorConfig_t) - sizeof(uint32_t);
         i++)
    {
        crc ^= data[i];

        for (uint8_t j = 0; j < 8; j++)
        {
            if (crc & 1U)
            {
                crc =
                    (crc >> 1)
                    ^ 0xEDB88320U;
            }
            else
            {
                crc >>= 1;
            }
        }
    }

    return crc;
}


/* =========================================================
 * Flash 参数读取
 * ========================================================= */
void Flash_ConfigLoad(MotorConfig_t *config)
{
    MotorConfig_t *flash_config =
        (MotorConfig_t *)FLASH_CONFIG_ADDRESS;

    uint32_t calculated_crc;


    /* 从Flash读取 */
    config->kp =
        flash_config->kp;

    config->ki =
        flash_config->ki;

    config->target_rpm =
        flash_config->target_rpm;

    config->crc =
        flash_config->crc;


    /* 计算CRC */
    calculated_crc =
        Flash_CalculateCRC(config);


    /* =====================================================
     * CRC正确
     * ===================================================== */
    if (calculated_crc == config->crc)
    {
        return;
    }


    /* =====================================================
     * CRC错误
     *
     * Flash中没有有效参数
     * 使用默认参数
     * ===================================================== */

    config->kp =
        DEFAULT_KP;

    config->ki =
        DEFAULT_KI;

    config->target_rpm =
        DEFAULT_TARGET_RPM;

    config->crc =
        Flash_CalculateCRC(config);
}


/* =========================================================
 * Flash 参数保存
 * ========================================================= */
HAL_StatusTypeDef Flash_ConfigSave(MotorConfig_t *config)
{
    HAL_StatusTypeDef status;

    uint32_t address =
        FLASH_CONFIG_ADDRESS;

    uint32_t sector_error = 0;

    FLASH_EraseInitTypeDef erase_init;

			uint32_t data1;
			uint32_t data2;
			uint32_t data3;
			uint32_t data4;

    /* =====================================================
     * 重新计算CRC
     * ===================================================== */

    config->crc =
        Flash_CalculateCRC(config);


    /* =====================================================
     * Flash解锁
     * ===================================================== */

    HAL_FLASH_Unlock();


    /* =====================================================
     * 清除Flash错误标志
     * ===================================================== */

    __HAL_FLASH_CLEAR_FLAG(
        FLASH_FLAG_EOP |
        FLASH_FLAG_OPERR |
        FLASH_FLAG_WRPERR |
        FLASH_FLAG_PGAERR |
        FLASH_FLAG_PGPERR |
        FLASH_FLAG_PGSERR
    );


    /* =====================================================
     * Sector 11 擦除
     * ===================================================== */

    erase_init.TypeErase =
        FLASH_TYPEERASE_SECTORS;

    erase_init.Sector =
        FLASH_SECTOR_11;

    erase_init.NbSectors =
        1;

    erase_init.VoltageRange =
        FLASH_VOLTAGE_RANGE_3;


    status =
        HAL_FLASHEx_Erase(
            &erase_init,
            &sector_error
        );


    /* =====================================================
     * 擦除失败
     * ===================================================== */

    if (status != HAL_OK)
    {
        uint32_t error_code;

        char msg[100];


        error_code =
            HAL_FLASH_GetError();


        snprintf(
            msg,
            sizeof(msg),
            "FLASH ERASE FAILED: ERROR=0x%08lX SECTOR=0x%08lX\r\n",
            (unsigned long)error_code,
            (unsigned long)sector_error
        );


        HAL_UART_Transmit(
            &huart2,
            (uint8_t *)msg,
            strlen(msg),
            100
        );


        HAL_FLASH_Lock();

        return HAL_ERROR;
    }


    /* =====================================================
     * 准备16字节参数
     *
     * 前8字节:
     *
     * kp
     * ki
     *
     * 后8字节:
     *
     * target_rpm
     * crc
     * ===================================================== */

/* =====================================================
 * 准备16字节参数
 *
 * 使用 32-bit WORD 编程
 *
 * data1 = kp
 * data2 = ki
 * data3 = target_rpm
 * data4 = crc
 * ===================================================== */

memcpy(
    &data1,
    ((uint8_t *)config) + 0,
    4
);

memcpy(
    &data2,
    ((uint8_t *)config) + 4,
    4
);

memcpy(
    &data3,
    ((uint8_t *)config) + 8,
    4
);

memcpy(
    &data4,
    ((uint8_t *)config) + 12,
    4
);


/* =====================================================
 * 写入第1个 WORD
 * ===================================================== */

status =
    HAL_FLASH_Program(
        FLASH_TYPEPROGRAM_WORD,
        address,
        data1
    );

if (status != HAL_OK)
{
    uint32_t error_code;
    char msg[100];

    error_code =
        HAL_FLASH_GetError();

    snprintf(
        msg,
        sizeof(msg),
        "FLASH WRITE1 FAILED: ERROR=0x%08lX\r\n",
        (unsigned long)error_code
    );

    HAL_UART_Transmit(
        &huart2,
        (uint8_t *)msg,
        strlen(msg),
        100
    );

    HAL_FLASH_Lock();

    return HAL_ERROR;
}


/* =====================================================
 * 写入第2个 WORD
 * ===================================================== */

status =
    HAL_FLASH_Program(
        FLASH_TYPEPROGRAM_WORD,
        address + 4,
        data2
    );

if (status != HAL_OK)
{
    uint32_t error_code;
    char msg[100];

    error_code =
        HAL_FLASH_GetError();

    snprintf(
        msg,
        sizeof(msg),
        "FLASH WRITE2 FAILED: ERROR=0x%08lX\r\n",
        (unsigned long)error_code
    );

    HAL_UART_Transmit(
        &huart2,
        (uint8_t *)msg,
        strlen(msg),
        100
    );

    HAL_FLASH_Lock();

    return HAL_ERROR;
}


/* =====================================================
 * 写入第3个 WORD
 * ===================================================== */

status =
    HAL_FLASH_Program(
        FLASH_TYPEPROGRAM_WORD,
        address + 8,
        data3
    );

if (status != HAL_OK)
{
    uint32_t error_code;
    char msg[100];

    error_code =
        HAL_FLASH_GetError();

    snprintf(
        msg,
        sizeof(msg),
        "FLASH WRITE3 FAILED: ERROR=0x%08lX\r\n",
        (unsigned long)error_code
    );

    HAL_UART_Transmit(
        &huart2,
        (uint8_t *)msg,
        strlen(msg),
        100
    );

    HAL_FLASH_Lock();

    return HAL_ERROR;
}


/* =====================================================
 * 写入第4个 WORD
 * ===================================================== */

status =
    HAL_FLASH_Program(
        FLASH_TYPEPROGRAM_WORD,
        address + 12,
        data4
    );

if (status != HAL_OK)
{
    uint32_t error_code;
    char msg[100];

    error_code =
        HAL_FLASH_GetError();

    snprintf(
        msg,
        sizeof(msg),
        "FLASH WRITE4 FAILED: ERROR=0x%08lX\r\n",
        (unsigned long)error_code
    );

    HAL_UART_Transmit(
        &huart2,
        (uint8_t *)msg,
        strlen(msg),
        100
    );

    HAL_FLASH_Lock();

    return HAL_ERROR;
}

    /* =====================================================
     * 第二次写入失败
     * ===================================================== */

    if (status != HAL_OK)
    {
        uint32_t error_code;

        char msg[100];


        error_code =
            HAL_FLASH_GetError();


        snprintf(
            msg,
            sizeof(msg),
            "FLASH WRITE2 FAILED: ERROR=0x%08lX\r\n",
            (unsigned long)error_code
        );


        HAL_UART_Transmit(
            &huart2,
            (uint8_t *)msg,
            strlen(msg),
            100
        );


        HAL_FLASH_Lock();

        return HAL_ERROR;
    }


    /* =====================================================
     * Flash重新上锁
     * ===================================================== */

    HAL_FLASH_Lock();


    /* =====================================================
     * 写入完成后进行校验
     * ===================================================== */

    MotorConfig_t verify_config;

    MotorConfig_t *flash_config =
        (MotorConfig_t *)FLASH_CONFIG_ADDRESS;


    verify_config.kp =
        flash_config->kp;

    verify_config.ki =
        flash_config->ki;

    verify_config.target_rpm =
        flash_config->target_rpm;

    verify_config.crc =
        flash_config->crc;


    /* =====================================================
     * CRC校验
     * ===================================================== */

    if (Flash_CalculateCRC(&verify_config)
        != verify_config.crc)
    {
        char msg[100];

        snprintf(
            msg,
            sizeof(msg),
            "FLASH VERIFY FAILED: CRC ERROR\r\n"
        );

        HAL_UART_Transmit(
            &huart2,
            (uint8_t *)msg,
            strlen(msg),
            100
        );

        return HAL_ERROR;
    }


    /* =====================================================
     * Flash保存成功
     * ===================================================== */

    HAL_UART_Transmit(
        &huart2,
        (uint8_t *)"FLASH VERIFY OK\r\n",
        strlen("FLASH VERIFY OK\r\n"),
        100
    );


    return HAL_OK;
}
