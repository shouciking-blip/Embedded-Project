#include "encoder_manager.h"
#include "tim.h"
#include "motor_status.h"

/* =========================================================
 * 编码器参数
 * ========================================================= */

/* MG310：
 * 13 PPR × 4倍频 × 20减速比
 * = 1040 counts / output shaft revolution
 */
#define ENCODER_CPR             1040.0f

/* 编码器测速周期：100ms */
#define ENCODER_SAMPLE_TIME     0.1f

/* 一阶低通滤波参数 */
#define ENCODER_FILTER_OLD      0.7f
#define ENCODER_FILTER_NEW      0.3f


/* =========================================================
 * 内部变量
 * ========================================================= */

static uint16_t last_count = 0;
static uint16_t current_count = 0;

static int16_t delta_count = 0;

static float rpm_raw = 0.0f;
static float rpm_filtered = 0.0f;

static uint8_t filter_initialized = 0;


/* =========================================================
 * 初始化
 * ========================================================= */

void EncoderManager_Init(void)
{
    /*
     * 读取当前编码器计数器值，
     * 防止刚启动时产生一个巨大的假 ΔCNT。
     */
    last_count =
        __HAL_TIM_GET_COUNTER(&htim3);

    current_count = last_count;

    delta_count = 0;

    rpm_raw = 0.0f;
    rpm_filtered = 0.0f;

    filter_initialized = 0;

    motor_status.actual_rpm = 0.0f;
}


/* =========================================================
 * 编码器数据更新
 * ========================================================= */

void EncoderManager_Update(void)
{
    /* -----------------------------------------------------
     * 1. 读取 TIM3 当前计数值
     * ----------------------------------------------------- */

    current_count =
        __HAL_TIM_GET_COUNTER(&htim3);


    /* -----------------------------------------------------
     * 2. 计算计数增量
     *
     * 使用 int16_t 自动处理 16 位计数器溢出：
     *
     * 65530 → 5
     *
     * current - last = -65525
     * 转换为 int16_t 后得到正确的小负数。
     * ----------------------------------------------------- */

    delta_count =
        (int16_t)(
            current_count -
            last_count
        );


    /* -----------------------------------------------------
     * 3. 保存当前计数值
     * ----------------------------------------------------- */

    last_count =
        current_count;


    /* -----------------------------------------------------
     * 4. RPM计算
     *
     * RPM =
     * ΔCNT × 60
     * ─────────────
     * CPR × Ts
     *
     * = ΔCNT × 60 / 1040 / 0.1
     * ----------------------------------------------------- */

    rpm_raw =
        (float)delta_count
        * 60.0f
        / ENCODER_CPR
        / ENCODER_SAMPLE_TIME;


    /* -----------------------------------------------------
     * 5. 一阶低通滤波
     * ----------------------------------------------------- */

    if (filter_initialized == 0)
    {
        rpm_filtered = rpm_raw;

        filter_initialized = 1;
    }
    else
    {
        rpm_filtered =
            ENCODER_FILTER_OLD * rpm_filtered
            +
            ENCODER_FILTER_NEW * rpm_raw;
    }


    /* -----------------------------------------------------
     * 6. 更新全局电机状态
     * ----------------------------------------------------- */

    motor_status.actual_rpm =
        rpm_filtered;
}


/* =========================================================
 * 获取滤波后的 RPM
 * ========================================================= */

float EncoderManager_GetRPM(void)
{
    return rpm_filtered;
}


/* =========================================================
 * 获取原始 RPM
 * ========================================================= */

float EncoderManager_GetRawRPM(void)
{
    return rpm_raw;
}


/* =========================================================
 * 获取 ΔCNT
 * ========================================================= */

int16_t EncoderManager_GetDeltaCount(void)
{
    return delta_count;
}


/* =========================================================
 * 获取当前 CNT
 * ========================================================= */

uint16_t EncoderManager_GetCount(void)
{
    return current_count;
}
