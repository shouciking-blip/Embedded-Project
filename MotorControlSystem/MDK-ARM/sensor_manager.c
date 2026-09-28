#include "sensor_manager.h"

#include "adc.h"
#include "motor_status.h"

#include <math.h>


/* =========================================================
 * SensorManager 参数
 * ========================================================= */


/* ADC DMA */
#define SENSOR_ADC_CHANNEL_COUNT       3
#define SENSOR_ADC_SAMPLE_COUNT        32
#define SENSOR_ADC_BUFFER_SIZE         \
    (SENSOR_ADC_CHANNEL_COUNT * SENSOR_ADC_SAMPLE_COUNT)


/* ADC参考电压 */
#define SENSOR_ADC_VREF                3.3f


/* INA180 */
#define SENSOR_SHUNT_RESISTANCE        0.01f
#define SENSOR_CURRENT_GAIN            50.0f


/* 电压分压 */
#define SENSOR_VOLTAGE_R_RATIO         5.7f
#define SENSOR_VOLTAGE_CALIBRATION     0.887f


/* 电流滤波 */
#define SENSOR_CURRENT_FILTER_ALPHA    0.2f


/* 温度滤波 */
#define SENSOR_TEMP_FILTER_ALPHA       0.2f


/* NTC */
#define SENSOR_NTC_R25                 10000.0f
#define SENSOR_NTC_BETA                3200.0f
#define SENSOR_NTC_T25                 298.15f


/* 温度状态 */
#define SENSOR_TEMP_WARNING_THRESHOLD  60.0f
#define SENSOR_TEMP_OVER_THRESHOLD     70.0f


/* =========================================================
 * 内部变量
 * ========================================================= */


/* ADC DMA缓冲区 */
static uint16_t adc_buffer[
    SENSOR_ADC_BUFFER_SIZE
];


/* ADC原始值 */
static uint16_t adc0 = 0;
static uint16_t adc1 = 0;
static uint16_t adc2 = 0;


/* ADC电压 */
static float adc0_voltage = 0.0f;
static float adc1_voltage = 0.0f;
static float adc2_voltage = 0.0f;


/* 电流 */
static float current_a = 0.0f;


/* 电压 */
static float bus_voltage = 0.0f;


/* NTC */
static float ntc_resistance = 0.0f;


/* 温度 */
static float temperature_c = 0.0f;


/* 温度状态 */
static TemperatureState_t temperature_state =
    TEMP_NORMAL;


/* =========================================================
 * 内部滤波状态
 * ========================================================= */

static uint8_t voltage_filter_initialized = 0;
static uint8_t current_filter_initialized = 0;
static uint8_t temp_filter_initialized = 0;


/* =========================================================
 * SensorManager_Init
 * ========================================================= */

void SensorManager_Init(void)
{
    voltage_filter_initialized = 0;
    current_filter_initialized = 0;
    temp_filter_initialized = 0;

    temperature_state = TEMP_NORMAL;

    adc0 = 0;
    adc1 = 0;
    adc2 = 0;

    adc0_voltage = 0.0f;
    adc1_voltage = 0.0f;
    adc2_voltage = 0.0f;

    current_a = 0.0f;
    bus_voltage = 0.0f;

    ntc_resistance = 0.0f;
    temperature_c = 0.0f;

    /*
     * =====================================================
     * ADC1 + DMA启动
     *
     * Rank 1 -> PA0 -> INA180电流
     * Rank 2 -> PA1 -> 电压
     * Rank 3 -> PA4 -> NTC
     *
     * 每组：
     * [PA0, PA1, PA4]
     *
     * 共32组
     * 3 × 32 = 96
     * =====================================================
     */

    if (HAL_ADC_Start_DMA(
            &hadc1,
            (uint32_t *)adc_buffer,
            SENSOR_ADC_BUFFER_SIZE
        ) != HAL_OK)
    {
        Error_Handler();
    }
}


/* =========================================================
 * SensorManager_Update
 * ========================================================= */

void SensorManager_Update(void)
{
    uint32_t sum_adc0 = 0;
    uint32_t sum_adc1 = 0;
    uint32_t sum_adc2 = 0;

    uint8_t i;


    /*
     * =====================================================
     * 1. 读取32组ADC数据
     * =====================================================
     */

    for (i = 0;
         i < SENSOR_ADC_SAMPLE_COUNT;
         i++)
    {
        sum_adc0 += adc_buffer[i * 3];

        sum_adc1 += adc_buffer[i * 3 + 1];

        sum_adc2 += adc_buffer[i * 3 + 2];
    }


    /*
     * =====================================================
     * 2. 计算平均值
     * =====================================================
     */

    adc0 =
        (uint16_t)(
            sum_adc0 /
            SENSOR_ADC_SAMPLE_COUNT
        );

    adc1 =
        (uint16_t)(
            sum_adc1 /
            SENSOR_ADC_SAMPLE_COUNT
        );

    adc2 =
        (uint16_t)(
            sum_adc2 /
            SENSOR_ADC_SAMPLE_COUNT
        );


    /*
     * =====================================================
     * 3. ADC -> 电压
     * =====================================================
     */

    adc0_voltage =
        (float)adc0 *
        SENSOR_ADC_VREF /
        4095.0f;

    adc1_voltage =
        (float)adc1 *
        3.21f /
        4095.0f;

    adc2_voltage =
        (float)adc2 *
        SENSOR_ADC_VREF /
        4095.0f;


    /*
     * =====================================================
     * 4. 电机母线电压
     *
     * 保留你当前已经验证成功的公式
     * =====================================================
     */

    bus_voltage =
        adc1_voltage *
        SENSOR_VOLTAGE_R_RATIO *
        SENSOR_VOLTAGE_CALIBRATION;


    /*
     * =====================================================
     * 5. 电压低通滤波
     * =====================================================
     */

    if (!voltage_filter_initialized)
    {
        motor_status.voltage =
            bus_voltage;

        voltage_filter_initialized = 1;
    }
    else
    {
        motor_status.voltage =
            0.2f * bus_voltage
            +
            0.8f *
            motor_status.voltage;
    }


    /*
     * =====================================================
     * 6. INA180电流计算
     *
     * I = Vout / (Rshunt × Gain)
     *
     * = Vout / (0.01 × 50)
     * =====================================================
     */

    current_a =
        adc0_voltage /
        (
            SENSOR_SHUNT_RESISTANCE *
            SENSOR_CURRENT_GAIN
        );


    /*
     * =====================================================
     * 7. 电流滤波
     * =====================================================
     */

    if (!current_filter_initialized)
    {
        motor_status.current =
            current_a;

        current_filter_initialized = 1;
    }
    else
    {
        motor_status.current =
            SENSOR_CURRENT_FILTER_ALPHA *
            current_a
            +
            (1.0f -
             SENSOR_CURRENT_FILTER_ALPHA) *
            motor_status.current;
    }


    /*
     * =====================================================
     * 8. NTC电阻计算
     *
     * Rntc =
     * 10000 × ADC / (4095 - ADC)
     * =====================================================
     */

    if (adc2 < 4090)
    {
        ntc_resistance =
            SENSOR_NTC_R25 *
            (float)adc2 /
            (
                4095.0f -
                (float)adc2
            );
    }
    else
    {
        ntc_resistance = 0.0f;
    }


    /*
     * =====================================================
     * 9. NTC温度计算
     *
     * Beta公式
     * =====================================================
     */

    if (ntc_resistance > 0.0f)
    {
        temperature_c =
            1.0f /
            (
                1.0f /
                SENSOR_NTC_T25
                +
                (1.0f /
                 SENSOR_NTC_BETA)
                *
                logf(
                    ntc_resistance /
                    SENSOR_NTC_R25
                )
            );

        temperature_c =
            temperature_c -
            273.15f;
    }
    else
    {
        temperature_c = 0.0f;
    }


    /*
     * =====================================================
     * 10. 温度低通滤波
     * =====================================================
     */

    if (!temp_filter_initialized)
    {
        motor_status.temperature =
            temperature_c;

        temp_filter_initialized = 1;
    }
    else
    {
        motor_status.temperature =
            SENSOR_TEMP_FILTER_ALPHA *
            temperature_c
            +
            (1.0f -
             SENSOR_TEMP_FILTER_ALPHA) *
            motor_status.temperature;
    }


    /*
     * =====================================================
     * 11. 温度状态
     * =====================================================
     */

    if (motor_status.temperature >=
        SENSOR_TEMP_OVER_THRESHOLD)
    {
        temperature_state =
            TEMP_OVER;
    }
    else if (motor_status.temperature >=
             SENSOR_TEMP_WARNING_THRESHOLD)
    {
        temperature_state =
            TEMP_WARNING;
    }
    else
    {
        temperature_state =
            TEMP_NORMAL;
    }
}


/* =========================================================
 * 获取温度状态
 * ========================================================= */

TemperatureState_t
SensorManager_GetTemperatureState(void)
{
    return temperature_state;
}


/* =========================================================
 * 获取ADC原始值
 * ========================================================= */

uint16_t SensorManager_GetADC0(void)
{
    return adc0;
}


uint16_t SensorManager_GetADC1(void)
{
    return adc1;
}


uint16_t SensorManager_GetADC2(void)
{
    return adc2;
}


/* =========================================================
 * 获取传感器计算值
 * ========================================================= */

float SensorManager_GetCurrentRaw(void)
{
    return current_a;
}


float SensorManager_GetBusVoltage(void)
{
    return bus_voltage;
}


float SensorManager_GetTemperatureRaw(void)
{
    return temperature_c;
}


float SensorManager_GetNTCResistance(void)
{
    return ntc_resistance;
}


/* =========================================================
 * 获取ADC电压
 * ========================================================= */

float SensorManager_GetADC0Voltage(void)
{
    return adc0_voltage;
}


float SensorManager_GetADC1Voltage(void)
{
    return adc1_voltage;
}


float SensorManager_GetADC2Voltage(void)
{
    return adc2_voltage;
}
