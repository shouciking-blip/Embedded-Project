#include "fault_manager.h"
#include "motor_status.h"
#include "motor_control.h"

#include <math.h>
/* =========================
 * 故障参数
 * ========================= */

#define TEMP_OVER_THRESHOLD        70.0f

#define CURRENT_OVER_THRESHOLD     0.15f

#define VOLTAGE_UNDER_THRESHOLD    6.5f
#define VOLTAGE_OVER_THRESHOLD     8.0f

#define STALL_PWM_THRESHOLD        50.0f
#define STALL_RPM_THRESHOLD        20.0f
#define STALL_TIME_COUNT           5

#define STARTUP_GRACE_TIME         1500U

#define FAULT_CONFIRM_COUNT        3
#define FAULT_MANAGER_STARTUP_DELAY 1000U

#define SPEED_ERROR_THRESHOLD       30.0f
#define SPEED_FAULT_CONFIRM_COUNT   3
#define SPEED_RECOVERY_DELAY        3000U
#define MAX_RECOVERY_COUNT          3

/* =========================
 * 内部变量
 * ========================= */

static uint8_t temperature_fault_count = 0;
static uint8_t overcurrent_fault_count = 0;

static uint8_t stall_count = 0;

static uint32_t fault_start_tick = 0;
static uint32_t fault_manager_start_tick = 0;

static uint8_t speed_fault_count = 0;
static uint8_t recovery_count = 0;
static uint32_t recovery_start_tick = 0;
static uint8_t recovery_waiting = 0;
static uint32_t speed_monitor_start_tick = 0;
static uint8_t speed_monitor_started = 0;
/* =========================
 * 初始化
 * ========================= */

void FaultManager_Init(void)
{
		speed_monitor_start_tick = 0;
		speed_monitor_started = 0;
    temperature_fault_count = 0;
    overcurrent_fault_count = 0;
    stall_count = 0;

    fault_start_tick = 0;

    fault_manager_start_tick = HAL_GetTick();

    motor_status.fault = FAULT_NONE;
		speed_fault_count = 0;
		recovery_count = 0;
		recovery_start_tick = 0;
		recovery_waiting = 0;
}


/* =========================
 * 故障状态字符串
 * ========================= */

const char* FaultManager_GetStateString(FaultState_t state)
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

        case FAULT_UNDERVOLTAGE:
            return "UNDERVOLTAGE";

        case FAULT_OVERVOLTAGE:
            return "OVERVOLTAGE";

        case FAULT_STALL:
            return "STALL";

        case FAULT_LOCKED:
            return "LOCKED";

        default:
            return "UNKNOWN";
    }
}


/* =========================
 * 故障更新
 * ========================= */

void FaultManager_Update(void)
{
	    /* =========================
     * 系统启动保护
     *
     * 等待 ADC / DMA / SensorTask
     * 完成初始数据采集后再进行故障判断
     * ========================= */

    if ((HAL_GetTick() - fault_manager_start_tick)
        < FAULT_MANAGER_STARTUP_DELAY)
    {
        return;
    }
		    /* =========================
     * 速度故障自动恢复
     * ========================= */

    if (motor_status.fault == FAULT_SPEED &&
        recovery_waiting)
    {
        if ((HAL_GetTick() - recovery_start_tick)
            >= SPEED_RECOVERY_DELAY)
        {
            recovery_waiting = 0;

            if (recovery_count < MAX_RECOVERY_COUNT)
            {
                recovery_count++;

                motor_status.fault = FAULT_NONE;
                motor_status.motor_running = 1;
            }
            else
            {
                motor_status.fault = FAULT_LOCKED;
                motor_status.motor_running = 0;
                motor_status.pwm = 0;

                MotorControl_Stop();
            }
        }

        return;
    }
		
    float rpm;
    float target;
    float current;
    float voltage;
    float temperature;
    uint16_t pwm;

    rpm = motor_status.actual_rpm;
    target = motor_status.target_rpm;

    current = motor_status.current;
    voltage = motor_status.voltage;
    temperature = motor_status.temperature;

    pwm = motor_status.pwm;


    /* 已经锁定 */
    if (motor_status.fault == FAULT_LOCKED)
    {
        return;
    }


/* =========================
 * 温度故障
 * ========================= */



    if (temperature >= TEMP_OVER_THRESHOLD)
    {
        if (temperature_fault_count < FAULT_CONFIRM_COUNT)
            temperature_fault_count++;

        if (temperature_fault_count >= FAULT_CONFIRM_COUNT)
        {
            motor_status.fault = FAULT_TEMP;

            motor_status.motor_running = 0;
            motor_status.pwm = 0;

            MotorControl_Stop();

            return;
        }
    }
    else
    {
        temperature_fault_count = 0;
    }




/* =========================
 * 电压保护
 * ========================= */



    if (motor_status.voltage < VOLTAGE_UNDER_THRESHOLD)
    {
        motor_status.fault = FAULT_UNDERVOLTAGE;

        motor_status.motor_running = 0;
        motor_status.pwm = 0;

        MotorControl_Stop();

        return;
    }

    if (motor_status.voltage > VOLTAGE_OVER_THRESHOLD)
    {
        motor_status.fault = FAULT_OVERVOLTAGE;

        motor_status.motor_running = 0;
        motor_status.pwm = 0;

        MotorControl_Stop();

        return;
    }


/* =========================
 * 过流
 * ========================= */


    if (current >= CURRENT_OVER_THRESHOLD)
    {
        if (overcurrent_fault_count < FAULT_CONFIRM_COUNT)
            overcurrent_fault_count++;

        if (overcurrent_fault_count >= FAULT_CONFIRM_COUNT)
        {
            motor_status.fault = FAULT_OVERCURRENT;

            motor_status.motor_running = 0;
            motor_status.pwm = 0;

            MotorControl_Stop();

            return;
        }
    }
    else
    {
        overcurrent_fault_count = 0;
    }


/* =========================
 * 堵转检测
 * ========================= */


    if (motor_status.motor_running)
    {
        if (target >= 50.0f &&
            pwm >= 2100 &&
            rpm < STALL_RPM_THRESHOLD)
        {
            if (stall_count < STALL_TIME_COUNT)
                stall_count++;

            if (stall_count >= STALL_TIME_COUNT)
            {
                motor_status.fault = FAULT_STALL;

                motor_status.motor_running = 0;
                motor_status.pwm = 0;

                MotorControl_Stop();

                return;
            }
        }
        else
        {
            stall_count = 0;
        }
    }
    else
    {
        stall_count = 0;
    }


		
		
		
/* =========================
 * 速度异常检测
 * ========================= */

if (motor_status.motor_running)
{
    if (!speed_monitor_started)
    {
        speed_monitor_started = 1;
        speed_monitor_start_tick = HAL_GetTick();
        speed_fault_count = 0;
    }

    /*
     * 电机启动后的宽限时间
     * 给电机留出加速到目标转速的时间
     */
    if ((HAL_GetTick() - speed_monitor_start_tick)
        >= STARTUP_GRACE_TIME)
    {
        if (target >= 50.0f &&
            fabsf(target - rpm) >= SPEED_ERROR_THRESHOLD)
        {
            if (speed_fault_count < SPEED_FAULT_CONFIRM_COUNT)
                speed_fault_count++;

            if (speed_fault_count >= SPEED_FAULT_CONFIRM_COUNT)
            {
                motor_status.fault = FAULT_SPEED;

                motor_status.motor_running = 0;
                motor_status.pwm = 0;

                MotorControl_Stop();

                recovery_start_tick = HAL_GetTick();
                recovery_waiting = 1;

                speed_fault_count = 0;

                return;
            }
        }
        else
        {
            speed_fault_count = 0;
        }
    }
}
else
{
    speed_fault_count = 0;
    speed_monitor_started = 0;
}

}



/* =========================
 * 清除故障
 * ========================= */

uint8_t FaultManager_Clear(void)
{
    /*
     * LOCKED 状态允许人工 CLEAR
     */

    motor_status.fault = FAULT_NONE;

    temperature_fault_count = 0;
    overcurrent_fault_count = 0;
    stall_count = 0;
    speed_fault_count = 0;

    recovery_count = 0;
    recovery_start_tick = 0;
    recovery_waiting = 0;

    motor_status.motor_running = 0;
    motor_status.pwm = 0;

    MotorControl_Stop();

    return 1;
}


/* =========================
 * 获取故障状态
 * ========================= */

FaultState_t FaultManager_GetState(void)
{
    return (FaultState_t)motor_status.fault;
}
