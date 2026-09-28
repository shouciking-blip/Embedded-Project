#include "motor_status.h"


MotorStatus_t motor_status =
{
    .actual_rpm = 0.0f,
    .target_rpm = 80.0f,

    .motor_running = 0,

    .pwm = 0,

    .current = 0.0f,
    .voltage = 0.0f,
    .temperature = 0.0f,

    .fault = 0
};
