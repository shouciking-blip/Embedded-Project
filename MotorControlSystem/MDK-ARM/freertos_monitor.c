#include "freertos_monitor.h"
#include "uart_manager.h"
#include <stdio.h>
#include <string.h>
#define FREERTOS_MONITOR_MAX_TASKS 16


void FreeRTOS_Monitor_PrintTasks(void)
{
    char buffer[1024];

    TaskStatus_t task_status[10];
    UBaseType_t task_count;
    UBaseType_t i;

    memset(task_status, 0, sizeof(task_status));

    task_count = uxTaskGetSystemState(
    task_status,
    FREERTOS_MONITOR_MAX_TASKS,
    NULL
);

    snprintf(buffer, sizeof(buffer),
             "\r\n"
             "===== FREERTOS TASK MONITOR =====\r\n"
             "Name                 State  Priority  StackFree\r\n"
             "-------------------------------------------------\r\n");

    UART_Manager_SendString(buffer);

    for (i = 0; i < task_count; i++)
    {
        const char *state;

        switch (task_status[i].eCurrentState)
        {
            case eRunning:
                state = "RUN";
                break;

            case eReady:
                state = "READY";
                break;

            case eBlocked:
                state = "BLOCK";
                break;

            case eSuspended:
                state = "SUSP";
                break;

            case eDeleted:
                state = "DEL";
                break;

            default:
                state = "?";
                break;
        }

        snprintf(buffer, sizeof(buffer),
                 "%-20s %-5s %-9lu %lu\r\n",
                 task_status[i].pcTaskName,
                 state,
                 (unsigned long)task_status[i].uxCurrentPriority,
                 (unsigned long)task_status[i].usStackHighWaterMark);

        UART_Manager_SendString(buffer);
    }

    UART_Manager_SendString(
        "=================================\r\n"
    );
}

void FreeRTOS_Monitor_PrintCPU(void)
{
		TaskStatus_t task_status[FREERTOS_MONITOR_MAX_TASKS];
		static TaskHandle_t last_handles[FREERTOS_MONITOR_MAX_TASKS];
		static uint32_t last_runtime[FREERTOS_MONITOR_MAX_TASKS];

    UBaseType_t task_count;
    UBaseType_t i;
    uint32_t total_delta = 0;

    char buffer[128];

    task_count = uxTaskGetSystemState(
    task_status,
    FREERTOS_MONITOR_MAX_TASKS,
    NULL
);

    /*
     * 第一次采样只保存基准值
     */
    static uint8_t first_sample = 1;

    if (first_sample)
    {
        for (i = 0; i < task_count; i++)
        {
            last_handles[i] = task_status[i].xHandle;
            last_runtime[i] = task_status[i].ulRunTimeCounter;
        }

        first_sample = 0;

        UART_Manager_SendString(
            "\r\n===== FREERTOS CPU MONITOR =====\r\n"
            "First sample: baseline initialized\r\n"
            "=================================\r\n"
        );

        return;
    }

    /*
     * 计算所有任务本次采样周期的运行时间
     */
    uint32_t delta_runtime[10];

    for (i = 0; i < task_count; i++)
    {
        uint8_t found = 0;
        UBaseType_t j;

        delta_runtime[i] = 0;

       for (j = 0; j < FREERTOS_MONITOR_MAX_TASKS; j++)
        {
            if (last_handles[j] == task_status[i].xHandle)
            {
                /*
                 * uint32_t 无符号减法可以处理计数器回绕
                 */
                delta_runtime[i] =
                    task_status[i].ulRunTimeCounter -
                    last_runtime[j];

                found = 1;
                break;
            }
        }

        if (!found)
        {
            delta_runtime[i] = 0;
        }

        total_delta += delta_runtime[i];
    }

    /*
     * 输出标题
     */
    UART_Manager_SendString(
        "\r\n"
        "===== FREERTOS CPU MONITOR =====\r\n"
        "Name                 CPU\r\n"
        "--------------------------\r\n"
    );

    /*
     * 输出每个任务 CPU 占用率
     */
    for (i = 0; i < task_count; i++)
    {
        float cpu_usage = 0.0f;

        if (total_delta > 0)
        {
            cpu_usage =
                ((float)delta_runtime[i] /
                 (float)total_delta) * 100.0f;
        }

        snprintf(
            buffer,
            sizeof(buffer),
            "%-20s %6.2f%%\r\n",
            task_status[i].pcTaskName,
            cpu_usage
        );

        UART_Manager_SendString(buffer);
    }

    UART_Manager_SendString(
        "--------------------------\r\n"
    );

    /*
     * 保存本次采样值
     */
    for (i = 0; i < task_count; i++)
    {
        last_handles[i] = task_status[i].xHandle;
        last_runtime[i] = task_status[i].ulRunTimeCounter;
    }

    UART_Manager_SendString(
        "TOTAL               100.00%\r\n"
        "=================================\r\n"
    );
}



