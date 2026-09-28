#ifndef __FREERTOS_MONITOR_H
#define __FREERTOS_MONITOR_H

#include "FreeRTOS.h"
#include "task.h"

void FreeRTOS_Monitor_PrintTasks(void);
void FreeRTOS_Monitor_PrintCPU(void);

#endif
