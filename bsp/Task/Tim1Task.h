#ifndef TIM1TASK_H
#define TIM1TASK_H

#include "Sys.h"
#include "task.h"

extern volatile unsigned long long FreeRTOSRunTimeTicks;

void FreertosTimeTick_Task(void *argument);

#endif
