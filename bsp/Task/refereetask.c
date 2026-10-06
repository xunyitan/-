#include "refereetask.h"

void referee_fifoTask(void *argument)
{
	TickType_t xLastWakeTime=xTaskGetTickCount();
	const TickType_t xFrequency=pdMS_TO_TICKS(10);
	while(1)
	{
		referee_unpack_fifo_data();
		vTaskDelayUntil(&xLastWakeTime,xFrequency);
	}
}
