#include "RC_ProtectionTask.h"

void RC_ProtectionTask(void *argument)
{
	//获取当前时间:
	TickType_t xLastWakeTime=xTaskGetTickCount();
	//执行频率，一节拍::当前对应一毫秒(可以在freertosconfig里面更改)
	const TickType_t xFrequency=pdMS_TO_TICKS(20);
	RC_Ctl_t defaultRC;
	while(1)
	{
		taskENTER_CRITICAL();
		RC_Protection--;
		taskEXIT_CRITICAL();
		if(RC_Protection<4)
		{
			defaultRC.rc.s1=0;
			defaultRC.rc.s2=0;
			defaultRC.rc.ch0=1024;
			defaultRC.rc.ch1=1024;
			defaultRC.rc.ch2=1024;
			defaultRC.rc.ch3=1024;
			defaultRC.keep.che=1024;

			xQueueReset(RC_MouseCC);
			xQueueReset(RC_MouseCC2);
			xQueueReset(RC_MouseCC3);
			xQueueSend(RC_MouseCC,&defaultRC,0);
			xQueueSend(RC_MouseCC2,&defaultRC,0);
			xQueueSend(RC_MouseCC3,&defaultRC,0);
		}
		vTaskDelayUntil(&xLastWakeTime,xFrequency);
	}
}

