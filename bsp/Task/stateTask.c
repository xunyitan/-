#include "stateTask.h"

CHE_state CHE_state_t;

//模块状态检测任务,当前用于检测keep.che的数值和方向的变化情况，用于给YAW轴6020一定的角速度
void CHE_stateTask(void *argument)
{
	RC_Ctl_t my_RCCtl;
	my_RCCtl.keep.che=RC_MIDDLE;
	//获取当前时间:
	TickType_t xLastWakeTime=xTaskGetTickCount();
	//执行频率，一节拍::当前对应一毫秒(可以在freertosconfig里面更改)
	const TickType_t xFrequency=pdMS_TO_TICKS(1);
	while(1)
	{
		xQueueReceive(RC_MouseCC,(void*)&my_RCCtl,pdMS_TO_TICKS(portMAX_DELAY));
		if((my_RCCtl.keep.che-RC_MIDDLE)>0)
		{
			CHE_state_t.zheng=1;
			CHE_state_t.fu=0;
		}
		if((my_RCCtl.keep.che-RC_MIDDLE)<0)
		{
			CHE_state_t.zheng=0;
			CHE_state_t.fu=1;
		}
		vTaskDelayUntil(&xLastWakeTime,xFrequency);
	}
}



