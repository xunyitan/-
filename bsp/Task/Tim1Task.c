#include "Tim1Task.h"

volatile unsigned long long FreeRTOSRunTimeTicks;

//回调函数更新中断
//void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
//{
//	if(htim->Instance==TIM1)
//	{
//		FreeRTOSRunTimeTicks++;
//	}
//}

//任务状态记录任务:任务名 堆栈大小 CPU占用率
void FreertosTimeTick_Task(void *argument)
{
	//获取当前时间:
	TickType_t xLastWakeTime=xTaskGetTickCount();
	//执行频率，一节拍::当前对应一毫秒(可以在freertosconfig里面更改)
	const TickType_t xFrequency=pdMS_TO_TICKS(500);
	static char TaskRunTimes[1024];
	while(1)
	{
		vTaskGetRunTimeStats(TaskRunTimes);
		//再打印出来:
//		usart_printf("%s\r\n",TaskRunTimes);
		vTaskDelayUntil(&xLastWakeTime,xFrequency);
	}
}


