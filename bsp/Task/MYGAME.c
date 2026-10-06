#include "MYGAME.h"

void MYGame(void* Args)
{
	buzzer_task();
	taskENTER_CRITICAL();
	//所有任务创建
	taskEXIT_CRITICAL();
	//删除自身，释放内存:
	vTaskDelete(NULL);
}
