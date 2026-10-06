#include "VisionSendTask.h"

uint8_t VisionData[8];

void VisionSendTask(void *argument)
{
	TickType_t xLastWakeTime=xTaskGetTickCount();
	const TickType_t xFrequency=pdMS_TO_TICKS(2);
	while(1)
	{
		GimbalFinding_ModeTx[0].Q1=(int32_t)(INS.q[1]*1e4);//x
		GimbalFinding_ModeTx[1].Q2=(int32_t)(INS.q[2]*1e4);//y
		GimbalFinding_ModeTx[2].Q3=(int32_t)(INS.q[3]*1e4);//z
		GimbalFinding_ModeTx[3].Q4=(int32_t)(INS.q[0]*1e4);//w
		
		//已解决数据溢出问题:
		VisionData[0]=(uint8_t)((GimbalFinding_ModeTx[0].Q1)>>8);
		VisionData[1]=(uint8_t)((GimbalFinding_ModeTx[0].Q1)&0xFF);
		VisionData[2]=(uint8_t)((GimbalFinding_ModeTx[1].Q2)>>8);
		VisionData[3]=(uint8_t)((GimbalFinding_ModeTx[1].Q2)&0xFF);
		VisionData[4]=(uint8_t)((GimbalFinding_ModeTx[2].Q3)>>8);
		VisionData[5]=(uint8_t)((GimbalFinding_ModeTx[2].Q3)&0xFF);
		VisionData[6]=(uint8_t)((GimbalFinding_ModeTx[3].Q4)>>8);
		VisionData[7]=(uint8_t)((GimbalFinding_ModeTx[3].Q4)&0xFF);
		
		CAN2_TO_Queue(0x100,sizeof(VisionData),VisionData);
		
		
		vTaskDelayUntil(&xLastWakeTime,xFrequency);
	}
}
