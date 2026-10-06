#include "CanTask.h"

static osMessageQId xCAN1Queue=NULL;
static osMessageQId xCAN2Queue=NULL;

void CAN1_Send_Queue_Init(void)
{
	xCAN1Queue=xQueueCreate(10,sizeof(CAN_TxMessage));
	if(xCAN1Queue==NULL)
	{
		Error_Handler();
	}
}

void CAN2_Send_Queue_Init(void)
{
	xCAN2Queue=xQueueCreate(15,sizeof(CAN_TxMessage));
	if(xCAN2Queue==NULL)
	{
		Error_Handler();
	}
}

void SendCAN1MsgQueue(void)
{
	CAN_TxMessage CAN1_Tx;
	uint32_t mailbox;
	if(xQueueReceive(xCAN1Queue,(void*)&CAN1_Tx,0)==pdPASS)
	{
		//暂时不加入状态判断
		HAL_CAN_AddTxMessage(&hcan1,&CAN1_Tx.Header,CAN1_Tx.data,&mailbox);
	}
}

void SendCAN2MsgQueue(void)
{
	CAN_TxMessage CAN2_Tx;
	uint32_t mailbox;
	if(xQueueReceive(xCAN2Queue,(void*)&CAN2_Tx,0)==pdPASS)
	{
		HAL_CAN_AddTxMessage(&hcan2,&CAN2_Tx.Header,CAN2_Tx.data,&mailbox);
	}
}


//数据帧标准帧
void CAN1_TO_Queue(uint32_t StdID,uint8_t DLC,uint8_t* Data)
{
	CAN_TxMessage Can1_Msg;  
	Can1_Msg.Header.IDE=CAN_ID_STD;
	Can1_Msg.Header.RTR=CAN_RTR_DATA;
	Can1_Msg.Header.StdId=StdID;
	Can1_Msg.Header.DLC=DLC;
	//复制信息:memcpy
	memcpy(Can1_Msg.data,Data,Can1_Msg.Header.DLC);
	//消息队列发送:
	xQueueSend(xCAN1Queue,(void*)&Can1_Msg,0);
}

void CAN2_TO_Queue(uint32_t StdID,uint8_t DLC,uint8_t* Data)
{
	CAN_TxMessage Can2_Msg;
	Can2_Msg.Header.IDE=CAN_ID_STD;
	Can2_Msg.Header.RTR=CAN_RTR_DATA;
	Can2_Msg.Header.StdId=StdID;
	Can2_Msg.Header.DLC=DLC;
	//复制信息:memcpy
	memcpy(Can2_Msg.data,Data,Can2_Msg.Header.DLC);
	//消息队列发送:
	xQueueSend(xCAN2Queue,(void*)&Can2_Msg,0);
}

void CAN1_Task(void *argument)
{
	TickType_t xLastWakeTime=xTaskGetTickCount();
	const TickType_t xFrequency=pdMS_TO_TICKS(1);
	while(1)
	{
		SendCAN1MsgQueue();
		vTaskDelayUntil(&xLastWakeTime,xFrequency);
	}
}

void CAN2_Task(void *argument)
{
	TickType_t xLastWakeTime=xTaskGetTickCount();
	const TickType_t xFrequency=pdMS_TO_TICKS(1);
	while(1)
	{
		SendCAN2MsgQueue();
		vTaskDelayUntil(&xLastWakeTime,xFrequency);
	}
}
