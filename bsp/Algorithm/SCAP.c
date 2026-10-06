#include "SCAP.h"

static CAN_TxHeaderTypeDef SCAPtx_message;
SCAP_Rx SCAPrx;

//超电发送限制值设置函数(待更改为队列发送)
uint8_t Superpower_cap_Init(float power_limit,float buffer,uint8_t state,uint8_t reset)
{
	SCAP_Tx TxData;
	//设置数据:
	TxData.enableDCDC=state;
	TxData.systemRestart=reset;
//	TxData.clearError=1;
	TxData.resv0=0;
	TxData.refereePowerLimit=buffer;
	TxData.refereeEnergyBuffer=power_limit;
//	TxData.resv0[0]=0;
//	TxData.resv0[1]=0;
//	TxData.resv0[2]=0;
	
	//设置CAN发送:
	uint32_t send_mail_box;//用于接收邮箱编号
	SCAPtx_message.StdId=0x061;//0x061
	SCAPtx_message.ExtId=0x00;
	SCAPtx_message.RTR=CAN_RTR_DATA;
	SCAPtx_message.IDE=CAN_ID_STD;
	SCAPtx_message.DLC=sizeof(TxData);
	//将消息添加到邮箱:
	HAL_StatusTypeDef status=HAL_CAN_AddTxMessage(&SCAP_CAN,&SCAPtx_message,(uint8_t*)&TxData,&send_mail_box);
	
	return (status==HAL_OK)?1:0;
}

uint8_t Superpower_cap_Init2(float power_limit,float buffer,uint8_t state,uint8_t reset)
{
	SCAP_Tx TxData;
	//设置数据:
	TxData.enableDCDC=state;
	TxData.systemRestart=reset;
	TxData.clearError=1;
	TxData.resv0=0;
	TxData.refereePowerLimit=buffer;
	TxData.refereeEnergyBuffer=power_limit;
//	TxData.resv0[0]=0;
//	TxData.resv0[1]=0;
//	TxData.resv0[2]=0;
	
	//设置CAN发送:
	uint32_t send_mail_box;//用于接收邮箱编号
	SCAPtx_message.StdId=0x061;//0x061
	SCAPtx_message.ExtId=0x00;
	SCAPtx_message.RTR=CAN_RTR_DATA;
	SCAPtx_message.IDE=CAN_ID_STD;
	SCAPtx_message.DLC=sizeof(TxData);
	//将消息添加到邮箱:
	HAL_StatusTypeDef status=HAL_CAN_AddTxMessage(&SCAP_CAN,&SCAPtx_message,(uint8_t*)&TxData,&send_mail_box);
	
	return (status==HAL_OK)?1:0;
}


//uint8_t SuperpowerCap_set(float power_limit,float buffer)
//{
//	SCAP_Tx TxData;
//	//设置数据:
//	TxData.enableDCDC=1;
//	TxData.systemRestart=0;
//	TxData.resv0=0;
//	TxData.feedbackRefereeEnergyBuffer=buffer;
//	TxData.feedbackRefereePowerLimit=power_limit;
//	TxData.resv1[0]=0;
//	TxData.resv1[1]=0;
//	TxData.resv1[2]=0;
//	
//	//设置CAN发送:
//	uint32_t send_mail_box;//用于接收邮箱编号
//	SCAPtx_message.StdId=0x061;//0x061
//	SCAPtx_message.ExtId=0x00;
//	SCAPtx_message.RTR=CAN_RTR_DATA;
//	SCAPtx_message.IDE=CAN_ID_STD;
//	SCAPtx_message.DLC=sizeof(TxData);
//	//将消息添加到邮箱:
//	HAL_StatusTypeDef status=HAL_CAN_AddTxMessage(&SCAP_CAN,&SCAPtx_message,(uint8_t*)&TxData,&send_mail_box);
//	
//	return (status==HAL_OK)?1:0;
//}

//超电初始化:
void SCAP_Init(void)
{
	Superpower_cap_Init(0,60,1,0);//开超电，充快点
}

