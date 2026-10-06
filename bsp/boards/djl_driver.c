#include "djl_driver.h"

djl_motorCan_t djl_3508[4];
djl_motorCan_t djl_3508FW[2];
djl_motorCan_t djl_6020[4];
djl_motorCan_t djl_2006[1];
Gimbal_FindingModeRx_t GimbalFinding_Mode[2];
bmi088_t bmi088_param[2];
Moving_flag Moving_flag_t;
Gimbal_FindingModeTx_t GimbalFinding_ModeTx[2];
Gimbal_FindingModeRx_t GimbalFinding_Mode[2];

void djlMotor_Init(djl_motorCan_t *pthis,uint32_t StdID)
{
	//设置id:
	pthis->StdID=StdID;
	//设置数据帧长度:
	pthis->DLC=0x00;
	//设置发送数据:
	pthis->Tx.m_Current=0;
	pthis->Tx.m_Pos=0;
	pthis->Tx.m_Speed=0;
	//设置接收数据:
	pthis->Rx.Pos=0;
	pthis->Rx.Speed=0;
	pthis->Rx.Temperature=0;
	pthis->Rx.Torque=0;
}


