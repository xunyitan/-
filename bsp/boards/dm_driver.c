#include "dm_driver.h"

DM_Motor_t DM_Motor;

void DM_MotorInit(DM_Motor_t* pthis, uint32_t StdID, uint8_t DLC, DM_MotorCtrlMode Mode)
{
	pthis->StdID = StdID + Mode;
	pthis->DLC = DLC;
	
	pthis->Tx.Pos = 0;
	pthis->Tx.Speed = 0;
	pthis->Tx.Torque = 0;
	pthis->Tx.Kp = 0;
	pthis->Tx.Kd = 0;
	
	pthis->Rx.Error = 0;
	pthis->Rx.Pos = 0;
	pthis->Rx.Speed = 0;
	pthis->Rx.Torque = 0;
}

float uint_to_float(int x_int, float x_min, float x_max, int bits)
{
	float span = x_max - x_min;
	float offset = x_min;
	return ((float)x_int)*span/((float)((1<<bits)-1)) + offset;
}

int float_to_uint(float x, float x_min, float x_max, int bits) 
{
    float span = x_max - x_min;
    float offset = x_min;
    return (int) ((x - offset) * ((float)((1 << bits) - 1)) / span);
}

void DM_MotorEnable(DM_Motor_t *pthis)
{
	uint8_t Enable[8]  = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFC};
	pthis->DLC = sizeof(Enable);
	CAN_TxHeaderTypeDef TxHeader;
	uint32_t mailbox;
	TxHeader.IDE = CAN_ID_STD;
	TxHeader.RTR = CAN_RTR_DATA;
	TxHeader.StdId = pthis->StdID;
	TxHeader.DLC = pthis->DLC;
	HAL_CAN_AddTxMessage(&hcan2, &TxHeader, Enable, &mailbox);
}

void DM_MitControl(DM_Motor_t *pthis)
{
	if(!pthis)
		return;
	
	uint16_t tmp_pos, tmp_val, tmp_torque, tmp_k_p, tmp_k_d;
	tmp_pos = float_to_uint(pthis->Tx.Pos, P_MIN, P_MAX, 16);
	tmp_val = float_to_uint(pthis->Tx.Speed, V_MIN, V_MAX, 12);
	tmp_torque = float_to_uint(pthis->Tx.Torque, T_MIN, T_MAX, 12);
	tmp_k_p = float_to_uint(pthis->Tx.Kp, KP_MIN, KP_MAX, 12);
	tmp_k_d = float_to_uint(pthis->Tx.Kd, KD_MIN, KD_MAX, 12);

	pthis->TxData[0] = (tmp_pos >> 8);
	pthis->TxData[1] = tmp_pos;
	pthis->TxData[2] = (tmp_val >> 4);
	pthis->TxData[3] = ((tmp_val & 0x0F) << 4) | (tmp_k_p >> 8);
	pthis->TxData[4] = tmp_k_p;
	pthis->TxData[5] = (tmp_k_d >> 4);
	pthis->TxData[6] = ((tmp_k_d & 0x0F) << 4)| (tmp_torque>>8);
	pthis->TxData[7] = tmp_torque; 
	CAN2_TO_Queue(pthis->StdID,pthis->DLC,pthis->TxData);
}

void Gimbal4310_EnableTask(void *argument)
{
	TickType_t xLastWakeTime=xTaskGetTickCount();
  const TickType_t xFrequency=pdMS_TO_TICKS(10);
	while(1)
	{
		DM_MotorEnable(&DM_Motor);
		vTaskDelayUntil(&xLastWakeTime,xFrequency);
	}
}





