#include "Gimbal_FindingMode.h"

//发送四元数的can函数:
void GimbalQuatSend(int Q0,int Q1,int Q2,int Q3)
{
	uint8_t GimbalQuat_data[4]={Q0,Q1,Q2,Q3};
	CAN2_TO_Queue(CAN_QUATERNION_ID,sizeof(GimbalQuat_data),GimbalQuat_data);
}

