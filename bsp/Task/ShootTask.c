#include "ShootTask.h"

uint8_t djlFW_data[6];
FW_Shoot FW_Shoot_t;
FW_FLGA  FW_FLGA_t;
PidTypeDef FrictionWheel_Speed[2];
PidTypeDef Strumming_Speed;

float FrictionWheel_Speed_PID[3]={26,0.99,0.2};
float Strumming_Speed_PID[3]={26,1,0};

float ShootingUpLimit;
float ShootSpeed;

int8_t fwspeedupFlag=0;
int8_t fwspeednovidion=0;
int8_t shootcmd=0;

//拨弹盘旋转任务:
void Shooting_Task(void *argument)
{
	TickType_t xLastWakeTime=xTaskGetTickCount();
	const TickType_t xFrequency=pdMS_TO_TICKS(2);
	RC_Ctl_t RC;
	CAN_TxHeaderTypeDef CAN2_Tx_Header;
	uint32_t mailbox;
	CAN2_Tx_Header.IDE=CAN_ID_STD;
	CAN2_Tx_Header.RTR=CAN_RTR_DATA;
	CAN2_Tx_Header.StdId=0x200;
	CAN2_Tx_Header.DLC=8;
	while(1)
	{
		xQueueReceive(RC_MouseCC3,&RC,xFrequency);
		if(RC.rc.s2==1)
		{
			FW_Shooting(-3500);
//			if(RC.mouse.press_r>0)
//			{
//				fwspeedupFlag=1;
//			}
//			if(RC.key.q>0)
//			{
//				fwspeedupFlag=0;
//			}
//			if(RC.mouse.press_l>0)
//			{
//				FW_Shooting(-3500);
//			}
//			else
//			{
//				if(RC.key.g>0)
//				{
//					FW_Shooting(3500);
//				}
//				else
//				{
//					FW_Shooting(0);
//				}
//			}
//			if(fwspeedupFlag==1)
//			{
//				FW_SpeedUP(6900,-6900);
//			}
//			if(fwspeedupFlag==0)
//			{
//				FW_SpeedUP(0,0);
//			}
			djlFW_data[0]=djl_3508FW[0].Tx.m_Speed>>8;
			djlFW_data[1]=djl_3508FW[0].Tx.m_Speed&0xFF;
			djlFW_data[2]=djl_3508FW[1].Tx.m_Speed>>8;
			djlFW_data[3]=djl_3508FW[1].Tx.m_Speed&0xFF;
			djlFW_data[4]=djl_2006[0].Tx.m_Speed>>8;
			djlFW_data[5]=djl_2006[0].Tx.m_Speed&0xFF;
			HAL_CAN_AddTxMessage(&hcan2,&CAN2_Tx_Header,djlFW_data,&mailbox);
	  }
		else
		{
			djl_2006[0].Tx.m_Speed=0;
			djl_3508FW[0].Tx.m_Speed=0;
			djl_3508FW[1].Tx.m_Speed=0;
			djlFW_data[0]=djl_3508FW[0].Tx.m_Speed>>8;
			djlFW_data[1]=djl_3508FW[0].Tx.m_Speed&0xFF;
			djlFW_data[2]=djl_3508FW[1].Tx.m_Speed>>8;
			djlFW_data[3]=djl_3508FW[1].Tx.m_Speed&0xFF;
			djlFW_data[4]=djl_2006[0].Tx.m_Speed>>8;
			djlFW_data[5]=djl_2006[0].Tx.m_Speed&0xFF;
			HAL_CAN_AddTxMessage(&hcan2,&CAN2_Tx_Header,djlFW_data,&mailbox);
		}
		usart_printf("%f,%f\r\n",(float)power_heat_data_t.shooter_heat0,(float)robot_state.chassis_power_limit);
		vTaskDelayUntil(&xLastWakeTime,xFrequency);
	}
}

//摩擦轮加速函数
void FW_SpeedUP(int FW_1,int FW_2)
{
	FW_Shoot_t.M3508_FW_Out[0]=FrictionWheel_Speed[0].Cal_PID(&FrictionWheel_Speed[0],djl_3508FW[0].Rx.Speed,FW_1);
	FW_Shoot_t.M3508_FW_Out[1]=FrictionWheel_Speed[1].Cal_PID(&FrictionWheel_Speed[1],djl_3508FW[1].Rx.Speed,FW_2);
	djl_3508FW[0].Tx.m_Speed=FW_Shoot_t.M3508_FW_Out[0];
	djl_3508FW[1].Tx.m_Speed=FW_Shoot_t.M3508_FW_Out[1];
}

//射击函数
void FW_Shooting(int speed)
{
	FW_Shoot_t.M2006_Out=Strumming_Speed.Cal_PID(&Strumming_Speed,djl_2006[0].Rx.Speed,speed);
	djl_2006[0].Tx.m_Speed=FW_Shoot_t.M2006_Out;
}

