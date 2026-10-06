#include "Can_receive.h"
#include "main.h"

/*
	用于3508,6020,2006,dmi_imu,超级电容的通信测试:
*/
static CAN_TxHeaderTypeDef C3508_tx_message;
static CAN_TxHeaderTypeDef G6020_tx_message;
static CAN_TxHeaderTypeDef C3508Fw_tx_message;
static CAN_TxHeaderTypeDef G2006_tx_message;

uint8_t pitch_off;


static motor_measure_t motor_chassis[7];
float Motor_Power[4]={0};

static uint8_t C3508_data[8];
static uint8_t G6020_data[4];

//motor_data_read:
#define get_motor_measure(ptr, data)                                    \
    {                                                                   \
        (ptr)->last_ecd = (ptr)->ecd;                                   \
        (ptr)->ecd = (uint16_t)((data)[0] << 8 | (data)[1]);            \
        (ptr)->speed_rpm = (uint16_t)((data)[2] << 8 | (data)[3]);      \
        (ptr)->given_current = (uint16_t)((data)[4] << 8 | (data)[5]);  \
        (ptr)->temperate = (data)[6];                                   \
    }

/**
  * @brief          hal库CAN回调函数,接收电机数据
  * @param[in]      hcan:CAN句柄指针
  * @retval         none
*/
void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *hcan)
{
	uint32_t state=taskENTER_CRITICAL_FROM_ISR();
	CAN_RxHeaderTypeDef  rx_header;
	uint8_t rx_data[8];
	if(HAL_CAN_GetRxMessage(hcan,CAN_RX_FIFO0,&rx_header,rx_data)==HAL_OK)
	{
		//检查是否是标准数据帧且数据长度为8字节(3508电机)
		if((rx_header.IDE==CAN_ID_STD)&&(rx_header.RTR==CAN_RTR_DATA)&&(rx_header.DLC==8))
		{
			if(hcan->Instance==CAN1)
			{
				switch(rx_header.StdId)
				{
					case CAN_3508_M1_ID:
					{
						djl_3508[0].Rx.Pos=(rx_data[0]<< 8) | (rx_data[1]);
						djl_3508[0].Rx.Speed=(rx_data[2] << 8) | (rx_data[3]);
						djl_3508[0].Rx.Torque=(rx_data[4] << 8) | (rx_data[5]);
						djl_3508[0].Rx.Temperature=rx_data[6];
						Motor_Power[0]=fabs(djl_3508[0].Rx.Speed*djl_3508[0].Rx.Torque*0.0000025567f);
						break;
					}
					case CAN_3508_M2_ID:
					{
						djl_3508[1].Rx.Pos=(rx_data[0]<< 8) | (rx_data[1]);
						djl_3508[1].Rx.Speed=(rx_data[2] << 8) | (rx_data[3]);
						djl_3508[1].Rx.Torque=(rx_data[4] << 8) | (rx_data[5]);
						djl_3508[1].Rx.Temperature=rx_data[6];
						Motor_Power[1]=fabs(djl_3508[1].Rx.Speed*djl_3508[1].Rx.Torque*0.0000025567f);
						break;
					}
					case CAN_3508_M3_ID:
					{
						djl_3508[2].Rx.Pos=(rx_data[0]<< 8) | (rx_data[1]);
						djl_3508[2].Rx.Speed=(rx_data[2] << 8) | (rx_data[3]);
						djl_3508[2].Rx.Torque=(rx_data[4] << 8) | (rx_data[5]);
						djl_3508[2].Rx.Temperature=rx_data[6];
						Motor_Power[2]=fabs(djl_3508[2].Rx.Speed*djl_3508[2].Rx.Torque*0.0000025567f);
						break;
					}
					case CAN_3508_M4_ID:
					{
						djl_3508[3].Rx.Pos = (rx_data[0]<< 8) | (rx_data[1]);
						djl_3508[3].Rx.Speed = (rx_data[2] << 8) | (rx_data[3]);
						djl_3508[3].Rx.Torque = (rx_data[4] << 8) | (rx_data[5]);
						djl_3508[3].Rx.Temperature = rx_data[6];
						Motor_Power[3]=fabs(djl_3508[3].Rx.Speed*djl_3508[3].Rx.Torque*0.0000025567f);
						break;
					}
					case SCAP_Slave://超电接收数据:
					{
						SCAPrx.errorCode=rx_data[0];
						uint32_t combined = (uint32_t)(rx_data[4]) << 24 |(uint32_t)(rx_data[3]) << 16 |(uint32_t)(rx_data[2]) << 8 |(uint32_t)(rx_data[1]);
						SCAPrx.chassisPower=*((float*)&combined);
						SCAPrx.chassisPowerLimit=rx_data[6]<<8|rx_data[5];
						SCAPrx.CapEnergy=rx_data[7];
						break;
					}
				}
			}
			if(hcan->Instance==CAN2)
			{
				switch(rx_header.StdId)
				{
					case CAN_3508_M1_ID:
					{
						djl_3508FW[0].Rx.Pos=(rx_data[0]<< 8) | (rx_data[1]);
						djl_3508FW[0].Rx.Speed=(rx_data[2] << 8) | (rx_data[3]);
						djl_3508FW[0].Rx.Torque=(rx_data[4] << 8) | (rx_data[5]);
						djl_3508FW[0].Rx.Temperature=rx_data[6];
						djl_3508FW[0].Rx.errorcode=rx_data[7];
						break;
					}
					case CAN_3508_M2_ID:
					{
						djl_3508FW[1].Rx.Pos=(rx_data[0]<< 8) | (rx_data[1]);
						djl_3508FW[1].Rx.Speed=(rx_data[2] << 8) | (rx_data[3]);
						djl_3508FW[1].Rx.Torque=(rx_data[4] << 8) | (rx_data[5]);
						djl_3508FW[1].Rx.Temperature=rx_data[6];
						break;
					}
					case CAN_3508_M3_ID:
					{
						djl_2006[0].Rx.Pos=(rx_data[0]<< 8) | (rx_data[1]);
						djl_2006[0].Rx.Speed=(rx_data[2] << 8) | (rx_data[3]);
						djl_2006[0].Rx.Torque=(rx_data[4] << 8) | (rx_data[5]);
						djl_2006[0].Rx.Temperature=rx_data[6];
						break;
					}
					case CAN_PIT_MOTOR_ID:
					{
						djl_6020[0].Rx.Pos=(rx_data[0]<< 8) | (rx_data[1]);
						djl_6020[0].Rx.Speed=(rx_data[2] << 8) | (rx_data[3]);
						djl_6020[0].Rx.Torque=(rx_data[4] << 8) | (rx_data[5]);
						djl_6020[0].Rx.Temperature=rx_data[6];
						break;
					}
					case CAN_YAW_MOTOR_ID:
					{
						djl_6020[1].Rx.Pos=(rx_data[0]<< 8) | (rx_data[1]);
						djl_6020[1].Rx.Speed=(rx_data[2] << 8) | (rx_data[3]);
						djl_6020[1].Rx.Torque=(rx_data[4] << 8) | (rx_data[5]);
						djl_6020[1].Rx.Temperature=rx_data[6];
						break;
					}
					case CAN_4310ID:
					{
						DM_Motor.Rx.Pos = uint_to_float((rx_data[1] << 8 | rx_data[2]), -12.5f, 12.5f, 16);
						DM_Motor.Rx.Speed = uint_to_float((rx_data[3] << 4 | rx_data[4] >> 4), -45.0f, 45.0f, 12);
						DM_Motor.Rx.Torque = uint_to_float(((rx_data[4] & 0xF) << 8) | rx_data[5], -18.0f, 18.0f, 12);
						break;
					}
					case CAN_FINDINGMODE_ID:
					{
						GimbalFinding_Mode[0].ControlCmd=rx_data[0];
						GimbalFinding_Mode[0].ShootCmd=rx_data[1];
						GimbalFinding_Mode[0].Yaw_raw=(rx_data[2]<<8)|rx_data[3];
						GimbalFinding_Mode[0].Yaw=(float)GimbalFinding_Mode[0].Yaw_raw/1e4;
						GimbalFinding_Mode[0].Pitch_yaw=(rx_data[4]<<8)|rx_data[5];
						GimbalFinding_Mode[0].Pitch=(float)GimbalFinding_Mode[0].Pitch_yaw/1e4;
						GimbalFinding_Mode[0].Distance_raw=(rx_data[6]<<8)|rx_data[7];
						GimbalFinding_Mode[0].Horizon_distance=(float)GimbalFinding_Mode[0].Distance_raw/1e4;
						break;
					}
					case 0x102:
					{
						pitch_off=rx_data[0];
						break;
					}
				}
			}
		}
	}
	taskEXIT_CRITICAL_FROM_ISR(state);
}

void C3508_cmd(int16_t motor1, int16_t motor2, int16_t motor3, int16_t motor4)
{
	uint32_t send_mail_box;//用于接收邮箱编号
	C3508_tx_message.StdId=CAN_CHASSIS_ALL_ID;
	C3508_tx_message.IDE=CAN_ID_STD;
	C3508_tx_message.RTR=CAN_RTR_DATA;
	C3508_tx_message.DLC=0x08;
	C3508_data[0]=motor1>>8;
	C3508_data[1]=motor1&0xFF;
	C3508_data[2]=motor2>>8;
	C3508_data[3]=motor2&0xFF;
	C3508_data[4]=motor3>>8;
	C3508_data[5]=motor3&0xFF;
	C3508_data[6]=motor4>>8;
	C3508_data[7]=motor4&0xFF;
	//将消息添加到邮箱:
	HAL_CAN_AddTxMessage(&CHASSIS_CAN,&C3508_tx_message,C3508_data,&send_mail_box);
}

void G6020_cmd(int16_t motor1,int16_t motor2)
{
	uint32_t send_mail_box;//用于接收邮箱编号
	G6020_tx_message.StdId=CAN_GIMBAL_ALL_ID;
	G6020_tx_message.IDE=CAN_ID_STD;
	G6020_tx_message.RTR=CAN_RTR_DATA;
	G6020_tx_message.DLC=0x08;
	G6020_data[0]=motor1>>8;//yaw
	G6020_data[1]=motor1&0xFF;
	G6020_data[2]=motor2>>8;//pitch
	G6020_data[3]=motor2&0xFF;
	//将消息添加到邮箱:
	HAL_CAN_AddTxMessage(&GIMBAL_CAN,&G6020_tx_message,G6020_data,&send_mail_box);
}

/**
  * @brief          返回底盘电机 3508电机数据指针
  * @param[in]      i: 电机编号,范围[0,3]
  * @retval         电机数据指针
  */
const motor_measure_t *get_chassis_motor_measure_point(uint8_t i)
{
	return &motor_chassis[(i & 0x03)];
}

