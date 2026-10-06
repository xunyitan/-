#include "Gimbal6020.h"
#include "Can_receive.h"


uint8_t djl6020_data[8];
Gimbal_Remote Gimbal_Remote_t;
GimbalYaw_SMC_t GimbalYaw_SMC;
Lowpassfilter_t Lowpassfilter_6020[4];
PidTypeDef Gimbal_Yaw_Speed;
PidTypeDef Gimbal_Pitch_Speed;
float Gimbal_Yaw_Speed_PID[3]= {11,1,0};//11,1,0
float Gimbal_Pitch_Speed_PID[3]= {6,1,0.1};//{6,1,0.1};
float tau_pitch;
float pitch4310Target;
float PitchTarget;
//自瞄控制器相关变量:
//yaw:
float error,target,actual,last_actual;
float integral,lasterror;
float out;
float Kp=2500,Ki=1.5,Kd=0;//2500 1.5（自瞄pid）

float error1,target1,Truetarget,actual1,last_actual1;
float integral1,lasterror1;
float out1;
float Kp1=20,Ki1=0,Kd1=0;//
uint8_t FindingModeCount;

//pitch:
float error2,target2,actual2,last_actual2;
float integral2,lasterror2;
float out2;
float Kp2=4,Ki2=0.03,Kd2=0;//6 0.05

float error3,target3,Truetarget3,actual3,last_actual3;
float integral3,lasterror3;
float out3;
float Kp3=0.05,Ki3=0,Kd3=0;//0.06
uint8_t FindingModeCount3;


float errorp,lasterrorp,integralp,outp;
float errorl,lasterrorl,integrall,outl;
float Kpp=4,Kip;
float KLp,KLi;
float targetL;
extern uint8_t pitch_off;


float LQRerror,LQRintegral;

float Pitchout;

float ErrorP,integralP,lasterror;

float Data_Clipping(float Data,float Data_Min,float Data_Max)
{
	if(Data>Data_Max)return Data_Max;
	else if(Data<Data_Min)return Data_Min;
	return Data;
}

void GimbalTask_Test(void *argument)
{
	TickType_t xLastWakeTime=xTaskGetTickCount();
	const TickType_t xFrequency=pdMS_TO_TICKS(2);
	RC_Ctl_t my_RCCtl2;	
	CAN_TxHeaderTypeDef CAN2_Tx_Header;
	CAN_TxHeaderTypeDef CAN24310_TxHeader;
	uint32_t mailbox,mailbox2;
	CAN2_Tx_Header.IDE=CAN_ID_STD;
	CAN2_Tx_Header.RTR=CAN_RTR_DATA;
	CAN2_Tx_Header.StdId=0x1ff;
	CAN2_Tx_Header.DLC=8;
	my_RCCtl2.rc.ch0=RC_MIDDLE;
	my_RCCtl2.rc.ch1=RC_MIDDLE;
	while(1)
	{
		xQueueReceive(RC_MouseCC2,(void*)&my_RCCtl2,xFrequency);
		Gimbal_Remote_t.X=my_RCCtl2.rc.ch0;
		Gimbal_Remote_t.Y=my_RCCtl2.rc.ch1;
		if(my_RCCtl2.rc.s2==RC_S2_MIDDLE||my_RCCtl2.rc.s2==RC_S2_DOWN||my_RCCtl2.rc.s2==RC_S2_UP)
		{
			if(GimbalFinding_Mode[0].ControlCmd>0)
			{
				if(my_RCCtl2.key.f==1)
				{
					GimbalFindingMode_PID(GimbalFinding_Mode[0].Pitch,-INS.Pitch); 
					GimbalFindingMode_PIDDouble(GimbalFinding_Mode[0].Yaw,INS.Yaw);
					if(Moving_flag_t.Gyro_360_motion_flag==1)
					{
						djl_6020[1].Tx.m_Speed=(short)(out)+(short)(djl_6020[1].Control_t.Speed.P_Myset.Out);
					}
					if(Moving_flag_t.Gyro_360_motion_flag!=1)
					{
						djl_6020[1].Tx.m_Speed=(short)(out);
					}
					//Yaw 轴赋值不变
					djl6020_data[0] = 0;
					djl6020_data[1] = 0;
					djl6020_data[2] = djl_6020[1].Tx.m_Speed >> 8;
					djl6020_data[3] = djl_6020[1].Tx.m_Speed&0xFF;//yaw
					DM_Motor.Tx.Pos=0;
					DM_Motor.Tx.Speed=0;
					DM_Motor.Tx.Torque = Pitchout+out2*0;
					DM_Motor.Tx.Kp=0;  
					DM_Motor.Tx.Kd=0;
					djl_6020[0].Tx.m_Speed=0;
					DM_MitControl(&DM_Motor);
					HAL_CAN_AddTxMessage(&hcan2,&CAN2_Tx_Header,djl6020_data,&mailbox);
				}
				else
				{
					djl_6020[1].Control_t.Speed.P_Myset.Target=-(double)((Gimbal_Remote_t.X+my_RCCtl2.mouse.x*5)-RC_MIDDLE)*Gimbal_Sensitivity;//yaw
//					djl_6020[1].Control_t.Speed.P_Myset.Target=LowPassFilter_Update(&LowPassFilter[1],djl_6020[1].Control_t.Speed.P_Myset.Target);
					djl_6020[1].Control_t.Speed.P_Myset.Actual=BMI088.GyroRawLSB[2];
					djl_6020[1].Control_t.Speed.P_Myset.Out=Gimbal_Yaw_Speed.Cal_PID(&Gimbal_Yaw_Speed,djl_6020[1].Control_t.Speed.P_Myset.Actual,djl_6020[1].Control_t.Speed.P_Myset.Target);
					djl_6020[1].Tx.m_Speed=(short)(djl_6020[1].Control_t.Speed.P_Myset.Out);//yaw
					pitch4310Target+=(-((Gimbal_Remote_t.Y+my_RCCtl2.mouse.y*4)-RC_MIDDLE)*0.000023);
					pitch4310Target=Data_Clipping(pitch4310Target,-2.9,0.30);
					Gimbal_PitchLQRControll(-pitch4310Target);
					DM_Motor.Tx.Pos=0;
					DM_Motor.Tx.Speed=0;
					DM_Motor.Tx.Torque=-tau_pitch;//tau_pitch
					DM_Motor.Tx.Kp=0;  
					DM_Motor.Tx.Kd=0;
					djl_6020[0].Tx.m_Speed=0;

					djl6020_data[0] = djl_6020[0].Tx.m_Speed >> 8;
					djl6020_data[1] = djl_6020[0].Tx.m_Speed&0xFF;
					djl6020_data[2] = djl_6020[1].Tx.m_Speed >> 8;
					djl6020_data[3] = djl_6020[1].Tx.m_Speed&0xFF;//yaw
					DM_MitControl(&DM_Motor);
					HAL_CAN_AddTxMessage(&hcan2,&CAN2_Tx_Header,djl6020_data,&mailbox);
				}
			}
			else
			{
				djl_6020[1].Control_t.Speed.P_Myset.Target=-(double)((Gimbal_Remote_t.X+my_RCCtl2.mouse.x*5)-RC_MIDDLE)*Gimbal_Sensitivity;//yaw
//				djl_6020[1].Control_t.Speed.P_Myset.Target=LowPassFilter_Update(&LowPassFilter[1],djl_6020[1].Control_t.Speed.P_Myset.Target);
				djl_6020[1].Control_t.Speed.P_Myset.Actual=BMI088.GyroRawLSB[2];
				djl_6020[1].Control_t.Speed.P_Myset.Out=Gimbal_Yaw_Speed.Cal_PID(&Gimbal_Yaw_Speed,djl_6020[1].Control_t.Speed.P_Myset.Actual,djl_6020[1].Control_t.Speed.P_Myset.Target);
				djl_6020[1].Tx.m_Speed=(short)(djl_6020[1].Control_t.Speed.P_Myset.Out);//yaw
				pitch4310Target+=(-((Gimbal_Remote_t.Y+my_RCCtl2.mouse.y*4)-RC_MIDDLE)*0.000023);//4是灵敏度
				PitchTarget+=(-(Gimbal_Remote_t.Y-1024)*0.00002);//0.000023
				pitch4310Target=Data_Clipping(pitch4310Target,-2.9,0.30);
				PitchTarget=Data_Clipping(PitchTarget,-2.9,0.3);
				Gimbal_PitchLQRControll(-pitch4310Target);
				Gimbal_TwoPIDControl(PitchTarget);
				DM_Motor.Tx.Pos=0;
				DM_Motor.Tx.Speed=0;
				DM_Motor.Tx.Torque=-tau_pitch+(outp)*0;//tau_pitch
				DM_Motor.Tx.Kp=0;  
				DM_Motor.Tx.Kd=0;
				djl_6020[0].Tx.m_Speed=0;

				djl6020_data[0] = djl_6020[0].Tx.m_Speed >> 8;
				djl6020_data[1] = djl_6020[0].Tx.m_Speed&0xFF;
				djl6020_data[2] = djl_6020[1].Tx.m_Speed >> 8;
				djl6020_data[3] = djl_6020[1].Tx.m_Speed&0xFF;//yaw
				DM_MitControl(&DM_Motor);
				HAL_CAN_AddTxMessage(&hcan2,&CAN2_Tx_Header,djl6020_data,&mailbox);
			}
		}
		else
		{
			pitch4310Target=Attitude.RadPitch;
			djl_6020[0].Tx.m_Speed=0;//pitch
			djl_6020[1].Tx.m_Speed=0;//yaw
			
			Gimbal_PitchLQRControll(-pitch4310Target);
			DM_Motor.Tx.Pos=0;
			DM_Motor.Tx.Speed=0;
			DM_Motor.Tx.Torque=-tau_pitch;//tau_pitch
			DM_Motor.Tx.Kp=0;  
			DM_Motor.Tx.Kd=0;
			
			djl6020_data[0] = djl_6020[0].Tx.m_Speed >> 8;
			djl6020_data[1] = djl_6020[0].Tx.m_Speed&0xFF;//pitch
			djl6020_data[2] = djl_6020[1].Tx.m_Speed >> 8;
			djl6020_data[3] = djl_6020[1].Tx.m_Speed&0xFF;//yaw
			
			DM_MitControl(&DM_Motor);
			HAL_CAN_AddTxMessage(&hcan2,&CAN2_Tx_Header,djl6020_data,&mailbox);
		}
		
		vTaskDelayUntil(&xLastWakeTime,xFrequency);
	}
}

void Gimbal_PitchLQRControll(float targetPitch)
{
	#define K1 9 //12
	#define K2 0.6 //1 3
	#define KI 0.03 
	
	LQRerror=targetPitch-INS.Pitch;
	LQRintegral+=LQRerror;
	LQRintegral=Limit(LQRintegral,-20,20);
	
	tau_pitch=K1*LQRerror+KI*LQRintegral-K2*INS.GyroFiltered[1];//后续改变目标值只需要通过摇杆改变PITCH轴的对应数值即可控制PITCH轴
	//再限幅
	tau_pitch=Limit(tau_pitch,-5,5);
}

void Gimbal_TwoPIDControl(float targetPitch)
{
	errorp=targetPitch*57.2958f-INS.Pitch*57.2958f;
	if(Kip!=0)
	{
		integralp+=errorp;
	}
	else
	{
		integralp=0;
	}
	integralp=Data_Clipping(integralp,-100,100);
	outp=Kpp*errorp+Kip*integralp;
	outp=Data_Clipping(outp,-5,5);
	lasterrorp=errorp;
	
	targetL=outp;
	errorl=targetL-INS.GyroFiltered[1]*57.2958f;
	if(KLi!=0)
	{
		integrall+=errorl;
	}
	else
	{
		integrall=0;
	}
	integrall=Data_Clipping(integrall,-1000,1000);
	outl=KLp*errorl+KLi*integrall;
	outl=Data_Clipping(outl,-8,8);
	lasterrorl=errorl;
}

// --- 重写并完善Pitch轴自瞄串级PID：外环角度，内环角速度 ---
void GimbalFindingMode_PID(float target,float actual)
{
	// 1. 外环：角度环
	// 假设 target 单位为度(°)，传入的 actual 为弧度，需转为角度
	error2 = (target * 57.2958f - 10) - actual * 57.2958f;//5
	
	if(Ki2 != 0) {
		integral2 += error2;
	} else {
		integral2 = 0;
	}  
	integral2 = Limit(integral2, -100, 100); // 积分限幅
	
	// 外环输出计算，同时补充微分 Kd2 项
	out2 = Kp2 * error2 + Ki2 * integral2 + Kd2 * (error2 - lasterror2);
	out2 = Limit(out2, -60, 60); // 外环输出目标角速度限幅
	lasterror2 = error2;

	// 2. 内环：角速度环
	target3 = out2;//out2 // 外环输出作为内环目标
	// 获取陀螺仪Pitch轴的反馈角速度 (rad/s 转为 deg/s)
	actual3 = -INS.GyroFiltered[1] * 57.2958f;
	error3 = target3 - actual3;
	
	if(Ki3 != 0) {
		integral3 += error3;
	} else {
		integral3 = 0;  
	}
	integral3 = Limit(integral3, -3000, 3000); // 内环积分限幅
	
	// 内环输出计算，补充微分 Kd3 项
	out3 = Kp3 * error3 + Ki3 * integral3 + Kd3 * (error3 - lasterror3);
	// 达妙电机发送的力矩/电流限幅，防止超调导致疯甩
	out3 = Limit(out3, -8.0f, 8.0f); //-5.0f, 5.0f
	lasterror3 = error3;

	// 3. 最终输出力矩
	Pitchout = out3;
//	usart_printf("%f,%f\r\n", target * 57.2958f, actual * 57.2958f);//外环
}
// -------------------------------------------------------------

void GimbalFindingMode_PIDDouble(float target,float actual)
{
	//角度环:
	if(target==0)
	{
		FindingModeCount++;
		if(FindingModeCount>=10)
		{
			target=0;
		}
		else
		{
			target=target;
			FindingModeCount=0;
		}
	}
	else
	{
		target=target;
	}
	Truetarget=target;
	error1=(Truetarget*57.2958f-0.8)-actual*57.2958f;
	if(Ki1!=0)
	{
		integral+=error1;
	}
	else
	{
		integral=0;
	}
	out1=Kp1*error1+Ki1*integral+Kd*(error1-lasterror1);
	integral=Limit(integral,-1000,1000);
	out1=Limit(out1,-180,180);
	lasterror1=error1;
	
	//角速度环:
	target=out1;
	actual=INS.GyroFiltered[2]*57.2958f;
	error=target-actual;
	if(Ki!=0)
	{
		integral1+=error1;
	}
	else
	{
		integral1=0;
	}
	out=Kp*error+Ki*integral1+Kd*(error-lasterror);
	integral1=Limit(integral1,-3000,3000);
	out=Limit(out,-9000,9000);
	lasterror=error;
}

//待修改...
void SMC_GimbalTask(void *argument)
{
	TickType_t xLastWakeTime=xTaskGetTickCount();
	const TickType_t xFrequency=pdMS_TO_TICKS(2);
	RC_Ctl_t RC;
	RC.rc.ch0=RC_MIDDLE;
	RC.rc.ch1=RC_MIDDLE;
	while(1)
	{
		xQueueReceive(RC_MouseCC2,&RC,xFrequency);
		if(RC.rc.s2==RC_S2_MIDDLE||RC.rc.s2==RC_S2_UP||RC.rc.s2==RC_S2_DOWN)
		{
			Gimbal_Remote_t.X=RC.rc.ch0;
			
			GimbalYaw_SMC.stick_value=(double)((Gimbal_Remote_t.X+RC.mouse.x*3)-RC_MIDDLE)*Gimbal_Sensitivity;//+-5280
			GimbalYaw_SMC.yaw_rate=(GimbalYaw_SMC.stick_value/5280.0f)*180.0f;
			GimbalYaw_SMC.angle_increment=GimbalYaw_SMC.yaw_rate;
			djl_6020[1].Control_t.Speed.P_Myset.Target+=GimbalYaw_SMC.angle_increment;
			if(djl_6020[1].Control_t.Speed.P_Myset.Target>180)
				djl_6020[1].Control_t.Speed.P_Myset.Target-=360;
			if(djl_6020[1].Control_t.Speed.P_Myset.Target<-180)
				djl_6020[1].Control_t.Speed.P_Myset.Target+=360;
			
			djl_6020[1].Control_t.Speed.P_Myset.Actual=bmi088_param[0].angle[2];//°每秒
			djl_6020[1].Control_t.Speed.P_Myset.Actual_vel=bmi088_param[0].gyro[2];//°每秒
			
			SMC_Controller_Update(&Yaw_SMC,djl_6020[1].Control_t.Speed.P_Myset.Target,
			djl_6020[1].Control_t.Speed.P_Myset.Actual,
			djl_6020[1].Control_t.Speed.P_Myset.Actual_vel,
			0.002);
			
			djl_6020[0].Tx.m_Speed=0;
			djl_6020[1].Tx.m_Speed=(short)(Yaw_SMC.out);
			
			djl6020_data[0]=djl_6020[0].Tx.m_Speed>>8;
			djl6020_data[1]=djl_6020[0].Tx.m_Speed&0xFF;
			djl6020_data[2]=djl_6020[1].Tx.m_Speed>>8;
			djl6020_data[3]=djl_6020[1].Tx.m_Speed&0xFF;//yaw
			
			CAN2_TO_Queue(0x1ff,sizeof(djl6020_data),djl6020_data);
			
		}
		else
		{
			djl_6020[0].Tx.m_Speed=0;
			djl_6020[1].Tx.m_Speed=0;//yaw
			
			djl6020_data[0]=djl_6020[0].Tx.m_Speed>>8;
			djl6020_data[1]=djl_6020[0].Tx.m_Speed&0xFF;
			djl6020_data[2]=djl_6020[1].Tx.m_Speed>>8;
			djl6020_data[3]=djl_6020[1].Tx.m_Speed&0xFF;//yaw
			CAN2_TO_Queue(0x1ff,sizeof(djl6020_data),djl6020_data);
		}
		vTaskDelayUntil(&xLastWakeTime,xFrequency);
	}
}
