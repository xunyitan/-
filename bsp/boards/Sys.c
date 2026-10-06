#include "Sys.h"

void System_Init(void)
{
	//底盘电机相关参数和所用控制器初始化:
	for(uint8_t i=0;i<4;i++)
	{
		djlMotor_Init(&djl_3508[i],i);//电机id初始化
		pid_init2(&Chassis_Speed[i]);
		pid_init2(&Chassis_Gyro_Speed[i]);
		pid_init2(&Turn_Middle[i]);
		Chassis_Speed[i].Param_Init(&Chassis_Speed[i],PID_DELTA,Chassis_Speed_PID,10000,5000,16384);
		Chassis_Gyro_Speed[i].Param_Init(&Chassis_Gyro_Speed[i],PID_DELTA,Chassis_Gyro_Speed_PID,10000,5000,16384);
		Turn_Middle[i].Param_Init(&Turn_Middle[i],PID_POSITION2,Turn_Middle_PID,10000,5000,16384);
	}
	Moving_flag_t.Chassis_flowingflag=0;
	Moving_flag_t.Chassis_sidewayflag=0;
	Moving_flag_t.Gyro_360_motion_flag=0;
	
	//云台电机相关参数和所用控制器初始化:
	for(uint8_t i=0;i<2;i++)
	{
		djlMotor_Init(&djl_6020[i],i);
		LowPassFilter_Init(&LowPassFilter[i],0.5);
	}
	pid_init2(&Gimbal_Pitch_Speed);
	pid_init2(&Gimbal_Yaw_Speed);
	Gimbal_Pitch_Speed.Param_Init(&Gimbal_Pitch_Speed,PID_POSITION,Gimbal_Pitch_Speed_PID,9999,19999,29999);
	Gimbal_Yaw_Speed.Param_Init(&Gimbal_Yaw_Speed,PID_POSITION,Gimbal_Yaw_Speed_PID,9999,19999,29999);
	
	//发射机构相关参数和所用控制器初始化:
	for(uint8_t i=0;i<2;i++)
	{
		djlMotor_Init(&djl_3508FW[i],i);
	}
	djlMotor_Init(&djl_2006[0],0);
	pid_init2(&FrictionWheel_Speed[0]);
	pid_init2(&FrictionWheel_Speed[1]);
	pid_init2(&Strumming_Speed);
	FrictionWheel_Speed[0].Param_Init(&FrictionWheel_Speed[0],PID_POSITION,FrictionWheel_Speed_PID,150,10000,17999);
	FrictionWheel_Speed[1].Param_Init(&FrictionWheel_Speed[1],PID_POSITION,FrictionWheel_Speed_PID,150,10000,17999);
	Strumming_Speed.Param_Init(&Strumming_Speed,PID_POSITION,Strumming_Speed_PID,150,10000,17999);
	
	//温控bmi088相关参数和控制器初始化:
	PID_Init(&bmi088_param[0].Control_t.Pos);
	PID_CparameterUpdate(&bmi088_param[0].Control_t.Pos,1600.0f,0.2f,0);
	PID_IntegralLimitUpdate(&bmi088_param[0].Control_t.Pos,-4400.0f,4400.0f);
	
	//DM电机初始化:
	DM_MotorInit(&DM_Motor,0x01,8,DM_MotorCtrlMode_Mit);
	
	//姿态解算bmi088相关滤波器初始化:
	INS_MahonyInit();
	
//	//Yaw轴滑模控制器相关参数初始化:
//	SlidingMode_Param_Init(&Yaw_SMC);
	
	//can发送队列初始化:
	CAN1_Send_Queue_Init();
	CAN2_Send_Queue_Init();
	
//	//超电启动初始化:
	SCAP_Init();
	
	//功控相关参数初始化:
	ChassisPowerControl_ParamsInit();
	
//	//预测功率的初始化:
//	Power_Init(&powerMannage);
}
