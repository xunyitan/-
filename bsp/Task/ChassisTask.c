#include "ChassisTask.h"
#include "Powercontrol.h"
#include "PersonalUITask.h"
  
uint8_t djl_data[8];
float powerbuffer;
float chassislimit;
SlopeFilter_t SlopeFilter[4];
ChassisSlope_t ChassisSlope[4];
ChassisSlope_Param_t ChassisSlope_Param;
Lowpassfilter_t LowPassFilter[4];
Chassis_Remote Chassis_Remote_t;
Chassis_Following Chassis_Following_t;
Chassis_Gyro360 Chassis_Gyro360_t;
PidTypeDef Chassis_Speed[4];
PidTypeDef Chassis_Gyro_Speed[4];
PidTypeDef Turn_Middle[4];
float Chassis_Speed_PID[3]={1.2,0.91,1.6};
float Chassis_Gyro_Speed_PID[3]={1,0.2,0};
float Turn_Middle_PID[3]={1.3,0,0.8};//1.2
//未受到功控的计算输出:
float raw_pid_output[4];
float target_speed_rpm[4];
//功率分配:
float controlled_output[4];
float msms;

uint32_t Keyborad_moves=0;
uint32_t GimfolChassis_moves=0;

//测试,暂未定义任务执行周期,初步定义任务执行频率为100HZ 10ms
void ChassisTask_Test(void *argument)
{
	TickType_t xLastWakeTime=xTaskGetTickCount();
  const TickType_t xFrequency=pdMS_TO_TICKS(10);
	RC_Ctl_t my_RCCtl;
	//遥控器摇杆参数初始化,作用是电机保护;
	my_RCCtl.rc.ch2=RC_MIDDLE;
	my_RCCtl.rc.ch3=RC_MIDDLE;
	my_RCCtl.keep.che=RC_MIDDLE;
	while(1)
	{
		xQueueReceive(RC_MouseCC,(void*)&my_RCCtl,xFrequency);//从队列中读取到遥控器数据,当前等待队列十毫秒便有遥控器数据
		Chassis_Remote_t.Vx=(Chassis_Remote_t.X-RC_MIDDLE)*VX_GAINS;
		Chassis_Remote_t.Vy=(Chassis_Remote_t.Y-RC_MIDDLE)*VY_GAINS;
		Chassis_Remote_t.Vz=(Chassis_Remote_t.Z-RC_MIDDLE)*VZ_GAINS;
		powerbuffer=power_heat_data_t.chassis_power_buffer;//缓冲能量，连上服务器才有数据
//		Superpower_cap_Init(90,60,1,0);
//		Chassis_PowerControlTest(Chassis_Remote_t.Vx,Chassis_Remote_t.Vy,Chassis_Remote_t.Vz,18000);//旧功控，或许26赛季不再使用
		//底盘键鼠控制:
		if(my_RCCtl.key.b==1)
		{
			Moving_flag_t.Chassis_sidewayflag=0;
			Moving_flag_t.Chassis_flowingflag=0;
			Moving_flag_t.Gyro_360_motion_flag=1;
			Chassis_State=2;
		}
		if(my_RCCtl.key.v==1)
		{
			Moving_flag_t.Chassis_sidewayflag=0;
			Moving_flag_t.Chassis_flowingflag=1;
			Moving_flag_t.Gyro_360_motion_flag=0;
			Chassis_State=1;
		}
		if(my_RCCtl.key.c==1)
		{
			Superpower_cap_Init(90,60,1,1);
		}
		if(my_RCCtl.key.shift>0)
		{
			Keyborad_moves=600;
			GimfolChassis_moves=400;
		}
		else
		{
			Keyborad_moves=300;
			GimfolChassis_moves=210;
		}
		if(my_RCCtl.rc.s1==RC_S1_DOWN)//测试模式，测试遥控器控制电机
		{
			Chassis_Remote_t.X=my_RCCtl.rc.ch3+my_RCCtl.key.w*Keyborad_moves-my_RCCtl.key.s*Keyborad_moves;
			Chassis_Remote_t.Y=my_RCCtl.rc.ch2-my_RCCtl.key.a*Keyborad_moves+my_RCCtl.key.d*Keyborad_moves;
			Chassis_Remote_t.Z=my_RCCtl.keep.che;
			Chassis_sideway();//麦轮运动学解算
			for(uint8_t j=0;j<4;j++)
			{
				djl_3508[j].Control_t.Speed.P_Myset.Target=djl_3508[j].Control_t.Speed.P_Myset.Target;
				djl_3508[j].Control_t.Speed.P_Myset.Actual=(double)(djl_3508[j].Rx.Speed);
				djl_3508[j].Control_t.Speed.P_Myset.Out=Chassis_Speed[j].Cal_PID(&Chassis_Speed[j],djl_3508[j].Control_t.Speed.P_Myset.Actual,Chassis_Slope(400,djl_3508[j].Control_t.Speed.P_Myset.Target,djl_3508[j].Control_t.Speed.P_Myset.Actual));
				djl_3508[j].Tx.m_Speed=(short)(djl_3508[j].Control_t.Speed.P_Myset.Out);
			}
			//发给电调实现电流控制,按照协议发送
			djl_data[0] = djl_3508[0].Tx.m_Speed >> 8;
			djl_data[1] = djl_3508[0].Tx.m_Speed&0xFF;
			djl_data[2] = djl_3508[1].Tx.m_Speed >> 8;
			djl_data[3] = djl_3508[1].Tx.m_Speed&0xFF;
			djl_data[4] = djl_3508[2].Tx.m_Speed >> 8;
			djl_data[5] = djl_3508[2].Tx.m_Speed&0xFF;
			djl_data[6] = djl_3508[3].Tx.m_Speed >> 8;
			djl_data[7] = djl_3508[3].Tx.m_Speed&0xFF;
			CAN1_TO_Queue(0x200,sizeof(djl_data),djl_data);
		}
		if(my_RCCtl.rc.s1==RC_S1_UP||Moving_flag_t.Gyro_360_motion_flag==1)//小陀螺模式
		{
			Chassis_Remote_t.X=my_RCCtl.rc.ch3+my_RCCtl.key.w*Keyborad_moves-my_RCCtl.key.s*Keyborad_moves;
			Chassis_Remote_t.Y=my_RCCtl.rc.ch2-my_RCCtl.key.a*Keyborad_moves+my_RCCtl.key.d*Keyborad_moves;
			Chassis_Remote_t.Z=my_RCCtl.keep.che;
			Gyro_360_motion(2000,Level_Gyro);
			for(uint8_t i=0;i<4;i++)
			{
				djl_3508[i].Control_t.Speed.P_Myset.Target=djl_3508[i].Control_t.Speed.P_Myset.Target;
				djl_3508[i].Control_t.Speed.P_Myset.Actual=(double)(djl_3508[i].Rx.Speed);
				djl_3508[i].Control_t.Speed.P_Myset.Out=Chassis_Gyro_Speed[i].Cal_PID(&Chassis_Gyro_Speed[i],djl_3508[i].Control_t.Speed.P_Myset.Actual,djl_3508[i].Control_t.Speed.P_Myset.Target);
				djl_3508[i].Tx.m_Speed=(short)(djl_3508[i].Control_t.Speed.P_Myset.Out);
			}
			//发给电调实现电流控制,按照协议发送
			djl_data[0] = djl_3508[0].Tx.m_Speed >> 8;
			djl_data[1] = djl_3508[0].Tx.m_Speed&0xFF;
			djl_data[2] = djl_3508[1].Tx.m_Speed >> 8;
			djl_data[3] = djl_3508[1].Tx.m_Speed&0xFF;
			djl_data[4] = djl_3508[2].Tx.m_Speed >> 8;
			djl_data[5] = djl_3508[2].Tx.m_Speed&0xFF;
			djl_data[6] = djl_3508[3].Tx.m_Speed >> 8;
			djl_data[7] = djl_3508[3].Tx.m_Speed&0xFF;
			CAN1_TO_Queue(0x200,sizeof(djl_data),djl_data);
		}
		if(my_RCCtl.rc.s1==RC_S1_MIDDLE||Moving_flag_t.Chassis_flowingflag==1)//底盘跟随云台
		{
			Chassis_Remote_t.X=my_RCCtl.rc.ch3+my_RCCtl.key.w*GimfolChassis_moves-my_RCCtl.key.s*GimfolChassis_moves;
			Chassis_Remote_t.Y=my_RCCtl.rc.ch2-my_RCCtl.key.a*GimfolChassis_moves+my_RCCtl.key.d*GimfolChassis_moves;
			Chassis_Remote_t.Z=my_RCCtl.keep.che;
			ChassisFollowing_Gimbal(600,0);
			//原始速度:
			for(uint8_t i=0;i<4;i++)
			{
			  djl_3508[i].Control_t.Speed.P_Myset.Target=djl_3508[i].Control_t.Speed.P_Myset.Target;
				djl_3508[i].Control_t.Speed.P_Myset.Actual=(double)(djl_3508[i].Rx.Speed);
				djl_3508[i].Control_t.Speed.P_Myset.Out=Chassis_Speed[i].Cal_PID(&Chassis_Speed[i],djl_3508[i].Control_t.Speed.P_Myset.Actual,Chassis_Slope(1000,djl_3508[i].Control_t.Speed.P_Myset.Target,djl_3508[i].Control_t.Speed.P_Myset.Actual));//得用坡度化，不然超功率
				djl_3508[i].Tx.m_Speed=(short)(djl_3508[i].Control_t.Speed.P_Myset.Out);
			}
			//发给电调实现电流控制,按照协议发送
			djl_data[0] = djl_3508[0].Tx.m_Speed >> 8;
			djl_data[1] = djl_3508[0].Tx.m_Speed&0xFF;
			djl_data[2] = djl_3508[1].Tx.m_Speed >> 8;
			djl_data[3] = djl_3508[1].Tx.m_Speed&0xFF;
			djl_data[4] = djl_3508[2].Tx.m_Speed >> 8;
			djl_data[5] = djl_3508[2].Tx.m_Speed&0xFF;
			djl_data[6] = djl_3508[3].Tx.m_Speed >> 8;
			djl_data[7] = djl_3508[3].Tx.m_Speed&0xFF;
			CAN1_TO_Queue(0x200,sizeof(djl_data),djl_data);
		}
		if(RC_Protection<5)
		{
			for(uint8_t i=0;i<4;i++)
			{
				djl_3508[i].Tx.m_Speed=0;
			}
			uint8_t djl_data[8]={djl_3508[0].Tx.m_Speed >> 8,
			djl_3508[0].Tx.m_Speed&0xFF,
			djl_3508[1].Tx.m_Speed >> 8,
			djl_3508[1].Tx.m_Speed&0xFF,
			djl_3508[2].Tx.m_Speed >> 8,
			djl_3508[2].Tx.m_Speed&0xFF,
			djl_3508[3].Tx.m_Speed >> 8,
			djl_3508[3].Tx.m_Speed&0xFF};
			CAN1_TO_Queue(0x200,sizeof(djl_data),djl_data);
			Moving_flag_t.Chassis_sidewayflag=0;
			Moving_flag_t.Chassis_flowingflag=0;
			Moving_flag_t.Gyro_360_motion_flag=0;
		}
//		usart_printf("%d,%d,%f\r\n",SCAPrx.CapEnergy,SCAPrx.errorCode,SCAPrx.chassisPower);//250 0/136,128
//		usart_printf("%f,%f,%f,%d\r\n",djl_3508[3].Control_t.Speed.P_Myset.Target,djl_3508[3].Control_t.Speed.P_Myset.Actual,djl_3508[3].Control_t.Speed.P_Myset.Out,djl_3508[3].Tx.m_Speed);//打印数据查看响应情况,要求阶跃但不超调,并且停下时过阻尼
		vTaskDelayUntil(&xLastWakeTime,xFrequency);
	}
}

void Chassis_sideway(void)
{
	djl_3508[0].Control_t.Speed.P_Myset.Target=(double)((Chassis_Remote_t.X-RC_MIDDLE)*VX_GAINS-(Chassis_Remote_t.Y-RC_MIDDLE)*VY_GAINS-(Chassis_Remote_t.Z-RC_MIDDLE)*0);
	djl_3508[1].Control_t.Speed.P_Myset.Target=(double)(-(Chassis_Remote_t.X-RC_MIDDLE)*VX_GAINS-(Chassis_Remote_t.Y-RC_MIDDLE)*VY_GAINS-(Chassis_Remote_t.Z-RC_MIDDLE)*0);
	djl_3508[2].Control_t.Speed.P_Myset.Target=(double)(-(Chassis_Remote_t.X-RC_MIDDLE)*VX_GAINS+(Chassis_Remote_t.Y-RC_MIDDLE)*VY_GAINS-(Chassis_Remote_t.Z-RC_MIDDLE)*0);
	djl_3508[3].Control_t.Speed.P_Myset.Target=(double)((Chassis_Remote_t.X-RC_MIDDLE)*VX_GAINS+(Chassis_Remote_t.Y-RC_MIDDLE)*VY_GAINS-(Chassis_Remote_t.Z-RC_MIDDLE)*0);
}

void ChassisFollowing_Gimbal(int Middle,short way_limit)
{
	float gains;
	gains=1;
	Chassis_Remote_t.scaled_X=(Chassis_Remote_t.X-RC_MIDDLE)*gains;
	Chassis_Remote_t.scaled_Y=(Chassis_Remote_t.Y-RC_MIDDLE)*gains;
	Chassis_Remote_t.scaled_Z=(Chassis_Remote_t.Z-RC_MIDDLE)*gains;
	Chassis_Following_t.way_chart_a=way_limit;
	//计算偏差角:
	if(Chassis_Following_t.way_selet==0)
	{
		Chassis_Following_t.Middle_location=Middle;
	}
	else
	{
		Chassis_Following_t.Middle_location=Middle-DJL_MOTOR_6020_PosHalf;
	}
	if(Chassis_Following_t.Middle_location>DJL_MOTOR_6020_PosMax)
	{
		Chassis_Following_t.Middle_location=Chassis_Following_t.Middle_location-DJL_MOTOR_6020_PosMax;
	}
	else if(Chassis_Following_t.Middle_location<0)
	{
		Chassis_Following_t.Middle_location=Chassis_Following_t.Middle_location+DJL_MOTOR_6020_PosMax;
	}
	djl_6020[1].Rx.Pos=DJL_MOTOR_6020_PosMax-djl_6020[1].Rx.Pos;
	
	if(Chassis_Following_t.Middle_location>=djl_6020[1].Rx.Pos)
	{
		if((Chassis_Following_t.Middle_location-djl_6020[1].Rx.Pos)>=DJL_MOTOR_6020_PosHalf)
		{
			Chassis_Following_t.Middle_err=-djl_6020[1].Rx.Pos+(Chassis_Following_t.Middle_location-DJL_MOTOR_6020_PosMax);
		}
		else if((Chassis_Following_t.Middle_location-djl_6020[1].Rx.Pos)<DJL_MOTOR_6020_PosHalf)
		{
			Chassis_Following_t.Middle_err=(Chassis_Following_t.Middle_location-djl_6020[1].Rx.Pos);
		}
	}
	else if(Chassis_Following_t.Middle_location<djl_6020[1].Rx.Pos)
	{
		if((Chassis_Following_t.Middle_location-djl_6020[1].Rx.Pos) >= -DJL_MOTOR_6020_PosHalf)
		{
			Chassis_Following_t.Middle_err=-(djl_6020[1].Rx.Pos - Chassis_Following_t.Middle_location);
		}	
		else if((Chassis_Following_t.Middle_location-djl_6020[1].Rx.Pos) < -DJL_MOTOR_6020_PosHalf)
		{
			Chassis_Following_t.Middle_err=(DJL_MOTOR_6020_PosMax - djl_6020[1].Rx.Pos + Chassis_Following_t.Middle_location);
		}			
	}
	
	if(Chassis_Following_t.way_chart_a==1)
	{
		if(Chassis_Following_t.Middle_err>DJL_MOTOR_6020_PosQuarter||Chassis_Following_t.Middle_err<-DJL_MOTOR_6020_PosQuarter)
		{
			Chassis_Following_t.way_selet=!Chassis_Following_t.way_selet;
		}
	}
	else
	{
		Chassis_Following_t.way_selet=0;
	}
	
	//回中速度限幅:
	if(Chassis_Following_t.Middle_err>Middle_Speed_Limit)
	{
		Chassis_Following_t.Middle_err=Middle_Speed_Limit;
	}
	if(Chassis_Following_t.Middle_err<-Middle_Speed_Limit)
	{
		Chassis_Following_t.Middle_err=-Middle_Speed_Limit;
	}
	
	//遥控器控制:
	if(Chassis_Following_t.way_selet==0)
	{
		djl_3508[0].Control_t.Speed.P_Myset.Target=(double)((Chassis_Remote_t.scaled_X) *VX_GAINS-(Chassis_Remote_t.scaled_Y)*VY_GAINS-(Chassis_Remote_t.scaled_Z)*0);
		djl_3508[1].Control_t.Speed.P_Myset.Target=(double)(-(Chassis_Remote_t.scaled_X) *VX_GAINS-(Chassis_Remote_t.scaled_Y)*VY_GAINS-(Chassis_Remote_t.scaled_Z)*0);
		djl_3508[2].Control_t.Speed.P_Myset.Target=(double)(-(Chassis_Remote_t.scaled_X) *VX_GAINS+(Chassis_Remote_t.scaled_Y)*VY_GAINS-(Chassis_Remote_t.scaled_Z)*0);
		djl_3508[3].Control_t.Speed.P_Myset.Target=(double)((Chassis_Remote_t.scaled_X) *VX_GAINS+(Chassis_Remote_t.scaled_Y)*VY_GAINS-(Chassis_Remote_t.scaled_Z)*0);
	}
	else
	{
		djl_3508[0].Control_t.Speed.P_Myset.Target=(double)(-(Chassis_Remote_t.scaled_X) *VX_GAINS+(Chassis_Remote_t.scaled_Y)*VY_GAINS-(Chassis_Remote_t.scaled_Z)*0);
		djl_3508[1].Control_t.Speed.P_Myset.Target=(double)((Chassis_Remote_t.scaled_X) *VX_GAINS+(Chassis_Remote_t.scaled_Y)*VY_GAINS-(Chassis_Remote_t.scaled_Z)*0);
		djl_3508[2].Control_t.Speed.P_Myset.Target=(double)((Chassis_Remote_t.scaled_X) *VX_GAINS-(Chassis_Remote_t.scaled_Y)*VY_GAINS-(Chassis_Remote_t.scaled_Z)*0);
		djl_3508[3].Control_t.Speed.P_Myset.Target=(double)(-(Chassis_Remote_t.scaled_X) *VX_GAINS-(Chassis_Remote_t.scaled_Y)*VY_GAINS-(Chassis_Remote_t.scaled_Z)*0);
	}
	
	//回中pid计算:
	for(uint8_t i=0;i<4;i++)
	{
		djl_3508[i].Control_t.Speed.P_Myset.Target=djl_3508[i].Control_t.Speed.P_Myset.Target+Turn_Middle[i].Cal_PID(&Turn_Middle[i],0,0);
	}
}

void Gyro_360_motion(int Center_adjustment_value,float target)
{
	float gains,zgains;
	gains=PC.V_Gains;
	zgains=PC.Z_Gains;
	Chassis_Remote_t.scaled_X=(Chassis_Remote_t.X-RC_MIDDLE)*gains;
	Chassis_Remote_t.scaled_Y=(Chassis_Remote_t.Y-RC_MIDDLE)*gains;
	//6020角度处理,限定在0-8191:
	djl_6020[1].Rx.Pos=djl_6020[1].Rx.Pos+Center_adjustment_value;
	if(djl_6020[1].Rx.Pos>DJL_MOTOR_6020_PosMax)
	{
		djl_6020[1].Rx.Pos=djl_6020[1].Rx.Pos-DJL_MOTOR_6020_PosMax;
	}
	Chassis_Gyro360_t.Chassis_360_speed_target=target;
	//计算世界角度:
	Chassis_Gyro360_t.Bittop_Angle=PI*((DJL_MOTOR_6020_PosMax-djl_6020[1].Rx.Pos)/22.755)/180;
	Chassis_Gyro360_t.Bittop_X=(-(Chassis_Remote_t.scaled_X) * sin(Chassis_Gyro360_t.Bittop_Angle)) -((Chassis_Remote_t.scaled_Y) * cos(Chassis_Gyro360_t.Bittop_Angle));
	Chassis_Gyro360_t.Bittop_Y=(-(Chassis_Remote_t.scaled_X) * cos(Chassis_Gyro360_t.Bittop_Angle)) +((Chassis_Remote_t.scaled_Y) * sin(Chassis_Gyro360_t.Bittop_Angle));
	//放到目标速度:
	djl_3508[0].Control_t.Speed.P_Myset.Target=(Chassis_Gyro360_t.Bittop_X) *10 + (Chassis_Gyro360_t.Bittop_Y)*10 + Chassis_Gyro360_t.Chassis_360_speed_target*10*zgains;
	djl_3508[1].Control_t.Speed.P_Myset.Target=(-Chassis_Gyro360_t.Bittop_X) *10 + (Chassis_Gyro360_t.Bittop_Y)*10 + Chassis_Gyro360_t.Chassis_360_speed_target*10*zgains;
	djl_3508[2].Control_t.Speed.P_Myset.Target=(-Chassis_Gyro360_t.Bittop_X) *10 - (Chassis_Gyro360_t.Bittop_Y)*10 + Chassis_Gyro360_t.Chassis_360_speed_target*10*zgains;
	djl_3508[3].Control_t.Speed.P_Myset.Target=(Chassis_Gyro360_t.Bittop_X) *10 - (Chassis_Gyro360_t.Bittop_Y)*10 + Chassis_Gyro360_t.Chassis_360_speed_target*10*zgains;
}

float Chassis_Slope(int len,float target_set,float value)
{
	int M3508_len;
	float M3508_erro,M3508_value,M3508_target,M3508_target_set;
	M3508_len = len;
	M3508_target_set = target_set;
	M3508_value = value;
	M3508_erro = M3508_target_set - M3508_value;
		
	if(M3508_erro > 0)
	{
	if(M3508_erro > M3508_len)
		{
			M3508_target = M3508_value + M3508_len;
		}
	else
		{
			M3508_target = M3508_value + M3508_erro;
		}		
	}
	else
	{   
		if(M3508_erro < -M3508_len)
		 {
			 M3508_target = M3508_value - M3508_len;
		 }
		else
		 {
			M3508_target = M3508_value + M3508_erro;
		 }
	}
	return M3508_target;
}


