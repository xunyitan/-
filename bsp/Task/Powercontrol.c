//#include "Powercontrol.h"
//#include "RLS.h"
//#include "stdbool.h"

////当前功率模型:P=τΩ+k1|Ω|+k2τ^2;
///*
//P = τΩ + k1|Ω| + k2τ2 + k3
//其中：
//- τ: 电机转矩
//- Ω: 电机角速度  
//- k1: 速度相关损耗系数
//- k2: 转矩平方损耗系数
//- k3: 恒定损耗
//*/

//power_t powerMannage;
//power_pidt Energy_PID;
//static power_pid_t powerPD_base;  // 基础能量PID
//static power_pid_t powerPD_full;  // 满能量PID
//motor_data_t motor_data[4];
//uint8_t g_Init=0;//使能标志，后续可以换成预编译
//float cmdPower[4]={0};//四个电机命令需求数组

//void PowerParam_Init(power_t *power,float k1,float k2,float k3)
//{
//	if(!power)
//		return;
//	
//	//初始化功率损耗系数
//	power->k1=k1;
//	power->k2=k2;
//	power->k3=k3;
//	
//	// 初始化错误标志
//	power->error = 0;
//	
//	// 初始化功率限制参数
//	power->powerUpperLimit = 0;
//	power->refereeMaxPower = 90.0f;//裁判系统允许最大功率
//	power->userConfiguredMaxPower = 120.0f;//当前我设置为60W底盘
//	
//	// 初始化能量缓冲设定点
//	power->fullBuffSet = 60.0f;  // 裁判系统满缓冲 要结合裁判系统数据
//	power->baseBuffSet = 30.0f;  // 裁判系统基础缓冲
//	
////	// 初始化双PID
////	energyPID_Init(&powerPD_base, POWER_PD_KP, POWER_PD_KD, 50);
////	energyPID_Init(&powerPD_full, POWER_PD_KP, POWER_PD_KD, 50);
//	
//	// 初始化测量值
//	power->measuredPower = 0.0f;
//	power->estimatedPower = 0.0f;
//	power->estimatedCapEnergy = 0.0f;
//	power->powerBuffer = 60.0f;  // 初始化为满缓冲
//	
//	Energy_PID.Kp=1.2;
//	Energy_PID.Ki=0.01;
//	Energy_PID.Kd=0.3;
//	
//	Energy_PID.energy_control_enabled=1;//开启能量环PID控制
//	
//	power->rls_enabled=1;//rls使能
//	
//}

////初始化功率控制器:
////初始化功率控制器:
//void Power_Init(power_t *power)
//{
//	if(!power)
//			return;
//	
//	PowerParam_Init(power, 0.22f, 1.2f, 2.78f);
//	
//	// 初始化RLS估计器参数
//	float init_params[2] = {power->k1, power->k2};
//	
//	// 确保RLS_t已正确初始化
//	RLS_Init(&RLS_t,1e-5f,0.99999f,2);  //接近1的lambda 1e-5f
//	
//	// 设置初始参数
//	Matrixf initParamVec;
//	float initParamData[2] = {power->k1, power->k2};
//	initParamVec.rows = 2;
//	initParamVec.cols = 1;
//	initParamVec.data = initParamData;
//	arm_mat_init_f32(&initParamVec.arm_mat_, 2, 1, initParamData);
//	
//	RLS_setParamVector(&RLS_t, &initParamVec);
//	
//	g_Init = 1; // 使能
//}

//void energyPID_Init(power_pid_t *pid,float kp,float kd,float output_limit)
//{
//	pid->kp=kp;
//	pid->kd=kd;
//	pid->output_limit=output_limit;
//	pid->last_error=0;
//	pid->output=0;
//}

////能量环PD计算:
//float pid_calc(power_pid_t *pid, float target, float actual) 
//{
//	float error = target - actual;
//	pid->output = pid->kp * error + pid->kd * (error - pid->last_error);
//	pid->output = Limit(pid->output, -pid->output_limit, pid->output_limit);
//	pid->last_error = error;
//	return pid->output;
//}

//void power_set_max_power(power_t *power,float maxPower)
//{
//	//钳位处理，确保在合适的范围内:
//	if(maxPower<minPower)
//	{
//		maxPower=minPower;
//	}
//	if(maxPower>power->powerUpperLimit)
//	{
//		maxPower=power->powerUpperLimit;
//	}
//	power->userConfiguredMaxPower=maxPower;
//}

////设置工作模式:
//void power_set_mode(power_t *power,uint8_t mode)
//{
//	if(!power)  
//		return;
//	
//	if(mode==1)
//	{
//		power_set_max_power(power,power->powerUpperLimit);
//	}
//	else
//	{
//		power_set_max_power(power,power->refereeMaxPower);
//	}
//}

////功率分配算法(放于底盘)
//void power_get_controlled_output(power_t *power,PowerObj *objs[4],float output[4])
//{
//	if (!power||!objs||!output)
//		return;
//	
//	//扭矩常数:DJL_MOTOR_3508_Kt 0.3
//	
//	float sumpower=0;//四个电机总命令功率需求
//	float sumErorr=0;//四个电机总跟踪误差
//	float error[4]={0};//每个电机跟踪误差数组
//	
//	//计算最大功率限制:
//	float maxPower=power->userConfiguredMaxPower;//60*4=240 SCAPrx.chassisPowerLimit 当前为50w底盘 当前超电有电时拉高限制功率，没电时降低限制功率
//	float allocatablePower=maxPower;//可分配功率
//	float sumPowerRequired=0;//需要功率分配的总功率
//	
//	//1.计算每个电机的命令功率和跟踪误差
//	for(uint8_t i=0;i<4;i++)
//	{
//		if(objs[i]!=NULL)
//		{
//			PowerObj *p=objs[i];
//			//功率估计模型:
//			cmdPower[i]=p->pidOutput*DJL_MOTOR_3508_Kt*p->curAv+
//			fabs(p->curAv)*power->k1+p->pidOutput*DJL_MOTOR_3508_Kt*p->pidOutput*DJL_MOTOR_3508_Kt*power->k2+
//			power->k3/4.0f;
//			
//			sumpower+=cmdPower[i];
//			error[i]=fabs(p->setAv-p->curAv);
//			
//			//计算可分配功率和所需功率:
//			if(float_equal(cmdPower[i],0.0f)||cmdPower[i]<0.0f)
//			{
//				allocatablePower+=fabs(cmdPower[i]);
//			}
//			else
//			{
//				sumErorr+=error[i];
//				sumPowerRequired+=cmdPower[i];
//			}
//		}
//		else
//		{
//			cmdPower[i]=0;
//			error[i]=0;
//		}
//	}
//	
//	if(sumPowerRequired<1e-5f) 
//	{  
//		sumPowerRequired=1e-5f;// 避免除零
//	} 
//	
//	//2.功率分配策略:
//	if(sumpower>maxPower)//四轮功率大于最大功率时,进入功率分配模式
//	{
//		//计算误差置信度:
//		float errorConfidence;
//		if(sumErorr>ERROR_POWER_DISTRIBUTION_SET)
//		{
//			errorConfidence=1;
//		}
//		else if(sumErorr>PROP_POWER_DISTRIBUTION_SET)
//		{
//			errorConfidence=(sumErorr-PROP_POWER_DISTRIBUTION_SET)/
//			(ERROR_POWER_DISTRIBUTION_SET-PROP_POWER_DISTRIBUTION_SET);
//			errorConfidence=Limit(errorConfidence,0,1);
//		}
//		else
//		{
//			errorConfidence=0;
//		}
//		
//		//对每个电机进行功率分配:
//		for(uint8_t i=0;i<4;i++)
//		{
//			if(objs[i]!=NULL)
//			{
//				PowerObj *p=objs[i];
//				
//				//对于零功率或负功率需求,直接使用PID输出
//				if (float_equal(cmdPower[i], 0.0f) || cmdPower[i] < 0.0f)  
//				{
//					output[i] = p->pidOutput;
//					continue;
//				}
//				
//				// 计算功率权重：基于误差和功率比例的加权
//				float powerWeight_Error = error[i] / sumErorr;
//				float powerWeight_Prop = cmdPower[i] / sumPowerRequired;
//				float powerWeight = errorConfidence * powerWeight_Error + 
//													(1.0f - errorConfidence) * powerWeight_Prop;
//				
//				// 解功率分配二次方程: k2*(k0*i)2 + curAv*(k0*i) + (k1*|curAv| + k3/4 - weight*allocPower) = 0
//				float delta = p->curAv * p->curAv - 4.0f * power->k2 * 
//										(power->k1 * fabsf(p->curAv) + power->k3 / 4.0f - 
//										 powerWeight * allocatablePower);
//				
//				// 根据判别式求解
//				if (float_equal(delta, 0.0f)) 
//				{
//					// 重根情况
//					output[i] = -p->curAv / (2.0f * power->k2) / DJL_MOTOR_3508_Kt;
//				} 
//				else if (delta > 0.0f) 
//				{
//					// 两个实根，根据PID输出符号选择
//					if (p->pidOutput > 0.0f) 
//					{
//						output[i] = (-p->curAv + sqrtf(delta)) / (2.0f * power->k2) / DJL_MOTOR_3508_Kt;
//					} 
//					else 
//					{
//						output[i] = (-p->curAv - sqrtf(delta)) / (2.0f * power->k2) / DJL_MOTOR_3508_Kt;
//					}
//				} 
//				else 
//				{
//					// 虚根情况，使用近似解
//					output[i] = -p->curAv / (2.0f * power->k2) / DJL_MOTOR_3508_Kt;
//				}
//				
//				// 输出钳位，确保在合理范围内
//				output[i] = Limit(output[i], -p->pidMaxOutput, p->pidMaxOutput);
//				} 
//				else 
//				{
//					output[i] = 0.0f;
//				}
//				
//			}
//		}
//		else
//		{
//			// 功率充足，直接使用PID输出
//			for (int i = 0; i < 4; i++) 
//			{
//				if (objs[i] != NULL) 
//				{
//					output[i] = objs[i]->pidOutput;
////					usart_printf("%f\r\n",output[0]); 
//				}   
//				else 
//				{
//					output[i] = 0.0f;
//				}
//			}
//		}
//}

////设置裁判系统数据:(当前设置不了)
//void power_set_referee_data(power_t *power,float buffer_energy,float chassis_power,uint8_t robot_level)
//{
//	if(!power)
//		return;
//	
//	power->powerBuffer=buffer_energy;//缓冲能量
//	power->measuredPower=chassis_power;//测量功率
//	
//	//根据机器人等级进行功率限制:(对抗赛需要，联盟赛直接等级拉满，测试需要)
//	
//}

//void power_set_chassis_power_limit(power_t *power, float power_limit_w)
//{
//	if(!power) return;
//	
//	// 安全限制
//	power_limit_w = Limit(power_limit_w, 30.0f, 150.0f);
//	
//	// 设置用户配置功率
//	power->userConfiguredMaxPower = power_limit_w;
//	
//	// 同时更新相关功率限制，确保一致性
//	power->powerUpperLimit = power_limit_w;
//	power->baseMaxPower = power_limit_w * 0.8f;
//	power->fullMaxPower = power_limit_w;
//}

////设置电容组数据:
//void power_set_capacitor_data(power_t *power, float cap_energy, float chassis_power, uint8_t is_connected)
//{
//	if(!power)
//		return;
//	
//	if(power&&is_connected==1)
//	{
//		power->estimatedCapEnergy=cap_energy;//设置电容组缓冲能量，c板发给超电
//		power->measuredPower=chassis_power;//设置超电的功率控制，c板发给超电
//	}
//	
//}

////获取功率状态:   
//power_status_t power_get_status(power_t *power)
//{
//	power_status_t status = {0};
//	
//	if(power) {
//		status.userConfiguredMaxPower = power->userConfiguredMaxPower;
//		status.effectivePower = power->estimatedPower - power->k3; // 简化的有效功率
//		status.powerLoss = power->measuredPower - status.effectivePower;
//		status.efficiency = (power->measuredPower > 0) ? 
//						   status.effectivePower / power->measuredPower : 0;
//		status.estimatedCapEnergy = (uint8_t)(SCAPrx.CapEnergy / 2100.0f * 255.0f);
//		status.error = power->error;
//	}
//	
//	return status;
//}

//void Energy_PIDcontrol_loop(power_pidt *pid, float target, float actual)
//{
//	if(!pid || !pid->energy_control_enabled)
//			return;
//	
//	// 计算当前误差
//	pid->error = target - actual;
//	
//	// 计算积分项（带积分限幅）
//	pid->integral += pid->error;
//	if(pid->integral > pid->target_cap) {
//			pid->integral = pid->target_cap;
//	} else if(pid->integral < pid->target_nocap) {
//			pid->integral = pid->target_nocap;
//	}
//	
//	// 计算微分项
//	pid->derivative = pid->error - pid->lasterror;
//	pid->lasterror = pid->error;
//	
//	// PID输出计算
//	pid->out = pid->Kp * pid->error + 
//						 pid->Ki * pid->integral + 
//						 pid->Kd * pid->derivative;
//	
//	// 输出限幅
//	if(pid->out > pid->target_cap) {
//			pid->out = pid->target_cap;
//	} else if(pid->out < pid->target_nocap) {
//			pid->out = pid->target_nocap;
//	}
//	
//	pid->actual = actual;
//	pid->target = target;
//}

////更新能量缓冲:
//void power_update_energy_buffer(power_t *power)
//{
//	if(!power)
//		return;
//	
//	power->powerBuffer=SCAPrx.CapEnergy;
//	power->fullBuffSet=240;//电容组满能量设定点
//	power->baseBuffSet=30; //电容组基础能量设定点
//	
//}

////更新功率限制:
//void power_update_power_limits(power_t *power)
//{
//	if(!power)
//		return;
//	
//	if(SCAPrx.CapEnergy>=100)
//	{
//		power->cap_elc_ok=1;
//	}
//	if(SCAPrx.CapEnergy<100)
//	{
//		power->cap_elc_ok=0;
//	}
//	
//	float basePower_limit=90;
//	
//	if(power->cap_elc_ok==1)
//	{
//		basePower_limit+=10;
//	}
//	
//	if(power->cap_elc_ok!=1)
//	{
//		basePower_limit=basePower_limit;
//	}
//	
//	if(Energy_PID.energy_control_enabled==1)
//	{
//		if(power->cap_elc_ok==1)
//		{
//			Energy_PID.target=power->fullBuffSet*0.8;
//			Energy_PID.actual=SCAPrx.CapEnergy/255.0f*2100;
//		}
//		else
//		{
//			Energy_PID.target=power->baseBuffSet;
//			Energy_PID.actual=power->powerBuffer;
//		}
//		Energy_PID.error = Energy_PID.actual - Energy_PID.target;
//		Energy_PID.integral += Energy_PID.error;
//		Energy_PID.derivative = Energy_PID.error - Energy_PID.lasterror;
//		Energy_PID.integral = Limit(Energy_PID.integral, -1000.0f, 1000.0f);
//		float pid_output = Energy_PID.Kp * Energy_PID.error + 
//											Energy_PID.Ki * Energy_PID.integral + 
//											Energy_PID.Kd * Energy_PID.derivative;
//		pid_output = Limit(pid_output, -30.0f, 30.0f);
//		Energy_PID.lasterror = Energy_PID.error;
//		basePower_limit += pid_output;
//	}
//	basePower_limit=Limit(basePower_limit,30,150);
//	//更新功率限制
//	power->powerUpperLimit = basePower_limit;
//	power->baseMaxPower = basePower_limit * 0.8f;
//	power->fullMaxPower = basePower_limit;
//	power->userConfiguredMaxPower=140;//设置为60W
////	power->userConfiguredMaxPower = Limit(power->userConfiguredMaxPower, 
////																			 power->baseMaxPower, power->fullMaxPower);
//}

//void update_motor_data(void)
//{
//	for(uint8_t i = 0; i < 4; i++) 
//	{
//		motor_data[i].torque_Nm = (djl_3508[i].Rx.Torque/16384.0f) * DJL_MOTOR_3508_Kt;
//		motor_data[i].speed_rpm = djl_3508[i].Rx.Speed;
//		motor_data[i].speed_radps = rpm_to_rads(motor_data[i].speed_rpm);
//	}
//}

////更新估计功率:
//void power_update_power_estimation(power_t *power)
//{
//	if(!power) return;
//	
//	update_motor_data();
//	
//	float total_effective_power = 0;
//	float total_speed_related = 0;
//	float total_torque_squared = 0;
//	
//	for(uint8_t i = 0; i < 4; i++)
//	{
//		float torque_Nm = motor_data[i].torque_Nm;  // 单位：N·m
//		
//		float speed_radps = motor_data[i].speed_radps;  // 单位：rad/s
//		
//		//有效功率:
//		total_effective_power+=torque_Nm*speed_radps;
//		
//		//速度相关损耗项:
//		total_speed_related+=fabs(speed_radps);
//		
//		//转矩平方损耗:
//		total_torque_squared+=torque_Nm*torque_Nm;
//		
//	}
//	
//	// 总功率估计（单位：W）
//	power->estimatedPower = total_effective_power + 
//												 power->k1 * total_speed_related +
//												 power->k2 * total_torque_squared + 
//												 power->k3;
//	
////	usart_printf("%.1f,%f\r\n",total_effective_power,SCAPrx.chassisPower);
//}

////更新测量功率:
//void power_update_measured_power(power_t *power)
//{
//	if(!power)
//		return;
////	
////	if(SCAPrx.CapEnergy>100.0f)
////	{
//	power->measuredPower=SCAPrx.chassisPower;//来自超电数据
////	}
//	
////	else
////	{
////		power->measuredPower=power->estimatedPower;//来自测量功率
////	}
//	
//}

////更新RLS参数:
//void power_update_rls_parameters(power_t *power)
//{
//	if(!power)
//			return;
//	
//	if(power->rls_enabled && fabs(power->measuredPower)>10.0f && fabs(power->estimatedPower-power->measuredPower)>5 && power->estimatedPower>5) 
//	{
//		// 准备样本数据: [总速度相关项, 总转矩平方项]
//		float samples_data[2] = {0};
//		Matrixf sampleVector;
//		sampleVector.rows = 2;
//		sampleVector.cols = 1;
//		sampleVector.data = samples_data;
//		
//		for(uint8_t i = 0; i < 4; i++)
//		{
//			if(motor_data[i].speed_radps != 0) { // 避免除零
//					samples_data[0] += fabsf(motor_data[i].speed_radps);  // 总速度绝对值
//					samples_data[1] += motor_data[i].torque_Nm * motor_data[i].torque_Nm;  // 总转矩平方
//			}
//		}
//		
//		// 初始化ARM矩阵结构
//		arm_mat_init_f32(&sampleVector.arm_mat_, 2, 1, samples_data);
//		
//		// measuredPower - effectivePower - k3
//		float effectivePower = 0;
//		for(uint8_t i = 0; i < 4; i++) {
//			effectivePower += motor_data[i].torque_Nm * motor_data[i].speed_radps;
//		}
//		float power_loss = power->measuredPower - effectivePower - power->k3;
//		
//		// 准备结果矩阵
//		Matrixf result;
//		float result_data[2] = {0};
//		result.rows = 2;
//		result.cols = 1;
//		result.data = result_data;
//		arm_mat_init_f32(&result.arm_mat_, 2, 1, result_data);
//		
//		// 安全地更新RLS参数
//		if(samples_data[0] > 1 || samples_data[1] > 0.05f)  // 避免除零和噪声
//		{
//			RLS_update(&RLS_t, &sampleVector, power_loss, &result);
//			
//			// 获取新参数并应用限制 - 与C++代码一致
//			const float *new_params = RLS_GetParamsVector(&RLS_t);
//			if(new_params) 
//			{
//				// 直接赋值，不进行平滑（与C++代码一致）
//				power->k1 = fmaxf(new_params[0], 1e-5f);  // 防止发散到负值
//				power->k2 = fmaxf(new_params[1], 1e-5f);
//				
//				power->k1 = fminf(fmaxf(power->k1, 0.05f), 3.0f);
//				power->k2 = fminf(fmaxf(power->k2, 0.1), 2);
//			}
//		}
//	}
//}

//void Energy_PIDcontrol(void)
//{
//	if(Energy_PID.energy_control_enabled!=1)
//	{
//		return;
//	}
//	
//	Energy_PID.target=powerMannage.cap_elc_ok?Energy_PID.target_cap:Energy_PID.target_nocap;
//	
//	if(powerMannage.cap_elc_ok==1)
//	{
//		Energy_PID.actual=SCAPrx.CapEnergy/255.0f*2100.0f;
//	}
//	
//	else
//	{
//		Energy_PID.actual=60;
//	}
//	
//	Energy_PID.error = Energy_PID.actual - Energy_PID.target;  // 实际值 - 目标值
//	
//	Energy_PID.integral += Energy_PID.error;
//	if(Energy_PID.integral > 1000.0f)
//			Energy_PID.integral = 1000.0f;
//	if(Energy_PID.integral < -1000.0f)
//			Energy_PID.integral = -1000.0f;
//	
//	Energy_PID.derivative = Energy_PID.error - Energy_PID.lasterror;
//	Energy_PID.lasterror = Energy_PID.error;
//	
//	Energy_PID.out = Energy_PID.Kp * Energy_PID.error + 
//									 Energy_PID.Ki * Energy_PID.integral + 
//									 Energy_PID.Kd * Energy_PID.derivative;
//	
//	// 输出限幅
//	if(Energy_PID.out>30.0f) Energy_PID.out=30.0f;
//	if(Energy_PID.out<-30.0f) Energy_PID.out=-30.0f;
//	
//	// 应用能量环控制
//	powerMannage.powerUpperLimit+=Energy_PID.out;
//	
//	// 确保功率限制在合理范围内
//	if(powerMannage.powerUpperLimit<10.0f) powerMannage.powerUpperLimit=10.0f;
//}

//void powerControl_IN_Chassis(void)
//{
//	
//	
//}

//float float_equal(float a,float b)
//{
//	return fabsf(a-b)<1e-5f;
//}

//float rpm_to_rads(float rpm)
//{
//	return rpm*((2*PI)/60.0f);
//}

//float rpm_to_mps_simple(float rpm)
//{
//	float motor_rad_per_sec = rpm_to_rads(rpm);
//	float wheel_rad_per_sec = motor_rad_per_sec / DJI_MOTOR_3508_REDUCTION_RATIO;
//	return wheel_rad_per_sec * wheel_radius;
//}

////功率守护任务:(只负责检测功率)
//void powerdaemon_task(void *argument)
//{
//	TickType_t xLastWakeTime=xTaskGetTickCount();
//	const TickType_t xFrequency=pdMS_TO_TICKS(1);
//	while(1)
//	{
//		update_motor_data();
//		
//		//能量缓冲和功率限制更新:
//		power_update_energy_buffer(&powerMannage);
//		
//		//功率限制计算
//		power_update_power_limits(&powerMannage);//待完善
//		
//		//功率估计:
//		power_update_power_estimation(&powerMannage);
//		
//		//更新测量功率:
//		power_update_measured_power(&powerMannage);
//		
//		//更新RLS参数:
//		power_update_rls_parameters(&powerMannage);

//		vTaskDelayUntil(&xLastWakeTime,xFrequency);
//	}
//}

#include "Powercontrol.h"

power_t PC;
power_pid Energy_PID;
static float Chassis_Energy=20000.0;//估计底盘能量
static float Last_RefereeSystem_Energy=20000.0;//上一次裁判系统剩余能量值，用于校准
float RefereeSystem_Energy=20000.0;

uint16_t ChassisLimit;

//功率控制相关参数初始化:
void ChassisPowerControl_ParamsInit(void)
{
  PC.count=0;
  PC.Max_Translation=4667.0f;
  PC.PowerSum=0;
  PC.V_Gains=1.0f;

  //能量环PID初始化:
  Energy_PID.target_cap=1030.0f;//1030
  Energy_PID.target_nocap=40.0f;
  Energy_PID.Kp=1.2f;
  Energy_PID.Ki=0.01f;
  Energy_PID.Kd=0.3f;
  Energy_PID.error = 0.0f;
  Energy_PID.lasterror = 0.0f;
  Energy_PID.actual = 0.0f;
  Energy_PID.integral = 0.0f;
  Energy_PID.derivative = 0.0f;
  Energy_PID.energy_control_enabled = 1;//开启PID控制
}

//能量环PID控制函数:
void Energy_PIDcontrol_loop(void)
{
	if(Energy_PID.energy_control_enabled != 1)
			return;
	
	Energy_PID.target = PC.cap_elc_ok ? Energy_PID.target_cap : Energy_PID.target_nocap;
	
	// 当超电有电时:
	if(PC.cap_elc_ok == 1)
	{
			// 使用超级电容能量
			Energy_PID.actual = SCAPrx.CapEnergy / 255.0f * 2100.0f;
	}
	// 当超电没电时:
	else
	{
			// 使用裁判系统能量
			Energy_PID.actual = judge_buffer;
	}
	
	// 修改误差计算方向
	Energy_PID.error = Energy_PID.actual - Energy_PID.target;  // 实际值 - 目标值
	
	Energy_PID.integral += Energy_PID.error;
	if(Energy_PID.integral > 1000.0f)
			Energy_PID.integral = 1000.0f;
	if(Energy_PID.integral < -1000.0f)
			Energy_PID.integral = -1000.0f;
	
	Energy_PID.derivative = Energy_PID.error - Energy_PID.lasterror;
	Energy_PID.lasterror = Energy_PID.error;
	
	Energy_PID.out = Energy_PID.Kp * Energy_PID.error + 
									 Energy_PID.Ki * Energy_PID.integral + 
									 Energy_PID.Kd * Energy_PID.derivative;
	
	// 输出限幅
	if(Energy_PID.out > 30.0f) Energy_PID.out = 30.0f;
	if(Energy_PID.out < -30.0f) Energy_PID.out = -30.0f;
	
	// 应用能量环控制
	PC.Power_Limit += Energy_PID.out;
	
	// 确保功率限制在合理范围内
	if(PC.Power_Limit < 10.0f) PC.Power_Limit = 10.0f;
}

//底盘功率控制任务
void Chassis_PowerControlTest(float Vx,float Vy,float Vz,float Input_Energy)
{
	//由缓冲能量得到的功率控制的功率上限:
	PC.Buffer_PowerRef=1.0f/(60.0f-pro_buffer)*judge_buffer;
	if(PC.Buffer_PowerRef>Mecanum_PowerControl_PowerMax)
	{
		PC.Buffer_PowerRef=Mecanum_PowerControl_PowerMax;
	}
	PC.Power_Limit=PC.Buffer_PowerRef*chassispowerlimit;

	PC.speed_magnitude=sqrt(Vx*Vx+Vy*Vy);
	PC.scale=PC.speed_magnitude/PC.Max_Translation;
		
	PC.cap_elc_ok=((SCAPrx.CapEnergy/255.0f)>0.4f);
	
	get_chassis_power_limit(&ChassisLimit);//来自于裁判系统的功率限制数据
		
	if(PC.scale>0.0f&&PC.scale<1.0f)
	{
			PC.Power_Limit*=PC.scale;//平移约束
	}
		
	Energy_PIDcontrol_loop();
		
	if(Moving_flag_t.Gyro_360_motion_flag==1)
	{
			if(PC.Power_Limit>90.0f)
				PC.Power_Limit=90.0f;
	}
	if(Moving_flag_t.Chassis_flowingflag==1)
	{
//		if(PC.Power_Limit>ChassisLimit)
//			PC.Power_Limit=ChassisLimit;
		if(PC.Power_Limit>80.0f)
			PC.Power_Limit=80.0f;
	}  
	if(Moving_flag_t.Chassis_sidewayflag==1)
	{
		if(PC.Power_Limit>45.0f)
			PC.Power_Limit=45.0f;
	}

	#ifndef GO_Mode //解除功率上限，全速测试，对抗赛使用;
	if(PC.Power_Limit>120.0f)
	{
		PC.Power_Limit=120.0f;
	}
	#endif
		
	if(PC.cap_elc_ok==1)
	{
		if(Moving_flag_t.pitch_up_flag==1)//准备飞坡
		{
			PC.Power_Limit=100.0f;
			Superpower_cap_Init(chassispowerlimit,judge_buffer,1,0);
		}
		else//飞坡标志位置为0，默认平地模式
		{
			if(RefereeSystem_GameStatus==1&&RefereeSystem_RemainTime>0&&Chassis_Energy>0)
			{
				PC.Energy_PowerLimit=Chassis_Energy/1.0f/RefereeSystem_RemainTime;
				if(PC.Power_Limit>PC.Energy_PowerLimit)
				{
					PC.Power_Limit=PC.Energy_PowerLimit;
				}
				//限制功率使用:
				Superpower_cap_Init(PC.Power_Limit,judge_buffer,1,0);
			}
		}
	}
	else//超电没电
	{
		if(Vx!=0||Vy!=0||Vz!=0)
		{
			Superpower_cap_Init(PC.Power_Limit,judge_buffer,1,0);
		}
		else
		{
			Superpower_cap_Init(chassispowerlimit,judge_buffer,1,0);
		}
		Moving_flag_t.pitch_up_flag=0;
		if(RefereeSystem_GameStatus==1&&RefereeSystem_RemainTime>0&&Chassis_Energy>0)
		{
			PC.Energy_PowerLimit=Chassis_Energy/1.0f/RefereeSystem_RemainTime;
			if(PC.Power_Limit>PC.Energy_PowerLimit)
			{
				PC.Power_Limit=PC.Energy_PowerLimit;
			}
		}
	}

	//powercontrol:
	PC.current_power=1.732050807568877f*(Motor_Power[0]+Motor_Power[1]+Motor_Power[2]+Motor_Power[3]);//可以更换模型，采取更精确的模型
	PC.PowerSum+=PC.current_power;
	if(RefereeSystem_GameStatus==1)
	{
		Input_Energy-=PC.current_power*0.002f;
		Chassis_Energy=Input_Energy;
		if(Last_RefereeSystem_Energy!=RefereeSystem_Energy)
		{
			Chassis_Energy=RefereeSystem_Energy;
		}
		if(Chassis_Energy>RefereeSystem_Energy)
		{
			Chassis_Energy=RefereeSystem_Energy;
		}
		if(Chassis_Energy<0)
		{
			Chassis_Energy=0;
		}
		Last_RefereeSystem_Energy=RefereeSystem_Energy;
	}
	if (PC.count >= Mecanum_PowerControl_T)
	{
			PC.count = 0;
			PC.average_power = PC.PowerSum / Mecanum_PowerControl_T;
			PC.PowerSum = 0;
			//超级电容没电时的全局处理
			//更改为:仅当功率接近上限时才降低增益
			if (!PC.cap_elc_ok)
			{
				if (PC.average_power > PC.Power_Limit * 0.9f) // 阈值提高到95%
				{
					PC.V_Gains -= 0.02f;  // 减小调整幅度
					if (PC.V_Gains < 0.8) // 提高最小增益限制
							PC.V_Gains = 0.8;
				}
				else if (PC.average_power < PC.Power_Limit * 0.7f)
				{
					// 当功率较低时允许缓慢恢复增益
					PC.V_Gains += 0.05f;
					if (PC.V_Gains > 2.2)
							PC.V_Gains = 2.2;
				}
			}
		else
		{
			//小陀螺模式特殊处理
			if (Moving_flag_t.Gyro_360_motion_flag == 1)
			{
				//超级电容有电
				if ((SCAPrx.CapEnergy / 255.0f) > 0.45)
				{
					//小陀螺模式下保持速度增益有效
					if (PC.average_power < PC.Power_Limit - 5.0f)
					{
						//功率低于限制，缓慢加速
						PC.V_Gains += 0.1f;
						if (PC.V_Gains > 1.5) PC.V_Gains = 1.5;//2
					}
					else if (PC.average_power > PC.Power_Limit + 5.0f)
					{
						//功率超过限制，减速
						if (judge_buffer < 60.0f - pro_buffer)
						{
								PC.V_Gains -= 0.05f;
						}
						else
						{
								PC.V_Gains -= 0.02f;
						}
						if (PC.V_Gains < 1) PC.V_Gains = 1;
					}
				}
				
				else
				{
					// 急速下降速度增益
					if (PC.average_power > PC.Power_Limit * 0.85f)
					{
						// 功率接近上限时急速下降
						PC.V_Gains -= 0.3f; // 大幅降低增益
						if (PC.V_Gains < 1) PC.V_Gains = 1;
					}
					else if (PC.average_power > PC.Power_Limit)
					{
						// 功率超过上限时更大幅度下降
						PC.V_Gains -= 0.5f;
						if (PC.V_Gains < 1) PC.V_Gains = 1;
					}
				}
			}
			//非小陀螺模式
			else
			{
				//原有增益调整逻辑
				if (PC.average_power < PC.Power_Limit / 2.0f)
				{
					PC.V_Gains += 0.05f;
					if (PC.V_Gains > 1) PC.V_Gains = 1;
				}
				else if (PC.average_power < PC.Power_Limit - 5)
				{
					PC.V_Gains += 0.02f;
					if (PC.V_Gains > 1) PC.V_Gains = 1;
				}
				else if (PC.average_power > PC.Power_Limit + 5)
				{
					if (judge_buffer < 60.0f - pro_buffer)
							PC.V_Gains -= 0.05f;
					else
							PC.V_Gains -= 0.02f;
					if (PC.V_Gains < 1)
							PC.V_Gains = 1;
				}
			}
		}
	}
	else if(Vx!=0||Vy!=0||Moving_flag_t.Gyro_360_motion_flag==1||Moving_flag_t.Chassis_flowingflag==1)
	{
		PC.count++;
	}
	else
	{
		PC.V_Gains=Mecanum_PowerControlGainCoefficientInitialValue;
	}
		
	if(Moving_flag_t.Gyro_360_motion_flag==1&&(Vx!=0||Vy!=0))
	{
	  Vx*=1;//当前全局变量作为速度缩放的话很浪费功率，要改为数组对每个电机的功率进行处理
		Vy*=1;
		Vz*=1;
	}
	if(!PC.cap_elc_ok) //超级电容没电
	{
			//额外限制最大速度
			float max_speed = 800.0f;
			if (fabs(Vx) > max_speed) Vx = (Vx > 0) ? max_speed : -max_speed;
			if (fabs(Vy) > max_speed) Vy = (Vy > 0) ? max_speed : -max_speed;
			if (fabs(Vz) > max_speed) Vz = (Vz > 0) ? max_speed : -max_speed;
	}
	Vx*=PC.V_Gains;
	Vy*=PC.V_Gains;
	if(Moving_flag_t.Gyro_360_motion_flag==1||Moving_flag_t.Chassis_flowingflag==1)
	{
			Vz*=PC.V_Gains;
	}
	if(SCAPrx.CapEnergy<100)
	{
		PC.Z_Gains=0.9;
	}
	else
	{
		PC.Z_Gains=1;
	}
}

void PowerControlTask(void *argument)
{
  TickType_t xLastWakeTime=xTaskGetTickCount();
  const TickType_t xFrequency=pdMS_TO_TICKS(8);
  while(1)
  {
		Chassis_Remote_t.Vx=(Chassis_Remote_t.X-RC_MIDDLE)*5;
		Chassis_Remote_t.Vy=(Chassis_Remote_t.Y-RC_MIDDLE)*5;
		Chassis_Remote_t.Vz=(Chassis_Remote_t.Z-RC_MIDDLE)*10;
		Chassis_PowerControlTest(Chassis_Remote_t.Vx,Chassis_Remote_t.Vy,Chassis_Remote_t.Vz,18000);
		vTaskDelayUntil(&xLastWakeTime,xFrequency);
  }
	
}





