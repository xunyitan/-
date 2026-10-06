#include "Sliding_Mode.h"

SlidingMode_t Yaw_SMC;

//非线性控制器，滑膜控制器，主要用于控制yaw轴云台
void SlidingMode_Param_Init(SlidingMode_t *SMMC)
{
	if(!SMMC)
		return;
	
	SMMC->C=20;
	SMMC->K=10;//120
	SMMC->Target_angle=0;
	SMMC->error_deadarea=0.2;
	SMMC->J=0.8;//机械测量
	SMMC->epsilon=0.6;
	SMMC->dt=0;
	SMMC->SMC.error=0;
	SMMC->SMC.lasterror=0;
	SMMC->SMC.dref=0;
	SMMC->SMC.ddref=0;
	SMMC->SMC.refl=0;
	SMMC->SMC.dref_last=0;
	SMMC->SMC.S=0;
	SMMC->angle=0;
	SMMC->angle_vel=0;
	SMMC->out=0;
	SMMC->outmax=25000;
	SMMC->outmin=-25000;
}

//滑模控制器:
void SMC_Controller_Update(SlidingMode_t *S,float target_angle,float current_angle,float current_vel,float dt)
{
	if(!S)
		return;
	
	if(S->dt>0)
		S->dt=dt;
	
	//更新系统状态:
	S->Target_angle=target_angle;
	S->angle=current_angle;
	S->angle_vel=current_vel;
	
	//计算目标值导数:
	S->SMC.dref=(S->Target_angle-S->SMC.refl)/S->dt;
	S->SMC.ddref=(S->SMC.dref-S->SMC.dref_last)/S->dt;
	
	//角度误差:
	S->SMC.error=S->angle-S->Target_angle;
	
	// 死区处理
	if (fabsf(S->SMC.error)<S->error_deadarea) 
	{
		S->out=0;
	}
	
	else
	{
		//计算滑模面:
		S->SMC.S = S->C * S->SMC.error + (S->angle_vel - S->SMC.dref);
		
		// 计算控制输出
		S->out = S->J * (S->SMC.ddref - S->C * (S->angle_vel - S->SMC.dref) 
												 - S->epsilon * Sat(S->SMC.S) - S->K * S->SMC.S);
		
		// 输出限幅
		if (S->out > S->outmax) S->out = S->outmax;
		if (S->out < S->outmin) S->out = S->outmin;
	}
	
	// 更新历史值
	S->SMC.lasterror = S->SMC.error;
	S->SMC.refl = S->Target_angle;
	S->SMC.dref_last = S->SMC.dref;
}

// 饱和函数
float Sat(float y) {
	if (fabsf(y) <= 1.0f) {
			return y;
	} else {
			return Sign(y);
	}
}

// 符号函数
int8_t Sign(float y) {
	if (y > 0.0f) return 1;
	else if (y < 0.0f) return -1;
	else return 0;
}
