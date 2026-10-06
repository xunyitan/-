#ifndef SLIDING_MODE_H
#define SLIDING_MODE_H

#include "Sys.h"

//滑膜控制器相关参数:
typedef struct SlidingMode_Control_t{
	float error;
	float lasterror;
	float dref;//目标角度一阶导数(目标角速度)
	float ddref;//目标角度二阶导数(目标角加速度)
	float refl;//上一时刻目标角度值
	float dref_last;//上上一时刻目标角度值
	float S;//滑模面
}SlidingMode_Control_t;

//滑膜相关参数:
typedef struct SlidingMode_t{
	float C;//滑模面系数
	float K;//增益
	float Target_angle;//目标角度
	float error_deadarea;//误差死区阈值
	float outmax;//输出最大值
	float outmin;//输出最小值
	float J;//系统转动惯量
	float epsilon;//滑膜切换项增益
	float angle;//当前角度
	float angle_vel;//当前角速度
	float out;//计算出来的输出
	float dt;//采样时间
	SlidingMode_Control_t SMC;//滑模控制器
}SlidingMode_t;

extern SlidingMode_t Yaw_SMC;

void SlidingMode_Param_Init(SlidingMode_t *SMMC);
void SMC_Controller_Update(SlidingMode_t *S,float target_angle,float current_angle,float current_vel,float dt);
float Sat(float y);
int8_t Sign(float y);

#endif
