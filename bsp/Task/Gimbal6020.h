#ifndef GIMBAL6020_H
#define GIMBAL6020_H

#include "Sys.h"

typedef struct Gimbal_Remote{
	float X;
	float Y;
}Gimbal_Remote;

typedef struct GimbalYaw_SMC_t{
	float stick_value;//摇杆输入
	float yaw_rate;//Yaw轴角速度
	float angle_increment;//角度增量
}GimbalYaw_SMC_t;

extern PidTypeDef Gimbal_Yaw_Speed;
extern PidTypeDef Gimbal_Pitch_Speed;
extern float Gimbal_Yaw_Speed_PID[3];//11,1,0
extern float Gimbal_Pitch_Speed_PID[3];//{6,1,0.1};

float Data_Clipping(float Data,float Data_Min,float Data_Max);
void GimbalTask_Test(void *argument);
void Gimbal_PitchLQRControll(float targetPitch);
void Gimbal_TwoPIDControl(float targetPitch);
void Gimbal_Control(void);
void SMC_GimbalTask(void *argument);
void GimbalFindingMode(void);
void pitchFindingMode(void);
void GimbalFindingMode_PID(float target,float actual);
void GimbalFindingMode_PIDDouble(float target,float actual);

#endif
