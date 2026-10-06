#ifndef IMUTASK_H
#define IMUTASK_H

#include "Sys.h"

#define AX 0
#define AY 1
#define AZ 2

#define INS_TASK_PERIOD 1

typedef struct
{
	float q[4]; // 四元数估计值

	float Gyro[3];  // 角速度
	float Accel[3]; // 加速度
	float MotionAccel_b[3]; // 机体坐标加速度
	float MotionAccel_n[3]; // 绝对系加速度
	float GyroFiltered[3];

	float AccelLPF; // 加速度低通滤波系数
	float GyroLPF;  // 角速度低通滤波系数

	// 加速度在绝对系的向量表示
	float xn[3];
	float yn[3];
	float zn[3];

	float atanxz;
	float atanyz;

	// 位姿
	float Roll;
	float Pitch;
	float Yaw;
	float YawTotalAngle;
	float YawAngleLast;
	float YawRoundCount;
	
	float actual_dt;
} INS_t;

//用于修正安装误差的参数,demo中可无视
typedef struct
{
	uint8_t flag;

	float scale[3];

	float Yaw;
	float Pitch;
	float Roll;
} IMU_Param_t;

// 姿态数据结构
typedef struct {
    float DegYaw;       // 角度制偏航角
    float DegPitch;     // 角度制俯仰角
    float DegRoll;      // 角度制横滚角
    float RadYaw;       // 弧度制偏航角
    float RadPitch;     // 弧度制俯仰角
    float RadRoll;      // 弧度制横滚角
} AttitudeData;

extern uint8_t bmi088_err;
// 全局姿态数据
extern AttitudeData Attitude;

extern INS_t INS;
extern struct MAHONY_FILTER_t mahony;

void INS_Init(void);
void INS_Task(float dt);
void INS_MahonyInit(void);
void INS_MahonyTask(float dt);
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q);
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q);
void IMURead_Task(void *argument);
void IMUTemp_ControlTask(void *argument);
void IMUAngle_Task(void *argument);

#endif
