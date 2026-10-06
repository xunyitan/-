#ifndef DJL_DRIVER_H
#define DJL_DRIVER_H

#include "pid.h"
#include "struct_typedef.h"
#include "main.h"
#include "Sys.h"

#define DJL_MOTOR_3508_Kt 0.3//3508电机扭矩常数

//3508额定转速(改装后)
#define DJI_MOTOR_3508_RATED_SPEED  (571U)  //RPM
//3508电机减速比(改装后)
//全向轮减速比:
#define DJI_MOTOR_3508_REDUCTION_RATIO  (268.0f/17.0f)//相比原来的全向轮，速度变快，减速能力变弱，扭矩变小
#define DJI_MOTOR_3508_SPEED_MAX		9000
#define DJI_Motor_CurrentMax	16000.0f//16000
#define DJL_Motor6020_CurrentMax 25000
//6020额定转速:
#define DJL_MOTOR_6020_RATED_SPEED  (132U)  //RPM
//6020电机减速比
#define DJL_MOTOR_6020_REDUCTION_RATIO  1
#define DJL_MOTOR_6020_SPEED_MAX    (320.0f)

#define DJL_MOTOR_6020_PosMax  8191
#define DJL_MOTOR_6020_PosHalf 4095
#define DJL_MOTOR_6020_PosQuarter 2048

#define Middle_Speed_Limit 1024

#define RC_MIDDLE 1024
#define RC_MAX    1684
#define RC_MIN    364

#define RC_S1_UP       1
#define RC_S1_MIDDLE   3
#define RC_S1_DOWN     2

#define RC_S2_DOWN   2
#define RC_S2_MIDDLE 3
#define RC_S2_UP     1

#define Buzzer_loudest 17000
#define Buzzer_middle  13000
#define Buzzer_min     12000
#define Buzzer_OFF     0

//全向轮步兵相关参数:
#define Level_Gyro 450//400
#define STICK_DEADZONE 50 //摇杆死区阈值
#define GYRO_TO_TARGET_SCALE 377.142857f
#define Gimbal_Sensitivity 8
#define Wheel_R            0.1523f //单位:mm
#define Wheelbase          0.2034f //单位:mm

//功控相关参数(测试用):
#define judge_buffer 60 //裁判系统缓冲能量上限60焦耳，暂时用宏定义表示
#define chassispowerlimit 70 //功率优先情况下的一级限定功率，单纯功率
#define pro_buffer 30 //裁判系统可用于调控进行功率限制的缓冲能量
#define Mecanum_PowerControl_PowerMax 4.0f
#define Scap_add_power 10.0f//使用超电时增加功率
#define RefereeSystem_GameStatus 1//1为比赛中，0为比赛未开始（要来自裁判系统的准确数据）
#define RefereeSystem_RemainTime 1//比赛时间（要来自裁判系统的准确数据）
#define Mecanum_PowerControl_T 30//超电没电时为3
#define Mecanum_PowerControlGainCoefficientInitialValue		0.2//功率控制增益系数初始值

//目标速度增益:
#define VX_GAINS 8
#define VY_GAINS 8
#define VZ_GAINS 10

#define wheel_radius 0.07f //轮子半径

typedef struct Gimbal_FindingMode_t{
	uint8_t ControlCmd;
	uint8_t ShootCmd;
	float Yaw;	
	float Pitch;
  float Horizon_distance;
	int16_t Yaw_raw;
	int16_t Pitch_yaw;
	int16_t Distance_raw;
}Gimbal_FindingModeRx_t;

typedef struct Gimbal_FindingModeTx_t{
	int16_t Q1;
	int16_t Q2;
	int16_t Q3;
	int16_t Q4;
}Gimbal_FindingModeTx_t;

//gm6020和m3508的rx数据相同,可以互用,不必增加
typedef struct djl_motorRx_t{
	int16_t Pos;//机械角度
	int16_t Speed;//速度
	int16_t Torque;//电流
	int8_t  Temperature;//温度
	int8_t errorcode;
}djl_motorRx_t;

typedef struct djl_motorTx_t{
	int16_t m_Current;
	int16_t m_Speed;
	int16_t m_Pos;
}djl_motorTx_t;

typedef struct djl_motorControl_t{
	PID_t Current;
	PID_t Speed;
	PID_t Pos;
	PID_t YawPos;
	PID_t PitchPos;
}djl_motorControl_t;

typedef struct djl_motorCan_t{
	uint32_t StdID;
	uint8_t DLC;
	djl_motorRx_t Rx;
	djl_motorTx_t Tx;
	djl_motorControl_t Control_t;
}djl_motorCan_t;

typedef struct bmi088_control_t{
	PID_t Pos;
}bmi088_control_t;

typedef struct bmi088_t{
	fp32 gyro[3];
	fp32 accel[3];
	fp32 temp;
	fp32 angle[3];
	bmi088_control_t Control_t;
	uint16_t pwm_value;
	int16_t gyro_raw[3];
	float GyroFiltered[3];
	float AccelLPF;
	float GyroLPF;
}bmi088_t;

typedef struct Moving_flag{
	uint8_t Gyro_360_motion_flag;
	uint8_t Chassis_flowingflag;
	uint8_t Chassis_sidewayflag;
	uint8_t pitch_up_flag;
	uint8_t Speedup_Flag;
}Moving_flag;

extern djl_motorCan_t djl_3508[4];
extern djl_motorCan_t djl_3508FW[2];
extern djl_motorCan_t djl_6020[4];
extern djl_motorCan_t djl_2006[1];
extern bmi088_t bmi088_param[2];
extern Gimbal_FindingModeRx_t GimbalFinding_Mode[2];
extern Moving_flag Moving_flag_t;
extern Gimbal_FindingModeTx_t GimbalFinding_ModeTx[2];
extern Gimbal_FindingModeRx_t GimbalFinding_Mode[2];

void djlMotor_Init(djl_motorCan_t *pthis,uint32_t StdID);

#endif
