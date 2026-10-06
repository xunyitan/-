//#ifndef CHASSISTASK_H
//#define CHASSISTASK_H

//#include "Sys.h"
//#include "filter.h"

//typedef struct Chassis_Remote
//{
//	float X;
//	float Y;
//	float Z;
//	float Vx;
//	float Vy;
//	float Vz;
//	double scaled_X;
//	double scaled_Y;
//	double scaled_Z;
//	float gains;
//	uint32_t SlopeLen;
//}Chassis_Remote;

//typedef struct Chassis_Following{
//	int Middle_location;
//	int Middle_err;
//	short way_selet;
//	short way_chart_a;
//}Chassis_Following;

//typedef struct Chassis_Gyro360{
//	float Bittop_X;
//	float Bittop_Y;
//	float Bittop_Angle;
//	float Chassis_360_speed_target;
//}Chassis_Gyro360;

//typedef struct ChassisSlope_Param_t{
//	int DJL3508_len;
//	float DJL3508_error;
//	float DJL3508_value;
//	float DJL3508_target;
//	float DJL3508_targetset;
//}ChassisSlope_Param_t;

////extern Moving_flag Moving_flag_t;
//extern Chassis_Following Chassis_Following_t;
//extern Chassis_Remote Chassis_Remote_t;
//extern Chassis_Following Chassis_Following_t;
//extern Chassis_Gyro360 Chassis_Gyro360_t;
//extern PidTypeDef Chassis_Speed[4];
//extern PidTypeDef Chassis_Gyro_Speed[4];
//extern PidTypeDef Turn_Middle[4];
//extern float Chassis_Speed_PID[3];
//extern float Chassis_Gyro_Speed_PID[3];
//extern float Turn_Middle_PID[3];

//void ChassisTask_Test(void *argument);
//void ChassisSquarewave_control(void *argument);
//void Chassis_sideway(void);
//void ChassisFollowing_Gimbal(int Middle,short way_limit);
//void Gyro_360_motion(int Center_adjustment_value ,float target);
//float Chassis_Slope(int len,float target_set,float value);
//void ChassisKeyBoard_Control(void);

//#endif

#ifndef CHASSISTASK_H
#define CHASSISTASK_H

#include "Sys.h"
#include "filter.h"

typedef struct Chassis_Remote
{
	float X;
	float Y;
	float Z;
	float Vx;
	float Vy;
	float Vz;
	double scaled_X;
	double scaled_Y;
	double scaled_Z;
	float gains;
	uint32_t SlopeLen;
}Chassis_Remote;

typedef struct Chassis_Following{
	int Middle_location;
	int Middle_err;
	short way_selet;
	short way_chart_a;
}Chassis_Following;

typedef struct Chassis_Gyro360{
	float Bittop_X;
	float Bittop_Y;
	float Bittop_Angle;
	float Chassis_360_speed_target;
}Chassis_Gyro360;

typedef struct ChassisSlope_Param_t{
	int DJL3508_len;
	float DJL3508_error;
	float DJL3508_value;
	float DJL3508_target;
	float DJL3508_targetset;
}ChassisSlope_Param_t;

//extern Moving_flag Moving_flag_t;
extern Chassis_Following Chassis_Following_t;
extern Chassis_Remote Chassis_Remote_t;
extern Chassis_Following Chassis_Following_t;
extern Chassis_Gyro360 Chassis_Gyro360_t;
extern PidTypeDef Chassis_Speed[4];
extern PidTypeDef Chassis_Gyro_Speed[4];
extern PidTypeDef Turn_Middle[4];
extern float Chassis_Speed_PID[3];
extern float Chassis_Gyro_Speed_PID[3];
extern float Turn_Middle_PID[3];
extern float powerbuffer;

void ChassisTask_Test(void *argument);
void ChassisSquarewave_control(void *argument);
void Chassis_sideway(void);
void ChassisFollowing_Gimbal(int Middle,short way_limit);
void Gyro_360_motion(int Center_adjustment_value ,float target);
float Chassis_Slope(int len,float target_set,float value);
void ChassisKeyBoard_Control(void);

#endif

