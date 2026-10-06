#ifndef DM_DRIVER_H
#define DM_DRIVER_H

#include "Sys.h"

#define P_MIN -12.5		   
#define P_MAX 12.5		    
#define V_MIN -45			
#define V_MAX 45			
#define KP_MIN 0.0		   
#define KP_MAX 500.0	   
#define KD_MIN 0.0		    
#define KD_MAX 5.0		   
#define T_MIN -18			
#define T_MAX 18			

typedef enum DM_MotorCtrlMode{
	DM_MotorCtrlMode_Mit = 0x000,    
	DM_MotorCtrlMode_Pos = 0x100,    
	DM_MotorCtrlMode_Speed = 0x200   
}DM_MotorCtrlMode;

typedef struct DM_MotorRx{
	uint8_t Error;
	float Pos;
	float Speed;
	float Torque;
}DM_MotorRx;

typedef struct DM_MotorMit_Tx{
	float Pos;
	float Speed;
	float Torque;
	float Kp;
	float Kd;
}DM_MotorMit_Tx;

typedef struct DM_Motor_t{
	uint32_t StdID;
	uint8_t DLC;
	DM_MotorMit_Tx Tx;
	DM_MotorRx Rx;
	uint8_t TxData[8];
}DM_Motor_t;

extern DM_Motor_t DM_Motor;
float uint_to_float(int x_int, float x_min, float x_max, int bits);
int float_to_uint(float x, float x_min, float x_max, int bits);
void DM_MotorInit(DM_Motor_t* pthis, uint32_t StdID, uint8_t DLC, DM_MotorCtrlMode Mode);
void DM_MotorEnable(DM_Motor_t *pthis);
void DM_MitControl(DM_Motor_t *pthis);
void Gimbal4310_EnableTask(void *argument);

#endif
