#ifndef SHOOTTASK_H
#define SHOOTTASK_H

#include "Sys.h"

typedef struct FW_Shoot{
	double M2006_Out;
	double M3508_FW_Out[2];
}FW_Shoot;

typedef struct FW_FLGA{
	uint8_t shootcmd;
	uint8_t fwSpeedcmd;
	uint8_t Eject_bullet;
}FW_FLGA;

extern PidTypeDef FrictionWheel_Speed[2];
extern PidTypeDef Strumming_Speed;
extern float FrictionWheel_Speed_PID[3];
extern float Strumming_Speed_PID[3];
extern float ShootingUpLimit;

void Shooting_Task(void *argument);
void FW_SpeedUP(int FW_1,int FW_2);
void FW_Shooting(int speed);

#endif
