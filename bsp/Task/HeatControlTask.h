#ifndef HEATCONTROLTASK_H
#define HEATCONTROLTASK_H

#include "Sys.h"

typedef struct ShootingHeatControl_t{
	float shootingTime;
	float CanShootTime_cmd;
	float shootingColdSpeed;
	float RemainingHeatValue;
	float ShootSpeed;
	float tar2006Speed;
	float Constparam;
	float FW3508Speed;
}ShootingHeatControl_t;

typedef struct ShootHeatControl_PID_t{
	float target,actual,error,lasterror;
	float Kp,Ki,Kd,integral;
	float targetHave,targetHavenot;
	float out,outmax,outmin;
	int8_t Shoot_is_ok;
}ShootHeatControl_PID_t;

extern ShootingHeatControl_t ShootingHeatControl;
extern ShootHeatControl_PID_t ShootHeatControl_PID;

void ShootPowerControlTask(void);
void HeatControlTask(void);

#endif
