#ifndef CHASSIS_IMU_H
#define CHASSIS_IMU_H

#include "Sys.h"

typedef struct dmimu_t{
	float Yaw;
	float Pitch;
	float Roll;
	
	float gyro[3];
	float accel[3];
	
	float q[4];
	
	float cur_temp;
}dmimu_t;

#endif
