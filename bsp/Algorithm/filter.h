#ifndef FILTER_H
#define FILTER_H

#include "main.h"
#include "pid.h"

typedef struct Lowpassfilter_t{
	float Prev_Value;//上一次的滤波值
	float Alpha;     //滤波系数(取值0~1，越小惯性越大)
}Lowpassfilter_t;

typedef struct SlopeFilter_t{
	float prev_target;   // 上一次的目标值
	float max_delta;     // 每个控制周期允许的最大变化量（坡度化强度）
}SlopeFilter_t;

typedef struct ChassisSlope_t{
	float max_len;
	float target;
	float actual;
}ChassisSlope_t;

extern Lowpassfilter_t LowPassFilter[4];

void LowPassFilter_Init(Lowpassfilter_t* LowPassFilter, float Alpha); 
float LowPassFilter_Update(Lowpassfilter_t* Low,float Value);
void SlopeFilter_Init(SlopeFilter_t* slope, float max_delta);
float SlopeFilter_Update(SlopeFilter_t* slope, float new_target);
void ChassisScope_Init(ChassisSlope_t* slope,float max_len);
float ChassisScope_Update(ChassisSlope_t* slope,float target,float actual,float max_len);

#endif
