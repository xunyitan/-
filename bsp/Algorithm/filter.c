#include "filter.h"

Lowpassfilter_t Lowpassfilter[4];

void LowPassFilter_Init(Lowpassfilter_t* LowPassFilter, float Alpha)
{
	LowPassFilter->Prev_Value=0;
	LowPassFilter->Alpha=Alpha;
}

float LowPassFilter_Update(Lowpassfilter_t* LowPassFilter, float Value)
{
	float filtered_value = LowPassFilter->Alpha * Value 
			+ (1.f - LowPassFilter->Alpha) * LowPassFilter->Prev_Value;
	LowPassFilter->Prev_Value = filtered_value;
	return filtered_value;
}

// 坡度化初始化函数
void SlopeFilter_Init(SlopeFilter_t* slope, float max_delta) 
{
	slope->prev_target = 0.0f;
	slope->max_delta = max_delta;
}

// 坡度化更新函数
float SlopeFilter_Update(SlopeFilter_t* slope, float new_target)
{
	float delta = new_target - slope->prev_target;
	
	// 限制变化率
	if (delta > slope->max_delta) {
			new_target = slope->prev_target + slope->max_delta;
	} else if (delta < -slope->max_delta) {
			new_target = slope->prev_target - slope->max_delta;
	}
	
	// 更新上一次的目标值
	slope->prev_target = new_target;
	
	return new_target;
}

//底盘坡度化初始化函数
void ChassisScope_Init(ChassisSlope_t* slope,float max_len)
{
	slope->target=0.0f;
	slope->actual=0.0f;
	slope->max_len=max_len;
}

//底盘坡度化函数
float ChassisScope_Update(ChassisSlope_t* slope,float target,float actual,float max_len)
{
	float error=target-actual;
	if(error>max_len)
	{
		return actual+max_len;
	}
	else if(error<-max_len)
	{
		return actual-max_len;
	}
	else
	{
		return target;
	}
}



