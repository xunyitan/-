#ifndef SQUAREWAVE_H
#define SQUAREWAVE_H

#include "Sys.h"

typedef struct Squarewave_t{
	uint32_t half_period_ticks;//方波半周期
	float excitation_amplitude;//方波幅度
}Squarewave_t;

extern uint32_t Stick;//方波计数器
extern Squarewave_t Squarewave;

void Squarewave_param_Init(uint32_t hpticks,float amplitude);

#endif
