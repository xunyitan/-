#include "Squarewave.h"

uint32_t Stick;//·½²¨¼ÆÊýÆ÷

Squarewave_t Squarewave;

void Squarewave_param_Init(uint32_t hpticks,float amplitude)
{
	Squarewave.half_period_ticks=hpticks;
	Squarewave.excitation_amplitude=amplitude;
}
