#ifndef STATETASK_H
#define STATETASK_H

#include "Sys.h"

typedef struct CHE_state{
	float zheng;
	float fu;
}CHE_state;

void CHE_stateTask(void *argument);

#endif
