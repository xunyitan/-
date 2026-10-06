#ifndef RLS_H
#define RLS_H

#include "matrix.h"

typedef struct RLS{
	uint32_t dimension;  // Dimension of the RLS space
	float lambda;        // The forget index
	float delta;         // Initialized value of the transferred matrix
	
	TickType_t lastUpdate;  // Last update tick
	uint32_t updateCnt;     // Total update Count
	
	/* RLS relevant matrix */
	Matrixf transMatrix;     // Transfer matrix instance
	Matrixf gainVector;      // Gain vector for params update
	Matrixf paramsVector;    // Params vector
	Matrixf defaultParamsVector;
	float output;           // Estimated / filtered output
	
	//¥¥Ω®¡Ÿ ±æÿ’Û:
	Matrixf temp1,temp2,temp3,temp4,temp5;
	float temp1_data[16];
	float temp2_data[16];
	float temp3_data[16];
	float temp4_data[16];
	float temp5_data[16];
	
}RLS;

extern RLS RLS_t;
extern Matrixf sampleVector, paramsVector;
extern float actualOutput, estimatedOutput;
extern float sample_data[2];
extern float params_data[2];

void RLS_Init(RLS* rls, float delta_, float lambda_, int dim);
void RLS_InitWithParam(RLS* rls, float delta_, float lambda_, int dim, Matrixf* initParam);
void RLS_reset(RLS* rls);
void RLS_update(RLS* rls, Matrixf* sampleVector, float actualOutput, Matrixf* result);
void RLS_setParamVector(RLS* rls, Matrixf* updatedParams);
void RLS_getParamsVector(const RLS* rls, Matrixf* result);
float RLS_getOutput(const RLS* rls);
void RLS_validate(const RLS* rls);
const float* RLS_GetParamsVector(const RLS* rls);

#endif
