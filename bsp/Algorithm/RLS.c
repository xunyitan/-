#include "RLS.h"
#include "matrix.h"

RLS RLS_t;

void RLS_Init(RLS* rls, float delta_, float lambda_, int dim)
{
	rls->dimension = dim;
	rls->lambda = lambda_;
	rls->delta = delta_;
	rls->lastUpdate = 0;
	rls->updateCnt = 0;
	rls->output = 0.0f;  // 初始化输出
	
	// 为每个矩阵分配独立的数据缓冲区
	rls->transMatrix.data = rls->temp5_data;  // dim x dim
	rls->gainVector.data = rls->temp1_data;   // dim x 1
	rls->paramsVector.data = rls->temp2_data; // dim x 1
	rls->defaultParamsVector.data = rls->temp3_data; // dim x 1
	
	// 设置矩阵维度
	rls->transMatrix.rows = dim;
	rls->transMatrix.cols = dim;
	rls->gainVector.rows = dim;
	rls->gainVector.cols = 1;
	rls->paramsVector.rows = dim;
	rls->paramsVector.cols = 1;
	rls->defaultParamsVector.rows = dim;
	rls->defaultParamsVector.cols = 1;
	
	// 初始化临时矩阵维度
	rls->temp1.rows = dim; rls->temp1.cols = 1;
	rls->temp2.rows = 1; rls->temp2.cols = dim;
	rls->temp3.rows = 1; rls->temp3.cols = dim;
	rls->temp4.rows = 1; rls->temp4.cols = 1;
	rls->temp5.rows = dim; rls->temp5.cols = dim;
	
	// 为临时矩阵分配数据缓冲区
	rls->temp1.data = rls->temp1_data;
	rls->temp2.data = rls->temp2_data;
	rls->temp3.data = rls->temp3_data;
	rls->temp4.data = rls->temp4_data;
	rls->temp5.data = rls->temp5_data;
	
	// 初始化ARM矩阵结构
	arm_mat_init_f32(&rls->transMatrix.arm_mat_, dim, dim, rls->transMatrix.data);
	arm_mat_init_f32(&rls->gainVector.arm_mat_, dim, 1, rls->gainVector.data);
	arm_mat_init_f32(&rls->paramsVector.arm_mat_, dim, 1, rls->paramsVector.data);
	arm_mat_init_f32(&rls->defaultParamsVector.arm_mat_, dim, 1, rls->defaultParamsVector.data);
	
	arm_mat_init_f32(&rls->temp1.arm_mat_, dim, 1, rls->temp1.data);
	arm_mat_init_f32(&rls->temp2.arm_mat_, 1, dim, rls->temp2.data);
	arm_mat_init_f32(&rls->temp3.arm_mat_, 1, dim, rls->temp3.data);
	arm_mat_init_f32(&rls->temp4.arm_mat_, 1, 1, rls->temp4.data);
	arm_mat_init_f32(&rls->temp5.arm_mat_, dim, dim, rls->temp5.data);
	
	RLS_reset(rls);
	RLS_validate(rls);
}

// 对应第二个构造函数 RLS(float delta_, float lambda_, Matrixf<dim, 1> initParam)
void RLS_InitWithParam(RLS* rls, float delta_, float lambda_, int dim, Matrixf* initParam)
{
	RLS_Init(rls, delta_, lambda_, dim);
	Matrixf_copy(&rls->defaultParamsVector, initParam);
}

// 对应 reset()
void RLS_reset(RLS* rls)
{
	// 初始化矩阵维度
	Matrixf_eye(&rls->transMatrix);
	Matrixf_mul_assign_float(&rls->transMatrix, rls->delta);
	
	Matrixf_zeros(&rls->gainVector);
	Matrixf_zeros(&rls->paramsVector);
	
	// 确保矩阵维度正确
	rls->transMatrix.rows = rls->dimension;
	rls->transMatrix.cols = rls->dimension;  
	rls->gainVector.rows = rls->dimension;
	rls->gainVector.cols = 1;
	rls->paramsVector.rows = rls->dimension;
	rls->paramsVector.cols = 1;
	rls->defaultParamsVector.rows = rls->dimension;
	rls->defaultParamsVector.cols = 1;
}

//这三个变量拿来测试的:
uint8_t yuuk;
float outo;
uint32_t timerc;
uint8_t yui=0;

// 对应 update()
void RLS_update(RLS* rls, Matrixf* sampleVector, float actualOutput, Matrixf* result)
{
	if (rls == NULL || sampleVector == NULL || result == NULL) {
			return;
	}
	
	float denominator, error;
	
	// 1. 计算增益向量: K = (P * x) / (lambda + x^T * P * x)
	Matrixf_mult(&rls->transMatrix, sampleVector, &rls->temp1);  // temp1 = P * x
	
	Matrixf_trans(sampleVector, &rls->temp2);  // temp2 = x^T
	
	Matrixf_mult(&rls->temp2, &rls->transMatrix, &rls->temp3);  // temp3 = x^T * P
	
	Matrixf_mult(&rls->temp3, sampleVector, &rls->temp4);  // temp4 = x^T * P * x (标量)
	
	denominator = rls->lambda + rls->temp4.data[0];
	
	// K = (P * x) / (lambda + x^T * P * x)
	Matrixf_copy(&rls->gainVector, &rls->temp1);
	Matrixf_div_assign_float(&rls->gainVector, denominator);
	
	// 2. 计算误差: e = y - x^T * θ
	Matrixf_mult(&rls->temp2, &rls->paramsVector, &rls->temp3);  // temp3 = x^T * θ
	error = actualOutput - rls->temp3.data[0];
	
	// 3. 更新参数: θ = θ + K * e
	Matrixf_copy(&rls->temp1, &rls->gainVector);
	Matrixf_mul_assign_float(&rls->temp1, error);  // temp1 = K * e
	
	Matrixf_add_assign(&rls->paramsVector, &rls->temp1);  // θ += K * e
	
	// 4. 更新协方差矩阵: P = (P - K * x^T * P) / lambda
	Matrixf_mult(&rls->gainVector, &rls->temp2, &rls->temp5);  // temp5 = K * x^T
	
	Matrixf_mult(&rls->temp5, &rls->transMatrix, &rls->temp1);  // temp1 = K * x^T * P
	
	Matrixf_sub(&rls->transMatrix, &rls->temp1, &rls->temp5);  // temp5 = P - K * x^T * P
	
	Matrixf_copy(&rls->transMatrix, &rls->temp5);
	Matrixf_div_assign_float(&rls->transMatrix, rls->lambda);  // P = (P - K * x^T * P) / lambda
	
	// 更新时间戳
	rls->updateCnt++;
	rls->lastUpdate = xTaskGetTickCount();
	
	// 返回结果
	Matrixf_copy(result, &rls->paramsVector);
	
	// 计算输出
	Matrixf_mult(&rls->temp2, &rls->paramsVector, &rls->temp3);
	rls->output = rls->temp3.data[0];
	outo=rls->output;
	timerc=rls->lastUpdate;
	
	yui=1;
}

// 对应 setParamVector()
void RLS_setParamVector(RLS* rls, Matrixf* updatedParams)
{
	Matrixf_copy(&rls->paramsVector, updatedParams);
	Matrixf_copy(&rls->defaultParamsVector, updatedParams);
}

// 对应 getParamsVector()
void RLS_getParamsVector(const RLS* rls, Matrixf* result)
{
	Matrixf_copy(result, &rls->paramsVector);
}

// 对应 getOutput()
float RLS_getOutput(const RLS* rls)
{
	return rls->output;
}

// 对应 validate()
void RLS_validate(const RLS* rls)
{
	configASSERT(rls->lambda >= 0.0f || rls->lambda <= 1.0f);
	configASSERT(rls->delta > 0);
}

// 获取参数向量的数据指针
const float* RLS_GetParamsVector(const RLS* rls)
{
	if(rls == NULL) 
			return NULL;
	
	return rls->paramsVector.data;
}

