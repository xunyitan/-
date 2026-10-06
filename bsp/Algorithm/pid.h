#ifndef PID_H
#define PID_H

#define Limit(data, min, max) ((data) < (min) ? (min) : ((data) > (max) ? (max) : (data)))

#include <stdio.h>
#include <stdlib.h>
#include <math.h>

typedef struct PidTypeDef
	{
    char mode; //PID类型
									
    float Kp;  //PID 三个参数
    float Ki;
    float Kd;

		float max_erro; //最大误差
    float max_out;  //最大输出
    float max_iout; //最大积分输出

    float set; //设定值
    float fdb; //反馈值

    float out;
    float Pout;
    float Iout;
    float Dout;
    float Dbuf[3];  //微分项 0最新 1上一次 2上上次
    float error[3]; //误差项 0最新 1上一次 2上上次
		float lastdout;
	
	void ( *Param_Init)(struct PidTypeDef *pid,  //PID参数初始化
		char mode,
		float PID[3],
		float max_erro,
		float max_iout,
		float max_out			
	);
	float (*Cal_PID)(struct PidTypeDef *pid, const float ref, const float set);   //pid计算
		
}PidTypeDef;

typedef enum
{
    PID_POSITION = 0,//位置式PID
		PID_POSITION2,
    PID_DELTA, //增量式PID
		PID_DELTA2,
}PID_MODE;

typedef struct PID_Private_t{
	float error;
	float last_error;
	float last_lasterror;
	float integral;
}PID_Private_t;//私有状态变量，不应该被我修改；

typedef struct PID_Control_t{
	float Kp;
	float Ki;
	float Kd;
	float IntegralMin;
	float IntegralMax;
}PID_Control_t;//PID控制器参数;

typedef struct PID_UserSet_t{
	float Target;
	float Actual;
	float Actual_vel;
	float Out;
}PID_UserSet_t;

typedef struct PID_FrontFeed_t{
	float LastLastPoint;
	float LastPoint;
	float Kf;          
	float Error;
	float FrontFeedOut;
}PID_FrontFeed_t;//前馈控制参数;

typedef struct{
	PID_Private_t P_Private;
	PID_Control_t P_Control;
	PID_UserSet_t P_Myset;
	PID_FrontFeed_t P_FrontFeed;
}PID_t;//完整pid控制器;

void PID_Init(PID_t *p);
void PID_CparameterUpdate(PID_t *p,float Kp,float Ki,float Kd);
void FrontFeedParamter_Updata(PID_t *p,float Kf);
void PID_IntegralLimitUpdate(PID_t *p, float IntegralMin, float IntegralMax) ;
void Pos_PID(PID_t *p);
void IMUPos_PID(PID_t *p);
void Inc_PID(PID_t *PID);
void Inc_PIDold(PID_t* PID);
void Posold_PID(PID_t *p);
void PosoldMiddle_PID(PID_t *p);
void pid_init2(PidTypeDef* pid);
float low_pass_filter2(float value);

#endif
