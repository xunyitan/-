#include "pid.h"
#include "ChassisTask.h"

float fc = 50.0f;     //截止频率
float Ts = 0.01f;    //采样周期
float pi = 3.14159f; //π
float alpha = 0;     //滤波系数

#define LimitMax2(input, max)   \
    {                          \
        if (input > max)       \
        {                      \
            input = max;       \
        }                      \
        else if (input < -max) \
        {                      \
            input = -max;      \
        }                      \
    }

	/*参数初始化--------------------------------------------------------------*/
static void pid_param_init(PidTypeDef *pid, char mode, float PID[3],float max_erro, float max_iout, float max_out)
{
	pid->Kp = PID[0];
	pid->Ki = PID[1];
	pid->Kd = PID[2];
	pid->mode = mode;
	pid->max_erro = max_erro;
	pid->max_out = max_out;
	pid->max_iout = max_iout;
	pid->Dbuf[0] = pid->Dbuf[1] = pid->Dbuf[2] = 0.0f;
	pid->error[0] = pid->error[1] = pid->error[2] = pid->Pout = pid->Iout = pid->Dout = pid->out = 0.0f;
}
	
static float pid_caculate(PidTypeDef *pid, const float ref, const float set)
{  
		if (pid == NULL)
		{
			return 0;
		}
		if (pid->mode == PID_POSITION) 
		{		
			pid->error[0] = set - ref;
			pid->Dbuf[0] = ref;
			pid->Iout = pid->Iout +  pid->error[0];
			LimitMax2(pid->Iout, pid->max_iout);
			if(pid->error[0] > pid->max_erro){pid->Iout = 0;}
			if(pid->error[0] < -pid->max_erro){pid->Iout = 0;}			
			pid->Dbuf[2] = pid->Dbuf[0] - pid->Dbuf[1];
			pid->Pout = pid->Kp * pid->error[0];
			pid->Iout = pid->Ki * pid->Iout;				
			pid->Dout = pid->Kd * pid->Dbuf[2];
			pid->out = pid->Pout + pid->Iout - pid->Dout;
			pid->Dbuf[1] = ref;
			LimitMax2(pid->out, pid->max_out);      
		}
		else if(pid->mode == PID_POSITION2)
		{
			pid->Dbuf[0] = Chassis_Following_t.Middle_err;
			pid->Dbuf[2] = pid->Dbuf[0]-pid->Dbuf[1];
			pid->Dbuf[2] = low_pass_filter2(pid->Dbuf[2]);
			pid->Pout = pid->Kp * Chassis_Following_t.Middle_err;
			pid->Dout = pid->Kd * pid->Dbuf[2];
			pid->out = (pid->Pout)- (pid->Dout);
			pid->Dbuf[1] = Chassis_Following_t.Middle_err;
			
		}
		else if (pid->mode == PID_DELTA)
		{
			pid->Dbuf[0] = ref;
			pid->Dbuf[2] = pid->Dbuf[0] - pid->Dbuf[1];
			pid->error[0] = set - ref;
			pid->Dout = pid->Kd * pid->Dbuf[2];
			pid->Pout = pid->Kp * pid->error[0];
			pid->Iout = pid->Iout * pid->Ki + pid->Pout - pid->Dout;
			pid->out = pid->Iout;
			pid->Dbuf[1] = ref;     
			LimitMax2(pid->out, pid->max_out);			
		}
		else if (pid->mode == PID_DELTA2)
		{
			pid->error[0] = set - ref;
			pid->set = set;
			pid->Pout = pid->Kp * (pid->error[0] - pid->error[1]);
			pid->Iout = pid->Ki * pid->error[0];
			LimitMax2(pid->Iout, pid->max_iout);
			pid->Dbuf[2] = pid->Dbuf[1];
			pid->Dbuf[1] = pid->Dbuf[0];
			pid->Dbuf[0] = (pid->error[0] - 2.0f * pid->error[1] + pid->error[2]);
			pid->Dout = pid->Kd * pid->Dbuf[0];
			pid->out += pid->Pout + pid->Iout + pid->Dout;
			LimitMax2(pid->out, pid->max_out);
			pid->error[2] = pid->error[1];
			pid->error[1] = pid->error[0];
			pid-> lastdout=pid->Dout;
		}
	return pid->out;
}
/*pid结构体初始化，每一个pid参数需要调用一次-----------------------------------------------------*/
void pid_init2(PidTypeDef* pid)
{
    pid->Param_Init = pid_param_init;
    pid->Cal_PID = pid_caculate;
}

//实际速度pid
void PID_Init(PID_t *p)
{
	if(!p)
			return;
	p->P_Control.Kp = 0;
	p->P_Control.Ki = 0;
	p->P_Control.Kd = 0;
	p->P_Control.IntegralMin = 0;
	p->P_Control.IntegralMax = 0;
	
	p->P_Myset.Target = 0;
	p->P_Myset.Actual = 0;
	p->P_Myset.Out = 0;
	
	p->P_Private.error = 0;
	p->P_Private.last_error = 0;
	p->P_Private.last_lasterror = 0;
	p->P_Private.integral = 0;
	
	p->P_FrontFeed.Kf = 0;
	p->P_FrontFeed.LastPoint = 0;
	p->P_FrontFeed.FrontFeedOut = 0;
	p->P_FrontFeed.LastLastPoint = 0;
	p->P_FrontFeed.Error = 0;
}

void PID_CparameterUpdate(PID_t *p,float Kp,float Ki,float Kd)
{
	if(!p)
		return;
	p->P_Control.Kp=Kp;
	p->P_Control.Ki=Ki;
	p->P_Control.Kd=Kd;
}

void FrontFeedParamter_Updata(PID_t *p,float Kf)
{
	if(!p)
		return;
	p->P_FrontFeed.Kf=Kf;
}

void PID_IntegralLimitUpdate(PID_t *p, float IntegralMin, float IntegralMax) 
{
	if(!p)
		return;
	p->P_Control.IntegralMin=IntegralMin;
	p->P_Control.IntegralMax=IntegralMax;
}

void Pos_PID(PID_t *p)
{
	p->P_FrontFeed.Error=p->P_Myset.Target-p->P_FrontFeed.LastPoint;
	p->P_Private.error=p->P_Myset.Target-p->P_Myset.Actual;
	p->P_Private.integral+=p->P_Private.error;
	p->P_Private.integral=Limit(p->P_Private.integral,p->P_Control.IntegralMax,p->P_Control.IntegralMin);
	//带前馈的pid输出
	p->P_Myset.Out=p->P_Control.Kp*p->P_Private.error+p->P_Control.Ki*p->P_Private.integral+p->P_Control.Kd*(p->P_Private.error-p->P_Private.last_error)+p->P_FrontFeed.FrontFeedOut;
	//输出限幅:
	p->P_Myset.Out=Limit(p->P_Myset.Out,-1.f,1.f);
	p->P_Private.last_error=p->P_Private.error;
	p->P_FrontFeed.LastPoint=p->P_Myset.Target;
}

void Posold_PID(PID_t *p)
{
	p->P_FrontFeed.Error=p->P_Myset.Target-p->P_FrontFeed.LastPoint;
	p->P_Private.error=p->P_Myset.Target-p->P_Myset.Actual;
	p->P_Private.integral+=p->P_Private.error;
	p->P_Private.integral=Limit(p->P_Private.integral,p->P_Control.IntegralMax,p->P_Control.IntegralMin);
	//带前馈的pid输出
	p->P_Myset.Out=p->P_Control.Kp*p->P_Private.error+p->P_Control.Ki*p->P_Private.integral+p->P_Control.Kd*(p->P_Private.error-p->P_Private.last_error)+p->P_FrontFeed.FrontFeedOut;
	//输出限幅:
	p->P_Myset.Out=Limit(p->P_Myset.Out,-29999.0,29999.0);
	p->P_Private.last_error=p->P_Private.error;
	p->P_FrontFeed.LastPoint=p->P_Myset.Target;
}

void PosoldMiddle_PID(PID_t *p)
{
	p->P_FrontFeed.Error=p->P_Myset.Target-p->P_FrontFeed.LastPoint;
	p->P_Private.error=p->P_Myset.Target-p->P_Myset.Actual;
	p->P_Private.integral+=p->P_Private.error;
	p->P_Private.integral=Limit(p->P_Private.integral,p->P_Control.IntegralMax,p->P_Control.IntegralMin);
	//带前馈的pid输出
	p->P_Myset.Out=p->P_Control.Kp*p->P_Private.error+p->P_Control.Ki*p->P_Private.integral+p->P_Control.Kd*(p->P_Private.error-p->P_Private.last_error)+p->P_FrontFeed.FrontFeedOut;
	//输出限幅:
	p->P_Myset.Out=Limit(p->P_Myset.Out,-16000.0,16000.0);
	p->P_Private.last_error=p->P_Private.error;
	p->P_FrontFeed.LastPoint=p->P_Myset.Target;
}

void IMUPos_PID(PID_t *p)
{
	p->P_Private.error=p->P_Myset.Target-p->P_Myset.Actual;
	p->P_Private.integral+=p->P_Private.error;
	p->P_Private.integral=Limit(p->P_Private.integral,p->P_Control.IntegralMax,p->P_Control.IntegralMin);
	//带前馈的pid输出
	p->P_Myset.Out=p->P_Control.Kp*p->P_Private.error+p->P_Control.Ki*p->P_Private.integral+p->P_Control.Kd*(p->P_Private.error-p->P_Private.last_error);
	//输出限幅:
	p->P_Myset.Out=Limit(p->P_Myset.Out,-4500.0f,4500.0f);
	p->P_Private.last_error=p->P_Private.error;
}

void Inc_PID(PID_t* PID)
{
	PID->P_Private.error = PID->P_Myset.Target - PID->P_Myset.Actual;
	if(PID->P_FrontFeed.Kf != 0)
	{
		PID->P_FrontFeed.Error = PID->P_Myset.Target - PID->P_FrontFeed.LastPoint;
		
		PID->P_FrontFeed.LastPoint = PID->P_Myset.Target;
		
		PID->P_FrontFeed.FrontFeedOut = PID->P_FrontFeed.Kf * PID->P_FrontFeed.Error;
									
	}
	else
	{
		PID->P_FrontFeed.FrontFeedOut = 0;
	}
	float delta_out = PID->P_Control.Kp * (PID->P_Private.error - PID->P_Private.last_error) 
			+ PID->P_Control.Ki * PID->P_Private.error 
			+ PID->P_Control.Kd * (PID->P_Private.error - 2.0 * PID->P_Private.last_error + PID->P_Private.last_lasterror);
	
	PID->P_Myset.Out += delta_out + PID->P_FrontFeed.FrontFeedOut;
	PID->P_Myset.Out = Limit(PID->P_Myset.Out, -1.0, 1.0);
	PID->P_Private.last_lasterror = PID->P_Private.last_error;
	PID->P_Private.last_error = PID->P_Private.error;
}

void Inc_PIDold(PID_t* PID)
{
	PID->P_Private.error = PID->P_Myset.Target - PID->P_Myset.Actual;
	if(PID->P_FrontFeed.Kf != 0)
	{
		PID->P_FrontFeed.Error = PID->P_Myset.Target - PID->P_FrontFeed.LastPoint;
		
		PID->P_FrontFeed.LastPoint = PID->P_Myset.Target;
		
		PID->P_FrontFeed.FrontFeedOut = PID->P_FrontFeed.Kf * PID->P_FrontFeed.Error;
									
	}
	else
	{
		PID->P_FrontFeed.FrontFeedOut = 0;
	}
	float delta_out = PID->P_Control.Kp * (PID->P_Private.error - PID->P_Private.last_error) 
			+ PID->P_Control.Ki * PID->P_Private.error 
			+ PID->P_Control.Kd * (PID->P_Private.error - 2.0 * PID->P_Private.last_error + PID->P_Private.last_lasterror);
	
	PID->P_Myset.Out += delta_out + PID->P_FrontFeed.FrontFeedOut;
	PID->P_Myset.Out = Limit(PID->P_Myset.Out, -16000.0,16000.0);
	PID->P_Private.last_lasterror = PID->P_Private.last_error;
	PID->P_Private.last_error = PID->P_Private.error;
}

float low_pass_filter2(float value)
{
  static float out_last = 0; //上一次滤波值
  float out;

	float b = 2.0f * (float)pi * fc * Ts;
  alpha = b / (b + 1);
	
	
  /***************** 如果第一次进入，则给 out_last 赋值 ******************/
  static char fisrt_flag = 1;
  if (fisrt_flag == 1)
  {
    fisrt_flag = 0;
    out_last = value;
  }

  /*************************** 一阶滤波 *********************************/
  out = out_last + alpha * (value - out_last);
  out_last = out;

  return out;
}

//void Inc_PID(PID_t *p)
//{
//	p->P_FrontFeed.Error=p->P_Myset.Target-p->P_FrontFeed.LastPoint;
//	p->P_Private.error=p->P_Myset.Target-p->P_Myset.Actual;
//	if(p->P_FrontFeed.Kf != 0.0f)
//	{
//		p->P_FrontFeed.FrontFeedOut=p->P_FrontFeed.Kf*p->P_FrontFeed.Error;
//		p->P_Myset.Out+=p->P_FrontFeed.FrontFeedOut
//		+p->P_Control.Kp*(p->P_Private.error-p->P_Private.last_lasterror)
//		+p->P_Control.Ki*(p->P_Private.error)
//		+p->P_Control.Kd*(p->P_Private.error-2.0*p->P_Private.last_lasterror+p->P_Private.last_error);
//		//更新上次目标值:
//		p->P_FrontFeed.LastPoint=p->P_Myset.Target;
//	}
//	else
//	{
//		//直接计算输出值:
//		p->P_Myset.Out+=+p->P_Control.Kp*(p->P_Private.error-p->P_Private.last_lasterror)
//		+p->P_Control.Ki*(p->P_Private.error)
//		+p->P_Control.Kd*(p->P_Private.error-2.0*p->P_Private.last_lasterror+p->P_Private.last_error);
//	}
//	p->P_Myset.Out=Limit(p->P_Myset.Out,-1.0,1.0);//输出限幅，也为归一化服务
//	p->P_Private.last_lasterror=p->P_Private.last_error;
//	p->P_Private.last_error=p->P_Private.error;
//}

//void Inc_PID(PID_t *p)
//{
//	p->P_FrontFeed.Error=p->P_Myset.Target-p->P_FrontFeed.LastPoint;
//	p->P_Private.error=p->P_Myset.Target-p->P_Myset.Actual;
//	if(!p->P_FrontFeed.Kf)
//	{
//		p->P_FrontFeed.FrontFeedOut=p->P_FrontFeed.Kf*p->P_FrontFeed.Error;
//		p->P_Myset.Out+=p->P_FrontFeed.FrontFeedOut
//		+p->P_Control.Kp*(p->P_Private.error-p->P_Private.last_lasterror)
//		+p->P_Control.Ki*(p->P_Private.error)
//		+p->P_Control.Kd*(p->P_Private.error-2.0*p->P_Private.last_lasterror+p->P_Private.last_error);
//		//更新上次目标值:
//		p->P_FrontFeed.LastPoint=p->P_Myset.Target;
//	}
//	else
//	{
//		//直接计算输出值:
//		p->P_Myset.Out+=+p->P_Control.Kp*(p->P_Private.error-p->P_Private.last_lasterror)
//		+p->P_Control.Ki*(p->P_Private.error)
//		+p->P_Control.Kd*(p->P_Private.error-2.0*p->P_Private.last_lasterror+p->P_Private.last_error);
//	}
//	p->P_Myset.Out=Limit(p->P_Myset.Out,-1.0,1.0);//输出限幅，也为归一化服务
//	p->P_Private.last_lasterror=p->P_Private.last_error;
//	p->P_Private.last_error=p->P_Private.error;
//}
