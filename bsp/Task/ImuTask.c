#include "ImuTask.h"
#include "EKF.h"
#include "BMI088driver.h"

INS_t INS;
IMU_Param_t IMU_Param;
AttitudeData Attitude;
struct MAHONY_FILTER_t mahony;
Axis3f Gyro,Accel;

uint8_t bmi088_err;

const float xb[3] = {1, 0, 0};
const float yb[3] = {0, 1, 0};
const float zb[3] = {0, 0, 1};

const float gravity[3] = {0, 0, 9.81f};

uint32_t INS_DWT_Count = 0;
float ins_dt = 0.0f;
static float dt = 0, t = 0;
uint8_t ins_debug_mode = 0;
float RefTemp = 40;
float ins_time;

static float LastDegYaw;        // 上一次角度制偏航角
static float ThisDegYaw;        // 本次角度制偏航角
static uint8_t YawFirstFlag = 1;// 第一次接收数据标志位
static float YawR = 0.0f;       // 偏航角圈数计数

static void IMU_Param_Correction(IMU_Param_t *param, float gyro[3], float accel[3]);

void INS_Init(void)
{
	IMU_Param.scale[AX] = 1;
	IMU_Param.scale[AY] = 1;
	IMU_Param.scale[AZ] = 1;
	IMU_Param.Yaw = 0;
	IMU_Param.Pitch = 0;
	IMU_Param.Roll = 0;
	IMU_Param.flag = 1;

	INS.AccelLPF = 0.0085;
}

void INS_Task(float dt)
{
	static uint32_t last_update_time = 0;
	const float gravity[3] = {0, 0, 9.81f};
	
	// 获取当前系统时间（毫秒）
	uint32_t now = HAL_GetTick();
	
	// 计算实际时间间隔（防止任务抖动）
	float actual_dt = (now - last_update_time) * 0.001f;  // 转换为秒
	last_update_time = now;

	// 更新传感器数据
	INS.Accel[AX] = bmi088_param[0].accel[AX];
	INS.Accel[AY] = bmi088_param[0].accel[AY];
	INS.Accel[AZ] = bmi088_param[0].accel[AZ];
	INS.Gyro[AX] = bmi088_param[0].gyro[AX];
	INS.Gyro[AY] = bmi088_param[0].gyro[AY];
	INS.Gyro[AZ] = bmi088_param[0].gyro[AZ];

	// 安装误差校正（实际使用时需启用）
	IMU_Param_Correction(&IMU_Param, INS.Gyro, INS.Accel);

	// 核心函数：EKF更新四元数（使用实际时间间隔）
	IMU_QuaternionEKF_Update(INS.Gyro[AX], INS.Gyro[AY], INS.Gyro[AZ], 
													 INS.Accel[AX], INS.Accel[AY], INS.Accel[AZ], 
													 actual_dt);

	memcpy(INS.q, QEKF_INS.q, sizeof(QEKF_INS.q));

	// 坐标系转换
	BodyFrameToEarthFrame(xb, INS.xn, INS.q);
	BodyFrameToEarthFrame(yb, INS.yn, INS.q);
	BodyFrameToEarthFrame(zb, INS.zn, INS.q);

	// 重力补偿和运动加速度计算
	float gravity_b[3];
	EarthFrameToBodyFrame(gravity, gravity_b, INS.q);
	
	for (uint8_t i = 0; i < 3; i++) {
			// 使用实际时间间隔进行滤波
			float alpha = actual_dt / (INS.AccelLPF + actual_dt);
			INS.MotionAccel_b[i] = (INS.Accel[i] - gravity_b[i]) * alpha 
													 + INS.MotionAccel_b[i] * (1 - alpha);
	}
	
	BodyFrameToEarthFrame(INS.MotionAccel_b, INS.MotionAccel_n, INS.q);

	// 获取最终姿态数据
	INS.Yaw = QEKF_INS.Yaw;
	INS.Pitch = QEKF_INS.Pitch;
	INS.Roll = QEKF_INS.Roll;
	INS.YawTotalAngle = QEKF_INS.YawTotalAngle;
}


/**
 * @brief          Transform 3dvector from BodyFrame to EarthFrame
 * @param[1]       vector in BodyFrame
 * @param[2]       vector in EarthFrame
 * @param[3]       quaternion
 */
void BodyFrameToEarthFrame(const float *vecBF, float *vecEF, float *q)
{
    vecEF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecBF[0] +
                       (q[1] * q[2] - q[0] * q[3]) * vecBF[1] +
                       (q[1] * q[3] + q[0] * q[2]) * vecBF[2]);

    vecEF[1] = 2.0f * ((q[1] * q[2] + q[0] * q[3]) * vecBF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecBF[1] +
                       (q[2] * q[3] - q[0] * q[1]) * vecBF[2]);

    vecEF[2] = 2.0f * ((q[1] * q[3] - q[0] * q[2]) * vecBF[0] +
                       (q[2] * q[3] + q[0] * q[1]) * vecBF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecBF[2]);
}

/**
 * @brief          Transform 3dvector from EarthFrame to BodyFrame
 * @param[1]       vector in EarthFrame
 * @param[2]       vector in BodyFrame
 * @param[3]       quaternion
 */
void EarthFrameToBodyFrame(const float *vecEF, float *vecBF, float *q)
{
    vecBF[0] = 2.0f * ((0.5f - q[2] * q[2] - q[3] * q[3]) * vecEF[0] +
                       (q[1] * q[2] + q[0] * q[3]) * vecEF[1] +
                       (q[1] * q[3] - q[0] * q[2]) * vecEF[2]);

    vecBF[1] = 2.0f * ((q[1] * q[2] - q[0] * q[3]) * vecEF[0] +
                       (0.5f - q[1] * q[1] - q[3] * q[3]) * vecEF[1] +
                       (q[2] * q[3] + q[0] * q[1]) * vecEF[2]);

    vecBF[2] = 2.0f * ((q[1] * q[3] + q[0] * q[2]) * vecEF[0] +
                       (q[2] * q[3] - q[0] * q[1]) * vecEF[1] +
                       (0.5f - q[1] * q[1] - q[2] * q[2]) * vecEF[2]);
}

/**
 * @brief reserved.用于修正IMU安装误差与标度因数误差,即陀螺仪轴和云台轴的安装偏移
 *
 *
 * @param param IMU参数
 * @param gyro  角速度
 * @param accel 加速度
 */
static void IMU_Param_Correction(IMU_Param_t *param, float gyro[3], float accel[3])
{
    static float lastYawOffset, lastPitchOffset, lastRollOffset;
    static float c_11, c_12, c_13, c_21, c_22, c_23, c_31, c_32, c_33;
    float cosPitch, cosYaw, cosRoll, sinPitch, sinYaw, sinRoll;

    if (fabsf(param->Yaw - lastYawOffset) > 0.001f ||
        fabsf(param->Pitch - lastPitchOffset) > 0.001f ||
        fabsf(param->Roll - lastRollOffset) > 0.001f || param->flag)
    {
        cosYaw = arm_cos_f32(param->Yaw / 57.295779513f);
        cosPitch = arm_cos_f32(param->Pitch / 57.295779513f);
        cosRoll = arm_cos_f32(param->Roll / 57.295779513f);
        sinYaw = arm_sin_f32(param->Yaw / 57.295779513f);
        sinPitch = arm_sin_f32(param->Pitch / 57.295779513f);
        sinRoll = arm_sin_f32(param->Roll / 57.295779513f);

        // 1.yaw(alpha) 2.pitch(beta) 3.roll(gamma)
        c_11 = cosYaw * cosRoll + sinYaw * sinPitch * sinRoll;
        c_12 = cosPitch * sinYaw;
        c_13 = cosYaw * sinRoll - cosRoll * sinYaw * sinPitch;
        c_21 = cosYaw * sinPitch * sinRoll - cosRoll * sinYaw;
        c_22 = cosYaw * cosPitch;
        c_23 = -sinYaw * sinRoll - cosYaw * cosRoll * sinPitch;
        c_31 = -cosPitch * sinRoll;
        c_32 = sinPitch;
        c_33 = cosPitch * cosRoll;
        param->flag = 0;
    }
    float gyro_temp[3];
    for (uint8_t i = 0; i < 3; i++)
        gyro_temp[i] = gyro[i] * param->scale[i];

    gyro[AX] = c_11 * gyro_temp[AX] +
              c_12 * gyro_temp[AY] +
              c_13 * gyro_temp[AZ];
    gyro[AY] = c_21 * gyro_temp[AX] +
              c_22 * gyro_temp[AY] +
              c_23 * gyro_temp[AZ];
    gyro[AZ] = c_31 * gyro_temp[AX] +
              c_32 * gyro_temp[AY] +
              c_33 * gyro_temp[AZ];

    float accel_temp[3];
    for (uint8_t i = 0; i < 3; i++)
        accel_temp[i] = accel[i];

    accel[AX] = c_11 * accel_temp[AX] +
               c_12 * accel_temp[AY] +
               c_13 * accel_temp[AZ];
    accel[AY] = c_21 * accel_temp[AX] +
               c_22 * accel_temp[AY] +
               c_23 * accel_temp[AZ];
    accel[AZ] = c_31 * accel_temp[AX] +
               c_32 * accel_temp[AY] +
               c_33 * accel_temp[AZ];

    lastYawOffset = param->Yaw;
    lastPitchOffset = param->Pitch;
    lastRollOffset = param->Roll;
}

void INS_MahonyInit(void)
{
	mahony.dt = 0.001f;
	mahony_init(&mahony,1.0f,0.0f,0.001f);
	INS.AccelLPF=0.0089f;
	INS.GyroLPF=0.0089f;
}

void INS_MahonyTask(float dt)
{
	static uint32_t last_update_time = 0;
	uint32_t now = HAL_GetTick();
	
	// 计算实际时间间隔（防止任务抖动）
	float actual_dt = (now - last_update_time) * 0.001f;  // 转换为秒
	last_update_time = now;
	
	// 更新传感器数据
	Accel.x = bmi088_param[0].accel[0];
	Accel.y = bmi088_param[0].accel[1];
	Accel.z = bmi088_param[0].accel[2];
	Gyro.x = bmi088_param[0].gyro[0];
	Gyro.y = bmi088_param[0].gyro[1];
	Gyro.z = bmi088_param[0].gyro[2];
	
	// 设置实际时间间隔
	mahony.dt = actual_dt;
	
	// Mahony 算法处理流程
	mahony_input(&mahony, Gyro, Accel);
	mahony_update(&mahony);
	mahony_output(&mahony);
	RotationMatrix_update(&mahony);
}

void IMU_temp_PWM(uint16_t pwm)
{
	TIM10->CCMR1=pwm;
}

//陀螺仪温控加读取角速度任务:
void IMUTemp_ControlTask(void *argument)
{
	TickType_t xLastWakeTime=xTaskGetTickCount();
	const TickType_t xFrequency=pdMS_TO_TICKS(50);
	while(1)
	{
		bmi088_param[0].Control_t.Pos.P_Myset.Target = 40.0f;
		bmi088_param[0].Control_t.Pos.P_Myset.Actual = BMI088.Temperature;
		IMUPos_PID(&bmi088_param[0].Control_t.Pos);
		if(bmi088_param[0].Control_t.Pos.P_Myset.Out<0.0f)
		{
			bmi088_param[0].Control_t.Pos.P_Myset.Out=0.0f;
		}
		if(BMI088.Temperature>45.0f) //过热保护
		{
			bmi088_param[0].pwm_value=0;  
		}
		bmi088_param[0].pwm_value=(uint16_t)(bmi088_param[0].Control_t.Pos.P_Myset.Out*4500.0f);
		IMU_temp_PWM(bmi088_param[0].pwm_value);
		vTaskDelayUntil(&xLastWakeTime,xFrequency);
	}
}

void IMUAngle_Task(void *argument)
{
	TickType_t xLastWakeTime=xTaskGetTickCount();
	TickType_t last_tick = xTaskGetTickCount();
	const TickType_t xFrequency=pdMS_TO_TICKS(1);
	while(1)
	{
		ins_dt=DWT_GetDeltaT(&INS_DWT_Count);
		
		// 执行传感器读取:
		BMI088_Read(&BMI088);
		
		//计算实际时间间隔:
		TickType_t current_tick = xTaskGetTickCount();
		INS.actual_dt=(current_tick - last_tick) * portTICK_PERIOD_MS * 0.001f;//转换为秒
		last_tick=current_tick;
		
		//设置mahony时间步长:
		mahony.dt=ins_dt;
		
		//准备传感器数据:
    INS.Accel[AX] = BMI088.Accel[AX];
    INS.Accel[AY] = BMI088.Accel[AY];
    INS.Accel[AZ] = BMI088.Accel[AZ];
	  Accel.x=BMI088.Accel[0];
	  Accel.y=BMI088.Accel[1];
		Accel.z=BMI088.Accel[2];
    INS.Gyro[AX] = BMI088.Gyro[AX];
    INS.Gyro[AY] = BMI088.Gyro[AY];
    INS.Gyro[AZ] = BMI088.Gyro[AZ];
  	Gyro.x=BMI088.Gyro[0];
		Gyro.y=BMI088.Gyro[1];
		Gyro.z=BMI088.Gyro[2];
		
		//Mahony滤波器更新:
		mahony_input(&mahony,Gyro,Accel);
		mahony_update(&mahony);
		mahony_output(&mahony);
		RotationMatrix_update(&mahony);
		
		INS.q[0]=mahony.q0;
		INS.q[1]=mahony.q1;
		INS.q[2]=mahony.q2;
		INS.q[3]=mahony.q3;
		
		float gravity_b[3];
		EarthFrameToBodyFrame(gravity,gravity_b,INS.q);
    for (uint8_t i = 0; i < 3; i++)
    {
      INS.MotionAccel_b[i] = (INS.Accel[i] - gravity_b[i]) * ins_dt / (INS.AccelLPF + ins_dt) 
														+ INS.MotionAccel_b[i] * INS.AccelLPF / (INS.AccelLPF + ins_dt); 			
		}
		for (uint8_t i = 0; i < 3; i++) 
		{
			INS.GyroFiltered[i] = BMI088.Gyro[i] * ins_dt / (INS.GyroLPF + ins_dt) 
													+ INS.GyroFiltered[i] * INS.GyroLPF / (INS.GyroLPF + ins_dt);
		}
		BodyFrameToEarthFrame(INS.MotionAccel_b,INS.MotionAccel_n,INS.q);
		
		if(ins_time>2000.0f)
		{
			INS.Roll=mahony.roll;
			INS.Pitch=mahony.pitch;
			INS.Yaw=mahony.yaw;
			if (INS.Yaw - INS.YawAngleLast > 3.1415926f)
			{
					INS.YawRoundCount--;
			}
			else if (INS.Yaw - INS.YawAngleLast < -3.1415926f)
			{
					INS.YawRoundCount++;
			}
			INS.YawTotalAngle = 6.283f* INS.YawRoundCount + INS.Yaw;
			INS.YawAngleLast = INS.Yaw;
		}
		else
		{
			ins_time++;
		}
		
		vTaskDelayUntil(&xLastWakeTime,xFrequency);
	}
}



