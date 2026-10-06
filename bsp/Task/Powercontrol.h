//#ifndef POWERCONTROL_H
//#define POWERCONTROL_H

//#include "Sys.h"
//#include "RLS.h"

//#define minPower 50
//#define ERROR_POWER_DISTRIBUTION_SET 20
//#define PROP_POWER_DISTRIBUTION_SET 15
//#define POWER_PD_KP 50.0f              // 能量环PID比例系数
//#define POWER_PD_KD 0.2f               // 能量环PID微分系数

//#define torqueConst 0.004f

//typedef struct power_t{
//	void* motor[4];           // 电机对象指针数组 [右前, 左前, 左后, 右后]
//	float k1,k2,k3;           // 功率损耗参数 k1:速度相关损耗系数 k2:电流平方损耗系数 k3:恒定功率损耗
//	
//	//能量缓冲管理:
//	float powerBuffer;         // 当前缓冲能量值 (0-60裁判系统/0-255电容)
//	float fullBuffSet;         // 满能量设定点 (触发严格功率限制的阈值)
//	float baseBuffSet;         // 基础能量设定点 (触发基础功率限制的阈值)
//	
//	//功率限制:
//	float fullMaxPower;        // 满能量时最大功率 (能量充足时的功率上限)
//	float baseMaxPower;        // 基础能量时最大功率 (能量不足时的保守功率上限)
//	float powerUpperLimit;     // 功率上限 (系统理论最大功率)
//	float refereeMaxPower;     // 裁判系统限制功率 (来自裁判系统的实时功率限制)
//	float userConfiguredMaxPower; // 用户配置功率 (用户设定的期望功率限制)
//	
//	// 功率测量与估计
//	float measuredPower;       // 测量功率 (从传感器获取的实际功率值)
//	float estimatedPower;      // 估计功率 (通过模型计算的理论功率值)
//	float estimatedCapEnergy;  // 估计电容能量 (超级电容的剩余能量估计)
//	float maxPowerLimited;     // 最大功率上限
//	float sumPowerCmd_before_clamp;//平均功率
//	
//	float dt;
//	uint32_t lastUpdateTick;
//	
//	uint8_t rls_enabled;      // rls使能标志位
//	
//	float cap_power_limit;    // 超电功率限制数值
//	float cap_power_buffer;   // 超电设置缓冲能量
//	uint8_t cap_enable_dcdc;  // 超电电容充放电标志位
//	uint8_t cap_reset_state;  // 超电重启标志位
//	
//	uint32_t cap_elc_ok;      // 超电是否有电的标志位
//	
//	uint8_t error;            // 错误标志位 (电机断开/裁判系统断开/电容断开等错误状态)
//}power_t;

//typedef struct PowerObj{
//	float pidOutput;    // 扭矩电流命令 [-pidMaxOutput, pidMaxOutput] (PID控制器输出的扭矩电流值)
//	float curAv;        // 当前角速度 (电机当前的实际角速度，单位: rad/s)
//	float setAv;        // 目标角速度 (电机期望达到的目标角速度，单位: rad/s)
//	float pidMaxOutput; // PID最大输出限制 (PID控制器输出的扭矩电流最大值限制)
//}PowerObj;//功率控制对象：用于单个电机的功率控制参数传递

//// 功率状态结构体
//typedef struct power_status_t{
//	float userConfiguredMaxPower;
//	float effectivePower;
//	float powerLoss;
//	float efficiency;
//	uint8_t estimatedCapEnergy;
//	uint8_t error;
//}power_status_t;

//typedef struct power_pidt{
//	float Kp;
//	float Ki;
//	float Kd;
//	float error;
//	float lasterror;
//	float target;
//	float target_cap;
//	float target_nocap;
//	float actual;
//	float integral;
//	float derivative;
//	float out;
//	uint8_t energy_control_enabled;//使能能量环PID控制的标志位
//}power_pidt;

//typedef struct power_pid_t{
//	float kp;             // 比例系数
//	float kd;             // 微分系数
//	float last_error;     // 上一次误差
//	float output;         // 输出
//	float output_limit;   // 输出限制
//}power_pid_t;

//typedef struct motor_data_t{
//	float current_A;      //反馈电流值
//	float torque_Nm;      // 转矩,单位:N*m
//	float speed_rpm;      // 转速,单位:RPM
//	float speed_radps;    // 角速度,单位:rad/s
//}motor_data_t;

//extern power_t powerMannage;
//extern power_pidt Energy_PID;
//void PowerParam_Init(power_t *power,float k1,float k2,float k3);
//void Power_Init(power_t *power);
//void energyPID_Init(power_pid_t *pid,float kp,float kd,float output_limit);
//void power_get_controlled_output(power_t *power,PowerObj *objs[4],float output[4]);
//void power_set_max_power(power_t *power,float maxPower);
//void power_set_mode(power_t *power,uint8_t mode);
//void power_update(power_t *power);
//void Energy_PIDcontrol_loop(power_pidt *pid, float target, float actual);
//void Energy_PIDcontrol(void);
//void power_set_chassis_power_limit(power_t *power, float power_limit_w);
//void powerControl_IN_Chassis(void);
//void powerdaemon_task(void *argument);
//float rpm_to_rads(float rpm);
//float rpm_to_mps_simple(float rpm);
//float float_equal(float a,float b);

//#endif

#ifndef POWERCONTROL_H
#define POWERCONTROL_H

#include "Sys.h"

typedef struct power_t{
	float Buffer_PowerRef;//功率增益;
	float Power_Limit;//功率上限;
	float scale;//速度缩放因子;
	float speed_magnitude;//速度向量模长，不考虑方向，只考虑大小的总平移速度大小
	float Max_Translation;//最大平移速度,要根据实际代码的Vx和Vy计算得到具体数据
	float current_power;//当前功率（计算值）
	float PowerSum;//控制周期内的功率积分
	uint8_t count;//功率控制周期计数器
	float average_power;//一定时间窗口内的平均功率值
	float RefereeSystem_Energy;//裁判系统能量反馈
	float Energy_PowerLimit;//能量策略下的功率限制
	uint8_t cap_elc_ok;
	float V_Gains;//速度增益;
	float Z_Gains;
}power_t;

typedef struct power_pid{
	float Kp;
	float Ki;
	float Kd;
	float error;
	float lasterror;
	float target;
	float target_cap;
	float target_nocap;
	float actual;
	float integral;
	float derivative;
	float out;
	uint8_t energy_control_enabled;//使能能量环PID控制的标志位
}power_pid;

extern power_t PC;
extern power_pid Energy_PID;

void ChassisPowerControl_ParamsInit(void);
void Energy_PIDcontrol_loop(void);
void Chassis_PowerControlTest(float Vx,float Vy,float Vz,float Input_Energy);
void powerdaemon_task(void *argument);

#endif



