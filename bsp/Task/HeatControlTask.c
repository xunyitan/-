#include "HeatControlTask.h"

ShootingHeatControl_t ShootingHeatControl;
ShootHeatControl_PID_t ShootHeatControl_PID;

//枪口热量控制任务，灵感源自于同济大学开源。
void ShootPowerControlTask(void)
{
	ShootingHeatControl.shootingColdSpeed=(float)robot_state.shooter_heat0_cooling_rate;
	ShootingHeatControl.RemainingHeatValue=(float)robot_state.shooter_heat0_cooling_limit-
	power_heat_data_t.shooter_heat0;
	ShootingHeatControl.Constparam=10.0f;
	if(ShootingHeatControl.shootingTime==0)
	{
	  ShootingHeatControl.CanShootTime_cmd=(ShootingHeatControl.RemainingHeatValue+2*ShootingHeatControl.shootingColdSpeed)*10;
		ShootingHeatControl.CanShootTime_cmd=Data_Clipping(ShootingHeatControl.CanShootTime_cmd,200,2000);
		if(ShootingHeatControl.RemainingHeatValue<100)
		{
			ShootingHeatControl.ShootSpeed=(10*ShootingHeatControl.RemainingHeatValue-ShootingHeatControl.shootingColdSpeed
			-3*ShootingHeatControl.Constparam)/(ShootingHeatControl.Constparam*(ShootingHeatControl.CanShootTime_cmd/100.0f))
			+ShootingHeatControl.shootingColdSpeed/ShootingHeatControl.Constparam;
		}
		else
		{
			ShootingHeatControl.ShootSpeed=(10*ShootingHeatControl.RemainingHeatValue-ShootingHeatControl.shootingColdSpeed
			-5*ShootingHeatControl.Constparam)/(ShootingHeatControl.Constparam*(ShootingHeatControl.CanShootTime_cmd/100.0f))
			+ShootingHeatControl.shootingColdSpeed/ShootingHeatControl.Constparam;
		}
	}
	else if(0<ShootingHeatControl.shootingTime&&ShootingHeatControl.shootingTime<ShootingHeatControl.CanShootTime_cmd)
	{
		ShootingHeatControl.tar2006Speed=ShootingHeatControl.ShootSpeed*2*PI/7;
		ShootingHeatControl.tar2006Speed=Data_Clipping(ShootingHeatControl.tar2006Speed,0,18);
	}
	else
	{
		ShootingHeatControl.tar2006Speed=(ShootingHeatControl.shootingColdSpeed/ShootingHeatControl.RemainingHeatValue)
		*2*PI/7;
		ShootingHeatControl.tar2006Speed=Data_Clipping(ShootingHeatControl.tar2006Speed,0,18);
	}
	if(ShootingHeatControl.shootingTime<ShootingHeatControl.CanShootTime_cmd)
	{
		ShootingHeatControl.shootingTime++;
	}
}

void HeatControl_Init(void)
{
	ShootHeatControl_PID.targetHave=220.0f;
	ShootHeatControl_PID.targetHavenot=10.0f;
	
	ShootHeatControl_PID.Kp=1.2;
	ShootHeatControl_PID.Ki=0;
	ShootHeatControl_PID.Kd=0;
	
	ShootHeatControl_PID.outmax=(float)robot_state.shooter_heat0_cooling_limit;
	ShootHeatControl_PID.outmin=-(float)robot_state.shooter_heat0_cooling_limit;
}

//这两个是自己写的：
void HeatControlPID(void)
{
	HeatControl_Init();
	
	ShootHeatControl_PID.Shoot_is_ok=((float)power_heat_data_t.shooter_heat0/(float)robot_state.shooter_heat0_cooling_limit)>0.6f;
	ShootHeatControl_PID.target=ShootHeatControl_PID.Shoot_is_ok?ShootHeatControl_PID.targetHave:ShootHeatControl_PID.targetHavenot;
	ShootHeatControl_PID.actual=power_heat_data_t.shooter_heat0;
	ShootHeatControl_PID.error=ShootHeatControl_PID.target-ShootHeatControl_PID.actual;
	ShootHeatControl_PID.error+=ShootHeatControl_PID.integral;
	
	ShootHeatControl_PID.integral=Data_Clipping(ShootHeatControl_PID.integral,-1000,1000);
	
	ShootHeatControl_PID.out=ShootHeatControl_PID.Kp*ShootHeatControl_PID.error+ShootHeatControl_PID.Ki*ShootHeatControl_PID.integral+
	ShootHeatControl_PID.Kd*(ShootHeatControl_PID.error-ShootHeatControl_PID.lasterror);
	
	ShootHeatControl_PID.out=Data_Clipping(ShootHeatControl_PID.out,ShootHeatControl_PID.outmin,ShootHeatControl_PID.outmax);
	
	ShootHeatControl_PID.lasterror=ShootHeatControl_PID.error;
}

void HeatControlTask(void)
{
	if(((float)power_heat_data_t.shooter_heat0/(float)robot_state.shooter_heat0_cooling_limit)>=0.8f)
	{
		ShootingHeatControl.tar2006Speed=0;
	}
	if(((float)power_heat_data_t.shooter_heat0/(float)robot_state.shooter_heat0_cooling_limit)<0.8f)
	{
		ShootingHeatControl.tar2006Speed=-3500;
	}
	if(shoot_data_t.bullet_speed>=24.8)
	{
		ShootingHeatControl.FW3508Speed=0;
	}
	if(shoot_data_t.bullet_speed<24.8)
	{
		ShootingHeatControl.FW3508Speed=6900;//7000
	}
}
