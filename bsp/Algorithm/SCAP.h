#ifndef __SCAP_H
#define __SCAP_H

#include "Sys.h"

#define JUDGE_ChassisPowerLimit 60   //还未建立与裁判系统的通信，先拿这个尝试

typedef  __packed struct{
	uint8_t enableDCDC: 1;                  // 允许启动DCDC
	uint8_t systemRestart: 1;               // 系统重启
	uint8_t resv0: 3;
	uint8_t clearError: 1;                  // 手动清除可清除的错误
	uint8_t enableActiveChargingLimit: 1;   // 是否启用主动充电限制
	uint8_t useNewFeedbackMessage: 1;       // 是否使用新的反馈消息格式

	uint16_t refereePowerLimit;             // 裁判限制功率，单位W
	uint16_t refereeEnergyBuffer;           // 裁判能量缓冲，单位J
	uint8_t activeChargingLimitRatio;       // 主动充电限制比例（能量），0-255
	int16_t resv2;
//	uint8_t enableDCDC:1;
//	uint8_t systemRestart:1;
//	uint8_t resv0:6;
//	uint16_t feedbackRefereePowerLimit;
//	uint16_t feedbackRefereeEnergyBuffer;
//	uint8_t resv1[3];
}SCAP_Tx;

typedef struct SCAP_Rx{
	uint8_t errorCode;
	float chassisPower;
	uint16_t chassisPowerLimit;
	uint8_t CapEnergy;
}SCAP_Rx;

extern SCAP_Rx SCAPrx;
uint8_t Superpower_cap_Init(float power_limit,float buffer,uint8_t state,uint8_t reset);
uint8_t SuperpowerCap_set(float power_limit,float buffer);
void SCAP_Init(void);
uint8_t Superpower_cap_Init2(float power_limit,float buffer,uint8_t state,uint8_t reset);

#endif
