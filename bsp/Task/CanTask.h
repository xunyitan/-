#ifndef CANTASK_H
#define CANTASK_H

#include "Sys.h"

typedef struct CAN_TxMessage{
	CAN_TxHeaderTypeDef Header;
	uint8_t data[8];
}CAN_TxMessage;

void CAN1_Send_Queue_Init(void);
void CAN2_Send_Queue_Init(void);
void CAN1_TO_Queue(uint32_t StdID,uint8_t DLC,uint8_t* Data);
void CAN2_TO_Queue(uint32_t StdID,uint8_t DLC,uint8_t* Data);
void CAN1_Task(void *argument);
void CAN2_Task(void *argument);

#endif
