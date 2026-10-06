#ifndef JUDGE_AND_DEBUG_H
#define JUDGE_AND_DEBUG_H

#include "main.h"
#include "usart.h"
#include <stdio.h>
#include <stdarg.h>
#include "string.h"
#include "struct_typedef.h"

//裁判系统标识符:

extern UART_HandleTypeDef huart1;
extern DMA_HandleTypeDef hdma_usart1_tx;

void usart1_tx_dma_init(void);
void usart1_tx_dma_enable(uint8_t *data, uint16_t len);
void usart_printf(const char *fmt,...);

#endif
