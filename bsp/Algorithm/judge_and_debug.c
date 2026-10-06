#include "judge_and_debug.h"

void usart1_tx_dma_init(void)
{
	//enable the DMA transfer for the receiver and tramsmit request
	//使能DMA串口接收和发送
	SET_BIT(huart1.Instance->CR3, USART_CR3_DMAR);
	SET_BIT(huart1.Instance->CR3, USART_CR3_DMAT);

	//disable DMA
	//失效DMA
	__HAL_DMA_DISABLE(&hdma_usart1_tx);

	while(hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
	{
			__HAL_DMA_DISABLE(&hdma_usart1_tx);
	}

	hdma_usart1_tx.Instance->PAR = (uint32_t) & (USART1->DR);
	hdma_usart1_tx.Instance->M0AR = (uint32_t)(NULL);
	hdma_usart1_tx.Instance->NDTR = 0;
}
void usart1_tx_dma_enable(uint8_t *data, uint16_t len)
{
    //disable DMA
    //失效DMA
    __HAL_DMA_DISABLE(&hdma_usart1_tx);

    while(hdma_usart1_tx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart1_tx);
    }

    __HAL_DMA_CLEAR_FLAG(&hdma_usart1_tx, DMA_HISR_TCIF7);

    hdma_usart1_tx.Instance->M0AR = (uint32_t)(data);
    __HAL_DMA_SET_COUNTER(&hdma_usart1_tx, len);

    __HAL_DMA_ENABLE(&hdma_usart1_tx);
}

void usart_printf(const char *fmt,...)
{
	static uint8_t tx_buf[256] = {0};
	static va_list ap;
	static uint16_t len;
	va_start(ap, fmt);
	//return length of string 
	//返回字符串长度
	len = vsprintf((char *)tx_buf, fmt, ap);
	va_end(ap);
	usart1_tx_dma_enable(tx_buf, len);
}

//回调函数:
