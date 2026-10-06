#include "dbus_receive.h"

// 双缓冲区存储SBUS数据（2个缓冲区，每个长度为SBUS_RX_BUF_LEN）
static uint8_t sbus_rx_buffer[2][SBUS_RX_BUF_LEN];
//遥控器保护标志位，要求每当进入一次中断的话就置为5(可以随意设置，但尽量往小的设置)
int32_t RC_Protection;
// 全局RC控制结构体
volatile RC_Ctl_t RC_Ctl;
// 用于传输RC数据的消息队列
QueueHandle_t RC_MouseCC;
QueueHandle_t RC_MouseCC2;
QueueHandle_t RC_MouseCC3;
/**
 * @brief  初始化DBUS的DMA接收
 * @param  rx1_buf: 第一个接收缓冲区
 * @param  rx2_buf: 第二个接收缓冲区
 * @param  dma_buf_num: 每个缓冲区的长度
 */
void dbus_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num)
{
    // 使能USART3的DMA接收
    SET_BIT(huart3.Instance->CR3, USART_CR3_DMAR);

    // 使能空闲中断（用于检测一帧数据接收完成）
    __HAL_UART_ENABLE_IT(&huart3, UART_IT_IDLE);

    // 先禁用DMA，确保配置前DMA处于停止状态
    __HAL_DMA_DISABLE(&hdma_usart3_rx);
    while(hdma_usart3_rx.Instance->CR & DMA_SxCR_EN)
    {
        __HAL_DMA_DISABLE(&hdma_usart3_rx);
    }

    // 配置DMA的外设地址（USART3的数据寄存器）
    hdma_usart3_rx.Instance->PAR = (uint32_t) & (USART3->DR);
    // 配置DMA的内存缓冲区1地址
    hdma_usart3_rx.Instance->M0AR = (uint32_t)(rx1_buf);
    // 配置DMA的内存缓冲区2地址（双缓冲区模式）
    hdma_usart3_rx.Instance->M1AR = (uint32_t)(rx2_buf);
    // 配置DMA传输的数据长度
    hdma_usart3_rx.Instance->NDTR = dma_buf_num;
    // 使能双缓冲区模式
    SET_BIT(hdma_usart3_rx.Instance->CR, DMA_SxCR_DBM);

    // 使能DMA
    __HAL_DMA_ENABLE(&hdma_usart3_rx);
}

/**
 * @brief  初始化RC遥控器（对外接口）
 */
void RC_Init(void)
{
	// 调用dbus_init初始化双缓冲区（修正原RC_init的函数名错误）
	dbus_init(sbus_rx_buffer[0], sbus_rx_buffer[1], SBUS_RX_BUF_LEN);
	// 初始化消息队列（用于ISR和任务间传递数据）
	RC_MouseCC=xQueueCreate(50,sizeof(RC_Ctl_t));
	RC_MouseCC2=xQueueCreate(50,sizeof(RC_Ctl_t));
	RC_MouseCC3=xQueueCreate(50,sizeof(RC_Ctl_t));
}

/**
 * @brief  解析SBUS数据到RC控制结构体
 * @param  RC_Ctrl: 输出参数，解析后的RC数据
 * @param  Buffer: 输入参数，接收到的SBUS原始数据缓冲区
 */
static void process_DBUS(RC_Ctl_t *RC_Ctrl, uint8_t *Buffer)
{
	if(RC_Ctrl == NULL || Buffer == NULL)
			return;
	
	//解析通道数据（修正数组访问方式：使用Buffer而非全局变量，解决类型不匹配）
	RC_Ctrl->rc.ch0 = (Buffer[0] | (Buffer[1] << 8)) & 0x07FF;
//	RC_Ctrl->rc.ch0-=1024;
	RC_Ctrl->rc.ch1 = ((Buffer[1] >> 3) | (Buffer[2] << 5)) & 0x07FF;
	RC_Ctrl->rc.ch2 = ((Buffer[2] >> 6) | (Buffer[3] << 2) | (Buffer[4] << 10)) & 0x07FF;
	RC_Ctrl->rc.ch3 = ((Buffer[4] >> 1) | (Buffer[5] << 7)) & 0x07FF;
	
	
	
	//解析开关数据
	RC_Ctrl->rc.s1 = ((Buffer[5] >> 4) & 0x0C) >> 2;
	RC_Ctrl->rc.s2 = (Buffer[5] >> 4) & 0x03;
	
	//解析鼠标数据
	RC_Ctrl->mouse.x = Buffer[6] | (Buffer[7] << 8);
	RC_Ctrl->mouse.y = Buffer[8] | (Buffer[9] << 8);
	RC_Ctrl->mouse.z = Buffer[10] | (Buffer[11] << 8);
	
	//解析电位器数据
	RC_Ctrl->keep.che = Buffer[16] | (Buffer[17] << 8);
	
	RC_Ctrl->mouse.press_l=Buffer[12];
	RC_Ctrl->mouse.press_r=Buffer[13];
	RC_Ctrl->key.ass=Buffer[14]|(Buffer[15]<<8);
	
	RC_Ctrl->key.w=RC_Ctrl->key.ass&0x01;
	RC_Ctrl->key.s=(RC_Ctrl->key.ass&0x02)>>1;
	RC_Ctrl->key.a=(RC_Ctrl->key.ass&0x04)>>2;
	RC_Ctrl->key.d=(RC_Ctrl->key.ass&0x08)>>3;
	RC_Ctrl->key.shift=(RC_Ctrl->key.ass&0x10)>>4;
	RC_Ctrl->key.ctrl=(RC_Ctrl->key.ass&0x20)>>5;
	RC_Ctrl->key.q=(RC_Ctrl->key.ass&0x40)>>6;
	RC_Ctrl->key.e=(RC_Ctrl->key.ass&0x80)>>7;
	RC_Ctrl->key.r=(RC_Ctrl->key.ass&0x100)>>8;
	RC_Ctrl->key.f=(RC_Ctrl->key.ass&0x200)>>9;
	RC_Ctrl->key.g=(RC_Ctrl->key.ass&0x400)>>10;
	RC_Ctrl->key.z=(RC_Ctrl->key.ass&0x800)>>11;
	RC_Ctrl->key.x=(RC_Ctrl->key.ass&0x1000)>>12;
	RC_Ctrl->key.c=(RC_Ctrl->key.ass&0x2000)>>13;
	RC_Ctrl->key.v=(RC_Ctrl->key.ass&0x4000)>>14;
	RC_Ctrl->key.b=(RC_Ctrl->key.ass&0x8000)>>15;
}

uint8_t tiu;

/**
 * @brief  USART3中断服务函数（处理空闲中断）
 */
void USART3_IRQHandler(void)
{
	if (__HAL_UART_GET_FLAG(&huart3, UART_FLAG_IDLE) != RESET)
	{
		RC_Ctl_t rc_temp;  // 临时变量存储解析后的数据
		BaseType_t xHigherPriorityTaskWoken = pdFALSE;
		
		RC_Protection=5;
		
		// 终止当前DMA传输（防止数据覆盖）
		HAL_DMA_Abort(&hdma_usart3_rx);
		
		// 进入临界区保护数据解析
		UBaseType_t uxSavedInterruptStatus = taskENTER_CRITICAL_FROM_ISR();
		
		// 判断当前DMA使用的缓冲区（双缓冲区切换）
		uint8_t *current_buf = (hdma_usart3_rx.Instance->CR & DMA_SxCR_CT) ? 
													sbus_rx_buffer[1] : sbus_rx_buffer[0];
		// 解析当前缓冲区的数据
		process_DBUS(&rc_temp, current_buf);
		
		// 退出临界区
		taskEXIT_CRITICAL_FROM_ISR(uxSavedInterruptStatus);
		
		// 将解析后的数据发送到消息队列（非阻塞）
		xQueueSendFromISR(RC_MouseCC,&rc_temp,&xHigherPriorityTaskWoken);
		xQueueSendFromISR(RC_MouseCC2,&rc_temp,&xHigherPriorityTaskWoken);
		xQueueSendFromISR(RC_MouseCC3,&rc_temp,&xHigherPriorityTaskWoken);
		// 清除空闲中断标志
		__HAL_UART_CLEAR_IDLEFLAG(&huart3);
		
		// 重新启动DMA接收（双缓冲区模式会自动切换缓冲区）
		hdma_usart3_rx.Instance->NDTR = SBUS_RX_BUF_LEN;//NDTR
		__HAL_DMA_ENABLE(&hdma_usart3_rx);
	}
	else
	{
		tiu=1;
		// 其他中断交给HAL库处理
		HAL_UART_IRQHandler(&huart3);
	}
}
