#ifndef DBUS_RECEIVE_H
#define DBUS_RECEIVE_H

#include "usart.h"
#include "struct_typedef.h"
#include "main.h"
#include "Sys.h"
#include "FreeRTOS.h"
#include "queue.h"  // 引入FreeRTOS队列头文件

// SBUS接收缓冲区长度（需大于最大数据索引+1，此处36足够）
#define SBUS_RX_BUF_LEN 36
// RC通道中点值（根据遥控器协议定义，通常为1024）
#define RC_MIDDLE 1024

/**
 * @brief RC遥控器数据结构体
 * 包含通道、鼠标、键盘和保留数据
 */
typedef struct
{
    struct
    { 
        uint16_t ch0;   // 通道0（如横滚）
        uint16_t ch1;   // 通道1（如俯仰）
        uint16_t ch2;   // 通道2（如油门）
        uint16_t ch3;   // 通道3（如偏航）
        uint8_t s1;     // 开关1
        uint8_t s2;     // 开关2
    }rc;
    
    struct 
    {
        int16_t x;      // 鼠标X轴
        int16_t y;      // 鼠标Y轴
        int16_t z;      // 鼠标Z轴
        uint16_t press_l; // 鼠标左键
        uint16_t press_r; // 鼠标右键
    }mouse;
    
    struct
    {
        uint16_t w;     // W键
        uint16_t s;     // S键
        uint16_t a;     // A键
        uint16_t d;     // D键
        uint16_t q;     // Q键
        uint16_t e;     // E键
        uint16_t shift; // Shift键
        uint16_t ctrl;  // Ctrl键
        uint16_t r;     // R键
        uint16_t f;     // F键
        uint16_t g;     // G键
        uint16_t z;     // Z键
        uint16_t x;     // X键
        uint16_t c;     // C键
        uint16_t v;     // V键
        uint16_t b;     // B键
        uint16_t ass;   // 其他键
    }key;
    
    struct
    {
        uint16_t che;   // 保留数据（如拨轮）
    }keep;

}RC_Ctl_t;

extern int32_t RC_Protection;
extern QueueHandle_t RC_MouseCC;          // RC数据消息队列
extern QueueHandle_t RC_MouseCC2;
extern QueueHandle_t RC_MouseCC3;
extern volatile RC_Ctl_t RC_Ctl;          // 全局RC控制数据
extern UART_HandleTypeDef huart3;         // USART3句柄
extern DMA_HandleTypeDef hdma_usart3_rx;  // USART3的DMA句柄

// 函数声明
void dbus_init(uint8_t *rx1_buf, uint8_t *rx2_buf, uint16_t dma_buf_num);
void RC_Init(void);

#endif
