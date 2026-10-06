#ifndef SYS_H
#define SYS_H

#include "main.h"
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "cmsis_os.h"
#include "FreeRTOS.h"
#include "task.h"
#include "queue.h"
#include "can.h"
#include "dma.h"
#include "spi.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include <stdint.h>
#include <stdbool.h>
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */
#include "arm_math.h"
#include <math.h>
#include <stdlib.h>
#include "BMI088driver.h"
#include "BMI088reg.h"
#include "BMI088Middleware.h"
#include "judge_and_debug.h"
#include "dbus_receive.h"
#include "can_Peripheral.h"
#include "buzzer_Peripheral.h"
#include "Can_receive.h"
#include "pid.h"
#include "Squarewave.h"
#include "filter.h"
#include "djl_driver.h"
#include "ChassisTask.h"
#include "Gimbal6020.h"
#include "ImuTask.h"
#include "CanTask.h"
#include "ShootTask.h"
#include "Tim1Task.h"
#include "RC_ProtectionTask.h"
#include "SCAP.h"
#include "mahony.h"
#include "Powercontrol.h"
#include "lazer.h"
#include "Sliding_Mode.h"
#include "Gimbal_FindingMode.h"
#include "HeatControlTask.h"
#include "matrix.h"
#include "refereetask.h"
#include "referee.h"
#include "fifo.h"
#include "CRC8_CRC16.h"
#include "VisionSendTask.h"
#include "dm_driver.h"
#include "dwt.h"

void System_Init(void);

#endif
