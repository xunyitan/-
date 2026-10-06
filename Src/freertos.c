/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * File Name          : freertos.c
  * Description        : Code for freertos applications
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2025 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */

/* Includes ------------------------------------------------------------------*/
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"
#include "Sys.h"
#include "PersonalUITask.h"
/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/
/* USER CODE BEGIN Variables */

/* USER CODE END Variables */
/* Definitions for defaultTask */
//osThreadId_t defaultTaskHandle;
//const osThreadAttr_t defaultTask_attributes = {
//  .name = "defaultTask",
//  .stack_size = 128 * 4,
//  .priority = (osPriority_t) osPriorityNormal,
//};
//任务句柄:
osThreadId_t Can1Task_Handle;
osThreadId_t Can2Task_Handle;
osThreadId_t ChassisTask_Handle;
osThreadId_t GimbalTask_Handle;
osThreadId_t ShootTask_Handle;
osThreadId_t ImuTask_Handle;
osThreadId_t ImuTempTask_Handle;
//osThreadId_t Tim1Task_Handle;
osThreadId_t RCProtect_Handle;
//osThreadId_t PowerTask_Handle;
osThreadId_t RefereeTask_Handle;
osThreadId_t DMETask_Handle;
osThreadId_t VisionTask_Handle;
osThreadId_t JudgeUITask_Handle;
osThreadId_t BuzzerTask_Handle;
// 任务属性定义:
const osThreadAttr_t Can1Task_attributes = {
  .name = "Can1Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

const osThreadAttr_t Can2Task_attributes = {
  .name = "Can2Task",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

const osThreadAttr_t ChassisTask_attributes = {
  .name = "ChassisTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

const osThreadAttr_t GimbalTask_attributes = {
  .name = "GimbalTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

const osThreadAttr_t ShootTask_attributes = {
  .name = "ShootTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

const osThreadAttr_t ImuTask_attributes = {
  .name = "ImuTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

const osThreadAttr_t ImuTempTask_attributes={
  .name = "ImuTempTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

//const osThreadAttr_t Tim1Task_attributes = {
//  .name = "Tim1Task",
//  .stack_size = 256 * 4,
//  .priority = (osPriority_t) osPriorityBelowNormal,
//};

const osThreadAttr_t DMETask_attributes = {
  .name = "DMETask",
  .stack_size = 100 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};

const osThreadAttr_t UITask_attributes = {
  .name = "UITask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityBelowNormal,
};

const osThreadAttr_t RCProtect_attributes = {
  .name = "RCProtect",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

//const osThreadAttr_t PowerTask_attributes = {
//  .name = "Powertask",
//  .stack_size = 1024 * 4,
//  .priority = (osPriority_t) osPriorityNormal,
//};

const osThreadAttr_t RefereeTask_attributes={
	.name="refereeTask",
	.stack_size = 256 * 4,
	.priority = (osPriority_t) osPriorityNormal,
};

const osThreadAttr_t VisionTask_attributes = {
  .name = "VisionTask",
  .stack_size = 256 * 4,
  .priority = (osPriority_t) osPriorityNormal,
};

const osThreadAttr_t BuzzerTask_attributes = {
  .name = "BuzzerTask",
  .stack_size = 100 * 4,
  .priority = (osPriority_t) osPriorityAboveNormal,//osPriorityAboveNormal
};
/* Private function prototypes -----------------------------------------------*/
/* USER CODE BEGIN FunctionPrototypes */

/* USER CODE END FunctionPrototypes */

void StartDefaultTask(void *argument);

void buzzer_task(void *argument);

void MX_FREERTOS_Init(void); /* (MISRA C 2004 rule 8.1) */
/**
  * @brief  FreeRTOS initialization
  * @param  None
  * @retval None
  */
void MX_FREERTOS_Init(void) {
  /* USER CODE BEGIN Init */
	taskENTER_CRITICAL();
	//创建can1任务:
	Can1Task_Handle=osThreadNew(CAN1_Task,NULL,&Can1Task_attributes);
	//创建can2任务:
	Can2Task_Handle=osThreadNew(CAN2_Task,NULL,&Can2Task_attributes);
	//创建底盘任务:
	ChassisTask_Handle=osThreadNew(ChassisTask_Test,NULL,&ChassisTask_attributes);
	//创建云台任务:
	GimbalTask_Handle=osThreadNew(GimbalTask_Test,NULL,&GimbalTask_attributes);
	//创建达妙电机使能任务:
	DMETask_Handle=osThreadNew(Gimbal4310_EnableTask,NULL,&DMETask_attributes);
	//创建射击任务:
	ShootTask_Handle=osThreadNew(Shooting_Task,NULL,&ShootTask_attributes);
	//创建遥控器保护任务:
	RCProtect_Handle=osThreadNew(RC_ProtectionTask,NULL,&RCProtect_attributes);
	//创建云台imu欧拉角解算任务:
	ImuTask_Handle=osThreadNew(IMUAngle_Task,NULL,&ImuTask_attributes);
	//创建云台imu温控任务:
	ImuTempTask_Handle=osThreadNew(IMUTemp_ControlTask,NULL,&ImuTempTask_attributes);
	//创建视觉发送四元数任务:
  VisionTask_Handle=osThreadNew(VisionSendTask,NULL,&VisionTask_attributes);
	//创建裁判系统UI任务:
	JudgeUITask_Handle=osThreadNew(Draw_UITask,NULL,&UITask_attributes);  
//	//创建功率控制任务:
//  PowerTask_Handle=osThreadNew(powerdaemon_task,NULL,&PowerTask_attributes);
	//创建裁判系统数据读取任务:
	RefereeTask_Handle=osThreadNew(referee_fifoTask,NULL,&RefereeTask_attributes);
	//创建提示任务:
	BuzzerTask_Handle=osThreadNew(buzzer_task,NULL,&BuzzerTask_attributes);
	taskEXIT_CRITICAL();
	//由于这个是MX_FREERTOS_Init（），相当于一个c函数，并不属于任务，属于裸机环境，本质上是一个初始化函数，并且​​不占用任何任务资源​​（无TCB，无任务栈），所以不必删除自身
  /* USER CODE END Init */
  /* USER CODE BEGIN RTOS_MUTEX */
  /* add mutexes, ... */
  /* USER CODE END RTOS_MUTEX */

  /* USER CODE BEGIN RTOS_SEMAPHORES */
  /* add semaphores, ... */
  /* USER CODE END RTOS_SEMAPHORES */

  /* USER CODE BEGIN RTOS_TIMERS */
  /* start timers, add new ones, ... */
  /* USER CODE END RTOS_TIMERS */

  /* USER CODE BEGIN RTOS_QUEUES */
  /* add queues, ... */
  /* USER CODE END RTOS_QUEUES */

  /* Create the thread(s) */
  /* creation of defaultTask */
//  defaultTaskHandle = osThreadNew(StartDefaultTask, NULL, &defaultTask_attributes);

  /* USER CODE BEGIN RTOS_THREADS */
  /* add threads, ... */
  /* USER CODE END RTOS_THREADS */

  /* USER CODE BEGIN RTOS_EVENTS */
  /* add events, ... */
  /* USER CODE END RTOS_EVENTS */

}

/* USER CODE BEGIN Header_StartDefaultTask */
/**
  * @brief  Function implementing the defaultTask thread.
  * @param  argument: Not used
  * @retval None
  */
/* USER CODE END Header_StartDefaultTask */
//void StartDefaultTask(void *argument)
//{
//  /* USER CODE BEGIN StartDefaultTask */
//  /* Infinite loop */
//  for(;;)
//  {
//    osDelay(1);
//  }
//  /* USER CODE END StartDefaultTask */
//}


void buzzer_task(void *argument)
{
	__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3,Buzzer_loudest);
	osDelay(200);
	__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3,Buzzer_min);
	osDelay(200);
	__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3,Buzzer_middle);
	osDelay(200);
	__HAL_TIM_SetCompare(&htim4,TIM_CHANNEL_3,Buzzer_OFF);
	BuzzerTask_Handle=NULL;
	vTaskDelete(NULL);
}
