#ifndef __MOVING_H
#define __MOVING_H

#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Emm_V5.h"

/**********************************************************
***	FreeRTOS 任务配置
**********************************************************/
#define MOVING_TASK_PRIO        2
#define MOVING_TASK_STK_SIZE    256

extern TaskHandle_t MovingTask_Handler;

/**********************************************************
***	任务创建接口声明
**********************************************************/
void Moving_Task_Create(void);

/**********************************************************
***	底盘运动控制接口声明
**********************************************************/
// 底盘前进（4电机多机同步位置运动）
void move_qian(uint32_t pulse);

#endif /* __MOVING_H */