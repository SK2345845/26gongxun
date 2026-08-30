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
#define REMOTE_MOVE_VEL         100
#define REMOTE_MOVE_ACC         100
#define REMOTE_MOVE_VEL_MIN     20
#define REMOTE_MOVE_VEL_MAX     1000
#define REMOTE_MOVE_VEL_STEP    20
/* 位置模式（定距离移动）默认速度/加速度，跑图调参改这里 */
#define REMOTE_POS_VEL          100
#define REMOTE_POS_ACC          100

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

// 位置模式定距移动：dir=w/a/s/d/e/r/q/z/c/v，pulse=脉冲数（16细分下 3200=1圈）
void Chassis_Remote_Move(char dir, uint32_t pulse);

// USART1遥控：w/a/s/d前后左右，e/r顺逆时针，x停止
void Chassis_Remote_Command(char command);
void Chassis_Remote_Stop(void);
void Chassis_Remote_SpeedAdjust(int8_t direction);
// 使能底盘4个电机（速度模式运行前必须先使能，否则驱动器不响应）
void Chassis_Remote_Enable(void);
// 读取USART1并分发底盘遥控指令（底盘专用，独立于机械臂调试）
void Chassis_Remote_Process(void);
// 诊断：关闭USART1接收中断，改为轮询寄存器（排除中断/接线问题）
void Chassis_Remote_RxProbe_Begin(void);
void Chassis_Remote_RxProbe(void);
// 诊断：逐个测试1~4号驱动器（使能→转0.4秒→停止），打印回包
void Chassis_Motor_Probe(void);

#endif /* __MOVING_H */
