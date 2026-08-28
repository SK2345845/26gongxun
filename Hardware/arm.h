#ifndef __ARM_H
#define __ARM_H

#include "stm32f4xx.h"
#include "FreeRTOS.h"
#include "task.h"
#include "Emm_V5.h"

/**********************************************************
*** 机械臂步进电机配置
**********************************************************/
#define ARM_BASE_MOTOR             5       // 5号电机地址：底部转盘
#define ARM_EXTEND_MOTOR           6       // 6号电机地址：四杆机构行程
#define ARM_LIFT_MOTOR             7       // 7号电机地址：丝杆升降机构
#define ARM_BASE_ONE_TURN_PULSE    3200   // 5号电机转一圈对应的脉冲数（16细分）
#define ARM_EXTEND_ONE_TURN_PULSE  3200   // 6号电机轴转一圈对应的脉冲数（16细分）
#define ARM_LIFT_ONE_TURN_PULSE    3200   // 7号电机轴转一圈对应的脉冲数（16细分）
#define ARM_DEGREE_SCALE           1000   // 角度定点精度：0.001度
#define ARM_BASE_DEGREE_PULSE      3200  // 5号电机一圈脉冲数，用于角度换算

#define ARM_DIR_CW                 0       // 电机正向：CW
#define ARM_DIR_CCW                1       // 电机反向：CCW

#define ARM_BASE_MOVE_VEL          60      // 5号电机运动速度（RPM）
#define ARM_BASE_MOVE_ACC          30      // 5号电机加速度，范围0~255
#define ARM_BASE_MAX_DEGREES       360     // 5号电机单次最大角度
#define ARM_BASE_TURN_TIME_MS      (60000 / ARM_BASE_MOVE_VEL + 300) // 5号每圈预计时间（毫秒）

#define ARM_EXTEND_MOVE_VEL        4000    // 6号电机运动速度（RPM）
#define ARM_EXTEND_MOVE_ACC        250     // 6号电机加速度，范围0~255
#define ARM_EXTEND_MAX_TURNS       100     // 6号电机单次最大运行圈数
#define ARM_TURN_SCALE             1000    // 圈数定点精度：0.001圈
#define ARM_EXTEND_TURN_TIME_MS    (60000 / ARM_EXTEND_MOVE_VEL + 300) // 每圈预计时间（毫秒）

#define ARM_LIFT_MOVE_VEL          4000    // 7号电机运动速度（RPM）
#define ARM_LIFT_MOVE_ACC          250     // 7号电机加速度，范围0~255
#define ARM_LIFT_MAX_TURNS         100     // 7号电机单次最大运行圈数
#define ARM_LIFT_TURN_TIME_MS      (60000 / ARM_LIFT_MOVE_VEL + 300) // 每圈预计时间（毫秒）

#define ARM_BASE_PID_KP            16000   // 5号电机闭环比例系数，不写入Flash
#define ARM_BASE_PID_KI            0       // 5号电机闭环积分系数，不写入Flash
#define ARM_BASE_PID_KD            30000   // 5号电机闭环微分系数，不写入Flash
/**********************************************************
*** 机械臂控制接口
**********************************************************/
// 5号电机：单圈回零，回零完成后当前位置自动作为零点
void Arm_Base_Origin_Return(void);

// 7号电机：丝杆机构相对位置运行一圈
void Arm_Lift_One_Turn_Test(void);
// 通过USART1读取圈数命令：支持小数，正数正转，负数反转，回车执行
void Arm_Lift_Debug_Process(void);
extern __IO int32_t Arm_Lift_Total_Turns;  // 实际单位为0.001圈
extern __IO int32_t Arm_Screw_Total_Turns; // 7号实际单位为0.001圈
extern __IO int32_t Arm_Base_Total_Degrees; // 5号实际单位为0.001度

// 配置5号电机内部闭环PID参数
void Arm_Base_PID_Configure(void);

// 5号电机：正向一圈后反向一圈
void Arm_Base_Forward_Reverse_Test(void);

// 执行5号电机回零硬件测试，默认不自动调用
void Arm_Test_Run(void);

#endif /* __ARM_H */
