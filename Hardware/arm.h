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

// 5号电机单圈回零参数（写入驱动器Flash，只需配置一次）
#define ARM_BASE_ORIGIN_MODE       0       // 回零模式：0=单圈就近回零
#define ARM_BASE_ORIGIN_DIR        0       // 回零方向：0=CW
#define ARM_BASE_ORIGIN_VEL        30      // 回零速度（RPM），慢一点找原点更稳
#define ARM_BASE_ORIGIN_TIMEOUT_MS 10000   // 驱动器侧回零超时（毫秒）
#define ARM_BASE_ORIGIN_WAIT_MS    12000   // 软件轮询等待上限（毫秒），须大于驱动器超时
#define ARM_BASE_ORIGIN_SAVE       1       // 1=回零参数存Flash（只需一次）；调试期可改0不存
#define ARM_BASE_HOME_TOL_PULSE 	2

/**********************************************************
*** 联合机械臂调试流程参数（输入 ARMSEQ 启动）
*** 同一阶段内的动作会同时启动，阶段之间按顺序执行。
**********************************************************/
#define ARM_SEQ_INITIAL_RAISE_TURNS       0       // 开始升到最高：默认已在最高位
#define ARM_SEQ_BASE_PICK_DEGREE          180     // 取物时5号目标角度
#define ARM_SEQ_EXTEND_TURNS              2       // 6号伸出圈数
#define ARM_SEQ_LOWER_PICK_TURNS          50      // 取物下降圈数
#define ARM_SEQ_RAISE_AFTER_PICK_TURNS    50      // 抓取后上升圈数
#define ARM_SEQ_BASE_HOME_DEGREE          0       // 返回时5号目标角度
#define ARM_SEQ_RETRACT_TURNS             2       // 6号回收圈数
#define ARM_SEQ_LOWER_RELEASE_TURNS       30      // 松开前下降圈数
#define ARM_SEQ_LIFT_UP_SIGN              1       // 7号上升方向：1=正转(CW)，-1=反转(CCW)
#define ARM_SEQ_MOTION_MARGIN_MS          300     // 每个动作完成后的安全余量
#define ARM_SEQ_GRIPPER_WAIT_MS           500     // 夹取/松开后的等待时间（舵机接入后生效）
/**********************************************************
*** 机械臂控制接口
**********************************************************/
// 5号电机：单圈回零，回零完成后当前位置自动作为零点
// 返回1=回零成功，0=失败/超时（结果同时通过USART1打印）
uint8_t Arm_Base_Origin_Return(void);

// 一次性标定：把5号电机当前位置设为回零零点并存入Flash（串口输入"5o"触发）
void Arm_Base_Set_Origin_Here(void);

// 联合机械臂分阶段并行调试流程（串口输入"ARMSEQ"触发）
void Arm_Combined_Sequence_Start(void);
void Arm_Gripper_Grab(void);       // 舵机暂缺：当前为空操作
void Arm_Gripper_Release(void);    // 舵机暂缺：当前为空操作

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
