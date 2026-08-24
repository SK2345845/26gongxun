#include "moving.h"

/**********************************************************
***	FreeRTOS 任务句柄
**********************************************************/
TaskHandle_t MovingTask_Handler = NULL;

/**********************************************************
***	外部声明 main.c 中的顺序业务逻辑函数
**********************************************************/
extern void User_Sequential_Logic(void);

/**
	* @brief   FreeRTOS 任务执行函数（运行 main.c 中的顺序控制逻辑）
	* @param   pvParameters: 任务参数
	* @retval  无
	*/
static void moving_task(void *pvParameters)
{
	// 执行 main.c 中编写的顺序逻辑
	User_Sequential_Logic();

	// 任务执行完毕自销毁
	vTaskDelete(NULL);
}

/**
	* @brief   创建 FreeRTOS 任务
	* @param   无
	* @retval  无
	*/
void Moving_Task_Create(void)
{
	xTaskCreate((TaskFunction_t )moving_task,
	            (const char*    )"moving_task",
	            (uint16_t       )MOVING_TASK_STK_SIZE,
	            (void*          )NULL,
	            (UBaseType_t    )MOVING_TASK_PRIO,
	            (TaskHandle_t*  )&MovingTask_Handler);
}

/**
	* @brief   底盘前进：4 电机多机同步位置控制
	* @param   pulse: 旋转脉冲数（16细分下 3200 脉冲 = 1 圈）
	* @retval  无
	*/
void move_qian(uint32_t pulse)
{
	// 1 号电机：位置模式，CW，1000RPM，加速度0，同步标志置1
	Emm_V5_Pos_Control(1, 0, 100, 100, pulse, 0, 1);
	vTaskDelay(pdMS_TO_TICKS(10));

	// 2 号电机：位置模式，CW，1000RPM，加速度0，同步标志置1
	Emm_V5_Pos_Control(2, 1, 100, 100, pulse, 0, 1);
	vTaskDelay(pdMS_TO_TICKS(10));

	// 3 号电机：位置模式，CW，1000RPM，加速度0，同步标志置1
	Emm_V5_Pos_Control(3, 0, 100, 100, pulse, 0, 1);
	vTaskDelay(pdMS_TO_TICKS(10));

	// 4 号电机：位置模式，CW，1000RPM，加速度0，同步标志置1
	Emm_V5_Pos_Control(4, 1, 100, 100, pulse, 0, 1);
	vTaskDelay(pdMS_TO_TICKS(10));

	// 广播地址 0 触发 4 台电机同时开始运动
	Emm_V5_Synchronous_motion(0);
	vTaskDelay(pdMS_TO_TICKS(10));
}
void move_zuo(uint32_t pulse)
{
	// 1 号电机：位置模式，CW，1000RPM，加速度0，同步标志置1
	Emm_V5_Pos_Control(1, 0, 100, 100, pulse, 0, 1);
	vTaskDelay(pdMS_TO_TICKS(10));

	// 2 号电机：位置模式，CW，1000RPM，加速度0，同步标志置1
	Emm_V5_Pos_Control(2, 0, 100, 100, pulse, 0, 1);
	vTaskDelay(pdMS_TO_TICKS(10));

	// 3 号电机：位置模式，CW，1000RPM，加速度0，同步标志置1
	Emm_V5_Pos_Control(3, 1, 100, 100, pulse, 0, 1);
	vTaskDelay(pdMS_TO_TICKS(10));

	// 4 号电机：位置模式，CW，1000RPM，加速度0，同步标志置1
	Emm_V5_Pos_Control(4, 1, 100, 100, pulse, 0, 1);
	vTaskDelay(pdMS_TO_TICKS(10));

	// 广播地址 0 触发 4 台电机同时开始运动
	Emm_V5_Synchronous_motion(0);
	vTaskDelay(pdMS_TO_TICKS(10));
}
void move_you(uint32_t pulse)
{
	// 1 号电机：位置模式，CW，1000RPM，加速度0，同步标志置1
	Emm_V5_Pos_Control(1, 1, 100, 100, pulse, 0, 1);
	vTaskDelay(pdMS_TO_TICKS(10));

	// 2 号电机：位置模式，CW，1000RPM，加速度0，同步标志置1
	Emm_V5_Pos_Control(2, 1, 100, 100, pulse, 0, 1);
	vTaskDelay(pdMS_TO_TICKS(10));

	// 3 号电机：位置模式，CW，1000RPM，加速度0，同步标志置1
	Emm_V5_Pos_Control(3, 0, 100, 100, pulse, 0, 1);
	vTaskDelay(pdMS_TO_TICKS(10));

	// 4 号电机：位置模式，CW，1000RPM，加速度0，同步标志置1
	Emm_V5_Pos_Control(4, 0, 100, 100, pulse, 0, 1);
	vTaskDelay(pdMS_TO_TICKS(10));

	// 广播地址 0 触发 4 台电机同时开始运动
	Emm_V5_Synchronous_motion(0);
	vTaskDelay(pdMS_TO_TICKS(10));
}
void move_hou(uint32_t pulse)
{
	// 1 号电机：位置模式，CW，1000RPM，加速度0，同步标志置1
	Emm_V5_Pos_Control(1, 1, 100, 100, pulse, 0, 1);
	vTaskDelay(pdMS_TO_TICKS(10));

	// 2 号电机：位置模式，CW，1000RPM，加速度0，同步标志置1
	Emm_V5_Pos_Control(2, 0, 100, 100, pulse, 0, 1);
	vTaskDelay(pdMS_TO_TICKS(10));

	// 3 号电机：位置模式，CW，1000RPM，加速度0，同步标志置1
	Emm_V5_Pos_Control(3, 1, 100, 100, pulse, 0, 1);
	vTaskDelay(pdMS_TO_TICKS(10));

	// 4 号电机：位置模式，CW，1000RPM，加速度0，同步标志置1
	Emm_V5_Pos_Control(4, 0, 100, 100, pulse, 0, 1);
	vTaskDelay(pdMS_TO_TICKS(10));

	// 广播地址 0 触发 4 台电机同时开始运动
	Emm_V5_Synchronous_motion(0);
	vTaskDelay(pdMS_TO_TICKS(10));
}
void zhuan_shun(uint32_t pulse)
{
	// 1 号电机：位置模式，CW，1000RPM，加速度0，同步标志置1
	Emm_V5_Pos_Control(1, 0, 100, 100, pulse, 0, 1);
	vTaskDelay(pdMS_TO_TICKS(10));

	// 2 号电机：位置模式，CW，1000RPM，加速度0，同步标志置1
	Emm_V5_Pos_Control(2, 0, 100, 100, pulse, 0, 1);
	vTaskDelay(pdMS_TO_TICKS(10));

	// 3 号电机：位置模式，CW，1000RPM，加速度0，同步标志置1
	Emm_V5_Pos_Control(3, 0, 100, 100, pulse, 0, 1);
	vTaskDelay(pdMS_TO_TICKS(10));

	// 4 号电机：位置模式，CW，1000RPM，加速度0，同步标志置1
	Emm_V5_Pos_Control(4, 0, 100, 100, pulse, 0, 1);
	vTaskDelay(pdMS_TO_TICKS(10));

	// 广播地址 0 触发 4 台电机同时开始运动
	Emm_V5_Synchronous_motion(0);
	vTaskDelay(pdMS_TO_TICKS(10));
}
void zhuan_ni(uint32_t pulse)
{
	// 1 号电机：位置模式，CW，1000RPM，加速度0，同步标志置1
	Emm_V5_Pos_Control(1, 1, 100, 100, pulse, 0, 1);
	vTaskDelay(pdMS_TO_TICKS(10));

	// 2 号电机：位置模式，CW，1000RPM，加速度0，同步标志置1
	Emm_V5_Pos_Control(2, 1, 100, 100, pulse, 0, 1);
	vTaskDelay(pdMS_TO_TICKS(10));

	// 3 号电机：位置模式，CW，1000RPM，加速度0，同步标志置1
	Emm_V5_Pos_Control(3, 1, 100, 100, pulse, 0, 1);
	vTaskDelay(pdMS_TO_TICKS(10));

	// 4 号电机：位置模式，CW，1000RPM，加速度0，同步标志置1
	Emm_V5_Pos_Control(4, 1, 100, 100, pulse, 0, 1);
	vTaskDelay(pdMS_TO_TICKS(10));

	// 广播地址 0 触发 4 台电机同时开始运动
	Emm_V5_Synchronous_motion(0);
	vTaskDelay(pdMS_TO_TICKS(10));
}
