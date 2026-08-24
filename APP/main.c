#include "board.h"
#include "delay.h"
#include "usart.h"
#include "Emm_V5.h"
#include "moving.h"

/**********************************************************
***	Emm_V5.0 步进闭环控制工程 (FreeRTOS)
***	main.c 仅负责硬件初始化与主干顺序控制逻辑
**********************************************************/

/**
	* @brief   用户主干顺序控制逻辑
	* @note    由 moving.c 创建的 FreeRTOS 任务调度执行，使用 vTaskDelay 实现非阻塞延时
	* @param   无
	* @retval  无
	*/
void User_Sequential_Logic(void)
{
	// 1. 上电稳定等待 500ms（非阻塞释放 CPU）
	vTaskDelay(pdMS_TO_TICKS(500));

	// 2. 顺序执行流程：调用底盘前进函数，传入目标脉冲数（如 32000 即 10 圈）
	zhuan_ni(32000);

	// 3. 执行完毕后进入休眠
	while(1)
	{
		vTaskDelay(pdMS_TO_TICKS(1000));
	}
}

/**
	* @brief   MAIN 函数
	* @param   无
	* @retval  无
	*/
int main(void)
{
	// 1. 初始化板载底层外设（时钟、NVIC、串口等）
	board_init();

	// 2. 创建 FreeRTOS 任务（由 Hardware/moving.c 管理）
	Moving_Task_Create();

	// 3. 启动 FreeRTOS 内核调度器
	vTaskStartScheduler();

	// 正常情况下不会执行到此处
	while(1)
	{
	}
}