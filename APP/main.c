#include "board.h"
#include "delay.h"
#include "usart.h"
#include "Emm_V5.h"
#include "moving.h"
#include "arm.h"
#include "scanner.h"
#include "rplidar.h"
#include "lidar_link.h"

/**********************************************************
***     Emm_V5.0 步进闭环控制工程 (FreeRTOS)
***     main.c 仅负责硬件初始化与主干顺序控制逻辑
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

        // 2. 【底盘调试模式】仅运行底盘遥控任务，其它任务暂时停用
        usart1_SendString("[REMOTE] chassis-only debug mode\r\n");
        usart1_SendString("[REMOTE] WASD move, E/R rotate, +/- speed, X stop\r\n");
        usart1_SendString("[REMOTE] press 'p' to probe/spin drivers 1-4\r\n");
        usart1_SendString("[REMOTE] waiting 2.5s for drivers ready...\r\n");

        // 3. 【关键】等待张大头驱动器完成自身初始化。
        //    驱动器冷启动需要 1~2 秒，期间收到的指令（含使能）会被直接吞掉。
        //    之前"断电重启后小车失灵"的头号嫌疑就是 STM32 在 500ms 就使能，
        //    而驱动器还没就绪。此处等足 2.5 秒，并补发一次使能兜底。
        vTaskDelay(pdMS_TO_TICKS(2000));
        Chassis_Remote_Enable();
        vTaskDelay(pdMS_TO_TICKS(500));
        Chassis_Remote_Enable();

        // 4. 关闭USART1接收中断（该硬件上RXNE中断不触发），改用轮询读取
        Chassis_Remote_RxProbe_Begin();

        // 5. 主循环：轮询读取USART1并驱动底盘。收到 w 会回显 [REMOTE] cmd=w，
        //    并向USART3发送速度指令。若回显有、车不动 → 问题在电机侧(USART3/驱动器)。
        while(1)
        {
                Chassis_Remote_Process();
                vTaskDelay(pdMS_TO_TICKS(2));
        }

        /* ===== 以下为机械臂/扫码/雷达任务，底盘调试期间暂时停用，调好后可恢复 =====
        Scanner_Init();
        Lidar_Init();
        uint32_t sent_lidar_rev = 0;
        while(1)
        {
                Arm_Lift_Debug_Process();
                Scanner_Process();
                Lidar_Process();
                {
                        LidarPoint_t *points;
                        uint16_t count = Lidar_GetScan(&points);
                        if(count > 0 && Lidar_GetRevCount() != sent_lidar_rev)
                        {
                                sent_lidar_rev = Lidar_GetRevCount();
                                LidarLink_SendScan(points, count, sent_lidar_rev);
                        }
                }
                vTaskDelay(pdMS_TO_TICKS(20));
        }
        ========================================================================= */
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
