#include "board.h"
#include "delay.h"
#include "usart.h"
#include "Emm_V5.h"
#include "moving.h"
#include "arm.h"
#include "scanner.h"
#include "task_code.h"
#include "hmi_menu.h"
#include "rplidar.h"
#include "lidar_link.h"
#include <string.h>

/**********************************************************
***     Emm_V5.0 步进闭环控制工程 (FreeRTOS)
***     main.c 仅负责硬件初始化与主干顺序控制逻辑
**********************************************************/

/**
        * @brief   扫码结果 → 任务码解析入库（含重复码去重）
        * @note    连续/感应模式下模块可能连读同一个二维码，
        *          同一内容 2 秒内只入库一次；非法码不覆盖已有任务
        */
static void Scanner_TaskCode_Poll(void)
{
        static Scanner_Code_t code;
        static char last_raw[TASK_RAW_MAX] = "";
        static uint32_t last_tick = 0;
        char brief[96];

        while (Scanner_GetCode(&code))
        {
                /* 调试：扫到内容显示到当前页 t1（屏须停在 page5；跨页写需 vscope=全局，故不用） */
                HMI_SetText("t1", code.code);

                /* 同步在 debug 串口(USART1)打印扫到的原始串 */
                usart1_SendString("[SCAN] ");
                usart1_SendString(code.code);
                usart1_SendString("\r\n");

                if (code.len == (TASK_RAW_MAX - 1) &&
                    strcmp(code.code, last_raw) == 0 &&
                    (code.tick - last_tick) < pdMS_TO_TICKS(2000))
                {
                        continue;   /* 2 秒内重复扫到同一个码，跳过 */
                }

                if (TaskCode_Accept(code.code))
                {
                        strcpy(last_raw, code.code);
                        last_tick = code.tick;
                        HMI_Menu_NotifyCode(code.code, 1);
                        usart1_SendString("[TASK] task code accepted: ");
                        usart1_SendString(code.code);
                        usart1_SendString("\r\n");
                        TaskCode_FormatBrief(TaskCode_Current(), brief, sizeof(brief));
                        usart1_SendString("[TASK] ");
                        usart1_SendString(brief);
                        usart1_SendString("\r\n");
                }
                else
                {
                        HMI_Menu_NotifyCode(code.code, 0);
                        usart1_SendString("[TASK] invalid code (keep old): ");
                        usart1_SendString(code.code);
                        usart1_SendString("\r\n");
                }
        }
}

/**
        * @brief   用户主干顺序控制逻辑
        * @note    由 moving.c 创建的 FreeRTOS 任务调度执行，使用 vTaskDelay 实现非阻塞延时
        * @param   无
        * @retval  无
        */
void User_Sequential_Logic(void)
{
        uint32_t last_trigger = 0;

        // 1. 上电稳定等待 500ms（非阻塞释放 CPU）
        vTaskDelay(pdMS_TO_TICKS(500));

        // 2. 【扫码+串口屏调试模式】只跑扫码与串口屏，底盘/机械臂/雷达等其它任务暂时停用
        usart1_SendString("[SCAN] scanner + HMI debug mode\r\n");

        Scanner_Init();
        TaskCode_Init();

        // 3. 等串口屏上电启动完成再发 HMI 指令：TJC 屏上电约需 1~2s，期间命令会被丢弃
        vTaskDelay(pdMS_TO_TICKS(2000));

        // 4. 显式设置扫描模式（参考 Gongchuang_27_Experiment/app_tasks.c，原来漏了这步）。
        //    TRIGGER(0x99)=命令触发、扫一次算一次；CONTINUOUS(0x22)=一直扫。
        //    调试期用 TRIGGER，配合下面每 2s 自动触发，确认命令链路与模块都正常。
        Scanner_SetModeRuntime(SCANNER_MODE_TRIGGER);

        HMI_Menu_Init();

        while(1)
        {
                // 调试期：每 2s 自动触发一次扫码（脱离人手；改用 CONTINUOUS 时删掉这段）
                if ((xTaskGetTickCount() - last_trigger) >= pdMS_TO_TICKS(2000))
                {
                        last_trigger = xTaskGetTickCount();
                        Scanner_Trigger();
                }

                Scanner_Process();          /* 解析 GM65 上报字节 → 码队列 */
                Scanner_TaskCode_Poll();    /* 出队：屏 page5.t1 + 解析入库 + debug 打印 */
                HMI_Menu_Process();         /* 屏触摸事件分发 + 当前页周期刷新 */
                vTaskDelay(pdMS_TO_TICKS(20));
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
