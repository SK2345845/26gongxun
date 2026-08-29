/**
 * @file    hmi_menu.h
 * @brief   串口屏菜单逻辑（26gongxun 版）：页面管理、触摸分发、任务/扫码显示
 *
 * 页面与控件 ID 对照见 docs/HMI_屏端对照表_26gongxun.md，
 * 屏端（HMI 编辑器）按该表摆放控件并保证创建顺序，ID 自动对齐。
 *
 * 页面规划（创建顺序即页 ID）：
 *   0 MAIN    主菜单（进入各功能页）
 *   1 TASK    任务显示（任务码 + 两批明细，接 APP/task_code）
 *   2 SCAN    扫码调试（触发扫码、显示原始码与解析结果、统计）
 *   3 REMOTE  底盘遥控（方向键 + 速度档，接 Hardware/moving）
 *
 * 中文编码：屏端 GBK，动态中文用 GBK 字节发送（见 hmi_menu.c 颜色表）。
 */

#ifndef __HMI_MENU_H
#define __HMI_MENU_H

#include <stdint.h>

/* 页 ID（屏端页面创建顺序必须一致） */
typedef enum
{
        HMI_PAGE_MAIN   = 0,
        HMI_PAGE_TASK   = 1,
        HMI_PAGE_SCAN   = 2,
        HMI_PAGE_REMOTE = 3,
        HMI_PAGE_COUNT
} HMI_Page_t;

/* 扫码页结果状态 */
typedef enum
{
        HMI_SCAN_IDLE = 0,      /* 未扫到 */
        HMI_SCAN_OK,            /* 解析入库成功 */
        HMI_SCAN_BAD            /* 非法码 */
} HMI_ScanState_t;

void    HMI_Menu_Init(void);     /* HMI_Init + 进主菜单（User_Sequential_Logic 调用） */
void    HMI_Menu_Process(void);  /* 每 20ms 调：事件分发 + 当前页周期刷新 */

/* 扫码流水线回调：每次扫码出结果后通知（main.c 的 Scanner_TaskCode_Poll 调用） */
void    HMI_Menu_NotifyCode(const char *raw, uint8_t parse_ok);

/* 当前所在页（供其它模块查询） */
uint8_t HMI_Menu_CurPage(void);

#endif /* __HMI_MENU_H */
