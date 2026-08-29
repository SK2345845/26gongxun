/**
 * @file    hmi_menu.c
 * @brief   串口屏菜单逻辑实现（26gongxun 版）
 *
 * 事件流：HMI_ProcessRx 解析触摸 → 本模块分发（按下/松开）→ 调驱动
 * 显示流：本模块周期调 HMI_SetText/HMI_SetTextBytes 刷新当前页
 *
 * 控件 ID 即屏端创建顺序：页面里按钮/文本按 docs 对照表从 0 递增摆放。
 * 方向键为「按住走、松开停」：OnPress 发方向命令，OnRelease 一律急停。
 *
 * 带宽预算：9600 波特率下一条 t0.txt 指令约 10~25ms，任务页 3 条文本
 * 200ms 刷新一次（约 40~60ms 发送时间）；遥控页无周期刷新、纯事件驱动，
 * 不会与底盘控制抢节拍。嫌慢可屏端+board.c 同步改 115200。
 */

#include "hmi_menu.h"
#include "hmi_tjc.h"
#include "task_code.h"
#include "scanner.h"
#include "moving.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

/* 刷新周期：文本类 200ms */
#define MENU_REFRESH_TICKS   pdMS_TO_TICKS(200)

/*-----------------------------------------------------------
 * GBK 字节表（屏端默认 GBK 编码，源码是 UTF-8，不能直接发中文串）
 *----------------------------------------------------------*/
static const char GBK_RED[]    = "\xBA\xEC";            /* 红 */
static const char GBK_YELLOW[] = "\xBB\xC6";            /* 黄 */
static const char GBK_BLUE[]   = "\xC0\xB6";            /* 蓝 */
static const char GBK_GREEN[]  = "\xC2\xCC";            /* 绿 */
static const char GBK_BLACK[]  = "\xBA\xDA";            /* 黑 */
static const char GBK_LBLUE[]  = "\xC7\xB3\xC0\xB6";    /* 浅蓝 */
static const char GBK_RING[]   = "\xBB\xB7";            /* 环 */
static const char GBK_BAD[]    = "\xCE\xDE\xD0\xA7";    /* 无效 */
static const char GBK_DASH[]   = "--";                  /* 占位 */

/* 颜色编号(1~6) → GBK 字节串，非法返回 0 */
static const char *ColorGBK(uint8_t color, uint16_t *len)
{
        switch (color) {
                case TASK_COLOR_RED:    *len = 2; return GBK_RED;
                case TASK_COLOR_YELLOW: *len = 2; return GBK_YELLOW;
                case TASK_COLOR_BLUE:   *len = 2; return GBK_BLUE;
                case TASK_COLOR_GREEN:  *len = 2; return GBK_GREEN;
                case TASK_COLOR_BLACK:  *len = 2; return GBK_BLACK;
                case TASK_COLOR_LBLUE:  *len = 4; return GBK_LBLUE;
                default: *len = 0; return 0;
        }
}

/* 拼 "颜色>环N "，buf 至少 12 字节，返回新长度 */
static uint16_t AppendColorRing(char *buf, uint16_t n, uint8_t color, uint8_t ring)
{
        const char *cg;
        uint16_t cl = 0;

        cg = ColorGBK(color, &cl);
        if (cg == 0) { return n; }
        memcpy(&buf[n], cg, cl);  n = (uint16_t)(n + cl);
        buf[n++] = '>';
        memcpy(&buf[n], GBK_RING, 2);  n = (uint16_t)(n + 2);
        buf[n++] = (char)('0' + ring);
        return n;
}

/*-----------------------------------------------------------
 * 模块状态
 *----------------------------------------------------------*/
static uint8_t  s_page = HMI_PAGE_MAIN;
static uint32_t s_last_refresh = 0;

/* 扫码页显示状态（由 HMI_Menu_NotifyCode 更新） */
static char     s_last_raw[TASK_RAW_MAX] = "";
static uint8_t  s_scan_state = HMI_SCAN_IDLE;
static uint32_t s_scan_cnt_ok  = 0;
static uint32_t s_scan_cnt_bad = 0;

/*-----------------------------------------------------------
 * 任务页刷新：t0 原始码，t1 批1明细，t2 批2明细
 *----------------------------------------------------------*/
static void RefreshTaskPage(void)
{
        const TaskPlan_t *plan = TaskCode_Current();
        char line[48];
        uint16_t n;
        uint8_t  i;

        if (!plan->valid) {
                HMI_SetText("t0", "NO TASK");
                HMI_SetTextBytes("t1", GBK_DASH, 2);
                HMI_SetTextBytes("t2", GBK_DASH, 2);
                return;
        }

        HMI_SetText("t0", plan->raw);

        /* 批 1 明细 */
        n = 0;
        for (i = 0; i < TASK_BATCH_N; i++) {
                if (i > 0) { line[n++] = ' '; }
                n = AppendColorRing(line, n, plan->b1_color[i], plan->b1_ring[i]);
        }
        HMI_SetTextBytes("t1", line, n);

        /* 批 2 明细 */
        n = 0;
        for (i = 0; i < TASK_BATCH_N; i++) {
                if (i > 0) { line[n++] = ' '; }
                n = AppendColorRing(line, n, plan->b2_color[i], plan->b2_ring[i]);
        }
        HMI_SetTextBytes("t2", line, n);
}

/*-----------------------------------------------------------
 * 扫码页刷新：t0 最后原始码，t1 结果，t2 统计
 *----------------------------------------------------------*/
static void RefreshScanPage(void)
{
        char text[64];
        Scanner_Stats_t st;

        HMI_SetText("t0", (s_last_raw[0] != '\0') ? s_last_raw : "--");

        switch (s_scan_state) {
                case HMI_SCAN_OK:  HMI_SetText("t1", "OK");   break;
                case HMI_SCAN_BAD: HMI_SetTextBytes("t1", GBK_BAD, 4); break;
                default:           HMI_SetTextBytes("t1", GBK_DASH, 2); break;
        }

        Scanner_GetStats(&st);
        /* 全 ASCII，直接 snprintf 风格手工拼 */
        {
                static const char hex_tab[] = "0123456789";
                char *p = text;
                const char *pref = "L=";
                while (*pref) { *p++ = *pref++; }
                /* rx_lines 十进制（无 stdio，手写不超过 10 位） */
                {
                        char tmp[10];
                        uint32_t v = st.rx_lines;
                        uint8_t k = 0;
                        if (v == 0) { *p++ = '0'; }
                        while (v > 0) { tmp[k++] = hex_tab[v % 10U]; v /= 10U; }
                        while (k > 0) { *p++ = tmp[--k]; }
                }
                *p++ = ' ';
                pref = "OK="; while (*pref) { *p++ = *pref++; }
                {
                        char tmp[10];
                        uint32_t v = s_scan_cnt_ok;
                        uint8_t k = 0;
                        if (v == 0) { *p++ = '0'; }
                        while (v > 0) { tmp[k++] = hex_tab[v % 10U]; v /= 10U; }
                        while (k > 0) { *p++ = tmp[--k]; }
                }
                *p++ = ' ';
                pref = "BAD="; while (*pref) { *p++ = *pref++; }
                {
                        char tmp[10];
                        uint32_t v = s_scan_cnt_bad;
                        uint8_t k = 0;
                        if (v == 0) { *p++ = '0'; }
                        while (v > 0) { tmp[k++] = hex_tab[v % 10U]; v /= 10U; }
                        while (k > 0) { *p++ = tmp[--k]; }
                }
                *p = '\0';
        }
        HMI_SetText("t2", text);
}

/*-----------------------------------------------------------
 * 事件分发
 *----------------------------------------------------------*/
static void DispatchEvent(const HMI_Event_t *ev)
{
        if (ev->event == 1) {
                /* ---- 按下 ---- */
                switch (ev->page) {
                        case HMI_PAGE_MAIN:
                                if      (ev->component == 0) { HMI_GotoPage(HMI_PAGE_TASK);   s_page = HMI_PAGE_TASK; }
                                else if (ev->component == 1) { HMI_GotoPage(HMI_PAGE_SCAN);   s_page = HMI_PAGE_SCAN; }
                                else if (ev->component == 2) { HMI_GotoPage(HMI_PAGE_REMOTE); s_page = HMI_PAGE_REMOTE; }
                                break;

                        case HMI_PAGE_TASK:
                                if (ev->component == 0) { Scanner_Trigger(); }   /* 重新扫码 */
                                break;

                        case HMI_PAGE_SCAN:
                                if (ev->component == 0) { Scanner_Trigger(); }   /* 触发扫码 */
                                break;

                        case HMI_PAGE_REMOTE:
                                switch (ev->component) {
                                        case 0: Chassis_Remote_Command('W'); break;   /* 前 */
                                        case 1: Chassis_Remote_Command('S'); break;   /* 后 */
                                        case 2: Chassis_Remote_Command('A'); break;   /* 左 */
                                        case 3: Chassis_Remote_Command('D'); break;   /* 右 */
                                        case 6: Chassis_Remote_SpeedAdjust(1);  break;/* 速度+ */
                                        case 7: Chassis_Remote_SpeedAdjust(-1); break;/* 速度- */
                                        default: break;
                                }
                                break;

                        default: break;
                }
        }
        else {
                /* ---- 松开 ---- */
                switch (ev->page) {
                        case HMI_PAGE_TASK:
                                if (ev->component == 1) { HMI_GotoPage(HMI_PAGE_MAIN); s_page = HMI_PAGE_MAIN; }
                                break;

                        case HMI_PAGE_SCAN:
                                if (ev->component == 1) { HMI_GotoPage(HMI_PAGE_MAIN); s_page = HMI_PAGE_MAIN; }
                                break;

                        case HMI_PAGE_REMOTE:
                                if (ev->component <= 3) {
                                        Chassis_Remote_Stop();   /* 方向键松开即停 */
                                } else if (ev->component == 5) {
                                        Chassis_Remote_Stop();
                                        HMI_GotoPage(HMI_PAGE_MAIN);
                                        s_page = HMI_PAGE_MAIN;
                                }
                                break;

                        default: break;
                }
        }
}

/*-----------------------------------------------------------
 * 对外接口
 *----------------------------------------------------------*/
void HMI_Menu_Init(void)
{
        HMI_Init();
        s_page = HMI_PAGE_MAIN;
        HMI_GotoPage(HMI_PAGE_MAIN);
        s_last_refresh = xTaskGetTickCount();
}

void HMI_Menu_Process(void)
{
        HMI_Event_t ev;
        uint32_t now = xTaskGetTickCount();

        while (HMI_GetEvent(&ev)) {
                if (ev.type == HMI_EVT_TOUCH) {
                        DispatchEvent(&ev);
                }
        }

        if ((now - s_last_refresh) >= MENU_REFRESH_TICKS) {
                s_last_refresh = now;
                switch (s_page) {
                        case HMI_PAGE_TASK: RefreshTaskPage(); break;
                        case HMI_PAGE_SCAN: RefreshScanPage(); break;
                        default: break;   /* 主菜单/遥控页无需周期刷新 */
                }
        }
}

void HMI_Menu_NotifyCode(const char *raw, uint8_t parse_ok)
{
        if (raw == 0) { return; }
        strncpy(s_last_raw, raw, sizeof(s_last_raw) - 1);
        s_last_raw[sizeof(s_last_raw) - 1] = '\0';

        if (parse_ok) { s_scan_state = HMI_SCAN_OK;  s_scan_cnt_ok++;  }
        else          { s_scan_state = HMI_SCAN_BAD; s_scan_cnt_bad++; }
}

uint8_t HMI_Menu_CurPage(void)
{
        return s_page;
}
