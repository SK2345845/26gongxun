/**
 * @file    hmi_tjc.h
 * @brief   淘晶驰 TJC 串口屏驱动（移植自 Gongchuang_27 暑期工程）
 *
 * 串口：USART2  PA2(TX) / PA3(RX)，9600 8N1
 *
 * 两条数据流：
 *   屏→MCU（触摸）：0x65 <页ID> <控件ID> <1按下/0松开> FF FF FF → 事件队列
 *   MCU→屏（显示）：HMI_SetText/HMI_SetNum 等指令 + FF FF FF 结尾
 *
 * 中文显示注意：TJC 默认 GBK 编码，而本工程源码/调试串口是 UTF-8。
 * 动态中文（颜色名等）用 HMI_SetTextBytes 发 GBK 原始字节，
 * ASCII 内容（任务码、数字）用 HMI_SetText 即可。
 */

#ifndef __HMI_TJC_H
#define __HMI_TJC_H

#include "stm32f4xx.h"
#include <stdint.h>

/* 事件类型 */
typedef enum
{
        HMI_EVT_NONE  = 0,
        HMI_EVT_TOUCH = 1,      /* 触摸事件 */
        HMI_EVT_VALUE = 2       /* 滑块/数值上报 */
} HMI_EventType_t;

/* 事件队列长度（2 的幂取模，别改非幂） */
#define HMI_EVT_QUEUE_LEN   8

typedef struct
{
        HMI_EventType_t type;
        uint8_t page;           /* 页 ID（页创建顺序，0 起） */
        uint8_t component;      /* 控件 ID（页内创建顺序，0 起） */
        uint8_t event;          /* 触摸：1=按下 0=松开 */
        int32_t value;          /* HMI_EVT_VALUE 时的数值 */
} HMI_Event_t;

/* ---- 底层收发 ---- */
void    HMI_Init(void);                          /* 清状态机与事件队列（USART2 由 board_init 配好） */
void    HMI_SendRaw(const char *cmd);            /* 发一条指令 + FF FF FF */
void    HMI_SetText(const char *obj, const char *text);          /* ASCII 文本 */
void    HMI_SetTextBytes(const char *obj, const char *bytes, uint16_t len); /* GBK/原始字节文本 */
void    HMI_SetNum(const char *obj, int32_t val);
void    HMI_GotoPage(uint8_t page);
void    HMI_ProcessRx(void);                     /* 周期调用：解析屏上报帧 → 事件队列 */
uint8_t HMI_GetEvent(HMI_Event_t *ev);           /* 取一条事件，返回 1=有事件 */
void    HMI_Test(void);                          /* 通路自检：page0 写测试内容 */

#endif /* __HMI_TJC_H */
