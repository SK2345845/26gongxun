/**
 * @file    hmi_tjc.c
 * @brief   淘晶驰 TJC 串口屏驱动实现（移植自 Gongchuang_27 暑期工程）
 *
 * 接收：ISR 已把字节灌入 usart2 环形缓冲（见 usart.c），
 *       HMI_ProcessRx() 周期逐字节解析成事件，存事件队列，
 *       业务层 HMI_GetEvent() 出队处理。解析与出队都在任务上下文，无需关中断。
 *
 * 发送：usart2_SendData 阻塞发送，每条指令以 FF FF FF 结尾。
 *       9600 波特率下一条 "t0.txt=..." 约 10~20ms，主循环 20ms 周期
 *       每次刷新别超过 2~3 条指令，否则会拖慢节拍（批量刷新降频到 200ms）。
 */

#include "hmi_tjc.h"
#include "usart.h"
#include <string.h>

/* 帧头 */
#define HMI_FRAME_TOUCH    0x65   /* 触摸事件：0x65 page comp event FF FF FF */
#define HMI_FRAME_VALUE    0x55   /* 滑块数值：0x55 page comp v[4] FF FF FF */
#define HMI_FRAME_GETSTR   0x70   /* get 字符串返回（本项目不使用，跳过） */
#define HMI_FRAME_GETNUM   0x71   /* get 数值返回（本项目不使用，跳过） */
#define HMI_FRAME_PAGEID   0x66   /* sendme 页面ID返回（本项目不使用，跳过） */

#define HMI_TERM_LEN       3

/*-----------------------------------------------------------
 * 事件队列（生产者=解析器，消费者=业务层，同任务上下文）
 *----------------------------------------------------------*/
static HMI_Event_t s_queue[HMI_EVT_QUEUE_LEN];
static volatile uint8_t s_q_head = 0;   /* 写位置 */
static volatile uint8_t s_q_tail = 0;   /* 读位置 */

static void EventQueue_Push(const HMI_Event_t *ev)
{
    uint8_t next = (uint8_t)((s_q_head + 1) % HMI_EVT_QUEUE_LEN);
    if (next == s_q_tail) {
        /* 队列满：丢弃最旧一条，避免阻塞解析 */
        s_q_tail = (uint8_t)((s_q_tail + 1) % HMI_EVT_QUEUE_LEN);
    }
    s_queue[s_q_head] = *ev;
    s_q_head = next;
}

/*-----------------------------------------------------------
 * 接收解析状态机
 *----------------------------------------------------------*/
typedef enum {
    RX_IDLE = 0,
    RX_TOUCH,      /* 0x65：再收 6 字节 */
    RX_VALUE,      /* 0x55：再收 9 字节 */
    RX_GETNUM,     /* 0x71：再收 7 字节 */
    RX_PAGEID,     /* 0x66：再收 4 字节 */
    RX_GETSTR,     /* 0x70：可变长，扫到 FF FF FF 为止 */
} HMI_RxState_t;

static HMI_RxState_t s_state = RX_IDLE;
static uint8_t s_buf[9];
static uint8_t s_idx  = 0;
static uint8_t s_need = 0;
static uint8_t s_ff_run = 0;   /* GETSTR 连续 0xFF 计数 */

void HMI_Init(void)
{
    s_state  = RX_IDLE;
    s_idx    = 0;
    s_need   = 0;
    s_ff_run = 0;
    s_q_head = s_q_tail = 0;
    while (usart2_ReadByte(0)) {}   /* 清空接收缓冲 */
}

/*-----------------------------------------------------------
 * 发送
 *----------------------------------------------------------*/
void HMI_SendRaw(const char *cmd)
{
    usart2_SendData((const uint8_t *)cmd, (uint16_t)strlen(cmd));
    usart2_SendData((const uint8_t *)"\xFF\xFF\xFF", 3);
}

void HMI_SetText(const char *obj, const char *text)
{
    char buf[80];
    uint16_t n = 0;
    uint16_t i;
    const char *p;

    /* 拼 obj.txt="text"（手工拼接，不引入 stdio） */
    for (p = obj; *p && n < sizeof(buf) - 1; p++) { buf[n++] = *p; }
    for (p = ".txt=\""; *p && n < sizeof(buf) - 1; p++) { buf[n++] = *p; }
    for (i = 0; text[i] != '\0' && n < sizeof(buf) - 1; i++) { buf[n++] = text[i]; }
    if (n < sizeof(buf) - 1) { buf[n++] = '"'; }
    buf[n] = '\0';
    HMI_SendRaw(buf);
}

void HMI_SetTextBytes(const char *obj, const char *bytes, uint16_t len)
{
    /* 供 GBK 中文动态文本：字节序列直接写入，不做字符串转义 */
    usart2_SendData((const uint8_t *)obj, (uint16_t)strlen(obj));
    usart2_SendData((const uint8_t *)".txt=\"", 6);
    usart2_SendData((const uint8_t *)bytes, len);
    usart2_SendData((const uint8_t *)"\"", 1);
    usart2_SendData((const uint8_t *)"\xFF\xFF\xFF", 3);
}

void HMI_SetNum(const char *obj, int32_t val)
{
    /* obj.val=<十进制>，手工数字格式化（int32 含负数） */
    char buf[48];
    char num[12];
    uint16_t n = 0, m = 0;
    const char *p;
    uint32_t u;
    char tmp[12];

    for (p = obj; *p && n < sizeof(buf) - 1; p++) { buf[n++] = *p; }
    for (p = ".val="; *p && n < sizeof(buf) - 1; p++) { buf[n++] = *p; }

    u = (val < 0) ? (uint32_t)(-(val + 1)) + 1U : (uint32_t)val;
    if (val < 0) { num[m++] = '-'; }
    if (u == 0) {
        num[m++] = '0';
    } else {
        uint8_t k = 0;
        while (u > 0) { tmp[k++] = (char)('0' + (u % 10U)); u /= 10U; }
        while (k > 0) { num[m++] = tmp[--k]; }
    }
    num[m] = '\0';
    for (m = 0; num[m] != '\0' && n < sizeof(buf) - 1; m++) { buf[n++] = num[m]; }
    buf[n] = '\0';
    HMI_SendRaw(buf);
}

void HMI_GotoPage(uint8_t page)
{
    char buf[10];
    uint8_t n = 0;

    buf[n++] = 'p'; buf[n++] = 'a'; buf[n++] = 'g'; buf[n++] = 'e'; buf[n++] = ' ';
    if (page >= 100) { buf[n++] = (char)('0' + page / 100); }
    if (page >= 10)  { buf[n++] = (char)('0' + (page / 10) % 10); }
    buf[n++] = (char)('0' + page % 10);
    buf[n] = '\0';
    HMI_SendRaw(buf);
}

/*-----------------------------------------------------------
 * 解析：喂入一个字节
 *----------------------------------------------------------*/
static void ParseFrame(HMI_RxState_t type)
{
    HMI_Event_t ev;

    /* 校验结尾必须是 FF FF FF（异常帧则丢弃） */
    if (s_buf[s_idx - 3] != 0xFF || s_buf[s_idx - 2] != 0xFF || s_buf[s_idx - 1] != 0xFF) {
        return;
    }

    memset(&ev, 0, sizeof(ev));

    switch (type) {
        case RX_TOUCH:
            ev.type      = HMI_EVT_TOUCH;
            ev.page      = s_buf[0];
            ev.component = s_buf[1];
            ev.event     = s_buf[2];
            EventQueue_Push(&ev);
            break;

        case RX_VALUE:
            ev.type      = HMI_EVT_VALUE;
            ev.page      = s_buf[0];
            ev.component = s_buf[1];
            ev.value     = (int32_t)((uint32_t)s_buf[2]          |
                                     ((uint32_t)s_buf[3] << 8)   |
                                     ((uint32_t)s_buf[4] << 16)  |
                                     ((uint32_t)s_buf[5] << 24));
            EventQueue_Push(&ev);
            break;

        default:
            /* GETNUM / PAGEID / GETSTR：本项目不用，收到即丢弃 */
            break;
    }
}

static void HMI_ParseByte(uint8_t byte)
{
    switch (s_state) {
        case RX_IDLE:
            switch (byte) {
                case HMI_FRAME_TOUCH:  s_state = RX_TOUCH;  s_idx = 0; s_need = 6; break;
                case HMI_FRAME_VALUE:  s_state = RX_VALUE;  s_idx = 0; s_need = 9; break;
                case HMI_FRAME_GETNUM: s_state = RX_GETNUM; s_idx = 0; s_need = 7; break;
                case HMI_FRAME_PAGEID: s_state = RX_PAGEID; s_idx = 0; s_need = 4; break;
                case HMI_FRAME_GETSTR: s_state = RX_GETSTR; s_ff_run = 0;          break;
                default: break;   /* 无关字节（含 0xFF）忽略 */
            }
            break;

        case RX_GETSTR:
            /* 字符串内容不含 FF FF FF，扫到 3 个连续 0xFF 即结束 */
            if (byte == 0xFF) {
                s_ff_run++;
                if (s_ff_run >= HMI_TERM_LEN) {
                    s_state = RX_IDLE;
                }
            } else {
                s_ff_run = 0;
            }
            break;

        default: {
            /* 定长帧：收满 need 字节后校验并解析 */
            HMI_RxState_t cur = s_state;
            s_buf[s_idx++] = byte;
            if (s_idx >= s_need) {
                ParseFrame(cur);
                s_state = RX_IDLE;
            }
            break;
        }
    }
}

void HMI_ProcessRx(void)
{
    uint8_t byte;
    while (usart2_ReadByte(&byte)) {
        HMI_ParseByte(byte);
    }
}

uint8_t HMI_GetEvent(HMI_Event_t *ev)
{
    if (s_q_head == s_q_tail) {
        return 0;
    }
    if (ev) {
        *ev = s_queue[s_q_tail];
    }
    s_q_tail = (uint8_t)((s_q_tail + 1) % HMI_EVT_QUEUE_LEN);
    return 1;
}

/* 通路自检：进 page0 写测试内容 */
void HMI_Test(void)
{
    HMI_GotoPage(0);
    HMI_SetText("t0", "HMI OK");
}
