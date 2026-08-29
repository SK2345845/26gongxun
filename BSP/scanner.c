/**
 * @file    scanner.c
 * @brief   GM65 条码/二维码识别模块驱动实现
 *          移植自 Gongchuang_27 暑期工程完整版，适配本工程 StdPeriph 串口层。
 *
 * 接收：ISR 已把字节灌入 uart5 环形缓冲（见 usart.c），
 *       Scanner_Process() 周期逐字节解析：
 *         - 0x02 0x00 开头的命令应答帧 → 校验后丢弃（非条码内容）
 *         - 其余字节 → 行缓冲，遇 0x0D/0x0A 结束一条 → 存码队列
 *       解析与出队都在任务上下文，无需关中断。
 *
 * 发送：命令帧经 uart5_SendData 阻塞发出（发后即返）；
 *       读寄存器阻塞等待应答（仅一次性状态查询用，不在周期循环里调）。
 */

#include "scanner.h"
#include "usart.h"
#include "FreeRTOS.h"
#include "task.h"
#include <string.h>

#if SCANNER_DEBUG_RAW
#include "delay.h"      /* 仅调试打印用 */
#endif

/* 命令帧字段 */
#define GM65_HEAD1       0x7E
#define GM65_HEAD2       0x00
#define GM65_CMD_READ    0x07   /* 读寄存器 */
#define GM65_CMD_WRITE   0x08   /* 写寄存器 */
#define GM65_CMD_SAVE    0x09   /* 存 Flash */

/* 寄存器地址 */
#define GM65_REG_MODE    0x0000 /* 扫描模式：0x99=命令触发 0x63=感应 0x22=连续 */
#define GM65_REG_TRIG    0x0002 /* 触发：写 0x01 触发一次扫码 */

#define GM65_RESP_TIMEOUT_MS  200   /* 读寄存器/应答帧超时 */
#define SCN_TIMEOUT_TICKS     pdMS_TO_TICKS(GM65_RESP_TIMEOUT_MS)

/*-----------------------------------------------------------
 * 扫码结果队列（生产者=解析器，消费者=业务层，同任务上下文）
 *----------------------------------------------------------*/
#define SCANNER_QUEUE_LEN   4
static Scanner_Code_t s_queue[SCANNER_QUEUE_LEN];
static volatile uint8_t s_q_head = 0;   /* 写位置 */
static volatile uint8_t s_q_tail = 0;   /* 读位置 */

static Scanner_Side_t s_side = SCANNER_SIDE_LEFT;

/*-----------------------------------------------------------
 * RX 解析状态机
 *----------------------------------------------------------*/
typedef enum {
    SCN_RX_LINE = 0,   /* 收条码行（遇 0x0D/0x0A 结束） */
    SCN_RX_RESP,       /* 收命令应答帧（0x02 0x00 开头） */
} Scanner_RxState_t;

static Scanner_RxState_t s_rx_state = SCN_RX_LINE;
static uint8_t  s_line[SCANNER_CODE_MAX + 2];
static uint16_t s_line_len = 0;
static uint8_t  s_resp[6 + 255];        /* 应答帧最长 6 + 255 字节 */
static uint16_t s_resp_idx = 0;
static TickType_t s_resp_start_tick = 0;
static Scanner_Stats_t s_stats;

#if SCANNER_DEBUG_RAW
static uint8_t s_raw_line_count = 0;
static char s_dbg_hex[4];

static void Scanner_DebugRaw(uint8_t b)
{
    const char hex_tab[] = "0123456789ABCDEF";

    if (s_raw_line_count == 0U) {
        usart1_SendString("[SCAN-RAW]");
    }
    s_dbg_hex[0] = ' ';
    s_dbg_hex[1] = hex_tab[(b >> 4) & 0x0F];
    s_dbg_hex[2] = hex_tab[b & 0x0F];
    s_dbg_hex[3] = '\0';
    usart1_SendString(s_dbg_hex);
    s_raw_line_count++;

    if (s_raw_line_count >= SCANNER_RAW_LINE_BYTES || b == 0x0D || b == 0x0A) {
        usart1_SendString("\r\n");
        s_raw_line_count = 0;
    }
}
#else
#define Scanner_DebugRaw(b) ((void)(b))
#endif

/*-----------------------------------------------------------
 * CRC_CCITT：poly 0x1021，初值 0，MSB 先，无反射
 * （与 GM65 手册 crc_cal_by_bit 逐位等价；注意用 32 位累加，
 *  避免 16 位截断导致 bit16 回灌丢失——旧简化版的坑）
 *----------------------------------------------------------*/
static uint16_t GM65_CRC16(const uint8_t *data, uint16_t len)
{
    uint32_t crc = 0;
    uint8_t i;

    while (len--) {
        for (i = 0x80; i != 0; i >>= 1) {
            crc <<= 1;
            if (crc & 0x10000UL) crc ^= 0x11021UL;
            if (*data & i)       crc ^= 0x1021UL;
        }
        data++;
    }
    return (uint16_t)(crc & 0xFFFFU);
}

/*-----------------------------------------------------------
 * 发送命令帧：{7E 00}{Type}{Len}{Addr[2]}{Data[Len]}{CRC[2]}
 * CRC 覆盖 Type~Data（帧头除外），高字节在前
 *----------------------------------------------------------*/
static uint8_t GM65_SendFrame(uint8_t type, uint16_t addr, const uint8_t *data, uint8_t len)
{
    uint8_t  frame[2 + 1 + 1 + 2 + 255 + 2];
    uint16_t idx = 0;
    uint8_t  i;

    frame[idx++] = GM65_HEAD1;
    frame[idx++] = GM65_HEAD2;
    frame[idx++] = type;
    frame[idx++] = len;
    frame[idx++] = (uint8_t)(addr >> 8);
    frame[idx++] = (uint8_t)(addr & 0xFF);
    for (i = 0; i < len; i++) {
        frame[idx++] = data[i];
    }

#if SCANNER_CMD_USE_CRC
    {
        uint16_t crc = GM65_CRC16(&frame[2], (uint16_t)(idx - 2));
        frame[idx++] = (uint8_t)(crc >> 8);
        frame[idx++] = (uint8_t)(crc & 0xFF);
    }
#else
    frame[idx++] = 0xAB;
    frame[idx++] = 0xCD;
#endif

    uart5_SendData(frame, idx);
    s_stats.tx_cmd_ok++;
    return 1;
}

uint8_t GM65_WriteReg(uint16_t addr, const uint8_t *data, uint8_t len)
{
    return GM65_SendFrame(GM65_CMD_WRITE, addr, data, len);
}

uint8_t GM65_SaveFlash(void)
{
    static const uint8_t zero = 0x00;
    return GM65_SendFrame(GM65_CMD_SAVE, 0x0000, &zero, 1);
}

uint8_t GM65_ReadReg(uint16_t addr, uint8_t len, uint8_t *out)
{
    uint8_t   resp[6 + 255];
    uint16_t  n = 0;
    uint16_t  expect = (uint16_t)(6 + len);   /* 02 00 00 len data[len] crc[2] */
    TickType_t start = xTaskGetTickCount();
    uint8_t   b;
    uint8_t   i;

    if (!GM65_SendFrame(GM65_CMD_READ, addr, &len, 1)) {
        return 0;
    }

    while ((xTaskGetTickCount() - start) < SCN_TIMEOUT_TICKS) {
        while (uart5_ReadByte(&b)) {
            if (n < sizeof(resp)) {
                resp[n++] = b;
            }
            if (n >= expect) {
                /* 校验帧头/类型/长度 + CRC（覆盖 Type+Len+Data） */
                if (resp[0] == 0x02 && resp[1] == 0x00 &&
                    resp[2] == 0x00 && resp[3] == len) {
                    uint16_t crc = GM65_CRC16(&resp[2], (uint16_t)(2 + len));
                    if (resp[4 + len] == (uint8_t)(crc >> 8) &&
                        resp[5 + len] == (uint8_t)(crc & 0xFF)) {
                        for (i = 0; i < len; i++) {
                            out[i] = resp[4 + i];
                        }
                        return 1;
                    }
                }
                return 0;   /* 帧格式/CRC 错误 */
            }
        }
    }
    return 0;   /* 超时 */
}

/*-----------------------------------------------------------
 * 结果队列
 *----------------------------------------------------------*/
static void ScannerQueue_Push(const Scanner_Code_t *code)
{
    uint8_t next = (uint8_t)((s_q_head + 1) % SCANNER_QUEUE_LEN);

    if (next == s_q_tail) {
        /* 队列满：丢最旧一条，避免阻塞解析 */
        s_q_tail = (uint8_t)((s_q_tail + 1) % SCANNER_QUEUE_LEN);
    }
    s_queue[s_q_head] = *code;
    s_q_head = next;
}

uint8_t Scanner_GetCode(Scanner_Code_t *code)
{
    if (s_q_head == s_q_tail) {
        return 0;
    }
    if (code != 0) {
        *code = s_queue[s_q_tail];
    }
    s_q_tail = (uint8_t)((s_q_tail + 1) % SCANNER_QUEUE_LEN);
    return 1;
}

void Scanner_GetStats(Scanner_Stats_t *stats)
{
    if (stats == 0) {
        return;
    }
    *stats = s_stats;
}

void Scanner_ClearStats(void)
{
    memset(&s_stats, 0, sizeof(s_stats));
}

static uint8_t Scanner_ResponseOk(void)
{
    uint8_t  len;
    uint16_t crc;

    if (s_resp_idx < 6U ||
        s_resp[0] != 0x02U ||
        s_resp[1] != 0x00U ||
        s_resp[2] != 0x00U) {
        return 0;
    }

    len = s_resp[3];
    if (s_resp_idx != (uint16_t)(6U + len)) {
        return 0;
    }

    crc = GM65_CRC16(&s_resp[2], (uint16_t)(2U + len));
    return (s_resp[4U + len] == (uint8_t)(crc >> 8) &&
            s_resp[5U + len] == (uint8_t)(crc & 0xFF)) ? 1U : 0U;
}

/*-----------------------------------------------------------
 * 应用接口
 *----------------------------------------------------------*/
void Scanner_Init(void)
{
    s_q_head = s_q_tail = 0;
    s_line_len = 0;
    s_rx_state = SCN_RX_LINE;
    s_resp_idx = 0;
    s_resp_start_tick = 0;
    s_side = SCANNER_SIDE_LEFT;
    Scanner_ClearStats();
}

void Scanner_SelectSide(Scanner_Side_t side)
{
    uint8_t data;

    /* 当前硬件没有左右切换脚，仅记录目标侧，禁止两路TX直接并联 */
    s_side = side;
    s_line_len = 0;
    s_rx_state = SCN_RX_LINE;
    s_resp_idx = 0;
    while (uart5_ReadByte(&data)) {}   /* 清空接收缓冲 */
}

Scanner_Side_t Scanner_GetSide(void)
{
    return s_side;
}

void Scanner_SetMode(uint8_t mode)
{
    static const uint8_t save = 0;

    GM65_WriteReg(GM65_REG_MODE, &mode, 1);
    GM65_SendFrame(GM65_CMD_SAVE, 0x0000, &save, 1);   /* 掉电不丢失 */
}

uint8_t Scanner_SetModeRuntime(uint8_t mode)
{
    return GM65_WriteReg(GM65_REG_MODE, &mode, 1);
}

uint8_t Scanner_Trigger(void)
{
    static const uint8_t trig = 0x01;

    return GM65_WriteReg(GM65_REG_TRIG, &trig, 1);
}

static void Scanner_PushLine(void)
{
    Scanner_Code_t code;
    uint16_t n = s_line_len;

    if (n == 0U) {
        return;
    }
    if (n > SCANNER_CODE_MAX) {
        n = SCANNER_CODE_MAX;
    }

    memcpy(code.code, s_line, n);
    code.code[n] = '\0';
    code.len  = n;
    code.tick = xTaskGetTickCount();
    ScannerQueue_Push(&code);
    s_stats.rx_lines++;
}

static void Scanner_ParseLineByte(uint8_t b)
{
    if (b == 0x0D || b == 0x0A) {
        Scanner_PushLine();
        s_line_len = 0;
    } else if (s_line_len < sizeof(s_line) - 1) {
        s_line[s_line_len++] = b;
    }
}

void Scanner_Process(void)
{
    uint8_t   b;
    uint8_t   b0;
    uint8_t   b1;
    uint8_t   b2;
    uint16_t  resp_expect;

    if (s_rx_state == SCN_RX_RESP &&
        (xTaskGetTickCount() - s_resp_start_tick) > SCN_TIMEOUT_TICKS) {
        s_rx_state = SCN_RX_LINE;
        s_resp_idx = 0;
        s_line_len = 0;
        s_stats.rx_resp_bad++;
    }

    while (uart5_ReadByte(&b)) {
        s_stats.rx_raw_bytes++;
        Scanner_DebugRaw(b);

        if (s_rx_state == SCN_RX_RESP) {
            /* 收命令应答帧：{02 00}{00}{len}{data[len]}{crc[2]}，共 6+len 字节 */
            if (s_resp_idx >= sizeof(s_resp)) {
                s_rx_state = SCN_RX_LINE;
                s_resp_idx = 0;
                s_line_len = 0;
                s_stats.rx_resp_bad++;
                continue;
            }

            s_resp[s_resp_idx++] = b;

            if (s_resp_idx == 2U && s_resp[1] != 0x00U) {
                /* 第二字节不是 0x00 → 不是应答帧，把已收字节还原回行解析 */
                b0 = s_resp[0];
                b1 = s_resp[1];
                s_rx_state = SCN_RX_LINE;
                s_resp_idx = 0;
                Scanner_ParseLineByte(b0);
                Scanner_ParseLineByte(b1);
                continue;
            }

            if (s_resp_idx == 3U && s_resp[2] != 0x00U) {
                b0 = s_resp[0];
                b1 = s_resp[1];
                b2 = s_resp[2];
                s_rx_state = SCN_RX_LINE;
                s_resp_idx = 0;
                Scanner_ParseLineByte(b0);
                Scanner_ParseLineByte(b1);
                Scanner_ParseLineByte(b2);
                continue;
            }

            if (s_resp_idx >= 4U) {
                resp_expect = (uint16_t)(6U + s_resp[3]);
                if (s_resp_idx >= resp_expect) {
                    if (Scanner_ResponseOk()) {
                        s_stats.rx_resp_ok++;
                    } else {
                        s_stats.rx_resp_bad++;
                    }
                    s_rx_state = SCN_RX_LINE;   /* 应答帧完整，丢弃（非条码内容） */
                    s_resp_idx = 0;
                    s_line_len = 0;
                }
            }
            continue;
        }

        /* SCN_RX_LINE */
        if (s_line_len == 0 && b == 0x02) {
            /* 0x02 开头疑似应答帧头，转入应答解析（后续按 02 00 校验） */
            s_rx_state = SCN_RX_RESP;
            s_resp_idx = 0;
            s_resp_start_tick = xTaskGetTickCount();
            s_resp[s_resp_idx++] = b;
            continue;
        }

        Scanner_ParseLineByte(b);
    }
}
