/**
 * @file    scanner.h
 * @brief   GM65 条码/二维码识别模块驱动（移植自 Gongchuang_27 暑期工程完整版）
 *
 * 串口：UART5  PC12(TX) / PD2(RX)，默认 9600 8N1，TTL-232 直连
 * 引脚：模块 4 针 —— 5V / TX(接PD2) / RX(接PC12) / GND
 *
 * 工作模式：命令触发（写寄存器 0x0000=0x99），
 *   触发一次扫码 → 写寄存器 0x0002=0x01；
 *   扫到内容后，模块以「数据 + 0x0D 结尾」上报（默认无前缀/后缀/CodeID）。
 *
 * 命令帧协议（主机 → 模块）：
 *   {0x7E 0x00} {Type} {Len} {Addr[2]} {Data[Len]} {CRC_CCITT[2]}
 *   Type = 0x07 读寄存器 / 0x08 写寄存器 / 0x09 存 Flash
 *   CRC_CCITT：poly 0x1021，初值 0，MSB 先，无反射，结果高字节在前
 *
 * 应答帧（模块 → 主机，命令用）：
 *   {0x02 0x00} {0x00} {Len} {Data[Len]} {CRC_CCITT[2]}
 *   Scanner_Process 内置应答帧状态机，命令 ACK 不会混进条码内容。
 *
 * 注意：模块默认可能是 USB 接口，需先扫设置码「10975611120049」切到串口，
 *   否则串口命令模块收不到（此时上位机软件/虚拟串口驱动可辅助确认）。
 */

#ifndef __SCANNER_H
#define __SCANNER_H

#include "stm32f4xx.h"
#include <stdint.h>

/* 条码/二维码内容最大字节数 */
#define SCANNER_CODE_MAX              64

/* 扫描模式（寄存器 0x0000 取值） */
#define SCANNER_MODE_CONTINUOUS      0x22   /* 连续扫描 */
#define SCANNER_MODE_SENSE           0x63   /* 感应模式 */
#define SCANNER_MODE_TRIGGER         0x99   /* 命令触发 */

/* GM65 手册允许命令帧 CRC 填 AB CD 关闭校验；联调期默认关闭，先保证模块能响应 */
#define SCANNER_CMD_USE_CRC           0

/* 打开后把扫码串口原始字节 hex 打到调试串口，定位"有RX但没解析出码" */
#define SCANNER_DEBUG_RAW             0
#define SCANNER_RAW_LINE_BYTES        16

/* 扫码结果 */
typedef struct
{
        char     code[SCANNER_CODE_MAX + 1];  /* 内容，'\0' 结尾 */
        uint16_t len;                          /* 实际长度（不含结尾符） */
        uint32_t tick;                         /* 收到时刻 xTaskGetTickCount() */
} Scanner_Code_t;

typedef enum
{
        SCANNER_SIDE_LEFT = 0,
        SCANNER_SIDE_RIGHT = 1
} Scanner_Side_t;

typedef struct
{
        uint32_t rx_raw_bytes;   /* 收到的原始字节总数 */
        uint32_t rx_lines;       /* 解析出的条码行数 */
        uint32_t rx_resp_ok;     /* 完整合法应答帧数 */
        uint32_t rx_resp_bad;    /* 异常/超时应答帧数 */
        uint32_t tx_cmd_ok;      /* 命令帧发送成功数 */
        uint32_t tx_cmd_fail;    /* 命令帧发送失败数 */
} Scanner_Stats_t;

/* ---- 扫码头选择（当前硬件为单路 UART5，仅记录目标侧并清缓冲） ---- */
void Scanner_SelectSide(Scanner_Side_t side);
Scanner_Side_t Scanner_GetSide(void);

/* ---- GM65 命令协议（底层） ---- */
uint8_t GM65_ReadReg(uint16_t addr, uint8_t len, uint8_t *out);         /* 读寄存器（阻塞） */
uint8_t GM65_WriteReg(uint16_t addr, const uint8_t *data, uint8_t len); /* 写寄存器（发后即返） */
uint8_t GM65_SaveFlash(void);                                           /* 保存到 Flash */

/* ---- 扫码应用 ---- */
void     Scanner_Init(void);                       /* 队列/状态机复位（UART5 由 board_init 完成） */
void     Scanner_SetMode(uint8_t mode);            /* 设置扫描模式 + 存 Flash */
uint8_t  Scanner_SetModeRuntime(uint8_t mode);     /* 仅运行时设置，不写 Flash */
uint8_t  Scanner_Trigger(void);                    /* 触发一次扫码，返回 1=命令已发出 */
void     Scanner_Process(void);                    /* 周期调用：解析 RX → 存码队列 */
uint8_t  Scanner_GetCode(Scanner_Code_t *code);    /* 取一条码，返回 1=有新码 */
void     Scanner_GetStats(Scanner_Stats_t *stats);
void     Scanner_ClearStats(void);

#endif /* __SCANNER_H */
