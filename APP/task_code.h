/**
 * @file    task_code.h
 * @brief   搬运任务码解析与存储（供串口屏显示和搬运任务处理）
 *
 * 任务码格式（比赛规则 3) 任务编码）：
 *   四组三位数以 '+' 连接，如 156+123+516+231
 *   第1组：第一批三个物料的颜色和搬运顺序（颜色编号见下，1~6）
 *   第2组：第一批物料在粗加工区和暂存区的放置位置（圆环号 1~3）
 *   第3组：第二批三个物料的颜色和搬运顺序（1~6）
 *   第4组：第二批物料在粗加工区的放置位置（圆环号 1~3）
 *
 * 颜色编号：红1 黄2 蓝3 绿4 黑5 浅蓝6
 *
 * 圆环号语义（当前采用的解读，如与规则书/领队会议口径不符，只改 task_code.c
 * 顶部的 TASK_RING_INTERPRET 注释区和取值范围即可）：
 *   第2组数字 d_i：第 i 个物料放入粗加工区 d_i 号圆环，
 *                 粗加工完成后转入暂存区 d_i 号圆环（同号环）；
 *   第4组数字 d_i：第二批第 i 个物料最终放入粗加工区 d_i 号圆环。
 *
 * 本模块只依赖 stdint/string，可在 PC 上用 gcc 直接编译做单元测试
 * （见 tools/test_task_code.c）。不含 FreeRTOS/串口依赖。
 */

#ifndef __TASK_CODE_H
#define __TASK_CODE_H

#include <stdint.h>

#define TASK_BATCH_N        3       /* 每批物料数 */
#define TASK_RAW_MAX        16      /* "156+123+516+231" = 15 字符 + '\0' */

/* 颜色编号（与规则一致） */
#define TASK_COLOR_RED      1
#define TASK_COLOR_YELLOW   2
#define TASK_COLOR_BLUE     3
#define TASK_COLOR_GREEN    4
#define TASK_COLOR_BLACK    5
#define TASK_COLOR_LBLUE    6

/* 颜色编号取值范围 */
#define TASK_COLOR_MIN      1
#define TASK_COLOR_MAX      6
/* 圆环号取值范围（圆环 1~3） */
#define TASK_RING_MIN       1
#define TASK_RING_MAX       3

typedef struct
{
        char    raw[TASK_RAW_MAX];            /* 原始任务码 */
        uint8_t b1_color[TASK_BATCH_N];       /* 第一批颜色，按搬运顺序 */
        uint8_t b1_ring[TASK_BATCH_N];        /* 第一批圆环号（粗加工区/暂存区） */
        uint8_t b2_color[TASK_BATCH_N];       /* 第二批颜色，按搬运顺序 */
        uint8_t b2_ring[TASK_BATCH_N];        /* 第二批粗加工区圆环号 */
        uint8_t valid;                        /* 1=内容有效 */
} TaskPlan_t;

/* 解析任务码字符串。成功返回 1 并填充 out；格式/取值非法返回 0（out 不动） */
uint8_t TaskCode_Parse(const char *str, TaskPlan_t *out);

/* 解析 + 入库：成功覆盖当前任务并返回 1；非法串不覆盖、返回 0 */
uint8_t TaskCode_Accept(const char *str);

/* 清空当前任务（上电/重新扫码前调用） */
void    TaskCode_Init(void);

/* 当前任务是否有效 */
uint8_t TaskCode_IsValid(void);

/* 取当前任务只读指针（未入库时 valid=0）；串口屏/搬运任务从此读 */
const TaskPlan_t *TaskCode_Current(void);

/* 颜色编号 → HMI/串口屏用索引（0~5，顺序 红 黄 蓝 绿 黑 浅蓝），非法返回 0xFF */
uint8_t TaskCode_ColorIndex(uint8_t color);

/* 生成供串口屏/调试的摘要串，如：
 * "156+123+516+231|B1 1>1 5>2 6>3|B2 5>2 1>3 6>1"
 * 颜色与圆环都用编号数字表示；buf 至少 96 字节，size 为缓冲大小，始终 '\0' 结尾 */
void    TaskCode_FormatBrief(const TaskPlan_t *plan, char *buf, uint16_t size);

#endif /* __TASK_CODE_H */
