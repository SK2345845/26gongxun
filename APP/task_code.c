/**
 * @file    task_code.c
 * @brief   搬运任务码解析与存储实现
 *
 * 解析规则见 task_code.h。设计要点：
 *   - 纯 C 实现（只含 stdint/string），PC 可直接 gcc 单测；
 *   - 去重不在本模块做：扫码可能连读同一个码（连续/感应模式尤其），
 *     由调用方结合 Scanner_Code_t.tick 与上次入库内容判断是否重复；
 *   - 只做格式与取值范围校验，不做"每批三色互异"之类的强校验——
 *     规则若允许重复色，强校验会把合法码拒之门外。
 */

#include "task_code.h"
#include <string.h>

/* 当前任务（单例）。搬运任务处理与串口屏显示都从 TaskCode_Current() 读 */
static TaskPlan_t s_plan;

static uint8_t is_digit(char c)
{
        return (c >= '0' && c <= '9') ? 1 : 0;
}

static uint8_t digit_val(char c)
{
        return (uint8_t)(c - '0');
}

uint8_t TaskCode_Parse(const char *str, TaskPlan_t *out)
{
        TaskPlan_t plan;
        uint8_t  g;
        uint8_t  i;
        uint16_t len;

        if (str == 0 || out == 0) { return 0; }

        len = (uint16_t)strlen(str);
        if (len != 15U) { return 0; }   /* 3+1+3+1+3+1+3 = 15 */

        /* 逐字符校验格式：第 4/8/12 位必须是 '+'，其余必须是数字 */
        for (i = 0; i < 15U; i++)
        {
                if (i == 3U || i == 7U || i == 11U)
                {
                        if (str[i] != '+') { return 0; }
                }
                else if (!is_digit(str[i]))
                {
                        return 0;
                }
        }

        /* 四组数字填入并做取值范围校验 */
        for (g = 0; g < 4U; g++)
        {
                const char *grp = &str[g * 4U];
                uint8_t d0 = digit_val(grp[0]);
                uint8_t d1 = digit_val(grp[1]);
                uint8_t d2 = digit_val(grp[2]);

                if (g == 0U)          /* 第1组：第一批颜色 */
                {
                        if (d0 < TASK_COLOR_MIN || d0 > TASK_COLOR_MAX ||
                            d1 < TASK_COLOR_MIN || d1 > TASK_COLOR_MAX ||
                            d2 < TASK_COLOR_MIN || d2 > TASK_COLOR_MAX) { return 0; }
                        plan.b1_color[0] = d0;
                        plan.b1_color[1] = d1;
                        plan.b1_color[2] = d2;
                }
                else if (g == 1U)     /* 第2组：第一批圆环号 */
                {
                        if (d0 < TASK_RING_MIN || d0 > TASK_RING_MAX ||
                            d1 < TASK_RING_MIN || d1 > TASK_RING_MAX ||
                            d2 < TASK_RING_MIN || d2 > TASK_RING_MAX) { return 0; }
                        plan.b1_ring[0] = d0;
                        plan.b1_ring[1] = d1;
                        plan.b1_ring[2] = d2;
                }
                else if (g == 2U)     /* 第3组：第二批颜色 */
                {
                        if (d0 < TASK_COLOR_MIN || d0 > TASK_COLOR_MAX ||
                            d1 < TASK_COLOR_MIN || d1 > TASK_COLOR_MAX ||
                            d2 < TASK_COLOR_MIN || d2 > TASK_COLOR_MAX) { return 0; }
                        plan.b2_color[0] = d0;
                        plan.b2_color[1] = d1;
                        plan.b2_color[2] = d2;
                }
                else                  /* 第4组：第二批粗加工区圆环号 */
                {
                        if (d0 < TASK_RING_MIN || d0 > TASK_RING_MAX ||
                            d1 < TASK_RING_MIN || d1 > TASK_RING_MAX ||
                            d2 < TASK_RING_MIN || d2 > TASK_RING_MAX) { return 0; }
                        plan.b2_ring[0] = d0;
                        plan.b2_ring[1] = d1;
                        plan.b2_ring[2] = d2;
                }
        }

        memset(plan.raw, 0, sizeof(plan.raw));
        memcpy(plan.raw, str, 15U);
        plan.valid = 1;
        *out = plan;
        return 1;
}

uint8_t TaskCode_Accept(const char *str)
{
        TaskPlan_t plan;

        if (!TaskCode_Parse(str, &plan)) { return 0; }
        s_plan = plan;
        return 1;
}

void TaskCode_Init(void)
{
        memset(&s_plan, 0, sizeof(s_plan));
}

uint8_t TaskCode_IsValid(void)
{
        return s_plan.valid;
}

const TaskPlan_t *TaskCode_Current(void)
{
        return &s_plan;
}

const char *TaskCode_ColorName(uint8_t color)
{
        switch (color)
        {
        case TASK_COLOR_RED:    return "红";
        case TASK_COLOR_YELLOW: return "黄";
        case TASK_COLOR_BLUE:   return "蓝";
        case TASK_COLOR_GREEN:  return "绿";
        case TASK_COLOR_BLACK:  return "黑";
        case TASK_COLOR_LBLUE:  return "浅蓝";
        default:                return "?";
        }
}

uint8_t TaskCode_ColorIndex(uint8_t color)
{
        if (color < TASK_COLOR_MIN || color > TASK_COLOR_MAX) { return 0xFFU; }
        return (uint8_t)(color - 1U);
}

/* ---- 摘要串拼接助手（带越界保护，始终 '\0' 结尾） ---- */
static void append_str(uint16_t *pos, char *buf, uint16_t size, const char *s)
{
        while (*s != '\0')
        {
                if (*pos + 1U >= size) { break; }   /* 留 1 字节给 '\0' */
                buf[(*pos)++] = *s++;
        }
        buf[*pos] = '\0';
}

static void append_num(uint16_t *pos, char *buf, uint16_t size, uint8_t n)
{
        char s[2];   /* 必须带 '\0' 结尾，否则 append_str 会读到栈垃圾 */
        s[0] = (char)('0' + (n % 10U));
        s[1] = '\0';
        append_str(pos, buf, size, s);
}

void TaskCode_FormatBrief(const TaskPlan_t *plan, char *buf, uint16_t size)
{
        uint16_t pos = 0;
        uint8_t  i;

        if (buf == 0 || size == 0U) { return; }
        buf[0] = '\0';

        if (plan == 0 || !plan->valid)
        {
                append_str(&pos, buf, size, "NO TASK");
                return;
        }

        /* 原始码 */
        append_str(&pos, buf, size, plan->raw);

        /* 第一批 */
        append_str(&pos, buf, size, "|B1 ");
        for (i = 0; i < TASK_BATCH_N; i++)
        {
                if (i > 0U) { append_str(&pos, buf, size, " "); }
                append_str(&pos, buf, size, TaskCode_ColorName(plan->b1_color[i]));
                append_str(&pos, buf, size, ">环");
                append_num(&pos, buf, size, plan->b1_ring[i]);
        }

        /* 第二批 */
        append_str(&pos, buf, size, "|B2 ");
        for (i = 0; i < TASK_BATCH_N; i++)
        {
                if (i > 0U) { append_str(&pos, buf, size, " "); }
                append_str(&pos, buf, size, TaskCode_ColorName(plan->b2_color[i]));
                append_str(&pos, buf, size, ">环");
                append_num(&pos, buf, size, plan->b2_ring[i]);
        }
}
