/**
 * @file    test_task_code.c
 * @brief   task_code 解析逻辑的 PC 端单元测试（gcc 编译，不依赖 STM32 环境）
 *
 * 编译运行：
 *   gcc -Wall -Wextra -I../APP -o /tmp/test_task_code test_task_code.c ../APP/task_code.c
 *   /tmp/test_task_code
 */

#include <stdio.h>
#include <string.h>
#include "task_code.h"

static int g_pass = 0;
static int g_fail = 0;

static void check(const char *name, int ok, const char *detail)
{
        if (ok) {
                printf("✅ %s %s\n", name, detail ? detail : "");
                g_pass++;
        } else {
                printf("❌ %s %s\n", name, detail ? detail : "");
                g_fail++;
        }
}

int main(void)
{
        TaskPlan_t plan;
        char brief[96];

        TaskCode_Init();
        check("初始状态无效", TaskCode_IsValid() == 0, "");

        /* ---- 规则书示例码 ---- */
        check("示例码解析成功", TaskCode_Parse("156+123+516+231", &plan) == 1, "");
        check("第一批颜色=红黑浅蓝",
              plan.b1_color[0] == TASK_COLOR_RED &&
              plan.b1_color[1] == TASK_COLOR_BLACK &&
              plan.b1_color[2] == TASK_COLOR_LBLUE, "");
        check("第一批圆环=1/2/3",
              plan.b1_ring[0] == 1 && plan.b1_ring[1] == 2 && plan.b1_ring[2] == 3, "");
        check("第二批颜色=黑红浅蓝",
              plan.b2_color[0] == TASK_COLOR_BLACK &&
              plan.b2_color[1] == TASK_COLOR_RED &&
              plan.b2_color[2] == TASK_COLOR_LBLUE, "");
        check("第二批圆环=2/3/1",
              plan.b2_ring[0] == 2 && plan.b2_ring[1] == 3 && plan.b2_ring[2] == 1, "");
        check("原始码保存", strcmp(plan.raw, "156+123+516+231") == 0, "");

        /* ---- 颜色名称/索引 ---- */
        check("颜色1=红", strcmp(TaskCode_ColorName(1), "红") == 0, "");
        check("颜色6=浅蓝", strcmp(TaskCode_ColorName(6), "浅蓝") == 0, "");
        check("颜色7非法名", strcmp(TaskCode_ColorName(7), "?") == 0, "");
        check("颜色索引6→5", TaskCode_ColorIndex(6) == 5, "");
        check("颜色索引0非法", TaskCode_ColorIndex(0) == 0xFF, "");

        /* ---- 摘要串 ---- */
        TaskCode_FormatBrief(&plan, brief, sizeof(brief));
        printf("   摘要串: %s\n", brief);
        check("摘要含原始码", strstr(brief, "156+123+516+231") != 0, "");
        check("摘要含B2段", strstr(brief, "B2") != 0, "");

        /* ---- Accept 入库 ---- */
        check("Accept 成功", TaskCode_Accept("234+312+651+123") == 1, "");
        check("入库后有效", TaskCode_IsValid() == 1, "");
        check("入库内容可读",
              TaskCode_Current()->b1_color[0] == TASK_COLOR_YELLOW &&
              TaskCode_Current()->b1_color[2] == TASK_COLOR_GREEN &&
              TaskCode_Current()->b2_ring[0] == 1, "");

        /* ---- 非法输入：一个都不能过 ---- */
        check("过短拒绝", TaskCode_Parse("156+123+516+23", &plan) == 0, "");
        check("过长拒绝", TaskCode_Parse("156+123+516+2311", &plan) == 0, "");
        check("缺分隔符拒绝", TaskCode_Parse("156123516 231", &plan) == 0, "");
        check("分隔符错位拒绝", TaskCode_Parse("156+1235+16+231", &plan) == 0, "");
        check("非数字拒绝", TaskCode_Parse("15A+123+516+231", &plan) == 0, "");
        check("颜色0拒绝", TaskCode_Parse("056+123+516+231", &plan) == 0, "");
        check("颜色7拒绝", TaskCode_Parse("176+123+516+231", &plan) == 0, "");
        check("圆环0拒绝", TaskCode_Parse("156+023+516+231", &plan) == 0, "");
        check("圆环4拒绝", TaskCode_Parse("156+423+516+231", &plan) == 0, "");
        check("第4组圆环4拒绝", TaskCode_Parse("156+123+516+431", &plan) == 0, "");
        check("空串拒绝", TaskCode_Parse("", &plan) == 0, "");
        check("NULL拒绝", TaskCode_Parse(0, &plan) == 0, "");
        check("非法码不覆盖已有任务",
              TaskCode_Accept("156+999+516+231") == 0 && TaskCode_IsValid() == 1 &&
              strcmp(TaskCode_Current()->raw, "234+312+651+123") == 0, "");

        /* ---- 边界值：全颜色合法 ---- */
        check("全6色边界码合法", TaskCode_Parse("654+321+456+111", &plan) == 1, "");

        /* ---- 摘要串越界保护 ---- */
        TaskCode_FormatBrief(&plan, brief, 8);
        check("摘要小缓冲安全截断", strlen(brief) < 8, "");

        printf("\n结果: %d 通过 / %d 失败\n", g_pass, g_fail);
        return g_fail == 0 ? 0 : 1;
}
