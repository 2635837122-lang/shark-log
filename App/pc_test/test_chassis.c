/* 底盘纯计算测试：给定按键 -> 调用 Chassis_Update -> 和讲义的表比较
 * 编译（在 代码\STM32\App 目录下）：
 *   gcc -std=c99 -Wall -Wextra -Werror -I ..\Support -I . pc_test\test_chassis.c Chassis.c -o test_chassis.exe
 * 运行：
 *   .\test_chassis.exe
 */
#include <stdio.h>
#include <string.h>
#include "Chassis.h"

/* 讲义 p14：默认安装反转 FL/FR/RL/RR = 0/1/0/1（负责人单轮核对后才改） */
static const unsigned char k_reverse[4] = {0u, 1u, 0u, 1u};
static unsigned failures;

/* 把“已含安装反转”的输出还原成逻辑轮目标，才能直接和讲义 p16 的单轴表比较 */
static int logical_wheel(const RobotOutput *out, unsigned i)
{
    int duty = (int)out->motor[i].duty_permille;
    unsigned char positive = (unsigned char)(out->motor[i].forward ^ k_reverse[i]);
    return positive ? duty : -duty;
}

static RobotOutput run(unsigned char enabled, unsigned char movement)
{
    RobotInput in;
    RobotOutput out;

    memset(&in, 0, sizeof in);
    memset(&out, 0, sizeof out);
    in.enabled  = enabled;
    in.movement = movement;
    Chassis_Update(&in, &out);
    return out;
}

static void check(const char *name, unsigned char enabled, unsigned char movement,
                  int w0, int w1, int w2, int w3)
{
    RobotOutput out = run(enabled, movement);
    int want[4];
    int got[4];
    unsigned i;
    int ok = 1;

    want[0] = w0; want[1] = w1; want[2] = w2; want[3] = w3;
    for (i = 0u; i < 4u; ++i) {
        got[i] = logical_wheel(&out, i);
        if (got[i] != want[i]) ok = 0;
    }
    printf("%s %-12s got [%5d %5d %5d %5d]  want [%5d %5d %5d %5d]\n",
           ok ? "ok  " : "FAIL", name,
           got[0], got[1], got[2], got[3], want[0], want[1], want[2], want[3]);
    if (!ok) ++failures;
}

int main(void)
{
    /* ---- 讲义 p08“先试这几组”：W、S、W+S、D、A+D、Q+E ---- */
    check("W",        1u,   1u,  700,  700,  700,  700);  /* +vx 向前：四轮同正 */
    check("S",        1u,   2u, -700, -700, -700, -700);
    check("W+S",      1u,   3u,    0,    0,    0,    0);
    check("A",        1u,   4u, -700,  700,  700, -700);  /* 向左 = -vy（p16） */
    check("D",        1u,   8u,  700, -700, -700,  700);  /* 向右 = +vy（p16） */
    check("A+D",      1u,  12u,    0,    0,    0,    0);
    check("Q",        1u,  16u, -700,  700, -700,  700);  /* 逆时针 = -vw（p16） */
    check("E",        1u,  32u,  700, -700,  700, -700);  /* 顺时针 = +vw（p16） */
    check("Q+E",      1u,  48u,    0,    0,    0,    0);

    /* ---- 讲义 p17：四轮一起缩小，不是逐轮截断 ---- */
    check("W+D",      1u,   9u, 1000,    0,    0, 1000);  /* [1400,0,0,1400]  -> 乘 1000/1400 */
    check("W+D+E",    1u,  41u, 1000, -333,  333,  333);  /* [2100,-700,700,700] -> 向零取整 */

    /* ---- 讲义 p12：禁用时执行输出必须全零 ---- */
    check("disabled+W", 0u, 1u,    0,    0,    0,    0);

    if (failures != 0u) {
        printf("\n%d item(s) FAILED\n", (int)failures);
        return 1;
    }
    printf("\nPASS: mixing, proportional scaling, disabled zeroing\n");
    return 0;
}