/* 调度器测试：把 7 个底层函数换成假的，在电脑上跑 main 的死循环 */
#include <stdio.h>
#include <stdlib.h>
#include "robot_support.h"

static unsigned long now_ms;
static unsigned long extra_delay;
static RobotInput    fake_in;
static RobotOutput   last_out;
static unsigned long submits;
static unsigned long stops;
static unsigned long loops;
static unsigned char submit_ok = 1u;
static unsigned long failures;

/* ---- 假的七个底层函数，替换 Support 里的真货 ---- */
uint32_t Robot_NowMs(void) { return (uint32_t)now_ms; }
void     Robot_Init(void) { }
void     Robot_Poll(void) { now_ms += 1u + extra_delay; extra_delay = 0u; }
void     Robot_ReadInput(RobotInput *in) { *in = fake_in; }
uint8_t  Robot_Submit(const RobotOutput *out) { last_out = *out; ++submits; return submit_ok; }
void     Robot_Stop(void) { ++stops; }

static void check(const char *what, int ok, unsigned long got, unsigned long want)
{
    printf("%s %-30s got=%lu want=%lu\n", ok ? "ok  " : "FAIL", what, got, want);
    if (!ok) ++failures;
}

/* 每圈末尾被 main 调用，所以就拿它做“时间轴上的检查点” */
void Robot_EndCycle(void)
{
    unsigned long i;
    unsigned long max_duty;

    ++loops;

    if (loops == 1u) {              /* 一开始就启用控制，按键 W */
        fake_in.enabled  = 1u;
        fake_in.movement = 1u;
    }
    if (loops == 12u) {             /* 1) 10 ms 到点就该跑过 1 次控制 */
        check("first control by 12ms", submits == 1u, submits, 1u);
        check("W -> FL duty 700", last_out.motor[0].duty_permille == 700u,
              (unsigned long)last_out.motor[0].duty_permille, 700u);
        fake_in.enabled = 0u;       /* 2) 关掉控制 */
    }
    if (loops == 30u) {             /* 3) 禁用期间提交出去必须全零 */
        max_duty = 0u;
        for (i = 0u; i < 4u; ++i)
            if (last_out.motor[i].duty_permille > max_duty) max_duty = last_out.motor[i].duty_permille;
        check("disabled -> all duty 0", max_duty == 0u, max_duty, 0u);

        fake_in.enabled = 1u;       /* 4) 允许控制，但让底层拒绝提交 */
        submit_ok = 0u;
        stops = 0u;
    }
    if (loops == 42u) {
        check("rejected submit -> Stop", stops > 0u, stops, 1u);

        submit_ok   = 1u;           /* 5) 注入一次 80 ms 卡顿 */
        extra_delay = 80u;
        submits     = 0u;
    }
    if (loops == 52u) {             /* 超期后只能补跑 1 次，不能连跑 8 次 */
        check("overrun: only 1 step", submits == 1u, submits, 1u);
        printf("\n%s\n", failures ? "SOME CHECKS FAILED"
                                  : "PASS: 10ms period / disabled zero / reject stop / overrun");
        exit(failures ? 1 : 0);
    }
}