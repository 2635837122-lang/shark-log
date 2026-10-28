/* =========================================================================
 * App/main.c
 * 入口 + 10 ms 控制调度 + 停止处理 + 提交检查
 * ========================================================================= */
#include "robot_support.h"
#include "Chassis.h"

/* ---------------- 可调参数 ---------------- */
#define CTRL_PERIOD_MS   10u   /* 控制周期 */
#define CTRL_JITTER_MS    2u   /* 正常抖动：10~12 ms 视为准时 */

/* ---------------- 调试观察区 ----------------
 * 只写不读。Keil 里加进 Watch 窗口，就能实时看到这一轮算出了什么。*/
typedef struct {
    volatile uint32_t dt_ms;
    volatile uint32_t cycles;
    volatile uint32_t submits;
    volatile uint32_t rejected;
    volatile uint32_t overruns;
    volatile uint32_t max_dt_ms;
    volatile uint8_t  enabled;
    volatile uint8_t  movement;
    volatile uint8_t  submit_ok;
    volatile uint16_t duty[4];
    volatile uint8_t  forward[4];
} AppDiag;

volatile AppDiag g_diag;

/* 把整组输出置为“不驱动” */
static void app_zero_output(RobotOutput *out)
{
    uint32_t i;

    for (i = 0u; i < 4u; ++i) {
        out->motor[i].duty_permille = 0u;
        out->motor[i].forward       = 0u;
    }
    out->pan_us       = 0u;   /* 0 = 关闭 PWM */
    out->tilt_us      = 0u;
    out->fire_trigger = 0u;
}

static void app_store_diag(const RobotInput *in, const RobotOutput *out,
                           uint32_t dt_ms, uint8_t ok)
{
    uint32_t i;

    g_diag.dt_ms     = dt_ms;
    g_diag.enabled   = in->enabled;
    g_diag.movement  = in->movement;
    g_diag.submit_ok = ok;

    for (i = 0u; i < 4u; ++i) {
        g_diag.duty[i]    = out->motor[i].duty_permille;
        g_diag.forward[i] = out->motor[i].forward;
    }
}

/* 一轮控制：读输入 -> 算目标 -> 组装完整输出 -> 提交 */
static void app_step(uint32_t dt_ms)
{
    RobotInput  in;
    RobotOutput out;
    uint8_t     ok;

    /* 1. 一轮只读一次输入 */
    Robot_ReadInput(&in);

    /* 2. 先清零，再按允许条件填写 */
    app_zero_output(&out);

    if (in.enabled != 0u) {
        /* 3. 底盘：按键位 -> 三轴意图 -> 四轮目标 -> 电机映射 */
        Chassis_Update(&in, &out);
    }
    /* enabled == 0 时一个控制模块都不调用：输出保持全零，这就是停止 */

    /* 4. 整组提交 */
    ok = Robot_Submit(&out);

    ++g_diag.cycles;
    ++g_diag.submits;

    /* 5. 只有“允许控制时被拒绝”才算异常 */
    if (in.enabled != 0u && ok == 0u) {
        ++g_diag.rejected;
        Robot_Stop();
    }

    app_store_diag(&in, &out, dt_ms, ok);
}

int main(void)
{
    uint32_t last;
    uint32_t now;
    uint32_t dt;

    Robot_Init();
    last = Robot_NowMs();

    for (;;) {
        Robot_Poll();

        now = Robot_NowMs();
        dt  = now - last;   /* 无符号相减：回绕也正确 */

        if (dt >= CTRL_PERIOD_MS) {
            if (dt <= CTRL_PERIOD_MS + CTRL_JITTER_MS) {
                last += CTRL_PERIOD_MS;   /* 准时：固定节拍，不漂移 */
            } else {
                ++g_diag.overruns;        /* 超期：记录，重同步，不补跑 */
                last = now;
            }
            if (dt > g_diag.max_dt_ms) {
                g_diag.max_dt_ms = dt;
            }
            app_step(dt);
        }

        Robot_EndCycle();
    }
}
