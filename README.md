# SHARK 小车 · STM32 控制层技术日志

记录我在 SHARK 小车培训中的学习过程：从读懂平台代码，到写出能通过 PC 测试、并真正链接进固件的底盘控制。

> 本仓库只放**我自己写的那部分**（`App/`）。
> 平台层（`Support/`、`Core/`、`Drivers/`、ESP32、上位机）由培训包提供，不在本仓库中。
> 因此本仓库不能独立编译，需要在培训包的 `代码/STM32/` 目录下使用。

## 目录

```
App/
├─ main.c              10 ms 调度 + 停止处理 + 提交检查
├─ Chassis.c           底盘：按键位 → 三轴意图 → 四轮混控 → 同比限幅 → 电机映射
├─ Chassis.h           只有公开接口
└─ pc_test/
   ├─ test_chassis.c   底盘纯计算测试（12 项）
   └─ test_scheduler.c 调度 / 停止 / 超期测试（5 项）
```

## 硬件与接口

| 项目 | 配置 |
|---|---|
| MCU | STM32F103C8T6，HSE 8 MHz → 72 MHz，Flash 64 KiB / RAM 20 KiB |
| 四轮 PWM | PA0–PA3 → TIM2 CH1–4，约 20 kHz |
| 四轮方向 | PB0–PB3 |
| 云台 | PA6 / PA7 → TIM3 CH1/2，50 Hz，1100–1900 μs |
| 发射 | PA8 → TIM1 CH1，1 ms 低 + 300 ms 高，最小间隔 334 ms |
| 通信 | I²C 从机地址 0x12，100 kHz，ESP32 为主机（PB6 = SCL，PB7 = SDA） |

平台提供的接口（`Support/robot_support.h`）：

`Robot_Init` / `Robot_Poll` / `Robot_NowMs` / `Robot_ReadInput` / `Robot_Submit` / `Robot_Stop` / `Robot_EndCycle`

---

## 学习日志

### 2026-10-26 · 读懂入口

- `App/main.c` 原本只有 15 行：`Robot_Init()` + 死循环里 `Robot_Poll()` / `Robot_EndCycle()`。
- 读 `Support/` 源码，搞清了几个关键事实：
  - `Comm_ApplyOutput()` 在「未启用 / 已急停」时直接 `outputs_off()` 并 **返回 0** —— 所以「停止状态下提交返回 0」是正常的，不能当故障处理；
  - 输出权限 `ROBOT_ENABLE_MOTORS/SERVOS/FIRE` 默认全 0，此时 PWM 寄存器被强制写 0；
  - 1 ms 时基里做 250 ms 指令超时判断，前台看门狗由 `Robot_EndCycle()` 喂。

### 2026-10-27 · 第一版底盘

- 按讲义的单轴叠加写 `Chassis_Update()`：解码按键位 → 三轴 → 四轮 → 同比限幅 → 电机映射。
- 文件一开始放错目录（放在 `MDK-ARM/App`，而工程 include path 只含 `../App`），后改正。
- Keil 编译 0 Error / 0 Warning，但 `stm32.map` 里有：
  `Removing chassis.o(i.Chassis_Update), (204 bytes).`
  —— 文件编译了，但因为没人调用，被链接器丢掉了。**「文件在工程里」≠「代码在固件里」。**

### 2026-10-28 · 测试与进固件

- 用 gcc 在电脑上跑纯计算测试（`pc_test/test_chassis.c`），12 项全过。
- 故意把 `D/A` 换回错误顺序，确认测试能抓到：`A`、`D`、`W+D`、`W+D+E` 四项 FAIL，其余 8 项仍 PASS
  —— 单轴错会连累包含它的组合，这个失败模式和预期完全一致。
- 换上带 10 ms 调度的 `main.c`，Keil 重新构建：
  `Code=10102, 0 Error / 0 Warning`，map 里出现 `i.Chassis_Update 0x080003ec`，`Removing chassis.o` 消失。
- 写 `pc_test/test_scheduler.c`：用假的七个 `Robot_*` 函数在电脑上驱动 `main()` 的死循环，5 项全过。

---

## 关键设计

### 1. 四轮混控

坐标约定：`+vx` 向前、`+vy` 向右、`+vw` 顺时针；按键基线幅值 700。

| 单轴正向 | FL | FR | RL | RR |
|---|---|---|---|---|
| 向前 +vx | + | + | + | + |
| 向右 +vy | + | − | − | + |
| 顺时针 +vw | + | − | + | − |

叠加：

```
FL = vx + vy + vw
FR = vx - vy - vw
RL = vx - vy + vw
RR = vx + vy - vw
```

按键映射：`W/S` → vx，`D/A` → vy，`E/Q` → vw。

### 2. 同比限幅（不是逐轮截断）

求四个轮子绝对值的最大值 `p`：

- `p <= 1000`：保持原值；
- `p > 1000`：每个轮子都乘 `1000 / p`。

逐轮截断会改变四轮之间的比例，车会跑偏。整数实现要先乘后除，中间量用 `int32_t`。

例：`W+D+E` → 逻辑轮 `[2100, -700, 700, 700]` → `[1000, -333, 333, 333]`（向零取整）。

### 3. 电机映射

逻辑轮 → `MotorOutput{ duty_permille, forward }`：

1. 正负号 → `forward`；绝对值 → `duty_permille`；
2. 安装反转表 `{0, 1, 0, 1}` **只在这一处**应用，底盘公式里不做反转；
3. 输出为 0 时方向恒为 0。

### 4. 主循环调度

```c
now = Robot_NowMs();
dt  = now - last;              /* 无符号相减，49.7 天回绕也正确 */
if (dt >= 10) {
    if (dt <= 12) last += 10;  /* 准时：固定节拍，不积累漂移 */
    else          { overruns++; last = now; }  /* 超期：记录并重同步，不补跑 */
    step(dt);
}
Robot_EndCycle();              /* 前台正常结束才喂狗，不进中断 */
```

### 5. 停止策略

- 每轮先把整个 `RobotOutput` 清零，再按 `enabled` 填充；
- `enabled == 0` 时不调用任何控制模块，提交出去的是全零；
- 只有 `enabled == 1 && Robot_Submit() == 0` 才算被拒绝 → 记数并 `Robot_Stop()`。

---

## 验证记录

### PC 单元测试（底盘）

```
ok   W            got [  700   700   700   700]  want [  700   700   700   700]
ok   S            got [ -700  -700  -700  -700]  want [ -700  -700  -700  -700]
ok   W+S          got [    0     0     0     0]  want [    0     0     0     0]
ok   A            got [ -700   700   700  -700]  want [ -700   700   700  -700]
ok   D            got [  700  -700  -700   700]  want [  700  -700  -700   700]
ok   A+D          got [    0     0     0     0]  want [    0     0     0     0]
ok   Q            got [ -700   700  -700   700]  want [ -700   700  -700   700]
ok   E            got [  700  -700   700  -700]  want [  700  -700   700  -700]
ok   Q+E          got [    0     0     0     0]  want [    0     0     0     0]
ok   W+D          got [ 1000     0     0  1000]  want [ 1000     0     0  1000]
ok   W+D+E        got [ 1000  -333   333   333]  want [ 1000  -333   333   333]
ok   disabled+W   got [    0     0     0     0]  want [    0     0     0     0]
PASS: mixing, proportional scaling, disabled zeroing
```

测试用「反转表还原逻辑轮符号」的方式，让期望值可以直接对照讲义的单轴表，不受电机安装方向影响。

### PC 调度测试

```
ok   first control by 12ms          got=1 want=1
ok   W -> FL duty 700               got=700 want=700
ok   disabled -> all duty 0         got=0 want=0
ok   rejected submit -> Stop        got=1 want=1
ok   overrun: only 1 step           got=1 want=1
PASS: 10ms period / disabled zero / reject stop / overrun
```

### 构建结果

```
compiling main.c...
compiling Chassis.c...
linking...
Program Size: Code=10102 RO-data=330 RW-data=108 ZI-data=1932
"..\build\stm32.axf" - 0 Error(s), 0 Warning(s).
```

map 中：

```
main.o(i.app_step) refers to chassis.o(i.Chassis_Update) for Chassis_Update
i.Chassis_Update   0x080003ec   Section   chassis.o(i.Chassis_Update)
```

---

## 踩过的坑

| 现象 | 原因 | 修复 |
|---|---|---|
| `gcc: error: No such file or directory` | 在子目录里又写了一次子目录名，相对路径算错 | 先看提示符所在目录，再数往上几级 |
| `'xxx' declared 'static' but never defined` | 头文件里声明了 `static` 辅助函数 | 头文件只保留公开接口 |
| `A`、`D`、`W+D`、`W+D+E` 测试失败 | 参数顺序写成 `(A, D)`，`+vy` 指向了左 | 改成 `(D, A)` |
| `Removing chassis.o(i.Chassis_Update)` | `main.c` 里没有调用 | 加调度并调用，map 中出现 `0x080003ec` |
| 调度测试第一版 `W -> FL duty 700` 失败 | 测试自己忘了在开始前把 `enabled` 置 1 | 修测试。**看到 FAIL 先怀疑测试，再怀疑代码** |

---

## 待办

- [ ] 云台：保存目标脉宽，按 `dt` 积分（200 μs/s），限幅 1100–1900 μs
- [ ] 发射：0→1 边沿触发，334 ms 冷却，处理毫秒计数回绕
- [ ] 上板：权限仍为 0 时，用调试器观察 `g_diag`（输入 → 四轮 → 提交返回值）
- [ ] 单轮悬空核对方向与反转表
- [ ] 供电与接线方案确认（分压模块、共地、舵机独立供电）


遇到的问题由比如A和D Q和E的方向写反
许多代码逻辑不正确 比如一些类型变量的定义

## 参考

- 《SHARK 小车项目：从零搭建机器人程序》培训讲义
- 《SHARK 小车项目：上课与接线说明》
