# STM32：Cortex-M、时钟、GPIO、定时器、中断与 DMA

> **一句话定位**：这一章把"一块 STM32 板子上，一行 `HAL_xxx()` 背后到底发生了什么"讲透——CPU 内核、时钟树、引脚、定时器、中断、DMA、启动流程——是读懂 P3（STM32F407）和 P7（STM32H7）全部嵌入式代码的地基。

**本章学完你能：**

- 说清 Cortex-M 内核的寄存器、双栈指针、异常入口自动压栈，以及复位后 CPU 到底先做什么；
- 手算时钟树（HSE → PLL → SYSCLK → AHB/APB → 定时器时钟）；
- 手算 PWM 频率/占空比、编码器计数换算角度、输入捕获测脉宽；
- 解释 NVIC 抢占优先级与子优先级、为什么 FreeRTOS 里"中断优先级数字有上限"；
- 解释 DMA 解决什么问题、通道/流/请求、循环模式、缓冲区状态；
- 说出 F4 与 H7 的核心差异：Cache、MPU、TCM、总线矩阵、DMA 可访问区域，以及它们造成的典型 bug；
- 对比 HAL 与寄存器写法，并用英文回答"一个中断从触发到执行完发生了什么"。

**前置知识：**

- C 语言指针、位运算、`volatile`（见 [C语言与嵌入式八股](../../03_八股知识/01_计算机基础/04_C语言与嵌入式八股.md)）；
- 数字电路基础：高低电平、时钟、寄存器。

**相关项目：** [P3 RoboMaster 英雄机器人嵌入式控制](../../01_项目课程/P3_RoboMaster英雄机器人嵌入式控制/README.md)（STM32F407 / FreeRTOS / CAN）；[P7 可变腿长轮腿机器人](../../01_项目课程/P7_可变腿长轮腿机器人/README.md)（STM32H7 / FreeRTOS / FDCAN）。

## 目录

1. Cortex-M 内核要点
2. 启动流程：从复位到 `main()`
3. 时钟树
4. GPIO 的模式
5. 定时器：PWM、编码器模式、输入捕获
6. 中断与 NVIC
7. DMA
8. F4 与 H7 的区别：Cache、MPU、TCM、总线
9. HAL 与寄存器
10. 常见错误与故障现象
11. 面试问答
12. 本章要点速记、自测题、相关链接

---

## 1. Cortex-M 内核要点

### 1.1 它要解决什么问题

STM32 是"一颗芯片里装了 CPU + Flash + RAM + 一堆外设（定时器、串口、CAN、ADC……）"。CPU 部分是 ARM 公司设计的 Cortex-M 内核。F4 系列用 Cortex-M4F（带单精度浮点单元 FPU），H7 系列用 Cortex-M7（更深流水线、带 Cache、双精度 FPU 可选）。理解内核，才知道"为什么中断能打断主程序而不丢状态"。

### 1.2 寄存器与符号

| 寄存器 | 含义 |
|---|---|
| R0–R12 | 通用寄存器；R0–R3 传参/返回值，R12 = IP |
| R13 = SP | 栈指针，实际有两个：MSP（主栈指针）和 PSP（进程栈指针） |
| R14 = LR | 链接寄存器，存返回地址；在异常里存特殊值 EXC_RETURN |
| R15 = PC | 程序计数器 |
| xPSR | 程序状态：条件标志 N Z C V，当前异常号 |
| PRIMASK / BASEPRI / FAULTMASK | 中断屏蔽寄存器 |
| CONTROL | 选择线程模式用 MSP 还是 PSP、是否特权 |

两种模式：**线程模式（Thread）** 跑普通代码，**处理模式（Handler）** 跑异常/中断。两种特权级：特权、非特权。

双栈指针的意义：RTOS 里，**每个任务有自己的栈（用 PSP）**，而中断/内核（Handler 模式）总是用 **MSP**。任务切换其实就是"换 PSP 指向的那块栈，并恢复那个任务保存的寄存器"。这直接引出 [FreeRTOS](./02_FreeRTOS实时系统.md) 的 PendSV 上下文切换。

### 1.3 异常进入：硬件自动压栈

当中断发生，CPU 做的事（**这一步没有一行软件代码**）：

1. 把 8 个寄存器依次压入当前栈：`xPSR, PC, LR, R12, R3, R2, R1, R0`（共 8 个字 = 32 字节；带 FPU 且用过浮点时再压 S0–S15、FPSCR，共 26 字 = 104 字节——这叫"惰性压栈"）；
2. 从**向量表**取出该中断对应的函数地址，写入 PC；
3. 设置 LR = EXC_RETURN（如 0xFFFFFFF9 表示"返回线程模式、用 MSP"，0xFFFFFFFD 表示"返回线程模式、用 PSP"）；
4. 切到 Handler 模式。

退出时执行 `BX LR`（或 `POP {PC}`）且 PC 值为 EXC_RETURN，硬件识别后自动弹出 8 个寄存器，回到被打断处。**所以中断服务函数（ISR）可以写成普通 C 函数，不需要特殊的保存/恢复汇编**——这是 Cortex-M 相对老式 8051/ARM7 的重要进步。

**手算：中断入口延迟**。教学示例：Cortex-M4 在 168 MHz，零等待状态，硬件压栈 + 取向量约 12 个周期。则延迟 = 12 / 168 MHz = 71.4 ns。如果 ISR 里调用了耗时 20 μs 的函数，它占用的 CPU 时间远大于入口开销；所以 ISR 里要"短平快"。

### 1.4 向量表

向量表是一张"地址数组"：第 0 项是**初始 MSP 值**，第 1 项是 **Reset_Handler 地址**，随后是 NMI、HardFault、……、SysTick，再往后是外设中断（TIM2_IRQHandler、USART1_IRQHandler……）。启动文件（`startup_stm32f407xx.s`）里这张表用 `.word` 一项项排好，放在 Flash 起始地址。`SCB->VTOR` 可以改变表的位置（做 Bootloader 时必须改）。

### 1.5 F4 与 H7 内核层面的差异预览

| 项目 | F407（Cortex-M4F） | H743 一类（Cortex-M7） |
|---|---|---|
| 主频（典型） | 168 MHz | 400–480 MHz（某些型号更高） |
| 流水线 | 3 级 | 6 级，双发射 |
| 浮点 | 单精度 FPU | 单精度 + 双精度 FPU |
| 指令/数据 Cache | 无（有 ART 加速器预取 Flash） | 各 16 KB L1 Cache |
| TCM 紧耦合内存 | 无（有 64 KB CCM RAM，仅 CPU 可访问，DMA 不能访问） | ITCM 64 KB、DTCM 128 KB |
| MPU | 有（8 区域，常不用） | 有（16 区域，用于配置 Cache/缓冲属性，实战必需） |
| CAN | bxCAN（经典 CAN） | FDCAN（支持 CAN FD） |

详细见第 8 节。

---

## 2. 启动流程：从复位到 `main()`

### 2.1 为什么要懂

很多人以为 `main()` 是程序入口。实际上在 `main()` 之前，有一段启动代码决定了全局变量是否有初值。理解它，才能解释"为什么全局变量未初始化是 0"和"为什么在 `main()` 之前不能用 `printf`/`new`"。

### 2.2 步骤

1. **上电/复位**：硬件读取 Flash 地址 0x0000_0000（映射到 0x0800_0000）的前两个字：第一个字装入 MSP，第二个字装入 PC（即 `Reset_Handler`）；
2. **Reset_Handler**（汇编，在启动文件里）：
   - 把 `.data` 段（已初始化的全局/静态变量）的初值从 Flash 复制到 RAM；
   - 把 `.bss` 段（未初始化/零初始化的全局变量）清零；
   - 调用 `SystemInit()`（设置 FPU 使能、向量表偏移、复位时钟等；H7 还要处理电源、Cache 前的准备）；
   - 调用 `__libc_init_array()`（C++ 全局对象构造函数、`__attribute__((constructor))`）；
   - 跳转 `main()`。
3. **`main()`**：典型 CubeMX 生成：`HAL_Init()`（设置 SysTick 为 1 ms 节拍、NVIC 优先级分组）→ `SystemClock_Config()`（配 PLL）→ `MX_GPIO_Init()`/`MX_xxx_Init()` → 启动 RTOS（`osKernelStart()`）或进入 `while(1)`。

链接脚本（`.ld`）决定 `.text`（代码）、`.rodata`、`.data`、`.bss`、堆、栈各放在哪块内存。

```text
Flash (0x0800_0000)                RAM (0x2000_0000)
+-----------------+                +--------------------+
| 向量表          |                | .data (初值已复制) |
| .text 代码      |  --- 复制 -->  | .bss  (清零)       |
| .rodata 常量    |                | 堆 heap  -->       |
| .data 的初值    |                |        <-- 栈 stack|
+-----------------+                +--------------------+
```

**手算：栈溢出判断**。RAM 128 KB，.data+.bss 共 20 KB，堆设 8 KB，主栈 2 KB。则剩余 128−20−8−2 = 98 KB 未用。如果有 FreeRTOS，任务栈是从 FreeRTOS 自己的堆（`configTOTAL_HEAP_SIZE`）里分的，不在 MSP 栈里，见 [FreeRTOS 章](./02_FreeRTOS实时系统.md)。

---

## 3. 时钟树

### 3.1 它要解决什么问题

MCU 里不同部分需要不同速度：CPU 要快，低速外设要慢省电，USB 必须精确 48 MHz，定时器需要稳定。**时钟树**就是一棵"从晶振出发，经过倍频、分频，分配给各模块"的树。时钟配错，轻则波特率不对，重则整个系统跑不起来。

### 3.2 符号

- **HSE**：外部高速晶振（如 8 MHz 或 25 MHz）；**HSI**：内部 RC（F4 为 16 MHz，精度差）；
- **PLL**：锁相环倍频器。F4：$f_{VCO\_in} = f_{HSE}/M$，$f_{VCO} = f_{VCO\_in}\times N$，$f_{SYSCLK} = f_{VCO}/P$，$f_{USB} = f_{VCO}/Q$；
- **AHB 预分频**→ HCLK（CPU、DMA、存储器总线）；
- **APB1/APB2 预分频** → PCLK1/PCLK2（外设总线）；
- **定时器时钟规则**：若 APB 预分频系数 ≠ 1，则挂在该 APB 上的定时器时钟 = 2 × PCLK；若 = 1，则 = PCLK。

### 3.3 推导：为什么定时器时钟是 2×PCLK

APB 总线分频后很慢，但定时器希望保持较细的计数分辨率，所以 ST 设计成"分频了就乘 2 补回去"。这是一个硬件约定，不是推导出来的定律，但记住这个规则能避免 PWM 频率算错一半。

### 3.4 完整手算（教学示例）

F407，HSE = 8 MHz，目标 SYSCLK = 168 MHz，USB 需要 48 MHz。

1. 选 M = 8：$f_{VCO\_in}$ = 8/8 = 1 MHz（PLL 输入推荐 1–2 MHz）；
2. 选 N = 336：$f_{VCO}$ = 1 × 336 = 336 MHz（需在 100–432 MHz 范围）；
3. 选 P = 2：SYSCLK = 336/2 = 168 MHz ✓；
4. 选 Q = 7：USB = 336/7 = 48 MHz ✓；
5. AHB 预分频 1 → HCLK = 168 MHz；
6. F4 规定 APB1 最大 42 MHz、APB2 最大 84 MHz，所以 APB1 分频 4 → PCLK1 = 42 MHz；APB2 分频 2 → PCLK2 = 84 MHz；
7. 定时器：APB1 预分频 4 ≠ 1，所以 TIM2/3/4/5…时钟 = 2×42 = 84 MHz；APB2 预分频 2 ≠ 1，所以 TIM1/8… 时钟 = 2×84 = 168 MHz。

这个结果会直接用在下一节 PWM 计算里。

H7 的时钟树更复杂：有 PLL1/PLL2/PLL3，三个电源域（D1/D2/D3）各有总线时钟，每个外设（如 FDCAN、SPI、USART）还有独立的**内核时钟选择器**。FDCAN 的位定时计算必须用它的内核时钟（kernel clock），而不是 APB 时钟——这是 P7 里常被忽略的细节。

### 3.5 常见错误

- 晶振频率与 `HSE_VALUE` 宏不一致 → 所有基于时钟的计算（串口波特率、延时）整体偏移；
- 改了主频没改 `FLASH_LATENCY`（等待周期）→ 间歇性死机；F4 在 168 MHz、3.3 V 需要 5 个等待周期；
- 忘了定时器的 ×2 规则 → PWM 频率差 2 倍。

---

## 4. GPIO 的模式

### 4.1 它要解决什么问题

引脚既要能读按键、驱动 LED，又要能被外设（串口、SPI、定时器）接管，还要能读模拟电压。每个引脚内部有一套可配置的电路：输入缓冲、输出驱动、上下拉电阻、复用选择器、模拟开关。

### 4.2 模式表

| 模式 | 说明 | 典型用途 |
|---|---|---|
| 输入（浮空/上拉/下拉） | 读外部电平；浮空易受干扰，要有明确上下拉 | 按键、限位开关 |
| 通用输出推挽（Push-Pull） | 可输出强高/强低 | LED、使能脚 |
| 通用输出开漏（Open-Drain） | 只能拉低，高电平靠外部上拉 | I2C、线与总线 |
| 复用功能（AF）推挽/开漏 | 引脚交给外设；用 AF 编号选哪个外设 | TIM 的 PWM 输出、USART TX、CAN TX |
| 模拟 | 断开数字缓冲，接 ADC/DAC | 读电位器、土壤湿度 |

输出速度（Low/Medium/High/Very High）控制边沿斜率，不是"频率越高越好"：边沿越陡，EMI 与过冲越大。

### 4.3 原子操作 BSRR

写 `GPIOx->ODR |= bit` 是"读-改-写"三步，若中途被中断打断且中断也改同一端口，会丢失更新。BSRR 寄存器：低 16 位"写 1 置位"，高 16 位"写 1 复位"，**一次写入、不读**，天然原子：

```c
GPIOA->BSRR = (1U << 5);          // PA5 置高
GPIOA->BSRR = (1U << (5 + 16));   // PA5 置低
```

### 4.4 手算：上拉电阻与 I2C 上升时间

I2C 开漏总线，线上电容 C = 100 pF，上拉 R = 4.7 kΩ。上升时间 $t_r \approx 0.847 RC$ = 0.847 × 4700 × 100e-12 = 398 ns。快速模式（400 kHz）要求 $t_r$ ≤ 300 ns，不满足；换 2.2 kΩ：0.847 × 2200 × 100e-12 = 186 ns ✓。这就是"I2C 为什么要选上拉电阻"的定量答案。

### 4.5 在项目里怎么用

P3：LED、蜂鸣器、限位/按键是 GPIO；电机 CAN、IMU 的 SPI、串口都是 AF 模式。P4：Hi3861 上用 GPIO 控制水泵继电器（见 [OpenHarmony 平台章](./06_OpenHarmony_Hi3861_RK3588平台基础.md)）。

### 4.6 常见错误

- 忘了开 GPIO 端口时钟（`__HAL_RCC_GPIOA_CLK_ENABLE()`）→ 引脚没反应；
- AF 编号选错 → 外设信号没出来；
- I2C 引脚配成推挽 → 总线冲突；
- 悬空输入读到随机值。

---

## 5. 定时器：PWM、编码器模式、输入捕获

### 5.1 它要解决什么问题

定时器是 MCU 里"用硬件数时间"的模块：一个计数器按时钟递增，到设定值就产生事件。它让 CPU 不必靠软件延时来产生精确波形、测量脉宽、读正交编码器。

### 5.2 基本结构与符号

- **PSC**（预分频，寄存器值 = 分频系数−1）：计数时钟 $f_{cnt} = f_{tim}/(PSC+1)$；
- **ARR**（自动重装载值）：计数器数到 ARR 后回零（向上计数）；
- **CNT**：当前计数值；
- **CCRx**：捕获/比较寄存器，用于比较输出或捕获输入；
- 更新事件频率：$f_{update} = \dfrac{f_{tim}}{(PSC+1)(ARR+1)}$。

通用定时器（TIM2–TIM5）、高级定时器（TIM1/TIM8，带互补输出与死区，用于三相逆变）、基本定时器（TIM6/TIM7，只计时，常用来触发 DAC/ADC）。F4 上 TIM2/TIM5 是 32 位，其余多为 16 位（最大 65535）。

### 5.3 PWM

PWM 模式 1，向上计数：CNT < CCR 时输出高，CNT ≥ CCR 时输出低。

- 频率：$f_{PWM} = f_{tim}/((PSC+1)(ARR+1))$；
- 占空比：$D = \dfrac{CCR}{ARR+1}$；
- 分辨率：占空比最小步进 = $1/(ARR+1)$。

**手算 1（教学示例）**：定时器时钟 84 MHz（见 3.4），要 1 kHz PWM、占空比 25%。

1. 取 PSC = 83 → 计数频率 84 MHz/84 = 1 MHz；
2. 要 1 kHz → ARR+1 = 1 MHz/1 kHz = 1000 → ARR = 999；
3. 占空比 25% → CCR = 0.25 × 1000 = 250；
4. 分辨率 1/1000 = 0.1%。

**手算 2：舵机**。舵机典型要 50 Hz，高电平 0.5–2.5 ms 对应 0–180°（以所用舵机手册为准）。PSC = 83（1 MHz 计数）、ARR = 19999 → 周期 20 ms，50 Hz ✓。1.5 ms → CCR = 1500。分辨率 1 μs ≈ 0.09°。

**手算 3：电调/驱动板的 PWM 用作力矩或速度指令**。若用 20 kHz 做电机驱动，$f_{tim}$ = 168 MHz，不分频（PSC = 0），ARR+1 = 168e6/20e3 = 8400，分辨率 1/8400 ≈ 0.012%，约 13 位。频率越高，分辨率越低，这是 PWM 的基本折中。

### 5.4 编码器模式（正交编码器接口）

**问题**：增量式光电/磁编码器输出两路相差 90° 的方波 A、B。靠 A、B 谁超前判断转向，靠边沿数计算位置。用 CPU 中断数边沿，转速一高就中断风暴；STM32 定时器有**硬件编码器接口**，自动数、自动判方向，CPU 只需读 CNT。

**四倍频**：A、B 的每个上升沿和下降沿都计数，所以每个"线"产生 4 个计数。

- 编码器线数 $N_{line}$（PPR），则每转计数 $N_{cnt} = 4N_{line}$；
- 角度：$\theta = 2\pi \cdot \dfrac{\Delta cnt}{4N_{line}}$；
- 速度：$\omega = \dfrac{2\pi \Delta cnt}{4N_{line}\,\Delta t}$。

**16 位计数器回绕处理（核心技巧）**：

```c
static uint16_t last = 0;
uint16_t now = __HAL_TIM_GET_COUNTER(&htim3);
int16_t  delta = (int16_t)(now - last);   // 无符号减，再按补码解释
last = now;
```

只要每个采样周期内真实变化量绝对值 < 32768，`(int16_t)(now - last)` 就总是正确。

**手算**：last = 65500，now = 20（正转越过 65535 回绕）。无符号 16 位减法：20 − 65500 mod 65536 = 56；转 int16 仍为 56。✓ 反转例：last = 10，now = 65530：65530 − 10 = 65520；转 int16 = 65520 − 65536 = −16 ✓。

**再算速度**：编码器 500 线，$N_{cnt}$ = 2000/转；1 ms 采样一次得 delta = 30 → 30/2000 = 0.015 转/ms = 15 转/秒 = 900 rpm = 94.2 rad/s。注意**量化误差**：delta 的最小分辨为 1 个计数，对应速度 1/2000 转/ms = 0.5 转/秒 = 30 rpm。低速时 1 ms 采样的速度非常"阶梯"，这就是为什么常用更长采样窗、M/T 法或低通滤波。详见 [控制理论：数字控制与实时性](../04_控制理论/08_数字控制与实时性.md) 与 [传感器章](../05_状态估计/01_传感器_IMU_编码器_激光雷达.md)。

### 5.5 输入捕获

**问题**：测量外部脉冲的宽度/周期（如超声波回波、PWM 遥控信号、转速脉冲）。

**原理**：配置某通道为输入捕获，边沿到来时硬件把当前 CNT 锁存进 CCR，并可触发中断/DMA。

**手算**：计数频率 1 MHz（tick = 1 μs）。上升沿捕获 c1 = 1200，下降沿捕获 c2 = 2700，脉宽 = (2700−1200) × 1 μs = 1500 μs = 1.5 ms（对应舵机中位）。若 c2 < c1（发生回绕），脉宽 = (c2 − c1) mod (ARR+1)，无符号减法自然处理。

### 5.6 在项目里怎么用

- P3：PWM 常用于摩擦轮电调（如 RoboMaster 摩擦轮用 PWM 控制 ESC，**本项目通常取标准舵机式 PWM：周期 20 ms（50 Hz）、高电平 1000–2000 μs 对应零到满油门；但 P3 的摩擦轮闭环速度环依赖电机反馈转速（RPM），因此摩擦轮电机走 CAN 带反馈，PWM 主要留给蜂鸣器、LED 与 IMU 加热**）、蜂鸣器、LED 呼吸灯；定时器也常作为 1 ms 时基或 IMU 加热 PID 的 PWM；
- P7：轮腿的车轮/关节电机走 FDCAN 总线，定时器多用于时基与统计任务运行时间（见 [FreeRTOS 章](./02_FreeRTOS实时系统.md)）；
- P2：底盘轮速的编码器反馈通过电机驱动器上报，MCU 侧编码器模式是原理背景（见 [电机与编码器章](./05_电机_编码器_驱动与力矩控制.md)）。

### 5.7 常见错误

- 忘记 PSC/ARR 是"值−1"；
- 改了 PWM 频率没重算 CCR 的比例；
- 编码器接口 A/B 接反 → 正转计数减少，方向反了；
- 编码器毛刺 → 加输入滤波（TIM 的 ICxF 字段）；
- 在高级定时器 TIM1/TIM8 忘记使能 `MOE`（主输出使能）→ PWM 不输出。

---

## 6. 中断与 NVIC

### 6.1 它要解决什么问题

CPU 不能一直"轮询"事件（串口来了字节没？编码器转了没？），否则要么漏事件，要么浪费算力。**中断**让外设在事件发生时"打断"CPU，CPU 去执行对应服务函数，处理完回到原处。**NVIC（嵌套向量中断控制器）** 负责使能、挂起、排优先级。

### 6.2 符号与规则

- 每个中断有一个**优先级数值，数值越小，优先级越高**；
- STM32 只实现了 4 位优先级（16 级，0–15），在 NVIC 里占寄存器高 4 位；
- **优先级分组**（`NVIC_PriorityGroup_x`）把这 4 位划分为"抢占优先级（preempt）"和"子优先级（sub）"。HAL 默认 `NVIC_PRIORITYGROUP_4`：4 位全为抢占，无子优先级——**这也是 FreeRTOS 要求的设置**；
- **抢占**：高抢占优先级的中断可以打断正在执行的低抢占优先级 ISR；相同抢占优先级不会互相打断；子优先级仅在"同时挂起"时决定谁先执行。

### 6.3 一个中断从触发到执行完（面试高频）

1. 外设置位中断标志（如 UART 收到一字节 `RXNE`）；
2. 若该外设中断被使能，信号送至 NVIC，NVIC 把它置为挂起（pending）；
3. 若其优先级高于当前运行代码的优先级（且未被 PRIMASK/BASEPRI 屏蔽），CPU 响应：硬件压栈、取向量、进入 ISR；
4. ISR 里**必须清除中断标志**（HAL 的 `HAL_UART_IRQHandler` 会做），否则退出后标志仍置位，立刻再次进入；
5. ISR 通常设一个标志、写一个队列、或给任务发通知，然后返回；
6. 硬件弹栈，回到原代码。

### 6.4 FreeRTOS 与中断优先级的关系（重要，很多 bug 来源）

Cortex-M3/4/7 上，FreeRTOS 通过 **BASEPRI** 寄存器进入临界区：把 BASEPRI 设为 `configMAX_SYSCALL_INTERRUPT_PRIORITY`，则**优先级数值 ≥ 该值**的中断被屏蔽，而数值更小（更紧急）的中断不被屏蔽，也**不允许调用任何 FreeRTOS API**。

**手算（教学示例）**：4 位优先级，`configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY = 5`。内核配置值 = 5 << (8−4) = 0x50。

- 一个 CAN 接收中断，优先级设为 6 → 在内核可管范围内，可以调用 `xQueueSendFromISR`；
- 一个"紧急"定时器中断设为 2 → 在内核屏蔽范围之外，临界区里它仍可响应（延迟更低、更实时），但**不能**调用 `FromISR` API，否则断言失败或随机崩溃；
- 设为 0 则最高，但同样不能用 RTOS API；
- 最低优先级 15 保留给 PendSV 和 SysTick（`configKERNEL_INTERRUPT_PRIORITY`）。

### 6.5 常见错误

- HAL 回调和 ISR 分不清：HAL 在 `xxx_IRQHandler` 里处理标志后再调用你写的 `HAL_xxx_Callback`，这是**在中断上下文里**执行的；
- ISR 里做 `HAL_Delay`（依赖 SysTick，SysTick 优先级低于当前 ISR → 死锁）；
- ISR 里 `printf`、`malloc`、长循环；
- 没清标志导致中断风暴；
- 共享变量未加 `volatile` 或未保护；
- 优先级分组不是 4（FreeRTOS 断言）。

---

## 7. DMA

### 7.1 它要解决什么问题

外设和内存之间要搬大量数据：ADC 连续采样、UART 收一大包、SPI 读 IMU。让 CPU 一字节一字节搬（"CPU 拷贝"），既慢又占着 CPU。**DMA（直接存储器访问）控制器**是一个独立的"搬运工"：你给它源地址、目的地址、数量、方向，它在总线上自己搬，搬完发中断通知。CPU 在这段时间可去做控制计算。

### 7.2 概念与符号

- F4：DMA1、DMA2，每个有 8 个 **Stream（流）**，每个流可通过 **Channel（通道 0–7）** 选择外设请求；即"哪个外设请求映射到哪个流"是查数据手册表格决定的；
- H7：DMA1/DMA2（沿用 Stream）+ **DMAMUX** 请求复用器（任意外设请求可路由到任意流）+ BDMA（D3 域）+ MDMA（主 DMA，可访问 TCM）；
- 配置要素：源地址、目的地址、数据宽度（字节/半字/字）、传输数量 NDTR、地址是否递增、方向（外设→内存、内存→外设、内存→内存）、模式（Normal/Circular）、优先级、FIFO；
- **半传输中断（HT）** 与 **传输完成中断（TC）**：缓冲区搬到一半、搬满各触发一次——配合**循环模式**做"乒乓缓冲"。

### 7.3 推导：为什么 DMA 省 CPU（手算）

教学示例：ADC 4 通道扫描，每通道 1 kHz 采样，每次 2 字节。

- 数据率 = 4 × 1000 × 2 = 8000 B/s；
- 若每个采样用一次中断：进入退出 + 读取 + 存储约 100 个周期（含 ISR 内部代码），168 MHz 下每次约 0.6 μs，每秒 4000 次 → 2.4 ms/s = 0.24% CPU。看起来小，但**抖动**很差；
- 如果用 DMA 循环搬运，一轮 4 个采样才触发一次完成中断（1 kHz）→ 中断频率降为 1/4，CPU 负担更小，且采样不受 ISR 延迟影响；
- 高速场景（如 1 MS/s 的 ADC 或 10 MHz 的 SPI 读图像），中断方式根本做不到，必须 DMA。

### 7.4 循环模式与缓冲区状态

循环模式：NDTR 减到 0 后自动重装，继续从缓冲区头开始写。

**手算**：UART 接收缓冲 256 字节，循环模式。某一时刻读寄存器 NDTR = 200。已写入字节数 = 256 − 200 = 56，写指针位置 = 56（下一个字节写入 buf[56]）。上次处理到 read_pos = 30，则新数据是 buf[30..55]，共 26 字节。若 write_pos = 10 < read_pos = 250，则发生了回绕，新数据 = buf[250..255] 加 buf[0..9]，共 6 + 10 = 16 字节。这就是 [UART 章](./04_UART_SPI_I2C_以太网_TCP.md) "DMA + 空闲中断"收包的基础。

### 7.5 常见错误与 H7 特有问题

- **DMA 与 Cache 一致性**（H7）：DMA 写内存，CPU 读到的是 Cache 里的旧数据；CPU 写内存后 DMA 读到的是主存里的旧数据。解决：缓冲区放在**非缓存区域**（MPU 配置），或读前 `SCB_InvalidateDCache_by_Addr`、写后 `SCB_CleanDCache_by_Addr`，且缓冲区 32 字节对齐（Cache line 32 B）、长度是 32 的整数倍；
- **DMA 不能访问某些 RAM**：F4 的 CCM RAM、H7 的 DTCM/ITCM（DMA1/2 不可访问），把缓冲区放进去就搬不了；
- 循环模式下缓冲区读取速度跟不上，数据被覆盖；
- 传输数量、数据宽度和内存声明类型不一致（把 `uint8_t` 缓冲当 `uint16_t` 配）；
- 忘记开 DMA 时钟、忘记在外设里使能 DMA 请求位。

---

## 8. F4 与 H7 的区别：Cache、MPU、TCM、总线

### 8.1 为什么 H7 更"难"

F4 简单：CPU 直接访问 SRAM，所有 RAM 对 DMA 一视同仁（除 CCM），没有 Cache，读到的就是内存的真值。H7 的速度（480 MHz）远快于 Flash/SRAM 访问速度，必须靠 Cache、TCM 和多层总线矩阵"喂饱"CPU；代价就是**一致性与内存放置**成了程序员的责任。

### 8.2 H7 的内存布局（以 H743 为例，容量为典型值，以手册为准）

| 区域 | 地址（典型） | 大小 | 特点 |
|---|---|---|---|
| ITCM | 0x0000_0000 | 64 KB | CPU 零等待取指，DMA 不可访问 |
| DTCM | 0x2000_0000 | 128 KB | CPU 零等待数据，DMA1/2 不可访问 |
| AXI SRAM（D1） | 0x2400_0000 | 512 KB | 通用，可 Cache，DMA 可访问 |
| SRAM1/2/3（D2） | 0x3000_0000 | 共 288 KB | 常放以太网/USB/DMA 缓冲 |
| SRAM4（D3） | 0x3800_0000 | 64 KB | 低功耗域，BDMA 可访问 |

> **提示**：链接脚本默认把栈、`.data`、`.bss` 放 DTCM（最快）。若 DMA 缓冲被放进 DTCM，就会"搬不了"——要显式指定 `__attribute__((section(".axi_sram")))` 之类的段。

### 8.3 Cache 一致性手算

L1 数据 Cache 行大小 32 字节。你有一个 100 字节的 DMA 接收缓冲 `uint8_t rx[100]`，放在 AXI SRAM 0x2400_0010 处。

- 问题 1：起始地址 0x10 不是 32 的倍数 → 缓冲区和前面的变量共享 Cache 行；`Invalidate` 会把相邻变量的**脏数据**一起丢弃，导致其他变量被"还原为旧值"，极难排查；
- 问题 2：长度 100 不是 32 的倍数 → 同样会波及后面的数据；
- 正确做法：`__attribute__((aligned(32))) uint8_t rx[128];`（128 = 4 × 32）→ 对齐且长度为 32 倍数。DMA 接收完成回调里：`SCB_InvalidateDCache_by_Addr((uint32_t*)rx, 128);` 再读。
- 发送方向：CPU 填 `tx` 后，`SCB_CleanDCache_by_Addr(tx, len)` 把 Cache 写回内存，再启动 DMA。

更省事的做法：用 MPU 把该区域配置为 **Non-cacheable**（或 Write-through），这样一致性问题从源头消失，代价是该区域 CPU 访问变慢。

### 8.4 MPU

MPU（内存保护单元）把地址空间划分成若干区域，每个区域可设：访问权限、是否可执行、是否可缓存、缓冲模式、共享属性。在 F4 上通常不配置；在 H7 上，**MPU 是控制"哪些区域走 Cache"的唯一手段**，CubeMX 生成的 `MPU_Config()` 常用于：

- 把某个 SRAM 区域设成 non-cacheable，供 DMA 缓冲、以太网描述符使用；
- 把 0 地址设为 no-access，捕获空指针访问。

### 8.5 总线矩阵与"双口"

H7 为多主设备（CPU、DMA1/2、MDMA、以太网、LTDC、SDMMC……）同时访问多块 SRAM，设计了 AXI/AHB 总线矩阵，不同域（D1/D2/D3）之间还有桥。含义：

- 不同 SRAM 可被不同主设备**并行**访问（类似"多口"），带宽高；同一块 SRAM 同时被 CPU 和 DMA 访问则仲裁；
- 数据在两域间走桥会增加延迟；
- 部分型号（如 H745/H747）有**双核**（M7+M4）和硬件信号量 HSEM、共享 SRAM，两个核心需要核间通信。

本项目的做法是：P7 使用 STM32H7 系列（Cortex-M7，具体子型号以板子为准），FDCAN 的收发缓冲使用外设自带的消息 RAM（SRAM3 一侧，由 CubeMX 按默认布局分配），DMA 缓冲放在 AXI SRAM 而不是 DTCM；MPU 采用 CubeMX 默认配置，没有额外自定义 Cache 区域，因此靠“DMA 缓冲所在区域设为非缓存或在读写前后做 Cache 维护”来保证一致性。面试时可直接按这个层次说明，并强调 DTCM 对 DMA 不可达这一点。

### 8.6 对照表（速查）

| 问题 | F407 | H7 |
|---|---|---|
| DMA 缓冲要考虑 Cache 吗 | 不需要 | 需要（对齐 + 失效/清理，或 MPU） |
| CPU 全速读 RAM | 168 MHz 约零等待 | 取决于区域：TCM 最快，AXI 次之 |
| 哪里 DMA 访问不了 | CCM RAM | DTCM/ITCM（对 DMA1/2） |
| CAN | bxCAN 经典 CAN，用 28 个滤波器组 | FDCAN，用 RAM 里的"消息 RAM"配置滤波器和 FIFO（见 [CAN 章](./03_CAN_FDCAN与电机通信.md)） |
| 时钟配置 | 一棵树 | 多 PLL、外设内核时钟需单独选 |

---

## 9. HAL 与寄存器

### 9.1 三层写法

1. **寄存器直接操作**：`GPIOA->BSRR = 1<<5;`——最快、最透明，代码短，但可读性差、可移植性差；
2. **LL（Low-Layer）库**：对寄存器的薄封装，`LL_GPIO_SetOutputPin(GPIOA, LL_GPIO_PIN_5)`；
3. **HAL 库**：`HAL_GPIO_WritePin(...)`，带句柄、状态、超时、回调，易用、通用但较重，且某些 API 带阻塞（如 `HAL_UART_Transmit` 超时等待）。

### 9.2 手算：HAL 与寄存器的代价差异

教学示例：翻转一个 GPIO。寄存器写 BSRR：1 次存储指令，约 2–3 个周期。`HAL_GPIO_WritePin` 经过：参数检查（`assert_param` 在 release 下关闭）、分支判断、写 BSRR，大约 10–20 个周期。168 MHz 下 15 周期 = 89 ns。1 kHz 控制任务里调用几次，完全无所谓；但如果你在 1 MHz 位翻转（软件模拟时序，如 WS2812 灯带）就不能用 HAL，需要直接写寄存器或用定时器 + DMA。

### 9.3 HAL 的三套收发 API（重要）

以 UART 为例：

| API | 行为 | 是否可在 ISR | 场景 |
|---|---|---|---|
| `HAL_UART_Transmit` | **阻塞**轮询，到超时/发完才返回 | 不应 | 调试打印 |
| `HAL_UART_Transmit_IT` | 启动后立即返回，发完触发回调 | 可以 | 短包 |
| `HAL_UART_Transmit_DMA` | 配置 DMA，立即返回，完成回调 | 可以 | 长包、高频 |

接收对应 `Receive`、`Receive_IT`、`Receive_DMA`，以及 `HAL_UARTEx_ReceiveToIdle_DMA`（DMA + 空闲中断一次搞定不定长，详见 [UART 章](./04_UART_SPI_I2C_以太网_TCP.md)）。

### 9.4 在项目里怎么用

典型 CubeMX + HAL + FreeRTOS 工程：外设初始化由 CubeMX 生成；业务代码写在 FreeRTOS 任务和回调里。读别人的代码时，从回调函数（如 `HAL_CAN_RxFifo0MsgPendingCallback`）找数据入口，从任务函数找控制周期。**读源码的顺序**：中断/回调（数据怎么进来）→ 任务（谁算）→ 发送（命令怎么出去）。这是 P3、P7 控制链共同的结构。

---

## 10. 常见错误 / 边界 / 故障现象（汇总）

| 现象 | 常见原因 |
|---|---|
| 程序下载后不运行 / 只跑一次 | 晶振没起振、`HSE_VALUE` 不对、Flash 等待周期不够 |
| 串口乱码 | 时钟或波特率不匹配；APB 分频改变 |
| PWM 频率差 2 倍 | 忘了定时器 ×2 规则 |
| 进了 HardFault | 空指针；栈溢出；未对齐访问；调用了未使能的 FPU；ISR 优先级配错调了 RTOS API |
| H7 上 DMA 数据"偶尔旧" | Cache 一致性 |
| H7 上 DMA 完全没数据 | 缓冲区在 DTCM |
| 中断一直进 | 没清标志 |
| 控制周期抖动大 | 高频中断过多；ISR 太长；总线被 DMA 占用 |
| 编码器速度有"台阶" | 采样窗太短（量化）；没滤波 |

---

## 11. 面试怎么问、怎么答

**Q1. What happens when an interrupt fires on a Cortex-M?**

> When the interrupt is pending and its priority is higher than what the CPU is running, the hardware automatically pushes eight registers onto the stack, loads the handler address from the vector table, and enters handler mode. My ISR is just a normal C function. On return, a special value in the link register tells the hardware to pop the registers and resume. I keep ISRs short: I clear the flag, hand data to a task through a queue or notification, and return.

中文要点：硬件自动压 8 个寄存器、查向量表、进 Handler 模式；返回时 EXC_RETURN 触发弹栈。ISR 短，清标志，数据交给任务。追问方向：嵌套？（高抢占优先级可打断低的）；ISR 里能阻塞吗？（不能）；FreeRTOS 里 ISR 优先级限制？（见 6.4）。

**Q2. Why did you use DMA, and what problems come with it?**

> DMA moves data between peripherals and memory without the CPU, so the control task is not interrupted per byte. For example, with a UART or SPI sensor I can receive into a circular buffer and be notified only at half or full transfer, or on an idle line. The problems are buffer ownership and, on the H7, cache coherency: the CPU might read stale cache lines unless the buffer is non-cacheable or I invalidate it, and the buffer must not sit in memory that the DMA cannot reach, like DTCM.

中文：DMA 免去逐字节中断；风险是缓冲归属与 H7 Cache 一致性、DTCM 不可达。在 P3/P7 中，DMA 主要用于串口接收（遥控器、上位机、裁判系统数据）与 IMU 的 SPI 读取，这两类都是“固定帧长、周期性到来”的数据，最适合 DMA 加空闲中断。

**Q3. What is the difference between STM32F4 and H7 that mattered to you?**

> The H7 is much faster but has caches, tightly coupled memories and several bus domains, so memory placement matters. On the F4 any RAM is just RAM for DMA. On the H7 I need to think about which SRAM a buffer is in, whether it is cacheable, and alignment. The H7 also has FDCAN instead of the classic bxCAN, with its own message RAM for filters and FIFOs.

**Q4. How would you generate a 1 kHz PWM with 25% duty on a 84 MHz timer?**

> I pick a prescaler of 83 to get a 1 MHz counter, set auto-reload to 999 for a 1 kHz period, and set the compare register to 250 for 25%. The resolution is one part in a thousand.

追问：想要更高分辨率怎么办？（降低 PSC 或接受更高频，PWM 频率与分辨率折中）。

---

## 本章要点速记

1. Cortex-M：R13 有 MSP/PSP 两个；异常进入硬件自动压 8 个寄存器，ISR 可用普通 C 函数；
2. 复位后：取 MSP、取 Reset_Handler → 复制 .data、清 .bss、SystemInit、构造全局对象 → main；
3. 时钟树：$f_{SYS}$ = $f_{HSE}/M \times N / P$；APB 分频≠1 时，定时器时钟 = 2×PCLK；
4. PWM：$f=\dfrac{f_{tim}}{(PSC+1)(ARR+1)}$，$D=CCR/(ARR+1)$；
5. 编码器模式 4 倍频，16 位计数用 `(int16_t)(now-last)` 抗回绕；
6. NVIC：数值越小越紧急；FreeRTOS 要求分组 4，调用 `FromISR` 的中断优先级数值必须 ≥ `configMAX_SYSCALL_INTERRUPT_PRIORITY`；
7. DMA：外设↔内存自动搬运，循环模式 + HT/TC 乒乓；已收字节数 = 缓冲大小 − NDTR；
8. H7 三大坑：Cache 一致性、DTCM DMA 不可达、外设内核时钟单独选；
9. HAL 阻塞 API 只用于调试，运行路径用 IT/DMA；
10. 读源码顺序：回调 → 任务 → 发送。

## 自测题

1. HSE = 25 MHz，目标 SYSCLK = 168 MHz，选 M = 25 时，N、P 取多少？（要点：VCO_in = 1 MHz，N = 336，P = 2。）
2. 定时器时钟 168 MHz，要 20 kHz PWM，PSC = 0，ARR 是多少？分辨率约几位？（要点：ARR+1 = 8400，ARR = 8399，约 13 位。）
3. 编码器 1024 线，4 倍频，1 ms 内 delta = −17，求角速度（rad/s）。（要点：4096 计数/转；−17/4096 转/ms = −4.15 转/s = −26.1 rad/s，注意负号为反向。）
4. 为什么 FreeRTOS 要求 NVIC 优先级分组为 4？（要点：避免子优先级使抢占语义混乱；内核用 BASEPRI 按整体数值比较优先级。）
5. UART DMA 循环接收缓冲 512 字节，上次 read_pos = 500，现在 NDTR = 490，有多少新字节？（要点：write_pos = 22，回绕，新数据 = 12 + 22 = 34 字节。）
6. H7 上 DMA 接收完数据，读出来的是旧值，列出两种修复方法。（要点：Invalidate Cache 且对齐 32；MPU 设为 non-cacheable。）

## 相关链接

- 下一章：[FreeRTOS 实时系统](./02_FreeRTOS实时系统.md)
- 相关：[CAN / FDCAN 与电机通信](./03_CAN_FDCAN与电机通信.md)、[UART / SPI / I2C / 以太网 / TCP](./04_UART_SPI_I2C_以太网_TCP.md)、[电机、编码器、驱动与力矩控制](./05_电机_编码器_驱动与力矩控制.md)
- 八股：[C语言与嵌入式八股](../../03_八股知识/01_计算机基础/04_C语言与嵌入式八股.md)、[嵌入式与机电八股](../../03_八股知识/03_机器人学/06_嵌入式与机电八股.md)
- 项目：[P3](../../01_项目课程/P3_RoboMaster英雄机器人嵌入式控制/README.md)、[P7](../../01_项目课程/P7_可变腿长轮腿机器人/README.md)
