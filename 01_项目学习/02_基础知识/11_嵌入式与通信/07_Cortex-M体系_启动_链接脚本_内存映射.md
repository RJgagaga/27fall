# Cortex-M 体系：启动流程、向量表、链接脚本、内存映射与 HardFault 排查

> **一句话定位**：上电之后第一条指令从哪里取、全局变量是怎样"变成"RAM 里的值的、栈和堆各占多少、程序跑飞时怎样从寄存器里读出"凶手"——这一章把 Cortex-M 从复位到 `main()` 的全过程、链接脚本与内存映射、FPU/MPU 以及 HardFault 排查方法一次讲透，是 P3（STM32F407）和 P7（STM32H7）固件"为什么会偶发崩溃"的底层依据。

**本章学完你能：**

- 说出复位后硬件做的两件事（读栈顶、读复位向量），并画出向量表前 16 项；
- 讲清 `Reset_Handler` 里 `.data` 拷贝、`.bss` 清零、`SystemInit`、`__libc_init_array`、`main` 的顺序及每一步的必要性；
- 读懂一份最小链接脚本，区分 LMA 与 VMA，解释"为什么初始化过的全局变量要在 Flash 里存一份、RAM 里再存一份"；
- 根据 `arm-none-eabi-size` 的输出手算 Flash 与 RAM 占用，并估算栈需求；
- 说出 STM32F407 与 STM32H7 在内存映射上的关键差异（CCM、TCM、Cache、DMA 可达性）；
- 解释 FPU 启用、惰性压栈、MPU 区域的作用；
- 面对 HardFault，按 CFSR/HFSR/MMFAR/BFAR 与压栈寄存器的顺序定位出错指令和地址；
- 用英文讲清"what happens between reset and main"。

**前置知识：**[STM32 时钟、GPIO、中断、DMA](./01_STM32_时钟_GPIO_中断_DMA.md)（尤其寄存器、双栈指针、异常压栈、内存映射初步）；[FreeRTOS 实时系统](./02_FreeRTOS实时系统.md)（任务栈与堆）；C 语言指针与存储类别。

**相关项目：**[P3 FreeRTOS 任务设计与时序](../../01_项目课程/P3_RoboMaster英雄机器人嵌入式控制/03_FreeRTOS任务设计与时序.md)（F407）、[P7 STM32H7/FreeRTOS/FDCAN 软件结构](../../01_项目课程/P7_可变腿长轮腿机器人/10_STM32H7_FreeRTOS_FDCAN软件结构.md)（H7，含 Cache 与不可缓存缓冲段）。

## 目录

1. 这一章要解决什么问题
2. 符号、寄存器与单位
3. 复位到 main：启动流程逐步拆解
4. 向量表与异常模型
5. 链接脚本：段、LMA 与 VMA
6. 内存映射：F407 与 H7 的差异
7. FPU、惰性压栈与 MPU
8. 完整手算：Flash/RAM 占用与栈预算
9. HardFault 排查：从寄存器到出错行
10. 在七个项目里怎么用
11. 常见错误与故障现象
12. 英文面试问答
13. 要点速记、自测题、相关链接

---

## 1. 这一章要解决什么问题

### 1.1 一个典型的"诡异故障"

你的固件在桌上跑得好好的，装到机器人上，运行十几分钟后某次急转弯时突然所有电机断电，调试器连上去发现停在 `HardFault_Handler` 的死循环里。此时你手里只有三条线索：停在哪个函数（往往是死循环本身，没有信息）、寄存器里的一堆十六进制数、以及"偶发"这个事实。要把这些线索变成"某个任务栈溢出踩坏了相邻任务的 TCB"或者"DMA 缓冲被 Cache 吞了一半"这样的结论，你必须知道：

1. **程序在内存里是怎么摆的**（代码、常量、全局变量、堆、每个任务的栈各在哪）；
2. **CPU 出错时硬件替你记下了什么**（压栈寄存器、故障状态寄存器）；
3. **启动阶段到底做了什么**（否则无法解释"全局变量初值不对"或"浮点运算结果乱跳"）。

这一章就回答这三件事。它不是新算法，而是所有算法能"稳定跑"的地基。

### 1.2 为什么控制工程师必须懂

- 控制周期里每多一次不必要的内存拷贝，就多一份延迟（参见 [数字控制与实时性](../05_控制理论/08_数字控制与实时性.md)）；
- 任务栈开小了，平时没事，调试时一加 `printf` 就崩——这是典型的"周期性偶发"，也是面试官最爱问的故障类型；
- H7 的 Cache 与 DMA 一致性问题，会表现为"传感器数据偶尔是旧的"，从控制上看像噪声，其实是软件结构问题。

---

## 2. 符号、寄存器与单位

| 符号 | 含义 | 说明 |
|---|---|---|
| MSP / PSP | 主栈指针 / 进程栈指针 | 复位后用 MSP；RTOS 任务用 PSP，中断与内核用 MSP |
| LR（R14） | 链接寄存器 | 函数调用保存返回地址；异常时装入 EXC_RETURN |
| PC（R15） | 程序计数器 | 低位 bit0 在跳转时表示 Thumb 状态，Cortex-M 只支持 Thumb，故必须为 1 |
| xPSR | 程序状态寄存器 | 含 IPSR（当前异常号） |
| VTOR | 向量表偏移寄存器，地址 0xE000ED08 | 决定向量表放在哪 |
| EXC_RETURN | 异常返回魔数 | 0xFFFFFFF1 / F9 / FD 等，见第 4 节 |
| LMA / VMA | 加载地址 / 运行地址 | 数据"存放"的位置与"使用"的位置，可以不同 |
| KB、KiB | 本章全部按 1 KB = 1024 B | 与芯片手册的写法一致 |

地址一律写十六进制，如 `0x0800_0000`（下划线仅为易读）。字（word）= 4 字节。

---

## 3. 复位到 main：启动流程逐步拆解

### 3.1 硬件做的两件事

Cortex-M 复位后，**硬件**（不是软件）做两个读操作，地址来自向量表起始处：

1. 从向量表第 0 个字读出 **初始 MSP**，装入 SP；
2. 从向量表第 1 个字读出 **复位向量**（`Reset_Handler` 的地址，bit0 为 1），装入 PC，开始执行。

在 STM32 上，芯片把 Flash 起始地址 `0x0800_0000` 在启动时映射（别名）到 `0x0000_0000`（取决于 BOOT 引脚），所以"向量表在地址 0"的 ARM 约定与"代码在 0x0800_0000"并不矛盾。

> **面试要点**：Cortex-M 与传统 ARM（如 ARM7/9）不同：向量表里放的是**地址**而不是**跳转指令**；第 0 项是**栈顶**而不是复位指令。这样 C 语言写的 `Reset_Handler` 一上来就有可用的栈。

### 3.2 `Reset_Handler` 做什么

厂家提供的启动文件（`startup_stm32f407xx.s` 之类，汇编或 C）典型流程如下，伪代码：

```c
void Reset_Handler(void)
{
    /* 1. 把 .data 的初值从 Flash 拷到 RAM */
    uint32_t *src = &_sidata;           /* 初值在 Flash 里的位置（LMA） */
    uint32_t *dst = &_sdata;            /* .data 在 RAM 里的起点（VMA） */
    while (dst < &_edata) { *dst++ = *src++; }

    /* 2. 把 .bss 清零 */
    for (dst = &_sbss; dst < &_ebss; ) { *dst++ = 0; }

    /* 3. 系统初始化：时钟前的基础配置、FPU 使能、向量表偏移 */
    SystemInit();

    /* 4. C 库初始化：调用全局对象构造函数、init_array */
    __libc_init_array();

    /* 5. 进入用户代码 */
    main();

    for (;;) { }   /* main 不应返回 */
}
```

逐步说明"为什么"：

- **`.data` 拷贝**：`int g = 5;` 这样的已初始化全局变量，运行时在 RAM 里（要能改），但掉电后 RAM 内容消失，所以初值必须存在 Flash 里。链接器把初值放进 Flash（LMA），启动代码在 `main` 之前拷到 RAM（VMA）。忘了拷贝，所有已初始化全局变量都是随机值。
- **`.bss` 清零**：未初始化或初值为 0 的全局/静态变量（C 标准保证初值为 0）放 `.bss`，不占 Flash，只在启动时清零。很多"上电后偶尔状态不对"来自 `.bss` 被误放到不会被清零的段（如 `.noinit`）。
- **`SystemInit()`**：在 F4 上典型内容是：使能 FPU（`SCB->CPACR |= (0xF << 20)`），把时钟复位到默认状态，设置 `SCB->VTOR`（如向量表重定位到 Flash 其他位置或 RAM）。**FPU 要在任何浮点指令之前使能**，否则第一条浮点指令就触发 UsageFault（NOCP 位）。
- **`__libc_init_array`**：C++ 全局对象的构造函数、`__attribute__((constructor))` 函数在这里被调用。因此 C++ 全局对象构造里**不能**依赖尚未初始化的外设。
- **进入 `main`**：此时时钟还没配到 168 MHz（HAL 的 `SystemClock_Config` 在 `main` 里做），所以 `main` 开头的外设初始化顺序有讲究。

### 3.3 时钟与 SysTick 的位置

`Reset_Handler` 之后、`main` 里 `HAL_Init()` 配置 SysTick（1 ms 节拍），再配 PLL。FreeRTOS 启动后接管 SysTick 作为 RTOS tick，HAL 的时基常改用另一个定时器（如 TIM6/TIM7），否则两者争用 SysTick 中断优先级会引起延时异常——这是很常见的移植问题，详见 [FreeRTOS 实时系统](./02_FreeRTOS实时系统.md)。

### 3.4 Bootloader 场景的启动

如果有 Bootloader（见 [电源管理、看门狗、Bootloader 与 OTA](./08_电源管理_低功耗_看门狗_Bootloader_OTA.md)），应用程序的向量表不在 Flash 起始，而在某个偏移处，这时必须设置 `VTOR` 指向应用的向量表，并且在跳转前把 MSP 设成应用向量表第 0 项。这一点放到第 8 章讲，这里只需记住：**向量表位置由 VTOR 决定，不是写死的**。

---

## 4. 向量表与异常模型

### 4.1 向量表布局（前 16 项）

每项 4 字节，偏移 = 序号 × 4：

| 序号 | 偏移 | 内容 |
|---|---|---|
| 0 | 0x00 | 初始 MSP |
| 1 | 0x04 | Reset |
| 2 | 0x08 | NMI |
| 3 | 0x0C | HardFault |
| 4 | 0x10 | MemManage（需使能） |
| 5 | 0x14 | BusFault（需使能） |
| 6 | 0x18 | UsageFault（需使能） |
| 7–10 | 0x1C–0x28 | 保留 |
| 11 | 0x2C | SVCall |
| 12 | 0x30 | DebugMonitor |
| 13 | 0x34 | 保留 |
| 14 | 0x38 | PendSV |
| 15 | 0x3C | SysTick |
| 16 起 | 0x40 起 | 外部中断 IRQ0、IRQ1…（芯片相关） |

默认情况下 MemManage、BusFault、UsageFault 没有使能，出错会**升级为 HardFault**。要得到更精确的故障信息，应在启动时使能：`SCB->SHCSR |= SCB_SHCSR_USGFAULTENA_Msk | SCB_SHCSR_BUSFAULTENA_Msk | SCB_SHCSR_MEMFAULTENA_Msk;`。故障状态寄存器 CFSR 无论是否使能都会更新，所以排查时总能读到。

向量表大小的要求：必须按"2 的幂且不小于表大小"对齐（F4 上因 IRQ 数量，常见 512 字节对齐，具体见芯片手册，未逐型号核对）。重定位向量表时若未对齐，VTOR 低位会被屏蔽，行为不可预期。

### 4.2 异常入口：硬件压栈与 EXC_RETURN

异常发生时硬件自动将 8 个寄存器压入**当前使用的栈**（线程模式下可能是 PSP）：

```text
高地址
  +--------+
  | xPSR   |  <- SP+28
  | PC     |  <- SP+24   出错指令或返回地址
  | LR     |  <- SP+20
  | R12    |  <- SP+16
  | R3     |  <- SP+12
  | R2     |  <- SP+8
  | R1     |  <- SP+4
  | R0     |  <- SP+0   (异常发生时的栈顶)
  +--------+
低地址
```

随后 LR 被装入 **EXC_RETURN**，其含义（M4/M7，带 FPU）：

| EXC_RETURN | 返回到 | 使用栈 | 压了浮点上下文 |
|---|---|---|---|
| 0xFFFFFFF1 | Handler 模式（嵌套异常） | MSP | 否 |
| 0xFFFFFFF9 | Thread 模式 | MSP | 否 |
| 0xFFFFFFFD | Thread 模式 | PSP | 否 |
| 0xFFFFFFE1 | Handler 模式 | MSP | 是 |
| 0xFFFFFFE9 | Thread 模式 | MSP | 是 |
| 0xFFFFFFED | Thread 模式 | PSP | 是 |

**bit 2** 表示返回后用哪个栈：1 = PSP，0 = MSP。这正是后面 HardFault 排查时"判断从 MSP 还是 PSP 取压栈寄存器"的依据：`tst lr, #4`。**bit 4** 为 0 表示压了浮点上下文（扩展帧 26 字 = 104 字节）。

### 4.3 延迟与咬尾

中断响应的典型延迟为 12 个周期（零等待状态下的 M3/M4），M7 在 Flash 有等待状态与 Cache 命中情况下会更复杂，但量级相近。**咬尾（tail-chaining）**：一个 ISR 结束时若有挂起的异常，可跳过出栈入栈，直接进入下一个 ISR，约 6 个周期。**迟到（late arrival）**：压栈期间来了更高优先级异常，直接去处理更高者。这些使 Cortex-M 的中断延迟既小又可预测，对实时控制很重要。

### 4.4 优先级与 BASEPRI

NVIC 优先级**数值越小越高**。STM32 只实现了高 4 位（`__NVIC_PRIO_BITS = 4`，16 级）。FreeRTOS 用 `BASEPRI` 屏蔽数值不小于 `configMAX_SYSCALL_INTERRUPT_PRIORITY` 的中断；比它更高（数值更小）的中断不受内核屏蔽，但也**不能**调用 `FromISR` API。具体写法见 [STM32 一章](./01_STM32_时钟_GPIO_中断_DMA.md)。

---

## 5. 链接脚本：段、LMA 与 VMA

### 5.1 常见段

| 段 | 内容 | 位置 |
|---|---|---|
| `.isr_vector` | 向量表 | Flash 起始，必须在最前 |
| `.text` | 代码 | Flash |
| `.rodata` | 常量、字符串字面量、`const` 全局 | Flash |
| `.data` | 已初始化的全局/静态变量 | VMA 在 RAM，LMA 在 Flash |
| `.bss` | 未初始化/零初值全局变量 | RAM（不占 Flash） |
| `._user_heap_stack` | 堆与（主）栈的预留区 | RAM，用来在链接期检查是否放得下 |
| 栈 | MSP 初始值指向 RAM 末尾 | RAM 顶端向下生长 |

### 5.2 一份最小链接脚本

下面是**教学示例**（接近 STM32CubeMX 生成的结构，数值简化）：

```text
MEMORY
{
  FLASH (rx)  : ORIGIN = 0x08000000, LENGTH = 1024K
  RAM   (xrw) : ORIGIN = 0x20000000, LENGTH = 112K
  CCMRAM (xrw): ORIGIN = 0x10000000, LENGTH = 64K
}

_estack = ORIGIN(RAM) + LENGTH(RAM);   /* 栈顶 = RAM 末尾 */
_Min_Heap_Size  = 0x200;
_Min_Stack_Size = 0x400;

SECTIONS
{
  .isr_vector : { KEEP(*(.isr_vector)) } >FLASH
  .text       : { *(.text*) *(.rodata*) } >FLASH

  _sidata = LOADADDR(.data);             /* .data 初值在 Flash 里的地址 */
  .data : { _sdata = .; *(.data*) _edata = .; } >RAM AT> FLASH

  .bss  : { _sbss = .; *(.bss*) *(COMMON) _ebss = .; } >RAM

  ._user_heap_stack :
  {
    . = ALIGN(8);
    . = . + _Min_Heap_Size;
    . = . + _Min_Stack_Size;
    . = ALIGN(8);
  } >RAM
}
```

**关键字**：`>RAM AT> FLASH` 的意思是"运行地址（VMA）在 RAM，加载地址（LMA）在 FLASH"。`LOADADDR(.data)` 返回 LMA，也就是启动代码拷贝的源地址 `_sidata`。`_sdata/_edata` 是 VMA 的起止，也就是目的地址。

> **注意**：如果只写 `>FLASH` 而不写 `AT>`，变量的运行地址会被放在 Flash（只读）里，写入时 HardFault 或静默失败。

### 5.3 把关键数据放到指定区域

要把某数组放到 CCM（F4）或不可缓存 RAM（H7），做法是在 C 里用属性，在链接脚本里加段：

```c
__attribute__((section(".ccmram"))) float big_table[4096];
__attribute__((section(".dma_buf"), aligned(32))) uint8_t imu_rx[32];
```

```text
.ccmram : { *(.ccmram*) } >CCMRAM
```

F4 的 CCM **不能被 DMA 访问**（参见 [STM32 一章](./01_STM32_时钟_GPIO_中断_DMA.md) 中的内存表），所以放 CCM 的只能是 CPU 专用数据，如控制任务栈、大查表数组。把 SPI/UART/CAN 的 DMA 缓冲放进 CCM，症状是 DMA 传输"成功"但数据全是 0 或不变。

### 5.4 map 文件

链接时加 `-Wl,-Map=out.map` 得到 map 文件，里面按符号给出地址与大小。排查"RAM 突然不够用"时，按 `.bss` 里符号大小排序，往往一眼找到一个忘了 `static` 的大数组。`arm-none-eabi-size out.elf` 输出 `text data bss dec`：

- `text` = 代码 + 只读数据（含 `.rodata`）；
- `data` = `.data` 大小；
- `bss` = `.bss` 大小；
- **Flash 占用 = text + data**（因为 `.data` 的初值在 Flash 里），**静态 RAM 占用 = data + bss**。

---

## 6. 内存映射：F407 与 H7 的差异

### 6.1 STM32F407（P3 使用）

| 区域 | 起始地址 | 大小 | 说明 |
|---|---|---|---|
| Flash | 0x0800_0000 | 1 MB（IG 型号） | 带 ART 加速器、预取，等待状态随主频升高 |
| SRAM1+SRAM2 | 0x2000_0000 | 112 KB + 16 KB | 通用，DMA 可访问 |
| CCM SRAM | 0x1000_0000 | 64 KB | 紧耦合，仅 CPU 访问，零等待；DMA 不可达 |
| 外设 | 0x4000_0000 起 | — | APB1/APB2/AHB |
| 系统存储器 | 0x1FFF_0000 | — | 内置 Bootloader（ROM） |

F407 **没有 Cache**，所以没有 DMA 一致性问题；但有 CCM 的"DMA 不可达"陷阱。Flash 等待状态：168 MHz、3.3 V 下通常需要 5 个等待状态（具体见手册，未核对），ART 预取与指令 Cache 缓解它。

### 6.2 STM32H7（P7 使用）

H7 内存模块多，且以 H743 为例（具体型号以数据手册为准，本章未逐型号核对）：

| 区域 | 起始地址 | 大小 | DMA 可达 |
|---|---|---|---|
| ITCM | 0x0000_0000 | 64 KB | 仅 CPU（和 MDMA） |
| DTCM | 0x2000_0000 | 128 KB | 仅 CPU（和 MDMA），DMA1/DMA2 不可达 |
| AXI SRAM | 0x2400_0000 | 512 KB | 是 |
| SRAM1/2/3 | 0x3000_0000 起 | 数十至一百多 KB | 是 |
| SRAM4 | 0x3800_0000 | 64 KB | 是（属 D3 域） |
| Flash | 0x0800_0000 | 最大 2 MB | — |

M7 内核有 **L1 指令 Cache 和数据 Cache（各 16 KB）**。CPU 通过 D-Cache 访问 AXI SRAM，而 DMA 直接写 RAM，不经过 Cache——于是出现**一致性问题**：DMA 把新数据写进 RAM，CPU 读时命中了 Cache 里的旧数据；或 CPU 写了 Cache 还没回写，DMA 读到旧数据。

三种处理方法：

1. 缓冲放进**不可缓存**区域（用 MPU 把某块 RAM 配置为 Non-cacheable 或 Device），最简单，代价是 CPU 访问变慢；
2. 接收后 `SCB_InvalidateDCache_by_Addr(buf, len)`，发送前 `SCB_CleanDCache_by_Addr(buf, len)`，地址与长度必须按 **32 字节（Cache 行）** 对齐，否则相邻变量被误失效；
3. 把缓冲放在 DMA 可达但本来就不被 Cache 的存储（如 SRAM4 配 MPU，或者 TCM，注意 DMA 可达性）。

P7 采用第 1 种，见 [P7 软件结构](../../01_项目课程/P7_可变腿长轮腿机器人/10_STM32H7_FreeRTOS_FDCAN软件结构.md)（课程中该做法的描述为：IMU 与 CAN 缓冲放进专用的不可缓存段）。

### 6.3 一个重要的区别

| 项目 | F407 | H7 |
|---|---|---|
| 主频量级 | 168 MHz | 数百 MHz（以实际配置为准） |
| Cache | 无 | L1 I/D Cache |
| DMA 一致性问题 | 无 | 有 |
| 专用快速 RAM | CCM（DMA 不可达） | ITCM/DTCM（DMA1/2 不可达） |
| FPU | 单精度 | 单精度+双精度（H743） |
| MPU | 8 区域 | 16 区域（M7，视型号） |

---

## 7. FPU、惰性压栈与 MPU

### 7.1 FPU

Cortex-M4F/M7 带 FPU（单精度，M7 部分型号带双精度）。使能方法：`SCB->CPACR |= (3UL << 20) | (3UL << 22);`（CP10、CP11 设为完全访问），随后用 `__DSB(); __ISB();`。编译选项需 `-mfpu=fpv4-sp-d16 -mfloat-abi=hard`（F4），H7 双精度可用 `fpv5-d16`。所有链接进来的库必须是同一 ABI，否则链接报错或运行时出现浮点参数乱码。

写 `1.0f` 与 `1.0` 的区别：后者是 double，在只有单精度 FPU 的芯片上会调用软件库，慢数十倍。控制代码里 `sin()` 与 `sinf()` 同理。这是很常见的性能陷阱，也是面试题。

### 7.2 惰性压栈（lazy stacking）

默认配置 `FPCCR.LSPEN = 1`：异常发生时若任务用过浮点，硬件**预留**浮点寄存器空间但不立即保存 S0–S15，只有当 ISR 里真的执行了浮点指令才补存。好处是纯整数 ISR 的入口依然快（12 周期）；代价是栈上要多预留 18 个字，使每个任务栈、MSP 的需求从 32 字节的最小帧变成 104 字节。**计算栈预算时必须按 104 字节算**。

FreeRTOS 的 `ARM_CM4F` 移植会在任务切换时手动保存 S16–S31，并依赖这一机制保存 S0–S15，所以 FPU 与 RTOS 的配合是已解决的，但要确保 `configENABLE_FPU` 或相应移植文件选对。

### 7.3 MPU（内存保护单元）

MPU 把地址空间划分为若干区域（M4：8 个；M7：8 或 16 个），每个区域有基址、大小（2 的幂）、访问权限、可缓存属性。典型用途：

1. **栈溢出保护**：在每个任务栈底设一个只读小区域，溢出立即触发 MemManage，而不是静默踩坏邻居（FreeRTOS-MPU 移植支持）；
2. **空指针保护**：把地址 0 附近区域设为不可访问；
3. **H7 的缓存属性控制**：如上节，把 DMA 缓冲所在区域设为 non-cacheable。

MPU 不是必须，但对"偶发崩溃"的可调试性大有帮助。启用后出错会在 MemManage 里给出精确的出错地址（MMFAR）。

---

## 8. 完整手算：Flash/RAM 占用与栈预算

以下数值均为**教学示例（非项目实测）**，目的是演示算法。

### 8.1 占用计算

某固件 `arm-none-eabi-size` 输出：`text = 48200`，`data = 1100`，`bss = 21300`（字节）。

- Flash 占用 = 48200 + 1100 = 49300 B ≈ 48.1 KiB，占 1 MiB Flash 的 4.7%；
- 静态 RAM 占用 = 1100 + 21300 = 22400 B ≈ 21.9 KiB；
- 112 KiB 的 SRAM 减去静态占用，剩 114688 − 22400 = 92288 B；
- 从剩余里再扣：FreeRTOS 堆（`configTOTAL_HEAP_SIZE`，它本身在 `.bss` 里，所以已经计入 bss，不要重复扣！）。

> **易错**：若 FreeRTOS 采用静态数组作堆（heap_4 的 `ucHeap[configTOTAL_HEAP_SIZE]`），它位于 `.bss`，`size` 的 bss 已经包含它。有人把它再减一次，就会误以为 RAM 不够。

### 8.2 栈预算

问题：MSP（中断与启动用）需要多大？采用"最坏嵌套"估算。

- 主函数（调度器启动前）调用深度：`main` → `HAL_Init` → …，最坏局部变量与帧合计取 200 B；调度器启动后这部分栈内容不再使用，MSP 被重置为向量表初值（FreeRTOS 在 `vPortStartFirstTask` 里重置 MSP），所以运行期 MSP 只服务中断。
- 低优先级中断 A（如 USB）的帧：扩展帧 104 B + ISR 局部/调用链 120 B = 224 B；
- 被更高优先级中断 B（如 CAN 接收）打断：104 B + 80 B = 184 B；
- 运行期 MSP 最坏 = 224 + 184 = 408 B（再加 SysTick、PendSV 等同优先级的不会嵌套，因为它们优先级最低）。

留 100% 余量即 816 B，取 1 KiB。这与链接脚本里的 `_Min_Stack_Size = 0x400`（1024 B）吻合。

**任务栈**：FreeRTOS 的栈大小单位是**字**（4 字节）。`xTaskCreate(..., 1024, ...)` 是 4096 字节，不是 1024 字节。P7 的控制任务栈 4 KB（见 [P7 软件结构](../../01_项目课程/P7_可变腿长轮腿机器人/10_STM32H7_FreeRTOS_FDCAN软件结构.md) 的设计取值）= 1024 字。用 `uxTaskGetStackHighWaterMark()`（返回的是**历史最小剩余字数**）验证。

### 8.3 向量表重定位的数值例子

把应用放在偏移 `0x0001_0000`（64 KiB）处，则：

- 应用向量表地址 = 0x0800_0000 + 0x0001_0000 = 0x0801_0000；
- `VTOR = 0x08010000`；
- 栈顶 = `*(uint32_t*)0x08010000`，复位向量 = `*(uint32_t*)0x08010004`；
- 若 Bootloader 要跳转：`__set_MSP(*(uint32_t*)0x08010000); ((void(*)(void))*(uint32_t*)0x08010004)();`。

F4 的 Flash 扇区大小不均匀（16 KB×4、64 KB×1、128 KB×7，1 MB 型号），应用起点要对齐到扇区边界；64 KiB 位置恰好是第 5 个扇区（64 KB 扇区）的起点（4×16 KB = 64 KB），这是选 `0x0001_0000` 的原因。

可以用下面的 Python 复核上述算术：

```python
text, data, bss = 48200, 1100, 21300
print("flash", text + data, "ram", data + bss)           # 49300 22400
print("flash pct", (text + data) / 1048576 * 100)         # 4.70
print("msp worst", (104 + 120) + (104 + 80))              # 408
print(hex(0x08000000 + 0x00010000))                      # 0x8010000
```

---

## 9. HardFault 排查：从寄存器到出错行

### 9.1 为什么先看 CFSR

故障状态寄存器：

| 寄存器 | 地址 | 内容 |
|---|---|---|
| CFSR | 0xE000ED28 | 可配置故障状态：MMFSR（位 0–7）、BFSR（位 8–15）、UFSR（位 16–25） |
| HFSR | 0xE000ED2C | HardFault 状态：FORCED（位 30）表示由其他故障升级而来 |
| MMFAR | 0xE000ED34 | MemManage 出错地址（MMARVALID 为 1 时有效） |
| BFAR | 0xE000ED38 | BusFault 出错地址（BFARVALID 为 1 时有效） |

CFSR 常用位：

| 位 | 名称 | 含义 |
|---|---|---|
| 0 | IACCVIOL | 取指违规 |
| 1 | DACCVIOL | 数据访问违规 |
| 7 | MMARVALID | MMFAR 有效 |
| 8 | IBUSERR | 取指总线错误 |
| 9 | PRECISERR | 精确数据总线错误（BFAR 有效，PC 指向出错指令） |
| 10 | IMPRECISERR | 不精确数据总线错误（写缓冲导致，PC 不准） |
| 11 / 12 | UNSTKERR / STKERR | 出栈 / 压栈时总线错误（常是栈指针已坏） |
| 15 | BFARVALID | BFAR 有效 |
| 16 | UNDEFINSTR | 未定义指令（常见于跑飞到数据区） |
| 17 | INVSTATE | 无效状态（多半是函数指针 bit0 为 0，试图切 ARM 状态） |
| 18 | INVPC | 非法 EXC_RETURN |
| 19 | NOCP | 无协处理器（FPU 未使能） |
| 24 | UNALIGNED | 非对齐访问（使能陷阱时） |
| 25 | DIVBYZERO | 除零（使能陷阱时；整数除零） |

### 9.2 取出压栈寄存器

HardFault 入口处，根据 EXC_RETURN 的 bit 2 判断用的是哪个栈，取出压栈的 PC、LR。常见写法（教学示例）：

```c
__attribute__((naked)) void HardFault_Handler(void)
{
    __asm volatile(
        "tst lr, #4        \n"   /* EXC_RETURN bit2: 1 -> PSP */
        "ite eq            \n"
        "mrseq r0, msp     \n"
        "mrsne r0, psp     \n"
        "b hard_fault_c    \n"
    );
}

void hard_fault_c(uint32_t *sp)
{
    volatile uint32_t r0  = sp[0],  r1 = sp[1], r2 = sp[2], r3 = sp[3];
    volatile uint32_t r12 = sp[4],  lr = sp[5], pc = sp[6], psr = sp[7];
    volatile uint32_t cfsr = *(volatile uint32_t*)0xE000ED28;
    volatile uint32_t hfsr = *(volatile uint32_t*)0xE000ED2C;
    volatile uint32_t mmfar = *(volatile uint32_t*)0xE000ED34;
    volatile uint32_t bfar  = *(volatile uint32_t*)0xE000ED38;
    (void)r0;(void)r1;(void)r2;(void)r3;(void)r12;(void)lr;(void)pc;(void)psr;
    (void)cfsr;(void)hfsr;(void)mmfar;(void)bfar;
    for (;;) { }     /* 在这里打断点，观察变量 */
}
```

调试器里在 `for(;;)` 处看这些变量，把 `pc` 用 `arm-none-eabi-addr2line -e out.elf -f -i 0x0800xxxx` 转成源码行。注意 `pc` 是**出错指令的地址**（精确故障时）；`lr` 在压栈帧里是**出错前的 LR**，可以追到调用者。

### 9.3 手算示例：解码一次 BusFault

某次崩溃读到（教学示例）：

- CFSR = `0x00008200`，HFSR = `0x40000000`，BFAR = `0x20020000`，压栈 PC = `0x0800A3C6`，LR = `0x0800A3B1`。

解码：

- `0x00008200` = 二进制 `1000 0010 0000 0000`：位 15（BFARVALID）与位 9（PRECISERR）为 1；
- 所以是**精确的数据总线错误**，出错地址 BFAR = 0x20020000，PC = 0x0800A3C6 就是出错的那条读/写指令；
- HFSR 位 30（FORCED）为 1，表示是由可配置故障升级成的 HardFault（BusFault 未使能）；
- 地址 0x20020000：假设 F407 的 SRAM1 在 0x2000_0000，大小 112 KiB = 0x1C000，则 SRAM1 末尾为 0x2001_C000，SRAM2（16 KiB）占 0x2001_C000–0x2002_0000；0x2002_0000 恰好是 SRAM2 的末尾之后，也就是**越界一字节**到了没有映射的区域。

结论：某个指针/数组下标越过 RAM 末尾——比如把栈当成往上长，或者数组越界写到 RAM 之外。接着看 PC 对应的源码行，找该循环的边界条件。可以用 Python 复核位解码：

```python
cfsr = 0x00008200
bits = [i for i in range(32) if cfsr >> i & 1]
print(bits)                          # [9, 15]
print(hex(0x20000000 + 112*1024), hex(0x20000000 + 128*1024))
# 0x2001c000 0x20020000
```

### 9.4 常见 HardFault 原因对照

| 症状（CFSR） | 常见原因 |
|---|---|
| NOCP（位 19） | FPU 未使能就执行浮点；或中断里用了浮点而 FPU 配置错 |
| INVSTATE（位 17） | 函数指针 bit0 = 0；回调指针未初始化 |
| UNDEFINSTR（位 16） | 跑飞到数据区；栈溢出覆盖了返回地址 |
| STKERR / UNSTKERR | 栈指针指向无效地址；任务栈严重溢出 |
| PRECISERR + BFARVALID | 访问了未映射/未使能时钟的外设（如没开外设时钟就读寄存器），或数组越界 |
| IMPRECISERR | 对没使能时钟的外设做写（写缓冲导致 PC 不准）；排查时可临时关闭写缓冲（`ACTLR.DISDEFWBUF`） |
| DACCVIOL | MPU 违规，如写 Flash 常量区 |
| DIVBYZERO | 整数除零（需 `SCB->CCR |= SCB_CCR_DIV_0_TRP_Msk`） |

最常见的一类是**外设时钟没开就访问寄存器**——例如在 `__HAL_RCC_xxx_CLK_ENABLE()` 之前调用外设初始化，这在 F4/H7 上会触发总线错误或读到全零。

---

## 10. 在七个项目里怎么用

| 项目 | 关联点 | 性质 |
|---|---|---|
| P3（F407） | 任务栈与堆：FreeRTOS 的堆是 `.bss` 里的数组；开发板 C 型 F407 的 CCM 不能给 SPI+DMA 的 IMU 缓冲用（BMI088 使用 SPI+DMA，见 [P3 自查表](../../01_项目课程/P3_RoboMaster英雄机器人嵌入式控制/README.md)）；`printf` 调试若栈太小会把任务栈撑爆 | 课程项目事实 + 通用工程知识 |
| P7（H7） | 控制任务栈 4 KB、水位要求；IMU 与 CAN 缓冲放不可缓存段；浮点上下文保存；这些都在 [P7 软件结构](../../01_项目课程/P7_可变腿长轮腿机器人/10_STM32H7_FreeRTOS_FDCAN软件结构.md) 的设计取值中 | 项目取值 |
| P1 / P2 | P2 的底盘固件基于 STM32H7/FDCAN（[P2 固件解读](../../01_项目课程/P2_未来动力双臂遥操作与差速底盘/09_STM32H7_FDCAN底盘固件解读.md)），启动与内存问题同 P7 | 项目事实 |
| P4（Hi3861） | Hi3861 的 RAM 仅约 352 KB，每任务栈几 KB，堆有限（见 [平台基础](./06_OpenHarmony_Hi3861_RK3588平台基础.md)）；其内核不是 Cortex-M 而是 RISC-V，向量表与启动细节不同，只有"栈预算""堆有限"的思路通用 | 延伸知识（已注明架构不同） |
| P5 / P6 | 运行在 Linux/OpenHarmony 标准系统上，有 MMU 与虚拟内存，不适用本章的裸机内存映射 | 不适用，仅作对照 |

> **诚实边界**：CV 没有写你配置过 MPU、写过链接脚本。面试时若被问"你写过链接脚本吗"，诚实回答"我读过并改过 CubeMX 生成的脚本（例如放置缓冲段），能解释每一段的作用"比夸大更安全，前提是你确实做过；没做过就说"我理解原理，项目里使用的是工具生成的版本"。

---

## 11. 常见错误与故障现象

1. **把 DMA 缓冲放进 CCM（F4）或 DTCM（H7）**：传输看似成功，数据却不变；用示波器看到 SPI 时钟有，数据在缓冲里不更新。
2. **H7 忘记处理 Cache 一致性**：传感器数据"偶尔"是上一帧，控制出现 1 周期延迟抖动。
3. **栈大小单位弄错**（字 vs 字节）：任务栈比预想的小 4 倍，加一句 `printf` 就崩。
4. **用 `double` 常量**：`x * 0.5` 在 float 变量上触发 double 运算，F4 上变成软件浮点，周期被拖长数倍。
5. **FPU 没使能**：第一条浮点指令触发 UsageFault（NOCP），表现为一启动就进 HardFault。
6. **ISR 里调用 `malloc`**：堆没有保护，重入破坏堆结构，偶发崩溃。
7. **向量表重定位后未设置 VTOR**：应用里中断一发生就跳到 Bootloader 的 ISR 或 0 地址。
8. **链接脚本栈与堆区域重叠**：堆往上长、栈往下长，相遇后互相覆盖；`_Min_Stack_Size` 的"链接期检查"能提前发现。
9. **`volatile` 缺失**：寄存器位读写被编译器优化掉（局部），或 ISR 与主循环共享的标志没加 `volatile`，主循环永远看不到变化。
10. **在 `Reset_Handler` 之前用全局变量**：启动前 `.data` 还没拷贝，读到的是随机值。

---

## 12. 英文面试问答

### Q1. What happens between reset and `main()` on a Cortex-M?

**Answer.** After reset, the hardware reads the initial stack pointer from the first word of the vector table, and the reset handler address from the second word. Then the reset handler runs. It copies the initialised globals, the `.data` section, from flash to RAM, zeroes the `.bss` section, calls `SystemInit` to enable the FPU and set the vector table, runs the C library init so global constructors are called, and finally calls `main`. If any of these steps were missing, global variables would have random values.

**中文译文**：复位后硬件先从向量表第一个字读初始栈指针，再从第二个字读复位处理函数地址。复位处理函数把已初始化全局变量从 Flash 拷到 RAM，清零 `.bss`，调用 `SystemInit` 使能 FPU、设置向量表，执行 C 库初始化以调用全局构造函数，最后调用 `main`。缺任何一步，全局变量都会是随机值。

**Follow-up 1:** Why must initial values be stored in flash as well as RAM? — Because RAM loses its content at power-off, but the variables must be writable at run time, so flash holds the initial image and startup code copies it. （因为 RAM 掉电丢数据，而变量运行时必须可写，所以 Flash 存初值，启动代码拷贝。）

**Follow-up 2:** What is the difference between LMA and VMA? — LMA is where the data is stored at load time, VMA is where it is used at run time. For `.data`, LMA is in flash and VMA is in RAM. （LMA 是存放位置，VMA 是运行时使用位置；对 `.data`，LMA 在 Flash、VMA 在 RAM。）

### Q2. How would you debug a HardFault?

**Answer.** First I make sure the fault handlers are enabled so I get a precise cause. In the HardFault handler I check the EXC_RETURN bit two to see if the stack is MSP or PSP, and read the stacked PC and LR. Then I read CFSR, HFSR, and if valid, BFAR or MMFAR. The CFSR tells me the type, for example a precise bus error, an undefined instruction, or a stacking error. Then I convert the stacked PC to a source line with addr2line. In my experience the usual causes are stack overflow, accessing a peripheral before its clock is enabled, a bad function pointer, or the FPU not being enabled.

**中文译文**：先使能各类故障异常以获得精确原因。在 HardFault 处理函数里，根据 EXC_RETURN 的 bit2 判断是 MSP 还是 PSP，读出压栈的 PC 和 LR，再读 CFSR、HFSR，有效时读 BFAR 或 MMFAR。CFSR 告诉我故障类型，例如精确总线错误、未定义指令、压栈错误。再用 addr2line 把压栈 PC 转成源码行。常见原因是栈溢出、外设时钟未开就访问、函数指针错误或 FPU 未使能。

**Follow-up 1:** What is an imprecise bus error and why is it harder? — The write was buffered, so the fault is reported after the instruction has moved on, and the stacked PC does not point to the culprit. I can disable the write buffer temporarily to make it precise. （写操作被缓冲，故障延后报告，压栈 PC 不指向真凶；可临时关闭写缓冲使其变精确。）

**Follow-up 2:** What if the stack pointer itself is corrupted? — Then stacking fails and you see a STKERR; I would check the task stack high-water marks and enable the MPU stack guard. （则压栈本身失败，出现 STKERR；我会查任务栈水位并启用 MPU 栈保护。）

### Q3. On STM32H7, why can DMA data look stale, and how do you fix it?

**Answer.** The Cortex-M7 has a data cache. DMA writes directly to RAM and bypasses the cache, so the CPU can read an old copy from the cache. There are two fixes. One is to put the DMA buffer in a non-cacheable region configured with the MPU. The other is to invalidate the cache lines after DMA receive, and clean them before DMA transmit, with addresses aligned to the 32-byte cache line. In my wheel-legged project, the design is to keep the IMU and CAN buffers in a dedicated non-cacheable section, so the control task always sees fresh data.

**中文译文**：Cortex-M7 有数据 Cache，DMA 直接写 RAM 不经过 Cache，CPU 可能读到 Cache 里的旧副本。两种办法：一是用 MPU 把 DMA 缓冲所在区域设为不可缓存；二是接收后失效 Cache 行、发送前清理 Cache 行，地址按 32 字节对齐。我的轮腿项目的设计是把 IMU 与 CAN 缓冲放在专用不可缓存段，保证控制任务读到的是最新数据。

**Follow-up 1:** Why 32-byte alignment? — Cache operations work on whole cache lines, so a misaligned buffer can invalidate neighbouring variables and lose their data. （Cache 操作以整行为单位，不对齐会连带失效相邻变量并丢数据。）

**Follow-up 2:** What is the downside of non-cacheable memory? — The CPU accesses it more slowly, so you keep it for small I/O buffers only. （CPU 访问更慢，所以只用于小型 I/O 缓冲。）

### Q4. How do you decide a task's stack size in FreeRTOS?

**Answer.** I start with an estimate from the call depth and local variables, then I add the exception frame, which is 104 bytes if the FPU is used. Then I run the system under the worst case, for example with logging on, and read `uxTaskGetStackHighWaterMark`, which returns the minimum free words ever seen. I keep at least 25 percent margin. One trap is the unit: the stack size parameter is in words, not bytes, so 1024 means 4 kilobytes on a 32-bit core.

**中文译文**：先按调用深度和局部变量估算，加上异常帧（用到 FPU 时为 104 字节）；然后在最坏工况（例如开日志）下运行，读 `uxTaskGetStackHighWaterMark` 得到历史最小剩余字数；至少留 25% 余量。陷阱是单位：栈大小参数以字为单位，1024 在 32 位核上是 4 KB。

**Follow-up 1:** Why does adding `printf` crash a task? — `printf` uses a lot of stack for formatting, and floating-point formatting uses even more. （`printf` 格式化占栈很大，浮点格式化更多。）

**Follow-up 2:** How can the MPU help? — A read-only guard region at the stack bottom turns silent overflow into an immediate MemManage fault. （栈底设只读保护区，把静默溢出变成立即触发的 MemManage 故障。）

---

## 13. 要点速记、自测题、相关链接

### 要点速记

1. 复位：硬件读 `[0]`→SP，`[1]`→PC；Reset_Handler 的 bit0 必须为 1。
2. `Reset_Handler` 顺序：拷 `.data`、清 `.bss`、`SystemInit`、`__libc_init_array`、`main`。
3. Flash 占用 = text + data；静态 RAM 占用 = data + bss。
4. LMA 与 VMA 不同是因为 `.data` 要"Flash 存、RAM 用"。
5. F4 的 CCM 与 H7 的 DTCM 对 DMA 不可达；H7 有 Cache，需要一致性处理。
6. 异常帧 8 字（32 B）；带浮点惰性压栈时 26 字（104 B）。
7. EXC_RETURN bit2 决定从 MSP 还是 PSP 取压栈寄存器。
8. HardFault 先看 CFSR、HFSR，再看 BFAR/MMFAR，再用 PC 查源码行。
9. FreeRTOS 栈参数单位是字；水位函数返回历史最小剩余字数。
10. `sinf` 对比 `sin`：单精度 FPU 上避免 double。

### 自测题

1. 复位后 CPU 读的前两个字分别是什么？为什么第二个字最低位必须是 1？
2. 某固件 text=30000，data=2000，bss=18000，求 Flash 与静态 RAM 占用。
3. 链接脚本里 `>RAM AT> FLASH` 起什么作用？去掉 `AT> FLASH` 会怎样？
4. 带 FPU 的 M4 上，一个 ISR 被更高优先级 ISR 嵌套，各用扩展帧 104 B，局部分别为 60 B 与 40 B，求最坏 MSP 栈需求。
5. 读到 CFSR = 0x00020000，说明什么？可能的原因是什么？

**参考答案要点**：1. 初始 MSP 与 Reset 向量；Cortex-M 只支持 Thumb，PC 的 bit0 表示 Thumb 状态，为 0 会 INVSTATE。2. Flash 32000 B，RAM 20000 B。3. 运行地址在 RAM，加载地址在 Flash；去掉后变量运行地址在 Flash，写入失败或 HardFault，且启动代码没有初值可拷。4. (104+60)+(104+40)=308 B，应再留余量。5. 位 17，INVSTATE：函数指针 bit0 为 0 或跳到了非 Thumb 地址。

### 相关链接

- 上一章：[电机、编码器、驱动与力矩控制](./05_电机_编码器_驱动与力矩控制.md)；平台：[OpenHarmony、Hi3861、RK3588](./06_OpenHarmony_Hi3861_RK3588平台基础.md)
- 下一章：[电源管理、低功耗、看门狗、Bootloader 与 OTA](./08_电源管理_低功耗_看门狗_Bootloader_OTA.md)
- 相关：[STM32 时钟、GPIO、中断、DMA](./01_STM32_时钟_GPIO_中断_DMA.md)、[FreeRTOS 实时系统](./02_FreeRTOS实时系统.md)、[嵌入式调试](./10_嵌入式调试_JTAG_SWD_逻辑分析仪_故障定位.md)
- 八股速查：[C 语言与嵌入式八股](../../03_八股知识/01_计算机基础/04_C语言与嵌入式八股.md)、[嵌入式与机电八股](../../03_八股知识/03_机器人学/06_嵌入式与机电八股.md)
- 项目：[P3](../../01_项目课程/P3_RoboMaster英雄机器人嵌入式控制/README.md)、[P7](../../01_项目课程/P7_可变腿长轮腿机器人/README.md)
