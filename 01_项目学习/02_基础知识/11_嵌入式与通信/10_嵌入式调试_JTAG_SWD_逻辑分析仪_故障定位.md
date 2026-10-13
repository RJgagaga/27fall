# 嵌入式调试：JTAG/SWD、断点与跟踪、逻辑分析仪、示波器与故障定位

> **一句话定位**：控制算法在仿真里跑得很好，上了机器人却"偶尔抽一下"，这时你手里的工具不是 IDE 的单步，而是 SWD 调试口、DWT 周期计数器、GPIO 翻转、逻辑分析仪和示波器。这一章讲这些工具的原理、选型的手算依据，以及一套可复用的故障定位方法，并结合 P3 的 CAN 离线/周期抖动案例与 P7 的 GPIO + 逻辑分析仪 + DWT 测量讲清怎么联调。

**本章学完你能：**

- 区分 JTAG 与 SWD，说出 SWD 的引脚、线速与 Cortex-M 调试架构（DAP、DWT、FPB、ITM、ETM）；
- 解释硬件断点与软件断点、数据观察点（watchpoint）的区别和数量限制；
- 用 DWT_CYCCNT 精确测量任务执行时间，并估计溢出时间；
- 估算逻辑分析仪采样率与存储深度、示波器带宽的最低要求；
- 解码 UART、SPI、I2C、CAN 的典型波形，快速判断模式/极性/上拉/终端电阻错误；
- 按"复现—隔离—假设—测量—验证"的流程定位故障，并写出可复盘的记录；
- 用英文描述一次完整的调试经历（STAR）。

**前置知识：**[Cortex-M 体系与 HardFault](./07_Cortex-M体系_启动_链接脚本_内存映射.md)；[STM32 一章](./01_STM32_时钟_GPIO_中断_DMA.md)；[FreeRTOS](./02_FreeRTOS实时系统.md)；[CAN 与 FDCAN](./03_CAN_FDCAN与电机通信.md)；[UART/SPI/I2C](./04_UART_SPI_I2C_以太网_TCP.md)。

**相关项目：**[P3 调试与实战故障案例](../../01_项目课程/P3_RoboMaster英雄机器人嵌入式控制/11_调试与实战故障案例.md)、[P7 调试与故障手册](../../01_项目课程/P7_可变腿长轮腿机器人/11_调试与故障手册.md)、[P7 工程化实践](../../01_项目课程/P7_可变腿长轮腿机器人/20_工程化实践_频率周期与问题解决.md)。

## 目录

1. 要解决什么问题
2. 符号与单位
3. 调试接口：JTAG 与 SWD
4. 片上调试资源：FPB、DWT、ITM、ETM
5. 断点、观察点与"不能停"的系统
6. 用 DWT 与 GPIO 测时间
7. 逻辑分析仪：采样率、深度与协议解码
8. 示波器：带宽、探头与信号完整性
9. 总线故障的典型波形
10. 故障定位方法论
11. 完整手算汇总
12. 在七个项目里怎么用
13. 常见错误
14. 英文面试问答
15. 要点速记、自测题、相关链接

---

## 1. 要解决什么问题

### 1.1 机器人调试的三个特点

1. **不能随便停**：电机在转，一个断点会让 CPU 停住，但电机驱动器还会按旧命令继续转，机器人可能倒下。所以"单步调试"只适合静态部分，运行时问题要靠**不打断系统的观测手段**：日志环形缓冲、GPIO 翻转、跟踪。
2. **问题偶发**：跨任务的竞争、DMA 一致性、总线偶发错误——都不是每次都出现。需要**记录到足够的上下文**，再回放分析。
3. **软硬交叉**：同一个"电机抖"可能来自接地、供电、总线终端、中断优先级或控制参数。需要能在**不同层**测量（电压、数字波形、总线帧、软件变量）。

### 1.2 核心思想

> **先测量，再假设。** 没有测量的"我觉得是 PID 参数不对"，就是猜。

---

## 2. 符号与单位

| 符号 | 含义 | 单位 |
|---|---|---|
| $f_{cpu}$ | CPU 时钟 | Hz |
| CYCCNT | DWT 周期计数器（32 位） | 周期 |
| $t_{exec}$ | 任务执行时间 | s、µs |
| $f_s$ | 逻辑分析仪采样率 | S/s（采样每秒） |
| $D$ | 存储深度 | 采样点数 |
| $t_{cap}$ | 单次捕获时长 | s |
| $BW$ | 示波器模拟带宽 | Hz |
| $t_r$ | 信号上升时间（10%–90%） | s |
| $f_{baud}$ | 波特率 | bit/s |

---

## 3. 调试接口：JTAG 与 SWD

### 3.1 JTAG

JTAG（IEEE 1149.1）：边界扫描标准，引脚 **TCK、TMS、TDI、TDO**（可选 nTRST）。多个芯片可以链式连接成一条扫描链。ARM 用它做调试访问，是老式标准接口。

### 3.2 SWD

SWD（Serial Wire Debug）：ARM 定义的两线调试接口，引脚 **SWCLK、SWDIO**（双向），常加 **NRST**（复位）、**SWO**（单线输出，用于 ITM 跟踪），再加电源与地。优点：占用引脚少（JTAG 4–5 根 → SWD 2 根），功能等价。Cortex-M 上 SWD 与 JTAG 共用引脚（STM32 上 PA13=SWDIO、PA14=SWCLK，PA15/PB3/PB4 是 JTAG 额外引脚，常被重映射用作 GPIO）。

> **常见坑**：在代码里把 PA13/PA14 配成普通 GPIO，会把调试口关掉，之后调试器连不上，需要在复位的同时连接（"connect under reset"）或用 BOOT0 进入系统存储器后擦除。

### 3.3 调试架构

```text
 调试器 (ST-Link / J-Link / CMSIS-DAP)
      |  SWD (SWCLK, SWDIO)
      v
 [ DP: 调试端口 ]  -->  [ AP: 访问端口 (AHB-AP) ]  -->  内存总线
                              |
                              +--> CPU 内核调试寄存器 (DHCSR 等: 停机/单步/读写寄存器)
                              +--> FPB   (Flash Patch and Breakpoint)  硬件断点
                              +--> DWT   (Data Watchpoint and Trace)    观察点/周期计数
                              +--> ITM   (Instrumentation Trace)        printf 通道
                              +--> ETM   (Embedded Trace，部分型号)      指令跟踪
```

调试器通过 AHB-AP 能**在不停止 CPU 的情况下读写内存**（"live watch"），这是可以在运行中观察变量的原因，但对带 Cache 的 H7，读到的是内存而不是 Cache 里的值（受一致性影响）。

### 3.4 SWD 速度

SWD 时钟常设 1–10 MHz（视线长和调试器，**以调试器手册为准**）。线长超过 20 cm 或接线质量差时降低速度。

---

## 4. 片上调试资源：FPB、DWT、ITM、ETM

### 4.1 FPB：硬件断点

Cortex-M3/M4 的 FPB 有 6 个指令比较器 + 2 个字面量比较器（共 8 个比较器，其中 6 个可当断点）；Cortex-M7 可达 8 个。**Flash 里的代码只能用硬件断点**（不能修改 Flash 插入指令），所以同时可设置的断点数量有限（量级 6–8，以内核手册为准）。RAM 里的代码可以用软件断点（把指令替换成 BKPT）。

### 4.2 DWT：数据观察点与周期计数

- **观察点**：监视某个地址的读/写，触发时停机或产生事件，数量有限（M4 常见 4 个）。用途：抓"谁把这个变量写坏了"——这是 C 语言野指针、数组越界类 bug 的利器。
- **CYCCNT**：32 位周期计数器，每个 CPU 周期加 1。使能方法：`CoreDebug->DEMCR |= CoreDebug_DEMCR_TRCENA_Msk; DWT->CYCCNT = 0; DWT->CTRL |= DWT_CTRL_CYCCNTENA_Msk;`（M7 有些版本需先解锁 `DWT->LAR = 0xC5ACCE55`）。
- 其他计数器：CPI、异常开销、睡眠周期、LSU 等。

### 4.3 ITM 与 SWO

ITM 提供多个"激励端口"，把 `printf` 重定向到 ITM 端口 0，通过 SWO 引脚输出到调试器。优点：比 UART 快、占用少，不需要额外串口。SWO 的波特率受限于 CPU 时钟的分频和调试器（常见 1–2 MHz 量级）。缺点：输出大量日志时会阻塞（FIFO 满等待），所以**高频日志要节流**，或只在关键事件发。

### 4.4 ETM 与指令跟踪

ETM 将指令流实时压缩输出到 trace 端口，通过 4 位 trace 引脚送到专用调试器（如 J-Trace），可得到**完整执行历史**，对偶发崩溃极有价值，但硬件昂贵、并非所有型号具备（以芯片手册为准）。

### 4.5 RTOS 感知的跟踪

用 SEGGER SystemView、Percepio Tracealyzer 等工具通过 RTT/SWO 记录任务切换、中断、队列操作，生成时间线图。它们能把"控制任务抖动"可视化为"被哪个中断抢占了多久"。使用时需要在 FreeRTOS 配置里加跟踪宏（`traceTASK_SWITCHED_IN` 等），代价是少量 CPU 开销。

---

## 5. 断点、观察点与"不能停"的系统

### 5.1 在运行的电机系统上用断点的后果

CPU 停住后：

- 控制任务不再更新，电机驱动器保持最后一条命令（P3 的 RELAX 概念：命令超时后电调自行停止或保持，取决于驱动器行为）；
- 看门狗会复位（除非用 DBGMCU 冻结，见 [上一章](./08_电源管理_低功耗_看门狗_Bootloader_OTA.md)）；
- CAN 总线上心跳停止，对端判失联。

所以调试带电机的系统要遵循：

1. **先让电机在安全状态**（抬离地面、断开功率电或限流、软件置零力矩）；
2. 断点只在静态路径（初始化、参数解析）使用；
3. 运行路径用 **条件日志 / GPIO / 跟踪**；
4. 若必须停，使用 **tracepoint（不停机采样）** 或"断点动作后继续"。

### 5.2 观察点抓内存被踩

故障：某个全局数组 `g_params[8]` 在运行中被改成怪值。做法：在 IDE 中对该地址设数据写观察点；触发时调试器停下，调用栈显示谁写的。结合 [07 章](./07_Cortex-M体系_启动_链接脚本_内存映射.md) 的 map 文件，可以找到相邻变量，判断是不是某数组越界。

---

## 6. 用 DWT 与 GPIO 测时间

### 6.1 DWT 测执行时间

```c
static inline uint32_t cyc(void) { return DWT->CYCCNT; }

void ControlStep(void)
{
    uint32_t t0 = cyc();
    do_control();                          /* 被测代码 */
    uint32_t dt = cyc() - t0;              /* 无符号减法，溢出一次也正确 */
    if (dt > g_dt_max) g_dt_max = dt;      /* 保存最大值 */
    g_dt_sum += dt; g_dt_cnt++;
}
```

**手算**：$f_{cpu}=168$ MHz：

- 1 周期 $=5.95$ ns；
- 若某次读数 $dt=63\,840$ 周期，则 $t=63840/168\times10^6=380\ \mu s$（正好是 P7 里控制任务一个周期执行预算的数量级，参见 [P7 软件结构](../../01_项目课程/P7_可变腿长轮腿机器人/10_STM32H7_FreeRTOS_FDCAN软件结构.md) 的预算表；此处只是数值换算示例，不代表实测）；
- 一个 2 ms 周期共 $168\times10^6\times2\times10^{-3}=336\,000$ 周期，占比 $63\,840/336\,000=19\%$；
- 计数器溢出时间 $2^{32}/168\times10^6=25.6$ s。测量间隔必须小于这个值；**无符号减法在只溢出一次时依然正确**，因为模 $2^{32}$ 运算。

### 6.2 GPIO 翻转

在任务入口置高、出口置低，用示波器/逻辑分析仪看波形：

- 高电平宽度 = 执行时间；
- 相邻上升沿间隔 = 实际周期；
- 周期的抖动 = 上升沿间隔的变化范围；
- 若在 ISR 入口/出口各用一个引脚，能看到 ISR 抢占任务的时间点。

优点：与 CPU 无关、不改变时序（一次 GPIO 写只有几个周期）、能看到长时间的统计。P7 项目就是这样验证控制周期的：GPIO 翻转 + 逻辑分析仪看控制任务入口间隔，DWT 统计执行时间；第一版用 1 ms 系统节拍的 delay-until，约 1 ms 抖动；改为硬件定时器释放任务后降到约 0.1 ms（见 [P7 工程化实践](../../01_项目课程/P7_可变腿长轮腿机器人/20_工程化实践_频率周期与问题解决.md)）。

### 6.3 为什么 1 ms tick 的 delay-until 抖动约 1 ms

任务唤醒对齐在 tick 边界，且受其他同优先级或更高优先级就绪任务影响。当周期是 2 ms（2 个 tick）时，任务在 tick 中断后才被调度，抖动量级与 tick 周期同阶。改用硬件定时器中断直接 `vTaskNotifyGiveFromISR` 唤醒，唤醒时刻由硬件决定，只剩中断延迟与调度开销（几微秒到几十微秒），再加最高优先级抢占，实测量级可到 0.1 ms。这与 [FreeRTOS 一章](./02_FreeRTOS实时系统.md) 的 tick 量化分析一致。

---

## 7. 逻辑分析仪：采样率、深度与协议解码

### 7.1 逻辑分析仪是什么

采集数字信号（0/1），按时间显示，并解码协议（UART、SPI、I2C、CAN、1-Wire 等）。它采样的是**门限判决后的数字电平**，不是模拟波形，所以看不出过冲、振铃、斜率问题，那些要用示波器。

### 7.2 采样率选择

**规则**：采样率至少是信号最高变化频率的 4–10 倍。更精确地说，要解码的信号，一个比特要有至少 4 个采样点，才能确定边沿位置和采样点；要判断抖动或毛刺则需要更高。

**手算**：

- UART 115200 baud：位时间 $1/115200=8.68\ \mu s$；采样率取 10 倍位速率 $=1.15$ MS/s，实际用 2–4 MS/s 即够；
- SPI 10 MHz 时钟：半周期 50 ns；要分辨边沿并检查时序，至少 50–100 MS/s；
- CAN 1 Mbps：位时间 1 µs，10 倍 $=10$ MS/s；
- I2C 400 kHz：位时间 2.5 µs，几 MS/s 足够。

### 7.3 存储深度与捕获时长

$$ t_{cap}=\frac{D}{f_s}. $$

**手算**：深度 1 M 采样点（$D=10^6$），采样率 10 MS/s：$t_{cap}=0.1$ s。要抓"偶发 2 s 一次的 CAN 错误"，不能用高速采样 + 小深度；需要**触发**（触发条件=错误帧 / 特定 ID / 某信号的毛刺）并用**预触发缓冲**保存触发前的数据。或者降低采样率：深度 1 M，$f_s=1$ MS/s，$t_{cap}=1$ s——但对 1 Mbps CAN 只剩 1 个采样/位，不够解码。取舍：用协议触发，只抓有价值的段落。

### 7.4 触发设置

好的触发把"偶发"变成"可捕获"：

- 边沿触发：某信号上升沿；
- 脉宽触发：脉冲宽度小于/大于某值（抓毛刺或超长执行时间）；
- 协议触发：特定 UART 字节、特定 CAN ID、I2C NACK；
- 序列触发：先出现 A，随后 B（抓顺序错误）。

### 7.5 协议解码时的常见发现

| 现象 | 可能原因 |
|---|---|
| UART 解码出乱码 | 波特率不对（解码器设置与设备不同）；极性反了；数据位/校验设置不一致 |
| SPI 全 0xFF 或全 0x00 | 从机没选中（CS 极性）、MISO 没连、从机没供电/没复位 |
| SPI 数据整体移位一位 | CPOL/CPHA（时钟极性/相位）设置错误：模式 0–3 采样沿不同 |
| I2C 无 ACK | 地址（7 位 vs 8 位格式）写错；缺上拉电阻；从机没供电 |
| I2C 波形边沿圆钝 | 上拉电阻太大或总线电容太大；需要降速或换小电阻 |
| CAN 只有发送没有应答 | 对端没上电、波特率不一致、终端电阻缺失 |

---

## 8. 示波器：带宽、探头与信号完整性

### 8.1 带宽与上升时间

示波器带宽 $BW$ 与它能看清的信号上升时间关系：

$$ BW\approx\frac{0.35}{t_r}\ (\text{示波器自身上升时间}). $$

要准确显示信号，示波器带宽至少为信号**最高有效频率成分**的 3–5 倍。数字信号的最高有效频率成分与上升时间相关：$f_{knee}\approx 0.35/t_r$（信号自身）。

**手算**：SPI 时钟 10 MHz，信号上升时间 5 ns。$f_{knee}=0.35/5\ \text{ns}=70$ MHz。示波器至少 $3\sim5$ 倍 → 200–350 MHz。如果只有 20 MHz 的示波器，看到的上升沿会被拉长成约 17.5 ns，边沿过冲和振铃全部看不见。对 CAN 1 Mbps、边沿几十纳秒的信号，100 MHz 示波器足够。

### 8.2 探头

- 10X 无源探头：输入电容约 10–15 pF，高阻，使用前必须用示波器自带的校准方波**补偿**（否则方波有过冲或圆角）；
- **接地夹引线越长电感越大**，会在快边沿上产生振铃，是"看到的振铃其实是探头引入的"的常见原因；高速信号用弹簧接地环（探针尖旁边直接接地）；
- 差分信号（CAN_H / CAN_L）：分别用两个探头测 H、L 相对地，并用数学通道 CH1−CH2 得到差分电压，或用差分探头；不要把示波器地夹夹在其中一条线上（会短路该线到大地）。

### 8.3 测什么

1. **电源轨**：纹波和跌落（用 AC 耦合、20 MHz 带宽限制、触发在跌落事件）；
2. **复位线**：看是否有毛刺；
3. **时钟起振**：晶振波形幅度；
4. **总线电平**：CAN 的显性差分约 2 V、隐性约 0 V；I2C 低电平是否小于 0.3 VDD。

---

## 9. 总线故障的典型波形

### 9.1 CAN

| 现象 | 看什么 | 常见根因 |
|---|---|---|
| 无通信 | CAN_H/CAN_L 静态电平是否约 2.5 V；终端电阻（断电测 CAN_H–CAN_L 应约 60 Ω） | H/L 接反、断线、终端电阻缺失/多了 |
| 偶发错误帧 | 示波器看显性位波形和采样点位置；错误计数器 | 线束接触不良、接地环路、位定时参数错 |
| 部分电机离线 | 逐个拔下节点，看错误是否消失 | 某节点短路拉低总线；供电跌落使节点复位 |
| Bus-Off | 读 ESR/ECR 寄存器的 TEC/REC | 发送无应答（对端没起），错误计数累积 |

测终端电阻：断电后量 CAN_H 与 CAN_L 之间，两端各一个 120 Ω 并联应约 60 Ω；若约 120 Ω，说明只有一个终端；若远大于，说明断线。这是最快的现场检查（见 [CAN 一章](./03_CAN_FDCAN与电机通信.md) 终端电阻部分）。

### 9.2 UART

起始位低、数据位 LSB 先、停止位高。逻辑分析仪量位宽 $\to$ 倒推波特率：量到最窄脉冲 8.7 µs → 约 115200 baud。解码错乱先检查波特率误差：要求总误差 < 约 2%–3%（累计 10 位后偏移不超过半个位），在时钟 HSI（±1%）与外设分频误差叠加时需要留意。

### 9.3 SPI

SPI 四个信号：SCK、MOSI、MISO、CS。逻辑分析仪抓下来，看 CS 低期间的时钟个数是否为 8 的倍数、MISO 在采样沿是否稳定。IMU（如 BMI088）读寄存器的常见格式：第一个字节 bit7=1 表示读，后面时钟周期内 MISO 返回数据（加速度计读时往往有一个 dummy 字节，具体以器件手册为准）。若 DMA 缓冲不对，逻辑分析仪上波形正常而内存里数据不对——说明问题在 MCU 端（Cache、地址、CCM）而不是线路。

### 9.4 I2C

SDA/SCL 开漏，需要上拉。上拉电阻太大，上升沿慢，高速时无法达到高电平；太小，低电平时电流大，低电平电压不够低。4.7 kΩ 与 400 pF 的总线电容：$RC=4.7\text{k}\Omega\times400\ \text{pF}=1.88\ \mu s$，上升时间（30%–70%）$\approx0.85RC=1.6\ \mu s$，超过 400 kHz 模式规定的最大 300 ns，所以高速 I2C 要用小上拉（如 1–2 kΩ）并控制线长。

---

## 10. 故障定位方法论

### 10.1 五步法

1. **复现**：找到稳定触发条件；若偶发，先量化发生率（每分钟几次）；
2. **隔离**：把系统切成小块，逐块屏蔽。电机动力断开？只留传感器？只留一个任务？换一块板？
3. **假设**：基于现象写出**可证伪**的假设，例如"控制周期偶发被拉长到 5 ms，原因是日志任务优先级过高"；
4. **测量**：设计能区分假设的测量。GPIO 看周期，跟踪看抢占；
5. **验证**：改动一处，对比前后；写下结果。

### 10.2 分层定位表

| 层 | 工具 | 回答的问题 |
|---|---|---|
| 物理/电源 | 万用表、示波器 | 电压对吗？地线干净吗？有跌落吗？ |
| 总线信号 | 示波器、逻辑分析仪 | 波形是否合规？帧是否正确？ |
| 协议 | CAN 分析仪、逻辑分析仪解码 | ID、DLC、数据是否符合协议？ |
| 驱动/外设 | 寄存器视图、SWD 读寄存器 | 外设状态是否正确（错误标志、FIFO 计数）？ |
| 任务/调度 | 跟踪、GPIO、DWT | 周期、抖动、抢占、栈水位？ |
| 算法/数值 | 日志、回放 | 输入输出是否合理？是否限幅、饱和？ |

**原则**：自下而上排查，先排除上层之前先确认下层没问题。例如"电机不转"，先量电源和总线，再看 CAN 帧，再看驱动使能，最后看控制算法。

### 10.3 日志设计

高频控制系统的日志要点：

- 使用**环形缓冲**，每个周期写固定大小的结构（时间戳、关键状态、命令、模式、周期用时）；
- 日志写入不阻塞（不在 ISR 里 `printf`）；
- 触发式转储：出现异常（周期超时、数据过期、保护触发）时，把前后若干个周期落盘或上传；
- 时间戳用同一时钟（DWT 或定时器），便于与外部设备对齐。

这与 [P7 调试与故障手册](../../01_项目课程/P7_可变腿长轮腿机器人/11_调试与故障手册.md) 的"日志必须有的字段"和"复现与回放"一致。

### 10.4 案例：P3 的 CAN 离线与周期问题

**案例 1（来自 P3 项目文档的案例 5：CAN 偏线或供电掉电导致电机离线）**：现象为个别电机反馈中断、对应机构 RELAX。分层排查：先用万用表量电源（排除掉电）、断电量 CAN_H–CAN_L 阻值（排除断线/终端）、示波器看总线波形（排除 H/L 接触不良），再看软件离线检测（电机反馈 50 ms 超时）。处理：重新压线、固定接插件，加强供电去耦，软件离线检测保持（详见 [P3 调试与实战故障案例](../../01_项目课程/P3_RoboMaster英雄机器人嵌入式控制/11_调试与实战故障案例.md)）。

**案例 2（P3 案例 10：任务优先级与 `vTaskDelay` 造成周期抖动）**：现象是控制周期不是标称值，实际约 3 ms，因为循环内几处一 tick 延时累加（为避免 bxCAN 的 3 个发送邮箱占满）。定位手段：GPIO 翻转测周期（文档中的诚实回答模板就是"I would verify it with a GPIO toggle"）。改进方向：按邮箱空闲事件发送，而不是用延时（参见 [P3 FreeRTOS 任务设计与时序](../../01_项目课程/P3_RoboMaster英雄机器人嵌入式控制/03_FreeRTOS任务设计与时序.md)）。

### 10.5 案例：P7 的状态不一致

P7 案例 A（平衡点偏置）：吊起放下后车缓慢向前爬行。排查：日志显示位置误差单调增长；静止时 IMU 俯仰角读数有约 1.5° 偏置；原因是质心偏前和 IMU 安装偏差；处理是吊起读零位并标定偏置，每周期一次快照确保 IMU 与关节数据同步，结果静止 30 s 位移小于 3 cm（本项目取值）。这个案例的方法论价值：**"现象是爬行"最后落在"零位/同步"**，只有看日志里的状态量才能发现，调增益不会解决。

---

## 11. 完整手算汇总

| 量 | 公式 | 结果（教学示例） |
|---|---|---|
| 周期时间 | $1/f_{cpu}$ | 168 MHz：5.95 ns |
| 执行时间 | $dt/f_{cpu}$ | 63840 周期 = 380 µs（19% / 2 ms） |
| CYCCNT 溢出 | $2^{32}/f_{cpu}$ | 25.6 s |
| UART 位时间 | $1/baud$ | 8.68 µs |
| 逻辑分析仪最低采样 | 10×baud | 1.15 MS/s |
| 捕获时长 | $D/f_s$ | 1 M / 10 MS/s = 0.1 s |
| 示波器带宽 | $0.35/t_r$×(3–5) | 5 ns 边沿：70 MHz 膝点，需 200–350 MHz |
| I2C 上升时间 | $0.85RC$ | 4.7 kΩ、400 pF：1.6 µs |

```python
f = 168e6
print(1e9/f, 63840/f*1e6, 63840/(f*2e-3), 2**32/f)   # 5.95 380 0.19 25.57
print(1e6/115200, 115200*10)                            # 8.68  1152000
print(0.35/5e-9/1e6, 0.85*4.7e3*400e-12*1e6)            # 70.0  1.598
```

---

## 12. 在七个项目里怎么用

| 项目 | 关联点 | 性质 |
|---|---|---|
| P3 | 周期抖动用 GPIO 翻转核实；CAN 离线用万用表/示波器分层查（案例 5、10）；IMU 通过 SPI+DMA，数据异常时先用逻辑分析仪区分线路与 MCU 端 | 课程文档案例 |
| P7 | GPIO 翻转 + 逻辑分析仪看控制任务入口间隔，DWT 统计执行时间，周期抖动由约 1 ms 降到约 0.1 ms（项目设计取值）；日志环形缓冲与回放；三个完整案例（状态不一致、力矩限幅、离地切换） | 项目事实 |
| P1 | ROS 状态处理 + CAN 电机，故障排查强调日志与时序；见 [P1 调试案例](../../01_项目课程/P1_AIRS球形机器人平衡控制/09_调试案例与故障排查.md) | 项目事实 |
| P2 | H7 FDCAN 底盘固件的联调：CAN 报文与轮速反馈对照，见 [P2 调试与故障案例](../../01_项目课程/P2_未来动力双臂遥操作与差速底盘/11_调试与故障案例.md) | 项目事实 |
| P4 | Hi3861 的串口日志、ADC 读数与继电器动作对照；RK3588 侧用 Linux 工具（`dmesg`、`v4l2-ctl`）；见 [P4 故障与测试](../../01_项目课程/P4_OpenHarmony智能园艺系统/13_故障与测试.md) | 项目事实 |
| P5 / P6 | 软件层调试为主：`ros2 topic echo`、`rosbag2` 回放、日志（见 12 号模块） | 不适用本章硬件工具 |

> **诚实边界**：CV 没有写你使用了 ETM/SystemView 等高级跟踪工具。面试时应以你真实用过的工具为主（GPIO、逻辑分析仪、DWT、日志）；其余作为"我知道有这类工具"。

---

## 13. 常见错误

1. **在运行中的带电机系统设断点**，机器人冲出或倒下。
2. **把 SWD 引脚配成 GPIO**，调试器连不上。
3. **示波器地夹接在差分信号一端**，短路总线。
4. **逻辑分析仪采样率过低**：看到的"毛刺"其实是欠采样产生的假象。
5. **用 `printf` 查实时问题**：改变了时序，问题消失（海森堡效应）；日志应走环形缓冲。
6. **只看现象不分层**：电机抖就调 PID。
7. **一次改多个地方**：不知道哪个起作用。
8. **没有记录**：两周后同样问题再来，忘了怎么解决的。写故障复盘（现象—原因—处理—验证）。
9. **忽略复位原因**：重启问题不读复位标志。
10. **优化调试构建**：`-O0` 下能复现不了问题，`-O2` 才出现（或相反）；调试时尽量用与发布一致的优化级别。

---

## 14. 英文面试问答

### Q1. How do you debug a problem that happens only occasionally on a running robot?

**Answer.** I first try to make it measurable. I add a ring buffer that logs the key states, command and cycle time in every control period, and a trigger that dumps the buffer when something looks wrong, for example when a cycle is late or data is stale. I also toggle a GPIO at the start and end of the control task and look at it with a logic analyser to see the real period and jitter. With the log I can see what changed just before the event. I avoid breakpoints on a running robot because they stop the control loop, and I avoid printf because it changes the timing.

**中文译文**：我先让问题可测量。加一个环形缓冲，每个控制周期记录关键状态、指令和周期用时，并设触发条件，周期超时或数据过期时转储。同时在控制任务入口出口翻转 GPIO，用逻辑分析仪看真实周期和抖动。有了日志，就能看到事件发生前什么变了。我不在运行的机器人上用断点（会停掉控制环），也避免 printf（会改变时序）。

**Follow-up 1:** How did you measure the control period in your project? — I toggled a GPIO and watched it on a logic analyser, and I used the DWT cycle counter for execution time. （用 GPIO 翻转加逻辑分析仪看周期，用 DWT 周期计数器测执行时间。）

**Follow-up 2:** What did you find? — In the wheel-legged robot, the first version, which used a delay-until with a one-millisecond tick, had around one millisecond of jitter; releasing the task from a hardware timer reduced it to about a tenth of a millisecond. （第一版用 1 ms tick 的 delay-until，抖动约 1 ms；改用硬件定时器释放任务后降到约 0.1 ms。）

### Q2. What is the difference between JTAG and SWD, and what can you do with them?

**Answer.** Both are debug interfaces. JTAG uses four or five pins and can chain several devices. SWD is an ARM two-wire protocol that uses a clock and a bidirectional data line, so it saves pins and is standard on Cortex-M. Through the debug port I can halt the core, read and write memory and registers, set hardware breakpoints and watchpoints, and in many chips read memory without stopping the CPU. There is also SWO for trace output, such as printf through ITM. A common mistake is reconfiguring the SWD pins as GPIO, which locks the debugger out.

**中文译文**：两者都是调试接口。JTAG 用四五根线，可以菊花链多个器件；SWD 是 ARM 的两线协议，一根时钟一根双向数据线，省引脚，是 Cortex-M 的标配。通过调试口可以停核、读写内存和寄存器、设硬件断点和观察点，很多芯片还能在不停 CPU 的情况下读内存。还有 SWO 输出跟踪，比如通过 ITM 的 printf。常见错误是把 SWD 引脚重配成 GPIO，导致调试器连不上。

**Follow-up 1:** How many hardware breakpoints do you have? — Typically six to eight on a Cortex-M4 or M7, depending on the core; code in flash can only use hardware breakpoints. （M4/M7 通常 6–8 个，视内核而定；Flash 中的代码只能用硬件断点。）

**Follow-up 2:** How do you recover a chip that you locked out? — Connect under reset, or use the boot pin to enter the system bootloader and erase it. （复位状态下连接，或用 BOOT 引脚进入系统 Bootloader 擦除。）

### Q3. How do you choose the sampling rate of a logic analyser?

**Answer.** I use at least four, and preferably ten, samples per bit of the fastest signal I want to decode. For UART at 115200 baud a bit is about 8.7 microseconds, so a few megasamples per second is plenty. For a 10 MHz SPI clock I want around 50 to 100 megasamples per second to see the edges. Then I check the capture length: memory depth divided by sample rate. One million samples at 10 megasamples per second is only 0.1 seconds, so for rare events I use a protocol or pulse-width trigger with pre-trigger memory, not a long capture.

**中文译文**：对想解码的最快信号，每位至少 4 个、最好 10 个采样点。115200 波特的 UART 一位约 8.7 µs，几 MS/s 足够；10 MHz SPI 时钟要 50–100 MS/s 才能看清边沿。然后检查捕获时长 = 存储深度 / 采样率：100 万点、10 MS/s 只有 0.1 s，所以对罕见事件要用协议或脉宽触发加预触发存储，而不是长时间捕获。

**Follow-up 1:** What can't a logic analyser show? — Analogue effects like overshoot, ringing, slow edges and supply droop; for those I need an oscilloscope. （过冲、振铃、慢边沿、电源跌落等模拟特性，需用示波器。）

**Follow-up 2:** How do you check CAN termination quickly? — With power off, measure between CAN high and low; two 120-ohm terminators give about 60 ohms. （断电量 CAN_H 与 CAN_L，两个 120 Ω 终端并联约 60 Ω。）

### Q4. Tell me about a hardware or timing bug you solved.

**Answer.** In the RoboMaster robot, intermittent motor dropouts were a wiring and supply issue rather than a software one. I did not start by changing control code. I added a per-motor last-receive timestamp with a fifty-millisecond timeout, so a dropout relaxes the mechanism safely. Then I checked the layers from the bottom: the supply for dips, the resistance between CAN high and low with power off, which should be about sixty ohms, and the bus waveform with an oscilloscope, to see whether one motor or the whole bus was affected. Fixing the connectors and supply margin solved the cause, and the software timeout stayed as the safety net. The lesson was to debug bottom up and not tune parameters before the signals are verified.

**中文译文**：RoboMaster 机器人上间歇性的电机掉线其实是线束和供电问题，不是软件问题。我没有先改控制代码，而是为每个电机加了最后接收时间戳和 50 ms 超时，掉线时让机构安全放松；然后自下而上检查：测电源是否跌落，断电测 CAN_H 与 CAN_L 间电阻（应约 60 Ω），用示波器看总线波形，判断是单个电机还是整条总线的问题。修好接插件并留足供电余量解决了根因，软件超时保留作为安全网。（此案例按课程文档的“典型情形”整理；面试时请按你的真实经历取舍。）

**Follow-up 1:** What would you add if you did it again? — A bus error counter in the log and a hardware watchdog, so a failure is visible and recoverable. （在日志里加总线错误计数，并加硬件看门狗，使故障可见可恢复。）

**Follow-up 2:** Why do you debug bottom up? — Because a fault at a lower layer produces symptoms at every layer above it. （底层故障会在上面各层都表现为症状。）

---

## 15. 要点速记、自测题、相关链接

### 要点速记

1. SWD 两线（SWCLK、SWDIO）+ NRST + 可选 SWO；PA13/PA14 不要乱配。
2. Flash 里代码只能用硬件断点，数量有限（量级 6–8）。
3. DWT 观察点抓内存被踩；CYCCNT 测时间，$2^{32}/f_{cpu}$ 溢出，无符号减法可跨一次溢出。
4. 运行的电机系统不要用断点；用环形日志、GPIO 翻转、跟踪。
5. GPIO 翻转：宽度=执行时间，间隔=周期，间隔变化=抖动。
6. 逻辑分析仪采样 ≥ 4–10×信号速率；捕获时长 = 深度/采样率；用触发抓偶发。
7. 示波器带宽 ≥ 3–5×膝点频率 $0.35/t_r$；探头要补偿，接地线要短。
8. CAN 断电量 CAN_H–CAN_L 应约 60 Ω。
9. 五步法：复现—隔离—假设—测量—验证；自下而上分层排查。
10. 每次故障写复盘：现象、原因、处理、验证。

### 自测题

1. 为什么 Flash 中的代码断点数量有限？
2. CPU 为 400 MHz，CYCCNT 多久溢出一次？如何安全计算跨溢出的时间差？
3. 要抓 CAN 1 Mbps 总线上每 3 秒出现一次的错误帧，逻辑分析仪深度 8 M 点，应该怎么配置？
4. 信号上升沿 2 ns，示波器至少多少带宽？
5. 运行中的电机系统里想看一个变量的变化，有哪些不停机的办法？

**参考答案要点**：1. Flash 不能改写指令，只能用 FPB 比较器，比较器数量有限。2. $2^{32}/400\times10^6=10.7$ s；用无符号 32 位减法，只要间隔不超过一次溢出。3. 用错误帧或特定 ID 做协议触发，设 10 MS/s 采样（10 倍位速率），捕获时间 $8\times10^6/10^7=0.8$ s，用预触发保存触发前的数据。4. 膝点 $0.35/2\ \text{ns}=175$ MHz，需 500–900 MHz；通常 500 MHz–1 GHz 示波器。5. 日志环形缓冲、SWD 实时读内存（live watch）、ITM/SWO 输出、GPIO 编码输出、跟踪工具。

### 相关链接

- 上一章：[ADC、PWM、定时器与 FOC](./09_ADC_PWM_定时器_FOC电机控制原理.md)
- 下一章：[实时调度理论：RMS、EDF 与优先级反转](./11_实时调度理论_RMS_EDF_优先级反转.md)
- 相关：[Cortex-M 与 HardFault](./07_Cortex-M体系_启动_链接脚本_内存映射.md)、[FreeRTOS](./02_FreeRTOS实时系统.md)、[CAN 与 FDCAN](./03_CAN_FDCAN与电机通信.md)、[UART/SPI/I2C](./04_UART_SPI_I2C_以太网_TCP.md)
- 项目：[P3](../../01_项目课程/P3_RoboMaster英雄机器人嵌入式控制/README.md)、[P7](../../01_项目课程/P7_可变腿长轮腿机器人/README.md)
