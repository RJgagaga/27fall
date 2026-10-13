# RTOS 与实时调度八股

> **定位**：[并发多线程与实时系统八股](../01_计算机基础/07_并发多线程与实时系统八股.md) 与 [嵌入式与机电八股](../03_机器人学/06_嵌入式与机电八股.md) 已覆盖线程安全、FreeRTOS 基本对象与 CAN。本文偏**实时调度理论与 RTOS 内部机制**：实时性定义与指标、RMS/EDF 可调度性、优先级反转与协议、FreeRTOS/Zephyr 内核机制、中断与任务的分工、时间测量与抖动分析、设计模式与调试。对应基础知识：[FreeRTOS 实时系统](../../02_基础知识/11_嵌入式与通信/02_FreeRTOS实时系统.md)、[实时调度理论](../../02_基础知识/11_嵌入式与通信/11_实时调度理论_RMS_EDF_优先级反转.md)、[数字控制与实时性](../../02_基础知识/05_控制理论/08_数字控制与实时性.md)。

**本章学完你能……**
- 区分硬/软实时并用 WCET、抖动、截止期错过率描述实时性；
- 手算 RMS 利用率界与响应时间分析，解释 EDF、优先级反转及其协议；
- 说清 FreeRTOS 的调度、tick、上下文切换、内存与同步对象的内部机制；
- 设计中断–任务分工、控制周期与通信任务，并排查抖动与死锁；
- 用英文回答 RTOS 面试问题，并联系 P3（STM32F407 + FreeRTOS）与 P7（STM32H7 + FreeRTOS）。

**前置知识**：[嵌入式与机电八股](../03_机器人学/06_嵌入式与机电八股.md)；[并发多线程与实时系统八股](../01_计算机基础/07_并发多线程与实时系统八股.md)。

> 示例数值（周期、WCET、优先级）均为教学示例，不是项目实测。项目口径：P3 为 STM32F407 + FreeRTOS + CAN，P7 为 STM32H7 + FreeRTOS + FDCAN，具体任务划分与周期以项目文档的自查表为准。

---

## 一、实时性基础

### Q1 什么是实时系统？硬实时与软实时的区别？（Real-time systems）
**优先级：★★★** · 对应基础知识：[实时调度理论_RMS_EDF_优先级反转](../../02_基础知识/11_嵌入式与通信/11_实时调度理论_RMS_EDF_优先级反转.md)

**答案要点**
- 实时系统的正确性取决于**结果的逻辑正确性 + 产生结果的时间**。关键是“可预测”而非“快”。
- **硬实时**：错过截止期后果严重（失稳、伤人），如电机电流环、平衡控制、安全链；**软实时**：偶尔错过只降低质量，如 UI、日志；**固实时**：错过的结果无价值但不致灾。
- 同一系统常混合：控制环硬实时，通信/日志软实时 → 用优先级与隔离保证关键任务。
- 评价：最坏情况执行时间（WCET）、响应时间、抖动、截止期错过率。

**English short answer**
A real-time system is correct only if it produces the right result within a deadline, so predictability matters more than raw speed. Hard real-time deadlines, like a motor current loop or balance control, must never be missed, while soft real-time ones such as logging only degrade quality. Most robots mix both, so I give critical tasks the highest priorities and isolate them from the rest.

（译：实时系统的正确性要求在截止期内给出正确结果，可预测性比纯速度更重要；硬实时截止期（电机电流环、平衡控制）绝不能错过，软实时（日志）错过只降低质量；机器人多是两者混合，所以给关键任务最高优先级并与其他部分隔离。）

**追问方向**：为什么快不等于实时；混合系统如何隔离。

### Q2 WCET 是什么？怎么得到？（Worst-case execution time）
**优先级：★★★** · 对应基础知识：[实时调度理论_RMS_EDF_优先级反转](../../02_基础知识/11_嵌入式与通信/11_实时调度理论_RMS_EDF_优先级反转.md)

**答案要点**
- WCET：任务在最坏输入与最坏硬件状态下的执行时间上界，是可调度性分析的输入。
- 获取：①**测量**（DWT 周期计数器/引脚翻转 + 示波器，覆盖最坏分支，加余量 20%–50%）；②**静态分析**（工具分析控制流与指令时序，考虑缓存/流水线，较保守）；③**混合**。
- 影响因素：分支、循环次数、缓存命中、Flash 等待状态、总线竞争、中断抢占（计入响应时间而非执行时间）。
- MCU 优势：Cortex-M3/M4 无复杂缓存，时序较确定；M7 有缓存需谨慎。
- 实务：记录峰值执行时间并在运行时监控超限。

**English short answer**
WCET is an upper bound on a task's execution time over all inputs and hardware states and feeds schedulability analysis. I measure it with the cycle counter or pin toggling while exercising the worst paths and add margin, or use static analysis tools, which are conservative. Caches, flash wait states and bus contention add variability, and I monitor peak execution time at run time.

（译：WCET 是任务在所有输入与硬件状态下执行时间的上界，是可调度性分析的输入；用周期计数器或引脚翻转测量并覆盖最坏路径、加余量，或用较保守的静态分析工具；缓存、Flash 等待状态与总线竞争增加不确定性；运行时监控执行时间峰值。）

**追问方向**：DWT 的用法；缓存对 WCET 的影响。

### Q3 抖动（jitter）的来源与度量？（Jitter）
**优先级：★★★** · 对应基础知识：[数字控制与实时性](../../02_基础知识/05_控制理论/08_数字控制与实时性.md)

**答案要点**
- 抖动：周期任务实际启动/完成时刻相对理想时刻的偏差。控制算法假设固定采样周期，抖动等效为噪声并降低相位裕度。
- 来源：高优先级任务/中断抢占、关中断临界区、tick 分辨率、调度器开销、Flash/缓存、总线仲裁、低功耗唤醒。
- 度量：记录每次进入时间戳（硬件定时器）→ 计算周期差的最大/最小/标准差；用引脚 + 逻辑分析仪观察。
- 降低：控制循环用硬件定时器触发（中断中置信号量/任务通知唤醒最高优先级任务）、缩短临界区、关键中断高优先级、避免在控制环调用阻塞/动态内存。

**English short answer**
Jitter is the deviation of a periodic task's actual start from its ideal time, which acts like noise in control loops. It comes from preemption, interrupt-disabled sections, tick granularity and hardware effects. I measure it by timestamping each activation with a hardware timer or probing a pin with a logic analyzer, and reduce it by triggering the control task from a hardware timer, keeping critical sections short and avoiding blocking calls.

（译：抖动是周期任务实际启动时刻相对理想时刻的偏差，在控制环中等效为噪声；来源包括抢占、关中断区、tick 粒度和硬件效应；用硬件定时器打时间戳或逻辑分析仪测引脚来度量；用硬件定时器触发控制任务、缩短临界区并避免阻塞调用来降低。）

**追问方向**：抖动对相位裕度的影响；如何用逻辑分析仪测。

### Q4 响应时间、延迟与截止期的关系？（Latency, response time, deadline）
**优先级：★★** · 对应基础知识：[实时调度理论_RMS_EDF_优先级反转](../../02_基础知识/11_嵌入式与通信/11_实时调度理论_RMS_EDF_优先级反转.md)

**答案要点**
- 中断延迟：事件→ISR 首条指令（硬件+关中断时间）；调度延迟：就绪→开始运行；响应时间：事件→任务完成；截止期 $D$：必须完成的最晚时刻。
- 关系：响应时间 = 中断延迟 + 调度延迟 + 执行时间 + 被更高优先级抢占时间 + 阻塞时间。可调度条件：最坏响应时间 $R\le D$。
- 控制链路总延迟：传感器采样→滤波→控制→PWM 更新，每一环节都占预算（常按周期分配）。
- 设计：先写出时间预算表，再选周期与优先级。

**English short answer**
Interrupt latency is event to first ISR instruction, scheduling latency is ready to running, and response time is event to completion, which includes execution, preemption by higher-priority work and blocking. A task is schedulable if its worst-case response time does not exceed its deadline. I write a timing budget for the whole sensing-to-actuation chain before choosing periods and priorities.

（译：中断延迟是事件到首条 ISR 指令，调度延迟是就绪到运行，响应时间是事件到完成（含执行、被更高优先级抢占和阻塞）；任务可调度当且仅当最坏响应时间不超过截止期；选周期和优先级前先为整个传感到执行链路写时间预算。）

**追问方向**：时间预算表怎么写；阻塞时间的来源。

---

## 二、调度理论

### Q5 速率单调调度（RMS）的原理与利用率界？（Rate monotonic scheduling）
**优先级：★★★** · 对应基础知识：[实时调度理论_RMS_EDF_优先级反转](../../02_基础知识/11_嵌入式与通信/11_实时调度理论_RMS_EDF_优先级反转.md)

**答案要点**
- 固定优先级：**周期越短优先级越高**。对 $n$ 个独立周期任务（截止期=周期），若总利用率 $U=\sum C_i/T_i\le n(2^{1/n}-1)$ 则必可调度；$n\to\infty$ 界趋于 $\ln2\approx0.693$。
- 例（教学）：3 任务界 0.780；任务 (C,T)=(1,4),(1,5),(2,10) 的 $U=0.25+0.2+0.2=0.65<0.78$ → 可调度。
- 界是充分条件：超过但 $<1$ 也可能可调度，需精确的响应时间分析；谐波周期（互为整数倍）时可达 $U=1$。
- 最优性：RMS 是静态优先级中最优。局限：假设独立、无阻塞、无抖动。

**English short answer**
Rate monotonic scheduling gives higher fixed priority to shorter periods, and a set of n independent periodic tasks is guaranteed schedulable if the total utilization is at most n times two to the one over n minus one, approaching 69 percent. The bound is sufficient, not necessary, so beyond it I use exact response-time analysis, and harmonic periods can reach full utilization. It is optimal among fixed-priority schemes.

（译：RMS 给周期越短的任务越高的固定优先级，n 个独立周期任务总利用率不超过 $n(2^{1/n}-1)$（趋近 69%）即保证可调度；该界是充分而非必要条件，超出时用精确响应时间分析，谐波周期可达满利用率；它是固定优先级方案中最优的。）

**追问方向**：界的推导直觉；谐波周期为什么更好。

### Q6 响应时间分析（RTA）怎么手算？（Response-time analysis）
**优先级：★★★** · 对应基础知识：[实时调度理论_RMS_EDF_优先级反转](../../02_基础知识/11_嵌入式与通信/11_实时调度理论_RMS_EDF_优先级反转.md)

**答案要点**
- 对优先级 $i$ 的任务：$R_i=C_i+B_i+\sum_{j\in hp(i)}\left\lceil\dfrac{R_i}{T_j}\right\rceil C_j$，迭代求不动点（初值 $R=C_i$），若 $R_i\le D_i$ 则可调度。$B_i$ 为被低优先级任务阻塞的最长时间（资源共享）。
- 例（教学）：任务 A(C=1,T=4)、B(C=2,T=6)：$R_B$：迭代 $R=2\to2+\lceil2/4\rceil\cdot1=3\to2+\lceil3/4\rceil=3$ 收敛，$R_B=3\le6$。
- 中断也计入（视为最高优先级的突发任务）。
- 比利用率界精确（充要，对固定优先级）。
- 实务：用 WCET 实测值 + 余量输入。

**English short answer**
For each task I compute its worst-case response time as its own execution time plus blocking plus the interference from every higher-priority task, iterating the ceiling-sum equation to a fixed point, and check that it does not exceed the deadline. Interrupts count as highest-priority interference. It is exact for fixed-priority scheduling, unlike the utilization bound.

（译：对每个任务，最坏响应时间等于自身执行时间加阻塞时间再加所有更高优先级任务的干扰，通过迭代带上取整求和的方程到不动点，并检查不超过截止期；中断视为最高优先级干扰；对固定优先级调度它是精确的，不同于利用率界。）

**追问方向**：阻塞时间 $B_i$ 如何确定；迭代为何收敛。

### Q7 EDF 调度与 RMS 的比较？（EDF vs RMS）
**优先级：★★** · 对应基础知识：[实时调度理论_RMS_EDF_优先级反转](../../02_基础知识/11_嵌入式与通信/11_实时调度理论_RMS_EDF_优先级反转.md)

**答案要点**
- **EDF（最早截止期优先）**：动态优先级，截止期最近者先运行；单处理器上对截止期=周期的独立任务，$U\le1$ 即可调度（最优）。
- 对比：RMS 实现简单（固定优先级，主流 RTOS 原生支持），过载时低优先级任务先错过，行为可预测；EDF 利用率高，但过载时可能出现“多米诺”式连锁错过，实现需动态优先级（Linux SCHED_DEADLINE 支持）。
- 工程：MCU 控制系统几乎都用固定优先级 + RMS 思路；EDF 用于利用率紧张的软实时场景。

**English short answer**
Earliest deadline first uses dynamic priorities and can schedule any independent task set with utilization up to one on a single processor, so it is optimal. Rate monotonic uses fixed priorities, is simple and natively supported by RTOS kernels, and degrades predictably in overload because low-priority tasks miss first, whereas EDF can cascade misses. Microcontroller control systems almost always use fixed priorities.

（译：EDF 使用动态优先级，单处理器上利用率不超过 1 的独立任务集都可调度，因而是最优的；RMS 使用固定优先级、实现简单且被 RTOS 内核原生支持，过载时低优先级任务先错过、可预测，而 EDF 可能连锁错过；MCU 控制系统几乎都用固定优先级。）

**追问方向**：SCHED_DEADLINE 的参数；过载时的行为差异。

### Q8 优先级反转与优先级继承/天花板协议？（Priority inversion and protocols）
**优先级：★★★** · 对应基础知识：[实时调度理论_RMS_EDF_优先级反转](../../02_基础知识/11_嵌入式与通信/11_实时调度理论_RMS_EDF_优先级反转.md)

**答案要点**
- 场景：低优先级任务 L 持有互斥量，高优先级任务 H 等待它；中优先级任务 M 抢占 L，使 H 间接被 M 阻塞，时间不可界定（火星探路者事件的典型原因）。
- **优先级继承**：L 持锁期间继承 H 的优先级，使 M 无法抢占，L 尽快释放；FreeRTOS 互斥量（`xSemaphoreCreateMutex`）支持，二值信号量不支持。
- **优先级天花板协议**：互斥量有“天花板”（使用它的最高任务优先级），持有者立即提升到天花板；可避免死锁并限定阻塞为一个临界区。
- 其他对策：缩短临界区、避免在高优先级任务和低优先级任务间共享锁、用无锁队列/消息传递代替。

**English short answer**
Priority inversion occurs when a low-priority task holds a mutex that a high-priority task needs while a medium-priority task preempts the holder, so the high-priority task is blocked for an unbounded time. Priority inheritance temporarily raises the holder to the waiter's priority, which FreeRTOS mutexes do but binary semaphores do not. The priority ceiling protocol raises the holder to the mutex's ceiling immediately, bounding blocking and preventing deadlock. Short critical sections or message passing avoid the issue.

（译：优先级反转发生在低优先级任务持有高优先级任务所需的互斥量、同时中优先级任务抢占持有者，使高优先级任务被无界阻塞；优先级继承临时把持有者提升到等待者的优先级（FreeRTOS 互斥量支持，二值信号量不支持）；天花板协议立即把持有者提升到互斥量的天花板，限定阻塞并避免死锁；缩短临界区或用消息传递可避免。）

**追问方向**：为何二值信号量不继承；火星探路者事件。

### Q9 时间触发与事件触发架构的取舍？（Time-triggered vs event-triggered）
**优先级：★★** · 对应基础知识：[机器人系统架构设计_分层_接口_需求](../../02_基础知识/13_系统工程与安全/01_机器人系统架构设计_分层_接口_需求.md)

**答案要点**
- **时间触发**：所有活动按预定时间表（周期定时器）启动，行为确定、易验证、抖动小；缺点：对突发事件响应慢（最坏等一个周期）、资源利用率低。
- **事件触发**：外部事件/中断驱动，响应快、灵活，但并发组合不可预测、验证难。
- 机器人控制：控制环、采样用时间触发（定时器驱动）；故障、按键、通信接收用事件触发（中断置标志/队列）；混合“时间触发内核 + 事件触发外设”。
- 安全关键系统偏向时间触发以便证明可预测。

**English short answer**
Time-triggered systems start all activities from a fixed schedule, so behavior is deterministic and easy to verify, but reaction to sporadic events can take up to one period. Event-triggered systems react immediately but combinations of events are hard to predict. Robot controllers usually run sampling and control on timers and handle faults, buttons and message reception through interrupts that set flags or queue data.

（译：时间触发系统按固定时间表启动所有活动，行为确定、易验证，但对突发事件的响应最多延迟一个周期；事件触发系统反应立即，但事件组合难以预测；机器人控制器通常让采样和控制由定时器驱动，故障、按键和消息接收通过置标志或入队的中断处理。）

**追问方向**：混合架构怎么设计；安全系统为何偏好时间触发。

### Q10 多核/多周期任务的负载分配与周期选择（谐波周期）？（Harmonic periods）
**优先级：★★**

**答案要点**
- 周期取成整数倍关系（1 ms、2 ms、5 ms、10 ms、100 ms）：超周期（hyperperiod）短、调度表易构造、RMS 利用率可达 100%、日志/采样同步方便。
- 例：电流环 10 kHz（中断）、速度环 1 kHz、位置/姿态环 500 Hz–1 kHz、上层 100 Hz–50 Hz、通信/日志 10 Hz；高频到低频逐级 2–10 倍比例。
- 非谐波周期会产生任务相位漂移与偶发同时就绪；相位错开（offset）可减少峰值负载。
- 多核（如 H7 的 M7+M4，或应用核 + 实时核）：按实时性隔离，核间通信用共享内存 + 信号。

**English short answer**
I choose periods that are integer multiples of each other, such as 1, 2, 10 and 100 ms, so the hyperperiod is short, tables are easy to build, rate monotonic can reach full utilization and sensing aligns across loops. Non-harmonic periods drift in phase and occasionally coincide, so I stagger offsets to flatten peak load. On multicore parts I isolate real-time work on a dedicated core and communicate through shared memory.

（译：选取互为整数倍的周期（如 1、2、10、100 ms），使超周期短、调度表易构造、RMS 可达满利用率且各回路采样易对齐；非谐波周期会发生相位漂移并偶尔同时就绪，用错开偏移削平峰值负载；多核芯片上把实时任务隔离在专用核，经共享内存通信。）

**追问方向**：偏移（offset）如何设置；超周期的含义。

---

## 三、FreeRTOS 内核机制

### Q11 FreeRTOS 的调度器如何工作？抢占与时间片？（FreeRTOS scheduler）
**优先级：★★★** · 对应基础知识：[FreeRTOS实时系统](../../02_基础知识/11_嵌入式与通信/02_FreeRTOS实时系统.md)

**答案要点**
- 固定优先级抢占式（`configUSE_PREEMPTION=1`）：就绪的最高优先级任务运行；同优先级任务按**时间片**轮转（`configUSE_TIME_SLICING`，每个 tick 切换）。
- 任务状态：运行、就绪、阻塞（等待延时/队列/信号量）、挂起、删除。每个优先级一个就绪链表；调度时选最高非空链表的头。
- 切换触发：tick 中断、任务阻塞/让出（`taskYIELD`）、高优先级任务被唤醒（从 ISR 用 `...FromISR` 并设置 `portYIELD_FROM_ISR`）。
- Cortex-M：用 **PendSV** 做上下文切换（最低优先级异常），**SysTick** 提供 tick。
- 空闲任务（最低优先级）：回收被删任务内存、可挂低功耗。

**English short answer**
FreeRTOS uses fixed-priority preemptive scheduling: the highest-priority ready task runs, and equal-priority tasks share time in round-robin slices each tick. Each priority has a ready list, and tasks block on delays, queues or semaphores. Context switches are triggered by the tick, by blocking or yielding, or by waking a higher-priority task from an ISR, and on Cortex-M they run in the lowest-priority PendSV exception while SysTick provides the tick.

（译：FreeRTOS 采用固定优先级抢占调度：最高优先级的就绪任务运行，同优先级任务每个 tick 轮转时间片；每个优先级一个就绪链表，任务在延时、队列或信号量上阻塞；tick、阻塞/让出或从 ISR 唤醒更高优先级任务触发上下文切换，在 Cortex-M 上于最低优先级的 PendSV 异常中执行，SysTick 提供 tick。）

**追问方向**：为什么用 PendSV；时间片轮转的开关。

### Q12 FreeRTOS 的上下文切换保存了什么？开销多大？（Context switch）
**优先级：★★** · 对应基础知识：[FreeRTOS实时系统](../../02_基础知识/11_嵌入式与通信/02_FreeRTOS实时系统.md)

**答案要点**
- 异常进入时硬件自动压栈 R0–R3、R12、LR、PC、xPSR；PendSV 软件保存 R4–R11（及 FPU 寄存器 S16–S31，若使用 FPU 懒保存），把 SP 存入任务控制块（TCB）；选出下一任务，恢复其寄存器并切换 PSP。
- 每任务有独立栈；MSP 用于中断与启动，PSP 用于任务。
- 开销：Cortex-M4 约数百个周期（量级，未核对），主频 168 MHz 时约微秒级；FPU 上下文增加开销。
- 优化：减少不必要切换（合并任务）、避免频繁 yield、必要时关闭 FPU 懒保存策略的不确定性。

**English short answer**
On exception entry the hardware stacks the caller-saved registers, and PendSV then saves the remaining core registers and optionally the FPU registers, stores the stack pointer in the task control block, picks the next task and restores its context. Tasks use the process stack pointer and interrupts use the main stack pointer. The cost is on the order of hundreds of cycles, so I avoid needless switches by merging tiny tasks.

（译：异常进入时硬件自动压入调用者保存寄存器，PendSV 再保存其余核心寄存器和可选的 FPU 寄存器，把栈指针存入任务控制块，选出下一任务并恢复其上下文；任务使用进程栈指针而中断使用主栈指针；开销约数百个周期，因此合并过小的任务以避免不必要的切换。）

**追问方向**：MSP 与 PSP 的分工；FPU 懒压栈。

### Q13 `configMAX_SYSCALL_INTERRUPT_PRIORITY` 是什么？（Interrupt priority rules in FreeRTOS）
**优先级：★★★** · 对应基础知识：[FreeRTOS实时系统](../../02_基础知识/11_嵌入式与通信/02_FreeRTOS实时系统.md)

**答案要点**
- 调用 FreeRTOS `...FromISR` API 的中断，其优先级必须**不高于**（数值不小于）`configMAX_SYSCALL_INTERRUPT_PRIORITY`；高于此阈值的中断**不被内核屏蔽**（内核临界区用 BASEPRI 屏蔽到该阈值），因此不能调用任何 FreeRTOS API，但延迟最小，适合电流环/紧急保护。
- Cortex-M 优先级数值越小越高，且只有高位若干位有效（`__NVIC_PRIO_BITS`，常见 4 位），需左移后写入；FreeRTOS 要求把优先级分组设为全部抢占位（`NVIC_PRIORITYGROUP_4`）。
- 常见错误：ISR 优先级配置得比阈值更高却调用 `xQueueSendFromISR` → 断言失败或随机崩溃；忘记优先级分组。
- 配置检查：`configASSERT` 打开，在 `vPortValidateInterruptPriority` 捕获。

**English short answer**
Interrupts that call FreeRTOS FromISR functions must have a numeric priority not smaller than configMAX_SYSCALL_INTERRUPT_PRIORITY, because the kernel masks only up to that level with BASEPRI. Higher-priority interrupts are never masked and have minimum latency but cannot use any FreeRTOS API. Cortex-M uses only the upper priority bits and FreeRTOS needs all bits as preemption priority. A wrong priority triggers asserts or random crashes.

（译：调用 FreeRTOS FromISR 函数的中断，其优先级数值必须不小于 configMAX_SYSCALL_INTERRUPT_PRIORITY，因为内核用 BASEPRI 只屏蔽到该级别；更高优先级的中断不被屏蔽、延迟最小，但不能使用任何 FreeRTOS API；Cortex-M 仅使用高位优先级位，FreeRTOS 需要全部位作为抢占优先级；配置错误会触发断言或随机崩溃。）

**追问方向**：BASEPRI 的作用；优先级分组的设置。

### Q14 tick 频率怎么选？tickless 模式是什么？（Tick rate and tickless idle）
**优先级：★★** · 对应基础知识：[电源管理_低功耗_看门狗_Bootloader_OTA](../../02_基础知识/11_嵌入式与通信/08_电源管理_低功耗_看门狗_Bootloader_OTA.md)

**答案要点**
- `configTICK_RATE_HZ` 常 1000（1 ms）：决定 `vTaskDelay` 的分辨率与时间片；过高增加 tick 中断开销，过低延时粗糙。
- 延时精度：`vTaskDelay(n)` 是**相对**延时（累积误差），周期任务用 `vTaskDelayUntil` 保持固定周期；最大误差 ±1 tick。
- **Tickless idle**：空闲时停止 tick 中断，用低功耗定时器唤醒（`configUSE_TICKLESS_IDLE`），降低功耗；会引入唤醒延迟，实时控制不适合。
- 更精确定时用硬件定时器（微秒级）和任务通知，不用 tick。

**English short answer**
A tick rate of 1 kHz sets the delay resolution and time slice, trading interrupt overhead against granularity. vTaskDelay is relative and accumulates error, so periodic tasks use vTaskDelayUntil for a fixed period, and accuracy is within one tick. Tickless idle stops the tick during idle to save power at the cost of wake-up latency, which is unsuitable for hard control loops. For finer timing I use hardware timers.

（译：1 kHz 的 tick 决定延时分辨率和时间片，在中断开销与粒度间权衡；vTaskDelay 是相对延时会累积误差，周期任务用 vTaskDelayUntil 保持固定周期，精度在一个 tick 内；tickless 空闲在空闲时停止 tick 以省电，代价是唤醒延迟，不适合硬实时控制环；更精细的定时用硬件定时器。）

**追问方向**：vTaskDelay 与 vTaskDelayUntil 的区别；低功耗唤醒延迟。

### Q15 任务通知（Task Notification）为什么比信号量快？（Task notifications）
**优先级：★★★** · 对应基础知识：[FreeRTOS实时系统](../../02_基础知识/11_嵌入式与通信/02_FreeRTOS实时系统.md)

**答案要点**
- 每个任务自带一个 32 位通知值与状态，无需创建独立的内核对象，**更省 RAM、更快**（官方称比二值信号量快约 45%，量级，未核对）。
- 用法：`xTaskNotifyGive`/`ulTaskNotifyTake`（计数信号量语义）、`xTaskNotify`（按位事件或值）；ISR 中 `vTaskNotifyGiveFromISR`。典型：ISR 唤醒处理任务。
- 限制：只能**一个接收者**任务阻塞等待（不能多任务等同一对象）；发送方不会阻塞（不是队列）；通知值有限。
- 选择：单任务被中断/其他任务唤醒 → 任务通知；多对多/传数据 → 队列；多事件等待 → 事件组。

**English short answer**
Each task has a built-in 32-bit notification value, so no separate kernel object is needed, which saves RAM and is faster than a binary semaphore. It can act as a counting semaphore or an event bitmask, and an ISR can wake a handler task with it. It only supports a single receiving task and does not carry queued data, so queues or event groups are used for many-to-many or multi-event cases.

（译：每个任务内置 32 位通知值，无需单独的内核对象，节省 RAM 且比二值信号量更快；可作计数信号量或事件位掩码，ISR 可用它唤醒处理任务；只支持单个接收任务且不排队传数据，多对多或多事件场景用队列或事件组。）

**追问方向**：通知与二值信号量的语义差异；ISR 中唤醒后立即切换。

### Q16 FreeRTOS 的队列是怎么实现的？拷贝语义？（Queue implementation）
**优先级：★★** · 对应基础知识：[FreeRTOS实时系统](../../02_基础知识/11_嵌入式与通信/02_FreeRTOS实时系统.md)

**答案要点**
- 队列存储**元素的拷贝**（按值，固定大小），内部为环形缓冲 + 等待发送/接收的任务链表；发送时若有任务在等接收则直接唤醒并拷贝。
- 大数据传**指针**（注意生命周期：指向的内存要在接收方使用期间有效，可用静态缓冲/内存池，所有权随消息转移）。
- API：`xQueueSend`（阻塞超时）、`xQueueSendFromISR`、`xQueueReceive`、`xQueueOverwrite`（长度 1 的“最新值”邮箱）、`xQueuePeek`。
- 临界区：队列操作在内部短暂关中断/临界区；ISR 版本不阻塞。
- 设计：队列长度要覆盖最大突发；溢出策略（丢旧/丢新/报错计数）；用队列集合并多源。

**English short answer**
A FreeRTOS queue is a ring buffer storing copies of fixed-size items together with lists of tasks waiting to send or receive, so it copies data by value. For large payloads I pass pointers to statically allocated buffers or pool blocks and transfer ownership. The ISR variants never block, an overwrite queue of length one serves as a latest-value mailbox, and I size queues for worst-case bursts and define an overflow policy.

（译：FreeRTOS 队列是存放固定大小元素拷贝的环形缓冲，附带等待发送或接收的任务链表，因此按值拷贝数据；大数据传递指向静态缓冲或池块的指针并转移所有权；ISR 版本从不阻塞，长度为 1 的覆写队列可作为最新值邮箱；按最坏突发确定队列长度并定义溢出策略。）

**追问方向**：传指针的生命周期风险；队列集合的用途。

### Q17 FreeRTOS 的内存管理方案（heap_1~5）怎么选？（Heap schemes）
**优先级：★★★** · 对应基础知识：[FreeRTOS实时系统](../../02_基础知识/11_嵌入式与通信/02_FreeRTOS实时系统.md)

**答案要点**
- heap_1：只分配不释放，最简单确定；heap_2：可释放但不合并碎片（不推荐）；**heap_3**：包装 `malloc/free`（线程安全包装）；**heap_4**：首次适应 + **合并相邻空闲块**，最常用；heap_5：heap_4 + 多个不连续内存区域（如 H7 的 AXI SRAM + DTCM）。
- 嵌入式倾向**静态分配**（`xTaskCreateStatic`、`xQueueCreateStatic`）：编译期确定内存，无碎片与分配失败，利于认证/确定性。
- 监控：`xPortGetFreeHeapSize`、`xPortGetMinimumEverFreeHeapSize` 观察最低水位；`configUSE_MALLOC_FAILED_HOOK` 捕获失败。
- 原则：初始化阶段创建所有对象，运行期不分配/释放。

**English short answer**
Heap_1 never frees, heap_4 uses first-fit with coalescing of adjacent free blocks and is the common choice, heap_5 extends it across non-contiguous memory regions, and heap_3 wraps malloc. For determinism I prefer static allocation of tasks and queues, which avoids fragmentation and runtime allocation failure. I monitor the minimum-ever free heap and create all objects at startup, with no allocation in the control loop.

（译：heap_1 只分配不释放，heap_4 用首次适应并合并相邻空闲块，是常用选择，heap_5 将其扩展到不连续内存区域，heap_3 包装 malloc；为获得确定性更偏好任务与队列的静态分配，避免碎片和运行期分配失败；监控历史最低空闲堆，并在启动时创建所有对象，控制环内不分配。）

**追问方向**：碎片化的表现；静态创建 API 的使用。

### Q18 栈大小怎么估计？高水位线怎么用？（Task stack sizing）
**优先级：★★★** · 对应基础知识：[FreeRTOS实时系统](../../02_基础知识/11_嵌入式与通信/02_FreeRTOS实时系统.md)

**答案要点**
- 栈内容：局部变量、调用链返回地址与保存寄存器、中断嵌套帧（异常入栈 8 字 + FPU 帧 26 字）、`printf` 家族（大栈需求）。
- 估计：静态（`-fstack-usage` 的 `.su` 文件求调用链最大和）+ 实测 `uxTaskGetStackHighWaterMark`（历史最小剩余字数）+ 余量 25%–50%。
- 检测：`configCHECK_FOR_STACK_OVERFLOW`=2（栈底填充模式检查）+ 钩子；MPU 栈底保护区（M4/M7）即时触发。
- 启动时填充 `0xA5` 模式，调试器查看已用深度。
- 经验：控制任务栈 512–1024 字（教学示例）；用到浮点 `printf`/日志格式化的任务应更大。

**English short answer**
Stack use includes locals, call chains, saved registers, interrupt frames and heavy library calls like printf. I combine static analysis of per-function stack usage with the measured high-water mark and add 25 to 50 percent margin. Overflow is detected by the kernel's fill-pattern check with a hook, or by an MPU guard region that faults immediately.

（译：栈使用包括局部变量、调用链、保存寄存器、中断帧和 printf 等重量级库调用；结合逐函数栈用量的静态分析与实测高水位线并加 25%～50% 余量；溢出由内核的填充模式检查加钩子检测，或由 MPU 保护区立即触发 fault。）

**追问方向**：中断帧在哪个栈上；水位线单位。

### Q19 FreeRTOS 的软件定时器任务是什么？有什么坑？（Software timers）
**优先级：★★** · 对应基础知识：[FreeRTOS实时系统](../../02_基础知识/11_嵌入式与通信/02_FreeRTOS实时系统.md)

**答案要点**
- 软件定时器由一个**守护任务（Timer Service Task）**执行回调，指令通过命令队列发送；回调在该任务上下文运行，**不能阻塞**，且受该任务优先级影响（需设置得足够高）。
- 精度受 tick 限制；回调执行时间长会延迟其他定时器。
- 与硬件定时器区别：硬件定时器中断精确、抖动小，适合控制环；软件定时器适合超时、心跳、非关键周期事件。
- 坑：在回调里调用阻塞 API；命令队列满导致 `xTimerStart` 失败；定时器优先级太低使回调延迟。

**English short answer**
FreeRTOS software timers run their callbacks in a daemon task that receives commands through a queue, so callbacks must not block, their accuracy is limited by the tick, and they depend on the daemon task's priority. They suit timeouts and heartbeats, whereas control loops need hardware timers for low jitter. A full command queue or low daemon priority delays or drops timer operations.

（译：FreeRTOS 软件定时器的回调在经队列接收命令的守护任务中运行，因此回调不能阻塞、精度受 tick 限制，并依赖守护任务的优先级；适合超时与心跳，控制环则需要低抖动的硬件定时器；命令队列满或守护任务优先级低会延迟或丢失定时器操作。）

**追问方向**：回调中的限制；命令队列长度配置。

### Q20 事件组（Event Group）和流缓冲区/消息缓冲区的用途？（Event groups and stream buffers）
**优先级：★★** · 对应基础知识：[FreeRTOS实时系统](../../02_基础知识/11_嵌入式与通信/02_FreeRTOS实时系统.md)

**答案要点**
- **事件组**：一个整数的多位标志；任务可等待“任意/全部”位被置位（`xEventGroupWaitBits`），支持同步点（`xEventGroupSync`）；适合多条件汇合（如“传感器就绪 AND 通信就绪”）。ISR 置位用 `...FromISR`（经守护任务延后执行）。
- **流缓冲区**：字节流，单生产者单消费者，适合 UART 接收流；**消息缓冲区**：保留消息边界。
- 限制：事件组位数有限（24 位），事件组操作在临界区较长；流缓冲区只允许一个写者一个读者（多写需加锁）。
- 选择：多条件同步 → 事件组；不定长串口数据 → 流缓冲区 + DMA 空闲中断。

**English short answer**
An event group holds several flag bits that tasks can wait on with any-or-all semantics, ideal for joining multiple readiness conditions or rendezvous synchronization. A stream buffer carries a byte stream between a single writer and single reader, suiting UART data, while a message buffer preserves message boundaries. Event groups have limited bits, and stream buffers are not safe for multiple writers without extra locking.

（译：事件组保存若干标志位，任务可用“任意/全部”语义等待，适合汇合多个就绪条件和会合同步；流缓冲区在单写者和单读者间传递字节流，适合 UART 数据，消息缓冲区则保留消息边界；事件组位数有限，流缓冲区在多写者时需额外加锁。）

**追问方向**：事件组 ISR 置位为何延后；流缓冲触发级别。

### Q21 临界区、调度器挂起与关中断的区别？（Critical sections vs scheduler suspend）
**优先级：★★★** · 对应基础知识：[FreeRTOS实时系统](../../02_基础知识/11_嵌入式与通信/02_FreeRTOS实时系统.md)

**答案要点**
- `taskENTER_CRITICAL`：屏蔽到 `configMAX_SYSCALL_INTERRUPT_PRIORITY` 以下的中断（BASEPRI），任务不会被切换也不会被受管中断打断；时间必须极短（微秒级），否则抖动大；可嵌套；在 ISR 中用 `taskENTER_CRITICAL_FROM_ISR`。
- `vTaskSuspendAll`：只挂起调度器（不关中断），中断仍运行，但任务切换被推迟；ISR 里 `...FromISR` 唤醒的任务延后到恢复后；适合较长的非原子操作，但不能调用阻塞 API。
- 完全关中断（`__disable_irq`）：连高优先级中断也屏蔽，仅用于极短的硬件序列。
- 选择：保护与 ISR 共享的小段数据 → 临界区；保护只与任务共享的较长段 → 互斥量；不阻塞其他任务的长操作 → 调度器挂起（谨慎）。

**English short answer**
A FreeRTOS critical section raises BASEPRI to mask managed interrupts and prevents task switches, so it must last microseconds and protects data shared with ISRs. Suspending the scheduler leaves interrupts running but defers task switches, allowing longer operations that must not call blocking APIs. Fully disabling interrupts also masks high-priority ones and is reserved for the shortest hardware sequences. For long task-only sections I use a mutex.

（译：FreeRTOS 临界区提升 BASEPRI 屏蔽受管中断并阻止任务切换，必须只持续微秒级，用于保护与 ISR 共享的数据；挂起调度器不关中断但推迟任务切换，允许较长操作但不得调用阻塞 API；完全关中断连高优先级中断也屏蔽，只用于最短的硬件序列；仅任务间共享的长区段用互斥量。）

**追问方向**：临界区最长允许时间；调度器挂起期间 ISR 唤醒的任务何时运行。

### Q22 FreeRTOS 的 `configASSERT` 与 Hook 函数该如何使用？（Assertions and hooks）
**优先级：★★** · 对应基础知识：[嵌入式调试_JTAG_SWD_逻辑分析仪_故障定位](../../02_基础知识/11_嵌入式与通信/10_嵌入式调试_JTAG_SWD_逻辑分析仪_故障定位.md)

**答案要点**
- `configASSERT(x)`：开发期开启，失败时记录文件/行并关中断死循环或触发调试断点；发布版可改为记录并**安全复位**。
- Hook：`vApplicationStackOverflowHook`（栈溢出）、`vApplicationMallocFailedHook`（分配失败）、`vApplicationIdleHook`（空闲，可喂狗/休眠）、`vApplicationTickHook`（每 tick，须极短且只用 ISR 版 API）。
- 实务：在 hook 中把故障原因写入保留 RAM 或备份寄存器，复位后读出上报；电机输出先置安全态（失能）再复位。
- 避免在 Hook 里做复杂/阻塞操作。

**English short answer**
configASSERT catches misconfiguration and kernel misuse during development, and in release builds it should record the location and trigger a safe reset. Hooks for stack overflow, allocation failure, idle and tick let me react to faults: I store the cause in retained memory, put the motors in a safe state and reset, and keep hook code short and non-blocking.

（译：configASSERT 在开发期捕获配置错误和内核误用，发布版应记录位置并触发安全复位；栈溢出、分配失败、空闲和 tick 钩子让我对故障做出反应：把原因存入保留内存、先使电机进入安全态再复位，并保持钩子代码短且不阻塞。）

**追问方向**：保留 RAM 的实现；Idle hook 里喂狗的风险。

---

## 四、中断与任务的分工

### Q23 ISR 里应该做什么、不应该做什么？（ISR design rules）
**优先级：★★★** · 对应基础知识：[STM32_时钟_GPIO_中断_DMA](../../02_基础知识/11_嵌入式与通信/01_STM32_时钟_GPIO_中断_DMA.md)

**答案要点**
- 做：清中断标志、读/写必要寄存器、记录时间戳、把数据放入队列/环形缓冲、通过 `FromISR` API 通知任务、少量计算（控制环里电流环可在 ISR）。
- 不做：阻塞/等待、`printf`、动态内存分配、长循环、浮点密集计算（若 FPU 上下文未保存）、调用非 ISR 安全 API、持锁。
- “下半部”模式：ISR（上半部）极短，重活交给任务（下半部）；保证不丢事件：计数信号量/队列缓冲，或 DMA + 环形缓冲。
- 共享变量：`volatile` + 原子/临界区；多字数据用双缓冲/序号。
- 测量 ISR 最长执行时间并监控。

**English short answer**
An ISR should clear the interrupt source, grab data or a timestamp, hand it to a task through a queue, notification or ring buffer, and return quickly. It must not block, allocate memory, print or run long computations. This top-half/bottom-half split keeps latency low for other interrupts, and shared variables need volatile plus atomic or critical-section protection. I measure the longest ISR execution time.

（译：ISR 应清除中断源、取得数据或时间戳，通过队列、通知或环形缓冲交给任务并快速返回；不得阻塞、分配内存、打印或做长计算；这种上下半部划分保证其他中断的低延迟，共享变量需 volatile 加原子或临界区保护；测量 ISR 最长执行时间。）

**追问方向**：多字数据的一致性；ISR 中能否用浮点。

### Q24 控制环放在 ISR 里还是任务里？（Control loop in ISR vs task）
**优先级：★★★** · 对应基础知识：[数字控制与实时性](../../02_基础知识/05_控制理论/08_数字控制与实时性.md)

**答案要点**
- **ISR 内**：抖动最小、延迟确定，适合最内层高频电流环（10–20 kHz）；但 ISR 不能运行太久，占用 CPU 影响其他中断，难以使用 RTOS API。
- **高优先级任务**（由硬件定时器中断通过通知唤醒）：适合 1 kHz 左右的速度/姿态环，代码可使用 RTOS 对象与日志，调试方便；抖动略大（调度延迟）。
- 典型分层：电流环 ISR（硬件定时器/ADC 触发）→ 速度/平衡环高优先级任务 → 位置/规划低优先级任务 → 通信/日志最低。
- 判据：周期 < 约 200 µs → ISR；≥ 1 ms → 任务；中间视 CPU 余量与抖动容忍度。
- P7 口径：平衡控制周期由定时器触发（具体周期以项目自查表为准）。

**English short answer**
Putting the loop in an ISR gives minimal jitter and fits the innermost current loop at tens of kilohertz, but it blocks other interrupts and cannot use most RTOS services. A high-priority task woken by a hardware timer through a notification suits kilohertz speed or balance loops, with easier debugging at slightly higher jitter. I layer them: current loop in ISR, speed and balance in a high-priority task, planning and communication below.

（译：把回路放在 ISR 中抖动最小，适合几十 kHz 的最内层电流环，但会阻塞其他中断且不能使用多数 RTOS 服务；由硬件定时器经通知唤醒的高优先级任务适合 kHz 级的速度或平衡环，调试更容易但抖动略大；采用分层：电流环在 ISR、速度与平衡环在高优先级任务、规划与通信更低。）

**追问方向**：任务唤醒的延迟量级；ISR 占用率的上限。

### Q25 如何设计 DMA + 空闲中断的 UART 接收？（UART DMA idle-line reception）
**优先级：★★★** · 对应基础知识：[UART_SPI_I2C_以太网_TCP](../../02_基础知识/11_嵌入式与通信/04_UART_SPI_I2C_以太网_TCP.md)

**答案要点**
- DMA 循环模式把串口数据写入环形缓冲；**IDLE 中断**（总线空闲一帧时间）触发后，读取 DMA 计数器得到已接收字节数，计算新数据区间（处理回绕），通过任务通知或流缓冲区交给解析任务。
- 加半传输/传输完成中断防止高吞吐时漏判；解析任务按协议（帧头、长度、CRC）拆包，处理粘包/半包。
- H7 上 DMA 缓冲区需在不可缓存区或做 cache invalidate，并按缓存行对齐。
- 错误处理：溢出（ORE）、帧错误、噪声；超时重同步。
- 优点：不丢数据、CPU 占用低、支持不定长帧。

**English short answer**
DMA fills a circular buffer from the UART, and the idle-line interrupt fires when the bus goes quiet, at which point I read the DMA counter to compute the new bytes, handling wrap-around, and notify a parser task. Half- and full-transfer interrupts cover long continuous streams. The parser handles sticky and partial frames using header, length and CRC. On Cortex-M7 the buffer must be non-cacheable or invalidated.

（译：DMA 把 UART 数据填入循环缓冲，总线静默时触发空闲线中断，此时读取 DMA 计数器计算新增字节（处理回绕）并通知解析任务；半传输和传输完成中断覆盖长连续数据流；解析任务用帧头、长度和 CRC 处理粘包半包；在 Cortex-M7 上缓冲区须不可缓存或做 invalidate。）

**追问方向**：回绕的处理；粘包半包的状态机。

### Q26 如何设计从中断到任务的“最新值”传递（避免读到半更新数据）？（Latest-value handoff）
**优先级：★★★** · 对应基础知识：[STM32_时钟_GPIO_中断_DMA](../../02_基础知识/11_嵌入式与通信/01_STM32_时钟_GPIO_中断_DMA.md)

**答案要点**
- 问题：多字段数据（位姿+时间戳）被 ISR 更新时任务读取，可能读到“半新半旧”。
- 方案：①**临界区**拷贝（短）；②**双缓冲 + 序号（seqlock）**：写者先将序号置奇、写数据、再置偶；读者读序号、读数据、再读序号，不一致或为奇则重读；③**队列长度 1 覆写**（`xQueueOverwrite` + `xQueuePeek`）；④无锁 SPSC 环形缓冲。
- 选择：数据小（<16 字节）→ 临界区；频繁更新、读者多 → seqlock/覆写队列。
- 加时间戳，读者可判断数据新鲜度并在过期时进入安全状态。

**English short answer**
Multi-field data can be read half-updated while an ISR writes it. I either copy it inside a very short critical section, use a sequence counter so readers retry if the counter changed or is odd, or use a length-one overwrite queue that always holds the latest value. A timestamp lets the reader detect stale data and fall back to a safe state.

（译：多字段数据可能在 ISR 写入时被读到半更新状态；可以在极短的临界区内拷贝，或用序号计数器使读者在计数变化或为奇数时重读，或用长度为 1 的覆写队列始终保存最新值；时间戳让读者检测过期数据并回退到安全状态。）

**追问方向**：seqlock 的内存序要求；覆写队列与 peek 的语义。

### Q27 周期任务实现：`vTaskDelayUntil` vs 定时器触发？（Periodic task implementation）
**优先级：★★★** · 对应基础知识：[FreeRTOS实时系统](../../02_基础知识/11_嵌入式与通信/02_FreeRTOS实时系统.md)

**答案要点**
- `vTaskDelayUntil(&last, period)`：以上次唤醒时刻为基准累加，避免累积漂移；周期受 tick 分辨率限制（1 ms）。若任务执行超过周期，则下次立即运行（可检测返回值判断错过）。
- **硬件定时器触发**：ISR 里 `vTaskNotifyGiveFromISR` 唤醒任务；周期精确（微秒级），可使用 100 µs 等低于 tick 的周期；需要注意 ISR 优先级配置与 `portYIELD_FROM_ISR`。
- 检测 overrun：记录开始时间戳，测执行时间；若通知计数 >1 表示漏了周期；连续超时进入降级/报警。
- 选择：≥ 1 ms 非严格周期 → `vTaskDelayUntil`；严格或 <1 ms → 硬件定时器 + 通知。

**English short answer**
vTaskDelayUntil schedules wake-ups from the previous wake time, avoiding drift, but is limited to tick resolution. A hardware timer ISR that gives a task notification yields microsecond-accurate and sub-tick periods. To detect overruns I check the notification count or the return value and the measured execution time, and react to repeated overruns with degradation or an alarm.

（译：vTaskDelayUntil 从上次唤醒时刻起计划下次唤醒以避免漂移，但受 tick 分辨率限制；硬件定时器 ISR 发送任务通知可获得微秒级精度和低于 tick 的周期；通过通知计数或返回值及实测执行时间检测超时，对重复超时采取降级或报警。）

**追问方向**：通知计数大于 1 的含义；超时降级策略。

### Q28 中断嵌套与优先级分配的实际方案？（Interrupt priority assignment）
**优先级：★★** · 对应基础知识：[STM32_时钟_GPIO_中断_DMA](../../02_基础知识/11_嵌入式与通信/01_STM32_时钟_GPIO_中断_DMA.md)

**答案要点**
- 原则：时间越紧迫的中断优先级越高；最内层控制定时器/ADC 中断最高（可高于 FreeRTOS 阈值，不调用 API）；然后是 DMA/通信中断；最后是普通外设；SysTick 和 PendSV 在 FreeRTOS 中设为**最低**。
- 分组：全部为抢占优先级（无子优先级）；限制嵌套深度，避免栈使用过多。
- 示例（教学）：TIM1 电流环 0；CAN/FDCAN RX 5；UART DMA 6；SPI IMU 数据就绪 4；其余 8–15；FreeRTOS 阈值 5（0–4 不可调用 API）。
- 验证：压力测试（所有中断同时触发）、测量最长中断响应、检查是否有中断饿死。

**English short answer**
The more time-critical an interrupt, the higher its priority: the innermost control timer or ADC interrupt on top, possibly above the FreeRTOS syscall threshold with no API calls, then DMA and communication, then ordinary peripherals, with SysTick and PendSV at the lowest. I use only preemption priorities, limit nesting depth to bound stack use, and stress-test by triggering everything at once.

（译：越紧迫的中断优先级越高：最内层控制定时器或 ADC 中断最高（可高于 FreeRTOS 系统调用阈值且不调用 API），其次是 DMA 与通信，再是普通外设，SysTick 和 PendSV 最低；只用抢占优先级，限制嵌套深度以界定栈使用，并通过同时触发所有中断做压力测试。）

**追问方向**：为什么 PendSV 最低；中断饿死的检测。

---

## 五、同步、死锁与设计模式

### Q29 互斥量使用的常见错误？（Mutex pitfalls）
**优先级：★★★** · 对应基础知识：[FreeRTOS实时系统](../../02_基础知识/11_嵌入式与通信/02_FreeRTOS实时系统.md)

**答案要点**
- 在 ISR 里使用互斥量（不允许，互斥量需任务上下文且有所有权）；持锁时调用可能阻塞的 API；忘记释放（异常/提前 return 路径）→ 用 RAII（C++）或统一出口；递归获取需用递归互斥量；锁的顺序不一致 → 死锁。
- 临界区过长使高优先级任务延迟；在高优先级任务里等待低优先级任务持有的锁（优先级反转，需继承）。
- 用二值信号量当互斥量（无继承）。
- 对策：最小化锁范围、固定加锁顺序、超时获取（`xSemaphoreTake(.., timeout)`）并处理失败、优先使用消息传递（单一所有者任务）。

**English short answer**
Mutexes cannot be used in ISRs, must be released on every exit path, should not be held across blocking calls, and recursive acquisition needs a recursive mutex. Inconsistent lock ordering causes deadlock and a binary semaphore used as a mutex lacks priority inheritance. I keep critical sections minimal, fix the lock order, take locks with timeouts and handle failure, and prefer a single owner task with message passing.

（译：互斥量不能用于 ISR，必须在每个退出路径释放，不应在阻塞调用期间持有，递归获取需要递归互斥量；加锁顺序不一致造成死锁，把二值信号量当互斥量会缺少优先级继承；保持临界区最小、固定加锁顺序、带超时获取并处理失败，并优先采用单一所有者任务加消息传递。）

**追问方向**：RAII 锁封装；超时获取失败怎么处理。

### Q30 死锁的四个条件与嵌入式中的避免方法？（Deadlock avoidance）
**优先级：★★★** · 对应基础知识：[实时调度理论_RMS_EDF_优先级反转](../../02_基础知识/11_嵌入式与通信/11_实时调度理论_RMS_EDF_优先级反转.md)

**答案要点**
- 条件：互斥、持有并等待、不可剥夺、循环等待。破坏任一即可：①固定**全局加锁顺序**（破坏循环等待）；②一次性申请所有资源或用 `try-lock` + 回退（破坏持有并等待）；③带**超时**（打破僵局）；④减少共享资源，用单线程所有权。
- 诊断：任务全部阻塞、看门狗复位；用 RTOS 感知调试器查看各任务阻塞对象与持有者；记录获取/释放日志。
- 看门狗兜底：任务各自上报心跳，死锁时系统看门狗不被喂 → 复位到安全状态。
- 优先级反转与死锁区别：前者是调度问题，后者是资源环路。

**English short answer**
Deadlock needs mutual exclusion, hold-and-wait, no preemption and circular wait; I break it with a global lock order, try-lock with backoff or timeouts, and by reducing shared resources with single-owner tasks. I diagnose with an RTOS-aware debugger that shows what each blocked task waits on, and rely on a watchdog fed only when all tasks report heartbeats to recover safely.

（译：死锁需要互斥、持有并等待、不可剥夺和循环等待四个条件，通过全局加锁顺序、带退避的 try-lock 或超时以及用单一所有者任务减少共享资源来打破；用 RTOS 感知调试器查看各阻塞任务的等待对象来诊断，并依赖仅在所有任务上报心跳时才喂的看门狗安全恢复。）

**追问方向**：任务心跳看门狗设计；日志如何定位环路。

### Q31 任务心跳与分层看门狗设计？（Task heartbeats and watchdog design）
**优先级：★★★** · 对应基础知识：[可靠性与故障诊断_FMEA_看门狗_降级策略](../../02_基础知识/13_系统工程与安全/03_可靠性与故障诊断_FMEA_看门狗_降级策略.md)

**答案要点**
- 问题：只在空闲任务或某个任务喂狗，无法发现其他关键任务卡死。
- 设计：每个关键任务周期性设置自己的“存活位”（事件组或计数器）；一个**监控任务**（或定时器）定期检查所有位都已置位且执行时间合理，**全部正常才喂硬件看门狗**，并清除标志；任何任务超时 → 不喂狗 → 复位，同时先把电机输出置安全态。
- 窗口看门狗（WWDG）要求喂狗时间落在窗口内，防止“跑飞后乱喂”。
- 复位原因记录：读复位标志（IWDG/WWDG/BOR/软复位），把任务状态快照写入保留 RAM。
- 超时设置：略大于最长正常周期（如控制周期 ×3–10）；调试时可暂停看门狗（调试冻结位）。

**English short answer**
Each critical task sets an alive flag periodically, and a supervisor task feeds the hardware watchdog only when all flags are set and timing is plausible, clearing them afterward. Any missing heartbeat lets the watchdog reset the MCU after the outputs are put in a safe state. A windowed watchdog rejects feeding at the wrong time, and I log the reset cause and a task snapshot in retained memory.

（译：每个关键任务周期性置存活标志，监督任务仅在所有标志已置位且时序合理时才喂硬件看门狗并随后清除标志；任何心跳缺失都会让看门狗在输出被置于安全态后复位 MCU；窗口看门狗拒绝在错误时刻喂狗，并把复位原因与任务快照记录在保留内存中。）

**追问方向**：窗口看门狗的窗口怎么设；复位后如何安全恢复。

### Q32 如何设计消息驱动的“单一所有者”任务架构？（Actor-style design）
**优先级：★★** · 对应基础知识：[机器人系统架构设计_分层_接口_需求](../../02_基础知识/13_系统工程与安全/01_机器人系统架构设计_分层_接口_需求.md)

**答案要点**
- 每个共享资源（CAN 总线、IMU 驱动、状态机）由**一个任务独占**，其他任务通过队列向其发请求/命令，所有者顺序处理 → 不需要锁，避免死锁与优先级反转。
- 消息设计：带类型、序号、时间戳、来源；响应用回复队列/通知；队列满策略明确。
- 优点：易推理、易测试（可回放消息）、职责清晰；缺点：消息拷贝开销、延迟、需要处理请求超时。
- 机器人应用：电机管理任务统一收发 CAN；状态机任务集中处理模式切换与故障；日志任务只负责落盘。
- 与共享内存+锁对比：高频小数据读（传感器最新值）用覆写队列/seqlock 更合适。

**English short answer**
Each shared resource is owned by one task that processes requests from a queue sequentially, so no locks are needed and deadlock and priority inversion disappear. Messages carry type, sequence number and timestamp, replies come via notifications or reply queues, and queue-full policies are explicit. It is easy to reason about and test by replaying messages, at the cost of copying and latency, and high-rate reads still use latest-value mailboxes.

（译：每个共享资源由一个任务独占并顺序处理队列中的请求，无需锁，死锁和优先级反转随之消失；消息带类型、序号和时间戳，回复通过通知或回复队列，队列满策略明确；易于推理并可通过回放消息测试，代价是拷贝与延迟，高频读取仍用最新值邮箱。）

**追问方向**：请求超时与重试；日志任务的背压。

### Q33 状态机任务的设计要点（模式管理）？（State machine task for mode management）
**优先级：★★★** · 对应基础知识：[机器人系统架构设计_分层_接口_需求](../../02_基础知识/13_系统工程与安全/01_机器人系统架构设计_分层_接口_需求.md)

**答案要点**
- 状态：上电自检 → 待机（失能）→ 使能待命 → 运行（各子模式）→ 故障（保护）→ 复位恢复；转移由事件（命令、传感器有效性、故障、超时）驱动。
- 原则：所有转移集中在**一个状态机任务**；每次转移在入口/出口执行确定动作（使能/失能电机、清积分、重置滤波器）；任意状态收到**急停/严重故障**立即转入故障状态（最高优先）。
- 防护：非法事件忽略并记录；故障状态只能经人工确认+自检后恢复；状态与转移原因写入日志。
- 与控制任务交互：状态机输出“模式命令”给控制任务；控制任务不自行决定模式。
- 测试：转移表全覆盖，仿真注入故障。

**English short answer**
I centralize mode transitions in one state machine task with states like self-test, standby, enabled, running and fault, driven by events such as commands, sensor validity, faults and timeouts. Each transition performs defined entry and exit actions such as enabling or disabling motors and resetting integrators, any state can jump to fault on an emergency event, illegal events are ignored and logged, and recovery from fault requires confirmation and a self-check.

（译：把模式转移集中在一个状态机任务中，状态包括自检、待机、使能、运行和故障，由命令、传感器有效性、故障和超时等事件驱动；每次转移执行确定的入口/出口动作（如使能或失能电机、重置积分器），任何状态遇到紧急事件都可跳到故障态，非法事件被忽略并记录，从故障恢复需确认和自检。）

**追问方向**：入口出口动作示例；故障恢复条件。

### Q34 日志在实时任务里怎么写才不影响控制？（Non-intrusive logging）
**优先级：★★★** · 对应基础知识：[数据与日志_可观测性_回放与复盘](../../02_基础知识/13_系统工程与安全/05_数据与日志_可观测性_回放与复盘.md)

**答案要点**
- 控制任务只写**无锁环形缓冲**或固定大小二进制记录（时间戳 + 事件码 + 数据），不格式化字符串、不调用 `printf`；
- 低优先级日志任务从缓冲取出，批量格式化后经 UART/USB/SD 输出（带宽有限，用 DMA）；缓冲满则丢弃并计数“丢失条数”。
- 关键事件（故障、模式切换）单独优先通道；日志时间戳用硬件定时器。
- 离线解析二进制日志（工具脚本），体积小、对实时影响小。
- 对比：`printf` 可能花费毫秒、使用大栈、阻塞 I/O。

**English short answer**
The control task writes fixed-size binary records with a timestamp and event code into a lock-free ring buffer and never formats strings or prints. A low-priority logging task drains the buffer, formats in batches and sends it over DMA-driven UART, USB or SD, and counts dropped records when the buffer fills. Critical events get a priority channel, and an offline tool decodes the binary log.

（译：控制任务把带时间戳和事件码的定长二进制记录写入无锁环形缓冲，不格式化字符串也不打印；低优先级日志任务取出缓冲、批量格式化并通过 DMA 驱动的 UART、USB 或 SD 输出，缓冲满时统计丢弃条数；关键事件走优先通道，用离线工具解码二进制日志。）

**追问方向**：二进制日志格式设计；日志丢失如何评估。

---

## 六、其他 RTOS 与 Linux 实时

### Q35 Zephyr、RT-Thread、ThreadX 与 FreeRTOS 的差异？（RTOS comparison）
**优先级：★★**

**答案要点**
- **FreeRTOS**：极简内核（任务/队列/信号量/定时器），生态广、移植简单、MIT 许可；驱动/网络栈靠第三方（HAL、lwIP）。
- **Zephyr**：完整平台（设备树、驱动模型、网络/蓝牙栈、电源管理、构建系统 west），Linux 基金会项目，Apache-2.0，支持板级广；学习曲线陡，配置复杂。
- **RT-Thread**：国产 RTOS，组件丰富（设备框架、FinSH 命令行、软件包），中文社区活跃。**ThreadX（Azure RTOS）**：确定性高、商业认证背景，后捐给 Eclipse 基金会（许可细节未核对）。
- 选择：轻量/快速移植/已有 HAL → FreeRTOS；复杂连接/多板支持/长期产品 → Zephyr；国内生态/组件 → RT-Thread。
- 面试：能说出各自一个特点即可，强调你实际用的是 FreeRTOS。

**English short answer**
FreeRTOS is a minimal, widely ported kernel under an MIT license, leaving drivers and stacks to third parties. Zephyr is a full platform with device tree, driver model, networking and power management, powerful but with a steeper learning curve. RT-Thread offers a rich component ecosystem with a strong Chinese community, and ThreadX is known for deterministic behavior and certification history. I choose by complexity of connectivity and board support, and I have used FreeRTOS.

（译：FreeRTOS 是极简、移植广泛、MIT 许可的内核，驱动与协议栈靠第三方；Zephyr 是带设备树、驱动模型、网络与电源管理的完整平台，强大但学习曲线陡；RT-Thread 组件生态丰富、中文社区活跃；ThreadX 以确定性与认证背景著称；按连接复杂度与板级支持选择，我实际使用的是 FreeRTOS。）

**追问方向**：设备树对移植的价值；FreeRTOS 的许可。

### Q36 PREEMPT_RT 如何让 Linux 具备实时性？（PREEMPT_RT）
**优先级：★★★** · 对应基础知识：[嵌入式Linux_设备树_驱动_RT-Preempt](../../02_基础知识/11_嵌入式与通信/12_嵌入式Linux_设备树_驱动_RT-Preempt.md)

**答案要点**
- 标准 Linux 内核许多路径不可抢占（自旋锁、中断处理），延迟毫秒级不确定。PREEMPT_RT 补丁使内核**几乎完全可抢占**：自旋锁改为可睡眠的 rt-mutex（带优先级继承）、中断处理线程化（硬中断只做最小工作）、高精度定时器。
- 用户空间：`SCHED_FIFO/SCHED_RR` 实时优先级（1–99），`mlockall` 锁内存，预分配，避免缺页，绑核（`isolcpus`、`taskset`），关闭频率调节与省电，使用 `clock_nanosleep(TIMER_ABSTIME)` 做周期。
- 测量：`cyclictest` 测调度延迟分布（关注最大值，需压力负载下跑数小时），`ftrace`/`trace-cmd` 定位长延迟。
- 典型结果：树莓派/ARM 上最坏延迟几十到一百多微秒量级（依硬件，未核对）。
- 限制：不是硬实时认证级别；驱动质量、SMI/热降频仍影响；极严苛控制仍交给 MCU。

**English short answer**
PREEMPT_RT makes nearly all kernel code preemptible by turning spinlocks into priority-inheriting sleeping mutexes and moving interrupt handling into threads. In user space I use SCHED_FIFO priorities, lock and preallocate memory, pin threads to isolated cores and sleep until absolute times. I validate with cyclictest under load for hours and look at the worst-case latency. It gives good soft-to-firm real-time performance, while the tightest loops stay on a microcontroller.

（译：PREEMPT_RT 通过把自旋锁改为带优先级继承的可睡眠互斥量、并将中断处理线程化，使几乎所有内核代码可抢占；用户空间使用 SCHED_FIFO 优先级、锁定并预分配内存、将线程绑到隔离核并睡眠到绝对时间；在负载下运行 cyclictest 数小时并看最坏延迟来验证；它提供不错的软到固实时性能，最严苛的回路仍交给 MCU。）

**追问方向**：为什么 cyclictest 看最大值；中断线程化的含义。

### Q37 Linux 上的实时线程应如何配置？（Configuring real-time threads on Linux）
**优先级：★★★** · 对应基础知识：[嵌入式Linux_设备树_驱动_RT-Preempt](../../02_基础知识/11_嵌入式与通信/12_嵌入式Linux_设备树_驱动_RT-Preempt.md)

**答案要点**
- 步骤：①`sched_setscheduler`/`pthread_setschedparam` 设 `SCHED_FIFO` 与优先级（高于一般，但低于内核关键线程如 `irq/` 线程视需要）；②`mlockall(MCL_CURRENT|MCL_FUTURE)`；③预分配并预触碰栈/堆；④用 `clock_nanosleep(CLOCK_MONOTONIC, TIMER_ABSTIME)` 做周期并累加绝对时间；⑤绑核 `pthread_setaffinity_np` + 隔离核；⑥避免阻塞 I/O、动态分配、锁竞争（用无锁/优先级继承互斥量 `PTHREAD_PRIO_INHERIT`）；⑦权限（`CAP_SYS_NICE` 或 `rtprio` 限制）。
- 风险：高优先级死循环会饿死系统（设置 `sched_rt_runtime_us` 限额、看门狗）。
- ROS 2：`rclcpp` 执行器线程可设置实时属性，控制节点（如 `ros2_control` 的控制管理器）在实时线程中运行。
- 验证：测量周期抖动与 overrun。

**English short answer**
I set SCHED_FIFO with a suitable priority, lock memory with mlockall, prefault stack and heap, sleep until absolute times on the monotonic clock, pin the thread to an isolated core and avoid blocking IO, allocation and ordinary locks, using priority-inheritance mutexes where needed. A runaway real-time thread can starve the system, so I keep an RT runtime limit and a watchdog, and I measure period jitter and overruns.

（译：设置合适优先级的 SCHED_FIFO，用 mlockall 锁定内存，预触碰栈和堆，在单调时钟上睡眠到绝对时间，把线程绑到隔离核并避免阻塞 IO、内存分配和普通锁（必要时用优先级继承互斥量）；失控的实时线程会饿死系统，因此设置 RT 运行时限额和看门狗，并测量周期抖动与超时。）

**追问方向**：为什么用绝对时间睡眠；RT 限额的作用。

### Q38 异构架构（Linux + MCU）下的实时分工与通信？（Linux plus MCU architectures）
**优先级：★★★** · 对应基础知识：[OpenHarmony_Hi3861_RK3588平台基础](../../02_基础知识/11_嵌入式与通信/06_OpenHarmony_Hi3861_RK3588平台基础.md)

**答案要点**
- 分工：MCU（STM32）跑硬实时：电机电流环/速度环、IMU 采样、急停与保护、看门狗；Linux 板（RK3588/Jetson）跑非实时：感知、规划、AI、通信、UI。
- 通信：CAN/FDCAN、UART、SPI、以太网（UDP/EtherCAT）、USB CDC；协议设计：帧头、序号、长度、CRC、版本；命令带**有效期**，超时 MCU 自动安全停止；MCU 周期性上报状态（带时间戳）。
- 关键：MCU 端**不依赖上位机也能安全**（失联保护、本地限幅）；时间同步（MCU 时间戳与主机时钟映射）；带宽与延迟预算；上位机重启不应让电机失控。
- 对应 CV：P3/P7 的 MCU 控制 + P6/P4 的上位平台，其具体接口以各项目文档为准。

**English short answer**
The microcontroller runs hard real-time work such as current and speed loops, IMU sampling, protection and the watchdog, while the Linux board runs perception, planning, AI and user interfaces. They communicate over CAN, UART, SPI or Ethernet with framed, sequenced, CRC-protected messages, and commands carry validity times so the MCU stops safely when the host goes silent. The MCU must stay safe without the host, and timestamps are mapped between clocks.

（译：MCU 运行电流环与速度环、IMU 采样、保护和看门狗等硬实时任务，Linux 板运行感知、规划、AI 和用户界面；二者经 CAN、UART、SPI 或以太网通信，使用带帧头、序号和 CRC 的消息，命令带有效期，使主机失联时 MCU 安全停止；MCU 在没有主机时也必须保持安全，并在时钟间映射时间戳。）

**追问方向**：命令有效期的设置；主机重启时如何避免电机失控。

---

## 七、调试与验证

### Q39 RTOS 应用的典型 bug 与排查思路？（Typical RTOS bugs）
**优先级：★★★** · 对应基础知识：[嵌入式调试_JTAG_SWD_逻辑分析仪_故障定位](../../02_基础知识/11_嵌入式与通信/10_嵌入式调试_JTAG_SWD_逻辑分析仪_故障定位.md)

**答案要点**
- 栈溢出（随机崩溃）、堆耗尽/碎片、优先级配置错误（ISR 优先级高于阈值调用 API）、忘记 `portYIELD_FROM_ISR`（唤醒延迟）、在 ISR 中调用非 ISR API、竞态（共享变量无保护）、优先级反转与死锁、tick 过快导致开销大、看门狗误喂、任务创建失败未检查返回值。
- 排查：开启 `configASSERT`、栈溢出检测、malloc 失败 hook；RTOS 感知调试（任务列表、栈水位、阻塞对象）；运行时统计（`vTaskGetRunTimeStats`）看 CPU 占用；Tracealyzer/SystemView 可视化调度时间线；引脚翻转 + 逻辑分析仪；HardFault 处理程序保存寄存器并解析。
- 习惯：每个 API 返回值检查；统一错误处理；关键对象启动时一次创建。

**English short answer**
Common problems are stack overflow, heap exhaustion, wrong interrupt priorities, calling non-ISR APIs from interrupts, missing yield after waking a task from an ISR, races on shared data, priority inversion, deadlock and unchecked creation failures. I enable asserts and overflow hooks, use an RTOS-aware debugger, runtime statistics and trace tools such as Tracealyzer or SystemView, and probe pins with a logic analyzer, and I check every API return value.

（译：常见问题有栈溢出、堆耗尽、中断优先级错误、在中断中调用非 ISR API、从 ISR 唤醒任务后缺少让出、共享数据竞态、优先级反转、死锁以及未检查的创建失败；开启断言和溢出钩子，使用 RTOS 感知调试器、运行时统计和 Tracealyzer 或 SystemView 等跟踪工具，用逻辑分析仪探测引脚，并检查每个 API 的返回值。）

**追问方向**：HardFault 现场如何解析；运行时统计的开销。

### Q40 如何测量任务的 CPU 占用与调度时间线？（Measuring CPU load and timing）
**优先级：★★** · 对应基础知识：[嵌入式调试_JTAG_SWD_逻辑分析仪_故障定位](../../02_基础知识/11_嵌入式与通信/10_嵌入式调试_JTAG_SWD_逻辑分析仪_故障定位.md)

**答案要点**
- 运行时统计：配置一个比 tick 高 10–20 倍的高分辨率定时器作为 `portGET_RUN_TIME_COUNTER_VALUE`，调用 `vTaskGetRunTimeStats` 得到各任务累计时间占比；空闲任务占比 ≈ 剩余 CPU。
- 时间线：Tracealyzer / SEGGER SystemView 通过 SWO/RTT 记录上下文切换、ISR 进出，看到抢占关系与阻塞时间。
- 简易法：任务入口翻转 GPIO，用逻辑分析仪测高电平宽度（执行时间）和周期。
- DWT 周期计数器测函数执行周期数（`DWT->CYCCNT`）。
- 记录峰值与均值；负载目标：稳态 CPU 不超过 60%–70%（留 RMS 与突发余量，经验值）。

**English short answer**
I enable runtime statistics with a fast timer to see each task's share and the idle percentage, record scheduling timelines with Tracealyzer or SystemView to see preemption and blocking, toggle GPIO pins at task entry and exit for logic analyzer measurements, and use the DWT cycle counter to time functions. I track peak as well as average load and keep steady-state utilization well below the schedulability bound.

（译：用高速定时器启用运行时统计查看各任务占比与空闲百分比，用 Tracealyzer 或 SystemView 记录调度时间线以观察抢占与阻塞，在任务入口和出口翻转 GPIO 用逻辑分析仪测量，并用 DWT 周期计数器给函数计时；同时跟踪峰值与平均负载，使稳态利用率远低于可调度性界。）

**追问方向**：运行时统计定时器的分辨率；负载余量的依据。

### Q41 实时系统的测试与验证策略？（Testing real-time behavior）
**优先级：★★★** · 对应基础知识：[测试策略_仿真在环_硬件在环_回归](../../02_基础知识/13_系统工程与安全/04_测试策略_仿真在环_硬件在环_回归.md)

**答案要点**
- 时序测试：长时间运行（数小时–数天）统计周期抖动与 overrun；最坏情况压力（所有通信满载、所有中断触发、最大计算路径、低电压/高温）；
- 故障注入：丢帧、总线错误、传感器超范围、任务阻塞、内存紧张、看门狗触发，验证安全状态；
- 仿真/硬件在环（HIL）：用仿真被控对象闭环测试控制器；
- 回归：关键指标阈值（最大抖动、最大 ISR 时间、栈水位、堆水位）纳入自动检查；
- 记录：固件版本、配置、测试条件；数据可复现。
- 面试：强调“用数据证明实时性”，而非“感觉流畅”。

**English short answer**
I run long soak tests collecting jitter and overrun statistics, stress the worst case with all interfaces busy and every interrupt firing, and inject faults such as dropped frames, bus errors, out-of-range sensors and blocked tasks to verify safe states. Hardware-in-the-loop with a simulated plant tests the closed loop, and thresholds on jitter, ISR time and stack and heap watermarks run as regression checks. Evidence from measurements is what proves real-time behavior.

（译：运行长时间浸泡测试收集抖动与超时统计，在所有接口繁忙且所有中断触发的最坏情况下施压，并注入丢帧、总线错误、传感器超范围和任务阻塞等故障来验证安全状态；用带仿真被控对象的硬件在环测试闭环，把抖动、ISR 时间和栈/堆水位的阈值作为回归检查；实时性靠测量数据证明。）

**追问方向**：HIL 的搭建；最坏情况压力用例设计。

---

## 本章要点速记
1. 实时 = 可预测地满足截止期；用 WCET、抖动、响应时间、错过率度量。
2. RMS：短周期高优先级，$U\le n(2^{1/n}-1)$ 充分；精确用响应时间分析 $R=C+B+\sum\lceil R/T_j\rceil C_j$；EDF $U\le1$。
3. 优先级反转用继承/天花板；互斥量有继承，二值信号量没有。
4. FreeRTOS：固定优先级抢占 + 同级时间片；PendSV 切换；ISR 优先级须不高于系统调用阈值才能调用 FromISR API。
5. 任务通知最快；队列拷贝语义；静态分配优于动态；栈用高水位线 + 余量。
6. ISR 只做最少工作（上半部），重活交任务；控制环按周期选 ISR 或高优先级任务；周期取谐波。
7. 分层看门狗：任务心跳全部正常才喂狗；日志用无锁环形缓冲 + 低优先级落盘。
8. Linux 实时：PREEMPT_RT + SCHED_FIFO + mlockall + 绑核；MCU 保证无主机也安全。

## 自测题
1. 任务 (C,T)=(1,5),(2,8),(3,20)，判断 RMS 可调度性并做响应时间分析。
2. 解释优先级反转如何发生，以及 FreeRTOS 互斥量与二值信号量的区别。
3. 为什么 `configMAX_SYSCALL_INTERRUPT_PRIORITY` 设置错误会随机崩溃？
4. 设计一个 1 kHz 控制任务的触发方式并说明如何检测超时。
5. 设计分层看门狗，说明任务卡死时电机如何进入安全态。
6. 如何证明一个 Linux 实时线程满足 1 ms 周期？

## 相关链接
- 同模块：[嵌入式与机电八股](../03_机器人学/06_嵌入式与机电八股.md) · [通信总线与协议八股](../01_计算机基础/04_C语言与嵌入式八股.md) · [嵌入式Linux_启动与调试八股](../03_机器人学/06_嵌入式与机电八股.md)
- 基础知识：[FreeRTOS 实时系统](../../02_基础知识/11_嵌入式与通信/02_FreeRTOS实时系统.md) · [实时调度理论](../../02_基础知识/11_嵌入式与通信/11_实时调度理论_RMS_EDF_优先级反转.md)
- 八股总入口：[README](../README.md)
