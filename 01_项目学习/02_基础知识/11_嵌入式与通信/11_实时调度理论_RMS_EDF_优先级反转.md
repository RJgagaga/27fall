# 实时调度理论：周期任务模型、RMS 与 EDF、响应时间分析与优先级反转

> **一句话定位**：[FreeRTOS 一章](./02_FreeRTOS实时系统.md) 告诉你"怎么创建任务、怎么设优先级"，这一章回答"这样设计到底能不能保证每个任务按时完成"——用周期任务模型、利用率界、响应时间分析和优先级反转的理论，把"我感觉 CPU 够用"变成"我证明了最坏响应时间小于截止期"。这是控制工程师把算法落到 P3（STM32F407/FreeRTOS）与 P7（STM32H7/FreeRTOS）固件上的理论依据。

**本章学完你能：**

- 用 $(C_i,T_i,D_i)$ 描述周期任务，区分硬实时、软实时，说清楚"实时不等于快"；
- 推导并使用 RMS 利用率界 $n(2^{1/n}-1)$，知道它是充分不必要条件；
- 做 EDF 可调度性判断（$U\le1$）并解释它与 RMS 的差别；
- 做精确的**响应时间分析**（RTA）迭代，含阻塞项与上下文切换开销；
- 构造并手算一个优先级反转时间线，说明优先级继承和优先级天花板各自如何限制阻塞；
- 讲出火星探路者（Mars Pathfinder）案例的要点，并说明 FreeRTOS 的互斥量提供什么；
- 把 P7 的任务划分（IMU、控制、监控、通信）套进 RTA，得出校验结论；
- 用英文回答 "How do you make sure your control loop meets its deadline?"

**前置知识：**[FreeRTOS 实时系统](./02_FreeRTOS实时系统.md)（任务、优先级、互斥量、优先级继承）；[数字控制与实时性](../05_控制理论/08_数字控制与实时性.md)（采样周期、延迟）；基础的求和与向上取整。

**相关项目：**[P3 FreeRTOS 任务设计与时序](../../01_项目课程/P3_RoboMaster英雄机器人嵌入式控制/03_FreeRTOS任务设计与时序.md)、[P7 STM32H7/FreeRTOS/FDCAN 软件结构](../../01_项目课程/P7_可变腿长轮腿机器人/10_STM32H7_FreeRTOS_FDCAN软件结构.md)、[P7 工程化实践](../../01_项目课程/P7_可变腿长轮腿机器人/20_工程化实践_频率周期与问题解决.md)。

## 目录

1. 要解决什么问题
2. 符号与单位
3. 任务模型与调度器分类
4. RMS 与利用率界
5. EDF
6. 响应时间分析（RTA）
7. 阻塞、优先级反转与两种协议
8. 完整手算：同一任务集的三种判断
9. 把 P7 的任务集套进去
10. 在七个项目里怎么用
11. 常见错误
12. 英文面试问答
13. 要点速记、自测题、相关链接

---

## 1. 要解决什么问题

### 1.1 从"能跑"到"保证"

你的平衡控制器每 2 ms 必须算出一次力矩。如果某个周期晚了 3 ms，会发生什么？对倒立摆来说，一次延迟相当于相位滞后，足以让接近稳定边界的系统发散（见 [数字控制与实时性](../05_控制理论/08_数字控制与实时性.md)）。所以控制任务有**截止期（deadline）**：必须在释放后的 $D$ 时间内完成。

设计者要回答：在最坏情况下——所有任务同时到来、中断同时触发、临界区正在被占用——控制任务能否仍然在截止期内完成？这不能靠"跑一晚上没出错"，因为最坏情况可能一年才出现一次。调度理论给出两类工具：

1. **利用率测试**：只用 $C/T$ 的和做快速判断（充分条件，保守）；
2. **响应时间分析**：对每个任务算最坏响应时间 $R_i$，与截止期比较（对固定优先级抢占调度，是精确的）。

### 1.2 实时的含义

- **硬实时（hard real-time）**：错过截止期即系统失败（如平衡控制、电机保护）。
- **软实时（soft real-time）**：偶尔错过可接受，降低质量（如日志、UI、遥测）。
- **"实时"不是"快"**：它是"可预测、有上界"。一个平均 10 µs 但偶尔 5 ms 的系统，不如固定 200 µs 的系统。

---

## 2. 符号与单位

| 符号 | 含义 | 单位 |
|---|---|---|
| $\tau_i$ | 第 $i$ 个任务 | — |
| $C_i$ | 最坏执行时间（WCET） | s、ms |
| $T_i$ | 周期（或最小到达间隔） | ms |
| $D_i$ | 相对截止期，通常 $D_i\le T_i$ | ms |
| $U_i=C_i/T_i$ | 单个任务利用率 | 无量纲 |
| $U=\sum_i U_i$ | 总利用率 | 无量纲 |
| $R_i$ | 最坏响应时间（从释放到完成） | ms |
| $B_i$ | 阻塞时间（被低优先级任务的临界区等拖延） | ms |
| $J_i$ | 释放抖动 | ms |
| hp($i$) | 优先级高于 $i$ 的任务集合 | — |
| $\lceil x\rceil$ | 向上取整 | — |
| $\sigma$ | 一次上下文切换开销 | µs |

本章约定：**数值越大优先级越高**（FreeRTOS 的约定）。NVIC 中断优先级则是数值越小越高，不要混淆。

---

## 3. 任务模型与调度器分类

### 3.1 周期任务模型

每个任务 $\tau_i=(C_i,T_i,D_i)$：每隔 $T_i$ 释放一个作业（job），需要最多 $C_i$ 的 CPU 时间，必须在释放后 $D_i$ 内完成。基本假设（Liu 与 Layland 1973，文献未核对原文）：

1. 单处理器；
2. 任务相互独立（无共享资源、无同步）；
3. $D_i=T_i$；
4. 上下文切换开销忽略；
5. 抢占式、任务可在任意点被抢占。

现实偏离这些假设就需要扩展：共享资源引入阻塞项 $B_i$，切换开销要计入 $C_i$，中断当成最高优先级的"任务"。

### 3.2 最坏情况与关键时刻

对固定优先级调度，一个任务的最坏响应出现在**关键时刻（critical instant）**：该任务与所有更高优先级任务**同时释放**。证明思路：把一个高优先级任务的释放时刻往后推，只会让它在被测任务的响应窗口里"出现次数不增加"；所以同时释放是最坏。这就是 RTA 里从"同时释放"开始迭代的原因。

### 3.3 固定优先级与动态优先级

| 类别 | 例子 | 说明 |
|---|---|---|
| 固定优先级 | RMS（周期越短优先级越高）、DMS（截止期越短优先级越高） | 实现简单，FreeRTOS 等 RTOS 原生支持 |
| 动态优先级 | EDF（绝对截止期最近者优先） | 理论上利用率可达 100%，实现和分析更复杂 |

RMS（Rate Monotonic Scheduling）：在 $D_i=T_i$ 的假设下，是固定优先级中的最优策略（若任何固定优先级分配能调度该任务集，则 RMS 也可）。当 $D_i<T_i$ 时，DMS（Deadline Monotonic）最优。

> **与 FreeRTOS 的关系**：FreeRTOS 是固定优先级、抢占式，同优先级时间片轮转。你把优先级按周期从短到长递减设置，就是在手动实现 RMS。但 FreeRTOS 里常见的实际设置（如控制任务最高、IMU 次之）不一定严格按周期，这时用 RTA 校验，而不是依赖 RMS 界。

---

## 4. RMS 与利用率界

### 4.1 Liu–Layland 界

$n$ 个任务在 RMS 下可调度的**充分条件**：

$$ U=\sum_{i=1}^{n}\frac{C_i}{T_i}\le n\left(2^{1/n}-1\right). $$

数值：

| $n$ | 界 |
|---|---|
| 1 | 1.0000 |
| 2 | 0.8284 |
| 3 | 0.7798 |
| 4 | 0.7568 |
| 5 | 0.7435 |
| 10 | 0.7177 |
| $n\to\infty$ | $\ln2=0.6931$ |

所以"总利用率不超过 69.3%"是任意数量任务的安全线。超过这条线**不一定**不可调度（它只是充分条件），需要进一步分析。

### 4.2 推导 $n=2$ 的界（为什么是 0.8284）

设 $T_1<T_2<2T_1$，任务 1 优先级高。令 $T_1=1$，$T_2=r\in(1,2)$。任务 1 在任务 2 的一个周期 $[0,r]$ 内被释放两次（$t=0$ 和 $t=1$），所以任务 2 能完成的条件是它的执行加上被抢占的两次执行不超过 $r$：

$$ C_2+2C_1\le r. $$

保证界等于"恰好把 CPU 占满、利用率最低"的那类任务集的利用率。Liu 与 Layland 的论证（此处只给结论的结构，原文未核对）是：这类最坏任务集中，任务 1 在 $t=1$ 的第二次执行刚好在 $t=r$ 处结束，即 $C_1=r-1$；任务 2 用满剩余的空闲时间，$C_2=r-2C_1=2-r$。于是

$$ C_1=r-1,\qquad C_2=2-r. $$

可以验证：此时 $C_2+2C_1=2-r+2r-2=r$，恰好等于窗口长度，任务 2 在 $t=r$ 完成，没有任何富余，再增加任何一点执行时间就会错过截止期。

利用率：

$$ U(r)=C_1+\frac{C_2}{r}=(r-1)+\frac{2-r}{r}=r+\frac2r-2. $$

对 $r$ 求导 $1-2/r^2=0$，得 $r=\sqrt2$，$U_{min}=\sqrt2+\sqrt2-2=2\sqrt2-2=0.8284$。这就是 $n=2$ 的界：**任何利用率不超过它的任务集一定能调度；存在利用率略高于它而恰好不能调度的任务集**。推广到 $n$ 个任务得 $n(2^{1/n}-1)$。

### 4.3 双曲线界

更紧一点的充分条件（Bini 等 2003，文献未核对）：

$$ \prod_{i=1}^{n}\left(\frac{C_i}{T_i}+1\right)\le2. $$

它比 Liu–Layland 界宽松，但仍然是充分条件。

### 4.4 谐波周期

若所有周期互为整数倍（谐波），如 1、2、10、20 ms，RMS 的利用率界可达 100%。控制系统里常把周期设计为谐波（1 ms、2 ms、10 ms、20 ms），这是有理论依据的。

---

## 5. EDF

### 5.1 原理与条件

EDF（Earliest Deadline First）：每个时刻运行**绝对截止期最近**的就绪作业。对 $D_i=T_i$ 的独立周期任务，可调度的**充要条件**为：

$$ U=\sum_i\frac{C_i}{T_i}\le1. $$

即 EDF 能用满 CPU，而 RMS 的保证线只有 69%–83%（不过实际工程里 RMS 的真实可调度上限常高于界）。

### 5.2 为什么工程里仍多用固定优先级

1. **过载行为**：RMS 过载时，低优先级任务先错过截止期，高优先级任务不受影响，行为可预测；EDF 过载时，可能出现"多米诺"——很多任务一起错过截止期。
2. **实现**：固定优先级只需要一个就绪表；EDF 需要按绝对截止期动态排序。FreeRTOS 官方内核不提供 EDF。
3. **与中断模型一致**：硬件中断天然是固定优先级。
4. **可解释**：RTA 对固定优先级是精确的、工程师熟悉的。

---

## 6. 响应时间分析（RTA）

### 6.1 公式

对任务 $i$ 在固定优先级抢占下，最坏响应时间 $R_i$ 满足递推方程：

$$ R_i=C_i+B_i+\sum_{j\in hp(i)}\left\lceil\frac{R_i+J_j}{T_j}\right\rceil C_j. $$

解释：任务 $i$ 的响应由三部分组成：自己的执行 $C_i$；被低优先级任务阻塞的时间 $B_i$；在 $[0,R_i]$ 窗口里每个高优先级任务 $j$ 的抢占，次数最多 $\lceil R_i/T_j\rceil$ 次（加上抖动 $J_j$ 时更多），每次耗时 $C_j$。

右边含 $R_i$，用**不动点迭代**：

1. 取 $R^{(0)}=C_i+B_i$；
2. $R^{(k+1)}=C_i+B_i+\sum_{j\in hp(i)}\lceil R^{(k)}/T_j\rceil C_j$；
3. 若 $R^{(k+1)}=R^{(k)}$ 收敛，得 $R_i$；若 $R^{(k)}>D_i$，则不可调度，停止。

因为右边关于 $R$ 单调不减且阶梯上升，迭代序列单调不减，要么收敛要么超过 $D_i$，所以一定终止。

### 6.2 加入上下文切换与中断

- 每次作业至少两次上下文切换（切入、切出），把 $C_j$ 换成 $C_j'=C_j+2\sigma$；
- 中断可以当作最高优先级的"任务"，有自己的 $C$ 和最小间隔 $T$（例如 CAN 接收中断 1 kHz，每次 20 µs）；
- 系统 tick 中断（1 kHz，每次几微秒到十几微秒）也要算：$C=10\ \mu s,\ T=1\ ms$，利用率 1%。

### 6.3 在 FreeRTOS 里怎么得到 $C_i$

WCET 不能靠"跑几次取最大"，但工程上的折中是：DWT 周期计数器测量最大值（见 [嵌入式调试](./10_嵌入式调试_JTAG_SWD_逻辑分析仪_故障定位.md)），加 20%–30% 余量作为 $C_i$。注意 Cache 命中情况（H7）、Flash 等待状态（F4）、分支预测对执行时间的影响——最坏情况可能是冷 Cache。

---

## 7. 阻塞、优先级反转与两种协议

### 7.1 优先级反转

当高优先级任务 H 需要被低优先级任务 L 持有的互斥量时，H 被阻塞。这是正常的（互斥的代价）。**问题**出现在：L 在临界区里被一个**中优先级**任务 M 抢占，M 与资源无关但优先级高于 L，于是 H 要等 M 运行完、L 才能继续并释放资源——H 的阻塞时间被一个与它无关的任务拉长，且上界取决于 M 的运行时间甚至多个 M 任务，称为**无界优先级反转**。

### 7.2 完整手算时间线

任务（数字越大越高）：L（优先级 1）、M（2）、H（3），单位 ms（教学示例）。

- t=0：L 开始运行，立即锁住互斥量 X，临界区需 2 ms；
- t=1：H 释放（需要执行 1 ms 且需要 X）；H 抢占 L，试图锁 X 失败，阻塞；同一刻 M 释放（需 6 ms，不需要 X）。

**无优先级继承**：

| 时间 | 运行者 | 说明 |
|---|---|---|
| 0–1 | L | 持有 X，临界区已做 1 ms，剩 1 ms |
| 1 | H 阻塞 | 等 X |
| 1–7 | M | M 优先级高于 L，抢占 L 运行 6 ms |
| 7–8 | L | 完成剩余 1 ms，释放 X |
| 8–9 | H | 获得 X，运行 1 ms |

H 的响应时间 $=9-1=8$ ms，其中阻塞 7 ms（L 的 1 ms + M 的 6 ms）。

**有优先级继承**：t=1 时 L 继承 H 的优先级（3）：

| 时间 | 运行者 | 说明 |
|---|---|---|
| 0–1 | L | 持有 X |
| 1–2 | L（继承了优先级 3） | M 无法抢占，L 完成 1 ms，释放 X，回到优先级 1 |
| 2–3 | H | 获得 X，运行 1 ms |
| 3–9 | M | 运行 6 ms |

H 的响应时间 $=3-1=2$ ms，阻塞只有 1 ms（L 的剩余临界区）。**继承把阻塞限制在"低优先级任务的临界区长度"，与 M 无关。**

### 7.3 优先级继承协议（PIP）

当 H 阻塞于 L 持有的锁时，L 的优先级临时提升到 H 的优先级，释放锁后恢复。性质：

- 阻塞有界：一个任务最多被每个低优先级任务阻塞一次，且最多被每个互斥量阻塞一次；
- **不能防止死锁**；
- 可能出现**链式阻塞**（chained blocking）：H 依次被多个持锁的低优先级任务阻塞。

FreeRTOS 的互斥量（`xSemaphoreCreateMutex`）实现了优先级继承；二值信号量没有。因此**用互斥保护共享资源必须用互斥量，不能用二值信号量**，这在 [FreeRTOS 一章](./02_FreeRTOS实时系统.md) 里也有提到。

### 7.4 优先级天花板协议（PCP/ICPP）

给每个互斥量一个天花板优先级 = 可能锁它的所有任务中的最高优先级。**立即天花板协议（ICPP）**：任务一旦锁住互斥量，优先级立刻升到该天花板。性质：

- 一个任务**至多被阻塞一次**（至多一个低优先级任务的一个临界区）；
- 单处理器上**不会死锁**（持锁者永远不会被可能争同一资源的任务抢占）；
- 代价：即使没有竞争，也提升优先级，可能延迟不相关的中优先级任务；需要预先知道所有使用者。

阻塞项公式：对任务 $i$，$B_i=\max\{\text{临界区长度}\ \text{of}\ \text{低优先级任务}\ k\ \text{的使用天花板} \ge P_i\ \text{的资源}\}$，即一个临界区的最大值。

FreeRTOS 官方内核没有天花板协议；可用"在临界区进出时手工调整优先级"或关调度器（`vTaskSuspendAll`）、关中断（`taskENTER_CRITICAL`）来近似，代价是同时阻塞了所有任务或中断。

### 7.5 火星探路者案例

1997 年美国火星探路者（Mars Pathfinder）着陆器在火星上出现反复的系统复位。公开资料的解释是：运行于 VxWorks 的低优先级气象数据任务持有一个与总线管理相关的互斥量，中优先级通信任务抢占了它，使高优先级的总线管理任务长时间拿不到互斥量；看门狗检测到总线管理任务超时，触发复位。修复方法是通过远程上传补丁，打开该互斥量的优先级继承选项。（这是一个常被引用的经典案例；具体细节以原始报道为准，本文未核对原文，面试只需要讲到"低优先级持锁—中优先级抢占—高优先级被阻塞—看门狗复位—启用优先级继承"这条因果链。）

### 7.6 其他阻塞来源

- **关中断/临界区**：`taskENTER_CRITICAL` 期间所有 FreeRTOS 可屏蔽中断被延迟，这是对所有任务的阻塞，应保持在几微秒以内；
- **非抢占段**：Flash 擦写、DMA 传输期间的总线占用；
- **共享总线仲裁**：CAN 发送邮箱满、SPI 总线被另一任务占用。

---

## 8. 完整手算：同一任务集的三种判断

任务集（教学示例，非项目实测），时间单位 ms，$D_i=T_i$：

| 任务 | $C$ | $T$ | $U$ |
|---|---|---|---|
| $\tau_1$ | 1 | 4 | 0.25 |
| $\tau_2$ | 2 | 6 | 0.333 |
| $\tau_3$ | 3 | 12 | 0.25 |

$U=0.8333$。

**(1) Liu–Layland 界**：$n=3$，界 0.7798。$0.8333>0.7798$，**不能由 LL 界保证**。

**(2) 双曲线界**：$(1.25)(1.333)(1.25)=2.083>2$，**也不能保证**。

**(3) RTA**（RMS 优先级：$\tau_1>\tau_2>\tau_3$）：

- $R_1=C_1=1\le4$ ✓；
- $R_2$：初值 $2$；$R=2+\lceil2/4\rceil\cdot1=3$；再迭代 $2+\lceil3/4\rceil\cdot1=3$ 收敛。$R_2=3\le6$ ✓；
- $R_3$：初值 3；
  - $R=3+\lceil3/4\rceil\cdot1+\lceil3/6\rceil\cdot2=3+1+2=6$；
  - $R=3+\lceil6/4\rceil\cdot1+\lceil6/6\rceil\cdot2=3+2+2=7$；
  - $R=3+\lceil7/4\rceil\cdot1+\lceil7/6\rceil\cdot2=3+2+4=9$；
  - $R=3+\lceil9/4\rceil\cdot1+\lceil9/6\rceil\cdot2=3+3+4=10$；
  - $R=3+\lceil10/4\rceil\cdot1+\lceil10/6\rceil\cdot2=3+3+4=10$ 收敛。$R_3=10\le12$ ✓。

结论：**RTA 证明可调度**，尽管两个利用率界都不能保证——说明利用率界保守。

**(4) EDF**：$U=0.8333\le1$，可调度。

**再看一个临界例子**：$\tau_1=(2,5)$，$\tau_2=(4,7)$，$U=0.4+0.571=0.971$。RMS 下 $R_2$：初值 4；$4+\lceil4/5\rceil\cdot2=6$；$4+\lceil6/5\rceil\cdot2=8$；$4+\lceil8/5\rceil\cdot2=8$ 收敛，$R_2=8>7$，**RMS 不可调度**。但 $U\le1$，EDF **可调度**。这个例子说明 EDF 的利用率优势，也说明为什么利用率高时要谨慎。

```python
import math
def rta(tasks, blocking=None):
    out = []
    for i, (c, t) in enumerate(tasks):
        b = blocking[i] if blocking else 0
        r = c + b
        while True:
            rn = c + b + sum(math.ceil(r / tj) * cj for cj, tj in tasks[:i])
            if rn == r or rn > 1000:
                break
            r = rn
        out.append(r)
    return out

print(rta([(1, 4), (2, 6), (3, 12)]))      # [1, 3, 10]
print(rta([(2, 5), (4, 7)]))               # [2, 8]
print(sum(c / t for c, t in [(1, 4), (2, 6), (3, 12)]))   # 0.8333
```

---

## 9. 把 P7 的任务集套进去

P7 的任务划分和设计取值见 [P7 软件结构](../../01_项目课程/P7_可变腿长轮腿机器人/10_STM32H7_FreeRTOS_FDCAN软件结构.md)：IMU 1 ms、控制 2 ms（执行预算约 380 µs）、监控 10 ms、通信 20 ms。下面的 $C$ 中，控制 0.38 ms 来自该预算表，其余三个 $C$ 是本章为了演示而设的**教学示例**（不是实测）：

| 任务 | $C$ (ms) | $T$ (ms) | $U$ |
|---|---|---|---|
| IMU | 0.10 | 1 | 0.100 |
| 控制 | 0.38 | 2 | 0.190 |
| 监控 | 0.20 | 10 | 0.020 |
| 通信 | 0.30 | 20 | 0.015 |

总 $U=0.325$，远低于 $n=4$ 的界 0.7568（也低于 0.693），**直接由 LL 界保证可调度**。再做 RTA 确认响应时间（RMS 优先级：IMU > 控制 > 监控 > 通信）：

- $R_{IMU}=0.10$；
- $R_{ctrl}=0.38+\lceil R/1\rceil\cdot0.1$：$R=0.48$ → $0.38+0.1=0.48$ 收敛；
- $R_{mon}$：$0.20+\lceil R/1\rceil0.1+\lceil R/2\rceil0.38$：$R=0.68$ 收敛；
- $R_{comm}$：$0.30+\ldots=0.98$。

考虑控制任务**最高优先级**（P7 的设计取值）时，IMU 任务被控制任务抢占一次：$R_{IMU}=0.10+0.38=0.48\le1$ ✓，仍然满足截止期。

加入阻塞与切换开销：假设控制任务与监控/通信任务共享一个互斥量，最长临界区 0.05 ms（优先级继承保证只被阻塞一次）：$B_{ctrl}=0.05$，则 $R_{ctrl}=0.38+0.05+0.1=0.53$ ms，仍远低于 2 ms；再把切换开销每次 5 µs 计入（$C'=C+0.01$）：$R_{ctrl}\approx0.50$–0.53 ms。CPU 总负载设计在 40% 以下与此一致。

**从结果能得到什么**：

1. 控制任务有约 $2-0.53=1.47$ ms 的**松弛量（slack）**，所以 CAN 发送延迟、偶发的中断风暴不会立刻让它错过截止期；
2. 把控制任务周期缩短为 1 ms 的可行性：$U$ 增加 $0.19$，达到 0.515，仍低于界，但 IMU 与控制同周期，需要重新划分优先级与总线带宽（CAN 负载见 [CAN 一章](./03_CAN_FDCAN与电机通信.md)）；
3. 真正的风险不是平均利用率，而是**周期漂移**和**优先级配置错误**——例如 [P3 FreeRTOS 任务设计与时序](../../01_项目课程/P3_RoboMaster英雄机器人嵌入式控制/03_FreeRTOS任务设计与时序.md) 里三处 `vTaskDelay(1)` 让循环周期变成约 3 ms（利用率分析正确但周期假设错误），这属于"模型与实现不一致"，RTA 无能为力，只能靠测量（GPIO 翻转）发现。

```python
import math
def rta(tasks, blocking=None):
    out = []
    for i, (c, t) in enumerate(tasks):
        b = blocking[i] if blocking else 0
        r = c + b
        while True:
            rn = c + b + sum(math.ceil(r / tj) * cj for cj, tj in tasks[:i])
            if rn == r or rn > 1000:
                break
            r = rn
        out.append(round(r, 3))
    return out

p7 = [(0.1, 1), (0.38, 2), (0.2, 10), (0.3, 20)]
print(sum(c / t for c, t in p7))                 # 0.325
print(rta(p7))                                   # [0.1, 0.48, 0.68, 0.98]
print(rta(p7, [0, 0.05, 0.05, 0.05]))            # [0.1, 0.53, 0.73, 1.13]
```

---

## 10. 在七个项目里怎么用

| 项目 | 关联点 | 性质 |
|---|---|---|
| P7 | 任务划分、周期、执行预算、优先级配置（控制任务最高）见 [P7 软件结构](../../01_项目课程/P7_可变腿长轮腿机器人/10_STM32H7_FreeRTOS_FDCAN软件结构.md)；上一节是把这些取值套进 RTA 的教学演算 | 项目取值 + 本章理论 |
| P3 | 周期由 `vTaskDelay(1)` 的 tick 量化造成约 3 ms；优先级设计里 IMU 任务设成 idle 档的问题（见 [P3 第 03 章](../../01_项目课程/P3_RoboMaster英雄机器人嵌入式控制/03_FreeRTOS任务设计与时序.md) 的讨论）；互斥量与优先级反转的讲解同样在该章；RTA 可用于事后核算 | 课程文档事实 |
| P1 | 控制节点 100 Hz，驱动侧 100 ms 命令看门狗；ROS 侧是通用 Linux 调度，没有硬实时保证 | 项目事实 |
| P6 | 任务规划与实时控制分层（见 [P6 第 07 章](../../01_项目课程/P6_深开鸿M-Robots平台软件/07_任务规划与实时控制的分层.md)）：高层不保证实时，底层控制保证——分层的理论依据就是"把有界响应的工作放在高优先级/实时域" | 项目事实 + 理论联系 |
| P4 | Hi3861 的 LiteOS-M 是 RTOS，滞回状态机运行在低速任务；优先级反转原理同样适用，细节未核对具体内核实现 | 延伸知识 |
| P5 | ROS2 回调执行器的调度（单线程执行器顺序执行回调，没有抢占）与本章不同，实时性需另行考虑 | 对比 |

> **面试要点**：不要说"我用了 RMS"。实话实说：我按"周期短 / 截止期紧的任务优先级高"的原则分配（相当于 RMS/DMS 的直觉），并用测量验证周期与执行时间；理论上可以用响应时间分析核算，我知道怎么算。

---

## 11. 常见错误

1. **把优先级当频率**：高优先级不等于高频率。
2. **用二值信号量保护共享数据**：没有优先级继承，会出现无界反转；应使用互斥量。
3. **在高优先级任务里 `printf` 或阻塞等待外设**：$C_i$ 变成不可预测。
4. **忽略中断的 CPU 占用**：1 kHz 的中断每次 30 µs，就占 3%；多个中断叠加影响显著。
5. **以平均执行时间代替最坏执行时间**：偶发路径（异常处理、重试）决定最坏情况。
6. **同优先级任务假设为"先到先得"**：FreeRTOS 同优先级时间片轮转，响应时间分析要把同级任务视作互相干扰。
7. **在临界区内调用可能阻塞的函数**：阻塞时间无上界。
8. **用 `taskENTER_CRITICAL` 保护很长的代码**：所有可屏蔽中断被延迟，抖动变大。
9. **利用率界当成必要条件**：超过界并不意味着不可调度；也不要拿它证明可调度之外的结论。
10. **忽略 tick 量化**：`vTaskDelay(1)` 实际睡眠在 (0,1] ms；周期被量化成整数个 tick。

---

## 12. 英文面试问答

### Q1. How do you make sure your control loop meets its deadline?

**Answer.** First I design the task set: the control task has the highest priority, it is released at a fixed period with a delay-until call or a hardware timer, and it does no blocking work. Then I measure the worst execution time with the DWT cycle counter and check the period and jitter with a GPIO toggle. With those numbers I can do a response-time analysis: the execution time plus the interference from higher priority tasks and interrupts, plus any blocking from shared resources. I keep a margin, and I try to keep total CPU load well below the limit. In my wheel-legged robot the design target was about nineteen percent for the control task and under forty percent total.

**中文译文**：先设计任务集：控制任务最高优先级，用 delay-until 或硬件定时器固定周期释放，不做阻塞工作。然后用 DWT 周期计数器测最坏执行时间，用 GPIO 翻转核对周期和抖动。有了这些数据就可以做响应时间分析：自身执行时间加高优先级任务和中断的干扰，再加共享资源造成的阻塞。我保留余量并让总 CPU 负载远低于上限。轮腿机器人里设计目标是控制任务约 19%，总负载低于 40%。

**Follow-up 1:** What is the difference between a utilisation test and response-time analysis? — Utilisation is a quick sufficient test and is conservative; response-time analysis is exact for fixed-priority preemptive scheduling. （利用率测试是快速的充分条件，保守；响应时间分析对固定优先级抢占调度是精确的。）

**Follow-up 2:** What if you cannot measure the worst case? — I measure the maximum over long runs with stress, add margin, and monitor the cycle time at run time so a violation is detected. （长时间加压测最大值，加余量，并在运行时监测周期，违规可被发现。）

### Q2. What is priority inversion and how do you prevent it?

**Answer.** Priority inversion happens when a high priority task waits for a mutex held by a low priority task, and a medium priority task preempts the low priority one. Then the high priority task is delayed by a task that has nothing to do with the resource. The classic fix is priority inheritance: the low priority task temporarily gets the priority of the waiting task, so the medium task cannot preempt it, and the blocking is limited to the length of the critical section. FreeRTOS mutexes do this; binary semaphores do not. A stronger option is the priority ceiling protocol, which also prevents deadlock on a single processor. The well-known example is the Mars Pathfinder resets in 1997.

**中文译文**：优先级反转是指高优先级任务等待低优先级任务持有的互斥量，而中优先级任务抢占了低优先级任务，于是高优先级任务被一个与资源无关的任务拖延。经典办法是优先级继承：低优先级任务临时获得等待者的优先级，中优先级任务就无法抢占它，阻塞只限于临界区长度。FreeRTOS 的互斥量支持，二值信号量不支持。更强的是优先级天花板协议，在单处理器上还能防死锁。著名案例是 1997 年火星探路者的反复复位。

**Follow-up 1:** Can priority inheritance prevent deadlock? — No. Deadlock needs a different design, such as a fixed lock order, or the ceiling protocol. （不能；防死锁要靠固定加锁顺序或天花板协议。）

**Follow-up 2:** How would you avoid locks in a control task? — Use a snapshot or a double buffer with a sequence counter, so the control task reads without blocking. （用快照或带序号的双缓冲，使控制任务无阻塞读取。）

### Q3. What is the rate monotonic scheduling bound, and is it necessary?

**Answer.** For rate monotonic scheduling, where shorter period means higher priority, n independent periodic tasks are guaranteed to meet their deadlines if the total utilisation is at most n times two to the power one over n minus one. For two tasks it is about 0.83, for three about 0.78, and it approaches 0.69 for many tasks. It is only sufficient, not necessary. A task set above the bound can still be schedulable, so I use response-time analysis for an exact answer. If the periods are harmonic, the bound rises to one hundred percent.

**中文译文**：在速率单调调度（周期越短优先级越高）下，n 个独立周期任务只要总利用率不超过 n(2^(1/n)−1) 就保证满足截止期。两个任务约 0.83，三个约 0.78，任务很多时趋于 0.69。它只是充分条件，不是必要条件；超过界的任务集仍可能可调度，所以我用响应时间分析得到精确结论。若周期是谐波的，界可升到 100%。

**Follow-up 1:** Give a case where EDF works but RMS does not. — Two tasks with execution and period of two over five and four over seven: utilisation is 0.97, EDF schedules it, RMS misses the deadline of the second task. （(2,5) 与 (4,7)：利用率 0.97，EDF 可调度，RMS 下第二个任务错过截止期。）

**Follow-up 2:** Why is RMS popular in practice despite lower utilisation? — Predictable overload behaviour, simple implementation, and it matches interrupt priorities. （过载行为可预测、实现简单、与中断优先级一致。）

### Q4. How did you choose task priorities in your robot?

**Answer.** I followed the idea that tasks with tighter deadlines or higher criticality should have higher priority. In the wheel-legged robot, the control task is at the highest priority because a late control cycle costs the most. The IMU task is also high priority because it determines how fresh the attitude is, and it is short. Monitoring and communication run at lower priority. I also made sure that priority is not the same as rate. For the RoboMaster project, I later found that the loop period was set by several one-tick delays and was about three milliseconds, not one, so I learned to measure the real period instead of trusting the constants.

**中文译文**：原则是截止期更紧、更关键的任务优先级更高。轮腿机器人里控制任务优先级最高，因为控制周期晚了代价最大；IMU 任务也较高，因为它决定姿态新鲜度且很短；监控和通信优先级较低。同时注意优先级不等于频率。RoboMaster 项目里我后来发现循环周期由几处一 tick 延时决定，约 3 ms 而不是 1 ms，所以学会了测量真实周期而不是相信常量。

**Follow-up 1:** Would you change anything? — I would trigger the control task from a hardware timer and verify the margin with response-time analysis. （用硬件定时器触发控制任务，并用响应时间分析核算余量。）

**Follow-up 2:** Where does the interrupt fit in the analysis? — As the highest priority tasks with their own execution time and minimum inter-arrival time. （作为最高优先级任务，有自己的执行时间和最小到达间隔。）

---

## 13. 要点速记、自测题、相关链接

### 要点速记

1. 周期任务 $(C,T,D)$；利用率 $U=\sum C/T$；响应时间 $R$。
2. 关键时刻：被测任务与所有高优先级任务同时释放，是最坏情形。
3. RMS 界 $n(2^{1/n}-1)$：充分不必要；趋于 69.3%；谐波周期可到 100%。
4. EDF：$U\le1$ 充要（$D=T$）；过载时行为差。
5. RTA：$R=C+B+\sum\lceil R/T_j\rceil C_j$，不动点迭代，超过 $D$ 则不可调度。
6. 中断当作最高优先级任务；切换开销加进 $C$。
7. 优先级反转：低持锁、中抢占、高等待；继承把阻塞限制在临界区长度。
8. FreeRTOS：互斥量有继承，二值信号量没有；无官方天花板协议。
9. 优先级继承不防死锁；天花板协议在单处理器防死锁且至多阻塞一次。
10. 测量 + 分析：DWT 测 $C$、GPIO 测周期、RTA 算余量。

### 自测题

1. $n=3$ 时 RMS 的界是多少？一个总利用率 0.80 的三任务集能否由此界保证调度？
2. 任务集 $(1,3),(1,4),(2,12)$，RMS 优先级下用 RTA 求每个任务的响应时间，是否可调度？
3. 说明为什么二值信号量不能代替互斥量保护共享变量。
4. 在第 7.2 节的时间线里，若 M 的运行时间增大到 20 ms，有/无优先级继承时 H 的响应时间分别是多少？
5. 为什么要把中断计入响应时间分析？

**参考答案要点**：1. 0.7798；不能保证（0.80 > 0.7798），需 RTA。2. 优先级 $\tau_1>\tau_2>\tau_3$：$R_1=1$；$R_2=1+\lceil R/3\rceil$，$R=1\to2\to2$ 收敛，$R_2=2\le4$；$R_3=2+\lceil R/3\rceil+\lceil R/4\rceil$，迭代 $2\to4\to5\to6\to6$ 收敛，$R_3=6\le12$，可调度。3. 二值信号量无优先级继承，会出现无界反转。4. 无继承：M 运行 20 ms（t=1–21），L 完成剩余 1 ms（21–22），H 运行 1 ms（22–23），响应 $=23-1=22$ ms；有继承：仍为 2 ms，与 M 的长度无关。5. 中断抢占所有任务，占用 CPU 时间属于干扰项。

### 相关链接

- 上一章：[嵌入式调试](./10_嵌入式调试_JTAG_SWD_逻辑分析仪_故障定位.md)
- 下一章：[嵌入式 Linux：设备树、驱动与 RT-Preempt](./12_嵌入式Linux_设备树_驱动_RT-Preempt.md)
- 相关：[FreeRTOS 实时系统](./02_FreeRTOS实时系统.md)、[STM32 时钟、GPIO、中断、DMA](./01_STM32_时钟_GPIO_中断_DMA.md)、[数字控制与实时性](../05_控制理论/08_数字控制与实时性.md)
- 八股：[并发多线程与实时系统八股](../../03_八股知识/01_计算机基础/07_并发多线程与实时系统八股.md)、[操作系统](../../03_八股知识/01_计算机基础/02_操作系统.md)
- 项目：[P3](../../01_项目课程/P3_RoboMaster英雄机器人嵌入式控制/README.md)、[P7](../../01_项目课程/P7_可变腿长轮腿机器人/README.md)
