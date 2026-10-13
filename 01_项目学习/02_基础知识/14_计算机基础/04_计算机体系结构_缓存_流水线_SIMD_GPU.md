# 计算机体系结构：存储层次与缓存、流水线与分支、SIMD、GPU 并行模型与带宽瓶颈

> **一句话定位**：同样是 $O(n^2)$ 的两重循环，按行遍历和按列遍历在同一台机器上可以差 7–8 倍，原因不在算法而在硬件：CPU 比内存快得多，靠缓存、流水线、向量指令来"掩盖"差距。本章把存储层次、缓存组织、局部性、流水线冒险、分支预测、SIMD、GPU 的 SIMT 模型和 Roofline 带宽瓶颈讲成可以手算的东西，并落到 P4 在 RK3588 上做 INT8 边缘推理时"为什么量化既省算力也省带宽"。八股速查的相关条目见 [C++ 八股](../../03_八股知识/01_计算机基础/05_C++八股.md) 与 [并发多线程与实时系统八股](../../03_八股知识/01_计算机基础/07_并发多线程与实时系统八股.md)。

**本章学完你能：**

- 画出存储层次金字塔，解释时间局部性与空间局部性，估算平均访存时间 AMAT；
- 把一个地址拆成 tag / index / offset，手算直接映射与组相联缓存的命中与缺失序列；
- 说明为什么行优先遍历比列优先快，并给出一次真实运行的对比数据；
- 解释五级流水线、数据/控制/结构冒险、转发与分支预测，估算有分支惩罚的 CPI；
- 解释 SIMD 的数据并行、对齐与自动向量化的限制，区分 SIMD 与多线程；
- 解释 GPU 的 SIMT 模型（线程、warp、块、网格）与分支发散，以及 CPU 与 GPU 设计哲学的差异；
- 用算术强度与 Roofline 模型判断一个核函数是计算受限还是带宽受限，并说明量化/融合算子在边缘推理中的作用；
- 用英文回答 "Why is iterating a matrix row by row faster than column by column?"

**前置知识**：[操作系统](./02_操作系统_进程线程_内存_调度_同步.md)（虚拟内存、TLB）；二进制地址的位运算；[算法与数据结构复习](./01_算法与数据结构复习_复杂度_图_动态规划.md) 中的复杂度概念。

**相关项目**：[P4 INT8 量化与 RKNN 转换](../../01_项目课程/P4_OpenHarmony智能园艺系统/07_INT8量化与RKNN转换.md)、[P4 RK3588 边缘视觉](../../01_项目课程/P4_OpenHarmony智能园艺系统/05_RK3588边缘视觉_V4L2取帧与预处理.md)、[P3 数字 PID 实现](../../01_项目课程/P3_RoboMaster英雄机器人嵌入式控制/12_数字PID实现.md)、[Cortex-M 体系与内存映射](../11_嵌入式与通信/07_Cortex-M体系_启动_链接脚本_内存映射.md)。

## 目录

1. 要解决什么问题
2. 符号与术语
3. 存储层次与局部性
4. 缓存的组织与手算
5. 流水线、冒险与分支预测
6. SIMD 数据并行
7. GPU 并行模型
8. 算术强度与 Roofline
9. 完整手算与实测（Python 与 C 复核）
10. 在七个项目里怎么用
11. 常见错误
12. 英文面试问答
13. 要点速记、自测题、相关链接

---

## 1. 要解决什么问题

20 世纪后期以来，处理器速度提升快于主存速度，两者差距称为"存储墙"。一次访问主存（DRAM）的延迟在百纳秒量级，而 CPU 一个时钟周期在亚纳秒到纳秒量级，也就是一次主存访问相当于数百个周期的计算机会被浪费。体系结构用三类技术缓解：

1. **缓存**：在 CPU 与主存之间放小而快的存储，利用**局部性**让大多数访问命中；
2. **并行**：流水线（指令级）、超标量/乱序、SIMD（数据级）、多核（线程级）、GPU（大规模数据并行）；
3. **预测**：分支预测、预取，赌未来的行为以便提前做事。

对做机器人算法的人，这些直接决定"同样的算法能否在 1 ms 控制周期内跑完""同一个 YOLO 模型在 NPU 上是否被带宽卡住"。

## 2. 符号与术语

| 术语 / 符号 | 含义 |
|---|---|
| 缓存行 / 块（cache line / block） | 缓存与主存之间一次搬运的最小单位，常见 64 B |
| 命中 / 缺失（hit / miss） | 请求的数据在 / 不在该级缓存 |
| 命中率 $h$，缺失率 $m=1-h$ | |
| $t_{\text{hit}}$，$t_{\text{pen}}$ | 命中时间 / 缺失惩罚（从下一级取数据的额外时间） |
| AMAT | 平均访存时间 $=t_{\text{hit}}+m\cdot t_{\text{pen}}$ |
| tag / index / offset | 地址的三个字段：标记 / 组索引 / 行内偏移 |
| 路（way） | 每组可容纳的缓存行数；1 路为直接映射 |
| CPI / IPC | 每条指令平均周期数 / 每周期指令数 |
| 冒险（hazard） | 流水线中使下一条指令无法在下个周期执行的相关性 |
| SIMD / SIMT | 单指令多数据 / 单指令多线程 |
| warp | NVIDIA GPU 中一起执行同一指令的一组线程（常见 32 个） |
| FLOP / FLOPS | 浮点运算次数 / 每秒浮点运算次数 |
| 算术强度 AI | $\text{FLOP}/\text{Byte}$：每搬一个字节做多少次运算 |
| Roofline | 性能上限 $=\min(\text{峰值算力},\ \text{带宽}\times\text{AI})$ |

## 3. 存储层次与局部性

### 3.1 金字塔

```text
        寄存器          ~ 0.3 ns   KB 级        最快、最小、最贵
        L1 缓存         ~ 1 ns     数十 KB
        L2 缓存         ~ 3-4 ns   数百 KB - 数 MB
        L3 缓存         ~ 10+ ns   数 MB - 数十 MB（多核共享）
        主存 DRAM       ~ 60-100 ns  GB 级
        闪存 / 磁盘     ~ 10^5 ns 以上  百 GB - TB
```

上图延迟为数量级，具体数值因芯片而异（教学估计，非实测，未逐芯片核对）。关键是**每下一级大约慢一个数量级、大一个数量级**。

### 3.2 局部性原理

程序的访存不是均匀随机的：

- **时间局部性**：刚访问过的数据很可能再次访问（循环变量、函数的局部变量）；
- **空间局部性**：访问了某地址，很可能马上访问相邻地址（顺序遍历数组、顺序执行的指令）。

缓存按"行"搬运正是赌空间局部性：读 1 个 `float`（4 B），整条 64 B 的行（16 个 `float`）一起进缓存，后面 15 次顺序访问都命中。

**行优先 vs 列优先**：C 语言二维数组按行存储（`a[i][j]` 的下一个元素是 `a[i][j+1]`）。设 `float a[4096][4096]`：

- 按行遍历（内层 `j`）：步长 4 B，每 16 次访问才缺失 1 次，缺失率 $4/64=6.25\%$；
- 按列遍历（内层 `i`）：步长 $4096\times4=16384$ B，每次访问都落在不同的缓存行，缺失率接近 100%（还可能因为 16384 恰为 2 的幂，各行映射到同一缓存组，造成冲突缺失）。

这就是第 9 节实测里 7–8 倍差距的来源。

### 3.3 平均访存时间

$$\text{AMAT}=t_{\text{hit}}+m\cdot t_{\text{pen}}$$

多级缓存：$\text{AMAT}=t_{L1}+m_{L1}\left(t_{L2}+m_{L2}\,t_{\text{mem}}\right)$。

**手算**（示例，非实测）：$t_{\text{hit}}=1$，$m=5\%$，$t_{\text{pen}}=100$ 周期：$\text{AMAT}=1+0.05\times100=6$ 周期。缺失率只有 5%，平均访存时间却是命中时的 6 倍——缺失惩罚极大，所以要把缺失率压到很低。两级缓存：$t_{L1}=1$，$m_{L1}=10\%$，$t_{L2}=10$，$m_{L2}=20\%$，$t_{\text{mem}}=100$：$\text{AMAT}=1+0.1\times(10+0.2\times100)=4$ 周期（Python 复核）。

## 4. 缓存的组织与手算

### 4.1 地址的三个字段

缓存容量 $C$ 字节，行大小 $B$ 字节，$W$ 路。组数 $S=C/(B\cdot W)$。一个地址分成：

$$\underbrace{\text{tag}}_{\text{高位}}\ \big|\ \underbrace{\text{index}}_{\log_2 S\text{ 位}}\ \big|\ \underbrace{\text{offset}}_{\log_2 B\text{ 位}}$$

offset 选行内字节，index 选组，tag 用来在组内与各路比较确认是否是目标数据。

**手算**：32 位地址，32 KiB、4 路、64 B 行：$S=32768/(64\times4)=128$，index 7 位，offset 6 位，tag $=32-7-6=19$ 位。

- **直接映射**（$W=1$）：每个块只能放一个位置，硬件简单，但两块映射到同组就互相驱逐（冲突缺失）；
- **组相联**：每个块可放组内 $W$ 个位置，折中；
- **全相联**：可放任意位置，没有冲突缺失，但比较电路大，只用于 TLB 等小结构。

三类缺失（3C）：**强制缺失**（第一次访问）、**容量缺失**（工作集超过缓存）、**冲突缺失**（映射冲突）。

### 4.2 直接映射缓存手算

缓存 4 行（4 组，直接映射），行大小 16 B。地址序列（字节地址）：`0, 4, 8, 12, 16, 20, 64, 68, 0, 4, 64, 0`。

块号 $=\lfloor\text{addr}/16\rfloor$，组号 $=\text{块号}\bmod4$，tag $=\lfloor\text{块号}/4\rfloor$。

| 访问地址 | 块号 | 组号 | tag | 结果 | 说明 |
|---|---|---|---|---|---|
| 0 | 0 | 0 | 0 | 缺失 | 强制缺失 |
| 4 | 0 | 0 | 0 | 命中 | 空间局部性 |
| 8 | 0 | 0 | 0 | 命中 | |
| 12 | 0 | 0 | 0 | 命中 | |
| 16 | 1 | 1 | 0 | 缺失 | 新块 |
| 20 | 1 | 1 | 0 | 命中 | |
| 64 | 4 | 0 | 1 | 缺失 | 与块 0 同组，驱逐块 0 |
| 68 | 4 | 0 | 1 | 命中 | |
| 0 | 0 | 0 | 0 | 缺失 | 被驱逐的块 0 重新读取（冲突缺失） |
| 4 | 0 | 0 | 0 | 命中 | |
| 64 | 4 | 0 | 1 | 缺失 | 又驱逐块 0 |
| 0 | 0 | 0 | 0 | 缺失 | 乒乓冲突 |

共 12 次访问，命中 6 次，命中率 50%（Python 模拟复核）。同样的序列换成 2 路组相联（2 组、每组 2 路），块 0 与块 4 同组但都能留住，命中数升至 9，命中率 75%（Python 复核）。这个对比说明冲突缺失可以通过提高相联度缓解。

### 4.3 写策略与一致性

- **写直达（write-through）**：写时同时更新缓存和下一级，简单但带宽消耗大；
- **写回（write-back）**：只写缓存，行被驱逐时才写回下一级（用脏位标记），带宽省，是主流。写缺失时通常 **写分配（write-allocate）**：先把行读进缓存再写。
- **多核缓存一致性**：每个核有私有 L1/L2，同一内存位置可能有多份副本，需要协议（如 MESI 的四状态：修改、独占、共享、无效）维护一致。后果之一是**伪共享（false sharing）**：两个线程各改同一缓存行里的不同变量，行在核之间来回失效，性能骤降。详见 [并发与内存模型](./05_并发与内存模型_锁_原子_无锁.md)。

### 4.4 软件如何配合缓存

1. **按存储顺序遍历**（行优先数组按行扫）；
2. **分块（tiling）**：矩阵乘法把矩阵切成能放入缓存的小块，使每块被重复使用多次；
3. **结构体数组 AoS vs 数组结构体 SoA**：若只用到每个结构体的一个字段，SoA 让该字段连续存放，缓存与 SIMD 都更高效；
4. **对齐与填充**：避免热点变量与别的核频繁写的变量落在同一缓存行；
5. **预取**：顺序访问时硬件预取器自动提前取行，随机指针追逐（链表）则无法预取，所以链表遍历常很慢。

## 5. 流水线、冒险与分支预测

### 5.1 经典五级流水线

一条指令分五级：取指（IF）、译码（ID）、执行（EX）、访存（MEM）、写回（WB）。不流水线时每条指令占 5 个周期；流水线让五条指令各处于不同级，**理想情况每周期完成一条指令**，CPI→1。流水线**不降低单条指令的延迟**，提高的是吞吐。

```text
周期:   1   2   3   4   5   6   7
指令1:  IF  ID  EX  MEM WB
指令2:      IF  ID  EX  MEM WB
指令3:          IF  ID  EX  MEM WB
```

### 5.2 三类冒险

1. **结构冒险**：两条指令同时需要同一硬件资源（如单端口存储器），靠复制资源（指令存储器与数据存储器分开）解决。
2. **数据冒险**：后一条指令需要前一条的结果。例如 `ADD R1,R2,R3` 后紧跟 `SUB R4,R1,R5`（读后写，RAW）。解决：**转发/旁路（forwarding）**把 EX 级结果直接送给下一条的 EX 输入；若是加载指令的结果（`LOAD R1` 后立刻用 R1），转发也来不及，需要**停顿 1 个周期（load-use hazard）**。编译器可调度无关指令填入。
3. **控制冒险**：分支指令的方向在较晚阶段才知道，期间已经取了后续指令。

### 5.3 分支预测与分支惩罚

若预测错误，需要冲刷已取入的错误指令，损失若干周期（深流水线可达十几到二十个周期，具体依微架构，未核对）。

$$\text{CPI}=\text{CPI}_{\text{ideal}}+\text{分支频率}\times\text{误预测率}\times\text{惩罚周期数}$$

**手算**（示例，非实测）：理想 CPI=1，分支占 20%，预测错误率 15%，惩罚 3 个周期：$\text{CPI}=1+0.2\times0.15\times3=1.09$（Python 复核）。若换成深流水线惩罚 15 个周期，同样的预测准确率：$\text{CPI}=1+0.2\times0.15\times15=1.45$，说明深流水线对预测准确率更敏感。

预测器：静态（总预测不跳转）、**1 位/2 位饱和计数器**（2 位计数器要连续两次预测错才改变方向，对循环出口友好）、全局历史+相关性预测等。程序员能做的事：让热点循环里的分支可预测（对排序后的数据做条件判断比随机数据快），或把数据相关分支改成无分支算术（条件传送）。注意不要过早优化，用性能剖析确认瓶颈。

### 5.4 超标量与乱序

超标量处理器每周期可发射多条指令（IPC>1）；**乱序执行**让后面不相关的指令越过等待数据的指令先执行，用重排序缓冲保证最终按序提交。副作用：2018 年公开的 Spectre 一类漏洞利用推测执行的缓存残留信息（背景知识，细节未核对）。嵌入式常用的 Cortex-M 系列多为顺序执行的短流水线，行为更可预测，这正是实时控制喜欢它们的原因之一（参见 [Cortex-M 体系](../11_嵌入式与通信/07_Cortex-M体系_启动_链接脚本_内存映射.md)）。

## 6. SIMD 数据并行

### 6.1 概念

SIMD 用一条指令同时处理向量寄存器中的多个数据。128 位寄存器可放 4 个 `float32` 或 16 个 `int8`；256 位放 8 个 `float32`（`256/32=8`）。代表：x86 的 SSE/AVX、ARM 的 NEON、RISC-V 向量扩展。RK3588 的 ARM 核带 NEON（延伸知识：具体核型号与特性未核对）。

向量加法 `c[i]=a[i]+b[i]`：标量版每次 1 个加法，4 路 SIMD 每次 4 个，理论加速 4 倍；实际受限于内存带宽（向量加法每个元素读 8 B 写 4 B，只做 1 次加法，是典型带宽受限）。

### 6.2 SIMD 友好的条件

1. 数据**连续**存放（SoA）；
2. 循环无**循环携带依赖**（`a[i]=a[i-1]+x` 不能直接向量化）；
3. 指针**无别名**（C 里 `restrict`，C++ 里编译器选项或注解），否则编译器保守不向量化；
4. 对齐（不少指令要求或偏好对齐访问）；
5. 分支用掩码（predicate）处理，两个分支都算，再按掩码选结果。

编译器的自动向量化（`-O3` 等）能处理简单循环；复杂情形需要手写内部函数（intrinsics）或使用库（Eigen、NumPy）。Python 的 NumPy 把循环下沉到已向量化的 C 代码，这也是它比纯 Python 循环快数十倍的原因之一（另一原因是避免解释器开销）。

### 6.3 SIMD 与多线程、与量化的关系

SIMD 是**一个核内**的数据并行，多线程是**多个核**的任务并行，两者可叠加。INT8 量化让同样宽度的向量寄存器一次处理的元素数是 FP32 的 4 倍；同时模型权重体积缩小为 1/4，访存字节数同比减少（见第 10 节 P4 的联系）。

## 7. GPU 并行模型

### 7.1 设计哲学

CPU 优化**单线程延迟**：大缓存、复杂控制逻辑（乱序、分支预测）；GPU 优化**吞吐**：把晶体管用于成千上万个简单的运算单元，靠**大量线程切换**掩盖访存延迟（一个 warp 等内存时，调度器立刻切到另一个就绪 warp，切换几乎免费，因为每个线程的寄存器常驻硬件）。

### 7.2 SIMT：线程层次

以 NVIDIA CUDA 术语（常见，也是通用概念的代表）：

```text
网格 grid
 └─ 线程块 block（同块线程共享"共享内存"，可同步）
     └─ warp（32 个线程，锁步执行同一条指令）
         └─ 线程 thread（各有自己的寄存器与索引）
```

程序员写**核函数（kernel）**，描述一个线程做什么，用线程索引决定处理哪个数据；GPU 并行启动成千上万个线程。**SIMT** 在编程上像是每个线程独立，硬件上则是一个 warp 内线程共享一个程序计数器，以锁步执行。

### 7.3 分支发散

若同一 warp 内的线程在 `if` 上走了不同方向，硬件要把两条路径**依次**执行（不参与的线程被掩码关闭），耗时是两条路径之和。所以 GPU 代码要尽量让同一 warp 的线程走相同分支。

### 7.4 存储层次与访问合并

GPU 的存储：寄存器 > 共享内存/L1（块内共享，片上，快）> L2 > 全局显存（带宽高但延迟高）。同一 warp 的线程访问**连续地址**时，硬件把多次访问**合并**成少量显存事务；跨步或随机访问则无法合并，带宽利用率大降——与 CPU 的空间局部性是同一思想。

### 7.5 GPU 不是万能的

数据量小、逻辑分支多、串行依赖强的任务不适合 GPU。主机与设备之间的数据传输（PCIe）有开销，若计算量不足以摊薄传输，反而更慢。边缘端 SoC（如 RK3588 的 NPU/GPU 与 CPU 共享 DRAM）没有独立显存，传输开销小，但带宽是共享的稀缺资源。

## 8. 算术强度与 Roofline

### 8.1 定义

算术强度 $\text{AI}=\dfrac{\text{浮点运算次数}}{\text{访问主存的字节数}}$，单位 FLOP/Byte。Roofline 模型：

$$P=\min\big(P_{\text{peak}},\ \text{BW}\times\text{AI}\big)$$

当 AI 小于**拐点** $\text{AI}^*=P_{\text{peak}}/\text{BW}$ 时，性能受带宽限制（斜线部分）；大于拐点时，受峰值算力限制（水平部分）。

**手算**（假想设备，教学示例，非任何真实芯片）：$P_{\text{peak}}=1000\,\text{GFLOP/s}$，$\text{BW}=50\,\text{GB/s}$，拐点 $=20\,\text{FLOP/B}$。

| 核函数 | AI（FLOP/B） | 性能上限（GFLOP/s） | 受限于 |
|---|---|---|---|
| 向量加法 float32：每元素 1 FLOP，读 2 个写 1 个 = 12 B | 0.083 | 4.2 | 带宽 |
| 假想 AI=2 的核 | 2 | 100 | 带宽 |
| 假想 AI=20 的核 | 20 | 1000 | 刚好拐点 |
| 大矩阵乘法 $n=1024$：$2n^3$ FLOP，理想访存 $3n^2\times4$ B | 约 170.7 | 1000 | 算力 |

（向量加法与矩阵乘法的 AI 以及各档上限由 Python 复核；矩阵乘法的 170.7 是"理想缓存、只算一次读写"的上界，实际受分块效率影响。）

### 8.2 结论

大多数视觉预处理（逐像素的缩放、颜色转换、归一化）、逐元素算子（ReLU、加法）的 AI 很低，**带宽受限**。这就是推理引擎做**算子融合**（把卷积 + 批归一化 + 激活合成一个核，中间结果不落回内存）、**量化**（每个元素字节数缩小）的原因：它们提高 AI 或降低字节数，而不是增加算力。

**一个带宽估算**（示例，按常见 1080p 30 帧计算，非项目实测）：1920×1080 RGB 每像素 3 B，30 fps：$1920\times1080\times3\times30\approx186.6$ MB/s，仅这一路相机的原始数据就是近 187 MB/s，如果预处理多次读写整帧（缩放、色彩转换、归一化各一遍），带宽被成倍放大，所以工程上会合并步骤、在硬件模块（如 ISP/RGA）里完成（RGA 等具体硬件模块的能力未核对）。

## 9. 完整手算与实测（Python 与 C 复核）

### 9.1 行优先 vs 列优先的实测

下面的 C 程序（教学示例，非项目源码）对 `float a[4096][4096]` 求和，先按行再按列：

```c
#include <stdio.h>
#include <time.h>
#define N 4096
static float a[N][N];
int main(void){
    for(int i=0;i<N;i++) for(int j=0;j<N;j++) a[i][j]=1.0f;
    clock_t t0=clock(); double s=0;
    for(int i=0;i<N;i++) for(int j=0;j<N;j++) s+=a[i][j];   /* 行优先 */
    clock_t t1=clock(); double s2=0;
    for(int j=0;j<N;j++) for(int i=0;i<N;i++) s2+=a[i][j];  /* 列优先 */
    clock_t t2=clock();
    printf("row %.3f s col %.3f s\n",(double)(t1-t0)/CLOCKS_PER_SEC,(double)(t2-t1)/CLOCKS_PER_SEC);
    return 0;}
```

在撰写本章的开发机上（MinGW GCC 8.1.0，`-O0`，两次运行）：行优先约 0.026–0.027 s，列优先约 0.199–0.203 s，比值约 7.5–7.7 倍；两种顺序的结果都是 16777216，说明只有访存顺序不同。**这是一次本机实测，数字随机器、编译选项、缓存大小变化；方向（行快于列）是稳定的，倍数不是。** 开启 `-O2` 后编译器可能交换循环或向量化，差距会变化。

### 9.2 为什么是这个比值

按行遍历：16 个 `float` 共享一条 64 B 缓存行，缺失率 6.25%；按列遍历：每次一个新行，缺失率接近 100%。缺失率相差 16 倍，但实际耗时只差约 7.6 倍，因为按行时硬件预取器与向量化已把大部分延迟藏住，而按列时多个缺失可以重叠（内存级并行），把差距压低；比值是这些因素的综合，不是简单的 16。

### 9.3 缓存模拟代码

```python
# 教学示例：组相联缓存模拟（LRU）
def sim(addrs, lines=4, block=16, ways=1):
    sets = lines // ways
    cache = [[] for _ in range(sets)]
    hits = 0
    for a in addrs:
        blk = a // block
        s, tag = blk % sets, blk // sets
        if tag in cache[s]:
            hits += 1
            cache[s].remove(tag); cache[s].append(tag)   # 刷新 LRU
        else:
            if len(cache[s]) == ways:
                cache[s].pop(0)                          # 驱逐最久未用
            cache[s].append(tag)
    return hits

addrs = [0,4,8,12,16,20,64,68,0,4,64,0]
print(sim(addrs, ways=1), sim(addrs, ways=2))   # 6 9
```

## 10. 在七个项目里怎么用

| 项目 | 体系结构相关点 | 性质 |
|---|---|---|
| P4 | RK3588 上的 YOLOv5 INT8 推理：INT8 权重/激活使每个元素 1 字节，较 FP32 的访存量缩小到 1/4（例：约 25 M 参数的模型，FP32 权重 100 MB，INT8 约 25 MB，示例数字，非项目模型）；NPU 的算力较高，但输入预处理和输出后处理（NMS 等）在 CPU 上，常成为瓶颈。见 [INT8 量化与 RKNN 转换](../../01_项目课程/P4_OpenHarmony智能园艺系统/07_INT8量化与RKNN转换.md) | 项目内 |
| P4 | 取帧与预处理（缩放、颜色空间转换、归一化）是典型低 AI、带宽受限操作；合并步骤、减少拷贝（零拷贝）是对症手段。见 [V4L2 取帧与预处理](../../01_项目课程/P4_OpenHarmony智能园艺系统/05_RK3588边缘视觉_V4L2取帧与预处理.md) | 项目内 |
| P3/P7 | STM32F407/H7 是 Cortex-M4/M7 类内核（M7 带缓存与更深流水线，M4 较简单；具体型号以数据手册为准，此处按常见配置叙述）。控制环里的矩阵向量运算用 FPU（单精度）；DMA 与 CPU 访问同一内存时，在带缓存的 M7 上要注意缓存一致性（清理/失效），这是 H7 上 DMA 常见的坑 | 项目内（类比） |
| P1/P2/P5 | 上位机侧的 LQR/MPC/KDL/Pinocchio 计算和点云处理在 x86 或 ARM 的 CPU 上，矩阵库（Eigen）利用 SIMD 与缓存分块；小矩阵固定大小比动态大小更快（延伸知识） | 延伸 |
| 通用 | 若面试官追问"GPU 加速 MPC/SLAM"，属延伸：本人项目未使用 GPU 计算，可回答原理，不要声称做过 | 延伸 |

> **面试要点**：把"量化提速"解释为"算力与带宽双重收益"，比只说"INT8 更快"更有深度；同时诚实说明 NPU 的具体微架构你没有深入研究。

## 11. 常见错误

1. **只看大 O 不看常数与访存**：同阶算法因访存模式差一个数量级。
2. **以为缓存对所有语言布局一致**：C 是行优先，Fortran、MATLAB 是列优先。MATLAB 里按列遍历才快。
3. **链表遍历当数组用**：指针追逐无法预取，缓存缺失严重；性能关键处优先连续容器。
4. **忽略伪共享**：多线程把相邻变量放在同一缓存行并各自频繁写。
5. **误把流水线加深当成"速度翻倍"**：深流水线提高频率，但冒险与误预测惩罚也变大。
6. **以为 SIMD 总能加速 4 倍**：受带宽、对齐、尾部处理、依赖限制。
7. **GPU 上无视发散与合并访问**：warp 内分支不同、访问不连续导致性能暴跌。
8. **小任务上 GPU 反而慢**：数据传输与启动开销没有被摊薄。
9. **在 Roofline 上判断错受限类型**：对带宽受限的核函数堆算力无用。
10. **DMA 与缓存不一致**：带缓存的 MCU 上，DMA 写入的数据 CPU 读到旧缓存行；需要失效缓存或把缓冲区放在非缓存区域。

## 12. 英文面试问答

### Q1. Why is iterating a matrix row by row faster than column by column in C?

"C stores a 2D array in row-major order, so consecutive elements of a row are adjacent in memory. The CPU loads a whole cache line, typically 64 bytes, so one miss brings in 16 floats and the next 15 accesses hit. Going down a column jumps by the row length each time, so nearly every access touches a new cache line and misses. I measured about a seven to eight times difference on a 4096 by 4096 float array on my laptop, though the exact ratio depends on the machine."

中文要点：C 行优先；缓存行 64 B 含 16 个 float，行遍历命中 15/16；列遍历步长大，几乎每次缺失；本机实测约 7–8 倍，比值随机器变化。

**追问**：

- *"How would you fix a column-major access pattern you can't avoid?"* — "Tile the loops so a small block fits in cache and is reused, or transpose once into a layout that matches the access order."
- *"Does this matter for your robot code?"* — "Mostly on the Linux side, for point clouds and image preprocessing. On the STM32 the loops are tiny, but the same idea applies to buffer layout."（诚实说明范围。）

### Q2. What is a pipeline hazard and how are hazards handled?

"A hazard stops the next instruction from executing in the next cycle. Structural hazards come from resource conflicts, data hazards from dependencies like reading a register that a previous instruction hasn't written yet, and control hazards from branches. Hardware handles data hazards with forwarding and stalls, for example a one-cycle stall for a load followed by a use, and control hazards with branch prediction. Compilers help by scheduling independent instructions in between."

中文要点：结构、数据、控制三类；转发 + 停顿；加载-使用需停 1 周期；分支预测；编译器调度。

### Q3. How does a GPU differ from a CPU, and what is warp divergence?

"A CPU is designed for low latency on a few threads, with big caches and out-of-order logic. A GPU is designed for throughput: thousands of simple lanes, and it hides memory latency by switching between many warps. A warp is a group of threads, typically 32, that execute the same instruction in lockstep. If threads in a warp take different branches, the hardware runs both paths one after the other with some lanes masked off, so you lose parallelism. I would try to keep threads in a warp on the same path."

中文要点：CPU 低延迟，GPU 高吞吐；warp 锁步执行；分支发散时两路径串行执行；让同 warp 线程走同一路径。

### Q4. How does quantizing a model to INT8 speed it up on an edge device?

"There are two effects. First, INT8 values are a quarter of the size of FP32, so the weights and activations need a quarter of the memory traffic, and many edge workloads are bandwidth bound. Second, integer units can process more elements per instruction, in SIMD or on an NPU. The cost is some accuracy loss, so I check the accuracy and calibrate with representative data. In my gardening project I deployed a YOLOv5 INT8 model on RK3588 through RKNN."

中文要点：INT8 体积为 FP32 的 1/4，访存量下降；整数单元每条指令处理更多元素；代价是精度，需用代表性数据校准；项目：RK3588 + RKNN 的 YOLOv5 INT8。说具体加速倍数需要有实测，没有就不要报数。

## 13. 要点速记、自测题、相关链接

### 要点速记

1. 存储层次每级约慢一个数量级、大一个数量级；缓存靠时间与空间局部性生效。
2. $\text{AMAT}=t_{\text{hit}}+m\,t_{\text{pen}}$，缺失惩罚极大，5% 缺失率就能让平均访存变 6 倍。
3. 地址拆为 tag / index / offset；$S=C/(BW)$；三类缺失：强制、容量、冲突。
4. 行优先数组按行遍历快；列遍历每次新缓存行，本机实测约 7–8 倍差距。
5. 流水线提高吞吐不降低单指令延迟；冒险三类：结构、数据、控制。
6. $\text{CPI}=1+\text{分支频率}\times\text{误预测率}\times\text{惩罚}$；转发解决大部分数据冒险，加载-使用要停一周期。
7. SIMD 要求数据连续、无循环依赖、无别名；INT8 单寄存器装 FP32 的 4 倍元素。
8. GPU 靠大量线程掩盖延迟；warp 锁步，分支发散串行；访问合并是关键。
9. Roofline：$P=\min(P_{\text{peak}},\text{BW}\times\text{AI})$；低 AI 算子受带宽限制。
10. 融合算子与量化提升 AI 或减少字节数；预处理常是带宽受限的瓶颈。

### 自测题

1. 64 KiB、8 路、64 B 行的缓存，32 位地址，tag/index/offset 各多少位？
   *要点*：$S=65536/(64\times8)=128$，index 7，offset 6，tag 19。
2. 理想 CPI=1，分支 25%，误预测 10%，惩罚 12 周期，求 CPI。
   *要点*：$1+0.25\times0.1\times12=1.3$。
3. 用直接映射缓存（4 组，16 B 行）判断地址 0 与 64 是否冲突，并说明如何缓解。
   *要点*：块号 0 与 4，组号都是 0，冲突；提高相联度、调整布局或改变访问次序。
4. 一个核函数每个元素做 4 次 FLOP，读写共 16 B；设备 $P_{\text{peak}}=800$ GFLOP/s，$\text{BW}=40$ GB/s，受限于什么？上限多少？
   *要点*：AI=0.25；$40\times0.25=10$ GFLOP/s，带宽受限。
5. 为什么 GPU 上同一 warp 内线程访问连续地址更快？
   *要点*：硬件可合并成少量显存事务，提高有效带宽。

### 相关链接

- 上一章：[计算机网络](./03_计算机网络_TCP_UDP_以太网_时延.md)；下一章：[并发与内存模型](./05_并发与内存模型_锁_原子_无锁.md)
- 八股速查：[C++ 八股](../../03_八股知识/01_计算机基础/05_C++八股.md)、[C 语言与嵌入式八股](../../03_八股知识/01_计算机基础/04_C语言与嵌入式八股.md)
- 相关：[Cortex-M 体系](../11_嵌入式与通信/07_Cortex-M体系_启动_链接脚本_内存映射.md)、[卷积网络与目标检测 YOLO](../10_机器学习与深度学习/03_卷积网络与目标检测_YOLO.md)
- [模块 README](./README.md)
