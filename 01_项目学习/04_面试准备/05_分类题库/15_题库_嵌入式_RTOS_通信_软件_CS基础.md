# 分类题库 15：嵌入式、RTOS、通信与软件/计算机基础（104 题，C/C++、内存、MCU、中断与 DMA、FreeRTOS、CAN/UART/SPI/I2C、ROS2/DDS、Linux、操作系统、数据结构与算法、网络、并发、数据库、Git）

> **一句话定位**：你的 CV 技能栏写着 C/C++、Python、ROS/ROS2、Linux、STM32、FreeRTOS、CAN/FDCAN、OpenHarmony、RK3588、Git，教育背景里有 Data Structures and Algorithms 和 Operating Systems 两门课；CS 方向（CityU、CUHK-Shenzhen CS 等）和机器人方向的面试官都可能从这些"基础"问起。本文件把这类问题做成 104 道英文问答：每题有口语答案、中文翻译、两个追问，以及"追问到你的项目怎么接"的一句话，答案里的技术内容力求准确，项目内容只用 CV 与项目 README 里的事实。

**本章学完你能……**

- 用 60–90 秒讲清 `volatile`、栈与堆、DMA、中断、优先级反转、CAN 仲裁、DDS QoS、虚拟内存、TCP 三次握手、死锁、B 树索引、Git 合并与变基等高频概念，不背书、不含糊；
- 把每个概念接回你的项目：P3 的 STM32F407/FreeRTOS/CAN 内环，P7 的 STM32H7/FDCAN，P2 的 SocketCAN，P5/P6 的 ROS2 与 M-DDS，P4 的 TCP 与异步服务；
- 对"没做过"的内容（例如写 Linux 内核驱动、PREEMPT_RT）诚实说明边界，再讲清楚你理解的原理；
- 对算法题给出口述思路（复杂度、边界、测试），而不只是代码。

**前置知识**：[CV 事实基线](../../06_维护说明/CV事实基线.md)；[港硕面试真题调研](../00_港硕面试真题调研.md)；基础教材：[STM32 时钟、GPIO、中断、DMA](../../02_基础知识/11_嵌入式与通信/01_STM32_时钟_GPIO_中断_DMA.md)、[FreeRTOS](../../02_基础知识/11_嵌入式与通信/02_FreeRTOS实时系统.md)、[CAN/FDCAN](../../02_基础知识/11_嵌入式与通信/03_CAN_FDCAN与电机通信.md)、[UART/SPI/I2C/以太网/TCP](../../02_基础知识/11_嵌入式与通信/04_UART_SPI_I2C_以太网_TCP.md)、[Cortex-M 与启动](../../02_基础知识/11_嵌入式与通信/07_Cortex-M体系_启动_链接脚本_内存映射.md)、[看门狗与 OTA](../../02_基础知识/11_嵌入式与通信/08_电源管理_低功耗_看门狗_Bootloader_OTA.md)、[实时调度理论](../../02_基础知识/11_嵌入式与通信/11_实时调度理论_RMS_EDF_优先级反转.md)、[嵌入式 Linux](../../02_基础知识/11_嵌入式与通信/12_嵌入式Linux_设备树_驱动_RT-Preempt.md)、[Linux 与 Git](../../02_基础知识/12_机器人软件/01_Linux与Shell_Git.md)、[C++ 现代特性](../../02_基础知识/12_机器人软件/02_C++现代特性与工程实践.md)、[ROS2 Humble](../../02_基础知识/12_机器人软件/04_ROS2_Humble核心概念与ROS1对比.md)、[DDS](../../02_基础知识/12_机器人软件/05_DDS与分布式通信.md)；八股速查：[数据结构与算法](../../03_八股知识/01_计算机基础/01_数据结构与算法.md)、[操作系统](../../03_八股知识/01_计算机基础/02_操作系统.md)、[计算机网络](../../03_八股知识/01_计算机基础/03_计算机网络.md)、[C 语言与嵌入式](../../03_八股知识/01_计算机基础/04_C语言与嵌入式八股.md)、[C++](../../03_八股知识/01_计算机基础/05_C++八股.md)、[并发与实时](../../03_八股知识/01_计算机基础/07_并发多线程与实时系统八股.md)、[Linux、Git、构建](../../03_八股知识/01_计算机基础/08_Linux_Git_编译构建八股.md)、[数据库与系统设计](../../03_八股知识/01_计算机基础/09_数据库与系统设计基础.md)、[嵌入式与机电](../../03_八股知识/03_机器人学/06_嵌入式与机电八股.md)。

## 目录

1. 本文件怎么用
2. A　C/C++ 与内存（Q1–Q16）
3. B　MCU、中断、DMA 与启动（Q17–Q28）
4. C　RTOS 与实时性（Q29–Q42）
5. D　总线与通信：CAN、UART、SPI、I2C（Q43–Q56）
6. E　ROS2、DDS 与 Linux（Q57–Q68）
7. F　操作系统（Q69–Q78）
8. G　数据结构与算法（Q79–Q88）
9. H　网络、并发、数据库与 Git（Q89–Q104）
10. 自测与练习安排

---

## 1. 本文件怎么用

1. **每题的结构**：英文题目、中文题意、考察点、项目关联、英文回答（口语，第一人称或第三人称叙述）、中文翻译、两个追问、"追问到你的项目怎么接"、关键词与易错点。先遮住英文，只看中文题意，自己讲一遍，再对照。
2. **项目口径（必须记牢，与各项目 README 自查表一致）**：
   - **P3 RoboMaster 英雄**：STM32F407（开发板 C 型，168 MHz）+ FreeRTOS + bxCAN；两路 CAN 都是 1 Mbps；控制任务周期**约 3 ms（不说 1 kHz）**，IMU 任务约 1 kHz，detect 任务 10 ms；bxCAN 只有 3 个发送邮箱；云台板 CAN1 负载约 63%。
   - **P7 轮腿**：STM32H7 + FreeRTOS + FDCAN，**经典 CAN 模式**（8 字节帧，非 CAN FD），两路 1 Mbps；控制与估计 500 Hz（2 ms），IMU 1 kHz；IMU 与 CAN 缓冲放不可缓存段。
   - **P2**：ROS2 节点经 SocketCAN（本项目取值 1 Mbit/s）与电机通信；底盘固件基于 STM32H7/FDCAN。
   - **P1**：主循环 100 Hz，ROS 状态处理 + CAN 电机执行，驱动侧命令看门狗 100 ms。
   - **P5**：ROS2 Humble，EKF/AMCL + Hybrid-A* + TEB + MPC。
   - **P6**：OpenHarmony + C++，M-DDS 做设备发现、能力注册、状态同步、任务分发；IMU 100 Hz。
   - **P4**：RK3588（YOLOv5/RKNN INT8，V4L2 取帧）+ Hi3861（LiteOS-M，水泵滞回状态机）+ 服务端（TCP、WebSocket、asyncio、LangGraph）。
   - **红线**：读码发现的隐患只能说 "I noticed"，不是自己的贡献；CV 没有写你写过 Linux 内核驱动、配置过 PREEMPT_RT、实现过 FOC、写过链接脚本——被问到时按"理解原理、没有亲手做"诚实回答。
3. **算法题**：Part G 的答案是"口述思路"。面试官在白板或共享文档上要代码时，先说思路、复杂度、边界，再写，最后用一个小例子走一遍。
4. **题量与优先级**：104 题不必全背。第一轮抓每个部分带 ★ 的题（高频），第二轮开口练习，第三轮练追问。

---

## 2. A　C/C++ 与内存（Q1–Q16）

### Q1. What does `volatile` do, and when do you need it? ★
**中文题意**：`volatile` 的作用？什么时候需要？
**考察点**：编译器优化与硬件/并发可见性的区别。
**项目关联**：P3、P7（寄存器、ISR 与任务共享标志）。
**English answer（口语）**：
> `volatile` tells the compiler that a variable can change outside the normal program flow, so every read and write must really happen and must not be cached in a register or removed. I use it for memory-mapped hardware registers, and for flags shared between an interrupt handler and the main loop. But it does not make operations atomic and it does not give memory ordering between threads. For a counter shared between tasks, I need a critical section, an atomic type, or a mutex, not just `volatile`.

**中文翻译**：`volatile` 告诉编译器变量可能在正常程序流之外改变，所以每次读写都必须真正发生，不能缓存在寄存器里或被优化掉。我用它修饰内存映射的硬件寄存器，以及中断处理函数与主循环共享的标志。但它不保证原子性，也不提供线程间的内存顺序。任务之间共享的计数器，需要临界区、原子类型或互斥量，而不是只靠 `volatile`。
**追问 1**：*Can a variable be both `const` and `volatile`?* Yes, for example a read-only status register: the program must not write it, but the hardware changes it.
　中文：可以，例如只读状态寄存器：程序不能写，但硬件会改变它。
**追问 2**：*Is `volatile` enough for a flag set in an ISR and read in a task?* For a single-byte flag on Cortex-M, usually yes in practice; for multi-word data you need a snapshot with interrupts masked or a sequence counter.
　中文：对 Cortex-M 上单字节标志，实践中通常够；多字数据需要关中断快照或序号计数。
**如果追问到你的项目**：*In the robot firmware, data from the IMU and the CAN receive interrupt are written into a shared structure and the control task takes a snapshot each cycle, so it never reads half-updated data.*
　中文：机器人固件里，IMU 与 CAN 接收中断写入共享结构，控制任务每个周期取一次快照，避免读到更新一半的数据。
**关键词与易错点**：volatile、memory-mapped register、atomic、memory barrier。不要说"volatile 保证线程安全"。

### Q2. What is the difference between `const char *p`, `char * const p` and `const char * const p`?
**中文题意**：三种 `const` 指针的区别？
**考察点**：指针与常量的声明读法。
**项目关联**：P3、P7（常量表、只读配置放 Flash）。
**English answer（口语）**：
> I read declarations from the name outward. `const char *p` means p is a pointer to a constant char: I can move the pointer but I cannot change the data through it. `char * const p` means p is a constant pointer to a char: the pointer cannot be reassigned but the data can change. `const char * const p` means neither can change. On a microcontroller, global `const` data is placed in flash, which saves RAM, so marking lookup tables and string literals as `const` is a real memory optimisation, not just style.

**中文翻译**：我从名字往外读声明。`const char *p` 是指向常量字符的指针：指针可以移动，但不能通过它改数据。`char * const p` 是常量指针：指针不能重新赋值，但数据可以改。`const char * const p` 两者都不能改。在单片机上，全局 `const` 数据放在 Flash 里，能节省 RAM，所以给查表和字符串字面量加 `const` 是实际的内存优化，不只是风格。
**追问 1**：*Where does a `const` global go on Cortex-M?* In `.rodata`, which the linker places in flash.
　中文：在 `.rodata` 段，链接器把它放进 Flash。
**追问 2**：*Is `const` the same as a compile-time constant?* Not always; in C a `const` variable is not a constant expression, while `constexpr` in C++ guarantees compile-time evaluation.
　中文：不一定；C 里 `const` 变量不是常量表达式，C++ 的 `constexpr` 保证编译期求值。
**如果追问到你的项目**：*Lookup tables, such as the gain-scheduling polynomial coefficients, are naturally const data that stay in flash.*
　中文：增益调度的多项式系数这类查表数据，自然是放在 Flash 里的 const 数据。
**关键词与易错点**：const correctness、.rodata、flash。注意 `const` 不等于线程安全。

### Q3. Explain stack versus heap. When do you avoid the heap in embedded code? ★
**中文题意**：栈与堆的区别？嵌入式里什么时候避免用堆？
**考察点**：内存管理与确定性。
**项目关联**：P3、P7（FreeRTOS 堆、任务栈）。
**English answer（口语）**：
> The stack holds local variables and return addresses; allocation and release are automatic and very fast, but the size is fixed and a deep call chain can overflow it. The heap is for dynamic allocation with `malloc` or `new`; it is flexible, but allocation time varies, memory can fragment, and a leak slowly eats RAM. In real-time embedded code I avoid the heap after start-up. I allocate objects once at initialisation, or use static buffers and pools, so that timing is predictable and I cannot run out of memory in the field after hours of running.

**中文翻译**：栈保存局部变量和返回地址，分配释放自动且很快，但大小固定，调用链太深会溢出。堆用于 `malloc` 或 `new` 的动态分配，灵活，但分配时间不固定、可能碎片化，泄漏会慢慢吃光 RAM。实时嵌入式代码里，启动之后我避免使用堆：在初始化时一次性分配对象，或者用静态缓冲区和内存池，这样时间可预测，也不会在现场运行几小时后耗尽内存。
**追问 1**：*What is fragmentation?* Free memory exists but is split into pieces too small for a request, so allocation fails even though the total is enough.
　中文：总空闲量够，但被切成太小的块，满足不了一次请求，分配照样失败。
**追问 2**：*How do you check stack usage in FreeRTOS?* `uxTaskGetStackHighWaterMark` returns the minimum free stack words ever seen; keep a margin, say 25 percent.
　中文：`uxTaskGetStackHighWaterMark` 返回历史最小剩余栈字数；至少留约 25% 余量。
**如果追问到你的项目**：*For the wheel-legged controller the design target was a four-kilobyte control-task stack with a measured high-water mark well below it, and no allocation inside the control loop.*
　中文：轮腿控制器的设计目标是控制任务栈 4 KB，实测水位远低于它，控制环内不做分配。
**关键词与易错点**：stack overflow、fragmentation、static allocation、high-water mark。不要说"嵌入式绝对不能用堆"，而是"运行期实时路径避免"。

### Q4. Describe the memory layout of a C program and the sections `.text`, `.data`, `.bss`. ★
**中文题意**：C 程序的内存布局，`.text`、`.data`、`.bss` 各是什么？
**考察点**：链接与启动的基础。
**项目关联**：P3、P7（启动代码、Flash/RAM 占用）。
**English answer（口语）**：
> `.text` is the code, stored in flash. `.rodata` is read-only data such as constants. `.data` holds initialised globals and statics: the initial values are stored in flash, and the start-up code copies them to RAM before `main`. `.bss` holds globals that are zero or uninitialised, so it takes no flash space and the start-up code just clears that RAM region. Besides these, there are the stack and the heap in RAM. So flash usage is text plus data, and static RAM usage is data plus bss.

**中文翻译**：`.text` 是代码，存在 Flash。`.rodata` 是常量等只读数据。`.data` 存已初始化的全局和静态变量：初值存在 Flash，启动代码在 `main` 之前把它们拷到 RAM。`.bss` 存零初值或未初始化的全局变量，不占 Flash，启动代码只需把这块 RAM 清零。另外 RAM 里还有栈和堆。所以 Flash 占用 = text + data，静态 RAM 占用 = data + bss。
**追问 1**：*What goes wrong if the start-up code forgets to copy `.data`?* Initialised globals contain random values.
　中文：已初始化的全局变量会是随机值。
**追问 2**：*How do you see the sizes?* Run `arm-none-eabi-size` on the ELF, or read the map file.
　中文：对 ELF 运行 `arm-none-eabi-size`，或看 map 文件。
**如果追问到你的项目**：*I have read and used the generated linker scripts, for example to put the DMA buffers into a dedicated section, but I did not write a linker script from scratch.*
　中文：我读过也用过生成的链接脚本，例如把 DMA 缓冲放进专用段，但没有从零写过链接脚本。
**关键词与易错点**：LMA/VMA、start-up code、map file。不要说 `.bss` 占 Flash。

### Q5. Why is `malloc` often discouraged in real-time code?
**中文题意**：为什么实时代码里常不建议用 `malloc`？
**考察点**：确定性、碎片、线程安全。
**项目关联**：P3、P7、P6。
**English answer（口语）**：
> There are four reasons. First, execution time is not bounded: the allocator may search a free list for a long time. Second, fragmentation can make an allocation fail later even if enough total memory is free. Third, the standard allocator may not be thread-safe or interrupt-safe, so I must never call it from an ISR. Fourth, failure handling: if it returns null in the middle of a control loop, there is no good recovery. The alternatives are static allocation, fixed-size memory pools with constant-time operations, or a deterministic allocator such as FreeRTOS heap four, used only at start-up.

**中文翻译**：有四个原因。第一，执行时间没有上界：分配器可能在空闲链表里搜很久。第二，碎片：即使总空闲量足够，之后的分配也可能失败。第三，标准分配器可能不是线程安全或中断安全的，绝不能在 ISR 里调用。第四，失败处理：控制环中途返回空指针，没有好的恢复办法。替代方案是静态分配、常数时间的固定块内存池，或只在启动时使用的确定性分配器（例如 FreeRTOS 的 heap_4）。
**追问 1**：*What is a memory pool?* A preallocated array of equal-size blocks with a free list, so allocation and release take constant time and cannot fragment.
　中文：预分配的等大小块数组加空闲链表，分配释放常数时间，无碎片。
**追问 2**：*Can you call `malloc` in C++ containers on the control path?* Not safely; reserve capacity in advance or use fixed-size containers.
　中文：不安全；事先 reserve 容量，或用固定大小容器。
**如果追问到你的项目**：*On the ROS2 side, nodes allocate freely at higher rates of tens of hertz; on the STM32 control path, nothing is allocated after initialisation.*
　中文：ROS2 侧节点按几十赫兹的频率可以自由分配；STM32 控制路径上初始化之后不做分配。
**关键词与易错点**：determinism、fragmentation、memory pool。不要说"malloc 一定慢"，要说"时间不可预测"。

### Q6. What is structure padding and why does it matter for communication? ★
**中文题意**：结构体填充是什么？对通信有什么影响？
**考察点**：对齐、`packed`、序列化。
**项目关联**：P3（板间 CAN 与视觉串口协议）、P7（CAN 帧打包）。
**English answer（口语）**：
> The compiler inserts padding bytes so that each member is aligned to its natural boundary; for example, a `char` followed by an `int` takes eight bytes on a 32-bit core, because three padding bytes follow the char. That is fine inside a program but dangerous for protocols: if I copy a struct directly into a CAN or serial frame, the layout depends on the compiler and the other side may read the wrong offsets. For protocols I either use packed structs with care about unaligned access, or, better, serialise field by field with fixed width types and an explicit byte order.

**中文翻译**：编译器插入填充字节，使每个成员对齐到自然边界；例如在 32 位核上，一个 `char` 后面跟一个 `int` 占 8 字节，因为 char 后有 3 字节填充。程序内部没问题，但对协议很危险：如果我把结构体直接拷进 CAN 或串口帧，布局取决于编译器，对端可能读错偏移。协议里我要么小心地用 packed 结构体（注意非对齐访问），要么更好的做法是逐字段按固定宽度类型和明确的字节序序列化。
**追问 1**：*What is the risk of a packed struct on Cortex-M?* Unaligned access; Cortex-M4 and M7 usually handle it with a small penalty, but M0 faults, and a DMA may require alignment.
　中文：非对齐访问；M4/M7 通常能处理但有性能代价，M0 会出错，DMA 也可能要求对齐。
**追问 2**：*How do you guard against version mismatch between two boards?* Put a version byte and a checksum in the frame, and share one header file.
　中文：帧里放版本字节和校验，两端共享同一头文件。
**如果追问到你的项目**：*In the RoboMaster robot the two boards exchange fixed-layout frames over CAN; a byte-order mismatch once gave garbage speed values, so the frame definition went into a shared header with little-endian fixed.*
　中文：RoboMaster 两块板通过 CAN 交换固定布局帧；字节序不一致曾导致速度值乱码，所以把帧定义放进共享头文件并固定小端。
**关键词与易错点**：alignment、packed、endianness、serialisation。不要说"直接 memcpy 结构体最方便"。

### Q7. What are the differences between pointers and references in C++? Between arrays and pointers in C?
**中文题意**：C++ 指针与引用的区别？C 里数组与指针的区别？
**考察点**：语言基础。
**项目关联**：P6（C++ 设备抽象）。
**English answer（口语）**：
> A reference is an alias that must be bound at initialisation, cannot be null, and cannot be rebound. A pointer can be null, can be reassigned, and supports arithmetic. I use references for parameters that must exist, and pointers or `std::optional` when absence is allowed. In C, an array name decays to a pointer to its first element when passed to a function, so the size is lost; `sizeof` of the array gives the whole size only in the scope where it is declared. That is why functions that take arrays also take a length.

**中文翻译**：引用是别名，必须在初始化时绑定，不能为空，也不能重新绑定。指针可以为空、可以重新赋值，并支持算术运算。必须存在的参数我用引用，允许缺省时用指针或 `std::optional`。C 里数组名传给函数时退化为指向首元素的指针，大小信息丢失；`sizeof` 只有在数组声明的作用域里才给出整体大小。所以接收数组的函数通常还要传长度。
**追问 1**：*What is a dangling reference?* A reference to an object that has been destroyed, such as returning a reference to a local variable.
　中文：引用的对象已被销毁，例如返回局部变量的引用。
**追问 2**：*Why pass large objects by const reference?* To avoid copying while promising not to modify.
　中文：避免拷贝，同时承诺不修改。
**如果追问到你的项目**：*In the P6 device abstraction, interfaces take references to capability objects whose lifetime is owned by the registry, so ownership stays clear.*
　中文：P6 的设备抽象里，接口接收由注册表持有生命周期的能力对象的引用，所有权清楚。
**关键词与易错点**：decay、dangling、nullability。不要说"引用就是常量指针"（语义上相近但不等同）。

### Q8. What is undefined behaviour? Give embedded-relevant examples. ★
**中文题意**：什么是未定义行为？举与嵌入式相关的例子。
**考察点**：C/C++ 风险意识。
**项目关联**：P3、P7（偶发崩溃）。
**English answer（口语）**：
> Undefined behaviour means the language standard places no requirement on what happens, so the compiler may assume it never occurs and optimise accordingly; the result can be a crash, a wrong value or something that works until you change the optimisation level. Examples are signed integer overflow, reading an uninitialised variable, out-of-bounds array access, dereferencing a null or dangling pointer, violating strict aliasing, and shifting by too many bits. In embedded systems these create the classic "it only fails in the release build" bugs. I use compiler warnings, static analysis, sanitisers on the host, and unit tests to catch them.

**中文翻译**：未定义行为是指语言标准对结果没有任何要求，编译器可以假设它永不发生并据此优化；结果可能是崩溃、错误值，或者在换优化级别之前一直"正常"。例子包括有符号整数溢出、读取未初始化变量、数组越界、解引用空指针或悬空指针、违反严格别名规则、移位位数过大。在嵌入式系统里，这就是经典的"只有 release 版本才出错"的 bug。我用编译器警告、静态分析、主机上的 sanitizer 和单元测试来抓。
**追问 1**：*Is unsigned overflow undefined?* No, unsigned arithmetic wraps modulo two to the power n; signed overflow is undefined.
　中文：不是，无符号运算按 2 的 n 次方取模回绕；有符号溢出才是未定义。
**追问 2**：*How can you detect it on a PC?* Build with AddressSanitizer and UndefinedBehaviorSanitizer and run the algorithm tests.
　中文：用 AddressSanitizer 和 UBSan 构建并运行算法测试。
**如果追问到你的项目**：*Control algorithms can be compiled and tested on the host with sanitisers before they go to the STM32, which is a cheap way to find indexing errors.*
　中文：控制算法可以先在主机上带 sanitizer 编译测试，再放上 STM32，这是发现下标错误的廉价办法。
**关键词与易错点**：UB、sanitizer、strict aliasing。不要说"UB 就是崩溃"，它也可能"看起来正常"。

### Q9. What does `static` mean in C, and in C++ classes?
**中文题意**：`static` 在 C 和 C++ 类里的含义？
**考察点**：存储期与链接性。
**项目关联**：P3、P6。
**English answer（口语）**：
> In C, `static` has two meanings. For a local variable it gives static storage duration: the variable keeps its value between calls and lives in `.data` or `.bss`. For a global variable or function it gives internal linkage: the name is visible only within that source file, which helps avoid name clashes. In a C++ class, a static member belongs to the class rather than an object, and a static member function has no `this` pointer. One caution: a static local makes a function non-reentrant, so it is unsafe if two tasks call it at the same time without protection.

**中文翻译**：C 里 `static` 有两层含义。对局部变量，它给出静态存储期：变量在多次调用间保持值，位于 `.data` 或 `.bss`。对全局变量或函数，它给出内部链接性：名字只在本源文件可见，避免命名冲突。C++ 类里，静态成员属于类而不是对象，静态成员函数没有 `this` 指针。需注意：静态局部变量使函数不可重入，两个任务无保护地同时调用就不安全。
**追问 1**：*What does reentrant mean?* A function can be interrupted and called again before the first call finishes without corrupting data, typically by using only local variables or protected state.
　中文：函数在执行中被打断并再次被调用也不会破坏数据，通常只用局部变量或有保护的状态。
**追问 2**：*Is the initialisation of a function-local static thread-safe in C++11?* Yes, the standard requires it.
　中文：是，C++11 标准要求其初始化线程安全。
**如果追问到你的项目**：*For example, a PID controller that keeps its integral in a function-static variable cannot be instantiated twice for two wheels, so I keep state in a struct per controller instead.*
　中文：例如把积分量放在函数静态变量里的 PID，没法给两个轮子各用一份，所以我把状态放进每个控制器自己的结构体。
**关键词与易错点**：storage duration、linkage、reentrancy。不要把"static 变量线程安全"当成事实。

### Q10. What is a function pointer and how do you use callbacks in firmware?
**中文题意**：函数指针是什么？固件里怎么用回调？
**考察点**：接口设计。
**项目关联**：P3、P7、P6。
**English answer（口语）**：
> A function pointer stores the address of a function, so I can choose behaviour at run time. In firmware I use them for callbacks, for example a driver that calls a user function when a CAN frame arrives, or a table of command handlers indexed by message ID. To keep it safe, I pass a context pointer along with the callback, check for null before calling, and keep callbacks short if they run in an interrupt. In C++ I would prefer `std::function` or a template, which are type-safe and can capture state.

**中文翻译**：函数指针保存函数地址，使我能在运行时选择行为。固件里用于回调，例如驱动在 CAN 帧到达时调用用户函数，或者以消息 ID 为索引的命令处理函数表。为了安全，我会随回调传一个上下文指针，调用前检查是否为空，在中断里运行的回调要短。C++ 里我更愿意用 `std::function` 或模板，类型安全且可捕获状态。
**追问 1**：*What is the Thumb bit in a function pointer?* On Cortex-M the lowest bit of an address used for a branch is 1 to indicate Thumb state; a pointer with bit zero would cause a fault.
　中文：Cortex-M 上分支地址最低位为 1 表示 Thumb 状态，为 0 会触发故障。
**追问 2**：*What is the downside of callbacks inside interrupts?* Hidden execution time and the risk of calling non-ISR-safe functions.
　中文：隐藏的执行时间，以及可能调用了非中断安全的函数。
**如果追问到你的项目**：*A CAN receive callback that only decodes the frame and writes a timestamped record, then lets the control task do the rest, is the pattern I use.*
　中文：CAN 接收回调只做解码并写入带时间戳的记录，其余交给控制任务，这是我用的模式。
**关键词与易错点**：callback、context pointer、ISR-safe。不要在回调里做阻塞或 printf。

### Q11. What is RAII and why does it matter in C++? ★
**中文题意**：RAII 是什么？为什么重要？
**考察点**：资源管理。
**项目关联**：P6（C++ 设备抽象）。
**English answer（口语）**：
> RAII, resource acquisition is initialisation, means a resource is owned by an object: it is acquired in the constructor and released in the destructor. Because destructors run automatically when the object goes out of scope, even when an exception is thrown or a function returns early, there are no leaks. Examples are `std::unique_ptr` for memory, `std::lock_guard` for mutexes, and file or device handle wrappers. In my device abstraction work, wrapping a device handle in an object means a failed initialisation path cannot forget to release the hardware.

**中文翻译**：RAII 即资源获取即初始化：资源由对象拥有，构造函数获取，析构函数释放。对象离开作用域时析构函数自动运行，即使抛异常或提前返回也不会泄漏。例子有管理内存的 `std::unique_ptr`、管理互斥量的 `std::lock_guard`，以及文件或设备句柄的封装。在我的设备抽象工作里，把设备句柄封装成对象，意味着初始化失败的路径不会忘记释放硬件。
**追问 1**：*Why is `lock_guard` better than manual lock and unlock?* The mutex is released on every exit path, including early returns and exceptions.
　中文：每条退出路径（包括提前返回和异常）都会释放互斥量。
**追问 2**：*What is the rule of five?* If a class manages a resource and defines one of destructor, copy or move operations, it should consider defining all five, or delete the ones it does not support.
　中文：类管理资源并自定义析构、拷贝或移动之一时，应考虑五个都定义，或显式删除不支持的。
**如果追问到你的项目**：*In P6, the capability objects wrap driver handles in this way, and registering or unregistering is tied to object lifetime.*
　中文：P6 里能力对象就这样封装驱动句柄，注册与注销绑定到对象生命周期。
**关键词与易错点**：RAII、destructor、ownership、exception safety。

### Q12. How do virtual functions work, and what is the cost?
**中文题意**：虚函数怎么实现？代价是什么？
**考察点**：vtable、多态。
**项目关联**：P6（统一能力接口）。
**English answer（口语）**：
> A class with virtual functions has a hidden table of function pointers, the vtable, and each object holds a pointer to it, the vptr. A virtual call loads the vptr, then the function address from the table, then jumps; so the cost is one extra indirection and it usually prevents inlining. The benefit is run-time polymorphism: code written against a base interface can drive different hardware. For a hard real-time inner loop I would avoid virtual calls, or devirtualise by using templates; for a device abstraction layer called at tens of hertz, the cost is negligible.

**中文翻译**：有虚函数的类有一张隐藏的函数指针表，即 vtable，每个对象持有指向它的指针 vptr。虚调用先取 vptr，再从表里取函数地址，然后跳转；代价是多一次间接寻址，通常还阻止内联。好处是运行时多态：基于基类接口写的代码能驱动不同硬件。硬实时内环里我会避免虚调用，或用模板去虚拟化；以几十赫兹调用的设备抽象层，这点代价可以忽略。
**追问 1**：*Why must a base class destructor be virtual?* Deleting a derived object through a base pointer without a virtual destructor is undefined behaviour and skips the derived destructor.
　中文：通过基类指针删除派生类对象而基类析构非虚，是未定义行为，且不会调用派生析构。
**追问 2**：*What is object slicing?* Copying a derived object into a base object by value loses the derived part.
　中文：按值把派生对象拷给基类对象，派生部分被丢弃。
**如果追问到你的项目**：*The unified capability model is an interface that different sensors, actuators and controllers implement, so motion code does not depend on a specific driver.*
　中文：统一能力模型是由不同传感器、执行器、控制器实现的接口，运动代码不依赖具体驱动。
**关键词与易错点**：vtable、vptr、virtual destructor、slicing。

### Q13. Explain move semantics and when you would use `std::move`.
**中文题意**：解释移动语义，什么时候用 `std::move`？
**考察点**：C++11 基础。
**项目关联**：P6、P2（大数据传递）。
**English answer（口语）**：
> Move semantics let an object transfer its resources, such as a buffer pointer, to another object instead of copying them. A move constructor takes an rvalue reference, steals the pointer and leaves the source in a valid but empty state. `std::move` itself does not move anything; it only casts an object to an rvalue so that the move constructor is chosen. I use it when returning or passing large containers or buffers that I will not use again, which avoids deep copies. After a move I should not use the moved-from object except to assign or destroy it.

**中文翻译**：移动语义让对象把资源（例如缓冲区指针）转移给另一个对象，而不是拷贝。移动构造函数接收右值引用，拿走指针，并让源对象处于有效但为空的状态。`std::move` 本身什么也不移动，它只是把对象转换成右值，从而选中移动构造函数。我在返回或传递以后不再使用的大容器或缓冲区时使用它，避免深拷贝。移动之后除了赋值或销毁，不应再使用被移走的对象。
**追问 1**：*What is copy elision?* The compiler constructs a returned object directly in the caller's storage, so no copy or move occurs; guaranteed in some cases since C++17.
　中文：编译器直接在调用者存储中构造返回对象，不发生拷贝或移动；C++17 起在某些情况下是保证的。
**追问 2**：*Should you write `return std::move(local);`?* Usually not, it can prevent elision.
　中文：通常不要，这样写可能阻止拷贝省略。
**如果追问到你的项目**：*Passing point clouds or image buffers between pipeline stages by moving them is the typical use in the vision and mapping parts of a robot stack.*
　中文：在流水线阶段间移动点云或图像缓冲，是机器人视觉和建图部分的典型用法。
**关键词与易错点**：rvalue reference、move constructor、copy elision。不要说 `std::move` "移动了东西"。

### Q14. How do smart pointers differ: `unique_ptr`, `shared_ptr`, `weak_ptr`? ★
**中文题意**：三种智能指针的区别？
**考察点**：所有权。
**项目关联**：P6、ROS2（rclcpp 的 SharedPtr）。
**English answer（口语）**：
> `unique_ptr` expresses exclusive ownership: it cannot be copied, only moved, and has no overhead compared with a raw pointer. `shared_ptr` expresses shared ownership with a reference count; the object is destroyed when the last owner goes away. It costs an atomic counter and a control block. `weak_ptr` observes a `shared_ptr` object without owning it, which breaks reference cycles. My rule is: default to `unique_ptr`, use `shared_ptr` only when ownership is genuinely shared, and use raw pointers or references only for non-owning access.

**中文翻译**：`unique_ptr` 表示独占所有权：不能拷贝，只能移动，相对裸指针没有额外开销。`shared_ptr` 表示共享所有权，带引用计数；最后一个持有者离开时对象销毁，代价是原子计数和控制块。`weak_ptr` 观察 `shared_ptr` 管理的对象而不拥有它，用来打破循环引用。我的规则是：默认用 `unique_ptr`，只有所有权真正共享时才用 `shared_ptr`，裸指针或引用只用于不拥有的访问。
**追问 1**：*How does a reference cycle leak memory?* Two objects hold `shared_ptr` to each other, so the counts never reach zero; use `weak_ptr` for one direction.
　中文：两个对象互相持有 `shared_ptr`，计数永不归零；其中一个方向用 `weak_ptr`。
**追问 2**：*Is `shared_ptr` thread-safe?* The control block counting is thread-safe, but access to the pointed-to object is not.
　中文：控制块的计数是线程安全的，但被指对象的访问不是。
**如果追问到你的项目**：*In ROS2 C++, messages are passed as shared pointers, and with intra-process communication the same message object can be shared without copying.*
　中文：ROS2 C++ 里消息以共享指针传递，进程内通信时同一个消息对象可以不拷贝地共享。
**关键词与易错点**：ownership、reference count、cycle。不要到处用 `shared_ptr`。

### Q15. What is endianness, and how do you serialise a 32-bit value into a CAN frame? ★
**中文题意**：什么是字节序？怎样把 32 位数写进 CAN 帧？
**考察点**：协议编码。
**项目关联**：P3、P7、P2。
**English answer（口语）**：
> Endianness is the byte order of a multi-byte value in memory. Little-endian stores the least significant byte first, which is what Cortex-M and x86 use; network protocols traditionally use big-endian. When I pack a value into a frame, I do not copy the memory directly. I shift and mask: byte zero is the value and 0xFF, byte one is the value shifted right by eight and masked, and so on, in a byte order the protocol defines. For fractional values like a torque, motors often use a fixed-point encoding, for example a 12-bit unsigned integer mapped linearly to a range.

**中文翻译**：字节序是多字节数值在内存中的字节顺序。小端把最低有效字节放在前面，Cortex-M 和 x86 用小端；网络协议传统上用大端。往帧里打包数值时，我不直接拷贝内存，而是移位和掩码：第 0 字节是值与 0xFF，第 1 字节是值右移 8 位再掩码，依此类推，字节顺序按协议规定。对力矩这类带小数的量，电机常用定点编码，例如把 12 位无符号整数线性映射到一个范围。
**追问 1**：*How would you encode a torque in the range minus ten to ten into 12 bits?* Compute `(t + 10) / 20 * 4095`, round, clamp to the range, and send; the decoder reverses it.
　中文：计算 `(t + 10) / 20 * 4095`，取整并夹紧后发送；解码端做逆运算。
**追问 2**：*What is the resolution?* Twenty divided by 4095, about 4.9 milli-newton-metres.
　中文：20 除以 4095，约 4.9 mN·m。
**如果追问到你的项目**：*In the wheel-legged robot, the MIT-style frame packs position, velocity, gain and torque as fixed-point fields in eight bytes, with the encoding ranges set in the design; the torque range is plus or minus ten newton-metres.*
　中文：轮腿机器人的 MIT 风格帧把位置、速度、增益、力矩按定点字段打包进 8 字节，编码范围在设计中规定，力矩范围 ±10 N·m。
**关键词与易错点**：endianness、fixed-point、clamp。不要忘记对输入夹紧，否则溢出会翻转符号。

### Q16. Why is `float` versus `double` important on a microcontroller?
**中文题意**：单片机上 `float` 与 `double` 为什么重要？
**考察点**：FPU 与性能。
**项目关联**：P3（F407 单精度 FPU）、P7（H7）。
**English answer（口语）**：
> Many Cortex-M4 and M7 parts have a hardware FPU for single precision only. Single-precision `float` operations run in a few cycles, but `double` is done in software and can be tens of times slower. A common mistake is writing `0.5` instead of `0.5f`, or calling `sin` instead of `sinf`, which silently promotes the whole expression to double. I enable the hardware FPU and compile with the matching float ABI. I also remember that float has about seven decimal digits, so for long-running accumulation, like integrating time, I use integer ticks or a double on the host.

**中文翻译**：很多 Cortex-M4/M7 只有单精度硬件 FPU。单精度 `float` 运算几个周期，而 `double` 靠软件实现，可能慢几十倍。常见错误是写 `0.5` 而不是 `0.5f`，或调用 `sin` 而不是 `sinf`，整个表达式被悄悄提升为 double。我会启用硬件 FPU 并使用匹配的浮点 ABI。还要记得 float 只有约 7 位十进制有效数字，长时间累加（比如累积时间）要用整数节拍，或在主机上用 double。
**追问 1**：*Why can float time accumulation drift?* As the value grows, the spacing between floats grows, so adding a small step is eventually lost.
　中文：数值变大后相邻 float 的间隔变大，加上一个小步长最终被吞掉。
**追问 2**：*What happens if the FPU is not enabled?* The first floating-point instruction raises a usage fault.
　中文：第一条浮点指令触发 UsageFault。
**如果追问到你的项目**：*All controller code on the STM32 targets is written in single precision with the hardware FPU, and the cycle counter is used to check the execution time against the budget.*
　中文：STM32 上的控制器代码全部用单精度和硬件 FPU，并用周期计数器核对执行时间是否在预算内。
**关键词与易错点**：FPU、single precision、implicit promotion。不要说 float 在所有 MCU 上都快（Cortex-M0 没有 FPU）。

---

## 3. B　MCU、中断、DMA 与启动（Q17–Q28）

### Q17. What are the rules for writing an interrupt service routine? ★
**中文题意**：写 ISR 有哪些规则？
**考察点**：中断设计。
**项目关联**：P3（CAN 接收、UART/DMA 中断）、P7。
**English answer（口语）**：
> Keep it short and deterministic. In the ISR I clear the interrupt flag, read or write the minimum hardware data, record a timestamp if needed, and hand the rest to a task through a queue, a task notification or a semaphore. I do not call blocking functions, `printf`, `malloc`, or functions that are not safe in interrupt context. Shared variables must be `volatile` and protected for multi-word access. In an RTOS I use the `FromISR` API versions and make sure the interrupt priority is not higher than the allowed maximum, otherwise the kernel cannot mask it.

**中文翻译**：保持短小且确定。ISR 里我清中断标志，读写最少的硬件数据，必要时记时间戳，其余通过队列、任务通知或信号量交给任务。不调用阻塞函数、`printf`、`malloc`，或不能在中断上下文中使用的函数。共享变量要 `volatile`，多字访问还需保护。在 RTOS 里使用 `FromISR` 版 API，并确保中断优先级不高于允许的最大值，否则内核无法屏蔽它。
**追问 1**：*Why must you clear the flag?* Otherwise the interrupt fires again immediately after returning.
　中文：否则返回后中断立刻再次触发。
**追问 2**：*What is `portYIELD_FROM_ISR` for?* If the ISR woke a higher-priority task, it requests a context switch on exit so that task runs immediately.
　中文：ISR 唤醒了更高优先级任务时，请求退出时立即切换，让该任务马上运行。
**如果追问到你的项目**：*In the wheel-legged firmware the CAN receive interrupt only decodes the frame and timestamps it, which takes tens of microseconds, and the control task consumes the data from a snapshot.*
　中文：轮腿固件里 CAN 接收中断只解码并打时间戳，耗时几十微秒，控制任务从快照里取数据。
**关键词与易错点**：short ISR、deferred processing、FromISR、flag clearing。不要在 ISR 里等待。

### Q18. How does the NVIC priority scheme work on Cortex-M? Lower number means what?
**中文题意**：Cortex-M 的 NVIC 优先级怎么工作？数值小意味着什么？
**考察点**：抢占优先级与子优先级。
**项目关联**：P3、P7。
**English answer（口语）**：
> In the NVIC, a lower number means a higher priority. The priority byte is split into preemption priority and sub-priority; a higher preemption level can interrupt a running ISR, while the sub-priority only decides which of two pending interrupts of the same level runs first. STM32 implements only the upper four bits, so there are sixteen levels. With FreeRTOS, interrupts that call the kernel `FromISR` functions must have a numeric value no smaller than `configMAX_SYSCALL_INTERRUPT_PRIORITY`. Interrupts above that are never masked by the kernel but must not call it.

**中文翻译**：NVIC 里数值越小优先级越高。优先级字节分成抢占优先级和子优先级；更高的抢占级可以打断正在运行的 ISR，子优先级只决定同级的两个挂起中断谁先运行。STM32 只实现高四位，共 16 级。使用 FreeRTOS 时，调用内核 `FromISR` 函数的中断，其数值必须不小于 `configMAX_SYSCALL_INTERRUPT_PRIORITY`；比它更高的中断不会被内核屏蔽，但也不能调用内核。
**追问 1**：*What goes wrong if an ISR calling `FromISR` has too high a priority?* The kernel can be interrupted inside critical sections, causing random crashes.
　中文：内核临界区内可能被它打断，导致随机崩溃。
**追问 2**：*What is tail-chaining?* When an ISR finishes and another is pending, the core skips the full unstack and restack, saving cycles.
　中文：一个 ISR 结束且有另一个挂起时，内核跳过完整的出栈入栈，节省周期。
**如果追问到你的项目**：*In the wheel-legged design, the hard fault and watchdog-related interrupts are at priority zero, the timer that releases the control task is at five, and the IMU DMA complete interrupt is at six.*
　中文：轮腿设计里 HardFault 与看门狗相关中断优先级为 0，释放控制任务的定时器为 5，IMU DMA 完成中断为 6。
**关键词与易错点**：preemption priority、sub-priority、BASEPRI、configMAX_SYSCALL_INTERRUPT_PRIORITY。别把"数值大"当"优先级高"。

### Q19. What is a critical section and how do you implement one on Cortex-M?
**中文题意**：什么是临界区？Cortex-M 上怎么实现？
**考察点**：互斥的轻量方式。
**项目关联**：P3、P7。
**English answer（口语）**：
> A critical section is a piece of code that must not be interrupted or run concurrently with another accessing the same data. On a single-core microcontroller the simplest way is to disable interrupts for a short time, either globally with `PRIMASK` or only those at or below a threshold with `BASEPRI`, which is what FreeRTOS does. The section must be very short, because during it, latency for interrupts increases. For longer protection between tasks I use a mutex, which allows other tasks to run and supports priority inheritance.

**中文翻译**：临界区是不能被打断、也不能与访问同一数据的代码并发执行的一段代码。单核单片机上最简单的办法是短时间关中断：用 `PRIMASK` 全局关，或用 `BASEPRI` 只屏蔽某阈值及以下的中断，FreeRTOS 就是这样做的。临界区必须很短，因为期间中断延迟会增加。任务之间需要更长的保护时，用互斥量，它允许其他任务运行，并支持优先级继承。
**追问 1**：*Why use `BASEPRI` instead of `PRIMASK`?* It keeps the highest-priority interrupts, such as fault or safety interrupts, running.
　中文：这样最高优先级的中断（如故障、安全中断）仍可运行。
**追问 2**：*Can critical sections nest?* Yes, with a nesting counter or by saving and restoring the previous mask.
　中文：可以，用嵌套计数，或保存并恢复先前的屏蔽值。
**如果追问到你的项目**：*I keep critical sections to copying a small snapshot structure, and the long computation happens outside of it.*
　中文：我让临界区只用于拷贝一个小的快照结构，长时间的计算放在外面。
**关键词与易错点**：PRIMASK、BASEPRI、nesting、latency。不要在临界区里调用阻塞 API。

### Q20. What is DMA, and when do you use circular mode and half-transfer interrupts? ★
**中文题意**：DMA 是什么？循环模式和半传输中断何时用？
**考察点**：数据搬运与 CPU 负载。
**项目关联**：P3（IMU 的 SPI+DMA、串口）、P7。
**English answer（口语）**：
> DMA is a hardware engine that moves data between memory and peripherals without the CPU copying each byte. The CPU sets the source, destination and count, and gets an interrupt when the transfer completes. In circular mode the DMA restarts automatically at the end of the buffer, which is ideal for continuous streams like ADC samples or a UART receive buffer. With half-transfer and transfer-complete interrupts, I can process the first half while the DMA fills the second, which is double buffering. It reduces CPU load and gives regular timing.

**中文翻译**：DMA 是一个硬件引擎，在存储器与外设之间搬运数据，不需要 CPU 逐字节拷贝。CPU 设置源、目的和数量，传输完成时收到中断。循环模式下 DMA 在缓冲区末尾自动重新开始，适合 ADC 采样、UART 接收缓冲这类连续数据流。配合半传输和传输完成中断，我可以在 DMA 填第二半的同时处理第一半，这就是双缓冲。它降低 CPU 负载，并让时序更规律。
**追问 1**：*What are possible DMA pitfalls?* Buffer in a memory region the DMA cannot reach, wrong alignment, cache coherency on cores with a data cache, and forgetting to re-arm a one-shot transfer.
　中文：缓冲区在 DMA 到不了的区域、对齐错误、带数据 Cache 的核上的一致性问题、单次传输忘了重新启动。
**追问 2**：*How does DMA reduce latency jitter?* The transfer follows the peripheral clock, independent of how busy the CPU is.
　中文：传输跟随外设时钟，与 CPU 忙闲无关。
**如果追问到你的项目**：*The RoboMaster IMU is read through SPI with DMA, and the wheel-legged IMU data arrives with DMA into a non-cacheable buffer, so the control task does not spend cycles copying bytes.*
　中文：RoboMaster 的 IMU 通过 SPI+DMA 读取，轮腿的 IMU 数据通过 DMA 进入不可缓存缓冲，控制任务不用花周期去搬字节。
**关键词与易错点**：DMA、circular mode、half-transfer、double buffering。不要忘记 DMA 与 Cache 的一致性。

### Q21. On STM32H7, what is the cache coherency problem with DMA, and how did you handle it? ★
**中文题意**：STM32H7 上 DMA 与 Cache 的一致性问题是什么？怎么处理？
**考察点**：M7 的 Cache。
**项目关联**：P7。
**English answer（口语）**：
> The Cortex-M7 has a data cache. The CPU reads through the cache, but the DMA reads and writes RAM directly. So after a DMA receive, the CPU may read stale data from the cache, and before a DMA transmit, the newest CPU data may still sit in the cache. There are two fixes. I can put the DMA buffers in a non-cacheable region configured with the MPU, which is simple. Or I can invalidate the cache lines after receive and clean them before transmit, with the buffer aligned to the thirty-two byte cache line. In my wheel-legged design the IMU and CAN buffers are in a dedicated non-cacheable section.

**中文翻译**：Cortex-M7 有数据 Cache。CPU 通过 Cache 读取，而 DMA 直接读写 RAM。所以 DMA 接收后，CPU 可能从 Cache 读到旧数据；DMA 发送前，CPU 最新的数据可能还留在 Cache 里。有两种办法。一是把 DMA 缓冲放到用 MPU 配置为不可缓存的区域，简单。二是接收后失效 Cache 行、发送前清理 Cache 行，缓冲区按 32 字节 Cache 行对齐。我的轮腿设计里 IMU 与 CAN 缓冲放在专用的不可缓存段。
**追问 1**：*Why must the buffer be 32-byte aligned and sized?* Cache maintenance works on whole lines; neighbouring variables on the same line could be lost.
　中文：Cache 维护以整行为单位，同一行上相邻的变量可能被误操作。
**追问 2**：*Which memory is not reachable by DMA1 and DMA2 on H7?* The tightly coupled DTCM and ITCM.
　中文：紧耦合的 DTCM 和 ITCM。
**如果追问到你的项目**：*The symptom of ignoring it was control data that was occasionally one cycle old, which looks like noise; the chosen layout prevents this class of bug.*
　中文：忽略它的症状是控制数据偶尔旧一个周期，看起来像噪声；所选的布局避免这一类 bug。
**关键词与易错点**：D-cache、invalidate、clean、MPU、non-cacheable。不要说"H7 没有 DMA 问题"。

### Q22. How do you get a 168 MHz system clock from an 8 MHz crystal on STM32F4?
**中文题意**：STM32F4 怎样从 8 MHz 晶振得到 168 MHz？
**考察点**：时钟树与 PLL 计算。
**项目关联**：P3（F407，168 MHz）。
**English answer（口语）**：
> The PLL multiplies a divided input. The 8 MHz oscillator is divided by M to give a 1 to 2 MHz PLL input; with M equal to 8, that is 1 MHz. The VCO frequency is the input times N; with N equal to 336, the VCO runs at 336 MHz. The system clock is the VCO divided by P; with P equal to 2, I get 168 MHz. Peripheral buses are divided from it, for example the APB1 at 42 MHz and APB2 at 84 MHz, and I must set the flash wait states for the speed. The USB and SDIO clock comes from VCO divided by Q, and Q of 7 gives 48 MHz.

**中文翻译**：PLL 先对输入分频再倍频。8 MHz 晶振除以 M 得到 1–2 MHz 的 PLL 输入，M 取 8 就是 1 MHz。VCO 频率是输入乘 N，N 取 336 则 VCO 为 336 MHz。系统时钟是 VCO 除以 P，P 取 2 得 168 MHz。外设总线再从它分频，例如 APB1 为 42 MHz、APB2 为 84 MHz，还必须按速度设置 Flash 等待周期。USB 和 SDIO 时钟来自 VCO 除以 Q，Q 取 7 得 48 MHz。
**追问 1**：*Why must the PLL input be 1 to 2 MHz?* A low comparison frequency reduces jitter in the VCO.
　中文：较低的比较频率可降低 VCO 抖动（手册建议值，具体以数据手册为准）。
**追问 2**：*What happens if flash wait states are too low at high speed?* The core reads wrong instructions and crashes.
　中文：内核取到错误指令而崩溃。
**如果追问到你的项目**：*The RoboMaster board runs at 168 MHz with the timer clocks and the CAN bit timing derived from the APB clocks, so the clock tree directly sets the 1 Mbps CAN timing.*
　中文：RoboMaster 板跑在 168 MHz，定时器时钟和 CAN 位定时由 APB 时钟推出，所以时钟树直接决定 1 Mbps 的 CAN 时序。
**关键词与易错点**：HSE、PLL M/N/P/Q、wait states、APB prescaler。注意定时器时钟在 APB 分频不为 1 时是 APB 的两倍。

### Q23. How do you calculate PWM frequency and resolution on an STM32 timer?
**中文题意**：STM32 定时器的 PWM 频率和分辨率怎么算？
**考察点**：PSC/ARR。
**项目关联**：P4（水泵继电器的 PWM/GPIO 概念）、电机驱动原理。
**English answer（口语）**：
> The PWM frequency equals the timer clock divided by the prescaler plus one, times the auto-reload plus one. For example, with a 168 MHz timer clock, a prescaler of zero and an auto-reload of 8399, the frequency is 168 million divided by 8400, which is 20 kilohertz in edge-aligned mode. The duty resolution is the number of steps in the period, here 8400, about 13 bits. In center-aligned mode the counter counts up and down, so the period is doubled for the same reload value. Higher frequency means finer ripple but fewer duty steps and more switching loss.

**中文翻译**：PWM 频率等于定时器时钟除以（预分频 + 1）再除以（自动重装载 + 1）。例如定时器时钟 168 MHz、预分频 0、自动重装载 8399，则频率为 1.68 亿除以 8400，即边沿对齐模式下 20 kHz。占空比分辨率是一个周期内的步数，这里是 8400，约 13 位。中心对齐模式下计数器上下计数，同样的重装载值周期翻倍。频率越高纹波越细，但占空比步数越少、开关损耗越大。
**追问 1**：*Why do motor drives use center-aligned PWM?* Switching edges are symmetric, current ripple is lower, and all phases share a common sampling instant.
　中文：开关沿对称、电流纹波较低、各相有共同的采样时刻。
**追问 2**：*What is dead time?* A short interval where both switches of a half-bridge are off, to prevent shoot-through.
　中文：半桥两个开关都关断的短暂间隔，防止直通。
**如果追问到你的项目**：*In my projects the motor drivers generate the PWM internally and I send commands over CAN; I know the principle but I did not write the inner current loop.*
　中文：我的项目里 PWM 由电机驱动器内部产生，我通过 CAN 发命令；我了解原理，但没写内部电流环。
**关键词与易错点**：prescaler、auto-reload、center-aligned、dead time。

### Q24. How do you choose an ADC sampling time, and how do you reduce noise?
**中文题意**：ADC 采样时间怎么选？怎么降噪？
**考察点**：源阻抗、采样保持。
**项目关联**：P4（Hi3861 土壤湿度 ADC 标定）。
**English answer（口语）**：
> The ADC has a sample-and-hold capacitor that must charge through the source impedance within the sampling time. If the source impedance is high, as with a soil moisture sensor, I need a longer sampling time or a buffer amplifier, otherwise readings are too low and cross-talk appears between channels. To reduce noise I average multiple samples or use oversampling, add an RC filter near the input, keep analogue ground and supply clean, and take samples at a quiet moment relative to digital switching. Finally, I calibrate offset and gain against known references.

**中文翻译**：ADC 有一个采样保持电容，必须在采样时间内通过信号源阻抗充满。若源阻抗高（比如土壤湿度传感器），需要更长的采样时间或加缓冲放大器，否则读数偏低，通道之间还会串扰。降噪方法包括多次采样平均或过采样、在输入端加 RC 滤波、保持模拟地和电源干净，并选在数字开关相对安静的时刻采样。最后对偏移和增益用已知参考做标定。
**追问 1**：*How many bits do you gain from oversampling by 16?* About two extra bits, since each factor of four gives one bit, assuming uncorrelated noise.
　中文：约 2 位（每 4 倍多 1 位，前提是噪声不相关）。
**追问 2**：*Why calibrate?* Sensor offset and gain vary by unit and temperature.
　中文：传感器偏移和增益随个体和温度而变。
**如果追问到你的项目**：*In the irrigation project, the soil moisture reading is mapped to a percentage with a calibration, and the pump uses separate start and stop thresholds so noise does not make it chatter.*
　中文：灌溉项目里土壤湿度读数经标定映射成百分比，水泵用独立的启停阈值，避免噪声使其抖动。
**关键词与易错点**：source impedance、oversampling、calibration、hysteresis。

### Q25. What is a watchdog and how should you feed it correctly? ★
**中文题意**：看门狗是什么？怎样正确喂狗？
**考察点**：可靠性设计。
**项目关联**：P7（监控任务喂独立看门狗）、P1（100 ms 命令看门狗）。
**English answer（口语）**：
> A watchdog is a hardware timer that resets the system if the software does not refresh it in time. The independent watchdog runs on a separate low-speed oscillator, so it works even if the main clock fails. The key design point is when to feed it. I never feed it from a timer interrupt, because the interrupt keeps running even if the tasks are dead. Instead, a monitor task checks that every critical task has shown a heartbeat in the last period, and only then refreshes the watchdog. Before the reset, the motors should be put into a safe state.

**中文翻译**：看门狗是硬件定时器，软件没有及时刷新就复位系统。独立看门狗用单独的低速振荡器，主时钟故障时也能工作。关键的设计点是何时喂狗。我从不在定时器中断里喂狗，因为任务死了中断照样运行。而是由监控任务检查每个关键任务在上一个周期内都有心跳，才刷新看门狗。复位之前应让电机进入安全状态。
**追问 1**：*What is the difference to a window watchdog?* A window watchdog also resets if it is refreshed too early.
　中文：窗口看门狗在喂得过早时也会复位。
**追问 2**：*How do you debug with a watchdog running?* Freeze it during a halt using the debug configuration, otherwise it resets the chip at every breakpoint.
　中文：在调试配置里让它在停机时冻结，否则每个断点都会复位芯片。
**如果追问到你的项目**：*In the wheel-legged design the monitor task feeds the independent watchdog only when all tasks are healthy; for the ballbot, the driver side has a 100 millisecond command watchdog that zeroes the wheel speeds.*
　中文：轮腿设计里监控任务只在所有任务健康时才喂独立看门狗；球形机器人的驱动侧有 100 ms 命令看门狗，超时则轮速置零。
**关键词与易错点**：IWDG、heartbeat、safe state。不要在 ISR 里无条件喂狗。

### Q26. What happens between reset and `main()`? ★
**中文题意**：复位到 `main()` 之间发生了什么？
**考察点**：启动流程。
**项目关联**：P3、P7。
**English answer（口语）**：
> The hardware loads the stack pointer from the first word of the vector table and the program counter from the second word, which is the reset handler. The reset handler copies the initialised data from flash to RAM, clears the bss section, calls the system init that enables the FPU and sets the vector table offset, runs the C library initialisation including global constructors, and then calls `main`. Only then does my code configure the clocks and peripherals. If the reset handler skipped any step, globals would have wrong values.

**中文翻译**：硬件从向量表第一个字加载栈指针，从第二个字加载程序计数器，也就是复位处理函数。复位处理函数把已初始化数据从 Flash 拷到 RAM，清零 bss，调用系统初始化（使能 FPU、设置向量表偏移），运行 C 库初始化（包括全局构造函数），然后调用 `main`。之后我的代码才配置时钟和外设。复位处理函数若漏掉任一步，全局变量就会有错误的值。
**追问 1**：*Why does the reset vector have its lowest bit set?* Cortex-M runs only Thumb instructions, and the bit marks Thumb state.
　中文：Cortex-M 只运行 Thumb 指令，该位标记 Thumb 状态。
**追问 2**：*Where is the vector table when a bootloader is present?* At the application offset; the bootloader sets VTOR and the stack pointer before jumping.
　中文：在应用偏移处；Bootloader 跳转前设置 VTOR 和栈指针。
**如果追问到你的项目**：*I have debugged start-up issues such as peripheral clocks enabled in the wrong order; the sequence itself is the standard one generated by the vendor tools.*
　中文：我排查过外设时钟启用顺序不对之类的启动问题；流程本身是厂商工具生成的标准流程。
**关键词与易错点**：vector table、Reset_Handler、.data copy、.bss clear。

### Q27. How do you debug a HardFault? ★
**中文题意**：怎么排查 HardFault？
**考察点**：故障定位。
**项目关联**：P3、P7。
**English answer（口语）**：
> First I enable the specific fault handlers to get more detail. In the HardFault handler I look at the exception return value to see whether the main or process stack was in use, then read the stacked registers, especially PC and LR. I read the configurable fault status register to see the type: bus error, undefined instruction, stacking error, or a usage fault such as an unaligned access or a disabled FPU. If the address-valid bit is set, the fault address register tells me where it tried to access. Finally I map the PC to a source line with addr2line. Common causes are stack overflow, bad pointers, and using a peripheral before its clock is on.

**中文翻译**：首先使能各个具体的故障处理，以获得更多信息。在 HardFault 处理函数里，我根据异常返回值判断用的是主栈还是进程栈，然后读压栈寄存器，尤其是 PC 和 LR。读可配置故障状态寄存器判断类型：总线错误、未定义指令、压栈错误，或非对齐访问、FPU 未启用这类使用错误。若地址有效位置位，故障地址寄存器告诉我它想访问哪里。最后用 addr2line 把 PC 映射到源码行。常见原因是栈溢出、野指针、外设时钟未开就使用外设。
**追问 1**：*What is an imprecise bus fault?* The write was buffered, so the reported PC does not point at the faulting instruction; disabling the write buffer helps locate it.
　中文：写操作被缓冲，报告的 PC 不指向出错指令；关闭写缓冲有助于定位。
**追问 2**：*How would you catch a variable being corrupted?* Set a data watchpoint on its address.
　中文：对它的地址设数据观察点。
**如果追问到你的项目**：*For the RoboMaster and wheel-legged firmware, the usual suspects were stack size and buffers in the wrong memory region, so I check the stack high-water marks first.*
　中文：RoboMaster 和轮腿固件里常见的嫌疑是栈大小和缓冲放错内存区域，所以我先查栈水位。
**关键词与易错点**：CFSR、BFAR、stacked PC、addr2line。不要只会"重启试试"。

### Q28. How does a bootloader jump to the application, and how would you make an update safe?
**中文题意**：Bootloader 怎么跳到应用？怎样让升级安全？
**考察点**：启动与升级流程。
**项目关联**：延伸知识（CV 未写 OTA）。
**English answer（口语）**：
> The bootloader first checks that the application's stack pointer and reset vector are in valid ranges and that its checksum matches. Then it disables interrupts and the SysTick, de-initialises peripherals, sets the vector table offset register to the application's base, loads the application's stack pointer, and jumps to its reset vector. For a safe update I would use two application slots, download into the inactive one in checked blocks, verify a CRC and a version, boot it in a trial state, and roll back to the old slot if the new firmware does not confirm itself, with the watchdog catching hangs.

**中文翻译**：Bootloader 先检查应用的栈指针和复位向量是否在有效范围，且校验和匹配。然后关中断和 SysTick，反初始化外设，把向量表偏移寄存器设为应用基址，装载应用的栈指针，跳转到应用的复位向量。要让升级安全，我会用两个应用区，分块校验后写入非活动区，验证 CRC 和版本，以试运行状态启动，若新固件没有自我确认就回滚到旧区，用看门狗捕获卡死。
**追问 1**：*Does a CRC protect against malicious firmware?* No, only against accidental errors; a signature is needed for authenticity.
　中文：不能，只防偶发错误；要认证来源需要签名。
**追问 2**：*Why erase by sector matters?* Flash erase granularity is a sector, so partitions must be sector aligned.
　中文：Flash 擦除粒度是扇区，分区必须按扇区对齐。
**如果追问到你的项目**：*My projects flashed firmware through the debug interface; I know the OTA design from study and can explain it, but I did not implement OTA on the robots.*
　中文：我的项目通过调试口烧录固件；OTA 设计来自学习，我可以讲解，但没有在机器人上实现过 OTA。
**关键词与易错点**：VTOR、MSP、A/B slot、rollback、CRC。不要把学习内容说成项目经历。

## 4. C　RTOS 与实时性（Q29–Q42）

### Q29. Why use an RTOS instead of a super-loop, and what does "real-time" mean? ★
**中文题意**：为什么用 RTOS 而不是大循环？"实时"是什么意思？
**考察点**：实时性的定义。
**项目关联**：P3、P7（FreeRTOS）。
**English answer（口语）**：
> A super-loop runs everything in one `while(1)`, so one slow function delays everything else, and the code becomes tightly coupled. An RTOS gives each activity its own task with a priority, and a scheduler that runs the highest-priority ready task, plus safe communication tools like queues and mutexes. "Real-time" does not mean fast. It means the response time is bounded and predictable: a hard real-time system fails if a deadline is missed, like a balance controller, while a soft real-time system just degrades, like logging. The key is the worst case, not the average.

**中文翻译**：大循环把所有事情放在一个 `while(1)` 里，一个慢函数会拖慢全部，代码也强耦合。RTOS 给每项活动一个有优先级的任务，由调度器运行最高优先级的就绪任务，并提供队列、互斥量等安全的通信工具。"实时"不等于快，而是响应时间有上界、可预测：硬实时系统错过截止期就失败，比如平衡控制器；软实时只是质量下降，比如日志。关键是最坏情况而不是平均。
**追问 1**：*Does an RTOS make code faster?* No, it adds overhead; it makes timing predictable and the structure cleaner.
　中文：不会，它有额外开销；它让时序可预测、结构更清晰。
**追问 2**：*When is a super-loop enough?* A small system with few independent activities and relaxed timing.
　中文：活动少、相互独立、时序要求宽松的小系统。
**如果追问到你的项目**：*The RoboMaster robot and the wheel-legged robot both have IMU reading, control, CAN communication, remote parsing and monitoring with different rhythms, so they use FreeRTOS tasks.*
　中文：RoboMaster 和轮腿机器人都有节奏不同的 IMU 读取、控制、CAN 通信、遥控解析和监控，所以用 FreeRTOS 任务。
**关键词与易错点**：bounded response、determinism、hard vs soft。不要说"实时就是很快"。

### Q30. How does the FreeRTOS scheduler work? What are the task states?
**中文题意**：FreeRTOS 调度器怎么工作？任务有哪些状态？
**考察点**：固定优先级抢占。
**项目关联**：P3、P7。
**English answer（口语）**：
> FreeRTOS uses fixed-priority preemptive scheduling. At any time the highest-priority ready task runs; if a higher-priority task becomes ready, for example when an interrupt gives a semaphore, it preempts the current one at once. Tasks of the same priority share the CPU in time slices of one tick. The states are running, ready, blocked, which means waiting for a delay or an event, and suspended. The idle task runs at the lowest priority when nothing else is ready. The context switch itself is done in the PendSV exception.

**中文翻译**：FreeRTOS 采用固定优先级抢占式调度。任意时刻运行最高优先级的就绪任务；若更高优先级任务变为就绪（例如中断释放了信号量），会立即抢占当前任务。同优先级的任务按一个 tick 的时间片轮流运行。状态有运行、就绪、阻塞（等待延时或事件）和挂起。没有其他就绪任务时，空闲任务以最低优先级运行。上下文切换本身在 PendSV 异常里完成。
**追问 1**：*Why is PendSV set to the lowest priority?* So the switch never preempts another interrupt handler.
　中文：这样切换不会抢占其他中断处理。
**追问 2**：*What happens to a task blocked on a queue?* It is removed from the ready list until data arrives or the timeout expires.
　中文：它从就绪列表移出，直到数据到达或超时。
**如果追问到你的项目**：*The wheel-legged control task is blocked until the hardware timer notifies it, so it uses no CPU between periods.*
　中文：轮腿控制任务阻塞等待硬件定时器通知，周期之间不占 CPU。
**关键词与易错点**：preemption、time slicing、PendSV、idle task。

### Q31. Is a higher priority the same as a higher frequency? ★
**中文题意**：高优先级等于高频率吗？
**考察点**：RTOS 常见误解。
**项目关联**：P3、P7。
**English answer（口语）**：
> No. Priority decides which ready task runs first when several want the CPU. Frequency is how often a task wakes up, which comes from how it blocks, for example with a delay or by waiting for an event. A high-priority task that wakes every hundred milliseconds runs at ten hertz, and a low-priority task that wakes every millisecond runs at one kilohertz as long as the CPU has time. We give a control task a high priority because missing its deadline costs the most, not because it is the fastest. Rate-monotonic scheduling is a different rule that ties priority to period.

**中文翻译**：不是。优先级决定多个任务就绪时谁先运行；频率是任务多久醒一次，取决于它怎样阻塞（延时或等待事件）。每 100 ms 醒一次的高优先级任务就是 10 Hz，每 1 ms 醒一次的低优先级任务只要 CPU 有空就是 1 kHz。我们给控制任务高优先级，是因为错过截止期代价最大，而不是因为它最快。速率单调调度是另一条规则，把优先级与周期挂钩。
**追问 1**：*Can a low-priority task starve?* Yes, if higher-priority tasks never block.
　中文：会，如果更高优先级任务从不阻塞。
**追问 2**：*So how do you decide priorities?* By deadline tightness and criticality, then verify with measurement or response-time analysis.
　中文：按截止期紧迫度和关键程度，再用测量或响应时间分析验证。
**如果追问到你的项目**：*In the RoboMaster code I found that one task had a low priority although it produced the attitude used by the others; I treat that as a lesson: sensing tasks that feed control should not be starved.*
　中文：RoboMaster 代码里我发现一个任务优先级偏低，尽管它产生的姿态被其他任务使用；我把它当作教训：给控制供数的采样任务不该被饿着。
**关键词与易错点**：priority vs period、starvation。不要把优先级说成频率。

### Q32. What is the difference between `vTaskDelay` and `vTaskDelayUntil`? ★
**中文题意**：`vTaskDelay` 与 `vTaskDelayUntil` 的区别？
**考察点**：周期任务的漂移。
**项目关联**：P3（周期约 3 ms 的来源）、P7。
**English answer（口语）**：
> `vTaskDelay` blocks for a number of ticks counted from the moment it is called, so the period is the execution time plus the delay, and it drifts. `vTaskDelayUntil` wakes the task at fixed absolute times, using the previous wake time as the base, so the long-term period is exact if the work always finishes within the period. Both are quantised to the tick, so with a one-millisecond tick the jitter is at least of that order. For a tighter period I release the task from a hardware timer. In RoboMaster the effective loop period was about three milliseconds because of several one-tick delays inside the loop.

**中文翻译**：`vTaskDelay` 从调用时刻起阻塞若干 tick，所以周期等于执行时间加延时，会漂移。`vTaskDelayUntil` 在固定的绝对时刻唤醒任务，以上一次唤醒时刻为基准，只要工作总能在周期内完成，长期周期就是精确的。两者都按 tick 量化，1 ms tick 下抖动至少是这个量级。要更紧的周期，我用硬件定时器释放任务。RoboMaster 里由于循环内有几处一 tick 的延时，实际循环周期约 3 ms。
**追问 1**：*What if the work takes longer than the period?* The task misses wake-ups; the kernel just returns immediately, so you must detect overruns.
　中文：任务会错过唤醒点；内核直接返回，所以要检测超时。
**追问 2**：*How would you measure the real period?* Toggle a GPIO at the task start and look at it with a logic analyser, plus the cycle counter for execution time.
　中文：任务开始时翻转 GPIO，用逻辑分析仪看，再用周期计数器测执行时间。
**如果追问到你的项目**：*In the wheel-legged robot, the first version with a one-millisecond tick delay-until had about one millisecond of jitter; releasing it from a hardware timer brought it to about a tenth of a millisecond.*
　中文：轮腿机器人第一版用 1 ms tick 的 delay-until 抖动约 1 ms，改成硬件定时器释放后降到约 0.1 ms。
**关键词与易错点**：drift、absolute time、tick quantisation。不要把名义 tick 当真实周期。

### Q33. Compare queue, binary semaphore, mutex and task notification. ★
**中文题意**：比较队列、二值信号量、互斥量、任务通知。
**考察点**：同步原语选择。
**项目关联**：P3、P7。
**English answer（口语）**：
> A queue copies data between tasks or from an ISR, and can block on empty or full. A binary semaphore is a signalling flag: an ISR gives it, a task takes it; it has no owner and no priority inheritance. A mutex protects a shared resource: it has an owner, must be given back by the same task, and in FreeRTOS supports priority inheritance. A task notification is the lightest option: a direct signal or value sent to one task, faster and with less RAM than a semaphore, but only one receiver. I choose by purpose: data, signal, exclusion, or fast one-to-one wake-up.

**中文翻译**：队列在任务之间或从 ISR 向任务拷贝数据，空或满时可阻塞。二值信号量是信号标志：ISR 释放，任务获取；没有所有者，也没有优先级继承。互斥量保护共享资源：有所有者，必须由同一任务归还，FreeRTOS 里支持优先级继承。任务通知最轻量：直接给某一任务发信号或数值，比信号量更快、占 RAM 更少，但只有一个接收者。我按用途选择：数据、信号、互斥，还是快速一对一唤醒。
**追问 1**：*Why not use a binary semaphore for mutual exclusion?* It has no priority inheritance, so unbounded priority inversion is possible.
　中文：它没有优先级继承，可能出现无界优先级反转。
**追问 2**：*Can you use a mutex in an ISR?* No, mutexes are for tasks; use a queue or notification from the ISR.
　中文：不能，互斥量用于任务；ISR 里用队列或通知。
**如果追问到你的项目**：*In the wheel-legged firmware, the timer ISR wakes the control task with a notification, and the control task reads a snapshot instead of taking a mutex, so it never blocks.*
　中文：轮腿固件里定时器 ISR 用通知唤醒控制任务，控制任务读快照而不是拿互斥量，因此不会阻塞。
**关键词与易错点**：queue、semaphore、mutex、notification、priority inheritance。

### Q34. Explain priority inversion and how priority inheritance fixes it. ★
**中文题意**：解释优先级反转，优先级继承怎样解决？
**考察点**：经典 RTOS 问题。
**项目关联**：P3 文档、通用。
**English answer（口语）**：
> Suppose a low-priority task L holds a mutex, a high-priority task H then waits for it, and a medium-priority task M, which does not use the mutex, preempts L. Now H is delayed by M, which has lower priority than H; this is priority inversion, and the delay is unbounded if M runs long. With priority inheritance, when H blocks on the mutex, L temporarily runs at H's priority, so M cannot preempt it; L finishes the critical section quickly, releases the mutex, and drops back. The block time is limited to the critical section length. Mars Pathfinder is the famous case.

**中文翻译**：假设低优先级任务 L 持有互斥量，高优先级任务 H 随后等待它，而不使用该互斥量的中优先级任务 M 抢占了 L。这样 H 被优先级低于它的 M 拖延，这就是优先级反转，若 M 运行很久则延迟无界。有了优先级继承，H 阻塞于互斥量时，L 临时以 H 的优先级运行，M 无法抢占；L 很快完成临界区、释放互斥量并恢复原优先级。阻塞时间被限制在临界区长度内。火星探路者是著名案例。
**追问 1**：*Does inheritance prevent deadlock?* No; a fixed lock order or the priority ceiling protocol is needed.
　中文：不能防死锁；需要固定加锁顺序或优先级天花板协议。
**追问 2**：*Does FreeRTOS implement the ceiling protocol?* Not in the standard kernel; mutexes use inheritance.
　中文：标准内核没有；互斥量用继承。
**如果追问到你的项目**：*I avoid locks in the control path by using snapshots, and where a mutex is needed, I use the FreeRTOS mutex and keep the critical section tiny.*
　中文：我在控制路径上用快照避免锁，必须用互斥量时用 FreeRTOS 互斥量并让临界区极小。
**关键词与易错点**：unbounded inversion、inheritance、ceiling、Pathfinder。

### Q35. How can deadlock happen in an RTOS and how do you avoid it?
**中文题意**：RTOS 中死锁怎么发生？怎样避免？
**考察点**：死锁条件。
**项目关联**：通用。
**English answer（口语）**：
> Deadlock needs four conditions: mutual exclusion, hold and wait, no preemption of resources, and a circular wait. A typical case is task A holding mutex one and waiting for mutex two, while task B holds mutex two and waits for mutex one. I avoid it by breaking one condition. The easiest is a global lock order, so all tasks take locks in the same sequence. I also use timeouts on lock acquisition so a task can detect the problem, avoid holding a lock while calling unknown code, and prefer lock-free designs such as snapshots or queues for data exchange.

**中文翻译**：死锁需要四个条件：互斥、占有且等待、资源不可抢占、循环等待。典型情形是任务 A 持有互斥量 1 并等待互斥量 2，而任务 B 持有互斥量 2 并等待互斥量 1。我通过破坏其中一个条件来避免。最简单的是全局加锁顺序，所有任务按同一顺序取锁。我还给取锁加超时以便任务发现问题，避免持锁时调用未知代码，数据交换尽量用无锁设计，如快照或队列。
**追问 1**：*What is the difference between deadlock and starvation?* In starvation some task never gets the resource but others progress; in deadlock nobody progresses.
　中文：饥饿是某任务得不到资源而其他任务在前进；死锁是谁也不前进。
**追问 2**：*How do you detect deadlock at run time?* A watchdog that notices missing heartbeats, and lock timeouts that log which lock was waited on.
　中文：看门狗发现心跳缺失，以及取锁超时并记录等待的是哪把锁。
**如果追问到你的项目**：*A heartbeat-based monitor, as in the wheel-legged design, would catch a deadlocked task because its heartbeat stops.*
　中文：像轮腿设计里那样的心跳监控能发现死锁任务，因为它的心跳停了。
**关键词与易错点**：four conditions、lock order、timeout。

### Q36. How do you size a task stack and detect overflow?
**中文题意**：怎样确定任务栈大小并检测溢出？
**考察点**：栈管理。
**项目关联**：P3、P7（控制任务栈 4 KB）。
**English answer（口语）**：
> I estimate from the call depth and large locals, add the exception frame, which is 104 bytes if the FPU is used, then measure. I run the worst-case scenario with logging on and read the high-water mark, which is the minimum free stack seen. I keep at least a quarter margin. The stack size parameter in FreeRTOS is in words, not bytes. For detection, I enable the stack overflow hook, method two, which checks a fill pattern at the end of the stack on each switch; an MPU guard region gives an immediate fault.

**中文翻译**：我按调用深度和大的局部变量估算，加上异常帧（用 FPU 时为 104 字节），然后实测。在开着日志的最坏场景下运行，读水位（历史最小剩余栈）。至少留四分之一余量。FreeRTOS 栈大小参数单位是字而不是字节。检测上，我启用栈溢出钩子的方法二，每次切换检查栈末尾的填充图案；MPU 保护区则会立即触发故障。
**追问 1**：*Why does printf often cause overflow?* Formatting uses a lot of stack, floating point formatting even more.
　中文：格式化占栈多，浮点格式化更多。
**追问 2**：*Does the high-water mark guarantee safety?* It shows the worst case seen so far, not a proven bound.
　中文：它只是迄今见过的最坏情况，不是证明过的上界。
**如果追问到你的项目**：*The wheel-legged design uses a four-kilobyte control task stack with a target high-water mark under about seventy percent.*
　中文：轮腿设计里控制任务栈 4 KB，目标水位低于约 70%。
**关键词与易错点**：high-water mark、words vs bytes、overflow hook。

### Q37. How do you pass data from an interrupt to a task? ★
**中文题意**：怎样把数据从中断传给任务？
**考察点**：延迟处理模式。
**项目关联**：P3、P7（CAN 接收、IMU DMA）。
**English answer（口语）**：
> The standard pattern is deferred interrupt handling. The ISR does the minimum, then uses a `FromISR` call: for a stream of items it posts to a queue, for a plain event it gives a semaphore or sends a task notification. The task blocks on that object and does the heavy work at task level. The ISR then calls the yield-from-ISR macro so that, if a higher-priority task was woken, the switch happens immediately. For multi-word state, such as an IMU reading, the ISR can write into a buffer with a sequence counter, and the task takes a consistent snapshot.

**中文翻译**：标准模式是延迟中断处理。ISR 只做最少的事，然后调用 `FromISR` 接口：数据流用队列投递，纯事件用信号量或任务通知。任务阻塞在该对象上，在任务级做重活。ISR 随后调用 yield-from-ISR 宏，这样若唤醒了更高优先级任务，切换立即发生。多字状态（如 IMU 读数）可以由 ISR 写入带序号的缓冲，任务取一致的快照。
**追问 1**：*Why not process everything in the ISR?* It would block other interrupts and make latency unpredictable.
　中文：会阻塞其他中断，使延迟不可预测。
**追问 2**：*What if the queue is full in the ISR?* The send fails without blocking; count the drops and decide a policy, such as keeping the newest.
　中文：发送失败但不阻塞；统计丢弃次数并定策略，比如保留最新。
**如果追问到你的项目**：*CAN feedback frames from the motors are decoded in the receive interrupt, timestamped, and written into the shared state used by the control task.*
　中文：电机的 CAN 反馈帧在接收中断里解码、打时间戳，写入控制任务使用的共享状态。
**关键词与易错点**：deferred handling、FromISR、yield。

### Q38. Compare rate monotonic scheduling and earliest deadline first. ★
**中文题意**：比较 RMS 与 EDF。
**考察点**：调度理论。
**项目关联**：延伸知识。
**English answer（口语）**：
> Rate monotonic scheduling gives fixed priorities by period: the shorter the period, the higher the priority. For independent periodic tasks it is guaranteed to meet deadlines if total utilisation is below n times two to the one over n minus one, which tends to sixty-nine percent; it is sufficient, not necessary, and response-time analysis gives an exact answer. Earliest deadline first uses dynamic priorities, always running the job whose deadline is nearest, and can schedule up to one hundred percent utilisation. Fixed priority is popular because it is simple and behaves predictably in overload.

**中文翻译**：速率单调调度按周期分配固定优先级：周期越短优先级越高。对独立周期任务，只要总利用率低于 n(2^(1/n)−1)（趋近 69%）就保证满足截止期；这是充分而非必要条件，响应时间分析给出精确答案。最早截止期优先使用动态优先级，总运行截止期最近的作业，利用率可达 100%。固定优先级流行是因为简单，过载时行为可预测。
**追问 1**：*What if periods are harmonic?* The RMS bound rises to one hundred percent.
　中文：RMS 的界升到 100%。
**追问 2**：*What happens in overload under EDF?* Many jobs may miss deadlines in a cascade.
　中文：可能出现连锁式的多个作业错过截止期。
**如果追问到你的项目**：*I did not use a formal analysis on the robots; I assigned priorities by criticality and deadline tightness and measured the periods, and I know how a response-time check would be done.*
　中文：机器人上我没做正式分析；按关键程度和截止期紧迫度分配优先级并测周期，我知道响应时间校验怎么做。
**关键词与易错点**：utilisation bound、dynamic priority、overload。不要说"我用了 RMS"。

### Q39. How would you check that a control task meets its deadline? Give a numeric example.
**中文题意**：怎样校验控制任务满足截止期？给个数值例子。
**考察点**：响应时间分析。
**项目关联**：P7（任务集与预算）。
**English answer（口语）**：
> I use response-time analysis. The response time equals the task's own worst-case execution time, plus blocking, plus the interference from every higher-priority task, which is the number of its releases in the window times its execution time. I solve it by iteration until it converges. For example, take an IMU task of 0.1 ms every 1 ms, a control task of 0.38 ms every 2 ms, a monitor of 0.2 ms every 10 ms and a communication task of 0.3 ms every 20 ms. The control task's response is 0.48 ms, and even with 0.05 ms blocking it is 0.53 ms, far below 2 ms. Those execution times are an illustration.

**中文翻译**：我用响应时间分析。响应时间等于任务自身最坏执行时间，加阻塞时间，再加每个更高优先级任务的干扰（窗口内释放次数乘执行时间），通过迭代求解直到收敛。例如 IMU 任务每 1 ms 执行 0.1 ms，控制任务每 2 ms 执行 0.38 ms，监控任务每 10 ms 执行 0.2 ms，通信任务每 20 ms 执行 0.3 ms。控制任务响应时间为 0.48 ms，即使有 0.05 ms 阻塞也是 0.53 ms，远小于 2 ms。这些执行时间只是示意。
**追问 1**：*How do you get the worst-case execution time?* Measure the maximum with the cycle counter under stress, then add a margin.
　中文：在压力下用周期计数器测最大值，再加余量。
**追问 2**：*Do interrupts count?* Yes, treat them as the highest-priority tasks.
　中文：要计入，把它们当作最高优先级任务。
**如果追问到你的项目**：*The wheel-legged control task has a design budget of about 380 microseconds in a two-millisecond period, around nineteen percent, with total CPU load designed below forty percent.*
　中文：轮腿控制任务在 2 ms 周期内的设计预算约 380 µs，约 19%，总 CPU 负载设计在 40% 以下。
**关键词与易错点**：response-time analysis、fixed point iteration、blocking、WCET。

### Q40. What is jitter, and how do you reduce it in a periodic control task?
**中文题意**：什么是抖动？怎样减小周期控制任务的抖动？
**考察点**：时序质量。
**项目关联**：P7、P3。
**English answer（口语）**：
> Jitter is the variation in when a periodic task actually starts, compared with the ideal time. It matters because controllers and estimators assume a fixed sample time; jitter acts like noise and delay in the loop. To reduce it, release the task from a hardware timer rather than the system tick, give the task the highest priority, keep interrupt handlers short, avoid blocking calls and printing in the loop, and use timestamps for the actual time step in derivatives and integrals. I measure it by toggling a pin and looking at the edges on a logic analyser.

**中文翻译**：抖动是周期任务实际开始时刻相对理想时刻的变化。它之所以重要，是因为控制器和估计器假设采样时间固定；抖动等效于回路里的噪声和延迟。减小抖动的办法：用硬件定时器而不是系统 tick 释放任务，给任务最高优先级，保持中断处理很短，循环里避免阻塞调用和打印，微分积分里用时间戳得到的实际步长。我通过翻转引脚并在逻辑分析仪上看边沿来测量。
**追问 1**：*Why not just use a larger margin?* A larger period reduces control bandwidth, and jitter still varies the effective gain.
　中文：更大的周期会降低控制带宽，且抖动仍会改变有效增益。
**追问 2**：*What if dt is hard-coded?* A period change silently changes integral and derivative action; use measured dt or a fixed verified period.
　中文：周期一变，积分和微分作用悄悄改变；用实测 dt 或验证过的固定周期。
**如果追问到你的项目**：*In RoboMaster the loop was about three milliseconds because of several one-tick delays, and some code used a nominal dt of one millisecond; I treat that as a lesson to measure the real period.*
　中文：RoboMaster 里循环因几处一 tick 延时约为 3 ms，且部分代码用名义 dt 1 ms；我把它当作要测真实周期的教训。
**关键词与易错点**：jitter、hardware timer release、timestamp。

### Q41. How would you partition tasks for a balance robot's firmware? ★
**中文题意**：平衡机器人固件的任务怎么划分？
**考察点**：任务设计。
**项目关联**：P7、P3。
**English answer（口语）**：
> I separate by timing and criticality. A control task at the highest priority runs at a fixed period: take a snapshot of the state, compute kinematics and estimation, run the balance controller and leg controller, limit the outputs, and send the CAN commands. An IMU task reads the sensor over SPI with DMA and updates attitude. The CAN receive path is an interrupt that decodes and timestamps motor feedback. A communication task parses remote commands at a low rate with ramping. A monitor task checks data age, temperature, link loss and emergency stop, and feeds the watchdog. Data moves through snapshots, not long locks.

**中文翻译**：我按时序和关键程度划分。控制任务最高优先级，固定周期：取状态快照，算运动学和估计，运行平衡控制器和腿控制器，限幅，发送 CAN 命令。IMU 任务通过 SPI+DMA 读传感器并更新姿态。CAN 接收路径是中断，解码电机反馈并打时间戳。通信任务低速解析遥控指令并做斜坡。监控任务检查数据年龄、温度、链路丢失和急停，并喂看门狗。数据通过快照传递，而不是长时间持锁。
**追问 1**：*Why is the monitor lower priority than control?* It is not time-critical, but it must still run regularly, otherwise the watchdog resets the system.
　中文：它没那么紧迫，但必须定期运行，否则看门狗会复位。
**追问 2**：*Where would you put logging?* A low-priority task reading from a ring buffer written by the control task.
　中文：低优先级任务，从控制任务写入的环形缓冲读取。
**如果追问到你的项目**：*This is the structure of the wheel-legged firmware: control at 2 milliseconds, IMU at 1 kilohertz, monitor at 100 hertz, communication at 50 hertz.*
　中文：这就是轮腿固件的结构：控制 2 ms、IMU 1 kHz、监控 100 Hz、通信 50 Hz。
**关键词与易错点**：task partition、snapshot、monitor、watchdog。

### Q42. How do you share data safely between an interrupt, an IMU task and the control task?
**中文题意**：中断、IMU 任务、控制任务之间怎样安全共享数据？
**考察点**：数据竞争。
**项目关联**：P7（快照）、P3。
**English answer（口语）**：
> The risk is a torn read: the control task reads an attitude structure while the writer is halfway through updating it, giving a state that never existed. Options are a short critical section around the copy, a mutex, a double buffer with an atomic index, or a sequence lock where the writer increments a counter before and after, and the reader retries if the counter changed. For a hard real-time reader I prefer the double buffer or sequence counter, so the reader never blocks. I also attach a timestamp so the reader can detect stale data.

**中文翻译**：风险是撕裂读：控制任务读取姿态结构时写者更新到一半，得到一个从未存在过的状态。选项有：拷贝时用短临界区、互斥量、带原子索引的双缓冲，或序列锁（写者前后各把计数加一，读者发现计数变化就重试）。对硬实时的读者，我偏好双缓冲或序列计数，这样读者不会阻塞。我还附上时间戳，让读者能发现过期数据。
**追问 1**：*Why not just use `volatile`?* It does not make multi-word access atomic.
　中文：它不能让多字访问原子化。
**追问 2**：*What do you do with stale data?* Treat it as invalid: hold the last safe output or go to a safe state.
　中文：视为无效：保持上一个安全输出或进入安全状态。
**如果追问到你的项目**：*The control task takes a single snapshot each cycle and checks data age; stale data triggers the safety path, as in my debugging of attitude and joint data that were out of sync.*
　中文：控制任务每周期取一次快照并检查数据年龄；过期数据触发安全路径，这与我排查姿态和关节数据不同步的经历相符。
**关键词与易错点**：torn read、double buffer、sequence lock、timestamp。

---

## 5. D　总线与通信：CAN、UART、SPI、I2C（Q43–Q56）

### Q43. Why is CAN widely used in robots? ★
**中文题意**：为什么机器人广泛使用 CAN？
**考察点**：总线特点。
**项目关联**：P1、P2、P3、P7。
**English answer（口语）**：
> CAN is a differential two-wire multi-master bus designed for noisy environments like cars. Differential signalling gives good noise immunity, so it works near motors. It supports many nodes on one pair of wires, which saves wiring in a robot arm or legs. It has built-in arbitration, so there are no collisions to resolve in software, plus CRC, acknowledgement and error handling in hardware. Messages are broadcast with an identifier that also sets priority. Its limits are small payload, eight bytes in classic CAN, and one megabit per second, but that is enough for motor commands at hundreds of hertz.

**中文翻译**：CAN 是差分两线多主总线，为汽车这类噪声环境设计。差分信号抗噪好，因此能在电机附近工作。一对线上可以挂很多节点，在机械臂或腿上节省布线。它自带仲裁，软件不必处理冲突，还有硬件实现的 CRC、应答和错误处理。消息以广播方式发送，标识符同时决定优先级。局限是载荷小（经典 CAN 仅 8 字节）和速率 1 Mbps，但足以支持几百赫兹的电机命令。
**追问 1**：*Why is a termination resistor needed?* To match the line impedance and stop reflections; 120 ohms at each end.
　中文：匹配线路阻抗、防止反射；两端各 120 Ω。
**追问 2**：*How would you quickly check termination?* With power off, measure between CAN high and low; about 60 ohms means both terminators are present.
　中文：断电量 CAN_H 与 CAN_L，约 60 Ω 说明两端终端都在。
**如果追问到你的项目**：*All of my robots use CAN: RoboMaster motors at 1 Mbps, the wheel-legged joints and wheels on two 1 Mbps buses, and the ROS2 projects through SocketCAN.*
　中文：我所有的机器人都用 CAN：RoboMaster 电机 1 Mbps，轮腿关节和轮子各一路 1 Mbps，ROS2 项目通过 SocketCAN。
**关键词与易错点**：differential、multi-master、arbitration、termination。

### Q44. How does CAN arbitration work? ★
**中文题意**：CAN 仲裁怎么工作？
**考察点**：显性/隐性位与优先级。
**项目关联**：P3、P7。
**English answer（口语）**：
> CAN uses non-destructive bitwise arbitration. The bus acts like a wired AND: a dominant zero overrides a recessive one. When several nodes start sending at the same time, each transmits its identifier from the most significant bit and reads back the bus. If a node sends a recessive one but sees a dominant zero, it has lost, stops sending, and becomes a receiver, retrying later. The node with the numerically lowest identifier wins and its frame is not damaged. So lower ID means higher priority, and no bandwidth is wasted on collisions.

**中文翻译**：CAN 使用非破坏性的逐位仲裁。总线表现为线与：显性的 0 压过隐性的 1。多个节点同时开始发送时，每个节点从最高位开始发送标识符并回读总线。若某节点发出隐性的 1 却读到显性的 0，它就输了，停止发送转为接收，稍后重试。数值最小的标识符获胜，其帧不受破坏。所以 ID 越小优先级越高，也不会因冲突浪费带宽。
**追问 1**：*Can two nodes use the same ID?* No for data frames sent at the same time; it causes errors, so IDs must be unique per message.
　中文：同时发送数据帧时不行，会引起错误，所以每种消息的 ID 要唯一。
**追问 2**：*Does arbitration guarantee latency?* A high-priority frame waits at most for the current frame to finish, but low-priority frames can be delayed by load.
　中文：高优先级帧最多等当前帧发完，低优先级帧会因负载被推迟。
**如果追问到你的项目**：*In the RoboMaster protocol motors reply with fixed IDs, and my own command frames use lower IDs for the more critical messages.*
　中文：RoboMaster 协议里电机用固定 ID 回复，我自己的指令帧给更关键的消息分配更小的 ID。
**关键词与易错点**：dominant/recessive、wired-AND、lowest ID wins。

### Q45. Describe the classic CAN data frame and bit stuffing.
**中文题意**：描述经典 CAN 数据帧和位填充。
**考察点**：帧结构。
**项目关联**：P3、P7。
**English answer（口语）**：
> A standard data frame starts with a dominant start-of-frame bit, then an eleven-bit identifier, the remote request bit, control bits including the four-bit data length code, zero to eight data bytes, a fifteen-bit CRC, the acknowledgement slot where receivers pull the bus dominant, and the end-of-frame. Bit stuffing inserts an opposite bit after five identical bits, so receivers keep synchronisation and edges appear regularly. That adds some bits, so an eight-byte frame is about one hundred and eleven bits in the worst case estimate I use for load calculations. The extended format has a twenty-nine-bit identifier.

**中文翻译**：标准数据帧以显性的帧起始位开头，然后是 11 位标识符、远程请求位、含 4 位数据长度码的控制位、0–8 字节数据、15 位 CRC、接收者将总线拉为显性的应答间隙，以及帧结束。位填充在连续 5 个相同位后插入一个相反位，使接收者保持同步、边沿规律出现。这会增加一些位，所以 8 字节帧在我用于负载估算的保守估计里约 111 位。扩展格式有 29 位标识符。
**追问 1**：*Why 111 bits?* The 8-byte frame has about 108 bits before stuffing, and stuffing plus inter-frame space adds a few more; it is a planning figure.
　中文：8 字节帧填充前约 108 位，加上填充和帧间隔再多几位；这是规划用的数字。
**追问 2**：*What is the remote frame?* A request for data with a given ID, rarely used in robots.
　中文：请求某 ID 数据的帧，机器人里很少用。
**如果追问到你的项目**：*I use that 111-bit figure to estimate bus load in the RoboMaster and wheel-legged designs.*
　中文：我用这个 111 位的数字估算 RoboMaster 和轮腿设计的总线负载。
**关键词与易错点**：SOF、DLC、CRC、ACK、stuffing。

### Q46. How do you configure the bit timing for 1 Mbps CAN? ★
**中文题意**：1 Mbps CAN 的位定时怎么配置？
**考察点**：时间量子与采样点。
**项目关联**：P3（F407，APB1 42 MHz）。
**English answer（口语）**：
> One bit is divided into time quanta: a synchronisation segment of one, a first segment before the sample point, and a second segment after it. The bit rate equals the CAN clock divided by the prescaler and by the total quanta per bit. With a 42 MHz clock, a prescaler of three gives 14 MHz, so 14 quanta make one microsecond, which is 1 Mbps. I choose one sync quantum, eleven in the first segment and two in the second, so the sample point is at twelve of fourteen, about eighty-six percent. Both ends must use the same bit rate, and a sample point around eighty percent is typical.

**中文翻译**：一个位被划分为若干时间量子：同步段 1 个，采样点之前的第一段，以及采样点之后的第二段。位速率等于 CAN 时钟除以预分频再除以每位总量子数。42 MHz 时钟、预分频 3 得 14 MHz，14 个量子正好是 1 µs，即 1 Mbps。我选 1 个同步量子、第一段 11 个、第二段 2 个，采样点在 14 个中的第 12 个，约 86%。两端必须用相同的位速率，采样点一般在 80% 左右。
**追问 1**：*What does the synchronisation jump width do?* It limits how many quanta a node can lengthen or shorten a segment to resynchronise.
　中文：限制节点为重新同步可延长或缩短某段的量子数。
**追问 2**：*Symptom of wrong bit timing?* Error frames, receive errors, or the bus going to error passive or bus-off.
　中文：错误帧、接收错误，或总线进入错误被动或总线关闭。
**如果追问到你的项目**：*The CAN timing in the RoboMaster board follows from the APB clock, which is why the clock tree must be right before CAN works.*
　中文：RoboMaster 板的 CAN 时序由 APB 时钟决定，所以时钟树正确才能让 CAN 工作。
**关键词与易错点**：time quantum、sample point、prescaler。具体数值以芯片手册和实际配置为准。

### Q47. What happens when a CAN node has errors? Explain error counters and bus-off. ★
**中文题意**：CAN 节点出错会怎样？解释错误计数器和 Bus-Off。
**考察点**：故障限制。
**项目关联**：P3、P7。
**English answer（口语）**：
> Each node keeps a transmit error counter and a receive error counter. Detected errors, such as bit errors, stuff errors, CRC errors, form errors and missing acknowledgements, increase them quickly, and successful frames decrease them slowly. Below 128 the node is error active and sends active error flags. Above 127 it is error passive and sends passive flags. If the transmit counter passes 255, the node goes bus-off and disconnects, preventing a broken node from blocking the whole bus. It needs recovery, such as monitoring recessive bits, before it returns. A common cause is no other node acknowledging, or wrong bit rate.

**中文翻译**：每个节点有发送错误计数器和接收错误计数器。检测到位错误、填充错误、CRC 错误、格式错误和无应答等错误，计数快速增加，成功的帧使其缓慢减少。低于 128 时节点处于错误主动，发送主动错误标志；高于 127 进入错误被动，发送被动标志；发送计数器超过 255 则进入 Bus-Off 并断开，避免损坏的节点阻塞整条总线。恢复需要例如监测隐性位的过程才能回来。常见原因是没有别的节点应答，或位速率不对。
**追问 1**：*Why is a lone node in a loopback test stuck in error?* No receiver gives the acknowledgement, so it counts errors until error passive.
　中文：没有接收者应答，它持续记错误直到错误被动。
**追问 2**：*How do you recover from bus-off?* Automatic recovery or software restart after the recovery sequence, with logging.
　中文：自动恢复，或在恢复序列后软件重启，并记录日志。
**如果追问到你的项目**：*I treat a motor going offline as a timeout on its feedback frames, relax the mechanism safely, and check bus wiring and supply, since in practice those were the causes.*
　中文：我把电机离线当作其反馈帧超时，安全放松机构，并检查总线接线和供电，因为实践中原因往往在这里。
**关键词与易错点**：TEC/REC、error active/passive、bus-off。

### Q48. How do you estimate CAN bus load? Give an example. ★
**中文题意**：怎样估算 CAN 总线负载？举例。
**考察点**：带宽计算。
**项目关联**：P3（云台板约 63%）、P7。
**English answer（口语）**：
> I estimate each eight-byte frame as about one hundred and eleven bits, so at 1 Mbps a frame takes about 111 microseconds. Then I multiply by frames per second. For the RoboMaster gimbal board on one bus: five motors send feedback at 1 kilohertz, which is 5000 frames per second, and two command frames are sent at roughly 333 hertz, about 667 frames per second. Total 5667 frames, times 111 microseconds, is about 63 percent. I like to keep load under about 70 percent so that retransmissions and lower-priority frames have room.

**中文翻译**：我把每个 8 字节帧估计为约 111 位，所以 1 Mbps 下一帧约 111 µs，再乘以每秒帧数。以 RoboMaster 云台板的一条总线为例：5 个电机以 1 kHz 发送反馈，即每秒 5000 帧；两个指令帧约 333 Hz 发送，约每秒 667 帧。共 5667 帧，乘 111 µs，约 63%。我希望负载控制在约 70% 以下，为重发和低优先级帧留余量。
**追问 1**：*What if you sent the commands every control cycle at 1 kHz?* Load rises to about 78 percent in that design.
　中文：该设计里负载升到约 78%。
**追问 2**：*What does high load do?* Low-priority frames are delayed, latency grows, and with errors retransmissions can cascade.
　中文：低优先级帧被推迟，延迟增大，出错时重发可能连锁。
**如果追问到你的项目**：*For the wheel-legged robot, with eight frames per two milliseconds on the joint bus, the load is around forty-four to fifty-four percent, and the wheel bus with four frames around twenty-two to twenty-seven percent.*
　中文：轮腿机器人关节总线每 2 ms 八帧，负载约 44%–54%；轮总线四帧，约 22%–27%。
**关键词与易错点**：frame bits、utilisation、margin。

### Q49. What is the difference between classic CAN, CAN FD and what you used on the H7? ★
**中文题意**：经典 CAN、CAN FD 的区别，以及你在 H7 上用的是什么？
**考察点**：FDCAN 外设与模式。
**项目关联**：P7、P2。
**English answer（口语）**：
> CAN FD extends classic CAN in two ways: the data field can be up to sixty-four bytes, and the data phase can run at a higher bit rate than the arbitration phase, with a bit-rate switch flag. The STM32H7 has an FDCAN peripheral, which can run in classic mode or FD mode. In my wheel-legged project it is configured as classic CAN with eight-byte frames, no bit-rate switching, at one megabit per second, because the motors speak classic CAN. So the chip is capable of FD, but the motors on the bus decide the mode.

**中文翻译**：CAN FD 在两方面扩展经典 CAN：数据域最多 64 字节，数据段可用比仲裁段更高的位速率，由位速率切换标志指示。STM32H7 的 FDCAN 外设可以工作在经典模式或 FD 模式。我的轮腿项目把它配置为经典 CAN，8 字节帧，无位速率切换，1 Mbps，因为电机说的是经典 CAN。所以芯片支持 FD，但总线上的电机决定模式。
**追问 1**：*Can classic and FD nodes share a bus?* Classic nodes cannot decode FD frames and would flag errors, so the bus mode must match.
　中文：经典节点无法解码 FD 帧并会报错，所以总线模式必须匹配。
**追问 2**：*Why use FD at all?* More payload per frame and higher throughput for large data like firmware updates.
　中文：每帧载荷更大、吞吐更高，适合固件升级等大数据。
**如果追问到你的项目**：*The wheel-legged robot uses classic CAN, not CAN FD; that is a point I state clearly in interviews.*
　中文：轮腿机器人用经典 CAN，不是 CAN FD；我会在面试里明确说明。
**关键词与易错点**：classic mode、bit-rate switching、64-byte payload。红线：P7 不是 CAN FD。

### Q50. What are the practical wiring and EMC practices for a robot CAN bus?
**中文题意**：机器人 CAN 总线的接线与电磁兼容有哪些实践？
**考察点**：硬件可靠性。
**项目关联**：P3 案例。
**English answer（口语）**：
> Use twisted pair for CAN high and low, with a 120-ohm terminator at each physical end of the bus and nowhere else. Keep stubs short, ideally a few centimetres at high speed, and use a daisy-chain topology rather than a star. Share a common ground reference between nodes, route the bus away from motor phase wires, and fix connectors mechanically, because intermittent contacts cause the hardest faults. Add decoupling and a stable supply so nodes do not reset when motors draw peak current. Test with a connector wiggle test and measure the bus differential waveform.

**中文翻译**：CAN_H 与 CAN_L 使用双绞线，总线物理两端各接一个 120 Ω 终端电阻，别处不接。分支要短，高速时最好只有几厘米，拓扑用菊花链而不是星形。节点间共地参考，总线布线远离电机相线，并把连接器机械固定，因为接触不良引起的间歇故障最难查。加去耦并保证稳定供电，使电机峰值电流时节点不复位。用晃动连接器的测试和测量总线差分波形来验证。
**追问 1**：*How many terminators should a good bus have?* Two, one at each end.
　中文：两个，每端一个。
**追问 2**：*What if a node resets when motors accelerate?* Check the supply dip and ground bounce first.
　中文：先查电源跌落和地弹。
**如果追问到你的项目**：*In RoboMaster, intermittent motor dropouts were traced to wiring and supply rather than software, which is why the offline detection and the safe state matter.*
　中文：RoboMaster 里间歇性电机掉线查到是线束和供电问题而非软件，这就是离线检测和安全状态的意义。
**关键词与易错点**：twisted pair、termination、stub、ground。

### Q51. How does UART work, and what causes framing errors? How do you receive variable-length data efficiently?
**中文题意**：UART 怎么工作？什么导致帧错误？怎样高效接收变长数据？
**考察点**：异步串行。
**项目关联**：P3（USB CDC/串口协议）、P4。
**English answer（口语）**：
> UART is asynchronous: there is no clock line, so both sides agree on a baud rate. A frame is a start bit, usually eight data bits least significant first, optional parity, and a stop bit. Framing errors come from a wrong baud rate, noise, or a break. The baud rates of both sides must match within roughly two to three percent. To receive variable-length data efficiently, I use DMA in circular mode with the idle-line interrupt: when the line goes idle after a burst, the interrupt tells me how many bytes arrived, so I can parse a whole message without per-byte interrupts.

**中文翻译**：UART 是异步的，没有时钟线，双方约定波特率。一帧包括起始位、通常 8 位数据（低位在前）、可选校验位和停止位。帧错误来自波特率错误、噪声或 break。两端波特率偏差需在约 2%–3% 以内。要高效接收变长数据，我用循环模式的 DMA 加空闲线中断：一串数据后线路空闲时，中断告诉我收到了多少字节，这样可以整条解析，而不用逐字节中断。
**追问 1**：*Why is a CRC or checksum needed in the protocol?* UART has no built-in error detection beyond parity.
　中文：UART 除奇偶校验外没有内建错误检测。
**追问 2**：*How do you find the start of a message?* A header byte, a length field, and a checksum to validate; resynchronise on failure.
　中文：帧头字节、长度字段和校验来验证；失败时重新同步。
**如果追问到你的项目**：*In RoboMaster the vision link uses a header byte 0xA5, length and CRC8 and CRC16 checks over a USB virtual serial port.*
　中文：RoboMaster 的视觉链路在 USB 虚拟串口上使用帧头 0xA5、长度、CRC8 与 CRC16 校验。
**关键词与易错点**：baud rate、idle interrupt、DMA、framing。

### Q52. Explain SPI modes, and how you would read an IMU over SPI.
**中文题意**：解释 SPI 模式，以及怎样用 SPI 读 IMU。
**考察点**：CPOL/CPHA。
**项目关联**：P3（BMI088 SPI+DMA）、P7。
**English answer（口语）**：
> SPI is a synchronous full-duplex bus with a clock, two data lines and a chip select. The master generates the clock. The mode is set by clock polarity and clock phase: polarity gives the idle level of the clock, and phase decides whether data is sampled on the first or second edge. Master and slave must use the same mode, otherwise data appears shifted or wrong. To read an IMU, I pull chip select low, send the register address with the read bit set, clock out the data bytes, and release chip select. With DMA the transfer runs by itself and an interrupt tells the IMU task when it is complete.

**中文翻译**：SPI 是同步全双工总线，有时钟线、两条数据线和片选。主机产生时钟。模式由时钟极性和时钟相位决定：极性决定时钟空闲电平，相位决定在第一个还是第二个边沿采样。主从必须用同一模式，否则数据错位或出错。读 IMU 时，我拉低片选，发送带读标志位的寄存器地址，时钟移出数据字节，再释放片选。用 DMA 时传输自动进行，完成时中断通知 IMU 任务。
**追问 1**：*Why are SPI buses faster than I2C?* Push-pull drivers and dedicated lines, no addressing or acknowledgement overhead.
　中文：推挽驱动、专用线路，没有寻址和应答开销。
**追问 2**：*What if the data is all 0xFF?* Check chip select polarity, MISO wiring, device power, and mode.
　中文：检查片选极性、MISO 接线、器件供电和模式。
**如果追问到你的项目**：*The RoboMaster IMU is a BMI088 read through SPI with DMA, and the gyro is sampled at around one kilohertz.*
　中文：RoboMaster 的 IMU 是通过 SPI+DMA 读取的 BMI088，陀螺采样约 1 kHz。
**关键词与易错点**：CPOL、CPHA、chip select、full duplex。

### Q53. How does I2C work, and why does it need pull-up resistors?
**中文题意**：I2C 怎么工作？为什么需要上拉电阻？
**考察点**：开漏总线。
**项目关联**：通用（传感器）。
**English answer（口语）**：
> I2C uses two open-drain lines, SDA for data and SCL for the clock, shared by many devices addressed by a seven-bit address. Devices only pull a line low and let it float high, so the lines need pull-up resistors to return to high; this also allows multiple masters and clock stretching without driver conflict. Each byte is followed by an acknowledge bit. The pull-up value is a trade-off: too large and the rise time is too slow for the speed, too small and the low level may not be low enough. Typical values are one to ten kilohms depending on speed and bus capacitance.

**中文翻译**：I2C 用两条开漏线，SDA 传数据、SCL 传时钟，由多个器件共享，用 7 位地址寻址。器件只拉低线路，释放后线路靠上拉变高，所以需要上拉电阻；这也使多主机和时钟拉伸不会造成驱动冲突。每个字节后有一个应答位。上拉阻值是折中：太大则上升时间太慢跟不上速度，太小则低电平可能不够低。典型值 1–10 kΩ，取决于速度和总线电容。
**追问 1**：*Symptom of missing pull-ups?* No ACK and lines stuck low or floating.
　中文：没有应答，线路卡低或悬空。
**追问 2**：*What happens if a slave holds SDA low?* The bus hangs; you toggle SCL up to nine times to release it and reset the peripheral.
　中文：总线挂死；可翻转 SCL 最多九次释放，并复位外设。
**如果追问到你的项目**：*I use I2C mostly for slower sensors; the main sensing in my robot projects used SPI for the IMU.*
　中文：I2C 我主要用于较慢的传感器；机器人项目的主要传感（IMU）用 SPI。
**关键词与易错点**：open-drain、pull-up、ACK、address。

### Q54. How do you choose between UART, SPI, I2C and CAN?
**中文题意**：怎么在 UART、SPI、I2C、CAN 间选择？
**考察点**：工程选型。
**项目关联**：P3、P7、P4。
**English answer（口语）**：
> I choose by distance, speed, number of devices and robustness. SPI is for fast on-board peripherals, like an IMU, with one chip select per device. I2C is for several slow sensors using only two wires. UART is for simple point-to-point links like a debug console or a vision computer, and its robustness comes only from my protocol. CAN is for longer, noisy, multi-node links like motors and joints, with arbitration and error handling built in. For a link with a computer, USB or Ethernet may be better because of bandwidth. The answer is always a trade-off.

**中文翻译**：我按距离、速度、器件数量和鲁棒性选择。SPI 用于板上高速外设，如 IMU，每个器件一个片选。I2C 用于多个慢速传感器，只占两根线。UART 用于简单的点对点链路，如调试串口或视觉计算机，其可靠性只来自我的协议。CAN 用于较长、有噪声、多节点的链路，如电机和关节，自带仲裁和错误处理。与计算机的链路，因为带宽，USB 或以太网可能更合适。答案永远是权衡。
**追问 1**：*Which is best for a long cable near motors?* CAN, for differential signalling and error handling.
　中文：CAN，因为差分和错误处理。
**追问 2**：*Which has the highest throughput?* SPI, usually, then UART and I2C depending on speed.
　中文：通常 SPI 最高，UART 与 I2C 视速度而定。
**如果追问到你的项目**：*My STM32 designs use SPI for the IMU, CAN for motors, and a serial link to the vision computer; the Hi3861 reads an analogue soil sensor and drives a relay.*
　中文：我的 STM32 设计用 SPI 接 IMU，CAN 接电机，串口接视觉计算机；Hi3861 读模拟土壤传感器并驱动继电器。
**关键词与易错点**：distance、speed、topology、robustness。

### Q55. How do you design a robust serial or CAN protocol between two boards? ★
**中文题意**：怎样设计两块板之间稳健的串口或 CAN 协议？
**考察点**：协议设计。
**项目关联**：P3（板间 CAN）、P7、P4（TCP）。
**English answer（口语）**：
> I define a fixed frame layout: a header or ID, a message type, a length, the payload with fixed-width fields and an explicit byte order, a sequence number or timestamp, and a checksum or CRC. I add a version field to detect mismatched firmware. The receiver validates length and checksum, and resynchronises on errors. I define rates and timeouts: for example, if no frame arrives for one hundred milliseconds, treat the link as lost and go to a safe state. Both sides share one header file, so the definition has a single source of truth. I also test with fault injection, such as dropped and corrupted frames.

**中文翻译**：我定义固定的帧布局：帧头或 ID、消息类型、长度、定宽字段且字节序明确的载荷、序号或时间戳，以及校验或 CRC。再加版本字段以发现固件不匹配。接收端验证长度和校验，出错时重新同步。我规定速率和超时：例如 100 ms 收不到帧就判链路丢失并进入安全状态。双方共享同一个头文件，定义只有一个来源。我还用故障注入测试，比如丢帧和损坏帧。
**追问 1**：*Why a sequence number?* To detect lost or duplicated messages.
　中文：用于发现丢失或重复的消息。
**追问 2**：*How do you handle an old firmware on one side?* The version field lets the receiver reject or adapt rather than misinterpret.
　中文：版本字段让接收端拒绝或适配，而不是误解释。
**如果追问到你的项目**：*In RoboMaster, the two boards talk over CAN2 at 1 Mbps with a 100 hertz and a 50 hertz message, and a link is declared lost after 100 milliseconds without frames, which relaxes the gimbal and the chassis.*
　中文：RoboMaster 两块板在 CAN2 上以 1 Mbps 通信，一个 100 Hz、一个 50 Hz 的消息，100 ms 没收到帧即判失联，云台与底盘放松。
**关键词与易错点**：framing、CRC、timeout、versioning、single source of truth。

### Q56. The bxCAN peripheral has only three transmit mailboxes. What problems does that cause and how do you handle them? ★
**中文题意**：bxCAN 只有三个发送邮箱，会有什么问题？怎么处理？
**考察点**：外设限制。
**项目关联**：P3。
**English answer（口语）**：
> If the software tries to send more than three frames before any are transmitted, the extra calls fail because all mailboxes are busy, and frames are lost. This happens when I send several motor command frames back to back at a high rate. Solutions are to check for a free mailbox before sending, to put a software transmit queue with a sender task or the transmit-complete interrupt feeding the mailboxes, and to reduce the number of frames per cycle by packing several motors per frame when the protocol allows. A crude fix, which I used and now consider a weakness, is a delay between frames; it stretches the loop period.

**中文翻译**：如果软件在任何帧发出之前试图发送超过三个帧，多出的调用会因为邮箱全忙而失败，帧丢失。当我高速连续发送多个电机命令帧时就会这样。解决办法是发送前检查是否有空闲邮箱，建立软件发送队列，由发送任务或发送完成中断给邮箱喂数据，并在协议允许时一帧打包多个电机以减少每周期帧数。我用过一个粗糙办法，现在认为是弱点：帧间加延时，它会拉长循环周期。
**追问 1**：*What is the better design?* A dedicated sender with a queue, driven by the transmit-complete event, so the control loop never waits.
　中文：用专职发送者加队列，由发送完成事件驱动，控制环不等待。
**追问 2**：*How do mailboxes decide order?* By identifier priority or by request order, depending on the configuration.
　中文：按配置，按标识符优先级或请求顺序。
**如果追问到你的项目**：*In RoboMaster, one-tick delays between command frames avoided mailbox overflow but stretched the loop to about three milliseconds; I learned to measure the period and would restructure the sending.*
　中文：RoboMaster 里命令帧之间的一 tick 延时避免了邮箱溢出，但把循环拉到约 3 ms；我学到要测周期，并会重构发送方式。
**关键词与易错点**：mailbox、transmit queue、back-to-back frames。不要把延时说成最佳实践。

## 6. E　ROS2、DDS 与 Linux（Q57–Q68）

### Q57. What are the main differences between ROS1 and ROS2? ★
**中文题意**：ROS1 与 ROS2 的主要区别？
**考察点**：架构理解。
**项目关联**：P5（Humble）、P6、P2。
**English answer（口语）**：
> ROS1 relied on a central master for discovery, used its own transport, and had weak support for real-time and multi-robot systems. ROS2 is built on DDS, a data-distribution middleware, so there is no master: nodes discover each other automatically over the network. ROS2 adds quality-of-service policies, lifecycle nodes, components that can be composed in one process, better support for multiple platforms, and security options. Its client library has a common C core with language bindings. The migration cost is a different build system and APIs, but the QoS and distributed design suit multi-robot and embedded use.

**中文翻译**：ROS1 依赖中心化的 master 做发现，使用自己的传输，对实时和多机器人支持较弱。ROS2 建立在 DDS（数据分发中间件）上，没有 master：节点通过网络自动互相发现。ROS2 增加了服务质量策略、生命周期节点、可在同一进程组合的组件、更好的多平台支持和安全选项。客户端库有统一的 C 核心加各语言绑定。迁移代价是构建系统和 API 不同，但 QoS 和分布式设计适合多机器人和嵌入式场景。
**追问 1**：*What happens in ROS1 if the master dies?* New connections cannot be made, although existing links continue.
　中文：无法建立新连接，已有连接仍可继续。
**追问 2**：*Which ROS2 distribution do you use?* Humble, which is a long-term support release.
　中文：Humble，长期支持版本。
**如果追问到你的项目**：*The navigation robot used ROS2 Humble with the Nav2 stack; at the platform company I mapped ROS and ROS2 motion commands to low-level drivers.*
　中文：导航机器人用 ROS2 Humble 和 Nav2；在平台公司我把 ROS 与 ROS2 的运动指令映射到底层驱动。
**关键词与易错点**：DDS、no master、QoS、lifecycle。不要说 ROS2 是"操作系统"。

### Q58. When do you use a topic, a service, or an action? ★
**中文题意**：什么时候用话题、服务、动作？
**考察点**：通信模式。
**项目关联**：P5（Nav2 动作）、P2。
**English answer（口语）**：
> A topic is publish-subscribe for continuous streams, such as sensor data, odometry or velocity commands; there is no reply and many subscribers are possible. A service is a request-response for short, quick operations, like getting a parameter or triggering a calibration; the caller waits, so it should return fast. An action is for long-running goals that need feedback and can be cancelled, such as navigating to a pose: you send a goal, receive periodic feedback, and get a final result. Using a service for a long task blocks the client and gives no progress, so I avoid that.

**中文翻译**：话题是发布订阅，用于连续数据流，如传感器数据、里程计、速度命令；没有应答，可有多个订阅者。服务是请求应答，用于短而快的操作，如获取参数或触发标定；调用者要等待，所以应快速返回。动作用于需要反馈且可取消的长时间目标，如导航到某位姿：发送目标，收到周期性反馈，最后得到结果。用服务做长任务会阻塞客户端且看不到进度，所以我避免这样做。
**追问 1**：*Which is used by Nav2 to go to a goal?* The NavigateToPose action.
　中文：NavigateToPose 动作。
**追问 2**：*Can a service call be made from inside a callback?* Only with care, as blocking inside a callback can deadlock a single-threaded executor.
　中文：要小心，回调里阻塞可能使单线程执行器死锁。
**如果追问到你的项目**：*In the navigation project, velocity commands and odometry are topics, while sending a navigation goal and monitoring progress is an action.*
　中文：导航项目里速度命令和里程计是话题，发送导航目标并监控进度是动作。
**关键词与易错点**：pub/sub、request/response、goal/feedback/result。

### Q59. What is QoS in ROS2 and what happens when publisher and subscriber profiles do not match? ★
**中文题意**：ROS2 的 QoS 是什么？发布者和订阅者配置不匹配会怎样？
**考察点**：可靠性、持久性、历史。
**项目关联**：P5、P6。
**English answer（口语）**：
> QoS lets each endpoint state how data should be delivered. The main policies are reliability, reliable or best effort; durability, volatile or transient local, which lets late joiners receive the last message; history, keep last N or keep all, with a depth; and deadline and liveliness. For high-rate sensor streams I use best effort with a small depth, because the newest sample matters more than a retransmitted old one. For configuration or maps I use reliable and transient local. If a subscriber requests a stricter policy than the publisher offers, they do not connect, and messages silently never arrive.

**中文翻译**：QoS 让每个端点声明数据应如何投递。主要策略有：可靠性（reliable 或 best effort）；持久性（volatile 或 transient local，后者让晚加入者收到最后一条消息）；历史（保留最近 N 条或全部，带深度）；以及截止期和存活性。高频传感器流我用 best effort 加小深度，因为最新样本比重传的旧样本更重要。配置或地图用 reliable 加 transient local。若订阅者请求的策略比发布者提供的更严格，两者不会连接，消息会悄悄收不到。
**追问 1**：*Why does a map topic need transient local?* A node starting later still receives the last map.
　中文：后启动的节点仍能收到最后一张地图。
**追问 2**：*How do you debug a QoS mismatch?* `ros2 topic info -v` shows the profiles; warnings often appear in the node log.
　中文：`ros2 topic info -v` 显示各端配置；节点日志常有警告。
**如果追问到你的项目**：*A common cause of "no data" in navigation is a sensor topic published best-effort while a subscriber expects reliable, so I check QoS first.*
　中文：导航里"收不到数据"的常见原因是传感器话题以 best effort 发布而订阅者要求 reliable，所以我先查 QoS。
**关键词与易错点**：reliability、durability、history depth、compatibility。

### Q60. What is DDS, and how does discovery work? ★
**中文题意**：DDS 是什么？发现机制怎么工作？
**考察点**：分布式中间件。
**项目关联**：P6（M-DDS）、ROS2。
**English answer（口语）**：
> DDS, the Data Distribution Service, is an OMG standard for data-centric publish-subscribe middleware. Data is organised in topics inside a domain identified by a domain ID; participants in the same domain can talk to each other. The wire protocol is RTPS. Discovery is automatic: participants announce themselves, usually with multicast on the local network, then exchange information about their publishers and subscribers, and match those with compatible types and QoS. There is no broker, so latency is low and there is no single point of failure, but discovery traffic grows with the number of participants.

**中文翻译**：DDS（数据分发服务）是 OMG 的以数据为中心的发布订阅中间件标准。数据按话题组织，处于由域 ID 标识的域中；同一域的参与者可以互相通信。线上协议是 RTPS。发现是自动的：参与者通常在局域网用组播宣告自己，再交换各自发布者和订阅者的信息，并匹配类型和 QoS 兼容的端点。没有代理（broker），所以延迟低且没有单点故障，但发现流量随参与者数量增长。
**追问 1**：*How do you isolate two robots on one network?* Use different domain IDs, or restrict discovery.
　中文：使用不同的域 ID，或限制发现范围。
**追问 2**：*What if multicast is blocked, such as on Wi-Fi?* Configure unicast peers or a discovery server.
　中文：配置单播对端或发现服务器。
**如果追问到你的项目**：*On the M-Robots platform I implemented device discovery, capability registration, state synchronisation and task dispatch over M-DDS, the platform's distributed middleware.*
　中文：M-Robots 平台上我基于平台的分布式中间件 M-DDS 实现了设备发现、能力注册、状态同步和任务分发。
**关键词与易错点**：domain ID、RTPS、discovery、QoS matching。不要给 M-DDS 编造没有依据的内部细节。

### Q61. How do executors and callbacks work in rclcpp, and what mistakes should you avoid?
**中文题意**：rclcpp 的执行器与回调怎么工作？要避免哪些错误？
**考察点**：并发与阻塞。
**项目关联**：P5、P6。
**English answer（口语）**：
> The executor waits for events such as a message, a timer or a service request, and calls the matching callback. A single-threaded executor runs one callback at a time, so a callback that blocks, sleeps or does heavy computation delays every timer and subscription in the process. A multi-threaded executor can run callbacks in parallel, controlled by callback groups; a mutually exclusive group serialises its callbacks. So I keep callbacks short, move heavy work to a worker thread, avoid blocking service calls inside callbacks, and protect shared data.

**中文翻译**：执行器等待消息、定时器或服务请求等事件，并调用相应回调。单线程执行器一次只运行一个回调，所以阻塞、睡眠或做重计算的回调会延迟进程里所有定时器和订阅。多线程执行器可并行运行回调，由回调组控制；互斥回调组使其回调串行。所以我让回调保持短小，把重活放到工作线程，避免在回调里做阻塞式服务调用，并保护共享数据。
**追问 1**：*Why can a service call inside a callback deadlock?* The response needs the executor to run, but the executor thread is blocked waiting.
　中文：响应需要执行器运行，但执行器线程正阻塞等待。
**追问 2**：*What is intra-process communication?* Nodes in one process pass messages by pointer without serialising.
　中文：同进程的节点按指针传递消息，不做序列化。
**如果追问到你的项目**：*In the navigation stack, the control loop and the perception callbacks must not block each other, so heavy planning runs in its own thread or node.*
　中文：导航栈里控制环和感知回调不能互相阻塞，所以重规划在自己的线程或节点里运行。
**关键词与易错点**：executor、callback group、blocking。

### Q62. Why are timestamps and TF important, and what goes wrong without them? ★
**中文题意**：时间戳与 TF 为什么重要？没有它们会出什么问题？
**考察点**：多源数据对齐。
**项目关联**：P5、P6（统一时间戳、坐标系）。
**English answer（口语）**：
> Each sensor measurement describes the world at one moment and in one coordinate frame. To fuse them, I must transform everything to a common time and frame. TF stores transforms between frames over time so I can ask for the pose of one frame in another at a specific timestamp, with interpolation. If timestamps are wrong, for example by using arrival time instead of measurement time, a moving robot gets a smeared or shifted scan; if frames are inconsistent, the pose jumps. So I standardise timestamps, frame IDs and validity checks at the input of the state layer.

**中文翻译**：每个传感器测量描述的是某一时刻、某一坐标系中的世界。要融合它们，必须统一到公共的时间和坐标系。TF 存储随时间变化的坐标系间变换，使我能查询某个时间戳上一个坐标系在另一个坐标系中的位姿，并带插值。若时间戳不对，比如用到达时间代替测量时间，运动中的机器人会得到拖影或偏移的扫描；坐标系不一致则位姿跳变。所以我在状态层输入处统一时间戳、坐标系 ID 和有效性检查。
**追问 1**：*What is a typical transform chain in navigation?* Map to odom to base link to sensor frames.
　中文：map → odom → base_link → 各传感器坐标系。
**追问 2**：*What does "extrapolation into the future" error mean?* A transform was requested for a time newer than the latest available data.
　中文：请求的时间比最新可用数据还新。
**如果追问到你的项目**：*At the platform company I standardised timestamps, coordinate frames and validity checks across perception and actuator feedback, so the motion layer sees one consistent trusted state; in the navigation project, drift and pose jumps came from inconsistent timing, frames and motion direction.*
　中文：在平台公司我统一了感知与执行器反馈的时间戳、坐标系和有效性检查，使运动层看到一致可信的状态；导航项目里漂移和位姿跳变来自时序、坐标系和运动方向不一致。
**关键词与易错点**：TF2、timestamp、frame ID、interpolation。

### Q63. How would you find why a ROS2 node uses 100 percent CPU or has high latency?
**中文题意**：怎样查 ROS2 节点 CPU 100% 或延迟高的原因？
**考察点**：诊断流程。
**项目关联**：P5、P6。
**English answer（口语）**：
> I go layer by layer. With `top` or `htop` I see which process and thread uses CPU. With `ros2 topic hz` and `ros2 topic delay` I check publishing rates and message age. Then I look for common causes: a tight loop with no sleep or a timer period that is too short, a callback that does heavy work, large messages being copied or serialised, QoS with a large queue backlog, or logging at high rate. Then I profile with `perf` or a flame graph, reduce copies using intra-process communication, move heavy work to its own thread, and check again.

**中文翻译**：我分层排查。用 `top` 或 `htop` 看哪个进程和线程占用 CPU。用 `ros2 topic hz` 和 `ros2 topic delay` 检查发布频率和消息年龄。然后找常见原因：没有 sleep 的紧循环或定时器周期过短、做重计算的回调、大消息的拷贝或序列化、队列积压过大的 QoS、高频日志。再用 `perf` 或火焰图做性能分析，用进程内通信减少拷贝，把重活移到独立线程，然后再测。
**追问 1**：*Why do large point clouds cost so much?* Serialisation and copying; sharing pointers in one process avoids it.
　中文：序列化和拷贝；同进程共享指针可避免。
**追问 2**：*What does a growing queue indicate?* The consumer is slower than the producer.
　中文：消费者比生产者慢。
**如果追问到你的项目**：*I check timing with topic rates and message ages first, since in my navigation work most "algorithm" problems turned out to be timing or frame problems.*
　中文：我先用话题频率和消息年龄检查时序，因为在导航工作里大多数"算法"问题其实是时序或坐标系问题。
**关键词与易错点**：top、perf、topic hz、serialisation。

### Q64. Which Linux commands do you use daily when debugging a robot?
**中文题意**：调试机器人时你每天用哪些 Linux 命令？
**考察点**：Linux 熟练度。
**项目关联**：P2、P4、P5。
**English answer（口语）**：
> For processes and resources: `ps`, `top`, `htop`, `free`, `df`, `lsof`. For logs: `journalctl`, `dmesg`, and `tail -f` with `grep`. For devices: `lsusb`, `ls /dev`, `v4l2-ctl` for cameras, and `ip` plus `candump` for CAN. For network: `ip addr`, `ping`, `ss`, `tcpdump`. For files and text: `find`, `grep`, `sed`, `awk`. I also use `tmux` for persistent sessions over SSH, `scp` or `rsync` for copying, and `strace` to see system calls when something hangs. Scripts in bash or Python wrap repeated steps.

**中文翻译**：进程与资源：`ps`、`top`、`htop`、`free`、`df`、`lsof`。日志：`journalctl`、`dmesg`，以及 `tail -f` 加 `grep`。设备：`lsusb`、`ls /dev`，相机用 `v4l2-ctl`，CAN 用 `ip` 加 `candump`。网络：`ip addr`、`ping`、`ss`、`tcpdump`。文件与文本：`find`、`grep`、`sed`、`awk`。我还用 `tmux` 在 SSH 上保持会话，用 `scp` 或 `rsync` 拷贝，用 `strace` 看卡住时的系统调用。重复的步骤用 bash 或 Python 脚本封装。
**追问 1**：*What does `dmesg` show you?* Kernel messages: driver probe results, USB events, out-of-memory kills.
　中文：内核消息：驱动 probe 结果、USB 事件、OOM 终止。
**追问 2**：*How do you find which process uses a port?* `ss -ltnp` or `lsof -i`.
　中文：`ss -ltnp` 或 `lsof -i`。
**如果追问到你的项目**：*For the CAN-based robots I bring the interface up with `ip link` and watch traffic with `candump`; for the camera on the RK3588 I use V4L2 tools to check formats.*
　中文：CAN 机器人用 `ip link` 拉起接口，用 `candump` 看流量；RK3588 上的相机用 V4L2 工具检查格式。
**关键词与易错点**：dmesg、journalctl、candump、strace。

### Q65. How do you use SocketCAN on Linux? ★
**中文题意**：Linux 上怎么使用 SocketCAN？
**考察点**：CAN 与 Linux。
**项目关联**：P2（ROS2-CAN 执行链）、P1。
**English answer（口语）**：
> SocketCAN treats a CAN controller as a network interface, like `can0`. I set the bit rate and bring it up with `ip link set can0 type can bitrate 1000000` and `ip link set can0 up`. Then programs open a raw CAN socket, or use a library such as python-can, and read and write frames; `candump` and `cansend` from can-utils are the command-line tools. Because it is an ordinary kernel network interface, several processes can listen at once, and filters can select IDs. The limit is that Linux scheduling adds jitter, so safety limits and timeouts must also live on the motor or MCU side.

**中文翻译**：SocketCAN 把 CAN 控制器当作网络接口，如 `can0`。用 `ip link set can0 type can bitrate 1000000` 设置位速率，再用 `ip link set can0 up` 拉起。程序打开原始 CAN socket，或用 python-can 之类的库收发帧；can-utils 里的 `candump` 与 `cansend` 是命令行工具。因为它是普通的内核网络接口，多个进程可同时监听，并可用过滤器选 ID。局限是 Linux 调度带来抖动，所以安全限幅和超时也必须放在电机或 MCU 一侧。
**追问 1**：*How do you see error counters?* `ip -details -statistics link show can0`.
　中文：`ip -details -statistics link show can0`。
**追问 2**：*What if `can0` does not exist?* The driver is not loaded or the adapter is not recognised; check `dmesg` and the device tree or USB adapter.
　中文：驱动没加载或适配器未识别；查 `dmesg` 以及设备树或 USB 适配器。
**如果追问到你的项目**：*In the dual-arm and chassis project, the ROS2 hardware node used SocketCAN at 1 Mbit per second with two channels, and the safety timeout was enforced below the ROS layer.*
　中文：双臂与底盘项目里，ROS2 硬件节点用 SocketCAN，1 Mbit/s、两个通道，安全超时在 ROS 层之下执行。
**关键词与易错点**：can0、bitrate、candump、python-can。

### Q66. What is the difference between user space and kernel space, and have you written a driver?
**中文题意**：用户态与内核态的区别？你写过驱动吗？
**考察点**：OS 与诚实边界。
**项目关联**：P4、P6。
**English answer（口语）**：
> The kernel runs in a privileged mode with direct access to hardware and all memory; user programs run in an unprivileged mode with their own virtual address space, and they request kernel services through system calls. Drivers live in the kernel, or in a user-space framework, and expose devices as files, network interfaces or similar. To be honest, I have not written a Linux kernel driver. I have used Linux as an application platform: V4L2 for a camera, SocketCAN, and ROS2. I understand how a device tree describes hardware and how a platform driver binds to it.

**中文翻译**：内核运行在特权模式，可直接访问硬件和全部内存；用户程序运行在非特权模式，有自己的虚拟地址空间，通过系统调用请求内核服务。驱动位于内核，或在用户态框架中，把设备以文件、网络接口等形式暴露出来。老实说，我没写过 Linux 内核驱动。我把 Linux 当应用平台使用：用 V4L2 取相机画面、SocketCAN、ROS2。我理解设备树如何描述硬件，以及平台驱动如何与之绑定。
**追问 1**：*What does a system call cost?* A mode switch and some overhead, small but not free.
　中文：一次模式切换和一些开销，不大但不是零。
**追问 2**：*Why can't user code access hardware directly?* For protection: one process must not corrupt the kernel or others.
　中文：为了保护：一个进程不能破坏内核或其他进程。
**如果追问到你的项目**：*In the gardening project, the RK3588 captures frames through V4L2 buffers mapped into user space, which avoids copying.*
　中文：园艺项目里 RK3588 通过映射到用户空间的 V4L2 缓冲取帧，避免拷贝。
**关键词与易错点**：privilege levels、system call、device node。不要夸大驱动经验。

### Q67. Can Linux be used for real-time control?
**中文题意**：Linux 能做实时控制吗？
**考察点**：实时 Linux。
**项目关联**：延伸知识。
**English answer（口语）**：
> Standard Linux is not deterministic, but with preparation it can be soft or firm real-time. With the PREEMPT_RT kernel most kernel code becomes preemptible, interrupts run as threads, and locks support priority inheritance. In user space I use the SCHED_FIFO policy, pin the thread to an isolated core, lock memory to avoid page faults, and sleep until absolute times on the monotonic clock. Then I measure the worst-case latency under load with cyclictest. For hard real-time loops such as torque control at hundreds of hertz, I still prefer a microcontroller with an RTOS, and let Linux run the higher-level nodes.

**中文翻译**：标准 Linux 不是确定性的，但经过准备可以做到软实时或准硬实时。使用 PREEMPT_RT 内核时，大多数内核代码可抢占，中断以线程方式运行，锁支持优先级继承。用户态我使用 SCHED_FIFO 策略，把线程固定到隔离的核，锁定内存避免页错误，并在单调时钟上睡眠到绝对时刻。然后用 cyclictest 在负载下测最坏延迟。对于几百赫兹的力矩控制这类硬实时环路，我仍倾向用带 RTOS 的单片机，让 Linux 跑高层节点。
**追问 1**：*What is the first number you look at in cyclictest?* The maximum latency, not the average.
　中文：最大延迟，而不是平均值。
**追问 2**：*What protects the system from a runaway real-time thread?* Real-time throttling, by default ninety-five percent of CPU time.
　中文：实时限流，默认 95% CPU 时间。
**如果追问到你的项目**：*In my projects Linux hosts ROS2 nodes at tens of hertz and the real-time inner loops are on the STM32; I have not configured PREEMPT_RT myself.*
　中文：我的项目里 Linux 以几十赫兹运行 ROS2 节点，实时内环在 STM32 上；我没有亲自配置过 PREEMPT_RT。
**关键词与易错点**：PREEMPT_RT、SCHED_FIFO、mlockall、cyclictest。

### Q68. How do you build and deploy C++ code for a different target, such as an ARM board? ★
**中文题意**：怎样为不同目标（例如 ARM 板）构建和部署 C++ 代码？
**考察点**：交叉编译与构建。
**项目关联**：P4（RK3588）、P6（OpenHarmony）。
**English answer（口语）**：
> I use cross-compilation: a toolchain on the host compiles for the target architecture, for example an aarch64 compiler with a sysroot containing the target's libraries and headers. A build system such as CMake takes a toolchain file that sets the compiler and sysroot, so the same project builds for both host and target. I deploy the binary and its libraries by `scp` or a package, and check dynamic dependencies with `ldd` on the target. Common problems are ABI or library version mismatch, missing runtime libraries, and wrong float or architecture flags. For ROS2, colcon with a toolchain file does the same.

**中文翻译**：我用交叉编译：主机上的工具链为目标架构编译，例如带 sysroot（含目标机库和头文件）的 aarch64 编译器。CMake 这类构建系统通过工具链文件设置编译器和 sysroot，同一项目可同时构建主机和目标版本。通过 `scp` 或安装包部署二进制及其库，在目标上用 `ldd` 检查动态依赖。常见问题是 ABI 或库版本不匹配、缺少运行时库、浮点或架构标志不对。ROS2 用 colcon 加工具链文件也是同样做法。
**追问 1**：*What is a sysroot?* A directory with the target's headers and libraries used at link time.
　中文：含目标机头文件和库、链接时使用的目录。
**追问 2**：*How do you avoid "works on my machine"?* Fixed toolchain versions, containers, and a clean CI build.
　中文：固定工具链版本、使用容器、干净的 CI 构建。
**如果追问到你的项目**：*For the RK3588 and OpenHarmony work, I built C++ components for the board and used the vendor build tools; the main lesson was to match the library versions with the image on the board.*
　中文：RK3588 与 OpenHarmony 工作中，我为板子构建 C++ 组件并使用厂商构建工具；主要教训是库版本要与板上镜像匹配。
**关键词与易错点**：cross-compilation、toolchain file、sysroot、ldd。

---

## 7. F　操作系统（Q69–Q78）

### Q69. What is the difference between a process and a thread? ★
**中文题意**：进程与线程的区别？
**考察点**：OS 基础（你修过 Operating Systems）。
**项目关联**：ROS2 节点、FreeRTOS 任务。
**English answer（口语）**：
> A process is an instance of a running program with its own virtual address space, file descriptors and resources, isolated from other processes. A thread is a unit of execution inside a process; threads of the same process share the address space, heap and open files, but each has its own stack and registers. Creating and switching threads is cheaper than processes, and sharing data is easy, but that also means races and a crash in one thread can bring down the whole process. Processes give isolation, at the cost of heavier communication through IPC.

**中文翻译**：进程是运行中的程序实例，有自己的虚拟地址空间、文件描述符和资源，与其他进程隔离。线程是进程内的执行单元；同一进程的线程共享地址空间、堆和打开的文件，但各自有栈和寄存器。创建和切换线程比进程便宜，共享数据容易，但这也意味着会有竞争，且一个线程崩溃可能拖垮整个进程。进程提供隔离，代价是需要通过 IPC 做更重的通信。
**追问 1**：*Is a FreeRTOS task a process or a thread?* More like a thread: no memory isolation unless an MPU port is used.
　中文：更像线程：除非用 MPU 移植，否则没有内存隔离。
**追问 2**：*Why separate robot functions into processes?* Fault isolation and independent restart, at the cost of communication overhead.
　中文：故障隔离和独立重启，代价是通信开销。
**如果追问到你的项目**：*ROS2 nodes can run as separate processes or be composed into one process; I choose by fault isolation versus the cost of copying large messages.*
　中文：ROS2 节点可作为独立进程，也可组合进一个进程；我在故障隔离和大消息拷贝成本之间取舍。
**关键词与易错点**：address space、stack、isolation、IPC。

### Q70. What happens during a context switch, and what does it cost?
**中文题意**：上下文切换时发生什么？代价是什么？
**考察点**：调度机制。
**项目关联**：FreeRTOS PendSV。
**English answer（口语）**：
> The OS saves the state of the running thread, including registers and the stack pointer, into its control block, picks the next thread, and restores its state. For a different process it also switches the address space, which may flush the TLB. The direct cost is the save and restore, which on a Cortex-M running FreeRTOS is on the order of microseconds or less, done in the PendSV handler. The indirect cost is cache and TLB misses after the switch, which can be larger on a big CPU. That is why too many context switches hurt throughput.

**中文翻译**：操作系统把运行线程的状态（包括寄存器和栈指针）保存到其控制块，挑选下一个线程，并恢复它的状态。若切换到不同进程，还要切换地址空间，可能冲刷 TLB。直接代价是保存和恢复，在运行 FreeRTOS 的 Cortex-M 上约为微秒或更少，在 PendSV 处理函数里完成。间接代价是切换后的缓存与 TLB 未命中，在大 CPU 上可能更大。所以过多的上下文切换会损害吞吐。
**追问 1**：*Why is a thread switch cheaper than a process switch?* Same address space, so no page-table switch.
　中文：同一地址空间，无需切换页表。
**追问 2**：*Which registers are saved by hardware on Cortex-M?* Eight core registers at exception entry; the OS saves the rest.
　中文：异常入口硬件压 8 个核心寄存器，其余由 OS 保存。
**如果追问到你的项目**：*On the STM32, I keep the number of tasks small and rely on notifications, so the scheduling overhead stays a small fraction of the period.*
　中文：STM32 上我保持任务数少并依赖通知，使调度开销只占周期的一小部分。
**关键词与易错点**：PCB/TCB、TLB flush、cache miss。

### Q71. Explain virtual memory, paging and page faults. ★
**中文题意**：解释虚拟内存、分页和缺页。
**考察点**：内存管理。
**项目关联**：Linux 实时（mlockall）。
**English answer（口语）**：
> Virtual memory gives each process its own address space, mapped to physical memory by page tables in fixed-size pages, commonly four kilobytes. The MMU translates addresses and checks permissions, so processes are isolated and can use more memory than is physically present. If a page is not mapped or not in RAM, a page fault occurs; the kernel then loads the page from disk, allocates a new zero page, or kills the process for an invalid access. Page faults can take milliseconds if disk is involved, which is why real-time programs lock memory and pre-touch their buffers.

**中文翻译**：虚拟内存给每个进程独立的地址空间，用页表以固定大小的页（通常 4 KB）映射到物理内存。MMU 翻译地址并检查权限，因此进程相互隔离，且能使用超过物理容量的内存。若页未映射或不在 RAM 里，就发生缺页；内核会从磁盘加载该页、分配新的零页，或因非法访问杀死进程。涉及磁盘的缺页可能耗时毫秒级，所以实时程序会锁定内存并预先触碰缓冲区。
**追问 1**：*What is the TLB?* A small cache of recent address translations; a miss requires walking the page table.
　中文：最近地址翻译的小缓存；未命中需要遍历页表。
**追问 2**：*What is thrashing?* Constant paging because the working set exceeds RAM, so the system spends its time swapping.
　中文：工作集超过 RAM，持续换页，系统时间都花在交换上。
**如果追问到你的项目**：*Microcontrollers like the STM32 have no MMU, so addresses are physical; on the RK3588 virtual memory applies, which matters when mapping camera buffers.*
　中文：STM32 这类单片机没有 MMU，地址是物理地址；RK3588 上有虚拟内存，映射相机缓冲时要注意。
**关键词与易错点**：page table、MMU、TLB、page fault、swap。

### Q72. Describe common CPU scheduling algorithms and how Linux schedules normal tasks.
**中文题意**：常见 CPU 调度算法，以及 Linux 如何调度普通任务？
**考察点**：调度。
**项目关联**：OS 课程。
**English answer（口语）**：
> Classic algorithms are first-come-first-served, shortest-job-first, round-robin with a time quantum, and priority scheduling. Shortest-job-first minimises average waiting time but needs the run length and can starve long jobs. Round-robin gives fairness and good response. Multi-level feedback queues adapt priorities to behaviour. Linux's default scheduler for normal tasks, the completely fair scheduler, tracks virtual runtime and always picks the task with the least, using a red-black tree, with weights from nice values. Real-time policies like SCHED_FIFO and SCHED_RR sit above it and use fixed priorities.

**中文翻译**：经典算法有先来先服务、最短作业优先、带时间片的轮转和优先级调度。最短作业优先使平均等待时间最小，但需要知道运行时长，且可能饿死长作业。轮转带来公平和良好响应。多级反馈队列根据行为调整优先级。Linux 对普通任务的默认调度器（完全公平调度器）跟踪虚拟运行时间，用红黑树总是选取最小者，权重来自 nice 值。SCHED_FIFO 与 SCHED_RR 这类实时策略在其之上，使用固定优先级。
**追问 1**：*What is starvation and how is it prevented?* A task never runs; aging raises the priority of waiting tasks.
　中文：任务一直得不到运行；老化机制提高等待任务的优先级。
**追问 2**：*What is the difference between SCHED_FIFO and SCHED_RR?* RR has time slices between equal priorities; FIFO runs until it blocks or yields.
　中文：RR 在同优先级间有时间片；FIFO 运行到阻塞或让出。
**如果追问到你的项目**：*FreeRTOS uses fixed-priority preemptive scheduling with round-robin among equal priorities, which fits control tasks with known deadlines.*
　中文：FreeRTOS 用固定优先级抢占，同优先级轮转，适合有已知截止期的控制任务。
**关键词与易错点**：FCFS、SJF、RR、MLFQ、CFS、nice。

### Q73. Compare mutex, semaphore, spinlock and condition variable. ★
**中文题意**：比较互斥锁、信号量、自旋锁、条件变量。
**考察点**：同步原语。
**项目关联**：并发。
**English answer（口语）**：
> A mutex gives exclusive access to a resource and has an owner; a waiting thread sleeps. A counting semaphore allows up to N holders and can be signalled by another thread, so it models resource counts or events. A spinlock makes a waiting thread busy-loop until the lock is free; it is only suitable for very short critical sections on multi-core systems, never for long ones or on a single core without care. A condition variable lets a thread sleep until a condition on shared data becomes true, always used with a mutex and a loop that rechecks the condition because of spurious wakeups.

**中文翻译**：互斥锁提供对资源的独占访问，有所有者，等待的线程睡眠。计数信号量允许最多 N 个持有者，并可由另一线程发信号，所以可表示资源数量或事件。自旋锁让等待线程忙等直到锁空闲；只适合多核系统上非常短的临界区，长临界区或单核上要格外小心。条件变量让线程睡眠直到共享数据上的条件成立，总是与互斥锁和一个重新检查条件的循环配合使用，因为存在虚假唤醒。
**追问 1**：*Why recheck the condition in a loop?* Spurious wakeups, and another thread may have consumed the condition.
　中文：虚假唤醒，并且别的线程可能已消费了该条件。
**追问 2**：*Can a semaphore be used as a mutex?* It lacks ownership and priority inheritance, so it is a poor mutex.
　中文：它没有所有权和优先级继承，作互斥量并不合适。
**如果追问到你的项目**：*On the STM32 I use notifications and snapshots; on Linux ROS2 nodes I use mutexes and condition variables for shared state between callbacks and worker threads.*
　中文：STM32 上我用通知和快照；Linux ROS2 节点里用互斥锁和条件变量保护回调与工作线程共享的状态。
**关键词与易错点**：ownership、busy-wait、spurious wakeup。

### Q74. What are the four conditions for deadlock, and how does the banker's algorithm relate?
**中文题意**：死锁的四个条件？银行家算法与之有什么关系？
**考察点**：OS 理论。
**项目关联**：OS 课程。
**English answer（口语）**：
> Deadlock needs mutual exclusion, hold and wait, no preemption, and circular wait. Prevention breaks one of them, for example a global lock ordering removes circular wait. Avoidance, like the banker's algorithm, grants a request only if the system stays in a safe state, meaning there is some order in which all processes can finish; it needs to know the maximum demands in advance. Detection finds cycles in the resource-allocation graph and recovers by killing or rolling back. In practice most systems prevent by design, using lock ordering and timeouts, because avoidance needs information that is rarely available.

**中文翻译**：死锁需要互斥、占有且等待、不可抢占和循环等待。预防是破坏其中一个，比如全局加锁顺序消除循环等待。避免（如银行家算法）只在系统仍处于安全状态时才批准请求，所谓安全状态是存在一种顺序让所有进程都能完成；它需要事先知道最大需求。检测是在资源分配图中找环，并通过杀死进程或回滚恢复。实践中多数系统靠设计预防，用加锁顺序和超时，因为避免所需的信息很少可得。
**追问 1**：*What is a safe state?* A state in which there is an order of completion for all processes with the available resources.
　中文：用现有资源存在一种使所有进程完成的顺序。
**追问 2**：*Is an unsafe state always a deadlock?* No, but it may lead to one.
　中文：不一定，但可能导致死锁。
**如果追问到你的项目**：*In firmware I avoid the whole problem by keeping the control path lock-free, with snapshots and notifications.*
　中文：固件里我让控制路径无锁（快照和通知），从根本上避免这个问题。
**关键词与易错点**：safe state、lock ordering、resource graph。

### Q75. What IPC mechanisms exist, and how do you choose?
**中文题意**：有哪些进程间通信机制？怎么选？
**考察点**：IPC。
**项目关联**：P6（IPC）、ROS2。
**English answer（口语）**：
> Common IPC mechanisms are pipes and FIFOs for byte streams, shared memory for the fastest data sharing but with synchronisation needed, message queues, Unix domain sockets and network sockets, signals for simple notifications, and files. I choose by data size, rate and coupling. Shared memory with a lock or lock-free ring buffer is best for large high-rate data, such as images. Sockets are flexible and also work across machines. ROS2 and DDS are higher-level publish-subscribe layers built on such transports. Binder-like RPC mechanisms provide structured calls between services on mobile and embedded operating systems.

**中文翻译**：常见 IPC 机制有：用于字节流的管道和 FIFO，最快的数据共享方式但需要同步的共享内存，消息队列，Unix 域套接字和网络套接字，用于简单通知的信号，以及文件。我按数据量、速率和耦合度选择。大数据高速率（如图像）用带锁或无锁环形缓冲的共享内存最合适；套接字灵活，也可跨机器。ROS2 与 DDS 是建立在这类传输之上的更高层发布订阅。类似 Binder 的 RPC 机制在移动和嵌入式操作系统里提供服务间的结构化调用。
**追问 1**：*Why is shared memory fast?* No copy through the kernel; both processes map the same pages.
　中文：不经内核拷贝；两个进程映射同一批页。
**追问 2**：*What does shared memory require?* Synchronisation, and care with data layout and lifetime.
　中文：同步，以及对数据布局和生命周期的注意。
**如果追问到你的项目**：*On the OpenHarmony platform, I got to know its inter-process communication, and for communication across devices I worked with M-DDS.*
　中文：在 OpenHarmony 平台上我了解它的进程间通信，跨设备通信则接触 M-DDS。
**关键词与易错点**：pipe、shared memory、socket、RPC。

### Q76. What is a system call and what happens when you call `read()`?
**中文题意**：什么是系统调用？调用 `read()` 时发生了什么？
**考察点**：用户/内核边界。
**项目关联**：V4L2、串口。
**English answer（口语）**：
> A system call is the controlled entry from user mode into the kernel to request a service. When I call `read()`, the C library puts the call number and arguments in registers and executes a trap instruction. The CPU switches to kernel mode and jumps to the system call handler, which checks the file descriptor and arguments, calls the file system or driver, possibly blocks the process until data is ready, copies data to the user buffer, and returns. Back in user mode, `read` returns the byte count or an error. Each crossing has a cost, so I batch operations.

**中文翻译**：系统调用是从用户态进入内核请求服务的受控入口。调用 `read()` 时，C 库把调用号和参数放进寄存器并执行陷入指令。CPU 切换到内核态并跳到系统调用处理函数，它检查文件描述符和参数，调用文件系统或驱动，可能阻塞进程直到数据就绪，把数据拷贝到用户缓冲区，然后返回。回到用户态后，`read` 返回字节数或错误。每次跨越有成本，所以我会批量操作。
**追问 1**：*What does a blocking read do to the process?* It is put to sleep and woken when data is available.
　中文：进程睡眠，数据可用时被唤醒。
**追问 2**：*How can you wait on many descriptors?* `select`, `poll` or `epoll`.
　中文：`select`、`poll` 或 `epoll`。
**如果追问到你的项目**：*Reading camera frames or CAN frames in a loop uses `poll` so the thread sleeps until data arrives instead of spinning.*
　中文：循环读取相机帧或 CAN 帧时用 `poll`，线程睡眠到数据到来，而不是空转。
**关键词与易错点**：trap、kernel mode、blocking I/O、copy to user。

### Q77. Compare blocking I/O, non-blocking I/O, I/O multiplexing and asynchronous I/O. ★
**中文题意**：比较阻塞 I/O、非阻塞 I/O、I/O 多路复用和异步 I/O。
**考察点**：I/O 模型。
**项目关联**：P4（asyncio 服务端）。
**English answer（口语）**：
> With blocking I/O the call sleeps until data is ready. Non-blocking I/O returns immediately with "would block", so I must retry, which wastes CPU if done naively. I/O multiplexing, with `select`, `poll` or `epoll`, lets one thread wait on many descriptors and handle those that are ready; `epoll` scales well to many connections. True asynchronous I/O lets the kernel perform the operation and notify on completion, as with `io_uring`. Python's `asyncio` uses an event loop on top of multiplexing, so a single thread handles many connections as long as the code does not block.

**中文翻译**：阻塞 I/O 时调用睡眠到数据就绪。非阻塞 I/O 立即返回"会阻塞"，所以我必须重试，天真地做会浪费 CPU。I/O 多路复用（`select`、`poll`、`epoll`）让一个线程同时等待多个描述符并处理就绪的；`epoll` 能很好地扩展到大量连接。真正的异步 I/O 由内核执行操作并在完成时通知，例如 `io_uring`。Python 的 `asyncio` 在多路复用之上提供事件循环，只要代码不阻塞，单线程就能处理很多连接。
**追问 1**：*Why is `epoll` better than `select` for many sockets?* It does not scan all descriptors each call and has no small fixed limit.
　中文：不用每次扫描所有描述符，也没有较小的固定上限。
**追问 2**：*What breaks an asyncio server?* A blocking call inside a coroutine stalls the whole loop.
　中文：协程里的阻塞调用会卡住整个事件循环。
**如果追问到你的项目**：*The gardening project's server uses asyncio with WebSocket and TCP, so slow device I/O must be awaited instead of blocking the loop.*
　中文：园艺项目的服务端用 asyncio 配合 WebSocket 和 TCP，所以慢速设备 I/O 必须 await，不能阻塞事件循环。
**关键词与易错点**：epoll、event loop、non-blocking、io_uring。

### Q78. What are zombie and orphan processes, and what is the OOM killer?
**中文题意**：僵尸进程与孤儿进程是什么？OOM killer 是什么？
**考察点**：Linux 进程管理。
**项目关联**：Linux 运维。
**English answer（口语）**：
> A zombie is a process that has exited but whose parent has not yet read its exit status with `wait`, so its table entry remains. It uses almost no memory but occupies a process slot; the fix is for the parent to call `wait` or handle `SIGCHLD`. An orphan is a running process whose parent has exited; it is adopted by init, which will reap it. The OOM killer is a kernel mechanism that, when memory is exhausted, picks a process by a badness score and kills it. On embedded boards, a sudden missing process with a line in `dmesg` often means it was OOM-killed.

**中文翻译**：僵尸进程是已退出但父进程还没用 `wait` 读取其退出状态的进程，所以进程表项保留。它几乎不占内存但占用进程槽；解决办法是父进程调用 `wait` 或处理 `SIGCHLD`。孤儿进程是父进程已退出但仍在运行的进程；它会被 init 收养并回收。OOM killer 是内核机制，内存耗尽时按"糟糕程度"评分挑选进程并杀死。在嵌入式板子上，进程突然消失且 `dmesg` 有记录，往往是被 OOM 杀掉了。
**追问 1**：*How do you find zombies?* `ps` with state Z.
　中文：`ps` 中状态为 Z。
**追问 2**：*How do you prevent OOM kills of a critical process?* Lower its OOM score, limit other processes, and add memory monitoring.
　中文：降低其 OOM 评分、限制其他进程，并加内存监控。
**如果追问到你的项目**：*On the RK3588, memory use of the vision pipeline and the agent service must be watched together, because model buffers are large.*
　中文：RK3588 上视觉流水线和 Agent 服务的内存要一起监控，因为模型缓冲很大。
**关键词与易错点**：wait、SIGCHLD、init、oom_score。

---

## 8. G　数据结构与算法（Q79–Q88）

### Q79. What is time complexity, and how do you analyse an algorithm? ★
**中文题意**：什么是时间复杂度？怎样分析一个算法？
**考察点**：Big-O。
**项目关联**：《数据结构与算法》课程。
**English answer（口语）**：
> Time complexity describes how the running time grows with the input size n, ignoring constants, written in big-O notation for the upper bound. I count the dominant operations: a single loop over n items is O of n; nested loops are O of n squared; halving the problem each step, like binary search, is O of log n; sorting by comparison is O of n log n. I distinguish worst, average and amortised cases: for example, appending to a dynamic array is amortised O of one although a resize costs O of n. I also consider space complexity and real constants, since for small n a simple algorithm may be faster.

**中文翻译**：时间复杂度描述运行时间随输入规模 n 的增长，忽略常数，用大 O 表示上界。我统计主导操作：对 n 个元素的单层循环是 O(n)；嵌套循环是 O(n²)；每步把问题减半（如二分查找）是 O(log n)；基于比较的排序是 O(n log n)。我区分最坏、平均和摊还情形：例如向动态数组追加元素摊还为 O(1)，尽管一次扩容代价为 O(n)。我也考虑空间复杂度和实际常数，因为 n 小时简单算法可能更快。
**追问 1**：*Why is dynamic array append amortised constant?* Doubling the capacity means the total cost of n appends is about 2n.
　中文：容量翻倍，n 次追加的总代价约 2n。
**追问 2**：*Big-O versus big-Theta?* O is an upper bound, Theta is a tight bound.
　中文：O 是上界，Θ 是紧界。
**如果追问到你的项目**：*In the navigation project, planning cost grows with the map resolution and the search space, which is why the costmap resolution and heuristic matter.*
　中文：导航项目里规划代价随地图分辨率和搜索空间增长，所以代价地图分辨率和启发式很重要。
**关键词与易错点**：Big-O、amortised、worst case。

### Q80. Compare arrays and linked lists, and when to use each.
**中文题意**：比较数组与链表，各自何时使用。
**考察点**：数据结构选择。
**项目关联**：嵌入式缓冲。
**English answer（口语）**：
> An array stores elements contiguously, so indexing is O of one, it is cache friendly, and it has no per-element overhead; inserting in the middle costs O of n. A linked list gives O of one insertion or deletion at a known node, but access by index is O of n, each node has pointer overhead, and cache behaviour is poor. In practice arrays or vectors win for most workloads, even where theory favours lists. In embedded code I prefer fixed-size arrays or ring buffers; linked lists appear in kernels, for example the ready lists in an RTOS.

**中文翻译**：数组连续存储元素，所以索引为 O(1)，对缓存友好，每个元素没有额外开销；在中间插入代价为 O(n)。链表在已知节点处插入或删除为 O(1)，但按下标访问是 O(n)，每个节点有指针开销，缓存表现差。实践中数组或 vector 在大多数负载下更优，即使理论上链表占优。嵌入式代码里我偏好定长数组或环形缓冲；链表出现在内核里，例如 RTOS 的就绪列表。
**追问 1**：*How do you reverse a singly linked list?* Walk through with three pointers, previous, current and next, reversing each link in O of n time and O of one space.
　中文：用 prev、cur、next 三个指针遍历，逐个反转链接，时间 O(n)、空间 O(1)。
**追问 2**：*How do you detect a cycle?* Floyd's tortoise and hare: two pointers moving at different speeds meet if there is a cycle.
　中文：Floyd 龟兔算法：快慢指针若相遇则有环。
**如果追问到你的项目**：*Sensor buffers on the microcontroller are fixed arrays, so memory is known at compile time.*
　中文：单片机上的传感器缓冲是定长数组，内存在编译期已知。
**关键词与易错点**：contiguous、cache locality、O(1) insert。

### Q81. How does a hash table work, and what are collisions? ★
**中文题意**：哈希表怎么工作？什么是冲突？
**考察点**：哈希。
**项目关联**：Python dict、状态存储。
**English answer（口语）**：
> A hash table maps a key to an array index with a hash function, giving average O of one lookup, insert and delete. Different keys can hash to the same index, a collision. Chaining stores a list of entries in each bucket; open addressing probes for another free slot. The load factor, entries divided by buckets, controls performance: when it gets high, the table resizes and rehashes. Worst-case lookup is O of n if many keys collide, which is why a good hash function matters. Keys must be hashable and immutable, and hash tables do not keep order.

**中文翻译**：哈希表用哈希函数把键映射到数组下标，平均查找、插入、删除为 O(1)。不同键可能映射到同一下标，即冲突。链地址法在每个桶里存一个条目链表；开放寻址则探测另一个空槽。负载因子（条目数除以桶数）控制性能：过高时表会扩容并重新哈希。若大量键冲突，最坏查找为 O(n)，所以好的哈希函数很重要。键必须可哈希且不可变，哈希表不保持顺序。
**追问 1**：*What is a good load factor?* Roughly under 0.7 for open addressing; chaining tolerates around one.
　中文：开放寻址大致低于 0.7；链地址法可容忍约 1。
**追问 2**：*Compare with a balanced tree.* A tree gives O of log n and ordered iteration; a hash table gives faster average lookup but no order.
　中文：树提供 O(log n) 和有序遍历；哈希表平均查找更快但无序。
**如果追问到你的项目**：*In the gardening server, device states are kept in a keyed store, which behaves like a hash map, so lookup by device ID is constant time.*
　中文：园艺服务端用键值存储保存设备状态，行为类似哈希表，按设备 ID 查找是常数时间。
**关键词与易错点**：hash function、collision、load factor、rehash。

### Q82. What are stacks, queues and ring buffers, and where do you use them in firmware? ★
**中文题意**：栈、队列、环形缓冲是什么？固件里用在哪？
**考察点**：基本结构。
**项目关联**：P3、P7（UART/CAN 缓冲、日志）。
**English answer（口语）**：
> A stack is last in, first out: function calls, undo, and depth-first search use it. A queue is first in, first out: tasks, messages, and breadth-first search. A ring buffer is a fixed-size queue in an array with a read and a write index that wrap around; it needs no allocation and has constant-time operations. A single-producer, single-consumer ring buffer can be lock-free if each index is written by only one side. I use ring buffers for UART and CAN receive data, and for logging in the control loop. To tell full from empty, keep one slot free or use a count.

**中文翻译**：栈是后进先出：函数调用、撤销和深度优先搜索都用它。队列是先进先出：任务、消息和广度优先搜索。环形缓冲是数组里的定长队列，读写下标回绕；不需要分配，操作为常数时间。单生产者单消费者的环形缓冲，如果每个下标只由一方写，可以做到无锁。我用环形缓冲存放 UART 和 CAN 接收数据，也用于控制环中的日志。区分满和空可以留一个空槽，或使用计数。
**追问 1**：*What if the producer is faster than the consumer?* The buffer fills; decide a policy: drop oldest, drop newest, or flag overflow.
　中文：缓冲会满；要定策略：丢最旧、丢最新或标记溢出。
**追问 2**：*Why is a power-of-two size convenient?* The wrap-around can use a bit mask instead of modulo.
　中文：回绕可用位掩码代替取模。
**如果追问到你的项目**：*The wheel-legged control loop writes a log record every period into a ring buffer, and a low-priority task uploads it, so logging never blocks control.*
　中文：轮腿控制环每周期向环形缓冲写一条日志记录，低优先级任务负责上传，因此日志不会阻塞控制。
**关键词与易错点**：LIFO/FIFO、ring buffer、SPSC、overflow policy。

### Q83. What is a heap (priority queue), and where does A* use it? ★
**中文题意**：堆（优先队列）是什么？A* 在哪用到？
**考察点**：堆与图搜索。
**项目关联**：P5（Hybrid-A*）。
**English answer（口语）**：
> A binary heap is a complete binary tree stored in an array where each parent is smaller than its children for a min-heap. Getting the minimum is O of one, and insert and remove-minimum are O of log n, using sift-up and sift-down. Building a heap from n items takes O of n. A priority queue built on a heap is the open list in Dijkstra and A*: it always gives the node with the smallest estimated total cost next. The heap is not sorted overall and has no fast search for arbitrary elements, so decrease-key is often implemented by inserting duplicates.

**中文翻译**：二叉堆是存放在数组里的完全二叉树，最小堆中每个父节点小于其子节点。取最小值是 O(1)，插入和删除最小值是 O(log n)，通过上浮和下沉实现。从 n 个元素建堆需要 O(n)。基于堆的优先队列就是 Dijkstra 和 A* 的 open 列表：它总是先给出估计总代价最小的节点。堆整体并不有序，也没有对任意元素的快速查找，所以 decrease-key 常通过插入重复项来实现。
**追问 1**：*How is a heap stored in an array?* The children of index i are at 2i plus 1 and 2i plus 2.
　中文：下标 i 的孩子在 2i+1 和 2i+2。
**追问 2**：*Heap sort complexity?* O of n log n time, O of one extra space, not stable.
　中文：时间 O(n log n)，额外空间 O(1)，不稳定。
**如果追问到你的项目**：*In the navigation project, the global planner is Hybrid-A*, which keeps an open set ordered by cost plus heuristic, and the vehicle's heading and kinematic constraints are part of the search state.*
　中文：导航项目里全局规划是 Hybrid-A*，用按代价加启发式排序的 open 集，车辆朝向和运动学约束是搜索状态的一部分。
**关键词与易错点**：min-heap、sift-up、priority queue、open list。

### Q84. Explain binary search trees and why balancing matters.
**中文题意**：解释二叉搜索树，以及为什么要平衡。
**考察点**：树。
**项目关联**：课程。
**English answer（口语）**：
> A binary search tree keeps smaller keys in the left subtree and larger keys in the right, so search, insert and delete take time proportional to the height. If keys are inserted in sorted order, the tree degenerates into a linked list with height n, making operations O of n. Self-balancing trees, such as AVL and red-black trees, keep the height at O of log n by rotations after changes. AVL is stricter and faster for lookups; red-black does fewer rotations and is common in libraries, such as C++ `std::map`. In-order traversal visits keys in sorted order.

**中文翻译**：二叉搜索树把较小的键放左子树、较大的放右子树，所以查找、插入、删除的时间与高度成正比。若按有序顺序插入，树退化为高度为 n 的链表，操作变成 O(n)。自平衡树（如 AVL 和红黑树）在改动后通过旋转把高度保持在 O(log n)。AVL 更严格，查找更快；红黑树旋转更少，常用于库中，比如 C++ 的 `std::map`。中序遍历按排序顺序访问键。
**追问 1**：*What is the height of a balanced tree with a million nodes?* About twenty.
　中文：约 20。
**追问 2**：*How do you check if a tree is a valid BST?* In-order traversal must be strictly increasing, or check ranges recursively.
　中文：中序遍历必须严格递增，或递归检查取值范围。
**如果追问到你的项目**：*I rely on library containers for ordered maps; the algorithmic knowledge helps me choose between ordered and hashed containers.*
　中文：有序映射我依赖库容器；算法知识帮助我在有序和哈希容器之间选择。
**关键词与易错点**：BST、rotation、AVL、red-black、in-order。

### Q85. Compare common sorting algorithms. ★
**中文题意**：比较常见排序算法。
**考察点**：排序与稳定性。
**项目关联**：课程。
**English answer（口语）**：
> Quicksort picks a pivot and partitions; average O of n log n, worst case O of n squared with bad pivots, in place, not stable. Merge sort divides and merges; always O of n log n, stable, but needs O of n extra space. Heap sort is O of n log n in place but not stable and has poor cache behaviour. Insertion sort is O of n squared but fast on small or nearly sorted arrays. Library sorts combine methods, for example introsort or timsort. Stable means equal keys keep their relative order, which matters when sorting by several keys in turn. The comparison-sort lower bound is n log n.

**中文翻译**：快速排序选基准并划分；平均 O(n log n)，基准差时最坏 O(n²)，原地，不稳定。归并排序先分后合；总是 O(n log n)，稳定，但需要 O(n) 额外空间。堆排序 O(n log n)、原地，但不稳定且缓存表现差。插入排序 O(n²)，但在小规模或近乎有序的数组上很快。库中的排序会组合多种方法，例如 introsort 或 timsort。稳定指相等键保持相对顺序，按多个键依次排序时很重要。基于比较的排序下界是 n log n。
**追问 1**：*How do you avoid quicksort's worst case?* Random or median-of-three pivots, or switch to heap sort.
　中文：随机或三数取中的基准，或切换到堆排序。
**追问 2**：*When does counting sort beat n log n?* When keys are integers in a small range.
　中文：键是小范围整数时。
**如果追问到你的项目**：*I rarely sort large data on the microcontroller; on the host side the library sort is enough and I choose by stability and memory needs.*
　中文：我很少在单片机上对大数据排序；主机侧库排序足够，按稳定性与内存需求选择。
**关键词与易错点**：stable、in-place、pivot、lower bound。

### Q86. How do you write a correct binary search, and what are the common bugs?
**中文题意**：怎样写对二分查找？常见 bug 有哪些？
**考察点**：边界处理。
**项目关联**：课程、查表。
**English answer（口语）**：
> Binary search needs a sorted array, and it repeatedly halves the interval. I fix an invariant, for example the answer is in the half-open range from low to high, and update the bounds so the interval always shrinks. The middle index should be computed as low plus half of the difference, to avoid integer overflow. Common bugs are off-by-one errors that cause infinite loops, wrong handling of duplicates when I want the first or last occurrence, and forgetting that the input must be sorted. Complexity is O of log n, and I test with empty, one-element, and not-found cases.

**中文翻译**：二分查找需要有序数组，并反复把区间减半。我先确定不变量，例如答案在左闭右开区间 [low, high) 内，并更新边界使区间总在缩小。中点应写成 low 加差值的一半，避免整数溢出。常见 bug 有：差一错误导致死循环，要找第一个或最后一个出现位置时对重复元素处理错误，以及忘了输入必须有序。复杂度 O(log n)，我用空数组、单元素和找不到的情形测试。
**追问 1**：*Where else does binary search apply?* Searching a monotonic condition, such as the smallest value that meets a requirement, or finding a threshold.
　中文：搜索单调条件，如满足要求的最小值或找阈值。
**追问 2**：*How would you search a rotated sorted array?* Decide which half is sorted, then check whether the target lies in it.
　中文：判断哪一半有序，再检查目标是否在其中。
**如果追问到你的项目**：*Lookup tables for sensor calibration or gain scheduling can use binary search to find the interval and then interpolate.*
　中文：传感器标定或增益调度的查表可用二分查找找区间再插值。
**关键词与易错点**：invariant、overflow、off-by-one、duplicates。

### Q87. Explain BFS, DFS, Dijkstra and A*, and how they relate. ★
**中文题意**：解释 BFS、DFS、Dijkstra、A* 及它们的关系。
**考察点**：图算法。
**项目关联**：P5（Hybrid-A*）。
**English answer（口语）**：
> BFS explores a graph level by level using a queue and finds the shortest path in an unweighted graph. DFS goes deep first using a stack or recursion and is used for connectivity, cycle detection and topological sort. Dijkstra generalises BFS to non-negative weights using a priority queue, so it always expands the node with the smallest known cost. A* is Dijkstra guided by a heuristic: it expands by cost so far plus an estimate to the goal. If the heuristic never overestimates, it is admissible and A* finds the optimal path, usually exploring far fewer nodes.

**中文翻译**：BFS 用队列逐层探索图，在无权图里找最短路径。DFS 用栈或递归先深入，用于连通性、环检测和拓扑排序。Dijkstra 把 BFS 推广到非负权重，使用优先队列，总是扩展已知代价最小的节点。A* 是带启发式引导的 Dijkstra：按已走代价加到目标的估计来扩展。如果启发式从不高估，就是可采纳的，A* 能找到最优路径，通常探索的节点少得多。
**追问 1**：*Why does Dijkstra fail with negative edges?* A finalised node's cost could later be reduced by a negative path.
　中文：已确定的节点代价之后可能被负权路径降低。
**追问 2**：*What is the complexity of Dijkstra with a binary heap?* O of (V plus E) log V.
　中文：O((V+E) log V)。
**如果追问到你的项目**：*In the navigation project, Hybrid-A* searches over continuous vehicle states with heading and a minimum turning radius, so the result is drivable for a car-like robot, while the grid heuristic keeps the search efficient.*
　中文：导航项目里 Hybrid-A* 在含朝向和最小转弯半径的连续车辆状态上搜索，所以结果对车式机器人可行驶，栅格启发式让搜索保持高效。
**关键词与易错点**：queue、stack、priority queue、admissible heuristic。

### Q88. Explain dynamic programming with an example, such as edit distance or the knapsack problem.
**中文题意**：用编辑距离或背包问题解释动态规划。
**考察点**：DP。
**项目关联**：课程。
**English answer（口语）**：
> Dynamic programming solves problems with overlapping subproblems and optimal substructure by storing the results of subproblems. For the zero-one knapsack, let `dp[i][w]` be the best value using the first i items with capacity w. Either I skip item i, giving `dp[i-1][w]`, or I take it, giving its value plus `dp[i-1][w - weight]`, and I keep the larger. The table has n times W entries, so the time is O of nW, and I can reduce space to O of W by iterating the capacity backwards. I always state the state, the transition, the base case, and the order of computation.

**中文翻译**：动态规划通过存储子问题的结果，解决具有重叠子问题和最优子结构的问题。以 0-1 背包为例，令 `dp[i][w]` 为使用前 i 个物品、容量为 w 时的最优价值。要么不选物品 i，得到 `dp[i-1][w]`；要么选它，得到它的价值加 `dp[i-1][w - 重量]`；取较大者。表有 n 乘 W 个条目，所以时间为 O(nW)，把容量倒序遍历可将空间降到 O(W)。我总是说明状态、转移、边界条件和计算顺序。
**追问 1**：*Difference between DP and greedy?* Greedy makes a locally optimal choice without revisiting; DP considers all subproblem combinations.
　中文：贪心做局部最优选择且不回头；DP 考虑所有子问题组合。
**追问 2**：*Why is the backward capacity loop needed for 1D knapsack?* To avoid using an item twice in the same pass.
　中文：避免同一轮里重复使用同一物品。
**如果追问到你的项目**：*The same idea of optimal substructure appears in the Viterbi algorithm and in the Bellman equation behind LQR and reinforcement learning, which I met in my control studies.*
　中文：同样的最优子结构思想出现在 Viterbi 算法，以及 LQR 和强化学习背后的 Bellman 方程里，我在控制课程中接触过。
**关键词与易错点**：state、transition、base case、overlapping subproblems。

## 9. H　网络、并发、数据库与 Git（Q89–Q104）

### Q89. What are the differences between TCP and UDP? ★
**中文题意**：TCP 与 UDP 的区别？
**考察点**：传输层。
**项目关联**：P4（TCP 通信）、DDS（UDP）。
**English answer（口语）**：
> TCP is connection-oriented and gives a reliable, ordered byte stream: it retransmits lost segments, controls flow and congestion, and detects errors. It costs a handshake, extra latency under loss, and head-of-line blocking. UDP is connectionless and sends independent datagrams with no guarantee of delivery or order, but with very low overhead and latency. I use TCP when correctness matters, like commands and configuration. I use UDP, or protocols on top of it, for real-time streams where a late packet is useless, like sensor data, video, or discovery messages. DDS typically runs over UDP and adds its own reliability when requested.

**中文翻译**：TCP 面向连接，提供可靠有序的字节流：重传丢失的报文段，做流量和拥塞控制，并检测错误。代价是握手、丢包时额外延迟和队头阻塞。UDP 无连接，发送独立数据报，不保证送达和顺序，但开销和延迟很低。正确性重要时用 TCP，如命令和配置；实时流里迟到的包没用，则用 UDP 或建在其上的协议，如传感器数据、视频、发现消息。DDS 通常跑在 UDP 上，并在需要时自己加可靠性。
**追问 1**：*Does TCP preserve message boundaries?* No, it is a byte stream; the application must frame messages.
　中文：不保留消息边界，是字节流，应用必须自己分帧。
**追问 2**：*Why is UDP good for multicast?* It is connectionless, so one datagram can reach many receivers.
　中文：无连接，一个数据报可到达多个接收者。
**如果追问到你的项目**：*In the gardening system the device and server exchange structured messages over TCP with a defined protocol, because commands must not be lost or reordered.*
　中文：园艺系统里设备与服务端用定义好的协议通过 TCP 交换结构化消息，因为命令不能丢失或乱序。
**关键词与易错点**：reliable stream、datagram、head-of-line blocking、framing。

### Q90. Describe the TCP three-way handshake and the four-way close. Why TIME_WAIT? ★
**中文题意**：描述 TCP 三次握手与四次挥手。为什么有 TIME_WAIT？
**考察点**：连接管理。
**项目关联**：P4。
**English answer（口语）**：
> In the handshake the client sends a SYN with its initial sequence number, the server replies with SYN and ACK carrying its own number, and the client sends an ACK. Three steps are needed so both sides confirm that the other can send and receive and agree on initial sequence numbers. Closing is four steps because each direction closes independently: one side sends FIN, the other ACKs, later sends its own FIN, and gets an ACK. The side that closes first enters TIME_WAIT for about twice the maximum segment lifetime, so late duplicate packets die out and the last ACK can be retransmitted.

**中文翻译**：握手时客户端发送带初始序号的 SYN，服务端回复 SYN+ACK 并带上自己的序号，客户端再发 ACK。需要三步是为了双方确认对方既能发也能收，并就初始序号达成一致。关闭是四步，因为两个方向独立关闭：一方发 FIN，另一方 ACK，之后再发自己的 FIN，并收到 ACK。先关闭的一方进入 TIME_WAIT 约两倍最大报文段寿命，使迟到的重复包消失，且最后的 ACK 可以重传。
**追问 1**：*What is a SYN flood?* An attack that sends many SYNs without completing the handshake, exhausting the server's half-open connection table.
　中文：发送大量 SYN 而不完成握手，耗尽服务端半连接表。
**追问 2**：*Many sockets stuck in TIME_WAIT: what to do?* Reuse connections, enable address reuse, and avoid closing from the busy side if possible.
　中文：复用连接、启用地址复用，并尽量避免由忙碌一侧先关闭。
**如果追问到你的项目**：*For long-lived device links, I keep the connection open with application heartbeats and reconnect with backoff on failure, rather than opening a new connection per message.*
　中文：长连接的设备链路上，我用应用层心跳保持连接，失败时退避重连，而不是每条消息新建连接。
**关键词与易错点**：SYN、ACK、FIN、TIME_WAIT、2MSL。

### Q91. How does TCP provide reliability, flow control and congestion control?
**中文题意**：TCP 如何提供可靠性、流量控制和拥塞控制？
**考察点**：TCP 机制。
**项目关联**：网络课程。
**English answer（口语）**：
> Reliability comes from sequence numbers, acknowledgements, checksums and retransmission on timeout or after duplicate acknowledgements. Flow control protects the receiver: it advertises a window of free buffer space, and the sender never has more unacknowledged data than that. Congestion control protects the network: the sender keeps a congestion window that starts small and grows exponentially in slow start, then linearly in congestion avoidance; on loss it cuts the window, multiplicatively. This additive-increase, multiplicative-decrease behaviour shares bandwidth fairly. Modern variants such as CUBIC and BBR change the growth rules.

**中文翻译**：可靠性来自序号、确认、校验和，以及超时或重复确认后的重传。流量控制保护接收方：它通告空闲缓冲区的窗口，发送方未确认的数据不会超过该窗口。拥塞控制保护网络：发送方维护拥塞窗口，慢启动阶段从小开始指数增长，拥塞避免阶段线性增长；丢包时按倍数减小窗口。这种加性增、乘性减的行为使带宽公平共享。CUBIC、BBR 等现代变体改变了增长规则。
**追问 1**：*Why is TCP slow on lossy Wi-Fi?* It interprets loss as congestion and shrinks the window even when the loss is due to radio errors.
　中文：它把丢包当作拥塞而缩小窗口，即使丢包是无线错误造成的。
**追问 2**：*What is fast retransmit?* Retransmitting after three duplicate ACKs without waiting for the timeout.
　中文：收到三个重复 ACK 后不等超时就重传。
**如果追问到你的项目**：*For commands over a wireless link, I add application-level timeouts and idempotent messages, so a delayed or repeated command is safe.*
　中文：无线链路上的命令，我加应用层超时和幂等消息，使延迟或重复的命令也安全。
**关键词与易错点**：window、slow start、AIMD、fast retransmit。

### Q92. What happens when you type a URL into a browser?
**中文题意**：在浏览器输入网址后发生了什么？
**考察点**：端到端网络理解。
**项目关联**：P4（Web 交互）。
**English answer（口语）**：
> The browser parses the URL and checks its caches. It resolves the host name to an IP address through DNS, from local caches or recursive resolvers. It opens a TCP connection to the server, and for HTTPS performs a TLS handshake to agree on keys and verify the server's certificate. It sends an HTTP request; the server may pass it through load balancers and application code, and returns a response. The browser parses the HTML, fetches CSS, scripts and images, which may reuse connections, then builds the page. Each layer adds latency, so caching and connection reuse matter.

**中文翻译**：浏览器解析网址并检查缓存。通过 DNS（本地缓存或递归解析器）把主机名解析为 IP 地址。与服务器建立 TCP 连接，若是 HTTPS，则做 TLS 握手以协商密钥并验证服务器证书。发送 HTTP 请求；服务器可能经过负载均衡和应用代码，然后返回响应。浏览器解析 HTML，获取 CSS、脚本和图片（可复用连接），再渲染页面。每一层都会增加延迟，所以缓存和连接复用很重要。
**追问 1**：*What does TLS provide?* Encryption, integrity, and authentication of the server.
　中文：加密、完整性，以及对服务器的认证。
**追问 2**：*What is the difference between HTTP/1.1 and HTTP/2?* HTTP/2 multiplexes requests over one connection with header compression.
　中文：HTTP/2 在一条连接上多路复用请求并压缩头部。
**如果追问到你的项目**：*The gardening system's web interface talks to the server over HTTP and WebSocket, so the user can see device state and send commands.*
　中文：园艺系统的 Web 界面通过 HTTP 和 WebSocket 与服务端通信，用户可查看设备状态并下发命令。
**关键词与易错点**：DNS、TCP、TLS、HTTP。

### Q93. TCP is a byte stream. How do you frame messages, and what is Nagle's algorithm? ★
**中文题意**：TCP 是字节流，怎样给消息分帧？Nagle 算法是什么？
**考察点**：应用层协议设计。
**项目关联**：P4。
**English answer（口语）**：
> Because TCP does not preserve message boundaries, one send may arrive in several reads, and several sends may arrive together. I frame messages with a length prefix, a delimiter, or a fixed header with a length field, and the receiver accumulates bytes in a buffer until a whole message is present, then validates a checksum or version. Nagle's algorithm delays small writes to combine them into larger segments, which saves bandwidth but adds latency. For interactive or control messages I disable it with TCP_NODELAY. I also set timeouts and keepalives to detect dead peers.

**中文翻译**：因为 TCP 不保留消息边界，一次发送可能分几次读到，几次发送也可能一起到达。我用长度前缀、分隔符或带长度字段的固定头给消息分帧，接收端在缓冲里累积字节直到整条消息到齐，再验证校验和或版本。Nagle 算法会延迟小写入以合并成较大的报文段，节省带宽但增加延迟。对交互式或控制消息，我用 TCP_NODELAY 关闭它。我还设置超时和 keepalive 来发现已死的对端。
**追问 1**：*What is a half-open connection?* One side has crashed or lost the network without closing, and the other still thinks the connection is alive; heartbeats detect it.
　中文：一端崩溃或断网而未关闭，另一端仍以为连接存在；心跳可发现。
**追问 2**：*How do you protect against a malformed length field?* Cap the maximum length and drop the connection if it is exceeded.
　中文：限制最大长度，超出就断开连接。
**如果追问到你的项目**：*In the gardening project, I designed a protocol for device observations and commands with structured messages and checks, so the server can trace each state change.*
　中文：园艺项目里，我为设备观测和命令设计了带结构化消息和校验的协议，使服务端能追踪每次状态变化。
**关键词与易错点**：length prefix、TCP_NODELAY、keepalive、buffering。

### Q94. When would you choose HTTP, WebSocket or MQTT?
**中文题意**：什么时候选 HTTP、WebSocket 或 MQTT？
**考察点**：应用层协议选型。
**项目关联**：P4。
**English answer（口语）**：
> HTTP is request-response, simple and cacheable, good for configuration, queries and file download. WebSocket upgrades an HTTP connection into a persistent, full-duplex channel, so the server can push updates immediately; it fits live dashboards and bidirectional control from a browser. MQTT is a lightweight publish-subscribe protocol with a broker, designed for constrained devices and unreliable networks, with quality-of-service levels and retained messages; it fits many small IoT devices. My choice depends on who connects, how often data flows, and whether I need server push or a broker.

**中文翻译**：HTTP 是请求应答，简单且可缓存，适合配置、查询和文件下载。WebSocket 把 HTTP 连接升级为持久的全双工通道，服务端可立即推送更新；适合实时仪表盘和浏览器的双向控制。MQTT 是带代理的轻量发布订阅协议，为资源受限设备和不可靠网络设计，有服务质量等级和保留消息；适合大量小型 IoT 设备。我的选择取决于谁来连接、数据多久流动一次，以及是否需要服务端推送或代理。
**追问 1**：*Why not poll with HTTP for live updates?* It wastes requests and adds latency; WebSocket pushes only when something changes.
　中文：浪费请求且延迟大；WebSocket 仅在有变化时推送。
**追问 2**：*What do MQTT QoS levels mean?* At most once, at least once, and exactly once delivery.
　中文：至多一次、至少一次、恰好一次。
**如果追问到你的项目**：*The gardening server uses WebSocket for live web interaction, while device links use a TCP protocol; the control commands pass checks before reaching devices.*
　中文：园艺服务端用 WebSocket 做实时 Web 交互，设备链路用 TCP 协议；控制命令在到达设备前要通过检查。
**关键词与易错点**：request-response、full duplex、broker、QoS。

### Q95. What is a race condition, and what are atomics and memory ordering? ★
**中文题意**：什么是竞争条件？原子操作和内存序是什么？
**考察点**：并发正确性。
**项目关联**：P3、P7（ISR/任务共享）、ROS2 回调。
**English answer（口语）**：
> A race condition occurs when the result depends on the unpredictable timing of concurrent accesses to shared data, with at least one write. For example, two threads incrementing a counter can lose an update, because increment is read, add, write. An atomic operation executes as one indivisible step. Memory ordering defines when the effects of one thread's writes become visible to another, because compilers and CPUs reorder operations. In C++ I use `std::atomic` with the default sequentially consistent ordering unless I can justify a weaker one. The simplest safe approach is to avoid sharing, or to protect shared data with a mutex.

**中文翻译**：竞争条件是指结果取决于对共享数据的并发访问的不可预测时序，且至少有一个写。例如两个线程对计数器自增会丢失一次更新，因为自增是读、加、写。原子操作作为一个不可分割的步骤执行。内存序定义一个线程的写入何时对另一个线程可见，因为编译器和 CPU 会重排操作。C++ 里我用 `std::atomic` 的默认顺序一致性，除非能证明较弱的顺序是安全的。最简单安全的做法是避免共享，或用互斥量保护共享数据。
**追问 1**：*Is `volatile` a substitute for atomics in C++?* No; it prevents compiler optimisation of accesses but gives no atomicity or ordering between threads.
　中文：不是；它阻止编译器优化访问，但不提供线程间的原子性和顺序。
**追问 2**：*What tools find races?* ThreadSanitizer, careful code review, and stress tests.
　中文：ThreadSanitizer、仔细的代码审查和压力测试。
**如果追问到你的项目**：*On the STM32 I handle the interrupt-to-task case with snapshots and short critical sections; on Linux ROS2 nodes I protect state shared between callbacks with mutexes.*
　中文：STM32 上我用快照和短临界区处理中断到任务的情形；Linux ROS2 节点里用互斥量保护回调间共享的状态。
**关键词与易错点**：data race、atomic、memory order、ThreadSanitizer。

### Q96. When would you use a lock-free design instead of a mutex?
**中文题意**：什么时候用无锁设计而不是互斥量？
**考察点**：并发设计权衡。
**项目关联**：实时控制。
**English answer（口语）**：
> I use lock-free structures when blocking is unacceptable, such as in an interrupt handler or a hard real-time control loop, or when contention is high enough that locks become a bottleneck. Examples are a single-producer single-consumer ring buffer, a double buffer with an atomic index, and a sequence lock. The cost is complexity: lock-free algorithms are hard to get right and to test, and problems like the ABA issue appear with compare-and-swap. If the critical section is short and contention low, a mutex is simpler and often fast enough. I prefer the simplest design that meets the timing requirement.

**中文翻译**：当阻塞不可接受时用无锁结构，比如在中断处理函数或硬实时控制环里，或者竞争大到锁成为瓶颈。例子有单生产者单消费者环形缓冲、带原子索引的双缓冲和序列锁。代价是复杂：无锁算法很难写对、测试，使用比较并交换时还会出现 ABA 问题。如果临界区短、竞争低，互斥量更简单，通常也够快。我偏好满足时序要求的最简单设计。
**追问 1**：*What is the ABA problem?* A value changes from A to B and back to A, so a compare-and-swap wrongly sees no change.
　中文：值从 A 变 B 再变回 A，比较并交换误以为没有变化。
**追问 2**：*Is lock-free the same as wait-free?* No; lock-free guarantees system-wide progress, wait-free guarantees each thread finishes in bounded steps.
　中文：不同；无锁保证系统整体有进展，无等待保证每个线程在有限步内完成。
**如果追问到你的项目**：*The control task reads a snapshot, so it never waits for the IMU or CAN interrupt; this keeps the control timing predictable.*
　中文：控制任务读快照，所以从不等待 IMU 或 CAN 中断；这让控制时序可预测。
**关键词与易错点**：SPSC、double buffer、ABA、wait-free。

### Q97. What is the Python GIL, and how do threads, processes and asyncio compare? ★
**中文题意**：Python 的 GIL 是什么？线程、进程、asyncio 怎么比较？
**考察点**：Python 并发。
**项目关联**：P4（asyncio 服务端）、P2（Python 节点）。
**English answer（口语）**：
> In CPython the global interpreter lock lets only one thread execute Python bytecode at a time, so CPU-bound code gets no speed-up from threads; threads still help with I/O-bound work because the lock is released while waiting. For CPU-bound work, I use multiple processes or native extensions such as NumPy, which release the lock. `asyncio` uses a single-threaded event loop with coroutines; it handles many connections with low overhead, provided that nothing blocks the loop. I choose asyncio for networked I/O, processes for heavy computation, and threads for blocking libraries.

**中文翻译**：CPython 中全局解释器锁使一次只有一个线程执行 Python 字节码，所以 CPU 密集代码用线程得不到加速；I/O 密集的工作线程仍有帮助，因为等待时会释放锁。CPU 密集任务我用多进程或释放锁的原生扩展（如 NumPy）。`asyncio` 用单线程事件循环和协程；只要没有东西阻塞事件循环，就能以很低的开销处理大量连接。网络 I/O 我选 asyncio，重计算用进程，阻塞式库用线程。
**追问 1**：*How do you call a blocking function from a coroutine?* Run it in an executor thread so the loop stays free.
　中文：放到执行器线程里运行，使事件循环保持空闲。
**追问 2**：*Does the GIL make code thread-safe?* Single bytecodes are atomic, but compound operations are not.
　中文：单个字节码是原子的，但复合操作不是。
**如果追问到你的项目**：*The gardening server is asyncio-based with WebSocket and device links, and the heavy model inference runs on the RK3588 edge device, not in the server loop.*
　中文：园艺服务端基于 asyncio，带 WebSocket 与设备链路，而重的模型推理在 RK3588 边缘设备上，不在服务端事件循环里。
**关键词与易错点**：GIL、event loop、executor、CPU-bound vs I/O-bound。

### Q98. Explain the producer-consumer problem and how you solve it.
**中文题意**：解释生产者-消费者问题及解法。
**考察点**：同步经典问题。
**项目关联**：ISR 到任务、日志。
**English answer（口语）**：
> Producers put items into a shared bounded buffer and consumers take them out. The problems are mutual exclusion on the buffer, blocking producers when it is full, and blocking consumers when it is empty. With semaphores I use one for empty slots, one for filled slots, and a mutex around the buffer operations. With a mutex and condition variables, producers wait while the buffer is full and consumers wait while it is empty, each rechecking in a loop. In RTOS code a queue does this for me. The design question is the overflow policy, such as blocking, dropping oldest, or dropping newest.

**中文翻译**：生产者向共享的有界缓冲放入元素，消费者取出。问题包括对缓冲的互斥、缓冲满时阻塞生产者、缓冲空时阻塞消费者。用信号量时，一个表示空槽、一个表示已填槽，再加一个保护缓冲操作的互斥量。用互斥量和条件变量时，缓冲满则生产者等待，缓冲空则消费者等待，各自在循环中重新检查。RTOS 代码里队列已替我做了这些。设计上要考虑的是溢出策略，如阻塞、丢最旧或丢最新。
**追问 1**：*What happens with one slow consumer?* The buffer fills and the policy applies; monitor the queue depth.
　中文：缓冲填满，按策略处理；监控队列深度。
**追问 2**：*How would you handle multiple consumers?* Use a thread-safe queue so each item goes to exactly one consumer.
　中文：用线程安全队列，使每个元素只给一个消费者。
**如果追问到你的项目**：*The CAN receive interrupt is the producer and the control task the consumer; I keep the interrupt side non-blocking and count overruns.*
　中文：CAN 接收中断是生产者，控制任务是消费者；我让中断侧不阻塞，并统计溢出。
**关键词与易错点**：bounded buffer、semaphore、condition variable、overflow policy。

### Q99. What are ACID properties and transaction isolation levels?
**中文题意**：ACID 是什么？事务隔离级别有哪些？
**考察点**：数据库基础。
**项目关联**：P4（状态存储）。
**English answer（口语）**：
> ACID stands for atomicity, consistency, isolation and durability. Atomicity means a transaction either fully commits or has no effect. Consistency means it moves the database between valid states respecting constraints. Isolation means concurrent transactions do not see each other's partial work as if they ran one after another. Durability means committed data survives a crash, usually through a write-ahead log. The SQL isolation levels are read uncommitted, read committed, repeatable read and serialisable; stronger levels prevent anomalies such as dirty reads, non-repeatable reads and phantoms, at the cost of concurrency.

**中文翻译**：ACID 即原子性、一致性、隔离性、持久性。原子性指事务要么完全提交要么没有效果。一致性指事务使数据库在满足约束的有效状态间转换。隔离性指并发事务彼此看不到对方的中间结果，如同依次执行。持久性指已提交的数据在崩溃后依然存在，通常靠预写日志。SQL 隔离级别有读未提交、读已提交、可重复读和可串行化；更强的级别防止脏读、不可重复读、幻读等异常，代价是并发度降低。
**追问 1**：*What is a dirty read?* Reading data that another transaction has changed but not yet committed.
　中文：读到另一个事务已修改但未提交的数据。
**追问 2**：*How does a write-ahead log give durability?* The change is written to the log on disk before the commit is acknowledged, so it can be replayed after a crash.
　中文：确认提交前先把变更写入磁盘日志，崩溃后可重放。
**如果追问到你的项目**：*For the traceable system state in the gardening project, I think of each state change as a record with a time and a cause, so history can be replayed.*
　中文：园艺项目里可追溯的系统状态，我把每次状态变化看作带时间和原因的记录，历史可重放。
**关键词与易错点**：ACID、WAL、isolation level、anomalies。

### Q100. How does a database index work, and why do databases use B+ trees? ★
**中文题意**：数据库索引怎么工作？为什么用 B+ 树？
**考察点**：索引。
**项目关联**：数据库课程。
**English answer（口语）**：
> An index is an extra data structure that lets the database find rows without scanning the whole table. Most databases use B+ trees: a balanced tree with a high fan-out, so a lookup in a table of millions of rows needs only three or four page reads. All values live in the leaves, which are linked in order, so range queries and sorted scans are efficient. Compared with binary trees, B+ trees match disk page sizes and reduce I/O. The costs are extra storage and slower writes, because each insert must update the indexes, so I index the columns used in filters and joins, not everything.

**中文翻译**：索引是额外的数据结构，使数据库不必扫描整张表就能找到行。多数数据库用 B+ 树：高扇出的平衡树，所以在百万行的表里查找只需三四次页读取。所有值存放在按顺序链接的叶子节点，因此范围查询和有序扫描很高效。与二叉树相比，B+ 树匹配磁盘页大小，减少 I/O。代价是额外存储和更慢的写入，因为每次插入都要更新索引，所以我只给过滤和连接用到的列建索引，而不是全部。
**追问 1**：*What is a composite index and the leftmost prefix rule?* An index on several columns is usable for queries that filter on a leading subset of those columns.
　中文：多列索引可用于过滤其前导列子集的查询。
**追问 2**：*When is a full table scan better?* When the query returns a large fraction of the rows or the table is small.
　中文：查询返回大部分行，或表很小。
**如果追问到你的项目**：*I have not run a large database in my robot projects; the idea helps me design logs and state stores that are queried by time and device ID.*
　中文：机器人项目里我没有运行过大型数据库；这个思想帮助我设计按时间和设备 ID 查询的日志和状态存储。
**关键词与易错点**：B+ tree、fan-out、range query、index cost。不要把没用过的数据库经验说成项目经历。

### Q101. How would you store robot telemetry or state: SQL, key-value or time-series?
**中文题意**：机器人遥测或状态怎么存：SQL、键值还是时序库？
**考察点**：存储选型。
**项目关联**：P4、P6。
**English answer（口语）**：
> I look at the access pattern. For the current state per device, a key-value store keyed by device ID is simple and fast. For configuration and relationships, such as devices, users and permissions, a relational database gives constraints and joins. For high-rate numeric history, such as sensor streams, a time-series database or append-only files such as rosbags are better, because writes are sequential and queries are by time range. I would also keep an event log with timestamps and causes to make decisions traceable. For on-device storage, I choose the simplest option that survives power loss.

**中文翻译**：我看访问模式。对每个设备的当前状态，以设备 ID 为键的键值存储简单又快。对配置和关系（设备、用户、权限），关系型数据库提供约束和连接。对高频数值历史（如传感器流），时序数据库或追加写文件（如 rosbag）更合适，因为写入是顺序的，查询按时间范围。我还会保留带时间戳和原因的事件日志，使决策可追溯。设备端存储则选能经受掉电的最简单方案。
**追问 1**：*Why is a rosbag good for debugging?* You can replay recorded data into the stack to reproduce a problem.
　中文：可以把录制的数据回放进系统来复现问题。
**追问 2**：*How do you avoid unbounded log growth?* Rotation and retention policies, with sampling for high-rate data.
　中文：日志轮转和保留策略，高频数据做降采样。
**如果追问到你的项目**：*In the gardening system I unified visual observations, environment state and user commands into a traceable system state, with timestamps and source for each item.*
　中文：园艺系统里我把视觉观测、环境状态和用户命令统一成可追溯的系统状态，每项都带时间戳和来源。
**关键词与易错点**：access pattern、time-series、event log、retention。

### Q102. What is the difference between `git merge` and `git rebase`? ★
**中文题意**：`git merge` 与 `git rebase` 的区别？
**考察点**：Git 工作流。
**项目关联**：技能栏 Git。
**English answer（口语）**：
> Merge combines two branches by creating a merge commit with two parents, preserving the true history and its branching. Rebase replays my commits on top of another branch, creating new commits and giving a linear history. Merge is safe for shared branches because it does not rewrite history. Rebase makes history cleaner, and I use it on my own local feature branch to update from main before opening a pull request. The golden rule is never to rebase commits that others have already pulled, because rewriting shared history causes conflicts and lost work.

**中文翻译**：merge 通过创建带两个父提交的合并提交来合并两个分支，保留真实历史及其分叉。rebase 把我的提交重放到另一个分支之上，生成新的提交，得到线性历史。merge 对共享分支是安全的，因为不改写历史。rebase 让历史更干净，我在自己本地的特性分支上用它从 main 更新，再发起 pull request。黄金法则是不要 rebase 别人已经拉取过的提交，因为改写共享历史会造成冲突和工作丢失。
**追问 1**：*What does `git pull --rebase` do?* Fetches and replays local commits on top of the remote branch instead of creating a merge commit.
　中文：拉取后把本地提交重放到远程分支之上，而不是产生合并提交。
**追问 2**：*How do you undo a rebase mistake?* Use `git reflog` to find the previous HEAD and reset to it.
　中文：用 `git reflog` 找到之前的 HEAD 并重置。
**如果追问到你的项目**：*I use branches per feature or experiment, commit in small steps with clear messages, and keep the main branch buildable.*
　中文：我按特性或实验建分支，小步提交并写清楚提交信息，保持主分支可构建。
**关键词与易错点**：merge commit、linear history、reflog、shared history。

### Q103. How do you resolve a merge conflict, and what do `reset`, `revert` and `bisect` do?
**中文题意**：怎样解决合并冲突？`reset`、`revert`、`bisect` 各做什么？
**考察点**：Git 实操。
**项目关联**：协作开发。
**English answer（口语）**：
> A conflict happens when two branches change the same lines. Git marks the conflicting sections in the file; I open them, decide the correct result by understanding both changes, remove the markers, build and test, then stage the file and complete the merge or rebase. `git reset` moves the branch pointer, and with `--hard` also discards working changes, so it rewrites local history. `git revert` creates a new commit that undoes an earlier one, safe on shared branches. `git bisect` does a binary search through the commit history to find the commit that introduced a bug, given a good and a bad commit.

**中文翻译**：当两个分支修改了同一行时发生冲突。Git 在文件中标出冲突部分；我打开它们，理解双方改动后决定正确结果，去掉标记，构建并测试，然后暂存文件并完成合并或变基。`git reset` 移动分支指针，加 `--hard` 还会丢弃工作区改动，所以会改写本地历史。`git revert` 创建一个新提交来撤销早先的提交，在共享分支上是安全的。`git bisect` 在提交历史中做二分查找，给定一个好提交和一个坏提交，找出引入 bug 的提交。
**追问 1**：*How do you reduce merge conflicts?* Small frequent merges, clear module ownership, and not reformatting whole files.
　中文：小而频繁地合并、清晰的模块归属，避免重排整个文件的格式。
**追问 2**：*What does `git stash` do?* Temporarily shelves uncommitted changes so you can switch branches.
　中文：暂存未提交的改动，以便切换分支。
**如果追问到你的项目**：*When a regression appeared between two robot tests, I would bisect the history, using a reproducible test or replayed data as the good or bad check.*
　中文：两次机器人测试之间出现回归时，我会用可复现的测试或回放数据作为好坏判据，对历史做 bisect。
**关键词与易错点**：conflict markers、reset --hard、revert、bisect。

### Q104. How do you keep code quality high in a robotics project: testing, review and CI? ★
**中文题意**：机器人项目里怎样保证代码质量：测试、评审、CI？
**考察点**：软件工程实践。
**项目关联**：P5、P6、P7。
**English answer（口语）**：
> I use layers. Unit tests check algorithms and parsers on the host, including edge cases and fault injection such as dropped or corrupted frames. Simulation tests run the controller against a model before touching hardware. Replay tests feed recorded data, such as rosbags, into the software to catch regressions in the estimator or planner. Hardware-in-the-loop tests and a safety checklist come before running on the real robot. Code review catches design issues and unclear assumptions, and continuous integration builds and runs the tests on every change. Logging key signals, with version and parameter records, makes field problems reproducible.

**中文翻译**：我分层进行。单元测试在主机上检查算法和解析器，包括边界情形和故障注入（如丢帧或损坏帧）。仿真测试在接触硬件之前让控制器跑模型。回放测试把录制的数据（如 rosbag）喂给软件，抓估计器或规划器的回归。硬件在环测试和安全检查清单先于真机运行。代码评审发现设计问题和不清晰的假设，持续集成对每次改动构建并跑测试。记录关键信号，并附上版本和参数记录，使现场问题可复现。
**追问 1**：*What is a regression test?* A test that re-checks previously working behaviour after a change.
　中文：改动后重新检查原本正常行为的测试。
**追问 2**：*How do you test safety logic?* Inject faults, such as stale data, a lost link, or an out-of-range command, and verify that the system enters the safe state.
　中文：注入故障（过期数据、链路丢失、越界命令），验证系统进入安全状态。
**如果追问到你的项目**：*For the safe-execution chain at the platform company, the tests that matter inject unsafe proposals and check that they are rejected at the policy check and that state feedback closes the loop; in the control projects, I toggled compensation under fixed trajectories to verify its effect.*
　中文：平台公司的安全执行链，关键的测试是注入不安全的提案，检查它们会在策略检查处被拒绝、状态反馈能闭合回路；在控制项目里，我在固定轨迹下开关补偿来验证其效果。
**关键词与易错点**：unit test、replay、fault injection、CI、review。

---

## 10. 自测与练习安排

### 10.1 十个自测题（不看答案，用英文各讲 60–90 秒）

1. `volatile` 解决什么、不解决什么？中断与任务共享一个 32 位计数器和一个 3 个字的结构体，各自怎么保护？（Q1、Q42、Q95）
2. 画出 FreeRTOS 里"CAN 接收中断 → 控制任务"的数据路径，说出你会用队列、通知还是快照，为什么？（Q37、Q42）
3. 一个 2 ms 周期的控制任务，实测周期有 1 ms 抖动，你怎样定位并改进？（Q32、Q40）
4. 1 Mbps CAN 上 5 个电机 1 kHz 反馈加 667 帧/秒的命令，总线负载多少？（Q48：约 63%）
5. 经典 CAN 与 CAN FD 的区别；你的轮腿项目用哪一种？（Q49）
6. 解释优先级反转，并说明为什么二值信号量不能代替互斥量。（Q33、Q34）
7. 说出 DMA 与 Cache 一致性的问题和两种解法。（Q21）
8. ROS2 的 QoS 不匹配时现象是什么？怎么查？（Q59）
9. TCP 三次握手为什么是三次？TIME_WAIT 为什么存在？（Q90）
10. merge 和 rebase 的区别与使用场景？（Q102）

### 10.2 三轮练习安排

| 轮次 | 目标 | 做法 | 用时 |
|---|---|---|---|
| 第一轮：读懂 | 理解 104 题的结构和术语 | 每天 12 题；先读中文翻译，理解为什么这样回答；把项目数字抄进一页小抄 | 约 9 天 |
| 第二轮：开口 | 60–90 秒讲完一个问题 | 遮住英文，只看中文题意，计时说一遍并录音；重点练带 ★ 的题 | 约 8 天 |
| 第三轮：追问 | 接两层追问并转到项目 | 随机抽题，对方问 "Why?" "What number?" "Have you done this yourself?"；数字忘记时用诚实模板 | 约 6 天 |

### 10.3 优先级提醒

- **最高频 25 题**：Q1、Q3、Q4、Q8、Q11、Q14、Q17、Q20、Q21、Q25、Q26、Q27、Q29、Q31、Q32、Q33、Q34、Q43、Q44、Q48、Q49、Q57、Q69、Q71、Q89。
- **与简历直接对应、最容易被深挖的 10 题**：Q21（H7 的 Cache 与 DMA）、Q32（delay-until 与周期）、Q41（任务划分）、Q47（CAN 错误处理）、Q48（总线负载）、Q49（FDCAN 经典模式）、Q56（bxCAN 邮箱）、Q60（DDS 与 M-DDS）、Q62（时间戳与 TF）、Q65（SocketCAN）。
- **CS 方向（CityU CS、CUHK-Shenzhen CS 等）重点**：Part F 与 Part G 全部，加 Q89–Q91、Q95、Q97、Q100、Q102；算法题务必练习"先口述思路和复杂度，再写代码，再用小例子走一遍"。
- **红线再提醒**：P3 控制任务**约 3 ms，不说 1 kHz**；P7 是**经典 CAN，不是 CAN FD**；CV 没有写你写过 Linux 内核驱动、配置过 PREEMPT_RT、实现过 FOC、做过 OTA 或写过链接脚本——这些问题按"理解原理、没有亲手做"诚实回答；读码发现的隐患说 "I noticed"，不是自己的贡献；3V3 二等奖是团队成绩。
- 数字被追问到精确值时用诚实模板："I don't remember the exact number, but it was roughly …"，然后讲清方法和趋势。

### 10.4 相关链接

- 嵌入式与通信教材：[11 模块 README](../../02_基础知识/11_嵌入式与通信/README.md)　[STM32、中断、DMA](../../02_基础知识/11_嵌入式与通信/01_STM32_时钟_GPIO_中断_DMA.md)　[FreeRTOS](../../02_基础知识/11_嵌入式与通信/02_FreeRTOS实时系统.md)　[CAN/FDCAN](../../02_基础知识/11_嵌入式与通信/03_CAN_FDCAN与电机通信.md)　[UART/SPI/I2C/TCP](../../02_基础知识/11_嵌入式与通信/04_UART_SPI_I2C_以太网_TCP.md)　[电机与编码器](../../02_基础知识/11_嵌入式与通信/05_电机_编码器_驱动与力矩控制.md)　[Cortex-M 体系](../../02_基础知识/11_嵌入式与通信/07_Cortex-M体系_启动_链接脚本_内存映射.md)　[看门狗与 OTA](../../02_基础知识/11_嵌入式与通信/08_电源管理_低功耗_看门狗_Bootloader_OTA.md)　[FOC 原理](../../02_基础知识/11_嵌入式与通信/09_ADC_PWM_定时器_FOC电机控制原理.md)　[嵌入式调试](../../02_基础知识/11_嵌入式与通信/10_嵌入式调试_JTAG_SWD_逻辑分析仪_故障定位.md)　[实时调度](../../02_基础知识/11_嵌入式与通信/11_实时调度理论_RMS_EDF_优先级反转.md)　[嵌入式 Linux](../../02_基础知识/11_嵌入式与通信/12_嵌入式Linux_设备树_驱动_RT-Preempt.md)
- 机器人软件教材：[Linux 与 Git](../../02_基础知识/12_机器人软件/01_Linux与Shell_Git.md)　[C++ 现代特性](../../02_基础知识/12_机器人软件/02_C++现代特性与工程实践.md)　[Python 工程](../../02_基础知识/12_机器人软件/03_Python工程与科学计算.md)　[ROS2 Humble](../../02_基础知识/12_机器人软件/04_ROS2_Humble核心概念与ROS1对比.md)　[DDS](../../02_基础知识/12_机器人软件/05_DDS与分布式通信.md)　[URDF 与 TF](../../02_基础知识/12_机器人软件/07_URDF_TF_坐标系实践.md)
- 八股速查：[数据结构与算法](../../03_八股知识/01_计算机基础/01_数据结构与算法.md)　[操作系统](../../03_八股知识/01_计算机基础/02_操作系统.md)　[计算机网络](../../03_八股知识/01_计算机基础/03_计算机网络.md)　[C 语言与嵌入式](../../03_八股知识/01_计算机基础/04_C语言与嵌入式八股.md)　[C++](../../03_八股知识/01_计算机基础/05_C++八股.md)　[Python](../../03_八股知识/01_计算机基础/06_Python八股.md)　[并发与实时](../../03_八股知识/01_计算机基础/07_并发多线程与实时系统八股.md)　[Linux、Git、构建](../../03_八股知识/01_计算机基础/08_Linux_Git_编译构建八股.md)　[数据库与系统设计](../../03_八股知识/01_计算机基础/09_数据库与系统设计基础.md)　[嵌入式与机电](../../03_八股知识/03_机器人学/06_嵌入式与机电八股.md)　[ROS 与机器人软件](../../03_八股知识/03_机器人学/05_ROS与机器人软件八股.md)
- 项目文档：[P3 RoboMaster 英雄](../../01_项目课程/P3_RoboMaster英雄机器人嵌入式控制/README.md)　[P7 可变腿长轮腿](../../01_项目课程/P7_可变腿长轮腿机器人/README.md)　[P2 双臂遥操作与差速底盘](../../01_项目课程/P2_未来动力双臂遥操作与差速底盘/README.md)　[P1 球形机器人平衡](../../01_项目课程/P1_AIRS球形机器人平衡控制/README.md)　[P4 智能园艺](../../01_项目课程/P4_OpenHarmony智能园艺系统/README.md)　[P5 车式机器人导航](../../01_项目课程/P5_车式移动机器人自主导航/README.md)　[P6 M-Robots 平台](../../01_项目课程/P6_深开鸿M-Robots平台软件/README.md)
- 面试准备：[港硕面试真题调研](../00_港硕面试真题调研.md)　[高频100问](../04_模拟与冲刺/01_高频100问_英文速答.md)　[跨项目技术对比问答](../03_项目口述总览/02_跨项目技术对比问答.md)　[CV 事实基线](../../06_维护说明/CV事实基线.md)
- 同一分类题库：[10 数学、优化与概率](./10_题库_数学_优化_概率.md)　[11 控制理论与机器人控制](./11_题库_控制理论与机器人控制.md)　[12 运动学、动力学、状态估计与 SLAM](./12_题库_运动学_动力学_状态估计_SLAM.md)　[13 规划、导航与感知](./13_题库_规划_导航_感知.md)　[14 机器学习、深度学习、强化学习与大模型](./14_题库_机器学习_深度学习_强化学习_大模型.md)
